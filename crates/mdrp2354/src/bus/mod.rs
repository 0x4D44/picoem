use std::collections::HashMap;

use crate::memory::{Memory, SRAM_SIZE};

/// Bus fabric — address decode and cycle accounting.
///
/// Phase 1: flat memory, single-cycle access everywhere.
/// Phase 2 adds AHB5 arbitration, APB bridge latency, bus contention.
pub struct Bus {
    pub memory: Memory,
    /// Total cycles of the most recent bus access (for testing/debug).
    last_access_cycles: u32,
    /// Accumulated extra wait states beyond 1-cycle baseline during current instruction.
    /// Reset by decode_execute before dispatch, added to cycle count after.
    extra_wait_states: u32,
    /// Stub backing store for peripheral registers (APB + AHB).
    /// Keyed by canonical address (alias bits stripped).
    /// TODO: Replace with direct Peripheral trait dispatch when real peripherals are added.
    peripheral_regs: HashMap<u32, u32>,
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: Memory::new(),
            last_access_cycles: 0,
            extra_wait_states: 0,
            peripheral_regs: HashMap::new(),
        }
    }

    // --- Bus arbitration ---

    /// Determine the downstream port ID for an address.
    /// Two addresses that return the same port ID will contend.
    /// Returns None for core-local ports (SIO, PPB) that never contend.
    pub fn downstream_port(addr: u32) -> Option<u8> {
        match addr >> 28 {
            0x0 => Some(0),  // ROM — single port
            0x1 => Some(1),  // XIP — single port
            0x2 => {
                // SRAM — per-bank ports
                match Memory::bank_for_address(addr) {
                    Some(bank) => Some(2 + bank), // ports 2-11
                    None => Some(2),              // out-of-range SRAM, treat as bank 0
                }
            }
            0x4 => Some(12), // APB bridge — single port
            0x5 => Some(13), // AHB peripherals — single port
            0xD => None,     // SIO — core-local, no contention
            0xE => None,     // PPB — core-local, no contention
            _ => Some(14),   // unmapped — treat as single port
        }
    }

    /// Check if a single core's access has any stall from contention.
    /// With only one core accessing, there's never contention.
    pub fn arbitrate_stall(&self, _core: u8, _addr: u32) -> u32 {
        0 // single core never stalls
    }

    /// Given two simultaneous accesses (core 0 and core 1), determine stall
    /// cycles for each. Core 0 has higher priority (wins ties).
    /// Returns (core0_stall, core1_stall).
    pub fn arbitrate_pair(&self, core0_addr: u32, core1_addr: u32) -> (u32, u32) {
        let port0 = Self::downstream_port(core0_addr);
        let port1 = Self::downstream_port(core1_addr);

        match (port0, port1) {
            (Some(p0), Some(p1)) if p0 == p1 => {
                // Same downstream port — core 1 stalls (core 0 wins)
                (0, 1)
            }
            _ => {
                // Different ports, or one/both are core-local — no contention
                (0, 0)
            }
        }
    }

    // --- Latency accounting ---

    /// Returns the cycle cost of the most recent bus access.
    pub fn last_access_cycles(&self) -> u32 {
        self.last_access_cycles
    }

    /// Returns accumulated extra wait states for the current instruction.
    pub fn extra_wait_states(&self) -> u32 {
        self.extra_wait_states
    }

    /// Reset extra wait state accumulator. Called at start of each instruction.
    pub fn reset_extra_wait_states(&mut self) {
        self.extra_wait_states = 0;
    }

    /// Compute read latency for an address region.
    #[inline(always)]
    fn read_latency(region: u32) -> (u32, u32) {
        match region {
            0x0 => (1, 0), // ROM
            0x1 => (1, 0), // XIP cache hit
            0x2 => (1, 0), // SRAM
            0x4 => (3, 2), // APB peripherals
            0x5 => (1, 0), // AHB peripherals
            0xD => (1, 0), // SIO
            0xE => (1, 0), // PPB
            _   => (1, 0), // unmapped
        }
    }

    /// Compute write latency for an address region.
    #[inline(always)]
    fn write_latency(region: u32) -> (u32, u32) {
        match region {
            0x2 => (1, 0), // SRAM
            0x4 => (4, 3), // APB peripherals
            0x5 => (1, 0), // AHB peripherals
            0xD => (1, 0), // SIO
            0xE => (1, 0), // PPB
            _   => (1, 0), // unmapped/ROM
        }
    }

    // --- 8-bit access ---

    pub fn read8(&mut self, addr: u32) -> u8 {
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x0 if offset < 0x8000 => self.memory.rom_read8(offset),
            0x1 => self.memory.xip_read8(offset),
            0x2 if offset < SRAM_SIZE as u32 => self.memory.sram_read8(offset),
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let word_addr = canonical & !3;
                let word = *self.peripheral_regs.get(&word_addr).unwrap_or(&0);
                let byte_idx = (canonical & 3) as usize;
                word.to_le_bytes()[byte_idx]
            }
            0xD => 0, // SIO (stub)
            0xE => 0, // PPB (stub)
            _ => 0,   // unmapped
        }
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x2 if offset < SRAM_SIZE as u32 => self.memory.sram_write8(offset, val),
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let word_addr = canonical & !3;
                let byte_idx = (canonical & 3) as usize;
                let old_word = *self.peripheral_regs.get(&word_addr).unwrap_or(&0);
                let mut bytes = old_word.to_le_bytes();
                let old_byte = bytes[byte_idx];
                bytes[byte_idx] = match alias {
                    0 => val,
                    1 => old_byte ^ val,
                    2 => old_byte | val,
                    3 => old_byte & !val,
                    _ => unreachable!(),
                };
                self.peripheral_regs.insert(word_addr, u32::from_le_bytes(bytes));
            }
            _ => {} // ROM read-only, others unmapped/stub
        }
    }

    // --- 16-bit access ---

    pub fn read16(&mut self, addr: u32) -> u16 {
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x0 if offset + 1 < 0x8000 => self.memory.rom_read16(offset),
            0x1 => self.memory.xip_read16(offset),
            0x2 if (offset + 1) < SRAM_SIZE as u32 => self.memory.sram_read16(offset),
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let word_addr = canonical & !3;
                let word = *self.peripheral_regs.get(&word_addr).unwrap_or(&0);
                let half_idx = ((canonical >> 1) & 1) as usize;
                let halves: [u16; 2] = [word as u16, (word >> 16) as u16];
                halves[half_idx]
            }
            0xD => 0, // SIO (stub)
            0xE => 0, // PPB (stub)
            _ => 0,
        }
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x2 if (offset + 1) < SRAM_SIZE as u32 => self.memory.sram_write16(offset, val),
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let word_addr = canonical & !3;
                let half_idx = ((canonical >> 1) & 1) as usize;
                let old_word = *self.peripheral_regs.get(&word_addr).unwrap_or(&0);
                let mut halves: [u16; 2] = [old_word as u16, (old_word >> 16) as u16];
                let old_half = halves[half_idx];
                halves[half_idx] = match alias {
                    0 => val,
                    1 => old_half ^ val,
                    2 => old_half | val,
                    3 => old_half & !val,
                    _ => unreachable!(),
                };
                let new_word = (halves[0] as u32) | ((halves[1] as u32) << 16);
                self.peripheral_regs.insert(word_addr, new_word);
            }
            _ => {}
        }
    }

    // --- 32-bit access ---

    pub fn read32(&mut self, addr: u32) -> u32 {
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x0 if offset + 3 < 0x8000 => self.memory.rom_read32(offset),
            0x1 => self.memory.xip_read32(offset),
            0x2 if (offset + 3) < SRAM_SIZE as u32 => self.memory.sram_read32(offset),
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                *self.peripheral_regs.get(&canonical).unwrap_or(&0)
            }
            0xD => 0, // SIO (stub)
            0xE => 0, // PPB (stub)
            // Unmapped or gap addresses return 0. BusFault is a Phase 3 feature.
            _ => 0,
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        // NOTE: SRAM atomic aliases (0x2100_0000 XOR, 0x2200_0000 SET, 0x2300_0000 CLR)
        // are not yet implemented — they use different address offsets and are deferred
        // to a later phase.
        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x2 if (offset + 3) < SRAM_SIZE as u32 => self.memory.sram_write32(offset, val),
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let old = *self.peripheral_regs.get(&canonical).unwrap_or(&0);
                let new_val = match alias {
                    0 => val,
                    1 => old ^ val,
                    2 => old | val,
                    3 => old & !val,
                    _ => unreachable!(),
                };
                self.peripheral_regs.insert(canonical, new_val);
            }
            _ => {}
        }
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

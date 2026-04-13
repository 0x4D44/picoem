pub mod peripherals;
pub mod ppb;

use std::collections::HashMap;

use crate::memory::{Memory, SRAM_SIZE};
use crate::pio::PioBlock;
use crate::sio::Sio;

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
    /// Downstream port core 0 last accessed this cycle (for contention detection).
    core0_port: Option<u8>,
    /// Whether to check contention on bus accesses (active during core 1's step).
    contention_check_active: bool,
    /// Per-core PPB register files (NVIC, SCB, SysTick stubs).
    pub ppb: [ppb::Ppb; 2],
    /// RESETS peripheral state: bits set = peripheral in reset.
    /// Default 0x1FFF_FFFF — all peripherals held in reset at boot.
    pub resets_state: u32,
    /// Bus fault detected on last access.
    bus_fault: bool,
    /// Address that caused the most recent bus fault.
    bus_fault_addr: u32,
    /// Whether flash (XIP) content has been loaded.
    flash_loaded: bool,
    /// Suppress per-word SRAM bank wait states during burst transfers
    /// (STM/LDM/PUSH/POP). The SRAM controller handles sequential word
    /// accesses without per-word bank penalties.
    burst_mode: bool,
    /// 4 KB boot RAM at 0xEFFF_F000..0xF000_0000.
    /// RP2350 maps this as the secure boot stack (USB DPRAM secure alias).
    /// Initial SP = 0xF000_0000 (top of this region).
    boot_ram: Box<[u8; 4096]>,
    /// 16 KB XIP SRAM at 0x1C00_0000..0x1C00_3FFF.
    /// RP2350 XIP cache memory accessible as SRAM.
    xip_sram: Box<[u8; 16384]>,
    /// QMI register backing store (offsets 0x000..0x06C, 28 words).
    qmi_regs: [u32; 28],
    /// CLK_REF_CTRL register (CLOCKS offset 0x030).
    clk_ref_ctrl: u32,
    /// CLK_SYS_CTRL register (CLOCKS offset 0x060).
    clk_sys_ctrl: u32,
    /// SIO GPIO_HI_IN (offset 0x008). Upper QSPI GPIO pins.
    /// When flash is loaded, returns pseudo-random noise to simulate
    /// QSPI pin activity (the bootrom samples this to detect flash).
    gpio_hi_noise_state: u32,
    /// XIP cache window offset: maps XIP SRAM reads (0x1C00_0000)
    /// to flash content at this byte offset. Set by QMI M0_RFMT writes.
    xip_cache_offset: u32,
    /// Single-cycle IO block (GPIO, CPUID, spinlocks, FIFO, divider, etc.).
    pub sio: Sio,
    /// Per-core event flag for WFE/SEV protocol.
    pub event_flag: [bool; 2],
    /// Per-core RCP salt value (shared state for cross-core writes).
    pub rcp_salt: [u32; 2],
    /// Per-core RCP salt validity flag.
    pub rcp_salt_valid: [bool; 2],
    /// Three PIO blocks (PIO0, PIO1, PIO2).
    pub pio: [PioBlock; 3],
    /// Combined GPIO pin state (readable by SIO and PIO).
    pub gpio_in: u32,
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: Memory::new(),
            last_access_cycles: 0,
            extra_wait_states: 0,
            peripheral_regs: HashMap::new(),
            resets_state: 0x1FFF_FFFF,
            core0_port: None,
            contention_check_active: false,
            ppb: [ppb::Ppb::default(), ppb::Ppb::default()],
            bus_fault: false,
            bus_fault_addr: 0,
            flash_loaded: false,
            burst_mode: false,
            boot_ram: Box::new([0u8; 4096]),
            xip_sram: Box::new([0u8; 16384]),
            qmi_regs: [0u32; 28],
            clk_ref_ctrl: 0,
            clk_sys_ctrl: 0,
            gpio_hi_noise_state: 0xA5A5_A5A5,
            xip_cache_offset: 0,
            sio: Sio::new(),
            event_flag: [false; 2],
            rcp_salt: [0; 2],
            rcp_salt_valid: [false; 2],
            pio: [PioBlock::new(), PioBlock::new(), PioBlock::new()],
            gpio_in: 0,
        }
    }

    // --- XIP SRAM helpers (0x1C00_0000..0x1C00_3FFF) ---

    fn is_xip_sram(addr: u32) -> bool {
        addr >= 0x1C00_0000 && addr < 0x1C00_4000
    }

    fn xip_sram_read8(&self, addr: u32) -> u8 {
        self.xip_sram[(addr - 0x1C00_0000) as usize]
    }

    fn xip_sram_write8(&mut self, addr: u32, val: u8) {
        self.xip_sram[(addr - 0x1C00_0000) as usize] = val;
    }

    fn xip_sram_read16(&self, addr: u32) -> u16 {
        let off = (addr - 0x1C00_0000) as usize;
        u16::from_le_bytes([self.xip_sram[off], self.xip_sram[off + 1]])
    }

    fn xip_sram_write16(&mut self, addr: u32, val: u16) {
        let off = (addr - 0x1C00_0000) as usize;
        self.xip_sram[off..off + 2].copy_from_slice(&val.to_le_bytes());
    }

    fn xip_sram_read32(&self, addr: u32) -> u32 {
        let off = (addr - 0x1C00_0000) as usize;
        u32::from_le_bytes([
            self.xip_sram[off], self.xip_sram[off + 1],
            self.xip_sram[off + 2], self.xip_sram[off + 3],
        ])
    }

    fn xip_sram_write32(&mut self, addr: u32, val: u32) {
        let off = (addr - 0x1C00_0000) as usize;
        self.xip_sram[off..off + 4].copy_from_slice(&val.to_le_bytes());
    }

    // --- Boot RAM helpers (0xEFFF_F000..0xF000_0000) ---

    /// Check if address is in the 4KB boot RAM region.
    pub fn is_boot_ram(addr: u32) -> bool {
        addr >= 0xEFFF_F000 && addr < 0xF000_0000
    }

    fn boot_ram_read8(&self, addr: u32) -> u8 {
        let off = (addr - 0xEFFF_F000) as usize;
        self.boot_ram[off]
    }

    fn boot_ram_write8(&mut self, addr: u32, val: u8) {
        let off = (addr - 0xEFFF_F000) as usize;
        self.boot_ram[off] = val;
    }

    pub fn boot_ram_read32(&self, addr: u32) -> u32 {
        let off = (addr - 0xEFFF_F000) as usize;
        u32::from_le_bytes([
            self.boot_ram[off],
            self.boot_ram[off + 1],
            self.boot_ram[off + 2],
            self.boot_ram[off + 3],
        ])
    }

    pub fn boot_ram_write32(&mut self, addr: u32, val: u32) {
        let off = (addr - 0xEFFF_F000) as usize;
        let bytes = val.to_le_bytes();
        self.boot_ram[off..off + 4].copy_from_slice(&bytes);
    }

    fn boot_ram_read16(&self, addr: u32) -> u16 {
        let off = (addr - 0xEFFF_F000) as usize;
        u16::from_le_bytes([self.boot_ram[off], self.boot_ram[off + 1]])
    }

    fn boot_ram_write16(&mut self, addr: u32, val: u16) {
        let off = (addr - 0xEFFF_F000) as usize;
        let bytes = val.to_le_bytes();
        self.boot_ram[off..off + 2].copy_from_slice(&bytes);
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

    /// Clear contention tracking state. Called at start of each tick.
    pub fn clear_contention_state(&mut self) {
        self.core0_port = None;
        self.contention_check_active = false;
    }

    /// Begin checking contention against core 0's recorded port.
    /// Called between core 0 and core 1 steps.
    pub fn begin_contention_check(&mut self) {
        self.contention_check_active = true;
    }

    /// Returns the active core index: 0 during core 0's step, 1 during core 1's.
    pub fn active_core(&self) -> usize {
        if self.contention_check_active { 1 } else { 0 }
    }

    /// Returns true if a bus fault was detected on the last access.
    pub fn bus_fault(&self) -> bool {
        self.bus_fault
    }

    /// Returns the address that caused the most recent bus fault.
    pub fn bus_fault_addr(&self) -> u32 {
        self.bus_fault_addr
    }

    /// Clear the bus fault flag.
    pub fn clear_bus_fault(&mut self) {
        self.bus_fault = false;
    }

    /// Set whether flash (XIP) content has been loaded.
    pub fn set_flash_loaded(&mut self, loaded: bool) {
        self.flash_loaded = loaded;
    }

    /// Signal an SEV event to both cores.
    pub fn signal_sev(&mut self) {
        self.event_flag[0] = true;
        self.event_flag[1] = true;
    }

    /// Read GPIO_HI_IN (SIO offset 0x008). Returns QSPI pin state.
    /// When flash is loaded, returns noise with bit 29 frequently set.
    /// The bootrom's flash-detect loop reads this 21 times, extracting
    /// bit 29 via `lsrs (gpio>>28), #2` and accumulating with `adcs`.
    /// The threshold is 0xF1 (241); without carry the sum is 231, so we
    /// need bit 29 set in ~11 of 21 reads.
    fn read_gpio_hi_in(&mut self) -> u32 {
        if !self.flash_loaded {
            return 0;
        }
        // Advance simple LFSR for variation, then force bit 29 on
        // most reads. Real QSPI lines are noisy — bias toward "alive".
        let s = self.gpio_hi_noise_state;
        self.gpio_hi_noise_state = s.wrapping_mul(1103515245).wrapping_add(12345);
        // Set bits 29-31 (QSPI data lines) to simulate flash responses.
        // Keep bit 28 toggling for additional entropy.
        self.gpio_hi_noise_state | 0xE000_0000
    }

    /// Load flash data into XIP memory and mark flash as loaded.
    pub fn load_flash(&mut self, data: &[u8]) {
        self.memory.load_flash(data);
        self.flash_loaded = true;
    }

    /// Check if this access contends with core 0. Returns extra stall cycles.
    /// Called internally by each read/write method.
    #[inline(always)]
    fn check_contention(&mut self, addr: u32) -> u32 {
        let port = Self::downstream_port(addr);
        if self.contention_check_active {
            // Core 1's access — check against core 0's port
            if let (Some(p0), Some(p1)) = (self.core0_port, port) {
                if p0 == p1 {
                    return 1;
                }
            }
        } else {
            // Core 0's access — record the port
            self.core0_port = port;
        }
        0
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

    /// Enable burst mode — suppresses per-word SRAM bank wait states.
    /// Used by multi-word instructions (STM/LDM/PUSH/POP).
    pub fn set_burst_mode(&mut self) {
        self.burst_mode = true;
    }

    /// Disable burst mode after multi-word transfer completes.
    pub fn clear_burst_mode(&mut self) {
        self.burst_mode = false;
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
        let contention = self.check_contention(addr);
        self.extra_wait_states += contention;

        let offset = match region {
            0x2 => addr & 0x00FF_FFFF, // strip SRAM alias bits [27:24]
            _   => addr & 0x0FFF_FFFF,
        };
        match region {
            0x0 if offset < 0x8000 => self.memory.rom_read8(offset),
            0x1 if Self::is_xip_sram(addr) && self.flash_loaded => {
                self.memory.xip_read8((addr - 0x1C00_0000) + self.xip_cache_offset)
            }
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_read8(addr),
            0x1 => {
                if !self.flash_loaded {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                    return 0;
                }
                self.memory.xip_read8(offset)
            }
            0x2 if offset < SRAM_SIZE as u32 => {
                let val = self.memory.sram_read8(offset);
                self.extra_wait_states += sram_bank_wait(addr, self.burst_mode);
                val
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let word_addr = canonical & !3;
                let offset = word_addr & 0x0000_0FFF;
                let word = match base {
                    0x4000_0000 => self.sysinfo_read(offset),
                    0x4002_0000 => self.resets_read(offset),
                    0x4001_0000 => self.clocks_read(offset),
                    0x4004_8000 => self.xosc_read(offset),
                    0x4005_0000 => self.pll_sys_read(offset),
                    0x4005_8000 => self.pll_usb_read(offset),
                    0x400D_0000 => self.qmi_read(offset),
                    0x5020_0000 => self.pio[0].read32(offset),
                    0x5030_0000 => self.pio[1].read32(offset),
                    0x5040_0000 => self.pio[2].read32(offset),
                    _ => *self.peripheral_regs.get(&word_addr).unwrap_or(&0),
                };
                let byte_idx = (canonical & 3) as usize;
                word.to_le_bytes()[byte_idx]
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                let word_offset = reg_offset & !3;
                let word = match word_offset {
                    0x004 => self.gpio_in,
                    0x008 => self.read_gpio_hi_in(),
                    _ => {
                        let core = self.active_core();
                        self.sio.read32(word_offset, core)
                    }
                };
                word.to_le_bytes()[(addr & 3) as usize]
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_read8(addr),
            0xE => 0, // PPB (stub)
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
                0
            }
        }
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;
        let contention = self.check_contention(addr);
        self.extra_wait_states += contention;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x00FF_FFFF;
        match region {
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_write8(addr, val),
            0x2 if offset < SRAM_SIZE as u32 => {
                let sram_alias = (addr >> 24) & 0x3;
                if sram_alias == 0 {
                    self.memory.sram_write8(offset, val);
                } else {
                    let old = self.memory.sram_read8(offset);
                    let new_val = match sram_alias {
                        1 => old ^ val,
                        2 => old | val,
                        3 => old & !val,
                        _ => unreachable!(),
                    };
                    self.memory.sram_write8(offset, new_val);
                }
                self.extra_wait_states += sram_bank_wait(addr, self.burst_mode);
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                match base {
                    0x4000_0000 | 0x4004_8000 | 0x4005_0000 | 0x4005_8000 => {
                        // SYSINFO (read-only), XOSC, PLL: ignore byte writes
                    }
                    0x400D_0000 => {
                        // QMI: do RMW on the word
                        let word_addr = canonical & !3;
                        let byte_idx = (canonical & 3) as usize;
                        let reg_offset = word_addr & 0x0000_0FFF;
                        let old_word = self.qmi_read(reg_offset);
                        let mut bytes = old_word.to_le_bytes();
                        bytes[byte_idx] = val;
                        self.qmi_write(reg_offset, u32::from_le_bytes(bytes));
                    }
                    0x4001_0000 => {
                        // CLOCKS: do RMW on the word
                        let word_addr = canonical & !3;
                        let byte_idx = (canonical & 3) as usize;
                        let reg_offset = word_addr & 0x0000_0FFF;
                        let old_word = self.clocks_read(reg_offset);
                        let mut bytes = old_word.to_le_bytes();
                        bytes[byte_idx] = val;
                        self.clocks_write(reg_offset, u32::from_le_bytes(bytes));
                    }
                    0x4002_0000 => {
                        // RESETS: only word-aligned writes meaningful, ignore byte
                    }
                    0x5020_0000 | 0x5030_0000 | 0x5040_0000 => {} // PIO: 32-bit access only
                    _ => {
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
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_write8(addr, val),
            _ => {} // ROM read-only, others unmapped/stub
        }
    }

    // --- 16-bit access ---

    pub fn read16(&mut self, addr: u32) -> u16 {
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;
        let contention = self.check_contention(addr);
        self.extra_wait_states += contention;

        let offset = match region {
            0x2 => addr & 0x00FF_FFFF, // strip SRAM alias bits [27:24]
            _   => addr & 0x0FFF_FFFF,
        };
        match region {
            0x0 if offset + 1 < 0x8000 => self.memory.rom_read16(offset),
            0x1 if Self::is_xip_sram(addr) && self.flash_loaded => {
                self.memory.xip_read16((addr - 0x1C00_0000) + self.xip_cache_offset)
            }
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_read16(addr),
            0x1 => {
                if !self.flash_loaded {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                    return 0;
                }
                self.memory.xip_read16(offset)
            }
            0x2 if (offset + 1) < SRAM_SIZE as u32 => {
                let val = self.memory.sram_read16(offset);
                self.extra_wait_states += sram_bank_wait(addr, self.burst_mode);
                val
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let word_addr = canonical & !3;
                let offset = word_addr & 0x0000_0FFF;
                let word = match base {
                    0x4000_0000 => self.sysinfo_read(offset),
                    0x4002_0000 => self.resets_read(offset),
                    0x4001_0000 => self.clocks_read(offset),
                    0x4004_8000 => self.xosc_read(offset),
                    0x4005_0000 => self.pll_sys_read(offset),
                    0x4005_8000 => self.pll_usb_read(offset),
                    0x400D_0000 => self.qmi_read(offset),
                    0x5020_0000 => self.pio[0].read32(offset),
                    0x5030_0000 => self.pio[1].read32(offset),
                    0x5040_0000 => self.pio[2].read32(offset),
                    _ => *self.peripheral_regs.get(&word_addr).unwrap_or(&0),
                };
                let half_idx = ((canonical >> 1) & 1) as usize;
                let halves: [u16; 2] = [word as u16, (word >> 16) as u16];
                halves[half_idx]
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                let word_offset = reg_offset & !3;
                let word = match word_offset {
                    0x004 => self.gpio_in,
                    0x008 => self.read_gpio_hi_in(),
                    _ => {
                        let core = self.active_core();
                        self.sio.read32(word_offset, core)
                    }
                };
                let half_idx = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half_idx]
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_read16(addr),
            0xE => {
                // PPB: extract halfword from 32-bit register read
                let word = self.ppb[self.active_core()].read32(addr & !3);
                let half_idx = ((addr >> 1) & 1) as usize;
                [word as u16, (word >> 16) as u16][half_idx]
            }
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
                0
            }
        }
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;
        let contention = self.check_contention(addr);
        self.extra_wait_states += contention;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x00FF_FFFF;
        match region {
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_write16(addr, val),
            0x2 if (offset + 1) < SRAM_SIZE as u32 => {
                let sram_alias = (addr >> 24) & 0x3;
                if sram_alias == 0 {
                    self.memory.sram_write16(offset, val);
                } else {
                    let old = self.memory.sram_read16(offset);
                    let new_val = match sram_alias {
                        1 => old ^ val,
                        2 => old | val,
                        3 => old & !val,
                        _ => unreachable!(),
                    };
                    self.memory.sram_write16(offset, new_val);
                }
                self.extra_wait_states += sram_bank_wait(addr, self.burst_mode);
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                match base {
                    0x4000_0000 | 0x4004_8000 | 0x4005_0000 | 0x4005_8000 => {
                        // SYSINFO (read-only), XOSC, PLL: ignore halfword writes
                    }
                    0x400D_0000 => {
                        // QMI: do RMW on the word
                        let word_addr = canonical & !3;
                        let half_idx = ((canonical >> 1) & 1) as usize;
                        let reg_offset = word_addr & 0x0000_0FFF;
                        let old_word = self.qmi_read(reg_offset);
                        let mut halves: [u16; 2] = [old_word as u16, (old_word >> 16) as u16];
                        halves[half_idx] = val;
                        self.qmi_write(reg_offset, (halves[0] as u32) | ((halves[1] as u32) << 16));
                    }
                    0x4001_0000 => {
                        // CLOCKS: do RMW on the word
                        let word_addr = canonical & !3;
                        let half_idx = ((canonical >> 1) & 1) as usize;
                        let reg_offset = word_addr & 0x0000_0FFF;
                        let old_word = self.clocks_read(reg_offset);
                        let mut halves: [u16; 2] = [old_word as u16, (old_word >> 16) as u16];
                        halves[half_idx] = val;
                        self.clocks_write(reg_offset, (halves[0] as u32) | ((halves[1] as u32) << 16));
                    }
                    0x4002_0000 => {
                        // RESETS: only word-aligned writes meaningful, ignore halfword
                    }
                    0x5020_0000 | 0x5030_0000 | 0x5040_0000 => {} // PIO: 32-bit access only
                    _ => {
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
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_write16(addr, val),
            _ => {}
        }
    }

    // --- 32-bit access ---

    pub fn read32(&mut self, addr: u32) -> u32 {
        let region = addr >> 28;
        let (cycles, extra) = Self::read_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;
        let contention = self.check_contention(addr);
        self.extra_wait_states += contention;

        let offset = match region {
            0x2 => addr & 0x00FF_FFFF, // strip SRAM alias bits [27:24]
            _   => addr & 0x0FFF_FFFF,
        };
        match region {
            0x0 if offset + 3 < 0x8000 => self.memory.rom_read32(offset),
            0x1 if Self::is_xip_sram(addr) && self.flash_loaded => {
                // XIP SRAM (0x1C00_0000): when flash is loaded, the bootrom
                // reads flash through this window. Map reads to flash content
                // using the current window offset tracked by QMI configuration.
                let xip_offset = (addr - 0x1C00_0000) + self.xip_cache_offset;
                self.memory.xip_read32(xip_offset)
            }
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_read32(addr),
            0x1 => {
                if !self.flash_loaded {
                    self.bus_fault = true;
                    self.bus_fault_addr = addr;
                    return 0;
                }
                self.memory.xip_read32(offset)
            }
            0x2 if (offset + 3) < SRAM_SIZE as u32 => {
                let val = self.memory.sram_read32(offset);
                self.extra_wait_states += sram_bank_wait(addr, self.burst_mode);
                val
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let offset = canonical & 0x0000_0FFF;
                match base {
                    0x4000_0000 => self.sysinfo_read(offset),
                    0x4002_0000 => self.resets_read(offset),
                    0x4001_0000 => self.clocks_read(offset),
                    0x4004_8000 => self.xosc_read(offset),
                    0x400E_8000 => self.rosc_read(offset),
                    0x4005_0000 => self.pll_sys_read(offset),
                    0x4005_8000 => self.pll_usb_read(offset),
                    0x400D_0000 => self.qmi_read(offset),
                    0x5020_0000 => self.pio[0].read32(offset),
                    0x5030_0000 => self.pio[1].read32(offset),
                    0x5040_0000 => self.pio[2].read32(offset),
                    _ => *self.peripheral_regs.get(&canonical).unwrap_or(&0),
                }
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                match reg_offset {
                    0x004 => self.gpio_in,
                    0x008 => self.read_gpio_hi_in(),
                    _ => {
                        let core = self.active_core();
                        self.sio.read32(reg_offset, core)
                    }
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_read32(addr),
            0xE => self.ppb[self.active_core()].read32(addr),
            _ => {
                self.bus_fault = true;
                self.bus_fault_addr = addr;
                0
            }
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        let region = addr >> 28;
        let alias = (addr >> 12) & 3;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;
        let contention = self.check_contention(addr);
        self.extra_wait_states += contention;

        // Interposed atomics: APB XOR/SET/CLR writes cost +2 cycles
        if region == 0x4 && alias != 0 {
            self.last_access_cycles += 2;
            self.extra_wait_states += 2;
        }

        let offset = addr & 0x00FF_FFFF;
        match region {
            0x1 if Self::is_xip_sram(addr) => self.xip_sram_write32(addr, val),
            0x2 if (offset + 3) < SRAM_SIZE as u32 => {
                let sram_alias = (addr >> 24) & 0x3;
                if sram_alias == 0 {
                    self.memory.sram_write32(offset, val);
                } else {
                    let old = self.memory.sram_read32(offset);
                    let new_val = match sram_alias {
                        1 => old ^ val,
                        2 => old | val,
                        3 => old & !val,
                        _ => unreachable!(),
                    };
                    self.memory.sram_write32(offset, new_val);
                }
                self.extra_wait_states += sram_bank_wait(addr, self.burst_mode);
            }
            0x4 | 0x5 => {
                let canonical = addr & !0x3000;
                let base = canonical & 0xFFFF_F000;
                let offset = canonical & 0x0000_0FFF;
                match base {
                    0x4002_0000 => self.resets_write(offset, val, alias),
                    0x400D_0000 => self.qmi_write(offset, val),
                    0x4001_0000 => self.clocks_write(offset, val),
                    0x400E_8000 | 0x4004_8000 | 0x4005_0000 | 0x4005_8000 => {
                        // ROSC, XOSC, PLL: accept writes, ignore
                    }
                    // SYSINFO (0x4000_0000): read-only, ignore writes
                    0x4000_0000 => {}
                    0x5020_0000 => self.pio[0].write32(offset, val, alias),
                    0x5030_0000 => self.pio[1].write32(offset, val, alias),
                    0x5040_0000 => self.pio[2].write32(offset, val, alias),
                    _ => {
                        // Existing HashMap path with alias logic
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
                }
            }
            0xD => {
                let reg_offset = addr & 0xFFF;
                let core = self.active_core();
                self.sio.write32(reg_offset, val, core);
                // FIFO_WR event signaling: set event_flag for receiver core.
                if let Some(receiver) = self.sio.pending_fifo_event.take() {
                    self.event_flag[receiver] = true;
                }
            }
            0xE if Self::is_boot_ram(addr) => self.boot_ram_write32(addr, val),
            0xE => {
                let core = self.active_core();
                self.ppb[core].write32(addr, val);
            }
            _ => {}
        }
    }
}

/// Extra wait-state for SRAM bank access.
/// Banks 2 and 6 have +1 cycle on RP2350 (measured on silicon via DWT CYCCNT).
/// Returns 0 during burst mode (STM/LDM/PUSH/POP) — the SRAM controller
/// handles sequential accesses without per-word bank penalties.
fn sram_bank_wait(addr: u32, burst: bool) -> u32 {
    if burst {
        return 0;
    }
    let offset = addr & 0x000F_FFFF;
    if offset < 0x8_0000 {
        // Striped SRAM0-7
        let bank = (offset >> 2) & 7;
        if bank == 2 || bank == 6 {
            1
        } else {
            0
        }
    } else {
        0 // SRAM8-9 non-striped: no extra wait
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

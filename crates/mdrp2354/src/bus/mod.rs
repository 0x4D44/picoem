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
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: Memory::new(),
            last_access_cycles: 0,
            extra_wait_states: 0,
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
            0x4 => 0, // APB peripherals (stub)
            0x5 => 0, // AHB peripherals (stub)
            0xD => 0, // SIO (stub)
            0xE => 0, // PPB (stub)
            _ => 0,   // unmapped
        }
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        let region = addr >> 28;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x2 if offset < SRAM_SIZE as u32 => self.memory.sram_write8(offset, val),
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
            0x4 => 0, // APB peripherals (stub)
            0x5 => 0, // AHB peripherals (stub)
            0xD => 0, // SIO (stub)
            0xE => 0, // PPB (stub)
            _ => 0,
        }
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        let region = addr >> 28;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x2 if (offset + 1) < SRAM_SIZE as u32 => self.memory.sram_write16(offset, val),
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
            0x4 => 0, // APB peripherals (stub)
            0x5 => 0, // AHB peripherals (stub)
            0xD => 0, // SIO (stub)
            0xE => 0, // PPB (stub)
            // Unmapped or gap addresses return 0. BusFault is a Phase 3 feature.
            _ => 0,
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        let region = addr >> 28;
        let (cycles, extra) = Self::write_latency(region);
        self.last_access_cycles = cycles;
        self.extra_wait_states += extra;

        let offset = addr & 0x0FFF_FFFF;
        match region {
            0x2 if (offset + 3) < SRAM_SIZE as u32 => self.memory.sram_write32(offset, val),
            _ => {}
        }
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

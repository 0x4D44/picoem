use crate::memory::{Memory, SRAM_SIZE};

/// Bus fabric — address decode and cycle accounting.
///
/// Phase 1: flat memory, single-cycle access everywhere.
/// Phase 2 adds AHB5 arbitration, APB bridge latency, bus contention.
pub struct Bus {
    pub memory: Memory,
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: Memory::new(),
        }
    }

    // --- 8-bit access ---

    pub fn read8(&self, addr: u32) -> u8 {
        let offset = addr & 0x0FFF_FFFF;
        match addr >> 28 {
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
        let offset = addr & 0x0FFF_FFFF;
        match addr >> 28 {
            0x2 if offset < SRAM_SIZE as u32 => self.memory.sram_write8(offset, val),
            _ => {} // ROM read-only, others unmapped/stub
        }
    }

    // --- 16-bit access ---

    pub fn read16(&self, addr: u32) -> u16 {
        let offset = addr & 0x0FFF_FFFF;
        match addr >> 28 {
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
        let offset = addr & 0x0FFF_FFFF;
        match addr >> 28 {
            0x2 if (offset + 1) < SRAM_SIZE as u32 => self.memory.sram_write16(offset, val),
            _ => {}
        }
    }

    // --- 32-bit access ---

    pub fn read32(&self, addr: u32) -> u32 {
        let offset = addr & 0x0FFF_FFFF;
        match addr >> 28 {
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
        let offset = addr & 0x0FFF_FFFF;
        match addr >> 28 {
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

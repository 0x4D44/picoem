use crate::memory::Memory;

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
        match addr >> 28 {
            0x0 => self.memory.rom_read8(addr & 0x0FFF_FFFF),
            0x1 => self.memory.xip_read8(addr & 0x0FFF_FFFF),
            0x2 => self.memory.sram_read8(addr & 0x0FFF_FFFF),
            // 0x4 => APB peripherals (stub)
            // 0x5 => AHB peripherals (stub)
            // 0xD => SIO (stub)
            // 0xE => PPB (stub)
            _ => 0,
        }
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        match addr >> 28 {
            0x2 => self.memory.sram_write8(addr & 0x0FFF_FFFF, val),
            _ => {} // ROM read-only, others unmapped/stub
        }
    }

    // --- 16-bit access ---

    pub fn read16(&self, addr: u32) -> u16 {
        match addr >> 28 {
            0x0 => self.memory.rom_read16(addr & 0x0FFF_FFFF),
            0x1 => self.memory.xip_read16(addr & 0x0FFF_FFFF),
            0x2 => self.memory.sram_read16(addr & 0x0FFF_FFFF),
            _ => 0,
        }
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        match addr >> 28 {
            0x2 => self.memory.sram_write16(addr & 0x0FFF_FFFF, val),
            _ => {}
        }
    }

    // --- 32-bit access ---

    pub fn read32(&self, addr: u32) -> u32 {
        match addr >> 28 {
            0x0 => self.memory.rom_read32(addr & 0x0FFF_FFFF),
            0x1 => self.memory.xip_read32(addr & 0x0FFF_FFFF),
            0x2 => self.memory.sram_read32(addr & 0x0FFF_FFFF),
            _ => 0,
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        match addr >> 28 {
            0x2 => self.memory.sram_write32(addr & 0x0FFF_FFFF, val),
            _ => {}
        }
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

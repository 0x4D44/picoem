/// ROM size: 32 kB.
pub const ROM_SIZE: usize = 32 * 1024;
/// SRAM size: 520 kB (10 banks: SRAM0-7 striped 64 kB each, SRAM8-9 non-striped 4 kB each).
pub const SRAM_SIZE: usize = 520 * 1024;

/// Unified memory backing stores. Owns the actual byte arrays for ROM, SRAM,
/// and flash (XIP). No bus fabric or timing — just raw storage.
pub struct Memory {
    rom: Vec<u8>,
    sram: Vec<u8>,
    xip: Vec<u8>,
}

impl Memory {
    pub fn new() -> Self {
        Self {
            rom: vec![0u8; ROM_SIZE],
            sram: vec![0u8; SRAM_SIZE],
            xip: Vec::new(),
        }
    }

    /// Construct a `Memory` with chip-specific ROM and SRAM sizes.
    /// Used by `mdrp2040` (16 KB ROM, 264 KB SRAM) and any future chip
    /// crate that differs from the RP2350 defaults baked into `new()`.
    /// XIP starts empty; populate via `load_flash`.
    pub fn with_sizes(rom_size: usize, sram_size: usize) -> Self {
        Self {
            rom: vec![0u8; rom_size],
            sram: vec![0u8; sram_size],
            xip: Vec::new(),
        }
    }

    // --- ROM ---

    pub fn load_rom(&mut self, data: &[u8]) {
        let len = data.len().min(ROM_SIZE);
        self.rom[..len].copy_from_slice(&data[..len]);
    }

    pub fn rom_read8(&self, offset: u32) -> u8 {
        self.rom.get(offset as usize).copied().unwrap_or(0)
    }

    pub fn rom_read16(&self, offset: u32) -> u16 {
        let off = offset as usize;
        if off + 1 < self.rom.len() {
            u16::from_le_bytes([self.rom[off], self.rom[off + 1]])
        } else {
            0
        }
    }

    pub fn rom_read32(&self, offset: u32) -> u32 {
        let off = offset as usize;
        if off + 3 < self.rom.len() {
            u32::from_le_bytes([
                self.rom[off],
                self.rom[off + 1],
                self.rom[off + 2],
                self.rom[off + 3],
            ])
        } else {
            0
        }
    }

    // --- SRAM ---

    pub fn sram_read8(&self, offset: u32) -> u8 {
        self.sram.get(offset as usize).copied().unwrap_or(0)
    }

    pub fn sram_read16(&self, offset: u32) -> u16 {
        let off = offset as usize;
        if off + 1 < self.sram.len() {
            u16::from_le_bytes([self.sram[off], self.sram[off + 1]])
        } else {
            0
        }
    }

    pub fn sram_read32(&self, offset: u32) -> u32 {
        let off = offset as usize;
        if off + 3 < self.sram.len() {
            u32::from_le_bytes([
                self.sram[off],
                self.sram[off + 1],
                self.sram[off + 2],
                self.sram[off + 3],
            ])
        } else {
            0
        }
    }

    pub fn sram_write8(&mut self, offset: u32, val: u8) {
        let off = offset as usize;
        if off < self.sram.len() {
            self.sram[off] = val;
        }
    }

    pub fn sram_write16(&mut self, offset: u32, val: u16) {
        let off = offset as usize;
        if off + 1 < self.sram.len() {
            let bytes = val.to_le_bytes();
            self.sram[off] = bytes[0];
            self.sram[off + 1] = bytes[1];
        }
    }

    pub fn sram_write32(&mut self, offset: u32, val: u32) {
        let off = offset as usize;
        if off + 3 < self.sram.len() {
            let bytes = val.to_le_bytes();
            self.sram[off] = bytes[0];
            self.sram[off + 1] = bytes[1];
            self.sram[off + 2] = bytes[2];
            self.sram[off + 3] = bytes[3];
        }
    }

    // --- XIP (flash) ---

    pub fn load_flash(&mut self, data: &[u8]) {
        self.xip = data.to_vec();
    }

    pub fn xip_read8(&self, offset: u32) -> u8 {
        self.xip.get(offset as usize).copied().unwrap_or(0)
    }

    pub fn xip_read16(&self, offset: u32) -> u16 {
        let off = offset as usize;
        if off + 1 < self.xip.len() {
            u16::from_le_bytes([self.xip[off], self.xip[off + 1]])
        } else {
            0
        }
    }

    pub fn xip_read32(&self, offset: u32) -> u32 {
        let off = offset as usize;
        if off + 3 < self.xip.len() {
            u32::from_le_bytes([
                self.xip[off],
                self.xip[off + 1],
                self.xip[off + 2],
                self.xip[off + 3],
            ])
        } else {
            0
        }
    }

    // --- Direct access (for test / debug, bypasses bus) ---

    pub fn peek8(&self, addr: u32) -> u8 {
        match addr >> 28 {
            0x0 => self.rom_read8(addr & 0x0FFF_FFFF),
            0x1 => self.xip_read8(addr & 0x0FFF_FFFF),
            0x2 => self.sram_read8(addr & 0x00FF_FFFF), // strip SRAM alias bits
            _ => 0,
        }
    }

    pub fn poke8(&mut self, addr: u32, val: u8) {
        match addr >> 28 {
            0x2 => self.sram_write8(addr & 0x00FF_FFFF, val),
            _ => {} // ROM and XIP are read-only, others unmapped
        }
    }

    pub fn peek32(&self, addr: u32) -> u32 {
        match addr >> 28 {
            0x0 => self.rom_read32(addr & 0x0FFF_FFFF),
            0x1 => self.xip_read32(addr & 0x0FFF_FFFF),
            0x2 => self.sram_read32(addr & 0x00FF_FFFF), // strip SRAM alias bits
            _ => 0,
        }
    }

    pub fn poke32(&mut self, addr: u32, val: u32) {
        match addr >> 28 {
            0x2 => self.sram_write32(addr & 0x00FF_FFFF, val),
            _ => {} // ROM and XIP are read-only, others unmapped
        }
    }
}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}

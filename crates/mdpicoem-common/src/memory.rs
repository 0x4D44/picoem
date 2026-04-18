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

    /// Construct a `Memory` with chip-specific ROM, SRAM, and a fixed-size
    /// flash window. Used by `mdrp2040` for its 2 MB XIP window: the
    /// bus decoder maps a fixed address range, so the flash buffer must
    /// cover the whole window regardless of image size.
    ///
    /// Flash bytes are zero-initialised; populate via [`Self::load_flash`],
    /// which clamps to `flash_size` and zeroes any remaining tail.
    pub fn with_flash(rom_size: usize, sram_size: usize, flash_size: usize) -> Self {
        Self {
            rom: vec![0u8; rom_size],
            sram: vec![0u8; sram_size],
            xip: vec![0u8; flash_size],
        }
    }

    /// Current flash (XIP) buffer size in bytes. Zero when constructed
    /// via `new()` / `with_sizes()` and `load_flash` has not been called
    /// yet (mdrp2350 dynamic-resize path).
    pub fn flash_size(&self) -> usize {
        self.xip.len()
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

    /// Copy `data` into the flash buffer starting at offset 0.
    ///
    /// * If the buffer was pre-sized via [`Self::with_flash`], the copy
    ///   clamps at the buffer length and any previously-loaded tail is
    ///   zeroed so a re-load doesn't leak stale bytes past the new image.
    /// * Otherwise (default / `with_sizes` path — mdrp2350) the buffer
    ///   is resized to match the new data. This preserves the pre-PicoGUS
    ///   behaviour where callers treat XIP as a dynamically-sized image.
    pub fn load_flash(&mut self, data: &[u8]) {
        if self.xip.is_empty() {
            self.xip = data.to_vec();
        } else {
            let n = data.len().min(self.xip.len());
            self.xip[..n].copy_from_slice(&data[..n]);
            for b in &mut self.xip[n..] {
                *b = 0;
            }
        }
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

    /// Consume the backing store, yielding `(rom, sram, xip)` Vec<u8>
    /// triples. Used by the threading runtime (`mdrp2350::threaded`) to
    /// seed a `SharedMemory` from an existing `Emulator`'s `Bus::memory`
    /// without bulk-reading every byte through the scalar accessors.
    pub fn into_parts(self) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        (self.rom, self.sram, self.xip)
    }
}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn with_flash_preallocates_zeroed_buffer() {
        // `with_flash` is the pre-sized flash constructor used by chips
        // with a fixed-capacity flash window (e.g. mdrp2040's 2 MB XIP).
        // Pre-allocated bytes must read back as zero.
        let mem = Memory::with_flash(16 * 1024, 264 * 1024, 2 * 1024 * 1024);
        assert_eq!(mem.flash_size(), 2 * 1024 * 1024);
        assert_eq!(mem.xip_read8(0), 0);
        assert_eq!(mem.xip_read8(2 * 1024 * 1024 - 1), 0);
        assert_eq!(mem.xip_read32(0), 0);
    }

    #[test]
    fn with_flash_load_clamps_into_fixed_buffer() {
        // Loading data into a pre-sized buffer clamps at capacity and
        // copies from offset 0. Previously-loaded bytes past the new
        // image are zeroed so a re-load doesn't leak stale content.
        let mut mem = Memory::with_flash(16 * 1024, 264 * 1024, 2 * 1024 * 1024);
        mem.load_flash(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(mem.xip_read32(0), 0xDDCCBBAA);
        // Past the loaded length: still zero within the mapped window.
        assert_eq!(mem.xip_read8(4), 0);
        assert_eq!(mem.xip_read8((2 * 1024 * 1024) - 1), 0);
        // Re-load with a shorter image: old tail must be zeroed.
        mem.load_flash(&[0x01]);
        assert_eq!(mem.xip_read8(0), 0x01);
        assert_eq!(mem.xip_read8(1), 0);
        assert_eq!(mem.xip_read8(3), 0);
    }

    #[test]
    fn with_sizes_keeps_legacy_dynamic_flash_behavior() {
        // mdrp2350 uses `with_sizes` and expects `load_flash` to resize
        // the buffer to the loaded bytes (current behaviour). Changing
        // this would break the RP2350 XIP tests.
        let mut mem = Memory::with_sizes(32 * 1024, 520 * 1024);
        assert_eq!(mem.flash_size(), 0);
        mem.load_flash(&[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(mem.flash_size(), 4);
        assert_eq!(mem.xip_read32(0), 0x44332211);
    }
}

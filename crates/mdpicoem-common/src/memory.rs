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
        let len = data.len().min(self.rom.len());
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
        // ROM and XIP are read-only, others unmapped.
        if addr >> 28 == 0x2 {
            self.sram_write8(addr & 0x00FF_FFFF, val);
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
        // ROM and XIP are read-only, others unmapped.
        if addr >> 28 == 0x2 {
            self.sram_write32(addr & 0x00FF_FFFF, val);
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

    // ------------------------------------------------------------------
    // Default constructor — covers the RP2350 sizes and the Default impl
    // ------------------------------------------------------------------

    #[test]
    fn new_uses_rp2350_default_sizes() {
        let mem = Memory::new();
        // ROM reads at the tail must succeed (returns 0, unmapped upper
        // word reads must fall through to the out-of-range branch).
        assert_eq!(mem.rom_read8(ROM_SIZE as u32 - 1), 0);
        // XIP starts empty on the default construction path.
        assert_eq!(mem.flash_size(), 0);
    }

    #[test]
    fn default_matches_new() {
        // The Default impl just delegates to `new()`. Exercising it here
        // covers the otherwise-dead `impl Default for Memory`.
        let a = Memory::default();
        let b = Memory::new();
        assert_eq!(a.flash_size(), b.flash_size());
        // Same SRAM sizing: last byte at SRAM_SIZE-1 reads 0 on both.
        assert_eq!(a.sram_read8(SRAM_SIZE as u32 - 1), 0);
        assert_eq!(b.sram_read8(SRAM_SIZE as u32 - 1), 0);
    }

    // ------------------------------------------------------------------
    // ROM read paths — in-range round-trip and out-of-range fall-through
    // ------------------------------------------------------------------

    #[test]
    fn rom_load_and_read_roundtrip() {
        let mut mem = Memory::new();
        // `load_rom` clamps at ROM_SIZE, so oversize input must truncate.
        let data: Vec<u8> = (0..ROM_SIZE as u32 + 64)
            .map(|i| (i & 0xFF) as u8)
            .collect();
        mem.load_rom(&data);
        // First few bytes round-trip as-written.
        assert_eq!(mem.rom_read8(0), 0x00);
        assert_eq!(mem.rom_read8(1), 0x01);
        // 16- and 32-bit reads inside the in-range branch.
        let expected16 = u16::from_le_bytes([data[2], data[3]]);
        assert_eq!(mem.rom_read16(2), expected16);
        let expected32 = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        assert_eq!(mem.rom_read32(4), expected32);
    }

    #[test]
    fn rom_reads_out_of_range_return_zero() {
        // Covers the `else { 0 }` branches at memory.rs:70 (rom_read16)
        // and :79 (rom_read32) plus the `unwrap_or(0)` path in rom_read8.
        let mut mem = Memory::new();
        mem.load_rom(&[0xFF; 16]);
        // read8 past the end: unwrap_or path.
        assert_eq!(mem.rom_read8(ROM_SIZE as u32), 0);
        // read16 where off+1 == rom.len(): both len-1 and len trigger.
        assert_eq!(mem.rom_read16(ROM_SIZE as u32 - 1), 0);
        assert_eq!(mem.rom_read16(ROM_SIZE as u32), 0);
        // read32 where off+3 reaches past end: covers the fallback.
        assert_eq!(mem.rom_read32(ROM_SIZE as u32 - 3), 0);
        assert_eq!(mem.rom_read32(ROM_SIZE as u32), 0);
    }

    // ------------------------------------------------------------------
    // SRAM paths — in-range, writes round-trip, out-of-range no-ops
    // ------------------------------------------------------------------

    #[test]
    fn sram_roundtrip_all_widths() {
        let mut mem = Memory::new();
        mem.sram_write8(0, 0xAB);
        assert_eq!(mem.sram_read8(0), 0xAB);
        mem.sram_write16(4, 0xBEEF);
        assert_eq!(mem.sram_read16(4), 0xBEEF);
        mem.sram_write32(8, 0xDEAD_BEEF);
        assert_eq!(mem.sram_read32(8), 0xDEAD_BEEF);
    }

    #[test]
    fn sram_reads_out_of_range_return_zero() {
        // Covers the `else { 0 }` branches at :99 and :108.
        let mem = Memory::new();
        assert_eq!(mem.sram_read8(SRAM_SIZE as u32), 0);
        assert_eq!(mem.sram_read16(SRAM_SIZE as u32 - 1), 0);
        assert_eq!(mem.sram_read32(SRAM_SIZE as u32 - 3), 0);
    }

    #[test]
    fn sram_writes_out_of_range_are_noops() {
        // Covers the bounds-guard branches at :122 (write8), :129 (write16),
        // :138 (write32). None must panic and none must mutate past the tail.
        let mut mem = Memory::new();
        // Seed the last valid byte so we can detect a stray write.
        mem.sram_write8(SRAM_SIZE as u32 - 1, 0x5A);
        mem.sram_write8(SRAM_SIZE as u32, 0xFF);
        // write16 where off+1 == len: off=SRAM_SIZE-1, off+1=SRAM_SIZE → skip.
        mem.sram_write16(SRAM_SIZE as u32 - 1, 0x1234);
        // write32 where off+3 >= len: off=SRAM_SIZE-3, off+3=SRAM_SIZE → skip.
        mem.sram_write32(SRAM_SIZE as u32 - 3, 0xDEAD_BEEF);
        // Last valid byte still the sentinel — no write leaked through.
        assert_eq!(mem.sram_read8(SRAM_SIZE as u32 - 1), 0x5A);
        // Reads past end return 0 and do not panic.
        assert_eq!(mem.sram_read8(SRAM_SIZE as u32 + 16), 0);
    }

    #[test]
    fn sram_write_then_read_at_edge_valid() {
        // Edge of the in-range branch: off = SRAM_SIZE - 4 is valid for
        // write32 (off+3 = SRAM_SIZE-1, still < len).
        let mut mem = Memory::new();
        let edge = SRAM_SIZE as u32 - 4;
        mem.sram_write32(edge, 0xCAFE_F00D);
        assert_eq!(mem.sram_read32(edge), 0xCAFE_F00D);
    }

    // ------------------------------------------------------------------
    // XIP out-of-range branches — :175 and :184
    // ------------------------------------------------------------------

    #[test]
    fn xip_reads_out_of_range_return_zero() {
        // Fixed-size XIP: out-of-range reads must hit the `else { 0 }`
        // arms at :175 (xip_read16) and :184 (xip_read32).
        let mut mem = Memory::with_flash(16 * 1024, 264 * 1024, 16);
        mem.load_flash(&[0xDE, 0xAD, 0xBE, 0xEF]);
        // In-range sanity — covers the `if` arm of xip_read16 and xip_read32.
        assert_eq!(mem.xip_read16(0), 0xADDE);
        assert_eq!(mem.xip_read16(2), 0xEFBE);
        assert_eq!(mem.xip_read32(0), 0xEFBEADDE);
        // Out-of-range read16: off+1 == len → fallback.
        assert_eq!(mem.xip_read16(15), 0);
        assert_eq!(mem.xip_read16(16), 0);
        // Out-of-range read32: off+3 >= len → fallback.
        assert_eq!(mem.xip_read32(13), 0);
        assert_eq!(mem.xip_read32(16), 0);
        // xip_read8 past end: Vec::get().unwrap_or(0).
        assert_eq!(mem.xip_read8(16), 0);
    }

    #[test]
    fn xip_dynamic_mode_out_of_range_reads_zero() {
        // The `with_sizes` / dynamic XIP path: before any load_flash
        // call, xip is empty and every read is out of range.
        let mem = Memory::with_sizes(32 * 1024, 520 * 1024);
        assert_eq!(mem.xip_read8(0), 0);
        assert_eq!(mem.xip_read16(0), 0);
        assert_eq!(mem.xip_read32(0), 0);
    }

    // ------------------------------------------------------------------
    // peek / poke dispatchers and into_parts
    // ------------------------------------------------------------------

    #[test]
    fn peek_and_poke_cover_all_regions() {
        // peek8/peek32/poke8/poke32 dispatch on addr[31:28] and only
        // writes into SRAM are honoured. Exercise every arm.
        let mut mem = Memory::with_flash(16, 64, 8);
        // Seed ROM and XIP so their peek arms observe non-zero data.
        mem.load_rom(&[0x11, 0x22, 0x33, 0x44]);
        mem.load_flash(&[0xAA, 0xBB, 0xCC, 0xDD]);
        // ROM peek arm (addr[31:28] == 0x0).
        assert_eq!(mem.peek8(0x0000_0000), 0x11);
        assert_eq!(mem.peek32(0x0000_0000), 0x44332211);
        // XIP peek arm (addr[31:28] == 0x1).
        assert_eq!(mem.peek8(0x1000_0000), 0xAA);
        assert_eq!(mem.peek32(0x1000_0000), 0xDDCCBBAA);
        // SRAM peek arm (addr[31:28] == 0x2). Strip the alias bits.
        mem.poke32(0x2000_0010, 0xF00D_CAFE);
        assert_eq!(mem.peek32(0x2000_0010), 0xF00D_CAFE);
        mem.poke8(0x2000_0020, 0x7E);
        assert_eq!(mem.peek8(0x2000_0020), 0x7E);
        // Unmapped region (addr[31:28] == 0xE): peek returns 0 default.
        assert_eq!(mem.peek8(0xE000_0000), 0);
        assert_eq!(mem.peek32(0xE000_0000), 0);
        // Writes into ROM/XIP/unmapped must be silent no-ops — SRAM stays
        // intact. Read the earlier SRAM byte to prove nothing was lost.
        mem.poke8(0x0000_0000, 0xFF); // ROM range — no-op
        mem.poke32(0x1000_0000, 0xFFFF_FFFF); // XIP range — no-op
        mem.poke8(0xE000_0000, 0xFF); // unmapped — no-op
        mem.poke32(0xE000_0000, 0xFFFF_FFFF); // unmapped — no-op
        assert_eq!(mem.peek8(0x2000_0020), 0x7E);
        assert_eq!(mem.peek32(0x2000_0010), 0xF00D_CAFE);
    }

    #[test]
    fn into_parts_yields_owned_buffers() {
        // `into_parts` is the hand-off path used by the threading
        // runtime. Verify it surfaces the three independently-sized Vecs.
        let mut mem = Memory::with_flash(16, 64, 8);
        mem.load_rom(&[0x77; 4]);
        mem.sram_write8(3, 0xA5);
        mem.load_flash(&[0x42; 2]);
        let (rom, sram, xip) = mem.into_parts();
        assert_eq!(rom.len(), 16);
        assert_eq!(rom[..4], [0x77; 4]);
        assert_eq!(sram.len(), 64);
        assert_eq!(sram[3], 0xA5);
        assert_eq!(xip.len(), 8);
        assert_eq!(xip[..2], [0x42; 2]);
        // Tail of the pre-sized XIP stays zero after the clamp.
        assert_eq!(xip[2..], [0u8; 6]);
    }

    #[test]
    fn load_rom_clamps_to_rom_len_not_constant() {
        // Regression: `load_rom` previously clamped against the
        // RP2350 `ROM_SIZE` constant (32 KB) regardless of the
        // actual `self.rom` length. On the RP2040 path
        // (`with_sizes(16 * 1024, ...)`) a 32 KB input would panic
        // inside `copy_from_slice` because the destination slice
        // was only 16 KB long. The fix clamps to `self.rom.len()`
        // — symmetric with `load_flash`.
        let mut mem = Memory::with_sizes(16 * 1024, 0);
        let data: Vec<u8> = (0..32 * 1024_u32).map(|i| (i & 0xFF) as u8).collect();
        // Must not panic.
        mem.load_rom(&data);
        // First 16 KB must be the input prefix verbatim.
        for i in 0..16 * 1024_u32 {
            assert_eq!(mem.rom_read8(i), (i & 0xFF) as u8);
        }
        // Reads past `self.rom.len()` fall through to the
        // out-of-range branches — they return 0, not the input
        // tail. Confirms we did not silently grow `rom`.
        assert_eq!(mem.rom_read8(0x4000), 0);
        assert_eq!(mem.rom_read32(0x4000), 0);
    }

    #[test]
    fn with_sizes_zero_regions_are_usable() {
        // Corner: zero-sized ROM and SRAM. Every read must fall through
        // to the out-of-range branch (no panic) and every write must be
        // a no-op. `load_rom` with empty input also degenerates cleanly.
        let mut mem = Memory::with_sizes(0, 0);
        mem.load_rom(&[]);
        mem.sram_write8(0, 0xFF);
        mem.sram_write16(0, 0xFFFF);
        mem.sram_write32(0, 0xFFFF_FFFF);
        assert_eq!(mem.rom_read8(0), 0);
        assert_eq!(mem.rom_read16(0), 0);
        assert_eq!(mem.rom_read32(0), 0);
        assert_eq!(mem.sram_read8(0), 0);
        assert_eq!(mem.sram_read16(0), 0);
        assert_eq!(mem.sram_read32(0), 0);
    }
}

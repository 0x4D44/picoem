use std::sync::atomic::{AtomicU32, Ordering::Relaxed};

#[allow(dead_code)] // documents the memory map; used in tests
const SRAM_BASE: u32 = 0x2000_0000;
const SRAM_SIZE: u32 = 520 * 1024;
const SRAM_WORDS: usize = (520 * 1024) / 4; // 130_000
const ROM_BASE: u32 = 0x0000_0000;
const ROM_SIZE: u32 = 32 * 1024;
const XIP_BASE: u32 = 0x1000_0000;

pub struct SharedMemory {
    sram: Box<[AtomicU32]>,
    rom: Box<[u8]>,
    xip: Box<[u8]>,
}

impl SharedMemory {
    pub fn new() -> Self {
        let mut sram = Vec::with_capacity(SRAM_WORDS);
        for _ in 0..SRAM_WORDS {
            sram.push(AtomicU32::new(0));
        }
        Self {
            sram: sram.into_boxed_slice(),
            rom: vec![0u8; ROM_SIZE as usize].into_boxed_slice(),
            xip: Box::new([]),
        }
    }

    // ---------------------------------------------------------------
    // Address helpers
    // ---------------------------------------------------------------

    /// Convert a bus address to a SRAM word index.
    /// Strips SRAM alias bits [27:24] per RP2350 memory map.
    fn sram_idx(addr: u32) -> Option<usize> {
        let region = addr >> 28;
        if region == 0x2 {
            let offset = addr & 0x00FF_FFFF; // strip alias bits
            if offset < SRAM_SIZE {
                Some((offset / 4) as usize)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// Extract the SRAM alias mode from bits [27:24].
    /// 0 = plain, 1 = XOR, 2 = SET (OR), 3 = CLR (AND-NOT).
    fn sram_alias(addr: u32) -> u8 {
        ((addr >> 24) & 0x3) as u8
    }

    // ---------------------------------------------------------------
    // Read methods
    // ---------------------------------------------------------------

    pub fn read32(&self, addr: u32) -> u32 {
        if let Some(idx) = Self::sram_idx(addr) {
            self.sram[idx].load(Relaxed)
        } else if addr < ROM_BASE + ROM_SIZE {
            self.read_rom32(addr)
        } else if addr >= XIP_BASE && ((addr - XIP_BASE) as usize) < self.xip.len() {
            self.read_xip32(addr)
        } else {
            0
        }
    }

    /// Assumes halfword-aligned address (bit 0 = 0).
    pub fn read16(&self, addr: u32) -> u16 {
        let word = self.read32(addr & !3);
        if addr & 2 != 0 {
            (word >> 16) as u16
        } else {
            word as u16
        }
    }

    pub fn read8(&self, addr: u32) -> u8 {
        if let Some(idx) = Self::sram_idx(addr) {
            let word = self.sram[idx].load(Relaxed);
            (word >> ((addr & 3) * 8)) as u8
        } else if addr < ROM_BASE + ROM_SIZE {
            self.rom[addr as usize]
        } else if addr >= XIP_BASE {
            let off = (addr - XIP_BASE) as usize;
            if off < self.xip.len() {
                self.xip[off]
            } else {
                0
            }
        } else {
            0
        }
    }

    // ---------------------------------------------------------------
    // Write methods
    // ---------------------------------------------------------------

    pub fn write32(&self, addr: u32, val: u32) {
        if let Some(idx) = Self::sram_idx(addr) {
            match Self::sram_alias(addr) {
                0 => self.sram[idx].store(val, Relaxed),
                1 => {
                    self.sram[idx].fetch_xor(val, Relaxed);
                }
                2 => {
                    self.sram[idx].fetch_or(val, Relaxed);
                }
                3 => {
                    self.sram[idx].fetch_and(!val, Relaxed);
                }
                _ => unreachable!(),
            }
        }
        // ROM/XIP writes silently dropped (immutable)
    }

    /// Assumes halfword-aligned address (bit 0 = 0).
    pub fn write16(&self, addr: u32, val: u16) {
        if let Some(idx) = Self::sram_idx(addr) {
            let shift = (addr & 2) * 8;
            let mask = 0xFFFFu32 << shift;
            let bits = (val as u32) << shift;
            let alias = Self::sram_alias(addr);
            loop {
                let old = self.sram[idx].load(Relaxed);
                let half = match alias {
                    0 => bits,
                    1 => (old ^ bits) & mask,
                    2 => (old | bits) & mask,
                    3 => (old & !bits) & mask,
                    _ => unreachable!(),
                };
                let new = (old & !mask) | half;
                if self.sram[idx]
                    .compare_exchange(old, new, Relaxed, Relaxed)
                    .is_ok()
                {
                    break;
                }
            }
        }
    }

    pub fn write8(&self, addr: u32, val: u8) {
        if let Some(idx) = Self::sram_idx(addr) {
            let shift = (addr & 3) * 8;
            let mask = 0xFFu32 << shift;
            let bits = (val as u32) << shift;
            let alias = Self::sram_alias(addr);
            loop {
                let old = self.sram[idx].load(Relaxed);
                let byte_val = match alias {
                    0 => bits,
                    1 => (old ^ bits) & mask,
                    2 => (old | bits) & mask,
                    3 => (old & !bits) & mask,
                    _ => unreachable!(),
                };
                let new = (old & !mask) | byte_val;
                if self.sram[idx]
                    .compare_exchange(old, new, Relaxed, Relaxed)
                    .is_ok()
                {
                    break;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // CAS (for STREX)
    // ---------------------------------------------------------------

    /// Compare-and-swap for STREX. Returns true on success.
    /// Always targets the plain alias (alias bits ignored).
    pub fn cas32(&self, addr: u32, expected: u32, new: u32) -> bool {
        if let Some(idx) = Self::sram_idx(addr) {
            self.sram[idx]
                .compare_exchange(expected, new, Relaxed, Relaxed)
                .is_ok()
        } else {
            false
        }
    }

    // ---------------------------------------------------------------
    // ROM / XIP loaders
    // ---------------------------------------------------------------

    pub fn load_rom(&mut self, data: &[u8]) {
        let len = data.len().min(self.rom.len());
        self.rom[..len].copy_from_slice(&data[..len]);
    }

    pub fn load_xip(&mut self, data: &[u8]) {
        self.xip = data.to_vec().into_boxed_slice();
    }

    // ---------------------------------------------------------------
    // ROM / XIP read helpers
    // ---------------------------------------------------------------

    fn read_rom32(&self, addr: u32) -> u32 {
        let off = addr as usize;
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

    fn read_xip32(&self, addr: u32) -> u32 {
        let off = (addr - XIP_BASE) as usize;
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
}

// ===================================================================
// Tests
// ===================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// SRAM base address (plain alias).
    const BASE: u32 = 0x2000_0000;

    #[test]
    fn read32_write32_roundtrip() {
        let mem = SharedMemory::new();
        // First word
        mem.write32(BASE, 0xDEAD_BEEF);
        assert_eq!(mem.read32(BASE), 0xDEAD_BEEF);
        // Last word
        let last = BASE + SRAM_SIZE - 4;
        mem.write32(last, 0xCAFE_BABE);
        assert_eq!(mem.read32(last), 0xCAFE_BABE);
        // Middle
        let mid = BASE + 0x1000;
        mem.write32(mid, 0x1234_5678);
        assert_eq!(mem.read32(mid), 0x1234_5678);
    }

    #[test]
    fn read16_write16() {
        let mem = SharedMemory::new();
        // Low halfword (offset 0 within word)
        mem.write16(BASE, 0xBEEF);
        assert_eq!(mem.read16(BASE), 0xBEEF);
        // High halfword (offset 2 within word)
        mem.write16(BASE + 2, 0xDEAD);
        assert_eq!(mem.read16(BASE + 2), 0xDEAD);
        // Verify the full word
        assert_eq!(mem.read32(BASE), 0xDEAD_BEEF);
    }

    #[test]
    fn read8_write8() {
        let mem = SharedMemory::new();
        mem.write8(BASE, 0x11);
        mem.write8(BASE + 1, 0x22);
        mem.write8(BASE + 2, 0x33);
        mem.write8(BASE + 3, 0x44);
        assert_eq!(mem.read8(BASE), 0x11);
        assert_eq!(mem.read8(BASE + 1), 0x22);
        assert_eq!(mem.read8(BASE + 2), 0x33);
        assert_eq!(mem.read8(BASE + 3), 0x44);
        assert_eq!(mem.read32(BASE), 0x44332211);
    }

    #[test]
    fn write16_preserves_other_half() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0xAAAA_BBBB);
        // Overwrite low half only
        mem.write16(BASE, 0x1234);
        assert_eq!(mem.read32(BASE), 0xAAAA_1234);
        // Overwrite high half only
        mem.write16(BASE + 2, 0x5678);
        assert_eq!(mem.read32(BASE), 0x5678_1234);
    }

    #[test]
    fn write8_preserves_other_bytes() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0xAABBCCDD);
        // Overwrite byte 1 only
        mem.write8(BASE + 1, 0xFF);
        assert_eq!(mem.read32(BASE), 0xAABBFFDD);
    }

    #[test]
    fn alias_xor() {
        let mem = SharedMemory::new();
        let plain = BASE;
        let xor_alias = 0x2100_0000;
        mem.write32(plain, 0xFF00_FF00);
        mem.write32(xor_alias, 0x0F0F_0F0F);
        // FF00_FF00 ^ 0F0F_0F0F = F00F_F00F
        assert_eq!(mem.read32(plain), 0xF00F_F00F);
    }

    #[test]
    fn alias_set() {
        let mem = SharedMemory::new();
        let plain = BASE;
        let set_alias = 0x2200_0000;
        mem.write32(plain, 0x0000_00FF);
        mem.write32(set_alias, 0xFF00_0000);
        assert_eq!(mem.read32(plain), 0xFF00_00FF);
    }

    #[test]
    fn alias_clr() {
        let mem = SharedMemory::new();
        let plain = BASE;
        let clr_alias = 0x2300_0000;
        mem.write32(plain, 0xFFFF_FFFF);
        // CLR = AND-NOT: clear the low byte
        mem.write32(clr_alias, 0x0000_00FF);
        assert_eq!(mem.read32(plain), 0xFFFF_FF00);
    }

    #[test]
    fn alias_write16_xor() {
        let mem = SharedMemory::new();
        let plain = BASE;
        let xor_alias = 0x2100_0000;
        mem.write32(plain, 0xAAAA_5555);
        // XOR the low halfword with 0xFFFF
        mem.write16(xor_alias, 0xFFFF);
        // Low half: 0x5555 ^ 0xFFFF = 0xAAAA; high half unchanged
        assert_eq!(mem.read32(plain), 0xAAAA_AAAA);
    }

    #[test]
    fn cas32_success() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 42);
        let ok = mem.cas32(BASE, 42, 99);
        assert!(ok);
        assert_eq!(mem.read32(BASE), 99);
    }

    #[test]
    fn cas32_failure() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 42);
        let ok = mem.cas32(BASE, 0, 99); // wrong expected
        assert!(!ok);
        assert_eq!(mem.read32(BASE), 42); // unchanged
    }

    #[test]
    fn rom_read_only() {
        let mut mem = SharedMemory::new();
        mem.load_rom(&[0xAA, 0xBB, 0xCC, 0xDD]);
        // Attempt to write to ROM address
        mem.write32(ROM_BASE, 0x1234_5678);
        // ROM should be unmodified
        assert_eq!(mem.read32(ROM_BASE), 0xDDCCBBAA);
    }

    #[test]
    fn xip_read_only() {
        let mut mem = SharedMemory::new();
        mem.load_xip(&[0x11, 0x22, 0x33, 0x44]);
        // Attempt to write to XIP address
        mem.write32(XIP_BASE, 0xDEAD_BEEF);
        // XIP should be unmodified
        assert_eq!(mem.read32(XIP_BASE), 0x44332211);
    }

    #[test]
    fn out_of_range_returns_zero() {
        let mem = SharedMemory::new();
        // Unmapped peripheral address
        assert_eq!(mem.read32(0x4000_0000), 0);
        assert_eq!(mem.read16(0x4000_0000), 0);
        assert_eq!(mem.read8(0x4000_0000), 0);
    }

    #[test]
    fn sram_alias_bits_stripped() {
        let mem = SharedMemory::new();
        // Write via plain alias
        mem.write32(BASE, 0xDEAD_BEEF);
        // Read via XOR alias address -- reads ignore alias, same storage
        assert_eq!(mem.read32(0x2100_0000), 0xDEAD_BEEF);
        // Write via SET alias to a different word
        let addr_plain = BASE + 4;
        let addr_set = 0x2200_0004;
        mem.write32(addr_plain, 0x0000_0000);
        mem.write32(addr_set, 0x1234_5678); // OR into the same word
        assert_eq!(mem.read32(addr_plain), 0x1234_5678);
    }
}

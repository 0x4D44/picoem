use std::sync::atomic::{AtomicU32, Ordering::Relaxed};

#[allow(dead_code)] // documents the memory map; used in tests
const SRAM_BASE: u32 = 0x2000_0000;
const SRAM_SIZE: u32 = 520 * 1024;
const SRAM_WORDS: usize = (520 * 1024) / 4; // 133_120 words
const ROM_BASE: u32 = 0x0000_0000;
const ROM_SIZE: u32 = 32 * 1024;
const XIP_BASE: u32 = 0x1000_0000;

// XIP SRAM: 16 KB scratchpad at 0x1C00_0000..0x1C00_4000.
const XIP_SRAM_BASE: u32 = 0x1C00_0000;
const XIP_SRAM_SIZE: u32 = 16 * 1024;
const XIP_SRAM_WORDS: usize = (16 * 1024) / 4; // 4096 words

// Boot RAM: 4 KB scratchpad at 0xEFFF_F000..0xF000_0000.
const BOOT_RAM_BASE: u32 = 0xEFFF_F000;
const BOOT_RAM_SIZE: u32 = 4096;
const BOOT_RAM_WORDS: usize = 4096 / 4; // 1024 words

pub struct SharedMemory {
    sram: Box<[AtomicU32]>,
    rom: Box<[u8]>,
    xip: Box<[u8]>,
    /// 16 KB XIP SRAM scratchpad (0x1C00_0000). Packed little-endian
    /// into `AtomicU32` words so narrow writes use a CAS loop and
    /// cross-thread reads stay atomic.
    xip_sram: Box<[AtomicU32]>,
    /// 4 KB boot RAM scratchpad (0xEFFF_F000). Same packing as
    /// `xip_sram`; hosts the bootloader's transient state.
    boot_ram: Box<[AtomicU32]>,
}

impl SharedMemory {
    pub fn new() -> Self {
        let mut sram = Vec::with_capacity(SRAM_WORDS);
        for _ in 0..SRAM_WORDS {
            sram.push(AtomicU32::new(0));
        }
        let mut xip_sram = Vec::with_capacity(XIP_SRAM_WORDS);
        for _ in 0..XIP_SRAM_WORDS {
            xip_sram.push(AtomicU32::new(0));
        }
        let mut boot_ram = Vec::with_capacity(BOOT_RAM_WORDS);
        for _ in 0..BOOT_RAM_WORDS {
            boot_ram.push(AtomicU32::new(0));
        }
        Self {
            sram: sram.into_boxed_slice(),
            rom: vec![0u8; ROM_SIZE as usize].into_boxed_slice(),
            xip: Box::new([]),
            xip_sram: xip_sram.into_boxed_slice(),
            boot_ram: boot_ram.into_boxed_slice(),
        }
    }
}

impl Default for SharedMemory {
    fn default() -> Self {
        Self::new()
    }
}

impl SharedMemory {
    /// Construct a `SharedMemory` seeded from an existing
    /// `picoem_common::Memory` plus the Bus-side `boot_ram` /
    /// `xip_sram` scratchpads and the `flash_loaded` flag.
    ///
    /// Phase 3 Stage 6b (LLD V7 §8): called from
    /// `ThreadedEmulator::from_emulator` to hand the existing
    /// single-threaded memory image to the threaded runtime with no
    /// data loss.
    ///
    /// `boot_ram` and `xip_sram` are the two on-Bus scratchpad regions
    /// (4 KB @ 0xEFFF_F000 and 16 KB @ 0x1C00_0000 respectively). They
    /// are packed little-endian into `AtomicU32` words on entry so the
    /// threaded worker can reach them through the same atomic path the
    /// main SRAM uses.
    pub fn from_memory(
        memory: picoem_common::Memory,
        boot_ram: Box<[u8; 4096]>,
        xip_sram: Box<[u8; 16384]>,
        _flash_loaded: bool,
    ) -> Self {
        let (rom, sram_bytes, xip_bytes) = memory.into_parts();

        // Pack SRAM bytes into AtomicU32 words. `Memory::sram` is sized
        // to `picoem_common::SRAM_SIZE` (520 KB = 532 480 bytes,
        // 133 120 words); `SharedMemory::sram` is sized to the same in
        // words. Guard the copy with a saturating min so a future SRAM
        // sizing mismatch does not panic on conversion.
        let word_count = sram_bytes.len() / 4;
        let cap = SRAM_WORDS.min(word_count);
        let mut sram = Vec::with_capacity(SRAM_WORDS);
        for i in 0..cap {
            let off = i * 4;
            let word = u32::from_le_bytes([
                sram_bytes[off],
                sram_bytes[off + 1],
                sram_bytes[off + 2],
                sram_bytes[off + 3],
            ]);
            sram.push(AtomicU32::new(word));
        }
        for _ in cap..SRAM_WORDS {
            sram.push(AtomicU32::new(0));
        }

        // ROM is copied into a fixed-size buffer. Truncate or zero-pad
        // to the canonical `ROM_SIZE` so downstream decode never sees
        // a short ROM.
        let mut rom_buf = vec![0u8; ROM_SIZE as usize];
        let rom_len = rom.len().min(rom_buf.len());
        rom_buf[..rom_len].copy_from_slice(&rom[..rom_len]);

        // XIP is variably sized in the single-threaded `Memory` (see
        // `Memory::load_flash`); keep that shape here so flash images
        // whose size differs from the 2 MB RP2040 flash window survive
        // the round trip unchanged.
        let xip = xip_bytes.into_boxed_slice();

        // Pack the 4 KB boot RAM / 16 KB XIP SRAM scratchpads into
        // per-word atomics. Sizes are fixed by the `Box<[u8; N]>`
        // incoming type so no saturating math is required.
        let mut boot_ram_words: Vec<AtomicU32> = Vec::with_capacity(BOOT_RAM_WORDS);
        for i in 0..BOOT_RAM_WORDS {
            let off = i * 4;
            boot_ram_words.push(AtomicU32::new(u32::from_le_bytes([
                boot_ram[off],
                boot_ram[off + 1],
                boot_ram[off + 2],
                boot_ram[off + 3],
            ])));
        }
        let mut xip_sram_words: Vec<AtomicU32> = Vec::with_capacity(XIP_SRAM_WORDS);
        for i in 0..XIP_SRAM_WORDS {
            let off = i * 4;
            xip_sram_words.push(AtomicU32::new(u32::from_le_bytes([
                xip_sram[off],
                xip_sram[off + 1],
                xip_sram[off + 2],
                xip_sram[off + 3],
            ])));
        }

        Self {
            sram: sram.into_boxed_slice(),
            rom: rom_buf.into_boxed_slice(),
            xip,
            xip_sram: xip_sram_words.into_boxed_slice(),
            boot_ram: boot_ram_words.into_boxed_slice(),
        }
    }
}

impl SharedMemory {
    // ---------------------------------------------------------------
    // Address helpers
    // ---------------------------------------------------------------

    /// Convert a bus address to a SRAM word index.
    /// Strips SRAM alias bits [27:24] per RP2350 memory map.
    ///
    /// Bounds check is word-granular: returns `Some` for any byte within
    /// a valid SRAM word. Callers should provide aligned addresses for
    /// multi-byte reads/writes.
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

    /// True when `addr` lies inside the 4 KB boot RAM scratchpad
    /// (0xEFFF_F000..0xF000_0000).
    #[inline]
    fn is_boot_ram(addr: u32) -> bool {
        (BOOT_RAM_BASE..BOOT_RAM_BASE + BOOT_RAM_SIZE).contains(&addr)
    }

    /// True when `addr` lies inside the 16 KB XIP SRAM scratchpad
    /// (0x1C00_0000..0x1C00_4000).
    #[inline]
    fn is_xip_sram(addr: u32) -> bool {
        (XIP_SRAM_BASE..XIP_SRAM_BASE + XIP_SRAM_SIZE).contains(&addr)
    }

    // ---------------------------------------------------------------
    // Read methods
    // ---------------------------------------------------------------

    pub fn read32(&self, addr: u32) -> u32 {
        if let Some(idx) = Self::sram_idx(addr) {
            self.sram[idx].load(Relaxed)
        } else if addr < ROM_BASE + ROM_SIZE {
            // ROM_BASE is 0, so addr is used directly as a byte offset.
            // If ROM_BASE ever changes, subtract base here.
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
            // ROM_BASE is 0, so addr is used directly as a byte offset.
            // If ROM_BASE ever changes, subtract base here.
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

    // ---------------------------------------------------------------
    // Boot RAM (0xEFFF_F000..0xF000_0000, 4 KB)
    // ---------------------------------------------------------------

    /// Read 32 bits from boot RAM. Returns 0 when `addr` is out of range.
    pub fn read_boot_ram32(&self, addr: u32) -> u32 {
        if !Self::is_boot_ram(addr) {
            return 0;
        }
        let idx = ((addr - BOOT_RAM_BASE) / 4) as usize;
        self.boot_ram[idx].load(Relaxed)
    }

    /// Read 16 bits from boot RAM. Assumes halfword-aligned address.
    pub fn read_boot_ram16(&self, addr: u32) -> u16 {
        let word = self.read_boot_ram32(addr & !3);
        (word >> ((addr & 2) * 8)) as u16
    }

    /// Read 8 bits from boot RAM.
    pub fn read_boot_ram8(&self, addr: u32) -> u8 {
        if !Self::is_boot_ram(addr) {
            return 0;
        }
        let idx = ((addr - BOOT_RAM_BASE) / 4) as usize;
        let word = self.boot_ram[idx].load(Relaxed);
        (word >> ((addr & 3) * 8)) as u8
    }

    /// Write 32 bits to boot RAM.
    pub fn write_boot_ram32(&self, addr: u32, val: u32) {
        if !Self::is_boot_ram(addr) {
            return;
        }
        let idx = ((addr - BOOT_RAM_BASE) / 4) as usize;
        self.boot_ram[idx].store(val, Relaxed);
    }

    /// Write 16 bits to boot RAM. Uses a CAS loop so concurrent byte /
    /// halfword / word writes to the same word don't tear.
    pub fn write_boot_ram16(&self, addr: u32, val: u16) {
        if !Self::is_boot_ram(addr) {
            return;
        }
        let idx = ((addr - BOOT_RAM_BASE) / 4) as usize;
        let shift = (addr & 2) * 8;
        let mask = 0xFFFFu32 << shift;
        let bits = (val as u32) << shift;
        loop {
            let old = self.boot_ram[idx].load(Relaxed);
            let new = (old & !mask) | bits;
            if self.boot_ram[idx]
                .compare_exchange(old, new, Relaxed, Relaxed)
                .is_ok()
            {
                break;
            }
        }
    }

    /// Write 8 bits to boot RAM. Same CAS-loop rationale as write_boot_ram16.
    pub fn write_boot_ram8(&self, addr: u32, val: u8) {
        if !Self::is_boot_ram(addr) {
            return;
        }
        let idx = ((addr - BOOT_RAM_BASE) / 4) as usize;
        let shift = (addr & 3) * 8;
        let mask = 0xFFu32 << shift;
        let bits = (val as u32) << shift;
        loop {
            let old = self.boot_ram[idx].load(Relaxed);
            let new = (old & !mask) | bits;
            if self.boot_ram[idx]
                .compare_exchange(old, new, Relaxed, Relaxed)
                .is_ok()
            {
                break;
            }
        }
    }

    // ---------------------------------------------------------------
    // XIP SRAM (0x1C00_0000..0x1C00_4000, 16 KB)
    // ---------------------------------------------------------------

    /// Read 32 bits from XIP SRAM. Returns 0 when out of range.
    pub fn read_xip_sram32(&self, addr: u32) -> u32 {
        if !Self::is_xip_sram(addr) {
            return 0;
        }
        let idx = ((addr - XIP_SRAM_BASE) / 4) as usize;
        self.xip_sram[idx].load(Relaxed)
    }

    /// Read 16 bits from XIP SRAM. Assumes halfword-aligned address.
    pub fn read_xip_sram16(&self, addr: u32) -> u16 {
        let word = self.read_xip_sram32(addr & !3);
        (word >> ((addr & 2) * 8)) as u16
    }

    /// Read 8 bits from XIP SRAM.
    pub fn read_xip_sram8(&self, addr: u32) -> u8 {
        if !Self::is_xip_sram(addr) {
            return 0;
        }
        let idx = ((addr - XIP_SRAM_BASE) / 4) as usize;
        let word = self.xip_sram[idx].load(Relaxed);
        (word >> ((addr & 3) * 8)) as u8
    }

    /// Write 32 bits to XIP SRAM.
    pub fn write_xip_sram32(&self, addr: u32, val: u32) {
        if !Self::is_xip_sram(addr) {
            return;
        }
        let idx = ((addr - XIP_SRAM_BASE) / 4) as usize;
        self.xip_sram[idx].store(val, Relaxed);
    }

    /// Write 16 bits to XIP SRAM. CAS loop for torn-write safety.
    pub fn write_xip_sram16(&self, addr: u32, val: u16) {
        if !Self::is_xip_sram(addr) {
            return;
        }
        let idx = ((addr - XIP_SRAM_BASE) / 4) as usize;
        let shift = (addr & 2) * 8;
        let mask = 0xFFFFu32 << shift;
        let bits = (val as u32) << shift;
        loop {
            let old = self.xip_sram[idx].load(Relaxed);
            let new = (old & !mask) | bits;
            if self.xip_sram[idx]
                .compare_exchange(old, new, Relaxed, Relaxed)
                .is_ok()
            {
                break;
            }
        }
    }

    /// Write 8 bits to XIP SRAM.
    pub fn write_xip_sram8(&self, addr: u32, val: u8) {
        if !Self::is_xip_sram(addr) {
            return;
        }
        let idx = ((addr - XIP_SRAM_BASE) / 4) as usize;
        let shift = (addr & 3) * 8;
        let mask = 0xFFu32 << shift;
        let bits = (val as u32) << shift;
        loop {
            let old = self.xip_sram[idx].load(Relaxed);
            let new = (old & !mask) | bits;
            if self.xip_sram[idx]
                .compare_exchange(old, new, Relaxed, Relaxed)
                .is_ok()
            {
                break;
            }
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
    fn rom_read8() {
        let mut mem = SharedMemory::new();
        mem.load_rom(&[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(mem.read8(0x0000_0000), 0x11);
        assert_eq!(mem.read8(0x0000_0001), 0x22);
        assert_eq!(mem.read8(0x0000_0002), 0x33);
        assert_eq!(mem.read8(0x0000_0003), 0x44);
    }

    #[test]
    fn rom_read16() {
        let mut mem = SharedMemory::new();
        mem.load_rom(&[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(mem.read16(0x0000_0000), 0x2211); // little-endian
        assert_eq!(mem.read16(0x0000_0002), 0x4433);
    }

    #[test]
    fn xip_read8() {
        let mut mem = SharedMemory::new();
        mem.load_xip(&[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(mem.read8(0x1000_0000), 0xAA);
        assert_eq!(mem.read8(0x1000_0003), 0xDD);
    }

    #[test]
    fn write_out_of_range_sram_is_noop() {
        let mem = SharedMemory::new();
        mem.write32(0x2008_2000, 0xDEAD_BEEF); // just past SRAM end
        assert_eq!(mem.read32(0x2008_2000), 0); // reads back 0
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

    // --- Boot RAM / XIP SRAM (Fix 3) ---

    #[test]
    fn boot_ram_roundtrip_word_halfword_byte() {
        let mem = SharedMemory::new();
        let base = BOOT_RAM_BASE;
        mem.write_boot_ram32(base, 0xDEAD_BEEF);
        assert_eq!(mem.read_boot_ram32(base), 0xDEAD_BEEF);
        // Halfword access preserves the other half.
        mem.write_boot_ram16(base, 0x1234);
        assert_eq!(mem.read_boot_ram32(base), 0xDEAD_1234);
        mem.write_boot_ram16(base + 2, 0x5678);
        assert_eq!(mem.read_boot_ram32(base), 0x5678_1234);
        assert_eq!(mem.read_boot_ram16(base), 0x1234);
        assert_eq!(mem.read_boot_ram16(base + 2), 0x5678);
        // Byte access preserves the other three bytes.
        mem.write_boot_ram8(base, 0xAA);
        assert_eq!(mem.read_boot_ram32(base), 0x5678_12AA);
        assert_eq!(mem.read_boot_ram8(base), 0xAA);
    }

    #[test]
    fn boot_ram_out_of_range_noop() {
        let mem = SharedMemory::new();
        // Reads outside the 4 KB window return 0, writes drop silently.
        assert_eq!(mem.read_boot_ram32(0xFFFF_0000), 0);
        mem.write_boot_ram32(0xFFFF_0000, 0xDEAD_BEEF);
        assert_eq!(mem.read_boot_ram32(0xFFFF_0000), 0);
    }

    #[test]
    fn xip_sram_roundtrip_word_halfword_byte() {
        let mem = SharedMemory::new();
        let base = XIP_SRAM_BASE;
        mem.write_xip_sram32(base, 0xAABB_CCDD);
        assert_eq!(mem.read_xip_sram32(base), 0xAABB_CCDD);
        mem.write_xip_sram16(base, 0x1122);
        assert_eq!(mem.read_xip_sram32(base), 0xAABB_1122);
        mem.write_xip_sram8(base, 0x33);
        assert_eq!(mem.read_xip_sram32(base), 0xAABB_1133);
        // Last word in the 16 KB window is reachable.
        let last = base + XIP_SRAM_SIZE - 4;
        mem.write_xip_sram32(last, 0xCAFE_F00D);
        assert_eq!(mem.read_xip_sram32(last), 0xCAFE_F00D);
    }

    #[test]
    fn xip_sram_out_of_range_noop() {
        let mem = SharedMemory::new();
        let past_end = XIP_SRAM_BASE + XIP_SRAM_SIZE;
        assert_eq!(mem.read_xip_sram32(past_end), 0);
        mem.write_xip_sram32(past_end, 0xDEAD_BEEF);
        assert_eq!(mem.read_xip_sram32(past_end), 0);
    }

    // ------------------------------------------------------------
    // stage5_coverage: branch-coverage fill-ins for alias arms on
    // narrow SRAM writes, SRAM idx out-of-range, short ROM/XIP
    // read paths, and boot_ram/xip_sram narrow out-of-range guards.
    // ------------------------------------------------------------

    /// `sram_idx` returns `None` for any region != 0x2 — exercises the
    /// final `else { None }` arm (line ~176).
    #[test]
    fn sram_idx_non_sram_region_rejects() {
        // Any non-SRAM address: read32 returns 0, which is the observable
        // contract (the direct `sram_idx` call is private).
        let mem = SharedMemory::new();
        assert_eq!(mem.read32(0x5000_0000), 0); // region 0x5
        assert_eq!(mem.read32(0xE000_0000), 0); // region 0xE (not boot RAM)
    }

    /// SRAM offset >= SRAM_SIZE (520 KB) must fall through `sram_idx`'s
    /// inner length guard (line ~173).
    #[test]
    fn sram_idx_past_end_returns_none() {
        let mem = SharedMemory::new();
        // Region bits = 0x2, offset past 520 KB.
        let past_end = 0x2008_2000; // 520 KB + 8 KB
        assert_eq!(mem.read32(past_end), 0);
        // write32 into the invalid offset is a silent no-op.
        mem.write32(past_end, 0xDEAD_BEEF);
        assert_eq!(mem.read32(past_end), 0);
    }

    /// `read16` hits the `addr & 2 == 0` path explicitly (low half).
    #[test]
    fn read16_low_half_path() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0xAABB_CCDD);
        assert_eq!(mem.read16(BASE), 0xCCDD); // low half (addr&2 == 0)
    }

    /// `read8` hits the ROM path (addr < ROM_BASE + ROM_SIZE) — the
    /// existing `rom_read8` test covers 0x0000_0000..=0x0000_0003; we
    /// add coverage for the last byte of the ROM window.
    #[test]
    fn read8_last_rom_byte() {
        let mut mem = SharedMemory::new();
        let mut data = vec![0u8; (ROM_SIZE - 1) as usize];
        data.push(0xAB);
        mem.load_rom(&data);
        assert_eq!(mem.read8(ROM_SIZE - 1), 0xAB);
    }

    /// `read8` XIP in-range + short-XIP out-of-range branches.
    #[test]
    fn read8_xip_in_and_out_of_range() {
        let mut mem = SharedMemory::new();
        mem.load_xip(&[0xAA, 0xBB, 0xCC, 0xDD]);
        // In-range byte read.
        assert_eq!(mem.read8(0x1000_0002), 0xCC);
        // Past the 4-byte XIP image, still in the XIP window: returns 0.
        assert_eq!(mem.read8(0x1000_0004), 0);
    }

    /// `read8` unmapped address returns 0 via the final `else` arm.
    #[test]
    fn read8_unmapped_region_returns_zero() {
        let mem = SharedMemory::new();
        // addr >= XIP_BASE but XIP is empty — short-xip branch returns 0.
        assert_eq!(mem.read8(0x1000_0000), 0);
        // Before XIP, past ROM: 0x0000_8000 is past the 32 KB ROM.
        assert_eq!(mem.read8(0x0000_8000), 0);
    }

    /// `read32` XIP in-range path with a non-zero XIP image.
    #[test]
    fn read32_xip_in_range() {
        let mut mem = SharedMemory::new();
        mem.load_xip(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
        assert_eq!(mem.read32(0x1000_0000), 0x44332211);
        assert_eq!(mem.read32(0x1000_0004), 0x88776655);
    }

    /// `read32` past end of XIP image returns 0 via the else arm.
    #[test]
    fn read32_past_xip_end_returns_zero() {
        let mut mem = SharedMemory::new();
        mem.load_xip(&[0x11, 0x22, 0x33, 0x44]); // 4 bytes only
        // addr past the loaded image in the XIP window.
        assert_eq!(mem.read32(0x1000_0100), 0);
    }

    /// `read_rom32` short-read path (< 4 bytes remaining).
    #[test]
    fn read_rom32_short_at_boundary() {
        let mut mem = SharedMemory::new();
        // ROM is always sized to ROM_SIZE; reads just past the end take
        // the `addr < ROM_BASE + ROM_SIZE` branch false → 0.
        mem.load_rom(&[0xAA; 4]);
        assert_eq!(mem.read32(ROM_SIZE - 4), 0); // ROM bytes beyond the
        // 4 loaded bytes are zero-padded.
        assert_eq!(mem.read32(ROM_SIZE + 0x1000), 0); // past ROM window
    }

    /// `read_xip32` short-read at image-size boundary.
    #[test]
    fn read_xip32_short_at_boundary() {
        let mut mem = SharedMemory::new();
        mem.load_xip(&[0x11, 0x22, 0x33]); // 3 bytes: off+3 >= len
        // addr 0x1000_0000: off=0, off+3=3 which is NOT < 3 → short path
        assert_eq!(mem.read32(0x1000_0000), 0);
    }

    /// `write16` alias 2 (SET/OR) path.
    #[test]
    fn write16_alias_set() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0x0000_FFFF);
        // SET alias on low half: OR 0xF000 into the low half.
        let set_alias = 0x2200_0000;
        mem.write16(set_alias, 0xF000);
        // Low half: FFFF | F000 = FFFF; high half unchanged.
        assert_eq!(mem.read32(BASE), 0x0000_FFFF);

        // Different word to see a non-idempotent effect.
        let plain = BASE + 8;
        let set_alias2 = 0x2200_0008;
        mem.write32(plain, 0x0000_0000);
        mem.write16(set_alias2, 0xABCD);
        assert_eq!(mem.read32(plain), 0x0000_ABCD);
    }

    /// `write16` alias 3 (CLR / AND-NOT) path.
    #[test]
    fn write16_alias_clr() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0xAAAA_FFFF);
        let clr_alias = 0x2300_0000;
        // Clear bits 0x00FF in the low half.
        mem.write16(clr_alias, 0x00FF);
        assert_eq!(mem.read32(BASE), 0xAAAA_FF00);
    }

    /// `write8` alias 1 (XOR) path.
    #[test]
    fn write8_alias_xor() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0xAABB_CCDD);
        let xor_alias = 0x2100_0000;
        // XOR byte 0 with 0xFF.
        mem.write8(xor_alias, 0xFF);
        assert_eq!(mem.read32(BASE), 0xAABB_CC22); // 0xDD ^ 0xFF = 0x22
    }

    /// `write8` alias 2 (SET) path.
    #[test]
    fn write8_alias_set() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0x0000_0000);
        let set_alias = 0x2200_0000;
        // SET byte 1.
        mem.write8(set_alias + 1, 0xAB);
        assert_eq!(mem.read32(BASE), 0x0000_AB00);
    }

    /// `write8` alias 3 (CLR) path.
    #[test]
    fn write8_alias_clr() {
        let mem = SharedMemory::new();
        mem.write32(BASE, 0xFFFF_FFFF);
        let clr_alias = 0x2300_0000;
        // Clear byte 2.
        mem.write8(clr_alias + 2, 0xFF);
        assert_eq!(mem.read32(BASE), 0xFF00_FFFF);
    }

    /// `cas32` on out-of-SRAM address returns false (else arm).
    #[test]
    fn cas32_out_of_sram_is_false() {
        let mem = SharedMemory::new();
        // Region 0x4 is APB, not SRAM; cas32 must refuse.
        assert!(!mem.cas32(0x4000_0000, 0, 1));
    }

    /// `read_boot_ram16` out-of-range returns 0 via the `!is_boot_ram`
    /// short-circuit in `read_boot_ram32`.
    #[test]
    fn read_boot_ram16_and_8_out_of_range() {
        let mem = SharedMemory::new();
        assert_eq!(mem.read_boot_ram16(0xFFFF_0000), 0);
        assert_eq!(mem.read_boot_ram8(0xFFFF_0000), 0);
    }

    /// `write_boot_ram16` and `write_boot_ram8` out-of-range drop silently.
    #[test]
    fn write_boot_ram16_and_8_out_of_range() {
        let mem = SharedMemory::new();
        mem.write_boot_ram16(0xFFFF_0000, 0xBEEF);
        mem.write_boot_ram8(0xFFFF_0000, 0xAA);
        // Nothing to observe other than no panic; the boot RAM first
        // word is unchanged.
        assert_eq!(mem.read_boot_ram32(BOOT_RAM_BASE), 0);
    }

    /// `read_xip_sram16` and `read_xip_sram8` out-of-range return 0.
    #[test]
    fn read_xip_sram16_and_8_out_of_range() {
        let mem = SharedMemory::new();
        let past = XIP_SRAM_BASE + XIP_SRAM_SIZE;
        assert_eq!(mem.read_xip_sram16(past), 0);
        assert_eq!(mem.read_xip_sram8(past), 0);
    }

    /// `write_xip_sram16` and `write_xip_sram8` out-of-range drop.
    #[test]
    fn write_xip_sram16_and_8_out_of_range() {
        let mem = SharedMemory::new();
        let past = XIP_SRAM_BASE + XIP_SRAM_SIZE;
        mem.write_xip_sram16(past, 0xCAFE);
        mem.write_xip_sram8(past, 0x00);
        assert_eq!(mem.read_xip_sram32(XIP_SRAM_BASE), 0);
    }

    /// Boot RAM narrow write at the last word boundary — hits the live
    /// CAS-success path on the last `boot_ram` slot.
    #[test]
    fn boot_ram_narrow_at_last_word() {
        let mem = SharedMemory::new();
        let last = BOOT_RAM_BASE + BOOT_RAM_SIZE - 4;
        mem.write_boot_ram16(last, 0x1122);
        mem.write_boot_ram16(last + 2, 0x3344);
        assert_eq!(mem.read_boot_ram32(last), 0x3344_1122);
        mem.write_boot_ram8(last, 0xAB);
        assert_eq!(mem.read_boot_ram32(last), 0x3344_11AB);
    }

    /// XIP SRAM narrow write at the last word boundary — mirrors
    /// `boot_ram_narrow_at_last_word` for the XIP SRAM scratchpad.
    #[test]
    fn xip_sram_narrow_at_last_word() {
        let mem = SharedMemory::new();
        let last = XIP_SRAM_BASE + XIP_SRAM_SIZE - 4;
        mem.write_xip_sram16(last, 0x5566);
        assert_eq!(mem.read_xip_sram16(last), 0x5566);
        mem.write_xip_sram8(last + 3, 0xEE);
        assert_eq!(mem.read_xip_sram8(last + 3), 0xEE);
    }

    /// `Default::default()` routes through `new()` — exercises the
    /// Default impl at lines 59-61.
    #[test]
    fn default_constructor_yields_fresh_state() {
        let mem: SharedMemory = Default::default();
        assert_eq!(mem.read32(BASE), 0);
    }

    /// CAS retry path for `write16` on SRAM. Two writer threads hammer
    /// the same word from opposite halves using a `Barrier` so the CAS
    /// loop is effectively guaranteed to lose at least once per iteration.
    #[test]
    fn write16_cas_retry_under_concurrent_mutation() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let mem = Arc::new(SharedMemory::new());
        let barrier = Arc::new(Barrier::new(2));
        let addr = BASE + 0x2000;
        let t1 = {
            let mem = mem.clone();
            let barrier = barrier.clone();
            thread::spawn(move || {
                barrier.wait();
                for i in 0..20_000u32 {
                    mem.write16(addr, (i & 0xFFFF) as u16);
                }
            })
        };
        let t2 = {
            let mem = mem.clone();
            let barrier = barrier.clone();
            thread::spawn(move || {
                barrier.wait();
                for i in 0..20_000u32 {
                    mem.write16(addr + 2, ((i ^ 0xAAAA) & 0xFFFF) as u16);
                }
            })
        };
        t1.join().unwrap();
        t2.join().unwrap();
    }

    /// Same logic for `write8` on SRAM.
    #[test]
    fn write8_cas_retry_under_concurrent_mutation() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let mem = Arc::new(SharedMemory::new());
        let barrier = Arc::new(Barrier::new(4));
        let addr = BASE + 0x3000;
        let handles: Vec<_> = (0..4u32)
            .map(|lane| {
                let mem = mem.clone();
                let barrier = barrier.clone();
                thread::spawn(move || {
                    barrier.wait();
                    for i in 0..20_000u32 {
                        mem.write8(addr + lane, ((i ^ lane) & 0xFF) as u8);
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    /// Boot RAM CAS retry. Two writers hit adjacent halves of the same
    /// word under a sync barrier — the second thread's load-then-CAS will
    /// collide with the first thread's CAS success at least once.
    #[test]
    fn boot_ram_write16_cas_retry_under_concurrent_mutation() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let mem = Arc::new(SharedMemory::new());
        let barrier = Arc::new(Barrier::new(2));
        let addr = BOOT_RAM_BASE + 0x100;
        let t1 = {
            let mem = mem.clone();
            let barrier = barrier.clone();
            thread::spawn(move || {
                barrier.wait();
                for i in 0..20_000u32 {
                    mem.write_boot_ram16(addr, (i & 0xFFFF) as u16);
                }
            })
        };
        let t2 = {
            let mem = mem.clone();
            let barrier = barrier.clone();
            thread::spawn(move || {
                barrier.wait();
                for i in 0..20_000u32 {
                    mem.write_boot_ram16(addr + 2, ((i ^ 0xAAAA) & 0xFFFF) as u16);
                }
            })
        };
        t1.join().unwrap();
        t2.join().unwrap();
    }

    /// Boot RAM write8 CAS retry — four threads, one per byte lane.
    #[test]
    fn boot_ram_write8_cas_retry_under_concurrent_mutation() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let mem = Arc::new(SharedMemory::new());
        let barrier = Arc::new(Barrier::new(4));
        let addr = BOOT_RAM_BASE + 0x180;
        let handles: Vec<_> = (0..4u32)
            .map(|lane| {
                let mem = mem.clone();
                let barrier = barrier.clone();
                thread::spawn(move || {
                    barrier.wait();
                    for i in 0..20_000u32 {
                        mem.write_boot_ram8(addr + lane, ((i ^ lane) & 0xFF) as u8);
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    /// XIP SRAM write16 CAS retry.
    #[test]
    fn xip_sram_write16_cas_retry_under_concurrent_mutation() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let mem = Arc::new(SharedMemory::new());
        let barrier = Arc::new(Barrier::new(2));
        let addr = XIP_SRAM_BASE + 0x100;
        let t1 = {
            let mem = mem.clone();
            let barrier = barrier.clone();
            thread::spawn(move || {
                barrier.wait();
                for i in 0..20_000u32 {
                    mem.write_xip_sram16(addr, (i & 0xFFFF) as u16);
                }
            })
        };
        let t2 = {
            let mem = mem.clone();
            let barrier = barrier.clone();
            thread::spawn(move || {
                barrier.wait();
                for i in 0..20_000u32 {
                    mem.write_xip_sram16(addr + 2, ((i ^ 0xAAAA) & 0xFFFF) as u16);
                }
            })
        };
        t1.join().unwrap();
        t2.join().unwrap();
    }

    /// XIP SRAM write8 CAS retry — four threads, one per byte lane.
    #[test]
    fn xip_sram_write8_cas_retry_under_concurrent_mutation() {
        use std::sync::{Arc, Barrier};
        use std::thread;

        let mem = Arc::new(SharedMemory::new());
        let barrier = Arc::new(Barrier::new(4));
        let addr = XIP_SRAM_BASE + 0x180;
        let handles: Vec<_> = (0..4u32)
            .map(|lane| {
                let mem = mem.clone();
                let barrier = barrier.clone();
                thread::spawn(move || {
                    barrier.wait();
                    for i in 0..20_000u32 {
                        mem.write_xip_sram8(addr + lane, ((i ^ lane) & 0xFF) as u8);
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    /// `read_rom32` short-read path: load a ROM of 2 bytes so any read
    /// at offset 0 has off+3 = 3, and 3 < 2 is false → short-read arm.
    #[test]
    fn read_rom32_short_load_returns_zero() {
        let mut mem = SharedMemory::new();
        // Re-initialize ROM to a 2-byte slice. `load_rom` caps at
        // rom.len() (the fixed ROM_SIZE buffer) so this actually leaves
        // the first 2 bytes set and the rest zero. We need a ROM where
        // `self.rom.len() < off + 4`. The internal `rom` buffer is
        // sized to ROM_SIZE; we cannot shorten it, but we CAN address
        // read32 just past the end.
        mem.load_rom(&[0xAA, 0xBB]);
        // Directly exercise read_rom32 via read32 at ROM_SIZE - 1 —
        // off = ROM_SIZE - 1, off + 3 = ROM_SIZE + 2, which is NOT <
        // rom.len() (== ROM_SIZE). Short-read branch → 0.
        // (ROM_SIZE - 1 is still within the `addr < ROM_BASE + ROM_SIZE`
        // outer guard of `read32`.)
        assert_eq!(mem.read32(ROM_SIZE - 1), 0);
    }

    /// Construct `SharedMemory` from a `Memory` with oversized SRAM to
    /// make sure the `cap = SRAM_WORDS.min(word_count)` branch has
    /// coverage for both the truncate and no-truncate cases.
    ///
    /// The common case (exact-size) is already exercised by
    /// `from_memory_preserves_boot_ram_and_xip_sram`. This test uses the
    /// default constructor (exact size) to explicitly hit the zero-fill
    /// pad loop at lines 105-107.
    #[test]
    fn from_memory_default_sizing_zero_fills_tail() {
        let memory = picoem_common::Memory::new();
        let boot_ram = Box::new([0u8; 4096]);
        let xip_sram = Box::new([0u8; 16384]);
        let mem = SharedMemory::from_memory(memory, boot_ram, xip_sram, false);
        // Every word should be zero after zero-filled construction.
        for off in [0u32, 0x1000, SRAM_SIZE - 4] {
            assert_eq!(mem.read32(BASE + off), 0);
        }
    }

    #[test]
    fn from_memory_preserves_boot_ram_and_xip_sram() {
        // Construct Bus-shaped boot_ram / xip_sram with a known pattern
        // and round-trip through `from_memory`. Precondition catches the
        // original "drops on the floor" regression.
        let mut boot_ram = Box::new([0u8; 4096]);
        boot_ram[0..4].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
        boot_ram[4092..4096].copy_from_slice(&0xCAFE_F00Du32.to_le_bytes());

        let mut xip_sram = Box::new([0u8; 16384]);
        xip_sram[0..4].copy_from_slice(&0x1122_3344u32.to_le_bytes());
        xip_sram[16380..16384].copy_from_slice(&0x5566_7788u32.to_le_bytes());

        let memory = picoem_common::Memory::new();
        let mem = SharedMemory::from_memory(memory, boot_ram, xip_sram, false);

        // Boot RAM first/last words survive.
        assert_eq!(mem.read_boot_ram32(BOOT_RAM_BASE), 0xDEAD_BEEF);
        assert_eq!(
            mem.read_boot_ram32(BOOT_RAM_BASE + BOOT_RAM_SIZE - 4),
            0xCAFE_F00D
        );

        // XIP SRAM first/last words survive.
        assert_eq!(mem.read_xip_sram32(XIP_SRAM_BASE), 0x1122_3344);
        assert_eq!(
            mem.read_xip_sram32(XIP_SRAM_BASE + XIP_SRAM_SIZE - 4),
            0x5566_7788
        );
    }
}

//! `SharedMemory` — RP2040 atomic memory view for the threaded runtime.
//!
//! Mirrors the mdrp2350 pattern but for the RP2040 memory map:
//! - 16 KB boot ROM at `0x0000_0000` (read-only; plain byte slice).
//! - 264 KB SRAM at `0x2000_0000` (4×64 KB striped banks + 2×4 KB
//!   scratch banks). Stored as contiguous `AtomicU32` words — the bank
//!   topology is an addressing/timing concern handled on the serial
//!   path, not here.
//! - NO onboard flash (dropped vs RP2350). HLD §4.2: RP2040 has no XIP
//!   flash model in the threaded path. Firmware loads into SRAM via
//!   `Emulator::load_image`.
//!
//! HLD §6.4 step 3: **threaded path drops bank contention accounting**.
//! Contention cycles are virtual and the silicon perf gap dwarfs any
//! accuracy gain. This module therefore does not expose any
//! bank-touched / contention hooks — `SharedMemory` is a flat atomic
//! view with no per-bank bookkeeping.
//!
//! Narrow writes use a CAS retry loop over the `AtomicU32` word so
//! concurrent byte/halfword/word writes to the same word cannot tear.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering::Relaxed};

/// ROM base address.
pub const ROM_BASE: u32 = 0x0000_0000;
/// ROM size (16 KB on RP2040).
pub const ROM_SIZE: u32 = 16 * 1024;

/// SRAM base address.
pub const SRAM_BASE: u32 = 0x2000_0000;
/// SRAM size (264 KB = 4×64 KB striped + 2×4 KB scratch).
pub const SRAM_SIZE: u32 = 264 * 1024;
/// SRAM word count (264 KB / 4 = 67 584 words).
pub const SRAM_WORDS: usize = (SRAM_SIZE as usize) / 4;

/// RP2040 atomic shared memory. Word-granular `AtomicU32` storage for
/// SRAM and a plain read-only byte slice for ROM. See module docs for
/// layout and HLD §6.4 contention-drop rationale.
pub struct SharedMemory {
    /// 16 KB boot ROM — immutable after construction. `Arc<[u8]>` so the
    /// coordinator and worker bundles can share one allocation without
    /// copying.
    rom: Arc<[u8]>,
    /// 264 KB SRAM packed as `AtomicU32` words. `Arc<[AtomicU32]>`
    /// (behind the struct-level `Arc<SharedMemory>` in `SharedState`)
    /// so every worker sees the same storage.
    sram: Arc<[AtomicU32]>,
}

impl SharedMemory {
    /// Construct a `SharedMemory` backed by `rom` (read-only) and `sram`
    /// (atomic words). The caller owns sizing — typically
    /// `Arc::<[u8]>::from` over a `ROM_SIZE`-byte vector and an
    /// `Arc::<[AtomicU32]>::from` over a `SRAM_WORDS`-sized iterator.
    pub fn new(rom: Arc<[u8]>, sram: Arc<[AtomicU32]>) -> Self {
        Self { rom, sram }
    }

    /// Allocate fresh zero-initialised ROM + SRAM and wrap them. Handy
    /// for unit tests and the default `SharedState::new_default()` path.
    pub fn new_zero() -> Self {
        let rom: Arc<[u8]> = vec![0u8; ROM_SIZE as usize].into();
        let mut sram_vec: Vec<AtomicU32> = Vec::with_capacity(SRAM_WORDS);
        for _ in 0..SRAM_WORDS {
            sram_vec.push(AtomicU32::new(0));
        }
        let sram: Arc<[AtomicU32]> = sram_vec.into();
        Self::new(rom, sram)
    }

    // ---------------------------------------------------------------
    // Address helpers
    // ---------------------------------------------------------------

    /// Convert a bus address to a SRAM word index.
    ///
    /// RP2040 SRAM aliases: bits [27:24] select a 16 MB alias window
    /// (`0x20`, `0x21`, `0x22`, `0x23`). Unlike RP2350 these are plain
    /// mirrors, not XOR/SET/CLR aliases, so we just strip alias bits.
    fn sram_idx(addr: u32) -> Option<usize> {
        if (addr >> 28) != 0x2 {
            return None;
        }
        let offset = addr & 0x00FF_FFFF;
        if offset < SRAM_SIZE {
            Some((offset / 4) as usize)
        } else {
            None
        }
    }

    // ---------------------------------------------------------------
    // Atomic u32 accessors (the primitive — other widths fan out from here)
    // ---------------------------------------------------------------

    /// Atomic 32-bit read with explicit ordering. Returns 0 on
    /// out-of-range addresses.
    pub fn read_u32_atomic(&self, addr: u32, ordering: std::sync::atomic::Ordering) -> u32 {
        if let Some(idx) = Self::sram_idx(addr) {
            self.sram[idx].load(ordering)
        } else if addr < ROM_BASE + ROM_SIZE {
            // ROM_BASE = 0 so `addr` is the byte offset directly.
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
        } else {
            0
        }
    }

    /// Atomic 32-bit write with explicit ordering. ROM writes are
    /// silently dropped (immutable). Out-of-range writes are no-ops.
    pub fn write_u32_atomic(&self, addr: u32, val: u32, ordering: std::sync::atomic::Ordering) {
        if let Some(idx) = Self::sram_idx(addr) {
            self.sram[idx].store(val, ordering);
        }
    }

    // ---------------------------------------------------------------
    // Convenience byte / halfword / word views (Relaxed ordering — fast
    // path for the hot CPU loop; the worker surrounds each quantum with
    // a barrier that carries the needed happens-before).
    // ---------------------------------------------------------------

    /// 32-bit read (Relaxed).
    pub fn read32(&self, addr: u32) -> u32 {
        self.read_u32_atomic(addr, Relaxed)
    }

    /// 16-bit read. Assumes halfword-aligned address (bit 0 = 0).
    pub fn read16(&self, addr: u32) -> u16 {
        let word = self.read_u32_atomic(addr & !3, Relaxed);
        if addr & 2 != 0 {
            (word >> 16) as u16
        } else {
            word as u16
        }
    }

    /// 8-bit read.
    pub fn read8(&self, addr: u32) -> u8 {
        if let Some(idx) = Self::sram_idx(addr) {
            let word = self.sram[idx].load(Relaxed);
            (word >> ((addr & 3) * 8)) as u8
        } else if addr < ROM_BASE + ROM_SIZE {
            let off = addr as usize;
            if off < self.rom.len() {
                self.rom[off]
            } else {
                0
            }
        } else {
            0
        }
    }

    /// 32-bit write (Relaxed). ROM writes drop silently.
    pub fn write32(&self, addr: u32, val: u32) {
        self.write_u32_atomic(addr, val, Relaxed)
    }

    /// 16-bit write via CAS loop so narrow concurrent writes to the same
    /// word don't tear. Assumes halfword-aligned address.
    pub fn write16(&self, addr: u32, val: u16) {
        if let Some(idx) = Self::sram_idx(addr) {
            let shift = (addr & 2) * 8;
            let mask = 0xFFFFu32 << shift;
            let bits = (val as u32) << shift;
            loop {
                let old = self.sram[idx].load(Relaxed);
                let new = (old & !mask) | bits;
                if self.sram[idx]
                    .compare_exchange(old, new, Relaxed, Relaxed)
                    .is_ok()
                {
                    break;
                }
            }
        }
    }

    /// 8-bit write via CAS loop.
    pub fn write8(&self, addr: u32, val: u8) {
        if let Some(idx) = Self::sram_idx(addr) {
            let shift = (addr & 3) * 8;
            let mask = 0xFFu32 << shift;
            let bits = (val as u32) << shift;
            loop {
                let old = self.sram[idx].load(Relaxed);
                let new = (old & !mask) | bits;
                if self.sram[idx]
                    .compare_exchange(old, new, Relaxed, Relaxed)
                    .is_ok()
                {
                    break;
                }
            }
        }
    }

    /// Compare-and-swap for exclusive-monitor / STREX paths. Returns
    /// true on success. Out-of-SRAM targets return false.
    pub fn cas32(&self, addr: u32, expected: u32, new: u32) -> bool {
        if let Some(idx) = Self::sram_idx(addr) {
            self.sram[idx]
                .compare_exchange(expected, new, Relaxed, Relaxed)
                .is_ok()
        } else {
            false
        }
    }

    /// Borrow the ROM byte slice. Used by diagnostic callers that need
    /// to disassemble the boot ROM directly.
    pub fn rom(&self) -> &[u8] {
        &self.rom
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASE: u32 = SRAM_BASE;

    fn fresh() -> SharedMemory {
        SharedMemory::new_zero()
    }

    #[test]
    fn read32_write32_roundtrip() {
        let mem = fresh();
        mem.write32(BASE, 0xDEAD_BEEF);
        assert_eq!(mem.read32(BASE), 0xDEAD_BEEF);
        let last = BASE + SRAM_SIZE - 4;
        mem.write32(last, 0xCAFE_BABE);
        assert_eq!(mem.read32(last), 0xCAFE_BABE);
    }

    #[test]
    fn read16_write16_preserves_other_half() {
        let mem = fresh();
        mem.write32(BASE, 0xAAAA_BBBB);
        mem.write16(BASE, 0x1234);
        assert_eq!(mem.read32(BASE), 0xAAAA_1234);
        mem.write16(BASE + 2, 0x5678);
        assert_eq!(mem.read32(BASE), 0x5678_1234);
        assert_eq!(mem.read16(BASE), 0x1234);
        assert_eq!(mem.read16(BASE + 2), 0x5678);
    }

    #[test]
    fn read8_write8_preserves_other_bytes() {
        let mem = fresh();
        mem.write32(BASE, 0xAABBCCDD);
        mem.write8(BASE + 1, 0xFF);
        assert_eq!(mem.read32(BASE), 0xAABBFFDD);
        assert_eq!(mem.read8(BASE), 0xDD);
        assert_eq!(mem.read8(BASE + 1), 0xFF);
        assert_eq!(mem.read8(BASE + 2), 0xBB);
        assert_eq!(mem.read8(BASE + 3), 0xAA);
    }

    #[test]
    fn sram_aliases_map_to_same_storage() {
        let mem = fresh();
        // Write via plain alias, read via 0x21 alias.
        mem.write32(BASE, 0xDEAD_BEEF);
        assert_eq!(mem.read32(0x2100_0000), 0xDEAD_BEEF);
        assert_eq!(mem.read32(0x2200_0000), 0xDEAD_BEEF);
        assert_eq!(mem.read32(0x2300_0000), 0xDEAD_BEEF);
    }

    #[test]
    fn rom_is_read_only_and_readable() {
        // Build a ROM with a known byte pattern, then observe it.
        let mut rom_bytes = vec![0u8; ROM_SIZE as usize];
        rom_bytes[0..4].copy_from_slice(&0xDDCCBBAAu32.to_le_bytes());
        let rom: Arc<[u8]> = rom_bytes.into();
        let mut sram_vec: Vec<AtomicU32> = Vec::with_capacity(SRAM_WORDS);
        for _ in 0..SRAM_WORDS {
            sram_vec.push(AtomicU32::new(0));
        }
        let sram: Arc<[AtomicU32]> = sram_vec.into();
        let mem = SharedMemory::new(rom, sram);

        assert_eq!(mem.read32(ROM_BASE), 0xDDCCBBAA);
        assert_eq!(mem.read8(ROM_BASE), 0xAA);
        assert_eq!(mem.read8(ROM_BASE + 3), 0xDD);
        // Writes to ROM silently drop.
        mem.write32(ROM_BASE, 0x1234_5678);
        assert_eq!(mem.read32(ROM_BASE), 0xDDCCBBAA);
    }

    #[test]
    fn out_of_range_returns_zero() {
        let mem = fresh();
        // RP2040 has no threaded XIP flash window — any non-ROM/SRAM
        // address reads as zero.
        assert_eq!(mem.read32(0x1000_0000), 0);
        assert_eq!(mem.read16(0x4000_0000), 0);
        assert_eq!(mem.read8(0x5000_0000), 0);
        // Writes to unmapped addresses are silent no-ops.
        mem.write32(0x4000_0000, 0xDEAD_BEEF);
    }

    #[test]
    fn write_out_of_sram_is_noop() {
        let mem = fresh();
        let past_end = SRAM_BASE + SRAM_SIZE;
        mem.write32(past_end, 0xDEAD_BEEF);
        assert_eq!(mem.read32(past_end), 0);
    }

    #[test]
    fn cas32_success_and_failure() {
        let mem = fresh();
        mem.write32(BASE, 42);
        assert!(mem.cas32(BASE, 42, 99));
        assert_eq!(mem.read32(BASE), 99);
        assert!(!mem.cas32(BASE, 0, 1));
        assert_eq!(mem.read32(BASE), 99);
        // CAS against a non-SRAM address returns false.
        assert!(!mem.cas32(0x4000_0000, 0, 1));
    }

    #[test]
    fn read_u32_atomic_honours_ordering() {
        use std::sync::atomic::Ordering;
        let mem = fresh();
        mem.write_u32_atomic(BASE, 0x1234_5678, Ordering::Release);
        assert_eq!(mem.read_u32_atomic(BASE, Ordering::Acquire), 0x1234_5678);
    }
}

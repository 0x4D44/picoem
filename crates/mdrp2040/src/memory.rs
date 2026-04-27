//! RP2040-specific memory bank topology.
//!
//! The storage primitive ([`Memory`]) and its scalar accessors live in
//! [`mdpicoem_common::memory`]. This module layers the RP2040 SRAM
//! topology on top of it: four 64 KB striped banks (SRAM0-3) plus two
//! 4 KB scratch banks (SRAM4 at `0x2004_0000`, SRAM5 at `0x2004_1000`).
//!
//! Differs from RP2350 (8 striped + 2 scratch, 512 + 8 KB).

pub use mdpicoem_common::memory::Memory;

/// ROM size: 16 KB.
pub const ROM_SIZE: usize = 16 * 1024;
/// SRAM size: 264 KB (4×64 striped + 2×4 scratch).
pub const SRAM_SIZE: usize = 264 * 1024;
/// XIP flash window size: 2 MB (stock Pico flash). Mapped at
/// `0x1000_0000..0x1020_0000` with aliases at `0x1100_0000`,
/// `0x1200_0000`, `0x1300_0000` (see `bus::region1_read`).
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

/// Striped SRAM size: 256 KB (SRAM0-3 @ 64 KB each).
pub(crate) const STRIPED_END: u32 = 0x0004_0000;
/// SRAM4 scratch start (offset from SRAM base).
pub(crate) const SRAM4_START: u32 = 0x0004_0000;
/// SRAM5 scratch start (offset from SRAM base).
pub(crate) const SRAM5_START: u32 = 0x0004_1000;
/// End of SRAM5 scratch (exclusive).
pub(crate) const SRAM5_END: u32 = 0x0004_2000;

/// Returns the SRAM bank number (0-5) for a given RP2040 address.
///
/// * `SRAM0-3` are word-striped — bank = `(word_offset) % 4`.
/// * `SRAM4` covers `0x2004_0000..0x2004_1000` (4 KB scratch).
/// * `SRAM5` covers `0x2004_1000..0x2004_2000` (4 KB scratch).
///
/// Returns `None` if the offset is outside SRAM range. Accepts the full
/// address (any of the 0x20/21/22/23 aliases map to the same bank).
#[inline]
pub fn bank_for_address(addr: u32) -> Option<u8> {
    if (addr >> 28) != 0x2 {
        return None;
    }
    let offset = addr & 0x00FF_FFFF; // strip alias bits [27:24]
    if offset < STRIPED_END {
        // Striped region: 4-way word stripe.
        Some(((offset >> 2) & 3) as u8)
    } else if (SRAM4_START..SRAM5_START).contains(&offset) {
        Some(4)
    } else if (SRAM5_START..SRAM5_END).contains(&offset) {
        Some(5)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn striped_bank_wraps_4_way() {
        // Word 0 → bank 0, word 1 → bank 1, word 4 → bank 0.
        assert_eq!(bank_for_address(0x2000_0000), Some(0));
        assert_eq!(bank_for_address(0x2000_0004), Some(1));
        assert_eq!(bank_for_address(0x2000_0008), Some(2));
        assert_eq!(bank_for_address(0x2000_000C), Some(3));
        assert_eq!(bank_for_address(0x2000_0010), Some(0));
    }

    #[test]
    fn scratch_banks_4_and_5() {
        assert_eq!(bank_for_address(0x2004_0000), Some(4));
        assert_eq!(bank_for_address(0x2004_0FFF), Some(4));
        assert_eq!(bank_for_address(0x2004_1000), Some(5));
        assert_eq!(bank_for_address(0x2004_1FFF), Some(5));
    }

    #[test]
    fn aliases_map_to_same_bank() {
        // 0x20, 0x21, 0x22, 0x23 aliases all resolve to the same bank.
        assert_eq!(bank_for_address(0x2100_0008), Some(2));
        assert_eq!(bank_for_address(0x2200_0008), Some(2));
        assert_eq!(bank_for_address(0x2300_0008), Some(2));
    }

    #[test]
    fn outside_sram_returns_none() {
        assert_eq!(bank_for_address(0x1000_0000), None); // XIP
        assert_eq!(bank_for_address(0x2004_2000), None); // past scratch
        assert_eq!(bank_for_address(0x200F_FFFF), None); // way past SRAM
    }
}

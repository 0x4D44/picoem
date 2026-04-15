//! Off-chip and on-chip peripheral models.
//!
//! Two sub-categories share this module:
//!
//! * `psram` — off-chip PicoGUS v2 SPI PSRAM (lives on GPIO0..3).
//! * `watchdog_tick` / future siblings — on-chip RP2040 peripherals with
//!   a register surface that fronts `Bus::peripheral_read32` /
//!   `peripheral_write32`.
//!
//! # Inherent-methods convention (HLD V7 §5.1)
//!
//! On-chip peripherals are plain structs with inherent methods — no
//! trait. Dispatch is a match arm in [`crate::Bus::peripheral_read32`] /
//! [`crate::Bus::peripheral_write32`]; consistency is enforced by
//! review, not by a compile-time contract. Rationale:
//!
//! * No runtime polymorphism needed — the set of peripherals is closed
//!   and known at compile time.
//! * Register surfaces vary enough that a single trait shape (e.g.
//!   `read32(offset) / write32(offset, val, alias)`) forces every
//!   peripheral into the same shape whether it fits or not — peripherals
//!   with per-register alias semantics, byte/halfword side-effect
//!   registers (UART_DR), or separate IRQ-assert fan-out end up pushing
//!   arguments through the trait signature for no gain.
//! * The [`Peripheral`](mdpicoem_common::Peripheral) trait that used to
//!   live in the common crate had zero implementations workspace-wide
//!   and was deleted in this wave (HLD V7 §5.1).
//!
//! # Alias-dispatch helper
//!
//! APB peripherals on RP2040 expose four aliases of every register at
//! offsets `+0x0000` (normal), `+0x1000` (XOR), `+0x2000` (BITSET),
//! `+0x3000` (BITCLR). Peripherals with plain-storage registers route
//! through [`apply_alias_rmw`] so the alias math lives in one place.
//! Peripherals with side-effect registers (TIMER_INTR W1C, UART_DR FIFO
//! push) keep their own per-register dispatch — the helper is for the
//! common case only.
//!
//! The helper takes `alias` in the canonical 2-bit form (0..=3), matching
//! `Bus::peripheral_write32`'s normalised alias argument. Callers need
//! not re-shift into the original `0x0000 / 0x1000 / 0x2000 / 0x3000`
//! bit positions.

pub mod psram;
pub mod timer;
pub mod watchdog_tick;

/// Apply an APB alias read-modify-write onto a plain-storage register.
///
/// Alias encoding (HLD V7 §5.4), in the 2-bit normalised form used
/// throughout bus dispatch:
///
/// | `alias` | Operation    | Effect                |
/// |---------|--------------|-----------------------|
/// | `0`     | Plain write  | `*stored = value`     |
/// | `1`     | XOR          | `*stored ^= value`    |
/// | `2`     | BITSET       | `*stored \|= value`   |
/// | `3`     | BITCLR       | `*stored &= !value`   |
///
/// This matches the 2-bit form `Bus::peripheral_write32` already hands
/// to every peripheral — no per-site re-shift into the `0x0000 /
/// 0x1000 / 0x2000 / 0x3000` representation is required.
///
/// Panics on any other `alias` value — callers must supply one of the
/// four canonical alias codes.
#[inline]
pub fn apply_alias_rmw(stored: &mut u32, value: u32, alias: u32) {
    match alias {
        0 => *stored = value,
        1 => *stored ^= value,
        2 => *stored |= value,
        3 => *stored &= !value,
        _ => unreachable!("apply_alias_rmw: alias must be 0..=3, got {:#X}", alias),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alias_normal_write_replaces_stored() {
        let mut v = 0xAAAA_AAAA;
        apply_alias_rmw(&mut v, 0x1234_5678, 0);
        assert_eq!(v, 0x1234_5678);
    }

    #[test]
    fn alias_xor_toggles_matching_bits() {
        let mut v = 0xF0F0_F0F0;
        apply_alias_rmw(&mut v, 0x0F0F_0F0F, 1);
        assert_eq!(v, 0xFFFF_FFFF);
    }

    #[test]
    fn alias_bitset_ors_into_stored() {
        let mut v = 0x0000_0001;
        apply_alias_rmw(&mut v, 0x0000_0006, 2);
        assert_eq!(v, 0x0000_0007);
    }

    #[test]
    fn alias_bitclr_and_nots_from_stored() {
        let mut v = 0x0000_000F;
        apply_alias_rmw(&mut v, 0x0000_0006, 3);
        assert_eq!(v, 0x0000_0009);
    }

    #[test]
    fn alias_bitset_preserves_unmasked_bits() {
        // Regression against an early-draft bug where BITSET was
        // written as `*stored = (stored & !value) | value`, which
        // silently zeroed unmasked bits.
        let mut v = 0x1234_5678;
        apply_alias_rmw(&mut v, 0x0000_0F00, 2);
        assert_eq!(v, 0x1234_5F78);
    }

    #[test]
    fn alias_bitclr_preserves_unmasked_bits() {
        let mut v = 0xFFFF_FFFF;
        apply_alias_rmw(&mut v, 0x0000_00FF, 3);
        assert_eq!(v, 0xFFFF_FF00);
    }

    #[test]
    #[should_panic(expected = "apply_alias_rmw")]
    fn alias_out_of_range_panics() {
        let mut v = 0;
        apply_alias_rmw(&mut v, 0, 4);
    }
}

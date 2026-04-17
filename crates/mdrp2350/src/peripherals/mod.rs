//! RP2350 on-chip peripheral models.
//!
//! # Inherent-methods convention (HLD V5 §5.1, mirrors V7 §5.1)
//!
//! On-chip peripherals are plain structs with inherent methods — no
//! trait. Dispatch is a match arm in [`crate::Bus::read32`] /
//! [`crate::Bus::write32`]; consistency is enforced by review, not by
//! a compile-time contract. Rationale tracks the V7 RP2040 peripherals
//! module — register surfaces vary enough that a single trait shape
//! forces every peripheral into the same signature whether it fits or
//! not.
//!
//! # Alias-dispatch helper
//!
//! APB peripherals on RP2350 expose four aliases of every register at
//! offsets `+0x0000` (normal), `+0x1000` (XOR), `+0x2000` (BITSET),
//! `+0x3000` (BITCLR). Peripherals with plain-storage registers route
//! through [`apply_alias_rmw`] so the alias math lives in one place.
//! Peripherals with side-effect registers (TIMER INTR W1C, UART_DR
//! FIFO push) keep their own per-register dispatch.
//!
//! The helper takes `alias` in the canonical 2-bit form (0..=3), matching
//! the normalised alias argument already threaded through `Bus::write32`.

pub mod adc;
pub mod i2c;
pub mod inert;
pub mod io_bank0;
pub mod pads_bank0;
pub mod psm;
pub mod pwm;
pub mod spi;
pub mod ticks;
pub mod timer;
pub mod uart;
pub mod watchdog;

#[cfg(test)]
mod phase1_tests;

#[cfg(test)]
mod phase2_tests;

/// Apply an APB alias read-modify-write onto a plain-storage register.
///
/// Alias encoding (HLD V5 §5.4), in the 2-bit normalised form used
/// throughout bus dispatch:
///
/// | `alias` | Operation    | Effect                |
/// |---------|--------------|-----------------------|
/// | `0`     | Plain write  | `*stored = value`     |
/// | `1`     | XOR          | `*stored ^= value`    |
/// | `2`     | BITSET       | `*stored \|= value`   |
/// | `3`     | BITCLR       | `*stored &= !value`   |
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
    #[should_panic(expected = "apply_alias_rmw")]
    fn alias_out_of_range_panics() {
        let mut v = 0;
        apply_alias_rmw(&mut v, 0, 4);
    }
}

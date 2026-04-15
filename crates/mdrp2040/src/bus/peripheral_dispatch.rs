//! Bus-level peripheral dispatch helpers.
//!
//! HLD V7 §5.3 folds the RESETS check into the bus layer rather than
//! having every peripheral implement `if held { return 0 }` at the top
//! of `read32` / `write32`. One table ([`BASE_RESET_MAP`]) lists every
//! peripheral's base address + RESETS bit; [`is_held_in_reset`] looks
//! up the table and consults [`super::Resets::is_held`]. Dispatch in
//! [`super::Bus::peripheral_read32`] / `peripheral_write32` short-
//! circuits before ever routing to the peripheral module.
//!
//! Bit numbering follows RP2040 datasheet §2.14 Table 26. TIMER (bit 21)
//! and WATCHDOG (bit 24) have **separate** RESETS bits and are released
//! independently. The 1 µs cadence coupling — WATCHDOG_TICK driving
//! TIMER's microsecond heartbeat — is a runtime signalling relationship,
//! not a RESETS-level one: firmware releases TIMER and WATCHDOG with two
//! distinct CLR writes. Phase 3 adds ADC (bit 0) and PWM (bit 14).

use super::{
    ADC_BASE, Bus, DMA_BASE, I2C0_BASE, I2C1_BASE, PWM_BASE, SPI0_BASE, SPI1_BASE, TIMER_BASE,
    UART0_BASE, UART1_BASE, WATCHDOG_BASE,
};

/// RESETS bit for the ADC peripheral (RP2040 datasheet §2.14 Table 26).
pub const RESET_ADC: u8 = 0;
/// RESETS bit for the DMA peripheral (RP2040 datasheet §2.14 Table 26).
/// Phase 4 (HLD V7 §5.6) — bit 2 gates the entire 12-channel DMA window.
pub const RESET_DMA: u8 = 2;
/// RESETS bit for I2C0 (RP2040 datasheet §2.14 Table 26).
pub const RESET_I2C0: u8 = 3;
/// RESETS bit for I2C1.
pub const RESET_I2C1: u8 = 4;
/// RESETS bit for the PWM peripheral.
pub const RESET_PWM: u8 = 14;
/// RESETS bit for SPI0.
pub const RESET_SPI0: u8 = 16;
/// RESETS bit for SPI1.
pub const RESET_SPI1: u8 = 17;
/// RESETS bit for the TIMER peripheral.
pub const RESET_TIMER: u8 = 21;
/// RESETS bit for UART0.
pub const RESET_UART0: u8 = 22;
/// RESETS bit for UART1.
pub const RESET_UART1: u8 = 23;

/// RESETS bit for the watchdog + its tick divider (RP2040 datasheet §2.14
/// Table 26). Independent of [`RESET_TIMER`] — the 1 µs cadence coupling
/// between WATCHDOG_TICK and TIMER is a runtime signalling relationship,
/// not a reset-bit one.
pub const RESET_WATCHDOG: u8 = 24;

/// Map from peripheral base address to the RESETS bit that gates it.
///
/// Entries are added per-phase as peripherals are implemented. Phase 1
/// carries TIMER and WATCHDOG; Phase 2 adds UART0 / UART1 / SPI0 /
/// SPI1 / I2C0 / I2C1; Phase 3 adds ADC (bit 0) / PWM (bit 14); Phase
/// 4 adds DMA.
///
/// A peripheral absent from this table is NOT reset-gated at the Bus
/// level — either because it has no reset bit (SIO, PPB, XIP_CTRL,
/// memory) or because RESETS routing for it hasn't landed yet.
pub static BASE_RESET_MAP: &[(u32, u8)] = &[
    (ADC_BASE, RESET_ADC),
    (DMA_BASE, RESET_DMA),
    (I2C0_BASE, RESET_I2C0),
    (I2C1_BASE, RESET_I2C1),
    (PWM_BASE, RESET_PWM),
    (SPI0_BASE, RESET_SPI0),
    (SPI1_BASE, RESET_SPI1),
    (TIMER_BASE, RESET_TIMER),
    (UART0_BASE, RESET_UART0),
    (UART1_BASE, RESET_UART1),
    (WATCHDOG_BASE, RESET_WATCHDOG),
];

/// True iff the peripheral at `base` is currently held in RESETS.
///
/// Dispatch in [`super::Bus::peripheral_read32`] /
/// [`super::Bus::peripheral_write32`] calls this and, if it returns
/// `true`, returns 0 (read) or no-ops (write) without routing to the
/// peripheral module. Peripherals therefore never need to handle the
/// held-in-reset case themselves.
///
/// `base` is the 4 KB-aligned peripheral base address (alias bits
/// stripped, already canonicalised by the bus dispatch). Unknown bases
/// return `false` so non-reset-gated regions fall through to the
/// normal match arm.
#[inline]
pub(crate) fn is_held_in_reset(bus: &Bus, base: u32) -> bool {
    for &(b, bit) in BASE_RESET_MAP {
        if b == base {
            return bus.resets.is_held(bit);
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::{Bus, TIMER_BASE, WATCHDOG_BASE};

    #[test]
    fn fresh_bus_holds_timer_in_reset() {
        let bus = Bus::new();
        // Default RESETS state holds everything.
        assert!(is_held_in_reset(&bus, TIMER_BASE));
        assert!(is_held_in_reset(&bus, WATCHDOG_BASE));
    }

    #[test]
    fn timer_and_watchdog_have_separate_reset_bits() {
        // Regression: RP2040 datasheet §2.14 Table 26 assigns TIMER to
        // bit 21 and WATCHDOG to bit 24. Earlier drafts incorrectly
        // gated TIMER on bit 24 (WATCHDOG_TICK's 1 µs cadence coupling
        // is runtime-level, not reset-level). Releasing one must not
        // release the other.

        // Release only bit 21 (TIMER): TIMER unblocks, WATCHDOG still held.
        let mut bus = Bus::new();
        bus.write32(0x4000_F000, 1u32 << RESET_TIMER);
        assert!(!is_held_in_reset(&bus, TIMER_BASE));
        assert!(is_held_in_reset(&bus, WATCHDOG_BASE));

        // Release only bit 24 (WATCHDOG): WATCHDOG unblocks, TIMER still held.
        let mut bus = Bus::new();
        bus.write32(0x4000_F000, 1u32 << RESET_WATCHDOG);
        assert!(is_held_in_reset(&bus, TIMER_BASE));
        assert!(!is_held_in_reset(&bus, WATCHDOG_BASE));
    }

    #[test]
    fn non_gated_base_returns_false() {
        let bus = Bus::new();
        // CLOCKS_BASE isn't in the map — not reset-gated at the bus level.
        assert!(!is_held_in_reset(&bus, 0x4000_8000));
    }

    #[test]
    fn adc_and_pwm_have_distinct_reset_bits() {
        // RP2040 datasheet §2.14 Table 26: ADC = bit 0, PWM = bit 14.
        // Releasing one must not release the other.
        let mut bus = Bus::new();
        bus.write32(0x4000_F000, 1u32 << RESET_ADC);
        assert!(!is_held_in_reset(&bus, ADC_BASE));
        assert!(is_held_in_reset(&bus, PWM_BASE));

        let mut bus = Bus::new();
        bus.write32(0x4000_F000, 1u32 << RESET_PWM);
        assert!(is_held_in_reset(&bus, ADC_BASE));
        assert!(!is_held_in_reset(&bus, PWM_BASE));
    }
}

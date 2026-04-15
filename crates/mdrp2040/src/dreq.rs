//! RP2040 DREQ (data request) constants — Phase 4.
//!
//! Source: RP2040 datasheet §2.5.3.1 Table 120. 40 numbered DREQ sources
//! plus a sentinel `FORCE` value (`0x3F`) that bypasses the DREQ matrix.
//! The `CTRL.TREQ_SEL` field is 6 bits so every value here fits.
//!
//! Also pinned in
//! `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V7.md`
//! Appendix C.
//!
//! Constants are `u8` rather than `u32` — they index a 64-bit bitmap built
//! by [`crate::bus::Bus::collect_dreqs`] and are compared directly against
//! the 6-bit `TREQ_SEL` field.

/// PIO0 SM0 TX FIFO.
pub const DREQ_PIO0_TX0: u8 = 0;
/// PIO0 SM1 TX FIFO.
pub const DREQ_PIO0_TX1: u8 = 1;
/// PIO0 SM2 TX FIFO.
pub const DREQ_PIO0_TX2: u8 = 2;
/// PIO0 SM3 TX FIFO.
pub const DREQ_PIO0_TX3: u8 = 3;

/// PIO0 SM0 RX FIFO.
pub const DREQ_PIO0_RX0: u8 = 4;
/// PIO0 SM1 RX FIFO.
pub const DREQ_PIO0_RX1: u8 = 5;
/// PIO0 SM2 RX FIFO.
pub const DREQ_PIO0_RX2: u8 = 6;
/// PIO0 SM3 RX FIFO.
pub const DREQ_PIO0_RX3: u8 = 7;

/// PIO1 SM0 TX FIFO.
pub const DREQ_PIO1_TX0: u8 = 8;
/// PIO1 SM1 TX FIFO.
pub const DREQ_PIO1_TX1: u8 = 9;
/// PIO1 SM2 TX FIFO.
pub const DREQ_PIO1_TX2: u8 = 10;
/// PIO1 SM3 TX FIFO.
pub const DREQ_PIO1_TX3: u8 = 11;

/// PIO1 SM0 RX FIFO.
pub const DREQ_PIO1_RX0: u8 = 12;
/// PIO1 SM1 RX FIFO.
pub const DREQ_PIO1_RX1: u8 = 13;
/// PIO1 SM2 RX FIFO.
pub const DREQ_PIO1_RX2: u8 = 14;
/// PIO1 SM3 RX FIFO.
pub const DREQ_PIO1_RX3: u8 = 15;

/// SPI0 TX FIFO.
pub const DREQ_SPI0_TX: u8 = 16;
/// SPI0 RX FIFO.
pub const DREQ_SPI0_RX: u8 = 17;
/// SPI1 TX FIFO.
pub const DREQ_SPI1_TX: u8 = 18;
/// SPI1 RX FIFO.
pub const DREQ_SPI1_RX: u8 = 19;

/// UART0 TX FIFO.
pub const DREQ_UART0_TX: u8 = 20;
/// UART0 RX FIFO.
pub const DREQ_UART0_RX: u8 = 21;
/// UART1 TX FIFO.
pub const DREQ_UART1_TX: u8 = 22;
/// UART1 RX FIFO.
pub const DREQ_UART1_RX: u8 = 23;

/// PWM slice 0 wrap. One-shot per wrap; not modelled in V1.
pub const DREQ_PWM_WRAP0: u8 = 24;
pub const DREQ_PWM_WRAP1: u8 = 25;
pub const DREQ_PWM_WRAP2: u8 = 26;
pub const DREQ_PWM_WRAP3: u8 = 27;
pub const DREQ_PWM_WRAP4: u8 = 28;
pub const DREQ_PWM_WRAP5: u8 = 29;
pub const DREQ_PWM_WRAP6: u8 = 30;
pub const DREQ_PWM_WRAP7: u8 = 31;

/// I2C0 TX FIFO.
pub const DREQ_I2C0_TX: u8 = 32;
/// I2C0 RX FIFO.
pub const DREQ_I2C0_RX: u8 = 33;
/// I2C1 TX FIFO.
pub const DREQ_I2C1_TX: u8 = 34;
/// I2C1 RX FIFO.
pub const DREQ_I2C1_RX: u8 = 35;

/// ADC FIFO (DREQ when FIFO level crosses threshold with `DREQ_EN`).
pub const DREQ_ADC: u8 = 36;

/// XIP stream (not modelled in V1).
pub const DREQ_XIP_STREAM: u8 = 37;
/// XIP SSITX (not modelled in V1).
pub const DREQ_XIP_SSITX: u8 = 38;
/// XIP SSIRX (not modelled in V1).
pub const DREQ_XIP_SSIRX: u8 = 39;

/// FORCE — `CTRL.TREQ_SEL == 63` bypasses the DREQ matrix and always
/// runs. Used for pure memory-to-memory transfers (`hello_dma`).
pub const DREQ_FORCE: u8 = 63;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dreq_numbering_matches_datasheet() {
        // Spot-checks against RP2040 datasheet §2.5.3.1 Table 120.
        assert_eq!(DREQ_PIO0_TX0, 0);
        assert_eq!(DREQ_PIO0_RX0, 4);
        assert_eq!(DREQ_PIO1_TX0, 8);
        assert_eq!(DREQ_PIO1_RX0, 12);
        assert_eq!(DREQ_SPI0_TX, 16);
        assert_eq!(DREQ_UART0_TX, 20);
        assert_eq!(DREQ_PWM_WRAP0, 24);
        assert_eq!(DREQ_I2C0_TX, 32);
        assert_eq!(DREQ_ADC, 36);
        assert_eq!(DREQ_FORCE, 63);
    }

    #[test]
    fn dreq_force_fits_six_bit_treq_sel() {
        // CTRL.TREQ_SEL is 6 bits (bits [20:15]); 63 is the maximum.
        assert!(DREQ_FORCE <= 0x3F);
    }
}

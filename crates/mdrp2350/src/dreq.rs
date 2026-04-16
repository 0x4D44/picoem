//! RP2350 DREQ (data request) constants — Phase 3.
//!
//! Source: RP2350 datasheet §12.6.4.2 Table 124. 55 numbered DREQ sources
//! plus a sentinel `FORCE` value (`0x3F`) that bypasses the DREQ matrix.
//! The `CTRL.TREQ_SEL` field is 6 bits so every value here fits.
//!
//! RP2350 extends the RP2040 table with PIO2 TX/RX (indices 16..23),
//! HSTX (52), CORESIGHT_TRACE (53), SHA256 (54). The SPI/UART/PWM/I2C/ADC
//! indices shift upward compared to RP2040.
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

/// PIO2 SM0 TX FIFO (RP2350-only; RP2040 has no PIO2).
pub const DREQ_PIO2_TX0: u8 = 16;
/// PIO2 SM1 TX FIFO.
pub const DREQ_PIO2_TX1: u8 = 17;
/// PIO2 SM2 TX FIFO.
pub const DREQ_PIO2_TX2: u8 = 18;
/// PIO2 SM3 TX FIFO.
pub const DREQ_PIO2_TX3: u8 = 19;

/// PIO2 SM0 RX FIFO.
pub const DREQ_PIO2_RX0: u8 = 20;
/// PIO2 SM1 RX FIFO.
pub const DREQ_PIO2_RX1: u8 = 21;
/// PIO2 SM2 RX FIFO.
pub const DREQ_PIO2_RX2: u8 = 22;
/// PIO2 SM3 RX FIFO.
pub const DREQ_PIO2_RX3: u8 = 23;

/// SPI0 TX FIFO.
pub const DREQ_SPI0_TX: u8 = 24;
/// SPI0 RX FIFO.
pub const DREQ_SPI0_RX: u8 = 25;
/// SPI1 TX FIFO (not modelled — no SPI1 peripheral in V1).
pub const DREQ_SPI1_TX: u8 = 26;
/// SPI1 RX FIFO.
pub const DREQ_SPI1_RX: u8 = 27;

/// UART0 TX FIFO.
pub const DREQ_UART0_TX: u8 = 28;
/// UART0 RX FIFO.
pub const DREQ_UART0_RX: u8 = 29;
/// UART1 TX FIFO (not modelled — no UART1 peripheral in V1).
pub const DREQ_UART1_TX: u8 = 30;
/// UART1 RX FIFO.
pub const DREQ_UART1_RX: u8 = 31;

/// PWM slice 0 wrap. One-shot per wrap; not modelled in V1.
pub const DREQ_PWM_WRAP0: u8 = 32;
pub const DREQ_PWM_WRAP1: u8 = 33;
pub const DREQ_PWM_WRAP2: u8 = 34;
pub const DREQ_PWM_WRAP3: u8 = 35;
pub const DREQ_PWM_WRAP4: u8 = 36;
pub const DREQ_PWM_WRAP5: u8 = 37;
pub const DREQ_PWM_WRAP6: u8 = 38;
pub const DREQ_PWM_WRAP7: u8 = 39;
pub const DREQ_PWM_WRAP8: u8 = 40;
pub const DREQ_PWM_WRAP9: u8 = 41;
pub const DREQ_PWM_WRAP10: u8 = 42;
pub const DREQ_PWM_WRAP11: u8 = 43;

/// I2C0 TX FIFO.
pub const DREQ_I2C0_TX: u8 = 44;
/// I2C0 RX FIFO.
pub const DREQ_I2C0_RX: u8 = 45;
/// I2C1 TX FIFO (not modelled — no I2C1 peripheral in V1).
pub const DREQ_I2C1_TX: u8 = 46;
/// I2C1 RX FIFO.
pub const DREQ_I2C1_RX: u8 = 47;

/// ADC FIFO (DREQ when FIFO level crosses threshold with `DREQ_EN`).
pub const DREQ_ADC: u8 = 48;

/// XIP stream (not modelled in V1).
pub const DREQ_XIP_STREAM: u8 = 49;
/// XIP QMITX (not modelled in V1).
pub const DREQ_XIP_QMITX: u8 = 50;
/// XIP QMIRX (not modelled in V1).
pub const DREQ_XIP_QMIRX: u8 = 51;

/// HSTX (not modelled in V1).
pub const DREQ_HSTX: u8 = 52;
/// CORESIGHT trace (not modelled in V1).
pub const DREQ_CORESIGHT: u8 = 53;
/// SHA256 (not modelled in V1).
pub const DREQ_SHA256: u8 = 54;

/// DMA internal timer 0 (`CTRL.TREQ_SEL == 59`). Rate = X/Y per
/// `DMA.TIMER0` register (bits [31:16] = X, [15:0] = Y). Accumulator
/// fires when `accum += X` overflows Y.
pub const DREQ_TIMER0: u8 = 59;
/// DMA internal timer 1 (`CTRL.TREQ_SEL == 60`).
pub const DREQ_TIMER1: u8 = 60;
/// DMA internal timer 2 (`CTRL.TREQ_SEL == 61`).
pub const DREQ_TIMER2: u8 = 61;
/// DMA internal timer 3 (`CTRL.TREQ_SEL == 62`).
pub const DREQ_TIMER3: u8 = 62;

/// FORCE — `CTRL.TREQ_SEL == 63` bypasses the DREQ matrix and always
/// runs. Used for pure memory-to-memory transfers (`hello_dma`).
pub const DREQ_FORCE: u8 = 63;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dreq_numbering_matches_rp2350_datasheet() {
        // Spot-checks against RP2350 datasheet §12.6.4.2 Table 124.
        assert_eq!(DREQ_PIO0_TX0, 0);
        assert_eq!(DREQ_PIO0_RX0, 4);
        assert_eq!(DREQ_PIO1_TX0, 8);
        assert_eq!(DREQ_PIO1_RX0, 12);
        assert_eq!(DREQ_PIO2_TX0, 16);
        assert_eq!(DREQ_PIO2_RX0, 20);
        assert_eq!(DREQ_SPI0_TX, 24);
        assert_eq!(DREQ_UART0_TX, 28);
        assert_eq!(DREQ_PWM_WRAP0, 32);
        assert_eq!(DREQ_I2C0_TX, 44);
        assert_eq!(DREQ_ADC, 48);
        assert_eq!(DREQ_HSTX, 52);
        assert_eq!(DREQ_TIMER0, 59);
        assert_eq!(DREQ_TIMER1, 60);
        assert_eq!(DREQ_TIMER2, 61);
        assert_eq!(DREQ_TIMER3, 62);
        assert_eq!(DREQ_FORCE, 63);
    }

    #[test]
    fn dreq_force_fits_six_bit_treq_sel() {
        // CTRL.TREQ_SEL is 6 bits (bits [20:15]); 63 is the maximum.
        assert!(DREQ_FORCE <= 0x3F);
    }

    #[test]
    fn all_dreq_values_fit_in_u64_bitmap() {
        // Every named DREQ constant must fit in a 64-bit bitmap.
        let all: &[u8] = &[
            DREQ_PIO0_TX0, DREQ_PIO0_TX1, DREQ_PIO0_TX2, DREQ_PIO0_TX3,
            DREQ_PIO0_RX0, DREQ_PIO0_RX1, DREQ_PIO0_RX2, DREQ_PIO0_RX3,
            DREQ_PIO1_TX0, DREQ_PIO1_TX1, DREQ_PIO1_TX2, DREQ_PIO1_TX3,
            DREQ_PIO1_RX0, DREQ_PIO1_RX1, DREQ_PIO1_RX2, DREQ_PIO1_RX3,
            DREQ_PIO2_TX0, DREQ_PIO2_TX1, DREQ_PIO2_TX2, DREQ_PIO2_TX3,
            DREQ_PIO2_RX0, DREQ_PIO2_RX1, DREQ_PIO2_RX2, DREQ_PIO2_RX3,
            DREQ_SPI0_TX, DREQ_SPI0_RX,
            DREQ_SPI1_TX, DREQ_SPI1_RX,
            DREQ_UART0_TX, DREQ_UART0_RX,
            DREQ_UART1_TX, DREQ_UART1_RX,
            DREQ_PWM_WRAP0, DREQ_PWM_WRAP1, DREQ_PWM_WRAP2, DREQ_PWM_WRAP3,
            DREQ_PWM_WRAP4, DREQ_PWM_WRAP5, DREQ_PWM_WRAP6, DREQ_PWM_WRAP7,
            DREQ_PWM_WRAP8, DREQ_PWM_WRAP9, DREQ_PWM_WRAP10, DREQ_PWM_WRAP11,
            DREQ_I2C0_TX, DREQ_I2C0_RX,
            DREQ_I2C1_TX, DREQ_I2C1_RX,
            DREQ_ADC,
            DREQ_XIP_STREAM, DREQ_XIP_QMITX, DREQ_XIP_QMIRX,
            DREQ_HSTX, DREQ_CORESIGHT, DREQ_SHA256,
            DREQ_TIMER0, DREQ_TIMER1, DREQ_TIMER2, DREQ_TIMER3,
            DREQ_FORCE,
        ];
        for &d in all {
            assert!(d < 64, "DREQ {d} does not fit in u64 bitmap");
        }
    }
}

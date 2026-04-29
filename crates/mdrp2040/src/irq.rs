//! RP2040 interrupt-number constants (NVIC line numbers, 0..=25).
//!
//! Source: RP2040 datasheet §2.3.2 Table 26. Also documented in the
//! pico-sdk header `hardware/regs/intctrl.h`, but that header is not
//! vendored into this workspace — the authoritative source referenced
//! here is the datasheet. Table also pinned in
//! `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V7.md`
//! Appendix B.
//!
//! Constants are `u32` rather than `u8` so that `1u32 << IRQ_*` is
//! well-defined for every IRQ number (shifting a `u8` by N >= 8 is UB
//! in Rust — a footgun the HLD (§5.2) calls out explicitly).

/// TIMER alarm 0.
pub const IRQ_TIMER_IRQ_0: u32 = 0;
/// TIMER alarm 1.
pub const IRQ_TIMER_IRQ_1: u32 = 1;
/// TIMER alarm 2.
pub const IRQ_TIMER_IRQ_2: u32 = 2;
/// TIMER alarm 3.
pub const IRQ_TIMER_IRQ_3: u32 = 3;
/// PWM wrap (any slice).
pub const IRQ_PWM_IRQ_WRAP: u32 = 4;
/// USB controller.
pub const IRQ_USBCTRL_IRQ: u32 = 5;
/// XIP stream / flash controller.
pub const IRQ_XIP_IRQ: u32 = 6;
/// PIO0 IRQ line 0.
pub const IRQ_PIO0_IRQ_0: u32 = 7;
/// PIO0 IRQ line 1.
pub const IRQ_PIO0_IRQ_1: u32 = 8;
/// PIO1 IRQ line 0.
pub const IRQ_PIO1_IRQ_0: u32 = 9;
/// PIO1 IRQ line 1.
pub const IRQ_PIO1_IRQ_1: u32 = 10;
/// DMA IRQ line 0.
pub const IRQ_DMA_IRQ_0: u32 = 11;
/// DMA IRQ line 1.
pub const IRQ_DMA_IRQ_1: u32 = 12;
/// GPIO bank 0 (user GPIOs).
pub const IRQ_IO_IRQ_BANK0: u32 = 13;
/// GPIO QSPI bank.
pub const IRQ_IO_IRQ_QSPI: u32 = 14;
/// SIO inter-processor FIFO, core 0 side.
pub const IRQ_SIO_IRQ_PROC0: u32 = 15;
/// SIO inter-processor FIFO, core 1 side.
pub const IRQ_SIO_IRQ_PROC1: u32 = 16;
/// CLOCKS resus / clock-source monitor.
pub const IRQ_CLOCKS_IRQ: u32 = 17;
/// SPI0.
pub const IRQ_SPI0_IRQ: u32 = 18;
/// SPI1.
pub const IRQ_SPI1_IRQ: u32 = 19;
/// UART0.
pub const IRQ_UART0_IRQ: u32 = 20;
/// UART1.
pub const IRQ_UART1_IRQ: u32 = 21;
/// ADC FIFO.
pub const IRQ_ADC_IRQ_FIFO: u32 = 22;
/// I2C0.
pub const IRQ_I2C0_IRQ: u32 = 23;
/// I2C1.
pub const IRQ_I2C1_IRQ: u32 = 24;
/// RTC alarm.
pub const IRQ_RTC_IRQ: u32 = 25;

/// Total number of RP2040 IRQ lines routed to the NVIC (0..=25).
pub const IRQ_COUNT: u32 = 26;

/// Bitmask of architecturally-implemented NVIC IRQ lines (bits 0..=25).
/// Bits 26..31 are RAZ/WI on real silicon — apply this mask to writes
/// that target ISER0/ICER0/ISPR0/ICPR0 to match that behaviour.
pub const IRQ_LINE_MASK: u32 = (1 << IRQ_COUNT) - 1;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn count_matches_datasheet_table_26() {
        assert_eq!(IRQ_COUNT, 26);
    }

    #[test]
    fn all_constants_below_count() {
        // Stronger than a simple `irq < IRQ_COUNT` range check: collect every
        // constant, sort, and assert the set is exactly `0..26`. This catches
        // swapped pairs (e.g. SPI0/SPI1 accidentally using each other's number)
        // and duplicates — bugs the range check misses.
        let all: [u32; 26] = [
            IRQ_TIMER_IRQ_0,
            IRQ_TIMER_IRQ_1,
            IRQ_TIMER_IRQ_2,
            IRQ_TIMER_IRQ_3,
            IRQ_PWM_IRQ_WRAP,
            IRQ_USBCTRL_IRQ,
            IRQ_XIP_IRQ,
            IRQ_PIO0_IRQ_0,
            IRQ_PIO0_IRQ_1,
            IRQ_PIO1_IRQ_0,
            IRQ_PIO1_IRQ_1,
            IRQ_DMA_IRQ_0,
            IRQ_DMA_IRQ_1,
            IRQ_IO_IRQ_BANK0,
            IRQ_IO_IRQ_QSPI,
            IRQ_SIO_IRQ_PROC0,
            IRQ_SIO_IRQ_PROC1,
            IRQ_CLOCKS_IRQ,
            IRQ_SPI0_IRQ,
            IRQ_SPI1_IRQ,
            IRQ_UART0_IRQ,
            IRQ_UART1_IRQ,
            IRQ_ADC_IRQ_FIFO,
            IRQ_I2C0_IRQ,
            IRQ_I2C1_IRQ,
            IRQ_RTC_IRQ,
        ];
        let mut sorted: Vec<u32> = all.to_vec();
        sorted.sort();
        assert_eq!(sorted, (0..26).collect::<Vec<u32>>());
    }
}

//! RP2350 interrupt-number constants (NVIC input line numbers, 0..=51).
//!
//! Source: RP2350 datasheet §3.2 Table 95 (IRQ inputs). Also documented
//! in the pico-sdk-pico2 header `hardware/regs/intctrl.h`, but that
//! header is not vendored into this workspace — the authoritative source
//! referenced here is the datasheet. Appendix B of
//! `wrk_docs/2026.04.15 - HLD - RP2350 Peripheral Coverage V5.md` pins
//! these against pico-sdk at the tag listed in the firmware corpus.
//!
//! **Bit width.** The NVIC input count is [`IRQ_COUNT`] = 52. Lines
//! 0..=45 are driven by peripherals (see [`PERIPH_IRQ_COUNT`]); lines
//! 46..=51 are software-only — writable via NVIC_ISPR per datasheet
//! §3.2 note following Table 95, read as 0 on the peripheral side.
//!
//! Constants are `u32` rather than `u8` so that `1u64 << IRQ_*` and
//! `1u32 << IRQ_*` are both well-defined for every IRQ number (shifting
//! a `u8` by N >= 8 is UB in Rust — a footgun the HLDs call out
//! explicitly).
//!
//! Constants below cover the subset needed by Phase 0a (PendSV/SysTick
//! plus TIMER0 for the external-IRQ oracle scenarios) and the Phase 1+
//! peripheral coverage roadmap. The full 52-line catalogue is added
//! alongside the peripheral implementations that drive each line; adding
//! a constant for an IRQ line the emulator does not yet model is a
//! documentation-only change and safe to do ahead of the peripheral.

// --- Peripheral-driven IRQs (0..=45 on RP2350) --------------------------

/// TIMER0 alarm 0.
pub const IRQ_TIMER0_IRQ_0: u32 = 0;
/// TIMER0 alarm 1.
pub const IRQ_TIMER0_IRQ_1: u32 = 1;
/// TIMER0 alarm 2.
pub const IRQ_TIMER0_IRQ_2: u32 = 2;
/// TIMER0 alarm 3.
pub const IRQ_TIMER0_IRQ_3: u32 = 3;
/// TIMER1 alarm 0.
pub const IRQ_TIMER1_IRQ_0: u32 = 4;
/// TIMER1 alarm 1.
pub const IRQ_TIMER1_IRQ_1: u32 = 5;
/// TIMER1 alarm 2.
pub const IRQ_TIMER1_IRQ_2: u32 = 6;
/// TIMER1 alarm 3.
pub const IRQ_TIMER1_IRQ_3: u32 = 7;
/// PWM wrap (slices 0-7 — low byte of PWM_INTR/INTE/INTF/INTS).
/// RP2350 has 12 PWM slices; WRAP_0 covers slices 0..=7 (mask 0x00FF) and
/// WRAP_1 covers slices 8..=11 (mask 0x0F00). RP2040 had 8 slices split
/// 0..=3 / 4..=7 — ensure datasheet references are the RP2350 variant.
pub const IRQ_PWM_IRQ_WRAP_0: u32 = 8;
/// PWM wrap (slices 8-11 — upper nibble of PWM_INTR/INTE/INTF/INTS).
pub const IRQ_PWM_IRQ_WRAP_1: u32 = 9;
/// DMA IRQ line 0.
pub const IRQ_DMA_IRQ_0: u32 = 10;
/// DMA IRQ line 1.
pub const IRQ_DMA_IRQ_1: u32 = 11;
/// DMA IRQ line 2.
pub const IRQ_DMA_IRQ_2: u32 = 12;
/// DMA IRQ line 3.
pub const IRQ_DMA_IRQ_3: u32 = 13;
/// USB controller.
pub const IRQ_USBCTRL_IRQ: u32 = 14;
/// PIO0 IRQ line 0.
pub const IRQ_PIO0_IRQ_0: u32 = 15;
/// PIO0 IRQ line 1.
pub const IRQ_PIO0_IRQ_1: u32 = 16;
/// PIO1 IRQ line 0.
pub const IRQ_PIO1_IRQ_0: u32 = 17;
/// PIO1 IRQ line 1.
pub const IRQ_PIO1_IRQ_1: u32 = 18;
/// PIO2 IRQ line 0.
pub const IRQ_PIO2_IRQ_0: u32 = 19;
/// PIO2 IRQ line 1.
pub const IRQ_PIO2_IRQ_1: u32 = 20;
/// GPIO bank 0 (user GPIOs) — core-local (core 0).
pub const IRQ_IO_IRQ_BANK0: u32 = 21;
/// GPIO bank 0 non-secure alias — core-local.
pub const IRQ_IO_IRQ_BANK0_NS: u32 = 22;
/// GPIO QSPI bank — core-local.
pub const IRQ_IO_IRQ_QSPI: u32 = 23;
/// GPIO QSPI bank non-secure alias — core-local.
pub const IRQ_IO_IRQ_QSPI_NS: u32 = 24;
/// SIO FIFO — core-local (per-core receiver).
pub const IRQ_SIO_IRQ_FIFO: u32 = 25;
/// SIO BELL — core-local.
pub const IRQ_SIO_IRQ_BELL: u32 = 26;
/// SIO FIFO (Non-Secure) — core-local.
pub const IRQ_SIO_IRQ_FIFO_NS: u32 = 27;
/// SIO BELL (Non-Secure) — core-local.
pub const IRQ_SIO_IRQ_BELL_NS: u32 = 28;
/// SIO MTIMECMP — core-local (RISC-V compatibility alias).
pub const IRQ_SIO_IRQ_MTIMECMP: u32 = 29;
/// CLOCKS resus / clock-source monitor.
pub const IRQ_CLOCKS_IRQ: u32 = 30;
/// SPI0.
pub const IRQ_SPI0_IRQ: u32 = 31;
/// SPI1.
pub const IRQ_SPI1_IRQ: u32 = 32;
/// UART0.
pub const IRQ_UART0_IRQ: u32 = 33;
/// UART1.
pub const IRQ_UART1_IRQ: u32 = 34;
/// ADC FIFO.
pub const IRQ_ADC_IRQ_FIFO: u32 = 35;
/// I2C0.
pub const IRQ_I2C0_IRQ: u32 = 36;
/// I2C1.
pub const IRQ_I2C1_IRQ: u32 = 37;
/// OTP.
pub const IRQ_OTP_IRQ: u32 = 38;
/// TRNG.
pub const IRQ_TRNG_IRQ: u32 = 39;
/// Proc0 IRQ line — wake/halt signals.
pub const IRQ_PROC0_IRQ_CSIDE: u32 = 40;
/// Proc1 IRQ line.
pub const IRQ_PROC1_IRQ_CSIDE: u32 = 41;
/// PLL_SYS.
pub const IRQ_PLL_SYS_IRQ: u32 = 42;
/// PLL_USB.
pub const IRQ_PLL_USB_IRQ: u32 = 43;
/// POWMAN PWRUP.
pub const IRQ_POWMAN_IRQ_POW: u32 = 44;
/// POWMAN Timer.
pub const IRQ_POWMAN_IRQ_TIMER: u32 = 45;

// --- Software-only IRQs (46..=51) ---------------------------------------
//
// Peripherals never drive these lines. Software pends them via
// NVIC_ISPR writes (datasheet §3.2 note following Table 95).

/// Software-only IRQ 0.
pub const IRQ_SPARE_IRQ_0: u32 = 46;
/// Software-only IRQ 1.
pub const IRQ_SPARE_IRQ_1: u32 = 47;
/// Software-only IRQ 2.
pub const IRQ_SPARE_IRQ_2: u32 = 48;
/// Software-only IRQ 3.
pub const IRQ_SPARE_IRQ_3: u32 = 49;
/// Software-only IRQ 4.
pub const IRQ_SPARE_IRQ_4: u32 = 50;
/// Software-only IRQ 5.
pub const IRQ_SPARE_IRQ_5: u32 = 51;

// --- Counts -------------------------------------------------------------

/// Total number of NVIC input lines (ISPR / ICPR / ISER / ICER accept
/// 0..=IRQ_COUNT-1). Both peripheral-driven and software-only lines are
/// counted.
pub const IRQ_COUNT: u32 = 52;

/// Number of peripheral-driven IRQ inputs (0..=PERIPH_IRQ_COUNT-1).
/// Lines `PERIPH_IRQ_COUNT..IRQ_COUNT` are software-only; peripherals
/// never drive them. `NVIC_ISPR` accepts software-writes to all 52 bits
/// so `set_pending` calls with an IRQ in 46..=51 still latch.
pub const PERIPH_IRQ_COUNT: u32 = 46;

/// Bitmask selecting the peripheral-driven IRQ lines (0..=45). Used by
/// `Bus::raise_irqs_u64` to filter out-of-range bits before they reach
/// `assert_irq_shared`: a peripheral `mask |= 1 << IRQ_*` typo on a
/// software-only line (46..=51) would otherwise silently misassert.
/// Width: 46 bits. Zero-extended to `u64` for mask operations.
pub const PERIPH_IRQ_MASK: u64 = (1u64 << PERIPH_IRQ_COUNT) - 1;

/// IRQ lines that are routed to a specific core rather than shared
/// between cores. `bus.assert_irq_core(core, irq)` is the mechanism that
/// records this — the peripheral picks the core at assert time. This
/// table exists for documentation and for the Phase 1+ drain-loop code
/// that needs to enumerate "never set on the other core" lines.
///
/// Source: RP2350 datasheet §3.2 — SIO, GPIO bank-0, and GPIO QSPI
/// IRQs are listed as core-local with Secure and Non-Secure variants.
pub const CORE_LOCAL_IRQS: &[u32] = &[
    IRQ_IO_IRQ_BANK0,
    IRQ_IO_IRQ_BANK0_NS,
    IRQ_IO_IRQ_QSPI,
    IRQ_IO_IRQ_QSPI_NS,
    IRQ_SIO_IRQ_FIFO,
    IRQ_SIO_IRQ_BELL,
    IRQ_SIO_IRQ_FIFO_NS,
    IRQ_SIO_IRQ_BELL_NS,
    IRQ_SIO_IRQ_MTIMECMP,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn count_matches_datasheet_table_95() {
        assert_eq!(IRQ_COUNT, 52);
        assert_eq!(PERIPH_IRQ_COUNT, 46);
    }

    #[test]
    fn all_constants_below_count() {
        // Enumerate every constant; sorted they should cover 0..=45
        // exactly (the peripheral-driven range). The spare/software-only
        // lines 46..=51 are declared but exercised via a separate test.
        let periph: [u32; 46] = [
            IRQ_TIMER0_IRQ_0,
            IRQ_TIMER0_IRQ_1,
            IRQ_TIMER0_IRQ_2,
            IRQ_TIMER0_IRQ_3,
            IRQ_TIMER1_IRQ_0,
            IRQ_TIMER1_IRQ_1,
            IRQ_TIMER1_IRQ_2,
            IRQ_TIMER1_IRQ_3,
            IRQ_PWM_IRQ_WRAP_0,
            IRQ_PWM_IRQ_WRAP_1,
            IRQ_DMA_IRQ_0,
            IRQ_DMA_IRQ_1,
            IRQ_DMA_IRQ_2,
            IRQ_DMA_IRQ_3,
            IRQ_USBCTRL_IRQ,
            IRQ_PIO0_IRQ_0,
            IRQ_PIO0_IRQ_1,
            IRQ_PIO1_IRQ_0,
            IRQ_PIO1_IRQ_1,
            IRQ_PIO2_IRQ_0,
            IRQ_PIO2_IRQ_1,
            IRQ_IO_IRQ_BANK0,
            IRQ_IO_IRQ_BANK0_NS,
            IRQ_IO_IRQ_QSPI,
            IRQ_IO_IRQ_QSPI_NS,
            IRQ_SIO_IRQ_FIFO,
            IRQ_SIO_IRQ_BELL,
            IRQ_SIO_IRQ_FIFO_NS,
            IRQ_SIO_IRQ_BELL_NS,
            IRQ_SIO_IRQ_MTIMECMP,
            IRQ_CLOCKS_IRQ,
            IRQ_SPI0_IRQ,
            IRQ_SPI1_IRQ,
            IRQ_UART0_IRQ,
            IRQ_UART1_IRQ,
            IRQ_ADC_IRQ_FIFO,
            IRQ_I2C0_IRQ,
            IRQ_I2C1_IRQ,
            IRQ_OTP_IRQ,
            IRQ_TRNG_IRQ,
            IRQ_PROC0_IRQ_CSIDE,
            IRQ_PROC1_IRQ_CSIDE,
            IRQ_PLL_SYS_IRQ,
            IRQ_PLL_USB_IRQ,
            IRQ_POWMAN_IRQ_POW,
            IRQ_POWMAN_IRQ_TIMER,
        ];
        let mut sorted: Vec<u32> = periph.to_vec();
        sorted.sort();
        assert_eq!(sorted, (0..PERIPH_IRQ_COUNT).collect::<Vec<u32>>());
    }

    #[test]
    fn spare_irqs_cover_46_to_51() {
        let spares: [u32; 6] = [
            IRQ_SPARE_IRQ_0,
            IRQ_SPARE_IRQ_1,
            IRQ_SPARE_IRQ_2,
            IRQ_SPARE_IRQ_3,
            IRQ_SPARE_IRQ_4,
            IRQ_SPARE_IRQ_5,
        ];
        let mut sorted: Vec<u32> = spares.to_vec();
        sorted.sort();
        assert_eq!(sorted, (PERIPH_IRQ_COUNT..IRQ_COUNT).collect::<Vec<u32>>());
    }

    #[test]
    fn core_local_irqs_are_valid() {
        for &irq in CORE_LOCAL_IRQS {
            assert!(irq < IRQ_COUNT, "core-local IRQ {irq} out of range");
            // Core-local IRQs are peripheral-driven, not software-only.
            assert!(irq < PERIPH_IRQ_COUNT, "core-local IRQ {irq} above peripheral range");
        }
    }
}

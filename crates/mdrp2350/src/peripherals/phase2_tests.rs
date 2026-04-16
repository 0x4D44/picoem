//! Phase 2 integration tests — UART0, SPI0, I2C0, ADC, PWM, IO_BANK0,
//! PADS_BANK0 exercised through the Bus public API (HLD V5 §6 row 2).
//!
//! Unit-level tests for each peripheral live alongside the peripheral
//! module; this file exercises Bus-level wiring:
//!
//! - Reset-state: all seven peripherals are released post-bootrom
//!   (V5 §5.7 table).
//! - Dispatch: read32/write32 routes to the peripheral via the APB
//!   match arm rather than falling through to the HashMap stub.
//! - Narrow-access dispatch: byte writes to UARTDR, SSPDR, IC_DATA_CMD
//!   take the narrow path (one FIFO push per byte), not word-RMW.
//! - RESETS guard: held peripherals return 0 / discard writes.
//! - `tick_peripherals` advances each peripheral that's stateful on the
//!   sys_clk.
//! - IRQ routing: unmasked raw-interrupt bits set the correct NVIC
//!   pending bit in `bus.irq_pending` via `assert_irq_shared`.

use crate::Bus;
use crate::bus::{
    RESET_ADC, RESET_I2C0, RESET_IO_BANK0, RESET_PADS_BANK0, RESET_PWM, RESET_SPI0, RESET_UART0,
};
use crate::irq::{IRQ_ADC_IRQ_FIFO, IRQ_PWM_IRQ_WRAP_0, IRQ_UART0_IRQ};
use crate::peripherals::adc::{ADC_BASE, CS as ADC_CS, CS_EN, CS_READY, FCS, FCS_EN, FCS_THRESH_SHIFT, FIFO as ADC_FIFO, INTE as ADC_INTE};
use crate::peripherals::i2c::{
    I2C0_BASE, IC_DATA_CMD, IC_ENABLE, IC_RAW_INTR_STAT, IC_TAR, IC_TX_ABRT_SOURCE, INT_TX_ABRT,
};
use crate::peripherals::io_bank0::IO_BANK0_BASE;
use crate::peripherals::pads_bank0::PADS_BANK0_BASE;
use crate::peripherals::pwm::{CSR_EN, EN as PWM_EN, INTE0, INTR as PWM_INTR, PWM_BASE};
use crate::peripherals::spi::{
    SPI0_BASE, SSPCR0, SSPCR1, SSPDR, SSPSR,
};
use crate::peripherals::uart::{
    UART0_BASE, UARTCR, UARTDR, UARTFR, UARTIMSC, UARTLCR_H, UART_INT_TX,
};

// RESETS register at 0x4002_0000 — BITSET alias @ +0x2000, BITCLR @ +0x3000.
const RESETS_BASE: u32 = 0x4002_0000;

const UARTCR_UARTEN: u32 = 1 << 0;
const UARTCR_TXE: u32 = 1 << 8;
const UARTLCR_H_FEN: u32 = 1 << 4;
const SSPCR1_SSE: u32 = 1 << 1;
const SSPCR1_LBM: u32 = 1 << 0;
const SSPSR_TFE: u32 = 1 << 0;
const CS_START_ONCE: u32 = 1 << 2;

// ----------------------------------------------------------------------------
// Reset-state table (HLD V5 §5.7): all seven peripherals released post-bootrom
// ----------------------------------------------------------------------------

#[test]
fn post_bootrom_uart0_is_released() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state & (1 << RESET_UART0), 0);
}

#[test]
fn post_bootrom_spi0_is_released() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state & (1 << RESET_SPI0), 0);
}

#[test]
fn post_bootrom_i2c0_is_released() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state & (1 << RESET_I2C0), 0);
}

#[test]
fn post_bootrom_adc_is_released() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state & (1 << RESET_ADC), 0);
}

#[test]
fn post_bootrom_pwm_is_released() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state & (1 << RESET_PWM), 0);
}

#[test]
fn post_bootrom_io_bank0_is_released() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state & (1 << RESET_IO_BANK0), 0);
}

#[test]
fn post_bootrom_pads_bank0_is_released() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state & (1 << RESET_PADS_BANK0), 0);
}

// ----------------------------------------------------------------------------
// UART0 dispatch — reset-state UARTFR reads TXFE | RXFE
// ----------------------------------------------------------------------------

#[test]
fn uart0_fr_at_reset_reports_txfe_and_rxfe() {
    let mut bus = Bus::new();
    let fr = bus.read32(UART0_BASE + UARTFR, 0);
    // CTS (bit 0) + TXFE (bit 7) + RXFE (bit 4) = 0x91 minimum.
    assert_ne!(fr & 0x80, 0, "TXFE set at reset");
    assert_ne!(fr & 0x10, 0, "RXFE set at reset");
}

#[test]
fn uart0_narrow_byte_write_to_dr_pushes_one_byte() {
    let mut bus = Bus::new();
    // Enable UART + FIFO + TXE.
    bus.write32(UART0_BASE + UARTLCR_H, UARTLCR_H_FEN, 0);
    bus.write32(UART0_BASE + UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0);
    // Byte write via narrow dispatch.
    bus.write8(UART0_BASE + UARTDR, 0x5A, 0);
    // After one byte, UARTFR.TXFE should clear, BUSY should set.
    let fr = bus.read32(UART0_BASE + UARTFR, 0);
    assert_eq!(fr & 0x80, 0, "TXFE should be 0 after one byte");
    assert_ne!(fr & 0x08, 0, "BUSY should be 1 while TX FIFO non-empty");
}

#[test]
fn uart0_irq_routed_to_bus_irq_pending() {
    let mut bus = Bus::new();
    // Force TXIS via tick (enable, set IMSC, push byte, tick).
    bus.write32(UART0_BASE + UARTLCR_H, UARTLCR_H_FEN, 0);
    bus.write32(UART0_BASE + UARTCR, UARTCR_UARTEN | UARTCR_TXE, 0);
    bus.write32(UART0_BASE + UARTIMSC, UART_INT_TX, 0);
    bus.write32(UART0_BASE + UARTDR, 0x55, 0);
    // Tick enough sys_clks to drain the byte at default (IBRD=0 → 1 cycle/byte).
    bus.tick_peripherals(100);
    for core in 0..2 {
        assert_ne!(
            bus.irq_pending[core] & (1u64 << IRQ_UART0_IRQ),
            0,
            "UART0 shared IRQ must pend on core {core}"
        );
    }
}

// ----------------------------------------------------------------------------
// SPI0 dispatch — reset-state SSPSR reports TFE
// ----------------------------------------------------------------------------

#[test]
fn spi0_sspsr_at_reset_reports_tfe() {
    let mut bus = Bus::new();
    let sr = bus.read32(SPI0_BASE + SSPSR, 0);
    assert_ne!(sr & SSPSR_TFE, 0, "TFE set at reset");
}

#[test]
fn spi0_loopback_rx_matches_tx() {
    let mut bus = Bus::new();
    // DSS=7 (8-bit), SSE=1, LBM=1.
    bus.write32(SPI0_BASE + SSPCR0, 7, 0);
    bus.write32(SPI0_BASE + SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0);
    bus.write32(SPI0_BASE + SSPDR, 0xA5, 0);
    // Narrow read via read8 must pop one byte from RX FIFO.
    let got = bus.read32(SPI0_BASE + SSPDR, 0);
    assert_eq!(got & 0xFF, 0xA5);
}

#[test]
fn spi0_byte_narrow_read_from_dr() {
    let mut bus = Bus::new();
    bus.write32(SPI0_BASE + SSPCR0, 7, 0);
    bus.write32(SPI0_BASE + SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0);
    bus.write32(SPI0_BASE + SSPDR, 0x33, 0);
    let b = bus.read8(SPI0_BASE + SSPDR, 0);
    assert_eq!(b, 0x33);
}

// ----------------------------------------------------------------------------
// I2C0 dispatch — bus_scan nack path
// ----------------------------------------------------------------------------

#[test]
fn i2c0_bus_scan_nacks_with_abrt_7b() {
    let mut bus = Bus::new();
    // Target 0x7F (I2C-reserved 7-bit address, never occupied by real
    // devices — matches `silicon_scenarios::S_I2C0_BUS_SCAN_NACK`).
    bus.write32(I2C0_BASE + IC_TAR, 0x7F, 0);
    bus.write32(I2C0_BASE + IC_ENABLE, 1, 0);
    bus.write32(I2C0_BASE + IC_DATA_CMD, (1 << 8) | (1 << 9), 0); // READ + STOP
    // Expect TX_ABRT in raw interrupt status.
    let raw = bus.read32(I2C0_BASE + IC_RAW_INTR_STAT, 0);
    assert_ne!(raw & INT_TX_ABRT, 0, "TX_ABRT must latch on NACK");
    // Abort source: 7-bit no ack.
    let src = bus.read32(I2C0_BASE + IC_TX_ABRT_SOURCE, 0);
    assert_eq!(src & 0x1, 0x1);
}

#[test]
fn i2c0_narrow_write_to_data_cmd() {
    let mut bus = Bus::new();
    bus.write32(I2C0_BASE + IC_ENABLE, 1, 0);
    // Byte write to IC_DATA_CMD should trigger one transaction, not
    // word-RMW over the register (which would double-fire via the
    // narrow dispatch miss path).
    bus.write8(I2C0_BASE + IC_DATA_CMD, 0x55, 0);
    let raw = bus.read32(I2C0_BASE + IC_RAW_INTR_STAT, 0);
    // Even one byte write latches TX_ABRT for an empty ACK list.
    assert_ne!(raw & INT_TX_ABRT, 0);
}

// ----------------------------------------------------------------------------
// ADC dispatch — CS.EN latches READY; one-shot conversion completes
// ----------------------------------------------------------------------------

#[test]
fn adc_cs_en_latches_ready() {
    let mut bus = Bus::new();
    bus.write32(ADC_BASE + ADC_CS, CS_EN, 0);
    let cs = bus.read32(ADC_BASE + ADC_CS, 0);
    assert_ne!(cs & CS_READY, 0, "CS.READY must latch after EN 0→1");
}

#[test]
fn adc_one_shot_completes_via_tick() {
    let mut bus = Bus::new();
    // Enable + one-shot on channel 3.
    bus.write32(ADC_BASE + ADC_CS, CS_EN | CS_START_ONCE | (3 << 12), 0);
    // 96 adc_clks @ 48 MHz = 96*150/48 = 300 sys_clks.
    bus.tick_peripherals(500);
    let cs = bus.read32(ADC_BASE + ADC_CS, 0);
    assert_ne!(cs & CS_READY, 0, "READY after one-shot completes");
    assert_eq!(cs & CS_START_ONCE, 0, "START_ONCE auto-clears");
}

#[test]
fn adc_fifo_irq_routed_when_unmasked() {
    let mut bus = Bus::new();
    // FIFO-enabled, THRESH=1 → any sample raises IRQ. INTE.FIFO=1.
    bus.write32(ADC_BASE + FCS, FCS_EN | (1 << FCS_THRESH_SHIFT), 0);
    bus.write32(ADC_BASE + ADC_INTE, 1, 0);
    bus.write32(ADC_BASE + ADC_CS, CS_EN | CS_START_ONCE, 0);
    bus.tick_peripherals(500);
    for core in 0..2 {
        assert_ne!(
            bus.irq_pending[core] & (1u64 << IRQ_ADC_IRQ_FIFO),
            0,
            "ADC_FIFO shared IRQ must pend on core {core}"
        );
    }
}

// ----------------------------------------------------------------------------
// PWM dispatch — wrap fires, INTE0 routes to PWM_IRQ_WRAP_0
// ----------------------------------------------------------------------------

#[test]
fn pwm_slice0_wrap_latches_intr_bit() {
    let mut bus = Bus::new();
    // Slice 0: CSR.EN, TOP=100.
    bus.write32(PWM_BASE + 0x00, CSR_EN, 0); // SLICE_CSR
    bus.write32(PWM_BASE + 0x10, 100, 0); // SLICE_TOP
    bus.write32(PWM_BASE + PWM_EN, 1, 0);
    // 101 sys_clks → one wrap.
    bus.tick_peripherals(101);
    let intr = bus.read32(PWM_BASE + PWM_INTR, 0);
    assert_eq!(intr & 0x1, 0x1);
}

#[test]
fn pwm_slice0_inte0_routes_to_wrap0_irq() {
    let mut bus = Bus::new();
    bus.write32(PWM_BASE + 0x00, CSR_EN, 0);
    bus.write32(PWM_BASE + 0x10, 50, 0);
    bus.write32(PWM_BASE + PWM_EN, 1, 0);
    bus.write32(PWM_BASE + INTE0, 1, 0);
    bus.tick_peripherals(60);
    for core in 0..2 {
        assert_ne!(
            bus.irq_pending[core] & (1u64 << IRQ_PWM_IRQ_WRAP_0),
            0,
            "PWM_IRQ_WRAP_0 must pend on core {core}"
        );
    }
}

// ----------------------------------------------------------------------------
// IO_BANK0 / PADS_BANK0 — plain storage round-trips
// ----------------------------------------------------------------------------

#[test]
fn io_bank0_gpio0_ctrl_roundtrip() {
    let mut bus = Bus::new();
    // GPIO0_CTRL at 0x04 (offset within IO_BANK0).
    bus.write32(IO_BANK0_BASE + 0x04, 0x0000_0002, 0);
    assert_eq!(bus.read32(IO_BANK0_BASE + 0x04, 0), 0x0000_0002);
}

#[test]
fn io_bank0_gpio25_ctrl_roundtrip() {
    let mut bus = Bus::new();
    // GPIO25_CTRL = 0x04 + 25*0x08 = 0xCC.
    let addr = IO_BANK0_BASE + 25 * 8 + 4;
    bus.write32(addr, 0xDEAD_BEEF, 0);
    assert_eq!(bus.read32(addr, 0), 0xDEAD_BEEF);
}

#[test]
fn pads_bank0_gpio0_roundtrip() {
    let mut bus = Bus::new();
    // GPIO0 pad at 0x04.
    bus.write32(PADS_BANK0_BASE + 0x04, 0x56, 0);
    assert_eq!(bus.read32(PADS_BANK0_BASE + 0x04, 0), 0x56);
}

#[test]
fn pads_bank0_voltage_select_roundtrip() {
    let mut bus = Bus::new();
    bus.write32(PADS_BANK0_BASE + 0x00, 1, 0);
    assert_eq!(bus.read32(PADS_BANK0_BASE + 0x00, 0), 1);
}

// ----------------------------------------------------------------------------
// RESETS guard — held peripheral returns 0 / discards writes
// ----------------------------------------------------------------------------

#[test]
fn resets_guard_held_uart0_reads_zero() {
    let mut bus = Bus::new();
    // Assert UART0 reset via BITSET alias at RESETS_BASE + 0x2000.
    bus.write32(RESETS_BASE + 0x2000, 1 << RESET_UART0, 0);
    assert_ne!(bus.resets_state & (1 << RESET_UART0), 0);
    // UARTFR should read 0 — guard intercepts before PL011 reset flags.
    assert_eq!(bus.read32(UART0_BASE + UARTFR, 0), 0);
}

#[test]
fn resets_guard_held_spi0_discards_writes() {
    let mut bus = Bus::new();
    bus.write32(RESETS_BASE + 0x2000, 1 << RESET_SPI0, 0);
    // Write SSPCR1 — must be dropped.
    bus.write32(SPI0_BASE + SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0);
    // Release the peripheral.
    bus.write32(RESETS_BASE + 0x3000, 1 << RESET_SPI0, 0);
    // CR1 should still be 0.
    assert_eq!(bus.read32(SPI0_BASE + SSPCR1, 0), 0);
}

#[test]
fn resets_guard_held_pwm_discards_tick_advance() {
    let mut bus = Bus::new();
    // Program slice 0 first while released, then hold PWM in reset.
    bus.write32(PWM_BASE + 0x00, CSR_EN, 0);
    bus.write32(PWM_BASE + 0x10, 100, 0);
    bus.write32(PWM_BASE + PWM_EN, 1, 0);
    bus.write32(RESETS_BASE + 0x2000, 1 << RESET_PWM, 0);
    bus.tick_peripherals(500);
    // Release and read CTR — must still be 0.
    bus.write32(RESETS_BASE + 0x3000, 1 << RESET_PWM, 0);
    assert_eq!(bus.read32(PWM_BASE + 0x08, 0), 0);
}

#[test]
fn resets_guard_held_io_bank0_reads_zero() {
    let mut bus = Bus::new();
    // Write a CTRL, then assert reset.
    bus.write32(IO_BANK0_BASE + 0x04, 0xABCD, 0);
    bus.write32(RESETS_BASE + 0x2000, 1 << RESET_IO_BANK0, 0);
    // Held: reads 0.
    assert_eq!(bus.read32(IO_BANK0_BASE + 0x04, 0), 0);
    // Release and re-read — stored value still present.
    bus.write32(RESETS_BASE + 0x3000, 1 << RESET_IO_BANK0, 0);
    assert_eq!(bus.read32(IO_BANK0_BASE + 0x04, 0), 0xABCD);
}

// ----------------------------------------------------------------------------
// Narrow-access ADC FIFO guard — byte/halfword writes must NOT pop the FIFO
// ----------------------------------------------------------------------------

/// Regression for code-review B1. A byte write to the ADC FIFO used to
/// take the word-RMW branch, which read `adc.read32(FIFO)` (which pops a
/// sample) then wrote back via `adc.write32(FIFO, …)` (no-op). Net
/// effect: any `str.b` to ADC FIFO silently popped one sample. The fix
/// adds ADC_BASE + FIFO to the narrow-write early-return table.
#[test]
fn adc_fifo_byte_write_does_not_pop_sample() {
    let mut bus = Bus::new();
    // FIFO-enabled, THRESH=1, one-shot on channel 3 → sample
    // `(3<<8)|0 = 0x300` lands in FIFO (non-zero so the test can
    // distinguish "pending sample" from "empty pop returns 0").
    bus.write32(ADC_BASE + FCS, FCS_EN | (1 << FCS_THRESH_SHIFT), 0);
    bus.write32(ADC_BASE + ADC_CS, CS_EN | CS_START_ONCE | (3 << 12), 0);
    bus.tick_peripherals(500);
    // Byte write to FIFO — pre-fix this popped the sample.
    bus.write8(ADC_BASE + ADC_FIFO, 0xAA, 0);
    // Full word read must still see the sample.
    let sample = bus.read32(ADC_BASE + ADC_FIFO, 0);
    assert_ne!(
        sample & 0xFFFF, 0,
        "ADC FIFO byte-write must not pop the pending sample"
    );
}

#[test]
fn adc_fifo_halfword_write_does_not_pop_sample() {
    let mut bus = Bus::new();
    bus.write32(ADC_BASE + FCS, FCS_EN | (1 << FCS_THRESH_SHIFT), 0);
    bus.write32(ADC_BASE + ADC_CS, CS_EN | CS_START_ONCE | (3 << 12), 0);
    bus.tick_peripherals(500);
    bus.write16(ADC_BASE + ADC_FIFO, 0xBEEF, 0);
    let sample = bus.read32(ADC_BASE + ADC_FIFO, 0);
    assert_ne!(
        sample & 0xFFFF, 0,
        "ADC FIFO halfword-write must not pop the pending sample"
    );
}

// ----------------------------------------------------------------------------
// raise_irqs_u64 filter — software-only IRQ bits (46..=51) must be dropped
// ----------------------------------------------------------------------------

/// Regression for code-review B2. Prior to the fix, a peripheral
/// `mask |= 1 << IRQ_*` typo on an out-of-range constant would reach
/// `assert_irq_shared` with an IRQ number in 46..=51 (software-only
/// lines) and silently misassert it. Filter at the entry point.
#[test]
fn raise_irqs_u64_drops_software_only_bits() {
    let mut bus = Bus::new();
    // Peripheral-driven bit (TIMER0_IRQ_0 = 0) plus every software-only
    // line 46..=51. The software-only bits must be filtered out.
    let mut mask = 1u64 << crate::irq::IRQ_TIMER0_IRQ_0;
    for bit in 46..=51u32 {
        mask |= 1u64 << bit;
    }
    bus.raise_irqs_u64(mask);
    for core in 0..2 {
        assert_ne!(
            bus.irq_pending[core] & (1u64 << crate::irq::IRQ_TIMER0_IRQ_0),
            0,
            "in-range IRQ_TIMER0_IRQ_0 must pend on core {core}"
        );
        for bit in 46..=51u32 {
            assert_eq!(
                bus.irq_pending[core] & (1u64 << bit),
                0,
                "software-only IRQ bit {bit} must NOT be asserted by \
                 peripheral raise on core {core}"
            );
        }
    }
}

#[test]
fn raise_irqs_u64_empty_mask_is_noop() {
    let mut bus = Bus::new();
    bus.raise_irqs_u64(0);
    assert_eq!(bus.irq_pending[0], 0);
    assert_eq!(bus.irq_pending[1], 0);
}

#[test]
fn raise_irqs_u64_all_software_only_is_noop() {
    let mut bus = Bus::new();
    let mut mask = 0u64;
    for bit in 46..=51u32 {
        mask |= 1u64 << bit;
    }
    bus.raise_irqs_u64(mask);
    assert_eq!(bus.irq_pending[0], 0);
    assert_eq!(bus.irq_pending[1], 0);
}

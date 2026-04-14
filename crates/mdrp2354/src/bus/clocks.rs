//! Clock tree model for the RP2350.
//!
//! Derived clock frequencies are recomputed eagerly whenever a
//! clock-relevant register (CLOCKS, and in later phases PLL_SYS /
//! PLL_USB) is written. The result lives in [`ClockTree`], which is
//! owned by [`crate::bus::Bus`] and read by the Pacer.
//!
//! See `wrk_docs/2026.04.14 - LLD - Clock Tree Model V2.md` §4 for
//! the full design. Phase A covers only the skeleton — ROSC / XOSC
//! sources and the `CLK_SYS_DIV` integer divider. PLL output is
//! stubbed to 0 Hz until Phase B.

/// ROSC nominal frequency (~6.5 MHz). The RP2350 boots on ROSC;
/// PLL configuration (if any) happens later in firmware.
pub const ROSC_FREQ_HZ: u32 = 6_500_000;

/// XOSC nominal frequency (12 MHz). Standard Pico SDK configuration.
pub const XOSC_FREQ_HZ: u32 = 12_000_000;

/// Derived clock tree frequencies. Recomputed eagerly whenever any
/// clock-relevant register (CLOCKS, PLL_SYS, PLL_USB) changes.
#[derive(Debug, Clone, Copy)]
pub struct ClockTree {
    /// Effective system clock in Hz. Drives the Pacer.
    pub sys_clk_hz: u32,
    /// Effective reference clock in Hz.
    pub ref_clk_hz: u32,
}

impl Default for ClockTree {
    fn default() -> Self {
        Self {
            sys_clk_hz: ROSC_FREQ_HZ,
            ref_clk_hz: ROSC_FREQ_HZ,
        }
    }
}

// Phase B will add PLL output computation. For now PLL sources return 0.
#[allow(dead_code)]
pub(crate) fn pll_output_hz(_regs: &[u32; 4]) -> u32 {
    0
}

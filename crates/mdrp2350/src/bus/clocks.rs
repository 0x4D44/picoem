//! Clock tree model for the RP2350.
//!
//! Derived clock frequencies are recomputed eagerly whenever a
//! clock-relevant register (CLOCKS, PLL_SYS, PLL_USB) is written. The
//! result lives in [`ClockTree`], which is owned by
//! [`crate::bus::Bus`] and read by the Pacer.
//!
//! See `wrk_docs/2026.04.14 - LLD - Clock Tree Model V2.md` §4 for
//! the full design. Phase A covered the CLOCKS side (ROSC / XOSC
//! sources and the `CLK_SYS_DIV` divider). Phase B adds real PLL
//! output computation from the PLL_SYS / PLL_USB register arrays.

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

/// Compute a PLL's output frequency in Hz from its four-register image.
///
/// `regs[0]` is CS (REFDIV in `[5:0]`), `regs[2]` is FBDIV_INT
/// (FBDIV in `[11:0]`), `regs[3]` is PRIM (POSTDIV1 in `[18:16]`,
/// POSTDIV2 in `[14:12]`). PWR (`regs[1]`) is accepted but unused —
/// see LLD V2 §3 note on PLL power-gating fidelity.
///
/// Returns **0** when `FBDIV == 0` (unconfigured PLL), rather than a
/// `.max(1)` hack that would silently turn an unconfigured PLL into a
/// ~244 kHz signal. The Pacer guards against 0 Hz (Phase C).
///
/// Uses u64 intermediates to avoid `u32` overflow: with REFDIV=1 and
/// FBDIV=4095, `XOSC * FBDIV = 49_140_000_000` — well outside u32.
/// The final result is clamped defensively to `u32::MAX`.
pub(crate) fn pll_output_hz(regs: &[u32; 4]) -> u32 {
    let fbdiv = (regs[2] & 0xFFF) as u64;
    if fbdiv == 0 {
        return 0;
    }
    let refdiv = ((regs[0] & 0x3F).max(1)) as u64;
    let postdiv1 = (((regs[3] >> 16) & 0x7).max(1)) as u64;
    let postdiv2 = (((regs[3] >> 12) & 0x7).max(1)) as u64;

    let vco_hz = (XOSC_FREQ_HZ as u64 / refdiv) * fbdiv;
    let out_hz_64 = vco_hz / (postdiv1 * postdiv2);
    out_hz_64.min(u32::MAX as u64) as u32
}

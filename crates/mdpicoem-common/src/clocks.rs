//! Clock tree primitives — ROSC/XOSC reference frequencies, PLL output
//! math, and the cached [`ClockTree`] result.
//!
//! Derived clock frequencies are recomputed eagerly whenever a
//! clock-relevant register (CLOCKS, PLL_SYS, PLL_USB) is written. The
//! cache lives on each chip's `Bus` and is read by the Pacer.
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
    /// Effective peripheral clock in Hz (UART / SPI / I2C source).
    /// Follows the selected `CLK_PERI_CTRL.AUXSRC` on RP2040; falls back
    /// to `sys_clk_hz` when the peripheral clock is not separately
    /// programmed (the pico-sdk default).
    pub peri_clk_hz: u32,
}

impl Default for ClockTree {
    fn default() -> Self {
        Self {
            sys_clk_hz: ROSC_FREQ_HZ,
            ref_clk_hz: ROSC_FREQ_HZ,
            peri_clk_hz: ROSC_FREQ_HZ,
        }
    }
}

impl ClockTree {
    /// Current peripheral-clock frequency in Hz. UART baud-rate
    /// divisors, SPI bit-rate prescalers, and I2C SCL generators all
    /// derive their cadence from this frequency.
    #[inline]
    pub fn peri_hz(&self) -> u64 {
        self.peri_clk_hz as u64
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
pub fn pll_output_hz(regs: &[u32; 4]) -> u32 {
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

// --- PLL LOCK modelling (shared between RP2350 / RP2040) --------------------
//
// See `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` for the full design.
// The bug being fixed: both chip crates unconditionally forced CS[31] (LOCK)
// high on every PLL CS read, regardless of power state, FBDIV configuration,
// or elapsed settle time. The silicon oracle
// (`silicon_periph_diff_rp2350 pll_sys_lock_timing`) catches this at ~1133
// sysclks after `PWR=0`, where real hardware still reports LOCK=0.
//
// Fix shape: three pure functions here (predicate, CS read, write-time
// transition), plus per-chip `Option<u64> pll_*_lock_at_cycle` storage and a
// `master_cycle` stash on each `Bus` populated at step entry. The
// `pll_should_arm_lock` helper implements **Option B** — rearm on
// FBDIV/REFDIV change while still powered — closing the fidelity gap called
// out in the HLD §3 versus Option C.

/// PLL lock-detect delay, in sysclks.
///
/// Tuned to be strictly greater than what
/// `silicon_periph_diff_rp2350 pll_sys_lock_timing` observes on real RP2354
/// silicon (1133 sysclks at ROSC boot clock). Not a fit to the datasheet's
/// LOCK_DETECT_COUNTER field — that would require a richer PWR-cycle model.
/// See `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §4.
pub const PLL_LOCK_DELAY_SYSCLKS: u64 = 2_000;

/// True iff the PLL's current register image has the VCO powered up and a
/// non-zero feedback divider — i.e. the lock-detect counter would be running
/// on real silicon.
///
/// Inspects PWR[0] (`PD`), PWR[5] (`VCOPD`), and FBDIV[11:0]. BYPASS (CS[8])
/// is intentionally *not* inspected: the conservative interpretation is that
/// bypass routes the reference clock around the VCO but does not itself
/// assert LOCK (see HLD §2 / §9).
pub fn pll_is_locked_base(regs: &[u32; 4]) -> bool {
    let pwr = regs[1];
    let fbdiv = regs[2] & 0xFFF;
    let pd = (pwr & 0x01) != 0;
    let vcopd = (pwr & 0x20) != 0;
    fbdiv != 0 && !pd && !vcopd
}

/// Read CS with the correct LOCK bit (CS[31]) given the current lock-arm
/// state and master cycle count.
///
/// LOCK reads 1 iff [`pll_is_locked_base`] is true AND the arm point has
/// elapsed. `lock_at == None` always yields LOCK=0, regardless of the
/// predicate — the arm has not yet been scheduled.
pub fn pll_cs_read_with_lock(regs: &[u32; 4], lock_at: Option<u64>, now: u64) -> u32 {
    let locked_time = matches!(lock_at, Some(arm) if now >= arm);
    let locked = pll_is_locked_base(regs) && locked_time;
    if locked {
        regs[0] | (1 << 31)
    } else {
        regs[0] & !(1 << 31)
    }
}

/// Compute the new `lock_at_cycle` value after a PLL register write.
///
/// Call this **after** the underlying `regs` have been updated with the
/// write (alias-applied, etc.). Returns the `Option<u64>` that should be
/// stored back into the chip's `pll_*_lock_at_cycle` field.
///
/// Implements HLD §4 "Option B" semantics:
///
/// - If the new state is not powered/configured → `None` (drop any arm).
/// - Else if previously un-armed → `Some(now + PLL_LOCK_DELAY_SYSCLKS)` (fresh arm).
/// - Else if REFDIV (CS[5:0]) or FBDIV (FBDIV_INT[11:0]) changed while still
///   powered → `Some(now + PLL_LOCK_DELAY_SYSCLKS)` (re-arm per silicon).
/// - Else → `prev_lock_at` unchanged.
///
/// PRIM changes (POSTDIV1/POSTDIV2) deliberately do **not** rearm: those
/// are post-VCO and have no effect on the lock-detect counter per silicon
/// behaviour.
pub fn pll_should_arm_lock(
    old_regs: &[u32; 4],
    new_regs: &[u32; 4],
    prev_lock_at: Option<u64>,
    now: u64,
) -> Option<u64> {
    if !pll_is_locked_base(new_regs) {
        None
    } else if prev_lock_at.is_none() {
        Some(now + PLL_LOCK_DELAY_SYSCLKS)
    } else if (old_regs[0] & 0x3F) != (new_regs[0] & 0x3F) {
        // REFDIV changed while still powered → re-arm.
        Some(now + PLL_LOCK_DELAY_SYSCLKS)
    } else if (old_regs[2] & 0xFFF) != (new_regs[2] & 0xFFF) {
        // FBDIV changed while still powered → re-arm.
        Some(now + PLL_LOCK_DELAY_SYSCLKS)
    } else {
        prev_lock_at
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- pll_is_locked_base -------------------------------------------------

    #[test]
    fn pll_is_locked_base_false_when_pd_set() {
        // PWR=0x01 (PD only), FBDIV=100 → predicate false despite FBDIV.
        let regs: [u32; 4] = [0x01, 0x01, 100, 0];
        assert!(!pll_is_locked_base(&regs));
    }

    #[test]
    fn pll_is_locked_base_false_when_vcopd_set() {
        // PWR=0x20 (VCOPD only), FBDIV=100 → predicate false.
        let regs: [u32; 4] = [0x01, 0x20, 100, 0];
        assert!(!pll_is_locked_base(&regs));
    }

    #[test]
    fn pll_is_locked_base_false_when_fbdiv_zero() {
        // PWR=0 (everything powered), FBDIV=0 → predicate false (unconfigured).
        let regs: [u32; 4] = [0x01, 0, 0, 0];
        assert!(!pll_is_locked_base(&regs));
    }

    #[test]
    fn pll_is_locked_base_true_when_powered_and_configured() {
        // PWR=0, FBDIV=100 → predicate true.
        let regs: [u32; 4] = [0x01, 0, 100, 0];
        assert!(pll_is_locked_base(&regs));
    }

    #[test]
    fn pll_is_locked_base_false_at_reset_defaults() {
        // Reset image: PWR=0x2D (PD+DSMPD+POSTDIVPD+VCOPD all set), FBDIV=0.
        let regs: [u32; 4] = [0x0000_0001, 0x0000_002D, 0, 0x0007_7000];
        assert!(!pll_is_locked_base(&regs));
    }

    #[test]
    fn pll_is_locked_base_ignores_bypass() {
        // CS[8]=1 (BYPASS) but PWR=0 and FBDIV=100 → still true; BYPASS is
        // orthogonal to the lock-detect predicate (see HLD §9).
        let regs: [u32; 4] = [0x101, 0, 100, 0];
        assert!(pll_is_locked_base(&regs));
    }

    // --- pll_cs_read_with_lock ---------------------------------------------

    #[test]
    fn pll_cs_read_returns_lock_set_when_locked_and_past_arm() {
        let regs: [u32; 4] = [0x01, 0, 100, 0];
        let cs = pll_cs_read_with_lock(&regs, Some(10), 1_000);
        assert_eq!(cs & (1 << 31), 1 << 31);
        assert_eq!(cs & 0x3F, 0x01, "REFDIV bits preserved under LOCK merge");
    }

    #[test]
    fn pll_cs_read_returns_lock_clear_before_arm() {
        let regs: [u32; 4] = [0x01, 0, 100, 0];
        let cs = pll_cs_read_with_lock(&regs, Some(5_000), 10);
        assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 before arm cycle");
    }

    #[test]
    fn pll_cs_read_returns_lock_clear_when_base_false() {
        // Powered-down PLL: even with a past arm, LOCK must read 0.
        let regs: [u32; 4] = [0x01, 0x2D, 100, 0];
        let cs = pll_cs_read_with_lock(&regs, Some(10), 10_000);
        assert_eq!(cs & (1 << 31), 0);
    }

    #[test]
    fn pll_cs_read_returns_lock_clear_when_arm_none() {
        let regs: [u32; 4] = [0x01, 0, 100, 0];
        let cs = pll_cs_read_with_lock(&regs, None, 10_000);
        assert_eq!(cs & (1 << 31), 0, "None arm → LOCK=0 regardless of now");
    }

    // --- pll_should_arm_lock ------------------------------------------------

    #[test]
    fn pll_should_arm_lock_drops_when_powered_down() {
        // Start with PWR=0, transition to PWR=0x2D → predicate flips false,
        // lock should be dropped.
        let old: [u32; 4] = [0x01, 0, 100, 0];
        let new: [u32; 4] = [0x01, 0x2D, 100, 0];
        let prev = Some(2_000);
        let now = 500;
        assert_eq!(pll_should_arm_lock(&old, &new, prev, now), None);
    }

    #[test]
    fn pll_should_arm_lock_arms_from_powered_down() {
        // Previously un-armed (PWR=0x2D reset), transition to PWR=0 + FBDIV=100
        // → arm at now + delay.
        let old: [u32; 4] = [0x01, 0x2D, 0, 0];
        let new: [u32; 4] = [0x01, 0, 100, 0];
        let now = 100;
        assert_eq!(
            pll_should_arm_lock(&old, &new, None, now),
            Some(100 + PLL_LOCK_DELAY_SYSCLKS)
        );
    }

    #[test]
    fn pll_should_arm_lock_rearms_on_refdiv_change() {
        // Already powered+armed, change REFDIV (CS[5:0]) → rearm.
        let old: [u32; 4] = [0x01, 0, 100, 0];
        let new: [u32; 4] = [0x05, 0, 100, 0];
        let prev = Some(500);
        let now = 10_000;
        assert_eq!(
            pll_should_arm_lock(&old, &new, prev, now),
            Some(10_000 + PLL_LOCK_DELAY_SYSCLKS)
        );
    }

    #[test]
    fn pll_should_arm_lock_rearms_on_fbdiv_change() {
        // Already powered+armed, change FBDIV (FBDIV_INT[11:0]) → rearm.
        let old: [u32; 4] = [0x01, 0, 100, 0];
        let new: [u32; 4] = [0x01, 0, 125, 0];
        let prev = Some(500);
        let now = 10_000;
        assert_eq!(
            pll_should_arm_lock(&old, &new, prev, now),
            Some(10_000 + PLL_LOCK_DELAY_SYSCLKS)
        );
    }

    #[test]
    fn pll_should_arm_lock_keeps_existing_on_prim_change() {
        // PRIM (POSTDIV1/2) changes are post-VCO → do NOT rearm.
        let old: [u32; 4] = [0x01, 0, 100, 0x0007_7000];
        let new: [u32; 4] = [0x01, 0, 100, 0x0002_2000];
        let prev = Some(500);
        let now = 10_000;
        assert_eq!(pll_should_arm_lock(&old, &new, prev, now), Some(500));
    }

    #[test]
    fn pll_should_arm_lock_noop_when_nothing_changes() {
        // Identical before/after → keep prev_lock_at as-is (even None).
        let regs: [u32; 4] = [0x01, 0, 100, 0];
        assert_eq!(pll_should_arm_lock(&regs, &regs, Some(42), 1_000), Some(42));
        // Unconfigured case: predicate is false → always None.
        let unconf: [u32; 4] = [0x01, 0x2D, 0, 0];
        assert_eq!(pll_should_arm_lock(&unconf, &unconf, None, 1_000), None);
    }
}

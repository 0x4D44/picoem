// Shared types + DWT helpers for the silicon oracles.
//
// Every silicon-gated oracle produces a `Vec<CaseOutcome>`. The orchestrator
// (`test_silicon`) concatenates them into a unified report. Each oracle still
// owns its own case type, runner, and diff logic — only the *outcome* shape
// is shared.
//
// The DWT / CoreDebug helpers (`enable_cyccnt`, `reset_cyccnt`,
// `read_cyccnt`) are deduped from the existing per-oracle copies so the
// catalog binaries and the orchestrator all go through one implementation.

use probe_rs::{Core, MemoryInterface};

// ---------------------------------------------------------------------------
// DWT / CoreDebug MMIO (single source of truth)
// ---------------------------------------------------------------------------
//
// Both probe-rs (`u64` addresses) and the emulator bus (`u32` addresses) need
// the same physical constants. Kept as side-by-side pairs so every oracle
// goes through this one module. The `_U32` suffix marks the emulator-facing
// variant to avoid silent type coercion bugs at call sites.

/// Debug Exception and Monitor Control Register — probe-rs path.
pub const DEMCR: u64 = 0xE000_EDFC;
/// Debug Exception and Monitor Control Register — emulator-bus path.
pub const DEMCR_U32: u32 = 0xE000_EDFC;
/// DWT Control register — probe-rs path.
pub const DWT_CTRL: u64 = 0xE000_1000;
/// DWT Control register — emulator-bus path.
pub const DWT_CTRL_U32: u32 = 0xE000_1000;
/// DWT Cycle Counter — probe-rs path.
pub const DWT_CYCCNT: u64 = 0xE000_1004;
/// DWT Cycle Counter — emulator-bus path. Matches the `DWT_CYCCNT_ADDR` name
/// the cycle oracle has used from day one (the literal pool embedded in
/// `MEASUREMENT_STUB` is keyed off this symbol; renaming it would ripple
/// through the binary wrappers for no gain).
pub const DWT_CYCCNT_ADDR: u32 = 0xE000_1004;
/// `DEMCR.TRCENA` — global trace / DWT enable.
pub const TRCENA: u32 = 1 << 24;
/// `DWT_CTRL.CYCCNTENA` — cycle counter enable.
pub const CYCCNTENA: u32 = 1 << 0;

// ---------------------------------------------------------------------------
// Outcome types
// ---------------------------------------------------------------------------

/// A single case's verdict.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Verdict {
    Pass,
    Fail,
}

impl Verdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Verdict::Pass => "PASS",
            Verdict::Fail => "FAIL",
        }
    }
}

/// Outcome of running one case from one oracle. Every silicon oracle's
/// `run_against` returns `Vec<CaseOutcome>`.
///
/// `oracle` / `case` are `&'static str` because catalogues are `&'static` —
/// nothing dynamic to allocate. `detail` is owned because the first-divergence
/// messages are formatted at runtime.
#[derive(Clone, Debug)]
pub struct CaseOutcome {
    pub oracle: &'static str,
    pub case: &'static str,
    pub verdict: Verdict,
    /// Human-readable first divergence; empty on pass.
    pub detail: String,
    /// Wall-clock cost of this case (HW + EMU roundtrip).
    pub elapsed_ms: u32,
}

impl CaseOutcome {
    pub fn pass(oracle: &'static str, case: &'static str, elapsed_ms: u32) -> Self {
        Self {
            oracle,
            case,
            verdict: Verdict::Pass,
            detail: String::new(),
            elapsed_ms,
        }
    }

    pub fn fail(
        oracle: &'static str,
        case: &'static str,
        detail: impl Into<String>,
        elapsed_ms: u32,
    ) -> Self {
        Self {
            oracle,
            case,
            verdict: Verdict::Fail,
            detail: detail.into(),
            elapsed_ms,
        }
    }
}

// ---------------------------------------------------------------------------
// DWT helpers (probe-rs Core)
// ---------------------------------------------------------------------------

/// Enable DWT CYCCNT: set `DEMCR.TRCENA` and `DWT_CTRL.CYCCNTENA`.
pub fn enable_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    let demcr: u32 = core.read_word_32(DEMCR)?;
    core.write_word_32(DEMCR, demcr | TRCENA)?;
    let ctrl: u32 = core.read_word_32(DWT_CTRL)?;
    core.write_word_32(DWT_CTRL, ctrl | CYCCNTENA)?;
    Ok(())
}

/// Zero the DWT cycle counter.
pub fn reset_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    core.write_word_32(DWT_CYCCNT, 0)
}

/// Read the DWT cycle counter.
pub fn read_cyccnt(core: &mut Core) -> Result<u32, probe_rs::Error> {
    core.read_word_32(DWT_CYCCNT)
}

// ---------------------------------------------------------------------------
// Filter helper (shared across oracles)
// ---------------------------------------------------------------------------

/// Return `true` iff `filter` matches `name` — None matches everything, Some
/// matches via substring. Hoisted here so every oracle uses the same
/// semantics.
pub fn name_matches_filter(name: &str, filter: Option<&str>) -> bool {
    match filter {
        None => true,
        Some(sub) => name.contains(sub),
    }
}

/// Return `true` iff `name` should be excluded — None never excludes,
/// Some excludes when the name contains the substring.
pub fn should_exclude(name: &str, exclude: Option<&str>) -> bool {
    match exclude {
        None => false,
        Some(sub) => name.contains(sub),
    }
}

/// Select scenario/case names from a slice given `filter` (include) and
/// `exclude` (skip) substrings.
///
/// Rules:
/// - Both `None`: all names pass.
/// - `filter` only: include names containing the filter substring.
/// - `exclude` only: include names NOT containing the exclude substring.
/// - Both: apply filter first, then subtract any that match exclude.
///
/// Returns `(selected_indices, n_skipped_by_filter, n_skipped_by_exclude)`.
pub fn select_by_name<'a>(
    names: &[&'a str],
    filter: Option<&str>,
    exclude: Option<&str>,
) -> (Vec<usize>, usize, usize) {
    let mut selected = Vec::new();
    let mut skipped_filter = 0usize;
    let mut skipped_exclude = 0usize;

    for (i, name) in names.iter().enumerate() {
        if !name_matches_filter(name, filter) {
            skipped_filter += 1;
            continue;
        }
        if should_exclude(name, exclude) {
            skipped_exclude += 1;
            continue;
        }
        selected.push(i);
    }

    (selected, skipped_filter, skipped_exclude)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_verdict_as_str() {
        assert_eq!(Verdict::Pass.as_str(), "PASS");
        assert_eq!(Verdict::Fail.as_str(), "FAIL");
    }

    #[test]
    fn test_case_outcome_pass_has_empty_detail() {
        let o = CaseOutcome::pass("cycle", "nop_chain_8", 12);
        assert_eq!(o.oracle, "cycle");
        assert_eq!(o.case, "nop_chain_8");
        assert_eq!(o.verdict, Verdict::Pass);
        assert!(o.detail.is_empty());
        assert_eq!(o.elapsed_ms, 12);
    }

    #[test]
    fn test_case_outcome_fail_carries_detail() {
        let o = CaseOutcome::fail("cycle", "backward_branch_large", "hw=14 emu=16 delta=-2 tol=0", 5);
        assert_eq!(o.verdict, Verdict::Fail);
        assert_eq!(o.detail, "hw=14 emu=16 delta=-2 tol=0");
    }

    #[test]
    fn test_name_matches_filter_none_matches_all() {
        assert!(name_matches_filter("anything", None));
        assert!(name_matches_filter("", None));
    }

    #[test]
    fn test_name_matches_filter_substring() {
        assert!(name_matches_filter("pio0_nop_loop", Some("pio0")));
        assert!(name_matches_filter("pio0_nop_loop", Some("nop")));
        assert!(!name_matches_filter("pll_sys_lock_timing", Some("pio0")));
    }

    #[test]
    fn test_should_exclude_none_never_excludes() {
        assert!(!should_exclude("anything", None));
        assert!(!should_exclude("adc_one_shot", None));
    }

    #[test]
    fn test_should_exclude_substring() {
        assert!(should_exclude("adc_one_shot", Some("adc")));
        assert!(should_exclude("adc_round_robin_2ch", Some("adc")));
        assert!(!should_exclude("pio0_nop_loop", Some("adc")));
    }

    // -------------------------------------------------------------------
    // select_by_name — the central selection helper used by all 4 binaries
    // -------------------------------------------------------------------

    const NAMES: &[&str] = &[
        "pio0_nop_loop",
        "pio1_nop_loop",
        "adc_one_shot",
        "adc_round_robin_2ch",
        "pll_sys_lock_timing",
    ];

    #[test]
    fn test_select_both_none_selects_all() {
        let (sel, sf, se) = select_by_name(NAMES, None, None);
        assert_eq!(sel, vec![0, 1, 2, 3, 4]);
        assert_eq!(sf, 0);
        assert_eq!(se, 0);
    }

    #[test]
    fn test_select_filter_only() {
        let (sel, sf, se) = select_by_name(NAMES, Some("pio"), None);
        assert_eq!(sel, vec![0, 1]);
        assert_eq!(sf, 3);  // adc_one_shot, adc_round_robin_2ch, pll_sys_lock_timing
        assert_eq!(se, 0);
    }

    #[test]
    fn test_select_exclude_only() {
        let (sel, sf, se) = select_by_name(NAMES, None, Some("adc"));
        assert_eq!(sel, vec![0, 1, 4]);
        assert_eq!(sf, 0);
        assert_eq!(se, 2);  // adc_one_shot, adc_round_robin_2ch
    }

    #[test]
    fn test_select_filter_and_exclude_disjoint() {
        // filter="pio" picks indices 0,1; exclude="adc" hits nothing in that set
        let (sel, sf, se) = select_by_name(NAMES, Some("pio"), Some("adc"));
        assert_eq!(sel, vec![0, 1]);
        assert_eq!(sf, 3);
        assert_eq!(se, 0);
    }

    #[test]
    fn test_select_filter_and_exclude_overlapping() {
        // filter="" matches all 5; exclude="adc" removes 2
        let (sel, sf, se) = select_by_name(NAMES, Some(""), Some("adc"));
        assert_eq!(sel, vec![0, 1, 4]);
        assert_eq!(sf, 0);
        assert_eq!(se, 2);
    }

    #[test]
    fn test_select_exclude_removes_all_in_filter() {
        // filter="adc" picks 2; exclude="adc" removes all 2 → empty
        let (sel, sf, se) = select_by_name(NAMES, Some("adc"), Some("adc"));
        assert!(sel.is_empty());
        assert_eq!(sf, 3);  // pio0, pio1, pll skipped by filter
        assert_eq!(se, 2);  // both adc* skipped by exclude
    }
}

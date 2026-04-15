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
}

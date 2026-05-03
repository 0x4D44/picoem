//! Cross-binary CLI helpers. Lifted out of probe_diff_*, smoke_powman_*,
//! test_silicon, and riscv_probe_spike on 2026-04-26 (tech-debt sweep,
//! action plan C1 first slice).
//!
//! Kept narrow on purpose. The broader CLI extraction (a unified
//! `OracleFilterArgs` / `BenchArgs` builder pattern) is still deferred
//! per the action plan's opportunistic guidance — pull more helpers up
//! here as they get touched.

use probe_rs::probe::DebugProbeSelector;

/// Parse a `VID:PID:SERIAL` probe selector with a friendly error
/// message that prefixes the offending input string. All probe-using
/// binaries should funnel through this so the error format is uniform.
pub fn parse_probe_selector(s: &str) -> Result<DebugProbeSelector, String> {
    DebugProbeSelector::try_from(s).map_err(|e| format!("invalid probe selector '{s}': {e}"))
}

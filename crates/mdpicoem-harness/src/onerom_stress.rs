//! OneROM stress harness — shared library pieces.
//!
//! Sweep every `addr_bits` value in the A11=A12=1 stimulus range
//! (`0x1800..=0x1FFF`, 2048 cases) through an existing
//! [`onerom_serving_oracle`]-compatible `run_case` and aggregate the
//! resulting [`CaseResult`]s into a latency histogram + plain-text
//! report.
//!
//! This module is intentionally pure: the sweep generator, histogram
//! aggregator, and report formatter are all side-effect-free. Driver
//! binaries (`onerom_stress_pio_rp2350`, `onerom_stress_cpu_rp2350`)
//! own the emulator loop and feed results in.
//!
//! Design: `wrk_docs/2026.04.17 - HLD - OneROM Stress Harness.md`.

use std::fmt::Write as _;
use std::time::Duration;

use crate::onerom_serving_oracle::{
    stimulus_level_pub, Case, CaseResult, Verdict, SHADOW_SIZE,
};

// ---------------------------------------------------------------------------
// Sweep generator
// ---------------------------------------------------------------------------

/// First `addr_bits` in the A11=A12=1 sweep range (inclusive).
const SWEEP_START: u16 = 0x1800;
/// Last `addr_bits` in the A11=A12=1 sweep range (inclusive).
const SWEEP_END: u16 = 0x1FFF;

/// Static label used for every swept case. The sweep is dense — 2048
/// entries, differentiated by `addr_bits` — so a per-case textual label
/// would be noise; the driver binary surfaces `addr_bits` directly in
/// per-failure lines.
const SWEEP_LABEL: &str = "stress sweep";

/// Generate the full 2048-entry `addr_bits ∈ 0x1800..=0x1FFF` sweep.
///
/// Each case reuses [`Case::new`] (same A11=A12=1 invariant assert as
/// the 15-case [`DEFAULT_CASES`][crate::onerom_serving_oracle::DEFAULT_CASES]).
/// The expected byte is *not* stored on `Case` — the driver computes it
/// post-hoc via [`expected_byte_for`], which mirrors the evaluator's
/// shadow index at `onerom_serving_oracle::evaluate_case_trace`.
pub fn generate_sweep_cases() -> Vec<Case> {
    (SWEEP_START..=SWEEP_END)
        .map(|addr_bits| Case::new(SWEEP_LABEL, addr_bits))
        .collect()
}

/// Expected byte for a given `addr_bits`, given the lifted shadow.
///
/// Driver binaries (PIO/CPU stress) must call this rather than
/// re-deriving the expected byte inline — keep the shadow-lookup
/// formula in one place.
///
/// Matches the lookup in `evaluate_case_trace`:
/// `resolved = (0x2000 << 16) | (stimulus_level(addr_bits) & 0xFFFF)`,
/// `expected = shadow[resolved - SHADOW_BASE]`, which for the current
/// `SHADOW_BASE = 0x2000_0000` collapses to
/// `shadow[stimulus_level(addr_bits) & 0xFFFF]`.
#[must_use]
pub fn expected_byte_for(shadow: &[u8; SHADOW_SIZE], addr_bits: u16) -> u8 {
    let idx = (stimulus_level_pub(addr_bits) & 0xFFFF) as usize;
    shadow[idx]
}

// ---------------------------------------------------------------------------
// Histogram aggregator
// ---------------------------------------------------------------------------

/// Aggregated per-run latency summary.
///
/// `count` is the total number of results fed in; `pass`/`fail` split
/// them by [`Verdict::Pass`] vs. everything else. Only `Pass` cases
/// contribute to the ns statistics — non-Pass verdicts have no
/// meaningful latency to aggregate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Histogram {
    pub count: usize,
    pub pass: usize,
    pub fail: usize,
    pub min_ns: u32,
    pub max_ns: u32,
    pub mean_ns: u32,
    pub p50_ns: u32,
    pub p95_ns: u32,
    pub p99_ns: u32,
    /// Number of distinct cycle counts observed across `Pass` cases. A
    /// small value (≤ 3) signals that the per-case latency is
    /// near-deterministic and percentile rows are a misleading
    /// "statistical" framing — the report surfaces this so a reader
    /// doesn't quote `p95` as if it were a tail.
    pub unique_cycles: usize,
}

/// Convert a cycle count to nanoseconds via the provided sysclk.
/// Uses u64 intermediate math so sysclks × 1e9 doesn't overflow u32.
fn cycles_to_ns(cycles: u32, sys_clk_hz: u32) -> u64 {
    (cycles as u64) * 1_000_000_000 / (sys_clk_hz as u64)
}

/// Nearest-rank percentile on a pre-sorted ascending slice.
///
/// Picks index `ceil(p * n / 100) - 1` clamped to `[0, n - 1]`. `p` is
/// the percentile in `0..=100`. Caller must ensure `sorted` is not
/// empty. Generic so both cycle counts (`u32`) and wall-clock
/// nanosecond samples (`u64`) share the same rank logic.
fn nearest_rank<T: Copy + Ord>(sorted: &[T], p: u32) -> T {
    let n = sorted.len();
    debug_assert!(n > 0, "nearest_rank: empty slice");
    // ceil(p * n / 100) via integer math: (p*n + 99) / 100.
    let raw = ((p as usize) * n + 99) / 100;
    let idx = raw.saturating_sub(1).min(n - 1);
    sorted[idx]
}

/// Compute the latency histogram over a set of case results.
///
/// Only [`Verdict::Pass`] results contribute to the ns statistics; non-
/// Pass results are counted in `fail` but their latencies (if any) are
/// discarded. An empty input returns an all-zero histogram.
pub fn compute_histogram(results: &[CaseResult], sys_clk_hz: u32) -> Histogram {
    let count = results.len();
    let pass = results.iter().filter(|r| r.verdict == Verdict::Pass).count();
    let fail = count - pass;

    if pass == 0 {
        return Histogram {
            count,
            pass,
            fail,
            min_ns: 0,
            max_ns: 0,
            mean_ns: 0,
            p50_ns: 0,
            p95_ns: 0,
            p99_ns: 0,
            unique_cycles: 0,
        };
    }

    // Project Pass → cycles. A Pass result without `latency_cycles`
    // would be a structural bug in the caller (the evaluator always sets
    // it on a Pass verdict), but defend with `filter_map` rather than
    // `expect` to keep this aggregator total. Statistics are computed in
    // cycles and converted to ns once at the end — matches the neighbour
    // `onerom_serving_oracle`'s pattern and avoids per-sample
    // integer-truncation error accumulating into the mean.
    let mut cycles: Vec<u32> = results
        .iter()
        .filter(|r| r.verdict == Verdict::Pass)
        .filter_map(|r| r.latency_cycles)
        .collect();

    if cycles.is_empty() {
        // All Pass results lacked cycle counts — treat as empty.
        return Histogram {
            count,
            pass,
            fail,
            min_ns: 0,
            max_ns: 0,
            mean_ns: 0,
            p50_ns: 0,
            p95_ns: 0,
            p99_ns: 0,
            unique_cycles: 0,
        };
    }

    cycles.sort_unstable();
    let unique_cycles = cycles.windows(2).filter(|w| w[0] != w[1]).count() + 1;
    let min_cycles = *cycles.first().unwrap();
    let max_cycles = *cycles.last().unwrap();
    // Sum fits in u64: max u32 cycles × 2048 max cases ≪ 2^64.
    let sum: u64 = cycles.iter().map(|&v| v as u64).sum();
    let mean_cycles = u32::try_from(sum / cycles.len() as u64).unwrap_or(u32::MAX);
    let p50_cycles = nearest_rank(&cycles, 50);
    let p95_cycles = nearest_rank(&cycles, 95);
    let p99_cycles = nearest_rank(&cycles, 99);

    // Convert to ns in one place. `u32::try_from` guards against
    // pathological cycle counts that would otherwise silently wrap.
    let to_ns = |c: u32| u32::try_from(cycles_to_ns(c, sys_clk_hz)).unwrap_or(u32::MAX);
    let min_ns = to_ns(min_cycles);
    let max_ns = to_ns(max_cycles);
    let mean_ns = to_ns(mean_cycles);
    let p50_ns = to_ns(p50_cycles);
    let p95_ns = to_ns(p95_cycles);
    let p99_ns = to_ns(p99_cycles);

    Histogram {
        count,
        pass,
        fail,
        min_ns,
        max_ns,
        mean_ns,
        p50_ns,
        p95_ns,
        p99_ns,
        unique_cycles,
    }
}

// ---------------------------------------------------------------------------
// Wall-clock aggregator
// ---------------------------------------------------------------------------

/// Host wall-clock stats over one sweep.
///
/// Unlike [`Histogram`]'s `*_ns` fields — which are *model predictions*
/// derived from emulated cycle counts × (1 / `sys_clk_hz`) — these are
/// measurements of the host clock via `Instant::now()` around each
/// `run_case`. They answer "is this emulator fast enough to drive a
/// real bus in real time on this machine?", not "would real silicon
/// meet the ROM's timing envelope?".
///
/// All cases contribute regardless of verdict: a failing case still
/// spent host time and belongs in the throughput denominator. That
/// differs from [`Histogram`], where only `Verdict::Pass` contributes
/// (a WrongByte case has no meaningful emulated latency to aggregate,
/// but it absolutely has wall-clock cost).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WallClockStats {
    pub count: usize,
    /// Sum of per-case host durations. Used as the throughput
    /// denominator; also rendered as "total" in the report.
    pub total_ns: u64,
    pub min_ns: u64,
    pub max_ns: u64,
    pub mean_ns: u64,
    pub p50_ns: u64,
    pub p95_ns: u64,
    pub p99_ns: u64,
}

impl WallClockStats {
    /// Cases per second of host wall-clock, from `count / total_ns`.
    ///
    /// Returned as `f64` because the natural scale spans several orders
    /// of magnitude (fast host: 1e4+ cases/sec; slow emulator: single
    /// digits). Callers that want an integer render should round.
    #[must_use]
    pub fn cases_per_sec(&self) -> f64 {
        if self.total_ns == 0 {
            return 0.0;
        }
        (self.count as f64) * 1e9 / (self.total_ns as f64)
    }
}

/// Aggregate per-case host durations into a [`WallClockStats`].
///
/// Empty input returns an all-zero struct. `Duration::as_nanos()`
/// returns `u128`; we saturate to `u64` — a single case running for
/// >584 years is not a realistic failure mode, but the saturation
/// keeps the type narrow for the percentile sort.
#[must_use]
pub fn compute_wall_clock_stats(durations: &[Duration]) -> WallClockStats {
    let count = durations.len();
    if count == 0 {
        return WallClockStats {
            count: 0,
            total_ns: 0,
            min_ns: 0,
            max_ns: 0,
            mean_ns: 0,
            p50_ns: 0,
            p95_ns: 0,
            p99_ns: 0,
        };
    }

    let mut ns: Vec<u64> = durations
        .iter()
        .map(|d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
        .collect();
    let total_ns: u64 = ns.iter().fold(0u64, |acc, &v| acc.saturating_add(v));
    ns.sort_unstable();
    let min_ns = *ns.first().unwrap();
    let max_ns = *ns.last().unwrap();
    let mean_ns = total_ns / count as u64;
    let p50_ns = nearest_rank(&ns, 50);
    let p95_ns = nearest_rank(&ns, 95);
    let p99_ns = nearest_rank(&ns, 99);

    WallClockStats {
        count,
        total_ns,
        min_ns,
        max_ns,
        mean_ns,
        p50_ns,
        p95_ns,
        p99_ns,
    }
}

// ---------------------------------------------------------------------------
// Report formatter
// ---------------------------------------------------------------------------

/// Upper bound (inclusive) for the "fast" band. Placeholder band — not
/// silicon-calibrated; see HLD §Non-goals.
const FAST_NS: u32 = 100;
/// Upper bound (inclusive) for the "standard" band. Placeholder band —
/// not silicon-calibrated; see HLD §Non-goals.
const STANDARD_NS: u32 = 200;

/// ROM speed classification for the summary footer.
///
/// Bands per HLD §Output format in the stress HLD — coarser than the
/// serving oracle's report (which uses six bands) because the stress
/// tool is a regression signal, not a silicon-calibrated diagnostic.
fn rom_speed_class(mean_ns: u32) -> &'static str {
    if mean_ns <= FAST_NS {
        "fast"
    } else if mean_ns <= STANDARD_NS {
        "standard"
    } else {
        "slow"
    }
}

/// Single-line rendering of one failing case. Enum parameters stay
/// inline (no pretty-printing) so the driver's "first N failures"
/// block stays compact.
fn format_fail_line(r: &CaseResult) -> String {
    let addr = format!("0x{:04X}", r.case.addr_bits);
    let expected = match r.expected_byte {
        Some(b) => format!("0x{:02X}", b),
        None => "-".to_string(),
    };
    let observed = match r.observed_byte {
        Some(b) => format!("0x{:02X}", b),
        None => "-".to_string(),
    };
    let cycles = match r.latency_cycles {
        Some(c) => format!("{}", c),
        None => "-".to_string(),
    };
    let verdict = match r.verdict {
        Verdict::Pass => "Pass".to_string(),
        Verdict::WrongByte { .. } => "WrongByte".to_string(),
        Verdict::NoResolve => "NoResolve".to_string(),
        Verdict::NoStableByte => "NoStableByte".to_string(),
        Verdict::ResolvedAddrOutOfRange { .. } => "ResolvedAddrOutOfRange".to_string(),
        Verdict::LatencyOutOfEnvelope { .. } => "LatencyOutOfEnvelope".to_string(),
    };
    format!(
        "  addr={} expected={} observed={} cycles={} verdict={}",
        addr, expected, observed, cycles, verdict
    )
}

/// Render the stress report to a string. Pure function — no I/O.
///
/// `label` is a free-form one-liner (e.g. `"1541 $E000 kernal
/// (901229-06AA), PIO mode"`), `fixture_path` is the workspace-relative
/// fixture path, `rom_set` is the selected ROM set index, `sys_clk_hz`
/// drives the reported "sys_clk_hz:" line, `hist` is the aggregated
/// summary, and `first_fails` is an already-truncated slice (caller
/// decides the cap — HLD suggests 20).
pub fn format_report(
    label: &str,
    fixture_path: &str,
    rom_set: u8,
    sys_clk_hz: u32,
    hist: &Histogram,
    wall: &WallClockStats,
    first_fails: &[CaseResult],
) -> String {
    let mut out = String::new();
    writeln!(out, "OneROM Stress — {}", label).unwrap();
    writeln!(out, "sys_clk_hz: {} MHz", sys_clk_hz / 1_000_000).unwrap();
    writeln!(out, "fixture: {}", fixture_path).unwrap();
    writeln!(out, "rom_set: {}", rom_set).unwrap();
    writeln!(out).unwrap();
    writeln!(out, "cases    : {}", hist.count).unwrap();
    writeln!(out, "pass     : {}", hist.pass).unwrap();
    writeln!(out, "fail     : {}", hist.fail).unwrap();
    writeln!(out).unwrap();
    // Model-predicted latency: emulated cycles × 1/sysclk. Only trustworthy
    // to the extent our cycle model matches silicon — this tool does not
    // calibrate that, `test_silicon_cycle_oracle_rp2350` does.
    writeln!(out, "emulated latency (model: cycles x 1/sysclk, uncalibrated) (ns):").unwrap();
    writeln!(out, "  min    : {:>4}", hist.min_ns).unwrap();
    writeln!(out, "  p50    : {:>4}", hist.p50_ns).unwrap();
    writeln!(out, "  mean   : {:>4}", hist.mean_ns).unwrap();
    writeln!(out, "  p95    : {:>4}", hist.p95_ns).unwrap();
    writeln!(out, "  p99    : {:>4}", hist.p99_ns).unwrap();
    writeln!(out, "  max    : {:>4}", hist.max_ns).unwrap();
    writeln!(out, "  unique cycle values: {}", hist.unique_cycles).unwrap();
    if hist.pass > 0 && hist.unique_cycles <= 3 {
        writeln!(
            out,
            "  (note: near-deterministic distribution — percentiles above \
             are not meaningful as 'tails', they name the 2–3 steady-state \
             cycle buckets the emulator exhibits for this fixture)"
        )
        .unwrap();
    }
    writeln!(out).unwrap();
    // Host wall-clock: this is what Instant::now() actually measured on
    // the machine that ran the sweep. Tells you how close (or far) this
    // emulator is to real-time, independent of the cycle model.
    writeln!(out, "wall-clock per case (host-measured, this run) (us):").unwrap();
    writeln!(out, "  min    : {:>9.3}", ns_to_us_f64(wall.min_ns)).unwrap();
    writeln!(out, "  p50    : {:>9.3}", ns_to_us_f64(wall.p50_ns)).unwrap();
    writeln!(out, "  mean   : {:>9.3}", ns_to_us_f64(wall.mean_ns)).unwrap();
    writeln!(out, "  p95    : {:>9.3}", ns_to_us_f64(wall.p95_ns)).unwrap();
    writeln!(out, "  p99    : {:>9.3}", ns_to_us_f64(wall.p99_ns)).unwrap();
    writeln!(out, "  max    : {:>9.3}", ns_to_us_f64(wall.max_ns)).unwrap();
    writeln!(
        out,
        "  total  : {:.3} s, throughput: {:.0} cases/sec",
        (wall.total_ns as f64) / 1e9,
        wall.cases_per_sec()
    )
    .unwrap();
    // Honesty footer: relate the two blocks so a reader can see at a
    // glance whether this emulator could replace silicon in real-time.
    // Ratio > 1 means host is slower than real silicon; ratio < 1 would
    // mean faster-than-real-time (rare for a cycle-accurate emulator).
    if hist.pass > 0 && hist.mean_ns > 0 {
        let ratio = (wall.mean_ns as f64) / (hist.mean_ns as f64);
        writeln!(
            out,
            "  (host is {:.0}x slower than the emulated-model mean; \
             real-time capability requires ratio ~= 1)",
            ratio
        )
        .unwrap();
    }
    writeln!(out).unwrap();
    writeln!(
        out,
        "ROM speed class: {} (mean {} ns, model — not calibrated to silicon)",
        rom_speed_class(hist.mean_ns),
        hist.mean_ns
    )
    .unwrap();

    if hist.fail > 0 {
        writeln!(out).unwrap();
        writeln!(
            out,
            "first failures ({} of {}):",
            first_fails.len(),
            hist.fail
        )
        .unwrap();
        for r in first_fails {
            writeln!(out, "{}", format_fail_line(r)).unwrap();
        }
    }

    out
}

/// Render a nanosecond count as microseconds with f64 precision, for
/// the wall-clock block. Kept as a tiny helper so the formatter's
/// arithmetic doesn't sprawl inline.
#[inline]
fn ns_to_us_f64(ns: u64) -> f64 {
    (ns as f64) / 1_000.0
}

// ---------------------------------------------------------------------------
// Tests (TDD order: histogram-empty → percentiles → sweep coverage)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::onerom_serving_oracle::SHADOW_BASE;

    /// Build a synthetic `Pass` CaseResult with a specific cycle count.
    /// Used to feed hand-crafted latency distributions into the
    /// histogram aggregator without spinning up an emulator.
    fn mk_pass_result(addr_bits: u16, cycles: u32) -> CaseResult {
        CaseResult {
            case: Case::new("test", addr_bits),
            resolved_addr: Some(SHADOW_BASE),
            expected_byte: Some(0x00),
            observed_byte: Some(0x00),
            latency_cycles: Some(cycles),
            verdict: Verdict::Pass,
        }
    }

    /// Test A: empty input → all-zero histogram. The aggregator must
    /// not panic (no sort on empty, no divide-by-zero) and must report
    /// zero for every ns field.
    #[test]
    fn histogram_of_empty_is_zeroed() {
        let h = compute_histogram(&[], 150_000_000);
        assert_eq!(
            h,
            Histogram {
                count: 0,
                pass: 0,
                fail: 0,
                min_ns: 0,
                max_ns: 0,
                mean_ns: 0,
                p50_ns: 0,
                p95_ns: 0,
                p99_ns: 0,
                unique_cycles: 0,
            }
        );
    }

    /// Test B: nearest-rank percentiles on 7 hand-chosen cycle counts.
    /// Cycles [9, 15, 21, 30, 42, 48, 60] at 150 MHz → ns [60, 100,
    /// 140, 200, 280, 320, 400]. Statistics are computed in cycles
    /// first, then converted once: mean_cycles = 225/7 = 32, mean_ns =
    /// cycles_to_ns(32, 150 MHz) = 213 (vs 214 if averaged in ns — the
    /// per-sample truncation that order-of-ops avoids).
    /// p50 = index ceil(0.5*7)-1 = 3 → cycles[3]=30 → 200 ns.
    /// p95 = p99 = index 6 → cycles[6]=60 → 400 ns.
    #[test]
    fn histogram_percentiles_nearest_rank() {
        let cycles = [9u32, 15, 21, 30, 42, 48, 60];
        let results: Vec<CaseResult> = cycles
            .iter()
            .enumerate()
            .map(|(i, &c)| mk_pass_result(0x1800 + i as u16, c))
            .collect();
        let h = compute_histogram(&results, 150_000_000);
        assert_eq!(h.count, 7);
        assert_eq!(h.pass, 7);
        assert_eq!(h.fail, 0);
        assert_eq!(h.min_ns, 60);
        assert_eq!(h.max_ns, 400);
        assert_eq!(h.mean_ns, 213, "mean_ns = to_ns(225/7) = to_ns(32) = 213");
        assert_eq!(h.p50_ns, 200, "nearest-rank p50 on n=7 → idx 3 → 200");
        assert_eq!(h.p95_ns, 400, "nearest-rank p95 on n=7 → idx 6 → 400");
        assert_eq!(h.p99_ns, 400, "nearest-rank p99 on n=7 → idx 6 → 400");
    }

    /// Test B': nearest-rank percentile edge cases — n=1, n=2, p=100.
    /// These pin down the formula's behaviour at boundaries that a
    /// "floor" or "lower-hinge" implementation would get wrong.
    /// At 150 MHz, 15 cycles → 100 ns and 30 cycles → 200 ns.
    #[test]
    fn histogram_percentiles_edge_cases() {
        // n=1: every percentile returns the single value.
        let single = vec![mk_pass_result(0x1800, 15)];
        let h1 = compute_histogram(&single, 150_000_000);
        assert_eq!(h1.min_ns, 100);
        assert_eq!(h1.max_ns, 100);
        assert_eq!(h1.mean_ns, 100);
        assert_eq!(h1.p50_ns, 100, "n=1 → p50 returns the single value");
        assert_eq!(h1.p95_ns, 100, "n=1 → p95 returns the single value");
        assert_eq!(h1.p99_ns, 100, "n=1 → p99 returns the single value");

        // n=2 distinct: p50 via ceil(0.5*2)-1 = 0 picks the *lower*
        // value. A "lower-hinge" impl (floor) would also return 100, so
        // this case distinguishes those from a "ceil without -1" impl
        // that would pick index 1 (200).
        let pair = vec![mk_pass_result(0x1800, 15), mk_pass_result(0x1801, 30)];
        let h2 = compute_histogram(&pair, 150_000_000);
        assert_eq!(h2.min_ns, 100);
        assert_eq!(h2.max_ns, 200);
        assert_eq!(h2.p50_ns, 100, "n=2 → p50 = sorted[ceil(0.5*2)-1] = sorted[0] = 100");
        // p=100 must return sorted[n-1], never index n. The nearest-rank
        // index is ceil(1.0*2)-1 = 1, so we expect the upper value.
        assert_eq!(h2.p95_ns, 200, "n=2 → p95 = sorted[1] = 200");
        assert_eq!(h2.p99_ns, 200, "n=2 → p99 = sorted[1] = 200");

        // p=100 on a larger sample: still sorted[n-1], never index n.
        // 5 cycles → 33 ns (integer-truncated), so the sample is
        // [33, 66, 100, 133, 166] at 150 MHz. p99 → ceil(0.99*5)-1 = 4.
        let five: Vec<CaseResult> = (1u32..=5)
            .enumerate()
            .map(|(i, n)| mk_pass_result(0x1800 + i as u16, n * 5))
            .collect();
        let h5 = compute_histogram(&five, 150_000_000);
        assert_eq!(h5.p99_ns, h5.max_ns, "n=5 → p99 = sorted[n-1] = max_ns");
        // Explicit value (not re-derived from the formula under test):
        // 25 cycles / 150 MHz = 166 ns (integer-truncated from 166.66…).
        assert_eq!(h5.p99_ns, 166, "n=5 → p99 = 25 cycles @ 150 MHz = 166 ns");
    }

    /// Test C: the sweep covers every addr_bits from 0x1800 to 0x1FFF
    /// inclusive, and the expected-byte helper looks up the shadow via
    /// `stimulus_level_pub(addr_bits) & 0xFFFF`.
    #[test]
    fn generate_sweep_cases_covers_full_range() {
        // Predictable pattern: shadow[i] = (i & 0xFF) as u8.
        let mut shadow = Box::new([0u8; SHADOW_SIZE]);
        for i in 0..SHADOW_SIZE {
            shadow[i] = (i & 0xFF) as u8;
        }

        let cases = generate_sweep_cases();
        assert_eq!(cases.len(), 2048);
        assert_eq!(cases.first().unwrap().addr_bits, 0x1800);
        assert_eq!(cases.last().unwrap().addr_bits, 0x1FFF);

        // Denseness: every addr_bits from 0x1800..=0x1FFF must be
        // present exactly once, in order. Catches a broken generator
        // that emits duplicates (e.g. [0x1800, 0x1FFF, 0x1FFF, ...]).
        for (i, case) in cases.iter().enumerate() {
            assert_eq!(case.addr_bits, 0x1800 + i as u16);
        }

        // Known case: addr_bits = 0x1802. The expected byte comes from
        // the shadow at index `stimulus_level(0x1802) & 0xFFFF`.
        let addr_bits = 0x1802u16;
        let expected_idx = (stimulus_level_pub(addr_bits) & 0xFFFF) as usize;
        let expected = shadow[expected_idx];
        assert_eq!(
            expected_byte_for(&shadow, addr_bits),
            expected,
            "expected byte must match shadow[stimulus_level(0x1802) & 0xFFFF]"
        );
    }

    /// `unique_cycles` must count distinct cycle buckets across Pass
    /// cases, so the report can flag near-deterministic distributions
    /// where percentiles would otherwise be quoted as if they were
    /// statistical tails. Three hand-picked inputs cover the shapes
    /// we care about: all-same (bucket count 1), two-bucket step
    /// function (count 2), continuous (count equals sample count).
    #[test]
    fn histogram_unique_cycles_counts_distinct_buckets() {
        use crate::onerom_serving_oracle::Case;
        let mk = |label: &'static str, addr: u16, cycles: u32| CaseResult {
            case: Case::new(label, addr),
            expected_byte: Some(0),
            observed_byte: Some(0),
            resolved_addr: None,
            latency_cycles: Some(cycles),
            verdict: Verdict::Pass,
        };

        // All-same: five passes at 20 cycles → 1 bucket.
        let h1 = compute_histogram(
            &[
                mk("a", 0x1800, 20),
                mk("b", 0x1801, 20),
                mk("c", 0x1802, 20),
                mk("d", 0x1803, 20),
                mk("e", 0x1804, 20),
            ],
            150_000_000,
        );
        assert_eq!(h1.unique_cycles, 1);

        // Two-bucket step: alternating 34 and 36 → 2 buckets.
        let h2 = compute_histogram(
            &[
                mk("a", 0x1800, 34),
                mk("b", 0x1801, 36),
                mk("c", 0x1802, 34),
                mk("d", 0x1803, 36),
            ],
            150_000_000,
        );
        assert_eq!(h2.unique_cycles, 2);

        // Continuous: four distinct cycle counts → 4 buckets.
        let h3 = compute_histogram(
            &[
                mk("a", 0x1800, 10),
                mk("b", 0x1801, 20),
                mk("c", 0x1802, 30),
                mk("d", 0x1803, 40),
            ],
            150_000_000,
        );
        assert_eq!(h3.unique_cycles, 4);
    }

    /// Wall-clock aggregator: empty input → all-zero stats. No
    /// divide-by-zero, no panic, `cases_per_sec()` returns 0.0 when
    /// total_ns is 0.
    #[test]
    fn wall_clock_stats_of_empty_is_zeroed() {
        let wc = compute_wall_clock_stats(&[]);
        assert_eq!(
            wc,
            WallClockStats {
                count: 0,
                total_ns: 0,
                min_ns: 0,
                max_ns: 0,
                mean_ns: 0,
                p50_ns: 0,
                p95_ns: 0,
                p99_ns: 0,
            }
        );
        assert_eq!(wc.cases_per_sec(), 0.0);
    }

    /// Wall-clock aggregator: 5 samples at 100/200/300/400/500 ns. Mean
    /// = 300, total = 1500, throughput = 5 / 1.5 μs = 3 333 333 cases/s.
    /// p50 via ceil(0.5*5)-1 = idx 2 → 300; p95/p99 → idx 4 → 500.
    #[test]
    fn wall_clock_stats_percentiles_nearest_rank() {
        let durations = vec![
            Duration::from_nanos(100),
            Duration::from_nanos(200),
            Duration::from_nanos(300),
            Duration::from_nanos(400),
            Duration::from_nanos(500),
        ];
        let wc = compute_wall_clock_stats(&durations);
        assert_eq!(wc.count, 5);
        assert_eq!(wc.total_ns, 1500);
        assert_eq!(wc.min_ns, 100);
        assert_eq!(wc.max_ns, 500);
        assert_eq!(wc.mean_ns, 300);
        assert_eq!(wc.p50_ns, 300);
        assert_eq!(wc.p95_ns, 500);
        assert_eq!(wc.p99_ns, 500);
        // 5 cases / 1.5 μs = 3 333 333.33 cases/sec. Allow a loose
        // epsilon for f64 rounding.
        let cps = wc.cases_per_sec();
        assert!((cps - 3_333_333.33).abs() < 1.0, "cps was {}", cps);
    }

    /// Wall-clock aggregator: unsorted input must be sorted before
    /// percentile lookup — feed a reverse-ordered set and check that
    /// min/max/p50 come out right anyway.
    #[test]
    fn wall_clock_stats_sorts_before_percentiles() {
        let durations = vec![
            Duration::from_nanos(500),
            Duration::from_nanos(100),
            Duration::from_nanos(400),
            Duration::from_nanos(200),
            Duration::from_nanos(300),
        ];
        let wc = compute_wall_clock_stats(&durations);
        assert_eq!(wc.min_ns, 100, "min is the smallest, regardless of input order");
        assert_eq!(wc.max_ns, 500);
        assert_eq!(wc.p50_ns, 300, "p50 is the middle of the *sorted* set");
    }
}

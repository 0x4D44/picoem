// Shared infrastructure for the test_silicon orchestrator family
// (`test_silicon` for RP2350/RP2354, `test_silicon_rp2040` for RP2040).
//
// Lifted out of `bin/test_silicon.rs` so the same parsing, shuffle,
// reattach, and bookkeeping primitives can be reused across chips. Each
// orchestrator owns its own oracle dispatch table; everything here is
// chip-agnostic.
//
// The module is intentionally narrow: parser primitives, RNG seeding,
// Fisher-Yates shuffle, name-uniqueness validation, an interner for
// synthesised case names, log helpers, retry constants, and the bounded
// attach-with-retries helper. No probe-rs dispatch; no `OracleKind`
// enum (those stay per-binary because the kinds differ between chips).

use std::collections::BTreeMap;
use std::fs::OpenOptions;
use std::io::Write as IoWrite;
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use probe_rs::probe::{DebugProbeSelector, list::Lister};
use probe_rs::{Permissions, Session, SessionConfig};
use rand::rngs::StdRng;

// ---------------------------------------------------------------------------
// Reattach / heartbeat / give-up constants
// ---------------------------------------------------------------------------

/// Sleep between reattach retries.
pub const REATTACH_RETRY: Duration = Duration::from_secs(5);
/// Total budget for reattach retries before giving up on this attempt.
pub const REATTACH_TIMEOUT: Duration = Duration::from_secs(60);
/// Initial sleep after a probe-rs failure before the first reattach.
pub const REATTACH_INITIAL_SLEEP: Duration = Duration::from_secs(1);
/// Number of consecutive reattach failures that trigger the orchestrator
/// to bail out of soak mode with rc=2.
pub const GIVE_UP_THRESHOLD: u32 = 3;
/// Heartbeat cadence in soak mode (quiet-by-default).
pub const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(3600);

// ---------------------------------------------------------------------------
// CLI helpers
// ---------------------------------------------------------------------------

/// Parse a `humantime` duration string (`30m`, `4h`, `7d`, etc.).
pub fn parse_duration(s: &str) -> Result<Duration, String> {
    humantime::parse_duration(s).map_err(|e| format!("invalid duration '{s}': {e}"))
}

/// Default RNG seed: seconds since the Unix epoch. Stable enough for soak
/// mode while still being easy to reproduce via `--seed`.
pub fn default_seed() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// Per-iteration soak seed: `base + iter` with wrapping semantics so soak
/// runs of any length stay reproducible.
pub fn iter_seed(base_seed: u64, iter_index: u64) -> u64 {
    base_seed.wrapping_add(iter_index)
}

// ---------------------------------------------------------------------------
// Fisher-Yates shuffle (deterministic given the RNG)
// ---------------------------------------------------------------------------

pub fn shuffle_in_place<T>(v: &mut [T], rng: &mut StdRng) {
    use rand::RngCore;
    for i in (1..v.len()).rev() {
        let j = (rng.next_u64() % (i as u64 + 1)) as usize;
        v.swap(i, j);
    }
}

// ---------------------------------------------------------------------------
// Catalogue substring-uniqueness validator
// ---------------------------------------------------------------------------

/// Validate that the combined set of case names has no name that is a
/// strict substring of another. The orchestrator's filter semantics are
/// substring-match, and one oracle's name being a substring of another
/// would alias two cases under a single `--filter` flag. Fail fast at
/// startup — a corrupt filter is worse than a clean refusal.
///
/// Returns `Err` with a human-readable message pointing at the first
/// offending pair; returns `Ok(())` otherwise.
pub fn validate_catalogue_names_are_unique(names: &[&str]) -> Result<(), String> {
    // O(N^2). Catalogues together carry ~50 names; the whole check
    // runs in microseconds.
    for (i, a) in names.iter().enumerate() {
        for (j, b) in names.iter().enumerate() {
            if i == j {
                continue;
            }
            if a == b {
                return Err(format!(
                    "duplicate case name '{a}' appears in two catalogues; \
                     names must be unique across oracles",
                ));
            }
            if a.contains(b) {
                return Err(format!(
                    "case name '{b}' is a substring of '{a}'; \
                     the orchestrator's filter logic relies on uniqueness",
                ));
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Synthetic case-name interner
// ---------------------------------------------------------------------------
//
// `CaseOutcome.case` is `&'static str`. When the orchestrator synthesises
// a FAIL outcome for a watchdog / probe-rs error, the case name can be a
// runtime-assembled string (e.g. the oracle name plus a sentinel). We
// intern each such name exactly once via `Box::leak` so repeated failures
// on the same synthetic name do not accumulate leaks over a week-long
// soak run. Bounded by the number of distinct synthetic names ever
// emitted — in practice just the sentinels plus per-oracle tags.

#[derive(Default)]
pub struct NameInterner {
    pub seen: BTreeMap<String, &'static str>,
}

impl NameInterner {
    pub fn intern(&mut self, s: &str) -> &'static str {
        if let Some(v) = self.seen.get(s) {
            return v;
        }
        let leaked: &'static str = Box::leak(s.to_string().into_boxed_str());
        self.seen.insert(s.to_string(), leaked);
        leaked
    }
}

// ---------------------------------------------------------------------------
// Logging helpers
// ---------------------------------------------------------------------------

/// Format an elapsed duration as `[+HH:MM:SS]`.
pub fn fmt_elapsed(d: Duration) -> String {
    let total = d.as_secs();
    let h = total / 3600;
    let m = (total % 3600) / 60;
    let s = total % 60;
    format!("[+{h:02}:{m:02}:{s:02}]")
}

/// Build the per-binary error log path:
/// `fuzz-runs/<binary_prefix>.<pid>.errors.log`.
pub fn errors_log_path(binary_prefix: &str) -> PathBuf {
    let pid = std::process::id();
    PathBuf::from(format!("fuzz-runs/{binary_prefix}.{pid}.errors.log"))
}

pub fn ensure_fuzz_runs_dir() -> std::io::Result<()> {
    std::fs::create_dir_all("fuzz-runs")
}

pub fn append_error_log(path: &PathBuf, line: &str) {
    let _ = ensure_fuzz_runs_dir();
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(f, "{line}");
    }
}

/// Print a line to stdout, mirror to stderr, and append to the
/// catastrophic-failure log. One call per FAIL / probe error / reattach
/// failure keeps the soak-loop body terse.
pub fn emit_log_line(log_path: &PathBuf, line: &str) {
    println!("{line}");
    eprintln!("{line}");
    append_error_log(log_path, line);
}

/// Wall-clock ISO-8601 timestamp for log lines (UTC, second resolution).
pub fn now_iso() -> String {
    let now = SystemTime::now();
    let dt: chrono::DateTime<chrono::Utc> = now.into();
    dt.format("%Y-%m-%dT%H:%M:%SZ").to_string()
}

// ---------------------------------------------------------------------------
// Summary bookkeeping
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct Stats {
    pub pass: u64,
    pub fail: u64,
    /// Filter-gap / coverage-only skips (e.g. UNDEF on silicon).
    pub skip: u64,
    /// Probe-rs / transport-level errors during the case. Used by the
    /// orchestrator to drive degraded-rate-triggered reattach.
    pub degraded: u64,
}

/// Cross-oracle summary used by both the RP2350 and RP2040 orchestrators.
/// `failing_cases` / `skipping_cases` / `degraded_cases` each keep the
/// smallest-seen seed per `(oracle, case)` so the final report points at
/// a deterministic repro per kind. They are tracked independently — a
/// case that skips on iteration 5 and fails on iteration 10 will appear
/// in both maps with their respective smallest seeds.
///
/// Keys are `(&'static str, &'static str)` because `CaseOutcome.case` is
/// already `&'static str` (per-oracle catalogues plus the orchestrator's
/// `NameInterner` for synthesised names) — no per-failure allocation.
#[derive(Default)]
pub struct Summary {
    pub totals: BTreeMap<&'static str, Stats>,
    pub reattach_count: u64,
    /// (oracle, case) -> smallest iter_seed that reproduced a FAIL.
    pub failing_cases: BTreeMap<(&'static str, &'static str), u64>,
    /// (oracle, case) -> smallest iter_seed that produced a SKIP.
    pub skipping_cases: BTreeMap<(&'static str, &'static str), u64>,
    /// (oracle, case) -> smallest iter_seed that produced a DEGRADED.
    pub degraded_cases: BTreeMap<(&'static str, &'static str), u64>,
}

impl Summary {
    pub fn record(
        &mut self,
        outcomes: &[crate::silicon_oracle::CaseOutcome],
        iter_seed_for_fail: u64,
    ) {
        use crate::silicon_oracle::Verdict;
        for o in outcomes {
            let s = self.totals.entry(o.oracle).or_default();
            match o.verdict {
                Verdict::Pass => s.pass += 1,
                Verdict::Fail => {
                    s.fail += 1;
                    Self::insert_smallest(&mut self.failing_cases, o.oracle, o.case, iter_seed_for_fail);
                }
                Verdict::Skip => {
                    s.skip += 1;
                    Self::insert_smallest(&mut self.skipping_cases, o.oracle, o.case, iter_seed_for_fail);
                }
                Verdict::Degraded => {
                    s.degraded += 1;
                    Self::insert_smallest(&mut self.degraded_cases, o.oracle, o.case, iter_seed_for_fail);
                }
            }
        }
    }

    fn insert_smallest(
        map: &mut BTreeMap<(&'static str, &'static str), u64>,
        oracle: &'static str,
        case: &'static str,
        seed: u64,
    ) {
        let key = (oracle, case);
        map.entry(key)
            .and_modify(|old| {
                if seed < *old {
                    *old = seed;
                }
            })
            .or_insert(seed);
    }

    pub fn total_fail(&self) -> u64 {
        self.totals.values().map(|s| s.fail).sum()
    }

    pub fn total_skip(&self) -> u64 {
        self.totals.values().map(|s| s.skip).sum()
    }

    pub fn total_degraded(&self) -> u64 {
        self.totals.values().map(|s| s.degraded).sum()
    }

    pub fn total_pass(&self) -> u64 {
        self.totals.values().map(|s| s.pass).sum()
    }

    /// Print the summary block. `header` lets the per-binary orchestrator
    /// brand its banner ("test_silicon summary" vs "test_silicon_rp2040
    /// summary").
    pub fn print(&self, header: &str, iterations: u64) {
        println!();
        println!("================ {header} ================");
        println!("iterations:   {iterations}");
        for (oracle, s) in &self.totals {
            println!(
                "  {:<8} pass={:>6}  fail={:>6}  skip={:>6}  degraded={:>6}",
                oracle, s.pass, s.fail, s.skip, s.degraded,
            );
        }
        println!("reattach_count:    {}", self.reattach_count);
        if self.failing_cases.is_empty() {
            println!("failing cases: none");
        } else {
            println!("failing cases (oracle / case / smallest-repro-seed):");
            for ((oracle, case), seed) in &self.failing_cases {
                println!("  {oracle:<8} {case:<40} seed={seed}");
            }
        }
        if !self.skipping_cases.is_empty() {
            println!("skipped cases (oracle / case / smallest-repro-seed):");
            for ((oracle, case), seed) in &self.skipping_cases {
                println!("  {oracle:<8} {case:<40} seed={seed}");
            }
        }
        if !self.degraded_cases.is_empty() {
            println!("degraded cases (oracle / case / smallest-repro-seed):");
            for ((oracle, case), seed) in &self.degraded_cases {
                println!("  {oracle:<8} {case:<40} seed={seed}");
            }
        }
        println!("======================================================");
    }
}

// ---------------------------------------------------------------------------
// Attach / reattach helpers
// ---------------------------------------------------------------------------

/// Open a fresh probe-rs Session for `chip` (e.g. `"rp2040"`,
/// `"rp2350"`). Optionally selects a specific probe by `VID:PID:SERIAL`.
/// Reset+halts core 0 on success so the first oracle finds the target in
/// a known state — matches the standalone oracle binaries' first action.
pub fn attach(
    chip: &str,
    probe: Option<&DebugProbeSelector>,
) -> Result<Session, probe_rs::Error> {
    let mut session = match probe {
        None => Session::auto_attach(chip, SessionConfig::default())?,
        Some(selector) => {
            let probe = Lister::new().open(selector.clone())?;
            probe.attach(chip, Permissions::default())?
        }
    };
    session
        .core(0)?
        .reset_and_halt(Duration::from_millis(500))?;
    Ok(session)
}

/// Bounded retry loop: try `max_attempts` times with a short sleep
/// between attempts, returning the last error if all attempts fail.
/// Useful for the initial attach where transient USB hiccups are
/// common on RP2040 / Pico Probe combinations.
pub fn attach_with_retries(
    chip: &str,
    probe: Option<&DebugProbeSelector>,
    max_attempts: usize,
) -> Result<Session, probe_rs::Error> {
    let mut last_err: Option<probe_rs::Error> = None;
    for attempt in 0..max_attempts {
        if attempt > 0 {
            tracing::info!("attach retry {attempt}/{max_attempts}");
            thread::sleep(Duration::from_millis(300));
        }
        match attach(chip, probe) {
            Ok(s) => return Ok(s),
            Err(e) => last_err = Some(e),
        }
    }
    Err(last_err.expect("attach_with_retries: max_attempts must be >= 1"))
}

/// Re-attach with the HLD soak schedule: sleep 1s, then retry every 5s
/// up to 60s. Returns the fresh Session or the last error stringified.
pub fn reattach_with_retries(
    chip: &str,
    probe: Option<&DebugProbeSelector>,
) -> Result<Session, String> {
    thread::sleep(REATTACH_INITIAL_SLEEP);
    let deadline = Instant::now() + REATTACH_TIMEOUT;
    loop {
        match attach(chip, probe) {
            Ok(s) => return Ok(s),
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(e.to_string());
                }
                thread::sleep(REATTACH_RETRY);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::silicon_oracle::{CaseOutcome, Verdict};
    use rand::SeedableRng;

    #[test]
    fn iter_seed_is_deterministic() {
        assert_eq!(iter_seed(42, 7), 49);
        assert_eq!(iter_seed(42, 7), iter_seed(42, 7));
        // Wrapping semantics.
        assert_eq!(iter_seed(u64::MAX, 1), 0);
    }

    #[test]
    fn shuffle_is_a_permutation() {
        let original: Vec<u32> = (0..16).collect();
        let mut a = original.clone();
        let mut b = original.clone();
        let mut rng_a = StdRng::seed_from_u64(12345);
        let mut rng_b = StdRng::seed_from_u64(12345);
        shuffle_in_place(&mut a, &mut rng_a);
        shuffle_in_place(&mut b, &mut rng_b);
        assert_eq!(a, b);
        assert_eq!(a.len(), original.len());
        let mut sorted_a = a.clone();
        sorted_a.sort();
        assert_eq!(sorted_a, original);
    }

    #[test]
    fn shuffle_changes_order() {
        let original: Vec<u32> = (0..256).collect();
        let mut shuffled = original.clone();
        let mut rng = StdRng::seed_from_u64(1);
        shuffle_in_place(&mut shuffled, &mut rng);
        assert_ne!(shuffled, original);
    }

    #[test]
    fn parse_duration_30m() {
        let d = parse_duration("30m").unwrap();
        assert_eq!(d, Duration::from_secs(30 * 60));
    }

    #[test]
    fn parse_duration_4h() {
        let d = parse_duration("4h").unwrap();
        assert_eq!(d, Duration::from_secs(4 * 3600));
    }

    #[test]
    fn parse_duration_7d() {
        let d = parse_duration("7d").unwrap();
        assert_eq!(d, Duration::from_secs(7 * 24 * 3600));
    }

    #[test]
    fn parse_duration_rejects_garbage() {
        assert!(parse_duration("bogus").is_err());
        assert!(parse_duration("").is_err());
    }

    #[test]
    fn fmt_elapsed_seconds() {
        assert_eq!(fmt_elapsed(Duration::from_secs(0)), "[+00:00:00]");
        assert_eq!(fmt_elapsed(Duration::from_secs(59)), "[+00:00:59]");
    }

    #[test]
    fn fmt_elapsed_minutes() {
        assert_eq!(fmt_elapsed(Duration::from_secs(60)), "[+00:01:00]");
        assert_eq!(fmt_elapsed(Duration::from_secs(3599)), "[+00:59:59]");
    }

    #[test]
    fn fmt_elapsed_hours() {
        assert_eq!(fmt_elapsed(Duration::from_secs(3600)), "[+01:00:00]");
        assert_eq!(
            fmt_elapsed(Duration::from_secs(2 * 3600 + 30 * 60 + 15)),
            "[+02:30:15]"
        );
    }

    #[test]
    fn fmt_elapsed_large() {
        let secs = 48 * 3600 + 3 * 60 + 4;
        assert_eq!(fmt_elapsed(Duration::from_secs(secs)), "[+48:03:04]");
    }

    #[test]
    fn errors_log_path_carries_pid_and_prefix() {
        let p = errors_log_path("test_silicon_rp2040");
        let s = p.to_string_lossy().to_string();
        assert!(s.starts_with("fuzz-runs/test_silicon_rp2040."));
        assert!(s.ends_with(".errors.log"));
    }

    #[test]
    fn validator_passes_on_clean_names() {
        let names = ["alpha", "beta", "gamma", "delta_x"];
        assert!(validate_catalogue_names_are_unique(&names).is_ok());
    }

    #[test]
    fn validator_catches_substring() {
        let names = ["bank_ldr_b0", "bank_ldr", "cycle_foo"];
        let err = validate_catalogue_names_are_unique(&names)
            .expect_err("substring must fail the validator");
        assert!(err.contains("bank_ldr"), "err must cite inner name: {err}");
        assert!(
            err.contains("bank_ldr_b0"),
            "err must cite outer name: {err}"
        );
    }

    #[test]
    fn validator_catches_duplicate() {
        let names = ["foo", "bar", "foo"];
        let err = validate_catalogue_names_are_unique(&names)
            .expect_err("duplicate must fail the validator");
        assert!(err.contains("duplicate"), "{err}");
        assert!(err.contains("foo"), "{err}");
    }

    #[test]
    fn name_interner_returns_same_static_on_repeat() {
        let mut i = NameInterner::default();
        let a = i.intern("watchdog_timeout");
        let b = i.intern("watchdog_timeout");
        assert_eq!(a.as_ptr(), b.as_ptr());
        let c = i.intern("probe_error");
        assert_ne!(a.as_ptr(), c.as_ptr());
        assert_eq!(i.seen.len(), 2);
    }

    #[test]
    fn summary_failing_case_dedup_keeps_smallest_seed() {
        let mut s = Summary::default();
        let oc = |case_id: &'static str| CaseOutcome {
            oracle: "cycle",
            case: case_id,
            verdict: Verdict::Fail,
            detail: "delta=-2".into(),
            elapsed_ms: 7,
        };
        s.record(&[oc("backward_branch_large")], 100);
        s.record(&[oc("backward_branch_large")], 50);
        s.record(&[oc("backward_branch_large")], 75);
        s.record(&[oc("nop_chain_8")], 200);

        assert_eq!(s.failing_cases.len(), 2);
        let key1 = ("cycle", "backward_branch_large");
        let key2 = ("cycle", "nop_chain_8");
        assert_eq!(s.failing_cases.get(&key1), Some(&50));
        assert_eq!(s.failing_cases.get(&key2), Some(&200));
        assert_eq!(s.total_fail(), 4);
    }

    #[test]
    fn summary_pass_outcomes_do_not_appear_in_failing_cases() {
        let mut s = Summary::default();
        s.record(&[CaseOutcome::pass("periph", "pio0_nop_loop", 12)], 42);
        assert!(s.failing_cases.is_empty());
        assert!(s.skipping_cases.is_empty());
        assert!(s.degraded_cases.is_empty());
        assert_eq!(s.totals.get("periph").map(|x| x.pass), Some(1));
    }

    #[test]
    fn summary_records_skip_independently() {
        let mut s = Summary::default();
        s.record(
            &[CaseOutcome::skip(
                "probe_diff",
                "filter_gap_undef",
                "UNDEF_ON_SILICON: pc=0x00000004",
                3,
            )],
            7,
        );
        assert_eq!(s.totals.get("probe_diff").map(|x| x.skip), Some(1));
        assert_eq!(s.totals.get("probe_diff").map(|x| x.fail), Some(0));
        assert_eq!(s.total_skip(), 1);
        assert_eq!(s.total_fail(), 0);
        assert!(s.failing_cases.is_empty());
        assert_eq!(s.skipping_cases.len(), 1);
    }

    #[test]
    fn summary_records_degraded_independently() {
        let mut s = Summary::default();
        s.record(
            &[CaseOutcome::degraded(
                "probe_diff",
                "usb_stall_case",
                "probe-rs error: USB stall",
                4,
            )],
            9,
        );
        assert_eq!(s.totals.get("probe_diff").map(|x| x.degraded), Some(1));
        assert_eq!(s.total_degraded(), 1);
        assert_eq!(s.total_fail(), 0);
        assert!(s.failing_cases.is_empty());
        assert_eq!(s.degraded_cases.len(), 1);
    }

    #[test]
    fn summary_skip_and_degraded_track_smallest_seed() {
        let mut s = Summary::default();
        let skip = |c: &'static str| CaseOutcome::skip("probe_diff", c, "u", 1);
        let degr = |c: &'static str| CaseOutcome::degraded("probe_diff", c, "d", 1);
        s.record(&[skip("a")], 100);
        s.record(&[skip("a")], 50);
        s.record(&[skip("a")], 200);
        s.record(&[degr("b")], 80);
        s.record(&[degr("b")], 40);

        assert_eq!(s.totals.get("probe_diff").map(|x| x.skip), Some(3));
        assert_eq!(s.totals.get("probe_diff").map(|x| x.degraded), Some(2));
        let key_skip = ("probe_diff", "a");
        let key_degr = ("probe_diff", "b");
        assert_eq!(s.skipping_cases.get(&key_skip), Some(&50));
        assert_eq!(s.degraded_cases.get(&key_degr), Some(&40));
    }

    #[test]
    fn summary_mixed_outcomes_separate_buckets() {
        let mut s = Summary::default();
        s.record(
            &[
                CaseOutcome::pass("o", "p1", 1),
                CaseOutcome::fail("o", "f1", "x", 1),
                CaseOutcome::skip("o", "s1", "x", 1),
                CaseOutcome::degraded("o", "d1", "x", 1),
            ],
            5,
        );
        let st = s.totals.get("o").expect("oracle stats");
        assert_eq!(st.pass, 1);
        assert_eq!(st.fail, 1);
        assert_eq!(st.skip, 1);
        assert_eq!(st.degraded, 1);
        assert_eq!(s.failing_cases.len(), 1);
        assert_eq!(s.skipping_cases.len(), 1);
        assert_eq!(s.degraded_cases.len(), 1);
    }
}

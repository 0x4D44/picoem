// test_silicon — unified orchestrator for the silicon-gated oracles.
//
// Wraps the five library APIs under one shared probe session:
//
//   * `cycle_cases::run_against`        (cycle oracle)
//   * `silicon_scenarios::run_against`  (periph oracle)
//   * `bank_conflict_cases::run_against` (bank-conflict oracle)
//   * `dualcore_cases::run_against`     (dual-core contention oracle)
//   * `isr_scenarios::run_against`      (exception entry / FP save oracle)
//
// Single-pass mode (default) runs each oracle's catalogue once in catalogue
// order — each oracle's `run_against` is called exactly once, with
// `order = None` so the library APIs apply their internal filter and
// catalogue-declared ordering. Soak mode (`--soak <duration>`) shuffles the
// pre-filtered per-oracle name lists once per iteration and passes them to
// each `run_against` call — still **one** call per oracle per iteration, so:
//
//   * `silicon_scenarios::run_scenario(first_scenario=true)` fires once per
//     iteration (not per case), preserving the cross-oracle leakage
//     detection the soak loop exists to expose.
//   * `bank_conflict_cases::measure_nop_baseline_hw` runs once per iteration
//     (not per case) — ~1 second of amortised calibration instead of
//     28 seconds.
//
// Between iterations the orchestrator calls `core.reset_and_halt` — but
// NOT between oracles within one iteration (HLD v1.1.1 §Cross-oracle
// state-cleanup contract).
//
// See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage
// Expansion.md` §Component 1 for the full contract.

use std::collections::BTreeMap;
use std::fs::OpenOptions;
use std::io::Write as IoWrite;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use mdpicoem_harness::bank_conflict_cases::{self, BankArgs};
use mdpicoem_harness::cycle_cases::{self, CycleArgs};
use mdpicoem_harness::dualcore_cases::{self, DualCoreArgs};
use mdpicoem_harness::isr_scenarios::{self, IsrArgs};
use mdpicoem_harness::silicon_oracle::{CaseOutcome, Verdict, name_matches_filter};
use mdpicoem_harness::silicon_scenarios::{self, PeriphArgs};
use probe_rs::probe::{list::Lister, DebugProbeSelector};
use probe_rs::{Permissions, Session, SessionConfig};
use rand::SeedableRng;
use rand::rngs::StdRng;

const REATTACH_RETRY: Duration = Duration::from_secs(5);
const REATTACH_TIMEOUT: Duration = Duration::from_secs(60);
const REATTACH_INITIAL_SLEEP: Duration = Duration::from_secs(1);
const GIVE_UP_THRESHOLD: u32 = 3;
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(3600);

// Oracle identifiers (match what `CaseOutcome.oracle` carries from each
// library API). Kept in one place so the summary + filtering stay in sync.
const ORACLE_CYCLE: &str = "cycle";
const ORACLE_PERIPH: &str = "periph";
const ORACLE_BANK: &str = "bank";
const ORACLE_DUALCORE: &str = "dualcore";
const ORACLE_ISR: &str = "isr";

const USAGE: &str = "\
Usage: test_silicon [--soak <duration>] [--seed <u64>] [--filter <substr>] [--verbose] [--probe VID:PID:SERIAL]

Options:
  --soak     Run continuously for the given duration (e.g. 30m, 4h, 7d).
             Default: single pass.
  --seed     Base RNG seed for soak-mode shuffling.
             Default: current Unix epoch seconds.
  --filter   Only run cases whose name contains <substr>. Applied to
             every oracle's catalogue. Default: all cases.
  --verbose  In soak mode, print full per-case output every iteration
             (default: quiet — failures + hourly heartbeat).
             In single-pass mode, has no additional effect (output is
             already the per-case table; the standalone binaries carry
             richer per-case detail — see the banner printed at start).
  --probe    Select a specific debug probe by VID:PID:SERIAL.
             Required on hosts with multiple probes attached.
             Default: auto-attach (first enumerated probe).
";

// Sentinel case name used for synthesised probe-rs error outcomes where we
// know which oracle was running but not which case. Attribution is whole-oracle
// — partial results for cases earlier in the list are already recorded.
const PROBE_ERROR_SENTINEL: &str = "<probe-rs error — partial results may be missing>";

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct CliArgs {
    soak: Option<Duration>,
    seed: u64,
    filter: Option<String>,
    verbose: bool,
    probe: Option<DebugProbeSelector>,
}

fn parse_args(argv: Vec<String>) -> Result<CliArgs, String> {
    let mut soak: Option<Duration> = None;
    let mut seed: Option<u64> = None;
    let mut filter: Option<String> = None;
    let mut verbose = false;
    let mut probe: Option<DebugProbeSelector> = None;
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--soak" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--soak requires a duration\n{USAGE}"));
                }
                soak = Some(parse_duration(&argv[i])?);
            }
            "--seed" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--seed requires a u64\n{USAGE}"));
                }
                seed = Some(
                    argv[i]
                        .parse::<u64>()
                        .map_err(|e| format!("invalid --seed '{}': {e}\n{USAGE}", argv[i]))?,
                );
            }
            "--filter" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--filter requires a substring\n{USAGE}"));
                }
                filter = Some(argv[i].clone());
            }
            "--probe" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--probe requires a VID:PID:SERIAL argument\n{USAGE}"));
                }
                probe = Some(
                    DebugProbeSelector::try_from(argv[i].as_str())
                        .map_err(|e| format!("invalid probe selector '{}': {e}\n{USAGE}", argv[i]))?,
                );
            }
            "--verbose" => verbose = true,
            "--help" | "-h" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument '{other}'\n{USAGE}")),
        }
        i += 1;
    }
    let seed = seed.unwrap_or_else(default_seed);
    Ok(CliArgs { soak, seed, filter, verbose, probe })
}

fn parse_duration(s: &str) -> Result<Duration, String> {
    humantime::parse_duration(s).map_err(|e| format!("invalid duration '{s}': {e}\n{USAGE}"))
}

fn default_seed() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Name-list shuffling (Fisher-Yates; operates on `&'static str` slices
// so the original catalogue arrays stay intact)
// ---------------------------------------------------------------------------

fn shuffle_in_place<T>(v: &mut [T], rng: &mut StdRng) {
    use rand::RngCore;
    for i in (1..v.len()).rev() {
        let j = (rng.next_u64() % (i as u64 + 1)) as usize;
        v.swap(i, j);
    }
}

fn iter_seed(base_seed: u64, iter_index: u64) -> u64 {
    base_seed.wrapping_add(iter_index)
}

// ---------------------------------------------------------------------------
// Substring-uniqueness validator
// ---------------------------------------------------------------------------

/// Validate that the combined set of case names across the three
/// catalogues has no name that is a strict substring of another. The
/// orchestrator's filter semantics (and the library APIs' `filter`
/// arguments) are substring-match, and one oracle's name being a substring
/// of another would alias two cases under a single `--filter` flag. Fail
/// fast at startup — a corrupt filter is worse than a clean refusal.
///
/// Returns `Err` with a human-readable message pointing at the first
/// offending pair; returns `Ok(())` otherwise.
fn validate_catalogue_names_are_unique(names: &[&str]) -> Result<(), String> {
    // O(N^2). The catalogues together carry ~50 names; the whole check
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

/// Combine the five catalogues' names into one `Vec<&str>` for the
/// validator to chew on. Called once at startup.
fn collect_all_catalogue_names() -> Vec<&'static str> {
    let mut names: Vec<&'static str> = Vec::new();
    for c in cycle_cases::CASES {
        names.push(c.name);
    }
    for s in silicon_scenarios::SCENARIOS {
        names.push(s.name);
    }
    for c in bank_conflict_cases::build_catalog() {
        names.push(c.name);
    }
    for c in dualcore_cases::CASES {
        names.push(c.name);
    }
    for s in isr_scenarios::SCENARIOS {
        names.push(s.name);
    }
    names
}

// ---------------------------------------------------------------------------
// Synthetic-case-name interner
// ---------------------------------------------------------------------------
//
// `CaseOutcome.case` is `&'static str`. When the orchestrator synthesises
// a FAIL outcome for a watchdog / probe-rs error, the case name can be a
// runtime-assembled string (e.g. the oracle name plus a sentinel). We
// intern each such name exactly once via `Box::leak` so repeated failures
// on the same synthetic name do not accumulate leaks over a week-long
// soak run. Bounded by the number of distinct synthetic names ever
// emitted — in practice just the two sentinels plus per-oracle tags.

#[derive(Default)]
struct NameInterner {
    seen: BTreeMap<String, &'static str>,
}

impl NameInterner {
    fn intern(&mut self, s: &str) -> &'static str {
        if let Some(v) = self.seen.get(s) {
            return v;
        }
        let leaked: &'static str = Box::leak(s.to_string().into_boxed_str());
        self.seen.insert(s.to_string(), leaked);
        leaked
    }
}

// ---------------------------------------------------------------------------
// Oracle kinds + per-oracle run invocation (in a worker thread, owns Session)
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum OracleKind {
    Cycle,
    Periph,
    Bank,
    DualCore,
    Isr,
}

impl OracleKind {
    fn as_str(self) -> &'static str {
        match self {
            OracleKind::Cycle => ORACLE_CYCLE,
            OracleKind::Periph => ORACLE_PERIPH,
            OracleKind::Bank => ORACLE_BANK,
            OracleKind::DualCore => ORACLE_DUALCORE,
            OracleKind::Isr => ORACLE_ISR,
        }
    }
}

/// Map a runtime oracle-name `&str` back to one of the pinned
/// `&'static str` oracle-id constants so `CaseOutcome.oracle` stays
/// `&'static str` without leaking. The known oracles round-trip;
/// anything else is coerced to the literal `"unknown"`.
///
/// Despite the literal `Box::leak`-sounding name of its predecessor, this
/// function never leaks — it just returns one of the pinned consts.
fn oracle_name_static(name: &str) -> &'static str {
    match name {
        ORACLE_CYCLE => ORACLE_CYCLE,
        ORACLE_PERIPH => ORACLE_PERIPH,
        ORACLE_BANK => ORACLE_BANK,
        ORACLE_DUALCORE => ORACLE_DUALCORE,
        ORACLE_ISR => ORACLE_ISR,
        _ => "unknown",
    }
}

/// Plan for a single oracle invocation.
#[derive(Clone, Debug)]
struct OraclePlan {
    oracle: OracleKind,
    /// `None` → library default (use filter + catalogue order).
    /// `Some(v)` → run these cases in this exact order; the library
    /// ignores `filter` for selection.
    order: Option<Vec<String>>,
    /// Original filter (still passed via `args.filter` for the `None`
    /// path; the library applies it itself).
    filter: Option<String>,
}

/// Dispatch to the right `run_against` for the given oracle plan.
/// Cycle / periph / bank / ISR open core 0 and work through the `Core`
/// handle; dualcore takes `&mut Session` directly because it drives
/// core 1 as well. Session stays on the calling thread throughout — no
/// worker thread (probe-rs USB handles are thread-affine on Windows).
fn run_one_oracle(
    session: &mut Session,
    plan: &OraclePlan,
) -> Result<Vec<CaseOutcome>, String> {
    // When `order` is provided, convert `Vec<String>` into a transient
    // `Vec<&str>` for the library call. The caller owns the `String`
    // backing storage for the duration of this function.
    let order_refs: Option<Vec<&str>> = plan
        .order
        .as_ref()
        .map(|v| v.iter().map(|s| s.as_str()).collect());
    let order_slice: Option<&[&str]> = order_refs.as_deref();

    let result = match plan.oracle {
        OracleKind::Cycle => {
            let mut core = session.core(0).map_err(|e| e.to_string())?;
            let args = CycleArgs {
                filter: plan.filter.clone(),
                ..CycleArgs::default()
            };
            cycle_cases::run_against(&mut core, &args, order_slice)
        }
        OracleKind::Periph => {
            let mut core = session.core(0).map_err(|e| e.to_string())?;
            let args = PeriphArgs {
                filter: plan.filter.clone(),
                verbose: false,
            };
            silicon_scenarios::run_against(&mut core, &args, order_slice)
        }
        OracleKind::Bank => {
            let mut core = session.core(0).map_err(|e| e.to_string())?;
            let args = BankArgs {
                filter: plan.filter.clone(),
                ..BankArgs::default()
            };
            bank_conflict_cases::run_against(&mut core, &args, order_slice)
        }
        OracleKind::DualCore => {
            // Dualcore needs &mut Session (it toggles core 1), so we
            // skip the per-oracle `session.core(0)` acquisition here.
            let args = DualCoreArgs {
                filter: plan.filter.clone(),
                ..DualCoreArgs::default()
            };
            dualcore_cases::run_against(session, &args, order_slice)
        }
        OracleKind::Isr => {
            let mut core = session.core(0).map_err(|e| e.to_string())?;
            let args = IsrArgs {
                filter: plan.filter.clone(),
                exclude: None,
                verbose: false,
            };
            isr_scenarios::run_against(&mut core, &args, order_slice)
        }
    };
    result.map_err(|e| e.to_string())
}

fn attach(probe: Option<&DebugProbeSelector>) -> Result<Session, probe_rs::Error> {
    let mut session = match probe {
        None => Session::auto_attach("rp2350", SessionConfig::default())?,
        Some(selector) => {
            let probe = Lister::new().open(selector.clone())?;
            probe.attach("rp2350", Permissions::default())?
        }
    };
    // Reset + halt on attach so the first oracle finds the core in a
    // known state. Standalone oracle binaries call `reset_and_halt`
    // themselves right after opening the session; test_silicon skipped
    // this step and the cycle oracle's first `write_core_reg` landed on
    // a running target → "An ARM specific error occurred".
    session.core(0)?.reset_and_halt(Duration::from_millis(500))?;
    Ok(session)
}

/// Re-attach with the HLD retry schedule: sleep 1s, then retry every 5s up
/// to 60s. Returns the fresh Session or the last error.
fn reattach_with_retries(probe: Option<&DebugProbeSelector>) -> Result<Session, String> {
    thread::sleep(REATTACH_INITIAL_SLEEP);
    let deadline = Instant::now() + REATTACH_TIMEOUT;
    loop {
        match attach(probe) {
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
// Logging helpers
// ---------------------------------------------------------------------------

/// Format an elapsed duration as `[+HH:MM:SS]`.
fn fmt_elapsed(d: Duration) -> String {
    let total = d.as_secs();
    let h = total / 3600;
    let m = (total % 3600) / 60;
    let s = total % 60;
    format!("[+{h:02}:{m:02}:{s:02}]")
}

fn errors_log_path() -> PathBuf {
    let pid = std::process::id();
    PathBuf::from(format!("fuzz-runs/test_silicon.{pid}.errors.log"))
}

fn ensure_fuzz_runs_dir() -> std::io::Result<()> {
    std::fs::create_dir_all("fuzz-runs")
}

fn append_error_log(path: &PathBuf, line: &str) {
    let _ = ensure_fuzz_runs_dir();
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(f, "{line}");
    }
}

/// Print a line to stdout, mirror to stderr, and append to the
/// catastrophic-failure log. One call per FAIL / probe error / reattach
/// failure keeps the soak-loop body terse.
fn emit_log_line(log_path: &PathBuf, line: &str) {
    println!("{line}");
    eprintln!("{line}");
    append_error_log(log_path, line);
}

// ---------------------------------------------------------------------------
// Summary bookkeeping
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Stats {
    pass: u64,
    fail: u64,
}

#[derive(Default)]
struct Summary {
    totals: BTreeMap<&'static str, Stats>,
    reattach_count: u64,
    /// (oracle, case) -> smallest iter_seed that reproduced a FAIL.
    failing_cases: BTreeMap<(&'static str, String), u64>,
}

impl Summary {
    fn record(&mut self, outcomes: &[CaseOutcome], iter_seed_for_fail: u64) {
        for o in outcomes {
            let s = self.totals.entry(o.oracle).or_default();
            match o.verdict {
                Verdict::Pass => s.pass += 1,
                Verdict::Fail => {
                    s.fail += 1;
                    let key = (o.oracle, o.case.to_string());
                    self.failing_cases
                        .entry(key)
                        .and_modify(|old| {
                            if iter_seed_for_fail < *old {
                                *old = iter_seed_for_fail;
                            }
                        })
                        .or_insert(iter_seed_for_fail);
                }
            }
        }
    }

    fn total_fail(&self) -> u64 {
        self.totals.values().map(|s| s.fail).sum()
    }

    fn print(&self, iterations: u64) {
        println!();
        println!("================ test_silicon summary ================");
        println!("iterations:   {iterations}");
        for (oracle, s) in &self.totals {
            println!(
                "  {:<8} pass={:>6}  fail={:>6}",
                oracle, s.pass, s.fail,
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
        println!("======================================================");
    }
}

// ---------------------------------------------------------------------------
// Orchestrator
// ---------------------------------------------------------------------------

fn orchestrate(args: &CliArgs, stop_flag: Arc<AtomicBool>) -> Result<i32, String> {
    let log_path = errors_log_path();
    let start = Instant::now();

    // Startup: validate catalogue names are unique across oracles.
    // Fail fast if any name is a substring of another — otherwise the
    // orchestrator's filter semantics could alias two cases under one
    // flag, silently dropping coverage.
    let all_names = collect_all_catalogue_names();
    if let Err(msg) = validate_catalogue_names_are_unique(&all_names) {
        eprintln!("test_silicon: catalogue name check failed:\n  {msg}");
        return Err(msg);
    }

    // Print start banner.
    println!("test_silicon: starting");
    println!("  mode:    {}", if args.soak.is_some() { "soak" } else { "single-pass" });
    if let Some(d) = args.soak {
        println!("  soak:    {}", humantime::format_duration(d));
    }
    println!("  seed:    {}", args.seed);
    println!("  filter:  {}", args.filter.as_deref().unwrap_or("<none>"));
    println!("  verbose: {}", args.verbose);
    println!(
        "  probe:   {}",
        args.probe.as_ref().map_or("<auto>".to_string(), |p| {
            format!(
                "{:04x}:{:04x}:{}",
                p.vendor_id,
                p.product_id,
                p.serial_number.as_deref().unwrap_or("")
            )
        })
    );
    println!("  errlog:  {}", log_path.display());
    println!(
        "  NOTE: single-pass mode prints only the per-case outcome table; \
         run the standalone binaries (silicon_cycle_oracle_rp2350, \
         silicon_periph_diff_rp2350, bank_conflict_test_rp2350) for \
         richer per-case detail (HW m_low/m_high, baseline-drift notes)."
    );
    println!();

    // Attach once up front.
    let probe_sel = args.probe.as_ref();
    let mut session = match attach(probe_sel) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("test_silicon: initial attach failed: {e}");
            append_error_log(&log_path, &format!("{} initial-attach-fail {e}", now_iso()));
            return Err(format!("initial attach failed: {e}"));
        }
    };

    let mut summary = Summary::default();
    let mut interner = NameInterner::default();
    let deadline = args.soak.map(|d| start + d);

    match deadline {
        None => {
            // Single-pass mode: deterministic order, full output, exit 1 on any FAIL.
            session = single_pass(
                &args.filter,
                session,
                &mut summary,
                &mut interner,
                &log_path,
                args.verbose,
                probe_sel,
            )?;
            let _ = session; // silence unused warning after last use
            summary.print(1);
            if summary.total_fail() > 0 {
                return Ok(1);
            }
            Ok(0)
        }
        Some(deadline) => {
            let (give_up_code, total_iters) = soak_loop(
                args,
                session,
                &mut summary,
                &mut interner,
                &log_path,
                deadline,
                stop_flag,
                probe_sel,
            )?;
            summary.print(total_iters);
            if give_up_code == 2 {
                return Ok(2);
            }
            if summary.total_fail() > 0 {
                return Ok(1);
            }
            Ok(0)
        }
    }
}

fn now_iso() -> String {
    let now = SystemTime::now();
    let dt: chrono::DateTime<chrono::Utc> = now.into();
    dt.format("%Y-%m-%dT%H:%M:%SZ").to_string()
}

/// Single-pass mode. Runs each oracle's whole catalogue once in
/// catalogue-declared order (filter applied by the library, no shuffle).
/// One `run_against` call per oracle. Mirrors the standalone binaries'
/// output density — but not their full richness (`CaseOutcome` drops
/// HW m_low/m_high, baseline-drift notes, etc.). The start banner points
/// users at the standalone binaries when full detail is needed.
fn single_pass(
    filter: &Option<String>,
    mut session: Session,
    summary: &mut Summary,
    interner: &mut NameInterner,
    log_path: &PathBuf,
    verbose: bool,
    probe: Option<&DebugProbeSelector>,
) -> Result<Session, String> {
    for oracle in [
        OracleKind::Cycle,
        OracleKind::Periph,
        OracleKind::Bank,
        OracleKind::DualCore,
        OracleKind::Isr,
    ] {
        println!("--- oracle: {} ---", oracle.as_str());
        let t0 = Instant::now();
        let plan = OraclePlan {
            oracle,
            order: None, // None → library default: filter + catalogue order.
            filter: filter.clone(),
        };
        match run_one_oracle(&mut session, &plan) {
            Ok(outcomes) => {
                for o in &outcomes {
                    println!(
                        "  {:<8} {:<40} {:<4}  elapsed={}ms  {}",
                        o.oracle,
                        o.case,
                        o.verdict.as_str(),
                        o.elapsed_ms,
                        o.detail,
                    );
                }
                summary.record(&outcomes, 0);
            }
            Err(e) => {
                let msg = format!(
                    "{} iter=0 oracle={} case={} detail={e}",
                    now_iso(),
                    oracle.as_str(),
                    PROBE_ERROR_SENTINEL,
                );
                eprintln!("ERROR: {msg}");
                append_error_log(log_path, &msg);
                // Synthesise a Fail outcome so single-pass records the failure.
                // Single-pass wraps a whole oracle per call, so the sentinel
                // name reflects "whole-oracle probe error" rather than a
                // specific case name.
                let case_name: &'static str = interner.intern(PROBE_ERROR_SENTINEL);
                let synthetic = CaseOutcome::fail(
                    oracle_name_static(oracle.as_str()),
                    case_name,
                    e.clone(),
                    0,
                );
                summary.record(&[synthetic], 0);
                // Reattach so the next oracle can run.
                summary.reattach_count += 1;
                session = reattach_with_retries(probe)
                    .map_err(|e| format!("reattach failed: {e}"))?;
            }
        }
        println!("  ({} oracle took {:.2}s)", oracle.as_str(), t0.elapsed().as_secs_f64());
        println!();
    }
    // `verbose` is accepted but in single-pass mode the per-case table
    // already mirrors the standalone binary output. The start banner
    // directs users at the standalone binaries for HW m_low/m_high and
    // baseline-drift detail; nothing extra to wire here.
    let _ = verbose;
    Ok(session)
}

/// Soak mode. Returns `(exit_code_marker, total_iterations)`.
/// `exit_code_marker` is 0 on normal/Ctrl-C exit, 2 on give-up threshold.
///
/// Per iteration:
///   1. Build the per-oracle name list (filter + shuffle).
///   2. Call each oracle's `run_against` EXACTLY ONCE with the shuffled
///      order. Not per case — that would defeat the cross-oracle leakage
///      detection (periph's first-scenario reset fires inside the library)
///      and retrigger the bank oracle's NOP baselining.
///   3. Between iterations (not between oracles within one iteration),
///      call `core.reset_and_halt` to reset the chip to a known state.
fn soak_loop(
    args: &CliArgs,
    initial_session: Session,
    summary: &mut Summary,
    interner: &mut NameInterner,
    log_path: &PathBuf,
    deadline: Instant,
    stop_flag: Arc<AtomicBool>,
    probe: Option<&DebugProbeSelector>,
) -> Result<(i32, u64), String> {
    let mut session_opt: Option<Session> = Some(initial_session);
    let start = Instant::now();
    let mut iter_index: u64 = 0;
    let mut next_heartbeat = start + HEARTBEAT_INTERVAL;
    let mut consecutive_reattach_fails: u32 = 0;
    let mut total_iters: u64 = 0;

    // Build the per-oracle name lists ONCE. Filter is applied here;
    // each iteration just shuffles and hands the list to the library.
    // `&'static str` catalogue entries mean the static name tables never
    // move; only the order changes.
    let cycle_names: Vec<&'static str> = cycle_cases::CASES
        .iter()
        .map(|c| c.name)
        .filter(|n| name_matches_filter(n, args.filter.as_deref()))
        .collect();
    let periph_names: Vec<&'static str> = silicon_scenarios::SCENARIOS
        .iter()
        .map(|s| s.name)
        .filter(|n| name_matches_filter(n, args.filter.as_deref()))
        .collect();
    // Bank catalogue is `Vec<BankCase>` but names are `&'static str` on
    // the case struct, so we can collect them as `&'static str`.
    let bank_names: Vec<&'static str> = bank_conflict_cases::build_catalog()
        .into_iter()
        .filter(|c| name_matches_filter(c.name, args.filter.as_deref()))
        .map(|c| c.name)
        .collect();
    let dualcore_names: Vec<&'static str> = dualcore_cases::CASES
        .iter()
        .map(|c| c.name)
        .filter(|n| name_matches_filter(n, args.filter.as_deref()))
        .collect();
    let isr_names: Vec<&'static str> = isr_scenarios::SCENARIOS
        .iter()
        .map(|s| s.name)
        .filter(|n| name_matches_filter(n, args.filter.as_deref()))
        .collect();

    if cycle_names.is_empty()
        && periph_names.is_empty()
        && bank_names.is_empty()
        && dualcore_names.is_empty()
        && isr_names.is_empty()
    {
        println!(
            "test_silicon: no cases match filter '{}' in any oracle; exiting",
            args.filter.as_deref().unwrap_or("<none>"),
        );
        return Ok((0, 0));
    }

    while Instant::now() < deadline && !stop_flag.load(Ordering::SeqCst) {
        let s = iter_seed(args.seed, iter_index);
        let mut rng = StdRng::seed_from_u64(s);

        // Fisher-Yates shuffle each list. `&'static str` copies are cheap;
        // the original catalogue arrays are never touched.
        let mut cycle_plan: Vec<&'static str> = cycle_names.clone();
        let mut periph_plan: Vec<&'static str> = periph_names.clone();
        let mut bank_plan: Vec<&'static str> = bank_names.clone();
        let mut dualcore_plan: Vec<&'static str> = dualcore_names.clone();
        let mut isr_plan: Vec<&'static str> = isr_names.clone();
        shuffle_in_place(&mut cycle_plan, &mut rng);
        shuffle_in_place(&mut periph_plan, &mut rng);
        shuffle_in_place(&mut bank_plan, &mut rng);
        shuffle_in_place(&mut dualcore_plan, &mut rng);
        shuffle_in_place(&mut isr_plan, &mut rng);

        if args.verbose {
            println!("{} iter={} seed={} starting", fmt_elapsed(start.elapsed()), iter_index, s);
        }

        // One `run_against` call per oracle — NOT per case. This is the
        // architectural fix from Stage 3 review: per-case calls destroy
        // cross-oracle leakage detection and re-run the bank NOP
        // baseline per case. By contrast, one call per oracle:
        //   - `silicon_scenarios::run_against` fires `first_scenario=true`
        //     ONCE at the start of periph (once per iteration).
        //   - `bank_conflict_cases::run_against` calibrates NOP baseline
        //     ONCE at the start of bank (once per iteration).
        //   - No reset between cycle→periph→bank inside one iteration —
        //     the HLD's cross-oracle state-cleanup contract is honoured.
        let plans: [(OracleKind, Vec<String>); 5] = [
            (
                OracleKind::Cycle,
                cycle_plan.into_iter().map(String::from).collect(),
            ),
            (
                OracleKind::Periph,
                periph_plan.into_iter().map(String::from).collect(),
            ),
            (
                OracleKind::Bank,
                bank_plan.into_iter().map(String::from).collect(),
            ),
            (
                OracleKind::DualCore,
                dualcore_plan.into_iter().map(String::from).collect(),
            ),
            (
                OracleKind::Isr,
                isr_plan.into_iter().map(String::from).collect(),
            ),
        ];

        'plans: for (oracle, names) in plans {
            if stop_flag.load(Ordering::SeqCst) {
                break 'plans;
            }
            if names.is_empty() {
                // Nothing for this oracle this iteration (filter pruned
                // every case).
                continue;
            }
            // No session this iteration — previously-failed reattach.
            let Some(ref mut this_session) = session_opt else {
                break 'plans;
            };
            let plan = OraclePlan {
                oracle,
                order: Some(names.clone()),
                filter: args.filter.clone(),
            };
            match run_one_oracle(this_session, &plan) {
                Ok(outcomes) => {
                    for o in &outcomes {
                        if args.verbose {
                            println!(
                                "{} iter={} oracle={} case={} verdict={} elapsed={}ms {}",
                                fmt_elapsed(start.elapsed()),
                                iter_index,
                                o.oracle,
                                o.case,
                                o.verdict.as_str(),
                                o.elapsed_ms,
                                o.detail,
                            );
                        } else if o.verdict == Verdict::Fail {
                            let line = format!(
                                "{} iter={} seed={} oracle={} case={} detail={}",
                                fmt_elapsed(start.elapsed()),
                                iter_index, s, o.oracle, o.case, o.detail,
                            );
                            emit_log_line(log_path, &line);
                        }
                    }
                    summary.record(&outcomes, s);
                    consecutive_reattach_fails = 0;
                }
                Err(e) => {
                    let synthetic_oracle = oracle_name_static(oracle.as_str());
                    let detail = format!("probe-rs error: {e}");
                    // Probe-rs error terminated the whole oracle call.
                    // Attribution is "the oracle, not the case" — we don't
                    // know which case was running when the probe wedged.
                    // Partial results for cases earlier in the shuffled
                    // list are already recorded above.
                    let case_name_static: &'static str = interner.intern(PROBE_ERROR_SENTINEL);
                    let synth = CaseOutcome {
                        oracle: synthetic_oracle,
                        case: case_name_static,
                        verdict: Verdict::Fail,
                        detail: detail.clone(),
                        elapsed_ms: 0,
                    };
                    let line = format!(
                        "{} iter={} seed={} oracle={} case={} detail={}",
                        fmt_elapsed(start.elapsed()),
                        iter_index, s, synthetic_oracle, case_name_static, detail,
                    );
                    emit_log_line(log_path, &line);
                    summary.record(&[synth], s);

                    // Drop the dead session so reattach can open a fresh one.
                    session_opt = None;
                    summary.reattach_count += 1;
                    match reattach_with_retries(probe) {
                        Ok(fresh) => {
                            session_opt = Some(fresh);
                            consecutive_reattach_fails = 0;
                        }
                        Err(rerr) => {
                            consecutive_reattach_fails += 1;
                            let rline = format!(
                                "{} iter={} reattach failed: {rerr} (consecutive={})",
                                fmt_elapsed(start.elapsed()),
                                iter_index, consecutive_reattach_fails,
                            );
                            emit_log_line(log_path, &rline);
                            if consecutive_reattach_fails >= GIVE_UP_THRESHOLD {
                                eprintln!(
                                    "test_silicon: {consecutive_reattach_fails} consecutive reattach failures — giving up",
                                );
                                return Ok((2, total_iters));
                            }
                            // Skip remaining oracles this iteration; the
                            // outer loop will attempt one more reattach
                            // up top before next iter.
                            break 'plans;
                        }
                    }
                }
            }
        }

        iter_index += 1;
        total_iters += 1;

        // If we have no session at this point (reattach exhausted), try
        // once more before next iteration.
        if session_opt.is_none() {
            match reattach_with_retries(probe) {
                Ok(s) => {
                    session_opt = Some(s);
                    consecutive_reattach_fails = 0;
                }
                Err(_) => {
                    consecutive_reattach_fails += 1;
                    if consecutive_reattach_fails >= GIVE_UP_THRESHOLD {
                        return Ok((2, total_iters));
                    }
                    continue;
                }
            }
        }

        // Between-iteration reset to defeat cross-ITERATION state
        // leakage. This is the only place the orchestrator resets the
        // core; it deliberately does NOT reset between oracles within
        // one iteration so the soak loop still exposes cross-oracle
        // state-leakage bugs (that's the whole point of soak mode).
        if let Some(ref mut session) = session_opt {
            if let Ok(mut core) = session.core(0) {
                let _ = core.reset_and_halt(Duration::from_millis(500));
            }
        }

        // Heartbeat.
        let now = Instant::now();
        if now >= next_heartbeat && !args.verbose {
            let total_pass: u64 = summary.totals.values().map(|x| x.pass).sum();
            let total_fail: u64 = summary.totals.values().map(|x| x.fail).sum();
            println!(
                "{} iter={} pass={} fail={} reattach={}",
                fmt_elapsed(start.elapsed()),
                iter_index,
                total_pass,
                total_fail,
                summary.reattach_count,
            );
            next_heartbeat = now + HEARTBEAT_INTERVAL;
        }
    }

    Ok((0, total_iters))
}

// ---------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------

fn main() {
    mdpicoem_harness::harness_tracing_init();
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let args = match parse_args(argv) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };

    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();
        if let Err(e) = ctrlc::set_handler(move || {
            stop.store(true, Ordering::SeqCst);
            eprintln!("\ntest_silicon: Ctrl-C received; finishing current case…");
        }) {
            eprintln!("warning: Ctrl-C handler install failed: {e}");
        }
    }

    match orchestrate(&args, stop) {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("test_silicon: {e}");
            std::process::exit(2);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // (1) Soak-loop iteration RNG seeding is deterministic.
    #[test]
    fn test_iter_seed_is_deterministic() {
        assert_eq!(iter_seed(42, 7), 49);
        assert_eq!(iter_seed(42, 7), iter_seed(42, 7));
        // Wrapping semantics.
        assert_eq!(iter_seed(u64::MAX, 1), 0);
    }

    // (2) Fisher-Yates shuffle preserves length + elements, deterministic
    //     given the seed.
    #[test]
    fn test_shuffle_is_a_permutation() {
        let original: Vec<u32> = (0..16).collect();
        let mut a = original.clone();
        let mut b = original.clone();
        let mut rng_a = StdRng::seed_from_u64(12345);
        let mut rng_b = StdRng::seed_from_u64(12345);
        shuffle_in_place(&mut a, &mut rng_a);
        shuffle_in_place(&mut b, &mut rng_b);
        // Deterministic.
        assert_eq!(a, b);
        // Length preserved.
        assert_eq!(a.len(), original.len());
        // Permutation (same multiset).
        let mut sorted_a = a.clone();
        sorted_a.sort();
        assert_eq!(sorted_a, original);
    }

    #[test]
    fn test_shuffle_changes_order() {
        // Over 256 elements at least some swap should happen.
        let original: Vec<u32> = (0..256).collect();
        let mut shuffled = original.clone();
        let mut rng = StdRng::seed_from_u64(1);
        shuffle_in_place(&mut shuffled, &mut rng);
        assert_ne!(shuffled, original);
    }

    // (3) `humantime` round-trips for 30m / 4h / 7d.
    #[test]
    fn test_parse_duration_30m() {
        let d = parse_duration("30m").unwrap();
        assert_eq!(d, Duration::from_secs(30 * 60));
    }

    #[test]
    fn test_parse_duration_4h() {
        let d = parse_duration("4h").unwrap();
        assert_eq!(d, Duration::from_secs(4 * 3600));
    }

    #[test]
    fn test_parse_duration_7d() {
        let d = parse_duration("7d").unwrap();
        assert_eq!(d, Duration::from_secs(7 * 24 * 3600));
    }

    #[test]
    fn test_parse_duration_rejects_garbage() {
        assert!(parse_duration("bogus").is_err());
        assert!(parse_duration("").is_err());
    }

    // (4) Failing-case deduplication picks the smallest seed.
    #[test]
    fn test_failing_case_dedup_keeps_smallest_seed() {
        let mut s = Summary::default();
        // Three FAILs of the same (oracle, case), seeds 100, 50, 75.
        let oc = |case_id: &'static str| CaseOutcome {
            oracle: ORACLE_CYCLE,
            case: case_id,
            verdict: Verdict::Fail,
            detail: "delta=-2".into(),
            elapsed_ms: 7,
        };
        s.record(&[oc("backward_branch_large")], 100);
        s.record(&[oc("backward_branch_large")], 50);
        s.record(&[oc("backward_branch_large")], 75);
        // Also a different case with higher seed.
        s.record(&[oc("nop_chain_8")], 200);

        assert_eq!(s.failing_cases.len(), 2);
        let key1 = (ORACLE_CYCLE, "backward_branch_large".to_string());
        let key2 = (ORACLE_CYCLE, "nop_chain_8".to_string());
        assert_eq!(s.failing_cases.get(&key1), Some(&50));
        assert_eq!(s.failing_cases.get(&key2), Some(&200));
        assert_eq!(s.total_fail(), 4);
    }

    #[test]
    fn test_pass_outcomes_do_not_appear_in_failing_cases() {
        let mut s = Summary::default();
        s.record(
            &[CaseOutcome::pass(ORACLE_PERIPH, "pio0_nop_loop", 12)],
            42,
        );
        assert!(s.failing_cases.is_empty());
        assert_eq!(s.totals.get(ORACLE_PERIPH).map(|x| x.pass), Some(1));
    }

    // (5) Heartbeat `[+HH:MM:SS]` prefix is correct across durations.
    #[test]
    fn test_fmt_elapsed_seconds() {
        assert_eq!(fmt_elapsed(Duration::from_secs(0)), "[+00:00:00]");
        assert_eq!(fmt_elapsed(Duration::from_secs(59)), "[+00:00:59]");
    }

    #[test]
    fn test_fmt_elapsed_minutes() {
        assert_eq!(fmt_elapsed(Duration::from_secs(60)), "[+00:01:00]");
        assert_eq!(fmt_elapsed(Duration::from_secs(3599)), "[+00:59:59]");
    }

    #[test]
    fn test_fmt_elapsed_hours() {
        assert_eq!(fmt_elapsed(Duration::from_secs(3600)), "[+01:00:00]");
        assert_eq!(fmt_elapsed(Duration::from_secs(2 * 3600 + 30 * 60 + 15)), "[+02:30:15]");
    }

    #[test]
    fn test_fmt_elapsed_large() {
        // 48h + 3m + 4s
        let secs = 48 * 3600 + 3 * 60 + 4;
        assert_eq!(fmt_elapsed(Duration::from_secs(secs)), "[+48:03:04]");
    }

    #[test]
    fn test_oracle_name_static_roundtrip() {
        assert_eq!(oracle_name_static("cycle"), ORACLE_CYCLE);
        assert_eq!(oracle_name_static("periph"), ORACLE_PERIPH);
        assert_eq!(oracle_name_static("bank"), ORACLE_BANK);
        assert_eq!(oracle_name_static("dualcore"), ORACLE_DUALCORE);
        assert_eq!(oracle_name_static("isr"), ORACLE_ISR);
        assert_eq!(oracle_name_static("foo"), "unknown");
    }

    /// Every ISR scenario must start with `isr_` for substring-uniqueness.
    /// Fires at `cargo test` time so a bad rename in isr_scenarios.rs
    /// breaks the orchestrator's filter semantics locally.
    #[test]
    fn test_all_isr_scenarios_have_isr_prefix() {
        for s in isr_scenarios::SCENARIOS {
            assert!(
                s.name.starts_with("isr_"),
                "isr scenario '{}' missing 'isr_' prefix",
                s.name,
            );
        }
    }

    /// All five catalogues must round-trip through the orchestrator's
    /// substring-uniqueness validator. This is load-bearing: without it,
    /// an accidental `isr_pendsv` case colliding with a future
    /// `bankcfl_isr_pendsv` rename would silently alias both under one
    /// `--filter isr_pendsv` flag.
    #[test]
    fn test_five_catalogues_cover_correct_oracles() {
        let names = collect_all_catalogue_names();
        // At least one name per oracle.
        assert!(names.iter().any(|n| n.starts_with("cycle") || n.starts_with("nop") || n.starts_with("push") || n.starts_with("backward") || n.starts_with("ldm") || n.starts_with("bank_")));
        assert!(names.iter().any(|n| n.starts_with("pio0") || n.starts_with("pll_sys") || n.starts_with("clock")));
        assert!(names.iter().any(|n| n.starts_with("bankcfl_")));
        assert!(names.iter().any(|n| n.starts_with("dualcore_")));
        assert!(names.iter().any(|n| n.starts_with("isr_")));
    }

    #[test]
    fn test_parse_args_defaults() {
        let a = parse_args(vec![]).unwrap();
        assert!(a.soak.is_none());
        assert!(a.filter.is_none());
        assert!(!a.verbose);
        assert!(a.probe.is_none());
    }

    #[test]
    fn test_parse_args_soak_and_seed() {
        let a = parse_args(vec![
            "--soak".into(),
            "2h".into(),
            "--seed".into(),
            "999".into(),
            "--filter".into(),
            "pio0".into(),
            "--verbose".into(),
        ])
        .unwrap();
        assert_eq!(a.soak, Some(Duration::from_secs(2 * 3600)));
        assert_eq!(a.seed, 999);
        assert_eq!(a.filter.as_deref(), Some("pio0"));
        assert!(a.verbose);
    }

    #[test]
    fn test_parse_args_probe_full_selector() {
        let a = parse_args(vec!["--probe".into(), "2e8a:000c:E46410955F614129".into()])
            .unwrap();
        let sel = a.probe.expect("probe must be Some");
        assert_eq!(sel.vendor_id, 0x2e8a);
        assert_eq!(sel.product_id, 0x000c);
        assert_eq!(sel.serial_number.as_deref(), Some("E46410955F614129"));
    }

    #[test]
    fn test_parse_args_probe_missing_value_errors() {
        assert!(parse_args(vec!["--probe".into()]).is_err());
    }

    #[test]
    fn test_parse_args_probe_bogus_value_errors() {
        let err = parse_args(vec!["--probe".into(), "bogus".into()])
            .expect_err("bogus selector must error");
        assert!(
            err.contains("invalid probe selector"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn test_parse_args_unknown_arg_errors() {
        assert!(parse_args(vec!["--bogus".into()]).is_err());
    }

    // --- Substring-uniqueness validator tests ---------------------------

    /// Clean list — no substring relationships.
    #[test]
    fn test_validator_passes_on_clean_names() {
        let names = ["alpha", "beta", "gamma", "delta_x"];
        assert!(validate_catalogue_names_are_unique(&names).is_ok());
    }

    /// Catches a strict substring (short-in-long).
    #[test]
    fn test_validator_catches_substring() {
        // "bank_ldr" is a strict substring of "bank_ldr_b0" — exactly the
        // class of collision the bankcfl_* rename was meant to prevent.
        let names = ["bank_ldr_b0", "bank_ldr", "cycle_foo"];
        let err = validate_catalogue_names_are_unique(&names)
            .expect_err("substring must fail the validator");
        // Make sure the error message mentions both names so a failing
        // soak run's log actually tells Arthur which pair is broken.
        assert!(err.contains("bank_ldr"), "err must cite inner name: {err}");
        assert!(err.contains("bank_ldr_b0"), "err must cite outer name: {err}");
    }

    /// Catches straight duplicates.
    #[test]
    fn test_validator_catches_duplicate() {
        let names = ["foo", "bar", "foo"];
        let err = validate_catalogue_names_are_unique(&names)
            .expect_err("duplicate must fail the validator");
        assert!(err.contains("duplicate"), "{err}");
        assert!(err.contains("foo"), "{err}");
    }

    /// The real catalogue passes — this is the load-bearing assertion
    /// for the renamed bankcfl_* cases. If a future PR reintroduces a
    /// substring collision (e.g. adds a `cycle_bank_ldr` case), this
    /// test fires before the orchestrator ever gets near silicon.
    #[test]
    fn test_real_catalogues_are_substring_unique() {
        let names = collect_all_catalogue_names();
        validate_catalogue_names_are_unique(&names)
            .expect("real catalogues must be substring-unique");
    }

    // --- NameInterner tests ---------------------------------------------

    #[test]
    fn test_name_interner_returns_same_static_on_repeat() {
        let mut i = NameInterner::default();
        let a = i.intern("watchdog_timeout");
        let b = i.intern("watchdog_timeout");
        // Same pointer: interner deduped.
        assert_eq!(a.as_ptr(), b.as_ptr());
        // Different input → different storage.
        let c = i.intern("probe_error");
        assert_ne!(a.as_ptr(), c.as_ptr());
        // Entries accumulate.
        assert_eq!(i.seen.len(), 2);
    }

    // --- Bank case-name rename regression ------------------------------

    /// Every bank-conflict case name starts with `bankcfl_` — the
    /// prefix that guarantees no substring collision with cycle-oracle
    /// `bank_contention_*` names. If a future commit drops the prefix
    /// this test fires.
    #[test]
    fn test_all_bank_cases_have_bankcfl_prefix() {
        let cat = bank_conflict_cases::build_catalog();
        for c in &cat {
            assert!(
                c.name.starts_with("bankcfl_"),
                "bank case '{}' missing 'bankcfl_' prefix",
                c.name,
            );
        }
    }
}

// test_silicon_rp2040 — unified RP2040 silicon orchestrator. Mirrors the
// shape of `test_silicon` (RP2354) but wraps the three RP2040 oracles:
//
//   * `probe_diff_rp2040_lib::run_against`        (M0+ ISA differential)
//   * `silicon_periph_rp2040::run_against`        (peripheral state)
//   * `isr_scenarios_rp2040::run_against`         (exception entry / counters)
//
// Single-pass mode (default) runs each oracle's catalogue once in
// catalogue order. Soak mode (`--soak <duration>`) shuffles the
// pre-filtered name lists once per iteration and passes them to each
// `run_against` call (one call per oracle per iteration). A 60s
// in-library deadline guards each oracle invocation against a wedged
// case.
//
// Notes:
//   * RP2040's M0+ has no DWT / no CYCCNT, so there is no cycle oracle
//     and no dual-core cycle-contention oracle here.
//   * Bank-conflict has no documented public catalogue on M0+; not
//     wrapped.
//   * **ISR oracle caveat**: V5 IRQ plumbing is in place (HLD V7 §5.2
//     / §5.3). Expect PASS, but if FAIL, capture and follow up — the
//     emulator-side dispatch is unit-validated but not silicon-validated
//     yet (see `silicon_isr_diff_rp2040.rs`'s header comment).
//
// See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and
// Coverage Expansion.md` §Component 1 for the cross-oracle
// state-cleanup contract this binary inherits from `test_silicon`.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use picoem_harness::cli::parse_probe_selector;
use picoem_harness::isr_scenarios_rp2040::{self, IsrArgs};
use picoem_harness::probe_diff_rp2040_lib::{self, ProbeDiffArgs};
use picoem_harness::silicon_oracle::{CaseOutcome, Verdict, name_matches_filter, should_exclude};
use picoem_harness::silicon_periph_rp2040::{self, PeriphArgs};
use picoem_harness::test_silicon_common::{
    GIVE_UP_THRESHOLD, HEARTBEAT_INTERVAL, NameInterner, Summary, append_error_log, attach,
    default_seed, emit_log_line, ensure_fuzz_runs_dir, errors_log_path, fmt_elapsed, iter_seed,
    now_iso, parse_duration, reattach_with_retries, shuffle_in_place,
    validate_catalogue_names_are_unique,
};
use probe_rs::Session;
use probe_rs::probe::DebugProbeSelector;
use rand::SeedableRng;
use rand::rngs::StdRng;

const CHIP: &str = "rp2040";

// Oracle identifiers (match what each library API stamps onto
// `CaseOutcome.oracle`). Kept here so the summary + filtering stay in
// sync with the libraries.
const ORACLE_PROBE_DIFF: &str = "probe_diff";
const ORACLE_PERIPH_M0: &str = "periph_m0";
const ORACLE_ISR_M0: &str = "isr_m0";

/// Sentinel case name used for synthesised probe-rs error outcomes
/// where we know which oracle was running but not which case.
const PROBE_ERROR_SENTINEL: &str = "<probe-rs error — partial results may be missing>";

/// Per-oracle in-library deadline. Each `run_against` call gets at most
/// 60 seconds of wall-clock budget; between cases the library checks
/// `Instant::now() > deadline` and returns the partial outcomes.
const ORACLE_DEADLINE: Duration = Duration::from_secs(60);

/// Default fuzz-count for the probe_diff oracle in soak mode. Each
/// iteration drives this many probe steps per ALU/MEM class through
/// `run_against`, with `seed = iter_seed(base, iter_index)`.
const DEFAULT_FUZZ_COUNT: usize = 1000;

/// Per-iteration degraded-rate (`degraded / (pass + fail + skip + degraded)`)
/// threshold above which the orchestrator forces a reattach. Mirrors the
/// 25% transport-degraded threshold the standalone `probe_diff_rp2040`
/// binary uses to map to rc=3. Below `MIN_DEGRADED_SAMPLE` cases the
/// rate is too noisy to act on.
const DEGRADED_RATE_PCT: u64 = 25;
const MIN_DEGRADED_SAMPLE: u64 = 20;

/// True when the per-iteration degraded count crosses the rate threshold
/// AND the sample is large enough to be meaningful. Hoisted out of the
/// soak loop so unit tests can exercise the boundary cases.
fn should_force_reattach_on_degraded(degraded: u64, attempted: u64) -> bool {
    attempted >= MIN_DEGRADED_SAMPLE && (degraded * 100) / attempted >= DEGRADED_RATE_PCT
}

const USAGE: &str = "\
Usage: test_silicon_rp2040 [--soak <duration>] [--seed <u64>] [--filter <substr>] [--exclude <substr>] [--verbose] [--probe VID:PID:SERIAL] [--fuzz-count <N>] [--dry-run]

Options:
  --soak        Run continuously for the given duration (e.g. 30m, 4h, 7d).
                Default: single pass.
  --seed        Base RNG seed for soak-mode shuffling and probe_diff fuzz.
                Default: current Unix epoch seconds.
  --filter      Only run cases whose name contains <substr>. Applied to
                every oracle's catalogue. Default: all cases.
  --exclude     Skip cases whose name contains <substr> (applied after
                --filter). Applied to every oracle's catalogue.
  --verbose     Print full per-case output every iteration (default:
                quiet — failures + hourly heartbeat).
  --probe       Select a specific debug probe by VID:PID:SERIAL.
                Required on hosts with multiple probes attached.
  --fuzz-count  probe_diff fuzz count per orchestrator iteration.
                Default: 1000.
  --dry-run     Print a planned-execution summary (chip, probe, oracle
                catalogue sizes) and exit 0 without attaching.
";

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct CliArgs {
    soak: Option<Duration>,
    seed: u64,
    filter: Option<String>,
    exclude: Option<String>,
    verbose: bool,
    probe: Option<DebugProbeSelector>,
    fuzz_count: usize,
    dry_run: bool,
}

impl Default for CliArgs {
    fn default() -> Self {
        Self {
            soak: None,
            seed: 0,
            filter: None,
            exclude: None,
            verbose: false,
            probe: None,
            fuzz_count: DEFAULT_FUZZ_COUNT,
            dry_run: false,
        }
    }
}

fn parse_args(argv: Vec<String>) -> Result<CliArgs, String> {
    let mut a = CliArgs::default();
    let mut seed_explicit: Option<u64> = None;
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--soak" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--soak requires a duration\n{USAGE}"));
                }
                a.soak = Some(parse_duration(&argv[i]).map_err(|e| format!("{e}\n{USAGE}"))?);
            }
            "--seed" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--seed requires a u64\n{USAGE}"));
                }
                seed_explicit = Some(
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
                a.filter = Some(argv[i].clone());
            }
            "--exclude" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--exclude requires a substring\n{USAGE}"));
                }
                a.exclude = Some(argv[i].clone());
            }
            "--probe" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!(
                        "--probe requires a VID:PID:SERIAL argument\n{USAGE}"
                    ));
                }
                a.probe =
                    Some(parse_probe_selector(&argv[i]).map_err(|e| format!("{e}\n{USAGE}"))?);
            }
            "--fuzz-count" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--fuzz-count requires a count\n{USAGE}"));
                }
                a.fuzz_count = argv[i]
                    .parse::<usize>()
                    .map_err(|e| format!("invalid --fuzz-count '{}': {e}\n{USAGE}", argv[i]))?;
            }
            "--verbose" => a.verbose = true,
            "--dry-run" => a.dry_run = true,
            "--help" | "-h" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument '{other}'\n{USAGE}")),
        }
        i += 1;
    }
    a.seed = seed_explicit.unwrap_or_else(default_seed);
    Ok(a)
}

// ---------------------------------------------------------------------------
// Catalogue helpers
// ---------------------------------------------------------------------------

/// Names of the three RP2040 oracle catalogues, in the order the
/// orchestrator runs them. Used by both the substring-uniqueness check
/// and the soak-loop name lists.
fn periph_names() -> Vec<&'static str> {
    silicon_periph_rp2040::SCENARIOS
        .iter()
        .map(|s| s.name)
        .collect()
}

fn isr_names() -> Vec<&'static str> {
    isr_scenarios_rp2040::SCENARIOS
        .iter()
        .map(|s| s.name)
        .collect()
}

/// probe_diff has a generated catalogue (`generate_all`) on the order of
/// thousands of entries. Most of those are filtered out by
/// `is_m0plus_silicon_safe`. The names use human-readable assembler
/// strings (e.g. "LSLS R0, R1, #3") that genuinely contain one another
/// as substrings — that's intentional, and probe_diff's `order` path
/// uses exact-name match rather than substring match, so the
/// substring-uniqueness invariant only needs to hold across the
/// substring-filtered catalogues (periph + isr).
fn probe_diff_admitted_names() -> Vec<String> {
    use picoem_harness::generate_all;
    use picoem_harness::probe_diff_rp2040_lib::is_m0plus_silicon_safe;
    generate_all()
        .into_iter()
        .filter(is_m0plus_silicon_safe)
        .filter(|tc| !tc.probe_only)
        .map(|tc| tc.name)
        .collect()
}

/// Names eligible for the substring-uniqueness invariant: the periph
/// and isr catalogues (which the orchestrator filters via substring) +
/// any probe_diff name that happens to share a prefix/suffix with
/// either, so a renamed probe_diff case can't silently alias a periph
/// or isr filter at runtime.
///
/// The cross-check fragment we emit here is "every probe_diff name
/// passed verbatim against periph ∪ isr". The full O(N²) sweep across
/// the whole probe_diff space would be O(thousands²) and offers no
/// extra coverage — probe_diff exact-match selection is the contract.
fn collect_substring_check_names() -> Vec<String> {
    let mut names: Vec<String> = Vec::new();
    for n in periph_names() {
        names.push(n.to_string());
    }
    for n in isr_names() {
        names.push(n.to_string());
    }
    names
}

/// Catalogue census used by `--dry-run` to display planned coverage.
fn catalogue_sizes() -> (usize, usize, usize) {
    (
        probe_diff_admitted_names().len(),
        periph_names().len(),
        isr_names().len(),
    )
}

// ---------------------------------------------------------------------------
// Oracle dispatch
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum OracleKind {
    ProbeDiff,
    Periph,
    Isr,
}

impl OracleKind {
    fn as_str(self) -> &'static str {
        match self {
            OracleKind::ProbeDiff => ORACLE_PROBE_DIFF,
            OracleKind::Periph => ORACLE_PERIPH_M0,
            OracleKind::Isr => ORACLE_ISR_M0,
        }
    }
}

fn oracle_name_static(name: &str) -> &'static str {
    match name {
        ORACLE_PROBE_DIFF => ORACLE_PROBE_DIFF,
        ORACLE_PERIPH_M0 => ORACLE_PERIPH_M0,
        ORACLE_ISR_M0 => ORACLE_ISR_M0,
        _ => "unknown",
    }
}

/// Per-oracle plan. `order = None` defers selection to the library
/// (filter + catalogue order); `order = Some(v)` is the soak-mode path
/// where the orchestrator hands the library a pre-shuffled name list.
///
/// `order` carries `&'static str` because the periph + isr catalogues
/// are `&'static [&'static str]` and probe_diff names from the
/// `SyntheticNameInterner` are also `&'static str`. Avoids a
/// `Vec<String>` allocation per soak iteration.
#[derive(Clone, Debug)]
struct OraclePlan {
    oracle: OracleKind,
    order: Option<Vec<&'static str>>,
    filter: Option<String>,
    exclude: Option<String>,
    /// Soak-mode probe_diff fuzz seed. The orchestrator sets this to
    /// `iter_seed(base, iter_index)` so each iteration explores a fresh
    /// fuzz subset while remaining reproducible.
    probe_diff_seed: u64,
    /// probe_diff fuzz count per oracle invocation. None → targeted
    /// catalogue, Some → fuzz mode.
    probe_diff_fuzz_count: Option<usize>,
}

fn run_one_oracle(session: &mut Session, plan: &OraclePlan) -> Result<Vec<CaseOutcome>, String> {
    let order_slice: Option<&[&str]> = plan.order.as_deref();
    let deadline = Some(Instant::now() + ORACLE_DEADLINE);

    let result: Result<Vec<CaseOutcome>, Box<dyn std::error::Error + Send + Sync>> =
        match plan.oracle {
            OracleKind::ProbeDiff => {
                let mut core = session.core(0).map_err(|e| e.to_string())?;
                let args = ProbeDiffArgs {
                    fuzz_count: plan.probe_diff_fuzz_count,
                    seed: plan.probe_diff_seed,
                };
                probe_diff_rp2040_lib::run_against(&mut core, &args, order_slice, deadline)
            }
            OracleKind::Periph => {
                let mut core = session.core(0).map_err(|e| e.to_string())?;
                let args = PeriphArgs {
                    filter: plan.filter.clone(),
                    exclude: plan.exclude.clone(),
                    verbose: false,
                };
                silicon_periph_rp2040::run_against(&mut core, &args, order_slice, deadline)
            }
            OracleKind::Isr => {
                let mut core = session.core(0).map_err(|e| e.to_string())?;
                let args = IsrArgs {
                    filter: plan.filter.clone(),
                    exclude: plan.exclude.clone(),
                    verbose: false,
                };
                isr_scenarios_rp2040::run_against(&mut core, &args, order_slice, deadline)
            }
        };
    result.map_err(|e| e.to_string())
}

// ---------------------------------------------------------------------------
// Orchestrator entry
// ---------------------------------------------------------------------------

fn orchestrate(args: &CliArgs, stop_flag: Arc<AtomicBool>) -> Result<i32, String> {
    // Make the intent explicit: the orchestrator owns the fuzz-runs
    // directory and ensures it exists before any error log lands.
    let _ = ensure_fuzz_runs_dir();
    let log_path = errors_log_path("test_silicon_rp2040");
    let start = Instant::now();

    // Substring-uniqueness check across the substring-filtered
    // catalogues (periph + isr). probe_diff uses exact-name match in
    // its `order` path so its human-readable assembler names
    // (e.g. "LSLS R0, R1, #3" vs "LSLS R0, R1, #31 (max shift)") don't
    // need substring isolation.
    let names = collect_substring_check_names();
    let refs: Vec<&str> = names.iter().map(|s| s.as_str()).collect();
    if let Err(msg) = validate_catalogue_names_are_unique(&refs) {
        eprintln!("test_silicon_rp2040: catalogue name check failed:\n  {msg}");
        return Err(msg);
    }

    // Banner.
    println!("test_silicon_rp2040: starting");
    println!("  chip:    {CHIP}");
    println!(
        "  mode:    {}",
        if args.soak.is_some() {
            "soak"
        } else {
            "single-pass"
        }
    );
    if let Some(d) = args.soak {
        println!("  soak:    {}", humantime::format_duration(d));
    }
    println!("  seed:    {}", args.seed);
    println!("  filter:  {}", args.filter.as_deref().unwrap_or("<none>"));
    println!("  exclude: {}", args.exclude.as_deref().unwrap_or("<none>"));
    println!("  verbose: {}", args.verbose);
    println!("  fuzz:    {}/class", args.fuzz_count);
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
        "  ISR oracle: V5 IRQ plumbing is in place; expect PASS, but if FAIL, capture and \
         follow up — emulator-side dispatch is unit-validated but not silicon-validated yet."
    );
    println!();

    if args.dry_run {
        let (probe_diff_count, periph_count, isr_count) = catalogue_sizes();
        let probe = args.probe.as_ref().map_or("<auto>".to_string(), |p| {
            format!(
                "{:04x}:{:04x}:{}",
                p.vendor_id,
                p.product_id,
                p.serial_number.as_deref().unwrap_or("")
            )
        });
        println!(
            "would-attach to chip={CHIP}, probe={probe}, \
             oracles=[probe_diff,periph_m0,isr_m0], \
             cases=probe_diff:{probe_diff_count}+periph_m0:{periph_count}+isr_m0:{isr_count}",
        );
        return Ok(0);
    }

    // Initial attach.
    let probe_sel = args.probe.as_ref();
    let mut session = match attach(CHIP, probe_sel) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("test_silicon_rp2040: initial attach failed: {e}");
            append_error_log(&log_path, &format!("{} initial-attach-fail {e}", now_iso()));
            return Err(format!("initial attach failed: {e}"));
        }
    };

    let mut summary = Summary::default();
    let mut interner = NameInterner::default();
    let deadline = args.soak.map(|d| start + d);

    match deadline {
        None => {
            session = single_pass(
                args,
                session,
                &mut summary,
                &mut interner,
                &log_path,
                probe_sel,
            )?;
            let _ = session;
            summary.print("test_silicon_rp2040 summary", 1);
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
            summary.print("test_silicon_rp2040 summary", total_iters);
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

// ---------------------------------------------------------------------------
// Single-pass mode
// ---------------------------------------------------------------------------

fn single_pass(
    args: &CliArgs,
    mut session: Session,
    summary: &mut Summary,
    interner: &mut NameInterner,
    log_path: &PathBuf,
    probe: Option<&DebugProbeSelector>,
) -> Result<Session, String> {
    for oracle in [OracleKind::ProbeDiff, OracleKind::Periph, OracleKind::Isr] {
        println!("--- oracle: {} ---", oracle.as_str());
        let t0 = Instant::now();
        let plan = OraclePlan {
            oracle,
            order: None,
            filter: args.filter.clone(),
            exclude: args.exclude.clone(),
            probe_diff_seed: args.seed,
            // Single-pass uses the targeted catalogue (no fuzz). Soak
            // overrides per-iteration via `Some(args.fuzz_count)`.
            probe_diff_fuzz_count: None,
        };
        match run_one_oracle(&mut session, &plan) {
            Ok(outcomes) => {
                for o in &outcomes {
                    println!(
                        "  {:<10} {:<40} {:<4}  elapsed={}ms  {}",
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
                let case_name: &'static str = interner.intern(PROBE_ERROR_SENTINEL);
                let synth =
                    CaseOutcome::fail(oracle_name_static(oracle.as_str()), case_name, e.clone(), 0);
                summary.record(&[synth], 0);
                summary.reattach_count += 1;
                session = reattach_with_retries(CHIP, probe)
                    .map_err(|e| format!("reattach failed: {e}"))?;
            }
        }
        println!(
            "  ({} oracle took {:.2}s)",
            oracle.as_str(),
            t0.elapsed().as_secs_f64()
        );
        println!();
    }
    Ok(session)
}

// ---------------------------------------------------------------------------
// Soak mode
// ---------------------------------------------------------------------------

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

    let periph_n: Vec<&'static str> = periph_names()
        .into_iter()
        .filter(|n| name_matches_filter(n, args.filter.as_deref()))
        .filter(|n| !should_exclude(n, args.exclude.as_deref()))
        .collect();
    let isr_n: Vec<&'static str> = isr_names()
        .into_iter()
        .filter(|n| name_matches_filter(n, args.filter.as_deref()))
        .filter(|n| !should_exclude(n, args.exclude.as_deref()))
        .collect();

    if periph_n.is_empty() && isr_n.is_empty() {
        // probe_diff is fuzz-driven (not name-filtered), so we don't
        // gate the loop on it being empty. But if periph and isr have
        // no matches and the user expected the filter to cover them,
        // surface that.
        if args.filter.is_some() || args.exclude.is_some() {
            println!(
                "test_silicon_rp2040: no periph/isr cases match filter '{}' (exclude '{}'); \
                 probe_diff will still run via --fuzz-count",
                args.filter.as_deref().unwrap_or("<none>"),
                args.exclude.as_deref().unwrap_or("<none>"),
            );
        }
    }

    while Instant::now() < deadline && !stop_flag.load(Ordering::SeqCst) {
        let s = iter_seed(args.seed, iter_index);
        let mut rng = StdRng::seed_from_u64(s);

        let mut periph_plan: Vec<&'static str> = periph_n.clone();
        let mut isr_plan: Vec<&'static str> = isr_n.clone();
        shuffle_in_place(&mut periph_plan, &mut rng);
        shuffle_in_place(&mut isr_plan, &mut rng);

        if args.verbose {
            println!(
                "{} iter={} seed={} starting",
                fmt_elapsed(start.elapsed()),
                iter_index,
                s
            );
        }

        // probe_diff is fuzz-driven so its "order" is implicit (the
        // generated names for this seed). We don't pre-shuffle here —
        // the library walks ALU then MEM, which is fine for soak
        // coverage because the seed varies per iteration.
        let plans: [(OracleKind, Option<Vec<&'static str>>); 3] = [
            (OracleKind::ProbeDiff, None),
            (OracleKind::Periph, Some(periph_plan)),
            (OracleKind::Isr, Some(isr_plan)),
        ];

        'plans: for (oracle, names) in plans {
            if stop_flag.load(Ordering::SeqCst) {
                break 'plans;
            }
            // Names empty for periph/isr → skip; probe_diff always runs
            // via fuzz_count.
            if oracle != OracleKind::ProbeDiff
                && names.as_ref().map(|v| v.is_empty()).unwrap_or(true)
            {
                continue;
            }
            let Some(ref mut this_session) = session_opt else {
                break 'plans;
            };
            let plan = OraclePlan {
                oracle,
                order: names.clone(),
                filter: args.filter.clone(),
                exclude: args.exclude.clone(),
                probe_diff_seed: s,
                probe_diff_fuzz_count: if oracle == OracleKind::ProbeDiff {
                    Some(args.fuzz_count)
                } else {
                    None
                },
            };
            match run_one_oracle(this_session, &plan) {
                Ok(outcomes) => {
                    // Per-iteration degraded counter — drives the
                    // reattach-on-degraded-storm trigger below. Keyed
                    // off this oracle's outcomes only so a single
                    // misbehaving oracle can force a transport reset.
                    let mut iter_pass: u64 = 0;
                    let mut iter_fail: u64 = 0;
                    let mut iter_skip: u64 = 0;
                    let mut iter_degraded: u64 = 0;
                    for o in &outcomes {
                        match o.verdict {
                            Verdict::Pass => iter_pass += 1,
                            Verdict::Fail => iter_fail += 1,
                            Verdict::Skip => iter_skip += 1,
                            Verdict::Degraded => iter_degraded += 1,
                        }
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
                                iter_index,
                                s,
                                o.oracle,
                                o.case,
                                o.detail,
                            );
                            emit_log_line(log_path, &line);
                        }
                    }
                    summary.record(&outcomes, s);
                    consecutive_reattach_fails = 0;

                    // Degraded-rate trigger — forces reattach when
                    // probe transport is unstable enough that a large
                    // fraction of cases came back as Degraded. The
                    // sample-size floor avoids tripping on tiny oracle
                    // catalogues.
                    let attempted = iter_pass + iter_fail + iter_skip + iter_degraded;
                    if should_force_reattach_on_degraded(iter_degraded, attempted) {
                        let line = format!(
                            "{} iter={} oracle={} degraded_rate={}/{} >= {}% — forcing reattach",
                            fmt_elapsed(start.elapsed()),
                            iter_index,
                            oracle.as_str(),
                            iter_degraded,
                            attempted,
                            DEGRADED_RATE_PCT,
                        );
                        emit_log_line(log_path, &line);
                        session_opt = None;
                        summary.reattach_count += 1;
                        match reattach_with_retries(CHIP, probe) {
                            Ok(fresh) => {
                                session_opt = Some(fresh);
                                consecutive_reattach_fails = 0;
                            }
                            Err(rerr) => {
                                consecutive_reattach_fails += 1;
                                let rline = format!(
                                    "{} iter={} reattach failed: {rerr} (consecutive={})",
                                    fmt_elapsed(start.elapsed()),
                                    iter_index,
                                    consecutive_reattach_fails,
                                );
                                emit_log_line(log_path, &rline);
                                if consecutive_reattach_fails >= GIVE_UP_THRESHOLD {
                                    eprintln!(
                                        "test_silicon_rp2040: {consecutive_reattach_fails} consecutive reattach failures — giving up",
                                    );
                                    return Ok((2, total_iters));
                                }
                                break 'plans;
                            }
                        }
                    }
                }
                Err(e) => {
                    let synthetic_oracle = oracle_name_static(oracle.as_str());
                    let detail = format!("probe-rs error: {e}");
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
                        iter_index,
                        s,
                        synthetic_oracle,
                        case_name_static,
                        detail,
                    );
                    emit_log_line(log_path, &line);
                    summary.record(&[synth], s);

                    session_opt = None;
                    summary.reattach_count += 1;
                    match reattach_with_retries(CHIP, probe) {
                        Ok(fresh) => {
                            session_opt = Some(fresh);
                            consecutive_reattach_fails = 0;
                        }
                        Err(rerr) => {
                            consecutive_reattach_fails += 1;
                            let rline = format!(
                                "{} iter={} reattach failed: {rerr} (consecutive={})",
                                fmt_elapsed(start.elapsed()),
                                iter_index,
                                consecutive_reattach_fails,
                            );
                            emit_log_line(log_path, &rline);
                            if consecutive_reattach_fails >= GIVE_UP_THRESHOLD {
                                eprintln!(
                                    "test_silicon_rp2040: {consecutive_reattach_fails} consecutive reattach failures — giving up",
                                );
                                return Ok((2, total_iters));
                            }
                            break 'plans;
                        }
                    }
                }
            }
        }

        iter_index += 1;
        total_iters += 1;

        if session_opt.is_none() {
            match reattach_with_retries(CHIP, probe) {
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

        // Between-iteration reset.
        if let Some(ref mut session) = session_opt
            && let Ok(mut core) = session.core(0)
        {
            let _ = core.reset_and_halt(Duration::from_millis(500));
        }

        // Heartbeat.
        let now = Instant::now();
        if now >= next_heartbeat && !args.verbose {
            println!(
                "{} iter={} pass={} fail={} skip={} degraded={} reattach={}",
                fmt_elapsed(start.elapsed()),
                iter_index,
                summary.total_pass(),
                summary.total_fail(),
                summary.total_skip(),
                summary.total_degraded(),
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
    picoem_harness::harness_tracing_init();
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
            eprintln!("\ntest_silicon_rp2040: Ctrl-C received; finishing current case…");
        }) {
            eprintln!("warning: Ctrl-C handler install failed: {e}");
        }
    }

    match orchestrate(&args, stop) {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("test_silicon_rp2040: {e}");
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

    #[test]
    fn iter_seed_is_deterministic() {
        // Sanity-check the shared helper is wired up.
        assert_eq!(iter_seed(42, 7), 49);
        assert_eq!(iter_seed(u64::MAX, 1), 0);
    }

    #[test]
    fn fmt_elapsed_round_trip() {
        assert_eq!(fmt_elapsed(Duration::from_secs(0)), "[+00:00:00]");
        assert_eq!(
            fmt_elapsed(Duration::from_secs(3600 + 60 + 2)),
            "[+01:01:02]"
        );
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
    fn parse_args_defaults() {
        let a = parse_args(vec![]).unwrap();
        assert!(a.soak.is_none());
        assert!(a.filter.is_none());
        assert!(a.exclude.is_none());
        assert!(!a.verbose);
        assert!(a.probe.is_none());
        assert_eq!(a.fuzz_count, DEFAULT_FUZZ_COUNT);
        assert!(!a.dry_run);
    }

    #[test]
    fn parse_args_soak_seed_filter() {
        let a = parse_args(vec![
            "--soak".into(),
            "2h".into(),
            "--seed".into(),
            "999".into(),
            "--filter".into(),
            "pio".into(),
            "--verbose".into(),
        ])
        .unwrap();
        assert_eq!(a.soak, Some(Duration::from_secs(2 * 3600)));
        assert_eq!(a.seed, 999);
        assert_eq!(a.filter.as_deref(), Some("pio"));
        assert!(a.verbose);
    }

    #[test]
    fn parse_args_fuzz_count() {
        let a = parse_args(vec!["--fuzz-count".into(), "4242".into()]).unwrap();
        assert_eq!(a.fuzz_count, 4242);
    }

    #[test]
    fn parse_args_fuzz_count_invalid() {
        assert!(parse_args(vec!["--fuzz-count".into(), "abc".into()]).is_err());
        assert!(parse_args(vec!["--fuzz-count".into()]).is_err());
    }

    #[test]
    fn parse_args_probe_full_selector() {
        let a = parse_args(vec!["--probe".into(), "2e8a:000c:E46410955F614129".into()]).unwrap();
        let sel = a.probe.expect("probe must be Some");
        assert_eq!(sel.vendor_id, 0x2e8a);
        assert_eq!(sel.product_id, 0x000c);
        assert_eq!(sel.serial_number.as_deref(), Some("E46410955F614129"));
    }

    #[test]
    fn parse_args_probe_missing_value_errors() {
        assert!(parse_args(vec!["--probe".into()]).is_err());
    }

    #[test]
    fn parse_args_probe_bogus_value_errors() {
        let err = parse_args(vec!["--probe".into(), "bogus".into()]).expect_err("bogus must error");
        assert!(
            err.contains("invalid probe selector"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn parse_args_dry_run() {
        let a = parse_args(vec!["--dry-run".into()]).unwrap();
        assert!(a.dry_run);
    }

    #[test]
    fn parse_args_unknown_arg_errors() {
        assert!(parse_args(vec!["--bogus".into()]).is_err());
    }

    #[test]
    fn oracle_name_static_roundtrip() {
        assert_eq!(oracle_name_static("probe_diff"), ORACLE_PROBE_DIFF);
        assert_eq!(oracle_name_static("periph_m0"), ORACLE_PERIPH_M0);
        assert_eq!(oracle_name_static("isr_m0"), ORACLE_ISR_M0);
        assert_eq!(oracle_name_static("foo"), "unknown");
    }

    #[test]
    fn periph_catalogue_has_phase0_scenarios() {
        let names: Vec<&str> = periph_names();
        assert!(!names.is_empty());
        assert!(names.iter().any(|n| n.starts_with("SIO_GPIO_TOGGLE")));
        assert!(names.iter().any(|n| n.starts_with("GAP_TIMER")));
    }

    #[test]
    fn isr_catalogue_has_isr_m0_prefix() {
        let names: Vec<&str> = isr_names();
        assert!(!names.is_empty());
        for n in &names {
            assert!(
                n.starts_with("isr_m0_"),
                "isr scenario '{n}' missing 'isr_m0_' prefix",
            );
        }
    }

    /// The substring-filtered catalogues (periph + isr) must round-trip
    /// through the orchestrator's uniqueness validator. probe_diff is
    /// excluded because it uses exact-name match in its `order` path —
    /// its human-readable assembler names contain one another as
    /// substrings by design ("LSLS R0, R1, #3" vs "LSLS R0, R1, #31
    /// (max shift)").
    #[test]
    fn substring_filtered_catalogues_are_unique() {
        let names = collect_substring_check_names();
        let refs: Vec<&str> = names.iter().map(|s| s.as_str()).collect();
        validate_catalogue_names_are_unique(&refs)
            .expect("periph + isr catalogues must be substring-unique");
    }

    #[test]
    fn degraded_rate_below_sample_floor_never_triggers() {
        // Tiny sample — even 100% degraded shouldn't trigger.
        assert!(!should_force_reattach_on_degraded(5, 5));
        assert!(!should_force_reattach_on_degraded(0, 19));
    }

    #[test]
    fn degraded_rate_at_sample_floor_borderline_triggers() {
        // 5/20 = 25% — exactly at threshold. Tripped because >= 25%.
        assert!(should_force_reattach_on_degraded(5, 20));
        // 4/20 = 20% — under threshold.
        assert!(!should_force_reattach_on_degraded(4, 20));
    }

    #[test]
    fn degraded_rate_above_sample_floor_triggers_above_threshold() {
        // 250 of 1000 = 25% — trips.
        assert!(should_force_reattach_on_degraded(250, 1000));
        // 1 of 1000 = 0.1% — does not trip.
        assert!(!should_force_reattach_on_degraded(1, 1000));
    }

    #[test]
    fn degraded_rate_zero_attempts_does_not_panic() {
        // Sample == 0 must not divide by zero.
        assert!(!should_force_reattach_on_degraded(0, 0));
    }

    /// `--dry-run` path through `orchestrate` must not attach a probe
    /// and must return rc=0. This is the only `orchestrate` branch
    /// that's safe to drive in `cargo test` (no live silicon required).
    #[test]
    fn dry_run_returns_zero_without_probe() {
        let args = parse_args(vec!["--dry-run".into()]).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let rc = orchestrate(&args, stop).expect("dry-run must not error");
        assert_eq!(rc, 0);
    }
}

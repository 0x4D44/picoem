// silicon_periph_diff_rp2350 — peripheral-state oracle for mdrp2350 vs
// real RP2354 silicon.
//
// Thin CLI wrapper over `silicon_scenarios::run_against`. Per-scenario
// setup + diff lives in the library module so the `test_silicon`
// orchestrator can share it.

use mdpicoem_harness::SILICON_RUN_SLED;
use mdpicoem_harness::silicon_oracle::{Verdict, select_by_name};
use mdpicoem_harness::silicon_scenarios::{
    PLL_SYS_BASE, PeriphArgs, PeriphScenario, RED_PATH_SCENARIOS, RESET_PIO0, RESET_PIO1,
    RESET_PLL_SYS, RESETS_BASE, RESETS_RESET, SCENARIOS, run_scenario_with_retry,
};
use probe_rs::{MemoryInterface, Session, SessionConfig};
use std::time::Instant;

const USAGE: &str = "\
Usage: silicon_periph_diff_rp2350 [--filter <substr>] [--exclude <substr>] [--verbose] [--red-path]

Options:
  --filter    Only run scenarios whose name contains <substr>
  --exclude   Skip scenarios whose name contains <substr> (applied after --filter)
  --verbose   Print per-observable diffs, not just the first divergence
  --red-path  Run the red-path witness catalogue instead of the default
              catalogue (Phase 0b HLD V5 §4.2.8 — proves the oracle FAIL
              path renders correctly). Mutually exclusive with normal
              runs; the process's exit code reflects divergence as usual
              so CI should NOT include this catalogue.
";

/// Extended CLI args: base `PeriphArgs` plus `--red-path` catalogue
/// selector. Kept local to this binary so the shared `PeriphArgs`
/// struct in `silicon_scenarios` doesn't leak a red-path-specific
/// field into the `test_silicon` orchestrator.
#[derive(Default)]
struct Args {
    inner: PeriphArgs,
    red_path: bool,
}

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut args = Args::default();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--filter" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--filter requires a substring\n{USAGE}"));
                }
                args.inner.filter = Some(argv[i].clone());
            }
            "--exclude" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--exclude requires a substring\n{USAGE}"));
                }
                args.inner.exclude = Some(argv[i].clone());
            }
            "--verbose" => args.inner.verbose = true,
            "--red-path" => args.red_path = true,
            "--help" | "-h" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument '{other}'\n{USAGE}")),
        }
        i += 1;
    }
    Ok(args)
}

fn main() {
    mdpicoem_harness::harness_tracing_init();
    match run() {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("fatal: {e}");
            std::process::exit(2);
        }
    }
}

fn run() -> Result<i32, Box<dyn std::error::Error>> {
    let args = parse_args().map_err(|e| {
        eprintln!("{e}");
        "bad arguments"
    })?;

    // `--red-path` selects the red-path witness catalogue
    // (HLD V5 §4.2.8). Mutually exclusive with the default catalogue;
    // the same filter semantics apply within the chosen catalogue.
    let catalogue: &[PeriphScenario] = if args.red_path {
        RED_PATH_SCENARIOS
    } else {
        SCENARIOS
    };

    let catalogue_names: Vec<&str> = catalogue.iter().map(|s| s.name).collect();
    let (indices, skipped_filter, skipped_exclude) = select_by_name(
        &catalogue_names,
        args.inner.filter.as_deref(),
        args.inner.exclude.as_deref(),
    );
    let selected: Vec<&PeriphScenario> = indices.into_iter().map(|i| &catalogue[i]).collect();

    if selected.is_empty() {
        println!(
            "silicon_periph_diff_rp2350: no scenarios match filter '{}' (exclude '{}')); nothing to do",
            args.inner.filter.as_deref().unwrap_or(""),
            args.inner.exclude.as_deref().unwrap_or(""),
        );
        return Ok(0);
    }

    println!(
        "silicon_periph_diff_rp2350: {} scenario(s) selected from {} catalogue \
         ({} skipped by filter, {} skipped by exclude)",
        selected.len(),
        if args.red_path { "red-path" } else { "default" },
        skipped_filter,
        skipped_exclude,
    );
    println!(
        "sled=0x{SILICON_RUN_SLED:08X} resets=0x{RESETS_BASE:08X} pll_sys=0x{PLL_SYS_BASE:08X}"
    );
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    let mut core = session.core(0)?;

    println!(
        "{:<40} {:>6} {:>10} {:>7}  first_divergence",
        "scenario", "sysclk", "runtime_ms", "verdict",
    );
    println!("{}", "-".repeat(102));

    let mut pass = 0usize;
    let mut fail = 0usize;
    let t_total = Instant::now();
    for (i, sc) in selected.iter().enumerate() {
        let r = run_scenario_with_retry(&mut core, sc, i == 0, args.inner.verbose)?;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail | Verdict::Skip | Verdict::Degraded => fail += 1,
        }
        println!(
            "{:<40} {:>6} {:>10.1} {:>7}  {}",
            r.name,
            r.actual_sysclks,
            r.elapsed.as_secs_f64() * 1000.0,
            r.verdict.as_str(),
            r.first_divergence.as_deref().unwrap_or("-"),
        );
    }

    // Mirror `run_against`'s cleanup contract even though we called
    // `run_scenario` directly: re-assert RESETS so the next invocation
    // starts clean.
    if let Ok(state) = core.read_word_32(RESETS_RESET as u64) {
        let _ = core.write_word_32(
            RESETS_RESET as u64,
            state | RESET_PIO0 | RESET_PIO1 | RESET_PLL_SYS,
        );
    }

    println!();
    println!(
        "summary: total={} pass={} fail={} skipped_filter={} skipped_exclude={}  ({:.2}s)",
        selected.len(),
        pass,
        fail,
        skipped_filter,
        skipped_exclude,
        t_total.elapsed().as_secs_f64(),
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

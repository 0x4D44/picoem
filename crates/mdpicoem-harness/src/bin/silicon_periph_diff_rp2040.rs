// silicon_periph_diff_rp2040 — peripheral-state oracle for mdrp2040 vs
// real RP2040 silicon. Thin CLI wrapper around the
// `silicon_periph_rp2040::run_against` library API; the catalogue, sled
// assembler, scenario runner, and SysTick timing-window logic all live
// in `crates/mdpicoem-harness/src/silicon_periph_rp2040.rs`.
//
// Phase 0 sub-task 0.E per
// `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V7.md` §4.2 / §4.4.

use mdpicoem_harness::SILICON_RUN_SLED;
use mdpicoem_harness::silicon_oracle::Verdict;
use mdpicoem_harness::silicon_periph_rp2040::{self, PeriphArgs, SCENARIOS, TIMER_BASE, XOSC_BASE};
use probe_rs::{Session, SessionConfig};
use std::time::Instant;

const RESETS_BASE: u32 = 0x4000_C000;

const USAGE: &str = "\
Usage: silicon_periph_diff_rp2040 [--filter <substr>] [--verbose]

Options:
  --filter   Only run scenarios whose name contains <substr>
  --verbose  Print per-observable diffs, not just the first divergence
";

fn parse_args() -> Result<PeriphArgs, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut args = PeriphArgs::default();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--filter" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--filter requires a substring\n{USAGE}"));
                }
                args.filter = Some(argv[i].clone());
            }
            "--verbose" => args.verbose = true,
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

    let selected: Vec<&silicon_periph_rp2040::PeriphScenario> = SCENARIOS
        .iter()
        .filter(|s| {
            args.filter
                .as_deref()
                .is_none_or(|sub| s.name.contains(sub))
        })
        .collect();
    let skipped = SCENARIOS.len() - selected.len();
    if selected.is_empty() {
        println!(
            "silicon_periph_diff_rp2040: no scenarios match filter '{}'; nothing to do",
            args.filter.as_deref().unwrap_or(""),
        );
        return Ok(0);
    }

    println!(
        "silicon_periph_diff_rp2040: {} scenario(s) selected ({} skipped by filter)",
        selected.len(),
        skipped,
    );
    println!(
        "sled=0x{:08X} resets=0x{:08X} timer=0x{:08X} xosc=0x{:08X}",
        SILICON_RUN_SLED, RESETS_BASE, TIMER_BASE, XOSC_BASE,
    );
    println!();

    let mut session = Session::auto_attach("rp2040", SessionConfig::default())?;
    let mut core = session.core(0)?;

    println!(
        "{:<40} {:>10} {:>7}  first_divergence",
        "scenario", "runtime_ms", "verdict",
    );
    println!("{}", "-".repeat(102));

    let t_total = Instant::now();
    // Build an order list so the binary preserves catalogue order even
    // after filtering. Passing `order = Some(...)` also bypasses the
    // library's filter/exclude path; the filter is purely for the binary's
    // own progress reporting.
    let order: Vec<&str> = selected.iter().map(|s| s.name).collect();
    let outcomes = silicon_periph_rp2040::run_against(&mut core, &args, Some(&order), None)
        .map_err(|e| -> Box<dyn std::error::Error> { e.to_string().into() })?;

    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;
    let mut degraded = 0usize;
    for o in &outcomes {
        match o.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail => fail += 1,
            Verdict::Skip => skip += 1,
            Verdict::Degraded => degraded += 1,
        }
        println!(
            "{:<40} {:>10} {:>7}  {}",
            o.case,
            o.elapsed_ms,
            o.verdict.as_str(),
            if o.detail.is_empty() { "-" } else { &o.detail },
        );
    }

    println!();
    println!(
        "summary: total={} pass={} fail={} skip={} degraded={} filter_skipped={}  ({:.2}s)",
        outcomes.len(),
        pass,
        fail,
        skip,
        degraded,
        skipped,
        t_total.elapsed().as_secs_f64(),
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

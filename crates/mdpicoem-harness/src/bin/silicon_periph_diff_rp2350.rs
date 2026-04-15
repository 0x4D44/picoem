// silicon_periph_diff_rp2350 — peripheral-state oracle for mdrp2350 vs
// real RP2354 silicon.
//
// Thin CLI wrapper over `silicon_scenarios::run_against`. Per-scenario
// setup + diff lives in the library module so the `test_silicon`
// orchestrator can share it.

use mdpicoem_harness::silicon_oracle::Verdict;
use mdpicoem_harness::silicon_scenarios::{
    run_scenario, PeriphArgs, PeriphScenario, PLL_SYS_BASE, RESETS_BASE, RESETS_RESET,
    RESET_PIO0, RESET_PIO1, RESET_PLL_SYS, SCENARIOS,
};
use mdpicoem_harness::SILICON_RUN_SLED;
use probe_rs::{MemoryInterface, Session, SessionConfig};
use std::time::Instant;

const USAGE: &str = "\
Usage: silicon_periph_diff_rp2350 [--filter <substr>] [--verbose]

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

    let selected: Vec<&PeriphScenario> = SCENARIOS
        .iter()
        .filter(|s| args.filter.as_deref().is_none_or(|sub| s.name.contains(sub)))
        .collect();

    let skipped = SCENARIOS.len() - selected.len();
    if selected.is_empty() {
        println!(
            "silicon_periph_diff_rp2350: no scenarios match filter '{}'; nothing to do",
            args.filter.as_deref().unwrap_or(""),
        );
        return Ok(0);
    }

    println!(
        "silicon_periph_diff_rp2350: {} scenario(s) selected ({} skipped by filter)",
        selected.len(),
        skipped,
    );
    println!("sled=0x{SILICON_RUN_SLED:08X} resets=0x{RESETS_BASE:08X} pll_sys=0x{PLL_SYS_BASE:08X}");
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    let mut core = session.core(0)?;

    println!(
        "{:<28} {:>6} {:>10} {:>7}  {}",
        "scenario", "sysclk", "runtime_ms", "verdict", "first_divergence",
    );
    println!("{}", "-".repeat(98));

    let mut pass = 0usize;
    let mut fail = 0usize;
    let t_total = Instant::now();
    for (i, sc) in selected.iter().enumerate() {
        let r = run_scenario(&mut core, sc, i == 0, args.verbose)?;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail => fail += 1,
        }
        println!(
            "{:<28} {:>6} {:>10.1} {:>7}  {}",
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
        "summary: total={} pass={} fail={} skipped={}  ({:.2}s)",
        selected.len(),
        pass,
        fail,
        skipped,
        t_total.elapsed().as_secs_f64(),
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

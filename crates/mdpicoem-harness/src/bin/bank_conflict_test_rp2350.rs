// bank_conflict_test_rp2350 — SRAM bank-conflict oracle for mdrp2350 vs
// real RP2354 silicon (halt-step protocol).
//
// Thin CLI wrapper over `bank_conflict_cases::run_against`. Per the HLD
// (wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage
// Expansion.md §Library-API extraction), this oracle stays with
// halt-step + NOP-baseline calibration because the K-delta protocol
// masks the +1 cycle SRAM bank-contention signal (see
// tech_debt.md:545-554). The emulator-side measurement runs
// `emu.cores[0].step(&mut emu.bus)` once per case and compares the
// cycle count against the NOP-corrected HW median.

use mdpicoem_harness::bank_conflict_cases::{
    build_catalog, measure_nop_baseline_hw, run_bank_case, BankArgs, BankCase,
};
use mdpicoem_harness::silicon_oracle::{enable_cyccnt, name_matches_filter, Verdict};
use probe_rs::{Session, SessionConfig};
use std::time::{Duration, Instant};

const USAGE: &str = "\
Usage: bank_conflict_test_rp2350 [--filter <substr>] [--num-samples <N>] \
[--tolerance <N>]

Options:
  --filter       Only run cases whose name contains <substr>
  --num-samples  HW samples per case; median reported (default 20)
  --tolerance    |(hw-overhead) - emu| tolerance in cycles (default 1)
";

fn parse_args() -> Result<BankArgs, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut args = BankArgs::default();
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
            "--num-samples" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--num-samples requires a value\n{USAGE}"));
                }
                args.num_samples = argv[i]
                    .parse()
                    .map_err(|e| format!("invalid --num-samples '{}': {e}\n{USAGE}", argv[i]))?;
            }
            "--tolerance" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--tolerance requires a value\n{USAGE}"));
                }
                args.tolerance = argv[i]
                    .parse()
                    .map_err(|e| format!("invalid --tolerance '{}': {e}\n{USAGE}", argv[i]))?;
            }
            "--help" | "-h" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument '{other}'\n{USAGE}")),
        }
        i += 1;
    }
    if args.num_samples == 0 {
        return Err(format!("--num-samples must be >= 1\n{USAGE}"));
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

    let catalog = build_catalog();
    let selected: Vec<&BankCase> = catalog
        .iter()
        .filter(|c| name_matches_filter(c.name, args.filter.as_deref()))
        .collect();

    if selected.is_empty() {
        println!("no bank cases match filter; nothing to do");
        return Ok(0);
    }

    println!(
        "bank_conflict_test_rp2350: halt-step protocol (num_samples={} tol={})",
        args.num_samples, args.tolerance,
    );
    println!(
        "(fetch lives in bank 0 by default; data bank cycles through 0..8 for LDR + STR; \
         fetch-bank sweep cycles fetch 0..8)"
    );
    println!("selected {} case(s)", selected.len());
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    let mut core = session.core(0)?;
    core.reset_and_halt(Duration::from_millis(500))?;
    enable_cyccnt(&mut core)?;

    // Calibrate the NOP baseline once per run — mirrors the original
    // `bank_conflict_test` and matches `run_against`'s contract so the
    // binary and orchestrator agree on the debug-overhead constant.
    let nop_baseline = measure_nop_baseline_hw(&mut core, args.num_samples)?;
    println!("nop_baseline (median) = {nop_baseline} cycles");
    println!();

    println!(
        "{:<26} {:>5} {:>5} {:>11} {:>9} {:>7} {:>7} {:>+7} {:>6} {:>6}",
        "case",
        "fbnk",
        "dbnk",
        "data",
        "hw_med",
        "hw_adj",
        "emu",
        "delta",
        "tol",
        "verdict",
    );
    println!("{}", "-".repeat(110));

    let mut total = 0usize;
    let mut pass = 0usize;
    let mut fail = 0usize;
    let t0 = Instant::now();
    for case in &selected {
        let r = run_bank_case(&mut core, case, args.num_samples, args.tolerance, nop_baseline)?;
        total += 1;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail => fail += 1,
        }
        println!(
            "{:<26} {:>5} {:>5} 0x{:08X} {:>9} {:>7} {:>7} {:>+7} {:>6} {:>6}",
            r.name,
            r.fetch_bank,
            r.data_bank,
            r.data_addr,
            r.hw_median,
            r.hw_adjusted,
            r.emu_cycles,
            r.delta,
            args.tolerance,
            r.verdict.as_str(),
        );
        if r.emu_cycles != r.emu_baseline {
            println!(
                "    NOTE: emu ({}) differs from BankCase::emu_baseline ({}); update catalogue",
                r.emu_cycles, r.emu_baseline,
            );
        }
        println!(
            "    samples: {:?}  (fetch=0x{:08X})",
            r.hw_samples, r.fetch_addr,
        );
    }

    let elapsed = t0.elapsed();
    println!();
    println!(
        "summary: total={total} pass={pass} fail={fail}  ({:.2}s)",
        elapsed.as_secs_f64()
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

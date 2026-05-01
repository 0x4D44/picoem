// silicon_cycle_oracle_rp2350 — sequence-in-loop cycle-cost oracle for
// mdrp2350 vs. real RP2354 silicon.
//
// Thin CLI wrapper over `cycle_cases::run_against`. The catalogue + the
// measurement protocol live in the library module so the `test_silicon`
// orchestrator can share them.
//
// Usage:
//   silicon_cycle_oracle_rp2350
//   silicon_cycle_oracle_rp2350 -- --filter push
//   silicon_cycle_oracle_rp2350 -- --iter-low 51 --iter-high 151
//   silicon_cycle_oracle_rp2350 -- --tolerance 1

use mdpicoem_harness::CYCLE_MAILBOX_BASE;
use mdpicoem_harness::cycle_cases::{
    self, CASES, CYCLE_SEQ_SLOT, CycleArgs, CycleCase, DWT_CYCCNT_ADDR, STUB_START, run_cycle_case,
};
use mdpicoem_harness::silicon_oracle::{Verdict, enable_cyccnt, select_by_name};
use probe_rs::{MemoryInterface, Session, SessionConfig};
use std::time::{Duration, Instant};

const USAGE: &str = "\
Usage: silicon_cycle_oracle_rp2350 [--filter <substr>] [--exclude <substr>] \
[--iter-low <K1>] [--iter-high <K2>] [--tolerance <N>]

Options:
  --filter    Only run cases whose name contains <substr>
  --exclude   Skip cases whose name contains <substr> (applied after --filter)
  --iter-low  K_low   (default 101)
  --iter-high K_high  (default 201, must be > K_low)
  --tolerance Cycle-delta tolerance before marking FAIL (default 0)
";

fn parse_args() -> Result<CycleArgs, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut args = CycleArgs::default();
    let mut i = 0;
    while i < argv.len() {
        let a = argv[i].as_str();
        match a {
            "--filter" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--filter requires a substring\n{USAGE}"));
                }
                args.filter = Some(argv[i].clone());
            }
            "--exclude" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--exclude requires a substring\n{USAGE}"));
                }
                args.exclude = Some(argv[i].clone());
            }
            "--iter-low" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--iter-low requires a value\n{USAGE}"));
                }
                args.iter_low = argv[i]
                    .parse()
                    .map_err(|e| format!("invalid --iter-low '{}': {e}\n{USAGE}", argv[i]))?;
            }
            "--iter-high" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--iter-high requires a value\n{USAGE}"));
                }
                args.iter_high = argv[i]
                    .parse()
                    .map_err(|e| format!("invalid --iter-high '{}': {e}\n{USAGE}", argv[i]))?;
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
    if args.iter_high <= args.iter_low {
        return Err(format!(
            "--iter-high ({}) must be > --iter-low ({})\n{USAGE}",
            args.iter_high, args.iter_low,
        ));
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

    let case_names: Vec<&str> = CASES.iter().map(|c| c.name).collect();
    let (indices, skipped_filter, skipped_exclude) =
        select_by_name(&case_names, args.filter.as_deref(), args.exclude.as_deref());
    let selected: Vec<&CycleCase> = indices.into_iter().map(|i| &CASES[i]).collect();

    if selected.is_empty() {
        println!("no cases match filter/exclude; nothing to do");
        return Ok(0);
    }

    println!(
        "silicon_cycle_oracle_rp2350: K_low={} K_high={} tol={}",
        args.iter_low, args.iter_high, args.tolerance,
    );
    println!(
        "stub=0x{STUB_START:08X} seq=0x{CYCLE_SEQ_SLOT:08X} mailbox=0x{CYCLE_MAILBOX_BASE:08X} dwt=0x{DWT_CYCCNT_ADDR:08X}",
    );
    println!(
        "selected {} case(s) ({} skipped by filter, {} skipped by exclude)",
        selected.len(),
        skipped_filter,
        skipped_exclude,
    );
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    let mut core = session.core(0)?;
    core.reset_and_halt(Duration::from_millis(500))?;
    enable_cyccnt(&mut core)?;

    // Header row. Run each case via `run_cycle_case` so we keep the full
    // per-case diagnostic output the standalone binary has always produced;
    // `cycle_cases::run_against` is the same path but returns only
    // `CaseOutcome` (used by the orchestrator).
    //
    // "tol" column shows the effective budget (max of per-case and CLI).
    // "verdict" column annotates known-delta passes per the Track B Cycle
    // Oracle Fidelity HLD — tolerance is a floor, not a silencer, so the
    // hw/emu/delta columns still surface any drift.
    println!(
        "{:<36} {:>10} {:>10} {:>10} {:>10} {:>6}  verdict",
        "case", "HW/iter", "EMU/iter", "delta", "baseline", "tol",
    );
    println!("{}", "-".repeat(108));

    // Prepare the stub + mailbox once — `run_cycle_case` assumes they are
    // resident. `run_against` does this internally; here we mirror it so
    // the per-case loop stays thin.
    //
    // The cleanest way to share is to route through `run_against` and get
    // the outcomes back, then print. But the standalone binary wants its
    // HW/EMU m_low/m_high samples in the diagnostic output, which
    // `CaseOutcome` does not carry. Call the per-case helper directly.
    let stub_bytes = cycle_cases::pack_stub();
    core.write_8(STUB_START as u64, &stub_bytes)?;
    // Zero mailbox by writing all six slots.
    for off in [0u32, 4, 8, 12, 16, 20] {
        core.write_word_32((CYCLE_MAILBOX_BASE + off) as u64, 0)?;
    }

    let mut total = 0usize;
    let mut pass = 0usize;
    let mut fail = 0usize;
    let t0 = Instant::now();
    for case in &selected {
        let r = run_cycle_case(
            &mut core,
            case,
            args.iter_low,
            args.iter_high,
            args.tolerance,
        )?;
        total += 1;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail | Verdict::Skip | Verdict::Degraded => fail += 1,
        }
        // HLD §6.2 + lead addendum: when a case passes only due to
        // non-zero effective tolerance, print `PASS (known Δ=<delta>,
        // tol=<N>)`. The full hw/emu/delta columns are still emitted
        // above so any drift remains visible.
        let verdict_str = if r.known_delta_pass {
            format!(
                "PASS (known Δ={:+}, tol={})",
                r.delta, r.effective_tolerance
            )
        } else {
            r.verdict.as_str().to_string()
        };
        println!(
            "{:<36} {:>10} {:>10} {:>+10} {:>10} {:>6}  {}",
            r.name,
            r.hw_per_iter,
            r.emu_per_iter,
            r.delta,
            r.emu_baseline,
            r.effective_tolerance,
            verdict_str,
        );
        if r.emu_per_iter != r.emu_baseline {
            println!(
                "    NOTE: emu per-iter ({}) differs from catalog emu_baseline ({}); update CycleCase::emu_baseline",
                r.emu_per_iter, r.emu_baseline,
            );
        }
        println!(
            "    HW  m_low={} m_high={}   EMU m_low={} m_high={}",
            r.hw_low, r.hw_high, r.emu_low, r.emu_high,
        );
    }

    let elapsed = t0.elapsed();
    println!();
    println!(
        "summary: total={total} pass={pass} fail={fail} \
         skipped_filter={skipped_filter} skipped_exclude={skipped_exclude}  ({:.2}s)",
        elapsed.as_secs_f64()
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

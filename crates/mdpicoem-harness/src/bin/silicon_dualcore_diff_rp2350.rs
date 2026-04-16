// silicon_dualcore_diff_rp2350 — dual-core contention cycle-cost oracle
// for mdrp2350 vs. real RP2354 silicon.
//
// Thin CLI wrapper over `dualcore_cases::run_against`. Each case runs the
// cycle-oracle K-delta measurement stub on core 0 while core 1 spins an
// antagonist loop uploaded to `DUALCORE_ANTAGONIST_SLOT`. The per-iter
// HW and EMU numbers are diffed; a delta outside the tolerance fails.
//
// Usage:
//   silicon_dualcore_diff_rp2350
//   silicon_dualcore_diff_rp2350 -- --filter spinlock
//   silicon_dualcore_diff_rp2350 -- --iter-low 51 --iter-high 151
//   silicon_dualcore_diff_rp2350 -- --tolerance 1
//
// Hardware prerequisite: Pico debug probe attached to an RP2354 board.
// See the HLD §Component 3 and `CLAUDE.md` under "Testing Topology" for
// the hardware-gated prerequisites.

use mdpicoem_harness::dualcore_cases::{self, DualCoreArgs, DualCoreCase, CASES};
use mdpicoem_harness::silicon_oracle::{enable_cyccnt, select_by_name, Verdict};
use mdpicoem_harness::{CYCLE_MAILBOX_BASE, DUALCORE_ANTAGONIST_SLOT};
use probe_rs::{MemoryInterface, Session, SessionConfig};
use std::time::{Duration, Instant};

const USAGE: &str = "\
Usage: silicon_dualcore_diff_rp2350 [--filter <substr>] [--exclude <substr>] \
[--iter-low <K1>] [--iter-high <K2>] [--tolerance <N>]

Options:
  --filter    Only run cases whose name contains <substr>
  --exclude   Skip cases whose name contains <substr> (applied after --filter)
  --iter-low  K_low   (default 101)
  --iter-high K_high  (default 201, must be > K_low)
  --tolerance Cycle-delta tolerance before marking FAIL (default 0)
";

fn parse_args() -> Result<DualCoreArgs, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut args = DualCoreArgs::default();
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
    let selected: Vec<&DualCoreCase> = indices.into_iter().map(|i| &CASES[i]).collect();

    if selected.is_empty() {
        println!("no cases match filter/exclude; nothing to do");
        return Ok(0);
    }

    println!(
        "silicon_dualcore_diff_rp2350: K_low={} K_high={} tol={}",
        args.iter_low, args.iter_high, args.tolerance,
    );
    println!(
        "antagonist=0x{DUALCORE_ANTAGONIST_SLOT:08X} mailbox=0x{CYCLE_MAILBOX_BASE:08X}",
    );
    println!(
        "selected {} case(s) ({} skipped by filter, {} skipped by exclude)",
        selected.len(), skipped_filter, skipped_exclude,
    );
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    {
        let mut core = session.core(0)?;
        core.reset_and_halt(Duration::from_millis(500))?;
        enable_cyccnt(&mut core)?;
    }

    // Assumption 1 dependency — see the top of `dualcore_cases` and
    // the Stage 1 smoke binary. If CYCCNT on RP2354 is aliased across
    // cores rather than per-core, core 0's readings get polluted by
    // core 1's concurrent execution and EVERY case will report HW != EMU.
    println!(
        "NOTE: requires per-core CYCCNT (Assumption 1). If unverified, run \
         smoke_per_core_cyccnt_rp2350 first.",
    );

    // Header row for the per-case table.
    println!(
        "{:<36} {:>10} {:>10} {:>10} {:>10} {:>6} {:>6}",
        "case", "HW/iter", "EMU/iter", "delta", "baseline", "tol", "verdict",
    );
    println!("{}", "-".repeat(96));

    // Prepare the stub + mailbox once — `run_case_rich` / `run_against`
    // both assume they are resident. The library's `run_against` does the
    // same sequence; we mirror it here so the standalone binary keeps its
    // per-case diagnostic output without duplicating the measurement
    // body.
    let stub_bytes = mdpicoem_harness::cycle_cases::pack_stub();
    {
        let mut core = session.core(0)?;
        core.write_8(mdpicoem_harness::cycle_cases::STUB_START as u64, &stub_bytes)?;
        // Zero mailbox (six u32 slots).
        for off in [0u32, 4, 8, 12, 16, 20] {
            core.write_word_32((CYCLE_MAILBOX_BASE + off) as u64, 0)?;
        }
    }

    let mut total = 0usize;
    let mut pass = 0usize;
    let mut fail = 0usize;
    let t0 = Instant::now();
    for case in &selected {
        let r = dualcore_cases::run_case_rich(
            &mut session,
            case,
            args.iter_low,
            args.iter_high,
            args.tolerance,
        )?;
        total += 1;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail => fail += 1,
        }
        let eff_tol = dualcore_cases::effective_tolerance(case, args.tolerance);
        println!(
            "{:<36} {:>10} {:>10} {:>+10} {:>10} {:>6} {:>6}",
            r.name,
            r.hw_per_iter,
            r.emu_per_iter,
            r.delta,
            r.emu_baseline,
            eff_tol,
            r.verdict.as_str(),
        );
        if r.emu_per_iter != r.emu_baseline {
            println!(
                "    NOTE: emu per-iter ({}) differs from catalog emu_baseline ({}); update DualCoreCase::emu_baseline",
                r.emu_per_iter, r.emu_baseline,
            );
        }
        println!(
            "    HW  m_low={} m_high={}   EMU m_low={} m_high={}",
            r.hw_low, r.hw_high, r.emu_low, r.emu_high,
        );
    }

    // Best-effort cleanup mirror of the library's cleanup contract: halt
    // core 1, release SPINLOCK 0. `run_case_rich` / `run_dualcore_case`
    // leave core 1 halted between cases, but we also let a final pass
    // here catch any errant state.
    {
        if let Ok(mut c1) = session.core(1) {
            let _ = c1.halt(Duration::from_millis(200));
        }
        if let Ok(mut c0) = session.core(0) {
            let _ = c0.write_word_32(0xD000_0100u64, 1);
        }
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

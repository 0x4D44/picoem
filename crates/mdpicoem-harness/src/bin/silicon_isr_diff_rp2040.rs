// silicon_isr_diff_rp2040 — ISR differential oracle for mdrp2040 vs.
// real RP2040 silicon.
//
// Thin CLI wrapper around `isr_scenarios_rp2040::run_against`. Each
// scenario uploads a hand-assembled SRAM image (17-entry vector
// table + handler stubs + main routine + literal pool) at
// `ISR_IMAGE_BASE`, reprograms VTOR to point there, runs main which
// pends the relevant exception(s), and diffs the resulting SRAM
// counters + peripheral-pending observables. Ships the V1 minimum
// (timer_cold + tail_chain) plus the V2 expansion
// (nvic_high_bits_razwi, masked_pending_unmask, wfi_wake,
// priority_preempt). All six scenarios are silicon-validated.
//
// Usage:
//   silicon_isr_diff_rp2040
//   silicon_isr_diff_rp2040 --filter tail
//   silicon_isr_diff_rp2040 --verbose
//
// Hardware prerequisite: Pico debug probe attached to an RP2040
// board.

use mdpicoem_harness::isr_scenarios_rp2040::{self, IsrArgs};
use mdpicoem_harness::silicon_oracle::Verdict;
use mdpicoem_harness::{ISR_IMAGE_BASE, ISR_STACK_TOP};
use probe_rs::{Session, SessionConfig};
use std::time::{Duration, Instant};

const USAGE: &str = "\
Usage: silicon_isr_diff_rp2040 [--filter <substr>] [--verbose]

Options:
  --filter   Only run scenarios whose name contains <substr>
  --verbose  Print all observables, not just the first divergence
";

fn parse_args() -> Result<IsrArgs, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut args = IsrArgs::default();
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

    let selected: Vec<&isr_scenarios_rp2040::IsrScenario> = isr_scenarios_rp2040::SCENARIOS
        .iter()
        .filter(|s| {
            mdpicoem_harness::silicon_oracle::name_matches_filter(s.name, args.filter.as_deref())
        })
        .collect();

    if selected.is_empty() {
        println!(
            "silicon_isr_diff_rp2040: no scenarios match filter '{}'; nothing to do",
            args.filter.as_deref().unwrap_or(""),
        );
        return Ok(0);
    }

    println!(
        "silicon_isr_diff_rp2040: {} scenario(s) selected",
        selected.len(),
    );
    println!("image_base=0x{ISR_IMAGE_BASE:08X} stack_top=0x{ISR_STACK_TOP:08X}",);
    println!(
        "NOTE: V5 IRQ plumbing is complete (NVIC MMIO + SysTick + unified exception dispatcher); EMU-side scenarios should pass.",
    );
    println!();

    let mut session = Session::auto_attach("rp2040", SessionConfig::default())?;
    {
        let mut core = session.core(0)?;
        core.reset_and_halt(Duration::from_millis(500))?;
    }

    let t_total = Instant::now();
    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;
    let mut degraded = 0usize;

    println!(
        "{:<40} {:>8} {:>8}  first_divergence",
        "scenario", "elapsed", "verdict",
    );
    println!("{}", "-".repeat(102));

    {
        let mut core = session.core(0)?;
        let outcomes = isr_scenarios_rp2040::run_against(&mut core, &args, None, None)
            .map_err(|e| -> Box<dyn std::error::Error> { e.to_string().into() })?;
        for o in &outcomes {
            match o.verdict {
                Verdict::Pass => pass += 1,
                Verdict::Fail => fail += 1,
                Verdict::Skip => skip += 1,
                Verdict::Degraded => degraded += 1,
            }
            println!(
                "{:<40} {:>6}ms {:>8}  {}",
                o.case,
                o.elapsed_ms,
                o.verdict.as_str(),
                if o.detail.is_empty() { "-" } else { &o.detail },
            );
        }
    }

    let elapsed = t_total.elapsed();
    println!();
    println!(
        "summary: total={} pass={} fail={} skip={} degraded={}  ({:.2}s)",
        pass + fail + skip + degraded,
        pass,
        fail,
        skip,
        degraded,
        elapsed.as_secs_f64(),
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

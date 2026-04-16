// silicon_isr_diff_rp2350 — exception entry / lazy FP save oracle for
// mdrp2350 vs. real RP2354 silicon.
//
// Thin CLI wrapper over `isr_scenarios::run_against`. Each scenario
// uploads a hand-assembled SRAM image (vector table + handler stub +
// main routine + literal pool) at `ISR_IMAGE_BASE`, reprograms VTOR to
// point there (both Secure and Non-Secure aliases), runs main which
// pends PendSV (and optionally SysTick / dirties the FP state), then
// diffs stacked-frame / FPCCR / CYCCNT observables post-BKPT.
//
// Usage:
//   silicon_isr_diff_rp2350
//   silicon_isr_diff_rp2350 --filter lazy
//   silicon_isr_diff_rp2350 --verbose
//
// Hardware prerequisite: Pico debug probe attached to an RP2354 board.
// See the HLD §Component 2 and `CLAUDE.md` under "Testing Topology" for
// the hardware-gated prerequisites. The ISR oracle is directly
// addressed to `tech_debt.md:295` ("Exception entry/exit not
// differentially validated") — v1 scenarios are expected to FAIL on the
// EMU side until mdrp2350's step loop polls ICSR for pending exceptions.

use mdpicoem_harness::isr_scenarios::{self, IsrArgs, IsrScenario, SCENARIOS};
use mdpicoem_harness::silicon_oracle::{enable_cyccnt, name_matches_filter, Verdict};
use mdpicoem_harness::{ISR_IMAGE_BASE, ISR_MAILBOX_CYCCNT, ISR_STACK_TOP};
use probe_rs::{Session, SessionConfig};
use std::time::{Duration, Instant};

const USAGE: &str = "\
Usage: silicon_isr_diff_rp2350 [--filter <substr>] [--verbose]

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

    let selected: Vec<&IsrScenario> = SCENARIOS
        .iter()
        .filter(|s| name_matches_filter(s.name, args.filter.as_deref()))
        .collect();

    if selected.is_empty() {
        println!(
            "silicon_isr_diff_rp2350: no scenarios match filter '{}'; nothing to do",
            args.filter.as_deref().unwrap_or(""),
        );
        return Ok(0);
    }

    println!(
        "silicon_isr_diff_rp2350: {} scenario(s) selected",
        selected.len(),
    );
    println!(
        "image_base=0x{ISR_IMAGE_BASE:08X} stack_top=0x{ISR_STACK_TOP:08X} mailbox=0x{ISR_MAILBOX_CYCCNT:08X}",
    );
    println!(
        "NOTE: v1 expected to FAIL on EMU until pending-exception dispatch lands (tech_debt.md:295)",
    );
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    {
        let mut core = session.core(0)?;
        core.reset_and_halt(Duration::from_millis(500))?;
        enable_cyccnt(&mut core)?;
    }

    let t_total = Instant::now();
    let mut pass = 0usize;
    let mut fail = 0usize;

    println!(
        "{:<40} {:>8} {:>8}  {}",
        "scenario", "elapsed", "verdict", "first_divergence",
    );
    println!("{}", "-".repeat(96));

    {
        let mut core = session.core(0)?;
        let outcomes = isr_scenarios::run_against(&mut core, &args, None)?;
        for o in &outcomes {
            match o.verdict {
                Verdict::Pass => pass += 1,
                Verdict::Fail => fail += 1,
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
        "summary: total={} pass={} fail={}  ({:.2}s)",
        pass + fail,
        pass,
        fail,
        elapsed.as_secs_f64(),
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

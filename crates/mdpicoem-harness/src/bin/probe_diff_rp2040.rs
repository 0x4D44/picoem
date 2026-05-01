// Hardware differential test runner — mdrp2040 (Cortex-M0+) vs real RP2040
// silicon via SWD.
//
// Thin CLI wrapper around the `probe_diff_rp2040_lib` library API. The
// M0+ silicon-safe filter, the per-test probe / emulator drivers, the
// `DiffError` enum, the post-step PC classifier, and the rc=1/3 mapping
// all live in `crates/mdpicoem-harness/src/probe_diff_rp2040_lib.rs` so
// the `test_silicon_rp2040` orchestrator can reuse them.
//
// Usage (mirrors `probe_diff_rp2350` minus `--cycles`):
//   probe_diff_rp2040                      Run targeted edge-case tests
//   probe_diff_rp2040 --fuzz N             Random fuzz tests (N per class)
//   probe_diff_rp2040 --fuzz N --seed S    Reproducible fuzz

use mdpicoem_harness::cli::parse_probe_selector;
use mdpicoem_harness::m0plus::Bus as M0Bus;
use mdpicoem_harness::probe_diff_rp2040_lib::{
    DiffError, is_m0plus_silicon_safe, rc_for, run_one_diff,
};
use mdpicoem_harness::{TestCase, generate_all, generate_fuzz};
use probe_rs::probe::{DebugProbeSelector, list::Lister};
use probe_rs::{Core, Permissions, Session, SessionConfig};
use std::time::{Duration, Instant};

fn main() {
    mdpicoem_harness::harness_tracing_init();
    if let Err(e) = run() {
        eprintln!("fatal: {e}");
        let mut source = std::error::Error::source(&*e);
        while let Some(s) = source {
            eprintln!("  caused by: {s}");
            source = s.source();
        }
        std::process::exit(2);
    }
}

// ============================================================================
// Argument parsing
// ============================================================================

struct Args {
    fuzz_count: Option<usize>,
    seed: Option<u64>,
    probe: Option<DebugProbeSelector>,
}

fn parse_args_from<I: IntoIterator<Item = String>>(argv: I) -> Result<Args, String> {
    let args: Vec<String> = argv.into_iter().collect();
    let mut fuzz_count = None;
    let mut seed = None;
    let mut probe = None;
    let mut i = 0;

    while i < args.len() {
        match args[i].as_str() {
            "--fuzz" => {
                i += 1;
                if i >= args.len() {
                    return Err("--fuzz requires a count argument".into());
                }
                fuzz_count = Some(
                    args[i]
                        .parse::<usize>()
                        .map_err(|e| format!("invalid fuzz count '{}': {e}", args[i]))?,
                );
            }
            "--seed" => {
                i += 1;
                if i >= args.len() {
                    return Err("--seed requires a value argument".into());
                }
                seed = Some(
                    args[i]
                        .parse::<u64>()
                        .map_err(|e| format!("invalid seed '{}': {e}", args[i]))?,
                );
            }
            "--probe" => {
                i += 1;
                if i >= args.len() {
                    return Err("--probe requires a VID:PID:SERIAL argument".into());
                }
                probe = Some(parse_probe_selector(&args[i])?);
            }
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     probe_diff_rp2040                      Run targeted edge-case tests\n  \
                     probe_diff_rp2040 --fuzz N             Random fuzz tests (N per class)\n  \
                     probe_diff_rp2040 --fuzz N --seed S    Reproducible fuzz\n  \
                     probe_diff_rp2040 --probe VID:PID:SERIAL  Select a specific probe"
                ));
            }
        }
        i += 1;
    }

    if seed.is_some() && fuzz_count.is_none() {
        return Err("--seed requires --fuzz".into());
    }

    Ok(Args {
        fuzz_count,
        seed,
        probe,
    })
}

fn parse_args() -> Result<Args, String> {
    parse_args_from(std::env::args().skip(1))
}

// ============================================================================
// Attach helpers
// ============================================================================

/// Attach to the target with a bounded retry loop. Track A.1 Phase 2 Option F
/// (defensive posture — see `wrk_docs/2026.04.22 - HLD - Track A.1 RP2040
/// Attach Fix.md` §6).
fn attach_with_retry(
    chip: &str,
    selector: Option<&DebugProbeSelector>,
    max_attempts: usize,
) -> Result<Session, probe_rs::Error> {
    let mut last_err: Option<probe_rs::Error> = None;
    for attempt in 0..max_attempts {
        if attempt > 0 {
            tracing::info!("attach retry {attempt}/{max_attempts}");
            std::thread::sleep(Duration::from_millis(300));
        }
        let result = match selector {
            None => Session::auto_attach(chip, SessionConfig::default()),
            Some(sel) => Lister::new()
                .open(sel.clone())
                .map_err(probe_rs::Error::from)
                .and_then(|p| p.attach(chip, Permissions::default())),
        };
        match result {
            Ok(s) => return Ok(s),
            Err(e) => last_err = Some(e),
        }
    }
    Err(last_err.expect("attach_with_retry: max_attempts must be >= 1"))
}

// ============================================================================
// Runner
// ============================================================================

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;

    println!("probe_diff_rp2040: RP2040 hardware differential test runner");
    println!("===========================================================");

    let mut session = attach_with_retry("rp2040", args.probe.as_ref(), 3)?;
    let mut core = session.core(0)?;
    println!("Attached to target, using core 0");

    core.reset_and_halt(Duration::from_millis(500))?;

    match args.fuzz_count {
        None => run_targeted(&mut core),
        Some(count) => {
            let seed = args.seed.unwrap_or_else(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64
            });
            run_fuzz(&mut core, count, seed)
        }
    }
}

/// Run the targeted edge-case test suite. Drives the library's per-test
/// `run_one_diff` directly so the binary can preserve the rich console
/// output the orchestrator path collapses into a `CaseOutcome.detail`.
fn run_targeted(core: &mut Core) -> Result<(), Box<dyn std::error::Error>> {
    let all = generate_all();
    let tests: Vec<TestCase> = all
        .into_iter()
        .filter(is_m0plus_silicon_safe)
        .filter(|tc| !tc.probe_only)
        .collect();
    let total = tests.len();
    println!("Running {total} M0+-compatible targeted tests...");

    let mut bus = M0Bus::new();
    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;
    let mut undef = 0usize;
    let t0 = Instant::now();

    for (i, tc) in tests.iter().enumerate() {
        if (i + 1) % 100 == 0 {
            eprintln!("[{}/{}] {} failures so far...", i + 1, total, fail);
        }

        match run_one_diff(core, &mut bus, tc) {
            Ok(()) => pass += 1,
            Err(DiffError::Mismatch(diff)) => {
                fail += 1;
                eprintln!(
                    "[FAIL] {}\n  opcode: {:#06x}  hw1: {:?}\n  xpsr_pre: {:#010x}\n  reg_pre: {:?}\n  diff: {}",
                    tc.name, tc.opcode, tc.hw1, tc.xpsr_pre, tc.reg_pre, diff
                );
            }
            Err(DiffError::ProbeError(e)) => {
                skip += 1;
                eprintln!("[SKIP] {}: probe-rs error: {e}", tc.name);
            }
            Err(DiffError::UndefOnSilicon { pc }) => {
                undef += 1;
                eprintln!(
                    "[UNDEF] {}: silicon dispatched to bootrom @ {:#010x} (filter gap)\n  opcode: {:#06x}  hw1: {:?}",
                    tc.name, pc, tc.opcode, tc.hw1
                );
            }
        }
    }

    let elapsed = t0.elapsed();
    print_summary("targeted", pass, fail, skip, undef, elapsed);

    let rc = rc_for(pass, fail, skip, undef);
    if rc == 3 {
        let attempted = pass + fail + skip + undef;
        let pct = (skip * 100) / attempted;
        eprintln!(
            "=== DEGRADED: {skip}/{attempted} cases skipped ({pct}%); probe transport unstable, exiting rc=3 ==="
        );
    }
    if rc != 0 {
        std::process::exit(rc);
    }
    Ok(())
}

fn run_fuzz(
    core: &mut Core,
    count_per_class: usize,
    seed: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Fuzz mode: {count_per_class} tests/class, seed={seed}");
    println!("(reproduce with: probe_diff_rp2040 --fuzz {count_per_class} --seed {seed})");

    let (alu_all, mem_all) = generate_fuzz(count_per_class, seed);
    let alu: Vec<TestCase> = alu_all
        .into_iter()
        .filter(is_m0plus_silicon_safe)
        .filter(|tc| !tc.probe_only)
        .collect();
    let mem: Vec<TestCase> = mem_all
        .into_iter()
        .filter(is_m0plus_silicon_safe)
        .filter(|tc| !tc.probe_only)
        .collect();
    let total = alu.len() + mem.len();
    println!(
        "Generated {total} M0+-compatible tests ({} ALU + {} memory)",
        alu.len(),
        mem.len()
    );

    let mut bus = M0Bus::new();
    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;
    let mut undef = 0usize;
    let mut done = 0usize;
    let t0 = Instant::now();

    for tc in alu.iter().chain(mem.iter()) {
        done += 1;
        if done.is_multiple_of(100) {
            eprintln!("[{done}/{total}] {fail} failures...");
        }

        match run_one_diff(core, &mut bus, tc) {
            Ok(()) => pass += 1,
            Err(DiffError::Mismatch(diff)) => {
                fail += 1;
                eprintln!(
                    "[FAIL] {}\n  opcode: {:#06x}  hw1: {:?}\n  xpsr_pre: {:#010x}\n  reg_pre: {:?}\n  diff: {}",
                    tc.name, tc.opcode, tc.hw1, tc.xpsr_pre, tc.reg_pre, diff
                );
            }
            Err(DiffError::ProbeError(e)) => {
                skip += 1;
                eprintln!("[SKIP] {}: probe-rs error: {e}", tc.name);
            }
            Err(DiffError::UndefOnSilicon { pc }) => {
                undef += 1;
                eprintln!(
                    "[UNDEF] {}: silicon dispatched to bootrom @ {:#010x} (filter gap)\n  opcode: {:#06x}  hw1: {:?}",
                    tc.name, pc, tc.opcode, tc.hw1
                );
            }
        }
    }

    let elapsed = t0.elapsed();
    print_summary("fuzz", pass, fail, skip, undef, elapsed);
    println!("Seed: {seed}");
    let rc = if undef > 0 {
        1
    } else {
        rc_for(pass, fail, skip, undef)
    };
    if rc == 1 {
        println!("Reproduce: probe_diff_rp2040 --fuzz {count_per_class} --seed {seed}");
    }
    if rc == 3 {
        let attempted = pass + fail + skip + undef;
        let pct = (skip * 100) / attempted;
        eprintln!(
            "=== DEGRADED: {skip}/{attempted} cases skipped ({pct}%); probe transport unstable, exiting rc=3 ==="
        );
    }
    if rc != 0 {
        std::process::exit(rc);
    }
    Ok(())
}

fn print_summary(
    mode: &str,
    pass: usize,
    fail: usize,
    skip: usize,
    undef: usize,
    elapsed: Duration,
) {
    let total = pass + fail + skip + undef;
    println!();
    println!("=== {mode} summary ===");
    println!("Total:   {total}");
    println!("Passed:  {pass}");
    println!("Failed:  {fail}");
    println!("Skipped: {skip}");
    println!("Undef:   {undef}");
    println!("Time:    {:.1}s", elapsed.as_secs_f64());
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args, String> {
        parse_args_from(args.iter().map(|s| s.to_string()))
    }

    #[test]
    fn probe_flag_parses_full_selector() {
        let args = parse(&["--probe", "2e8a:000c:ABC"]).expect("selector must parse");
        let sel = args.probe.expect("probe must be Some");
        assert_eq!(sel.vendor_id, 0x2e8a);
        assert_eq!(sel.product_id, 0x000c);
        assert_eq!(sel.serial_number.as_deref(), Some("ABC"));
    }

    #[test]
    fn probe_flag_missing_value_errors() {
        match parse(&["--probe"]) {
            Err(err) => assert!(err.contains("--probe requires"), "unexpected error: {err}"),
            Ok(_) => panic!("bare --probe must error"),
        }
    }

    #[test]
    fn probe_flag_bogus_value_errors_cleanly() {
        match parse(&["--probe", "bogus"]) {
            Err(err) => {
                assert!(
                    err.contains("invalid probe selector"),
                    "error should name the flag: {err}"
                );
                assert!(
                    err.contains("bogus"),
                    "error should echo the bad value: {err}"
                );
            }
            Ok(_) => panic!("bogus selector must error"),
        }
    }

    #[test]
    fn probe_flag_absent_leaves_probe_none() {
        let args = parse(&["--fuzz", "10"]).expect("parse");
        assert!(args.probe.is_none());
    }
}

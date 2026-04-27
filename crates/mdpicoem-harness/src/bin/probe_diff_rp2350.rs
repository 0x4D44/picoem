// Hardware differential test runner: emulator vs real RP2354 silicon via SWD.
//
// Validates Thumb instruction semantics (and optionally cycle counts) by
// executing identical instructions on both the emulator and halted hardware
// (via probe-rs single-step), then comparing post-state.
//
// Usage:
//   probe_diff_rp2350                      Run targeted edge-case tests
//   probe_diff_rp2350 --fuzz N             Random fuzz tests (N per class)
//   probe_diff_rp2350 --fuzz N --seed S    Reproducible fuzz
//   probe_diff_rp2350 --cycles             Also compare cycle counts

use mdpicoem_harness::*;
use probe_rs::probe::{DebugProbeSelector, list::Lister};
use probe_rs::{Core, MemoryInterface, Permissions, RegisterId, Session, SessionConfig};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// ARM Cortex-M register IDs (AADR numbering used by probe-rs).
const PC: RegisterId = RegisterId(15);
const XPSR: RegisterId = RegisterId(16);

// DWT / CoreDebug MMIO addresses.
const DEMCR: u64 = 0xE000_EDFC;
const DWT_CTRL: u64 = 0xE000_1000;
const DWT_CYCCNT: u64 = 0xE000_1004;

// DEMCR bits.
const TRCENA: u32 = 1 << 24;
// DWT_CTRL bits.
const CYCCNTENA: u32 = 1 << 0;

// NOP for cycle calibration.
const NOP: u16 = 0xBF00;
// BKPT #0 sentinel placed after test instruction.
const BKPT: u16 = 0xBE00;

fn main() {
    mdpicoem_harness::harness_tracing_init();
    if let Err(e) = run() {
        eprintln!("fatal: {e}");
        std::process::exit(2);
    }
}

// ============================================================================
// Argument parsing
// ============================================================================

struct Args {
    fuzz_count: Option<usize>,
    seed: Option<u64>,
    cycles: bool,
    probe: Option<DebugProbeSelector>,
}

use mdpicoem_harness::cli::parse_probe_selector;

fn parse_args_from<I: IntoIterator<Item = String>>(argv: I) -> Result<Args, String> {
    let args: Vec<String> = argv.into_iter().collect();
    let mut fuzz_count = None;
    let mut seed = None;
    let mut cycles = false;
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
            "--cycles" => {
                cycles = true;
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
                     probe_diff_rp2350                      Run targeted edge-case tests\n  \
                     probe_diff_rp2350 --fuzz N             Random fuzz tests (N per class)\n  \
                     probe_diff_rp2350 --fuzz N --seed S    Reproducible fuzz\n  \
                     probe_diff_rp2350 --cycles             Also compare cycle counts\n  \
                     probe_diff_rp2350 --probe VID:PID:SERIAL  Select a specific probe"
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
        cycles,
        probe,
    })
}

fn parse_args() -> Result<Args, String> {
    parse_args_from(std::env::args().skip(1))
}

// ============================================================================
// DWT helpers (copied from probe_verify_rp2350.rs)
// ============================================================================

/// Enable DWT CYCCNT: set TRCENA in DEMCR, then CYCCNTENA in DWT_CTRL.
fn enable_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    let demcr: u32 = core.read_word_32(DEMCR)?;
    core.write_word_32(DEMCR, demcr | TRCENA)?;
    let ctrl: u32 = core.read_word_32(DWT_CTRL)?;
    core.write_word_32(DWT_CTRL, ctrl | CYCCNTENA)?;
    Ok(())
}

/// Reset CYCCNT to zero.
fn reset_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    core.write_word_32(DWT_CYCCNT, 0)?;
    Ok(())
}

/// Read CYCCNT.
fn read_cyccnt(core: &mut Core) -> Result<u32, probe_rs::Error> {
    core.read_word_32(DWT_CYCCNT)
}

// ============================================================================
// Cycle calibration
// ============================================================================

/// Measure NOP cycle count 20 times and return the median as the baseline.
/// The baseline includes debug halt/resume overhead. Net instruction cost
/// is `measured - baseline + 1` (since NOP itself is 1 cycle).
fn calibrate_cycles(core: &mut Core) -> Result<u32, Box<dyn std::error::Error>> {
    let mut counts = Vec::with_capacity(20);
    for _ in 0..20 {
        // Write NOP at test slot
        core.write_8(EMU_TEST_SLOT as u64, &NOP.to_le_bytes())?;
        core.write_core_reg(PC, EMU_TEST_SLOT as u64)?;
        reset_cyccnt(core)?;
        core.step()?;
        counts.push(read_cyccnt(core)?);
    }

    let min = *counts.iter().min().unwrap();
    let max = *counts.iter().max().unwrap();
    if min != max {
        eprintln!("warning: NOP CYCCNT not consistent: min={min}, max={max}, counts={counts:?}");
        eprintln!("         Cycle comparisons may be unreliable in halt-step mode.");
    }

    // Median
    counts.sort();
    let median = counts[counts.len() / 2];
    Ok(median)
}

// ============================================================================
// Hardware-side execution
// ============================================================================

/// Execute a single test case on hardware via probe-rs single-step.
/// Returns post-execution state including CYCCNT.
fn run_one_probe(core: &mut Core, tc: &TestCase) -> Result<RunState, probe_rs::Error> {
    let is_fpu = is_fpu_test(tc);
    let n_steps: usize;

    // 1. Write instruction sequence + BKPT sentinel to test slot.
    if is_fpu {
        let (halfwords, n_insn) = build_fpu_test_sequence(tc);
        n_steps = n_insn;
        let mut code: Vec<u8> = Vec::new();
        for &hw in &halfwords {
            code.extend_from_slice(&hw.to_le_bytes());
        }
        code.extend_from_slice(&BKPT.to_le_bytes());
        core.write_8(EMU_TEST_SLOT as u64, &code)?;
    } else {
        // Standard path: opcode [+ hw1] [+ opcode2 [+ hw1_2]] BKPT
        let mut code = tc.opcode.to_le_bytes().to_vec();
        if let Some(hw1) = tc.hw1 {
            code.extend_from_slice(&hw1.to_le_bytes());
        }
        if let Some(op2) = tc.opcode2 {
            code.extend_from_slice(&op2.to_le_bytes());
            if let Some(hw1_2) = tc.hw1_2 {
                code.extend_from_slice(&hw1_2.to_le_bytes());
            }
        }
        code.extend_from_slice(&BKPT.to_le_bytes());
        core.write_8(EMU_TEST_SLOT as u64, &code)?;
        n_steps = if tc.opcode2.is_some() { 2 } else { 1 };
    }

    // 2. Set register defaults: R0-R12 = 0
    for i in 0..=12u16 {
        core.write_core_reg(RegisterId(i), 0u32)?;
    }
    // SP = test stack, LR = sentinel, PC = test slot, xPSR = precondition
    core.write_core_reg(RegisterId(13), EMU_TEST_STACK)?;
    core.write_core_reg(RegisterId(14), 0xFFFF_FFFFu32)?;
    core.write_core_reg(PC, EMU_TEST_SLOT)?;
    core.write_core_reg(XPSR, tc.xpsr_pre)?;

    // 3. Apply register preconditions (same address space — EMU addresses)
    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, EMU_TEST_SCRATCH);
        core.write_core_reg(RegisterId(reg as u16), val)?;
    }

    // 4. Memory setup (zero scratch + write preconditions)
    if tc.needs_bus {
        core.write_8(EMU_TEST_SCRATCH as u64, &[0u8; SCRATCH_SIZE as usize])?;
        for &(offset, val) in &tc.mem_pre {
            core.write_8((EMU_TEST_SCRATCH + offset) as u64, &[val])?;
        }
    }

    // 4b. FPU preconditions
    if is_fpu {
        core.write_core_reg(RegisterId(12), EMU_FPU_SCRATCH)?;
        // Always set R11 (even when fpscr_pre=0): the prelude always
        // executes VMSR FPSCR, R11 to clear sticky exception bits.
        core.write_core_reg(RegisterId(11), tc.fpscr_pre)?;
        // Clear FPU scratch (136 bytes)
        core.write_8(EMU_FPU_SCRATCH as u64, &[0u8; 136])?;
        for &(sn, bits) in &tc.fpu_pre {
            core.write_8(
                (EMU_FPU_SCRATCH + (sn as u32) * 4) as u64,
                &bits.to_le_bytes(),
            )?;
        }
    }

    // 5. Reset CYCCNT, single-step through all instructions.
    reset_cyccnt(core)?;
    for _ in 0..n_steps {
        core.step()?;
    }

    // 6. Read post-state
    let mut regs = [0u32; 16];
    for i in 0..16u32 {
        regs[i as usize] = core.read_core_reg(RegisterId(i as u16))?;
    }
    let xpsr: u32 = core.read_core_reg(XPSR)?;
    let cycles = read_cyccnt(core)?;

    // 7. Read memory at mem_check offsets
    let mut mem = Vec::new();
    for &offset in &tc.mem_check {
        let mut byte = [0u8; 1];
        core.read_8((EMU_TEST_SCRATCH + offset) as u64, &mut byte)?;
        mem.push(byte[0]);
    }

    // 8. Read FPU results from FPU scratch memory
    let mut fpu = Vec::new();
    let mut fpscr = 0u32;
    if is_fpu {
        for &sn in &tc.fpu_check {
            let mut bytes = [0u8; 4];
            core.read_8((EMU_FPU_SCRATCH + (sn as u32) * 4) as u64, &mut bytes)?;
            fpu.push(u32::from_le_bytes(bytes));
        }
        if tc.fpscr_mask != 0 {
            let mut bytes = [0u8; 4];
            core.read_8((EMU_FPU_SCRATCH + 128) as u64, &mut bytes)?;
            fpscr = u32::from_le_bytes(bytes);
        }
    }

    Ok(RunState {
        regs,
        xpsr,
        mem,
        cycles,
        fpu,
        fpscr,
    })
}

// ============================================================================
// Main runner
// ============================================================================

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;

    println!("probe_diff_rp2350: RP2354 hardware differential test runner");
    println!("===========================================================");

    // 1. Attach to target via probe-rs. With --probe, route through the
    // explicit selector to disambiguate multiple attached probes (see HLD
    // §2.1 — `auto_attach` just picks the first-enumerated probe).
    let mut session = match args.probe.as_ref() {
        None => Session::auto_attach("rp2350", SessionConfig::default())?,
        Some(selector) => {
            let probe = Lister::new().open(selector.clone())?;
            probe.attach("rp2350", Permissions::default())?
        }
    };
    let mut core = session.core(0)?;
    println!("Attached to target, using core 0");

    // 2. Reset and halt
    core.reset_and_halt(Duration::from_millis(500))?;

    // 3. Enable DWT cycle counter + FPU
    enable_cyccnt(&mut core)?;
    // Enable FPU: CPACR CP10/CP11 full access
    core.write_word_32(0xE000_ED88, 0x00F0_0000)?;

    // 4. Calibrate cycle counter
    let baseline = calibrate_cycles(&mut core)?;
    println!("DWT baseline: {baseline} cycles (NOP median, includes debug overhead)");

    // 5. Generate tests
    match args.fuzz_count {
        None => run_targeted(&mut core, &args, baseline),
        Some(count) => {
            let seed = args.seed.unwrap_or_else(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64
            });
            run_fuzz(&mut core, &args, baseline, count, seed)
        }
    }
}

/// Run the targeted edge-case test suite.
fn run_targeted(
    core: &mut Core,
    args: &Args,
    baseline: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let tests = generate_all();
    let total = tests.len();
    println!("Running {total} targeted tests...");

    let mut shared_bus = Bus::new();
    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;
    let mut cycle_mismatches = 0usize;
    let t0 = Instant::now();

    for (i, tc) in tests.iter().enumerate() {
        if (i + 1) % 100 == 0 {
            eprintln!("[{}/{}] {} failures so far...", i + 1, total, fail);
        }

        match run_one_diff(core, &mut shared_bus, tc, args, baseline) {
            Ok(cycle_ok) => {
                pass += 1;
                if !cycle_ok {
                    cycle_mismatches += 1;
                }
            }
            Err(DiffError::Mismatch(diff)) => {
                fail += 1;
                eprintln!(
                    "[FAIL] {}\n  opcode: {:#06x}  hw1: {:?}\n  xpsr_pre: {:#010x}\n  reg_pre: {:?}\n  diff: {}",
                    tc.name, tc.opcode, tc.hw1, tc.xpsr_pre, tc.reg_pre, diff
                );
            }
            Err(DiffError::ProbeError(e)) => {
                // counter directly drives rc=3 — do not increment from filter rejections; funnel those through a new variant or pre-loop drop.
                skip += 1;
                eprintln!("[SKIP] {}: probe-rs error: {e}", tc.name);
            }
        }
    }

    let elapsed = t0.elapsed();
    print_summary(
        "targeted",
        pass,
        fail,
        skip,
        cycle_mismatches,
        args.cycles,
        elapsed,
    );

    let rc = rc_for(pass, fail, skip);
    if rc == 3 {
        let attempted = pass + fail + skip;
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

/// Run fuzz tests with progress reporting.
fn run_fuzz(
    core: &mut Core,
    args: &Args,
    baseline: u32,
    count_per_class: usize,
    seed: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Fuzz mode: {count_per_class} tests/class, seed={seed}");
    println!("(reproduce with: probe_diff_rp2350 --fuzz {count_per_class} --seed {seed})");

    let (alu_tests, mem_tests) = generate_fuzz(count_per_class, seed);
    let total = alu_tests.len() + mem_tests.len();
    println!(
        "Generated {} tests ({} ALU + {} memory)",
        total,
        alu_tests.len(),
        mem_tests.len()
    );

    let mut shared_bus = Bus::new();
    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;
    let mut cycle_mismatches = 0usize;
    let mut done = 0usize;
    let t0 = Instant::now();

    for tc in alu_tests.iter().chain(mem_tests.iter()) {
        done += 1;
        if done % 100 == 0 {
            eprintln!("[{done}/{total}] {fail} failures...");
        }

        match run_one_diff(core, &mut shared_bus, tc, args, baseline) {
            Ok(cycle_ok) => {
                pass += 1;
                if !cycle_ok {
                    cycle_mismatches += 1;
                }
            }
            Err(DiffError::Mismatch(diff)) => {
                fail += 1;
                eprintln!(
                    "[FAIL] {}\n  opcode: {:#06x}  hw1: {:?}\n  xpsr_pre: {:#010x}\n  reg_pre: {:?}\n  diff: {}",
                    tc.name, tc.opcode, tc.hw1, tc.xpsr_pre, tc.reg_pre, diff
                );
            }
            Err(DiffError::ProbeError(e)) => {
                // counter directly drives rc=3 — do not increment from filter rejections; funnel those through a new variant or pre-loop drop.
                skip += 1;
                eprintln!("[SKIP] {}: probe-rs error: {e}", tc.name);
            }
        }
    }

    let elapsed = t0.elapsed();
    print_summary(
        "fuzz",
        pass,
        fail,
        skip,
        cycle_mismatches,
        args.cycles,
        elapsed,
    );
    if args.cycles || args.fuzz_count.is_some() {
        println!("Seed: {seed}");
        if fail > 0 {
            println!("Reproduce: probe_diff_rp2350 --fuzz {count_per_class} --seed {seed}");
        }
    }

    let rc = rc_for(pass, fail, skip);
    if rc == 3 {
        let attempted = pass + fail + skip;
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

// ============================================================================
// Per-test differential execution
// ============================================================================

enum DiffError {
    Mismatch(String),
    ProbeError(probe_rs::Error),
}

/// Run one test on both hardware and emulator, compare results.
/// Returns Ok(true) if semantic + cycle match, Ok(false) if semantic match
/// but cycle mismatch (only when --cycles is enabled).
fn run_one_diff(
    core: &mut Core,
    shared_bus: &mut Bus,
    tc: &TestCase,
    args: &Args,
    baseline: u32,
) -> Result<bool, DiffError> {
    // Hardware side
    let hw_state = run_one_probe(core, tc).map_err(DiffError::ProbeError)?;

    // Emulator side
    let emu_state = if is_fpu_test(tc) {
        run_one_emu_fpu(tc, shared_bus)
    } else if tc.opcode2.is_some() {
        run_one_emu_multistep(tc, shared_bus)
    } else {
        run_one_emu(tc, shared_bus)
    };

    // Semantic comparison
    compare_probe(tc, &hw_state, &emu_state).map_err(DiffError::Mismatch)?;

    // Cycle comparison (if enabled, and only for single-step tests —
    // multi-step and FPU tests intentionally return cycles=0 on the emulator side).
    let mut cycle_ok = true;
    if args.cycles && tc.opcode2.is_none() && !is_fpu_test(tc) {
        // Net cycles = measured - baseline + 1 (NOP is 1 cycle, baseline is NOP's raw count)
        let hw_cycles = hw_state.cycles.saturating_sub(baseline) + 1;
        let emu_cycles = emu_state.cycles;
        if hw_cycles != emu_cycles {
            cycle_ok = false;
            eprintln!(
                "[CYCLE] {}: HW={} (raw={}) EMU={}",
                tc.name, hw_cycles, hw_state.cycles, emu_cycles
            );
        }
    }

    Ok(cycle_ok)
}

// ============================================================================
// Summary
// ============================================================================

/// Map post-run counters to a process exit code.
///
/// Order of precedence: rc=1 (any failure) > rc=3 (degraded transport) > rc=0.
/// rc=3 fires only when at least 20 cases were attempted AND at least 25% of
/// them ended in `[SKIP]` (probe-rs transport errors). See HLD §3.
fn rc_for(pass: usize, fail: usize, skip: usize) -> i32 {
    if fail > 0 {
        return 1;
    }
    let attempted = pass + fail + skip;
    if attempted >= 20 && (skip * 100) / attempted >= 25 {
        return 3;
    }
    0
}

fn print_summary(
    mode: &str,
    pass: usize,
    fail: usize,
    skip: usize,
    cycle_mismatches: usize,
    cycles_enabled: bool,
    elapsed: Duration,
) {
    let total = pass + fail + skip;
    println!();
    println!("=== {mode} summary ===");
    println!("Total:   {total}");
    println!("Passed:  {pass}");
    println!("Failed:  {fail}");
    println!("Skipped: {skip}");
    if cycles_enabled {
        println!("Cycle mismatches: {cycle_mismatches}");
    }
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

    // -----------------------------------------------------------------
    // rc_for — degraded-mode exit-code mapping
    // -----------------------------------------------------------------

    #[test]
    fn rc_for_clean_run_returns_zero() {
        assert_eq!(rc_for(8000, 0, 0), 0);
    }

    #[test]
    fn rc_for_any_failures_returns_one() {
        // A single failure dominates a sea of passes.
        assert_eq!(rc_for(7999, 1, 0), 1);
        // Failure beats degraded — 6000 skips and 0 passes still rc=1 if any fail.
        assert_eq!(rc_for(0, 1, 6000), 1);
    }

    #[test]
    fn rc_for_high_skip_returns_three() {
        // 2026-04-25 08:51 incident: 1885 passed, 0 failed, 6115 skipped (~76%).
        assert_eq!(rc_for(1885, 0, 6115), 3);
    }

    #[test]
    fn rc_for_borderline_skip_below_threshold() {
        // Cascade-2 trigger batch: 7169 passed, 0 failed, 831 skipped (~10.4%) — below 25%.
        assert_eq!(rc_for(7169, 0, 831), 0);
    }

    #[test]
    fn rc_for_small_attempted_does_not_trip() {
        // Sanity floor: < 20 attempted cases never trips rc=3.
        assert_eq!(rc_for(0, 0, 5), 0);
    }

    #[test]
    fn rc_for_exactly_at_threshold() {
        // 75 + 0 + 25 = 100 attempted; 25/100 = 25% — boundary inclusive.
        assert_eq!(rc_for(75, 0, 25), 3);
    }
}

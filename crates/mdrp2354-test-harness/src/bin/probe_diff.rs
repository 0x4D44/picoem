// Hardware differential test runner: emulator vs real RP2354 silicon via SWD.
//
// Validates Thumb instruction semantics (and optionally cycle counts) by
// executing identical instructions on both the emulator and halted hardware
// (via probe-rs single-step), then comparing post-state.
//
// Usage:
//   probe_diff                      Run targeted edge-case tests
//   probe_diff --fuzz N             Random fuzz tests (N per class)
//   probe_diff --fuzz N --seed S    Reproducible fuzz
//   probe_diff --cycles             Also compare cycle counts

use mdrp2354_test_harness::*;
use probe_rs::{Core, MemoryInterface, RegisterId, Session, SessionConfig};
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
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut fuzz_count = None;
    let mut seed = None;
    let mut cycles = false;
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
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     probe_diff                      Run targeted edge-case tests\n  \
                     probe_diff --fuzz N             Random fuzz tests (N per class)\n  \
                     probe_diff --fuzz N --seed S    Reproducible fuzz\n  \
                     probe_diff --cycles             Also compare cycle counts"
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
    })
}

// ============================================================================
// DWT helpers (copied from probe_verify.rs)
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
        eprintln!(
            "warning: NOP CYCCNT not consistent: min={min}, max={max}, counts={counts:?}"
        );
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
fn run_one_probe(
    core: &mut Core,
    tc: &TestCase,
) -> Result<RunState, probe_rs::Error> {
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
            core.read_8(
                (EMU_FPU_SCRATCH + (sn as u32) * 4) as u64,
                &mut bytes,
            )?;
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

    println!("probe_diff: RP2354 hardware differential test runner");
    println!("====================================================");

    // 1. Attach to target via probe-rs
    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
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
                skip += 1;
                eprintln!("[SKIP] {}: probe-rs error: {e}", tc.name);
            }
        }
    }

    let elapsed = t0.elapsed();
    print_summary("targeted", pass, fail, skip, cycle_mismatches, args.cycles, elapsed);

    if fail > 0 {
        std::process::exit(1);
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
    println!("(reproduce with: probe_diff --fuzz {count_per_class} --seed {seed})");

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
                skip += 1;
                eprintln!("[SKIP] {}: probe-rs error: {e}", tc.name);
            }
        }
    }

    let elapsed = t0.elapsed();
    print_summary("fuzz", pass, fail, skip, cycle_mismatches, args.cycles, elapsed);
    if args.cycles || args.fuzz_count.is_some() {
        println!("Seed: {seed}");
        if fail > 0 {
            println!("Reproduce: probe_diff --fuzz {count_per_class} --seed {seed}");
        }
    }

    if fail > 0 {
        std::process::exit(1);
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

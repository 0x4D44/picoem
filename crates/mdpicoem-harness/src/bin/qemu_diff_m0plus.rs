// QEMU differential test runner — mdrp2040 (Cortex-M0+) oracle.
//
// Parallel to `qemu_diff_m33` but targets:
//   * QEMU machine: `microbit` (nRF51822 — the only ARMv6-M board QEMU
//     10.2 ships with a working GDB stub; mps2-an385 pins CPU to
//     cortex-m3, mps2-an505 pins CPU to cortex-m33).
//   * QEMU CPU: `cortex-m0` (QEMU does not expose a `cortex-m0plus`
//     model — the M0+ ISA is a strict superset of M0 for the
//     Thumb-16/Thumb-32 subset we differentially test, so M0 is a
//     safe reference).
//   * GDB port: 3334 (3333 is in use by qemu_diff_m33).
//   * Emulator: `mdrp2040::CortexM0Plus` + `mdrp2040::Bus`.
//
// Test-case filter: only Thumb-16 instructions that are valid on both
// M0+ and M33 are fuzzed. M33-only encodings (IT blocks, CBZ/CBNZ,
// Thumb-32 DP/ldm/stm/mul, FP) are filtered out. The fuzz generator
// lives in `mdpicoem_harness::lib`; we just select the subset.
//
// Usage (mirrors qemu_diff_m33):
//   qemu_diff_m0plus                              Run targeted edge-case tests
//   qemu_diff_m0plus --fuzz N                     Random fuzz, N tests per class
//   qemu_diff_m0plus --fuzz N --seed S            Reproducible fuzz

use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use mdpicoem_harness::gdb_client::{GdbClient, QemuProcess, QemuProfile};
use mdpicoem_harness::m0plus::{Bus as M0Bus, CortexM0Plus};
use mdpicoem_harness::{
    CompareBases, EMU_M0PLUS_TEST_SCRATCH, EMU_M0PLUS_TEST_SLOT, EMU_M0PLUS_TEST_STACK, FuzzClass,
    MASK_ALL_FLAGS, MASK_NZ_ONLY, QEMU_M0PLUS_TEST_SCRATCH, QEMU_M0PLUS_TEST_SLOT,
    QEMU_M0PLUS_TEST_STACK, QEMU_M0PLUS_VECTOR_TABLE_BASE, REG_LR, REG_PC, REG_SP, REG_XPSR,
    RunState, SCRATCH_SIZE, TestCase, compare, generate_all, generate_fuzz_classes,
    select_fuzz_class, setup_reg,
};

/// BKPT #0 instruction (little-endian bytes).
const BKPT_BYTES: [u8; 2] = [0x00, 0xBE];

/// Set by the Ctrl-C handler. See sibling comment in `qemu_diff_m33`.
static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn shutdown_requested() -> bool {
    SHUTDOWN.load(Ordering::SeqCst)
}

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

    if let Err(e) = ctrlc::set_handler(|| SHUTDOWN.store(true, Ordering::SeqCst)) {
        eprintln!("warning: failed to install Ctrl-C handler: {e}");
    }

    match run() {
        Ok(code) => code,
        Err(e) => {
            eprintln!("fatal: {e}");
            ExitCode::from(2)
        }
    }
}

// ============================================================================
// Argument parsing (simplified — M0+ has no FPU class)
// ============================================================================

struct Args {
    fuzz_count: Option<usize>,
    seed: Option<u64>,
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut fuzz_count = None;
    let mut seed = None;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--fuzz" => {
                i += 1;
                if i >= args.len() {
                    return Err("--fuzz requires a count".into());
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
                    return Err("--seed requires a value".into());
                }
                seed = Some(
                    args[i]
                        .parse::<u64>()
                        .map_err(|e| format!("invalid seed '{}': {e}", args[i]))?,
                );
            }
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     qemu_diff_m0plus                     Run targeted edge-case tests\n  \
                     qemu_diff_m0plus --fuzz N            Run N random tests per class\n  \
                     qemu_diff_m0plus --fuzz N --seed S   Reproducible fuzz run"
                ));
            }
        }
        i += 1;
    }
    if seed.is_some() && fuzz_count.is_none() {
        return Err("--seed requires --fuzz".into());
    }
    Ok(Args { fuzz_count, seed })
}

// ============================================================================
// Main runner
// ============================================================================

fn run() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let args = parse_args()?;

    let profile = QemuProfile::M0_PLUS_RP2040;
    eprintln!(
        "Spawning QEMU: machine={}, cpu={}, gdb=localhost:{}",
        profile.machine, profile.cpu, profile.gdb_port
    );

    // 1. Spawn QEMU (bind to `_qemu` so the child stays alive for the run — drop kills it).
    let _qemu = QemuProcess::spawn_with(profile)?;

    // 2. Connect GDB with retry
    let mut gdb = GdbClient::connect(&profile.gdb_addr(), Duration::from_secs(5))?;
    gdb.handshake()?;

    // 3. Write minimal vector table at 0x2000_0000 (SP + reset vector) so
    //    a stray reset wouldn't immediately HardFault. We set PC directly
    //    before each test so the vector table is belt-and-braces.
    setup_vector_table(&mut gdb)?;

    match args.fuzz_count {
        None => run_targeted(&mut gdb),
        Some(count) => {
            let seed = args.seed.unwrap_or_else(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64
            });
            run_fuzz(&mut gdb, count, seed)
        }
    }
}

fn run_targeted(gdb: &mut GdbClient) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let all = generate_all();
    let tests: Vec<TestCase> = all
        .into_iter()
        .filter(is_m0plus_safe)
        .filter(|tc| !tc.probe_only)
        .collect();
    eprintln!("Running {} M0+-compatible targeted tests", tests.len());

    let mut bus = M0Bus::new();
    let mut pass = 0usize;
    let mut fail = 0usize;
    for tc in &tests {
        if shutdown_requested() {
            eprintln!("interrupted (Ctrl-C); exiting cleanly");
            return Ok(ExitCode::from(130));
        }
        match run_one_test(gdb, &mut bus, tc) {
            Ok(()) => pass += 1,
            Err(d) => {
                fail += 1;
                eprintln!("[FAIL] {}: {}", tc.name, d);
            }
        }
    }
    println!("{pass}/{} passed", pass + fail);
    if fail > 0 {
        return Ok(ExitCode::from(1));
    }
    Ok(ExitCode::SUCCESS)
}

fn run_fuzz(
    gdb: &mut GdbClient,
    count_per_class: usize,
    seed: u64,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    println!("M0+ fuzz mode: {count_per_class} tests/class, seed={seed}");
    println!("(reproduce with: qemu_diff_m0plus --fuzz {count_per_class} --seed {seed})");

    // We only ever care about the Base class on M0+ — no FPU, no DSP.
    let buckets = select_fuzz_class(
        generate_fuzz_classes(count_per_class, seed),
        FuzzClass::Base,
    );
    let alu: Vec<TestCase> = buckets
        .base_alu
        .into_iter()
        .filter(is_m0plus_safe)
        .filter(|tc| !tc.probe_only)
        .collect();
    let mem: Vec<TestCase> = buckets
        .base_mem
        .into_iter()
        .filter(is_m0plus_safe)
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
    let mut done = 0usize;

    for bucket in [&alu, &mem] {
        for tc in bucket {
            done += 1;
            if done % 1000 == 0 {
                eprintln!("[{done}/{total}] {fail} failures...");
                if shutdown_requested() {
                    eprintln!("interrupted (Ctrl-C); exiting cleanly");
                    return Ok(ExitCode::from(130));
                }
            }
            if shutdown_requested() {
                eprintln!("interrupted (Ctrl-C); exiting cleanly");
                return Ok(ExitCode::from(130));
            }
            match run_one_test(gdb, &mut bus, tc) {
                Ok(()) => pass += 1,
                Err(d) => {
                    fail += 1;
                    eprintln!(
                        "[FAIL] {}\n  opcode: {:#06x}\n  xpsr_pre: {:#010x}\n  diff: {}",
                        tc.name, tc.opcode, tc.xpsr_pre, d
                    );
                }
            }
        }
    }

    println!();
    println!("=== M0+ fuzz summary ===");
    println!("Seed:   {seed}");
    println!("Total:  {total}");
    println!("Passed: {pass}");
    println!("Failed: {fail}");
    if fail > 0 {
        println!("\nReproduce: qemu_diff_m0plus --fuzz {count_per_class} --seed {seed}");
        return Ok(ExitCode::from(1));
    }
    Ok(ExitCode::SUCCESS)
}

// ============================================================================
// M0+ compatibility filter
// ============================================================================

/// Is this test case runnable on M0+?
///
/// Excludes:
///   * FPU tests (M0+ has no FPU).
///   * Thumb-32 tests except the 6 supported encodings (BL, MRS, MSR,
///     DSB, DMB, ISB). For simplicity we drop **all** Thumb-32 tests —
///     the 6 supported encodings are dominated by the BL branch target
///     which is address-sensitive and better covered by the targeted
///     suite anyway. The fuzz generators for Thumb-32 target M33-only
///     DP/ldm/stm/mul encodings.
///   * Multi-step tests (`opcode2.is_some()`) — these are IT-block
///     sequences which M0+ doesn't support.
///   * CBZ/CBNZ (`0xB1xx`, `0xB3xx`, `0xB9xx`, `0xBBxx`) — M33-only
///     conditional zero-compare branches.
///   * IT (`0xBFxx` with cond != 0 — but the whole range `0xBF00..=0xBFFF`
///     is safest to filter since NOP/YIELD/WFE/WFI/SEV share the prefix
///     and we don't need to fuzz hints).
///   * Saturation / Q-flag / GE-flag tests — these mask specific xPSR
///     bits M0+ doesn't implement. Detect via `xpsr_mask != MASK_ALL_FLAGS`
///     and `xpsr_mask != 0` — keep only the common-case flag masks.
fn is_m0plus_safe(tc: &TestCase) -> bool {
    if !tc.fpu_pre.is_empty() || !tc.fpu_check.is_empty() || tc.fpscr_mask != 0 {
        return false;
    }
    if tc.hw1.is_some() || tc.opcode2.is_some() || tc.hw1_2.is_some() {
        return false;
    }
    // 0xBFxx: IT + hints.
    if (tc.opcode & 0xFF00) == 0xBF00 {
        return false;
    }
    // 0xB1xx/0xB3xx/0xB9xx/0xBBxx: CBZ/CBNZ.
    if matches!(tc.opcode & 0xF500, 0xB100) {
        return false;
    }
    // Non-standard xPSR masks (Q / GE flags) imply M33-only instructions.
    // MASK_NO_FLAGS (0), MASK_ALL_FLAGS (NZCV+Q), and MASK_NZ_ONLY are safe.
    // Anything else (MASK_Q_ONLY, MASK_ALL_FLAGS_GE) is filtered.
    let m = tc.xpsr_mask;
    if m != 0 && m != MASK_ALL_FLAGS && m != MASK_NZ_ONLY {
        return false;
    }
    true
}

// ============================================================================
// Per-test execution
// ============================================================================

fn run_one_test(gdb: &mut GdbClient, bus: &mut M0Bus, tc: &TestCase) -> Result<(), String> {
    let qemu_state = run_qemu_side(gdb, tc).map_err(|e| format!("QEMU error: {e}"))?;
    let emu_state = run_emu_side(tc, bus);
    compare(tc, &qemu_state, &emu_state, &CompareBases::M0PLUS_RP2040)
}

fn setup_vector_table(gdb: &mut GdbClient) -> Result<(), Box<dyn std::error::Error>> {
    let sp_bytes = QEMU_M0PLUS_TEST_STACK.to_le_bytes();
    let reset_vector = (QEMU_M0PLUS_TEST_SLOT | 1).to_le_bytes();
    let mut table = [0u8; 8];
    table[0..4].copy_from_slice(&sp_bytes);
    table[4..8].copy_from_slice(&reset_vector);
    gdb.write_mem(QEMU_M0PLUS_VECTOR_TABLE_BASE, &table)?;
    Ok(())
}

/// Execute the test on QEMU and read back post-state.
fn run_qemu_side(gdb: &mut GdbClient, tc: &TestCase) -> std::io::Result<RunState> {
    // Write instruction + BKPT sentinel.
    gdb.write_mem(QEMU_M0PLUS_TEST_SLOT, &tc.opcode.to_le_bytes())?;
    gdb.write_mem(QEMU_M0PLUS_TEST_SLOT + 2, &BKPT_BYTES)?;

    // Register defaults.
    for i in 0..=12u8 {
        gdb.write_reg(i, 0)?;
    }
    gdb.write_reg(REG_SP, QEMU_M0PLUS_TEST_STACK)?;
    gdb.write_reg(REG_LR, 0xFFFF_FFFF)?;
    gdb.write_reg(REG_PC, QEMU_M0PLUS_TEST_SLOT)?;
    gdb.write_reg(REG_XPSR, tc.xpsr_pre)?;

    // Register preconditions (with address translation).
    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, QEMU_M0PLUS_TEST_SCRATCH);
        gdb.write_reg(reg, val)?;
    }

    // Memory preconditions.
    if tc.needs_bus {
        gdb.write_mem(QEMU_M0PLUS_TEST_SCRATCH, &[0u8; SCRATCH_SIZE as usize])?;
        for &(off, v) in &tc.mem_pre {
            gdb.write_mem(QEMU_M0PLUS_TEST_SCRATCH + off, &[v])?;
        }
    }

    // Single step.
    gdb.step()?;

    // Read post-state.
    let mut regs = [0u32; 16];
    for i in 0..16u8 {
        regs[i as usize] = gdb.read_reg(i)?;
    }
    let xpsr = gdb.read_reg(REG_XPSR)?;

    let mem: Vec<u8> = tc
        .mem_check
        .iter()
        .map(|&off| {
            gdb.read_mem(QEMU_M0PLUS_TEST_SCRATCH + off, 1)
                .map(|b| b[0])
        })
        .collect::<std::io::Result<Vec<u8>>>()?;

    Ok(RunState {
        regs,
        xpsr,
        mem,
        cycles: 0,
        fpu: Vec::new(),
        fpscr: 0,
    })
}

/// Execute the test on our mdrp2040 emulator and read back post-state.
///
/// Parallels `mdpicoem_harness::run_one_emu` but uses the M0+ core and
/// the M0+ SRAM layout (test slot in SRAM at 0x2000_0100, matching QEMU).
fn run_emu_side(tc: &TestCase, bus: &mut M0Bus) -> RunState {
    let mut core = CortexM0Plus::new();

    // Defaults.
    for i in 0..=12 {
        core.set_reg(i, 0);
    }
    core.set_reg(13, EMU_M0PLUS_TEST_STACK);
    core.set_reg(14, 0xFFFF_FFFF);
    core.regs.set_pc(EMU_M0PLUS_TEST_SLOT);
    core.regs.xpsr = tc.xpsr_pre;

    // Register preconditions.
    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, EMU_M0PLUS_TEST_SCRATCH);
        core.set_reg(reg as usize, val);
    }

    // Memory preconditions.
    if tc.needs_bus {
        for i in 0..SCRATCH_SIZE {
            bus.write8(EMU_M0PLUS_TEST_SCRATCH + i, 0);
        }
        for &(off, v) in &tc.mem_pre {
            bus.write8(EMU_M0PLUS_TEST_SCRATCH + off, v);
        }
    }

    // Execute.
    let _cycles = if tc.needs_bus {
        core.execute_one_with_bus(tc.opcode, bus)
    } else {
        core.execute_one(tc.opcode)
    };

    // Collect post-state.
    let mut regs = [0u32; 16];
    for i in 0..16 {
        regs[i] = core.reg(i);
    }
    let xpsr = core.regs.xpsr;
    let mem: Vec<u8> = tc
        .mem_check
        .iter()
        .map(|&off| bus.read8(EMU_M0PLUS_TEST_SCRATCH + off))
        .collect();

    RunState {
        regs,
        xpsr,
        mem,
        cycles: 0,
        fpu: Vec::new(),
        fpscr: 0,
    }
}

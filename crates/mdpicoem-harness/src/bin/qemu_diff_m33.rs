// QEMU differential test runner — mdrp2350 (Cortex-M33) oracle.
//
// Orchestrates: spawn QEMU (mps2-an505 + cortex-m33), connect GDB on
// localhost:3333, generate tests, run each test in both QEMU and our
// mdrp2350 emulator, compare results, report.
//
// The parallel RP2040 runner is `qemu_diff_m0plus` (microbit + cortex-m0
// on port 3334). See the workspace restructure HLD Phase 6 section.
//
// Usage:
//   qemu_diff_m33                              Run targeted edge-case tests (default)
//   qemu_diff_m33 --fuzz N                     Run N random tests per instruction class
//   qemu_diff_m33 --fuzz N --seed S            Reproducible fuzz run with seed S
//   qemu_diff_m33 --fuzz N --classes=base|fpu|all
//                                              Restrict fuzz to base (non-FPU) or FPU
//                                              instructions. Defaults to `all`.
//                                              Per HLD §11, only base and FPU classes
//                                              are QEMU-oracled; CP0/CP4/CP5/CP7 are
//                                              validated via softfloat_diff/unit tests.

use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use mdpicoem_harness::gdb_client::{sanity_check, GdbClient, QemuProcess};
use mdpicoem_harness::*;

/// BKPT #0 instruction (little-endian bytes).
const BKPT_BYTES: [u8; 2] = [0x00, 0xBE];

/// Vector table base address (secure alias of ssram-0).
const VECTOR_TABLE_BASE: u32 = 0x1000_0000;

/// Set by the Ctrl-C handler. Polled at fuzz-loop checkpoints; never
/// inspected from the handler itself (handlers must not call
/// `process::exit` — that would skip `QemuProcess`'s `Drop` and re-open
/// the very leak this HLD closes).
static SHUTDOWN: AtomicBool = AtomicBool::new(false);

/// Returns true once the user has requested shutdown via Ctrl-C.
fn shutdown_requested() -> bool {
    SHUTDOWN.load(Ordering::SeqCst)
}

fn main() -> ExitCode {
    // Best-effort Ctrl-C handler: flips a flag the main loop polls. If the
    // OS rejects the install (rare; unsupported platform, already-installed
    // handler), keep going — the cooperative `Drop` and the Job Object will
    // still keep QEMU off the GDB port on normal exit.
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
// Argument parsing
// ============================================================================

struct Args {
    fuzz_count: Option<usize>,
    seed: Option<u64>,
    class: FuzzClass,
}

fn parse_class(s: &str) -> Result<FuzzClass, String> {
    match s {
        "base" => Ok(FuzzClass::Base),
        "fpu" => Ok(FuzzClass::Fpu),
        "all" => Ok(FuzzClass::All),
        other => Err(format!(
            "invalid --classes value '{other}' (expected base|fpu|all)"
        )),
    }
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut fuzz_count = None;
    let mut seed = None;
    let mut class = FuzzClass::All;
    let mut i = 0;

    while i < args.len() {
        let a = args[i].as_str();
        // Accept both `--classes=X` and `--classes X` forms.
        if let Some(val) = a.strip_prefix("--classes=") {
            class = parse_class(val)?;
            i += 1;
            continue;
        }
        match a {
            "--fuzz" => {
                i += 1;
                if i >= args.len() {
                    return Err("--fuzz requires a count argument".into());
                }
                fuzz_count = Some(args[i].parse::<usize>().map_err(|e| {
                    format!("invalid fuzz count '{}': {e}", args[i])
                })?);
            }
            "--seed" => {
                i += 1;
                if i >= args.len() {
                    return Err("--seed requires a value argument".into());
                }
                seed = Some(args[i].parse::<u64>().map_err(|e| {
                    format!("invalid seed '{}': {e}", args[i])
                })?);
            }
            "--classes" => {
                i += 1;
                if i >= args.len() {
                    return Err("--classes requires base|fpu|all".into());
                }
                class = parse_class(&args[i])?;
            }
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     qemu_diff_m33                              Run targeted edge-case tests (default)\n  \
                     qemu_diff_m33 --fuzz N                     Run N random tests per class\n  \
                     qemu_diff_m33 --fuzz N --seed S            Reproducible fuzz run\n  \
                     qemu_diff_m33 --fuzz N --classes=base|fpu|all   Restrict fuzz to class"
                ));
            }
        }
        i += 1;
    }

    if seed.is_some() && fuzz_count.is_none() {
        return Err("--seed requires --fuzz".into());
    }
    if class != FuzzClass::All && fuzz_count.is_none() {
        return Err("--classes requires --fuzz".into());
    }

    Ok(Args { fuzz_count, seed, class })
}

// ============================================================================
// Main runner
// ============================================================================

fn run() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let args = parse_args()?;

    // 1. Spawn QEMU
    let mut qemu = QemuProcess::spawn()?;

    // 2. Connect GDB with retry
    let mut gdb = GdbClient::connect("localhost:3333", Duration::from_secs(5))?;
    gdb.handshake()?;

    // 3. Write minimal vector table so QEMU isn't stuck in HardFault
    setup_vector_table(&mut gdb)?;

    // 4. Sanity-check register round-trips (must follow vector table setup)
    sanity_check(&mut gdb)?;

    match args.fuzz_count {
        None => run_targeted(&mut gdb, &mut qemu),
        Some(count) => {
            let seed = args.seed.unwrap_or_else(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64
            });
            run_fuzz(&mut gdb, &mut qemu, count, seed, args.class)
        }
    }
}

/// Run the targeted edge-case test suite (original behavior).
fn run_targeted(
    gdb: &mut GdbClient,
    qemu: &mut QemuProcess,
) -> Result<(), Box<dyn std::error::Error>> {
    let all_tests = generate_all();
    let total_before = all_tests.len();
    let tests: Vec<TestCase> = all_tests.into_iter().filter(|tc| !tc.probe_only).collect();
    let filtered = total_before - tests.len();
    if filtered > 0 {
        println!("Filtered {filtered} probe-only tests (run via probe_diff)");
    }
    let mut shared_bus = Bus::new();
    let mut pass = 0usize;
    let mut fail = 0usize;

    for tc in &tests {
        match run_with_recovery(gdb, qemu, &mut shared_bus, tc) {
            Ok(()) => pass += 1,
            Err(diff) => {
                fail += 1;
                eprintln!("[FAIL] {}: {}", tc.name, diff);
            }
        }
    }

    println!("{pass}/{} passed", pass + fail);
    if fail > 0 {
        std::process::exit(1);
    }
    Ok(())
}

/// Run fuzz tests with progress reporting and recovery.
fn run_fuzz(
    gdb: &mut GdbClient,
    qemu: &mut QemuProcess,
    count_per_class: usize,
    seed: u64,
    class: FuzzClass,
) -> Result<(), Box<dyn std::error::Error>> {
    let class_str = match class {
        FuzzClass::All => "all",
        FuzzClass::Base => "base",
        FuzzClass::Fpu => "fpu",
    };
    println!("Fuzz mode: {count_per_class} tests/class, seed={seed}, classes={class_str}");
    println!(
        "(reproduce with: qemu_diff_m33 --fuzz {count_per_class} --seed {seed} --classes={class_str})"
    );

    let buckets = select_fuzz_class(generate_fuzz_classes(count_per_class, seed), class);
    let raw_total = buckets.base_alu.len() + buckets.base_mem.len() + buckets.fpu.len();
    let alu_tests: Vec<TestCase> =
        buckets.base_alu.into_iter().filter(|tc| !tc.probe_only).collect();
    let mem_tests: Vec<TestCase> =
        buckets.base_mem.into_iter().filter(|tc| !tc.probe_only).collect();
    let fpu_tests: Vec<TestCase> =
        buckets.fpu.into_iter().filter(|tc| !tc.probe_only).collect();
    let total = alu_tests.len() + mem_tests.len() + fpu_tests.len();
    let filtered = raw_total - total;
    if filtered > 0 {
        println!("Filtered {filtered} probe-only tests (run via probe_diff)");
    }
    println!(
        "Generated {} tests ({} ALU + {} memory + {} FPU)",
        total,
        alu_tests.len(),
        mem_tests.len(),
        fpu_tests.len()
    );

    let mut shared_bus = Bus::new();
    let mut pass = 0usize;
    let mut fail = 0usize;
    let mut skip = 0usize;
    let mut done = 0usize;

    // Order: base ALU (fastest) -> FPU (no memory setup) -> base memory (slowest).
    let buckets: [(&str, &[TestCase]); 3] = [
        ("ALU", &alu_tests),
        ("FPU", &fpu_tests),
        ("MEM", &mem_tests),
    ];

    for (_label, bucket) in &buckets {
        for tc in *bucket {
            done += 1;
            if done % 1000 == 0 {
                eprintln!("[{done}/{total}] {fail} failures...");
            }
            match run_with_recovery(gdb, qemu, &mut shared_bus, tc) {
                Ok(()) => pass += 1,
                Err(diff) if diff.starts_with("SKIPPED") => {
                    skip += 1;
                    eprintln!("[SKIP] {}: {}", tc.name, diff);
                }
                Err(diff) => {
                    fail += 1;
                    eprintln!(
                        "[FAIL] {}\n  opcode: {:#06x}  hw1: {:?}\n  xpsr_pre: {:#010x}\n  reg_pre: {:?}\n  diff: {}",
                        tc.name, tc.opcode, tc.hw1, tc.xpsr_pre, tc.reg_pre, diff
                    );
                }
            }
        }
    }

    // Summary
    println!();
    println!("=== Fuzz summary ===");
    println!("Seed:    {seed}");
    println!("Classes: {class_str}");
    println!("Total:   {total}");
    println!("Passed:  {pass}");
    println!("Failed:  {fail}");
    println!("Skipped: {skip}");

    if fail > 0 {
        println!(
            "\nReproduce: qemu_diff_m33 --fuzz {count_per_class} --seed {seed} --classes={class_str}"
        );
        std::process::exit(1);
    }
    Ok(())
}

// ============================================================================
// Test execution
// ============================================================================

/// Write minimal vector table to 0x10000000.
///
/// Word 0: initial SP (QEMU_TEST_STACK)
/// Word 1: reset vector (QEMU_TEST_SLOT | 1, with Thumb bit)
fn setup_vector_table(gdb: &mut GdbClient) -> Result<(), Box<dyn std::error::Error>> {
    let sp_bytes = QEMU_TEST_STACK.to_le_bytes();
    let reset_vector = (QEMU_TEST_SLOT | 1).to_le_bytes();
    let mut table = [0u8; 8];
    table[0..4].copy_from_slice(&sp_bytes);
    table[4..8].copy_from_slice(&reset_vector);
    gdb.write_mem(VECTOR_TABLE_BASE, &table)?;
    // Enable FPU: CPACR CP10/CP11 full access
    gdb.write_mem(0xE000_ED88, &0x00F0_0000u32.to_le_bytes())?;
    Ok(())
}

/// Run a single differential test: set up both sides, execute, compare.
fn run_one_test(
    gdb: &mut GdbClient,
    shared_bus: &mut Bus,
    tc: &TestCase,
) -> Result<(), String> {
    let qemu_state = run_qemu_side(gdb, tc).map_err(|e| format!("QEMU error: {e}"))?;
    let emu_state = if is_fpu_test(tc) {
        run_one_emu_fpu(tc, shared_bus)
    } else if tc.opcode2.is_some() {
        run_one_emu_multistep(tc, shared_bus)
    } else {
        run_one_emu(tc, shared_bus)
    };
    compare(tc, &qemu_state, &emu_state, &CompareBases::M33_RP2350)
}

/// Run a test with GDB error recovery. If GDB fails, respawn QEMU and reconnect.
fn run_with_recovery(
    gdb: &mut GdbClient,
    qemu: &mut QemuProcess,
    bus: &mut Bus,
    tc: &TestCase,
) -> Result<(), String> {
    match run_one_test(gdb, bus, tc) {
        Ok(()) => Ok(()),
        Err(e) if is_gdb_error(&e) => {
            eprintln!(
                "[RECOVER] GDB error on {}: {}, respawning QEMU...",
                tc.name, e
            );
            // Kill old QEMU (drop does this) and spawn fresh
            match respawn_qemu(qemu, gdb) {
                Ok(()) => Err(format!("SKIPPED (recovery): {e}")),
                Err(re) => Err(format!("RECOVERY FAILED: {re} (original: {e})")),
            }
        }
        Err(e) => Err(e),
    }
}

/// Check if an error string indicates a GDB/IO problem rather than a test diff.
fn is_gdb_error(e: &str) -> bool {
    e.starts_with("QEMU error:")
}

/// Kill the old QEMU process, spawn a new one, and reconnect GDB.
fn respawn_qemu(
    qemu: &mut QemuProcess,
    gdb: &mut GdbClient,
) -> Result<(), String> {
    // Drop old QEMU (kill on drop), spawn new
    *qemu = QemuProcess::spawn().map_err(|e| e.to_string())?;
    std::thread::sleep(Duration::from_millis(500));
    *gdb = GdbClient::connect("localhost:3333", Duration::from_secs(5))
        .map_err(|e| e.to_string())?;
    gdb.handshake().map_err(|e| e.to_string())?;
    setup_vector_table(gdb).map_err(|e| e.to_string())?;
    Ok(())
}

/// Execute the test on QEMU via GDB and read back post-state.
fn run_qemu_side(
    gdb: &mut GdbClient,
    tc: &TestCase,
) -> std::io::Result<RunState> {
    let is_fpu = is_fpu_test(tc);

    // Determine the number of single-steps and write instruction sequence.
    let n_steps: usize;

    if is_fpu {
        // FPU test: build the full prelude/test/epilogue sequence.
        let (halfwords, n_insn) = build_fpu_test_sequence(tc);
        n_steps = n_insn;
        let mut addr = QEMU_TEST_SLOT;
        for &hw in &halfwords {
            gdb.write_mem(addr, &hw.to_le_bytes())?;
            addr += 2;
        }
        gdb.write_mem(addr, &BKPT_BYTES)?;
    } else {
        // Standard (non-FPU) path: write opcode [+ hw1] [+ opcode2 [+ hw1_2]] + BKPT.
        gdb.write_mem(QEMU_TEST_SLOT, &tc.opcode.to_le_bytes())?;
        let mut next: u32 = QEMU_TEST_SLOT + 2;
        if let Some(hw1) = tc.hw1 {
            gdb.write_mem(next, &hw1.to_le_bytes())?;
            next += 2;
        }
        if let Some(op2) = tc.opcode2 {
            gdb.write_mem(next, &op2.to_le_bytes())?;
            next += 2;
            if let Some(hw1_2) = tc.hw1_2 {
                gdb.write_mem(next, &hw1_2.to_le_bytes())?;
                next += 2;
            }
        }
        gdb.write_mem(next, &BKPT_BYTES)?;
        n_steps = if tc.opcode2.is_some() { 2 } else { 1 };
    }

    // Set register defaults
    for i in 0..=12u8 {
        gdb.write_reg(i, 0)?;
    }
    gdb.write_reg(REG_SP, QEMU_TEST_STACK)?;
    gdb.write_reg(REG_LR, 0xFFFF_FFFF)?;
    gdb.write_reg(REG_PC, QEMU_TEST_SLOT)?;
    gdb.write_reg(REG_XPSR, tc.xpsr_pre)?;

    // Apply register preconditions with address translation
    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, QEMU_TEST_SCRATCH);
        gdb.write_reg(reg, val)?;
    }

    // Zero scratch + write memory preconditions
    if tc.needs_bus {
        gdb.write_mem(QEMU_TEST_SCRATCH, &[0u8; SCRATCH_SIZE as usize])?;
        for &(offset, val) in &tc.mem_pre {
            gdb.write_mem(QEMU_TEST_SCRATCH + offset, &[val])?;
        }
    }

    // FPU preconditions: set R12 = QEMU_FPU_SCRATCH, R11 = fpscr_pre,
    // write fpu_pre bit patterns to QEMU_FPU_SCRATCH memory.
    if is_fpu {
        gdb.write_reg(12, QEMU_FPU_SCRATCH)?;
        // Always set R11 (even when fpscr_pre=0): the prelude always
        // executes VMSR FPSCR, R11 to clear sticky exception bits.
        gdb.write_reg(11, tc.fpscr_pre)?;
        // Clear FPU scratch (136 bytes: 32 S-regs * 4 + FPSCR at offset 128)
        gdb.write_mem(QEMU_FPU_SCRATCH, &[0u8; 136])?;
        for &(sn, bits) in &tc.fpu_pre {
            gdb.write_mem(QEMU_FPU_SCRATCH + (sn as u32) * 4, &bits.to_le_bytes())?;
        }
    }

    // Step through the instruction sequence
    for _ in 0..n_steps {
        gdb.step()?;
    }

    // Read post-state
    let mut regs = [0u32; 16];
    for i in 0..16u8 {
        regs[i as usize] = gdb.read_reg(i)?;
    }
    let xpsr = gdb.read_reg(REG_XPSR)?;

    // Read memory at mem_check offsets
    let mem: Vec<u8> = tc
        .mem_check
        .iter()
        .map(|&offset| {
            gdb.read_mem(QEMU_TEST_SCRATCH + offset, 1)
                .map(|bytes| bytes[0])
        })
        .collect::<std::io::Result<Vec<u8>>>()?;

    // Read FPU results from QEMU_FPU_SCRATCH
    let mut fpu = Vec::new();
    let mut fpscr = 0u32;
    if is_fpu {
        for &sn in &tc.fpu_check {
            let bytes = gdb.read_mem(QEMU_FPU_SCRATCH + (sn as u32) * 4, 4)?;
            fpu.push(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]));
        }
        if tc.fpscr_mask != 0 {
            let bytes = gdb.read_mem(QEMU_FPU_SCRATCH + 128, 4)?;
            fpscr = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        }
    }

    Ok(RunState { regs, xpsr, mem, cycles: 0, fpu, fpscr })
}

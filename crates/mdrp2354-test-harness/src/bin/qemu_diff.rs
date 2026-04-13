// QEMU differential test runner.
//
// Orchestrates: spawn QEMU, connect GDB, generate tests, run each test
// in both QEMU and our emulator, compare results, report.
//
// Usage:
//   qemu_diff                    Run targeted edge-case tests (default)
//   qemu_diff --fuzz N           Run N random tests per instruction class
//   qemu_diff --fuzz N --seed S  Reproducible fuzz run with seed S

use std::time::Duration;

use mdrp2354_test_harness::gdb_client::{sanity_check, GdbClient, QemuProcess};
use mdrp2354_test_harness::*;

/// BKPT #0 instruction (little-endian bytes).
const BKPT_BYTES: [u8; 2] = [0x00, 0xBE];

/// Vector table base address (secure alias of ssram-0).
const VECTOR_TABLE_BASE: u32 = 0x1000_0000;

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
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     qemu_diff                    Run targeted edge-case tests (default)\n  \
                     qemu_diff --fuzz N           Run N random tests per instruction class\n  \
                     qemu_diff --fuzz N --seed S  Reproducible fuzz run with seed S"
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

fn run() -> Result<(), Box<dyn std::error::Error>> {
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
            run_fuzz(&mut gdb, &mut qemu, count, seed)
        }
    }
}

/// Run the targeted edge-case test suite (original behavior).
fn run_targeted(
    gdb: &mut GdbClient,
    qemu: &mut QemuProcess,
) -> Result<(), Box<dyn std::error::Error>> {
    let tests = generate_all();
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
) -> Result<(), Box<dyn std::error::Error>> {
    println!("Fuzz mode: {count_per_class} tests/class, seed={seed}");
    println!("(reproduce with: qemu_diff --fuzz {count_per_class} --seed {seed})");

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
    let mut done = 0usize;

    // Run ALU tests first (fast, no memory setup)
    for tc in &alu_tests {
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

    // Run memory tests (slower)
    for tc in &mem_tests {
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

    // Summary
    println!();
    println!("=== Fuzz summary ===");
    println!("Seed:    {seed}");
    println!("Total:   {total}");
    println!("Passed:  {pass}");
    println!("Failed:  {fail}");
    println!("Skipped: {skip}");

    if fail > 0 {
        println!("\nReproduce: qemu_diff --fuzz {count_per_class} --seed {seed}");
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
    Ok(())
}

/// Run a single differential test: set up both sides, execute, compare.
fn run_one_test(
    gdb: &mut GdbClient,
    shared_bus: &mut Bus,
    tc: &TestCase,
) -> Result<(), String> {
    let qemu_state = run_qemu_side(gdb, tc).map_err(|e| format!("QEMU error: {e}"))?;
    let emu_state = run_one_emu(tc, shared_bus);
    compare(tc, &qemu_state, &emu_state)
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
    // Write instruction to test slot, then BKPT sentinel after it.
    gdb.write_mem(QEMU_TEST_SLOT, &tc.opcode.to_le_bytes())?;
    match tc.hw1 {
        None => {
            gdb.write_mem(QEMU_TEST_SLOT + 2, &BKPT_BYTES)?;
        }
        Some(hw1) => {
            gdb.write_mem(QEMU_TEST_SLOT + 2, &hw1.to_le_bytes())?;
            gdb.write_mem(QEMU_TEST_SLOT + 4, &BKPT_BYTES)?;
        }
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

    // Single-step
    gdb.step()?;

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

    Ok(RunState { regs, xpsr, mem, cycles: 0 })
}

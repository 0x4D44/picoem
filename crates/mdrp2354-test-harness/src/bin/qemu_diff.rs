// QEMU differential test runner.
//
// Orchestrates: spawn QEMU, connect GDB, generate tests, run each test
// in both QEMU and our emulator, compare results, report.

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

fn run() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Spawn QEMU
    let _qemu = QemuProcess::spawn()?;

    // 2. Connect GDB with retry
    let mut gdb = GdbClient::connect("localhost:3333", Duration::from_secs(5))?;
    gdb.handshake()?;
    sanity_check(&mut gdb)?;

    // 3. Write minimal vector table to secure alias
    setup_vector_table(&mut gdb)?;

    // 4. Generate tests
    let tests = generate_all();
    let mut shared_bus = Bus::new(); // reused across bus-tests
    let mut pass = 0usize;
    let mut fail = 0usize;

    // 5. Run each test
    for tc in &tests {
        match run_one_test(&mut gdb, &mut shared_bus, tc) {
            Ok(()) => pass += 1,
            Err(diff) => {
                fail += 1;
                eprintln!("[FAIL] {}: {}", tc.name, diff);
            }
        }
    }

    // 6. Report
    println!("{pass}/{} passed", pass + fail);
    if fail > 0 {
        std::process::exit(1);
    }
    Ok(())
}

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
    // Map io::Error to String for the comparison result type
    let qemu_state = run_qemu_side(gdb, tc).map_err(|e| format!("QEMU error: {e}"))?;
    let emu_state = run_one_emu(tc, shared_bus);
    compare(tc, &qemu_state, &emu_state)
}

/// Execute the test on QEMU via GDB and read back post-state.
fn run_qemu_side(
    gdb: &mut GdbClient,
    tc: &TestCase,
) -> std::io::Result<RunState> {
    // Write instruction + BKPT to test slot
    let instr_bytes = tc.opcode.to_le_bytes();
    gdb.write_mem(QEMU_TEST_SLOT, &instr_bytes)?;
    gdb.write_mem(QEMU_TEST_SLOT + 2, &BKPT_BYTES)?;

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
        gdb.write_mem(QEMU_TEST_SCRATCH, &[0u8; 256])?;
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

    Ok(RunState { regs, xpsr, mem })
}

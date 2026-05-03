// Hardware verification program for probe-rs + RP2354 (OneROM) via Pico H debug probe.
//
// Validates assumptions about register access, SRAM execution, DWT CYCCNT,
// xPSR behaviour, and cycle-count consistency — all via SWD single-stepping.
//
// Run: cargo run -p picoem-harness --bin probe_verify_rp2350

use probe_rs::config::MemoryRegion;
use probe_rs::{Core, MemoryInterface, RegisterId, Session, SessionConfig};
use std::time::Instant;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// SRAM slot we use for injected instructions.
const TEST_SLOT: u64 = 0x2000_0100;

// ARM Cortex-M register IDs (AADR numbering used by probe-rs).
const R0: RegisterId = RegisterId(0);
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

// Thumb-2 encoded instructions (16-bit, little-endian halfwords).
const NOP: u32 = 0xBF00;
const MOVS_R0_42: u32 = 0x202A; // MOVS R0, #42
const ADDS_R0_R0_1: u32 = 0x3001; // ADDS R0, R0, #1
const LDR_R0_R1: u32 = 0x6808; // LDR R0, [R1, #0] — 2-cycle load
const MUL_R0_R1: u32 = 0x4348; // MULS R0, R1, R0 — 1-cycle on M33

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct Results {
    pass: u32,
    fail: u32,
}

impl Results {
    fn new() -> Self {
        Self { pass: 0, fail: 0 }
    }
    fn check(&mut self, name: &str, ok: bool, detail: &str) {
        if ok {
            self.pass += 1;
            println!("  [PASS] {name}: {detail}");
        } else {
            self.fail += 1;
            println!("  [FAIL] {name}: {detail}");
        }
    }
}

/// Write a 16-bit Thumb instruction at `addr` in SRAM.
fn write_thumb(core: &mut Core, addr: u64, hw: u32) -> Result<(), probe_rs::Error> {
    let bytes = (hw as u16).to_le_bytes();
    core.write_8(addr, &bytes)?;
    Ok(())
}

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

/// Measure cycles for a single-stepped instruction. Assumes core is halted,
/// DWT is enabled. Writes instruction at TEST_SLOT, sets PC, resets CYCCNT,
/// steps, reads CYCCNT.
fn measure_insn(core: &mut Core, insn: u32) -> Result<u32, probe_rs::Error> {
    write_thumb(core, TEST_SLOT, insn)?;
    core.write_core_reg(PC, TEST_SLOT)?;
    reset_cyccnt(core)?;
    core.step()?;
    read_cyccnt(core)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn test1_register_roundtrip(core: &mut Core, res: &mut Results) -> Result<(), probe_rs::Error> {
    println!("\n=== Test 1: Halt, write regs, step, read regs ===");

    core.reset_and_halt(std::time::Duration::from_millis(500))?;

    // Write R0 = 0xDEADBEEF
    core.write_core_reg(R0, 0xDEAD_BEEFu64)?;

    // Write NOP at TEST_SLOT
    write_thumb(core, TEST_SLOT, NOP)?;

    // Set PC to TEST_SLOT
    core.write_core_reg(PC, TEST_SLOT)?;

    // Confirm writes took effect before stepping
    let r0_pre: u64 = core.read_core_reg(R0)?;
    let pc_pre: u64 = core.read_core_reg(PC)?;
    println!("  Before step: R0=0x{r0_pre:08X}, PC=0x{pc_pre:08X}");

    res.check(
        "R0 write",
        r0_pre == 0xDEAD_BEEF,
        &format!("expected 0xDEADBEEF, got 0x{r0_pre:08X}"),
    );
    res.check(
        "PC write",
        pc_pre == TEST_SLOT,
        &format!("expected 0x{TEST_SLOT:08X}, got 0x{pc_pre:08X}"),
    );

    // Single-step
    core.step()?;

    let r0_post: u64 = core.read_core_reg(R0)?;
    let pc_post: u64 = core.read_core_reg(PC)?;
    println!("  After step:  R0=0x{r0_post:08X}, PC=0x{pc_post:08X}");

    res.check(
        "R0 preserved after NOP",
        r0_post == 0xDEAD_BEEF,
        &format!("expected 0xDEADBEEF, got 0x{r0_post:08X}"),
    );
    res.check(
        "PC advanced by 2",
        pc_post == TEST_SLOT + 2,
        &format!("expected 0x{:08X}, got 0x{pc_post:08X}", TEST_SLOT + 2),
    );

    Ok(())
}

fn test2_sram_executable(core: &mut Core, res: &mut Results) -> Result<(), probe_rs::Error> {
    println!("\n=== Test 2: SRAM executable after reset_and_halt ===");

    core.reset_and_halt(std::time::Duration::from_millis(500))?;

    // Write MOVS R0, #42 at TEST_SLOT
    write_thumb(core, TEST_SLOT, MOVS_R0_42)?;

    // Set R0 to something else first so we can confirm it changes
    core.write_core_reg(R0, 0u64)?;
    core.write_core_reg(PC, TEST_SLOT)?;

    core.step()?;

    let r0: u64 = core.read_core_reg(R0)?;
    println!("  R0 after MOVS R0, #42: {r0} (0x{r0:08X})");

    res.check(
        "SRAM executable",
        r0 == 42,
        &format!("expected 42, got {r0}"),
    );

    Ok(())
}

fn test3_dwt_cyccnt(core: &mut Core, res: &mut Results) -> Result<(), probe_rs::Error> {
    println!("\n=== Test 3: DWT CYCCNT during single-step ===");

    core.reset_and_halt(std::time::Duration::from_millis(500))?;
    enable_cyccnt(core)?;

    // Verify CYCCNT resets to 0
    reset_cyccnt(core)?;
    let before: u32 = read_cyccnt(core)?;
    println!("  CYCCNT after reset: {before}");
    res.check(
        "CYCCNT resets to 0",
        before == 0,
        &format!("expected 0, got {before}"),
    );

    // Write NOP, set PC, step
    write_thumb(core, TEST_SLOT, NOP)?;
    core.write_core_reg(PC, TEST_SLOT)?;
    reset_cyccnt(core)?;
    core.step()?;

    let after: u32 = read_cyccnt(core)?;
    println!("  CYCCNT after stepping NOP: {after}");
    res.check(
        "CYCCNT counts during step",
        after > 0,
        &format!("expected >0, got {after}"),
    );

    // Also read DWT_CTRL to see what's enabled
    let dwt_ctrl: u32 = core.read_word_32(DWT_CTRL)?;
    println!("  DWT_CTRL = 0x{dwt_ctrl:08X}");
    println!(
        "    CYCCNTENA={}, NOCYCCNT={}, NOPRFCNT={}, NUMCOMP={}",
        dwt_ctrl & 1,
        (dwt_ctrl >> 25) & 1,
        (dwt_ctrl >> 24) & 1,
        (dwt_ctrl >> 28) & 0xF
    );

    Ok(())
}

fn test4_xpsr(core: &mut Core, res: &mut Results) -> Result<(), probe_rs::Error> {
    println!("\n=== Test 4: xPSR inspection and manipulation ===");

    core.reset_and_halt(std::time::Duration::from_millis(500))?;

    let xpsr: u64 = core.read_core_reg(XPSR)?;
    println!("  xPSR after reset_and_halt: 0x{xpsr:08X}");

    let t_bit = (xpsr >> 24) & 1;
    println!("  T bit (Thumb state): {t_bit}");
    res.check("T bit set", t_bit == 1, &format!("T bit = {t_bit}"));

    // Decode flags
    println!(
        "  Flags: N={} Z={} C={} V={}",
        (xpsr >> 31) & 1,
        (xpsr >> 30) & 1,
        (xpsr >> 29) & 1,
        (xpsr >> 28) & 1,
    );
    println!("  Exception number: {}", xpsr & 0x1FF);

    // Try writing xPSR with N+Z set (keep T bit!)
    let new_xpsr: u64 = 0xC100_0000; // N=1, Z=1, T=1
    core.write_core_reg(XPSR, new_xpsr)?;
    let readback: u64 = core.read_core_reg(XPSR)?;
    println!("  Wrote xPSR = 0x{new_xpsr:08X}, read back = 0x{readback:08X}");

    let n_set = (readback >> 31) & 1 == 1;
    let z_set = (readback >> 30) & 1 == 1;
    let t_still = (readback >> 24) & 1 == 1;
    res.check(
        "N flag writeable",
        n_set,
        &format!("N={}", (readback >> 31) & 1),
    );
    res.check(
        "Z flag writeable",
        z_set,
        &format!("Z={}", (readback >> 30) & 1),
    );
    res.check(
        "T bit preserved",
        t_still,
        &format!("T={}", (readback >> 24) & 1),
    );

    Ok(())
}

fn test5_cyccnt_calibration(core: &mut Core, res: &mut Results) -> Result<(), probe_rs::Error> {
    println!("\n=== Test 5: CYCCNT calibration (20 measurements each) ===");

    core.reset_and_halt(std::time::Duration::from_millis(500))?;
    enable_cyccnt(core)?;

    // Measure NOP 20 times
    let mut nop_counts = Vec::with_capacity(20);
    for _ in 0..20 {
        let c = measure_insn(core, NOP)?;
        nop_counts.push(c);
    }
    println!("  NOP cycles:  {:?}", nop_counts);

    let nop_min = *nop_counts.iter().min().unwrap();
    let nop_max = *nop_counts.iter().max().unwrap();
    let nop_consistent = nop_min == nop_max;
    res.check(
        "NOP consistent",
        nop_consistent,
        &format!("min={nop_min}, max={nop_max}"),
    );

    // Measure ADDS R0, R0, #1 twenty times
    let mut adds_counts = Vec::with_capacity(20);
    for _ in 0..20 {
        let c = measure_insn(core, ADDS_R0_R0_1)?;
        adds_counts.push(c);
    }
    println!("  ADDS cycles: {:?}", adds_counts);

    let adds_min = *adds_counts.iter().min().unwrap();
    let adds_max = *adds_counts.iter().max().unwrap();
    let adds_consistent = adds_min == adds_max;
    res.check(
        "ADDS consistent",
        adds_consistent,
        &format!("min={adds_min}, max={adds_max}"),
    );

    // Compare
    if nop_consistent && adds_consistent {
        println!(
            "  NOP={nop_min} cy, ADDS={adds_min} cy, delta={}",
            adds_min as i32 - nop_min as i32
        );
    }

    // Measure LDR R0, [R1, #0] — needs R1 pointing to valid SRAM
    core.write_core_reg(RegisterId(1), 0x2000_0200u64)?; // R1 = scratch area
    core.write_word_32(0x2000_0200, 0x1234_5678)?; // put data there
    let mut ldr_counts = Vec::with_capacity(20);
    for _ in 0..20 {
        let c = measure_insn(core, LDR_R0_R1)?;
        ldr_counts.push(c);
    }
    println!("  LDR  cycles: {:?}", ldr_counts);

    // Measure MUL R0, R1
    let mut mul_counts = Vec::with_capacity(20);
    for _ in 0..20 {
        let c = measure_insn(core, MUL_R0_R1)?;
        mul_counts.push(c);
    }
    println!("  MUL  cycles: {:?}", mul_counts);

    let ldr_min = *ldr_counts.iter().min().unwrap();
    let mul_min = *mul_counts.iter().min().unwrap();
    println!("  Summary: NOP={nop_min} ADDS={adds_min} LDR={ldr_min} MUL={mul_min}");
    println!(
        "  Deltas from NOP: ADDS={} LDR={} MUL={}",
        adds_min as i32 - nop_min as i32,
        ldr_min as i32 - nop_min as i32,
        mul_min as i32 - nop_min as i32,
    );

    Ok(())
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<(), Box<dyn std::error::Error>> {
    picoem_harness::harness_tracing_init();
    println!("probe_verify_rp2350: RP2354 hardware assumption checker");
    println!("=======================================================");

    // Attach to the RP2350 (covers both RP2350 and RP2354 variants).
    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;

    println!("Attached to target. Available memory regions:");
    for region in session.target().memory_map.iter() {
        match region {
            MemoryRegion::Ram(r) => {
                println!("  RAM:  0x{:08X}..0x{:08X}", r.range.start, r.range.end);
            }
            MemoryRegion::Nvm(r) => {
                println!("  NVM:  0x{:08X}..0x{:08X}", r.range.start, r.range.end);
            }
            MemoryRegion::Generic(r) => {
                println!("  Generic: 0x{:08X}..0x{:08X}", r.range.start, r.range.end);
            }
        }
    }

    let mut core = session.core(0)?;
    println!("Using core 0");

    let mut res = Results::new();

    let t0 = Instant::now();

    test1_register_roundtrip(&mut core, &mut res)?;
    test2_sram_executable(&mut core, &mut res)?;
    test3_dwt_cyccnt(&mut core, &mut res)?;
    test4_xpsr(&mut core, &mut res)?;
    test5_cyccnt_calibration(&mut core, &mut res)?;

    let elapsed = t0.elapsed();

    println!("\n================================================");
    println!(
        "Results: {} passed, {} failed (in {:.1}s)",
        res.pass,
        res.fail,
        elapsed.as_secs_f64()
    );

    if res.fail > 0 {
        std::process::exit(1);
    }

    Ok(())
}

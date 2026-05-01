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
// Test-case filter: Thumb-16 instructions plus the M0+ Thumb-32 subset
// (`BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`) are exercised. M33-only
// encodings (IT blocks, CBZ/CBNZ, M33-only Thumb-32 DP/ldm/stm/multiply,
// FP, BASEPRI/FAULTMASK SYSm, banked `_NS` aliases) are filtered out.
// The fuzz generator lives in `mdpicoem_harness::lib`; we just select
// the subset. The doc-comment on `is_m0plus_safe` is authoritative.
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
    MASK_ALL_FLAGS, MASK_NZ_ONLY, MASK_NZCV_ONLY, QEMU_M0PLUS_PRIMER_SLOT,
    QEMU_M0PLUS_TEST_SCRATCH, QEMU_M0PLUS_TEST_SLOT, QEMU_M0PLUS_TEST_STACK,
    QEMU_M0PLUS_VECTOR_TABLE_BASE, REG_LR, REG_PC, REG_SP, REG_XPSR, RunState, SCRATCH_SIZE,
    TestCase, compare, generate_all, generate_fuzz_classes, select_fuzz_class, setup_reg,
};

/// BKPT #0 instruction (little-endian bytes).
const BKPT_BYTES: [u8; 2] = [0x00, 0xBE];

/// `MSR PRIMASK, R0` — Thumb-32, 4 bytes (hw0=0xF380, hw1=0x8810). Written
/// once to `QEMU_M0PLUS_PRIMER_SLOT` at startup and stepped through before
/// every test (with R0=0 from the per-test register default) to clear
/// PRIMASK on QEMU. See the `QEMU_M0PLUS_PRIMER_SLOT` doc-comment for the
/// rationale.
const PRIMER_BYTES: [u8; 4] = [0x80, 0xF3, 0x10, 0x88];

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

    // Sanity gate (HLD §5.5): emit admit-count of the curated `generate_all`
    // corpus through the relaxed filter before any QEMU work. If a future
    // encoder change silently drops every Thumb-32 admit, `run_targeted`
    // could still report all-passes; this line makes that loud.
    let all = generate_all();
    let admitted: Vec<&TestCase> = all.iter().filter(|tc| is_m0plus_safe(tc)).collect();
    let total = admitted.len();
    let t32 = admitted.iter().filter(|tc| tc.hw1.is_some()).count();
    let t16 = total - t32;
    println!("Test corpus: {total} total, {t16} Thumb-16, {t32} Thumb-32 (after filter).");

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
            if done.is_multiple_of(1000) {
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

/// Is this test case runnable on QEMU `cortex-m0`?
///
/// Admits Thumb-16 instructions common to M0+ and M33 **plus** the M0+
/// Thumb-32 subset: `BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`. All six are
/// plain ARMv6-M and execute on QEMU `cortex-m0` even though QEMU has no
/// `cortex-m0plus` model.
///
/// Rejects:
///   * FPU tests (M0+ has no FPU).
///   * Multi-step / IT-block tests (`opcode2.is_some()`) — M0+ does not
///     implement IT. Also rejects raw IT opcodes (`0xBFxx` with cond).
///   * CBZ / CBNZ (`0xB1xx` / `0xB3xx` / `0xB9xx` / `0xBBxx`) — M33-only
///     conditional zero-compare branches.
///   * Non-standard xPSR masks (Q-flag-only / GE-flag families) — M0+
///     doesn't implement those flags. Admitted: no-flags, NZ-only,
///     NZCV-only, and the full NZCVQ legacy mask.
///   * MSR / MRS with sysm ∈ {17 (BASEPRI), 19 (FAULTMASK)} — M33-only
///     special registers. Also rejects any `sysm >= 0x80` banked `_NS`
///     aliases (TrustZone-only on M33; M0+ UNDEFs them).
///   * Any **other** Thumb-32 encoding — the M0+ ISA's 32-bit subset is
///     exactly the six encodings above, so we key the admit list off
///     concrete hw0/hw1 patterns and reject everything else.
///
/// Intentionally duplicates `is_m0plus_silicon_safe` in `probe_diff_rp2040`
/// rather than sharing code: the two filters happen to agree today, but
/// the soft constraints might drift (e.g. a future QEMU regression on a
/// specific SYSm could force the QEMU-side filter to narrow further). The
/// shared *unit tests* are the consistency oracle.
fn is_m0plus_safe(tc: &TestCase) -> bool {
    // FPU tests: M0+ has no FPU.
    if !tc.fpu_pre.is_empty() || !tc.fpu_check.is_empty() || tc.fpscr_mask != 0 {
        return false;
    }

    // Multi-step / IT-body tests: M0+ has no IT blocks.
    if tc.opcode2.is_some() || tc.hw1_2.is_some() {
        return false;
    }

    // Raw IT / hint prefix (0xBFxx): IT itself is M33-only; NOP / YIELD /
    // WFE / WFI / SEV are architecturally supported on M0+ but we don't
    // need to fuzz hints, so filter the whole range.
    if (tc.opcode & 0xFF00) == 0xBF00 {
        return false;
    }

    // CBZ / CBNZ (0xB1xx / 0xB3xx / 0xB9xx / 0xBBxx).
    if matches!(tc.opcode & 0xF500, 0xB100) {
        return false;
    }

    // M33-only xPSR flag families (Q-flag alone, GE flags). M0+ accepts
    // no-flags, NZ-only, NZCV-only (the architectural ARMv6-M APSR width,
    // used by MSR APSR fuzz cases — Q is ARMv7-M-only), and full NZCVQ
    // (legacy width, M0+ just leaves Q clear).
    let m = tc.xpsr_mask;
    if m != 0 && m != MASK_ALL_FLAGS && m != MASK_NZ_ONLY && m != MASK_NZCV_ONLY {
        return false;
    }

    // Thumb-32 admit list. `opcode` is the first halfword, `hw1` is the
    // second. A Thumb-32 test case always has `hw1 = Some(_)`.
    if let Some(hw1) = tc.hw1 {
        let hw0 = tc.opcode;

        // BL (T1): hw0[15:11] = 0b11110, hw1[15:14] = 0b11, hw1[12] = 1.
        //   pattern: hw0 & 0xF800 == 0xF000, hw1 & 0xD000 == 0xD000.
        let is_bl = (hw0 & 0xF800) == 0xF000 && (hw1 & 0xD000) == 0xD000;

        // MSR (T1): hw0 = 0xF380 | Rn (i.e. hw0 & 0xFFF0 == 0xF380),
        //           hw1 high byte = 0x88 | mask (mask occupies bits 11:10).
        //   Pattern: hw0 & 0xFFF0 == 0xF380, hw1 & 0xFF00 == 0x8800
        //   with hw1[7:0] = SYSm.
        //
        // Pattern admits mask = 0b10 only because `enc_t32_msr` at
        // `thumb32_gen.rs:723` hardcodes mask = 0b10 (NZCVQ). Extend the
        // pattern if the generator ever emits other mask values.
        let is_msr = (hw0 & 0xFFF0) == 0xF380 && (hw1 & 0xFF00) == 0x8800;

        // MRS (T1): hw0 = 0xF3EF (Rn forced to 0b1111 per spec; R bit 0),
        //           hw1 = 0x8000 | (Rd << 8) | SYSm (top nybble 0b1000).
        //   Pattern: hw0 == 0xF3EF, hw1 & 0xF0FF <= full range; the
        //   generator's Rd is in [0, 15] so hw1[11:8] is free. Require
        //   the fixed bits: hw1 & 0xF000 == 0x8000.
        let is_mrs = hw0 == 0xF3EF && (hw1 & 0xF000) == 0x8000;

        // Barriers (DSB / DMB / ISB): hw0 = 0xF3BF, hw1 = 0x8Fxy where
        // y is the option field (typically 0xF = SY) and x is the op
        // (4=DSB, 5=DMB, 6=ISB). Accept any option/op in that space —
        // M0+ implements these as ordering-only NOPs.
        let is_barrier = hw0 == 0xF3BF && (hw1 & 0xFF00) == 0x8F00;

        // For MSR / MRS, additionally gate sysm:
        //   sysm == 17 → BASEPRI   — M33-only, reject.
        //   sysm == 19 → FAULTMASK — M33-only, reject.
        //   sysm >= 0x80          — banked _NS aliases, M33 TrustZone
        //                           only, reject.
        if is_msr || is_mrs {
            let sysm = hw1 & 0xFF;
            if sysm == 17 || sysm == 19 || sysm >= 0x80 {
                return false;
            }
        }

        if !(is_bl || is_msr || is_mrs || is_barrier) {
            return false;
        }
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
    gdb.write_mem(QEMU_M0PLUS_PRIMER_SLOT, &PRIMER_BYTES)?;
    Ok(())
}

/// Execute the test on QEMU and read back post-state.
fn run_qemu_side(gdb: &mut GdbClient, tc: &TestCase) -> std::io::Result<RunState> {
    // Write instruction (16 or 32 bits) + BKPT sentinel as one image.
    let mut code = tc.opcode.to_le_bytes().to_vec();
    if let Some(hw1) = tc.hw1 {
        code.extend_from_slice(&hw1.to_le_bytes());
    }
    code.extend_from_slice(&BKPT_BYTES);
    gdb.write_mem(QEMU_M0PLUS_TEST_SLOT, &code)?;

    // Register defaults. R0=0 must be set before the primer step so that
    // `MSR PRIMASK, R0` clears PRIMASK to 0. `reg_pre` overrides (which may
    // set R0 to non-zero) are applied AFTER the primer.
    for i in 0..=12u8 {
        gdb.write_reg(i, 0)?;
    }
    gdb.write_reg(REG_SP, QEMU_M0PLUS_TEST_STACK)?;
    gdb.write_reg(REG_LR, 0xFFFF_FFFF)?;
    gdb.write_reg(REG_PC, QEMU_M0PLUS_PRIMER_SLOT)?;
    gdb.write_reg(REG_XPSR, tc.xpsr_pre)?;

    // PRIMASK reset primer: step `MSR PRIMASK, R0` (R0=0 ⇒ PRIMASK=0).
    // Aligns QEMU's persistent CPU state with the EMU's fresh-CPU-per-test
    // model. xPSR flag bits are unchanged by `MSR PRIMASK,Rn` per ARMv6-M
    // B5.2.3, so xpsr_pre survives the primer.
    gdb.step()?;

    // Register preconditions (with address translation). Applied after the
    // primer so that an `R0` override can't accidentally set PRIMASK=1.
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

    // Re-aim PC at the test slot (primer left it at PRIMER_SLOT+4) and step
    // the actual test instruction.
    gdb.write_reg(REG_PC, QEMU_M0PLUS_TEST_SLOT)?;
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

    // Execute. Dispatches to the wide executor when the test case
    // carries a `hw1` (Thumb-32 subset).
    let _cycles = match tc.hw1 {
        None => {
            if tc.needs_bus {
                core.execute_one_with_bus(tc.opcode, bus)
            } else {
                core.execute_one(tc.opcode)
            }
        }
        Some(hw1) => {
            if tc.needs_bus {
                core.execute_one_wide_with_bus(tc.opcode, hw1, bus)
            } else {
                core.execute_one_wide(tc.opcode, hw1)
            }
        }
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

// ============================================================================
// Filter-level self-tests (no QEMU required)
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use mdpicoem_harness::thumb32_gen::{enc_t32_mrs, enc_t32_msr};

    fn msr_case(sysm: u16) -> TestCase {
        let (hw0, hw1) = enc_t32_msr(0, sysm);
        TestCase {
            name: format!("MSR sysm={sysm}"),
            opcode: hw0,
            hw1: Some(hw1),
            ..TestCase::default()
        }
    }

    fn mrs_case(sysm: u16) -> TestCase {
        let (hw0, hw1) = enc_t32_mrs(0, sysm);
        TestCase {
            name: format!("MRS sysm={sysm}"),
            opcode: hw0,
            hw1: Some(hw1),
            ..TestCase::default()
        }
    }

    #[test]
    fn filter_admits_msr_primask_control() {
        assert!(is_m0plus_safe(&msr_case(16)), "PRIMASK must be allowed");
        assert!(is_m0plus_safe(&msr_case(20)), "CONTROL must be allowed");
        assert!(is_m0plus_safe(&mrs_case(16)), "MRS PRIMASK must be allowed");
        assert!(is_m0plus_safe(&mrs_case(20)), "MRS CONTROL must be allowed");
    }

    #[test]
    fn filter_rejects_basepri_faultmask() {
        assert!(!is_m0plus_safe(&msr_case(17)), "BASEPRI must be rejected");
        assert!(!is_m0plus_safe(&msr_case(19)), "FAULTMASK must be rejected");
        assert!(
            !is_m0plus_safe(&mrs_case(17)),
            "MRS BASEPRI must be rejected"
        );
        assert!(
            !is_m0plus_safe(&mrs_case(19)),
            "MRS FAULTMASK must be rejected"
        );
    }

    #[test]
    fn filter_rejects_banked_ns_aliases() {
        // sysm >= 0x80 are banked _NS aliases (M33 TrustZone only).
        assert!(
            !is_m0plus_safe(&msr_case(0x90)),
            "banked MSR must be rejected"
        );
        assert!(
            !is_m0plus_safe(&mrs_case(0x94)),
            "banked MRS must be rejected"
        );
    }

    #[test]
    fn filter_admits_barriers() {
        // DMB / DSB / ISB — all three share hw0 = 0xF3BF, hw1[15:8] = 0x8F.
        let dmb = TestCase {
            opcode: 0xF3BF,
            hw1: Some(0x8F5F),
            ..TestCase::default()
        };
        let dsb = TestCase {
            opcode: 0xF3BF,
            hw1: Some(0x8F4F),
            ..TestCase::default()
        };
        let isb = TestCase {
            opcode: 0xF3BF,
            hw1: Some(0x8F6F),
            ..TestCase::default()
        };
        assert!(is_m0plus_safe(&dmb));
        assert!(is_m0plus_safe(&dsb));
        assert!(is_m0plus_safe(&isb));
    }

    #[test]
    fn filter_admits_bl() {
        // BL to a small positive offset — hw0 & 0xF800 == 0xF000,
        // hw1 & 0xD000 == 0xD000.
        let (hw0, hw1) = mdpicoem_harness::thumb32_gen::enc_t32_bl(4);
        let tc = TestCase {
            opcode: hw0,
            hw1: Some(hw1),
            ..TestCase::default()
        };
        assert!(is_m0plus_safe(&tc), "BL must be allowed");
    }

    #[test]
    fn filter_rejects_other_thumb32() {
        // A random non-subset Thumb-32 — e.g. TBB (hw0 = 0xE8DF, hw1 = 0xF000).
        let tc = TestCase {
            opcode: 0xE8DF,
            hw1: Some(0xF000),
            ..TestCase::default()
        };
        assert!(!is_m0plus_safe(&tc), "TBB must be rejected");

        // LDRD literal — another M33-only Thumb-32 encoding.
        let tc = TestCase {
            opcode: 0xE95F,
            hw1: Some(0x0100),
            ..TestCase::default()
        };
        assert!(!is_m0plus_safe(&tc), "LDRD literal must be rejected");
    }

    #[test]
    fn filter_rejects_it_and_cbz() {
        // IT EQ — 0xBF08.
        let it = TestCase {
            opcode: 0xBF08,
            ..TestCase::default()
        };
        assert!(!is_m0plus_safe(&it), "IT must be rejected");

        // CBZ R0, <label> — 0xB100 | ...
        let cbz = TestCase {
            opcode: 0xB108,
            ..TestCase::default()
        };
        assert!(!is_m0plus_safe(&cbz), "CBZ must be rejected");

        // CBNZ — 0xB9xx.
        let cbnz = TestCase {
            opcode: 0xB920,
            ..TestCase::default()
        };
        assert!(!is_m0plus_safe(&cbnz), "CBNZ must be rejected");
    }

    #[test]
    fn filter_rejects_fpu_and_multistep() {
        // FPU test (non-empty fpu_pre).
        let fpu = TestCase {
            opcode: 0x0000,
            fpu_pre: vec![(0, 0x3F80_0000)],
            ..TestCase::default()
        };
        assert!(!is_m0plus_safe(&fpu), "FPU test must be rejected");

        // Multi-step IT body.
        let multi = TestCase {
            opcode: 0xBF08,
            opcode2: Some(0x0000),
            ..TestCase::default()
        };
        assert!(!is_m0plus_safe(&multi), "multi-step must be rejected");
    }

    #[test]
    fn filter_admits_common_thumb16_alu() {
        // MOVS R0, #42 — 0x202A.
        let movs = TestCase {
            opcode: 0x202A,
            ..TestCase::default()
        };
        assert!(is_m0plus_safe(&movs));

        // ADDS R0, R1, R2 — 0x1888.
        let adds = TestCase {
            opcode: 0x1888,
            ..TestCase::default()
        };
        assert!(is_m0plus_safe(&adds));
    }

    /// Stage E.2 regression: `MASK_NZCV_ONLY` (0xF000_0000) is the architectural
    /// ARMv6-M APSR width and is the mask used by `fuzz_m0plus_msr` for MSR
    /// APSR (sysm=0) cases. Pre-fix the filter rejected it as a "non-standard
    /// xPSR flag family", silently dropping every APSR-write fuzz case.
    #[test]
    fn filter_admits_mask_nzcv_only() {
        // ANDS r1, r0 — Thumb-16 ALU, satisfies all non-mask gates.
        let case = TestCase {
            opcode: 0x4001,
            xpsr_mask: MASK_NZCV_ONLY,
            ..TestCase::default()
        };
        assert!(is_m0plus_safe(&case));
    }
}

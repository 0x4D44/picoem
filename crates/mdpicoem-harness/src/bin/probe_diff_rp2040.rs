// Hardware differential test runner — mdrp2040 (Cortex-M0+) vs real RP2040
// silicon via SWD.
//
// Ports `probe_diff_rp2350.rs` with three RP2040-specific adjustments:
//
//   * probe-rs target: `"rp2040"` (not `"rp2350"`).
//   * Emulator: `mdrp2040::{Bus, CortexM0Plus}` via the harness's `m0plus`
//     namespace.
//   * Test-case filter: a fresh `is_m0plus_silicon_safe()` that whitelists
//     the M0+ Thumb-32 subset (`BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`) but
//     rejects M33-only MSR/MRS `sysm` values — `BASEPRI` (17), `FAULTMASK`
//     (19), and any banked `_NS` aliases (sysm >= 0x80). On M0+ those
//     encodings UNDEF / fault to HardFault, so running them on silicon
//     would trash the differential comparison.
//
// The sibling QEMU runner (`qemu_diff_m0plus`) keeps its stricter filter
// that rejects **all** Thumb-32: QEMU's `cortex-m0` model has no Thumb-32
// support, whereas real M0+ silicon implements the subset listed above.
//
// Cycle comparison: RP2040 does not fit DWT in its Cortex-M0+ (no
// CYCCNT / no DWT_CTRL), so the `--cycles` flag present on the M33 runner
// is intentionally omitted here. Emulator-side cycle counts are collected
// but not compared.
//
// Usage (mirrors `probe_diff_rp2350` minus `--cycles`):
//   probe_diff_rp2040                      Run targeted edge-case tests
//   probe_diff_rp2040 --fuzz N             Random fuzz tests (N per class)
//   probe_diff_rp2040 --fuzz N --seed S    Reproducible fuzz

use mdpicoem_harness::m0plus::{Bus as M0Bus, CortexM0Plus};
use mdpicoem_harness::{
    compare_probe, generate_all, generate_fuzz, setup_reg, RunState, TestCase,
    EMU_M0PLUS_TEST_SCRATCH, EMU_M0PLUS_TEST_SLOT, EMU_M0PLUS_TEST_STACK,
    MASK_ALL_FLAGS, MASK_NZ_ONLY, SCRATCH_SIZE,
};
use probe_rs::probe::{list::Lister, DebugProbeSelector};
use probe_rs::{Core, MemoryInterface, Permissions, RegisterId, Session, SessionConfig};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// ARM Cortex-M register IDs (AADR numbering used by probe-rs).
const PC: RegisterId = RegisterId(15);
const XPSR: RegisterId = RegisterId(16);

// BKPT #0 sentinel placed after test instruction.
const BKPT: u16 = 0xBE00;

// RP2040 bootrom occupies 0x0000_0000..=0x0000_3FFF (16 KB). The
// UNDEF-on-silicon classifier treats any post-step PC landing in this
// range as evidence that silicon HardFaulted and was dispatched via
// VTOR=0 into the bootrom's fault handler.
const RP2040_BOOTROM_END: u32 = 0x0000_4000;

fn main() {
    mdpicoem_harness::harness_tracing_init();
    if let Err(e) = run() {
        eprintln!("fatal: {e}");
        std::process::exit(2);
    }
}

// ============================================================================
// Argument parsing (simplified vs M33 — no --cycles, no FPU class)
// ============================================================================

struct Args {
    fuzz_count: Option<usize>,
    seed: Option<u64>,
    probe: Option<DebugProbeSelector>,
}

fn parse_probe_selector(s: &str) -> Result<DebugProbeSelector, String> {
    DebugProbeSelector::try_from(s)
        .map_err(|e| format!("invalid probe selector '{s}': {e}"))
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

    Ok(Args { fuzz_count, seed, probe })
}

fn parse_args() -> Result<Args, String> {
    parse_args_from(std::env::args().skip(1))
}

// ============================================================================
// M0+ silicon compatibility filter
// ============================================================================

/// Is this test case runnable on real RP2040 / Cortex-M0+ silicon?
///
/// Admits Thumb-16 instructions common to M0+ and M33 **plus** the M0+
/// Thumb-32 subset: `BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`.
///
/// Rejects:
///   * FPU tests (M0+ has no FPU).
///   * Multi-step / IT-block tests (`opcode2.is_some()`) — M0+ does not
///     implement IT. Also rejects raw IT opcodes (`0xBFxx` with cond).
///   * CBZ / CBNZ (`0xB1xx` / `0xB3xx` / `0xB9xx` / `0xBBxx`) — M33-only
///     conditional zero-compare branches.
///   * Non-standard xPSR masks (Q-flag / GE-flag families) — M0+ doesn't
///     implement those flags.
///   * MSR / MRS with sysm ∈ {17 (BASEPRI), 19 (FAULTMASK)} — M33-only
///     special registers. Also rejects any `sysm >= 0x80` banked `_NS`
///     aliases (TrustZone-only on M33; M0+ UNDEFs them).
///   * Any **other** Thumb-32 encoding — the M0+ ISA's 32-bit subset is
///     exactly the six encodings above, so we key the admit list off
///     concrete hw0/hw1 patterns and reject everything else.
fn is_m0plus_silicon_safe(tc: &TestCase) -> bool {
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
    // need to fuzz hints, so filter the whole range (matches
    // `qemu_diff_m0plus`'s treatment).
    if (tc.opcode & 0xFF00) == 0xBF00 {
        return false;
    }

    // CBZ / CBNZ (0xB1xx / 0xB3xx / 0xB9xx / 0xBBxx).
    if matches!(tc.opcode & 0xF500, 0xB100) {
        return false;
    }

    // M33-only xPSR flag families (Q-flag alone, GE flags). M0+ accepts
    // no-flags, NZ-only, and full NZCV (plus Q in the mask, but M0+ just
    // leaves Q clear).
    let m = tc.xpsr_mask;
    if m != 0 && m != MASK_ALL_FLAGS && m != MASK_NZ_ONLY {
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
// Hardware-side execution
// ============================================================================

/// Execute a single test case on hardware via probe-rs single-step.
/// Returns post-execution state (no cycle count — M0+ has no DWT CYCCNT).
fn run_one_probe(
    core: &mut Core,
    tc: &TestCase,
) -> Result<RunState, DiffError> {
    // 1. Write instruction (16 or 32 bits) + BKPT sentinel to test slot.
    let mut code = tc.opcode.to_le_bytes().to_vec();
    if let Some(hw1) = tc.hw1 {
        code.extend_from_slice(&hw1.to_le_bytes());
    }
    code.extend_from_slice(&BKPT.to_le_bytes());
    core.write_8(EMU_M0PLUS_TEST_SLOT as u64, &code).map_err(DiffError::ProbeError)?;

    // 2. Set register defaults: R0-R12 = 0
    for i in 0..=12u16 {
        core.write_core_reg(RegisterId(i), 0u32).map_err(DiffError::ProbeError)?;
    }
    // SP = test stack, LR = sentinel, PC = test slot, xPSR = precondition.
    core.write_core_reg(RegisterId(13), EMU_M0PLUS_TEST_STACK).map_err(DiffError::ProbeError)?;
    core.write_core_reg(RegisterId(14), 0xFFFF_FFFFu32).map_err(DiffError::ProbeError)?;
    core.write_core_reg(PC, EMU_M0PLUS_TEST_SLOT).map_err(DiffError::ProbeError)?;
    core.write_core_reg(XPSR, tc.xpsr_pre).map_err(DiffError::ProbeError)?;

    // 3. Apply register preconditions (same address space as the emulator).
    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, EMU_M0PLUS_TEST_SCRATCH);
        core.write_core_reg(RegisterId(reg as u16), val).map_err(DiffError::ProbeError)?;
    }

    // 4. Memory setup (zero scratch + write preconditions).
    if tc.needs_bus {
        core.write_8(
            EMU_M0PLUS_TEST_SCRATCH as u64,
            &[0u8; SCRATCH_SIZE as usize],
        ).map_err(DiffError::ProbeError)?;
        for &(offset, val) in &tc.mem_pre {
            core.write_8((EMU_M0PLUS_TEST_SCRATCH + offset) as u64, &[val]).map_err(DiffError::ProbeError)?;
        }
    }

    // 5. Single-step the instruction under test. Thumb-32 is one step on
    // M0+ just like on M33 — the CPU consumes both halfwords per step.
    core.step().map_err(DiffError::ProbeError)?;

    // 6. UNDEF-on-silicon sanity: if PC landed inside bootrom, silicon
    // HardFaulted and was dispatched via VTOR=0. Short-circuit before
    // touching the rest of the state. Branch targets inside the test
    // region are left for `compare_probe` to evaluate semantically.
    let pc_after: u32 = core.read_core_reg(PC).map_err(DiffError::ProbeError)?;
    if let Some(pc) = classify_post_step_pc(pc_after) {
        return Err(DiffError::UndefOnSilicon { pc });
    }

    // 7. Read post-state.
    let mut regs = [0u32; 16];
    for i in 0..16u32 {
        regs[i as usize] = core.read_core_reg(RegisterId(i as u16)).map_err(DiffError::ProbeError)?;
    }
    let xpsr: u32 = core.read_core_reg(XPSR).map_err(DiffError::ProbeError)?;

    // 8. Read memory at mem_check offsets.
    let mut mem = Vec::new();
    for &offset in &tc.mem_check {
        let mut byte = [0u8; 1];
        core.read_8((EMU_M0PLUS_TEST_SCRATCH + offset) as u64, &mut byte).map_err(DiffError::ProbeError)?;
        mem.push(byte[0]);
    }

    Ok(RunState {
        regs,
        xpsr,
        mem,
        cycles: 0,
        fpu: Vec::new(),
        fpscr: 0,
    })
}

// ============================================================================
// Emulator-side execution (M0+ flavour)
// ============================================================================

/// Run a single test case on the mdrp2040 emulator and return post-state.
///
/// Parallel to `mdpicoem_harness::run_one_emu` but uses `CortexM0Plus` +
/// the M0+ SRAM layout. Dispatches to `execute_one_wide_with_bus` when
/// the test case carries a `hw1` (Thumb-32 subset).
fn run_one_emu_m0plus(tc: &TestCase, bus: &mut M0Bus) -> RunState {
    let mut core = CortexM0Plus::new();

    for i in 0..=12 {
        core.set_reg(i, 0);
    }
    core.set_reg(13, EMU_M0PLUS_TEST_STACK);
    core.set_reg(14, 0xFFFF_FFFF);
    core.regs.set_pc(EMU_M0PLUS_TEST_SLOT);
    core.regs.xpsr = tc.xpsr_pre;

    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, EMU_M0PLUS_TEST_SCRATCH);
        core.set_reg(reg as usize, val);
    }

    if tc.needs_bus {
        for i in 0..SCRATCH_SIZE {
            bus.write8(EMU_M0PLUS_TEST_SCRATCH + i, 0);
        }
        for &(offset, val) in &tc.mem_pre {
            bus.write8(EMU_M0PLUS_TEST_SCRATCH + offset, val);
        }
    }

    let cycles = match tc.hw1 {
        None => if tc.needs_bus {
            core.execute_one_with_bus(tc.opcode, bus)
        } else {
            core.execute_one(tc.opcode)
        },
        Some(hw1) => if tc.needs_bus {
            core.execute_one_wide_with_bus(tc.opcode, hw1, bus)
        } else {
            core.execute_one_wide(tc.opcode, hw1)
        },
    };

    let mut regs = [0u32; 16];
    for i in 0..16 {
        regs[i] = core.reg(i);
    }
    let xpsr = core.regs.xpsr;
    let mem: Vec<u8> = tc
        .mem_check
        .iter()
        .map(|&offset| bus.read8(EMU_M0PLUS_TEST_SCRATCH + offset))
        .collect();

    RunState {
        regs,
        xpsr,
        mem,
        cycles,
        fpu: Vec::new(),
        fpscr: 0,
    }
}

// ============================================================================
// Main runner
// ============================================================================

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;

    println!("probe_diff_rp2040: RP2040 hardware differential test runner");
    println!("===========================================================");

    // 1. Attach to target via probe-rs. With --probe, route through the
    // explicit selector to disambiguate multiple attached probes (see HLD
    // §2.1 — `auto_attach` just picks the first-enumerated probe).
    let mut session = match args.probe.as_ref() {
        None => Session::auto_attach("rp2040", SessionConfig::default())?,
        Some(selector) => {
            let probe = Lister::new().open(selector.clone())?;
            probe.attach("rp2040", Permissions::default())?
        }
    };
    let mut core = session.core(0)?;
    println!("Attached to target, using core 0");

    // 2. Reset and halt. Longer timeout than the M33 path (some RP2040
    // probes are slower to respond under soft reset).
    core.reset_and_halt(Duration::from_millis(500))?;

    // 3. No DWT to enable — RP2040 / Cortex-M0+ does not implement DWT.

    // 4. Generate and run.
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

/// Run the targeted edge-case test suite.
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

    if fail > 0 || undef > 0 {
        std::process::exit(1);
    }
    Ok(())
}

/// Run fuzz tests with progress reporting.
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
        if done % 100 == 0 {
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
    if fail > 0 || undef > 0 {
        println!("Reproduce: probe_diff_rp2040 --fuzz {count_per_class} --seed {seed}");
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
    /// Post-step PC landed inside the RP2040 bootrom (0..0x3FFF), which
    /// on a VTOR=0 core means the instruction UNDEF'd on silicon and was
    /// dispatched into the bootrom's HardFault handler. Surfaces
    /// filter-gap bugs in `is_m0plus_silicon_safe`.
    UndefOnSilicon { pc: u32 },
}

/// Classify the PC read after `core.step()`. Returns `Some(pc)` only when
/// silicon dispatched into the bootrom (the specific case the HLD set out
/// to catch); all other post-step PCs — BKPT sentinel, branch targets,
/// POP-to-PC landings — are left to the register-level comparison.
fn classify_post_step_pc(pc: u32) -> Option<u32> {
    if pc < RP2040_BOOTROM_END {
        Some(pc)
    } else {
        None
    }
}

/// Run one test on both hardware and the emulator, compare results.
/// Semantic-only comparison — no cycle check (M0+ lacks DWT CYCCNT).
fn run_one_diff(
    core: &mut Core,
    bus: &mut M0Bus,
    tc: &TestCase,
) -> Result<(), DiffError> {
    let hw_state = run_one_probe(core, tc)?;
    let emu_state = run_one_emu_m0plus(tc, bus);
    compare_probe(tc, &hw_state, &emu_state).map_err(DiffError::Mismatch)
}

// ============================================================================
// Summary
// ============================================================================

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
// Filter-level self-tests (emu-side only; no silicon required)
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
        assert!(is_m0plus_silicon_safe(&msr_case(16)), "PRIMASK must be allowed");
        assert!(is_m0plus_silicon_safe(&msr_case(20)), "CONTROL must be allowed");
        assert!(is_m0plus_silicon_safe(&mrs_case(16)), "MRS PRIMASK must be allowed");
        assert!(is_m0plus_silicon_safe(&mrs_case(20)), "MRS CONTROL must be allowed");
    }

    #[test]
    fn filter_rejects_basepri_faultmask() {
        assert!(!is_m0plus_silicon_safe(&msr_case(17)), "BASEPRI must be rejected");
        assert!(!is_m0plus_silicon_safe(&msr_case(19)), "FAULTMASK must be rejected");
        assert!(!is_m0plus_silicon_safe(&mrs_case(17)), "MRS BASEPRI must be rejected");
        assert!(!is_m0plus_silicon_safe(&mrs_case(19)), "MRS FAULTMASK must be rejected");
    }

    #[test]
    fn filter_rejects_banked_ns_aliases() {
        // sysm >= 0x80 are banked _NS aliases (M33 TrustZone only).
        assert!(!is_m0plus_silicon_safe(&msr_case(0x90)), "banked MSR must be rejected");
        assert!(!is_m0plus_silicon_safe(&mrs_case(0x94)), "banked MRS must be rejected");
    }

    #[test]
    fn filter_admits_barriers() {
        // DMB / DSB / ISB — all three share hw0 = 0xF3BF, hw1[15:8] = 0x8F.
        let dmb = TestCase {
            opcode: 0xF3BF, hw1: Some(0x8F5F),
            ..TestCase::default()
        };
        let dsb = TestCase {
            opcode: 0xF3BF, hw1: Some(0x8F4F),
            ..TestCase::default()
        };
        let isb = TestCase {
            opcode: 0xF3BF, hw1: Some(0x8F6F),
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&dmb));
        assert!(is_m0plus_silicon_safe(&dsb));
        assert!(is_m0plus_silicon_safe(&isb));
    }

    #[test]
    fn filter_admits_bl() {
        // BL to a small positive offset — hw0 & 0xF800 == 0xF000,
        // hw1 & 0xD000 == 0xD000.
        let (hw0, hw1) = mdpicoem_harness::thumb32_gen::enc_t32_bl(4);
        let tc = TestCase {
            opcode: hw0, hw1: Some(hw1),
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&tc), "BL must be allowed");
    }

    #[test]
    fn filter_rejects_other_thumb32() {
        // A random non-subset Thumb-32 — e.g. TBB (hw0 = 0xE8DF, hw1 = 0xF000).
        let tc = TestCase {
            opcode: 0xE8DF, hw1: Some(0xF000),
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&tc), "TBB must be rejected");

        // LDRD literal — another M33-only Thumb-32 encoding.
        let tc = TestCase {
            opcode: 0xE95F, hw1: Some(0x0100),
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&tc), "LDRD literal must be rejected");
    }

    #[test]
    fn filter_rejects_it_and_cbz() {
        // IT EQ — 0xBF08.
        let it = TestCase {
            opcode: 0xBF08,
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&it), "IT must be rejected");

        // CBZ R0, <label> — 0xB100 | ...
        let cbz = TestCase {
            opcode: 0xB108,
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&cbz), "CBZ must be rejected");

        // CBNZ — 0xB9xx.
        let cbnz = TestCase {
            opcode: 0xB920,
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&cbnz), "CBNZ must be rejected");
    }

    #[test]
    fn filter_rejects_fpu_and_multistep() {
        // FPU test (non-empty fpu_pre).
        let fpu = TestCase {
            opcode: 0x0000,
            fpu_pre: vec![(0, 0x3F80_0000)],
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&fpu), "FPU test must be rejected");

        // Multi-step IT body.
        let multi = TestCase {
            opcode: 0xBF08,
            opcode2: Some(0x0000),
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&multi), "multi-step must be rejected");
    }

    #[test]
    fn filter_admits_common_thumb16_alu() {
        // MOVS R0, #42 — 0x202A.
        let movs = TestCase {
            opcode: 0x202A,
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&movs));

        // ADDS R0, R1, R2 — 0x1888.
        let adds = TestCase {
            opcode: 0x1888,
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&adds));
    }

    #[test]
    fn emu_side_runs_msr_primask_without_panic() {
        // End-to-end smoke of the emu-side pipeline on one admitted Thumb-32
        // case: MSR PRIMASK, R0 with R0=1. No probe attach; just validates
        // the dispatch into `execute_one_wide_with_bus` runs cleanly.
        let (hw0, hw1) = enc_t32_msr(0, 16);
        let tc = TestCase {
            name: "MSR PRIMASK,R0=1 (emu smoke)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 1)],
            xpsr_pre: 0x0100_0000,
            xpsr_mask: 0,
            ..TestCase::default()
        };
        let mut bus = M0Bus::new();
        let state = run_one_emu_m0plus(&tc, &mut bus);
        // R0 preserved, PC advanced by 4.
        assert_eq!(state.regs[0], 1);
        assert_eq!(state.regs[15], EMU_M0PLUS_TEST_SLOT + 4);
    }

    #[test]
    fn emu_side_runs_thumb16_without_panic() {
        // MOVS R0, #0x5A. Smoke-tests the non-wide dispatch path.
        let tc = TestCase {
            name: "MOVS R0,#0x5A (emu smoke)".into(),
            opcode: 0x205A,
            ..TestCase::default()
        };
        let mut bus = M0Bus::new();
        let state = run_one_emu_m0plus(&tc, &mut bus);
        assert_eq!(state.regs[0], 0x5A);
        assert_eq!(state.regs[15], EMU_M0PLUS_TEST_SLOT + 2);
    }

    // -----------------------------------------------------------------
    // --probe flag parsing
    // -----------------------------------------------------------------

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
                assert!(err.contains("bogus"), "error should echo the bad value: {err}");
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
    // Post-step PC classification
    // -----------------------------------------------------------------

    #[test]
    fn classify_pc_in_bootrom_flags_undef() {
        // Anywhere in 0..0x3FFF is evidence of HardFault dispatch via VTOR=0.
        for pc in [0x0000_0004, 0x0000_0010, 0x0000_1234, 0x0000_3FFC] {
            assert_eq!(classify_post_step_pc(pc), Some(pc), "pc={pc:#010x}");
        }
    }

    #[test]
    fn classify_pc_at_bootrom_boundary_is_not_undef() {
        // 0x4000 is the first address outside bootrom — exclusive upper bound.
        assert_eq!(classify_post_step_pc(0x0000_4000), None);
    }

    #[test]
    fn classify_pc_at_test_slot_bkpt_is_not_undef() {
        // Happy-path PCs for Thumb-16 (slot+2) and Thumb-32 (slot+4) BKPT sentinels.
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_SLOT + 2), None);
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_SLOT + 4), None);
    }

    #[test]
    fn classify_pc_at_bl_target_is_not_undef() {
        // Regression for the bug this fix closes: BL lands past slot+4 but
        // still inside the test region, NOT inside bootrom.
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_SLOT + 8), None);
    }

    #[test]
    fn classify_pc_at_pop_pc_garbage_is_not_undef() {
        // POP {PC} with stack garbage lands at some non-bootrom address;
        // let `compare_probe` flag it semantically rather than UNDEF-classify.
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_STACK + 0x100), None);
    }

    #[test]
    fn generate_all_admits_reasonable_subset() {
        // End-to-end: the targeted catalogue should produce a non-trivial
        // M0+-safe subset that includes both the Thumb-16 body and the
        // Thumb-32 subset (BL / MRS PRIMASK-CONTROL / MSR PRIMASK-CONTROL /
        // barriers). If the filter drops the admit list to zero or
        // excludes the subset entirely, something has gone wrong.
        let all = generate_all();
        let admitted: Vec<_> = all
            .into_iter()
            .filter(is_m0plus_silicon_safe)
            .collect();
        assert!(
            admitted.len() > 100,
            "filter should admit at least 100 common Thumb-16 cases; got {}",
            admitted.len()
        );

        // Heuristic: at least one admitted case should be Thumb-32.
        let t32_count = admitted.iter().filter(|tc| tc.hw1.is_some()).count();
        assert!(
            t32_count > 0,
            "filter should admit at least one Thumb-32 case (BL / MSR PRIMASK / MRS PRIMASK / barrier)"
        );

        // Heuristic: no admitted MSR/MRS case targets sysm 17 or 19.
        for tc in &admitted {
            if let Some(hw1) = tc.hw1 {
                let hw0 = tc.opcode;
                let is_msr = (hw0 & 0xFFF0) == 0xF380 && (hw1 & 0xFF00) == 0x8800;
                let is_mrs = hw0 == 0xF3EF && (hw1 & 0xF000) == 0x8000;
                if is_msr || is_mrs {
                    let sysm = hw1 & 0xFF;
                    assert!(
                        sysm != 17 && sysm != 19 && sysm < 0x80,
                        "admitted MSR/MRS with disallowed sysm={sysm}: {}",
                        tc.name
                    );
                }
            }
        }
    }
}

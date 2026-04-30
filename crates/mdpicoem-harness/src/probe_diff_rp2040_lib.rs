// probe_diff_rp2040_lib — RP2040 hardware differential test runner as a
// library API. Wraps the M0+ silicon-safe filter, the per-test probe
// driver, the per-test emulator driver, and the comparison oracle so the
// `test_silicon_rp2040` orchestrator can call this oracle alongside the
// peripheral / ISR oracles under one shared probe session.
//
// The standalone binary (`bin/probe_diff_rp2040.rs`) is a thin wrapper
// that opens the probe session and forwards the borrowed `Core` handle
// to `run_against`. CLI surface (flags, exit codes) is unchanged.

use crate::m0plus::{Bus as M0Bus, CortexM0Plus};
use crate::silicon_oracle::CaseOutcome;
use crate::{
    EMU_M0PLUS_TEST_SCRATCH, EMU_M0PLUS_TEST_SLOT, EMU_M0PLUS_TEST_STACK, MASK_ALL_FLAGS,
    MASK_NZ_ONLY, RunState, SCRATCH_SIZE, TestCase, compare_probe, generate_all, generate_fuzz,
    setup_reg,
};
use probe_rs::{Core, MemoryInterface, RegisterId};
use std::time::Instant;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const PC: RegisterId = RegisterId(15);
const XPSR: RegisterId = RegisterId(16);
const BKPT: u16 = 0xBE00;

/// RP2040 bootrom occupies 0x0000_0000..=0x0000_3FFF (16 KB). The
/// UNDEF-on-silicon classifier treats any post-step PC landing in this
/// range as evidence that silicon HardFaulted and was dispatched via
/// VTOR=0 into the bootrom's fault handler.
pub const RP2040_BOOTROM_END: u32 = 0x0000_4000;

// ---------------------------------------------------------------------------
// CLI args (library-level — no probe attach, that's the caller's job)
// ---------------------------------------------------------------------------

/// Library-level args for `run_against`. The orchestrator and the
/// standalone binary populate these from their own CLI parsers.
#[derive(Clone, Debug, Default)]
pub struct ProbeDiffArgs {
    /// `Some(N)` — run fuzz mode with N tests per class. `None` — run
    /// the targeted edge-case suite.
    pub fuzz_count: Option<usize>,
    /// Fuzz seed (only meaningful with `fuzz_count = Some(_)`). The
    /// orchestrator sets this per iteration via `iter_seed(base, iter)`.
    pub seed: u64,
}

// ---------------------------------------------------------------------------
// M0+ silicon compatibility filter
// ---------------------------------------------------------------------------

/// Is this test case runnable on real RP2040 / Cortex-M0+ silicon?
///
/// Admits Thumb-16 instructions common to M0+ and M33 **plus** the M0+
/// Thumb-32 subset: `BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`. Mirrors the
/// QEMU sibling — both gate Thumb-32 through `m0plus_admits_wide`.
///
/// Rejects: FPU tests, multi-step / IT-block tests, raw IT/hint
/// (0xBFxx), CBZ / CBNZ, non-standard xPSR masks (Q-flag / GE-flag),
/// MSR / MRS targeting BASEPRI / FAULTMASK / banked _NS aliases, and
/// any other Thumb-32 encoding outside the M0+ subset.
pub fn is_m0plus_silicon_safe(tc: &TestCase) -> bool {
    // FPU tests: M0+ has no FPU.
    if !tc.fpu_pre.is_empty() || !tc.fpu_check.is_empty() || tc.fpscr_mask != 0 {
        return false;
    }

    // Multi-step / IT-body tests: M0+ has no IT blocks.
    if tc.opcode2.is_some() || tc.hw1_2.is_some() {
        return false;
    }

    // Raw IT / hint prefix (0xBFxx).
    if (tc.opcode & 0xFF00) == 0xBF00 {
        return false;
    }

    // CBZ / CBNZ.
    if matches!(tc.opcode & 0xF500, 0xB100) {
        return false;
    }

    // M33-only xPSR flag families. Admitted: no-flags, NZ-only,
    // NZCV-only (architectural ARMv6-M APSR width — the mask used by
    // `fuzz_m0plus_msr` for MSR APSR sysm=0 cases), and full NZCVQ.
    let m = tc.xpsr_mask;
    if m != 0 && m != MASK_ALL_FLAGS && m != MASK_NZ_ONLY && m != crate::MASK_NZCV_ONLY {
        return false;
    }

    // Thumb-32 admit list (BL / MSR / MRS / DSB / DMB / ISB) with sysm
    // reject set (BASEPRI=17, FAULTMASK=19, banked >= 0x80).
    if let Some(hw1) = tc.hw1 {
        if !crate::m0plus_admits_wide(tc.opcode, hw1) {
            return false;
        }
    }

    true
}

// ---------------------------------------------------------------------------
// DiffError + post-step PC classifier
// ---------------------------------------------------------------------------

pub enum DiffError {
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
pub fn classify_post_step_pc(pc: u32) -> Option<u32> {
    if pc < RP2040_BOOTROM_END {
        Some(pc)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Hardware-side execution
// ---------------------------------------------------------------------------

/// Execute a single test case on hardware via probe-rs single-step.
/// Returns post-execution state (no cycle count — M0+ has no DWT CYCCNT).
pub fn run_one_probe(core: &mut Core, tc: &TestCase) -> Result<RunState, DiffError> {
    let mut code = tc.opcode.to_le_bytes().to_vec();
    if let Some(hw1) = tc.hw1 {
        code.extend_from_slice(&hw1.to_le_bytes());
    }
    code.extend_from_slice(&BKPT.to_le_bytes());
    core.write_8(EMU_M0PLUS_TEST_SLOT as u64, &code)
        .map_err(DiffError::ProbeError)?;

    for i in 0..=12u16 {
        core.write_core_reg(RegisterId(i), 0u32)
            .map_err(DiffError::ProbeError)?;
    }
    core.write_core_reg(RegisterId(13), EMU_M0PLUS_TEST_STACK)
        .map_err(DiffError::ProbeError)?;
    core.write_core_reg(RegisterId(14), 0xFFFF_FFFFu32)
        .map_err(DiffError::ProbeError)?;
    core.write_core_reg(PC, EMU_M0PLUS_TEST_SLOT)
        .map_err(DiffError::ProbeError)?;
    core.write_core_reg(XPSR, tc.xpsr_pre)
        .map_err(DiffError::ProbeError)?;

    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, EMU_M0PLUS_TEST_SCRATCH);
        core.write_core_reg(RegisterId(reg as u16), val)
            .map_err(DiffError::ProbeError)?;
    }

    if tc.needs_bus {
        core.write_8(
            EMU_M0PLUS_TEST_SCRATCH as u64,
            &[0u8; SCRATCH_SIZE as usize],
        )
        .map_err(DiffError::ProbeError)?;
        for &(offset, val) in &tc.mem_pre {
            core.write_8((EMU_M0PLUS_TEST_SCRATCH + offset) as u64, &[val])
                .map_err(DiffError::ProbeError)?;
        }
    }

    core.step().map_err(DiffError::ProbeError)?;

    let pc_after: u32 = core.read_core_reg(PC).map_err(DiffError::ProbeError)?;
    if let Some(pc) = classify_post_step_pc(pc_after) {
        return Err(DiffError::UndefOnSilicon { pc });
    }

    let mut regs = [0u32; 16];
    for i in 0..16u32 {
        regs[i as usize] = core
            .read_core_reg(RegisterId(i as u16))
            .map_err(DiffError::ProbeError)?;
    }
    let xpsr: u32 = core.read_core_reg(XPSR).map_err(DiffError::ProbeError)?;

    let mut mem = Vec::new();
    for &offset in &tc.mem_check {
        let mut byte = [0u8; 1];
        core.read_8((EMU_M0PLUS_TEST_SCRATCH + offset) as u64, &mut byte)
            .map_err(DiffError::ProbeError)?;
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

// ---------------------------------------------------------------------------
// Emulator-side execution (M0+ flavour)
// ---------------------------------------------------------------------------

/// Run a single test case on the mdrp2040 emulator and return post-state.
pub fn run_one_emu_m0plus(tc: &TestCase, bus: &mut M0Bus) -> RunState {
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

/// Run one test on both hardware and the emulator, compare results.
/// Semantic-only comparison — no cycle check (M0+ lacks DWT CYCCNT).
pub fn run_one_diff(core: &mut Core, bus: &mut M0Bus, tc: &TestCase) -> Result<(), DiffError> {
    let hw_state = run_one_probe(core, tc)?;
    let emu_state = run_one_emu_m0plus(tc, bus);
    compare_probe(tc, &hw_state, &emu_state).map_err(DiffError::Mismatch)
}

// ---------------------------------------------------------------------------
// rc_for — degraded-mode exit-code mapping (used by the standalone binary)
// ---------------------------------------------------------------------------

/// Map post-run counters to a process exit code.
///
/// rc=1 (any failure or UNDEF-on-silicon) > rc=3 (degraded transport) > rc=0.
pub fn rc_for(pass: usize, fail: usize, skip: usize, undef: usize) -> i32 {
    if fail > 0 || undef > 0 {
        return 1;
    }
    let attempted = pass + fail + skip + undef;
    if attempted >= 20 && (skip * 100) / attempted >= 25 {
        return 3;
    }
    0
}

// ---------------------------------------------------------------------------
// Library entry point
// ---------------------------------------------------------------------------

/// Run the probe_diff oracle against `core`. Returns `Vec<CaseOutcome>`.
///
/// * `args.fuzz_count = None` → run targeted edge-case catalogue.
/// * `args.fuzz_count = Some(N)` → fuzz mode: N tests per class, seeded by `args.seed`.
///
/// `order` filters/orders cases by name (substring-equality match
/// against `TestCase.name`); `None` = all admitted cases in default
/// order. `deadline = Some(t)` returns early between cases when
/// `Instant::now() > t` (the orchestrator's 60s watchdog).
///
/// Verdict mapping:
/// * `run_one_diff` Ok → Pass.
/// * `Mismatch(d)` → Fail with `detail = d`.
/// * `ProbeError(e)` → Degraded with `detail = "probe-rs error: {e}"` —
///   transport hiccup, not divergence.
/// * `UndefOnSilicon { pc }` → Skip with
///   `detail = "UNDEF_ON_SILICON: pc=0x{pc:08X} (filter gap)"` —
///   coverage gap, not divergence.
pub fn run_against(
    core: &mut Core,
    args: &ProbeDiffArgs,
    order: Option<&[&str]>,
    deadline: Option<Instant>,
) -> Result<Vec<CaseOutcome>, Box<dyn std::error::Error + Send + Sync>> {
    // Build the candidate list.
    let cases: Vec<TestCase> = match args.fuzz_count {
        None => generate_all()
            .into_iter()
            .filter(is_m0plus_silicon_safe)
            .filter(|tc| !tc.probe_only)
            .collect(),
        Some(count) => {
            let (alu, mem) = generate_fuzz(count, args.seed);
            alu.into_iter()
                .chain(mem.into_iter())
                .filter(is_m0plus_silicon_safe)
                .filter(|tc| !tc.probe_only)
                .collect()
        }
    };

    // Apply order filter if any.
    let cases: Vec<TestCase> = match order {
        None => cases,
        Some(names) => {
            let mut by_name: std::collections::HashMap<String, TestCase> = cases
                .into_iter()
                .map(|tc| (tc.name.clone(), tc))
                .collect();
            let mut out: Vec<TestCase> = Vec::with_capacity(names.len());
            for n in names {
                if let Some(tc) = by_name.remove(*n) {
                    out.push(tc);
                }
            }
            out
        }
    };

    let mut bus = M0Bus::new();
    let mut outcomes: Vec<CaseOutcome> = Vec::with_capacity(cases.len());
    let mut interner = SyntheticNameInterner::default();

    for tc in &cases {
        if let Some(d) = deadline {
            if Instant::now() > d {
                break;
            }
        }
        let t0 = Instant::now();
        let case_static = interner.intern(&tc.name);
        match run_one_diff(core, &mut bus, tc) {
            Ok(()) => {
                let elapsed_ms = t0.elapsed().as_millis().min(u32::MAX as u128) as u32;
                outcomes.push(CaseOutcome::pass("probe_diff", case_static, elapsed_ms));
            }
            Err(DiffError::Mismatch(diff)) => {
                let elapsed_ms = t0.elapsed().as_millis().min(u32::MAX as u128) as u32;
                outcomes.push(CaseOutcome::fail(
                    "probe_diff",
                    case_static,
                    diff,
                    elapsed_ms,
                ));
            }
            Err(DiffError::ProbeError(e)) => {
                let elapsed_ms = t0.elapsed().as_millis().min(u32::MAX as u128) as u32;
                outcomes.push(CaseOutcome::degraded(
                    "probe_diff",
                    case_static,
                    format!("probe-rs error: {e}"),
                    elapsed_ms,
                ));
            }
            Err(DiffError::UndefOnSilicon { pc }) => {
                let elapsed_ms = t0.elapsed().as_millis().min(u32::MAX as u128) as u32;
                outcomes.push(CaseOutcome::skip(
                    "probe_diff",
                    case_static,
                    format!("UNDEF_ON_SILICON: pc=0x{pc:08X} (filter gap)"),
                    elapsed_ms,
                ));
            }
        }
    }

    Ok(outcomes)
}

// ---------------------------------------------------------------------------
// Local synthetic-name interner
// ---------------------------------------------------------------------------
//
// `CaseOutcome.case` is `&'static str` but the probe_diff catalogue's
// `TestCase.name` is `String`. Box::leak each unique name once so the
// orchestrator can record the case under a stable static identifier
// without leaking on every iteration of a soak run. Bounded by the
// ~thousand-or-so unique generated names.

#[derive(Default)]
struct SyntheticNameInterner {
    seen: std::collections::HashMap<String, &'static str>,
}

impl SyntheticNameInterner {
    fn intern(&mut self, s: &str) -> &'static str {
        if let Some(v) = self.seen.get(s) {
            return v;
        }
        let leaked: &'static str = Box::leak(s.to_string().into_boxed_str());
        self.seen.insert(s.to_string(), leaked);
        leaked
    }
}

// ---------------------------------------------------------------------------
// Tests (filter / classifier / rc — no probe access)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::thumb32_gen::{enc_t32_mrs, enc_t32_msr};

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
        assert!(is_m0plus_silicon_safe(&msr_case(16)));
        assert!(is_m0plus_silicon_safe(&msr_case(20)));
        assert!(is_m0plus_silicon_safe(&mrs_case(16)));
        assert!(is_m0plus_silicon_safe(&mrs_case(20)));
    }

    #[test]
    fn filter_rejects_basepri_faultmask() {
        assert!(!is_m0plus_silicon_safe(&msr_case(17)));
        assert!(!is_m0plus_silicon_safe(&msr_case(19)));
        assert!(!is_m0plus_silicon_safe(&mrs_case(17)));
        assert!(!is_m0plus_silicon_safe(&mrs_case(19)));
    }

    #[test]
    fn filter_rejects_banked_ns_aliases() {
        assert!(!is_m0plus_silicon_safe(&msr_case(0x90)));
        assert!(!is_m0plus_silicon_safe(&mrs_case(0x94)));
    }

    #[test]
    fn filter_admits_barriers() {
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
        assert!(is_m0plus_silicon_safe(&dmb));
        assert!(is_m0plus_silicon_safe(&dsb));
        assert!(is_m0plus_silicon_safe(&isb));
    }

    #[test]
    fn filter_admits_bl() {
        let (hw0, hw1) = crate::thumb32_gen::enc_t32_bl(4);
        let tc = TestCase {
            opcode: hw0,
            hw1: Some(hw1),
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&tc));
    }

    #[test]
    fn filter_rejects_other_thumb32() {
        // TBB.
        let tc = TestCase {
            opcode: 0xE8DF,
            hw1: Some(0xF000),
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&tc));
        // LDRD literal.
        let tc = TestCase {
            opcode: 0xE95F,
            hw1: Some(0x0100),
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&tc));
    }

    #[test]
    fn filter_rejects_it_and_cbz() {
        let it = TestCase {
            opcode: 0xBF08,
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&it));
        let cbz = TestCase {
            opcode: 0xB108,
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&cbz));
        let cbnz = TestCase {
            opcode: 0xB920,
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&cbnz));
    }

    #[test]
    fn filter_rejects_fpu_and_multistep() {
        let fpu = TestCase {
            opcode: 0x0000,
            fpu_pre: vec![(0, 0x3F80_0000)],
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&fpu));
        let multi = TestCase {
            opcode: 0xBF08,
            opcode2: Some(0x0000),
            ..TestCase::default()
        };
        assert!(!is_m0plus_silicon_safe(&multi));
    }

    #[test]
    fn filter_admits_common_thumb16_alu() {
        let movs = TestCase {
            opcode: 0x202A,
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&movs));
        let adds = TestCase {
            opcode: 0x1888,
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&adds));
    }

    #[test]
    fn emu_side_runs_msr_primask_without_panic() {
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
        assert_eq!(state.regs[0], 1);
        assert_eq!(state.regs[15], EMU_M0PLUS_TEST_SLOT + 4);
    }

    #[test]
    fn emu_side_runs_thumb16_without_panic() {
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

    #[test]
    fn classify_pc_in_bootrom_flags_undef() {
        for pc in [0x0000_0004, 0x0000_0010, 0x0000_1234, 0x0000_3FFC] {
            assert_eq!(classify_post_step_pc(pc), Some(pc), "pc={pc:#010x}");
        }
    }

    #[test]
    fn classify_pc_at_bootrom_boundary_is_not_undef() {
        assert_eq!(classify_post_step_pc(0x0000_4000), None);
    }

    #[test]
    fn classify_pc_at_test_slot_bkpt_is_not_undef() {
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_SLOT + 2), None);
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_SLOT + 4), None);
    }

    #[test]
    fn classify_pc_at_bl_target_is_not_undef() {
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_SLOT + 8), None);
    }

    #[test]
    fn classify_pc_at_pop_pc_garbage_is_not_undef() {
        assert_eq!(classify_post_step_pc(EMU_M0PLUS_TEST_STACK + 0x100), None);
    }

    #[test]
    fn rc_for_clean_run_returns_zero() {
        assert_eq!(rc_for(8000, 0, 0, 0), 0);
    }

    #[test]
    fn rc_for_any_failures_returns_one() {
        assert_eq!(rc_for(7999, 1, 0, 0), 1);
        assert_eq!(rc_for(0, 1, 6000, 0), 1);
    }

    #[test]
    fn rc_for_undef_returns_one() {
        assert_eq!(rc_for(0, 0, 0, 5), 1);
        assert_eq!(rc_for(1000, 0, 6000, 1), 1);
    }

    #[test]
    fn rc_for_high_skip_returns_three() {
        assert_eq!(rc_for(1885, 0, 6115, 0), 3);
    }

    #[test]
    fn rc_for_borderline_skip_below_threshold() {
        assert_eq!(rc_for(7169, 0, 831, 0), 0);
    }

    #[test]
    fn rc_for_small_attempted_does_not_trip() {
        assert_eq!(rc_for(0, 0, 5, 0), 0);
    }

    #[test]
    fn rc_for_exactly_at_threshold() {
        assert_eq!(rc_for(75, 0, 25, 0), 3);
    }

    /// Stage E.2 regression: `MASK_NZCV_ONLY` (0xF000_0000) is the
    /// architectural ARMv6-M APSR width and the mask used by
    /// `fuzz_m0plus_msr` for MSR APSR (sysm=0) cases. Pre-fix the filter
    /// rejected it as a "non-standard xPSR flag family", silently
    /// dropping every APSR-write fuzz case.
    #[test]
    fn filter_admits_mask_nzcv_only() {
        // ANDS r1, r0 — Thumb-16 ALU, satisfies all non-mask gates.
        let case = TestCase {
            opcode: 0x4001,
            xpsr_mask: crate::MASK_NZCV_ONLY,
            ..TestCase::default()
        };
        assert!(is_m0plus_silicon_safe(&case));
    }

    #[test]
    fn generate_all_admits_reasonable_subset() {
        let all = generate_all();
        let admitted: Vec<_> = all.into_iter().filter(is_m0plus_silicon_safe).collect();
        assert!(
            admitted.len() > 100,
            "filter should admit at least 100 common Thumb-16 cases; got {}",
            admitted.len()
        );

        let t32_count = admitted.iter().filter(|tc| tc.hw1.is_some()).count();
        assert!(t32_count > 0);

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

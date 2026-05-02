// FPU differential test harness — Phase 7 Stage A.1.
//
// Each test case pumps a random (or targeted) operand pair through:
//   1. Our emulator's VFP arithmetic path (via `CortexM33::execute_one_wide`).
//   2. The hand-rolled IEEE-754 reference in `mdpicoem_harness::ieee754_ref`.
//
// We diff both the result bits AND the FPSCR exception flag delta. Any
// discrepancy is a real bug in the emulator (the reference is the oracle).
//
// The binary name is `softfloat_diff` for continuity with the HLD even
// though we hand-rolled the reference instead of wrapping Berkeley SoftFloat;
// see HLD §17.1 for the decision rationale.
//
// Usage:
//   softfloat_diff                   Run edge-case tests (FPU)
//   softfloat_diff --fuzz N          Run N random tests per instruction type
//   softfloat_diff --fuzz N --seed S Reproducible random run
//   softfloat_diff --mode dcp        Switch to DCP (CP4/5) differential fuzzing
//   softfloat_diff --mode all        FPU + DCP, edge-case or fuzz

use mdpicoem_harness::ieee754_ref;
use mdpicoem_harness::{Bus, CortexM33};
use rand::Rng;
use rand::SeedableRng;
use rand::rngs::StdRng;

// Which six FPSCR bits we care about (cumulative exception flags).
const FPSCR_FLAG_MASK: u32 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 7);

// FPSCR control bits we sweep over: FZ (flush-to-zero, bit 24) and DN
// (default NaN, bit 25). All four combinations are exercised so the oracle
// covers FTZ input flushing, FTZ output flushing, and DN NaN replacement.
const FPSCR_MODES: [(u32, &str); 4] = [
    (0, "FZ=0 DN=0"),
    (ieee754_ref::FZ, "FZ=1 DN=0"),
    (ieee754_ref::DN, "FZ=0 DN=1"),
    (ieee754_ref::FZ | ieee754_ref::DN, "FZ=1 DN=1"),
];

// ============================================================================
// Argument parsing
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Fpu,
    Dcp,
    All,
}

struct Args {
    fuzz_count: Option<usize>,
    seed: Option<u64>,
    mode: Mode,
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut fuzz_count = None;
    let mut seed = None;
    let mut mode = Mode::Fpu;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--fuzz" => {
                i += 1;
                fuzz_count = Some(
                    args.get(i)
                        .ok_or("--fuzz requires a count argument")?
                        .parse()
                        .map_err(|e| format!("invalid fuzz count: {e}"))?,
                );
            }
            "--seed" => {
                i += 1;
                seed = Some(
                    args.get(i)
                        .ok_or("--seed requires a value argument")?
                        .parse()
                        .map_err(|e| format!("invalid seed: {e}"))?,
                );
            }
            "--mode" => {
                i += 1;
                mode = match args
                    .get(i)
                    .ok_or("--mode requires a value (fpu|dcp|all)")?
                    .as_str()
                {
                    "fpu" => Mode::Fpu,
                    "dcp" => Mode::Dcp,
                    "all" => Mode::All,
                    other => return Err(format!("unknown mode '{other}' (fpu|dcp|all)")),
                };
            }
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     softfloat_diff                   Run edge-case tests (FPU)\n  \
                     softfloat_diff --fuzz N          Random tests per op type\n  \
                     softfloat_diff --fuzz N --seed S Reproducible random run\n  \
                     softfloat_diff --mode dcp        Fuzz CP4/5 DCP against ref_d*\n  \
                     softfloat_diff --mode all        FPU + DCP"
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
        mode,
    })
}

// ============================================================================
// VFP instruction encoders (duplicated from mdrp2350 tests.rs — those are
// private. Low cost to copy and keeps the test-harness self-contained.)
// ============================================================================

fn vfp_dp(op_hi: u16, op_lo: u16, op2_lo: u16, sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xEE00 | (op_hi << 7) | (d << 6) | (op_lo << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | (op2_lo << 6) | (m << 5) | vm;
    (hw0, hw1)
}

fn enc_vadd(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(0, 0b11, 0, sd, sn, sm)
}
fn enc_vsub(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(0, 0b11, 1, sd, sn, sm)
}
fn enc_vmul(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(0, 0b10, 0, sd, sn, sm)
}
fn enc_vdiv(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(1, 0b00, 0, sd, sn, sm)
}
fn enc_vfma(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(1, 0b10, 0, sd, sn, sm)
}

fn enc_vsqrt(sd: u16, sm: u16) -> (u16, u16) {
    // unary: hw0[7:4]=1D11, hw1[6]=1, opc3=0b0001, t=1
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xEE00 | (1 << 7) | (d << 6) | (0b11 << 4) | 0b0001;
    let hw1 = (vd << 12) | 0x0A00 | (1 << 7) | (1 << 6) | (m << 5) | vm;
    (hw0, hw1)
}

/// Generic VFP unary encoder (opc3 / t style: VRINT*, VCVT.F16↔F32, etc.).
fn vfp_unary(opc3: u16, t: u16, sd: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xEE00 | (1 << 7) | (d << 6) | (0b11 << 4) | (opc3 & 0xF);
    let hw1 = (vd << 12) | 0x0A00 | ((t & 1) << 7) | (1 << 6) | (m << 5) | vm;
    (hw0, hw1)
}

/// VMINNM.F32: hw0 = `1111_1110_1_00_D_Vn` (with hw0[7]=1 splitting from
/// VSEL, hw0[6:5]=00 disambiguating from reserved sub-ops);
/// hw1 = `Vd_1010_N_1_M_0_Vm` where hw1[6]=1 selects min vs max.
fn enc_vminnm(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xFE00 | (1 << 7) | (d << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | (1 << 6) | (m << 5) | vm;
    (hw0, hw1)
}

/// VMAXNM.F32: same shape as VMINNM but with hw1[6] = 0.
fn enc_vmaxnm(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xFE00 | (1 << 7) | (d << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | (m << 5) | vm;
    (hw0, hw1)
}

/// VSEL<cc>.F32. cc encodes in hw0[6:5]: 00=EQ, 01=VS, 10=GE, 11=GT.
/// hw0[7]=0 distinguishes from VMINNM/VMAXNM. hw1[8]=0, hw1[6]=0 (single,
/// no op2 selector).
fn enc_vsel(cc: u16, sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xFE00 | ((cc & 0x3) << 5) | (d << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | (m << 5) | vm;
    (hw0, hw1)
}

/// VCVT.F32.U32 — uint→float. (opc3=0b1000, t=0)
fn enc_vcvt_f32_u32(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b1000, 0, sd, sm)
}
/// VCVT.F32.S32 — int→float. (opc3=0b1000, t=1)
fn enc_vcvt_f32_s32(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b1000, 1, sd, sm)
}
/// VCVTR.U32.F32 — float→uint, RMode. (opc3=0b1100, t=0)
fn enc_vcvtr_u32_f32(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b1100, 0, sd, sm)
}
/// VCVT.U32.F32 — float→uint, RTZ. (opc3=0b1100, t=1)
fn enc_vcvt_u32_f32(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b1100, 1, sd, sm)
}
/// VCVTR.S32.F32 — float→int, RMode. (opc3=0b1101, t=0)
fn enc_vcvtr_s32_f32(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b1101, 0, sd, sm)
}
/// VCVT.S32.F32 — float→int, RTZ. (opc3=0b1101, t=1)
fn enc_vcvt_s32_f32(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b1101, 1, sd, sm)
}

fn enc_vcmp(sd: u16, sm: u16) -> (u16, u16) {
    // (opc3=0b0100, t=0) — fpu_unary dispatch row 893.
    vfp_unary(0b0100, 0, sd, sm)
}
fn enc_vcmpe(sd: u16, sm: u16) -> (u16, u16) {
    // (opc3=0b0100, t=1) — fpu_unary dispatch row 898. mdrp2350 treats
    // VCMP and VCMPE identically (no SNaN signalling).
    vfp_unary(0b0100, 1, sd, sm)
}
fn enc_vabs(sd: u16, sm: u16) -> (u16, u16) {
    // (opc3=0b0000, t=1) — see fpu_unary dispatch table at execute_fpu.rs:840.
    vfp_unary(0b0000, 1, sd, sm)
}
fn enc_vneg(sd: u16, sm: u16) -> (u16, u16) {
    // (opc3=0b0001, t=0)
    vfp_unary(0b0001, 0, sd, sm)
}
fn enc_vrintr(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b0110, 0, sd, sm)
}
fn enc_vrintz(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b0110, 1, sd, sm)
}
fn enc_vrintx(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b0111, 0, sd, sm)
}
// T-variants of VCVT.F16↔F32 share the same conversion logic — only the half
// of Sd/Sm differs. We test the B-variants; T-variants are exercised by the
// in-crate unit tests in `crates/mdrp2350/src/tests.rs`.
fn enc_vcvtb_f16_f32(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b0010, 0, sd, sm)
}
fn enc_vcvtb_f32_f16(sd: u16, sm: u16) -> (u16, u16) {
    vfp_unary(0b0011, 0, sd, sm)
}

// ============================================================================
// Test runner
// ============================================================================

#[derive(Clone, Copy)]
enum Op {
    Add,
    Sub,
    Mul,
    Div,
    Sqrt,
    Fma,
    VRintR,
    VRintX,
    VRintZ,
    Vabs,
    Vneg,
    Vminnm,
    Vmaxnm,
    Vcmp,
    Vcmpe,
    /// VCVT.F32.U32 — uint→float.
    VcvtFU,
    /// VCVT.F32.S32 — int→float.
    VcvtFS,
    /// VCVTR.U32.F32 — float→uint, FPSCR.RMode.
    VcvtUR,
    /// VCVT.U32.F32 — float→uint, RTZ.
    VcvtU,
    /// VCVTR.S32.F32 — float→int, FPSCR.RMode.
    VcvtSR,
    /// VCVT.S32.F32 — float→int, RTZ.
    VcvtS,
    /// VSEL.F32 with cc=EQ.
    VselEq,
    /// VSEL.F32 with cc=VS.
    VselVs,
    /// VSEL.F32 with cc=GE.
    VselGe,
    /// VSEL.F32 with cc=GT.
    VselGt,
}

impl Op {
    fn name(self) -> &'static str {
        match self {
            Op::Add => "VADD",
            Op::Sub => "VSUB",
            Op::Mul => "VMUL",
            Op::Div => "VDIV",
            Op::Sqrt => "VSQRT",
            Op::Fma => "VFMA",
            Op::VRintR => "VRINTR",
            Op::VRintX => "VRINTX",
            Op::VRintZ => "VRINTZ",
            Op::Vabs => "VABS",
            Op::Vneg => "VNEG",
            Op::Vminnm => "VMINNM",
            Op::Vmaxnm => "VMAXNM",
            Op::Vcmp => "VCMP",
            Op::Vcmpe => "VCMPE",
            Op::VcvtFU => "VCVT.F32.U32",
            Op::VcvtFS => "VCVT.F32.S32",
            Op::VcvtUR => "VCVTR.U32.F32",
            Op::VcvtU => "VCVT.U32.F32",
            Op::VcvtSR => "VCVTR.S32.F32",
            Op::VcvtS => "VCVT.S32.F32",
            Op::VselEq => "VSEL.EQ",
            Op::VselVs => "VSEL.VS",
            Op::VselGe => "VSEL.GE",
            Op::VselGt => "VSEL.GT",
        }
    }

    fn arity(self) -> usize {
        match self {
            Op::Sqrt
            | Op::VRintR
            | Op::VRintX
            | Op::VRintZ
            | Op::Vabs
            | Op::Vneg
            | Op::VcvtFU
            | Op::VcvtFS
            | Op::VcvtUR
            | Op::VcvtU
            | Op::VcvtSR
            | Op::VcvtS => 1,
            // FMA carries a third operand (accumulator).
            // VSEL repurposes the third slot to carry APSR.NZCV bits — see
            // `is_vsel` and the run_single setup.
            Op::Fma | Op::VselEq | Op::VselVs | Op::VselGe | Op::VselGt => 3,
            _ => 2,
        }
    }

    /// True if the op is a VSEL variant. The third slot of (a, b, c) is
    /// repurposed to carry APSR.NZCV bits via `c.to_bits()`.
    fn is_vsel(self) -> bool {
        matches!(self, Op::VselEq | Op::VselVs | Op::VselGe | Op::VselGt)
    }

    /// VSEL `cc` field for the encoder (EQ=0, VS=1, GE=2, GT=3).
    fn vsel_cc(self) -> u16 {
        match self {
            Op::VselEq => 0b00,
            Op::VselVs => 0b01,
            Op::VselGe => 0b10,
            Op::VselGt => 0b11,
            _ => panic!("vsel_cc on non-VSEL op {}", self.name()),
        }
    }

    /// True if the op writes FPSCR.NZCV instead of an Sd value (VCMP family).
    /// Compare-shaped ops are dispatched via `run_vcmp_one`; the rest go
    /// through `run_single`.
    fn writes_nzcv(self) -> bool {
        matches!(self, Op::Vcmp | Op::Vcmpe)
    }

    /// True if the op's f32 result must be compared bit-exact regardless
    /// of whether the bits encode a NaN. Two cases:
    /// - VCVT-to-int: the result encodes an integer; saturation values
    ///   like `u32::MAX` happen to look like NaN bits, so the NaN-vs-NaN
    ///   equivalence shortcut would mask saturation divergences.
    /// - VSEL: the result is one of the input operands verbatim. A
    ///   mutation in `vsel_condition_holds` could pick the wrong operand
    ///   and still be classified "match" if both operands are NaN.
    fn integer_result_bits(self) -> bool {
        matches!(
            self,
            Op::VcvtUR
                | Op::VcvtU
                | Op::VcvtSR
                | Op::VcvtS
                | Op::VselEq
                | Op::VselVs
                | Op::VselGe
                | Op::VselGt
        )
    }

    /// True if the op's behavior depends on FPSCR.RMode (so the runner sweeps
    /// all four rmodes). VRINTZ ignores RMode (always RZ); arithmetic ops
    /// produce identical results across rmodes for the operand classes the
    /// fuzz harness exercises (we don't probe rounding boundaries here).
    fn rmode_sensitive(self) -> bool {
        matches!(self, Op::VRintR | Op::VRintX | Op::VcvtUR | Op::VcvtSR)
    }
}

struct Discrepancy {
    op: Op,
    mode_label: &'static str,
    fpscr_mode: u32,
    a: f32,
    b: f32,
    c: f32, // addend for FMA, ignored otherwise
    emu_result_bits: u32,
    ref_result_bits: u32,
    emu_flags: u32,
    ref_flags: u32,
}

impl std::fmt::Display for Discrepancy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} [{} fpscr_in=0x{:08X}] a=0x{:08X}({}) b=0x{:08X}({}) c=0x{:08X}({}) | \
             emu result=0x{:08X} flags=0x{:02X} | \
             ref result=0x{:08X} flags=0x{:02X}",
            self.op.name(),
            self.mode_label,
            self.fpscr_mode,
            self.a.to_bits(),
            self.a,
            self.b.to_bits(),
            self.b,
            self.c.to_bits(),
            self.c,
            self.emu_result_bits,
            self.emu_flags,
            self.ref_result_bits,
            self.ref_flags,
        )
    }
}

fn run_single(
    op: Op,
    a: f32,
    b: f32,
    c: f32,
    fpscr_mode: u32,
    mode_label: &'static str,
) -> Option<Discrepancy> {
    // Emulator side.
    let mut emu = CortexM33::default();
    // Pre-load FPSCR with the control bits for this mode. Cumulative
    // exception flags start zero so we can read them cleanly after the op.
    emu.regs.fpscr = fpscr_mode;
    emu.regs.s[2] = a;
    emu.regs.s[4] = b;
    // For FMA, the accumulator is in Sd itself (pre-op).
    // For VSEL, `c` carries APSR.NZCV bits (positions 31/30/29/28); the
    // s[0] pre-load is harmless (VSEL writes Sd from the chosen operand).
    emu.regs.s[0] = c;
    if op.is_vsel() {
        emu.regs.xpsr = c.to_bits() & 0xF000_0000;
    }

    let (hw0, hw1) = match op {
        Op::Add => enc_vadd(0, 2, 4),
        Op::Sub => enc_vsub(0, 2, 4),
        Op::Mul => enc_vmul(0, 2, 4),
        Op::Div => enc_vdiv(0, 2, 4),
        Op::Sqrt => enc_vsqrt(0, 2),
        Op::Fma => enc_vfma(0, 2, 4),
        Op::VRintR => enc_vrintr(0, 2),
        Op::VRintX => enc_vrintx(0, 2),
        Op::VRintZ => enc_vrintz(0, 2),
        Op::Vabs => enc_vabs(0, 2),
        Op::Vneg => enc_vneg(0, 2),
        Op::Vminnm => enc_vminnm(0, 2, 4),
        Op::Vmaxnm => enc_vmaxnm(0, 2, 4),
        Op::Vcmp | Op::Vcmpe => unreachable!("VCMP routed via run_vcmp_one"),
        Op::VcvtFU => enc_vcvt_f32_u32(0, 2),
        Op::VcvtFS => enc_vcvt_f32_s32(0, 2),
        Op::VcvtUR => enc_vcvtr_u32_f32(0, 2),
        Op::VcvtU => enc_vcvt_u32_f32(0, 2),
        Op::VcvtSR => enc_vcvtr_s32_f32(0, 2),
        Op::VcvtS => enc_vcvt_s32_f32(0, 2),
        Op::VselEq | Op::VselVs | Op::VselGe | Op::VselGt => enc_vsel(op.vsel_cc(), 0, 2, 4),
    };
    emu.execute_one_wide(hw0, hw1);
    let emu_result = emu.regs.s[0];
    let emu_flags = emu.regs.fpscr & FPSCR_FLAG_MASK;

    // Reference side. VRINT variants take the rmode encoded in fpscr_mode
    // bits [23:22] — the FPSCR.RMode field.
    let rmode = (fpscr_mode >> 22) & 0x3;
    let (ref_result, ref_flags) = match op {
        Op::Add => ieee754_ref::ref_add(a, b, fpscr_mode),
        Op::Sub => ieee754_ref::ref_sub(a, b, fpscr_mode),
        Op::Mul => ieee754_ref::ref_mul(a, b, fpscr_mode),
        Op::Div => ieee754_ref::ref_div(a, b, fpscr_mode),
        Op::Sqrt => ieee754_ref::ref_sqrt(a, fpscr_mode),
        Op::Fma => ieee754_ref::ref_fma(a, b, c, fpscr_mode),
        Op::VRintR => ieee754_ref::ref_vrint(a, rmode, fpscr_mode, false),
        Op::VRintX => ieee754_ref::ref_vrint(a, rmode, fpscr_mode, true),
        // VRINTZ ignores FPSCR.RMode — always RZ (rmode=0b11).
        Op::VRintZ => ieee754_ref::ref_vrint(a, 0b11, fpscr_mode, false),
        Op::Vabs => ieee754_ref::ref_vabs(a, fpscr_mode),
        Op::Vneg => ieee754_ref::ref_vneg(a, fpscr_mode),
        Op::Vminnm => ieee754_ref::ref_vminnm(a, b, fpscr_mode),
        Op::Vmaxnm => ieee754_ref::ref_vmaxnm(a, b, fpscr_mode),
        Op::Vcmp | Op::Vcmpe => unreachable!("VCMP routed via run_vcmp_one"),
        Op::VcvtFU => ieee754_ref::ref_vcvt_f32_from_u32(a, fpscr_mode),
        Op::VcvtFS => ieee754_ref::ref_vcvt_f32_from_s32(a, fpscr_mode),
        Op::VcvtUR => ieee754_ref::ref_vcvt_u32_rmode(a, rmode, fpscr_mode),
        Op::VcvtU => ieee754_ref::ref_vcvt_u32_rtz(a, fpscr_mode),
        Op::VcvtSR => ieee754_ref::ref_vcvt_s32_rmode(a, rmode, fpscr_mode),
        Op::VcvtS => ieee754_ref::ref_vcvt_s32_rtz(a, fpscr_mode),
        Op::VselEq | Op::VselVs | Op::VselGe | Op::VselGt => {
            let apsr = c.to_bits();
            let n_flag = (apsr & 0x8000_0000) != 0;
            let z_flag = (apsr & 0x4000_0000) != 0;
            let v_flag = (apsr & 0x1000_0000) != 0;
            (
                ieee754_ref::ref_vsel(op.vsel_cc(), n_flag, z_flag, v_flag, a, b),
                0,
            )
        }
    };

    // Compare results. With DN=0, two NaN results match regardless of bits
    // (payload propagation is not contracted — HLD §A.3). With DN=1 both
    // sides are required to emit the canonical 0x7FC0_0000, so we keep
    // strict bit-equality in that case.
    //
    // VCVT-to-int family bypass: their f32 result encodes an integer (the
    // emulator stores `f32::from_bits(int_result)`), and integer bit
    // patterns can incidentally land on NaN-encoded bits (e.g. u32::MAX =
    // 0xFFFFFFFF is a NaN). We require strict bit-equality regardless of
    // NaN-ness for those ops.
    let results_match = if op.integer_result_bits() {
        emu_result.to_bits() == ref_result.to_bits()
    } else if emu_result.is_nan() && ref_result.is_nan() {
        if fpscr_mode & ieee754_ref::DN != 0 {
            emu_result.to_bits() == ref_result.to_bits()
        } else {
            true
        }
    } else {
        emu_result.to_bits() == ref_result.to_bits()
    };
    let flags_match = emu_flags == ref_flags;
    if results_match && flags_match {
        None
    } else {
        Some(Discrepancy {
            op,
            mode_label,
            fpscr_mode,
            a,
            b,
            c,
            emu_result_bits: emu_result.to_bits(),
            ref_result_bits: ref_result.to_bits(),
            emu_flags,
            ref_flags,
        })
    }
}

/// Top-level dispatch: VCMP-family ops route to `run_vcmp_one`,
/// everything else goes through `run_single`.
fn run_one(
    op: Op,
    a: f32,
    b: f32,
    c: f32,
    fpscr_mode: u32,
    mode_label: &'static str,
) -> Option<Discrepancy> {
    if op.writes_nzcv() {
        run_vcmp_one(op, a, b, fpscr_mode, mode_label)
    } else {
        run_single(op, a, b, c, fpscr_mode, mode_label)
    }
}

/// Dedicated runner for VCMP / VCMPE. Output is FPSCR.NZCV (bits 31..=28),
/// not an Sd register write — so the existing `run_single` shape doesn't
/// fit. Uses Sd=s[2] (pre-loaded with `a`) and Sm=s[4] (`b`), mirroring
/// `run_single`'s operand layout.
fn run_vcmp_one(
    op: Op,
    a: f32,
    b: f32,
    fpscr_mode: u32,
    mode_label: &'static str,
) -> Option<Discrepancy> {
    let mut emu = CortexM33::default();
    emu.regs.fpscr = fpscr_mode;
    emu.regs.s[2] = a;
    emu.regs.s[4] = b;
    let (hw0, hw1) = match op {
        Op::Vcmp => enc_vcmp(2, 4),
        Op::Vcmpe => enc_vcmpe(2, 4),
        _ => unreachable!("run_vcmp_one called with non-VCMP op {:?}", op.name()),
    };
    emu.execute_one_wide(hw0, hw1);
    let emu_nzcv = emu.regs.fpscr & 0xF000_0000;
    let emu_flags = emu.regs.fpscr & FPSCR_FLAG_MASK;

    let (ref_nzcv, ref_flags) = ieee754_ref::ref_vcmp(a, b, fpscr_mode);

    if emu_nzcv == ref_nzcv && emu_flags == ref_flags {
        None
    } else {
        Some(Discrepancy {
            op,
            mode_label,
            fpscr_mode,
            a,
            b,
            c: 0.0,
            // Pack the NZCV nibble into the result-bits fields so the
            // existing Discrepancy display can show `emu nzcv=0x...` /
            // `ref nzcv=0x...`. Reader interprets these as nibble-only.
            emu_result_bits: emu_nzcv,
            ref_result_bits: ref_nzcv,
            emu_flags,
            ref_flags,
        })
    }
}

/// Generate a random f32 drawn from a mix of edge cases and uniform bits.
fn random_f32(rng: &mut StdRng) -> f32 {
    // 25% chance of an edge-case value; 75% uniform random bits.
    let roll = rng.gen_range(0..20);
    match roll {
        0 => 0.0,
        1 => -0.0,
        2 => f32::INFINITY,
        3 => f32::NEG_INFINITY,
        4 => f32::from_bits(0x7FC0_0000), // qNaN
        5 => f32::from_bits(0x7F80_0001), // sNaN
        6 => f32::MIN_POSITIVE,           // smallest normal
        7 => f32::from_bits(0x0000_0001), // smallest subnormal
        8 => f32::from_bits(0x007F_FFFF), // largest subnormal
        9 => f32::MAX,
        10 => -f32::MAX,
        11 => 1.0,
        12 => -1.0,
        13 => {
            // Small magnitude, can trigger underflow when multiplied.
            let bits = rng.r#gen::<u32>() & 0x0FFF_FFFF;
            f32::from_bits(bits | 0x1800_0000) // ~1e-12 magnitude
        }
        14 => {
            // Large magnitude, can trigger overflow when multiplied.
            let bits = rng.r#gen::<u32>() & 0x0FFF_FFFF;
            f32::from_bits(bits | 0x6800_0000) // ~1e+24 magnitude
        }
        _ => f32::from_bits(rng.r#gen::<u32>()),
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    mdpicoem_harness::harness_tracing_init();
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };

    let seed = args.seed.unwrap_or_else(|| {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64
    });

    let mut any_fail = false;
    if args.mode == Mode::Fpu || args.mode == Mode::All {
        if !run_fpu(args.fuzz_count, seed) {
            any_fail = true;
        }
        if !run_vcvt_half(args.fuzz_count, seed) {
            any_fail = true;
        }
    }
    if (args.mode == Mode::Dcp || args.mode == Mode::All) && !run_dcp(args.fuzz_count, seed) {
        any_fail = true;
    }
    if any_fail {
        std::process::exit(1);
    }
}

/// Iterate FPSCR modes for a given op. For RMode-sensitive ops (VRINTR/X) we
/// emit four modes per base FZ/DN combo (one per RMode); for everything else
/// the base mode is sufficient.
fn fpscr_modes_for(op: Op) -> Vec<(u32, String)> {
    let mut out = Vec::new();
    if op.rmode_sensitive() {
        for &(base, label) in &FPSCR_MODES {
            for rmode in 0..4u32 {
                let combined = base | (rmode << 22);
                let rmode_label = match rmode {
                    0 => "RN",
                    1 => "RP",
                    2 => "RM",
                    _ => "RZ",
                };
                out.push((combined, format!("{label} {rmode_label}")));
            }
        }
    } else {
        for &(base, label) in &FPSCR_MODES {
            out.push((base, label.to_string()));
        }
    }
    out
}

/// FPU mode runner — returns true on all-pass.
fn run_fpu(fuzz_count: Option<usize>, seed: u64) -> bool {
    let ops = [
        Op::Add,
        Op::Sub,
        Op::Mul,
        Op::Div,
        Op::Sqrt,
        Op::Fma,
        Op::VRintR,
        Op::VRintX,
        Op::VRintZ,
        Op::Vabs,
        Op::Vneg,
        Op::Vminnm,
        Op::Vmaxnm,
        Op::Vcmp,
        Op::Vcmpe,
        Op::VcvtFU,
        Op::VcvtFS,
        Op::VcvtUR,
        Op::VcvtU,
        Op::VcvtSR,
        Op::VcvtS,
        Op::VselEq,
        Op::VselVs,
        Op::VselGe,
        Op::VselGt,
    ];

    match fuzz_count {
        None => {
            let cases = edge_cases();
            let mut fail = 0usize;
            let mut total = 0usize;
            for (op, a, b, c) in cases {
                for (mode, label) in fpscr_modes_for(op) {
                    total += 1;
                    // Leak the label string as &'static str — runner's lifetime
                    // is the whole process; cheap and avoids threading a
                    // lifetime through Discrepancy.
                    let label_static: &'static str = Box::leak(label.into_boxed_str());
                    if let Some(d) = run_one(op, a, b, c, mode, label_static) {
                        eprintln!("[FAIL] {d}");
                        fail += 1;
                    }
                }
            }
            println!(
                "[FPU] Edge cases: {}/{} passed (FPSCR modes vary per op)",
                total - fail,
                total
            );
            fail == 0
        }
        Some(count) => {
            // Total count is a rough upper bound for the print; per-op modes
            // vary (VRINT* multiply by 4 rmodes).
            let total_modes: usize = ops.iter().map(|&o| fpscr_modes_for(o).len()).sum();
            println!(
                "[FPU] Fuzz: {count} tests/op/mode, {} (op,mode) cells, seed={seed}\n\
                 (reproduce: softfloat_diff --fuzz {count} --seed {seed})",
                total_modes
            );
            let mut rng = StdRng::seed_from_u64(seed);
            let mut fail = 0usize;
            let mut total = 0usize;
            for op in ops {
                let arity = op.arity();
                for (mode, label) in fpscr_modes_for(op) {
                    let label_static: &'static str = Box::leak(label.into_boxed_str());
                    for _ in 0..count {
                        total += 1;
                        let a = random_f32(&mut rng);
                        let b = if arity >= 2 {
                            random_f32(&mut rng)
                        } else {
                            0.0
                        };
                        let c = if arity >= 3 {
                            random_f32(&mut rng)
                        } else {
                            0.0
                        };
                        if let Some(d) = run_one(op, a, b, c, mode, label_static) {
                            if fail < 20 {
                                eprintln!("[FPU FAIL] {d}");
                            }
                            fail += 1;
                        }
                    }
                }
            }
            println!("[FPU] Fuzz: {}/{} passed", total - fail, total);
            if fail >= 20 {
                eprintln!(
                    "[FPU] (suppressed output for {} additional failures)",
                    fail - 20
                );
            }
            fail == 0
        }
    }
}

// ============================================================================
// Half-precision VCVT runner — separate from `run_fpu` because the I/O types
// (u16 in/out) don't fit the f32-only `Op` enum cleanly.
// ============================================================================

#[derive(Clone, Copy)]
enum HalfDir {
    F32FromF16,
    F16FromF32,
}

impl HalfDir {
    fn name(self) -> &'static str {
        match self {
            HalfDir::F32FromF16 => "VCVTB.F32.F16",
            HalfDir::F16FromF32 => "VCVTB.F16.F32",
        }
    }
}

/// Generate a random f16 bit pattern, mixing edge cases with uniform bits.
fn random_f16_bits(rng: &mut StdRng) -> u16 {
    let roll = rng.gen_range(0..16);
    match roll {
        0 => 0x0000,  // +0
        1 => 0x8000,  // -0
        2 => 0x7C00,  // +inf
        3 => 0xFC00,  // -inf
        4 => 0x7E00,  // canonical QNaN (default)
        5 => 0x7C01,  // SNaN
        6 => 0x0001,  // smallest subnormal
        7 => 0x03FF,  // largest subnormal
        8 => 0x0400,  // smallest normal
        9 => 0x7BFF,  // largest normal
        10 => 0x3C00, // +1.0
        11 => 0xBC00, // -1.0
        _ => rng.r#gen::<u16>(),
    }
}

/// Half-precision edge-case suite — returns f32 inputs for F32→F16 and
/// f16 bit patterns for F16→F32.
fn half_edge_cases_f32() -> Vec<f32> {
    let snan = f32::from_bits(0x7F80_0001);
    let qnan = f32::from_bits(0x7FCD_EAD0);
    let denorm = f32::from_bits(0x0000_0001);
    vec![
        0.0,
        -0.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        snan,
        qnan,
        denorm,
        1.0,
        -1.0,
        65504.0, // largest f16 normal
        -65504.0,
        65536.0,                     // overflows to f16 inf
        f32::from_bits(0x387F_C000), // largest f16 subnormal as f32
        6.103_515_6e-5,              // smallest f16 normal (2^-14)
        f32::from_bits(0x33800000),  // smallest f16 subnormal (2^-24)
        1.0 / 3.0,                   // inexact
    ]
}

/// Targeted f32 inputs that exercise the round-half-to-even paths in
/// `f32_to_f16_bits` — both subnormal (lines 1516-1547 of execute_fpu.rs)
/// and normal (1549-1570). Constructed so that round_bit / sticky / lsb
/// at the f16-mantissa boundary vary independently, plus boundary cases
/// for the `rounded > 0x3FF` carry-into-exponent test at line 1563.
///
/// Closes the V3 Stream B Bucket 1 candidates flagged in V2 Triage HLD
/// §10.3:
///
/// - line 1534 `mantissa & 1` (subnormal lsb)
/// - line 1536 `if round_bit != 0 && (sticky || lsb != 0)` (round trigger)
/// - line 1553 `(frac & 0xFFF) != 0` (normal sticky)
/// - line 1554 `mantissa & 1` (normal lsb)
/// - line 1563 `if rounded > 0x3FF` (mantissa carry)
fn vcvt_f16_rounding_edge_cases_f32() -> Vec<f32> {
    let mut out = Vec::new();
    // ---------- Subnormal output (e ∈ [-24, -15]) ----------
    //
    // For each exponent in this range, build an f32 whose mantissa bits
    // place a controllable lsb at bit `shift` and round_bit at bit
    // `shift - 1` of the implicit-1-prepended `m = (1 << 23) | frac`.
    //
    // shift = -e - 1 ∈ [14, 23]. For shift=14 the f16 mantissa has 9
    // independently-varying bits; for shift=23 it has zero (mantissa is
    // always 1 from the implicit leading bit). Sweep lsb / round_bit /
    // sticky independently across `shift ∈ {14, 17, 20}` to hit a
    // representative spread.
    for &e in &[-15i32, -18, -21] {
        let exp = (e + 127) as u32;
        let shift = (-e - 1) as u32;
        // Build masks within the 23-bit frac field (bit 23 is implicit-1
        // and not part of frac).
        // - lsb_bit at bit `shift` of m → bit `shift` of frac (since shift < 23)
        // - round_bit at bit `shift - 1` of m → bit `shift - 1` of frac
        // - sticky covers bits 0..(shift - 1) of m → same in frac
        for &(round_bit, sticky_bit, lsb) in &[
            (0u32, 0u32, 0u32),
            (0, 0, 1),
            (0, 1, 0),
            (0, 1, 1),
            (1, 0, 0),
            (1, 0, 1), // line 1534/1536: original rounds-up, mut1534 doesn't
            (1, 1, 0), // line 1536: round-up via sticky alone
            (1, 1, 1),
        ] {
            let mut frac: u32 = 0;
            if lsb == 1 && shift < 23 {
                frac |= 1 << shift;
            }
            if round_bit == 1 && shift >= 1 {
                frac |= 1 << (shift - 1);
            }
            if sticky_bit == 1 && shift >= 2 {
                // Set bit 0 only — guarantees sticky is true.
                frac |= 1;
            }
            let bits = (exp << 23) | (frac & 0x7F_FFFF);
            out.push(f32::from_bits(bits));
            // Sign-mirrored variant for free coverage of the sign-bit
            // assembly path at lines 1488/1546.
            out.push(f32::from_bits(bits | 0x8000_0000));
        }
    }

    // ---------- Normal output, generic round-half-to-even spread ----------
    //
    // For e ∈ {-10, 0, 10} (representative normal range), vary round_bit
    // (frac bit 12), sticky bits (frac bits 0..=11), and the f16
    // mantissa lsb (frac bit 13).
    for &e in &[-10i32, 0, 10] {
        let exp = (e + 127) as u32;
        for &(round_bit, sticky_bit, lsb) in &[
            (0u32, 0u32, 0u32),
            (0, 0, 1),
            (0, 1, 0),
            (1, 0, 0), // mantissa unchanged (round_bit set but no carry)
            (1, 0, 1), // line 1554/1536: round-up via lsb only
            (1, 1, 0), // line 1553: round-up via sticky only
            (1, 1, 1),
        ] {
            let mut frac: u32 = 0;
            if lsb == 1 {
                frac |= 1 << 13;
            }
            if round_bit == 1 {
                frac |= 1 << 12;
            }
            if sticky_bit == 1 {
                // Set bit 0 — guarantees sticky non-zero. Bit 0 is in
                // the lower 12 of frac so it doesn't cross the round_bit
                // / lsb positions.
                frac |= 1;
            }
            // Set the upper f16-mantissa bits so we don't sit on
            // mantissa = 0 (which is a special path).
            frac |= 0xA << 19; // frac[22:19] = 1010, gives mantissa[9:6] = 1010
            let bits = (exp << 23) | (frac & 0x7F_FFFF);
            out.push(f32::from_bits(bits));
        }
    }

    // ---------- Normal-branch carry boundary (line 1563) ----------
    //
    // `if rounded > 0x3FF` carries the result into the next exponent.
    // The mutation `> → >=` differs when `rounded == 0x3FF` exactly, so
    // we need (a) cases where `rounded == 0x3FF` and (b) cases where
    // `rounded == 0x3FE` (just below) and `rounded == 0x400` (just
    // above) to disambiguate.
    //
    // Construct frac so `mantissa = frac >> 13` lands on 0x3FF (or
    // 0x3FE rounded up to 0x3FF, etc).
    for &e in &[0i32, 5, -10] {
        let exp = (e + 127) as u32;
        // mantissa = 0x3FF, round_bit = 0, sticky = 0 → rounded = 0x3FF.
        // Line 1563 takes the false branch; mutation `>=` takes true.
        let frac_3ff_no_round = (0x3FFu32) << 13;
        out.push(f32::from_bits(
            (exp << 23) | (frac_3ff_no_round & 0x7F_FFFF),
        ));
        // mantissa = 0x3FE, round_bit = 1, sticky = 1 → rounded =
        // 0x3FF. Same boundary, different path in.
        let frac_3fe_round_sticky = ((0x3FEu32) << 13) | (1 << 12) | 1;
        out.push(f32::from_bits(
            (exp << 23) | (frac_3fe_round_sticky & 0x7F_FFFF),
        ));
        // mantissa = 0x3FF, round_bit = 1, sticky = 1 → rounded =
        // 0x400. Original carries; mutation also carries (>= 0x3FF
        // is also true). No divergence here, but useful coverage.
        let frac_3ff_round_sticky = ((0x3FFu32) << 13) | (1 << 12) | 1;
        out.push(f32::from_bits(
            (exp << 23) | (frac_3ff_round_sticky & 0x7F_FFFF),
        ));
    }

    // ---------- Normal-branch sticky-zero corner (line 1553) ----------
    //
    // The mutation `(frac & 0xFFF) != 0` → `(frac ^ 0xFFF) != 0` flips
    // sticky from false to true unless `frac & 0xFFF == 0xFFF`. Generate
    // f32 where `frac & 0xFFF == 0` exactly so original sticky is false
    // and the mutation makes it true.
    for &e in &[0i32, -5, 5] {
        let exp = (e + 127) as u32;
        // frac with lower 12 bits zero. Set round_bit = 1, lsb = 0 so
        // original (with sticky=false, lsb=0) does NOT round up; mutation
        // (with sticky=true) DOES round up. Detectable.
        let frac = ((0x300u32) << 13) | (1 << 12);
        out.push(f32::from_bits((exp << 23) | (frac & 0x7F_FFFF)));
    }

    out
}

fn half_edge_cases_f16() -> Vec<u16> {
    vec![
        0x0000, 0x8000, // ±0
        0x7C00, 0xFC00, // ±inf
        0x7E00, // canonical QNaN
        0x7C01, 0x7DFF, // SNaNs
        0x7E01, 0x7FFF, // QNaNs (with payload)
        0x0001, 0x03FF, // ±subnormals
        0x8001, 0x83FF, 0x0400, 0x3C00, 0xBC00, // smallest normal, ±1.0
        0x7BFF, 0xFBFF, // largest ±normals
    ]
}

fn run_one_half(dir: HalfDir, fpscr_mode: u32, mode_label: &str, f32_in: f32, f16_in: u16) -> bool {
    let mut emu = CortexM33::default();
    emu.regs.fpscr = fpscr_mode;

    match dir {
        HalfDir::F16FromF32 => {
            // Pre-set Sd=0 so the emulator's "preserve top half" path leaves
            // the top half zero, simplifying the comparison.
            emu.regs.s[0] = 0.0;
            emu.regs.s[2] = f32_in;
            let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
            emu.execute_one_wide(hw0, hw1);
            let emu_half = (emu.regs.s[0].to_bits() & 0xFFFF) as u16;
            let emu_flags = emu.regs.fpscr & FPSCR_FLAG_MASK;

            let (ref_half, ref_flags) = ieee754_ref::ref_vcvt_f16_from_f32(f32_in, fpscr_mode);

            let result_match = if is_f16_nan(emu_half) && is_f16_nan(ref_half) {
                if fpscr_mode & ieee754_ref::DN != 0 {
                    emu_half == ref_half
                } else {
                    true
                }
            } else {
                emu_half == ref_half
            };
            if result_match && emu_flags == ref_flags {
                return true;
            }
            eprintln!(
                "[FPU FAIL] {} [{} fpscr_in=0x{:08X}] in=0x{:08X}({}) | \
                 emu=0x{:04X} flags=0x{:02X} | ref=0x{:04X} flags=0x{:02X}",
                dir.name(),
                mode_label,
                fpscr_mode,
                f32_in.to_bits(),
                f32_in,
                emu_half,
                emu_flags,
                ref_half,
                ref_flags
            );
            false
        }
        HalfDir::F32FromF16 => {
            // Pack the f16 into the bottom half of S2.
            emu.regs.s[2] = f32::from_bits(f16_in as u32);
            let (hw0, hw1) = enc_vcvtb_f32_f16(0, 2);
            emu.execute_one_wide(hw0, hw1);
            let emu_result = emu.regs.s[0];
            let emu_flags = emu.regs.fpscr & FPSCR_FLAG_MASK;

            let (ref_result, ref_flags) = ieee754_ref::ref_vcvt_f32_from_f16(f16_in, fpscr_mode);

            let result_match = if emu_result.is_nan() && ref_result.is_nan() {
                if fpscr_mode & ieee754_ref::DN != 0 {
                    emu_result.to_bits() == ref_result.to_bits()
                } else {
                    true
                }
            } else {
                emu_result.to_bits() == ref_result.to_bits()
            };
            if result_match && emu_flags == ref_flags {
                return true;
            }
            eprintln!(
                "[FPU FAIL] {} [{} fpscr_in=0x{:08X}] in=0x{:04X} | \
                 emu=0x{:08X}({}) flags=0x{:02X} | ref=0x{:08X}({}) flags=0x{:02X}",
                dir.name(),
                mode_label,
                fpscr_mode,
                f16_in,
                emu_result.to_bits(),
                emu_result,
                emu_flags,
                ref_result.to_bits(),
                ref_result,
                ref_flags
            );
            false
        }
    }
}

fn is_f16_nan(h: u16) -> bool {
    let exp = (h >> 10) & 0x1F;
    let frac = h & 0x3FF;
    exp == 0x1F && frac != 0
}

fn run_vcvt_half(fuzz_count: Option<usize>, seed: u64) -> bool {
    let mut all_pass = true;
    match fuzz_count {
        None => {
            let mut total = 0usize;
            let mut fail = 0usize;
            // F16→F32 edge cases.
            for &h in &half_edge_cases_f16() {
                for (mode, label) in FPSCR_MODES {
                    total += 1;
                    if !run_one_half(HalfDir::F32FromF16, mode, label, 0.0, h) {
                        fail += 1;
                    }
                }
            }
            // F32→F16 edge cases.
            for &v in &half_edge_cases_f32() {
                for (mode, label) in FPSCR_MODES {
                    total += 1;
                    if !run_one_half(HalfDir::F16FromF32, mode, label, v, 0) {
                        fail += 1;
                    }
                }
            }
            // F32→F16 round-half-to-even targeted cases (subnormal +
            // normal-branch boundary). Closes V3 Bucket 1 candidates at
            // f32_to_f16_bits lines 1534/1536/1553/1554/1563.
            for &v in &vcvt_f16_rounding_edge_cases_f32() {
                for (mode, label) in FPSCR_MODES {
                    total += 1;
                    if !run_one_half(HalfDir::F16FromF32, mode, label, v, 0) {
                        fail += 1;
                    }
                }
            }
            println!(
                "[FPU] VCVT.F16 edge cases: {}/{} passed",
                total - fail,
                total
            );
            if fail != 0 {
                all_pass = false;
            }
        }
        Some(count) => {
            let mut rng = StdRng::seed_from_u64(seed.wrapping_add(0xCAFE));
            let mut total = 0usize;
            let mut fail = 0usize;
            // F16 → F32
            for (mode, label) in FPSCR_MODES {
                for _ in 0..count {
                    total += 1;
                    let h = random_f16_bits(&mut rng);
                    if !run_one_half(HalfDir::F32FromF16, mode, label, 0.0, h) {
                        if fail < 20 { /* already printed */ }
                        fail += 1;
                    }
                }
            }
            // F32 → F16, deterministic round-half-to-even targeted
            // sweep (run once per FPSCR mode regardless of fuzz count
            // — small constant cost, ~100 cases × 4 modes).
            let rounding_cases = vcvt_f16_rounding_edge_cases_f32();
            for (mode, label) in FPSCR_MODES {
                for &v in &rounding_cases {
                    total += 1;
                    if !run_one_half(HalfDir::F16FromF32, mode, label, v, 0) {
                        fail += 1;
                    }
                }
            }
            // F32 → F16, random fuzz.
            for (mode, label) in FPSCR_MODES {
                for _ in 0..count {
                    total += 1;
                    let v = random_f32(&mut rng);
                    if !run_one_half(HalfDir::F16FromF32, mode, label, v, 0) {
                        fail += 1;
                    }
                }
            }
            println!("[FPU] VCVT.F16 fuzz: {}/{} passed", total - fail, total);
            if fail != 0 {
                all_pass = false;
            }
        }
    }
    all_pass
}

// ============================================================================
// DCP (CP4/5) differential runner — Phase 7 Stage D
// ============================================================================

#[derive(Clone, Copy)]
enum DcpOp {
    Add,
    Sub,
    Mul,
    Div,
    Sqrt,
}

impl DcpOp {
    fn name(self) -> &'static str {
        match self {
            DcpOp::Add => "dadd",
            DcpOp::Sub => "dsub",
            DcpOp::Mul => "dmul",
            DcpOp::Div => "ddiv",
            DcpOp::Sqrt => "dsqrt",
        }
    }
    /// The opc2 field in our DCP CDP encoding for opc1=0 (arithmetic).
    fn opc2(self) -> u16 {
        match self {
            DcpOp::Add => 0,
            DcpOp::Sub => 1,
            DcpOp::Mul => 2,
            DcpOp::Div => 3,
            DcpOp::Sqrt => 4,
        }
    }
    fn arity(self) -> usize {
        match self {
            DcpOp::Sqrt => 1,
            _ => 2,
        }
    }
}

/// Encode a DCP CDP instruction on CP4 (see `coprocessor.rs::dcp_cdp_family`).
///   hw0 = 0xEE00 | (opc1 << 4) | CRn
///   hw1 = (CRd << 12) | (coproc << 8) | (opc2 << 5) | CRm     (bit 4 = 0 → CDP)
fn enc_dcp_cdp(opc1: u16, opc2: u16, crd: u16, crn: u16, crm: u16) -> (u16, u16) {
    let hw0 = 0xEE00 | ((opc1 & 0xF) << 4) | (crn & 0xF);
    let hw1 = ((crd & 0xF) << 12) | (4 << 8) | ((opc2 & 0x7) << 5) | (crm & 0xF);
    (hw0, hw1)
}

/// Enable a coprocessor in CPACR. Phase 0b.1 Commit B moved the per-core
/// PPB onto `CortexM33.ppb`; the public API exposes CPACR via the
/// `enable_coprocessor` helper on CortexM33.
fn enable_cp(cpu: &mut CortexM33, coproc: u8) {
    cpu.enable_coprocessor(coproc);
}

struct DcpDiscrepancy {
    op: DcpOp,
    a: f64,
    b: f64,
    emu_result: f64,
    ref_result: f64,
    emu_status: u32,
    ref_status: u32,
}

impl std::fmt::Display for DcpDiscrepancy {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} a=0x{:016X}({}) b=0x{:016X}({}) | \
             emu=0x{:016X} status=0x{:02X} | ref=0x{:016X} status=0x{:02X}",
            self.op.name(),
            self.a.to_bits(),
            self.a,
            self.b.to_bits(),
            self.b,
            self.emu_result.to_bits(),
            self.emu_status,
            self.ref_result.to_bits(),
            self.ref_status,
        )
    }
}

fn run_dcp_single(op: DcpOp, a: f64, b: f64) -> Option<DcpDiscrepancy> {
    // Emulator side — load a, b into d[0], d[1]; execute into d[2].
    let mut emu = CortexM33::default();
    let mut bus = Bus::default();
    enable_cp(&mut emu, 4);
    emu.dcp_set_double(0, a);
    if op.arity() >= 2 {
        emu.dcp_set_double(1, b);
    }
    let (hw0, hw1) = enc_dcp_cdp(0, op.opc2(), 2, 0, 1);
    emu.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    let emu_result = emu.dcp_get_double(2);
    let emu_status = emu.dcp_get_status();

    // Reference side.
    let (ref_result, ref_status) = match op {
        DcpOp::Add => ieee754_ref::ref_dadd(a, b),
        DcpOp::Sub => ieee754_ref::ref_dsub(a, b),
        DcpOp::Mul => ieee754_ref::ref_dmul(a, b),
        DcpOp::Div => ieee754_ref::ref_ddiv(a, b),
        DcpOp::Sqrt => ieee754_ref::ref_dsqrt(a),
    };

    // NaN-vs-NaN: we don't contract payload bits (matches FPU handling).
    let result_match = if emu_result.is_nan() && ref_result.is_nan() {
        true
    } else {
        emu_result.to_bits() == ref_result.to_bits()
    };
    // When both sides produce a NaN, the status's N (sign) bit is
    // implementation-defined per IEEE-754. Compare status with N masked
    // out on NaN-vs-NaN.
    let status_mask = if emu_result.is_nan() && ref_result.is_nan() {
        !ieee754_ref::DCP_N
    } else {
        !0u32
    };
    let status_match = (emu_status & status_mask) == (ref_status & status_mask);

    if result_match && status_match {
        None
    } else {
        Some(DcpDiscrepancy {
            op,
            a,
            b,
            emu_result,
            ref_result,
            emu_status,
            ref_status,
        })
    }
}

fn random_f64(rng: &mut StdRng) -> f64 {
    let roll = rng.gen_range(0..20);
    match roll {
        0 => 0.0,
        1 => -0.0,
        2 => f64::INFINITY,
        3 => f64::NEG_INFINITY,
        4 => f64::NAN,
        5 => f64::MIN_POSITIVE,
        6 => f64::MAX,
        7 => -f64::MAX,
        8 => 1.0,
        9 => -1.0,
        10 => {
            // Small magnitude.
            let bits = rng.r#gen::<u64>() & 0x0FFF_FFFF_FFFF_FFFF;
            f64::from_bits(bits | 0x0100_0000_0000_0000_u64) // ~1e-307
        }
        11 => {
            // Large magnitude.
            let bits = rng.r#gen::<u64>() & 0x0FFF_FFFF_FFFF_FFFF;
            f64::from_bits(bits | 0x6E00_0000_0000_0000_u64) // ~1e+230
        }
        _ => f64::from_bits(rng.r#gen::<u64>()),
    }
}

/// DCP mode runner — returns true on all-pass.
fn run_dcp(fuzz_count: Option<usize>, seed: u64) -> bool {
    let ops = [DcpOp::Add, DcpOp::Sub, DcpOp::Mul, DcpOp::Div, DcpOp::Sqrt];

    match fuzz_count {
        None => {
            let mut fail = 0usize;
            let mut total = 0usize;
            for &op in &ops {
                for (a, b) in dcp_edge_cases(op) {
                    total += 1;
                    if let Some(d) = run_dcp_single(op, a, b) {
                        eprintln!("[DCP FAIL] {d}");
                        fail += 1;
                    }
                }
            }
            println!("[DCP] Edge cases: {}/{} passed", total - fail, total);
            fail == 0
        }
        Some(count) => {
            println!(
                "[DCP] Fuzz: {count} tests/op × 5 ops = {} total, seed={seed}",
                count * ops.len()
            );
            let mut rng = StdRng::seed_from_u64(seed);
            let mut fail = 0usize;
            let mut total = 0usize;
            for &op in &ops {
                let arity = op.arity();
                for _ in 0..count {
                    total += 1;
                    let a = random_f64(&mut rng);
                    let b = if arity >= 2 {
                        random_f64(&mut rng)
                    } else {
                        0.0
                    };
                    if let Some(d) = run_dcp_single(op, a, b) {
                        if fail < 20 {
                            eprintln!("[DCP FAIL] {d}");
                        }
                        fail += 1;
                    }
                }
            }
            println!("[DCP] Fuzz: {}/{} passed", total - fail, total);
            if fail >= 20 {
                eprintln!(
                    "[DCP] (suppressed output for {} additional failures)",
                    fail - 20
                );
            }
            fail == 0
        }
    }
}

/// Small targeted DCP edge-case list per op.
fn dcp_edge_cases(op: DcpOp) -> Vec<(f64, f64)> {
    let mut v = Vec::new();
    match op {
        DcpOp::Add | DcpOp::Sub => {
            v.extend_from_slice(&[
                (1.0, 2.0),
                (1.0, -1.0),
                (f64::INFINITY, f64::NEG_INFINITY),
                (f64::MAX, f64::MAX),
                (1e300, 1e300),
                (0.0, -0.0),
                (f64::NAN, 1.0),
                (1.0, 1e-16),
            ]);
        }
        DcpOp::Mul => {
            v.extend_from_slice(&[
                (2.0, 3.0),
                (1e200, 1e200),
                (1e-200, 1e-200),
                (f64::INFINITY, 0.0),
                (0.0, f64::INFINITY),
                (f64::NAN, 1.0),
                (1.0, f64::NAN),
                (f64::MAX, 0.5),
            ]);
        }
        DcpOp::Div => {
            v.extend_from_slice(&[
                (1.0, 3.0),
                (1.0, 0.0),
                (-1.0, 0.0),
                (0.0, 0.0),
                (f64::INFINITY, f64::INFINITY),
                (f64::NAN, 1.0),
                (f64::MAX, 0.5),
                (1.0, f64::MAX),
            ]);
        }
        DcpOp::Sqrt => {
            for a in [
                4.0,
                2.0,
                -1.0,
                0.0,
                -0.0,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NAN,
                1e200,
                1e-200,
            ] {
                v.push((a, 0.0));
            }
        }
    }
    v
}

/// Static suite of edge-case triplets: (op, a, b, c).
fn edge_cases() -> Vec<(Op, f32, f32, f32)> {
    let snan = f32::from_bits(0x7F80_0001);
    let qnan = f32::from_bits(0x7FC0_0000);
    let denorm = f32::from_bits(0x0000_0001);
    let max_sub = f32::from_bits(0x007F_FFFF);

    let mut v = Vec::new();
    // VADD
    for (a, b) in [
        (1.0, 2.0),
        (1.0, -1.0),
        (f32::INFINITY, f32::NEG_INFINITY),
        (f32::MAX, f32::MAX),
        (-f32::MAX, -f32::MAX),
        (1.0, 1e-8), // inexact
        (denorm, 1.0),
        (snan, 1.0),
        (qnan, 1.0),
        (0.0, -0.0),
    ] {
        v.push((Op::Add, a, b, 0.0));
    }
    // VSUB
    for (a, b) in [
        (1.0, 2.0),
        (f32::INFINITY, f32::INFINITY),
        (f32::NEG_INFINITY, f32::NEG_INFINITY),
        (f32::MAX, -f32::MAX),
        (snan, 1.0),
        (denorm, 0.0),
    ] {
        v.push((Op::Sub, a, b, 0.0));
    }
    // VMUL
    for (a, b) in [
        (2.0, 3.0),
        (1e20, 1e20),
        (1e-20, 1e-20),
        (f32::INFINITY, 0.0),
        (0.0, f32::INFINITY),
        (snan, 1.0),
        (max_sub, 2.0),
        (1.0, 1.0 / 3.0),
    ] {
        v.push((Op::Mul, a, b, 0.0));
    }
    // VDIV
    for (a, b) in [
        (1.0, 3.0),
        (1.0, 0.0),
        (-1.0, 0.0),
        (0.0, 0.0),
        (f32::INFINITY, f32::INFINITY),
        (snan, 1.0),
        (denorm, 2.0),
        (1.0, denorm),
        (f32::MAX, 0.5),
    ] {
        v.push((Op::Div, a, b, 0.0));
    }
    // VSQRT
    for a in [
        4.0,
        2.0,
        -1.0,
        0.0,
        -0.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        snan,
        denorm,
    ] {
        v.push((Op::Sqrt, a, 0.0, 0.0));
    }
    // VFMA: a*b + c
    for (a, b, c) in [
        (2.0f32, 3.0f32, 1.0f32),
        (f32::INFINITY, 0.0, 1.0),
        (f32::INFINITY, 1.0, f32::NEG_INFINITY),
        (snan, 1.0, 1.0),
        (1.0, 1.0, qnan),
        (1e20, 1e20, 1.0),
        (1e-20, 1e-20, 0.0),
        (1.0, 3.0, 0.0), // inexact product
    ] {
        v.push((Op::Fma, a, b, c));
    }

    // VRINTR / VRINTX / VRINTZ — share the input list; the runner sweeps
    // FPSCR.RMode for the rmode-sensitive variants.
    let vrint_inputs = [
        0.0f32,
        -0.0,
        1.0,
        -1.0,
        2.5,
        -2.5, // half-way (RN: 2.0/-2.0; RP: 3/-2; RM: 2/-3; RZ: 2/-2)
        1.5,
        -1.5,
        0.5,
        -0.5, // half-way at exponent boundary
        100.0,
        -100.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        snan,
        qnan,
        denorm,
        f32::from_bits(0x4B00_0001), // > 2^23, not exactly representable as integer
        f32::from_bits(0x3F7F_FFFF), // 0.999... — rounds up under RP, down under RZ
    ];
    for &val in &vrint_inputs {
        v.push((Op::VRintR, val, 0.0, 0.0));
        v.push((Op::VRintX, val, 0.0, 0.0));
        v.push((Op::VRintZ, val, 0.0, 0.0));
    }

    // VABS / VNEG — sign-bit operations, same input set covers all cases.
    let unary_sign_inputs = [
        0.0f32,
        -0.0,
        1.0,
        -1.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        snan,
        qnan,
        denorm,
        max_sub,
        f32::MIN_POSITIVE,
        -f32::MIN_POSITIVE,
        f32::MAX,
        -f32::MAX,
        // SNaN with negative sign bit — exercises sign-flip on payload-bearing NaN.
        f32::from_bits(0xFF80_0001),
    ];
    for &val in &unary_sign_inputs {
        v.push((Op::Vabs, val, 0.0, 0.0));
        v.push((Op::Vneg, val, 0.0, 0.0));
    }

    // VMINNM / VMAXNM — IEEE-754-2008 minNum/maxNum semantics. Coverage:
    // both NaN, exactly one NaN (Q + S), zero-sign rules, ordered pairs.
    let nm_inputs = [
        (1.0f32, 2.0),
        (2.0, 1.0),
        (1.0, 1.0),
        (-1.0, 1.0),
        (0.0, 0.0),
        (0.0, -0.0),
        (-0.0, 0.0),
        (-0.0, -0.0),
        (qnan, 1.0),
        (1.0, qnan),
        (qnan, qnan),
        (snan, 1.0), // SNaN sets IOC; result is the non-NaN operand
        (1.0, snan),
        (snan, qnan),
        (snan, snan),
        (f32::INFINITY, 1.0),
        (-f32::INFINITY, 1.0),
        (f32::INFINITY, -f32::INFINITY),
        (denorm, 0.0),
    ];
    for &(a, b) in &nm_inputs {
        v.push((Op::Vminnm, a, b, 0.0));
        v.push((Op::Vmaxnm, a, b, 0.0));
    }

    // VCMP / VCMPE — output is FPSCR.NZCV, not Sd. Coverage: ordered
    // pairs (less / equal / greater), unordered (NaN), zero-sign edge.
    // `run_vcmp_one` reads s[2] (Sd) and s[4] (Sm); the pre-load loop
    // in `run_single` would put `a` into s[2] and `b` into s[4].
    let cmp_inputs = [
        (1.0f32, 2.0), // less
        (2.0, 1.0),    // greater
        (1.0, 1.0),    // equal
        (0.0, -0.0),   // equal at zero (per IEEE)
        (-0.0, 0.0),   // ditto
        (0.0, 0.0),
        (qnan, 1.0), // unordered (LHS NaN)
        (1.0, qnan), // unordered (RHS NaN)
        (qnan, qnan),
        (snan, 1.0),
        (1.0, snan),
        (f32::INFINITY, 1.0),
        (-f32::INFINITY, 1.0),
        (f32::INFINITY, f32::NEG_INFINITY),
        (f32::INFINITY, f32::INFINITY),
        (denorm, 0.0),
    ];
    for &(a, b) in &cmp_inputs {
        v.push((Op::Vcmp, a, b, 0.0));
        v.push((Op::Vcmpe, a, b, 0.0));
    }

    // VCVT-to-int / VCVT-from-int — coverage: NaN, ±inf, ±zero, large
    // magnitudes near the i32/u32 saturation boundaries, fractional
    // half-way values, denormals.
    let vcvt_inputs = [
        0.0f32,
        -0.0,
        1.0,
        -1.0,
        2.5, // RN: 2; RP: 3; RM: 2; RZ: 2
        -2.5,
        1.5,
        -1.5,
        f32::INFINITY,
        f32::NEG_INFINITY,
        snan,
        qnan,
        denorm,
        // Boundary values for i32/u32 saturation. f32 can't represent
        // 2^31 exactly past the round-up, but close.
        2147483648.0,  // = 2^31, just over i32::MAX
        -2147483648.0, // = -2^31, exactly i32::MIN as f32
        4294967296.0,  // = 2^32, just over u32::MAX
        2147483520.0,  // = i32::MAX rounded down
        f32::MAX,      // very large
        -f32::MAX,
        // VCVT.F32.U32 / VCVT.F32.S32 inputs reinterpret f32 bits as
        // integer; pre-encode common integer values as f32-bits.
        f32::from_bits(0),
        f32::from_bits(1),
        f32::from_bits(0xFFFF_FFFF),
        f32::from_bits(0x8000_0000),
        f32::from_bits(0x7FFF_FFFF),
        f32::from_bits(0x4000_0000),
    ];
    for &val in &vcvt_inputs {
        v.push((Op::VcvtFU, val, 0.0, 0.0));
        v.push((Op::VcvtFS, val, 0.0, 0.0));
        v.push((Op::VcvtUR, val, 0.0, 0.0));
        v.push((Op::VcvtU, val, 0.0, 0.0));
        v.push((Op::VcvtSR, val, 0.0, 0.0));
        v.push((Op::VcvtS, val, 0.0, 0.0));
    }

    // VSEL — sweep all 16 APSR.NZCV combinations × representative (sn, sm)
    // pairs. The third slot `c` carries APSR-bits as f32::from_bits().
    // Operand pairs include NaN to verify the integer_result_bits check
    // catches a wrong-operand pick under DN=0.
    let vsel_operand_pairs = [
        (1.0f32, 2.0), // distinct finite
        (qnan, 1.0),   // NaN as sn
        (1.0, qnan),   // NaN as sm
        (snan, qnan),  // both NaN, different payloads
    ];
    for nzcv_bits in 0..16u32 {
        let apsr_bits = nzcv_bits << 28;
        let apsr_f32 = f32::from_bits(apsr_bits);
        for &(sn, sm) in &vsel_operand_pairs {
            v.push((Op::VselEq, sn, sm, apsr_f32));
            v.push((Op::VselVs, sn, sm, apsr_f32));
            v.push((Op::VselGe, sn, sm, apsr_f32));
            v.push((Op::VselGt, sn, sm, apsr_f32));
        }
    }

    v
}

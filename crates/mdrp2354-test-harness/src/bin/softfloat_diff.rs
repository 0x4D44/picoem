// FPU differential test harness — Phase 7 Stage A.1.
//
// Each test case pumps a random (or targeted) operand pair through:
//   1. Our emulator's VFP arithmetic path (via `CortexM33::execute_one_wide`).
//   2. The hand-rolled IEEE-754 reference in `mdrp2354_test_harness::ieee754_ref`.
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

use mdrp2354_test_harness::ieee754_ref;
use mdrp2354_test_harness::{Bus, CortexM33};
use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;

// Which six FPSCR bits we care about (cumulative exception flags).
const FPSCR_FLAG_MASK: u32 =
    (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 7);

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
    Ok(Args { fuzz_count, seed, mode })
}

// ============================================================================
// VFP instruction encoders (duplicated from mdrp2354 tests.rs — those are
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
        }
    }

    fn arity(self) -> usize {
        match self {
            Op::Sqrt => 1,
            Op::Fma => 3, // a, b, and accumulator
            _ => 2,
        }
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
    let mut emu = CortexM33::new();
    // Pre-load FPSCR with the control bits for this mode. Cumulative
    // exception flags start zero so we can read them cleanly after the op.
    emu.regs.fpscr = fpscr_mode;
    emu.regs.s[2] = a;
    emu.regs.s[4] = b;
    // For FMA, the accumulator is in Sd itself (pre-op).
    emu.regs.s[0] = c;

    let (hw0, hw1) = match op {
        Op::Add => enc_vadd(0, 2, 4),
        Op::Sub => enc_vsub(0, 2, 4),
        Op::Mul => enc_vmul(0, 2, 4),
        Op::Div => enc_vdiv(0, 2, 4),
        Op::Sqrt => enc_vsqrt(0, 2),
        Op::Fma => enc_vfma(0, 2, 4),
    };
    emu.execute_one_wide(hw0, hw1);
    let emu_result = emu.regs.s[0];
    let emu_flags = emu.regs.fpscr & FPSCR_FLAG_MASK;

    // Reference side.
    let (ref_result, ref_flags) = match op {
        Op::Add => ieee754_ref::ref_add(a, b, fpscr_mode),
        Op::Sub => ieee754_ref::ref_sub(a, b, fpscr_mode),
        Op::Mul => ieee754_ref::ref_mul(a, b, fpscr_mode),
        Op::Div => ieee754_ref::ref_div(a, b, fpscr_mode),
        Op::Sqrt => ieee754_ref::ref_sqrt(a, fpscr_mode),
        Op::Fma => ieee754_ref::ref_fma(a, b, c, fpscr_mode),
    };

    // Compare results. With DN=0, two NaN results match regardless of bits
    // (payload propagation is not contracted — HLD §A.3). With DN=1 both
    // sides are required to emit the canonical 0x7FC0_0000, so we keep
    // strict bit-equality in that case.
    let results_match = if emu_result.is_nan() && ref_result.is_nan() {
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

/// Generate a random f32 drawn from a mix of edge cases and uniform bits.
fn random_f32(rng: &mut StdRng) -> f32 {
    // 25% chance of an edge-case value; 75% uniform random bits.
    let roll = rng.gen_range(0..20);
    match roll {
        0 => 0.0,
        1 => -0.0,
        2 => f32::INFINITY,
        3 => f32::NEG_INFINITY,
        4 => f32::from_bits(0x7FC0_0000),  // qNaN
        5 => f32::from_bits(0x7F80_0001),  // sNaN
        6 => f32::MIN_POSITIVE,            // smallest normal
        7 => f32::from_bits(0x0000_0001),  // smallest subnormal
        8 => f32::from_bits(0x007F_FFFF),  // largest subnormal
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
    }
    if args.mode == Mode::Dcp || args.mode == Mode::All {
        if !run_dcp(args.fuzz_count, seed) {
            any_fail = true;
        }
    }
    if any_fail {
        std::process::exit(1);
    }
}

/// FPU mode runner — returns true on all-pass.
fn run_fpu(fuzz_count: Option<usize>, seed: u64) -> bool {
    let ops = [Op::Add, Op::Sub, Op::Mul, Op::Div, Op::Sqrt, Op::Fma];

    match fuzz_count {
        None => {
            let cases = edge_cases();
            let mut fail = 0usize;
            let mut total = 0usize;
            for (op, a, b, c) in cases {
                for (mode, label) in FPSCR_MODES {
                    total += 1;
                    if let Some(d) = run_single(op, a, b, c, mode, label) {
                        eprintln!("[FAIL] {d}");
                        fail += 1;
                    }
                }
            }
            println!("[FPU] Edge cases: {}/{} passed (across 4 FPSCR modes)", total - fail, total);
            fail == 0
        }
        Some(count) => {
            println!(
                "[FPU] Fuzz: {count} tests/op/mode × 4 FPSCR modes × 6 ops = {} total, seed={seed}\n\
                 (reproduce: softfloat_diff --fuzz {count} --seed {seed})",
                count * 4 * ops.len()
            );
            let mut rng = StdRng::seed_from_u64(seed);
            let mut fail = 0usize;
            let mut total = 0usize;
            for (mode, label) in FPSCR_MODES {
                for op in ops {
                    let arity = op.arity();
                    for _ in 0..count {
                        total += 1;
                        let a = random_f32(&mut rng);
                        let b = if arity >= 2 { random_f32(&mut rng) } else { 0.0 };
                        let c = if arity >= 3 { random_f32(&mut rng) } else { 0.0 };
                        if let Some(d) = run_single(op, a, b, c, mode, label) {
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
                eprintln!("[FPU] (suppressed output for {} additional failures)", fail - 20);
            }
            fail == 0
        }
    }
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

/// Enable a coprocessor in CPACR for core 0. Needed before any CP op.
fn enable_cp(bus: &mut Bus, coproc: u8) {
    let core = bus.active_core();
    bus.ppb[core].cpacr |= 0x3 << (coproc as u32 * 2);
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
    let mut emu = CortexM33::new();
    let mut bus = Bus::default();
    enable_cp(&mut bus, 4);
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
                    let b = if arity >= 2 { random_f64(&mut rng) } else { 0.0 };
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
                eprintln!("[DCP] (suppressed output for {} additional failures)", fail - 20);
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
    v
}

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
//   softfloat_diff                   Run edge-case tests
//   softfloat_diff --fuzz N          Run N random tests per instruction type
//   softfloat_diff --fuzz N --seed S Reproducible random run

use mdrp2354_test_harness::ieee754_ref;
use mdrp2354_test_harness::CortexM33;
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
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     softfloat_diff                   Run edge-case tests\n  \
                     softfloat_diff --fuzz N          Random tests per op type\n  \
                     softfloat_diff --fuzz N --seed S Reproducible random run"
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

    let ops = [Op::Add, Op::Sub, Op::Mul, Op::Div, Op::Sqrt, Op::Fma];

    match args.fuzz_count {
        None => {
            // Targeted edge-case suite — run each case under all 4 FPSCR modes.
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
            println!("Edge cases: {}/{} passed (across 4 FPSCR modes)", total - fail, total);
            if fail > 0 {
                std::process::exit(1);
            }
        }
        Some(count) => {
            let seed = args.seed.unwrap_or_else(|| {
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64
            });
            println!(
                "Fuzz mode: {count} tests/op/mode × 4 FPSCR modes × 6 ops = {} total, seed={seed}\n\
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
                                eprintln!("[FAIL] {d}");
                            }
                            fail += 1;
                        }
                    }
                }
            }
            println!("Fuzz: {}/{} passed", total - fail, total);
            if fail > 0 {
                if fail >= 20 {
                    eprintln!("(suppressed output for {} additional failures)", fail - 20);
                }
                std::process::exit(1);
            }
        }
    }
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

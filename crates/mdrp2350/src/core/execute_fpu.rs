// FPv5 single-precision FPU — Cortex-M33 coprocessor 10.
//
// CP10 = single-precision, CP11 = double-precision (not present on RP2350).
//
// Encoding classes (Thumb-2, hw0:hw1):
//   Data processing:  hw0[15:8]=0xEE, hw1[11:8]=0xA, hw1[4]=0
//   Register transfer: hw0[15:8]=0xEE, hw1[11:8]=0xA, hw1[4]=1
//   Load/store:        hw0[15:12]=0xE, hw0[11:9]=0b110, hw1[11:8]=0xA

use crate::bus::Bus;
use super::CortexM33;

// ============================================================================
// VFP register extraction
// ============================================================================
//
// Single-precision registers S0-S31 are encoded as 4-bit:1-bit pairs.
// In the 32-bit Thumb instruction (Inst[31:0] = hw0:hw1):
//   Sd = (Vd << 1) | D  where Vd=hw1[15:12], D=hw0[6]
//   Sn = (Vn << 1) | N  where Vn=hw0[3:0],   N=hw1[7]
//   Sm = (Vm << 1) | M  where Vm=hw1[3:0],    M=hw1[5]

#[inline(always)]
fn vfp_sd(hw0: u16, hw1: u16) -> usize {
    let vd = ((hw1 >> 12) & 0xF) as usize;
    let d = ((hw0 >> 6) & 1) as usize;
    (vd << 1) | d
}

#[inline(always)]
fn vfp_sn(hw0: u16, hw1: u16) -> usize {
    let vn = (hw0 & 0xF) as usize;
    let n = ((hw1 >> 7) & 1) as usize;
    (vn << 1) | n
}

#[inline(always)]
fn vfp_sm(hw1: u16) -> usize {
    let vm = (hw1 & 0xF) as usize;
    let m = ((hw1 >> 5) & 1) as usize;
    (vm << 1) | m
}

// ============================================================================
// FPSCR flag helpers
// ============================================================================

const FPSCR_N: u32 = 1 << 31;
const FPSCR_Z: u32 = 1 << 30;
const FPSCR_C: u32 = 1 << 29;
const FPSCR_V: u32 = 1 << 28;

fn fpscr_set_nzcv(fpscr: &mut u32, n: bool, z: bool, c: bool, v: bool) {
    *fpscr &= !(FPSCR_N | FPSCR_Z | FPSCR_C | FPSCR_V);
    if n { *fpscr |= FPSCR_N; }
    if z { *fpscr |= FPSCR_Z; }
    if c { *fpscr |= FPSCR_C; }
    if v { *fpscr |= FPSCR_V; }
}

#[inline(always)]
fn fpscr_rmode(fpscr: u32) -> u32 {
    (fpscr >> 22) & 0x3
}

// ----- Cumulative exception flags (Phase 7 Stage A.1, HLD §A.1) ------------
//
// All six flags are **sticky**: set by the op that triggered them, cleared
// only by VMSR to FPSCR. M33 has no trapped handling (FPSCR[15:8] RAZ/WI),
// so we accumulate only.
//
// Bit positions per DDI0553 §D1.2.88.

const FPSCR_IOC: u32 = 1 << 0;
const FPSCR_DZC: u32 = 1 << 1;
const FPSCR_OFC: u32 = 1 << 2;
const FPSCR_UFC: u32 = 1 << 3;
const FPSCR_IXC: u32 = 1 << 4;
const FPSCR_IDC: u32 = 1 << 7;

/// FZ (flush-to-zero) control bit. When set, denormal inputs are treated as
/// ±0 (+IDC) and tininess-before-rounding results are flushed (+UFC+IXC).
const FPSCR_FZ: u32 = 1 << 24;

/// DN (default NaN) control bit. When set, any NaN result becomes the canonical
/// quiet NaN 0x7FC0_0000.
const FPSCR_DN: u32 = 1 << 25;

/// Smallest positive normal f32 as f64 constant (2^-126).
///
/// Must match the `MIN_NORMAL` constant in
/// `crates/mdpicoem-harness/src/ieee754_ref.rs`. The two crates don't
/// share arithmetic helpers, so duplication is pragmatic — but any drift
/// would mis-classify underflow boundary cases in the differential oracle.
const F32_MIN_NORMAL_F64: f64 = 1.175_494_350_822_287_5e-38;

/// Returns true if `v` is a non-zero subnormal (denormal) f32.
#[inline]
fn is_denormal(v: f32) -> bool {
    let bits = v.to_bits();
    let exp = (bits >> 23) & 0xFF;
    let frac = bits & 0x007F_FFFF;
    exp == 0 && frac != 0
}

/// Apply FZ flush-to-zero to a denormal input. When FZ=1 and `v` is denormal,
/// returns a signed zero and sets IDC. Otherwise returns `v` unchanged (IDC
/// still accumulates on denormal input per ARM semantics).
#[inline]
fn ftz_input(fpscr: &mut u32, v: f32) -> f32 {
    if is_denormal(v) {
        *fpscr |= FPSCR_IDC;
        if *fpscr & FPSCR_FZ != 0 {
            return if v.is_sign_negative() { -0.0 } else { 0.0 };
        }
    }
    v
}

/// Apply FZ flush-to-zero to the *result* when the unrounded exact value was
/// tiny (|exact| < MIN_NORMAL) and the rounded f32 is subnormal (or zero after
/// inexact rounding from a subnormal). Sets UFC+IXC and returns signed zero.
/// Returns `None` if no flush is required, so the caller can keep the original
/// result and emit UFC via the standard underflow path.
#[inline]
fn ftz_output(fpscr: u32, result: f32, exact: f64) -> Option<f32> {
    if fpscr & FPSCR_FZ == 0 {
        return None;
    }
    // Don't flush NaN or infinity.
    if result.is_nan() || result.is_infinite() {
        return None;
    }
    // Tininess-before-rounding: the pre-rounding magnitude is below MIN_NORMAL.
    if exact.abs() >= F32_MIN_NORMAL_F64 || exact == 0.0 {
        return None;
    }
    Some(if result.is_sign_negative() { -0.0 } else { 0.0 })
}

/// Apply Default NaN (DN=1): replace any NaN with canonical quiet NaN.
#[inline]
fn apply_dn(fpscr: u32, result: f32) -> f32 {
    if fpscr & FPSCR_DN != 0 && result.is_nan() {
        f32::from_bits(ARM_DEFAULT_NAN)
    } else {
        result
    }
}

// Detection primitives shared across ops.

#[inline]
fn overflowed(result: f32, any_input_inf: bool) -> bool {
    result.is_infinite() && !any_input_inf
}

/// Tininess-before-rounding + inexact: result is a finite tiny value that
/// differs from the mathematical exact value.
#[inline]
fn underflowed(result: f32, exact: f64) -> bool {
    if !result.is_finite() {
        return false;
    }
    let abs_exact = exact.abs();
    if abs_exact == 0.0 {
        return false;
    }
    abs_exact < F32_MIN_NORMAL_F64 && (result as f64) != exact
}

/// `fp_add` wrapper — performs addition with FPSCR flag tracking.
fn fp_add(fpscr: &mut u32, a: f32, b: f32) -> f32 {
    let (a, b) = (ftz_input(fpscr, a), ftz_input(fpscr, b));
    if is_snan(a) || is_snan(b) {
        *fpscr |= FPSCR_IOC;
    }
    // inf + (-inf) is invalid.
    if a.is_infinite() && b.is_infinite() && a.is_sign_negative() != b.is_sign_negative() {
        *fpscr |= FPSCR_IOC;
        let nan = canonicalize_nan(a + b, a, b);
        return apply_dn(*fpscr, nan);
    }
    let result = canonicalize_nan(a + b, a, b);
    if result.is_nan() {
        return apply_dn(*fpscr, result);
    }
    let exact = (a as f64) + (b as f64);
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        *fpscr |= FPSCR_OFC | FPSCR_IXC;
    } else if let Some(flushed) = ftz_output(*fpscr, result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
        return flushed;
    } else if underflowed(result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
    } else if (result as f64) != exact {
        *fpscr |= FPSCR_IXC;
    }
    result
}

/// `fp_sub` wrapper — performs subtraction with FPSCR flag tracking.
fn fp_sub(fpscr: &mut u32, a: f32, b: f32) -> f32 {
    let (a, b) = (ftz_input(fpscr, a), ftz_input(fpscr, b));
    if is_snan(a) || is_snan(b) {
        *fpscr |= FPSCR_IOC;
    }
    // inf - inf (same sign) is invalid.
    if a.is_infinite() && b.is_infinite() && a.is_sign_negative() == b.is_sign_negative() {
        *fpscr |= FPSCR_IOC;
        let nan = canonicalize_nan(a - b, a, b);
        return apply_dn(*fpscr, nan);
    }
    let result = canonicalize_nan(a - b, a, b);
    if result.is_nan() {
        return apply_dn(*fpscr, result);
    }
    let exact = (a as f64) - (b as f64);
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        *fpscr |= FPSCR_OFC | FPSCR_IXC;
    } else if let Some(flushed) = ftz_output(*fpscr, result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
        return flushed;
    } else if underflowed(result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
    } else if (result as f64) != exact {
        *fpscr |= FPSCR_IXC;
    }
    result
}

/// `fp_mul` wrapper — performs multiplication with FPSCR flag tracking.
fn fp_mul(fpscr: &mut u32, a: f32, b: f32) -> f32 {
    let (a, b) = (ftz_input(fpscr, a), ftz_input(fpscr, b));
    if is_snan(a) || is_snan(b) {
        *fpscr |= FPSCR_IOC;
    }
    // 0 * inf or inf * 0 is invalid.
    if is_mul_inf_zero(a, b) {
        *fpscr |= FPSCR_IOC;
        let nan = canonicalize_nan(a * b, a, b);
        return apply_dn(*fpscr, nan);
    }
    let result = canonicalize_nan(a * b, a, b);
    if result.is_nan() {
        return apply_dn(*fpscr, result);
    }
    let exact = (a as f64) * (b as f64);
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        *fpscr |= FPSCR_OFC | FPSCR_IXC;
    } else if let Some(flushed) = ftz_output(*fpscr, result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
        return flushed;
    } else if underflowed(result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
    } else if (result as f64) != exact {
        *fpscr |= FPSCR_IXC;
    }
    result
}

/// `fp_div` wrapper — performs division with FPSCR flag tracking.
fn fp_div(fpscr: &mut u32, a: f32, b: f32) -> f32 {
    let (a, b) = (ftz_input(fpscr, a), ftz_input(fpscr, b));
    if is_snan(a) || is_snan(b) {
        *fpscr |= FPSCR_IOC;
    }
    // 0/0 and inf/inf are invalid.
    if (a == 0.0 && b == 0.0) || (a.is_infinite() && b.is_infinite()) {
        *fpscr |= FPSCR_IOC;
        let nan = canonicalize_nan(a / b, a, b);
        return apply_dn(*fpscr, nan);
    }
    // Finite nonzero / 0 → divide by zero.
    if b == 0.0 && a.is_finite() && a != 0.0 {
        *fpscr |= FPSCR_DZC;
        return a / b;
    }
    let result = canonicalize_nan(a / b, a, b);
    if result.is_nan() {
        return apply_dn(*fpscr, result);
    }
    let exact = (a as f64) / (b as f64);
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        *fpscr |= FPSCR_OFC | FPSCR_IXC;
    } else if let Some(flushed) = ftz_output(*fpscr, result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
        return flushed;
    } else if underflowed(result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
    } else {
        // Residual test: a − q·b ≠ 0 ⇒ inexact.
        let residual = (-(result as f64)).mul_add(b as f64, a as f64);
        if residual != 0.0 {
            *fpscr |= FPSCR_IXC;
        }
    }
    result
}

/// `fp_sqrt` wrapper — performs square root with FPSCR flag tracking.
fn fp_sqrt(fpscr: &mut u32, a: f32) -> f32 {
    let a = ftz_input(fpscr, a);
    if is_snan(a) {
        *fpscr |= FPSCR_IOC;
    }
    // sqrt(negative non-zero, non-NaN) is invalid. sqrt(-0) = -0 is allowed.
    if a.is_sign_negative() && a != 0.0 && !a.is_nan() {
        *fpscr |= FPSCR_IOC;
        let nan = canonicalize_nan_unary(a.sqrt(), a);
        return apply_dn(*fpscr, nan);
    }
    let result = canonicalize_nan_unary(a.sqrt(), a);
    if result.is_nan() {
        return apply_dn(*fpscr, result);
    }
    if !result.is_finite() || result == 0.0 {
        return result;
    }
    // Residual test: r² − x ≠ 0 ⇒ inexact.
    let residual = (result as f64).mul_add(result as f64, -(a as f64));
    if residual != 0.0 {
        *fpscr |= FPSCR_IXC;
    }
    result
}

/// `fp_fma` wrapper — performs fused multiply-add with FPSCR flag tracking.
/// Computes `a * b + c` with a single rounding.
//
// f64 mul_add is our IXC probe; for worst-case operands requiring >53 bits
// of precision, the probe itself rounds and may miss inexactness. Accepted
// limit — not visible in current fuzz coverage.
fn fp_fma(fpscr: &mut u32, a: f32, b: f32, c: f32) -> f32 {
    let (a, b, c) = (
        ftz_input(fpscr, a),
        ftz_input(fpscr, b),
        ftz_input(fpscr, c),
    );
    if is_snan(a) || is_snan(b) || is_snan(c) {
        *fpscr |= FPSCR_IOC;
    }
    // 0 * inf in the product is invalid, regardless of addend.
    if is_mul_inf_zero(a, b) {
        *fpscr |= FPSCR_IOC;
        let nan = canonicalize_nan_fma(a.mul_add(b, c), c, a, b);
        return apply_dn(*fpscr, nan);
    }
    let raw = a.mul_add(b, c);
    let result = canonicalize_nan_fma(raw, c, a, b);
    if result.is_nan() {
        // (±inf) + (∓inf) via product + addend: IOC.
        let prod_sign = a.is_sign_negative() ^ b.is_sign_negative();
        let product_is_inf = (a.is_infinite() && b != 0.0 && !b.is_nan())
            || (b.is_infinite() && a != 0.0 && !a.is_nan());
        if product_is_inf && c.is_infinite() && prod_sign != c.is_sign_negative() {
            *fpscr |= FPSCR_IOC;
        }
        return apply_dn(*fpscr, result);
    }
    let exact = (a as f64).mul_add(b as f64, c as f64);
    if overflowed(
        result,
        a.is_infinite() || b.is_infinite() || c.is_infinite(),
    ) {
        *fpscr |= FPSCR_OFC | FPSCR_IXC;
    } else if let Some(flushed) = ftz_output(*fpscr, result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
        return flushed;
    } else if underflowed(result, exact) {
        *fpscr |= FPSCR_UFC | FPSCR_IXC;
    } else if (result as f64) != exact {
        *fpscr |= FPSCR_IXC;
    }
    result
}

// ============================================================================
// VFP immediate expansion (VMOV.F32 Sd, #imm)
// ============================================================================

/// VFPExpandImm for single-precision (ARM ARM A7.4.6):
///   result = imm8[7] : NOT(imm8[6]) : Replicate(imm8[6],5) : imm8[5:0] : Zeros(19)
fn vfp_expand_imm_f32(imm8: u8) -> f32 {
    let sign = ((imm8 >> 7) & 1) as u32;
    let b = ((imm8 >> 6) & 1) as u32;
    let not_b = b ^ 1;
    let rep_b = if b != 0 { 0x1F_u32 } else { 0x00_u32 };
    let payload = (imm8 & 0x3F) as u32;
    let bits = (sign << 31) | (not_b << 30) | (rep_b << 25) | (payload << 19);
    f32::from_bits(bits)
}

// ============================================================================
// Float-to-integer conversion helpers
// ============================================================================

fn f32_to_i32_rtz(val: f32) -> i32 {
    if val.is_nan() { return 0; }
    if val >= i32::MAX as f32 { return i32::MAX; }
    if val <= i32::MIN as f32 { return i32::MIN; }
    val as i32
}

fn f32_to_u32_rtz(val: f32) -> u32 {
    if val.is_nan() || val < 0.0 { return 0; }
    if val >= u32::MAX as f32 { return u32::MAX; }
    val as u32
}

fn f32_to_i32_rmode(val: f32, rmode: u32) -> i32 {
    if val.is_nan() { return 0; }
    let rounded = match rmode {
        0b00 => val.round_ties_even(),
        0b01 => val.ceil(),
        0b10 => val.floor(),
        _ => return f32_to_i32_rtz(val),
    };
    if rounded >= i32::MAX as f32 { return i32::MAX; }
    if rounded <= i32::MIN as f32 { return i32::MIN; }
    rounded as i32
}

fn f32_to_u32_rmode(val: f32, rmode: u32) -> u32 {
    if val.is_nan() || val < 0.0 { return 0; }
    let rounded = match rmode {
        0b00 => val.round_ties_even(),
        0b01 => val.ceil(),
        0b10 => val.floor(),
        _ => return f32_to_u32_rtz(val),
    };
    if rounded >= u32::MAX as f32 { return u32::MAX; }
    if rounded < 0.0 { return 0; }
    rounded as u32
}

// ============================================================================
// NaN canonicalization (ARM FPv5 default NaN rules)
// ============================================================================

/// ARM default NaN for single precision: positive quiet NaN.
const ARM_DEFAULT_NAN: u32 = 0x7FC0_0000;

/// Returns true if the value is a signaling NaN (quiet bit = 0).
#[inline(always)]
fn is_snan(v: f32) -> bool {
    let bits = v.to_bits();
    (bits & 0x7FC0_0000) == 0x7F80_0000 && (bits & 0x003F_FFFF) != 0
}

/// Canonicalize NaN result per ARM FPv5 rules.
/// Priority: SNaN (either operand) > QNaN (either operand) > default NaN.
/// Among same NaN type, first operand wins.
#[inline]
fn canonicalize_nan(result: f32, a: f32, b: f32) -> f32 {
    if result.is_nan() {
        if is_snan(a) {
            f32::from_bits(a.to_bits() | 0x0040_0000)
        } else if is_snan(b) {
            f32::from_bits(b.to_bits() | 0x0040_0000)
        } else if a.is_nan() {
            f32::from_bits(a.to_bits() | 0x0040_0000)
        } else if b.is_nan() {
            f32::from_bits(b.to_bits() | 0x0040_0000)
        } else {
            f32::from_bits(ARM_DEFAULT_NAN)
        }
    } else {
        result
    }
}

/// Unary variant for VSQRT and similar single-operand instructions.
#[inline]
fn canonicalize_nan_unary(result: f32, a: f32) -> f32 {
    if result.is_nan() {
        if a.is_nan() {
            f32::from_bits(a.to_bits() | 0x0040_0000)
        } else {
            f32::from_bits(ARM_DEFAULT_NAN)
        }
    } else {
        result
    }
}

/// Returns true if a * b would produce an Invalid Operation (inf * 0 or 0 * inf).
#[inline(always)]
fn is_mul_inf_zero(a: f32, b: f32) -> bool {
    (a.is_infinite() && b == 0.0) || (a == 0.0 && b.is_infinite())
}

/// Fused multiply-add NaN canonicalization per ARM FPMulAdd pseudocode.
/// Priority: SNaN(addend,op1,op2) > inf*0 invalid > QNaN(addend,op1,op2) > default.
#[inline]
fn canonicalize_nan_fma(result: f32, addend: f32, op1: f32, op2: f32) -> f32 {
    if result.is_nan() {
        if is_snan(addend) {
            f32::from_bits(addend.to_bits() | 0x0040_0000)
        } else if is_snan(op1) {
            f32::from_bits(op1.to_bits() | 0x0040_0000)
        } else if is_snan(op2) {
            f32::from_bits(op2.to_bits() | 0x0040_0000)
        } else if is_mul_inf_zero(op1, op2) {
            f32::from_bits(ARM_DEFAULT_NAN)
        } else if addend.is_nan() {
            f32::from_bits(addend.to_bits() | 0x0040_0000)
        } else if op1.is_nan() {
            f32::from_bits(op1.to_bits() | 0x0040_0000)
        } else if op2.is_nan() {
            f32::from_bits(op2.to_bits() | 0x0040_0000)
        } else {
            f32::from_bits(ARM_DEFAULT_NAN)
        }
    } else {
        result
    }
}

// ============================================================================
// Implementation
// ============================================================================

impl CortexM33 {
    // -- Top-level dispatch --------------------------------------------------

    /// Execute a VFP instruction. Called from thumb32_coprocessor when
    /// coproc is 10 or 11.
    pub(crate) fn fpu_execute(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let coproc = ((hw1 >> 8) & 0xF) as u8;
        if coproc == 11 {
            // Double-precision not present on RP2350
            return self.thumb32_undefined(hw0, hw1, bus);
        }

        // Phase 7 Stage B: lazy FP context flush. If a prior exception
        // entry deferred the FP register write (FPCCR.LSPACT=1), flush
        // S0-S15 + FPSCR to the reserved frame *before* the first in-
        // handler FP op. On flush failure, LSPACT is left set so a retry
        // or exception-return sees the unflushed state; the fault itself
        // is signalled via `bus.bus_fault` (for BusFault) or, in future,
        // `self.pending_fault = Fault::MemManage` set inside
        // flush_lazy_fp_context. Either way, we return zero cycles and
        // let step() deliver the fault. After a successful flush (or if
        // no flush was needed), set CONTROL.FPCA=1 — this is the only
        // site that turns FPCA on.
        if self.ppb.fpccr & crate::bus::ppb::FPCCR_LSPACT != 0 {
            match self.flush_lazy_fp_context(bus) {
                Ok(()) => {
                    self.ppb.fpccr &= !crate::bus::ppb::FPCCR_LSPACT;
                }
                Err(()) => {
                    return 0;
                }
            }
        }
        self.regs.control |= crate::core::exceptions::CONTROL_FPCA;

        // Armv8-M FP extensions (VSEL, VMAXNM, VMINNM) are encoded with the
        // 0xFE prefix; the existing VFPv4 ops live under 0xEE.
        // `(hw1 & 0x10) == 0` means "CDP form" — the data-processing branch.
        // The set bit is reserved for coprocessor register transfers, which
        // the 0xFE family does not use.
        let hw0_15_8 = (hw0 >> 8) & 0xFF;
        if hw0_15_8 == 0xFE && hw1 & 0x10 == 0 {
            return self.fpu_v8m_dp(hw0, hw1);
        }

        // Distinguish data-processing / register-transfer from load/store
        // by hw0[11:8]. CDP/MCR/MRC have hw0[11:8]=0b1110, LDC/STC have 0b110x.
        let hw0_11_8 = (hw0 >> 8) & 0xF;
        if hw0_11_8 == 0xE {
            // Data processing or register transfer
            if hw1 & 0x10 != 0 {
                self.fpu_reg_transfer(hw0, hw1)
            } else {
                self.fpu_data_processing(hw0, hw1)
            }
        } else {
            // Load/store (single or multiple)
            self.fpu_load_store(hw0, hw1, bus)
        }
    }

    // -- Armv8-M data-processing extensions (VSEL / VMAXNM / VMINNM) --------
    //
    // Shared prefix hw0[15:8]=0xFE. The high bit hw0[7] splits the family:
    //   hw0[7]=0  → VSEL<cc>   (hw0[6:5]=cc, cc in {EQ,VS,GE,GT})
    //   hw0[7]=1  → VMAXNM/VMINNM (hw1[6]=op: 0=max, 1=min; hw0[6:5]=00)

    fn fpu_v8m_dp(&mut self, hw0: u16, hw1: u16) -> u32 {
        // The 0xFE family encodes D at hw0[4].
        let vd = ((hw1 >> 12) & 0xF) as usize;
        let d = ((hw0 >> 4) & 1) as usize;
        let sd = (vd << 1) | d;
        let vn = (hw0 & 0xF) as usize;
        let n = ((hw1 >> 7) & 1) as usize;
        let sn = (vn << 1) | n;
        let sm = vfp_sm(hw1);

        if hw0 & 0x80 == 0 {
            // VSEL<cc>.F32 — hw0[6:5] = cc: 00=EQ, 01=VS, 10=GE, 11=GT
            let cc = (hw0 >> 5) & 0x3;
            let take_sn = self.vsel_condition_holds(cc);
            self.regs.s[sd] = if take_sn { self.regs.s[sn] } else { self.regs.s[sm] };
            1
        } else if (hw0 >> 5) & 0x3 == 0 {
            // VMAXNM / VMINNM
            //
            // Per DDI0553 §D1.2.88: these set IOC when either input is a
            // signaling NaN (the rest of the semantics — NaN propagation,
            // signed-zero — is handled by `fpu_maxnum` / `fpu_minnum`).
            // No other flags are raised; the ops are pass-through otherwise.
            let (sn_val, sm_val) = (self.regs.s[sn], self.regs.s[sm]);
            if is_snan(sn_val) || is_snan(sm_val) {
                self.regs.fpscr |= FPSCR_IOC;
            }
            let is_min = hw1 & (1 << 6) != 0;
            self.regs.s[sd] = if is_min {
                fpu_minnum(sn_val, sm_val)
            } else {
                fpu_maxnum(sn_val, sm_val)
            };
            1
        } else {
            self.pending_fault = Some(super::Fault::UsageFault);
            0
        }
    }

    /// Evaluate a VSEL condition against the current APSR flags.
    /// Only four conditions are encodable: EQ, VS, GE, GT.
    #[inline]
    fn vsel_condition_holds(&self, cc: u16) -> bool {
        let n = self.regs.flag_n();
        let z = self.regs.flag_z();
        let v = self.regs.flag_v();
        match cc & 0x3 {
            0b00 => z,               // EQ: Z == 1
            0b01 => v,               // VS: V == 1
            0b10 => n == v,          // GE: N == V
            _    => !z && (n == v),  // GT: Z == 0 && N == V
        }
    }

    // -- Data processing -----------------------------------------------------
    //
    // Dispatch on three key bits:
    //   op_hi  = hw0[7]    (opc1[3])
    //   op_lo  = hw0[5:4]  (opc1[1:0]; opc1[2]=hw0[6] is the D register bit)
    //   op2_lo = hw1[6]    (opc2[0]; opc2[1]=hw1[7] is the N register bit)
    //
    // Verified against assembled T32 encodings (EE__:0A__):
    //   (0, 00, 0) VMLA  EE00  (0, 00, 1) VMLS  EE00+40
    //   (0, 01, 0) VNMLS EE10  (0, 01, 1) VNMLA EE10+40
    //   (0, 10, 0) VMUL  EE20  (0, 10, 1) VNMUL EE20+40
    //   (0, 11, 0) VADD  EE30  (0, 11, 1) VSUB  EE30+40
    //   (1, 00, 0) VDIV  EE80  (1, 00, 1) —
    //   (1, 01, 0) VFNMS EE90  (1, 01, 1) VFNMA EE90+40
    //   (1, 10, 0) VFMA  EEA0  (1, 10, 1) VFMS  EEA0+40
    //   (1, 11, 0) VMOV imm    (1, 11, 1) Unary/misc

    fn fpu_data_processing(&mut self, hw0: u16, hw1: u16) -> u32 {
        let op_hi = (hw0 >> 7) & 1;
        let op_lo = (hw0 >> 4) & 0x3;
        let op2_lo = (hw1 >> 6) & 1;

        let sd = vfp_sd(hw0, hw1);
        let sn = vfp_sn(hw0, hw1);
        let sm = vfp_sm(hw1);

        match (op_hi, op_lo, op2_lo) {
            (0, 0b00, 0) => {
                // VMLA.F32 Sd, Sn, Sm — Sd += Sn*Sm (non-fused: two sequential ops)
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                let mul = fp_mul(&mut self.regs.fpscr, sn_val, sm_val);
                self.regs.s[sd] = fp_add(&mut self.regs.fpscr, d, mul);
                3
            }
            (0, 0b00, 1) => {
                // VMLS.F32 Sd, Sn, Sm — Sd -= Sn*Sm (non-fused)
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                let mul = fp_mul(&mut self.regs.fpscr, sn_val, sm_val);
                self.regs.s[sd] = fp_sub(&mut self.regs.fpscr, d, mul);
                3
            }
            (0, 0b01, 0) => {
                // VNMLS.F32 Sd, Sn, Sm — Sd = Sn*Sm - Sd (non-fused, product first)
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                let mul = fp_mul(&mut self.regs.fpscr, sn_val, sm_val);
                self.regs.s[sd] = fp_sub(&mut self.regs.fpscr, mul, d);
                3
            }
            (0, 0b01, 1) => {
                // VNMLA.F32 Sd, Sn, Sm — Sd = -(Sn*Sm + Sd) (non-fused, product first)
                //
                // FPNeg (DDI0553 §A2.2.6) is an unconditional sign-bit flip on
                // bit [31], including for NaNs. If DN=1, `fp_add` has already
                // canonicalized the NaN before we get it; the trailing negate
                // is then re-canonicalized by `apply_dn` below.
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                let mul = fp_mul(&mut self.regs.fpscr, sn_val, sm_val);
                let sum = fp_add(&mut self.regs.fpscr, mul, d);
                self.regs.s[sd] = apply_dn(self.regs.fpscr, -sum);
                3
            }
            (0, 0b10, 0) => {
                // VMUL.F32 Sd, Sn, Sm
                let (sn_val, sm_val) = (self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_mul(&mut self.regs.fpscr, sn_val, sm_val);
                1
            }
            (0, 0b10, 1) => {
                // VNMUL.F32 Sd, Sn, Sm — Sd = -(Sn * Sm)
                //
                // Compute the multiply with flag tracking; the negation is a
                // pure sign flip and doesn't change any exception flag.
                // FPNeg (DDI0553 §A2.2.6) flips bit [31] unconditionally —
                // including for NaNs — so we negate even NaN products. When
                // DN=1 the product was already the canonical quiet NaN
                // (positive); `apply_dn` re-canonicalizes after the flip so
                // the visible result is still 0x7FC00000.
                let (sn_val, sm_val) = (self.regs.s[sn], self.regs.s[sm]);
                let prod = fp_mul(&mut self.regs.fpscr, sn_val, sm_val);
                self.regs.s[sd] = apply_dn(self.regs.fpscr, -prod);
                1
            }
            (0, 0b11, 0) => {
                // VADD.F32 Sd, Sn, Sm
                let (sn_val, sm_val) = (self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_add(&mut self.regs.fpscr, sn_val, sm_val);
                1
            }
            (0, 0b11, 1) => {
                // VSUB.F32 Sd, Sn, Sm
                let (sn_val, sm_val) = (self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_sub(&mut self.regs.fpscr, sn_val, sm_val);
                1
            }
            (1, 0b00, 0) => {
                // VDIV.F32 Sd, Sn, Sm
                let (sn_val, sm_val) = (self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_div(&mut self.regs.fpscr, sn_val, sm_val);
                14
            }
            (1, 0b01, 0) => {
                // VFNMS.F32 Sd, Sn, Sm — Sd = Sn*Sm - Sd (fused)
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_fma(&mut self.regs.fpscr, sn_val, sm_val, -d);
                3
            }
            (1, 0b01, 1) => {
                // VFNMA.F32 Sd, Sn, Sm — Sd = -(Sn*Sm + Sd) (fused)
                //
                // Encoded as (-Sn)*Sm + (-Sd) per ARM pseudocode; the result
                // sign is already correct because the fused op rounds once.
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_fma(&mut self.regs.fpscr, -sn_val, sm_val, -d);
                3
            }
            (1, 0b10, 0) => {
                // VFMA.F32 Sd, Sn, Sm — Sd = Sd + Sn*Sm (fused)
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_fma(&mut self.regs.fpscr, sn_val, sm_val, d);
                3
            }
            (1, 0b10, 1) => {
                // VFMS.F32 Sd, Sn, Sm — Sd = Sd - Sn*Sm (fused)
                //
                // Encoded as (-Sn)*Sm + Sd to get a single-rounded fused op.
                let (d, sn_val, sm_val) = (self.regs.s[sd], self.regs.s[sn], self.regs.s[sm]);
                self.regs.s[sd] = fp_fma(&mut self.regs.fpscr, -sn_val, sm_val, d);
                3
            }
            (1, 0b11, 0) => {
                // VMOV.F32 Sd, #imm — load immediate
                // imm8 = imm4H:imm4L where imm4H = hw0[3:0], imm4L = hw1[3:0]
                let imm4h = (hw0 & 0xF) as u8;
                let imm4l = (hw1 & 0xF) as u8;
                let imm8 = (imm4h << 4) | imm4l;
                self.regs.s[sd] = vfp_expand_imm_f32(imm8);
                1
            }
            (1, 0b11, 1) => {
                // Unary / misc operations (VMOV reg, VABS, VNEG, VSQRT,
                // VCMP, VCMPE, VCVT, VRINTR, VRINTZ, VRINTX)
                self.fpu_unary(hw0, hw1, sd, sm)
            }
            _ => {
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
        }
    }

    // -- Unary / misc --------------------------------------------------------
    //
    // All have opc1=1_D_11, opc2[0]=1. The sub-operation is encoded in:
    //   opc3 = hw0[3:0] (repurposed Vn field, since these are single-operand)
    //   T    = hw1[7]   (repurposed N bit)

    fn fpu_unary(&mut self, hw0: u16, hw1: u16, sd: usize, sm: usize) -> u32 {
        let opc3 = hw0 & 0xF;
        let t = (hw1 >> 7) & 1;

        match (opc3, t) {
            (0b0000, 0) => {
                // VMOV.F32 Sd, Sm (register copy)
                self.regs.s[sd] = self.regs.s[sm];
                1
            }
            (0b0000, 1) => {
                // VABS.F32 Sd, Sm
                self.regs.s[sd] = self.regs.s[sm].abs();
                1
            }
            (0b0001, 0) => {
                // VNEG.F32 Sd, Sm
                self.regs.s[sd] = -self.regs.s[sm];
                1
            }
            (0b0001, 1) => {
                // VSQRT.F32 Sd, Sm
                let sm_val = self.regs.s[sm];
                self.regs.s[sd] = fp_sqrt(&mut self.regs.fpscr, sm_val);
                14
            }
            (0b0010, 0) => {
                // VCVTB.F16.F32 Sd, Sm — f32 Sm → f16 into bottom half of Sd
                let (half, flags) = f32_to_f16_bits(self.regs.s[sm], self.regs.fpscr);
                self.regs.fpscr |= flags;
                let dst = self.regs.s[sd].to_bits();
                self.regs.s[sd] = f32::from_bits((dst & 0xFFFF_0000) | half as u32);
                1
            }
            (0b0010, 1) => {
                // VCVTT.F16.F32 Sd, Sm — f32 Sm → f16 into top half of Sd
                let (half, flags) = f32_to_f16_bits(self.regs.s[sm], self.regs.fpscr);
                self.regs.fpscr |= flags;
                let dst = self.regs.s[sd].to_bits();
                self.regs.s[sd] = f32::from_bits((dst & 0x0000_FFFF) | ((half as u32) << 16));
                1
            }
            (0b0011, 0) => {
                // VCVTB.F32.F16 Sd, Sm — bottom half of Sm as f16 → f32 Sd
                let half = (self.regs.s[sm].to_bits() & 0xFFFF) as u16;
                let (val, flags) = f16_bits_to_f32(half, self.regs.fpscr);
                self.regs.fpscr |= flags;
                self.regs.s[sd] = val;
                1
            }
            (0b0011, 1) => {
                // VCVTT.F32.F16 Sd, Sm — top half of Sm as f16 → f32 Sd
                let half = ((self.regs.s[sm].to_bits() >> 16) & 0xFFFF) as u16;
                let (val, flags) = f16_bits_to_f32(half, self.regs.fpscr);
                self.regs.fpscr |= flags;
                self.regs.s[sd] = val;
                1
            }
            (0b0100, 0) => {
                // VCMP.F32 Sd, Sm — compare, quiet on NaN
                self.fpu_vcmp(sd, self.regs.s[sm]);
                1
            }
            (0b0100, 1) => {
                // VCMPE.F32 Sd, Sm — compare, exception on NaN
                // Same result as VCMP for emulation (no FP exceptions)
                self.fpu_vcmp(sd, self.regs.s[sm]);
                1
            }
            (0b0101, 0) => {
                // VCMP.F32 Sd, #0.0
                self.fpu_vcmp(sd, 0.0);
                1
            }
            (0b0101, 1) => {
                // VCMPE.F32 Sd, #0.0
                self.fpu_vcmp(sd, 0.0);
                1
            }
            (0b0110, 0) => {
                // VRINTR.F32 Sd, Sm — round per FPSCR.RMode (no IXC tracking)
                let rmode = fpscr_rmode(self.regs.fpscr);
                let (val, flags) = fpu_vrint(self.regs.s[sm], rmode, self.regs.fpscr, false);
                self.regs.fpscr |= flags;
                self.regs.s[sd] = val;
                1
            }
            (0b0110, 1) => {
                // VRINTZ.F32 Sd, Sm — round toward zero (no IXC tracking)
                let (val, flags) =
                    fpu_vrint(self.regs.s[sm], 0b11, self.regs.fpscr, false);
                self.regs.fpscr |= flags;
                self.regs.s[sd] = val;
                1
            }
            (0b0111, 0) => {
                // VRINTX.F32 Sd, Sm — round per FPSCR.RMode, exact (raises IXC)
                let rmode = fpscr_rmode(self.regs.fpscr);
                let (val, flags) = fpu_vrint(self.regs.s[sm], rmode, self.regs.fpscr, true);
                self.regs.fpscr |= flags;
                self.regs.s[sd] = val;
                1
            }
            (0b1000, 0) => {
                // VCVT.F32.U32 Sd, Sm — unsigned int → float
                let bits = self.regs.s[sm].to_bits();
                self.regs.s[sd] = bits as f32;
                1
            }
            (0b1000, 1) => {
                // VCVT.F32.S32 Sd, Sm — signed int → float
                let bits = self.regs.s[sm].to_bits() as i32;
                self.regs.s[sd] = bits as f32;
                1
            }
            (0b1010, 0) => {
                // VCVT.F32.FX.U16 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            (0b1010, 1) => {
                // VCVT.F32.FX.S16 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            (0b1011, 0) => {
                // VCVT.F32.FX.U32 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            (0b1011, 1) => {
                // VCVT.F32.FX.S32 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            (0b1100, 0) => {
                // VCVTR.U32.F32 Sd, Sm — float → unsigned int (round per FPSCR)
                let rmode = fpscr_rmode(self.regs.fpscr);
                let result = f32_to_u32_rmode(self.regs.s[sm], rmode);
                self.regs.s[sd] = f32::from_bits(result);
                1
            }
            (0b1100, 1) => {
                // VCVT.U32.F32 Sd, Sm — float → unsigned int (round toward zero)
                let result = f32_to_u32_rtz(self.regs.s[sm]);
                self.regs.s[sd] = f32::from_bits(result);
                1
            }
            (0b1101, 0) => {
                // VCVTR.S32.F32 Sd, Sm — float → signed int (round per FPSCR)
                let rmode = fpscr_rmode(self.regs.fpscr);
                let result = f32_to_i32_rmode(self.regs.s[sm], rmode);
                self.regs.s[sd] = f32::from_bits(result as u32);
                1
            }
            (0b1101, 1) => {
                // VCVT.S32.F32 Sd, Sm — float → signed int (round toward zero)
                let result = f32_to_i32_rtz(self.regs.s[sm]);
                self.regs.s[sd] = f32::from_bits(result as u32);
                1
            }
            (0b1110, 0) => {
                // VCVT.FX.U16.F32 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            (0b1110, 1) => {
                // VCVT.FX.S16.F32 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            (0b1111, 0) => {
                // VCVT.FX.U32.F32 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            (0b1111, 1) => {
                // VCVT.FX.S32.F32 — fixed-point, stub
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
            _ => {
                self.pending_fault = Some(super::Fault::UsageFault);
                0
            }
        }
    }

    /// VCMP helper: compare Sd against a value, set FPSCR N,Z,C,V.
    fn fpu_vcmp(&mut self, sd: usize, rhs: f32) {
        let lhs = self.regs.s[sd];
        let (n, z, c, v) = if lhs.is_nan() || rhs.is_nan() {
            (false, false, true, true) // unordered
        } else if lhs == rhs {
            (false, true, true, false) // equal
        } else if lhs < rhs {
            (true, false, false, false) // less than
        } else {
            (false, false, true, false) // greater than
        };
        fpscr_set_nzcv(&mut self.regs.fpscr, n, z, c, v);
    }

    // -- Register transfer (VMOV to/from ARM, VMRS, VMSR) -------------------
    //
    // Encoding: hw0 = 1110_1110_opc1[3:0]_Vn/CRn
    //           hw1 = Rt_1010_opc2_1_CRm
    //
    // For single-precision VFP register transfers:
    //   VMOV Sn, Rt:    opc1=000L where L=0, hw0[7:4]=000D, hw1[15:12]=Rt
    //   VMOV Rt, Sn:    opc1=000L where L=1, hw0[7:4]=000D, hw1[15:12]=Rt
    //
    // Actually, the register transfer encoding is:
    //   VMOV Sn, Rt:  EE0n_Rt_A10 → hw0[7:4]=0_0_0_0, hw0[3:0]=Vn, hw1[4]=1
    //                 This is MCR: write ARM reg to coproc
    //   VMOV Rt, Sn:  EE1n_Rt_A10 → hw0[7:4]=0_0_0_1
    //                 This is MRC: read coproc reg to ARM
    //
    // The L bit is hw0[4] (Inst[20]):
    //   L=0 → MCR (ARM→FPU): VMOV Sn, Rt  and  VMSR FPSCR, Rt
    //   L=1 → MRC (FPU→ARM): VMOV Rt, Sn  and  VMRS Rt, FPSCR
    //
    // VMRS/VMSR are distinguished by hw0[7:5]=0b111:
    //   VMRS Rt, FPSCR: hw0=0xEEF1, hw1=Rt_A10
    //   VMSR FPSCR, Rt: hw0=0xEEE1, hw1=Rt_A10

    fn fpu_reg_transfer(&mut self, hw0: u16, hw1: u16) -> u32 {
        let l = (hw0 >> 4) & 1; // L bit: 0=to-coproc, 1=from-coproc
        let rt = ((hw1 >> 12) & 0xF) as usize;

        // Check for VMRS/VMSR (special register transfer)
        // VMRS: hw0[7:4]=1111, VMSR: hw0[7:4]=1110
        let opc_hi = (hw0 >> 5) & 0x7;
        if opc_hi == 0b111 {
            if l != 0 {
                // VMRS Rt, FPSCR — read FPSCR into ARM register
                if rt == 15 {
                    // VMRS APSR_nzcv, FPSCR — copy FPSCR flags to APSR
                    let nzcv = self.regs.fpscr & 0xF000_0000;
                    self.regs.xpsr = (self.regs.xpsr & 0x0FFF_FFFF) | nzcv;
                } else {
                    self.regs.r[rt] = self.regs.fpscr;
                }
            } else {
                // VMSR FPSCR, Rt — write ARM register to FPSCR
                self.regs.fpscr = self.regs.r[rt];
            }
            return 1;
        }

        // VMOV between ARM and FPU registers
        // The VFP register is Sn = (Vn << 1) | N
        let sn = vfp_sn(hw0, hw1);

        if l == 0 {
            // VMOV Sn, Rt — ARM → FPU
            self.regs.s[sn] = f32::from_bits(self.regs.r[rt]);
        } else {
            // VMOV Rt, Sn — FPU → ARM
            self.regs.r[rt] = self.regs.s[sn].to_bits();
        }
        1
    }

    // -- Load/store (VLDR, VSTR, VLDM, VSTM, VPUSH, VPOP) ------------------
    //
    // LDC/STC encoding:
    //   hw0 = 1110_110_P_U_D_W_L_Rn
    //   hw1 = Vd_1010_imm8
    //
    // Bits:
    //   P = hw0[8]  — pre/post indexed
    //   U = hw0[7]  — add/subtract offset
    //   D = hw0[6]  — part of Sd register encoding
    //   W = hw0[5]  — writeback
    //   L = hw0[4]  — load (1) / store (0)
    //   Rn = hw0[3:0]
    //   Vd = hw1[15:12]
    //   imm8 = hw1[7:0]
    //
    // For single-register (VLDR/VSTR): P=1, W=0
    //   Sd = (Vd << 1) | D
    //   Address = Rn ± (imm8 << 2)
    //
    // For multiple (VLDM/VSTM): P and U determine direction
    //   Sd = (Vd << 1) | D  (first register)
    //   Count = imm8 (number of single registers)

    fn fpu_load_store(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let p = (hw0 >> 8) & 1;
        let u = (hw0 >> 7) & 1;
        let w = (hw0 >> 5) & 1;
        let l = (hw0 >> 4) & 1;
        let rn = (hw0 & 0xF) as usize;
        let sd = vfp_sd(hw0, hw1);
        let imm8 = (hw1 & 0xFF) as u32;

        if p == 1 && w == 0 {
            // VLDR / VSTR (single register, immediate offset)
            let offset = imm8 << 2;
            let base = if rn == 15 {
                self.read_pc() & !0x3 // Align(PC, 4)
            } else {
                self.regs.r[rn]
            };
            let addr = if u != 0 {
                base.wrapping_add(offset)
            } else {
                base.wrapping_sub(offset)
            };

            if l != 0 {
                // VLDR.32
                self.regs.s[sd] = f32::from_bits(self.bus_read32(addr, bus));
                2
            } else {
                // VSTR.32
                self.bus_write32(addr, self.regs.s[sd].to_bits(), bus);
                1
            }
        } else {
            // VLDM / VSTM / VPUSH / VPOP (multiple registers)
            let count = imm8 as usize;
            if count == 0 {
                return self.thumb32_undefined(hw0, hw1, bus);
            }

            let base = self.regs.r[rn];

            // Determine start address based on P and U:
            //   VLDMIA / VPOP:  P=0, U=1 → start = Rn, Rn += count*4
            //   VSTMDB / VPUSH: P=1, U=0 → start = Rn - count*4, Rn -= count*4
            //   VLDMDB:         P=1, U=0 → start = Rn - count*4
            //   VSTMIA:         P=0, U=1 → start = Rn
            let mut addr = if u != 0 {
                base // increment-after
            } else {
                base.wrapping_sub((count as u32) << 2) // decrement-before
            };

            for i in 0..count {
                let reg = sd + i;
                if reg >= 32 { break; }

                if l != 0 {
                    self.regs.s[reg] = f32::from_bits(self.bus_read32(addr, bus));
                } else {
                    self.bus_write32(addr, self.regs.s[reg].to_bits(), bus);
                }
                addr = addr.wrapping_add(4);
            }

            // Writeback
            if w != 0 {
                if u != 0 {
                    self.regs.r[rn] = base.wrapping_add((count as u32) << 2);
                } else {
                    self.regs.r[rn] = base.wrapping_sub((count as u32) << 2);
                }
            }

            if l != 0 {
                1 + count as u32 // load: 1 + N cycles
            } else {
                count as u32 // store: N cycles
            }
        }
    }

    /// Flush lazy FP context.
    ///
    /// Called from `fpu_execute` when FPCCR.LSPACT=1 — meaning a prior
    /// exception entry reserved space at FPCAR but deferred the actual
    /// S0-S15 + FPSCR write. Walks the reserved 18-word frame and writes
    /// the 17 live values (S0..S15 + FPSCR; the trailing word is
    /// architecturally reserved). Per HLD §B.8, on failure we leave
    /// LSPACT set so a retry or exception-return sees the unflushed state.
    ///
    /// # Fault signalling
    ///
    /// Failures are communicated through two orthogonal channels rather
    /// than a `Fault` enum:
    ///   * **BusFault** — a bus-side write abort sets `bus.bus_fault = true`
    ///     (picked up by `step()` in the main loop) and this function
    ///     records the abort in `FPCCR.BFRDY`. `step()` then delivers the
    ///     BusFault via `enter_exception(5)`.
    ///   * **MemManage** — *not yet wired*. When Stage E enforces the MPU
    ///     on data writes, this function will also set `FPCCR.MMRDY` and
    ///     assign `self.pending_fault = Some(Fault::MemManage)` directly.
    ///
    /// The `Err(())` return is purely a signal that the flush aborted;
    /// the caller just reports zero cycles and lets `step()` pick up the
    /// side-channel bus-fault flag (or, in future, the pending MemManage).
    /// Using a unit error avoids fabricating a `Fault::UsageFault` that
    /// is never actually delivered (step() catches the bus flag first).
    pub(crate) fn flush_lazy_fp_context(
        &mut self,
        bus: &mut Bus,
    ) -> Result<(), ()> {
        let base = self.ppb.fpcar;

        let core = self.core_id as usize;
        // S0..S15 → +0..+60.
        for i in 0..16 {
            self.bus_write32(base.wrapping_add((i as u32) * 4), self.regs.s[i].to_bits(), bus);
            if bus.bus_fault(core) {
                self.ppb.fpccr |= crate::bus::ppb::FPCCR_BFRDY;
                return Err(());
            }
        }
        // FPSCR → +64; reserved → +68 (write zero per architecture).
        self.bus_write32(base.wrapping_add(64), self.regs.fpscr, bus);
        if bus.bus_fault(core) {
            self.ppb.fpccr |= crate::bus::ppb::FPCCR_BFRDY;
            return Err(());
        }
        self.bus_write32(base.wrapping_add(68), 0, bus);
        if bus.bus_fault(core) {
            self.ppb.fpccr |= crate::bus::ppb::FPCCR_BFRDY;
            return Err(());
        }
        Ok(())
    }
}

// ============================================================================
// VRINT helper
// ============================================================================

/// Round a single-precision value to integer per `rmode`, with full FPSCR
/// accounting per ARM DDI0553 FPRoundInt.
///
/// `exact = true` matches VRINTX semantics (raise IXC when the rounded value
/// differs from the input). `exact = false` matches VRINTR/VRINTZ (no IXC).
///
/// All variants set IDC on denormal input, flush input under FZ=1, raise IOC
/// on SNaN, and replace any NaN result with the ARM default NaN under DN=1.
/// Under DN=0, NaN payload is propagated with the quiet bit forced — matching
/// the spec rather than relying on platform-specific NaN handling in the host
/// rounding intrinsics.
fn fpu_vrint(val: f32, rmode: u32, fpscr_in: u32, exact: bool) -> (f32, u32) {
    let mut flags = 0u32;
    let val = ftz_input_value(fpscr_in, &mut flags, val);

    if val.is_nan() {
        if is_snan(val) {
            flags |= FPSCR_IOC;
        }
        let quietened = quieten_nan(val);
        return (apply_dn(fpscr_in, quietened), flags);
    }
    if val.is_infinite() || val == 0.0 {
        return (val, flags);
    }
    let rounded = match rmode {
        0b00 => val.round_ties_even(),
        0b01 => val.ceil(),
        0b10 => val.floor(),
        _ => val.trunc(),
    };
    if exact && rounded != val {
        flags |= FPSCR_IXC;
    }
    (rounded, flags)
}

/// FTZ on input that returns flags via `&mut` rather than mutating an FPSCR
/// register directly. Used by helpers that compose flag deltas.
#[inline]
fn ftz_input_value(fpscr_in: u32, flags: &mut u32, v: f32) -> f32 {
    if is_denormal(v) {
        *flags |= FPSCR_IDC;
        if fpscr_in & FPSCR_FZ != 0 {
            return if v.is_sign_negative() { -0.0 } else { 0.0 };
        }
    }
    v
}

/// Force the quiet bit on any NaN value (no-op on non-NaN). Matches FPProcess
/// quietening per DDI0553.
#[inline]
fn quieten_nan(v: f32) -> f32 {
    if v.is_nan() {
        f32::from_bits(v.to_bits() | 0x0040_0000)
    } else {
        v
    }
}

// ============================================================================
// IEEE 754-2008 maxNum / minNum (NaN-aware)
// ============================================================================
//
// If exactly one operand is NaN, return the other; if both are NaN, return a
// default quiet NaN.
//
// Signed-zero handling (IEEE 754-2008 §5.3.1): maxNum(+0, -0) = +0 and
// minNum(+0, -0) = -0 regardless of operand order. The float comparisons
// `a > b` / `a < b` treat +0 and -0 as equal, so we must special-case zeros
// via the sign bit.
//
// Exception flag tracking (IOC on sNaN, etc.) is deferred to Stage A.1.

#[inline]
fn fpu_maxnum(a: f32, b: f32) -> f32 {
    if a.is_nan() && b.is_nan() {
        f32::from_bits(ARM_DEFAULT_NAN)
    } else if a.is_nan() {
        b
    } else if b.is_nan() {
        a
    } else if a == 0.0 && b == 0.0 {
        // Both zero: return +0 if either operand is +0.
        // Sign bit is bit 31; 0 => positive.
        if (a.to_bits() & 0x8000_0000) == 0 || (b.to_bits() & 0x8000_0000) == 0 {
            0.0
        } else {
            -0.0
        }
    } else if a > b {
        a
    } else {
        b
    }
}

#[inline]
fn fpu_minnum(a: f32, b: f32) -> f32 {
    if a.is_nan() && b.is_nan() {
        f32::from_bits(ARM_DEFAULT_NAN)
    } else if a.is_nan() {
        b
    } else if b.is_nan() {
        a
    } else if a == 0.0 && b == 0.0 {
        // Both zero: return -0 if either operand is -0.
        if (a.to_bits() & 0x8000_0000) != 0 || (b.to_bits() & 0x8000_0000) != 0 {
            -0.0
        } else {
            0.0
        }
    } else if a < b {
        a
    } else {
        b
    }
}

// ============================================================================
// Half-precision (IEEE 754 binary16) ↔ single-precision conversion
// ============================================================================
//
// IEEE-754 layout:
//   f16: sign(1) | exp(5) | frac(10)  — bias 15
//   f32: sign(1) | exp(8) | frac(23)  — bias 127
//
// The FPSCR AHP bit (bit 26) selects "alternative half-precision" encoding
// (no Inf/NaN, extended exponent range). RP2350 ships with AHP=0 at reset;
// downstream code leaves it cleared. We implement IEEE (AHP=0) faithfully
// and silently treat AHP=1 the same as AHP=0 for now — see Phase 7 HLD §A.2.

/// True if a half-precision encoding is a signaling NaN (exp all-ones, frac
/// non-zero, quiet bit clear). Quiet bit for f16 is bit 9 of the 10-bit
/// fraction field.
#[inline]
fn is_snan_f16(h: u16) -> bool {
    let exp = (h >> 10) & 0x1F;
    let frac = h & 0x3FF;
    exp == 0x1F && frac != 0 && (frac & 0x200) == 0
}

/// Convert IEEE binary16 bits to an f32 value with FPSCR accounting.
///
/// Sets IOC on signaling-NaN input. Replaces any NaN result with the ARM
/// default NaN (0x7FC0_0000) when DN=1; otherwise propagates the f16 payload
/// into the f32 fraction with the quiet bit forced. f16 denormals are
/// value-preserved into f32 normals (Cortex-M33 has no FZ16 control).
// TODO(phase-7.1): honor FPSCR.AHP (alternative half-precision encoding).
fn f16_bits_to_f32(h: u16, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0u32;
    let sign = ((h as u32) & 0x8000) << 16;
    let exp = ((h as u32) >> 10) & 0x1F;
    let frac = (h as u32) & 0x3FF;

    let bits = if exp == 0 {
        if frac == 0 {
            // ±0
            sign
        } else {
            // Subnormal half → normalized f32. Shift frac left until the
            // hidden bit position is reached.
            let mut mantissa = frac;
            let mut e: i32 = 1; // biased-15 exponent of leading-1 after normalization
            while mantissa & 0x400 == 0 {
                mantissa <<= 1;
                e -= 1;
            }
            let exp32 = (e - 15 + 127) as u32;
            sign | (exp32 << 23) | ((mantissa & 0x3FF) << 13)
        }
    } else if exp == 0x1F {
        // Inf / NaN
        if frac == 0 {
            sign | 0x7F80_0000
        } else {
            if is_snan_f16(h) {
                flags |= FPSCR_IOC;
            }
            // Preserve NaN payload into f32 fraction, force quiet bit.
            // `apply_dn` below substitutes the canonical NaN under DN=1.
            sign | 0x7F80_0000 | (frac << 13) | 0x0040_0000
        }
    } else {
        // Normalized half → normalized f32
        let exp32 = (exp as i32 - 15 + 127) as u32;
        sign | (exp32 << 23) | (frac << 13)
    };
    let result = f32::from_bits(bits);
    (apply_dn(fpscr_in, result), flags)
}

/// Convert an f32 value to IEEE binary16 bits (round-to-nearest-even, AHP=0)
/// with FPSCR accounting.
///
/// Sets IOC on signaling-NaN input. Sets IDC on f32 denormal input (which
/// flushes to f16 ±0 regardless of FZ — denormal f32 magnitudes are below the
/// smallest representable f16 subnormal). Replaces any NaN result with the f16
/// default NaN (0x7E00) when DN=1.
// TODO(phase-7.1): honor FPSCR.AHP (alternative half-precision encoding).
fn f32_to_f16_bits(v: f32, fpscr_in: u32) -> (u16, u32) {
    let mut flags = 0u32;
    let bits = v.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exp = ((bits >> 23) & 0xFF) as i32;
    let frac = bits & 0x7F_FFFF;

    if exp == 0xFF {
        // Inf or NaN
        let h = if frac == 0 {
            sign | 0x7C00
        } else {
            if is_snan(v) {
                flags |= FPSCR_IOC;
            }
            if fpscr_in & FPSCR_DN != 0 {
                // Default NaN for f16: positive QNaN, no payload.
                0x7E00
            } else {
                // Quiet NaN: preserve top 9 payload bits, force quiet bit.
                // IEEE: sNaN must quieten; lower payload bits are lost — spec-allowed.
                let payload = (frac >> 13) as u16;
                sign | 0x7E00 | (payload & 0x1FF)
            }
        };
        return (h, flags);
    }

    if exp == 0 {
        if frac != 0 {
            // f32 subnormal → IDC. Always flushes to f16 ±0 (tiny enough that
            // f16 round-to-nearest produces zero).
            flags |= FPSCR_IDC;
        }
        // f32 ±0 or subnormal → f16 ±0.
        return (sign, flags);
    }

    // Unbiased f32 exponent.
    let e = exp - 127;

    if e > 15 {
        // Overflow → +/- inf in IEEE half-precision.
        return (sign | 0x7C00, flags);
    }
    if e < -24 {
        // Smaller than smallest subnormal → ±0 (round-to-nearest-even).
        return (sign, flags);
    }

    if e < -14 {
        // Subnormal result in half precision. Compute
        //   frac_10 = m * 2^(e+1)   where m is the 24-bit f32 mantissa
        //   (implicit 1 + 23 fraction bits). Shift is `-e - 1` in [14, 23]
        //   for e in [-15, -24] (we already returned zero for e < -24).
        let m = (frac | 0x0080_0000) as u64;
        let shift: u32 = (-e - 1) as u32;
        let mantissa = (m >> shift) as u32;
        let round_bit = if shift == 0 { 0 } else { ((m >> (shift - 1)) & 1) as u32 };
        let sticky = if shift < 2 { false } else { (m & ((1u64 << (shift - 1)) - 1)) != 0 };
        let lsb = mantissa & 1;
        let rounded = mantissa + if round_bit != 0 && (sticky || lsb != 0) { 1 } else { 0 };
        // Rounding up may carry the result into the normal range (2^-14),
        // which is exactly exp=1, frac=0 in half precision.
        if rounded >= 0x400 {
            return (sign | (1 << 10), flags);
        }
        return (sign | (rounded as u16 & 0x3FF), flags);
    }

    // Normal result.
    let exp16 = (e + 15) as u16;
    let mantissa = frac >> 13;
    let round_bit = (frac >> 12) & 1;
    let sticky = (frac & 0xFFF) != 0;
    let lsb = mantissa & 1;
    let rounded = mantissa + if round_bit != 0 && (sticky || lsb != 0) { 1 } else { 0 };

    // Rounding may overflow the 10-bit fraction into the exponent.
    if rounded > 0x3FF {
        let new_exp = exp16 + 1;
        if new_exp >= 0x1F {
            return (sign | 0x7C00, flags); // overflow to inf
        }
        return (sign | (new_exp << 10), flags);
    }
    (sign | (exp16 << 10) | (rounded as u16 & 0x3FF), flags)
}

// IEEE-754 single-precision reference oracle for FPSCR flag validation.
//
// This is the **independent oracle** used by the Phase 7 retro-validation
// harness (`bin/softfloat_diff.rs`) and by unit tests. It must not call into
// emulator code — its job is to compute the "correct" f32 result *and* the
// correct FPSCR exception flag set from first principles.
//
// Design note (HLD §17.1): we hand-rolled this instead of wrapping Berkeley
// SoftFloat. Rationale: no MSVC C toolchain dependency, and the 6 flags we
// track (IXC/OFC/UFC/DZC/IOC/IDC) are enumerable in ~100 LOC of Rust.
//
// Flag bits (matching `execute_fpu.rs`):
//   bit 0  IOC — invalid operation
//   bit 1  DZC — divide by zero
//   bit 2  OFC — overflow
//   bit 3  UFC — underflow
//   bit 4  IXC — inexact
//   bit 7  IDC — input denormal
//   bit 24 FZ  — flush-to-zero (control, not a cumulative flag)
//   bit 25 DN  — default NaN (control, not a cumulative flag)
//
// FZ/DN are **control bits**; the oracle reads them from `fpscr_in` but
// never modifies them. Only the six cumulative flags appear in the returned
// flag word.

// The reference oracle relies on `f64::mul_add` being a true single-rounding
// fused op (for `ref_fma` and the division residual probe). Without the
// `+fma` target feature the compiler falls back to a * b + c with two
// roundings, which silently de-synchronises the oracle from the emulator
// (which uses `f32::mul_add`) and masks boundary bugs. Fail the build
// LOUDLY rather than enumerate every possible host target in .cargo/config.toml.
#[cfg(not(target_feature = "fma"))]
compile_error!(
    "mdrp2354-test-harness requires the +fma target feature. \
     Add 'rustflags = [\"-C\", \"target-feature=+fma\"]' to your \
     target's section in .cargo/config.toml."
);

pub const IOC: u32 = 1 << 0;
pub const DZC: u32 = 1 << 1;
pub const OFC: u32 = 1 << 2;
pub const UFC: u32 = 1 << 3;
pub const IXC: u32 = 1 << 4;
pub const IDC: u32 = 1 << 7;

/// FZ (flush-to-zero) control bit in FPSCR. When set, denormal inputs are
/// treated as ±0 with IDC set, and tininess-before-rounding outputs are
/// flushed to ±0 with UFC+IXC set.
pub const FZ: u32 = 1 << 24;

/// DN (default NaN) control bit in FPSCR. When set, any NaN result is
/// replaced with the canonical quiet NaN (0x7FC0_0000).
pub const DN: u32 = 1 << 25;

/// ARM default NaN (positive quiet NaN, zero payload).
const ARM_DEFAULT_NAN: u32 = 0x7FC0_0000;

/// f32 smallest positive normal as f64: 2^-126 exactly. Must match the
/// `F32_MIN_NORMAL_F64` constant in `crates/mdrp2354/src/core/execute_fpu.rs`;
/// any drift would mis-classify underflow boundary cases.
const MIN_NORMAL_F64: f64 = 1.175_494_350_822_287_5e-38;

/// True if `v` is a subnormal (non-zero) f32.
#[inline]
fn is_denormal_f32(v: f32) -> bool {
    let bits = v.to_bits();
    let exp = (bits >> 23) & 0xFF;
    let frac = bits & 0x7F_FFFF;
    exp == 0 && frac != 0
}

/// True if `v` is a signaling NaN (IEEE-754: NaN with quiet bit clear).
#[inline]
fn is_snan_f32(v: f32) -> bool {
    let bits = v.to_bits();
    (bits & 0x7F80_0000) == 0x7F80_0000
        && (bits & 0x003F_FFFF) != 0
        && (bits & 0x0040_0000) == 0
}

/// Apply FZ flush-to-zero to an input. When `v` is denormal, always set IDC;
/// when FZ=1 additionally replace the value with a signed zero. Mirrors
/// `ftz_input` in the emulator.
#[inline]
fn ftz_input(fpscr_in: u32, flags: &mut u32, v: f32) -> f32 {
    if is_denormal_f32(v) {
        *flags |= IDC;
        if fpscr_in & FZ != 0 {
            return if v.is_sign_negative() { -0.0 } else { 0.0 };
        }
    }
    v
}

/// Apply FZ flush-to-zero to an op's result. Returns `Some(flushed)` when
/// FZ=1 *and* the unrounded exact magnitude is tininess-before-rounding;
/// caller must then set UFC+IXC and use the flushed value. Returns `None`
/// when no flush is required (either FZ=0, result is NaN/inf, or exact
/// magnitude is >= MIN_NORMAL). Mirrors `ftz_output` in the emulator.
#[inline]
fn ftz_output(fpscr_in: u32, result: f32, exact: f64) -> Option<f32> {
    if fpscr_in & FZ == 0 {
        return None;
    }
    if result.is_nan() || result.is_infinite() {
        return None;
    }
    let abs_exact = exact.abs();
    if abs_exact >= MIN_NORMAL_F64 || abs_exact == 0.0 {
        return None;
    }
    Some(if result.is_sign_negative() { -0.0 } else { 0.0 })
}

/// Apply Default NaN (DN=1): replace any NaN with canonical quiet NaN.
/// If the value is not a NaN, returns it unchanged. For the oracle we don't
/// need to reproduce ARM's NaN canonicalization priority rules — both the
/// emulator and oracle settle on *some* NaN, and the diff harness treats
/// NaN-vs-NaN as equal when DN=0 (payload propagation is not contracted,
/// HLD §A.3). With DN=1 both sides must emit exactly 0x7FC0_0000.
#[inline]
fn apply_dn(fpscr_in: u32, result: f32) -> f32 {
    if fpscr_in & DN != 0 && result.is_nan() {
        f32::from_bits(ARM_DEFAULT_NAN)
    } else if result.is_nan() {
        // Base-case canonicalization (quiet NaN) when DN=0.
        // Oracle emits *a* NaN; payload equality with the emulator is not
        // contracted. See `softfloat_diff.rs` — NaN-vs-NaN compares equal.
        canonicalize_nan_unary(result)
    } else {
        result
    }
}

/// Overflow detection: the op produced ±inf as a rounded result without
/// either input being ±inf.
fn overflowed(result: f32, inputs_any_inf: bool) -> bool {
    result.is_infinite() && !inputs_any_inf
}

/// Underflow detection (tininess before rounding + inexact): the arithmetic
/// result magnitude, computed in extended precision, is below the smallest
/// normal f32, and the f32 result differs from the exact value.
fn underflowed(result: f32, exact: f64) -> bool {
    if result.is_nan() || result.is_infinite() {
        return false;
    }
    let abs_exact = exact.abs();
    if abs_exact == 0.0 {
        return false;
    }
    abs_exact < MIN_NORMAL_F64 && (result as f64) != exact
}

/// Apply canonical-NaN output rule (for unary / non-DN paths where we just
/// need *some* quiet NaN).
fn canonicalize_nan_unary(result: f32) -> f32 {
    if result.is_nan() {
        f32::from_bits(ARM_DEFAULT_NAN)
    } else {
        result
    }
}

pub fn ref_add(a: f32, b: f32, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0;
    let a = ftz_input(fpscr_in, &mut flags, a);
    let b = ftz_input(fpscr_in, &mut flags, b);

    if is_snan_f32(a) || is_snan_f32(b) {
        flags |= IOC;
    }
    // inf + (-inf) is invalid.
    if a.is_infinite() && b.is_infinite() && a.is_sign_negative() != b.is_sign_negative() {
        flags |= IOC;
        return (apply_dn(fpscr_in, a + b), flags);
    }

    let exact = (a as f64) + (b as f64);
    let result = a + b;

    if result.is_nan() {
        return (apply_dn(fpscr_in, result), flags);
    }
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        flags |= OFC | IXC;
    } else if let Some(flushed) = ftz_output(fpscr_in, result, exact) {
        flags |= UFC | IXC;
        return (flushed, flags);
    } else if underflowed(result, exact) {
        flags |= UFC | IXC;
    } else if (result as f64) != exact {
        flags |= IXC;
    }
    (result, flags)
}

pub fn ref_sub(a: f32, b: f32, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0;
    let a = ftz_input(fpscr_in, &mut flags, a);
    let b = ftz_input(fpscr_in, &mut flags, b);

    if is_snan_f32(a) || is_snan_f32(b) {
        flags |= IOC;
    }
    // inf - inf (same sign) is invalid.
    if a.is_infinite() && b.is_infinite() && a.is_sign_negative() == b.is_sign_negative() {
        flags |= IOC;
        return (apply_dn(fpscr_in, a - b), flags);
    }

    let exact = (a as f64) - (b as f64);
    let result = a - b;

    if result.is_nan() {
        return (apply_dn(fpscr_in, result), flags);
    }
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        flags |= OFC | IXC;
    } else if let Some(flushed) = ftz_output(fpscr_in, result, exact) {
        flags |= UFC | IXC;
        return (flushed, flags);
    } else if underflowed(result, exact) {
        flags |= UFC | IXC;
    } else if (result as f64) != exact {
        flags |= IXC;
    }
    (result, flags)
}

pub fn ref_mul(a: f32, b: f32, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0;
    let a = ftz_input(fpscr_in, &mut flags, a);
    let b = ftz_input(fpscr_in, &mut flags, b);

    if is_snan_f32(a) || is_snan_f32(b) {
        flags |= IOC;
    }
    // 0 * inf is invalid. (Post-FTZ: a flushed denormal becomes 0, which
    // correctly makes `inf * denormal` invalid under FZ=1, matching the
    // emulator's `is_mul_inf_zero` check ordering.)
    if (a.is_infinite() && b == 0.0) || (a == 0.0 && b.is_infinite()) {
        flags |= IOC;
        return (apply_dn(fpscr_in, a * b), flags);
    }

    let exact = (a as f64) * (b as f64);
    let result = a * b;

    if result.is_nan() {
        return (apply_dn(fpscr_in, result), flags);
    }
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        flags |= OFC | IXC;
    } else if let Some(flushed) = ftz_output(fpscr_in, result, exact) {
        flags |= UFC | IXC;
        return (flushed, flags);
    } else if underflowed(result, exact) {
        flags |= UFC | IXC;
    } else if (result as f64) != exact {
        flags |= IXC;
    }
    (result, flags)
}

pub fn ref_div(a: f32, b: f32, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0;
    let a = ftz_input(fpscr_in, &mut flags, a);
    let b = ftz_input(fpscr_in, &mut flags, b);

    if is_snan_f32(a) || is_snan_f32(b) {
        flags |= IOC;
    }
    // 0/0 and inf/inf are invalid.
    if (a == 0.0 && b == 0.0) || (a.is_infinite() && b.is_infinite()) {
        flags |= IOC;
        return (apply_dn(fpscr_in, a / b), flags);
    }
    // Finite nonzero / 0 → divide by zero. Post-FTZ: 1 / denormal when FZ=1
    // becomes 1 / 0, producing inf with DZC — matches emulator.
    if b == 0.0 && a.is_finite() && a != 0.0 {
        flags |= DZC;
        return (a / b, flags);
    }

    let exact = (a as f64) / (b as f64);
    let result = a / b;

    if result.is_nan() {
        return (apply_dn(fpscr_in, result), flags);
    }
    if overflowed(result, a.is_infinite() || b.is_infinite()) {
        flags |= OFC | IXC;
    } else if let Some(flushed) = ftz_output(fpscr_in, result, exact) {
        flags |= UFC | IXC;
        return (flushed, flags);
    } else if underflowed(result, exact) {
        flags |= UFC | IXC;
    } else {
        // Residual test: a − q·b; any non-zero residual → inexact.
        let residual = (-(result as f64)).mul_add(b as f64, a as f64);
        if residual != 0.0 {
            flags |= IXC;
        }
    }
    (result, flags)
}

pub fn ref_sqrt(a: f32, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0;
    let a = ftz_input(fpscr_in, &mut flags, a);

    if is_snan_f32(a) {
        flags |= IOC;
    }
    // sqrt(negative finite non-zero) is invalid. sqrt(-0) = -0 (allowed).
    if a.is_sign_negative() && a != 0.0 && !a.is_nan() {
        flags |= IOC;
        let result = if a.is_infinite() {
            f32::from_bits(ARM_DEFAULT_NAN)
        } else {
            a.sqrt() // produces NaN
        };
        return (apply_dn(fpscr_in, result), flags);
    }

    let result = a.sqrt();
    if result.is_nan() {
        return (apply_dn(fpscr_in, result), flags);
    }
    if !result.is_finite() || result == 0.0 {
        return (result, flags);
    }
    // Residual: r² − x in f64 via fused op.
    let residual = (result as f64).mul_add(result as f64, -(a as f64));
    if residual != 0.0 {
        flags |= IXC;
    }
    (result, flags)
}

/// Reference fused multiply-add: `a * b + c` with a single rounding.
///
/// Uses `f64::mul_add` to compute both the reference result and the exactness
/// probe. The `+fma` target feature is compile-asserted at the top of this
/// module so the host always emits a true fused op.
//
// f64 mul_add is our IXC probe; for worst-case operands requiring >53 bits
// of precision, the probe itself rounds and may miss inexactness. Accepted
// limit — not visible in current fuzz coverage.
pub fn ref_fma(a: f32, b: f32, c: f32, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0;
    let a = ftz_input(fpscr_in, &mut flags, a);
    let b = ftz_input(fpscr_in, &mut flags, b);
    let c = ftz_input(fpscr_in, &mut flags, c);

    if is_snan_f32(a) || is_snan_f32(b) || is_snan_f32(c) {
        flags |= IOC;
    }
    // 0 * inf in the product is invalid, regardless of addend.
    if (a.is_infinite() && b == 0.0) || (a == 0.0 && b.is_infinite()) {
        flags |= IOC;
        return (apply_dn(fpscr_in, a.mul_add(b, c)), flags);
    }

    let result = a.mul_add(b, c);
    if result.is_nan() {
        // (±inf) + (∓inf) via the product + addend path: IOC.
        let prod_sign = a.is_sign_negative() ^ b.is_sign_negative();
        let product_is_inf = (a.is_infinite() && b != 0.0 && !b.is_nan())
            || (b.is_infinite() && a != 0.0 && !a.is_nan());
        if product_is_inf && c.is_infinite() && prod_sign != c.is_sign_negative() {
            flags |= IOC;
        }
        return (apply_dn(fpscr_in, result), flags);
    }

    let exact = (a as f64).mul_add(b as f64, c as f64);
    if overflowed(
        result,
        a.is_infinite() || b.is_infinite() || c.is_infinite(),
    ) {
        flags |= OFC | IXC;
    } else if let Some(flushed) = ftz_output(fpscr_in, result, exact) {
        flags |= UFC | IXC;
        return (flushed, flags);
    } else if underflowed(result, exact) {
        flags |= UFC | IXC;
    } else if (result as f64) != exact {
        flags |= IXC;
    }
    (result, flags)
}

// ============================================================================
// Unit tests — the oracle against known values
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ref_add_exact_integers_no_flags() {
        let (r, f) = ref_add(1.0, 2.0, 0);
        assert_eq!(r, 3.0);
        assert_eq!(f, 0);
    }

    #[test]
    fn ref_add_inexact_sets_ixc() {
        // 1.0 + 2^-24 is inexact in f32 (one ULP below representable).
        let eps = f32::from_bits(0x3380_0000); // 2^-24
        let (_, f) = ref_add(1.0, eps, 0);
        assert!(f & IXC != 0, "expected IXC, got {f:#x}");
    }

    #[test]
    fn ref_add_inf_minus_inf_ioc() {
        let (r, f) = ref_add(f32::INFINITY, f32::NEG_INFINITY, 0);
        assert!(r.is_nan());
        assert_eq!(r.to_bits(), 0x7FC0_0000);
        assert!(f & IOC != 0);
    }

    #[test]
    fn ref_add_overflow_sets_ofc_ixc() {
        let big = f32::MAX;
        let (r, f) = ref_add(big, big, 0);
        assert!(r.is_infinite());
        assert!(f & OFC != 0);
        assert!(f & IXC != 0);
    }

    #[test]
    fn ref_div_by_zero_sets_dzc() {
        let (r, f) = ref_div(1.0, 0.0, 0);
        assert!(r.is_infinite() && r.is_sign_positive());
        assert!(f & DZC != 0);
        assert!(f & IOC == 0);
    }

    #[test]
    fn ref_div_zero_by_zero_sets_ioc() {
        let (r, f) = ref_div(0.0, 0.0, 0);
        assert!(r.is_nan());
        assert!(f & IOC != 0);
        assert!(f & DZC == 0);
    }

    #[test]
    fn ref_div_inf_by_inf_sets_ioc() {
        let (r, f) = ref_div(f32::INFINITY, f32::INFINITY, 0);
        assert!(r.is_nan());
        assert!(f & IOC != 0);
    }

    #[test]
    fn ref_div_one_by_three_sets_ixc() {
        let (_, f) = ref_div(1.0, 3.0, 0);
        assert!(f & IXC != 0);
    }

    #[test]
    fn ref_mul_inf_times_zero_sets_ioc() {
        let (r, f) = ref_mul(f32::INFINITY, 0.0, 0);
        assert!(r.is_nan());
        assert!(f & IOC != 0);
    }

    #[test]
    fn ref_mul_overflow_sets_ofc() {
        let (_, f) = ref_mul(1e20, 1e20, 0);
        assert!(f & OFC != 0);
    }

    #[test]
    fn ref_mul_underflow_sets_ufc() {
        // 1e-20 * 1e-20 = 1e-40, below MIN_NORMAL ≈ 1.18e-38.
        let (_, f) = ref_mul(1e-20, 1e-20, 0);
        assert!(f & UFC != 0, "expected UFC, got flags {f:#x}");
        assert!(f & IXC != 0);
    }

    #[test]
    fn ref_sqrt_negative_sets_ioc() {
        let (r, f) = ref_sqrt(-1.0, 0);
        assert!(r.is_nan());
        assert!(f & IOC != 0);
    }

    #[test]
    fn ref_sqrt_two_sets_ixc() {
        let (_, f) = ref_sqrt(2.0, 0);
        assert!(f & IXC != 0);
    }

    #[test]
    fn ref_sqrt_four_exact() {
        let (r, f) = ref_sqrt(4.0, 0);
        assert_eq!(r, 2.0);
        assert_eq!(f, 0);
    }

    #[test]
    fn ref_input_denormal_sets_idc() {
        let denorm = f32::from_bits(0x0000_0001); // smallest positive subnormal
        let (_, f) = ref_add(denorm, 1.0, 0);
        assert!(f & IDC != 0);
    }

    #[test]
    fn ref_fma_exact_no_flags() {
        let (r, f) = ref_fma(2.0, 3.0, 1.0, 0);
        assert_eq!(r, 7.0);
        assert_eq!(f, 0);
    }

    #[test]
    fn ref_fma_snan_sets_ioc() {
        let snan = f32::from_bits(0x7F80_0001);
        let (_, f) = ref_fma(snan, 1.0, 0.0, 0);
        assert!(f & IOC != 0);
    }

    // ------------------------------------------------------------------------
    // FZ (flush-to-zero) coverage
    // ------------------------------------------------------------------------

    #[test]
    fn ref_add_fz_flushes_denormal_input_to_zero_with_idc() {
        // With FZ=1 and a denormal input, the input is treated as +0; result
        // is 1.0 exactly and IDC is set.
        let denorm = f32::from_bits(0x0000_0001);
        let (r, f) = ref_add(denorm, 1.0, FZ);
        assert_eq!(r, 1.0);
        assert!(f & IDC != 0, "IDC must set when FZ=1 and input denormal");
        assert!(f & IXC == 0, "result is exact after flush; no IXC");
    }

    #[test]
    fn ref_add_fz_denormal_sign_preserved_in_flush() {
        // Negative denormal input flushes to -0.
        let neg_denorm = f32::from_bits(0x8000_0001);
        let (r, f) = ref_add(neg_denorm, 0.0, FZ);
        // -0 + 0 = 0 under round-to-nearest-even.
        assert_eq!(r.to_bits(), 0.0f32.to_bits());
        assert!(f & IDC != 0);
    }

    #[test]
    fn ref_mul_fz_flushes_tiny_result_with_ufc_ixc() {
        // 1e-20 * 1e-20 = 1e-40, tininess-before-rounding. With FZ=1, result
        // is flushed to +0 with UFC+IXC.
        let (r, f) = ref_mul(1e-20, 1e-20, FZ);
        assert_eq!(r.to_bits(), 0.0f32.to_bits(), "FZ=1 must flush tiny result to +0");
        assert!(f & UFC != 0, "UFC must set on FTZ output flush");
        assert!(f & IXC != 0, "IXC must set on FTZ output flush");
    }

    #[test]
    fn ref_mul_fz_inf_times_flushed_denormal_sets_ioc() {
        // Under FZ=1, a denormal operand flushes to 0 *before* the 0*inf
        // invalid-op check runs. So inf * denormal ⇒ inf * 0 ⇒ IOC,
        // matching the emulator's ordering.
        let denorm = f32::from_bits(0x0000_0001);
        let (r, f) = ref_mul(f32::INFINITY, denorm, FZ);
        assert!(r.is_nan());
        assert!(f & IOC != 0, "FZ-flushed denormal with inf must set IOC");
        assert!(f & IDC != 0, "IDC still set on denormal input");
    }

    #[test]
    fn ref_div_fz_nonzero_by_flushed_denormal_sets_dzc() {
        // Under FZ=1, b denormal flushes to 0; a/b then takes the n/0 path.
        let denorm = f32::from_bits(0x0000_0001);
        let (r, f) = ref_div(1.0, denorm, FZ);
        assert!(r.is_infinite() && r.is_sign_positive());
        assert!(f & DZC != 0, "FZ-flushed denominator must trigger DZC");
        assert!(f & IDC != 0, "IDC still set on denormal input");
    }

    #[test]
    fn ref_fma_fz_flushes_tiny_product_output() {
        // 1e-20 * 1e-20 + 0 = 1e-40, tiny; FZ=1 flushes to +0 with UFC+IXC.
        let (r, f) = ref_fma(1e-20, 1e-20, 0.0, FZ);
        assert_eq!(r.to_bits(), 0.0f32.to_bits());
        assert!(f & UFC != 0);
        assert!(f & IXC != 0);
    }

    #[test]
    fn ref_fz_zero_means_denormal_preserved() {
        // Same inputs as the flush tests, but FZ=0: denormal preserved, no UFC
        // from tiny-result flushing.
        let denorm = f32::from_bits(0x0000_0001);
        let (r, f) = ref_add(denorm, 0.0, 0);
        assert_eq!(r.to_bits(), denorm.to_bits(), "FZ=0 preserves denormal input");
        assert!(f & IDC != 0, "IDC still set even with FZ=0");
    }

    // ------------------------------------------------------------------------
    // DN (default NaN) coverage
    // ------------------------------------------------------------------------

    #[test]
    fn ref_add_dn_replaces_nan_with_canonical() {
        // NaN + 1.0 produces NaN; DN=1 forces the canonical 0x7FC0_0000.
        let custom_nan = f32::from_bits(0x7FC1_2345);
        let (r, f) = ref_add(custom_nan, 1.0, DN);
        assert_eq!(
            r.to_bits(),
            0x7FC0_0000,
            "DN=1 must canonicalize NaN; got 0x{:08X}",
            r.to_bits()
        );
        // No sNaN input, so no IOC.
        assert!(f & IOC == 0);
    }

    #[test]
    fn ref_mul_dn_canonicalizes_invalid_op_nan() {
        // 0 * inf with DN=1: IOC set, result is the canonical NaN.
        let (r, f) = ref_mul(0.0, f32::INFINITY, DN);
        assert_eq!(r.to_bits(), 0x7FC0_0000);
        assert!(f & IOC != 0);
    }

    #[test]
    fn ref_sqrt_dn_canonicalizes_invalid_op_nan() {
        let (r, f) = ref_sqrt(-4.0, DN);
        assert_eq!(r.to_bits(), 0x7FC0_0000);
        assert!(f & IOC != 0);
    }

    #[test]
    fn ref_fma_dn_canonicalizes_nan() {
        // sNaN * 1 + 0 → NaN with IOC; DN=1 forces canonical.
        let snan = f32::from_bits(0x7F80_0001);
        let (r, f) = ref_fma(snan, 1.0, 0.0, DN);
        assert_eq!(r.to_bits(), 0x7FC0_0000);
        assert!(f & IOC != 0);
    }

    // ------------------------------------------------------------------------
    // Combined FZ=1 + DN=1
    // ------------------------------------------------------------------------

    #[test]
    fn ref_mul_fz_dn_flushed_denorm_and_canonical_nan() {
        // inf * denormal with FZ=1 → inf * 0 → IOC, NaN canonicalized by DN=1.
        let denorm = f32::from_bits(0x0000_0001);
        let (r, f) = ref_mul(f32::INFINITY, denorm, FZ | DN);
        assert_eq!(r.to_bits(), 0x7FC0_0000);
        assert!(f & IOC != 0);
        assert!(f & IDC != 0);
    }
}

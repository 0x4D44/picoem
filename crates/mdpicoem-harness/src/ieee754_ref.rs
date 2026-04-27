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
//
// Exclude `doctest` builds: rustdoc compiles doctests without the rustflags
// declared in `.cargo/config.toml`, so the `+fma` check would spuriously
// trigger. Doctests here do not execute FMA-sensitive code, and the real
// `cargo test` path still enforces the feature via `--lib`/`--bin`.
#[cfg(all(not(target_feature = "fma"), not(doctest)))]
compile_error!(
    "mdpicoem-harness requires the +fma target feature. \
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
/// `F32_MIN_NORMAL_F64` constant in `crates/mdrp2350/src/core/execute_fpu.rs`;
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
    (bits & 0x7F80_0000) == 0x7F80_0000 && (bits & 0x003F_FFFF) != 0 && (bits & 0x0040_0000) == 0
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

/// Reference VRINT (round to integer per `rmode`).
///
/// `exact = true` matches VRINTX (raises IXC when the rounded value differs
/// from the input); `exact = false` matches VRINTR/VRINTZ (no IXC tracking).
/// All variants set IDC on denormal input, flush input under FZ=1, raise IOC
/// on SNaN, and honor DN.
///
/// `rmode` encoding matches FPSCR[23:22]: 00=RN, 01=RP, 10=RM, 11=RZ.
pub fn ref_vrint(val: f32, rmode: u32, fpscr_in: u32, exact: bool) -> (f32, u32) {
    let mut flags = 0u32;
    let val = ftz_input(fpscr_in, &mut flags, val);

    if val.is_nan() {
        if is_snan_f32(val) {
            flags |= IOC;
        }
        // Quieten the NaN payload so DN=0 path is bit-stable across hosts;
        // `apply_dn` substitutes the canonical NaN under DN=1.
        let quiet = f32::from_bits(val.to_bits() | 0x0040_0000);
        return (apply_dn(fpscr_in, quiet), flags);
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
        flags |= IXC;
    }
    (rounded, flags)
}

// ============================================================================
// Half-precision (IEEE binary16) reference converters
// ============================================================================
//
// Independent implementations — these must not call into the emulator's
// half-precision helpers, since their job is to validate them.

#[inline]
fn is_snan_f16_bits(h: u16) -> bool {
    let exp = (h >> 10) & 0x1F;
    let frac = h & 0x3FF;
    exp == 0x1F && frac != 0 && (frac & 0x200) == 0
}

/// Reference VCVTB/VCVTT.F32.F16 — convert a half-precision encoding to f32.
///
/// Cortex-M33 has no FZ16 control, so f16 denormals are value-preserved into
/// f32 normals (no IDC). SNaN input raises IOC; DN=1 emits the f32 default
/// NaN; DN=0 propagates the f16 payload into the f32 fraction with the quiet
/// bit forced. `apply_dn` is the only DN choke point.
pub fn ref_vcvt_f32_from_f16(h: u16, fpscr_in: u32) -> (f32, u32) {
    let mut flags = 0u32;
    let sign = ((h as u32) & 0x8000) << 16;
    let exp = ((h as u32) >> 10) & 0x1F;
    let frac = (h as u32) & 0x3FF;

    let bits = if exp == 0 {
        if frac == 0 {
            sign
        } else {
            // Subnormal half → normalized f32. Re-derive without the emulator's
            // shift loop. Let `lz` be the count of leading zero bits above
            // bit 9 of the 10-bit fraction (so lz ∈ [0, 9]). The highest set
            // bit is at position `9 - lz`, the value is `2^(9-lz-24)`, and the
            // biased f32 exponent is `(9 - lz) - 24 + 127 = 112 - lz`.
            let lz = (frac as u16).leading_zeros() - 6;
            let mantissa = (frac << (lz + 1)) & 0x3FF;
            let exp32 = (112 - lz) as u32;
            sign | (exp32 << 23) | (mantissa << 13)
        }
    } else if exp == 0x1F {
        if frac == 0 {
            sign | 0x7F80_0000
        } else {
            if is_snan_f16_bits(h) {
                flags |= IOC;
            }
            sign | 0x7F80_0000 | (frac << 13) | 0x0040_0000
        }
    } else {
        let exp32 = (exp as i32 - 15 + 127) as u32;
        sign | (exp32 << 23) | (frac << 13)
    };
    (apply_dn(fpscr_in, f32::from_bits(bits)), flags)
}

/// Reference VCVTB/VCVTT.F16.F32 — convert f32 → f16 with round-to-nearest-even.
///
/// f32 denormal input flushes to f16 ±0 with IDC (denormal f32 magnitudes are
/// well below the smallest f16 subnormal). SNaN input raises IOC; DN=1 emits
/// the f16 default NaN (0x7E00).
pub fn ref_vcvt_f16_from_f32(v: f32, fpscr_in: u32) -> (u16, u32) {
    let mut flags = 0u32;
    let bits = v.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exp = ((bits >> 23) & 0xFF) as i32;
    let frac = bits & 0x7F_FFFF;

    if exp == 0xFF {
        let h = if frac == 0 {
            sign | 0x7C00
        } else {
            if is_snan_f32(v) {
                flags |= IOC;
            }
            if fpscr_in & DN != 0 {
                0x7E00
            } else {
                let payload = (frac >> 13) as u16;
                sign | 0x7E00 | (payload & 0x1FF)
            }
        };
        return (h, flags);
    }
    if exp == 0 {
        if frac != 0 {
            flags |= IDC;
        }
        return (sign, flags);
    }

    let e = exp - 127;
    if e > 15 {
        return (sign | 0x7C00, flags);
    }
    if e < -24 {
        return (sign, flags);
    }
    if e < -14 {
        // Subnormal f16 result. Build the unrounded mantissa/round/sticky from
        // the f32 fields directly — no shared helpers with the emulator.
        let m = (frac | 0x0080_0000) as u64;
        let shift: u32 = (-e - 1) as u32;
        let mantissa = (m >> shift) as u32;
        let round_bit = if shift == 0 {
            0
        } else {
            ((m >> (shift - 1)) & 1) as u32
        };
        let sticky = if shift < 2 {
            false
        } else {
            (m & ((1u64 << (shift - 1)) - 1)) != 0
        };
        let lsb = mantissa & 1;
        let rounded = mantissa
            + if round_bit != 0 && (sticky || lsb != 0) {
                1
            } else {
                0
            };
        if rounded >= 0x400 {
            return (sign | (1 << 10), flags);
        }
        return (sign | (rounded as u16 & 0x3FF), flags);
    }

    let exp16 = (e + 15) as u16;
    let mantissa = frac >> 13;
    let round_bit = (frac >> 12) & 1;
    let sticky = (frac & 0xFFF) != 0;
    let lsb = mantissa & 1;
    let rounded = mantissa
        + if round_bit != 0 && (sticky || lsb != 0) {
            1
        } else {
            0
        };

    if rounded > 0x3FF {
        let new_exp = exp16 + 1;
        if new_exp >= 0x1F {
            return (sign | 0x7C00, flags);
        }
        return (sign | (new_exp << 10), flags);
    }
    (sign | (exp16 << 10) | (rounded as u16 & 0x3FF), flags)
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
        assert_eq!(
            r.to_bits(),
            0.0f32.to_bits(),
            "FZ=1 must flush tiny result to +0"
        );
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
        assert_eq!(
            r.to_bits(),
            denorm.to_bits(),
            "FZ=0 preserves denormal input"
        );
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

    // ------------------------------------------------------------------------
    // VRINT
    // ------------------------------------------------------------------------

    #[test]
    fn ref_vrint_integer_input_no_flags() {
        // Round-to-nearest of an exact integer leaves it alone, no flags.
        let (r, f) = ref_vrint(3.0, 0b00, 0, true);
        assert_eq!(r, 3.0);
        assert_eq!(f, 0);
    }

    #[test]
    fn ref_vrint_inexact_sets_ixc_only_when_exact() {
        let (r1, f1) = ref_vrint(2.5, 0b00, 0, true); // VRINTX
        assert_eq!(r1, 2.0); // round-to-even
        assert!(f1 & IXC != 0, "VRINTX should raise IXC on inexact");
        let (r2, f2) = ref_vrint(2.5, 0b00, 0, false); // VRINTR
        assert_eq!(r2, 2.0);
        assert!(f2 & IXC == 0, "VRINTR must not raise IXC");
    }

    #[test]
    fn ref_vrint_snan_sets_ioc_and_dn_canonicalizes() {
        let snan = f32::from_bits(0x7F80_0001);
        let (r, f) = ref_vrint(snan, 0b00, DN, false);
        assert_eq!(r.to_bits(), 0x7FC0_0000);
        assert!(f & IOC != 0);
    }

    #[test]
    fn ref_vrint_qnan_dn_off_no_ioc() {
        // QNaN input is not signaling — no IOC. Under DN=0 the oracle emits
        // *a* QNaN (payload-collapse is intentional; the diff harness treats
        // NaN-vs-NaN as equal under DN=0, see ieee754_ref::apply_dn).
        let qnan = f32::from_bits(0x7FCD_EAD0);
        let (r, f) = ref_vrint(qnan, 0b00, 0, true);
        assert!(r.is_nan());
        assert!(f & IOC == 0);
    }

    #[test]
    fn ref_vrint_denormal_input_sets_idc_fz_flushes() {
        let denorm = f32::from_bits(0x0000_0001);
        let (_, f) = ref_vrint(denorm, 0b00, 0, true);
        assert!(f & IDC != 0);
        let (r, f) = ref_vrint(denorm, 0b00, FZ, true);
        assert_eq!(r, 0.0);
        assert!(f & IDC != 0);
        // Flushed input is exact zero → no IXC, even with `exact = true`.
        assert!(f & IXC == 0);
    }

    #[test]
    fn ref_vrint_rmode_dispatch() {
        // 1.5 with each rmode.
        assert_eq!(ref_vrint(1.5, 0b00, 0, false).0, 2.0); // RN: round-to-even
        assert_eq!(ref_vrint(1.5, 0b01, 0, false).0, 2.0); // RP: ceil
        assert_eq!(ref_vrint(1.5, 0b10, 0, false).0, 1.0); // RM: floor
        assert_eq!(ref_vrint(1.5, 0b11, 0, false).0, 1.0); // RZ: trunc
    }

    // ------------------------------------------------------------------------
    // VCVT.F32.F16 / VCVT.F16.F32
    // ------------------------------------------------------------------------

    #[test]
    fn ref_vcvt_f32_from_f16_basic_values() {
        // ±0
        assert_eq!(ref_vcvt_f32_from_f16(0x0000, 0).0, 0.0);
        assert_eq!(ref_vcvt_f32_from_f16(0x8000, 0).0.to_bits(), 0x8000_0000);
        // 1.0 in f16 = 0x3C00 → 1.0 in f32
        assert_eq!(ref_vcvt_f32_from_f16(0x3C00, 0).0, 1.0);
        // ±inf
        assert!(ref_vcvt_f32_from_f16(0x7C00, 0).0.is_infinite());
        assert!(ref_vcvt_f32_from_f16(0xFC00, 0).0.is_infinite());
    }

    #[test]
    fn ref_vcvt_f32_from_f16_subnormal_smallest() {
        // f16 0x0001 = smallest subnormal = 2^-24 = 0x33800000 in f32.
        let (r, f) = ref_vcvt_f32_from_f16(0x0001, 0);
        assert_eq!(r.to_bits(), 0x3380_0000);
        assert_eq!(f, 0);
    }

    #[test]
    fn ref_vcvt_f32_from_f16_subnormal_largest() {
        // f16 0x03FF = largest subnormal = (1023/1024) * 2^-14
        // = 0.999... * 2^-14 = (1 + 1022/1024) * 2^-15 ≈ 6.097e-5
        // f32 encoding: exp=112, frac=0x7FC000 → 0x387FC000.
        let (r, _) = ref_vcvt_f32_from_f16(0x03FF, 0);
        assert_eq!(r.to_bits(), 0x387F_C000);
    }

    #[test]
    fn ref_vcvt_f32_from_f16_snan_sets_ioc_dn_canonicalizes() {
        // f16 SNaN: exp=0x1F, frac non-zero, quiet bit (bit 9) clear.
        let snan = 0x7C01_u16; // exp=11111, frac=0000000001
        let (_, f) = ref_vcvt_f32_from_f16(snan, 0);
        assert!(f & IOC != 0);
        let (r, _) = ref_vcvt_f32_from_f16(snan, DN);
        assert_eq!(r.to_bits(), 0x7FC0_0000);
    }

    #[test]
    fn ref_vcvt_f16_from_f32_basic_values() {
        assert_eq!(ref_vcvt_f16_from_f32(0.0, 0).0, 0x0000);
        assert_eq!(ref_vcvt_f16_from_f32(-0.0, 0).0, 0x8000);
        assert_eq!(ref_vcvt_f16_from_f32(1.0, 0).0, 0x3C00);
        assert_eq!(ref_vcvt_f16_from_f32(f32::INFINITY, 0).0, 0x7C00);
        assert_eq!(ref_vcvt_f16_from_f32(f32::NEG_INFINITY, 0).0, 0xFC00);
    }

    #[test]
    fn ref_vcvt_f16_from_f32_overflow_to_inf() {
        // 2^16 overflows half-precision (max normal exp = 15).
        let big = f32::from_bits(0x4780_0000); // 65536.0
        assert_eq!(ref_vcvt_f16_from_f32(big, 0).0, 0x7C00);
    }

    #[test]
    fn ref_vcvt_f16_from_f32_denormal_input_sets_idc_flushes() {
        let denorm = f32::from_bits(0x0000_0001);
        let (h, f) = ref_vcvt_f16_from_f32(denorm, 0);
        assert_eq!(h, 0x0000);
        assert!(f & IDC != 0);
    }

    #[test]
    fn ref_vcvt_f16_from_f32_snan_sets_ioc_dn_canonicalizes() {
        let snan = f32::from_bits(0x7F80_0001);
        let (_, f) = ref_vcvt_f16_from_f32(snan, 0);
        assert!(f & IOC != 0);
        let (h, _) = ref_vcvt_f16_from_f32(snan, DN);
        assert_eq!(h, 0x7E00);
    }

    #[test]
    fn ref_vcvt_f16_from_f32_round_trip_subnormals() {
        // For every f16 subnormal, f16 → f32 → f16 must be the identity.
        for h in 0x0001u16..=0x03FF {
            let (val, _) = ref_vcvt_f32_from_f16(h, 0);
            let (back, _) = ref_vcvt_f16_from_f32(val, 0);
            assert_eq!(back, h, "round trip failed for f16 0x{:04X}", h);
        }
    }
}

// ============================================================================
// DCP (CP4/5 double-precision coprocessor) reference oracle — Phase 7 Stage D
// ============================================================================
//
// Unlike the single-precision FPU, the DCP doesn't have an FPSCR. Its status
// register tracks four bits of the result only:
//
//   bit 0 — zero
//   bit 1 — negative (sign bit set on result)
//   bit 2 — infinity
//   bit 3 — NaN
//
// No stickiness; each arithmetic op overwrites the register. No FZ/DN control;
// we use native host f64 (IEEE-754 bit-exact).
//
// The reference is "host f64 op + derive the four status bits from the
// result." Emulator and reference run the identical host op with identical
// inputs, so bit equality is the only interesting test — but locking in the
// status-bit derivation in *one* place is still worth it: if we ever move
// the emulator to a non-trivial f64 implementation, this stays the oracle.

/// DCP status bit masks.
pub const DCP_Z: u32 = 1 << 0;
pub const DCP_N: u32 = 1 << 1;
pub const DCP_INF: u32 = 1 << 2;
pub const DCP_NAN: u32 = 1 << 3;

#[inline]
fn dcp_status_from(r: f64) -> u32 {
    let mut s = 0u32;
    if r == 0.0 {
        s |= DCP_Z;
    }
    if r.is_sign_negative() {
        s |= DCP_N;
    }
    if r.is_infinite() {
        s |= DCP_INF;
    }
    if r.is_nan() {
        s |= DCP_NAN;
    }
    s
}

pub fn ref_dadd(a: f64, b: f64) -> (f64, u32) {
    let r = a + b;
    (r, dcp_status_from(r))
}

pub fn ref_dsub(a: f64, b: f64) -> (f64, u32) {
    let r = a - b;
    (r, dcp_status_from(r))
}

pub fn ref_dmul(a: f64, b: f64) -> (f64, u32) {
    let r = a * b;
    (r, dcp_status_from(r))
}

pub fn ref_ddiv(a: f64, b: f64) -> (f64, u32) {
    let r = a / b;
    (r, dcp_status_from(r))
}

pub fn ref_dsqrt(a: f64) -> (f64, u32) {
    let r = a.sqrt();
    (r, dcp_status_from(r))
}

pub fn ref_dcmp_eq(a: f64, b: f64) -> bool {
    a == b
}
pub fn ref_dcmp_lt(a: f64, b: f64) -> bool {
    a < b
}
pub fn ref_dcmp_le(a: f64, b: f64) -> bool {
    a <= b
}
pub fn ref_dcmp_gt(a: f64, b: f64) -> bool {
    a > b
}
pub fn ref_dcmp_ge(a: f64, b: f64) -> bool {
    a >= b
}

/// Reference i32 → f64 conversion.
pub fn ref_i2d(i: i32) -> f64 {
    i as f64
}
/// Reference u32 → f64 conversion.
pub fn ref_u2d(u: u32) -> f64 {
    u as f64
}
/// Reference f64 → i32 (truncating, Rust `as` semantics).
pub fn ref_d2i(d: f64) -> i32 {
    d as i32
}
/// Reference f64 → u32 (truncating).
pub fn ref_d2u(d: f64) -> u32 {
    d as u32
}
/// Reference f64 → f32 (rounding per host f64-to-f32 convert).
pub fn ref_d2f(d: f64) -> f32 {
    d as f32
}
/// Reference f32 → f64 (exact, f32 is a subset of f64).
pub fn ref_f2d(f: f32) -> f64 {
    f as f64
}

#[cfg(test)]
mod dcp_tests {
    use super::*;

    #[test]
    fn ref_dadd_exact_integers_status_clear() {
        let (r, s) = ref_dadd(1.0, 2.0);
        assert_eq!(r, 3.0);
        assert_eq!(s, 0);
    }

    #[test]
    fn ref_dsub_produces_zero() {
        let (r, s) = ref_dsub(3.0, 3.0);
        assert_eq!(r, 0.0);
        assert_eq!(s & DCP_Z, DCP_Z);
    }

    #[test]
    fn ref_dsub_produces_negative() {
        let (r, s) = ref_dsub(1.0, 2.0);
        assert_eq!(r, -1.0);
        assert_eq!(s & DCP_N, DCP_N);
        assert_eq!(s & DCP_Z, 0);
    }

    #[test]
    fn ref_ddiv_one_by_zero_plus_inf() {
        let (r, s) = ref_ddiv(1.0, 0.0);
        assert!(r.is_infinite() && r.is_sign_positive());
        assert_eq!(s & DCP_INF, DCP_INF);
        assert_eq!(s & DCP_Z, 0);
        assert_eq!(s & DCP_N, 0);
    }

    #[test]
    fn ref_ddiv_neg_one_by_zero_minus_inf() {
        let (r, s) = ref_ddiv(-1.0, 0.0);
        assert!(r.is_infinite() && r.is_sign_negative());
        assert_eq!(s & DCP_INF, DCP_INF);
        assert_eq!(s & DCP_N, DCP_N);
    }

    #[test]
    fn ref_ddiv_zero_by_zero_nan() {
        let (r, s) = ref_ddiv(0.0, 0.0);
        assert!(r.is_nan());
        assert_eq!(s & DCP_NAN, DCP_NAN);
    }

    #[test]
    fn ref_dsqrt_four() {
        let (r, s) = ref_dsqrt(4.0);
        assert_eq!(r, 2.0);
        assert_eq!(s, 0);
    }

    #[test]
    fn ref_dsqrt_neg_produces_nan() {
        let (r, s) = ref_dsqrt(-1.0);
        assert!(r.is_nan());
        assert_eq!(s & DCP_NAN, DCP_NAN);
    }

    #[test]
    fn ref_dmul_large_overflow_to_inf() {
        let (r, s) = ref_dmul(1e200, 1e200);
        assert!(r.is_infinite());
        assert_eq!(s & DCP_INF, DCP_INF);
    }

    #[test]
    fn ref_dcmp_basic_predicates() {
        assert!(ref_dcmp_eq(1.0, 1.0));
        assert!(!ref_dcmp_eq(1.0, 2.0));
        assert!(ref_dcmp_lt(1.0, 2.0));
        assert!(!ref_dcmp_lt(2.0, 1.0));
        assert!(!ref_dcmp_lt(1.0, 1.0));
        assert!(ref_dcmp_le(1.0, 1.0));
        assert!(ref_dcmp_gt(2.0, 1.0));
        assert!(ref_dcmp_ge(2.0, 2.0));
    }

    #[test]
    fn ref_dcmp_nan_all_false() {
        let nan = f64::NAN;
        assert!(!ref_dcmp_eq(nan, 1.0));
        assert!(!ref_dcmp_lt(nan, 1.0));
        assert!(!ref_dcmp_le(nan, 1.0));
        assert!(!ref_dcmp_gt(nan, 1.0));
        assert!(!ref_dcmp_ge(nan, 1.0));
    }

    #[test]
    fn ref_convert_roundtrip() {
        // i2d + d2i roundtrip.
        let i = -0x1234_5678_i32;
        let d = ref_i2d(i);
        assert_eq!(d, i as f64);
        assert_eq!(ref_d2i(d), i);

        // u2d + d2u.
        let u = 0xFEDC_BA98_u32;
        let d = ref_u2d(u);
        assert_eq!(d, u as f64);
        assert_eq!(ref_d2u(d), u);

        // f2d is exact; d2f may round.
        let f = 3.5f32;
        assert_eq!(ref_f2d(f), f as f64);
        assert_eq!(ref_d2f(f as f64), f);
    }
}

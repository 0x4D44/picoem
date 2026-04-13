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
            return self.thumb32_undefined(hw0, hw1);
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
                // VMLA.F32 Sd, Sn, Sm — Sd += Sn*Sm
                let d = self.regs.s[sd];
                self.regs.s[sd] = d + self.regs.s[sn] * self.regs.s[sm];
                3
            }
            (0, 0b00, 1) => {
                // VMLS.F32 Sd, Sn, Sm — Sd -= Sn*Sm
                let d = self.regs.s[sd];
                self.regs.s[sd] = d - self.regs.s[sn] * self.regs.s[sm];
                3
            }
            (0, 0b01, 0) => {
                // VNMLS.F32 Sd, Sn, Sm — Sd = Sn*Sm - Sd
                let d = self.regs.s[sd];
                self.regs.s[sd] = self.regs.s[sn] * self.regs.s[sm] - d;
                3
            }
            (0, 0b01, 1) => {
                // VNMLA.F32 Sd, Sn, Sm — Sd = -(Sn*Sm + Sd)
                let d = self.regs.s[sd];
                self.regs.s[sd] = -(self.regs.s[sn] * self.regs.s[sm] + d);
                3
            }
            (0, 0b10, 0) => {
                // VMUL.F32 Sd, Sn, Sm
                self.regs.s[sd] = self.regs.s[sn] * self.regs.s[sm];
                1
            }
            (0, 0b10, 1) => {
                // VNMUL.F32 Sd, Sn, Sm — Sd = -(Sn * Sm)
                self.regs.s[sd] = -(self.regs.s[sn] * self.regs.s[sm]);
                1
            }
            (0, 0b11, 0) => {
                // VADD.F32 Sd, Sn, Sm
                self.regs.s[sd] = self.regs.s[sn] + self.regs.s[sm];
                1
            }
            (0, 0b11, 1) => {
                // VSUB.F32 Sd, Sn, Sm
                self.regs.s[sd] = self.regs.s[sn] - self.regs.s[sm];
                1
            }
            (1, 0b00, 0) => {
                // VDIV.F32 Sd, Sn, Sm
                self.regs.s[sd] = self.regs.s[sn] / self.regs.s[sm];
                14
            }
            (1, 0b01, 0) => {
                // VFNMS.F32 Sd, Sn, Sm — Sd = Sn*Sm - Sd (fused)
                let d = self.regs.s[sd];
                self.regs.s[sd] = self.regs.s[sn].mul_add(self.regs.s[sm], -d);
                3
            }
            (1, 0b01, 1) => {
                // VFNMA.F32 Sd, Sn, Sm — Sd = -(Sn*Sm + Sd) (fused)
                let d = self.regs.s[sd];
                self.regs.s[sd] = (-self.regs.s[sn]).mul_add(self.regs.s[sm], -d);
                3
            }
            (1, 0b10, 0) => {
                // VFMA.F32 Sd, Sn, Sm — Sd = Sd + Sn*Sm (fused)
                let d = self.regs.s[sd];
                self.regs.s[sd] = self.regs.s[sn].mul_add(self.regs.s[sm], d);
                3
            }
            (1, 0b10, 1) => {
                // VFMS.F32 Sd, Sn, Sm — Sd = Sd - Sn*Sm (fused)
                let d = self.regs.s[sd];
                self.regs.s[sd] = (-self.regs.s[sn]).mul_add(self.regs.s[sm], d);
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
            _ => self.thumb32_undefined(hw0, hw1),
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
                self.regs.s[sd] = self.regs.s[sm].sqrt();
                14
            }
            (0b0010, 0) => {
                // VCVTB.F16.F32 — half-precision conversion (stub)
                self.thumb32_undefined(hw0, hw1)
            }
            (0b0010, 1) => {
                // VCVTT.F16.F32 — half-precision conversion (stub)
                self.thumb32_undefined(hw0, hw1)
            }
            (0b0011, 0) => {
                // VCVTB.F32.F16 — half-precision conversion (stub)
                self.thumb32_undefined(hw0, hw1)
            }
            (0b0011, 1) => {
                // VCVTT.F32.F16 — half-precision conversion (stub)
                self.thumb32_undefined(hw0, hw1)
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
                // VRINTR.F32 Sd, Sm — round per FPSCR.RMode
                let rmode = fpscr_rmode(self.regs.fpscr);
                self.regs.s[sd] = fpu_vrint(self.regs.s[sm], rmode);
                1
            }
            (0b0110, 1) => {
                // VRINTZ.F32 Sd, Sm — round toward zero
                self.regs.s[sd] = self.regs.s[sm].trunc();
                1
            }
            (0b0111, 0) => {
                // VRINTX.F32 Sd, Sm — round per FPSCR.RMode (exact, may set FPSCR.IXC)
                let rmode = fpscr_rmode(self.regs.fpscr);
                self.regs.s[sd] = fpu_vrint(self.regs.s[sm], rmode);
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
                self.thumb32_undefined(hw0, hw1)
            }
            (0b1010, 1) => {
                // VCVT.F32.FX.S16 — fixed-point, stub
                self.thumb32_undefined(hw0, hw1)
            }
            (0b1011, 0) => {
                // VCVT.F32.FX.U32 — fixed-point, stub
                self.thumb32_undefined(hw0, hw1)
            }
            (0b1011, 1) => {
                // VCVT.F32.FX.S32 — fixed-point, stub
                self.thumb32_undefined(hw0, hw1)
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
                self.thumb32_undefined(hw0, hw1)
            }
            (0b1110, 1) => {
                // VCVT.FX.S16.F32 — fixed-point, stub
                self.thumb32_undefined(hw0, hw1)
            }
            (0b1111, 0) => {
                // VCVT.FX.U32.F32 — fixed-point, stub
                self.thumb32_undefined(hw0, hw1)
            }
            (0b1111, 1) => {
                // VCVT.FX.S32.F32 — fixed-point, stub
                self.thumb32_undefined(hw0, hw1)
            }
            _ => self.thumb32_undefined(hw0, hw1),
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
                self.regs.s[sd] = f32::from_bits(bus.read32(addr));
                2
            } else {
                // VSTR.32
                bus.write32(addr, self.regs.s[sd].to_bits());
                1
            }
        } else {
            // VLDM / VSTM / VPUSH / VPOP (multiple registers)
            let count = imm8 as usize;
            if count == 0 {
                return self.thumb32_undefined(hw0, hw1);
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
                    self.regs.s[reg] = f32::from_bits(bus.read32(addr));
                } else {
                    bus.write32(addr, self.regs.s[reg].to_bits());
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
}

// ============================================================================
// VRINT helper
// ============================================================================

fn fpu_vrint(val: f32, rmode: u32) -> f32 {
    match rmode {
        0b00 => val.round_ties_even(),
        0b01 => val.ceil(),
        0b10 => val.floor(),
        _ => val.trunc(),
    }
}

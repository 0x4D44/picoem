// Thumb-32 encoding helpers and test generators for QEMU differential testing.
//
// Each encoder returns a `(u16, u16)` halfword pair that the emulator's
// `execute_thumb32` dispatch will route to the correct handler.  Every bit
// layout was verified against the decoder in `execute_thumb32.rs` and
// `decode.rs`.

#[allow(dead_code)]

// When test generators are added later, they'll use:
//   use crate::*;

// ============================================================================
// Op-code constants — data processing (modified immediate / shifted register)
// ============================================================================

pub const DP_AND: u16 = 0;
pub const DP_BIC: u16 = 1;
pub const DP_ORR: u16 = 2;
pub const DP_ORN: u16 = 3;
pub const DP_EOR: u16 = 4;
pub const DP_ADD: u16 = 8;
pub const DP_ADC: u16 = 10;
pub const DP_SBC: u16 = 11;
pub const DP_SUB: u16 = 13;
pub const DP_RSB: u16 = 14;

pub const SHIFT_LSL: u16 = 0;
pub const SHIFT_LSR: u16 = 1;
pub const SHIFT_ASR: u16 = 2;
pub const SHIFT_ROR: u16 = 3;

// ============================================================================
// 1. Data processing — modified immediate
// ============================================================================
//
// hw0: 11110_i_0_op[3:0]_S_Rn[3:0]
// hw1: 0_imm3_Rd[3:0]_imm8
// imm12 = i:imm3:imm8

pub fn enc_t32_dp_mod_imm(op: u16, s: bool, rn: u16, rd: u16, imm12: u16) -> (u16, u16) {
    let i = (imm12 >> 11) & 1;
    let imm3 = (imm12 >> 8) & 0x7;
    let imm8 = imm12 & 0xFF;

    let hw0 = 0b11110_0_0_0000_0_0000u16
        | (i << 10)
        | ((op & 0xF) << 5)
        | (u16::from(s) << 4)
        | (rn & 0xF);

    let hw1 = (imm3 << 12) | ((rd & 0xF) << 8) | imm8;

    (hw0, hw1)
}

// ============================================================================
// 2. Data processing — shifted register
// ============================================================================
//
// hw0: 11101_01_op[3:0]_S_Rn[3:0]
// hw1: 0_imm3_Rd[3:0]_imm2_type[1:0]_Rm[3:0]
// shift_amount = imm3:imm2 (5 bits, imm3=top3, imm2=bottom2)

pub fn enc_t32_dp_shift_reg(
    op: u16, s: bool, rn: u16, rd: u16, rm: u16, stype: u16, samount: u16,
) -> (u16, u16) {
    let imm3 = (samount >> 2) & 0x7;
    let imm2 = samount & 0x3;

    let hw0 = 0b11101_01_0000_0_0000u16
        | ((op & 0xF) << 5)
        | (u16::from(s) << 4)
        | (rn & 0xF);

    let hw1 = (imm3 << 12)
        | ((rd & 0xF) << 8)
        | (imm2 << 6)
        | ((stype & 0x3) << 4)
        | (rm & 0xF);

    (hw0, hw1)
}

// ============================================================================
// 3. Data processing — plain binary immediate
// ============================================================================

/// Split a 16-bit immediate into the scattered MOVW/MOVT fields:
/// imm16 = imm4:i:imm3:imm8  →  hw0 gets imm4 in [3:0] and i in [10],
/// hw1 gets imm3 in [14:12] and imm8 in [7:0].
#[inline]
fn scatter_imm16(base_hw0: u16, rd: u16, imm16: u16) -> (u16, u16) {
    let imm4 = (imm16 >> 12) & 0xF;
    let i    = (imm16 >> 11) & 1;
    let imm3 = (imm16 >> 8) & 0x7;
    let imm8 = imm16 & 0xFF;

    let hw0 = base_hw0 | (i << 10) | imm4;
    let hw1 = (imm3 << 12) | ((rd & 0xF) << 8) | imm8;
    (hw0, hw1)
}

/// MOVW Rd, #imm16
pub fn enc_t32_movw(rd: u16, imm16: u16) -> (u16, u16) {
    // hw0 base: 11110_i_10_0100_Rn=0000 → 0xF240
    scatter_imm16(0xF240, rd, imm16)
}

/// MOVT Rd, #imm16
pub fn enc_t32_movt(rd: u16, imm16: u16) -> (u16, u16) {
    // hw0 base: 11110_i_10_1100_Rn=0000 → 0xF2C0
    scatter_imm16(0xF2C0, rd, imm16)
}

/// Scatter a 12-bit immediate for ADDW/SUBW:
/// imm12 = i:imm3:imm8 → hw0 bit10=i, hw1 bits [14:12]=imm3, [7:0]=imm8.
#[inline]
fn scatter_imm12(base_hw0: u16, rd: u16, rn: u16, imm12: u16) -> (u16, u16) {
    let i    = (imm12 >> 11) & 1;
    let imm3 = (imm12 >> 8) & 0x7;
    let imm8 = imm12 & 0xFF;

    let hw0 = base_hw0 | (i << 10) | (rn & 0xF);
    let hw1 = (imm3 << 12) | ((rd & 0xF) << 8) | imm8;
    (hw0, hw1)
}

/// ADDW Rd, Rn, #imm12
pub fn enc_t32_addw(rd: u16, rn: u16, imm12: u16) -> (u16, u16) {
    // hw0 base: 11110_i_10_0000_Rn → 0xF200
    scatter_imm12(0xF200, rd, rn, imm12)
}

/// SUBW Rd, Rn, #imm12
pub fn enc_t32_subw(rd: u16, rn: u16, imm12: u16) -> (u16, u16) {
    // hw0 base: 11110_i_10_1010_Rn → 0xF2A0
    scatter_imm12(0xF2A0, rd, rn, imm12)
}

/// Encode lsb from a 5-bit value split as imm3:imm2 into hw1 fields.
#[inline]
fn pack_lsb(lsb: u16) -> u16 {
    let imm3 = (lsb >> 2) & 0x7;
    let imm2 = lsb & 0x3;
    (imm3 << 12) | (imm2 << 6)
}

/// BFI Rd, Rn, #lsb, #width  (BFC when Rn=15)
/// Decoder: op=0b10110, lsb = imm3:imm2, msb = hw1[4:0] where msb = lsb + width - 1
pub fn enc_t32_bfi(rd: u16, rn: u16, lsb: u16, width: u16) -> (u16, u16) {
    let msb = lsb + width - 1;
    // hw0: 11110_0_11_0110_0_Rn → 0xF360 | Rn
    let hw0 = 0xF360 | (rn & 0xF);
    let hw1 = pack_lsb(lsb) | ((rd & 0xF) << 8) | (msb & 0x1F);
    (hw0, hw1)
}

/// Encode BFC (Bit Field Clear) — BFI with Rn=15.
pub fn enc_t32_bfc(rd: u16, lsb: u16, width: u16) -> (u16, u16) {
    enc_t32_bfi(rd, 15, lsb, width)
}

/// SBFX Rd, Rn, #lsb, #width
/// Decoder: op=0b10100, widthm1 = hw1[4:0] = width-1
pub fn enc_t32_sbfx(rd: u16, rn: u16, lsb: u16, width: u16) -> (u16, u16) {
    // hw0: 11110_0_11_0100_0_Rn → 0xF340 | Rn
    let hw0 = 0xF340 | (rn & 0xF);
    let hw1 = pack_lsb(lsb) | ((rd & 0xF) << 8) | ((width - 1) & 0x1F);
    (hw0, hw1)
}

/// UBFX Rd, Rn, #lsb, #width
/// Decoder: op=0b11100, widthm1 = hw1[4:0] = width-1
pub fn enc_t32_ubfx(rd: u16, rn: u16, lsb: u16, width: u16) -> (u16, u16) {
    // hw0: 11110_0_11_1100_0_Rn → 0xF3C0 | Rn
    let hw0 = 0xF3C0 | (rn & 0xF);
    let hw1 = pack_lsb(lsb) | ((rd & 0xF) << 8) | ((width - 1) & 0x1F);
    (hw0, hw1)
}

/// SSAT Rd, #sat, Rn, shift_type, shift_amount
/// Decoder: op=0b10000 (LSL) or 0b10010 (ASR).
///          sat_bit = hw1[4:0] + 1   → encode (sat-1).
///          shift_n = imm3:imm2.
pub fn enc_t32_ssat(rd: u16, rn: u16, sat: u16, stype: u16, samount: u16) -> (u16, u16) {
    // sh bit: 0 for LSL, 1 for ASR → op = 0b10000 | (sh << 1)
    let sh = if stype == SHIFT_ASR { 1u16 } else { 0u16 };
    // hw0: 11110_0_11_0000_0_Rn | (sh << 5) → base 0xF300
    let hw0 = 0xF300 | (sh << 5) | (rn & 0xF);
    let hw1 = pack_lsb(samount) | ((rd & 0xF) << 8) | ((sat - 1) & 0x1F);
    (hw0, hw1)
}

/// USAT Rd, #sat, Rn, shift_type, shift_amount
/// Decoder: op=0b11000 (LSL) or 0b11010 (ASR).
///          sat_bit = hw1[4:0].
///          shift_n = imm3:imm2.
pub fn enc_t32_usat(rd: u16, rn: u16, sat: u16, stype: u16, samount: u16) -> (u16, u16) {
    let sh = if stype == SHIFT_ASR { 1u16 } else { 0u16 };
    // hw0: 11110_0_11_1000_0_Rn | (sh << 5) → base 0xF380
    let hw0 = 0xF380 | (sh << 5) | (rn & 0xF);
    let hw1 = pack_lsb(samount) | ((rd & 0xF) << 8) | (sat & 0x1F);
    (hw0, hw1)
}

// ============================================================================
// 4. Load/store single
// ============================================================================
//
// The decoder (`thumb32_load_store_single`) reads:
//   size  = hw0[6:5]   (00=byte, 01=half, 10=word)
//   load  = hw0[4]     (1=load, 0=store)
//   sign  = hw0[8]     (1=signed load; only meaningful on loads)
//   Rn    = hw0[3:0]
//   Rt    = hw1[15:12]
//
// Three addressing modes, distinguished by hw0[7] and hw1[11]:
//
//   a) imm12 unsigned offset : hw0[7]=1
//   b) imm8 with P/U/W       : hw0[7]=0, hw1[11]=1
//   c) register offset        : hw0[7]=0, hw1[11]=0

/// Build the common hw0 bits for load/store single.
#[inline]
fn ls_hw0(size: u16, load: bool, signed: bool, rn: u16) -> u16 {
    // hw0 top 5 bits = 11111 (op1=11, op2[6]=0, op2[5]=0 part)
    // Full: 1111_100S_sz[1]sz[0]_L_Rn  with the addressing-mode bits mixed in.
    0b1111_1000_0000_0000u16
        | ((signed as u16) << 8)
        | ((size & 0x3) << 5)
        | ((load as u16) << 4)
        | (rn & 0xF)
}

/// 12-bit unsigned offset mode.
pub fn enc_t32_ls_imm12(
    size: u16, load: bool, signed: bool, rn: u16, rt: u16, imm12: u16,
) -> (u16, u16) {
    // hw0[7] = 1 selects this mode
    let hw0 = ls_hw0(size, load, signed, rn) | (1 << 7);
    let hw1 = ((rt & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// 8-bit offset with P/U/W bits (pre-index, post-index, negative offset).
pub fn enc_t32_ls_imm8(
    size: u16, load: bool, signed: bool, rn: u16, rt: u16,
    p: bool, u: bool, w: bool, imm8: u16,
) -> (u16, u16) {
    // hw0[7] = 0, hw1[11] = 1 selects this mode
    let hw0 = ls_hw0(size, load, signed, rn);
    let hw1 = ((rt & 0xF) << 12)
        | (1 << 11)
        | ((p as u16) << 10)
        | ((u as u16) << 9)
        | ((w as u16) << 8)
        | (imm8 & 0xFF);
    (hw0, hw1)
}

/// Register offset with shift.
pub fn enc_t32_ls_reg(
    size: u16, load: bool, signed: bool, rn: u16, rt: u16, rm: u16, shift: u16,
) -> (u16, u16) {
    // hw0[7] = 0, hw1[11] = 0 selects this mode
    let hw0 = ls_hw0(size, load, signed, rn);
    let hw1 = ((rt & 0xF) << 12)
        | ((shift & 0x3) << 4)
        | (rm & 0xF);
    (hw0, hw1)
}

// ============================================================================
// 5. Load/store multiple and dual
// ============================================================================

/// STM{IA|DB}.W Rn{!}, reglist
///
/// hw0: 1110_100_op[1:0]_0_W_0_Rn
///   op=01 → IA, op=10 → DB
/// hw1: register list (16 bits)
pub fn enc_t32_stm(rn: u16, w: bool, db: bool, reglist: u16) -> (u16, u16) {
    let op = if db { 0b10u16 } else { 0b01u16 };
    let hw0 = 0b1110_100_00_0_0_0_0000u16
        | (op << 7)
        | ((w as u16) << 5)
        | (rn & 0xF);
    (hw0, reglist)
}

/// LDM{IA|DB}.W Rn{!}, reglist
///
/// hw0: 1110_100_op[1:0]_0_W_1_Rn
pub fn enc_t32_ldm(rn: u16, w: bool, db: bool, reglist: u16) -> (u16, u16) {
    let op = if db { 0b10u16 } else { 0b01u16 };
    let hw0 = 0b1110_100_00_0_0_1_0000u16
        | (op << 7)
        | ((w as u16) << 5)
        | (rn & 0xF);
    (hw0, reglist)
}

/// LDRD Rt, Rt2, [Rn, #±imm8*4]  (P/U/W)
///
/// hw0: 1110_100_P_U_1_W_1_Rn
/// hw1: Rt[15:12] Rt2[11:8] imm8[7:0]
pub fn enc_t32_ldrd(
    rt: u16, rt2: u16, rn: u16, p: bool, u: bool, w: bool, imm8: u16,
) -> (u16, u16) {
    let hw0 = 0b1110_100_0_0_1_0_1_0000u16
        | ((p as u16) << 8)
        | ((u as u16) << 7)
        | ((w as u16) << 5)
        | (rn & 0xF);
    let hw1 = ((rt & 0xF) << 12) | ((rt2 & 0xF) << 8) | (imm8 & 0xFF);
    (hw0, hw1)
}

/// STRD Rt, Rt2, [Rn, #±imm8*4]  (P/U/W)
///
/// hw0: 1110_100_P_U_1_W_0_Rn
/// hw1: Rt[15:12] Rt2[11:8] imm8[7:0]
pub fn enc_t32_strd(
    rt: u16, rt2: u16, rn: u16, p: bool, u: bool, w: bool, imm8: u16,
) -> (u16, u16) {
    let hw0 = 0b1110_100_0_0_1_0_0_0000u16
        | ((p as u16) << 8)
        | ((u as u16) << 7)
        | ((w as u16) << 5)
        | (rn & 0xF);
    let hw1 = ((rt & 0xF) << 12) | ((rt2 & 0xF) << 8) | (imm8 & 0xFF);
    (hw0, hw1)
}

/// TBB [Rn, Rm]
///
/// Decoder check: hw0 & 0xFFF0 == 0xE8D0, hw1[15:12]=0xF, hw1[7:5]=0, hw1[4]=0
pub fn enc_t32_tbb(rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xE8D0 | (rn & 0xF);
    let hw1 = 0xF000 | (rm & 0xF);
    (hw0, hw1)
}

/// TBH [Rn, Rm, LSL #1]
///
/// Same as TBB but hw1[4]=1 (H bit).
pub fn enc_t32_tbh(rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xE8D0 | (rn & 0xF);
    let hw1 = 0xF010 | (rm & 0xF);
    (hw0, hw1)
}

// ============================================================================
// 6. Multiply / divide
// ============================================================================
//
// 32-bit result multiply: op1=11, op2[6]=0, op2[5]=1, op2[4]=0, op2[3]=0
//   hw0: 1111_1011_0_op1[2:0]_Rn
//   hw1: Ra[15:12]_Rd[11:8]_op2[1:0]_00_Rm[3:0]
//
// 64-bit long multiply/divide: op1=11, op2[6]=0, op2[5]=1, op2[4]=0, op2[3]=1
//   hw0: 1111_1011_1_op1[2:0]_Rn
//   hw1: RdLo[15:12]_RdHi[11:8]_op2[3:0]_Rm[3:0]

/// MUL Rd, Rn, Rm  (MLA with Ra=0xF)
pub fn enc_t32_mul(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    enc_t32_mla(rd, rn, rm, 0xF)
}

/// MLA Rd, Rn, Rm, Ra
/// op1=000, op2=00
pub fn enc_t32_mla(rd: u16, rn: u16, rm: u16, ra: u16) -> (u16, u16) {
    let hw0 = 0xFB00 | (rn & 0xF);
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// MLS Rd, Rn, Rm, Ra
/// op1=000, op2=01
pub fn enc_t32_mls(rd: u16, rn: u16, rm: u16, ra: u16) -> (u16, u16) {
    let hw0 = 0xFB00 | (rn & 0xF);
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (0b01 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMULL RdLo, RdHi, Rn, Rm
/// op1=000, op2=0000
pub fn enc_t32_smull(rdlo: u16, rdhi: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFB80 | (rn & 0xF);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// UMULL RdLo, RdHi, Rn, Rm
/// op1=010, op2=0000
pub fn enc_t32_umull(rdlo: u16, rdhi: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFBA0 | (rn & 0xF);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// SMLAL RdLo, RdHi, Rn, Rm
/// op1=100, op2=0000
pub fn enc_t32_smlal(rdlo: u16, rdhi: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFBC0 | (rn & 0xF);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// UMLAL RdLo, RdHi, Rn, Rm
/// op1=110, op2=0000
pub fn enc_t32_umlal(rdlo: u16, rdhi: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFBE0 | (rn & 0xF);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// SDIV Rd, Rn, Rm
/// Long-multiply path: op1=001, op2=1111
pub fn enc_t32_sdiv(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFB90 | (rn & 0xF);
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (0xF << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// UDIV Rd, Rn, Rm
/// Long-multiply path: op1=011, op2=1111
pub fn enc_t32_udiv(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFBB0 | (rn & 0xF);
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (0xF << 4) | (rm & 0xF);
    (hw0, hw1)
}

// ============================================================================
// 7. Branches
// ============================================================================
//
// The emulator decodes branches via `thumb32_branch_misc`:
//   hw1[14]=1           → BL
//   hw1[14]=0, hw1[12]=1 → B.W T4 (unconditional)
//   hw1[14]=0, hw1[12]=0, hw0[9:6]!=0b111x → B.W T3 (conditional)
//   otherwise            → misc control (MSR/MRS/hints)

/// B<cond>.W — T3 encoding, 21-bit signed offset.
///
/// hw0: 11110_S_cond[3:0]_imm6[5:0]
/// hw1: 10_J1_0_J2_imm11[10:0]
///
/// imm21 = S:J2:J1:imm6:imm11:0  (no XOR trick for T3)
pub fn enc_t32_b_cond(cond: u16, offset: i32) -> (u16, u16) {
    let uoff = offset as u32;
    let s     = (uoff >> 20) & 1;
    let j2    = (uoff >> 19) & 1;
    let j1    = (uoff >> 18) & 1;
    let imm6  = (uoff >> 12) & 0x3F;
    let imm11 = (uoff >> 1) & 0x7FF;

    let hw0 = 0xF000u16
        | ((s as u16) << 10)
        | ((cond & 0xF) << 6)
        | (imm6 as u16);

    let hw1 = 0x8000u16
        | ((j1 as u16) << 13)
        | ((j2 as u16) << 11)
        | (imm11 as u16);

    (hw0, hw1)
}

/// B.W — T4 encoding, 25-bit signed offset (unconditional).
///
/// hw0: 11110_S_imm10[9:0]
/// hw1: 10_J1_1_J2_imm11[10:0]
///
/// Uses the XOR trick: I1 = NOT(J1 XOR S), I2 = NOT(J2 XOR S)
/// imm25 = S:I1:I2:imm10:imm11:0
pub fn enc_t32_b_uncond(offset: i32) -> (u16, u16) {
    let uoff  = offset as u32;
    let s     = (uoff >> 24) & 1;
    let i1    = (uoff >> 23) & 1;
    let i2    = (uoff >> 22) & 1;
    let imm10 = (uoff >> 12) & 0x3FF;
    let imm11 = (uoff >> 1) & 0x7FF;

    // Reverse the XOR trick to get J1, J2
    let j1 = (i1 ^ s) ^ 1;
    let j2 = (i2 ^ s) ^ 1;

    let hw0 = 0xF000u16
        | ((s as u16) << 10)
        | (imm10 as u16);

    let hw1 = 0x9000u16
        | ((j1 as u16) << 13)
        | ((j2 as u16) << 11)
        | (imm11 as u16);

    (hw0, hw1)
}

/// BL — 25-bit signed offset, same J1/J2 XOR trick as B.W T4.
///
/// hw0: 11110_S_imm10[9:0]
/// hw1: 11_J1_1_J2_imm11[10:0]
pub fn enc_t32_bl(offset: i32) -> (u16, u16) {
    let uoff  = offset as u32;
    let s     = (uoff >> 24) & 1;
    let i1    = (uoff >> 23) & 1;
    let i2    = (uoff >> 22) & 1;
    let imm10 = (uoff >> 12) & 0x3FF;
    let imm11 = (uoff >> 1) & 0x7FF;

    let j1 = (i1 ^ s) ^ 1;
    let j2 = (i2 ^ s) ^ 1;

    let hw0 = 0xF000u16
        | ((s as u16) << 10)
        | (imm10 as u16);

    let hw1 = 0xD000u16
        | ((j1 as u16) << 13)
        | ((j2 as u16) << 11)
        | (imm11 as u16);

    (hw0, hw1)
}

// ============================================================================
// 8. Miscellaneous / DSP / Register ops
// ============================================================================

// -- MSR / MRS ---------------------------------------------------------------
// MSR: hw0 = 11110_0_11100_R_Rn → 0xF380 | Rn  (R bit sets mask[1])
// MRS: hw0 = 11110_0_11111_R_1111 → 0xF3EF

/// MSR — write Rn to special register SYSm.
/// Encoding: hw0 = 0xF380 | Rn, hw1 = 0x8800 | sysm  (mask=0b10 for NZCVQ)
pub fn enc_t32_msr(rn: u16, sysm: u16) -> (u16, u16) {
    let hw0 = 0xF380 | (rn & 0xF);
    // hw1: 10_00_mask_00_SYSm — mask=0b10 covers NZCVQ flags
    let hw1 = 0x8800 | (sysm & 0xFF);
    (hw0, hw1)
}

/// MRS — read special register SYSm into Rd.
/// Encoding: hw0 = 0xF3EF, hw1 = 0x8000 | (Rd << 8) | sysm
pub fn enc_t32_mrs(rd: u16, sysm: u16) -> (u16, u16) {
    let hw0 = 0xF3EF;
    let hw1 = 0x8000 | ((rd & 0xF) << 8) | (sysm & 0xFF);
    (hw0, hw1)
}

// -- CLZ / RBIT / REV family -------------------------------------------------
// Decoder: thumb32_dp_register with hw0[7]=1, hw0[4]=1
//   hw0 = 1111_1010_1_op1[1:0]_1_Rm(=Rn)
//   hw1 = 1111_Rd_1000_op2[1:0]_Rm
//   Rn appears in hw0[3:0] but the decoder reads Rm from hw1[3:0].
//   For CLZ: op1=01, op2=00.  For REV: op1=00, op2=00. etc.

/// CLZ Rd, Rm
pub fn enc_t32_clz(rd: u16, rm: u16) -> (u16, u16) {
    // op1_lo=01, op2_lo=00 → hw0[6:5]=01, hw1[5:4]=00
    let hw0 = 0xFAB0 | (rm & 0xF);
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// RBIT Rd, Rm
pub fn enc_t32_rbit(rd: u16, rm: u16) -> (u16, u16) {
    // op1_lo=00, op2_lo=10
    let hw0 = 0xFA90 | (rm & 0xF);
    let hw1 = 0xF0A0 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// REV.W Rd, Rm
pub fn enc_t32_rev_w(rd: u16, rm: u16) -> (u16, u16) {
    // op1_lo=00, op2_lo=00
    let hw0 = 0xFA90 | (rm & 0xF);
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// REV16.W Rd, Rm
pub fn enc_t32_rev16_w(rd: u16, rm: u16) -> (u16, u16) {
    // op1_lo=00, op2_lo=01
    let hw0 = 0xFA90 | (rm & 0xF);
    let hw1 = 0xF090 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// REVSH.W Rd, Rm
pub fn enc_t32_revsh_w(rd: u16, rm: u16) -> (u16, u16) {
    // op1_lo=00, op2_lo=11
    let hw0 = 0xFA90 | (rm & 0xF);
    let hw1 = 0xF0B0 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

// -- Wide register shifts (by register) --------------------------------------
// Decoder: thumb32_dp_register, hw0[7]=0, hw1[7]=0
//   hw0 = 1111_1010_0_stype[1:0]_S_Rn
//   hw1 = 1111_Rd_0000_Rm

/// LSL.W Rd, Rn, Rm
pub fn enc_t32_lsl_w_reg(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA00 | (rn & 0xF);  // stype=00, S=0
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// LSR.W Rd, Rn, Rm
pub fn enc_t32_lsr_w_reg(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA20 | (rn & 0xF);  // stype=01, S=0
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// ASR.W Rd, Rn, Rm
pub fn enc_t32_asr_w_reg(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA40 | (rn & 0xF);  // stype=10, S=0
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// ROR.W Rd, Rn, Rm
pub fn enc_t32_ror_w_reg(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA60 | (rn & 0xF);  // stype=11, S=0
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

// -- Extend instructions (with rotation) -------------------------------------
// Decoder: thumb32_dp_register, hw0[7]=0, hw1[7]=1
//   hw0 = 1111_1010_0_ext[2:0]_Rn  (Rn=15 → plain extend)
//   hw1 = 1111_Rd_1_0_rot[1:0]_Rm
//
// ext: 000=SXTH, 001=UXTH, 100=SXTB, 101=UXTB

/// SXTB.W Rd, Rm, {ROR #rot}  (rot = 0, 8, 16, 24)
pub fn enc_t32_sxtb_w(rd: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA4F;  // ext=100, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// UXTB.W Rd, Rm, {ROR #rot}
pub fn enc_t32_uxtb_w(rd: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA5F;  // ext=101, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SXTH.W Rd, Rm, {ROR #rot}
pub fn enc_t32_sxth_w(rd: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA0F;  // ext=000, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// UXTH.W Rd, Rm, {ROR #rot}
pub fn enc_t32_uxth_w(rd: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA1F;  // ext=001, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Saturating ops (QADD/QSUB/QDADD/QDSUB) ---------------------------------
// Decoder: thumb32_dp_register, hw0[7]=1, hw1[7]=1, hw0[4]=0
//   hw0 = 1111_1010_1_op1[1:0]_0_Rn  (op1_65=hw0[6:5])
//   hw1 = 1111_Rd_1000_op2[1:0]_Rm   (op2_54=hw1[5:4])
//
// QADD:  op1_65=00, op2_54=00
// QDADD: op1_65=00, op2_54=01
// QSUB:  op1_65=00, op2_54=10
// QDSUB: op1_65=00, op2_54=11

/// QADD Rd, Rn, Rm
pub fn enc_t32_qadd(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA80 | (rn & 0xF);
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// QSUB Rd, Rn, Rm
pub fn enc_t32_qsub(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA80 | (rn & 0xF);
    let hw1 = 0xF0A0 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// QDADD Rd, Rn, Rm
pub fn enc_t32_qdadd(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA80 | (rn & 0xF);
    let hw1 = 0xF090 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// QDSUB Rd, Rn, Rm
pub fn enc_t32_qdsub(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA80 | (rn & 0xF);
    let hw1 = 0xF0B0 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

// -- Parallel add/subtract ---------------------------------------------------
// Decoder: thumb32_dp_register, hw0[7]=1, hw1[7]=0
//   hw0 = 1111_1010_1_op1[2:0]_Rn      (par_op1 = hw0[6:4])
//   hw1 = 1111_Rd_0_op2[2:0]_Rm        (par_op2 = hw1[6:4])

/// Parameterised parallel add/sub encoder.
/// `prefix` = par_op1 (3 bits), `op` = par_op2 (3 bits).
pub fn enc_t32_parallel(prefix: u16, op: u16, rn: u16, rd: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA80 | ((prefix & 0x7) << 4) | (rn & 0xF);
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | ((op & 0x7) << 4) | (rm & 0xF);
    (hw0, hw1)
}

// ============================================================================
// Unit tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Verify the imm12 field is correctly scattered for data-processing
    /// modified immediate.  The decoder reconstructs imm12 as i:imm3:imm8.
    #[test]
    fn dp_mod_imm_imm12_scatter() {
        // imm12 = 0xABC → i=1, imm3=0b010, imm8=0xBC
        let (hw0, hw1) = enc_t32_dp_mod_imm(DP_ADD, true, 3, 5, 0xABC);
        // Extract imm12 back the way the decoder does
        let i = ((hw0 >> 10) & 1) as u32;
        let imm3 = ((hw1 >> 12) & 0x7) as u32;
        let imm8 = (hw1 & 0xFF) as u32;
        let recovered = (i << 11) | (imm3 << 8) | imm8;
        assert_eq!(recovered, 0xABC);
        // Check op=ADD=8 in hw0[8:5]
        assert_eq!((hw0 >> 5) & 0xF, DP_ADD);
        // S bit
        assert_ne!(hw0 & (1 << 4), 0);
        // Rn=3
        assert_eq!(hw0 & 0xF, 3);
        // Rd=5
        assert_eq!((hw1 >> 8) & 0xF, 5);
    }

    /// Verify MOVW immediate scatter: imm16 = imm4:i:imm3:imm8.
    #[test]
    fn movw_imm16_scatter() {
        // imm16 = 0xBEEF
        //   imm4 = 0xB, i = 1, imm3 = 0b110, imm8 = 0xEF
        let (hw0, hw1) = enc_t32_movw(7, 0xBEEF);
        let imm4 = (hw0 & 0xF) as u32;
        let i    = ((hw0 >> 10) & 1) as u32;
        let imm3 = ((hw1 >> 12) & 0x7) as u32;
        let imm8 = (hw1 & 0xFF) as u32;
        let recovered = (imm4 << 12) | (i << 11) | (imm3 << 8) | imm8;
        assert_eq!(recovered, 0xBEEF);
        assert_eq!((hw1 >> 8) & 0xF, 7); // Rd
    }

    /// Verify MOVT uses the same scatter but different base.
    #[test]
    fn movt_base() {
        let (hw0, _) = enc_t32_movt(0, 0);
        // Should start with 0xF2C0
        assert_eq!(hw0 & 0xFBF0, 0xF2C0);
    }

    /// Verify BL J1/J2 XOR trick roundtrips.
    /// offset=+256 → small positive, S=0, I1=1, I2=1 → J1=0^0^1=1, J2=0^0^1=1
    #[test]
    fn bl_xor_roundtrip_positive() {
        let offset = 256i32;
        let (hw0, hw1) = enc_t32_bl(offset);
        // Decode
        let s = ((hw0 >> 10) & 1) as u32;
        let imm10 = (hw0 & 0x3FF) as u32;
        let j1 = ((hw1 >> 13) & 1) as u32;
        let j2 = ((hw1 >> 11) & 1) as u32;
        let imm11 = (hw1 & 0x7FF) as u32;
        let i1 = (j1 ^ s) ^ 1;
        let i2 = (j2 ^ s) ^ 1;
        let imm25 = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
        // Sign-extend 25 bits
        let decoded = ((imm25 as i32) << 7) >> 7;
        assert_eq!(decoded, offset);
    }

    /// Verify BL with a negative offset roundtrips.
    #[test]
    fn bl_xor_roundtrip_negative() {
        let offset = -1024i32;
        let (hw0, hw1) = enc_t32_bl(offset);
        let s = ((hw0 >> 10) & 1) as u32;
        let imm10 = (hw0 & 0x3FF) as u32;
        let j1 = ((hw1 >> 13) & 1) as u32;
        let j2 = ((hw1 >> 11) & 1) as u32;
        let imm11 = (hw1 & 0x7FF) as u32;
        let i1 = (j1 ^ s) ^ 1;
        let i2 = (j2 ^ s) ^ 1;
        let imm25 = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
        let decoded = ((imm25 as i32) << 7) >> 7;
        assert_eq!(decoded, offset);
    }

    /// Verify B.W conditional (T3) roundtrips a positive offset.
    #[test]
    fn b_cond_roundtrip() {
        let offset = 0x1000i32; // +4096
        let (hw0, hw1) = enc_t32_b_cond(0xA, offset); // cond=GE
        let s = ((hw0 >> 10) & 1) as u32;
        let cond = (hw0 >> 6) & 0xF;
        let imm6 = (hw0 & 0x3F) as u32;
        let j1 = ((hw1 >> 13) & 1) as u32;
        let j2 = ((hw1 >> 11) & 1) as u32;
        let imm11 = (hw1 & 0x7FF) as u32;
        let imm21 = (s << 20) | (j2 << 19) | (j1 << 18) | (imm6 << 12) | (imm11 << 1);
        let decoded = ((imm21 as i32) << 11) >> 11;
        assert_eq!(decoded, offset);
        assert_eq!(cond, 0xA);
    }

    /// Load/store imm12 mode selects hw0[7]=1.
    #[test]
    fn ls_imm12_mode() {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, true, false, 3, 5, 100);
        assert_ne!(hw0 & (1 << 7), 0); // imm12 mode
        assert_eq!((hw0 >> 5) & 0x3, 0b10); // word size
        assert_ne!(hw0 & (1 << 4), 0); // load
        assert_eq!(hw0 & 0xF, 3); // Rn
        assert_eq!((hw1 >> 12) & 0xF, 5); // Rt
        assert_eq!(hw1 & 0xFFF, 100); // imm12
    }

    /// Load/store imm8 mode selects hw0[7]=0, hw1[11]=1.
    #[test]
    fn ls_imm8_mode() {
        let (hw0, hw1) = enc_t32_ls_imm8(0b01, true, false, 2, 4, true, true, false, 42);
        assert_eq!(hw0 & (1 << 7), 0); // NOT imm12 mode
        assert_ne!(hw1 & (1 << 11), 0); // imm8 selector
        assert_ne!(hw1 & (1 << 10), 0); // P
        assert_ne!(hw1 & (1 << 9), 0);  // U
        assert_eq!(hw1 & (1 << 8), 0);  // W=false
        assert_eq!(hw1 & 0xFF, 42);
    }

    /// Shifted register: verify shift_amount split as imm3:imm2.
    #[test]
    fn dp_shift_reg_shift_amount_split() {
        // samount=13 = 0b01101 → imm3=0b011, imm2=0b01
        let (hw0, hw1) = enc_t32_dp_shift_reg(DP_ADD, false, 1, 2, 3, SHIFT_LSL, 13);
        let imm3 = (hw1 >> 12) & 0x7;
        let imm2 = (hw1 >> 6) & 0x3;
        assert_eq!((imm3 << 2) | imm2, 13);
        // Verify routing: op1=01, op2 top bits = 01
        assert_eq!((hw0 >> 11) & 0x3, 0b01); // op1
        assert_eq!(((hw0 >> 4) & 0x7F) >> 5, 0b01); // op2[6:5]
    }
}

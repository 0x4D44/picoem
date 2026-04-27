// Thumb-32 encoding helpers and test generators for QEMU differential testing.
//
// Each encoder returns a `(u16, u16)` halfword pair that the emulator's
// `execute_thumb32` dispatch will route to the correct handler.  Every bit
// layout was verified against the decoder in `execute_thumb32.rs` and
// `decode.rs`.
//
// Underscore positions inside binary literals here document Thumb-32
// instruction-encoding bit-fields (op:Rn:S:imm), not 4-bit visual groups
// — clippy's uniform-grouping suggestion would erase that documentation.
#![allow(clippy::unusual_byte_groupings)]

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
    op: u16,
    s: bool,
    rn: u16,
    rd: u16,
    rm: u16,
    stype: u16,
    samount: u16,
) -> (u16, u16) {
    let imm3 = (samount >> 2) & 0x7;
    let imm2 = samount & 0x3;

    let hw0 = 0b11101_01_0000_0_0000u16 | ((op & 0xF) << 5) | (u16::from(s) << 4) | (rn & 0xF);

    let hw1 = (imm3 << 12) | ((rd & 0xF) << 8) | (imm2 << 6) | ((stype & 0x3) << 4) | (rm & 0xF);

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
    let i = (imm16 >> 11) & 1;
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
    let i = (imm12 >> 11) & 1;
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
    size: u16,
    load: bool,
    signed: bool,
    rn: u16,
    rt: u16,
    imm12: u16,
) -> (u16, u16) {
    // hw0[7] = 1 selects this mode
    let hw0 = ls_hw0(size, load, signed, rn) | (1 << 7);
    let hw1 = ((rt & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// 8-bit offset with P/U/W bits (pre-index, post-index, negative offset).
pub fn enc_t32_ls_imm8(
    size: u16,
    load: bool,
    signed: bool,
    rn: u16,
    rt: u16,
    p: bool,
    u: bool,
    w: bool,
    imm8: u16,
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
    size: u16,
    load: bool,
    signed: bool,
    rn: u16,
    rt: u16,
    rm: u16,
    shift: u16,
) -> (u16, u16) {
    // hw0[7] = 0, hw1[11] = 0 selects this mode
    let hw0 = ls_hw0(size, load, signed, rn);
    let hw1 = ((rt & 0xF) << 12) | ((shift & 0x3) << 4) | (rm & 0xF);
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
    let hw0 = 0b1110_100_00_0_0_0_0000u16 | (op << 7) | ((w as u16) << 5) | (rn & 0xF);
    (hw0, reglist)
}

/// LDM{IA|DB}.W Rn{!}, reglist
///
/// hw0: 1110_100_op[1:0]_0_W_1_Rn
pub fn enc_t32_ldm(rn: u16, w: bool, db: bool, reglist: u16) -> (u16, u16) {
    let op = if db { 0b10u16 } else { 0b01u16 };
    let hw0 = 0b1110_100_00_0_0_1_0000u16 | (op << 7) | ((w as u16) << 5) | (rn & 0xF);
    (hw0, reglist)
}

/// LDRD Rt, Rt2, [Rn, #±imm8*4]  (P/U/W)
///
/// hw0: 1110_100_P_U_1_W_1_Rn
/// hw1: Rt[15:12] Rt2[11:8] imm8[7:0]
pub fn enc_t32_ldrd(
    rt: u16,
    rt2: u16,
    rn: u16,
    p: bool,
    u: bool,
    w: bool,
    imm8: u16,
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
    rt: u16,
    rt2: u16,
    rn: u16,
    p: bool,
    u: bool,
    w: bool,
    imm8: u16,
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

// -- DSP halfword multiply (SMULBB/BT/TB/TT, SMLABB/BT/TB/TT) ---------------
// Decoder: thumb32_multiply, op1=001
//   hw0 = 1111_1011_0001_Rn   (0xFB10 | Rn)
//   hw1 = Ra_Rd_N_M_Rm        (Ra=15 → SMULXY, Ra!=15 → SMLABB etc.)
//
// op2 bits: bit1=N_high (Rn halfword), bit0=M_high (Rm halfword)
//   BB=00, BT=01, TB=10, TT=11

/// SMULXY Rd, Rn, Rm — halfword multiply (Ra=15, no accumulate).
/// `n_high`/`m_high` select top (true) or bottom (false) halfword.
pub fn enc_t32_smulxy(rd: u16, rn: u16, rm: u16, n_high: bool, m_high: bool) -> (u16, u16) {
    let hw0 = 0xFB10 | (rn & 0xF);
    let op2 = ((n_high as u16) << 1) | (m_high as u16);
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMLABB/BT/TB/TT Rd, Rn, Rm, Ra — halfword multiply-accumulate.
/// `n_high`/`m_high` select top (true) or bottom (false) halfword.
pub fn enc_t32_smlabb(
    rd: u16,
    rn: u16,
    rm: u16,
    ra: u16,
    n_high: bool,
    m_high: bool,
) -> (u16, u16) {
    let hw0 = 0xFB10 | (rn & 0xF);
    let op2 = ((n_high as u16) << 1) | (m_high as u16);
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Dual halfword multiply (SMUAD/SMUADX, SMLAD/SMLADX, SMUSD/SMUSDX, SMLSD/SMLSDX)
// Decoder: thumb32_multiply, op1=010 for add / op1=100 for sub.
// hw0 = 0xFB20 (add) / 0xFB40 (sub)  | Rn
// hw1 = Ra_Rd_0_X_Rm   (Ra=15 → no-accumulate SMU*, Ra!=15 → SML*)

/// SMUAD / SMUADX Rd, Rn, Rm — dual halfword multiply-add (no accumulate).
/// `cross` swaps Rm halfwords before the two products (the X suffix).
pub fn enc_t32_smuad(rd: u16, rn: u16, rm: u16, cross: bool) -> (u16, u16) {
    let hw0 = 0xFB20 | (rn & 0xF);
    let op2 = cross as u16;
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMLAD / SMLADX Rd, Rn, Rm, Ra — dual halfword multiply-add-accumulate.
pub fn enc_t32_smlad(rd: u16, rn: u16, rm: u16, ra: u16, cross: bool) -> (u16, u16) {
    let hw0 = 0xFB20 | (rn & 0xF);
    let op2 = cross as u16;
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMUSD / SMUSDX Rd, Rn, Rm — dual halfword multiply-subtract (no accumulate).
pub fn enc_t32_smusd(rd: u16, rn: u16, rm: u16, cross: bool) -> (u16, u16) {
    let hw0 = 0xFB40 | (rn & 0xF);
    let op2 = cross as u16;
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMLSD / SMLSDX Rd, Rn, Rm, Ra — dual halfword multiply-subtract-accumulate.
pub fn enc_t32_smlsd(rd: u16, rn: u16, rm: u16, ra: u16, cross: bool) -> (u16, u16) {
    let hw0 = 0xFB40 | (rn & 0xF);
    let op2 = cross as u16;
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Word x halfword (SMULWB/SMULWT, SMLAWB/SMLAWT)
// Decoder: thumb32_multiply, op1=011
// hw0 = 0xFB30 | Rn
// hw1 = Ra_Rd_0_M_Rm   (M=1 → top halfword of Rm; Ra=15 → SMULW)

/// SMULWB / SMULWT Rd, Rn, Rm — word times halfword (no accumulate).
pub fn enc_t32_smulw(rd: u16, rn: u16, rm: u16, m_high: bool) -> (u16, u16) {
    let hw0 = 0xFB30 | (rn & 0xF);
    let op2 = m_high as u16;
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMLAWB / SMLAWT Rd, Rn, Rm, Ra — word times halfword multiply-accumulate.
pub fn enc_t32_smlaw(rd: u16, rn: u16, rm: u16, ra: u16, m_high: bool) -> (u16, u16) {
    let hw0 = 0xFB30 | (rn & 0xF);
    let op2 = m_high as u16;
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Most significant word multiply (SMMUL/SMMULR, SMMLA/SMMLAR, SMMLS/SMMLSR)
// Decoder: thumb32_multiply, op1=101 (mul/MLA) / op1=110 (MLS)
// hw0 = 0xFB50 (MUL/MLA) / 0xFB60 (MLS)  | Rn
// hw1 = Ra_Rd_0_R_Rm   (R=1 → rounding variant; Ra=15 → SMMUL)

/// SMMUL / SMMULR Rd, Rn, Rm — most significant word multiply.
/// `round` selects the rounding variant (R suffix).
pub fn enc_t32_smmul(rd: u16, rn: u16, rm: u16, round: bool) -> (u16, u16) {
    let hw0 = 0xFB50 | (rn & 0xF);
    let op2 = round as u16;
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMMLA / SMMLAR Rd, Rn, Rm, Ra — most significant word multiply-accumulate.
pub fn enc_t32_smmla(rd: u16, rn: u16, rm: u16, ra: u16, round: bool) -> (u16, u16) {
    let hw0 = 0xFB50 | (rn & 0xF);
    let op2 = round as u16;
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMMLS / SMMLSR Rd, Rn, Rm, Ra — most significant word multiply-subtract.
pub fn enc_t32_smmls(rd: u16, rn: u16, rm: u16, ra: u16, round: bool) -> (u16, u16) {
    let hw0 = 0xFB60 | (rn & 0xF);
    let op2 = round as u16;
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Sum of absolute differences (USAD8, USADA8)
// Decoder: thumb32_multiply, op1=111
// hw0 = 0xFB70 | Rn
// hw1 = Ra_Rd_0000_Rm  (Ra=15 → USAD8, else USADA8)

/// USAD8 Rd, Rn, Rm — sum of absolute differences (no accumulate).
pub fn enc_t32_usad8(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFB70 | (rn & 0xF);
    let hw1 = (0xF << 12) | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// USADA8 Rd, Rn, Rm, Ra — sum of absolute differences, accumulate.
pub fn enc_t32_usada8(rd: u16, rn: u16, rm: u16, ra: u16) -> (u16, u16) {
    let hw0 = 0xFB70 | (rn & 0xF);
    let hw1 = ((ra & 0xF) << 12) | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

// -- Long halfword multiply-accumulate (SMLAL<x><y>)
// Decoder: thumb32_long_multiply, op1=100, op2=1000..1011
// hw0 = 0xFBC0 | Rn
// hw1 = RdLo_RdHi_10_N_M_Rm   (N=high Rn, M=high Rm)

/// SMLALBB/BT/TB/TT RdLo, RdHi, Rn, Rm — signed 64-bit halfword MAC.
pub fn enc_t32_smlalxy(
    rdlo: u16,
    rdhi: u16,
    rn: u16,
    rm: u16,
    n_high: bool,
    m_high: bool,
) -> (u16, u16) {
    let hw0 = 0xFBC0 | (rn & 0xF);
    let op2 = 0b1000 | ((n_high as u16) << 1) | (m_high as u16);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Long dual halfword multiply-add/subtract (SMLALD/SMLALDX, SMLSLD/SMLSLDX)
// Decoder: thumb32_long_multiply, op1=100 (add) / op1=101 (sub), op2=1100/1101
// hw0 = 0xFBC0 (add) / 0xFBD0 (sub)  | Rn
// hw1 = RdLo_RdHi_110_X_Rm

/// SMLALD / SMLALDX RdLo, RdHi, Rn, Rm — signed 64-bit dual halfword MAC.
pub fn enc_t32_smlald(rdlo: u16, rdhi: u16, rn: u16, rm: u16, cross: bool) -> (u16, u16) {
    let hw0 = 0xFBC0 | (rn & 0xF);
    let op2 = 0b1100 | (cross as u16);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SMLSLD / SMLSLDX RdLo, RdHi, Rn, Rm — signed 64-bit dual halfword multiply-subtract.
pub fn enc_t32_smlsld(rdlo: u16, rdhi: u16, rn: u16, rm: u16, cross: bool) -> (u16, u16) {
    let hw0 = 0xFBD0 | (rn & 0xF);
    let op2 = 0b1100 | (cross as u16);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (op2 << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Unsigned 64-bit multiply-accumulate-accumulate (UMAAL)
// Decoder: thumb32_long_multiply, op1=110, op2=0110
// hw0 = 0xFBE0 | Rn
// hw1 = RdLo_RdHi_0110_Rm

/// UMAAL RdLo, RdHi, Rn, Rm — unsigned 64-bit multiply plus two 32-bit accumulates.
pub fn enc_t32_umaal(rdlo: u16, rdhi: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFBE0 | (rn & 0xF);
    let hw1 = ((rdlo & 0xF) << 12) | ((rdhi & 0xF) << 8) | (0b0110 << 4) | (rm & 0xF);
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
    let s = (uoff >> 20) & 1;
    let j2 = (uoff >> 19) & 1;
    let j1 = (uoff >> 18) & 1;
    let imm6 = (uoff >> 12) & 0x3F;
    let imm11 = (uoff >> 1) & 0x7FF;

    let hw0 = 0xF000u16 | ((s as u16) << 10) | ((cond & 0xF) << 6) | (imm6 as u16);

    let hw1 = 0x8000u16 | ((j1 as u16) << 13) | ((j2 as u16) << 11) | (imm11 as u16);

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
    let uoff = offset as u32;
    let s = (uoff >> 24) & 1;
    let i1 = (uoff >> 23) & 1;
    let i2 = (uoff >> 22) & 1;
    let imm10 = (uoff >> 12) & 0x3FF;
    let imm11 = (uoff >> 1) & 0x7FF;

    // Reverse the XOR trick to get J1, J2
    let j1 = (i1 ^ s) ^ 1;
    let j2 = (i2 ^ s) ^ 1;

    let hw0 = 0xF000u16 | ((s as u16) << 10) | (imm10 as u16);

    let hw1 = 0x9000u16 | ((j1 as u16) << 13) | ((j2 as u16) << 11) | (imm11 as u16);

    (hw0, hw1)
}

/// BL — 25-bit signed offset, same J1/J2 XOR trick as B.W T4.
///
/// hw0: 11110_S_imm10[9:0]
/// hw1: 11_J1_1_J2_imm11[10:0]
pub fn enc_t32_bl(offset: i32) -> (u16, u16) {
    let uoff = offset as u32;
    let s = (uoff >> 24) & 1;
    let i1 = (uoff >> 23) & 1;
    let i2 = (uoff >> 22) & 1;
    let imm10 = (uoff >> 12) & 0x3FF;
    let imm11 = (uoff >> 1) & 0x7FF;

    let j1 = (i1 ^ s) ^ 1;
    let j2 = (i2 ^ s) ^ 1;

    let hw0 = 0xF000u16 | ((s as u16) << 10) | (imm10 as u16);

    let hw1 = 0xD000u16 | ((j1 as u16) << 13) | ((j2 as u16) << 11) | (imm11 as u16);

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
    let hw0 = 0xFA00 | (rn & 0xF); // stype=00, S=0
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// LSR.W Rd, Rn, Rm
pub fn enc_t32_lsr_w_reg(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA20 | (rn & 0xF); // stype=01, S=0
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// ASR.W Rd, Rn, Rm
pub fn enc_t32_asr_w_reg(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA40 | (rn & 0xF); // stype=10, S=0
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | (rm & 0xF);
    (hw0, hw1)
}

/// ROR.W Rd, Rn, Rm
pub fn enc_t32_ror_w_reg(rd: u16, rn: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA60 | (rn & 0xF); // stype=11, S=0
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
    let hw0 = 0xFA4F; // ext=100, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// UXTB.W Rd, Rm, {ROR #rot}
pub fn enc_t32_uxtb_w(rd: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA5F; // ext=101, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SXTH.W Rd, Rm, {ROR #rot}
pub fn enc_t32_sxth_w(rd: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA0F; // ext=000, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// UXTH.W Rd, Rm, {ROR #rot}
pub fn enc_t32_uxth_w(rd: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA1F; // ext=001, Rn=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

// -- Extend-and-add instructions -----------------------------------------------
// Same encoding as plain extends but with Rn != 15 (Rn supplies the addend).
//   hw0 = 1111_1010_0_ext[2:0]_Rn
//   hw1 = 1111_Rd_1_0_rot[1:0]_Rm
//
// ext: 000=SXTAH, 001=UXTAH, 100=SXTAB, 101=UXTAB

/// SXTAB Rd, Rn, Rm, {ROR #rot}  (sign-extend byte, add to Rn)
pub fn enc_t32_sxtab(rd: u16, rn: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA40 | (rn & 0xF); // ext=100, Rn!=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// UXTAB Rd, Rn, Rm, {ROR #rot}  (zero-extend byte, add to Rn)
pub fn enc_t32_uxtab(rd: u16, rn: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA50 | (rn & 0xF); // ext=101, Rn!=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// SXTAH Rd, Rn, Rm, {ROR #rot}  (sign-extend halfword, add to Rn)
pub fn enc_t32_sxtah(rd: u16, rn: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA00 | (rn & 0xF); // ext=000, Rn!=15
    let hw1 = 0xF080 | ((rd & 0xF) << 8) | ((rot / 8) << 4) | (rm & 0xF);
    (hw0, hw1)
}

/// UXTAH Rd, Rn, Rm, {ROR #rot}  (zero-extend halfword, add to Rn)
pub fn enc_t32_uxtah(rd: u16, rn: u16, rm: u16, rot: u16) -> (u16, u16) {
    let hw0 = 0xFA10 | (rn & 0xF); // ext=001, Rn!=15
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
/// `operation` = par_op1/hw0[6:4] (3 bits), `modifier` = par_op2/hw1[6:4] (3 bits).
pub fn enc_t32_parallel(operation: u16, modifier: u16, rn: u16, rd: u16, rm: u16) -> (u16, u16) {
    let hw0 = 0xFA80 | ((operation & 0x7) << 4) | (rn & 0xF);
    let hw1 = 0xF000 | ((rd & 0xF) << 8) | ((modifier & 0x7) << 4) | (rm & 0xF);
    (hw0, hw1)
}

// ============================================================================
// Test generators — Priority 1
// ============================================================================

use crate::{
    MASK_ALL_FLAGS, MASK_ALL_FLAGS_GE, MASK_NO_FLAGS, MASK_Q_ONLY, TestCase, mem_check_u16,
    mem_check_u32, mem_pre_u16, mem_pre_u32,
};

// ---------------------------------------------------------------------------
// Generator 1: Data processing — modified immediate  (~80 tests)
// ---------------------------------------------------------------------------

/// Test data processing with modified (ThumbExpandImm) immediates.
///
/// Covers all five imm12 sub-modes, every ALU op with S=1, flag-only
/// variants (Rd=15), no-source variants (Rn=15), and carry-out from
/// rotation.
pub fn gen_t32_dp_mod_imm() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32; // T bit
    let tb_c = tb | (1 << 29); // T bit + C flag set

    // -- Helper: make a TestCase for S=1 (flag-updating) ALU op --
    let mk = |name: &str,
              op: u16,
              rn: u16,
              rd: u16,
              imm12: u16,
              regs: Vec<(u8, u32)>,
              xpsr: u32|
     -> TestCase {
        let (hw0, hw1) = enc_t32_dp_mod_imm(op, true, rn, rd, imm12);
        TestCase {
            name: name.into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: xpsr,
            xpsr_mask: MASK_ALL_FLAGS,
            ..TestCase::default()
        }
    };

    // ----------------------------------------------------------------
    // ThumbExpandImm sub-modes (verify immediate decoding)
    // ----------------------------------------------------------------

    // Mode 0: imm12[11:10]=00, [9:8]=00 → plain byte 0x000000ii
    t.push(mk(
        "ADDS.W R3,R5,#0x1F (plain byte)",
        DP_ADD,
        5,
        3,
        0x01F,
        vec![(5, 0)],
        tb,
    ));
    t.push(mk(
        "ADDS.W R0,R1,#0x00 (plain byte zero)",
        DP_ADD,
        1,
        0,
        0x000,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "ADDS.W R0,R1,#0xFF (plain byte max)",
        DP_ADD,
        1,
        0,
        0x0FF,
        vec![(1, 0)],
        tb,
    ));

    // Mode 1: imm12[11:10]=00, [9:8]=01 → 0x00ii00ii
    // imm12 = 0b00_01_iiiiiiii = 0x100 | imm8
    t.push(mk(
        "ADDS.W R0,R1,#0x00420042 (rep x2)",
        DP_ADD,
        1,
        0,
        0x142,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "ORRS.W R2,R3,#0x00FF00FF (rep x2 max)",
        DP_ORR,
        3,
        2,
        0x1FF,
        vec![(3, 0)],
        tb,
    ));

    // Mode 2: imm12[11:10]=00, [9:8]=10 → 0xii00ii00
    // imm12 = 0b00_10_iiiiiiii = 0x200 | imm8
    t.push(mk(
        "ADDS.W R0,R1,#0x42004200 (rep x2 shifted)",
        DP_ADD,
        1,
        0,
        0x242,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "EORS.W R4,R5,#0xAB00AB00 (rep x2 shifted)",
        DP_EOR,
        5,
        4,
        0x2AB,
        vec![(5, 0xAB00AB00)],
        tb,
    ));

    // Mode 3: imm12[11:10]=00, [9:8]=11 → 0xiiiiiiii
    // imm12 = 0b00_11_iiiiiiii = 0x300 | imm8
    t.push(mk(
        "ADDS.W R0,R1,#0xABABABAB (rep x4)",
        DP_ADD,
        1,
        0,
        0x3AB,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "ANDS.W R2,R3,#0xFFFFFFFF (rep x4 max)",
        DP_AND,
        3,
        2,
        0x3FF,
        vec![(3, 0x12345678)],
        tb,
    ));

    // Mode 4: imm12[11:10]!=00 → rotated (0x80|imm7) ROR amount
    // imm12 = 0b01_00000_ii = 0x400 | imm7:  (0x80|0) ROR 8  = 0x80000000
    t.push(mk(
        "ADDS.W R0,R1,#0x80000000 (rotated)",
        DP_ADD,
        1,
        0,
        0x400,
        vec![(1, 0)],
        tb,
    ));
    // imm12 = 0b10_00000_00 = 0x800: (0x80|0) ROR 16 = 0x00008000
    t.push(mk(
        "ADDS.W R0,R1,#0x00008000 (rotated)",
        DP_ADD,
        1,
        0,
        0x800,
        vec![(1, 0)],
        tb,
    ));
    // imm12 = 0b01_00001_01 = 0x405: (0x80|5) ROR 8 = 0x85000000
    t.push(mk(
        "ADDS.W R0,R1,#rotated(0x85 ROR 8)",
        DP_ADD,
        1,
        0,
        0x405,
        vec![(1, 0)],
        tb,
    ));

    // ----------------------------------------------------------------
    // ALU operations with S=1 — verify flag behavior
    // ----------------------------------------------------------------

    // ADDS.W: zero result (Z)
    t.push(mk(
        "ADDS.W R0,R1,#0 (Z flag, zero+zero)",
        DP_ADD,
        1,
        0,
        0x000,
        vec![(1, 0)],
        tb,
    ));
    // ADDS.W: negative result (N)
    t.push(mk(
        "ADDS.W R0,R1,#1 (N flag)",
        DP_ADD,
        1,
        0,
        0x001,
        vec![(1, 0xFFFF_FFFE)],
        tb,
    ));
    // ADDS.W: carry (C)
    t.push(mk(
        "ADDS.W R0,R1,#1 (C flag, wrap)",
        DP_ADD,
        1,
        0,
        0x001,
        vec![(1, 0xFFFF_FFFF)],
        tb,
    ));
    // ADDS.W: overflow (V)
    t.push(mk(
        "ADDS.W R0,R1,#1 (V flag, pos overflow)",
        DP_ADD,
        1,
        0,
        0x001,
        vec![(1, 0x7FFF_FFFF)],
        tb,
    ));

    // SUBS.W
    t.push(mk(
        "SUBS.W R0,R1,#1 (borrow)",
        DP_SUB,
        1,
        0,
        0x001,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "SUBS.W R0,R1,#0 (no borrow)",
        DP_SUB,
        1,
        0,
        0x000,
        vec![(1, 5)],
        tb,
    ));
    t.push(mk(
        "SUBS.W R0,R1,#5 (equal)",
        DP_SUB,
        1,
        0,
        0x005,
        vec![(1, 5)],
        tb,
    ));

    // CMP.W (SUB with Rd=15, flag-only)
    t.push(mk(
        "CMP.W R1,#0 (Rd=15, equal)",
        DP_SUB,
        1,
        15,
        0x000,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "CMP.W R2,#1 (Rd=15, less)",
        DP_SUB,
        2,
        15,
        0x001,
        vec![(2, 0)],
        tb,
    ));
    t.push(mk(
        "CMP.W R3,#1 (Rd=15, greater)",
        DP_SUB,
        3,
        15,
        0x001,
        vec![(3, 5)],
        tb,
    ));

    // ANDS.W
    t.push(mk(
        "ANDS.W R0,R1,#0xFF (non-zero)",
        DP_AND,
        1,
        0,
        0x0FF,
        vec![(1, 0x1234_5678)],
        tb,
    ));
    t.push(mk(
        "ANDS.W R0,R1,#0xFF (zero result)",
        DP_AND,
        1,
        0,
        0x0FF,
        vec![(1, 0x1234_5600)],
        tb,
    ));

    // TST.W (AND with Rd=15, flag-only)
    t.push(mk(
        "TST.W R1,#0xFF (Rd=15, non-zero)",
        DP_AND,
        1,
        15,
        0x0FF,
        vec![(1, 0x42)],
        tb,
    ));
    t.push(mk(
        "TST.W R2,#0xFF (Rd=15, zero)",
        DP_AND,
        2,
        15,
        0x0FF,
        vec![(2, 0x100)],
        tb,
    ));

    // ORRS.W
    t.push(mk(
        "ORRS.W R0,R1,#0xFF",
        DP_ORR,
        1,
        0,
        0x0FF,
        vec![(1, 0x1234_5600)],
        tb,
    ));
    t.push(mk(
        "ORRS.W R0,R1,#0 (N flag)",
        DP_ORR,
        1,
        0,
        0x000,
        vec![(1, 0x8000_0000)],
        tb,
    ));

    // EORS.W
    t.push(mk(
        "EORS.W R0,R1,#0xFF (non-zero)",
        DP_EOR,
        1,
        0,
        0x0FF,
        vec![(1, 0xFF)],
        tb,
    ));
    t.push(mk(
        "EORS.W R0,R1,#0xFF (zero result)",
        DP_EOR,
        1,
        0,
        0x0FF,
        vec![(1, 0xFF)],
        tb,
    ));

    // TEQ.W (EOR with Rd=15, flag-only)
    t.push(mk(
        "TEQ.W R1,#0xFF (Rd=15, zero)",
        DP_EOR,
        1,
        15,
        0x0FF,
        vec![(1, 0xFF)],
        tb,
    ));
    t.push(mk(
        "TEQ.W R2,#0xFF (Rd=15, non-zero)",
        DP_EOR,
        2,
        15,
        0x0FF,
        vec![(2, 0x100)],
        tb,
    ));

    // BICS.W
    t.push(mk(
        "BICS.W R0,R1,#0xFF (clear low byte)",
        DP_BIC,
        1,
        0,
        0x0FF,
        vec![(1, 0x1234_56FF)],
        tb,
    ));
    t.push(mk(
        "BICS.W R0,R1,#0xFF (zero result)",
        DP_BIC,
        1,
        0,
        0x0FF,
        vec![(1, 0xFF)],
        tb,
    ));

    // ORNS.W
    t.push(mk(
        "ORNS.W R0,R1,#0xFF (ORN = Rn | ~imm)",
        DP_ORN,
        1,
        0,
        0x0FF,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "ORNS.W R0,R1,#0 (ORN with imm=0)",
        DP_ORN,
        1,
        0,
        0x000,
        vec![(1, 0)],
        tb,
    ));

    // CMN.W (ADD with Rd=15, flag-only)
    t.push(mk(
        "CMN.W R1,#0 (Rd=15, zero)",
        DP_ADD,
        1,
        15,
        0x000,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "CMN.W R2,#1 (Rd=15, wrap)",
        DP_ADD,
        2,
        15,
        0x001,
        vec![(2, 0xFFFF_FFFF)],
        tb,
    ));

    // ----------------------------------------------------------------
    // ADCS.W / SBCS.W / RSBS.W — carry-in sensitive
    // ----------------------------------------------------------------

    // ADCS.W with C=0
    t.push(mk(
        "ADCS.W R0,R1,#1 (C_in=0)",
        DP_ADC,
        1,
        0,
        0x001,
        vec![(1, 10)],
        tb,
    ));
    // ADCS.W with C=1
    t.push(mk(
        "ADCS.W R0,R1,#1 (C_in=1)",
        DP_ADC,
        1,
        0,
        0x001,
        vec![(1, 10)],
        tb_c,
    ));
    // ADCS.W producing carry
    t.push(mk(
        "ADCS.W R0,R1,#1 (C_in=1, wrap)",
        DP_ADC,
        1,
        0,
        0x001,
        vec![(1, 0xFFFF_FFFE)],
        tb_c,
    ));

    // SBCS.W with C=1 (no borrow) — SBC subtracts (imm + !C), C=1 means no extra borrow
    t.push(mk(
        "SBCS.W R0,R1,#1 (C_in=1, no borrow)",
        DP_SBC,
        1,
        0,
        0x001,
        vec![(1, 10)],
        tb_c,
    ));
    // SBCS.W with C=0 (borrow)
    t.push(mk(
        "SBCS.W R0,R1,#1 (C_in=0, borrow)",
        DP_SBC,
        1,
        0,
        0x001,
        vec![(1, 10)],
        tb,
    ));
    // SBCS.W zero result
    t.push(mk(
        "SBCS.W R0,R1,#5 (C_in=1, zero result)",
        DP_SBC,
        1,
        0,
        0x005,
        vec![(1, 5)],
        tb_c,
    ));

    // RSBS.W (reverse subtract: imm - Rn)
    t.push(mk(
        "RSBS.W R0,R1,#0 (negate)",
        DP_RSB,
        1,
        0,
        0x000,
        vec![(1, 1)],
        tb,
    ));
    t.push(mk(
        "RSBS.W R0,R1,#0xFF (positive result)",
        DP_RSB,
        1,
        0,
        0x0FF,
        vec![(1, 0)],
        tb,
    ));
    t.push(mk(
        "RSBS.W R0,R1,#0 (negate zero)",
        DP_RSB,
        1,
        0,
        0x000,
        vec![(1, 0)],
        tb,
    ));

    // ----------------------------------------------------------------
    // No-source variants (Rn=15)
    // ----------------------------------------------------------------

    // MOV.W (ORR with Rn=15 → result = 0 | imm = imm)
    t.push(mk(
        "MOV.W R0,#42 (Rn=15, ORR)",
        DP_ORR,
        15,
        0,
        0x02A,
        vec![],
        tb,
    ));
    t.push(mk(
        "MOV.W R5,#0 (Rn=15, zero)",
        DP_ORR,
        15,
        5,
        0x000,
        vec![],
        tb,
    ));
    t.push(mk(
        "MOV.W R3,#0xFF (Rn=15, 0xFF)",
        DP_ORR,
        15,
        3,
        0x0FF,
        vec![],
        tb,
    ));

    // MVN.W (ORN with Rn=15 → result = 0xFFFFFFFF | ~imm... wait: ORN = Rn | ~imm,
    // but Rn=15 → ORN reads as 0, so result = ~imm)
    t.push(mk(
        "MVN.W R0,#0 (Rn=15, ~0 = 0xFFFFFFFF)",
        DP_ORN,
        15,
        0,
        0x000,
        vec![],
        tb,
    ));
    t.push(mk(
        "MVN.W R0,#0xFF (Rn=15, ~0xFF)",
        DP_ORN,
        15,
        0,
        0x0FF,
        vec![],
        tb,
    ));

    // ----------------------------------------------------------------
    // Carry-out from ThumbExpandImm rotation (S=1)
    // ----------------------------------------------------------------

    // Rotated mode with S=1: the MSB of the rotated constant is the carry-out.
    // imm12 = 0b01_00000_00 = 0x400 → 0x80 ROR 8 = 0x80000000 → C=1 (bit 31)
    t.push(mk(
        "ANDS.W R0,R1,#0x80000000 (rotation C=1)",
        DP_AND,
        1,
        0,
        0x400,
        vec![(1, 0xFFFF_FFFF)],
        tb,
    ));
    // imm12 = 0b10_00000_00 = 0x800 → 0x80 ROR 16 = 0x00008000 → C=0 (bit 31=0)
    t.push(mk(
        "ANDS.W R0,R1,#0x00008000 (rotation C=0)",
        DP_AND,
        1,
        0,
        0x800,
        vec![(1, 0xFFFF_FFFF)],
        tb,
    ));
    // imm12 = 0b01_11111_11 = 0x7FF → (0x80|0x7F=0xFF) ROR 8 = 0xFF000000 → C=1
    t.push(mk(
        "ORRS.W R0,R1,#0xFF000000 (rotation C=1)",
        DP_ORR,
        1,
        0,
        0x47F,
        vec![(1, 0)],
        tb,
    ));

    // Verify rotation carry-out OVERWRITES incoming C flag (not preserves it).
    // Rotated mode: imm12=0x201 → [11:7]=0b00100=4, [6:0]=1
    // Constant = (0x80|0x01) ROR 4 = 0x81 ROR 4 = 0x1000_0008, bit 31=0 → C_out=0
    // Incoming C=1 should be cleared by rotation carry_out=0.
    {
        let (hw0, hw1) = enc_t32_dp_mod_imm(DP_AND, true, 0, 0, 0x201);
        t.push(TestCase {
            name: "ANDS.W R0,R0,#(rot) C_out=0 clears incoming C=1".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xFFFF_FFFF)],
            xpsr_pre: tb | (1 << 29), // C=1 incoming, should be cleared by rotation C_out=0
            xpsr_mask: MASK_ALL_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // Register field extraction (different rd/rn combos)
    // ----------------------------------------------------------------
    t.push(mk(
        "ADDS.W R7,R8,#1 (high Rn)",
        DP_ADD,
        8,
        7,
        0x001,
        vec![(8, 100)],
        tb,
    ));
    t.push(mk(
        "ADDS.W R10,R2,#1 (high Rd)",
        DP_ADD,
        2,
        10,
        0x001,
        vec![(2, 50)],
        tb,
    ));
    t.push(mk(
        "ADDS.W R12,R11,#0 (both high)",
        DP_ADD,
        11,
        12,
        0x000,
        vec![(11, 0xDEAD_BEEF)],
        tb,
    ));

    // -- Ensure we didn't have the EOR zero-result test wrong; fix it:
    // EORS 0xFF ^ 0x00 = 0xFF (not zero), move the actual zero-result test
    t.push(mk(
        "EORS.W R0,R1,#0x42 (zero: 0x42^0x42=0)",
        DP_EOR,
        1,
        0,
        0x042,
        vec![(1, 0x42)],
        tb,
    ));

    t
}

// ---------------------------------------------------------------------------
// Generator 2: Load/store single (~60 tests)
// ---------------------------------------------------------------------------

/// Test Thumb-32 load/store single with all three addressing modes:
/// imm12 positive offset, imm8 with P/U/W, and register offset with shift.
pub fn gen_t32_load_store_single() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // ----------------------------------------------------------------
    // imm12 positive offset mode
    // ----------------------------------------------------------------

    // LDR.W — word loads
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, true, false, 1, 0, 0);
        t.push(TestCase {
            name: "LDR.W R0,[R1,#0] (word, zero offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(0, 0xDEAD_BEEF),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, true, false, 1, 0, 4);
        t.push(TestCase {
            name: "LDR.W R0,[R1,#4]".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(4, 0xCAFE_BABE),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, true, false, 2, 3, 100);
        t.push(TestCase {
            name: "LDR.W R3,[R2,#100] (field extract)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 0x00)],
            addr_regs: vec![2],
            needs_bus: true,
            mem_pre: mem_pre_u32(100, 0x1234_5678),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRB.W — byte load, zero extension
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b00, true, false, 1, 0, 8);
        t.push(TestCase {
            name: "LDRB.W R0,[R1,#8] (zero-extend)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(8, 0xAB)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b00, true, false, 1, 0, 0);
        t.push(TestCase {
            name: "LDRB.W R0,[R1,#0] (0xFF, stays 0xFF)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(0, 0xFF)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRH.W — halfword load, zero extension
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b01, true, false, 1, 0, 4);
        t.push(TestCase {
            name: "LDRH.W R0,[R1,#4] (zero-extend)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(4, 0xBEEF),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSB.W — signed byte, sign extension
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b00, true, true, 1, 0, 16);
        t.push(TestCase {
            name: "LDRSB.W R0,[R1,#16] (0x80 -> 0xFFFFFF80)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(16, 0x80)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b00, true, true, 1, 0, 0);
        t.push(TestCase {
            name: "LDRSB.W R0,[R1,#0] (0x7F positive)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(0, 0x7F)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSH.W — signed halfword, sign extension
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b01, true, true, 1, 0, 4);
        t.push(TestCase {
            name: "LDRSH.W R0,[R1,#4] (0x8000 -> 0xFFFF8000)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(4, 0x8000),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b01, true, true, 1, 0, 0);
        t.push(TestCase {
            name: "LDRSH.W R0,[R1,#0] (0x7FFF positive)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(0, 0x7FFF),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STR.W — word stores
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, false, false, 1, 0, 0);
        t.push(TestCase {
            name: "STR.W R0,[R1,#0]".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xDEAD_BEEF), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u32(0),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, false, false, 2, 3, 8);
        t.push(TestCase {
            name: "STR.W R3,[R2,#8] (field extract)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(3, 0x1234_5678), (2, 0x00)],
            addr_regs: vec![2],
            needs_bus: true,
            mem_check: mem_check_u32(8),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STRB.W
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b00, false, false, 1, 0, 4);
        t.push(TestCase {
            name: "STRB.W R0,[R1,#4]".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xAB), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: vec![4],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STRH.W
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b01, false, false, 1, 0, 4);
        t.push(TestCase {
            name: "STRH.W R0,[R1,#4]".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xBEEF), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u16(4),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // imm8 with P/U/W (pre-index, post-index, negative offset)
    // ----------------------------------------------------------------

    // Positive offset, no writeback: P=1, U=1, W=0
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, true, false, 1, 0, true, true, false, 8);
        t.push(TestCase {
            name: "LDR.W R0,[R1,#8] (imm8 P=1,U=1,W=0)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(8, 0xAAAA_BBBB),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Negative offset: P=1, U=0, W=0 — load from base-offset
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, true, false, 1, 0, true, false, false, 8);
        t.push(TestCase {
            name: "LDR.W R0,[R1,#-8] (negative offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 16)], // base=16, effective=16-8=8
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(8, 0x1111_2222),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Pre-index with writeback: P=1, U=1, W=1 — base updated to base+offset
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, true, false, 1, 0, true, true, true, 4);
        t.push(TestCase {
            name: "LDR.W R0,[R1,#4]! (pre-index writeback)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(4, 0xFACE_CAFE),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Post-index: P=0, U=1, W=1 — access at base, then base updated to base+offset
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, true, false, 1, 0, false, true, true, 4);
        t.push(TestCase {
            name: "LDR.W R0,[R1],#4 (post-index)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(0, 0xBEEF_DEAD),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Pre-index negative: P=1, U=0, W=1
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, true, false, 1, 0, true, false, true, 8);
        t.push(TestCase {
            name: "LDR.W R0,[R1,#-8]! (pre-index neg writeback)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 16)], // effective addr = 16-8 = 8
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(8, 0x3333_4444),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Post-index negative: P=0, U=0, W=1
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, true, false, 1, 0, false, false, true, 4);
        t.push(TestCase {
            name: "LDR.W R0,[R1],#-4 (post-index neg)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 8)], // load from addr 8, then base = 8-4 = 4
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(8, 0x5555_6666),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Store with pre-index writeback
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, false, false, 1, 0, true, true, true, 4);
        t.push(TestCase {
            name: "STR.W R0,[R1,#4]! (pre-index writeback store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0x7777_8888), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u32(4),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Store with post-index
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b10, false, false, 1, 0, false, true, true, 4);
        t.push(TestCase {
            name: "STR.W R0,[R1],#4 (post-index store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0x9999_AAAA), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u32(0),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRB.W with negative offset
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b00, true, false, 1, 0, true, false, false, 4);
        t.push(TestCase {
            name: "LDRB.W R0,[R1,#-4] (neg offset byte)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 8)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(4, 0xCD)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRH.W with pre-index writeback
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b01, true, false, 1, 0, true, true, true, 4);
        t.push(TestCase {
            name: "LDRH.W R0,[R1,#4]! (pre-index half)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(4, 0xABCD),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STRB.W with post-index
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b00, false, false, 1, 0, false, true, true, 4);
        t.push(TestCase {
            name: "STRB.W R0,[R1],#4 (post-index byte store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0x42), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: vec![0],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STRH.W with negative offset
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b01, false, false, 1, 0, true, false, false, 4);
        t.push(TestCase {
            name: "STRH.W R0,[R1,#-4] (neg offset half store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xFACE), (1, 8)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u16(4),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSB.W with pre-index
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b00, true, true, 1, 0, true, true, true, 4);
        t.push(TestCase {
            name: "LDRSB.W R0,[R1,#4]! (pre-index signed byte)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(4, 0x80)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSH.W with post-index
    {
        let (hw0, hw1) = enc_t32_ls_imm8(0b01, true, true, 1, 0, false, true, true, 4);
        t.push(TestCase {
            name: "LDRSH.W R0,[R1],#4 (post-index signed half)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(0, 0x8000),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // Register offset mode
    // ----------------------------------------------------------------

    // LDR.W Rt,[Rn,Rm,LSL #0]
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b10, true, false, 1, 0, 2, 0);
        t.push(TestCase {
            name: "LDR.W R0,[R1,R2] (reg offset, shift=0)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00), (2, 8)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(8, 0xAAAA_BBBB),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDR.W Rt,[Rn,Rm,LSL #2]
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b10, true, false, 1, 0, 2, 2);
        t.push(TestCase {
            name: "LDR.W R0,[R1,R2,LSL #2] (shift=2)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00), (2, 4)], // effective = 0 + 4<<2 = 16
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(16, 0x1111_2222),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STR.W with register offset
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b10, false, false, 1, 0, 2, 0);
        t.push(TestCase {
            name: "STR.W R0,[R1,R2] (reg offset store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xCAFE_BABE), (1, 0x00), (2, 4)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u32(4),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRB.W with register offset
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b00, true, false, 1, 0, 2, 0);
        t.push(TestCase {
            name: "LDRB.W R0,[R1,R2] (reg offset byte)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00), (2, 3)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(3, 0xEF)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRH.W with register offset, shift=1
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b01, true, false, 1, 0, 2, 1);
        t.push(TestCase {
            name: "LDRH.W R0,[R1,R2,LSL #1] (shift=1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00), (2, 4)], // effective = 0 + 4<<1 = 8
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(8, 0xDEAD),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STRB.W with register offset
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b00, false, false, 1, 0, 2, 0);
        t.push(TestCase {
            name: "STRB.W R0,[R1,R2] (reg offset byte store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xAB), (1, 0x00), (2, 5)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: vec![5],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STRH.W with register offset, shift=1
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b01, false, false, 1, 0, 2, 1);
        t.push(TestCase {
            name: "STRH.W R0,[R1,R2,LSL #1] (reg offset half store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xBEEF), (1, 0x00), (2, 2)], // effective = 0 + 2<<1 = 4
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u16(4),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSB.W with register offset
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b00, true, true, 1, 0, 2, 0);
        t.push(TestCase {
            name: "LDRSB.W R0,[R1,R2] (reg, sign extend 0x80)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00), (2, 0)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(0, 0x80)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSH.W with register offset
    {
        let (hw0, hw1) = enc_t32_ls_reg(0b01, true, true, 1, 0, 2, 0);
        t.push(TestCase {
            name: "LDRSH.W R0,[R1,R2] (reg, sign extend 0xFFFF)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00), (2, 4)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(4, 0xFFFF),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // Additional edge cases: max-ish imm12 within scratch, zero data
    // ----------------------------------------------------------------

    // LDR.W with larger offset (within SCRATCH_SIZE)
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, true, false, 1, 0, 252);
        t.push(TestCase {
            name: "LDR.W R0,[R1,#252] (near scratch limit)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u32(252, 0x9876_5432),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STR.W then verify with zero value
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, false, false, 1, 0, 0);
        t.push(TestCase {
            name: "STR.W R0,[R1,#0] (zero value)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u32(0),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STR.W with max value
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, false, false, 1, 0, 0);
        t.push(TestCase {
            name: "STR.W R0,[R1,#0] (0xFFFFFFFF)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xFFFF_FFFF), (1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_check: mem_check_u32(0),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Field extraction: different Rn/Rt combos
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, true, false, 8, 9, 0);
        t.push(TestCase {
            name: "LDR.W R9,[R8,#0] (high regs)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(8, 0x00)],
            addr_regs: vec![8],
            needs_bus: true,
            mem_pre: mem_pre_u32(0, 0x1234_ABCD),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b10, false, false, 10, 11, 4);
        t.push(TestCase {
            name: "STR.W R11,[R10,#4] (high regs store)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(11, 0xFEDC_BA98), (10, 0x00)],
            addr_regs: vec![10],
            needs_bus: true,
            mem_check: mem_check_u32(4),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSB zero value → no sign extension
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b00, true, true, 1, 0, 0);
        t.push(TestCase {
            name: "LDRSB.W R0,[R1,#0] (0x00 zero)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(0, 0x00)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSH zero value
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b01, true, true, 1, 0, 0);
        t.push(TestCase {
            name: "LDRSH.W R0,[R1,#0] (0x0000 zero)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(0, 0x0000),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSB 0xFF → -1
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b00, true, true, 1, 0, 0);
        t.push(TestCase {
            name: "LDRSB.W R0,[R1,#0] (0xFF -> -1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: vec![(0, 0xFF)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDRSH 0xFFFF → -1
    {
        let (hw0, hw1) = enc_t32_ls_imm12(0b01, true, true, 1, 0, 0);
        t.push(TestCase {
            name: "LDRSH.W R0,[R1,#0] (0xFFFF -> -1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x00)],
            addr_regs: vec![1],
            needs_bus: true,
            mem_pre: mem_pre_u16(0, 0xFFFF),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    t
}

// ---------------------------------------------------------------------------
// Generator 3: Multiply / divide (~40 tests)
// ---------------------------------------------------------------------------

/// Test 32-bit multiply, 64-bit multiply, and integer division instructions.
pub fn gen_t32_multiply_divide() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // ----------------------------------------------------------------
    // MUL — 32-bit multiply (lower 32 bits of product)
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_mul(0, 1, 2);
        t.push(TestCase {
            name: "MUL R0, R1, R2 (3*7=21)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 3), (2, 7)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_mul(3, 4, 5);
        t.push(TestCase {
            name: "MUL R3, R4, R5 (field extract)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 100), (5, 200)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_mul(0, 1, 2);
        t.push(TestCase {
            name: "MUL R0, R1, R2 (by zero)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x1234_5678), (2, 0)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_mul(0, 1, 2);
        t.push(TestCase {
            name: "MUL R0, R1, R2 (by one)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0xDEAD_BEEF), (2, 1)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_mul(0, 1, 2);
        t.push(TestCase {
            name: "MUL R0, R1, R2 (large, truncated)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0001_0000), (2, 0x0001_0000)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // MLA — multiply-accumulate: Rd = Rn*Rm + Ra
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_mla(0, 1, 2, 3);
        t.push(TestCase {
            name: "MLA R0, R1, R2, R3 (3*7+10=31)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 3), (2, 7), (3, 10)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_mla(4, 5, 6, 7);
        t.push(TestCase {
            name: "MLA R4, R5, R6, R7 (field extract)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(5, 2), (6, 3), (7, 100)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_mla(0, 1, 2, 3);
        t.push(TestCase {
            name: "MLA R0, R1, R2, R3 (accum only, 0*x+Ra)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0), (2, 42), (3, 99)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // MLS — multiply-subtract: Rd = Ra - Rn*Rm
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_mls(0, 1, 2, 3);
        t.push(TestCase {
            name: "MLS R0, R1, R2, R3 (100-3*7=79)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 3), (2, 7), (3, 100)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_mls(0, 1, 2, 3);
        t.push(TestCase {
            name: "MLS R0, R1, R2, R3 (0-3*7, wraps)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 3), (2, 7), (3, 0)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // SMULL — signed 64-bit multiply
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_smull(0, 1, 2, 3);
        t.push(TestCase {
            name: "SMULL R0,R1, R2,R3 (0x10000*0x10000)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 0x0001_0000), (3, 0x0001_0000)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // RdLo=0, RdHi=1
    }
    {
        let (hw0, hw1) = enc_t32_smull(0, 1, 2, 3);
        t.push(TestCase {
            name: "SMULL R0,R1, R2,R3 (small: 3*7)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 3), (3, 7)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // RdLo=21, RdHi=0
    }
    {
        let (hw0, hw1) = enc_t32_smull(0, 1, 2, 3);
        t.push(TestCase {
            name: "SMULL R0,R1, R2,R3 (neg: -1 * 2)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 0xFFFF_FFFF), (3, 2)], // -1 * 2 = -2
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // RdLo=0xFFFFFFFE, RdHi=0xFFFFFFFF
    }
    {
        let (hw0, hw1) = enc_t32_smull(4, 5, 6, 7);
        t.push(TestCase {
            name: "SMULL R4,R5, R6,R7 (field extract, 0*x)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(6, 0), (7, 0x1234_5678)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // UMULL — unsigned 64-bit multiply
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_umull(0, 1, 2, 3);
        t.push(TestCase {
            name: "UMULL R0,R1, R2,R3 (0x10000*0x10000)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 0x0001_0000), (3, 0x0001_0000)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_umull(0, 1, 2, 3);
        t.push(TestCase {
            name: "UMULL R0,R1, R2,R3 (MAX*2)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 0xFFFF_FFFF), (3, 2)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // 0xFFFFFFFF*2 = 0x1_FFFFFFFE
    }

    // ----------------------------------------------------------------
    // SMLAL — signed 64-bit multiply-accumulate
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_smlal(0, 1, 2, 3);
        t.push(TestCase {
            name: "SMLAL R0,R1, R2,R3 (accum 3*7+100)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 100), (1, 0), (2, 3), (3, 7)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // 64-bit accum: (0:100) + 21 = 121
    }
    {
        let (hw0, hw1) = enc_t32_smlal(0, 1, 2, 3);
        t.push(TestCase {
            name: "SMLAL R0,R1, R2,R3 (neg product)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 100), (1, 0), (2, 0xFFFF_FFFF), (3, 2)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // accum + (-1*2) = 100 + (-2) = 98
    }

    // ----------------------------------------------------------------
    // UMLAL — unsigned 64-bit multiply-accumulate
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_umlal(0, 1, 2, 3);
        t.push(TestCase {
            name: "UMLAL R0,R1, R2,R3 (accum 5*6+1000)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 1000), (1, 0), (2, 5), (3, 6)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // 1000 + 30 = 1030
    }
    {
        let (hw0, hw1) = enc_t32_umlal(0, 1, 2, 3);
        t.push(TestCase {
            name: "UMLAL R0,R1, R2,R3 (carry into hi)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xFFFF_FFFF), (1, 0), (2, 1), (3, 2)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }); // (0:0xFFFFFFFF) + 2 = (1:0x00000001)
    }

    // ----------------------------------------------------------------
    // SDIV — signed division
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_sdiv(0, 1, 2);
        t.push(TestCase {
            name: "SDIV R0, R1, R2 (21/7=3)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 21), (2, 7)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_sdiv(0, 1, 2);
        t.push(TestCase {
            name: "SDIV R0, R1, R2 (-21/7=-3)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, (-21i32) as u32), (2, 7)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_sdiv(0, 1, 2);
        t.push(TestCase {
            name: "SDIV R0, R1, R2 (-21/-7=3)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, (-21i32) as u32), (2, (-7i32) as u32)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_sdiv(0, 1, 2);
        t.push(TestCase {
            name: "SDIV R0, R1, R2 (div by zero = 0)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 42), (2, 0)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // INT32_MIN / -1 should return INT32_MIN (overflow wraps)
        let (hw0, hw1) = enc_t32_sdiv(0, 1, 2);
        t.push(TestCase {
            name: "SDIV R0, R1, R2 (INT32_MIN/-1 = INT32_MIN)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x8000_0000), (2, (-1i32) as u32)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // Round toward zero: 7/2 = 3 (not 4)
        let (hw0, hw1) = enc_t32_sdiv(0, 1, 2);
        t.push(TestCase {
            name: "SDIV R0, R1, R2 (7/2=3, round-toward-zero)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 7), (2, 2)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // Round toward zero negative: -7/2 = -3 (not -4)
        let (hw0, hw1) = enc_t32_sdiv(0, 1, 2);
        t.push(TestCase {
            name: "SDIV R0, R1, R2 (-7/2=-3, round-toward-zero)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, (-7i32) as u32), (2, 2)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_sdiv(3, 4, 5);
        t.push(TestCase {
            name: "SDIV R3, R4, R5 (field extract)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 100), (5, 10)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // UDIV — unsigned division
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_udiv(0, 1, 2);
        t.push(TestCase {
            name: "UDIV R0, R1, R2 (100/10=10)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 100), (2, 10)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_udiv(0, 1, 2);
        t.push(TestCase {
            name: "UDIV R0, R1, R2 (div by zero = 0)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0xFFFF_FFFF), (2, 0)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_udiv(0, 1, 2);
        t.push(TestCase {
            name: "UDIV R0, R1, R2 (7/2=3, truncated)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 7), (2, 2)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_udiv(0, 1, 2);
        t.push(TestCase {
            name: "UDIV R0, R1, R2 (MAX/1=MAX)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0xFFFF_FFFF), (2, 1)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_udiv(0, 1, 2);
        t.push(TestCase {
            name: "UDIV R0, R1, R2 (1/1=1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 1), (2, 1)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_t32_udiv(6, 7, 8);
        t.push(TestCase {
            name: "UDIV R6, R7, R8 (field extract)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(7, 255), (8, 5)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    t
}

// ============================================================================
// Test generators — Priority 2
// ============================================================================

// ---------------------------------------------------------------------------
// Generator 4: Branches — B.W conditional, B.W unconditional, BL  (~30 tests)
// ---------------------------------------------------------------------------

/// Test branch instructions: conditional, unconditional, and BL.
///
/// Conditional branches test each major condition code in both taken and
/// not-taken states.  Unconditional branches test positive and negative
/// offsets.  BL verifies both the PC delta and that LR is set.
pub fn gen_t32_branch() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32; // T bit

    // ----------------------------------------------------------------
    // B<cond>.W — T3 encoding (conditional wide branch)
    // ----------------------------------------------------------------

    // Condition codes and the flag bits that satisfy them:
    //   EQ (0x0): Z=1        NE (0x1): Z=0
    //   CS (0x2): C=1        CC (0x3): C=0
    //   MI (0x4): N=1        PL (0x5): N=0
    //   VS (0x6): V=1        VC (0x7): V=0
    //   HI (0x8): C=1 & Z=0  LS (0x9): C=0 | Z=1
    //   GE (0xA): N==V       LT (0xB): N!=V
    //   GT (0xC): Z=0 & N==V LE (0xD): Z=1 | N!=V

    let conds: &[(u16, &str, u32, u32)] = &[
        // (cond, name, flags_taken, flags_not_taken)  — flags in bits [31:28] = NZCV
        (0x0, "EQ", 1 << 30, 0),       // Z=1 / Z=0
        (0x1, "NE", 0, 1 << 30),       // Z=0 / Z=1
        (0x2, "CS", 1 << 29, 0),       // C=1 / C=0
        (0x3, "CC", 0, 1 << 29),       // C=0 / C=1
        (0x4, "MI", 1 << 31, 0),       // N=1 / N=0
        (0x5, "PL", 0, 1 << 31),       // N=0 / N=1
        (0x6, "VS", 1 << 28, 0),       // V=1 / V=0
        (0x7, "VC", 0, 1 << 28),       // V=0 / V=1
        (0x8, "HI", 1 << 29, 1 << 30), // C=1,Z=0 / Z=1
        (0x9, "LS", 1 << 30, 1 << 29), // Z=1 / C=1,Z=0
        (0xA, "GE", 0, 1 << 31),       // N=V=0 / N=1,V=0
        (0xB, "LT", 1 << 31, 0),       // N=1,V=0 / N=V=0
        (0xC, "GT", 0, 1 << 30),       // N=V=0,Z=0 / Z=1
        (0xD, "LE", 1 << 30, 0),       // Z=1 / N=V=0,Z=0
    ];

    for &(cond, name, flags_taken, flags_not_taken) in conds {
        let offset = 16i32;
        let (hw0, hw1) = enc_t32_b_cond(cond, offset);

        // Taken
        t.push(TestCase {
            name: format!("B{name}.W +16 (taken)"),
            opcode: hw0,
            hw1: Some(hw1),
            xpsr_pre: tb | flags_taken,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });

        // Not taken
        t.push(TestCase {
            name: format!("B{name}.W +16 (not taken)"),
            opcode: hw0,
            hw1: Some(hw1),
            xpsr_pre: tb | flags_not_taken,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // B.W — T4 encoding (unconditional wide branch)
    // ----------------------------------------------------------------

    for &(offset, label) in &[(16i32, "+16"), (100, "+100"), (-8, "-8"), (-100, "-100")] {
        let (hw0, hw1) = enc_t32_b_uncond(offset);
        t.push(TestCase {
            name: format!("B.W {label} (unconditional)"),
            opcode: hw0,
            hw1: Some(hw1),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // BL — branch with link
    // ----------------------------------------------------------------

    for &(offset, label) in &[(16i32, "+16"), (-8, "-8")] {
        let (hw0, hw1) = enc_t32_bl(offset);
        t.push(TestCase {
            name: format!("BL {label}"),
            opcode: hw0,
            hw1: Some(hw1),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            modifies_lr: true,
            ..TestCase::default()
        });
    }

    t
}

// ---------------------------------------------------------------------------
// Generator 5: Data processing — shifted register  (~50 tests)
// ---------------------------------------------------------------------------

/// Test data processing with shifted register operand (T32 encoding).
///
/// Covers all shift types (LSL, LSR, ASR, ROR, RRX), all major ALU
/// operations with S=1, standalone shift forms (Rn=15 → MOV), and
/// edge values.
pub fn gen_t32_dp_shift_reg() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;
    let tb_c = tb | (1 << 29); // T bit + carry set

    // Helper: build a flag-updating dp_shift_reg TestCase.
    let mk = |name: &str,
              op: u16,
              rn: u16,
              rd: u16,
              rm: u16,
              stype: u16,
              samount: u16,
              regs: Vec<(u8, u32)>,
              xpsr: u32|
     -> TestCase {
        let (hw0, hw1) = enc_t32_dp_shift_reg(op, true, rn, rd, rm, stype, samount);
        TestCase {
            name: name.into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: xpsr,
            xpsr_mask: MASK_ALL_FLAGS,
            ..TestCase::default()
        }
    };

    // ----------------------------------------------------------------
    // Shift types with ADDS (verify shift decoding)
    // ----------------------------------------------------------------

    // LSL #0 (no shift, identity)
    t.push(mk(
        "ADDS.W R0,R1,R2,LSL #0",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 10), (2, 20)],
        tb,
    ));
    // LSL #1
    t.push(mk(
        "ADDS.W R0,R1,R2,LSL #1",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSL,
        1,
        vec![(1, 10), (2, 5)],
        tb,
    ));
    // LSL #16
    t.push(mk(
        "ADDS.W R0,R1,R2,LSL #16",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSL,
        16,
        vec![(1, 0), (2, 1)],
        tb,
    ));
    // LSL #31 (max shift)
    t.push(mk(
        "ADDS.W R0,R1,R2,LSL #31",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSL,
        31,
        vec![(1, 0), (2, 1)],
        tb,
    ));

    // LSR #1
    t.push(mk(
        "ADDS.W R0,R1,R2,LSR #1",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSR,
        1,
        vec![(1, 0), (2, 0x8000_0000)],
        tb,
    ));
    // LSR #16
    t.push(mk(
        "ADDS.W R0,R1,R2,LSR #16",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSR,
        16,
        vec![(1, 0), (2, 0xFFFF_0000)],
        tb,
    ));
    // LSR #32 (encoded as imm5=0 with type=LSR)
    t.push(mk(
        "ADDS.W R0,R1,R2,LSR #32",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSR,
        0,
        vec![(1, 0), (2, 0x8000_0000)],
        tb,
    ));

    // ASR #1 (positive)
    t.push(mk(
        "ADDS.W R0,R1,R2,ASR #1 (pos)",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_ASR,
        1,
        vec![(1, 0), (2, 0x40)],
        tb,
    ));
    // ASR #16 (negative, sign-extends)
    t.push(mk(
        "ADDS.W R0,R1,R2,ASR #16 (neg)",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_ASR,
        16,
        vec![(1, 0), (2, 0x8000_0000)],
        tb,
    ));
    // ASR #32 (encoded as 0)
    t.push(mk(
        "ADDS.W R0,R1,R2,ASR #32",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_ASR,
        0,
        vec![(1, 0), (2, 0x8000_0000)],
        tb,
    ));

    // ROR #1
    t.push(mk(
        "ADDS.W R0,R1,R2,ROR #1",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_ROR,
        1,
        vec![(1, 0), (2, 1)],
        tb,
    ));
    // ROR #16
    t.push(mk(
        "ADDS.W R0,R1,R2,ROR #16",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_ROR,
        16,
        vec![(1, 0), (2, 0xFFFF)],
        tb,
    ));

    // RRX (type=ROR, amount=0): carry rotated to bit 31
    t.push(mk(
        "ADDS.W R0,R1,R2,RRX (C=1)",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_ROR,
        0,
        vec![(1, 0), (2, 0)],
        tb_c,
    ));
    t.push(mk(
        "ADDS.W R0,R1,R2,RRX (C=0)",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_ROR,
        0,
        vec![(1, 0), (2, 0xFFFF_FFFF)],
        tb,
    ));

    // ----------------------------------------------------------------
    // ALU operations with shifted operand (S=1)
    // ----------------------------------------------------------------

    // SUBS.W
    t.push(mk(
        "SUBS.W R0,R1,R2,LSL #2",
        DP_SUB,
        1,
        0,
        2,
        SHIFT_LSL,
        2,
        vec![(1, 100), (2, 10)],
        tb,
    ));
    t.push(mk(
        "SUBS.W R0,R1,R2,LSR #4 (borrow)",
        DP_SUB,
        1,
        0,
        2,
        SHIFT_LSR,
        4,
        vec![(1, 0), (2, 0x10)],
        tb,
    ));

    // CMP.W (SUB with Rd=15)
    t.push(mk(
        "CMP.W R1,R2,LSL #1 (equal)",
        DP_SUB,
        1,
        15,
        2,
        SHIFT_LSL,
        1,
        vec![(1, 20), (2, 10)],
        tb,
    ));
    t.push(mk(
        "CMP.W R3,R4,ASR #1 (less)",
        DP_SUB,
        3,
        15,
        4,
        SHIFT_ASR,
        1,
        vec![(3, 0), (4, 0x40)],
        tb,
    ));

    // ANDS.W
    t.push(mk(
        "ANDS.W R0,R1,R2,LSL #8",
        DP_AND,
        1,
        0,
        2,
        SHIFT_LSL,
        8,
        vec![(1, 0xFF00_FF00), (2, 0xFF)],
        tb,
    ));

    // TST.W (AND with Rd=15)
    t.push(mk(
        "TST.W R1,R2,LSL #16",
        DP_AND,
        1,
        15,
        2,
        SHIFT_LSL,
        16,
        vec![(1, 0xFFFF_0000), (2, 1)],
        tb,
    ));
    t.push(mk(
        "TST.W R1,R2,LSR #1 (zero)",
        DP_AND,
        1,
        15,
        2,
        SHIFT_LSR,
        1,
        vec![(1, 0x0000_0001), (2, 0x0000_0001)],
        tb,
    ));

    // ORRS.W
    t.push(mk(
        "ORRS.W R0,R1,R2,ROR #8",
        DP_ORR,
        1,
        0,
        2,
        SHIFT_ROR,
        8,
        vec![(1, 0), (2, 0xFF)],
        tb,
    ));

    // EORS.W
    t.push(mk(
        "EORS.W R0,R1,R2,LSL #4",
        DP_EOR,
        1,
        0,
        2,
        SHIFT_LSL,
        4,
        vec![(1, 0xFF), (2, 0x0F)],
        tb,
    ));

    // BICS.W
    t.push(mk(
        "BICS.W R0,R1,R2,LSL #0",
        DP_BIC,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 0xFFFF_FFFF), (2, 0x0000_00FF)],
        tb,
    ));

    // ORNS.W
    t.push(mk(
        "ORNS.W R0,R1,R2,LSL #0",
        DP_ORN,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 0), (2, 0)],
        tb,
    ));

    // ADCS.W
    t.push(mk(
        "ADCS.W R0,R1,R2,LSL #0 (C=1)",
        DP_ADC,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 10), (2, 20)],
        tb_c,
    ));

    // SBCS.W
    t.push(mk(
        "SBCS.W R0,R1,R2,LSL #1 (C=1)",
        DP_SBC,
        1,
        0,
        2,
        SHIFT_LSL,
        1,
        vec![(1, 100), (2, 10)],
        tb_c,
    ));

    // RSBS.W
    t.push(mk(
        "RSBS.W R0,R1,R2,LSL #0",
        DP_RSB,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 10), (2, 20)],
        tb,
    ));

    // ----------------------------------------------------------------
    // Rn=15 standalone shift forms (MOV.W Rd, Rm, shift)
    // ----------------------------------------------------------------

    // MOV.W = ORR with Rn=15
    // LSL
    t.push(mk(
        "MOV.W R0,R2,LSL #3 (Rn=15)",
        DP_ORR,
        15,
        0,
        2,
        SHIFT_LSL,
        3,
        vec![(2, 1)],
        tb,
    ));
    // LSR
    t.push(mk(
        "MOV.W R0,R2,LSR #4 (Rn=15)",
        DP_ORR,
        15,
        0,
        2,
        SHIFT_LSR,
        4,
        vec![(2, 0x100)],
        tb,
    ));
    // ASR
    t.push(mk(
        "MOV.W R0,R2,ASR #8 (Rn=15)",
        DP_ORR,
        15,
        0,
        2,
        SHIFT_ASR,
        8,
        vec![(2, 0x8000_0000)],
        tb,
    ));
    // ROR
    t.push(mk(
        "MOV.W R0,R2,ROR #16 (Rn=15)",
        DP_ORR,
        15,
        0,
        2,
        SHIFT_ROR,
        16,
        vec![(2, 0xDEAD_BEEF)],
        tb,
    ));
    // RRX (carry in)
    t.push(mk(
        "MOV.W R0,R2,RRX (Rn=15, C=1)",
        DP_ORR,
        15,
        0,
        2,
        SHIFT_ROR,
        0,
        vec![(2, 0)],
        tb_c,
    ));

    // TEQ.W (EOR with Rd=15, Rn!=15)
    t.push(mk(
        "TEQ.W R1,R2,LSL #0",
        DP_EOR,
        1,
        15,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 0x8000_0000), (2, 0x8000_0000)],
        tb,
    ));

    // ----------------------------------------------------------------
    // High register combos and edge values
    // ----------------------------------------------------------------

    t.push(mk(
        "ADDS.W R8,R9,R10,LSL #1 (high regs)",
        DP_ADD,
        9,
        8,
        10,
        SHIFT_LSL,
        1,
        vec![(9, 100), (10, 50)],
        tb,
    ));
    t.push(mk(
        "SUBS.W R8,R9,R10,ROR #8 (high regs)",
        DP_SUB,
        9,
        8,
        10,
        SHIFT_ROR,
        8,
        vec![(9, 0x1000), (10, 0xFF)],
        tb,
    ));

    // Edge values
    t.push(mk(
        "ADDS.W R0,R1,R2,LSL #0 (0+0)",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 0), (2, 0)],
        tb,
    ));
    t.push(mk(
        "ADDS.W R0,R1,R2,LSL #0 (MAX+1)",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 0xFFFF_FFFF), (2, 1)],
        tb,
    ));
    t.push(mk(
        "ADDS.W R0,R1,R2,LSL #0 (MIN+MIN)",
        DP_ADD,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 0x8000_0000), (2, 0x8000_0000)],
        tb,
    ));
    t.push(mk(
        "EORS.W R0,R1,R2,LSL #0 (all ones)",
        DP_EOR,
        1,
        0,
        2,
        SHIFT_LSL,
        0,
        vec![(1, 0xFFFF_FFFF), (2, 0xFFFF_FFFF)],
        tb,
    ));

    // Carry-out from shift on logical ops (S=1)
    t.push(mk(
        "ANDS.W R0,R1,R2,LSL #1 (carry out)",
        DP_AND,
        1,
        0,
        2,
        SHIFT_LSL,
        1,
        vec![(1, 0xFFFF_FFFF), (2, 0x8000_0000)],
        tb,
    ));
    t.push(mk(
        "ORRS.W R0,R1,R2,LSR #1 (carry out)",
        DP_ORR,
        1,
        0,
        2,
        SHIFT_LSR,
        1,
        vec![(1, 0), (2, 0x0000_0001)],
        tb,
    ));
    t.push(mk(
        "EORS.W R0,R1,R2,ASR #1 (carry out)",
        DP_EOR,
        1,
        0,
        2,
        SHIFT_ASR,
        1,
        vec![(1, 0), (2, 0xFFFF_FFFF)],
        tb,
    ));

    t
}

// ---------------------------------------------------------------------------
// Generator 6: LDM/STM wide  (~20 tests)
// ---------------------------------------------------------------------------

/// Test wide LDM/STM (Thumb-32 encoding).
///
/// Covers STMIA.W, STMDB.W, LDMIA.W, LDMDB.W with writeback.
/// Includes high-register tests that distinguish Thumb-32 from Thumb-16.
pub fn gen_t32_ldm_stm() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // ----------------------------------------------------------------
    // STMIA.W with writeback
    // ----------------------------------------------------------------

    // 3 low registers
    {
        let reglist = (1 << 0) | (1 << 1) | (1 << 2); // R0, R1, R2
        let (hw0, hw1) = enc_t32_stm(4, true, false, reglist);
        t.push(TestCase {
            name: "STMIA.W R4!, {R0,R1,R2}".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0x11111111), (1, 0x22222222), (2, 0x33333333), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0);
                c.extend(mem_check_u32(4));
                c.extend(mem_check_u32(8));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // High registers (R8, R9, R10) — Thumb-32 capability
    {
        let reglist = (1 << 8) | (1 << 9) | (1 << 10);
        let (hw0, hw1) = enc_t32_stm(3, true, false, reglist);
        t.push(TestCase {
            name: "STMIA.W R3!, {R8,R9,R10} (high regs)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![
                (8, 0xAAAA_AAAA),
                (9, 0xBBBB_BBBB),
                (10, 0xCCCC_CCCC),
                (3, 0x00),
            ],
            addr_regs: vec![3],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0);
                c.extend(mem_check_u32(4));
                c.extend(mem_check_u32(8));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Single register
    {
        let reglist = 1 << 0;
        let (hw0, hw1) = enc_t32_stm(4, true, false, reglist);
        t.push(TestCase {
            name: "STMIA.W R4!, {R0} (single)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xDEAD_BEEF), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: mem_check_u32(0),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Mixed low + high
    {
        let reglist = (1 << 0) | (1 << 1) | (1 << 8) | (1 << 9);
        let (hw0, hw1) = enc_t32_stm(4, true, false, reglist);
        t.push(TestCase {
            name: "STMIA.W R4!, {R0,R1,R8,R9} (mixed)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0x11), (1, 0x22), (8, 0x88), (9, 0x99), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = Vec::new();
                for i in 0..4u32 {
                    c.extend(mem_check_u32(i * 4));
                }
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // STMDB.W (decrement before) with writeback
    // ----------------------------------------------------------------

    // 3 registers: base starts at offset 12, stores decrement before
    {
        let reglist = (1 << 0) | (1 << 1) | (1 << 2);
        let (hw0, hw1) = enc_t32_stm(4, true, true, reglist); // db=true
        t.push(TestCase {
            name: "STMDB.W R4!, {R0,R1,R2}".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xAA), (1, 0xBB), (2, 0xCC), (4, 12)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0); // base-12 = offset 0
                c.extend(mem_check_u32(4));
                c.extend(mem_check_u32(8));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // STMDB.W with high registers
    {
        let reglist = (1 << 8) | (1 << 9);
        let (hw0, hw1) = enc_t32_stm(4, true, true, reglist);
        t.push(TestCase {
            name: "STMDB.W R4!, {R8,R9} (high regs)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(8, 0x1234_5678), (9, 0x9ABC_DEF0), (4, 8)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0);
                c.extend(mem_check_u32(4));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // LDMIA.W with writeback
    // ----------------------------------------------------------------

    // 3 low registers
    {
        let reglist = (1 << 0) | (1 << 1) | (1 << 2);
        let (hw0, hw1) = enc_t32_ldm(4, true, false, reglist);
        t.push(TestCase {
            name: "LDMIA.W R4!, {R0,R1,R2}".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0x11111111);
                m.extend(mem_pre_u32(4, 0x22222222));
                m.extend(mem_pre_u32(8, 0x33333333));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // High registers
    {
        let reglist = (1 << 8) | (1 << 9) | (1 << 10);
        let (hw0, hw1) = enc_t32_ldm(3, true, false, reglist);
        t.push(TestCase {
            name: "LDMIA.W R3!, {R8,R9,R10} (high regs)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(3, 0x00)],
            addr_regs: vec![3],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0xDEAD_0001);
                m.extend(mem_pre_u32(4, 0xDEAD_0002));
                m.extend(mem_pre_u32(8, 0xDEAD_0003));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Single register
    {
        let reglist = 1 << 0;
        let (hw0, hw1) = enc_t32_ldm(4, true, false, reglist);
        t.push(TestCase {
            name: "LDMIA.W R4!, {R0} (single)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: mem_pre_u32(0, 0xCAFE_BABE),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // LDMDB.W with writeback
    // ----------------------------------------------------------------

    // 3 registers: pre-populate memory at offsets 0..12, base at 12
    {
        let reglist = (1 << 0) | (1 << 1) | (1 << 2);
        let (hw0, hw1) = enc_t32_ldm(4, true, true, reglist); // db=true
        t.push(TestCase {
            name: "LDMDB.W R4!, {R0,R1,R2}".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 12)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0x1111);
                m.extend(mem_pre_u32(4, 0x2222));
                m.extend(mem_pre_u32(8, 0x3333));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // LDMDB.W high registers
    {
        let reglist = (1 << 8) | (1 << 9);
        let (hw0, hw1) = enc_t32_ldm(4, true, true, reglist);
        t.push(TestCase {
            name: "LDMDB.W R4!, {R8,R9} (high regs)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 8)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0xAAAA);
                m.extend(mem_pre_u32(4, 0xBBBB));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // No writeback (W=0)
    {
        let reglist = (1 << 0) | (1 << 1);
        let (hw0, hw1) = enc_t32_ldm(4, false, false, reglist); // w=false
        t.push(TestCase {
            name: "LDMIA.W R4, {R0,R1} (no writeback)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0x5555);
                m.extend(mem_pre_u32(4, 0x6666));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    t
}

// ---------------------------------------------------------------------------
// Generator 7: LDRD/STRD dual  (~20 tests)
// ---------------------------------------------------------------------------

/// Test LDRD/STRD (load/store dual register).
///
/// Covers offset, pre-index, and post-index addressing modes with
/// positive and negative offsets.
pub fn gen_t32_ldrd_strd() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // ----------------------------------------------------------------
    // STRD — offset mode (P=1, W=0)
    // ----------------------------------------------------------------

    // Positive offset: STRD R0, R1, [R4, #+8]
    {
        let (hw0, hw1) = enc_t32_strd(0, 1, 4, true, true, false, 2); // imm8=2 → offset=8
        t.push(TestCase {
            name: "STRD R0,R1,[R4,#+8] (offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xAAAA_BBBB), (1, 0xCCCC_DDDD), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(8);
                c.extend(mem_check_u32(12));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Zero offset: STRD R2, R3, [R4, #+0]
    {
        let (hw0, hw1) = enc_t32_strd(2, 3, 4, true, true, false, 0);
        t.push(TestCase {
            name: "STRD R2,R3,[R4,#+0] (zero offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 0x1234_5678), (3, 0x9ABC_DEF0), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0);
                c.extend(mem_check_u32(4));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Negative offset: STRD R0, R1, [R4, #-8]
    {
        let (hw0, hw1) = enc_t32_strd(0, 1, 4, true, false, false, 2); // U=0 → subtract
        t.push(TestCase {
            name: "STRD R0,R1,[R4,#-8] (neg offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0xDEAD), (1, 0xBEEF), (4, 8)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0);
                c.extend(mem_check_u32(4));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // High registers: STRD R8, R9, [R4, #+0]
    {
        let (hw0, hw1) = enc_t32_strd(8, 9, 4, true, true, false, 0);
        t.push(TestCase {
            name: "STRD R8,R9,[R4,#+0] (high regs)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(8, 0xFEDC_BA98), (9, 0x7654_3210), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0);
                c.extend(mem_check_u32(4));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // STRD — pre-index (P=1, W=1)
    // ----------------------------------------------------------------

    // STRD R0, R1, [R4, #+12]!
    {
        let (hw0, hw1) = enc_t32_strd(0, 1, 4, true, true, true, 3); // W=1, imm8=3 → +12
        t.push(TestCase {
            name: "STRD R0,R1,[R4,#+12]! (pre-index)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0x11), (1, 0x22), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(12);
                c.extend(mem_check_u32(16));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // STRD — post-index (P=0, W=1)
    // ----------------------------------------------------------------

    // STRD R0, R1, [R4], #+8
    {
        let (hw0, hw1) = enc_t32_strd(0, 1, 4, false, true, true, 2); // P=0, W=1
        t.push(TestCase {
            name: "STRD R0,R1,[R4],#+8 (post-index)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0x33), (1, 0x44), (4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_check: {
                let mut c = mem_check_u32(0); // stored at base (before update)
                c.extend(mem_check_u32(4));
                c
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // LDRD — offset mode (P=1, W=0)
    // ----------------------------------------------------------------

    // Positive offset: LDRD R0, R1, [R4, #+8]
    {
        let (hw0, hw1) = enc_t32_ldrd(0, 1, 4, true, true, false, 2);
        t.push(TestCase {
            name: "LDRD R0,R1,[R4,#+8] (offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(8, 0xDEAD_BEEF);
                m.extend(mem_pre_u32(12, 0xCAFE_BABE));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Zero offset: LDRD R2, R3, [R4, #+0]
    {
        let (hw0, hw1) = enc_t32_ldrd(2, 3, 4, true, true, false, 0);
        t.push(TestCase {
            name: "LDRD R2,R3,[R4,#+0] (zero offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0x1111_2222);
                m.extend(mem_pre_u32(4, 0x3333_4444));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Negative offset: LDRD R0, R1, [R4, #-8]
    {
        let (hw0, hw1) = enc_t32_ldrd(0, 1, 4, true, false, false, 2); // U=0
        t.push(TestCase {
            name: "LDRD R0,R1,[R4,#-8] (neg offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 8)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0x5555);
                m.extend(mem_pre_u32(4, 0x6666));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // High registers: LDRD R8, R9, [R4, #+0]
    {
        let (hw0, hw1) = enc_t32_ldrd(8, 9, 4, true, true, false, 0);
        t.push(TestCase {
            name: "LDRD R8,R9,[R4,#+0] (high regs)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0xAAAA_BBBB);
                m.extend(mem_pre_u32(4, 0xCCCC_DDDD));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // LDRD — pre-index (P=1, W=1)
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_ldrd(0, 1, 4, true, true, true, 3); // W=1, +12
        t.push(TestCase {
            name: "LDRD R0,R1,[R4,#+12]! (pre-index)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(12, 0xAA);
                m.extend(mem_pre_u32(16, 0xBB));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // LDRD — post-index (P=0, W=1)
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_ldrd(0, 1, 4, false, true, true, 2); // P=0, W=1, +8
        t.push(TestCase {
            name: "LDRD R0,R1,[R4],#+8 (post-index)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(0, 0xCC);
                m.extend(mem_pre_u32(4, 0xDD));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Edge: LDRD R0, R1, [R4, #+252] (max imm8=63, offset=252)
    {
        let (hw0, hw1) = enc_t32_ldrd(0, 1, 4, true, true, false, 63);
        t.push(TestCase {
            name: "LDRD R0,R1,[R4,#+252] (max offset)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: {
                let mut m = mem_pre_u32(252, 0xFACE);
                m.extend(mem_pre_u32(256, 0xB00C));
                m
            },
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    t
}

// ---------------------------------------------------------------------------
// Generator 8: TBB/TBH (table branch)  (~10 tests)
// ---------------------------------------------------------------------------

/// Test TBB/TBH table branch instructions.
///
/// TBB reads a byte from `[Rn + Rm]` and branches `PC + 2*byte`.
/// TBH reads a halfword from `[Rn + Rm*2]` and branches `PC + 2*halfword`.
/// In both cases, PC = instruction_addr + 4 (Thumb-32 read_pc).
pub fn gen_t32_tbb_tbh() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // ----------------------------------------------------------------
    // TBB — table branch byte
    // ----------------------------------------------------------------

    // Index 0: table[0] = 4 → branch forward by 2*4 = 8 bytes from PC
    {
        let (hw0, hw1) = enc_t32_tbb(4, 0); // Rn=R4 (base), Rm=R0 (index)
        t.push(TestCase {
            name: "TBB [R4,R0] idx=0, table[0]=4".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (0, 0)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: vec![(0, 4)], // table[0] = 4
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Index 1: table[1] = 10 → branch forward by 20 bytes from PC
    {
        let (hw0, hw1) = enc_t32_tbb(4, 0);
        t.push(TestCase {
            name: "TBB [R4,R0] idx=1, table[1]=10".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (0, 1)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: vec![(1, 10)], // table[1] = 10
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Index 2 with different index register
    {
        let (hw0, hw1) = enc_t32_tbb(4, 1); // Rm=R1
        t.push(TestCase {
            name: "TBB [R4,R1] idx=2, table[2]=0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (1, 2)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: vec![(2, 0)], // table[2] = 0 → PC + 0 = fall-through
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Max byte value: table[0] = 255 → branch forward by 510 bytes
    {
        let (hw0, hw1) = enc_t32_tbb(4, 0);
        t.push(TestCase {
            name: "TBB [R4,R0] idx=0, table[0]=255 (max)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (0, 0)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: vec![(0, 255)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Nonzero base offset: table starts at offset 16
    {
        let (hw0, hw1) = enc_t32_tbb(4, 0);
        t.push(TestCase {
            name: "TBB [R4,R0] base+16, idx=3, table[3]=7".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 16), (0, 3)], // base=16, index=3, addr=16+3=19
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: vec![(19, 7)], // table[3] at offset 19 = 7
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // TBH — table branch halfword
    // ----------------------------------------------------------------

    // Index 0: table[0] = 0x0008 → branch forward by 2*8 = 16 bytes from PC
    {
        let (hw0, hw1) = enc_t32_tbh(4, 0);
        t.push(TestCase {
            name: "TBH [R4,R0,LSL#1] idx=0, table[0]=8".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (0, 0)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: mem_pre_u16(0, 8), // halfword at offset 0
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Index 1: table[1] = 0x0020 → branch forward by 64 bytes
    {
        let (hw0, hw1) = enc_t32_tbh(4, 0);
        t.push(TestCase {
            name: "TBH [R4,R0,LSL#1] idx=1, table[1]=32".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (0, 1)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: mem_pre_u16(2, 32), // halfword at offset 2 (index 1 * 2)
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Index 2, different Rm register
    {
        let (hw0, hw1) = enc_t32_tbh(4, 1); // Rm=R1
        t.push(TestCase {
            name: "TBH [R4,R1,LSL#1] idx=2, table[2]=0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (1, 2)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: mem_pre_u16(4, 0), // halfword at offset 4 (index 2 * 2) = 0
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // Large halfword value: table[0] = 0x00FF → branch forward by 510 bytes
    {
        let (hw0, hw1) = enc_t32_tbh(4, 0);
        t.push(TestCase {
            name: "TBH [R4,R0,LSL#1] idx=0, table[0]=0x00FF".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(4, 0x00), (0, 0)],
            addr_regs: vec![4],
            needs_bus: true,
            mem_pre: mem_pre_u16(0, 0x00FF),
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    t
}

// ============================================================================
// Test generators — Priority 3
// ============================================================================

// ---------------------------------------------------------------------------
// Generator: Data processing — plain binary immediate  (~30 tests)
// ---------------------------------------------------------------------------

/// Test MOVW, MOVT, ADDW, SUBW, BFI, BFC, SBFX, UBFX.
/// None of these set flags (xpsr_mask = MASK_NO_FLAGS).
pub fn gen_t32_dp_plain_imm() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // Helper for plain-binary-imm tests (no flags).
    let mk = |name: &str, hw0: u16, hw1: u16, regs: Vec<(u8, u32)>| -> TestCase {
        TestCase {
            name: name.into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }
    };

    // ----------------------------------------------------------------
    // MOVW — load 16-bit immediate, zero-extend to 32 bits
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_movw(0, 0);
        t.push(mk("MOVW R0,#0", hw0, hw1, vec![]));
    }
    {
        let (hw0, hw1) = enc_t32_movw(0, 1);
        t.push(mk("MOVW R0,#1", hw0, hw1, vec![]));
    }
    {
        let (hw0, hw1) = enc_t32_movw(0, 0xFF);
        t.push(mk("MOVW R0,#0xFF", hw0, hw1, vec![]));
    }
    {
        let (hw0, hw1) = enc_t32_movw(3, 0x1234);
        t.push(mk("MOVW R3,#0x1234", hw0, hw1, vec![]));
    }
    {
        let (hw0, hw1) = enc_t32_movw(5, 0xFFFF);
        t.push(mk("MOVW R5,#0xFFFF", hw0, hw1, vec![]));
    }
    // Verify upper bits cleared: pre-set R8 to 0xDEAD_0000, MOVW should overwrite to imm16
    {
        let (hw0, hw1) = enc_t32_movw(8, 0x0042);
        t.push(mk(
            "MOVW R8,#0x42 (clears upper)",
            hw0,
            hw1,
            vec![(8, 0xDEAD_0000)],
        ));
    }
    // High register R12
    {
        let (hw0, hw1) = enc_t32_movw(12, 0xABCD);
        t.push(mk("MOVW R12,#0xABCD (high reg)", hw0, hw1, vec![]));
    }

    // ----------------------------------------------------------------
    // MOVT — write upper 16 bits, preserve lower 16
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_movt(0, 0);
        t.push(mk(
            "MOVT R0,#0 (upper=0, lower preserved)",
            hw0,
            hw1,
            vec![(0, 0x0000_1234)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_movt(2, 0x5678);
        t.push(mk(
            "MOVT R2,#0x5678 (pair: lower=0x1234)",
            hw0,
            hw1,
            vec![(2, 0x0000_1234)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_movt(1, 0xFFFF);
        t.push(mk(
            "MOVT R1,#0xFFFF (max upper)",
            hw0,
            hw1,
            vec![(1, 0x0000_BEEF)],
        ));
    }

    // ----------------------------------------------------------------
    // ADDW — 12-bit immediate add, no flags
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_addw(0, 1, 42);
        t.push(mk("ADDW R0,R1,#42", hw0, hw1, vec![(1, 100)]));
    }
    {
        let (hw0, hw1) = enc_t32_addw(3, 3, 0xFFF);
        t.push(mk("ADDW R3,R3,#0xFFF (max imm12)", hw0, hw1, vec![(3, 0)]));
    }
    {
        let (hw0, hw1) = enc_t32_addw(5, 0, 0);
        t.push(mk("ADDW R5,R0,#0 (identity)", hw0, hw1, vec![(0, 0xDEAD)]));
    }

    // ----------------------------------------------------------------
    // SUBW — 12-bit immediate subtract, no flags
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_subw(0, 1, 10);
        t.push(mk("SUBW R0,R1,#10", hw0, hw1, vec![(1, 100)]));
    }
    {
        let (hw0, hw1) = enc_t32_subw(2, 2, 100);
        t.push(mk("SUBW R2,R2,#100 (to zero)", hw0, hw1, vec![(2, 100)]));
    }
    {
        let (hw0, hw1) = enc_t32_subw(4, 4, 1);
        t.push(mk(
            "SUBW R4,R4,#1 (from zero, wraps)",
            hw0,
            hw1,
            vec![(4, 0)],
        ));
    }

    // ----------------------------------------------------------------
    // BFI — bit field insert
    // ----------------------------------------------------------------

    // Insert bottom byte
    {
        let (hw0, hw1) = enc_t32_bfi(0, 1, 0, 8);
        t.push(mk(
            "BFI R0,R1,#0,#8 (insert low byte)",
            hw0,
            hw1,
            vec![(0, 0xFFFF_FF00), (1, 0x42)],
        ));
    }
    // Insert upper half
    {
        let (hw0, hw1) = enc_t32_bfi(0, 1, 16, 16);
        t.push(mk(
            "BFI R0,R1,#16,#16 (insert upper half)",
            hw0,
            hw1,
            vec![(0, 0x0000_ABCD), (1, 0x1234)],
        ));
    }
    // Single bit insertion
    {
        let (hw0, hw1) = enc_t32_bfi(3, 2, 4, 1);
        t.push(mk(
            "BFI R3,R2,#4,#1 (single bit)",
            hw0,
            hw1,
            vec![(3, 0x0000_0000), (2, 0x0000_0001)],
        ));
    }

    // ----------------------------------------------------------------
    // BFC — bit field clear
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_bfc(0, 0, 8);
        t.push(mk(
            "BFC R0,#0,#8 (clear low byte)",
            hw0,
            hw1,
            vec![(0, 0xFFFF_FFFF)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_bfc(1, 8, 16);
        t.push(mk(
            "BFC R1,#8,#16 (clear middle bits)",
            hw0,
            hw1,
            vec![(1, 0xFFFF_FFFF)],
        ));
    }

    // ----------------------------------------------------------------
    // SBFX — signed bit field extract
    // ----------------------------------------------------------------

    // Extract byte at bit 0, width 8: value 0x80 → sign-extend → 0xFFFFFF80
    {
        let (hw0, hw1) = enc_t32_sbfx(0, 1, 0, 8);
        t.push(mk(
            "SBFX R0,R1,#0,#8 (neg: 0x80)",
            hw0,
            hw1,
            vec![(1, 0x0000_0080)],
        ));
    }
    // Extract byte at bit 0, width 8: value 0x42 → positive, zero-extended
    {
        let (hw0, hw1) = enc_t32_sbfx(0, 1, 0, 8);
        t.push(mk(
            "SBFX R0,R1,#0,#8 (pos: 0x42)",
            hw0,
            hw1,
            vec![(1, 0x0000_0042)],
        ));
    }
    // Extract at non-zero lsb
    {
        let (hw0, hw1) = enc_t32_sbfx(2, 3, 16, 8);
        t.push(mk(
            "SBFX R2,R3,#16,#8 (extract upper byte)",
            hw0,
            hw1,
            vec![(3, 0x00FF_0000)],
        ));
    }

    // ----------------------------------------------------------------
    // UBFX — unsigned bit field extract
    // ----------------------------------------------------------------

    // Extract byte at bit 0, width 8: value 0x80 → 0x80 (no sign extension)
    {
        let (hw0, hw1) = enc_t32_ubfx(0, 1, 0, 8);
        t.push(mk(
            "UBFX R0,R1,#0,#8 (0x80, no sign-ext)",
            hw0,
            hw1,
            vec![(1, 0xFFFF_FF80)],
        ));
    }
    // Extract nibble at bit 4
    {
        let (hw0, hw1) = enc_t32_ubfx(0, 1, 4, 4);
        t.push(mk(
            "UBFX R0,R1,#4,#4 (nibble)",
            hw0,
            hw1,
            vec![(1, 0xABCD_EF56)],
        ));
    }
    // Extract upper 16 bits
    {
        let (hw0, hw1) = enc_t32_ubfx(5, 6, 16, 16);
        t.push(mk(
            "UBFX R5,R6,#16,#16 (upper half)",
            hw0,
            hw1,
            vec![(6, 0x1234_5678)],
        ));
    }

    t
}

// ---------------------------------------------------------------------------
// Generator: DSP instructions  (~40 tests)
// ---------------------------------------------------------------------------

/// Test SSAT, USAT, QADD/QSUB/QDADD/QDSUB, parallel add/sub, SEL.
pub fn gen_t32_dsp() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;
    let tb_q = tb | (1 << 27); // T bit + Q flag set

    // ----------------------------------------------------------------
    // SSAT — signed saturate
    // ----------------------------------------------------------------

    // In range: SSAT #16 on value 100 → no saturation
    {
        let (hw0, hw1) = enc_t32_ssat(0, 1, 16, SHIFT_LSL, 0);
        t.push(TestCase {
            name: "SSAT R0,#16,R1 (in range, 100)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 100)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // Exceed positive: SSAT #8 on 200 → saturated to 127
    {
        let (hw0, hw1) = enc_t32_ssat(0, 1, 8, SHIFT_LSL, 0);
        t.push(TestCase {
            name: "SSAT R0,#8,R1 (pos overflow, 200 -> 127)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 200)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // Below negative: SSAT #8 on -200 → saturated to -128
    {
        let (hw0, hw1) = enc_t32_ssat(0, 1, 8, SHIFT_LSL, 0);
        t.push(TestCase {
            name: "SSAT R0,#8,R1 (neg overflow, -200 -> -128)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, (-200i32) as u32)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // SSAT with ASR shift applied before saturation
    {
        let (hw0, hw1) = enc_t32_ssat(2, 3, 8, SHIFT_ASR, 4);
        t.push(TestCase {
            name: "SSAT R2,#8,R3,ASR#4 (shifted)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(3, 0x0000_0FF0)], // >> 4 = 0xFF → exceeds 8-bit signed max
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // USAT — unsigned saturate
    // ----------------------------------------------------------------

    // In range
    {
        let (hw0, hw1) = enc_t32_usat(0, 1, 8, SHIFT_LSL, 0);
        t.push(TestCase {
            name: "USAT R0,#8,R1 (in range, 100)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 100)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // Negative → saturated to 0
    {
        let (hw0, hw1) = enc_t32_usat(0, 1, 8, SHIFT_LSL, 0);
        t.push(TestCase {
            name: "USAT R0,#8,R1 (negative -> 0)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, (-50i32) as u32)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // Exceed limit: USAT #8 on 300 → saturated to 255
    {
        let (hw0, hw1) = enc_t32_usat(0, 1, 8, SHIFT_LSL, 0);
        t.push(TestCase {
            name: "USAT R0,#8,R1 (exceeds 255 -> 255)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 300)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // USAT with LSL shift
    {
        let (hw0, hw1) = enc_t32_usat(4, 5, 16, SHIFT_LSL, 2);
        t.push(TestCase {
            name: "USAT R4,#16,R5,LSL#2 (shifted)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(5, 0x0000_4000)], // << 2 = 0x10000, exceeds 16-bit
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // QADD / QSUB / QDADD / QDSUB
    // ----------------------------------------------------------------

    // NOTE: The emulator's QADD/QSUB/QDADD/QDSUB implementation currently
    // wraps on overflow instead of clamping to INT32_MAX/INT32_MIN as the ARM
    // spec requires. These saturation tests will fail against QEMU until the
    // emulator is fixed. The tests are correct — the failures validate that
    // the differential harness catches this class of bug.

    // QADD: normal (no saturation)
    {
        let (hw0, hw1) = enc_t32_qadd(0, 1, 2);
        t.push(TestCase {
            name: "QADD R0,R1,R2 (normal, 10+20)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 10), (2, 20)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // QADD: saturation (INT32_MAX + 1)
    {
        let (hw0, hw1) = enc_t32_qadd(0, 1, 2);
        t.push(TestCase {
            name: "QADD R0,R1,R2 (saturating, MAX+1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x7FFF_FFFF), (2, 1)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // QADD: negative saturation
    {
        let (hw0, hw1) = enc_t32_qadd(0, 1, 2);
        t.push(TestCase {
            name: "QADD R0,R1,R2 (saturating, MIN-1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x8000_0000), (2, (-1i32) as u32)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // QSUB: normal
    {
        let (hw0, hw1) = enc_t32_qsub(0, 1, 2);
        t.push(TestCase {
            name: "QSUB R0,R1,R2 (normal, 30-10)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 30), (2, 10)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // QSUB: saturation
    {
        let (hw0, hw1) = enc_t32_qsub(0, 1, 2);
        t.push(TestCase {
            name: "QSUB R0,R1,R2 (saturating, MIN-1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x8000_0000), (2, 1)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // QDADD: normal
    {
        let (hw0, hw1) = enc_t32_qdadd(0, 1, 2);
        t.push(TestCase {
            name: "QDADD R0,R1,R2 (normal, 10+2*5)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 10), (2, 5)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // QDADD: double saturates
    {
        let (hw0, hw1) = enc_t32_qdadd(0, 1, 2);
        t.push(TestCase {
            name: "QDADD R0,R1,R2 (double saturates, 2*MAX)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0), (2, 0x7FFF_FFFF)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // QDSUB: normal
    {
        let (hw0, hw1) = enc_t32_qdsub(0, 1, 2);
        t.push(TestCase {
            name: "QDSUB R0,R1,R2 (normal, 20-2*3)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 20), (2, 3)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // QDSUB: double saturates
    {
        let (hw0, hw1) = enc_t32_qdsub(0, 1, 2);
        t.push(TestCase {
            name: "QDSUB R0,R1,R2 (double saturates, 2*MIN)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0), (2, 0x8000_0000)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // Q flag stickiness: pre-set Q=1, no saturation, Q stays 1
    {
        let (hw0, hw1) = enc_t32_qadd(0, 1, 2);
        t.push(TestCase {
            name: "QADD R0,R1,R2 (Q sticky: pre=1, no sat)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 1), (2, 1)],
            xpsr_pre: tb_q,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // Parallel add/subtract — SADD16, SSUB16, UADD8, USUB8
    // ----------------------------------------------------------------
    // SADD16: operation=ADD16=0b001, modifier=signed=0b000
    {
        let (hw0, hw1) = enc_t32_parallel(0b001, 0b000, 1, 0, 2);
        t.push(TestCase {
            name: "SADD16 R0,R1,R2 (packed 16-bit add)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0003_0005), (2, 0x0001_0002)], // hi: 3+1=4, lo: 5+2=7
            xpsr_pre: tb,
            xpsr_mask: MASK_ALL_FLAGS_GE,
            ..TestCase::default()
        });
    }
    // SADD16 with negative result in one lane
    {
        let (hw0, hw1) = enc_t32_parallel(0b001, 0b000, 1, 0, 2);
        t.push(TestCase {
            name: "SADD16 R0,R1,R2 (neg lane)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            // hi: 0xFFFF + 0x0001 = 0, lo: 0x8000 + 0x0001 = 0x8001 (negative)
            reg_pre: vec![(1, 0xFFFF_8000), (2, 0x0001_0001)],
            xpsr_pre: tb,
            xpsr_mask: MASK_ALL_FLAGS_GE,
            ..TestCase::default()
        });
    }
    // SSUB16: operation=SUB16=0b101, modifier=signed=0b000
    {
        let (hw0, hw1) = enc_t32_parallel(0b101, 0b000, 1, 0, 2);
        t.push(TestCase {
            name: "SSUB16 R0,R1,R2 (packed 16-bit sub)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0005_000A), (2, 0x0001_0002)], // hi: 5-1=4, lo: 10-2=8
            xpsr_pre: tb,
            xpsr_mask: MASK_ALL_FLAGS_GE,
            ..TestCase::default()
        });
    }

    // UADD8: operation=ADD8=0b000, modifier=unsigned=0b100
    {
        let (hw0, hw1) = enc_t32_parallel(0b000, 0b100, 1, 0, 2);
        t.push(TestCase {
            name: "UADD8 R0,R1,R2 (packed 8-bit add)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            // lanes: 0x01+0x02=0x03, 0x10+0x20=0x30, 0x80+0x80=0x100(carry), 0xFF+0x01=0x100(carry)
            reg_pre: vec![(1, 0xFF80_1001), (2, 0x0180_2002)],
            xpsr_pre: tb,
            xpsr_mask: MASK_ALL_FLAGS_GE,
            ..TestCase::default()
        });
    }
    // USUB8: operation=SUB8=0b100, modifier=unsigned=0b100
    {
        let (hw0, hw1) = enc_t32_parallel(0b100, 0b100, 1, 0, 2);
        t.push(TestCase {
            name: "USUB8 R0,R1,R2 (packed 8-bit sub)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0A_05_FF_80), (2, 0x01_02_01_01)],
            xpsr_pre: tb,
            xpsr_mask: MASK_ALL_FLAGS_GE,
            ..TestCase::default()
        });
    }

    // SADD8: operation=ADD8=0b000, modifier=signed=0b000
    {
        let (hw0, hw1) = enc_t32_parallel(0b000, 0b000, 1, 0, 2);
        t.push(TestCase {
            name: "SADD8 R0,R1,R2 (signed 8-bit add)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x01_FE_7F_80), (2, 0x01_01_01_01)],
            xpsr_pre: tb,
            xpsr_mask: MASK_ALL_FLAGS_GE,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // SEL — select bytes based on GE flags
    // ----------------------------------------------------------------
    // SEL: op1_65=0b01, op2_54=0b00
    // hw0 = 0xFAA0 | Rn, hw1 = 0xF080 | (Rd << 8) | Rm

    // GE = 0b1010: select bytes 1,3 from Rn; bytes 0,2 from Rm
    {
        let hw0 = 0xFAA0 | 1; // Rn=R1
        let hw1: u16 = 0xF080 | 2; // Rd=R0, Rm=R2
        // GE[3:0] stored in xPSR bits [19:16]
        let ge_flags = 0b1010u32;
        t.push(TestCase {
            name: "SEL R0,R1,R2 (GE=0b1010)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0xAA_BB_CC_DD), (2, 0x11_22_33_44)],
            xpsr_pre: tb | (ge_flags << 16),
            xpsr_mask: MASK_NO_FLAGS, // SEL doesn't modify flags
            ..TestCase::default()
        });
    }
    // GE = 0b1111: all bytes from Rn
    {
        let hw0 = 0xFAA0 | 3;
        let hw1: u16 = 0xF080 | 4;
        t.push(TestCase {
            name: "SEL R0,R3,R4 (GE=0b1111, all from Rn)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(3, 0xDEAD_BEEF), (4, 0x1234_5678)],
            xpsr_pre: tb | (0xF << 16),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // GE = 0b0000: all bytes from Rm
    {
        let hw0 = 0xFAA0 | 3;
        let hw1: u16 = 0xF080 | 4;
        t.push(TestCase {
            name: "SEL R0,R3,R4 (GE=0b0000, all from Rm)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(3, 0xDEAD_BEEF), (4, 0x1234_5678)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // Saturating add/sub with parallel: QADD16 (operation=ADD16=0b001, modifier=Q-signed=0b001)
    // ----------------------------------------------------------------
    {
        let (hw0, hw1) = enc_t32_parallel(0b001, 0b001, 1, 0, 2);
        t.push(TestCase {
            name: "QADD16 R0,R1,R2 (saturating 16-bit add)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            // 0x7FFF + 0x0001 → saturated to 0x7FFF, 0x0001 + 0x0001 = 0x0002
            reg_pre: vec![(1, 0x7FFF_0001), (2, 0x0001_0001)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // UADD16: operation=ADD16=0b001, modifier=unsigned=0b100
    {
        let (hw0, hw1) = enc_t32_parallel(0b001, 0b100, 1, 0, 2);
        t.push(TestCase {
            name: "UADD16 R0,R1,R2 (unsigned 16-bit add)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0100_FF00), (2, 0x0200_0100)],
            xpsr_pre: tb,
            xpsr_mask: MASK_ALL_FLAGS_GE,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // DSP halfword multiply (SMULBB/BT/TB/TT, SMLABB/BT/TB/TT)
    // ----------------------------------------------------------------

    // SMULBB: bottom×bottom, 3 × 4 = 12
    {
        let (hw0, hw1) = enc_t32_smulxy(0, 1, 2, false, false);
        t.push(TestCase {
            name: "SMULBB R0,R1,R2 (3*4=12)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0xAAAA_0003), (2, 0xBBBB_0004)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // SMULTT: top×top, 5 × 6 = 30
    {
        let (hw0, hw1) = enc_t32_smulxy(0, 1, 2, true, true);
        t.push(TestCase {
            name: "SMULTT R0,R1,R2 (5*6=30)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0005_CCCC), (2, 0x0006_DDDD)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // SMULBT: bottom(Rn)×top(Rm), 7 × (-1) = -7
    {
        let (hw0, hw1) = enc_t32_smulxy(0, 1, 2, false, true);
        t.push(TestCase {
            name: "SMULBT R0,R1,R2 (7*(-1)=-7)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0000_0007), (2, 0xFFFF_0000)], // top=0xFFFF=-1
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // SMLABB: multiply-accumulate, 3*4+100=112
    {
        let (hw0, hw1) = enc_t32_smlabb(0, 1, 2, 3, false, false);
        t.push(TestCase {
            name: "SMLABB R0,R1,R2,R3 (3*4+100=112)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 3), (2, 4), (3, 100)],
            xpsr_pre: tb,
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }
    // SMULBB edge: 0x7FFF × 0x7FFF = 0x3FFF0001 (max positive × max positive)
    {
        let (hw0, hw1) = enc_t32_smulxy(0, 1, 2, false, false);
        t.push(TestCase {
            name: "SMULBB R0,R1,R2 (0x7FFF*0x7FFF)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0000_7FFF), (2, 0x0000_7FFF)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // SMULBB edge: 0x8000 × 0x8000 = 0x40000000 (min negative × min negative)
    // -32768 × -32768 = 1073741824 = 0x40000000
    {
        let (hw0, hw1) = enc_t32_smulxy(0, 1, 2, false, false);
        t.push(TestCase {
            name: "SMULBB R0,R1,R2 (0x8000*0x8000)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x0000_8000), (2, 0x0000_8000)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    t
}

// ---------------------------------------------------------------------------
// Generator: Data processing — register (misc)  (~30 tests)
// ---------------------------------------------------------------------------

/// Test CLZ, RBIT, REV.W, REV16.W, REVSH.W, wide register shifts,
/// and extend instructions (SXTB/UXTB/SXTH/UXTH with rotation).
pub fn gen_t32_dp_register() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // Helper for register-op tests (no flags).
    let mk = |name: &str, hw0: u16, hw1: u16, regs: Vec<(u8, u32)>| -> TestCase {
        TestCase {
            name: name.into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        }
    };

    // ----------------------------------------------------------------
    // CLZ — count leading zeros
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_clz(0, 1);
        t.push(mk(
            "CLZ R0,R1 (0x00000000 -> 32)",
            hw0,
            hw1,
            vec![(1, 0x0000_0000)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_clz(0, 1);
        t.push(mk(
            "CLZ R0,R1 (0x00000001 -> 31)",
            hw0,
            hw1,
            vec![(1, 0x0000_0001)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_clz(0, 1);
        t.push(mk(
            "CLZ R0,R1 (0x80000000 -> 0)",
            hw0,
            hw1,
            vec![(1, 0x8000_0000)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_clz(0, 1);
        t.push(mk(
            "CLZ R0,R1 (0x0000FFFF -> 16)",
            hw0,
            hw1,
            vec![(1, 0x0000_FFFF)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_clz(0, 1);
        t.push(mk(
            "CLZ R0,R1 (0xFFFFFFFF -> 0)",
            hw0,
            hw1,
            vec![(1, 0xFFFF_FFFF)],
        ));
    }

    // ----------------------------------------------------------------
    // RBIT — reverse bits
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_rbit(0, 1);
        t.push(mk(
            "RBIT R0,R1 (0x00000001 -> 0x80000000)",
            hw0,
            hw1,
            vec![(1, 0x0000_0001)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_rbit(0, 1);
        t.push(mk(
            "RBIT R0,R1 (0x80000000 -> 0x00000001)",
            hw0,
            hw1,
            vec![(1, 0x8000_0000)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_rbit(0, 1);
        t.push(mk(
            "RBIT R0,R1 (0x0F0F0F0F -> 0xF0F0F0F0)",
            hw0,
            hw1,
            vec![(1, 0x0F0F_0F0F)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_rbit(0, 1);
        t.push(mk(
            "RBIT R0,R1 (0x00000000 -> 0x00000000)",
            hw0,
            hw1,
            vec![(1, 0x0000_0000)],
        ));
    }

    // ----------------------------------------------------------------
    // REV.W — byte reverse full word
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_rev_w(0, 1);
        t.push(mk(
            "REV.W R0,R1 (0x12345678 -> 0x78563412)",
            hw0,
            hw1,
            vec![(1, 0x1234_5678)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_rev_w(0, 1);
        t.push(mk(
            "REV.W R0,R1 (0x00000001 -> 0x01000000)",
            hw0,
            hw1,
            vec![(1, 0x0000_0001)],
        ));
    }

    // ----------------------------------------------------------------
    // REV16.W — byte reverse each halfword
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_rev16_w(0, 1);
        t.push(mk(
            "REV16.W R0,R1 (0x12345678 -> 0x34127856)",
            hw0,
            hw1,
            vec![(1, 0x1234_5678)],
        ));
    }

    // ----------------------------------------------------------------
    // REVSH.W — byte-reverse low halfword and sign-extend
    // ----------------------------------------------------------------

    {
        let (hw0, hw1) = enc_t32_revsh_w(0, 1);
        t.push(mk(
            "REVSH.W R0,R1 (0x0000FF80 -> sign-ext)",
            hw0,
            hw1,
            vec![(1, 0x0000_FF80)],
        ));
    }
    {
        let (hw0, hw1) = enc_t32_revsh_w(0, 1);
        t.push(mk(
            "REVSH.W R0,R1 (0x00000001 -> 0x00000100)",
            hw0,
            hw1,
            vec![(1, 0x0000_0001)],
        ));
    }

    // ----------------------------------------------------------------
    // Wide register shifts: LSL.W, LSR.W, ASR.W, ROR.W
    // ----------------------------------------------------------------

    // LSL.W by 0 (identity)
    {
        let (hw0, hw1) = enc_t32_lsl_w_reg(0, 1, 2);
        t.push(mk(
            "LSL.W R0,R1,R2 (shift=0, identity)",
            hw0,
            hw1,
            vec![(1, 0xDEAD_BEEF), (2, 0)],
        ));
    }
    // LSL.W by 1
    {
        let (hw0, hw1) = enc_t32_lsl_w_reg(0, 1, 2);
        t.push(mk(
            "LSL.W R0,R1,R2 (shift=1)",
            hw0,
            hw1,
            vec![(1, 0x4000_0000), (2, 1)],
        ));
    }
    // LSL.W by 31
    {
        let (hw0, hw1) = enc_t32_lsl_w_reg(0, 1, 2);
        t.push(mk(
            "LSL.W R0,R1,R2 (shift=31)",
            hw0,
            hw1,
            vec![(1, 1), (2, 31)],
        ));
    }
    // LSL.W by 32 (gives 0)
    {
        let (hw0, hw1) = enc_t32_lsl_w_reg(0, 1, 2);
        t.push(mk(
            "LSL.W R0,R1,R2 (shift=32, gives 0)",
            hw0,
            hw1,
            vec![(1, 0xFFFF_FFFF), (2, 32)],
        ));
    }

    // LSR.W by 1
    {
        let (hw0, hw1) = enc_t32_lsr_w_reg(0, 1, 2);
        t.push(mk(
            "LSR.W R0,R1,R2 (shift=1)",
            hw0,
            hw1,
            vec![(1, 0x8000_0000), (2, 1)],
        ));
    }
    // LSR.W by 32 (gives 0)
    {
        let (hw0, hw1) = enc_t32_lsr_w_reg(0, 1, 2);
        t.push(mk(
            "LSR.W R0,R1,R2 (shift=32, gives 0)",
            hw0,
            hw1,
            vec![(1, 0xFFFF_FFFF), (2, 32)],
        ));
    }

    // ASR.W by 31 (sign fills)
    {
        let (hw0, hw1) = enc_t32_asr_w_reg(0, 1, 2);
        t.push(mk(
            "ASR.W R0,R1,R2 (shift=31, neg)",
            hw0,
            hw1,
            vec![(1, 0x8000_0000), (2, 31)],
        ));
    }
    // ASR.W by 32 (all sign)
    {
        let (hw0, hw1) = enc_t32_asr_w_reg(0, 1, 2);
        t.push(mk(
            "ASR.W R0,R1,R2 (shift=32, all sign)",
            hw0,
            hw1,
            vec![(1, 0x8000_0000), (2, 32)],
        ));
    }

    // ROR.W by 0 (identity)
    {
        let (hw0, hw1) = enc_t32_ror_w_reg(0, 1, 2);
        t.push(mk(
            "ROR.W R0,R1,R2 (shift=0, identity)",
            hw0,
            hw1,
            vec![(1, 0xDEAD_BEEF), (2, 0)],
        ));
    }
    // ROR.W by 16
    {
        let (hw0, hw1) = enc_t32_ror_w_reg(0, 1, 2);
        t.push(mk(
            "ROR.W R0,R1,R2 (shift=16)",
            hw0,
            hw1,
            vec![(1, 0x1234_5678), (2, 16)],
        ));
    }

    // ----------------------------------------------------------------
    // Extend instructions with rotation
    // ----------------------------------------------------------------

    // SXTB.W: 0x80 → 0xFFFFFF80
    {
        let (hw0, hw1) = enc_t32_sxtb_w(0, 1, 0);
        t.push(mk(
            "SXTB.W R0,R1 (0x80 -> sign-ext)",
            hw0,
            hw1,
            vec![(1, 0x0000_0080)],
        ));
    }
    // UXTB.W: 0x80 → 0x00000080
    {
        let (hw0, hw1) = enc_t32_uxtb_w(0, 1, 0);
        t.push(mk(
            "UXTB.W R0,R1 (0x80 -> zero-ext)",
            hw0,
            hw1,
            vec![(1, 0xFFFF_FF80)],
        ));
    }
    // SXTH.W: 0x8000 → 0xFFFF8000
    {
        let (hw0, hw1) = enc_t32_sxth_w(0, 1, 0);
        t.push(mk(
            "SXTH.W R0,R1 (0x8000 -> sign-ext)",
            hw0,
            hw1,
            vec![(1, 0x0000_8000)],
        ));
    }
    // UXTH.W: 0x8000 → 0x00008000
    {
        let (hw0, hw1) = enc_t32_uxth_w(0, 1, 0);
        t.push(mk(
            "UXTH.W R0,R1 (0x8000 -> zero-ext)",
            hw0,
            hw1,
            vec![(1, 0xFFFF_8000)],
        ));
    }
    // SXTB.W with rotation 8: ROR #8 first, then sign-extend byte
    {
        let (hw0, hw1) = enc_t32_sxtb_w(0, 1, 8);
        t.push(mk(
            "SXTB.W R0,R1,ROR#8 (rot then sign-ext)",
            hw0,
            hw1,
            vec![(1, 0x0000_FF00)],
        )); // ROR 8 → 0x000000FF, byte=0xFF → sign-ext
    }
    // UXTB.W with rotation 16
    {
        let (hw0, hw1) = enc_t32_uxtb_w(0, 1, 16);
        t.push(mk(
            "UXTB.W R0,R1,ROR#16 (rot then zero-ext)",
            hw0,
            hw1,
            vec![(1, 0x00AB_0000)],
        )); // ROR 16 → 0x0000_00AB, byte=0xAB
    }

    // ----------------------------------------------------------------
    // Extend-and-add instructions (SXTAB, UXTAB, SXTAH, UXTAH)
    // ----------------------------------------------------------------

    // SXTAB: sign-extend byte from Rm, add to Rn
    // Rm=0x80 → sign-extends to 0xFFFFFF80 (-128), Rn=256 → 256+(-128) = 128
    {
        let (hw0, hw1) = enc_t32_sxtab(0, 1, 2, 0);
        t.push(mk(
            "SXTAB R0,R1,R2 (0x80 sign-ext + 256 = 128)",
            hw0,
            hw1,
            vec![(1, 256), (2, 0x0000_0080)],
        ));
    }
    // UXTAB: zero-extend byte from Rm, add to Rn
    // Rm=0xFF80 → byte=0x80, zero-extends to 128, Rn=100 → 228
    {
        let (hw0, hw1) = enc_t32_uxtab(0, 1, 2, 0);
        t.push(mk(
            "UXTAB R0,R1,R2 (0x80 zero-ext + 100 = 228)",
            hw0,
            hw1,
            vec![(1, 100), (2, 0x0000_FF80)],
        ));
    }
    // SXTAH: sign-extend halfword from Rm, add to Rn
    // Rm=0x8000 → sign-extends to 0xFFFF8000 (-32768), Rn=0x10000 → 0x10000-0x8000 = 0x8000
    {
        let (hw0, hw1) = enc_t32_sxtah(0, 1, 2, 0);
        t.push(mk(
            "SXTAH R0,R1,R2 (0x8000 sign-ext + 0x10000)",
            hw0,
            hw1,
            vec![(1, 0x0001_0000), (2, 0x0000_8000)],
        ));
    }
    // UXTAH: zero-extend halfword from Rm, add to Rn
    // Rm=0xFFFF8000 → halfword=0x8000, zero-extends to 0x8000, Rn=0x100 → 0x8100
    {
        let (hw0, hw1) = enc_t32_uxtah(0, 1, 2, 0);
        t.push(mk(
            "UXTAH R0,R1,R2 (0x8000 zero-ext + 0x100)",
            hw0,
            hw1,
            vec![(1, 0x0000_0100), (2, 0xFFFF_8000)],
        ));
    }
    // SXTAB with rotation: rot=1 (ROR #8), Rm=0x0000_FF80
    // After ROR 8: 0x800000FF, byte = 0xFF → sign-extends to -1, Rn=10 → 9
    {
        let (hw0, hw1) = enc_t32_sxtab(0, 1, 2, 8);
        t.push(mk(
            "SXTAB R0,R1,R2,ROR#8 (rot then sign-ext+add)",
            hw0,
            hw1,
            vec![(1, 10), (2, 0x0000_FF00)],
        )); // ROR 8 → 0x000000FF, byte=0xFF→-1, 10-1=9
    }
    // UXTAB with rotation: rot=2 (ROR #16), Rm=0x00AB_0000
    // After ROR 16: 0x0000_00AB, byte = 0xAB → zero-extends to 0xAB, Rn=5 → 5+0xAB=0xB0
    {
        let (hw0, hw1) = enc_t32_uxtab(0, 1, 2, 16);
        t.push(mk(
            "UXTAB R0,R1,R2,ROR#16 (rot then zero-ext+add)",
            hw0,
            hw1,
            vec![(1, 5), (2, 0x00AB_0000)],
        )); // ROR 16 → 0x0000_00AB, 5+0xAB=0xB0
    }
    // SXTAH with rotation: rot=1 (ROR #8), Rm=0x0080_FF00
    // After ROR 8: 0x000080FF, halfword = 0x80FF → sign-extends to 0xFFFF80FF (-32513), Rn=0x10000
    {
        let (hw0, hw1) = enc_t32_sxtah(0, 1, 2, 8);
        t.push(mk(
            "SXTAH R0,R1,R2,ROR#8 (rot then sign-ext hw+add)",
            hw0,
            hw1,
            vec![(1, 0x0001_0000), (2, 0x0080_FF00)],
        ));
    }
    // UXTAH with rotation: rot=1 (ROR #8), Rm=0x0080_FF00
    // After ROR 8: 0x000080FF, halfword = 0x80FF → zero-extends to 0x80FF, Rn=1 → 0x8100
    {
        let (hw0, hw1) = enc_t32_uxtah(0, 1, 2, 8);
        t.push(mk(
            "UXTAH R0,R1,R2,ROR#8 (rot then zero-ext hw+add)",
            hw0,
            hw1,
            vec![(1, 1), (2, 0x0080_FF00)],
        ));
    }

    t
}

// ---------------------------------------------------------------------------
// Generator: Miscellaneous control  (~15 tests)
// ---------------------------------------------------------------------------

/// Test MSR, MRS, NOP.W.
pub fn gen_t32_misc_control() -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    // ----------------------------------------------------------------
    // MSR — write to special register (verify no GPR corruption)
    // ----------------------------------------------------------------

    // MSR PRIMASK, R0 (sysm=16) — set PRIMASK to 1
    {
        let (hw0, hw1) = enc_t32_msr(0, 16);
        t.push(TestCase {
            name: "MSR PRIMASK,R0 (sysm=16, val=1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 1)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // MSR BASEPRI, R1 (sysm=17)
    {
        let (hw0, hw1) = enc_t32_msr(1, 17);
        t.push(TestCase {
            name: "MSR BASEPRI,R1 (sysm=17, val=0x40)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(1, 0x40)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // MSR FAULTMASK, R2 (sysm=19)
    {
        let (hw0, hw1) = enc_t32_msr(2, 19);
        t.push(TestCase {
            name: "MSR FAULTMASK,R2 (sysm=19, val=1)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(2, 1)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // MSR PRIMASK, R0 — set to 0 (disable)
    {
        let (hw0, hw1) = enc_t32_msr(0, 16);
        t.push(TestCase {
            name: "MSR PRIMASK,R0 (sysm=16, val=0)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(0, 0)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // MRS — read special register into Rd
    // ----------------------------------------------------------------

    // MRS R0, PRIMASK (sysm=16)
    {
        let (hw0, hw1) = enc_t32_mrs(0, 16);
        t.push(TestCase {
            name: "MRS R0,PRIMASK (sysm=16)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // MRS R1, BASEPRI (sysm=17)
    {
        let (hw0, hw1) = enc_t32_mrs(1, 17);
        t.push(TestCase {
            name: "MRS R1,BASEPRI (sysm=17)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // MRS R2, CONTROL (sysm=20)
    {
        let (hw0, hw1) = enc_t32_mrs(2, 20);
        t.push(TestCase {
            name: "MRS R2,CONTROL (sysm=20)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // MRS into high register
    {
        let (hw0, hw1) = enc_t32_mrs(8, 16);
        t.push(TestCase {
            name: "MRS R8,PRIMASK (high reg)".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // NOP.W — verify no state change
    // ----------------------------------------------------------------

    // NOP.W: hw0=0xF3AF, hw1=0x8000
    {
        t.push(TestCase {
            name: "NOP.W (no state change)".into(),
            opcode: 0xF3AF,
            hw1: Some(0x8000),
            reg_pre: vec![(0, 0xDEAD_BEEF), (1, 0xCAFE_BABE)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // NOP.W with flags pre-set — verify flags preserved
    {
        t.push(TestCase {
            name: "NOP.W (flags preserved)".into(),
            opcode: 0xF3AF,
            hw1: Some(0x8000),
            reg_pre: vec![],
            xpsr_pre: tb | (0xF << 28), // NZCV all set
            xpsr_mask: MASK_ALL_FLAGS,
            ..TestCase::default()
        });
    }

    // ----------------------------------------------------------------
    // DMB/DSB/ISB — barrier hints (should be NOPs in emulation)
    // ----------------------------------------------------------------

    // DMB: hw0=0xF3BF, hw1=0x8F5F (option=0xF, op=5=DMB)
    {
        t.push(TestCase {
            name: "DMB (barrier, no state change)".into(),
            opcode: 0xF3BF,
            hw1: Some(0x8F5F),
            reg_pre: vec![(0, 42)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // DSB: hw0=0xF3BF, hw1=0x8F4F (option=0xF, op=4=DSB)
    {
        t.push(TestCase {
            name: "DSB (barrier, no state change)".into(),
            opcode: 0xF3BF,
            hw1: Some(0x8F4F),
            reg_pre: vec![(0, 42)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    // ISB: hw0=0xF3BF, hw1=0x8F6F (option=0xF, op=6=ISB)
    {
        t.push(TestCase {
            name: "ISB (barrier, no state change)".into(),
            opcode: 0xF3BF,
            hw1: Some(0x8F6F),
            reg_pre: vec![(0, 42)],
            xpsr_pre: tb,
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    t
}

// ============================================================================
// Fuzz test generators
// ============================================================================

use crate::RngExt;
use rand::rngs::StdRng;

/// All valid DP op constants (not contiguous: 0-4, 8, 10-11, 13-14).
const DP_OPS: [u16; 10] = [
    DP_AND, DP_BIC, DP_ORR, DP_ORN, DP_EOR, DP_ADD, DP_ADC, DP_SBC, DP_SUB, DP_RSB,
];

/// Random register R0-R12 (avoids SP=13 and PC=15).
fn rand_reg(rng: &mut StdRng) -> u16 {
    rng.range(0..13)
}

/// Random 32-bit value.
fn rand_val(rng: &mut StdRng) -> u32 {
    rng.random()
}

/// Random xPSR flags (N, Z, C, V in bits 31:28) with T bit set.
fn rand_flags(rng: &mut StdRng) -> u32 {
    let flags: u32 = rng.range(0..16);
    0x0100_0000 | (flags << 28)
}

/// All GP registers R0-R12 set to random values.
fn rand_gp_regs(rng: &mut StdRng) -> Vec<(u8, u32)> {
    (0..13).map(|i| (i, rand_val(rng))).collect()
}

/// Biased f32 bit pattern: 10% NaN, 10% ±Inf, 10% denormal, 10% ±zero, 60% normal.
fn biased_f32(rng: &mut StdRng) -> u32 {
    let r: f64 = rng.range(0.0..1.0);
    if r < 0.1 {
        // NaN: exponent=0xFF, fraction!=0
        0x7F80_0000 | rng.range(1u32..0x0080_0000)
    } else if r < 0.2 {
        // Inf: +/- Inf
        if rng.coin(0.5) {
            0x7F80_0000
        } else {
            0xFF80_0000
        }
    } else if r < 0.3 {
        // Denormal: exponent=0, fraction!=0
        rng.range(1u32..0x0080_0000)
    } else if r < 0.4 {
        // Zero: +/- 0
        if rng.coin(0.5) {
            0x0000_0000
        } else {
            0x8000_0000
        }
    } else {
        // Normal
        rng.random()
    }
}

/// Generate `count` random Thumb-32 ALU fuzz tests per instruction class.
pub fn generate_fuzz_t32_alu(count: usize, rng: &mut StdRng) -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- DP modified immediate ---
    for i in 0..count {
        let op = DP_OPS[rng.range(0..DP_OPS.len())];
        let rn = rand_reg(rng);
        let rd = rand_reg(rng);
        let imm12: u16 = rng.range(0..0x1000);
        let (hw0, hw1) = enc_t32_dp_mod_imm(op, true, rn, rd, imm12);
        let mut regs = rand_gp_regs(rng);
        // Ensure Rn has the random value already in the list
        regs.retain(|&(r, _)| r != rn as u8);
        regs.push((rn as u8, rand_val(rng)));
        t.push(TestCase {
            name: format!("FUZZ:T32_DP_IMM:{i} op={op} R{rd},R{rn},#{imm12:#05x}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- DP shifted register ---
    for i in 0..count {
        let op = DP_OPS[rng.range(0..DP_OPS.len())];
        let rn = rand_reg(rng);
        let rd = rand_reg(rng);
        let rm = rand_reg(rng);
        let stype: u16 = rng.range(0..4);
        let samount: u16 = rng.range(0..32);
        let (hw0, hw1) = enc_t32_dp_shift_reg(op, true, rn, rd, rm, stype, samount);
        let mut regs = rand_gp_regs(rng);
        // Ensure Rn and Rm have explicit random values
        regs.retain(|&(r, _)| r != rn as u8 && r != rm as u8);
        regs.push((rn as u8, rand_val(rng)));
        regs.push((rm as u8, rand_val(rng)));
        t.push(TestCase {
            name: format!("FUZZ:T32_DP_SREG:{i} op={op} R{rd},R{rn},R{rm},sh={stype}#{samount}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Multiply (MUL/MLA/MLS) ---
    for i in 0..count {
        let variant: u8 = rng.range(0..3);
        let rd = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rd {
                break r;
            }
        };
        let rm = loop {
            let r = rand_reg(rng);
            if r != rd && r != rn {
                break r;
            }
        };
        let mut regs = rand_gp_regs(rng);
        let (name_tag, hw0, hw1) = match variant {
            0 => {
                let (h0, h1) = enc_t32_mul(rd, rn, rm);
                ("MUL", h0, h1)
            }
            1 => {
                let ra = loop {
                    let r = rand_reg(rng);
                    if r != rd && r != rn && r != rm {
                        break r;
                    }
                };
                regs.retain(|&(r, _)| r != ra as u8);
                regs.push((ra as u8, rand_val(rng)));
                let (h0, h1) = enc_t32_mla(rd, rn, rm, ra);
                ("MLA", h0, h1)
            }
            _ => {
                let ra = loop {
                    let r = rand_reg(rng);
                    if r != rd && r != rn && r != rm {
                        break r;
                    }
                };
                regs.retain(|&(r, _)| r != ra as u8);
                regs.push((ra as u8, rand_val(rng)));
                let (h0, h1) = enc_t32_mls(rd, rn, rm, ra);
                ("MLS", h0, h1)
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_MUL:{i} {name_tag} R{rd},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Division (SDIV/UDIV) ---
    for i in 0..count {
        let signed = rng.coin(0.5);
        let rd = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rd {
                break r;
            }
        };
        let rm = loop {
            let r = rand_reg(rng);
            if r != rd && r != rn {
                break r;
            }
        };
        let mut regs = rand_gp_regs(rng);
        // Occasionally test division by zero
        let divisor: u32 = if rng.coin(0.1) { 0 } else { rand_val(rng) };
        regs.retain(|&(r, _)| r != rm as u8 && r != rn as u8);
        regs.push((rn as u8, rand_val(rng)));
        regs.push((rm as u8, divisor));
        let (hw0, hw1) = if signed {
            enc_t32_sdiv(rd, rn, rm)
        } else {
            enc_t32_udiv(rd, rn, rm)
        };
        let tag = if signed { "SDIV" } else { "UDIV" };
        t.push(TestCase {
            name: format!("FUZZ:T32_DIV:{i} {tag} R{rd},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Long multiply (SMULL/UMULL/SMLAL/UMLAL) ---
    for i in 0..count {
        let variant: u8 = rng.range(0..4);
        let rdlo = rand_reg(rng);
        let rdhi = loop {
            let r = rand_reg(rng);
            if r != rdlo {
                break r;
            }
        };
        let rn = rand_reg(rng);
        let rm = rand_reg(rng);
        let regs = rand_gp_regs(rng);
        let (tag, hw0, hw1) = match variant {
            0 => {
                let (h0, h1) = enc_t32_smull(rdlo, rdhi, rn, rm);
                ("SMULL", h0, h1)
            }
            1 => {
                let (h0, h1) = enc_t32_umull(rdlo, rdhi, rn, rm);
                ("UMULL", h0, h1)
            }
            2 => {
                let (h0, h1) = enc_t32_smlal(rdlo, rdhi, rn, rm);
                ("SMLAL", h0, h1)
            }
            _ => {
                let (h0, h1) = enc_t32_umlal(rdlo, rdhi, rn, rm);
                ("UMLAL", h0, h1)
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_LMUL:{i} {tag} R{rdlo},R{rdhi},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Halfword multiply (SMUL<x><y>) ---
    for i in 0..count {
        let n_high = rng.coin(0.5);
        let m_high = rng.coin(0.5);
        let rd = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rd {
                break r;
            }
        };
        let rm = loop {
            let r = rand_reg(rng);
            if r != rd && r != rn {
                break r;
            }
        };
        let regs = rand_gp_regs(rng);
        let (hw0, hw1) = enc_t32_smulxy(rd, rn, rm, n_high, m_high);
        let nt = if n_high { 'T' } else { 'B' };
        let mt = if m_high { 'T' } else { 'B' };
        t.push(TestCase {
            name: format!("FUZZ:T32_SMULxy:{i} SMUL{nt}{mt} R{rd},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Halfword multiply-accumulate (SMLA<x><y>) ---
    for i in 0..count {
        let n_high = rng.coin(0.5);
        let m_high = rng.coin(0.5);
        let rd = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rd {
                break r;
            }
        };
        let rm = loop {
            let r = rand_reg(rng);
            if r != rd && r != rn {
                break r;
            }
        };
        let ra = loop {
            let r = rand_reg(rng);
            if r != rd && r != rn && r != rm {
                break r;
            }
        };
        let regs = rand_gp_regs(rng);
        let (hw0, hw1) = enc_t32_smlabb(rd, rn, rm, ra, n_high, m_high);
        let nt = if n_high { 'T' } else { 'B' };
        let mt = if m_high { 'T' } else { 'B' };
        // SMLA<x><y> writes Q on overflow of the 32-bit accumulate
        t.push(TestCase {
            name: format!("FUZZ:T32_SMLAxy:{i} SMLA{nt}{mt} R{rd},R{rn},R{rm},R{ra}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // --- SMMUL family (SMMUL/SMMULR, SMMLA/SMMLAR, SMMLS/SMMLSR) ---
    // None of these write flags — the top 32 bits of a 64-bit product cannot
    // overflow a signed 32-bit accumulate in any way that Arm's spec cares
    // about, so no Q bit is ever set here.
    for i in 0..count {
        let variant: u8 = rng.range(0..3);
        let round = rng.coin(0.5);
        let rd = rand_reg(rng);
        let rn = rand_reg(rng);
        let rm = rand_reg(rng);
        let regs = rand_gp_regs(rng);
        let (tag, hw0, hw1) = match variant {
            0 => {
                let (h0, h1) = enc_t32_smmul(rd, rn, rm, round);
                (if round { "SMMULR" } else { "SMMUL" }, h0, h1)
            }
            1 => {
                let ra = rand_reg(rng);
                let (h0, h1) = enc_t32_smmla(rd, rn, rm, ra, round);
                (if round { "SMMLAR" } else { "SMMLA" }, h0, h1)
            }
            _ => {
                let ra = rand_reg(rng);
                let (h0, h1) = enc_t32_smmls(rd, rn, rm, ra, round);
                (if round { "SMMLSR" } else { "SMMLS" }, h0, h1)
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_SMM:{i} {tag} R{rd},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Dual halfword (SMUAD/SMUADX/SMLAD/SMLADX/SMUSD/SMUSDX/SMLSD/SMLSDX) ---
    // SMLAD/SMLSD can set Q on overflow of the 32-bit accumulate. SMUAD can
    // also set Q when the two halfword products overflow the intermediate
    // 32-bit sum. Use MASK_Q_ONLY across the class.
    for i in 0..count {
        let variant: u8 = rng.range(0..4);
        let cross = rng.coin(0.5);
        let rd = rand_reg(rng);
        let rn = rand_reg(rng);
        let rm = rand_reg(rng);
        let regs = rand_gp_regs(rng);
        let (tag, hw0, hw1) = match variant {
            0 => {
                let (h0, h1) = enc_t32_smuad(rd, rn, rm, cross);
                (if cross { "SMUADX" } else { "SMUAD" }, h0, h1)
            }
            1 => {
                let ra = rand_reg(rng);
                let (h0, h1) = enc_t32_smlad(rd, rn, rm, ra, cross);
                (if cross { "SMLADX" } else { "SMLAD" }, h0, h1)
            }
            2 => {
                let (h0, h1) = enc_t32_smusd(rd, rn, rm, cross);
                (if cross { "SMUSDX" } else { "SMUSD" }, h0, h1)
            }
            _ => {
                let ra = rand_reg(rng);
                let (h0, h1) = enc_t32_smlsd(rd, rn, rm, ra, cross);
                (if cross { "SMLSDX" } else { "SMLSD" }, h0, h1)
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_DUALH:{i} {tag} R{rd},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // --- Word x halfword (SMULWB/SMULWT, SMLAWB/SMLAWT) ---
    // SMLAW can set Q on overflow of the 32-bit accumulate.
    for i in 0..count {
        let variant: u8 = rng.range(0..2);
        let m_high = rng.coin(0.5);
        let rd = rand_reg(rng);
        let rn = rand_reg(rng);
        let rm = rand_reg(rng);
        let regs = rand_gp_regs(rng);
        let mt = if m_high { 'T' } else { 'B' };
        let (tag, hw0, hw1) = if variant == 0 {
            let (h0, h1) = enc_t32_smulw(rd, rn, rm, m_high);
            (format!("SMULW{mt}"), h0, h1)
        } else {
            let ra = rand_reg(rng);
            let (h0, h1) = enc_t32_smlaw(rd, rn, rm, ra, m_high);
            (format!("SMLAW{mt}"), h0, h1)
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_SMULW:{i} {tag} R{rd},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // --- Long halfword (SMLAL<x><y>, SMLALD/SMLALDX, SMLSLD/SMLSLDX) ---
    // All three write the RdLo:RdHi pair and never touch flags.
    for i in 0..count {
        let variant: u8 = rng.range(0..3);
        let rdlo = rand_reg(rng);
        let rdhi = loop {
            let r = rand_reg(rng);
            if r != rdlo {
                break r;
            }
        };
        let rn = rand_reg(rng);
        let rm = rand_reg(rng);
        let regs = rand_gp_regs(rng);
        let (tag, hw0, hw1) = match variant {
            0 => {
                let n_high = rng.coin(0.5);
                let m_high = rng.coin(0.5);
                let nt = if n_high { 'T' } else { 'B' };
                let mt = if m_high { 'T' } else { 'B' };
                let (h0, h1) = enc_t32_smlalxy(rdlo, rdhi, rn, rm, n_high, m_high);
                (format!("SMLAL{nt}{mt}"), h0, h1)
            }
            1 => {
                let cross = rng.coin(0.5);
                let (h0, h1) = enc_t32_smlald(rdlo, rdhi, rn, rm, cross);
                (
                    if cross {
                        "SMLALDX".to_string()
                    } else {
                        "SMLALD".to_string()
                    },
                    h0,
                    h1,
                )
            }
            _ => {
                let cross = rng.coin(0.5);
                let (h0, h1) = enc_t32_smlsld(rdlo, rdhi, rn, rm, cross);
                (
                    if cross {
                        "SMLSLDX".to_string()
                    } else {
                        "SMLSLD".to_string()
                    },
                    h0,
                    h1,
                )
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_LMULH:{i} {tag} R{rdlo},R{rdhi},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Special DSP (UMAAL, USAD8, USADA8) ---
    // None of these write flags.
    for i in 0..count {
        let variant: u8 = rng.range(0..3);
        let regs = rand_gp_regs(rng);
        let (tag, hw0, hw1) = match variant {
            0 => {
                let rdlo = rand_reg(rng);
                let rdhi = loop {
                    let r = rand_reg(rng);
                    if r != rdlo {
                        break r;
                    }
                };
                let rn = rand_reg(rng);
                let rm = rand_reg(rng);
                let (h0, h1) = enc_t32_umaal(rdlo, rdhi, rn, rm);
                (format!("UMAAL R{rdlo},R{rdhi},R{rn},R{rm}"), h0, h1)
            }
            1 => {
                let rd = rand_reg(rng);
                let rn = rand_reg(rng);
                let rm = rand_reg(rng);
                let (h0, h1) = enc_t32_usad8(rd, rn, rm);
                (format!("USAD8 R{rd},R{rn},R{rm}"), h0, h1)
            }
            _ => {
                let rd = rand_reg(rng);
                let rn = rand_reg(rng);
                let rm = rand_reg(rng);
                let ra = rand_reg(rng);
                let (h0, h1) = enc_t32_usada8(rd, rn, rm, ra);
                (format!("USADA8 R{rd},R{rn},R{rm},R{ra}"), h0, h1)
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_DSPSP:{i} {tag}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Saturating arithmetic (QADD/QSUB/QDADD/QDSUB) ---
    for i in 0..count {
        let variant: u8 = rng.range(0..4);
        let rd = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rd {
                break r;
            }
        };
        let rm = loop {
            let r = rand_reg(rng);
            if r != rd && r != rn {
                break r;
            }
        };
        let mut regs = rand_gp_regs(rng);
        // Bias ~20% of operand values toward saturation edges. The outer four
        // hit the QADD/QSUB boundary; the inner four hit the 2*Rn doubling
        // boundary that QDADD/QDSUB saturate on.
        let edge_val = |rng: &mut StdRng| -> u32 {
            if rng.coin(0.2) {
                let pick: u8 = rng.range(0..8);
                match pick {
                    0 => 0x7FFF_FFFF,
                    1 => 0x8000_0000,
                    2 => 0x7FFF_FFFE,
                    3 => 0x8000_0001,
                    4 => 0x4000_0000,
                    5 => 0x3FFF_FFFF,
                    6 => 0xC000_0000,
                    _ => 0xBFFF_FFFF,
                }
            } else {
                rand_val(rng)
            }
        };
        regs.retain(|&(r, _)| r != rn as u8 && r != rm as u8);
        regs.push((rn as u8, edge_val(rng)));
        regs.push((rm as u8, edge_val(rng)));
        let (tag, hw0, hw1) = match variant {
            0 => {
                let (h0, h1) = enc_t32_qadd(rd, rn, rm);
                ("QADD", h0, h1)
            }
            1 => {
                let (h0, h1) = enc_t32_qsub(rd, rn, rm);
                ("QSUB", h0, h1)
            }
            2 => {
                let (h0, h1) = enc_t32_qdadd(rd, rn, rm);
                ("QDADD", h0, h1)
            }
            _ => {
                let (h0, h1) = enc_t32_qdsub(rd, rn, rm);
                ("QDSUB", h0, h1)
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:T32_QSAT:{i} {tag} R{rd},R{rn},R{rm}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // --- Parallel add/subtract (SADD16, UADD8, QSAX, UHSUB16, ...) ---
    // Valid modifier (par_op2): signed=0b000, Q=0b001, H=0b010, unsigned=0b100, UQ=0b101, UH=0b110.
    // Valid operation (par_op1): ADD8=0b000, ADD16=0b001, ASX=0b010, SUB8=0b100, SUB16=0b101, SAX=0b110.
    // Sat/halving modifiers are 16-bit only — par_op1 must then be one of {001,010,101,110}.
    for i in 0..count {
        let modifiers = [0b000u16, 0b001, 0b010, 0b100, 0b101, 0b110];
        let modifier = modifiers[rng.range(0..modifiers.len())];
        let sixteen_only = matches!(modifier, 0b001 | 0b010 | 0b101 | 0b110);
        let operation: u16 = if sixteen_only {
            let ops16 = [0b001u16, 0b010, 0b110, 0b101];
            ops16[rng.range(0..ops16.len())]
        } else {
            let ops_any = [0b000u16, 0b001, 0b010, 0b110, 0b100, 0b101];
            ops_any[rng.range(0..ops_any.len())]
        };
        let rd = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rd {
                break r;
            }
        };
        let rm = loop {
            let r = rand_reg(rng);
            if r != rd && r != rn {
                break r;
            }
        };
        let regs = rand_gp_regs(rng);
        let (hw0, hw1) = enc_t32_parallel(operation, modifier, rn, rd, rm);
        // Plain signed/unsigned variants set GE flags; sat and halving do not.
        let mask = if modifier == 0b000 || modifier == 0b100 {
            MASK_ALL_FLAGS_GE
        } else {
            MASK_NO_FLAGS
        };
        t.push(TestCase {
            name: format!(
                "FUZZ:T32_PARADD:{i} op={operation:03b} mod={modifier:03b} R{rd},R{rn},R{rm}"
            ),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: mask,
            ..TestCase::default()
        });
    }

    // --- Signed/unsigned saturate (SSAT / USAT) ---
    for i in 0..count {
        let is_signed = rng.coin(0.5);
        let rd = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rd {
                break r;
            }
        };
        // Randomise shift type/amount. ASR#0 is UNPREDICTABLE; keep ASR in 1..=31.
        let use_asr = rng.coin(0.5);
        let (stype, samount) = if use_asr {
            (SHIFT_ASR, rng.range(1..32))
        } else {
            (SHIFT_LSL, rng.range(0..32))
        };
        // Bias 30% toward values close to signed saturation bounds.
        let mut regs = rand_gp_regs(rng);
        let val: u32 = if rng.coin(0.3) {
            let pick: u8 = rng.range(0..4);
            match pick {
                0 => 0x7FFF_FFFF,
                1 => 0x8000_0000,
                2 => rand_val(rng) & 0x0000_FFFF,
                _ => (rand_val(rng) | 0xFFFF_0000) ^ 0x8000_0000,
            }
        } else {
            rand_val(rng)
        };
        regs.retain(|&(r, _)| r != rn as u8);
        regs.push((rn as u8, val));
        let (tag, sat, hw0, hw1) = if is_signed {
            // SSAT encodes (sat-1) into 5 bits → valid sat is 1..=32.
            let sat: u16 = rng.range(1..33);
            let (h0, h1) = enc_t32_ssat(rd, rn, sat, stype, samount);
            ("SSAT", sat, h0, h1)
        } else {
            // USAT encodes sat into 5 bits → valid sat is 0..=31.
            let sat: u16 = rng.range(0..32);
            let (h0, h1) = enc_t32_usat(rd, rn, sat, stype, samount);
            ("USAT", sat, h0, h1)
        };
        let sh = if stype == SHIFT_ASR { "ASR" } else { "LSL" };
        t.push(TestCase {
            name: format!("FUZZ:T32_SAT:{i} {tag} R{rd},#{sat},R{rn},{sh}#{samount}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_Q_ONLY,
            ..TestCase::default()
        });
    }

    // --- Conditional branch B<cond>.W ---
    for i in 0..count {
        let cond: u16 = rng.range(0..14); // 0-13, excluding 14/15
        // Safe offset: -1048576..+1048574, must be even. Keep small to stay safe.
        let half: i32 = rng.range(-512..512);
        let offset: i32 = half * 2;
        let (hw0, hw1) = enc_t32_b_cond(cond, offset);
        t.push(TestCase {
            name: format!("FUZZ:T32_BCOND:{i} cond={cond} off={offset}"),
            opcode: hw0,
            hw1: Some(hw1),
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    t
}

/// Generate `count` random Thumb-32 memory fuzz tests per instruction class.
pub fn generate_fuzz_t32_mem(count: usize, rng: &mut StdRng) -> Vec<TestCase> {
    use crate::{EMU_TEST_SLOT, SCRATCH_SIZE};
    let mut t = Vec::new();

    // --- Load/store single (imm12) ---
    for i in 0..count {
        // size: 0=byte, 1=half, 2=word
        let size_sel: u8 = rng.range(0..3);
        let is_load = rng.coin(0.5);
        let (size, align, max_off) = match size_sel {
            0 => (0u16, 1u32, SCRATCH_SIZE - 1), // byte
            1 => (1u16, 2u32, SCRATCH_SIZE - 2), // half
            _ => (2u16, 4u32, SCRATCH_SIZE - 4), // word
        };
        let rt = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rt {
                break r;
            }
        };
        let imm12 = (rng.range(0..max_off / align) * align) as u16;
        let offset = imm12 as u32;
        let (hw0, hw1) = enc_t32_ls_imm12(size, is_load, false, rn, rt, imm12);

        let mut reg_pre: Vec<(u8, u32)> = rand_gp_regs(rng);
        reg_pre.retain(|&(r, _)| r != rn as u8);
        reg_pre.push((rn as u8, 0)); // base at scratch start

        let (mem_pre, mem_check) = if is_load {
            let data: u32 = rand_val(rng);
            let mp = match size_sel {
                0 => vec![(offset, data as u8)],
                1 => mem_pre_u16(offset, data as u16),
                _ => mem_pre_u32(offset, data),
            };
            (mp, Vec::new())
        } else {
            let data: u32 = rand_val(rng);
            reg_pre.retain(|&(r, _)| r != rt as u8);
            reg_pre.push((rt as u8, data));
            let mc = match size_sel {
                0 => vec![offset],
                1 => mem_check_u16(offset),
                _ => mem_check_u32(offset),
            };
            (Vec::new(), mc)
        };

        let tag = if is_load { "LDR" } else { "STR" };
        let sz = ["B", "H", ""][size_sel as usize];
        t.push(TestCase {
            name: format!("FUZZ:T32_LS12:{i} {tag}{sz} R{rt},[R{rn},#{imm12}]"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre,
            addr_regs: vec![rn as u8],
            needs_bus: true,
            mem_pre,
            mem_check,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Load/store single (imm8 P/U/W) ---
    for i in 0..count {
        let size_sel: u8 = rng.range(0..3);
        let is_load = rng.coin(0.5);
        let (size, align) = match size_sel {
            0 => (0u16, 1u32),
            1 => (1u16, 2u32),
            _ => (2u16, 4u32),
        };

        let rt = rand_reg(rng);
        let rn = loop {
            let r = rand_reg(rng);
            if r != rt {
                break r;
            }
        };

        // P/U/W: avoid P=0,U=0,W=0 (undefined). For W=1, keep base in middle of scratch.
        let p = rng.coin(0.7);
        let u = rng.coin(0.5);
        let w = if p { rng.coin(0.3) } else { true }; // post-index requires W=1
        let max_imm8 = if w { 32u16 } else { 64 }; // small offsets for writeback safety
        let imm8: u16 = (rng.range(0..max_imm8) / align as u16) * align as u16;

        // Base offset: place base in middle of scratch so +/- offsets stay in range
        let base_offset: u32 = SCRATCH_SIZE / 2;
        let effective_offset = if u {
            base_offset + imm8 as u32
        } else {
            base_offset - imm8 as u32
        };

        let (hw0, hw1) = enc_t32_ls_imm8(size, is_load, false, rn, rt, p, u, w, imm8);

        let mut reg_pre: Vec<(u8, u32)> = rand_gp_regs(rng);
        reg_pre.retain(|&(r, _)| r != rn as u8);
        reg_pre.push((rn as u8, base_offset));

        let (mem_pre, mem_check) = if is_load {
            let data: u32 = rand_val(rng);
            let mp = match size_sel {
                0 => vec![(effective_offset, data as u8)],
                1 => mem_pre_u16(effective_offset, data as u16),
                _ => mem_pre_u32(effective_offset, data),
            };
            (mp, Vec::new())
        } else {
            let data: u32 = rand_val(rng);
            reg_pre.retain(|&(r, _)| r != rt as u8);
            reg_pre.push((rt as u8, data));
            let mc = match size_sel {
                0 => vec![effective_offset],
                1 => mem_check_u16(effective_offset),
                _ => mem_check_u32(effective_offset),
            };
            (Vec::new(), mc)
        };

        let tag = if is_load { "LDR" } else { "STR" };
        let sz = ["B", "H", ""][size_sel as usize];
        t.push(TestCase {
            name: format!("FUZZ:T32_LS8:{i} {tag}{sz} R{rt},[R{rn},#±{imm8}] P={p} U={u} W={w}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre,
            addr_regs: vec![rn as u8],
            needs_bus: true,
            mem_pre,
            mem_check,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- LDRD/STRD ---
    for i in 0..count {
        let is_load = rng.coin(0.5);
        let rt = rand_reg(rng);
        let rt2 = loop {
            let r = rand_reg(rng);
            if r != rt {
                break r;
            }
        };
        let rn = loop {
            let r = rand_reg(rng);
            if r != rt && r != rt2 {
                break r;
            }
        };

        let p = rng.coin(0.7);
        let u = rng.coin(0.5);
        let w = if p { rng.coin(0.3) } else { true };
        // imm8 is in words (offset = imm8 * 4), keep small for scratch safety
        let imm8: u16 = rng.range(0..32);
        let byte_offset = imm8 as u32 * 4;

        // Place base in middle of scratch
        let base_offset: u32 = SCRATCH_SIZE / 2;
        let effective_offset = if u {
            base_offset + byte_offset
        } else {
            base_offset - byte_offset
        };

        let (hw0, hw1) = if is_load {
            enc_t32_ldrd(rt, rt2, rn, p, u, w, imm8)
        } else {
            enc_t32_strd(rt, rt2, rn, p, u, w, imm8)
        };

        let mut reg_pre: Vec<(u8, u32)> = rand_gp_regs(rng);
        reg_pre.retain(|&(r, _)| r != rn as u8);
        reg_pre.push((rn as u8, base_offset));

        let (mem_pre, mem_check) = if is_load {
            let d0: u32 = rand_val(rng);
            let d1: u32 = rand_val(rng);
            let mut mp = mem_pre_u32(effective_offset, d0);
            mp.extend(mem_pre_u32(effective_offset + 4, d1));
            (mp, Vec::new())
        } else {
            let d0: u32 = rand_val(rng);
            let d1: u32 = rand_val(rng);
            reg_pre.retain(|&(r, _)| r != rt as u8 && r != rt2 as u8);
            reg_pre.push((rt as u8, d0));
            reg_pre.push((rt2 as u8, d1));
            let mut mc = mem_check_u32(effective_offset);
            mc.extend(mem_check_u32(effective_offset + 4));
            (Vec::new(), mc)
        };

        let tag = if is_load { "LDRD" } else { "STRD" };
        t.push(TestCase {
            name: format!("FUZZ:T32_DRD:{i} {tag} R{rt},R{rt2},[R{rn},#±{byte_offset}]"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre,
            addr_regs: vec![rn as u8],
            needs_bus: true,
            mem_pre,
            mem_check,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- LDM/STM.W ---
    for i in 0..count {
        let is_load = rng.coin(0.5);
        let db = rng.coin(0.3); // LDMDB/STMDB vs LDMIA/STMIA
        let rn: u16 = rng.range(0..13);

        // Build reglist: 2-5 low registers, exclude rn to avoid conflicts
        let num_regs: usize = rng.range(2..6);
        let mut reglist: u16 = 0;
        let mut picked = 0;
        while picked < num_regs {
            let r: u16 = rng.range(0..13);
            if r != rn && (reglist & (1 << r)) == 0 {
                reglist |= 1 << r;
                picked += 1;
            }
        }

        // Occasionally add PC (bit 15) to an IA LDM (probe_only). DB mode
        // is skipped because its memory layout gets complicated when PC
        // must come last in the natural register ordering. STMs never
        // include PC — M33 LDM.W is the only relevant case for probe_diff.
        let include_pc = is_load && !db && rng.coin(0.25);
        if include_pc {
            reglist |= 1 << 15;
        }

        let w = !is_load || (reglist & (1 << rn)) == 0; // writeback safe if rn not in list
        let reg_count = reglist.count_ones();

        let (hw0, hw1) = if is_load {
            enc_t32_ldm(rn, w, db, reglist)
        } else {
            enc_t32_stm(rn, w, db, reglist)
        };

        // For DB mode, base must be high enough to decrement.
        // For IA mode, base at 0 is fine.
        let base_offset: u32 = if db { reg_count * 4 } else { 0 };

        let mut reg_pre: Vec<(u8, u32)> = rand_gp_regs(rng);
        reg_pre.retain(|&(r, _)| r != rn as u8);
        reg_pre.push((rn as u8, base_offset));

        let (mem_pre, mem_check) = if is_load {
            let mut mp = Vec::new();
            // For DB: data at [base - N*4 .. base - 4]
            // For IA: data at [base .. base + (N-1)*4]
            // When PC is in the list (IA only), it's the highest-numbered
            // register and therefore the last word loaded. Write a
            // thumb-valid SRAM address at that slot so the post-state PC
            // points somewhere well-defined (we only single-step, so we
            // never actually fetch from the loaded address).
            for word in 0..reg_count {
                let off = if db {
                    base_offset - (reg_count - word) * 4
                } else {
                    base_offset + word * 4
                };
                let val = if include_pc && word == reg_count - 1 {
                    EMU_TEST_SLOT + 4 + 1
                } else {
                    rand_val(rng)
                };
                mp.extend(mem_pre_u32(off, val));
            }
            (mp, Vec::new())
        } else {
            let mut mc = Vec::new();
            for word in 0..reg_count {
                let off = if db {
                    base_offset - (reg_count - word) * 4
                } else {
                    base_offset + word * 4
                };
                mc.extend(mem_check_u32(off));
            }
            (Vec::new(), mc)
        };

        let tag = if is_load { "LDM" } else { "STM" };
        let dir = if db { "DB" } else { "IA" };
        t.push(TestCase {
            name: format!("FUZZ:T32_LDM:{i} {tag}{dir} R{rn}! list={reglist:#06x}"),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre,
            addr_regs: if reglist & (1 << rn) == 0 {
                vec![rn as u8]
            } else {
                vec![]
            },
            needs_bus: true,
            mem_pre,
            mem_check,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            probe_only: include_pc,
            ..TestCase::default()
        });
    }

    t
}

// ============================================================================
// FPU test generators
// ============================================================================

/// Helper: build an FPU binary-op test case (VADD, VSUB, VMUL, VDIV, etc.).
fn fpu_binop_tc(
    name: &str,
    enc_fn: fn(u16, u16, u16) -> (u16, u16),
    sd: u16,
    sn: u16,
    sm: u16,
    val_n: u32,
    val_m: u32,
) -> TestCase {
    let (hw0, hw1) = enc_fn(sd, sn, sm);
    TestCase {
        name: name.into(),
        opcode: hw0,
        hw1: Some(hw1),
        fpu_pre: vec![(sn as u8, val_n), (sm as u8, val_m)],
        fpu_check: vec![sd as u8],
        addr_regs: vec![12],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    }
}

/// Helper: build an FPU multiply-accumulate test (VMLA, VFMA, etc.).
/// The accumulator Sd is both input and output.
fn fpu_mac_tc(
    name: &str,
    enc_fn: fn(u16, u16, u16) -> (u16, u16),
    sd: u16,
    sn: u16,
    sm: u16,
    val_d: u32,
    val_n: u32,
    val_m: u32,
) -> TestCase {
    let (hw0, hw1) = enc_fn(sd, sn, sm);
    TestCase {
        name: name.into(),
        opcode: hw0,
        hw1: Some(hw1),
        fpu_pre: vec![(sd as u8, val_d), (sn as u8, val_n), (sm as u8, val_m)],
        fpu_check: vec![sd as u8],
        addr_regs: vec![12],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    }
}

/// Helper: build an FPU unary test (VMOV, VABS, VNEG, VSQRT).
fn fpu_unary_tc(
    name: &str,
    enc_fn: fn(u16, u16) -> (u16, u16),
    sd: u16,
    sm: u16,
    val_m: u32,
) -> TestCase {
    let (hw0, hw1) = enc_fn(sd, sm);
    TestCase {
        name: name.into(),
        opcode: hw0,
        hw1: Some(hw1),
        fpu_pre: vec![(sm as u8, val_m)],
        fpu_check: vec![sd as u8],
        addr_regs: vec![12],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    }
}

/// Well-known f32 bit patterns for targeted tests.
const F32_POS_ZERO: u32 = 0x0000_0000;
const F32_NEG_ZERO: u32 = 0x8000_0000;
const F32_POS_INF: u32 = 0x7F80_0000;
const F32_NEG_INF: u32 = 0xFF80_0000;
const F32_QNAN: u32 = 0x7FC0_0000;
const F32_SNAN: u32 = 0x7F80_0001;
const F32_DENORM: u32 = 0x0000_0001; // smallest positive denormal
const F32_ONE: u32 = 0x3F80_0000;
const F32_NEG_ONE: u32 = 0xBF80_0000;
const F32_TWO: u32 = 0x4000_0000;
const F32_THREE: u32 = 0x4040_0000;
const F32_FOUR: u32 = 0x4080_0000;
const F32_HALF: u32 = 0x3F00_0000;

/// Generate ~60 targeted FPU test cases.
pub fn gen_t32_fpu() -> Vec<TestCase> {
    use crate::{
        MASK_NO_FLAGS, enc_vabs, enc_vadd, enc_vcmp, enc_vcmp_zero, enc_vcvt_f32_s32,
        enc_vcvt_f32_u32, enc_vcvt_s32_f32, enc_vcvt_u32_f32, enc_vcvtr_s32_f32, enc_vdiv,
        enc_vfma, enc_vfms, enc_vfnma, enc_vfnms, enc_vmla, enc_vmls, enc_vmov_reg,
        enc_vmov_to_arm, enc_vmov_to_fpu, enc_vmrs, enc_vmul, enc_vneg, enc_vnmla, enc_vnmls,
        enc_vnmul, enc_vsqrt, enc_vsub,
    };

    let mut t = Vec::new();

    // --- Arithmetic: VADD ---
    t.push(fpu_binop_tc(
        "VADD 1.0+2.0",
        enc_vadd,
        0,
        1,
        2,
        F32_ONE,
        F32_TWO,
    ));
    t.push(fpu_binop_tc(
        "VADD +0+-0",
        enc_vadd,
        0,
        1,
        2,
        F32_POS_ZERO,
        F32_NEG_ZERO,
    ));
    t.push(fpu_binop_tc(
        "VADD +Inf+1",
        enc_vadd,
        0,
        1,
        2,
        F32_POS_INF,
        F32_ONE,
    ));
    t.push(fpu_binop_tc(
        "VADD +Inf+-Inf",
        enc_vadd,
        0,
        1,
        2,
        F32_POS_INF,
        F32_NEG_INF,
    ));
    t.push(fpu_binop_tc(
        "VADD NaN+1",
        enc_vadd,
        0,
        1,
        2,
        F32_QNAN,
        F32_ONE,
    ));
    t.push(fpu_binop_tc(
        "VADD denorm+denorm",
        enc_vadd,
        0,
        1,
        2,
        F32_DENORM,
        F32_DENORM,
    ));

    // --- Arithmetic: VSUB ---
    t.push(fpu_binop_tc(
        "VSUB 3.0-1.0",
        enc_vsub,
        4,
        5,
        6,
        F32_THREE,
        F32_ONE,
    ));
    t.push(fpu_binop_tc(
        "VSUB +0-+0",
        enc_vsub,
        4,
        5,
        6,
        F32_POS_ZERO,
        F32_POS_ZERO,
    ));
    t.push(fpu_binop_tc(
        "VSUB NaN-1",
        enc_vsub,
        4,
        5,
        6,
        F32_QNAN,
        F32_ONE,
    ));

    // --- Arithmetic: VMUL ---
    t.push(fpu_binop_tc(
        "VMUL 2.0*3.0",
        enc_vmul,
        0,
        1,
        2,
        F32_TWO,
        F32_THREE,
    ));
    t.push(fpu_binop_tc(
        "VMUL +Inf*0",
        enc_vmul,
        0,
        1,
        2,
        F32_POS_INF,
        F32_POS_ZERO,
    ));
    t.push(fpu_binop_tc(
        "VMUL -1*-1",
        enc_vmul,
        0,
        1,
        2,
        F32_NEG_ONE,
        F32_NEG_ONE,
    ));

    // --- Arithmetic: VNMUL ---
    t.push(fpu_binop_tc(
        "VNMUL 2.0*3.0",
        enc_vnmul,
        0,
        1,
        2,
        F32_TWO,
        F32_THREE,
    ));

    // --- Arithmetic: VDIV ---
    t.push(fpu_binop_tc(
        "VDIV 4.0/2.0",
        enc_vdiv,
        0,
        1,
        2,
        F32_FOUR,
        F32_TWO,
    ));
    t.push(fpu_binop_tc(
        "VDIV 1.0/0.0",
        enc_vdiv,
        0,
        1,
        2,
        F32_ONE,
        F32_POS_ZERO,
    ));
    t.push(fpu_binop_tc(
        "VDIV 0.0/0.0",
        enc_vdiv,
        0,
        1,
        2,
        F32_POS_ZERO,
        F32_POS_ZERO,
    ));

    // --- Multiply-accumulate: VMLA (Sd = Sd + Sn*Sm) ---
    t.push(fpu_mac_tc(
        "VMLA 1+2*3",
        enc_vmla,
        0,
        1,
        2,
        F32_ONE,
        F32_TWO,
        F32_THREE,
    ));
    t.push(fpu_mac_tc(
        "VMLA 0+Inf*0",
        enc_vmla,
        0,
        1,
        2,
        F32_POS_ZERO,
        F32_POS_INF,
        F32_POS_ZERO,
    ));

    // --- Multiply-accumulate: VMLS (Sd = Sd - Sn*Sm) ---
    t.push(fpu_mac_tc(
        "VMLS 4-2*1",
        enc_vmls,
        0,
        1,
        2,
        F32_FOUR,
        F32_TWO,
        F32_ONE,
    ));

    // --- VNMLA (Sd = -(Sd + Sn*Sm)) ---
    t.push(fpu_mac_tc(
        "VNMLA 1+2*3",
        enc_vnmla,
        0,
        1,
        2,
        F32_ONE,
        F32_TWO,
        F32_THREE,
    ));

    // --- VNMLS (Sd = -Sd + Sn*Sm) ---
    t.push(fpu_mac_tc(
        "VNMLS 1+2*3",
        enc_vnmls,
        0,
        1,
        2,
        F32_ONE,
        F32_TWO,
        F32_THREE,
    ));

    // --- VFMA (fused) ---
    t.push(fpu_mac_tc(
        "VFMA 1+2*3",
        enc_vfma,
        0,
        1,
        2,
        F32_ONE,
        F32_TWO,
        F32_THREE,
    ));
    t.push(fpu_mac_tc(
        "VFMA NaN+1*1",
        enc_vfma,
        0,
        1,
        2,
        F32_QNAN,
        F32_ONE,
        F32_ONE,
    ));

    // --- VFMS (fused) ---
    t.push(fpu_mac_tc(
        "VFMS 4-2*1",
        enc_vfms,
        0,
        1,
        2,
        F32_FOUR,
        F32_TWO,
        F32_ONE,
    ));

    // --- VFNMA ---
    t.push(fpu_mac_tc(
        "VFNMA 1+2*3",
        enc_vfnma,
        0,
        1,
        2,
        F32_ONE,
        F32_TWO,
        F32_THREE,
    ));

    // --- VFNMS ---
    t.push(fpu_mac_tc(
        "VFNMS 1+2*3",
        enc_vfnms,
        0,
        1,
        2,
        F32_ONE,
        F32_TWO,
        F32_THREE,
    ));

    // --- Unary: VMOV.F32 ---
    t.push(fpu_unary_tc(
        "VMOV.F32 S0,S1 (3.0)",
        enc_vmov_reg,
        0,
        1,
        F32_THREE,
    ));
    t.push(fpu_unary_tc(
        "VMOV.F32 S0,S1 (NaN)",
        enc_vmov_reg,
        0,
        1,
        F32_QNAN,
    ));

    // --- Unary: VABS ---
    t.push(fpu_unary_tc("VABS -1.0", enc_vabs, 0, 1, F32_NEG_ONE));
    t.push(fpu_unary_tc("VABS +1.0", enc_vabs, 0, 1, F32_ONE));
    t.push(fpu_unary_tc("VABS -0.0", enc_vabs, 0, 1, F32_NEG_ZERO));

    // --- Unary: VNEG ---
    t.push(fpu_unary_tc("VNEG 1.0", enc_vneg, 0, 1, F32_ONE));
    t.push(fpu_unary_tc("VNEG -1.0", enc_vneg, 0, 1, F32_NEG_ONE));
    t.push(fpu_unary_tc("VNEG +0", enc_vneg, 0, 1, F32_POS_ZERO));

    // --- Unary: VSQRT ---
    t.push(fpu_unary_tc("VSQRT 4.0", enc_vsqrt, 0, 1, F32_FOUR));
    t.push(fpu_unary_tc("VSQRT 1.0", enc_vsqrt, 0, 1, F32_ONE));
    t.push(fpu_unary_tc("VSQRT -1.0", enc_vsqrt, 0, 1, F32_NEG_ONE));
    t.push(fpu_unary_tc("VSQRT +0", enc_vsqrt, 0, 1, F32_POS_ZERO));

    // --- Compare: VCMP with FPSCR flag check ---
    // VCMP sets FPSCR NZCV flags. We emit: VCMP Sd,Sm + VMRS APSR,FPSCR is NOT
    // needed here — the epilogue reads FPSCR directly via VMRS R11,FPSCR.
    {
        // Equal: 1.0 == 1.0 → Z=1, C=1
        let (hw0, hw1) = enc_vcmp(0, 1);
        t.push(TestCase {
            name: "VCMP 1.0==1.0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(0, F32_ONE), (1, F32_ONE)],
            fpu_check: vec![],
            fpscr_mask: 0xF000_0000,
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // Less: 1.0 < 2.0 → N=1
        let (hw0, hw1) = enc_vcmp(0, 1);
        t.push(TestCase {
            name: "VCMP 1.0<2.0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(0, F32_ONE), (1, F32_TWO)],
            fpu_check: vec![],
            fpscr_mask: 0xF000_0000,
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // Greater: 2.0 > 1.0 → C=1
        let (hw0, hw1) = enc_vcmp(0, 1);
        t.push(TestCase {
            name: "VCMP 2.0>1.0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(0, F32_TWO), (1, F32_ONE)],
            fpu_check: vec![],
            fpscr_mask: 0xF000_0000,
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // Unordered: NaN vs 1.0 → C=1,V=1
        let (hw0, hw1) = enc_vcmp(0, 1);
        t.push(TestCase {
            name: "VCMP NaN vs 1.0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(0, F32_QNAN), (1, F32_ONE)],
            fpu_check: vec![],
            fpscr_mask: 0xF000_0000,
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- VCMP against zero ---
    {
        let (hw0, hw1) = enc_vcmp_zero(0);
        t.push(TestCase {
            name: "VCMP 0.0 vs #0.0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(0, F32_POS_ZERO)],
            fpu_check: vec![],
            fpscr_mask: 0xF000_0000,
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        let (hw0, hw1) = enc_vcmp_zero(0);
        t.push(TestCase {
            name: "VCMP -0.0 vs #0.0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(0, F32_NEG_ZERO)],
            fpu_check: vec![],
            fpscr_mask: 0xF000_0000,
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Convert: VCVT int -> float ---
    {
        // VCVT.F32.S32: signed 42 -> 42.0
        let (hw0, hw1) = enc_vcvt_f32_s32(0, 1);
        t.push(TestCase {
            name: "VCVT.F32.S32 42".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, 42u32)],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // VCVT.F32.S32: -1 (0xFFFFFFFF) -> -1.0
        let (hw0, hw1) = enc_vcvt_f32_s32(0, 1);
        t.push(TestCase {
            name: "VCVT.F32.S32 -1".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, 0xFFFF_FFFF)],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // VCVT.F32.U32: unsigned 42 -> 42.0
        let (hw0, hw1) = enc_vcvt_f32_u32(0, 1);
        t.push(TestCase {
            name: "VCVT.F32.U32 42".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, 42u32)],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Convert: float -> int ---
    {
        // VCVT.S32.F32: 3.7 -> 3 (round toward zero)
        let (hw0, hw1) = enc_vcvt_s32_f32(0, 1);
        t.push(TestCase {
            name: "VCVT.S32.F32 3.7".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, 3.7f32.to_bits())],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // VCVT.S32.F32: NaN -> 0
        let (hw0, hw1) = enc_vcvt_s32_f32(0, 1);
        t.push(TestCase {
            name: "VCVT.S32.F32 NaN".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, F32_QNAN)],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // VCVT.U32.F32: -1.0 -> 0 (negative to unsigned saturates to 0)
        let (hw0, hw1) = enc_vcvt_u32_f32(0, 1);
        t.push(TestCase {
            name: "VCVT.U32.F32 -1.0".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, F32_NEG_ONE)],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }
    {
        // VCVTR.S32.F32: 3.7 -> 4 (round per FPSCR, default = round-nearest)
        let (hw0, hw1) = enc_vcvtr_s32_f32(0, 1);
        t.push(TestCase {
            name: "VCVTR.S32.F32 3.7".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, 3.7f32.to_bits())],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Transfer: VMOV arm -> fpu ---
    {
        // VMOV S0, R3 — put 0xDEAD_BEEF in R3, read S0
        let (hw0, hw1) = enc_vmov_to_fpu(0, 3);
        t.push(TestCase {
            name: "VMOV S0,R3".into(),
            opcode: hw0,
            hw1: Some(hw1),
            reg_pre: vec![(3, 0xDEAD_BEEF)],
            fpu_pre: vec![],
            fpu_check: vec![0],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Transfer: VMOV fpu -> arm ---
    {
        // VMOV R3, S1 — put 0xCAFE_BABE in S1, read R3
        let (hw0, hw1) = enc_vmov_to_arm(3, 1);
        t.push(TestCase {
            name: "VMOV R3,S1".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(1, 0xCAFE_BABE)],
            fpu_check: vec![],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- VMRS R0, FPSCR ---
    {
        let (hw0, hw1) = enc_vmrs(0);
        t.push(TestCase {
            name: "VMRS R0,FPSCR".into(),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![],
            fpu_check: vec![],
            // VMRS reads FPSCR into an ARM register.
            // With default FPSCR=0, R0 should be 0.
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- High S-register test: VADD S31, S30, S29 ---
    t.push(fpu_binop_tc(
        "VADD S31,S30,S29",
        enc_vadd,
        31,
        30,
        29,
        F32_ONE,
        F32_TWO,
    ));

    // --- VSUB with denormals ---
    t.push(fpu_binop_tc(
        "VSUB denorm-denorm",
        enc_vsub,
        0,
        1,
        2,
        F32_DENORM,
        F32_DENORM,
    ));

    // --- VMUL with Inf ---
    t.push(fpu_binop_tc(
        "VMUL +Inf*1",
        enc_vmul,
        0,
        1,
        2,
        F32_POS_INF,
        F32_ONE,
    ));

    // --- VDIV NaN ---
    t.push(fpu_binop_tc(
        "VDIV NaN/1",
        enc_vdiv,
        0,
        1,
        2,
        F32_QNAN,
        F32_ONE,
    ));

    // --- VADD with signalling NaN ---
    t.push(fpu_binop_tc(
        "VADD sNaN+1",
        enc_vadd,
        0,
        1,
        2,
        F32_SNAN,
        F32_ONE,
    ));

    // --- VMUL with 0.5 ---
    t.push(fpu_binop_tc(
        "VMUL 0.5*2.0",
        enc_vmul,
        0,
        1,
        2,
        F32_HALF,
        F32_TWO,
    ));

    t
}

/// Generate `count` random FPU fuzz tests per sub-class.
///
/// 6 sub-classes: arithmetic, multiply-accumulate, unary, convert, compare, vmov transfer.
/// Returns tests. FPU tests use the multi-step path (prelude/epilogue).
pub fn generate_fuzz_fpu(count: usize, rng: &mut StdRng) -> Vec<TestCase> {
    use crate::{
        MASK_NO_FLAGS, enc_vabs, enc_vadd, enc_vcmp, enc_vcvt_f32_s32, enc_vcvt_f32_u32,
        enc_vcvt_s32_f32, enc_vcvt_u32_f32, enc_vdiv, enc_vfma, enc_vmla, enc_vmov_reg,
        enc_vmov_to_arm, enc_vmov_to_fpu, enc_vmul, enc_vneg, enc_vsqrt, enc_vsub,
    };

    let mut t = Vec::new();

    // Random S-register 0-31
    let rand_sreg = |rng: &mut StdRng| -> u16 { rng.range(0..32) };

    // --- Arithmetic (VADD, VSUB, VMUL, VDIV) ---
    let arith_ops: [fn(u16, u16, u16) -> (u16, u16); 4] = [enc_vadd, enc_vsub, enc_vmul, enc_vdiv];
    let arith_names = ["VADD", "VSUB", "VMUL", "VDIV"];

    for i in 0..count {
        let op_idx = rng.range(0..4usize);
        let sd = rand_sreg(rng);
        let sn = rand_sreg(rng);
        let sm = rand_sreg(rng);
        let val_n = biased_f32(rng);
        let val_m = biased_f32(rng);
        let (hw0, hw1) = arith_ops[op_idx](sd, sn, sm);

        // Build fpu_pre, handling the case where sn == sm
        let mut fpu_pre = vec![(sn as u8, val_n)];
        if sm != sn {
            fpu_pre.push((sm as u8, val_m));
        }

        t.push(TestCase {
            name: format!(
                "FUZZ:FPU_ARITH:{i} {} S{sd},S{sn},S{sm}",
                arith_names[op_idx]
            ),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre,
            fpu_check: vec![sd as u8],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Multiply-accumulate (VMLA, VFMA) ---
    let mac_ops: [fn(u16, u16, u16) -> (u16, u16); 2] = [enc_vmla, enc_vfma];
    let mac_names = ["VMLA", "VFMA"];

    for i in 0..count {
        let op_idx = rng.range(0..2usize);
        let sd = rand_sreg(rng);
        let sn = rand_sreg(rng);
        let sm = rand_sreg(rng);
        let val_d = biased_f32(rng);
        let val_n = biased_f32(rng);
        let val_m = biased_f32(rng);
        let (hw0, hw1) = mac_ops[op_idx](sd, sn, sm);

        // Build fpu_pre — sd is also an input
        let mut fpu_pre = vec![(sd as u8, val_d)];
        if sn != sd {
            fpu_pre.push((sn as u8, val_n));
        }
        if sm != sd && sm != sn {
            fpu_pre.push((sm as u8, val_m));
        }

        t.push(TestCase {
            name: format!("FUZZ:FPU_MAC:{i} {} S{sd},S{sn},S{sm}", mac_names[op_idx]),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre,
            fpu_check: vec![sd as u8],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Unary (VMOV.F32, VABS, VNEG, VSQRT) ---
    let unary_ops: [fn(u16, u16) -> (u16, u16); 4] = [enc_vmov_reg, enc_vabs, enc_vneg, enc_vsqrt];
    let unary_names = ["VMOV", "VABS", "VNEG", "VSQRT"];

    for i in 0..count {
        let op_idx = rng.range(0..4usize);
        let sd = rand_sreg(rng);
        let sm = rand_sreg(rng);
        let val_m = biased_f32(rng);
        let (hw0, hw1) = unary_ops[op_idx](sd, sm);
        t.push(TestCase {
            name: format!("FUZZ:FPU_UNARY:{i} {} S{sd},S{sm}", unary_names[op_idx]),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre: vec![(sm as u8, val_m)],
            fpu_check: vec![sd as u8],
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Convert (int<->float) ---
    let cvt_to_float: [fn(u16, u16) -> (u16, u16); 2] = [enc_vcvt_f32_s32, enc_vcvt_f32_u32];
    let cvt_from_float: [fn(u16, u16) -> (u16, u16); 2] = [enc_vcvt_s32_f32, enc_vcvt_u32_f32];
    let cvt_to_names = ["VCVT.F32.S32", "VCVT.F32.U32"];
    let cvt_from_names = ["VCVT.S32.F32", "VCVT.U32.F32"];

    for i in 0..count {
        let sd = rand_sreg(rng);
        let sm = rand_sreg(rng);

        if rng.coin(0.5) {
            // int -> float: input is a random integer bit pattern
            let op_idx = rng.range(0..2usize);
            let val: u32 = rng.random();
            let (hw0, hw1) = cvt_to_float[op_idx](sd, sm);
            t.push(TestCase {
                name: format!("FUZZ:FPU_CVT:{i} {} S{sd},S{sm}", cvt_to_names[op_idx]),
                opcode: hw0,
                hw1: Some(hw1),
                fpu_pre: vec![(sm as u8, val)],
                fpu_check: vec![sd as u8],
                addr_regs: vec![12],
                xpsr_mask: MASK_NO_FLAGS,
                ..TestCase::default()
            });
        } else {
            // float -> int: use biased float patterns
            let op_idx = rng.range(0..2usize);
            let val = biased_f32(rng);
            let (hw0, hw1) = cvt_from_float[op_idx](sd, sm);
            t.push(TestCase {
                name: format!("FUZZ:FPU_CVT:{i} {} S{sd},S{sm}", cvt_from_names[op_idx]),
                opcode: hw0,
                hw1: Some(hw1),
                fpu_pre: vec![(sm as u8, val)],
                fpu_check: vec![sd as u8],
                addr_regs: vec![12],
                xpsr_mask: MASK_NO_FLAGS,
                ..TestCase::default()
            });
        }
    }

    // --- Compare (VCMP) — check FPSCR flags ---
    for i in 0..count {
        let s0 = rand_sreg(rng);
        let s1 = rand_sreg(rng);
        let val0 = biased_f32(rng);
        let val1 = biased_f32(rng);
        let (hw0, hw1) = enc_vcmp(s0, s1);

        let mut fpu_pre = vec![(s0 as u8, val0)];
        if s1 != s0 {
            fpu_pre.push((s1 as u8, val1));
        }

        t.push(TestCase {
            name: format!("FUZZ:FPU_CMP:{i} VCMP S{s0},S{s1}"),
            opcode: hw0,
            hw1: Some(hw1),
            fpu_pre,
            fpu_check: vec![],
            fpscr_mask: 0xF000_0000,
            addr_regs: vec![12],
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- VMOV arm↔fpu transfers ---
    for i in 0..count {
        let sn = rand_sreg(rng);
        let rt: u16 = rng.range(0..11); // R0-R10, exclude R11/R12

        if rng.coin(0.5) {
            // ARM → FPU: VMOV Sn, Rt
            let val: u32 = rng.random();
            let (hw0, hw1) = enc_vmov_to_fpu(sn, rt);
            t.push(TestCase {
                name: format!("FUZZ:FPU_VMOV:{i} VMOV S{sn},R{rt}"),
                opcode: hw0,
                hw1: Some(hw1),
                reg_pre: vec![(rt as u8, val)],
                fpu_check: vec![sn as u8],
                addr_regs: vec![12],
                xpsr_mask: MASK_NO_FLAGS,
                fpscr_mask: 0,
                ..TestCase::default()
            });
        } else {
            // FPU → ARM: VMOV Rt, Sn
            let val: u32 = rng.random();
            let (hw0, hw1) = enc_vmov_to_arm(rt, sn);
            t.push(TestCase {
                name: format!("FUZZ:FPU_VMOV:{i} VMOV R{rt},S{sn}"),
                opcode: hw0,
                hw1: Some(hw1),
                fpu_pre: vec![(sn as u8, val)],
                addr_regs: vec![12],
                xpsr_mask: MASK_NO_FLAGS,
                fpscr_mask: 0,
                ..TestCase::default()
            });
        }
    }

    t
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
        let i = ((hw0 >> 10) & 1) as u32;
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
        assert_ne!(hw1 & (1 << 9), 0); // U
        assert_eq!(hw1 & (1 << 8), 0); // W=false
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

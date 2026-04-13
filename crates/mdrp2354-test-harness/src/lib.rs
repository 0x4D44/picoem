// QEMU differential test harness — foundation types and test generation.
//
// Validates Thumb-2 instruction semantics by executing identical instructions
// in both QEMU (Cortex-M33 model) and our emulator, then diffing state.

pub mod gdb_client;
pub mod thumb32_gen;

use rand::rngs::StdRng;
use rand::SeedableRng;

/// Extension trait to call Rng::gen() without hitting the `gen` keyword reservation.
trait RngExt {
    fn random<T>(&mut self) -> T
    where
        rand::distributions::Standard: rand::distributions::Distribution<T>;
    fn range<T, R>(&mut self, range: R) -> T
    where
        T: rand::distributions::uniform::SampleUniform,
        R: rand::distributions::uniform::SampleRange<T>;
    fn coin(&mut self, p: f64) -> bool;
}

impl RngExt for StdRng {
    fn random<T>(&mut self) -> T
    where
        rand::distributions::Standard: rand::distributions::Distribution<T>,
    {
        <Self as rand::Rng>::r#gen(self)
    }
    fn range<T, R>(&mut self, range: R) -> T
    where
        T: rand::distributions::uniform::SampleUniform,
        R: rand::distributions::uniform::SampleRange<T>,
    {
        <Self as rand::Rng>::gen_range(self, range)
    }
    fn coin(&mut self, p: f64) -> bool {
        <Self as rand::Rng>::gen_bool(self, p)
    }
}

// Re-export emulator types the harness needs.
pub use mdrp2354::{Bus, CortexM33};

// ============================================================================
// Address constants — QEMU side (MPS2-AN505 ssram-0)
// ============================================================================

/// QEMU: instruction slot in ssram-0.
pub const QEMU_TEST_SLOT: u32 = 0x0000_0100;
/// QEMU: stack pointer for push/pop/load/store tests.
pub const QEMU_TEST_STACK: u32 = 0x0004_0000;
/// QEMU: scratch SRAM for load/store data.
pub const QEMU_TEST_SCRATCH: u32 = 0x0000_0200;

// ============================================================================
// Address constants — Emulator side (our SRAM address space)
// ============================================================================

/// Emulator: instruction slot in SRAM.
pub const EMU_TEST_SLOT: u32 = 0x2000_0100;
/// Emulator: stack pointer.
pub const EMU_TEST_STACK: u32 = 0x2004_0000;
/// Emulator: scratch SRAM.
pub const EMU_TEST_SCRATCH: u32 = 0x2000_0200;

/// Scratch area size in bytes. Covers LDRD/STRD max offset (imm8×4 = 1020).
pub const SCRATCH_SIZE: u32 = 1024;

// ============================================================================
// GDB register indices (stable across QEMU >= 7.0)
// ============================================================================

/// R0-R12 are indices 0-12.
pub const REG_R0: u8 = 0;
pub const REG_SP: u8 = 13;
pub const REG_LR: u8 = 14;
pub const REG_PC: u8 = 15;
/// Indices 16-24 are legacy FPA (return E14 on QEMU 10.2). xPSR is at index 25.
/// Note: QEMU's M-profile GDB stub omits EPSR.T (bit 24) from xPSR reads.
pub const REG_XPSR: u8 = 25;

// ============================================================================
// xPSR comparison masks
// ============================================================================

/// N, Z, C, V, Q — all condition flags.
pub const MASK_ALL_FLAGS: u32 = 0xF800_0000;
/// N, Z only — for MUL where C and V are UNPREDICTABLE.
pub const MASK_NZ_ONLY: u32 = 0xC000_0000;
/// No flags — for MOV/ADD (high register) which don't update flags.
pub const MASK_NO_FLAGS: u32 = 0x0000_0000;
/// N, Z, C, V, Q + GE[3:0] — for DSP parallel add/sub and SEL.
pub const MASK_ALL_FLAGS_GE: u32 = 0xF80F_0000;
/// Q flag only — for saturation instructions (SSAT, USAT, QADD, etc.).
pub const MASK_Q_ONLY: u32 = 0x0800_0000;

// ============================================================================
// Test case model
// ============================================================================

/// A single differential test case: one instruction with preconditions.
pub struct TestCase {
    /// Human-readable name (e.g., "ADDS R0, R1, R2 (overflow)").
    pub name: String,
    /// Instruction opcode (16-bit for Phase A).
    pub opcode: u16,
    /// Register preconditions: (index, value). Unset registers default to 0.
    pub reg_pre: Vec<(u8, u32)>,
    /// xPSR precondition. Default: 0x01000000 (T bit set, flags clear).
    pub xpsr_pre: u32,
    /// Whether this instruction accesses memory (use execute_one_with_bus).
    pub needs_bus: bool,
    /// Registers whose values are addresses (offsets from scratch base).
    /// The runner translates these by adding the per-side TEST_SCRATCH base.
    pub addr_regs: Vec<u8>,
    /// Memory preconditions as offsets from scratch area.
    /// Written to QEMU_TEST_SCRATCH+offset and EMU_TEST_SCRATCH+offset.
    pub mem_pre: Vec<(u32, u8)>,
    /// Memory offsets to compare after execution.
    pub mem_check: Vec<u32>,
    /// xPSR flag mask for comparison. Default: MASK_ALL_FLAGS.
    pub xpsr_mask: u32,
    /// Second halfword for Thumb-32 instructions. None = Thumb-16.
    pub hw1: Option<u16>,
    /// BL sets LR to a per-side absolute return address.
    /// When true, compare LR as delta from test slot.
    pub modifies_lr: bool,
}

impl Default for TestCase {
    fn default() -> Self {
        Self {
            name: String::new(),
            opcode: 0,
            reg_pre: Vec::new(),
            xpsr_pre: 0x0100_0000, // T bit set, flags clear
            needs_bus: false,
            addr_regs: Vec::new(),
            mem_pre: Vec::new(),
            mem_check: Vec::new(),
            xpsr_mask: MASK_ALL_FLAGS,
            hw1: None,
            modifies_lr: false,
        }
    }
}

// ============================================================================
// Encoding helpers
// ============================================================================

/// Encode LSLS Rd, Rm, #imm5: 00000_imm5_Rm_Rd
fn enc_lsl_imm(rd: u16, rm: u16, imm5: u16) -> u16 {
    (imm5 << 6) | (rm << 3) | rd
}

/// Encode LSRS Rd, Rm, #imm5: 00001_imm5_Rm_Rd
fn enc_lsr_imm(rd: u16, rm: u16, imm5: u16) -> u16 {
    (1 << 11) | (imm5 << 6) | (rm << 3) | rd
}

/// Encode ASRS Rd, Rm, #imm5: 00010_imm5_Rm_Rd
fn enc_asr_imm(rd: u16, rm: u16, imm5: u16) -> u16 {
    (2 << 11) | (imm5 << 6) | (rm << 3) | rd
}

/// Encode ADDS Rd, Rn, Rm: 0001100_Rm_Rn_Rd
fn enc_adds_reg(rd: u16, rn: u16, rm: u16) -> u16 {
    (0b0001100 << 9) | (rm << 6) | (rn << 3) | rd
}

/// Encode SUBS Rd, Rn, Rm: 0001101_Rm_Rn_Rd
fn enc_subs_reg(rd: u16, rn: u16, rm: u16) -> u16 {
    (0b0001101 << 9) | (rm << 6) | (rn << 3) | rd
}

/// Encode ADDS Rd, Rn, #imm3: 0001110_imm3_Rn_Rd
fn enc_adds_imm3(rd: u16, rn: u16, imm3: u16) -> u16 {
    (0b0001110 << 9) | (imm3 << 6) | (rn << 3) | rd
}

/// Encode SUBS Rd, Rn, #imm3: 0001111_imm3_Rn_Rd
fn enc_subs_imm3(rd: u16, rn: u16, imm3: u16) -> u16 {
    (0b0001111 << 9) | (imm3 << 6) | (rn << 3) | rd
}

/// Encode MOVS Rd, #imm8: 00100_Rd_imm8
fn enc_movs_imm(rd: u16, imm8: u16) -> u16 {
    (0b00100 << 11) | (rd << 8) | (imm8 & 0xFF)
}

/// Encode CMP Rn, #imm8: 00101_Rn_imm8
fn enc_cmp_imm(rn: u16, imm8: u16) -> u16 {
    (0b00101 << 11) | (rn << 8) | (imm8 & 0xFF)
}

/// Encode ADDS Rdn, #imm8: 00110_Rdn_imm8
fn enc_adds_imm8(rdn: u16, imm8: u16) -> u16 {
    (0b00110 << 11) | (rdn << 8) | (imm8 & 0xFF)
}

/// Encode SUBS Rdn, #imm8: 00111_Rdn_imm8
fn enc_subs_imm8(rdn: u16, imm8: u16) -> u16 {
    (0b00111 << 11) | (rdn << 8) | (imm8 & 0xFF)
}

/// Encode data processing (register): 010000_op_Rm_Rdn
fn enc_data_proc(op: u16, rm: u16, rdn: u16) -> u16 {
    (0b010000 << 10) | (op << 6) | (rm << 3) | rdn
}

/// Encode ADD Rd, Rm (high registers): 01000100_D_Rm_Rd
/// D is bit 7 of the destination. rd is the full 4-bit index.
fn enc_add_high(rd: u16, rm: u16) -> u16 {
    let d_hi = (rd >> 3) & 1;
    let d_lo = rd & 7;
    (0b01000100 << 8) | (d_hi << 7) | (rm << 3) | d_lo
}

/// Encode MOV Rd, Rm (high registers): 01000110_D_Rm_Rd
fn enc_mov_high(rd: u16, rm: u16) -> u16 {
    let d_hi = (rd >> 3) & 1;
    let d_lo = rd & 7;
    (0b01000110 << 8) | (d_hi << 7) | (rm << 3) | d_lo
}

/// Encode BX Rm: 01000111_0_Rm_000
fn enc_bx(rm: u16) -> u16 {
    (0b010001110 << 7) | (rm << 3)
}

/// Encode load/store register offset: 0101_opc_Rm_Rn_Rt
fn enc_ls_reg(opc: u16, rm: u16, rn: u16, rt: u16) -> u16 {
    (0b0101 << 12) | (opc << 9) | (rm << 6) | (rn << 3) | rt
}

/// Encode STR Rt, [Rn, #imm5*4]: 01100_imm5_Rn_Rt
fn enc_str_imm(rt: u16, rn: u16, imm5: u16) -> u16 {
    (0b01100 << 11) | (imm5 << 6) | (rn << 3) | rt
}

/// Encode LDR Rt, [Rn, #imm5*4]: 01101_imm5_Rn_Rt
fn enc_ldr_imm(rt: u16, rn: u16, imm5: u16) -> u16 {
    (0b01101 << 11) | (imm5 << 6) | (rn << 3) | rt
}

/// Encode STRB Rt, [Rn, #imm5]: 01110_imm5_Rn_Rt
fn enc_strb_imm(rt: u16, rn: u16, imm5: u16) -> u16 {
    (0b01110 << 11) | (imm5 << 6) | (rn << 3) | rt
}

/// Encode LDRB Rt, [Rn, #imm5]: 01111_imm5_Rn_Rt
fn enc_ldrb_imm(rt: u16, rn: u16, imm5: u16) -> u16 {
    (0b01111 << 11) | (imm5 << 6) | (rn << 3) | rt
}

/// Encode STRH Rt, [Rn, #imm5*2]: 10000_imm5_Rn_Rt
fn enc_strh_imm(rt: u16, rn: u16, imm5: u16) -> u16 {
    (0b10000 << 11) | (imm5 << 6) | (rn << 3) | rt
}

/// Encode LDRH Rt, [Rn, #imm5*2]: 10001_imm5_Rn_Rt
fn enc_ldrh_imm(rt: u16, rn: u16, imm5: u16) -> u16 {
    (0b10001 << 11) | (imm5 << 6) | (rn << 3) | rt
}

/// Encode STR Rt, [SP, #imm8*4]: 10010_Rt_imm8
fn enc_str_sp(rt: u16, imm8: u16) -> u16 {
    (0b10010 << 11) | (rt << 8) | (imm8 & 0xFF)
}

/// Encode LDR Rt, [SP, #imm8*4]: 10011_Rt_imm8
fn enc_ldr_sp(rt: u16, imm8: u16) -> u16 {
    (0b10011 << 11) | (rt << 8) | (imm8 & 0xFF)
}

/// Encode ADR Rd, #imm8*4: 10100_Rd_imm8
#[allow(dead_code)] // Differential tests skipped (address-space-dependent), but encoder is correct.
fn enc_adr(rd: u16, imm8: u16) -> u16 {
    (0b10100 << 11) | (rd << 8) | (imm8 & 0xFF)
}

/// Encode ADD Rd, SP, #imm8*4: 10101_Rd_imm8
#[allow(dead_code)] // Differential tests skipped (address-space-dependent), but encoder is correct.
fn enc_add_sp_imm(rd: u16, imm8: u16) -> u16 {
    (0b10101 << 11) | (rd << 8) | (imm8 & 0xFF)
}

/// Encode ADD SP, SP, #imm7*4: 10110000_0_imm7
fn enc_add_sp_sp(imm7: u16) -> u16 {
    (0b10110000 << 8) | (imm7 & 0x7F)
}

/// Encode SUB SP, SP, #imm7*4: 10110000_1_imm7
fn enc_sub_sp_sp(imm7: u16) -> u16 {
    (0b10110000 << 8) | (1 << 7) | (imm7 & 0x7F)
}

/// Encode SXTH Rd, Rm: 10110010_00_Rm_Rd
fn enc_sxth(rd: u16, rm: u16) -> u16 {
    (0b10110010_00 << 6) | (rm << 3) | rd
}

/// Encode SXTB Rd, Rm: 10110010_01_Rm_Rd
fn enc_sxtb(rd: u16, rm: u16) -> u16 {
    (0b10110010_01 << 6) | (rm << 3) | rd
}

/// Encode UXTH Rd, Rm: 10110010_10_Rm_Rd
fn enc_uxth(rd: u16, rm: u16) -> u16 {
    (0b10110010_10 << 6) | (rm << 3) | rd
}

/// Encode UXTB Rd, Rm: 10110010_11_Rm_Rd
fn enc_uxtb(rd: u16, rm: u16) -> u16 {
    (0b10110010_11 << 6) | (rm << 3) | rd
}

/// Encode REV Rd, Rm: 10111010_00_Rm_Rd
fn enc_rev(rd: u16, rm: u16) -> u16 {
    (0b10111010_00 << 6) | (rm << 3) | rd
}

/// Encode REV16 Rd, Rm: 10111010_01_Rm_Rd
fn enc_rev16(rd: u16, rm: u16) -> u16 {
    (0b10111010_01 << 6) | (rm << 3) | rd
}

/// Encode REVSH Rd, Rm: 10111010_11_Rm_Rd
fn enc_revsh(rd: u16, rm: u16) -> u16 {
    (0b10111010_11 << 6) | (rm << 3) | rd
}

/// Encode PUSH {reglist}: 1011_0100_reglist8.  bit 8 = include LR.
fn enc_push(reglist8: u16, lr: bool) -> u16 {
    (0b1011_0100 << 8) | (if lr { 1 << 8 } else { 0 }) | (reglist8 & 0xFF)
}

/// Encode POP {reglist}: 1011_1100_reglist8.  bit 8 = include PC.
fn enc_pop(reglist8: u16, pc: bool) -> u16 {
    (0b1011_1100 << 8) | (if pc { 1 << 8 } else { 0 }) | (reglist8 & 0xFF)
}

/// Encode STM Rn!, {reglist}: 11000_Rn_reglist8
fn enc_stm(rn: u16, reglist8: u16) -> u16 {
    (0b11000 << 11) | (rn << 8) | (reglist8 & 0xFF)
}

/// Encode LDM Rn!, {reglist}: 11001_Rn_reglist8
fn enc_ldm(rn: u16, reglist8: u16) -> u16 {
    (0b11001 << 11) | (rn << 8) | (reglist8 & 0xFF)
}

/// Encode B<cond> offset: 1101_cond_imm8.
/// offset is in bytes, must be even, sign-extended from 9 bits.
fn enc_branch_cond(cond: u16, offset_bytes: i16) -> u16 {
    let imm8 = ((offset_bytes >> 1) as u16) & 0xFF;
    (0b1101 << 12) | (cond << 8) | imm8
}

/// Encode B (unconditional) offset: 11100_imm11.
/// offset is in bytes, must be even, sign-extended from 12 bits.
fn enc_branch_uncond(offset_bytes: i32) -> u16 {
    let imm11 = ((offset_bytes >> 1) as u16) & 0x7FF;
    (0b11100 << 11) | imm11
}

/// Write a u32 value as 4 little-endian bytes into mem_pre entries.
fn mem_pre_u32(offset: u32, val: u32) -> Vec<(u32, u8)> {
    vec![
        (offset, (val & 0xFF) as u8),
        (offset + 1, ((val >> 8) & 0xFF) as u8),
        (offset + 2, ((val >> 16) & 0xFF) as u8),
        (offset + 3, ((val >> 24) & 0xFF) as u8),
    ]
}

/// Write a u16 value as 2 little-endian bytes into mem_pre entries.
fn mem_pre_u16(offset: u32, val: u16) -> Vec<(u32, u8)> {
    vec![
        (offset, (val & 0xFF) as u8),
        (offset + 1, ((val >> 8) & 0xFF) as u8),
    ]
}

/// Byte offsets for a 32-bit word check.
fn mem_check_u32(offset: u32) -> Vec<u32> {
    vec![offset, offset + 1, offset + 2, offset + 3]
}

/// Byte offsets for a 16-bit halfword check.
fn mem_check_u16(offset: u32) -> Vec<u32> {
    vec![offset, offset + 1]
}

// ============================================================================
// Test generators
// ============================================================================

/// LSL, LSR, ASR (immediate). Encoding: 000xx. ~30 tests.
fn gen_shift_imm() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- LSLS Rd, Rm, #imm5 ---

    // Register field extraction
    t.push(TestCase {
        name: "LSLS R0, R1, #3".into(),
        opcode: enc_lsl_imm(0, 1, 3),
        reg_pre: vec![(1, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R5, R3, #7".into(),
        opcode: enc_lsl_imm(5, 3, 7),
        reg_pre: vec![(3, 0x100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R7, R7, #1 (same reg)".into(),
        opcode: enc_lsl_imm(7, 7, 1),
        reg_pre: vec![(7, 0x4000_0000)],
        ..TestCase::default()
    });

    // Value-space edge cases
    t.push(TestCase {
        name: "LSLS R0, R1, #0 (MOVS)".into(),
        opcode: enc_lsl_imm(0, 1, 0),
        reg_pre: vec![(1, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R1, #31 (max shift)".into(),
        opcode: enc_lsl_imm(0, 1, 31),
        reg_pre: vec![(1, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R0, #1 (carry out, result=0)".into(),
        opcode: enc_lsl_imm(0, 0, 1),
        reg_pre: vec![(0, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R1, #1 (zero input)".into(),
        opcode: enc_lsl_imm(0, 1, 1),
        reg_pre: vec![(1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R1, #16 (alternating bits)".into(),
        opcode: enc_lsl_imm(0, 1, 16),
        reg_pre: vec![(1, 0x5555_5555)],
        ..TestCase::default()
    });

    // --- LSRS Rd, Rm, #imm5 ---

    t.push(TestCase {
        name: "LSRS R0, R1, #3".into(),
        opcode: enc_lsr_imm(0, 1, 3),
        reg_pre: vec![(1, 0x80)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R4, R2, #8".into(),
        opcode: enc_lsr_imm(4, 2, 8),
        reg_pre: vec![(2, 0xFF00)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R0, #32 (imm5=0)".into(),
        opcode: enc_lsr_imm(0, 0, 0),
        reg_pre: vec![(0, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R1, #1 (carry out)".into(),
        opcode: enc_lsr_imm(0, 1, 1),
        reg_pre: vec![(1, 0x0000_0001)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R1, #31".into(),
        opcode: enc_lsr_imm(0, 1, 31),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R1, #16 (zero result)".into(),
        opcode: enc_lsr_imm(0, 1, 16),
        reg_pre: vec![(1, 0x0000_FFFF)],
        ..TestCase::default()
    });

    // --- ASRS Rd, Rm, #imm5 ---

    t.push(TestCase {
        name: "ASRS R0, R1, #3 (positive)".into(),
        opcode: enc_asr_imm(0, 1, 3),
        reg_pre: vec![(1, 0x40)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1, #4 (negative)".into(),
        opcode: enc_asr_imm(0, 1, 4),
        reg_pre: vec![(1, 0xFFFF_FF00)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R6, R5, #1".into(),
        opcode: enc_asr_imm(6, 5, 1),
        reg_pre: vec![(5, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R0, #32 (imm5=0, positive)".into(),
        opcode: enc_asr_imm(0, 0, 0),
        reg_pre: vec![(0, 0x7FFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1, #32 (imm5=0, negative)".into(),
        opcode: enc_asr_imm(0, 1, 0),
        reg_pre: vec![(1, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1, #1 (carry out)".into(),
        opcode: enc_asr_imm(0, 1, 1),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1, #31 (negative)".into(),
        opcode: enc_asr_imm(0, 1, 31),
        reg_pre: vec![(1, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1, #1 (zero)".into(),
        opcode: enc_asr_imm(0, 1, 1),
        reg_pre: vec![(1, 0)],
        ..TestCase::default()
    });

    // Additional register field extraction
    t.push(TestCase {
        name: "LSLS R2, R4, #5".into(),
        opcode: enc_lsl_imm(2, 4, 5),
        reg_pre: vec![(4, 0x0100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R6, R0, #10".into(),
        opcode: enc_lsl_imm(6, 0, 10),
        reg_pre: vec![(0, 0xFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R3, R5, #16".into(),
        opcode: enc_lsr_imm(3, 5, 16),
        reg_pre: vec![(5, 0xFFFF_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R7, R0, #4".into(),
        opcode: enc_lsr_imm(7, 0, 4),
        reg_pre: vec![(0, 0xF0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R3, R2, #8".into(),
        opcode: enc_asr_imm(3, 2, 8),
        reg_pre: vec![(2, 0xFF00)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R7, R6, #16".into(),
        opcode: enc_asr_imm(7, 6, 16),
        reg_pre: vec![(6, 0x8000_0000)],
        ..TestCase::default()
    });
    // MAX values
    t.push(TestCase {
        name: "LSLS R0, R1, #1 (MAX input)".into(),
        opcode: enc_lsl_imm(0, 1, 1),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R1, #1 (MAX input)".into(),
        opcode: enc_lsr_imm(0, 1, 1),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R1, #15 (halfword boundary)".into(),
        opcode: enc_lsl_imm(0, 1, 15),
        reg_pre: vec![(1, 0x0001_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R1, #8 (byte boundary)".into(),
        opcode: enc_lsr_imm(0, 1, 8),
        reg_pre: vec![(1, 0x0000_FF00)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1, #15 (alternating bits neg)".into(),
        opcode: enc_asr_imm(0, 1, 15),
        reg_pre: vec![(1, 0xAAAA_AAAA)],
        ..TestCase::default()
    });

    t
}

/// ADD/SUB register and 3-bit imm. Encoding: 000110-000111. ~30 tests.
fn gen_add_sub_reg() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- ADDS Rd, Rn, Rm ---

    t.push(TestCase {
        name: "ADDS R0, R1, R2 (basic)".into(),
        opcode: enc_adds_reg(0, 1, 2),
        reg_pre: vec![(1, 5), (2, 3)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R5, R3, R4 (field extraction)".into(),
        opcode: enc_adds_reg(5, 3, 4),
        reg_pre: vec![(3, 100), (4, 200)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R0, R1 (rd=rn)".into(),
        opcode: enc_adds_reg(0, 0, 1),
        reg_pre: vec![(0, 10), (1, 20)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, R0 (rd=rm)".into(),
        opcode: enc_adds_reg(0, 1, 0),
        reg_pre: vec![(0, 7), (1, 3)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, R2 (overflow)".into(),
        opcode: enc_adds_reg(0, 1, 2),
        reg_pre: vec![(1, 0x7FFF_FFFF), (2, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, R2 (carry)".into(),
        opcode: enc_adds_reg(0, 1, 2),
        reg_pre: vec![(1, 0xFFFF_FFFF), (2, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, R2 (zero result)".into(),
        opcode: enc_adds_reg(0, 1, 2),
        reg_pre: vec![(1, 0), (2, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, R2 (MAX + MAX)".into(),
        opcode: enc_adds_reg(0, 1, 2),
        reg_pre: vec![(1, 0xFFFF_FFFF), (2, 0xFFFF_FFFF)],
        ..TestCase::default()
    });

    // --- SUBS Rd, Rn, Rm ---

    t.push(TestCase {
        name: "SUBS R0, R1, R2 (basic)".into(),
        opcode: enc_subs_reg(0, 1, 2),
        reg_pre: vec![(1, 10), (2, 3)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, R2 (borrow)".into(),
        opcode: enc_subs_reg(0, 1, 2),
        reg_pre: vec![(1, 3), (2, 10)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, R2 (zero result)".into(),
        opcode: enc_subs_reg(0, 1, 2),
        reg_pre: vec![(1, 42), (2, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, R2 (negative overflow)".into(),
        opcode: enc_subs_reg(0, 1, 2),
        reg_pre: vec![(1, 0x8000_0000), (2, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R3, R3, R3 (same reg, zero)".into(),
        opcode: enc_subs_reg(3, 3, 3),
        reg_pre: vec![(3, 0x1234_5678)],
        ..TestCase::default()
    });

    // --- ADDS Rd, Rn, #imm3 ---

    t.push(TestCase {
        name: "ADDS R0, R1, #3".into(),
        opcode: enc_adds_imm3(0, 1, 3),
        reg_pre: vec![(1, 100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, #7 (max imm3)".into(),
        opcode: enc_adds_imm3(0, 1, 7),
        reg_pre: vec![(1, 0xFFFF_FFF9)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, #0".into(),
        opcode: enc_adds_imm3(0, 1, 0),
        reg_pre: vec![(1, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R7, R6, #1 (carry boundary)".into(),
        opcode: enc_adds_imm3(7, 6, 1),
        reg_pre: vec![(6, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, #1 (signed overflow)".into(),
        opcode: enc_adds_imm3(0, 1, 1),
        reg_pre: vec![(1, 0x7FFF_FFFF)],
        ..TestCase::default()
    });

    // --- SUBS Rd, Rn, #imm3 ---

    t.push(TestCase {
        name: "SUBS R0, R1, #3".into(),
        opcode: enc_subs_imm3(0, 1, 3),
        reg_pre: vec![(1, 100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, #1 (to zero)".into(),
        opcode: enc_subs_imm3(0, 1, 1),
        reg_pre: vec![(1, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, #7 (borrow)".into(),
        opcode: enc_subs_imm3(0, 1, 7),
        reg_pre: vec![(1, 3)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, #0 (no-op sub)".into(),
        opcode: enc_subs_imm3(0, 1, 0),
        reg_pre: vec![(1, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, #1 (negative overflow)".into(),
        opcode: enc_subs_imm3(0, 1, 1),
        reg_pre: vec![(1, 0x8000_0000)],
        ..TestCase::default()
    });

    // Additional register field + value edge cases
    t.push(TestCase {
        name: "ADDS R7, R0, R1 (max low regs)".into(),
        opcode: enc_adds_reg(7, 0, 1),
        reg_pre: vec![(0, 0x5555_5555), (1, 0xAAAA_AAAA)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R6, R5, R4".into(),
        opcode: enc_subs_reg(6, 5, 4),
        reg_pre: vec![(5, 0x7FFF_FFFF), (4, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R2, R3, R4 (both zero)".into(),
        opcode: enc_adds_reg(2, 3, 4),
        reg_pre: vec![(3, 0), (4, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, R1 (rd=rm)".into(),
        opcode: enc_adds_reg(0, 1, 1),
        reg_pre: vec![(1, 0x4000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R0, R0 (self sub)".into(),
        opcode: enc_subs_reg(0, 0, 0),
        reg_pre: vec![(0, 0xDEAD_BEEF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R4, R5, #5".into(),
        opcode: enc_adds_imm3(4, 5, 5),
        reg_pre: vec![(5, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R7, R6, #4".into(),
        opcode: enc_subs_imm3(7, 6, 4),
        reg_pre: vec![(6, 10)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, R2 (neg + neg)".into(),
        opcode: enc_adds_reg(0, 1, 2),
        reg_pre: vec![(1, 0x8000_0000), (2, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, R2 (equal MAX)".into(),
        opcode: enc_subs_reg(0, 1, 2),
        reg_pre: vec![(1, 0xFFFF_FFFF), (2, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, R1, #2 (alternating bits)".into(),
        opcode: enc_adds_imm3(0, 1, 2),
        reg_pre: vec![(1, 0x5555_5555)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, R1, #5 (from MAX)".into(),
        opcode: enc_subs_imm3(0, 1, 5),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });

    t
}

/// MOV, CMP, ADD, SUB with 8-bit imm. Encoding: 001xx. ~30 tests.
fn gen_mov_cmp_imm8() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- MOVS Rd, #imm8 ---

    t.push(TestCase {
        name: "MOVS R0, #42".into(),
        opcode: enc_movs_imm(0, 42),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOVS R7, #0xFF".into(),
        opcode: enc_movs_imm(7, 0xFF),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOVS R0, #0 (Z flag)".into(),
        opcode: enc_movs_imm(0, 0),
        reg_pre: vec![(0, 999)], // overwrite nonzero
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOVS R3, #1".into(),
        opcode: enc_movs_imm(3, 1),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOVS R4, #0x55 (alternating bits)".into(),
        opcode: enc_movs_imm(4, 0x55),
        ..TestCase::default()
    });

    // --- CMP Rn, #imm8 ---

    t.push(TestCase {
        name: "CMP R0, #42 (equal)".into(),
        opcode: enc_cmp_imm(0, 42),
        reg_pre: vec![(0, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, #42 (greater)".into(),
        opcode: enc_cmp_imm(0, 42),
        reg_pre: vec![(0, 100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, #42 (less)".into(),
        opcode: enc_cmp_imm(0, 42),
        reg_pre: vec![(0, 10)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, #0 (zero)".into(),
        opcode: enc_cmp_imm(0, 0),
        reg_pre: vec![(0, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, #0xFF (large imm)".into(),
        opcode: enc_cmp_imm(0, 0xFF),
        reg_pre: vec![(0, 0xFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, #1 (negative result)".into(),
        opcode: enc_cmp_imm(0, 1),
        reg_pre: vec![(0, 0x8000_0000)],
        ..TestCase::default()
    });

    // --- ADDS Rdn, #imm8 ---

    t.push(TestCase {
        name: "ADDS R0, #25".into(),
        opcode: enc_adds_imm8(0, 25),
        reg_pre: vec![(0, 100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, #0xFF (carry)".into(),
        opcode: enc_adds_imm8(0, 0xFF),
        reg_pre: vec![(0, 0xFFFF_FF01)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, #1 (signed overflow)".into(),
        opcode: enc_adds_imm8(0, 1),
        reg_pre: vec![(0, 0x7FFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R0, #0 (no change)".into(),
        opcode: enc_adds_imm8(0, 0),
        reg_pre: vec![(0, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R7, #1 (zero result)".into(),
        opcode: enc_adds_imm8(7, 1),
        reg_pre: vec![(7, 0xFFFF_FFFF)],
        ..TestCase::default()
    });

    // --- SUBS Rdn, #imm8 ---

    t.push(TestCase {
        name: "SUBS R0, #25".into(),
        opcode: enc_subs_imm8(0, 25),
        reg_pre: vec![(0, 100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, #1 (to zero)".into(),
        opcode: enc_subs_imm8(0, 1),
        reg_pre: vec![(0, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, #1 (borrow)".into(),
        opcode: enc_subs_imm8(0, 1),
        reg_pre: vec![(0, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, #0xFF (large imm)".into(),
        opcode: enc_subs_imm8(0, 0xFF),
        reg_pre: vec![(0, 0xFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R0, #1 (negative overflow)".into(),
        opcode: enc_subs_imm8(0, 1),
        reg_pre: vec![(0, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R5, #0x80 (alternating bits)".into(),
        opcode: enc_subs_imm8(5, 0x80),
        reg_pre: vec![(5, 0x5555_5555)],
        ..TestCase::default()
    });

    // Additional register fields + value edges
    t.push(TestCase {
        name: "MOVS R1, #0x80".into(),
        opcode: enc_movs_imm(1, 0x80),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOVS R6, #0xAA".into(),
        opcode: enc_movs_imm(6, 0xAA),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R1, #0 (positive val)".into(),
        opcode: enc_cmp_imm(1, 0),
        reg_pre: vec![(1, 100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R3, #0x80".into(),
        opcode: enc_cmp_imm(3, 0x80),
        reg_pre: vec![(3, 0x80)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, #1 (carry boundary from 0)".into(),
        opcode: enc_cmp_imm(0, 1),
        reg_pre: vec![(0, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R1, #0x55".into(),
        opcode: enc_adds_imm8(1, 0x55),
        reg_pre: vec![(1, 0xAAAA_AAAB)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R2, #1 (from MAX-1)".into(),
        opcode: enc_adds_imm8(2, 1),
        reg_pre: vec![(2, 0xFFFF_FFFE)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R3, #0x55".into(),
        opcode: enc_subs_imm8(3, 0x55),
        reg_pre: vec![(3, 0x55)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R4, #0xFF (from 0)".into(),
        opcode: enc_subs_imm8(4, 0xFF),
        reg_pre: vec![(4, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOVS R2, #0".into(),
        opcode: enc_movs_imm(2, 0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R6, #0xAA".into(),
        opcode: enc_adds_imm8(6, 0xAA),
        reg_pre: vec![(6, 0x5555_5556)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R7, #0xFF (negative val)".into(),
        opcode: enc_cmp_imm(7, 0xFF),
        reg_pre: vec![(7, 0xFFFF_FF00)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUBS R6, #1 (from 0x80000000)".into(),
        opcode: enc_subs_imm8(6, 1),
        reg_pre: vec![(6, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADDS R3, #0x80 (overflow boundary)".into(),
        opcode: enc_adds_imm8(3, 0x80),
        reg_pre: vec![(3, 0x7FFF_FF80)],
        ..TestCase::default()
    });

    t
}

/// Data processing (register). Encoding: 010000. ~40 tests.
fn gen_data_proc_reg() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- ANDS ---
    t.push(TestCase {
        name: "ANDS R0, R1 (basic)".into(),
        opcode: enc_data_proc(0, 1, 0),
        reg_pre: vec![(0, 0xFF), (1, 0x0F)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ANDS R0, R1 (zero result)".into(),
        opcode: enc_data_proc(0, 1, 0),
        reg_pre: vec![(0, 0xFF00), (1, 0x00FF)],
        ..TestCase::default()
    });

    // --- EORS ---
    t.push(TestCase {
        name: "EORS R0, R1 (basic)".into(),
        opcode: enc_data_proc(1, 1, 0),
        reg_pre: vec![(0, 0xFF), (1, 0xF0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "EORS R0, R0 (self, zero result)".into(),
        opcode: enc_data_proc(1, 0, 0),
        reg_pre: vec![(0, 0xDEAD_BEEF)],
        ..TestCase::default()
    });

    // --- LSLS (register) ---
    t.push(TestCase {
        name: "LSLS R0, R1 (reg, shift 4)".into(),
        opcode: enc_data_proc(2, 1, 0),
        reg_pre: vec![(0, 1), (1, 4)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R1 (reg, shift 0)".into(),
        opcode: enc_data_proc(2, 1, 0),
        reg_pre: vec![(0, 0xDEAD_BEEF), (1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R1 (reg, shift 32)".into(),
        opcode: enc_data_proc(2, 1, 0),
        reg_pre: vec![(0, 0x8000_0001), (1, 32)],
        ..TestCase::default()
    });

    // --- LSRS (register) ---
    t.push(TestCase {
        name: "LSRS R0, R1 (reg, shift 4)".into(),
        opcode: enc_data_proc(3, 1, 0),
        reg_pre: vec![(0, 0x100), (1, 4)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R1 (reg, shift 32)".into(),
        opcode: enc_data_proc(3, 1, 0),
        reg_pre: vec![(0, 0x8000_0000), (1, 32)],
        ..TestCase::default()
    });

    // --- ASRS (register) ---
    t.push(TestCase {
        name: "ASRS R0, R1 (reg, positive)".into(),
        opcode: enc_data_proc(4, 1, 0),
        reg_pre: vec![(0, 0x80), (1, 3)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1 (reg, negative)".into(),
        opcode: enc_data_proc(4, 1, 0),
        reg_pre: vec![(0, 0x8000_0000), (1, 4)],
        ..TestCase::default()
    });

    // --- ADCS ---
    t.push(TestCase {
        name: "ADCS R0, R1 (C=1)".into(),
        opcode: enc_data_proc(5, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 0)],
        xpsr_pre: 0x0100_0000 | (1 << 29), // T + C
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADCS R0, R1 (C=0)".into(),
        opcode: enc_data_proc(5, 1, 0),
        reg_pre: vec![(0, 5), (1, 3)],
        ..TestCase::default()
    });

    // --- SBCS ---
    t.push(TestCase {
        name: "SBCS R0, R1 (C=1, no borrow)".into(),
        opcode: enc_data_proc(6, 1, 0),
        reg_pre: vec![(0, 10), (1, 3)],
        xpsr_pre: 0x0100_0000 | (1 << 29), // T + C
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SBCS R0, R1 (C=0, borrow)".into(),
        opcode: enc_data_proc(6, 1, 0),
        reg_pre: vec![(0, 10), (1, 3)],
        ..TestCase::default()
    });

    // --- RORS ---
    t.push(TestCase {
        name: "RORS R0, R1 (rotate 1)".into(),
        opcode: enc_data_proc(7, 1, 0),
        reg_pre: vec![(0, 1), (1, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RORS R0, R1 (rotate 0)".into(),
        opcode: enc_data_proc(7, 1, 0),
        reg_pre: vec![(0, 0xDEAD_BEEF), (1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RORS R0, R1 (rotate 16)".into(),
        opcode: enc_data_proc(7, 1, 0),
        reg_pre: vec![(0, 0x0000_FFFF), (1, 16)],
        ..TestCase::default()
    });

    // --- TST ---
    t.push(TestCase {
        name: "TST R0, R1 (no common bits)".into(),
        opcode: enc_data_proc(8, 1, 0),
        reg_pre: vec![(0, 0xFF00), (1, 0x00FF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "TST R0, R1 (all bits common)".into(),
        opcode: enc_data_proc(8, 1, 0),
        reg_pre: vec![(0, 0x8000_0000), (1, 0x8000_0000)],
        ..TestCase::default()
    });

    // --- RSBS (NEG) ---
    t.push(TestCase {
        name: "RSBS R0, R1 (negate 42)".into(),
        opcode: enc_data_proc(9, 1, 0),
        reg_pre: vec![(1, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RSBS R0, R1 (negate 0)".into(),
        opcode: enc_data_proc(9, 1, 0),
        reg_pre: vec![(1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RSBS R0, R1 (negate MIN)".into(),
        opcode: enc_data_proc(9, 1, 0),
        reg_pre: vec![(1, 0x8000_0000)],
        ..TestCase::default()
    });

    // --- CMP (register) ---
    t.push(TestCase {
        name: "CMP R0, R1 (equal)".into(),
        opcode: enc_data_proc(0xA, 1, 0),
        reg_pre: vec![(0, 42), (1, 42)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, R1 (greater)".into(),
        opcode: enc_data_proc(0xA, 1, 0),
        reg_pre: vec![(0, 100), (1, 42)],
        ..TestCase::default()
    });

    // --- CMN ---
    t.push(TestCase {
        name: "CMN R0, R1 (carry+zero)".into(),
        opcode: enc_data_proc(0xB, 1, 0),
        reg_pre: vec![(0, 1), (1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });

    // --- ORRS ---
    t.push(TestCase {
        name: "ORRS R0, R1".into(),
        opcode: enc_data_proc(0xC, 1, 0),
        reg_pre: vec![(0, 0xF0), (1, 0x0F)],
        ..TestCase::default()
    });

    // --- MULS ---
    t.push(TestCase {
        name: "MULS R0, R1 (7*6)".into(),
        opcode: enc_data_proc(0xD, 1, 0),
        reg_pre: vec![(0, 7), (1, 6)],
        xpsr_mask: MASK_NZ_ONLY,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MULS R0, R1 (zero)".into(),
        opcode: enc_data_proc(0xD, 1, 0),
        reg_pre: vec![(0, 0), (1, 42)],
        xpsr_mask: MASK_NZ_ONLY,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MULS R0, R1 (large, negative result)".into(),
        opcode: enc_data_proc(0xD, 1, 0),
        reg_pre: vec![(0, 0x1_0000), (1, 0x1_0000)],
        xpsr_mask: MASK_NZ_ONLY,
        ..TestCase::default()
    });

    // --- BICS ---
    t.push(TestCase {
        name: "BICS R0, R1".into(),
        opcode: enc_data_proc(0xE, 1, 0),
        reg_pre: vec![(0, 0xFF), (1, 0x0F)],
        ..TestCase::default()
    });

    // --- MVNS ---
    t.push(TestCase {
        name: "MVNS R0, R1 (NOT 0)".into(),
        opcode: enc_data_proc(0xF, 1, 0),
        reg_pre: vec![(1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MVNS R0, R1 (NOT MAX)".into(),
        opcode: enc_data_proc(0xF, 1, 0),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });

    // Additional value-edge cases
    t.push(TestCase {
        name: "ANDS R0, R1 (alternating bits)".into(),
        opcode: enc_data_proc(0, 1, 0),
        reg_pre: vec![(0, 0x5555_5555), (1, 0xAAAA_AAAA)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ANDS R3, R4 (field extract)".into(),
        opcode: enc_data_proc(0, 4, 3),
        reg_pre: vec![(3, 0xFFFF_FFFF), (4, 0x0000_FF00)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "EORS R2, R3 (alternating bits)".into(),
        opcode: enc_data_proc(1, 3, 2),
        reg_pre: vec![(2, 0x5555_5555), (3, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSLS R0, R1 (reg, shift 33)".into(),
        opcode: enc_data_proc(2, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 33)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LSRS R0, R1 (reg, shift 33)".into(),
        opcode: enc_data_proc(3, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 33)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ASRS R0, R1 (reg, shift 32)".into(),
        opcode: enc_data_proc(4, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 32)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADCS R0, R1 (both MAX, C=1)".into(),
        opcode: enc_data_proc(5, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 0xFFFF_FFFF)],
        xpsr_pre: 0x0100_0000 | (1 << 29),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SBCS R0, R1 (equal, C=1)".into(),
        opcode: enc_data_proc(6, 1, 0),
        reg_pre: vec![(0, 42), (1, 42)],
        xpsr_pre: 0x0100_0000 | (1 << 29),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SBCS R0, R1 (0 - 0, C=0)".into(),
        opcode: enc_data_proc(6, 1, 0),
        reg_pre: vec![(0, 0), (1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RORS R0, R1 (rotate 32)".into(),
        opcode: enc_data_proc(7, 1, 0),
        reg_pre: vec![(0, 0xDEAD_BEEF), (1, 32)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "TST R0, R0 (negative)".into(),
        opcode: enc_data_proc(8, 0, 0),
        reg_pre: vec![(0, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RSBS R0, R1 (negate 1)".into(),
        opcode: enc_data_proc(9, 1, 0),
        reg_pre: vec![(1, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RSBS R0, R1 (negate MAX)".into(),
        opcode: enc_data_proc(9, 1, 0),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, R1 (less)".into(),
        opcode: enc_data_proc(0xA, 1, 0),
        reg_pre: vec![(0, 10), (1, 100)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R0, R1 (neg vs pos)".into(),
        opcode: enc_data_proc(0xA, 1, 0),
        reg_pre: vec![(0, 0x8000_0000), (1, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMN R0, R1 (both zero)".into(),
        opcode: enc_data_proc(0xB, 1, 0),
        reg_pre: vec![(0, 0), (1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMN R0, R1 (overflow)".into(),
        opcode: enc_data_proc(0xB, 1, 0),
        reg_pre: vec![(0, 0x7FFF_FFFF), (1, 1)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ORRS R0, R1 (zero inputs)".into(),
        opcode: enc_data_proc(0xC, 1, 0),
        reg_pre: vec![(0, 0), (1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ORRS R0, R1 (negative result)".into(),
        opcode: enc_data_proc(0xC, 1, 0),
        reg_pre: vec![(0, 0x8000_0000), (1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MULS R0, R1 (1*1)".into(),
        opcode: enc_data_proc(0xD, 1, 0),
        reg_pre: vec![(0, 1), (1, 1)],
        xpsr_mask: MASK_NZ_ONLY,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MULS R0, R1 (MAX*2, wrap)".into(),
        opcode: enc_data_proc(0xD, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 2)],
        xpsr_mask: MASK_NZ_ONLY,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BICS R5, R6 (all bits)".into(),
        opcode: enc_data_proc(0xE, 6, 5),
        reg_pre: vec![(5, 0xFFFF_FFFF), (6, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BICS R0, R1 (no overlap)".into(),
        opcode: enc_data_proc(0xE, 1, 0),
        reg_pre: vec![(0, 0x00FF), (1, 0xFF00)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MVNS R3, R4 (alternating bits)".into(),
        opcode: enc_data_proc(0xF, 4, 3),
        reg_pre: vec![(4, 0x5555_5555)],
        ..TestCase::default()
    });

    // More register combos and corner cases
    t.push(TestCase {
        name: "ANDS R7, R6".into(),
        opcode: enc_data_proc(0, 6, 7),
        reg_pre: vec![(7, 0x8000_0001), (6, 0x8000_0000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "EORS R5, R4 (MAX ^ 0)".into(),
        opcode: enc_data_proc(1, 4, 5),
        reg_pre: vec![(5, 0xFFFF_FFFF), (4, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ORRS R3, R2 (MAX | MAX)".into(),
        opcode: enc_data_proc(0xC, 2, 3),
        reg_pre: vec![(3, 0xFFFF_FFFF), (2, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MULS R5, R6 (neg * neg)".into(),
        opcode: enc_data_proc(0xD, 6, 5),
        reg_pre: vec![(5, 0xFFFF_FFFF), (6, 0xFFFF_FFFF)],
        xpsr_mask: MASK_NZ_ONLY,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADCS R3, R4 (0 + 0, C=0)".into(),
        opcode: enc_data_proc(5, 4, 3),
        reg_pre: vec![(3, 0), (4, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SBCS R5, R6 (MAX - 0, C=1)".into(),
        opcode: enc_data_proc(6, 6, 5),
        reg_pre: vec![(5, 0xFFFF_FFFF), (6, 0)],
        xpsr_pre: 0x0100_0000 | (1 << 29),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "RORS R3, R4 (rotate 8)".into(),
        opcode: enc_data_proc(7, 4, 3),
        reg_pre: vec![(3, 0x1234_5678), (4, 8)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "TST R3, R4 (both MAX)".into(),
        opcode: enc_data_proc(8, 4, 3),
        reg_pre: vec![(3, 0xFFFF_FFFF), (4, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "CMP R5, R6 (both zero)".into(),
        opcode: enc_data_proc(0xA, 6, 5),
        reg_pre: vec![(5, 0), (6, 0)],
        ..TestCase::default()
    });

    t
}

/// Special data: MOV high, ADD high, BX. Encoding: 010001. ~15 tests.
fn gen_special_data_bx() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- MOV Rd, Rm (high registers) ---
    t.push(TestCase {
        name: "MOV R0, R8 (high to low)".into(),
        opcode: enc_mov_high(0, 8),
        reg_pre: vec![(8, 0xDEAD_BEEF)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOV R8, R0 (low to high)".into(),
        opcode: enc_mov_high(8, 0),
        reg_pre: vec![(0, 0xCAFE_BABE)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOV R0, R9 (high to low, zero)".into(),
        opcode: enc_mov_high(0, 9),
        reg_pre: vec![(9, 0)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOV R10, R11 (high to high)".into(),
        opcode: enc_mov_high(10, 11),
        reg_pre: vec![(11, 0x1234_5678)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });

    // --- ADD Rd, Rm (high registers) ---
    t.push(TestCase {
        name: "ADD R0, R8 (high reg add)".into(),
        opcode: enc_add_high(0, 8),
        reg_pre: vec![(0, 10), (8, 20)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD R8, R0 (low to high add)".into(),
        opcode: enc_add_high(8, 0),
        reg_pre: vec![(8, 100), (0, 50)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD R0, R8 (large values)".into(),
        opcode: enc_add_high(0, 8),
        reg_pre: vec![(0, 0xFFFF_FFFF), (8, 1)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD R0, R9 (alternating bits)".into(),
        opcode: enc_add_high(0, 9),
        reg_pre: vec![(0, 0x5555_5555), (9, 0xAAAA_AAAA)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });

    // --- BX Rm ---
    // BX changes PC, so we verify via PC delta.
    // Target address must be within reasonable range and have Thumb bit.
    t.push(TestCase {
        name: "BX R0 (basic)".into(),
        opcode: enc_bx(0),
        // Target = scratch + some offset, but BX doesn't need bus.
        // We set a specific address with Thumb bit.
        reg_pre: vec![(0, 0x0000_0201)], // scratch_base + 1 for Thumb
        addr_regs: vec![0],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BX R3 (different reg)".into(),
        opcode: enc_bx(3),
        reg_pre: vec![(3, 0x0000_0211)], // arbitrary valid address + Thumb
        addr_regs: vec![3],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BX R8 (high reg)".into(),
        opcode: enc_bx(8),
        reg_pre: vec![(8, 0x0000_0221)], // valid address + Thumb
        addr_regs: vec![8],
        ..TestCase::default()
    });

    // Additional MOV/ADD high register cases
    t.push(TestCase {
        name: "MOV R1, R10".into(),
        opcode: enc_mov_high(1, 10),
        reg_pre: vec![(10, 0x5555_5555)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOV R12, R0 (to high)".into(),
        opcode: enc_mov_high(12, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD R1, R10 (high to low)".into(),
        opcode: enc_add_high(1, 10),
        reg_pre: vec![(1, 0x1000), (10, 0x2000)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD R12, R1 (low to high)".into(),
        opcode: enc_add_high(12, 1),
        reg_pre: vec![(12, 0), (1, 42)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "MOV R0, R12 (max value)".into(),
        opcode: enc_mov_high(0, 12),
        reg_pre: vec![(12, 0xFFFF_FFFF)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD R0, R10 (zero + zero)".into(),
        opcode: enc_add_high(0, 10),
        reg_pre: vec![(0, 0), (10, 0)],
        xpsr_mask: MASK_NO_FLAGS,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BX R1 (low reg)".into(),
        opcode: enc_bx(1),
        reg_pre: vec![(1, 0x0000_0241)],
        addr_regs: vec![1],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BX R12 (high reg)".into(),
        opcode: enc_bx(12),
        reg_pre: vec![(12, 0x0000_0251)],
        addr_regs: vec![12],
        ..TestCase::default()
    });

    t
}

/// Load/store register offset. Encoding: 0101. ~30 tests.
fn gen_load_store_reg() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- STR Rt, [Rn, Rm] (opc=000) ---
    t.push(TestCase {
        name: "STR R0, [R1, R2]".into(),
        opcode: enc_ls_reg(0b000, 2, 1, 0),
        reg_pre: vec![(0, 0xCAFE_BABE), (1, 0), (2, 4)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R3, [R4, R5] (field extract)".into(),
        opcode: enc_ls_reg(0b000, 5, 4, 3),
        reg_pre: vec![(3, 0x1234_5678), (4, 0), (5, 8)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_check: mem_check_u32(8),
        ..TestCase::default()
    });

    // --- STRH Rt, [Rn, Rm] (opc=001) ---
    t.push(TestCase {
        name: "STRH R0, [R1, R2]".into(),
        opcode: enc_ls_reg(0b001, 2, 1, 0),
        reg_pre: vec![(0, 0xBEEF), (1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(0),
        ..TestCase::default()
    });

    // --- STRB Rt, [Rn, Rm] (opc=010) ---
    t.push(TestCase {
        name: "STRB R0, [R1, R2]".into(),
        opcode: enc_ls_reg(0b010, 2, 1, 0),
        reg_pre: vec![(0, 0xAB), (1, 0), (2, 1)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: vec![1],
        ..TestCase::default()
    });

    // --- LDRSB Rt, [Rn, Rm] (opc=011) ---
    t.push(TestCase {
        name: "LDRSB R0, [R1, R2] (positive)".into(),
        opcode: enc_ls_reg(0b011, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: vec![(0, 0x7F)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRSB R0, [R1, R2] (negative, sign extend)".into(),
        opcode: enc_ls_reg(0b011, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: vec![(0, 0x80)],
        ..TestCase::default()
    });

    // --- LDR Rt, [Rn, Rm] (opc=100) ---
    t.push(TestCase {
        name: "LDR R0, [R1, R2] (basic)".into(),
        opcode: enc_ls_reg(0b100, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 4)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(4, 0xDEAD_BEEF),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R5, [R3, R4] (field extract)".into(),
        opcode: enc_ls_reg(0b100, 4, 3, 5),
        reg_pre: vec![(3, 0), (4, 8)],
        addr_regs: vec![3],
        needs_bus: true,
        mem_pre: mem_pre_u32(8, 0x1234_5678),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R0, [R1, R2] (zero)".into(),
        opcode: enc_ls_reg(0b100, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R0, [R1, R2] (MAX)".into(),
        opcode: enc_ls_reg(0b100, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xFFFF_FFFF),
        ..TestCase::default()
    });

    // --- LDRH Rt, [Rn, Rm] (opc=101) ---
    t.push(TestCase {
        name: "LDRH R0, [R1, R2]".into(),
        opcode: enc_ls_reg(0b101, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 4)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(4, 0xBEEF),
        ..TestCase::default()
    });

    // --- LDRB Rt, [Rn, Rm] (opc=110) ---
    t.push(TestCase {
        name: "LDRB R0, [R1, R2]".into(),
        opcode: enc_ls_reg(0b110, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: vec![(0, 0xCD)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRB R0, [R1, R2] (zero)".into(),
        opcode: enc_ls_reg(0b110, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 2)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: vec![(2, 0)],
        ..TestCase::default()
    });

    // --- LDRSH Rt, [Rn, Rm] (opc=111) ---
    t.push(TestCase {
        name: "LDRSH R0, [R1, R2] (positive)".into(),
        opcode: enc_ls_reg(0b111, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(0, 0x7FFF),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRSH R0, [R1, R2] (negative, sign extend)".into(),
        opcode: enc_ls_reg(0b111, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(0, 0x8000),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRSH R0, [R1, R2] (0xFFFF = -1)".into(),
        opcode: enc_ls_reg(0b111, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 4)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(4, 0xFFFF),
        ..TestCase::default()
    });

    // STR/LDR roundtrip
    t.push(TestCase {
        name: "STR R0, [R1, R2] (MAX value)".into(),
        opcode: enc_ls_reg(0b000, 2, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRB R0, [R1, R2] (0xFF)".into(),
        opcode: enc_ls_reg(0b010, 2, 1, 0),
        reg_pre: vec![(0, 0xFF), (1, 0), (2, 3)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: vec![3],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRH R0, [R1, R2] (0xFFFF)".into(),
        opcode: enc_ls_reg(0b001, 2, 1, 0),
        reg_pre: vec![(0, 0xFFFF), (1, 0), (2, 6)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(6),
        ..TestCase::default()
    });

    // Additional field extraction and edge cases
    t.push(TestCase {
        name: "STR R7, [R6, R5]".into(),
        opcode: enc_ls_reg(0b000, 5, 6, 7),
        reg_pre: vec![(7, 0xBEEF_CAFE), (6, 0), (5, 0)],
        addr_regs: vec![6],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R7, [R6, R5]".into(),
        opcode: enc_ls_reg(0b100, 5, 6, 7),
        reg_pre: vec![(6, 0), (5, 12)],
        addr_regs: vec![6],
        needs_bus: true,
        mem_pre: mem_pre_u32(12, 0xAAAA_BBBB),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRB R5, [R3, R4] (field extract)".into(),
        opcode: enc_ls_reg(0b010, 4, 3, 5),
        reg_pre: vec![(5, 0x42), (3, 0), (4, 5)],
        addr_regs: vec![3],
        needs_bus: true,
        mem_check: vec![5],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRB R6, [R4, R5] (zero)".into(),
        opcode: enc_ls_reg(0b110, 5, 4, 6),
        reg_pre: vec![(4, 0), (5, 10)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_pre: vec![(10, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRSB R3, [R4, R5] (0xFF = -1)".into(),
        opcode: enc_ls_reg(0b011, 5, 4, 3),
        reg_pre: vec![(4, 0), (5, 8)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_pre: vec![(8, 0xFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRH R2, [R3, R4] (zero)".into(),
        opcode: enc_ls_reg(0b101, 4, 3, 2),
        reg_pre: vec![(3, 0), (4, 0)],
        addr_regs: vec![3],
        needs_bus: true,
        mem_pre: mem_pre_u16(0, 0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRSH R0, [R1, R2] (0x0001)".into(),
        opcode: enc_ls_reg(0b111, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 8)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(8, 0x0001),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRH R3, [R1, R2] (field extract)".into(),
        opcode: enc_ls_reg(0b001, 2, 1, 3),
        reg_pre: vec![(3, 0x1234), (1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [R1, R2] (zero value)".into(),
        opcode: enc_ls_reg(0b000, 2, 1, 0),
        reg_pre: vec![(0, 0), (1, 0), (2, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRSB R0, [R1, R2] (0x00)".into(),
        opcode: enc_ls_reg(0b011, 2, 1, 0),
        reg_pre: vec![(1, 0), (2, 3)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: vec![(3, 0)],
        ..TestCase::default()
    });

    t
}

/// Load/store immediate offset. Encoding: 011xx, 100xx. ~30 tests.
fn gen_load_store_imm() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- STR Rt, [Rn, #imm5*4] ---
    t.push(TestCase {
        name: "STR R0, [R1, #0]".into(),
        opcode: enc_str_imm(0, 1, 0),
        reg_pre: vec![(0, 0xDEAD_BEEF), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [R1, #4]".into(),
        opcode: enc_str_imm(0, 1, 1),
        reg_pre: vec![(0, 0x1234_5678), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [R1, #124] (max offset)".into(),
        opcode: enc_str_imm(0, 1, 31),
        reg_pre: vec![(0, 0xCAFE), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(124),
        ..TestCase::default()
    });

    // --- LDR Rt, [Rn, #imm5*4] ---
    t.push(TestCase {
        name: "LDR R0, [R1, #0]".into(),
        opcode: enc_ldr_imm(0, 1, 0),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xCAFE_BABE),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R2, [R1, #8]".into(),
        opcode: enc_ldr_imm(2, 1, 2),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(8, 0x1234_5678),
        ..TestCase::default()
    });

    // --- STRB Rt, [Rn, #imm5] ---
    t.push(TestCase {
        name: "STRB R0, [R1, #2]".into(),
        opcode: enc_strb_imm(0, 1, 2),
        reg_pre: vec![(0, 0xCD), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: vec![2],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRB R0, [R1, #31] (max offset)".into(),
        opcode: enc_strb_imm(0, 1, 31),
        reg_pre: vec![(0, 0xAB), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: vec![31],
        ..TestCase::default()
    });

    // --- LDRB Rt, [Rn, #imm5] ---
    t.push(TestCase {
        name: "LDRB R0, [R1, #0]".into(),
        opcode: enc_ldrb_imm(0, 1, 0),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: vec![(0, 0xEF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRB R0, [R1, #5]".into(),
        opcode: enc_ldrb_imm(0, 1, 5),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: vec![(5, 0x42)],
        ..TestCase::default()
    });

    // --- STRH Rt, [Rn, #imm5*2] ---
    t.push(TestCase {
        name: "STRH R0, [R1, #0]".into(),
        opcode: enc_strh_imm(0, 1, 0),
        reg_pre: vec![(0, 0xBEEF), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRH R0, [R1, #4]".into(),
        opcode: enc_strh_imm(0, 1, 2),
        reg_pre: vec![(0, 0xFACE), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRH R0, [R1, #62] (max offset)".into(),
        opcode: enc_strh_imm(0, 1, 31),
        reg_pre: vec![(0, 0x1234), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(62),
        ..TestCase::default()
    });

    // --- LDRH Rt, [Rn, #imm5*2] ---
    t.push(TestCase {
        name: "LDRH R0, [R1, #0]".into(),
        opcode: enc_ldrh_imm(0, 1, 0),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(0, 0xDEAD),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRH R0, [R1, #4]".into(),
        opcode: enc_ldrh_imm(0, 1, 2),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(4, 0xBEEF),
        ..TestCase::default()
    });

    // Value edge cases for stores
    t.push(TestCase {
        name: "STR R0, [R1, #0] (zero value)".into(),
        opcode: enc_str_imm(0, 1, 0),
        reg_pre: vec![(0, 0), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [R1, #0] (MAX value)".into(),
        opcode: enc_str_imm(0, 1, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R0, [R1, #0] (MAX value)".into(),
        opcode: enc_ldr_imm(0, 1, 0),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xFFFF_FFFF),
        ..TestCase::default()
    });

    // Different register fields
    t.push(TestCase {
        name: "STR R7, [R6, #8]".into(),
        opcode: enc_str_imm(7, 6, 2),
        reg_pre: vec![(7, 0xABCD_EF01), (6, 0)],
        addr_regs: vec![6],
        needs_bus: true,
        mem_check: mem_check_u32(8),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R5, [R4, #12]".into(),
        opcode: enc_ldr_imm(5, 4, 3),
        reg_pre: vec![(4, 0)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_pre: mem_pre_u32(12, 0x5555_AAAA),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRB R3, [R2, #10]".into(),
        opcode: enc_ldrb_imm(3, 2, 10),
        reg_pre: vec![(2, 0)],
        addr_regs: vec![2],
        needs_bus: true,
        mem_pre: vec![(10, 0x55)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRB R0, [R1, #0] (0xFF)".into(),
        opcode: enc_strb_imm(0, 1, 0),
        reg_pre: vec![(0, 0xFF), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: vec![0],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRH R0, [R1, #6]".into(),
        opcode: enc_ldrh_imm(0, 1, 3),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(6, 0xFFFF),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRH R0, [R1, #10]".into(),
        opcode: enc_strh_imm(0, 1, 5),
        reg_pre: vec![(0, 0), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(10),
        ..TestCase::default()
    });

    // Additional field extraction and edge cases
    t.push(TestCase {
        name: "STR R3, [R2, #16]".into(),
        opcode: enc_str_imm(3, 2, 4),
        reg_pre: vec![(3, 0x5555_AAAA), (2, 0)],
        addr_regs: vec![2],
        needs_bus: true,
        mem_check: mem_check_u32(16),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R7, [R6, #0]".into(),
        opcode: enc_ldr_imm(7, 6, 0),
        reg_pre: vec![(6, 0)],
        addr_regs: vec![6],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xBEEF_CAFE),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRB R4, [R5, #10]".into(),
        opcode: enc_strb_imm(4, 5, 10),
        reg_pre: vec![(4, 0x42), (5, 0)],
        addr_regs: vec![5],
        needs_bus: true,
        mem_check: vec![10],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRB R7, [R6, #20]".into(),
        opcode: enc_ldrb_imm(7, 6, 20),
        reg_pre: vec![(6, 0)],
        addr_regs: vec![6],
        needs_bus: true,
        mem_pre: vec![(20, 0xAB)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRH R5, [R4, #8]".into(),
        opcode: enc_strh_imm(5, 4, 4),
        reg_pre: vec![(5, 0xABCD), (4, 0)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_check: mem_check_u16(8),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRH R6, [R5, #2]".into(),
        opcode: enc_ldrh_imm(6, 5, 1),
        reg_pre: vec![(5, 0)],
        addr_regs: vec![5],
        needs_bus: true,
        mem_pre: mem_pre_u16(2, 0x5555),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [R1, #8] (alternating bits)".into(),
        opcode: enc_str_imm(0, 1, 2),
        reg_pre: vec![(0, 0xAAAA_AAAA), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u32(8),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R0, [R1, #4] (alternating bits)".into(),
        opcode: enc_ldr_imm(0, 1, 1),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(4, 0x5555_5555),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRB R1, [R0, #15]".into(),
        opcode: enc_strb_imm(1, 0, 15),
        reg_pre: vec![(1, 0xEE), (0, 0)],
        addr_regs: vec![0],
        needs_bus: true,
        mem_check: vec![15],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRB R2, [R3, #0] (zero)".into(),
        opcode: enc_ldrb_imm(2, 3, 0),
        reg_pre: vec![(3, 0)],
        addr_regs: vec![3],
        needs_bus: true,
        mem_pre: vec![(0, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STRH R0, [R1, #0] (zero value)".into(),
        opcode: enc_strh_imm(0, 1, 0),
        reg_pre: vec![(0, 0), (1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: mem_check_u16(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDRH R0, [R1, #0] (zero)".into(),
        opcode: enc_ldrh_imm(0, 1, 0),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u16(0, 0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R4, [R3, #20]".into(),
        opcode: enc_ldr_imm(4, 3, 5),
        reg_pre: vec![(3, 0)],
        addr_regs: vec![3],
        needs_bus: true,
        mem_pre: mem_pre_u32(20, 0x8000_0001),
        ..TestCase::default()
    });

    t
}

/// STR, LDR (SP-relative). Encoding: 1001x. ~10 tests.
fn gen_load_store_sp() -> Vec<TestCase> {
    let mut t = Vec::new();

    // STR Rt, [SP, #imm8*4]
    t.push(TestCase {
        name: "STR R0, [SP, #0]".into(),
        opcode: enc_str_sp(0, 0),
        reg_pre: vec![(0, 0xDEAD_BEEF), (13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [SP, #8]".into(),
        opcode: enc_str_sp(0, 2),
        reg_pre: vec![(0, 0xCAFE), (13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(8),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R7, [SP, #4]".into(),
        opcode: enc_str_sp(7, 1),
        reg_pre: vec![(7, 0x1234_5678), (13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(4),
        ..TestCase::default()
    });

    // LDR Rt, [SP, #imm8*4]
    t.push(TestCase {
        name: "LDR R0, [SP, #0]".into(),
        opcode: enc_ldr_sp(0, 0),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xDEAD_BEEF),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R1, [SP, #8]".into(),
        opcode: enc_ldr_sp(1, 2),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(8, 0xCAFE_BABE),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R0, [SP, #0] (zero)".into(),
        opcode: enc_ldr_sp(0, 0),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R0, [SP, #0] (MAX)".into(),
        opcode: enc_ldr_sp(0, 0),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xFFFF_FFFF),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [SP, #0] (MAX value)".into(),
        opcode: enc_str_sp(0, 0),
        reg_pre: vec![(0, 0xFFFF_FFFF), (13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R0, [SP, #100] (large offset)".into(),
        opcode: enc_str_sp(0, 25),
        reg_pre: vec![(0, 42), (13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(100),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R5, [SP, #12]".into(),
        opcode: enc_ldr_sp(5, 3),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(12, 0x5555_AAAA),
        ..TestCase::default()
    });

    // Additional SP-relative cases
    t.push(TestCase {
        name: "STR R3, [SP, #16]".into(),
        opcode: enc_str_sp(3, 4),
        reg_pre: vec![(3, 0xAAAA_BBBB), (13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(16),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R6, [SP, #20]".into(),
        opcode: enc_ldr_sp(6, 5),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(20, 0x1111_2222),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STR R2, [SP, #0] (alternating bits)".into(),
        opcode: enc_str_sp(2, 0),
        reg_pre: vec![(2, 0x5555_5555), (13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDR R4, [SP, #4] (alternating bits)".into(),
        opcode: enc_ldr_sp(4, 1),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(4, 0xAAAA_AAAA),
        ..TestCase::default()
    });

    t
}

/// ADR, ADD Rd, SP, #imm. Encoding: 1010x. ~10 tests.
fn gen_adr_add_sp() -> Vec<TestCase> {
    // Skipped: produces address-space-dependent result (see LLD Section 8.3)
    //
    // ADR Rd, #imm computes Align(PC,4) + imm into Rd. Since QEMU and
    // our emulator run at different PC addresses (0x100 vs 0x2000_0100),
    // the result in Rd differs and the absolute R0-R12 comparison fails.
    //
    // ADD Rd, SP, #imm computes SP + imm*4 into Rd. Without an explicit
    // SP precondition, SP defaults to TEST_STACK which differs per side
    // (0x0004_0000 vs 0x2004_0000), so the result in Rd differs.
    //
    // These instructions are still validated by the emulator's own unit
    // tests; they just can't be compared cross-environment via absolute
    // register values.

    Vec::new()
}

/// ADD/SUB SP, SXTH, SXTB, UXTH, UXTB, REV, REV16, REVSH. Encoding: 1011xxxx. ~20 tests.
fn gen_misc() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- ADD SP, SP, #imm7*4 ---
    t.push(TestCase {
        name: "ADD SP, SP, #16".into(),
        opcode: enc_add_sp_sp(4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD SP, SP, #0".into(),
        opcode: enc_add_sp_sp(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "ADD SP, SP, #508 (max)".into(),
        opcode: enc_add_sp_sp(127),
        ..TestCase::default()
    });

    // --- SUB SP, SP, #imm7*4 ---
    t.push(TestCase {
        name: "SUB SP, SP, #16".into(),
        opcode: enc_sub_sp_sp(4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUB SP, SP, #508 (max)".into(),
        opcode: enc_sub_sp_sp(127),
        ..TestCase::default()
    });

    // --- SXTH ---
    t.push(TestCase {
        name: "SXTH R0, R1 (positive)".into(),
        opcode: enc_sxth(0, 1),
        reg_pre: vec![(1, 0x7FFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTH R0, R1 (negative)".into(),
        opcode: enc_sxth(0, 1),
        reg_pre: vec![(1, 0x8000)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTH R0, R1 (upper bits ignored)".into(),
        opcode: enc_sxth(0, 1),
        reg_pre: vec![(1, 0xDEAD_0042)],
        ..TestCase::default()
    });

    // --- SXTB ---
    t.push(TestCase {
        name: "SXTB R0, R1 (positive)".into(),
        opcode: enc_sxtb(0, 1),
        reg_pre: vec![(1, 0x7F)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTB R0, R1 (negative)".into(),
        opcode: enc_sxtb(0, 1),
        reg_pre: vec![(1, 0x80)],
        ..TestCase::default()
    });

    // --- UXTH ---
    t.push(TestCase {
        name: "UXTH R0, R1".into(),
        opcode: enc_uxth(0, 1),
        reg_pre: vec![(1, 0xDEAD_BEEF)],
        ..TestCase::default()
    });

    // --- UXTB ---
    t.push(TestCase {
        name: "UXTB R0, R1".into(),
        opcode: enc_uxtb(0, 1),
        reg_pre: vec![(1, 0xDEAD_BEEF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "UXTB R0, R1 (0xFF)".into(),
        opcode: enc_uxtb(0, 1),
        reg_pre: vec![(1, 0xFF)],
        ..TestCase::default()
    });

    // --- REV ---
    t.push(TestCase {
        name: "REV R0, R1".into(),
        opcode: enc_rev(0, 1),
        reg_pre: vec![(1, 0x12_34_56_78)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REV R0, R1 (all same)".into(),
        opcode: enc_rev(0, 1),
        reg_pre: vec![(1, 0xAAAA_AAAA)],
        ..TestCase::default()
    });

    // --- REV16 ---
    t.push(TestCase {
        name: "REV16 R0, R1".into(),
        opcode: enc_rev16(0, 1),
        reg_pre: vec![(1, 0x1234_5678)],
        ..TestCase::default()
    });

    // --- REVSH ---
    t.push(TestCase {
        name: "REVSH R0, R1 (positive)".into(),
        opcode: enc_revsh(0, 1),
        reg_pre: vec![(1, 0x0001)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REVSH R0, R1 (negative, sign extend)".into(),
        opcode: enc_revsh(0, 1),
        reg_pre: vec![(1, 0x0080)], // swap -> 0x8000, sign extend -> 0xFFFF8000
        ..TestCase::default()
    });

    // Additional misc edge cases
    t.push(TestCase {
        name: "ADD SP, SP, #4".into(),
        opcode: enc_add_sp_sp(1),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SUB SP, SP, #4".into(),
        opcode: enc_sub_sp_sp(1),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTH R3, R4".into(),
        opcode: enc_sxth(3, 4),
        reg_pre: vec![(4, 0xFFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTH R0, R1 (zero)".into(),
        opcode: enc_sxth(0, 1),
        reg_pre: vec![(1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTB R2, R3 (zero)".into(),
        opcode: enc_sxtb(2, 3),
        reg_pre: vec![(3, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTB R0, R1 (0xFF = -1)".into(),
        opcode: enc_sxtb(0, 1),
        reg_pre: vec![(1, 0xFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "UXTH R2, R3 (zero)".into(),
        opcode: enc_uxth(2, 3),
        reg_pre: vec![(3, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "UXTB R3, R4 (0x00)".into(),
        opcode: enc_uxtb(3, 4),
        reg_pre: vec![(4, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REV R3, R4 (zero)".into(),
        opcode: enc_rev(3, 4),
        reg_pre: vec![(4, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REV R0, R1 (MAX)".into(),
        opcode: enc_rev(0, 1),
        reg_pre: vec![(1, 0xFFFF_FFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REV16 R0, R1 (zero)".into(),
        opcode: enc_rev16(0, 1),
        reg_pre: vec![(1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REV16 R3, R4 (alternating)".into(),
        opcode: enc_rev16(3, 4),
        reg_pre: vec![(4, 0xAAAA_5555)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REVSH R0, R1 (zero)".into(),
        opcode: enc_revsh(0, 1),
        reg_pre: vec![(1, 0)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "REVSH R0, R1 (0xFFFF)".into(),
        opcode: enc_revsh(0, 1),
        reg_pre: vec![(1, 0xFFFF)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "SXTH R5, R6 (alternating bits)".into(),
        opcode: enc_sxth(5, 6),
        reg_pre: vec![(6, 0x5555)],
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "UXTB R5, R6 (0xAA)".into(),
        opcode: enc_uxtb(5, 6),
        reg_pre: vec![(6, 0xFFFF_FFAA)],
        ..TestCase::default()
    });

    t
}

/// PUSH, POP. Encoding: 1011x10x. ~15 tests.
fn gen_push_pop() -> Vec<TestCase> {
    let mut t = Vec::new();

    // PUSH {R0}: SP -= 4, store R0
    t.push(TestCase {
        name: "PUSH {R0}".into(),
        opcode: enc_push(0x01, false),
        reg_pre: vec![(0, 0xDEAD_BEEF), (13, 16)], // SP starts at scratch+16
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(12), // SP decrements to scratch+12
        ..TestCase::default()
    });

    // PUSH {R0, R1}
    t.push(TestCase {
        name: "PUSH {R0, R1}".into(),
        opcode: enc_push(0x03, false),
        reg_pre: vec![(0, 0xAAAA), (1, 0xBBBB), (13, 16)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(8);  // R0 at scratch+8
            c.extend(mem_check_u32(12));   // R1 at scratch+12
            c
        },
        ..TestCase::default()
    });

    // PUSH {LR}
    t.push(TestCase {
        name: "PUSH {LR}".into(),
        opcode: enc_push(0x00, true),
        reg_pre: vec![(14, 0x0800_0101), (13, 16)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(12),
        ..TestCase::default()
    });

    // PUSH {R0, R1, LR}
    t.push(TestCase {
        name: "PUSH {R0, R1, LR}".into(),
        opcode: enc_push(0x03, true),
        reg_pre: vec![(0, 0x11), (1, 0x22), (14, 0x33), (13, 24)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(12); // R0 at scratch+12
            c.extend(mem_check_u32(16));   // R1 at scratch+16
            c.extend(mem_check_u32(20));   // LR at scratch+20
            c
        },
        ..TestCase::default()
    });

    // POP {R0}: load from [SP], SP += 4
    t.push(TestCase {
        name: "POP {R0}".into(),
        opcode: enc_pop(0x01, false),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xCAFE_BABE),
        ..TestCase::default()
    });

    // POP {R0, R1}
    t.push(TestCase {
        name: "POP {R0, R1}".into(),
        opcode: enc_pop(0x03, false),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0x1111);
            m.extend(mem_pre_u32(4, 0x2222));
            m
        },
        ..TestCase::default()
    });

    // Skipped: produces address-space-dependent result (see LLD Section 8.3)
    //
    // POP {PC} loads an absolute address from memory into PC. The stored
    // value (mem_pre) is raw bytes, not translated via addr_regs, so both
    // sides pop the same absolute address. But PC delta comparison uses
    // each side's TEST_SLOT as base, producing different deltas.

    // PUSH {R0-R7}
    t.push(TestCase {
        name: "PUSH {R0-R7}".into(),
        opcode: enc_push(0xFF, false),
        reg_pre: vec![
            (0, 0x00), (1, 0x11), (2, 0x22), (3, 0x33),
            (4, 0x44), (5, 0x55), (6, 0x66), (7, 0x77),
            (13, 64),
        ],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: {
            let mut c = Vec::new();
            for i in 0..8u32 {
                c.extend(mem_check_u32(32 + i * 4)); // scratch+32 .. scratch+60
            }
            c
        },
        ..TestCase::default()
    });

    // POP {R2, R3}
    t.push(TestCase {
        name: "POP {R2, R3}".into(),
        opcode: enc_pop(0x0C, false),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0xAAAA);
            m.extend(mem_pre_u32(4, 0xBBBB));
            m
        },
        ..TestCase::default()
    });

    // PUSH single then POP single (can't verify roundtrip in one step, but
    // each step is independently verified against QEMU)
    t.push(TestCase {
        name: "PUSH {R5}".into(),
        opcode: enc_push(0x20, false),
        reg_pre: vec![(5, 0x5555_5555), (13, 8)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(4),
        ..TestCase::default()
    });

    // PUSH {R0, LR} (mixed low + LR)
    t.push(TestCase {
        name: "PUSH {R0, LR}".into(),
        opcode: enc_push(0x01, true),
        reg_pre: vec![(0, 0xAA), (14, 0xBB), (13, 16)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(8);
            c.extend(mem_check_u32(12));
            c
        },
        ..TestCase::default()
    });

    // POP {R4, R5, R6, R7}
    t.push(TestCase {
        name: "POP {R4, R5, R6, R7}".into(),
        opcode: enc_pop(0xF0, false),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0x44);
            m.extend(mem_pre_u32(4, 0x55));
            m.extend(mem_pre_u32(8, 0x66));
            m.extend(mem_pre_u32(12, 0x77));
            m
        },
        ..TestCase::default()
    });

    // Additional push/pop cases
    t.push(TestCase {
        name: "PUSH {R2}".into(),
        opcode: enc_push(0x04, false),
        reg_pre: vec![(2, 0xBEEF_CAFE), (13, 8)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: mem_check_u32(4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "POP {R7}".into(),
        opcode: enc_pop(0x80, false),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0x7777_7777),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "PUSH {R3, R4, R5}".into(),
        opcode: enc_push(0x38, false),
        reg_pre: vec![(3, 0x33), (4, 0x44), (5, 0x55), (13, 24)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(12);
            c.extend(mem_check_u32(16));
            c.extend(mem_check_u32(20));
            c
        },
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "POP {R0, R1, R2, R3}".into(),
        opcode: enc_pop(0x0F, false),
        reg_pre: vec![(13, 0)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0x11);
            m.extend(mem_pre_u32(4, 0x22));
            m.extend(mem_pre_u32(8, 0x33));
            m.extend(mem_pre_u32(12, 0x44));
            m
        },
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "PUSH {R6, LR}".into(),
        opcode: enc_push(0x40, true),
        reg_pre: vec![(6, 0x66), (14, 0xAA), (13, 16)],
        addr_regs: vec![13],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(8);
            c.extend(mem_check_u32(12));
            c
        },
        ..TestCase::default()
    });

    t
}

/// STM, LDM. Encoding: 1100x. ~15 tests.
fn gen_stm_ldm() -> Vec<TestCase> {
    let mut t = Vec::new();

    // --- STM R4!, {R0, R1, R2} ---
    t.push(TestCase {
        name: "STM R4!, {R0, R1, R2}".into(),
        opcode: enc_stm(4, 0x07),
        reg_pre: vec![(4, 0), (0, 0x11), (1, 0x22), (2, 0x33)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(0);
            c.extend(mem_check_u32(4));
            c.extend(mem_check_u32(8));
            c
        },
        ..TestCase::default()
    });

    // STM R0!, {R1}
    t.push(TestCase {
        name: "STM R0!, {R1}".into(),
        opcode: enc_stm(0, 0x02),
        reg_pre: vec![(0, 0), (1, 0xDEAD_BEEF)],
        addr_regs: vec![0],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });

    // STM R3!, {R0-R2, R4-R7} — omit R3 from register list because STM
    // with Rn in the list stores the translated base address, which differs
    // between QEMU and emulator address spaces.
    t.push(TestCase {
        name: "STM R3!, {R0-R2, R4-R7}".into(),
        opcode: enc_stm(3, 0xF7), // bits 0-2 + 4-7 = 0b1111_0111
        reg_pre: vec![
            (3, 0), (0, 0x00), (1, 0x11), (2, 0x22),
            (4, 0x44), (5, 0x55), (6, 0x66), (7, 0x77),
        ],
        addr_regs: vec![3],
        needs_bus: true,
        mem_check: {
            let mut c = Vec::new();
            // 7 registers * 4 bytes each
            for i in 0..7u32 {
                c.extend(mem_check_u32(i * 4));
            }
            c
        },
        ..TestCase::default()
    });

    // STM with value edge case
    t.push(TestCase {
        name: "STM R4!, {R0} (MAX value)".into(),
        opcode: enc_stm(4, 0x01),
        reg_pre: vec![(4, 0), (0, 0xFFFF_FFFF)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });

    // --- LDM R5!, {R0, R1, R2} ---
    t.push(TestCase {
        name: "LDM R5!, {R0, R1, R2}".into(),
        opcode: enc_ldm(5, 0x07),
        reg_pre: vec![(5, 0)],
        addr_regs: vec![5],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0x11);
            m.extend(mem_pre_u32(4, 0x22));
            m.extend(mem_pre_u32(8, 0x33));
            m
        },
        ..TestCase::default()
    });

    // LDM R0!, {R1} (Rn not in reglist: writeback)
    t.push(TestCase {
        name: "LDM R0!, {R1}".into(),
        opcode: enc_ldm(0, 0x02),
        reg_pre: vec![(0, 0)],
        addr_regs: vec![0],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xCAFE),
        ..TestCase::default()
    });

    // LDM R0!, {R0, R1} (Rn in reglist: no writeback)
    t.push(TestCase {
        name: "LDM R0!, {R0, R1} (Rn in list)".into(),
        opcode: enc_ldm(0, 0x03),
        reg_pre: vec![(0, 0)],
        addr_regs: vec![0],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0xAA);
            m.extend(mem_pre_u32(4, 0xBB));
            m
        },
        ..TestCase::default()
    });

    // LDM R1!, {R0}
    t.push(TestCase {
        name: "LDM R1!, {R0}".into(),
        opcode: enc_ldm(1, 0x01),
        reg_pre: vec![(1, 0)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0xFFFF_FFFF),
        ..TestCase::default()
    });

    // STM R2!, {R0, R1} (field extraction)
    t.push(TestCase {
        name: "STM R2!, {R0, R1}".into(),
        opcode: enc_stm(2, 0x03),
        reg_pre: vec![(2, 0), (0, 0x5555), (1, 0xAAAA)],
        addr_regs: vec![2],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(0);
            c.extend(mem_check_u32(4));
            c
        },
        ..TestCase::default()
    });

    // LDM R7!, {R0-R6}
    t.push(TestCase {
        name: "LDM R7!, {R0-R6}".into(),
        opcode: enc_ldm(7, 0x7F),
        reg_pre: vec![(7, 0)],
        addr_regs: vec![7],
        needs_bus: true,
        mem_pre: {
            let mut m = Vec::new();
            for i in 0..7u32 {
                m.extend(mem_pre_u32(i * 4, 0x10 + i));
            }
            m
        },
        ..TestCase::default()
    });

    // LDM R4!, {R0} (zero value)
    t.push(TestCase {
        name: "LDM R4!, {R0} (zero)".into(),
        opcode: enc_ldm(4, 0x01),
        reg_pre: vec![(4, 0)],
        addr_regs: vec![4],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0),
        ..TestCase::default()
    });

    // STM R6!, {R5} (adjacent registers)
    t.push(TestCase {
        name: "STM R6!, {R5}".into(),
        opcode: enc_stm(6, 0x20),
        reg_pre: vec![(6, 0), (5, 0xBEEF_CAFE)],
        addr_regs: vec![6],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });

    // Additional STM/LDM cases
    t.push(TestCase {
        name: "STM R5!, {R0, R3}".into(),
        opcode: enc_stm(5, 0x09),
        reg_pre: vec![(5, 0), (0, 0xAA), (3, 0xBB)],
        addr_regs: vec![5],
        needs_bus: true,
        mem_check: {
            let mut c = mem_check_u32(0);
            c.extend(mem_check_u32(4));
            c
        },
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDM R3!, {R0, R2, R4}".into(),
        opcode: enc_ldm(3, 0x15),
        reg_pre: vec![(3, 0)],
        addr_regs: vec![3],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0x10);
            m.extend(mem_pre_u32(4, 0x20));
            m.extend(mem_pre_u32(8, 0x30));
            m
        },
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STM R7!, {R0} (MAX value)".into(),
        opcode: enc_stm(7, 0x01),
        reg_pre: vec![(7, 0), (0, 0xFFFF_FFFF)],
        addr_regs: vec![7],
        needs_bus: true,
        mem_check: mem_check_u32(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDM R6!, {R0, R1} (MAX values)".into(),
        opcode: enc_ldm(6, 0x03),
        reg_pre: vec![(6, 0)],
        addr_regs: vec![6],
        needs_bus: true,
        mem_pre: {
            let mut m = mem_pre_u32(0, 0xFFFF_FFFF);
            m.extend(mem_pre_u32(4, 0xFFFF_FFFF));
            m
        },
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "STM R1!, {R0, R2, R4, R6}".into(),
        opcode: enc_stm(1, 0x55),
        reg_pre: vec![(1, 0), (0, 0x00), (2, 0x22), (4, 0x44), (6, 0x66)],
        addr_regs: vec![1],
        needs_bus: true,
        mem_check: {
            let mut c = Vec::new();
            for i in 0..4u32 {
                c.extend(mem_check_u32(i * 4));
            }
            c
        },
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "LDM R2!, {R0} (alternating bits)".into(),
        opcode: enc_ldm(2, 0x01),
        reg_pre: vec![(2, 0)],
        addr_regs: vec![2],
        needs_bus: true,
        mem_pre: mem_pre_u32(0, 0x5555_AAAA),
        ..TestCase::default()
    });

    t
}

/// B<cond>. Encoding: 1101. ~20 tests.
fn gen_branch_cond() -> Vec<TestCase> {
    let mut t = Vec::new();

    // Test each condition code with appropriate flags.
    // Condition codes: EQ(0), NE(1), CS(2), CC(3), MI(4), PL(5),
    //                  VS(6), VC(7), HI(8), LS(9), GE(10), LT(11),
    //                  GT(12), LE(13)

    let z = 1u32 << 30;
    let c = 1u32 << 29;
    let n = 1u32 << 31;
    let v = 1u32 << 28;
    let tb = 0x0100_0000u32; // T bit (always set)

    // BEQ (cond=0): taken when Z=1
    t.push(TestCase {
        name: "BEQ +6 (taken, Z=1)".into(),
        opcode: enc_branch_cond(0, 6),
        xpsr_pre: tb | z,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BEQ +6 (not taken, Z=0)".into(),
        opcode: enc_branch_cond(0, 6),
        xpsr_pre: tb,
        ..TestCase::default()
    });

    // BNE (cond=1): taken when Z=0
    t.push(TestCase {
        name: "BNE +6 (taken, Z=0)".into(),
        opcode: enc_branch_cond(1, 6),
        xpsr_pre: tb,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BNE +6 (not taken, Z=1)".into(),
        opcode: enc_branch_cond(1, 6),
        xpsr_pre: tb | z,
        ..TestCase::default()
    });

    // BCS/BHS (cond=2): taken when C=1
    t.push(TestCase {
        name: "BCS +10 (taken, C=1)".into(),
        opcode: enc_branch_cond(2, 10),
        xpsr_pre: tb | c,
        ..TestCase::default()
    });

    // BCC/BLO (cond=3): taken when C=0
    t.push(TestCase {
        name: "BCC +10 (taken, C=0)".into(),
        opcode: enc_branch_cond(3, 10),
        xpsr_pre: tb,
        ..TestCase::default()
    });

    // BMI (cond=4): taken when N=1
    t.push(TestCase {
        name: "BMI +8 (taken, N=1)".into(),
        opcode: enc_branch_cond(4, 8),
        xpsr_pre: tb | n,
        ..TestCase::default()
    });

    // BPL (cond=5): taken when N=0
    t.push(TestCase {
        name: "BPL +8 (taken, N=0)".into(),
        opcode: enc_branch_cond(5, 8),
        xpsr_pre: tb,
        ..TestCase::default()
    });

    // BVS (cond=6): taken when V=1
    t.push(TestCase {
        name: "BVS +4 (taken, V=1)".into(),
        opcode: enc_branch_cond(6, 4),
        xpsr_pre: tb | v,
        ..TestCase::default()
    });

    // BVC (cond=7): taken when V=0
    t.push(TestCase {
        name: "BVC +4 (taken, V=0)".into(),
        opcode: enc_branch_cond(7, 4),
        xpsr_pre: tb,
        ..TestCase::default()
    });

    // BHI (cond=8): taken when C=1 AND Z=0
    t.push(TestCase {
        name: "BHI +6 (taken, C=1 Z=0)".into(),
        opcode: enc_branch_cond(8, 6),
        xpsr_pre: tb | c,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BHI +6 (not taken, C=1 Z=1)".into(),
        opcode: enc_branch_cond(8, 6),
        xpsr_pre: tb | c | z,
        ..TestCase::default()
    });

    // BLS (cond=9): taken when C=0 OR Z=1
    t.push(TestCase {
        name: "BLS +6 (taken, Z=1)".into(),
        opcode: enc_branch_cond(9, 6),
        xpsr_pre: tb | z,
        ..TestCase::default()
    });

    // BGE (cond=10): taken when N==V
    t.push(TestCase {
        name: "BGE +6 (taken, N=0 V=0)".into(),
        opcode: enc_branch_cond(10, 6),
        xpsr_pre: tb,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BGE +6 (taken, N=1 V=1)".into(),
        opcode: enc_branch_cond(10, 6),
        xpsr_pre: tb | n | v,
        ..TestCase::default()
    });

    // BLT (cond=11): taken when N!=V
    t.push(TestCase {
        name: "BLT +6 (taken, N=1 V=0)".into(),
        opcode: enc_branch_cond(11, 6),
        xpsr_pre: tb | n,
        ..TestCase::default()
    });

    // BGT (cond=12): taken when Z=0 AND N==V
    t.push(TestCase {
        name: "BGT +6 (taken, Z=0 N=0 V=0)".into(),
        opcode: enc_branch_cond(12, 6),
        xpsr_pre: tb,
        ..TestCase::default()
    });

    // BLE (cond=13): taken when Z=1 OR N!=V
    t.push(TestCase {
        name: "BLE +6 (taken, Z=1)".into(),
        opcode: enc_branch_cond(13, 6),
        xpsr_pre: tb | z,
        ..TestCase::default()
    });

    // Backward branches
    t.push(TestCase {
        name: "BEQ -4 (backward, taken)".into(),
        opcode: enc_branch_cond(0, -4),
        xpsr_pre: tb | z,
        ..TestCase::default()
    });

    // Not-taken cases for remaining condition codes
    t.push(TestCase {
        name: "BCS +10 (not taken, C=0)".into(),
        opcode: enc_branch_cond(2, 10),
        xpsr_pre: tb,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BCC +10 (not taken, C=1)".into(),
        opcode: enc_branch_cond(3, 10),
        xpsr_pre: tb | c,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BMI +8 (not taken, N=0)".into(),
        opcode: enc_branch_cond(4, 8),
        xpsr_pre: tb,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BPL +8 (not taken, N=1)".into(),
        opcode: enc_branch_cond(5, 8),
        xpsr_pre: tb | n,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BVS +4 (not taken, V=0)".into(),
        opcode: enc_branch_cond(6, 4),
        xpsr_pre: tb,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BVC +4 (not taken, V=1)".into(),
        opcode: enc_branch_cond(7, 4),
        xpsr_pre: tb | v,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BGE +6 (not taken, N=1 V=0)".into(),
        opcode: enc_branch_cond(10, 6),
        xpsr_pre: tb | n,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BLT +6 (not taken, N=0 V=0)".into(),
        opcode: enc_branch_cond(11, 6),
        xpsr_pre: tb,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BGT +6 (not taken, Z=1)".into(),
        opcode: enc_branch_cond(12, 6),
        xpsr_pre: tb | z,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BLE +6 (not taken, Z=0 N=V=0)".into(),
        opcode: enc_branch_cond(13, 6),
        xpsr_pre: tb,
        ..TestCase::default()
    });
    // Large offsets
    t.push(TestCase {
        name: "BEQ +254 (max forward)".into(),
        opcode: enc_branch_cond(0, 254),
        xpsr_pre: tb | z,
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "BNE -256 (max backward)".into(),
        opcode: enc_branch_cond(1, -256),
        xpsr_pre: tb,
        ..TestCase::default()
    });

    t
}

/// B (unconditional). Encoding: 11100. ~10 tests.
fn gen_branch_uncond() -> Vec<TestCase> {
    let mut t = Vec::new();

    t.push(TestCase {
        name: "B +8 (forward)".into(),
        opcode: enc_branch_uncond(8),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +0 (self, offset=0)".into(),
        opcode: enc_branch_uncond(0),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B -4 (backward, loops to self)".into(),
        opcode: enc_branch_uncond(-4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +100 (large forward)".into(),
        opcode: enc_branch_uncond(100),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B -100 (large backward)".into(),
        opcode: enc_branch_uncond(-100),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +2 (minimal forward)".into(),
        opcode: enc_branch_uncond(2),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B -2 (minimal backward)".into(),
        opcode: enc_branch_uncond(-2),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +2046 (near max forward)".into(),
        opcode: enc_branch_uncond(2046),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B -2048 (max backward)".into(),
        opcode: enc_branch_uncond(-2048),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +1000 (medium forward)".into(),
        opcode: enc_branch_uncond(1000),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B -1000 (medium backward)".into(),
        opcode: enc_branch_uncond(-1000),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +500".into(),
        opcode: enc_branch_uncond(500),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B -500".into(),
        opcode: enc_branch_uncond(-500),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +4".into(),
        opcode: enc_branch_uncond(4),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B +6".into(),
        opcode: enc_branch_uncond(6),
        ..TestCase::default()
    });
    t.push(TestCase {
        name: "B -6".into(),
        opcode: enc_branch_uncond(-6),
        ..TestCase::default()
    });

    t
}

/// Generate all Thumb-16 test cases.
pub fn generate_all() -> Vec<TestCase> {
    let mut all = Vec::new();
    all.extend(gen_shift_imm());
    all.extend(gen_add_sub_reg());
    all.extend(gen_mov_cmp_imm8());
    all.extend(gen_data_proc_reg());
    all.extend(gen_special_data_bx());
    all.extend(gen_load_store_reg());
    all.extend(gen_load_store_imm());
    all.extend(gen_load_store_sp());
    all.extend(gen_adr_add_sp());
    all.extend(gen_misc());
    all.extend(gen_push_pop());
    all.extend(gen_stm_ldm());
    all.extend(gen_branch_cond());
    all.extend(gen_branch_uncond());
    all
}

// ============================================================================
// Fuzz test generators — random inputs for each instruction class
// ============================================================================

/// Generate random ALU (non-bus) fuzz tests. Fast — no memory setup needed.
fn generate_fuzz_alu(count: usize, rng: &mut StdRng) -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32; // T bit

    // Helper: random xPSR flags (N, Z, C, V in bits 31:28) with T bit
    let rand_flags = |rng: &mut StdRng| -> u32 {
        let flags: u32 = rng.range(0..16);
        tb | (flags << 28)
    };

    // Helper: random register values for all 8 low registers
    let rand_low_regs = |rng: &mut StdRng| -> Vec<(u8, u32)> {
        (0..8).map(|i| (i, rng.random())).collect()
    };

    // --- Shifts (LSL/LSR/ASR immediate) ---
    for i in 0..count {
        let rd: u16 = rng.range(0..8);
        let rm: u16 = rng.range(0..8);
        let variant = rng.range(0..3u8);
        let (name_prefix, opcode, imm_desc) = match variant {
            0 => {
                let imm5: u16 = rng.range(0..32);
                ("LSL", enc_lsl_imm(rd, rm, imm5), imm5)
            }
            1 => {
                // LSR: imm5=0 encodes shift-by-32, valid range 0-31 in encoding
                let imm5: u16 = rng.range(0..32);
                ("LSR", enc_lsr_imm(rd, rm, imm5), imm5)
            }
            _ => {
                let imm5: u16 = rng.range(0..32);
                ("ASR", enc_asr_imm(rd, rm, imm5), imm5)
            }
        };
        let mut regs = rand_low_regs(rng);
        // Ensure rm has a random value (already covered by rand_low_regs)
        t.push(TestCase {
            name: format!("FUZZ:SHIFT:{i} {name_prefix} R{rd},R{rm},#{imm_desc}"),
            opcode,
            reg_pre: regs.drain(..).collect(),
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Add/Sub register + 3-bit immediate ---
    for i in 0..count {
        let rd: u16 = rng.range(0..8);
        let rn: u16 = rng.range(0..8);
        let variant = rng.range(0..4u8);
        let (name_prefix, opcode) = match variant {
            0 => {
                let rm: u16 = rng.range(0..8);
                ("ADDS_R", enc_adds_reg(rd, rn, rm))
            }
            1 => {
                let rm: u16 = rng.range(0..8);
                ("SUBS_R", enc_subs_reg(rd, rn, rm))
            }
            2 => {
                let imm3: u16 = rng.range(0..8);
                ("ADDS_I3", enc_adds_imm3(rd, rn, imm3))
            }
            _ => {
                let imm3: u16 = rng.range(0..8);
                ("SUBS_I3", enc_subs_imm3(rd, rn, imm3))
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:ADDSUB:{i} {name_prefix}"),
            opcode,
            reg_pre: rand_low_regs(rng),
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Mov/Cmp/Add/Sub 8-bit immediate ---
    for i in 0..count {
        let rd: u16 = rng.range(0..8);
        let imm8: u16 = rng.range(0..256);
        let variant = rng.range(0..4u8);
        let (name_prefix, opcode) = match variant {
            0 => ("MOVS_I8", enc_movs_imm(rd, imm8)),
            1 => ("CMP_I8", enc_cmp_imm(rd, imm8)),
            2 => ("ADDS_I8", enc_adds_imm8(rd, imm8)),
            _ => ("SUBS_I8", enc_subs_imm8(rd, imm8)),
        };
        t.push(TestCase {
            name: format!("FUZZ:IMM8:{i} {name_prefix}"),
            opcode,
            reg_pre: rand_low_regs(rng),
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Data processing (register) ---
    for i in 0..count {
        let rdn: u16 = rng.range(0..8);
        let rm: u16 = rng.range(0..8);
        let op: u16 = rng.range(0..16);
        let opcode = enc_data_proc(op, rm, rdn);
        // MUL (op=13): C and V are UNPREDICTABLE
        let xpsr_mask = if op == 13 { MASK_NZ_ONLY } else { MASK_ALL_FLAGS };
        t.push(TestCase {
            name: format!("FUZZ:DPROC:{i} op={op}"),
            opcode,
            reg_pre: rand_low_regs(rng),
            xpsr_pre: rand_flags(rng),
            xpsr_mask,
            ..TestCase::default()
        });
    }

    // --- Special data (MOV/ADD high registers) ---
    for i in 0..count {
        let rd: u16 = rng.range(0..12); // avoid SP(13), LR(14), PC(15)
        let rm: u16 = rng.range(0..12);
        let variant = rng.range(0..2u8);
        let (name_prefix, opcode) = match variant {
            0 => ("MOV_HI", enc_mov_high(rd, rm)),
            _ => ("ADD_HI", enc_add_high(rd, rm)),
        };
        // Set all GP regs (0-12) to random values to catch clobbering
        let regs: Vec<(u8, u32)> = (0..=12).map(|r| (r, rng.random())).collect();
        t.push(TestCase {
            name: format!("FUZZ:SPECIAL:{i} {name_prefix} R{rd},R{rm}"),
            opcode,
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        });
    }

    // --- Misc (SXTH/SXTB/UXTH/UXTB/REV/REV16/REVSH, ADD/SUB SP) ---
    for i in 0..count {
        let rd: u16 = rng.range(0..8);
        let rm: u16 = rng.range(0..8);
        let variant = rng.range(0..9u8);
        let (name_prefix, opcode, regs) = match variant {
            0 => ("SXTH", enc_sxth(rd, rm), rand_low_regs(rng)),
            1 => ("SXTB", enc_sxtb(rd, rm), rand_low_regs(rng)),
            2 => ("UXTH", enc_uxth(rd, rm), rand_low_regs(rng)),
            3 => ("UXTB", enc_uxtb(rd, rm), rand_low_regs(rng)),
            4 => ("REV", enc_rev(rd, rm), rand_low_regs(rng)),
            5 => ("REV16", enc_rev16(rd, rm), rand_low_regs(rng)),
            6 => ("REVSH", enc_revsh(rd, rm), rand_low_regs(rng)),
            7 => {
                let imm7: u16 = rng.range(0..128);
                ("ADD_SP", enc_add_sp_sp(imm7), Vec::new())
            }
            _ => {
                let imm7: u16 = rng.range(0..128);
                ("SUB_SP", enc_sub_sp_sp(imm7), Vec::new())
            }
        };
        t.push(TestCase {
            name: format!("FUZZ:MISC:{i} {name_prefix}"),
            opcode,
            reg_pre: regs,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Conditional branches ---
    for i in 0..count {
        let cond: u16 = rng.range(0..14); // 0-13, excluding 14 (UND) and 15 (SVC)
        // Safe offset range: -128..+126, must be even
        let half: i16 = rng.range(-64..64);
        let offset_bytes: i16 = half * 2; // always even
        let opcode = enc_branch_cond(cond, offset_bytes);
        t.push(TestCase {
            name: format!("FUZZ:BCOND:{i} cond={cond} off={offset_bytes}"),
            opcode,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Unconditional branches ---
    for i in 0..count {
        // Safe offset range: -2048..+2046, must be even
        let half: i32 = rng.range(-1024..1024);
        let offset_bytes: i32 = half * 2; // always even
        let opcode = enc_branch_uncond(offset_bytes);
        t.push(TestCase {
            name: format!("FUZZ:BUNCOND:{i} off={offset_bytes}"),
            opcode,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    t
}

/// Generate random memory (bus) fuzz tests. Slower — needs memory setup.
fn generate_fuzz_mem(count: usize, rng: &mut StdRng) -> Vec<TestCase> {
    let mut t = Vec::new();
    let tb = 0x0100_0000u32;

    let rand_flags = |rng: &mut StdRng| -> u32 {
        let flags: u32 = rng.range(0..16);
        tb | (flags << 28)
    };

    // --- Load/store register offset ---
    for i in 0..count {
        // Ensure rt, rn, rm are all distinct to avoid register aliasing
        // (e.g., STR R4, [R1, R4] would clobber the offset with data).
        let rt: u16 = rng.range(0..8);
        let rn: u16 = loop {
            let r = rng.range(0..8);
            if r != rt { break r; }
        };
        let rm: u16 = loop {
            let r = rng.range(0..8);
            if r != rn && r != rt { break r; }
        };
        // Offset must be word-aligned for word ops, half-aligned for half ops.
        // Use small offset to stay in scratch area (256 bytes).
        let opc: u16 = rng.range(0..7); // 0-6: STR, STRH, STRB, LDRSB, LDR, LDRH, LDRSH
        let (offset, data_val): (u32, u32) = match opc {
            0 | 4 => {
                // Word: 4-byte aligned, max offset ~240
                let off = (rng.range(0..60u32)) * 4;
                (off, rng.random())
            }
            1 | 5 | 6 => {
                // Half: 2-byte aligned
                let off = (rng.range(0..120u32)) * 2;
                (off, rng.random::<u32>() & 0xFFFF)
            }
            _ => {
                // Byte
                let off = rng.range(0..240u32);
                (off, rng.random::<u32>() & 0xFF)
            }
        };

        let is_store = matches!(opc, 0 | 1 | 2);
        let mut reg_pre: Vec<(u8, u32)> = Vec::new();
        // Set all low regs to random values
        for r in 0..8u8 {
            reg_pre.push((r, rng.random()));
        }
        // Override base and offset regs
        let rn8 = rn as u8;
        let rm8 = rm as u8;
        reg_pre.retain(|&(r, _)| r != rn8 && r != rm8);
        reg_pre.push((rn8, 0)); // base = 0 (addr_regs translates to scratch)
        reg_pre.push((rm8, offset));
        if is_store {
            // Override rt with data to store
            reg_pre.retain(|&(r, _)| r != rt as u8);
            reg_pre.push((rt as u8, data_val));
        }

        let mut mem_pre = Vec::new();
        let mut mem_check = Vec::new();
        if is_store {
            match opc {
                0 => mem_check = mem_check_u32(offset),
                1 => mem_check = mem_check_u16(offset),
                2 => mem_check = vec![offset],
                _ => {}
            }
        } else {
            match opc {
                4 => mem_pre = mem_pre_u32(offset, data_val),
                5 | 6 => mem_pre = mem_pre_u16(offset, data_val as u16),
                3 => mem_pre = vec![(offset, data_val as u8)], // LDRSB
                _ => {}
            }
        }

        t.push(TestCase {
            name: format!("FUZZ:LSREG:{i} opc={opc}"),
            opcode: enc_ls_reg(opc, rm, rn, rt),
            reg_pre,
            addr_regs: vec![rn as u8],
            needs_bus: true,
            mem_pre,
            mem_check,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Load/store immediate offset ---
    for i in 0..count {
        let rt: u16 = rng.range(0..8);
        // Ensure rt != rn so store data doesn't clobber base address
        let rn: u16 = loop {
            let r = rng.range(0..8);
            if r != rt { break r; }
        };
        let variant = rng.range(0..6u8);
        let data_val: u32 = rng.random();

        let (name_prefix, opcode, mem_pre, mem_check, imm_offset) = match variant {
            0 => {
                // STR [Rn, #imm5*4]: offset = imm5*4, max imm5=31 -> 124
                // But keep within 240 bytes of scratch
                let imm5: u16 = rng.range(0..32);
                let off = imm5 as u32 * 4;
                ("STR_I", enc_str_imm(rt, rn, imm5), Vec::new(), mem_check_u32(off), off)
            }
            1 => {
                // LDR [Rn, #imm5*4]
                let imm5: u16 = rng.range(0..32);
                let off = imm5 as u32 * 4;
                ("LDR_I", enc_ldr_imm(rt, rn, imm5), mem_pre_u32(off, data_val), Vec::new(), off)
            }
            2 => {
                // STRB [Rn, #imm5]: offset = imm5
                let imm5: u16 = rng.range(0..32);
                let off = imm5 as u32;
                ("STRB_I", enc_strb_imm(rt, rn, imm5), Vec::new(), vec![off], off)
            }
            3 => {
                // LDRB [Rn, #imm5]
                let imm5: u16 = rng.range(0..32);
                let off = imm5 as u32;
                ("LDRB_I", enc_ldrb_imm(rt, rn, imm5), vec![(off, data_val as u8)], Vec::new(), off)
            }
            4 => {
                // STRH [Rn, #imm5*2]: offset = imm5*2
                let imm5: u16 = rng.range(0..32);
                let off = imm5 as u32 * 2;
                ("STRH_I", enc_strh_imm(rt, rn, imm5), Vec::new(), mem_check_u16(off), off)
            }
            _ => {
                // LDRH [Rn, #imm5*2]
                let imm5: u16 = rng.range(0..32);
                let off = imm5 as u32 * 2;
                ("LDRH_I", enc_ldrh_imm(rt, rn, imm5), mem_pre_u16(off, data_val as u16), Vec::new(), off)
            }
        };

        let is_store = matches!(variant, 0 | 2 | 4);
        let mut reg_pre: Vec<(u8, u32)> = Vec::new();
        for r in 0..8u8 {
            reg_pre.push((r, rng.random()));
        }
        reg_pre.retain(|&(r, _)| r != rn as u8);
        reg_pre.push((rn as u8, 0)); // base at scratch start
        if is_store {
            reg_pre.retain(|&(r, _)| r != rt as u8);
            reg_pre.push((rt as u8, data_val));
        }

        let _ = imm_offset; // used in offset calculation above
        t.push(TestCase {
            name: format!("FUZZ:LSIMM:{i} {name_prefix}"),
            opcode,
            reg_pre,
            addr_regs: vec![rn as u8],
            needs_bus: true,
            mem_pre,
            mem_check,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    // --- Push/Pop ---
    for i in 0..count {
        let variant = rng.range(0..2u8);
        match variant {
            0 => {
                // PUSH: random register list (at least 1 bit set)
                let reglist8: u16 = rng.range(1..256);
                let lr = rng.coin(0.3);
                let opcode = enc_push(reglist8, lr);

                let reg_count = reglist8.count_ones() + if lr { 1 } else { 0 };
                let sp_start = reg_count * 4; // SP starts high enough to push down
                let mut reg_pre: Vec<(u8, u32)> = Vec::new();
                for r in 0..8u8 {
                    reg_pre.push((r, rng.random()));
                }
                if lr {
                    reg_pre.push((14, rng.random()));
                }
                reg_pre.push((13, sp_start));

                // After push, check memory starting at scratch+0 (SP decremented)
                let mut mem_check = Vec::new();
                for word in 0..reg_count {
                    mem_check.extend(mem_check_u32(word * 4));
                }

                t.push(TestCase {
                    name: format!("FUZZ:PUSH:{i} list={reglist8:#05x} lr={lr}"),
                    opcode,
                    reg_pre,
                    addr_regs: vec![13],
                    needs_bus: true,
                    mem_check,
                    xpsr_pre: rand_flags(rng),
                    ..TestCase::default()
                });
            }
            _ => {
                // POP: random register list (at least 1 bit set), no PC (address-space-dependent)
                let reglist8: u16 = rng.range(1..256);
                let opcode = enc_pop(reglist8, false);

                let reg_count = reglist8.count_ones();
                // Set up memory with random values at scratch+0..
                let mut mem_pre = Vec::new();
                for word in 0..reg_count {
                    mem_pre.extend(mem_pre_u32(word * 4, rng.random()));
                }

                t.push(TestCase {
                    name: format!("FUZZ:POP:{i} list={reglist8:#05x}"),
                    opcode,
                    reg_pre: vec![(13, 0)], // SP at scratch base
                    addr_regs: vec![13],
                    needs_bus: true,
                    mem_pre,
                    xpsr_pre: rand_flags(rng),
                    ..TestCase::default()
                });
            }
        }
    }

    // --- STM/LDM ---
    for i in 0..count {
        let variant = rng.range(0..2u8);
        // Use a base register that's NOT in reglist to avoid address-space issues.
        // We'll use register rn for the base, and only include other regs in the list.
        match variant {
            0 => {
                // STM Rn!, {reglist}
                let rn: u16 = rng.range(0..8);
                // Build reglist excluding rn (to avoid storing the address-translated value)
                let mut reglist8: u16 = rng.range(1..256);
                reglist8 &= !(1 << rn); // clear rn from list
                if reglist8 == 0 { reglist8 = 1 << ((rn + 1) % 8); } // ensure at least 1

                let opcode = enc_stm(rn, reglist8);
                let reg_count = reglist8.count_ones();

                let mut reg_pre: Vec<(u8, u32)> = Vec::new();
                for r in 0..8u8 {
                    if r == rn as u8 {
                        reg_pre.push((r, 0)); // base at scratch start
                    } else {
                        reg_pre.push((r, rng.random()));
                    }
                }

                let mut mem_check = Vec::new();
                for word in 0..reg_count {
                    mem_check.extend(mem_check_u32(word * 4));
                }

                t.push(TestCase {
                    name: format!("FUZZ:STM:{i} R{rn}! list={reglist8:#05x}"),
                    opcode,
                    reg_pre,
                    addr_regs: vec![rn as u8],
                    needs_bus: true,
                    mem_check,
                    xpsr_pre: rand_flags(rng),
                    ..TestCase::default()
                });
            }
            _ => {
                // LDM Rn!, {reglist}
                let rn: u16 = rng.range(0..8);
                let mut reglist8: u16 = rng.range(1..256);
                // Keep rn in list sometimes (no writeback) for variety
                if rng.coin(0.5) {
                    reglist8 &= !(1 << rn);
                    if reglist8 == 0 { reglist8 = 1 << ((rn + 1) % 8); }
                }

                let opcode = enc_ldm(rn, reglist8);
                let reg_count = reglist8.count_ones();

                let mut mem_pre = Vec::new();
                for word in 0..reg_count {
                    mem_pre.extend(mem_pre_u32(word * 4, rng.random()));
                }

                t.push(TestCase {
                    name: format!("FUZZ:LDM:{i} R{rn}! list={reglist8:#05x}"),
                    opcode,
                    reg_pre: vec![(rn as u8, 0)],
                    addr_regs: vec![rn as u8],
                    needs_bus: true,
                    mem_pre,
                    xpsr_pre: rand_flags(rng),
                    ..TestCase::default()
                });
            }
        }
    }

    // --- Load/store SP-relative ---
    for i in 0..count {
        let rt: u16 = rng.range(0..8);
        // Keep imm8 small so offset stays within 256-byte scratch
        let imm8: u16 = rng.range(0..16); // offset = imm8 * 4, max 60
        let variant = rng.range(0..2u8);
        let data_val: u32 = rng.random();

        let (name_prefix, opcode, mem_pre, mem_check) = match variant {
            0 => {
                let off = imm8 as u32 * 4;
                ("STR_SP", enc_str_sp(rt, imm8), Vec::new(), mem_check_u32(off))
            }
            _ => {
                let off = imm8 as u32 * 4;
                ("LDR_SP", enc_ldr_sp(rt, imm8), mem_pre_u32(off, data_val), Vec::new())
            }
        };

        let is_store = variant == 0;
        let mut reg_pre: Vec<(u8, u32)> = Vec::new();
        for r in 0..8u8 {
            reg_pre.push((r, rng.random()));
        }
        reg_pre.push((13, 0)); // SP at scratch base
        if is_store {
            reg_pre.retain(|&(r, _)| r != rt as u8);
            reg_pre.push((rt as u8, data_val));
        }

        t.push(TestCase {
            name: format!("FUZZ:LSSP:{i} {name_prefix}"),
            opcode,
            reg_pre,
            addr_regs: vec![13],
            needs_bus: true,
            mem_pre,
            mem_check,
            xpsr_pre: rand_flags(rng),
            ..TestCase::default()
        });
    }

    t
}

/// Generate fuzz tests: random register values, random encodings.
///
/// `count_per_class` tests are generated for each instruction class.
/// `seed` makes the output reproducible.
///
/// Returns (alu_tests, mem_tests) so the runner can prioritize differently.
pub fn generate_fuzz(count_per_class: usize, seed: u64) -> (Vec<TestCase>, Vec<TestCase>) {
    let mut rng = StdRng::seed_from_u64(seed);
    let alu = generate_fuzz_alu(count_per_class, &mut rng);
    let mem = generate_fuzz_mem(count_per_class, &mut rng);
    (alu, mem)
}

// ============================================================================
// Run state — snapshot of CPU + memory after execution
// ============================================================================

/// Post-execution state snapshot for comparison between QEMU and emulator.
pub struct RunState {
    /// R0-R15 register values.
    pub regs: [u32; 16],
    /// xPSR value.
    pub xpsr: u32,
    /// Bytes at mem_check offsets (in order of tc.mem_check).
    pub mem: Vec<u8>,
    /// Cycle count from execution. DWT CYCCNT for probe, execute_one return
    /// value for emulator, 0 for QEMU (which doesn't report cycles).
    pub cycles: u32,
}

// ============================================================================
// Address translation
// ============================================================================

/// Translate a register value if it's an address register.
///
/// Registers listed in `tc.addr_regs` contain offsets from the scratch area.
/// This adds the per-side scratch base to make them absolute addresses.
pub fn setup_reg(reg: u8, val: u32, tc: &TestCase, scratch_base: u32) -> u32 {
    if tc.addr_regs.contains(&reg) {
        scratch_base.wrapping_add(val)
    } else {
        val
    }
}

// ============================================================================
// Emulator-side execution
// ============================================================================

/// Run a single test case on the emulator. Returns post-execution state.
///
/// Uses the provided `shared_bus` for memory-accessing instructions (reused
/// across tests to avoid repeated 552KB allocations).
pub fn run_one_emu(tc: &TestCase, shared_bus: &mut Bus) -> RunState {
    debug_assert!(
        tc.hw1.is_none() || tc.opcode >= 0xE800,
        "Thumb-32 test has hw1 but opcode {:#06x} < 0xE800", tc.opcode
    );

    let mut core = CortexM33::new();

    // Set defaults: R0-R12 = 0, SP = stack, LR = sentinel, PC = slot
    for i in 0..=12 {
        core.set_reg(i, 0);
    }
    core.set_reg(13, EMU_TEST_STACK);
    core.set_reg(14, 0xFFFF_FFFF);
    core.regs.set_pc(EMU_TEST_SLOT);
    core.regs.xpsr = tc.xpsr_pre;

    // Apply register preconditions with address translation
    for &(reg, val) in &tc.reg_pre {
        let val = setup_reg(reg, val, tc, EMU_TEST_SCRATCH);
        core.set_reg(reg as usize, val);
    }

    // Execute
    if tc.needs_bus {
        for i in 0..SCRATCH_SIZE {
            shared_bus.write8(EMU_TEST_SCRATCH + i, 0);
        }
        for &(offset, val) in &tc.mem_pre {
            shared_bus.write8(EMU_TEST_SCRATCH + offset, val);
        }
    }
    let cycles = match tc.hw1 {
        None => if tc.needs_bus {
            core.execute_one_with_bus(tc.opcode, shared_bus)
        } else {
            core.execute_one(tc.opcode)
        },
        Some(hw1) => if tc.needs_bus {
            core.execute_one_wide_with_bus(tc.opcode, hw1, shared_bus)
        } else {
            core.execute_one_wide(tc.opcode, hw1)
        },
    };

    // Collect post-state
    let mut regs = [0u32; 16];
    for i in 0..16 {
        regs[i] = core.reg(i);
    }
    let xpsr = core.regs.xpsr;
    let mem: Vec<u8> = tc
        .mem_check
        .iter()
        .map(|&offset| shared_bus.read8(EMU_TEST_SCRATCH + offset))
        .collect();

    RunState { regs, xpsr, mem, cycles }
}

// ============================================================================
// Comparison logic
// ============================================================================

/// Compare QEMU and emulator post-execution states.
///
/// Returns `Ok(())` if they match, or `Err(description)` listing all
/// mismatches. This is a pure function — all I/O is done before calling it.
pub fn compare(tc: &TestCase, qemu: &RunState, emu: &RunState) -> Result<(), String> {
    let mut diffs = Vec::new();

    // R0-R12: absolute comparison.
    // Skip registers in addr_regs — they were intentionally set to different
    // absolute values per side (QEMU_SCRATCH vs EMU_SCRATCH).
    for i in 0..=12 {
        if tc.addr_regs.contains(&(i as u8)) {
            continue;
        }
        if qemu.regs[i] != emu.regs[i] {
            diffs.push(format!(
                "R{i}: QEMU={:#010x} EMU={:#010x}",
                qemu.regs[i], emu.regs[i]
            ));
        }
    }

    // SP (R13): relative delta comparison.
    // Base depends on whether SP was set via addr_regs (scratch) or default (stack).
    let (qemu_sp_base, emu_sp_base) = if tc.addr_regs.contains(&13) {
        (QEMU_TEST_SCRATCH, EMU_TEST_SCRATCH)
    } else {
        (QEMU_TEST_STACK, EMU_TEST_STACK)
    };
    let qemu_sp_delta = qemu.regs[13].wrapping_sub(qemu_sp_base);
    let emu_sp_delta = emu.regs[13].wrapping_sub(emu_sp_base);
    if qemu_sp_delta != emu_sp_delta {
        diffs.push(format!(
            "SP delta: QEMU={:#x} EMU={:#x}",
            qemu_sp_delta, emu_sp_delta
        ));
    }

    // LR (R14): delta comparison for BL (different return addresses per side),
    // absolute comparison for everything else.
    if tc.modifies_lr {
        let qemu_lr = qemu.regs[14] & !1u32;
        let emu_lr = emu.regs[14] & !1u32;
        let qemu_delta = qemu_lr.wrapping_sub(QEMU_TEST_SLOT);
        let emu_delta = emu_lr.wrapping_sub(EMU_TEST_SLOT);
        if qemu_delta != emu_delta {
            diffs.push(format!(
                "LR delta: QEMU={:#x} EMU={:#x}",
                qemu_delta, emu_delta
            ));
        }
    } else if qemu.regs[14] != emu.regs[14] {
        diffs.push(format!(
            "LR: QEMU={:#010x} EMU={:#010x}",
            qemu.regs[14], emu.regs[14]
        ));
    }

    // PC (R15): relative delta comparison (different address spaces)
    let qemu_pc_delta = qemu.regs[15].wrapping_sub(QEMU_TEST_SLOT);
    let emu_pc_delta = emu.regs[15].wrapping_sub(EMU_TEST_SLOT);
    if qemu_pc_delta != emu_pc_delta {
        diffs.push(format!(
            "PC delta: QEMU={:#x} EMU={:#x}",
            qemu_pc_delta, emu_pc_delta
        ));
    }

    // xPSR flags: masked comparison
    let qemu_flags = qemu.xpsr & tc.xpsr_mask;
    let emu_flags = emu.xpsr & tc.xpsr_mask;
    if qemu_flags != emu_flags {
        diffs.push(format!(
            "xPSR: QEMU={:#010x} EMU={:#010x} (mask={:#010x})",
            qemu.xpsr, emu.xpsr, tc.xpsr_mask
        ));
    }

    // Memory: byte-by-byte at mem_check offsets
    for (idx, &offset) in tc.mem_check.iter().enumerate() {
        let qemu_val = qemu.mem[idx];
        let emu_val = emu.mem[idx];
        if qemu_val != emu_val {
            diffs.push(format!(
                "MEM[+{offset:#x}]: QEMU={:#04x} EMU={:#04x}",
                qemu_val, emu_val
            ));
        }
    }

    if diffs.is_empty() {
        Ok(())
    } else {
        Err(diffs.join(", "))
    }
}

// ============================================================================
// Probe comparison logic (same address space — no translation)
// ============================================================================

/// Compare probe (real hardware) and emulator post-execution states.
///
/// Simpler than `compare()` because both sides use the same address space
/// (RP2354 SRAM at 0x20000000). All register values are compared as absolute
/// values — no addr_regs skipping, no delta computation.
///
/// The xPSR mask includes the T bit (bit 24) because real hardware reports
/// EPSR.T via SWD, unlike QEMU which strips it.
pub fn compare_probe(tc: &TestCase, hw: &RunState, emu: &RunState) -> Result<(), String> {
    let mut diffs = Vec::new();

    // R0-R12: absolute comparison (same address space, no skipping)
    for i in 0..=12 {
        if hw.regs[i] != emu.regs[i] {
            diffs.push(format!(
                "R{i}: HW={:#010x} EMU={:#010x}",
                hw.regs[i], emu.regs[i]
            ));
        }
    }

    // SP (R13): absolute
    if hw.regs[13] != emu.regs[13] {
        diffs.push(format!(
            "SP: HW={:#010x} EMU={:#010x}",
            hw.regs[13], emu.regs[13]
        ));
    }

    // LR (R14): absolute
    if hw.regs[14] != emu.regs[14] {
        diffs.push(format!(
            "LR: HW={:#010x} EMU={:#010x}",
            hw.regs[14], emu.regs[14]
        ));
    }

    // PC (R15): absolute
    if hw.regs[15] != emu.regs[15] {
        diffs.push(format!(
            "PC: HW={:#010x} EMU={:#010x}",
            hw.regs[15], emu.regs[15]
        ));
    }

    // xPSR: include T bit (bit 24) — real hardware reports it via SWD
    let probe_mask = tc.xpsr_mask | 0x0100_0000;
    let hw_flags = hw.xpsr & probe_mask;
    let emu_flags = emu.xpsr & probe_mask;
    if hw_flags != emu_flags {
        diffs.push(format!(
            "xPSR: HW={:#010x} EMU={:#010x}",
            hw.xpsr, emu.xpsr
        ));
    }

    // Memory: byte-by-byte at mem_check offsets
    for (idx, &offset) in tc.mem_check.iter().enumerate() {
        if hw.mem[idx] != emu.mem[idx] {
            diffs.push(format!(
                "MEM[+{offset:#x}]: HW={:#04x} EMU={:#04x}",
                hw.mem[idx], emu.mem[idx]
            ));
        }
    }

    if diffs.is_empty() {
        Ok(())
    } else {
        Err(diffs.join(", "))
    }
}

// ============================================================================
// Unit tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- TestCase::default() --

    #[test]
    fn default_xpsr_has_thumb_bit() {
        let tc = TestCase::default();
        assert_eq!(tc.xpsr_pre, 0x0100_0000, "T bit must be set");
    }

    #[test]
    fn default_mask_is_all_flags() {
        let tc = TestCase::default();
        assert_eq!(tc.xpsr_mask, MASK_ALL_FLAGS);
    }

    #[test]
    fn default_no_bus() {
        let tc = TestCase::default();
        assert!(!tc.needs_bus);
    }

    #[test]
    fn default_empty_preconditions() {
        let tc = TestCase::default();
        assert!(tc.reg_pre.is_empty());
        assert!(tc.addr_regs.is_empty());
        assert!(tc.mem_pre.is_empty());
        assert!(tc.mem_check.is_empty());
    }

    // -- Mask constants --

    #[test]
    fn mask_all_flags_covers_nzcvq() {
        // N=bit31, Z=bit30, C=bit29, V=bit28, Q=bit27
        assert_eq!(MASK_ALL_FLAGS, 0xF800_0000);
        assert_ne!(MASK_ALL_FLAGS & (1 << 31), 0, "N bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 30), 0, "Z bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 29), 0, "C bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 28), 0, "V bit");
        assert_ne!(MASK_ALL_FLAGS & (1 << 27), 0, "Q bit");
    }

    #[test]
    fn mask_nz_only_covers_nz() {
        assert_eq!(MASK_NZ_ONLY, 0xC000_0000);
        assert_ne!(MASK_NZ_ONLY & (1 << 31), 0, "N bit");
        assert_ne!(MASK_NZ_ONLY & (1 << 30), 0, "Z bit");
        assert_eq!(MASK_NZ_ONLY & (1 << 29), 0, "C bit excluded");
        assert_eq!(MASK_NZ_ONLY & (1 << 28), 0, "V bit excluded");
        assert_eq!(MASK_NZ_ONLY & (1 << 27), 0, "Q bit excluded");
    }

    #[test]
    fn mask_no_flags_is_zero() {
        assert_eq!(MASK_NO_FLAGS, 0);
    }

    // -- Address constants --

    #[test]
    fn qemu_addresses_non_overlapping() {
        assert!(QEMU_TEST_SLOT < QEMU_TEST_SCRATCH);
        assert!(QEMU_TEST_STACK > QEMU_TEST_SCRATCH);
    }

    #[test]
    fn emu_addresses_non_overlapping() {
        assert!(EMU_TEST_SLOT < EMU_TEST_SCRATCH);
        assert!(EMU_TEST_STACK > EMU_TEST_SCRATCH);
    }

    #[test]
    fn qemu_addresses_correct() {
        assert_eq!(QEMU_TEST_SLOT, 0x0000_0100);
        assert_eq!(QEMU_TEST_STACK, 0x0004_0000);
        assert_eq!(QEMU_TEST_SCRATCH, 0x0000_0200);
    }

    #[test]
    fn emu_addresses_correct() {
        assert_eq!(EMU_TEST_SLOT, 0x2000_0100);
        assert_eq!(EMU_TEST_STACK, 0x2004_0000);
        assert_eq!(EMU_TEST_SCRATCH, 0x2000_0200);
    }

    #[test]
    fn emu_addresses_in_sram() {
        assert!(EMU_TEST_SLOT >= 0x2000_0000);
        assert!(EMU_TEST_STACK >= 0x2000_0000);
        assert!(EMU_TEST_SCRATCH >= 0x2000_0000);
    }

    #[test]
    fn slot_scratch_separation() {
        assert_eq!(QEMU_TEST_SCRATCH - QEMU_TEST_SLOT, 0x100);
        assert_eq!(EMU_TEST_SCRATCH - EMU_TEST_SLOT, 0x100);
    }

    // -- GDB register indices --

    #[test]
    fn reg_indices_correct() {
        assert_eq!(REG_R0, 0);
        assert_eq!(REG_SP, 13);
        assert_eq!(REG_LR, 14);
        assert_eq!(REG_PC, 15);
        assert_eq!(REG_XPSR, 25);
    }

    // -- generate_all() tests --

    #[test]
    fn generate_all_returns_nonempty() {
        let tests = generate_all();
        assert!(!tests.is_empty(), "generate_all() must return tests");
    }

    #[test]
    fn generate_all_count_in_range() {
        let tests = generate_all();
        let count = tests.len();
        assert!(
            (380..=600).contains(&count),
            "expected 380-600 tests, got {count}"
        );
    }

    #[test]
    fn all_test_names_nonempty() {
        for tc in &generate_all() {
            assert!(!tc.name.is_empty(), "found test with empty name");
        }
    }

    #[test]
    fn no_duplicate_test_names() {
        let tests = generate_all();
        let mut names = std::collections::HashSet::new();
        for tc in &tests {
            assert!(
                names.insert(&tc.name),
                "duplicate test name: {}",
                tc.name
            );
        }
    }

    #[test]
    fn opcode_width_matches_encoding() {
        // Thumb-16 opcodes must have bits[15:11] < 0b11101 (< 0xE800).
        // Thumb-32 opcodes (hw1.is_some()) must have bits[15:11] >= 0b11101 (>= 0xE800).
        for tc in &generate_all() {
            if tc.hw1.is_none() {
                assert!(
                    tc.opcode < 0xE800,
                    "Thumb-16 test '{}' has opcode {:#06x} >= 0xE800 (looks like Thumb-32)",
                    tc.name,
                    tc.opcode
                );
            } else {
                assert!(
                    tc.opcode >= 0xE800,
                    "Thumb-32 test '{}' has opcode {:#06x} < 0xE800 (looks like Thumb-16)",
                    tc.name,
                    tc.opcode
                );
            }
        }
    }

    #[test]
    fn bus_tests_have_addr_regs() {
        for tc in &generate_all() {
            if tc.needs_bus {
                assert!(
                    !tc.addr_regs.is_empty(),
                    "test '{}' has needs_bus=true but addr_regs is empty",
                    tc.name
                );
            }
        }
    }

    #[test]
    fn mem_pre_requires_bus() {
        for tc in &generate_all() {
            if !tc.mem_pre.is_empty() {
                assert!(
                    tc.needs_bus,
                    "test '{}' has mem_pre but needs_bus=false",
                    tc.name
                );
            }
        }
    }

    #[test]
    fn all_opcodes_are_thumb16() {
        // All Phase A opcodes must be valid 16-bit Thumb.
        // Opcodes >= 0xE800 that are NOT unconditional branches
        // would be 32-bit. Our unconditional branch encoding is
        // 11100_xxxxxxxxxxx which is < 0xE800.
        for tc in &generate_all() {
            assert!(
                tc.opcode < 0xE800,
                "test '{}' has opcode {:#06x} in Thumb-32 space",
                tc.name,
                tc.opcode
            );
        }
    }

    // -- Encoding sanity checks --

    #[test]
    fn enc_lsl_imm_matches_tests_rs() {
        // LSLS R0, R1, #3 should be 0x00C8 (from tests.rs)
        assert_eq!(enc_lsl_imm(0, 1, 3), 0x00C8);
    }

    #[test]
    fn enc_adds_reg_matches_tests_rs() {
        // ADDS R0, R0, R1 should be 0x1840 (from tests.rs)
        assert_eq!(enc_adds_reg(0, 0, 1), 0x1840);
    }

    #[test]
    fn enc_movs_imm_matches_tests_rs() {
        // MOVS R0, #42 should be 0x202A (from tests.rs)
        assert_eq!(enc_movs_imm(0, 42), 0x202A);
    }

    #[test]
    fn enc_ands_matches_tests_rs() {
        // ANDS R0, R1 should be 0x4008 (from tests.rs)
        assert_eq!(enc_data_proc(0, 1, 0), 0x4008);
    }

    #[test]
    fn enc_mov_high_matches_tests_rs() {
        // MOV R0, R8 should be 0x4640 (from tests.rs)
        assert_eq!(enc_mov_high(0, 8), 0x4640);
    }

    #[test]
    fn enc_bx_matches_tests_rs() {
        // BX R0 should be 0x4700 (from tests.rs)
        assert_eq!(enc_bx(0), 0x4700);
    }

    #[test]
    fn enc_str_sp_matches_tests_rs() {
        // STR R0, [SP, #8] should be 0x9002 (from tests.rs)
        assert_eq!(enc_str_sp(0, 2), 0x9002);
    }

    #[test]
    fn enc_adr_matches_tests_rs() {
        // ADR R0, #16 should be 0xA004 (from tests.rs)
        assert_eq!(enc_adr(0, 4), 0xA004);
    }

    #[test]
    fn enc_add_sp_sp_matches_tests_rs() {
        // ADD SP, SP, #16 should be 0xB004 (from tests.rs)
        assert_eq!(enc_add_sp_sp(4), 0xB004);
    }

    #[test]
    fn enc_sub_sp_sp_matches_tests_rs() {
        // SUB SP, SP, #16 should be 0xB084 (from tests.rs)
        assert_eq!(enc_sub_sp_sp(4), 0xB084);
    }

    #[test]
    fn enc_sxth_matches_tests_rs() {
        // SXTH R0, R1 should be 0xB208 (from tests.rs)
        assert_eq!(enc_sxth(0, 1), 0xB208);
    }

    #[test]
    fn enc_uxtb_matches_tests_rs() {
        // UXTB R0, R1 should be 0xB2C8 (from tests.rs)
        assert_eq!(enc_uxtb(0, 1), 0xB2C8);
    }

    #[test]
    fn enc_rev_matches_tests_rs() {
        // REV R0, R1 should be 0xBA08 (from tests.rs)
        assert_eq!(enc_rev(0, 1), 0xBA08);
    }

    #[test]
    fn enc_push_matches_tests_rs() {
        // PUSH {R0, R1} should be 0xB403 (from tests.rs)
        assert_eq!(enc_push(0x03, false), 0xB403);
        // PUSH {LR} should be 0xB500 (from tests.rs)
        assert_eq!(enc_push(0x00, true), 0xB500);
    }

    #[test]
    fn enc_pop_matches_tests_rs() {
        // POP {R2, R3} should be 0xBC0C (from tests.rs)
        assert_eq!(enc_pop(0x0C, false), 0xBC0C);
        // POP {PC} should be 0xBD00 (from tests.rs)
        assert_eq!(enc_pop(0x00, true), 0xBD00);
    }

    #[test]
    fn enc_stm_matches_tests_rs() {
        // STM R4!, {R0, R1, R2} should be 0xC407 (from tests.rs)
        assert_eq!(enc_stm(4, 0x07), 0xC407);
    }

    #[test]
    fn enc_branch_uncond_matches_tests_rs() {
        // B +8 should be 0xE004 (from tests.rs: imm11 = 8/2 = 4)
        assert_eq!(enc_branch_uncond(8), 0xE004);
        // B -4 should be 0xE7FE (from tests.rs)
        assert_eq!(enc_branch_uncond(-4), 0xE7FE);
    }

    #[test]
    fn enc_branch_cond_matches_tests_rs() {
        // BEQ +6 should be 0xD003 (from tests.rs: cond=0, imm8=3)
        assert_eq!(enc_branch_cond(0, 6), 0xD003);
    }

    // -- Per-generator count checks --

    #[test]
    fn gen_shift_imm_count() {
        let tests = gen_shift_imm();
        assert!(
            tests.len() >= 20 && tests.len() <= 35,
            "gen_shift_imm: expected 20-35, got {}",
            tests.len()
        );
    }

    #[test]
    fn gen_add_sub_reg_count() {
        let tests = gen_add_sub_reg();
        assert!(
            tests.len() >= 20 && tests.len() <= 35,
            "gen_add_sub_reg: expected 20-35, got {}",
            tests.len()
        );
    }

    #[test]
    fn gen_data_proc_reg_count() {
        let tests = gen_data_proc_reg();
        assert!(
            tests.len() >= 30 && tests.len() <= 70,
            "gen_data_proc_reg: expected 30-70, got {}",
            tests.len()
        );
    }

    #[test]
    fn gen_branch_cond_count() {
        let tests = gen_branch_cond();
        assert!(
            tests.len() >= 15 && tests.len() <= 35,
            "gen_branch_cond: expected 15-35, got {}",
            tests.len()
        );
    }

    // -- setup_reg tests --

    #[test]
    fn setup_reg_non_addr_returns_literal() {
        let tc = TestCase {
            addr_regs: vec![1], // only R1 is an address reg
            ..TestCase::default()
        };
        // R0 is not in addr_regs, so value passes through unchanged
        assert_eq!(setup_reg(0, 0x42, &tc, EMU_TEST_SCRATCH), 0x42);
    }

    #[test]
    fn setup_reg_addr_reg_adds_base() {
        let tc = TestCase {
            addr_regs: vec![1],
            ..TestCase::default()
        };
        // R1 is an address reg: offset 0x10 + scratch base
        assert_eq!(
            setup_reg(1, 0x10, &tc, EMU_TEST_SCRATCH),
            EMU_TEST_SCRATCH + 0x10
        );
    }

    #[test]
    fn setup_reg_qemu_base() {
        let tc = TestCase {
            addr_regs: vec![3],
            ..TestCase::default()
        };
        assert_eq!(
            setup_reg(3, 0x20, &tc, QEMU_TEST_SCRATCH),
            QEMU_TEST_SCRATCH + 0x20
        );
    }

    #[test]
    fn setup_reg_empty_addr_regs() {
        let tc = TestCase::default();
        // No addr_regs — all values are literal
        assert_eq!(setup_reg(5, 0xDEAD, &tc, EMU_TEST_SCRATCH), 0xDEAD);
    }

    #[test]
    fn setup_reg_wrapping_add() {
        let tc = TestCase {
            addr_regs: vec![0],
            ..TestCase::default()
        };
        // Large offset that wraps around
        assert_eq!(
            setup_reg(0, 0xFFFF_FF00, &tc, EMU_TEST_SCRATCH),
            EMU_TEST_SCRATCH.wrapping_add(0xFFFF_FF00)
        );
    }

    // -- compare tests --

    fn make_state(regs: [u32; 16], xpsr: u32, mem: Vec<u8>) -> RunState {
        RunState { regs, xpsr, mem, cycles: 0 }
    }

    fn base_regs_qemu() -> [u32; 16] {
        let mut r = [0u32; 16];
        r[13] = QEMU_TEST_STACK;
        r[14] = 0xFFFF_FFFF;
        r[15] = QEMU_TEST_SLOT + 2; // PC after one 16-bit instruction
        r
    }

    fn base_regs_emu() -> [u32; 16] {
        let mut r = [0u32; 16];
        r[13] = EMU_TEST_STACK;
        r[14] = 0xFFFF_FFFF;
        r[15] = EMU_TEST_SLOT + 2; // PC after one 16-bit instruction
        r
    }

    #[test]
    fn compare_identical_states_ok() {
        let tc = TestCase::default();
        let qemu = make_state(base_regs_qemu(), 0x0100_0000, vec![]);
        let emu = make_state(base_regs_emu(), 0x0100_0000, vec![]);
        assert!(compare(&tc, &qemu, &emu).is_ok());
    }

    #[test]
    fn compare_register_mismatch() {
        let tc = TestCase::default();
        let mut qemu_regs = base_regs_qemu();
        let mut emu_regs = base_regs_emu();
        qemu_regs[3] = 42;
        emu_regs[3] = 99;
        let qemu = make_state(qemu_regs, 0x0100_0000, vec![]);
        let emu = make_state(emu_regs, 0x0100_0000, vec![]);
        let err = compare(&tc, &qemu, &emu).unwrap_err();
        assert!(err.contains("R3"), "expected R3 in error: {err}");
    }

    #[test]
    fn compare_sp_delta_mismatch() {
        let tc = TestCase::default();
        let mut qemu_regs = base_regs_qemu();
        let emu_regs = base_regs_emu();
        // QEMU's SP moved down by 4, emulator's didn't
        qemu_regs[13] = QEMU_TEST_STACK - 4;
        // emu_regs[13] = EMU_TEST_STACK (delta=0 vs delta=-4)
        let qemu = make_state(qemu_regs, 0x0100_0000, vec![]);
        let emu = make_state(emu_regs, 0x0100_0000, vec![]);
        let err = compare(&tc, &qemu, &emu).unwrap_err();
        assert!(err.contains("SP delta"), "expected SP delta in error: {err}");
    }

    #[test]
    fn compare_flag_mismatch() {
        let tc = TestCase::default(); // xpsr_mask = MASK_ALL_FLAGS
        let qemu = make_state(base_regs_qemu(), 0xC100_0000, vec![]); // N+Z set
        let emu = make_state(base_regs_emu(), 0x0100_0000, vec![]); // flags clear
        let err = compare(&tc, &qemu, &emu).unwrap_err();
        assert!(err.contains("xPSR"), "expected xPSR in error: {err}");
    }

    #[test]
    fn compare_flags_ignored_when_masked() {
        let tc = TestCase {
            xpsr_mask: MASK_NO_FLAGS,
            ..TestCase::default()
        };
        // Flags differ but mask is zero — should pass
        let qemu = make_state(base_regs_qemu(), 0xF100_0000, vec![]);
        let emu = make_state(base_regs_emu(), 0x0100_0000, vec![]);
        assert!(compare(&tc, &qemu, &emu).is_ok());
    }

    #[test]
    fn compare_pc_delta_mismatch() {
        let tc = TestCase::default();
        let mut qemu_regs = base_regs_qemu();
        let emu_regs = base_regs_emu();
        // QEMU branched further than emulator
        qemu_regs[15] = QEMU_TEST_SLOT + 10;
        // emu_regs[15] = EMU_TEST_SLOT + 2 (default)
        let qemu = make_state(qemu_regs, 0x0100_0000, vec![]);
        let emu = make_state(emu_regs, 0x0100_0000, vec![]);
        let err = compare(&tc, &qemu, &emu).unwrap_err();
        assert!(err.contains("PC delta"), "expected PC delta in error: {err}");
    }

    #[test]
    fn compare_pc_same_delta_ok() {
        let tc = TestCase::default();
        let mut qemu_regs = base_regs_qemu();
        let mut emu_regs = base_regs_emu();
        // Both branched +10 from their respective slot
        qemu_regs[15] = QEMU_TEST_SLOT + 10;
        emu_regs[15] = EMU_TEST_SLOT + 10;
        let qemu = make_state(qemu_regs, 0x0100_0000, vec![]);
        let emu = make_state(emu_regs, 0x0100_0000, vec![]);
        assert!(compare(&tc, &qemu, &emu).is_ok());
    }

    #[test]
    fn compare_lr_mismatch() {
        let tc = TestCase::default();
        let mut qemu_regs = base_regs_qemu();
        let mut emu_regs = base_regs_emu();
        // LR set to different absolute values
        qemu_regs[14] = 0xAAAA_AAAA;
        emu_regs[14] = 0xBBBB_BBBB;
        let qemu = make_state(qemu_regs, 0x0100_0000, vec![]);
        let emu = make_state(emu_regs, 0x0100_0000, vec![]);
        let err = compare(&tc, &qemu, &emu).unwrap_err();
        assert!(err.contains("LR"), "expected LR in error: {err}");
    }

    #[test]
    fn compare_memory_mismatch() {
        let tc = TestCase {
            needs_bus: true,
            addr_regs: vec![0],
            mem_check: vec![0, 1, 2, 3],
            ..TestCase::default()
        };
        let qemu = make_state(base_regs_qemu(), 0x0100_0000, vec![0xAB, 0xCD, 0xEF, 0x01]);
        let emu = make_state(base_regs_emu(), 0x0100_0000, vec![0xAB, 0xCD, 0x00, 0x01]);
        let err = compare(&tc, &qemu, &emu).unwrap_err();
        assert!(err.contains("MEM"), "expected MEM in error: {err}");
        assert!(err.contains("+0x2"), "expected offset +0x2 in error: {err}");
    }

    #[test]
    fn compare_memory_match_ok() {
        let tc = TestCase {
            needs_bus: true,
            addr_regs: vec![0],
            mem_check: vec![0, 1],
            ..TestCase::default()
        };
        let qemu = make_state(base_regs_qemu(), 0x0100_0000, vec![0xAB, 0xCD]);
        let emu = make_state(base_regs_emu(), 0x0100_0000, vec![0xAB, 0xCD]);
        assert!(compare(&tc, &qemu, &emu).is_ok());
    }

    #[test]
    fn compare_multiple_diffs_joined() {
        let tc = TestCase::default();
        let mut qemu_regs = base_regs_qemu();
        let mut emu_regs = base_regs_emu();
        qemu_regs[0] = 1;
        emu_regs[0] = 2;
        qemu_regs[1] = 3;
        emu_regs[1] = 4;
        let qemu = make_state(qemu_regs, 0x0100_0000, vec![]);
        let emu = make_state(emu_regs, 0x0100_0000, vec![]);
        let err = compare(&tc, &qemu, &emu).unwrap_err();
        assert!(err.contains("R0"), "expected R0: {err}");
        assert!(err.contains("R1"), "expected R1: {err}");
        assert!(err.contains(", "), "expected comma-separated: {err}");
    }

    // -- run_one_emu tests --

    #[test]
    fn run_one_emu_movs_r0_42() {
        // MOVS R0, #42 = 0x202A
        let tc = TestCase {
            name: "MOVS R0, #42".into(),
            opcode: enc_movs_imm(0, 42),
            ..TestCase::default()
        };
        let mut bus = Bus::new();
        let state = run_one_emu(&tc, &mut bus);
        assert_eq!(state.regs[0], 42, "R0 should be 42");
    }

    #[test]
    fn run_one_emu_sets_defaults() {
        // NOP = MOVS R0, #0 (opcode 0x2000) — leaves everything at defaults
        let tc = TestCase {
            name: "MOVS R0, #0 (verify defaults)".into(),
            opcode: enc_movs_imm(0, 0),
            ..TestCase::default()
        };
        let mut bus = Bus::new();
        let state = run_one_emu(&tc, &mut bus);
        // SP should be EMU_TEST_STACK
        assert_eq!(state.regs[13], EMU_TEST_STACK);
        // LR should be sentinel
        assert_eq!(state.regs[14], 0xFFFF_FFFF);
        // PC should have advanced by 2 from EMU_TEST_SLOT
        assert_eq!(state.regs[15], EMU_TEST_SLOT + 2);
    }

    #[test]
    fn run_one_emu_with_reg_pre() {
        // ADDS R0, R1, R2 with R1=100, R2=200
        let tc = TestCase {
            name: "ADDS R0, R1, R2".into(),
            opcode: enc_adds_reg(0, 1, 2),
            reg_pre: vec![(1, 100), (2, 200)],
            ..TestCase::default()
        };
        let mut bus = Bus::new();
        let state = run_one_emu(&tc, &mut bus);
        assert_eq!(state.regs[0], 300, "R0 should be 300");
    }

    #[test]
    fn run_one_emu_xpsr_pre_applied() {
        // CMP R0, #0 with Z flag already set — verify xpsr_pre is honored
        let tc = TestCase {
            name: "MOVS R0, #1 (C flag pre-set)".into(),
            opcode: enc_movs_imm(0, 1),
            xpsr_pre: 0x2100_0000, // T bit + C flag set
            ..TestCase::default()
        };
        let mut bus = Bus::new();
        let state = run_one_emu(&tc, &mut bus);
        // MOVS sets N,Z but preserves C — so C should still be set
        assert_ne!(state.xpsr & 0x2000_0000, 0, "C flag should be preserved");
    }

    // -- Fuzz generator tests --

    #[test]
    fn fuzz_deterministic_with_fixed_seed() {
        let (alu1, mem1) = generate_fuzz(5, 42);
        let (alu2, mem2) = generate_fuzz(5, 42);
        assert_eq!(alu1.len(), alu2.len());
        assert_eq!(mem1.len(), mem2.len());
        for (a, b) in alu1.iter().zip(alu2.iter()) {
            assert_eq!(a.name, b.name, "names must match for same seed");
            assert_eq!(a.opcode, b.opcode, "opcodes must match for same seed");
            assert_eq!(a.xpsr_pre, b.xpsr_pre, "xpsr_pre must match for same seed");
            assert_eq!(a.reg_pre, b.reg_pre, "reg_pre must match for same seed");
        }
        for (a, b) in mem1.iter().zip(mem2.iter()) {
            assert_eq!(a.name, b.name);
            assert_eq!(a.opcode, b.opcode);
        }
    }

    #[test]
    fn fuzz_different_seeds_differ() {
        let (alu1, _) = generate_fuzz(10, 1);
        let (alu2, _) = generate_fuzz(10, 2);
        // With different seeds, at least some opcodes should differ
        let differs = alu1.iter().zip(alu2.iter()).any(|(a, b)| a.opcode != b.opcode);
        assert!(differs, "different seeds should produce different tests");
    }

    #[test]
    fn fuzz_alu_opcodes_are_valid_thumb16() {
        let (alu, _) = generate_fuzz(20, 123);
        for tc in &alu {
            assert!(
                tc.opcode < 0xE800,
                "fuzz test '{}' has opcode {:#06x} >= 0xE800",
                tc.name, tc.opcode
            );
        }
    }

    #[test]
    fn fuzz_mem_opcodes_are_valid_thumb16() {
        let (_, mem) = generate_fuzz(20, 456);
        for tc in &mem {
            assert!(
                tc.opcode < 0xE800,
                "fuzz test '{}' has opcode {:#06x} >= 0xE800",
                tc.name, tc.opcode
            );
        }
    }

    #[test]
    fn fuzz_all_names_nonempty() {
        let (alu, mem) = generate_fuzz(10, 789);
        for tc in alu.iter().chain(mem.iter()) {
            assert!(!tc.name.is_empty(), "found fuzz test with empty name");
        }
    }

    #[test]
    fn fuzz_all_names_have_fuzz_prefix() {
        let (alu, mem) = generate_fuzz(5, 999);
        for tc in alu.iter().chain(mem.iter()) {
            assert!(
                tc.name.starts_with("FUZZ:"),
                "fuzz test name '{}' missing FUZZ: prefix",
                tc.name
            );
        }
    }

    #[test]
    fn fuzz_mem_tests_have_addr_regs() {
        let (_, mem) = generate_fuzz(20, 555);
        for tc in &mem {
            assert!(
                !tc.addr_regs.is_empty(),
                "fuzz mem test '{}' has empty addr_regs",
                tc.name
            );
        }
    }

    #[test]
    fn fuzz_mem_tests_have_needs_bus() {
        let (_, mem) = generate_fuzz(20, 666);
        for tc in &mem {
            assert!(
                tc.needs_bus,
                "fuzz mem test '{}' has needs_bus=false",
                tc.name
            );
        }
    }

    #[test]
    fn fuzz_alu_tests_no_bus() {
        let (alu, _) = generate_fuzz(20, 777);
        for tc in &alu {
            assert!(
                !tc.needs_bus,
                "fuzz ALU test '{}' has needs_bus=true",
                tc.name
            );
        }
    }

    #[test]
    fn fuzz_generates_expected_count() {
        let (alu, mem) = generate_fuzz(10, 0);
        // ALU: 9 classes * 10 = 90 (shift, addsub, imm8, dproc, special, misc, bcond, buncond)
        // Wait — let me count: shift, addsub, imm8, dproc, special, misc, bcond, buncond = 8 loops
        // Mem: lsreg, lsimm, push/pop, stm/ldm, lssp = 5 loops
        assert_eq!(alu.len(), 8 * 10, "ALU count: 8 classes * 10");
        assert_eq!(mem.len(), 5 * 10, "MEM count: 5 classes * 10");
    }

    #[test]
    fn fuzz_xpsr_always_has_thumb_bit() {
        let (alu, mem) = generate_fuzz(20, 111);
        for tc in alu.iter().chain(mem.iter()) {
            assert_ne!(
                tc.xpsr_pre & 0x0100_0000, 0,
                "fuzz test '{}' missing T bit in xpsr_pre: {:#010x}",
                tc.name, tc.xpsr_pre
            );
        }
    }

    // -- RunState.cycles --

    #[test]
    fn runstate_cycles_default_is_zero() {
        let state = make_state([0; 16], 0, vec![]);
        assert_eq!(state.cycles, 0);
    }

    // -- run_one_emu captures cycles --

    #[test]
    fn run_one_emu_captures_cycles() {
        // MOVS R0, #42 (encoding T1: 0x202A) — should take 1 cycle
        let tc = TestCase {
            opcode: 0x202A,
            ..TestCase::default()
        };
        let mut bus = Bus::new();
        let state = run_one_emu(&tc, &mut bus);
        assert_eq!(state.regs[0], 42);
        assert_eq!(state.cycles, 1, "MOVS R0, #42 should be 1 cycle");
    }

    // -- compare_probe tests --

    fn base_regs_probe() -> [u32; 16] {
        let mut r = [0u32; 16];
        r[13] = EMU_TEST_STACK;
        r[14] = 0xFFFF_FFFF;
        r[15] = EMU_TEST_SLOT + 2; // PC after one 16-bit instruction
        r
    }

    #[test]
    fn compare_probe_identical_states_ok() {
        let tc = TestCase::default();
        let hw = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        let emu = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        assert!(compare_probe(&tc, &hw, &emu).is_ok());
    }

    #[test]
    fn compare_probe_register_mismatch() {
        let tc = TestCase::default();
        let mut hw_regs = base_regs_probe();
        hw_regs[3] = 0xDEAD_BEEF;
        let hw = make_state(hw_regs, 0x0100_0000, vec![]);
        let emu = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        let err = compare_probe(&tc, &hw, &emu).unwrap_err();
        assert!(err.contains("R3"), "should report R3 mismatch: {err}");
    }

    #[test]
    fn compare_probe_xpsr_t_bit_mismatch() {
        let tc = TestCase::default();
        // HW has T bit set, emu does not
        let hw = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        let emu = make_state(base_regs_probe(), 0x0000_0000, vec![]);
        let err = compare_probe(&tc, &hw, &emu).unwrap_err();
        assert!(err.contains("xPSR"), "should report xPSR mismatch: {err}");
    }

    #[test]
    fn compare_probe_no_addr_regs_skipping() {
        // In the QEMU compare(), addr_regs causes registers to be skipped.
        // compare_probe() must NOT skip them — it compares all regs.
        let tc = TestCase {
            addr_regs: vec![2],
            ..TestCase::default()
        };
        let mut hw_regs = base_regs_probe();
        hw_regs[2] = 0x1111;
        let mut emu_regs = base_regs_probe();
        emu_regs[2] = 0x2222;
        let hw = make_state(hw_regs, 0x0100_0000, vec![]);
        let emu = make_state(emu_regs, 0x0100_0000, vec![]);
        let err = compare_probe(&tc, &hw, &emu).unwrap_err();
        assert!(err.contains("R2"), "should detect R2 diff even with addr_regs: {err}");
    }

    #[test]
    fn compare_probe_sp_absolute() {
        let tc = TestCase::default();
        let mut hw_regs = base_regs_probe();
        hw_regs[13] = EMU_TEST_STACK - 4;
        let hw = make_state(hw_regs, 0x0100_0000, vec![]);
        let emu = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        let err = compare_probe(&tc, &hw, &emu).unwrap_err();
        assert!(err.contains("SP"), "should detect SP diff: {err}");
    }

    #[test]
    fn compare_probe_lr_absolute() {
        let tc = TestCase::default();
        let mut hw_regs = base_regs_probe();
        hw_regs[14] = 0x2000_0102;
        let hw = make_state(hw_regs, 0x0100_0000, vec![]);
        let emu = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        let err = compare_probe(&tc, &hw, &emu).unwrap_err();
        assert!(err.contains("LR"), "should detect LR diff: {err}");
    }

    #[test]
    fn compare_probe_pc_absolute() {
        let tc = TestCase::default();
        let mut hw_regs = base_regs_probe();
        hw_regs[15] = EMU_TEST_SLOT + 4;
        let hw = make_state(hw_regs, 0x0100_0000, vec![]);
        let emu = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        let err = compare_probe(&tc, &hw, &emu).unwrap_err();
        assert!(err.contains("PC"), "should detect PC diff: {err}");
    }

    #[test]
    fn compare_probe_memory_mismatch() {
        let tc = TestCase {
            mem_check: vec![0, 4],
            ..TestCase::default()
        };
        let hw = RunState {
            regs: base_regs_probe(),
            xpsr: 0x0100_0000,
            mem: vec![0xAA, 0xBB],
            cycles: 0,
        };
        let emu = RunState {
            regs: base_regs_probe(),
            xpsr: 0x0100_0000,
            mem: vec![0xAA, 0xCC],
            cycles: 0,
        };
        let err = compare_probe(&tc, &hw, &emu).unwrap_err();
        assert!(err.contains("MEM"), "should detect memory diff: {err}");
        assert!(!err.contains("+0x0"), "offset 0 should match");
        assert!(err.contains("+0x4"), "offset 4 should mismatch: {err}");
    }

    #[test]
    fn compare_probe_xpsr_mask_applies() {
        // Flags that are outside the mask should not cause a mismatch
        let tc = TestCase {
            xpsr_mask: 0x8000_0000, // only N flag
            ..TestCase::default()
        };
        // Both have T bit set, both have N=0, but differ in Z (bit 30)
        let hw = make_state(base_regs_probe(), 0x4100_0000, vec![]);
        let emu = make_state(base_regs_probe(), 0x0100_0000, vec![]);
        // Z bit differs but is outside mask — should be Ok
        assert!(compare_probe(&tc, &hw, &emu).is_ok());
    }
}

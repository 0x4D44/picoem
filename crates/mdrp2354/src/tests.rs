use crate::core::CortexM33;
use crate::bus::Bus;

// ============================================================================
// Helper: build a core + bus, optionally pre-load SRAM
// ============================================================================

fn core_and_bus() -> (CortexM33, Bus) {
    (CortexM33::new(), Bus::new())
}

// ============================================================================
// Shift (immediate)
// ============================================================================

#[test]
fn lsls_imm_basic() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0001);
    let cy = c.execute_one(0x00C8); // LSLS R0, R1, #3 → R0 = 1 << 3 = 8
    assert_eq!(c.reg(0), 8);
    assert!(!c.flag_n());
    assert!(!c.flag_z());
    assert!(!c.flag_c());
    assert_eq!(cy, 1);
}

#[test]
fn lsls_imm_zero_shift_is_movs() {
    let mut c = CortexM33::new();
    c.set_reg(2, 42);
    c.regs.set_flag_c(true); // carry should be preserved
    c.execute_one(0x0010); // LSLS R0, R2, #0 → MOVS R0, R2
    assert_eq!(c.reg(0), 42);
    assert!(c.flag_c()); // unchanged
}

#[test]
fn lsls_imm_carry_out() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x8000_0000);
    c.execute_one(0x0040); // LSLS R0, R0, #1 → 0, carry = 1
    assert_eq!(c.reg(0), 0);
    assert!(c.flag_z());
    assert!(c.flag_c());
}

#[test]
fn lsrs_imm_basic() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0x80);
    c.execute_one(0x08C8); // LSRS R0, R1, #3 → 0x80 >> 3 = 0x10
    assert_eq!(c.reg(0), 0x10);
    assert!(!c.flag_c());
    assert_eq!(c.execute_one(0x08C8), 1); // cycle count
}

#[test]
fn lsrs_imm_shift_32() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x8000_0000);
    // LSRS R0, R0, #32 → encoded as imm5=0 → result=0, carry=bit31
    c.execute_one(0x0800); // bits: 00001_00000_000_000
    assert_eq!(c.reg(0), 0);
    assert!(c.flag_c()); // bit 31 was set
    assert!(c.flag_z());
}

#[test]
fn asrs_imm_positive() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0x40);
    c.execute_one(0x10C8); // ASRS R0, R1, #3 → 0x40 >> 3 = 8
    assert_eq!(c.reg(0), 8);
    assert!(!c.flag_n());
}

#[test]
fn asrs_imm_negative() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0xFFFF_FF00u32);
    c.execute_one(0x1108); // ASRS R0, R1, #4 → sign-extended right shift
    assert_eq!(c.reg(0), 0xFFFF_FFF0);
    assert!(c.flag_n());
}

// ============================================================================
// Add/Sub (register and 3-bit immediate)
// ============================================================================

#[test]
fn adds_reg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 5);
    c.set_reg(1, 3);
    let cy = c.execute_one(0x1840); // ADDS R0, R0, R1
    assert_eq!(c.reg(0), 8);
    assert!(!c.flag_z());
    assert!(!c.flag_n());
    assert!(!c.flag_c());
    assert!(!c.flag_v());
    assert_eq!(cy, 1);
}

#[test]
fn adds_reg_overflow() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x7FFF_FFFF);
    c.set_reg(1, 1);
    c.execute_one(0x1840); // ADDS R0, R0, R1
    assert_eq!(c.reg(0), 0x8000_0000);
    assert!(c.flag_n());
    assert!(c.flag_v()); // signed overflow
    assert!(!c.flag_c()); // no unsigned overflow
}

#[test]
fn subs_reg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 10);
    c.set_reg(1, 3);
    c.execute_one(0x1A40); // SUBS R0, R0, R1
    assert_eq!(c.reg(0), 7);
    assert!(!c.flag_z());
    assert!(!c.flag_n());
    assert!(c.flag_c()); // no borrow
}

#[test]
fn subs_reg_borrow() {
    let mut c = CortexM33::new();
    c.set_reg(0, 3);
    c.set_reg(1, 10);
    c.execute_one(0x1A40); // SUBS R0, R0, R1
    assert_eq!(c.reg(0), 3u32.wrapping_sub(10));
    assert!(c.flag_n());
    assert!(!c.flag_c()); // borrow
}

#[test]
fn adds_imm3() {
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.execute_one(0x1CC8); // ADDS R0, R1, #3
    assert_eq!(c.reg(0), 103);
}

#[test]
fn subs_imm3() {
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.execute_one(0x1EC8); // SUBS R0, R1, #3
    assert_eq!(c.reg(0), 97);
}

// ============================================================================
// Move/Compare/Add/Sub 8-bit immediate
// ============================================================================

#[test]
fn movs_imm() {
    let mut c = CortexM33::new();
    c.execute_one(0x202A); // MOVS R0, #42
    assert_eq!(c.reg(0), 42);
    assert!(!c.flag_z());
    assert!(!c.flag_n());
}

#[test]
fn movs_imm_zero() {
    let mut c = CortexM33::new();
    c.set_reg(0, 999);
    c.execute_one(0x2000); // MOVS R0, #0
    assert_eq!(c.reg(0), 0);
    assert!(c.flag_z());
}

#[test]
fn cmp_imm_equal() {
    let mut c = CortexM33::new();
    c.set_reg(0, 42);
    c.execute_one(0x282A); // CMP R0, #42
    assert!(c.flag_z()); // equal
    assert!(c.flag_c()); // no borrow (42 >= 42)
    assert!(!c.flag_v());
}

#[test]
fn cmp_imm_greater() {
    let mut c = CortexM33::new();
    c.set_reg(0, 100);
    c.execute_one(0x282A); // CMP R0, #42
    assert!(!c.flag_z());
    assert!(c.flag_c()); // no borrow (100 >= 42)
    assert!(!c.flag_n());
}

#[test]
fn cmp_imm_less() {
    let mut c = CortexM33::new();
    c.set_reg(0, 10);
    c.execute_one(0x282A); // CMP R0, #42
    assert!(!c.flag_z());
    assert!(!c.flag_c()); // borrow (10 < 42)
    assert!(c.flag_n());
}

#[test]
fn adds_imm8() {
    let mut c = CortexM33::new();
    c.set_reg(0, 100);
    c.execute_one(0x3019); // ADDS R0, #25
    assert_eq!(c.reg(0), 125);
}

#[test]
fn subs_imm8() {
    let mut c = CortexM33::new();
    c.set_reg(0, 100);
    c.execute_one(0x3819); // SUBS R0, #25
    assert_eq!(c.reg(0), 75);
    assert!(c.flag_c()); // no borrow
}

// ============================================================================
// Data processing (register)
// ============================================================================

#[test]
fn ands() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0xFF);
    c.set_reg(1, 0x0F);
    c.execute_one(0x4008); // ANDS R0, R1
    assert_eq!(c.reg(0), 0x0F);
}

#[test]
fn eors() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0xFF);
    c.set_reg(1, 0xF0);
    c.execute_one(0x4048); // EORS R0, R1
    assert_eq!(c.reg(0), 0x0F);
}

#[test]
fn lsls_reg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 1);
    c.set_reg(1, 4);
    c.execute_one(0x4088); // LSLS R0, R1 (shift R0 by R1)
    assert_eq!(c.reg(0), 16);
}

#[test]
fn lsrs_reg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x100);
    c.set_reg(1, 4);
    c.execute_one(0x40C8); // LSRS R0, R1
    assert_eq!(c.reg(0), 0x10);
}

#[test]
fn adcs() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0xFFFF_FFFF);
    c.set_reg(1, 0);
    c.regs.set_flag_c(true);
    c.execute_one(0x4148); // ADCS R0, R1 → 0xFFFFFFFF + 0 + 1 = 0, carry=1
    assert_eq!(c.reg(0), 0);
    assert!(c.flag_z());
    assert!(c.flag_c());
}

#[test]
fn sbcs() {
    let mut c = CortexM33::new();
    c.set_reg(0, 10);
    c.set_reg(1, 3);
    c.regs.set_flag_c(true); // C=1 means no borrow from previous
    c.execute_one(0x4188); // SBCS R0, R1 → 10 + NOT(3) + 1 = 10 - 3 = 7
    assert_eq!(c.reg(0), 7);
    assert!(c.flag_c()); // no borrow
}

#[test]
fn rors() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x0000_0001);
    c.set_reg(1, 1);
    c.execute_one(0x41C8); // RORS R0, R1 → rotate right by 1
    assert_eq!(c.reg(0), 0x8000_0000);
    assert!(c.flag_n());
    assert!(c.flag_c()); // bit 31 of result
}

#[test]
fn tst() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0xFF00);
    c.set_reg(1, 0x00FF);
    c.execute_one(0x4208); // TST R0, R1
    assert!(c.flag_z()); // no bits in common
    assert_eq!(c.reg(0), 0xFF00); // unchanged
}

#[test]
fn rsbs_neg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0);
    c.set_reg(1, 42);
    c.execute_one(0x4248); // RSBS R0, R1, #0 → 0 - 42
    assert_eq!(c.reg(0), (0u32).wrapping_sub(42));
    assert!(c.flag_n());
}

#[test]
fn cmp_reg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 42);
    c.set_reg(1, 42);
    c.execute_one(0x4288); // CMP R0, R1
    assert!(c.flag_z());
    assert!(c.flag_c());
}

#[test]
fn cmn() {
    let mut c = CortexM33::new();
    c.set_reg(0, 1);
    c.set_reg(1, 0xFFFF_FFFF);
    c.execute_one(0x42C8); // CMN R0, R1 → 1 + 0xFFFFFFFF = 0, carry
    assert!(c.flag_z());
    assert!(c.flag_c());
}

#[test]
fn orrs() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0xF0);
    c.set_reg(1, 0x0F);
    c.execute_one(0x4308); // ORRS R0, R1
    assert_eq!(c.reg(0), 0xFF);
}

#[test]
fn muls() {
    let mut c = CortexM33::new();
    c.set_reg(0, 7);
    c.set_reg(1, 6);
    c.execute_one(0x4348); // MULS R0, R1
    assert_eq!(c.reg(0), 42);
}

#[test]
fn bics() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0xFF);
    c.set_reg(1, 0x0F);
    c.execute_one(0x4388); // BICS R0, R1
    assert_eq!(c.reg(0), 0xF0);
}

#[test]
fn mvns() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0);
    c.set_reg(1, 0);
    c.execute_one(0x43C8); // MVNS R0, R1
    assert_eq!(c.reg(0), 0xFFFF_FFFF);
    assert!(c.flag_n());
}

// ============================================================================
// Special data / BX / BLX
// ============================================================================

#[test]
fn mov_high_reg() {
    let mut c = CortexM33::new();
    c.set_reg(8, 0xDEAD_BEEF);
    // MOV R0, R8: 01000110_0_1000_000 = 0x4640
    c.execute_one(0x4640);
    assert_eq!(c.reg(0), 0xDEAD_BEEF);
}

#[test]
fn add_high_reg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 10);
    c.set_reg(8, 20);
    // ADD R0, R8: 01000100_0_1000_000 = 0x4440
    c.execute_one(0x4440);
    assert_eq!(c.reg(0), 30);
}

#[test]
fn bx_reg() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x2000_0001); // bit 0 = Thumb
    c.regs.set_pc(0x1000);
    // BX R0: 0100_0111_0_0000_000 = 0x4700
    c.execute_one(0x4700);
    assert_eq!(c.regs.pc(), 0x2000_0000);
}

// ============================================================================
// Load/store (register offset)
// ============================================================================

#[test]
fn str_ldr_reg() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xCAFE_BABE);
    c.set_reg(1, 0x2000_0000); // SRAM base
    c.set_reg(2, 4);           // offset
    // STR R0, [R1, R2]: 0101_000_010_001_000 = 0x5088
    c.execute_one_with_bus(0x5088, &mut bus);
    assert_eq!(bus.read32(0x2000_0004), 0xCAFE_BABE);
    // LDR R3, [R1, R2]: 0101_100_010_001_011 = 0x588B
    c.execute_one_with_bus(0x588B, &mut bus);
    assert_eq!(c.reg(3), 0xCAFE_BABE);
}

#[test]
fn strb_ldrb_reg() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xAB);
    c.set_reg(1, 0x2000_0000);
    c.set_reg(2, 1);
    // STRB R0, [R1, R2]: 0101_010_010_001_000 = 0x5488
    c.execute_one_with_bus(0x5488, &mut bus);
    assert_eq!(bus.read8(0x2000_0001), 0xAB);
    // LDRB R3, [R1, R2]: 0101_110_010_001_011 = 0x5C8B
    c.execute_one_with_bus(0x5C8B, &mut bus);
    assert_eq!(c.reg(3), 0xAB);
}

#[test]
fn ldrsb_sign_extends() {
    let (mut c, mut bus) = core_and_bus();
    bus.write8(0x2000_0000, 0x80); // -128 as signed byte
    c.set_reg(1, 0x2000_0000);
    c.set_reg(2, 0);
    // LDRSB R0, [R1, R2]: 0101_011_010_001_000 = 0x5688
    c.execute_one_with_bus(0x5688, &mut bus);
    assert_eq!(c.reg(0), 0xFFFF_FF80);
}

// ============================================================================
// Load/store (immediate offset)
// ============================================================================

#[test]
fn str_ldr_imm_word() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0x1234_5678);
    c.set_reg(1, 0x2000_0000);
    // STR R0, [R1, #8]: 01100_00010_001_000 = 0x6088
    c.execute_one_with_bus(0x6088, &mut bus);
    assert_eq!(bus.read32(0x2000_0008), 0x1234_5678);
    // LDR R2, [R1, #8]: 01101_00010_001_010 = 0x688A
    c.execute_one_with_bus(0x688A, &mut bus);
    assert_eq!(c.reg(2), 0x1234_5678);
}

#[test]
fn strb_ldrb_imm() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xCD);
    c.set_reg(1, 0x2000_0000);
    // STRB R0, [R1, #2]: 01110_00010_001_000 = 0x7088
    c.execute_one_with_bus(0x7088, &mut bus);
    assert_eq!(bus.read8(0x2000_0002), 0xCD);
}

#[test]
fn strh_ldrh_imm() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xBEEF);
    c.set_reg(1, 0x2000_0000);
    // STRH R0, [R1, #4]: 10000_00010_001_000 = 0x8088
    c.execute_one_with_bus(0x8088, &mut bus);
    assert_eq!(bus.read16(0x2000_0004), 0xBEEF);
    // LDRH R2, [R1, #4]: 10001_00010_001_010 = 0x888A
    c.execute_one_with_bus(0x888A, &mut bus);
    assert_eq!(c.reg(2), 0xBEEF);
}

// ============================================================================
// SP-relative load/store
// ============================================================================

#[test]
fn str_ldr_sp() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(13, 0x2000_1000); // SP
    c.set_reg(0, 0xDEAD_BEEF);
    // STR R0, [SP, #8]: 10010_000_00000010 = 0x9002
    c.execute_one_with_bus(0x9002, &mut bus);
    assert_eq!(bus.read32(0x2000_1008), 0xDEAD_BEEF);
    // LDR R1, [SP, #8]: 10011_001_00000010 = 0x9902
    c.execute_one_with_bus(0x9902, &mut bus);
    assert_eq!(c.reg(1), 0xDEAD_BEEF);
}

// ============================================================================
// ADR / ADD SP
// ============================================================================

#[test]
fn adr_pc_relative() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // ADR R0, #16: 10100_000_00000100 = 0xA004
    c.execute_one(0xA004);
    // read_pc() = 0x1000 + 4 = 0x1004, aligned = 0x1004, + 16 = 0x1014
    assert_eq!(c.reg(0), 0x1014);
}

#[test]
fn add_rd_sp_imm() {
    let mut c = CortexM33::new();
    c.set_reg(13, 0x2000_1000);
    // ADD R0, SP, #32: 10101_000_00001000 = 0xA808
    c.execute_one(0xA808);
    assert_eq!(c.reg(0), 0x2000_1020);
}

// ============================================================================
// Miscellaneous
// ============================================================================

#[test]
fn add_sp_imm() {
    let mut c = CortexM33::new();
    c.set_reg(13, 0x2000_1000);
    // ADD SP, SP, #16: 10110000_0_0000100 = 0xB004
    c.execute_one(0xB004);
    assert_eq!(c.regs.sp(), 0x2000_1010);
}

#[test]
fn sub_sp_imm() {
    let mut c = CortexM33::new();
    c.set_reg(13, 0x2000_1000);
    // SUB SP, SP, #16: 10110000_1_0000100 = 0xB084
    c.execute_one(0xB084);
    assert_eq!(c.regs.sp(), 0x2000_0FF0);
}

#[test]
fn sxth() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_8000); // -32768 as i16
    // SXTH R0, R1: 10110010_00_001_000 = 0xB208
    c.execute_one(0xB208);
    assert_eq!(c.reg(0), 0xFFFF_8000);
}

#[test]
fn uxtb() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0xDEAD_BEEF);
    // UXTB R0, R1: 10110010_11_001_000 = 0xB2C8
    c.execute_one(0xB2C8);
    assert_eq!(c.reg(0), 0xEF);
}

#[test]
fn rev() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0x12_34_56_78);
    // REV R0, R1: 10111010_00_001_000 = 0xBA08
    c.execute_one(0xBA08);
    assert_eq!(c.reg(0), 0x78_56_34_12);
}

#[test]
fn rev16() {
    let mut c = CortexM33::new();
    c.set_reg(1, 0x1234_5678);
    // REV16 R0, R1: 10111010_01_001_000 = 0xBA48
    c.execute_one(0xBA48);
    assert_eq!(c.reg(0), 0x3412_7856);
}

// ============================================================================
// Push / Pop
// ============================================================================

#[test]
fn push_pop_basic() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(13, 0x2000_1000); // SP
    c.set_reg(0, 0xAAAA);
    c.set_reg(1, 0xBBBB);
    // PUSH {R0, R1}: 1011_0100_00000011 = 0xB403
    c.execute_one_with_bus(0xB403, &mut bus);
    assert_eq!(c.regs.sp(), 0x2000_0FF8); // SP -= 8 (2 regs)
    assert_eq!(bus.read32(0x2000_0FF8), 0xAAAA);
    assert_eq!(bus.read32(0x2000_0FFC), 0xBBBB);
    // POP {R2, R3}: 1011_1100_00001100 = 0xBC0C
    c.execute_one_with_bus(0xBC0C, &mut bus);
    assert_eq!(c.reg(2), 0xAAAA);
    assert_eq!(c.reg(3), 0xBBBB);
    assert_eq!(c.regs.sp(), 0x2000_1000); // SP restored
}

#[test]
fn push_lr_pop_pc() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(13, 0x2000_1000);
    c.set_reg(14, 0x0800_0101); // LR (with Thumb bit)
    // PUSH {LR}: 1011_0101_00000000 = 0xB500
    c.execute_one_with_bus(0xB500, &mut bus);
    assert_eq!(c.regs.sp(), 0x2000_0FFC);
    assert_eq!(bus.read32(0x2000_0FFC), 0x0800_0101);
    // POP {PC}: 1011_1101_00000000 = 0xBD00
    c.execute_one_with_bus(0xBD00, &mut bus);
    assert_eq!(c.regs.pc(), 0x0800_0100); // bit 0 cleared
    assert_eq!(c.regs.sp(), 0x2000_1000);
}

// ============================================================================
// STM / LDM
// ============================================================================

#[test]
fn stm_ldm() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(4, 0x2000_0100); // base
    c.set_reg(0, 0x11);
    c.set_reg(1, 0x22);
    c.set_reg(2, 0x33);
    // STM R4!, {R0, R1, R2}: 11000_100_00000111 = 0xC407
    c.execute_one_with_bus(0xC407, &mut bus);
    assert_eq!(c.reg(4), 0x2000_010C); // writeback
    assert_eq!(bus.read32(0x2000_0100), 0x11);
    assert_eq!(bus.read32(0x2000_0104), 0x22);
    assert_eq!(bus.read32(0x2000_0108), 0x33);
    // LDM R5!, {R0, R1, R2} — load from same address
    c.set_reg(5, 0x2000_0100);
    // LDM R5!, {R0,R1,R2}: 11001_101_00000111 = 0xCD07
    // Actually Rn=R5 → bits[10:8]=101
    c.execute_one_with_bus(0xCD07, &mut bus);
    assert_eq!(c.reg(0), 0x11);
    assert_eq!(c.reg(1), 0x22);
    assert_eq!(c.reg(2), 0x33);
    assert_eq!(c.reg(5), 0x2000_010C);
}

// ============================================================================
// Branches
// ============================================================================

#[test]
fn branch_unconditional() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // B +8: PC = read_pc() + 8 = 0x1004 + 8 = 0x100C
    // imm11 = 8/2 = 4 → 11100_00000000100 = 0xE004
    c.execute_one(0xE004);
    assert_eq!(c.regs.pc(), 0x100C);
}

#[test]
fn branch_unconditional_backward() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // B -4: offset = -4, imm11 = (-4/2) & 0x7FF = 0x7FE
    // 11100_11111111110 = 0xE7FE
    c.execute_one(0xE7FE);
    assert_eq!(c.regs.pc(), 0x1000); // loops to self
}

#[test]
fn branch_cond_taken() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    c.regs.set_flag_z(true);
    // BEQ +6: cond=0000(EQ), imm8 = 6/2 = 3
    // 1101_0000_00000011 = 0xD003
    let cy = c.execute_one(0xD003);
    assert_eq!(c.regs.pc(), 0x100A); // read_pc()=0x1004, +6=0x100A
    assert_eq!(cy, 2); // taken
}

#[test]
fn branch_cond_not_taken() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    c.regs.set_flag_z(false);
    // BEQ +6 but Z=0: not taken
    let cy = c.execute_one(0xD003);
    // PC should NOT change (execute_one doesn't advance PC)
    assert_eq!(cy, 1); // not taken
}

#[test]
fn bl_forward() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // BL +100: offset = 100
    // S=0, I1=0, I2=0 → J1 = NOT(0^0) = 1, J2 = NOT(0^0) = 1
    // imm10 = 0, imm11 = 50 (100/2)
    // hw0 = 11110_0_0000000000 = 0xF000
    // hw1 = 11_1_1_1_00000110010 = 0xF832
    let cy = c.execute_one_wide(0xF000, 0xF832);
    // LR = next_instr | 1 = 0x1004 | 1 = 0x1005
    assert_eq!(c.regs.lr(), 0x1005);
    // target = read_pc() + 100 = 0x1004 + 100 = 0x1068
    assert_eq!(c.regs.pc(), 0x1068);
    assert_eq!(cy, 4);
}

// ============================================================================
// Integration: small program in SRAM
// ============================================================================

#[test]
fn run_small_program() {
    // Program: sum 1+2+3+4+5 in R0
    // MOVS R0, #0       → 0x2000
    // MOVS R1, #1       → 0x2101
    // MOVS R2, #5       → 0x2205
    // ADDS R0, R0, R1   → 0x1840
    // ADDS R1, R1, #1   → 0x3101
    // CMP R1, R2        → 0x4291  (actually CMP R1, R2 is data-proc: 010000_1010_010_001)
    // BLE -8             → 0xDDFA  (LE = cond 1101, offset = -8 → imm8 = 0xFA? let me compute)
    // Actually let me just use a different approach:
    // CMP R1, #6        → 0x2906
    // BNE -8             → offset=-8 from read_pc. imm8 = (-8/2) & 0xFF = 0xFC
    // BNE: cond=0001, 1101_0001_11111100 = 0xD1FC

    let (mut c, mut bus) = core_and_bus();
    let program: &[u16] = &[
        0x2000, // MOVS R0, #0
        0x2101, // MOVS R1, #1
        0x1840, // ADDS R0, R0, R1
        0x3101, // ADDS R1, #1
        0x2906, // CMP R1, #6
        0xD1FB, // BNE -10 (back to ADDS R0, R0, R1)
        0xE7FE, // B . (infinite loop — halting point)
    ];

    // Load program into SRAM at 0x20000000
    let base = 0x2000_0000u32;
    for (i, &instr) in program.iter().enumerate() {
        let addr = base + (i as u32) * 2;
        bus.write16(addr, instr);
    }

    // Set PC to program start
    c.regs.set_pc(base);

    // Run until we hit the infinite loop (PC stable across two non-stall steps)
    let mut stable_count = 0u32;
    let mut prev_pc = !0u32;
    for _ in 0..500 {
        c.step(&mut bus);
        if c.stall_cycles() == 0 {
            let pc = c.regs.pc();
            if pc == prev_pc {
                stable_count += 1;
                if stable_count >= 2 {
                    break;
                }
            } else {
                stable_count = 0;
            }
            prev_pc = pc;
        }
    }

    // R0 should be 1+2+3+4+5 = 15
    assert_eq!(c.reg(0), 15);
}

// ============================================================================
// ThumbExpandImm
// ============================================================================

use crate::core::execute_thumb32::{thumb_expand_imm_c, thumb_expand_imm, extract_imm12};

#[test]
fn thumb_expand_imm_pattern_00() {
    // imm12 = 0b00_00_01000010 = 0x042 → pattern 00, imm8 = 0x42
    let (val, carry) = thumb_expand_imm_c(0x042, false);
    assert_eq!(val, 0x0000_0042);
    assert!(!carry); // carry_in unchanged

    let (val2, carry2) = thumb_expand_imm_c(0x042, true);
    assert_eq!(val2, 0x0000_0042);
    assert!(carry2); // carry_in unchanged
}

#[test]
fn thumb_expand_imm_pattern_01() {
    // imm12 = 0b00_01_10101011 = 0x1AB → pattern 01, imm8 = 0xAB
    let (val, carry) = thumb_expand_imm_c(0x1AB, false);
    assert_eq!(val, 0x00AB_00AB);
    assert!(!carry);
}

#[test]
fn thumb_expand_imm_pattern_10() {
    // imm12 = 0b00_10_11001101 = 0x2CD → pattern 10, imm8 = 0xCD
    let (val, carry) = thumb_expand_imm_c(0x2CD, true);
    assert_eq!(val, 0xCD00_CD00);
    assert!(carry); // carry_in unchanged
}

#[test]
fn thumb_expand_imm_pattern_11() {
    // imm12 = 0b00_11_11101111 = 0x3EF → pattern 11, imm8 = 0xEF
    let (val, carry) = thumb_expand_imm_c(0x3EF, false);
    assert_eq!(val, 0xEFEF_EFEF);
    assert!(!carry);
}

#[test]
fn thumb_expand_imm_rotation_no_carry() {
    // Rotation path: imm12[11:10] != 00.
    // imm12 = 0xF80 = 0b1111_1000_0000
    //   unrotated = 0x80 | (0xF80 & 0x7F) = 0x80 | 0x00 = 0x80
    //   rotation = (0xF80 >> 7) & 0x1F = 0x1F = 31
    //   val = 0x80.rotate_right(31) = 0x00000100
    //   carry = val >> 31 = 0
    let (val, carry) = thumb_expand_imm_c(0xF80, false);
    assert_eq!(val, 0x0000_0100);
    assert!(!carry);
}

#[test]
fn thumb_expand_imm_rotation_with_carry() {
    // Rotation path producing MSB=1.
    // imm12 = 0x480 = 0b0100_1000_0000
    //   unrotated = 0x80 | (0x480 & 0x7F) = 0x80 | 0x00 = 0x80
    //   rotation = (0x480 >> 7) & 0x1F = 0x09
    //   val = 0x80.rotate_right(9) = 0x80 >> 9 | 0x80 << 23 = 0x40000000
    //   carry = val >> 31 = 0 ... need MSB=1.
    // Let's use rotation=1: imm12 must have bits[11:7] = 00001 and bits[11:10] != 00.
    // imm12 = 0b0100_0000_0000 = 0x400
    //   unrotated = 0x80 | 0 = 0x80
    //   rotation = (0x400 >> 7) & 0x1F = 0x08
    //   val = 0x80.rotate_right(8) = 0x80000000
    //   carry = 1
    let (val, carry) = thumb_expand_imm_c(0x400, false);
    assert_eq!(val, 0x8000_0000);
    assert!(carry);
}

#[test]
fn thumb_expand_imm_convenience() {
    // thumb_expand_imm discards carry
    assert_eq!(thumb_expand_imm(0x042), 0x0000_0042);
    assert_eq!(thumb_expand_imm(0x1AB), 0x00AB_00AB);
    assert_eq!(thumb_expand_imm(0x400), 0x8000_0000);
}

#[test]
fn extract_imm12_basic() {
    // hw0[10] = i, hw1[14:12] = imm3, hw1[7:0] = imm8
    // Test: i=1, imm3=0b101, imm8=0x42
    // imm12 = (1 << 11) | (0b101 << 8) | 0x42 = 0x800 | 0x500 | 0x42 = 0xD42
    let hw0: u16 = 1 << 10; // i=1
    let hw1: u16 = (0b101 << 12) | 0x42;
    assert_eq!(extract_imm12(hw0, hw1), 0xD42);
}

// ============================================================================
// CBZ / CBNZ
// ============================================================================

#[test]
fn cbz_taken() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    c.set_reg(0, 0); // R0 = 0 → CBZ should branch

    // CBZ R0, +8: opcode = 1011_0_0_0_1_00100_000
    // bit 11=0 (CBZ), i=0, imm5=4 (offset = 4<<1 = 8)
    // 10110_0_0_1_00100_000 = 0xB100 | (4 << 3) = 0xB120
    // Actually: 1011_n_0_i_1_imm5_Rn where n=bit11, i=bit9
    // CBZ: 1011_0_0_0_1_imm5_Rn
    // imm5=00100=4, Rn=000 → 10110001_00100_000 = 0xB120
    let cy = c.execute_one(0xB120);
    // read_pc() = 0x1000 + 4 = 0x1004, target = 0x1004 + 8 = 0x100C
    assert_eq!(c.regs.pc(), 0x100C);
    assert_eq!(cy, 2);
}

#[test]
fn cbz_not_taken() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    c.set_reg(0, 1); // R0 = 1 → CBZ should NOT branch

    let cy = c.execute_one(0xB120); // CBZ R0, +8
    // PC not changed (beyond the +2 from execute_one setup)
    assert_eq!(c.regs.pc(), 0x1002);
    assert_eq!(cy, 1);
}

#[test]
fn cbnz_not_taken() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    c.set_reg(0, 0); // R0 = 0 → CBNZ should NOT branch

    // CBNZ R0, +8: bit11=1
    // 1011_1_0_0_1_00100_000 = 0xB920
    let cy = c.execute_one(0xB920);
    assert_eq!(c.regs.pc(), 0x1002);
    assert_eq!(cy, 1);
}

#[test]
fn cbnz_taken() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    c.set_reg(0, 5); // R0 = 5 → CBNZ should branch

    let cy = c.execute_one(0xB920); // CBNZ R0, +8
    assert_eq!(c.regs.pc(), 0x100C);
    assert_eq!(cy, 2);
}

// ============================================================================
// Thumb-32 decode tree routing
// ============================================================================

#[test]
fn thumb32_movw_routes_to_stub() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // MOVW R0, #0x0000: hw0=0xF240, hw1=0x0000
    // op1 = (0xF240 >> 11) & 0x3 = 0b10
    // op2 = (0xF240 >> 4) & 0x7F = 0x24 = 0b0100100
    // op  = (0x0000 >> 15) & 1 = 0
    // → op1=10, op=0, op2 & 0x20 = 0x20 → dp_plain_imm (stub → undefined → 1)
    let cy = c.execute_one_wide(0xF240, 0x0000);
    assert_eq!(cy, 1); // stub returns 1
}

#[test]
fn thumb32_bl_routes_through_branch_misc() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // BL +100: same encoding as the existing bl_forward test
    let cy = c.execute_one_wide(0xF000, 0xF832);
    assert_eq!(c.regs.lr(), 0x1005);
    assert_eq!(c.regs.pc(), 0x1068);
    assert_eq!(cy, 4);
}

#[test]
fn thumb32_ldr_w_routes_to_load_store_single() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // LDR.W R0, [R1, #0]: hw0=0xF8D1, hw1=0x0000
    // op1 = (0xF8D1 >> 11) & 0x3 = 0b11
    // op2 = (0xF8D1 >> 4) & 0x7F = 0x0D = 0b0001101
    // op2 & 0x40 = 0, op2 & 0x20 = 0 → load_store_single → load costs 2
    let cy = c.execute_one_wide(0xF8D1, 0x0000);
    assert_eq!(cy, 2);
}

// ============================================================================
// Thumb-32: Data Processing (Modified Immediate)
// ============================================================================

/// Encode a data-processing (modified immediate) instruction.
/// Format: 11110_i_0_op[3:0]_S_Rn  0_imm3_Rd_imm8
fn encode_dp_mod_imm(op: u8, s: bool, rn: u8, rd: u8, imm12: u32) -> (u16, u16) {
    let i = ((imm12 >> 11) & 1) as u16;
    let imm3 = ((imm12 >> 8) & 0x7) as u16;
    let imm8 = (imm12 & 0xFF) as u16;
    let hw0 = 0xF000 | (i << 10) | ((op as u16) << 5) | ((s as u16) << 4) | (rn as u16);
    let hw1 = (imm3 << 12) | ((rd as u16) << 8) | imm8;
    (hw0, hw1)
}

#[test]
fn adds_w_imm() {
    // ADDS.W R0, R1, #256
    // imm12 for 256: byte-replication mode 00, imm8=0 won't work.
    // 256 = 0x100 → imm12 = 0x100 (mode 01: 0x00ii00ii with imm8=0 is 0,
    // that's wrong). Use rotation: 0x80 rotated right by 24 → 0x100.
    // rotation=24, imm12[11:7]=24=0b11000, imm12[6:0]=0 (unrotated=0x80).
    // imm12 = 0b110_0000_0000_0 = 0xC00. Wait, let me recalculate.
    // For rotation path: imm12[11:10] != 00, unrotated = 0x80 | imm12[6:0],
    // rotation = imm12[11:7].
    // We want unrotated = 0x80 (imm12[6:0] = 0), rotation = 24.
    // imm12 = (24 << 7) | 0 = 0xC00. But imm12 is 12 bits (0..0xFFF).
    // 24 << 7 = 0xC00, yes that fits. Check: imm12[11:10] = 0b11 != 00 → rotation path.
    // unrotated = 0x80, rotation = (0xC00 >> 7) & 0x1F = 24.
    // 0x80.rotate_right(24) = 0x80 << 8 = 0x8000. That's not 256.
    //
    // Actually: 256 = 0x100. Let's just use imm12 = 0x100 directly.
    // imm12 = 0x100: imm12[11:10] = 0b00 → byte replication.
    // (imm12 >> 8) & 0x3 = 1 → mode 01: val = (imm8 << 16) | imm8.
    // imm8 = 0 → val = 0. That's wrong.
    //
    // OK, simplest: use a small constant. #42 = imm12 = 0x2A (mode 00: val=0x2A).
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    let (hw0, hw1) = encode_dp_mod_imm(0b1000, true, 1, 0, 0x2A); // ADDS.W R0, R1, #42
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 142);
    assert!(!c.flag_n());
    assert!(!c.flag_z());
    assert!(!c.flag_c());
    assert!(!c.flag_v());
    assert_eq!(cy, 1);
}

#[test]
fn subs_w_imm() {
    // SUBS.W R0, R1, #100
    let mut c = CortexM33::new();
    c.set_reg(1, 150);
    let (hw0, hw1) = encode_dp_mod_imm(0b1101, true, 1, 0, 100); // SUBS.W R0, R1, #100
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 50);
    assert!(!c.flag_n());
    assert!(!c.flag_z());
    assert!(c.flag_c()); // no borrow → carry set
    assert!(!c.flag_v());
    assert_eq!(cy, 1);
}

#[test]
fn and_w_imm_no_flags() {
    // AND.W R0, R1, #0xFF (S=0, no flag update)
    let mut c = CortexM33::new();
    c.set_reg(1, 0x1234_5678);
    c.regs.set_flag_n(true); // pre-set flags to verify they don't change
    c.regs.set_flag_z(true);
    let (hw0, hw1) = encode_dp_mod_imm(0b0000, false, 1, 0, 0xFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x78);
    assert!(c.flag_n()); // unchanged
    assert!(c.flag_z()); // unchanged
    assert_eq!(cy, 1);
}

#[test]
fn ands_w_imm_carry() {
    // ANDS.W with rotation immediate to test carry from ThumbExpandImm.
    // Use imm12 = 0xC00: rotation path. unrotated=0x80, rotation=24.
    // val = 0x80.rotate_right(24) = 0x8000. carry = val >> 31 = 0.
    // Actually: 0x80 rotated right by 24 = 0x80 << (32-24) = 0x80 << 8 = 0x8000.
    // val = 0x8000, carry = (0x8000 >> 31) = 0 → false.
    //
    // Let's use rotation=1: imm12[11:7]=1, imm12[6:0]=0.
    // imm12 = (1 << 7) = 0x80. Check bits [11:10]: (0x80 >> 10) = 0 → that's 00,
    // byte replication path. Need imm12[11:10] != 00.
    //
    // rotation=8: imm12 = (8 << 7) | 0 = 0x400. bits[11:10] = 0b01 → rotation path.
    // unrotated = 0x80, rotation = 8. val = 0x80.rotate_right(8) = 0x80000000.
    // carry = val >> 31 = 1 → true.
    let mut c = CortexM33::new();
    c.set_reg(1, 0xFFFF_FFFF);
    let (hw0, hw1) = encode_dp_mod_imm(0b0000, true, 1, 0, 0x400);
    let cy = c.execute_one_wide(hw0, hw1);
    // imm32 = 0x8000_0000
    assert_eq!(c.reg(0), 0x8000_0000); // 0xFFFFFFFF & 0x80000000
    assert!(c.flag_n());  // bit 31 set
    assert!(!c.flag_z());
    assert!(c.flag_c());  // carry from ThumbExpandImm rotation
    assert_eq!(cy, 1);
}

#[test]
fn mov_w_imm() {
    // MOV.W R0, #imm via ORR with Rn=15, S=0
    // Use imm12 = 0x34 → imm32 = 0x34
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_dp_mod_imm(0b0010, false, 15, 0, 0x34);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x34);
    assert_eq!(cy, 1);
}

#[test]
fn mvn_w_imm() {
    // MVN.W R0, #0 → R0 = 0xFFFFFFFF (via ORN with Rn=15, S=0)
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_dp_mod_imm(0b0011, false, 15, 0, 0x00);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xFFFF_FFFF);
    assert_eq!(cy, 1);
}

#[test]
fn cmp_w_imm() {
    // CMP.W R0, #50 → SUB with S=1, Rd=15 (discard result, flags only)
    let mut c = CortexM33::new();
    c.set_reg(0, 50);
    let (hw0, hw1) = encode_dp_mod_imm(0b1101, true, 0, 15, 50);
    let cy = c.execute_one_wide(hw0, hw1);
    // 50 - 50 = 0 → Z=1, C=1 (no borrow), N=0, V=0
    assert!(!c.flag_n());
    assert!(c.flag_z());
    assert!(c.flag_c());
    assert!(!c.flag_v());
    // Rd=15, so R15 should NOT have been changed to the result (0).
    // R15 was set by execute_one_wide to pc+4. Verify it's still there.
    assert_ne!(c.reg(15), 0);
    assert_eq!(cy, 1);
}

#[test]
fn tst_w_imm() {
    // TST.W R0, #0xFF → AND with S=1, Rd=15 (discard result, flags only)
    let mut c = CortexM33::new();
    c.set_reg(0, 0x100); // bit 8 set, low byte = 0
    let (hw0, hw1) = encode_dp_mod_imm(0b0000, true, 0, 15, 0xFF);
    let cy = c.execute_one_wide(hw0, hw1);
    // 0x100 & 0xFF = 0 → Z=1, N=0
    assert!(!c.flag_n());
    assert!(c.flag_z());
    assert_eq!(cy, 1);
}

#[test]
fn orr_w_imm() {
    // ORR.W R0, R1, #0xFF00
    // 0xFF00 = 0xFF << 8. Using byte-replication mode 10: (imm8 << 24) | (imm8 << 8).
    // mode 10: imm12[9:8] = 0b10, imm8 = 0xFF → val = (0xFF << 24) | (0xFF << 8) = 0xFF00FF00.
    // That's not 0xFF00. Let me use rotation instead.
    // 0xFF00 = 0xFF shifted left by 8 = 0x80|0x7F rotated right by 24.
    // unrotated = 0xFF (0x80 | 0x7F), rotation = 24.
    // imm12 = (24 << 7) | 0x7F = 0xC7F. Check bits: 0xC7F >> 10 = 3 → != 00 → rotation path.
    // rotation = (0xC7F >> 7) & 0x1F = 24. unrotated = 0x80 | 0x7F = 0xFF.
    // val = 0xFF.rotate_right(24) = 0xFF << 8 = 0xFF00. Correct!
    let mut c = CortexM33::new();
    c.set_reg(1, 0x1234_0000);
    let (hw0, hw1) = encode_dp_mod_imm(0b0010, false, 1, 0, 0xC7F);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x1234_FF00);
    assert_eq!(cy, 1);
}

#[test]
fn bic_w_imm() {
    // BIC.W R0, R1, #0x0F → R0 = R1 & ~0x0F (clear low nibble)
    let mut c = CortexM33::new();
    c.set_reg(1, 0xABCD_EF9A);
    let (hw0, hw1) = encode_dp_mod_imm(0b0001, false, 1, 0, 0x0F);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xABCD_EF90);
    assert_eq!(cy, 1);
}

#[test]
fn adc_w_imm() {
    // ADCS.W R0, R1, #10 with carry-in = 1
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.regs.set_flag_c(true); // carry-in
    let (hw0, hw1) = encode_dp_mod_imm(0b1010, true, 1, 0, 10);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 111); // 100 + 10 + 1
    assert!(!c.flag_n());
    assert!(!c.flag_z());
    assert!(!c.flag_c());
    assert!(!c.flag_v());
    assert_eq!(cy, 1);
}

#[test]
fn sbc_w_imm() {
    // SBCS.W R0, R1, #10 with carry-in = 1 (no borrow)
    // SBC: Rd = Rn + ~imm32 + C
    // 100 + ~10 + 1 = 100 + 0xFFFFFFF5 + 1 = 100 - 10 = 90
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.regs.set_flag_c(true);
    let (hw0, hw1) = encode_dp_mod_imm(0b1011, true, 1, 0, 10);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 90);
    assert!(!c.flag_n());
    assert!(!c.flag_z());
    assert!(c.flag_c()); // no borrow
    assert!(!c.flag_v());
    assert_eq!(cy, 1);
}

#[test]
fn rsb_w_imm() {
    // RSBS.W R0, R1, #100 → R0 = 100 - R1 = 100 - 30 = 70
    // RSB: Rd = ~Rn + imm32 + 1
    let mut c = CortexM33::new();
    c.set_reg(1, 30);
    let (hw0, hw1) = encode_dp_mod_imm(0b1110, true, 1, 0, 100);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 70);
    assert!(!c.flag_n());
    assert!(!c.flag_z());
    assert!(c.flag_c()); // no borrow
    assert!(!c.flag_v());
    assert_eq!(cy, 1);
}

#[test]
fn eor_w_imm() {
    // EOR.W R0, R1, #0xFF (S=0)
    let mut c = CortexM33::new();
    c.set_reg(1, 0xAA);
    let (hw0, hw1) = encode_dp_mod_imm(0b0100, false, 1, 0, 0xFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xAA ^ 0xFF); // 0x55
    assert_eq!(cy, 1);
}

#[test]
fn orn_w_imm() {
    // ORN.W R0, R1, #0xFF → R0 = R1 | ~0xFF = R1 | 0xFFFFFF00
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0042);
    let (hw0, hw1) = encode_dp_mod_imm(0b0011, false, 1, 0, 0xFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xFFFF_FF42);
    assert_eq!(cy, 1);
}

// ============================================================================
// Thumb-32: Data Processing (Plain Binary Immediate)
// ============================================================================

/// Encode MOVW Rd, #imm16.
/// Format: 11110_i_10_0100_0_imm4  0_imm3_Rd_imm8
/// imm16 = imm4:i:imm3:imm8
fn encode_movw(rd: u8, imm16: u16) -> (u16, u16) {
    let imm4 = (imm16 >> 12) & 0xF;
    let i = (imm16 >> 11) & 1;
    let imm3 = (imm16 >> 8) & 0x7;
    let imm8 = imm16 & 0xFF;
    let hw0: u16 = 0xF200 | ((0b00100u16) << 4) | (i << 10) | imm4;
    let hw1: u16 = (imm3 << 12) | ((rd as u16) << 8) | imm8;
    (hw0, hw1)
}

/// Encode MOVT Rd, #imm16.
/// Format: 11110_i_10_1100_0_imm4  0_imm3_Rd_imm8
fn encode_movt(rd: u8, imm16: u16) -> (u16, u16) {
    let imm4 = (imm16 >> 12) & 0xF;
    let i = (imm16 >> 11) & 1;
    let imm3 = (imm16 >> 8) & 0x7;
    let imm8 = imm16 & 0xFF;
    let hw0: u16 = 0xF200 | ((0b01100u16) << 4) | (i << 10) | imm4;
    let hw1: u16 = (imm3 << 12) | ((rd as u16) << 8) | imm8;
    (hw0, hw1)
}

/// Encode ADDW Rd, Rn, #imm12 (or ADR when Rn=15).
/// Format: 11110_i_10_0000_0_Rn  0_imm3_Rd_imm8
fn encode_addw(rd: u8, rn: u8, imm12: u16) -> (u16, u16) {
    let i = (imm12 >> 11) & 1;
    let imm3 = (imm12 >> 8) & 0x7;
    let imm8 = imm12 & 0xFF;
    let hw0: u16 = 0xF200 | ((0b00000u16) << 4) | (i << 10) | (rn as u16);
    let hw1: u16 = (imm3 << 12) | ((rd as u16) << 8) | imm8;
    (hw0, hw1)
}

/// Encode SUBW Rd, Rn, #imm12 (or ADR when Rn=15).
/// Format: 11110_i_10_1010_0_Rn  0_imm3_Rd_imm8
fn encode_subw(rd: u8, rn: u8, imm12: u16) -> (u16, u16) {
    let i = (imm12 >> 11) & 1;
    let imm3 = (imm12 >> 8) & 0x7;
    let imm8 = imm12 & 0xFF;
    let hw0: u16 = 0xF200 | ((0b01010u16) << 4) | (i << 10) | (rn as u16);
    let hw1: u16 = (imm3 << 12) | ((rd as u16) << 8) | imm8;
    (hw0, hw1)
}

/// Encode BFI Rd, Rn, #lsb, #width (or BFC when Rn=15).
/// Format: 11110_0_11_0110_0_Rn  0_imm3_Rd_imm2_0_msb[4:0]
/// op=0b10110, lsb = imm3:imm2, msb = lsb + width - 1
fn encode_bfi(rd: u8, rn: u8, lsb: u8, width: u8) -> (u16, u16) {
    let msb = lsb + width - 1;
    let imm3 = ((lsb >> 2) & 0x7) as u16;
    let imm2 = (lsb & 0x3) as u16;
    let hw0: u16 = 0xF200 | ((0b10110u16) << 4) | (rn as u16);
    let hw1: u16 = (imm3 << 12) | ((rd as u16) << 8) | (imm2 << 6) | (msb as u16);
    (hw0, hw1)
}

/// Encode UBFX Rd, Rn, #lsb, #width.
/// Format: 11110_0_11_1100_0_Rn  0_imm3_Rd_imm2_0_widthm1[4:0]
fn encode_ubfx(rd: u8, rn: u8, lsb: u8, width: u8) -> (u16, u16) {
    let widthm1 = width - 1;
    let imm3 = ((lsb >> 2) & 0x7) as u16;
    let imm2 = (lsb & 0x3) as u16;
    let hw0: u16 = 0xF200 | ((0b11100u16) << 4) | (rn as u16);
    let hw1: u16 = (imm3 << 12) | ((rd as u16) << 8) | (imm2 << 6) | (widthm1 as u16);
    (hw0, hw1)
}

/// Encode SBFX Rd, Rn, #lsb, #width.
/// Format: 11110_0_11_0100_0_Rn  0_imm3_Rd_imm2_0_widthm1[4:0]
fn encode_sbfx(rd: u8, rn: u8, lsb: u8, width: u8) -> (u16, u16) {
    let widthm1 = width - 1;
    let imm3 = ((lsb >> 2) & 0x7) as u16;
    let imm2 = (lsb & 0x3) as u16;
    let hw0: u16 = 0xF200 | ((0b10100u16) << 4) | (rn as u16);
    let hw1: u16 = (imm3 << 12) | ((rd as u16) << 8) | (imm2 << 6) | (widthm1 as u16);
    (hw0, hw1)
}

#[test]
fn movw_basic() {
    // MOVW R0, #0x1234
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_movw(0, 0x1234);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x1234);
    assert_eq!(cy, 1);
}

#[test]
fn movw_all_bits() {
    // MOVW R0, #0xFFFF — all 16 bits set
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_movw(0, 0xFFFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x0000_FFFF);
    assert_eq!(cy, 1);
}

#[test]
fn movt_basic() {
    // MOVT R0, #0xABCD — set top half, preserve bottom half
    let mut c = CortexM33::new();
    c.set_reg(0, 0x0000_5678);
    let (hw0, hw1) = encode_movt(0, 0xABCD);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xABCD_5678);
    assert_eq!(cy, 1);
}

#[test]
fn movw_movt_pair() {
    // Load 0xDEADBEEF via MOVW + MOVT
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_movw(0, 0xBEEF);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x0000_BEEF);

    let (hw0, hw1) = encode_movt(0, 0xDEAD);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xDEAD_BEEF);
}

#[test]
fn addw_basic() {
    // ADDW R0, R1, #4000
    let mut c = CortexM33::new();
    c.set_reg(1, 1000);
    let (hw0, hw1) = encode_addw(0, 1, 4000);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 5000);
    assert_eq!(cy, 1);
}

#[test]
fn subw_basic() {
    // SUBW R0, R1, #2000
    let mut c = CortexM33::new();
    c.set_reg(1, 5000);
    let (hw0, hw1) = encode_subw(0, 1, 2000);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 3000);
    assert_eq!(cy, 1);
}

#[test]
fn adr_add() {
    // ADR R0, [PC, #100] — ADDW with Rn=15
    // PC=0x1000, read_pc = 0x1000 + 4 = 0x1004, Align(0x1004, 4) = 0x1004
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    let (hw0, hw1) = encode_addw(0, 15, 100);
    let cy = c.execute_one_wide(hw0, hw1);
    // read_pc = current_instr_addr + 4 = 0x1000 + 4 = 0x1004
    // Align(0x1004, 4) = 0x1004, result = 0x1004 + 100 = 0x1068
    assert_eq!(c.reg(0), 0x1068);
    assert_eq!(cy, 1);
}

#[test]
fn adr_sub() {
    // ADR R0, [PC, #-100] — SUBW with Rn=15
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    let (hw0, hw1) = encode_subw(0, 15, 100);
    let cy = c.execute_one_wide(hw0, hw1);
    // read_pc = 0x1004, Align = 0x1004, result = 0x1004 - 100 = 0x0FA0
    assert_eq!(c.reg(0), 0x0FA0);
    assert_eq!(cy, 1);
}

#[test]
fn bfi_basic() {
    // BFI R0, R1, #4, #8 — insert bits [11:4] from R1 into R0
    let mut c = CortexM33::new();
    c.set_reg(0, 0xFFFF_FFFF);
    c.set_reg(1, 0xAB);       // low 8 bits = 0xAB
    let (hw0, hw1) = encode_bfi(0, 1, 4, 8);
    let cy = c.execute_one_wide(hw0, hw1);
    // mask = 0xFF << 4 = 0xFF0
    // result = (0xFFFFFFFF & !0xFF0) | ((0xAB << 4) & 0xFF0)
    //        = 0xFFFFF00F | 0xAB0 = 0xFFFFFABF
    assert_eq!(c.reg(0), 0xFFFF_FABF);
    assert_eq!(cy, 1);
}

#[test]
fn bfc_basic() {
    // BFC R0, #8, #4 — clear bits [11:8] (Rn=15)
    let mut c = CortexM33::new();
    c.set_reg(0, 0xFFFF_FFFF);
    let (hw0, hw1) = encode_bfi(0, 15, 8, 4);
    let cy = c.execute_one_wide(hw0, hw1);
    // mask = 0xF << 8 = 0xF00
    // result = 0xFFFFFFFF & !0xF00 = 0xFFFFF0FF
    assert_eq!(c.reg(0), 0xFFFF_F0FF);
    assert_eq!(cy, 1);
}

#[test]
fn ubfx_basic() {
    // UBFX R0, R1, #4, #8 — extract bits [11:4] unsigned
    let mut c = CortexM33::new();
    c.set_reg(1, 0xDEAD_BEEF);
    let (hw0, hw1) = encode_ubfx(0, 1, 4, 8);
    let cy = c.execute_one_wide(hw0, hw1);
    // (0xDEADBEEF >> 4) & 0xFF = 0x0DEADBEE & 0xFF = 0xEE
    assert_eq!(c.reg(0), 0xEE);
    assert_eq!(cy, 1);
}

#[test]
fn sbfx_positive() {
    // SBFX R0, R1, #4, #8 — extract bits [11:4] signed, positive value
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0750); // bits [11:4] = 0x75 = 0b0111_0101 (positive)
    let (hw0, hw1) = encode_sbfx(0, 1, 4, 8);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x75);
    assert_eq!(cy, 1);
}

#[test]
fn sbfx_negative() {
    // SBFX R0, R1, #4, #8 — extract bits [11:4] signed, negative value
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0F50); // bits [11:4] = 0xF5 = 0b1111_0101 (negative in 8-bit)
    let (hw0, hw1) = encode_sbfx(0, 1, 4, 8);
    let cy = c.execute_one_wide(hw0, hw1);
    // sign_extend(0xF5, 8) = 0xFFFF_FFF5
    assert_eq!(c.reg(0), 0xFFFF_FFF5);
    assert_eq!(cy, 1);
}

// ============================================================================
// Thumb-32: Load/Store Single — encoding helpers
// ============================================================================

// hw0 format for load_store_single (imm12):
//   hw0[15:9] = 1111100, hw0[8] = sign, hw0[7] = 1 (imm12),
//   hw0[6:5] = size, hw0[4] = load, hw0[3:0] = Rn
//   hw1[15:12] = Rt, hw1[11:0] = imm12

/// Encode LDR.W Rt, [Rn, #imm12] — word load, unsigned 12-bit offset.
fn encode_ldr_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=10, load=1, sign=0, hw0[7]=1
    let hw0 = 0xF8D0 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode STR.W Rt, [Rn, #imm12] — word store, unsigned 12-bit offset.
fn encode_str_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=10, load=0, sign=0, hw0[7]=1
    let hw0 = 0xF8C0 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode LDR.W Rt, [Rn, Rm, LSL #shift] — word load, register offset.
fn encode_ldr_w_reg(rt: u8, rn: u8, rm: u8, shift: u8) -> (u16, u16) {
    // size=10, load=1, sign=0, hw0[7]=0
    let hw0 = 0xF850 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | ((shift as u16 & 0x3) << 4) | (rm as u16 & 0xF);
    (hw0, hw1)
}

/// Encode LDRB.W Rt, [Rn, #imm12] — unsigned byte load.
fn encode_ldrb_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=00, load=1, sign=0, hw0[7]=1
    let hw0 = 0xF890 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode STRB.W Rt, [Rn, #imm12] — byte store.
fn encode_strb_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=00, load=0, sign=0, hw0[7]=1
    let hw0 = 0xF880 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode LDRH.W Rt, [Rn, #imm12] — unsigned halfword load.
fn encode_ldrh_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=01, load=1, sign=0, hw0[7]=1
    let hw0 = 0xF8B0 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode STRH.W Rt, [Rn, #imm12] — halfword store.
fn encode_strh_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=01, load=0, sign=0, hw0[7]=1
    let hw0 = 0xF8A0 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode LDRSB.W Rt, [Rn, #imm12] — signed byte load.
fn encode_ldrsb_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=00, load=1, sign=1, hw0[7]=1
    let hw0 = 0xF990 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode LDRSH.W Rt, [Rn, #imm12] — signed halfword load.
fn encode_ldrsh_w_imm12(rt: u8, rn: u8, imm12: u16) -> (u16, u16) {
    // size=01, load=1, sign=1, hw0[7]=1
    let hw0 = 0xF9B0 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12) | (imm12 & 0xFFF);
    (hw0, hw1)
}

/// Encode LDR.W Rt, [Rn, #imm8] with P/U/W bits (pre/post-index).
/// p=true, u=direction, w=writeback for pre-index; p=false for post-index.
fn encode_ldr_w_imm8_puw(rt: u8, rn: u8, imm8: u8, p: bool, u: bool, w: bool) -> (u16, u16) {
    // size=10, load=1, sign=0, hw0[7]=0
    let hw0 = 0xF850 | (rn as u16 & 0xF);
    let hw1 = ((rt as u16 & 0xF) << 12)
        | 0x800 // hw1[11]=1 selects imm8 mode
        | if p { 0x400 } else { 0 }
        | if u { 0x200 } else { 0 }
        | if w { 0x100 } else { 0 }
        | (imm8 as u16);
    (hw0, hw1)
}

// ============================================================================
// Thumb-32: Load/Store Single — tests
// ============================================================================

#[test]
fn ldr_w_imm12() {
    // LDR.W R0, [R1, #100]
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0064, 0xDEAD_BEEF);
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_ldr_w_imm12(0, 1, 100);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0xDEAD_BEEF);
    assert_eq!(cy, 2);
}

#[test]
fn str_w_imm12() {
    // STR.W R0, [R1, #100] then verify with read
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xCAFE_BABE);
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_str_w_imm12(0, 1, 100);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(bus.read32(0x2000_0064), 0xCAFE_BABE);
    assert_eq!(cy, 1);
}

#[test]
fn ldr_w_reg() {
    // LDR.W R0, [R1, R2, LSL #2] — array indexing pattern
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0010, 0x1234_5678); // array[4] at base + 4*4
    c.set_reg(1, 0x2000_0000); // base
    c.set_reg(2, 4);           // index
    let (hw0, hw1) = encode_ldr_w_reg(0, 1, 2, 2); // LSL #2
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0x1234_5678);
    assert_eq!(cy, 2);
}

#[test]
fn ldrb_w_imm12() {
    // LDRB.W R0, [R1, #10] — unsigned byte load
    let (mut c, mut bus) = core_and_bus();
    bus.write8(0x2000_000A, 0xAB);
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_ldrb_w_imm12(0, 1, 10);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0xAB); // zero-extended
    assert_eq!(cy, 2);
}

#[test]
fn ldrh_w_imm12() {
    // LDRH.W R0, [R1, #6] — unsigned halfword load
    let (mut c, mut bus) = core_and_bus();
    bus.write16(0x2000_0006, 0xBEEF);
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_ldrh_w_imm12(0, 1, 6);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0xBEEF); // zero-extended
    assert_eq!(cy, 2);
}

#[test]
fn ldrsb_w_imm12() {
    // LDRSB.W R0, [R1, #0] — signed byte, negative value
    let (mut c, mut bus) = core_and_bus();
    bus.write8(0x2000_0000, 0x80); // -128 signed
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_ldrsb_w_imm12(0, 1, 0);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0xFFFF_FF80); // sign-extended
    assert_eq!(cy, 2);
}

#[test]
fn ldrsh_w_imm12() {
    // LDRSH.W R0, [R1, #2] — signed halfword, negative value
    let (mut c, mut bus) = core_and_bus();
    bus.write16(0x2000_0002, 0x8001); // -32767 signed
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_ldrsh_w_imm12(0, 1, 2);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0xFFFF_8001); // sign-extended
    assert_eq!(cy, 2);
}

#[test]
fn strb_w_imm12() {
    // STRB.W R0, [R1, #5] — byte store
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xFFFF_FF42); // only low byte stored
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_strb_w_imm12(0, 1, 5);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(bus.read8(0x2000_0005), 0x42);
    assert_eq!(cy, 1);
}

#[test]
fn strh_w_imm12() {
    // STRH.W R0, [R1, #8] — halfword store
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xFFFF_BEEF); // only low halfword stored
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_strh_w_imm12(0, 1, 8);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(bus.read16(0x2000_0008), 0xBEEF);
    assert_eq!(cy, 1);
}

#[test]
fn ldr_w_literal() {
    // LDR.W R0, [PC, #imm12] — PC-relative literal load (Rn=15)
    // PC must be in SRAM so the literal address is also in SRAM (writable).
    let (mut c, mut bus) = core_and_bus();
    c.regs.set_pc(0x2000_1000);
    // read_pc() = instr_addr + 4 = 0x2000_1000 + 4 = 0x2000_1004, aligned = 0x2000_1004
    // With imm12=8, addr = 0x2000_1004 + 8 = 0x2000_100C
    bus.write32(0x2000_100C, 0xAAAA_BBBB);
    // Rn=15 with U=1 (hw0[7]=1): LDR.W R0, [PC, #+imm12]
    let hw0: u16 = 0xF8DF; // sign=0, hw0[7]=1, size=10, load=1, Rn=1111
    let hw1: u16 = 0x0008; // Rt=R0, imm12=8
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0xAAAA_BBBB);
    assert_eq!(cy, 2);
}

#[test]
fn ldr_w_pre_index() {
    // LDR.W R0, [R1, #4]! — pre-index with writeback
    // P=1, U=1, W=1, imm8=4
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0004, 0x1111_2222);
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_ldr_w_imm8_puw(0, 1, 4, true, true, true);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0x1111_2222);      // loaded from base+4
    assert_eq!(c.reg(1), 0x2000_0004);      // R1 updated (writeback)
    assert_eq!(cy, 2);
}

#[test]
fn ldr_w_post_index() {
    // LDR.W R0, [R1], #4 — post-index
    // P=0, U=1, W=1 (post-index: p=false implies writeback)
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0000, 0x3333_4444);
    c.set_reg(1, 0x2000_0000);
    let (hw0, hw1) = encode_ldr_w_imm8_puw(0, 1, 4, false, true, true);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0x3333_4444);      // loaded from original base
    assert_eq!(c.reg(1), 0x2000_0004);      // R1 updated after load
    assert_eq!(cy, 2);
}

#[test]
fn pld_rt15_is_nop() {
    // Load with Rt=15 is PLD/PLI (preload hint), treated as NOP.
    let (mut c, mut bus) = core_and_bus();
    c.regs.set_pc(0x1000);
    c.set_reg(1, 0x2000_0000);
    bus.write32(0x2000_0000, 0xDEAD_BEEF);
    // LDR.W R15, [R1, #0] → Rt=15 → PLD, returns 1
    let (hw0, _) = encode_ldr_w_imm12(15, 1, 0);
    let hw1: u16 = (15u16 << 12) | 0; // Rt=15, imm12=0
    let pc_before = c.regs.pc();
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    // PC should be at pc_before+4 (normal advance), not modified by load
    assert_eq!(c.regs.pc(), pc_before + 4);
    assert_eq!(cy, 1); // NOP cost
}

// ============================================================================
// Encoding helpers: wide branches
// ============================================================================

/// Encode B.W conditional (T3).
/// offset is a signed value in bytes (must be even, 21-bit signed range).
/// Format: hw0 = 11110_S_cond_imm6, hw1 = 10_J1_0_J2_imm11
fn encode_b_w_cond(cond: u8, offset: i32) -> (u16, u16) {
    let uoffset = offset as u32;
    let s = (uoffset >> 20) & 1;
    let j2 = (uoffset >> 19) & 1;
    let j1 = (uoffset >> 18) & 1;
    let imm6 = (uoffset >> 12) & 0x3F;
    let imm11 = (uoffset >> 1) & 0x7FF;

    let hw0 = 0xF000u16 | ((s as u16) << 10) | ((cond as u16) << 6) | imm6 as u16;
    let hw1 = 0x8000u16 | ((j1 as u16) << 13) | ((j2 as u16) << 11) | imm11 as u16;
    (hw0, hw1)
}

/// Encode B.W unconditional (T4).
/// offset is a signed value in bytes (must be even, 25-bit signed range).
/// Format: hw0 = 11110_S_imm10, hw1 = 10_J1_1_J2_imm11
/// Uses XOR trick: I1=NOT(J1^S), I2=NOT(J2^S), so J1=NOT(I1^S), J2=NOT(I2^S).
fn encode_b_w_uncond(offset: i32) -> (u16, u16) {
    let uoffset = offset as u32;
    let s = (uoffset >> 24) & 1;
    let i1 = (uoffset >> 23) & 1;
    let i2 = (uoffset >> 22) & 1;
    let imm10 = (uoffset >> 12) & 0x3FF;
    let imm11 = (uoffset >> 1) & 0x7FF;

    // Reverse the XOR trick: J1 = NOT(I1 XOR S), J2 = NOT(I2 XOR S)
    let j1 = (i1 ^ s) ^ 1;
    let j2 = (i2 ^ s) ^ 1;

    let hw0 = 0xF000u16 | ((s as u16) << 10) | imm10 as u16;
    let hw1 = 0x9000u16 | ((j1 as u16) << 13) | ((j2 as u16) << 11) | imm11 as u16;
    (hw0, hw1)
}

// ============================================================================
// Tests: B.W conditional (T3)
// ============================================================================

#[test]
fn b_w_cond_taken() {
    // BEQ.W +100 with Z=1 -> branch taken
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    c.regs.set_flag_z(true); // EQ condition met
    let (hw0, hw1) = encode_b_w_cond(0x0, 100); // cond=0 (EQ), offset=+100
    let cy = c.execute_one_wide(hw0, hw1);
    // read_pc = 0x1000 + 4 = 0x1004, target = 0x1004 + 100 = 0x1068
    assert_eq!(c.regs.pc(), 0x1068);
    assert_eq!(cy, 2);
}

#[test]
fn b_w_cond_not_taken() {
    // BEQ.W +100 with Z=0 -> not taken
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    c.regs.set_flag_z(false); // EQ condition not met
    let (hw0, hw1) = encode_b_w_cond(0x0, 100); // cond=0 (EQ), offset=+100
    let cy = c.execute_one_wide(hw0, hw1);
    // Not taken: PC stays at 0x1000 + 4 = 0x1004
    assert_eq!(c.regs.pc(), 0x1004);
    assert_eq!(cy, 1);
}

#[test]
fn b_w_cond_backward() {
    // BNE.W -50 with Z=0 (NE condition met) -> backward branch taken
    let mut c = CortexM33::new();
    c.regs.set_pc(0x2000);
    c.regs.set_flag_z(false); // NE condition met
    let (hw0, hw1) = encode_b_w_cond(0x1, -50); // cond=1 (NE), offset=-50
    let cy = c.execute_one_wide(hw0, hw1);
    // read_pc = 0x2000 + 4 = 0x2004, target = 0x2004 + (-50) = 0x1FD2
    assert_eq!(c.regs.pc(), 0x1FD2);
    assert_eq!(cy, 2);
}

// ============================================================================
// Tests: B.W unconditional (T4)
// ============================================================================

#[test]
fn b_w_uncond_forward() {
    // B.W +1000 (unconditional forward)
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    let (hw0, hw1) = encode_b_w_uncond(1000);
    let cy = c.execute_one_wide(hw0, hw1);
    // read_pc = 0x1000 + 4 = 0x1004, target = 0x1004 + 1000 = 0x13EC
    assert_eq!(c.regs.pc(), 0x13EC);
    assert_eq!(cy, 2);
}

#[test]
fn b_w_uncond_backward() {
    // B.W -100 (unconditional backward)
    let mut c = CortexM33::new();
    c.regs.set_pc(0x2000);
    let (hw0, hw1) = encode_b_w_uncond(-100);
    let cy = c.execute_one_wide(hw0, hw1);
    // read_pc = 0x2000 + 4 = 0x2004, target = 0x2004 + (-100) = 0x1FA0
    assert_eq!(c.regs.pc(), 0x1FA0);
    assert_eq!(cy, 2);
}

// ============================================================================
// Tests: BL through branch_misc dispatch
// ============================================================================

#[test]
fn bl_still_works() {
    // Verify existing BL functionality routes through the new dispatch
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    // BL +100: same encoding as the existing bl_forward test
    let cy = c.execute_one_wide(0xF000, 0xF832);
    assert_eq!(c.regs.lr(), 0x1005);
    assert_eq!(c.regs.pc(), 0x1068);
    assert_eq!(cy, 4);
}

// ============================================================================
// Tests: Miscellaneous control (hints, barriers)
// ============================================================================

#[test]
fn nop_w() {
    // NOP.W: hw0=0xF3AF, hw1=0x8000
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);
    let cy = c.execute_one_wide(0xF3AF, 0x8000);
    // PC should advance normally (set by execute_one_wide to 0x1004)
    assert_eq!(c.regs.pc(), 0x1004);
    assert_eq!(cy, 1);
}

#[test]
fn dsb_dmb_isb() {
    let mut c = CortexM33::new();

    // DSB: hw0=0xF3BF, hw1=0x8F4F (option=0xF, barrier_op=4)
    c.regs.set_pc(0x1000);
    let cy = c.execute_one_wide(0xF3BF, 0x8F4F);
    assert_eq!(cy, 1);

    // DMB: hw0=0xF3BF, hw1=0x8F5F (barrier_op=5)
    c.regs.set_pc(0x2000);
    let cy = c.execute_one_wide(0xF3BF, 0x8F5F);
    assert_eq!(cy, 1);

    // ISB: hw0=0xF3BF, hw1=0x8F6F (barrier_op=6)
    c.regs.set_pc(0x3000);
    let cy = c.execute_one_wide(0xF3BF, 0x8F6F);
    assert_eq!(cy, 1);
}

// ============================================================================
// Tests: Thumb-32 load/store multiple (LDM.W, STM.W, PUSH.W, POP.W)
// ============================================================================

// Encoding helpers for Thumb-32 LDM/STM
fn encode_stmia_w(rn: u8, w: bool, reglist: u16) -> (u16, u16) {
    let hw0 = 0xE880 | ((w as u16) << 5) | rn as u16;
    (hw0, reglist)
}

fn encode_ldmia_w(rn: u8, w: bool, reglist: u16) -> (u16, u16) {
    let hw0 = 0xE890 | ((w as u16) << 5) | rn as u16;
    (hw0, reglist)
}

fn encode_stmdb_w(rn: u8, w: bool, reglist: u16) -> (u16, u16) {
    let hw0 = 0xE900 | ((w as u16) << 5) | rn as u16;
    (hw0, reglist)
}

#[allow(dead_code)] // Available for future LDMDB tests
fn encode_ldmdb_w(rn: u8, w: bool, reglist: u16) -> (u16, u16) {
    let hw0 = 0xE910 | ((w as u16) << 5) | rn as u16;
    (hw0, reglist)
}

#[test]
fn stm_w_ia() {
    // STMIA.W R4!, {R0-R3} — store 4 regs starting at R4, writeback
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xAAAA_0000);
    c.set_reg(1, 0xBBBB_1111);
    c.set_reg(2, 0xCCCC_2222);
    c.set_reg(3, 0xDDDD_3333);
    c.set_reg(4, 0x2000_0100); // base address
    c.regs.set_pc(0x1000);

    let (hw0, hw1) = encode_stmia_w(4, true, 0x000F); // {R0-R3}
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);

    // Verify memory contents
    assert_eq!(bus.read32(0x2000_0100), 0xAAAA_0000); // R0
    assert_eq!(bus.read32(0x2000_0104), 0xBBBB_1111); // R1
    assert_eq!(bus.read32(0x2000_0108), 0xCCCC_2222); // R2
    assert_eq!(bus.read32(0x2000_010C), 0xDDDD_3333); // R3
    // Writeback: R4 = 0x2000_0100 + 4*4 = 0x2000_0110
    assert_eq!(c.reg(4), 0x2000_0110);
    // Cost: 1 + 4 = 5
    assert_eq!(cy, 5);
}

#[test]
fn ldm_w_ia() {
    // LDMIA.W R4!, {R0-R3} — load 4 regs from R4, writeback
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0200;
    bus.write32(base, 0x1111_1111);
    bus.write32(base + 4, 0x2222_2222);
    bus.write32(base + 8, 0x3333_3333);
    bus.write32(base + 12, 0x4444_4444);
    c.set_reg(4, base);
    c.regs.set_pc(0x1000);

    let (hw0, hw1) = encode_ldmia_w(4, true, 0x000F); // {R0-R3}
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);

    assert_eq!(c.reg(0), 0x1111_1111);
    assert_eq!(c.reg(1), 0x2222_2222);
    assert_eq!(c.reg(2), 0x3333_3333);
    assert_eq!(c.reg(3), 0x4444_4444);
    // Writeback: R4 = base + 16
    assert_eq!(c.reg(4), base + 16);
    // Cost: 1 + 4 = 5
    assert_eq!(cy, 5);
}

#[test]
fn stm_w_db() {
    // STMDB.W R13!, {R4-R7, LR} — push pattern (5 regs)
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(4, 0x4444_4444);
    c.set_reg(5, 0x5555_5555);
    c.set_reg(6, 0x6666_6666);
    c.set_reg(7, 0x7777_7777);
    c.set_reg(14, 0xEEEE_EEEE); // LR
    let sp = 0x2000_1000;
    c.set_reg(13, sp);
    c.regs.set_pc(0x1000);

    // reglist = R4|R5|R6|R7|LR = bits 4,5,6,7,14 = 0x40F0
    let (hw0, hw1) = encode_stmdb_w(13, true, 0x40F0);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);

    // DB: start addr = SP - 5*4 = 0x2000_0FEC
    let start = sp - 20;
    assert_eq!(bus.read32(start),      0x4444_4444); // R4
    assert_eq!(bus.read32(start + 4),  0x5555_5555); // R5
    assert_eq!(bus.read32(start + 8),  0x6666_6666); // R6
    assert_eq!(bus.read32(start + 12), 0x7777_7777); // R7
    assert_eq!(bus.read32(start + 16), 0xEEEE_EEEE); // LR
    // Writeback: SP = SP - 5*4 = 0x2000_0FEC
    assert_eq!(c.reg(13), start);
    // Cost: 1 + 5 = 6
    assert_eq!(cy, 6);
}

#[test]
fn ldm_w_db_with_pc() {
    // LDMIA.W R13!, {R4-R7, PC} — pop with PC (5 regs including PC)
    let (mut c, mut bus) = core_and_bus();
    let sp = 0x2000_0FEC;
    bus.write32(sp,      0x4444_4444); // R4
    bus.write32(sp + 4,  0x5555_5555); // R5
    bus.write32(sp + 8,  0x6666_6666); // R6
    bus.write32(sp + 12, 0x7777_7777); // R7
    bus.write32(sp + 16, 0x0800_0101); // PC value (Thumb bit set)
    c.set_reg(13, sp);
    c.regs.set_pc(0x1000);

    // reglist = R4|R5|R6|R7|PC = bits 4,5,6,7,15 = 0x80F0
    let (hw0, hw1) = encode_ldmia_w(13, true, 0x80F0);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);

    assert_eq!(c.reg(4), 0x4444_4444);
    assert_eq!(c.reg(5), 0x5555_5555);
    assert_eq!(c.reg(6), 0x6666_6666);
    assert_eq!(c.reg(7), 0x7777_7777);
    // PC loaded: value & !1 = 0x0800_0100
    assert_eq!(c.regs.pc(), 0x0800_0100);
    // Writeback: SP = SP + 5*4 = 0x2000_1000
    assert_eq!(c.reg(13), sp + 20);
    // Cost: 1 + 5 + 3 (PC flush) = 9
    assert_eq!(cy, 9);
}

#[test]
fn push_w_pop_w_roundtrip() {
    // STMDB SP!, {R8-R11} then LDMIA SP!, {R8-R11} — high register roundtrip
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(8, 0xAAAA_BBBB);
    c.set_reg(9, 0xCCCC_DDDD);
    c.set_reg(10, 0xEEEE_FF00);
    c.set_reg(11, 0x1234_5678);
    let sp = 0x2000_2000;
    c.set_reg(13, sp);
    c.regs.set_pc(0x1000);

    // Push: STMDB SP!, {R8-R11} — reglist bits 8,9,10,11 = 0x0F00
    let (hw0, hw1) = encode_stmdb_w(13, true, 0x0F00);
    let cy_push = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cy_push, 5); // 1 + 4
    assert_eq!(c.reg(13), sp - 16);

    // Clobber registers
    c.set_reg(8, 0);
    c.set_reg(9, 0);
    c.set_reg(10, 0);
    c.set_reg(11, 0);
    c.regs.set_pc(0x1004);

    // Pop: LDMIA SP!, {R8-R11}
    let (hw0, hw1) = encode_ldmia_w(13, true, 0x0F00);
    let cy_pop = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(cy_pop, 5); // 1 + 4
    assert_eq!(c.reg(13), sp); // SP restored

    // Verify values roundtripped correctly
    assert_eq!(c.reg(8), 0xAAAA_BBBB);
    assert_eq!(c.reg(9), 0xCCCC_DDDD);
    assert_eq!(c.reg(10), 0xEEEE_FF00);
    assert_eq!(c.reg(11), 0x1234_5678);
}

// ============================================================================
// Tests: Thumb-32 data processing (register) — shifts, extends, misc
// ============================================================================

#[test]
fn lsl_w_reg() {
    // LSL.W R0, R1, R2: hw0=0xFA01, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0003);
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA01, 0xF002);
    assert_eq!(c.reg(0), 0x0000_0030);
    assert_eq!(cy, 1);
}

#[test]
fn lsr_w_reg() {
    // LSR.W R0, R1, R2: hw0=0xFA21, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_FF00);
    c.set_reg(2, 8);
    let cy = c.execute_one_wide(0xFA21, 0xF002);
    assert_eq!(c.reg(0), 0x0000_00FF);
    assert_eq!(cy, 1);
}

#[test]
fn asr_w_reg() {
    // ASR.W R0, R1, R2: hw0=0xFA41, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x8000_0000); // negative value
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA41, 0xF002);
    assert_eq!(c.reg(0), 0xF800_0000); // sign-extended
    assert_eq!(cy, 1);
}

#[test]
fn ror_w_reg() {
    // ROR.W R0, R1, R2: hw0=0xFA61, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_00FF);
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA61, 0xF002);
    assert_eq!(c.reg(0), 0xF000_000F); // low 4 bits rotated to top
    assert_eq!(cy, 1);
}

#[test]
fn lsls_w_reg_flags() {
    // LSLS.W R0, R1, R2 (S=1): hw0=0xFA11, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x8000_0001); // bit 31 set
    c.set_reg(2, 1);           // shift left by 1
    let cy = c.execute_one_wide(0xFA11, 0xF002);
    assert_eq!(c.reg(0), 0x0000_0002);
    assert!(!c.flag_n());    // result bit 31 = 0
    assert!(!c.flag_z());    // result != 0
    assert!(c.flag_c());     // bit 31 shifted out → carry = 1
    assert_eq!(cy, 1);
}

#[test]
fn sxth_w() {
    // SXTH R0, R1: hw0=0xFA0F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_FF80); // halfword 0xFF80 = -128 as i16
    let cy = c.execute_one_wide(0xFA0F, 0xF081);
    assert_eq!(c.reg(0), 0xFFFF_FF80); // sign-extended to 32 bits
    assert_eq!(cy, 1);
}

#[test]
fn sxtb_w() {
    // SXTB R0, R1: hw0=0xFA4F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0090); // byte 0x90 = -112 as i8
    let cy = c.execute_one_wide(0xFA4F, 0xF081);
    assert_eq!(c.reg(0), 0xFFFF_FF90); // sign-extended
    assert_eq!(cy, 1);
}

#[test]
fn uxth_w() {
    // UXTH R0, R1: hw0=0xFA1F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0xDEAD_BEEF);
    let cy = c.execute_one_wide(0xFA1F, 0xF081);
    assert_eq!(c.reg(0), 0x0000_BEEF); // zero-extended halfword
    assert_eq!(cy, 1);
}

#[test]
fn uxtb_w() {
    // UXTB R0, R1: hw0=0xFA5F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0xDEAD_BEEF);
    let cy = c.execute_one_wide(0xFA5F, 0xF081);
    assert_eq!(c.reg(0), 0x0000_00EF); // zero-extended byte
    assert_eq!(cy, 1);
}

#[test]
fn rev_w() {
    // REV.W R0, R1: hw0=0xFA91, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x12345678);
    let cy = c.execute_one_wide(0xFA91, 0xF081);
    assert_eq!(c.reg(0), 0x78563412);
    assert_eq!(cy, 1);
}

#[test]
fn rev16_w() {
    // REV16.W R0, R1: hw0=0xFA91, hw1=0xF091
    let mut c = CortexM33::new();
    c.set_reg(1, 0xAABB_CCDD);
    let cy = c.execute_one_wide(0xFA91, 0xF091);
    assert_eq!(c.reg(0), 0xBBAA_DDCC);
    assert_eq!(cy, 1);
}

#[test]
fn revsh_w() {
    // REVSH.W R0, R1: hw0=0xFA91, hw1=0xF0B1
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_01FF); // low halfword 0x01FF, byte-swapped = 0xFF01 = -255 as i16
    let cy = c.execute_one_wide(0xFA91, 0xF0B1);
    assert_eq!(c.reg(0), 0xFFFF_FF01); // sign-extended to 32 bits
    assert_eq!(cy, 1);
}

#[test]
fn rbit_w() {
    // RBIT R0, R1: hw0=0xFA91, hw1=0xF0A1
    let mut c = CortexM33::new();
    c.set_reg(1, 0x8000_0000); // only bit 31 set
    let cy = c.execute_one_wide(0xFA91, 0xF0A1);
    assert_eq!(c.reg(0), 0x0000_0001); // reversed → only bit 0 set
    assert_eq!(cy, 1);
}

#[test]
fn clz_w() {
    // CLZ R0, R1: hw0=0xFAB1, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0010_0000); // bit 20 set → 11 leading zeros
    let cy = c.execute_one_wide(0xFAB1, 0xF081);
    assert_eq!(c.reg(0), 11);
    assert_eq!(cy, 1);
}

#[test]
fn clz_zero() {
    // CLZ of 0 → 32
    let mut c = CortexM33::new();
    c.set_reg(1, 0);
    let cy = c.execute_one_wide(0xFAB1, 0xF081);
    assert_eq!(c.reg(0), 32);
    assert_eq!(cy, 1);
}

// ============================================================================
// Tests: Multiply, multiply-accumulate, and divide (Thumb-32)
// ============================================================================

// Encoding helpers for 32-bit result multiply: 1111_1011_0_op1_Rn | Ra_Rd_op2_Rm
fn encode_mul_w(rd: u8, rn: u8, rm: u8) -> (u16, u16) {
    // MUL: op1=000, op2=00, Ra=0xF
    let hw0 = 0xFB00u16 | rn as u16;
    let hw1 = 0xF000u16 | ((rd as u16) << 8) | rm as u16;
    (hw0, hw1)
}

fn encode_mla(rd: u8, rn: u8, rm: u8, ra: u8) -> (u16, u16) {
    // MLA: op1=000, op2=00, Ra!=0xF
    let hw0 = 0xFB00u16 | rn as u16;
    let hw1 = ((ra as u16) << 12) | ((rd as u16) << 8) | rm as u16;
    (hw0, hw1)
}

fn encode_mls(rd: u8, rn: u8, rm: u8, ra: u8) -> (u16, u16) {
    // MLS: op1=000, op2=01
    let hw0 = 0xFB00u16 | rn as u16;
    let hw1 = ((ra as u16) << 12) | ((rd as u16) << 8) | 0x0010 | rm as u16;
    (hw0, hw1)
}

// Encoding helpers for long multiply/divide: 1111_1011_1_op1_Rn | RdLo_RdHi_op2_Rm
fn encode_smull(rd_lo: u8, rd_hi: u8, rn: u8, rm: u8) -> (u16, u16) {
    // SMULL: op1=000, op2=0000
    let hw0 = 0xFB80u16 | rn as u16;
    let hw1 = ((rd_lo as u16) << 12) | ((rd_hi as u16) << 8) | rm as u16;
    (hw0, hw1)
}

fn encode_umull(rd_lo: u8, rd_hi: u8, rn: u8, rm: u8) -> (u16, u16) {
    // UMULL: op1=010, op2=0000
    let hw0 = 0xFBA0u16 | rn as u16;
    let hw1 = ((rd_lo as u16) << 12) | ((rd_hi as u16) << 8) | rm as u16;
    (hw0, hw1)
}

fn encode_smlal(rd_lo: u8, rd_hi: u8, rn: u8, rm: u8) -> (u16, u16) {
    // SMLAL: op1=100, op2=0000
    let hw0 = 0xFBC0u16 | rn as u16;
    let hw1 = ((rd_lo as u16) << 12) | ((rd_hi as u16) << 8) | rm as u16;
    (hw0, hw1)
}

fn encode_umlal(rd_lo: u8, rd_hi: u8, rn: u8, rm: u8) -> (u16, u16) {
    // UMLAL: op1=110, op2=0000
    let hw0 = 0xFBE0u16 | rn as u16;
    let hw1 = ((rd_lo as u16) << 12) | ((rd_hi as u16) << 8) | rm as u16;
    (hw0, hw1)
}

fn encode_sdiv(rd: u8, rn: u8, rm: u8) -> (u16, u16) {
    // SDIV: op1=001, op2=1111, RdHi=0xF
    let hw0 = 0xFB90u16 | rn as u16;
    let hw1 = ((rd as u16) << 12) | 0x0F00 | 0x00F0 | rm as u16;
    (hw0, hw1)
}

fn encode_udiv(rd: u8, rn: u8, rm: u8) -> (u16, u16) {
    // UDIV: op1=011, op2=1111, RdHi=0xF
    let hw0 = 0xFBB0u16 | rn as u16;
    let hw1 = ((rd as u16) << 12) | 0x0F00 | 0x00F0 | rm as u16;
    (hw0, hw1)
}

#[test]
fn mul_w() {
    // MUL R0, R1, R2: 7 * 6 = 42
    let mut c = CortexM33::new();
    c.set_reg(1, 7);
    c.set_reg(2, 6);
    let (hw0, hw1) = encode_mul_w(0, 1, 2);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 42);
    assert_eq!(cy, 1);
}

#[test]
fn mla_w() {
    // MLA R0, R1, R2, R3: 3 * 4 + 5 = 17
    let mut c = CortexM33::new();
    c.set_reg(1, 3);
    c.set_reg(2, 4);
    c.set_reg(3, 5);
    let (hw0, hw1) = encode_mla(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 17);
    assert_eq!(cy, 1);
}

#[test]
fn mls_w() {
    // MLS R0, R1, R2, R3: 100 - 7 * 6 = 58
    let mut c = CortexM33::new();
    c.set_reg(1, 7);
    c.set_reg(2, 6);
    c.set_reg(3, 100);
    let (hw0, hw1) = encode_mls(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 58);
    assert_eq!(cy, 1);
}

#[test]
fn smull_basic() {
    // SMULL R0, R1, R2, R3: 100_000 * 200 = 20_000_000
    let mut c = CortexM33::new();
    c.set_reg(2, 100_000);
    c.set_reg(3, 200);
    let (hw0, hw1) = encode_smull(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 20_000_000); // lo
    assert_eq!(c.reg(1), 0);          // hi = 0
    assert_eq!(cy, 1);
}

#[test]
fn smull_negative() {
    // SMULL R0, R1, R2, R3: (-3) * 7 = -21
    let mut c = CortexM33::new();
    c.set_reg(2, (-3i32) as u32);
    c.set_reg(3, 7);
    let (hw0, hw1) = encode_smull(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    let result = ((c.reg(1) as u64) << 32) | c.reg(0) as u64;
    assert_eq!(result as i64, -21);
    assert_eq!(cy, 1);
}

#[test]
fn umull_basic() {
    // UMULL R0, R1, R2, R3: 1000 * 2000 = 2_000_000
    let mut c = CortexM33::new();
    c.set_reg(2, 1000);
    c.set_reg(3, 2000);
    let (hw0, hw1) = encode_umull(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 2_000_000);
    assert_eq!(c.reg(1), 0);
    assert_eq!(cy, 1);
}

#[test]
fn umull_large() {
    // UMULL R0, R1, R2, R3: 0xFFFF_FFFF * 2 = 0x1_FFFF_FFFE
    let mut c = CortexM33::new();
    c.set_reg(2, 0xFFFF_FFFF);
    c.set_reg(3, 2);
    let (hw0, hw1) = encode_umull(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xFFFF_FFFE); // lo
    assert_eq!(c.reg(1), 1);            // hi
    assert_eq!(cy, 1);
}

#[test]
fn sdiv_basic() {
    // SDIV R0, R1, R2: 100 / 7 = 14
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.set_reg(2, 7);
    let (hw0, hw1) = encode_sdiv(0, 1, 2);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 14);
    assert_eq!(cy, 4);
}

#[test]
fn sdiv_negative() {
    // SDIV R0, R1, R2: -100 / 7 = -14
    let mut c = CortexM33::new();
    c.set_reg(1, (-100i32) as u32);
    c.set_reg(2, 7);
    let (hw0, hw1) = encode_sdiv(0, 1, 2);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0) as i32, -14);
    assert_eq!(cy, 4);
}

#[test]
fn sdiv_by_zero() {
    // SDIV R0, R1, R2: 42 / 0 = 0
    let mut c = CortexM33::new();
    c.set_reg(0, 0xDEAD_BEEF); // should be overwritten
    c.set_reg(1, 42);
    c.set_reg(2, 0);
    let (hw0, hw1) = encode_sdiv(0, 1, 2);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0);
    assert_eq!(cy, 4);
}

#[test]
fn udiv_basic() {
    // UDIV R0, R1, R2: 100 / 7 = 14
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.set_reg(2, 7);
    let (hw0, hw1) = encode_udiv(0, 1, 2);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 14);
    assert_eq!(cy, 4);
}

#[test]
fn udiv_by_zero() {
    // UDIV R0, R1, R2: 42 / 0 = 0
    let mut c = CortexM33::new();
    c.set_reg(0, 0xDEAD_BEEF);
    c.set_reg(1, 42);
    c.set_reg(2, 0);
    let (hw0, hw1) = encode_udiv(0, 1, 2);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0);
    assert_eq!(cy, 4);
}

#[test]
fn smlal_basic() {
    // SMLAL R0, R1, R2, R3: accumulator=1000, product=3*7=21, result=1021
    let mut c = CortexM33::new();
    c.set_reg(0, 1000); // rd_lo (accumulator low)
    c.set_reg(1, 0);    // rd_hi (accumulator high)
    c.set_reg(2, 3);    // rn
    c.set_reg(3, 7);    // rm
    let (hw0, hw1) = encode_smlal(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 1021); // lo
    assert_eq!(c.reg(1), 0);    // hi
    assert_eq!(cy, 1);
}

#[test]
fn umlal_basic() {
    // UMLAL R0, R1, R2, R3: accumulator=500, product=100*200=20000, result=20500
    let mut c = CortexM33::new();
    c.set_reg(0, 500);  // rd_lo
    c.set_reg(1, 0);    // rd_hi
    c.set_reg(2, 100);  // rn
    c.set_reg(3, 200);  // rm
    let (hw0, hw1) = encode_umlal(0, 1, 2, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 20500);
    assert_eq!(c.reg(1), 0);
    assert_eq!(cy, 1);
}

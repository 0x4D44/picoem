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
fn thumb32_ldr_w_routes_to_stub() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // LDR.W R0, [R1, #0]: hw0=0xF8D1, hw1=0x0000
    // op1 = (0xF8D1 >> 11) & 0x3 = 0b11
    // op2 = (0xF8D1 >> 4) & 0x7F = 0x0D = 0b0001101
    // op2 & 0x40 = 0, op2 & 0x20 = 0 → load_store_single (stub → 1)
    let cy = c.execute_one_wide(0xF8D1, 0x0000);
    assert_eq!(cy, 1);
}

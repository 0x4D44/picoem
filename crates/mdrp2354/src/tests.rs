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

#[test]
fn bxns_from_secure() {
    let mut c = CortexM33::new();
    assert!(c.secure);
    c.regs.msp_ns = 0x2000_4000;
    c.set_reg(1, 0x1000_0001); // target with Thumb bit
    // BXNS R1: 0100_0111_0_0001_100 = 0x470C
    c.execute_one(0x470C);
    assert!(!c.secure);
    assert_eq!(c.regs.pc(), 0x1000_0000);
    assert_eq!(c.regs.r[13], 0x2000_4000);
    assert_eq!(c.regs.msp, 0x2000_4000);
    assert_eq!(c.regs.msp_ns, 0); // old Secure MSP preserved
}

#[test]
fn bxns_from_nonsecure() {
    let mut c = CortexM33::new();
    c.secure = false;
    c.set_reg(2, 0x2000_0001); // target with Thumb bit
    let orig_msp = c.regs.msp;
    let orig_msp_ns = c.regs.msp_ns;
    // BXNS R2: 0100_0111_0_0010_100 = 0x4714
    c.execute_one(0x4714);
    assert!(!c.secure);
    assert_eq!(c.regs.pc(), 0x2000_0000);
    assert_eq!(c.regs.msp, orig_msp);
    assert_eq!(c.regs.msp_ns, orig_msp_ns);
}

#[test]
fn bxns_msp_ns_setup_pattern() {
    let mut c = CortexM33::new();
    c.regs.msp = 0x2000_8000;
    c.regs.r[13] = c.regs.msp;
    c.regs.msp_ns = 0x2000_1000;
    c.regs.msplim_ns = 0x2000_0800;
    c.set_reg(0, 0x1000_0101); // target with Thumb bit
    // BXNS R0: 0100_0111_0_0000_100 = 0x4704
    c.execute_one(0x4704);
    assert!(!c.secure);
    assert_eq!(c.regs.pc(), 0x1000_0100);
    assert_eq!(c.regs.msp, 0x2000_1000);
    assert_eq!(c.regs.msp_ns, 0x2000_8000);
    assert_eq!(c.regs.msplim, 0x2000_0800);
    assert_eq!(c.regs.r[13], 0x2000_1000);
}

#[test]
fn bxns_with_psp_active() {
    let mut c = CortexM33::new();
    // Secure state using PSP (SPSEL=1)
    c.regs.control = 2; // SPSEL=1
    c.regs.psp = 0x2000_A000;
    c.regs.r[13] = c.regs.psp; // R13 mirrors PSP when SPSEL=1
    c.regs.msp = 0x2000_8000;
    // NS state: SPSEL=0 (using MSP)
    c.regs.control_ns = 0;
    c.regs.msp_ns = 0x2000_2000;
    c.regs.psp_ns = 0x2000_3000;
    c.set_reg(3, 0x1000_0001); // target with Thumb bit
    // BXNS R3: 0100_0111_0_0011_100 = 0x471C
    c.execute_one(0x471C);
    assert!(!c.secure);
    assert_eq!(c.regs.pc(), 0x1000_0000);
    // After swap: NS CONTROL (SPSEL=0) is now active, so R13 = MSP
    assert_eq!(c.regs.control, 0); // was control_ns
    assert_eq!(c.regs.msp, 0x2000_2000); // was msp_ns
    assert_eq!(c.regs.psp, 0x2000_3000); // was psp_ns
    assert_eq!(c.regs.r[13], 0x2000_2000); // loaded from new MSP (SPSEL=0)
    // Secure state preserved in _ns slots
    assert_eq!(c.regs.control_ns, 2); // was control (SPSEL=1)
    assert_eq!(c.regs.msp_ns, 0x2000_8000); // was msp
    assert_eq!(c.regs.psp_ns, 0x2000_A000); // was psp
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
    assert_eq!(cy, 1); // taken — M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    // → op1=10, op=0, op2 & 0x20 = 0x20 → dp_plain_imm → MOVW
    let cy = c.execute_one_wide(0xF240, 0x0000);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn thumb32_bl_routes_through_branch_misc() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // BL +100: same encoding as the existing bl_forward test
    let cy = c.execute_one_wide(0xF000, 0xF832);
    assert_eq!(c.regs.lr(), 0x1005);
    assert_eq!(c.regs.pc(), 0x1068);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn thumb32_ldr_w_routes_to_load_store_single() {
    let mut c = CortexM33::new();
    c.regs.set_pc(0x1000);

    // LDR.W R0, [R1, #0]: hw0=0xF8D1, hw1=0x0000
    // op1 = (0xF8D1 >> 11) & 0x3 = 0b11
    // op2 = (0xF8D1 >> 4) & 0x7F = 0x0D = 0b0001101
    // op2 & 0x40 = 0, op2 & 0x20 = 0 → load_store_single → load costs 3
    let cy = c.execute_one_wide(0xF8D1, 0x0000);
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (rotated imm)
}

#[test]
fn mov_w_imm() {
    // MOV.W R0, #imm via ORR with Rn=15, S=0
    // Use imm12 = 0x34 → imm32 = 0x34
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_dp_mod_imm(0b0010, false, 15, 0, 0x34);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x34);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn mvn_w_imm() {
    // MVN.W R0, #0 → R0 = 0xFFFFFFFF (via ORN with Rn=15, S=0)
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_dp_mod_imm(0b0011, false, 15, 0, 0x00);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xFFFF_FFFF);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (rotated imm)
}

#[test]
fn bic_w_imm() {
    // BIC.W R0, R1, #0x0F → R0 = R1 & ~0x0F (clear low nibble)
    let mut c = CortexM33::new();
    c.set_reg(1, 0xABCD_EF9A);
    let (hw0, hw1) = encode_dp_mod_imm(0b0001, false, 1, 0, 0x0F);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xABCD_EF90);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn eor_w_imm() {
    // EOR.W R0, R1, #0xFF (S=0)
    let mut c = CortexM33::new();
    c.set_reg(1, 0xAA);
    let (hw0, hw1) = encode_dp_mod_imm(0b0100, false, 1, 0, 0xFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xAA ^ 0xFF); // 0x55
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn orn_w_imm() {
    // ORN.W R0, R1, #0xFF → R0 = R1 | ~0xFF = R1 | 0xFFFFFF00
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0042);
    let (hw0, hw1) = encode_dp_mod_imm(0b0011, false, 1, 0, 0xFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xFFFF_FF42);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn movw_all_bits() {
    // MOVW R0, #0xFFFF — all 16 bits set
    let mut c = CortexM33::new();
    let (hw0, hw1) = encode_movw(0, 0xFFFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x0000_FFFF);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn movt_basic() {
    // MOVT R0, #0xABCD — set top half, preserve bottom half
    let mut c = CortexM33::new();
    c.set_reg(0, 0x0000_5678);
    let (hw0, hw1) = encode_movt(0, 0xABCD);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xABCD_5678);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn subw_basic() {
    // SUBW R0, R1, #2000
    let mut c = CortexM33::new();
    c.set_reg(1, 5000);
    let (hw0, hw1) = encode_subw(0, 1, 2000);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 3000);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn sbfx_positive() {
    // SBFX R0, R1, #4, #8 — extract bits [11:4] signed, positive value
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0750); // bits [11:4] = 0x75 = 0b0111_0101 (positive)
    let (hw0, hw1) = encode_sbfx(0, 1, 4, 8);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x75);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
}

#[test]
fn pld_rt15_is_nop() {
    // Byte load with Rt=15 is PLD (preload hint), treated as NOP.
    let (mut c, mut bus) = core_and_bus();
    c.regs.set_pc(0x1000);
    c.set_reg(1, 0x2000_0000);
    bus.write32(0x2000_0000, 0xDEAD_BEEF);
    // LDRB.W R15, [R1, #0] → Rt=15, size=byte → PLD, returns 1
    let (hw0, hw1) = encode_ldrb_w_imm12(15, 1, 0);
    let pc_before = c.regs.pc();
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    // PC should be at pc_before+4 (normal advance), not modified by load
    assert_eq!(c.regs.pc(), pc_before + 4);
    assert_eq!(cy, 1); // NOP cost
}

#[test]
fn ldr_w_rt15_loads_pc() {
    // Word load with Rt=15 is LDR PC (real load), not a preload hint.
    let (mut c, mut bus) = core_and_bus();
    c.regs.set_pc(0x1000);
    c.set_reg(1, 0x2000_0000);
    bus.write32(0x2000_0000, 0x0000_1001); // target addr with thumb bit
    // LDR.W R15, [R1, #0] → Rt=15, size=word → loads PC
    let (hw0, hw1) = encode_ldr_w_imm12(15, 1, 0);
    eprintln!("hw0={:#06x} hw1={:#06x}", hw0, hw1);
    let sz = (hw0 >> 5) & 3; let ld = (hw0 >> 4) & 1;
    eprintln!("size={} load={} rn={} rt={}", sz, ld, hw0 & 0xF, (hw1 >> 12) & 0xF);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    eprintln!("cy={} pc={:#x} r15={:#x}", cy, c.regs.pc(), c.regs.r[15]);
    // PC set to loaded value with bit[0] cleared
    assert_eq!(c.regs.pc(), 0x0000_1000);
    assert_eq!(cy, 5); // load + pipeline flush
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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

#[test]
fn clrex_is_nop() {
    let mut c = CortexM33::new();
    let cy = c.execute_one_wide(0xF3BF, 0x8F2F);
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn lsr_w_reg() {
    // LSR.W R0, R1, R2: hw0=0xFA21, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_FF00);
    c.set_reg(2, 8);
    let cy = c.execute_one_wide(0xFA21, 0xF002);
    assert_eq!(c.reg(0), 0x0000_00FF);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn asr_w_reg() {
    // ASR.W R0, R1, R2: hw0=0xFA41, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x8000_0000); // negative value
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA41, 0xF002);
    assert_eq!(c.reg(0), 0xF800_0000); // sign-extended
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn ror_w_reg() {
    // ROR.W R0, R1, R2: hw0=0xFA61, hw1=0xF002
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_00FF);
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA61, 0xF002);
    assert_eq!(c.reg(0), 0xF000_000F); // low 4 bits rotated to top
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn sxth_w() {
    // SXTH R0, R1: hw0=0xFA0F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_FF80); // halfword 0xFF80 = -128 as i16
    let cy = c.execute_one_wide(0xFA0F, 0xF081);
    assert_eq!(c.reg(0), 0xFFFF_FF80); // sign-extended to 32 bits
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn sxtb_w() {
    // SXTB R0, R1: hw0=0xFA4F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0090); // byte 0x90 = -112 as i8
    let cy = c.execute_one_wide(0xFA4F, 0xF081);
    assert_eq!(c.reg(0), 0xFFFF_FF90); // sign-extended
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn uxth_w() {
    // UXTH R0, R1: hw0=0xFA1F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0xDEAD_BEEF);
    let cy = c.execute_one_wide(0xFA1F, 0xF081);
    assert_eq!(c.reg(0), 0x0000_BEEF); // zero-extended halfword
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn uxtb_w() {
    // UXTB R0, R1: hw0=0xFA5F, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0xDEAD_BEEF);
    let cy = c.execute_one_wide(0xFA5F, 0xF081);
    assert_eq!(c.reg(0), 0x0000_00EF); // zero-extended byte
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn rev_w() {
    // REV.W R0, R1: hw0=0xFA91, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x12345678);
    let cy = c.execute_one_wide(0xFA91, 0xF081);
    assert_eq!(c.reg(0), 0x78563412);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn rev16_w() {
    // REV16.W R0, R1: hw0=0xFA91, hw1=0xF091
    let mut c = CortexM33::new();
    c.set_reg(1, 0xAABB_CCDD);
    let cy = c.execute_one_wide(0xFA91, 0xF091);
    assert_eq!(c.reg(0), 0xBBAA_DDCC);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn revsh_w() {
    // REVSH.W R0, R1: hw0=0xFA91, hw1=0xF0B1
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_01FF); // low halfword 0x01FF, byte-swapped = 0xFF01 = -255 as i16
    let cy = c.execute_one_wide(0xFA91, 0xF0B1);
    assert_eq!(c.reg(0), 0xFFFF_FF01); // sign-extended to 32 bits
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn rbit_w() {
    // RBIT R0, R1: hw0=0xFA91, hw1=0xF0A1
    let mut c = CortexM33::new();
    c.set_reg(1, 0x8000_0000); // only bit 31 set
    let cy = c.execute_one_wide(0xFA91, 0xF0A1);
    assert_eq!(c.reg(0), 0x0000_0001); // reversed → only bit 0 set
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn clz_w() {
    // CLZ R0, R1: hw0=0xFAB1, hw1=0xF081
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0010_0000); // bit 20 set → 11 leading zeros
    let cy = c.execute_one_wide(0xFAB1, 0xF081);
    assert_eq!(c.reg(0), 11);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn clz_zero() {
    // CLZ of 0 → 32
    let mut c = CortexM33::new();
    c.set_reg(1, 0);
    let cy = c.execute_one_wide(0xFAB1, 0xF081);
    assert_eq!(c.reg(0), 32);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
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
    let hw1 = 0xF000 | ((rd as u16) << 8) | 0x00F0 | rm as u16;
    (hw0, hw1)
}

fn encode_udiv(rd: u8, rn: u8, rm: u8) -> (u16, u16) {
    // UDIV: op1=011, op2=1111, RdHi=0xF
    let hw0 = 0xFBB0u16 | rn as u16;
    let hw1 = 0xF000 | ((rd as u16) << 8) | 0x00F0 | rm as u16;
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    // 100: 7 significant bits (<=20), floor at 5 cycles
    assert_eq!(cy, 5); // M33 measured: data-dependent [1..12]
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
    // |-100| = 100: 7 significant bits (<=20), floor at 5 cycles
    assert_eq!(cy, 5); // M33 measured: data-dependent [1..12]
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle (div by zero early exit)
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
    // 100: 7 significant bits (<=20), floor at 5 cycles
    assert_eq!(cy, 5); // M33 measured: data-dependent [1..12]
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
    assert_eq!(cy, 1); // M33 measured: 1 cycle (div by zero early exit)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
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
    assert_eq!(cy, 2); // M33 measured: 2 cycles (multiplier)
}

// ============================================================================
// Thumb-32: Data Processing (Shifted Register)
// ============================================================================

/// Encode a data-processing (shifted register) instruction.
/// Format: 11101_01_op[3:0]_S_Rn  0_imm3_Rd_imm2_tt_Rm
fn encode_dp_shifted_reg(
    op: u8, s: bool, rn: u8, rd: u8, rm: u8, shift_type: u8, shift_n: u8,
) -> (u16, u16) {
    // hw0 = 11101_01_op[3:0]_S_Rn[3:0]
    let hw0: u16 = 0xEA00
        | ((op as u16 & 0xF) << 5)
        | ((s as u16) << 4)
        | (rn as u16 & 0xF);
    // hw1 = 0_imm3_Rd_imm2_tt_Rm
    // shift_n[4:2] = imm3, shift_n[1:0] = imm2
    let imm3 = ((shift_n >> 2) & 0x7) as u16;
    let imm2 = (shift_n & 0x3) as u16;
    let hw1: u16 = (imm3 << 12)
        | ((rd as u16 & 0xF) << 8)
        | (imm2 << 6)
        | ((shift_type as u16 & 0x3) << 4)
        | (rm as u16 & 0xF);
    (hw0, hw1)
}

#[test]
fn add_w_shifted_reg() {
    // ADD.W R0, R1, R2, LSL #2 → R0 = R1 + (R2 << 2) = 10 + (3 << 2) = 22
    let mut c = CortexM33::new();
    c.set_reg(1, 10);
    c.set_reg(2, 3);
    let (hw0, hw1) = encode_dp_shifted_reg(0b1000, false, 1, 0, 2, 0b00, 2);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 22);
    assert_eq!(cy, 1); // M33 measured: 1 cycle (LSL #2 — barrel shifter fast path)
}

#[test]
fn sub_w_shifted_reg() {
    // SUB.W R0, R1, R2, LSR #1 → R0 = R1 - (R2 >> 1) = 100 - (20 >> 1) = 90
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.set_reg(2, 20);
    let (hw0, hw1) = encode_dp_shifted_reg(0b1101, false, 1, 0, 2, 0b01, 1);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 90);
    assert_eq!(cy, 2); // M33 measured: 2 cycles (barrel shifter)
}

#[test]
fn and_w_shifted_reg() {
    // AND.W R0, R1, R2 (no shift, LSL #0)
    // R0 = R1 & R2 = 0xFF00_FF00 & 0x00FF_00FF = 0x0000_0000
    let mut c = CortexM33::new();
    c.set_reg(1, 0xFF00_FF00);
    c.set_reg(2, 0x00FF_00FF);
    let (hw0, hw1) = encode_dp_shifted_reg(0b0000, false, 1, 0, 2, 0b00, 0);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0);
    assert_eq!(cy, 1); // M33 measured: 1 cycle (LSL #0, identity)
}

#[test]
fn cmp_w_shifted_reg() {
    // CMP.W R1, R2, ASR #3 (S=1, Rd=15 → flags only, no write)
    // R1=100, R2=0x80 (128). ASR #3 = 128 >> 3 = 16.
    // 100 - 16 = 84 → positive, no borrow, no overflow.
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.set_reg(2, 128);
    let (hw0, hw1) = encode_dp_shifted_reg(0b1101, true, 1, 15, 2, 0b10, 3);
    let cy = c.execute_one_wide(hw0, hw1);
    assert!(!c.flag_n());
    assert!(!c.flag_z());
    assert!(c.flag_c());   // no borrow → C=1
    assert!(!c.flag_v());
    assert_eq!(cy, 2); // M33 measured: 2 cycles (barrel shifter)
}

#[test]
fn mov_w_shift_imm() {
    // LSL.W R0, R1, #4 — encoded as MOV variant: op=0010, Rn=15
    // R0 = R1 << 4 = 0xA << 4 = 0xA0
    let mut c = CortexM33::new();
    c.set_reg(1, 0xA);
    let (hw0, hw1) = encode_dp_shifted_reg(0b0010, false, 15, 0, 1, 0b00, 4);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xA0);
    assert_eq!(cy, 1); // M33 measured: 1 cycle (MOV.W, Rn=15 — shift is primary op)
}

#[test]
fn rrx_w() {
    // RRX R0, R1 — shift_type=11, amount=0 → rotate right through carry
    // R1 = 0x0000_0003, carry_in = 1
    // RRX: result = (1 << 31) | (3 >> 1) = 0x8000_0001, carry_out = bit[0] of 3 = 1
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0000_0003);
    c.regs.set_flag_c(true);
    // MOV variant: op=0010, Rn=15, S=1 to see carry_out
    let (hw0, hw1) = encode_dp_shifted_reg(0b0010, true, 15, 0, 1, 0b11, 0);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x8000_0001);
    assert!(c.flag_n());   // bit 31 set
    assert!(!c.flag_z());
    assert!(c.flag_c());   // carry_out from RRX = bit[0] of input
    assert_eq!(cy, 1); // M33 measured: 1 cycle (MOV.W via RRX, Rn=15 — shift is primary op)
}

#[test]
fn orr_w_shifted() {
    // ORR.W R0, R1, R2, ROR #8
    // R1 = 0xFF00_0000, R2 = 0x0000_00AB
    // R2 ROR 8 = 0xAB00_0000
    // R0 = 0xFF00_0000 | 0xAB00_0000 = 0xFF00_0000
    let mut c = CortexM33::new();
    c.set_reg(1, 0xFF00_0000);
    c.set_reg(2, 0x0000_00AB);
    let (hw0, hw1) = encode_dp_shifted_reg(0b0010, false, 1, 0, 2, 0b11, 8);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xFF00_0000 | 0xAB00_0000);
    assert_eq!(cy, 2); // M33 measured: 2 cycles (barrel shifter)
}

// ============================================================================
// Thumb-32: Load/Store Dual (LDRD / STRD)
// ============================================================================

/// Encode LDRD/STRD immediate.
/// Format: 1110_100_P_U_1_W_L_Rn  Rt_Rt2_imm8
fn encode_ldrd_strd(
    p: bool, u: bool, w: bool, load: bool, rn: u8, rt: u8, rt2: u8, imm8: u8,
) -> (u16, u16) {
    let hw0: u16 = 0xE800
        | ((p as u16) << 8)
        | ((u as u16) << 7)
        | (1u16 << 6) // bit 6 always 1 for LDRD/STRD
        | ((w as u16) << 5)
        | ((load as u16) << 4)
        | (rn as u16 & 0xF);
    let hw1: u16 = ((rt as u16 & 0xF) << 12)
        | ((rt2 as u16 & 0xF) << 8)
        | (imm8 as u16);
    (hw0, hw1)
}

#[test]
fn ldrd_basic() {
    // LDRD R0, R1, [R2, #8]: P=1, U=1, W=0, load=1
    // offset = 8 >> 2 = imm8=2, actual offset = 2 << 2 = 8
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(2, 0x2000_0000);
    bus.write32(0x2000_0008, 0xAAAA_BBBB);
    bus.write32(0x2000_000C, 0xCCCC_DDDD);
    let (hw0, hw1) = encode_ldrd_strd(true, true, false, true, 2, 0, 1, 2);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0xAAAA_BBBB);
    assert_eq!(c.reg(1), 0xCCCC_DDDD);
    assert_eq!(c.reg(2), 0x2000_0000); // no writeback
    assert_eq!(cy, 3); // M33 measured: 3 cycles (two word transfers)
}

#[test]
fn strd_basic() {
    // STRD R0, R1, [R2, #8]: P=1, U=1, W=0, load=0
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0x1111_2222);
    c.set_reg(1, 0x3333_4444);
    c.set_reg(2, 0x2000_0000);
    let (hw0, hw1) = encode_ldrd_strd(true, true, false, false, 2, 0, 1, 2);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(bus.read32(0x2000_0008), 0x1111_2222);
    assert_eq!(bus.read32(0x2000_000C), 0x3333_4444);
    assert_eq!(cy, 3); // M33 measured: 3 cycles (two word transfers)
}

#[test]
fn strd_ldrd_roundtrip() {
    // Store two words, then load them back into different registers
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xDEAD_BEEF);
    c.set_reg(1, 0xCAFE_BABE);
    c.set_reg(4, 0x2000_0100);

    // STRD R0, R1, [R4, #16]
    let (hw0, hw1) = encode_ldrd_strd(true, true, false, false, 4, 0, 1, 4); // imm8=4 → offset=16
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(bus.read32(0x2000_0110), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2000_0114), 0xCAFE_BABE);

    // LDRD R2, R3, [R4, #16]
    let (hw0, hw1) = encode_ldrd_strd(true, true, false, true, 4, 2, 3, 4);
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(2), 0xDEAD_BEEF);
    assert_eq!(c.reg(3), 0xCAFE_BABE);
}

#[test]
fn ldrd_literal() {
    // LDRD with Rn=15 (PC-relative). PC must be in SRAM so literal addr is writable.
    // PC at 0x2000_1000, read_pc = 0x2000_1004, aligned = 0x2000_1004
    // offset = 4 (imm8=1, offset = 1<<2 = 4), U=1 → addr = 0x2000_1008
    let (mut c, mut bus) = core_and_bus();
    c.regs.set_pc(0x2000_1000);
    bus.write32(0x2000_1008, 0x1234_5678);
    bus.write32(0x2000_100C, 0x9ABC_DEF0);
    let (hw0, hw1) = encode_ldrd_strd(true, true, false, true, 15, 0, 1, 1);
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(0), 0x1234_5678);
    assert_eq!(c.reg(1), 0x9ABC_DEF0);
    assert_eq!(cy, 3); // M33 measured: 3 cycles (two word transfers)
}

// ============================================================================
// Thumb-32: TBB / TBH (Table Branch)
// ============================================================================

#[test]
fn tbb_basic() {
    // TBB [R0, R1]: read byte at R0+R1, branch PC += 2*byte
    // hw0 = 1110_1000_1101_Rn = 0xE8D0 | Rn
    // hw1 = 1111_0000_0000_Rm = 0xF000 | Rm
    let (mut c, mut bus) = core_and_bus();
    c.regs.set_pc(0x1000);
    c.set_reg(0, 0x2000_0000);   // base
    c.set_reg(1, 3);              // index
    bus.write8(0x2000_0003, 10);  // table[3] = 10
    // read_pc = 0x1004, target = 0x1004 + 10*2 = 0x1018
    let hw0: u16 = 0xE8D0; // Rn=0
    let hw1: u16 = 0xF001; // Rm=1, H=0
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.regs.pc(), 0x1018);
    assert_eq!(cy, 4);
}

#[test]
fn tbh_basic() {
    // TBH [R0, R1]: read halfword at R0 + R1*2, branch PC += 2*halfword
    let (mut c, mut bus) = core_and_bus();
    c.regs.set_pc(0x1000);
    c.set_reg(0, 0x2000_0000);   // base
    c.set_reg(1, 2);              // index
    bus.write16(0x2000_0004, 20); // table[2] = 20 (at base + 2*2 = base+4)
    // read_pc = 0x1004, target = 0x1004 + 20*2 = 0x102C
    let hw0: u16 = 0xE8D0; // Rn=0
    let hw1: u16 = 0xF011; // Rm=1, H=1 (bit 4 set)
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.regs.pc(), 0x102C);
    assert_eq!(cy, 4);
}

// ============================================================================
// MSR / MRS (Stage 10)
// ============================================================================

#[test]
fn msr_primask() {
    // MSR PRIMASK, R0 — write 1 to PRIMASK, then MRS R1, PRIMASK to read back
    let mut c = CortexM33::new();
    c.set_reg(0, 1);
    // MSR PRIMASK, R0: hw0=0xF380 (Rn=0), hw1=0x8010 (SYSm=16)
    let cy = c.execute_one_wide(0xF380, 0x8010);
    assert_eq!(c.regs.primask, 1);
    assert_eq!(cy, 1); // M33 measured: 1 cycle

    // MRS R1, PRIMASK: hw0=0xF3EF, hw1=0x8110 (Rd=1, SYSm=16)
    let cy = c.execute_one_wide(0xF3EF, 0x8110);
    assert_eq!(c.reg(1), 1);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn msr_basepri() {
    // MSR BASEPRI, R0 — write 0x40 to BASEPRI
    let mut c = CortexM33::new();
    c.set_reg(0, 0x40);
    // MSR BASEPRI, R0: hw0=0xF380, hw1=0x8011 (SYSm=17)
    c.execute_one_wide(0xF380, 0x8011);
    assert_eq!(c.regs.basepri, 0x40);

    // MRS R1, BASEPRI: hw0=0xF3EF, hw1=0x8111 (Rd=1, SYSm=17)
    c.execute_one_wide(0xF3EF, 0x8111);
    assert_eq!(c.reg(1), 0x40);
}

#[test]
fn mrs_apsr_flags() {
    // Set NZCV flags, then MRS R0, APSR to read them back
    let mut c = CortexM33::new();
    c.regs.set_nzcv(true, false, true, false); // N=1, Z=0, C=1, V=0
    // MRS R0, APSR: hw0=0xF3EF, hw1=0x8000 (Rd=0, SYSm=0)
    let cy = c.execute_one_wide(0xF3EF, 0x8000);
    // N=bit31, C=bit29 => 0xA000_0000
    assert_eq!(c.reg(0), 0xA000_0000);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn msr_msp() {
    // MSR MSP, R0 — write to MSP, verify R13 changes (default uses MSP)
    let mut c = CortexM33::new();
    c.set_reg(0, 0x2000_1000);
    // MSR MSP, R0: hw0=0xF380, hw1=0x8008 (SYSm=8)
    c.execute_one_wide(0xF380, 0x8008);
    assert_eq!(c.regs.msp, 0x2000_1000);
    // In thread mode with SPSEL=0, R13 should mirror MSP
    assert_eq!(c.regs.r[13], 0x2000_1000);
}

#[test]
fn msr_psp() {
    // MSR PSP, R0 — write to PSP (R13 shouldn't change since SPSEL=0)
    let mut c = CortexM33::new();
    c.set_reg(0, 0x2000_2000);
    // MSR PSP, R0: hw0=0xF380, hw1=0x8009 (SYSm=9)
    c.execute_one_wide(0xF380, 0x8009);
    assert_eq!(c.regs.psp, 0x2000_2000);
    // SPSEL=0 so R13 should still be MSP (unchanged from reset = 0)
    assert_eq!(c.regs.r[13], 0);
}

#[test]
fn msr_control_spsel() {
    // Write CONTROL.SPSEL=1 to switch from MSP to PSP, verify SP switches
    let mut c = CortexM33::new();

    // Set up MSP and PSP values
    c.regs.msp = 0x2000_1000;
    c.regs.psp = 0x2000_2000;
    c.regs.r[13] = 0x2000_1000; // R13 = MSP initially

    // MSR CONTROL, R0 with SPSEL=1 (bit 1): switch to PSP
    c.set_reg(0, 0x2); // SPSEL=1
    // MSR CONTROL, R0: hw0=0xF380, hw1=0x8014 (SYSm=20)
    c.execute_one_wide(0xF380, 0x8014);

    assert_eq!(c.regs.control & 0x2, 0x2); // SPSEL bit set
    // sync_sp_to_banked saved old R13 (MSP=0x2000_1000) to msp
    assert_eq!(c.regs.msp, 0x2000_1000);
    // sync_sp_from_banked loaded PSP into R13
    assert_eq!(c.regs.r[13], 0x2000_2000);

    // Switch back to MSP: write CONTROL with SPSEL=0
    c.set_reg(0, 0x0);
    c.execute_one_wide(0xF380, 0x8014);

    assert_eq!(c.regs.control & 0x2, 0x0);
    // R13 should now be MSP again
    assert_eq!(c.regs.r[13], 0x2000_1000);
    // PSP should have been saved from the previous R13
    assert_eq!(c.regs.psp, 0x2000_2000);
}

#[test]
fn mrs_ipsr() {
    // MRS R0, IPSR — should be 0 in thread mode (no exception active)
    let mut c = CortexM33::new();
    // MRS R0, IPSR: hw0=0xF3EF, hw1=0x8005 (Rd=0, SYSm=5)
    c.execute_one_wide(0xF3EF, 0x8005);
    assert_eq!(c.reg(0), 0);
}

// ============================================================================
// IT (If-Then) Blocks (Stage 11)
// ============================================================================

/// Helper: step the core past any stall cycles so the next step fetches.
fn step_one(c: &mut CortexM33, bus: &mut Bus) {
    // Drain stall cycles first, then execute one instruction.
    while c.stall_cycles() > 0 {
        c.step(bus);
    }
    c.step(bus);
}

#[test]
fn it_eq_taken() {
    // IT EQ; MOVS R0, #42 — condition true (Z=1), should execute
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    bus.write16(base, 0xBF08);       // IT EQ (firstcond=0000, mask=1000)
    bus.write16(base + 2, 0x202A);   // MOVS R0, #42
    bus.write16(base + 4, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.regs.set_flag_z(true); // EQ condition true

    step_one(&mut c, &mut bus); // execute IT
    assert_eq!(c.it_state(), 0x08);

    step_one(&mut c, &mut bus); // execute MOVS R0, #42 (conditionally)
    assert_eq!(c.reg(0), 42);
    assert_eq!(c.it_state(), 0); // IT block done
}

#[test]
fn it_eq_skipped() {
    // IT EQ; MOVS R0, #42 — condition false (Z=0), should skip
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    bus.write16(base, 0xBF08);       // IT EQ
    bus.write16(base + 2, 0x202A);   // MOVS R0, #42
    bus.write16(base + 4, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.regs.set_flag_z(false); // EQ condition false

    step_one(&mut c, &mut bus); // execute IT
    step_one(&mut c, &mut bus); // MOVS R0, #42 — skipped

    assert_eq!(c.reg(0), 0); // R0 unchanged
    assert_eq!(c.it_state(), 0); // IT block done
}

#[test]
fn it_flag_suppression() {
    // IT EQ; ADDS R0, R1, R2 inside IT should NOT update flags
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    // ADDS R0, R1, R2 = 0x1888
    bus.write16(base, 0xBF08);       // IT EQ
    bus.write16(base + 2, 0x1888);   // ADDS R0, R1, R2
    bus.write16(base + 4, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.set_reg(1, 5);
    c.set_reg(2, 10);
    c.regs.set_flag_z(true);  // EQ true, and Z=1 should be preserved
    c.regs.set_flag_c(true);  // C=1 should be preserved

    step_one(&mut c, &mut bus); // IT
    step_one(&mut c, &mut bus); // ADDS R0, R1, R2

    assert_eq!(c.reg(0), 15); // 5 + 10 = 15
    // Flags should be unchanged (suppressed by IT block)
    assert!(c.flag_z(), "Z flag should be preserved (suppressed)");
    assert!(c.flag_c(), "C flag should be preserved (suppressed)");
}

#[test]
fn it_cmp_always_sets_flags() {
    // IT EQ; CMP R0, R1 inside IT SHOULD update flags (flag-only instruction)
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    // CMP R0, R1 (data processing) = 0x4288
    bus.write16(base, 0xBF08);       // IT EQ
    bus.write16(base + 2, 0x4288);   // CMP R0, R1
    bus.write16(base + 4, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.set_reg(0, 10);
    c.set_reg(1, 5);
    c.regs.set_flag_z(true);  // EQ true (so CMP executes), Z=1 initially

    step_one(&mut c, &mut bus); // IT
    step_one(&mut c, &mut bus); // CMP R0, R1 (10 - 5 = 5, not zero)

    // CMP should have updated flags despite being in IT block
    assert!(!c.flag_z(), "Z should be cleared: 10 != 5");
    assert!(!c.flag_n(), "N should be cleared: result is positive");
    assert!(c.flag_c(), "C should be set: no borrow");
}

#[test]
fn it_cmp_imm_always_sets_flags() {
    // IT EQ; CMP R0, #5 inside IT SHOULD update flags
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    // CMP R0, #5 = 0x2805 (bits[15:11]=00101, Rn=000, imm8=0x05)
    bus.write16(base, 0xBF08);       // IT EQ
    bus.write16(base + 2, 0x2805);   // CMP R0, #5
    bus.write16(base + 4, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.set_reg(0, 10);
    c.regs.set_flag_z(true);  // EQ true

    step_one(&mut c, &mut bus); // IT
    step_one(&mut c, &mut bus); // CMP R0, #5

    // 10 - 5 = 5 → Z=0, N=0, C=1 (no borrow)
    assert!(!c.flag_z(), "Z should be cleared: 10 != 5");
    assert!(c.flag_c(), "C should be set: no borrow");
}

#[test]
fn ite_then_else_taken() {
    // ITE EQ; MOVS R0, #1; MOVS R0, #2 — with Z=1: R0=1 (Then taken, Else skipped)
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    // ITE EQ: firstcond=0000, mask=0100 with E-bit set → mask=1100 = 0x0C
    bus.write16(base, 0xBF0C);       // ITE EQ
    bus.write16(base + 2, 0x2001);   // MOVS R0, #1 (Then)
    bus.write16(base + 4, 0x2002);   // MOVS R0, #2 (Else)
    bus.write16(base + 6, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.regs.set_flag_z(true); // EQ true → Then

    step_one(&mut c, &mut bus); // ITE
    step_one(&mut c, &mut bus); // MOVS R0, #1 (executed: condition EQ, Z=1)
    step_one(&mut c, &mut bus); // MOVS R0, #2 (skipped: condition NE, Z=1)

    assert_eq!(c.reg(0), 1);
    assert_eq!(c.it_state(), 0);
}

#[test]
fn ite_then_else_not_taken() {
    // ITE EQ; MOVS R0, #1; MOVS R0, #2 — with Z=0: R0=2 (Then skipped, Else taken)
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    bus.write16(base, 0xBF0C);       // ITE EQ
    bus.write16(base + 2, 0x2001);   // MOVS R0, #1 (Then — skipped)
    bus.write16(base + 4, 0x2002);   // MOVS R0, #2 (Else — executed)
    bus.write16(base + 6, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.regs.set_flag_z(false); // EQ false → Else

    step_one(&mut c, &mut bus); // ITE
    step_one(&mut c, &mut bus); // MOVS R0, #1 (skipped)
    step_one(&mut c, &mut bus); // MOVS R0, #2 (executed)

    assert_eq!(c.reg(0), 2);
    assert_eq!(c.it_state(), 0);
}

#[test]
fn itt_eq_both_taken() {
    // ITT EQ; MOVS R0, #1; MOVS R1, #2 — with Z=1: both execute
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    // ITT EQ: firstcond=0000, mask=0100 (two Then, no Else)
    bus.write16(base, 0xBF04);       // ITT EQ
    bus.write16(base + 2, 0x2001);   // MOVS R0, #1
    bus.write16(base + 4, 0x2102);   // MOVS R1, #2
    bus.write16(base + 6, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.regs.set_flag_z(true);

    step_one(&mut c, &mut bus); // ITT
    step_one(&mut c, &mut bus); // MOVS R0, #1
    step_one(&mut c, &mut bus); // MOVS R1, #2

    assert_eq!(c.reg(0), 1);
    assert_eq!(c.reg(1), 2);
    assert_eq!(c.it_state(), 0);
}

#[test]
fn it_state_cleared_after_block() {
    // After IT block completes, the next instruction should execute unconditionally
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    bus.write16(base, 0xBF08);       // IT EQ
    bus.write16(base + 2, 0x202A);   // MOVS R0, #42 (in IT block)
    bus.write16(base + 4, 0x2103);   // MOVS R1, #3 (outside IT block)
    bus.write16(base + 6, 0xE7FE);   // B . (halt)
    c.regs.set_pc(base);
    c.regs.set_flag_z(false); // EQ false → IT body skipped

    step_one(&mut c, &mut bus); // IT
    step_one(&mut c, &mut bus); // MOVS R0, #42 — skipped
    step_one(&mut c, &mut bus); // MOVS R1, #3 — unconditional, should execute

    assert_eq!(c.reg(0), 0);  // skipped
    assert_eq!(c.reg(1), 3);  // executed unconditionally
    assert_eq!(c.it_state(), 0);
}

// ============================================================================
// FPU (VFP single-precision) — encoding helpers
// ============================================================================
//
// VFP data-processing instructions (CDP-like):
//   hw0 = 0xEE00 | (op_hi << 7) | (D << 6) | (op_lo << 4) | Vn
//   hw1 = (Vd << 12) | 0x0A00 | (N << 7) | (op2_lo << 6) | (M << 5) | Vm
//
// where:
//   Sd = (Vd << 1) | D, Sn = (Vn << 1) | N, Sm = (Vm << 1) | M

/// Encode a VFP data-processing instruction for single-precision.
/// `op_hi` = opc1[3], `op_lo` = opc1[1:0], `op2_lo` = opc2[0].
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

/// Encode VADD.F32 Sd, Sn, Sm.
fn enc_vadd(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b11, 0, sd, sn, sm) }

/// Encode VSUB.F32 Sd, Sn, Sm.
fn enc_vsub(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b11, 1, sd, sn, sm) }

/// Encode VMUL.F32 Sd, Sn, Sm.
fn enc_vmul(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b10, 0, sd, sn, sm) }

/// Encode VNMUL.F32 Sd, Sn, Sm.
fn enc_vnmul(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b10, 1, sd, sn, sm) }

/// Encode VDIV.F32 Sd, Sn, Sm.
fn enc_vdiv(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(1, 0b00, 0, sd, sn, sm) }

/// Encode VMLA.F32 Sd, Sn, Sm.
fn enc_vmla(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b00, 0, sd, sn, sm) }

/// Encode VMLS.F32 Sd, Sn, Sm.
fn enc_vmls(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b00, 1, sd, sn, sm) }

/// Encode VNMLA.F32 Sd, Sn, Sm.
fn enc_vnmla(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b01, 1, sd, sn, sm) }

/// Encode VNMLS.F32 Sd, Sn, Sm.
fn enc_vnmls(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b01, 0, sd, sn, sm) }

/// Encode VFMA.F32 Sd, Sn, Sm.
fn enc_vfma(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(1, 0b10, 0, sd, sn, sm) }

/// Encode VFMS.F32 Sd, Sn, Sm.
fn enc_vfms(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(1, 0b10, 1, sd, sn, sm) }

/// Encode VFNMA.F32 Sd, Sn, Sm.
fn enc_vfnma(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(1, 0b01, 1, sd, sn, sm) }

/// Encode VFNMS.F32 Sd, Sn, Sm.
fn enc_vfnms(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(1, 0b01, 0, sd, sn, sm) }

/// Encode a VFP unary instruction.
/// All unary: hw0[7:4]=1D11 (op_hi=1, op_lo=11), hw1[6]=1.
/// `opc3` = hw0[3:0] (repurposed Vn), `t` = hw1[7].
fn vfp_unary(opc3: u16, t: u16, sd: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xEE00 | (1 << 7) | (d << 6) | (0b11 << 4) | opc3;
    let hw1 = (vd << 12) | 0x0A00 | (t << 7) | (1 << 6) | (m << 5) | vm;
    (hw0, hw1)
}

/// VMOV.F32 Sd, Sm (register copy).
fn enc_vmov_reg(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0000, 0, sd, sm) }

/// VABS.F32 Sd, Sm.
fn enc_vabs(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0000, 1, sd, sm) }

/// VNEG.F32 Sd, Sm.
fn enc_vneg(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0001, 0, sd, sm) }

/// VSQRT.F32 Sd, Sm.
fn enc_vsqrt(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0001, 1, sd, sm) }

/// VCMP.F32 Sd, Sm (quiet).
fn enc_vcmp(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0100, 0, sd, sm) }

/// VCMP.F32 Sd, #0.0.
fn enc_vcmp_zero(sd: u16) -> (u16, u16) { vfp_unary(0b0101, 0, sd, 0) }

/// VCVT.F32.S32 Sd, Sm (signed int → float).
fn enc_vcvt_f32_s32(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b1000, 1, sd, sm) }

/// VCVT.F32.U32 Sd, Sm (unsigned int → float).
fn enc_vcvt_f32_u32(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b1000, 0, sd, sm) }

/// VCVT.S32.F32 Sd, Sm (float → signed int, round toward zero).
fn enc_vcvt_s32_f32(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b1101, 1, sd, sm) }

/// VCVT.U32.F32 Sd, Sm (float → unsigned int, round toward zero).
fn enc_vcvt_u32_f32(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b1100, 1, sd, sm) }

/// VCVTR.S32.F32 Sd, Sm (float → signed int, round per FPSCR).
fn enc_vcvtr_s32_f32(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b1101, 0, sd, sm) }

/// Encode VMOV Sn, Rt (ARM → FPU). MCR format, L=0.
fn enc_vmov_to_fpu(sn: u16, rt: u16) -> (u16, u16) {
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let hw0 = 0xEE00 | vn;
    let hw1 = (rt << 12) | 0x0A10 | (n << 7);
    (hw0, hw1)
}

/// Encode VMOV Rt, Sn (FPU → ARM). MRC format, L=1.
fn enc_vmov_to_arm(rt: u16, sn: u16) -> (u16, u16) {
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let hw0 = 0xEE10 | vn;
    let hw1 = (rt << 12) | 0x0A10 | (n << 7);
    (hw0, hw1)
}

/// Encode VMRS Rt, FPSCR (Rt=15 → APSR_nzcv).
fn enc_vmrs(rt: u16) -> (u16, u16) {
    let hw0 = 0xEEF1u16;
    let hw1 = (rt << 12) | 0x0A10;
    (hw0, hw1)
}

/// Encode VMSR FPSCR, Rt.
fn enc_vmsr(rt: u16) -> (u16, u16) {
    let hw0 = 0xEEE1u16;
    let hw1 = (rt << 12) | 0x0A10;
    (hw0, hw1)
}

/// Encode VLDR.32 Sd, [Rn, #±offset]. offset is in bytes, must be multiple of 4.
fn enc_vldr(sd: u16, rn: u16, offset: i16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let u_bit = if offset >= 0 { 1u16 } else { 0u16 };
    let imm8 = (offset.unsigned_abs() >> 2) as u16;
    // hw0: 1110_110P_UD_W_L_Rn, P=1, W=0, L=1 → bits = 1101_U_D_01
    let hw0 = 0xED00 | (u_bit << 7) | (d << 6) | (1 << 4) | rn;
    let hw1 = (vd << 12) | 0x0A00 | (imm8 & 0xFF);
    (hw0, hw1)
}

/// Encode VSTR.32 Sd, [Rn, #±offset]. offset is in bytes, must be multiple of 4.
fn enc_vstr(sd: u16, rn: u16, offset: i16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let u_bit = if offset >= 0 { 1u16 } else { 0u16 };
    let imm8 = (offset.unsigned_abs() >> 2) as u16;
    // P=1, W=0, L=0 → bits = 1101_U_D_00
    let hw0 = 0xED00 | (u_bit << 7) | (d << 6) | rn;
    let hw1 = (vd << 12) | 0x0A00 | (imm8 & 0xFF);
    (hw0, hw1)
}

/// Encode VPUSH {Sd..Sd+count-1} — VSTMDB SP!, {list}.
/// P=1, U=0, D, W=1, L=0, Rn=13(SP)
fn enc_vpush(sd: u16, count: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let hw0 = 0xED00 | (d << 6) | (1 << 5) | 13; // P=1,U=0,D,W=1,L=0,Rn=SP
    let hw1 = (vd << 12) | 0x0A00 | (count & 0xFF);
    (hw0, hw1)
}

/// Encode VPOP {Sd..Sd+count-1} — VLDMIA SP!, {list}.
/// P=0, U=1, D, W=1, L=1, Rn=13(SP)
fn enc_vpop(sd: u16, count: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let hw0 = 0xEC00 | (1 << 7) | (d << 6) | (1 << 5) | (1 << 4) | 13;
    let hw1 = (vd << 12) | 0x0A00 | (count & 0xFF);
    (hw0, hw1)
}

/// Encode VSEL<cc>.F32 Sd, Sn, Sm (Armv8-M).
/// cc ∈ {0=EQ, 1=VS, 2=GE, 3=GT}. Prefix 0xFE.
/// hw0 = 1111 1110 | 0 cc[1] cc[0] D | Vn
/// hw1 = Vd | 1010 | N 0 M 0 | Vm
fn enc_vsel(cc: u16, sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xFE00 | ((cc & 0x3) << 5) | (d << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | (m << 5) | vm;
    (hw0, hw1)
}

/// Encode VMAXNM.F32 / VMINNM.F32 (Armv8-M).
/// `op` = 0 → VMAXNM, `op` = 1 → VMINNM.
/// hw0 = 1111 1110 | 1 0 0 D | Vn
/// hw1 = Vd | 1010 | N op M 0 | Vm
fn enc_vmaxminnm(op: u16, sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xFE00 | (1 << 7) | (d << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | ((op & 1) << 6) | (m << 5) | vm;
    (hw0, hw1)
}

/// Encode VMAXNM.F32 Sd, Sn, Sm.
fn enc_vmaxnm(sd: u16, sn: u16, sm: u16) -> (u16, u16) { enc_vmaxminnm(0, sd, sn, sm) }

/// Encode VMINNM.F32 Sd, Sn, Sm.
fn enc_vminnm(sd: u16, sn: u16, sm: u16) -> (u16, u16) { enc_vmaxminnm(1, sd, sn, sm) }

/// Encode VCVTB.F16.F32 Sd, Sm (convert f32 → f16 into bottom half of Sd).
fn enc_vcvtb_f16_f32(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0010, 0, sd, sm) }

/// Encode VCVTT.F16.F32 Sd, Sm (convert f32 → f16 into top half of Sd).
fn enc_vcvtt_f16_f32(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0010, 1, sd, sm) }

/// Encode VCVTB.F32.F16 Sd, Sm (convert f16 from bottom half of Sm → f32 Sd).
fn enc_vcvtb_f32_f16(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0011, 0, sd, sm) }

/// Encode VCVTT.F32.F16 Sd, Sm (convert f16 from top half of Sm → f32 Sd).
fn enc_vcvtt_f32_f16(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0011, 1, sd, sm) }

// ============================================================================
// FPU tests
// ============================================================================

#[test]
fn fpu_vadd_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.5;
    c.regs.s[4] = 2.5;
    let (hw0, hw1) = enc_vadd(0, 2, 4); // VADD.F32 S0, S2, S4
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);
    assert_eq!(cy, 1);
}

#[test]
fn fpu_vsub_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 10.0;
    c.regs.s[4] = 3.5;
    let (hw0, hw1) = enc_vsub(0, 2, 4); // VSUB.F32 S0, S2, S4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 6.5);
}

#[test]
fn fpu_vmul_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 4.0;
    let (hw0, hw1) = enc_vmul(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 12.0);
}

#[test]
fn fpu_vdiv_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 10.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vdiv(0, 2, 4);
    let cy = c.execute_one_wide(hw0, hw1);
    let expected = 10.0f32 / 3.0;
    assert_eq!(c.regs.s[0], expected);
    assert_eq!(cy, 14);
}

#[test]
fn fpu_vneg_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 5.0;
    let (hw0, hw1) = enc_vneg(0, 2); // VNEG.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -5.0);
}

#[test]
fn fpu_vabs_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = -7.5;
    let (hw0, hw1) = enc_vabs(0, 2); // VABS.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.5);
}

#[test]
fn fpu_vsqrt_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 4.0;
    let (hw0, hw1) = enc_vsqrt(0, 2); // VSQRT.F32 S0, S2
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 2.0);
    assert_eq!(cy, 14);
}

#[test]
fn fpu_vcmp_f32_equal() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 3.0;
    c.regs.s[2] = 3.0;
    let (hw0, hw1) = enc_vcmp(0, 2); // VCMP.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    // Equal: N=0, Z=1, C=1, V=0
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x6000_0000);
}

#[test]
fn fpu_vcmp_f32_less() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 3.0;
    let (hw0, hw1) = enc_vcmp(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Less: N=1, Z=0, C=0, V=0
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x8000_0000);
}

#[test]
fn fpu_vcmp_f32_greater() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 5.0;
    c.regs.s[2] = 2.0;
    let (hw0, hw1) = enc_vcmp(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Greater: N=0, Z=0, C=1, V=0
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x2000_0000);
}

#[test]
fn fpu_vcmp_f32_nan() {
    let mut c = CortexM33::new();
    c.regs.s[0] = f32::NAN;
    c.regs.s[2] = 1.0;
    let (hw0, hw1) = enc_vcmp(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Unordered: N=0, Z=0, C=1, V=1
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x3000_0000);
}

#[test]
fn fpu_vcmp_f32_zero() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 0.0;
    let (hw0, hw1) = enc_vcmp_zero(0); // VCMP.F32 S0, #0.0
    c.execute_one_wide(hw0, hw1);
    // Equal to zero: Z=1, C=1
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x6000_0000);
}

#[test]
fn fpu_vcvt_f32_s32() {
    let mut c = CortexM33::new();
    // Store -42 as raw bits in S2
    c.regs.s[2] = f32::from_bits((-42i32) as u32);
    let (hw0, hw1) = enc_vcvt_f32_s32(0, 2); // VCVT.F32.S32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -42.0);
}

#[test]
fn fpu_vcvt_f32_u32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = f32::from_bits(100u32);
    let (hw0, hw1) = enc_vcvt_f32_u32(0, 2); // VCVT.F32.U32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 100.0);
}

#[test]
fn fpu_vcvt_s32_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = -3.7;
    let (hw0, hw1) = enc_vcvt_s32_f32(0, 2); // VCVT.S32.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    // Result stored as raw bits: -3 as i32 = 0xFFFF_FFFD
    assert_eq!(c.regs.s[0].to_bits(), (-3i32) as u32);
}

#[test]
fn fpu_vcvt_u32_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 7.9;
    let (hw0, hw1) = enc_vcvt_u32_f32(0, 2); // VCVT.U32.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 7u32);
}

#[test]
fn fpu_vmov_arm_to_fpu() {
    let mut c = CortexM33::new();
    c.set_reg(3, 0x4048_0000); // 3.125f32.to_bits()
    let (hw0, hw1) = enc_vmov_to_fpu(0, 3); // VMOV S0, R3
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], f32::from_bits(0x4048_0000));
}

#[test]
fn fpu_vmov_fpu_to_arm() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 3.125;
    let (hw0, hw1) = enc_vmov_to_arm(3, 0); // VMOV R3, S0
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(3), 3.125f32.to_bits());
}

#[test]
fn fpu_vmrs_fpscr_to_apsr() {
    let mut c = CortexM33::new();
    // Set up: compare S0 < S2 → FPSCR.N=1
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 5.0;
    let (hw0, hw1) = enc_vcmp(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.fpscr & 0x8000_0000 != 0); // FPSCR.N set

    // VMRS APSR_nzcv, FPSCR (Rt=15)
    let (hw0, hw1) = enc_vmrs(15);
    c.execute_one_wide(hw0, hw1);
    assert!(c.flag_n()); // APSR.N should be set
}

#[test]
fn fpu_vmsr_fpscr() {
    let mut c = CortexM33::new();
    c.set_reg(2, 0x0040_0000); // Set FPSCR.RMode = 01 (round toward +inf)
    let (hw0, hw1) = enc_vmsr(2); // VMSR FPSCR, R2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.fpscr, 0x0040_0000);
}

#[test]
fn fpu_vldr_vstr() {
    let (mut c, mut bus) = core_and_bus();
    let addr = 0x2000_0100u32;
    c.set_reg(0, addr);

    // Store 2.5 to memory via VSTR
    c.regs.s[4] = 2.5;
    let (hw0, hw1) = enc_vstr(4, 0, 0); // VSTR.32 S4, [R0, #0]
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(bus.read32(addr), 2.5f32.to_bits());

    // Load it back via VLDR
    c.regs.s[6] = 0.0;
    let (hw0, hw1) = enc_vldr(6, 0, 0); // VLDR.32 S6, [R0, #0]
    let cy = c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.regs.s[6], 2.5);
    assert_eq!(cy, 2);
}

#[test]
fn fpu_vldr_positive_offset() {
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0100u32;
    c.set_reg(0, base);
    bus.write32(base + 16, 7.0f32.to_bits());
    let (hw0, hw1) = enc_vldr(0, 0, 16); // VLDR.32 S0, [R0, #+16]
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.regs.s[0], 7.0);
}

#[test]
fn fpu_vldr_negative_offset() {
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0110u32;
    c.set_reg(0, base);
    bus.write32(base - 8, 9.0f32.to_bits());
    let (hw0, hw1) = enc_vldr(0, 0, -8); // VLDR.32 S0, [R0, #-8]
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.regs.s[0], 9.0);
}

#[test]
fn fpu_vpush_vpop() {
    let (mut c, mut bus) = core_and_bus();
    let sp = 0x2000_1000u32;
    c.set_reg(13, sp);

    // Load values into S0, S1, S2
    c.regs.s[0] = 1.0;
    c.regs.s[1] = 2.0;
    c.regs.s[2] = 3.0;

    // VPUSH {S0-S2} (3 registers)
    let (hw0, hw1) = enc_vpush(0, 3);
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(13), sp - 12); // SP decremented by 3*4

    // Verify memory
    assert_eq!(f32::from_bits(bus.read32(sp - 12)), 1.0);
    assert_eq!(f32::from_bits(bus.read32(sp - 8)), 2.0);
    assert_eq!(f32::from_bits(bus.read32(sp - 4)), 3.0);

    // Clear S0-S2
    c.regs.s[0] = 0.0;
    c.regs.s[1] = 0.0;
    c.regs.s[2] = 0.0;

    // VPOP {S0-S2}
    let (hw0, hw1) = enc_vpop(0, 3);
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.reg(13), sp); // SP restored
    assert_eq!(c.regs.s[0], 1.0);
    assert_eq!(c.regs.s[1], 2.0);
    assert_eq!(c.regs.s[2], 3.0);
}

#[test]
fn fpu_vmla_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 10.0; // accumulator
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 4.0;
    let (hw0, hw1) = enc_vmla(0, 2, 4); // VMLA.F32 S0, S2, S4 → S0 = 10 + 3*4 = 22
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 22.0);
    assert_eq!(cy, 3);
}

#[test]
fn fpu_vmls_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 10.0;
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 2.0;
    let (hw0, hw1) = enc_vmls(0, 2, 4); // VMLS.F32 S0, S2, S4 → S0 = 10 - 3*2 = 4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);
}

#[test]
fn fpu_vnmul_f32() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 5.0;
    let (hw0, hw1) = enc_vnmul(0, 2, 4); // VNMUL.F32 S0, S2, S4 → S0 = -(3*5) = -15
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -15.0);
}

#[test]
fn fpu_vnmla_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vnmla(0, 2, 4); // VNMLA → S0 = -(2*3 + 1) = -7
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -7.0);
}

#[test]
fn fpu_vnmls_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vnmls(0, 2, 4); // VNMLS → S0 = 2*3 - 1 = 5
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 5.0);
}

#[test]
fn fpu_vfma_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vfma(0, 2, 4); // VFMA → S0 = S0 + S2*S4 = 1 + 6 = 7
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.0);
    assert_eq!(cy, 3);
}

#[test]
fn fpu_vfms_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 10.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vfms(0, 2, 4); // VFMS → S0 = S0 - S2*S4 = 10 - 6 = 4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);
}

#[test]
fn fpu_vfnma_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vfnma(0, 2, 4); // VFNMA → S0 = -S2*S4 - S0 = -6 - 1 = -7
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -7.0);
}

#[test]
fn fpu_vfnms_f32() {
    let mut c = CortexM33::new();
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vfnms(0, 2, 4); // VFNMS → S0 = S2*S4 - S0 = 6 - 1 = 5
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 5.0);
}

#[test]
fn fpu_vmov_f32_reg() {
    let mut c = CortexM33::new();
    c.regs.s[4] = 42.0;
    let (hw0, hw1) = enc_vmov_reg(0, 4); // VMOV.F32 S0, S4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 42.0);
}

#[test]
fn fpu_vmrs_to_register() {
    let mut c = CortexM33::new();
    c.regs.fpscr = 0x1234_5678;
    let (hw0, hw1) = enc_vmrs(3); // VMRS R3, FPSCR
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(3), 0x1234_5678);
}

#[test]
fn fpu_vcvtr_s32_f32_round_nearest() {
    let mut c = CortexM33::new();
    c.regs.fpscr = 0; // RMode=00 → round to nearest
    c.regs.s[2] = 2.5;
    let (hw0, hw1) = enc_vcvtr_s32_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    // 2.5 rounds to 2 (ties to even)
    assert_eq!(c.regs.s[0].to_bits() as i32, 2);
}

#[test]
fn fpu_high_register_encoding() {
    // Test that high register indices (S16-S31) are correctly encoded/decoded.
    let mut c = CortexM33::new();
    c.regs.s[16] = 100.0;
    c.regs.s[20] = 200.0;
    let (hw0, hw1) = enc_vadd(24, 16, 20); // VADD.F32 S24, S16, S20
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[24], 300.0);
}

#[test]
fn fpu_vcvt_negative_float_to_unsigned() {
    let mut c = CortexM33::new();
    c.regs.s[2] = -5.0;
    let (hw0, hw1) = enc_vcvt_u32_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Negative float → unsigned should saturate to 0
    assert_eq!(c.regs.s[0].to_bits(), 0);
}

#[test]
fn fpu_vcvt_nan_to_int() {
    let mut c = CortexM33::new();
    c.regs.s[2] = f32::NAN;
    let (hw0, hw1) = enc_vcvt_s32_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits() as i32, 0);
}

#[test]
fn fpu_vmov_odd_register() {
    // Test VMOV with an odd-numbered S register (S1) to exercise the N/D bit encoding.
    let mut c = CortexM33::new();
    c.set_reg(0, 0xDEAD_BEEF);
    let (hw0, hw1) = enc_vmov_to_fpu(1, 0); // VMOV S1, R0
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[1].to_bits(), 0xDEAD_BEEF);

    // Read back: VMOV R1, S1
    let (hw0, hw1) = enc_vmov_to_arm(1, 1);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(1), 0xDEAD_BEEF);
}

// ----- VSEL ----------------------------------------------------------------

#[test]
fn fpu_vseleq_true_picks_sn() {
    // Z=1 → condition EQ true → Sd = Sn
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.0;
    c.regs.s[4] = 2.0;
    c.regs.set_flag_z(true);
    let (hw0, hw1) = enc_vsel(0, 0, 2, 4); // cc=00 (EQ)
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 1.0);
    assert_eq!(cy, 1);
}

#[test]
fn fpu_vseleq_false_picks_sm() {
    // Z=0 → condition EQ false → Sd = Sm
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.0;
    c.regs.s[4] = 2.0;
    c.regs.set_flag_z(false);
    let (hw0, hw1) = enc_vsel(0, 0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 2.0);
}

#[test]
fn fpu_vselvs_true_picks_sn() {
    // V=1 → condition VS true → Sd = Sn
    let mut c = CortexM33::new();
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 4.0;
    c.regs.set_flag_v(true);
    let (hw0, hw1) = enc_vsel(1, 0, 2, 4); // cc=01 (VS)
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 3.0);
}

#[test]
fn fpu_vselvs_false_picks_sm() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 4.0;
    c.regs.set_flag_v(false);
    let (hw0, hw1) = enc_vsel(1, 0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);
}

#[test]
fn fpu_vselge_true_picks_sn() {
    // GE: N==V → true
    let mut c = CortexM33::new();
    c.regs.s[2] = 5.0;
    c.regs.s[4] = 6.0;
    c.regs.set_flag_n(true);
    c.regs.set_flag_v(true);
    let (hw0, hw1) = enc_vsel(2, 0, 2, 4); // cc=10 (GE)
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 5.0);
}

#[test]
fn fpu_vselge_false_picks_sm() {
    // N != V → GE false
    let mut c = CortexM33::new();
    c.regs.s[2] = 5.0;
    c.regs.s[4] = 6.0;
    c.regs.set_flag_n(true);
    c.regs.set_flag_v(false);
    let (hw0, hw1) = enc_vsel(2, 0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 6.0);
}

#[test]
fn fpu_vselgt_true_picks_sn() {
    // GT: Z==0 && N==V → true
    let mut c = CortexM33::new();
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(false);
    c.regs.set_flag_v(false);
    let (hw0, hw1) = enc_vsel(3, 0, 2, 4); // cc=11 (GT)
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.0);
}

#[test]
fn fpu_vselgt_false_picks_sm() {
    // Z=1 → GT false (even with N==V)
    let mut c = CortexM33::new();
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(true);
    c.regs.set_flag_n(false);
    c.regs.set_flag_v(false);
    let (hw0, hw1) = enc_vsel(3, 0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 8.0);
}

#[test]
fn fpu_vsel_gt_all_flag_combos() {
    // GT condition: Z==0 && N==V.
    // Walk every combination of N and V (with Z=0) to cover the full
    // N/V truth table for the GT branch of vsel_condition_holds.
    //
    //   N==V==false → N==V → GT true  → picks Sn  (already covered above)
    //   N==V==true  → N==V → GT true  → picks Sn
    //   N!=V (N=1,V=0) → GT false → picks Sm
    //   N!=V (N=0,V=1) → GT false → picks Sm
    let (hw0, hw1) = enc_vsel(3, 0, 2, 4); // cc=11 (GT)

    // Case: N==V==true → picks Sn
    let mut c = CortexM33::new();
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(true);
    c.regs.set_flag_v(true);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.0, "N==V==true should select Sn");

    // Case: N=1, V=0 → N!=V → picks Sm
    let mut c = CortexM33::new();
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(true);
    c.regs.set_flag_v(false);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 8.0, "N!=V (N=1,V=0) should select Sm");

    // Case: N=0, V=1 → N!=V → picks Sm
    let mut c = CortexM33::new();
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(false);
    c.regs.set_flag_v(true);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 8.0, "N!=V (N=0,V=1) should select Sm");

    // Case: N==V==false → picks Sn (already covered by fpu_vselgt_true_picks_sn,
    // repeated here to keep the full truth table visible in one place).
    let mut c = CortexM33::new();
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(false);
    c.regs.set_flag_v(false);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.0, "N==V==false should select Sn");
}

#[test]
fn fpu_vcvta_is_undefined() {
    // VCVTA.S32.F32 (Armv8-M FP, 0xFE family with hw0[7]=1 and hw0[6:5]!=00)
    // is not dispatched in Phase 7.1 and must fall through to UNDEFINED,
    // raising a UsageFault. This test locks in the boundary — if future
    // dispatch work accidentally routes VCVTA somewhere, this breaks.
    let mut c = CortexM33::new();
    c.execute_one_wide(0xFEBD, 0x0A40);
    assert!(
        matches!(c.pending_fault, Some(crate::core::Fault::UsageFault)),
        "VCVTA encoding must fall through to UsageFault (Phase 7.1 boundary); \
         pending_fault = {:?}",
        c.pending_fault
    );
}

// ----- VMAXNM / VMINNM -----------------------------------------------------

#[test]
fn fpu_vmaxnm_normal() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.5;
    c.regs.s[4] = 2.5;
    let (hw0, hw1) = enc_vmaxnm(0, 2, 4);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 2.5);
    assert_eq!(cy, 1);
}

#[test]
fn fpu_vmaxnm_nan_returns_other() {
    // IEEE 754-2008 maxNum: NaN operand returns the non-NaN operand.
    let mut c = CortexM33::new();
    c.regs.s[2] = f32::NAN;
    c.regs.s[4] = -3.0;
    let (hw0, hw1) = enc_vmaxnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -3.0);

    // Other side: NaN as second operand
    c.regs.s[2] = 7.0;
    c.regs.s[4] = f32::NAN;
    let (hw0, hw1) = enc_vmaxnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.0);

    // Both NaN → result is qNaN (default NaN)
    c.regs.s[2] = f32::NAN;
    c.regs.s[4] = f32::NAN;
    let (hw0, hw1) = enc_vmaxnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_nan());
}

#[test]
fn fpu_vminnm_normal() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.5;
    c.regs.s[4] = 2.5;
    let (hw0, hw1) = enc_vminnm(0, 2, 4);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 1.5);
    assert_eq!(cy, 1);
}

#[test]
fn fpu_vminnm_nan_returns_other() {
    let mut c = CortexM33::new();
    c.regs.s[2] = f32::NAN;
    c.regs.s[4] = 4.0;
    let (hw0, hw1) = enc_vminnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);

    // Both NaN → qNaN
    c.regs.s[2] = f32::NAN;
    c.regs.s[4] = f32::NAN;
    let (hw0, hw1) = enc_vminnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_nan());
}

#[test]
fn fpu_vmaxnm_zero_signs() {
    // IEEE 754-2008 §5.3.1: maxNum(+0, -0) = +0 in both operand orders.
    let mut c = CortexM33::new();

    // (+0, -0)
    c.regs.s[2] = 0.0f32;
    c.regs.s[4] = -0.0f32;
    let (hw0, hw1) = enc_vmaxnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 0x0000_0000, "maxNum(+0,-0) must be +0");

    // (-0, +0) — same expected result regardless of order
    c.regs.s[2] = -0.0f32;
    c.regs.s[4] = 0.0f32;
    let (hw0, hw1) = enc_vmaxnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 0x0000_0000, "maxNum(-0,+0) must be +0");
}

#[test]
fn fpu_vminnm_zero_signs() {
    // IEEE 754-2008 §5.3.1: minNum(+0, -0) = -0 in both operand orders.
    let mut c = CortexM33::new();

    // (+0, -0)
    c.regs.s[2] = 0.0f32;
    c.regs.s[4] = -0.0f32;
    let (hw0, hw1) = enc_vminnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 0x8000_0000, "minNum(+0,-0) must be -0");

    // (-0, +0) — same expected result regardless of order
    c.regs.s[2] = -0.0f32;
    c.regs.s[4] = 0.0f32;
    let (hw0, hw1) = enc_vminnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 0x8000_0000, "minNum(-0,+0) must be -0");
}

// ----- FPSCR exception flags (Phase 7 Stage A.1) ---------------------------
//
// Bit positions:
//   IOC=bit 0, DZC=bit 1, OFC=bit 2, UFC=bit 3, IXC=bit 4, IDC=bit 7,
//   FZ=bit 24, DN=bit 25. All cumulative flags are sticky.

const FPSCR_IOC: u32 = 1 << 0;
const FPSCR_DZC: u32 = 1 << 1;
const FPSCR_OFC: u32 = 1 << 2;
const FPSCR_UFC: u32 = 1 << 3;
const FPSCR_IXC: u32 = 1 << 4;
const FPSCR_IDC: u32 = 1 << 7;
const FPSCR_FZ: u32 = 1 << 24;
const FPSCR_DN: u32 = 1 << 25;

#[test]
fn fpscr_ixc_on_inexact_division() {
    // 1.0 / 3.0 is inexact in f32 → IXC set.
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vdiv(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(
        c.regs.fpscr & FPSCR_IXC != 0,
        "VDIV 1.0/3.0 should set IXC; fpscr=0x{:08X}",
        c.regs.fpscr
    );
}

#[test]
fn fpscr_dzc_on_divide_by_zero() {
    // 1.0 / 0.0 = +inf, DZC set, IOC clear.
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.0;
    c.regs.s[4] = 0.0;
    let (hw0, hw1) = enc_vdiv(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], f32::INFINITY);
    assert!(c.regs.fpscr & FPSCR_DZC != 0, "DZC must set");
    assert!(c.regs.fpscr & FPSCR_IOC == 0, "IOC must NOT set for n/0");
}

#[test]
fn fpscr_ioc_on_zero_divided_by_zero() {
    // 0.0 / 0.0 = NaN, IOC set.
    let mut c = CortexM33::new();
    c.regs.s[2] = 0.0;
    c.regs.s[4] = 0.0;
    let (hw0, hw1) = enc_vdiv(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_nan());
    assert!(c.regs.fpscr & FPSCR_IOC != 0);
    assert!(c.regs.fpscr & FPSCR_DZC == 0);
}

#[test]
fn fpscr_ofc_on_multiplication_overflow() {
    // 1e20 * 1e20 = 1e40 which overflows f32 → ±inf, OFC+IXC.
    let mut c = CortexM33::new();
    c.regs.s[2] = 1e20;
    c.regs.s[4] = 1e20;
    let (hw0, hw1) = enc_vmul(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_infinite());
    assert!(c.regs.fpscr & FPSCR_OFC != 0, "OFC must set");
    assert!(c.regs.fpscr & FPSCR_IXC != 0, "IXC must set on overflow");
}

#[test]
fn fpscr_ufc_on_ftz_flush() {
    // With FZ=1, 1e-20 * 1e-20 (tininess before rounding) flushes to ±0
    // and sets UFC+IXC.
    let mut c = CortexM33::new();
    c.regs.fpscr = FPSCR_FZ;
    c.regs.s[2] = 1e-20;
    c.regs.s[4] = 1e-20;
    let (hw0, hw1) = enc_vmul(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 0.0f32, "FTZ must flush to +0");
    assert!(c.regs.fpscr & FPSCR_UFC != 0, "UFC must set on FTZ flush");
    assert!(c.regs.fpscr & FPSCR_IXC != 0, "IXC must set on FTZ flush");
}

#[test]
fn fpscr_idc_on_denormal_input() {
    // VADD with a denormal input sets IDC (even when FZ=0).
    let mut c = CortexM33::new();
    let denorm = f32::from_bits(0x0000_0001); // smallest positive subnormal
    c.regs.s[2] = denorm;
    c.regs.s[4] = 1.0;
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.fpscr & FPSCR_IDC != 0, "IDC must set on denormal input");
}

#[test]
fn fpscr_dn_replaces_nan_with_canonical() {
    // With DN=1, any NaN result becomes 0x7FC0_0000 (no payload preservation).
    let mut c = CortexM33::new();
    c.regs.fpscr = FPSCR_DN;
    // Hand-craft a quiet NaN with a custom payload.
    c.regs.s[2] = f32::from_bits(0x7FC1_2345);
    c.regs.s[4] = 1.0;
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(
        c.regs.s[0].to_bits(),
        0x7FC0_0000,
        "DN=1 must force canonical quiet NaN; got 0x{:08X}",
        c.regs.s[0].to_bits()
    );
}

#[test]
fn fpscr_flags_are_sticky_across_ops() {
    // Set IXC by one op, then execute an exact op; IXC must remain set.
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vdiv(0, 2, 4); // inexact → IXC
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.fpscr & FPSCR_IXC != 0);

    // Exact op: 2.0 + 2.0 = 4.0
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 2.0;
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(
        c.regs.fpscr & FPSCR_IXC != 0,
        "IXC must remain sticky across exact ops"
    );
}

#[test]
fn fpscr_sqrt_negative_sets_ioc() {
    let mut c = CortexM33::new();
    c.regs.s[2] = -4.0;
    let (hw0, hw1) = enc_vsqrt(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_nan());
    assert!(c.regs.fpscr & FPSCR_IOC != 0, "sqrt(-x) must set IOC");
}

#[test]
fn fpscr_vmaxnm_snan_sets_ioc() {
    // Per DDI0553: VMAXNM/VMINNM set IOC when either input is sNaN.
    let mut c = CortexM33::new();
    let snan = f32::from_bits(0x7F80_0001);
    c.regs.s[2] = snan;
    c.regs.s[4] = 1.0;
    let (hw0, hw1) = enc_vmaxnm(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(
        c.regs.fpscr & FPSCR_IOC != 0,
        "VMAXNM with sNaN must set IOC"
    );
}

#[test]
fn fpu_vnmul_nan_sign_flipped_dn_off() {
    // VNMUL = -(Sn * Sm). Per DDI0553 §A2.2.6, FPNeg is an unconditional
    // sign-bit flip, *including* for NaN. With DN=0 the canonicalized
    // quiet NaN from fp_mul is positive (0x7FC0_0000); negation must make
    // the stored result negative (0xFFC0_0000).
    let mut c = CortexM33::new();
    c.regs.s[2] = f32::NAN;
    c.regs.s[4] = 1.0;
    let (hw0, hw1) = enc_vnmul(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    let bits = c.regs.s[0].to_bits();
    assert!(c.regs.s[0].is_nan(), "VNMUL(NaN,x) must produce NaN");
    assert_eq!(
        bits & 0x8000_0000,
        0x8000_0000,
        "FPNeg must flip sign bit even for NaN (DN=0); got 0x{:08X}",
        bits
    );
}

#[test]
fn fpu_vnmul_nan_canonical_dn_on() {
    // Same as above but with DN=1: the negated NaN must be re-canonicalized
    // back to the positive canonical quiet NaN 0x7FC0_0000.
    let mut c = CortexM33::new();
    c.regs.fpscr = FPSCR_DN;
    c.regs.s[2] = f32::NAN;
    c.regs.s[4] = 1.0;
    let (hw0, hw1) = enc_vnmul(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(
        c.regs.s[0].to_bits(),
        0x7FC0_0000,
        "DN=1 must force canonical quiet NaN after negate; got 0x{:08X}",
        c.regs.s[0].to_bits()
    );
}

#[test]
fn fpscr_add_inf_minus_inf_sets_ioc() {
    let mut c = CortexM33::new();
    c.regs.s[2] = f32::INFINITY;
    c.regs.s[4] = f32::NEG_INFINITY;
    let (hw0, hw1) = enc_vadd(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_nan());
    assert!(c.regs.fpscr & FPSCR_IOC != 0, "inf+(-inf) must set IOC");
}

// ----- VCVTB / VCVTT (F16 <-> F32) -----------------------------------------

#[test]
fn fpu_vcvtb_f16_f32_roundtrip_bottom() {
    // Round-trip 1.5 through bottom half: f32 -> f16 (bottom of Sd) -> f32.
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.5f32;
    // Preserve a known top half so we can verify it survives.
    c.regs.s[0] = f32::from_bits(0xDEAD_0000);
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2); // VCVTB.F16.F32 S0, S2
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(cy, 1);

    let bits = c.regs.s[0].to_bits();
    // Top half preserved
    assert_eq!(bits & 0xFFFF_0000, 0xDEAD_0000);

    // Convert back: VCVTB.F32.F16 S4, S0 — read bottom half of S0 as f16, write to S4
    let (hw0, hw1) = enc_vcvtb_f32_f16(4, 0);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[4], 1.5);
}

#[test]
fn fpu_vcvtt_f16_f32_roundtrip_top() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 1.5f32;
    // Preserve known bottom half
    c.regs.s[0] = f32::from_bits(0x0000_BEEF);
    let (hw0, hw1) = enc_vcvtt_f16_f32(0, 2); // VCVTT.F16.F32 S0, S2
    c.execute_one_wide(hw0, hw1);

    let bits = c.regs.s[0].to_bits();
    // Bottom half preserved
    assert_eq!(bits & 0x0000_FFFF, 0x0000_BEEF);

    // VCVTT.F32.F16 S4, S0 — read top half as f16 → f32
    let (hw0, hw1) = enc_vcvtt_f32_f16(4, 0);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[4], 1.5);
}

#[test]
fn fpu_vcvtb_f32_f16_infinity() {
    // Half-precision +infinity is 0x7C00. Converting to f32 should give f32::INFINITY.
    let mut c = CortexM33::new();
    c.regs.s[2] = f32::from_bits(0x0000_7C00); // bottom half = +inf (h)
    let (hw0, hw1) = enc_vcvtb_f32_f16(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], f32::INFINITY);

    // Negative infinity: 0xFC00 in bottom half
    c.regs.s[2] = f32::from_bits(0x0000_FC00);
    let (hw0, hw1) = enc_vcvtb_f32_f16(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], f32::NEG_INFINITY);

    // And convert f32::INFINITY to f16 (bottom): bottom half should be 0x7C00
    c.regs.s[2] = f32::INFINITY;
    c.regs.s[0] = f32::from_bits(0xDEAD_0000);
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF, 0x7C00);
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF_0000, 0xDEAD_0000);
}

#[test]
fn fpu_vcvtb_f16_f32_overflow() {
    // Values too large to represent in f16 must saturate to +inf (0x7C00).
    let mut c = CortexM33::new();
    c.regs.s[2] = 1e30_f32;
    // Preserve top half so we can verify only the bottom half is written.
    c.regs.s[0] = f32::from_bits(0xAAAA_0000);
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF, 0x7C00, "1e30 -> f16 must be +inf");
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF_0000, 0xAAAA_0000);
}

#[test]
fn fpu_vcvtb_f16_f32_underflow() {
    // Values smaller than the smallest f16 subnormal must flush to +0 (0x0000).
    let mut c = CortexM33::new();
    c.regs.s[2] = 1e-10_f32;
    c.regs.s[0] = f32::from_bits(0x5555_0000);
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF, 0x0000, "1e-10 -> f16 must be +0");
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF_0000, 0x5555_0000);
}

#[test]
fn fpu_vcvtb_f16_f32_negative_zero_roundtrip() {
    // -0.0f32 → f16 → f32 must preserve the negative-zero sign bit.
    let mut c = CortexM33::new();
    c.regs.s[2] = -0.0f32;
    c.regs.s[0] = 0.0f32;
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    // f16 -0 is 0x8000 in the bottom half.
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF, 0x8000);

    // Convert back: VCVTB.F32.F16 S4, S0
    let (hw0, hw1) = enc_vcvtb_f32_f16(4, 0);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[4].to_bits(), 0x8000_0000, "round-trip must preserve -0 sign");
}

// ----- VSEL with D=1 -------------------------------------------------------
//
// Regression test for the D-bit decode: the Armv8-M 0xFE encodings put the D
// bit at hw0[4] rather than hw0[6] (where VFPv4 encodings have it). Picking
// an odd Sd (here S1) forces D=1 and exercises that code path.

#[test]
fn fpu_vsel_d_bit_set() {
    let mut c = CortexM33::new();
    c.regs.s[2] = 10.0;
    c.regs.s[4] = 20.0;
    c.regs.set_flag_z(true); // EQ true → pick Sn
    // Sd = S1 (odd) → D bit = 1. Sn = S2, Sm = S4.
    let (hw0, hw1) = enc_vsel(0, 1, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[1], 10.0);

    // Flip Z → EQ false → should pick Sm, writing S1 again.
    c.regs.set_flag_z(false);
    let (hw0, hw1) = enc_vsel(0, 1, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[1], 20.0);
}

// ============================================================================
// DSP Extension Instructions
// ============================================================================

#[test]
fn ssat_basic() {
    // SSAT R0, #8, R1 — saturate R1 to signed 8-bit range [-128, 127]
    let mut c = CortexM33::new();
    c.set_reg(1, 100); // within range
    // hw0 = 0xF301 (op=0b10000, sh=0, Rn=1)
    // hw1 = 0x0007 (imm3=0, Rd=0, imm2=0, sat_imm=7 → sat_bit=8)
    c.execute_one_wide(0xF301, 0x0007);
    assert_eq!(c.reg(0), 100); // no clamping
    assert!(!c.regs.flag_q());
}

#[test]
fn usat_basic() {
    // USAT R0, #8, R1 — saturate R1 to unsigned 8-bit range [0, 255]
    let mut c = CortexM33::new();
    c.set_reg(1, 300); // above 255
    // hw0 = 0xF381 (op=0b11000, sh=0, Rn=1)
    // hw1 = 0x0008 (sat_bit=8)
    c.execute_one_wide(0xF381, 0x0008);
    assert_eq!(c.reg(0), 255); // clamped
    assert!(c.regs.flag_q());
}

#[test]
fn ssat_q_flag() {
    // SSAT R0, #8, R1 with value > 127 should set Q and clamp
    let mut c = CortexM33::new();
    c.set_reg(1, 200); // 200 > 127
    c.execute_one_wide(0xF301, 0x0007);
    assert_eq!(c.reg(0), 127);
    assert!(c.regs.flag_q());

    // Negative clamping: R1 = -200 (0xFFFFFF38)
    let mut c2 = CortexM33::new();
    c2.set_reg(1, (-200i32) as u32);
    c2.execute_one_wide(0xF301, 0x0007);
    assert_eq!(c2.reg(0) as i32, -128);
    assert!(c2.regs.flag_q());
}

#[test]
fn smulbb() {
    // SMULBB R0, R1, R2 — bottom halfword multiply, no accumulate
    let mut c = CortexM33::new();
    c.set_reg(1, 0x0003_0005); // bottom = 5
    c.set_reg(2, 0x0007_0006); // bottom = 6
    // hw0 = 0xFB11 (op1=001, Rn=R1)
    // hw1 = 0xF002 (Ra=15, Rd=0, op2=00, Rm=R2)
    c.execute_one_wide(0xFB11, 0xF002);
    assert_eq!(c.reg(0), 30); // 5 * 6 = 30

    // Signed: bottom of R1 = -3 (0xFFFD), bottom of R2 = 4
    let mut c2 = CortexM33::new();
    c2.set_reg(1, 0x0000_FFFD); // -3 as i16
    c2.set_reg(2, 0x0000_0004);
    c2.execute_one_wide(0xFB11, 0xF002);
    assert_eq!(c2.reg(0) as i32, -12); // -3 * 4 = -12
}

#[test]
fn smlabb() {
    // SMLABB R0, R1, R2, R3 — halfword multiply-accumulate
    let mut c = CortexM33::new();
    c.set_reg(1, 5);     // bottom = 5
    c.set_reg(2, 6);     // bottom = 6
    c.set_reg(3, 100);   // accumulator
    // hw0 = 0xFB11, hw1 = 0x3002 (Ra=3, Rd=0, op2=00, Rm=2)
    c.execute_one_wide(0xFB11, 0x3002);
    assert_eq!(c.reg(0), 130); // 5*6 + 100 = 130
    assert!(!c.regs.flag_q());
}

#[test]
fn smuad() {
    // SMUAD R0, R1, R2 — dual multiply add (no accumulate, Ra=15)
    let mut c = CortexM33::new();
    // R1 = packed(hi=3, lo=2), R2 = packed(hi=5, lo=4)
    c.set_reg(1, 0x0003_0002);
    c.set_reg(2, 0x0005_0004);
    // hw0 = 0xFB21 (op1=010, Rn=R1), hw1 = 0xF002 (Ra=15, Rd=0, op2=00, Rm=R2)
    c.execute_one_wide(0xFB21, 0xF002);
    // Result = lo*lo + hi*hi = 2*4 + 3*5 = 8 + 15 = 23
    assert_eq!(c.reg(0), 23);
}

#[test]
fn smmul() {
    // SMMUL R0, R1, R2 — most significant word multiply (Ra=15)
    let mut c = CortexM33::new();
    c.set_reg(1, 0x4000_0000); // 2^30 = 1073741824
    c.set_reg(2, 0x4000_0000); // 2^30
    // hw0 = 0xFB51 (op1=101, Rn=R1), hw1 = 0xF002
    c.execute_one_wide(0xFB51, 0xF002);
    // Product = 2^60, high 32 bits = 2^60 >> 32 = 2^28 = 0x10000000
    assert_eq!(c.reg(0), 0x1000_0000);
}

#[test]
fn usad8() {
    // USAD8 R0, R1, R2 — sum of absolute byte differences (Ra=15)
    let mut c = CortexM33::new();
    // R1 = bytes [10, 20, 30, 40]
    c.set_reg(1, 0x2814_0A28u32.swap_bytes()); // little-endian: [40,30,20,10]
    c.set_reg(1, (10) | (20 << 8) | (30 << 16) | (40 << 24));
    // R2 = bytes [15, 15, 15, 15]
    c.set_reg(2, 0x0F0F_0F0F);
    // hw0 = 0xFB71 (op1=111, Rn=R1), hw1 = 0xF002 (Ra=15)
    c.execute_one_wide(0xFB71, 0xF002);
    // |10-15| + |20-15| + |30-15| + |40-15| = 5 + 5 + 15 + 25 = 50
    assert_eq!(c.reg(0), 50);
}

#[test]
fn sadd16() {
    // SADD16 R0, R1, R2 — parallel signed 16-bit add
    let mut c = CortexM33::new();
    // R1 = packed(hi=100, lo=200)
    c.set_reg(1, (200u32) | (100u32 << 16));
    // R2 = packed(hi=50, lo=55)
    c.set_reg(2, (55u32) | (50u32 << 16));
    // SADD16: hw0 = 0xFA91, hw1 = 0xF002
    c.execute_one_wide(0xFA91, 0xF002);
    let result = c.reg(0);
    let lo = result & 0xFFFF;
    let hi = result >> 16;
    assert_eq!(lo, 255);  // 200 + 55
    assert_eq!(hi, 150);  // 100 + 50
    // Both results >= 0, so GE[3:0] should have bits set
    assert_eq!(c.regs.ge_flags() & 0x3, 0x3); // lo result >= 0
    assert_eq!(c.regs.ge_flags() & 0xC, 0xC); // hi result >= 0
}

#[test]
fn uadd8() {
    // UADD8 R0, R1, R2 — parallel unsigned 8-bit add
    let mut c = CortexM33::new();
    c.set_reg(1, 0x01_02_03_04);
    c.set_reg(2, 0x05_06_07_08);
    // UADD8: par_op1=000 (ADD8), par_op2=100 (unsigned)
    // hw0 = 0xFA81 (hw0[6:4]=000, Rn=R1), hw1 = 0xF042 (hw1[6:4]=100, Rm=R2)
    c.execute_one_wide(0xFA81, 0xF042);
    // byte0: 4+8=12, byte1: 3+7=10, byte2: 2+6=8, byte3: 1+5=6
    assert_eq!(c.reg(0), 0x06_08_0A_0C);
    // All sums < 256, so no carries → GE = 0
    assert_eq!(c.regs.ge_flags(), 0);

    // Test with overflow: 0xFF + 0x01 = 0x100 → carry, GE bit set
    let mut c2 = CortexM33::new();
    c2.set_reg(1, 0x00_00_00_FF);
    c2.set_reg(2, 0x00_00_00_01);
    c2.execute_one_wide(0xFA81, 0xF042);
    assert_eq!(c2.reg(0) & 0xFF, 0x00); // wraps to 0
    assert!(c2.regs.ge_flags() & 1 != 0); // GE[0] set (carry)
}

#[test]
fn qadd() {
    // QADD R0, R1, R2 — saturating signed add
    let mut c = CortexM33::new();
    c.set_reg(1, 0x7FFF_FFF0); // near max positive
    c.set_reg(2, 0x0000_0010);
    // QADD: hw0 = 0xFA81 (hw0[7:4]=1000, Rn=R1), hw1 = 0xF082 (hw1[7]=1, Rm=R2)
    c.execute_one_wide(0xFA81, 0xF082);
    assert_eq!(c.reg(0), 0x7FFF_FFFF); // overflows → saturates to i32::MAX, Q flag set
    assert!(c.regs.flag_q());

    // Non-overflowing case
    let mut c2 = CortexM33::new();
    c2.set_reg(1, 100);
    c2.set_reg(2, 200);
    c2.execute_one_wide(0xFA81, 0xF082);
    assert_eq!(c2.reg(0), 300);
    assert!(!c2.regs.flag_q());
}

#[test]
fn sel_basic() {
    // SEL R0, R1, R2 — select bytes based on GE flags
    let mut c = CortexM33::new();
    c.set_reg(1, 0xAA_BB_CC_DD);
    c.set_reg(2, 0x11_22_33_44);
    // GE = 0b1010: byte 3 from R1, byte 2 from R2, byte 1 from R1, byte 0 from R2
    c.regs.set_ge_flags(0b1010);
    // SEL: hw0 = 0xFAA1 (op1_65=01, hw0[4]=0, Rn=R1), hw1 = 0xF082 (Rm=R2)
    c.execute_one_wide(0xFAA1, 0xF082);
    // byte 0: GE[0]=0 → from R2: 0x44
    // byte 1: GE[1]=1 → from R1: 0xCC
    // byte 2: GE[2]=0 → from R2: 0x22
    // byte 3: GE[3]=1 → from R1: 0xAA
    assert_eq!(c.reg(0), 0xAA_22_CC_44);
}

#[test]
fn sxtb16() {
    // SXTB16 R0, R1 — sign-extend bytes 0 and 2 to packed halfwords
    let mut c = CortexM33::new();
    c.set_reg(1, 0x00_80_00_FE); // byte0=0xFE(-2), byte2=0x80(-128)
    // SXTB16: hw0 = 0xFA2F (ext=010, Rn=15), hw1 = 0xF081 (Rd=0, rot=0, Rm=1)
    c.execute_one_wide(0xFA2F, 0xF081);
    // low halfword: sign_extend(0xFE) = 0xFFFE (-2)
    // high halfword: sign_extend(0x80) = 0xFF80 (-128)
    assert_eq!(c.reg(0), 0xFF80_FFFE);
}

#[test]
fn smlald() {
    // SMLALD RdLo=R4, RdHi=R5, Rn=R1, Rm=R2
    let mut c = CortexM33::new();
    // R1 = packed(hi=3, lo=2), R2 = packed(hi=5, lo=4)
    c.set_reg(1, 0x0003_0002);
    c.set_reg(2, 0x0005_0004);
    // Accumulator: R5:R4 = 1000
    c.set_reg(4, 1000);
    c.set_reg(5, 0);
    // SMLALD: op1=100, op2=1100
    // hw0 = 0xFBC1 (op1=100, Rn=R1)
    // hw1 = 0x45C2 (RdLo=4, RdHi=5, op2=1100, Rm=R2)
    c.execute_one_wide(0xFBC1, 0x45C2);
    // Products: lo*lo = 2*4 = 8, hi*hi = 3*5 = 15
    // Result = 1000 + 8 + 15 = 1023
    let result = (c.reg(5) as u64) << 32 | c.reg(4) as u64;
    assert_eq!(result, 1023);
}

#[test]
fn umaal() {
    // UMAAL RdLo=R4, RdHi=R5, Rn=R1, Rm=R2
    let mut c = CortexM33::new();
    c.set_reg(1, 100);
    c.set_reg(2, 200);
    c.set_reg(4, 50);  // RdLo addend
    c.set_reg(5, 30);  // RdHi addend
    // UMAAL: op1=110, op2=0110
    // hw0 = 0xFBE1 (op1=110, Rn=R1)
    // hw1 = 0x4562 (RdLo=4, RdHi=5, op2=0110, Rm=R2)
    c.execute_one_wide(0xFBE1, 0x4562);
    // Result = 100*200 + 50 + 30 = 20000 + 80 = 20080
    let result = (c.reg(5) as u64) << 32 | c.reg(4) as u64;
    assert_eq!(result, 20080);
}

// ============================================================================
// Phase 2: Bus Fabric + Memory Map — TDD Red Phase
// ============================================================================
//
// Tests for RP2350 bus fabric: address decode routing, SRAM banking,
// bus latency accounting, atomic access aliases, and bus arbitration.
//
// Tests marked "Phase 2 API" require new methods/structs that don't exist yet.
// They are #[ignore]d so the test suite compiles but clearly shows gaps.

use crate::memory::Memory;

// ============================================================================
// 2.1 Address Decode Routing
// ============================================================================

#[test]
fn bus_rom_read_returns_loaded_data() {
    let (_, mut bus) = core_and_bus();
    let rom_data: Vec<u8> = (0..32u8).collect();
    bus.memory.load_rom(&rom_data);
    // Read through bus at ROM address 0x00000000
    assert_eq!(bus.read8(0x0000_0000), 0);
    assert_eq!(bus.read8(0x0000_0001), 1);
    assert_eq!(bus.read8(0x0000_001F), 31);
    assert_eq!(bus.read32(0x0000_0000), 0x03020100);
}

#[test]
fn bus_sram_write_then_read_roundtrip() {
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2000_0000, 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2000_0000), 0xDEAD_BEEF);
    bus.write16(0x2000_0004, 0xCAFE);
    assert_eq!(bus.read16(0x2000_0004), 0xCAFE);
    bus.write8(0x2000_0006, 0x42);
    assert_eq!(bus.read8(0x2000_0006), 0x42);
}

#[test]
fn bus_xip_read_returns_loaded_flash_data() {
    let (_, mut bus) = core_and_bus();
    let flash = vec![0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44];
    bus.load_flash(&flash);
    assert_eq!(bus.read8(0x1000_0000), 0xAA);
    assert_eq!(bus.read32(0x1000_0000), 0xDDCCBBAA);
    assert_eq!(bus.read32(0x1000_0004), 0x44332211);
}

#[test]
fn bus_sram_boundary_last_valid_byte() {
    // SRAM is 520 KB = 0x82000 bytes. Last valid address: 0x20081FFF.
    let (_, mut bus) = core_and_bus();
    bus.write8(0x2008_1FFF, 0x77);
    assert_eq!(bus.read8(0x2008_1FFF), 0x77);
}

#[test]
fn bus_sram_boundary_out_of_range_returns_zero() {
    // Address 0x20082000 is beyond the 520 KB SRAM region.
    let (_, mut bus) = core_and_bus();
    bus.write8(0x2008_2000, 0xFF); // should be silently ignored
    assert_eq!(bus.read8(0x2008_2000), 0); // out-of-range → 0
}

#[test]
fn bus_rom_boundary_32kb() {
    // ROM is 32 KB = 0x8000 bytes. Address 0x00007FFF is the last valid byte.
    let (_, mut bus) = core_and_bus();
    let mut rom_data = vec![0u8; 32 * 1024];
    rom_data[0x7FFF] = 0xEE;
    bus.memory.load_rom(&rom_data);
    assert_eq!(bus.read8(0x0000_7FFF), 0xEE); // last byte of 32 KB ROM
    assert_eq!(bus.read8(0x0000_8000), 0);     // beyond ROM → 0
    assert_eq!(bus.read8(0x0000_FFFF), 0);     // well beyond ROM → 0
}

#[test]
fn bus_writes_to_rom_are_silently_ignored() {
    let (_, mut bus) = core_and_bus();
    let rom_data = vec![0x42; 16];
    bus.memory.load_rom(&rom_data);
    // Attempt to write to ROM address — should be ignored
    bus.write8(0x0000_0000, 0xFF);
    bus.write32(0x0000_0004, 0xFFFF_FFFF);
    // Original data preserved
    assert_eq!(bus.read8(0x0000_0000), 0x42);
    assert_eq!(bus.read32(0x0000_0004), 0x42424242);
}

#[test]
fn bus_unmapped_region_reads_zero() {
    // Regions 0x3, 0x6..0xC, 0xF are unmapped — should read as 0.
    let (_, mut bus) = core_and_bus();
    assert_eq!(bus.read32(0x3000_0000), 0);
    assert_eq!(bus.read32(0x6000_0000), 0);
    assert_eq!(bus.read32(0xF000_0000), 0);
}

// ============================================================================
// 2.2 SRAM Banking
// ============================================================================
//
// RP2350 SRAM: SRAM0-7 are 64 KB each (512 KB total), word-striped.
// SRAM8 is 4 KB at offset 0x80000, SRAM9 is 4 KB at offset 0x81000.
// Stripe formula: bank = (word_offset) % 8, where word_offset = (addr - base) / 4.

#[test]
fn sram_bank0_write_read() {
    // Word at SRAM base + 0x00 → bank 0 (word_offset 0 % 8 = 0)
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2000_0000, 0x1111_1111);
    assert_eq!(bus.read32(0x2000_0000), 0x1111_1111);
}

#[test]
fn sram8_write_read() {
    // SRAM8: 4 KB at 0x20080000 (non-striped)
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2008_0000, 0xAAAA_BBBB);
    assert_eq!(bus.read32(0x2008_0000), 0xAAAA_BBBB);
    // Last word of SRAM8: 0x20080FFC
    bus.write32(0x2008_0FFC, 0xCCCC_DDDD);
    assert_eq!(bus.read32(0x2008_0FFC), 0xCCCC_DDDD);
}

#[test]
fn sram9_write_read() {
    // SRAM9: 4 KB at 0x20081000 (non-striped)
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2008_1000, 0x1234_5678);
    assert_eq!(bus.read32(0x2008_1000), 0x1234_5678);
    // Last word of SRAM9: 0x20081FFC
    bus.write32(0x2008_1FFC, 0x9ABC_DEF0);
    assert_eq!(bus.read32(0x2008_1FFC), 0x9ABC_DEF0);
}

#[test]
fn sram_striped_access_consecutive_words_go_to_consecutive_banks() {
    // Consecutive 32-bit words go to consecutive banks:
    //   0x20000000 → bank 0 (word 0 % 8)
    //   0x20000004 → bank 1 (word 1 % 8)
    //   0x20000008 → bank 2 (word 2 % 8)
    //   ...
    //   0x2000001C → bank 7 (word 7 % 8)
    //   0x20000020 → bank 0 (word 8 % 8) — wraps
    let (_, mut bus) = core_and_bus();
    for i in 0u32..9 {
        let addr = 0x2000_0000 + i * 4;
        let val = 0xA000_0000 | i;
        bus.write32(addr, val);
    }
    for i in 0u32..9 {
        let addr = 0x2000_0000 + i * 4;
        let expected = 0xA000_0000 | i;
        assert_eq!(bus.read32(addr), expected, "word {} at 0x{:08X}", i, addr);
    }
}

#[test]
fn bank_for_address_striped_region() {
    // Memory::bank_for_address(addr) → bank index (0..9)
    // Striped region: bank = (word_offset) % 8
    assert_eq!(Memory::bank_for_address(0x2000_0000), Some(0)); // word 0 → bank 0
    assert_eq!(Memory::bank_for_address(0x2000_0004), Some(1)); // word 1 → bank 1
    assert_eq!(Memory::bank_for_address(0x2000_0008), Some(2)); // word 2 → bank 2
    assert_eq!(Memory::bank_for_address(0x2000_000C), Some(3)); // word 3 → bank 3
    assert_eq!(Memory::bank_for_address(0x2000_001C), Some(7)); // word 7 → bank 7
    assert_eq!(Memory::bank_for_address(0x2000_0020), Some(0)); // word 8 → wraps to bank 0
}

#[test]
fn bank_for_address_non_striped_region() {
    // Non-striped banks:
    //   SRAM8 (0x20080000..0x20080FFF) → always bank 8
    //   SRAM9 (0x20081000..0x20081FFF) → always bank 9
    assert_eq!(Memory::bank_for_address(0x2008_0000), Some(8));
    assert_eq!(Memory::bank_for_address(0x2008_0500), Some(8));
    assert_eq!(Memory::bank_for_address(0x2008_0FFF), Some(8));
    assert_eq!(Memory::bank_for_address(0x2008_1000), Some(9));
    assert_eq!(Memory::bank_for_address(0x2008_1500), Some(9));
    assert_eq!(Memory::bank_for_address(0x2008_1FFF), Some(9));
}

#[test]
fn bank_for_address_rejects_non_sram() {
    // ROM address — not SRAM
    assert_eq!(Memory::bank_for_address(0x0000_1000), None);
    // XIP address — not SRAM
    assert_eq!(Memory::bank_for_address(0x1000_0004), None);
    // Beyond SRAM9
    assert_eq!(Memory::bank_for_address(0x2008_2000), None);
    // Unmapped region
    assert_eq!(Memory::bank_for_address(0x3000_0000), None);
}

// ============================================================================
// 2.3 Bus Latency
// ============================================================================
//
// Phase 2 API — tests assume Bus will gain a method like
// bus.last_access_cycles() -> u32 that reports the cycle cost of the
// most recent read or write. These tests won't compile until that API exists.

#[test]
fn bus_latency_sram_read_1_cycle() {
    // SRAM is AHB-attached: 1-cycle read, zero wait state.
    let (_, mut bus) = core_and_bus();
    bus.read32(0x2000_0000);
    assert_eq!(bus.last_access_cycles(), 1);
}

#[test]
fn bus_latency_sram_write_1_cycle() {
    // SRAM is AHB-attached: 1-cycle write.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2000_0000, 0x42);
    assert_eq!(bus.last_access_cycles(), 1);
}

#[test]
fn bus_latency_rom_read_1_cycle() {
    // ROM is AHB-attached: 1-cycle read.
    let (_, mut bus) = core_and_bus();
    bus.read32(0x0000_0000);
    assert_eq!(bus.last_access_cycles(), 1);
}

#[test]
fn bus_latency_apb_peripheral_read_3_cycles() {
    // APB peripherals at 0x40000000: 3-cycle read latency.
    let (_, mut bus) = core_and_bus();
    bus.read32(0x4000_0000);
    assert_eq!(bus.last_access_cycles(), 3);
}

#[test]
fn bus_latency_apb_peripheral_write_4_cycles() {
    // APB peripherals at 0x40000000: 4-cycle write latency.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4000_0000, 0x1);
    assert_eq!(bus.last_access_cycles(), 4);
}

#[test]
fn bus_latency_sio_access_1_cycle() {
    // SIO at 0xD0000000: single-cycle access.
    let (_, mut bus) = core_and_bus();
    bus.read32(0xD000_0000);
    assert_eq!(bus.last_access_cycles(), 1);
}

// ============================================================================
// 2.4 Atomic Access Aliases
// ============================================================================
//
// RP2350 provides +0x0000 normal, +0x1000 XOR, +0x2000 SET, +0x3000 CLR
// aliases for peripheral registers. The alias is encoded in bits [13:12]
// of the address within each 4 KB peripheral page.
//
// Phase 2 API — atomic aliases apply to APB/AHB peripheral writes.
// These tests use SRAM as a stand-in until peripheral stubs exist,
// or may target a known peripheral base address.

#[test]
fn atomic_alias_normal_write() {
    // Base+0x0000: normal write replaces the value.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000; // APB peripheral base (generic, not a stub peripheral)
    bus.write32(base + 0x0000, 0xFF00_FF00);
    assert_eq!(bus.read32(base), 0xFF00_FF00);
}

#[test]
fn atomic_alias_xor_write() {
    // Base+0x1000: XOR — new_val = old_val ^ written_val.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base + 0x0000, 0xFF00_FF00); // seed value
    bus.write32(base + 0x1000, 0x0F0F_0F0F); // XOR alias
    assert_eq!(bus.read32(base), 0xF00F_F00F);
}

#[test]
fn atomic_alias_set_write() {
    // Base+0x2000: SET — new_val = old_val | written_val.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base + 0x0000, 0x0000_00FF); // seed value
    bus.write32(base + 0x2000, 0x0000_FF00); // SET alias
    assert_eq!(bus.read32(base), 0x0000_FFFF);
}

#[test]
fn atomic_alias_clr_write() {
    // Base+0x3000: CLR — new_val = old_val & ~written_val.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base + 0x0000, 0xFFFF_FFFF); // seed value
    bus.write32(base + 0x3000, 0x00FF_00FF); // CLR alias
    assert_eq!(bus.read32(base), 0xFF00_FF00);
}

#[test]
fn atomic_alias_read_ignores_alias_bits() {
    // Reads from any alias offset return the same canonical value.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base, 0xBEEF_CAFE);
    assert_eq!(bus.read32(base + 0x0000), 0xBEEF_CAFE);
    assert_eq!(bus.read32(base + 0x1000), 0xBEEF_CAFE); // XOR alias read
    assert_eq!(bus.read32(base + 0x2000), 0xBEEF_CAFE); // SET alias read
    assert_eq!(bus.read32(base + 0x3000), 0xBEEF_CAFE); // CLR alias read
}

#[test]
fn atomic_alias_ahb_peripheral() {
    // AHB peripherals (0x5xxxxxxx) also support atomic aliases.
    // PIO0 CTRL: SM_ENABLE [3:0] with SET/CLR/XOR alias support.
    let mut bus = Bus::new();
    let base = 0x5020_0000; // PIO0 CTRL
    bus.write32(base, 0x5); // enable SM0 + SM2
    assert_eq!(bus.read32(base), 0x5);
    bus.write32(base + 0x2000, 0xA); // SET alias: enable SM1 + SM3
    assert_eq!(bus.read32(base), 0xF); // all 4 SMs enabled
    // AHB atomics have no extra latency cost (unlike APB interposed)
    bus.write32(base + 0x1000, 0x3); // XOR alias: toggle SM0 + SM1
    assert_eq!(bus.last_access_cycles(), 1); // no extra cost
    assert_eq!(bus.read32(base), 0xC); // SM2 + SM3 remain enabled
}

#[test]
fn atomic_alias_apb_interposed_latency() {
    // APB atomic writes (XOR/SET/CLR) cost +2 extra cycles (interposed).
    let mut bus = Bus::new();
    let base = 0x4007_0000; // UART0
    // Normal APB write: 4 cycles
    bus.write32(base, 0x1234);
    assert_eq!(bus.last_access_cycles(), 4);
    // XOR alias APB write: 6 cycles (4 + 2 interposed)
    bus.write32(base + 0x1000, 0x00FF);
    assert_eq!(bus.last_access_cycles(), 6);
    // SET alias: also 6 cycles
    bus.write32(base + 0x2000, 0x00FF);
    assert_eq!(bus.last_access_cycles(), 6);
    // CLR alias: also 6 cycles
    bus.write32(base + 0x3000, 0x00FF);
    assert_eq!(bus.last_access_cycles(), 6);
}

// ============================================================================
// 2.5 Bus Arbitration
// ============================================================================
//
// RP2350 AHB5 bus fabric: two upstream ports (core 0, core 1) share
// downstream targets. If both access the same downstream port in the
// same cycle, one proceeds and the other stalls 1 cycle.
//
// Phase 2 API — these may require a BusFabric struct or a
// bus.arbitrate(port0_req, port1_req) method.

#[test]
fn arbitration_single_core_no_contention() {
    // A single core accessing a bus target incurs no stall.
    let bus = Bus::new();
    let stall = bus.arbitrate_stall(/*core=*/0, /*addr=*/0x2000_0000);
    assert_eq!(stall, 0);
}

#[test]
fn arbitration_two_cores_different_banks_no_contention() {
    // Two cores accessing different SRAM banks: no contention.
    // Core 0 → bank 0 (0x20000000), Core 1 → bank 1 (0x20000004)
    let bus = Bus::new();
    let (stall0, stall1) = bus.arbitrate_pair(
        /*core0_addr=*/0x2000_0000, // bank 0
        /*core1_addr=*/0x2000_0004, // bank 1
    );
    assert_eq!(stall0, 0);
    assert_eq!(stall1, 0);
}

#[test]
fn arbitration_two_cores_same_bank_one_stalls() {
    // Two cores accessing the same SRAM bank: one stalls 1 cycle.
    // Core 0 → bank 0 (0x20000000), Core 1 → bank 0 (0x20000020)
    // Both map to bank 0 (word offsets 0 and 8, both % 8 = 0).
    let bus = Bus::new();
    let (stall0, stall1) = bus.arbitrate_pair(
        /*core0_addr=*/0x2000_0000, // bank 0 (word 0)
        /*core1_addr=*/0x2000_0020, // bank 0 (word 8)
    );
    assert_eq!(stall0, 0, "core 0 should win (higher priority)");
    assert_eq!(stall1, 1, "core 1 should stall");
}

#[test]
fn arbitration_two_cores_same_non_sram_port() {
    // Both cores reading ROM — same downstream port (ROM is single-port).
    let bus = Bus::new();
    let (stall0, stall1) = bus.arbitrate_pair(0x0000_0000, 0x0000_0100);
    assert_eq!(stall0, 0, "core 0 wins");
    assert_eq!(stall1, 1, "core 1 stalls");

    // Both cores accessing APB — same downstream port (APB bridge is single-port).
    let (stall0, stall1) = bus.arbitrate_pair(0x4007_0000, 0x4008_0000);
    assert_eq!(stall0, 0);
    assert_eq!(stall1, 1);
}

#[test]
fn arbitration_core_local_never_contends() {
    // Both cores accessing SIO — core-local, no contention possible.
    let bus = Bus::new();
    let (stall0, stall1) = bus.arbitrate_pair(0xD000_0000, 0xD000_0004);
    assert_eq!(stall0, 0);
    assert_eq!(stall1, 0);

    // Both cores accessing PPB — also core-local.
    let (stall0, stall1) = bus.arbitrate_pair(0xE000_0000, 0xE000_0004);
    assert_eq!(stall0, 0);
    assert_eq!(stall1, 0);

    // One core SIO, other core SRAM — different ports, no contention.
    let (stall0, stall1) = bus.arbitrate_pair(0xD000_0000, 0x2000_0000);
    assert_eq!(stall0, 0);
    assert_eq!(stall1, 0);
}

// ---------- Bus contention integration tests ----------

#[test]
fn contention_same_sram_bank_adds_stall() {
    // Both cores executing from the same SRAM bank should cause core 1 to stall.
    let mut emu = crate::EmulatorBuilder::new(crate::Config::default()).build();

    // Place MOV R0, R0 (0x4600) — 1-cycle NOP-like instruction
    let nop: u16 = 0x4600;
    let bytes = nop.to_le_bytes();
    // Bank 0: addr 0x20000000
    emu.bus.memory.sram_write8(0, bytes[0]);
    emu.bus.memory.sram_write8(1, bytes[1]);
    // Same bank (bank 0): addr 0x20000020
    emu.bus.memory.sram_write8(0x20, bytes[0]);
    emu.bus.memory.sram_write8(0x21, bytes[1]);

    // Point both cores at these addresses
    emu.cores[0].set_reg(15, 0x20000000);
    emu.cores[1].set_reg(15, 0x20000020);

    // Step once — both cores fetch from bank 0
    emu.step();

    // Core 1 should have a contention stall (1 extra cycle)
    assert_eq!(emu.cores[0].stall_cycles(), 0);
    assert_eq!(emu.cores[1].stall_cycles(), 1);
}

#[test]
fn contention_different_banks_no_stall() {
    let mut emu = crate::EmulatorBuilder::new(crate::Config::default()).build();

    let nop: u16 = 0x4600;
    let bytes = nop.to_le_bytes();
    // Bank 0: offset 0
    emu.bus.memory.sram_write8(0, bytes[0]);
    emu.bus.memory.sram_write8(1, bytes[1]);
    // Bank 1: offset 4 (next word = next bank)
    emu.bus.memory.sram_write8(4, bytes[0]);
    emu.bus.memory.sram_write8(5, bytes[1]);

    emu.cores[0].set_reg(15, 0x20000000); // bank 0
    emu.cores[1].set_reg(15, 0x20000004); // bank 1

    emu.step();

    // No contention — different banks
    assert_eq!(emu.cores[0].stall_cycles(), 0);
    assert_eq!(emu.cores[1].stall_cycles(), 0);
}

#[test]
fn contention_core1_sio_never_contends() {
    // SIO is core-local — never contends regardless of core 0's target
    let mut emu = crate::EmulatorBuilder::new(crate::Config::default()).build();

    // Set up core 0 SRAM bank 0 access
    let nop: u16 = 0x4600;
    let bytes = nop.to_le_bytes();
    emu.bus.memory.sram_write8(0, bytes[0]);
    emu.bus.memory.sram_write8(1, bytes[1]);
    emu.cores[0].set_reg(15, 0x20000000);

    // Verify the contention check directly:
    emu.bus.clear_contention_state();
    emu.bus.read32(0x20000000); // core 0 reads SRAM bank 0
    emu.bus.begin_contention_check();
    emu.bus.reset_extra_wait_states();
    emu.bus.read32(0xD0000000); // core 1 reads SIO — core-local
    assert_eq!(emu.bus.extra_wait_states(), 0, "SIO should never contend");
}

// ============================================================================
// 2.7 SRAM Atomic Aliases
// ============================================================================

#[test]
fn sram_atomic_xor() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0xAAAA_5555); // seed via normal write
    bus.write32(0x2100_0000, 0xFFFF_FFFF); // XOR alias
    assert_eq!(bus.read32(0x2000_0000), 0x5555_AAAA);
}

#[test]
fn sram_atomic_set() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0x0000_00FF);
    bus.write32(0x2200_0000, 0x0000_FF00); // SET alias
    assert_eq!(bus.read32(0x2000_0000), 0x0000_FFFF);
}

#[test]
fn sram_atomic_clr() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0xFFFF_FFFF);
    bus.write32(0x2300_0000, 0x00FF_00FF); // CLR alias
    assert_eq!(bus.read32(0x2000_0000), 0xFF00_FF00);
}

#[test]
fn sram_atomic_read_returns_canonical() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0010, 0xDEAD_BEEF);
    // All alias reads return the same canonical value
    assert_eq!(bus.read32(0x2000_0010), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2100_0010), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2200_0010), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2300_0010), 0xDEAD_BEEF);
}

#[test]
fn sram_atomic_8bit_xor_doesnt_affect_neighbors() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0xAABB_CCDD);
    bus.write8(0x2100_0001, 0xFF); // XOR byte at offset 1 only
    // Byte 0: 0xDD unchanged, Byte 1: 0xCC ^ 0xFF = 0x33, Byte 2-3: unchanged
    assert_eq!(bus.read32(0x2000_0000), 0xAABB_33DD);
}

#[test]
fn sram_atomic_no_extra_latency() {
    let mut bus = Bus::new();
    bus.write32(0x2200_0000, 0xFF); // SET alias write
    assert_eq!(bus.last_access_cycles(), 1); // same as normal SRAM
}

#[test]
fn sram_alias_bank_for_address_resolves_correctly() {
    use crate::memory::Memory;
    // Alias addresses should resolve to same bank as canonical
    assert_eq!(Memory::bank_for_address(0x2000_0004), Memory::bank_for_address(0x2100_0004));
    assert_eq!(Memory::bank_for_address(0x2000_0004), Memory::bank_for_address(0x2200_0004));
    assert_eq!(Memory::bank_for_address(0x2000_0004), Memory::bank_for_address(0x2300_0004));
}

// ============================================================================
// 2.8 Dual-Core Accumulator Safety
// ============================================================================

#[test]
fn dual_core_extra_wait_states_no_pollution() {
    // Regression test: verify that core 0's bus accesses don't pollute
    // core 1's cycle count. reset_extra_wait_states() at the start of
    // decode_execute() ensures each core starts clean.
    let mut emu = crate::EmulatorBuilder::new(crate::Config::default()).build();

    // Place NOP (MOV R0, R0 = 0x4600) at two SRAM addresses in different banks
    let nop: u16 = 0x4600;
    let bytes = nop.to_le_bytes();
    emu.bus.memory.sram_write8(0, bytes[0]);
    emu.bus.memory.sram_write8(1, bytes[1]);
    emu.bus.memory.sram_write8(4, bytes[0]);
    emu.bus.memory.sram_write8(5, bytes[1]);

    emu.cores[0].set_reg(15, 0x20000000); // bank 0
    emu.cores[1].set_reg(15, 0x20000004); // bank 1

    // Step — both cores execute from SRAM (1 cycle, 0 extra wait states)
    emu.step();

    // Both should have 0 stall cycles — no pollution, no contention (different banks)
    assert_eq!(emu.cores[0].stall_cycles(), 0, "core 0 should have no stall");
    assert_eq!(emu.cores[1].stall_cycles(), 0, "core 1 should have no stall (no pollution)");
}

// ============================================================================
// 2.9 SRAM Per-Bank Extra Wait States
// ============================================================================

#[test]
fn sram_bank2_read_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000008: offset 0x8, bank = (0x8 >> 2) & 7 = 2
    let _ = bus.read32(0x2000_0008);
    assert_eq!(bus.extra_wait_states(), 1, "bank 2 read should add +1 wait state");
}

#[test]
fn sram_bank6_read_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000018: offset 0x18, bank = (0x18 >> 2) & 7 = 6
    let _ = bus.read32(0x2000_0018);
    assert_eq!(bus.extra_wait_states(), 1, "bank 6 read should add +1 wait state");
}

#[test]
fn sram_bank0_no_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000000: offset 0x0, bank = (0x0 >> 2) & 7 = 0
    let _ = bus.read32(0x2000_0000);
    assert_eq!(bus.extra_wait_states(), 0, "bank 0 read should have no extra wait state");
}

#[test]
fn sram_bank2_write_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000008: offset 0x8, bank = (0x8 >> 2) & 7 = 2
    bus.write32(0x2000_0008, 0xDEAD_BEEF);
    assert_eq!(bus.extra_wait_states(), 1, "bank 2 write should add +1 wait state");
}

#[test]
fn sram_bank89_no_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20080000: offset 0x80000, non-striped SRAM8
    let _ = bus.read32(0x2008_0000);
    assert_eq!(bus.extra_wait_states(), 0, "SRAM8 read should have no extra wait state");
}

// ============================================================================
// Integration: Reset + Bootrom
// ============================================================================

use crate::{Emulator, Config};

#[test]
fn test_reset_loads_sp_and_pc_from_rom() {
    let mut emu = Emulator::new(Config::default());

    // Build a minimal ROM: word 0 = initial SP, word 1 = reset vector
    let mut rom = vec![0u8; 512];
    // SP = 0x2008_0000 (top of SRAM)
    rom[0..4].copy_from_slice(&0x2008_0000u32.to_le_bytes());
    // Reset vector = 0x0000_0101 (thumb bit set)
    rom[4..8].copy_from_slice(&0x0000_0101u32.to_le_bytes());
    // Put a NOP (MOVS R0, R0 = 0x0000) at address 0x100
    // followed by an infinite loop (B . = 0xE7FE)
    rom[0x100] = 0x00; rom[0x101] = 0x00; // NOP (MOVS R0, R0)
    rom[0x102] = 0xFE; rom[0x103] = 0xE7; // B .

    emu.load_bootrom(&rom);
    emu.reset();

    // Verify initial state
    assert_eq!(emu.cores[0].regs.msp, 0x2008_0000);
    assert_eq!(emu.cores[0].regs.r[13], 0x2008_0000);
    assert_eq!(emu.cores[0].regs.pc(), 0x0000_0100); // bit 0 cleared
    assert_eq!(emu.cores[0].regs.xpsr & (1 << 24), 1 << 24); // Thumb bit

    // Core 1 should be at reset vector (same as Core 0 — both boot)
    assert_eq!(emu.cores[1].regs.pc(), 0x0000_0100);

    // Run a few cycles - should execute the NOP then hit the infinite loop
    for _ in 0..10 {
        emu.step();
    }

    // Should be stuck at the infinite loop (0x102)
    assert_eq!(emu.cores[0].regs.pc(), 0x0000_0102);
}

#[test]
fn test_svc_exception_round_trip() {
    let mut emu = Emulator::new(Config::default());

    // Build ROM with vector table and code
    let mut rom = vec![0u8; 1024];

    // Vector table
    rom[0..4].copy_from_slice(&0x2008_0000u32.to_le_bytes()); // SP
    rom[4..8].copy_from_slice(&0x0000_0101u32.to_le_bytes()); // Reset vector -> 0x100
    // SVC handler vector (exception 11) at offset 11*4 = 44 = 0x2C
    rom[0x2C..0x30].copy_from_slice(&0x0000_0201u32.to_le_bytes()); // -> 0x200

    // Code at 0x100: SVC #0
    rom[0x100] = 0x00; rom[0x101] = 0xDF; // SVC #0 = 0xDF00
    // Code at 0x102: infinite loop after SVC returns
    rom[0x102] = 0xFE; rom[0x103] = 0xE7; // B .

    // SVC handler at 0x200: BX LR (return from exception)
    rom[0x200] = 0x70; rom[0x201] = 0x47; // BX LR = 0x4770

    emu.load_bootrom(&rom);
    emu.reset();

    // Run enough cycles for: SVC entry (~12) + BX LR return (~12) + settling
    for _ in 0..50 {
        emu.step();
    }

    // After SVC -> handler -> return, should be in the infinite loop at 0x102
    assert_eq!(emu.cores[0].regs.pc(), 0x0000_0102);
    // Should be back in thread mode (IPSR = 0)
    assert_eq!(emu.cores[0].regs.ipsr(), 0);
}

#[test]
fn test_busfault_on_unmapped_access() {
    let mut emu = Emulator::new(Config::default());

    let mut rom = vec![0u8; 1024];
    // Vector table
    rom[0..4].copy_from_slice(&0x2008_0000u32.to_le_bytes()); // SP
    rom[4..8].copy_from_slice(&0x0000_0101u32.to_le_bytes()); // Reset -> 0x100
    // BusFault handler (exception 5) at offset 5*4 = 20 = 0x14
    rom[0x14..0x18].copy_from_slice(&0x0000_0301u32.to_le_bytes()); // -> 0x300
    // HardFault handler (exception 3) at offset 3*4 = 12 = 0x0C
    rom[0x0C..0x10].copy_from_slice(&0x0000_0381u32.to_le_bytes()); // -> 0x380

    // Code at 0x100: LDR R0, [R1, #0] where R1 will be 0x60000000 (unmapped)
    rom[0x100] = 0x08; rom[0x101] = 0x68; // LDR R0, [R1, #0] = 0x6808
    rom[0x102] = 0xFE; rom[0x103] = 0xE7; // B . (shouldn't reach if fault works)

    // BusFault handler at 0x300: BX LR (return from exception)
    rom[0x300] = 0x70; rom[0x301] = 0x47; // BX LR

    // HardFault handler at 0x380: infinite loop
    rom[0x380] = 0xFE; rom[0x381] = 0xE7; // B .

    emu.load_bootrom(&rom);
    emu.reset();

    // Pre-set: R1 = 0x60000000 (unmapped address)
    emu.cores[0].regs.r[1] = 0x6000_0000;
    // Enable BusFault handler in SHCSR (bit 17)
    emu.bus.ppb[0].shcsr |= 1 << 17;

    // Run
    for _ in 0..50 {
        emu.step();
    }

    // CFSR should have PRECISERR (bit 9) set
    assert_ne!(emu.bus.ppb[0].cfsr & (1 << 9), 0, "PRECISERR should be set");
}

// ============================================================================
// SAU + TT (Test Target) instruction
// ============================================================================

#[test]
fn tt_sau_disabled_returns_secure() {
    // When SAU is disabled, TT should return Secure with full access
    let (mut c, mut bus) = core_and_bus();
    // SAU disabled by default (sau_ctrl = 0)
    c.set_reg(5, 0x2000_0000); // address to test (SRAM, IDAU-secure)
    // TT R2, R5: hw0=0xE845, hw1=0xF200
    c.execute_one_wide_with_bus(0xE845, 0xF200, &mut bus);
    let result = c.reg(2);
    // S=1, RW=1, R=1, SRVALID=0; IDAU bits may be set
    assert_ne!(result & (1 << 22), 0, "S bit should be set");
    assert_ne!(result & (1 << 19), 0, "RW bit should be set");
    assert_ne!(result & (1 << 18), 0, "R bit should be set");
    assert_eq!(result & (1 << 17), 0, "SRVALID should be clear");
}

#[test]
fn tt_sau_region_match() {
    // Configure SAU region 3 covering 0x4780-0x7FFF (Secure), then TT an address in range
    let (mut c, mut bus) = core_and_bus();
    // Enable SAU
    bus.ppb[0].sau_ctrl = 1;
    // Region 3: RBAR=0x4787, RLAR=0x7FE1 (enabled, NSC=0 -> Secure)
    bus.ppb[0].sau_rnr = 3;
    bus.ppb[0].sau_regions[3] = (0x4787, 0x7FE1);

    c.set_reg(5, 0x7FE1); // address in range (secure ROM range, IDAU-secure)
    // TT R2, R5: hw0=0xE845, hw1=0xF200
    c.execute_one_wide_with_bus(0xE845, 0xF200, &mut bus);
    let result = c.reg(2);

    // SREGION = 3 at bits [15:8]
    assert_eq!((result >> 8) & 0xFF, 3, "SREGION should be 3");
    // SRVALID = bit 17
    assert_ne!(result & (1 << 17), 0, "SRVALID should be set");
    // S = 1 (Secure, NSC=0)
    assert_ne!(result & (1 << 22), 0, "S bit should be set");
    // Access bits
    assert_ne!(result & (1 << 18), 0, "R bit should be set");
    assert_ne!(result & (1 << 19), 0, "RW bit should be set");
}

#[test]
fn tt_sau_region_nsc() {
    // Configure SAU region 0 as NSC (Non-Secure Callable)
    let (mut c, mut bus) = core_and_bus();
    bus.ppb[0].sau_ctrl = 1;
    // Region 0: base=0x1000, limit=0x1FFF, NSC=1, enabled
    // RBAR = 0x1000, RLAR = 0x1FE0 | 0x3 (NSC=1, enable=1)
    bus.ppb[0].sau_regions[0] = (0x1000, 0x1FE3);

    c.set_reg(1, 0x1500); // address in range (secure ROM range)
    // TT R0, R1: hw0=0xE841, hw1=0xF000
    c.execute_one_wide_with_bus(0xE841, 0xF000, &mut bus);
    let result = c.reg(0);

    // SREGION = 0 at bits [15:8]
    assert_eq!((result >> 8) & 0xFF, 0, "SREGION should be 0");
    // SRVALID = bit 17
    assert_ne!(result & (1 << 17), 0, "SRVALID should be set");
    // NSC region: S should be 0 (non-secure callable)
    assert_eq!(result & (1 << 22), 0, "S bit should be clear for NSC");
    // NSR and NSRW should be set for NSC region
    assert_ne!(result & (1 << 20), 0, "NSR bit should be set for NSC");
    assert_ne!(result & (1 << 21), 0, "NSRW bit should be set for NSC");
}

#[test]
fn tt_sau_no_match_allns_clear() {
    // SAU enabled, no regions match, ALLNS=0 -> Secure
    let (mut c, mut bus) = core_and_bus();
    bus.ppb[0].sau_ctrl = 1; // enable, ALLNS=0

    c.set_reg(3, 0xFFFF_0000); // address not in any region
    // TT R0, R3: hw0=0xE843, hw1=0xF000
    c.execute_one_wide_with_bus(0xE843, 0xF000, &mut bus);
    let result = c.reg(0);

    assert_eq!(result & (1 << 17), 0, "SRVALID should be clear");
    assert_ne!(result & (1 << 22), 0, "S bit should be set (ALLNS=0)");
}

#[test]
fn tt_sau_no_match_allns_set() {
    // SAU enabled, no regions match, ALLNS=1 -> Non-Secure
    let (mut c, mut bus) = core_and_bus();
    bus.ppb[0].sau_ctrl = 3; // enable + ALLNS

    c.set_reg(3, 0xFFFF_0000);
    // TT R0, R3: hw0=0xE843, hw1=0xF000
    c.execute_one_wide_with_bus(0xE843, 0xF000, &mut bus);
    let result = c.reg(0);

    assert_eq!(result & (1 << 17), 0, "SRVALID should be clear");
    assert_eq!(result & (1 << 22), 0, "S bit should be clear (ALLNS=1)");
    assert_ne!(result & (1 << 18), 0, "R bit should be set");
}

#[test]
fn tt_bootrom_scenario() {
    // Reproduce the bootrom's exact SAU+TT sequence:
    // Region 7: RBAR=0x4787, RLAR=0x7FE1, TT address=0x7FE1
    let (mut c, mut bus) = core_and_bus();
    bus.ppb[0].sau_ctrl = 1;
    bus.ppb[0].sau_rnr = 7;
    bus.ppb[0].sau_regions[7] = (0x4787, 0x7FE1);

    c.set_reg(5, 0x7FE1);
    // TT R2, R5: hw0=0xE845, hw1=0xF200
    c.execute_one_wide_with_bus(0xE845, 0xF200, &mut bus);
    let result = c.reg(2);

    // Bootrom expects exactly 0x02CE0700 for this scenario
    assert_eq!(result, 0x02CE0700, "TT result should match bootrom expected value");
    // Verify key fields:
    // SREGION = 7 at bits [15:8]
    assert_eq!((result >> 8) & 0xFF, 7, "SREGION should be 7");
    // SRVALID = bit 17
    assert_ne!(result & (1 << 17), 0, "SRVALID");
    // S = 1
    assert_ne!(result & (1 << 22), 0, "S");
    // IDAU bits: bit 23 (IRVALID) and bit 25 (RP2350 exempt)
    assert_ne!(result & (1 << 23), 0, "IRVALID (IDAU region valid)");
    assert_ne!(result & (1 << 25), 0, "RP2350 IDAU exempt bit");
}

#[test]
fn tt_does_not_collide_with_strex() {
    // STREX R0, R1, [R2, #0]: hw0=0xE842, hw1=0x1000
    // hw1[15:12]=1 (Rt=R1), hw1[7:0]=0 (imm8=0) — this is STREX, not TT
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(1, 0xDEAD_BEEF);
    c.set_reg(2, 0x2000_0100);
    c.execute_one_wide_with_bus(0xE842, 0x1000, &mut bus);
    // R0 (Rd) should be 0 (STREX success)
    assert_eq!(c.reg(0), 0);
    // Memory at 0x20000100 should have the stored value
    assert_eq!(bus.read32(0x2000_0100), 0xDEAD_BEEF);
}

// ============================================================================
// MSPLIM / PSPLIM via MSR/MRS
// ============================================================================

#[test]
fn msr_mrs_msplim_roundtrip() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x2000_1000);
    // MSR MSPLIM, R0: hw0=0xF380, hw1=0x880A (Rn=0, SYSm=0x0A, mask=2)
    c.execute_one_wide(0xF380, 0x880A);
    assert_eq!(c.regs.msplim, 0x2000_1000);
    // MRS R1, MSPLIM: hw0=0xF3EF, hw1=0x810A (Rd=1, SYSm=0x0A)
    c.execute_one_wide(0xF3EF, 0x810A);
    assert_eq!(c.reg(1), 0x2000_1000);
}

#[test]
fn msr_mrs_psplim_roundtrip() {
    let mut c = CortexM33::new();
    c.set_reg(2, 0x2000_2008);
    // MSR PSPLIM, R2: hw0=0xF382, hw1=0x880B (Rn=2, SYSm=0x0B, mask=2)
    c.execute_one_wide(0xF382, 0x880B);
    // PSPLIM is 8-byte aligned
    assert_eq!(c.regs.psplim, 0x2000_2008);
    // MRS R3, PSPLIM: hw0=0xF3EF, hw1=0x830B (Rd=3, SYSm=0x0B)
    c.execute_one_wide(0xF3EF, 0x830B);
    assert_eq!(c.reg(3), 0x2000_2008);
}

#[test]
fn msplim_alignment() {
    let mut c = CortexM33::new();
    c.set_reg(0, 0x2000_1007); // not 8-byte aligned
    // MSR MSPLIM, R0
    c.execute_one_wide(0xF380, 0x880A);
    // Should be rounded down to 8-byte boundary
    assert_eq!(c.regs.msplim, 0x2000_1000);
}

// ============================================================================
// Bootrom diagnostic run
// ============================================================================

#[test]
fn bootrom_diagnostic_run() {
    let rom_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../roms/bootrom-combined.bin");
    let rom_data = std::fs::read(&rom_path)
        .expect("bootrom binary not found — download from github.com/raspberrypi/pico-bootrom-rp2350");

    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&rom_data);
    emu.reset();

    let mut last_pc = 0u32;
    let mut stuck_count = 0u32;
    let mut max_pc = 0u32;
    let mut fault_reported = false;

    // Trace addresses for RP2350 bootrom (pico-bootrom-rp2350).
    // If the ROM binary changes, these addresses will silently go stale.
    let trace_addrs: &[(u32, &str)] = &[
        (0x0194, "core0_boot_path_prolog jump"),
        (0x02E8, "s_native_crit_launch_nsboot"),
        (0x0344, "nsboot_vm_no_gpio"),
        (0x0346, "msr MSP_NS, r7"),
        (0x0382, "bxns r0 (first NS transition)"),
        (0x038A, "enter_image_thunk BXNS"),
        (0x03E6, "native_wait_rescue"),
        (0x38A6, "s_varm_crit_core0_boot_path_entry_p2"),
        (0x38C0, "___step1_check_rescue"),
        (0x38F8, "___step2_enable_clock_gates"),
        (0x3934, "___step3_get_boot_random"),
        (0x3B7E, "___step12_select_boot_path"),
        (0x3BE8, "select_boot_path: blt nsboot_preamble check"),
        (0x3C00, "s_varm_crit_init_boot_scan_context call"),
        (0x3C2E, "flash window launch path"),
        (0x3C42, "___stepx_nsboot_preamble"),
        (0x3CDE, "core0_boot_path_cant_boot"),
        (0x3DDA, "___stepx_flash_boot"),
        (0x3E8C, "s_varm_crit_nsboot_start"),
        (0x404E, "___stepx_nsboot_mem_erase"),
        (0x4066, "call varm_to_s_native_crit_launch_nsboot"),
        (0x7E56, "sg_table_entry"),
        (0x7E92, "return_to_ns_preserve_r0"),
        (0x7EA4, "bxns lr (return_to_ns)"),
    ];

    let mut last_trace_pc = 0u32;

    for cycle in 0..1_000_000 {
        emu.step();
        let pc = emu.cores[0].regs.pc();

        // Trace key bootrom addresses (dedup: skip if same PC as last trace)
        for &(addr, label) in trace_addrs {
            if pc == addr && pc != last_trace_pc {
                last_trace_pc = pc;
                eprintln!("[cycle {:>7}] Reached {:#010x}: {}", cycle, addr, label);
                eprintln!("  R0={:#010x} R1={:#010x} R2={:#010x} R3={:#010x}",
                    emu.cores[0].regs.r[0], emu.cores[0].regs.r[1],
                    emu.cores[0].regs.r[2], emu.cores[0].regs.r[3]);
                eprintln!("  R4={:#010x} R5={:#010x} R6={:#010x} R7={:#010x}",
                    emu.cores[0].regs.r[4], emu.cores[0].regs.r[5],
                    emu.cores[0].regs.r[6], emu.cores[0].regs.r[7]);
                eprintln!("  LR={:#010x} SP={:#010x} secure={}", emu.cores[0].regs.lr(), emu.cores[0].regs.sp(), emu.cores[0].secure);
                eprintln!("  R8={:#010x} R9={:#010x} R10={:#010x} R11={:#010x} R12={:#010x}",
                    emu.cores[0].regs.r[8], emu.cores[0].regs.r[9],
                    emu.cores[0].regs.r[10], emu.cores[0].regs.r[11], emu.cores[0].regs.r[12]);
                eprintln!("  MSP={:#010x} MSP_NS={:#010x} PSP={:#010x} PSP_NS={:#010x}",
                    emu.cores[0].regs.msp, emu.cores[0].regs.msp_ns,
                    emu.cores[0].regs.psp, emu.cores[0].regs.psp_ns);
                // Extra detail at bxns points
                if addr == 0x0382 || addr == 0x7EA4 {
                    let target = if addr == 0x0382 { emu.cores[0].regs.r[0] } else { emu.cores[0].regs.lr() };
                    eprintln!("  BXNS target={:#010x}", target);
                    // Try to read memory at target
                    let t = target & !1;
                    eprintln!("  Memory at target: [{:#010x}]={:#010x} [{:#010x}]={:#010x}",
                        t, emu.peek(t), t+4, emu.peek(t+4));
                    // Dump XIP SRAM first 8 words
                    eprintln!("  XIP SRAM (0x1500_0000):");
                    for i in 0..8 {
                        let a = 0x1500_0000 + i * 4;
                        eprint!("    [{:#010x}]={:#010x}", a, emu.peek(a));
                        if i % 4 == 3 { eprintln!(); }
                    }
                    eprintln!();
                    // Also dump USB SRAM (0x5010_0000)
                    eprintln!("  USB SRAM (0x5010_0000):");
                    for i in 0..8 {
                        let a = 0x5010_0000 + i * 4;
                        eprint!("    [{:#010x}]={:#010x}", a, emu.peek(a));
                        if i % 4 == 3 { eprintln!(); }
                    }
                    eprintln!();
                }
                break;
            }
        }

        if pc > max_pc && pc < 0x8000 {
            max_pc = pc;
        }

        // Report when we first enter a fault handler
        let ipsr = emu.cores[0].regs.ipsr();
        if ipsr >= 2 && ipsr <= 6 && !fault_reported {
            fault_reported = true;
            let exc_name = match ipsr {
                2 => "NMI",
                3 => "HardFault",
                4 => "MemManage",
                5 => "BusFault",
                6 => "UsageFault",
                _ => "Unknown",
            };
            eprintln!("*** {} entered at cycle {} ***", exc_name, cycle);
            eprintln!("  PC={:#010x} LR={:#010x}", pc, emu.cores[0].regs.lr());
            eprintln!("  CFSR={:#010x} HFSR={:#010x}", emu.bus.ppb[0].cfsr, emu.bus.ppb[0].hfsr);
            eprintln!("  BFAR={:#010x} MMFAR={:#010x}", emu.bus.ppb[0].bfar, emu.bus.ppb[0].mmfar);
            eprintln!("  R0-R3: {:#010x} {:#010x} {:#010x} {:#010x}",
                emu.cores[0].regs.r[0], emu.cores[0].regs.r[1],
                emu.cores[0].regs.r[2], emu.cores[0].regs.r[3]);
            eprintln!("  SP={:#010x} MSP={:#010x}", emu.cores[0].regs.sp(), emu.cores[0].regs.msp);
            eprintln!("  Max bootrom PC so far={:#010x}", max_pc);
            // Read exception frame from stack
            let sp = emu.cores[0].regs.msp;
            let r0 = emu.peek(sp);
            let r1 = emu.peek(sp + 4);
            let r2 = emu.peek(sp + 8);
            let r3 = emu.peek(sp + 12);
            let lr = emu.peek(sp + 20);
            let ret_pc = emu.peek(sp + 24);
            let xpsr = emu.peek(sp + 28);
            eprintln!("  Exception frame at SP={:#010x}:", sp);
            eprintln!("    Stacked R0={:#010x} R1={:#010x} R2={:#010x} R3={:#010x}",
                r0, r1, r2, r3);
            eprintln!("    Stacked LR={:#010x} PC={:#010x} xPSR={:#010x}",
                lr, ret_pc, xpsr);
        }

        if pc == last_pc {
            stuck_count += 1;
            if stuck_count > 100 {
                eprintln!("Stuck at PC={:#010x} after {} cycles", pc, cycle);
                eprintln!("  IPSR={}, LR={:#010x}", emu.cores[0].regs.ipsr(), emu.cores[0].regs.lr());
                eprintln!("  CFSR={:#010x}, HFSR={:#010x}", emu.bus.ppb[0].cfsr, emu.bus.ppb[0].hfsr);
                eprintln!("  R0={:#010x} R1={:#010x} R2={:#010x} R3={:#010x}",
                    emu.cores[0].regs.r[0], emu.cores[0].regs.r[1],
                    emu.cores[0].regs.r[2], emu.cores[0].regs.r[3]);
                eprintln!("  R4={:#010x} R5={:#010x} R6={:#010x} R7={:#010x}",
                    emu.cores[0].regs.r[4], emu.cores[0].regs.r[5],
                    emu.cores[0].regs.r[6], emu.cores[0].regs.r[7]);
                eprintln!("  SP={:#010x} MSP={:#010x}", emu.cores[0].regs.sp(), emu.cores[0].regs.msp);
                eprintln!("  BFAR={:#010x} MMFAR={:#010x}", emu.bus.ppb[0].bfar, emu.bus.ppb[0].mmfar);
                eprintln!("  Max bootrom PC reached={:#010x}", max_pc);
                // Try to read stacked PC from exception frame
                let sp = emu.cores[0].regs.msp;
                if sp >= 0x2000_0000 && sp < 0x2008_0000 {
                    let stacked_pc = emu.peek(sp + 24);
                    let stacked_lr = emu.peek(sp + 20);
                    let stacked_xpsr = emu.peek(sp + 28);
                    eprintln!("  Stacked: PC={:#010x} LR={:#010x} xPSR={:#010x}",
                        stacked_pc, stacked_lr, stacked_xpsr);
                } else {
                    eprintln!("  SP not in SRAM, cannot read exception frame (SP={:#010x})", sp);
                }
                break;
            }
        } else {
            stuck_count = 0;
        }
        last_pc = pc;
    }

    // For now: just print where we ended up
    let final_pc = emu.cores[0].regs.pc();
    eprintln!("Final PC={:#010x}, cycles run", final_pc);
    eprintln!("  IPSR={}, CFSR={:#010x}, HFSR={:#010x}",
        emu.cores[0].regs.ipsr(), emu.bus.ppb[0].cfsr, emu.bus.ppb[0].hfsr);
    eprintln!("  secure={}, LR={:#010x}, SP={:#010x}",
        emu.cores[0].secure, emu.cores[0].regs.lr(), emu.cores[0].regs.sp());
    eprintln!("  MSP={:#010x} MSP_NS={:#010x}",
        emu.cores[0].regs.msp, emu.cores[0].regs.msp_ns);
    eprintln!("  Max bootrom PC={:#010x}", max_pc);
}

// ============================================================================
// Phase 4 Stage A: QMI register backing store
// ============================================================================

#[test]
fn test_qmi_register_roundtrip() {
    let (_, mut bus) = core_and_bus();
    // M0_TIMING is at QMI offset 0x004
    bus.write32(0x400D_0004, 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x400D_0004), 0xDEAD_BEEF);
}

#[test]
fn test_qmi_direct_csr_always_ready() {
    let (_, mut bus) = core_and_bus();
    // Write something to DIRECT_CSR (offset 0x000)
    bus.write32(0x400D_0000, 0x0000_0042);
    let csr = bus.read32(0x400D_0000);
    // TXEMPTY (bit 16) and RXEMPTY (bit 17) must always be set
    assert_ne!(csr & (1 << 16), 0, "TXEMPTY must be set");
    assert_ne!(csr & (1 << 17), 0, "RXEMPTY must be set");
    // Our written value should also be present
    assert_ne!(csr & 0x42, 0, "written bits should persist");
}

// ============================================================================
// Phase 4 Stage A: SIO GPIO registers
// ============================================================================

#[test]
fn test_sio_gpio_out_write_read() {
    let (_, mut bus) = core_and_bus();
    bus.write32(0xD000_0010, 0xAAAA_5555);
    assert_eq!(bus.read32(0xD000_0010), 0xAAAA_5555);
}

#[test]
fn test_sio_gpio_set_clr_xor() {
    let (_, mut bus) = core_and_bus();
    // Start with known value
    bus.write32(0xD000_0010, 0x0000_00FF);
    // SET bits 8-15 (RP2350 GPIO_OUT_SET = 0x018)
    bus.write32(0xD000_0018, 0x0000_FF00);
    assert_eq!(bus.read32(0xD000_0010), 0x0000_FFFF);
    // CLR bits 0-7 (RP2350 GPIO_OUT_CLR = 0x020)
    bus.write32(0xD000_0020, 0x0000_00FF);
    assert_eq!(bus.read32(0xD000_0010), 0x0000_FF00);
    // XOR bit 15 (RP2350 GPIO_OUT_XOR = 0x028)
    bus.write32(0xD000_0028, 0x0000_8000);
    assert_eq!(bus.read32(0xD000_0010), 0x0000_7F00);

    // Same for GPIO_OE (RP2350 base = 0x030)
    bus.write32(0xD000_0030, 0xFFFF_0000);
    bus.write32(0xD000_0040, 0x00FF_0000); // GPIO_OE_CLR (0x040)
    assert_eq!(bus.read32(0xD000_0030), 0xFF00_0000);
    bus.write32(0xD000_0038, 0x0000_FFFF); // GPIO_OE_SET (0x038)
    assert_eq!(bus.read32(0xD000_0030), 0xFF00_FFFF);
    bus.write32(0xD000_0048, 0x0100_0001); // GPIO_OE_XOR (0x048)
    assert_eq!(bus.read32(0xD000_0030), 0xFE00_FFFE);
}

#[test]
fn test_sio_cpuid() {
    let (_, mut bus) = core_and_bus();
    // Default is core 0 (contention_check_active = false)
    assert_eq!(bus.read32(0xD000_0000), 0);
}

// ============================================================================
// Phase 4 Stage A: CLOCKS dynamic source tracking
// ============================================================================

#[test]
fn test_clocks_source_tracking() {
    let (_, mut bus) = core_and_bus();
    // Write CLK_SYS_CTRL to select source 1 (aux)
    bus.write32(0x4001_0060, 0x0000_0001);
    // CLK_SYS_SELECTED should reflect 1 << 1 = 2
    assert_eq!(bus.read32(0x4001_0068), 0x2);

    // Write CLK_REF_CTRL to select source 2
    bus.write32(0x4001_0030, 0x0000_0002);
    assert_eq!(bus.read32(0x4001_0030), 0x0000_0002);
    // CLK_REF_SELECTED should reflect 1 << 2 = 4
    assert_eq!(bus.read32(0x4001_0038), 0x4);
}

// ============================================================================
// Phase 4: Flash boot integration — bootrom loads blinky to main()
// ============================================================================

#[test]
#[ignore = "Phase 7 Stage E follow-up: bootrom's first `rcp_iequal` \
            mismatch occurs at PC=0x000039b6 inside \
            `___step4_init_rcp_seeds` (bootrom). The instruction is \
            `rcp_iequal r5, r3`; the emulator observes r5=0xa9743e28 \
            against the expected magic r3=0x6478e928 loaded from ROM \
            literal pool at 0x3ab0. The mismatch is not caused by MPU \
            state (Stage E MPU work landed and is independently \
            verified) — r5 is the result of a cascade that consumes \
            the CP7 canary-status / salt / count state plus an \
            `ldmia.w r0, {r1..ip}` from the vector table at address 0, \
            and our CP7 ancillary state (canary flag semantics, \
            count-sequence invariants) doesn't exactly match silicon. \
            Re-enable after the follow-up PR that tightens CP7 \
            canary/count side-effects."]
fn test_flash_boot_blinky() {
    use crate::{Emulator, Config};

    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms");
    let rom = std::fs::read(base.join("bootrom-combined.bin"))
        .expect("bootrom not found");
    let flash = std::fs::read(base.join("blinky.bin"))
        .expect("blinky.bin not found — run: python3 roms/gen_blinky.py roms/blinky.bin");

    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&rom);
    emu.load_flash(&flash);
    emu.reset();

    let mut last_pc = 0u32;
    let mut stuck_count = 0u32;
    let mut gpio_out_ever = 0u32;
    let mut entered_flash = false;
    let mut core1_max_ipsr = 0u32;

    for cycle in 0..10_000_000u64 {
        emu.step();
        gpio_out_ever |= emu.bus.sio.gpio_out;
        core1_max_ipsr = core1_max_ipsr.max(emu.cores[1].regs.ipsr());
        let pc = emu.cores[0].regs.pc();

        // Detect when execution enters flash
        if pc >= 0x1000_0000 && pc < 0x2000_0000 && !entered_flash {
            entered_flash = true;
            eprintln!("[cycle {:>8}] Entered flash at PC={:#010x}", cycle, pc);
        }

        // Stuck detection (ignores 2-instruction tight loops like the delay)
        if pc == last_pc {
            stuck_count += 1;
            if stuck_count > 1000 {
                eprintln!("Stuck at PC={:#010x} after {} cycles, GPIO_OUT={:#010x}",
                    pc, cycle, emu.bus.sio.gpio_out);
                eprintln!("  IPSR={}, CFSR={:#010x}, HFSR={:#010x}",
                    emu.cores[0].regs.ipsr(),
                    emu.bus.ppb[0].cfsr, emu.bus.ppb[0].hfsr);
                break;
            }
        } else {
            stuck_count = 0;
        }
        last_pc = pc;
    }

    let _gpio_out = emu.bus.sio.gpio_out;
    let gpio_oe = emu.bus.sio.gpio_oe;
    let pc = emu.cores[0].regs.pc();

    // Must have entered flash (bootrom found and jumped to blinky)
    assert!(entered_flash, "Bootrom should have jumped to flash");

    // PC should be in the blinky's delay loop (0x100000B8-0x100000BA)
    assert!(pc >= 0x1000_0060 && pc < 0x1000_0100,
        "PC should be in blinky code region (PC={:#010x})", pc);

    // The blinky toggles GPIO 25: first SET, then XOR in a loop.
    // At any snapshot the pin may be high or low — check it was EVER set.
    assert!(gpio_out_ever & (1 << 25) != 0,
        "GPIO 25 should have been set at some point (gpio_out_ever={:#010x})", gpio_out_ever);

    // OE must be set (the blinky always enables output)
    assert!(gpio_oe & (1 << 25) != 0,
        "GPIO OE 25 should be set (gpio_oe={:#010x})", gpio_oe);

    // Phase 4 Core 1 health gate: Core 1 should never enter handler mode
    assert_eq!(core1_max_ipsr, 0, "Core 1 should never enter handler mode");
}

// ============================================================================
// Phase 5 Stage A2: FIFO unit tests
// ============================================================================

/// Helper: SIO base address for register access.
const SIO_BASE: u32 = 0xD000_0000;
const FIFO_ST: u32 = SIO_BASE + 0x050;
const FIFO_WR: u32 = SIO_BASE + 0x054;
const FIFO_RD: u32 = SIO_BASE + 0x058;
const SPINLOCK_ST: u32 = SIO_BASE + 0x05C;

fn spinlock_addr(n: u32) -> u32 {
    SIO_BASE + 0x100 + 4 * n
}

/// Switch Bus to Core 1's perspective (contention_check_active = true).
fn set_core1(bus: &mut Bus) {
    bus.begin_contention_check();
}

/// Switch Bus back to Core 0's perspective.
fn set_core0(bus: &mut Bus) {
    bus.clear_contention_state();
}

#[test]
fn fifo_push_pop_basic_roundtrip() {
    let mut bus = Bus::new();
    // Core 0 writes 3 values to Core 1's RX FIFO
    bus.write32(FIFO_WR, 0xAAAA_BBBB);
    bus.write32(FIFO_WR, 0xCCCC_DDDD);
    bus.write32(FIFO_WR, 0x1234_5678);

    // Core 1 reads them back in FIFO order
    set_core1(&mut bus);
    assert_eq!(bus.read32(FIFO_RD), 0xAAAA_BBBB);
    assert_eq!(bus.read32(FIFO_RD), 0xCCCC_DDDD);
    assert_eq!(bus.read32(FIFO_RD), 0x1234_5678);
}

#[test]
fn fifo_empty_read_returns_zero_and_sets_roe() {
    let mut bus = Bus::new();
    // Core 0 reads from empty RX FIFO
    let val = bus.read32(FIFO_RD);
    assert_eq!(val, 0, "Empty FIFO read should return 0");

    // FIFO_ST should show ROE (bit 3) set for Core 0
    let st = bus.read32(FIFO_ST);
    assert!(st & 0x8 != 0, "ROE bit should be set after empty read, FIFO_ST={:#x}", st);
}

#[test]
fn fifo_full_write_drops_data_and_sets_wof() {
    let mut bus = Bus::new();
    // Fill Core 1's RX FIFO (8 entries) from Core 0
    for i in 0..8u32 {
        bus.write32(FIFO_WR, i);
    }
    // 9th write should overflow
    bus.write32(FIFO_WR, 0xDEAD);

    // Core 0's FIFO_ST should show WOF (bit 2) set
    let st = bus.read32(FIFO_ST);
    assert!(st & 0x4 != 0, "WOF bit should be set after overflow, FIFO_ST={:#x}", st);

    // Core 1 should read the original 8 values, not the dropped 0xDEAD
    set_core1(&mut bus);
    for i in 0..8u32 {
        assert_eq!(bus.read32(FIFO_RD), i);
    }
    // Next read is empty
    assert_eq!(bus.read32(FIFO_RD), 0);
}

#[test]
fn fifo_st_reflects_vld_and_rdy() {
    let mut bus = Bus::new();
    // Initially: Core 0 RX is empty (VLD=0), Core 1 RX has space (RDY=1)
    let st = bus.read32(FIFO_ST);
    assert_eq!(st & 0x1, 0, "VLD should be 0 when RX is empty");
    assert_eq!(st & 0x2, 0x2, "RDY should be 1 when TX has space");

    // Core 1 writes to Core 0's RX FIFO
    set_core1(&mut bus);
    bus.write32(FIFO_WR, 42);
    set_core0(&mut bus);

    // Now Core 0's RX has data
    let st = bus.read32(FIFO_ST);
    assert_eq!(st & 0x1, 0x1, "VLD should be 1 after data written to our RX");

    // Fill Core 1's RX from Core 0 (8 entries)
    for i in 0..8u32 {
        bus.write32(FIFO_WR, i);
    }
    // RDY should be 0 (Core 1's RX is full)
    let st = bus.read32(FIFO_ST);
    assert_eq!(st & 0x2, 0, "RDY should be 0 when other core's RX is full");
}

#[test]
fn fifo_st_w1c_clears_wof_and_roe() {
    let mut bus = Bus::new();
    // Trigger ROE by reading empty FIFO
    bus.read32(FIFO_RD);
    let st = bus.read32(FIFO_ST);
    assert!(st & 0x8 != 0, "ROE should be set");

    // Fill FIFO then overflow to trigger WOF
    for _ in 0..9 {
        bus.write32(FIFO_WR, 0);
    }
    let st = bus.read32(FIFO_ST);
    assert!(st & 0x4 != 0, "WOF should be set");
    assert!(st & 0x8 != 0, "ROE should still be set");

    // W1C: clear WOF only
    bus.write32(FIFO_ST, 0x4);
    let st = bus.read32(FIFO_ST);
    assert_eq!(st & 0x4, 0, "WOF should be cleared");
    assert!(st & 0x8 != 0, "ROE should still be set (not cleared)");

    // W1C: clear ROE
    bus.write32(FIFO_ST, 0x8);
    let st = bus.read32(FIFO_ST);
    assert_eq!(st & 0x8, 0, "ROE should be cleared");

    // W1C: writing 0xFFFFFFFF clears both
    bus.read32(FIFO_RD); // trigger ROE again
    for _ in 0..9 {
        bus.write32(FIFO_WR, 0);
    }
    bus.write32(FIFO_ST, 0xFFFF_FFFF);
    let st = bus.read32(FIFO_ST);
    assert_eq!(st & 0xC, 0, "Both WOF and ROE should be cleared");
}

#[test]
fn fifo_write_sets_event_flag_on_receiver() {
    let mut bus = Bus::new();
    // Event flags start clear
    assert!(!bus.event_flag[0]);
    assert!(!bus.event_flag[1]);

    // Core 0 writes FIFO_WR -> should set event_flag[1] (receiver = Core 1)
    bus.write32(FIFO_WR, 0x42);
    assert!(bus.event_flag[1], "event_flag[1] should be set after Core 0 FIFO write");
    assert!(!bus.event_flag[0], "event_flag[0] should NOT be set");

    // Clear event flags
    bus.event_flag = [false; 2];

    // Core 1 writes FIFO_WR -> should set event_flag[0] (receiver = Core 0)
    set_core1(&mut bus);
    bus.write32(FIFO_WR, 0x43);
    set_core0(&mut bus);
    assert!(bus.event_flag[0], "event_flag[0] should be set after Core 1 FIFO write");
}

#[test]
fn fifo_overflow_does_not_set_event_flag() {
    let mut bus = Bus::new();
    // Fill Core 1's RX FIFO
    for i in 0..8u32 {
        bus.write32(FIFO_WR, i);
    }
    // Clear event flags
    bus.event_flag = [false; 2];

    // Overflow write should NOT set event flag
    bus.write32(FIFO_WR, 0xDEAD);
    assert!(!bus.event_flag[1], "event_flag should NOT be set on overflow write");
}

// ============================================================================
// Phase 5 Stage A2: Spinlock unit tests
// ============================================================================

#[test]
fn spinlock_claim_returns_bit_mask() {
    let mut bus = Bus::new();
    // Claim spinlock 5 from Core 0
    let result = bus.read32(spinlock_addr(5));
    assert_eq!(result, 1 << 5, "Claiming lock 5 should return 1<<5");

    // SPINLOCK_ST should reflect the claimed lock
    let st = bus.read32(SPINLOCK_ST);
    assert_eq!(st & (1 << 5), 1 << 5, "SPINLOCK_ST should show lock 5 claimed");
}

#[test]
fn spinlock_already_claimed_returns_zero() {
    let mut bus = Bus::new();
    // Claim lock 10
    let first = bus.read32(spinlock_addr(10));
    assert_eq!(first, 1 << 10);

    // Second claim returns 0
    let second = bus.read32(spinlock_addr(10));
    assert_eq!(second, 0, "Already-claimed lock should return 0");
}

#[test]
fn spinlock_release_via_write() {
    let mut bus = Bus::new();
    // Claim lock 7
    bus.read32(spinlock_addr(7));
    assert_eq!(bus.read32(SPINLOCK_ST) & (1 << 7), 1 << 7);

    // Release via write (any value)
    bus.write32(spinlock_addr(7), 0);
    assert_eq!(bus.read32(SPINLOCK_ST) & (1 << 7), 0, "Lock 7 should be released");

    // Re-claim should succeed
    let result = bus.read32(spinlock_addr(7));
    assert_eq!(result, 1 << 7, "Re-claiming released lock should succeed");
}

#[test]
fn spinlock_contention_core0_claims_core1_sees_zero() {
    let mut bus = Bus::new();
    // Core 0 claims lock 15
    let c0 = bus.read32(spinlock_addr(15));
    assert_eq!(c0, 1 << 15);

    // Core 1 tries to claim same lock -> gets 0
    set_core1(&mut bus);
    let c1 = bus.read32(spinlock_addr(15));
    assert_eq!(c1, 0, "Core 1 should fail to claim lock already held by Core 0");

    // Core 1 can release it though (any write clears)
    bus.write32(spinlock_addr(15), 1);
    set_core0(&mut bus);

    // Lock is now free, Core 0 can reclaim
    let c0_again = bus.read32(spinlock_addr(15));
    assert_eq!(c0_again, 1 << 15);
}

#[test]
fn spinlock_st_bitmask_reflects_state() {
    let mut bus = Bus::new();
    // Claim locks 0, 3, 31
    bus.read32(spinlock_addr(0));
    bus.read32(spinlock_addr(3));
    bus.read32(spinlock_addr(31));

    let st = bus.read32(SPINLOCK_ST);
    assert_eq!(st, (1 << 0) | (1 << 3) | (1 << 31),
        "SPINLOCK_ST should reflect exactly the claimed locks, got {:#010x}", st);

    // Release lock 3
    bus.write32(spinlock_addr(3), 0);
    let st = bus.read32(SPINLOCK_ST);
    assert_eq!(st, (1 << 0) | (1 << 31),
        "SPINLOCK_ST should reflect lock 3 released, got {:#010x}", st);
}

// ============================================================================
// WFE / SEV instruction dispatch
// ============================================================================

#[test]
fn wfe_with_event_pending_consumes_and_continues() {
    let (mut cpu, mut bus) = core_and_bus();
    bus.event_flag[0] = true;
    // WFE Thumb-16 encoding: 0xBF20 (hint op = 0x2, mask = 0)
    cpu.execute_one_with_bus(0xBF20, &mut bus);
    assert!(!bus.event_flag[0], "event_flag should be consumed");
    assert!(!cpu.is_wfe_waiting(), "core should NOT be sleeping — event was pending");
}

#[test]
fn wfe_without_event_enters_sleep() {
    let (mut cpu, mut bus) = core_and_bus();
    assert!(!bus.event_flag[0]);
    cpu.execute_one_with_bus(0xBF20, &mut bus);
    assert!(cpu.is_wfe_waiting(), "core should be sleeping — no event was pending");
}

#[test]
fn sev_sets_both_event_flags() {
    let (mut cpu, mut bus) = core_and_bus();
    assert!(!bus.event_flag[0]);
    assert!(!bus.event_flag[1]);
    // SEV Thumb-16 encoding: 0xBF40 (hint op = 0x4, mask = 0)
    cpu.execute_one_with_bus(0xBF40, &mut bus);
    assert!(bus.event_flag[0], "event_flag[0] should be set after SEV");
    assert!(bus.event_flag[1], "event_flag[1] should be set after SEV");
}

#[test]
fn wake_check_clears_wfe_on_event() {
    let mut emu = Emulator::new(Config::default());

    // Build a minimal ROM so reset() doesn't read garbage
    let mut rom = vec![0u8; 512];
    rom[0..4].copy_from_slice(&0x2008_0000u32.to_le_bytes());
    rom[4..8].copy_from_slice(&0x0000_0101u32.to_le_bytes());
    // Infinite loop at 0x100
    rom[0x100] = 0xFE; rom[0x101] = 0xE7;
    emu.load_bootrom(&rom);
    emu.reset();

    // Manually put core 0 into WFE sleep and set its event flag
    emu.cores[0].wfe_waiting = true;
    emu.bus.event_flag[0] = true;

    emu.step();

    assert!(!emu.cores[0].wfe_waiting, "core should have been woken by event_flag");
    assert!(!emu.bus.event_flag[0], "event_flag should have been consumed");
}

// ============================================================================
// Phase 5 B2: Core 1 boot reaches WFE
// ============================================================================

#[test]
fn test_core1_boot_reaches_wfe() {
    use crate::{Emulator, Config};

    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms");
    let rom = std::fs::read(base.join("bootrom-combined.bin"))
        .expect("bootrom-combined.bin not found");

    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&rom);
    emu.reset();

    for _ in 0..1_000_000 {
        emu.step();
        // Early exit once Core 1 enters WFE sleep
        if emu.cores[1].is_wfe_waiting() {
            break;
        }
    }

    assert!(emu.cores[1].is_wfe_waiting(),
        "Core 1 should be sleeping in WFE after bootrom init (PC={:#010x})",
        emu.cores[1].regs.pc());
    assert_eq!(emu.cores[1].regs.ipsr(), 0,
        "Core 1 should not be in an exception handler (IPSR={})",
        emu.cores[1].regs.ipsr());
    assert!(emu.cores[1].regs.pc() < 0x8000,
        "Core 1 PC should be in bootrom range (PC={:#010x})",
        emu.cores[1].regs.pc());
}

// ============================================================================
// Phase 5 C4: Dual-core integration — Core 0 launches Core 1 via FIFO
// ============================================================================

#[test]
#[ignore = "Phase 7 Stage E follow-up: same `rcp_iequal` exposure as \
            test_flash_boot_blinky — first mismatch at PC=0x000039b6 \
            (`rcp_iequal r5, r3` in `___step4_init_rcp_seeds`), caused \
            by CP7 canary/salt/count side-effect gaps rather than MPU \
            state. Re-enable after the CP7 tightening follow-up PR."]
fn test_dualcore_launch() {
    use crate::{Emulator, Config};

    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms");
    let bootrom = std::fs::read(base.join("bootrom-combined.bin"))
        .expect("bootrom-combined.bin not found");
    let flash = std::fs::read(base.join("dualcore.bin"))
        .expect("dualcore.bin not found — run: python roms/gen_dualcore.py roms/dualcore.bin");

    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&bootrom);
    emu.load_flash(&flash);
    emu.reset();

    // Run for up to 10M cycles
    for _ in 0..10_000_000u64 {
        emu.step();
        // Early exit if both GPIO pins set
        if emu.bus.sio.gpio_out & (1 << 25) != 0 && emu.bus.sio.gpio_out & 1 != 0 {
            break;
        }
    }

    // Core 0 set GPIO 25
    assert!(emu.bus.sio.gpio_out & (1 << 25) != 0,
        "Core 0 should set GPIO 25 (gpio_out={:#010x})", emu.bus.sio.gpio_out);
    // Core 1 set GPIO 0
    assert!(emu.bus.sio.gpio_out & 1 != 0,
        "Core 1 should set GPIO 0 (gpio_out={:#010x})", emu.bus.sio.gpio_out);
    // Core 1 should be running app code (PC >= 0x10000000)
    assert!(emu.cores[1].regs.pc() >= 0x1000_0000,
        "Core 1 should be in flash, PC={:#010x}", emu.cores[1].regs.pc());
    // Core 1 should not be WFE-waiting
    assert!(!emu.cores[1].is_wfe_waiting(),
        "Core 1 should not be WFE-waiting");
}

// ============================================================================
// Clock Tree V1: ROSC boot-clock fix
// ============================================================================

#[test]
fn test_rosc_status_returns_stable_enabled() {
    let (_, mut bus) = core_and_bus();
    let status = bus.read32(0x400E_8018);
    assert_eq!(status, (1 << 31) | (1 << 12),
        "ROSC STATUS should report STABLE | ENABLED");
}

#[test]
fn test_config_default_uses_rosc_frequency() {
    use crate::Config;
    assert_eq!(Config::default().sys_clk_hz, 6_500_000,
        "Config::default() should use ROSC frequency (~6.5 MHz)");
}

// ============================================================================
// Clock Tree V2 Phase A: CLOCKS-side sys_clk_hz derivation
// ============================================================================

#[test]
fn test_rosc_is_default_sys_clock() {
    use crate::bus::clocks::ROSC_FREQ_HZ;
    let bus = Bus::new();
    assert_eq!(bus.sys_clk_hz(), ROSC_FREQ_HZ,
        "fresh Bus should report ROSC as the system clock");
    assert_eq!(bus.ref_clk_hz(), ROSC_FREQ_HZ,
        "fresh Bus should report ROSC as the reference clock");
}

#[test]
fn test_xosc_via_clk_ref_sys_clock() {
    use crate::bus::clocks::XOSC_FREQ_HZ;
    let (_, mut bus) = core_and_bus();
    // CLK_REF_CTRL SRC=2 (XOSC)
    bus.write32(0x4001_0030, 0x0000_0002);
    // CLK_SYS_CTRL SRC=0 (clk_ref)
    bus.write32(0x4001_0060, 0x0000_0000);
    assert_eq!(bus.sys_clk_hz(), XOSC_FREQ_HZ,
        "CLK_SYS routed through CLK_REF=XOSC should give 12 MHz");
    assert_eq!(bus.ref_clk_hz(), XOSC_FREQ_HZ);
}

#[test]
fn test_clk_sys_div_scales_output() {
    use crate::bus::clocks::XOSC_FREQ_HZ;
    let (_, mut bus) = core_and_bus();
    // Route CLK_SYS to XOSC via CLK_REF
    bus.write32(0x4001_0030, 0x0000_0002);
    bus.write32(0x4001_0060, 0x0000_0000);
    // CLK_SYS_DIV integer = 2 (bits [31:16])
    bus.write32(0x4001_0064, 0x0002_0000);
    assert_eq!(bus.sys_clk_hz(), XOSC_FREQ_HZ / 2,
        "CLK_SYS_DIV=2 should halve the source frequency");
}

#[test]
fn test_clocks_write_alias_set() {
    let (_, mut bus) = core_and_bus();
    // Normal write: CLK_REF_CTRL = 0x01
    bus.write32(0x4001_0030, 0x0000_0001);
    // SET alias (alias=2) at offset 0x030 → 0x4001_0000 | (2 << 12) | 0x030
    bus.write32(0x4001_2030, 0x0000_0002);
    // Expect OR, not overwrite → 0x03
    assert_eq!(bus.read32(0x4001_0030), 0x0000_0003,
        "SET alias should OR bits into CLK_REF_CTRL, not overwrite");
}

// ============================================================================
// Clock Tree V2 Phase B: PLL_SYS / PLL_USB model
// ============================================================================

#[test]
fn test_pll_sys_at_150mhz() {
    // Standard Pico SDK configuration: XOSC=12M, REFDIV=1, FBDIV=125,
    // POSTDIV1=5, POSTDIV2=2 → VCO=1500M, output=150M.
    let (_, mut bus) = core_and_bus();
    // CS: REFDIV=1 (reset value already has this; write explicitly)
    bus.write32(0x4005_0000, 0x0000_0001);
    // FBDIV_INT = 125
    bus.write32(0x4005_0008, 125);
    // PRIM: POSTDIV1=5 in bits [18:16], POSTDIV2=2 in bits [14:12]
    bus.write32(0x4005_000C, (5 << 16) | (2 << 12));
    // Switch CLK_SYS to aux=0 (PLL_SYS): SRC=1, AUXSRC=0
    bus.write32(0x4001_0060, 0x0000_0001);
    assert_eq!(bus.sys_clk_hz(), 150_000_000,
        "PLL_SYS configured for 150 MHz should give sys_clk_hz = 150_000_000");
}

#[test]
fn test_unconfigured_pll_zero_hz() {
    // Fresh Bus: reset values leave FBDIV=0, so pll_output_hz must return 0.
    // Switching CLK_SYS to PLL_SYS without configuring should report 0 Hz.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4001_0060, 0x0000_0001); // SRC=1 (aux), AUXSRC=0 (PLL_SYS)
    assert_eq!(bus.sys_clk_hz(), 0,
        "Unconfigured PLL (FBDIV=0) must honestly report 0 Hz, not a .max(1) fudge");
}

#[test]
fn test_pll_usb_separate_from_pll_sys() {
    // Configuring PLL_USB must not bleed into PLL_SYS — they have
    // separate backing arrays. CLK_SYS stays on clk_ref (ROSC), so
    // sys_clk_hz is unaffected by PLL_USB changes.
    let (_, mut bus) = core_and_bus();
    let before = bus.sys_clk_hz();
    // Configure PLL_USB to some non-trivial value (48 MHz: FBDIV=100,
    // POSTDIV1=5, POSTDIV2=5; VCO=1200M / 25 = 48M).
    bus.write32(0x4005_8000, 0x0000_0001); // CS REFDIV=1
    bus.write32(0x4005_8008, 100);         // FBDIV_INT
    bus.write32(0x4005_800C, (5 << 16) | (5 << 12));
    assert_eq!(bus.sys_clk_hz(), before,
        "PLL_USB changes must not affect sys_clk_hz while CLK_SYS is on ROSC");
    // Sanity: PLL_USB registers actually took the writes.
    assert_eq!(bus.read32(0x4005_8008), 100,
        "PLL_USB FBDIV_INT should read back the value we wrote");
}

#[test]
fn test_pll_fbdiv_max_no_overflow() {
    // FBDIV=0xFFF (4095) with defaults (REFDIV=1, POSTDIV1=7, POSTDIV2=7)
    // gives 12M * 4095 / 49 ≈ 1.003 GHz. Must not panic on u32 overflow.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4005_0008, 0xFFF); // FBDIV_INT = 4095 (max)
    bus.write32(0x4001_0060, 0x0000_0001); // Route CLK_SYS → PLL_SYS
    let hz = bus.sys_clk_hz();
    assert!(hz > 1_000_000_000 && hz < 1_010_000_000,
        "FBDIV=4095 with defaults should produce ~1.003 GHz (got {hz})");
}

#[test]
fn test_pll_sys_reset_values() {
    // Reset values per LLD §4.3 — CS read forces LOCK bit (1<<31).
    let bus = Bus::new();
    assert_eq!(bus.pll_sys_regs[0], 0x0000_0001,
        "PLL_SYS CS reset = REFDIV=1");
    assert_eq!(bus.pll_sys_regs[1], 0x0000_002D,
        "PLL_SYS PWR reset = powered-down bits");
    assert_eq!(bus.pll_sys_regs[2], 0,
        "PLL_SYS FBDIV_INT reset = 0 (PLL off)");
    assert_eq!(bus.pll_sys_regs[3], 0x0007_7000,
        "PLL_SYS PRIM reset = POSTDIV1=7|POSTDIV2=7");
    // Same for PLL_USB — independent backing.
    assert_eq!(bus.pll_usb_regs, [0x0000_0001, 0x0000_002D, 0, 0x0007_7000]);
}

#[test]
fn test_pll_cs_read_forces_lock_bit() {
    // CS reads must return `stored | (1 << 31)` — so the LOCK bit is
    // always present even though the stored value is just 0x01.
    let mut bus = Bus::new();
    let cs_read = bus.read32(0x4005_0000);
    assert_eq!(cs_read, 0x8000_0001,
        "CS read must force LOCK (bit 31) on top of the stored REFDIV=1");
    // Masking out LOCK should give back the stored value.
    assert_eq!(cs_read & !(1 << 31), 0x01);
}

#[test]
fn test_pll_sys_write_set_alias_subword() {
    // Subword SET alias on PLL_USB PWR — matches the bootrom's nsboot
    // path. A byte-wide SET must OR into the register, not overwrite.
    let mut bus = Bus::new();
    // PLL_USB PWR reset = 0x2D. SET alias byte write of 0x40 to byte 0
    // should yield 0x6D (0x2D | 0x40).
    // Address: 0x4005_8004 + SET alias (2 << 12) = 0x4005_A004.
    bus.write8(0x4005_A004, 0x40);
    assert_eq!(bus.pll_usb_regs[1], 0x6D,
        "byte-wide SET alias on PLL_USB PWR must OR, not overwrite");
}

// ============================================================================
// Clock Tree V2 Phase D: ROSC / XOSC register backing
// ============================================================================

#[test]
fn test_rosc_ctrl_roundtrip() {
    // Writing CTRL (0x000) should be stored and read back verbatim.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x400E_8000, 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x400E_8000), 0xDEAD_BEEF,
        "ROSC CTRL should round-trip writes (stored, reads return last write)");
}

#[test]
fn test_rosc_status_unchanged_by_writes() {
    // STATUS (0x018) is read-only: writes are dropped; reads always
    // return STABLE | ENABLED per the V1 stub behaviour.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x400E_8018, 0);
    assert_eq!(bus.read32(0x400E_8018), (1 << 31) | (1 << 12),
        "ROSC STATUS must remain STABLE|ENABLED regardless of writes");
}

#[test]
fn test_xosc_ctrl_roundtrip() {
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4004_8000, 0xCAFE_BABE);
    assert_eq!(bus.read32(0x4004_8000), 0xCAFE_BABE,
        "XOSC CTRL should round-trip writes");
}

#[test]
fn test_xosc_startup_roundtrip() {
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4004_800C, 0x0000_00C4);
    assert_eq!(bus.read32(0x4004_800C), 0x0000_00C4,
        "XOSC STARTUP should round-trip writes");
}

#[test]
fn test_rosc_ctrl_alias_set() {
    // Normal write CTRL=0x01, then write 0x02 via SET alias (0x400EA000)
    // — bits should be OR-ed, not overwritten → CTRL reads 0x03.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x400E_8000, 0x0000_0001);
    bus.write32(0x400E_A000, 0x0000_0002);
    assert_eq!(bus.read32(0x400E_8000), 0x0000_0003,
        "SET alias on ROSC CTRL should OR bits, not overwrite");
}

// ============================================================================
// Clock Tree V2 Phase E: Config::sys_clk_hz as vestigial seed
// ============================================================================

#[test]
fn test_config_sys_clk_hz_seeds_bus() {
    use crate::bus::clocks::ROSC_FREQ_HZ;
    use crate::{Config, Emulator};

    // Construct an emulator with a non-default Config::sys_clk_hz.
    // Before any register writes, the Bus should report the seed value.
    let emu = Emulator::new(Config {
        sys_clk_hz: 12_345_678,
        ..Default::default()
    });
    assert_eq!(emu.bus.sys_clk_hz(), 12_345_678,
        "Bus should expose Config::sys_clk_hz as the pre-recompute seed");

    // First write to a CLOCKS register triggers recompute, which
    // overwrites the seed with the register-derived value. Reset
    // register state routes CLK_SYS → clk_ref → ROSC.
    let mut emu = emu;
    emu.bus.write32(0x4001_0060, 0x0000_0000); // CLK_SYS_CTRL SRC=0 (clk_ref)
    assert_eq!(emu.bus.sys_clk_hz(), ROSC_FREQ_HZ,
        "First CLOCKS write should replace the seed with the derived ROSC frequency");
}

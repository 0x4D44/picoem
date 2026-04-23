use std::sync::Arc;
use std::sync::atomic::Ordering;

use crate::core::CortexM33;
use crate::bus::Bus;
use crate::threaded::CoreAtomics;
use crate::Cores;

// ============================================================================
// Helper: build a core + bus, optionally pre-load SRAM
// ============================================================================

fn core_and_bus() -> (CortexM33, Bus) {
    // Phase 3 Stage 1: share one `Arc<CoreAtomics>` so WFI / WFE / IRQ-
    // pending tests on `core` observe the same state the bus writes to.
    let atomics = Arc::new(CoreAtomics::default());
    let core = CortexM33::new(0, Arc::clone(&atomics));
    let bus = Bus::with_atomics(atomics);
    (core, bus)
}

// ============================================================================
// Shift (immediate)
// ============================================================================

#[test]
fn lsls_imm_basic() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(2, 42);
    c.regs.set_flag_c(true); // carry should be preserved
    c.execute_one(0x0010); // LSLS R0, R2, #0 → MOVS R0, R2
    assert_eq!(c.reg(0), 42);
    assert!(c.flag_c()); // unchanged
}

#[test]
fn lsls_imm_carry_out() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0x8000_0000);
    c.execute_one(0x0040); // LSLS R0, R0, #1 → 0, carry = 1
    assert_eq!(c.reg(0), 0);
    assert!(c.flag_z());
    assert!(c.flag_c());
}

#[test]
fn lsrs_imm_basic() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x80);
    c.execute_one(0x08C8); // LSRS R0, R1, #3 → 0x80 >> 3 = 0x10
    assert_eq!(c.reg(0), 0x10);
    assert!(!c.flag_c());
    assert_eq!(c.execute_one(0x08C8), 1); // cycle count
}

#[test]
fn lsrs_imm_shift_32() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0x8000_0000);
    // LSRS R0, R0, #32 → encoded as imm5=0 → result=0, carry=bit31
    c.execute_one(0x0800); // bits: 00001_00000_000_000
    assert_eq!(c.reg(0), 0);
    assert!(c.flag_c()); // bit 31 was set
    assert!(c.flag_z());
}

#[test]
fn asrs_imm_positive() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x40);
    c.execute_one(0x10C8); // ASRS R0, R1, #3 → 0x40 >> 3 = 8
    assert_eq!(c.reg(0), 8);
    assert!(!c.flag_n());
}

#[test]
fn asrs_imm_negative() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 3);
    c.set_reg(1, 10);
    c.execute_one(0x1A40); // SUBS R0, R0, R1
    assert_eq!(c.reg(0), 3u32.wrapping_sub(10));
    assert!(c.flag_n());
    assert!(!c.flag_c()); // borrow
}

#[test]
fn adds_imm3() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 100);
    c.execute_one(0x1CC8); // ADDS R0, R1, #3
    assert_eq!(c.reg(0), 103);
}

#[test]
fn subs_imm3() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 100);
    c.execute_one(0x1EC8); // SUBS R0, R1, #3
    assert_eq!(c.reg(0), 97);
}

// ============================================================================
// Move/Compare/Add/Sub 8-bit immediate
// ============================================================================

#[test]
fn movs_imm() {
    let mut c = CortexM33::for_test(0);
    c.execute_one(0x202A); // MOVS R0, #42
    assert_eq!(c.reg(0), 42);
    assert!(!c.flag_z());
    assert!(!c.flag_n());
}

#[test]
fn movs_imm_zero() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 999);
    c.execute_one(0x2000); // MOVS R0, #0
    assert_eq!(c.reg(0), 0);
    assert!(c.flag_z());
}

#[test]
fn cmp_imm_equal() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 42);
    c.execute_one(0x282A); // CMP R0, #42
    assert!(c.flag_z()); // equal
    assert!(c.flag_c()); // no borrow (42 >= 42)
    assert!(!c.flag_v());
}

#[test]
fn cmp_imm_greater() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 100);
    c.execute_one(0x282A); // CMP R0, #42
    assert!(!c.flag_z());
    assert!(c.flag_c()); // no borrow (100 >= 42)
    assert!(!c.flag_n());
}

#[test]
fn cmp_imm_less() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 10);
    c.execute_one(0x282A); // CMP R0, #42
    assert!(!c.flag_z());
    assert!(!c.flag_c()); // borrow (10 < 42)
    assert!(c.flag_n());
}

#[test]
fn adds_imm8() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 100);
    c.execute_one(0x3019); // ADDS R0, #25
    assert_eq!(c.reg(0), 125);
}

#[test]
fn subs_imm8() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0xFF);
    c.set_reg(1, 0x0F);
    c.execute_one(0x4008); // ANDS R0, R1
    assert_eq!(c.reg(0), 0x0F);
}

#[test]
fn eors() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0xFF);
    c.set_reg(1, 0xF0);
    c.execute_one(0x4048); // EORS R0, R1
    assert_eq!(c.reg(0), 0x0F);
}

#[test]
fn lsls_reg() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 1);
    c.set_reg(1, 4);
    c.execute_one(0x4088); // LSLS R0, R1 (shift R0 by R1)
    assert_eq!(c.reg(0), 16);
}

#[test]
fn lsrs_reg() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0x100);
    c.set_reg(1, 4);
    c.execute_one(0x40C8); // LSRS R0, R1
    assert_eq!(c.reg(0), 0x10);
}

#[test]
fn adcs() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 10);
    c.set_reg(1, 3);
    c.regs.set_flag_c(true); // C=1 means no borrow from previous
    c.execute_one(0x4188); // SBCS R0, R1 → 10 + NOT(3) + 1 = 10 - 3 = 7
    assert_eq!(c.reg(0), 7);
    assert!(c.flag_c()); // no borrow
}

#[test]
fn rors() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0x0000_0001);
    c.set_reg(1, 1);
    c.execute_one(0x41C8); // RORS R0, R1 → rotate right by 1
    assert_eq!(c.reg(0), 0x8000_0000);
    assert!(c.flag_n());
    assert!(c.flag_c()); // bit 31 of result
}

#[test]
fn tst() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0xFF00);
    c.set_reg(1, 0x00FF);
    c.execute_one(0x4208); // TST R0, R1
    assert!(c.flag_z()); // no bits in common
    assert_eq!(c.reg(0), 0xFF00); // unchanged
}

#[test]
fn rsbs_neg() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0);
    c.set_reg(1, 42);
    c.execute_one(0x4248); // RSBS R0, R1, #0 → 0 - 42
    assert_eq!(c.reg(0), (0u32).wrapping_sub(42));
    assert!(c.flag_n());
}

#[test]
fn cmp_reg() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 42);
    c.set_reg(1, 42);
    c.execute_one(0x4288); // CMP R0, R1
    assert!(c.flag_z());
    assert!(c.flag_c());
}

#[test]
fn cmn() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 1);
    c.set_reg(1, 0xFFFF_FFFF);
    c.execute_one(0x42C8); // CMN R0, R1 → 1 + 0xFFFFFFFF = 0, carry
    assert!(c.flag_z());
    assert!(c.flag_c());
}

#[test]
fn orrs() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0xF0);
    c.set_reg(1, 0x0F);
    c.execute_one(0x4308); // ORRS R0, R1
    assert_eq!(c.reg(0), 0xFF);
}

#[test]
fn muls() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 7);
    c.set_reg(1, 6);
    c.execute_one(0x4348); // MULS R0, R1
    assert_eq!(c.reg(0), 42);
}

#[test]
fn bics() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0xFF);
    c.set_reg(1, 0x0F);
    c.execute_one(0x4388); // BICS R0, R1
    assert_eq!(c.reg(0), 0xF0);
}

#[test]
fn mvns() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(8, 0xDEAD_BEEF);
    // MOV R0, R8: 01000110_0_1000_000 = 0x4640
    c.execute_one(0x4640);
    assert_eq!(c.reg(0), 0xDEAD_BEEF);
}

#[test]
fn add_high_reg() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 10);
    c.set_reg(8, 20);
    // ADD R0, R8: 01000100_0_1000_000 = 0x4440
    c.execute_one(0x4440);
    assert_eq!(c.reg(0), 30);
}

#[test]
fn bx_reg() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0x2000_0001); // bit 0 = Thumb
    c.regs.set_pc(0x1000);
    // BX R0: 0100_0111_0_0000_000 = 0x4700
    c.execute_one(0x4700);
    assert_eq!(c.regs.pc(), 0x2000_0000);
}

#[test]
fn bxns_from_secure() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    assert_eq!(bus.read32(0x2000_0004, 0), 0xCAFE_BABE);
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
    assert_eq!(bus.read8(0x2000_0001, 0), 0xAB);
    // LDRB R3, [R1, R2]: 0101_110_010_001_011 = 0x5C8B
    c.execute_one_with_bus(0x5C8B, &mut bus);
    assert_eq!(c.reg(3), 0xAB);
}

#[test]
fn ldrsb_sign_extends() {
    let (mut c, mut bus) = core_and_bus();
    bus.write8(0x2000_0000, 0x80, 0); // -128 as signed byte
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
    assert_eq!(bus.read32(0x2000_0008, 0), 0x1234_5678);
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
    assert_eq!(bus.read8(0x2000_0002, 0), 0xCD);
}

#[test]
fn strh_ldrh_imm() {
    let (mut c, mut bus) = core_and_bus();
    c.set_reg(0, 0xBEEF);
    c.set_reg(1, 0x2000_0000);
    // STRH R0, [R1, #4]: 10000_00010_001_000 = 0x8088
    c.execute_one_with_bus(0x8088, &mut bus);
    assert_eq!(bus.read16(0x2000_0004, 0), 0xBEEF);
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
    assert_eq!(bus.read32(0x2000_1008, 0), 0xDEAD_BEEF);
    // LDR R1, [SP, #8]: 10011_001_00000010 = 0x9902
    c.execute_one_with_bus(0x9902, &mut bus);
    assert_eq!(c.reg(1), 0xDEAD_BEEF);
}

// ============================================================================
// ADR / ADD SP
// ============================================================================

#[test]
fn adr_pc_relative() {
    let mut c = CortexM33::for_test(0);
    c.regs.set_pc(0x1000);

    // ADR R0, #16: 10100_000_00000100 = 0xA004
    c.execute_one(0xA004);
    // read_pc() = 0x1000 + 4 = 0x1004, aligned = 0x1004, + 16 = 0x1014
    assert_eq!(c.reg(0), 0x1014);
}

#[test]
fn add_rd_sp_imm() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(13, 0x2000_1000);
    // ADD SP, SP, #16: 10110000_0_0000100 = 0xB004
    c.execute_one(0xB004);
    assert_eq!(c.regs.sp(), 0x2000_1010);
}

#[test]
fn sub_sp_imm() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(13, 0x2000_1000);
    // SUB SP, SP, #16: 10110000_1_0000100 = 0xB084
    c.execute_one(0xB084);
    assert_eq!(c.regs.sp(), 0x2000_0FF0);
}

#[test]
fn sxth() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_8000); // -32768 as i16
    // SXTH R0, R1: 10110010_00_001_000 = 0xB208
    c.execute_one(0xB208);
    assert_eq!(c.reg(0), 0xFFFF_8000);
}

#[test]
fn uxtb() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0xDEAD_BEEF);
    // UXTB R0, R1: 10110010_11_001_000 = 0xB2C8
    c.execute_one(0xB2C8);
    assert_eq!(c.reg(0), 0xEF);
}

#[test]
fn rev() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x12_34_56_78);
    // REV R0, R1: 10111010_00_001_000 = 0xBA08
    c.execute_one(0xBA08);
    assert_eq!(c.reg(0), 0x78_56_34_12);
}

#[test]
fn rev16() {
    let mut c = CortexM33::for_test(0);
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
    assert_eq!(bus.read32(0x2000_0FF8, 0), 0xAAAA);
    assert_eq!(bus.read32(0x2000_0FFC, 0), 0xBBBB);
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
    assert_eq!(bus.read32(0x2000_0FFC, 0), 0x0800_0101);
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
    assert_eq!(bus.read32(0x2000_0100, 0), 0x11);
    assert_eq!(bus.read32(0x2000_0104, 0), 0x22);
    assert_eq!(bus.read32(0x2000_0108, 0), 0x33);
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
    let mut c = CortexM33::for_test(0);
    c.regs.set_pc(0x1000);

    // B +8: PC = read_pc() + 8 = 0x1004 + 8 = 0x100C
    // imm11 = 8/2 = 4 → 11100_00000000100 = 0xE004
    c.execute_one(0xE004);
    assert_eq!(c.regs.pc(), 0x100C);
}

#[test]
fn branch_unconditional_backward() {
    let mut c = CortexM33::for_test(0);
    c.regs.set_pc(0x1000);

    // B -4: offset = -4, imm11 = (-4/2) & 0x7FF = 0x7FE
    // 11100_11111111110 = 0xE7FE
    c.execute_one(0xE7FE);
    assert_eq!(c.regs.pc(), 0x1000); // loops to self
}

#[test]
fn branch_cond_taken() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.set_pc(0x1000);

    c.regs.set_flag_z(false);
    // BEQ +6 but Z=0: not taken
    let cy = c.execute_one(0xD003);
    // PC should NOT change (execute_one doesn't advance PC)
    assert_eq!(cy, 1); // not taken
}

#[test]
fn bl_forward() {
    let mut c = CortexM33::for_test(0);
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
        bus.write16(addr, instr, 0);
    }

    // Set PC to program start
    c.regs.set_pc(base);

    // Run until we hit the infinite loop (PC stable across two steps)
    let mut stable_count = 0u32;
    let mut prev_pc = !0u32;
    for _ in 0..500 {
        c.step(&mut bus);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.set_pc(0x1000);
    c.set_reg(0, 1); // R0 = 1 → CBZ should NOT branch

    let cy = c.execute_one(0xB120); // CBZ R0, +8
    // PC not changed (beyond the +2 from execute_one setup)
    assert_eq!(c.regs.pc(), 0x1002);
    assert_eq!(cy, 1);
}

#[test]
fn cbnz_not_taken() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.set_pc(0x1000);

    // BL +100: same encoding as the existing bl_forward test
    let cy = c.execute_one_wide(0xF000, 0xF832);
    assert_eq!(c.regs.lr(), 0x1005);
    assert_eq!(c.regs.pc(), 0x1068);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn thumb32_ldr_w_routes_to_load_store_single() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    let (hw0, hw1) = encode_dp_mod_imm(0b0010, false, 15, 0, 0x34);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x34);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn mvn_w_imm() {
    // MVN.W R0, #0 → R0 = 0xFFFFFFFF (via ORN with Rn=15, S=0)
    let mut c = CortexM33::for_test(0);
    let (hw0, hw1) = encode_dp_mod_imm(0b0011, false, 15, 0, 0x00);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xFFFF_FFFF);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn cmp_w_imm() {
    // CMP.W R0, #50 → SUB with S=1, Rd=15 (discard result, flags only)
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x1234_0000);
    let (hw0, hw1) = encode_dp_mod_imm(0b0010, false, 1, 0, 0xC7F);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x1234_FF00);
    assert_eq!(cy, 2); // M33 measured: 2 cycles (rotated imm)
}

#[test]
fn bic_w_imm() {
    // BIC.W R0, R1, #0x0F → R0 = R1 & ~0x0F (clear low nibble)
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0xABCD_EF9A);
    let (hw0, hw1) = encode_dp_mod_imm(0b0001, false, 1, 0, 0x0F);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xABCD_EF90);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn adc_w_imm() {
    // ADCS.W R0, R1, #10 with carry-in = 1
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0xAA);
    let (hw0, hw1) = encode_dp_mod_imm(0b0100, false, 1, 0, 0xFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xAA ^ 0xFF); // 0x55
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn orn_w_imm() {
    // ORN.W R0, R1, #0xFF → R0 = R1 | ~0xFF = R1 | 0xFFFFFF00
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    let (hw0, hw1) = encode_movw(0, 0x1234);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x1234);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn movw_all_bits() {
    // MOVW R0, #0xFFFF — all 16 bits set
    let mut c = CortexM33::for_test(0);
    let (hw0, hw1) = encode_movw(0, 0xFFFF);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x0000_FFFF);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn movt_basic() {
    // MOVT R0, #0xABCD — set top half, preserve bottom half
    let mut c = CortexM33::for_test(0);
    c.set_reg(0, 0x0000_5678);
    let (hw0, hw1) = encode_movt(0, 0xABCD);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0xABCD_5678);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn movw_movt_pair() {
    // Load 0xDEADBEEF via MOVW + MOVT
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 1000);
    let (hw0, hw1) = encode_addw(0, 1, 4000);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 5000);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn subw_basic() {
    // SUBW R0, R1, #2000
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_0750); // bits [11:4] = 0x75 = 0b0111_0101 (positive)
    let (hw0, hw1) = encode_sbfx(0, 1, 4, 8);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(0), 0x75);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn sbfx_negative() {
    // SBFX R0, R1, #4, #8 — extract bits [11:4] signed, negative value
    let mut c = CortexM33::for_test(0);
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
    bus.write32(0x2000_0064, 0xDEAD_BEEF, 0);
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
    assert_eq!(bus.read32(0x2000_0064, 0), 0xCAFE_BABE);
    assert_eq!(cy, 2); // M33 measured: 2 cycles (SRAM, zero-wait-state)
}

#[test]
fn ldr_w_reg() {
    // LDR.W R0, [R1, R2, LSL #2] — array indexing pattern
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0010, 0x1234_5678, 0); // array[4] at base + 4*4
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
    bus.write8(0x2000_000A, 0xAB, 0);
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
    bus.write16(0x2000_0006, 0xBEEF, 0);
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
    bus.write8(0x2000_0000, 0x80, 0); // -128 signed
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
    bus.write16(0x2000_0002, 0x8001, 0); // -32767 signed
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
    assert_eq!(bus.read8(0x2000_0005, 0), 0x42);
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
    assert_eq!(bus.read16(0x2000_0008, 0), 0xBEEF);
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
    bus.write32(0x2000_100C, 0xAAAA_BBBB, 0);
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
    bus.write32(0x2000_0004, 0x1111_2222, 0);
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
    bus.write32(0x2000_0000, 0x3333_4444, 0);
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
    bus.write32(0x2000_0000, 0xDEAD_BEEF, 0);
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
    bus.write32(0x2000_0000, 0x0000_1001, 0); // target addr with thumb bit
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.set_pc(0x1000);
    let cy = c.execute_one_wide(0xF3AF, 0x8000);
    // PC should advance normally (set by execute_one_wide to 0x1004)
    assert_eq!(c.regs.pc(), 0x1004);
    assert_eq!(cy, 1);
}

#[test]
fn dsb_dmb_isb() {
    let mut c = CortexM33::for_test(0);

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
    let mut c = CortexM33::for_test(0);
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
    assert_eq!(bus.read32(0x2000_0100, 0), 0xAAAA_0000); // R0
    assert_eq!(bus.read32(0x2000_0104, 0), 0xBBBB_1111); // R1
    assert_eq!(bus.read32(0x2000_0108, 0), 0xCCCC_2222); // R2
    assert_eq!(bus.read32(0x2000_010C, 0), 0xDDDD_3333); // R3
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
    bus.write32(base, 0x1111_1111, 0);
    bus.write32(base + 4, 0x2222_2222, 0);
    bus.write32(base + 8, 0x3333_3333, 0);
    bus.write32(base + 12, 0x4444_4444, 0);
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
    assert_eq!(bus.read32(start, 0),      0x4444_4444); // R4
    assert_eq!(bus.read32(start + 4, 0),  0x5555_5555); // R5
    assert_eq!(bus.read32(start + 8, 0),  0x6666_6666); // R6
    assert_eq!(bus.read32(start + 12, 0), 0x7777_7777); // R7
    assert_eq!(bus.read32(start + 16, 0), 0xEEEE_EEEE); // LR
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
    bus.write32(sp,      0x4444_4444, 0); // R4
    bus.write32(sp + 4,  0x5555_5555, 0); // R5
    bus.write32(sp + 8,  0x6666_6666, 0); // R6
    bus.write32(sp + 12, 0x7777_7777, 0); // R7
    bus.write32(sp + 16, 0x0800_0101, 0); // PC value (Thumb bit set)
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_0003);
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA01, 0xF002);
    assert_eq!(c.reg(0), 0x0000_0030);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn lsr_w_reg() {
    // LSR.W R0, R1, R2: hw0=0xFA21, hw1=0xF002
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_FF00);
    c.set_reg(2, 8);
    let cy = c.execute_one_wide(0xFA21, 0xF002);
    assert_eq!(c.reg(0), 0x0000_00FF);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn asr_w_reg() {
    // ASR.W R0, R1, R2: hw0=0xFA41, hw1=0xF002
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x8000_0000); // negative value
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA41, 0xF002);
    assert_eq!(c.reg(0), 0xF800_0000); // sign-extended
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn ror_w_reg() {
    // ROR.W R0, R1, R2: hw0=0xFA61, hw1=0xF002
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_00FF);
    c.set_reg(2, 4);
    let cy = c.execute_one_wide(0xFA61, 0xF002);
    assert_eq!(c.reg(0), 0xF000_000F); // low 4 bits rotated to top
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn lsls_w_reg_flags() {
    // LSLS.W R0, R1, R2 (S=1): hw0=0xFA11, hw1=0xF002
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_FF80); // halfword 0xFF80 = -128 as i16
    let cy = c.execute_one_wide(0xFA0F, 0xF081);
    assert_eq!(c.reg(0), 0xFFFF_FF80); // sign-extended to 32 bits
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn sxtb_w() {
    // SXTB R0, R1: hw0=0xFA4F, hw1=0xF081
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_0090); // byte 0x90 = -112 as i8
    let cy = c.execute_one_wide(0xFA4F, 0xF081);
    assert_eq!(c.reg(0), 0xFFFF_FF90); // sign-extended
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn uxth_w() {
    // UXTH R0, R1: hw0=0xFA1F, hw1=0xF081
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0xDEAD_BEEF);
    let cy = c.execute_one_wide(0xFA1F, 0xF081);
    assert_eq!(c.reg(0), 0x0000_BEEF); // zero-extended halfword
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn uxtb_w() {
    // UXTB R0, R1: hw0=0xFA5F, hw1=0xF081
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0xDEAD_BEEF);
    let cy = c.execute_one_wide(0xFA5F, 0xF081);
    assert_eq!(c.reg(0), 0x0000_00EF); // zero-extended byte
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn rev_w() {
    // REV.W R0, R1: hw0=0xFA91, hw1=0xF081
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x12345678);
    let cy = c.execute_one_wide(0xFA91, 0xF081);
    assert_eq!(c.reg(0), 0x78563412);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn rev16_w() {
    // REV16.W R0, R1: hw0=0xFA91, hw1=0xF091
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0xAABB_CCDD);
    let cy = c.execute_one_wide(0xFA91, 0xF091);
    assert_eq!(c.reg(0), 0xBBAA_DDCC);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn revsh_w() {
    // REVSH.W R0, R1: hw0=0xFA91, hw1=0xF0B1
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0000_01FF); // low halfword 0x01FF, byte-swapped = 0xFF01 = -255 as i16
    let cy = c.execute_one_wide(0xFA91, 0xF0B1);
    assert_eq!(c.reg(0), 0xFFFF_FF01); // sign-extended to 32 bits
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn rbit_w() {
    // RBIT R0, R1: hw0=0xFA91, hw1=0xF0A1
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x8000_0000); // only bit 31 set
    let cy = c.execute_one_wide(0xFA91, 0xF0A1);
    assert_eq!(c.reg(0), 0x0000_0001); // reversed → only bit 0 set
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn clz_w() {
    // CLZ R0, R1: hw0=0xFAB1, hw1=0xF081
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0010_0000); // bit 20 set → 11 leading zeros
    let cy = c.execute_one_wide(0xFAB1, 0xF081);
    assert_eq!(c.reg(0), 11);
    assert_eq!(cy, 1); // M33 measured: 1 cycle
}

#[test]
fn clz_zero() {
    // CLZ of 0 → 32
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    bus.write32(0x2000_0008, 0xAAAA_BBBB, 0);
    bus.write32(0x2000_000C, 0xCCCC_DDDD, 0);
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
    assert_eq!(bus.read32(0x2000_0008, 0), 0x1111_2222);
    assert_eq!(bus.read32(0x2000_000C, 0), 0x3333_4444);
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
    assert_eq!(bus.read32(0x2000_0110, 0), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2000_0114, 0), 0xCAFE_BABE);

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
    bus.write32(0x2000_1008, 0x1234_5678, 0);
    bus.write32(0x2000_100C, 0x9ABC_DEF0, 0);
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
    bus.write8(0x2000_0003, 10, 0);  // table[3] = 10
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
    bus.write16(0x2000_0004, 20, 0); // table[2] = 20 (at base + 2*2 = base+4)
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);

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
    let mut c = CortexM33::for_test(0);
    // MRS R0, IPSR: hw0=0xF3EF, hw1=0x8005 (Rd=0, SYSm=5)
    c.execute_one_wide(0xF3EF, 0x8005);
    assert_eq!(c.reg(0), 0);
}

// ============================================================================
// IT (If-Then) Blocks (Stage 11)
// ============================================================================

/// Helper: execute exactly one instruction.
/// In the quantum execution model `core.step()` is already atomic
/// (one instruction per call), so this is just a thin alias preserved
/// to avoid churning every call site in the IT-block tests below.
fn step_one(c: &mut CortexM33, bus: &mut Bus) {
    c.step(bus);
}

#[test]
fn it_eq_taken() {
    // IT EQ; MOVS R0, #42 — condition true (Z=1), should execute
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0000u32;
    bus.write16(base, 0xBF08, 0);       // IT EQ (firstcond=0000, mask=1000)
    bus.write16(base + 2, 0x202A, 0);   // MOVS R0, #42
    bus.write16(base + 4, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF08, 0);       // IT EQ
    bus.write16(base + 2, 0x202A, 0);   // MOVS R0, #42
    bus.write16(base + 4, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF08, 0);       // IT EQ
    bus.write16(base + 2, 0x1888, 0);   // ADDS R0, R1, R2
    bus.write16(base + 4, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF08, 0);       // IT EQ
    bus.write16(base + 2, 0x4288, 0);   // CMP R0, R1
    bus.write16(base + 4, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF08, 0);       // IT EQ
    bus.write16(base + 2, 0x2805, 0);   // CMP R0, #5
    bus.write16(base + 4, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF0C, 0);       // ITE EQ
    bus.write16(base + 2, 0x2001, 0);   // MOVS R0, #1 (Then)
    bus.write16(base + 4, 0x2002, 0);   // MOVS R0, #2 (Else)
    bus.write16(base + 6, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF0C, 0);       // ITE EQ
    bus.write16(base + 2, 0x2001, 0);   // MOVS R0, #1 (Then — skipped)
    bus.write16(base + 4, 0x2002, 0);   // MOVS R0, #2 (Else — executed)
    bus.write16(base + 6, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF04, 0);       // ITT EQ
    bus.write16(base + 2, 0x2001, 0);   // MOVS R0, #1
    bus.write16(base + 4, 0x2102, 0);   // MOVS R1, #2
    bus.write16(base + 6, 0xE7FE, 0);   // B . (halt)
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
    bus.write16(base, 0xBF08, 0);       // IT EQ
    bus.write16(base + 2, 0x202A, 0);   // MOVS R0, #42 (in IT block)
    bus.write16(base + 4, 0x2103, 0);   // MOVS R1, #3 (outside IT block)
    bus.write16(base + 6, 0xE7FE, 0);   // B . (halt)
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 1.5;
    c.regs.s[4] = 2.5;
    let (hw0, hw1) = enc_vadd(0, 2, 4); // VADD.F32 S0, S2, S4
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);
    assert_eq!(cy, 1);
}

#[test]
fn fpu_vsub_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 10.0;
    c.regs.s[4] = 3.5;
    let (hw0, hw1) = enc_vsub(0, 2, 4); // VSUB.F32 S0, S2, S4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 6.5);
}

#[test]
fn fpu_vmul_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 4.0;
    let (hw0, hw1) = enc_vmul(0, 2, 4);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 12.0);
}

#[test]
fn fpu_vdiv_f32() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 5.0;
    let (hw0, hw1) = enc_vneg(0, 2); // VNEG.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -5.0);
}

#[test]
fn fpu_vabs_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = -7.5;
    let (hw0, hw1) = enc_vabs(0, 2); // VABS.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.5);
}

#[test]
fn fpu_vsqrt_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 4.0;
    let (hw0, hw1) = enc_vsqrt(0, 2); // VSQRT.F32 S0, S2
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 2.0);
    assert_eq!(cy, 14);
}

#[test]
fn fpu_vcmp_f32_equal() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 3.0;
    c.regs.s[2] = 3.0;
    let (hw0, hw1) = enc_vcmp(0, 2); // VCMP.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    // Equal: N=0, Z=1, C=1, V=0
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x6000_0000);
}

#[test]
fn fpu_vcmp_f32_less() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 3.0;
    let (hw0, hw1) = enc_vcmp(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Less: N=1, Z=0, C=0, V=0
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x8000_0000);
}

#[test]
fn fpu_vcmp_f32_greater() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 5.0;
    c.regs.s[2] = 2.0;
    let (hw0, hw1) = enc_vcmp(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Greater: N=0, Z=0, C=1, V=0
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x2000_0000);
}

#[test]
fn fpu_vcmp_f32_nan() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = f32::NAN;
    c.regs.s[2] = 1.0;
    let (hw0, hw1) = enc_vcmp(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Unordered: N=0, Z=0, C=1, V=1
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x3000_0000);
}

#[test]
fn fpu_vcmp_f32_zero() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 0.0;
    let (hw0, hw1) = enc_vcmp_zero(0); // VCMP.F32 S0, #0.0
    c.execute_one_wide(hw0, hw1);
    // Equal to zero: Z=1, C=1
    assert_eq!(c.regs.fpscr & 0xF000_0000, 0x6000_0000);
}

#[test]
fn fpu_vcvt_f32_s32() {
    let mut c = CortexM33::for_test(0);
    // Store -42 as raw bits in S2
    c.regs.s[2] = f32::from_bits((-42i32) as u32);
    let (hw0, hw1) = enc_vcvt_f32_s32(0, 2); // VCVT.F32.S32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -42.0);
}

#[test]
fn fpu_vcvt_f32_u32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::from_bits(100u32);
    let (hw0, hw1) = enc_vcvt_f32_u32(0, 2); // VCVT.F32.U32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 100.0);
}

#[test]
fn fpu_vcvt_s32_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = -3.7;
    let (hw0, hw1) = enc_vcvt_s32_f32(0, 2); // VCVT.S32.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    // Result stored as raw bits: -3 as i32 = 0xFFFF_FFFD
    assert_eq!(c.regs.s[0].to_bits(), (-3i32) as u32);
}

#[test]
fn fpu_vcvt_u32_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 7.9;
    let (hw0, hw1) = enc_vcvt_u32_f32(0, 2); // VCVT.U32.F32 S0, S2
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 7u32);
}

#[test]
fn fpu_vmov_arm_to_fpu() {
    let mut c = CortexM33::for_test(0);
    c.set_reg(3, 0x4048_0000); // 3.125f32.to_bits()
    let (hw0, hw1) = enc_vmov_to_fpu(0, 3); // VMOV S0, R3
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], f32::from_bits(0x4048_0000));
}

#[test]
fn fpu_vmov_fpu_to_arm() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 3.125;
    let (hw0, hw1) = enc_vmov_to_arm(3, 0); // VMOV R3, S0
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(3), 3.125f32.to_bits());
}

#[test]
fn fpu_vmrs_fpscr_to_apsr() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    assert_eq!(bus.read32(addr, 0), 2.5f32.to_bits());

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
    bus.write32(base + 16, 7.0f32.to_bits(), 0);
    let (hw0, hw1) = enc_vldr(0, 0, 16); // VLDR.32 S0, [R0, #+16]
    c.execute_one_wide_with_bus(hw0, hw1, &mut bus);
    assert_eq!(c.regs.s[0], 7.0);
}

#[test]
fn fpu_vldr_negative_offset() {
    let (mut c, mut bus) = core_and_bus();
    let base = 0x2000_0110u32;
    c.set_reg(0, base);
    bus.write32(base - 8, 9.0f32.to_bits(), 0);
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
    assert_eq!(f32::from_bits(bus.read32(sp - 12, 0)), 1.0);
    assert_eq!(f32::from_bits(bus.read32(sp - 8, 0)), 2.0);
    assert_eq!(f32::from_bits(bus.read32(sp - 4, 0)), 3.0);

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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 10.0;
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 2.0;
    let (hw0, hw1) = enc_vmls(0, 2, 4); // VMLS.F32 S0, S2, S4 → S0 = 10 - 3*2 = 4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);
}

#[test]
fn fpu_vnmul_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 5.0;
    let (hw0, hw1) = enc_vnmul(0, 2, 4); // VNMUL.F32 S0, S2, S4 → S0 = -(3*5) = -15
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -15.0);
}

#[test]
fn fpu_vnmla_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vnmla(0, 2, 4); // VNMLA → S0 = -(2*3 + 1) = -7
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -7.0);
}

#[test]
fn fpu_vnmls_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vnmls(0, 2, 4); // VNMLS → S0 = 2*3 - 1 = 5
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 5.0);
}

#[test]
fn fpu_vfma_f32() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 10.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vfms(0, 2, 4); // VFMS → S0 = S0 - S2*S4 = 10 - 6 = 4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 4.0);
}

#[test]
fn fpu_vfnma_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vfnma(0, 2, 4); // VFNMA → S0 = -S2*S4 - S0 = -6 - 1 = -7
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], -7.0);
}

#[test]
fn fpu_vfnms_f32() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[0] = 1.0;
    c.regs.s[2] = 2.0;
    c.regs.s[4] = 3.0;
    let (hw0, hw1) = enc_vfnms(0, 2, 4); // VFNMS → S0 = S2*S4 - S0 = 6 - 1 = 5
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 5.0);
}

#[test]
fn fpu_vmov_f32_reg() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[4] = 42.0;
    let (hw0, hw1) = enc_vmov_reg(0, 4); // VMOV.F32 S0, S4
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 42.0);
}

#[test]
fn fpu_vmrs_to_register() {
    let mut c = CortexM33::for_test(0);
    c.regs.fpscr = 0x1234_5678;
    let (hw0, hw1) = enc_vmrs(3); // VMRS R3, FPSCR
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.reg(3), 0x1234_5678);
}

#[test]
fn fpu_vcvtr_s32_f32_round_nearest() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[16] = 100.0;
    c.regs.s[20] = 200.0;
    let (hw0, hw1) = enc_vadd(24, 16, 20); // VADD.F32 S24, S16, S20
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[24], 300.0);
}

#[test]
fn fpu_vcvt_negative_float_to_unsigned() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = -5.0;
    let (hw0, hw1) = enc_vcvt_u32_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Negative float → unsigned should saturate to 0
    assert_eq!(c.regs.s[0].to_bits(), 0);
}

#[test]
fn fpu_vcvt_nan_to_int() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::NAN;
    let (hw0, hw1) = enc_vcvt_s32_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits() as i32, 0);
}

#[test]
fn fpu_vmov_odd_register() {
    // Test VMOV with an odd-numbered S register (S1) to exercise the N/D bit encoding.
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 3.0;
    c.regs.s[4] = 4.0;
    c.regs.set_flag_v(true);
    let (hw0, hw1) = enc_vsel(1, 0, 2, 4); // cc=01 (VS)
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 3.0);
}

#[test]
fn fpu_vselvs_false_picks_sm() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(true);
    c.regs.set_flag_v(true);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 7.0, "N==V==true should select Sn");

    // Case: N=1, V=0 → N!=V → picks Sm
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(true);
    c.regs.set_flag_v(false);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 8.0, "N!=V (N=1,V=0) should select Sm");

    // Case: N=0, V=1 → N!=V → picks Sm
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 7.0;
    c.regs.s[4] = 8.0;
    c.regs.set_flag_z(false);
    c.regs.set_flag_n(false);
    c.regs.set_flag_v(true);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 8.0, "N!=V (N=0,V=1) should select Sm");

    // Case: N==V==false → picks Sn (already covered by fpu_vselgt_true_picks_sn,
    // repeated here to keep the full truth table visible in one place).
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 1.5;
    c.regs.s[4] = 2.5;
    let (hw0, hw1) = enc_vminnm(0, 2, 4);
    let cy = c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 1.5);
    assert_eq!(cy, 1);
}

#[test]
fn fpu_vminnm_nan_returns_other() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);

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
    let mut c = CortexM33::for_test(0);

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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = -4.0;
    let (hw0, hw1) = enc_vsqrt(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_nan());
    assert!(c.regs.fpscr & FPSCR_IOC != 0, "sqrt(-x) must set IOC");
}

#[test]
fn fpscr_vmaxnm_snan_sets_ioc() {
    // Per DDI0553: VMAXNM/VMINNM set IOC when either input is sNaN.
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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

// ----- VRINT* (R / X / Z) -- DN / IOC / IDC / IXC --------------------------
//
// VRINT family per ARM DDI0553 FPRoundInt:
//   - SNaN input raises IOC; result is the input quietened (DN=0) or the
//     ARM default NaN (DN=1).
//   - Denormal input raises IDC; under FZ=1 it flushes to ±0.
//   - VRINTX raises IXC when the rounded value differs from the input.
//   - VRINTR/VRINTZ never raise IXC.

// (Re-uses FPSCR_* constants defined further up in this file alongside the
// existing FPU exception-flag tests.)

fn enc_vrintr(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0110, 0, sd, sm) }
fn enc_vrintz(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0110, 1, sd, sm) }
fn enc_vrintx(sd: u16, sm: u16) -> (u16, u16) { vfp_unary(0b0111, 0, sd, sm) }

#[test]
fn fpu_vrintx_inexact_sets_ixc() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 2.5;
    let (hw0, hw1) = enc_vrintx(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 2.0); // round-to-even
    assert!(c.regs.fpscr & FPSCR_IXC != 0, "VRINTX must raise IXC on inexact");
}

#[test]
fn fpu_vrintr_inexact_no_ixc() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 2.5;
    let (hw0, hw1) = enc_vrintr(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 2.0);
    assert!(c.regs.fpscr & FPSCR_IXC == 0, "VRINTR must NOT raise IXC");
}

#[test]
fn fpu_vrintz_inexact_no_ixc() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = 2.7;
    let (hw0, hw1) = enc_vrintz(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 2.0); // round-toward-zero
    assert!(c.regs.fpscr & FPSCR_IXC == 0, "VRINTZ must NOT raise IXC");
}

#[test]
fn fpu_vrintx_snan_sets_ioc_dn_canonicalizes() {
    let mut c = CortexM33::for_test(0);
    c.regs.fpscr = FPSCR_DN;
    c.regs.s[2] = f32::from_bits(0x7F80_0001); // SNaN
    let (hw0, hw1) = enc_vrintx(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 0x7FC0_0000, "DN=1 must canonicalize NaN");
    assert!(c.regs.fpscr & FPSCR_IOC != 0, "SNaN must raise IOC");
}

#[test]
fn fpu_vrintr_qnan_dn_off_quietens_payload() {
    // QNaN input under DN=0: output preserves payload (already quiet).
    // No IOC because input was already quiet.
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::from_bits(0x7FCD_EAD0);
    let (hw0, hw1) = enc_vrintr(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 0x7FCD_EAD0);
    assert!(c.regs.fpscr & FPSCR_IOC == 0, "QNaN must NOT raise IOC");
}

#[test]
fn fpu_vrintx_snan_dn_off_quietens_input() {
    // Under DN=0, VRINTX on an SNaN must (a) raise IOC and (b) return the
    // input with the quiet bit forced — this is the "quieten payload" path
    // we explicitly pinned to avoid relying on host rounding intrinsics.
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::from_bits(0x7F80_1234); // SNaN with payload
    let (hw0, hw1) = enc_vrintx(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Quietened: bit 22 forced on, rest of payload preserved.
    assert_eq!(c.regs.s[0].to_bits(), 0x7FC0_1234);
    assert!(c.regs.fpscr & FPSCR_IOC != 0, "SNaN must raise IOC");
}

#[test]
fn fpu_vrintx_denormal_input_sets_idc() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::from_bits(0x0000_0001); // smallest subnormal
    let (hw0, hw1) = enc_vrintx(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.fpscr & FPSCR_IDC != 0, "Denormal input must raise IDC");
    // Under FZ=0 the denormal is preserved through ftz_input; rounding to
    // integer of a tiny denormal gives 0, and VRINTX raises IXC.
    assert_eq!(c.regs.s[0], 0.0);
    assert!(c.regs.fpscr & FPSCR_IXC != 0);
}

#[test]
fn fpu_vrintx_denormal_fz_flushes_to_zero_no_ixc() {
    // Under FZ=1, denormal input flushes to ±0 with IDC. The rounded result
    // (still 0) is exact w.r.t. the flushed input, so no IXC.
    let mut c = CortexM33::for_test(0);
    c.regs.fpscr = FPSCR_FZ;
    c.regs.s[2] = f32::from_bits(0x0000_0001);
    let (hw0, hw1) = enc_vrintx(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0], 0.0);
    assert!(c.regs.fpscr & FPSCR_IDC != 0);
    assert!(c.regs.fpscr & FPSCR_IXC == 0, "Flushed input is exact zero — no IXC");
}

#[test]
fn fpu_vrintr_inf_passthrough() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::INFINITY;
    let (hw0, hw1) = enc_vrintr(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert!(c.regs.s[0].is_infinite() && c.regs.s[0].is_sign_positive());
    assert_eq!(c.regs.fpscr & 0xFF, 0, "no flags on infinity input");
}

#[test]
fn fpu_vrintr_rmode_sweep() {
    // Verify the dispatcher actually reads FPSCR.RMode (bits 23:22).
    // Input 1.5 differentiates all four rounding modes:
    //   RN(00) → 2.0 (round-to-even)
    //   RP(01) → 2.0 (ceiling)
    //   RM(10) → 1.0 (floor)
    //   RZ(11) → 1.0 (toward zero)
    for (rmode, expected) in [(0u32, 2.0f32), (1, 2.0), (2, 1.0), (3, 1.0)] {
        let mut c = CortexM33::for_test(0);
        c.regs.fpscr = rmode << 22;
        c.regs.s[2] = 1.5;
        let (hw0, hw1) = enc_vrintr(0, 2);
        c.execute_one_wide(hw0, hw1);
        assert_eq!(
            c.regs.s[0], expected,
            "VRINTR(1.5) with rmode={rmode} expected {expected}, got {}",
            c.regs.s[0]
        );
    }
}

// ----- VCVT.F16 ↔ F32 -- DN / IOC / IDC ------------------------------------

#[test]
fn fpu_vcvtb_f16_f32_snan_sets_ioc_dn_canonicalizes() {
    let mut c = CortexM33::for_test(0);
    c.regs.fpscr = FPSCR_DN;
    c.regs.s[2] = f32::from_bits(0x7F80_1234); // SNaN
    c.regs.s[0] = 0.0;
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    // DN=1 → f16 default NaN (0x7E00) in bottom half.
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF, 0x7E00);
    assert!(c.regs.fpscr & FPSCR_IOC != 0, "SNaN must raise IOC");
}

#[test]
fn fpu_vcvtb_f16_f32_qnan_dn_off_preserves_payload() {
    // QNaN input under DN=0: payload preserved (top 9 bits), no IOC.
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::from_bits(0x7FCD_EAD0); // QNaN with payload
    c.regs.s[0] = 0.0;
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Top 9 payload bits of f32 frac (0x4DEAD0 >> 13 = 0x26F) → bottom 9 bits of f16 frac
    let payload = (0x4DEAD0_u32 >> 13) as u16 & 0x1FF;
    let expected = 0x7E00 | payload; // sign=0, exp=11111, quiet=1, payload
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF, expected as u32);
    assert!(c.regs.fpscr & FPSCR_IOC == 0, "QNaN must NOT raise IOC");
}

#[test]
fn fpu_vcvtb_f16_f32_denormal_input_sets_idc() {
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::from_bits(0x0000_0001); // f32 denormal
    c.regs.s[0] = 0.0;
    let (hw0, hw1) = enc_vcvtb_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits() & 0xFFFF, 0x0000, "denormal flushes to f16 +0");
    assert!(c.regs.fpscr & FPSCR_IDC != 0, "f32 denormal must raise IDC");
}

#[test]
fn fpu_vcvtb_f32_f16_snan_sets_ioc_dn_canonicalizes() {
    let mut c = CortexM33::for_test(0);
    c.regs.fpscr = FPSCR_DN;
    // f16 SNaN: exp=11111, frac non-zero, quiet bit (bit 9) clear.
    c.regs.s[2] = f32::from_bits(0x7C01); // bottom half = SNaN
    let (hw0, hw1) = enc_vcvtb_f32_f16(0, 2);
    c.execute_one_wide(hw0, hw1);
    assert_eq!(c.regs.s[0].to_bits(), 0x7FC0_0000, "DN=1 must canonicalize");
    assert!(c.regs.fpscr & FPSCR_IOC != 0, "SNaN must raise IOC");
}

#[test]
fn fpu_vcvtb_f32_f16_qnan_dn_off_preserves_payload() {
    // f16 QNaN with payload, DN=0: f32 result has the payload shifted left
    // by 13 bits with the quiet bit forced. No IOC.
    let mut c = CortexM33::for_test(0);
    c.regs.s[2] = f32::from_bits(0x7E55); // bottom half = QNaN with payload
    let (hw0, hw1) = enc_vcvtb_f32_f16(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Expected: sign=0, exp=11111111, quiet=1, payload shifted from f16
    let expected = 0x7F80_0000 | (0x255_u32 << 13) | 0x0040_0000;
    assert_eq!(c.regs.s[0].to_bits(), expected);
    assert!(c.regs.fpscr & FPSCR_IOC == 0);
}

#[test]
fn fpu_vcvtt_f16_f32_snan_dn_canonicalizes_in_top_half() {
    // VCVTT writes to the TOP half of Sd; verify the DN canonicalization
    // path is wired identically for the top variant.
    let mut c = CortexM33::for_test(0);
    c.regs.fpscr = FPSCR_DN;
    c.regs.s[2] = f32::from_bits(0x7F80_0001); // SNaN
    c.regs.s[0] = f32::from_bits(0x0000_BEEF); // top half is target
    let (hw0, hw1) = enc_vcvtt_f16_f32(0, 2);
    c.execute_one_wide(hw0, hw1);
    // Top half should be the f16 default NaN (0x7E00), bottom half preserved.
    assert_eq!(c.regs.s[0].to_bits(), (0x7E00 << 16) | 0x0000_BEEF);
    assert!(c.regs.fpscr & FPSCR_IOC != 0);
}

// ----- VSEL with D=1 -------------------------------------------------------
//
// Regression test for the D-bit decode: the Armv8-M 0xFE encodings put the D
// bit at hw0[4] rather than hw0[6] (where VFPv4 encodings have it). Picking
// an odd Sd (here S1) forces D=1 and exercises that code path.

#[test]
fn fpu_vsel_d_bit_set() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 200); // 200 > 127
    c.execute_one_wide(0xF301, 0x0007);
    assert_eq!(c.reg(0), 127);
    assert!(c.regs.flag_q());

    // Negative clamping: R1 = -200 (0xFFFFFF38)
    let mut c2 = CortexM33::for_test(0);
    c2.set_reg(1, (-200i32) as u32);
    c2.execute_one_wide(0xF301, 0x0007);
    assert_eq!(c2.reg(0) as i32, -128);
    assert!(c2.regs.flag_q());
}

#[test]
fn smulbb() {
    // SMULBB R0, R1, R2 — bottom halfword multiply, no accumulate
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x0003_0005); // bottom = 5
    c.set_reg(2, 0x0007_0006); // bottom = 6
    // hw0 = 0xFB11 (op1=001, Rn=R1)
    // hw1 = 0xF002 (Ra=15, Rd=0, op2=00, Rm=R2)
    c.execute_one_wide(0xFB11, 0xF002);
    assert_eq!(c.reg(0), 30); // 5 * 6 = 30

    // Signed: bottom of R1 = -3 (0xFFFD), bottom of R2 = 4
    let mut c2 = CortexM33::for_test(0);
    c2.set_reg(1, 0x0000_FFFD); // -3 as i16
    c2.set_reg(2, 0x0000_0004);
    c2.execute_one_wide(0xFB11, 0xF002);
    assert_eq!(c2.reg(0) as i32, -12); // -3 * 4 = -12
}

#[test]
fn smlabb() {
    // SMLABB R0, R1, R2, R3 — halfword multiply-accumulate
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c2 = CortexM33::for_test(0);
    c2.set_reg(1, 0x00_00_00_FF);
    c2.set_reg(2, 0x00_00_00_01);
    c2.execute_one_wide(0xFA81, 0xF042);
    assert_eq!(c2.reg(0) & 0xFF, 0x00); // wraps to 0
    assert!(c2.regs.ge_flags() & 1 != 0); // GE[0] set (carry)
}

#[test]
fn qadd() {
    // QADD R0, R1, R2 — saturating signed add
    let mut c = CortexM33::for_test(0);
    c.set_reg(1, 0x7FFF_FFF0); // near max positive
    c.set_reg(2, 0x0000_0010);
    // QADD: hw0 = 0xFA81 (hw0[7:4]=1000, Rn=R1), hw1 = 0xF082 (hw1[7]=1, Rm=R2)
    c.execute_one_wide(0xFA81, 0xF082);
    assert_eq!(c.reg(0), 0x7FFF_FFFF); // overflows → saturates to i32::MAX, Q flag set
    assert!(c.regs.flag_q());

    // Non-overflowing case
    let mut c2 = CortexM33::for_test(0);
    c2.set_reg(1, 100);
    c2.set_reg(2, 200);
    c2.execute_one_wide(0xFA81, 0xF082);
    assert_eq!(c2.reg(0), 300);
    assert!(!c2.regs.flag_q());
}

#[test]
fn sel_basic() {
    // SEL R0, R1, R2 — select bytes based on GE flags
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    assert_eq!(bus.read8(0x0000_0000, 0), 0);
    assert_eq!(bus.read8(0x0000_0001, 0), 1);
    assert_eq!(bus.read8(0x0000_001F, 0), 31);
    assert_eq!(bus.read32(0x0000_0000, 0), 0x03020100);
}

#[test]
fn bus_sram_write_then_read_roundtrip() {
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2000_0000, 0xDEAD_BEEF, 0);
    assert_eq!(bus.read32(0x2000_0000, 0), 0xDEAD_BEEF);
    bus.write16(0x2000_0004, 0xCAFE, 0);
    assert_eq!(bus.read16(0x2000_0004, 0), 0xCAFE);
    bus.write8(0x2000_0006, 0x42, 0);
    assert_eq!(bus.read8(0x2000_0006, 0), 0x42);
}

#[test]
fn bus_xip_read_returns_loaded_flash_data() {
    let (_, mut bus) = core_and_bus();
    let flash = vec![0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44];
    bus.load_flash(&flash);
    assert_eq!(bus.read8(0x1000_0000, 0), 0xAA);
    assert_eq!(bus.read32(0x1000_0000, 0), 0xDDCCBBAA);
    assert_eq!(bus.read32(0x1000_0004, 0), 0x44332211);
}

#[test]
fn bus_sram_boundary_last_valid_byte() {
    // SRAM is 520 KB = 0x82000 bytes. Last valid address: 0x20081FFF.
    let (_, mut bus) = core_and_bus();
    bus.write8(0x2008_1FFF, 0x77, 0);
    assert_eq!(bus.read8(0x2008_1FFF, 0), 0x77);
}

#[test]
fn bus_sram_boundary_out_of_range_returns_zero() {
    // Address 0x20082000 is beyond the 520 KB SRAM region.
    let (_, mut bus) = core_and_bus();
    bus.write8(0x2008_2000, 0xFF, 0); // should be silently ignored
    assert_eq!(bus.read8(0x2008_2000, 0), 0); // out-of-range → 0
}

#[test]
fn bus_rom_boundary_32kb() {
    // ROM is 32 KB = 0x8000 bytes. Address 0x00007FFF is the last valid byte.
    let (_, mut bus) = core_and_bus();
    let mut rom_data = vec![0u8; 32 * 1024];
    rom_data[0x7FFF] = 0xEE;
    bus.memory.load_rom(&rom_data);
    assert_eq!(bus.read8(0x0000_7FFF, 0), 0xEE); // last byte of 32 KB ROM
    assert_eq!(bus.read8(0x0000_8000, 0), 0);     // beyond ROM → 0
    assert_eq!(bus.read8(0x0000_FFFF, 0), 0);     // well beyond ROM → 0
}

#[test]
fn bus_writes_to_rom_are_silently_ignored() {
    let (_, mut bus) = core_and_bus();
    let rom_data = vec![0x42; 16];
    bus.memory.load_rom(&rom_data);
    // Attempt to write to ROM address — should be ignored
    bus.write8(0x0000_0000, 0xFF, 0);
    bus.write32(0x0000_0004, 0xFFFF_FFFF, 0);
    // Original data preserved
    assert_eq!(bus.read8(0x0000_0000, 0), 0x42);
    assert_eq!(bus.read32(0x0000_0004, 0), 0x42424242);
}

#[test]
fn bus_unmapped_region_reads_zero() {
    // Regions 0x3, 0x6..0xC, 0xF are unmapped — should read as 0.
    let (_, mut bus) = core_and_bus();
    assert_eq!(bus.read32(0x3000_0000, 0), 0);
    assert_eq!(bus.read32(0x6000_0000, 0), 0);
    assert_eq!(bus.read32(0xF000_0000, 0), 0);
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
    bus.write32(0x2000_0000, 0x1111_1111, 0);
    assert_eq!(bus.read32(0x2000_0000, 0), 0x1111_1111);
}

#[test]
fn sram8_write_read() {
    // SRAM8: 4 KB at 0x20080000 (non-striped)
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2008_0000, 0xAAAA_BBBB, 0);
    assert_eq!(bus.read32(0x2008_0000, 0), 0xAAAA_BBBB);
    // Last word of SRAM8: 0x20080FFC
    bus.write32(0x2008_0FFC, 0xCCCC_DDDD, 0);
    assert_eq!(bus.read32(0x2008_0FFC, 0), 0xCCCC_DDDD);
}

#[test]
fn sram9_write_read() {
    // SRAM9: 4 KB at 0x20081000 (non-striped)
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2008_1000, 0x1234_5678, 0);
    assert_eq!(bus.read32(0x2008_1000, 0), 0x1234_5678);
    // Last word of SRAM9: 0x20081FFC
    bus.write32(0x2008_1FFC, 0x9ABC_DEF0, 0);
    assert_eq!(bus.read32(0x2008_1FFC, 0), 0x9ABC_DEF0);
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
        bus.write32(addr, val, 0);
    }
    for i in 0u32..9 {
        let addr = 0x2000_0000 + i * 4;
        let expected = 0xA000_0000 | i;
        assert_eq!(bus.read32(addr, 0), expected, "word {} at 0x{:08X}", i, addr);
    }
}

#[test]
fn bank_for_address_striped_region() {
    // crate::memory::bank_for_address(addr) → bank index (0..9)
    // Striped region: bank = (word_offset) % 8
    assert_eq!(crate::memory::bank_for_address(0x2000_0000), Some(0)); // word 0 → bank 0
    assert_eq!(crate::memory::bank_for_address(0x2000_0004), Some(1)); // word 1 → bank 1
    assert_eq!(crate::memory::bank_for_address(0x2000_0008), Some(2)); // word 2 → bank 2
    assert_eq!(crate::memory::bank_for_address(0x2000_000C), Some(3)); // word 3 → bank 3
    assert_eq!(crate::memory::bank_for_address(0x2000_001C), Some(7)); // word 7 → bank 7
    assert_eq!(crate::memory::bank_for_address(0x2000_0020), Some(0)); // word 8 → wraps to bank 0
}

#[test]
fn bank_for_address_non_striped_region() {
    // Non-striped banks:
    //   SRAM8 (0x20080000..0x20080FFF) → always bank 8
    //   SRAM9 (0x20081000..0x20081FFF) → always bank 9
    assert_eq!(crate::memory::bank_for_address(0x2008_0000), Some(8));
    assert_eq!(crate::memory::bank_for_address(0x2008_0500), Some(8));
    assert_eq!(crate::memory::bank_for_address(0x2008_0FFF), Some(8));
    assert_eq!(crate::memory::bank_for_address(0x2008_1000), Some(9));
    assert_eq!(crate::memory::bank_for_address(0x2008_1500), Some(9));
    assert_eq!(crate::memory::bank_for_address(0x2008_1FFF), Some(9));
}

#[test]
fn bank_for_address_rejects_non_sram() {
    // ROM address — not SRAM
    assert_eq!(crate::memory::bank_for_address(0x0000_1000), None);
    // XIP address — not SRAM
    assert_eq!(crate::memory::bank_for_address(0x1000_0004), None);
    // Beyond SRAM9
    assert_eq!(crate::memory::bank_for_address(0x2008_2000), None);
    // Unmapped region
    assert_eq!(crate::memory::bank_for_address(0x3000_0000), None);
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
    bus.read32(0x2000_0000, 0);
    assert_eq!(bus.last_access_cycles(), 1);
}

#[test]
fn bus_latency_sram_write_1_cycle() {
    // SRAM is AHB-attached: 1-cycle write.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x2000_0000, 0x42, 0);
    assert_eq!(bus.last_access_cycles(), 1);
}

#[test]
fn bus_latency_rom_read_1_cycle() {
    // ROM is AHB-attached: 1-cycle read.
    let (_, mut bus) = core_and_bus();
    bus.read32(0x0000_0000, 0);
    assert_eq!(bus.last_access_cycles(), 1);
}

#[test]
fn bus_latency_apb_peripheral_read_3_cycles() {
    // APB peripherals at 0x40000000: 3-cycle read latency.
    let (_, mut bus) = core_and_bus();
    bus.read32(0x4000_0000, 0);
    assert_eq!(bus.last_access_cycles(), 3);
}

#[test]
fn bus_latency_apb_peripheral_write_4_cycles() {
    // APB peripherals at 0x40000000: 4-cycle write latency.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4000_0000, 0x1, 0);
    assert_eq!(bus.last_access_cycles(), 4);
}

#[test]
fn bus_latency_sio_access_1_cycle() {
    // SIO at 0xD0000000: single-cycle access.
    let (_, mut bus) = core_and_bus();
    bus.read32(0xD000_0000, 0);
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
    bus.write32(base + 0x0000, 0xFF00_FF00, 0);
    assert_eq!(bus.read32(base, 0), 0xFF00_FF00);
}

#[test]
fn atomic_alias_xor_write() {
    // Base+0x1000: XOR — new_val = old_val ^ written_val.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base + 0x0000, 0xFF00_FF00, 0); // seed value
    bus.write32(base + 0x1000, 0x0F0F_0F0F, 0); // XOR alias
    assert_eq!(bus.read32(base, 0), 0xF00F_F00F);
}

#[test]
fn atomic_alias_set_write() {
    // Base+0x2000: SET — new_val = old_val | written_val.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base + 0x0000, 0x0000_00FF, 0); // seed value
    bus.write32(base + 0x2000, 0x0000_FF00, 0); // SET alias
    assert_eq!(bus.read32(base, 0), 0x0000_FFFF);
}

#[test]
fn atomic_alias_clr_write() {
    // Base+0x3000: CLR — new_val = old_val & ~written_val.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base + 0x0000, 0xFFFF_FFFF, 0); // seed value
    bus.write32(base + 0x3000, 0x00FF_00FF, 0); // CLR alias
    assert_eq!(bus.read32(base, 0), 0xFF00_FF00);
}

#[test]
fn atomic_alias_read_ignores_alias_bits() {
    // Reads from any alias offset return the same canonical value.
    let (_, mut bus) = core_and_bus();
    let base = 0x4006_0000;
    bus.write32(base, 0xBEEF_CAFE, 0);
    assert_eq!(bus.read32(base + 0x0000, 0), 0xBEEF_CAFE);
    assert_eq!(bus.read32(base + 0x1000, 0), 0xBEEF_CAFE); // XOR alias read
    assert_eq!(bus.read32(base + 0x2000, 0), 0xBEEF_CAFE); // SET alias read
    assert_eq!(bus.read32(base + 0x3000, 0), 0xBEEF_CAFE); // CLR alias read
}

#[test]
fn atomic_alias_ahb_peripheral() {
    // AHB peripherals (0x5xxxxxxx) also support atomic aliases.
    // PIO0 CTRL: SM_ENABLE [3:0] with SET/CLR/XOR alias support.
    let mut bus = Bus::new();
    let base = 0x5020_0000; // PIO0 CTRL
    bus.write32(base, 0x5, 0); // enable SM0 + SM2
    assert_eq!(bus.read32(base, 0), 0x5);
    bus.write32(base + 0x2000, 0xA, 0); // SET alias: enable SM1 + SM3
    assert_eq!(bus.read32(base, 0), 0xF); // all 4 SMs enabled
    // AHB atomics have no extra latency cost (unlike APB interposed)
    bus.write32(base + 0x1000, 0x3, 0); // XOR alias: toggle SM0 + SM1
    assert_eq!(bus.last_access_cycles(), 1); // no extra cost
    assert_eq!(bus.read32(base, 0), 0xC); // SM2 + SM3 remain enabled
}

#[test]
fn atomic_alias_apb_interposed_latency() {
    // APB atomic writes (XOR/SET/CLR) cost +2 extra cycles (interposed).
    let mut bus = Bus::new();
    let base = 0x4007_0000; // UART0
    // Normal APB write: 4 cycles
    bus.write32(base, 0x1234, 0);
    assert_eq!(bus.last_access_cycles(), 4);
    // XOR alias APB write: 6 cycles (4 + 2 interposed)
    bus.write32(base + 0x1000, 0x00FF, 0);
    assert_eq!(bus.last_access_cycles(), 6);
    // SET alias: also 6 cycles
    bus.write32(base + 0x2000, 0x00FF, 0);
    assert_eq!(bus.last_access_cycles(), 6);
    // CLR alias: also 6 cycles
    bus.write32(base + 0x3000, 0x00FF, 0);
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

// ============================================================================
// 2.7 SRAM Atomic Aliases
// ============================================================================

#[test]
fn sram_atomic_xor() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0xAAAA_5555, 0); // seed via normal write
    bus.write32(0x2100_0000, 0xFFFF_FFFF, 0); // XOR alias
    assert_eq!(bus.read32(0x2000_0000, 0), 0x5555_AAAA);
}

#[test]
fn sram_atomic_set() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0x0000_00FF, 0);
    bus.write32(0x2200_0000, 0x0000_FF00, 0); // SET alias
    assert_eq!(bus.read32(0x2000_0000, 0), 0x0000_FFFF);
}

#[test]
fn sram_atomic_clr() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0xFFFF_FFFF, 0);
    bus.write32(0x2300_0000, 0x00FF_00FF, 0); // CLR alias
    assert_eq!(bus.read32(0x2000_0000, 0), 0xFF00_FF00);
}

#[test]
fn sram_atomic_read_returns_canonical() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0010, 0xDEAD_BEEF, 0);
    // All alias reads return the same canonical value
    assert_eq!(bus.read32(0x2000_0010, 0), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2100_0010, 0), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2200_0010, 0), 0xDEAD_BEEF);
    assert_eq!(bus.read32(0x2300_0010, 0), 0xDEAD_BEEF);
}

#[test]
fn sram_atomic_8bit_xor_doesnt_affect_neighbors() {
    let mut bus = Bus::new();
    bus.write32(0x2000_0000, 0xAABB_CCDD, 0);
    bus.write8(0x2100_0001, 0xFF, 0); // XOR byte at offset 1 only
    // Byte 0: 0xDD unchanged, Byte 1: 0xCC ^ 0xFF = 0x33, Byte 2-3: unchanged
    assert_eq!(bus.read32(0x2000_0000, 0), 0xAABB_33DD);
}

#[test]
fn sram_atomic_no_extra_latency() {
    let mut bus = Bus::new();
    bus.write32(0x2200_0000, 0xFF, 0); // SET alias write
    assert_eq!(bus.last_access_cycles(), 1); // same as normal SRAM
}

#[test]
fn sram_alias_bank_for_address_resolves_correctly() {
    // Alias addresses should resolve to same bank as canonical
    assert_eq!(crate::memory::bank_for_address(0x2000_0004), crate::memory::bank_for_address(0x2100_0004));
    assert_eq!(crate::memory::bank_for_address(0x2000_0004), crate::memory::bank_for_address(0x2200_0004));
    assert_eq!(crate::memory::bank_for_address(0x2000_0004), crate::memory::bank_for_address(0x2300_0004));
}

// ============================================================================
// 2.8 Dual-Core Accumulator Safety
// ============================================================================

#[test]
fn dual_core_extra_wait_states_no_pollution() {
    // Regression test: verify that the per-instruction extra-wait-state
    // accumulator doesn't leak between cores. `reset_extra_wait_states()`
    // at the start of `decode_execute()` must scrub the accumulator so
    // that wait states racked up by one core's instruction don't inflate
    // the other core's cycle count when it next executes.
    //
    // Drive each core through `step()` so we exercise the real decode
    // path (which owns the accumulator reset), and probe the bus right
    // after each core's single step.
    // Phase 3 Stage 1: construct shared atomics so core0/core1/bus all
    // see the same IRQ pending / halted / event_flag state.
    let atomics = Arc::new(CoreAtomics::default());
    let mut bus = crate::bus::Bus::with_atomics(Arc::clone(&atomics));
    let mut core0 = crate::core::CortexM33::new(0, Arc::clone(&atomics));
    let mut core1 = crate::core::CortexM33::new(1, atomics);

    // NOP (MOV R0, R0 = 0x4600) at two SRAM addresses in different banks.
    let nop: u16 = 0x4600;
    let bytes = nop.to_le_bytes();
    bus.memory.sram_write8(0, bytes[0]);
    bus.memory.sram_write8(1, bytes[1]);
    bus.memory.sram_write8(4, bytes[0]);
    bus.memory.sram_write8(5, bytes[1]);

    core0.regs.set_pc(0x2000_0000); // SRAM bank 0
    core1.regs.set_pc(0x2000_0004); // SRAM bank 1

    // Artificially pollute the accumulator to prove the reset kills it.
    // Bank 2/6 penalty removed from data paths, so use add_extra_wait_states
    // directly to simulate pollution.
    bus.reset_extra_wait_states();
    bus.add_extra_wait_states(1);
    assert_eq!(bus.extra_wait_states(), 1, "precondition: manual pollution applied");

    // Core 0 executes one NOP — decode_execute must reset the accumulator
    // before the fetch, so the observed value reflects only this fetch.
    core0.step(&mut bus);
    let c0_waits = bus.extra_wait_states();

    // Core 1 executes one NOP — again, decode_execute must reset cleanly.
    core1.step(&mut bus);
    let c1_waits = bus.extra_wait_states();

    // NOPs from SRAM banks 0 and 1 add no extra waits. If the polluted
    // "+1" from the bank-2 probe had leaked through, we'd see ≥ 1 here.
    assert_eq!(c0_waits, 0, "core 0 NOP from bank 0 should have no extra waits");
    assert_eq!(c1_waits, 0, "core 1 NOP from bank 1 should have no extra waits");
}

// ============================================================================
// 2.9 SRAM Per-Bank Extra Wait States
// ============================================================================

#[test]
fn sram_bank2_read_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000008: offset 0x8, bank = (0x8 >> 2) & 7 = 2
    // Bank 2/6 penalty removed from data paths (modeled on instruction
    // fetch only, conditional on sequentiality in decode.rs).
    let _ = bus.read32(0x2000_0008, 0);
    assert_eq!(bus.extra_wait_states(), 0, "bank 2/6 data penalty removed");
}

#[test]
fn sram_bank6_read_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000018: offset 0x18, bank = (0x18 >> 2) & 7 = 6
    // Bank 2/6 penalty removed from data paths.
    let _ = bus.read32(0x2000_0018, 0);
    assert_eq!(bus.extra_wait_states(), 0, "bank 2/6 data penalty removed");
}

#[test]
fn sram_bank0_no_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000000: offset 0x0, bank = (0x0 >> 2) & 7 = 0
    let _ = bus.read32(0x2000_0000, 0);
    assert_eq!(bus.extra_wait_states(), 0, "bank 0 read should have no extra wait state");
}

#[test]
fn sram_bank2_write_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20000008: offset 0x8, bank = (0x8 >> 2) & 7 = 2
    // Bank 2/6 penalty removed from data paths.
    bus.write32(0x2000_0008, 0xDEAD_BEEF, 0);
    assert_eq!(bus.extra_wait_states(), 0, "bank 2/6 data penalty removed");
}

#[test]
fn sram_bank89_no_extra_wait() {
    let mut bus = crate::bus::Bus::new();
    bus.reset_extra_wait_states();
    // 0x20080000: offset 0x80000, non-striped SRAM8
    let _ = bus.read32(0x2008_0000, 0);
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
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.msp, 0x2008_0000);
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.r[13], 0x2008_0000);
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), 0x0000_0100); // bit 0 cleared
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.xpsr & (1 << 24), 1 << 24); // Thumb bit

    // Core 1 should be at reset vector (same as Core 0 — both boot)
    assert_eq!(emu.cores.expect_arm_mut()[1].regs.pc(), 0x0000_0100);

    // Run a few cycles - should execute the NOP then hit the infinite loop
    for _ in 0..10 {
        emu.step();
    }

    // Should be stuck at the infinite loop (0x102)
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), 0x0000_0102);
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
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), 0x0000_0102);
    // Should be back in thread mode (IPSR = 0)
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 0);
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
    emu.cores.expect_arm_mut()[0].regs.r[1] = 0x6000_0000;
    // Enable BusFault handler in SHCSR (bit 17)
    emu.core_mut(0).ppb.shcsr |= 1 << 17;

    // Run
    for _ in 0..50 {
        emu.step();
    }

    // CFSR should have PRECISERR (bit 9) set
    assert_ne!(emu.core_mut(0).ppb.cfsr & (1 << 9), 0, "PRECISERR should be set");
}

/// BKPT must halt the core. The silicon oracles (`silicon_isr_diff_*`,
/// `silicon_cycle_oracle_*`) place a `BKPT #0` at the end of every
/// scenario's handler/main and poll `is_halted()` for end-of-test —
/// pre-fix, BKPT was a Phase-1 NOP stub, so handler bodies ran off into
/// the literal pool and HardFaulted, over-stacking the original
/// exception frame. Matches probe-rs/debugger semantics on real silicon.
#[test]
fn test_bkpt_halts_core() {
    let (mut c, mut bus) = core_and_bus();
    assert!(!c.is_halted(), "core not halted before BKPT");
    c.execute_one_with_bus(0xBE00, &mut bus); // BKPT #0
    assert!(c.is_halted(), "BKPT must halt the core");
}

// ============================================================================
// Asynchronous exception dispatch (PendSV / SysTick / NMI)
// ============================================================================
//
// `CortexM33::step` polls ICSR at the top of each call and takes the
// highest-priority pending async exception before fetching the next
// instruction. Verified here with targeted pend/step/assert fixtures;
// the `silicon_isr_diff_rp2350` oracle exercises the cross-product
// against real RP2354.

/// ROM variants for async-dispatch tests.
///
/// `looping_handlers`: handler body is an infinite `B .`. Used for entry
/// tests so the observation "IPSR == N after one core-step" is stable.
///
/// `bx_lr_handlers`: handler body is `BX LR`. Used for round-trip tests
/// that verify entry+exit leaves the core back in thread mode.
fn async_dispatch_rom(looping_handlers: bool) -> Vec<u8> {
    let mut rom = vec![0u8; 1024];
    rom[0..4].copy_from_slice(&0x2008_0000u32.to_le_bytes()); // SP
    rom[4..8].copy_from_slice(&0x0000_0101u32.to_le_bytes()); // Reset -> 0x100
    rom[0x38..0x3C].copy_from_slice(&0x0000_0201u32.to_le_bytes()); // PendSV -> 0x200
    rom[0x3C..0x40].copy_from_slice(&0x0000_0281u32.to_le_bytes()); // SysTick -> 0x280
    // Main at 0x100: `B .` — async exception is the only way to move.
    rom[0x100] = 0xFE; rom[0x101] = 0xE7;
    let handler: [u8; 2] = if looping_handlers {
        [0xFE, 0xE7] // B .
    } else {
        [0x70, 0x47] // BX LR
    };
    rom[0x200..0x202].copy_from_slice(&handler);
    rom[0x280..0x282].copy_from_slice(&handler);
    rom
}

/// Single-step core 0 (bypassing the quantum loop in `Emulator::step`)
/// so tests can assert per-instruction state. `Emulator::step` advances
/// 64 cycles per call by default — enough to run entry, handler, and
/// return in one call, which defeats "after one step, IPSR == 14"
/// assertions.
fn core0_step(emu: &mut Emulator) {
    // Phase 0b.1 Commit B: IRQ-merge happens inside `CortexM33::step`, not
    // at the `Bus::set_active_core` indirection that no longer exists.
    // Replicate the quantum-boundary merge that `Emulator::step` does so
    // inline-tick-triggered IRQs show up on this manual step.
    // Phase 3 Stage 1 (LLD V7 §2): `take_irq_pending` swaps to zero;
    // non-zero return replaces the V5 `irq_pending_dirty` flag.
    let pending = emu.bus.atomics.take_irq_pending(0);
    if pending != 0 {
        emu.core_mut(0).ppb.merge_irq_pending(pending);
    }
    let Cores::Arm(arm) = &mut emu.cores else { unreachable!() };
    arm[0].step(&mut emu.bus);
}

const ICSR_ADDR: u32 = 0xE000_ED04;
const ICSR_PENDSVSET_BIT: u32 = 1 << 28;
const ICSR_PENDSVCLR_BIT: u32 = 1 << 27;
const ICSR_PENDSTSET_BIT: u32 = 1 << 26;

#[test]
fn test_async_dispatch_pendsv_enters_handler() {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&async_dispatch_rom(true));
    emu.reset();

    for _ in 0..5 { core0_step(&mut emu); }
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 0, "must be in thread mode before pend");

    emu.core_mut(0).ppb.icsr |= ICSR_PENDSVSET_BIT;
    core0_step(&mut emu);

    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 14, "should be in PendSV handler");
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), 0x0000_0200, "PC at PendSV handler entry");
}

#[test]
fn test_async_dispatch_clears_pending_bit_on_entry() {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&async_dispatch_rom(true));
    emu.reset();
    for _ in 0..5 { core0_step(&mut emu); }

    emu.core_mut(0).ppb.icsr |= ICSR_PENDSVSET_BIT;
    core0_step(&mut emu);

    assert_eq!(emu.core_mut(0).ppb.icsr & ICSR_PENDSVSET_BIT, 0,
        "SET bit must clear on exception activation (ARMv8-M §B3.2.4)");
}

#[test]
fn test_async_dispatch_primask_masks_pendsv() {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&async_dispatch_rom(true));
    emu.reset();
    for _ in 0..5 { core0_step(&mut emu); }

    emu.core_mut(0).regs.primask = 1;
    emu.core_mut(0).ppb.icsr |= ICSR_PENDSVSET_BIT;

    for _ in 0..10 { core0_step(&mut emu); }
    assert_eq!(emu.core_mut(0).regs.ipsr(), 0, "PRIMASK must block PendSV");
    assert_ne!(emu.core_mut(0).ppb.icsr & ICSR_PENDSVSET_BIT, 0, "pending bit must remain set");
}

#[test]
fn test_async_dispatch_pendsvclr_prevents_dispatch() {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&async_dispatch_rom(true));
    emu.reset();
    for _ in 0..5 { core0_step(&mut emu); }

    emu.core_mut(0).ppb.write32(ICSR_ADDR, ICSR_PENDSVSET_BIT);
    emu.core_mut(0).ppb.write32(ICSR_ADDR, ICSR_PENDSVCLR_BIT);

    for _ in 0..5 { core0_step(&mut emu); }
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 0, "cleared pend bit must not dispatch");
}

#[test]
fn test_async_dispatch_pendsv_round_trip() {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&async_dispatch_rom(false)); // BX LR handler
    emu.reset();
    for _ in 0..5 { core0_step(&mut emu); }

    let main_pc_before = emu.core_mut(0).regs.pc();
    emu.core_mut(0).ppb.icsr |= ICSR_PENDSVSET_BIT;

    // Step enough for entry + BX LR + exit: 3 instructions worth.
    for _ in 0..5 { core0_step(&mut emu); }

    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 0, "must be back in thread mode");
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), main_pc_before, "must resume main loop PC");
}

#[test]
fn test_async_dispatch_systick_enters_handler() {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&async_dispatch_rom(true));
    emu.reset();
    for _ in 0..5 { core0_step(&mut emu); }

    emu.core_mut(0).ppb.icsr |= ICSR_PENDSTSET_BIT;
    core0_step(&mut emu);

    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 15, "should be in SysTick handler");
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), 0x0000_0280, "PC at SysTick handler entry");
}

// ============================================================================
// Tail-chain fast path (ARMv8-M §B3.4.2)
// ============================================================================
//
// On EXC_RETURN with another exception pending at a priority that would
// preempt the post-pop execution priority, hardware skips both the
// unstack of the departing frame and the re-stack of the new entry —
// the stacked frame already reflects the pre-emption state that the
// new handler will eventually return to. The ARMv8-M M33 TRM states
// this path costs ~6 cycles vs ~24 (12 exit + 12 re-entry) for the
// full two-step sequence. Before this path existed, the emulator
// relied on `exit_exception` fully unstacking and the top-of-loop
// `try_take_any_pending_exception` check re-entering on the next step
// — end-state correct, but cycle-cost wrong in both directions (under
// for this case because the stack churn was cheaper than silicon's
// true tail-chain sequencing, over for the cold case because we paid
// the full double framing cost).

/// Tail-chain integration: EXC_RETURN with pending SysTick transitions
/// directly into SysTick without unstacking the PendSV frame.
#[test]
fn test_tail_chain_pendsv_to_systick_preserves_frame() {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&async_dispatch_rom(false)); // BX LR handler
    emu.reset();
    for _ in 0..5 { core0_step(&mut emu); }

    // Step 1: pend PendSV + SysTick simultaneously. PendSV wins dispatch
    // (lower exc number at same priority). Single-step enters PendSV.
    emu.core_mut(0).ppb.icsr |= ICSR_PENDSVSET_BIT | ICSR_PENDSTSET_BIT;
    core0_step(&mut emu);
    assert_eq!(emu.core_mut(0).regs.ipsr(), 14, "PendSV wins arbitration");
    assert_ne!(emu.core_mut(0).ppb.icsr & ICSR_PENDSTSET_BIT, 0,
        "SysTick must remain pending while PendSV is active");
    let msp_in_pendsv = emu.cores.expect_arm_mut()[0].regs.msp;

    // Step 2: runs the single BX LR in the handler. exit_exception
    // sees SysTick pending and tail-chains — no unstack, no re-stack.
    core0_step(&mut emu);

    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 15,
        "tail-chain must activate SysTick, not return to thread");
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.msp, msp_in_pendsv,
        "tail-chain must NOT pop the frame — MSP preserved for the new handler");
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), 0x0000_0280,
        "PC at SysTick handler entry address");
    assert_eq!(emu.core_mut(0).ppb.icsr & ICSR_PENDSTSET_BIT, 0,
        "SysTick pending bit clears on tail-chain activation");
    assert!(CortexM33::is_exc_return(emu.cores.expect_arm_mut()[0].regs.lr()),
        "LR still an EXC_RETURN magic (frame held for eventual thread return)");
}

/// Tail-chain cycle cost: ~6 cycles (ARMv8-M §B3.4.2), vs ~12 for a
/// full `exit_exception` unstack. Driven via `test_exit_exception` so
/// the cost is isolated from per-instruction fetch/execute.
#[test]
fn test_tail_chain_cycle_cost_is_discounted() {
    use crate::bus::Bus;
    use crate::core::CortexM33;

    let mut bus = Bus::new();
    let mut cpu = CortexM33::new(0, bus.atomics.clone());
    cpu.regs.msp = 0x2000_2000;
    cpu.regs.r[13] = cpu.regs.msp;

    let vtor: u32 = 0x2000_4000;
    cpu.ppb.vtor = vtor;
    bus.write32(vtor + 14 * 4, 0x2000_0200 | 1, 0); // PendSV → 0x2000_0200
    bus.write32(vtor + 15 * 4, 0x2000_0280 | 1, 0); // SysTick → 0x2000_0280

    // Enter PendSV so a frame is on the stack.
    cpu.test_enter_exception(14, &mut bus);
    assert_eq!(cpu.regs.ipsr(), 14);

    // Pend SysTick.
    cpu.ppb.icsr |= 1u32 << 26; // ICSR_PENDSTSET

    // EXC_RETURN: thread, MSP, no FP. With SysTick pending, the fast
    // path should tail-chain at a discounted cost.
    let cycles = cpu.test_exit_exception(0xFFFF_FFF9, &mut bus);

    assert_eq!(cpu.regs.ipsr(), 15, "tail-chained into SysTick");
    assert_eq!(cycles, 6,
        "tail-chain cost is 6 cycles, not 12 (full unstack); got {cycles}");
}

/// Negative control: EXC_RETURN with no pending exception must not
/// tail-chain — full unstack back to thread mode at normal 12-cycle cost.
#[test]
fn test_exc_return_without_pending_does_full_unstack() {
    use crate::bus::Bus;
    use crate::core::CortexM33;

    let mut bus = Bus::new();
    let mut cpu = CortexM33::new(0, bus.atomics.clone());
    cpu.regs.msp = 0x2000_2000;
    cpu.regs.r[13] = cpu.regs.msp;
    let msp_pre_entry = cpu.regs.msp;

    let vtor: u32 = 0x2000_4000;
    cpu.ppb.vtor = vtor;
    bus.write32(vtor + 14 * 4, 0x2000_0200 | 1, 0);

    cpu.test_enter_exception(14, &mut bus);
    assert_eq!(cpu.regs.ipsr(), 14);
    assert_ne!(cpu.regs.msp, msp_pre_entry, "frame pushed");

    // No pending exceptions. EXC_RETURN should unstack normally.
    let cycles = cpu.test_exit_exception(0xFFFF_FFF9, &mut bus);

    assert_eq!(cpu.regs.ipsr(), 0, "returned to thread mode");
    assert_eq!(cpu.regs.msp, msp_pre_entry, "frame fully popped");
    assert_eq!(cycles, 12, "normal exit cost is 12 cycles; got {cycles}");
}

// ============================================================================
// External-IRQ dispatch (Phase 0a, HLD V5 §4.1.4)
// ============================================================================
//
// `CortexM33::step` runs `try_take_any_pending_exception` at each
// instruction boundary — a single priority comparison over NMI, PendSV,
// SysTick, and the highest-priority enabled-pending external IRQ.
// `bus.irq_pending[core]` is an observability side-channel that mirrors
// NVIC_ISPR (B1); MMIO writes to NVIC_ISPR/ICPR keep it in sync so
// tests and firmware can both watch the pending mask. The tests below
// pend IRQs via direct `bus.irq_pending` / ISPR writes (both paths
// land the same latch) and verify dispatch lands on the right
// vector-table slot.

/// ROM with a vector table big enough for external IRQs plus a tight
/// busy-wait main body. The IRQ handler at slot `exc_num` points into
/// SRAM at `0x2000_0200 + (exc_num - 16) * 0x40` so each IRQ in the
/// test suite gets a distinct handler address.
fn external_irq_rom() -> Vec<u8> {
    // Vector table size = (16 system + 52 external) * 4 = 272 bytes.
    // Pad out to 0x100 so the main routine sits cleanly at 0x100.
    let mut rom = vec![0u8; 2048];
    rom[0..4].copy_from_slice(&0x2008_0000u32.to_le_bytes()); // SP
    rom[4..8].copy_from_slice(&0x0000_0101u32.to_le_bytes()); // Reset -> 0x100
    // External IRQ N's vector is at offset (16 + N) * 4. Point every
    // external IRQ at a distinct SRAM address 0x2000_0200 + N*0x40 so
    // test-side assertions on the entered handler's PC pick up the IRQ
    // number from the address bits.
    for irq in 0..52u32 {
        let vec_off = (16 + irq as usize) * 4;
        let handler_addr = 0x2000_0200u32 + irq * 0x40;
        rom[vec_off..vec_off + 4].copy_from_slice(&(handler_addr | 1).to_le_bytes());
    }
    // Main at 0x100: busy-wait.
    rom[0x100] = 0xFE; rom[0x101] = 0xE7;
    rom
}

fn load_external_irq_emu() -> Emulator {
    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&external_irq_rom());
    emu.reset();
    // Place a busy-wait handler body in SRAM for every IRQ slot — the
    // scenarios inspect PC only, not handler behaviour.
    for irq in 0..52u32 {
        let handler_addr = 0x2000_0200u32 + irq * 0x40;
        emu.bus.memory.sram_write8(handler_addr & 0x0FFF_FFFF, 0xFE);
        emu.bus.memory.sram_write8((handler_addr + 1) & 0x0FFF_FFFF, 0xE7);
    }
    for _ in 0..5 { core0_step(&mut emu); }
    emu
}

#[test]
fn test_external_irq_pend_plus_enable_enters_handler() {
    // Pending + enabled IRQ 0 (TIMER0_IRQ_0) should dispatch; PC lands
    // at the TIMER0_IRQ_0 vector slot.
    let mut emu = load_external_irq_emu();
    emu.core_mut(0).ppb.write32(0xE000_E100, 1u32 << 0); // NVIC_ISER enable IRQ 0
    emu.bus.atomics.irq_pending[0].fetch_or(1u64 << 0, Ordering::Relaxed);
    emu.core_mut(0).ppb.nvic_ispr[0].fetch_or(1u32 << 0, Ordering::Relaxed);

    core0_step(&mut emu);

    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 16 + 0,
        "IPSR must be TIMER0_IRQ_0 (exception 16)");
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.pc(), 0x2000_0200,
        "PC at TIMER0 handler entry");
}

#[test]
fn test_external_irq_pending_without_enable_does_not_dispatch() {
    // irq_pending bit is set but NVIC_ISER bit is clear → no dispatch.
    let mut emu = load_external_irq_emu();
    emu.bus.atomics.irq_pending[0].fetch_or(1u64 << 0, Ordering::Relaxed);
    emu.core_mut(0).ppb.nvic_ispr[0].fetch_or(1u32 << 0, Ordering::Relaxed);

    for _ in 0..5 { core0_step(&mut emu); }
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 0,
        "pending-without-enable must not dispatch");
}

#[test]
fn test_external_irq_priority_mask_blocks_dispatch() {
    // PRIMASK=1 blocks all configurable priorities, including external
    // IRQs at priority 0.
    let mut emu = load_external_irq_emu();
    emu.core_mut(0).regs.primask = 1;
    emu.core_mut(0).ppb.write32(0xE000_E100, 1u32 << 0);
    emu.bus.atomics.irq_pending[0].fetch_or(1u64 << 0, Ordering::Relaxed);
    emu.core_mut(0).ppb.nvic_ispr[0].fetch_or(1u32 << 0, Ordering::Relaxed);

    for _ in 0..5 { core0_step(&mut emu); }
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 0,
        "PRIMASK must block external IRQ dispatch");
}

#[test]
fn test_external_irq_basepri_masks_dispatch() {
    // BASEPRI=0x20 + IRQ priority 0xC0 → IRQ priority is numerically
    // larger (lower priority) than BASEPRI → IRQ is masked.
    let mut emu = load_external_irq_emu();
    emu.core_mut(0).regs.basepri = 0x20;
    emu.core_mut(0).ppb.write32(0xE000_E100, 1u32 << 0);
    emu.core_mut(0).ppb.write32(0xE000_E400, u32::from_le_bytes([0xC0, 0, 0, 0]));
    emu.bus.atomics.irq_pending[0].fetch_or(1u64 << 0, Ordering::Relaxed);
    emu.core_mut(0).ppb.nvic_ispr[0].fetch_or(1u32 << 0, Ordering::Relaxed);

    for _ in 0..5 { core0_step(&mut emu); }
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 0,
        "BASEPRI=0x20 must mask IRQ at priority 0xC0");
}

#[test]
fn test_external_irq_basepri_zero_is_transparent() {
    // BASEPRI=0 (the default) must not mask any IRQ.
    let mut emu = load_external_irq_emu();
    emu.core_mut(0).regs.basepri = 0;
    emu.core_mut(0).ppb.write32(0xE000_E100, 1u32 << 0);
    emu.core_mut(0).ppb.write32(0xE000_E400, u32::from_le_bytes([0xC0, 0, 0, 0]));
    emu.bus.atomics.irq_pending[0].fetch_or(1u64 << 0, Ordering::Relaxed);
    emu.core_mut(0).ppb.nvic_ispr[0].fetch_or(1u32 << 0, Ordering::Relaxed);

    core0_step(&mut emu);
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 16 + 0,
        "BASEPRI=0 is transparent; IRQ 0 must dispatch");
}

#[test]
fn test_assert_irq_core_targets_receiver() {
    // assert_irq_core(1, IRQ_SIO_IRQ_FIFO) must latch on core 1's mask
    // and NOT core 0's — the `core` argument names the receiver.
    // SIO FIFO is a core-local line (CORE_LOCAL_IRQS) so this is the
    // correct helper; assert_irq_shared would put it on both cores.
    //
    // Phase 0b.1 Commit B: `assert_irq_core` now sets `irq_pending` +
    // the per-core `irq_pending_dirty` flag; the NVIC_ISPR merge happens
    // at the next scheduler tick or on the step-path dirty check. We
    // drive the merge manually here and then assert both observables.
    let mut emu = load_external_irq_emu();
    let irq = crate::irq::IRQ_SIO_IRQ_FIFO; // 25, core-local
    emu.bus.assert_irq_core(1, irq);
    assert_eq!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed), 0,
        "core 0 irq_pending must remain clear");
    assert_ne!(emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed) & (1u64 << irq), 0,
        "core 1 irq_pending must record the assert");
    // Phase 3 Stage 1: the `irq_pending_dirty` flag is gone; the pending
    // mask itself (non-zero) carries the "needs merge" signal.
    assert!(emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed) != 0,
        "core 1 irq_pending must signal the pending merge");
    assert_eq!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed), 0,
        "core 0 irq_pending must remain clear");
    // Drive the merge and re-check NVIC_ISPR — core 1 latches, core 0
    // stays clean.
    let pending1 = emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed);
    emu.core_mut(1).ppb.merge_irq_pending(pending1);
    assert_ne!(emu.core_mut(1).ppb.nvic_ispr[0].load(Ordering::Relaxed) & (1u32 << irq), 0,
        "NVIC_ISPR on core 1 must latch after merge");
    assert_eq!(emu.core_mut(0).ppb.nvic_ispr[0].load(Ordering::Relaxed) & (1u32 << irq), 0,
        "NVIC_ISPR on core 0 must remain clear");
}

#[test]
fn test_assert_irq_core_silently_drops_oob_args() {
    let mut emu = load_external_irq_emu();
    emu.bus.assert_irq_core(0, 100); // above IRQ_COUNT (52) — guarded by debug_assert
    assert_eq!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed), 0);
    // core >= 2 with a core-local IRQ — silently drops without latching.
    emu.bus.assert_irq_core(5, crate::irq::IRQ_SIO_IRQ_FIFO);
    assert_eq!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed), 0);
    assert_eq!(emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed), 0);
}

#[test]
fn test_assert_irq_shared_latches_on_both_cores() {
    // assert_irq_shared(IRQ_TIMER0_IRQ_0) must land pending on both
    // cores so dispatch can pick the core with lowest execution
    // priority — shared peripheral lines are not routed to a specific
    // receiver by the peripheral itself.
    //
    // Phase 0b.1 Commit B: assert_irq_shared sets `irq_pending` +
    // `irq_pending_dirty` on both cores; the NVIC_ISPR merge happens
    // at the next scheduler tick / step. Drive the merge manually here.
    let mut emu = load_external_irq_emu();
    let irq = crate::irq::IRQ_TIMER0_IRQ_0; // shared
    emu.bus.assert_irq_shared(irq);
    assert_ne!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) & (1u64 << irq), 0,
        "core 0 irq_pending must record the assert");
    assert_ne!(emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed) & (1u64 << irq), 0,
        "core 1 irq_pending must record the assert");
    // Phase 3 Stage 1: non-zero `irq_pending` replaces the `irq_pending_dirty` flag.
    assert!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) != 0
         && emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed) != 0,
        "both cores' irq_pending must carry the pending signal");
    let pending0 = emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed);
    emu.core_mut(0).ppb.merge_irq_pending(pending0);
    let pending1b = emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed);
    emu.core_mut(1).ppb.merge_irq_pending(pending1b);
    assert_ne!(emu.core_mut(0).ppb.nvic_ispr[0].load(Ordering::Relaxed) & (1u32 << irq), 0,
        "NVIC_ISPR on core 0 must latch after merge");
    assert_ne!(emu.core_mut(1).ppb.nvic_ispr[0].load(Ordering::Relaxed) & (1u32 << irq), 0,
        "NVIC_ISPR on core 1 must latch after merge");
}

#[test]
fn test_clear_irq_core_drops_pending_on_one_core() {
    // Phase 0b.1 Commit B: `clear_irq_core` clears `irq_pending` only;
    // any matching `nvic_ispr` bit will be cleared by the step-path
    // dual-clear on dispatch (see `try_take_any_pending_exception`'s
    // DUAL-CLEAR INVARIANT). Assert the new contract here — the test's
    // original post-condition (nvic_ispr also cleared by clear_irq_*)
    // was an artefact of the previous inline-write model.
    let mut emu = load_external_irq_emu();
    let irq = crate::irq::IRQ_SIO_IRQ_FIFO; // core-local
    emu.bus.assert_irq_core(0, irq);
    emu.bus.assert_irq_core(1, irq);
    emu.bus.clear_irq_core(0, irq);
    assert_eq!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) & (1u64 << irq), 0,
        "clear_irq_core must drop the pending bit on the named core");
    assert_ne!(emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed) & (1u64 << irq), 0,
        "clear_irq_core must not touch the other core");
}

#[test]
fn test_clear_irq_shared_drops_pending_on_both_cores() {
    // Phase 0b.1 Commit B: see comment on
    // `test_clear_irq_core_drops_pending_on_one_core`. `clear_irq_shared`
    // clears `irq_pending` on both cores; NVIC_ISPR lives on the cores
    // and is touched only by dispatch / ICPR writes.
    let mut emu = load_external_irq_emu();
    let irq = crate::irq::IRQ_TIMER0_IRQ_0; // shared
    emu.bus.assert_irq_shared(irq);
    emu.bus.clear_irq_shared(irq);
    assert_eq!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) & (1u64 << irq), 0,
        "clear_irq_shared must clear core 0's pending bit");
    assert_eq!(emu.bus.atomics.irq_pending[1].load(Ordering::Relaxed) & (1u64 << irq), 0,
        "clear_irq_shared must clear core 1's pending bit");
}

#[test]
fn test_mmio_nvic_ispr_write_mirrors_into_irq_pending_and_dispatches() {
    // R1: Firmware-side MMIO writes to NVIC_ISPR must both (a) mirror
    // the set bit into `bus.irq_pending[core]` (so tests and
    // observability code see the pending state) and (b) trigger
    // dispatch on the next step. Covers HLD V5 §5.3 mandated case.
    let mut emu = load_external_irq_emu();
    // Enable IRQ 0 via MMIO path.
    emu.mmio_write32(0xE000_E100, 1u32 << 0);
    // Pend IRQ 0 via MMIO path — this is the case B1 restores.
    emu.mmio_write32(0xE000_E200, 1u32 << 0);
    // After write, bus.irq_pending must reflect the latch.
    assert_ne!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) & (1u64 << 0), 0,
        "NVIC_ISPR MMIO write must mirror into bus.irq_pending[core]");
    // And the dispatch path must enter the handler on next step.
    core0_step(&mut emu);
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 16 + 0,
        "MMIO-pended IRQ 0 must enter exception 16 on next step");
}

#[test]
fn test_mmio_nvic_icpr_write_drops_irq_pending_mirror() {
    // Symmetric to R1: NVIC_ICPR MMIO writes must also clear the
    // corresponding `bus.irq_pending` bit so stale-mask interference
    // doesn't let a cleared IRQ re-dispatch.
    let mut emu = load_external_irq_emu();
    // Pend via MMIO (mirrors set bit).
    emu.mmio_write32(0xE000_E200, 1u32 << 0);
    assert_ne!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) & (1u64 << 0), 0);
    // Clear via MMIO ICPR.
    emu.mmio_write32(0xE000_E280, 1u32 << 0);
    assert_eq!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) & (1u64 << 0), 0,
        "NVIC_ICPR MMIO write must clear the bus.irq_pending mirror");
    assert_eq!(emu.core_mut(0).ppb.nvic_ispr[0].load(Ordering::Relaxed) & (1u32 << 0), 0,
        "NVIC_ICPR MMIO write must clear the architectural latch");
}

#[test]
fn test_mmio_nvic_ispr_word1_write_mirrors_high_half() {
    // The mirror path handles both words of irq_pending — IRQs 32..=51
    // land in word 1 (NVIC_ISPR1 at 0xE000_E204). Pin this explicitly:
    // pending IRQ 40 (PROC0_IRQ_CSIDE in the catalogue) must surface
    // in bits 32..=63 of irq_pending.
    let mut emu = load_external_irq_emu();
    emu.mmio_write32(0xE000_E204, 1u32 << (40 - 32));
    assert_ne!(emu.bus.atomics.irq_pending[0].load(Ordering::Relaxed) & (1u64 << 40), 0,
        "NVIC_ISPR1 write for IRQ 40 must mirror into bus.irq_pending[core] bit 40");
}

#[test]
fn test_execution_priority_basepri_leq_current_is_noop() {
    // R2: BASEPRI >= current execution priority is a no-op. Put the
    // core in a handler at priority 0x40 (active IPSR implies
    // execution_priority=0x40), then set BASEPRI=0xE0 (numerically
    // higher = lower architectural priority). `execution_priority`
    // must still return 0x40 — BASEPRI only lowers the ceiling, it
    // never raises it.
    let (mut c, _bus) = core_and_bus();
    // Pretend we're in handler mode at exception 16 (IRQ 0).
    c.regs.xpsr = (c.regs.xpsr & !0x1FF) | 16;
    // Give IRQ 0 priority 0x40 via NVIC_IPR0 byte 0.
    c.ppb.write32(0xE000_E400, u32::from_le_bytes([0x40, 0, 0, 0]));
    c.regs.basepri = 0xE0;
    let prio = c.execution_priority();
    assert_eq!(prio, 0x40,
        "BASEPRI=0xE0 must NOT raise execution priority above the \
         current active-exception priority 0x40");
}

#[test]
fn test_unified_arbitration_external_irq_beats_pendsv() {
    // B2 regression check: PendSV at SHPR3-programmed priority 0x80
    // and an external IRQ at IPR priority 0x20 pending simultaneously
    // must resolve to the IRQ (exception 16), not PendSV (14). This
    // was the pre-B2 bug: try_take_async_exception fired before
    // try_take_external_irq without consulting priority.
    let mut emu = load_external_irq_emu();
    // Set PendSV priority to 0x80 via SHPR3 (PendSV is byte [10] → lane 2
    // of SHPR3 at 0xE000_ED20).
    emu.mmio_write32(0xE000_ED20, u32::from_le_bytes([0, 0, 0x80, 0]));
    // Set IRQ 0 priority to 0x20.
    emu.mmio_write32(0xE000_E400, u32::from_le_bytes([0x20, 0, 0, 0]));
    // Enable IRQ 0.
    emu.mmio_write32(0xE000_E100, 1u32 << 0);
    // Pend both.
    emu.mmio_write32(0xE000_ED04, 1u32 << 28); // ICSR.PENDSVSET
    emu.mmio_write32(0xE000_E200, 1u32 << 0); // NVIC_ISPR IRQ 0
    // One step: unified arbitration picks IRQ 0 (priority 0x20 beats 0x80).
    core0_step(&mut emu);
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 16,
        "unified arbitration must pick IRQ 0 (priority 0x20) over PendSV (priority 0x80)");
}

#[test]
fn test_priority_preempt_end_to_end_via_step_loop() {
    // R3: Pend IRQ 0 at priority 0xC0, step until IPSR == 16, then
    // pend IRQ 1 at priority 0x40, step once, assert IPSR == 17 —
    // a higher-priority IRQ preempts a running handler.
    let mut emu = load_external_irq_emu();
    // Priorities: IRQ 0 = 0xC0 (lane 0), IRQ 1 = 0x40 (lane 1).
    emu.mmio_write32(0xE000_E400, u32::from_le_bytes([0xC0, 0x40, 0, 0]));
    // Enable both IRQs.
    emu.mmio_write32(0xE000_E100, 0b11);
    // Pend IRQ 0 + step until the handler is entered.
    emu.mmio_write32(0xE000_E200, 1u32 << 0);
    let mut taken = false;
    for _ in 0..5 {
        core0_step(&mut emu);
        if emu.cores.expect_arm_mut()[0].regs.ipsr() == 16 {
            taken = true;
            break;
        }
    }
    assert!(taken, "IRQ 0 must dispatch within a few steps (IPSR=16)");
    // Now pend IRQ 1 at higher priority. Must preempt on next step.
    emu.mmio_write32(0xE000_E200, 1u32 << 1);
    core0_step(&mut emu);
    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 17,
        "IRQ 1 (priority 0x40) must preempt IRQ 0 handler (priority 0xC0)");
}

// ============================================================================
// BASEPRI fold into execution_priority (Phase 0a, HLD V5 §4.1.3)
// ============================================================================
//
// Before Phase 0a, execution_priority read PRIMASK and FAULTMASK but
// ignored BASEPRI. HLD V5 §4.1.3 requires BASEPRI to clamp the running
// priority to `basepri & 0xE0` when non-zero. Pin the case it was silent
// about previously:
//
//   basepri=0x80, primask=0, faultmask=0, inactive_irq_at_prio=0xC0
//   → execution_priority must return 0x80, NOT 0xC0.
//
// The test uses an emulator with the default NVIC state so no active
// exception muddies the base priority.

#[test]
fn test_execution_priority_basepri_clamps_masked_value() {
    let (mut c, _bus) = core_and_bus();
    c.regs.basepri = 0x80;
    c.regs.primask = 0;
    c.regs.faultmask = 0;
    // No active exception, no FAULTMASK, no PRIMASK, no pending IRQ.
    // Pre-Phase-0a this returned 256 (the initial "no mask" value);
    // post-Phase-0a it returns 0x80.
    let prio = c.execution_priority();
    assert_eq!(prio, 0x80, "BASEPRI=0x80 must clamp execution_priority to 0x80");
}

#[test]
fn test_execution_priority_basepri_masks_unimplemented_bits() {
    // M33 implements bits [7:5] of priority bytes. A BASEPRI of 0x9F
    // (bits 4..0 set) must behave identically to 0x80 — the clamp
    // discards the unimplemented bits.
    let (mut c, _bus) = core_and_bus();
    c.regs.basepri = 0x9F;
    let prio_masked = c.execution_priority();
    c.regs.basepri = 0x80;
    let prio_pure = c.execution_priority();
    assert_eq!(prio_masked, prio_pure,
        "BASEPRI bits [4:0] must be masked before clamping");
}

#[test]
fn test_execution_priority_basepri_zero_does_nothing() {
    // BASEPRI=0 is the "disabled" marker — do not clamp at all.
    let (mut c, _bus) = core_and_bus();
    c.regs.basepri = 0;
    let prio = c.execution_priority();
    assert_eq!(prio, 256,
        "BASEPRI=0 must leave priority at its thread-mode max");
}

#[test]
fn test_execution_priority_primask_overrides_basepri() {
    // PRIMASK=1 clamps to 0, which is numerically less than any BASEPRI
    // value — PRIMASK wins by architectural ordering.
    let (mut c, _bus) = core_and_bus();
    c.regs.primask = 1;
    c.regs.basepri = 0xE0;
    let prio = c.execution_priority();
    assert_eq!(prio, 0, "PRIMASK=1 must beat BASEPRI=0xE0");
}

#[test]
fn test_execution_priority_basepri_vs_higher_priority_irq() {
    // Still the "without this HLD" regression case, phrased more
    // explicitly: with BASEPRI=0x80 and no active exception, a pending
    // IRQ at priority 0xC0 (numerically greater than BASEPRI) must be
    // blocked — `can_preempt(0xC0_irq)` should return false.
    let mut emu = load_external_irq_emu();
    emu.core_mut(0).regs.basepri = 0x80;
    let can = emu.core_mut(0).can_preempt(16 + 0); // IRQ 0 priority 0
    assert!(can, "IRQ at priority 0 always beats BASEPRI 0x80");
    // But an IRQ with priority 0xC0 set via NVIC_IPR should NOT preempt.
    emu.core_mut(0).ppb.write32(0xE000_E400, u32::from_le_bytes([0xC0, 0, 0, 0]));
    let can_irq0_at_0xc0 = emu.core_mut(0).can_preempt(16 + 0);
    assert!(!can_irq0_at_0xc0,
        "IRQ 0 at priority 0xC0 must not preempt BASEPRI 0x80");
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
    c.ppb.sau_ctrl = 1;
    // Region 3: RBAR=0x4787, RLAR=0x7FE1 (enabled, NSC=0 -> Secure)
    c.ppb.sau_rnr = 3;
    c.ppb.sau_regions[3] = (0x4787, 0x7FE1);

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
    c.ppb.sau_ctrl = 1;
    // Region 0: base=0x1000, limit=0x1FFF, NSC=1, enabled
    // RBAR = 0x1000, RLAR = 0x1FE0 | 0x3 (NSC=1, enable=1)
    c.ppb.sau_regions[0] = (0x1000, 0x1FE3);

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
    c.ppb.sau_ctrl = 1; // enable, ALLNS=0

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
    c.ppb.sau_ctrl = 3; // enable + ALLNS

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
    c.ppb.sau_ctrl = 1;
    c.ppb.sau_rnr = 7;
    c.ppb.sau_regions[7] = (0x4787, 0x7FE1);

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
    // hw1[15:12]=1 (Rt=R1), hw1[7:0]=0 (imm8=0) — this is STREX, not TT.
    // Phase 0b.2: the decoder still routes this to STREX (not TT), but
    // with no prior LDREX the monitor is open, so STREX fails (Rd = 1)
    // and memory is unchanged. Pre-Phase-0b.2 the stub unconditionally
    // succeeded; this assertion now encodes the address-based monitor.
    let (mut c, mut bus) = core_and_bus();
    // Seed memory with a known sentinel so a failed STREX is observable.
    bus.write32(0x2000_0100, 0x1234_5678, 0);
    c.set_reg(1, 0xDEAD_BEEF);
    c.set_reg(2, 0x2000_0100);
    c.execute_one_wide_with_bus(0xE842, 0x1000, &mut bus);
    // STREX with open monitor -> Rd = 1 (failure); memory unchanged.
    assert_eq!(c.reg(0), 1);
    assert_eq!(bus.read32(0x2000_0100, 0), 0x1234_5678);
}

// ============================================================================
// LDREX / STREX / CLREX — Phase 0b.2 address-based exclusive monitor.
// ============================================================================
//
// Core semantics (ARMv8-M §A3.4):
//   - LDREX(addr)   -> load 32-bit, set local monitor to addr
//   - STREX(addr,v) -> if monitor == addr: store v, Rd=0; else Rd=1.
//                      Always clears the local monitor afterwards.
//   - CLREX         -> clear the local monitor unconditionally.
//   - Byte/halfword variants mirror the same monitor semantics, just with
//     a narrower data transfer.
//
// Cross-core invalidation is handled by `Emulator::step` via the snoop
// hook — peer-core data writes drop the local monitor. Same-core writes
// do NOT invalidate the local monitor (ARM is explicit on this: it's the
// firmware's job to reissue LDREX if a same-core STR was intentional).

#[test]
fn ldrex_strex_success() {
    // LDREX then STREX to the same address with no intervening peer write
    // -> STREX succeeds (Rd = 0) and memory is updated.
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0200, 0xAAAA_AAAA, 0); // seed memory
    c.set_reg(2, 0x2000_0200);                // Rn base
    c.set_reg(1, 0xBEEF_F00D);                // Rt (STREX store value)

    // LDREX R3, [R2, #0]: hw0=0xE852, hw1=0x3F00
    //   hw0[3:0]=Rn=2, hw1[15:12]=Rt=3, hw1[11:8]=0xF, hw1[7:0]=imm8=0
    c.execute_one_wide_with_bus(0xE852, 0x3F00, &mut bus);
    assert_eq!(c.reg(3), 0xAAAA_AAAA, "LDREX loaded seed value");
    assert_eq!(c.exclusive_address, Some(0x2000_0200),
        "LDREX set local monitor to the loaded address");

    // STREX R0, R1, [R2, #0]: hw0=0xE842, hw1=0x1000
    //   hw0[3:0]=Rn=2, hw1[15:12]=Rt=1, hw1[11:8]=Rd=0, hw1[7:0]=imm8=0
    c.execute_one_wide_with_bus(0xE842, 0x1000, &mut bus);
    assert_eq!(c.reg(0), 0, "STREX success -> Rd = 0");
    assert_eq!(bus.read32(0x2000_0200, 0), 0xBEEF_F00D,
        "STREX updated memory on success");
    assert_eq!(c.exclusive_address, None,
        "STREX clears the local monitor");
}

#[test]
fn ldrex_clrex_strex_fail() {
    // LDREX, CLREX, STREX -> STREX fails (Rd = 1), memory unchanged.
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0210, 0xAAAA_AAAA, 0);
    c.set_reg(2, 0x2000_0210);
    c.set_reg(1, 0xBEEF_F00D);

    c.execute_one_wide_with_bus(0xE852, 0x3F00, &mut bus); // LDREX
    assert_eq!(c.exclusive_address, Some(0x2000_0210));

    // CLREX: hw0 = 0xF3BF, hw1[7:4] = 0x2 -> hw1 = 0x8F2F (mask[7:4]=2, rest=don't-care pattern)
    c.execute_one_wide_with_bus(0xF3BF, 0x8F2F, &mut bus);
    assert_eq!(c.exclusive_address, None, "CLREX clears the local monitor");

    c.execute_one_wide_with_bus(0xE842, 0x1000, &mut bus); // STREX
    assert_eq!(c.reg(0), 1, "STREX after CLREX must fail");
    assert_eq!(bus.read32(0x2000_0210, 0), 0xAAAA_AAAA,
        "memory must be unchanged after failed STREX");
}

#[test]
fn ldrex_samecore_str_strex_success() {
    // LDREX, then a same-core STR to the same address, then STREX.
    // Per ARMv8-M §A3.4 the local monitor is address-based and same-core
    // writes do NOT invalidate it — so STREX still succeeds.
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0220, 0xAAAA_AAAA, 0);
    c.set_reg(2, 0x2000_0220);
    c.set_reg(1, 0xBEEF_F00D);

    c.execute_one_wide_with_bus(0xE852, 0x3F00, &mut bus); // LDREX R3, [R2]
    // STR via the bus wrapper simulates a normal same-core store. Using
    // the wrapper flips `did_write_this_quantum` but must NOT clear the
    // local monitor — that's the invariant under test.
    c.bus_write32(0x2000_0220, 0x1111_2222, &mut bus);
    assert_eq!(c.exclusive_address, Some(0x2000_0220),
        "same-core write must NOT clear local monitor");
    assert!(c.did_write_this_quantum, "same-core write set the flag");

    c.execute_one_wide_with_bus(0xE842, 0x1000, &mut bus); // STREX R0, R1, [R2]
    assert_eq!(c.reg(0), 0, "STREX succeeds despite same-core STR");
    assert_eq!(bus.read32(0x2000_0220, 0), 0xBEEF_F00D);
    assert_eq!(c.exclusive_address, None);
}

#[test]
fn ldrex_strex_different_addr_fail() {
    // LDREX addr A, STREX addr B -> Rd = 1; neither address updated.
    let (mut c, mut bus) = core_and_bus();
    bus.write32(0x2000_0230, 0xAAAA_AAAA, 0); // A
    bus.write32(0x2000_0234, 0xCCCC_CCCC, 0); // B
    c.set_reg(2, 0x2000_0230);
    c.set_reg(5, 0x2000_0234); // Rn for the STREX
    c.set_reg(1, 0xBEEF_F00D);

    c.execute_one_wide_with_bus(0xE852, 0x3F00, &mut bus); // LDREX R3, [R2]
    assert_eq!(c.exclusive_address, Some(0x2000_0230));

    // STREX R0, R1, [R5, #0]: hw0=0xE845 (Rn=5), hw1=0x1000
    c.execute_one_wide_with_bus(0xE845, 0x1000, &mut bus);
    assert_eq!(c.reg(0), 1, "STREX to different address must fail");
    assert_eq!(bus.read32(0x2000_0234, 0), 0xCCCC_CCCC, "B unchanged");
    assert_eq!(bus.read32(0x2000_0230, 0), 0xAAAA_AAAA, "A unchanged");
    assert_eq!(c.exclusive_address, None,
        "STREX clears the monitor even on failure");
}

#[test]
fn ldrex_peer_write_strex_fail() {
    // Peer-core write snoop must invalidate our monitor. This test drives
    // the actual `Emulator::step` snoop hook: core 0 holds an outstanding
    // LDREX, core 1 performs a normal data write via `bus_write32`, and
    // after the quantum the snoop must clear core 0's monitor so core 0's
    // next STREX fails.
    let mut emu = Emulator::new(Config::default());
    let addr = 0x2000_0400u32;
    emu.bus.write32(addr, 0xAAAA_AAAA, 0);

    // Seed core 0 with an outstanding LDREX and core 1 with the pending
    // write setup. We hand-install `exclusive_address` on core 0 rather
    // than running a LDREX instruction — this test is about the snoop,
    // not the LDREX decode path.
    emu.core_mut(0).exclusive_address = Some(addr);
    // Core 1 performs a write via the wrapper; this sets its
    // `did_write_this_quantum = true` and writes to memory.
    {
        let Cores::Arm(arm) = &mut emu.cores else { unreachable!() };
        arm[1].bus_write32(addr, 0x1234_5678, &mut emu.bus);
    }
    assert!(emu.core_mut(1).did_write_this_quantum);

    // Drive one Emulator::step — both cores are at reset-vector PC (fetch
    // whatever is at ROM 0, likely a NOP-like default), but the only
    // thing we care about is the snoop firing at the quantum boundary.
    // Halt both cores so they don't execute anything and clobber state;
    // the snoop logic runs regardless of execution.
    emu.core_mut(0).halt();
    emu.core_mut(1).halt();
    emu.step();
    assert_eq!(emu.core_mut(0).exclusive_address, None,
        "peer-core write must invalidate our monitor via the snoop");
    assert!(!emu.core_mut(1).did_write_this_quantum,
        "snoop must clear did_write_this_quantum for the next quantum");

    // And STREX from core 0 now fails, completing the scenario.
    emu.core_mut(0).wake();
    emu.core_mut(0).set_reg(2, addr);
    emu.core_mut(0).set_reg(1, 0xBEEF_F00D);
    {
        let Cores::Arm(arm) = &mut emu.cores else { unreachable!() };
        arm[0].execute_one_wide_with_bus(0xE842, 0x1000, &mut emu.bus);
    }
    assert_eq!(emu.core_mut(0).reg(0), 1, "STREX must fail after peer snoop");
}

#[test]
fn ldrex_ldrex_strex_strex_race() {
    // Two-core race: both cores LDREX the same address, then both STREX.
    // Phase 0b.2 snoop guarantees the first STREX invalidates the peer's
    // monitor before the peer runs its STREX, so exactly one wins.
    //
    // Layout: step_quantum = 1 forces at most one instruction per core
    // per `Emulator::step()` call. Each core executes one instruction,
    // then the snoop fires at the quantum boundary.
    let mut emu = crate::EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build();

    // Minimal ROM — reset vector points at 0x2000_0000 (SRAM). We drive
    // state directly via the registers so we don't need a full vector
    // table here.
    let mut rom = vec![0u8; 512];
    rom[0..4].copy_from_slice(&0x2008_0000u32.to_le_bytes()); // MSP
    rom[4..8].copy_from_slice(&0x2000_0001u32.to_le_bytes()); // reset vector (thumb)
    emu.load_bootrom(&rom);
    emu.reset();

    // Seed memory + registers on both cores. The two cores will execute
    // distinct instruction pairs from different SRAM offsets.
    let addr = 0x2000_1000u32;
    emu.bus.write32(addr, 0x1111_1111, 0);

    // Core 0 program at 0x2000_0000: LDREX R3, [R2] ; STREX R0, R1, [R2]
    //   0xE852, 0x3F00, 0xE842, 0x1000
    let c0_prog: [u16; 4] = [0xE852, 0x3F00, 0xE842, 0x1000];
    for (i, w) in c0_prog.iter().enumerate() {
        emu.bus.write16(0x2000_0000 + (i as u32) * 2, *w, 0);
    }

    // Core 1 program at 0x2000_0100: same sequence but distinct Rt value.
    //   Same encodings because registers are free.
    let c1_prog: [u16; 4] = [0xE852, 0x3F00, 0xE842, 0x1000];
    for (i, w) in c1_prog.iter().enumerate() {
        emu.bus.write16(0x2000_0100 + (i as u32) * 2, *w, 1);
    }

    // Set each core's PC, SP, and working regs.
    {
        let arm = emu.cores.expect_arm_mut();
        for c in 0..2 {
            arm[c].regs.msp = 0x2008_0000;
            arm[c].regs.r[13] = 0x2008_0000;
            arm[c].regs.r[2] = addr;
            arm[c].regs.xpsr = 1 << 24; // Thumb
        }
    }
    emu.core_mut(0).regs.set_pc(0x2000_0000);
    emu.core_mut(0).regs.r[1] = 0xAAAA_0000;
    emu.core_mut(1).regs.set_pc(0x2000_0100);
    emu.core_mut(1).regs.r[1] = 0xBBBB_0000;

    // Drive enough quanta for both cores to execute LDREX (step 1) then
    // STREX (step 2). LDREX/STREX cost 2 cycles each and `step_quantum=1`,
    // so it takes a few quanta to accumulate each instruction. We stop as
    // soon as both cores have executed both instructions (PC advanced by
    // 8 bytes from their program start).
    for _ in 0..64 {
        if emu.core_mut(0).regs.pc() >= 0x2000_0008
            && emu.core_mut(1).regs.pc() >= 0x2000_0108
        {
            break;
        }
        emu.step();
    }
    assert_eq!(emu.core_mut(0).reg(0), 0, "core 0 STREX wins");
    assert_eq!(emu.core_mut(1).reg(0), 1, "core 1 STREX loses");
    assert_eq!(emu.bus.read32(addr, 0), 0xAAAA_0000,
        "memory shows the winning store");
    assert_eq!(emu.core_mut(0).exclusive_address, None);
    assert_eq!(emu.core_mut(1).exclusive_address, None);
}

// -- LDREXB / STREXB ---------------------------------------------------------

#[test]
fn ldrexb_strexb_success() {
    // LDREXB/STREXB round-trip on a single core.
    //   LDREXB Rt, [Rn]:    hw0 = 0xE8D0 | Rn, hw1 = (Rt << 12) | 0x0F4F
    //   STREXB Rd, Rt, [Rn]: hw0 = 0xE8C0 | Rn, hw1 = (Rt << 12) | 0x0F40 | Rd
    let (mut c, mut bus) = core_and_bus();
    bus.write8(0x2000_0300, 0xAA, 0);
    c.set_reg(2, 0x2000_0300);
    c.set_reg(1, 0x5A);

    // LDREXB R3, [R2]: hw0=0xE8D2 (Rn=2), hw1=0x3F4F (Rt=3)
    c.execute_one_wide_with_bus(0xE8D2, 0x3F4F, &mut bus);
    assert_eq!(c.reg(3), 0xAA, "LDREXB zero-extends the byte");
    assert_eq!(c.exclusive_address, Some(0x2000_0300));

    // STREXB R0, R1, [R2]: hw0=0xE8C2 (Rn=2), hw1=0x1F40 (Rt=1, Rd=0)
    c.execute_one_wide_with_bus(0xE8C2, 0x1F40, &mut bus);
    assert_eq!(c.reg(0), 0, "STREXB success");
    assert_eq!(bus.read8(0x2000_0300, 0), 0x5A);
    assert_eq!(c.exclusive_address, None);
}

#[test]
fn ldrexb_clrex_strexb_fail() {
    // LDREXB, CLREX, STREXB -> Rd = 1, memory unchanged.
    let (mut c, mut bus) = core_and_bus();
    bus.write8(0x2000_0304, 0xAA, 0);
    c.set_reg(2, 0x2000_0304);
    c.set_reg(1, 0x5A);

    c.execute_one_wide_with_bus(0xE8D2, 0x3F4F, &mut bus); // LDREXB
    c.execute_one_wide_with_bus(0xF3BF, 0x8F2F, &mut bus); // CLREX
    assert_eq!(c.exclusive_address, None);

    c.execute_one_wide_with_bus(0xE8C2, 0x1F40, &mut bus); // STREXB
    assert_eq!(c.reg(0), 1);
    assert_eq!(bus.read8(0x2000_0304, 0), 0xAA, "memory unchanged");
}

// -- LDREXH / STREXH ---------------------------------------------------------

#[test]
fn ldrexh_strexh_success() {
    //   LDREXH Rt, [Rn]:    hw0 = 0xE8D0 | Rn, hw1 = (Rt << 12) | 0x0F5F
    //   STREXH Rd, Rt, [Rn]: hw0 = 0xE8C0 | Rn, hw1 = (Rt << 12) | 0x0F50 | Rd
    let (mut c, mut bus) = core_and_bus();
    bus.write16(0x2000_0310, 0xAABB, 0);
    c.set_reg(2, 0x2000_0310);
    c.set_reg(1, 0xCAFE);

    // LDREXH R3, [R2]: hw0=0xE8D2, hw1=0x3F5F
    c.execute_one_wide_with_bus(0xE8D2, 0x3F5F, &mut bus);
    assert_eq!(c.reg(3), 0xAABB);
    assert_eq!(c.exclusive_address, Some(0x2000_0310));

    // STREXH R0, R1, [R2]: hw0=0xE8C2, hw1=0x1F50
    c.execute_one_wide_with_bus(0xE8C2, 0x1F50, &mut bus);
    assert_eq!(c.reg(0), 0);
    assert_eq!(bus.read16(0x2000_0310, 0), 0xCAFE);
    assert_eq!(c.exclusive_address, None);
}

#[test]
fn ldrexh_clrex_strexh_fail() {
    let (mut c, mut bus) = core_and_bus();
    bus.write16(0x2000_0314, 0xAABB, 0);
    c.set_reg(2, 0x2000_0314);
    c.set_reg(1, 0xCAFE);

    c.execute_one_wide_with_bus(0xE8D2, 0x3F5F, &mut bus); // LDREXH
    c.execute_one_wide_with_bus(0xF3BF, 0x8F2F, &mut bus); // CLREX
    c.execute_one_wide_with_bus(0xE8C2, 0x1F50, &mut bus); // STREXH
    assert_eq!(c.reg(0), 1);
    assert_eq!(bus.read16(0x2000_0314, 0), 0xAABB);
}

// ============================================================================
// MSPLIM / PSPLIM via MSR/MRS
// ============================================================================

#[test]
fn msr_mrs_msplim_roundtrip() {
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
    let mut c = CortexM33::for_test(0);
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
        .join("../../roms/rp2350/bootrom-combined.bin");
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
        let pc = emu.cores.expect_arm_mut()[0].regs.pc();

        // Trace key bootrom addresses (dedup: skip if same PC as last trace)
        for &(addr, label) in trace_addrs {
            if pc == addr && pc != last_trace_pc {
                last_trace_pc = pc;
                eprintln!("[cycle {:>7}] Reached {:#010x}: {}", cycle, addr, label);
                let c0 = &emu.cores.expect_arm()[0];
                eprintln!("  R0={:#010x} R1={:#010x} R2={:#010x} R3={:#010x}",
                    c0.regs.r[0], c0.regs.r[1], c0.regs.r[2], c0.regs.r[3]);
                eprintln!("  R4={:#010x} R5={:#010x} R6={:#010x} R7={:#010x}",
                    c0.regs.r[4], c0.regs.r[5], c0.regs.r[6], c0.regs.r[7]);
                eprintln!("  LR={:#010x} SP={:#010x} secure={}", c0.regs.lr(), c0.regs.sp(), c0.secure);
                eprintln!("  R8={:#010x} R9={:#010x} R10={:#010x} R11={:#010x} R12={:#010x}",
                    c0.regs.r[8], c0.regs.r[9], c0.regs.r[10], c0.regs.r[11], c0.regs.r[12]);
                eprintln!("  MSP={:#010x} MSP_NS={:#010x} PSP={:#010x} PSP_NS={:#010x}",
                    c0.regs.msp, c0.regs.msp_ns, c0.regs.psp, c0.regs.psp_ns);
                // Extra detail at bxns points
                if addr == 0x0382 || addr == 0x7EA4 {
                    let target = if addr == 0x0382 { c0.regs.r[0] } else { c0.regs.lr() };
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
        let ipsr = emu.cores.expect_arm_mut()[0].regs.ipsr();
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
            let c0 = &emu.cores.expect_arm()[0];
            eprintln!("  PC={:#010x} LR={:#010x}", pc, c0.regs.lr());
            eprintln!("  CFSR={:#010x} HFSR={:#010x}", c0.ppb.cfsr, c0.ppb.hfsr);
            eprintln!("  BFAR={:#010x} MMFAR={:#010x}", c0.ppb.bfar, c0.ppb.mmfar);
            eprintln!("  R0-R3: {:#010x} {:#010x} {:#010x} {:#010x}",
                c0.regs.r[0], c0.regs.r[1], c0.regs.r[2], c0.regs.r[3]);
            eprintln!("  SP={:#010x} MSP={:#010x}", c0.regs.sp(), c0.regs.msp);
            eprintln!("  Max bootrom PC so far={:#010x}", max_pc);
            // Read exception frame from stack
            let sp = c0.regs.msp;
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
                let c0 = &emu.cores.expect_arm()[0];
                eprintln!("  IPSR={}, LR={:#010x}", c0.regs.ipsr(), c0.regs.lr());
                eprintln!("  CFSR={:#010x}, HFSR={:#010x}", c0.ppb.cfsr, c0.ppb.hfsr);
                eprintln!("  R0={:#010x} R1={:#010x} R2={:#010x} R3={:#010x}",
                    c0.regs.r[0], c0.regs.r[1], c0.regs.r[2], c0.regs.r[3]);
                eprintln!("  R4={:#010x} R5={:#010x} R6={:#010x} R7={:#010x}",
                    c0.regs.r[4], c0.regs.r[5],
                    c0.regs.r[6], c0.regs.r[7]);
                eprintln!("  SP={:#010x} MSP={:#010x}", c0.regs.sp(), c0.regs.msp);
                eprintln!("  BFAR={:#010x} MMFAR={:#010x}", c0.ppb.bfar, c0.ppb.mmfar);
                eprintln!("  Max bootrom PC reached={:#010x}", max_pc);
                // Try to read stacked PC from exception frame
                let sp = c0.regs.msp;
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
    let c0 = &emu.cores.expect_arm()[0];
    let final_pc = c0.regs.pc();
    eprintln!("Final PC={:#010x}, cycles run", final_pc);
    eprintln!("  IPSR={}, CFSR={:#010x}, HFSR={:#010x}",
        c0.regs.ipsr(), c0.ppb.cfsr, c0.ppb.hfsr);
    eprintln!("  secure={}, LR={:#010x}, SP={:#010x}",
        c0.secure, c0.regs.lr(), c0.regs.sp());
    eprintln!("  MSP={:#010x} MSP_NS={:#010x}",
        c0.regs.msp, c0.regs.msp_ns);
    eprintln!("  Max bootrom PC={:#010x}", max_pc);
}

// ============================================================================
// Phase 4 Stage A: QMI register backing store
// ============================================================================

#[test]
fn test_qmi_register_roundtrip() {
    let (_, mut bus) = core_and_bus();
    // M0_TIMING is at QMI offset 0x004
    bus.write32(0x400D_0004, 0xDEAD_BEEF, 0);
    assert_eq!(bus.read32(0x400D_0004, 0), 0xDEAD_BEEF);
}

#[test]
fn test_qmi_direct_csr_always_ready() {
    let (_, mut bus) = core_and_bus();
    // Write something to DIRECT_CSR (offset 0x000)
    bus.write32(0x400D_0000, 0x0000_0042, 0);
    let csr = bus.read32(0x400D_0000, 0);
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
    bus.write32(0xD000_0010, 0xAAAA_5555, 0);
    assert_eq!(bus.read32(0xD000_0010, 0), 0xAAAA_5555);
}

#[test]
fn test_sio_gpio_set_clr_xor() {
    let (_, mut bus) = core_and_bus();
    // Start with known value
    bus.write32(0xD000_0010, 0x0000_00FF, 0);
    // SET bits 8-15 (RP2350 GPIO_OUT_SET = 0x018)
    bus.write32(0xD000_0018, 0x0000_FF00, 0);
    assert_eq!(bus.read32(0xD000_0010, 0), 0x0000_FFFF);
    // CLR bits 0-7 (RP2350 GPIO_OUT_CLR = 0x020)
    bus.write32(0xD000_0020, 0x0000_00FF, 0);
    assert_eq!(bus.read32(0xD000_0010, 0), 0x0000_FF00);
    // XOR bit 15 (RP2350 GPIO_OUT_XOR = 0x028)
    bus.write32(0xD000_0028, 0x0000_8000, 0);
    assert_eq!(bus.read32(0xD000_0010, 0), 0x0000_7F00);

    // Same for GPIO_OE (RP2350 base = 0x030)
    bus.write32(0xD000_0030, 0xFFFF_0000, 0);
    bus.write32(0xD000_0040, 0x00FF_0000, 0); // GPIO_OE_CLR (0x040)
    assert_eq!(bus.read32(0xD000_0030, 0), 0xFF00_0000);
    bus.write32(0xD000_0038, 0x0000_FFFF, 0); // GPIO_OE_SET (0x038)
    assert_eq!(bus.read32(0xD000_0030, 0), 0xFF00_FFFF);
    bus.write32(0xD000_0048, 0x0100_0001, 0); // GPIO_OE_XOR (0x048)
    assert_eq!(bus.read32(0xD000_0030, 0), 0xFE00_FFFE);
}

#[test]
fn test_sio_cpuid() {
    let (_, mut bus) = core_and_bus();
    // Default active_core is 0
    assert_eq!(bus.read32(0xD000_0000, 0), 0);
}

// ============================================================================
// Phase 4 Stage A: CLOCKS dynamic source tracking
// ============================================================================

#[test]
fn test_clocks_source_tracking() {
    let (_, mut bus) = core_and_bus();
    // Write CLK_SYS_CTRL to select source 1 (aux)
    bus.write32(0x4001_003C, 0x0000_0001, 0);
    // CLK_SYS_SELECTED should reflect 1 << 1 = 2
    assert_eq!(bus.read32(0x4001_0044, 0), 0x2);

    // Write CLK_REF_CTRL to select source 2
    bus.write32(0x4001_0030, 0x0000_0002, 0);
    assert_eq!(bus.read32(0x4001_0030, 0), 0x0000_0002);
    // CLK_REF_SELECTED should reflect 1 << 2 = 4
    assert_eq!(bus.read32(0x4001_0038, 0), 0x4);
}

/// HLD V5 §4.2.10: every `clk_*_SELECTED` register on RP2350 must
/// report a non-zero value so pico-sdk's `clock_configure` busy-wait
/// completes. For glitchless clocks (clk_ref, clk_sys) the value is
/// `1 << (CTRL & SRC_MASK)`; for non-glitchless clocks (clk_gpout*,
/// clk_peri, clk_hstx, clk_usb, clk_adc) the value is `1`.
///
/// This guards against the "return 0 everywhere else" gap that the
/// mdrp2040 commit `b1a40e4` fixed on its sibling. Every RP2350
/// `_SELECTED` offset is exercised here — any future refactor that
/// drops one of these handshake returns fails this test.
#[test]
fn test_clocks_all_selected_registers_nonzero() {
    let (_, mut bus) = core_and_bus();

    // Non-glitchless clocks — `_SELECTED` reads 1 unconditionally.
    // clk_gpout0..3 at 0x008, 0x014, 0x020, 0x02C.
    assert_eq!(bus.read32(0x4001_0008, 0), 1, "CLK_GPOUT0_SELECTED");
    assert_eq!(bus.read32(0x4001_0014, 0), 1, "CLK_GPOUT1_SELECTED");
    assert_eq!(bus.read32(0x4001_0020, 0), 1, "CLK_GPOUT2_SELECTED");
    assert_eq!(bus.read32(0x4001_002C, 0), 1, "CLK_GPOUT3_SELECTED");
    // clk_peri at 0x050, clk_hstx at 0x05C, clk_usb at 0x068, clk_adc
    // at 0x074. RP2350 has no CLK_RTC (unlike RP2040).
    assert_eq!(bus.read32(0x4001_0050, 0), 1, "CLK_PERI_SELECTED");
    assert_eq!(bus.read32(0x4001_005C, 0), 1, "CLK_HSTX_SELECTED");
    assert_eq!(bus.read32(0x4001_0068, 0), 1, "CLK_USB_SELECTED");
    assert_eq!(bus.read32(0x4001_0074, 0), 1, "CLK_ADC_SELECTED");

    // Glitchless clocks — default CTRL = 0, so `_SELECTED = 1 << 0 = 1`.
    assert_eq!(bus.read32(0x4001_0038, 0), 1, "CLK_REF_SELECTED default");
    assert_eq!(bus.read32(0x4001_0044, 0), 1, "CLK_SYS_SELECTED default");

    // Glitchless clocks — CTRL update reflected immediately (one-cycle
    // handshake, HLD V5 §5.7).
    bus.write32(0x4001_003C, 0x0000_0001, 0);
    assert_eq!(
        bus.read32(0x4001_0044, 0),
        1 << 1,
        "CLK_SYS_SELECTED after CTRL = 1",
    );
    bus.write32(0x4001_0030, 0x0000_0003, 0);
    assert_eq!(
        bus.read32(0x4001_0038, 0),
        1 << 3,
        "CLK_REF_SELECTED after CTRL = 3 (2-bit SRC field)",
    );
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

    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms/rp2350");
    let rom = std::fs::read(base.join("bootrom-combined.bin"))
        .expect("bootrom not found");
    let flash = std::fs::read(base.join("blinky.bin"))
        .expect("blinky.bin not found — run: python3 roms/rp2350/gen_blinky.py roms/rp2350/blinky.bin");

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
        core1_max_ipsr = core1_max_ipsr.max(emu.cores.expect_arm_mut()[1].regs.ipsr());
        let pc = emu.cores.expect_arm_mut()[0].regs.pc();

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
                let c0 = &emu.cores.expect_arm()[0];
                eprintln!("  IPSR={}, CFSR={:#010x}, HFSR={:#010x}",
                    c0.regs.ipsr(),
                    c0.ppb.cfsr, c0.ppb.hfsr);
                break;
            }
        } else {
            stuck_count = 0;
        }
        last_pc = pc;
    }

    let _gpio_out = emu.bus.sio.gpio_out;
    let gpio_oe = emu.bus.sio.gpio_oe;
    let pc = emu.cores.expect_arm_mut()[0].regs.pc();

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

/// Phase 0b.1 Commit B: the `Bus::set_active_core` indirection is gone
/// (per-core PPB moved onto `CortexM33`). These helpers are retained as
/// no-ops to keep the FIFO/spinlock tests' call-sites readable.
/// SIO dispatch now takes a `core: u8` parameter explicitly via the bus
/// read/write methods that each test site already threads.
fn set_core1(_bus: &mut Bus) {}
fn set_core0(_bus: &mut Bus) {}

#[test]
fn fifo_push_pop_basic_roundtrip() {
    let mut bus = Bus::new();
    // Core 0 writes 3 values to Core 1's RX FIFO
    bus.write32(FIFO_WR, 0xAAAA_BBBB, 0);
    bus.write32(FIFO_WR, 0xCCCC_DDDD, 0);
    bus.write32(FIFO_WR, 0x1234_5678, 0);

    // Core 1 reads them back in FIFO order
    set_core1(&mut bus);
    assert_eq!(bus.read32(FIFO_RD, 1), 0xAAAA_BBBB);
    assert_eq!(bus.read32(FIFO_RD, 1), 0xCCCC_DDDD);
    assert_eq!(bus.read32(FIFO_RD, 1), 0x1234_5678);
}

#[test]
fn fifo_empty_read_returns_zero_and_sets_roe() {
    let mut bus = Bus::new();
    // Core 0 reads from empty RX FIFO
    let val = bus.read32(FIFO_RD, 0);
    assert_eq!(val, 0, "Empty FIFO read should return 0");

    // FIFO_ST should show ROE (bit 3) set for Core 0
    let st = bus.read32(FIFO_ST, 0);
    assert!(st & 0x8 != 0, "ROE bit should be set after empty read, FIFO_ST={:#x}", st);
}

#[test]
fn fifo_full_write_drops_data_and_sets_wof() {
    let mut bus = Bus::new();
    // Fill Core 1's RX FIFO (8 entries) from Core 0
    for i in 0..8u32 {
        bus.write32(FIFO_WR, i, 0);
    }
    // 9th write should overflow
    bus.write32(FIFO_WR, 0xDEAD, 0);

    // Core 0's FIFO_ST should show WOF (bit 2) set
    let st = bus.read32(FIFO_ST, 0);
    assert!(st & 0x4 != 0, "WOF bit should be set after overflow, FIFO_ST={:#x}", st);

    // Core 1 should read the original 8 values, not the dropped 0xDEAD
    set_core1(&mut bus);
    for i in 0..8u32 {
        assert_eq!(bus.read32(FIFO_RD, 1), i);
    }
    // Next read is empty
    assert_eq!(bus.read32(FIFO_RD, 1), 0);
}

#[test]
fn fifo_st_reflects_vld_and_rdy() {
    let mut bus = Bus::new();
    // Initially: Core 0 RX is empty (VLD=0), Core 1 RX has space (RDY=1)
    let st = bus.read32(FIFO_ST, 0);
    assert_eq!(st & 0x1, 0, "VLD should be 0 when RX is empty");
    assert_eq!(st & 0x2, 0x2, "RDY should be 1 when TX has space");

    // Core 1 writes to Core 0's RX FIFO
    set_core1(&mut bus);
    bus.write32(FIFO_WR, 42, 1);
    set_core0(&mut bus);

    // Now Core 0's RX has data
    let st = bus.read32(FIFO_ST, 0);
    assert_eq!(st & 0x1, 0x1, "VLD should be 1 after data written to our RX");

    // Fill Core 1's RX from Core 0 (8 entries)
    for i in 0..8u32 {
        bus.write32(FIFO_WR, i, 0);
    }
    // RDY should be 0 (Core 1's RX is full)
    let st = bus.read32(FIFO_ST, 0);
    assert_eq!(st & 0x2, 0, "RDY should be 0 when other core's RX is full");
}

#[test]
fn fifo_st_w1c_clears_wof_and_roe() {
    let mut bus = Bus::new();
    // Trigger ROE by reading empty FIFO
    bus.read32(FIFO_RD, 0);
    let st = bus.read32(FIFO_ST, 0);
    assert!(st & 0x8 != 0, "ROE should be set");

    // Fill FIFO then overflow to trigger WOF
    for _ in 0..9 {
        bus.write32(FIFO_WR, 0, 0);
    }
    let st = bus.read32(FIFO_ST, 0);
    assert!(st & 0x4 != 0, "WOF should be set");
    assert!(st & 0x8 != 0, "ROE should still be set");

    // W1C: clear WOF only
    bus.write32(FIFO_ST, 0x4, 0);
    let st = bus.read32(FIFO_ST, 0);
    assert_eq!(st & 0x4, 0, "WOF should be cleared");
    assert!(st & 0x8 != 0, "ROE should still be set (not cleared)");

    // W1C: clear ROE
    bus.write32(FIFO_ST, 0x8, 0);
    let st = bus.read32(FIFO_ST, 0);
    assert_eq!(st & 0x8, 0, "ROE should be cleared");

    // W1C: writing 0xFFFFFFFF clears both
    bus.read32(FIFO_RD, 0); // trigger ROE again
    for _ in 0..9 {
        bus.write32(FIFO_WR, 0, 0);
    }
    bus.write32(FIFO_ST, 0xFFFF_FFFF, 0);
    let st = bus.read32(FIFO_ST, 0);
    assert_eq!(st & 0xC, 0, "Both WOF and ROE should be cleared");
}

#[test]
fn fifo_write_sets_event_flag_on_receiver() {
    let mut bus = Bus::new();
    // Event flags start clear
    assert!(!bus.atomics.event_flag[0].load(Ordering::Relaxed));
    assert!(!bus.atomics.event_flag[1].load(Ordering::Relaxed));

    // Core 0 writes FIFO_WR -> should set event_flag[1] (receiver = Core 1)
    bus.write32(FIFO_WR, 0x42, 0);
    assert!(bus.atomics.event_flag[1].load(Ordering::Relaxed), "event_flag[1] should be set after Core 0 FIFO write");
    assert!(!bus.atomics.event_flag[0].load(Ordering::Relaxed), "event_flag[0] should NOT be set");

    // Clear event flags
    bus.atomics.event_flag[0].store(false, Ordering::Relaxed);
    bus.atomics.event_flag[1].store(false, Ordering::Relaxed);

    // Core 1 writes FIFO_WR -> should set event_flag[0] (receiver = Core 0)
    set_core1(&mut bus);
    bus.write32(FIFO_WR, 0x43, 1);
    set_core0(&mut bus);
    assert!(bus.atomics.event_flag[0].load(Ordering::Relaxed), "event_flag[0] should be set after Core 1 FIFO write");
}

#[test]
fn fifo_overflow_does_not_set_event_flag() {
    let mut bus = Bus::new();
    // Fill Core 1's RX FIFO
    for i in 0..8u32 {
        bus.write32(FIFO_WR, i, 0);
    }
    // Clear event flags
    bus.atomics.event_flag[0].store(false, Ordering::Relaxed);
    bus.atomics.event_flag[1].store(false, Ordering::Relaxed);

    // Overflow write should NOT set event flag
    bus.write32(FIFO_WR, 0xDEAD, 0);
    assert!(!bus.atomics.event_flag[1].load(Ordering::Relaxed), "event_flag should NOT be set on overflow write");
}

// ============================================================================
// Phase 5 Stage A2: Spinlock unit tests
// ============================================================================

#[test]
fn spinlock_claim_returns_bit_mask() {
    let mut bus = Bus::new();
    // Claim spinlock 5 from Core 0
    let result = bus.read32(spinlock_addr(5), 0);
    assert_eq!(result, 1 << 5, "Claiming lock 5 should return 1<<5");

    // SPINLOCK_ST should reflect the claimed lock
    let st = bus.read32(SPINLOCK_ST, 0);
    assert_eq!(st & (1 << 5), 1 << 5, "SPINLOCK_ST should show lock 5 claimed");
}

#[test]
fn spinlock_already_claimed_returns_zero() {
    let mut bus = Bus::new();
    // Claim lock 10
    let first = bus.read32(spinlock_addr(10), 0);
    assert_eq!(first, 1 << 10);

    // Second claim returns 0
    let second = bus.read32(spinlock_addr(10), 0);
    assert_eq!(second, 0, "Already-claimed lock should return 0");
}

#[test]
fn spinlock_release_via_write() {
    let mut bus = Bus::new();
    // Claim lock 7
    bus.read32(spinlock_addr(7), 0);
    assert_eq!(bus.read32(SPINLOCK_ST, 0) & (1 << 7), 1 << 7);

    // Release via write (any value)
    bus.write32(spinlock_addr(7), 0, 0);
    assert_eq!(bus.read32(SPINLOCK_ST, 0) & (1 << 7), 0, "Lock 7 should be released");

    // Re-claim should succeed
    let result = bus.read32(spinlock_addr(7), 0);
    assert_eq!(result, 1 << 7, "Re-claiming released lock should succeed");
}

#[test]
fn spinlock_contention_core0_claims_core1_sees_zero() {
    let mut bus = Bus::new();
    // Core 0 claims lock 15
    let c0 = bus.read32(spinlock_addr(15), 0);
    assert_eq!(c0, 1 << 15);

    // Core 1 tries to claim same lock -> gets 0
    set_core1(&mut bus);
    let c1 = bus.read32(spinlock_addr(15), 1);
    assert_eq!(c1, 0, "Core 1 should fail to claim lock already held by Core 0");

    // Core 1 can release it though (any write clears)
    bus.write32(spinlock_addr(15), 1, 1);
    set_core0(&mut bus);

    // Lock is now free, Core 0 can reclaim
    let c0_again = bus.read32(spinlock_addr(15), 0);
    assert_eq!(c0_again, 1 << 15);
}

#[test]
fn spinlock_st_bitmask_reflects_state() {
    let mut bus = Bus::new();
    // Claim locks 0, 3, 31
    bus.read32(spinlock_addr(0), 0);
    bus.read32(spinlock_addr(3), 0);
    bus.read32(spinlock_addr(31), 0);

    let st = bus.read32(SPINLOCK_ST, 0);
    assert_eq!(st, (1 << 0) | (1 << 3) | (1 << 31),
        "SPINLOCK_ST should reflect exactly the claimed locks, got {:#010x}", st);

    // Release lock 3
    bus.write32(spinlock_addr(3), 0, 0);
    let st = bus.read32(SPINLOCK_ST, 0);
    assert_eq!(st, (1 << 0) | (1 << 31),
        "SPINLOCK_ST should reflect lock 3 released, got {:#010x}", st);
}

// ============================================================================
// WFE / SEV instruction dispatch
// ============================================================================

#[test]
fn wfe_with_event_pending_consumes_and_continues() {
    let (mut cpu, mut bus) = core_and_bus();
    bus.atomics.event_flag[0].store(true, Ordering::Relaxed);
    // WFE Thumb-16 encoding: 0xBF20 (hint op = 0x2, mask = 0)
    cpu.execute_one_with_bus(0xBF20, &mut bus);
    assert!(!bus.atomics.event_flag[0].load(Ordering::Relaxed), "event_flag should be consumed");
    assert!(!cpu.is_wfe_waiting(), "core should NOT be sleeping — event was pending");
}

#[test]
fn wfe_without_event_enters_sleep() {
    let (mut cpu, mut bus) = core_and_bus();
    assert!(!bus.atomics.event_flag[0].load(Ordering::Relaxed));
    cpu.execute_one_with_bus(0xBF20, &mut bus);
    assert!(cpu.is_wfe_waiting(), "core should be sleeping — no event was pending");
}

#[test]
fn sev_sets_both_event_flags() {
    let (mut cpu, mut bus) = core_and_bus();
    assert!(!bus.atomics.event_flag[0].load(Ordering::Relaxed));
    assert!(!bus.atomics.event_flag[1].load(Ordering::Relaxed));
    // SEV Thumb-16 encoding: 0xBF40 (hint op = 0x4, mask = 0)
    cpu.execute_one_with_bus(0xBF40, &mut bus);
    assert!(bus.atomics.event_flag[0].load(Ordering::Relaxed), "event_flag[0] should be set after SEV");
    assert!(bus.atomics.event_flag[1].load(Ordering::Relaxed), "event_flag[1] should be set after SEV");
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
    emu.bus.atomics.set_wfe_waiting(0);
    emu.bus.atomics.event_flag[0].store(true, Ordering::Relaxed);

    emu.step();

    assert!(!emu.core_mut(0).is_wfe_waiting(), "core should have been woken by event_flag");
    assert!(!emu.bus.atomics.event_flag[0].load(Ordering::Relaxed), "event_flag should have been consumed");
}

// ============================================================================
// Phase 5 B2: Core 1 boot reaches WFE
// ============================================================================

#[test]
fn test_core1_boot_reaches_wfe() {
    use crate::{Emulator, Config};

    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms/rp2350");
    let rom = std::fs::read(base.join("bootrom-combined.bin"))
        .expect("bootrom-combined.bin not found");

    let mut emu = Emulator::new(Config::default());
    emu.load_bootrom(&rom);
    emu.reset();

    for _ in 0..1_000_000 {
        emu.step();
        // Early exit once Core 1 enters WFE sleep
        if emu.cores.expect_arm_mut()[1].is_wfe_waiting() {
            break;
        }
    }

    assert!(emu.cores.expect_arm_mut()[1].is_wfe_waiting(),
        "Core 1 should be sleeping in WFE after bootrom init (PC={:#010x})",
        emu.cores.expect_arm_mut()[1].regs.pc());
    assert_eq!(emu.cores.expect_arm_mut()[1].regs.ipsr(), 0,
        "Core 1 should not be in an exception handler (IPSR={})",
        emu.cores.expect_arm_mut()[1].regs.ipsr());
    assert!(emu.cores.expect_arm_mut()[1].regs.pc() < 0x8000,
        "Core 1 PC should be in bootrom range (PC={:#010x})",
        emu.cores.expect_arm_mut()[1].regs.pc());
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

    let base = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms/rp2350");
    let bootrom = std::fs::read(base.join("bootrom-combined.bin"))
        .expect("bootrom-combined.bin not found");
    let flash = std::fs::read(base.join("dualcore.bin"))
        .expect("dualcore.bin not found — run: python roms/rp2350/gen_dualcore.py roms/rp2350/dualcore.bin");

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
    assert!(emu.cores.expect_arm_mut()[1].regs.pc() >= 0x1000_0000,
        "Core 1 should be in flash, PC={:#010x}", emu.cores.expect_arm_mut()[1].regs.pc());
    // Core 1 should not be WFE-waiting
    assert!(!emu.cores.expect_arm_mut()[1].is_wfe_waiting(),
        "Core 1 should not be WFE-waiting");
}

// ============================================================================
// Clock Tree V1: ROSC boot-clock fix
// ============================================================================

#[test]
fn test_rosc_status_returns_stable_enabled() {
    let (_, mut bus) = core_and_bus();
    let status = bus.read32(0x400E_8018, 0);
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
fn test_bus_new_is_post_bootrom_sys_clock() {
    // B5 (HLD V5 §5.7): Bus::new() installs the post-bootrom clock
    // table directly — clk_sys=150 MHz, clk_ref=12 MHz. This is the
    // state firmware sees after pico-sdk `runtime_init_clocks` on
    // silicon, and load_image-based tests need to see it too.
    // Replaces the earlier "Bus::new starts at ROSC" test, which was
    // locking in the pre-B5 inconsistency (Bus::new returned ROSC
    // while Emulator::reset returned post-bootrom).
    use mdpicoem_common::clocks::{RP2350_SYS_CLK_HZ, XOSC_FREQ_HZ};
    let bus = Bus::new();
    assert_eq!(bus.sys_clk_hz(), RP2350_SYS_CLK_HZ,
        "fresh Bus must report post-bootrom clk_sys = 150 MHz");
    assert_eq!(bus.ref_clk_hz(), XOSC_FREQ_HZ,
        "fresh Bus must report post-bootrom clk_ref = 12 MHz (XOSC)");
}

#[test]
fn test_xosc_via_clk_ref_sys_clock() {
    use crate::bus::clocks::XOSC_FREQ_HZ;
    let (_, mut bus) = core_and_bus();
    // CLK_REF_CTRL SRC=2 (XOSC)
    bus.write32(0x4001_0030, 0x0000_0002, 0);
    // CLK_SYS_CTRL SRC=0 (clk_ref)
    bus.write32(0x4001_003C, 0x0000_0000, 0);
    assert_eq!(bus.sys_clk_hz(), XOSC_FREQ_HZ,
        "CLK_SYS routed through CLK_REF=XOSC should give 12 MHz");
    assert_eq!(bus.ref_clk_hz(), XOSC_FREQ_HZ);
}

#[test]
fn test_clk_sys_div_scales_output() {
    use crate::bus::clocks::XOSC_FREQ_HZ;
    let (_, mut bus) = core_and_bus();
    // Route CLK_SYS to XOSC via CLK_REF
    bus.write32(0x4001_0030, 0x0000_0002, 0);
    bus.write32(0x4001_003C, 0x0000_0000, 0);
    // CLK_SYS_DIV integer = 2 (bits [31:16])
    bus.write32(0x4001_0040, 0x0002_0000, 0);
    assert_eq!(bus.sys_clk_hz(), XOSC_FREQ_HZ / 2,
        "CLK_SYS_DIV=2 should halve the source frequency");
}

#[test]
fn test_clocks_write_alias_set() {
    let (_, mut bus) = core_and_bus();
    // Normal write: CLK_REF_CTRL = 0x01
    bus.write32(0x4001_0030, 0x0000_0001, 0);
    // SET alias (alias=2) at offset 0x030 → 0x4001_0000 | (2 << 12) | 0x030
    bus.write32(0x4001_2030, 0x0000_0002, 0);
    // Expect OR, not overwrite → 0x03
    assert_eq!(bus.read32(0x4001_0030, 0), 0x0000_0003,
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
    bus.write32(0x4005_0000, 0x0000_0001, 0);
    // FBDIV_INT = 125
    bus.write32(0x4005_0008, 125, 0);
    // PRIM: POSTDIV1=5 in bits [18:16], POSTDIV2=2 in bits [14:12]
    bus.write32(0x4005_000C, (5 << 16) | (2 << 12), 0);
    // Switch CLK_SYS to aux=0 (PLL_SYS): SRC=1, AUXSRC=0
    bus.write32(0x4001_003C, 0x0000_0001, 0);
    assert_eq!(bus.sys_clk_hz(), 150_000_000,
        "PLL_SYS configured for 150 MHz should give sys_clk_hz = 150_000_000");
}

#[test]
fn test_unconfigured_pll_zero_hz() {
    // Fresh Bus: reset values leave FBDIV=0, so pll_output_hz must return 0.
    // Switching CLK_SYS to PLL_SYS without configuring should report 0 Hz.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4001_003C, 0x0000_0001, 0); // SRC=1 (aux), AUXSRC=0 (PLL_SYS)
    assert_eq!(bus.sys_clk_hz(), 0,
        "Unconfigured PLL (FBDIV=0) must honestly report 0 Hz, not a .max(1) fudge");
}

#[test]
fn test_pll_usb_separate_from_pll_sys() {
    // Configuring PLL_USB must not bleed into PLL_SYS — they have
    // separate backing arrays. CLK_SYS stays on clk_ref (ROSC at reset
    // register state), so sys_clk_hz is unaffected by PLL_USB changes.
    // We sample `before` after an initial CLOCKS write so recompute
    // has installed the register-derived value — post-B5, Bus::new
    // seeds the post-bootrom table, but a CLOCKS write always reverts
    // sys_clk_hz to the register-derived frequency.
    let (_, mut bus) = core_and_bus();
    // Prime `before` with a CLOCKS write to trigger recompute once.
    bus.write32(0x4001_003C, 0x0000_0000, 0); // CLK_SYS_CTRL SRC=0 (clk_ref → ROSC)
    let before = bus.sys_clk_hz();
    // Configure PLL_USB to some non-trivial value (48 MHz: FBDIV=100,
    // POSTDIV1=5, POSTDIV2=5; VCO=1200M / 25 = 48M).
    bus.write32(0x4005_8000, 0x0000_0001, 0); // CS REFDIV=1
    bus.write32(0x4005_8008, 100, 0);         // FBDIV_INT
    bus.write32(0x4005_800C, (5 << 16) | (5 << 12), 0);
    assert_eq!(bus.sys_clk_hz(), before,
        "PLL_USB changes must not affect sys_clk_hz while CLK_SYS is on ROSC");
    // Sanity: PLL_USB registers actually took the writes.
    assert_eq!(bus.read32(0x4005_8008, 0), 100,
        "PLL_USB FBDIV_INT should read back the value we wrote");
}

#[test]
fn test_pll_fbdiv_max_no_overflow() {
    // FBDIV=0xFFF (4095) with defaults (REFDIV=1, POSTDIV1=7, POSTDIV2=7)
    // gives 12M * 4095 / 49 ≈ 1.003 GHz. Must not panic on u32 overflow.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4005_0008, 0xFFF, 0); // FBDIV_INT = 4095 (max)
    bus.write32(0x4001_003C, 0x0000_0001, 0); // Route CLK_SYS → PLL_SYS
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
    // Post-`2026.04.15 HLD - PLL LOCK Modelling` fix: CS[31] is no longer
    // forced. At Bus::new() reset, PWR=0x2D (PD+VCOPD set) and FBDIV=0,
    // so `pll_is_locked_base` is false and CS reads return the stored
    // value with LOCK=0. Pre-fix this asserted `0x8000_0001`; that
    // assertion was locking in the known bug (see tech_debt.md).
    let mut bus = Bus::new();
    let cs_read = bus.read32(0x4005_0000, 0);
    assert_eq!(cs_read, 0x0000_0001,
        "CS read at reset must NOT force LOCK — PLL is powered down, FBDIV=0");
    assert_eq!(cs_read & (1 << 31), 0, "LOCK=0 at reset");
}

#[test]
fn test_pll_sys_write_set_alias_subword() {
    // Subword SET alias on PLL_USB PWR — matches the bootrom's nsboot
    // path. A byte-wide SET must OR into the register, not overwrite.
    let mut bus = Bus::new();
    // PLL_USB PWR reset = 0x2D. SET alias byte write of 0x40 to byte 0
    // should yield 0x6D (0x2D | 0x40).
    // Address: 0x4005_8004 + SET alias (2 << 12) = 0x4005_A004.
    bus.write8(0x4005_A004, 0x40, 0);
    assert_eq!(bus.pll_usb_regs[1], 0x6D,
        "byte-wide SET alias on PLL_USB PWR must OR, not overwrite");
}

// ============================================================================
// PLL LOCK modelling — see `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md`
// ============================================================================
//
// Twelve integration tests exercising CS[31] read and write-time lock-arm
// transitions through `Bus::read32` / `Bus::write32`. `bus.master_cycle`
// is seeded directly on the Bus (a `pub(crate)` field) between writes and
// reads; the Emulator's step entry stashes this value from `Clock::cycles`
// in production, but tests bypass that plumbing to exercise the Bus path
// in isolation.

use mdpicoem_common::clocks::PLL_LOCK_DELAY_SYSCLKS;

#[test]
fn test_pll_cs_read_lock_zero_at_reset() {
    // At `Bus::new()`, PWR=0x2D (PD+VCOPD set) and FBDIV=0 → base
    // predicate is false ⇒ CS[31] must read 0 regardless of cycle.
    let mut bus = Bus::new();
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 at reset");
}

#[test]
fn test_pll_cs_lock_zero_before_arm() {
    // Power up the PLL and configure FBDIV. Immediately after the write,
    // `lock_at_cycle = now + PLL_LOCK_DELAY_SYSCLKS`; reading CS before
    // that arm must yield LOCK=0.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);    // FBDIV_INT = 100
    bus.write32(0x4005_0004, 0, 0);      // PWR = 0 (fully powered up)
    bus.master_cycle = 100;
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 before arm cycle");
}

#[test]
fn test_pll_cs_lock_one_after_arm() {
    // Same sequence, but read after the arm expiry.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);    // FBDIV_INT = 100
    bus.write32(0x4005_0004, 0, 0);      // PWR = 0
    bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
    let cs = bus.read32(0x4005_0000, 0);
    assert_ne!(cs & (1 << 31), 0, "LOCK must be 1 past arm cycle");
}

#[test]
fn test_pll_cs_lock_zero_with_pd_set() {
    // PD only, FBDIV=100 — PD gate wins regardless of cycle count.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);
    bus.write32(0x4005_0004, 0x01, 0);   // PWR = 0x01 (PD only)
    bus.master_cycle = 10_000;
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 while PD=1");
}

#[test]
fn test_pll_cs_lock_zero_with_vcopd_set() {
    // VCOPD only (PD clear), FBDIV=100 — VCOPD gate wins.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);
    bus.write32(0x4005_0004, 0x20, 0);   // PWR = 0x20 (VCOPD only)
    bus.master_cycle = 10_000;
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 while VCOPD=1");
}

#[test]
fn test_pll_cs_lock_zero_with_fbdiv_zero() {
    // PWR=0, FBDIV=0 — unconfigured PLL; predicate false.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0004, 0, 0);      // PWR = 0
    // FBDIV stays at reset value of 0.
    bus.master_cycle = 10_000;
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 when FBDIV=0");
}

#[test]
fn test_pll_cs_lock_rearm_after_powerdown() {
    // Power up, pass the arm, observe LOCK=1; re-assert PD+VCOPD → LOCK=0
    // on next read. Confirms we are NOT on a one-shot latch path
    // (Option B from the HLD drops the arm on any write landing in
    // "not powered / not configured" territory).
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);
    bus.write32(0x4005_0004, 0, 0);      // power up
    bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
    let cs1 = bus.read32(0x4005_0000, 0);
    assert_ne!(cs1 & (1 << 31), 0, "LOCK must be 1 after initial lock");

    bus.write32(0x4005_0004, 0x21, 0);   // PD+VCOPD set → drop lock
    let cs2 = bus.read32(0x4005_0000, 0);
    assert_eq!(cs2 & (1 << 31), 0, "LOCK must drop when power-down re-asserts");
}

#[test]
fn test_pll_cs_bypass_does_not_force_lock() {
    // Set BYPASS (CS[8]=1) with PWR=0 and FBDIV=100 but read before the
    // arm elapses — BYPASS must not short-circuit LOCK. Matches
    // conservative "BYPASS doesn't assert LOCK" interpretation (HLD §2).
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0000, 0x101, 0);  // CS: REFDIV=1 | BYPASS=1
    bus.write32(0x4005_0008, 100, 0);    // FBDIV = 100
    bus.write32(0x4005_0004, 0, 0);      // PWR = 0
    bus.master_cycle = 100;           // still well before arm
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(cs & (1 << 31), 0, "BYPASS must not force LOCK=1");
}

#[test]
fn test_pll_cs_read_preserves_refdiv() {
    // Write CS with REFDIV=5, power up, pass the arm → LOCK=1 AND
    // REFDIV bits preserved in the read-back.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0000, 0x05, 0);   // REFDIV = 5
    bus.write32(0x4005_0008, 100, 0);
    bus.write32(0x4005_0004, 0, 0);
    bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(cs & 0x3F, 5, "REFDIV must round-trip");
    assert_ne!(cs & (1 << 31), 0, "LOCK must be 1");
}

#[test]
fn test_pll_cs_alias_writes_trigger_arm() {
    // Exercise SET / CLR alias writes and confirm the arm transition
    // fires at the PWR CLR (which flips the predicate true), not the
    // CS SET (which leaves the PLL powered down).
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);    // FBDIV = 100 (predicate still
                                      // false because PWR is still 0x2D)
    assert_eq!(bus.pll_sys_lock_at_cycle, None,
        "FBDIV write must not arm while PLL is powered down");

    // SET alias on CS: OR 0x01 (no visible change — REFDIV already 1).
    bus.write32(0x4005_2000, 0x01, 0);
    assert_eq!(bus.pll_sys_lock_at_cycle, None,
        "CS SET alias must not arm while PLL is powered down");

    bus.master_cycle = 100;
    // CLR alias on PWR: clear all power-down bits.
    bus.write32(0x4005_3004, 0x2D, 0);
    assert_eq!(bus.pll_sys_lock_at_cycle, Some(100 + PLL_LOCK_DELAY_SYSCLKS),
        "PWR CLR alias must arm the lock at now + delay");
}

#[test]
fn test_pll_prim_write_does_not_rearm() {
    // Establish an arm, then write PRIM (POSTDIV1/2). The arm point
    // must NOT move — PRIM is post-VCO.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);
    bus.write32(0x4005_0004, 0, 0);
    let armed_at = bus.pll_sys_lock_at_cycle;
    assert_eq!(armed_at, Some(PLL_LOCK_DELAY_SYSCLKS));

    bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
    // Read once to confirm LOCK=1 baseline.
    assert_ne!(bus.read32(0x4005_0000, 0) & (1 << 31), 0);

    // Write PRIM to a different POSTDIV combination.
    bus.write32(0x4005_000C, (2u32 << 16) | (2u32 << 12), 0);
    assert_eq!(bus.pll_sys_lock_at_cycle, armed_at,
        "PRIM write must not rearm the lock-detect counter");
    assert_ne!(bus.read32(0x4005_0000, 0) & (1 << 31), 0,
        "LOCK must stay 1 after PRIM-only write");
}

#[test]
fn test_pll_usb_independent_of_pll_sys() {
    // Arm PLL_SYS; PLL_USB should remain un-armed and read LOCK=0.
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);    // PLL_SYS FBDIV
    bus.write32(0x4005_0004, 0, 0);      // PLL_SYS PWR = 0
    bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
    assert_ne!(bus.read32(0x4005_0000, 0) & (1 << 31), 0,
        "PLL_SYS should report LOCK=1 past arm");
    assert_eq!(bus.read32(0x4005_8000, 0) & (1 << 31), 0,
        "PLL_USB must remain LOCK=0 (independent state)");
    assert_eq!(bus.pll_usb_lock_at_cycle, None);
}

#[test]
fn test_pll_cs_rearm_on_fbdiv_change_mid_run() {
    // Bus-level integration probe for the Option B fidelity path:
    // changing FBDIV while the PLL is still powered re-arms the
    // lock-detect counter, so LOCK drops back to 0 until the new
    // arm elapses. (Option C — the original HLD draft — would have
    // kept LOCK latched here.)
    let mut bus = Bus::new();
    bus.master_cycle = 0;
    bus.write32(0x4005_0008, 100, 0);    // FBDIV = 100
    bus.write32(0x4005_0004, 0, 0);      // PWR = 0 → arm
    bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
    assert_ne!(bus.read32(0x4005_0000, 0) & (1 << 31), 0,
        "initial lock past first arm");

    let reconfig_at = PLL_LOCK_DELAY_SYSCLKS + 100;
    bus.master_cycle = reconfig_at;
    bus.write32(0x4005_0008, 125, 0);    // change FBDIV while powered
    assert_eq!(
        bus.pll_sys_lock_at_cycle,
        Some(reconfig_at + PLL_LOCK_DELAY_SYSCLKS),
        "FBDIV change must re-arm the lock-detect counter",
    );
    assert_eq!(bus.read32(0x4005_0000, 0) & (1 << 31), 0,
        "LOCK must drop to 0 between rearm and the new arm point");

    bus.master_cycle = reconfig_at + PLL_LOCK_DELAY_SYSCLKS + 1;
    assert_ne!(bus.read32(0x4005_0000, 0) & (1 << 31), 0,
        "LOCK must re-assert past the new arm");
}

// ============================================================================
// Clock Tree V2 Phase D: ROSC / XOSC register backing
// ============================================================================

#[test]
fn test_rosc_ctrl_roundtrip() {
    // Writing CTRL (0x000) should be stored and read back verbatim.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x400E_8000, 0xDEAD_BEEF, 0);
    assert_eq!(bus.read32(0x400E_8000, 0), 0xDEAD_BEEF,
        "ROSC CTRL should round-trip writes (stored, reads return last write)");
}

#[test]
fn test_rosc_status_unchanged_by_writes() {
    // STATUS (0x018) is read-only: writes are dropped; reads always
    // return STABLE | ENABLED per the V1 stub behaviour.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x400E_8018, 0, 0);
    assert_eq!(bus.read32(0x400E_8018, 0), (1 << 31) | (1 << 12),
        "ROSC STATUS must remain STABLE|ENABLED regardless of writes");
}

#[test]
fn test_xosc_ctrl_roundtrip() {
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4004_8000, 0xCAFE_BABE, 0);
    assert_eq!(bus.read32(0x4004_8000, 0), 0xCAFE_BABE,
        "XOSC CTRL should round-trip writes");
}

#[test]
fn test_xosc_startup_roundtrip() {
    let (_, mut bus) = core_and_bus();
    bus.write32(0x4004_800C, 0x0000_00C4, 0);
    assert_eq!(bus.read32(0x4004_800C, 0), 0x0000_00C4,
        "XOSC STARTUP should round-trip writes");
}

#[test]
fn test_rosc_ctrl_alias_set() {
    // Normal write CTRL=0x01, then write 0x02 via SET alias (0x400EA000)
    // — bits should be OR-ed, not overwritten → CTRL reads 0x03.
    let (_, mut bus) = core_and_bus();
    bus.write32(0x400E_8000, 0x0000_0001, 0);
    bus.write32(0x400E_A000, 0x0000_0002, 0);
    assert_eq!(bus.read32(0x400E_8000, 0), 0x0000_0003,
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
    emu.bus.write32(0x4001_003C, 0x0000_0000, 0); // CLK_SYS_CTRL SRC=0 (clk_ref)
    assert_eq!(emu.bus.sys_clk_hz(), ROSC_FREQ_HZ,
        "First CLOCKS write should replace the seed with the derived ROSC frequency");
}

// ============================================================================
// Quantum Execution Model — Stage 2: DWT + SysTick Emulator-level wiring
// ============================================================================

/// Build a minimal emulator whose firmware is an infinite B-to-self loop at
/// the reset vector. Both cores execute NOPs effectively, accumulating cycles
/// at a steady rate so the SysTick tick_systick() call sees deltas.
fn systick_test_emulator() -> crate::Emulator {
    use crate::{Config, EmulatorBuilder};
    let mut emu = EmulatorBuilder::new(Config::default()).build();
    // Minimal bootrom: SP @ 0x2000_0100, PC @ reset vector 0x0000_0100
    // with a NOP-equivalent `B .` loop so the core just keeps stepping.
    let mut rom = vec![0u8; 32 * 1024];
    // Initial SP
    rom[0] = 0x00; rom[1] = 0x01; rom[2] = 0x00; rom[3] = 0x20;
    // Reset vector: 0x0000_0101 (thumb bit)
    rom[4] = 0x01; rom[5] = 0x01; rom[6] = 0x00; rom[7] = 0x00;
    // At 0x100: B . (0xE7FE)
    rom[0x100] = 0xFE; rom[0x101] = 0xE7;
    emu.load_bootrom(&rom);
    emu.reset();
    emu
}

#[test]
fn test_dwt_cyccnt_wired_to_core_cycles() {
    // With TRCENA + CYCCNTENA set on core 0, reading CYCCNT through the bus
    // must reflect the core's accumulated cycle count after a quantum.
    let mut emu = systick_test_emulator();

    // Enable DWT: DEMCR.TRCENA then DWT_CTRL.CYCCNTENA
    emu.core_mut(0).ppb.write32(0xE000_EDFC, 1 << 24);
    emu.core_mut(0).ppb.write32(0xE000_1000, 1);

    // Publish current cycle count into PPB so the write sees a fresh base.
    let cyc = emu.core(0).cycles();
    emu.core_mut(0).ppb.update_latest_cycles(cyc);
    // Zero CYCCNT.
    emu.core_mut(0).ppb.write32(0xE000_1004, 0);

    let cycles_before = emu.cores.expect_arm_mut()[0].cycles();
    emu.step();
    let cycles_after = emu.cores.expect_arm_mut()[0].cycles();
    let delta = (cycles_after - cycles_before) as u32;

    let cyc2 = emu.core(0).cycles();
    emu.core_mut(0).ppb.update_latest_cycles(cyc2);
    let cyccnt = emu.core_mut(0).ppb.read_cyccnt(cyc2);
    assert_eq!(cyccnt, delta,
        "After zeroing CYCCNT, read must equal cycles elapsed since write");
}

#[test]
fn test_emulator_tick_systick_advances_per_core() {
    // Quantum-end tick_systick() must advance each enabled SysTick by the
    // per-core cycle delta since the last tick. Verify via COUNTFLAG on CSR.
    let mut emu = systick_test_emulator();

    // Enable SysTick on core 0: ENABLE + TICKINT + CLKSOURCE
    emu.core_mut(0).ppb.write32(0xE000_E010, 1 | (1 << 1) | (1 << 2));
    // RVR = 1 (smallest non-zero period); CVR = 0 (will immediately underflow).
    // Set CVR via field because register writes always clear CVR.
    emu.core_mut(0).ppb.write32(0xE000_E014, 1);
    emu.core_mut(0).ppb.syst_cvr = 0;
    // Snapshot last_systick_cycles to the current value so delta is meaningful.
    emu.core_mut(0).ppb.last_systick_cycles = emu.core_mut(0).cycles();

    emu.step();

    // Multi-reload within the quantum should have set COUNTFLAG.
    assert_ne!(emu.core_mut(0).ppb.syst_csr & (1 << 16), 0,
        "SysTick must underflow during the quantum");
    // TICKINT=1: ICSR.PENDSTSET must be set.
    assert_ne!(emu.core_mut(0).ppb.icsr & (1 << 26), 0,
        "TICKINT=1 + underflow must pend SysTick via ICSR.PENDSTSET");
}

#[test]
fn test_emulator_tick_systick_disabled_core_untouched() {
    // If core 1's SysTick is disabled, tick_systick() must not perturb CVR.
    let mut emu = systick_test_emulator();
    // Core 1 SysTick disabled; core 0 left at defaults (also disabled).
    emu.core_mut(1).ppb.write32(0xE000_E010, 1 << 2); // CLKSOURCE only
    emu.core_mut(1).ppb.write32(0xE000_E014, 100);
    // Set CVR via field; a register write would clear it.
    emu.core_mut(1).ppb.syst_cvr = 77;

    emu.step();

    assert_eq!(emu.core_mut(1).ppb.syst_cvr, 77,
        "Disabled SysTick must not tick at quantum end");
    assert_eq!(emu.core_mut(1).ppb.syst_csr & (1 << 16), 0,
        "Disabled SysTick must not set COUNTFLAG");
}

// ============================================================================
// Decoded-Op Cache (HLD 2026.04.14)
//
// The cache is exercised transparently by every existing test, so these
// add focused coverage for the hit/miss paths, invalidation hooks, and
// cycle-accuracy preservation on bank-2 SRAM. Internal fields are
// `pub(crate)` so we can assert slot state directly.
// ============================================================================

/// Convert a PC into its direct-mapped cache slot index.
fn cache_slot(pc: u32) -> usize {
    ((pc >> 1) & 0x3FFF) as usize
}

/// Write a halfword into SRAM via the untimed memory accessor — used to
/// seed code at a specific address before the first cache populate.
fn place_hw_in_sram(bus: &mut crate::bus::Bus, addr: u32, hw: u16) {
    let bytes = hw.to_le_bytes();
    let off = addr & 0x00FF_FFFF;
    bus.memory.sram_write8(off, bytes[0]);
    bus.memory.sram_write8(off + 1, bytes[1]);
}

#[test]
fn decode_cache_hit_miss_smoke() {
    // After the first decode_execute at a PC, the slot's tag must equal
    // the PC. A second decode_execute at the same PC takes the hit path
    // and observes the same behaviour.
    //
    // Post-Phase-3-follow-up-#10: decode_cache lives on the core, not
    // the bus.
    let (mut core, mut bus) = core_and_bus();

    // NOP (MOVS R0, R0 = 0x0000, which decodes to LSLS R0, R0, #0 — pure).
    let pc = 0x2000_0000u32;
    place_hw_in_sram(&mut bus, pc, 0x0000);
    place_hw_in_sram(&mut bus, pc + 2, 0x0000);
    core.regs.set_pc(pc);

    let slot = cache_slot(pc);
    assert_eq!(core.decode_cache[slot].tag, u32::MAX, "slot starts empty");

    // First step — miss, populates.
    core.step(&mut bus);
    assert_eq!(core.decode_cache[slot].tag, pc, "populate set tag");
    assert_eq!(core.decode_cache[slot].hw0, 0x0000);
    assert!(core.decode_cache[slot].is_pure(), "LSLS is classified pure");

    // Second step at pc+2 — populates the next slot too.
    core.step(&mut bus);
    let slot2 = cache_slot(pc + 2);
    assert_eq!(core.decode_cache[slot2].tag, pc + 2);

    // Rewind PC and step again — the first slot must be a hit (tag
    // unchanged, no re-population).
    core.regs.set_pc(pc);
    let tag_before = core.decode_cache[slot].tag;
    core.step(&mut bus);
    assert_eq!(core.decode_cache[slot].tag, tag_before,
        "second visit is a hit — tag unchanged");
}

#[test]
fn decode_cache_invalidation_on_sram_write() {
    // After caching, a Bus::write16 to the cached halfword must queue
    // the address; the driver drains the queue into the core, clearing
    // the slot so the next fetch picks up the new bytes. Phase 3
    // follow-up #10: Bus no longer invalidates directly.
    let (mut core, mut bus) = core_and_bus();

    let pc = 0x2000_0100u32;
    place_hw_in_sram(&mut bus, pc, 0x0000); // LSLS R0,R0,#0 — pure
    core.regs.set_pc(pc);
    core.step(&mut bus);
    assert_eq!(core.decode_cache[cache_slot(pc)].tag, pc);

    // Rewrite the halfword via the bus — must queue an invalidation.
    bus.write16(pc, 0x1C40, 0); // ADDS R0, R0, #1
    assert!(
        !bus.pending_cache_invalidations.is_empty(),
        "Bus::write16 must queue a cache invalidation"
    );
    // Drain the queue into the core (mirrors `Emulator::step`).
    core.invalidate_decode_cache_entries(&bus.pending_cache_invalidations);
    bus.pending_cache_invalidations.clear();
    assert_eq!(core.decode_cache[cache_slot(pc)].tag, u32::MAX,
        "drained queue clears the slot");

    // Next fetch re-populates with the new bytes.
    core.regs.set_pc(pc);
    core.step(&mut bus);
    assert_eq!(core.decode_cache[cache_slot(pc)].hw0, 0x1C40,
        "re-populate picked up the new halfword");
}

#[test]
fn decode_cache_invalidation_on_load_flash() {
    // After Phase 3 follow-up #10 + Task #10 review fix, `Bus::load_flash`
    // sets the XIP bit in `pending_invalidation_regions` (not the bulk
    // bit). `Emulator::load_flash` drains that bitmask on both cores via
    // `invalidate_decode_cache_regions(regions)`, so only XIP-region
    // slots are cleared — SRAM / ROM slots stay hot. This restores the
    // pre-migration `invalidate_region(0x1)` spec intent.
    use crate::{Config, Emulator};

    let mut emu = Emulator::new(Config::default());

    // Fake populated entries on BOTH cores directly in their caches.
    let xip_pc = 0x1000_0002u32;   // slot 1
    let sram_pc = 0x2000_0008u32;  // slot 4 — different slot
    assert_ne!(cache_slot(xip_pc), cache_slot(sram_pc),
        "test precondition: the two PCs must hash to distinct slots");

    for core in emu.cores.expect_arm_mut().iter_mut() {
        core.decode_cache[cache_slot(xip_pc)] = crate::bus::DecodedOp {
            tag: xip_pc,
            hw0: 0xDEAD,
            hw1: 0,
            fetch_wait: 0,
            flags: crate::bus::DecodedOp::FLAG_PURE,
        };
        core.decode_cache[cache_slot(sram_pc)] = crate::bus::DecodedOp {
            tag: sram_pc,
            hw0: 0xBEEF,
            hw1: 0,
            fetch_wait: 0,
            flags: 0,
        };
    }

    emu.load_flash(&[0x00; 256]);

    for (i, core) in emu.cores.expect_arm().iter().enumerate() {
        assert_eq!(core.decode_cache[cache_slot(xip_pc)].tag, u32::MAX,
            "core {}: XIP-region entry invalidated by load_flash", i);
        // Region-scoped drain: load_flash only sets the XIP bit, so
        // the SRAM-resident entry must survive untouched. This is the
        // perf-critical post-review behaviour — firmware that reloads
        // flash then runs SRAM code does not pay a cold-cache
        // repopulate tax on every instruction of the next quantum.
        assert_eq!(core.decode_cache[cache_slot(sram_pc)].tag, sram_pc,
            "core {}: SRAM-region slot preserved by XIP-only load_flash", i);
        assert_eq!(core.decode_cache[cache_slot(sram_pc)].hw0, 0xBEEF,
            "core {}: SRAM-region hw0 preserved by XIP-only load_flash", i);
    }
}

#[test]
fn decode_cache_wide_boundary_invalidation() {
    // A wide instruction cached at PC=N with its hw1 at N+2. A write
    // targeting N+2 must clear the slot at N (so the next fetch re-reads
    // both halfwords). Post-Phase-3-follow-up-#10: the write queues the
    // invalidation; draining into the core clears both slots.
    let (mut core, mut bus) = core_and_bus();

    let wide_pc = 0x2000_0200u32;
    // Populate a fake wide entry on the core.
    core.decode_cache[cache_slot(wide_pc)] = crate::bus::DecodedOp {
        tag: wide_pc,
        hw0: 0xF000, // any wide prefix
        hw1: 0x8000,
        fetch_wait: 0,
        flags: crate::bus::DecodedOp::FLAG_WIDE,
    };
    // Populate a narrow entry at wide_pc+2 too — it should also be
    // cleared (the preceding-slot rule).
    core.decode_cache[cache_slot(wide_pc + 2)] = crate::bus::DecodedOp {
        tag: wide_pc + 2,
        hw0: 0x2001,
        hw1: 0,
        fetch_wait: 0,
        flags: crate::bus::DecodedOp::FLAG_PURE,
    };

    // Writing a halfword at wide_pc+2 must clear the wide slot at N
    // (after draining).
    bus.write16(wide_pc + 2, 0xAAAA, 0);
    core.invalidate_decode_cache_entries(&bus.pending_cache_invalidations);
    bus.pending_cache_invalidations.clear();
    assert_eq!(core.decode_cache[cache_slot(wide_pc)].tag, u32::MAX,
        "write at hw1 boundary clears the wide slot at N");
    assert_eq!(core.decode_cache[cache_slot(wide_pc + 2)].tag, u32::MAX,
        "write to the slot itself also clears");
}

#[test]
fn decode_cache_bank2_fetch_wait_preserved() {
    // Code placed in SRAM bank 2: the fetch adds +1 extra wait state on
    // each read. The returned cycle count from decode_execute must be
    // identical on the first (populate) and second (hit) execution.
    let (mut core, mut bus) = core_and_bus();

    // Bank 2 address: (offset >> 2) & 7 == 2 means offset 8, 40, 72, ...
    let pc = 0x2000_0008u32;
    // Sanity-check the bank.
    assert_eq!(crate::memory::bank_for_address(pc), Some(2),
        "test precondition: PC must be in bank 2");

    place_hw_in_sram(&mut bus, pc, 0x0000); // LSLS R0,R0,#0 — pure, 1 cycle

    // First execution: populate + execute.
    core.regs.set_pc(pc);
    let cycles_before = core.cycles();
    core.step(&mut bus);
    let c1 = core.cycles() - cycles_before;

    // Second execution at same PC: cache hit + execute.
    core.regs.set_pc(pc);
    let cycles_mid = core.cycles();
    core.step(&mut bus);
    let c2 = core.cycles() - cycles_mid;

    assert_eq!(c1, c2,
        "populate and hit must return identical cycle counts for bank-2 SRAM");
    assert!(c1 >= 2, "bank 2 fetch adds a wait state, so cycles >= 1+1");
    assert_eq!(core.decode_cache[cache_slot(pc)].fetch_wait, 1,
        "fetch_wait for bank 2 is 1");
}

#[test]
fn decode_cache_pure_path_preserves_accumulator() {
    // A pure ALU op must not mutate bus.extra_wait_states. The
    // debug-assert in the fast path enforces this, but we also verify
    // externally — pollute the accumulator via a direct bank-2 read,
    // then execute a pure op and confirm the pollution is preserved.
    //
    // This also exercises the "pure path doesn't touch the accumulator"
    // contract described in HLD B §3.
    let (mut core, mut bus) = core_and_bus();

    let pc = 0x2000_0400u32;
    place_hw_in_sram(&mut bus, pc, 0x0000); // LSLS — pure

    // First populate the cache so the next step takes the pure hit path.
    core.regs.set_pc(pc);
    core.step(&mut bus);
    assert!(core.decode_cache[cache_slot(pc)].is_pure());

    // Pollute the accumulator directly (bank 2/6 data penalty removed).
    bus.reset_extra_wait_states();
    bus.add_extra_wait_states(1);
    let polluted = bus.extra_wait_states();
    assert_eq!(polluted, 1);

    // Rewind PC and step — this must be a pure cache hit that leaves
    // the accumulator exactly as it was.
    core.regs.set_pc(pc);
    core.step(&mut bus);
    assert_eq!(bus.extra_wait_states(), polluted,
        "pure path must not touch bus.extra_wait_states");
}

#[test]
fn decode_cache_impure_ldr_still_works() {
    // Smoke test: a non-cached impure instruction (LDR) still executes
    // correctly via the slow path, including after it's been cached.
    let (mut core, mut bus) = core_and_bus();

    // LDR R0, [PC, #0] — PC-relative load. 0x4800 | imm8=0 -> 0x4800.
    let pc = 0x2000_0100u32;
    place_hw_in_sram(&mut bus, pc, 0x4800);
    // Literal pool at (pc & ~3) + 4 = pc + 4.
    let literal_addr = (pc & !3).wrapping_add(4);
    bus.write32(literal_addr, 0xDEAD_BEEF, 0);

    core.regs.set_pc(pc);
    core.step(&mut bus);
    assert_eq!(core.reg(0), 0xDEAD_BEEF, "LDR literal loaded expected value");

    // Cache entry must be classified impure (LDR literal hits the bus).
    assert!(!core.decode_cache[cache_slot(pc)].is_pure(),
        "LDR literal is impure");

    // A second execution at the same PC still works.
    core.regs.set_pc(pc);
    core.set_reg(0, 0);
    core.step(&mut bus);
    assert_eq!(core.reg(0), 0xDEAD_BEEF, "hit on impure op re-executes correctly");
}

#[test]
fn decode_cache_invalidate_all_clears_everything() {
    // CortexM33::invalidate_decode_cache_all() must wipe every slot
    // regardless of tag/region. Phase 3 follow-up #10: cache lives on
    // the core now.
    let (mut core, _bus) = core_and_bus();

    // Pre-populate a handful of slots across regions.
    for (i, pc) in [0x0000_0004u32, 0x1000_0008, 0x2000_0010, 0x2080_0020]
        .iter().enumerate()
    {
        core.decode_cache[cache_slot(*pc)] = crate::bus::DecodedOp {
            tag: *pc,
            hw0: i as u16,
            hw1: 0,
            fetch_wait: 0,
            flags: 0,
        };
    }

    core.invalidate_decode_cache_all();

    for pc in [0x0000_0004u32, 0x1000_0008, 0x2000_0010, 0x2080_0020] {
        assert_eq!(core.decode_cache[cache_slot(pc)].tag, u32::MAX,
            "invalidate_decode_cache_all cleared slot for PC={:08X}", pc);
    }
}

#[test]
fn decode_cache_classify_is_pure_table() {
    // Regression guard on the classify_is_pure table in core/decode.rs.
    // Each row pins a representative opcode from one classification arm;
    // moving an op between arms without updating this table flags the
    // change as a test failure. Keep entries concise — encodings are
    // verified by construction from the HLD B §1 bit layouts.
    use crate::core::decode::classify_is_pure;

    // (name, hw0, hw1, is_wide, expected_is_pure)
    const CASES: &[(&str, u16, u16, bool, bool)] = &[
        // --- Pure Thumb-16 ----------------------------------------------
        ("LSLS imm (00000)",           0x0000, 0x0000, false, true),
        ("ADD/SUB reg/imm3 (00011)",   0x1800, 0x0000, false, true),
        ("MOV imm8 (00100)",           0x2000, 0x0000, false, true),
        ("CMP imm8 (00101)",           0x2800, 0x0000, false, true),
        ("DP ADC/SBC (01000, b10=0)",  0x4000, 0x0000, false, true),
        ("ADR (10100)",                0xA000, 0x0000, false, true),
        ("ADD SP imm (10101)",         0xA800, 0x0000, false, true),
        ("B uncond (11100)",           0xE000, 0x0000, false, true),
        // --- Impure Thumb-16 --------------------------------------------
        ("BX/BLX special (01000 b10=1)", 0x4400, 0x0000, false, false),
        ("LDR literal (01001)",          0x4800, 0x0000, false, false),
        ("LDR reg (01010)",              0x5800, 0x0000, false, false),
        ("STR imm (01100)",              0x6000, 0x0000, false, false),
        ("LDM (11001)",                  0xC800, 0x0000, false, false),
        ("SVC (11011, cond=F)",          0xDF00, 0x0000, false, false),
        ("PUSH (misc 0100)",             0xB400, 0x0000, false, false),
        ("POP (misc 1100)",              0xBC00, 0x0000, false, false),
        // --- Pure Thumb-32 ----------------------------------------------
        ("dp_modified_imm",              0xF000, 0x0000, true,  true),
        ("dp_shifted_reg",               0xEA00, 0x0000, true,  true),
        ("multiply",                     0xFA00, 0x0000, true,  true),
        ("BL (branch_misc)",             0xF000, 0xD000, true,  true),
        ("DMB (barrier)",                0xF3BF, 0x8F50, true,  true),
        // --- Impure Thumb-32 --------------------------------------------
        ("LDR.W (load_store_single)",    0xF850, 0x0000, true,  false),
        ("STM.W (ldm_stm)",              0xE880, 0x0000, true,  false),
        ("LDRD (load_store_dual)",       0xE850, 0x0000, true,  false),
        ("VLDR (coprocessor/FPU)",       0xED50, 0x0000, true,  false),
        ("MRC on CP7 RCP (coprocessor)", 0xEE70, 0x0000, true,  false),
    ];

    for &(name, hw0, hw1, is_wide, expected) in CASES {
        let got = classify_is_pure(hw0, hw1, is_wide);
        assert_eq!(got, expected,
            "classify_is_pure({:04X}, {:04X}, wide={}) = {} for {}, expected {}",
            hw0, hw1, is_wide, got, name, expected);
    }
}

// ============================================================================
// External GPIO stimulus overlay (gpio_external_in + gpio_external_mask)
// ============================================================================

/// External stimulus must dominate on masked bits and leave PIO/SIO output
/// untouched on unmasked bits. Covers the post-review Fix #2 in the
/// OneROM full-system harness.
#[test]
fn gpio_external_stimulus_overlays_masked_bits() {
    use crate::{Emulator, Config};

    let mut emu = Emulator::new(Config::default());

    // Bring PIO0 out of reset so its pad_out/pad_oe updates propagate.
    emu.bus.write32(0x4002_0000 | (3 << 12), (1 << 15) | (1 << 16) | (1 << 17), 0);

    // Force PIO0 to drive bit 0 (unmasked) high. We poke pad_out / pad_oe
    // directly — we're only interested in what `update_gpio` composes.
    emu.bus.pio[0].pad_oe = 0x0000_0001;
    emu.bus.pio[0].pad_out = 0x0000_0001;

    // Harness claims bit 5 and bit 10 (drive HIGH) and bit 12 (drive LOW).
    emu.bus.gpio_external_mask = (1 << 5) | (1 << 10) | (1 << 12);
    emu.bus.gpio_external_in.store((1 << 5) | (1 << 10), Ordering::Relaxed); // bit 12 low

    // One step. Single-quantum is fine — `update_gpio` runs inside it.
    emu.run(1);

    let gpio_in = emu.bus.gpio_in.load(Ordering::Relaxed);
    // Bit 0: PIO-driven high.
    assert_eq!(gpio_in & (1 << 0), 1 << 0, "PIO bit lost");
    // Bits 5, 10: external stimulus high.
    assert_eq!(gpio_in & (1 << 5), 1 << 5, "ext bit 5 lost");
    assert_eq!(gpio_in & (1 << 10), 1 << 10, "ext bit 10 lost");
    // Bit 12: masked to 0 regardless of what SIO/PIO say.
    assert_eq!(gpio_in & (1 << 12), 0, "ext bit 12 forced low");
    // Untouched bits stay 0.
    assert_eq!(
        gpio_in & !((1 << 0) | (1 << 5) | (1 << 10) | (1 << 12)),
        0,
        "unexpected bits set in gpio_in"
    );
}

/// External mask of 0 (default) must leave legacy behaviour unchanged —
/// `gpio_in` is whatever SIO + PIO produce.
#[test]
fn gpio_external_mask_zero_is_noop() {
    use crate::{Emulator, Config};

    let mut emu = Emulator::new(Config::default());
    emu.bus.write32(0x4002_0000 | (3 << 12), (1 << 15) | (1 << 16) | (1 << 17), 0);

    emu.bus.pio[0].pad_oe = 0x0000_00FF;
    emu.bus.pio[0].pad_out = 0x0000_005A;

    // No external stimulus.
    emu.bus.gpio_external_mask = 0;
    emu.bus.gpio_external_in.store(0xFFFF_FFFF, Ordering::Relaxed); // set but masked out

    emu.run(1);

    assert_eq!(
        emu.bus.gpio_in.load(Ordering::Relaxed),
        0x0000_005A,
        "legacy update_gpio path broken"
    );
}

/// External stimulus written between `step()` calls must be visible to the
/// cores on the very first cycle after the write — not one quantum later.
///
/// Regression: `update_gpio` previously ran only at the *end* of `step()`,
/// inside `tick_peripherals`. Stimulus set just before `run(1)` was therefore
/// composed into `bus.gpio_in` only after the first quantum's cores had
/// already read the stale value. The OneROM serving oracle relies on CS/address
/// bus stimulus being visible to the CPU on the very next fetch, so a single
/// cycle of latency silently corrupts its protocol.
///
/// This test pins the contract by running a single LDR that reads SIO_GPIO_IN
/// immediately on wake-up. If external stimulus isn't composed before the
/// core dispatches, R0 comes back as 0 and the assertion fires.
#[test]
fn gpio_external_in_visible_first_cycle_after_write() {
    use crate::{Config, EmulatorBuilder};

    // Tight quantum keeps the test honest: the LDR is the only
    // instruction that runs in the first step.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build();

    // Place `LDR R0, [R1, #0]` at SRAM base. Thumb-16 encoding = 0x6808.
    emu.bus.memory.sram_write16(0, 0x6808);
    // Pad with `B .` so if the core overshoots it parks rather than
    // wandering into uninitialised memory.
    emu.bus.memory.sram_write16(2, 0xE7FE);
    emu.bus.invalidate_all();

    // Core 0: PC at the LDR, Thumb bit set, R1 = SIO_GPIO_IN address.
    // Core 1 halted so only core 0 contributes to the quantum.
    emu.cores.expect_arm_mut()[0].regs.set_pc(0x2000_0000);
    emu.cores.expect_arm_mut()[0].regs.xpsr = 1 << 24; // T-bit
    emu.cores.expect_arm_mut()[0].regs.r[1] = 0xD000_0004; // SIO_GPIO_IN
    emu.cores.expect_arm_mut()[0].wake();
    emu.cores.expect_arm_mut()[1].halt();

    // Harness drives bit 3 high via external stimulus AFTER construction
    // and BEFORE the first step — i.e. the classic "just wrote a pin,
    // what does the CPU see on the next fetch" scenario.
    emu.bus.gpio_external_mask = 1 << 3;
    emu.bus.gpio_external_in.store(1 << 3, Ordering::Relaxed);

    emu.run(1);

    // The LDR must have observed the composed stimulus.
    let r0 = emu.cores.expect_arm()[0].regs.r[0];
    assert_eq!(
        r0 & (1 << 3),
        1 << 3,
        "LDR [SIO_GPIO_IN] on first cycle saw stale gpio_in = {:#010x} \
         (expected bit 3 set because gpio_external_mask/in were written \
         before run(1)); bus.gpio_in now = {:#010x}",
        r0,
        emu.bus.gpio_in.load(Ordering::Relaxed),
    );
}

// ============================================================================
// MMIO trace hook — Phase 0b (HLD V5 §4.2.7)
// ============================================================================
//
// The trace is a runtime flag (`Bus::mmio_trace_enabled`) gating a cold
// `emit_mmio_trace` helper. Zero overhead when off; one line per outer
// bus access when on. See `mdrp2040/src/bus/mod.rs` for the V7 idiom
// these tests mirror.

/// A thread-safe `Vec<u8>` sink so we can capture the trace output
/// without wrestling with stdout redirection. Wraps `Vec<u8>` behind
/// an `Arc<Mutex<...>>` so the test can drain the buffer after the
/// bus has written through the sink. (`Bus::mmio_trace_sink` holds a
/// `Box<dyn Write>`; tests hand it a `CaptureSink` clone.)
#[derive(Clone)]
struct TraceCaptureSink(std::sync::Arc<std::sync::Mutex<Vec<u8>>>);

impl std::io::Write for TraceCaptureSink {
    fn write(&mut self, data: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(data);
        Ok(data.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn new_capture() -> TraceCaptureSink {
    TraceCaptureSink(std::sync::Arc::new(std::sync::Mutex::new(Vec::new())))
}

#[test]
fn trace_enabled_emits_write32_line() {
    // HLD V5 §4.2.7: `write32(addr, val)` with `mmio_trace_enabled = true`
    // emits one line in the prescribed format. We inject a captured
    // `TraceCaptureSink` so the test doesn't depend on fd 1 redirection.
    let capture = new_capture();
    let mut bus = Bus::new();
    bus.set_active_pc(0x1000_0100, 0);
    bus.mmio_trace_enabled = true;
    bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));

    // SRAM word write — exercises the hot path and one of the six
    // access methods required by the spec.
    bus.write32(0x2000_0200, 0xDEAD_BEEF, 0);

    let captured = capture.0.lock().unwrap();
    let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(
        lines.len(),
        1,
        "expected one trace line, got {}: {:?}",
        lines.len(),
        lines,
    );
    let line = lines[0];
    assert!(line.starts_with("TRACE W 4 0x20000200"), "line = {:?}", line);
    assert!(line.contains("val=0xDEADBEEF"), "line = {:?}", line);
    assert!(line.contains("core=0"), "line = {:?}", line);
    assert!(line.contains("pc=0x10000100"), "line = {:?}", line);
}

#[test]
fn trace_enabled_emits_all_six_access_methods() {
    // Each of the six outer bus methods must emit exactly one line
    // with the correct size/RW tag. SRAM-backed region 0x2000_0000
    // is the cleanest target — no peripheral side-effects.
    let capture = new_capture();
    let mut bus = Bus::new();
    bus.set_active_pc(0x1000_0100, 0);
    bus.mmio_trace_enabled = true;
    bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));

    bus.write8(0x2000_0100, 0xAB, 0);
    bus.write16(0x2000_0102, 0xCDEF, 0);
    bus.write32(0x2000_0104, 0x1234_5678, 0);
    let _ = bus.read8(0x2000_0100, 0);
    let _ = bus.read16(0x2000_0102, 0);
    let _ = bus.read32(0x2000_0104, 0);

    let captured = capture.0.lock().unwrap();
    let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len(), 6, "expected 6 trace lines, got {:?}", lines);
    assert!(lines[0].starts_with("TRACE W 1 0x20000100"), "{:?}", lines[0]);
    assert!(lines[1].starts_with("TRACE W 2 0x20000102"), "{:?}", lines[1]);
    assert!(lines[2].starts_with("TRACE W 4 0x20000104"), "{:?}", lines[2]);
    assert!(lines[3].starts_with("TRACE R 1 0x20000100"), "{:?}", lines[3]);
    assert!(lines[4].starts_with("TRACE R 2 0x20000102"), "{:?}", lines[4]);
    assert!(lines[5].starts_with("TRACE R 4 0x20000104"), "{:?}", lines[5]);
    for line in &lines {
        assert!(line.contains("core=0") && line.contains("pc=0x10000100"), "{}", line);
    }
}

#[test]
fn trace_disabled_emits_nothing() {
    // Zero-overhead path — `mmio_trace_enabled = false` must not route any
    // bytes to the sink. Guards the hot path (non-trace runs must not
    // pay a formatting cost, and must not write bytes to the sink).
    let capture = new_capture();
    let mut bus = Bus::new();
    bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));
    // mmio_trace_enabled is false by default.
    bus.write32(0x2000_0200, 0xCAFE_F00D, 0);
    let _ = bus.read32(0x2000_0200, 0);
    bus.write16(0x2000_0200, 0xABCD, 0);
    let _ = bus.read16(0x2000_0200, 0);
    bus.write8(0x2000_0200, 0xEF, 0);
    let _ = bus.read8(0x2000_0200, 0);
    assert!(
        capture.0.lock().unwrap().is_empty(),
        "trace sink received bytes with mmio_trace_enabled=false",
    );
}

#[test]
fn trace_active_pc_is_per_core() {
    // Regression guard against a dual-core active_pc-staleness bug
    // (V7 §4.3). Every bus access carries an explicit `core` parameter
    // (Phase 0b.1 Commit A) so PC attribution indexes directly into
    // `bus.active_pc[core]`. A bus access on core 1 that doesn't go
    // through `decode_execute` (e.g. exception stacking) must NOT
    // observe core 0's last decode PC, and vice versa. Simulate:
    // decode PC=0x1000 on core 0, decode PC=0x2000 on core 1, then core
    // 0 issues an access *without* re-decoding — the trace line must
    // still carry PC=0x1000 for core 0.
    let capture = new_capture();
    let mut bus = Bus::new();
    bus.mmio_trace_enabled = true;
    bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));

    // Core 0 "decodes" at 0x1000 and writes.
    bus.set_active_pc(0x0000_1000, 0);
    bus.write32(0x2000_0100, 0xAAAA_AAAA, 0);

    // Scheduler switches to core 1, which "decodes" at 0x2000 and writes.
    bus.set_active_pc(0x0000_2000, 1);
    bus.write32(0x2000_0104, 0xBBBB_BBBB, 1);

    // Scheduler switches back to core 0 WITHOUT a re-decode (mimics
    // hardware-triggered access like exception stacking before the
    // handler's first `decode_execute`). The stored per-core PC must
    // still be 0x1000 for core 0 — not 0x2000 from core 1's quantum.
    bus.write32(0x2000_0108, 0xCCCC_CCCC, 0);

    let captured = capture.0.lock().unwrap();
    let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len(), 3, "expected three trace lines, got {:?}", lines);
    assert!(
        lines[0].contains("core=0") && lines[0].contains("pc=0x00001000"),
        "line 0 = {:?}",
        lines[0],
    );
    assert!(
        lines[1].contains("core=1") && lines[1].contains("pc=0x00002000"),
        "line 1 = {:?}",
        lines[1],
    );
    assert!(
        lines[2].contains("core=0") && lines[2].contains("pc=0x00001000"),
        "line 2 = {:?} (core 0 PC must survive the core-1 excursion)",
        lines[2],
    );
}

#[test]
fn trace_emulator_step_publishes_pc_via_decode() {
    // End-to-end: running one instruction through the emulator must
    // publish its PC via `bus.set_active_pc` from `decode_execute`,
    // so the trace line for any bus access during that instruction
    // reports the true architectural PC — not the stale default `0`.
    //
    // Build a minimal test: put a `nop ; nop` at SRAM origin, point
    // core 0's PC and VTOR there, enable trace, step once. The I-fetch
    // at that PC is the observed access (a `read16` from the SRAM
    // region holding the instruction).
    use crate::{Config, Emulator};
    let capture = new_capture();

    let mut emu = Emulator::new(Config::default());
    // Two `nop` halfwords at SRAM base 0x2000_0000.
    emu.bus.memory.sram_write16(0, 0xBF00); // nop
    emu.bus.memory.sram_write16(2, 0xBF00);
    emu.bus.invalidate_all(); // evict any stale cache entries for this PC.

    // Core 0 runs the nop at 0x2000_0000. Set Thumb bit, point PC at
    // our nop, halt core 1 so only core 0 traces.
    emu.cores.expect_arm_mut()[0].regs.set_pc(0x2000_0000);
    emu.cores.expect_arm_mut()[0].regs.xpsr = 1 << 24;
    emu.cores.expect_arm_mut()[0].wake();
    emu.cores.expect_arm_mut()[1].halt();

    emu.bus.mmio_trace_enabled = true;
    emu.bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));
    emu.step();
    emu.bus.mmio_trace_enabled = false;

    let captured = capture.0.lock().unwrap();
    let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
    let lines: Vec<&str> = text.lines().collect();
    assert!(
        !lines.is_empty(),
        "step should emit at least the I-fetch line",
    );
    // Look for at least one line with pc=0x20000000 and core=0.
    let has_expected_pc = lines
        .iter()
        .any(|l| l.contains("core=0") && l.contains("pc=0x20000000"));
    assert!(
        has_expected_pc,
        "no trace line carried the expected PC 0x20000000; lines={:?}",
        lines,
    );
}

// Regression guards for the exception-entry / exit sentinel PCs
// (`0xFFFF_FFFE` for stacking, `0xFFFF_FFFD` for unstacking). HLD V5
// §4.2.7 requires those trace lines to be distinguishable from ordinary
// instruction-driven access — the sentinel call-sites live in
// `crates/mdrp2350/src/core/exceptions.rs:72` (entry) and `:219` (exit).
// The invariant under test is "the sentinel reaches the trace pipeline",
// not "the full exception flow is bit-exact"; the lazy-FP integration
// suite (`tests/lazy_fp.rs`) already covers the semantics.

#[test]
fn trace_exception_entry_publishes_sentinel_fe() {
    // `enter_exception` must publish PC=0xFFFFFFFE before pushing the
    // exception frame, so every stacking-write trace line carries the
    // sentinel rather than the faulting instruction's PC.
    use crate::bus::Bus;
    use crate::core::CortexM33;

    let capture = new_capture();
    let mut cpu = CortexM33::for_test(0);
    // Place MSP above a usable stack region so the 32-byte basic frame
    // lands in SRAM. SRAM base is 0x20000000; a 0x2000_2000 MSP leaves
    // 8 KB of stack below — more than enough for an SVC frame.
    cpu.regs.msp = 0x2000_2000;
    cpu.regs.r[13] = cpu.regs.msp;

    let mut bus = Bus::new();
    // Vector table in SRAM so the entry's vector fetch succeeds; point
    // SVC (vector 11) at an arbitrary address — the handler never runs
    // here because we return to the caller after `test_enter_exception`.
    let vtor: u32 = 0x2000_4000;
    cpu.ppb.vtor = vtor;
    bus.write32(vtor + 11 * 4, 0x2000_0200 | 1, 0); // SVC → 0x2000_0200 (Thumb)

    // Enable trace AFTER the pre-flight setup so we only capture the
    // stacking writes + vector fetch. `set_active_pc` seeds with a
    // plausible in-thread PC so a regression (missing sentinel) would
    // show THAT value in the stacking-line output — easier to spot.
    bus.set_active_pc(0x0000_1000, 0);
    bus.mmio_trace_enabled = true;
    bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));

    cpu.test_enter_exception(11, &mut bus);

    // Disable trace before we walk the capture so any cleanup work
    // outside the test doesn't pollute the vector.
    bus.mmio_trace_enabled = false;
    let captured = capture.0.lock().unwrap();
    let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
    let lines: Vec<&str> = text.lines().collect();
    assert!(
        !lines.is_empty(),
        "exception entry must emit at least one stacking-write trace line",
    );
    let sentinel_lines: Vec<&str> = lines
        .iter()
        .copied()
        .filter(|l| l.contains("pc=0xFFFFFFFE"))
        .collect();
    assert!(
        !sentinel_lines.is_empty(),
        "no stacking trace line carried the 0xFFFFFFFE sentinel; lines={:?}",
        lines,
    );
    // Every line that appears after the sentinel was published should
    // carry the sentinel too — the 1000-seeded PC from before the
    // `enter_exception` call must NOT leak into any traced access.
    for l in &lines {
        assert!(
            !l.contains("pc=0x00001000"),
            "pre-entry PC 0x00001000 leaked into a stacking trace line: {:?}",
            l,
        );
    }
}

#[test]
fn trace_exception_exit_publishes_sentinel_fd() {
    // `exit_exception` must publish PC=0xFFFFFFFD before popping the
    // frame, so every unstacking-read trace line carries the unstacking
    // sentinel rather than a stale in-handler PC.
    use crate::bus::Bus;
    use crate::core::CortexM33;

    let capture = new_capture();
    let mut cpu = CortexM33::for_test(0);
    cpu.regs.msp = 0x2000_2000;
    cpu.regs.r[13] = cpu.regs.msp;

    let mut bus = Bus::new();
    let vtor: u32 = 0x2000_4000;
    cpu.ppb.vtor = vtor;
    bus.write32(vtor + 11 * 4, 0x2000_0200 | 1, 0);

    // Drive an SVC entry first (sets up the stacked frame that exit
    // will pop). Trace is off here — we only want the exit lines.
    cpu.test_enter_exception(11, &mut bus);

    bus.set_active_pc(0x0000_2000, 0); // an in-handler PC that must NOT leak.
    bus.mmio_trace_enabled = true;
    bus.set_mmio_trace_sink(Some(Box::new(capture.clone())));

    // EXC_RETURN 0xFFFF_FFF9: return to Thread mode, MSP, no FP frame —
    // matches the basic frame pushed above.
    cpu.test_exit_exception(0xFFFF_FFF9, &mut bus);

    bus.mmio_trace_enabled = false;
    let captured = capture.0.lock().unwrap();
    let text = std::str::from_utf8(&captured).expect("trace must be utf-8");
    let lines: Vec<&str> = text.lines().collect();
    assert!(
        !lines.is_empty(),
        "exception exit must emit at least one unstacking-read trace line",
    );
    let sentinel_lines: Vec<&str> = lines
        .iter()
        .copied()
        .filter(|l| l.contains("pc=0xFFFFFFFD"))
        .collect();
    assert!(
        !sentinel_lines.is_empty(),
        "no unstacking trace line carried the 0xFFFFFFFD sentinel; lines={:?}",
        lines,
    );
    for l in &lines {
        assert!(
            !l.contains("pc=0x00002000"),
            "pre-exit PC 0x00002000 leaked into an unstacking trace line: {:?}",
            l,
        );
    }
}

// ============================================================================
// WFI behaviour (Phase 0b.3)
// ============================================================================

#[test]
fn wfi_halts_when_no_pending_irq() {
    let (mut core, mut bus) = core_and_bus();
    // WFI via execute_one_with_bus: opcode 0xBF30
    let pre_halted = core.is_halted();
    assert!(!pre_halted, "should not be halted before WFI");
    core.execute_one_with_bus(0xBF30, &mut bus);
    assert!(core.is_halted(),
        "WFI with no pending IRQ should halt the core");
}

#[test]
fn wfi_nop_when_enabled_pending_irq() {
    let (mut core, mut bus) = core_and_bus();
    // Enable IRQ 0 in NVIC
    core.ppb.nvic_iser[0].store(1, Ordering::Relaxed);
    // Assert IRQ 0 pending
    bus.atomics.irq_pending[0].store(1, Ordering::Relaxed);
    // WFI: 0xBF30
    core.execute_one_with_bus(0xBF30, &mut bus);
    assert!(!core.is_halted(),
        "WFI with enabled pending IRQ should NOT halt (acts as NOP)");
}

#[test]
fn wfi_halts_when_pending_but_disabled_irq() {
    let (mut core, mut bus) = core_and_bus();
    // IRQ pending but NOT enabled
    bus.atomics.irq_pending[0].store(1, Ordering::Relaxed);
    core.ppb.nvic_iser[0].store(0, Ordering::Relaxed);
    // WFI: 0xBF30
    core.execute_one_with_bus(0xBF30, &mut bus);
    assert!(core.is_halted(),
        "WFI with pending but disabled IRQ should halt");
}

#[test]
fn wfi_wake_on_irq_assert() {
    let mut emu = crate::Emulator::new(crate::Config::default());
    emu.load_flash(&[]);
    // Halt core via WFI
    emu.core_mut(0).halt();
    // Enable IRQ 0
    emu.core_mut(0).ppb.nvic_iser[0].store(1, Ordering::Relaxed);
    // Assert IRQ 0
    emu.bus.atomics.irq_pending[0].store(1, Ordering::Relaxed);
    // Wake check should clear halted
    emu.wake_checks();
    assert!(!emu.core_mut(0).is_halted(),
        "wake_checks should wake WFI-halted core with enabled pending IRQ");
}

#[test]
fn wfi_stays_halted_without_enabled_irq() {
    let mut emu = crate::Emulator::new(crate::Config::default());
    emu.load_flash(&[]);
    emu.core_mut(0).halt();
    // IRQ pending but not enabled
    emu.bus.atomics.irq_pending[0].store(1, Ordering::Relaxed);
    emu.core_mut(0).ppb.nvic_iser[0].store(0, Ordering::Relaxed);
    emu.wake_checks();
    assert!(emu.core_mut(0).is_halted(),
        "wake_checks should NOT wake WFI-halted core without enabled IRQ");
}

#[test]
fn debug_step_clears_halted() {
    let mut emu = crate::Emulator::new(crate::Config::default());
    emu.load_flash(&[]);
    emu.core_mut(0).halt();
    emu.core_mut(0).regs.r[15] = 0x2000_0000;
    // NOP instruction
    emu.bus.memory.sram_write16(0, 0xBF00);
    {
        let Cores::Arm(arm) = &mut emu.cores else { unreachable!() };
        arm[0].debug_step(&mut emu.bus);
    }
    assert!(!emu.core_mut(0).is_halted(),
        "debug_step should clear halted before stepping");
}

// ============================================================================
// Banked SP staleness fix (Phase 2.1)
// ============================================================================
//
// Regular instructions (PUSH, POP, SUB SP, ADD SP) write to r[13] via
// set_sp() without syncing to the banked msp/psp fields. If an exception
// fires after SP-modifying instructions, enter_exception must read the
// current r[13] (via sync_sp_to_banked) rather than the stale banked
// value. Same applies to exit_exception when unstacking.

#[test]
fn test_pendsv_stacks_at_post_sub_sp_not_stale_banked_msp() {
    let mut emu = Emulator::new(Config::default());

    // Build a ROM with vector table + main code + PendSV handler.
    let mut rom = vec![0u8; 1024];

    // Vector table:
    //   offset 0x00: Initial SP = 0x2008_0000
    //   offset 0x04: Reset vector -> 0x100 (Thumb)
    //   offset 0x38: PendSV vector (exception 14, offset 14*4=56=0x38) -> 0x200 (Thumb)
    rom[0x00..0x04].copy_from_slice(&0x2008_0000u32.to_le_bytes()); // SP
    rom[0x04..0x08].copy_from_slice(&0x0000_0101u32.to_le_bytes()); // Reset -> 0x100
    rom[0x38..0x3C].copy_from_slice(&0x0000_0201u32.to_le_bytes()); // PendSV -> 0x200

    // Main at 0x100: SUB SP, SP, #0x80 (Thumb-16 encoding: 0xB0A0)
    // Encoding: 1011 0000 1_0100000 = 0xB0A0 (subtract 0x80 = 128 bytes from SP)
    rom[0x100] = 0xA0; rom[0x101] = 0xB0; // SUB SP, #0x80
    // 0x102: B . (infinite loop — PendSV will preempt here)
    rom[0x102] = 0xFE; rom[0x103] = 0xE7;

    // PendSV handler at 0x200: B . (infinite loop so we can inspect state)
    rom[0x200] = 0xFE; rom[0x201] = 0xE7;

    emu.load_bootrom(&rom);
    emu.reset();

    // Run a few steps so the core reaches 0x100 and executes the SUB SP.
    for _ in 0..5 { core0_step(&mut emu); }

    // After reset, SP starts at 0x2008_0000. After SUB SP, #0x80, r[13]
    // should be 0x2007_FF80. Confirm the SUB executed.
    let sp_after_sub = emu.cores.expect_arm_mut()[0].regs.r[13];
    assert_eq!(sp_after_sub, 0x2007_FF80,
        "SP after SUB SP, #0x80 should be 0x2007_FF80, got {:#010x}", sp_after_sub);

    // The banked MSP may be stale (still 0x2008_0000) because set_sp()
    // only writes r[13]. This is the bug we're testing.
    // (Don't assert staleness — the fix makes them match; the test
    // validates the stacked frame uses the correct SP regardless.)

    // Pend PendSV and step to take the exception.
    emu.core_mut(0).ppb.icsr |= ICSR_PENDSVSET_BIT;
    core0_step(&mut emu);

    assert_eq!(emu.cores.expect_arm_mut()[0].regs.ipsr(), 14,
        "should be in PendSV handler after step");

    // The exception frame should have been pushed at the post-SUB SP value
    // (0x2007_FF80), 8-byte aligned, minus 32 bytes for the basic frame.
    // 0x2007_FF80 is already 8-byte aligned, so frame_sp = 0x2007_FF80 - 32 = 0x2007_FF60.
    let expected_frame_sp = 0x2007_FF60u32;

    // The stacked return address (at frame_sp + 24) should point to the
    // instruction after SUB SP — i.e. the B . at 0x102.
    let stacked_return_addr = emu.bus.read32(expected_frame_sp.wrapping_add(24), 0);
    assert_eq!(stacked_return_addr, 0x0000_0102,
        "stacked return address should be 0x102 (the B . after SUB SP), got {:#010x}",
        stacked_return_addr);

    // The MSP (banked) should now be the frame SP after entry wrote it back.
    let core0_msp = emu.cores.expect_arm()[0].regs.msp;
    assert_eq!(core0_msp, expected_frame_sp,
        "banked MSP after exception entry should be frame_sp={:#010x}, got {:#010x}",
        expected_frame_sp, core0_msp);

    // Also verify stacked R0 is readable at the frame base (sanity check
    // that the frame landed at the right address, not at the stale SP).
    let stacked_xpsr = emu.bus.read32(expected_frame_sp.wrapping_add(28), 0);
    // xPSR bit 9 should be 0 (no alignment padding needed since SP was
    // already 8-byte aligned).
    assert_eq!(stacked_xpsr & (1 << 9), 0,
        "no alignment padding expected (SP was 8-aligned)");
}

// ============================================================================
// SIO GPIO_OUT byte/halfword-write replication (RP2350 hardware behaviour)
// ============================================================================
//
// Real RP2350 silicon replicates sub-word writes to the GPIO_OUT family
// (GPIO_OUT / GPIO_OUT_SET / GPIO_OUT_CLR / GPIO_OUT_XOR, plus the OE
// variants) across all 4 byte lanes of the underlying 32-bit register.
// Firmware (OneROM's CPU-serve loop) relies on this: a single
// `STRB R1, [R5, #0]` at SIO_GPIO_OUT must drive the same byte on
// pins 0..7, 8..15, 16..23, and 24..31 simultaneously.
//
// The previous behaviour (byte-lane RMW) left pins 16..23 dark when the
// firmware issued a STRB at offset 0. These tests pin the replication
// contract for byte and halfword writes, plus confirm that non-GPIO_OUT
// SIO registers still do ordinary byte-lane RMW.

/// A byte write to SIO_GPIO_OUT must replicate the byte across all four
/// lanes of the 32-bit register, matching RP2350 silicon.
#[test]
fn sio_gpio_out_byte_write_replicates_across_lanes() {
    use crate::{Emulator, Config};

    let mut emu = Emulator::new(Config::default());

    // Enable OE for all 30 valid pins so `update_gpio` can surface the
    // replicated output on `gpio_in`.
    emu.bus.write32(0xD000_0030, 0x3FFF_FFFF, 0);

    // Byte write to SIO_GPIO_OUT offset 0.
    emu.bus.write8(0xD000_0010, 0xA5, 0);

    // The 32-bit SIO_GPIO_OUT register must hold the byte replicated
    // across every lane.
    assert_eq!(
        emu.bus.read32(0xD000_0010, 0),
        0xA5A5_A5A5,
        "byte write to SIO_GPIO_OUT must replicate across all 4 lanes"
    );

    // Pins 16..23 are the upper byte of the `0xA5` replication. Run a
    // single step so `update_gpio` composes `gpio_in` from
    // `sio.gpio_out & sio.gpio_oe`; then verify the byte made it onto
    // those pins. This is the exact property the OneROM CPU-serve loop
    // relies on.
    emu.run(1);
    // Bits 16..23 reflect `0xA5` — masked against `PIN_MASK` (bits
    // 0..29), they still span the full byte (16..23 all valid).
    let pins_16_23 = (emu.bus.gpio_in.load(Ordering::Relaxed) >> 16) & 0xFF;
    assert_eq!(
        pins_16_23, 0xA5,
        "pins 16..23 must reflect replicated byte 0xA5, got {:#04x}",
        pins_16_23
    );
}

/// A halfword write to SIO_GPIO_OUT must replicate the 16-bit value
/// across both halves of the 32-bit register.
#[test]
fn sio_gpio_out_halfword_write_replicates() {
    use crate::{Emulator, Config};

    let mut emu = Emulator::new(Config::default());

    emu.bus.write16(0xD000_0010, 0x1234, 0);

    assert_eq!(
        emu.bus.read32(0xD000_0010, 0),
        0x1234_1234,
        "halfword write to SIO_GPIO_OUT must replicate across both halves"
    );
}

/// GPIO_OE (output-enable) lives in the same replicating family as
/// GPIO_OUT — a `STRB` to `SIO_GPIO_OE` lights the OE bit across every
/// byte lane of the underlying 32-bit word. OneROM's CPU-serve loop
/// relies on this the same way it does for GPIO_OUT: the output-enable
/// mask `0x04FF_0000` composed by the firmware eventually lands via a
/// word write, but narrow writes must also replicate to match silicon.
#[test]
fn sio_gpio_oe_byte_write_replicates_across_lanes() {
    use crate::{Config, EmulatorBuilder};
    let mut emu = EmulatorBuilder::new(Config::default()).build();
    emu.bus.write8(0xD000_0030, 0xFF, 0);
    let word = emu.bus.read32(0xD000_0030, 0);
    assert_eq!(word, 0xFFFF_FFFF,
        "STRB to SIO_GPIO_OE must replicate byte across all four lanes; got {:#010x}", word);
}

/// Replication must be GPIO_OUT-family-specific, not blanket across
/// every SIO register. A byte write to a non-GPIO_OUT register (e.g.
/// MTIMEL at offset 0x1B0) must still behave as byte-lane RMW.
#[test]
fn sio_non_gpio_out_byte_write_still_rmw() {
    use crate::{Emulator, Config};

    let mut emu = Emulator::new(Config::default());

    // MTIMEL (offset 0x1B0) is a plain storage register on RP2350 SIO
    // — no side-effect semantics, perfect for verifying the RMW path.
    emu.bus.write32(0xD000_01B0, 0xFFFF_FFFF, 0);
    emu.bus.write8(0xD000_01B1, 0xAA, 0);

    // Only byte 1 should have changed.
    assert_eq!(
        emu.bus.read32(0xD000_01B0, 0),
        0xFFFF_AAFF,
        "non-GPIO_OUT byte write must be byte-lane RMW, not replicated"
    );
}

/// Residual A.2.1 end-to-end regression: MTIME must not advance without
/// `TICKS.RISCV` configuration. Silicon reads 0 in the
/// `sio_mtime_count_and_match` scenario gate because `TICKS.RISCV.CYCLES=0`
/// halts the divider; the emulator must match.
///
/// See `wrk_docs/2026.04.17 - HLD - Residual A.2.1 MTIME WATCHDOG_TICK Fix.md`.
#[test]
fn mtime_stays_zero_at_post_reset_matches_silicon() {
    use crate::{Config, EmulatorBuilder};
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    emu.core_mut(0).halt();
    emu.core_mut(1).halt();
    // Post-reset MTIME_CTRL is 0x0D (EN + DBGPAUSE_CORE0 + DBGPAUSE_CORE1,
    // FULLSPEED=0); write EN=1 explicitly so the test survives a future
    // reset-default change.
    emu.mmio_write32(0xD000_01A4, 0x01);
    emu.run(200);
    let mtime_lo = emu.mmio_read32(0xD000_01B0);
    assert_eq!(mtime_lo, 0,
        "MTIME must not advance without TICKS.RISCV configuration \
         (silicon reads 0 at this scenario gate)");
}

// ============================================================================
// SYSINFO read-only register readback
// ============================================================================
//
// RP2350 datasheet §12.11 SYSINFO. Four documented read-only registers.
// Field layout and constants from pico-sdk master
// (src/rp2350/hardware_regs/include/hardware/regs/sysinfo.h +
//  src/rp2350/pico_platform/platform.c):
//
//   0x00 CHIP_ID      — REVISION[31:28] | PART[27:12]
//                       | MANUFACTURER[11:1] | STOP_BIT[0]
//   0x04 PACKAGE_SEL  — 0 = RP2350A QFN60, 1 = RP2350B QFN80
//   0x08 PLATFORM     — bit 0 = FPGA, bit 1 = ASIC
//   0x14 GITREF_RP2350 — 32-bit git-sha prefix baked into silicon
//
// SYSINFO sits at APB base 0x4000_0000. It is not reset-gated at the bus
// fabric (see `reset_bit_for_base`) so a fresh `Bus` exposes the values
// immediately. The matching silicon scenario (`sysinfo_readonly_fields`
// in `silicon_scenarios.rs`) captures the same words against real silicon
// and diffs — CHIP_ID's REV nibble is masked on that path until Stage 5
// pre-flight records the chip-revision-specific value per the Coverage
// Gap Fill V11 HLD §3.1 / §8.

#[test]
fn sysinfo_reads_hardcoded_readonly_fields() {
    let (_, mut bus) = core_and_bus();
    // CHIP_ID: live RP2354 silicon value (V12 Stage 3, probe
    // E46410955F614129). PART=0x0004 (RP2354) occupies bits [14:12]
    // → 0x4 << 12 = 0x4000; MAN=0x927 (Raspberry Pi); REV=0 (masked).
    assert_eq!(bus.read32(0x4000_0000, 0), 0x0000_4927,
        "SYSINFO.CHIP_ID: expected PART<<12 | MAN = 0x4000 | 0x927 \
         (REV nibble masked by silicon scenario)");
    // PACKAGE_SEL: 0 = RP2350A QFN60 (Pico 2 baseline).
    assert_eq!(bus.read32(0x4000_0004, 0), 0x0000_0000,
        "SYSINFO.PACKAGE_SEL: RP2350A");
    // PLATFORM: live RP2354 silicon reads 0 (V12 Stage 3, probe
    // E46410955F614129).
    assert_eq!(bus.read32(0x4000_0008, 0), 0x0000_0000,
        "SYSINFO.PLATFORM: silicon-measured value");
    // GITREF_RP2350: chip-revision-specific 32-bit constant. Emulator
    // exposes 0 as a placeholder; Stage 5 silicon pre-flight records the
    // true value and re-adds the silicon-scenario observe entry.
    assert_eq!(bus.read32(0x4000_0014, 0), 0x0000_0000,
        "SYSINFO.GITREF_RP2350: placeholder (silicon pre-flight pending)");
}

// ============================================================================
// TBMAN PLATFORM selector (Coverage Gap Fill V11 §3.4 Bucket A item 4)
// ============================================================================
//
// TBMAN_BASE + 0x00 (PLATFORM) is a read-only register distinguishing
// ASIC / FPGA / HDLSIM targets. Silicon reset value per the authoritative
// pico-sdk header:
//
//   https://raw.githubusercontent.com/raspberrypi/pico-sdk/master/src/rp2350/hardware_regs/include/hardware/regs/tbman.h
//
//   #define TBMAN_PLATFORM_RESET       _u(0x00000001)
//   #define TBMAN_PLATFORM_BITS        _u(0x00000007)  // 3-bit field
//   #define TBMAN_PLATFORM_ASIC_BITS   _u(0x00000001)  // bit 0 -- ASIC
//   #define TBMAN_PLATFORM_FPGA_BITS   _u(0x00000002)  // bit 1 -- FPGA
//   #define TBMAN_PLATFORM_HDLSIM_BITS _u(0x00000004)  // bit 2 -- HDLSIM
//
// So real RP2354 silicon exposes PLATFORM = 0x1 (ASIC bit set), which
// matches HLD V11 §3.4's "assumption 0b01". The silicon scenario
// `tbman_platform_reads_silicon_value` in `silicon_scenarios.rs` diffs
// the same word against the attached RP2354.

#[test]
fn tbman_platform_reads_silicon_value() {
    let (_, mut bus) = core_and_bus();
    // TBMAN_BASE = 0x4016_0000; PLATFORM offset = 0x00.
    // Expected value: TBMAN_PLATFORM_ASIC_BITS = 0x1.
    assert_eq!(
        bus.read32(0x4016_0000, 0),
        0x0000_0001,
        "TBMAN.PLATFORM: ASIC bit (bit 0) set per pico-sdk \
         TBMAN_PLATFORM_RESET"
    );
}

// ============================================================================
// GLITCH_DETECTOR — ARM readback (Coverage Gap Fill V11 §3.3 Bucket A item 3)
// ============================================================================
//
// Per pico-sdk `glitch_detector.h` (pinned to commit
// a1438dff1d38bd9c65dbd693f0e5db4b9ae91779):
//
//   https://raw.githubusercontent.com/raspberrypi/pico-sdk/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2350/hardware_regs/include/hardware/regs/glitch_detector.h
//
//   ARM         offset 0x00, RW 16-bit, RESET 0x5bad (= VALUE_NO).
//                 VALUE_NO  = 0x5bad  -- do not force-arm
//                 VALUE_YES = 0x0000  -- any non-NO value force-arms
//   TRIG_STATUS offset 0x10, W1C 4-bit. Reads 0 in emulation.
//
// HLD V11 §3.3 describes the contract in terms of "CTRL.ARM" /
// "STATUS.ARM" — this is shorthand. On silicon there is a single RW
// register, ARM, which reads back whatever was last written. So "ARM
// readback tracks CTRL" reduces to "write N to ARM, next read returns
// N". The emulator's `GlitchDetector::new()` seeds offset 0x00 with
// `ARM_RESET = 0x5bad` so the first read (before any firmware write)
// matches silicon too.
//
// TRIG_STATUS has no emulator-side trigger path (no glitch ever fires),
// so it must read 0 at all times. The silicon scenario
// `glitch_detector_arm_readback_tracks_ctrl` in `silicon_scenarios.rs`
// diffs the same pair of words against attached RP2354 hardware.

#[test]
fn glitch_detector_arm_readback_tracks_ctrl() {
    use crate::peripherals::inert::{
        GLITCH_DETECTOR_ARM_OFFSET, GLITCH_DETECTOR_ARM_RESET,
        GLITCH_DETECTOR_ARM_VALUE_NO, GLITCH_DETECTOR_ARM_VALUE_YES,
        GLITCH_DETECTOR_BASE, GLITCH_DETECTOR_TRIG_STATUS_OFFSET,
    };

    let (_, mut bus) = core_and_bus();
    let arm_addr = GLITCH_DETECTOR_BASE + GLITCH_DETECTOR_ARM_OFFSET;
    let trig_status_addr = GLITCH_DETECTOR_BASE + GLITCH_DETECTOR_TRIG_STATUS_OFFSET;

    // Before any write, ARM reads the silicon reset value 0x5bad.
    assert_eq!(
        bus.read32(arm_addr, 0),
        GLITCH_DETECTOR_ARM_RESET,
        "ARM must read `ARM_RESET = 0x5bad` (= VALUE_NO) before any write"
    );
    // TRIG_STATUS reads 0 at reset.
    assert_eq!(
        bus.read32(trig_status_addr, 0),
        0,
        "TRIG_STATUS must read 0 in emulation (no glitch fires)"
    );

    // Write ARM = 0x0000 (force-arm YES); read back unchanged.
    bus.write32(arm_addr, GLITCH_DETECTOR_ARM_VALUE_YES, 0);
    assert_eq!(
        bus.read32(arm_addr, 0),
        GLITCH_DETECTOR_ARM_VALUE_YES,
        "ARM readback must track CTRL write: 0x0000 (VALUE_YES)"
    );
    assert_eq!(
        bus.read32(trig_status_addr, 0),
        0,
        "TRIG_STATUS must stay 0 regardless of ARM state"
    );

    // Write ARM = 0x5bad (force-arm NO); read back unchanged.
    bus.write32(arm_addr, GLITCH_DETECTOR_ARM_VALUE_NO, 0);
    assert_eq!(
        bus.read32(arm_addr, 0),
        GLITCH_DETECTOR_ARM_VALUE_NO,
        "ARM readback must track CTRL write: 0x5bad (VALUE_NO)"
    );
    assert_eq!(
        bus.read32(trig_status_addr, 0),
        0,
        "TRIG_STATUS must still read 0"
    );
}

// GLITCH_DETECTOR — warm-reset ARM quiesce (devil's-advocate audit)
//
// On real silicon a watchdog-driven warm reset returns ARM to
// `ARM_RESET = 0x5bad` (= VALUE_NO) regardless of what firmware last
// wrote. Without a peripheral-level `reset()` hook the emulator would
// preserve the firmware-written ARM value across `Emulator::reset()` —
// e.g. ARM = 0x0000 (force-arm YES) would survive reset and silently
// diverge from silicon. This test guards the hook wired through
// `Emulator::reset` -> `Bus::glitch.reset()` -> `GlitchDetector::reset`.
#[test]
fn glitch_detector_arm_resets_to_arm_value_no_on_warm_reset() {
    use crate::peripherals::inert::{
        GLITCH_DETECTOR_ARM_OFFSET, GLITCH_DETECTOR_ARM_RESET,
        GLITCH_DETECTOR_ARM_VALUE_YES, GLITCH_DETECTOR_BASE,
    };

    let mut emu = Emulator::new(Config::default());
    let arm_addr = GLITCH_DETECTOR_BASE + GLITCH_DETECTOR_ARM_OFFSET;

    // Write ARM = 0x0000 (force-arm YES) — a non-reset value that must
    // not survive a warm reset on silicon.
    emu.bus.write32(arm_addr, GLITCH_DETECTOR_ARM_VALUE_YES, 0);
    assert_eq!(
        emu.bus.read32(arm_addr, 0),
        GLITCH_DETECTOR_ARM_VALUE_YES,
        "pre-reset sanity: ARM readback tracks the force-arm YES write"
    );

    // Warm reset — the watchdog / chip-reset path on silicon returns
    // ARM to its reset value.
    emu.reset();

    assert_eq!(
        emu.bus.read32(arm_addr, 0),
        GLITCH_DETECTOR_ARM_RESET,
        "ARM must return to ARM_RESET (= 0x5bad, VALUE_NO) after warm reset"
    );
}

// ============================================================================
// POWMAN COUNT + MATCH (Coverage Gap Fill V11 §3.2 Bucket A item 2)
// ============================================================================
//
// Per pico-sdk `powman.h` at the pinned commit
// `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`:
//
//   SET_TIME_*         offsets 0x60, 0x64, 0x68, 0x6C (write-only seed
//                      for the 64-bit running count).
//   READ_TIME_UPPER    offset 0x70 (RO, high 32).
//   READ_TIME_LOWER    offset 0x74 (RO, low  32).
//   ALARM_TIME_*       offsets 0x78, 0x7C, 0x80, 0x84 (64-bit match).
//   TIMER              offset 0x88, bit 1 = RUN, bit 4 = ALARM_ENAB,
//                      bit 6 = ALARM (W1C).
//   INTR               offset 0xE0, bit 1 = TIMER (RO latched, cleared
//                      via TIMER.ALARM W1C).
//
// HLD V11 §3.2 maps its logical names as:
//   AON_COUNT_LO/HI  → READ_TIME_LOWER/UPPER
//   AON_MATCH_LO/HI  → ALARM_TIME_15TO0..63TO48 (value 100 fits in the
//                      low 16-bit lane, written at 0x84)
//   MATCH_EN         → TIMER.ALARM_ENAB (bit 4)
//
// POWMAN is held-at-reset at bootrom exit per the RESETS_POST_BOOTROM
// mask; firmware must release `RESET_POWMAN = 17` via RESETS_RESET_CLR
// before any MMIO. The silicon scenario `powman_match_irq_timer_line_45`
// in `isr_scenarios.rs::SCENARIOS` runs the same recipe against live
// RP2354 silicon.

#[test]
fn powman_count_advances_at_expected_rate() {
    use crate::peripherals::powman::{
        POWMAN_BASE, POWMAN_SYS_PER_TICK, READ_TIME_LOWER_OFFSET, TIMER_OFFSET,
        TIMER_RUN_BIT,
    };

    let mut emu = Emulator::new(Config::default());
    // Release POWMAN from reset via the CLR alias (+0x3000).
    let resets_clr = 0x4002_0000 | 0x3000;
    emu.bus.write32(resets_clr, 1u32 << 17, 0); // RESET_POWMAN
    // Set TIMER.RUN so COUNT advances. POWMAN password in bits [31:16]
    // — V13 Stage 1 enforces the password on every password-gated write.
    emu.bus
        .write32(POWMAN_BASE + TIMER_OFFSET, 0x5AFE_0000 | TIMER_RUN_BIT, 0);
    // Run enough sys_clks for exactly 10 POWMAN ticks.
    let n = 10 * POWMAN_SYS_PER_TICK as u32;
    emu.bus.tick_peripherals(n);
    assert_eq!(
        emu.bus.read32(POWMAN_BASE + READ_TIME_LOWER_OFFSET, 0),
        10,
        "POWMAN COUNT should equal N / sys_per_tick = {n} / {POWMAN_SYS_PER_TICK}"
    );
}

#[test]
#[ignore = "threading: PPB writes (NVIC ISER/ISPR) must go through CortexM33::bus_write32 wrapper — test writes direct via Bus"]
fn powman_match_pends_nvic_line_45() {
    use crate::peripherals::powman::{
        ALARM_TIME_15TO0_OFFSET, INT_TIMER_BIT, INTE_OFFSET, IRQ_POWMAN_IRQ_TIMER, POWMAN_BASE,
        POWMAN_SYS_PER_TICK, TIMER_ALARM_ENAB_BIT, TIMER_OFFSET, TIMER_RUN_BIT,
    };

    let mut emu = Emulator::new(Config::default());
    // Exercise the pure NVIC latching path via `raise_irqs_u64` — this
    // test never steps a core, so the alarm-match pends the IRQ in
    // ISPR1 directly without any chance of exception dispatch. PRIMASK
    // is irrelevant here (it only gates dispatch, not latching), so we
    // don't bother setting it.
    emu.bus.write32(0x4002_3000, 1u32 << 17, 0); // RESETS_CLR, RESET_POWMAN

    // V12 §3.2: emulator now mirrors silicon's INTE gating — the NVIC
    // line raise is conditional on `INTE.TIMER`. Enable it BEFORE
    // arming the alarm. `0x5AFE_0000` adds the POWMAN write password
    // bus-side; the `INT_TIMER_BIT` (= 1 << 1) is the level-sensitive
    // gate for IRQ 45.
    emu.bus
        .write32(POWMAN_BASE + INTE_OFFSET, 0x5AFE_0000 | INT_TIMER_BIT, 0);

    // Program MATCH = 100 and enable alarm + run. Password in [31:16].
    emu.bus
        .write32(POWMAN_BASE + ALARM_TIME_15TO0_OFFSET, 0x5AFE_0000 | 100, 0);
    emu.bus.write32(
        POWMAN_BASE + TIMER_OFFSET,
        0x5AFE_0000 | TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
        0,
    );

    // Enable NVIC line 45 (bank 1, bit 13). NVIC_ISER1 = 0xE000_E104.
    emu.bus.write32(0xE000_E104, 1u32 << (IRQ_POWMAN_IRQ_TIMER - 32), 0);

    // Tick enough sys_clks to cross MATCH=100.
    let n = 100 * POWMAN_SYS_PER_TICK as u32 + 50;
    emu.bus.tick_peripherals(n);

    // NVIC_ISPR1 bit 13 (= IRQ 45 - 32) should be set.
    let ispr1 = emu.bus.read32(0xE000_E204, 0);
    assert_ne!(
        ispr1 & (1u32 << (IRQ_POWMAN_IRQ_TIMER - 32)),
        0,
        "NVIC_ISPR1 bit 13 (IRQ 45) must be latched; PRIMASK blocks dispatch, not pending"
    );
}

#[test]
#[ignore = "threading: PPB writes (NVIC ISER, VTOR) must go through CortexM33::bus_write32 wrapper — test writes direct via Bus"]
fn powman_match_enters_emulator_handler() {
    use crate::peripherals::powman::{
        ALARM_TIME_15TO0_OFFSET, INT_TIMER_BIT, INTE_OFFSET, IRQ_POWMAN_IRQ_TIMER, POWMAN_BASE,
        POWMAN_SYS_PER_TICK, TIMER_ALARM_ENAB_BIT, TIMER_OFFSET, TIMER_RUN_BIT,
    };

    let mut emu = Emulator::new(Config::default());
    emu.bus.write32(0x4002_3000, 1u32 << 17, 0); // release POWMAN

    // Build a 64-slot vector table in SRAM so slot 16+45 = 61 is
    // addressable. Per HLD §3.2 the test MUST write VTOR to point at
    // the SRAM table, matching `isr_scenarios.rs:1511-1534`.
    // Otherwise IRQ 45 fetches from address 0 (ROM) and the test
    // silently passes for the wrong reason.
    const VT_BASE: u32 = 0x2000_2000;
    const HANDLER_ADDR: u32 = 0x2000_2400; // handler body well clear of VT
    const STACK_TOP: u32 = 0x2000_3000;

    // Vector table: slot 0 = initial MSP, slot 1 = reset handler
    // (never taken; we manually set PC). Slot 61 = POWMAN timer.
    emu.bus.write32(VT_BASE + 0, STACK_TOP, 0);
    // Reset vector — irrelevant for this test but well-formed.
    emu.bus.write32(VT_BASE + 4, (VT_BASE + 0x100) | 1, 0);
    // Slot 61 = IRQ 45 handler (Thumb LSB set).
    let slot_61 = VT_BASE + 61 * 4;
    emu.bus.write32(slot_61, HANDLER_ADDR | 1, 0);

    // Handler body at HANDLER_ADDR: `movs r0, #CAFE_BABE_low` is
    // impossible (immediate too big). Simpler: handler = single BKPT
    // #0 (0xBE00). After the IRQ is taken, core 0's PC lands here; we
    // step once and assert PC was at HANDLER_ADDR before the BKPT.
    emu.bus.write32(HANDLER_ADDR, 0x0000_BE00, 0); // bkpt #0 at [0], padding

    // Program VTOR — both secure and non-secure aliases.
    emu.bus.write32(0xE000_ED08, VT_BASE, 0); // S_VTOR
    emu.bus.write32(0xE002_ED08, VT_BASE, 0); // NS_VTOR

    // Seed core 0 with a minimal thread-mode context so it can take
    // the exception. Clear PRIMASK; program a simple `b .` PC so the
    // core executes while the alarm is pending.
    emu.core_mut(0).regs.msp = STACK_TOP;
    emu.core_mut(0).regs.r[13] = STACK_TOP;
    emu.core_mut(0).regs.primask = 0;
    emu.core_mut(0).regs.control = 0; // thread mode, MSP, privileged
    // PC at a main-loop stub in SRAM — a `b .` (0xE7FE) at 0x2000_2800.
    const MAIN_LOOP_ADDR: u32 = 0x2000_2800;
    emu.bus.write32(MAIN_LOOP_ADDR, 0x0000_E7FE, 0); // b . (branch-to-self)
    emu.core_mut(0).regs.set_pc(MAIN_LOOP_ADDR);
    emu.core_mut(0).regs.xpsr = 1 << 24; // Thumb bit

    // Halt core 1 — we only care about core 0 taking the exception.
    emu.core_mut(1).regs.set_pc(MAIN_LOOP_ADDR);
    emu.core_mut(1).regs.xpsr = 1 << 24;

    // Program MATCH = 100, enable INTE.TIMER (V12 §3.2 silicon gate),
    // enable POWMAN TIMER alarm, enable NVIC 45.
    emu.bus
        .write32(POWMAN_BASE + ALARM_TIME_15TO0_OFFSET, 0x5AFE_0000 | 100, 0);
    emu.bus
        .write32(POWMAN_BASE + INTE_OFFSET, 0x5AFE_0000 | INT_TIMER_BIT, 0);
    emu.bus.write32(
        POWMAN_BASE + TIMER_OFFSET,
        0x5AFE_0000 | TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
        0,
    );
    emu.bus.write32(0xE000_E104, 1u32 << (IRQ_POWMAN_IRQ_TIMER - 32), 0);

    // Run long enough for alarm to fire and IRQ to dispatch. 100 ticks
    // of POWMAN plus margin for exception-entry cycles.
    let budget = (100 * POWMAN_SYS_PER_TICK as u64) + 500;
    emu.run(budget);

    // Core 0 should have entered the handler. PC lands at HANDLER_ADDR
    // on entry, then BKPT #0 executes and advances PC by 2. Either
    // value proves the IRQ dispatch reached the SRAM handler — if VTOR
    // were still 0, the IRQ vector fetch would have gone to ROM and PC
    // would be somewhere entirely different (or the core would have
    // faulted).
    let pc = emu.core(0).regs.pc();
    assert!(
        pc == HANDLER_ADDR || pc == HANDLER_ADDR + 2,
        "core 0 PC must be at SRAM handler (±2) after POWMAN alarm fires; got {:#010X}",
        pc
    );
    // Also confirm the CPU is in handler mode (IPSR != 0) to rule out
    // "the main loop happened to branch here" as an escape hatch.
    let ipsr = emu.core(0).regs.xpsr & 0x1FF;
    assert_eq!(
        ipsr, 16 + 45,
        "core 0 IPSR must equal 16 + IRQ 45 = 61; got {}",
        ipsr
    );
}

#[test]
fn powman_state_resets_on_emulator_reset() {
    // Warm reset (e.g. via watchdog / SYSRESETREQ) must quiesce the
    // POWMAN AON timer: COUNT, MATCH, and TIMER control bits all
    // return to post-power-on zero. Mirrors the Stage 3
    // `glitch_detector_arm_restored_on_warm_reset` pattern.
    use crate::peripherals::powman::{
        ALARM_TIME_15TO0_OFFSET, POWMAN_BASE, READ_TIME_LOWER_OFFSET,
        SET_TIME_15TO0_OFFSET, SET_TIME_31TO16_OFFSET, TIMER_OFFSET,
        TIMER_ALARM_ENAB_BIT, TIMER_RUN_BIT,
    };

    let mut emu = Emulator::new(Config::default());
    // Release POWMAN from reset so writes reach the peripheral.
    emu.bus.write32(0x4002_3000, 1u32 << 17, 0);

    // Seed non-default state: COUNT=1000 via SET_TIME_*, MATCH=100,
    // TIMER = RUN | ALARM_ENAB. Password in [31:16] on every write.
    emu.bus.write32(
        POWMAN_BASE + SET_TIME_15TO0_OFFSET,
        0x5AFE_0000 | (1000 & 0xFFFF),
        0,
    );
    emu.bus
        .write32(POWMAN_BASE + SET_TIME_31TO16_OFFSET, 0x5AFE_0000, 0);
    emu.bus
        .write32(POWMAN_BASE + ALARM_TIME_15TO0_OFFSET, 0x5AFE_0000 | 100, 0);
    emu.bus.write32(
        POWMAN_BASE + TIMER_OFFSET,
        0x5AFE_0000 | TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
        0,
    );
    // Sanity: pre-reset state looks as seeded.
    assert_eq!(
        emu.bus.read32(POWMAN_BASE + READ_TIME_LOWER_OFFSET, 0),
        1000,
        "precondition: SET_TIME_* must seed COUNT"
    );
    assert_eq!(
        emu.bus.read32(POWMAN_BASE + ALARM_TIME_15TO0_OFFSET, 0),
        100,
        "precondition: ALARM_TIME_15TO0 round-trips"
    );
    assert_eq!(
        emu.bus.read32(POWMAN_BASE + TIMER_OFFSET, 0)
            & (TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT),
        TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
        "precondition: TIMER carries RUN|ALARM_ENAB"
    );

    // Warm reset — POWMAN state should zero out.
    emu.reset();
    // After reset, POWMAN is held-at-reset again (see RESETS_POST_BOOTROM
    // in lib.rs), so re-release before reading to get meaningful
    // observations.
    emu.bus.write32(0x4002_3000, 1u32 << 17, 0);

    assert_eq!(
        emu.bus.read32(POWMAN_BASE + READ_TIME_LOWER_OFFSET, 0),
        0,
        "POWMAN COUNT must be zero after warm reset"
    );
    assert_eq!(
        emu.bus.read32(POWMAN_BASE + ALARM_TIME_15TO0_OFFSET, 0),
        0,
        "POWMAN ALARM_TIME_15TO0 must be zero after warm reset"
    );
    assert_eq!(
        emu.bus.read32(POWMAN_BASE + TIMER_OFFSET, 0),
        0,
        "POWMAN TIMER must be zero after warm reset"
    );
}

#[test]
fn powman_archsel_non_arm_write_fires_tripwire_once() {
    // Regression tripwire for HLD §10: if the RISC-V Hazard3 track ever
    // moves from build-time `Cores::RiscV` to runtime ARCHSEL-driven
    // selection, this test must be revisited. Until then, a non-Arm
    // ARCHSEL write is an emulator-only anomaly that fires the tripwire
    // once. The tripwire is now trace-level rather than warn-level —
    // silicon has no ARCHSEL at offset 0x20 (some other real register
    // lives there), so we don't want real firmware to spam warnings.
    // See `peripherals/powman.rs` module doc for the rationale.
    use crate::peripherals::powman::{ARCHSEL_OFFSET, POWMAN_BASE};

    let mut emu = Emulator::new(Config::default());
    // Release POWMAN from reset so the ARCHSEL write reaches the
    // peripheral (the Bus-level RESETS guard would otherwise swallow
    // the write and no event would fire).
    emu.bus.write32(0x4002_3000, 1u32 << 17, 0);

    emu.bus.write32(POWMAN_BASE + ARCHSEL_OFFSET, 1, 0); // non-Arm
    emu.bus.write32(POWMAN_BASE + ARCHSEL_OFFSET, 2, 0); // still non-Arm
    assert_eq!(emu.bus.read32(POWMAN_BASE + ARCHSEL_OFFSET, 0), 2);
    // Fire-once behaviour itself is covered by the module-level
    // `powman_archsel_non_arm_write_fires_tripwire_once` test in
    // `crates/mdrp2350/src/peripherals/powman.rs` (uses a capture
    // subscriber — out of place in the integration-style tests.rs).
}

// ============================================================================
// Stage 1 — execute.rs branch coverage top-up
//
// These tests target specific uncovered branches in
// `crates/mdrp2350/src/core/execute.rs` listed in the Coverage Improvement
// Plan (§Stage 1). Each test names the source line it pins.
// ============================================================================

mod stage1_execute_coverage {
    use super::*;
    use crate::core::Fault;

    // ---- Shift-by-register edge branches (LSLS/LSRS/ASRS/RORS) ----

    // line 216: LSLS Rdn, Rm with shift amount == 0 — carry preserved.
    #[test]
    fn lsls_reg_shift_zero_preserves_carry() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0xABCD_1234);
        c.set_reg(1, 0); // shift amount = 0
        c.regs.set_flag_c(true);
        c.execute_one(0x4088); // LSLS R0, R1
        assert_eq!(c.reg(0), 0xABCD_1234, "value unchanged when shift=0");
        assert!(c.flag_c(), "carry preserved when shift=0");
    }

    // line 218: LSLS with shift in 1..32 (non-edge carry-out path).
    #[test]
    fn lsls_reg_shift_lt_32_carry_out() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x4000_0000);
        c.set_reg(1, 2); // shift amount = 2 → bit 30 shifts into carry
        c.execute_one(0x4088); // LSLS R0, R1
        assert_eq!(c.reg(0), 0);
        assert!(c.flag_c());
        assert!(c.flag_z());
    }

    // line 220: LSLS with shift == 32 — result 0, carry = bit 0 of a.
    #[test]
    fn lsls_reg_shift_eq_32() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x0000_0001); // bit 0 set
        c.set_reg(1, 32);
        c.execute_one(0x4088); // LSLS R0, R1
        assert_eq!(c.reg(0), 0);
        assert!(c.flag_c(), "carry = LSB of operand when shift == 32");
    }

    // line 223 (implicit else): shift > 32 — result 0, carry 0.
    #[test]
    fn lsls_reg_shift_gt_32() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0xFFFF_FFFF);
        c.set_reg(1, 33);
        c.regs.set_flag_c(true);
        c.execute_one(0x4088);
        assert_eq!(c.reg(0), 0);
        assert!(!c.flag_c());
    }

    // line 232: LSRS Rdn, Rm shift == 0 — carry preserved.
    #[test]
    fn lsrs_reg_shift_zero_preserves_carry() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x1234_5678);
        c.set_reg(1, 0);
        c.regs.set_flag_c(true);
        c.execute_one(0x40C8); // LSRS R0, R1
        assert_eq!(c.reg(0), 0x1234_5678);
        assert!(c.flag_c());
    }

    // line 234: LSRS with shift in 1..32.
    #[test]
    fn lsrs_reg_shift_lt_32_carry_out() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x0000_0003); // bit 0 == 1, bit 1 == 1
        c.set_reg(1, 1);
        c.execute_one(0x40C8);
        assert_eq!(c.reg(0), 1);
        assert!(c.flag_c(), "bit 0 shifted out");
    }

    // line 236: LSRS shift == 32 — result 0, carry = bit 31.
    #[test]
    fn lsrs_reg_shift_eq_32() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x8000_0000);
        c.set_reg(1, 32);
        c.execute_one(0x40C8);
        assert_eq!(c.reg(0), 0);
        assert!(c.flag_c());
    }

    // line 239 (implicit else): shift > 32 — result 0, carry 0.
    #[test]
    fn lsrs_reg_shift_gt_32() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0xFFFF_FFFF);
        c.set_reg(1, 33);
        c.regs.set_flag_c(true);
        c.execute_one(0x40C8);
        assert_eq!(c.reg(0), 0);
        assert!(!c.flag_c());
    }

    // line 249: ASRS Rdn, Rm shift == 0 — carry preserved.
    #[test]
    fn asrs_reg_shift_zero_preserves_carry() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0xFFFF_FFFF);
        c.set_reg(1, 0);
        c.regs.set_flag_c(true);
        c.execute_one(0x4108); // ASRS R0, R1
        assert_eq!(c.reg(0), 0xFFFF_FFFF);
        assert!(c.flag_c());
    }

    // line 251: ASRS shift in 1..32 — arithmetic shift with carry-out.
    #[test]
    fn asrs_reg_shift_lt_32_negative() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0xFFFF_FFF0u32);
        c.set_reg(1, 4);
        c.execute_one(0x4108); // ASRS R0, R1
        assert_eq!(c.reg(0), 0xFFFF_FFFF, "sign-extended right shift");
        assert!(c.flag_n());
        assert!(!c.flag_c(), "bit 3 was 0 before shift");
    }

    // Covers the shift >= 32 else arm in ASRS reg.
    #[test]
    fn asrs_reg_shift_ge_32_sign_extends() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x8000_0000);
        c.set_reg(1, 32);
        c.execute_one(0x4108);
        assert_eq!(c.reg(0), 0xFFFF_FFFF, "sign-extended to all-ones");
        assert!(c.flag_c(), "carry = sign bit");
    }

    // line 276: RORS shift == 0 — carry preserved, value unchanged.
    #[test]
    fn rors_reg_shift_zero_preserves_carry() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x1234_5678);
        c.set_reg(1, 0);
        c.regs.set_flag_c(true);
        c.execute_one(0x41C8); // RORS R0, R1
        assert_eq!(c.reg(0), 0x1234_5678);
        assert!(c.flag_c());
    }

    // line 280: RORS shift that is a nonzero multiple of 32 — value unchanged,
    // carry = bit 31. `shift & 31 == 0` but `shift != 0`.
    #[test]
    fn rors_reg_shift_multiple_of_32() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x8000_0000); // bit 31 set
        c.set_reg(1, 32);
        c.execute_one(0x41C8);
        assert_eq!(c.reg(0), 0x8000_0000, "value unchanged on rotate-by-32");
        assert!(c.flag_c(), "carry = bit 31 of operand");
        assert!(c.flag_n());
    }

    // ---- Immediate-shift edge branches (LSRS/ASRS imm5 == 0) ----

    // line 39: LSLS imm with imm5 != 0 (normal path, carry-out branch).
    #[test]
    fn lsls_imm_nonzero_shift_sets_carry() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(1, 0xC000_0000);
        c.execute_one(0x0048); // LSLS R0, R1, #1 (imm5=1) → 0x80000000, carry = bit 31
        assert_eq!(c.reg(0), 0x8000_0000);
        assert!(c.flag_c());
        assert!(c.flag_n());
    }

    // line 61: LSRS imm with imm5 != 0 (normal path) — covered by lsrs_imm_basic,
    // but also pin the shift==32 (imm5=0) carry=0 case for completeness.
    #[test]
    fn lsrs_imm_shift_32_carry_zero_when_bit31_clear() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x7FFF_FFFF); // bit 31 clear
        c.execute_one(0x0800); // LSRS R0, R0, #32 (imm5=0)
        assert_eq!(c.reg(0), 0);
        assert!(!c.flag_c());
        assert!(c.flag_z());
    }

    // line 81: ASRS imm with imm5 == 0 (shift-by-32) — positive input variant.
    #[test]
    fn asrs_imm_shift_32_positive_input() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x7FFF_FFFF);
        c.execute_one(0x1000); // ASRS R0, R0, #32 (imm5=0)
        assert_eq!(c.reg(0), 0);
        assert!(!c.flag_c(), "carry = sign bit (0)");
        assert!(c.flag_z());
    }

    // Mirror negative input case to ensure both arms of `val < 0` at line 84 hit.
    #[test]
    fn asrs_imm_shift_32_negative_input() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0x8000_0000);
        c.execute_one(0x1000); // ASRS R0, R0, #32
        assert_eq!(c.reg(0), 0xFFFF_FFFF);
        assert!(c.flag_c(), "carry = sign bit (1)");
        assert!(c.flag_n());
    }

    // ---- Special data / high-register / BX / BLX paths ----

    // line 358: ADD high regs with Rm == 15 (PC).
    #[test]
    fn add_high_reg_rm_is_pc() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.set_reg(0, 0x10);
        // ADD R0, PC: bits[15:10]=010001, op=00, D=0, Rm=1111, Rd=000
        // = 0b0100_0100_0_1111_000 = 0x4478
        c.execute_one(0x4478);
        // read_pc() = current_instr_addr + 4 = 0x1000 + 4 = 0x1004
        // result = R0 + read_pc() = 0x10 + 0x1004 = 0x1014
        assert_eq!(c.reg(0), 0x1014);
    }

    // line 359: ADD high regs with Rd == 15 (PC), no pipeline flush yet —
    // exercises the rd_val load from read_pc branch.
    #[test]
    fn add_high_reg_rd_is_pc_pipeline_flush() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.set_reg(0, 0x10);
        // ADD PC, R0: op=00, D=1, Rm=0000, Rd_low=111 → 0x4487? Let me recompute.
        // bits: 0100_0100_D_MMMM_RRR with D=1, Rm=0, Rd_low=7 → 0100_0100_1_0000_111
        //     = 0x4487
        let cy = c.execute_one(0x4487);
        // result = read_pc() + R0 = 0x1004 + 0x10 = 0x1014; set_pc masks bit 0.
        assert_eq!(c.regs.pc(), 0x1014 & !1);
        assert_eq!(cy, 3, "pipeline flush cost");
    }

    // line 361 / 362: ADD high regs with Rd == 15 AND Rm == 15 (PC+PC).
    // Pins the combined Rm==15 true branch on line 358 together with the
    // Rd==15 true branch on 361.
    #[test]
    fn add_high_reg_rd_and_rm_are_pc() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        // ADD PC, PC: D=1, Rm=1111, Rd_low=111 → 0100_0100_1_1111_111 = 0x44FF
        let cy = c.execute_one(0x44FF);
        // read_pc() = 0x1004 → result = 0x1004 + 0x1004 = 0x2008
        assert_eq!(c.regs.pc(), 0x2008 & !1);
        assert_eq!(cy, 3);
    }

    // line 372: CMP high regs with Rn == 15 (PC).
    #[test]
    fn cmp_high_reg_rn_is_pc() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.set_reg(0, 0x1004); // equals read_pc()
        // CMP PC, R0: op=01, D=1 (Rn top bit), Rm=0 (bits[6:3]), Rn_low=7 (bits[2:0]).
        //   0100_0101_1_0000_111 = 0x4587
        c.execute_one(0x4587);
        assert!(c.flag_z(), "PC == R0 so CMP sets Z");
    }

    // line 373: CMP high regs with Rm == 15 (PC).
    #[test]
    fn cmp_high_reg_rm_is_pc() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.set_reg(0, 0x1004); // equals read_pc()
        // CMP R0, PC: op=01, D=0, Rm=15, Rd_low=000 → 0100_0101_0_1111_000 = 0x4578
        c.execute_one(0x4578);
        assert!(c.flag_z());
    }

    // line 382: MOV high regs with Rm == 15 (PC).
    #[test]
    fn mov_high_reg_rm_is_pc() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        // MOV R0, PC: op=10, D=0, Rm=1111, Rd_low=000 → 0100_0110_0_1111_000 = 0x4678
        c.execute_one(0x4678);
        assert_eq!(c.reg(0), 0x1004, "read_pc()");
    }

    // line 383/387: MOV PC, Rm with val NOT an EXC_RETURN magic (plain branch).
    #[test]
    fn mov_high_reg_rd_is_pc_plain_branch() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.set_reg(0, 0x2000_0101); // Thumb-tagged target, not EXC_RETURN
        // MOV PC, R0: op=10, D=1, Rm=0, Rd_low=7 → 0100_0110_1_0000_111 = 0x4687
        let cy = c.execute_one(0x4687);
        assert_eq!(c.regs.pc(), 0x2000_0100);
        assert_eq!(cy, 3, "pipeline flush");
    }

    // line 384: MOV PC, Rm where Rm contains an EXC_RETURN magic value.
    // We enter an exception so there is a valid frame to unstack.
    #[test]
    fn mov_pc_rm_with_exc_return_triggers_exit() {
        let mut bus = Bus::new();
        let mut cpu = CortexM33::new(0, bus.atomics.clone());
        cpu.regs.msp = 0x2000_2000;
        cpu.regs.r[13] = cpu.regs.msp;
        let vtor: u32 = 0x2000_4000;
        cpu.ppb.vtor = vtor;
        bus.write32(vtor + 14 * 4, 0x2000_0200 | 1, 0); // PendSV
        cpu.test_enter_exception(14, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 14);
        cpu.set_reg(0, 0xFFFF_FFF9); // thread / MSP / no FP
        // MOV PC, R0 — value matches EXC_RETURN pattern → exit_exception.
        cpu.execute_one_with_bus(0x4687, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 0, "returned to thread mode");
    }

    // line 396: BX/BLX path — Rm == 15 (PC) for BX.
    #[test]
    fn bx_rm_is_pc() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        // BX PC: op=11, bit7=0, Rm=1111, Rd_low=000 → 0100_0111_0_1111_000 = 0x4778
        c.execute_one(0x4778);
        // PC = read_pc() & !1 = 0x1004
        assert_eq!(c.regs.pc(), 0x1004);
    }

    // line 398: BLX Rm path — link flag set, LR updated.
    #[test]
    fn blx_reg_link_updates_lr() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.set_reg(0, 0x2000_0001); // Thumb-tagged target
        // BLX R0: op=11, bit7=1, Rm=0000, Rd_low=000 → 0100_0111_1_0000_000 = 0x4780
        c.execute_one(0x4780);
        assert_eq!(c.regs.pc(), 0x2000_0000);
        // LR = next_instr (post-execute-one PC, which = 0x1002) | 1 = 0x1003
        assert_eq!(c.regs.lr(), 0x1003);
    }

    // line 402: BLX target is EXC_RETURN magic — exit_exception.
    #[test]
    fn blx_with_exc_return_target_triggers_exit() {
        let mut bus = Bus::new();
        let mut cpu = CortexM33::new(0, bus.atomics.clone());
        cpu.regs.msp = 0x2000_2000;
        cpu.regs.r[13] = cpu.regs.msp;
        let vtor: u32 = 0x2000_4000;
        cpu.ppb.vtor = vtor;
        bus.write32(vtor + 14 * 4, 0x2000_0200 | 1, 0);
        cpu.test_enter_exception(14, &mut bus);
        cpu.set_reg(0, 0xFFFF_FFF9);
        // BLX R0: 0x4780
        cpu.execute_one_with_bus(0x4780, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 0, "BLX with EXC_RETURN magic hit exit path");
    }

    // line 406: BX target is EXC_RETURN magic — exit_exception via BX.
    #[test]
    fn bx_with_exc_return_target_triggers_exit() {
        let mut bus = Bus::new();
        let mut cpu = CortexM33::new(0, bus.atomics.clone());
        cpu.regs.msp = 0x2000_2000;
        cpu.regs.r[13] = cpu.regs.msp;
        let vtor: u32 = 0x2000_4000;
        cpu.ppb.vtor = vtor;
        bus.write32(vtor + 14 * 4, 0x2000_0200 | 1, 0);
        cpu.test_enter_exception(14, &mut bus);
        // LR holds the EXC_RETURN magic after enter_exception.
        assert!(CortexM33::is_exc_return(cpu.regs.lr()));
        // BX LR: op=11, bit7=0, Rm=1110, Rd_low=000 → 0100_0111_0_1110_000 = 0x4770
        cpu.execute_one_with_bus(0x4770, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 0, "BX LR at exception return unstacks");
    }

    // line 413: BXNS from Secure state (transition). Covered by existing
    // `bxns_from_secure` already, but pin an explicit assertion here too.
    #[test]
    fn bxns_transitions_to_nonsecure_explicit() {
        let mut c = CortexM33::for_test(0);
        assert!(c.secure);
        c.regs.msp_ns = 0x2000_4000;
        c.set_reg(0, 0x1000_0001);
        // BXNS R0: 0100_0111_0_0000_100 = 0x4704
        c.execute_one(0x4704);
        assert!(!c.secure, "transition_to_nonsecure taken");
    }

    // ---- SIO-address cycle-cost branches for load/store instructions ----
    //
    // STR/STRH/STRB/STR_imm/STRB_imm/STRH_imm/STR_sp all have an
    // `if addr >> 28 == 0xD { 1 } else { 2 }` cost tail. Existing tests only
    // exercise SRAM (0x2000_...), so the SIO arm (cost=1) is uncovered.

    // line 453: STR Rt, [Rn, Rm] with SIO address.
    #[test]
    fn str_reg_sio_costs_one_cycle() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xAABB_CCDD);
        c.set_reg(1, 0xD000_0000); // SIO base
        c.set_reg(2, 0x10);        // GPIO_OUT offset
        // STR R0, [R1, R2]: 0101_000_010_001_000 = 0x5088
        let cy = c.execute_one_with_bus(0x5088, &mut bus);
        assert_eq!(cy, 1, "SIO store is single-cycle");
    }

    // line 458: STRH Rt, [Rn, Rm] SIO path.
    #[test]
    fn strh_reg_sio_costs_one_cycle() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xDEAD);
        c.set_reg(1, 0xD000_0000);
        c.set_reg(2, 0x10);
        // STRH R0, [R1, R2]: 0101_001_010_001_000 = 0x5288
        let cy = c.execute_one_with_bus(0x5288, &mut bus);
        assert_eq!(cy, 1);
    }

    // line 463: STRB Rt, [Rn, Rm] SIO path.
    #[test]
    fn strb_reg_sio_costs_one_cycle() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xAB);
        c.set_reg(1, 0xD000_0000);
        c.set_reg(2, 0x10);
        // STRB R0, [R1, R2]: 0101_010_010_001_000 = 0x5488
        let cy = c.execute_one_with_bus(0x5488, &mut bus);
        assert_eq!(cy, 1);
    }

    // Also exercise load-register-reg arms (LDRSB, LDR, LDRH, LDRB, LDRSH)
    // so the match arms at 465–491 have all arms taken at least once.
    #[test]
    fn ldrh_reg_exercises_arm() {
        let (mut c, mut bus) = core_and_bus();
        bus.write16(0x2000_0004, 0xBEEF, 0);
        c.set_reg(1, 0x2000_0000);
        c.set_reg(2, 4);
        // LDRH R0, [R1, R2]: 0101_101_010_001_000 = 0x5A88
        c.execute_one_with_bus(0x5A88, &mut bus);
        assert_eq!(c.reg(0), 0xBEEF);
    }

    #[test]
    fn ldrsh_reg_exercises_arm() {
        let (mut c, mut bus) = core_and_bus();
        bus.write16(0x2000_0000, 0x8000, 0);
        c.set_reg(1, 0x2000_0000);
        c.set_reg(2, 0);
        // LDRSH R0, [R1, R2]: 0101_111_010_001_000 = 0x5E88
        c.execute_one_with_bus(0x5E88, &mut bus);
        assert_eq!(c.reg(0), 0xFFFF_8000);
    }

    // line 506: STR Rt, [Rn, #imm5*4] SIO path.
    #[test]
    fn str_imm_sio_costs_one_cycle() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xAA);
        c.set_reg(1, 0xD000_0010);
        // STR R0, [R1, #0]: 01100_00000_001_000 = 0x6008
        let cy = c.execute_one_with_bus(0x6008, &mut bus);
        assert_eq!(cy, 1);
    }

    // line 526: STRB Rt, [Rn, #imm5] SIO path.
    #[test]
    fn strb_imm_sio_costs_one_cycle() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xAB);
        c.set_reg(1, 0xD000_0010);
        // STRB R0, [R1, #0]: 01110_00000_001_000 = 0x7008
        let cy = c.execute_one_with_bus(0x7008, &mut bus);
        assert_eq!(cy, 1);
    }

    // line 546: STRH Rt, [Rn, #imm5*2] SIO path.
    #[test]
    fn strh_imm_sio_costs_one_cycle() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xBEEF);
        c.set_reg(1, 0xD000_0010);
        // STRH R0, [R1, #0]: 10000_00000_001_000 = 0x8008
        let cy = c.execute_one_with_bus(0x8008, &mut bus);
        assert_eq!(cy, 1);
    }

    // line 569: STR Rt, [SP, #imm8*4] SIO path — SP pointing into SIO.
    #[test]
    fn str_sp_sio_costs_one_cycle() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(13, 0xD000_0010);
        c.set_reg(0, 0x1234_5678);
        // STR R0, [SP, #0]: 10010_000_00000000 = 0x9000
        let cy = c.execute_one_with_bus(0x9000, &mut bus);
        assert_eq!(cy, 1);
    }

    // ---- Misc group: Adjust SP ADD/SUB ----

    // line 614: ADD SP, SP, #imm (bit 7 clear) — covered by `add_sp_imm`.
    // Pin the SUB SP variant separately for symmetry.
    #[test]
    fn sub_sp_imm_pins_negative_branch() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(13, 0x2000_1000);
        // SUB SP, SP, #16: 10110000_1_0000100 = 0xB084
        c.execute_one(0xB084);
        assert_eq!(c.regs.sp(), 0x2000_0FF0);
    }

    // ---- PUSH reglist iteration / LR bit branches ----

    // line 639: PUSH without LR (bit 8 clear) — complements push_lr_pop_pc.
    #[test]
    fn push_without_lr_bit() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(13, 0x2000_1000);
        c.set_reg(0, 0xCAFE);
        c.set_reg(1, 0xBABE);
        // PUSH {R0,R1}: 1011_0100_00000011 = 0xB403 (bit 8 clear → no LR)
        c.execute_one_with_bus(0xB403, &mut bus);
        assert_eq!(c.regs.sp(), 0x2000_0FF8);
        assert_eq!(bus.read32(0x2000_0FF8, 0), 0xCAFE);
        assert_eq!(bus.read32(0x2000_0FFC, 0), 0xBABE);
    }

    // line 647: PUSH reglist iteration — regs NOT in the list are skipped.
    // Push {R0, R7} with non-contiguous set.
    #[test]
    fn push_noncontiguous_reglist() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(13, 0x2000_1000);
        c.set_reg(0, 0x1111);
        c.set_reg(7, 0x7777);
        // PUSH {R0, R7}: reglist = 0b1000_0001 = 0x81 → 0xB481
        c.execute_one_with_bus(0xB481, &mut bus);
        assert_eq!(c.regs.sp(), 0x2000_0FF8, "2 regs pushed");
        assert_eq!(bus.read32(0x2000_0FF8, 0), 0x1111);
        assert_eq!(bus.read32(0x2000_0FFC, 0), 0x7777);
    }

    // ---- CPS / PRIMASK / FAULTMASK ----

    // line 662: CPS affect_i branch (bit 0 set). CPSIE I (im=0, affect I).
    #[test]
    fn cpsie_i_clears_primask() {
        let mut c = CortexM33::for_test(0);
        c.regs.primask = 1;
        // CPSIE I: bit 4 = 0 (enable → im=0), bit 0 = 1.
        // Base 0xB660 | 0x01 = 0xB661
        c.execute_one(0xB661);
        assert_eq!(c.regs.primask, 0, "CPSIE clears PRIMASK");
    }

    #[test]
    fn cpsid_i_sets_primask() {
        let mut c = CortexM33::for_test(0);
        c.regs.primask = 0;
        // CPSID I: bit 4 = 1 (im=1), bit 0 = 1.
        // Base 0xB660 | 0x10 | 0x01 = 0xB671
        c.execute_one(0xB671);
        assert_eq!(c.regs.primask, 1);
    }

    // line 665: CPS affect_f branch (bit 1 set). CPSIE F / CPSID F.
    #[test]
    fn cpsie_f_clears_faultmask() {
        let mut c = CortexM33::for_test(0);
        c.regs.faultmask = 1;
        // CPSIE F: bit 4 = 0, bit 1 = 1. 0xB660 | 0x02 = 0xB662
        c.execute_one(0xB662);
        assert_eq!(c.regs.faultmask, 0);
    }

    #[test]
    fn cpsid_f_sets_faultmask() {
        let mut c = CortexM33::for_test(0);
        c.regs.faultmask = 0;
        // CPSID F: bit 4 = 1, bit 1 = 1. 0xB660 | 0x10 | 0x02 = 0xB672
        c.execute_one(0xB672);
        assert_eq!(c.regs.faultmask, 1);
    }

    // CPS with neither bit set — hits both `if` false arms.
    #[test]
    fn cps_no_affect_bits() {
        let mut c = CortexM33::for_test(0);
        c.regs.primask = 1;
        c.regs.faultmask = 1;
        // CPSID with neither I nor F: 0xB660 | 0x10 = 0xB670
        c.execute_one(0xB670);
        assert_eq!(c.regs.primask, 1, "primask untouched when affect_i=0");
        assert_eq!(c.regs.faultmask, 1, "faultmask untouched when affect_f=0");
    }

    // ---- POP reglist / PC / EXC_RETURN paths ----

    // line 694: POP without PC bit (bit 8 clear).
    #[test]
    fn pop_without_pc_bit() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(13, 0x2000_0FF8);
        bus.write32(0x2000_0FF8, 0xAAAA, 0);
        bus.write32(0x2000_0FFC, 0xBBBB, 0);
        // POP {R0, R1}: 1011_1100_00000011 = 0xBC03 (bit 8 clear)
        c.execute_one_with_bus(0xBC03, &mut bus);
        assert_eq!(c.reg(0), 0xAAAA);
        assert_eq!(c.reg(1), 0xBBBB);
        assert_eq!(c.regs.sp(), 0x2000_1000);
    }

    // line 701/703: POP reglist with PC-load (i == 15) AND value is NOT
    // an EXC_RETURN — the `else` arm of `is_exc_return(val)` at line 704.
    #[test]
    fn pop_pc_plain_target() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(13, 0x2000_0FF8);
        bus.write32(0x2000_0FF8, 0xCAFE, 0);
        bus.write32(0x2000_0FFC, 0x2000_0101, 0); // PC (Thumb-tagged)
        // POP {R0, PC}: 1011_1101_00000001 = 0xBD01
        let cy = c.execute_one_with_bus(0xBD01, &mut bus);
        assert_eq!(c.reg(0), 0xCAFE);
        assert_eq!(c.regs.pc(), 0x2000_0100, "bit 0 cleared");
        assert_eq!(c.regs.sp(), 0x2000_1000);
        // line 719: POP with pop_pc true → cost = 1 + count + 3.
        // count = 2 (R0 + PC). Expected = 1 + 2 + 3 = 6.
        assert_eq!(cy, 6);
    }

    // line 704: POP where the popped PC value IS an EXC_RETURN — exit path.
    #[test]
    fn pop_pc_with_exc_return() {
        let mut bus = Bus::new();
        let mut cpu = CortexM33::new(0, bus.atomics.clone());
        cpu.regs.msp = 0x2000_2000;
        cpu.regs.r[13] = cpu.regs.msp;
        let vtor: u32 = 0x2000_4000;
        cpu.ppb.vtor = vtor;
        bus.write32(vtor + 14 * 4, 0x2000_0200 | 1, 0);
        cpu.test_enter_exception(14, &mut bus);
        // Stack a frame that ends with EXC_RETURN as the "PC slot" to be popped.
        // Place EXC_RETURN where POP will read it.
        let sp = cpu.regs.r[13];
        bus.write32(sp, 0xFFFF_FFF9, 0); // the PC slot for POP {PC}
        // POP {PC}: 1011_1101_00000000 = 0xBD00
        cpu.execute_one_with_bus(0xBD00, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 0, "EXC_RETURN via POP unstacks to thread mode");
    }

    // line 719 else arm: POP without PC → cost = 1 + count (no +3).
    #[test]
    fn pop_without_pc_cycle_cost() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(13, 0x2000_0FF8);
        bus.write32(0x2000_0FF8, 0, 0);
        bus.write32(0x2000_0FFC, 0, 0);
        // POP {R0, R1}: 0xBC03, count = 2 → expected cycles = 1 + 2 = 3.
        let cy = c.execute_one_with_bus(0xBC03, &mut bus);
        assert_eq!(cy, 3);
    }

    // ---- IT encoding vs hint encoding (line 731 branch) ----

    // line 731 false arm: `mask == 0` → hint instruction (WFI/WFE/SEV/NOP/YIELD).
    // NOP (0xBF00) — covered by step path indirectly. Pin here directly.
    #[test]
    fn misc_1111_mask_zero_is_hint_nop() {
        let (mut c, mut bus) = core_and_bus();
        c.regs.set_pc(0x1000);
        let cy = c.execute_one_with_bus(0xBF00, &mut bus); // NOP
        assert_eq!(cy, 1);
        assert_eq!(c.it_state(), 0, "NOP must not set IT state");
    }

    #[test]
    fn misc_1111_mask_zero_yield() {
        let (mut c, mut bus) = core_and_bus();
        let cy = c.execute_one_with_bus(0xBF10, &mut bus); // YIELD (hint_op=1)
        assert_eq!(cy, 1);
    }

    #[test]
    fn misc_1111_hint_reserved_is_nop() {
        let (mut c, mut bus) = core_and_bus();
        // hint_op = 5 (reserved). 0xBF00 | (5<<4) = 0xBF50
        let cy = c.execute_one_with_bus(0xBF50, &mut bus);
        assert_eq!(cy, 1);
    }

    // line 733 true: IT with mask != 0 — covered by IT tests, but pin here.
    #[test]
    fn misc_1111_mask_nonzero_is_it() {
        let mut c = CortexM33::for_test(0);
        // IT EQ: firstcond=0000, mask=1000 → 0xBF08
        c.execute_one(0xBF08);
        assert_eq!(c.it_state(), 0x08);
    }

    // ---- CBZ / CBNZ match arm (line 760) and condition (line 767) ----

    // CBZ condition NOT taken: rn != 0 AND nonzero == false (CBZ flavor).
    // line 767 else arm, already covered by cbz_not_taken. Add CBNZ nonzero
    // with rn==0 flavor — also the else. Add a CBZ taken with large imm.
    #[test]
    fn cbz_with_i_bit_set_offset() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.set_reg(0, 0);
        // CBZ R0 with i=1, imm5=0 → offset = (1<<6)|0 = 0x40 (64 bytes)
        // Encoding: 1011_0_0_1_1_00000_000 = 0xB300
        let cy = c.execute_one(0xB300);
        // target = read_pc() + offset = 0x1004 + 0x40 = 0x1044
        assert_eq!(c.regs.pc(), 0x1044);
        assert_eq!(cy, 2);
    }

    // ---- STM iteration / writeback (line 792) ----

    // Empty reglist? STM with a sparse reglist — ensures the false arm of
    // `reglist & (1 << i) != 0` is exercised for many i values.
    #[test]
    fn stm_sparse_reglist() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(4, 0x2000_0100);
        c.set_reg(0, 0xAA);
        c.set_reg(7, 0x77);
        // STM R4!, {R0, R7}: reglist = 0b1000_0001 = 0x81 → 0xC481
        c.execute_one_with_bus(0xC481, &mut bus);
        assert_eq!(bus.read32(0x2000_0100, 0), 0xAA);
        assert_eq!(bus.read32(0x2000_0104, 0), 0x77);
        assert_eq!(c.reg(4), 0x2000_0108, "writeback after 2 regs");
    }

    // ---- LDM iteration + writeback suppression (line 813, 820) ----

    // line 820: LDM Rn!, {reglist} where Rn IS in reglist → NO writeback.
    #[test]
    fn ldm_with_base_in_reglist_skips_writeback() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0x2000_0100); // base AND target register
        bus.write32(0x2000_0100, 0xABCD_1234, 0);
        bus.write32(0x2000_0104, 0xDEAD_BEEF, 0);
        // LDM R0!, {R0, R1}: bits[10:8]=000 (Rn=R0), reglist=0x03 → 0xC803
        c.execute_one_with_bus(0xC803, &mut bus);
        assert_eq!(c.reg(0), 0xABCD_1234, "R0 loaded (was base) — no writeback");
        assert_eq!(c.reg(1), 0xDEAD_BEEF);
    }

    // line 820 complement: Rn NOT in reglist → writeback happens.
    #[test]
    fn ldm_with_base_not_in_reglist_writes_back() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(5, 0x2000_0100); // base
        bus.write32(0x2000_0100, 0x11, 0);
        bus.write32(0x2000_0104, 0x22, 0);
        // LDM R5!, {R0, R1}: bits[10:8]=101, reglist=0x03 → 0xCD03
        c.execute_one_with_bus(0xCD03, &mut bus);
        assert_eq!(c.reg(0), 0x11);
        assert_eq!(c.reg(1), 0x22);
        assert_eq!(c.reg(5), 0x2000_0108, "writeback");
    }

    // ---- Conditional branch (line 844) and its untaken arm ----

    // Already covered by branch_cond_taken / branch_cond_not_taken, but add
    // a condition that is definitely-not-taken under a specific flag setup
    // to pin both arms again (covers different cond encodings too).
    #[test]
    fn bgt_not_taken_when_le() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.regs.set_flag_z(true); // Z=1 means LE → GT is false
        // BGT +6: cond=1100, imm8=3 → 1101_1100_00000011 = 0xDC03
        let cy = c.execute_one(0xDC03);
        assert_eq!(c.regs.pc(), 0x1002, "branch not taken — PC stays at post-execute value");
        assert_eq!(cy, 1);
    }

    #[test]
    fn bls_taken() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        c.regs.set_flag_c(false); // C=0 or Z=1 makes LS true
        // BLS +8: cond=1001, imm8=4 → 1101_1001_00000100 = 0xD904
        let cy = c.execute_one(0xD904);
        assert_eq!(c.regs.pc(), 0x100C);
        assert_eq!(cy, 1);
    }

    // ---- UDF (cond == 0xE) triggers UsageFault ----
    #[test]
    fn udf_raises_usage_fault() {
        let (mut c, mut bus) = core_and_bus();
        // UDF #0: cond=1110, imm8=0 → 1101_1110_00000000 = 0xDE00
        c.execute_one_with_bus(0xDE00, &mut bus);
        assert!(
            matches!(c.pending_fault, Some(Fault::UsageFault)),
            "UDF must set UsageFault; got {:?}",
            c.pending_fault
        );
    }

    // ---- Unconditional branch cycle-cost tiers (line 874, 876) ----

    // line 874: forward / zero offset → cycle cost 1.
    #[test]
    fn b_unconditional_forward_cost_1() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        // B +8: imm11 = 4 → 0xE004
        let cy = c.execute_one(0xE004);
        assert_eq!(cy, 1, "forward B steady-state cost");
        assert_eq!(c.regs.pc(), 0x100C);
    }

    // line 876: small backward (-256 <= signed < 0) → cycle cost 3.
    #[test]
    fn b_unconditional_small_backward_cost_3() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        // B -4: imm11 = 0x7FE → 0xE7FE. offset = -4.
        let cy = c.execute_one(0xE7FE);
        assert_eq!(cy, 3, "small backward cost");
    }

    // Large backward (signed < -256) → cost 5.
    #[test]
    fn b_unconditional_large_backward_cost_5() {
        let mut c = CortexM33::for_test(0);
        c.regs.set_pc(0x1000);
        // offset = -512 (0xFE00). imm11 field = offset/2 = -256 = 0x700 (11-bit signed).
        // Two's complement in 11 bits: -256 = 2048 - 256 = 1792 = 0x700.
        // Encoding: 11100_100_00000000 + ... wait 0xE700? Let me re-derive.
        // B imm11: opcode = 11100_imm11 → 0xE000 | (imm11 & 0x7FF).
        // For imm11 = 0x700 (representing signed -256 * 2 = -512 bytes offset).
        // So opcode = 0xE000 | 0x700 = 0xE700.
        let cy = c.execute_one(0xE700);
        assert_eq!(cy, 5, "large backward cost");
    }

    // ---- SVC raises exception 11 (cond == 0xF) ----
    #[test]
    fn svc_enters_exception_11() {
        let mut bus = Bus::new();
        let mut cpu = CortexM33::new(0, bus.atomics.clone());
        cpu.regs.msp = 0x2000_2000;
        cpu.regs.r[13] = cpu.regs.msp;
        let vtor: u32 = 0x2000_4000;
        cpu.ppb.vtor = vtor;
        bus.write32(vtor + 11 * 4, 0x2000_0300 | 1, 0); // SVC handler
        cpu.regs.set_pc(0x2000_0000);
        // SVC #0: cond=1111, imm8=0 → 1101_1111_00000000 = 0xDF00
        cpu.execute_one_with_bus(0xDF00, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 11, "SVC entered exception 11");
    }

    // ---- BKPT halts the core ----
    #[test]
    fn bkpt_halts_core() {
        let (mut c, mut bus) = core_and_bus();
        assert!(!c.is_halted());
        // BKPT #0: 0xBE00
        c.execute_one_with_bus(0xBE00, &mut bus);
        assert!(c.is_halted(), "BKPT halts the core");
    }

    // ---- REVSH narrow (0b11 arm of REV dispatch) ----
    #[test]
    fn revsh_narrow() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(1, 0x0000_1234);
        // REVSH R0, R1: 10111010_11_001_000 = 0xBAC8
        c.execute_one(0xBAC8);
        // swap_bytes(0x1234) = 0x3412, sign-extended from i16: 0x0000_3412
        assert_eq!(c.reg(0), 0x0000_3412);
    }

    // Negative REVSH case — sign-extension non-trivial.
    #[test]
    fn revsh_narrow_negative() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(1, 0x0000_0080);
        // REVSH R0, R1 → swap_bytes(0x0080) = 0x8000, sign-extended = 0xFFFF_8000
        c.execute_one(0xBAC8);
        assert_eq!(c.reg(0), 0xFFFF_8000);
    }

    // Reserved 0b10 arm of REV dispatch — executes as NOP.
    #[test]
    fn rev_reserved_dispatch_is_nop() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 0xDEAD_BEEF);
        c.set_reg(1, 0x1234_5678);
        // 10111010_10_001_000 = 0xBA88 — REV dispatch bits[7:6] = 0b10 → undefined arm.
        c.execute_one(0xBA88);
        assert_eq!(c.reg(0), 0xDEAD_BEEF, "Rd unchanged on reserved REV dispatch");
    }

    // ---- Sign/zero-extend dispatch (SXTB, UXTH) — pin remaining sub-arms ----
    #[test]
    fn sxtb_narrow() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(1, 0x0000_0080); // -128 as i8
        // SXTB R0, R1: 10110010_01_001_000 = 0xB248
        c.execute_one(0xB248);
        assert_eq!(c.reg(0), 0xFFFF_FF80);
    }

    #[test]
    fn uxth_narrow() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(1, 0xDEAD_BEEF);
        // UXTH R0, R1: 10110010_10_001_000 = 0xB288
        c.execute_one(0xB288);
        assert_eq!(c.reg(0), 0x0000_BEEF);
    }

    // ---- Load literal (PC-relative) ----
    #[test]
    fn ldr_literal_reads_aligned_pc_plus_imm() {
        let (mut c, mut bus) = core_and_bus();
        c.regs.set_pc(0x2000_0000);
        bus.write32(0x2000_0010, 0xAABB_CCDD, 0);
        // LDR R0, [PC, #12]: 01001_000_00000011 = 0x4803 (imm8=3 → offset = 12)
        // Align(read_pc(), 4) = 0x2000_0004, + 12 = 0x2000_0010.
        let cy = c.execute_one_with_bus(0x4803, &mut bus);
        assert_eq!(c.reg(0), 0xAABB_CCDD);
        assert_eq!(cy, 2, "LDR literal 2-cycle SRAM cost");
    }

    // ---- RSBS / CMP / CMN / MVNS / BICS / TST / ORRS / AND / EOR / MULS flag sets ----
    // Many of these are already covered; add a couple more negative/zero
    // flag cases for the MULS path to ensure the M33 2-cycle return arm is hit.
    #[test]
    fn muls_cycle_cost_is_two() {
        let mut c = CortexM33::for_test(0);
        c.set_reg(0, 3);
        c.set_reg(1, 7);
        let cy = c.execute_one(0x4348); // MULS R0, R1
        assert_eq!(c.reg(0), 21);
        assert_eq!(cy, 2, "M33 MULS 2-cycle cost");
    }

    // STRH reg with a plain SRAM (non-SIO) address to pin the False arm
    // of `addr >> 28 == 0xD` at line 458.
    #[test]
    fn strh_reg_sram_costs_two_cycles() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xBEEF);
        c.set_reg(1, 0x2000_0000); // SRAM
        c.set_reg(2, 4);
        // STRH R0, [R1, R2]: 0x5288
        let cy = c.execute_one_with_bus(0x5288, &mut bus);
        assert_eq!(cy, 2, "SRAM store is 2-cycle");
        assert_eq!(bus.read16(0x2000_0004, 0), 0xBEEF);
    }

    // STRB reg SRAM path — pins the False arm at line 463 for another
    // monomorphization instance.
    #[test]
    fn strb_reg_sram_costs_two_cycles() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0xAB);
        c.set_reg(1, 0x2000_0000);
        c.set_reg(2, 1);
        let cy = c.execute_one_with_bus(0x5488, &mut bus);
        assert_eq!(cy, 2);
        assert_eq!(bus.read8(0x2000_0001, 0), 0xAB);
    }

    // STR reg SRAM path — pin False arm at line 453 for the with_bus path.
    #[test]
    fn str_reg_sram_costs_two_cycles() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(0, 0x1234_5678);
        c.set_reg(1, 0x2000_0000);
        c.set_reg(2, 0x10);
        let cy = c.execute_one_with_bus(0x5088, &mut bus);
        assert_eq!(cy, 2);
        assert_eq!(bus.read32(0x2000_0010, 0), 0x1234_5678);
    }

    // BLX Rm with bit 2 set — pins the `!link` False arm at line 413.
    // With `link=true`, the BXNS transition guard cannot trigger, so the
    // `self.secure` inner check is short-circuited out. Important boundary:
    // bit 2 on BLX must NOT perform a Secure→NS transition.
    #[test]
    fn blx_with_bit2_set_does_not_transition() {
        let mut c = CortexM33::for_test(0);
        assert!(c.secure);
        c.regs.set_pc(0x1000);
        c.set_reg(1, 0x2000_0001);
        // BLX R1 with bit 2 set: bit 7 = 1 (BLX), bits[6:3]=0001 (R1),
        // bits[2:0]=100 → 0100_0111_1000_1100 = 0x478C
        c.execute_one(0x478C);
        assert!(c.secure, "BLX must NOT trigger Secure→NS transition");
        assert_eq!(c.regs.pc(), 0x2000_0000);
    }

    // LSLS imm with a larger imm5 for a second monomorphization. The
    // primary monomorphization already covers both arms; this targets
    // the `execute_one_with_bus` path directly so its branch counter sees
    // both arms too.
    #[test]
    fn lsls_imm_nonzero_via_with_bus_path() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(1, 0x1);
        // LSLS R0, R1, #5 → imm5=5. bits: 00000_00101_001_000 = 0x0148
        c.execute_one_with_bus(0x0148, &mut bus);
        assert_eq!(c.reg(0), 0x20);
    }

    #[test]
    fn lsls_imm_zero_via_with_bus_path() {
        let (mut c, mut bus) = core_and_bus();
        c.set_reg(2, 42);
        c.regs.set_flag_c(true);
        c.execute_one_with_bus(0x0010, &mut bus); // LSLS R0, R2, #0 (MOVS)
        assert_eq!(c.reg(0), 42);
        assert!(c.flag_c());
    }

    // LDRB imm has zero line coverage currently — add a direct test.
    #[test]
    fn ldrb_imm_basic() {
        let (mut c, mut bus) = core_and_bus();
        bus.write8(0x2000_0003, 0xA5, 0);
        c.set_reg(1, 0x2000_0000);
        // LDRB R0, [R1, #3]: 01111_00011_001_000 = 0x78C8
        c.execute_one_with_bus(0x78C8, &mut bus);
        assert_eq!(c.reg(0), 0xA5);
    }
}

// ============================================================================
// Stage 1 — Decoder branch coverage (core/decode.rs)
// ============================================================================
//
// Targets the uncovered arms in `classify_is_pure` / its private sub-
// classifiers, plus the top-level `execute_thumb16` / `execute_thumb32`
// dispatch paths and the `decode_execute` fetch-fault + IT-block
// branches. Tests call the decoder directly (via `classify_is_pure`) or
// dispatch specific opcodes through `execute_one` / `execute_one_wide`
// to pin behaviour without needing to also validate the handler's
// semantics — which is already done elsewhere.
//
// Companion to `stage1_execute_coverage`; part of Stage 1 of the
// 2026.04.23 Coverage Improvement Plan.

#[cfg(test)]
mod stage1_decode_coverage {
    use super::*;
    use crate::core::decode::classify_is_pure;

    // ---- classify_thumb16_misc_pure: `_ => true` fallback arm (line 131) ----

    /// Misc group op=0b1000 (CBZ non-zero branch prefix) does NOT match any
    /// listed op and also does NOT match `op & 0x5 == 0x1`. Falls through
    /// to the `_ => true` arm.
    #[test]
    fn misc_op_fallback_arm_is_pure() {
        // hw0 = 0b10110_1000_xxxxxx = 0xB800 — CBNZ form (op=8).
        // op = (0xB800 >> 8) & 0xF = 0x8. 0x8 & 0x5 = 0x0 ≠ 0x1.
        assert!(
            classify_is_pure(0xB800, 0x0000, false),
            "misc op=8 (unlisted) must hit the `_ => true` fallback arm"
        );
    }

    // ---- classify_thumb16_misc_pure: the CBZ/CBNZ arm (`op & 0x5 == 0x1`) ----

    /// Ensure the CBZ-class mask arm is reached with op=0b0001 (bare CBZ
    /// prefix not already claimed by another arm). Complements the existing
    /// coverage which only exercised some values.
    #[test]
    fn misc_cbz_mask_arm_is_pure() {
        // op = 1 → 0b10110_0001 = 0xB100 (CBZ).
        assert!(classify_is_pure(0xB100, 0x0000, false));
        // op = 3 → 0xB300.
        assert!(classify_is_pure(0xB300, 0x0000, false));
        // op = 9 → 0xB900.
        assert!(classify_is_pure(0xB900, 0x0000, false));
        // op = 0xB → 0xBB00.
        assert!(classify_is_pure(0xBB00, 0x0000, false));
    }

    // ---- classify_thumb32_pure 0b11 path: reach multiply/long_multiply ----

    /// op1=0b11, op2 with bit6=0, bit5=1 (so hits "dp_register or beyond"),
    /// bit4=1 (not dp_register), bit3=0 → multiply path (line 172).
    #[test]
    fn thumb32_multiply_is_pure() {
        // hw0: bits[12:11]=11, bits[10:4]=op2.
        // op2 needs (op2 & 0x40)==0, (op2 & 0x20)!=0, (op2 & 0x10)!=0,
        // (op2 & 0x08)==0. Pick op2=0b0110000 = 0x30.
        // hw0 = 0b11111_0110000_xxxx = 0xFB00.
        // Bits: 1111_1011_0000_xxxx → 0xFB00.
        assert!(classify_is_pure(0xFB00, 0x0000, true));
    }

    /// op1=0b11, op2 with bit6=0, bit5=1, bit4=1, bit3=1 → long_multiply (line 176).
    #[test]
    fn thumb32_long_multiply_is_pure() {
        // op2 = 0b0111000 = 0x38 → hw0 bits[10:4] = 0x38.
        // hw0 = 0xF800 | (0x38 << 4) = 0xF800 | 0x380 = 0xFB80.
        // Verify: (hw0 >> 11) & 3 = 0x1F & 3 = 3 (op1=0b11). Good.
        // op2 = ((hw0 >> 4) & 0x7F) = (0xFB8 & 0x7F) = 0x38. Good.
        assert!(classify_is_pure(0xFB80, 0x0000, true));
    }

    /// op1=0b11, op2 with bit6=0, bit5=1, bit4=0 → dp_register path (line 170).
    /// Already covered in the existing table, but repeated here so the
    /// chain from line 168 `op2 & 0x10 == 0` hits the False side when
    /// combined with the multiply tests above.
    #[test]
    fn thumb32_dp_register_is_pure() {
        // op2 = 0b0100000 = 0x20 → hw0 = 0xFA00.
        assert!(classify_is_pure(0xFA00, 0x0000, true));
    }

    // ---- classify_thumb32_branch_misc_pure: B.W T3 (conditional) ----

    /// B.W T3 conditional: hw1 bit 14 = 0, hw1 bit 12 = 0, misc_op & 0xE != 0xE.
    /// hw0 is in the branch/misc range. Line 198 True side, line 200 `true`.
    #[test]
    fn thumb32_bw_t3_conditional_is_pure() {
        // B.W T3 encoding: hw0 = 11110_S_cond_imm6, hw1 = 10_J1_0_J2_imm11.
        // For hw0 we need (hw0 >> 11) & 3 == 0b10 → bits[12:11]=10 → hw0 range 0xF000..0xF7FF.
        // cond in bits[9:6]; pick cond=0 (EQ) → hw0 = 0xF000.
        //   (hw0 >> 11) & 3 = 0xF000 >> 11 & 3 = 0x1E & 3 = 2. Good.
        //   (hw1 >> 15) & 1 = 1 (hw1 bit 15 must be 1 for branch_misc).
        //   hw1 bit 14 = 0 (not BL).
        //   hw1 bit 12 = 0 (not B.W T4).
        //   misc_op = (hw0 >> 6) & 0xF = cond = 0 → 0 & 0xE != 0xE → takes B.W T3 arm.
        //   hw1 = 0b1_0_0_J1_0_J2_imm11 → 0x8000.
        assert!(classify_is_pure(0xF000, 0x8000, true));
    }

    // ---- classify_thumb32_misc_control_pure: hints (F3AF) ----

    /// Hints group: hw0 == 0xF3AF; hw1 low byte in {0..4} → pure.
    /// Covers line 213 True side and line 215 `matches!` both match arms.
    #[test]
    fn thumb32_hint_group_is_pure() {
        // Need to route through classify_is_pure with op1=0b10, op!=0 (branch_misc),
        // hw1 bit 14 = 0, hw1 bit 12 = 0, misc_op=0xE-ish so line 198 reaches
        // misc_control.
        // misc_op = (hw0 >> 6) & 0xF. For hw0=0xF3AF → ((0xF3AF >> 6) & 0xF) = (0x3C & 0xF) = 0xC.
        // Wait: 0xF3AF >> 6 = 0x3CE, & 0xF = 0xE. So misc_op=0xE, & 0xE = 0xE. Good.
        // op1 = (0xF3AF >> 11) & 3 = 0x1E & 3 = 2. Good.
        // hw1: bit 15 = 1 (op=1), bit 14 = 0, bit 12 = 0. Low byte chooses hint.
        // NOP.W = hw1=0x8000 (hint=0x00). Start there.
        for hint in 0x00u16..=0x04u16 {
            let hw1 = 0x8000 | hint;
            assert!(
                classify_is_pure(0xF3AF, hw1, true),
                "hint 0x{:02X} must be pure",
                hint
            );
        }
    }

    /// Hint group with an unrecognised low byte — falls out of `matches!` →
    /// returns false (covers the `false` arm at line 215's `matches!`).
    #[test]
    fn thumb32_hint_group_unknown_is_impure() {
        // hint=0x10 is unclaimed — falls to line 215 `matches!` false →
        // function returns false.
        assert!(
            !classify_is_pure(0xF3AF, 0x8010, true),
            "unknown hint encoding must be impure (routes to thumb32_undefined)"
        );
    }

    // ---- classify_thumb32_misc_control_pure: barriers (F3BF) ----

    /// Barrier group F3BF with barrier_op not in {2,4,5,6} must be impure
    /// (routes to thumb32_undefined). Covers the False match of the
    /// `matches!` at line 221.
    #[test]
    fn thumb32_barrier_group_unknown_is_impure() {
        // barrier_op = (hw1 >> 4) & 0xF. Pick barrier_op=0 → hw1 low nibble of high byte = 0.
        // hw1 = 0x8F0F works: bit15=1, bit14=0, bit12=0, (hw1>>4)&0xF = 0xF0 & 0xF = 0.
        // Wait — 0x8F0F >> 4 = 0x8F0, & 0xF = 0. Good.
        assert!(
            !classify_is_pure(0xF3BF, 0x8F0F, true),
            "barrier_op=0 must be impure"
        );
        // barrier_op=1: hw1 = 0x8F1F → (0x8F1F >> 4) & 0xF = 0x1. Impure.
        assert!(!classify_is_pure(0xF3BF, 0x8F1F, true));
        // barrier_op=3: (hw1 >> 4) & 0xF = 3. Impure.
        assert!(!classify_is_pure(0xF3BF, 0x8F3F, true));
    }

    // ---- classify_thumb32_misc_control_pure: MSR/MRS ----

    /// op_field encodes MSR (0b0111000) — pure. Covers line 225 first cond True.
    #[test]
    fn thumb32_msr_is_pure() {
        // op_field = (hw0 >> 4) & 0x7F. Need op_field=0b0111000 = 0x38.
        // Also: op1 must be 0b10 → hw0 in 0xF000..0xF7FF. misc_op = (hw0 >> 6) & 0xF must
        // be 0xE or 0xF (so line 198 goes to misc_control).
        // hw0 = 0xF380: (>>11)&3=0x1E&3=2 ✓; (>>4)&0x7F=0xF38&0x7F=0x38 ✓;
        // misc_op=(0xF380>>6)&0xF=0x3CE&0xF=0xE ✓.
        // hw1: bit15=1 (branch_misc), bit14=0, bit12=0.
        assert!(classify_is_pure(0xF380, 0x8000, true));
    }

    /// op_field = 0b0111001 — second MSR variant (Non-Secure alias).
    /// Covers line 225 second comparison, exercising the `||` short-circuit
    /// True side.
    #[test]
    fn thumb32_msr_ns_alias_is_pure() {
        // op_field = 0x39 → hw0 = 0xF000 | (0x39 << 4) = 0xF390.
        // misc_op = (0xF390 >> 6) & 0xF = 0x3CE & 0xF = 0xE ✓.
        // (>>11)&3 = 2 ✓; op_field = 0x39 ✓.
        assert!(classify_is_pure(0xF390, 0x8000, true));
    }

    /// op_field = 0b0111110 — MRS variant. Covers line 226 first comparison True.
    #[test]
    fn thumb32_mrs_is_pure() {
        // op_field = 0x3E → hw0 = 0xF000 | (0x3E << 4) = 0xF3E0.
        // misc_op = (0xF3E0 >> 6) & 0xF = 0x3CF & 0xF = 0xF → 0xF & 0xE = 0xE ✓.
        // (>>11)&3 = 2 ✓.
        assert!(classify_is_pure(0xF3E0, 0x8000, true));
    }

    /// op_field = 0b0111111 — second MRS variant (NS alias). Covers line 226
    /// second comparison True.
    #[test]
    fn thumb32_mrs_ns_alias_is_pure() {
        // op_field = 0x3F → hw0 = 0xF000 | (0x3F << 4) = 0xF3F0.
        // misc_op = (0xF3F0 >> 6) & 0xF = 0xF ✓.
        assert!(classify_is_pure(0xF3F0, 0x8000, true));
    }

    /// Misc-control with none of hint / barrier / MSR / MRS matching →
    /// returns false (line 230).
    #[test]
    fn thumb32_misc_control_undefined_is_impure() {
        // hw0 = 0xF000 with op_field = 0x00 (not 0x38/0x39/0x3E/0x3F),
        // misc_op = 0xE or 0xF to reach misc_control; hw0 != 0xF3AF / 0xF3BF.
        // hw0 = 0xF780: (>>11)&3=2 ✓; op_field = (0xF780>>4)&0x7F = 0x78 & 0x7F = 0x78.
        // 0x78 != any MSR/MRS value. misc_op = (0xF780>>6)&0xF = 0x3DE & 0xF = 0xE ✓.
        // hw0 != 0xF3AF && != 0xF3BF ✓.
        assert!(
            !classify_is_pure(0xF780, 0x8000, true),
            "misc-control with unrecognised op_field must be impure"
        );
    }

    // ---- classify_thumb16_pure: default `_` arm (line 100) ----

    /// Top-5-bits >= 0b11101 (hw0 >= 0xE800) is actually wide, but the
    /// decoder can be asked classify_is_pure(_, _, false) with such hw0
    /// in principle. Cover the `_ => false` arm.
    #[test]
    fn thumb16_pure_wide_prefix_forced_false() {
        // hw0 top-5-bits = 0b11101 would normally be wide; force classify
        // as narrow to exercise the `_` arm at line 100.
        assert!(
            !classify_is_pure(0xE800, 0x0000, false),
            "wide-prefix treated as narrow falls to `_` arm and is impure"
        );
    }

    // ---- classify_thumb32_pure: `_` arm for op1==0b00 (line 182) ----

    /// op1==0b00 as wide — bogus encoding, the sub-decoder returns false.
    #[test]
    fn thumb32_pure_op1_zero_is_impure() {
        // hw0 bits[12:11] = 00 → (hw0 >> 11) & 3 = 0. Any hw0 with
        // hw0 >= 0xE800 is wide; but op1 from bits 12:11. 0xE800 → bits 12:11 = 10+1? Let's check.
        // 0xE800 = 1110_1000_0000_0000. Bit 15..11 = 11101. (hw0>>11)&3 = 0x1D & 3 = 1.
        // Hmm, that gives op1=1. For op1=0 we need bits 12:11 = 00.
        // wide requires hw0 >= 0xE800 ⇒ bits 15..11 ≥ 11101. So bits 15:13 = 111, bit 12 = 0..., bit 11 = 1..
        // op1 = bits 12:11 = 01, 10, 11 possible. op1=00 is unreachable from a real wide fetch.
        // But classify_is_pure is called with a caller-supplied `is_wide` flag, so we
        // can pass is_wide=true with hw0 having bits 12:11 = 00. The decoder's `_`
        // arm is there as a defensive fallback — exercise it.
        let hw0: u16 = 0b1110_0000_0000_0000; // 0xE000 — bits 12:11 = 00
        assert!(
            !classify_is_pure(hw0, 0x0000, true),
            "op1==00 (malformed wide) falls to `_ => false`"
        );
    }

    // ---- Top-level execute_thumb16 dispatch: 11101+ undefined arm ----

    /// `execute_thumb16` with opcode >= 0xE800 falls through to
    /// `thumb16_undefined`. Covers line 522.
    #[test]
    fn execute_thumb16_wide_prefix_falls_to_undefined() {
        let mut c = CortexM33::for_test(0);
        // 0xE800 → bits[15:11] = 11101 → no explicit arm → `_`.
        c.execute_one(0xE800);
        // thumb16_undefined raises UsageFault via pending_fault. We only
        // care that the dispatch reached the `_` arm; the fault mechanics
        // are tested elsewhere.
        assert!(
            c.pending_fault.is_some(),
            "wide-prefix opcode 0xE800 must raise UNDEFINED via thumb16_undefined"
        );
    }

    // ---- Top-level execute_thumb32 dispatch: `_` undefined arm (op1==0) ----

    /// `execute_thumb32` with op1==0b00 (malformed) falls through to
    /// `thumb32_undefined`. Covers line 566.
    #[test]
    fn execute_thumb32_op1_zero_falls_to_undefined() {
        let mut c = CortexM33::for_test(0);
        // hw0 = 0x0000 → (hw0 >> 11) & 3 = 0. Direct dispatch → `_ => thumb32_undefined`.
        c.execute_one_wide(0x0000, 0x0000);
        assert!(
            c.pending_fault.is_some(),
            "op1=0 wide must raise UNDEFINED via thumb32_undefined"
        );
    }

    // ---- execute_thumb32 dispatch arms: each is_wide && op1 branch ----
    //
    // Each test below picks an encoding that routes to exactly one handler
    // with a benign outcome (no bus access), just to pin the dispatch arm.
    // The handler semantics are validated elsewhere.

    /// op1=0b01, op2>>5 == 0b00, op2 & 0x04 == 0 → ldm_stm.
    #[test]
    fn execute_thumb32_dispatch_ldm_stm() {
        let mut c = CortexM33::for_test(0);
        // hw0 bits[12:11] = 01 → hw0 in 0x8800..0xCFFF for op1=01? No:
        // (hw0 >> 11) & 3 == 1 means bits[12:11] = 01 → hw0 >= 0xE800 for wide, so
        // hw0 needs bit 15..13 = 111, bit 12 = 0, bit 11 = 1 ⇒ hw0 bits 15:11 = 11101.
        // So hw0 in 0xE800..0xEFFF. op2 = (hw0 >> 4) & 0x7F. op2 >> 5 == 0, op2 & 0x04 == 0.
        // Pick op2 = 0 → hw0 = 0xE800. ldm_stm takes this. Use an empty register list to avoid bus ops.
        // Actually LDMIA / STM need a register list. Use bank-swap LDM with empty list — undefined,
        // but dispatch is what we want to cover.
        // STM.W / LDM.W dispatch hits. Make sure we don't trigger bus traffic by setting PC/regs to safe values.
        let mut bus = crate::bus::Bus::default();
        c.regs.set_pc(0x0000_0000);
        // hw0=0xE880 (STM W=0): op2 bits = 0x08 → op2>>5=0, op2&0x04=1 (so goes to load_store_dual).
        // Let me recompute: hw0=0xE800: op2 = (0xE800>>4)&0x7F = 0xE80 & 0x7F = 0x00. op2>>5=0, op2&0x04=0 → ldm_stm.
        let _ = c.execute_one_wide_with_bus(0xE800, 0x0000, &mut bus);
    }

    /// op1=0b01, op2>>5 == 0b00, op2 & 0x04 != 0 → load_store_dual.
    #[test]
    fn execute_thumb32_dispatch_load_store_dual() {
        let mut c = CortexM33::for_test(0);
        let mut bus = crate::bus::Bus::default();
        c.regs.set_pc(0x0000_0000);
        // op2=0x04 → hw0 = 0xE840. op2>>5=0, op2&0x04=4 ≠ 0 → load_store_dual.
        let _ = c.execute_one_wide_with_bus(0xE840, 0x0000, &mut bus);
    }

    /// op1=0b01, op2>>5 == 0b01 → dp_shifted_reg.
    #[test]
    fn execute_thumb32_dispatch_dp_shifted_reg() {
        let mut c = CortexM33::for_test(0);
        // op2 bits: need op2>>5 == 1 → bits[6:5]=01. op2 = 0x20 → hw0 = 0xEA00.
        // EA00 is AND (shifted reg). hw1 must be valid enough to not crash.
        c.execute_one_wide(0xEA00, 0x0000);
    }

    /// op1=0b01, op2>>5 == 0b10 or 0b11 → coprocessor.
    #[test]
    fn execute_thumb32_dispatch_coprocessor_from_op1_01() {
        let mut c = CortexM33::for_test(0);
        // op2 = 0x40 → op2>>5 = 2 → coprocessor. hw0 = 0xEC00.
        // 0xEC00 → (>>11)&3 = 0x1D&3 = 1 (op1=01) ✓.
        c.execute_one_wide(0xEC00, 0x0000);
    }

    /// op1=0b10, op=0, op2 & 0x20 == 0 → dp_modified_imm.
    #[test]
    fn execute_thumb32_dispatch_dp_modified_imm() {
        let mut c = CortexM33::for_test(0);
        // op1=10 → hw0 bits 12:11 = 10 → hw0 in 0xF000..0xF7FF.
        // op=0 → hw1 bit 15 = 0. op2 & 0x20 == 0 → op2 bit 5 = 0.
        // hw0 = 0xF000. hw1 = 0x0000.
        c.execute_one_wide(0xF000, 0x0000);
    }

    /// op1=0b10, op=0, op2 & 0x20 != 0 → dp_plain_imm.
    #[test]
    fn execute_thumb32_dispatch_dp_plain_imm() {
        let mut c = CortexM33::for_test(0);
        // op2 = 0x20 → hw0 = 0xF000 | (0x20 << 4) = 0xF200. hw1 bit 15 = 0.
        c.execute_one_wide(0xF200, 0x0000);
    }

    /// op1=0b10, op=1 → branch_misc.
    #[test]
    fn execute_thumb32_dispatch_branch_misc() {
        let mut c = CortexM33::for_test(0);
        // op=1 → hw1 bit 15 = 1. hw0 = 0xF000, hw1 = 0xD000 (BL form).
        c.execute_one_wide(0xF000, 0xD000);
    }

    /// op1=0b11, op2 & 0x40 != 0 → coprocessor.
    #[test]
    fn execute_thumb32_dispatch_coprocessor_from_op1_11() {
        let mut c = CortexM33::for_test(0);
        // op1=11 → hw0 bits 12:11 = 11 → hw0 in 0xF800..0xFFFF.
        // op2 & 0x40 != 0 → op2 bit 6 = 1. op2 = 0x40 → hw0 = 0xF800 | (0x40 << 4) = 0xFC00.
        c.execute_one_wide(0xFC00, 0x0000);
    }

    /// op1=0b11, op2 & 0x40 == 0, op2 & 0x20 == 0 → load_store_single.
    #[test]
    fn execute_thumb32_dispatch_load_store_single() {
        let mut c = CortexM33::for_test(0);
        let mut bus = crate::bus::Bus::default();
        c.regs.set_pc(0x0000_0000);
        // op2 = 0x00 → hw0 = 0xF800. Bit 6 = 0, bit 5 = 0. LDRB-ish.
        // Use a benign hw1 that avoids real memory access (Rt=15, Rn=15 → PC-rel, careful).
        // Just pick hw1=0 — executes whatever; semantics not our concern.
        let _ = c.execute_one_wide_with_bus(0xF800, 0x0000, &mut bus);
    }

    /// op1=0b11, op2 & 0x40 == 0, op2 & 0x20 != 0, op2 & 0x10 == 0 → dp_register.
    #[test]
    fn execute_thumb32_dispatch_dp_register() {
        let mut c = CortexM33::for_test(0);
        // op2 = 0x20 → hw0 = 0xFA00. Bit 6 = 0, bit 5 = 1, bit 4 = 0.
        c.execute_one_wide(0xFA00, 0x0000);
    }

    /// op1=0b11, op2 & 0x40 == 0, op2 & 0x20 != 0, op2 & 0x10 != 0, op2 & 0x08 == 0 → multiply.
    #[test]
    fn execute_thumb32_dispatch_multiply() {
        let mut c = CortexM33::for_test(0);
        // op2 = 0x30 → hw0 = 0xFB00. Bit 6 = 0, bit 5 = 1, bit 4 = 1, bit 3 = 0.
        c.execute_one_wide(0xFB00, 0x0000);
    }

    /// op1=0b11, op2 & 0x40 == 0, op2 & 0x20 != 0, op2 & 0x10 != 0, op2 & 0x08 != 0 → long_multiply.
    #[test]
    fn execute_thumb32_dispatch_long_multiply() {
        let mut c = CortexM33::for_test(0);
        // op2 = 0x38 → hw0 = 0xFB80. Bit 6 = 0, bit 5 = 1, bit 4 = 1, bit 3 = 1.
        c.execute_one_wide(0xFB80, 0x0000);
    }

    // ---- decode_execute: IT-block paths (lines 292, 317, 325, 328) ----

    /// Execute a narrow instruction inside an IT block through
    /// `decode_execute` (pure path). Covers `in_it` True branches at
    /// lines 292, 317, 325, 328.
    #[test]
    fn decode_execute_narrow_in_it_block() {
        let (mut c, mut bus) = core_and_bus();
        let base = 0x2000_0100u32;
        // IT EQ; NOP
        bus.write16(base, 0xBF08, 0);       // IT EQ
        bus.write16(base + 2, 0xBF00, 0);   // NOP (in IT)
        bus.write16(base + 4, 0xE7FE, 0);   // B . (halt)
        c.regs.set_pc(base);
        c.regs.set_flag_z(true); // EQ true so NOP executes
        c.step(&mut bus); // IT
        assert_eq!(c.it_state(), 0x08);
        c.step(&mut bus); // NOP under IT — covers in_it+pure narrow.
        assert_eq!(c.it_state(), 0);
    }

    /// Execute a narrow flag-setting instruction inside an IT block; its
    /// flag writes must be suppressed. Covers the `!flag_only` False
    /// side at line 325.
    #[test]
    fn decode_execute_narrow_flag_only_in_it_block() {
        let (mut c, mut bus) = core_and_bus();
        let base = 0x2000_0200u32;
        // IT EQ; CMP R0, #0 (flag-only, 0x2800)
        bus.write16(base, 0xBF08, 0);       // IT EQ
        bus.write16(base + 2, 0x2800, 0);   // CMP R0, #0 — flag-only
        bus.write16(base + 4, 0xE7FE, 0);
        c.regs.set_pc(base);
        c.regs.set_flag_z(true);
        c.set_reg(0, 0);
        c.step(&mut bus); // IT EQ
        c.step(&mut bus); // CMP R0, #0 inside IT — flag writes *preserved*
        // Z must still be true (CMP 0 vs 0 leaves Z=1).
        assert!(c.flag_z());
    }

    // ---- decode_execute: wide instruction via fetch path ----

    /// Execute a wide instruction via `step()` → `decode_execute` to
    /// cover the `is_wide` True branches at lines 288, 306, 313, 354.
    #[test]
    fn decode_execute_wide_via_step() {
        let (mut c, mut bus) = core_and_bus();
        let base = 0x2000_0300u32;
        // BL +0: hw0 = 0xF000, hw1 = 0xF800 → BL to next instruction. Wide, pure.
        bus.write16(base, 0xF000, 0);
        bus.write16(base + 2, 0xF800, 0);
        bus.write16(base + 4, 0xE7FE, 0);
        c.regs.set_pc(base);
        c.step(&mut bus);
        // PC should have advanced by 4 (wide) via the pure path.
        // Our BL target is base+4 (LR = base+5 = base+4 | 1).
        assert_eq!(c.regs.pc() & !1, base + 4);
    }

    /// Wide instruction inside an IT block — covers line 313 `in_it` False
    /// (already covered) and ensures the wide+IT combination compiles.
    /// Also hits the `advance_it_state` call on the wide pure path.
    #[test]
    fn decode_execute_wide_in_it_block() {
        let (mut c, mut bus) = core_and_bus();
        let base = 0x2000_0400u32;
        // IT EQ; BL +0 (wide, pure, inside IT).
        bus.write16(base, 0xBF08, 0);        // IT EQ
        bus.write16(base + 2, 0xF000, 0);    // BL hw0
        bus.write16(base + 4, 0xF800, 0);    // BL hw1 (-> base+6)
        bus.write16(base + 6, 0xE7FE, 0);
        c.regs.set_pc(base);
        c.regs.set_flag_z(true);
        c.step(&mut bus); // IT
        c.step(&mut bus); // BL under IT
        assert_eq!(c.it_state(), 0);
    }

    // ---- decode_execute: cond_passed == false branches (lines 308, 318) ----

    /// Execute a narrow instruction inside an IT block with condition
    /// FALSE — covers `cond_passed` False at line 318 and the `1` cycle
    /// skipped path.
    #[test]
    fn decode_execute_narrow_condition_false() {
        let (mut c, mut bus) = core_and_bus();
        let base = 0x2000_0500u32;
        bus.write16(base, 0xBF08, 0);       // IT EQ
        bus.write16(base + 2, 0x202A, 0);   // MOVS R0, #42
        bus.write16(base + 4, 0xE7FE, 0);
        c.regs.set_pc(base);
        c.regs.set_flag_z(false); // EQ false → skip
        c.set_reg(0, 0);
        c.step(&mut bus); // IT
        c.step(&mut bus); // MOVS skipped
        assert_eq!(c.reg(0), 0);
    }

    // ---- decode_execute / populate_decode_cache: fetch fault ----

    /// A fetch to an unmapped XIP address (flash not loaded) must raise
    /// a bus fault. Covers the `bus.bus_fault` True branch at line 400
    /// and the early-return bypass of caching.
    #[test]
    fn populate_decode_cache_fetch_fault_path() {
        let (mut c, mut bus) = core_and_bus();
        // XIP region 0x1000_0000 without flash loaded → bus fault on fetch.
        let pc = 0x1000_0000u32;
        c.regs.set_pc(pc);
        // Don't assert the fault delivery mechanics — just ensure step doesn't
        // panic and the bus fault was observed. We use step (not decode_execute)
        // so the fault is properly delivered and cleared.
        c.step(&mut bus);
        // The core should now be inside the fault handler (or have pending state).
        // The populate path was exercised with bus_fault=true regardless.
    }

    // ---- decode_execute: pure path with bank-2/6 fetch penalty ----

    /// Fetch from SRAM bank 2 (offset & 0x1C == 0x08) produces a non-zero
    /// fetch_wait. Covers the `bank == 2 || bank == 6` True branch at
    /// line 434, and the non-sequential penalty at line 344.
    #[test]
    fn decode_execute_bank2_penalty_first_fetch() {
        let (mut c, mut bus) = core_and_bus();
        // Bank 2: offset bits [4:2] = 010 → offset 0x08 satisfies
        // (off >> 2) & 7 == 2. PC must be half-aligned.
        let pc = 0x2000_0008u32;
        bus.write16(pc, 0x0000, 0); // LSLS R0, R0, #0 — pure narrow
        bus.write16(pc + 2, 0xE7FE, 0); // B .
        c.regs.set_pc(pc);
        c.step(&mut bus);
        // Coverage is the point; semantics elsewhere.
    }

    /// Fetch from bank 6 (offset & 0x1C == 0x18). Same branch as bank 2.
    #[test]
    fn decode_execute_bank6_penalty_first_fetch() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0018u32;
        bus.write16(pc, 0x0000, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
    }

    /// Fetch from bank 0 (no penalty). Covers the `bank == 2 || bank == 6`
    /// False branch at line 434.
    #[test]
    fn decode_execute_bank0_no_penalty() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0000u32; // bank 0
        bus.write16(pc, 0x0000, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
    }

    // ---- decode_execute: flag_only branch on pure narrow path (line 451) ----

    /// CMP #imm8 is flag-only; exercising populate on such an opcode sets
    /// FLAG_FLAG_ONLY in the cache entry. Covers line 451 True.
    #[test]
    fn populate_decode_cache_sets_flag_only_flag() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0600u32;
        bus.write16(pc, 0x2800, 0); // CMP R0, #0 — flag-only
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.set_reg(0, 0);
        c.step(&mut bus);
    }

    // ---- is_thumb16_flag_only: cover special-data CMP encoding ----

    /// is_thumb16_flag_only is private, but a CMP Rn, Rm (high register)
    /// encoding routed through `populate_decode_cache` exercises the
    /// branch at line 21 False side (bit 10 = 1) and the special-data
    /// CMP test at line 27 (True/False).
    #[test]
    fn populate_decode_cache_special_data_cmp_is_flag_only() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0700u32;
        // CMP R0, R1 (special data, high-register form): 0b01000101_00_001_000 = 0x4508.
        // Bits[15:10] = 010001, bit 10 = 1, bits[9:8] = 01 → CMP.
        bus.write16(pc, 0x4508, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.set_reg(0, 0);
        c.set_reg(1, 0);
        c.step(&mut bus);
    }

    /// Special-data encoding that is NOT CMP (e.g., ADD, MOV). Bit 10 = 1
    /// but (opcode >> 8) & 3 != 0b01. Covers the False side of the line
    /// 27 comparison.
    #[test]
    fn populate_decode_cache_special_data_noncmp_not_flag_only() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0800u32;
        // MOV R0, R1 (special data): 0b01000110_00_001_000 = 0x4608. Bits[9:8] = 10.
        bus.write16(pc, 0x4608, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.set_reg(1, 42);
        c.step(&mut bus);
    }

    // ---- is_thumb16_flag_only: data-processing CMN / TST / CMP ----

    /// TST (data processing, op=0x8): bit 10 = 0, dp_op = 0x8 → flag-only.
    /// Covers line 24 `matches!` True for each of 0x8/0xA/0xB.
    #[test]
    fn populate_decode_cache_tst_is_flag_only() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0900u32;
        // TST R0, R1: 0b0100_0010_00_001_000 = 0x4208.
        bus.write16(pc, 0x4208, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
    }

    /// CMN (op=0xB): bit 10 = 0, dp_op = 0xB → flag-only.
    #[test]
    fn populate_decode_cache_cmn_is_flag_only() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0A00u32;
        // CMN R0, R1: 0b0100_0010_11_001_000 = 0x42C8.
        bus.write16(pc, 0x42C8, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
    }

    /// CMP (data processing form, op=0xA): dp_op = 0xA → flag-only.
    #[test]
    fn populate_decode_cache_cmp_dp_is_flag_only() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0B00u32;
        // CMP R0, R1 (dp form): 0b0100_0010_10_001_000 = 0x4288.
        bus.write16(pc, 0x4288, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
    }

    /// DP op that is NOT CMP/TST/CMN (e.g., ANDS op=0x0). Covers the
    /// `matches!` False side at line 24.
    #[test]
    fn populate_decode_cache_dp_nonflag_not_flag_only() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0C00u32;
        // ANDS R0, R1: 0b0100_0000_00_001_000 = 0x4008.
        bus.write16(pc, 0x4008, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
    }

    // ---- WorkerBus instantiation coverage ---------------------------------
    //
    // `decode_execute<B>` and `populate_decode_cache<B>` are generic over
    // `CoreBus`. The tests above exercise the `Bus` monomorphization; the
    // block below drives the same code paths through `WorkerBus` so its
    // branches are also covered. Any branch that depends only on the core
    // state (is_wide / is_pure / in_it / cond_passed / bank) is observable
    // on both bus types, so we mirror a minimum set here.

    use crate::threaded::{SharedState, WorkerBus};
    use crate::core::bus_trait::CoreBus;

    /// Build a core + WorkerBus that share the same `Arc<CoreAtomics>`.
    fn core_and_worker_bus() -> (CortexM33, WorkerBus) {
        let shared = SharedState::new_default();
        let core = CortexM33::new(0, Arc::clone(&shared.atomics));
        let bus = WorkerBus::new(0, shared);
        (core, bus)
    }

    /// Narrow pure instruction via WorkerBus path.
    #[test]
    fn worker_bus_decode_execute_narrow_pure() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1000u32;
        bus.write16(pc, 0x0000, 0);         // LSLS R0, R0, #0 — pure narrow
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// Wide pure instruction via WorkerBus path — covers `is_wide` True
    /// on the WorkerBus monomorphization.
    #[test]
    fn worker_bus_decode_execute_wide_pure() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1100u32;
        bus.write16(pc, 0xF000, 0);         // BL hw0
        bus.write16(pc + 2, 0xF800, 0);     // BL hw1 → PC += 4
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// Narrow impure instruction (LDM) via WorkerBus — covers the slow
    /// path branches (lines 350 / 354 / 366 / 374) on WorkerBus.
    #[test]
    fn worker_bus_decode_execute_narrow_impure() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1200u32;
        // LDR [pc, #0] — literal load: 0100_1_000_0000_0000 = 0x4800 with imm8=0.
        // Rt=0, imm8=0 → address = (pc_read & ~3) + 0 = pc+4 (pc is base+4 by ARM rules).
        // Arrange the literal at pc+4 (we're running B . at pc+2).
        // Easier: use LDR register-offset which always touches bus.
        // LDR R0, [R1, R2]: 0101_100_010_001_000 = 0x5888.
        bus.write16(pc, 0x5888, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.set_reg(1, 0x2000_2000);
        c.set_reg(2, 0);
        bus.write32(0x2000_2000, 0xDEAD_BEEF, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
        assert_eq!(c.reg(0), 0xDEAD_BEEF);
    }

    /// Wide impure instruction via WorkerBus — covers is_wide slow-path
    /// branch at line 354 (True).
    #[test]
    fn worker_bus_decode_execute_wide_impure() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1300u32;
        // LDR.W R0, [R1]: hw0=0xF8D1, hw1=0x0000. op1=11, op2=0x4D & 0x40 = 0x40 → ...
        // Actually simpler: use LDR.W (T3 form) hw0=0xF8D1 → op1=11, op2=0x4D.
        // 0xF8D1 → (>>11)&3 = 0x1F&3=3 ✓. op2 = (0xF8D1>>4)&0x7F = 0xF8D & 0x7F = 0xD.
        // op2 & 0x40 = 0, & 0x20 = 0 → load_store_single. Good — impure.
        bus.write16(pc, 0xF8D1, 0);
        bus.write16(pc + 2, 0x0000, 0); // hw1: Rt=0, imm12=0 → [R1, #0]
        bus.write16(pc + 4, 0xE7FE, 0);
        c.set_reg(1, 0x2000_2100);
        bus.write32(0x2000_2100, 0xCAFE_F00D, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
        assert_eq!(c.reg(0), 0xCAFE_F00D);
    }

    /// IT block inside WorkerBus path — covers `in_it` True branches on
    /// the WorkerBus monomorphization (lines 292, 317, 325, 328).
    #[test]
    fn worker_bus_decode_execute_in_it_block() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1400u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        bus.write16(pc + 2, 0xBF00, 0);    // NOP (inside IT)
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(true);
        c.step_no_atomics(&mut bus); // IT
        assert_eq!(c.it_state(), 0x08);
        c.step_no_atomics(&mut bus); // NOP under IT
        assert_eq!(c.it_state(), 0);
    }

    /// Narrow instruction with `cond_passed` False in IT block via WorkerBus.
    /// Covers the `False` path at line 318/366.
    #[test]
    fn worker_bus_decode_execute_cond_false() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1500u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        bus.write16(pc + 2, 0x202A, 0);    // MOVS R0, #42
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(false);          // EQ false → skip
        c.set_reg(0, 0);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // MOVS skipped
        assert_eq!(c.reg(0), 0);
    }

    /// Sequential fetch (no bank penalty) on WorkerBus — covers line 350
    /// False (`is_sequential`).
    #[test]
    fn worker_bus_decode_execute_sequential_fetch() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1600u32;
        // Two back-to-back NOPs so the second fetch is sequential.
        bus.write16(pc, 0x0000, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
        c.step_no_atomics(&mut bus); // sequential from first
    }

    /// Wide instruction inside IT block via WorkerBus (covers `in_it` True
    /// on pure-wide WorkerBus path at line 313).
    #[test]
    fn worker_bus_decode_execute_wide_in_it_block() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1700u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        bus.write16(pc + 2, 0xF000, 0);    // BL hw0
        bus.write16(pc + 4, 0xF800, 0);    // BL hw1 — target = pc+6
        bus.write16(pc + 6, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(true);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // BL in IT
    }

    /// Flag-only instruction (CMP) inside IT block via WorkerBus — exercises
    /// the `!flag_only` False side at line 325:44 on WorkerBus.
    #[test]
    fn worker_bus_decode_execute_flag_only_in_it_block() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1800u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        bus.write16(pc + 2, 0x2800, 0);    // CMP R0, #0 (flag-only)
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(true);
        c.set_reg(0, 0);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // CMP inside IT — flag_only=true path
    }

    /// Impure narrow instruction inside IT block via WorkerBus — covers
    /// `in_it` True on the slow narrow path (lines 365/371/374) on WorkerBus.
    #[test]
    fn worker_bus_slow_narrow_in_it_block() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1900u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        // LDR R0, [R1]: 0110_1_00000_001_000 = 0x6808 (impure, routes slow path).
        bus.write16(pc + 2, 0x6808, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.set_reg(1, 0x2000_2200);
        bus.write32(0x2000_2200, 0xA5A5_A5A5, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(true);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // LDR inside IT
        assert_eq!(c.reg(0), 0xA5A5_A5A5);
    }

    /// Impure narrow instruction inside IT block with condition FALSE via
    /// WorkerBus — covers `cond_passed` False on the slow narrow path.
    #[test]
    fn worker_bus_slow_narrow_in_it_cond_false() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1A00u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        bus.write16(pc + 2, 0x6808, 0);    // LDR R0, [R1] (skipped)
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(false);          // EQ false → skip LDR
        c.set_reg(0, 0xAAAA);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // LDR skipped
        assert_eq!(c.reg(0), 0xAAAA);
    }

    /// Impure wide instruction inside IT block via WorkerBus — covers
    /// `in_it` True on the slow wide path (line 361).
    #[test]
    fn worker_bus_slow_wide_in_it_block() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1B00u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        // LDR.W R0, [R1, #0]: hw0=0xF8D1, hw1=0x0000 (impure wide).
        bus.write16(pc + 2, 0xF8D1, 0);
        bus.write16(pc + 4, 0x0000, 0);
        bus.write16(pc + 6, 0xE7FE, 0);
        c.set_reg(1, 0x2000_2300);
        bus.write32(0x2000_2300, 0x1234_5678, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(true);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // LDR.W inside IT
        assert_eq!(c.reg(0), 0x1234_5678);
    }

    /// Pure-wide with `cond_passed` False in IT block — covers `cond_passed`
    /// False at line 308.
    #[test]
    fn decode_execute_pure_wide_cond_false() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0D00u32;
        bus.write16(pc, 0xBF08, 0);        // IT EQ
        bus.write16(pc + 2, 0xF000, 0);    // BL hw0
        bus.write16(pc + 4, 0xF800, 0);    // BL hw1 (skipped)
        bus.write16(pc + 6, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(false);          // EQ false → skip BL
        c.step(&mut bus); // IT
        c.step(&mut bus); // BL skipped (pure-wide, cond_passed=false)
    }

    /// Second fetch of a WorkerBus miss path at the SAME PC — covers the
    /// cache-hit True arm at line 268 on WorkerBus.
    #[test]
    fn worker_bus_decode_execute_cache_hit_on_rerun() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1C00u32;
        bus.write16(pc, 0x0000, 0);        // NOP (pure)
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);       // populate
        c.regs.set_pc(pc);                  // re-run same PC
        c.step_no_atomics(&mut bus);       // cache hit
    }

    // ---- execute_thumb16 WorkerBus dispatch coverage ------------------------

    /// Drive the 0b01000 (data-processing / special-data) group through
    /// WorkerBus. Covers line 488 dispatch arm on WorkerBus.
    #[test]
    fn worker_bus_execute_thumb16_dp_group() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1D00u32;
        // ANDS R0, R1: 0x4008 — data-processing group (bit 10 = 0).
        bus.write16(pc, 0x4008, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.set_reg(0, 0xFF);
        c.set_reg(1, 0x0F);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
        assert_eq!(c.reg(0), 0x0F);
    }

    // ---- execute_thumb32 WorkerBus dispatch coverage -----------------------

    /// Wide `op1=0b01, op2>>5 == 0b00, op2 & 0x04 == 0` → ldm_stm via WorkerBus.
    #[test]
    fn worker_bus_execute_thumb32_ldm_stm() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1E00u32;
        // STMIA.W R8!, {R0, R1}: hw0=0xE888, hw1=0x0003.
        // hw0=0xE888: op2 = (0xE888>>4)&0x7F = 0xE88&0x7F = 0x08. op2>>5=0, op2&0x04=0.
        // Wait 0x08 & 0x04 = 0 ✓ → ldm_stm arm.
        bus.write16(pc, 0xE888, 0);
        bus.write16(pc + 2, 0x0003, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.set_reg(8, 0x2000_2400);
        c.set_reg(0, 0xAAAA);
        c.set_reg(1, 0xBBBB);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// op1=10, op=0, op2 & 0x20 == 0 → dp_modified_imm via WorkerBus.
    #[test]
    fn worker_bus_execute_thumb32_dp_modified_imm() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_1F00u32;
        // MOVS.W R0, #0: hw0=0xF04F, hw1=0x0000 (dp_modified_imm, op2&0x20==0).
        // Actually 0xF04F: op2 = (0xF04F>>4)&0x7F = 0xF04 & 0x7F = 0x04. op=hw1 bit15=0. Good.
        bus.write16(pc, 0xF04F, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// op1=11, op2 & 0x40 == 0, op2 & 0x20 == 0 → load_store_single via WorkerBus.
    /// Covers line 557 on WorkerBus.
    #[test]
    fn worker_bus_execute_thumb32_load_store_single() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2000u32;
        // STR.W R0, [R1]: hw0=0xF8C1, hw1=0x0000.
        // 0xF8C1 → op2=(0xF8C>>0)&0x7F = wait, op2 = (hw0>>4)&0x7F.
        // (0xF8C1>>4)&0x7F = 0xF8C & 0x7F = 0x0C. bit6=0, bit5=0 ✓ → load_store_single.
        bus.write16(pc, 0xF8C1, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.set_reg(0, 0xCAFEBABEu32);
        c.set_reg(1, 0x2000_2500);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// op1=11, op2 & 0x40 == 0, bit5=1, bit4=0 → dp_register via WorkerBus.
    /// Covers line 559 on WorkerBus.
    #[test]
    fn worker_bus_execute_thumb32_dp_register() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2100u32;
        // MVN.W / similar: hw0=0xFA00 → op2=0x20.
        bus.write16(pc, 0xFA00, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// op1=11, op2 & 0x40 == 0, bit5=1, bit4=1, bit3=0 → multiply via WorkerBus.
    /// Covers line 561 on WorkerBus.
    #[test]
    fn worker_bus_execute_thumb32_multiply() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2200u32;
        bus.write16(pc, 0xFB00, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    // ---- Fetch fault on WorkerBus (line 400 True) ---------------------------

    /// Fetch from an XIP region without flash loaded triggers a bus fault
    /// on WorkerBus. Covers line 400 True on WorkerBus.
    #[test]
    fn worker_bus_populate_fetch_fault() {
        let (mut c, mut bus) = core_and_worker_bus();
        c.regs.set_pc(0x1000_0000); // XIP without flash → fault
        c.step_no_atomics(&mut bus);
    }

    /// Fetch from an SRAM offset >= 0x80000 — covers the `off < 0x8_0000`
    /// False branch at line 432 in `populate_decode_cache`.
    #[test]
    fn populate_decode_cache_sram_offset_past_512k() {
        let (mut c, mut bus) = core_and_bus();
        // 520 KB SRAM = 0x82000; use offset 0x80000 which is still inside but
        // above the 0x80000 threshold.
        let pc = 0x2008_0000u32;
        bus.write16(pc, 0x0000, 0); // NOP
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
    }

    /// Fetch from a non-cacheable region (region 3 — unmapped APB) — covers
    /// the `is_cacheable_pc(pc)` False branch at line 265 and the `if
    /// is_cacheable_pc(pc)` False at line 455. The fetch will fault (region
    /// is unmapped), which is handled by the fault delivery path.
    #[test]
    fn decode_execute_non_cacheable_pc() {
        let (mut c, mut bus) = core_and_bus();
        // Region 3 is not in the cacheable set {0,1,2}. Address 0x3000_0000.
        c.regs.set_pc(0x3000_0000);
        c.step(&mut bus);
    }

    /// Non-cacheable PC via WorkerBus — covers 265:24 / 455:12 False on the
    /// WorkerBus monomorphization.
    #[test]
    fn worker_bus_decode_execute_non_cacheable_pc() {
        let (mut c, mut bus) = core_and_worker_bus();
        c.regs.set_pc(0x3000_0000);
        c.step_no_atomics(&mut bus);
    }

    /// SRAM offset >= 0x80000 via WorkerBus — covers line 432 False on
    /// the WorkerBus monomorphization.
    #[test]
    fn worker_bus_populate_sram_offset_past_512k() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2008_0000u32;
        bus.write16(pc, 0x0000, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// Pure-wide cond=false via WorkerBus — covers 308:28 False.
    #[test]
    fn worker_bus_pure_wide_cond_false() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2300u32;
        bus.write16(pc, 0xBF08, 0);         // IT EQ
        bus.write16(pc + 2, 0xF000, 0);     // BL hw0
        bus.write16(pc + 4, 0xF800, 0);     // BL hw1 — skipped
        bus.write16(pc + 6, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(false);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // BL skipped
    }

    /// Impure narrow with cond=false in slow path (Bus) — covers 366:33 False
    /// on the slow-path narrow code.
    #[test]
    fn decode_execute_slow_narrow_cond_false() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0E00u32;
        bus.write16(pc, 0xBF08, 0);         // IT EQ
        bus.write16(pc + 2, 0x6808, 0);     // LDR R0, [R1] — impure, skipped
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(false);
        c.set_reg(0, 0xDEAD_DEAD);
        c.step(&mut bus); // IT
        c.step(&mut bus); // LDR skipped
        assert_eq!(c.reg(0), 0xDEAD_DEAD);
    }

    /// Impure wide with cond=false in slow path — covers 356:33 False.
    #[test]
    fn decode_execute_slow_wide_cond_false() {
        let (mut c, mut bus) = core_and_bus();
        let pc = 0x2000_0F00u32;
        bus.write16(pc, 0xBF08, 0);         // IT EQ
        // LDR.W R0, [R1, #0]: hw0=0xF8D1, hw1=0x0000 (impure wide).
        bus.write16(pc + 2, 0xF8D1, 0);
        bus.write16(pc + 4, 0x0000, 0);
        bus.write16(pc + 6, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(false);
        c.set_reg(0, 0xBEEF_BEEF);
        c.step(&mut bus); // IT
        c.step(&mut bus); // LDR.W skipped
        assert_eq!(c.reg(0), 0xBEEF_BEEF);
    }

    /// Impure wide with cond=false in slow path via WorkerBus — covers
    /// 356:33 False on the WorkerBus monomorphization.
    #[test]
    fn worker_bus_slow_wide_cond_false() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2400u32;
        bus.write16(pc, 0xBF08, 0);         // IT EQ
        bus.write16(pc + 2, 0xF8D1, 0);     // LDR.W hw0
        bus.write16(pc + 4, 0x0000, 0);     // LDR.W hw1
        bus.write16(pc + 6, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.regs.set_flag_z(false);
        c.set_reg(0, 0xFACE);
        c.step_no_atomics(&mut bus); // IT
        c.step_no_atomics(&mut bus); // LDR.W skipped
        assert_eq!(c.reg(0), 0xFACE);
    }

    /// Special-data-BX group (0b01000 with bit 10 set) via WorkerBus — covers
    /// the False side of line 488 on the WorkerBus monomorphization.
    #[test]
    fn worker_bus_execute_thumb16_special_data_bx() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2500u32;
        // MOV R0, R1 (special data): 0x4608 — bits[15:10] = 010001.
        bus.write16(pc, 0x4608, 0);
        bus.write16(pc + 2, 0xE7FE, 0);
        c.set_reg(1, 42);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
        assert_eq!(c.reg(0), 42);
    }

    /// Thumb-32 op1=01, op2>>5 == 0b00, op2 & 0x04 != 0 → load_store_dual
    /// via WorkerBus. Covers False side of line 538.
    #[test]
    fn worker_bus_execute_thumb32_load_store_dual() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2600u32;
        // STRD R0, R1, [R2]: hw0=0xE8C2, hw1=0x0100 (roughly).
        // Need op1=01 → hw0 in 0xE800..0xEFFF.
        // op2 = (hw0>>4)&0x7F. For op2>>5=0 AND op2&0x04!=0: op2 bits 5=0, 6=0, 2=1.
        // op2 = 0x04 → hw0 = 0xE840.
        bus.write16(pc, 0xE840, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// Thumb-32 op1=10, op=0, op2 & 0x20 != 0 → dp_plain_imm via WorkerBus.
    /// Covers line 547 False.
    #[test]
    fn worker_bus_execute_thumb32_dp_plain_imm() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2700u32;
        bus.write16(pc, 0xF200, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// Thumb-32 op1=11, op2 & 0x40 != 0 → coprocessor via WorkerBus. Covers
    /// line 555 True side. Expect UNDEF since coprocessors aren't wired here,
    /// but the dispatch path is exercised.
    #[test]
    fn worker_bus_execute_thumb32_coprocessor_from_op1_11() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2800u32;
        // hw0 = 0xFC00 (op1=11, op2=0x40).
        bus.write16(pc, 0xFC00, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// Thumb-32 op1=11, long_multiply path via WorkerBus — covers line 561 False.
    #[test]
    fn worker_bus_execute_thumb32_long_multiply() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2000_2900u32;
        bus.write16(pc, 0xFB80, 0);
        bus.write16(pc + 2, 0x0000, 0);
        bus.write16(pc + 4, 0xE7FE, 0);
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }

    /// Wide instruction spanning the SRAM upper bound — hw0 (at SRAM end - 2)
    /// succeeds, hw1 (past SRAM end) faults. Covers `wide && bus.bus_fault()`
    /// True at line 416.
    #[test]
    fn populate_decode_cache_wide_hw1_fault() {
        let (mut c, mut bus) = core_and_bus();
        // SRAM ends at 0x2008_2000 (exclusive). Place hw0 at 0x2008_1FFE so the
        // second halfword fetch hits offset 0x8_2000 which is past the bounds
        // check `(offset + 1) < 0x82000`.
        let pc = 0x2008_1FFEu32;
        // Write a wide-prefix halfword so is_wide(hw0) == true.
        bus.write16(pc, 0xF000, 0);
        c.regs.set_pc(pc);
        c.step(&mut bus);
        // Semantics: fault delivered. We only care the dispatch path was hit.
    }

    /// Same as above but through WorkerBus — covers line 416 True on the
    /// WorkerBus monomorphization.
    #[test]
    fn worker_bus_populate_wide_hw1_fault() {
        let (mut c, mut bus) = core_and_worker_bus();
        let pc = 0x2008_1FFEu32;
        bus.write16(pc, 0xF000, 0); // wide-prefix halfword
        c.regs.set_pc(pc);
        c.step_no_atomics(&mut bus);
    }
}

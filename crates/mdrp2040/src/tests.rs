//! Phase 4.A unit tests for the Cortex-M0+ core.
//!
//! One test module per Thumb-16 group. Each group covers happy-path
//! semantics plus carry / overflow edge cases for flag-setting
//! instructions. M0+-specific undefined encodings (IT, CBZ/CBNZ) have
//! dedicated rejection tests.

use crate::bus::Bus;
use crate::core::CortexM0Plus;

// ---------------------------------------------------------------------------
// CP1 — Registers + decoder skeleton
// ---------------------------------------------------------------------------

mod registers {
    use crate::core::registers::{Registers, XPSR_T};

    #[test]
    fn reset_has_thumb_bit_set() {
        let r = Registers::new();
        assert_eq!(r.xpsr, XPSR_T);
        assert!(!r.flag_n() && !r.flag_z() && !r.flag_c() && !r.flag_v());
    }

    #[test]
    fn flag_accessors_roundtrip() {
        let mut r = Registers::new();
        r.set_flag_n(true);
        r.set_flag_z(true);
        r.set_flag_c(true);
        r.set_flag_v(true);
        assert!(r.flag_n() && r.flag_z() && r.flag_c() && r.flag_v());

        r.set_flag_n(false);
        r.set_flag_z(false);
        r.set_flag_c(false);
        r.set_flag_v(false);
        assert!(!r.flag_n() && !r.flag_z() && !r.flag_c() && !r.flag_v());
    }

    #[test]
    fn set_nzcv_clears_before_set() {
        let mut r = Registers::new();
        r.set_nzcv(true, true, true, true);
        r.set_nzcv(false, true, false, true);
        assert!(!r.flag_n() && r.flag_z() && !r.flag_c() && r.flag_v());
    }

    #[test]
    fn set_nz_picks_up_sign_and_zero() {
        let mut r = Registers::new();
        r.set_nz(0);
        assert!(!r.flag_n() && r.flag_z());
        r.set_nz(0x8000_0000);
        assert!(r.flag_n() && !r.flag_z());
        r.set_nz(1);
        assert!(!r.flag_n() && !r.flag_z());
    }

    #[test]
    fn condition_passed_covers_all_codes() {
        let mut r = Registers::new();
        // Z = 1
        r.set_flag_z(true);
        assert!(r.condition_passed(0x0)); // EQ
        assert!(!r.condition_passed(0x1)); // NE
        // C = 1
        r.set_flag_c(true);
        assert!(r.condition_passed(0x2));
        assert!(!r.condition_passed(0x3));
        // N = V → GE
        r.set_flag_n(true);
        r.set_flag_v(true);
        assert!(r.condition_passed(0xA));
        assert!(!r.condition_passed(0xB));
        // AL
        assert!(r.condition_passed(0xE));
    }

    #[test]
    fn sp_banking_helpers_respect_control_spsel() {
        let mut r = Registers::new();
        // Thread mode, SPSEL = 0 → MSP
        r.r[13] = 0x2000_0000;
        r.sync_sp_to_banked();
        assert_eq!(r.msp, 0x2000_0000);
        // Switch to PSP
        r.control |= 2;
        r.r[13] = 0x2000_1000;
        r.sync_sp_to_banked();
        assert_eq!(r.psp, 0x2000_1000);
        // Handler mode forces MSP even with SPSEL=1
        r.xpsr |= 0x1; // IPSR = 1 (non-zero)
        assert!(!r.active_sp_is_psp());
    }
}

mod decoder {
    use crate::core::decode::is_wide;

    #[test]
    fn is_wide_accepts_only_11110_prefix() {
        assert!(is_wide(0xF000));
        assert!(is_wide(0xF7FF));
        assert!(!is_wide(0xE800)); // M33 accepts this (0b11101); M0+ does not
        assert!(!is_wide(0xF800)); // 0b11111 — M33 accepts, M0+ does not
        assert!(!is_wide(0x0000));
        assert!(!is_wide(0xE000)); // unconditional B (0b11100) is Thumb-16
    }
}

// ---------------------------------------------------------------------------
// CP2 — Thumb-16 groups 0b00000..=0b00111
// ---------------------------------------------------------------------------

mod shifts_imm {
    use super::*;

    #[test]
    fn lsls_imm_sets_carry_from_msb_shifted_out() {
        // LSLS r0, r1, #1 → r0 = r1 << 1
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x8000_0000;
        cpu.execute_one(0x0048); // LSLS r0, r1, #1
        assert_eq!(cpu.regs.r[0], 0);
        assert!(cpu.flag_z());
        assert!(cpu.flag_c());
    }

    #[test]
    fn lsls_imm_zero_is_movs_preserves_carry() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_flag_c(true);
        cpu.regs.r[1] = 0x1234;
        cpu.execute_one(0x0008); // LSLS r0, r1, #0 == MOVS r0, r1
        assert_eq!(cpu.regs.r[0], 0x1234);
        assert!(cpu.flag_c());
    }

    #[test]
    fn lsrs_imm_zero_means_shift_by_32() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x8000_0000;
        cpu.execute_one(0x0848); // LSRS r0, r1, #1
        assert_eq!(cpu.regs.r[0], 0x4000_0000);
        assert!(!cpu.flag_c());

        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x8000_0000;
        cpu.execute_one(0x0808); // LSRS r0, r1, #0 → shift by 32
        assert_eq!(cpu.regs.r[0], 0);
        assert!(cpu.flag_z());
        assert!(cpu.flag_c());
    }

    #[test]
    fn asrs_imm_sign_extends() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0xFFFF_FFFE;
        cpu.execute_one(0x1048); // ASRS r0, r1, #1
        assert_eq!(cpu.regs.r[0], 0xFFFF_FFFF);
        assert!(cpu.flag_n());
        assert!(!cpu.flag_c());
    }

    #[test]
    fn asrs_imm_zero_is_shift_by_32() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x8000_0000;
        cpu.execute_one(0x1008); // ASRS r0, r1, #0 → shift by 32
        assert_eq!(cpu.regs.r[0], 0xFFFF_FFFF);
        assert!(cpu.flag_n());
        assert!(cpu.flag_c());
    }
}

mod add_sub_reg_imm3 {
    use super::*;

    #[test]
    fn adds_reg_sets_all_flags() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x7FFF_FFFF;
        cpu.regs.r[2] = 1;
        cpu.execute_one(0x1888); // ADDS r0, r1, r2
        assert_eq!(cpu.regs.r[0], 0x8000_0000);
        assert!(cpu.flag_n() && !cpu.flag_z() && !cpu.flag_c() && cpu.flag_v());
    }

    #[test]
    fn subs_reg_sets_carry_on_no_borrow() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 10;
        cpu.regs.r[2] = 3;
        cpu.execute_one(0x1A88); // SUBS r0, r1, r2
        assert_eq!(cpu.regs.r[0], 7);
        assert!(!cpu.flag_n() && !cpu.flag_z() && cpu.flag_c() && !cpu.flag_v());
    }

    #[test]
    fn subs_reg_clears_carry_on_borrow() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 3;
        cpu.regs.r[2] = 10;
        cpu.execute_one(0x1A88); // SUBS r0, r1, r2
        assert_eq!(cpu.regs.r[0], 0xFFFF_FFF9);
        assert!(cpu.flag_n() && !cpu.flag_z() && !cpu.flag_c());
    }

    #[test]
    fn adds_imm3_sets_z_when_zero() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0;
        cpu.execute_one(0x1C08); // ADDS r0, r1, #0
        assert_eq!(cpu.regs.r[0], 0);
        assert!(cpu.flag_z());
    }

    #[test]
    fn subs_imm3() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 5;
        cpu.execute_one(0x1E88); // SUBS r0, r1, #2
        assert_eq!(cpu.regs.r[0], 3);
        assert!(!cpu.flag_n() && !cpu.flag_z() && cpu.flag_c());
    }
}

mod mov_cmp_add_sub_imm8 {
    use super::*;

    #[test]
    fn movs_imm8_sets_z_when_zero() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0x2000); // MOVS r0, #0
        assert_eq!(cpu.regs.r[0], 0);
        assert!(cpu.flag_z());
    }

    #[test]
    fn movs_imm8_clears_z_for_positive() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_flag_z(true);
        cpu.execute_one(0x2042); // MOVS r0, #0x42
        assert_eq!(cpu.regs.r[0], 0x42);
        assert!(!cpu.flag_z());
    }

    #[test]
    fn cmp_imm8_sets_z_when_equal() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0x42;
        cpu.execute_one(0x2842); // CMP r0, #0x42
        assert_eq!(cpu.regs.r[0], 0x42);
        assert!(cpu.flag_z());
        assert!(cpu.flag_c());
    }

    #[test]
    fn adds_imm8_wraps_and_sets_carry() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xFFFF_FFFF;
        cpu.execute_one(0x3001); // ADDS r0, #1
        assert_eq!(cpu.regs.r[0], 0);
        assert!(cpu.flag_z());
        assert!(cpu.flag_c());
    }

    #[test]
    fn subs_imm8() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 10;
        cpu.execute_one(0x3805); // SUBS r0, #5
        assert_eq!(cpu.regs.r[0], 5);
        assert!(cpu.flag_c());
    }
}

// ---------------------------------------------------------------------------
// CP3 — Thumb-16 group 0b01000 (data processing + special data + BX)
// ---------------------------------------------------------------------------

mod data_processing {
    use super::*;

    #[test]
    fn ands_reg() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xF0F0;
        cpu.regs.r[1] = 0x0FF0;
        cpu.execute_one(0x4008); // ANDS r0, r1
        assert_eq!(cpu.regs.r[0], 0x00F0);
    }

    #[test]
    fn eors_reg_sets_z_on_self() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xDEAD_BEEF;
        cpu.execute_one(0x4040); // EORS r0, r0
        assert_eq!(cpu.regs.r[0], 0);
        assert!(cpu.flag_z());
    }

    #[test]
    fn lsls_reg_shift_by_zero_preserves_carry() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0x1234;
        cpu.regs.r[1] = 0;
        cpu.regs.set_flag_c(true);
        cpu.execute_one(0x4088); // LSLS r0, r1
        assert_eq!(cpu.regs.r[0], 0x1234);
        assert!(cpu.flag_c());
    }

    #[test]
    fn lsls_reg_shift_by_33_clears_value_and_carry() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xFFFF_FFFF;
        cpu.regs.r[1] = 33;
        cpu.execute_one(0x4088); // LSLS r0, r1
        assert_eq!(cpu.regs.r[0], 0);
        assert!(!cpu.flag_c());
    }

    #[test]
    fn lsrs_reg_shift_by_32_moves_bit31_to_carry() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0x8000_0000;
        cpu.regs.r[1] = 32;
        cpu.execute_one(0x40C8); // LSRS r0, r1
        assert_eq!(cpu.regs.r[0], 0);
        assert!(cpu.flag_c());
    }

    #[test]
    fn asrs_reg_large_shift_saturates_sign() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0x8000_0000;
        cpu.regs.r[1] = 40;
        cpu.execute_one(0x4108); // ASRS r0, r1
        assert_eq!(cpu.regs.r[0], 0xFFFF_FFFF);
        assert!(cpu.flag_c());
    }

    #[test]
    fn adcs_reg_respects_carry_in() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 1;
        cpu.regs.r[1] = 1;
        cpu.regs.set_flag_c(true);
        cpu.execute_one(0x4148); // ADCS r0, r1
        assert_eq!(cpu.regs.r[0], 3);
    }

    #[test]
    fn sbcs_reg_with_carry_in() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 10;
        cpu.regs.r[1] = 3;
        cpu.regs.set_flag_c(true); // C=1 means no borrow
        cpu.execute_one(0x4188); // SBCS r0, r1
        assert_eq!(cpu.regs.r[0], 7);
    }

    #[test]
    fn rors_reg() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0x0000_0001;
        cpu.regs.r[1] = 1;
        cpu.execute_one(0x41C8); // RORS r0, r1
        assert_eq!(cpu.regs.r[0], 0x8000_0000);
        assert!(cpu.flag_c());
        assert!(cpu.flag_n());
    }

    #[test]
    fn tst_reg_updates_flags_no_dest() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xFF;
        cpu.regs.r[1] = 0x100;
        cpu.execute_one(0x4208); // TST r0, r1
        assert_eq!(cpu.regs.r[0], 0xFF);
        assert!(cpu.flag_z());
    }

    #[test]
    fn rsbs_neg_negates_value() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 5;
        cpu.execute_one(0x4248); // RSBS r0, r1, #0 (NEG)
        assert_eq!(cpu.regs.r[0], 0xFFFF_FFFB);
        assert!(cpu.flag_n());
    }

    #[test]
    fn cmp_reg_low_equal() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 7;
        cpu.regs.r[1] = 7;
        cpu.execute_one(0x4288); // CMP r0, r1
        assert!(cpu.flag_z());
        assert!(cpu.flag_c());
    }

    #[test]
    fn cmn_reg_low() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 1;
        cpu.regs.r[1] = 0xFFFF_FFFF;
        cpu.execute_one(0x42C8); // CMN r0, r1
        assert!(cpu.flag_z());
        assert!(cpu.flag_c());
    }

    #[test]
    fn orrs_reg() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xF0;
        cpu.regs.r[1] = 0x0F;
        cpu.execute_one(0x4308); // ORRS r0, r1
        assert_eq!(cpu.regs.r[0], 0xFF);
    }

    #[test]
    fn muls_low_32_bits() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 7;
        cpu.regs.r[1] = 6;
        cpu.execute_one(0x4348); // MULS r0, r1
        assert_eq!(cpu.regs.r[0], 42);
    }

    #[test]
    fn muls_discards_overflow() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0x1_0000;
        cpu.regs.r[1] = 0x1_0000;
        cpu.execute_one(0x4348); // MULS r0, r1
        assert_eq!(cpu.regs.r[0], 0); // (1<<32) truncates
        assert!(cpu.flag_z());
    }

    #[test]
    fn mul_preserves_c_and_v() {
        // ARMv6-M A6.7.81 (MUL T1): MULS updates N and Z, leaves C and V
        // unchanged. 7 * 6 = 42 gives N=0, Z=0 so we can observe C/V
        // carried across the instruction cleanly.
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 7;
        cpu.regs.r[1] = 6;
        cpu.regs.set_flag_c(true);
        cpu.regs.set_flag_v(true);
        cpu.execute_one(0x4348); // MULS r0, r1
        assert_eq!(cpu.regs.r[0], 42);
        assert!(!cpu.flag_n());
        assert!(!cpu.flag_z());
        assert!(cpu.flag_c());
        assert!(cpu.flag_v());
    }

    #[test]
    fn bics_clears_bits() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xFF;
        cpu.regs.r[1] = 0x0F;
        cpu.execute_one(0x4388); // BICS r0, r1
        assert_eq!(cpu.regs.r[0], 0xF0);
    }

    #[test]
    fn mvns_inverts_bits() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0;
        cpu.execute_one(0x43C8); // MVNS r0, r1
        assert_eq!(cpu.regs.r[0], 0xFFFF_FFFF);
        assert!(cpu.flag_n());
    }
}

mod special_data_and_bx {
    use super::*;

    #[test]
    fn add_high_reg_no_flag_update() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 1;
        cpu.regs.r[8] = 2;
        cpu.execute_one(0x4440); // ADD r0, r8
        assert_eq!(cpu.regs.r[0], 3);
        assert!(!cpu.flag_z());
    }

    #[test]
    fn cmp_high_reg_updates_flags() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[8] = 10;
        cpu.regs.r[9] = 10;
        cpu.execute_one(0x45C8); // CMP r8, r9  (D:Rd=1000, Rm=1001 -> op=01)
        assert!(cpu.flag_z());
        assert!(cpu.flag_c());
    }

    #[test]
    fn mov_high_reg_no_flag_update() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[8] = 0xABCD;
        cpu.execute_one(0x4640); // MOV r0, r8
        assert_eq!(cpu.regs.r[0], 0xABCD);
    }

    #[test]
    fn bx_register_sets_pc() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[2] = 0x2000_1001; // Thumb bit set
        cpu.execute_one(0x4710); // BX r2
        assert_eq!(cpu.regs.r[15], 0x2000_1000); // T bit cleared on PC
    }

    #[test]
    fn blx_register_writes_lr_and_pc() {
        let mut cpu = CortexM0Plus::new();
        // `execute_one_with_bus` latches current_instr_addr from the
        // incoming PC. Set PC to the instruction address; the helper
        // then advances PC by 2 internally.
        cpu.regs.set_pc(0x1000);
        cpu.regs.r[3] = 0x2000_3001;
        cpu.execute_one_with_bus(0x4798, &mut Bus::default()); // BLX r3
        assert_eq!(cpu.regs.r[14], 0x1003); // (instr_addr+2) | 1
        assert_eq!(cpu.regs.r[15], 0x2000_3000);
    }
}

// ---------------------------------------------------------------------------
// CP4 — LDR literal + loads/stores by reg/imm/halfword/byte
// ---------------------------------------------------------------------------

mod ldr_literal {
    use super::*;

    #[test]
    fn ldr_pc_relative_word_aligned() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        // Instr at 0x2000_0000; helper moves PC to +2; read_pc = +4 = 0x2000_0004.
        // Word-aligned base = 0x2000_0004. Offset 1*4 = +4 → 0x2000_0008.
        cpu.regs.set_pc(0x2000_0000);
        bus.write32(0x2000_0008, 0xCAFE_BABE);
        cpu.execute_one_with_bus(0x4801, &mut bus); // LDR r0, [PC, #4]
        assert_eq!(cpu.regs.r[0], 0xCAFE_BABE);
    }

    #[test]
    fn ldr_pc_relative_aligns_base_down() {
        // Instr at 0x2000_0002 (halfword-aligned, not word-aligned).
        // read_pc = 0x2000_0006; base = 0x2000_0006 & !3 = 0x2000_0004.
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.set_pc(0x2000_0002);
        bus.write32(0x2000_0004, 0x1234_5678);
        cpu.execute_one_with_bus(0x4800, &mut bus); // LDR r0, [PC, #0]
        assert_eq!(cpu.regs.r[0], 0x1234_5678);
    }

    #[test]
    fn ldr_pc_relative_max_offset() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.set_pc(0x2000_0000);
        // base 0x2000_0004 + 0xFF*4 = 0x2000_0400
        bus.write32(0x2000_0400, 0xDEAD_BEEF);
        cpu.execute_one_with_bus(0x48FF, &mut bus); // LDR r0, [PC, #0x3FC]
        assert_eq!(cpu.regs.r[0], 0xDEAD_BEEF);
    }
}

mod load_store_reg {
    use super::*;

    fn cpu_with_bus() -> (CortexM0Plus, Bus) {
        (CortexM0Plus::new(), Bus::default())
    }

    #[test]
    fn str_reg_writes_word() {
        let (mut cpu, mut bus) = cpu_with_bus();
        cpu.regs.r[0] = 0xDEAD_BEEF;
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 4;
        cpu.execute_one_with_bus(0x5088, &mut bus); // STR r0, [r1, r2]
        assert_eq!(bus.read32(0x2000_0004), 0xDEAD_BEEF);
    }

    #[test]
    fn strh_reg_writes_halfword() {
        let (mut cpu, mut bus) = cpu_with_bus();
        cpu.regs.r[0] = 0xCAFE;
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 2;
        cpu.execute_one_with_bus(0x5288, &mut bus); // STRH r0, [r1, r2]
        assert_eq!(bus.read16(0x2000_0002), 0xCAFE);
    }

    #[test]
    fn strb_reg_writes_byte() {
        let (mut cpu, mut bus) = cpu_with_bus();
        cpu.regs.r[0] = 0xAB;
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 1;
        cpu.execute_one_with_bus(0x5488, &mut bus); // STRB r0, [r1, r2]
        assert_eq!(bus.read8(0x2000_0001), 0xAB);
    }

    #[test]
    fn ldrsb_reg_sign_extends() {
        let (mut cpu, mut bus) = cpu_with_bus();
        bus.write8(0x2000_0003, 0xFE);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 3;
        cpu.execute_one_with_bus(0x5688, &mut bus); // LDRSB r0, [r1, r2]
        assert_eq!(cpu.regs.r[0], 0xFFFF_FFFE);
    }

    #[test]
    fn ldr_reg_reads_word() {
        let (mut cpu, mut bus) = cpu_with_bus();
        bus.write32(0x2000_0010, 0x12345678);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 0x10;
        cpu.execute_one_with_bus(0x5888, &mut bus); // LDR r0, [r1, r2]
        assert_eq!(cpu.regs.r[0], 0x12345678);
    }

    #[test]
    fn ldrh_reg_zero_extends() {
        let (mut cpu, mut bus) = cpu_with_bus();
        bus.write16(0x2000_0006, 0xCAFE);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 6;
        cpu.execute_one_with_bus(0x5A88, &mut bus); // LDRH r0, [r1, r2]
        assert_eq!(cpu.regs.r[0], 0x0000_CAFE);
    }

    #[test]
    fn ldrb_reg_zero_extends() {
        let (mut cpu, mut bus) = cpu_with_bus();
        bus.write8(0x2000_0007, 0xAB);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 7;
        cpu.execute_one_with_bus(0x5C88, &mut bus); // LDRB r0, [r1, r2]
        assert_eq!(cpu.regs.r[0], 0x0000_00AB);
    }

    #[test]
    fn ldrsh_reg_sign_extends_halfword() {
        let (mut cpu, mut bus) = cpu_with_bus();
        bus.write16(0x2000_0008, 0xFF00);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.regs.r[2] = 8;
        cpu.execute_one_with_bus(0x5E88, &mut bus); // LDRSH r0, [r1, r2]
        assert_eq!(cpu.regs.r[0], 0xFFFF_FF00);
    }
}

mod load_store_imm {
    use super::*;

    #[test]
    fn str_imm_word() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x1234_5678;
        cpu.regs.r[1] = 0x2000_0000;
        cpu.execute_one_with_bus(0x6048, &mut bus); // STR r0, [r1, #4]
        assert_eq!(bus.read32(0x2000_0004), 0x1234_5678);
    }

    #[test]
    fn ldr_imm_word() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_0008, 0xDEAD_BEEF);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.execute_one_with_bus(0x6888, &mut bus); // LDR r0, [r1, #8]
        assert_eq!(cpu.regs.r[0], 0xDEAD_BEEF);
    }

    #[test]
    fn strb_imm() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0xCD;
        cpu.regs.r[1] = 0x2000_0000;
        cpu.execute_one_with_bus(0x7048, &mut bus); // STRB r0, [r1, #1]
        assert_eq!(bus.read8(0x2000_0001), 0xCD);
    }

    #[test]
    fn ldrb_imm() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write8(0x2000_0002, 0xEF);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.execute_one_with_bus(0x7888, &mut bus); // LDRB r0, [r1, #2]
        assert_eq!(cpu.regs.r[0], 0xEF);
    }

    #[test]
    fn strh_imm() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0xCAFE;
        cpu.regs.r[1] = 0x2000_0000;
        cpu.execute_one_with_bus(0x8048, &mut bus); // STRH r0, [r1, #2]
        assert_eq!(bus.read16(0x2000_0002), 0xCAFE);
    }

    #[test]
    fn ldrh_imm() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write16(0x2000_0004, 0xBEEF);
        cpu.regs.r[1] = 0x2000_0000;
        cpu.execute_one_with_bus(0x8888, &mut bus); // LDRH r0, [r1, #4]
        assert_eq!(cpu.regs.r[0], 0x0000_BEEF);
    }
}

// ---------------------------------------------------------------------------
// CP5 — SP-relative, ADR, ADD SP
// ---------------------------------------------------------------------------

mod sp_adr {
    use super::*;

    #[test]
    fn str_sp_relative() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0xCAFE_F00D;
        cpu.regs.r[13] = 0x2000_1000;
        cpu.execute_one_with_bus(0x9004, &mut bus); // STR r0, [SP, #16]
        assert_eq!(bus.read32(0x2000_1010), 0xCAFE_F00D);
    }

    #[test]
    fn ldr_sp_relative() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_1020, 0x1234_5678);
        cpu.regs.r[13] = 0x2000_1000;
        cpu.execute_one_with_bus(0x9808, &mut bus); // LDR r0, [SP, #32]
        assert_eq!(cpu.regs.r[0], 0x1234_5678);
    }

    #[test]
    fn adr_returns_pc_aligned_plus_offset() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x0); // instr at 0, read_pc=4, aligned=4
        cpu.execute_one(0xA001); // ADR r0, #4 (1*4)
        assert_eq!(cpu.regs.r[0], 0x0000_0008);
    }

    #[test]
    fn adr_aligns_pc_down_to_word() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x2); // instr at 2, read_pc=6, aligned=4
        cpu.execute_one(0xA000); // ADR r0, #0
        assert_eq!(cpu.regs.r[0], 0x0000_0004);
    }

    #[test]
    fn add_sp_imm8() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[13] = 0x2000_1000;
        cpu.execute_one(0xA802); // ADD r0, SP, #8
        assert_eq!(cpu.regs.r[0], 0x2000_1008);
    }

    #[test]
    fn add_sp_imm8_max() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[13] = 0x2000_0000;
        cpu.execute_one(0xA8FF); // ADD r0, SP, #0x3FC
        assert_eq!(cpu.regs.r[0], 0x2000_03FC);
    }
}

// ---------------------------------------------------------------------------
// CP6 — Misc (PUSH/POP/hints/SXT/UXT/REV/BKPT) and M0+-illegal encodings
// ---------------------------------------------------------------------------

mod misc_adjust_sp {
    use super::*;

    #[test]
    fn add_sp_sp_imm7() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[13] = 0x2000_1000;
        cpu.execute_one(0xB002); // ADD SP, SP, #8
        assert_eq!(cpu.regs.r[13], 0x2000_1008);
    }

    #[test]
    fn sub_sp_sp_imm7() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[13] = 0x2000_1000;
        cpu.execute_one(0xB082); // SUB SP, SP, #8
        assert_eq!(cpu.regs.r[13], 0x2000_0FF8);
    }

    #[test]
    fn add_sp_max_offset() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[13] = 0x0;
        cpu.execute_one(0xB07F); // ADD SP, SP, #0x1FC
        assert_eq!(cpu.regs.r[13], 0x1FC);
    }
}

mod misc_extend {
    use super::*;

    #[test]
    fn sxth_sign_extends_halfword() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0xFFFF_8000;
        cpu.execute_one(0xB208); // SXTH r0, r1
        assert_eq!(cpu.regs.r[0], 0xFFFF_8000);
    }

    #[test]
    fn sxtb_sign_extends_byte() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0xFFFF_FF80;
        cpu.execute_one(0xB248); // SXTB r0, r1
        assert_eq!(cpu.regs.r[0], 0xFFFF_FF80);
    }

    #[test]
    fn uxth_zero_extends() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0xFFFF_FFFF;
        cpu.execute_one(0xB288); // UXTH r0, r1
        assert_eq!(cpu.regs.r[0], 0x0000_FFFF);
    }

    #[test]
    fn uxtb_zero_extends() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0xFFFF_FFFF;
        cpu.execute_one(0xB2C8); // UXTB r0, r1
        assert_eq!(cpu.regs.r[0], 0x0000_00FF);
    }
}

mod misc_push_pop {
    use super::*;

    #[test]
    fn push_single_low_register() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0xDEAD_BEEF;
        cpu.regs.r[13] = 0x2000_1000;
        cpu.execute_one_with_bus(0xB401, &mut bus); // PUSH {r0}
        assert_eq!(cpu.regs.r[13], 0x2000_0FFC);
        assert_eq!(bus.read32(0x2000_0FFC), 0xDEAD_BEEF);
    }

    #[test]
    fn push_multiple_low_registers_and_lr() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0xAA;
        cpu.regs.r[1] = 0xBB;
        cpu.regs.set_lr(0x1234_5678);
        cpu.regs.r[13] = 0x2000_1000;
        cpu.execute_one_with_bus(0xB503, &mut bus); // PUSH {r0, r1, lr}
        assert_eq!(cpu.regs.r[13], 0x2000_0FF4);
        assert_eq!(bus.read32(0x2000_0FF4), 0xAA);
        assert_eq!(bus.read32(0x2000_0FF8), 0xBB);
        assert_eq!(bus.read32(0x2000_0FFC), 0x1234_5678);
    }

    #[test]
    fn pop_single_low_register() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_0FFC, 0xCAFE);
        cpu.regs.r[13] = 0x2000_0FFC;
        cpu.execute_one_with_bus(0xBC01, &mut bus); // POP {r0}
        assert_eq!(cpu.regs.r[0], 0xCAFE);
        assert_eq!(cpu.regs.r[13], 0x2000_1000);
    }

    #[test]
    fn pop_to_pc() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_0FFC, 0x2000_2001); // T-bit set
        cpu.regs.r[13] = 0x2000_0FFC;
        cpu.execute_one_with_bus(0xBD00, &mut bus); // POP {pc}
        assert_eq!(cpu.regs.r[15], 0x2000_2000);
        assert_eq!(cpu.regs.r[13], 0x2000_1000);
    }
}

mod misc_rev {
    use super::*;

    #[test]
    fn rev_swaps_bytes() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x1122_3344;
        cpu.execute_one(0xBA08); // REV r0, r1
        assert_eq!(cpu.regs.r[0], 0x4433_2211);
    }

    #[test]
    fn rev16_swaps_halfwords_internally() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x1122_3344;
        cpu.execute_one(0xBA48); // REV16 r0, r1
        assert_eq!(cpu.regs.r[0], 0x2211_4433);
    }

    #[test]
    fn revsh_sign_extends_swapped_low_halfword() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 0x1122_80FF;
        cpu.execute_one(0xBAC8); // REVSH r0, r1
        // 0x80FF bytes reversed = 0xFF80, sign-extended to 0xFFFF_FF80
        assert_eq!(cpu.regs.r[0], 0xFFFF_FF80);
    }

    #[test]
    fn rev_subop_0b10_is_undefined_on_m0plus() {
        let mut cpu = CortexM0Plus::new();
        // 0xBA88 decodes to Rd=r0, Rm=r1 on the M33 REV path. On M0+ the
        // sub-op is UNDEFINED — verify Rd is untouched (no clobber).
        cpu.regs.r[0] = 0xAABB_CCDD;
        cpu.regs.r[1] = 0x1234_5678;
        cpu.execute_one(0xBA88); // opcode >> 6 == 0b10 → UNDEFINED on ARMv6-M
        assert!(cpu.has_pending_fault());
        assert_eq!(cpu.regs.r[0], 0xAABB_CCDD);
    }
}

mod misc_hints_and_bkpt {
    use super::*;

    #[test]
    fn nop_is_supported() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xBF00); // NOP
        assert!(!cpu.has_pending_fault());
    }

    #[test]
    fn yield_is_supported() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xBF10); // YIELD
        assert!(!cpu.has_pending_fault());
    }

    #[test]
    fn sev_is_supported() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xBF40); // SEV
        assert!(!cpu.has_pending_fault());
    }

    #[test]
    fn it_encoding_is_undefined_on_m0plus() {
        let mut cpu = CortexM0Plus::new();
        // IT NE → mask != 0, so the M33 would decode as IT. On M0+ this is
        // UNDEFINED — verify the low GP registers and xPSR IT bits stay
        // untouched so we catch any accidental M33-style IT state set.
        cpu.regs.r[0] = 0xAABB_CCDD;
        cpu.regs.r[1] = 0x1122_3344;
        let xpsr_before = cpu.regs.xpsr;
        cpu.execute_one(0xBF18);
        assert!(cpu.has_pending_fault());
        assert_eq!(cpu.regs.r[0], 0xAABB_CCDD);
        assert_eq!(cpu.regs.r[1], 0x1122_3344);
        assert_eq!(cpu.regs.xpsr, xpsr_before);
    }

    #[test]
    fn bkpt_sets_pending_fault() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xBE00); // BKPT #0
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn cpsie_clears_primask() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.primask = 1;
        cpu.execute_one(0xB662); // CPSIE i — canonical encoding (I bit = bit 1)
        assert_eq!(cpu.regs.primask, 0);
    }

    #[test]
    fn cpsid_sets_primask() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xB672); // CPSID i — canonical encoding (I bit = bit 1)
        assert_eq!(cpu.regs.primask, 1);
    }

    #[test]
    fn cps_with_only_f_bit_is_noop_on_m0plus() {
        // ARMv6-M has no FAULTMASK; the F bit (bit 0) is UNPREDICTABLE on
        // M0+ and must not touch PRIMASK. 0xB661 sets only the F bit.
        let mut cpu = CortexM0Plus::new();
        cpu.regs.primask = 0x1;
        cpu.execute_one(0xB661);
        assert_eq!(cpu.regs.primask, 0x1);
    }
}

mod m0plus_undefined_encodings {
    use super::*;

    #[test]
    fn cbz_is_undefined_on_m0plus() {
        let mut cpu = CortexM0Plus::new();
        // M33: CBZ r0, #label. 0xB101 = CBZ with imm5=0, i=0, Rn=0
        cpu.execute_one(0xB100);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn cbnz_is_undefined_on_m0plus() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xB900);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn udf_cond_0b1110_is_undefined() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xDE00); // B, cond=0xE (AL) — UDF
        assert!(cpu.has_pending_fault());
    }
}

// ---------------------------------------------------------------------------
// CP7 — STM/LDM + branches + SVC
// ---------------------------------------------------------------------------

mod stm_ldm {
    use super::*;

    #[test]
    fn stm_writes_registers_with_writeback() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x11;
        cpu.regs.r[1] = 0x22;
        cpu.regs.r[2] = 0x33;
        cpu.regs.r[4] = 0x2000_0000;
        cpu.execute_one_with_bus(0xC407, &mut bus); // STMIA r4!, {r0, r1, r2}
        assert_eq!(bus.read32(0x2000_0000), 0x11);
        assert_eq!(bus.read32(0x2000_0004), 0x22);
        assert_eq!(bus.read32(0x2000_0008), 0x33);
        assert_eq!(cpu.regs.r[4], 0x2000_000C);
    }

    #[test]
    fn ldm_reads_registers_with_writeback() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_0000, 0xAA);
        bus.write32(0x2000_0004, 0xBB);
        cpu.regs.r[4] = 0x2000_0000;
        cpu.execute_one_with_bus(0xCC03, &mut bus); // LDMIA r4!, {r0, r1}
        assert_eq!(cpu.regs.r[0], 0xAA);
        assert_eq!(cpu.regs.r[1], 0xBB);
        assert_eq!(cpu.regs.r[4], 0x2000_0008);
    }

    #[test]
    fn ldm_no_writeback_when_rn_in_reglist() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_0000, 0x1234_5678);
        cpu.regs.r[0] = 0x2000_0000;
        cpu.execute_one_with_bus(0xC801, &mut bus); // LDMIA r0!, {r0}
        assert_eq!(cpu.regs.r[0], 0x1234_5678); // loaded value, NOT writeback
    }

    #[test]
    fn stm_single_register() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0xCAFE;
        cpu.regs.r[4] = 0x2000_0000;
        cpu.execute_one_with_bus(0xC401, &mut bus); // STMIA r4!, {r0}
        assert_eq!(bus.read32(0x2000_0000), 0xCAFE);
        assert_eq!(cpu.regs.r[4], 0x2000_0004);
    }
}

mod branches {
    use super::*;

    #[test]
    fn b_unconditional_positive_offset() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x1000); // instr at 0x1000, read_pc=0x1004
        cpu.execute_one(0xE002); // B +4 (imm11=2 → offset=4)
        // target = 0x1004 + 4 = 0x1008
        assert_eq!(cpu.regs.r[15], 0x1008);
    }

    #[test]
    fn b_unconditional_negative_offset() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x1000);
        // imm11 = 0x7FE → offset << 1 = 0xFFC; sign-extended from bit 11 = 0xFFFF_FFFC (−4)
        cpu.execute_one(0xE7FE); // B -4
        // target = 0x1004 + (−4) = 0x1000
        assert_eq!(cpu.regs.r[15], 0x1000);
    }

    #[test]
    fn b_cond_taken() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x1000);
        cpu.regs.set_flag_z(true);
        cpu.execute_one(0xD001); // BEQ +2 (imm8=1, offset=2)
        // target = 0x1004 + 2 = 0x1006
        assert_eq!(cpu.regs.r[15], 0x1006);
    }

    #[test]
    fn b_cond_not_taken() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x1000);
        cpu.regs.set_flag_z(false);
        cpu.execute_one(0xD001); // BEQ, Z=0 → not taken
        // helper still advances PC past the instruction
        assert_eq!(cpu.regs.r[15], 0x1002);
    }

    #[test]
    fn b_cond_backward_branch_ne_taken() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x1000);
        // BNE: imm8=0xFE → 0x1FC, sign-extended from bit 8 = 0xFFFF_FFFC (−4)
        cpu.execute_one(0xD1FE);
        // target = 0x1004 + (−4) = 0x1000
        assert_eq!(cpu.regs.r[15], 0x1000);
    }

    #[test]
    fn svc_sets_pending_fault_placeholder() {
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xDF00); // SVC #0
        // Phase 4.B wires in the real SVC handler; CP7 just verifies the
        // dispatch path reaches the SVC leg of thumb16_cond_branch_svc.
        assert!(cpu.has_pending_fault());
    }
}

// ---------------------------------------------------------------------------
// Wide-instruction detection
// ---------------------------------------------------------------------------

mod wide_detection {
    use super::*;

    #[test]
    fn non_wide_thumb16_dispatches_normally() {
        let mut cpu = CortexM0Plus::new();
        let cycles = cpu.execute_one(0x2042); // MOVS r0, #0x42
        assert!(cycles >= 1);
        assert_eq!(cpu.regs.r[0], 0x42);
    }

    #[test]
    fn decode_execute_flags_undefined_for_11101_prefix() {
        // 0xE8xx has M33 Thumb-32 prefix 0b11101 which doesn't exist on M0+.
        // We dispatch via execute_thumb16 since decode_execute's wide detector
        // only accepts 0b11110; an 0xE800..0xEFFF opcode reaches the thumb16
        // dispatch and should fall to undefined.
        let mut cpu = CortexM0Plus::new();
        cpu.execute_one(0xE800);
        assert!(cpu.has_pending_fault());
    }
}

// ---------------------------------------------------------------------------
// Cycle-count oracles — ensure `execute_one` returns the architected cycle
// cost for representative instructions. Phase 5 bus timing will recalibrate
// against real-silicon measurements, but the Phase 4.A ratios are fixed.
// ---------------------------------------------------------------------------

mod cycle_counts {
    use super::*;

    #[test]
    fn cycles_taken_branch_is_3() {
        // Taken conditional branch flushes the pipeline: 3 cycles.
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x1000);
        cpu.regs.set_flag_z(true);
        let cycles = cpu.execute_one(0xD001); // BEQ +2
        assert_eq!(cycles, 3);
    }

    #[test]
    fn cycles_simple_dp_is_1() {
        // ADDS Rd, Rn, Rm (register) costs 1 cycle.
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[1] = 1;
        cpu.regs.r[2] = 2;
        let cycles = cpu.execute_one(0x1888); // ADDS r0, r1, r2
        assert_eq!(cycles, 1);
        assert_eq!(cpu.regs.r[0], 3);
    }

    #[test]
    fn cycles_ldm_is_1_plus_count() {
        // LDMIA r0!, {r1, r2, r3} transfers 3 registers → 1 + 3 = 4 cycles.
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_0000, 0x11);
        bus.write32(0x2000_0004, 0x22);
        bus.write32(0x2000_0008, 0x33);
        cpu.regs.r[0] = 0x2000_0000;
        let cycles = cpu.execute_one_with_bus(0xC80E, &mut bus); // LDMIA r0!, {r1,r2,r3}
        assert_eq!(cycles, 4);
    }
}

// ---------------------------------------------------------------------------
// Phase 4.B — Thumb-32 subset (BL / MRS / MSR / DSB / DMB / ISB)
// ---------------------------------------------------------------------------

mod thumb32_bl {
    use super::*;

    /// BL with small positive offset:
    /// Assembled by arm-none-eabi-as for `bl target` where target is
    /// PC+4+4 at PC=0x1000 → target = 0x1008. Encoding = F000 F802.
    #[test]
    fn bl_sets_lr_to_next_instr_and_branches() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x1000);
        // BL +4: hw0=0xF000, hw1=0xF802 → imm25 = 0x000_0004
        let cycles = cpu.execute_one_wide(0xF000, 0xF802);
        assert_eq!(cpu.regs.lr(), 0x1004 | 1, "LR = return addr with T bit");
        assert_eq!(cpu.regs.pc(), 0x1008, "PC = target (T bit cleared)");
        assert_eq!(cycles, 4);
    }

    /// BL with negative offset: PC=0x1000, BL -4 → target = 0x1000.
    /// Encoding F7FF FFFE yields offset=-2 per the standard encoding.
    #[test]
    fn bl_negative_offset() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_pc(0x2000);
        // BL -2: hw0=0xF7FF hw1=0xFFFF → imm25 = 0x1FF_FFFE (sign-extended)
        // S=1, J1=J2=1 → I1=I2=1, imm10=0x3FF, imm11=0x7FF
        // imm25 = 0x1FFFFFE → sign-extended = 0xFFFFFFFE (i.e. -2)
        cpu.execute_one_wide(0xF7FF, 0xFFFF);
        // target = read_pc(=0x2004) + (-2) = 0x2002 → cleared to 0x2002.
        assert_eq!(cpu.regs.pc(), 0x2002);
        assert_eq!(cpu.regs.lr(), 0x2004 | 1);
    }
}

mod thumb32_mrs_msr {
    use super::*;

    /// MRS r0, PRIMASK — SYSm=16.
    /// Encoding: hw0=0xF3EF, hw1=0x8010 (Rd=0, SYSm=0x10).
    #[test]
    fn mrs_reads_primask() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.primask = 1;
        cpu.execute_one_wide(0xF3EF, 0x8010);
        assert_eq!(cpu.regs.r[0], 1);
    }

    /// MRS r1, xPSR (SYSm=0) — returns only NZCV flags.
    #[test]
    fn mrs_reads_xpsr_flags() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.set_flag_n(true);
        cpu.regs.set_flag_c(true);
        // hw1 = 0x8100 (Rd=1, SYSm=0)
        cpu.execute_one_wide(0xF3EF, 0x8100);
        // N and C bits set in r1.
        assert_eq!(cpu.regs.r[1] & 0xF000_0000, 0xA000_0000);
    }

    /// MSR PRIMASK, r2 — writes bit 0 of r2 into PRIMASK.
    #[test]
    fn msr_writes_primask() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[2] = 0xFFFF_FFFF;
        // hw0=0xF382, hw1=0x8810 (Rn=2, mask=1000, SYSm=0x10)
        cpu.execute_one_wide(0xF382, 0x8810);
        assert_eq!(cpu.regs.primask, 1);
    }

    /// MSR CONTROL, r3 — writes bit 1 / bit 0 of r3 into CONTROL.
    #[test]
    fn msr_writes_control_thread_mode() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_0100;
        cpu.regs.psp = 0x2000_0200;
        cpu.regs.r[3] = 0x2; // SPSEL=1
        // hw0=0xF383, hw1=0x8814 (Rn=3, mask=1000, SYSm=0x14)
        cpu.execute_one_wide(0xF383, 0x8814);
        assert_eq!(cpu.regs.control, 0x2);
        // SP now tracks PSP.
        assert_eq!(cpu.regs.sp(), 0x2000_0200);
    }

    /// MSR with reserved SYSm (e.g. SYSm=4) raises HardFault.
    /// ARMv6-M ARM §B5.2.3 — anything outside {0, 3, 5, 8, 9, 16, 20}
    /// is reserved on v6-M and must trap.
    #[test]
    fn msr_reserved_sysm_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.r[0] = 0xDEAD_BEEF;
        // hw0=0xF380 (Rn=0), hw1=0x8804 (mask=1000, SYSm=4 — reserved)
        cpu.execute_one_wide(0xF380, 0x8804);
        assert!(cpu.has_pending_fault());
    }

    /// MRS with reserved SYSm (e.g. SYSm=15) raises HardFault.
    #[test]
    fn mrs_reserved_sysm_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        // hw0=0xF3EF, hw1=0x800F (Rd=0, SYSm=15 — reserved)
        cpu.execute_one_wide(0xF3EF, 0x800F);
        assert!(cpu.has_pending_fault());
    }
}

mod thumb32_barriers {
    use super::*;

    /// DSB #SY — hw0=0xF3BF, hw1=0x8F4F.
    #[test]
    fn dsb_noops_cleanly() {
        let mut cpu = CortexM0Plus::new();
        let cycles = cpu.execute_one_wide(0xF3BF, 0x8F4F);
        assert_eq!(cycles, 1);
        assert!(!cpu.has_pending_fault());
    }

    /// DMB #SY — hw0=0xF3BF, hw1=0x8F5F.
    #[test]
    fn dmb_noops_cleanly() {
        let mut cpu = CortexM0Plus::new();
        let cycles = cpu.execute_one_wide(0xF3BF, 0x8F5F);
        assert_eq!(cycles, 1);
        assert!(!cpu.has_pending_fault());
    }

    /// ISB #SY — hw0=0xF3BF, hw1=0x8F6F.
    #[test]
    fn isb_noops_cleanly() {
        let mut cpu = CortexM0Plus::new();
        let cycles = cpu.execute_one_wide(0xF3BF, 0x8F6F);
        assert_eq!(cycles, 1);
        assert!(!cpu.has_pending_fault());
    }
}

// ---------------------------------------------------------------------------
// Phase 4.B — Exception model
// ---------------------------------------------------------------------------

/// Helper: lay out a minimal SRAM-based vector table at address 0x2000_0000
/// and point VTOR at it. Entry N (for N >= 1) → handler address 0x2000_1000 +
/// N*32. Returns `(bus, handler_addrs)` where `handler_addrs[N]` is the
/// handler PC we mapped for exception N.
fn make_test_bus_with_vector_table() -> (Bus, [u32; 16]) {
    let mut bus = Bus::default();
    let vtor: u32 = 0x2000_0000;
    let mut handlers = [0u32; 16];
    for i in 0..16 {
        let handler = 0x2000_1000 + (i as u32) * 32;
        bus.write32(vtor + (i as u32) * 4, handler | 1); // Thumb bit set
        handlers[i] = handler;
    }
    bus.ppb[0].vtor = vtor;
    (bus, handlers)
}

mod exceptions {
    use super::*;

    #[test]
    fn svc_delivers_exception_11() {
        let (mut bus, handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        // Place SVC #0 at 0x1000 so we can observe the return address.
        let prog = 0x2000_4000u32;
        bus.write16(prog, 0xDF00);
        cpu.regs.set_pc(prog);
        let cycles = cpu.step(&mut bus);
        // IPSR should now be 11, PC at SVC handler, SP decremented by 32.
        assert_eq!(cpu.regs.ipsr(), 11);
        assert_eq!(cpu.regs.pc(), handlers[11]);
        assert_eq!(cpu.regs.sp(), 0x2000_8000 - 32);
        // LR carries the EXC_RETURN magic for Thread+MSP.
        assert_eq!(cpu.regs.lr(), 0xFFFF_FFF9);
        assert!(cycles >= 16);
    }

    #[test]
    fn bkpt_delivers_hardfault() {
        let (mut bus, handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        let prog = 0x2000_4000u32;
        bus.write16(prog, 0xBE00); // BKPT #0
        cpu.regs.set_pc(prog);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 3);
        assert_eq!(cpu.regs.pc(), handlers[3]);
    }

    #[test]
    fn undefined_encoding_delivers_hardfault() {
        let (mut bus, handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        // Thumb-32 prefix with body that no misc-control encoding matches.
        let prog = 0x2000_4000u32;
        bus.write16(prog, 0xF000);
        bus.write16(prog + 2, 0x0000);
        cpu.regs.set_pc(prog);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 3);
        assert_eq!(cpu.regs.pc(), handlers[3]);
    }

    #[test]
    fn nmi_enters_handler_2() {
        let (mut bus, handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        cpu.test_enter_exception(2, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 2);
        assert_eq!(cpu.regs.pc(), handlers[2]);
    }

    #[test]
    fn exc_return_thread_msp_restores_state() {
        let (mut bus, _handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        // Pre-load caller state we can verify after unwind.
        for i in 0..4 {
            cpu.regs.r[i] = 0x1000 + i as u32;
        }
        cpu.regs.r[12] = 0xC12;
        cpu.regs.set_lr(0xBADC0DE1); // pre-entry LR (caller's return)
        cpu.regs.set_pc(0x1000);
        cpu.test_enter_exception(11, &mut bus);
        // Handler overwrites r0 to prove unwind reverses it.
        cpu.regs.r[0] = 0xFFFF_FFFF;
        // EXC_RETURN to thread + MSP.
        cpu.test_exit_exception(0xFFFF_FFF9, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 0, "Back in thread mode");
        assert_eq!(cpu.regs.r[0], 0x1000);
        assert_eq!(cpu.regs.r[12], 0xC12);
        assert_eq!(cpu.regs.pc(), 0x1000);
        assert_eq!(cpu.regs.sp(), 0x2000_8000);
    }

    #[test]
    fn exc_return_thread_psp_restores_psp_and_sp_selection() {
        let (mut bus, _handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.psp = 0x2000_4000;
        cpu.regs.control = 0x2; // SPSEL=1 (thread PSP)
        cpu.regs.set_sp(0x2000_4000);
        cpu.regs.set_pc(0x1100);
        cpu.test_enter_exception(11, &mut bus);
        // Entry pushed to PSP; EXC_RETURN magic should be 0xFFFF_FFFD.
        assert_eq!(cpu.regs.lr(), 0xFFFF_FFFD);
        cpu.test_exit_exception(0xFFFF_FFFD, &mut bus);
        assert_eq!(cpu.regs.control & 0x2, 0x2, "Back to PSP in thread mode");
        assert_eq!(cpu.regs.sp(), 0x2000_4000);
    }

    #[test]
    fn exc_return_handler_requires_active_exception() {
        let (mut bus, _handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        // Nested scenario: first enter #11 (SVC), then enter #2 (NMI) so
        // both are "active". LR after NMI entry is 0xFFFF_FFF1
        // (Handler, MSP). EXC_RETURN 0xF1 must be valid since #11 is
        // still active.
        cpu.regs.set_pc(0x1000);
        cpu.test_enter_exception(11, &mut bus);
        let lr_after_nmi = cpu.regs.lr(); // 0xFFFF_FFF1 for handler→handler
        cpu.test_enter_exception(2, &mut bus);
        assert_eq!(cpu.regs.lr(), 0xFFFF_FFF1);
        let _ = lr_after_nmi;
        // Return from NMI → should land back in SVC handler.
        cpu.test_exit_exception(0xFFFF_FFF1, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 11);
    }

    #[test]
    fn exc_return_invalid_low_nibble_raises_hardfault() {
        let (mut bus, _handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        cpu.test_enter_exception(11, &mut bus);
        // Corrupt LR value — bits[3:0] = 0x2 is not a legal EXC_RETURN.
        cpu.test_exit_exception(0xFFFF_FFF2, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn bx_to_exc_return_unwinds() {
        // Set up entry → handler writes BX LR → unwind observed.
        let (mut bus, _handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        cpu.regs.set_pc(0x1000);
        cpu.test_enter_exception(11, &mut bus);
        // BX LR with LR = EXC_RETURN. Encoding: 0x4770.
        bus.write16(cpu.regs.pc(), 0x4770);
        // Step through the BX.
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 0);
    }

    #[test]
    fn handler_sp_mutations_sync_to_banked_on_exit() {
        // Regression test for banked-SP staleness across exception entry/exit.
        // SUB SP / ADD SP / PUSH / POP write r[13] directly and never touch
        // the banked msp. If enter_exception / exit_exception read msp
        // without first syncing from r[13], mismatched SP manipulation
        // in a handler ends up popping from the wrong address.
        let (mut bus, _handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        let initial_sp = 0x2000_3F00u32;
        // Only touch r[13] — do NOT explicitly set regs.msp. A correct
        // enter_exception must sync r[13] into msp before reading it.
        cpu.regs.set_sp(initial_sp);
        cpu.regs.set_pc(0x1000);
        // Place the SVC handler in SRAM so we can step real instructions.
        let handler = 0x2000_5000u32;
        bus.write32(0x2000_0000 + 11 * 4, handler | 1);
        // Handler body: SUB SP,#8 ; ADD SP,#8 ; BX LR
        bus.write16(handler, 0xB082);        // SUB SP, #8
        bus.write16(handler + 2, 0xB002);    // ADD SP, #8
        bus.write16(handler + 4, 0x4770);    // BX LR
        // Deliver SVC via the real fault path so enter_exception is driven
        // by the same code path that normal execution uses.
        cpu.pending_fault = Some(crate::core::Fault::Svc);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 11);
        assert_eq!(cpu.regs.pc(), handler);
        assert_eq!(cpu.regs.sp(), initial_sp - 32);
        // Step through SUB SP, #8 — r[13] diverges from msp (msp stays
        // at initial_sp - 32).
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.sp(), initial_sp - 40);
        // Step through ADD SP, #8 — r[13] back to the post-entry value.
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.sp(), initial_sp - 32);
        // Step through BX LR with LR = EXC_RETURN — triggers exit_exception.
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 0, "Returned to thread mode");
        // Unwind deallocated 32 bytes from the stack — net SP back to start.
        assert_eq!(
            cpu.regs.sp(),
            initial_sp,
            "SP restored to pre-fault value"
        );
    }

    #[test]
    fn nonhardfault_with_t0_vector_escalates_to_hardfault() {
        // ARMv6-M ARM §B1.5 — a vector entry with the Thumb bit clear is
        // an entry-path fault. For HardFault itself, this is lockup; for
        // anything else, escalate to HardFault. The first step executes
        // the SVC and stages a HardFault; the second step delivers it.
        let (mut bus, handlers) = make_test_bus_with_vector_table();
        // Corrupt SVCall vector — strip the T bit to simulate a malformed
        // vector table entry. HardFault vector stays well-formed so the
        // escalation can actually land.
        let bad_svc = 0x2000_0200u32; // no T bit
        bus.write32(0x2000_0000 + 11 * 4, bad_svc);
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        let prog = 0x2000_4000u32;
        bus.write16(prog, 0xDF00); // SVC #0
        cpu.regs.set_pc(prog);
        // First step: SVC sets pending_fault=Svc, deliver_fault tries to
        // enter vector #11, finds T=0, escalates by setting
        // pending_fault=HardFault. No handler reached yet.
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 0, "Did not yet enter any handler");
        assert!(cpu.has_pending_fault(), "HardFault staged");
        // Stage the HardFault without fetching from a bogus PC — the step
        // loop's decode_execute would otherwise try to fetch from whatever
        // instruction follows the SVC.
        let fault = cpu.pending_fault.take().unwrap();
        cpu.deliver_fault(fault, &mut bus);
        assert_eq!(cpu.regs.ipsr(), 3);
        assert_eq!(cpu.regs.pc(), handlers[3]);
    }

    #[test]
    fn exception_entry_pads_when_sp_is_4_aligned_not_8() {
        // ARMv6-M ARM §B1.5.6 — exception entry forces 8-byte alignment
        // by pre-decrementing SP by 4 when the pre-entry SP is 4-aligned
        // but not 8-aligned. The padding bit (bit 9 of stacked xPSR)
        // records that fact so exit_exception can undo it.
        let (mut bus, _handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        let initial_sp = 0x2000_3FF4u32; // 4-aligned, not 8-aligned
        cpu.regs.msp = initial_sp;
        cpu.regs.set_sp(initial_sp);
        cpu.regs.set_pc(0x1000);
        cpu.test_enter_exception(11, &mut bus);
        // SP = initial_sp - 4 (pad) - 32 (frame) = initial_sp - 36.
        let frame_sp = initial_sp - 36;
        assert_eq!(cpu.regs.sp(), frame_sp);
        // Stacked xPSR lives at frame_sp + 28 — bit 9 must be set.
        let stacked_xpsr = bus.read32(frame_sp + 28);
        assert_ne!(
            stacked_xpsr & (1 << 9),
            0,
            "STKALIGN padding bit recorded in stacked xPSR"
        );
        // Unwind restores the pre-entry SP, including the pad.
        cpu.test_exit_exception(0xFFFF_FFF9, &mut bus);
        assert_eq!(cpu.regs.sp(), initial_sp);
    }

    /// An unmapped load sets `bus.bus_fault`; `step()` must observe the
    /// flag, stage a HardFault, and deliver it via vector #3 (the single
    /// synchronous-fault vector on ARMv6-M).
    #[test]
    fn unmapped_load_escalates_to_hardfault() {
        let (mut bus, handlers) = make_test_bus_with_vector_table();
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        // Program: LDR r1, [r0, #0] at 0x2000_4000 with r0 = 0x7000_0000
        // (unmapped). Width-4 load through read32 sets bus_fault.
        let prog = 0x2000_4000u32;
        bus.write16(prog, 0x6801); // LDR r1, [r0]
        cpu.regs.r[0] = 0x7000_0000;
        cpu.regs.set_pc(prog);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 3, "HardFault taken");
        assert_eq!(cpu.regs.pc(), handlers[3]);
        assert!(
            !bus.bus_fault(),
            "step() cleared the sticky bus_fault flag"
        );
    }
}

// ---------------------------------------------------------------------------
// Phase 4.B — Unaligned access fault
// ---------------------------------------------------------------------------

mod unaligned {
    use super::*;

    #[test]
    fn ldr_word_unaligned_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x2000_0001; // misaligned word
        // LDR r1, [r0, #0] — encoding 0x6801
        cpu.execute_one_with_bus(0x6801, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn str_word_unaligned_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x2000_0002;
        cpu.regs.r[1] = 0xDEAD_BEEF;
        // STR r1, [r0, #0] — encoding 0x6001
        cpu.execute_one_with_bus(0x6001, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn ldrh_unaligned_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x2000_0001;
        // LDRH r1, [r0, #0] — encoding 0x8801
        cpu.execute_one_with_bus(0x8801, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn ldm_unaligned_base_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x2000_0001; // misaligned LDM base
        // LDMIA r0!, {r1, r2} — 0xC806
        cpu.execute_one_with_bus(0xC806, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn ldrb_byte_any_alignment_ok() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write8(0x2000_0003, 0x42);
        cpu.regs.r[0] = 0x2000_0003; // byte access to odd address — fine
        // LDRB r1, [r0, #0] — encoding 0x7801
        cpu.execute_one_with_bus(0x7801, &mut bus);
        assert!(!cpu.has_pending_fault());
        assert_eq!(cpu.regs.r[1], 0x42);
    }
}

// ---------------------------------------------------------------------------
// Phase 4.B — T=0 branch target HardFault
// ---------------------------------------------------------------------------

mod t_bit_fault {
    use super::*;

    #[test]
    fn bx_with_t0_target_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[1] = 0x2000; // even address → T bit clear
        // BX r1 — encoding 0x4708
        cpu.execute_one_with_bus(0x4708, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn blx_with_t0_target_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[2] = 0x4000;
        // BLX r2 — encoding 0x4790
        cpu.execute_one_with_bus(0x4790, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn pop_pc_with_t0_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        bus.write32(0x2000_0000, 0x1000); // even popped PC
        cpu.regs.set_sp(0x2000_0000);
        // POP {pc} — 0xBD00
        cpu.execute_one_with_bus(0xBD00, &mut bus);
        assert!(cpu.has_pending_fault());
    }

    #[test]
    fn mov_pc_with_t0_raises_fault() {
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x1000; // even
        // MOV PC, r0 — encoding 0x4687 (op=10, Rm=0, D=1, rd=7)
        //   bits: 010001 10 D(0) Rm(0000) Rd(111) → 0x46 << 8 | 0x87
        cpu.execute_one_with_bus(0x4687, &mut bus);
        assert!(cpu.has_pending_fault());
    }
}

// ---------------------------------------------------------------------------
// Phase 4.B — Emulator::step integration smoke tests
// ---------------------------------------------------------------------------

mod emulator_step {
    use crate::{Config, EmulatorBuilder};

    #[test]
    fn step_executes_movs_sequence() {
        // Build a tiny program in SRAM and set PC there. Five MOVS instructions
        // writing constants to r0..r4.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let program_base: u32 = 0x2000_1000;
        let instrs: [u16; 5] = [
            0x2001, // MOVS r0, #1
            0x2102, // MOVS r1, #2
            0x2203, // MOVS r2, #3
            0x2304, // MOVS r3, #4
            0x2405, // MOVS r4, #5
        ];
        for (i, w) in instrs.iter().enumerate() {
            emu.bus.write16(program_base + (i as u32) * 2, *w);
        }
        emu.cores[0].regs.set_pc(program_base);
        for _ in 0..instrs.len() {
            emu.step();
        }
        assert_eq!(emu.cores[0].regs.r[0], 1);
        assert_eq!(emu.cores[0].regs.r[1], 2);
        assert_eq!(emu.cores[0].regs.r[2], 3);
        assert_eq!(emu.cores[0].regs.r[3], 4);
        assert_eq!(emu.cores[0].regs.r[4], 5);
    }

    #[test]
    fn step_handles_svc_and_return() {
        // Program: SVC #0 at 0x1000 followed by a NOP. Handler at 0x2000
        // is a single BX LR. Verify we reach the handler, then return.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let vtor = 0x2000_0000u32;
        let handler = 0x2000_1000u32;
        let stack_top = 0x2000_8000u32;
        // Vector table: entry 11 → handler|1
        for i in 0..16 {
            emu.bus.write32(vtor + (i as u32) * 4, 0);
        }
        emu.bus.write32(vtor + 11 * 4, handler | 1);
        emu.bus.ppb[0].vtor = vtor;
        // Caller program at 0x2000_4000
        let prog = 0x2000_4000u32;
        emu.bus.write16(prog, 0xDF00); // SVC #0
        emu.bus.write16(prog + 2, 0xBF00); // NOP (resume point)
        // Handler: BX LR
        emu.bus.write16(handler, 0x4770);
        // Init core
        emu.cores[0].regs.msp = stack_top;
        emu.cores[0].regs.set_sp(stack_top);
        emu.cores[0].regs.set_pc(prog);
        // Step 1: executes SVC → enters handler.
        emu.step();
        assert_eq!(emu.cores[0].regs.ipsr(), 11);
        assert_eq!(emu.cores[0].regs.pc(), handler);
        // Step 2: executes BX LR → unwinds.
        emu.step();
        assert_eq!(emu.cores[0].regs.ipsr(), 0);
        assert_eq!(emu.cores[0].regs.pc(), prog + 2);
    }

    #[test]
    fn step_hardfault_on_undefined_then_unwinds() {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let vtor = 0x2000_0000u32;
        let handler = 0x2000_1000u32;
        let stack_top = 0x2000_8000u32;
        for i in 0..16 {
            emu.bus.write32(vtor + (i as u32) * 4, 0);
        }
        emu.bus.write32(vtor + 3 * 4, handler | 1);
        emu.bus.ppb[0].vtor = vtor;
        // Program: undefined encoding (BKPT, which raises HardFault on M0+
        // without a debugger) at 0x2000_4000.
        let prog = 0x2000_4000u32;
        emu.bus.write16(prog, 0xBE00); // BKPT #0 → HardFault
        // Handler at 0x2000_1000: BX LR.
        emu.bus.write16(handler, 0x4770);
        emu.cores[0].regs.msp = stack_top;
        emu.cores[0].regs.set_sp(stack_top);
        emu.cores[0].regs.set_pc(prog);
        emu.step();
        assert_eq!(emu.cores[0].regs.ipsr(), 3);
        emu.step();
        assert_eq!(emu.cores[0].regs.ipsr(), 0);
    }

    #[test]
    fn run_advances_pc_over_nops() {
        // Emulator::run loops calling step until the cycle budget is met.
        // Lay down 10 NOPs and verify both PC and the cycle count advanced
        // as expected.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let prog = 0x2000_1000u32;
        for i in 0..10 {
            emu.bus.write16(prog + (i as u32) * 2, 0xBF00); // NOP
        }
        emu.cores[0].regs.set_pc(prog);
        let start_cycles = emu.cycles();
        let executed = emu.run(10);
        assert!(
            executed >= 10,
            "run() returned at least the requested cycle count"
        );
        // Each NOP takes 1 cycle on M0+, so ~10 steps to meet a 10-cycle
        // budget. PC should have advanced ≥20 bytes (10 × 2-byte NOPs).
        assert_eq!(emu.cores[0].regs.pc(), prog + 20);
        assert_eq!(emu.cycles() - start_cycles, executed);
    }

    #[test]
    fn step_primask_escalates_svc_to_hardfault() {
        // ARMv6-M ARM §B1.5.8: executing SVC while PRIMASK=1 cannot preempt
        // — SVCall priority (0) is not higher than execution priority (0
        // with PRIMASK set). The architectural response is to escalate
        // to HardFault rather than silently deliver the SVCall.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let vtor = 0x2000_0000u32;
        let svc_handler = 0x2000_1000u32;
        let hf_handler = 0x2000_2000u32;
        let stack_top = 0x2000_8000u32;
        for i in 0..16 {
            emu.bus.write32(vtor + (i as u32) * 4, 0);
        }
        emu.bus.write32(vtor + 3 * 4, hf_handler | 1);
        emu.bus.write32(vtor + 11 * 4, svc_handler | 1);
        emu.bus.ppb[0].vtor = vtor;
        let prog = 0x2000_4000u32;
        emu.bus.write16(prog, 0xDF00); // SVC #0
        emu.cores[0].regs.msp = stack_top;
        emu.cores[0].regs.set_sp(stack_top);
        emu.cores[0].regs.primask = 1;
        emu.cores[0].regs.set_pc(prog);
        emu.step();
        // SVC escalated to HardFault — land at vector #3, not #11.
        assert_eq!(emu.cores[0].regs.ipsr(), 3);
        assert_eq!(emu.cores[0].regs.pc(), hf_handler);
    }
}

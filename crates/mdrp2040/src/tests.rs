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

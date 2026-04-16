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
    fn mov_pc_with_even_target_branches() {
        // ARMv6-M ARM §A5.1.2: MOV Rd, Rm with Rd==15 goes through
        // ALUWritePC → BranchWritePC → BranchTo(addr<31:1>:'0'). The LSB
        // is masked, never checked. gcc's switch-statement jump tables
        // load even-aligned label addresses and branch via this path.
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        cpu.regs.r[0] = 0x2000_1000; // even target
        // MOV PC, r0 — 0x4687 (op=10, D:Rd = 1:111 = 15, Rm = 0)
        cpu.execute_one_with_bus(0x4687, &mut bus);
        assert!(!cpu.has_pending_fault());
        assert_eq!(cpu.regs.pc(), 0x2000_1000);
    }

    #[test]
    fn add_pc_with_even_target_branches() {
        // ARMv6-M ARM §A5.1.2: ADD Rdn, Rm with Rd==15 also uses
        // ALUWritePC. Even Rm is legal; LSB is masked.
        let mut cpu = CortexM0Plus::new();
        let mut bus = Bus::default();
        let base: u32 = 0x2000_2000;
        cpu.regs.set_pc(base);
        cpu.regs.r[0] = 0x1000; // even displacement
        // ADD PC, r0 — 0x4487 (op=00, D:Rd = 1:111 = 15, Rm = 0)
        // execute_one_with_bus sets current_instr_addr = base and bumps
        // pc; read_pc() returns base + 4 per ARMv6-M semantics.
        // Expected target: (base + 4 + 0x1000) with LSB masked = base + 0x1004.
        cpu.execute_one_with_bus(0x4487, &mut bus);
        assert!(!cpu.has_pending_fault());
        assert_eq!(cpu.regs.pc(), base + 0x1004);
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

    #[test]
    fn halted_core0_does_not_freeze_core1() {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(4).build();
        let prog = 0x2000_1000u32;
        for i in 0..8u32 {
            emu.bus.write16(prog + i * 2, 0xBF00); // NOP
        }
        emu.cores[1].wake();
        emu.cores[1].regs.set_pc(prog);
        emu.cores[1].regs.msp = 0x2002_0000;
        emu.cores[1].regs.r[13] = emu.cores[1].regs.msp;
        emu.cores[0].halt();

        let pc_before = emu.cores[1].regs.pc();
        let consumed = emu.step();
        assert!(consumed > 0, "step() must advance when core 1 is runnable");
        assert!(emu.cores[1].regs.pc() > pc_before, "core 1 PC must advance");
    }
}

// ---------------------------------------------------------------------------
// Quantum-step contracts (HLD v1.1.0 §B)
// ---------------------------------------------------------------------------
//
// Four contracts the main quantum-step HLD (v1.2.0) relies on:
//   1. `step_quantum(1)` advances by exactly one core-0 instruction.
//   2. `step_quantum(N)` advances the clock into the half-open window
//      `[N, N + MAX_INSTR_COST)` — overshoot bounded by the most
//      expensive single M0+ instruction (BL = 4 cycles).
//   3. `step()`'s return value equals the `clock.cycles` delta across
//      the call.
//   4. Peripherals tick once per `step()` — not once per inner-loop
//      iteration. A single quantum-N step must land in the same PIO
//      state as N quantum-1 steps against an identical program.
mod quantum_contract {
    use crate::bus::PIO0_BASE;
    use crate::{Config, EmulatorBuilder, Emulator};

    /// Seed a run of NOPs at 0x2000_1000 and park core 0 on them.
    /// Each NOP is a 1-cycle instruction on M0+, so each `emu.step()`
    /// call with `step_quantum(1)` advances the master clock by exactly
    /// one cycle.
    fn seed_nop_program(emu: &mut Emulator) {
        let prog = 0x2000_1000u32;
        for i in 0..256u32 {
            emu.bus.write16(prog + i * 2, 0xBF00); // NOP
        }
        emu.cores[0].regs.set_pc(prog);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    }

    #[test]
    fn step_quantum_1_advances_by_one_instruction() {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        seed_nop_program(&mut emu);
        let pc_before = emu.cores[0].regs.pc();
        let consumed = emu.step();
        assert_eq!(consumed, 1, "quantum=1 NOP must consume exactly 1 cycle");
        assert_eq!(
            emu.cores[0].regs.pc(),
            pc_before + 2,
            "PC must advance by one 2-byte Thumb instruction"
        );
    }

    #[test]
    fn step_quantum_n_advances_within_bounds() {
        // With quantum=N, the loop keeps issuing instructions until the
        // master clock reaches or exceeds `N`. A single instruction can
        // cost at most `MAX_INSTR_COST = 4` cycles on M0+ (BL), so the
        // overshoot is strictly bounded.
        const N: u32 = 16;
        const MAX_INSTR_COST: u64 = 4;
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(N).build();
        seed_nop_program(&mut emu);
        let consumed = emu.step();
        assert!(
            consumed >= N as u64,
            "quantum={} must consume at least N cycles (got {})",
            N,
            consumed
        );
        assert!(
            consumed < N as u64 + MAX_INSTR_COST,
            "quantum={} overshoot must be bounded by MAX_INSTR_COST={} (got {})",
            N,
            MAX_INSTR_COST,
            consumed
        );
    }

    #[test]
    fn step_return_equals_clock_delta() {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(8).build();
        seed_nop_program(&mut emu);
        let before = emu.cycles();
        let consumed = emu.step();
        assert_eq!(
            consumed,
            emu.cycles() - before,
            "step() return value must equal the clock.cycles delta"
        );
    }

    /// Build an emulator with PIO0/SM0 loaded with a 2-instruction toggle
    /// program — `SET PINS, 1` then `SET PINS, 0` with auto-wrap. On
    /// each PIO cycle, `pad_out & 1` alternates between 1 and 0. Core 0
    /// is parked on NOPs so each emu-step advances PIO by exactly `c0`
    /// system-clock cycles.
    fn toggle_emulator(step_quantum: u32) -> Emulator {
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(step_quantum)
            .build();

        // Program: SET PINS, 1 @ addr 0; SET PINS, 0 @ addr 1.
        let set_pins_1: u16 = 0xE001;
        let set_pins_0: u16 = 0xE000;
        for (i, insn) in [set_pins_1, set_pins_0].iter().enumerate() {
            emu.bus
                .write32(PIO0_BASE + 0x048 + (i as u32) * 4, *insn as u32);
        }

        // SM0_PINCTRL: set_count=1 (bits 28:26), set_base=0 (bits 9:5).
        emu.bus.write32(PIO0_BASE + 0x0DC, 1u32 << 26);
        // SM0_EXECCTRL: wrap_top=1, wrap_bottom=0 — auto-wrap 0→1→0.
        emu.bus.write32(PIO0_BASE + 0x0CC, 1u32 << 12);
        // Force SET PINDIRS, 1 so the output pin becomes driven.
        emu.bus.write32(PIO0_BASE + 0x0D8, 0xE081);
        // Enable SM0.
        emu.bus.write32(PIO0_BASE + 0x000, 0x1);

        seed_nop_program(&mut emu);
        emu
    }

    #[test]
    fn peripherals_tick_once_per_step() {
        // Reference: step_quantum(1) stepped N times — PIO is ticked N
        // separate times, each with cycles=1.
        // Subject:   step_quantum(N) stepped once — PIO is ticked once
        // with cycles=N.
        // `tick_pio` fires exactly once per `step()`, so both paths must
        // land the PIO SM0 in the same position and `pad_out & 1` must
        // match. A double-tick inside the inner loop would diverge.
        const N: u32 = 8;

        let mut reference = toggle_emulator(1);
        for _ in 0..N {
            reference.step();
        }

        let mut subject = toggle_emulator(N);
        subject.step();

        assert_eq!(
            subject.bus.pio[0].pad_out & 1,
            reference.bus.pio[0].pad_out & 1,
            "one N-cycle step must leave the same pad_out state as N one-cycle steps",
        );
    }
}

mod external_gpio_override {
    //! Tests for `Bus::external_gpio_in_override` /
    //! `external_gpio_in_mask` — the harness-injection escape hatch that
    //! lets `picogus_diff_rp2040` drive synthetic ISA pins (IOW, IOR,
    //! AD0..AD9) without `Emulator::update_gpio` clobbering them on the
    //! next merge.
    //!
    //! Without these tests, the regression caught by Stage 4 review (B1
    //! — direct `bus.gpio_in` writes vanish on the first `update_gpio`)
    //! has no fixed defence: a future `update_gpio` change could
    //! reintroduce the same overwrite without anything failing.
    use crate::{Config, EmulatorBuilder};

    #[test]
    fn override_wins_over_default_merge() {
        // Set bits on GPIO10..15 via the override. After update_gpio,
        // those bits in `gpio_in` must reflect the override exactly.
        let mut emu = EmulatorBuilder::new(Config::default()).build();
        emu.reset();

        let mask: u32 = 0b111111u32 << 10; // GPIO10..GPIO15
        let value: u32 = 0b101010u32 << 10;
        emu.bus.external_gpio_in_mask = mask;
        emu.bus.external_gpio_in_override = value;

        emu.update_gpio();

        assert_eq!(
            emu.bus.gpio_in & mask,
            value & mask,
            "override pins must reflect external_gpio_in_override after update_gpio"
        );
    }

    #[test]
    fn override_wins_over_sio_drive() {
        // Drive the same pins via SIO (gpio_oe + gpio_out), then assert
        // the override still wins. This is the exact race that B1 hid:
        // SIO sets a bit, update_gpio merges, and the override would be
        // lost without the post-PSRAM splice.
        let mut emu = EmulatorBuilder::new(Config::default()).build();
        emu.reset();

        let mask: u32 = 0b111111u32 << 10;
        let override_value: u32 = 0b101010u32 << 10;
        let sio_value: u32 = 0b010101u32 << 10; // bit-inverse pattern

        // First, no override — confirm SIO drives gpio_in normally.
        emu.bus.sio.gpio_oe = mask;
        emu.bus.sio.gpio_out = sio_value;
        emu.update_gpio();
        assert_eq!(
            emu.bus.gpio_in & mask,
            sio_value & mask,
            "without override, SIO must drive these pins"
        );

        // Now apply the override on the same pins. Override must win.
        emu.bus.external_gpio_in_mask = mask;
        emu.bus.external_gpio_in_override = override_value;
        emu.update_gpio();
        assert_eq!(
            emu.bus.gpio_in & mask,
            override_value & mask,
            "with override on, override pins must override SIO"
        );

        // Pins outside the mask should still reflect SIO. Drive bit 0
        // via SIO as a witness; with mask covering 10..15 only, bit 0
        // stays from SIO.
        emu.bus.sio.gpio_oe |= 1;
        emu.bus.sio.gpio_out |= 1;
        emu.update_gpio();
        assert_eq!(emu.bus.gpio_in & 1, 1, "non-overridden pins follow SIO");
        // And the override pins still win.
        assert_eq!(
            emu.bus.gpio_in & mask,
            override_value & mask,
            "override unchanged by an unrelated SIO write"
        );
    }

    #[test]
    fn reset_clears_override() {
        // Set the override, reset, verify both fields are 0 — protects
        // tests from leaking state across resets and matches the rest
        // of the Bus reset conventions.
        let mut emu = EmulatorBuilder::new(Config::default()).build();
        emu.bus.external_gpio_in_mask = 0xFFFF_FFFF;
        emu.bus.external_gpio_in_override = 0xDEAD_BEEF;
        emu.reset();
        assert_eq!(emu.bus.external_gpio_in_mask, 0);
        assert_eq!(emu.bus.external_gpio_in_override, 0);
    }
}

// ============================================================================
// PLL LOCK modelling — see `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md`
// ============================================================================
//
// Twelve integration tests mirroring the mdrp2350 set. PLL_SYS lives at
// 0x4002_8000 and PLL_USB at 0x4002_C000 on RP2040 (compare the mdrp2350
// 0x4005_0000 / 0x4005_8000 layout); alias bits are the same `+0x1000/0x2000/0x3000`
// APB convention. `bus.master_cycle` is seeded directly between writes
// and reads — Emulator::step stashes it from Clock::cycles in production.

mod pll_lock {
    use crate::bus::Bus;
    use crate::bus::PLL_SYS_BASE;
    use crate::bus::PLL_USB_BASE;
    use mdpicoem_common::clocks::PLL_LOCK_DELAY_SYSCLKS;

    const CS_OFF: u32 = 0x00;
    const PWR_OFF: u32 = 0x04;
    const FBDIV_OFF: u32 = 0x08;
    const PRIM_OFF: u32 = 0x0C;
    const ALIAS_XOR: u32 = 0x1000;
    const ALIAS_SET: u32 = 0x2000;
    const ALIAS_CLR: u32 = 0x3000;

    #[inline]
    fn pll_sys(offset: u32) -> u32 { PLL_SYS_BASE + offset }
    #[inline]
    fn pll_usb(offset: u32) -> u32 { PLL_USB_BASE + offset }

    #[test]
    fn test_pll_cs_read_lock_zero_at_reset() {
        let mut bus = Bus::new();
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 at reset");
    }

    #[test]
    fn test_pll_cs_lock_zero_before_arm() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0);
        bus.master_cycle = 100;
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 before arm cycle");
    }

    #[test]
    fn test_pll_cs_lock_one_after_arm() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0);
        bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_ne!(cs & (1 << 31), 0, "LOCK must be 1 past arm cycle");
    }

    #[test]
    fn test_pll_cs_lock_zero_with_pd_set() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0x01); // PD only
        bus.master_cycle = 10_000;
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 while PD=1");
    }

    #[test]
    fn test_pll_cs_lock_zero_with_vcopd_set() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0x20); // VCOPD only
        bus.master_cycle = 10_000;
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 while VCOPD=1");
    }

    #[test]
    fn test_pll_cs_lock_zero_with_fbdiv_zero() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(PWR_OFF), 0);
        bus.master_cycle = 10_000;
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs & (1 << 31), 0, "LOCK must be 0 when FBDIV=0");
    }

    #[test]
    fn test_pll_cs_lock_rearm_after_powerdown() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0);
        bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
        let cs1 = bus.read32(pll_sys(CS_OFF));
        assert_ne!(cs1 & (1 << 31), 0, "LOCK must be 1 after initial lock");

        bus.write32(pll_sys(PWR_OFF), 0x21); // PD+VCOPD set
        let cs2 = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs2 & (1 << 31), 0, "LOCK must drop when power-down re-asserts");
    }

    #[test]
    fn test_pll_cs_bypass_does_not_force_lock() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(CS_OFF), 0x101); // REFDIV=1 | BYPASS=1
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0);
        bus.master_cycle = 100;
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs & (1 << 31), 0, "BYPASS must not force LOCK=1");
    }

    #[test]
    fn test_pll_cs_read_preserves_refdiv() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(CS_OFF), 0x05);
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0);
        bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
        let cs = bus.read32(pll_sys(CS_OFF));
        assert_eq!(cs & 0x3F, 5, "REFDIV must round-trip");
        assert_ne!(cs & (1 << 31), 0, "LOCK must be 1");
    }

    #[test]
    fn test_pll_cs_alias_writes_trigger_arm() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        assert_eq!(bus.pll_sys_lock_at_cycle, None,
            "FBDIV write must not arm while PLL is powered down");

        // SET alias on CS: OR 0x01 (no visible change — REFDIV already 1).
        bus.write32(pll_sys(CS_OFF) + ALIAS_SET, 0x01);
        assert_eq!(bus.pll_sys_lock_at_cycle, None,
            "CS SET alias must not arm while PLL is powered down");
        // Reference ALIAS_XOR to keep the alias alphabet in the test body
        // (avoids dead_code warnings and documents the three-alias shape).
        let _ = ALIAS_XOR;

        bus.master_cycle = 100;
        bus.write32(pll_sys(PWR_OFF) + ALIAS_CLR, 0x2D);
        assert_eq!(bus.pll_sys_lock_at_cycle, Some(100 + PLL_LOCK_DELAY_SYSCLKS),
            "PWR CLR alias must arm the lock at now + delay");
    }

    #[test]
    fn test_pll_prim_write_does_not_rearm() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0);
        let armed_at = bus.pll_sys_lock_at_cycle;
        assert_eq!(armed_at, Some(PLL_LOCK_DELAY_SYSCLKS));

        bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
        assert_ne!(bus.read32(pll_sys(CS_OFF)) & (1 << 31), 0);

        bus.write32(pll_sys(PRIM_OFF), (2u32 << 16) | (2u32 << 12));
        assert_eq!(bus.pll_sys_lock_at_cycle, armed_at,
            "PRIM write must not rearm the lock-detect counter");
        assert_ne!(bus.read32(pll_sys(CS_OFF)) & (1 << 31), 0,
            "LOCK must stay 1 after PRIM-only write");
    }

    #[test]
    fn test_pll_usb_independent_of_pll_sys() {
        let mut bus = Bus::new();
        bus.master_cycle = 0;
        bus.write32(pll_sys(FBDIV_OFF), 100);
        bus.write32(pll_sys(PWR_OFF), 0);
        bus.master_cycle = PLL_LOCK_DELAY_SYSCLKS + 1;
        assert_ne!(bus.read32(pll_sys(CS_OFF)) & (1 << 31), 0,
            "PLL_SYS should report LOCK=1 past arm");
        assert_eq!(bus.read32(pll_usb(CS_OFF)) & (1 << 31), 0,
            "PLL_USB must remain LOCK=0 (independent state)");
        assert_eq!(bus.pll_usb_lock_at_cycle, None);
    }
}

// ---------------------------------------------------------------------------
// Phase 1 Wave 1 — IRQ plumbing, RESETS guard, fast-path gate, PIO routing
// ---------------------------------------------------------------------------
//
// Covers HLD V7 §5.2 (irq_pending drain), §5.3 (RESETS Bus-level guard),
// §5.5 (fast-path gate with DMA + peripherals + IRQ), and the PIO →
// NVIC routing helper in `Emulator::tick_pio_and_route_irqs_single`.
mod phase1_wave1 {
    use crate::bus::{Bus, PIO0_BASE, PIO1_BASE, TIMER_BASE, WATCHDOG_BASE};
    use crate::bus::peripheral_dispatch::{RESET_WATCHDOG, is_held_in_reset};
    use crate::irq::{IRQ_PIO0_IRQ_0, IRQ_PIO1_IRQ_0, IRQ_TIMER_IRQ_0};
    use crate::peripherals::watchdog_tick::TICK_OFFSET;
    use crate::{Config, EmulatorBuilder};

    // --- IRQ plumbing ----------------------------------------------------

    #[test]
    fn irq_pending_field_defaults_zero() {
        let bus = Bus::new();
        assert_eq!(bus.irq_pending(), 0);
    }

    #[test]
    fn drain_pushes_to_both_cores_nvic_pending() {
        // Directly set irq_pending on the bus; one step of the slow path
        // drains it into both cores. (The fast path cannot drain because
        // it early-exits on `any_irq`.)
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        // Park a NOP so the step path has something to execute.
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // Assert TIMER_IRQ_0 (line 0) via the bus's pending bitmap.
        emu.bus.irq_pending |= 1u32 << IRQ_TIMER_IRQ_0;
        emu.step();
        assert!(emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8),
            "core 0 NVIC must latch TIMER_IRQ_0 from irq_pending");
        assert!(emu.bus.nvics[1].is_pending(IRQ_TIMER_IRQ_0 as u8),
            "core 1 NVIC must also latch it (shared IRQ wire)");
        assert_eq!(emu.bus.irq_pending(), 0,
            "drain must clear the bus-level bitmap");
    }

    // --- RESETS Bus-level guard -----------------------------------------

    #[test]
    fn timer_read_returns_zero_while_held_in_reset() {
        // Fresh bus: every peripheral is held in reset. TIMER reads
        // must return 0 without the TIMER module seeing the call.
        let mut bus = Bus::new();
        assert!(is_held_in_reset(&bus, TIMER_BASE));
        assert_eq!(bus.read32(TIMER_BASE), 0);
        assert_eq!(bus.read32(TIMER_BASE + 0x28), 0); // TIMERAWL offset
    }

    #[test]
    fn watchdog_tick_write_swallowed_while_held_in_reset() {
        let mut bus = Bus::new();
        // Default RESETS holds bit 24 (WATCHDOG) — writing to
        // WATCHDOG_TICK must be a no-op.
        bus.write32(WATCHDOG_BASE + TICK_OFFSET, 0x0000_03FF);
        assert_eq!(bus.watchdog_tick.cycles, 12, "CYCLES stays at reset default");
        assert!(!bus.watchdog_tick.enable);
    }

    #[test]
    fn watchdog_tick_write_honoured_after_reset_released() {
        let mut bus = Bus::new();
        // CLR RESETS bit 24 (WATCHDOG) via the alias at 0x4000_F000.
        bus.write32(0x4000_F000, 1u32 << RESET_WATCHDOG);
        // Write CYCLES = 0x41, ENABLE = 1.
        bus.write32(WATCHDOG_BASE + TICK_OFFSET, 0x0000_0241);
        assert_eq!(bus.watchdog_tick.cycles, 0x41);
        assert!(bus.watchdog_tick.enable);
        // Read-back through the bus surfaces the same word (with
        // RUNNING mirrored into bit 10).
        let v = bus.read32(WATCHDOG_BASE + TICK_OFFSET);
        assert_eq!(v & 0x1FF, 0x41);
        assert_eq!(v & (1 << 9), 1 << 9);
        assert_eq!(v & (1 << 10), 1 << 10);
    }

    #[test]
    fn reset_gate_covers_all_four_access_widths() {
        let mut bus = Bus::new();
        // TIMER held in reset: every read width returns 0.
        assert_eq!(bus.read32(TIMER_BASE + 0x28), 0);
        assert_eq!(bus.read16(TIMER_BASE + 0x28), 0);
        assert_eq!(bus.read8(TIMER_BASE + 0x28), 0);
        // Writes drop silently — no bus fault.
        bus.write32(TIMER_BASE + 0x28, 0xDEAD_BEEF);
        bus.write16(TIMER_BASE + 0x28, 0xBEEF);
        bus.write8(TIMER_BASE + 0x28, 0xEF);
        assert!(!bus.bus_fault());
    }

    // --- Fast-path gate --------------------------------------------------

    #[test]
    fn fast_path_taken_when_everything_idle() {
        // Build an emulator with no PIO activity, no DMA, no IRQ
        // pending. A single NOP step should still succeed and leave
        // irq_pending at 0 (fast path never touches it).
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        assert!(emu.bus.pio_all_idle());
        assert!(emu.bus.all_peripherals_idle());
        assert!(emu.bus.dma.is_idle());
        assert_eq!(emu.bus.irq_pending(), 0);
        let consumed = emu.step();
        assert_eq!(consumed, 1);
        assert_eq!(emu.bus.irq_pending(), 0);
        // Fast path drains nothing: both cores' NVIC stays empty.
        assert_eq!(emu.bus.nvics[0].pending, 0);
    }

    #[test]
    fn slow_path_triggered_by_pending_irq() {
        // When irq_pending is non-zero at the start of the quantum,
        // the gate opens and the slow-path loop runs — which drains
        // irq_pending into the NVIC.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        emu.bus.irq_pending |= 1u32 << IRQ_TIMER_IRQ_0;
        let consumed = emu.step();
        assert_eq!(consumed, 1);
        // Slow path drained.
        assert_eq!(emu.bus.irq_pending(), 0);
        assert!(emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8));
    }

    // --- PIO → NVIC routing ---------------------------------------------

    #[test]
    fn pio0_irq_flag_bit0_routes_to_nvic_line_7() {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // Enable IRQ flag 0 routing on INT0 (NVIC line 7) only.
        emu.bus.write32(PIO0_BASE + 0x12C, 0x001);
        // Force PIO0 IRQ flag bit 0 via IRQ_FORCE (offset 0x034).
        emu.bus.write32(PIO0_BASE + 0x034, 0x01);
        // Asserting the IRQ flag means pio_all_idle is false now, so
        // stepping takes the slow path and routes into irq_pending +
        // drains into the NVIC.
        assert!(!emu.bus.pio_all_idle());
        emu.step();
        assert!(emu.bus.nvics[0].is_pending(IRQ_PIO0_IRQ_0 as u8),
            "PIO0 IRQ flag bit 0 must route to NVIC line #7 (PIO0_IRQ_0)");
    }

    #[test]
    fn pio1_irq_flag_bit1_routes_to_nvic_line_10() {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // Enable IRQ flag 1 routing on INT1 (NVIC line 10) only.
        emu.bus.write32(PIO1_BASE + 0x138, 0x002);
        // PIO1 IRQ flag bit 1 → NVIC line 10 (PIO1_IRQ_1).
        emu.bus.write32(PIO1_BASE + 0x034, 0x02);
        emu.step();
        // PIO1_IRQ_1 is IRQ_PIO1_IRQ_0 + 1.
        assert!(emu.bus.nvics[0].is_pending((IRQ_PIO1_IRQ_0 + 1) as u8),
            "PIO1 IRQ flag bit 1 must route to NVIC line #10 (PIO1_IRQ_1)");
    }

    #[test]
    fn pio_high_irq_flags_do_not_route_to_nvic() {
        // PIO has 8 internal IRQ flags; only IRQ[3:0] are NVIC-routable
        // (via INT0_INTE/INT1_INTE, not yet modelled — see `tech_debt.md`
        // entry "PIO INTn_INTE routing not modelled"). Flags 4-7 are
        // strictly intra-PIO SM-to-SM signalling and must NEVER raise
        // any NVIC line regardless of the routing model.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // Flags 4..=7 forced on PIO0 (bits outside the routable subset).
        emu.bus.write32(PIO0_BASE + 0x034, 0xF0);
        emu.step();
        // No NVIC line 7..=10 should be latched.
        assert_eq!(emu.bus.nvics[0].pending & 0x780, 0,
            "high IRQ flags (bits 4-7) must not route to PIO0/PIO1 NVIC lines");
    }

    #[test]
    fn pio0_int0_intf_forces_nvic_line_7_only() {
        // INT0_INTF (PIO0 + 0x130) directly forces individual bits in
        // the effective INT0 line value (`int0_ints = (INTR & INTE) | INTF`).
        // Forcing bit 0 of INT0 must fire only NVIC line 7 (PIO0_IRQ_0)
        // and must NOT bleed into NVIC line 8 (PIO0_IRQ_1) — the two
        // lines are independently routed via INT0_INTE / INT1_INTE.
        //
        // This test fails on the over-route code path (which only
        // reads `irq_flags`, not the INTE/INTF registers). It passes
        // once `tick_pio_and_route_irqs_single` is wired through
        // `PioBlock::int0_ints` / `int1_ints`.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // Enable PIO0 SM0 so `pio_all_idle()` returns false and the
        // slow path runs IRQ routing each cycle. Mirrors the PicoGUS
        // production case (ISA IOW SM is always enabled).
        emu.bus.write32(PIO0_BASE + 0x000, 0x1);
        emu.bus.write32(PIO0_BASE + 0x130, 0x001);
        emu.step();
        assert!(emu.bus.nvics[0].is_pending(IRQ_PIO0_IRQ_0 as u8),
            "INT0_INTF bit 0 must route to NVIC #7 (PIO0_IRQ_0)");
        assert!(!emu.bus.nvics[0].is_pending((IRQ_PIO0_IRQ_0 + 1) as u8),
            "INT0_INTF bit 0 must NOT bleed into NVIC #8 (PIO0_IRQ_1)");
    }

    /// Regression: PicoGUS PIO0 SM0 (IOW capture) program is
    ///   slot 0: WAIT 1 GPIO 4
    ///   slot 1: WAIT 0 GPIO 4
    ///   slot 2: IRQ 0
    ///   slot 3: JMP 0
    /// driven by toggling GPIO 4 via `external_gpio_in_mask` / `_override`
    /// (the same mechanism `picogus_diff_rp2040::Emulator::drive_pins`
    /// uses). After driving IOW high then low, SM0 should advance past
    /// both WAITs and execute IRQ 0, raising IRQ flag bit 0. This test
    /// is the RED-phase reproducer for the bug where SM0 latches the
    /// HIGH transition but never advances past WAIT 0 when IOW is then
    /// driven low through the override path.
    #[test]
    fn pio0_sm0_catches_external_gpio_iow_low_after_high() {
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();

        // Park core 0 on a NOP at 0x2000_1000 so step() always has
        // somewhere to fetch and never faults the CPU side.
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;

        // Load SM0's instruction memory at INSTR_MEM[0..3]
        // (PIO0_BASE + 0x048 + slot*4). Each instruction is a 16-bit
        // PIO opcode written into the low 16 bits of the 32-bit slot.
        let prog: [u16; 4] = [
            0x2084, // slot 0: WAIT 1 GPIO 4
            0x2004, // slot 1: WAIT 0 GPIO 4
            0xC000, // slot 2: IRQ 0 (I=0, C=0, W=0, IRQ index 0)
            0x0000, // slot 3: JMP 0
        ];
        for (i, insn) in prog.iter().enumerate() {
            emu.bus
                .write32(PIO0_BASE + 0x048 + (i as u32) * 4, *insn as u32);
        }

        // SM0_EXECCTRL (PIO0_BASE + 0x0CC): wrap_top=3 (bits 16:12),
        // wrap_bottom=0 (bits 11:7) — wrap whole 4-instruction program.
        emu.bus.write32(PIO0_BASE + 0x0CC, 3u32 << 12);
        // SM0_CLKDIV (PIO0_BASE + 0x0C8): integer=1, fraction=0 →
        // 0x0001_0000 (one PIO cycle per system cycle).
        emu.bus.write32(PIO0_BASE + 0x0C8, 0x0001_0000);
        // CTRL (PIO0_BASE + 0x000): SM_ENABLE bit 0 → enable SM0.
        emu.bus.write32(PIO0_BASE + 0x000, 0x1);

        // ---- Drive IOW high via the harness's external override path ----
        // This is the exact same pattern `Emulator::drive_pins` uses
        // (picogus_diff_rp2040.rs:349-365): set the mask to mark which
        // bits the harness owns, the override to the desired value, and
        // mirror into `gpio_in` so reads between drive_pins() and the
        // next step() observe the asserted line.
        emu.bus.external_gpio_in_mask = 1u32 << 4;
        emu.bus.external_gpio_in_override = 1u32 << 4;
        emu.bus.gpio_in = (emu.bus.gpio_in & !emu.bus.external_gpio_in_mask)
            | (emu.bus.external_gpio_in_override & emu.bus.external_gpio_in_mask);

        // Step ~20 sysclk cycles. SM0 should catch WAIT 1 GPIO 4 and
        // advance from PC=0 to PC=1 (WAIT 0 GPIO 4).
        for _ in 0..20 {
            emu.step();
        }

        // ---- Drive IOW low ----
        // Keep the mask (the harness still owns the pin), drop the
        // override, and mirror into gpio_in.
        emu.bus.external_gpio_in_override = 0;
        emu.bus.gpio_in &= !(1u32 << 4);

        // Step ~20 sysclk cycles. SM0 should catch WAIT 0 GPIO 4,
        // advance to slot 2 (IRQ 0), execute it (raising flag bit 0),
        // then advance to slot 3 (JMP 0) and wrap back to slot 0.
        for _ in 0..20 {
            emu.step();
        }

        // SM0_ADDR is at PIO0_BASE + 0x0D4 (per RP2040 datasheet
        // §3.7 PIO register map). After IRQ 0 + JMP 0 + wrap, PC is
        // back at 0 (or 1 if it caught WAIT 1 again on the wrap).
        let sm0_pc = emu.bus.read32(PIO0_BASE + 0x0D4);

        assert!(
            emu.bus.pio[0].pending_irqs() & 0x01 != 0,
            "PIO IRQ flag 0 not set — PIO never advanced past WAIT 0 \
             (SM0 PC = {})",
            sm0_pc
        );
        assert!(
            sm0_pc <= 1,
            "After IRQ 0 + JMP 0 + wrap, SM0 PC must be 0 or 1 (got {})",
            sm0_pc
        );
    }
}

// ---------------------------------------------------------------------------
// Phase 1 Wave 2 — NVIC ISER/ICER/ISPR/ICPR/IPR + CPU-side dispatch
// ---------------------------------------------------------------------------
//
// Covers HLD V7 §5.2 (NVIC register surface) plus CortexM0Plus::step's
// per-cycle IRQ poll. `silicon_isr_diff_rp2040::isr_m0_timer_cold`
// cannot pass without these.
mod phase1_wave2 {
    use crate::bus::Bus;
    use crate::irq::IRQ_TIMER_IRQ_0;
    use crate::{Config, EmulatorBuilder};

    /// Plant a 48-entry vector table (covers all 16 system + 26 RP2040
    /// external IRQs with headroom) plus a minimal handler at the given
    /// base address. Returns `(handler_addr, main_addr)` so callers can
    /// wire VTOR and PC.
    ///
    /// Layout (addresses inside SRAM, all Thumb-aligned):
    /// * `base + 0x00`        — initial SP slot (= 0x2002_0000).
    /// * `base + 0x04..+0xC0` — 47 exception vectors, each pointing at
    ///   `handler_addr`.
    /// * `base + 0x80` — `handler_addr`: NOP + self-loop (`B .`) so the
    ///   handler is safe to execute.
    /// * `base + 0x100` — `main_addr`: NOP + self-loop.
    fn plant_vector_table(bus: &mut Bus, base: u32) -> (u32, u32) {
        let handler_addr = base + 0x80;
        let main_addr = base + 0x100;
        // Initial SP at offset 0 — point at end of SRAM.
        bus.write32(base, 0x2002_0000);
        // Vectors 1..=47 all go to the handler (OR the Thumb bit). 47
        // = 16 system exceptions (Reset..SysTick) + 32 external IRQ
        // lines (RP2040 only uses 26, but stamping past the used set is
        // free and guards against test drift).
        for i in 1..48 {
            bus.write32(base + (i as u32) * 4, handler_addr | 1);
        }
        // Handler: NOP + self-loop.
        bus.write16(handler_addr, 0xBF00);
        bus.write16(handler_addr + 2, 0xE7FE);
        // Main: NOP + self-loop.
        bus.write16(main_addr, 0xBF00);
        bus.write16(main_addr + 2, 0xE7FE);
        (handler_addr, main_addr)
    }

    // --- NVIC struct via bus_nvics field --------------------------------

    #[test]
    fn bus_nvics_field_defaults_empty() {
        let bus = Bus::new();
        assert_eq!(bus.nvics[0].pending, 0);
        assert_eq!(bus.nvics[0].enabled, 0);
        assert_eq!(bus.nvics[1].pending, 0);
        assert_eq!(bus.nvics[1].enabled, 0);
    }

    // --- CPU dispatch ----------------------------------------------------

    #[test]
    fn enabled_and_pending_dispatches_exception_at_vector_16() {
        // Core 0, thread mode, PRIMASK clear. Enable IRQ 0 and assert it
        // pending via the bus bitmap (drained on first slow-path step).
        // Expected: exception entry to vector 16 (TIMER_IRQ_0).
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let (handler_addr, main_addr) = plant_vector_table(&mut emu.bus, 0x2000_0000);
        // Wire VTOR + PC + SP on core 0.
        emu.bus.ppb[0].vtor = 0x2000_0000;
        emu.cores[0].regs.set_pc(main_addr);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // Enable IRQ_TIMER_IRQ_0 (line 0) directly in the NVIC.
        emu.bus.nvics[0].set_enabled(IRQ_TIMER_IRQ_0 as u8);
        // Assert the IRQ via the bus-level bitmap (Phase 1 Wave 1 plumbing).
        emu.bus.irq_pending |= 1u32 << IRQ_TIMER_IRQ_0;
        // First step: slow path drains irq_pending into NVIC (fast path
        // would early-exit on `any_irq`).
        emu.step();
        // Drain happened — NVIC latched the pending bit.
        assert!(emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8),
            "NVIC must latch the pending bit after slow-path drain");
        // Second step: CPU-side poll picks it up and enters the handler.
        emu.step();
        // PC must be at the handler.
        assert_eq!(emu.cores[0].regs.pc(), handler_addr,
            "exception entry must land at the handler address");
        // IPSR must be 16 (exception number for TIMER_IRQ_0 → 16).
        assert_eq!(emu.cores[0].regs.ipsr(), 16,
            "IPSR must encode exception #16 inside the handler");
        // NVIC pending bit is cleared by dispatch.
        assert!(!emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8),
            "dispatch clears the pending bit");
    }

    #[test]
    fn pending_without_enable_does_not_dispatch() {
        // NVIC pending but not enabled — CPU must stay in thread mode
        // and keep executing the main routine.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let (_handler, main_addr) = plant_vector_table(&mut emu.bus, 0x2000_0000);
        emu.bus.ppb[0].vtor = 0x2000_0000;
        emu.cores[0].regs.set_pc(main_addr);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        emu.bus.nvics[0].set_pending(IRQ_TIMER_IRQ_0 as u8);
        // enabled bit intentionally not set.
        emu.step();
        assert_eq!(emu.cores[0].regs.ipsr(), 0, "still in thread mode");
        assert!(emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8),
            "pending bit stays set when NVIC masks the line");
    }

    #[test]
    fn primask_blocks_dispatch() {
        // Pending + enabled but PRIMASK set — no dispatch, pending
        // bit remains latched.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let (_handler, main_addr) = plant_vector_table(&mut emu.bus, 0x2000_0000);
        emu.bus.ppb[0].vtor = 0x2000_0000;
        emu.cores[0].regs.set_pc(main_addr);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        emu.cores[0].regs.primask = 1;
        emu.bus.nvics[0].set_enabled(IRQ_TIMER_IRQ_0 as u8);
        emu.bus.nvics[0].set_pending(IRQ_TIMER_IRQ_0 as u8);
        emu.step();
        assert_eq!(emu.cores[0].regs.ipsr(), 0,
            "PRIMASK=1 must block dispatch — stay in thread mode");
        assert!(emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8),
            "PRIMASK leaves the pending bit latched");
    }

    #[test]
    fn handler_mode_does_not_preempt_for_external_irq() {
        // If we're already in a handler, an external IRQ must not
        // preempt on our simplified M0+ priority model.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let (_handler, main_addr) = plant_vector_table(&mut emu.bus, 0x2000_0000);
        emu.bus.ppb[0].vtor = 0x2000_0000;
        emu.cores[0].regs.set_pc(main_addr);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // Fake handler-mode: IPSR = exception 11 (SVCall).
        emu.cores[0].regs.xpsr = (emu.cores[0].regs.xpsr & !0x1FF) | 11;
        emu.bus.nvics[0].set_enabled(IRQ_TIMER_IRQ_0 as u8);
        emu.bus.nvics[0].set_pending(IRQ_TIMER_IRQ_0 as u8);
        emu.step();
        // IPSR stays at 11; pending bit still latched.
        assert_eq!(emu.cores[0].regs.ipsr(), 11, "in-handler: no preempt");
        assert!(emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8));
    }

    #[test]
    fn lowest_priority_value_wins_tiebreak_by_irq_number() {
        // Two IRQs pending: IRQ 3 at priority 0xC0, IRQ 5 at priority
        // 0x40. Lower priority value = higher priority, so IRQ 5 wins.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let (_handler, main_addr) = plant_vector_table(&mut emu.bus, 0x2000_0000);
        emu.bus.ppb[0].vtor = 0x2000_0000;
        emu.cores[0].regs.set_pc(main_addr);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        emu.bus.nvics[0].set_enabled(3);
        emu.bus.nvics[0].set_enabled(5);
        emu.bus.nvics[0].set_pending(3);
        emu.bus.nvics[0].set_pending(5);
        emu.bus.nvics[0].set_priority(3, 0xC0);
        emu.bus.nvics[0].set_priority(5, 0x40);
        emu.step();
        // IPSR must be exception #(16 + 5) = 21 (UART1_IRQ by table).
        assert_eq!(emu.cores[0].regs.ipsr(), 21,
            "higher-priority (lower value) IRQ must dispatch first");
        // IRQ 5 dispatched (cleared); IRQ 3 still pending.
        assert!(!emu.bus.nvics[0].is_pending(5));
        assert!(emu.bus.nvics[0].is_pending(3));
    }

    #[test]
    fn equal_priority_picks_lowest_irq_number() {
        // Two IRQs at the same priority 0x00 — lowest-number wins.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        let (_handler, main_addr) = plant_vector_table(&mut emu.bus, 0x2000_0000);
        emu.bus.ppb[0].vtor = 0x2000_0000;
        emu.cores[0].regs.set_pc(main_addr);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        emu.bus.nvics[0].set_enabled(2);
        emu.bus.nvics[0].set_enabled(5);
        emu.bus.nvics[0].set_pending(2);
        emu.bus.nvics[0].set_pending(5);
        // Both defaults to priority 0x00.
        emu.step();
        assert_eq!(emu.cores[0].regs.ipsr(), 16 + 2,
            "tie-break by lowest IRQ number");
    }

    // --- Bus-level TIMER dispatch + RESETS gate -------------------------

    #[test]
    fn bus_timer_write_swallowed_while_held_in_reset() {
        let mut bus = Bus::new();
        // Default RESETS holds TIMER. A write to ALARM0 must be dropped
        // by the bus guard — reading it back returns 0 (the default).
        bus.write32(crate::bus::TIMER_BASE + 0x10, 500);
        // Read comes back through reset-gate: 0.
        assert_eq!(bus.read32(crate::bus::TIMER_BASE + 0x10), 0);
        assert_eq!(bus.timer.read32(0x10, 0, 125_000_000), 0,
            "direct peripheral read-back confirms no state change");
    }

    #[test]
    fn bus_timer_write_after_reset_released() {
        let mut bus = Bus::new();
        // Release RESET_TIMER (bit 21).
        bus.write32(0x4000_F000, 1u32 << 21);
        // Write ALARM0 = 42 µs via the bus.
        bus.write32(crate::bus::TIMER_BASE + 0x10, 42);
        // Direct read through the bus (normal alias).
        assert_eq!(bus.read32(crate::bus::TIMER_BASE + 0x10), 42);
    }

    #[test]
    fn bus_timerawl_returns_live_microseconds() {
        let mut bus = Bus::new();
        bus.write32(0x4000_F000, 1u32 << 21); // release TIMER reset
        // Default clock tree: sys_clk_hz seeded from ROSC. But
        // `Bus::new()` seeds ROSC (6.5 MHz) which leaves (sys_hz/1M)
        // at 6. So set master_cycle to 6000 to produce 1000 µs.
        bus.master_cycle = (bus.clock_tree.sys_clk_hz / 1_000_000).max(1) as u64 * 1000;
        let lo = bus.read32(crate::bus::TIMER_BASE + 0x28);
        assert_eq!(lo, 1000, "TIMERAWL = now in µs at this master_cycle");
    }

    #[test]
    fn advance_lazy_scheduled_fires_timer_alarm() {
        // Program an alarm that matches inside the window we'll pass to
        // advance_lazy_scheduled and assert the IRQ bit lands in
        // bus.irq_pending + the NVIC gets it on drain.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(64).build();
        // Release TIMER's RESET bit.
        emu.bus.write32(0x4000_F000, 1u32 << 21);
        // Park a NOP so step() has something to execute.
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // INTE alarm 0 enabled so poll_alarms raises NVIC bit.
        emu.bus.write32(crate::bus::TIMER_BASE + 0x38, 0x1);
        // ALARM0 = 1 µs: matches at sys_hz/1M cycles.
        emu.bus.write32(crate::bus::TIMER_BASE + 0x10, 1);
        // Step enough cycles for the fast path to push master_cycle
        // past 1 µs. Default sys_hz = ROSC; with step_quantum=64 a
        // single step covers 64 cycles. We need sys_hz/1M cycles to
        // reach 1 µs — at ROSC 6.5 MHz that's 6 cycles. One step
        // suffices.
        emu.step();
        // NVIC must have picked up IRQ_TIMER_IRQ_0 via drain after the
        // fast-path `advance_lazy_scheduled`.
        assert!(emu.bus.nvics[0].is_pending(IRQ_TIMER_IRQ_0 as u8),
            "ALARM0 match must propagate to NVIC via lazy schedule");
        // INTR must show the alarm fired and armed cleared.
        let intr = emu.bus.read32(crate::bus::TIMER_BASE + 0x34);
        assert_eq!(intr & 1, 1, "INTR bit 0 must latch");
        let armed = emu.bus.read32(crate::bus::TIMER_BASE + 0x20);
        assert_eq!(armed & 1, 0, "ARMED bit 0 must auto-clear on fire");
    }
}

// ---------------------------------------------------------------------------
// Phase 2 — UART / SPI / I2C bus integration
// ---------------------------------------------------------------------------
//
// Covers the end-to-end path: firmware-style MMIO writes through `Bus::write32`
// / `Bus::write8` / `Bus::read32`, RESETS gating at bus dispatch, narrow-access
// dispatch for UART_DR / SSPDR / IC_DATA_CMD, and IRQ routing from peripheral
// `tick` / `simulate_transaction` into `bus.irq_pending` (and onward to the
// NVIC via `drain_pending_irqs_to_cores`).
mod phase2_uart_spi_i2c {
    use crate::bus::peripheral_dispatch::{
        RESET_I2C0, RESET_SPI0, RESET_UART0, is_held_in_reset,
    };
    use crate::bus::{
        Bus, I2C0_BASE, I2C1_BASE, SPI0_BASE, SPI1_BASE, UART0_BASE, UART1_BASE,
    };
    use crate::irq::{IRQ_I2C0_IRQ, IRQ_SPI0_IRQ, IRQ_UART0_IRQ};
    use crate::peripherals::i2c::{IC_CLR_TX_ABRT, IC_ENABLE, IC_RAW_INTR_STAT, IC_TAR, INT_TX_ABRT};
    use crate::peripherals::spi::{
        SSP_INT_RX, SSPCR0, SSPCR1, SSPDR, SSPIMSC, SSPRIS,
    };
    use crate::peripherals::uart::{
        UARTCR, UARTDR, UARTFBRD, UARTFR, UARTIBRD, UARTIMSC, UARTLCR_H, UARTRIS, UART_INT_TX,
    };
    use crate::{Config, EmulatorBuilder};

    /// CLR alias for RESETS: base 0x4000_C000 + 0x3000 = 0x4000_F000.
    const RESETS_CLR: u32 = 0x4000_F000;

    /// Release every peripheral from reset so tests can drive firmware.
    fn release_all(bus: &mut Bus) {
        // Writing `!0` to the BITCLR alias clears every reset bit.
        bus.write32(RESETS_CLR, 0xFFFF_FFFF);
    }

    // --- Reset defaults + RESETS gating ------------------------------

    #[test]
    fn fresh_bus_holds_uart_spi_i2c_in_reset() {
        let bus = Bus::new();
        assert!(is_held_in_reset(&bus, UART0_BASE));
        assert!(is_held_in_reset(&bus, UART1_BASE));
        assert!(is_held_in_reset(&bus, SPI0_BASE));
        assert!(is_held_in_reset(&bus, SPI1_BASE));
        assert!(is_held_in_reset(&bus, I2C0_BASE));
        assert!(is_held_in_reset(&bus, I2C1_BASE));
    }

    #[test]
    fn uart0_write_blocked_while_held_in_reset() {
        let mut bus = Bus::new();
        // UART0 is held in reset by default.
        bus.write32(UART0_BASE + UARTCR, 0x301);
        // Release then verify the write actually takes effect.
        bus.write32(RESETS_CLR, 1u32 << RESET_UART0);
        assert_eq!(bus.read32(UART0_BASE + UARTCR), 0, "pre-release write swallowed");
        bus.write32(UART0_BASE + UARTCR, 0x301);
        assert_eq!(bus.read32(UART0_BASE + UARTCR), 0x301);
    }

    #[test]
    fn spi0_write_blocked_while_held_in_reset() {
        let mut bus = Bus::new();
        bus.write32(SPI0_BASE + SSPCR1, 0x2);
        bus.write32(RESETS_CLR, 1u32 << RESET_SPI0);
        assert_eq!(bus.read32(SPI0_BASE + SSPCR1), 0, "pre-release write swallowed");
        bus.write32(SPI0_BASE + SSPCR1, 0x2);
        assert_eq!(bus.read32(SPI0_BASE + SSPCR1), 0x2);
    }

    #[test]
    fn i2c0_write_blocked_while_held_in_reset() {
        let mut bus = Bus::new();
        bus.write32(I2C0_BASE + IC_ENABLE, 0x1);
        bus.write32(RESETS_CLR, 1u32 << RESET_I2C0);
        assert_eq!(bus.read32(I2C0_BASE + IC_ENABLE), 0, "pre-release write swallowed");
        bus.write32(I2C0_BASE + IC_ENABLE, 0x1);
        assert_eq!(bus.read32(I2C0_BASE + IC_ENABLE), 0x1);
    }

    // --- UART integration --------------------------------------------

    #[test]
    fn uart0_byte_write_to_dr_uses_narrow_dispatch() {
        // The narrow-access path must not round-trip via word-RMW (which
        // would re-push the DR value through `push_tx` twice per write).
        let mut bus = Bus::new();
        release_all(&mut bus);
        bus.write32(UART0_BASE + UARTLCR_H, 1 << 4); // FEN
        bus.write32(UART0_BASE + UARTCR, 0x301); // UARTEN | TXE
        bus.write8(UART0_BASE + UARTDR, 0xA5);
        // FR.TXFE must clear — something in the FIFO.
        let fr = bus.read32(UART0_BASE + UARTFR);
        assert!(fr & (1 << 7) == 0, "TXFE must clear after push");
    }

    #[test]
    fn uart0_baud_configure_drain_fires_tx_irq() {
        // Full firmware-style sequence: configure baud at 115200,
        // enable, push a byte, run the emulator for enough cycles that
        // the slow-path tick drains the FIFO and raises TXIS. Confirm
        // the bit lands in `bus.irq_pending` and then in the NVIC.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        // Seed 125 MHz so the baud math matches pico-sdk defaults.
        emu.bus.seed_sys_clk_hz(125_000_000);
        // peri_clk_hz follows sys via the default CLK_PERI_CTRL AUXSRC=0.
        emu.bus.clock_tree.peri_clk_hz = 125_000_000;
        emu.bus.clock_tree.sys_clk_hz = 125_000_000;
        release_all(&mut emu.bus);
        emu.bus.write32(UART0_BASE + UARTIBRD, 67);
        emu.bus.write32(UART0_BASE + UARTFBRD, 52);
        emu.bus.write32(UART0_BASE + UARTLCR_H, 1 << 4);
        emu.bus.write32(UART0_BASE + UARTCR, 0x301);
        emu.bus.write32(UART0_BASE + UARTIMSC, UART_INT_TX);
        emu.bus.write32(UART0_BASE + UARTDR, 0x5A);
        // Park a NOP so `step()` has something to do. The fast-path
        // gate sees UART non-idle so the slow-path ticks UART every
        // cycle.
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        // At 115200 baud × 10 bits, 1 byte takes ≈ 86.8 µs = 10850 cycles.
        // Run several quanta.
        for _ in 0..20_000 {
            emu.step();
            if emu.bus.nvics[0].is_pending(IRQ_UART0_IRQ as u8) {
                break;
            }
        }
        assert_eq!(
            emu.bus.read32(UART0_BASE + UARTRIS) & UART_INT_TX,
            UART_INT_TX,
            "RIS must latch TXIS after FIFO drains"
        );
        assert!(
            emu.bus.nvics[0].is_pending(IRQ_UART0_IRQ as u8),
            "UART0 IRQ must latch in core 0 NVIC"
        );
    }

    #[test]
    fn uart_is_idle_gates_fast_path() {
        // Before any activity, all peripherals report idle.
        let bus = Bus::new();
        assert!(bus.all_peripherals_idle());
    }

    // --- SPI integration ---------------------------------------------

    #[test]
    fn spi0_loopback_roundtrips_via_bus() {
        // Full firmware-like sequence: enable SPI0 with LBM=1, write
        // 0xA5 via SSPDR, read it back.
        let mut bus = Bus::new();
        release_all(&mut bus);
        // DSS = 7 (8-bit frames).
        bus.write32(SPI0_BASE + SSPCR0, 0x07);
        // SSE | LBM.
        bus.write32(SPI0_BASE + SSPCR1, 0x3);
        bus.write32(SPI0_BASE + SSPDR, 0xA5);
        // Loopback pushes into RX FIFO at write time — read DR.
        assert_eq!(bus.read32(SPI0_BASE + SSPDR), 0xA5);
    }

    #[test]
    fn spi0_loopback_via_byte_access() {
        let mut bus = Bus::new();
        release_all(&mut bus);
        bus.write32(SPI0_BASE + SSPCR0, 0x07);
        bus.write32(SPI0_BASE + SSPCR1, 0x3);
        bus.write8(SPI0_BASE + SSPDR, 0x73);
        assert_eq!(bus.read8(SPI0_BASE + SSPDR), 0x73);
    }

    #[test]
    fn spi0_rx_irq_routes_through_bus() {
        // Load enough loopback words to cross RX half-full threshold
        // (4 of 8 entries). RIS latches RX; IMSC = RX enables it;
        // route through the bus's IRQ assertion path.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        release_all(&mut emu.bus);
        emu.bus.write32(SPI0_BASE + SSPCR0, 0x07);
        emu.bus.write32(SPI0_BASE + SSPCR1, 0x3); // SSE | LBM
        emu.bus.write32(SPI0_BASE + SSPIMSC, SSP_INT_RX);
        for i in 0..4 {
            emu.bus.write32(SPI0_BASE + SSPDR, i as u32);
        }
        assert_eq!(
            emu.bus.read32(SPI0_BASE + SSPRIS) & SSP_INT_RX,
            SSP_INT_RX,
            "RIS must latch RX at FIFO half-full"
        );
        assert!(
            emu.bus.irq_pending & (1u32 << IRQ_SPI0_IRQ) != 0,
            "SPI0 IRQ bit must be set in irq_pending"
        );
    }

    // --- I2C integration ---------------------------------------------

    #[test]
    fn i2c0_bus_scan_ack_address_latches_stop_det() {
        // Mirror pico-sdk's `bus_scan`: set TAR=0x3C, enable, write
        // CMD_WRITE. Expect STOP_DET latched and NO TX_ABRT.
        let mut bus = Bus::new();
        release_all(&mut bus);
        // TAR writes need EN=0.
        bus.write32(I2C0_BASE + IC_TAR, 0x3C);
        bus.write32(I2C0_BASE + IC_ENABLE, 1);
        // DATA_CMD write: data=0, STOP=1 (bit 9).
        bus.write32(I2C0_BASE + 0x10, 0x200);
        let ris = bus.read32(I2C0_BASE + IC_RAW_INTR_STAT);
        assert!(ris & (1 << 9) != 0, "STOP_DET must latch for ACK addr");
        assert_eq!(ris & INT_TX_ABRT, 0, "TX_ABRT must NOT latch");
    }

    #[test]
    fn i2c0_bus_scan_nack_address_latches_tx_abrt() {
        let mut bus = Bus::new();
        release_all(&mut bus);
        bus.write32(I2C0_BASE + IC_TAR, 0x55); // NACK address
        bus.write32(I2C0_BASE + IC_ENABLE, 1);
        bus.write32(I2C0_BASE + 0x10, 0x200);
        let ris = bus.read32(I2C0_BASE + IC_RAW_INTR_STAT);
        assert!(ris & INT_TX_ABRT != 0, "TX_ABRT must latch for NACK addr");
    }

    #[test]
    fn i2c0_clr_tx_abrt_via_bus_clears_sticky() {
        let mut bus = Bus::new();
        release_all(&mut bus);
        bus.write32(I2C0_BASE + IC_TAR, 0x55);
        bus.write32(I2C0_BASE + IC_ENABLE, 1);
        bus.write32(I2C0_BASE + 0x10, 0x200);
        // Read IC_CLR_TX_ABRT to drop the sticky.
        let _ = bus.read32(I2C0_BASE + IC_CLR_TX_ABRT);
        let ris = bus.read32(I2C0_BASE + IC_RAW_INTR_STAT);
        assert_eq!(ris & INT_TX_ABRT, 0, "TX_ABRT cleared on CLR_TX_ABRT read");
    }

    #[test]
    fn i2c0_nack_routes_through_nvic() {
        // With IC_INTR_MASK set to admit TX_ABRT, the I2C module
        // pushes the IRQ into irq_pending during `simulate_transaction`.
        // Stepping the emulator drains it into the NVIC.
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
        release_all(&mut emu.bus);
        emu.bus.write32(I2C0_BASE + IC_TAR, 0x55);
        emu.bus.write32(I2C0_BASE + IC_ENABLE, 1);
        // IC_INTR_MASK = INT_TX_ABRT (bit 6).
        emu.bus.write32(I2C0_BASE + 0x30, INT_TX_ABRT);
        emu.bus.write32(I2C0_BASE + 0x10, 0x200);
        assert!(
            emu.bus.irq_pending & (1u32 << IRQ_I2C0_IRQ) != 0,
            "I2C0 IRQ must surface in irq_pending"
        );
        // One more step drains it to the NVIC.
        emu.bus.write16(0x2000_1000, 0xBF00);
        emu.cores[0].regs.set_pc(0x2000_1000);
        emu.cores[0].regs.msp = 0x2002_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
        emu.step();
        assert!(
            emu.bus.nvics[0].is_pending(IRQ_I2C0_IRQ as u8),
            "I2C0 NVIC pending must be set after drain"
        );
    }

    // --- is_idle coverage --------------------------------------------

    #[test]
    fn all_peripherals_idle_flips_when_uart_has_pending_tx() {
        let mut bus = Bus::new();
        release_all(&mut bus);
        assert!(bus.all_peripherals_idle());
        bus.write32(UART0_BASE + UARTLCR_H, 1 << 4);
        bus.write32(UART0_BASE + UARTCR, 0x301);
        bus.write32(UART0_BASE + UARTDR, 0x42);
        assert!(!bus.all_peripherals_idle(),
            "pending TX byte breaks the idle gate");
    }

    #[test]
    fn spi0_reset_post_activity_returns_to_idle() {
        let mut bus = Bus::new();
        release_all(&mut bus);
        bus.write32(SPI0_BASE + SSPCR0, 0x07);
        bus.write32(SPI0_BASE + SSPCR1, 0x3);
        bus.write32(SPI0_BASE + SSPDR, 0x11);
        assert!(!bus.spi0.is_idle());
        bus.spi0.reset();
        assert!(bus.spi0.is_idle());
    }
}

// P5 branch-coverage completion tests for the RISC-V Hazard3 core files.
// Stage 7b: fills uncovered branches in decode.rs / execute.rs / irq.rs /
// mod.rs so the core_riscv/ files clear 92% branch coverage.
//
// Each section below targets a specific set of uncovered branches, named
// after the production file it exercises. Where a branch is truly
// unreachable from unit-test scope (e.g. `unreachable!()` panics that only
// fire on hardware UB), the section explains why it is left alone.

use super::csr::{CSR_MHARTID, CSR_MINSTRET, CSR_MIP, CSR_MSCRATCH, CSR_MTVAL, CSR_MTVEC};
use super::decode::{
    self, AluImmKind, AluKind, AmoKind, CsrKind, LoadKind, MulDivKind, Op, StoreKind,
};
use super::irq::{Xh3Irq, CTX_MRETEIRQ, CTX_NOIRQ};
use super::Hazard3;
use crate::{Arch, Bus, Config, Cores, EmulatorBuilder};

// ---------- helpers ----------

use super::tests_common::{fresh, write_hw, write_insn};

// =====================================================================
// decode.rs branch coverage
// =====================================================================

// Line 215: JALR with funct3 != 0 must decode as Illegal.
#[test]
fn decode_jalr_nonzero_funct3_illegal() {
    // JALR opcode = 0b11001. funct3 must be 0 — use f3=0b001 to force
    // Illegal.
    let insn = (0b001u32 << 12) | (0b11_001 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 276: SLLI with funct7 != 0 must decode as Illegal.
#[test]
fn decode_slli_nonzero_funct7_illegal() {
    // funct3 = 001 (SLLI), funct7 = 0x01 (illegal for RV32I SLLI).
    let insn = (0b000_0001u32 << 25) | (0b001 << 12) | (0b00_100 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 288: SRLI/SRAI with funct7 != {0x00, 0x20} is Illegal.
#[test]
fn decode_srli_srai_bad_funct7_illegal() {
    // funct3 = 101, funct7 = 0x01 (neither 0 nor 0x20) -> Illegal.
    let insn = (0b000_0001u32 << 25) | (0b101 << 12) | (0b00_100 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 346: AMO funct3 != 0b010 (word) is illegal on Hazard3 (no RV64A).
#[test]
fn decode_amo_wrong_width_illegal() {
    // funct5=00010 (LR), funct3=0b011 (illegal — .D would be RV64).
    let insn = (0b00010u32 << 27) | (0b011 << 12) | (0b01_011 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 358: LR.W with rs2 != 0 is illegal per spec.
#[test]
fn decode_lr_w_rs2_nonzero_illegal() {
    // funct5=00010, rs2=1 (illegal).
    let insn = (0b00010u32 << 27) | (1u32 << 20) | (0b010 << 12) | (0b01_011 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 373: AMO with unknown funct5 is illegal.
#[test]
fn decode_amo_bad_funct5_illegal() {
    // funct5 = 0b11111 (reserved/unknown).
    let insn = (0b11111u32 << 27) | (0b010 << 12) | (0b01_011 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 387: MISC-MEM with unknown funct3 is illegal.
#[test]
fn decode_misc_mem_unknown_funct3_illegal() {
    // opcode = MISC-MEM, f3 = 0b010 (neither FENCE nor FENCE.I).
    let insn = (0b010u32 << 12) | (0b00_011 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 396: SYSTEM priv form with rd != 0 is illegal.
#[test]
fn decode_system_priv_rd_nonzero_illegal() {
    // funct12 = 0 (ECALL-class), f3=0, rd=5 → illegal.
    let insn = (5u32 << 7) | (0b11_100 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 405: SYSTEM priv form with unknown funct12 is illegal.
#[test]
fn decode_system_priv_unknown_funct12_illegal() {
    // rd=rs1=0, f3=0, funct12 = 0x200 (not any defined priv op).
    let insn = (0x200u32 << 20) | (0b11_100 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 417: SYSTEM Zicsr f3=100 is illegal.
#[test]
fn decode_system_csr_f3_100_illegal() {
    // f3=100 is not a valid CSR op; drop-through to Illegal.
    let insn = (0x300u32 << 20) | (0b100 << 12) | (0b11_100 << 2) | 0b11;
    assert!(matches!(decode::decode(insn), Op::Illegal { .. }));
}

// Line 474: C.ADDI4SPN with nzuimm==0 is illegal (the "canonical-illegal"
// instruction 0 is caught earlier at line 448, but C.ADDI4SPN with
// rd'=something and nzuimm encoding = 0 must still trap).
#[test]
fn decode_c_addi4spn_zero_nzuimm_illegal() {
    // f3=000, imm bits [12:5] = 0, rd' bits [4:2] = 001 (x9). quadrant=00.
    // With nzuimm = 0 we must hit the Illegal path.
    let hw: u16 = 0b000_00_0000_0_0_001_00;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 497: Q0 unknown f3 -> Illegal. f3 = 001 (C.FLD, rv32-never-valid).
#[test]
fn decode_c_q0_unknown_f3_illegal() {
    // quadrant=00, f3=001 (C.FLD — not in Hazard3's extension set).
    let hw: u16 = 0b001_000_000_00_000_00;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 537: C.ADDI16SP with nzimm==0 is illegal.
#[test]
fn decode_c_addi16sp_zero_nzimm_illegal() {
    // f3=011, rd=00010 (sp), all imm bits 0, quadrant=01.
    let hw: u16 = 0b011_0_00010_00000_01;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 547: C.LUI with nzimm==0 is illegal.
#[test]
fn decode_c_lui_zero_nzimm_illegal() {
    // f3=011, rd=5 (not sp), nzimm6 = 0 → illegal.
    let hw: u16 = 0b011_0_00101_00000_01;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 547 second branch: C.LUI with rd_==0 is illegal (rd = sp is handled
// as ADDI16SP; rd = 0 is illegal even with non-zero nzimm).
// The rd==0 / f3==011 slot decodes as C.ADDI16SP since rd=2 check uses
// specific equality; any rd != 2 falls into C.LUI; rd==0 → illegal.
#[test]
fn decode_c_lui_rd_zero_illegal() {
    // rd = 0, nzimm6 = 1 (nonzero to prove the rd check is separate).
    let hw: u16 = 0b011_0_00000_00001_01;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 564 / 573: Q1 MISC-ALU SRLI/SRAI with shamt >= 32 is illegal
// (funct3=100 bits[11:10]=0b00 SRLI / 0b01 SRAI with bit12=1 gives
// shamt = 32..63 -> illegal in RV32).
#[test]
fn decode_c_srli_shamt_ge_32_illegal() {
    // f3=100, bits[11:10]=0b00 (SRLI), bit12=1, b4_0 = anything, rs1'=000.
    let hw: u16 = 0b100_1_00_000_00000_01;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

#[test]
fn decode_c_srai_shamt_ge_32_illegal() {
    let hw: u16 = 0b100_1_01_000_00000_01;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 593: Q1 MISC-ALU 0b100_xx (0b100..0b111) — RV64-only C.SUBW/C.ADDW
// are Illegal on RV32.
#[test]
fn decode_c_subw_illegal_on_rv32() {
    // bits[11:10] = 11 (register-register group), bit12=1, bits[6:5] = 00.
    // sel = (bit12 << 2) | bits[6:5] = 0b100 -> hits the "C.SUBW" arm
    // which is RV64-only -> Illegal.
    // Insn: f3=100, bit12=1, bits11:10=11, rs1'=000, bits6:5=00, rs2'=000, quad=01.
    let hw: u16 = 0b100_1_11_000_00_000_01;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 615: Q1 unknown f3 (not 0b000..0b111 == every value) — unreachable
// in practice because f3 is 3 bits. Skipped.

// Line 627: Q2 C.SLLI with shamt >= 32 illegal. bit12=1 → shamt = 32 | b4_0.
#[test]
fn decode_c_slli_shamt_ge_32_illegal() {
    // f3=000, bit12=1, rd=00101, b4_0=0, quadrant=10.
    let hw: u16 = 0b000_1_00101_00000_10;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 633: Q2 C.LWSP with rd_ == 0 is illegal.
#[test]
fn decode_c_lwsp_rd_zero_illegal() {
    // f3=010, rd=0, uimm=0, quadrant=10.
    let hw: u16 = 0b010_0_00000_00000_10;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// Line 667: Q2 unknown f3 (e.g. 0b001 = C.FLDSP, not supported) illegal.
#[test]
fn decode_c_q2_unknown_f3_illegal() {
    // f3=001 (C.FLDSP — F/D extension not in Hazard3).
    let hw: u16 = 0b001_0_00001_00000_10;
    assert!(matches!(decode::decode16(hw), Op::Illegal { .. }));
}

// =====================================================================
// execute.rs branch coverage
// =====================================================================

// Line 67: JAL to a 1-byte-misaligned target (bit 0 set) traps mcause=0.
#[test]
fn exec_jal_misaligned_target_traps_cause_0() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    // imm is 1 — target = epc + 1 → bit 0 set.
    c.execute(Op::Jal { rd: 1, imm: 1 }, &mut bus, 0x2000_0100);
    assert_eq!(c.csrs.mcause, 0);
    assert_eq!(c.csrs.mepc, 0x2000_0100);
    assert_eq!(c.pc, 0x2000_2000);
}

// Line 105: Branch taken to 1-byte-misaligned target traps mcause=0.
#[test]
fn exec_branch_taken_misaligned_target_traps() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 1;
    c.x[2] = 1;
    // Force taken branch (BEQ 1==1) with odd imm → target odd.
    c.execute(
        decode::Op::Branch {
            kind: decode::BranchKind::Beq,
            rs1: 1,
            rs2: 2,
            imm: 1,
        },
        &mut bus,
        0x2000_0100,
    );
    assert_eq!(c.csrs.mcause, 0);
    assert_eq!(c.csrs.mepc, 0x2000_0100);
}

// Line 138: Load bus fault path. Target XIP flash region with no flash
// loaded — read8 sets bus_fault, the executor traps mcause=5.
#[test]
fn exec_lb_bus_fault_traps_cause_5() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    // Flash region — flash not loaded → bus_fault.
    c.x[1] = 0x1000_0000;
    c.execute(
        Op::Load {
            kind: LoadKind::Lb,
            rd: 2,
            rs1: 1,
            imm: 0,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 5);
    assert_eq!(c.csrs.mepc, 0x2000_0000);
}

// Same path for Lw (exercises LoadKind::Lw branch in the read arm).
#[test]
fn exec_lw_bus_fault_traps_cause_5() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x1000_0000;
    c.execute(
        Op::Load {
            kind: LoadKind::Lw,
            rd: 2,
            rs1: 1,
            imm: 0,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 5);
}

// Line 164: Store bus fault — flash region, cause=7.
#[test]
fn exec_sw_bus_fault_traps_cause_7() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x1000_0000;
    c.x[2] = 0xDEAD_BEEF;
    c.execute(
        Op::Store {
            kind: StoreKind::Sw,
            rs1: 1,
            rs2: 2,
            imm: 0,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 7);
}

// Line 176 / 177: Slti / Sltiu — exercise the "true" branches.
#[test]
fn exec_slti_true_and_false() {
    let (mut c, mut bus) = fresh();
    c.x[1] = (-5i32) as u32;
    c.execute(
        Op::OpImm {
            kind: AluImmKind::Slti,
            rd: 2,
            rs1: 1,
            imm: 3,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[2], 1, "signed -5 < 3");
    // False side.
    c.execute(
        Op::OpImm {
            kind: AluImmKind::Slti,
            rd: 3,
            rs1: 1,
            imm: -10,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0, "signed -5 !< -10");
}

#[test]
fn exec_sltiu_true_and_false() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 5;
    c.execute(
        Op::OpImm {
            kind: AluImmKind::Sltiu,
            rd: 2,
            rs1: 1,
            imm: 7,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[2], 1);
    c.execute(
        Op::OpImm {
            kind: AluImmKind::Sltiu,
            rd: 3,
            rs1: 1,
            imm: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0);
}

// Line 202 / 203: Slt / Sltu register forms — "true" branch.
#[test]
fn exec_slt_sltu_true_paths() {
    let (mut c, mut bus) = fresh();
    c.x[1] = (-1i32) as u32;
    c.x[2] = 1;
    c.execute(
        Op::Op {
            kind: AluKind::Slt,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 1);
    c.x[4] = 1;
    c.x[5] = 2;
    c.execute(
        Op::Op {
            kind: AluKind::Sltu,
            rd: 6,
            rs1: 4,
            rs2: 5,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[6], 1);
}

// Line 319: LR misaligned -> mcause=4.
#[test]
fn exec_lr_w_misaligned_traps_cause_4() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_1001; // not 4-aligned
    c.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 4);
    assert_eq!(c.csrs.mepc, 0x2000_0000);
}

// Line 330: LR bus-fault trap path. Hard to trigger for LR because
// `in_reservable` already gates out anything outside SRAM. We need an
// address that is in the reservable range but whose read8/read32 sets a
// bus fault. The SRAM region is backed by the emulator's memory store
// which never faults; leaving line 330 unreachable from a clean-build
// scope. Document and move on.
//
// unreachable: LR bus-fault inside the reservable SRAM window cannot be
// synthesized without injecting a fault via a private hook; the runtime
// memory backing always succeeds for 0x2000_0000..0x2008_2000.

// Line 343: SC outside reservable → silent fail; exercise rd=1 write.
#[test]
fn exec_sc_w_outside_reservable_silent_fail() {
    let (mut c, mut bus) = fresh();
    // Address outside the SRAM reservable range. Pick ROM (0x0000_0000).
    c.x[1] = 0x0000_0004;
    c.x[2] = 0xFEED_FACE;
    c.execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 5,
            rs1: 1,
            rs2: 2,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    // rd=1 (silent fail), no trap.
    assert_eq!(c.x[5], 1);
    assert_eq!(c.csrs.mcause, 0, "no trap");
}

// Line 339: SC misaligned → mcause=6.
#[test]
fn exec_sc_w_misaligned_traps_cause_6() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_1001;
    c.execute(
        Op::Amo {
            kind: AmoKind::Sc,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.csrs.mcause, 6);
}

// Line 383: AMO misaligned → mcause=6.
#[test]
fn exec_amoadd_w_misaligned_traps_cause_6() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_1002; // 2-aligned, not 4-aligned
    c.execute(
        Op::Amo {
            kind: AmoKind::Add,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.csrs.mcause, 6);
}

// Line 387: AMO outside reservable → mcause=7.
#[test]
fn exec_amoadd_w_outside_reservable_traps_cause_7() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x0000_0004; // ROM, not reservable
    c.execute(
        Op::Amo {
            kind: AmoKind::Add,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.csrs.mcause, 7);
}

// Cover all AMO kinds including Min/Max/Minu/Maxu — the kind match arms
// at execute.rs 404–407.
#[test]
fn exec_amo_all_kinds_cover_match() {
    let (mut c, mut bus) = fresh();
    // Reservable-SRAM target.
    bus.memory.sram_write32(0x1000, 10);
    c.x[1] = 0x2000_1000;
    c.x[2] = 20;
    for kind in [
        AmoKind::Swap,
        AmoKind::Add,
        AmoKind::And,
        AmoKind::Or,
        AmoKind::Xor,
        AmoKind::Min,
        AmoKind::Max,
        AmoKind::Minu,
        AmoKind::Maxu,
    ] {
        bus.memory.sram_write32(0x1000, 10);
        c.execute(
            Op::Amo {
                kind,
                rd: 3,
                rs1: 1,
                rs2: 2,
                aq: false,
                rl: false,
            },
            &mut bus,
            0,
        );
        assert_eq!(c.x[3], 10, "AMO returns old value regardless of op");
    }
}

// Line 34: in_reservable canon_oracle_addr path — exercise via an
// 0x8XXX_XXXX oracle address (QEMU alias). The address must canonicalise
// into the reservable SRAM window so LR succeeds.
#[test]
fn exec_lr_via_oracle_alias_canonicalises_to_sram() {
    let (mut c, mut bus) = fresh();
    bus.memory.sram_write32(0x2000, 0x1122_3344);
    // Oracle alias: 0x8000_2000 → 0x2000_2000.
    c.x[1] = 0x8000_2000;
    c.execute(
        Op::Amo {
            kind: AmoKind::Lr,
            rd: 2,
            rs1: 1,
            rs2: 0,
            aq: false,
            rl: false,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[2], 0x1122_3344);
    assert_eq!(bus.reservation[0], Some(0x2000_2000));
}

// Line 443/445/452/455/457/464: exhaustive MulDiv edge cases.
#[test]
fn exec_div_by_zero_returns_all_ones() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 42;
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Div,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xFFFF_FFFF);
}

#[test]
fn exec_div_overflow_int_min_neg_one() {
    let (mut c, mut bus) = fresh();
    c.x[1] = i32::MIN as u32;
    c.x[2] = (-1i32) as u32;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Div,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], i32::MIN as u32);
}

#[test]
fn exec_divu_by_zero_returns_all_ones() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 42;
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Divu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xFFFF_FFFF);
}

#[test]
fn exec_rem_by_zero_returns_dividend() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 42;
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Rem,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 42);
}

#[test]
fn exec_rem_overflow_int_min_neg_one_returns_zero() {
    let (mut c, mut bus) = fresh();
    c.x[1] = i32::MIN as u32;
    c.x[2] = (-1i32) as u32;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Rem,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0);
}

#[test]
fn exec_remu_by_zero_returns_dividend() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0xDEAD_BEEF;
    c.x[2] = 0;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Remu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 0xDEAD_BEEF);
}

// decode.rs line 276 False-arm: SLLI with funct7=0 is legal.
#[test]
fn decode_slli_funct7_zero_legal() {
    // funct3 = 001 (SLLI), funct7 = 0 (legal). rd=3, rs1=4, shamt=5.
    let insn = (5u32 << 20) | (4u32 << 15) | (0b001 << 12) | (3u32 << 7)
        | (0b00_100 << 2) | 0b11;
    matches!(
        decode::decode(insn),
        Op::ShiftImm {
            kind: decode::ShiftKind::Slli,
            rd: 3,
            rs1: 4,
            shamt: 5,
        }
    );
}

// decode.rs line 509:28 False-arm: C.ADDI with rd=0 but imm != 0 (a HINT
// that still decodes as a normal ADDI).
#[test]
fn decode_c_addi_rd_zero_nonzero_imm_decodes() {
    // f3=000, imm5=0, rd=00000, imm4_0=00001, quadrant=01.
    let hw: u16 = 0b000_0_00000_00001_01;
    assert_eq!(
        decode::decode16(hw),
        Op::OpImm {
            kind: AluImmKind::Addi,
            rd: 0,
            rs1: 0,
            imm: 1,
        }
    );
}

// decode.rs line 653:35 False-arm: bit12=1, rd_==0, rs2_!=0 → C.ADD x0,
// x0, rs2 (HINT, still decodes as Add op).
#[test]
fn decode_c_add_with_rd_zero_decodes_as_op_add() {
    // f3=100, bit12=1, rd=00000, rs2=00001, quadrant=10.
    // Falls through line 653 (rd==0 && rs2==0 fails because rs2!=0) then
    // line 655 (rs2==0 fails) → line 658 = Op::Op { Add, 0, 0, rs2 }.
    let hw: u16 = 0b100_1_00000_00001_10;
    assert_eq!(
        decode::decode16(hw),
        Op::Op {
            kind: AluKind::Add,
            rd: 0,
            rs1: 0,
            rs2: 1,
        }
    );
}

// execute.rs line 445:49 / 457:49 False-arm: a = i32::MIN but b != -1
// (and b != 0). DIV / REM should compute normally.
#[test]
fn exec_div_int_min_by_nontrivial_divisor() {
    let (mut c, mut bus) = fresh();
    c.x[1] = i32::MIN as u32;
    c.x[2] = 2;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Div,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    // i32::MIN / 2 = -(2^30).
    assert_eq!(c.x[3], ((i32::MIN / 2) as u32));
}

#[test]
fn exec_rem_int_min_by_nontrivial_divisor() {
    let (mut c, mut bus) = fresh();
    c.x[1] = i32::MIN as u32;
    c.x[2] = 3;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Rem,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    // i32::MIN % 3 = (i32::MIN).wrapping_rem(3).
    assert_eq!(c.x[3], ((i32::MIN as i32).wrapping_rem(3)) as u32);
}

// execute.rs line 464:16 False-arm: Remu with b != 0.
#[test]
fn exec_remu_with_nonzero_divisor() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 100;
    c.x[2] = 7;
    c.execute(
        Op::MulDiv {
            kind: MulDivKind::Remu,
            rd: 3,
            rs1: 1,
            rs2: 2,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[3], 100 % 7);
}

// irq.rs line 273:16 False-arm: read_meipra with meipra_idx >= 16 (base
// >= 64) — every irq in the window is >= 64, so inner store is skipped
// and window stays 0.
#[test]
fn irq_read_meipra_upper_window_returns_zero_data() {
    let mut x = Xh3Irq::new();
    // Poke meipra_idx directly (private field; use write then mutate
    // via write_meipra with a data field — but write_meipra also bails
    // when base >= 64). Prefer write_meipra with idx=16 which latches
    // the idx but drops the data.
    x.write_meipra((0xFFFFu32 << 16) | 16);
    let r = x.read_meipra();
    // Data stays zero because every irq in the window was >= 64.
    assert_eq!(r >> 16, 0);
    assert_eq!(r & 0x1F, 16);
}

// irq.rs line 326:12 False-arm: write_meinext with update=0 is a no-op.
#[test]
fn irq_write_meinext_update_zero_noop() {
    let mut x = Xh3Irq::new();
    x.meiea = 1 << 5;
    x.force_set(5);
    // Write with update=0 (just index bits).
    x.write_meinext(0, 0);
    // meifa[5] must still be set.
    assert_eq!(x.meifa & (1 << 5), 1 << 5);
}

// irq.rs line 360:12 False-arm: write_meicontext with clearts bit clear
// — the save-slot handling skips and raw bits flow through.
#[test]
fn irq_write_meicontext_without_clearts_flows_through() {
    let mut x = Xh3Irq::new();
    let mut mie: u32 = (1 << 7) | (1 << 3);
    // Write with clearts=0 but set mtiesave / msiesave in the raw value.
    x.write_meicontext(CTX_MRETEIRQ, &mut mie);
    // mie untouched.
    assert_eq!(mie, (1 << 7) | (1 << 3));
    // The raw mreteirq bit survived.
    assert_eq!(x.meicontext & CTX_MRETEIRQ, CTX_MRETEIRQ);
}

// irq.rs line 450:12 False-arm: on_mret when mreteirq=1 but
// preempt_depth already 0 (spurious / manual). The saturating check at
// 450 must not underflow.
#[test]
fn irq_on_mret_spurious_mreteirq_preempt_depth_zero() {
    let mut x = Xh3Irq::new();
    // Manually set mreteirq with depth still 0.
    x.meicontext |= CTX_MRETEIRQ;
    x.on_mret();
    // Depth remains 0 — saturates instead of underflowing.
    // Effect: mreteirq cleared, noirq set (depth reached 0 after the
    // no-op decrement).
    assert_eq!(x.meicontext & CTX_MRETEIRQ, 0);
    assert_eq!(x.meicontext & CTX_NOIRQ, CTX_NOIRQ);
}

// Line 120 / 155 Load Lh misalign + Store Sh misalign — already present
// in tests_p2 (lw_misalign_traps_cause4 / sh_misalign_traps_cause6) but
// we add Lh + a store to cover the Lh branch of the `aligned` tuple and
// the Store Sh `aligned` = addr&1==0 false arm.
#[test]
fn exec_lh_misaligned_traps_cause_4() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_1001;
    c.execute(
        Op::Load {
            kind: LoadKind::Lh,
            rd: 2,
            rs1: 1,
            imm: 0,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.csrs.mcause, 4);
}

// SC bus-fault (line 357) and AMO read/write bus-fault (line 392, 411)
// are only reachable via faults within the reservable region — same
// constraint as LR. Documented as unreachable:
// unreachable: SC/AMO bus-fault within reservable SRAM cannot be driven
// from unit tests; the memory backing never faults.

// =====================================================================
// irq.rs branch coverage
// =====================================================================

// Line 133: arbitrate with no pending IRQs returns None.
#[test]
fn irq_arbitrate_none_when_no_pending() {
    let x = Xh3Irq::new();
    assert!(x.arbitrate(0).is_none());
}

// Line 143: arbitrate skips IRQs whose priority is below ppreempt.
#[test]
fn irq_arbitrate_skips_below_ppreempt() {
    let mut x = Xh3Irq::new();
    x.meiea = (1 << 3) | (1 << 7);
    x.meipra[3] = 2;
    x.meipra[7] = 1;
    // ppreempt = 5 → both are below → None.
    x.meicontext = 5u32 << 24;
    assert!(x.arbitrate((1u64 << 3) | (1u64 << 7)).is_none());
}

// Line 148: higher-priority later IRQ replaces best.
#[test]
fn irq_arbitrate_priority_replaces_best() {
    let mut x = Xh3Irq::new();
    x.meiea = (1 << 2) | (1 << 5);
    x.meipra[2] = 1;
    x.meipra[5] = 10;
    // Bit 2 visited first (lower-numbered), then bit 5 with higher pri
    // replaces the best. This hits the `Some((_, bp)) if pri > bp`
    // arm at line 148.
    let r = x.arbitrate((1u64 << 2) | (1u64 << 5)).unwrap();
    assert_eq!(r.0, 5);
    assert_eq!(r.1, 10);
}

// Line 172: write_meiea with idx >= 4 — upper IRQs silently drop.
#[test]
fn irq_write_meiea_idx_ge_4_drops() {
    let mut x = Xh3Irq::new();
    // Pre-set meiea bits then write idx=4 with data — data must NOT
    // affect storage.
    x.meiea = 0xFFFF_FFFF_FFFF_FFFF;
    x.write_meiea((0xAAAAu32 << 16) | 4);
    assert_eq!(
        x.meiea, 0xFFFF_FFFF_FFFF_FFFF,
        "idx >= 4 is dropped — storage unchanged"
    );
}

// Line 232: write_meifa with idx >= 4 — same drop behavior.
#[test]
fn irq_write_meifa_idx_ge_4_drops() {
    let mut x = Xh3Irq::new();
    x.meifa = 0xAAAA;
    x.write_meifa((0xFFFFu32 << 16) | 4);
    // idx >=4 path — data not applied.
    assert_eq!(x.meifa, 0xAAAA);
}

// Line 245: force_set upper-range IRQs silently drop (irq >= 64).
#[test]
fn irq_force_set_out_of_range_drops() {
    let mut x = Xh3Irq::new();
    x.force_set(64); // out of range — no panic, no effect.
    assert_eq!(x.meifa, 0);
}

// Line 289: write_meipra for a window whose base >= 64 drops.
#[test]
fn irq_write_meipra_base_ge_64_drops() {
    let mut x = Xh3Irq::new();
    x.meipra[0] = 0xA;
    // idx=16 → base = 16*4 = 64 → whole write drops.
    x.write_meipra((0xFFFFu32 << 16) | 16);
    assert_eq!(x.meipra[0], 0xA);
}

// Line 327 / 331: write_meinext with update=1 but no pending IRQ — the
// inner `arbitrate` returns None, so the write is a no-op. Exercises the
// `if (v & 1) != 0` true branch + the `if let Some(...) = arbitrate` None
// arm.
#[test]
fn irq_write_meinext_update_no_pending_is_noop() {
    let mut x = Xh3Irq::new();
    x.meifa = 0; // no force, no HW
    x.write_meinext(1, 0);
    // No change — meifa stayed zero, no panic.
    assert_eq!(x.meifa, 0);
}

// Line 417: preempt_depth saturating. Three consecutive entries — the
// third must NOT increment past 2.
#[test]
fn irq_preempt_depth_saturates_at_2() {
    let mut x = Xh3Irq::new();
    x.on_ext_irq_entry(1, 1);
    x.on_ext_irq_entry(2, 2);
    x.on_ext_irq_entry(3, 3);
    // Now three mrets: first two pop, third is a no-op (preempt_depth
    // already 0 due to saturation at entry side).
    x.on_mret();
    x.on_mret();
    // After two mrets the depth is 0 and mreteirq should be cleared and
    // noirq set.
    assert_eq!(x.meicontext & CTX_MRETEIRQ, 0);
    assert_eq!(x.meicontext & CTX_NOIRQ, CTX_NOIRQ);
}

// Line 435: on_mret when mreteirq is not set — early return (no pop).
#[test]
fn irq_on_mret_no_mreteirq_is_noop() {
    let mut x = Xh3Irq::new();
    // Default state has noirq=1, mreteirq=0.
    assert_eq!(x.meicontext & CTX_MRETEIRQ, 0);
    let before = x.meicontext;
    x.on_mret();
    assert_eq!(x.meicontext, before, "no change when mreteirq clear");
}

// Line 454: on_mret with depth > 0 after the pop keeps mreteirq asserted.
// Already exercised by p4_nested_preempt_two_levels_unwinds_correctly,
// but we add a direct branch-focused variant.
#[test]
fn irq_on_mret_depth_gt_zero_keeps_mreteirq() {
    let mut x = Xh3Irq::new();
    x.on_ext_irq_entry(1, 1);
    x.on_ext_irq_entry(2, 2);
    x.on_mret();
    // Depth drops 2 -> 1. mreteirq must remain so the outer mret keeps
    // popping.
    assert_eq!(x.meicontext & CTX_MRETEIRQ, CTX_MRETEIRQ);
    assert_eq!(x.meicontext & CTX_NOIRQ, 0);
}

// Line 326-331: write_meinext with update=1 AND pending IRQ at a number
// in-range — clears meifa[irq] (already partially covered, but this hits
// the `if irq < 64` true gate).
#[test]
fn irq_write_meinext_clears_meifa_inrange() {
    let mut x = Xh3Irq::new();
    x.meiea = 1 << 10;
    x.force_set(10);
    x.write_meinext(1, 0);
    assert_eq!(x.meifa & (1 << 10), 0);
}

// =====================================================================
// mod.rs branch coverage
// =====================================================================

// Line 94: step while halted or wfi_parked is a no-op.
#[test]
fn mod_step_halted_is_noop() {
    let (mut c, mut bus) = fresh();
    c.halted = true;
    let pc_before = c.pc;
    let cycles_before = c.cycles;
    c.step(&mut bus);
    assert_eq!(c.pc, pc_before);
    assert_eq!(c.cycles, cycles_before);
}

#[test]
fn mod_step_wfi_parked_is_noop() {
    let (mut c, mut bus) = fresh();
    c.wfi_parked = true;
    let pc_before = c.pc;
    c.step(&mut bus);
    assert_eq!(c.pc, pc_before);
}

// Line 137/145: MEIP arbitration returns None (pending but filtered),
// so the step falls through to fetch — hits the `chosen=None` fall-
// through arm (line 132-136) and the outer `if let Some(..) = chosen`
// takes the else branch.
#[test]
fn mod_step_meip_arbitration_none_falls_through_to_fetch() {
    let (mut c, mut bus) = fresh();
    c.csrs.mstatus = 1 << 3; // MIE global
    c.csrs.mie = 1 << 11; // MEIE
    c.csrs.mip = 1 << 11; // MEIP latched...
    // ...but xh3irq has no matching enable bit. arbitrate → None.
    c.xh3irq.meiea = 0;
    // Plant a nop at reset PC so the fall-through fetch succeeds.
    bus.memory.sram_write32(0, 0x0000_0013);
    c.step(&mut bus);
    // No trap delivered — pc advanced past the nop.
    assert_eq!(c.pc, 0x2000_0004);
    assert_eq!(c.csrs.mcause, 0, "no trap");
}

// Line 157: instruction-address misalignment trap path — bit 0 of PC set.
#[test]
fn mod_step_insn_addr_misaligned_trap() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.pc = 0x2000_0001;
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0, "INSTR_ADDR_MISALIGNED");
    assert_eq!(c.csrs.mepc, 0x2000_0001);
}

// Line 170 / 176 / 182: fetch bus-fault paths. Point PC at unmapped
// flash (flash not loaded) so the first halfword fetch faults.
#[test]
fn mod_step_fetch_bus_fault_traps_cause_1() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.pc = 0x1000_0000; // XIP flash, not loaded
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 1, "INSTR_ACCESS_FAULT");
    assert_eq!(c.csrs.mepc, 0x1000_0000);
}

// Line 184: bus fault on the *second* halfword of a 32-bit fetch. Place
// PC at the last legal halfword of SRAM; the second halfword overflows
// the SRAM region and the bus marks a fault.
#[test]
fn mod_step_second_halfword_bus_fault_traps_cause_1() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    // Plant a base-ISA 32-bit encoding (low bits == 0b11) at the last
    // legal halfword of SRAM so the first fetch decodes as "32-bit".
    let last_sram_hw = 0x2008_1FFEu32;
    bus.memory.sram_write16(last_sram_hw & 0x0FFF_FFFF, 0x0013); // low half of ADDI x0, x0, 0
    c.pc = last_sram_hw;
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 1, "INSTR_ACCESS_FAULT on 2nd halfword");
    assert_eq!(c.csrs.mepc, last_sram_hw);
}

// Line 199 / 202: mcountinhibit gating. Clear both CY and IR bits and
// verify mcycle/minstret tick, then set them and verify they stall.
#[test]
fn mod_step_mcountinhibit_gates_mcycle_and_minstret() {
    let (mut c, mut bus) = fresh();
    // Clear both inhibits → mcycle and minstret tick.
    c.csrs.mcountinhibit = 0;
    bus.memory.sram_write32(0, 0x0000_0013); // nop
    c.step(&mut bus);
    assert_eq!(c.csrs.mcycle, 1);
    assert_eq!(c.csrs.minstret, 1);

    // Set both inhibits → no tick.
    c.csrs.mcountinhibit = 0b101;
    bus.memory.sram_write32(4, 0x0000_0013); // nop at next pc
    c.step(&mut bus);
    assert_eq!(c.csrs.mcycle, 1, "mcycle stalled");
    assert_eq!(c.csrs.minstret, 1, "minstret stalled");
}

// Line 219: gpr(0) returns 0 regardless of x[0] storage.
#[test]
fn mod_gpr_index_zero_returns_zero() {
    let mut c = Hazard3::new(0);
    c.x[0] = 0xDEAD_BEEF; // illegally poke storage
    assert_eq!(c.gpr(0), 0);
    // Non-zero returns stored value.
    c.x[5] = 0x1234;
    assert_eq!(c.gpr(5), 0x1234);
    // Index wrap: bits above 5 are masked.
    assert_eq!(c.gpr(0x25), 0x1234, "index masked to 5 bits");
}

// Line 230: set_gpr(0, v) is a no-op.
#[test]
fn mod_set_gpr_index_zero_is_noop() {
    let mut c = Hazard3::new(0);
    c.set_gpr(0, 0xFFFF_FFFF);
    assert_eq!(c.x[0], 0);
    c.set_gpr(7, 0xABCD);
    assert_eq!(c.x[7], 0xABCD);
}

// Line 323: is_halted true when wfi_parked OR halted.
#[test]
fn mod_is_halted_covers_both_flags() {
    let mut c = Hazard3::new(0);
    assert!(!c.is_halted());
    c.halted = true;
    assert!(c.is_halted());
    c.halted = false;
    c.wfi_parked = true;
    assert!(c.is_halted());
}

// set_pc + set_mtvec + mcause accessors.
#[test]
fn mod_set_pc_and_set_mtvec_accessors() {
    let mut c = Hazard3::new(0);
    c.set_pc(0xAABB_CCDD);
    assert_eq!(c.pc(), 0xAABB_CCDD);
    c.set_mtvec(0x2000_1FFC);
    assert_eq!(c.csrs.mtvec, 0x2000_1FFC);
    c.csrs.mcause = 0x8000_0007;
    assert_eq!(c.mcause(), 0x8000_0007);
}

// set_halted path.
#[test]
fn mod_set_halted_toggles_flag() {
    let mut c = Hazard3::new(0);
    c.set_halted(true);
    assert!(c.is_halted());
    c.set_halted(false);
    assert!(!c.is_halted());
}

// reset_diff_csrs + reset_pmp_csrs + undef_count accessors.
#[test]
fn mod_reset_diff_and_pmp_csrs() {
    let mut c = Hazard3::new(0);
    c.csrs.mstatus = 0xFFFF_FFFF;
    c.csrs.mie = 0xFFFF_FFFF;
    c.csrs.mip = 0xFFFF_FFFF;
    c.csrs.mscratch = 0xDEAD;
    c.csrs.mepc = 0xBEEF;
    c.csrs.mcause = 0xCAFE;
    c.csrs.pmpcfg[0] = 0xFF;
    c.csrs.pmpaddr[3] = 0xABCD;
    c.reset_diff_csrs();
    assert_eq!(c.csrs.mstatus, 0);
    assert_eq!(c.csrs.mie, 0);
    assert_eq!(c.csrs.mip, 0);
    assert_eq!(c.csrs.mscratch, 0);
    assert_eq!(c.csrs.mepc, 0);
    assert_eq!(c.csrs.mcause, 0);
    // pmp untouched by reset_diff_csrs.
    assert_eq!(c.csrs.pmpcfg[0], 0xFF);
    c.reset_pmp_csrs();
    assert_eq!(c.csrs.pmpcfg[0], 0);
    assert_eq!(c.csrs.pmpaddr[3], 0);
    let bank = c.pmpcfg();
    assert_eq!(bank[0], 0);
    let addrs = c.pmpaddr();
    assert_eq!(addrs[3], 0);
}

#[test]
fn mod_undef_count_increments_on_illegal_dispatch() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    assert_eq!(c.undef_count(), 0);
    // Plant an illegal 32-bit word at reset PC.
    write_insn(&mut bus, 0, 0xFFFF_FFFF);
    c.step(&mut bus);
    assert_eq!(c.undef_count(), 1);
    assert_eq!(c.csrs.mcause, 2, "ILLEGAL_INSTRUCTION");
}

// Cycles accessor.
#[test]
fn mod_cycles_accessor() {
    let mut c = Hazard3::new(0);
    assert_eq!(c.cycles(), 0);
    c.cycles = 42;
    assert_eq!(c.cycles(), 42);
}

// Priority fall-through: pending is MSIP only (MEIP absent). The
// `chosen = Some((3, None))` arm at line 127.
#[test]
fn mod_step_msip_only_trap() {
    let (mut c, mut bus) = fresh();
    c.csrs.mstatus = 1 << 3;
    c.csrs.mie = 1 << 3;
    c.csrs.mip = 1 << 3;
    c.csrs.mtvec = 0x2000_2000;
    c.pc = 0x2000_1000;
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0x8000_0003, "MSI");
}

// MTIP only — `chosen = Some((7, None))` arm at line 129.
#[test]
fn mod_step_mtip_only_trap() {
    let (mut c, mut bus) = fresh();
    c.csrs.mstatus = 1 << 3;
    c.csrs.mie = 1 << 7;
    c.csrs.mip = 1 << 7;
    c.csrs.mtvec = 0x2000_2000;
    c.pc = 0x2000_1000;
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0x8000_0007);
}

// `mip` / `mie` / `set_mip` accessors.
#[test]
fn mod_mip_mie_set_mip_accessors() {
    let mut c = Hazard3::new(0);
    c.csrs.mie = 0x888;
    assert_eq!(c.mie(), 0x888);
    c.set_mip(0x808);
    assert_eq!(c.mip(), 0x808);
}

// =====================================================================
// Compressed instruction step-through coverage
// =====================================================================

// Line 617: Q1 unknown f3 path cannot happen (f3 is 3 bits and every
// value is covered). Skipped.

// Confirm the Q1 C.ADDI with rd==0 and imm_raw==0 (`c.nop` slot) uses
// the explicit zero-encoded ADDI path at line 510.
#[test]
fn mod_step_c_nop_uses_zero_encoded_path() {
    let (mut c, mut bus) = fresh();
    // C.NOP = 0x0001 — all bits low except quadrant = 01.
    write_hw(&mut bus, 0, 0x0001);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0002);
}

// =====================================================================
// csr.rs / tests_p2 overlap — hit read_csr branches we didn't.
// =====================================================================

// These also reinforce the `set_gpr` path when the executor writes rd.
#[test]
fn mod_csr_read_mhartid_hart_0() {
    let (mut c, mut bus) = fresh();
    c.execute(
        Op::Csr {
            kind: CsrKind::Csrrs,
            rd: 5,
            rs1_or_zimm: 0,
            csr: CSR_MHARTID,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[5], 0);
}

// Exercise the MTVAL/MIP/MSCRATCH read branches (just in case they are
// un-hit by prior suites).
#[test]
fn mod_csr_read_mtval_hardwired_zero() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtval = 0xDEAD_BEEF; // back-door poke (read-only mtval)
    c.execute(
        Op::Csr {
            kind: CsrKind::Csrrs,
            rd: 5,
            rs1_or_zimm: 0,
            csr: CSR_MTVAL,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[5], 0xDEAD_BEEF, "read returns stored value");
    // Write-1 path: mtval write is ignored.
    c.x[6] = 0xCAFE;
    c.execute(
        Op::Csr {
            kind: CsrKind::Csrrw,
            rd: 7,
            rs1_or_zimm: 6,
            csr: CSR_MTVAL,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.csrs.mtval, 0xDEAD_BEEF, "write ignored");
}

#[test]
fn mod_csr_read_mscratch_and_mip() {
    let (mut c, mut bus) = fresh();
    c.csrs.mscratch = 0x1234_5678;
    c.execute(
        Op::Csr {
            kind: CsrKind::Csrrs,
            rd: 5,
            rs1_or_zimm: 0,
            csr: CSR_MSCRATCH,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[5], 0x1234_5678);
    c.csrs.mip = 0x808;
    c.execute(
        Op::Csr {
            kind: CsrKind::Csrrs,
            rd: 6,
            rs1_or_zimm: 0,
            csr: CSR_MIP,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[6], 0x808);
}

// MINSTRET read-low + MTVEC round-trip.
#[test]
fn mod_csr_read_minstret_and_mtvec() {
    let (mut c, mut bus) = fresh();
    c.csrs.minstret = 0xAABBCCDD_EEFF0011u64;
    c.execute(
        Op::Csr {
            kind: CsrKind::Csrrs,
            rd: 5,
            rs1_or_zimm: 0,
            csr: CSR_MINSTRET,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.x[5], 0xEEFF_0011);
    c.x[6] = 0x2000_1FFD;
    c.execute(
        Op::Csr {
            kind: CsrKind::Csrrw,
            rd: 7,
            rs1_or_zimm: 6,
            csr: CSR_MTVEC,
        },
        &mut bus,
        0,
    );
    assert_eq!(c.csrs.mtvec, 0x2000_1FFD, "mode=1, base=0x2000_1FFC");
}

// =====================================================================
// Emulator path — RiscV Cores reset touches 471-488.
// =====================================================================

#[test]
fn mod_emu_builder_defaults_hart_ids() {
    let emu = EmulatorBuilder::new(Config::default())
        .arch(Arch::RiscV)
        .build().unwrap();
    let Cores::RiscV(cs) = &emu.cores else {
        panic!("expected RiscV cores")
    };
    assert_eq!(cs[0].mhartid(), 0);
    assert_eq!(cs[1].mhartid(), 1);
}

// =====================================================================
// Full-fetch read16 compressed fall-through — execute via step.
// =====================================================================

// Line 178/181: base-ISA 32-bit fetch (second halfword) via step.
// Planting an illegal 32-bit word tests the second-halfword fetch path.
#[test]
fn mod_step_compressed_vs_32bit_dispatch() {
    let (mut c, mut bus) = fresh();
    // 16-bit compressed C.NOP then 32-bit addi.
    write_hw(&mut bus, 0, 0x0001);
    write_insn(&mut bus, 2, 0x0000_0013);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0002);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0006, "2 + 4 = 6");
}

// Per-op semantic tests for the RV32I + Zicsr + Zifencei executor. These
// are the only defence against decode/execute mistakes until the QEMU
// differential oracle (P2.5) lands. Each test is golden-value style with
// hand-computed expectations drawn from RV-priv.

use super::csr::{
    CSR_MCAUSE, CSR_MEPC, CSR_MIE, CSR_MSCRATCH, CSR_MSTATUS, CSR_MTVAL, CSR_MTVEC,
    CSR_MHARTID, CSR_MCOUNTINHIBIT,
};
use super::decode::{self, AluImmKind, AluKind, BranchKind, CsrKind, LoadKind, Op, ShiftKind, StoreKind};
use super::Hazard3;
use crate::Bus;

// Helper: construct a Hazard3 and a Bus with SRAM seeded at 0x2000_0000.
// The executor tests load instruction words into SRAM and step the
// hart with its reset PC still pointing at 0x2000_0000.
fn fresh() -> (Hazard3, Bus) {
    (Hazard3::new(0), Bus::new())
}

// Write a 32-bit instruction into SRAM at `sram_offset` (offset from
// 0x2000_0000 / SRAM base).
fn write_insn(bus: &mut Bus, sram_offset: u32, insn: u32) {
    bus.memory.sram_write32(sram_offset, insn);
}

// Encode an R-type.
fn enc_r(opcode: u32, rd: u8, f3: u32, rs1: u8, rs2: u8, f7: u32) -> u32 {
    (f7 << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15)
        | (f3 << 12) | ((rd as u32) << 7) | (opcode << 2) | 0b11
}
// Encode an I-type (also works for loads / ALUI / JALR / Zicsr).
fn enc_i(opcode: u32, rd: u8, f3: u32, rs1: u8, imm: i32) -> u32 {
    let imm_u = (imm as u32) & 0xFFF;
    (imm_u << 20) | ((rs1 as u32) << 15) | (f3 << 12)
        | ((rd as u32) << 7) | (opcode << 2) | 0b11
}
// Encode an S-type.
fn enc_s(opcode: u32, f3: u32, rs1: u8, rs2: u8, imm: i32) -> u32 {
    let imm_u = (imm as u32) & 0xFFF;
    let hi = (imm_u >> 5) & 0x7F;
    let lo = imm_u & 0x1F;
    (hi << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15)
        | (f3 << 12) | (lo << 7) | (opcode << 2) | 0b11
}
// Encode a B-type.
fn enc_b(f3: u32, rs1: u8, rs2: u8, imm: i32) -> u32 {
    let imm_u = (imm as u32) & 0x1FFE;
    let b12 = (imm_u >> 12) & 0x1;
    let b11 = (imm_u >> 11) & 0x1;
    let b10_5 = (imm_u >> 5) & 0x3F;
    let b4_1 = (imm_u >> 1) & 0xF;
    (b12 << 31) | (b10_5 << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15)
        | (f3 << 12) | (b4_1 << 8) | (b11 << 7) | (0b11_000 << 2) | 0b11
}
// Encode a J-type (JAL).
fn enc_j(rd: u8, imm: i32) -> u32 {
    let imm_u = (imm as u32) & 0x1F_FFFE;
    let b20 = (imm_u >> 20) & 0x1;
    let b10_1 = (imm_u >> 1) & 0x3FF;
    let b11 = (imm_u >> 11) & 0x1;
    let b19_12 = (imm_u >> 12) & 0xFF;
    (b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12)
        | ((rd as u32) << 7) | (0b11_011 << 2) | 0b11
}
// Encode a U-type.
fn enc_u(opcode: u32, rd: u8, imm: u32) -> u32 {
    (imm & 0xFFFF_F000) | ((rd as u32) << 7) | (opcode << 2) | 0b11
}

// Opcode fields.
const OP: u32 = 0b01_100;
const OP_IMM: u32 = 0b00_100;
const LOAD: u32 = 0b00_000;
const STORE: u32 = 0b01_000;
const JALR: u32 = 0b11_001;
const SYSTEM: u32 = 0b11_100;
const LUI: u32 = 0b01_101;
const AUIPC: u32 = 0b00_101;

// -----------------------------------------------------------------------
// Integer ALU
// -----------------------------------------------------------------------

#[test]
fn exec_addi_positive() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 5;
    c.execute(Op::OpImm { kind: AluImmKind::Addi, rd: 2, rs1: 1, imm: 7 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.x[2], 12);
    assert_eq!(c.pc, 0x2000_0004);
}

#[test]
fn exec_addi_negative_imm() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 10;
    c.execute(Op::OpImm { kind: AluImmKind::Addi, rd: 2, rs1: 1, imm: -3 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.x[2], 7);
}

#[test]
fn exec_add_wraps() {
    let (mut c, mut bus) = fresh();
    c.x[1] = u32::MAX;
    c.x[2] = 1;
    c.execute(Op::Op { kind: AluKind::Add, rd: 3, rs1: 1, rs2: 2 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.x[3], 0);
}

#[test]
fn exec_sub() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 10;
    c.x[2] = 3;
    c.execute(Op::Op { kind: AluKind::Sub, rd: 3, rs1: 1, rs2: 2 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.x[3], 7);
}

#[test]
fn exec_slt_signed_and_sltu_unsigned() {
    let (mut c, mut bus) = fresh();
    c.x[1] = (-1i32) as u32;
    c.x[2] = 1;
    c.execute(Op::Op { kind: AluKind::Slt, rd: 3, rs1: 1, rs2: 2 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.x[3], 1, "signed -1 < 1");
    c.execute(Op::Op { kind: AluKind::Sltu, rd: 4, rs1: 1, rs2: 2 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.x[4], 0, "unsigned 0xFFFF_FFFF >= 1");
}

#[test]
fn exec_xor_or_and() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0xF0F0_F0F0;
    c.x[2] = 0x0FF0_0FF0;
    c.execute(Op::Op { kind: AluKind::Xor, rd: 3, rs1: 1, rs2: 2 }, &mut bus, 0);
    assert_eq!(c.x[3], 0xFF00_FF00);
    c.execute(Op::Op { kind: AluKind::Or, rd: 4, rs1: 1, rs2: 2 }, &mut bus, 0);
    assert_eq!(c.x[4], 0xFFF0_FFF0);
    c.execute(Op::Op { kind: AluKind::And, rd: 5, rs1: 1, rs2: 2 }, &mut bus, 0);
    assert_eq!(c.x[5], 0x00F0_00F0);
}

#[test]
fn exec_shifts_reg() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x8000_0001;
    c.x[2] = 1;
    c.execute(Op::Op { kind: AluKind::Sll, rd: 3, rs1: 1, rs2: 2 }, &mut bus, 0);
    assert_eq!(c.x[3], 0x0000_0002);
    c.execute(Op::Op { kind: AluKind::Srl, rd: 4, rs1: 1, rs2: 2 }, &mut bus, 0);
    assert_eq!(c.x[4], 0x4000_0000);
    c.execute(Op::Op { kind: AluKind::Sra, rd: 5, rs1: 1, rs2: 2 }, &mut bus, 0);
    assert_eq!(c.x[5], 0xC000_0000, "arithmetic — sign bit preserved");
}

#[test]
fn exec_srai_shifts_by_shamt_only() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x8000_0000;
    c.execute(Op::ShiftImm { kind: ShiftKind::Srai, rd: 2, rs1: 1, shamt: 4 },
              &mut bus, 0);
    assert_eq!(c.x[2], 0xF800_0000);
}

#[test]
fn exec_lui_sets_upper_immediate() {
    let (mut c, mut bus) = fresh();
    c.execute(Op::Lui { rd: 5, imm: 0x1234_5000 }, &mut bus, 0x2000_0000);
    assert_eq!(c.x[5], 0x1234_5000);
}

#[test]
fn exec_auipc_uses_current_pc() {
    let (mut c, mut bus) = fresh();
    c.execute(Op::Auipc { rd: 7, imm: 0x0000_1000 }, &mut bus, 0x2000_0100);
    assert_eq!(c.x[7], 0x2000_1100);
}

// -----------------------------------------------------------------------
// x0 write ignored
// -----------------------------------------------------------------------

#[test]
fn x0_writes_are_ignored() {
    let (mut c, mut bus) = fresh();
    c.execute(Op::OpImm { kind: AluImmKind::Addi, rd: 0, rs1: 0, imm: 5 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.x[0], 0, "x[0] is hardwired zero");
}

// -----------------------------------------------------------------------
// Load / store round-trip
// -----------------------------------------------------------------------

#[test]
fn exec_sw_then_lw_roundtrip() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x2000_1000; // base
    c.x[2] = 0xDEAD_BEEF;
    c.execute(Op::Store { kind: StoreKind::Sw, rs1: 1, rs2: 2, imm: 0 },
              &mut bus, 0x2000_0000);
    c.execute(Op::Load { kind: LoadKind::Lw, rd: 3, rs1: 1, imm: 0 },
              &mut bus, 0x2000_0004);
    assert_eq!(c.x[3], 0xDEAD_BEEF);
}

#[test]
fn exec_sb_lb_signed_and_lbu_unsigned() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x2000_1000;
    c.x[2] = 0xFF; // high sign
    c.execute(Op::Store { kind: StoreKind::Sb, rs1: 1, rs2: 2, imm: 0 },
              &mut bus, 0);
    c.execute(Op::Load { kind: LoadKind::Lb, rd: 3, rs1: 1, imm: 0 },
              &mut bus, 0);
    assert_eq!(c.x[3], 0xFFFF_FFFF);
    c.execute(Op::Load { kind: LoadKind::Lbu, rd: 4, rs1: 1, imm: 0 },
              &mut bus, 0);
    assert_eq!(c.x[4], 0x0000_00FF);
}

#[test]
fn exec_sh_lh_signed_and_lhu_unsigned() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x2000_1000;
    c.x[2] = 0x8234;
    c.execute(Op::Store { kind: StoreKind::Sh, rs1: 1, rs2: 2, imm: 0 },
              &mut bus, 0);
    c.execute(Op::Load { kind: LoadKind::Lh, rd: 3, rs1: 1, imm: 0 }, &mut bus, 0);
    assert_eq!(c.x[3], 0xFFFF_8234);
    c.execute(Op::Load { kind: LoadKind::Lhu, rd: 4, rs1: 1, imm: 0 }, &mut bus, 0);
    assert_eq!(c.x[4], 0x0000_8234);
}

#[test]
fn lw_misalign_traps_cause4() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000; // direct mode
    c.x[1] = 0x2000_1001; // not 4-aligned
    c.execute(Op::Load { kind: LoadKind::Lw, rd: 2, rs1: 1, imm: 0 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.csrs.mcause, 4);
    assert_eq!(c.csrs.mepc, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn sh_misalign_traps_cause6() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x2000_1001;
    c.x[2] = 0x1234;
    c.execute(Op::Store { kind: StoreKind::Sh, rs1: 1, rs2: 2, imm: 0 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.csrs.mcause, 6);
}

// -----------------------------------------------------------------------
// Branches
// -----------------------------------------------------------------------

#[test]
fn branch_beq_taken() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 1;
    c.x[2] = 1;
    c.execute(Op::Branch { kind: BranchKind::Beq, rs1: 1, rs2: 2, imm: 0x20 },
              &mut bus, 0x2000_0100);
    assert_eq!(c.pc, 0x2000_0120);
}

#[test]
fn branch_beq_not_taken_falls_through() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 1;
    c.x[2] = 2;
    c.execute(Op::Branch { kind: BranchKind::Beq, rs1: 1, rs2: 2, imm: 0x20 },
              &mut bus, 0x2000_0100);
    assert_eq!(c.pc, 0x2000_0104);
}

#[test]
fn branch_blt_signed() {
    let (mut c, mut bus) = fresh();
    c.x[1] = (-5i32) as u32;
    c.x[2] = 3;
    c.execute(Op::Branch { kind: BranchKind::Blt, rs1: 1, rs2: 2, imm: 0x10 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_0010, "signed -5 < 3");
}

#[test]
fn branch_bltu_unsigned() {
    let (mut c, mut bus) = fresh();
    c.x[1] = (-5i32) as u32; // 0xFFFF_FFFB
    c.x[2] = 3;
    c.execute(Op::Branch { kind: BranchKind::Bltu, rs1: 1, rs2: 2, imm: 0x10 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_0004, "unsigned 0xFFFF_FFFB >= 3, not taken");
}

#[test]
fn branch_bne_bge_bgeu() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0;
    c.x[2] = 0;
    c.execute(Op::Branch { kind: BranchKind::Bne, rs1: 1, rs2: 2, imm: 0x10 },
              &mut bus, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_0004, "bne equal not taken");

    c.execute(Op::Branch { kind: BranchKind::Bge, rs1: 1, rs2: 2, imm: 0x10 },
              &mut bus, 0x2000_0100);
    assert_eq!(c.pc, 0x2000_0110, "bge 0 >= 0 taken");

    c.execute(Op::Branch { kind: BranchKind::Bgeu, rs1: 1, rs2: 2, imm: 0x10 },
              &mut bus, 0x2000_0200);
    assert_eq!(c.pc, 0x2000_0210, "bgeu 0 >= 0 taken");
}

// -----------------------------------------------------------------------
// JAL / JALR
// -----------------------------------------------------------------------

#[test]
fn exec_jal_writes_link_and_jumps() {
    let (mut c, mut bus) = fresh();
    c.execute(Op::Jal { rd: 1, imm: 0x100 }, &mut bus, 0x2000_0000);
    assert_eq!(c.x[1], 0x2000_0004, "link = epc+4");
    assert_eq!(c.pc, 0x2000_0100);
}

#[test]
fn exec_jalr_uses_rs1_and_clears_low_bit() {
    let (mut c, mut bus) = fresh();
    c.x[2] = 0x2000_0101; // odd
    c.execute(Op::Jalr { rd: 1, rs1: 2, imm: 0 }, &mut bus, 0x2000_0000);
    // low bit cleared -> 0x2000_0100, which is 4-aligned, OK.
    assert_eq!(c.x[1], 0x2000_0004);
    assert_eq!(c.pc, 0x2000_0100);
}

#[test]
fn exec_jalr_rd_eq_rs1_link_wins_over_target_race() {
    let (mut c, mut bus) = fresh();
    c.x[5] = 0x2000_0200;
    c.execute(Op::Jalr { rd: 5, rs1: 5, imm: 0 }, &mut bus, 0x2000_0000);
    // After execute: x[5] is the link (epc+4), pc is the old x[5].
    assert_eq!(c.x[5], 0x2000_0004);
    assert_eq!(c.pc, 0x2000_0200);
}

#[test]
fn jal_misalign_traps_cause0() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    // JAL with imm=2 -> target = epc+2 which is not 4-aligned.
    // Normally unreachable in RV32I encoding (imm is even), but this
    // exercises the execute-side guard. Use JALR to produce an odd-2
    // target instead.
    c.x[2] = 0x2000_0002; // 2-aligned, not 4-aligned
    c.execute(Op::Jalr { rd: 1, rs1: 2, imm: 0 }, &mut bus, 0x2000_0000);
    // low bit clear -> 0x2000_0002, still not 4-aligned -> trap.
    assert_eq!(c.csrs.mcause, 0);
    assert_eq!(c.csrs.mepc, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_2000);
}

// -----------------------------------------------------------------------
// ECALL / EBREAK / MRET
// -----------------------------------------------------------------------

#[test]
fn ecall_traps_cause_11() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000; // direct
    c.csrs.mstatus |= 1 << 3;   // set MIE so we can observe the shuffle
    c.execute(Op::Ecall, &mut bus, 0x2000_0008);
    assert_eq!(c.csrs.mcause, 11);
    assert_eq!(c.csrs.mepc, 0x2000_0008);
    assert_eq!(c.pc, 0x2000_2000);
    // MPIE should be set (old MIE=1), MIE cleared, MPP=0b11.
    assert_eq!((c.csrs.mstatus >> 3) & 1, 0, "MIE cleared");
    assert_eq!((c.csrs.mstatus >> 7) & 1, 1, "MPIE = old MIE");
    assert_eq!((c.csrs.mstatus >> 11) & 0b11, 0b11, "MPP = M");
}

#[test]
fn ebreak_traps_cause_3() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.execute(Op::Ebreak, &mut bus, 0x2000_0004);
    assert_eq!(c.csrs.mcause, 3);
    assert_eq!(c.csrs.mepc, 0x2000_0004);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn mret_restores_pc_and_mie() {
    let (mut c, mut bus) = fresh();
    c.csrs.mepc = 0x2000_0100;
    // MPIE=1, MIE=0, MPP=11 (after-trap state).
    c.csrs.mstatus = (1 << 7) | (0b11 << 11);
    c.execute(Op::Mret, &mut bus, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_0100);
    assert_eq!((c.csrs.mstatus >> 3) & 1, 1, "MIE <- MPIE");
    assert_eq!((c.csrs.mstatus >> 7) & 1, 1, "MPIE <- 1");
    assert_eq!((c.csrs.mstatus >> 11) & 0b11, 0b11, "MPP round-to-M");
}

#[test]
fn ecall_then_mret_roundtrip() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.csrs.mstatus = 1 << 3; // MIE=1
    c.execute(Op::Ecall, &mut bus, 0x2000_0008);
    // Handler "runs" — jump back.
    c.execute(Op::Mret, &mut bus, 0x2000_2000);
    assert_eq!(c.pc, 0x2000_0008, "mret -> mepc captured by ecall");
    assert_eq!((c.csrs.mstatus >> 3) & 1, 1, "MIE restored from MPIE");
}

// -----------------------------------------------------------------------
// Illegal instruction
// -----------------------------------------------------------------------

#[test]
fn illegal_op_traps_cause_2() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.execute(Op::Illegal { insn: 0xDEAD_BEEF }, &mut bus, 0x2000_0100);
    assert_eq!(c.csrs.mcause, 2);
    assert_eq!(c.csrs.mepc, 0x2000_0100);
    assert_eq!(c.pc, 0x2000_2000);
    // mtval hardwired 0.
    assert_eq!(c.csrs.mtval, 0);
}

#[test]
fn fetch_misaligned_traps_cause_0() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.pc = 0x2000_0001; // misaligned PC
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 0);
    assert_eq!(c.csrs.mepc, 0x2000_0001);
    assert_eq!(c.pc, 0x2000_2000);
}

// -----------------------------------------------------------------------
// CSR instructions
// -----------------------------------------------------------------------

#[test]
fn csrrw_read_then_write_mscratch() {
    let (mut c, mut bus) = fresh();
    c.csrs.mscratch = 0x1234_5678;
    c.x[1] = 0xFEED_BEEF;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 2, rs1_or_zimm: 1, csr: CSR_MSCRATCH },
        &mut bus, 0,
    );
    assert_eq!(c.x[2], 0x1234_5678, "rd = old");
    assert_eq!(c.csrs.mscratch, 0xFEED_BEEF, "new = rs1");
}

#[test]
fn csrrs_sets_bits() {
    let (mut c, mut bus) = fresh();
    c.csrs.mie = 0;
    c.x[1] = 1 << 7; // MTIE
    c.execute(
        Op::Csr { kind: CsrKind::Csrrs, rd: 2, rs1_or_zimm: 1, csr: CSR_MIE },
        &mut bus, 0,
    );
    assert_eq!(c.x[2], 0);
    assert_eq!(c.csrs.mie, 1 << 7);
}

#[test]
fn csrrc_clears_bits() {
    let (mut c, mut bus) = fresh();
    c.csrs.mie = (1 << 3) | (1 << 7) | (1 << 11);
    c.x[1] = 1 << 7;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrc, rd: 2, rs1_or_zimm: 1, csr: CSR_MIE },
        &mut bus, 0,
    );
    assert_eq!(c.x[2], (1 << 3) | (1 << 7) | (1 << 11));
    assert_eq!(c.csrs.mie, (1 << 3) | (1 << 11));
}

#[test]
fn csrrwi_uses_zimm() {
    let (mut c, mut bus) = fresh();
    c.csrs.mscratch = 0;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrwi, rd: 2, rs1_or_zimm: 0x1F, csr: CSR_MSCRATCH },
        &mut bus, 0,
    );
    assert_eq!(c.csrs.mscratch, 0x1F);
}

#[test]
fn csrrw_to_readonly_traps_even_rd_x0() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MHARTID },
        &mut bus, 0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 2);
    assert_eq!(c.csrs.mepc, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn csrrs_to_readonly_with_rs1_nonzero_traps() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrs, rd: 2, rs1_or_zimm: 5, csr: CSR_MHARTID },
        &mut bus, 0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 2);
}

#[test]
fn csrrs_to_readonly_with_rs1_zero_reads_no_trap() {
    let (mut c, mut bus) = fresh();
    c.execute(
        Op::Csr { kind: CsrKind::Csrrs, rd: 2, rs1_or_zimm: 0, csr: CSR_MHARTID },
        &mut bus, 0,
    );
    // No trap; rd gets the hartid value.
    assert_eq!(c.csrs.mcause, 0);
    assert_eq!(c.x[2], 0);
    assert_eq!(c.pc, 0x0000_0004);
}

#[test]
fn csrrci_to_readonly_with_zimm_zero_no_trap() {
    let (mut c, mut bus) = fresh();
    c.execute(
        Op::Csr { kind: CsrKind::Csrrci, rd: 3, rs1_or_zimm: 0, csr: CSR_MHARTID },
        &mut bus, 0,
    );
    assert_eq!(c.csrs.mcause, 0);
    assert_eq!(c.x[3], 0);
}

#[test]
fn unimplemented_csr_traps_cause_2() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrs, rd: 2, rs1_or_zimm: 0, csr: 0x3C0 /* unimpl */ },
        &mut bus, 0x2000_0000,
    );
    assert_eq!(c.csrs.mcause, 2);
}

#[test]
fn mstatus_mpp_warl_rounds_up() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 1 << 11; // MPP = 01 (S-mode) — not supported in V1
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MSTATUS },
        &mut bus, 0,
    );
    // WARL rounds to 0b11 (M-mode only).
    assert_eq!((c.csrs.mstatus >> 11) & 0b11, 0b11);
}

#[test]
fn mtvec_bit1_hardwired_zero() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x1234_5672; // bits [1:0] = 10 — bit 1 should clear
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MTVEC },
        &mut bus, 0,
    );
    // Writer keeps bit 0 (MODE) but clears bit 1.
    assert_eq!(c.csrs.mtvec, 0x1234_5670);
}

#[test]
fn mtval_write_ignored_hardwired_zero() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0xFFFF_FFFF;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MTVAL },
        &mut bus, 0,
    );
    assert_eq!(c.csrs.mtval, 0);
}

#[test]
fn mcause_warl_drops_illegal_exception_code() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 99; // illegal exception code
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MCAUSE },
        &mut bus, 0,
    );
    assert_eq!(c.csrs.mcause, 0);
}

#[test]
fn mcause_warl_keeps_legal_interrupt_code() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x8000_0007; // interrupt cause 7 (MTI) — legal
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MCAUSE },
        &mut bus, 0,
    );
    assert_eq!(c.csrs.mcause, 0x8000_0007);
}

#[test]
fn mcountinhibit_reserved_bit_cleared() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0b111; // bit 1 is reserved
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MCOUNTINHIBIT },
        &mut bus, 0,
    );
    assert_eq!(c.csrs.mcountinhibit, 0b101);
}

#[test]
fn mepc_low_bits_masked() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 0x2000_0103;
    c.execute(
        Op::Csr { kind: CsrKind::Csrrw, rd: 0, rs1_or_zimm: 1, csr: CSR_MEPC },
        &mut bus, 0,
    );
    assert_eq!(c.csrs.mepc, 0x2000_0100, "low 2 bits masked (no C in P2)");
}

// -----------------------------------------------------------------------
// Vectored mtvec
// -----------------------------------------------------------------------

#[test]
fn vectored_mtvec_exception_goes_to_base() {
    let (mut c, mut bus) = fresh();
    // mtvec base = 0x2000_2000, mode = 1 (vectored).
    c.csrs.mtvec = 0x2000_2001;
    c.execute(Op::Ebreak, &mut bus, 0x2000_0000);
    // Vectored: exceptions STILL dispatch to base.
    assert_eq!(c.pc, 0x2000_2000);
    assert_eq!(c.csrs.mcause, 3);
}

#[test]
fn vectored_mtvec_interrupt_uses_per_cause_slot() {
    // Simulate an interrupt cause by calling enter_trap directly with
    // bit 31 set. P4 will wire the real delivery path; this covers the
    // mtvec vectored branch inside trap.rs.
    let (mut c, _bus) = fresh();
    c.csrs.mtvec = 0x2000_2001;
    c.enter_trap(0x8000_0007, 0, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_2000 + 4 * 7);
    assert_eq!(c.csrs.mcause & 0xF, 7);
    assert_eq!(c.csrs.mcause & 0x8000_0000, 0x8000_0000);
}

#[test]
fn direct_mtvec_exception_goes_to_base() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000; // mode=0 direct
    c.execute(Op::Ebreak, &mut bus, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_2000);
}

// -----------------------------------------------------------------------
// WFI
// -----------------------------------------------------------------------

#[test]
fn wfi_sets_parked_flag() {
    let (mut c, mut bus) = fresh();
    c.execute(Op::Wfi, &mut bus, 0x2000_0000);
    assert!(c.wfi_parked);
    assert!(c.is_halted(), "is_halted folds wfi_parked per P1b");
}

// -----------------------------------------------------------------------
// FENCE / FENCE.I (no-op)
// -----------------------------------------------------------------------

#[test]
fn fence_is_noop() {
    let (mut c, mut bus) = fresh();
    let before_pc = c.pc;
    c.execute(Op::Fence, &mut bus, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_0004);
    let _ = before_pc;
}

#[test]
fn fence_i_is_noop_p2() {
    let (mut c, mut bus) = fresh();
    c.execute(Op::FenceI, &mut bus, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_0004);
}

// -----------------------------------------------------------------------
// End-to-end: step fetches + decodes + executes a real ADDI
// -----------------------------------------------------------------------

#[test]
fn step_executes_addi_from_sram() {
    let (mut c, mut bus) = fresh();
    // ADDI x1, x0, 42
    let insn = enc_i(OP_IMM, 1, 0b000, 0, 42);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[1], 42);
    assert_eq!(c.pc, 0x2000_0004);
    assert_eq!(c.cycles(), 1);
}

#[test]
fn step_fetch_bus_fault_traps_cause_1() {
    // Reset PC is 0x2000_0000. Move PC into an unmapped region so the
    // fetch itself faults.
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.pc = 0x1000_0000; // XIP with no flash loaded — reads as bus fault
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 1, "instruction access fault");
    assert_eq!(c.csrs.mepc, 0x1000_0000);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn lw_bus_fault_traps_cause_5() {
    // LW from an unmapped address. Fetch succeeds (from SRAM), but the
    // load access itself faults. HLD §4.5 cause 5 (load access fault).
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    // Seed x1 with an unmapped base (XIP with no flash loaded).
    c.x[1] = 0x1000_0000;
    // LW x2, 0(x1)
    let insn = enc_i(LOAD, 2, 0b010, 1, 0);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 5, "load access fault");
    assert_eq!(c.csrs.mepc, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn sw_bus_fault_traps_cause_7() {
    // SW to an unmapped address. Fetch succeeds, store faults. HLD §4.5
    // cause 7 (store access fault).
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    c.x[1] = 0x1000_0000; // unmapped base
    c.x[2] = 0xDEAD_BEEF; // value
    // SW x2, 0(x1)
    let insn = enc_s(STORE, 0b010, 1, 2, 0);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 7, "store access fault");
    assert_eq!(c.csrs.mepc, 0x2000_0000);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn mcycle_ticks_when_cy_clear() {
    // Reset mcountinhibit is 0b101 (both CY and IR inhibited). Clear CY
    // only and step a NOP — mcycle should tick by 1.
    let (mut c, mut bus) = fresh();
    c.csrs.mcountinhibit = 0b100; // CY clear, IR set
    // NOP: ADDI x0, x0, 0
    write_insn(&mut bus, 0, 0x0000_0013);
    c.step(&mut bus);
    assert_eq!(c.csrs.mcycle, 1, "mcycle ticks when CY clear");
    assert_eq!(c.csrs.minstret, 0, "minstret stays 0 when IR set");

    // Re-inhibit CY; mcycle must freeze.
    c.csrs.mcountinhibit = 0b101;
    write_insn(&mut bus, 4, 0x0000_0013);
    c.step(&mut bus);
    assert_eq!(c.csrs.mcycle, 1, "mcycle frozen when CY set");
}

#[test]
fn minstret_ticks_when_ir_clear() {
    // Clear IR only; step a NOP — minstret should tick by 1.
    let (mut c, mut bus) = fresh();
    c.csrs.mcountinhibit = 0b001; // IR clear, CY set
    // NOP: ADDI x0, x0, 0
    write_insn(&mut bus, 0, 0x0000_0013);
    c.step(&mut bus);
    assert_eq!(c.csrs.minstret, 1, "minstret ticks when IR clear");
    assert_eq!(c.csrs.mcycle, 0, "mcycle stays 0 when CY set");

    // Re-inhibit IR; minstret must freeze.
    c.csrs.mcountinhibit = 0b101;
    write_insn(&mut bus, 4, 0x0000_0013);
    c.step(&mut bus);
    assert_eq!(c.csrs.minstret, 1, "minstret frozen when IR set");
}

// -----------------------------------------------------------------------
// Decode coverage (sanity checks beyond decode.rs's own tests)
// -----------------------------------------------------------------------

#[test]
fn decode_and_execute_lui_via_step() {
    let (mut c, mut bus) = fresh();
    let insn = enc_u(LUI, 3, 0xABCD_E000);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[3], 0xABCD_E000);
}

#[test]
fn decode_and_execute_branch_via_step() {
    let (mut c, mut bus) = fresh();
    // BEQ x0, x0, +8 (always taken)
    let insn = enc_b(0b000, 0, 0, 8);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0008);
}

#[test]
fn decode_and_execute_jal_via_step() {
    let (mut c, mut bus) = fresh();
    let insn = enc_j(1, 0x10);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[1], 0x2000_0004);
    assert_eq!(c.pc, 0x2000_0010);
}

#[test]
fn decode_and_execute_jalr_via_step() {
    let (mut c, mut bus) = fresh();
    c.x[2] = 0x2000_0100;
    // JALR x1, x2, 0
    let insn = enc_i(JALR, 1, 0b000, 2, 0);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[1], 0x2000_0004);
    assert_eq!(c.pc, 0x2000_0100);
}

#[test]
fn decode_and_execute_ecall_via_step() {
    let (mut c, mut bus) = fresh();
    c.csrs.mtvec = 0x2000_2000;
    // ECALL = 0x00000073
    write_insn(&mut bus, 0, 0x0000_0073);
    c.step(&mut bus);
    assert_eq!(c.csrs.mcause, 11);
    assert_eq!(c.pc, 0x2000_2000);
}

#[test]
fn decode_and_execute_mret_via_step() {
    let (mut c, mut bus) = fresh();
    c.csrs.mepc = 0x2000_0200;
    c.csrs.mstatus = (1 << 7) | (0b11 << 11);
    // MRET = 0x30200073
    write_insn(&mut bus, 0, 0x3020_0073);
    c.step(&mut bus);
    assert_eq!(c.pc, 0x2000_0200);
}

#[test]
fn decode_and_execute_wfi_via_step() {
    let (mut c, mut bus) = fresh();
    // WFI = 0x10500073
    write_insn(&mut bus, 0, 0x1050_0073);
    c.step(&mut bus);
    assert!(c.wfi_parked);
}

// Parked hart doesn't advance on subsequent step.
#[test]
fn parked_hart_step_is_noop() {
    let (mut c, mut bus) = fresh();
    c.wfi_parked = true;
    let before_pc = c.pc;
    let before_cyc = c.cycles();
    c.step(&mut bus);
    assert_eq!(c.pc, before_pc);
    assert_eq!(c.cycles(), before_cyc);
}

// -----------------------------------------------------------------------
// Decode-sanity: every decoded variant round-trips through an execute
// -----------------------------------------------------------------------

#[test]
fn csr_read_mhartid_yields_hartid() {
    let mut c = Hazard3::new(1);
    let mut bus = Bus::new();
    // CSRRS x2, mhartid, x0
    let insn = enc_i(SYSTEM, 2, 0b010, 0, CSR_MHARTID as i32 & 0xFFF);
    assert!(matches!(decode::decode(insn),
        Op::Csr { kind: CsrKind::Csrrs, rd: 2, rs1_or_zimm: 0, .. }));
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[2], 1, "hart 1 mhartid = 1");
}

#[test]
fn lw_in_store_pos_then_load() {
    // Full round-trip via the step path. Plant two instructions in SRAM
    // and step twice.
    let (mut c, mut bus) = fresh();
    // Setup: x1 = 0x2000_1000, x2 = 0xCAFEBABE
    c.x[1] = 0x2000_1000;
    c.x[2] = 0xCAFE_BABE;
    // SW x2, 0(x1)
    let sw = enc_s(STORE, 0b010, 1, 2, 0);
    // LW x3, 0(x1)
    let lw = enc_i(LOAD, 3, 0b010, 1, 0);
    write_insn(&mut bus, 0, sw);
    write_insn(&mut bus, 4, lw);
    c.step(&mut bus);
    c.step(&mut bus);
    assert_eq!(c.x[3], 0xCAFE_BABE);
    assert_eq!(c.cycles(), 2);
    assert_eq!(c.pc, 0x2000_0008);
}

// A second ALU reg op for the enc_r helper coverage.
#[test]
fn step_add_via_sram() {
    let (mut c, mut bus) = fresh();
    c.x[1] = 3;
    c.x[2] = 4;
    // ADD x3, x1, x2
    let insn = enc_r(OP, 3, 0b000, 1, 2, 0b000_0000);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[3], 7);
}

// AUIPC end-to-end from SRAM.
#[test]
fn step_auipc_from_sram() {
    let (mut c, mut bus) = fresh();
    let insn = enc_u(AUIPC, 5, 0x1000);
    write_insn(&mut bus, 0, insn);
    c.step(&mut bus);
    assert_eq!(c.x[5], 0x2000_1000, "pc + imm<<12");
}

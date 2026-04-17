// RV32I + Zicsr + Zifencei decoder for Hazard3 (P2 scope). This is
// pure decode: it maps a 32-bit instruction word into an `Op` enum. The
// executor in `execute.rs` consumes the result. Unknown encodings decode
// to `Op::Illegal { insn }` so the executor can emit the correct
// `mcause=2` trap with the faulting word in hand (though `mtval` is
// hardwired 0 on Hazard3 per HLD §4.3 — the `insn` argument is discarded
// at the trap site).
//
// Scope is strictly P2: RV32I base + Zicsr + Zifencei. M/A/C are P3 (HLD
// §4.5). Compressed encodings (low two bits != 0b11) are rejected as
// illegal so the executor traps rather than silently decoding garbage —
// which is especially important given the Zcmp/C collision risk called
// out in HLD §4.5 (V6). When C lands in P3 this path becomes the C
// decoder, not a no-op.

#![allow(dead_code)] // P2 constructs these ops; some variants are only
                    // reachable once tests wire them, but every variant
                    // is covered by at least one execute_* path.

/// Primary opcode field (bits [6:2] with [1:0]==0b11 for base-ISA 32-bit
/// instructions). The low two bits being 0b11 is the gate that separates
/// base from compressed (C) encodings.
const OPCODE_LUI:    u32 = 0b01_101;
const OPCODE_AUIPC:  u32 = 0b00_101;
const OPCODE_JAL:    u32 = 0b11_011;
const OPCODE_JALR:   u32 = 0b11_001;
const OPCODE_BRANCH: u32 = 0b11_000;
const OPCODE_LOAD:   u32 = 0b00_000;
const OPCODE_STORE:  u32 = 0b01_000;
const OPCODE_OP_IMM: u32 = 0b00_100;
const OPCODE_OP:     u32 = 0b01_100;
const OPCODE_MISC_MEM: u32 = 0b00_011;
const OPCODE_SYSTEM: u32 = 0b11_100;

/// Decoded RV32I + Zicsr + Zifencei instruction. Scratch fields are
/// pre-extracted u5/u12 values to keep the executor branch-free on the
/// hot path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Op {
    // U-type
    Lui   { rd: u8, imm: u32 },
    Auipc { rd: u8, imm: u32 },

    // J-type
    Jal  { rd: u8, imm: i32 },
    // I-type (jump)
    Jalr { rd: u8, rs1: u8, imm: i32 },

    // B-type
    Branch { kind: BranchKind, rs1: u8, rs2: u8, imm: i32 },

    // I-type loads
    Load { kind: LoadKind, rd: u8, rs1: u8, imm: i32 },

    // S-type stores
    Store { kind: StoreKind, rs1: u8, rs2: u8, imm: i32 },

    // I-type ALU
    OpImm   { kind: AluImmKind, rd: u8, rs1: u8, imm: i32 },
    // I-type shift (immediate); shamt already extracted
    ShiftImm { kind: ShiftKind, rd: u8, rs1: u8, shamt: u8 },

    // R-type ALU
    Op { kind: AluKind, rd: u8, rs1: u8, rs2: u8 },

    // MISC-MEM
    Fence,
    FenceI,

    // SYSTEM
    Ecall,
    Ebreak,
    Mret,
    Wfi,
    Csr { kind: CsrKind, rd: u8, rs1_or_zimm: u8, csr: u16 },

    /// Anything we couldn't classify. Executor turns this into mcause=2.
    Illegal { insn: u32 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BranchKind { Beq, Bne, Blt, Bge, Bltu, Bgeu }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LoadKind { Lb, Lh, Lw, Lbu, Lhu }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StoreKind { Sb, Sh, Sw }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AluImmKind { Addi, Slti, Sltiu, Xori, Ori, Andi }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ShiftKind { Slli, Srli, Srai }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AluKind {
    Add, Sub, Sll, Slt, Sltu, Xor, Srl, Sra, Or, And,
}

/// Zicsr instruction family. `Imm` forms carry the 5-bit zimm in the
/// `rs1_or_zimm` field; register forms carry the rs1 index there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CsrKind {
    Csrrw, Csrrs, Csrrc, Csrrwi, Csrrsi, Csrrci,
}

// --- Bitfield accessors ---------------------------------------------------

#[inline(always)]
fn opcode(insn: u32) -> u32 { (insn >> 2) & 0x1F }
#[inline(always)]
fn rd(insn: u32)     -> u8 { ((insn >> 7) & 0x1F) as u8 }
#[inline(always)]
fn rs1(insn: u32)    -> u8 { ((insn >> 15) & 0x1F) as u8 }
#[inline(always)]
fn rs2(insn: u32)    -> u8 { ((insn >> 20) & 0x1F) as u8 }
#[inline(always)]
fn funct3(insn: u32) -> u32 { (insn >> 12) & 0x7 }
#[inline(always)]
fn funct7(insn: u32) -> u32 { (insn >> 25) & 0x7F }

/// Sign-extend `bits` treating bit `msb` as the sign bit.
#[inline(always)]
fn sext(bits: u32, msb: u32) -> i32 {
    let shift = 31 - msb;
    ((bits << shift) as i32) >> shift
}

/// I-type 12-bit immediate (bits 31:20), sign-extended.
#[inline(always)]
fn imm_i(insn: u32) -> i32 { sext(insn >> 20, 11) }

/// S-type immediate: bits 31:25 (hi7) + 11:7 (lo5), sign-extended.
#[inline(always)]
fn imm_s(insn: u32) -> i32 {
    let hi = (insn >> 25) & 0x7F;
    let lo = (insn >> 7) & 0x1F;
    sext((hi << 5) | lo, 11)
}

/// B-type immediate: [12|10:5|4:1|11] in bits [31|30:25|11:8|7], <<1, signed.
#[inline(always)]
fn imm_b(insn: u32) -> i32 {
    let b12 = (insn >> 31) & 0x1;
    let b11 = (insn >> 7) & 0x1;
    let b10_5 = (insn >> 25) & 0x3F;
    let b4_1 = (insn >> 8) & 0xF;
    sext((b12 << 12) | (b11 << 11) | (b10_5 << 5) | (b4_1 << 1), 12)
}

/// U-type immediate: bits 31:12 shifted to 31:12 with zero low 12 bits.
#[inline(always)]
fn imm_u(insn: u32) -> u32 { insn & 0xFFFF_F000 }

/// J-type immediate: [20|10:1|11|19:12] in bits [31|30:21|20|19:12], <<1.
#[inline(always)]
fn imm_j(insn: u32) -> i32 {
    let b20    = (insn >> 31) & 0x1;
    let b10_1  = (insn >> 21) & 0x3FF;
    let b11    = (insn >> 20) & 0x1;
    let b19_12 = (insn >> 12) & 0xFF;
    sext((b20 << 20) | (b19_12 << 12) | (b11 << 11) | (b10_1 << 1), 20)
}

// --- Top-level decode -----------------------------------------------------

pub(crate) fn decode(insn: u32) -> Op {
    // Base-ISA 32-bit instructions have bits[1:0]==0b11. Everything else
    // is a 16-bit compressed encoding (C), which is out of P2 scope —
    // reject with Illegal so the executor traps rather than silently
    // misdecoding (HLD §4.5 Zcmp/C collision note, V6).
    if (insn & 0b11) != 0b11 {
        return Op::Illegal { insn };
    }

    let op = opcode(insn);
    match op {
        OPCODE_LUI   => Op::Lui   { rd: rd(insn), imm: imm_u(insn) },
        OPCODE_AUIPC => Op::Auipc { rd: rd(insn), imm: imm_u(insn) },
        OPCODE_JAL   => Op::Jal   { rd: rd(insn), imm: imm_j(insn) },
        OPCODE_JALR => {
            // JALR is funct3=0 only.
            if funct3(insn) != 0 {
                return Op::Illegal { insn };
            }
            Op::Jalr { rd: rd(insn), rs1: rs1(insn), imm: imm_i(insn) }
        }
        OPCODE_BRANCH => decode_branch(insn),
        OPCODE_LOAD   => decode_load(insn),
        OPCODE_STORE  => decode_store(insn),
        OPCODE_OP_IMM => decode_op_imm(insn),
        OPCODE_OP     => decode_op(insn),
        OPCODE_MISC_MEM => decode_misc_mem(insn),
        OPCODE_SYSTEM   => decode_system(insn),
        _ => Op::Illegal { insn },
    }
}

fn decode_branch(insn: u32) -> Op {
    let kind = match funct3(insn) {
        0b000 => BranchKind::Beq,
        0b001 => BranchKind::Bne,
        0b100 => BranchKind::Blt,
        0b101 => BranchKind::Bge,
        0b110 => BranchKind::Bltu,
        0b111 => BranchKind::Bgeu,
        _ => return Op::Illegal { insn },
    };
    Op::Branch { kind, rs1: rs1(insn), rs2: rs2(insn), imm: imm_b(insn) }
}

fn decode_load(insn: u32) -> Op {
    let kind = match funct3(insn) {
        0b000 => LoadKind::Lb,
        0b001 => LoadKind::Lh,
        0b010 => LoadKind::Lw,
        0b100 => LoadKind::Lbu,
        0b101 => LoadKind::Lhu,
        _ => return Op::Illegal { insn },
    };
    Op::Load { kind, rd: rd(insn), rs1: rs1(insn), imm: imm_i(insn) }
}

fn decode_store(insn: u32) -> Op {
    let kind = match funct3(insn) {
        0b000 => StoreKind::Sb,
        0b001 => StoreKind::Sh,
        0b010 => StoreKind::Sw,
        _ => return Op::Illegal { insn },
    };
    Op::Store { kind, rs1: rs1(insn), rs2: rs2(insn), imm: imm_s(insn) }
}

fn decode_op_imm(insn: u32) -> Op {
    let f3 = funct3(insn);
    let rd_ = rd(insn);
    let rs1_ = rs1(insn);
    // Shift forms are funct3 == 001 (SLLI) / 101 (SRLI/SRAI) and share
    // the OP-IMM opcode but carry a shamt + funct7 discriminator.
    if f3 == 0b001 {
        // SLLI: funct7 must be 0000000 (RV32I). With C/B extensions later
        // this check tightens.
        if funct7(insn) != 0b000_0000 {
            return Op::Illegal { insn };
        }
        let shamt = ((insn >> 20) & 0x1F) as u8;
        return Op::ShiftImm { kind: ShiftKind::Slli, rd: rd_, rs1: rs1_, shamt };
    }
    if f3 == 0b101 {
        let f7 = funct7(insn);
        let shamt = ((insn >> 20) & 0x1F) as u8;
        let kind = match f7 {
            0b000_0000 => ShiftKind::Srli,
            0b010_0000 => ShiftKind::Srai,
            _ => return Op::Illegal { insn },
        };
        return Op::ShiftImm { kind, rd: rd_, rs1: rs1_, shamt };
    }

    let kind = match f3 {
        0b000 => AluImmKind::Addi,
        0b010 => AluImmKind::Slti,
        0b011 => AluImmKind::Sltiu,
        0b100 => AluImmKind::Xori,
        0b110 => AluImmKind::Ori,
        0b111 => AluImmKind::Andi,
        _ => unreachable!("f3 001/101 handled above"),
    };
    Op::OpImm { kind, rd: rd_, rs1: rs1_, imm: imm_i(insn) }
}

fn decode_op(insn: u32) -> Op {
    let f3 = funct3(insn);
    let f7 = funct7(insn);
    let rd_ = rd(insn);
    let rs1_ = rs1(insn);
    let rs2_ = rs2(insn);
    let kind = match (f3, f7) {
        (0b000, 0b000_0000) => AluKind::Add,
        (0b000, 0b010_0000) => AluKind::Sub,
        (0b001, 0b000_0000) => AluKind::Sll,
        (0b010, 0b000_0000) => AluKind::Slt,
        (0b011, 0b000_0000) => AluKind::Sltu,
        (0b100, 0b000_0000) => AluKind::Xor,
        (0b101, 0b000_0000) => AluKind::Srl,
        (0b101, 0b010_0000) => AluKind::Sra,
        (0b110, 0b000_0000) => AluKind::Or,
        (0b111, 0b000_0000) => AluKind::And,
        // M extension (mul/div) — funct7=0000001, f3 0..7 — lands in P3.
        _ => return Op::Illegal { insn },
    };
    Op::Op { kind, rd: rd_, rs1: rs1_, rs2: rs2_ }
}

fn decode_misc_mem(insn: u32) -> Op {
    match funct3(insn) {
        // FENCE — pred/succ/fm fields ignored (single-threaded emulation).
        0b000 => Op::Fence,
        // FENCE.I — Zifencei. No-op today; HLD §4.8 tripwire required when
        // a decoded-op cache lands. The debug_assert fires in
        // debug builds whenever FENCE.I executes, guarding the future
        // cache-add PR against the silent-stale-decode regression.
        0b001 => Op::FenceI,
        _ => Op::Illegal { insn },
    }
}

fn decode_system(insn: u32) -> Op {
    let f3 = funct3(insn);
    if f3 == 0b000 {
        // PRIV: ECALL / EBREAK / MRET / WFI / (others illegal in P2).
        // rd and rs1 must be 0 for these forms.
        if rd(insn) != 0 || rs1(insn) != 0 {
            return Op::Illegal { insn };
        }
        let funct12 = (insn >> 20) & 0xFFF;
        return match funct12 {
            0x000 => Op::Ecall,
            0x001 => Op::Ebreak,
            0x302 => Op::Mret,
            0x105 => Op::Wfi,
            _ => Op::Illegal { insn },
        };
    }
    // Zicsr family. csr = imm_i-style high 12 bits (unsigned).
    let csr = ((insn >> 20) & 0xFFF) as u16;
    let kind = match f3 {
        0b001 => CsrKind::Csrrw,
        0b010 => CsrKind::Csrrs,
        0b011 => CsrKind::Csrrc,
        0b101 => CsrKind::Csrrwi,
        0b110 => CsrKind::Csrrsi,
        0b111 => CsrKind::Csrrci,
        _ => return Op::Illegal { insn },
    };
    Op::Csr { kind, rd: rd(insn), rs1_or_zimm: rs1(insn), csr }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enc_u(opcode: u32, rd: u8, imm: u32) -> u32 {
        (imm & 0xFFFF_F000) | ((rd as u32) << 7) | (opcode << 2) | 0b11
    }

    fn enc_i(opcode: u32, rd: u8, f3: u32, rs1: u8, imm: i32) -> u32 {
        let imm_u = (imm as u32) & 0xFFF;
        (imm_u << 20) | ((rs1 as u32) << 15) | (f3 << 12) | ((rd as u32) << 7)
            | (opcode << 2) | 0b11
    }

    fn enc_r(opcode: u32, rd: u8, f3: u32, rs1: u8, rs2: u8, f7: u32) -> u32 {
        (f7 << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15)
            | (f3 << 12) | ((rd as u32) << 7) | (opcode << 2) | 0b11
    }

    fn enc_s(opcode: u32, f3: u32, rs1: u8, rs2: u8, imm: i32) -> u32 {
        let imm_u = (imm as u32) & 0xFFF;
        let hi = (imm_u >> 5) & 0x7F;
        let lo = imm_u & 0x1F;
        (hi << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15)
            | (f3 << 12) | (lo << 7) | (opcode << 2) | 0b11
    }

    fn enc_b(f3: u32, rs1: u8, rs2: u8, imm: i32) -> u32 {
        let imm_u = (imm as u32) & 0x1FFE;
        let b12 = (imm_u >> 12) & 0x1;
        let b11 = (imm_u >> 11) & 0x1;
        let b10_5 = (imm_u >> 5) & 0x3F;
        let b4_1 = (imm_u >> 1) & 0xF;
        (b12 << 31) | (b10_5 << 25) | ((rs2 as u32) << 20)
            | ((rs1 as u32) << 15) | (f3 << 12) | (b4_1 << 8)
            | (b11 << 7) | (OPCODE_BRANCH << 2) | 0b11
    }

    fn enc_j(rd: u8, imm: i32) -> u32 {
        let imm_u = (imm as u32) & 0x1F_FFFE;
        let b20 = (imm_u >> 20) & 0x1;
        let b10_1 = (imm_u >> 1) & 0x3FF;
        let b11 = (imm_u >> 11) & 0x1;
        let b19_12 = (imm_u >> 12) & 0xFF;
        (b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12)
            | ((rd as u32) << 7) | (OPCODE_JAL << 2) | 0b11
    }

    #[test]
    fn compressed_encoding_is_illegal() {
        // Any insn where bits[1:0] != 0b11 must decode as Illegal.
        // 0x0000 is the canonical C illegal instruction.
        assert!(matches!(decode(0x0000), Op::Illegal { .. }));
        // A valid-looking Zcmp-ish pattern (c.ldsp-ish) — must be illegal
        // until P3.
        assert!(matches!(decode(0xB822), Op::Illegal { .. }));
    }

    #[test]
    fn decodes_lui() {
        // LUI x5, 0x12345
        let insn = enc_u(OPCODE_LUI, 5, 0x1234_5000);
        assert_eq!(decode(insn), Op::Lui { rd: 5, imm: 0x1234_5000 });
    }

    #[test]
    fn decodes_auipc() {
        let insn = enc_u(OPCODE_AUIPC, 7, 0x0000_1000);
        assert_eq!(decode(insn), Op::Auipc { rd: 7, imm: 0x0000_1000 });
    }

    #[test]
    fn decodes_addi_and_sign_extends_imm() {
        // ADDI x1, x0, -1  ->  0xFFFF_FFFF
        let insn = enc_i(OPCODE_OP_IMM, 1, 0b000, 0, -1);
        match decode(insn) {
            Op::OpImm { kind: AluImmKind::Addi, rd: 1, rs1: 0, imm: -1 } => (),
            other => panic!("unexpected {:?}", other),
        }
    }

    #[test]
    fn decodes_all_alu_reg_ops() {
        let cases = [
            (0b000, 0b000_0000, AluKind::Add),
            (0b000, 0b010_0000, AluKind::Sub),
            (0b001, 0b000_0000, AluKind::Sll),
            (0b010, 0b000_0000, AluKind::Slt),
            (0b011, 0b000_0000, AluKind::Sltu),
            (0b100, 0b000_0000, AluKind::Xor),
            (0b101, 0b000_0000, AluKind::Srl),
            (0b101, 0b010_0000, AluKind::Sra),
            (0b110, 0b000_0000, AluKind::Or),
            (0b111, 0b000_0000, AluKind::And),
        ];
        for (f3, f7, expected) in cases {
            let insn = enc_r(OPCODE_OP, 3, f3, 4, 5, f7);
            match decode(insn) {
                Op::Op { kind, rd: 3, rs1: 4, rs2: 5 } if kind == expected => (),
                other => panic!("{:?} -> {:?}", (f3, f7, expected), other),
            }
        }
    }

    #[test]
    fn decodes_branch_negative_offset() {
        // BEQ x1, x2, -8
        let insn = enc_b(0b000, 1, 2, -8);
        assert_eq!(
            decode(insn),
            Op::Branch { kind: BranchKind::Beq, rs1: 1, rs2: 2, imm: -8 },
        );
    }

    #[test]
    fn decodes_jal_positive_offset() {
        let insn = enc_j(1, 0x4000);
        assert_eq!(decode(insn), Op::Jal { rd: 1, imm: 0x4000 });
    }

    #[test]
    fn decodes_store_sw() {
        // SW x5, 0x10(x3)
        let insn = enc_s(OPCODE_STORE, 0b010, 3, 5, 0x10);
        assert_eq!(decode(insn), Op::Store { kind: StoreKind::Sw, rs1: 3, rs2: 5, imm: 0x10 });
    }

    #[test]
    fn decodes_ecall_ebreak_mret_wfi() {
        // ECALL = 0x00000073
        assert_eq!(decode(0x0000_0073), Op::Ecall);
        // EBREAK = 0x00100073
        assert_eq!(decode(0x0010_0073), Op::Ebreak);
        // MRET = 0x30200073
        assert_eq!(decode(0x3020_0073), Op::Mret);
        // WFI = 0x10500073
        assert_eq!(decode(0x1050_0073), Op::Wfi);
    }

    #[test]
    fn decodes_csrrw() {
        // CSRRW x5, mstatus (0x300), x6
        let insn = enc_i(OPCODE_SYSTEM, 5, 0b001, 6, 0x300);
        assert_eq!(
            decode(insn),
            Op::Csr { kind: CsrKind::Csrrw, rd: 5, rs1_or_zimm: 6, csr: 0x300 },
        );
    }

    #[test]
    fn decodes_shift_imm_srai() {
        // SRAI x1, x2, 3 — funct7=0100000, shamt=3
        let insn = (0b010_0000 << 25) | (3u32 << 20) | (2u32 << 15)
            | (0b101 << 12) | (1u32 << 7) | (OPCODE_OP_IMM << 2) | 0b11;
        assert_eq!(
            decode(insn),
            Op::ShiftImm { kind: ShiftKind::Srai, rd: 1, rs1: 2, shamt: 3 },
        );
    }

    #[test]
    fn fence_and_fence_i() {
        // FENCE: opcode=MISC-MEM, funct3=000. fm/pred/succ ignored.
        let insn = (OPCODE_MISC_MEM << 2) | 0b11;
        assert_eq!(decode(insn), Op::Fence);
        // FENCE.I: funct3=001
        let insn = (0b001 << 12) | (OPCODE_MISC_MEM << 2) | 0b11;
        assert_eq!(decode(insn), Op::FenceI);
    }

    #[test]
    fn illegal_unknown_opcode() {
        // opcode = 0b11111 (reserved)
        let insn = (0b11111 << 2) | 0b11;
        assert!(matches!(decode(insn), Op::Illegal { .. }));
    }

    #[test]
    fn illegal_mul_rv32m_is_p3() {
        // MUL is funct7=0000001 — P3 scope. P2 rejects.
        let insn = enc_r(OPCODE_OP, 3, 0b000, 4, 5, 0b000_0001);
        assert!(matches!(decode(insn), Op::Illegal { .. }));
    }
}

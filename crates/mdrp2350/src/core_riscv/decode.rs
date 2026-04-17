// RV32I + Zicsr + Zifencei + M + A + C decoder for Hazard3. This is
// pure decode: it maps a 16-bit or 32-bit instruction word into an `Op`
// enum. The executor in `execute.rs` consumes the result. Unknown
// encodings decode to `Op::Illegal { insn }` so the executor can emit
// the correct `mcause=2` trap with the faulting word in hand (though
// `mtval` is hardwired 0 on Hazard3 per HLD §4.3 — the `insn` argument
// is discarded at the trap site).
//
// P3 extensions land here:
//   * RV32M (mul/div)  — funct7=0x01 under OPCODE_OP.
//   * RV32A (atomics)  — OPCODE_AMO (0b01011), funct3=0b010.
//   * RV32C (compressed) — `decode16` expands 16-bit compressed encodings
//     into the same `Op` variants used for 32-bit forms. The step wrapper
//     decides 16-vs-32-bit based on `insn[1:0]` before calling into this
//     module.

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
const OPCODE_AMO:    u32 = 0b01_011; // RV32A (funct3 = 010 for word-sized)

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

    // RV32M (multiply / divide). Shares OPCODE_OP with R-type ALU, but
    // uses funct7 == 0x01. Split into its own variant so the executor
    // can distinguish mul/div cleanly.
    MulDiv { kind: MulDivKind, rd: u8, rs1: u8, rs2: u8 },

    // RV32A (atomics, word-sized subset). `aq`/`rl` memory-ordering bits
    // are ignored on the emulator (single-hart-at-a-time bus, same as
    // FENCE) but preserved in case future work wants to model them.
    Amo { kind: AmoKind, rd: u8, rs1: u8, rs2: u8, aq: bool, rl: bool },

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

/// RV32M operations. Funct3 encoding: MUL=0, MULH=1, MULHSU=2, MULHU=3,
/// DIV=4, DIVU=5, REM=6, REMU=7.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MulDivKind {
    Mul, Mulh, Mulhsu, Mulhu,
    Div, Divu, Rem, Remu,
}

/// RV32A word-sized operations. LR/SC don't carry an rs2 address-operand
/// role; we still reuse the variant for encoding symmetry — the executor
/// ignores rs2 for `Lr` and treats it as the store value for `Sc`/amo*.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AmoKind {
    Lr, Sc,
    Swap, Add, And, Or, Xor,
    Min, Max, Minu, Maxu,
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
    // Base-ISA 32-bit instructions have bits[1:0]==0b11. Compressed (C)
    // encodings (low two bits != 0b11) are handled by `decode16` —
    // `Hazard3::step` peeks at the first halfword and dispatches. If
    // anything calls `decode` with a non-base-ISA word, something
    // upstream is wrong; report Illegal with the original word.
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
        OPCODE_AMO      => decode_amo(insn),
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
    // RV32M: funct7 == 0x01. Funct3 selects the op.
    if f7 == 0b000_0001 {
        let kind = match f3 {
            0b000 => MulDivKind::Mul,
            0b001 => MulDivKind::Mulh,
            0b010 => MulDivKind::Mulhsu,
            0b011 => MulDivKind::Mulhu,
            0b100 => MulDivKind::Div,
            0b101 => MulDivKind::Divu,
            0b110 => MulDivKind::Rem,
            0b111 => MulDivKind::Remu,
            _ => unreachable!("f3 is 3 bits"),
        };
        return Op::MulDiv { kind, rd: rd_, rs1: rs1_, rs2: rs2_ };
    }
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
        _ => return Op::Illegal { insn },
    };
    Op::Op { kind, rd: rd_, rs1: rs1_, rs2: rs2_ }
}

/// RV32A word-sized atomics. Bits 31..27 select the op, bits 26/25 are
/// aq/rl. funct3 must be 010 for word (.W) size; others are illegal
/// (Hazard3 doesn't implement RV64A).
fn decode_amo(insn: u32) -> Op {
    if funct3(insn) != 0b010 {
        return Op::Illegal { insn };
    }
    let rd_ = rd(insn);
    let rs1_ = rs1(insn);
    let rs2_ = rs2(insn);
    let funct5 = (insn >> 27) & 0x1F;
    let aq = ((insn >> 26) & 1) != 0;
    let rl = ((insn >> 25) & 1) != 0;
    let kind = match funct5 {
        0b00010 => {
            // LR.W — rs2 must be zero per spec.
            if rs2_ != 0 {
                return Op::Illegal { insn };
            }
            AmoKind::Lr
        }
        0b00011 => AmoKind::Sc,
        0b00001 => AmoKind::Swap,
        0b00000 => AmoKind::Add,
        0b01100 => AmoKind::And,
        0b01000 => AmoKind::Or,
        0b00100 => AmoKind::Xor,
        0b10000 => AmoKind::Min,
        0b10100 => AmoKind::Max,
        0b11000 => AmoKind::Minu,
        0b11100 => AmoKind::Maxu,
        _ => return Op::Illegal { insn },
    };
    Op::Amo { kind, rd: rd_, rs1: rs1_, rs2: rs2_, aq, rl }
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

// --- RV32C (compressed) decode -------------------------------------------
//
// `decode16` expands a 16-bit compressed instruction into the same `Op`
// variants the 32-bit decoder produces, so the executor sees one shape
// of instruction regardless of width. Encodings come from the RISC-V
// unprivileged ISA spec, Chapter "C" standard extension.
//
// RV32C layout sketch (quadrants by bits[1:0]):
//   Q0 (00): C.ADDI4SPN, C.LW, C.SW, illegal (C.FLD/C.FSD unused — no F/D)
//   Q1 (01): C.NOP/C.ADDI, C.JAL, C.LI, C.ADDI16SP/C.LUI, MISC-ALU (funct3=100),
//            C.J, C.BEQZ, C.BNEZ
//   Q2 (10): C.SLLI, C.LWSP, C.JR/C.MV/C.EBREAK/C.JALR/C.ADD, C.SWSP
//
// Bit numbering within the 16-bit insn follows the spec: bit 0 is LSB
// of the 16-bit word. Register-field helpers `creg3` etc. map 3-bit
// encodings to x8..x15 per spec.

#[inline(always)]
fn creg3(bits: u16) -> u8 { 8 + (bits & 0x7) as u8 }

/// Sign-extend a u32 of `msb+1` significant bits (bit `msb` is the sign).
#[inline(always)]
fn sext_u(bits: u32, msb: u32) -> i32 { sext(bits, msb) }

pub(crate) fn decode16(hw: u16) -> Op {
    // Illegal compressed instruction: the all-zeros 16-bit word.
    if hw == 0 {
        return Op::Illegal { insn: 0 };
    }
    let hw = hw as u32;
    let quadrant = hw & 0b11;
    let f3 = (hw >> 13) & 0b111;
    match quadrant {
        0b00 => decode16_q0(hw, f3),
        0b01 => decode16_q1(hw, f3),
        0b10 => decode16_q2(hw, f3),
        _ => Op::Illegal { insn: hw },
    }
}

fn decode16_q0(hw: u32, f3: u32) -> Op {
    let rd_p = creg3((hw >> 2) as u16);
    let rs1_p = creg3((hw >> 7) as u16);
    match f3 {
        0b000 => {
            // C.ADDI4SPN rd', nzuimm — rd = sp + nzuimm
            // nzuimm[5:4|9:6|2|3] in insn[12:11|10:7|6|5]. nzuimm != 0.
            let b5_4   = (hw >> 11) & 0b11;
            let b9_6   = (hw >> 7) & 0b1111;
            let b2     = (hw >> 6) & 0b1;
            let b3     = (hw >> 5) & 0b1;
            let nzuimm = (b9_6 << 6) | (b5_4 << 4) | (b3 << 3) | (b2 << 2);
            if nzuimm == 0 {
                return Op::Illegal { insn: hw };
            }
            Op::OpImm { kind: AluImmKind::Addi, rd: rd_p, rs1: 2, imm: nzuimm as i32 }
        }
        0b010 => {
            // C.LW rd', uimm(rs1')
            // uimm[5:3|2|6] in insn[12:10|6|5]
            let b5_3 = (hw >> 10) & 0b111;
            let b2   = (hw >> 6) & 0b1;
            let b6   = (hw >> 5) & 0b1;
            let uimm = (b6 << 6) | (b5_3 << 3) | (b2 << 2);
            Op::Load { kind: LoadKind::Lw, rd: rd_p, rs1: rs1_p, imm: uimm as i32 }
        }
        0b110 => {
            // C.SW rs2', uimm(rs1')
            let b5_3 = (hw >> 10) & 0b111;
            let b2   = (hw >> 6) & 0b1;
            let b6   = (hw >> 5) & 0b1;
            let uimm = (b6 << 6) | (b5_3 << 3) | (b2 << 2);
            let rs2_p = creg3((hw >> 2) as u16);
            Op::Store { kind: StoreKind::Sw, rs1: rs1_p, rs2: rs2_p, imm: uimm as i32 }
        }
        _ => Op::Illegal { insn: hw },
    }
}

fn decode16_q1(hw: u32, f3: u32) -> Op {
    match f3 {
        0b000 => {
            // C.NOP / C.ADDI rd, nzimm[5:0]
            // nzimm[5] = bit 12, nzimm[4:0] = bits 6:2. Sign-extended.
            let rd_ = ((hw >> 7) & 0x1F) as u8;
            let imm_raw = (((hw >> 12) & 1) << 5) | ((hw >> 2) & 0x1F);
            let imm = sext_u(imm_raw, 5);
            if rd_ == 0 && imm_raw == 0 {
                return Op::OpImm { kind: AluImmKind::Addi, rd: 0, rs1: 0, imm: 0 };
            }
            Op::OpImm { kind: AluImmKind::Addi, rd: rd_, rs1: rd_, imm }
        }
        0b001 => {
            // C.JAL imm — rv32-only. rd = x1.
            let imm = c_jimm(hw);
            Op::Jal { rd: 1, imm }
        }
        0b010 => {
            // C.LI rd, imm — addi rd, x0, imm
            let rd_ = ((hw >> 7) & 0x1F) as u8;
            let imm_raw = (((hw >> 12) & 1) << 5) | ((hw >> 2) & 0x1F);
            let imm = sext_u(imm_raw, 5);
            Op::OpImm { kind: AluImmKind::Addi, rd: rd_, rs1: 0, imm }
        }
        0b011 => {
            let rd_ = ((hw >> 7) & 0x1F) as u8;
            if rd_ == 2 {
                // C.ADDI16SP nzimm — scaled by 16. nzimm[9|4|6|8:7|5] in
                // bits[12|6|5|4:3|2]. Sign-extended from bit 9.
                let b9 = (hw >> 12) & 1;
                let b4 = (hw >> 6) & 1;
                let b6 = (hw >> 5) & 1;
                let b8_7 = (hw >> 3) & 0b11;
                let b5 = (hw >> 2) & 1;
                let nzimm_raw = (b9 << 9) | (b8_7 << 7) | (b6 << 6) | (b5 << 5) | (b4 << 4);
                if nzimm_raw == 0 {
                    return Op::Illegal { insn: hw };
                }
                let imm = sext_u(nzimm_raw, 9);
                Op::OpImm { kind: AluImmKind::Addi, rd: 2, rs1: 2, imm }
            } else {
                // C.LUI rd, nzimm[17:12]. Bits[12] = imm[17]; bits[6:2] = imm[16:12].
                let b17 = (hw >> 12) & 1;
                let b16_12 = (hw >> 2) & 0x1F;
                let nzimm6 = (b17 << 5) | b16_12;
                if nzimm6 == 0 || rd_ == 0 {
                    return Op::Illegal { insn: hw };
                }
                let sext_imm = sext_u(nzimm6, 5);
                let imm = (sext_imm as u32) << 12;
                Op::Lui { rd: rd_, imm }
            }
        }
        0b100 => {
            // Q1 MISC-ALU — bits[11:10] discriminator.
            let rs1_p = creg3((hw >> 7) as u16);
            let op = (hw >> 10) & 0b11;
            match op {
                0b00 => {
                    let b5 = ((hw >> 12) & 1) as u8;
                    let b4_0 = ((hw >> 2) & 0x1F) as u8;
                    let shamt = (b5 << 5) | b4_0;
                    if shamt >= 32 {
                        return Op::Illegal { insn: hw };
                    }
                    Op::ShiftImm { kind: ShiftKind::Srli, rd: rs1_p, rs1: rs1_p, shamt }
                }
                0b01 => {
                    let b5 = ((hw >> 12) & 1) as u8;
                    let b4_0 = ((hw >> 2) & 0x1F) as u8;
                    let shamt = (b5 << 5) | b4_0;
                    if shamt >= 32 {
                        return Op::Illegal { insn: hw };
                    }
                    Op::ShiftImm { kind: ShiftKind::Srai, rd: rs1_p, rs1: rs1_p, shamt }
                }
                0b10 => {
                    let imm_raw = (((hw >> 12) & 1) << 5) | ((hw >> 2) & 0x1F);
                    let imm = sext_u(imm_raw, 5);
                    Op::OpImm { kind: AluImmKind::Andi, rd: rs1_p, rs1: rs1_p, imm }
                }
                0b11 => {
                    let rs2_p = creg3((hw >> 2) as u16);
                    let sel = (((hw >> 12) & 1) << 2) | ((hw >> 5) & 0b11);
                    let kind = match sel {
                        0b000 => AluKind::Sub,
                        0b001 => AluKind::Xor,
                        0b010 => AluKind::Or,
                        0b011 => AluKind::And,
                        // 0b100..0b111 are C.SUBW/C.ADDW/reserved — RV64
                        // only. Illegal on RV32.
                        _ => return Op::Illegal { insn: hw },
                    };
                    Op::Op { kind, rd: rs1_p, rs1: rs1_p, rs2: rs2_p }
                }
                _ => unreachable!(),
            }
        }
        0b101 => {
            // C.J imm — unconditional jump. rd = x0.
            let imm = c_jimm(hw);
            Op::Jal { rd: 0, imm }
        }
        0b110 => {
            let rs1_p = creg3((hw >> 7) as u16);
            let imm = c_bimm(hw);
            Op::Branch { kind: BranchKind::Beq, rs1: rs1_p, rs2: 0, imm }
        }
        0b111 => {
            let rs1_p = creg3((hw >> 7) as u16);
            let imm = c_bimm(hw);
            Op::Branch { kind: BranchKind::Bne, rs1: rs1_p, rs2: 0, imm }
        }
        _ => Op::Illegal { insn: hw },
    }
}

fn decode16_q2(hw: u32, f3: u32) -> Op {
    let rd_ = ((hw >> 7) & 0x1F) as u8;
    let rs2_ = ((hw >> 2) & 0x1F) as u8;
    match f3 {
        0b000 => {
            let b5 = ((hw >> 12) & 1) as u8;
            let b4_0 = ((hw >> 2) & 0x1F) as u8;
            let shamt = (b5 << 5) | b4_0;
            if shamt >= 32 {
                return Op::Illegal { insn: hw };
            }
            Op::ShiftImm { kind: ShiftKind::Slli, rd: rd_, rs1: rd_, shamt }
        }
        0b010 => {
            if rd_ == 0 {
                return Op::Illegal { insn: hw };
            }
            let b5 = (hw >> 12) & 1;
            let b4_2 = (hw >> 4) & 0b111;
            let b7_6 = (hw >> 2) & 0b11;
            let uimm = (b7_6 << 6) | (b5 << 5) | (b4_2 << 2);
            Op::Load { kind: LoadKind::Lw, rd: rd_, rs1: 2, imm: uimm as i32 }
        }
        0b100 => {
            let bit12 = (hw >> 12) & 1;
            if bit12 == 0 {
                if rs2_ == 0 {
                    if rd_ == 0 {
                        return Op::Illegal { insn: hw };
                    }
                    Op::Jalr { rd: 0, rs1: rd_, imm: 0 }
                } else {
                    Op::Op { kind: AluKind::Add, rd: rd_, rs1: 0, rs2: rs2_ }
                }
            } else if rd_ == 0 && rs2_ == 0 {
                Op::Ebreak
            } else if rs2_ == 0 {
                Op::Jalr { rd: 1, rs1: rd_, imm: 0 }
            } else {
                Op::Op { kind: AluKind::Add, rd: rd_, rs1: rd_, rs2: rs2_ }
            }
        }
        0b110 => {
            let b5_2 = (hw >> 9) & 0b1111;
            let b7_6 = (hw >> 7) & 0b11;
            let uimm = (b7_6 << 6) | (b5_2 << 2);
            Op::Store { kind: StoreKind::Sw, rs1: 2, rs2: rs2_, imm: uimm as i32 }
        }
        _ => Op::Illegal { insn: hw },
    }
}

/// C.J / C.JAL immediate. 12-bit sign-extended.
/// imm[11|4|9:8|10|6|7|3:1|5] in bits[12|11|10:9|8|7|6|5:3|2].
fn c_jimm(hw: u32) -> i32 {
    let b11   = (hw >> 12) & 1;
    let b4    = (hw >> 11) & 1;
    let b9_8  = (hw >> 9) & 0b11;
    let b10   = (hw >> 8) & 1;
    let b6    = (hw >> 7) & 1;
    let b7    = (hw >> 6) & 1;
    let b3_1  = (hw >> 3) & 0b111;
    let b5    = (hw >> 2) & 1;
    let raw = (b11 << 11) | (b10 << 10) | (b9_8 << 8) | (b7 << 7) | (b6 << 6)
        | (b5 << 5) | (b4 << 4) | (b3_1 << 1);
    sext_u(raw, 11)
}

/// C.BEQZ / C.BNEZ immediate.
/// imm[8|4:3|7:6|2:1|5] in bits[12|11:10|6:5|4:3|2].
fn c_bimm(hw: u32) -> i32 {
    let b8   = (hw >> 12) & 1;
    let b4_3 = (hw >> 10) & 0b11;
    let b7_6 = (hw >> 5) & 0b11;
    let b2_1 = (hw >> 3) & 0b11;
    let b5   = (hw >> 2) & 1;
    let raw = (b8 << 8) | (b7_6 << 6) | (b5 << 5) | (b4_3 << 3) | (b2_1 << 1);
    sext_u(raw, 8)
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
    fn top_level_decode_rejects_compressed_word() {
        // `decode` is the 32-bit-base-ISA entrypoint. Compressed goes
        // through `decode16`. If `decode` ever sees a word with low bits
        // != 0b11 it means the caller forgot to dispatch — report Illegal
        // so the regression is visible.
        assert!(matches!(decode(0x0000), Op::Illegal { .. }));
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
    fn decodes_rv32m_mul() {
        // MUL is funct7=0000001. P3 decodes.
        let insn = enc_r(OPCODE_OP, 3, 0b000, 4, 5, 0b000_0001);
        assert_eq!(
            decode(insn),
            Op::MulDiv { kind: MulDivKind::Mul, rd: 3, rs1: 4, rs2: 5 },
        );
    }

    #[test]
    fn decodes_rv32a_lr_w_and_sc_w() {
        // LR.W x5, (x6)  — funct5=00010, rs2=0, funct3=010, opcode=0101111 (AMO)
        let insn = (0b00010u32 << 27) | (0u32 << 20) | (6u32 << 15)
            | (0b010 << 12) | (5u32 << 7) | (OPCODE_AMO << 2) | 0b11;
        assert_eq!(
            decode(insn),
            Op::Amo { kind: AmoKind::Lr, rd: 5, rs1: 6, rs2: 0, aq: false, rl: false },
        );
        // SC.W x5, x7, (x6)
        let insn = (0b00011u32 << 27) | (7u32 << 20) | (6u32 << 15)
            | (0b010 << 12) | (5u32 << 7) | (OPCODE_AMO << 2) | 0b11;
        assert_eq!(
            decode(insn),
            Op::Amo { kind: AmoKind::Sc, rd: 5, rs1: 6, rs2: 7, aq: false, rl: false },
        );
    }

    #[test]
    fn decodes_rv32a_amoadd_w() {
        // AMOADD.W x5, x7, (x6)  — funct5=00000
        let insn = (0b00000u32 << 27) | (7u32 << 20) | (6u32 << 15)
            | (0b010 << 12) | (5u32 << 7) | (OPCODE_AMO << 2) | 0b11;
        assert_eq!(
            decode(insn),
            Op::Amo { kind: AmoKind::Add, rd: 5, rs1: 6, rs2: 7, aq: false, rl: false },
        );
    }
}

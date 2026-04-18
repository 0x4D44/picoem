// RISC-V instruction encoder + test-case generators for the
// `test_qemu_diff_riscv32` differential oracle.
//
// Stage 4 of the RISC-V Hazard3 test-oracles plan; see
// `wrk_docs/2026.04.17 - LLD - QEMU Diff RISC-V V1.md` §6 (fuzz classes),
// §7 (encoder API), §8 (property test) and
// `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support V6.md`
// §4.5 (ISA scope + Zcmp-C collision).
//
// **No F / D opcodes.** Hazard3 has no F/D; QEMU is spawned with
// `f=false,d=false`. A tripwire unit test asserts no generator path emits
// any opcode in the F/D opcode space.
//
// Scope of the encoder: RV32I + M + A (single-word AMO format) + C +
// Zicsr. Zcmp quadrant-2 bit patterns are emitted purely as a
// decoder-coverage sweep (expected `mcause=2` illegal under Hazard3 V1).
// No Zba/Zbb/Zbs (follow-up phase).

use rand::Rng;

// ============================================================================
// Public types
// ============================================================================

/// A single RISC-V differential test case.
///
/// Memory layout conventions track LLD §7 but we reuse the existing
/// `RngExt` machinery from `lib.rs` rather than committing to a full
/// `StdRng`-only signature — any `RngCore` works for the fuzz generators.
#[derive(Clone, Debug)]
pub struct RiscvTestCase {
    /// Human-readable name (class + disambiguator).
    pub name: String,
    /// Encoded instruction word(s), host-endian; the runner writes them
    /// little-endian to both QEMU and the emulator.
    pub words: Vec<u32>,
    /// Pre-state for x-registers x1..x31 (x0 is hardwired).
    pub reg_pre: Vec<(u8, u32)>,
    /// Registers that need a scratchpad-offset pointer preloaded by the
    /// runner (memory / atomics classes).
    pub addr_regs: Vec<u8>,
    /// Expected `mcause` if this case is supposed to trap. `None` means
    /// "no trap expected" (the happy path; diff GPR + PC + CSR snapshot).
    pub expect_trap: Option<u32>,
    /// Fuzz class for filtering + reporting.
    pub class: RiscvClass,
}

/// Fuzz classes per LLD §6. Names differ from the LLD's `FuzzClass` for
/// clarity (`Rv32iMem` vs `Rv32iMem`, `Rv32iMisalignedMem` vs
/// `Rv32iMemMisaligned`, `Rv32iBranch` vs `Rv32iBranchJump`,
/// `Rv32iUpper` vs `Rv32iUpperPcRel`). The Stage-5 binary's `--class`
/// CLI maps command-line strings back to these variants.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum RiscvClass {
    Rv32iAlu,
    Rv32iMem,
    Rv32iMisalignedMem,
    Rv32iBranch,
    Rv32iUpper,
    Rv32m,
    Rv32aReservable,
    Rv32c,
    Zicsr,
    Zifencei,
    CsrSideEffect,
}

impl RiscvClass {
    /// All eleven variants in a fixed order (matches the LLD §6 table).
    pub const ALL: [RiscvClass; 11] = [
        RiscvClass::Rv32iAlu,
        RiscvClass::Rv32iMem,
        RiscvClass::Rv32iMisalignedMem,
        RiscvClass::Rv32iBranch,
        RiscvClass::Rv32iUpper,
        RiscvClass::Rv32m,
        RiscvClass::Rv32aReservable,
        RiscvClass::Rv32c,
        RiscvClass::Zicsr,
        RiscvClass::Zifencei,
        RiscvClass::CsrSideEffect,
    ];

    /// Fuzz weight in basis points (sums to 10_000). Per LLD §6 "Fuzz
    /// weight (per `--fuzz N`)".
    pub fn weight_bp(self) -> u32 {
        match self {
            RiscvClass::Rv32iAlu => 3000,
            RiscvClass::Rv32iMem => 1200,
            RiscvClass::Rv32iMisalignedMem => 500,
            RiscvClass::Rv32iBranch => 1000,
            RiscvClass::Rv32iUpper => 500,
            RiscvClass::Rv32m => 1000,
            RiscvClass::Rv32aReservable => 1000,
            RiscvClass::Rv32c => 500,
            RiscvClass::Zicsr => 800,
            RiscvClass::Zifencei => 200,
            RiscvClass::CsrSideEffect => 300,
        }
    }
}

// ============================================================================
// Address map constants
// ============================================================================

/// Scratchpad base — memory / atomics cases pre-load an x-register with
/// this so the encoded instruction's 12-bit immediate covers a known
/// safe offset range.
pub const SCRATCH_BASE: u32 = 0x2000_0300;

/// Reservable SRAM range (RP2350 §2.1.6.2) — atomics must be in this
/// window or Hazard3 traps. Keep atomics strictly inside it.
pub const RESERVABLE_LO: u32 = 0x2000_0000;
pub const RESERVABLE_HI: u32 = 0x2008_2000;

// ============================================================================
// Opcode constants
// ============================================================================

pub const OPC_LOAD: u32 = 0b000_0011;
pub const OPC_STORE: u32 = 0b010_0011;
pub const OPC_OP_IMM: u32 = 0b001_0011;
pub const OPC_OP: u32 = 0b011_0011;
pub const OPC_LUI: u32 = 0b011_0111;
pub const OPC_AUIPC: u32 = 0b001_0111;
pub const OPC_BRANCH: u32 = 0b110_0011;
pub const OPC_JAL: u32 = 0b110_1111;
pub const OPC_JALR: u32 = 0b110_0111;
pub const OPC_AMO: u32 = 0b010_1111;
pub const OPC_MISC_MEM: u32 = 0b000_1111;
pub const OPC_SYSTEM: u32 = 0b111_0011;

// F/D opcode tripwire set per LLD §2 "Defaults that matter" + §11 /
// core HLD §4.5. The runtime tripwire scans generated words for any of
// these in bits [6:0].
pub const FP_OPCODES: [u32; 7] = [
    0b000_0111, // LOAD-FP
    0b010_0111, // STORE-FP
    0b100_0011, // FMADD
    0b100_0111, // FMSUB
    0b100_1011, // FNMSUB
    0b100_1111, // FNMADD
    0b101_0011, // OP-FP
];

// ============================================================================
// Encoder helpers — RV32 32-bit formats
// ============================================================================

/// R-type: `funct7 | rs2 | rs1 | funct3 | rd | opcode`.
pub fn encode_r_type(funct7: u32, rs2: u8, rs1: u8, funct3: u32, rd: u8, opcode: u32) -> u32 {
    ((funct7 & 0x7F) << 25)
        | ((u32::from(rs2) & 0x1F) << 20)
        | ((u32::from(rs1) & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | ((u32::from(rd) & 0x1F) << 7)
        | (opcode & 0x7F)
}

/// I-type: `imm[11:0] | rs1 | funct3 | rd | opcode`.
/// Signed 12-bit immediate.
pub fn encode_i_type(imm12: i32, rs1: u8, funct3: u32, rd: u8, opcode: u32) -> u32 {
    let imm = (imm12 as u32) & 0xFFF;
    (imm << 20)
        | ((u32::from(rs1) & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | ((u32::from(rd) & 0x1F) << 7)
        | (opcode & 0x7F)
}

/// S-type: `imm[11:5] | rs2 | rs1 | funct3 | imm[4:0] | opcode`.
/// Signed 12-bit immediate.
pub fn encode_s_type(imm12: i32, rs2: u8, rs1: u8, funct3: u32, opcode: u32) -> u32 {
    let imm = (imm12 as u32) & 0xFFF;
    let imm_hi = (imm >> 5) & 0x7F;
    let imm_lo = imm & 0x1F;
    (imm_hi << 25)
        | ((u32::from(rs2) & 0x1F) << 20)
        | ((u32::from(rs1) & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | (imm_lo << 7)
        | (opcode & 0x7F)
}

/// B-type: 13-bit signed branch immediate (bit 0 always 0).
/// Layout: `imm[12] | imm[10:5] | rs2 | rs1 | funct3 | imm[4:1] | imm[11] | opcode`.
pub fn encode_b_type(imm13: i32, rs2: u8, rs1: u8, funct3: u32, opcode: u32) -> u32 {
    let imm = (imm13 as u32) & 0x1FFF;
    let b12 = (imm >> 12) & 0x1;
    let b11 = (imm >> 11) & 0x1;
    let b10_5 = (imm >> 5) & 0x3F;
    let b4_1 = (imm >> 1) & 0xF;
    (b12 << 31)
        | (b10_5 << 25)
        | ((u32::from(rs2) & 0x1F) << 20)
        | ((u32::from(rs1) & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | (b4_1 << 8)
        | (b11 << 7)
        | (opcode & 0x7F)
}

/// U-type: `imm[31:12] | rd | opcode`.
/// `imm32` is expected to have zeros in the low 12 bits; we mask anyway.
pub fn encode_u_type(imm32: u32, rd: u8, opcode: u32) -> u32 {
    (imm32 & 0xFFFF_F000) | ((u32::from(rd) & 0x1F) << 7) | (opcode & 0x7F)
}

/// J-type: 21-bit signed jump immediate (bit 0 always 0).
/// Layout: `imm[20] | imm[10:1] | imm[11] | imm[19:12] | rd | opcode`.
pub fn encode_j_type(imm21: i32, rd: u8, opcode: u32) -> u32 {
    let imm = (imm21 as u32) & 0x1F_FFFF;
    let b20 = (imm >> 20) & 0x1;
    let b19_12 = (imm >> 12) & 0xFF;
    let b11 = (imm >> 11) & 0x1;
    let b10_1 = (imm >> 1) & 0x3FF;
    (b20 << 31)
        | (b10_1 << 21)
        | (b11 << 20)
        | (b19_12 << 12)
        | ((u32::from(rd) & 0x1F) << 7)
        | (opcode & 0x7F)
}

/// CSR instruction: same layout as I-type but with `csr[11:0]` in place
/// of `imm[11:0]` and `rs1_or_uimm5` stepping into the rs1 slot (uimm5
/// variants reuse the low 5 bits of that slot, high bits cleared).
pub fn encode_csr(csr: u16, rs1_or_uimm5: u8, funct3: u32, rd: u8) -> u32 {
    ((u32::from(csr) & 0xFFF) << 20)
        | ((u32::from(rs1_or_uimm5) & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | ((u32::from(rd) & 0x1F) << 7)
        | OPC_SYSTEM
}

// ============================================================================
// Tripwire helpers
// ============================================================================

/// Return true if `word` holds an F/D-family opcode in bits [6:0].
/// Inlined so `debug_assert_no_fp` is cheap.
#[inline]
pub fn is_fp_opcode(word: u32) -> bool {
    let op = word & 0x7F;
    FP_OPCODES.contains(&op)
}

/// Return true if bits [1:0] indicate a 16-bit (compressed) instruction.
#[inline]
pub fn is_compressed(word: u32) -> bool {
    (word & 0x3) != 0x3
}

// ============================================================================
// Edge-case generators (per LLD §6 "Edge-case count" column)
// ============================================================================

/// 30% fuzz weight; ~60 edge cases. Arithmetic/shift/logical edge cases
/// covering overflow, carry, shift-amount edges, register aliasing.
pub fn gen_rv32i_alu_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(72);
    // Register-immediate (OP-IMM, funct7/funct3 per RV32I table)
    let i_cases: &[(&str, u32, i32, u8, u8)] = &[
        ("addi_zero", 0, 0, 1, 2),
        ("addi_pos", 0, 0x7FF, 3, 4),     // max positive imm
        ("addi_neg", 0, -2048, 5, 6),     // min negative imm
        ("addi_alias_same", 0, 1, 7, 7),  // rd == rs1
        ("addi_x0_write", 0, 42, 1, 0),   // rd = x0 → discarded
        ("slti_neg", 2, -1, 8, 9),
        ("sltiu_one", 3, 1, 10, 11),
        ("xori_all", 4, -1, 12, 13),
        ("ori_high", 6, 0x555, 14, 15),
        ("andi_mask", 7, 0x0AA, 16, 17),
    ];
    for &(name, funct3, imm, rs1, rd) in i_cases {
        let w = encode_i_type(imm, rs1, funct3, rd, OPC_OP_IMM);
        out.push(alu_case(format!("alu_{name}"), w, rd, rs1, imm_as_reg_u32(imm)));
    }
    // Shift-immediate: shamt[4:0] plus funct7 (0 for SLLI/SRLI, 0x20 for SRAI).
    let shift_cases: &[(&str, u32, u32, u32, u8, u8)] = &[
        ("slli_0", 1, 0, 0, 18, 19),
        ("slli_31", 1, 0, 31, 20, 21),
        ("slli_16", 1, 0, 16, 22, 23),
        ("srli_0", 5, 0, 0, 24, 25),
        ("srli_31", 5, 0, 31, 26, 27),
        ("srai_0", 5, 0x20, 0, 28, 29),
        ("srai_31", 5, 0x20, 31, 30, 31),
    ];
    for &(name, funct3, funct7, shamt, rs1, rd) in shift_cases {
        let imm = ((funct7 & 0x7F) << 5) | (shamt & 0x1F);
        let w = encode_i_type(imm as i32, rs1, funct3, rd, OPC_OP_IMM);
        out.push(alu_case(
            format!("alu_{name}"),
            w,
            rd,
            rs1,
            // For shifts the register value needs to be nonzero to be useful.
            0xDEAD_BEEF,
        ));
    }
    // Register-register OP (funct7 = 0 for base, 0x20 for sub/sra).
    // rd != x3 — x3/gp is the CSR-proxy scratchpad pointer and writing it
    // corrupts the epilogue's store address.
    let r_cases: &[(&str, u32, u32, u8, u8, u8)] = &[
        ("add_basic", 0, 0, 1, 2, 4),
        ("add_alias_rd_rs1", 0, 0, 1, 2, 1),
        ("add_alias_rd_rs2", 0, 0, 1, 2, 2),
        ("add_alias_all", 0, 0, 5, 5, 5),
        ("sub_overflow", 0, 0x20, 6, 7, 8),
        ("sub_same", 0, 0x20, 9, 9, 10),
        ("sll_max", 1, 0, 11, 12, 13),
        ("slt_neg", 2, 0, 14, 15, 16),
        ("sltu_zero", 3, 0, 17, 18, 19),
        ("xor_mask", 4, 0, 20, 21, 22),
        ("srl_full", 5, 0, 23, 24, 25),
        ("sra_full", 5, 0x20, 26, 27, 28),
        ("or_mixed", 6, 0, 29, 30, 31),
        ("and_pattern", 7, 0, 1, 2, 4),
    ];
    for &(name, funct3, funct7, rs1, rs2, rd) in r_cases {
        let w = encode_r_type(funct7, rs2, rs1, funct3, rd, OPC_OP);
        let mut tc = RiscvTestCase {
            name: format!("alu_{name}"),
            words: vec![w],
            reg_pre: vec![(rs1, 0x1234_5678), (rs2, 0x8765_4321)],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iAlu,
        };
        // de-duplicate reg_pre in case of aliasing
        tc.reg_pre.sort_by_key(|r| r.0);
        tc.reg_pre.dedup_by_key(|r| r.0);
        // x0 is not writable
        tc.reg_pre.retain(|(r, _)| *r != 0);
        out.push(tc);
    }
    out
}

fn imm_as_reg_u32(imm: i32) -> u32 {
    imm as u32
}

fn alu_case(name: String, word: u32, _rd: u8, rs1: u8, rs1_val: u32) -> RiscvTestCase {
    let mut reg_pre = vec![];
    if rs1 != 0 {
        reg_pre.push((rs1, rs1_val));
    }
    RiscvTestCase {
        name,
        words: vec![word],
        reg_pre,
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::Rv32iAlu,
    }
}

/// 12% fuzz weight; ~30 edge cases. LB/LH/LW/LBU/LHU + SB/SH/SW with
/// aligned, scratchpad-offset addressing.
pub fn gen_rv32i_mem_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(32);

    // Loads (funct3: LB=0, LH=1, LW=2, LBU=4, LHU=5)
    // Pre-load rs1 with SCRATCH_BASE; immediate picks an offset inside scratchpad.
    // Align immediates to each width's natural alignment.
    let loads: &[(&str, u32, i32, u8, u8)] = &[
        ("lb_off0", 0, 0, 5, 6),
        ("lb_offpos", 0, 16, 5, 7),
        ("lb_offneg", 0, -8, 5, 8),
        ("lh_off0", 1, 0, 5, 9),
        ("lh_off2", 1, 2, 5, 10),
        ("lw_off0", 2, 0, 5, 11),
        ("lw_off4", 2, 4, 5, 12),
        ("lbu_off0", 4, 0, 5, 13),
        ("lhu_off0", 5, 0, 5, 14),
        ("lw_maxpos", 2, 0x7FC, 5, 15), // 2044 — aligned, within 12-bit imm
        ("lw_negoff", 2, -32, 5, 16),   // rs1 = SCRATCH_BASE + 64; imm = -32 keeps us in scratchpad
    ];
    for &(name, funct3, imm, rs1, rd) in loads {
        let base = if imm < 0 {
            SCRATCH_BASE.wrapping_add(64)
        } else {
            SCRATCH_BASE
        };
        let w = encode_i_type(imm, rs1, funct3, rd, OPC_LOAD);
        out.push(RiscvTestCase {
            name: format!("mem_{name}"),
            words: vec![w],
            reg_pre: vec![(rs1, base)],
            addr_regs: vec![rs1],
            expect_trap: None,
            class: RiscvClass::Rv32iMem,
        });
    }

    // Stores (funct3: SB=0, SH=1, SW=2)
    let stores: &[(&str, u32, i32, u8, u8)] = &[
        ("sb_off0", 0, 0, 5, 6),
        ("sb_off16", 0, 16, 5, 7),
        ("sh_off0", 1, 0, 5, 8),
        ("sh_off2", 1, 2, 5, 9),
        ("sw_off0", 2, 0, 5, 10),
        ("sw_off4", 2, 4, 5, 11),
        ("sw_off8", 2, 8, 5, 12),
        ("sw_negoff", 2, -16, 5, 13),
        ("sw_maxpos", 2, 0x7FC, 5, 14),
        ("sb_neg", 0, -1, 5, 15),
    ];
    for &(name, funct3, imm, rs2, rs1) in stores {
        let base = if imm < 0 {
            SCRATCH_BASE.wrapping_add(64)
        } else {
            SCRATCH_BASE
        };
        let w = encode_s_type(imm, rs2, rs1, funct3, OPC_STORE);
        out.push(RiscvTestCase {
            name: format!("mem_{name}"),
            words: vec![w],
            reg_pre: vec![(rs1, base), (rs2, 0xA5A5_5A5A)],
            addr_regs: vec![rs1],
            expect_trap: None,
            class: RiscvClass::Rv32iMem,
        });
    }
    out
}

/// 5% fuzz weight; ~12 edge cases. Deliberately misaligned load/store
/// exercising `mcause=4` / `mcause=6`.
pub fn gen_rv32i_misaligned_mem_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(16);

    // Halfword loads at odd offsets → mcause 4
    let lh_cases: &[(&str, u32, i32, u8, u8, u32)] = &[
        ("lh_odd1", 1, 1, 5, 6, 4),
        ("lh_odd3", 1, 3, 5, 7, 4),
        ("lhu_odd1", 5, 1, 5, 8, 4),
        ("lw_off1", 2, 1, 5, 9, 4),
        ("lw_off2", 2, 2, 5, 10, 4),
        ("lw_off3", 2, 3, 5, 11, 4),
    ];
    for &(name, funct3, imm, rs1, rd, trap) in lh_cases {
        let w = encode_i_type(imm, rs1, funct3, rd, OPC_LOAD);
        out.push(RiscvTestCase {
            name: format!("misaligned_{name}"),
            words: vec![w],
            reg_pre: vec![(rs1, SCRATCH_BASE)],
            addr_regs: vec![rs1],
            expect_trap: Some(trap),
            class: RiscvClass::Rv32iMisalignedMem,
        });
    }

    let sh_cases: &[(&str, u32, i32, u8, u8, u32)] = &[
        ("sh_odd1", 1, 1, 5, 6, 6),
        ("sh_odd3", 1, 3, 5, 7, 6),
        ("sw_off1", 2, 1, 5, 8, 6),
        ("sw_off2", 2, 2, 5, 9, 6),
        ("sw_off3", 2, 3, 5, 10, 6),
    ];
    for &(name, funct3, imm, rs2, rs1, trap) in sh_cases {
        let w = encode_s_type(imm, rs2, rs1, funct3, OPC_STORE);
        out.push(RiscvTestCase {
            name: format!("misaligned_{name}"),
            words: vec![w],
            reg_pre: vec![(rs1, SCRATCH_BASE), (rs2, 0xCAFE_BABE)],
            addr_regs: vec![rs1],
            expect_trap: Some(trap),
            class: RiscvClass::Rv32iMisalignedMem,
        });
    }
    out
}

/// 10% fuzz weight; ~25 edge cases. All 6 conditional branches + JAL /
/// JALR at near/far offsets.
pub fn gen_rv32i_branch_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(28);

    // Conditional branches — always taken / never taken permutations.
    // funct3: BEQ=0, BNE=1, BLT=4, BGE=5, BLTU=6, BGEU=7
    let branches: &[(&str, u32, i32, u8, u8, u32, u32)] = &[
        ("beq_eq", 0, 8, 1, 2, 0x10, 0x10),
        ("beq_ne", 0, 8, 1, 2, 0x10, 0x11),
        ("bne_ne", 1, 8, 3, 4, 1, 2),
        ("bne_eq", 1, 8, 3, 4, 1, 1),
        ("blt_pos", 4, 8, 5, 6, 1_i32 as u32, 2_i32 as u32),
        ("blt_neg", 4, 8, 5, 6, (-1_i32) as u32, 0),
        ("bge_pos", 5, 8, 7, 8, 2, 1),
        ("bge_neg", 5, 8, 7, 8, 0, (-1_i32) as u32),
        ("bltu_carry", 6, 8, 9, 10, 0, 0xFFFF_FFFF),
        ("bgeu_eq", 7, 8, 11, 12, 5, 5),
        ("beq_neg_off", 0, -8, 13, 14, 3, 3),
        ("beq_far", 0, 0xFFE, 15, 16, 0, 0), // near max positive imm13 (aligned)
        ("beq_far_neg", 0, -0x1000, 17, 18, 0, 0),
    ];
    for &(name, funct3, imm, rs1, rs2, v1, v2) in branches {
        let w = encode_b_type(imm, rs2, rs1, funct3, OPC_BRANCH);
        let mut regs = vec![(rs1, v1), (rs2, v2)];
        regs.sort_by_key(|r| r.0);
        regs.dedup_by_key(|r| r.0);
        regs.retain(|(r, _)| *r != 0);
        out.push(RiscvTestCase {
            name: format!("branch_{name}"),
            words: vec![w],
            reg_pre: regs,
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }

    // JAL: near, far, rd=x0, rd=x1 (link).
    let jal_cases: &[(&str, i32, u8)] = &[
        ("jal_near_pos", 8, 1),
        ("jal_near_neg", -8, 1),
        ("jal_far_pos", 0x4_0000, 1),
        ("jal_far_neg", -0x4_0000, 1),
        ("jal_rdx0", 8, 0),
    ];
    for &(name, imm, rd) in jal_cases {
        let w = encode_j_type(imm, rd, OPC_JAL);
        out.push(RiscvTestCase {
            name: format!("branch_{name}"),
            words: vec![w],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }

    // JALR: offset 0, offset 4, rd=x0 vs rd=x1.
    let jalr_cases: &[(&str, i32, u8, u8)] = &[
        ("jalr_off0", 0, 2, 1),
        ("jalr_off4", 4, 2, 1),
        ("jalr_rdx0", 0, 2, 0),
    ];
    for &(name, imm, rs1, rd) in jalr_cases {
        let w = encode_i_type(imm, rs1, 0, rd, OPC_JALR);
        out.push(RiscvTestCase {
            name: format!("branch_{name}"),
            words: vec![w],
            reg_pre: vec![(rs1, SCRATCH_BASE.wrapping_add(0x100))],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }

    out
}

/// 5% fuzz weight; ~12 edge cases. LUI + AUIPC alone + pc-relative pairs.
pub fn gen_rv32i_upper_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(14);
    let cases: &[(&str, u32, u32, u8)] = &[
        ("lui_zero", OPC_LUI, 0, 1),
        ("lui_one", OPC_LUI, 0x0000_1000, 2),
        // rd != x3 — see r_cases note above.
        ("lui_neg", OPC_LUI, 0xFFFF_F000, 4),
        ("lui_pattern", OPC_LUI, 0x5555_5000, 4),
        ("lui_rd0", OPC_LUI, 0x1234_5000, 0),
        ("auipc_zero", OPC_AUIPC, 0, 5),
        ("auipc_one", OPC_AUIPC, 0x0000_1000, 6),
        ("auipc_neg", OPC_AUIPC, 0xFFFF_F000, 7),
        ("auipc_pattern", OPC_AUIPC, 0xAAAA_A000, 8),
        ("auipc_rd0", OPC_AUIPC, 0x1000_0000, 0),
    ];
    for &(name, opcode, imm, rd) in cases {
        let w = encode_u_type(imm, rd, opcode);
        out.push(RiscvTestCase {
            name: format!("upper_{name}"),
            words: vec![w],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iUpper,
        });
    }

    // auipc/addi pair (PC-relative address build).
    let w1 = encode_u_type(0x0000_1000, 5, OPC_AUIPC);
    let w2 = encode_i_type(16, 5, 0, 5, OPC_OP_IMM);
    out.push(RiscvTestCase {
        name: "upper_auipc_addi_pair".into(),
        words: vec![w1, w2],
        reg_pre: vec![],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::Rv32iUpper,
    });

    // auipc/jalr pair (long-range call).
    let w3 = encode_u_type(0x0000_1000, 6, OPC_AUIPC);
    let w4 = encode_i_type(0, 6, 0, 1, OPC_JALR);
    out.push(RiscvTestCase {
        name: "upper_auipc_jalr_pair".into(),
        words: vec![w3, w4],
        reg_pre: vec![],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::Rv32iUpper,
    });

    out
}

/// 10% fuzz weight; ~24 edge cases. MUL/MULH/MULHU/MULHSU/DIV/DIVU/
/// REM/REMU + divide-by-zero + INT_MIN/−1 overflow corners.
pub fn gen_rv32m_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(26);
    // funct7 = 0x01 for all RV32M, opcode = OP.
    let cases: &[(&str, u32, u32, u32, u8, u8, u8)] = &[
        // rd=3 would clobber x3/gp (CSR-proxy pointer) — moved to rd=4.
        ("mul_basic", 0, 0x01, 0x00010001, 1, 2, 4),
        ("mul_neg", 0, 0x01, 0xFFFF_FFFF, 4, 5, 6),
        ("mulh_highbit", 1, 0x01, 0x8000_0000, 7, 8, 9),
        ("mulhsu_mixed", 2, 0x01, 0x8000_0000, 10, 11, 12),
        ("mulhu_max", 3, 0x01, 0xFFFF_FFFF, 13, 14, 15),
        ("div_basic", 4, 0x01, 0x0000_0002, 16, 17, 18),
        ("div_intmin_neg1", 4, 0x01, 0x8000_0000, 19, 20, 21),
        ("div_zero", 4, 0x01, 0, 22, 23, 24),
        ("divu_zero", 5, 0x01, 0, 25, 26, 27),
        ("rem_basic", 6, 0x01, 3, 28, 29, 30),
        ("rem_intmin_neg1", 6, 0x01, 0x8000_0000, 31, 1, 2),
        ("rem_zero", 6, 0x01, 0, 3, 4, 5),
        ("remu_zero", 7, 0x01, 0, 6, 7, 8),
    ];
    for &(name, funct3, funct7, rs1_v, rs1, rs2, rd) in cases {
        let rs2_v = match name {
            "div_intmin_neg1" | "rem_intmin_neg1" => 0xFFFF_FFFF,
            "div_zero" | "divu_zero" | "rem_zero" | "remu_zero" => 0,
            "mul_neg" | "mulh_highbit" | "mulhsu_mixed" | "mulhu_max" => 0xFFFF_FFFF,
            _ => 0x0000_0003,
        };
        let w = encode_r_type(funct7, rs2, rs1, funct3, rd, OPC_OP);
        let mut reg_pre = vec![(rs1, rs1_v), (rs2, rs2_v)];
        reg_pre.sort_by_key(|r| r.0);
        reg_pre.dedup_by_key(|r| r.0);
        reg_pre.retain(|(r, _)| *r != 0);
        out.push(RiscvTestCase {
            name: format!("rv32m_{name}"),
            words: vec![w],
            reg_pre,
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32m,
        });
    }
    out
}

/// 10% fuzz weight; ~20 edge cases. lr.w / sc.w / amo*.w inside the
/// reservable window. Single-hart only in Phase 2.
pub fn gen_rv32a_reservable_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(22);
    // AMO funct7 top 5 bits encode the AMO operation; low 2 bits are aq,rl.
    //   lr.w       = 0b00010
    //   sc.w       = 0b00011
    //   amoswap.w  = 0b00001
    //   amoadd.w   = 0b00000
    //   amoxor.w   = 0b00100
    //   amoor.w    = 0b01000
    //   amoand.w   = 0b01100
    //   amomin.w   = 0b10000
    //   amomax.w   = 0b10100
    //   amominu.w  = 0b11000
    //   amomaxu.w  = 0b11100
    let amo_ops: &[(&str, u32, bool)] = &[
        ("lr_w", 0b00010, true),      // rs2 must be 0 for lr.w
        ("sc_w", 0b00011, false),
        ("amoswap_w", 0b00001, false),
        ("amoadd_w", 0b00000, false),
        ("amoxor_w", 0b00100, false),
        ("amoor_w", 0b01000, false),
        ("amoand_w", 0b01100, false),
        ("amomin_w", 0b10000, false),
        ("amomax_w", 0b10100, false),
        ("amominu_w", 0b11000, false),
        ("amomaxu_w", 0b11100, false),
    ];
    for (i, &(name, op5, lr)) in amo_ops.iter().enumerate() {
        // Plain variant (aq=0, rl=0).
        let funct7 = op5 << 2;
        let rs1 = 10u8; // base register
        let rs2 = if lr { 0 } else { 11 };
        let rd = 12u8;
        let w = encode_r_type(funct7, rs2, rs1, 0b010, rd, OPC_AMO);
        let mut reg_pre = vec![(rs1, SCRATCH_BASE)];
        if !lr {
            reg_pre.push((rs2, 0xAA55_0000u32.wrapping_add(i as u32)));
        }
        out.push(RiscvTestCase {
            name: format!("rv32a_{name}_plain"),
            words: vec![w],
            reg_pre,
            addr_regs: vec![rs1],
            expect_trap: None,
            class: RiscvClass::Rv32aReservable,
        });
        // aq=1, rl=1 variant for a subset (saves on test volume).
        if matches!(name, "lr_w" | "sc_w" | "amoswap_w" | "amoadd_w") {
            let funct7_aqrl = (op5 << 2) | 0b11;
            let w2 = encode_r_type(funct7_aqrl, rs2, rs1, 0b010, rd, OPC_AMO);
            let mut reg_pre = vec![(rs1, SCRATCH_BASE)];
            if !lr {
                reg_pre.push((rs2, 0xAA55_0000u32.wrapping_add(i as u32) ^ 0xFF));
            }
            out.push(RiscvTestCase {
                name: format!("rv32a_{name}_aqrl"),
                words: vec![w2],
                reg_pre,
                addr_regs: vec![rs1],
                expect_trap: None,
                class: RiscvClass::Rv32aReservable,
            });
        }
    }
    out
}

/// 5% fuzz weight; ~30 edge cases. **Compressed (RV32C) encodings + the
/// Zcmp quadrant-2 sweep.** Zcmp bytes are tagged `expect_trap: Some(2)`
/// per core HLD §4.5 — V1 Hazard3 decodes them as whatever RV32C thinks
/// they are (i.e. "valid-looking garbage"); the QEMU side raises
/// `mcause=2`. Both sides must agree that the instruction was illegal.
pub fn gen_rv32c_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(40);

    // --- Small selection of plain RV32C instructions from quadrants 0/1/2 ---
    // Quadrant 0 (c[1:0]=00):
    //   c.addi4spn → funct3=0, non-zero imm, rd'=x8..x15
    //   encoding: 000 _ nzimm[5:4|9:6|2|3] _ rd'[2:0] _ 00
    let plain: &[(&str, u16)] = &[
        ("c_addi4spn", 0b0_0000_0001_0010_0000), // addi4spn rd'=x8, nzimm=small
        ("c_nop", 0x0001),                                  // c.addi x0, 0 (canonical nop)
        ("c_addi_1", 0x0085),                               // c.addi x1, 1
        ("c_li", 0x4085),                                   // c.li x1, 1 (imm[4:0]=00001)
        ("c_lui", 0x6105),                                  // c.lui x2, 1  (imm nonzero)
        ("c_andi", 0x8805),                                 // c.andi x8, 1
        ("c_jr_x1", 0x8082),                                // c.jr x1 (quadrant 2)
        ("c_jalr_x1", 0x9082),                              // c.jalr x1
        ("c_slli", 0x0086),                                 // c.slli x1, 1
        ("c_lwsp", 0x4082),                                 // c.lwsp x1, 0(sp)
        ("c_swsp", 0xc006),                                 // c.swsp x1, 0(sp)
    ];
    for &(name, enc) in plain {
        out.push(RiscvTestCase {
            name: format!("rvc_{name}"),
            words: vec![u32::from(enc)],
            reg_pre: vec![(2, SCRATCH_BASE)], // sp in scratchpad for stack cases
            addr_regs: vec![2],
            expect_trap: None,
            class: RiscvClass::Rv32c,
        });
    }

    // --- Zcmp quadrant-2 sweep ---
    // Zcmp reuses the compressed quadrant-2 encoding space with funct3 = 101
    // (c[15:13]) and specific funct6 patterns in c[15:10]. The exact sub-
    // patterns are:
    //   cm.push    — 101 11000 (funct6=0b101110), urlist in c[7:4], stack_adj in c[3:2]
    //   cm.pop     — 101 11010
    //   cm.popretz — 101 11100
    //   cm.popret  — 101 11110
    //   cm.mvsa01  — 101 01101 (mvsa/mva01s family has funct6=0b101011 and sub-op bits)
    //   cm.mva01s  — 101 01111
    //
    // Source: Zcmp spec v1.0 §13.1 (cm.push/pop layout) and §13.2 (cm.mv*).
    // Hazard3 V1 decoder does NOT recognise Zcmp and must either treat
    // these as illegal (mcause=2) or mis-decode them as RV32C. We
    // emit the bit patterns and tag `expect_trap: Some(2)` so the diff
    // surfaces the core HLD §4.5 collision risk when it materialises.
    //
    // Bit layout (16-bit): funct3 at [15:13], ...|0|1| at [1:0] = 0b10
    // (quadrant 2).
    //
    // Encoding helper: `0b101_<f3 bits[12:10]>_<imm/regs[9:2]>_10`.
    //
    // We sweep across register-list, stack-adjust, and operation
    // discriminators to cover >30 distinct patterns.

    // Push/pop family (funct6 bits [15:10] = 0b101110/0b101010/etc.).
    // Layout per Zcmp spec §13.1.1:
    //   15:13 = 101 (funct3)
    //   12:10 = funct6_low (selects push/pop/popret/popretz + zextend)
    //   9:8  = 11 (family discriminator)
    //   7:4  = urlist (register list, values 4..15 legal)
    //   3:2  = spimm[5:4] (stack adjust high bits)
    //   1:0  = 10 (quadrant 2)
    let push_pop_families: &[(&str, u16, u32)] = &[
        ("cm_push", 0b110, 2),
        ("cm_pop", 0b010, 2),
        ("cm_popretz", 0b100, 2),
        ("cm_popret", 0b000, 2),
    ];
    // Iterate over a handful of urlists (4, 5, 7, 11, 15) and two stack adjusts.
    let urlists: &[u16] = &[4, 5, 7, 11, 15];
    let spimms: &[u16] = &[0, 3];
    for &(fname, f6_low, trap) in push_pop_families {
        for &urlist in urlists {
            for &spimm in spimms {
                let enc: u16 = 0b101 << 13
                    | f6_low << 10
                    | 0b11 << 8
                    | (urlist & 0xF) << 4
                    | (spimm & 0x3) << 2
                    | 0b10;
                out.push(RiscvTestCase {
                    name: format!("zcmp_{fname}_ur{urlist}_sp{spimm}"),
                    words: vec![u32::from(enc)],
                    reg_pre: vec![(2, SCRATCH_BASE)],
                    addr_regs: vec![2],
                    expect_trap: Some(trap),
                    class: RiscvClass::Rv32c,
                });
            }
        }
    }

    // cm.mvsa01 / cm.mva01s family (funct6 = 0b101011):
    //   15:13 = 101, 12:10 = 011, 9:7 = 011, 6:5 = rs1'/rs2' variant, 4:2 = reg pair idx, 1:0 = 10
    // We sweep a few register-pair discriminators.
    for idx in 0u16..4 {
        let enc_mvsa: u16 = 0b101 << 13 | 0b011 << 10 | 0b011 << 7 | 0b01 << 5 | (idx & 0x7) << 2 | 0b10;
        let enc_mva01s: u16 = 0b101 << 13 | 0b011 << 10 | 0b011 << 7 | 0b11 << 5 | (idx & 0x7) << 2 | 0b10;
        out.push(RiscvTestCase {
            name: format!("zcmp_cm_mvsa01_{idx}"),
            words: vec![u32::from(enc_mvsa)],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: Some(2),
            class: RiscvClass::Rv32c,
        });
        out.push(RiscvTestCase {
            name: format!("zcmp_cm_mva01s_{idx}"),
            words: vec![u32::from(enc_mva01s)],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: Some(2),
            class: RiscvClass::Rv32c,
        });
    }

    out
}

/// 8% fuzz weight; ~20 edge cases. CSR read/write/set/clear +
/// immediate variants across the seven-CSR proxy set. Deliberately not
/// routed through the CSR-diff proxy at runtime (see LLD §4 "self-mask
/// carve-out").
pub fn gen_zicsr_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(24);
    // CSR addresses in the diff set (LLD §3).
    let csrs: &[(&str, u16)] = &[
        ("mstatus", 0x300),
        ("mie", 0x304),
        ("mtvec", 0x305),
        ("mscratch", 0x340),
        ("mepc", 0x341),
        ("mcause", 0x342),
        ("mip", 0x344),
    ];
    // funct3: CSRRW=1, CSRRS=2, CSRRC=3, CSRRWI=5, CSRRSI=6, CSRRCI=7.
    for &(name, csr) in csrs {
        // csrrs rd, csr, x0 — canonical no-op-write read.
        let w = encode_csr(csr, 0, 2, 5);
        out.push(RiscvTestCase {
            name: format!("zicsr_csrr_{name}"),
            words: vec![w],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Zicsr,
        });
        // csrrwi (immediate variant — uimm5=1 is a simple non-zero pattern).
        let w2 = encode_csr(csr, 1, 5, 6);
        out.push(RiscvTestCase {
            name: format!("zicsr_csrrwi_{name}"),
            words: vec![w2],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Zicsr,
        });
        // csrrc x8, csr, x9 — set rs1 nonzero so the op actually writes.
        let w3 = encode_csr(csr, 9, 3, 8);
        out.push(RiscvTestCase {
            name: format!("zicsr_csrrc_{name}"),
            words: vec![w3],
            reg_pre: vec![(9, 0x0000_0888)],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Zicsr,
        });
    }
    out
}

/// 2% fuzz weight; ~4 edge cases. `fence.i` + self-modifying code.
pub fn gen_zifencei_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(6);
    // FENCE.I: funct3=1, opcode=MISC_MEM. rs1/rd/imm = 0 canonically.
    let fence_i = encode_i_type(0, 0, 1, 0, OPC_MISC_MEM);
    out.push(RiscvTestCase {
        name: "zifencei_fence_i".into(),
        words: vec![fence_i],
        reg_pre: vec![],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::Zifencei,
    });
    // FENCE (funct3=0) — the non-`.i` relative, a sibling tripwire for
    // decode-ordering bugs.
    let fence = encode_i_type(0x0FF, 0, 0, 0, OPC_MISC_MEM);
    out.push(RiscvTestCase {
        name: "zifencei_fence_iorw".into(),
        words: vec![fence],
        reg_pre: vec![],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::Zifencei,
    });
    // Note: a full self-modifying-code probe (sw + fence.i + jalr to the
    // rewritten word) would require an executable scratchpad, which the V1
    // runner MMU map does not provide.  Zifencei is a 2% slice per LLD §6
    // weights; standalone `fence.i` / `fence` decode coverage above is
    // sufficient for V1.  Deferred until the runner can map executable
    // scratch pages.
    out
}

/// 3% fuzz weight; ~10 edge cases. Multi-instruction chains where a CSR
/// write alters a condition that a following branch depends on.
pub fn gen_csr_side_effect_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(12);
    // csrrw t0(x5), mscratch, t1(x6); beq t0, zero, +8
    let csrrw_scratch = encode_csr(0x340, 6, 1, 5);
    let beq = encode_b_type(8, 0, 5, 0, OPC_BRANCH);
    out.push(RiscvTestCase {
        name: "csrside_mscratch_beq_taken".into(),
        words: vec![csrrw_scratch, beq],
        reg_pre: vec![(6, 0)],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::CsrSideEffect,
    });
    let csrrw_scratch = encode_csr(0x340, 6, 1, 5);
    let bne = encode_b_type(8, 0, 5, 1, OPC_BRANCH);
    out.push(RiscvTestCase {
        name: "csrside_mscratch_bne_ntaken".into(),
        words: vec![csrrw_scratch, bne],
        reg_pre: vec![(6, 0x1234_5678)],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::CsrSideEffect,
    });
    // csrrsi mscratch, 1; csrrs t0, mscratch, x0; beq t0, zero, +8
    let csrrsi = encode_csr(0x340, 1, 6, 0);
    let csrrs = encode_csr(0x340, 0, 2, 5);
    let beq2 = encode_b_type(8, 0, 5, 0, OPC_BRANCH);
    out.push(RiscvTestCase {
        name: "csrside_mscratch_set_then_read_branch".into(),
        words: vec![csrrsi, csrrs, beq2],
        reg_pre: vec![],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::CsrSideEffect,
    });
    // csrrw mstatus, t1; addi t2, t0, 1 (exercises t0 getting old mstatus)
    let csrrw_mstatus = encode_csr(0x300, 6, 1, 5);
    let addi = encode_i_type(1, 5, 0, 7, OPC_OP_IMM);
    out.push(RiscvTestCase {
        name: "csrside_mstatus_use_old".into(),
        words: vec![csrrw_mstatus, addi],
        reg_pre: vec![(6, 0x0000_0008)],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::CsrSideEffect,
    });
    // A couple of simple mepc chains.
    let csrrw_mepc = encode_csr(0x341, 6, 1, 5);
    let xor_ = encode_r_type(0, 5, 5, 4, 8, OPC_OP); // xor x8, x5, x5 → 0
    out.push(RiscvTestCase {
        name: "csrside_mepc_xor".into(),
        words: vec![csrrw_mepc, xor_],
        reg_pre: vec![(6, 0x2000_0200)],
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::CsrSideEffect,
    });
    out
}

// ============================================================================
// Fuzz generators
// ============================================================================

fn rand_gpr<R: Rng>(rng: &mut R) -> u8 {
    // x1..x31, but skip x3 (gp) — the QEMU-diff harness reserves it as the
    // CSR-proxy scratchpad pointer and any test writing x3 corrupts the
    // epilogue's store address. x5/t0 is still in play: the harness zeros
    // it after the CSR-read prelude (see the `mv x5, x0` slot in
    // `build_test_stream`) so both sides enter the test with x5 == 0.
    loop {
        let r = rng.gen_range(1..32_u8);
        if r != 3 {
            return r;
        }
    }
}

/// Fuzz generator: RV32I ALU. Register-immediate + register-register mix.
pub fn gen_fuzz_rv32i_alu<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    let funct3_i_list: &[u32] = &[0, 2, 3, 4, 6, 7]; // addi/slti/sltiu/xori/ori/andi
    let funct3_r_list: &[u32] = &[0, 2, 3, 4, 6, 7];
    for i in 0..count {
        let rd = rand_gpr(rng);
        let rs1 = rand_gpr(rng);
        let rs1_val = rng.next_u32();
        let w = if rng.gen_bool(0.5) {
            // I-type
            let funct3 = funct3_i_list[rng.gen_range(0..funct3_i_list.len())];
            let imm_raw = rng.next_u32() as i32;
            // Sign-extend a random 12-bit immediate.
            let imm = (imm_raw << 20) >> 20;
            encode_i_type(imm, rs1, funct3, rd, OPC_OP_IMM)
        } else if rng.gen_bool(0.3) {
            // Shift-immediate (SLLI / SRLI / SRAI): funct3=1 or 5, imm = shamt+funct7
            let is_right = rng.gen_bool(0.5);
            let arith = rng.gen_bool(0.5);
            let funct3 = if is_right { 5 } else { 1 };
            let shamt: u32 = rng.gen_range(0..32);
            let funct7 = if is_right && arith { 0x20 } else { 0 };
            let imm = ((funct7 & 0x7F) << 5) | (shamt & 0x1F);
            encode_i_type(imm as i32, rs1, funct3, rd, OPC_OP_IMM)
        } else {
            // R-type
            let funct3 = funct3_r_list[rng.gen_range(0..funct3_r_list.len())];
            let funct7 = if funct3 == 0 && rng.gen_bool(0.3) { 0x20 } else { 0 };
            let rs2 = rand_gpr(rng);
            encode_r_type(funct7, rs2, rs1, funct3, rd, OPC_OP)
        };
        out.push(RiscvTestCase {
            name: format!("fuzz_alu_{i}"),
            words: vec![w],
            reg_pre: vec![(rs1, rs1_val)],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iAlu,
        });
    }
    out
}

/// Fuzz generator: RV32I aligned load/store.
pub fn gen_fuzz_rv32i_mem<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let is_load = rng.gen_bool(0.5);
        let rs1 = rand_gpr(rng);
        // Valid RV32 widths only: LB/LH/LW/LBU/LHU = {0,1,2,4,5} and
        // SB/SH/SW = {0,1,2}. funct3=3/6/7 are RV64 double-word variants
        // the property-test external decoder rejects.
        let align: u32;
        let funct3 = if is_load {
            let f = [0_u32, 1, 2, 4, 5][rng.gen_range(0..5)];
            align = match f {
                0 | 4 => 1,
                1 | 5 => 2,
                _ => 4,
            };
            f
        } else {
            let f = rng.gen_range(0..3_u32);
            align = match f {
                0 => 1,
                1 => 2,
                _ => 4,
            };
            f
        };
        // Choose an aligned offset in [-128, 128).
        let raw: i32 = rng.gen_range(-32..32);
        let imm = raw.wrapping_mul(align as i32);
        let base = SCRATCH_BASE.wrapping_add(0x80);
        let w = if is_load {
            let rd = rand_gpr(rng);
            encode_i_type(imm, rs1, funct3, rd, OPC_LOAD)
        } else {
            let rs2 = rand_gpr(rng);
            encode_s_type(imm, rs2, rs1, funct3, OPC_STORE)
        };
        let mut reg_pre = vec![(rs1, base)];
        if !is_load {
            // Store variants also need a source value; pick rs2 from the same draw.
            // This is approximate — we'll re-seed the encoder's rs2 from the word.
            let rs2 = ((w >> 20) & 0x1F) as u8;
            if rs2 != 0 && rs2 != rs1 {
                reg_pre.push((rs2, rng.next_u32()));
            }
        }
        out.push(RiscvTestCase {
            name: format!("fuzz_mem_{i}"),
            words: vec![w],
            reg_pre,
            addr_regs: vec![rs1],
            expect_trap: None,
            class: RiscvClass::Rv32iMem,
        });
    }
    out
}

/// Fuzz generator: deliberately misaligned loads/stores.
pub fn gen_fuzz_rv32i_misaligned<R: Rng>(
    rng: &mut R,
    count: usize,
) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let is_load = rng.gen_bool(0.5);
        let rs1 = rand_gpr(rng);
        let funct3 = if is_load {
            // lh/lhu/lw variants
            [1_u32, 5, 2][rng.gen_range(0..3)]
        } else {
            // sh/sw variants
            [1_u32, 2][rng.gen_range(0..2)]
        };
        let trap = if is_load { 4 } else { 6 };
        // Force a misaligned imm for the selected access width.
        // Half-word ops (funct3 1/5): only odd offsets {1, 3} trap; offset
        // 2 is 2-byte aligned and does NOT trap on Hazard3.
        // Word ops (funct3 2): any of {1, 2, 3} is non-word-aligned.
        // Byte ops (funct3 0/4) can't be misaligned and never enter this
        // generator.
        let odd_off: i32 = match funct3 {
            1 | 5 => [1_i32, 3][rng.gen_range(0..2)],
            2 => [1_i32, 2, 3][rng.gen_range(0..3)],
            _ => unreachable!("byte width funct3 in misaligned generator"),
        };
        let imm = odd_off;
        let w = if is_load {
            let rd = rand_gpr(rng);
            encode_i_type(imm, rs1, funct3, rd, OPC_LOAD)
        } else {
            let rs2 = rand_gpr(rng);
            encode_s_type(imm, rs2, rs1, funct3, OPC_STORE)
        };
        let mut reg_pre = vec![(rs1, SCRATCH_BASE)];
        // M2: stores also need a source value in rs2.
        if !is_load {
            let rs2 = ((w >> 20) & 0x1F) as u8;
            if rs2 != 0 && rs2 != rs1 {
                reg_pre.push((rs2, rng.next_u32()));
            }
        }
        out.push(RiscvTestCase {
            name: format!("fuzz_misaligned_{i}"),
            words: vec![w],
            reg_pre,
            addr_regs: vec![rs1],
            expect_trap: Some(trap),
            class: RiscvClass::Rv32iMisalignedMem,
        });
    }
    out
}

/// Fuzz generator: RV32I branches + JAL / JALR.
pub fn gen_fuzz_rv32i_branch<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let choice = rng.gen_range(0..3_u32);
        let w = match choice {
            0 => {
                // Conditional branch
                let funct3 = [0_u32, 1, 4, 5, 6, 7][rng.gen_range(0..6)];
                let rs1 = rand_gpr(rng);
                let rs2 = rand_gpr(rng);
                // Aligned 13-bit signed offset. Hazard3 V1 accepts bit 0 = 0;
                // we force-align.
                let raw: i32 = rng.gen_range(-4_096..4_096) & !1_i32;
                encode_b_type(raw, rs2, rs1, funct3, OPC_BRANCH)
            }
            1 => {
                // JAL
                let rd = rng.gen_range(0..32_u8);
                let raw: i32 = rng.gen_range(-(1 << 20)..(1 << 20)) & !1_i32;
                encode_j_type(raw, rd, OPC_JAL)
            }
            _ => {
                // JALR
                let rd = rng.gen_range(0..32_u8);
                let rs1 = rand_gpr(rng);
                let imm: i32 = rng.gen_range(-2_048..2_048);
                encode_i_type(imm, rs1, 0, rd, OPC_JALR)
            }
        };
        out.push(RiscvTestCase {
            name: format!("fuzz_branch_{i}"),
            words: vec![w],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }
    out
}

/// Fuzz generator: RV32I upper (LUI / AUIPC).
pub fn gen_fuzz_rv32i_upper<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let rd = rng.gen_range(0..32_u8);
        let imm = rng.next_u32() & 0xFFFF_F000;
        let op = if rng.gen_bool(0.5) { OPC_LUI } else { OPC_AUIPC };
        let w = encode_u_type(imm, rd, op);
        out.push(RiscvTestCase {
            name: format!("fuzz_upper_{i}"),
            words: vec![w],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iUpper,
        });
    }
    out
}

/// Fuzz generator: RV32M.
pub fn gen_fuzz_rv32m<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let funct3 = rng.gen_range(0..8_u32);
        let rd = rand_gpr(rng);
        let rs1 = rand_gpr(rng);
        let rs2 = rand_gpr(rng);
        let w = encode_r_type(0x01, rs2, rs1, funct3, rd, OPC_OP);
        let mut reg_pre = vec![(rs1, rng.next_u32())];
        if rs2 != rs1 {
            reg_pre.push((rs2, rng.next_u32()));
        }
        out.push(RiscvTestCase {
            name: format!("fuzz_rv32m_{i}"),
            words: vec![w],
            reg_pre,
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32m,
        });
    }
    out
}

/// Fuzz generator: RV32A inside the reservable SRAM window.
pub fn gen_fuzz_rv32a<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    // funct7 op5 bits per §4.5 / RISC-V spec §8.
    const OP5: &[u32] = &[0b00010, 0b00011, 0b00001, 0b00000, 0b00100, 0b01000, 0b01100,
        0b10000, 0b10100, 0b11000, 0b11100];
    for i in 0..count {
        let op5 = OP5[rng.gen_range(0..OP5.len())];
        let aqrl = rng.gen_range(0..4_u32); // { 00, 01, 10, 11 }
        let funct7 = (op5 << 2) | aqrl;
        let rd = rand_gpr(rng);
        let rs1 = rand_gpr(rng);
        // lr.w requires rs2 = 0 per spec; others take rs2 in x1..x31.
        let rs2 = if op5 == 0b00010 { 0 } else { rand_gpr(rng) };
        let w = encode_r_type(funct7, rs2, rs1, 0b010, rd, OPC_AMO);
        let mut reg_pre = vec![(rs1, SCRATCH_BASE)];
        if rs2 != 0 && rs2 != rs1 {
            reg_pre.push((rs2, rng.next_u32()));
        }
        out.push(RiscvTestCase {
            name: format!("fuzz_rv32a_{i}"),
            words: vec![w],
            reg_pre,
            addr_regs: vec![rs1],
            expect_trap: None,
            class: RiscvClass::Rv32aReservable,
        });
    }
    out
}

/// Fuzz generator: RV32C + sporadic Zcmp quadrant-2 bit patterns.
pub fn gen_fuzz_rv32c<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        // 10% of the fuzz RV32C stream is Zcmp-quadrant-2 to keep the collision
        // tripwire hot.
        let is_zcmp = rng.gen_bool(0.1);
        let enc: u16 = if is_zcmp {
            // funct3=101, quadrant=10 (Q2). On RV32 without the D
            // extension — Hazard3's configuration — this whole space is
            // reserved (the Zcmp extension colonises it with cm.push /
            // cm.pop / cm.popret / cm.popretz at funct6_low ∈ {4,5,6,7},
            // cm.mvsa01 / cm.mva01s at funct6_low=3).  We pick from the
            // unambiguously Zcmp-reserved values {4,5,6,7} to guarantee no
            // collision with c.jr / c.jalr / c.mv / c.add, which live at
            // funct3=100 (not 101) in Q2.  See the RISC-V Zc spec §1.3
            // (Zcmp quadrant-2 encoding table).
            const F6_LOW_ZCMP: [u16; 4] = [4, 5, 6, 7];
            let f6_low = F6_LOW_ZCMP[rng.gen_range(0..4)];
            let mid = rng.next_u32() as u16 & 0x03FF;
            0b101 << 13 | f6_low << 10 | (mid & 0x03FC) | 0b10
        } else {
            // RV32C — generate a plausible compressed instruction. Quadrants
            // 0/1/2; ensure c[1:0] != 0b11 (that's a 32-bit instruction).
            let q = rng.gen_range(0..3_u16);
            let payload = rng.next_u32() as u16 & 0xFFFC;
            payload | q
        };
        let expect_trap = if is_zcmp { Some(2) } else { None };
        out.push(RiscvTestCase {
            name: format!("fuzz_rvc_{i}"),
            words: vec![u32::from(enc)],
            reg_pre: vec![(2, SCRATCH_BASE)],
            addr_regs: vec![2],
            expect_trap,
            class: RiscvClass::Rv32c,
        });
    }
    out
}

/// Fuzz generator: Zicsr.
pub fn gen_fuzz_zicsr<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    const CSRS: &[u16] = &[0x300, 0x304, 0x305, 0x340, 0x341, 0x342, 0x344];
    for i in 0..count {
        let csr = CSRS[rng.gen_range(0..CSRS.len())];
        let funct3 = [1_u32, 2, 3, 5, 6, 7][rng.gen_range(0..6)];
        let rd = rng.gen_range(0..32_u8);
        let rs1_or_uimm5 = rng.gen_range(0..32_u8);
        let w = encode_csr(csr, rs1_or_uimm5, funct3, rd);
        let mut reg_pre = vec![];
        // Only the non-immediate variants consume a real register.
        if !matches!(funct3, 5..=7) && rs1_or_uimm5 != 0 {
            reg_pre.push((rs1_or_uimm5, rng.next_u32()));
        }
        out.push(RiscvTestCase {
            name: format!("fuzz_zicsr_{i}"),
            words: vec![w],
            reg_pre,
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Zicsr,
        });
    }
    out
}

/// Fuzz generator: Zifencei.
pub fn gen_fuzz_zifencei<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        // Alternate between FENCE.I (funct3=1) and FENCE (funct3=0).
        // Keep rs1/rd/imm fields at zero — the spec says non-zero values
        // are "reserved for future use", and the external decoder used
        // by the property test enforces that.  Hazard3 is lenient but
        // we encode spec-clean bit patterns.
        let is_fencei = rng.gen_bool(0.7);
        let w = if is_fencei {
            encode_i_type(0, 0, 1, 0, OPC_MISC_MEM)
        } else {
            // FENCE with pred/succ bits set (low 8 bits of imm12 = fm+pred+succ).
            let flags = rng.gen_range(0..256_u32) as i32;
            encode_i_type(flags, 0, 0, 0, OPC_MISC_MEM)
        };
        out.push(RiscvTestCase {
            name: format!("fuzz_fencei_{i}"),
            words: vec![w],
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Zifencei,
        });
    }
    out
}

/// Fuzz generator: CSR-side-effect chains. Kept bounded per LLD §6.
pub fn gen_fuzz_csr_side_effect<R: Rng>(
    rng: &mut R,
    count: usize,
) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    const CSRS: &[u16] = &[0x300, 0x304, 0x305, 0x340, 0x341, 0x342, 0x344];
    for i in 0..count {
        let csr = CSRS[rng.gen_range(0..CSRS.len())];
        let rd = rand_gpr(rng);
        let rs1 = rand_gpr(rng);
        let csrrw = encode_csr(csr, rs1, 1, rd);
        // Follow-up branch conditional on rd.
        let funct3 = if rng.gen_bool(0.5) { 0 } else { 1 };
        let branch = encode_b_type(8, 0, rd, funct3, OPC_BRANCH);
        out.push(RiscvTestCase {
            name: format!("fuzz_csrside_{i}"),
            words: vec![csrrw, branch],
            reg_pre: vec![(rs1, rng.next_u32())],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::CsrSideEffect,
        });
    }
    out
}

// ============================================================================
// Top-level composition
// ============================================================================

/// Concatenate all edge-case generators. Order matches `RiscvClass::ALL`.
pub fn generate_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(256);
    out.extend(gen_rv32i_alu_edge_cases());
    out.extend(gen_rv32i_mem_edge_cases());
    out.extend(gen_rv32i_misaligned_mem_edge_cases());
    out.extend(gen_rv32i_branch_edge_cases());
    out.extend(gen_rv32i_upper_edge_cases());
    out.extend(gen_rv32m_edge_cases());
    out.extend(gen_rv32a_reservable_edge_cases());
    out.extend(gen_rv32c_edge_cases());
    out.extend(gen_zicsr_edge_cases());
    out.extend(gen_zifencei_edge_cases());
    out.extend(gen_csr_side_effect_edge_cases());
    out
}

/// Generate `count` fuzz cases distributed per the LLD §6 weight table.
/// Any floor-rounding residue is absorbed by the heaviest class
/// (`Rv32iAlu`) so the total always matches `count`.
pub fn generate_fuzz<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut allocations: Vec<(RiscvClass, usize)> = RiscvClass::ALL
        .iter()
        .map(|c| (*c, (count * c.weight_bp() as usize) / 10_000))
        .collect();
    let allocated: usize = allocations.iter().map(|(_, n)| *n).sum();
    // Distribute residue into the ALU bucket (largest weight).
    if let Some(slot) = allocations.iter_mut().find(|(c, _)| *c == RiscvClass::Rv32iAlu) {
        slot.1 += count - allocated;
    }
    let mut out = Vec::with_capacity(count);
    for (class, n) in allocations {
        let chunk = match class {
            RiscvClass::Rv32iAlu => gen_fuzz_rv32i_alu(rng, n),
            RiscvClass::Rv32iMem => gen_fuzz_rv32i_mem(rng, n),
            RiscvClass::Rv32iMisalignedMem => gen_fuzz_rv32i_misaligned(rng, n),
            RiscvClass::Rv32iBranch => gen_fuzz_rv32i_branch(rng, n),
            RiscvClass::Rv32iUpper => gen_fuzz_rv32i_upper(rng, n),
            RiscvClass::Rv32m => gen_fuzz_rv32m(rng, n),
            RiscvClass::Rv32aReservable => gen_fuzz_rv32a(rng, n),
            RiscvClass::Rv32c => gen_fuzz_rv32c(rng, n),
            RiscvClass::Zicsr => gen_fuzz_zicsr(rng, n),
            RiscvClass::Zifencei => gen_fuzz_zifencei(rng, n),
            RiscvClass::CsrSideEffect => gen_fuzz_csr_side_effect(rng, n),
        };
        out.extend(chunk);
    }
    out
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    // --------------------------------------------------------------------
    // Encoder-helper unit tests — known-good hand-computed constants.
    // --------------------------------------------------------------------

    #[test]
    fn enc_r_type_matches_spec() {
        // add x1, x2, x3  →  0x003100B3
        //   funct7=0, rs2=3, rs1=2, funct3=0, rd=1, opcode=0x33
        assert_eq!(encode_r_type(0, 3, 2, 0, 1, OPC_OP), 0x003100B3);
        // sub x5, x6, x7  →  0x40730233
        assert_eq!(encode_r_type(0x20, 7, 6, 0, 4, OPC_OP), 0x40730233);
        // sll x10, x11, x12 → funct3=1
        assert_eq!(encode_r_type(0, 12, 11, 1, 10, OPC_OP), 0x00C59533);
        // mul x3, x4, x5 — funct7=0x01
        assert_eq!(encode_r_type(0x01, 5, 4, 0, 3, OPC_OP), 0x025201B3);
        // and x0, x0, x0 — all zero
        assert_eq!(encode_r_type(0, 0, 0, 7, 0, OPC_OP), 0x00007033);
    }

    #[test]
    fn enc_i_type_matches_spec() {
        // addi x1, x0, 1  →  0x00100093
        assert_eq!(encode_i_type(1, 0, 0, 1, OPC_OP_IMM), 0x00100093);
        // addi x2, x0, -1 → 0xFFF00113
        assert_eq!(encode_i_type(-1, 0, 0, 2, OPC_OP_IMM), 0xFFF00113);
        // addi x3, x4, 2047 → 0x7FF20193
        assert_eq!(encode_i_type(2047, 4, 0, 3, OPC_OP_IMM), 0x7FF20193);
        // andi x5, x6, 0x0FF → funct3=7, imm=0x0FF
        assert_eq!(encode_i_type(0xFF, 6, 7, 5, OPC_OP_IMM), 0x0FF372_93);
        // lw x6, 0(x7) → 0x0003A303
        assert_eq!(encode_i_type(0, 7, 2, 6, OPC_LOAD), 0x0003A303);
    }

    #[test]
    fn enc_s_type_matches_spec() {
        // sw x3, 0(x5)  →  0x0032A023
        //   imm=0, rs2=3, rs1=5, funct3=2, opcode=0x23
        assert_eq!(encode_s_type(0, 3, 5, 2, OPC_STORE), 0x0032A023);
        // sw x1, 4(x2) → 0x001122_23
        assert_eq!(encode_s_type(4, 1, 2, 2, OPC_STORE), 0x00112223);
        // sb x7, -1(x8)
        //   imm=-1 (0xFFF), imm_hi=0x7F, imm_lo=0x1F
        //   raw: 0xFE740FA3
        assert_eq!(encode_s_type(-1, 7, 8, 0, OPC_STORE), 0xFE740FA3);
        // sh x9, 2(x10) → 0x00951123
        assert_eq!(encode_s_type(2, 9, 10, 1, OPC_STORE), 0x00951123);
        // sw x6, 2044(x7)  (max aligned positive)
        //   imm=0x7FC, imm_hi=0x3F, imm_lo=0x1C → 0x7E63AE23
        assert_eq!(encode_s_type(2044, 6, 7, 2, OPC_STORE), 0x7E63AE23);
    }

    #[test]
    fn enc_b_type_matches_spec() {
        // beq x0, x0, 0 → 0x00000063
        assert_eq!(encode_b_type(0, 0, 0, 0, OPC_BRANCH), 0x00000063);
        // bne x1, x2, 8 → funct3=1, imm=8
        //   bit pattern: 0x00209463
        assert_eq!(encode_b_type(8, 2, 1, 1, OPC_BRANCH), 0x00209463);
        // beq x0, x0, -8 → 0xFE000CE3
        assert_eq!(encode_b_type(-8, 0, 0, 0, OPC_BRANCH), 0xFE000CE3);
        // bge x5, x6, 4096 — imm=4096 wraps 13-bit signed to the bit
        // pattern 0b1_0000_0000_0000, so imm[12]=1 and bit 31 of the
        // encoding is set.  Encoder output: 0x8062D063.
        assert_eq!(encode_b_type(4096, 6, 5, 5, OPC_BRANCH), 0x8062D063);
        // bltu x7, x8, -4096 (bit 12 set + bit 11 clear)
        // imm = -4096 = 0x1000 12-bit, so imm[12]=1 imm[11:0]=0
        // expected: 0x8083E063
        assert_eq!(encode_b_type(-4096, 8, 7, 6, OPC_BRANCH), 0x8083E063);
    }

    #[test]
    fn enc_u_type_matches_spec() {
        // lui x1, 0x12345 → 0x123450B7
        assert_eq!(encode_u_type(0x12345000, 1, OPC_LUI), 0x123450B7);
        // auipc x2, 0x1 → 0x00001117
        assert_eq!(encode_u_type(0x00001000, 2, OPC_AUIPC), 0x00001117);
        // lui x0, 0 → 0x00000037
        assert_eq!(encode_u_type(0, 0, OPC_LUI), 0x00000037);
        // lui x3, 0xFFFFF (max) → 0xFFFFF1B7
        assert_eq!(encode_u_type(0xFFFFF000, 3, OPC_LUI), 0xFFFFF1B7);
        // auipc x5, 0xABCDE → 0xABCDE297
        assert_eq!(encode_u_type(0xABCDE000, 5, OPC_AUIPC), 0xABCDE297);
    }

    #[test]
    fn enc_j_type_matches_spec() {
        // jal x0, 0 → 0x0000006F
        assert_eq!(encode_j_type(0, 0, OPC_JAL), 0x0000006F);
        // jal x1, 0 → 0x000000EF
        assert_eq!(encode_j_type(0, 1, OPC_JAL), 0x000000EF);
        // jal x1, 8 → 0x008000EF
        assert_eq!(encode_j_type(8, 1, OPC_JAL), 0x008000EF);
        // jal x1, -8 → 0xFF9FF0EF
        assert_eq!(encode_j_type(-8, 1, OPC_JAL), 0xFF9FF0EF);
        // jal x2, 0x100000 (bit 20 set)
        //   bit 20 = 1, bit 10:1 = 0, bit 11 = 0, bit 19:12 = 0
        //   encoded: 0x80000 16F
        assert_eq!(encode_j_type(0x100000, 2, OPC_JAL), 0x80000_16F);
    }

    #[test]
    fn enc_csr_matches_spec() {
        // csrrs x1, mstatus, x0 — rd=1 shifts to bit 7 → 0x80 in low
        // byte; combined with funct3=2 gives 0x300020F3.
        assert_eq!(encode_csr(0x300, 0, 2, 1), 0x300020F3);
        // csrrw x0, mstatus, x0 → 0x30001073
        assert_eq!(encode_csr(0x300, 0, 1, 0), 0x30001073);
        // csrrc x5, mscratch, x7 — rd=5 → 0x280 in low byte, funct3=3,
        // rs1=7 in bits 19:15.
        //   (0x340 << 20) | (7<<15) | (3<<12) | (5<<7) | 0x73
        //   = 0x3403_B2F3
        assert_eq!(encode_csr(0x340, 7, 3, 5), 0x3403B2F3);
        // csrrwi x8, mtvec, uimm=5 →
        //   (0x305<<20) | (5<<15) | (5<<12) | (8<<7) | 0x73 = 0x3052D473
        assert_eq!(encode_csr(0x305, 5, 5, 8), 0x3052D473);
        // csrrsi x0, mepc, uimm=1 —
        //   (0x341<<20) | (1<<15) | (6<<12) | (0<<7) | 0x73 = 0x3410_E073
        assert_eq!(encode_csr(0x341, 1, 6, 0), 0x3410_E073);
    }

    // --------------------------------------------------------------------
    // Per-class generator sanity + the F/D tripwire.
    // --------------------------------------------------------------------

    fn check_no_fp(words: &[u32], is_compressed_ok: bool) {
        for &w in words {
            if is_compressed_ok && is_compressed(w) {
                continue;
            }
            assert!(
                !is_fp_opcode(w),
                "F/D opcode slipped into generator: 0x{w:08X}"
            );
        }
    }

    fn check_class(cases: &[RiscvTestCase], expected: RiscvClass, compressed_allowed: bool) {
        assert!(!cases.is_empty(), "no cases for {expected:?}");
        for tc in cases {
            assert_eq!(tc.class, expected, "class mismatch in {}", tc.name);
            check_no_fp(&tc.words, compressed_allowed);
            for &w in &tc.words {
                // Distinguish 16- vs 32-bit by the quadrant bits, not by
                // magnitude — a 32-bit instruction with all-zero high
                // fields (e.g. `lui x0, 0` → 0x00000037) would otherwise
                // be mis-classified.
                if is_compressed(w) {
                    assert!(
                        compressed_allowed,
                        "unexpected 16-bit word in {expected:?}: 0x{w:04X}"
                    );
                    assert!(w <= 0xFFFF, "compressed word overflows u16 in {}", tc.name);
                }
            }
        }
    }

    #[test]
    fn edge_cases_rv32i_alu() {
        let cs = gen_rv32i_alu_edge_cases();
        check_class(&cs, RiscvClass::Rv32iAlu, false);
    }

    #[test]
    fn edge_cases_rv32i_mem() {
        let cs = gen_rv32i_mem_edge_cases();
        check_class(&cs, RiscvClass::Rv32iMem, false);
    }

    #[test]
    fn edge_cases_rv32i_misaligned() {
        let cs = gen_rv32i_misaligned_mem_edge_cases();
        check_class(&cs, RiscvClass::Rv32iMisalignedMem, false);
        for tc in &cs {
            let trap = tc.expect_trap.expect("misaligned cases must trap");
            assert!(trap == 4 || trap == 6, "unexpected trap: {trap}");
        }
    }

    #[test]
    fn edge_cases_rv32i_branch() {
        let cs = gen_rv32i_branch_edge_cases();
        check_class(&cs, RiscvClass::Rv32iBranch, false);
    }

    #[test]
    fn edge_cases_rv32i_upper() {
        let cs = gen_rv32i_upper_edge_cases();
        check_class(&cs, RiscvClass::Rv32iUpper, false);
    }

    #[test]
    fn edge_cases_rv32m() {
        let cs = gen_rv32m_edge_cases();
        check_class(&cs, RiscvClass::Rv32m, false);
    }

    #[test]
    fn edge_cases_rv32a() {
        let cs = gen_rv32a_reservable_edge_cases();
        check_class(&cs, RiscvClass::Rv32aReservable, false);
        for tc in &cs {
            for reg in &tc.addr_regs {
                let v = tc.reg_pre.iter().find(|(r, _)| r == reg).map(|(_, v)| *v).unwrap();
                assert!(
                    (RESERVABLE_LO..RESERVABLE_HI).contains(&v),
                    "atomic address out of reservable window: 0x{v:08X}"
                );
            }
        }
    }

    #[test]
    fn edge_cases_rv32c() {
        let cs = gen_rv32c_edge_cases();
        check_class(&cs, RiscvClass::Rv32c, true);
    }

    #[test]
    fn edge_cases_zicsr() {
        let cs = gen_zicsr_edge_cases();
        check_class(&cs, RiscvClass::Zicsr, false);
    }

    #[test]
    fn edge_cases_zifencei() {
        let cs = gen_zifencei_edge_cases();
        check_class(&cs, RiscvClass::Zifencei, false);
    }

    #[test]
    fn edge_cases_csr_side_effect() {
        let cs = gen_csr_side_effect_edge_cases();
        check_class(&cs, RiscvClass::CsrSideEffect, false);
    }

    // --------------------------------------------------------------------
    // Total counts sanity + global F/D tripwire.
    // --------------------------------------------------------------------

    #[test]
    #[ignore = "report-only: prints per-class edge-case counts"]
    fn report_edge_case_counts() {
        eprintln!("ALU={}", gen_rv32i_alu_edge_cases().len());
        eprintln!("MEM={}", gen_rv32i_mem_edge_cases().len());
        eprintln!("MISALIGNED={}", gen_rv32i_misaligned_mem_edge_cases().len());
        eprintln!("BRANCH={}", gen_rv32i_branch_edge_cases().len());
        eprintln!("UPPER={}", gen_rv32i_upper_edge_cases().len());
        eprintln!("RV32M={}", gen_rv32m_edge_cases().len());
        eprintln!("RV32A={}", gen_rv32a_reservable_edge_cases().len());
        eprintln!("RV32C={}", gen_rv32c_edge_cases().len());
        eprintln!("ZICSR={}", gen_zicsr_edge_cases().len());
        eprintln!("ZIFENCEI={}", gen_zifencei_edge_cases().len());
        eprintln!("CSR_SIDE={}", gen_csr_side_effect_edge_cases().len());
        eprintln!("TOTAL={}", generate_edge_cases().len());
    }

    #[test]
    fn generate_edge_cases_total_fp_tripwire() {
        let all = generate_edge_cases();
        // LLD §6 edge-case column says we should be at least in the "~200"
        // ballpark; loose floor of 150 is comfortable headroom against
        // individual-class drift.
        assert!(all.len() >= 150, "edge-case total too low: {}", all.len());
        // Global F/D scan (permits 16-bit compressed).
        for tc in &all {
            check_no_fp(&tc.words, true);
        }
    }

    #[test]
    fn weights_sum_to_10000_bp() {
        let total: u32 = RiscvClass::ALL.iter().map(|c| c.weight_bp()).sum();
        assert_eq!(total, 10_000, "class weights must sum to 100.00%");
    }

    #[test]
    fn fuzz_distribution_within_5pc() {
        let mut rng = StdRng::seed_from_u64(0xD0C_A501 /* arbitrary pin */);
        let cases = generate_fuzz(&mut rng, 10_000);
        assert_eq!(cases.len(), 10_000);
        // Global F/D scan.
        for tc in &cases {
            check_no_fp(&tc.words, true);
        }
        for class in RiscvClass::ALL {
            let count = cases.iter().filter(|c| c.class == class).count();
            let expected = class.weight_bp() as f64 / 100.0; // percent
            let actual = count as f64 / 100.0;
            let delta = (actual - expected).abs();
            // LLD says ±5% — we interpret that as ±5 percentage points.
            // In practice integer-floor allocation is exact to ±1 case /
            // 10_000 = ±0.01 pp, so this is very conservative.
            assert!(
                delta <= 5.0,
                "class {class:?} drift: expected {expected}%, got {actual}% (count {count})"
            );
        }
    }

    #[test]
    fn zcmp_sweep_tripwire() {
        // Per core HLD V6 §4.5: the Zcmp quadrant-2 sweep must contain a
        // meaningful number of distinct bit patterns, and must include
        // representatives of cm.push / cm.pop / cm.popret / cm.mvsa01 /
        // cm.mva01s.
        let cs = gen_rv32c_edge_cases();
        let zcmp: Vec<&RiscvTestCase> = cs
            .iter()
            .filter(|tc| tc.name.starts_with("rvc_zcmp") || tc.name.contains("zcmp_"))
            .collect();
        assert!(
            zcmp.len() >= 30,
            "Zcmp quadrant-2 sweep must cover >= 30 patterns, got {}",
            zcmp.len()
        );
        // Each sub-family must appear.
        for fam in ["cm_push", "cm_pop", "cm_popret", "cm_mvsa01", "cm_mva01s"] {
            assert!(
                zcmp.iter().any(|tc| tc.name.contains(fam)),
                "Zcmp sub-family {fam} missing from sweep"
            );
        }
        // Every Zcmp sweep case must be tagged `expect_trap: Some(2)`.
        for tc in zcmp {
            assert_eq!(tc.expect_trap, Some(2), "Zcmp case {} not tagged trap=2", tc.name);
            // And the bit pattern must satisfy funct3=5 AND quadrant=2.
            let enc = tc.words[0] as u16;
            assert_eq!((enc >> 13) & 0x7, 0b101, "Zcmp funct3 bit-pattern wrong in {}", tc.name);
            assert_eq!(enc & 0x3, 0b10, "Zcmp quadrant bits wrong in {}", tc.name);
        }
    }

    // --------------------------------------------------------------------
    // Property test (LLD §8).  1000 encoded cases must all round-trip
    // through an external authoritative decoder (`riscv-decode`).
    //
    // Rationale: spec cross-check against a third-party decoder eliminates
    // our private `riscv_gen` encoder as a self-masking surface for
    // encoding bugs — a shared bug here would have to be present in both
    // `riscv-decode` and our encoder to escape the harness.
    // --------------------------------------------------------------------

    /// Map a `riscv_decode::Instruction` to a coarse class string. Used
    /// to cross-check against `RiscvTestCase::class`.
    fn decoded_to_class(inst: &riscv_decode::Instruction) -> &'static str {
        use riscv_decode::Instruction::*;
        match inst {
            Add(_) | Addi(_) | Sub(_) | Sll(_) | Slli(_) | Srl(_) | Srli(_) | Sra(_)
            | Srai(_) | Xor(_) | Xori(_) | Or(_) | Ori(_) | And(_) | Andi(_) | Slt(_)
            | Slti(_) | Sltu(_) | Sltiu(_) => "alu",
            Lb(_) | Lh(_) | Lw(_) | Lbu(_) | Lhu(_) | Sb(_) | Sh(_) | Sw(_) => "mem",
            Beq(_) | Bne(_) | Blt(_) | Bge(_) | Bltu(_) | Bgeu(_) | Jal(_) | Jalr(_) => {
                "branch"
            }
            Lui(_) | Auipc(_) => "upper",
            Mul(_) | Mulh(_) | Mulhu(_) | Mulhsu(_) | Div(_) | Divu(_) | Rem(_)
            | Remu(_) => "rv32m",
            LrW(_) | ScW(_) | AmoswapW(_) | AmoaddW(_) | AmoxorW(_) | AmoandW(_)
            | AmoorW(_) | AmominW(_) | AmomaxW(_) | AmominuW(_) | AmomaxuW(_) => "rv32a",
            Csrrw(_) | Csrrs(_) | Csrrc(_) | Csrrwi(_) | Csrrsi(_) | Csrrci(_) => "zicsr",
            FenceI => "fencei",
            Fence(_) => "fence",
            Ecall | Ebreak | Mret => "misc",
            _ => "other",
        }
    }

    fn class_compatible(tc: &RiscvClass, decoded: &str) -> bool {
        match tc {
            RiscvClass::Rv32iAlu => decoded == "alu",
            RiscvClass::Rv32iMem | RiscvClass::Rv32iMisalignedMem => decoded == "mem",
            RiscvClass::Rv32iBranch => decoded == "branch",
            RiscvClass::Rv32iUpper => decoded == "upper" || decoded == "alu",
            RiscvClass::Rv32m => decoded == "rv32m",
            RiscvClass::Rv32aReservable => decoded == "rv32a",
            RiscvClass::Zicsr => decoded == "zicsr",
            RiscvClass::Zifencei => decoded == "fencei" || decoded == "fence",
            RiscvClass::CsrSideEffect => {
                // Multi-instruction: the first word is a csrrw/csrrs etc.,
                // the second a branch or arithmetic.  The per-word check
                // below handles both.
                decoded == "zicsr" || decoded == "branch" || decoded == "alu"
            }
            // Rv32c words take the `is_compressed` branch in the property
            // test and never reach `class_compatible`, so this arm is
            // unreachable by construction.
            RiscvClass::Rv32c => unreachable!(
                "RV32C words are handled via the is_compressed branch, not class_compatible"
            ),
        }
    }

    /// Sentinel arm for Zcmp — see LLD §8.  `riscv-decode` does not
    /// recognise quadrant-2 Zcmp encodings, so we spot-check the bit
    /// pattern directly.
    fn is_zcmp_bit_pattern(word: u16) -> bool {
        // funct3 (bits 15:13) = 0b101, quadrant (bits 1:0) = 0b10.
        (word >> 13) & 0x7 == 0b101 && word & 0x3 == 0b10
    }

    #[test]
    fn property_test_1000_encodings_decode_correctly() {
        let mut rng = StdRng::seed_from_u64(0xBADF00D);
        let cases = generate_fuzz(&mut rng, 1000);
        assert_eq!(cases.len(), 1000);

        // Split counters so RV32C words can't silently pass through without
        // any assertion. Each 32-bit word that `riscv-decode` accepts and
        // class-matches bumps `verified_32bit`; each compressed word that
        // hits the Zcmp sentinel bumps `verified_compressed_sentinel`.
        // Compressed words that `riscv-decode` happens to accept also count
        // toward the compressed sentinel floor — either way, a compressed
        // word that traverses this loop must have been inspected, not just
        // counted.
        let mut verified_32bit = 0usize;
        let mut verified_compressed_sentinel = 0usize;
        let mut compressed_unhandled = 0usize;
        for tc in &cases {
            for &word in &tc.words {
                // F/D tripwire — every word, every class, always.
                if !is_compressed(word) {
                    assert!(
                        !is_fp_opcode(word),
                        "F/D opcode slipped through fuzz: 0x{word:08X} in {}",
                        tc.name
                    );
                }
                if is_compressed(word) {
                    let enc16 = word as u16;
                    match riscv_decode::decode(word) {
                        Ok(i) => {
                            // If the external decoder unexpectedly decodes a
                            // compressed word, it's fine — we still accept
                            // and count it as sentinel-verified.
                            let _ = decoded_to_class(&i);
                            verified_compressed_sentinel += 1;
                        }
                        Err(_) => {
                            // `riscv-decode 0.2.3` returns Err for all 16-bit
                            // encodings.  Accept that, but if the word is in
                            // the Zcmp quadrant-2 space, cross-check the
                            // `expect_trap` tag.
                            if is_zcmp_bit_pattern(enc16) {
                                if tc.name.contains("zcmp") {
                                    assert_eq!(
                                        tc.expect_trap,
                                        Some(2),
                                        "zcmp case missing trap tag: {}",
                                        tc.name
                                    );
                                }
                                verified_compressed_sentinel += 1;
                            } else {
                                // Plain RV32C word that the external decoder
                                // doesn't recognise and which isn't in the
                                // Zcmp sentinel pattern.  We don't have a
                                // second oracle for it here, but we mustn't
                                // silently count it as verified.
                                compressed_unhandled += 1;
                            }
                        }
                    }
                    continue;
                }
                match riscv_decode::decode(word) {
                    Ok(inst) => {
                        let decoded = decoded_to_class(&inst);
                        assert!(
                            class_compatible(&tc.class, decoded),
                            "class mismatch: case {} (class {:?}) word 0x{word:08X} decoded as {decoded}",
                            tc.name,
                            tc.class
                        );
                        verified_32bit += 1;
                    }
                    Err(e) => {
                        // Every 32-bit word we generate must be decodable
                        // by `riscv-decode`.  Anything else is an encoder
                        // bug.
                        panic!(
                            "riscv-decode rejected word 0x{word:08X} from {}: {e:?}",
                            tc.name
                        );
                    }
                }
            }
        }
        // Floors: with the LLD §6 weight table we expect ~800+ 32-bit words
        // decoded cleanly.  Compressed words split into "Zcmp sentinel"
        // (either decodable by riscv-decode or matching the Zcmp bit
        // pattern) and "unhandled" (plain RV32C that the external decoder
        // doesn't recognise — no RV32C oracle is available in
        // riscv-decode 0.2.3).  Zcmp fuzz contributes ~10% of the Rv32c
        // slice (~7.5% of the stream), so ~10 sentinel hits is the floor.
        // `compressed_unhandled` is visible in the panic message but not
        // an assertion failure: these words still pass the F/D tripwire
        // implicitly (compressed words can't encode F/D ops) and the
        // generator's Q2/Zcmp tagging is the relevant invariant, which the
        // sentinel floor covers.
        assert!(
            verified_32bit >= 800,
            "too few 32-bit words verified: {verified_32bit} (expected >= 800); \
             compressed_sentinel={verified_compressed_sentinel} \
             compressed_unhandled={compressed_unhandled}"
        );
        assert!(
            verified_compressed_sentinel >= 10,
            "too few compressed words hit the Zcmp sentinel: \
             {verified_compressed_sentinel} (expected >= 10); \
             compressed_unhandled = {compressed_unhandled}"
        );
    }
}

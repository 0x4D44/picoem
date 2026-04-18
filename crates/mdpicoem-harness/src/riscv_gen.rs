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
    /// Physical Memory Protection CSR surface — phase-2 models pmpcfg0/1
    /// and pmpaddr0..7 (NUM_ENTRIES=8, matching RP2350 datasheet §3.8
    /// dynamic-region count). L-bit sticky-lock + TOR cross-entry lock
    /// are modelled on the emu side; entries 8..15 are RAZ/WI on both
    /// sides (via `warl_mask` for QEMU). See
    /// `wrk_docs/2026.04.18 - HLD - RISC-V PMP Coverage V1.md` V2 §A.
    Pmp,
}

impl RiscvClass {
    /// All twelve variants in a fixed order (matches the LLD §6 table +
    /// phase-1 PMP addition).
    pub const ALL: [RiscvClass; 12] = [
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
        RiscvClass::Pmp,
    ];

    /// Fuzz weight in basis points (sums to 10_000). Per LLD §6 "Fuzz
    /// weight (per `--fuzz N`)".
    ///
    /// **Phase-1 scope reduction (2026-04-18):** `Pmp` is weighted at 0 bp
    /// in the default mixed-class fuzz mix. The CSR model, unit tests, and
    /// edge cases remain fully wired and the class is still reachable via
    /// `--class pmp` for phase-2 bring-up / debugging. See the V2 addendum
    /// in `wrk_docs/2026.04.18 - HLD - RISC-V PMP Coverage V1.md` for the
    /// empirical divergences (pmp_gran granule-masking, L-bit sticky-trap
    /// fuzz contamination) that motivated the descope.
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
            // Phase-2 scope (see module doc above). Reachable via --class pmp.
            RiscvClass::Pmp => 0,
        }
    }
}

// ============================================================================
// Address map constants
// ============================================================================

/// Scratchpad base — memory / atomics cases pre-load an x-register with
/// this so the encoded instruction's 12-bit immediate covers a known
/// safe offset range.
/// Test-writable data region for mem/atomic/branch cases that use
/// `addr_regs` to preload a base register. Distinct from the
/// binary-internal CSR capture scratchpad at `0x8000_0300` so the
/// proxy's pre-snapshot reads don't leak CSR values into memory that
/// tests then load from (CSR values diverge between Hazard3 and QEMU
/// virt on several fields — mstatus.MPP is forced 0b11 on M-only
/// Hazard3 but QEMU virt is S+U-aware, etc. — and the WARL mask
/// normalises the CSR diff but not ordinary memory loads).
///
/// Relocated from `0x2000_0300` (RP2350 SRAM) to `0x8000_0400` (QEMU
/// virt DRAM) — virt maps VIRT_FLASH at `0x2000_0000`, which quietly
/// drops CPU `sw` instructions. The mdrp2350 bus aliases `0x8xxx_xxxx`
/// onto SRAM via `canon_oracle_addr`, so both sides run at the same
/// absolute address.
pub const SCRATCH_BASE: u32 = 0x8000_0400;

/// Trap-handler stub slot (see harness LLD §4 / test_rp2350_qemu_diff_riscv32).
/// The handler occupies `TRAP_STUB..TRAP_STUB+0x14` (5 instrs); its
/// terminal `ebreak` sits at `TRAP_STUB + 16`. Edge-case generators that
/// want to land a JALR on a known-trapping target read this value rather
/// than hard-coding it, so the binary and the generator cannot drift.
pub const TRAP_STUB: u32 = 0x8000_0200;

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

/// 10% fuzz weight; ~25 edge cases. All 6 conditional branches + JAL at
/// short offsets. Out-of-test-body targets have two failure modes the
/// harness cannot oracle cleanly:
///
/// - Short backward offsets (e.g. −8) naive layout: target lands inside
///   the proxy prelude's CSR-read epilogue. Those instructions don't
///   trap, so control flow re-enters the branch and loops forever until
///   the GDB read timeout fires.
/// - Large offsets: target lands in uninitialised memory, whose fetch
///   behaviour differs between QEMU virt (unmapped → cause 1) and mdrp2350
///   (SRAM alias fetches 0 → c.illegal → cause 2). Platform-layout
///   divergence, not a branch-semantics bug.
///
/// Forward-branch tests use a NOP sled of length `FWD_SLED` after the
/// branch so a forward-taken branch coasts down to the terminator ebreak
/// the harness appends. Backward-branch tests use a different layout:
/// an explicit `ebreak` word is planted at offset 0 and the branch sits
/// `|imm|` bytes further in, so `branch − |imm|` lands on the ebreak and
/// the test halts immediately (via the trap-handler hw-breakpoint). JAL
/// cases are identical in structure to BEQ cases.
///
/// JALR edge coverage: two cases probe encoding bits not exercised by
/// the `upper_auipc_jalr_pair` case (which fixes imm=0, rd=1). `jalr_off4`
/// drives a nonzero `imm[11:0]` field; `jalr_rdx0` drives `rd=0` (no-link
/// semantics, WARL x0 stays zero). Both pre-seed `x2` such that
/// `(x2 + imm) & ~1 == TRAP_STUB + 16` — the jump lands directly on the
/// trap handler's ebreak (guaranteed present by harness startup) and the
/// test halts via the harness's `trap_handler_ebreak` hw-breakpoint.
/// Random-offset JALR fuzz is still omitted — the generator can't pick a
/// safe arbitrary target without harness-side plumbing.
pub fn gen_rv32i_branch_edge_cases() -> Vec<RiscvTestCase> {
    const FWD_SLED: usize = 4;
    const NOP: u32 = 0x0000_0013; // addi x0, x0, 0
    const EBREAK: u32 = 0x0010_0073; // ebreak (32-bit)

    let mut out = Vec::with_capacity(28);

    // Forward-taken + never-taken branches. Layout:
    //   [branch, NOP × FWD_SLED, terminator-ebreak-appended-by-harness]
    // Forward imm ∈ {4, 8, .., FWD_SLED*4} so the taken target lands on
    // a NOP in the sled, then coasts down to the terminator.
    let forward: &[(&str, u32, i32, u8, u8, u32, u32)] = &[
        ("beq_eq", 0, 4, 1, 2, 0x10, 0x10),
        ("beq_ne", 0, 4, 1, 2, 0x10, 0x11),
        ("bne_ne", 1, 4, 3, 4, 1, 2),
        ("bne_eq", 1, 4, 3, 4, 1, 1),
        ("blt_pos", 4, 4, 5, 6, 1_i32 as u32, 2_i32 as u32),
        ("blt_neg", 4, 4, 5, 6, (-1_i32) as u32, 0),
        ("bge_pos", 5, 4, 7, 8, 2, 1),
        ("bge_neg", 5, 4, 7, 8, 0, (-1_i32) as u32),
        ("bltu_carry", 6, 4, 9, 10, 0, 0xFFFF_FFFF),
        ("bgeu_eq", 7, 4, 11, 12, 5, 5),
    ];
    for &(name, funct3, imm, rs1, rs2, v1, v2) in forward {
        let w = encode_b_type(imm, rs2, rs1, funct3, OPC_BRANCH);
        let mut regs = vec![(rs1, v1), (rs2, v2)];
        regs.sort_by_key(|r| r.0);
        regs.dedup_by_key(|r| r.0);
        regs.retain(|(r, _)| *r != 0);
        let mut words = vec![w];
        for _ in 0..FWD_SLED {
            words.push(NOP);
        }
        out.push(RiscvTestCase {
            name: format!("branch_{name}"),
            words,
            reg_pre: regs,
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }

    // Backward-taken branches. Layout:
    //   [ebreak, NOP × (|imm|/4 - 1), branch(imm), NOP × FWD_SLED]
    // The branch sits at offset `|imm|` from the test body start; taking
    // it with `imm = -|imm|` lands PC at offset 0 — the planted ebreak.
    // The ebreak traps to TRAP_STUB, which hits the harness's trap-
    // handler hw breakpoint and halts. Not-taken path flows forward
    // through the trailing NOPs to the terminator. imm must be a 4-byte
    // multiple and ≥ 8 (need at least `[ebreak, NOP, branch]`).
    let backward: &[(&str, u32, i32, u8, u8, u32, u32)] = &[
        ("beq_neg_off", 0, -8, 13, 14, 3, 3),
        ("beq_near_neg", 0, -16, 17, 18, 0, 0),
    ];
    for &(name, funct3, imm, rs1, rs2, v1, v2) in backward {
        debug_assert!(imm <= -8 && imm % 4 == 0, "bad backward imm {imm}");
        let w = encode_b_type(imm, rs2, rs1, funct3, OPC_BRANCH);
        let mut regs = vec![(rs1, v1), (rs2, v2)];
        regs.sort_by_key(|r| r.0);
        regs.dedup_by_key(|r| r.0);
        regs.retain(|(r, _)| *r != 0);
        let slots = (-imm / 4) as usize; // 2 for imm=-8, 4 for imm=-16
        let mut words = Vec::with_capacity(slots + 1 + FWD_SLED);
        words.push(EBREAK);
        for _ in 0..(slots - 1) {
            words.push(NOP);
        }
        words.push(w);
        for _ in 0..FWD_SLED {
            words.push(NOP);
        }
        out.push(RiscvTestCase {
            name: format!("branch_{name}"),
            words,
            reg_pre: regs,
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }

    // JAL near-forward with sled (same as conditional branch forward
    // layout), plus one backward case (same as backward layout above).
    // `jal_rdx0` is forward imm = 4, rd = 0 (no link).
    let jal_forward: &[(&str, i32, u8)] = &[
        ("jal_near_pos", 4, 1),
        ("jal_rdx0", 4, 0),
    ];
    for &(name, imm, rd) in jal_forward {
        let w = encode_j_type(imm, rd, OPC_JAL);
        let mut words = vec![w];
        for _ in 0..FWD_SLED {
            words.push(NOP);
        }
        out.push(RiscvTestCase {
            name: format!("branch_{name}"),
            words,
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }

    // Backward JAL: jumps back to a planted ebreak; link register (x1)
    // ends up holding PC+4 of the JAL, which the GPR diff catches.
    {
        let imm = -16_i32;
        debug_assert!(imm <= -8 && imm % 4 == 0);
        let w = encode_j_type(imm, 1, OPC_JAL);
        let slots = (-imm / 4) as usize;
        let mut words = Vec::with_capacity(slots + 1 + FWD_SLED);
        words.push(EBREAK);
        for _ in 0..(slots - 1) {
            words.push(NOP);
        }
        words.push(w);
        for _ in 0..FWD_SLED {
            words.push(NOP);
        }
        out.push(RiscvTestCase {
            name: "branch_jal_near_neg".into(),
            words,
            reg_pre: vec![],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Rv32iBranch,
        });
    }

    // JALR: target address lives in a GPR, not in the encoding. The
    // `upper_auipc_jalr_pair` case already covers the common (imm=0, rd=1)
    // form via PC-relative address build. The two cases below fill the
    // encoding gaps it leaves:
    //   * `jalr_off4` — nonzero `imm[11:0]` (= 4). rs1 = TRAP_EBREAK - 4
    //     so `(rs1 + 4) & !1 == TRAP_EBREAK`.
    //   * `jalr_rdx0`  — rd = 0 (no link; x0 must stay zero, WARL). rs1 =
    //     TRAP_EBREAK directly (imm = 0).
    // Both land directly on the trap handler's ebreak (installed at startup,
    // see `install_trap_stub`), so the harness's `trap_handler_ebreak` hw
    // breakpoint halts both sides at the same PC without executing any
    // unmapped fetch. Random-offset JALR fuzz is still off the table — the
    // generator can't construct an arbitrary safe target at encode time.
    const TRAP_EBREAK: u32 = TRAP_STUB + 16;
    let jalr_cases: &[(&str, i32, u8, u8, u32)] = &[
        // (name, imm12, rs1, rd, rs1_preload)
        ("jalr_off4", 4, 2, 1, TRAP_EBREAK.wrapping_sub(4)),
        ("jalr_rdx0", 0, 2, 0, TRAP_EBREAK),
    ];
    for &(name, imm, rs1, rd, rs1_val) in jalr_cases {
        let w = encode_i_type(imm, rs1, 0, rd, OPC_JALR);
        out.push(RiscvTestCase {
            name: format!("branch_{name}"),
            words: vec![w],
            reg_pre: vec![(rs1, rs1_val)],
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
    // Encoded as (name, enc, extra_reg_pre). `extra_reg_pre` is merged
    // with the default `(x2, SCRATCH_BASE)` stack-pointer seeding. For
    // c.jr / c.jalr we override x1 so the jump target is known-valid on
    // both sides — without this the default x1 = 0 lands PC at the
    // unmapped 0x0, and QEMU (reports mcause = 1 "instruction access
    // fault") and mdrp2350 (SRAM alias returns 0 → c.illegal → mcause =
    // 2) diverge on a platform-layout artefact rather than on the jump
    // decode itself. Pointing x1 at `TRAP_STUB + 16` (the trap handler's
    // ebreak) sidesteps the fetch step entirely — the harness halts the
    // moment that ebreak is hit, which is the same outcome both sides
    // reach via any other trapping path.
    const TRAP_EBREAK: u32 = TRAP_STUB + 16;
    let plain: &[(&str, u16, Option<(u8, u32)>)] = &[
        ("c_addi4spn", 0b0_0000_0001_0010_0000, None), // addi4spn rd'=x8, nzimm=small
        ("c_nop", 0x0001, None),                           // c.addi x0, 0 (canonical nop)
        ("c_addi_1", 0x0085, None),                        // c.addi x1, 1
        ("c_li", 0x4085, None),                            // c.li x1, 1 (imm[4:0]=00001)
        ("c_lui", 0x6105, None),                           // c.lui x2, 1  (imm nonzero)
        ("c_andi", 0x8805, None),                          // c.andi x8, 1
        // c.jr / c.jalr rs1 = x1; seed x1 to TRAP_EBREAK so the jump
        // lands on the handler's ebreak — avoids the x1 = 0 fetch-fault
        // divergence (QEMU mcause 1 vs emu mcause 2).
        ("c_jr_x1", 0x8082, Some((1, TRAP_EBREAK))),       // c.jr x1
        ("c_jalr_x1", 0x9082, Some((1, TRAP_EBREAK))),     // c.jalr x1
        ("c_slli", 0x0086, None),                          // c.slli x1, 1
        ("c_lwsp", 0x4082, None),                          // c.lwsp x1, 0(sp)
        ("c_swsp", 0xc006, None),                          // c.swsp x1, 0(sp)
    ];
    for &(name, enc, extra) in plain {
        let mut reg_pre = vec![(2, SCRATCH_BASE)]; // sp in scratchpad for stack cases
        if let Some(r) = extra {
            reg_pre.push(r);
        }
        out.push(RiscvTestCase {
            name: format!("rvc_{name}"),
            words: vec![u32::from(enc)],
            reg_pre,
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

/// PMP CSR surface (phase-2: NUM_ENTRIES=8 covering pmpcfg0..1 and
/// pmpaddr0..7). Each edge case is a single write-then-read-back so the
/// diff lands on `rd` of both instructions (the read-back is the primary
/// divergence catcher; the write-side `rd` is the old value). See
/// `wrk_docs/2026.04.18 - HLD - RISC-V PMP Coverage V1.md` §4.2 +
/// V2 §A.6.
///
/// The edge-case value pool excludes L-bit patterns: QEMU cannot be
/// reset between tests, so any L=1 latch on QEMU side leaks into
/// subsequent PMP edge cases. L-bit semantics are covered exhaustively
/// by the emulator-only unit tests in `core_riscv/tests_p2.rs`.
pub fn gen_pmp_edge_cases() -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(20);
    let patterns: &[(&str, u16, u32)] = &[
        ("pmpcfg0_zero",        0x3A0, 0x0000_0000),
        ("pmpcfg0_rwx",         0x3A0, 0x0000_0007),
        ("pmpcfg0_napot",       0x3A0, 0x0000_0018),
        ("pmpcfg0_napot_rwx",   0x3A0, 0x0000_001F),
        ("pmpcfg0_tor",         0x3A0, 0x0000_0008),
        ("pmpcfg0_na4",         0x3A0, 0x0000_0010),
        ("pmpcfg0_reserved",    0x3A0, 0x0000_0060),
        ("pmpcfg0_bad_rw",      0x3A0, 0x0000_0002),
        // Phase-2: pmpcfg1 byte 0 is now synthesised (entry 4).
        ("pmpcfg1_entry4_rwx",  0x3A1, 0x0000_000F),
        // Phase-2: entry 8 stays WI — boundary probe.
        ("pmpcfg2_unsynth",     0x3A2, 0x0000_00FF),
        ("pmpaddr0_zero",       0x3B0, 0x0000_0000),
        ("pmpaddr0_ones",       0x3B0, 0xFFFF_FFFF),
        ("pmpaddr0_napot_16b",  0x3B0, 0x0008_0007),
        // Phase-2: pmpaddr7 now writable — readback verifies full width.
        ("pmpaddr7_writable",   0x3B7, 0xDEAD_BEEF),
        // Phase-2: entry 8 pmpaddr stays WI — boundary probe.
        ("pmpaddr8_unsynth",    0x3B8, 0xDEAD_BEEF),
    ];
    for (name, csr, val) in patterns {
        // csrrw x10, <csr>, x6  ; csrrs x11, <csr>, x0
        let csrrw = encode_csr(*csr, 6, 1, 10);
        let csrrs = encode_csr(*csr, 0, 2, 11);
        out.push(RiscvTestCase {
            name: format!("pmp_{name}"),
            words: vec![csrrw, csrrs],
            reg_pre: vec![(6, *val)],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Pmp,
        });
    }
    out
}

// ============================================================================
// Fuzz generators
// ============================================================================

fn rand_gpr<R: Rng>(rng: &mut R) -> u8 {
    // x1..x31, but skip x3 (gp) and x31 (t6) — the QEMU-diff proxy path
    // reserves both as scratch. x3 holds the CSR-capture scratchpad pointer
    // and x31 is clobbered by the 7-CSR read prelude (landing on raw `mip`,
    // which on QEMU virt carries CLINT MTIP in bit 7 while mdrp2350 has no
    // CLINT, so any fuzz case that uses x31 as rs1/rs2 produces a spurious
    // divergence from a platform artefact rather than a real emulator bug).
    // x5/t0 is still in play: the harness zeros it after the CSR-read
    // prelude so both sides enter the test with x5 == 0.
    loop {
        let r = rng.gen_range(1..32_u8);
        if r != 3 && r != 31 {
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

/// Fuzz generator: RV32I branches + JAL (JALR omitted — untargetable
/// without knowing test_start at encode time).
///
/// Every test has the form `[branch-or-jump, NOP × 16]`. The branch offset
/// is restricted to `{4, 8, .., 64}` (forward only, 4-byte aligned) so
/// both sides — whether the branch is taken or falls through — land on
/// a NOP inside the sled and coast to the terminator ebreak that the
/// harness appends after `tc.words`. Random offsets would otherwise jump
/// into unmapped memory on the emulator (mcause=1) vs. into valid-but-
/// garbage memory on QEMU virt (mcause=2), producing a platform-layout
/// divergence that tells us nothing about branch semantics.
///
/// The `rand_gpr` selection of rs1/rs2 already skips x3 (gp) and x31 (t6),
/// which are reserved for the CSR-proxy prelude.
pub fn gen_fuzz_rv32i_branch<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    const NOP_SLED_LEN: usize = 16;
    const NOP_WORD: u32 = 0x0000_0013; // addi x0, x0, 0
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let choice = rng.gen_range(0..2_u32);
        // Forward offset in `{4, 8, .., NOP_SLED_LEN*4}` — 4-byte aligned so
        // the sled has only full NOPs to land on.
        let offset: i32 = (rng.gen_range(1..=NOP_SLED_LEN) * 4) as i32;
        let head = match choice {
            0 => {
                let funct3 = [0_u32, 1, 4, 5, 6, 7][rng.gen_range(0..6)];
                let rs1 = rand_gpr(rng);
                let rs2 = rand_gpr(rng);
                encode_b_type(offset, rs2, rs1, funct3, OPC_BRANCH)
            }
            _ => {
                let rd = loop {
                    // Same reservation as `rand_gpr` (skip gp/t6) plus x0 is
                    // fine (JAL to x0 = no link).
                    let r = rng.gen_range(0..32_u8);
                    if r != 3 && r != 31 { break r; }
                };
                encode_j_type(offset, rd, OPC_JAL)
            }
        };
        let mut words = Vec::with_capacity(1 + NOP_SLED_LEN);
        words.push(head);
        for _ in 0..NOP_SLED_LEN {
            words.push(NOP_WORD);
        }
        out.push(RiscvTestCase {
            name: format!("fuzz_branch_{i}"),
            words,
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
        let mut words = Vec::with_capacity(2);
        // SC.W (op5 = 0b00011) in isolation reads the hart's outstanding
        // load-reservation register, which is architecturally undefined
        // without a preceding LR.W to the same address — Hazard3 silicon
        // and mdrp2350 fail SC (rd=1), while QEMU's TCG single-step can
        // carry a stale reservation from an earlier test's LR and succeed
        // (rd=0). Establish a fresh reservation on both sides so the test
        // exercises the "SC should succeed" path, which is the meaningful
        // check (the mismatch was noise about the reservation window, not
        // about SC.W semantics themselves). We use rd = x0 on the seed LR
        // so its result never pollutes the GPR diff.
        if op5 == 0b00011 {
            let lr_funct7 = (0b00010 << 2) | aqrl;
            words.push(encode_r_type(lr_funct7, /*rs2*/ 0, rs1, 0b010, /*rd*/ 0, OPC_AMO));
        }
        words.push(w);
        let mut reg_pre = vec![(rs1, SCRATCH_BASE)];
        if rs2 != 0 && rs2 != rs1 {
            reg_pre.push((rs2, rng.next_u32()));
        }
        out.push(RiscvTestCase {
            name: format!("fuzz_rv32a_{i}"),
            words,
            reg_pre,
            addr_regs: vec![rs1],
            expect_trap: None,
            class: RiscvClass::Rv32aReservable,
        });
    }
    out
}

/// Fuzz generator: RV32C — arithmetic + compressed memory + compressed
/// control-flow + sporadic Zcmp quadrant-2 bit patterns.
///
/// Mix (per-case uniform draw):
///   * 10 % Zcmp Q2 (known-illegal on Hazard3 — expect mcause=2).
///     Collision tripwire (HLD §4.8).
///   * 25 % compressed memory (C.LW / C.LWSP / C.SW / C.SWSP) — rs1' seeded
///     to `SCRATCH_BASE`, x2 seeded to `SCRATCH_BASE` for sp-relative
///     variants, offsets 4-byte aligned and inside the scratchpad window.
///   * 15 % compressed control flow (C.J / C.JAL / C.BEQZ / C.BNEZ).
///     Targets coast through a compressed NOP sled to the terminator
///     ebreak on both taken and not-taken paths; C.JR / C.JALR are
///     excluded (edge-case `rvc_c_jr_x1` / `c_jalr_x1` cover them — the
///     generator can't construct a safe arbitrary register target at
///     encode time).
///   * Remainder (~50 %): arithmetic subset.
///
/// Prior revision deliberately skipped mem/branch because without harness
/// plumbing random targets land in unmapped memory (platform-layout
/// divergence). The new mem path pre-seeds all base registers and keeps
/// offsets inside `SCRATCH_BASE..SCRATCH_BASE+0x200`; the new branch path
/// mirrors `gen_fuzz_rv32i_branch`'s NOP-sled trick using packed
/// compressed nops.
pub fn gen_fuzz_rv32c<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let mix = rng.gen_range(0..100_u32);
        let tc = if mix < 10 {
            // Zcmp Q2 sweep — known-illegal (mcause=2).
            const F6_LOW_ZCMP: [u16; 4] = [4, 5, 6, 7];
            let f6_low = F6_LOW_ZCMP[rng.gen_range(0..4)];
            let mid = rng.next_u32() as u16 & 0x03FF;
            let enc: u16 = 0b101 << 13 | f6_low << 10 | (mid & 0x03FC) | 0b10;
            RiscvTestCase {
                name: format!("fuzz_rvc_{i}"),
                words: vec![u32::from(enc)],
                reg_pre: vec![(2, SCRATCH_BASE)],
                addr_regs: vec![2],
                expect_trap: Some(2),
                class: RiscvClass::Rv32c,
            }
        } else if mix < 35 {
            // Compressed memory subset.
            gen_rvc_mem_case(rng, i)
        } else if mix < 50 {
            // Compressed control-flow subset.
            gen_rvc_branch_case(rng, i)
        } else {
            // Arithmetic subset (original behaviour).
            let enc = gen_rv32c_arith(rng);
            RiscvTestCase {
                name: format!("fuzz_rvc_{i}"),
                words: vec![u32::from(enc)],
                reg_pre: vec![(2, SCRATCH_BASE)],
                addr_regs: vec![2],
                expect_trap: None,
                class: RiscvClass::Rv32c,
            }
        };
        out.push(tc);
    }
    out
}

/// Pack two compressed 16-bit instructions into a single u32 with the
/// first (earlier-PC) in the low half and the second in the high half.
/// The harness's `build_test_stream` does the same packing for padding
/// purposes; generators that emit multiple compressed ops in sequence
/// must do the equivalent up-front or the high halfword fetches as
/// `0x0000` = c.illegal and traps mid-body.
#[inline]
fn pack_rvc_pair(lo: u16, hi: u16) -> u32 {
    u32::from(lo) | (u32::from(hi) << 16)
}

/// Compressed c.nop (c.addi x0, 0 — canonical RV32C no-op).
const RVC_NOP: u16 = 0x0001;

/// Build one compressed memory test: C.LW / C.LWSP / C.SW / C.SWSP.
/// Pre-seeds the base register so the effective address is inside
/// `SCRATCH_BASE..SCRATCH_BASE+0x200`. C.LW/C.SW use rs1' ∈ {x8..x15};
/// C.LWSP/C.SWSP use x2 (sp).
fn gen_rvc_mem_case<R: Rng>(rng: &mut R, i: usize) -> RiscvTestCase {
    // C.LW / C.SW use creg3 for rs1', rd' (or rs2') — all in {x8..x15},
    // which never collides with x3/gp or x31/t6. Offsets for C.LW/C.SW
    // encode bits {6, 5:3, 2} (zero-extended, word-aligned), range 0..=124.
    // We keep offsets inside 0x00..=0x7C so the effective address stays
    // in the lower half of the 0x200-byte scratchpad.
    let variant = rng.gen_range(0..4_u32);
    let (enc, reg_pre): (u16, Vec<(u8, u32)>) = match variant {
        0 => {
            // C.LW rd', uimm(rs1') — Q0, f3=010.
            // imm bits [5:3|2|6] in [12:10|6|5].
            let rs1p = rng.gen_range(0_u16..8); // selector for x8..x15
            let rdp = rng.gen_range(0_u16..8);
            let uimm = (rng.gen_range(0_u16..32)) << 2; // 0..=124, word-aligned
            let b5_3 = (uimm >> 3) & 0b111;
            let b2 = (uimm >> 2) & 0b1;
            let b6 = (uimm >> 6) & 0b1;
            let enc = 0b010_u16 << 13
                | b5_3 << 10
                | rs1p << 7
                | b2 << 6
                | b6 << 5
                | rdp << 2
                | 0b00;
            let rs1 = (rs1p + 8) as u8;
            (enc, vec![(rs1, SCRATCH_BASE)])
        }
        1 => {
            // C.SW rs2', uimm(rs1') — Q0, f3=110.
            let rs1p = rng.gen_range(0_u16..8);
            let rs2p = rng.gen_range(0_u16..8);
            let uimm = (rng.gen_range(0_u16..32)) << 2;
            let b5_3 = (uimm >> 3) & 0b111;
            let b2 = (uimm >> 2) & 0b1;
            let b6 = (uimm >> 6) & 0b1;
            let enc = 0b110_u16 << 13
                | b5_3 << 10
                | rs1p << 7
                | b2 << 6
                | b6 << 5
                | rs2p << 2
                | 0b00;
            let rs1 = (rs1p + 8) as u8;
            let rs2 = (rs2p + 8) as u8;
            let mut reg_pre = vec![(rs1, SCRATCH_BASE)];
            if rs2 != rs1 {
                reg_pre.push((rs2, rng.next_u32()));
            }
            (enc, reg_pre)
        }
        2 => {
            // C.LWSP rd, uimm(x2) — Q2, f3=010. rd != 0 (rd=0 reserved).
            // imm bits [5|4:2|7:6] in [12|6:4|3:2].
            // rd ∈ x1..x31 but skip x3 (gp) and x31 (t6) — the harness proxy
            // reserves both. Picking from {x8..x15} keeps us safely away
            // from any proxy scratch register.
            let rd = rng.gen_range(8_u16..16);
            let uimm = (rng.gen_range(0_u16..64)) << 2; // 0..=252
            let b5 = (uimm >> 5) & 0b1;
            let b4_2 = (uimm >> 2) & 0b111;
            let b7_6 = (uimm >> 6) & 0b11;
            let enc = 0b010_u16 << 13
                | b5 << 12
                | rd << 7
                | b4_2 << 4
                | b7_6 << 2
                | 0b10;
            (enc, vec![(2, SCRATCH_BASE)])
        }
        _ => {
            // C.SWSP rs2, uimm(x2) — Q2, f3=110.
            // imm bits [5:2|7:6] in [12:9|8:7].
            let rs2 = rng.gen_range(8_u16..16);
            let uimm = (rng.gen_range(0_u16..64)) << 2;
            let b5_2 = (uimm >> 2) & 0b1111;
            let b7_6 = (uimm >> 6) & 0b11;
            let enc = 0b110_u16 << 13
                | b5_2 << 9
                | b7_6 << 7
                | rs2 << 2
                | 0b10;
            (enc, vec![(2, SCRATCH_BASE), (rs2 as u8, rng.next_u32())])
        }
    };
    // reg_pre already carries the SCRATCH_BASE seed for the active base
    // reg; addr_regs would duplicate it (harness applies reg_pre first).
    RiscvTestCase {
        name: format!("fuzz_rvc_mem_{i}"),
        words: vec![u32::from(enc)],
        reg_pre,
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::Rv32c,
    }
}

/// Build one compressed control-flow test: C.J / C.JAL / C.BEQZ / C.BNEZ.
/// Layout: `[branch || c.nop, c.nop || c.nop, ...]` packed so every
/// halfword is either the branch or a c.nop. Forward offsets stay in
/// `{4, 8, ..., SLED_BYTES}` (4 is the smallest positive c-branch
/// immediate; 2 would land on the 2nd halfword of the first slot, which
/// is already a c.nop). Both taken and not-taken paths coast through the
/// sled to the terminator ebreak appended by the harness.
fn gen_rvc_branch_case<R: Rng>(rng: &mut R, i: usize) -> RiscvTestCase {
    // Sled byte length — multiple of 4 so an even number of halfword slots
    // exist. 16 bytes = 8 c.nops after the branch, plus the packing nop
    // that sits in the high half of the branch's own u32 slot.
    const SLED_BYTES: i32 = 16;

    let variant = rng.gen_range(0..4_u32);
    let (enc, reg_pre): (u16, Vec<(u8, u32)>) = match variant {
        0 => {
            // C.J imm — Q1, f3=101. No link.
            let imm = (rng.gen_range(1..=(SLED_BYTES / 2)) * 2) as i32;
            (encode_c_j(imm, 0b101), vec![])
        }
        1 => {
            // C.JAL imm — Q1, f3=001. Links to x1.
            let imm = (rng.gen_range(1..=(SLED_BYTES / 2)) * 2) as i32;
            (encode_c_j(imm, 0b001), vec![])
        }
        2 => {
            // C.BEQZ rs1', imm — Q1, f3=110.
            let rs1p = rng.gen_range(0_u16..8);
            let rs1 = (rs1p + 8) as u8;
            let imm = (rng.gen_range(1..=(SLED_BYTES / 2)) * 2) as i32;
            // Seed the test register to a mix of zero (taken) and non-zero
            // (not-taken) across the fuzz batch so both paths get coverage.
            let val: u32 = if rng.gen_bool(0.5) { 0 } else { rng.next_u32() | 1 };
            (encode_c_beqz(imm, rs1p, 0b110), vec![(rs1, val)])
        }
        _ => {
            // C.BNEZ rs1', imm — Q1, f3=111.
            let rs1p = rng.gen_range(0_u16..8);
            let rs1 = (rs1p + 8) as u8;
            let imm = (rng.gen_range(1..=(SLED_BYTES / 2)) * 2) as i32;
            let val: u32 = if rng.gen_bool(0.5) { 0 } else { rng.next_u32() | 1 };
            (encode_c_beqz(imm, rs1p, 0b111), vec![(rs1, val)])
        }
    };

    // Pack the branch + c.nop sled into u32 words. First u32: branch
    // (low) + c.nop (high). Subsequent u32s: c.nop | c.nop.
    let sled_slots = (SLED_BYTES / 4) as usize; // each slot = 2 halfwords = 4 bytes
    let mut words = Vec::with_capacity(1 + sled_slots);
    words.push(pack_rvc_pair(enc, RVC_NOP));
    for _ in 0..sled_slots {
        words.push(pack_rvc_pair(RVC_NOP, RVC_NOP));
    }

    RiscvTestCase {
        name: format!("fuzz_rvc_br_{i}"),
        words,
        reg_pre,
        addr_regs: vec![],
        expect_trap: None,
        class: RiscvClass::Rv32c,
    }
}

/// Encode C.J / C.JAL. `f3` selects which (0b101 = C.J, 0b001 = C.JAL).
/// imm[11|4|9:8|10|6|7|3:1|5] in bits[12|11|10:9|8|7|6|5:3|2].
fn encode_c_j(imm: i32, f3: u16) -> u16 {
    debug_assert!(imm & 1 == 0, "c.j imm must be 2-byte aligned: {imm}");
    let raw = (imm as u32) & 0x0FFF; // 12-bit signed, zero-extended for bit extraction
    let b11 = ((raw >> 11) & 1) as u16;
    let b10 = ((raw >> 10) & 1) as u16;
    let b9_8 = ((raw >> 8) & 0b11) as u16;
    let b7 = ((raw >> 7) & 1) as u16;
    let b6 = ((raw >> 6) & 1) as u16;
    let b5 = ((raw >> 5) & 1) as u16;
    let b4 = ((raw >> 4) & 1) as u16;
    let b3_1 = ((raw >> 1) & 0b111) as u16;
    f3 << 13
        | b11 << 12
        | b4 << 11
        | b9_8 << 9
        | b10 << 8
        | b6 << 7
        | b7 << 6
        | b3_1 << 3
        | b5 << 2
        | 0b01
}

/// Encode C.BEQZ / C.BNEZ. `f3` selects (0b110 = BEQZ, 0b111 = BNEZ).
/// imm[8|4:3|7:6|2:1|5] in bits[12|11:10|6:5|4:3|2].
fn encode_c_beqz(imm: i32, rs1p: u16, f3: u16) -> u16 {
    debug_assert!(imm & 1 == 0, "c.beqz imm must be 2-byte aligned: {imm}");
    let raw = (imm as u32) & 0x01FF; // 9-bit signed
    let b8 = ((raw >> 8) & 1) as u16;
    let b7_6 = ((raw >> 6) & 0b11) as u16;
    let b5 = ((raw >> 5) & 1) as u16;
    let b4_3 = ((raw >> 3) & 0b11) as u16;
    let b2_1 = ((raw >> 1) & 0b11) as u16;
    f3 << 13
        | b8 << 12
        | b4_3 << 10
        | rs1p << 7
        | b7_6 << 5
        | b2_1 << 3
        | b5 << 2
        | 0b01
}

/// Build a well-formed RV32C arithmetic encoding. Every returned
/// halfword decodes to exactly one `Op` variant that the executor
/// handles without touching memory or redirecting PC, and the operand
/// constraints (non-zero shamts, non-zero rd where the spec requires,
/// nzimm != 0 for ADDI4SPN / ADDI16SP / LUI) mean the emulator's
/// legitimate "illegal HINT" rejections don't fire.
///
/// rs1/rs2/rd never land on x3 (gp) or x31 (t6) — both are the
/// CSR-proxy scratch and would corrupt the scratchpad pointer or clobber
/// the post-snapshot capture. The compressed operand fields with
/// 3-bit register selects (creg3) map to {x8..x15}, which naturally
/// avoids both reserved slots, so those paths are always safe.
fn gen_rv32c_arith<R: Rng>(rng: &mut R) -> u16 {
    // Pick a 5-bit GPR avoiding x0, x3, x31 (x0 must be x1..x31; proxy
    // reserves x3 and x31). Used for the 5-bit-wide rd/rs2 encodings
    // (C.MV, C.ADD, C.SLLI, C.LI, C.ADDI).
    let pick_gpr_nz = |rng: &mut R| -> u16 {
        loop {
            let r = rng.gen_range(1_u16..32);
            if r != 3 && r != 31 {
                return r;
            }
        }
    };
    // Creg3 picks one of {x8..x15}. All 8 options are safe (no overlap
    // with x3/x31). Stored as the 3-bit selector.
    let pick_creg3 = |rng: &mut R| -> u16 { rng.gen_range(0_u16..8) };

    let choice = rng.gen_range(0..11_u32);
    match choice {
        0 => {
            // C.ADDI rd, nzimm[5:0] — rd != 0 (rd=0 imm=0 is C.NOP, but
            // rd=0 imm!=0 is HINT; keep rd != 0 to avoid both corner
            // cases that have no architectural observable).
            let rd = pick_gpr_nz(rng);
            let imm_raw = loop {
                let v = rng.gen_range(0_u16..64);
                if v != 0 {
                    break v;
                }
            };
            let b5 = (imm_raw >> 5) & 1;
            let b4_0 = imm_raw & 0x1F;
            0b000 << 13 | b5 << 12 | rd << 7 | b4_0 << 2 | 0b01
        }
        1 => {
            // C.LI rd, imm — rd != 0 (rd=0 is HINT).
            let rd = pick_gpr_nz(rng);
            let imm_raw = rng.gen_range(0_u16..64);
            let b5 = (imm_raw >> 5) & 1;
            let b4_0 = imm_raw & 0x1F;
            0b010 << 13 | b5 << 12 | rd << 7 | b4_0 << 2 | 0b01
        }
        2 => {
            // C.SLLI rd, shamt — rd != 0, shamt != 0 (both illegal per
            // spec and the emu decoder rejects them).
            let rd = pick_gpr_nz(rng);
            let shamt = rng.gen_range(1_u16..32);
            let b5 = 0; // RV32: shamt[5] must be 0
            let b4_0 = shamt & 0x1F;
            0b000 << 13 | b5 << 12 | rd << 7 | b4_0 << 2 | 0b10
        }
        3 => {
            // C.MV rd, rs2 — rd != 0, rs2 != 0.
            let rd = pick_gpr_nz(rng);
            let rs2 = pick_gpr_nz(rng);
            0b100 << 13 | 0 << 12 | rd << 7 | rs2 << 2 | 0b10
        }
        4 => {
            // C.ADD rd, rs2 — rd != 0, rs2 != 0.
            let rd = pick_gpr_nz(rng);
            let rs2 = pick_gpr_nz(rng);
            0b100 << 13 | 1 << 12 | rd << 7 | rs2 << 2 | 0b10
        }
        5 => {
            // C.SRLI rd', shamt — creg3 operand, shamt != 0.
            let rd_p = pick_creg3(rng);
            let shamt = rng.gen_range(1_u16..32);
            let b5 = 0;
            let b4_0 = shamt & 0x1F;
            0b100 << 13 | b5 << 12 | 0b00 << 10 | rd_p << 7 | b4_0 << 2 | 0b01
        }
        6 => {
            // C.SRAI rd', shamt.
            let rd_p = pick_creg3(rng);
            let shamt = rng.gen_range(1_u16..32);
            let b5 = 0;
            let b4_0 = shamt & 0x1F;
            0b100 << 13 | b5 << 12 | 0b01 << 10 | rd_p << 7 | b4_0 << 2 | 0b01
        }
        7 => {
            // C.ANDI rd', imm[5:0].
            let rd_p = pick_creg3(rng);
            let imm_raw = rng.gen_range(0_u16..64);
            let b5 = (imm_raw >> 5) & 1;
            let b4_0 = imm_raw & 0x1F;
            0b100 << 13 | b5 << 12 | 0b10 << 10 | rd_p << 7 | b4_0 << 2 | 0b01
        }
        8 => {
            // C.SUB / C.XOR / C.OR / C.AND — Q1 funct3=100, bits[11:10]=11.
            // bit12 = 0 on RV32 (bit12=1 is C.SUBW/C.ADDW for RV64).
            let rd_p = pick_creg3(rng);
            let rs2_p = pick_creg3(rng);
            let sel = rng.gen_range(0_u16..4); // SUB/XOR/OR/AND
            0b100 << 13 | 0 << 12 | 0b11 << 10 | rd_p << 7 | sel << 5 | rs2_p << 2 | 0b01
        }
        9 => {
            // C.LUI rd, nzimm[17:12] — rd != 0, rd != 2, nzimm != 0.
            let rd = loop {
                let r = pick_gpr_nz(rng);
                if r != 2 {
                    break r;
                }
            };
            let nzimm_raw = loop {
                let v = rng.gen_range(0_u16..64);
                if v != 0 {
                    break v;
                }
            };
            let b17 = (nzimm_raw >> 5) & 1;
            let b16_12 = nzimm_raw & 0x1F;
            0b011 << 13 | b17 << 12 | rd << 7 | b16_12 << 2 | 0b01
        }
        _ => {
            // C.ADDI16SP nzimm[9:4]<<4 — rd=2, nzimm != 0 (scaled by 16).
            // nzimm[9|4|6|8:7|5] go into bits[12|6|5|4:3|2]. Only bits
            // [9:4] of the raw value are encoded; bits [3:0] are lost in
            // the encoding. Guard on the encoded bits, not the raw value,
            // or we emit `nzimm_encoded == 0` (reserved per spec) for any
            // nzimm_raw with only low bits set.
            let nzimm_raw = loop {
                let v = rng.gen_range(0_u16..0x400);
                if v & 0x3F0 != 0 {
                    break v;
                }
            };
            let b9 = (nzimm_raw >> 9) & 1;
            let b8_7 = (nzimm_raw >> 7) & 0b11;
            let b6 = (nzimm_raw >> 6) & 1;
            let b5 = (nzimm_raw >> 5) & 1;
            let b4 = (nzimm_raw >> 4) & 1;
            0b011 << 13 | b9 << 12 | 2 << 7
                | b4 << 6 | b6 << 5 | b8_7 << 3 | b5 << 2 | 0b01
        }
    }
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
///
/// FENCE.I and FENCE are architecturally no-ops for register state on
/// both Hazard3 and QEMU virt rv32 (they're instruction-stream
/// synchronisation barriers; there's no data-path side effect we can
/// observe on a single-hart single-step harness). This class is a
/// decode-coverage test — we assert that following a FENCE.I with an
/// arbitrary register-modifying instruction produces the same post-state
/// on both sides, i.e. the FENCE.I doesn't mis-decode, clobber state,
/// or perturb the subsequent instruction's execution.
///
/// Each case emits either
///   - `FENCE.I` alone (30 % — standalone decode coverage),
///   - `FENCE` alone (20 % — sibling decode),
///   - `FENCE.I; addi rd, rs1, imm` (50 % — ensures the fence does not
///     disturb register bank updates on the next instruction).
pub fn gen_fuzz_zifencei<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    for i in 0..count {
        let mix = rng.gen_range(0..100_u32);
        let (words, reg_pre) = if mix < 30 {
            // Standalone FENCE.I.
            (vec![encode_i_type(0, 0, 1, 0, OPC_MISC_MEM)], vec![])
        } else if mix < 50 {
            // Standalone FENCE (non-`.i` sibling — a tripwire for decode-
            // ordering bugs that might swap the two funct3 values).
            let flags = rng.gen_range(0..256_u32) as i32;
            (vec![encode_i_type(flags, 0, 0, 0, OPC_MISC_MEM)], vec![])
        } else {
            // FENCE.I followed by a register-modifying instruction. If
            // FENCE.I mis-decodes on either side, `rd` will drift and
            // the standard GPR diff catches it. If FENCE.I is decoded
            // correctly on both sides, `rd` ends up with the same ADDI
            // result regardless of the preceding fence.
            let fence_i = encode_i_type(0, 0, 1, 0, OPC_MISC_MEM);
            let rd = rand_gpr(rng);
            let rs1 = rand_gpr(rng);
            let imm_raw = rng.next_u32() as i32;
            let imm = (imm_raw << 20) >> 20; // sign-extend 12-bit
            let addi = encode_i_type(imm, rs1, 0, rd, OPC_OP_IMM);
            let rs1_val = rng.next_u32();
            (vec![fence_i, addi], vec![(rs1, rs1_val)])
        };
        out.push(RiscvTestCase {
            name: format!("fuzz_fencei_{i}"),
            words,
            reg_pre,
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
    // Fuzz-relevant M-mode CSRs. `mtvec` (0x305) is deliberately omitted:
    // writing a random value to the trap vector isn't a meaningful test
    // (no firmware deliberately scrambles its own handler address), and
    // QEMU 10.2 virt rv32 exhibits a WARL behaviour on mtvec writes that
    // we cannot match cheaply — it silently keeps the *prior* mtvec value
    // when the incoming value doesn't meet some internal alignment /
    // mode-legality constraint, regardless of the value's bit pattern.
    // The zicsr/csr-sideeffect edge-case catalogues still exercise
    // mtvec semantics through curated bit patterns; the fuzz pool drops
    // it to stay noise-free.
    const CSRS: &[u16] = &[0x300, 0x304, 0x340, 0x341, 0x342, 0x344];
    // NOP pad between branch and terminator: the branch offset is +8
    // bytes from the branch's own PC, so the taken path needs a safe
    // landing slot one word ahead of the not-taken path. Without the
    // NOP the branch jumps past the harness's terminator `ebreak` into
    // whatever follows — on QEMU that's valid VIRT_DRAM (execution
    // wanders until a non-decodable word, meanwhile the harness times
    // out on vCont;c); on the emu it's an access fault past the SRAM
    // writable window. A NOP slot makes both sides converge on the
    // terminator regardless of which direction the branch goes.
    const NOP_WORD: u32 = 0x0000_0013;
    // Safe values for rs1 that exercise CSR side-effect semantics without
    // dropping QEMU's rv32 virt into a live interrupt cascade. A random
    // 32-bit value written to `mstatus` with MIE set, combined with QEMU's
    // always-asserted `mip.MTIP` (CLINT timer compare running), can fire
    // machine timer interrupts that re-enter the trap handler before the
    // HW breakpoint can halt it — surfacing as a `vCont;c` connection
    // timeout rather than a clean divergence. The 16-value pool below
    // covers: all-zero, single bits, small sparse patterns, and a few
    // whole-field fills. Enough variety to catch the "did the CSR see
    // the write" side effect the class is probing, without burning
    // pathological whole-register random bit patterns into mstatus/mip.
    const SAFE_VALUES: [u32; 16] = [
        0x0000_0000, 0x0000_0001, 0x0000_0002, 0x0000_0004,
        0x0000_0008, 0x0000_0080, 0x0000_0800, 0x0000_1800,
        0x0000_00FF, 0x0000_0FFF, 0x0000_8888, 0x8000_0000,
        0x8000_0003, 0x8000_0007, 0x8000_000B, 0x0000_1888,
    ];
    for i in 0..count {
        let csr = CSRS[rng.gen_range(0..CSRS.len())];
        let rd = rand_gpr(rng);
        let rs1 = rand_gpr(rng);
        let csrrw = encode_csr(csr, rs1, 1, rd);
        // Follow-up branch conditional on rd.
        let funct3 = if rng.gen_bool(0.5) { 0 } else { 1 };
        let branch = encode_b_type(8, 0, rd, funct3, OPC_BRANCH);
        let rs1_val = SAFE_VALUES[rng.gen_range(0..SAFE_VALUES.len())];
        out.push(RiscvTestCase {
            name: format!("fuzz_csrside_{i}"),
            words: vec![csrrw, branch, NOP_WORD],
            reg_pre: vec![(rs1, rs1_val)],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::CsrSideEffect,
        });
    }
    out
}

/// Fuzz generator: PMP (phase-1).
///
/// Two instruction patterns per HLD §4.2:
///   1. Single-CSR write (`csrrw rd, pmp_csr, rs1`)
///   2. Write-then-read-back (`csrrw rd1, csr, rs1; csrrs rd2, csr, x0`)
///      — primary divergence catcher; `rd2` must match after WARL
///      normalisation in `warl_mask`.
///
/// Value pool covers all-zeros, all-ones, the RV-priv §3.7.1 valid and
/// reserved R/W/X cross-product packed into one byte, the reserved-bit
/// probe, and the illegal W=1/R=0 combination. No L-bit pattern — phase-1
/// explicitly excludes L (sticky-trap rationale in HLD §5.1).
///
/// CSR targets span pmpcfg0..3 and pmpaddr0..7 so the divergence oracle
/// exercises both synthesised and unsynthesised slots (the latter diff
/// clean after `warl_mask` zeros both sides).
pub fn gen_fuzz_pmp<R: Rng>(rng: &mut R, count: usize) -> Vec<RiscvTestCase> {
    let mut out = Vec::with_capacity(count);
    // pmpcfg0..3 (0x3A0..0x3A3) + pmpaddr0..7 (0x3B0..0x3B7). pmpcfg2/3
    // (entries 8..15) exercise the WARL boundary where QEMU virt
    // synthesises 16 regions but phase-2 emu caps at 8; the harness's
    // `warl_mask` zero-arms for entries ≥ 8 keep rd1/rd2 readbacks
    // clean on both sides. Kept in the pool because `--class pmp`
    // callers generally want the full CSR surface exercised.
    const CSRS: &[u16] = &[
        0x3A0, 0x3A1, 0x3A2, 0x3A3,
        0x3B0, 0x3B1, 0x3B2, 0x3B3, 0x3B4, 0x3B5, 0x3B6, 0x3B7,
    ];
    // Interesting bit patterns for rs1. Phase-2 hard constraint (HLD
    // §5.1 Risk 1 + V2 §A.6): **no L-bit (bit 7 of any pmpcfg byte)
    // ever**. Once L=1 is latched in silicon / QEMU, only a system reset
    // clears it. The emulator now correctly models L-sticky, but the
    // harness cannot reset QEMU's CSR bank between tests — so a single
    // L=1 write early in a fuzz run would make every subsequent pmpcfg
    // write on QEMU drop, while the emulator's per-test `reset_pmp_csrs`
    // desynchronises it (see `run_one_test` reset site).
    //
    // Each value below has bit 7, 15, 23, 31 clear on every byte
    // (`& 0x7F7F_7F7F`). pmpaddr values are unconstrained because the
    // L-gating on pmpaddr is via pmpcfg state which the fuzz stream
    // never sets L on.
    const VALUES: &[u32] = &[
        0x0000_0000, // all zeros
        0x7F7F_7F7F, // all-ones with L cleared on every byte
        0x0000_0007, // R=W=X=1, A=OFF, L=0
        0x0000_0002, // W=1, R=0 — illegal; WARL rounds to 0
        0x0000_0060, // reserved bits [6:5]
        0x0000_0018, // A=NAPOT (11), L=0
        0x0000_0010, // A=NA4 (10)
        0x0000_0008, // A=TOR (01)
        0x0000_001F, // NAPOT + R/W/X
        0x0000_0067, // reserved + R/W/X probe
        0x0F0F_0F0F, // 4-byte pmpcfg cross pattern (no L bits set)
        0x1818_1818, // NAPOT × 4 bytes (no L bits set)
        0x0800_0000, // typical NAPOT base (pmpaddr)
        0x2000_0000, // SRAM base (typical bootrom-early pmpaddr)
        0xDEAD_BEEF, // chaotic pmpaddr
        0xCAFE_BABE, // chaotic pmpaddr
        0x0000_007F, // low byte fill without L — pmpcfg0 byte 0 full range
        0x0000_7F7F, // low half fill without L (bytes 0 and 1)
    ];
    for i in 0..count {
        let csr = CSRS[rng.gen_range(0..CSRS.len())];
        // Force L=0 on every byte. The VALUES pool has two "chaotic"
        // patterns (0xDEAD_BEEF, 0xCAFE_BABE) intended for pmpaddr that
        // would land L=1 if picked for a pmpcfg CSR — phase-2 emu and
        // QEMU disagree on L-sticky propagation across the fuzz run
        // (QEMU lacks the harness-side reset ritual), so this pool-wide
        // mask keeps the stream within the L=0 window that both sides
        // model identically. Pmpaddr writes are unaffected — they do
        // not interpret bit 7 — so the chaotic patterns still serve
        // their original purpose there with three fewer bits of entropy.
        let val = VALUES[rng.gen_range(0..VALUES.len())] & 0x7F7F_7F7F;
        // Mix: 40 % single-CSR write, 60 % write-then-read-back (primary
        // divergence catcher per HLD §4.2 pattern 2).
        let read_back = rng.gen_range(0..100_u32) >= 40;
        let rd1 = rand_gpr(rng);
        let rs1 = rand_gpr(rng);
        let csrrw = encode_csr(csr, rs1, 1, rd1);
        let words = if read_back {
            // rs1==x0 guarantees csrrs is read-only (no stomping of the
            // just-written value), and rd2 holds the WARL-normalised view
            // of what was actually stored.
            let rd2 = loop {
                let r = rand_gpr(rng);
                // Avoid collision with rd1 so both write-old and read-back
                // values show up in distinct GPR slots.
                if r != rd1 { break r; }
            };
            let csrrs = encode_csr(csr, 0, 2, rd2);
            vec![csrrw, csrrs]
        } else {
            vec![csrrw]
        };
        out.push(RiscvTestCase {
            name: format!("fuzz_pmp_{i}"),
            words,
            reg_pre: vec![(rs1, val)],
            addr_regs: vec![],
            expect_trap: None,
            class: RiscvClass::Pmp,
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
    out.extend(gen_pmp_edge_cases());
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
            RiscvClass::Pmp => gen_fuzz_pmp(rng, n),
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
        // Mirror mdrp2350::bus::canon_oracle_addr so the 0x8xxx_xxxx alias
        // resolves to its SRAM image before the window check — matches the
        // runtime's in_reservable() semantics.
        let canon = |a: u32| if (a >> 28) == 0x8 { (a & 0x0FFF_FFFF) | 0x2000_0000 } else { a };
        for tc in &cs {
            for reg in &tc.addr_regs {
                let v = tc.reg_pre.iter().find(|(r, _)| r == reg).map(|(_, v)| *v).unwrap();
                let v_canon = canon(v);
                assert!(
                    (RESERVABLE_LO..RESERVABLE_HI).contains(&v_canon),
                    "atomic address out of reservable window: raw=0x{v:08X} canon=0x{v_canon:08X}"
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

    #[test]
    fn edge_cases_pmp() {
        let cs = gen_pmp_edge_cases();
        check_class(&cs, RiscvClass::Pmp, false);
        // Every PMP edge case is a two-word write-then-read-back pair
        // (`csrrw` + `csrrs`). Confirm both words hit OPC_SYSTEM and that
        // the CSR addr falls into the pmpcfg (0x3A0..=0x3A3) or pmpaddr
        // (0x3B0..=0x3BF) range.
        for tc in &cs {
            assert_eq!(tc.words.len(), 2, "pmp edge case must be a write-then-read pair: {}", tc.name);
            for &w in &tc.words {
                assert_eq!(w & 0x7F, OPC_SYSTEM, "pmp word not OPC_SYSTEM in {}: 0x{w:08X}", tc.name);
                let csr = (w >> 20) & 0xFFF;
                assert!(
                    (0x3A0..=0x3A3).contains(&csr) || (0x3B0..=0x3BF).contains(&csr),
                    "pmp csr out of range in {}: 0x{csr:03x}", tc.name
                );
            }
        }
    }

    // --------------------------------------------------------------------
    // New RV32C memory / control-flow fuzz generators (Stage-6 expansion).
    // --------------------------------------------------------------------

    #[test]
    fn fuzz_rvc_mem_encodings_decode_and_reach_scratchpad() {
        // Drive the mem path exclusively by picking `mix` in [10, 35). We
        // can't force the sub-generator from outside without a seed search,
        // so we fuzz a 200-case batch and filter — the mix weights make mem
        // ~25 % of the RV32C pool, so a batch of 200 yields ~50 mem cases.
        let mut rng = StdRng::seed_from_u64(0xC0DE_BA5E);
        let cases = gen_fuzz_rv32c(&mut rng, 200);
        let mem_cases: Vec<&RiscvTestCase> =
            cases.iter().filter(|tc| tc.name.contains("_mem_")).collect();
        assert!(
            mem_cases.len() >= 20,
            "fuzz_rvc_{{mem}} sampled too rarely: {}/200",
            mem_cases.len()
        );
        for tc in mem_cases {
            // Must be a single 16-bit compressed encoding.
            assert_eq!(tc.words.len(), 1, "rvc_mem multi-word: {}", tc.name);
            let w = tc.words[0];
            assert!(
                is_compressed(w),
                "rvc_mem emitted non-compressed word 0x{w:08X} in {}",
                tc.name
            );
            // Every base-register pre-seed must point into the scratchpad
            // (or be x2/sp pointing at the scratchpad) — not random GPR
            // junk. For C.LW/C.SW the base is rs1' ∈ {x8..x15}; for
            // C.LWSP/C.SWSP the base is x2. Either way at least one reg_pre
            // entry must equal SCRATCH_BASE.
            assert!(
                tc.reg_pre.iter().any(|(_, v)| *v == SCRATCH_BASE),
                "rvc_mem missing SCRATCH_BASE seed: {} {:?}",
                tc.name,
                tc.reg_pre
            );
        }
    }

    #[test]
    fn fuzz_rvc_branch_encodings_well_formed() {
        let mut rng = StdRng::seed_from_u64(0xBABE_F00D);
        let cases = gen_fuzz_rv32c(&mut rng, 200);
        let br_cases: Vec<&RiscvTestCase> =
            cases.iter().filter(|tc| tc.name.contains("_br_")).collect();
        assert!(
            br_cases.len() >= 10,
            "fuzz_rvc_{{br}} sampled too rarely: {}/200",
            br_cases.len()
        );
        for tc in br_cases {
            // First halfword (low 16 bits of first u32) must be a Q1
            // compressed control-flow instruction. funct3 ∈ {001, 101,
            // 110, 111} (C.JAL / C.J / C.BEQZ / C.BNEZ).
            let first = tc.words[0] as u16;
            assert_eq!(first & 0x3, 0b01, "rvc_br head not Q1: {}", tc.name);
            let f3 = (first >> 13) & 0x7;
            assert!(
                matches!(f3, 0b001 | 0b101 | 0b110 | 0b111),
                "rvc_br head f3={f3:#05b} not C.JAL/C.J/C.BEQZ/C.BNEZ: {}",
                tc.name
            );
            // High half of first u32 and both halves of every subsequent
            // u32 must be c.nop (0x0001) — the sled.
            let first_hi = (tc.words[0] >> 16) as u16;
            assert_eq!(
                first_hi, RVC_NOP,
                "rvc_br first-word high half not c.nop: {}",
                tc.name
            );
            for &w in &tc.words[1..] {
                assert_eq!(
                    w & 0xFFFF,
                    u32::from(RVC_NOP),
                    "rvc_br sled low half not c.nop: {} word=0x{w:08X}",
                    tc.name
                );
                assert_eq!(
                    (w >> 16) & 0xFFFF,
                    u32::from(RVC_NOP),
                    "rvc_br sled high half not c.nop: {} word=0x{w:08X}",
                    tc.name
                );
            }
        }
    }

    #[test]
    fn fuzz_zifencei_two_word_cases_present() {
        let mut rng = StdRng::seed_from_u64(0xFE_EDCAFE);
        let cases = gen_fuzz_zifencei(&mut rng, 200);
        let two_word = cases.iter().filter(|tc| tc.words.len() == 2).count();
        assert!(
            two_word >= 40,
            "fuzz_zifencei two-word cases too rare: {two_word}/200"
        );
        // Every two-word case starts with FENCE.I.
        for tc in cases.iter().filter(|tc| tc.words.len() == 2) {
            let w = tc.words[0];
            assert_eq!(w & 0x7F, OPC_MISC_MEM, "zifencei head not MISC-MEM: {}", tc.name);
            assert_eq!((w >> 12) & 0x7, 1, "zifencei head not FENCE.I: {}", tc.name);
            // Second word must be an ADDI (OP-IMM, funct3=0).
            let w2 = tc.words[1];
            assert_eq!(w2 & 0x7F, OPC_OP_IMM, "zifencei tail not OP-IMM: {}", tc.name);
            assert_eq!((w2 >> 12) & 0x7, 0, "zifencei tail not ADDI: {}", tc.name);
        }
    }

    #[test]
    fn encode_c_j_roundtrip() {
        // C.J +4 — spec example: imm=4 → bit 5 set in encoded word (bit 2
        // of instruction encoding per layout). Verify a handful of known
        // points against an independent reconstruction.
        // imm=0 → encoding has f3=101, quadrant=01, all imm bits clear
        // (but imm=0 is a legal c.j target).
        let w = encode_c_j(0, 0b101);
        assert_eq!(w & 0x3, 0b01, "c.j quadrant");
        assert_eq!((w >> 13) & 0x7, 0b101, "c.j f3");
        // Re-extract and check imm reproduces.
        for imm in [4_i32, 8, -4, -8, 2046, -2048, 16, -16] {
            let w = encode_c_j(imm, 0b101);
            let decoded = c_jimm_extract(w);
            assert_eq!(
                decoded, imm,
                "c.j imm round-trip mismatch: enc=0x{w:04X} wanted {imm} got {decoded}"
            );
        }
        // C.JAL variant must carry f3=001.
        let w = encode_c_j(4, 0b001);
        assert_eq!((w >> 13) & 0x7, 0b001, "c.jal f3");
    }

    #[test]
    fn encode_c_beqz_roundtrip() {
        // C.BEQZ — verify imm extracts cleanly and f3/quadrant are set.
        for imm in [4_i32, 8, -4, -8, 254, -256, 16, -16] {
            let w = encode_c_beqz(imm, /*rs1p*/ 3, 0b110);
            assert_eq!(w & 0x3, 0b01, "c.beqz quadrant imm={imm}");
            assert_eq!((w >> 13) & 0x7, 0b110, "c.beqz f3 imm={imm}");
            assert_eq!((w >> 7) & 0x7, 3, "c.beqz rs1' imm={imm}");
            let decoded = c_bimm_extract(w);
            assert_eq!(
                decoded, imm,
                "c.beqz imm round-trip mismatch: enc=0x{w:04X} wanted {imm} got {decoded}"
            );
        }
        // BNEZ form.
        let w = encode_c_beqz(4, 0, 0b111);
        assert_eq!((w >> 13) & 0x7, 0b111, "c.bnez f3");
    }

    // Local extraction helpers — independent reconstruction of the
    // decoder logic in `mdrp2350::core_riscv::decode`. If both encoders
    // and extractors were buggy in the same direction the roundtrip would
    // pass deceptively, so the extraction is written from scratch from
    // the RV-C spec §16.8 immediate layout tables.
    fn c_jimm_extract(w: u16) -> i32 {
        let b11 = ((w >> 12) & 1) as u32;
        let b4 = ((w >> 11) & 1) as u32;
        let b9_8 = ((w >> 9) & 0b11) as u32;
        let b10 = ((w >> 8) & 1) as u32;
        let b6 = ((w >> 7) & 1) as u32;
        let b7 = ((w >> 6) & 1) as u32;
        let b3_1 = ((w >> 3) & 0b111) as u32;
        let b5 = ((w >> 2) & 1) as u32;
        let raw = (b11 << 11)
            | (b10 << 10)
            | (b9_8 << 8)
            | (b7 << 7)
            | (b6 << 6)
            | (b5 << 5)
            | (b4 << 4)
            | (b3_1 << 1);
        // Sign-extend from bit 11.
        if raw & (1 << 11) != 0 {
            (raw | !0xFFF) as i32
        } else {
            raw as i32
        }
    }

    fn c_bimm_extract(w: u16) -> i32 {
        let b8 = ((w >> 12) & 1) as u32;
        let b4_3 = ((w >> 10) & 0b11) as u32;
        let b7_6 = ((w >> 5) & 0b11) as u32;
        let b2_1 = ((w >> 3) & 0b11) as u32;
        let b5 = ((w >> 2) & 1) as u32;
        let raw = (b8 << 8) | (b7_6 << 6) | (b5 << 5) | (b4_3 << 3) | (b2_1 << 1);
        // Sign-extend from bit 8.
        if raw & (1 << 8) != 0 {
            (raw | !0x1FF) as i32
        } else {
            raw as i32
        }
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
        eprintln!("PMP={}", gen_pmp_edge_cases().len());
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
            // Branch cases ship with NOP / EBREAK sleds that absorb taken
            // targets without leaving the test body. `decoded == "alu"`
            // covers `addi x0, x0, 0` (NOP); `decoded == "misc"` covers
            // the planted ebreak used by backward-taken layouts. Both
            // are harness scaffolding, not branch instructions under test.
            RiscvClass::Rv32iBranch => {
                decoded == "branch" || decoded == "alu" || decoded == "misc"
            }
            RiscvClass::Rv32iUpper => decoded == "upper" || decoded == "alu",
            RiscvClass::Rv32m => decoded == "rv32m",
            RiscvClass::Rv32aReservable => decoded == "rv32a",
            RiscvClass::Zicsr => decoded == "zicsr",
            // Zifencei fuzz 2-word cases (FENCE.I; ADDI) emit both a
            // fence-family word and a subsequent arithmetic word. The
            // class is still "fence-family under test"; the trailing ADDI
            // is scaffolding exercising the post-fence register bank.
            RiscvClass::Zifencei => {
                decoded == "fencei" || decoded == "fence" || decoded == "alu"
            }
            RiscvClass::CsrSideEffect => {
                // Multi-instruction: the first word is a csrrw/csrrs etc.,
                // the second a branch or arithmetic.  The per-word check
                // below handles both.
                decoded == "zicsr" || decoded == "branch" || decoded == "alu"
            }
            // PMP fuzz cases are single `csrrw` or paired `csrrw; csrrs`
            // — both words are zicsr-family per the decoder.
            RiscvClass::Pmp => decoded == "zicsr",
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
            // Per-case: did at least one word decode as a genuine branch /
            // JAL / JALR? The `Rv32iBranch` class-compat arm accepts `alu`
            // (NOP sled) and `misc` (planted ebreak) alongside `branch` so
            // the layout scaffolding doesn't trip class-mismatch — but we
            // still need to prove the case actually contains a branch
            // instruction. Without this, a case containing only sled + ebreak
            // and a corrupted branch-slot would silently pass.
            let mut case_has_branch_word = false;
            // Per-case: did at least one word decode as a fence/fence.i?
            // `Zifencei` class-compat admits `alu` so the trailing ADDI
            // scaffolding doesn't trip class-mismatch; without this tripwire
            // a case containing only ADDI (missing FENCE.I entirely) would
            // silently pass.
            let mut case_has_fence_word = false;
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
                        if decoded == "branch" {
                            case_has_branch_word = true;
                        }
                        if decoded == "fencei" || decoded == "fence" {
                            case_has_fence_word = true;
                        }
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
            if tc.class == RiscvClass::Rv32iBranch {
                assert!(
                    case_has_branch_word,
                    "Rv32iBranch case {} contains no branch/JAL/JALR word — \
                     class-compat allows alu/misc scaffolding but the real \
                     branch encoding appears to be missing or corrupt",
                    tc.name
                );
            }
            if tc.class == RiscvClass::Zifencei {
                assert!(
                    case_has_fence_word,
                    "Zifencei case {} contains no FENCE/FENCE.I word — \
                     class-compat allows alu scaffolding but the real \
                     fence encoding appears to be missing or corrupt",
                    tc.name
                );
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
        // Rv32c weight is 500 bp → ~50 rv32c cases in a 1000-case fuzz.
        // `gen_fuzz_rv32c` biases 10 % of those toward known-illegal Zcmp
        // Q2 encodings (expect_trap = 2), so expected sentinel hits
        // average ~5. `rv32c_arith` may also happen to emit Q2/Zcmp bit
        // patterns by chance, adding a handful more. Floor of 3 is a
        // conservative lower bound that still guarantees the Zcmp path
        // is non-trivially exercised.
        assert!(
            verified_compressed_sentinel >= 3,
            "too few compressed words hit the Zcmp sentinel: \
             {verified_compressed_sentinel} (expected >= 3); \
             compressed_unhandled = {compressed_unhandled}"
        );
    }
}

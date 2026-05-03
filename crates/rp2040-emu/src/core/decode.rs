//! ARMv6-M Thumb-16 decoder (top-level dispatch).
//!
//! Phase 4.A covers every Thumb-16 encoding ARMv6-M supports. The five
//! M0+ Thumb-32 encodings (BL, MRS, MSR, DSB, DMB, ISB) have prefix
//! `0b11110` — detected here by [`is_wide`] and routed to a Phase 4.B
//! `execute_thumb32` stub.
//!
//! Structural differences vs. the rp2350_emu (M33) decoder:
//!
//! - No IT block state.
//! - No CBZ/CBNZ (M33-only encoding; see `thumb16_misc`).
//! - `is_wide` accepts exactly one Thumb-32 prefix (`0b11110`); the
//!   other two M33 wide prefixes (`0b11101`, `0b11111`) decode as
//!   undefined on M0+.
//!
//! Decode-cache integration: `decode_execute` consults a per-core,
//! direct-mapped cache keyed by the full PC. On a hit it skips the
//! `bus.read16(pc)` (and the second halfword fetch for wide
//! encodings) plus the `is_wide` branch, dispatching directly into the
//! Thumb-16 / Thumb-32 executor. Modelled on the rp2350_emu cache
//! (commit `0c31479`) but trimmed for ARMv6-M (no IT-block flag, no
//! fetch-wait replay — RP2040's bus does not feed wait states into the
//! core's cycle accumulator).

use super::{CoreBus, CortexM0Plus};
use crate::bus::{DECODE_CACHE_SIZE, DecodedOp, is_cacheable_pc};

/// Direct-mapped index mask for the decode cache. Kept local to avoid
/// crossing `pub(crate)` visibility boundaries for a one-liner.
const CACHE_INDEX_MASK: u32 = (DECODE_CACHE_SIZE as u32) - 1;

/// Returns true iff the first halfword is the Thumb-32 prefix defined
/// for ARMv6-M (`0b11110xxx xxxxxxxx`). M0+ supports exactly one wide
/// prefix — unlike M33 which also accepts `0b11101` and `0b11111`.
#[inline(always)]
pub(crate) fn is_wide(hw0: u16) -> bool {
    (hw0 >> 11) == 0b11110
}

/// Conservative purity classifier. Returns `true` only for
/// instructions whose handler does not touch the bus and cannot raise
/// a synchronous fault — i.e. pure-ALU on registers, MOV-imm, hints,
/// barriers, and BL / B / B.cond.
///
/// Decode oracle for the `classifier_tests` module below (V3 Stream A,
/// commit `dabf3b3`; see `wrk_journals/2026.04.30 - JRN - Mutation
/// Testing V3 Execution.md` Stage 1 for the methodology). Not consumed
/// by the production decoder on M0+: the iter7 fast-path skip was
/// attempted and reverted within noise (commit `2621534`); see
/// `wrk_docs/2026.04.30 - HLD - RP2040 FLAG_PURE Consumer Removal
/// V1.md` for context. Kept in tree under `#[cfg(test)]` so Stream A's
/// 28 test functions continue to land against an oracle.
///
/// Conservative-by-default: a false negative just means the slow path
/// would run (no harm); a false positive would silently change cycle
/// accounting, so anything that might touch the bus is classified
/// impure.
#[cfg(test)]
fn classify_is_pure(hw0: u16, hw1: u16, wide: bool) -> bool {
    if !wide {
        classify_thumb16_pure(hw0)
    } else {
        classify_thumb32_pure(hw0, hw1)
    }
}

#[cfg(test)]
fn classify_thumb16_pure(opcode: u16) -> bool {
    match opcode >> 11 {
        // Shifts / add/sub / mov-cmp-add-sub imm — pure ALU.
        0b00000..=0b00011 => true,
        0b00100..=0b00111 => true,
        // Data processing (bit10=0) is pure; special-data / BX (bit10=1)
        // is impure (BX/BLX may dispatch exception return).
        0b01000 => opcode & (1 << 10) == 0,
        // Loads / stores — impure.
        0b01001 => false,
        0b01010 | 0b01011 => false,
        0b01100..=0b10001 => false,
        0b10010 | 0b10011 => false,
        // ADR / ADD SP imm — pure.
        0b10100 | 0b10101 => true,
        // Misc — fan out.
        0b10110 | 0b10111 => classify_thumb16_misc_pure(opcode),
        // STM / LDM — impure.
        0b11000 | 0b11001 => false,
        // B.cond / SVC / UDF — B.cond pure, SVC / UDF impure.
        0b11010 | 0b11011 => {
            let cond = (opcode >> 8) & 0xF;
            cond < 0xE
        }
        // Unconditional B — pure.
        0b11100 => true,
        _ => false,
    }
}

#[cfg(test)]
fn classify_thumb16_misc_pure(opcode: u16) -> bool {
    let op = (opcode >> 8) & 0xF;
    match op {
        // ADD/SUB SP imm7 — pure.
        0b0000 => true,
        // SXT / UXT — pure.
        0b0010 => true,
        // PUSH — impure (burst writes).
        0b0100 | 0b0101 => false,
        // CPSIE / CPSID — pure (PRIMASK only on M0+).
        0b0110 => true,
        // REV / REV16 / REVSH — pure.
        0b1010 => true,
        // POP — impure (burst reads, PC-pop may dispatch exception
        // return).
        0b1100 | 0b1101 => false,
        // BKPT — sets pending_fault, classified impure.
        0b1110 => false,
        // Hints (NOP / YIELD / WFE / WFI / SEV) — pure.
        0b1111 => true,
        // Other misc encodings — conservative impure.
        _ => false,
    }
}

#[cfg(test)]
fn classify_thumb32_pure(hw0: u16, hw1: u16) -> bool {
    // BL — pure (writes LR + PC only).
    if (hw1 & 0xD000) == 0xD000 {
        return true;
    }
    // Misc-control: barriers (DSB/DMB/ISB) and MRS/MSR are pure (ISB
    // touches the cache via invalidate_decode_cache_all, which is not
    // a bus access — the per-core cache is core-local state, not bus
    // state). Unrecognised encodings raise pending_fault and are
    // therefore impure.
    if (hw1 & 0xD000) == 0x8000 {
        if hw0 == 0xF3BF && (hw1 & 0xFF00) == 0x8F00 {
            let barrier_op = (hw1 >> 4) & 0xF;
            return matches!(barrier_op, 0x4..=0x6);
        }
        let op_field = (hw0 >> 4) & 0x7F;
        if (op_field == 0b0111000 || op_field == 0b0111001) && (hw1 & 0xFF00) == 0x8800 {
            return true; // MSR
        }
        if (op_field == 0b0111110 || op_field == 0b0111111)
            && (hw0 & 0xF) == 0xF
            && (hw1 & 0xF000) == 0x8000
        {
            return true; // MRS
        }
    }
    false
}

impl CortexM0Plus {
    /// Fetch-decode-execute one instruction. Returns cycle count.
    ///
    /// Fast path: a PC-keyed cache hit skips `bus.read16` + the wide
    /// test + the second halfword fetch on wide encodings, dispatching
    /// straight into the Thumb-16 / Thumb-32 executor.
    ///
    /// Slow path (cache miss): runs the standard fetch + decode and
    /// populates the slot for next time. Identical cycle semantics to
    /// the pre-cache implementation.
    pub(crate) fn decode_execute<B: CoreBus>(&mut self, bus: &mut B) -> u32 {
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        // Publish the instruction PC on the bus so the MMIO trace
        // (HLD V7 §4.3) can report it for every access this instruction
        // performs. Set before the fetch so the I-fetch itself is tagged
        // with its own PC.
        bus.set_active_pc(pc);

        // Cache lookup — `DecodedOp: Copy`, so no borrow on `bus`
        // survives into dispatch.
        let entry = if is_cacheable_pc(pc) {
            let slot = ((pc >> 1) & CACHE_INDEX_MASK) as usize;
            let e = self.decode_cache[slot];
            if e.tag == pc { Some(e) } else { None }
        } else {
            None
        };

        let entry = match entry {
            Some(e) => e,
            None => self.populate_decode_cache(bus, pc),
        };

        let hw0 = entry.hw0;
        let hw1 = entry.hw1;

        if entry.is_wide() {
            self.regs.set_pc(pc.wrapping_add(4));
            self.execute_thumb32(hw0, hw1, bus)
        } else {
            self.regs.set_pc(pc.wrapping_add(2));
            self.execute_thumb16(hw0, bus)
        }
    }

    /// Populate path — runs on a cache miss. Fetches `hw0` (and `hw1`
    /// for wide instructions) via the bus and writes the slot. Returns
    /// a [`DecodedOp`] for the caller to dispatch immediately.
    ///
    /// Faulty fetches are NOT cached: the slot is left untouched, the
    /// returned entry still carries the fetched halfwords so the
    /// caller's dispatch path can drive the existing fault delivery
    /// (`step` checks `bus.bus_fault()` after `decode_execute` returns).
    #[cold]
    #[inline(never)]
    fn populate_decode_cache<B: CoreBus>(&mut self, bus: &mut B, pc: u32) -> DecodedOp {
        let hw0 = bus.read16(pc);
        if bus.bus_fault() {
            // Fetch fault — return a non-cacheable sentinel entry so
            // the caller can dispatch and the post-step fault delivery
            // runs.
            return DecodedOp {
                tag: u32::MAX,
                hw0,
                hw1: 0,
                flags: 0,
            };
        }

        let wide = is_wide(hw0);
        let hw1 = if wide {
            bus.read16(pc.wrapping_add(2))
        } else {
            0
        };
        if wide && bus.bus_fault() {
            return DecodedOp {
                tag: u32::MAX,
                hw0,
                hw1,
                flags: DecodedOp::FLAG_WIDE,
            };
        }

        let mut flags = 0u8;
        if wide {
            flags |= DecodedOp::FLAG_WIDE;
        }

        let entry = DecodedOp {
            tag: pc,
            hw0,
            hw1,
            flags,
        };

        if is_cacheable_pc(pc) {
            let slot = ((pc >> 1) & CACHE_INDEX_MASK) as usize;
            self.decode_cache[slot] = entry;
        }

        entry
    }

    /// Top-level Thumb-16 dispatch. Routes to instruction-group handlers
    /// in execute.rs based on bits [15:11].
    pub(crate) fn execute_thumb16<B: CoreBus>(&mut self, opcode: u16, bus: &mut B) -> u32 {
        match opcode >> 11 {
            // Shift (immediate)
            0b00000 => self.thumb16_lsl_imm(opcode),
            0b00001 => self.thumb16_lsr_imm(opcode),
            0b00010 => self.thumb16_asr_imm(opcode),
            // Add/sub register and 3-bit immediate
            0b00011 => self.thumb16_add_sub(opcode),
            // Move/compare/add/sub 8-bit immediate
            0b00100 => self.thumb16_mov_imm(opcode),
            0b00101 => self.thumb16_cmp_imm(opcode),
            0b00110 => self.thumb16_add_imm8(opcode),
            0b00111 => self.thumb16_sub_imm8(opcode),
            // Data processing + special data / BX / BLX
            // bits[15:10] = 010000 → data processing
            // bits[15:10] = 010001 → special data / BX / BLX
            0b01000 => {
                if opcode & (1 << 10) == 0 {
                    self.thumb16_data_processing(opcode)
                } else {
                    self.thumb16_special_data_bx(opcode, bus)
                }
            }
            0b01001 => self.thumb16_ldr_literal(opcode, bus),
            // Load/store register offset
            0b01010 | 0b01011 => self.thumb16_load_store_reg(opcode, bus),
            // Load/store word immediate offset
            0b01100 => self.thumb16_str_imm(opcode, bus),
            0b01101 => self.thumb16_ldr_imm(opcode, bus),
            // Load/store byte immediate offset
            0b01110 => self.thumb16_strb_imm(opcode, bus),
            0b01111 => self.thumb16_ldrb_imm(opcode, bus),
            // Load/store halfword immediate offset
            0b10000 => self.thumb16_strh_imm(opcode, bus),
            0b10001 => self.thumb16_ldrh_imm(opcode, bus),
            // SP-relative load/store
            0b10010 => self.thumb16_str_sp(opcode, bus),
            0b10011 => self.thumb16_ldr_sp(opcode, bus),
            // ADR (PC-relative) and ADD SP+imm
            0b10100 => self.thumb16_adr(opcode),
            0b10101 => self.thumb16_add_sp_imm(opcode),
            // Miscellaneous (PUSH/POP/hints/SXT/UXT/REV/BKPT/SUB SP)
            0b10110 | 0b10111 => self.thumb16_misc(opcode, bus),
            // Store/Load multiple
            0b11000 => self.thumb16_stm(opcode, bus),
            0b11001 => self.thumb16_ldm(opcode, bus),
            // Conditional branch + SVC
            0b11010 | 0b11011 => self.thumb16_cond_branch_svc(opcode),
            // Unconditional branch
            0b11100 => self.thumb16_branch(opcode),
            // Prefix 0b11101 / 0b11110 / 0b11111 are 32-bit on the M33
            // but only 0b11110 is defined for M0+. Any encoding we reach
            // here via the Thumb-16 path is undefined.
            _ => self.thumb16_undefined(opcode),
        }
    }
}

#[cfg(test)]
mod classifier_tests {
    //! Direct tests for the purity classifier helpers in this module.
    //!
    //! Strategy: each classifier is a pure function of opcode bits. We
    //! assert (a) per-match-arm structural cases with named encodings,
    //! and (b) an FNV-1a fingerprint over the entire 16-bit input space
    //! (Thumb-16) or a representative cross-product (Thumb-32). The
    //! fingerprint catches any mutation that changes even a single
    //! input → output mapping; the structural cases give human-readable
    //! diagnostics on failure and document the intended behaviour.
    //!
    //! Classifier role on rp2040_emu: the four classifier functions exist
    //! solely as decode oracles for these tests; the production decoder
    //! does not call them. The fast-path skip was attempted and reverted
    //! within noise (commit `2621534`) and the populate-side scaffolding
    //! was removed alongside this comment edit; see `wrk_docs/2026.04.30
    //! - HLD - RP2040 FLAG_PURE Consumer Removal V1.md` for context.
    //!
    //! NOTE: the fingerprint is "current behaviour, not architectural
    //! truth". If a real classifier bug is fixed, update the asserted
    //! constant after manually reviewing the diff.
    use super::{
        classify_is_pure, classify_thumb16_misc_pure, classify_thumb16_pure, classify_thumb32_pure,
        is_wide,
    };

    /// FNV-1a 64-bit hash of the byte sequence. Deterministic across
    /// Rust versions (unlike `std::collections::hash_map::DefaultHasher`).
    fn fnv1a64(bytes: &[u8]) -> u64 {
        let mut hash: u64 = 0xcbf29ce484222325;
        for &b in bytes {
            hash ^= b as u64;
            hash = hash.wrapping_mul(0x100000001b3);
        }
        hash
    }

    fn pack_bool_bits(values: impl IntoIterator<Item = bool>) -> Vec<u8> {
        let mut bytes = Vec::new();
        let mut byte = 0u8;
        let mut bit = 0u32;
        for v in values {
            if v {
                byte |= 1 << bit;
            }
            bit += 1;
            if bit == 8 {
                bytes.push(byte);
                byte = 0;
                bit = 0;
            }
        }
        if bit != 0 {
            bytes.push(byte);
        }
        bytes
    }

    // ---------- classify_thumb16_pure: per-prefix structural ----------

    /// Build a Thumb-16 opcode with prefix `p` (5 bits) and a body of zeros.
    fn t16(prefix: u16) -> u16 {
        prefix << 11
    }

    #[test]
    fn t16_shift_immediate_is_pure() {
        // 00000 LSL imm, 00001 LSR imm, 00010 ASR imm, 00011 ADD/SUB
        assert!(classify_thumb16_pure(t16(0b00000)));
        assert!(classify_thumb16_pure(t16(0b00001)));
        assert!(classify_thumb16_pure(t16(0b00010)));
        assert!(classify_thumb16_pure(t16(0b00011)));
    }

    #[test]
    fn t16_imm8_data_processing_is_pure() {
        // 00100 MOV imm8, 00101 CMP imm8, 00110 ADD imm8, 00111 SUB imm8
        assert!(classify_thumb16_pure(t16(0b00100)));
        assert!(classify_thumb16_pure(t16(0b00101)));
        assert!(classify_thumb16_pure(t16(0b00110)));
        assert!(classify_thumb16_pure(t16(0b00111)));
    }

    #[test]
    fn t16_data_processing_pure_special_data_impure() {
        // 0b01000 with bit10=0 → DP register (pure)
        assert!(classify_thumb16_pure(0b01000_00000_000000));
        // 0b01000 with bit10=1 → special data / BX / BLX (impure)
        assert!(!classify_thumb16_pure(0b01000_10000_000000));
    }

    #[test]
    fn t16_loads_stores_are_impure() {
        // 0b01001 LDR literal
        assert!(!classify_thumb16_pure(t16(0b01001)));
        // 0b01010, 0b01011 LDR/STR register offset
        assert!(!classify_thumb16_pure(t16(0b01010)));
        assert!(!classify_thumb16_pure(t16(0b01011)));
        // 0b01100..=0b10001 LDR/STR immediate offset (six handlers)
        for prefix in 0b01100..=0b10001u16 {
            assert!(
                !classify_thumb16_pure(t16(prefix)),
                "prefix {:05b} should be impure (LDR/STR imm offset)",
                prefix
            );
        }
        // 0b10010, 0b10011 LDR/STR SP-relative
        assert!(!classify_thumb16_pure(t16(0b10010)));
        assert!(!classify_thumb16_pure(t16(0b10011)));
    }

    #[test]
    fn t16_adr_and_add_sp_imm_are_pure() {
        assert!(classify_thumb16_pure(t16(0b10100))); // ADR
        assert!(classify_thumb16_pure(t16(0b10101))); // ADD SP, imm
    }

    #[test]
    fn t16_misc_group_dispatches() {
        // Prefix 10110 / 10111 routes to classify_thumb16_misc_pure.
        // We assert here that the dispatch happens correctly by picking
        // a canonical pure misc op and a canonical impure misc op.
        // op[11:8] = 0b0000 → ADD/SUB SP imm7 (pure)
        #[allow(clippy::identity_op, clippy::erasing_op)]
        let pure_misc = (0b10110u16 << 11) | (0b0000 << 8);
        assert!(classify_thumb16_pure(pure_misc));
        // op[11:8] = 0b0100 → PUSH (impure)
        let impure_misc = (0b10110u16 << 11) | (0b0100 << 8);
        assert!(!classify_thumb16_pure(impure_misc));
    }

    #[test]
    fn t16_stm_ldm_are_impure() {
        assert!(!classify_thumb16_pure(t16(0b11000))); // STM
        assert!(!classify_thumb16_pure(t16(0b11001))); // LDM
    }

    #[test]
    fn t16_b_cond_pure_svc_udf_impure() {
        // Prefix 11010/11011, cond field bits[11:8]:
        //   0x0..=0xD → B.cond (pure)
        //   0xE → UDF (impure)
        //   0xF → SVC (impure)
        for cond in 0x0..=0xDu16 {
            let opc = (0b11010u16 << 11) | (cond << 8);
            assert!(
                classify_thumb16_pure(opc),
                "B.cond cond={cond:#x} should be pure"
            );
        }
        let udf = (0b11010u16 << 11) | (0xE << 8);
        let svc = (0b11010u16 << 11) | (0xF << 8);
        assert!(!classify_thumb16_pure(udf));
        assert!(!classify_thumb16_pure(svc));
    }

    #[test]
    fn t16_b_unconditional_is_pure() {
        assert!(classify_thumb16_pure(t16(0b11100)));
    }

    #[test]
    fn t16_thumb32_prefixes_classify_impure_via_thumb16_path() {
        // Prefixes 0b11101 / 0b11110 / 0b11111 should never reach
        // classify_thumb16_pure (is_wide catches 0b11110), but if they
        // do, the function returns false (impure).
        assert!(!classify_thumb16_pure(t16(0b11101)));
        assert!(!classify_thumb16_pure(t16(0b11110)));
        assert!(!classify_thumb16_pure(t16(0b11111)));
    }

    // ---------- classify_thumb16_misc_pure: per-op structural ----------

    /// Build a misc-group opcode with op[11:8] = `op`. Prefix is 1011_0.
    fn misc(op: u16) -> u16 {
        (0b10110u16 << 11) | (op << 8)
    }

    #[test]
    fn misc_add_sub_sp_imm7_is_pure() {
        assert!(classify_thumb16_misc_pure(misc(0b0000)));
    }

    #[test]
    fn misc_sxt_uxt_is_pure() {
        assert!(classify_thumb16_misc_pure(misc(0b0010)));
    }

    #[test]
    fn misc_push_is_impure() {
        assert!(!classify_thumb16_misc_pure(misc(0b0100)));
        assert!(!classify_thumb16_misc_pure(misc(0b0101)));
    }

    #[test]
    fn misc_cps_is_pure() {
        assert!(classify_thumb16_misc_pure(misc(0b0110)));
    }

    #[test]
    fn misc_rev_is_pure() {
        assert!(classify_thumb16_misc_pure(misc(0b1010)));
    }

    #[test]
    fn misc_pop_is_impure() {
        assert!(!classify_thumb16_misc_pure(misc(0b1100)));
        assert!(!classify_thumb16_misc_pure(misc(0b1101)));
    }

    #[test]
    fn misc_bkpt_is_impure() {
        // rp2040_emu classifies BKPT impure (sets pending_fault).
        assert!(!classify_thumb16_misc_pure(misc(0b1110)));
    }

    #[test]
    fn misc_hints_are_pure() {
        // op[11:8] == 0b1111 → hints (NOP / YIELD / WFE / WFI / SEV).
        assert!(classify_thumb16_misc_pure(misc(0b1111)));
    }

    #[test]
    fn misc_other_ops_are_impure_by_default() {
        // Conservative: any op not explicitly enumerated → impure.
        // rp2040_emu lacks CBZ/CBNZ (ARMv6-M). Verify all unenumerated
        // misc ops classify impure.
        for op in 0..=0xFu16 {
            let expected = matches!(op, 0b0000 | 0b0010 | 0b0110 | 0b1010 | 0b1111);
            assert_eq!(
                classify_thumb16_misc_pure(misc(op)),
                expected,
                "misc op[11:8]={op:#x} expected {expected}"
            );
        }
    }

    // ---------- classify_thumb32_pure: per-encoding structural ----------

    #[test]
    fn t32_bl_is_pure() {
        // BL: hw1 has top bits 110x (J1/J2 fixed at 1 for BL T1).
        // Match: (hw1 & 0xD000) == 0xD000.
        let hw0 = 0xF000; // BL T1: hw0[15:11]=11110, S/imm10 left zero.
        let hw1 = 0xD000; // 1101 0000 0000 0000 → BL.
        assert!(classify_thumb32_pure(hw0, hw1));
    }

    #[test]
    fn t32_misc_control_msr_is_pure() {
        // MSR: op_field = (hw0 >> 4) & 0x7F ∈ {0b0111000, 0b0111001}
        //      AND (hw1 & 0xFF00) == 0x8800 AND (hw1 & 0xD000) == 0x8000.
        let hw0 = 0b11110_0_111000_0000u16; // op_field bits set.
        let hw1 = 0b1_000_1_0_00_0000_0000u16; // (& 0xFF00) == 0x8800
        assert_eq!(hw1 & 0xFF00, 0x8800);
        assert_eq!(hw1 & 0xD000, 0x8000);
        assert!(classify_thumb32_pure(hw0, hw1));

        // Variant op_field = 0b0111001:
        let hw0 = 0b11110_0_111001_0000u16;
        assert!(classify_thumb32_pure(hw0, hw1));
    }

    #[test]
    fn t32_misc_control_mrs_is_pure() {
        // MRS: op_field ∈ {0b0111110, 0b0111111}, (hw0 & 0xF) == 0xF,
        //      (hw1 & 0xF000) == 0x8000, AND outer (hw1 & 0xD000) == 0x8000.
        let hw0 = 0b11110_0_111110_1111u16;
        let hw1 = 0x8000;
        assert_eq!(hw0 & 0xF, 0xF);
        assert_eq!(hw1 & 0xF000, 0x8000);
        assert_eq!(hw1 & 0xD000, 0x8000);
        assert!(classify_thumb32_pure(hw0, hw1));

        let hw0 = 0b11110_0_111111_1111u16;
        assert!(classify_thumb32_pure(hw0, hw1));
    }

    #[test]
    fn t32_misc_control_barriers_are_pure() {
        // hw0 == 0xF3BF, (hw1 & 0xFF00) == 0x8F00, barrier_op = (hw1 >> 4) & 0xF
        // valid for barrier_op in 0x4..=0x6 (DSB / DMB / ISB).
        let hw0 = 0xF3BF;
        for barrier_op in [0x4u16, 0x5, 0x6] {
            let hw1 = 0x8F00 | (barrier_op << 4);
            assert!(
                classify_thumb32_pure(hw0, hw1),
                "barrier_op {barrier_op:#x} should be pure"
            );
        }
        // barrier_op outside 0x4..=0x6 falls to thumb32_undefined → impure.
        let hw1 = 0x8F00 | (0x7 << 4);
        assert!(!classify_thumb32_pure(hw0, hw1));
    }

    #[test]
    fn t32_unrecognised_encoding_is_impure() {
        // hw1 outer match fails: (hw1 & 0xD000) is neither 0xD000 nor 0x8000.
        let hw0 = 0xF000;
        let hw1 = 0x0000;
        assert!(!classify_thumb32_pure(hw0, hw1));
    }

    // ---------- classify_is_pure: dispatcher ----------

    #[test]
    fn dispatcher_routes_thumb16() {
        // wide=false dispatches to classify_thumb16_pure(hw0).
        assert_eq!(
            classify_is_pure(t16(0b00000), 0, false),
            classify_thumb16_pure(t16(0b00000))
        );
        assert_eq!(
            classify_is_pure(t16(0b01001), 0, false),
            classify_thumb16_pure(t16(0b01001))
        );
    }

    #[test]
    fn dispatcher_routes_thumb32() {
        // wide=true dispatches to classify_thumb32_pure(hw0, hw1).
        let hw0 = 0xF000;
        let hw1 = 0xD000;
        assert_eq!(
            classify_is_pure(hw0, hw1, true),
            classify_thumb32_pure(hw0, hw1)
        );
    }

    // ---------- is_wide ----------

    #[test]
    fn is_wide_only_matches_m0plus_thumb32_prefix() {
        // M0+ accepts exactly one wide prefix: 0b11110.
        assert!(is_wide(0b11110_000_00000000));
        // 0b11101 and 0b11111 are wide on M33 but NOT on M0+.
        assert!(!is_wide(0b11101_000_00000000));
        assert!(!is_wide(0b11111_000_00000000));
        // Narrow prefixes never wide.
        assert!(!is_wide(0b00000_000_00000000));
        assert!(!is_wide(0b11100_000_00000000));
    }

    // ---------- exhaustive Thumb-16 fingerprint ----------
    //
    // Snapshot test: enumerate the entire 16-bit Thumb-16 input space,
    // compute classify_thumb16_pure for each, and assert the FNV-1a
    // fingerprint matches a checked-in constant. ANY mutation that
    // changes even one input → output mapping flips this hash.

    #[test]
    fn t16_full_space_fingerprint() {
        let bits = (0..=0xFFFFu16).map(classify_thumb16_pure);
        let packed = pack_bool_bits(bits);
        let h = fnv1a64(&packed);
        assert_eq!(
            h, T16_PURE_FINGERPRINT,
            "classify_thumb16_pure fingerprint changed (computed = {h:#018x})"
        );
    }

    #[test]
    fn t16_misc_full_space_fingerprint() {
        // The misc classifier only inspects opcode[11:8]; the other
        // bits don't matter. Enumerate the 16 sub-ops at canonical
        // misc encoding (prefix 1011_0).
        let bits = (0..=0xFu16).map(misc).map(classify_thumb16_misc_pure);
        let packed = pack_bool_bits(bits);
        let h = fnv1a64(&packed);
        assert_eq!(
            h, MISC_PURE_FINGERPRINT,
            "classify_thumb16_misc_pure fingerprint changed (computed = {h:#018x})"
        );
    }

    /// FNV-1a 64-bit hash of the bit-packed `classify_thumb16_pure`
    /// output over inputs `0..=0xFFFF`. Computed 2026-04-30 against
    /// the M0+ classifier as committed at this point in the V3 work.
    const T16_PURE_FINGERPRINT: u64 = 0xba34c96a2f0a7f45;
    /// FNV-1a 64-bit hash of `classify_thumb16_misc_pure` over the
    /// 16 misc sub-ops (canonical prefix 1011_0).
    const MISC_PURE_FINGERPRINT: u64 = 0x08fb8e07b596aaac;
}

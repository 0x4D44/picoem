//! ARMv6-M Thumb-16 decoder (top-level dispatch).
//!
//! Phase 4.A covers every Thumb-16 encoding ARMv6-M supports. The five
//! M0+ Thumb-32 encodings (BL, MRS, MSR, DSB, DMB, ISB) have prefix
//! `0b11110` — detected here by [`is_wide`] and routed to a Phase 4.B
//! `execute_thumb32` stub.
//!
//! Structural differences vs. the mdrp2350 (M33) decoder:
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
//! Thumb-16 / Thumb-32 executor. Modelled on the mdrp2350 cache
//! (commit `0c31479`) but trimmed for ARMv6-M (no IT-block flag, no
//! fetch-wait replay — RP2040's bus does not feed wait states into the
//! core's cycle accumulator).

use super::{CortexM0Plus, CoreBus};
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
/// Reserved for the iter7 fast-path skip; iter6 sets the flag at
/// populate time but does not act on it. Conservative-by-default: a
/// false negative just means the slow path runs (no harm); a false
/// positive would silently change cycle accounting, so anything that
/// might touch the bus is classified impure.
fn classify_is_pure(hw0: u16, hw1: u16, wide: bool) -> bool {
    if !wide {
        classify_thumb16_pure(hw0)
    } else {
        classify_thumb32_pure(hw0, hw1)
    }
}

fn classify_thumb16_pure(opcode: u16) -> bool {
    match opcode >> 11 {
        // Shifts / add/sub / mov-cmp-add-sub imm — pure ALU.
        0b00000 | 0b00001 | 0b00010 | 0b00011 => true,
        0b00100 | 0b00101 | 0b00110 | 0b00111 => true,
        // Data processing (bit10=0) is pure; special-data / BX (bit10=1)
        // is impure (BX/BLX may dispatch exception return).
        0b01000 => opcode & (1 << 10) == 0,
        // Loads / stores — impure.
        0b01001 => false,
        0b01010 | 0b01011 => false,
        0b01100 | 0b01101 | 0b01110 | 0b01111 | 0b10000 | 0b10001 => false,
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
            return matches!(barrier_op, 0x4 | 0x5 | 0x6);
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
    /// for wide instructions) via the bus, classifies purity, and
    /// writes the slot. Returns a [`DecodedOp`] for the caller to
    /// dispatch immediately.
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
        let hw1 = if wide { bus.read16(pc.wrapping_add(2)) } else { 0 };
        if wide && bus.bus_fault() {
            return DecodedOp {
                tag: u32::MAX,
                hw0,
                hw1,
                flags: DecodedOp::FLAG_WIDE,
            };
        }

        let pure = classify_is_pure(hw0, hw1, wide);
        let mut flags = 0u8;
        if wide { flags |= DecodedOp::FLAG_WIDE; }
        if pure { flags |= DecodedOp::FLAG_PURE; }

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

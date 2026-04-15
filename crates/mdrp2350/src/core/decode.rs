use crate::bus::{Bus, DecodedOp, DECODE_CACHE_SIZE, Handler, is_cacheable_pc};
use super::CortexM33;

// Direct-mapped index mask — kept local to avoid crossing `pub(crate)`
// visibility boundaries for a one-liner.
const CACHE_INDEX_MASK: u32 = (DECODE_CACHE_SIZE as u32) - 1;

/// Returns true if the first halfword indicates a 32-bit Thumb-2 instruction.
/// Bits [15:11] of 0b11101, 0b11110, or 0b11111 → 32-bit.
#[inline(always)]
fn is_wide(hw0: u16) -> bool {
    hw0 >= 0xE800
}

/// Returns true if a Thumb-16 opcode is a flag-only instruction (CMP, CMN, TST).
/// These always set flags, even inside IT blocks.
fn is_thumb16_flag_only(opcode: u16) -> bool {
    match opcode >> 11 {
        0b00101 => true, // CMP Rn, #imm8
        0b01000 => {
            if opcode & (1 << 10) == 0 {
                // Data processing: TST (0x8), CMP (0xA), CMN (0xB)
                let dp_op = (opcode >> 6) & 0xF;
                matches!(dp_op, 0x8 | 0xA | 0xB)
            } else {
                // Special data: CMP Rn, Rm (high register)
                ((opcode >> 8) & 0x3) == 0b01
            }
        }
        _ => false,
    }
}

/// Classify a decoded Thumb instruction as pure (no bus wait-state
/// accumulation, no synchronous fault). Source of truth per HLD
/// `2026.04.14 - HLD - Cycle Accounting Short-Circuit.md` §1. The
/// classification is static — it depends only on the bytes at PC, not
/// on runtime state — so the result is valid for the lifetime of a
/// cache entry.
///
/// Pure ⇒ the fast path may skip `bus.reset_extra_wait_states()` and
/// `bus.extra_wait_states()` when dispatching this op.
///
/// Undefined-encoding subtlety: a handler classified pure that falls
/// through to `thumb16_undefined` / `thumb32_undefined` will raise a
/// synchronous fault (via `pending_fault`). This does NOT break cycle
/// accuracy: on fault, `CortexM33::step` discards `decode_execute`'s
/// return value and uses `deliver_fault`'s cycle count instead. So the
/// pure path's "no wait-state accumulation" contract is satisfied in
/// practice — any stacking done by fault delivery is accounted separately.
/// The HLD §1 rule is strictly stricter than correctness requires.
pub(crate) fn classify_is_pure(hw0: u16, hw1: u16, is_wide: bool) -> bool {
    if !is_wide {
        classify_thumb16_pure(hw0)
    } else {
        classify_thumb32_pure(hw0, hw1)
    }
}

/// Pure-classification for Thumb-16. Row numbers refer to the table in
/// HLD B §1 ("Thumb-16 classification").
fn classify_thumb16_pure(opcode: u16) -> bool {
    match opcode >> 11 {
        // 00000 LSL imm / 00001 LSR imm / 00010 ASR imm — pure ALU, no bus.
        0b00000 | 0b00001 | 0b00010 => true,
        // 00011 ADD/SUB reg / imm3 — pure.
        0b00011 => true,
        // 00100..00111 MOV/CMP/ADD/SUB imm8 — pure.
        0b00100 | 0b00101 | 0b00110 | 0b00111 => true,
        // 01000 — bit 10 discriminates data-processing (pure) from
        // special-data/BX (impure; BX/BLX/MOV-PC may hit exit_exception).
        0b01000 => opcode & (1 << 10) == 0,
        // 01001 LDR literal — impure (bus.read32).
        0b01001 => false,
        // 01010 / 01011 LDR/STR register offset — impure.
        0b01010 | 0b01011 => false,
        // 01100..10001 LDR/STR immediate offset (six handlers) — impure.
        0b01100 | 0b01101 | 0b01110 | 0b01111 | 0b10000 | 0b10001 => false,
        // 10010 / 10011 LDR/STR SP-relative — impure.
        0b10010 | 0b10011 => false,
        // 10100 ADR — pure.
        0b10100 => true,
        // 10101 ADD SP, imm — pure.
        0b10101 => true,
        // 10110 / 10111 misc — fan out by opcode[11:8].
        0b10110 | 0b10111 => classify_thumb16_misc_pure(opcode),
        // 11000 STM / 11001 LDM — impure (burst writes / reads).
        0b11000 | 0b11001 => false,
        // 11010 / 11011 B.cond / SVC / UDF — mixed: B.cond pure,
        // SVC / UDF impure (enter_exception / fault).
        0b11010 | 0b11011 => {
            let cond = (opcode >> 8) & 0xF;
            // cond == 0xE is UDF, cond == 0xF is SVC — both impure.
            cond < 0xE
        }
        // 11100 B — pure.
        0b11100 => true,
        // 11101+ — should not occur (is_wide would have matched);
        // treat as impure (undefined → fault).
        _ => false,
    }
}

/// Pure-classification for the Thumb-16 misc group (opcode[15:12] == 1011).
/// See HLD B §1 "Misc group".
fn classify_thumb16_misc_pure(opcode: u16) -> bool {
    let op = (opcode >> 8) & 0xF;
    match op {
        // 0000 ADD/SUB SP imm7 — pure.
        0b0000 => true,
        // 0010 SXTH / SXTB / UXTH / UXTB — pure (register only).
        0b0010 => true,
        // 0100 / 0101 PUSH — impure (burst-mode writes).
        0b0100 | 0b0101 => false,
        // 0110 CPSIE / CPSID — pure (PRIMASK/FAULTMASK).
        0b0110 => true,
        // 1010 REV / REV16 / REVSH — pure (register only).
        0b1010 => true,
        // 1100 / 1101 POP — impure (burst reads; PC-pop can hit
        // exit_exception, which we treat as bus-touching).
        0b1100 | 0b1101 => false,
        // 1110 BKPT — NOP stub, pure.
        0b1110 => true,
        // 1111 IT / hints (NOP / YIELD / WFE / WFI / SEV) — pure per HLD B
        // (hints touch direct fields, not bus wait-state accumulator).
        0b1111 => true,
        // CBZ / CBNZ match x0x1 (mask 0x5 == 0x1) — pure (PC write only).
        op if op & 0x5 == 0x1 => true,
        // Other misc encodings — currently NOP stubs, pure. Any future
        // impure sub-op added here must update this arm.
        _ => true,
    }
}

/// Pure-classification for Thumb-32. See HLD B §1 "Thumb-32 classification".
/// Uses the same decoder topology as `execute_thumb32` so every dispatch
/// target has a deterministic classification.
fn classify_thumb32_pure(hw0: u16, hw1: u16) -> bool {
    let op1 = (hw0 >> 11) & 0x3;
    let op2 = ((hw0 >> 4) & 0x7F) as u32;

    match op1 {
        0b01 => match op2 >> 5 {
            // ldm/stm / load_store_dual — impure.
            0b00 => false,
            // dp_shifted_reg — pure.
            0b01 => true,
            // coprocessor — blanket impure (HLD B §1 "Coprocessor and FPU").
            _ => false,
        },
        0b10 => {
            let op = (hw1 >> 15) & 0x1;
            if op == 0 {
                // dp_modified_imm / dp_plain_imm — pure.
                true
            } else {
                // branch_misc — sub-decode (BL, B.W, misc-control).
                classify_thumb32_branch_misc_pure(hw0, hw1)
            }
        }
        0b11 => {
            if op2 & 0x40 != 0 {
                // coprocessor — impure.
                false
            } else if op2 & 0x20 == 0 {
                // load_store_single — impure.
                false
            } else if op2 & 0x10 == 0 {
                // dp_register — pure.
                true
            } else if op2 & 0x08 == 0 {
                // multiply — pure.
                true
            } else {
                // long_multiply — pure.
                true
            }
        }
        // op1 == 0 is a narrow-prefix branch; reaching here via the wide
        // path means the decoder is handing us something malformed. The
        // actual execute path routes to `thumb32_undefined` (impure).
        _ => false,
    }
}

/// Pure-classification for the `thumb32_branch_misc` fan-out, mirroring
/// the sub-dispatch in `execute_thumb32.rs`. BL / B.W (both directions) /
/// MSR / MRS / hints / barriers are pure. Undefined is impure.
fn classify_thumb32_branch_misc_pure(hw0: u16, hw1: u16) -> bool {
    if hw1 & (1 << 14) != 0 {
        // BL — pure (register writes LR, PC).
        true
    } else if hw1 & (1 << 12) != 0 {
        // B.W T4 (unconditional) — pure.
        true
    } else {
        let misc_op = (hw0 >> 6) & 0xF;
        if misc_op & 0xE != 0xE {
            // B.W T3 (conditional) — pure.
            true
        } else {
            // misc control — hints, barriers, MSR, MRS all pure; anything
            // else is `thumb32_undefined` — impure.
            classify_thumb32_misc_control_pure(hw0, hw1)
        }
    }
}

/// Pure-classification for the misc-control sub-group of `thumb32_branch_misc`.
fn classify_thumb32_misc_control_pure(hw0: u16, hw1: u16) -> bool {
    // Hints (hw0 == 0xF3AF): NOP.W / YIELD.W / WFE.W / WFI.W / SEV.W all pure.
    // Any unrecognised hint falls into `thumb32_undefined` — impure.
    if hw0 == 0xF3AF {
        let hint = hw1 & 0xFF;
        return matches!(hint, 0x00 | 0x01 | 0x02 | 0x03 | 0x04);
    }
    // Barriers (hw0 == 0xF3BF): CLREX / DSB / DMB / ISB all pure; others
    // fall into `thumb32_undefined`.
    if hw0 == 0xF3BF {
        let barrier_op = (hw1 >> 4) & 0xF;
        return matches!(barrier_op, 0x2 | 0x4 | 0x5 | 0x6);
    }
    let op_field = (hw0 >> 4) & 0x7F;
    // MSR / MRS — register-file only, pure.
    if op_field == 0b0111000 || op_field == 0b0111001
        || op_field == 0b0111110 || op_field == 0b0111111 {
        return true;
    }
    // Otherwise falls into `thumb32_undefined` — impure.
    false
}

/// Map `(hw0, hw1, is_wide)` to the top-level dispatched handler.
///
/// **Mirror of `CortexM33::execute_thumb16` (`execute_thumb16` below) and
/// `CortexM33::execute_thumb32` (in `execute_thumb32.rs` via this module).**
/// Any edit to the live dispatcher trees MUST land here too, or cache-
/// populate will stamp the wrong handler. The `dispatch_equiv` test module
/// in `crates/mdrp2350/src/tests.rs` backstops this — it covers every
/// top-level arm (including every sub-arm) from HLD §2.4.
///
/// See HLD `2026.04.15 - HLD - Fn-Pointer Dispatch in DecodedOp.md` §2.4.
pub(crate) fn classify_handler(hw0: u16, hw1: u16, is_wide: bool) -> Handler {
    if !is_wide {
        return match hw0 >> 11 {
            0b00000 => CortexM33::thumb16_lsl_imm,
            0b00001 => CortexM33::thumb16_lsr_imm,
            0b00010 => CortexM33::thumb16_asr_imm,
            0b00011 => CortexM33::thumb16_add_sub,
            0b00100 => CortexM33::thumb16_mov_imm,
            0b00101 => CortexM33::thumb16_cmp_imm,
            0b00110 => CortexM33::thumb16_add_imm8,
            0b00111 => CortexM33::thumb16_sub_imm8,
            0b01000 => {
                if hw0 & (1 << 10) == 0 {
                    CortexM33::thumb16_data_processing
                } else {
                    CortexM33::thumb16_special_data_bx
                }
            }
            0b01001 => CortexM33::thumb16_ldr_literal,
            0b01010 | 0b01011 => CortexM33::thumb16_load_store_reg,
            0b01100 => CortexM33::thumb16_str_imm,
            0b01101 => CortexM33::thumb16_ldr_imm,
            0b01110 => CortexM33::thumb16_strb_imm,
            0b01111 => CortexM33::thumb16_ldrb_imm,
            0b10000 => CortexM33::thumb16_strh_imm,
            0b10001 => CortexM33::thumb16_ldrh_imm,
            0b10010 => CortexM33::thumb16_str_sp,
            0b10011 => CortexM33::thumb16_ldr_sp,
            0b10100 => CortexM33::thumb16_adr,
            0b10101 => CortexM33::thumb16_add_sp_imm,
            0b10110 | 0b10111 => CortexM33::thumb16_misc,
            0b11000 => CortexM33::thumb16_stm,
            0b11001 => CortexM33::thumb16_ldm,
            0b11010 | 0b11011 => CortexM33::thumb16_cond_branch_svc,
            0b11100 => CortexM33::thumb16_branch,
            _ => CortexM33::thumb16_undefined,
        };
    }

    let op1 = (hw0 >> 11) & 0x3;
    let op2 = ((hw0 >> 4) & 0x7F) as u32;
    let op = (hw1 >> 15) & 0x1;

    match op1 {
        0b01 => match op2 >> 5 {
            0b00 => {
                if op2 & 0x04 == 0 {
                    CortexM33::thumb32_ldm_stm
                } else {
                    CortexM33::thumb32_load_store_dual
                }
            }
            0b01 => CortexM33::thumb32_dp_shifted_reg,
            _ => CortexM33::thumb32_coprocessor,
        },
        0b10 => {
            if op == 0 {
                if op2 & 0x20 == 0 {
                    CortexM33::thumb32_dp_modified_imm
                } else {
                    CortexM33::thumb32_dp_plain_imm
                }
            } else {
                CortexM33::thumb32_branch_misc
            }
        }
        0b11 => {
            if op2 & 0x40 != 0 {
                CortexM33::thumb32_coprocessor
            } else if op2 & 0x20 == 0 {
                CortexM33::thumb32_load_store_single
            } else if op2 & 0x10 == 0 {
                CortexM33::thumb32_dp_register
            } else if op2 & 0x08 == 0 {
                CortexM33::thumb32_multiply
            } else {
                CortexM33::thumb32_long_multiply
            }
        }
        // op1 == 0 is unreachable: `is_wide` demands `hw0 >= 0xE800`, so
        // `(hw0 >> 11) & 3` ∈ {0b01, 0b10, 0b11}. The `_` arm mirrors the
        // unreachable fall-through of the live dispatcher for completeness.
        _ => CortexM33::thumb32_undefined,
    }
}

impl CortexM33 {
    /// Fetch, decode, and execute one instruction. Returns cycle count.
    ///
    /// Fast path: a PC-keyed cache hit skips `bus.read16` + the wide
    /// test + the top-level dispatch match. For ops classified as pure
    /// (HLD B §1) it also skips `bus.reset_extra_wait_states()` and the
    /// final `bus.extra_wait_states()` read — the fetch contribution is
    /// replayed from `entry.fetch_wait` instead.
    ///
    /// Slow path (cache miss, or hit on an impure op): identical cycle
    /// semantics to pre-cache behaviour.
    ///
    /// Dispatch goes through the cached `entry.handler` fn pointer stamped
    /// by `classify_handler` at populate time — a single indirect call
    /// replaces both `execute_thumb16` and `execute_thumb32`'s top-level
    /// match. `is_wide` is still consulted for the PC increment (+2 vs +4).
    /// See HLD `2026.04.15 - HLD - Fn-Pointer Dispatch in DecodedOp.md` §2.5.
    pub(crate) fn decode_execute(&mut self, bus: &mut Bus) -> u32 {
        let pc = self.regs.pc();
        self.current_instr_addr = pc;

        // Cache lookup — by-value (`DecodedOp: Copy`), so no borrow on
        // `bus` survives into dispatch.
        let entry = if is_cacheable_pc(pc) {
            let slot = ((pc >> 1) & CACHE_INDEX_MASK) as usize;
            let e = bus.decode_cache[slot];
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
        let is_wide = entry.is_wide();
        let is_pure = entry.is_pure();
        let flag_only = entry.is_flag_only();

        // IT block state — identical to pre-cache behaviour.
        let in_it = self.it_state & 0xF != 0;
        let cond = if in_it {
            (self.it_state >> 4) & 0xF
        } else {
            0xE // AL (always)
        };
        let cond_passed = self.regs.condition_passed(cond);

        if is_pure {
            // Fast path: neither the fetch (handled by the cache) nor
            // the handler touches `bus.extra_wait_states`. The
            // debug-assert below catches any misclassification.
            #[cfg(debug_assertions)]
            let ws_before = bus.extra_wait_states();

            let pc_inc = if is_wide { 4 } else { 2 };
            self.regs.set_pc(pc.wrapping_add(pc_inc));
            let saved_flags = if in_it && !is_wide {
                self.regs.xpsr & 0xF800_0000
            } else {
                0
            };
            let cycles = if cond_passed {
                (entry.handler)(self, hw0, hw1, bus)
            } else {
                1
            };
            if in_it && !is_wide && cond_passed && !flag_only {
                self.regs.xpsr = (self.regs.xpsr & !0xF800_0000) | saved_flags;
            }
            if in_it { self.advance_it_state(); }

            #[cfg(debug_assertions)]
            debug_assert_eq!(
                bus.extra_wait_states(),
                ws_before,
                "pure op at PC={:08X} (hw0={:04X}, hw1={:04X}) \
                 dirtied bus.extra_wait_states",
                pc, hw0, hw1,
            );

            // Fetch wait states are baked into the entry; no accumulator
            // touch, just a direct add.
            cycles + entry.fetch_wait as u32
        } else {
            // Slow path — preserves existing semantics verbatim.
            bus.reset_extra_wait_states();
            // Fetch contribution for this impure op must be accounted
            // for the same way today's code does: add it into the
            // accumulator before dispatch, so the final `+extra_wait_states()`
            // folds it in exactly as the non-cached path would.
            bus.add_extra_wait_states(entry.fetch_wait as u32);

            let pc_inc = if is_wide { 4 } else { 2 };
            self.regs.set_pc(pc.wrapping_add(pc_inc));
            let saved_flags = if in_it && !is_wide {
                self.regs.xpsr & 0xF800_0000
            } else {
                0
            };
            let cycles = if cond_passed {
                (entry.handler)(self, hw0, hw1, bus)
            } else {
                1
            };
            if in_it && !is_wide && cond_passed && !flag_only {
                self.regs.xpsr = (self.regs.xpsr & !0xF800_0000) | saved_flags;
            }
            if in_it { self.advance_it_state(); }
            cycles + bus.extra_wait_states()
        }
    }

    /// Populate path — runs on a cache miss. Fetches `hw0` (and `hw1`
    /// for wide instructions) via the bus, classifies purity, stamps
    /// the dispatched handler, and writes the slot. Returns a
    /// `DecodedOp` for the caller to dispatch immediately.
    ///
    /// Faulty fetches are NOT cached (see HLD §8.1): the slot is left
    /// untouched, the returned entry still carries the fetched
    /// halfwords so `decode_execute` can drive the existing fault
    /// delivery path (which checks `bus.bus_fault()` after the
    /// `decode_execute` call returns). The fault-fetch sentinel gets
    /// a width-matched undefined handler — `thumb16_undefined` for the
    /// narrow path, `thumb32_undefined` for the wide path — so the
    /// indirect dispatch on the caller side runs something harmless
    /// before `step()` delivers the fault. Strictly safer than the
    /// pre-Stage-B behaviour, which would have run the cached handler
    /// on junk halfwords. See HLD
    /// `2026.04.15 - HLD - Fn-Pointer Dispatch in DecodedOp.md` §5.
    #[cold]
    #[inline(never)]
    fn populate_decode_cache(&mut self, bus: &mut Bus, pc: u32) -> DecodedOp {
        // Reset the accumulator so the fetch's wait-state contribution
        // (sram bank 2/6 = +1, others = 0) can be captured cleanly in
        // `fetch_wait`. The caller (`decode_execute`) will not look at
        // `bus.extra_wait_states()` after we return; it uses
        // `entry.fetch_wait` on both paths.
        bus.reset_extra_wait_states();

        let hw0 = bus.read16(pc);
        if bus.bus_fault() {
            // Fetch fault — DO NOT cache. Return a minimal entry so the
            // caller's dispatch path can proceed and the post-step fault
            // delivery will fire. `is_pure = false` keeps us on the slow
            // path, which preserves today's `+extra_wait_states` behaviour.
            return DecodedOp {
                handler: CortexM33::thumb16_undefined,
                tag: u32::MAX,
                hw0,
                hw1: 0,
                fetch_wait: 0,
                flags: 0,
            };
        }

        let wide = is_wide(hw0);
        let hw1 = if wide { bus.read16(pc.wrapping_add(2)) } else { 0 };
        if wide && bus.bus_fault() {
            return DecodedOp {
                handler: CortexM33::thumb32_undefined,
                tag: u32::MAX,
                hw0,
                hw1,
                fetch_wait: 0,
                flags: DecodedOp::FLAG_WIDE,
            };
        }

        // Whatever the two fetches charged is the fetch contribution.
        // Max value is 2 (wide crossing bank 2/6 twice). u8 is ample.
        let fetch_wait = bus.extra_wait_states().min(u8::MAX as u32) as u8;

        let flag_only = !wide && is_thumb16_flag_only(hw0);
        let pure = classify_is_pure(hw0, hw1, wide);
        let handler = classify_handler(hw0, hw1, wide);

        let mut flags = 0u8;
        if wide { flags |= DecodedOp::FLAG_WIDE; }
        if pure { flags |= DecodedOp::FLAG_PURE; }
        if flag_only { flags |= DecodedOp::FLAG_FLAG_ONLY; }

        let entry = DecodedOp { handler, tag: pc, hw0, hw1, fetch_wait, flags };

        if is_cacheable_pc(pc) {
            let slot = ((pc >> 1) & CACHE_INDEX_MASK) as usize;
            bus.decode_cache[slot] = entry;
        }

        entry
    }

    /// Top-level Thumb-16 dispatch.
    ///
    /// Post-Stage-B this is a thin wrapper around `classify_handler` —
    /// the cache hit path in `decode_execute` dispatches via the cached
    /// fn pointer without touching this function. `execute_one*_with_bus`
    /// test helpers in `core/mod.rs` still call here so they can drive a
    /// single opcode without populating the cache.
    ///
    /// **Mirror:** `classify_handler` (this module) routes the exact same
    /// `(hw0, 0, is_wide=false)` input to the same handler item. Any edit
    /// to one MUST land in the other or the `dispatch_equiv` tests in
    /// `tests.rs` will fail.
    ///
    /// See HLD `2026.04.15 - HLD - Fn-Pointer Dispatch in DecodedOp.md` §2.6.
    pub(crate) fn execute_thumb16(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        (classify_handler(opcode, 0, false))(self, opcode, 0, bus)
    }

    /// Top-level Thumb-32 dispatch.
    ///
    /// Post-Stage-B this is a thin wrapper around `classify_handler` —
    /// the cache hit path in `decode_execute` dispatches via the cached
    /// fn pointer without touching this function. `execute_one_wide*`
    /// test helpers in `core/mod.rs` still call here.
    ///
    /// **Mirror:** `classify_handler` (this module) routes the exact same
    /// `(hw0, hw1, is_wide=true)` input to the same handler item. Any
    /// edit to one MUST land in the other or the `dispatch_equiv` tests
    /// in `tests.rs` will fail.
    ///
    /// See HLD `2026.04.15 - HLD - Fn-Pointer Dispatch in DecodedOp.md` §2.6.
    pub(crate) fn execute_thumb32(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        (classify_handler(hw0, hw1, true))(self, hw0, hw1, bus)
    }
}

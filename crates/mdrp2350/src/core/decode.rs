use super::{CoreBus, CortexM33};
use crate::bus::{DECODE_CACHE_SIZE, DecodedOp, is_cacheable_pc};

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
        0b00000..=0b00010 => true,
        // 00011 ADD/SUB reg / imm3 — pure.
        0b00011 => true,
        // 00100..00111 MOV/CMP/ADD/SUB imm8 — pure.
        0b00100..=0b00111 => true,
        // 01000 — bit 10 discriminates data-processing (pure) from
        // special-data/BX (impure; BX/BLX/MOV-PC may hit exit_exception).
        0b01000 => opcode & (1 << 10) == 0,
        // 01001 LDR literal — impure (bus.read32).
        0b01001 => false,
        // 01010 / 01011 LDR/STR register offset — impure.
        0b01010 | 0b01011 => false,
        // 01100..10001 LDR/STR immediate offset (six handlers) — impure.
        0b01100..=0b10001 => false,
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
        return matches!(hint, 0x00..=0x04);
    }
    // Barriers (hw0 == 0xF3BF): CLREX / DSB / DMB / ISB all pure; others
    // fall into `thumb32_undefined`.
    if hw0 == 0xF3BF {
        let barrier_op = (hw1 >> 4) & 0xF;
        return matches!(barrier_op, 0x2 | 0x4 | 0x5 | 0x6);
    }
    let op_field = (hw0 >> 4) & 0x7F;
    // MSR / MRS — register-file only, pure.
    if op_field == 0b0111000
        || op_field == 0b0111001
        || op_field == 0b0111110
        || op_field == 0b0111111
    {
        return true;
    }
    // Otherwise falls into `thumb32_undefined` — impure.
    false
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
    pub(crate) fn decode_execute<B: CoreBus>(&mut self, bus: &mut B) -> u32 {
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        // Publish the instruction PC on the bus so the MMIO trace
        // (HLD V5 §4.2.7) can report it for every access this
        // instruction performs. Set before the fetch so the I-fetch
        // itself is tagged with its own PC. Zero-cost when tracing is
        // off — the store lands in a cold struct field touched only
        // by `emit_mmio_trace`.
        bus.set_active_pc(pc, self.core_id);

        // Is this fetch sequential from the previous instruction?
        // The M33 prefetch buffer absorbs bank 2/6 penalty on sequential
        // fetches. Read BEFORE the cache lookup (which may call
        // populate_decode_cache).
        let last = bus.last_fetch_addr();
        let is_sequential = pc == last.wrapping_add(2) || pc == last.wrapping_add(4);

        // Cache lookup — by-value (`DecodedOp: Copy`), so no borrow on
        // `bus` survives into dispatch. Cache lives on `self`
        // (per-core) since Phase 3 follow-up #10.
        let entry = if is_cacheable_pc(pc) {
            let slot = ((pc >> 1) & CACHE_INDEX_MASK) as usize;
            let e = self.decode_cache_get(slot);
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

        // Update last_fetch_addr for the NEXT instruction's sequential
        // check. For wide instructions the second halfword is at pc+2,
        // so the next sequential PC is pc+4 (checked via wrapping_add(2)
        // on the stored value pc+2).
        bus.set_last_fetch_addr(if is_wide { pc.wrapping_add(2) } else { pc });

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

            let cycles = if is_wide {
                self.regs.set_pc(pc.wrapping_add(4));
                let c = if cond_passed {
                    self.execute_thumb32(hw0, hw1, bus)
                } else {
                    1
                };
                if in_it {
                    self.advance_it_state();
                }
                c
            } else {
                self.regs.set_pc(pc.wrapping_add(2));
                let saved_flags = if in_it {
                    self.regs.xpsr & 0xF800_0000
                } else {
                    0
                };
                let c = if cond_passed {
                    self.execute_thumb16(hw0, bus)
                } else {
                    1
                };
                // Flag-only suppression in IT blocks — same logic as the
                // slow path, but using the pre-computed `flag_only` bit.
                if in_it && cond_passed && !flag_only {
                    self.regs.xpsr = (self.regs.xpsr & !0xF800_0000) | saved_flags;
                }
                if in_it {
                    self.advance_it_state();
                }
                c
            };

            #[cfg(debug_assertions)]
            debug_assert_eq!(
                bus.extra_wait_states(),
                ws_before,
                "pure op at PC={:08X} (hw0={:04X}, hw1={:04X}) \
                 dirtied bus.extra_wait_states",
                pc,
                hw0,
                hw1,
            );

            // Apply bank penalty only on non-sequential fetches. The
            // M33 prefetch buffer absorbs the penalty on sequential PC
            // advances (PC+2 or PC+4).
            let bank_penalty = if is_sequential {
                0
            } else {
                entry.fetch_wait as u32
            };
            cycles + bank_penalty
        } else {
            // Slow path — preserves existing semantics verbatim.
            bus.reset_extra_wait_states();
            // Fetch contribution: apply only on non-sequential fetches.
            if !is_sequential {
                bus.add_extra_wait_states(entry.fetch_wait as u32);
            }

            if is_wide {
                self.regs.set_pc(pc.wrapping_add(4));
                let cycles = if cond_passed {
                    self.execute_thumb32(hw0, hw1, bus)
                } else {
                    1
                };
                if in_it {
                    self.advance_it_state();
                }
                cycles + bus.extra_wait_states()
            } else {
                self.regs.set_pc(pc.wrapping_add(2));
                let saved_flags = if in_it {
                    self.regs.xpsr & 0xF800_0000
                } else {
                    0
                };
                let cycles = if cond_passed {
                    self.execute_thumb16(hw0, bus)
                } else {
                    1
                };
                if in_it && cond_passed && !flag_only {
                    self.regs.xpsr = (self.regs.xpsr & !0xF800_0000) | saved_flags;
                }
                if in_it {
                    self.advance_it_state();
                }
                cycles + bus.extra_wait_states()
            }
        }
    }

    /// Populate path — runs on a cache miss. Fetches `hw0` (and `hw1`
    /// for wide instructions) via the bus, classifies purity, and
    /// writes the slot. Returns a `DecodedOp` for the caller to
    /// dispatch immediately.
    ///
    /// Faulty fetches are NOT cached (see HLD §8.1): the slot is left
    /// untouched, the returned entry still carries the fetched
    /// halfwords so `decode_execute` can drive the existing fault
    /// delivery path (which checks `bus.bus_fault()` after the
    /// `decode_execute` call returns).
    #[cold]
    #[inline(never)]
    fn populate_decode_cache<B: CoreBus>(&mut self, bus: &mut B, pc: u32) -> DecodedOp {
        // Reset the accumulator so the fetch's wait-state contribution
        // can be captured cleanly. bus.read16(pc) no longer accumulates
        // sram_bank_wait (removed from data paths), so extra_wait_states
        // only catches non-SRAM fetch penalty (APB/XIP).
        bus.reset_extra_wait_states();

        let hw0 = bus.read16(pc, self.core_id);
        if bus.bus_fault(self.core_id) {
            // Fetch fault — DO NOT cache. Return a minimal entry so the
            // caller's dispatch path can proceed and the post-step fault
            // delivery will fire. `is_pure = false` keeps us on the slow
            // path, which preserves today's `+extra_wait_states` behaviour.
            return DecodedOp {
                tag: u32::MAX,
                hw0,
                hw1: 0,
                fetch_wait: 0,
                flags: 0,
            };
        }

        let wide = is_wide(hw0);
        let hw1 = if wide {
            bus.read16(pc.wrapping_add(2), self.core_id)
        } else {
            0
        };
        if wide && bus.bus_fault(self.core_id) {
            return DecodedOp {
                tag: u32::MAX,
                hw0,
                hw1,
                fetch_wait: 0,
                flags: DecodedOp::FLAG_WIDE,
            };
        }

        // Compute raw SRAM bank penalty from the PC address. This is the
        // penalty for a NON-sequential fetch; the caller (decode_execute)
        // decides whether to apply it based on sequentiality at dispatch
        // time. bus.read16(pc) no longer accumulates sram_bank_wait.
        let sram_fetch_penalty: u8 = {
            let off = pc & 0x000F_FFFF;
            if off < 0x8_0000 {
                let bank = (off >> 2) & 7;
                if bank == 2 || bank == 6 { 1 } else { 0 }
            } else {
                0
            }
        };
        // Capture any NON-SRAM fetch penalty (APB/XIP) that the bus read
        // may have accumulated.
        let extra_from_bus = bus.extra_wait_states().min(u8::MAX as u32) as u8;
        bus.reset_extra_wait_states();
        let fetch_wait = sram_fetch_penalty + extra_from_bus;

        let flag_only = !wide && is_thumb16_flag_only(hw0);
        let pure = classify_is_pure(hw0, hw1, wide);

        let mut flags = 0u8;
        if wide {
            flags |= DecodedOp::FLAG_WIDE;
        }
        if pure {
            flags |= DecodedOp::FLAG_PURE;
        }
        if flag_only {
            flags |= DecodedOp::FLAG_FLAG_ONLY;
        }

        let entry = DecodedOp {
            tag: pc,
            hw0,
            hw1,
            fetch_wait,
            flags,
        };

        if is_cacheable_pc(pc) {
            let slot = ((pc >> 1) & CACHE_INDEX_MASK) as usize;
            self.decode_cache_set(slot, entry);
        }

        entry
    }

    /// Top-level Thumb-16 dispatch. Routes to instruction group handlers
    /// in execute.rs based on bits [15:11].
    ///
    /// Every arm calls a handler with the uniform signature
    /// `fn(&mut CortexM33, hw0: u16, hw1: u16, &mut Bus) -> u32`. For
    /// Thumb-16 the opcode is the single halfword, so `hw0 = opcode`
    /// and `hw1 = 0`. Prerequisite for fn-pointer dispatch (Stage A of
    /// HLD 2026.04.15).
    pub(crate) fn execute_thumb16<B: CoreBus>(&mut self, opcode: u16, bus: &mut B) -> u32 {
        match opcode >> 11 {
            // Shift (immediate)
            0b00000 => self.thumb16_lsl_imm(opcode, 0, bus),
            0b00001 => self.thumb16_lsr_imm(opcode, 0, bus),
            0b00010 => self.thumb16_asr_imm(opcode, 0, bus),
            // Add/sub register and 3-bit immediate
            0b00011 => self.thumb16_add_sub(opcode, 0, bus),
            // Move/compare/add/sub 8-bit immediate
            0b00100 => self.thumb16_mov_imm(opcode, 0, bus),
            0b00101 => self.thumb16_cmp_imm(opcode, 0, bus),
            0b00110 => self.thumb16_add_imm8(opcode, 0, bus),
            0b00111 => self.thumb16_sub_imm8(opcode, 0, bus),
            // Data processing + special data + BX
            // bits[15:10] = 010000 → data processing
            // bits[15:10] = 010001 → special data / BX / BLX
            0b01000 => {
                if opcode & (1 << 10) == 0 {
                    self.thumb16_data_processing(opcode, 0, bus)
                } else {
                    self.thumb16_special_data_bx(opcode, 0, bus)
                }
            }
            0b01001 => self.thumb16_ldr_literal(opcode, 0, bus),
            // Load/store register offset
            0b01010 | 0b01011 => self.thumb16_load_store_reg(opcode, 0, bus),
            // Load/store word immediate offset
            0b01100 => self.thumb16_str_imm(opcode, 0, bus),
            0b01101 => self.thumb16_ldr_imm(opcode, 0, bus),
            // Load/store byte immediate offset
            0b01110 => self.thumb16_strb_imm(opcode, 0, bus),
            0b01111 => self.thumb16_ldrb_imm(opcode, 0, bus),
            // Load/store halfword immediate offset
            0b10000 => self.thumb16_strh_imm(opcode, 0, bus),
            0b10001 => self.thumb16_ldrh_imm(opcode, 0, bus),
            // SP-relative load/store
            0b10010 => self.thumb16_str_sp(opcode, 0, bus),
            0b10011 => self.thumb16_ldr_sp(opcode, 0, bus),
            // ADR (PC-relative) and ADD SP+imm
            0b10100 => self.thumb16_adr(opcode, 0, bus),
            0b10101 => self.thumb16_add_sp_imm(opcode, 0, bus),
            // Miscellaneous
            0b10110 | 0b10111 => self.thumb16_misc(opcode, 0, bus),
            // Store/Load multiple
            0b11000 => self.thumb16_stm(opcode, 0, bus),
            0b11001 => self.thumb16_ldm(opcode, 0, bus),
            // Conditional branch + SVC
            0b11010 | 0b11011 => self.thumb16_cond_branch_svc(opcode, 0, bus),
            // Unconditional branch
            0b11100 => self.thumb16_branch(opcode, 0, bus),
            // 32-bit prefix (should not reach here via this path)
            _ => self.thumb16_undefined(opcode, 0, bus),
        }
    }

    /// Top-level Thumb-32 dispatch.
    ///
    /// Every arm calls a handler with the uniform signature
    /// `fn(&mut CortexM33, hw0: u16, hw1: u16, &mut Bus) -> u32`.
    /// Prerequisite for fn-pointer dispatch (Stage A of HLD 2026.04.15).
    pub(crate) fn execute_thumb32<B: CoreBus>(&mut self, hw0: u16, hw1: u16, bus: &mut B) -> u32 {
        let op1 = (hw0 >> 11) & 0x3;
        let op2 = ((hw0 >> 4) & 0x7F) as u32;
        let op = (hw1 >> 15) & 0x1;

        match op1 {
            0b01 => match op2 >> 5 {
                0b00 => {
                    if op2 & 0x04 == 0 {
                        self.thumb32_ldm_stm(hw0, hw1, bus)
                    } else {
                        self.thumb32_load_store_dual(hw0, hw1, bus)
                    }
                }
                0b01 => self.thumb32_dp_shifted_reg(hw0, hw1, bus),
                _ => self.thumb32_coprocessor(hw0, hw1, bus),
            },
            0b10 => {
                if op == 0 {
                    if op2 & 0x20 == 0 {
                        self.thumb32_dp_modified_imm(hw0, hw1, bus)
                    } else {
                        self.thumb32_dp_plain_imm(hw0, hw1, bus)
                    }
                } else {
                    self.thumb32_branch_misc(hw0, hw1, bus)
                }
            }
            0b11 => {
                if op2 & 0x40 != 0 {
                    self.thumb32_coprocessor(hw0, hw1, bus)
                } else if op2 & 0x20 == 0 {
                    self.thumb32_load_store_single(hw0, hw1, bus)
                } else if op2 & 0x10 == 0 {
                    self.thumb32_dp_register(hw0, hw1, bus)
                } else if op2 & 0x08 == 0 {
                    self.thumb32_multiply(hw0, hw1, bus)
                } else {
                    self.thumb32_long_multiply(hw0, hw1, bus)
                }
            }
            _ => self.thumb32_undefined(hw0, hw1, bus),
        }
    }
}

#[cfg(test)]
mod classifier_tests {
    //! Direct tests for the purity / flag-only classifier helpers in
    //! this module.
    //!
    //! Strategy: each classifier is a pure function of opcode bits. We
    //! assert (a) per-match-arm structural cases with named encodings,
    //! and (b) FNV-1a fingerprints over the relevant input space (full
    //! 16-bit space for Thumb-16; cross-product of structural patterns
    //! for Thumb-32). Any mutation that changes even one input → output
    //! mapping flips the fingerprint.
    //!
    //! Classifier consumer (mdrp2350): `decode_execute` reads
    //! `is_pure()` to gate the fast-path / slow-path split (line 284).
    //! The fast and slow paths produce identical register / xpsr state;
    //! only `bus.extra_wait_states()` accumulation differs, which feeds
    //! cycle counting. The QEMU oracle does not compare cycles cross-
    //! tool (qemu.cycles is hardcoded 0), so classifier mutations are
    //! observable only via direct unit tests like these.
    //!
    //! NOTE: the fingerprint is "current behaviour, not architectural
    //! truth". If a real classifier bug is fixed, update the asserted
    //! constant after manually reviewing the diff.
    use super::{
        classify_is_pure, classify_thumb16_misc_pure, classify_thumb16_pure,
        classify_thumb32_branch_misc_pure, classify_thumb32_misc_control_pure,
        classify_thumb32_pure, is_thumb16_flag_only, is_wide,
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

    /// Build a Thumb-16 opcode with prefix `p` (5 bits) and a body of zeros.
    fn t16(prefix: u16) -> u16 {
        prefix << 11
    }

    /// Build a misc-group opcode with op[11:8] = `op`. Prefix is 1011_0.
    fn misc(op: u16) -> u16 {
        (0b10110u16 << 11) | (op << 8)
    }

    // ---------- classify_thumb16_pure: per-prefix structural ----------

    #[test]
    fn t16_shift_immediate_is_pure() {
        // 00000 LSL imm, 00001 LSR imm, 00010 ASR imm
        for prefix in 0b00000..=0b00010u16 {
            assert!(
                classify_thumb16_pure(t16(prefix)),
                "prefix {prefix:05b} should be pure (shift imm)"
            );
        }
    }

    #[test]
    fn t16_add_sub_is_pure() {
        assert!(classify_thumb16_pure(t16(0b00011)));
    }

    #[test]
    fn t16_imm8_data_processing_is_pure() {
        for prefix in 0b00100..=0b00111u16 {
            assert!(
                classify_thumb16_pure(t16(prefix)),
                "prefix {prefix:05b} should be pure (imm8 DP)"
            );
        }
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
                "prefix {prefix:05b} should be impure (LDR/STR imm offset)"
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
        // op[11:8] = 0b0000 → ADD/SUB SP imm7 (pure)
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
        // classify_thumb16_pure (is_wide catches all three on M33), but
        // if they do, the function returns false (impure).
        assert!(!classify_thumb16_pure(t16(0b11101)));
        assert!(!classify_thumb16_pure(t16(0b11110)));
        assert!(!classify_thumb16_pure(t16(0b11111)));
    }

    // ---------- classify_thumb16_misc_pure: per-op structural ----------

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
    fn misc_bkpt_is_pure_on_m33() {
        // mdrp2350 classifies BKPT as a NOP stub → pure.
        // (mdrp2040 classifies BKPT impure because it sets pending_fault.)
        assert!(classify_thumb16_misc_pure(misc(0b1110)));
    }

    #[test]
    fn misc_hints_are_pure() {
        // op[11:8] == 0b1111 → IT / hints (NOP / YIELD / WFE / WFI / SEV).
        assert!(classify_thumb16_misc_pure(misc(0b1111)));
    }

    #[test]
    fn misc_cbz_cbnz_are_pure() {
        // CBZ / CBNZ match `op & 0x5 == 0x1` (op bits[11:8] in {0x1, 0x3, 0x9, 0xB}).
        // M33-only encoding; mdrp2040 lacks this.
        for op in [0x1u16, 0x3, 0x9, 0xB] {
            assert!(
                classify_thumb16_misc_pure(misc(op)),
                "CBZ/CBNZ op {op:#x} should be pure"
            );
        }
    }

    #[test]
    fn misc_unenumerated_ops_are_pure_by_fallback() {
        // The mdrp2350 fallback arm is `_ => true` (NOP stubs default
        // to pure). This is the structural counterpart to mdrp2040's
        // `_ => false` — encode the divergence so a mutation flipping
        // either crate's fallback is caught.
        for op in 0..=0xFu16 {
            let expected = match op {
                0b0100 | 0b0101 | 0b1100 | 0b1101 => false, // PUSH / POP
                _ => true,
            };
            assert_eq!(
                classify_thumb16_misc_pure(misc(op)),
                expected,
                "misc op[11:8]={op:#x} expected {expected}"
            );
        }
    }

    // ---------- is_thumb16_flag_only ----------

    #[test]
    fn flag_only_cmp_imm8_is_flag_only() {
        // 00101 CMP Rn, #imm8
        assert!(is_thumb16_flag_only(t16(0b00101)));
    }

    #[test]
    fn flag_only_data_processing_tst_cmp_cmn() {
        // 0b01000 with bit10=0, dp_op = (opcode >> 6) & 0xF.
        // Flag-only: TST (0x8), CMP (0xA), CMN (0xB).
        for dp_op in [0x8u16, 0xA, 0xB] {
            let opc = (0b01000u16 << 11) | (dp_op << 6);
            assert!(
                is_thumb16_flag_only(opc),
                "DP op {dp_op:#x} should be flag-only"
            );
        }
        // Other DP ops are NOT flag-only.
        for dp_op in [0x0u16, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x9, 0xC, 0xD, 0xE, 0xF] {
            let opc = (0b01000u16 << 11) | (dp_op << 6);
            assert!(
                !is_thumb16_flag_only(opc),
                "DP op {dp_op:#x} should NOT be flag-only"
            );
        }
    }

    #[test]
    fn flag_only_special_data_cmp() {
        // 0b01000 with bit10=1, opcode[9:8] = 0b01 → CMP Rn, Rm (high reg).
        let opc = (0b01000u16 << 11) | (1 << 10) | (0b01 << 8);
        assert!(is_thumb16_flag_only(opc));
        // Other special-data sub-ops are NOT flag-only.
        for sub in [0b00u16, 0b10, 0b11] {
            let opc = (0b01000u16 << 11) | (1 << 10) | (sub << 8);
            assert!(
                !is_thumb16_flag_only(opc),
                "special-data sub {sub:#x} should NOT be flag-only"
            );
        }
    }

    #[test]
    fn flag_only_other_prefixes_are_not_flag_only() {
        // Spot-check: shifts, branches, loads/stores never flag-only.
        for prefix in [0b00000u16, 0b00100, 0b01001, 0b11100] {
            assert!(
                !is_thumb16_flag_only(t16(prefix)),
                "prefix {prefix:05b} should NOT be flag-only"
            );
        }
    }

    // ---------- classify_thumb32_pure: per-encoding structural ----------

    #[test]
    fn t32_op1_01_dp_shifted_reg_is_pure() {
        // op1 = 0b01, op2 >> 5 = 0b01 → dp_shifted_reg (pure).
        let hw0 = (0b11101u16 << 11) | (0b01 << 11) | (0b01_00000 << 4);
        assert!(classify_thumb32_pure(hw0, 0));
    }

    #[test]
    fn t32_op1_01_load_store_dual_is_impure() {
        // op1 = 0b01, op2 >> 5 = 0b00 → ldm/stm/ldrd/strd (impure).
        let hw0 = (0b11101u16 << 11) | (0b01 << 11) | (0b00_00000 << 4);
        assert!(!classify_thumb32_pure(hw0, 0));
    }

    #[test]
    fn t32_op1_01_coprocessor_is_impure() {
        // op1 = 0b01, op2 >> 5 in {0b10, 0b11} → coprocessor (impure).
        let hw0_10 = (0b11101u16 << 11) | (0b01 << 11) | (0b10_00000 << 4);
        let hw0_11 = (0b11101u16 << 11) | (0b01 << 11) | (0b11_00000 << 4);
        assert!(!classify_thumb32_pure(hw0_10, 0));
        assert!(!classify_thumb32_pure(hw0_11, 0));
    }

    #[test]
    fn t32_op1_10_dp_imm_is_pure_when_hw1_msb_clear() {
        // op1 = 0b10, hw1 bit15 = 0 → dp_modified_imm / dp_plain_imm (pure).
        let hw0 = (0b11110u16 << 11) | (0b10 << 11);
        let hw1 = 0x0000;
        assert!(classify_thumb32_pure(hw0, hw1));
    }

    #[test]
    fn t32_op1_10_branch_misc_dispatches() {
        // op1 = 0b10, hw1 bit15 = 1 → branch_misc (recursive dispatch).
        let hw0 = (0b11110u16 << 11) | (0b10 << 11);
        // BL: hw1 bit14 set. classify_thumb32_branch_misc_pure returns true.
        let hw1 = 1 << 14;
        assert!(classify_thumb32_pure(hw0, hw1));
    }

    #[test]
    fn t32_op1_11_load_store_single_is_impure() {
        // op1 = 0b11, op2 & 0x40 = 0, op2 & 0x20 = 0 → load_store_single (impure).
        let hw0 = (0b11111u16 << 11) | (0b11 << 11) | (0b0000000 << 4);
        assert!(!classify_thumb32_pure(hw0, 0));
    }

    #[test]
    fn t32_op1_11_dp_register_is_pure() {
        // op1 = 0b11, op2 & 0x40 = 0, op2 & 0x20 = 1, op2 & 0x10 = 0 → dp_register (pure).
        let hw0 = (0b11111u16 << 11) | (0b11 << 11) | (0b0100000 << 4);
        assert!(classify_thumb32_pure(hw0, 0));
    }

    #[test]
    fn t32_op1_11_multiply_is_pure() {
        // op2 & 0x40 = 0, op2 & 0x20 = 1, op2 & 0x10 = 1, op2 & 0x08 = 0 → multiply (pure).
        let hw0 = (0b11111u16 << 11) | (0b11 << 11) | (0b0110000 << 4);
        assert!(classify_thumb32_pure(hw0, 0));
    }

    #[test]
    fn t32_op1_11_long_multiply_is_pure() {
        // op2 & 0x40 = 0, op2 & 0x20 = 1, op2 & 0x10 = 1, op2 & 0x08 = 1 → long_multiply (pure).
        let hw0 = (0b11111u16 << 11) | (0b11 << 11) | (0b0111000 << 4);
        assert!(classify_thumb32_pure(hw0, 0));
    }

    #[test]
    fn t32_op1_11_coprocessor_is_impure() {
        // op2 & 0x40 != 0 → coprocessor (impure).
        let hw0 = (0b11111u16 << 11) | (0b11 << 11) | (0b1000000 << 4);
        assert!(!classify_thumb32_pure(hw0, 0));
    }

    // ---------- classify_thumb32_branch_misc_pure ----------

    #[test]
    fn branch_misc_bl_is_pure() {
        // BL: hw1 bit14 set.
        assert!(classify_thumb32_branch_misc_pure(0xF000, 1 << 14));
    }

    #[test]
    fn branch_misc_b_w_t4_is_pure() {
        // B.W T4: hw1 bit14=0, hw1 bit12=1.
        assert!(classify_thumb32_branch_misc_pure(0xF000, 1 << 12));
    }

    #[test]
    fn branch_misc_b_w_t3_cond_is_pure() {
        // B.W T3: hw1 bit14=0, hw1 bit12=0, misc_op = (hw0 >> 6) & 0xF
        // with `misc_op & 0xE != 0xE` → conditional branch (pure).
        let hw0 = 0xF000; // misc_op bits[9:6] = 0b0000.
        assert!(classify_thumb32_branch_misc_pure(hw0, 0));
    }

    #[test]
    fn branch_misc_misc_control_dispatches() {
        // misc_op & 0xE == 0xE → misc_control sub-dispatch.
        // Use NOP.W (pure) and undefined hint (impure) cases.
        let hw0_nop = 0xF3AF;
        let hw1_nop = 0x0000; // hint = 0x00 → NOP.
        let hw1_unrecognised = 0x0005; // hint = 0x05 (out of 0..=0x04).
        assert!(classify_thumb32_branch_misc_pure(hw0_nop, hw1_nop));
        assert!(!classify_thumb32_branch_misc_pure(hw0_nop, hw1_unrecognised));
    }

    // ---------- classify_thumb32_misc_control_pure ----------

    #[test]
    fn misc_control_hints_are_pure() {
        let hw0 = 0xF3AF;
        for hint in 0x00..=0x04u16 {
            assert!(
                classify_thumb32_misc_control_pure(hw0, hint),
                "hint {hint:#x} should be pure"
            );
        }
        // Out-of-range hint → impure (falls into thumb32_undefined).
        assert!(!classify_thumb32_misc_control_pure(hw0, 0x05));
        assert!(!classify_thumb32_misc_control_pure(hw0, 0xFF));
    }

    #[test]
    fn misc_control_barriers_are_pure() {
        // hw0 == 0xF3BF, barrier_op = (hw1 >> 4) & 0xF in {0x2, 0x4, 0x5, 0x6}.
        let hw0 = 0xF3BF;
        for barrier_op in [0x2u16, 0x4, 0x5, 0x6] {
            let hw1 = barrier_op << 4;
            assert!(
                classify_thumb32_misc_control_pure(hw0, hw1),
                "barrier_op {barrier_op:#x} should be pure"
            );
        }
        // Unrecognised barrier (e.g. 0x7) → impure.
        let hw1 = 0x7 << 4;
        assert!(!classify_thumb32_misc_control_pure(hw0, hw1));
    }

    #[test]
    fn misc_control_msr_mrs_are_pure() {
        // op_field = (hw0 >> 4) & 0x7F in {0b0111000, 0b0111001, 0b0111110, 0b0111111}.
        for op_field in [0b0111000u16, 0b0111001, 0b0111110, 0b0111111] {
            let hw0 = (op_field & 0x7F) << 4;
            // hw0 must NOT be 0xF3BF or 0xF3AF — the function checks those first.
            // Pick a base that doesn't collide.
            let hw0 = hw0 | 0xF000;
            // Still need to avoid F3AF / F3BF.
            assert!(hw0 != 0xF3AF && hw0 != 0xF3BF);
            assert!(
                classify_thumb32_misc_control_pure(hw0, 0),
                "op_field {op_field:#b} should be pure"
            );
        }
    }

    #[test]
    fn misc_control_unrecognised_is_impure() {
        // Some op_field that isn't MSR/MRS/hints/barriers.
        let hw0 = 0xF000; // op_field = 0, not matching any case.
        assert!(!classify_thumb32_misc_control_pure(hw0, 0));
    }

    // ---------- classify_is_pure: dispatcher ----------

    #[test]
    fn dispatcher_routes_thumb16() {
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
        let hw0 = (0b11110u16 << 11) | (0b10 << 11);
        let hw1 = 1 << 14; // BL
        assert_eq!(
            classify_is_pure(hw0, hw1, true),
            classify_thumb32_pure(hw0, hw1)
        );
    }

    // ---------- is_wide ----------

    #[test]
    fn is_wide_matches_all_m33_thumb32_prefixes() {
        // M33 accepts three wide prefixes: 0b11101, 0b11110, 0b11111
        // (i.e. hw0 >= 0xE800).
        assert!(is_wide(0xE800));
        assert!(is_wide(0xF000)); // 0b11110
        assert!(is_wide(0xF800)); // 0b11111
        assert!(is_wide(0xFFFF));
        // Just below threshold → narrow.
        assert!(!is_wide(0xE7FF));
        assert!(!is_wide(0x0000));
        assert!(!is_wide(0x4000));
    }

    // ---------- exhaustive Thumb-16 fingerprint ----------
    //
    // Snapshot tests: enumerate the entire 16-bit input space, compute
    // each classifier's output, and assert the FNV-1a fingerprint
    // matches a checked-in constant. ANY mutation that changes even one
    // input → output mapping flips this hash.

    #[test]
    fn t16_pure_full_space_fingerprint() {
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
        // The misc classifier inspects opcode[11:8]; enumerate all 16
        // sub-ops at canonical misc encoding (prefix 1011_0).
        let bits = (0..=0xFu16).map(misc).map(classify_thumb16_misc_pure);
        let packed = pack_bool_bits(bits);
        let h = fnv1a64(&packed);
        assert_eq!(
            h, MISC_PURE_FINGERPRINT,
            "classify_thumb16_misc_pure fingerprint changed (computed = {h:#018x})"
        );
    }

    #[test]
    fn flag_only_full_space_fingerprint() {
        let bits = (0..=0xFFFFu16).map(is_thumb16_flag_only);
        let packed = pack_bool_bits(bits);
        let h = fnv1a64(&packed);
        assert_eq!(
            h, FLAG_ONLY_FINGERPRINT,
            "is_thumb16_flag_only fingerprint changed (computed = {h:#018x})"
        );
    }

    #[test]
    fn t32_pure_structured_fingerprint() {
        // Cross-product over a representative set of (hw0, hw1) pairs
        // covering the dispatch arms in classify_thumb32_pure. Chosen
        // to exercise every match arm — see the structural tests above.
        let mut bits = Vec::with_capacity(64);
        // op1 = 0b01, op2 >> 5 ∈ {0b00, 0b01, 0b10, 0b11}
        for op2_high in [0b00u16, 0b01, 0b10, 0b11] {
            let hw0 = (0b11101u16 << 11) | (0b01 << 11) | ((op2_high << 5) << 4);
            bits.push(classify_thumb32_pure(hw0, 0));
        }
        // op1 = 0b10, hw1 bit15 ∈ {0, 1} × hw1 bit14 ∈ {0, 1} × misc_op patterns
        for hw1 in [0x0000u16, 0x4000, 0x8000, 0xC000, 0x4000 | 0x1000, 0x8000 | 0xF000] {
            let hw0 = (0b11110u16 << 11) | (0b10 << 11);
            bits.push(classify_thumb32_pure(hw0, hw1));
        }
        // op1 = 0b11, op2 patterns
        for op2 in [0b0000000u16, 0b0100000, 0b0110000, 0b0111000, 0b1000000] {
            let hw0 = (0b11111u16 << 11) | (0b11 << 11) | (op2 << 4);
            bits.push(classify_thumb32_pure(hw0, 0));
        }
        let packed = pack_bool_bits(bits);
        let h = fnv1a64(&packed);
        assert_eq!(
            h, T32_PURE_STRUCTURED_FINGERPRINT,
            "classify_thumb32_pure fingerprint changed (computed = {h:#018x})"
        );
    }

    /// FNV-1a 64-bit hash of the bit-packed `classify_thumb16_pure`
    /// output over inputs `0..=0xFFFF`. Computed 2026-04-30 against
    /// the M33 classifier as committed at this point in the V3 work.
    const T16_PURE_FINGERPRINT: u64 = 0x1eb1ffd1d55d2865;
    /// FNV-1a 64-bit hash of `classify_thumb16_misc_pure` over the
    /// 16 misc sub-ops (canonical prefix 1011_0).
    const MISC_PURE_FINGERPRINT: u64 = 0x0acfb907b723bba3;
    /// FNV-1a 64-bit hash of `is_thumb16_flag_only` over the entire
    /// 16-bit input space.
    const FLAG_ONLY_FINGERPRINT: u64 = 0x15d893b9ae1ee96d;
    /// FNV-1a 64-bit hash of `classify_thumb32_pure` over the
    /// structured cross-product covering each dispatch arm.
    const T32_PURE_STRUCTURED_FINGERPRINT: u64 = 0x0a8f8d07b6ed8cea;
}

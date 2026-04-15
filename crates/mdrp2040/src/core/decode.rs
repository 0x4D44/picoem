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

use super::CortexM0Plus;
use crate::bus::Bus;

/// Returns true iff the first halfword is the Thumb-32 prefix defined
/// for ARMv6-M (`0b11110xxx xxxxxxxx`). M0+ supports exactly one wide
/// prefix — unlike M33 which also accepts `0b11101` and `0b11111`.
#[inline(always)]
pub(crate) fn is_wide(hw0: u16) -> bool {
    (hw0 >> 11) == 0b11110
}

impl CortexM0Plus {
    /// Fetch-decode-execute one instruction. Returns cycle count.
    ///
    /// Phase 4.B: the Thumb-32 path routes BL / MRS / MSR / DSB / DMB /
    /// ISB through [`Self::execute_thumb32`]; any other wide encoding
    /// raises HardFault via [`super::Fault::Undefined`].
    pub(crate) fn decode_execute(&mut self, bus: &mut Bus) -> u32 {
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        // Publish the instruction PC on the bus so the MMIO trace
        // (HLD V7 §4.3) can report it for every access this instruction
        // performs. Set before the fetch so the I-fetch itself is tagged
        // with its own PC.
        bus.set_active_pc(pc);
        let hw0 = bus.read16(pc);

        if is_wide(hw0) {
            let hw1 = bus.read16(pc.wrapping_add(2));
            self.regs.set_pc(pc.wrapping_add(4));
            self.execute_thumb32(hw0, hw1, bus)
        } else {
            self.regs.set_pc(pc.wrapping_add(2));
            self.execute_thumb16(hw0, bus)
        }
    }

    /// Top-level Thumb-16 dispatch. Routes to instruction-group handlers
    /// in execute.rs based on bits [15:11].
    pub(crate) fn execute_thumb16(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
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

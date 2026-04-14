use crate::bus::Bus;
use super::CortexM33;

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

impl CortexM33 {
    /// Fetch, decode, and execute one instruction. Returns cycle count.
    pub(crate) fn decode_execute(&mut self, bus: &mut Bus) -> u32 {
        bus.reset_extra_wait_states();
        let pc = self.regs.pc();
        self.current_instr_addr = pc;
        let hw0 = bus.read16(pc);

        // IT block condition check
        let in_it = self.it_state & 0xF != 0;
        let cond = if in_it {
            (self.it_state >> 4) & 0xF
        } else {
            0xE // AL (always)
        };
        let cond_passed = self.regs.condition_passed(cond);

        if is_wide(hw0) {
            let hw1 = bus.read16(pc.wrapping_add(2));
            self.regs.set_pc(pc.wrapping_add(4));
            let cycles = if cond_passed {
                self.execute_thumb32(hw0, hw1, bus)
            } else {
                1 // skipped instruction costs 1 cycle
            };
            if in_it { self.advance_it_state(); }
            cycles + bus.extra_wait_states()
        } else {
            self.regs.set_pc(pc.wrapping_add(2));

            // Flag suppression for Thumb-16 in IT blocks:
            // Save flags before execution, restore after if needed.
            let saved_flags = if in_it { self.regs.xpsr & 0xF800_0000 } else { 0 };

            let cycles = if cond_passed {
                self.execute_thumb16(hw0, bus)
            } else {
                1
            };

            // Suppress flag changes for Thumb-16 instructions inside IT blocks,
            // EXCEPT for flag-only instructions (CMP, CMN, TST) which always set flags.
            if in_it && cond_passed && !is_thumb16_flag_only(hw0) {
                self.regs.xpsr = (self.regs.xpsr & !0xF800_0000) | saved_flags;
            }

            if in_it { self.advance_it_state(); }
            cycles + bus.extra_wait_states()
        }
    }

    /// Top-level Thumb-16 dispatch. Routes to instruction group handlers
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
            // Data processing + special data + BX
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
            // Miscellaneous
            0b10110 | 0b10111 => self.thumb16_misc(opcode, bus),
            // Store/Load multiple
            0b11000 => self.thumb16_stm(opcode, bus),
            0b11001 => self.thumb16_ldm(opcode, bus),
            // Conditional branch + SVC
            0b11010 | 0b11011 => self.thumb16_cond_branch_svc(opcode, bus),
            // Unconditional branch
            0b11100 => self.thumb16_branch(opcode),
            // 32-bit prefix (should not reach here via this path)
            _ => self.thumb16_undefined(opcode),
        }
    }

    /// Top-level Thumb-32 dispatch.
    pub(crate) fn execute_thumb32(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let op1 = (hw0 >> 11) & 0x3;
        let op2 = ((hw0 >> 4) & 0x7F) as u32;
        let op  = (hw1 >> 15) & 0x1;

        match op1 {
            0b01 => match op2 >> 5 {
                0b00 => if op2 & 0x04 == 0 {
                    self.thumb32_ldm_stm(hw0, hw1, bus)
                } else {
                    self.thumb32_load_store_dual(hw0, hw1, bus)
                },
                0b01 => self.thumb32_dp_shifted_reg(hw0, hw1),
                _    => self.thumb32_coprocessor(hw0, hw1, bus),
            },
            0b10 => if op == 0 {
                if op2 & 0x20 == 0 {
                    self.thumb32_dp_modified_imm(hw0, hw1)
                } else {
                    self.thumb32_dp_plain_imm(hw0, hw1)
                }
            } else {
                self.thumb32_branch_misc(hw0, hw1, bus)
            },
            0b11 => if op2 & 0x40 != 0 {
                self.thumb32_coprocessor(hw0, hw1, bus)
            } else if op2 & 0x20 == 0 {
                self.thumb32_load_store_single(hw0, hw1, bus)
            } else if op2 & 0x10 == 0 {
                self.thumb32_dp_register(hw0, hw1)
            } else if op2 & 0x08 == 0 {
                self.thumb32_multiply(hw0, hw1)
            } else {
                self.thumb32_long_multiply(hw0, hw1)
            },
            _ => self.thumb32_undefined(hw0, hw1),
        }
    }
}

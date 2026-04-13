// Helpers used by stubs once instructions are implemented in later stages.
#![allow(dead_code)]

use crate::bus::Bus;
use super::CortexM33;
use super::execute::{sign_extend, add_with_carry};

// ============================================================================
// ThumbExpandImm helpers
// ============================================================================

#[inline(always)]
pub(crate) fn thumb_expand_imm_c(imm12: u32, carry_in: bool) -> (u32, bool) {
    if imm12 & 0xC00 == 0 {
        // Bits [11:10] = 00: byte replication. Carry unchanged.
        let imm8 = imm12 & 0xFF;
        let val = match (imm12 >> 8) & 0x3 {
            0b00 => imm8,
            0b01 => (imm8 << 16) | imm8,
            0b10 => (imm8 << 24) | (imm8 << 8),
            _    => imm8.wrapping_mul(0x01_01_01_01),
        };
        (val, carry_in)
    } else {
        // Bits [11:10] != 00: rotate (1:imm7) right by imm12[11:7].
        let unrotated = 0x80 | (imm12 & 0x7F);
        let rotation = (imm12 >> 7) & 0x1F;
        let val = unrotated.rotate_right(rotation);
        (val, val >> 31 != 0)
    }
}

#[inline(always)]
pub(crate) fn thumb_expand_imm(imm12: u32) -> u32 {
    thumb_expand_imm_c(imm12, false).0
}

// ============================================================================
// imm12 extraction helper
// ============================================================================

#[inline(always)]
pub(crate) fn extract_imm12(hw0: u16, hw1: u16) -> u32 {
    let i = ((hw0 >> 10) & 1) as u32;
    let imm3 = ((hw1 >> 12) & 0x7) as u32;
    let imm8 = (hw1 & 0xFF) as u32;
    (i << 11) | (imm3 << 8) | imm8
}

// ============================================================================
// Thumb-32 instruction handlers
// ============================================================================

impl CortexM33 {
    // -- Data processing (modified immediate) --------------------------------

    pub(crate) fn thumb32_dp_modified_imm(&mut self, hw0: u16, hw1: u16) -> u32 {
        let op = ((hw0 >> 5) & 0xF) as u8;
        let s = (hw0 >> 4) & 1 != 0;
        let rn = (hw0 & 0xF) as usize;
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let imm12 = extract_imm12(hw0, hw1);
        let (imm32, te_carry) = thumb_expand_imm_c(imm12, self.regs.flag_c());

        match op {
            // AND / TST / ANDS
            0b0000 => {
                let result = self.regs.r[rn] & imm32;
                if s && rd == 15 {
                    // TST — discard result, update flags only
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(te_carry);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nz(result);
                        self.regs.set_flag_c(te_carry);
                    }
                }
                1
            }
            // BIC / BICS
            0b0001 => {
                let result = self.regs.r[rn] & !imm32;
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(te_carry);
                }
                1
            }
            // ORR / MOV / ORRS / MOVS
            0b0010 => {
                let result = if rn == 15 {
                    imm32 // MOV / MOVS
                } else {
                    self.regs.r[rn] | imm32
                };
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(te_carry);
                }
                1
            }
            // ORN / MVN / ORNS / MVNS
            0b0011 => {
                let result = if rn == 15 {
                    !imm32 // MVN / MVNS
                } else {
                    self.regs.r[rn] | !imm32
                };
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(te_carry);
                }
                1
            }
            // EOR / TEQ / EORS
            0b0100 => {
                let result = self.regs.r[rn] ^ imm32;
                if s && rd == 15 {
                    // TEQ — discard result, update flags only
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(te_carry);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nz(result);
                        self.regs.set_flag_c(te_carry);
                    }
                }
                1
            }
            // ADD / CMN / ADDS
            0b1000 => {
                let (result, carry, overflow) = add_with_carry(self.regs.r[rn], imm32, false);
                if s && rd == 15 {
                    // CMN — discard result, update flags only
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                    }
                }
                1
            }
            // ADC / ADCS
            0b1010 => {
                let (result, carry, overflow) =
                    add_with_carry(self.regs.r[rn], imm32, self.regs.flag_c());
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1
            }
            // SBC / SBCS
            0b1011 => {
                let (result, carry, overflow) =
                    add_with_carry(self.regs.r[rn], !imm32, self.regs.flag_c());
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1
            }
            // SUB / CMP / SUBS
            0b1101 => {
                let (result, carry, overflow) = add_with_carry(self.regs.r[rn], !imm32, true);
                if s && rd == 15 {
                    // CMP — discard result, update flags only
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                    }
                }
                1
            }
            // RSB / RSBS
            0b1110 => {
                let (result, carry, overflow) = add_with_carry(!self.regs.r[rn], imm32, true);
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1
            }
            // Undefined op values
            _ => self.thumb32_undefined(hw0, hw1),
        }
    }

    // -- Data processing (plain binary immediate) ----------------------------

    pub(crate) fn thumb32_dp_plain_imm(&mut self, hw0: u16, hw1: u16) -> u32 {
        let op = ((hw0 >> 4) & 0x1F) as u8;
        let rn = (hw0 & 0xF) as usize;
        let rd = ((hw1 >> 8) & 0xF) as usize;

        match op {
            // ADDW / ADR (add variant)
            0b00000 => {
                let imm12 = extract_imm12(hw0, hw1);
                if rn == 15 {
                    // ADR: Rd = Align(PC, 4) + imm12
                    self.regs.r[rd] = (self.read_pc() & !3).wrapping_add(imm12);
                } else {
                    self.regs.r[rd] = self.regs.r[rn].wrapping_add(imm12);
                }
                1
            }
            // MOVW
            0b00100 => {
                let imm16 = ((hw0 as u32 & 0xF) << 12)
                    | (((hw0 >> 10) as u32 & 1) << 11)
                    | (((hw1 >> 12) as u32 & 0x7) << 8)
                    | (hw1 as u32 & 0xFF);
                self.regs.r[rd] = imm16;
                1
            }
            // SUBW / ADR (sub variant)
            0b01010 => {
                let imm12 = extract_imm12(hw0, hw1);
                if rn == 15 {
                    // ADR: Rd = Align(PC, 4) - imm12
                    self.regs.r[rd] = (self.read_pc() & !3).wrapping_sub(imm12);
                } else {
                    self.regs.r[rd] = self.regs.r[rn].wrapping_sub(imm12);
                }
                1
            }
            // MOVT
            0b01100 => {
                let imm16 = ((hw0 as u32 & 0xF) << 12)
                    | (((hw0 >> 10) as u32 & 1) << 11)
                    | (((hw1 >> 12) as u32 & 0x7) << 8)
                    | (hw1 as u32 & 0xFF);
                self.regs.r[rd] = (self.regs.r[rd] & 0xFFFF) | (imm16 << 16);
                1
            }
            // SSAT — stub
            0b10000 => self.thumb32_undefined(hw0, hw1),
            // SBFX
            0b10100 => {
                let lsb = (((hw1 >> 12) & 0x7) << 2 | ((hw1 >> 6) & 0x3)) as u32;
                let widthm1 = (hw1 & 0x1F) as u32;
                let width = widthm1 + 1;
                let val = (self.regs.r[rn] >> lsb) & ((1u32 << width) - 1);
                self.regs.r[rd] = sign_extend(val, width);
                1
            }
            // BFI / BFC
            0b10110 => {
                let lsb = (((hw1 >> 12) & 0x7) << 2 | ((hw1 >> 6) & 0x3)) as u32;
                let msb = (hw1 & 0x1F) as u32;
                let width = msb - lsb + 1;
                let mask = ((1u32 << width) - 1) << lsb;
                if rn == 15 {
                    // BFC: clear bits
                    self.regs.r[rd] = self.regs.r[rd] & !mask;
                } else {
                    // BFI: insert bits from Rn
                    self.regs.r[rd] = (self.regs.r[rd] & !mask)
                        | ((self.regs.r[rn] << lsb) & mask);
                }
                1
            }
            // USAT — stub
            0b11000 => self.thumb32_undefined(hw0, hw1),
            // UBFX
            0b11100 => {
                let lsb = (((hw1 >> 12) & 0x7) << 2 | ((hw1 >> 6) & 0x3)) as u32;
                let widthm1 = (hw1 & 0x1F) as u32;
                let width = widthm1 + 1;
                self.regs.r[rd] = (self.regs.r[rn] >> lsb) & ((1u32 << width) - 1);
                1
            }
            // Undefined
            _ => self.thumb32_undefined(hw0, hw1),
        }
    }

    // -- Data processing (shifted register) ----------------------------------

    pub(crate) fn thumb32_dp_shifted_reg(&mut self, hw0: u16, hw1: u16) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- Load/store single ---------------------------------------------------

    pub(crate) fn thumb32_load_store_single(&mut self, hw0: u16, hw1: u16, _bus: &mut Bus) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- Load/store multiple -------------------------------------------------

    pub(crate) fn thumb32_ldm_stm(&mut self, hw0: u16, hw1: u16, _bus: &mut Bus) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- Load/store dual, exclusive, table branch ----------------------------

    pub(crate) fn thumb32_load_store_dual(&mut self, hw0: u16, hw1: u16, _bus: &mut Bus) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- Branches and miscellaneous control ----------------------------------

    pub(crate) fn thumb32_branch_misc(&mut self, hw0: u16, hw1: u16, _bus: &mut Bus) -> u32 {
        // Sub-dispatch per LLD Section 5.7
        if hw1 & (1 << 14) != 0 {
            // hw1[14] = 1 -> BL
            self.thumb32_bl(hw0, hw1)
        } else if hw1 & (1 << 12) != 0 {
            // hw1[14] = 0, hw1[12] = 1 -> B.W T4 (unconditional)
            self.thumb32_undefined(hw0, hw1)
        } else {
            // hw1[14] = 0, hw1[12] = 0
            let misc_op = (hw0 >> 6) & 0xF;
            if misc_op & 0xE != 0xE {
                // hw0[9:6] != 0b111x -> B.W T3 (conditional)
                self.thumb32_undefined(hw0, hw1)
            } else {
                // hw0[9:6] == 0b111x -> miscellaneous control (MSR, MRS, hints, barriers)
                self.thumb32_undefined(hw0, hw1)
            }
        }
    }

    // -- Multiply (32-bit result) --------------------------------------------

    pub(crate) fn thumb32_multiply(&mut self, hw0: u16, hw1: u16) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- Long multiply / divide (64-bit result) ------------------------------

    pub(crate) fn thumb32_long_multiply(&mut self, hw0: u16, hw1: u16) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- Data processing (register) ------------------------------------------

    pub(crate) fn thumb32_dp_register(&mut self, hw0: u16, hw1: u16) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- Coprocessor ---------------------------------------------------------

    pub(crate) fn thumb32_coprocessor(&mut self, hw0: u16, hw1: u16, _bus: &mut Bus) -> u32 {
        self.thumb32_undefined(hw0, hw1)
    }

    // -- BL (branch with link) -----------------------------------------------

    pub(crate) fn thumb32_bl(&mut self, hw0: u16, hw1: u16) -> u32 {
        let s = ((hw0 >> 10) & 1) as u32;
        let imm10 = (hw0 & 0x3FF) as u32;
        let j1 = ((hw1 >> 13) & 1) as u32;
        let j2 = ((hw1 >> 11) & 1) as u32;
        let imm11 = (hw1 & 0x7FF) as u32;

        // I1 = NOT(J1 XOR S), I2 = NOT(J2 XOR S)
        let i1 = (j1 ^ s) ^ 1;
        let i2 = (j2 ^ s) ^ 1;

        // imm32 = SignExtend(S:I1:I2:imm10:imm11:0, 25)
        let imm25 = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
        let offset = sign_extend(imm25, 25);

        // LR = address of next instruction | 1 (Thumb bit)
        let next_instr = self.regs.pc() | 1;
        self.regs.set_lr(next_instr);

        // PC = PC + offset (PC here is the read_pc value = instr_addr + 4)
        let target = self.read_pc().wrapping_add(offset);
        self.regs.set_pc(target);
        4
    }

    // -- Undefined 32-bit instruction ----------------------------------------

    pub(crate) fn thumb32_undefined(&mut self, _hw0: u16, _hw1: u16) -> u32 {
        // TODO: raise UsageFault (Phase 3)
        1
    }
}

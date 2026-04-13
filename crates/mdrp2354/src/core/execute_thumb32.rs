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

    pub(crate) fn thumb32_load_store_single(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let size = ((hw0 >> 5) & 0x3) as u8;    // hw0[6:5]: 00=byte, 01=half, 10=word
        let load = (hw0 >> 4) & 1 != 0;         // hw0[4]: 1=load, 0=store
        let sign = (hw0 >> 8) & 1 != 0;         // hw0[8]: 1=signed load
        let rn = (hw0 & 0xF) as usize;          // hw0[3:0], 15=PC-relative
        let rt = ((hw1 >> 12) & 0xF) as usize;  // hw1[15:12]

        // GUARD: load with Rt=15 is PLD/PLI (preload hint), NOT a load to PC.
        if load && rt == 15 {
            return 1;
        }

        // Compute effective address
        let addr = if rn == 15 {
            // PC-relative literal load
            let base = self.read_pc() & !3; // word-aligned PC
            let u = (hw0 >> 7) & 1 != 0;
            let imm12 = (hw1 & 0xFFF) as u32;
            if u { base.wrapping_add(imm12) } else { base.wrapping_sub(imm12) }
        } else if (hw0 >> 7) & 1 != 0 {
            // Immediate 12-bit unsigned offset
            let imm12 = (hw1 & 0xFFF) as u32;
            self.regs.r[rn].wrapping_add(imm12)
        } else if hw1 & 0x800 != 0 {
            // 8-bit immediate with P/U/W
            let p = (hw1 >> 10) & 1 != 0;
            let u = (hw1 >> 9) & 1 != 0;
            let w = (hw1 >> 8) & 1 != 0;
            let imm8 = (hw1 & 0xFF) as u32;
            let offset = if u { imm8 } else { 0u32.wrapping_sub(imm8) };
            let base = self.regs.r[rn];
            let addr = if p { base.wrapping_add(offset) } else { base };

            // Perform the memory access before writeback
            let cycles = self.thumb32_ls_single_access(size, sign, load, rt, addr, bus);

            // Writeback: pre-index (p=true, w=true) or post-index (p=false)
            if w || !p {
                self.regs.r[rn] = base.wrapping_add(offset);
            }
            return cycles;
        } else {
            // Register offset with LSL
            let shift = ((hw1 >> 4) & 0x3) as u32;
            let rm = (hw1 & 0xF) as usize;
            let offset = self.regs.r[rm] << shift;
            self.regs.r[rn].wrapping_add(offset)
        };

        self.thumb32_ls_single_access(size, sign, load, rt, addr, bus)
    }

    /// Perform a single load/store memory access by size and sign.
    /// Returns cycle count: load=2, store=1, undefined=1.
    #[inline(always)]
    fn thumb32_ls_single_access(
        &mut self, size: u8, sign: bool, load: bool,
        rt: usize, addr: u32, bus: &mut Bus,
    ) -> u32 {
        match (size, sign) {
            (0b00, false) => {
                if load { self.regs.r[rt] = bus.read8(addr) as u32; }
                else { bus.write8(addr, self.regs.r[rt] as u8); }
            }
            (0b00, true) => {
                // LDRSB (load only; signed stores don't exist)
                self.regs.r[rt] = bus.read8(addr) as i8 as i32 as u32;
            }
            (0b01, false) => {
                if load { self.regs.r[rt] = bus.read16(addr) as u32; }
                else { bus.write16(addr, self.regs.r[rt] as u16); }
            }
            (0b01, true) => {
                // LDRSH (load only)
                self.regs.r[rt] = bus.read16(addr) as i16 as i32 as u32;
            }
            (0b10, false) => {
                if load { self.regs.r[rt] = bus.read32(addr); }
                else { bus.write32(addr, self.regs.r[rt]); }
            }
            _ => return 1, // undefined: signed word or size=11
        }
        if load { 2 } else { 1 }
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
            self.thumb32_b_w_uncond(hw0, hw1)
        } else {
            // hw1[14] = 0, hw1[12] = 0
            let misc_op = (hw0 >> 6) & 0xF;
            if misc_op & 0xE != 0xE {
                // hw0[9:6] != 0b111x -> B.W T3 (conditional)
                self.thumb32_b_w_cond(hw0, hw1)
            } else {
                // hw0[9:6] == 0b111x -> miscellaneous control
                self.thumb32_misc_control(hw0, hw1)
            }
        }
    }

    // -- B.W conditional (T3) ---------------------------------------------------

    fn thumb32_b_w_cond(&mut self, hw0: u16, hw1: u16) -> u32 {
        let s = ((hw0 >> 10) & 1) as u32;
        let cond = ((hw0 >> 6) & 0xF) as u8;
        let imm6 = (hw0 & 0x3F) as u32;
        let j1 = ((hw1 >> 13) & 1) as u32;
        let j2 = ((hw1 >> 11) & 1) as u32;
        let imm11 = (hw1 & 0x7FF) as u32;

        // J1/J2 used directly (no XOR trick for T3)
        let imm21 = (s << 20) | (j2 << 19) | (j1 << 18) | (imm6 << 12) | (imm11 << 1);
        let offset = sign_extend(imm21, 21);

        if self.regs.condition_passed(cond) {
            let target = self.read_pc().wrapping_add(offset);
            self.regs.set_pc(target);
            2
        } else {
            1
        }
    }

    // -- B.W unconditional (T4) -------------------------------------------------

    fn thumb32_b_w_uncond(&mut self, hw0: u16, hw1: u16) -> u32 {
        let s = ((hw0 >> 10) & 1) as u32;
        let imm10 = (hw0 & 0x3FF) as u32;
        let j1 = ((hw1 >> 13) & 1) as u32;
        let j2 = ((hw1 >> 11) & 1) as u32;
        let imm11 = (hw1 & 0x7FF) as u32;

        // XOR trick for extended range
        let i1 = (j1 ^ s) ^ 1;
        let i2 = (j2 ^ s) ^ 1;

        let imm25 = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
        let offset = sign_extend(imm25, 25);

        let target = self.read_pc().wrapping_add(offset);
        self.regs.set_pc(target);
        2
    }

    // -- Miscellaneous control (MSR, MRS, hints, barriers) ----------------------

    fn thumb32_misc_control(&mut self, hw0: u16, hw1: u16) -> u32 {
        // Hints: hw0 = 0xF3AF
        if hw0 == 0xF3AF {
            let hint = hw1 & 0xFF;
            return match hint {
                0x00 => 1, // NOP.W
                0x01 => 1, // YIELD.W
                0x02 => 1, // WFE.W
                0x03 => 1, // WFI.W
                0x04 => 1, // SEV.W
                _ => self.thumb32_undefined(hw0, hw1),
            };
        }

        // Barriers: hw0 = 0xF3BF
        if hw0 == 0xF3BF {
            let barrier_op = (hw1 >> 4) & 0xF;
            return match barrier_op {
                0x4 => 1, // DSB
                0x5 => 1, // DMB
                0x6 => 1, // ISB
                _ => self.thumb32_undefined(hw0, hw1),
            };
        }

        // MSR: hw0[10:4] = 0b0111000 or 0b0111001
        let op_field = (hw0 >> 4) & 0x7F;
        if op_field == 0b0111000 || op_field == 0b0111001 {
            // MSR -- stub for Stage 10
            return 1;
        }

        // MRS: hw0[10:4] = 0b0111110 or 0b0111111
        if op_field == 0b0111110 || op_field == 0b0111111 {
            // MRS -- stub for Stage 10
            return 1;
        }

        self.thumb32_undefined(hw0, hw1)
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

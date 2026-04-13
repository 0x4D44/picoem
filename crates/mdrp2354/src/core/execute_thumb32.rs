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
// Barrel shift helper (for shifted-register instructions)
// ============================================================================

/// Apply an immediate-specified barrel shift to a value.
/// shift_type: 00=LSL, 01=LSR, 10=ASR, 11=ROR (with amount=0 meaning RRX).
/// Returns (shifted_value, carry_out).
#[inline(always)]
pub(crate) fn barrel_shift(val: u32, shift_type: u8, amount: u32, carry_in: bool) -> (u32, bool) {
    match shift_type {
        0b00 => {
            // LSL
            if amount == 0 {
                (val, carry_in)
            } else {
                (val << amount, (val >> (32 - amount)) & 1 != 0)
            }
        }
        0b01 => {
            // LSR: amount=0 encodes LSR #32
            if amount == 0 {
                (0, val >> 31 != 0)
            } else {
                (val >> amount, (val >> (amount - 1)) & 1 != 0)
            }
        }
        0b10 => {
            // ASR: amount=0 encodes ASR #32
            let sv = val as i32;
            if amount == 0 {
                ((sv >> 31) as u32, sv < 0)
            } else {
                ((sv >> amount) as u32, (sv >> (amount as i32 - 1)) & 1 != 0)
            }
        }
        _ => {
            // ROR: amount=0 encodes RRX (rotate right through carry by 1)
            if amount == 0 {
                // RRX: (carry_in << 31) | (val >> 1), carry_out = bit[0]
                let result = ((carry_in as u32) << 31) | (val >> 1);
                (result, val & 1 != 0)
            } else {
                let result = val.rotate_right(amount);
                (result, (val >> (amount - 1)) & 1 != 0)
            }
        }
    }
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
                1 // M33 measured: 1 cycle
            }
            // MOVW
            0b00100 => {
                let imm16 = ((hw0 as u32 & 0xF) << 12)
                    | (((hw0 >> 10) as u32 & 1) << 11)
                    | (((hw1 >> 12) as u32 & 0x7) << 8)
                    | (hw1 as u32 & 0xFF);
                self.regs.r[rd] = imm16;
                1 // M33 measured: 1 cycle
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
                1 // M33 measured: 1 cycle
            }
            // MOVT
            0b01100 => {
                let imm16 = ((hw0 as u32 & 0xF) << 12)
                    | (((hw0 >> 10) as u32 & 1) << 11)
                    | (((hw1 >> 12) as u32 & 0x7) << 8)
                    | (hw1 as u32 & 0xFF);
                self.regs.r[rd] = (self.regs.r[rd] & 0xFFFF) | (imm16 << 16);
                1 // M33 measured: 1 cycle
            }
            // SSAT — signed saturate (LSL: op=0b10000, ASR: op=0b10010)
            0b10000 | 0b10010 => {
                let sat_bit = ((hw1 & 0x1F) + 1) as u32;
                let sh = (op >> 1) & 1; // 0=LSL, 1=ASR
                let shift_type = if sh != 0 { 0b10u8 } else { 0b00u8 };
                let shift_n = ((((hw1 >> 12) & 0x7) << 2) | ((hw1 >> 6) & 0x3)) as u32;
                let shifted = barrel_shift(self.regs.r[rn], shift_type, shift_n, false).0;
                let signed_val = shifted as i32;
                let max = (1i32 << (sat_bit - 1)) - 1;
                let min = -(1i32 << (sat_bit - 1));
                let result = if signed_val > max {
                    self.regs.set_flag_q();
                    max as u32
                } else if signed_val < min {
                    self.regs.set_flag_q();
                    min as u32
                } else {
                    shifted
                };
                self.regs.r[rd] = result;
                1 // M33 measured: 1 cycle
            }
            // SBFX
            0b10100 => {
                let lsb = (((hw1 >> 12) & 0x7) << 2 | ((hw1 >> 6) & 0x3)) as u32;
                let widthm1 = (hw1 & 0x1F) as u32;
                let width = widthm1 + 1;
                let val = (self.regs.r[rn] >> lsb) & ((1u32 << width) - 1);
                self.regs.r[rd] = sign_extend(val, width);
                1 // M33 measured: 1 cycle
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
                1 // M33 measured: 1 cycle
            }
            // USAT — unsigned saturate (LSL: op=0b11000, ASR: op=0b11010)
            0b11000 | 0b11010 => {
                let sat_bit = (hw1 & 0x1F) as u32;
                let sh = (op >> 1) & 1;
                let shift_type = if sh != 0 { 0b10u8 } else { 0b00u8 };
                let shift_n = ((((hw1 >> 12) & 0x7) << 2) | ((hw1 >> 6) & 0x3)) as u32;
                let shifted = barrel_shift(self.regs.r[rn], shift_type, shift_n, false).0;
                let signed_val = shifted as i32;
                let max = if sat_bit < 32 { (1i64 << sat_bit) - 1 } else { i64::from(i32::MAX) };
                let result = if signed_val < 0 {
                    self.regs.set_flag_q();
                    0u32
                } else if (signed_val as i64) > max {
                    self.regs.set_flag_q();
                    max as u32
                } else {
                    shifted
                };
                self.regs.r[rd] = result;
                1 // M33 measured: 1 cycle
            }
            // UBFX
            0b11100 => {
                let lsb = (((hw1 >> 12) & 0x7) << 2 | ((hw1 >> 6) & 0x3)) as u32;
                let widthm1 = (hw1 & 0x1F) as u32;
                let width = widthm1 + 1;
                self.regs.r[rd] = (self.regs.r[rn] >> lsb) & ((1u32 << width) - 1);
                1 // M33 measured: 1 cycle
            }
            // Undefined
            _ => self.thumb32_undefined(hw0, hw1),
        }
    }

    // -- Data processing (shifted register) ----------------------------------

    pub(crate) fn thumb32_dp_shifted_reg(&mut self, hw0: u16, hw1: u16) -> u32 {
        let op = ((hw0 >> 5) & 0xF) as u8;
        let s = (hw0 >> 4) & 1 != 0;
        let rn = (hw0 & 0xF) as usize;
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let rm = (hw1 & 0xF) as usize;
        let shift_type = ((hw1 >> 4) & 0x3) as u8;
        let shift_n = (((hw1 >> 12) & 0x7) << 2 | ((hw1 >> 6) & 0x3)) as u32;

        let (shifted, shift_carry) =
            barrel_shift(self.regs.r[rm], shift_type, shift_n, self.regs.flag_c());

        match op {
            // AND / TST
            0b0000 => {
                let result = self.regs.r[rn] & shifted;
                if s && rd == 15 {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(shift_carry);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nz(result);
                        self.regs.set_flag_c(shift_carry);
                    }
                }
                1 // M33: 1 cycle
            }
            // BIC
            0b0001 => {
                let result = self.regs.r[rn] & !shifted;
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(shift_carry);
                }
                1 // M33: 1 cycle
            }
            // ORR / MOV (Rn=15)
            0b0010 => {
                let result = if rn == 15 {
                    shifted // MOV.W / shift-by-immediate
                } else {
                    self.regs.r[rn] | shifted
                };
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(shift_carry);
                }
                1 // M33: 1 cycle
            }
            // ORN / MVN (Rn=15)
            0b0011 => {
                let result = if rn == 15 {
                    !shifted
                } else {
                    self.regs.r[rn] | !shifted
                };
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(shift_carry);
                }
                1 // M33: 1 cycle
            }
            // EOR / TEQ
            0b0100 => {
                let result = self.regs.r[rn] ^ shifted;
                if s && rd == 15 {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(shift_carry);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nz(result);
                        self.regs.set_flag_c(shift_carry);
                    }
                }
                1 // M33: 1 cycle
            }
            // ADD / CMN
            0b1000 => {
                let (result, carry, overflow) = add_with_carry(self.regs.r[rn], shifted, false);
                if s && rd == 15 {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                    }
                }
                1 // M33: 1 cycle
            }
            // ADC
            0b1010 => {
                let (result, carry, overflow) =
                    add_with_carry(self.regs.r[rn], shifted, self.regs.flag_c());
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1 // M33: 1 cycle
            }
            // SBC
            0b1011 => {
                let (result, carry, overflow) =
                    add_with_carry(self.regs.r[rn], !shifted, self.regs.flag_c());
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1 // M33: 1 cycle
            }
            // SUB / CMP
            0b1101 => {
                let (result, carry, overflow) = add_with_carry(self.regs.r[rn], !shifted, true);
                if s && rd == 15 {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                } else {
                    self.regs.r[rd] = result;
                    if s {
                        self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                    }
                }
                1 // M33: 1 cycle
            }
            // RSB
            0b1110 => {
                let (result, carry, overflow) = add_with_carry(!self.regs.r[rn], shifted, true);
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1 // M33: 1 cycle
            }
            _ => self.thumb32_undefined(hw0, hw1),
        }
    }

    // -- Load/store single ---------------------------------------------------

    pub(crate) fn thumb32_load_store_single(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let size = ((hw0 >> 5) & 0x3) as u8;    // hw0[6:5]: 00=byte, 01=half, 10=word
        let load = (hw0 >> 4) & 1 != 0;         // hw0[4]: 1=load, 0=store
        let sign = (hw0 >> 8) & 1 != 0;         // hw0[8]: 1=signed load
        let rn = (hw0 & 0xF) as usize;          // hw0[3:0], 15=PC-relative
        let rt = ((hw1 >> 12) & 0xF) as usize;  // hw1[15:12]

        // PLD/PLI: byte or halfword load with Rt=15 is a preload hint, not a real load.
        // Word-size loads with Rt=15 are real PC loads (LDR.W PC, [...]).
        if load && rt == 15 && size != 0b10 {
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
    /// Returns cycle count: load=2, store=2, undefined=1 (M33 measured).
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
                if load {
                    let val = bus.read32(addr);
                    if rt == 15 {
                        if Self::is_exc_return(val) {
                            return self.exit_exception(val, bus);
                        }
                        self.regs.set_pc(val & !1);
                        return 5; // load + pipeline flush
                    }
                    self.regs.r[rt] = val;
                } else {
                    bus.write32(addr, self.regs.r[rt]);
                }
            }
            _ => return 1, // undefined: signed word or size=11
        }
        if load { 2 } else { 2 } // M33 measured: 2 cycles (SRAM, zero-wait-state)
    }

    // -- Load/store multiple -------------------------------------------------

    pub(crate) fn thumb32_ldm_stm(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let w = (hw0 >> 5) & 1 != 0;
        let load = (hw0 >> 4) & 1 != 0;
        let rn = (hw0 & 0xF) as usize;
        let reglist = hw1 as u32;
        let count = reglist.count_ones();

        // Direction: IA (01) or DB (10)
        let op = (hw0 >> 7) & 0x3;
        let mut addr = match op {
            0b01 => self.regs.r[rn],                                // IA: start at Rn
            0b10 => self.regs.r[rn].wrapping_sub(count * 4),       // DB: start at Rn - 4*count
            _ => return self.thumb32_undefined(hw0, hw1),
        };

        bus.set_burst_mode();
        for i in 0..16 {
            if reglist & (1 << i) != 0 {
                if load {
                    let val = bus.read32(addr);
                    if i == 15 {
                        if Self::is_exc_return(val) {
                            bus.clear_burst_mode();
                            return self.exit_exception(val, bus);
                        }
                        self.regs.set_pc(val & !1);
                    } else {
                        self.regs.r[i] = val;
                    }
                } else {
                    bus.write32(addr, self.regs.r[i]);
                }
                addr = addr.wrapping_add(4);
            }
        }
        bus.clear_burst_mode();

        // Writeback: if W set AND (for loads) Rn is NOT in reglist
        if w && (!load || reglist & (1 << rn) == 0) {
            self.regs.r[rn] = match op {
                0b01 => self.regs.r[rn].wrapping_add(count * 4),   // IA: Rn + 4*count
                0b10 => self.regs.r[rn].wrapping_sub(count * 4),   // DB: Rn - 4*count
                _ => unreachable!(),
            };
        }

        // Cost: 1 + N, plus 3 extra if PC was loaded
        let pc_loaded = load && reglist & (1 << 15) != 0;
        1 + count + if pc_loaded { 3 } else { 0 }
    }

    // -- Load/store dual, exclusive, table branch ----------------------------

    pub(crate) fn thumb32_load_store_dual(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        // SG (Secure Gateway): encoding 0xE97F_E97F.
        // When executed from Non-Secure state, transitions to Secure and clears
        // LR bit 0 to mark the return address as Non-Secure.
        if hw0 == 0xE97F && hw1 == 0xE97F {
            if !self.secure {
                self.transition_to_secure();
                // Clear bit 0 of LR to mark return address as NS.
                let lr = self.regs.lr() & !1;
                self.regs.set_lr(lr);
            }
            // In Secure state, SG is a NOP.
            return 1;
        }

        // TBB/TBH: hw0 = 1110_1000_1101_Rn (hw0[7:4]=1101), hw1[15:12]=1111, hw1[7:5]=000
        if hw0 & 0xFFF0 == 0xE8D0 && (hw1 >> 12) & 0xF == 0xF && (hw1 >> 5) & 0x7 == 0 {
            let rn = (hw0 & 0xF) as usize;
            let rm = (hw1 & 0xF) as usize;
            let h = (hw1 >> 4) & 1 != 0;
            let base = self.regs.r[rn];
            if h {
                let halfword = bus.read16(base.wrapping_add(self.regs.r[rm] << 1));
                self.regs.set_pc(self.read_pc().wrapping_add((halfword as u32) << 1));
            } else {
                let byte = bus.read8(base.wrapping_add(self.regs.r[rm]));
                self.regs.set_pc(self.read_pc().wrapping_add((byte as u32) << 1));
            }
            return 4;
        }

        // LDREX: hw0 = 1110_1000_0101_Rn (0xE85x)
        // STREX: hw0 = 1110_1000_0100_Rn (0xE84x)
        if hw0 & 0xFFF0 == 0xE850 {
            // LDREX (treat as normal LDR for Phase 1)
            let rn = (hw0 & 0xF) as usize;
            let rt = ((hw1 >> 12) & 0xF) as usize;
            let imm8 = (hw1 & 0xFF) as u32;
            let addr = self.regs.r[rn].wrapping_add(imm8 << 2);
            self.regs.r[rt] = bus.read32(addr);
            return 2;
        }
        if hw0 & 0xFFF0 == 0xE840 {
            // TT family: hw1[15:12]=0xF, hw1[7:0]=0x00
            if (hw1 >> 12) & 0xF == 0xF && hw1 & 0xFF == 0x00 {
                let rn = (hw0 & 0xF) as usize;
                let rd = ((hw1 >> 8) & 0xF) as usize;
                let addr = self.regs.r[rn];
                self.regs.r[rd] = Self::execute_tt(addr, bus);
                return 1;
            }
            // STREX (treat as normal STR for Phase 1, Rd gets 0 = success)
            let rn = (hw0 & 0xF) as usize;
            let rt = ((hw1 >> 12) & 0xF) as usize;
            let rd = ((hw1 >> 8) & 0xF) as usize;
            let imm8 = (hw1 & 0xFF) as u32;
            let addr = self.regs.r[rn].wrapping_add(imm8 << 2);
            bus.write32(addr, self.regs.r[rt]);
            self.regs.r[rd] = 0; // success
            return 2;
        }

        // LDREXB/LDREXH/STREXB/STREXH: hw0 = 0xE8Cx or 0xE8Dx patterns
        // Phase 1: treat as normal load/store variants
        // (Falls through to LDRD/STRD for any other unrecognized pattern)

        // LDRD/STRD (immediate): default path
        let p = (hw0 >> 8) & 1 != 0;
        let u = (hw0 >> 7) & 1 != 0;
        let w = (hw0 >> 5) & 1 != 0;
        let load = (hw0 >> 4) & 1 != 0;
        let rn = (hw0 & 0xF) as usize;
        let rt = ((hw1 >> 12) & 0xF) as usize;
        let rt2 = ((hw1 >> 8) & 0xF) as usize;
        let imm8 = (hw1 & 0xFF) as u32;
        let offset = imm8 << 2;

        let base = if rn == 15 { self.read_pc() & !3 } else { self.regs.r[rn] };
        let offset_addr = if u { base.wrapping_add(offset) } else { base.wrapping_sub(offset) };
        let addr = if p { offset_addr } else { base };

        if load {
            self.regs.r[rt] = bus.read32(addr);
            self.regs.r[rt2] = bus.read32(addr.wrapping_add(4));
        } else {
            bus.write32(addr, self.regs.r[rt]);
            bus.write32(addr.wrapping_add(4), self.regs.r[rt2]);
        }

        if w && rn != 15 {
            self.regs.r[rn] = offset_addr;
        }

        4 // M33 measured: 4 cycles (two word transfers)
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
            1 // M33 measured: 1 cycle
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
        1 // M33 measured: 1 cycle
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
                0x2 => 1, // CLREX
                0x4 => 1, // DSB
                0x5 => 1, // DMB
                0x6 => 1, // ISB
                _ => self.thumb32_undefined(hw0, hw1),
            };
        }

        // MSR: hw0[10:4] = 0b0111000 or 0b0111001
        let op_field = (hw0 >> 4) & 0x7F;
        if op_field == 0b0111000 || op_field == 0b0111001 {
            return self.thumb32_msr(hw0, hw1);
        }

        // MRS: hw0[10:4] = 0b0111110 or 0b0111111
        if op_field == 0b0111110 || op_field == 0b0111111 {
            return self.thumb32_mrs(hw1);
        }

        self.thumb32_undefined(hw0, hw1)
    }

    /// MSR — write a general-purpose register to a special system register.
    /// Encoding: 11110_0111_00_R_Rn  10_00_mask_00_SYSm
    fn thumb32_msr(&mut self, hw0: u16, hw1: u16) -> u32 {
        let rn = (hw0 & 0xF) as usize;
        let sysm = (hw1 & 0xFF) as u8;
        let mask = ((hw1 >> 10) & 0x3) as u8;
        let val = self.regs.r[rn];

        match sysm {
            // APSR — write NZCVQ flags (mask[1] controls NZCVQ group)
            0 | 1 | 2 | 3 | 4 => {
                if mask & 2 != 0 {
                    self.regs.xpsr = (self.regs.xpsr & !0xF800_0000) | (val & 0xF800_0000);
                }
                if mask & 1 != 0 {
                    self.regs.set_ge_flags((val >> 16) & 0xF);
                }
            }
            // IPSR (5), EPSR (6), IEPSR (7) — read-only, ignore writes
            5 | 6 | 7 => {}
            // MSP
            8 => {
                self.regs.msp = val;
                if !self.regs.active_sp_is_psp() {
                    self.regs.r[13] = val;
                }
            }
            // PSP
            9 => {
                self.regs.psp = val;
                if self.regs.active_sp_is_psp() {
                    self.regs.r[13] = val;
                }
            }
            // MSPLIM
            10 => self.regs.msplim = val & !0x7, // 8-byte aligned
            // PSPLIM
            11 => self.regs.psplim = val & !0x7, // 8-byte aligned
            // PRIMASK
            16 => {
                self.regs.primask = val & 1;
            }
            // BASEPRI
            17 => {
                self.regs.basepri = val & 0xFF;
            }
            // BASEPRI_MAX — only lowers (numerically) the priority ceiling
            18 => {
                if val & 0xFF != 0
                    && ((val & 0xFF) < self.regs.basepri || self.regs.basepri == 0)
                {
                    self.regs.basepri = val & 0xFF;
                }
            }
            // FAULTMASK
            19 => {
                self.regs.faultmask = val & 1;
            }
            // CONTROL — nPRIV, SPSEL, FPCA; must sync SP around the switch
            20 => {
                self.regs.sync_sp_to_banked();
                self.regs.control = val & 0x7;
                self.regs.sync_sp_from_banked();
            }
            // --- Non-Secure banked registers (ARMv8-M, SYSm bit 7 = NS) ---
            // Accessible from Secure state; we treat everything as Secure.
            0x88 => self.regs.msp_ns = val,            // MSP_NS
            0x89 => self.regs.psp_ns = val,            // PSP_NS
            0x8A => self.regs.msplim_ns = val & !0x7,  // MSPLIM_NS
            0x8B => self.regs.psplim_ns = val & !0x7,  // PSPLIM_NS
            0x90 => self.regs.primask_ns = val & 1,     // PRIMASK_NS
            0x91 => self.regs.basepri_ns = val & 0xFF,  // BASEPRI_NS
            0x93 => self.regs.faultmask_ns = val & 1,   // FAULTMASK_NS
            0x94 => self.regs.control_ns = val & 0x7,   // CONTROL_NS
            _ => {} // reserved — ignore
        }
        1 // M33 measured: 1 cycle
    }

    /// MRS — read a special system register into a general-purpose register.
    /// Encoding: 11110_0111_11_R_1111  10_00_Rd_SYSm
    fn thumb32_mrs(&mut self, hw1: u16) -> u32 {
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let sysm = (hw1 & 0xFF) as u8;

        self.regs.r[rd] = match sysm {
            // APSR / IAPSR / EAPSR / XPSR / combined variants — NZCVQ flags
            0 | 1 | 2 | 3 | 4 => self.regs.xpsr & 0xF80F_0000,
            // IPSR — exception number
            5 => self.regs.xpsr & 0x1FF,
            // EPSR — execution state not readable
            6 => 0,
            // IEPSR — IPSR bits (IT/ICI masked)
            7 => self.regs.xpsr & 0x0700_01FF,
            // MSP
            8 => self.regs.msp,
            // PSP
            9 => self.regs.psp,
            // MSPLIM
            10 => self.regs.msplim,
            // PSPLIM
            11 => self.regs.psplim,
            // PRIMASK
            16 => self.regs.primask & 1,
            // BASEPRI
            17 => self.regs.basepri & 0xFF,
            // FAULTMASK
            19 => self.regs.faultmask & 1,
            // CONTROL
            20 => self.regs.control & 0x7,
            // --- Non-Secure banked registers (ARMv8-M, SYSm bit 7 = NS) ---
            0x88 => self.regs.msp_ns,
            0x89 => self.regs.psp_ns,
            0x8A => self.regs.msplim_ns,
            0x8B => self.regs.psplim_ns,
            0x90 => self.regs.primask_ns & 1,
            0x91 => self.regs.basepri_ns & 0xFF,
            0x93 => self.regs.faultmask_ns & 1,
            0x94 => self.regs.control_ns & 0x7,
            // Reserved
            _ => 0,
        };
        1 // M33 measured: 1 cycle
    }

    // -- Multiply (32-bit result) --------------------------------------------

    pub(crate) fn thumb32_multiply(&mut self, hw0: u16, hw1: u16) -> u32 {
        let op1 = ((hw0 >> 4) & 0x7) as u8;
        let rn = (hw0 & 0xF) as usize;
        let ra = ((hw1 >> 12) & 0xF) as usize;
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let op2 = ((hw1 >> 4) & 0x3) as u8;
        let rm = (hw1 & 0xF) as usize;

        match (op1, op2) {
            (0b000, 0b00) => {
                let result = self.regs.r[rn].wrapping_mul(self.regs.r[rm]);
                if ra == 15 {
                    // MUL
                    self.regs.r[rd] = result;
                } else {
                    // MLA
                    self.regs.r[rd] = result.wrapping_add(self.regs.r[ra]);
                }
            }
            (0b000, 0b01) => {
                // MLS
                let product = self.regs.r[rn].wrapping_mul(self.regs.r[rm]);
                self.regs.r[rd] = self.regs.r[ra].wrapping_sub(product);
            }
            // Halfword multiply: SMLABB/BT/TB/TT (Ra!=15) / SMULBB/BT/TB/TT (Ra=15)
            (0b001, _) => {
                let bottom_n = op2 & 0x2 == 0;
                let bottom_m = op2 & 0x1 == 0;
                let rn_half = if bottom_n {
                    self.regs.r[rn] as i16 as i32
                } else {
                    (self.regs.r[rn] >> 16) as i16 as i32
                };
                let rm_half = if bottom_m {
                    self.regs.r[rm] as i16 as i32
                } else {
                    (self.regs.r[rm] >> 16) as i16 as i32
                };
                let product = rn_half.wrapping_mul(rm_half);
                if ra == 15 {
                    self.regs.r[rd] = product as u32;
                } else {
                    let acc = self.regs.r[ra] as i32;
                    let (result, overflow) = product.overflowing_add(acc);
                    self.regs.r[rd] = result as u32;
                    if overflow { self.regs.set_flag_q(); }
                }
            }
            // Dual multiply add: SMLAD/SMLADX (Ra!=15) / SMUAD/SMUADX (Ra=15)
            (0b010, _) => {
                let cross = op2 & 0x1 != 0;
                let rn_lo = self.regs.r[rn] as i16 as i32;
                let rn_hi = (self.regs.r[rn] >> 16) as i16 as i32;
                let (rm_lo, rm_hi) = if cross {
                    ((self.regs.r[rm] >> 16) as i16 as i32, self.regs.r[rm] as i16 as i32)
                } else {
                    (self.regs.r[rm] as i16 as i32, (self.regs.r[rm] >> 16) as i16 as i32)
                };
                let p1 = rn_lo.wrapping_mul(rm_lo);
                let p2 = rn_hi.wrapping_mul(rm_hi);
                let (sum, ov1) = p1.overflowing_add(p2);
                if ra == 15 {
                    self.regs.r[rd] = sum as u32;
                    if ov1 { self.regs.set_flag_q(); }
                } else {
                    let acc = self.regs.r[ra] as i32;
                    let (result, ov2) = sum.overflowing_add(acc);
                    self.regs.r[rd] = result as u32;
                    if ov1 || ov2 { self.regs.set_flag_q(); }
                }
            }
            // Word x halfword: SMLAWB/SMLAWT (Ra!=15) / SMULWB/SMULWT (Ra=15)
            (0b011, _) => {
                let bottom_m = op2 & 0x1 == 0;
                let rm_half = if bottom_m {
                    self.regs.r[rm] as i16 as i32
                } else {
                    (self.regs.r[rm] >> 16) as i16 as i32
                };
                let product = (self.regs.r[rn] as i32 as i64) * (rm_half as i64);
                let product_hi = (product >> 16) as i32;
                if ra == 15 {
                    self.regs.r[rd] = product_hi as u32;
                } else {
                    let acc = self.regs.r[ra] as i32;
                    let (result, overflow) = product_hi.overflowing_add(acc);
                    self.regs.r[rd] = result as u32;
                    if overflow { self.regs.set_flag_q(); }
                }
            }
            // Dual multiply subtract: SMLSD/SMLSDX (Ra!=15) / SMUSD/SMUSDX (Ra=15)
            (0b100, _) => {
                let cross = op2 & 0x1 != 0;
                let rn_lo = self.regs.r[rn] as i16 as i32;
                let rn_hi = (self.regs.r[rn] >> 16) as i16 as i32;
                let (rm_lo, rm_hi) = if cross {
                    ((self.regs.r[rm] >> 16) as i16 as i32, self.regs.r[rm] as i16 as i32)
                } else {
                    (self.regs.r[rm] as i16 as i32, (self.regs.r[rm] >> 16) as i16 as i32)
                };
                let p1 = rn_lo.wrapping_mul(rm_lo);
                let p2 = rn_hi.wrapping_mul(rm_hi);
                let diff = p1.wrapping_sub(p2);
                if ra == 15 {
                    self.regs.r[rd] = diff as u32;
                } else {
                    let acc = self.regs.r[ra] as i32;
                    let (result, overflow) = diff.overflowing_add(acc);
                    self.regs.r[rd] = result as u32;
                    if overflow { self.regs.set_flag_q(); }
                }
            }
            // Most significant word multiply: SMMLA/SMMLAR / SMMUL/SMMULR
            (0b101, _) => {
                let round = op2 & 0x1 != 0;
                let product = (self.regs.r[rn] as i32 as i64) * (self.regs.r[rm] as i32 as i64);
                if ra == 15 {
                    let result = if round {
                        (product.wrapping_add(0x8000_0000) >> 32) as i32
                    } else {
                        (product >> 32) as i32
                    };
                    self.regs.r[rd] = result as u32;
                } else {
                    let acc = (self.regs.r[ra] as i32 as i64) << 32;
                    let sum = product.wrapping_add(acc);
                    let result = if round {
                        (sum.wrapping_add(0x8000_0000) >> 32) as i32
                    } else {
                        (sum >> 32) as i32
                    };
                    self.regs.r[rd] = result as u32;
                }
            }
            // Most significant word multiply-subtract: SMMLS/SMMLSR
            (0b110, _) => {
                let round = op2 & 0x1 != 0;
                let product = (self.regs.r[rn] as i32 as i64) * (self.regs.r[rm] as i32 as i64);
                let acc = (self.regs.r[ra] as i32 as i64) << 32;
                let diff = acc.wrapping_sub(product);
                let result = if round {
                    (diff.wrapping_add(0x8000_0000) >> 32) as i32
                } else {
                    (diff >> 32) as i32
                };
                self.regs.r[rd] = result as u32;
            }
            // Sum of absolute differences: USADA8 (Ra!=15) / USAD8 (Ra=15)
            (0b111, _) => {
                let a = self.regs.r[rn];
                let b = self.regs.r[rm];
                let mut sum = 0u32;
                for i in 0..4 {
                    let a_byte = ((a >> (i * 8)) & 0xFF) as i32;
                    let b_byte = ((b >> (i * 8)) & 0xFF) as i32;
                    sum += (a_byte - b_byte).unsigned_abs();
                }
                if ra == 15 {
                    self.regs.r[rd] = sum;
                } else {
                    self.regs.r[rd] = sum.wrapping_add(self.regs.r[ra]);
                }
            }
            _ => return self.thumb32_undefined(hw0, hw1),
        }
        1 // M33: 1 cycle
    }

    // -- Long multiply / divide (64-bit result) ------------------------------

    pub(crate) fn thumb32_long_multiply(&mut self, hw0: u16, hw1: u16) -> u32 {
        let op1 = ((hw0 >> 4) & 0x7) as u8;
        let rn = (hw0 & 0xF) as usize;
        let rd_lo = ((hw1 >> 12) & 0xF) as usize;
        let rd_hi = ((hw1 >> 8) & 0xF) as usize;
        let op2 = ((hw1 >> 4) & 0xF) as u8;
        let rm = (hw1 & 0xF) as usize;

        match (op1, op2) {
            (0b000, 0b0000) => {
                // SMULL
                let result = (self.regs.r[rn] as i32 as i64) * (self.regs.r[rm] as i32 as i64);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            (0b010, 0b0000) => {
                // UMULL
                let result = (self.regs.r[rn] as u64) * (self.regs.r[rm] as u64);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            (0b100, 0b0000) => {
                // SMLAL
                let acc = ((self.regs.r[rd_hi] as u64) << 32) | self.regs.r[rd_lo] as u64;
                let product = (self.regs.r[rn] as i32 as i64) * (self.regs.r[rm] as i32 as i64);
                let result = (acc as i64).wrapping_add(product);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            (0b110, 0b0000) => {
                // UMLAL
                let acc = ((self.regs.r[rd_hi] as u64) << 32) | self.regs.r[rd_lo] as u64;
                let product = (self.regs.r[rn] as u64) * (self.regs.r[rm] as u64);
                let result = acc.wrapping_add(product);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            (0b001, 0b1111) => {
                // SDIV
                let a = self.regs.r[rn] as i32;
                let b = self.regs.r[rm] as i32;
                self.regs.r[rd_hi] = if b == 0 { 0 } else { a.wrapping_div(b) as u32 };
                5 // M33 measured: 1-12 cycles (data-dependent), using 5 as average
            }
            (0b011, 0b1111) => {
                // UDIV
                let a = self.regs.r[rn];
                let b = self.regs.r[rm];
                self.regs.r[rd_hi] = if b == 0 { 0 } else { a / b };
                5 // M33 measured: 1-12 cycles (data-dependent), using 5 as average
            }
            // SMLALBB/BT/TB/TT: op1=100, op2=10xx
            (0b100, 0b1000..=0b1011) => {
                let bottom_n = op2 & 0x2 == 0;
                let bottom_m = op2 & 0x1 == 0;
                let rn_half = if bottom_n {
                    self.regs.r[rn] as i16 as i64
                } else {
                    (self.regs.r[rn] >> 16) as i16 as i64
                };
                let rm_half = if bottom_m {
                    self.regs.r[rm] as i16 as i64
                } else {
                    (self.regs.r[rm] >> 16) as i16 as i64
                };
                let product = rn_half * rm_half;
                let acc = ((self.regs.r[rd_hi] as u64) << 32) | self.regs.r[rd_lo] as u64;
                let result = (acc as i64).wrapping_add(product);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            // SMLALD/SMLALDX: op1=100, op2=1100/1101
            (0b100, 0b1100 | 0b1101) => {
                let cross = op2 & 0x1 != 0;
                let rn_lo = self.regs.r[rn] as i16 as i64;
                let rn_hi = (self.regs.r[rn] >> 16) as i16 as i64;
                let (rm_lo, rm_hi) = if cross {
                    ((self.regs.r[rm] >> 16) as i16 as i64, self.regs.r[rm] as i16 as i64)
                } else {
                    (self.regs.r[rm] as i16 as i64, (self.regs.r[rm] >> 16) as i16 as i64)
                };
                let p1 = rn_lo * rm_lo;
                let p2 = rn_hi * rm_hi;
                let acc = ((self.regs.r[rd_hi] as u64) << 32) | self.regs.r[rd_lo] as u64;
                let result = (acc as i64).wrapping_add(p1).wrapping_add(p2);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            // SMLSLD/SMLSLDX: op1=101, op2=1100/1101
            (0b101, 0b1100 | 0b1101) => {
                let cross = op2 & 0x1 != 0;
                let rn_lo = self.regs.r[rn] as i16 as i64;
                let rn_hi = (self.regs.r[rn] >> 16) as i16 as i64;
                let (rm_lo, rm_hi) = if cross {
                    ((self.regs.r[rm] >> 16) as i16 as i64, self.regs.r[rm] as i16 as i64)
                } else {
                    (self.regs.r[rm] as i16 as i64, (self.regs.r[rm] >> 16) as i16 as i64)
                };
                let p1 = rn_lo * rm_lo;
                let p2 = rn_hi * rm_hi;
                let acc = ((self.regs.r[rd_hi] as u64) << 32) | self.regs.r[rd_lo] as u64;
                let result = (acc as i64).wrapping_add(p1).wrapping_sub(p2);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            // UMAAL: op1=110, op2=0110
            (0b110, 0b0110) => {
                let product = (self.regs.r[rn] as u64) * (self.regs.r[rm] as u64);
                let result = product
                    .wrapping_add(self.regs.r[rd_lo] as u64)
                    .wrapping_add(self.regs.r[rd_hi] as u64);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1 // M33: 1 cycle
            }
            _ => return self.thumb32_undefined(hw0, hw1),
        }
    }

    // -- Data processing (register) ------------------------------------------

    pub(crate) fn thumb32_dp_register(&mut self, hw0: u16, hw1: u16) -> u32 {
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let rm = (hw1 & 0xF) as usize;

        if hw0 & 0x80 != 0 {
            // hw0[7]=1: misc ops, parallel add/sub, saturating, SEL
            if hw1 & 0x80 == 0 {
                // hw1[7]=0: Parallel add/subtract
                let rn = (hw0 & 0xF) as usize;
                let par_op1 = ((hw0 >> 4) & 0x7) as u8;
                let par_op2 = ((hw1 >> 4) & 0x7) as u8;
                return self.thumb32_parallel_add_sub(rd, rn, rm, par_op1, par_op2);
            }
            // hw1[7]=1: misc (REV/CLZ) or saturating (QADD/SEL)
            if hw0 & 0x10 != 0 {
                // hw0[4]=1: REV/CLZ family
                let op1_lo = (hw0 >> 5) & 0x3;
                let op2_lo = (hw1 >> 4) & 0x3;
                let val = self.regs.r[rm];
                match (op1_lo, op2_lo) {
                    (0b00, 0b00) => {
                        self.regs.r[rd] = val.swap_bytes();
                        1 // M33 measured: 1 cycle
                    }
                    (0b00, 0b01) => {
                        let lo = ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8);
                        let hi = ((val & 0x00FF_0000) << 8) | ((val & 0xFF00_0000) >> 8);
                        self.regs.r[rd] = hi | lo;
                        1 // M33 measured: 1 cycle
                    }
                    (0b00, 0b10) => {
                        self.regs.r[rd] = val.reverse_bits();
                        1 // M33 measured: 1 cycle
                    }
                    (0b00, 0b11) => {
                        let lo_hw = val as u16;
                        let swapped = ((lo_hw & 0xFF) << 8) | ((lo_hw >> 8) & 0xFF);
                        self.regs.r[rd] = swapped as i16 as i32 as u32;
                        1 // M33 measured: 1 cycle
                    }
                    (0b01, 0b00) => {
                        self.regs.r[rd] = val.leading_zeros();
                        1 // M33 measured: 1 cycle
                    }
                    _ => self.thumb32_undefined(hw0, hw1),
                }
            } else {
                // hw0[4]=0: QADD/QSUB/QDADD/QDSUB/SEL
                let rn = (hw0 & 0xF) as usize;
                let op1_65 = ((hw0 >> 5) & 0x3) as u8;
                let op2_54 = ((hw1 >> 4) & 0x3) as u8;
                // Saturate on overflow: if the wrapping result is negative,
                // positive overflow occurred (clamp to i32::MAX); if non-negative,
                // negative overflow occurred (clamp to i32::MIN).
                let saturate = |val: i32, ov: bool| -> i32 {
                    if ov { if val < 0 { i32::MAX } else { i32::MIN } } else { val }
                };
                match (op1_65, op2_54) {
                    (0b00, 0b00) => {
                        // QADD: Rd = saturate(Rn + Rm)
                        let a = self.regs.r[rn] as i32;
                        let b = self.regs.r[rm] as i32;
                        let (result, overflow) = a.overflowing_add(b);
                        if overflow { self.regs.set_flag_q(); }
                        self.regs.r[rd] = saturate(result, overflow) as u32;
                        1 // M33 measured: 1 cycle
                    }
                    (0b00, 0b01) => {
                        // QDADD: Rd = saturate(Rm + saturate(2*Rn))
                        let rn_val = self.regs.r[rn] as i32;
                        let rm_val = self.regs.r[rm] as i32;
                        let (doubled, ov1) = rn_val.overflowing_add(rn_val);
                        if ov1 { self.regs.set_flag_q(); }
                        let doubled = saturate(doubled, ov1);
                        let (result, ov2) = rm_val.overflowing_add(doubled);
                        if ov2 { self.regs.set_flag_q(); }
                        self.regs.r[rd] = saturate(result, ov2) as u32;
                        1 // M33 measured: 1 cycle
                    }
                    (0b00, 0b10) => {
                        // QSUB: Rd = saturate(Rm - Rn)
                        let a = self.regs.r[rm] as i32;
                        let b = self.regs.r[rn] as i32;
                        let (result, overflow) = a.overflowing_sub(b);
                        if overflow { self.regs.set_flag_q(); }
                        self.regs.r[rd] = saturate(result, overflow) as u32;
                        1 // M33 measured: 1 cycle
                    }
                    (0b00, 0b11) => {
                        // QDSUB: Rd = saturate(Rm - saturate(2*Rn))
                        let rn_val = self.regs.r[rn] as i32;
                        let rm_val = self.regs.r[rm] as i32;
                        let (doubled, ov1) = rn_val.overflowing_add(rn_val);
                        if ov1 { self.regs.set_flag_q(); }
                        let doubled = saturate(doubled, ov1);
                        let (result, ov2) = rm_val.overflowing_sub(doubled);
                        if ov2 { self.regs.set_flag_q(); }
                        self.regs.r[rd] = saturate(result, ov2) as u32;
                        1 // M33 measured: 1 cycle
                    }
                    (0b01, 0b00) => {
                        // SEL: select bytes based on GE flags
                        let ge = self.regs.ge_flags();
                        let a = self.regs.r[rn];
                        let b = self.regs.r[rm];
                        let mut result = 0u32;
                        for i in 0..4u32 {
                            let byte_mask = 0xFFu32 << (i * 8);
                            if ge & (1 << i) != 0 {
                                result |= a & byte_mask;
                            } else {
                                result |= b & byte_mask;
                            }
                        }
                        self.regs.r[rd] = result;
                        1 // M33 measured: 1 cycle
                    }
                    _ => self.thumb32_undefined(hw0, hw1),
                }
            }
        } else if hw1 & 0x80 != 0 {
            // -- Extend ops (hw0[7]=0, hw1[7]=1) --------------------------------
            // hw0 = 1111_1010_0_ext_Rn, hw1 = 1111_Rd_10_rot_Rm
            let rn = (hw0 & 0xF) as usize;
            let ext = ((hw0 >> 4) & 0x7) as u8;  // hw0[6:4]
            let rot = ((hw1 >> 4) & 0x3) * 8;    // rotation in bits: 0, 8, 16, 24
            let rotated = self.regs.r[rm].rotate_right(rot as u32);

            if rn == 15 {
                // Plain extend (no add)
                let result = match ext {
                    0b000 => (rotated as i16) as i32 as u32,          // SXTH
                    0b001 => rotated & 0xFFFF,                        // UXTH
                    0b010 => {
                        // SXTB16: sign-extend bytes 0 and 2 to halfwords
                        let b0 = (rotated & 0xFF) as i8 as i16 as u16 as u32;
                        let b2 = ((rotated >> 16) & 0xFF) as i8 as i16 as u16 as u32;
                        b0 | (b2 << 16)
                    }
                    0b011 => {
                        // UXTB16: zero-extend bytes 0 and 2 to halfwords
                        (rotated & 0xFF) | (((rotated >> 16) & 0xFF) << 16)
                    }
                    0b100 => (rotated as i8) as i32 as u32,           // SXTB
                    0b101 => rotated & 0xFF,                          // UXTB
                    _ => return self.thumb32_undefined(hw0, hw1),
                };
                self.regs.r[rd] = result;
            } else {
                // Extend-and-add (SXTAH, UXTAH, SXTAB, UXTAB)
                let addend = self.regs.r[rn];
                let result = match ext {
                    0b000 => addend.wrapping_add((rotated as i16) as i32 as u32), // SXTAH
                    0b001 => addend.wrapping_add(rotated & 0xFFFF),               // UXTAH
                    0b010 => {
                        // SXTAB16: packed halfword add with sign-extended bytes
                        let b0 = (rotated & 0xFF) as i8 as i16 as u16 as u32;
                        let b2 = ((rotated >> 16) & 0xFF) as i8 as i16 as u16 as u32;
                        let lo = (addend & 0xFFFF).wrapping_add(b0) & 0xFFFF;
                        let hi = ((addend >> 16) & 0xFFFF).wrapping_add(b2) & 0xFFFF;
                        lo | (hi << 16)
                    }
                    0b011 => {
                        // UXTAB16: packed halfword add with zero-extended bytes
                        let b0 = rotated & 0xFF;
                        let b2 = (rotated >> 16) & 0xFF;
                        let lo = (addend & 0xFFFF).wrapping_add(b0) & 0xFFFF;
                        let hi = ((addend >> 16) & 0xFFFF).wrapping_add(b2) & 0xFFFF;
                        lo | (hi << 16)
                    }
                    0b100 => addend.wrapping_add((rotated as i8) as i32 as u32),  // SXTAB
                    0b101 => addend.wrapping_add(rotated & 0xFF),                 // UXTAB
                    _ => return self.thumb32_undefined(hw0, hw1),
                };
                self.regs.r[rd] = result;
            }
            1 // M33 measured: 1 cycle
        } else {
            // -- Wide shifts by register (hw0[7]=0, hw1[7:4]=0000) --------------
            // hw0 = 1111_1010_0_stype_S_Rn, hw1 = 1111_Rd_0000_Rm
            let rn = (hw0 & 0xF) as usize;
            let stype = ((hw0 >> 5) & 0x3) as u8;  // hw0[6:5]
            let s = hw0 & (1 << 4) != 0;           // hw0[4] = S bit
            let shift = self.regs.r[rm] & 0xFF;
            let value = self.regs.r[rn];

            let (result, carry) = match stype {
                0b00 => {
                    // LSL.W
                    if shift == 0 {
                        (value, self.regs.flag_c())
                    } else if shift < 32 {
                        (value << shift, (value >> (32 - shift)) & 1 != 0)
                    } else if shift == 32 {
                        (0, value & 1 != 0)
                    } else {
                        (0, false)
                    }
                }
                0b01 => {
                    // LSR.W
                    if shift == 0 {
                        (value, self.regs.flag_c())
                    } else if shift < 32 {
                        (value >> shift, (value >> (shift - 1)) & 1 != 0)
                    } else if shift == 32 {
                        (0, value >> 31 != 0)
                    } else {
                        (0, false)
                    }
                }
                0b10 => {
                    // ASR.W
                    let sv = value as i32;
                    if shift == 0 {
                        (value, self.regs.flag_c())
                    } else if shift < 32 {
                        ((sv >> shift) as u32, (sv >> (shift as i32 - 1)) & 1 != 0)
                    } else {
                        ((sv >> 31) as u32, sv < 0)
                    }
                }
                _ => {
                    // ROR.W (stype=11)
                    if shift == 0 {
                        (value, self.regs.flag_c())
                    } else {
                        let eff = shift & 31;
                        if eff == 0 {
                            (value, value >> 31 != 0)
                        } else {
                            let r = value.rotate_right(eff);
                            (r, r >> 31 != 0)
                        }
                    }
                }
            };

            self.regs.r[rd] = result;
            if s {
                self.regs.set_nz(result);
                self.regs.set_flag_c(carry);
            }
            1 // M33 measured: 1 cycle
        }
    }

    // -- Parallel add/subtract ------------------------------------------------

    fn thumb32_parallel_add_sub(
        &mut self, rd: usize, rn: usize, rm: usize, par_op1: u8, par_op2: u8,
    ) -> u32 {
        // par_op1 = hw0[6:4] = base operation (ADD8/ADD16/ASX/SAX/SUB8/SUB16)
        // par_op2 = hw1[6:4] = modifier (signed/Q/halving/unsigned/UQ/UH)
        let a = self.regs.r[rn];
        let b = self.regs.r[rm];
        match par_op2 {
            // Signed variants
            0b000 => match par_op1 {
                0b001 | 0b010 | 0b011 | 0b101 => self.parallel_signed_16(rd, a, b, par_op1, false, false),
                0b000 | 0b100 => self.parallel_signed_8(rd, a, b, par_op1),
                _ => 1,
            },
            // Q-saturating signed (16-bit only)
            0b001 => self.parallel_signed_16(rd, a, b, par_op1, true, false),
            // Halving signed (16-bit only)
            0b010 => self.parallel_signed_16(rd, a, b, par_op1, false, true),
            // Unsigned variants
            0b100 => match par_op1 {
                0b001 | 0b010 | 0b011 | 0b101 => self.parallel_unsigned_16(rd, a, b, par_op1, false, false),
                0b000 | 0b100 => self.parallel_unsigned_8(rd, a, b, par_op1),
                _ => 1,
            },
            // Q-saturating unsigned (16-bit only)
            0b101 => self.parallel_unsigned_16(rd, a, b, par_op1, true, false),
            // Halving unsigned (16-bit only)
            0b110 => self.parallel_unsigned_16(rd, a, b, par_op1, false, true),
            _ => 1,
        }
    }

    fn parallel_signed_16(
        &mut self, rd: usize, a: u32, b: u32, op: u8, sat: bool, halving: bool,
    ) -> u32 {
        let a_lo = a as i16 as i32;
        let a_hi = (a >> 16) as i16 as i32;
        let b_lo = b as i16 as i32;
        let b_hi = (b >> 16) as i16 as i32;
        let (r_lo, r_hi) = match op {
            0b001 => (a_lo + b_lo, a_hi + b_hi),       // ADD16
            0b010 => (a_lo - b_hi, a_hi + b_lo),       // ASX
            0b011 => (a_lo + b_hi, a_hi - b_lo),       // SAX
            0b101 => (a_lo - b_lo, a_hi - b_hi),       // SUB16
            _ => return 1,
        };
        let (lo, hi) = if sat {
            (r_lo.clamp(-32768, 32767), r_hi.clamp(-32768, 32767))
        } else if halving {
            (r_lo >> 1, r_hi >> 1)
        } else {
            let mut ge = self.regs.ge_flags();
            if r_lo >= 0 { ge |= 0x3; } else { ge &= !0x3; }
            if r_hi >= 0 { ge |= 0xC; } else { ge &= !0xC; }
            self.regs.set_ge_flags(ge);
            (r_lo, r_hi)
        };
        self.regs.r[rd] = (lo as u16 as u32) | ((hi as u16 as u32) << 16);
        1 // M33 measured: 1 cycle
    }

    fn parallel_unsigned_16(
        &mut self, rd: usize, a: u32, b: u32, op: u8, sat: bool, halving: bool,
    ) -> u32 {
        let a_lo = (a & 0xFFFF) as u32;
        let a_hi = (a >> 16) as u32;
        let b_lo = (b & 0xFFFF) as u32;
        let b_hi = (b >> 16) as u32;
        // Use i32 for subtraction results to handle borrow
        let (r_lo_i, r_hi_i): (i32, i32) = match op {
            0b001 => (a_lo as i32 + b_lo as i32, a_hi as i32 + b_hi as i32),  // ADD16
            0b010 => (a_lo as i32 - b_hi as i32, a_hi as i32 + b_lo as i32),  // ASX
            0b011 => (a_lo as i32 + b_hi as i32, a_hi as i32 - b_lo as i32),  // SAX
            0b101 => (a_lo as i32 - b_lo as i32, a_hi as i32 - b_hi as i32),  // SUB16
            _ => return 1,
        };
        let (lo, hi) = if sat {
            (
                (r_lo_i as u32).min(0xFFFF),
                (r_hi_i as u32).min(0xFFFF),
            )
        } else if halving {
            ((r_lo_i as u32) >> 1, (r_hi_i as u32) >> 1)
        } else {
            let mut ge = self.regs.ge_flags();
            // GE set if no borrow (result >= 0x10000 for add, >= 0 for sub)
            match op {
                0b001 => {
                    if r_lo_i >= 0x10000 { ge |= 0x3; } else { ge &= !0x3; }
                    if r_hi_i >= 0x10000 { ge |= 0xC; } else { ge &= !0xC; }
                }
                _ => {
                    if r_lo_i >= 0 { ge |= 0x3; } else { ge &= !0x3; }
                    if r_hi_i >= 0 { ge |= 0xC; } else { ge &= !0xC; }
                }
            }
            self.regs.set_ge_flags(ge);
            (r_lo_i as u32, r_hi_i as u32)
        };
        self.regs.r[rd] = (lo as u16 as u32) | ((hi as u16 as u32) << 16);
        1 // M33 measured: 1 cycle
    }

    fn parallel_signed_8(&mut self, rd: usize, a: u32, b: u32, op: u8) -> u32 {
        let mut result = 0u32;
        let mut ge = 0u32;
        for i in 0..4u32 {
            let a_byte = ((a >> (i * 8)) & 0xFF) as i8 as i32;
            let b_byte = ((b >> (i * 8)) & 0xFF) as i8 as i32;
            let r = match op {
                0b000 => a_byte + b_byte,
                0b100 => a_byte - b_byte,
                _ => return 1,
            };
            if r >= 0 { ge |= 1 << i; }
            result |= ((r as u8) as u32) << (i * 8);
        }
        self.regs.set_ge_flags(ge);
        self.regs.r[rd] = result;
        1 // M33 measured: 1 cycle
    }

    fn parallel_unsigned_8(&mut self, rd: usize, a: u32, b: u32, op: u8) -> u32 {
        let mut result = 0u32;
        let mut ge = 0u32;
        for i in 0..4u32 {
            let a_byte = ((a >> (i * 8)) & 0xFF) as u32;
            let b_byte = ((b >> (i * 8)) & 0xFF) as u32;
            let r: i32 = match op {
                0b000 => (a_byte + b_byte) as i32,
                0b100 => a_byte as i32 - b_byte as i32,
                _ => return 1,
            };
            match op {
                0b000 => { if r >= 0x100 { ge |= 1 << i; } }
                _ => { if r >= 0 { ge |= 1 << i; } }
            }
            result |= ((r as u32) & 0xFF) << (i * 8);
        }
        self.regs.set_ge_flags(ge);
        self.regs.r[rd] = result;
        1 // M33 measured: 1 cycle
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
        1 // M33 measured: 1 cycle
    }

    // -- Undefined 32-bit instruction ----------------------------------------

    /// Undefined 32-bit instruction — raises UsageFault.
    pub(crate) fn thumb32_undefined(&mut self, _hw0: u16, _hw1: u16) -> u32 {
        self.pending_fault = Some(super::Fault::UsageFault);
        0
    }
}

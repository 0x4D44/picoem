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
                1
            }
            // BIC
            0b0001 => {
                let result = self.regs.r[rn] & !shifted;
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nz(result);
                    self.regs.set_flag_c(shift_carry);
                }
                1
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
                1
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
                1
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
                1
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
                1
            }
            // ADC
            0b1010 => {
                let (result, carry, overflow) =
                    add_with_carry(self.regs.r[rn], shifted, self.regs.flag_c());
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1
            }
            // SBC
            0b1011 => {
                let (result, carry, overflow) =
                    add_with_carry(self.regs.r[rn], !shifted, self.regs.flag_c());
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1
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
                1
            }
            // RSB
            0b1110 => {
                let (result, carry, overflow) = add_with_carry(!self.regs.r[rn], shifted, true);
                self.regs.r[rd] = result;
                if s {
                    self.regs.set_nzcv(result >> 31 != 0, result == 0, carry, overflow);
                }
                1
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

        for i in 0..16 {
            if reglist & (1 << i) != 0 {
                if load {
                    let val = bus.read32(addr);
                    if i == 15 {
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

        // Writeback: if W set AND (for loads) Rn is NOT in reglist
        if w && (!load || reglist & (1 << rn) == 0) {
            self.regs.r[rn] = match op {
                0b01 => self.regs.r[rn].wrapping_add(count * 4),   // IA: Rn + 4*count
                0b10 => self.regs.r[rn].wrapping_sub(count * 4),   // DB: Rn - 4*count
                _ => unreachable!(),
            };
        }

        // Cost: 1 + count, plus 3 extra if PC was loaded
        let pc_loaded = load && reglist & (1 << 15) != 0;
        1 + count + if pc_loaded { 3 } else { 0 }
    }

    // -- Load/store dual, exclusive, table branch ----------------------------

    pub(crate) fn thumb32_load_store_dual(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
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

        3 // 1+2 for both load and store
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
                // GE bits (mask[0]) not implemented for Phase 1
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
            _ => {} // reserved — ignore
        }
        2
    }

    /// MRS — read a special system register into a general-purpose register.
    /// Encoding: 11110_0111_11_R_1111  10_00_Rd_SYSm
    fn thumb32_mrs(&mut self, hw1: u16) -> u32 {
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let sysm = (hw1 & 0xFF) as u8;

        self.regs.r[rd] = match sysm {
            // APSR / IAPSR / EAPSR / XPSR / combined variants — NZCVQ flags
            0 | 1 | 2 | 3 | 4 => self.regs.xpsr & 0xF800_0000,
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
            // PRIMASK
            16 => self.regs.primask & 1,
            // BASEPRI
            17 => self.regs.basepri & 0xFF,
            // FAULTMASK
            19 => self.regs.faultmask & 1,
            // CONTROL
            20 => self.regs.control & 0x7,
            // Reserved
            _ => 0,
        };
        2
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
            _ => return self.thumb32_undefined(hw0, hw1),
        }
        1 // all 1 cycle
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
                1
            }
            (0b010, 0b0000) => {
                // UMULL
                let result = (self.regs.r[rn] as u64) * (self.regs.r[rm] as u64);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1
            }
            (0b100, 0b0000) => {
                // SMLAL
                let acc = ((self.regs.r[rd_hi] as u64) << 32) | self.regs.r[rd_lo] as u64;
                let product = (self.regs.r[rn] as i32 as i64) * (self.regs.r[rm] as i32 as i64);
                let result = (acc as i64).wrapping_add(product);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1
            }
            (0b110, 0b0000) => {
                // UMLAL
                let acc = ((self.regs.r[rd_hi] as u64) << 32) | self.regs.r[rd_lo] as u64;
                let product = (self.regs.r[rn] as u64) * (self.regs.r[rm] as u64);
                let result = acc.wrapping_add(product);
                self.regs.r[rd_lo] = result as u32;
                self.regs.r[rd_hi] = (result >> 32) as u32;
                1
            }
            (0b001, 0b1111) => {
                // SDIV
                let a = self.regs.r[rn] as i32;
                let b = self.regs.r[rm] as i32;
                self.regs.r[rd_lo] = if b == 0 { 0 } else { a.wrapping_div(b) as u32 };
                4 // placeholder
            }
            (0b011, 0b1111) => {
                // UDIV
                let a = self.regs.r[rn];
                let b = self.regs.r[rm];
                self.regs.r[rd_lo] = if b == 0 { 0 } else { a / b };
                4 // placeholder
            }
            _ => return self.thumb32_undefined(hw0, hw1),
        }
    }

    // -- Data processing (register) ------------------------------------------

    pub(crate) fn thumb32_dp_register(&mut self, hw0: u16, hw1: u16) -> u32 {
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let rm = (hw1 & 0xF) as usize;

        if hw0 & 0x80 != 0 {
            // -- Misc single-register ops (hw0[7]=1) ----------------------------
            // hw0 = 1111_1010_1xxx_Rm, hw1 = 1111_Rd_1xxx_Rm
            let op1_lo = (hw0 >> 5) & 0x3;  // hw0[6:5]
            let op2_lo = (hw1 >> 4) & 0x3;  // hw1[5:4]
            let val = self.regs.r[rm];

            match (op1_lo, op2_lo) {
                (0b00, 0b00) => {
                    // REV.W — byte reverse word
                    self.regs.r[rd] = val.swap_bytes();
                    1
                }
                (0b00, 0b01) => {
                    // REV16.W — byte reverse packed halfwords
                    let lo = ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8);
                    let hi = ((val & 0x00FF_0000) << 8) | ((val & 0xFF00_0000) >> 8);
                    self.regs.r[rd] = hi | lo;
                    1
                }
                (0b00, 0b10) => {
                    // RBIT — reverse bits
                    self.regs.r[rd] = val.reverse_bits();
                    1
                }
                (0b00, 0b11) => {
                    // REVSH.W — byte reverse signed halfword
                    let lo_hw = val as u16;
                    let swapped = ((lo_hw & 0xFF) << 8) | ((lo_hw >> 8) & 0xFF);
                    self.regs.r[rd] = swapped as i16 as i32 as u32;
                    1
                }
                (0b01, 0b00) => {
                    // CLZ — count leading zeros
                    self.regs.r[rd] = val.leading_zeros();
                    1
                }
                _ => self.thumb32_undefined(hw0, hw1),
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
                    0b100 => (rotated as i8) as i32 as u32,           // SXTB
                    0b101 => rotated & 0xFF,                          // UXTB
                    _ => return self.thumb32_undefined(hw0, hw1),     // SXTB16/UXTB16: DSP, skip
                };
                self.regs.r[rd] = result;
            } else {
                // Extend-and-add (SXTAH, UXTAH, SXTAB, UXTAB)
                let addend = self.regs.r[rn];
                let result = match ext {
                    0b000 => addend.wrapping_add((rotated as i16) as i32 as u32), // SXTAH
                    0b001 => addend.wrapping_add(rotated & 0xFFFF),               // UXTAH
                    0b100 => addend.wrapping_add((rotated as i8) as i32 as u32),  // SXTAB
                    0b101 => addend.wrapping_add(rotated & 0xFF),                 // UXTAB
                    _ => return self.thumb32_undefined(hw0, hw1),
                };
                self.regs.r[rd] = result;
            }
            1
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
            1
        }
    }

    // -- Coprocessor ---------------------------------------------------------

    pub(crate) fn thumb32_coprocessor(&mut self, hw0: u16, hw1: u16, _bus: &mut Bus) -> u32 {
        let _coproc = ((hw1 >> 8) & 0xF) as u8;
        // Phase 1: all coprocessor instructions are undefined stubs.
        // Future phases will dispatch on _coproc:
        //   0       → GPIO coprocessor
        //   4 | 5   → DCP (double-precision coprocessor)
        //   7       → RCP (runtime check coprocessor)
        //   10 | 11 → FPU
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

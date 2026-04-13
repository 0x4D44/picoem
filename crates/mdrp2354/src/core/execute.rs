use crate::bus::Bus;
use super::CortexM33;

// ============================================================================
// Helpers
// ============================================================================

/// Add with carry. Returns (result, carry_out, overflow).
/// Used for ADD, ADC, SUB (a + NOT(b) + 1), SBC, RSB, CMP, CMN.
#[inline(always)]
pub(crate) fn add_with_carry(a: u32, b: u32, carry_in: bool) -> (u32, bool, bool) {
    let wide = (a as u64) + (b as u64) + (carry_in as u64);
    let result = wide as u32;
    let carry_out = wide > 0xFFFF_FFFF;
    let overflow = (((a ^ result) & (b ^ result)) >> 31) != 0;
    (result, carry_out, overflow)
}

/// Sign-extend a value from `bits` width to 32 bits.
#[inline(always)]
pub(crate) fn sign_extend(val: u32, bits: u32) -> u32 {
    let shift = 32 - bits;
    ((val << shift) as i32 >> shift) as u32
}

// ============================================================================
// Thumb-16: Shift (immediate)
// ============================================================================

impl CortexM33 {
    /// LSLS Rd, Rm, #imm5 — encoding T1 (00000_imm5_Rm_Rd).
    /// When imm5=0 this is MOVS Rd, Rm (carry unchanged).
    pub(crate) fn thumb16_lsl_imm(&mut self, opcode: u16) -> u32 {
        let rd = (opcode & 0x7) as usize;
        let rm = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let val = self.regs.r[rm];

        if imm5 == 0 {
            // MOVS Rd, Rm — no shift, carry unchanged
            self.regs.r[rd] = val;
            self.regs.set_nz(val);
        } else {
            let result = val << imm5;
            let carry = (val >> (32 - imm5)) & 1 != 0;
            self.regs.r[rd] = result;
            self.regs.set_nz(result);
            self.regs.set_flag_c(carry);
        }
        1
    }

    /// LSRS Rd, Rm, #imm5 — encoding T1 (00001_imm5_Rm_Rd).
    /// imm5=0 encodes shift by 32.
    pub(crate) fn thumb16_lsr_imm(&mut self, opcode: u16) -> u32 {
        let rd = (opcode & 0x7) as usize;
        let rm = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let val = self.regs.r[rm];

        let (result, carry) = if imm5 == 0 {
            // Shift by 32: result = 0, carry = bit 31
            (0, val >> 31 != 0)
        } else {
            (val >> imm5, (val >> (imm5 - 1)) & 1 != 0)
        };
        self.regs.r[rd] = result;
        self.regs.set_nz(result);
        self.regs.set_flag_c(carry);
        1
    }

    /// ASRS Rd, Rm, #imm5 — encoding T1 (00010_imm5_Rm_Rd).
    /// imm5=0 encodes shift by 32.
    pub(crate) fn thumb16_asr_imm(&mut self, opcode: u16) -> u32 {
        let rd = (opcode & 0x7) as usize;
        let rm = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let val = self.regs.r[rm] as i32;

        let (result, carry) = if imm5 == 0 {
            // Shift by 32: result = sign-extended, carry = bit 31
            let r = val >> 31; // all 0s or all 1s
            (r as u32, val < 0)
        } else {
            let r = val >> imm5;
            let c = (val >> (imm5 as i32 - 1)) & 1 != 0;
            (r as u32, c)
        };
        self.regs.r[rd] = result;
        self.regs.set_nz(result);
        self.regs.set_flag_c(carry);
        1
    }

    // ========================================================================
    // Thumb-16: Add/Sub (register and 3-bit immediate)
    // ========================================================================

    /// Bits[15:11]=00011. Sub-decode on bits[10:9]:
    /// 00=ADDS reg, 01=SUBS reg, 10=ADDS imm3, 11=SUBS imm3.
    pub(crate) fn thumb16_add_sub(&mut self, opcode: u16) -> u32 {
        let rd = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let rn_val = self.regs.r[rn];

        match (opcode >> 9) & 0x3 {
            0b00 => {
                // ADDS Rd, Rn, Rm
                let rm = ((opcode >> 6) & 0x7) as usize;
                let rm_val = self.regs.r[rm];
                let (result, c, v) = add_with_carry(rn_val, rm_val, false);
                self.regs.r[rd] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0b01 => {
                // SUBS Rd, Rn, Rm
                let rm = ((opcode >> 6) & 0x7) as usize;
                let rm_val = self.regs.r[rm];
                let (result, c, v) = add_with_carry(rn_val, !rm_val, true);
                self.regs.r[rd] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0b10 => {
                // ADDS Rd, Rn, #imm3
                let imm3 = ((opcode >> 6) & 0x7) as u32;
                let (result, c, v) = add_with_carry(rn_val, imm3, false);
                self.regs.r[rd] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            _ => {
                // SUBS Rd, Rn, #imm3
                let imm3 = ((opcode >> 6) & 0x7) as u32;
                let (result, c, v) = add_with_carry(rn_val, !imm3, true);
                self.regs.r[rd] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
        }
        1
    }

    // ========================================================================
    // Thumb-16: Move/Compare/Add/Sub 8-bit immediate
    // ========================================================================

    /// MOVS Rd, #imm8 (00100_Rd_imm8).
    pub(crate) fn thumb16_mov_imm(&mut self, opcode: u16) -> u32 {
        let rd = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        self.regs.r[rd] = imm8;
        self.regs.set_nz(imm8);
        1
    }

    /// CMP Rn, #imm8 (00101_Rn_imm8).
    pub(crate) fn thumb16_cmp_imm(&mut self, opcode: u16) -> u32 {
        let rn = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let rn_val = self.regs.r[rn];
        let (result, c, v) = add_with_carry(rn_val, !imm8, true);
        self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
        1
    }

    /// ADDS Rd, Rd, #imm8 (00110_Rdn_imm8).
    pub(crate) fn thumb16_add_imm8(&mut self, opcode: u16) -> u32 {
        let rdn = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let rdn_val = self.regs.r[rdn];
        let (result, c, v) = add_with_carry(rdn_val, imm8, false);
        self.regs.r[rdn] = result;
        self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
        1
    }

    /// SUBS Rd, Rd, #imm8 (00111_Rdn_imm8).
    pub(crate) fn thumb16_sub_imm8(&mut self, opcode: u16) -> u32 {
        let rdn = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let rdn_val = self.regs.r[rdn];
        let (result, c, v) = add_with_carry(rdn_val, !imm8, true);
        self.regs.r[rdn] = result;
        self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
        1
    }

    // ========================================================================
    // Thumb-16: Data processing (register)
    // ========================================================================

    /// 16 register-register ALU ops. Opcode bits[9:6] select the operation.
    /// All operate on low registers (R0-R7), all update flags.
    pub(crate) fn thumb16_data_processing(&mut self, opcode: u16) -> u32 {
        let op = (opcode >> 6) & 0xF;
        let rm = ((opcode >> 3) & 0x7) as usize;
        let rdn = (opcode & 0x7) as usize;
        let a = self.regs.r[rdn];
        let b = self.regs.r[rm];

        match op {
            0x0 => {
                // ANDS Rdn, Rm
                let result = a & b;
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
            }
            0x1 => {
                // EORS Rdn, Rm
                let result = a ^ b;
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
            }
            0x2 => {
                // LSLS Rdn, Rm (shift by register)
                let shift = b & 0xFF;
                let (result, carry) = if shift == 0 {
                    (a, self.regs.flag_c())
                } else if shift < 32 {
                    (a << shift, (a >> (32 - shift)) & 1 != 0)
                } else if shift == 32 {
                    (0, a & 1 != 0)
                } else {
                    (0, false)
                };
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
                self.regs.set_flag_c(carry);
            }
            0x3 => {
                // LSRS Rdn, Rm (shift by register)
                let shift = b & 0xFF;
                let (result, carry) = if shift == 0 {
                    (a, self.regs.flag_c())
                } else if shift < 32 {
                    (a >> shift, (a >> (shift - 1)) & 1 != 0)
                } else if shift == 32 {
                    (0, a >> 31 != 0)
                } else {
                    (0, false)
                };
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
                self.regs.set_flag_c(carry);
            }
            0x4 => {
                // ASRS Rdn, Rm (shift by register)
                let shift = b & 0xFF;
                let sa = a as i32;
                let (result, carry) = if shift == 0 {
                    (a, self.regs.flag_c())
                } else if shift < 32 {
                    ((sa >> shift) as u32, (sa >> (shift as i32 - 1)) & 1 != 0)
                } else {
                    // shift >= 32: result = sign extension, carry = sign bit
                    ((sa >> 31) as u32, sa < 0)
                };
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
                self.regs.set_flag_c(carry);
            }
            0x5 => {
                // ADCS Rdn, Rm
                let (result, c, v) = add_with_carry(a, b, self.regs.flag_c());
                self.regs.r[rdn] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0x6 => {
                // SBCS Rdn, Rm — Rdn = Rdn - Rm - NOT(C) = Rdn + NOT(Rm) + C
                let (result, c, v) = add_with_carry(a, !b, self.regs.flag_c());
                self.regs.r[rdn] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0x7 => {
                // RORS Rdn, Rm
                let shift = b & 0xFF;
                let (result, carry) = if shift == 0 {
                    (a, self.regs.flag_c())
                } else {
                    let eff = (shift & 31) as u32;
                    if eff == 0 {
                        // Rotate by 32 (or multiple of 32): result unchanged, carry = bit 31
                        (a, a >> 31 != 0)
                    } else {
                        let r = a.rotate_right(eff);
                        (r, r >> 31 != 0)
                    }
                };
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
                self.regs.set_flag_c(carry);
            }
            0x8 => {
                // TST Rn, Rm — AND but discard result
                let result = a & b;
                self.regs.set_nz(result);
            }
            0x9 => {
                // RSBS Rdn, Rm, #0 (NEG) — result = 0 - Rm = 0 + NOT(Rm) + 1
                // Note: ARM encoding says Rdn = result, Rm = source
                // Actually: RSBS Rd, Rn, #0 where Rn=bits[5:3], Rd=bits[2:0]
                // So: result = 0 - b (source is Rm=bits[5:3])
                let (result, c, v) = add_with_carry(0, !b, true);
                self.regs.r[rdn] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0xA => {
                // CMP Rn, Rm (low registers) — SUB but discard result
                let (result, c, v) = add_with_carry(a, !b, true);
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0xB => {
                // CMN Rn, Rm — ADD but discard result
                let (result, c, v) = add_with_carry(a, b, false);
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0xC => {
                // ORRS Rdn, Rm
                let result = a | b;
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
            }
            0xD => {
                // MULS Rdn, Rm — Rd = Rd * Rm, only low 32 bits
                let result = a.wrapping_mul(b);
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
                // Note: C and V flags are UNPREDICTABLE for M33 MUL
            }
            0xE => {
                // BICS Rdn, Rm — Rd = Rd AND NOT(Rm)
                let result = a & !b;
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
            }
            _ => {
                // 0xF: MVNS Rdn, Rm — Rd = NOT(Rm)
                let result = !b;
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
            }
        }
        1
    }

    // ========================================================================
    // Thumb-16: Special data / BX / BLX
    // ========================================================================

    /// High-register ADD/CMP/MOV and BX/BLX.
    pub(crate) fn thumb16_special_data_bx(&mut self, opcode: u16, _bus: &mut Bus) -> u32 {
        let op = (opcode >> 8) & 0x3;
        match op {
            0b00 => {
                // ADD Rd, Rm (high registers, no flag update)
                let d = ((opcode >> 4) & 0x8 | opcode & 0x7) as usize; // D:Rd
                let rm = ((opcode >> 3) & 0xF) as usize;
                let rm_val = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                let rd_val = if d == 15 { self.read_pc() } else { self.regs.r[d] };
                let result = rd_val.wrapping_add(rm_val);
                if d == 15 {
                    self.regs.set_pc(result & !1);
                    return 3; // pipeline flush
                }
                self.regs.r[d] = result;
                1
            }
            0b01 => {
                // CMP Rn, Rm (high registers)
                let n = ((opcode >> 4) & 0x8 | opcode & 0x7) as usize;
                let rm = ((opcode >> 3) & 0xF) as usize;
                let rn_val = if n == 15 { self.read_pc() } else { self.regs.r[n] };
                let rm_val = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                let (result, c, v) = add_with_carry(rn_val, !rm_val, true);
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
                1
            }
            0b10 => {
                // MOV Rd, Rm (high registers, no flag update)
                let d = ((opcode >> 4) & 0x8 | opcode & 0x7) as usize;
                let rm = ((opcode >> 3) & 0xF) as usize;
                let val = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                if d == 15 {
                    self.regs.set_pc(val & !1);
                    return 3; // pipeline flush
                }
                self.regs.r[d] = val;
                1
            }
            _ => {
                // BX / BLX
                let rm = ((opcode >> 3) & 0xF) as usize;
                let target = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                let link = opcode & (1 << 7) != 0; // bit 7: 0=BX, 1=BLX
                if link {
                    // BLX Rm: LR = address of next instruction | 1
                    let next = self.regs.pc() | 1;
                    self.regs.set_lr(next);
                }
                // Bit 0 of target encodes Thumb state. Must be 1 for M33.
                // For Phase 1: accept and mask off.
                self.regs.set_pc(target & !1);
                3 // pipeline flush
            }
        }
    }

    // ========================================================================
    // Thumb-16: Load literal (PC-relative)
    // ========================================================================

    /// LDR Rt, [PC, #imm8*4] (01001_Rt_imm8).
    pub(crate) fn thumb16_ldr_literal(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        // PC is aligned down to word boundary, then offset added
        let base = self.read_pc() & !3;
        let addr = base.wrapping_add(imm8 << 2);
        self.regs.r[rt] = bus.read32(addr);
        2 // 1 (addr calc) + 1 (memory)
    }

    // ========================================================================
    // Thumb-16: Load/store register offset
    // ========================================================================

    /// STR/STRH/STRB/LDRSB/LDR/LDRH/LDRB/LDRSH with register offset.
    /// Encoding: 0101_opc_Rm_Rn_Rt.
    pub(crate) fn thumb16_load_store_reg(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let rm = ((opcode >> 6) & 0x7) as usize;
        let opc = (opcode >> 9) & 0x7;
        let addr = self.regs.r[rn].wrapping_add(self.regs.r[rm]);

        match opc {
            0b000 => {
                // STR Rt, [Rn, Rm]
                bus.write32(addr, self.regs.r[rt]);
                1
            }
            0b001 => {
                // STRH Rt, [Rn, Rm]
                bus.write16(addr, self.regs.r[rt] as u16);
                1
            }
            0b010 => {
                // STRB Rt, [Rn, Rm]
                bus.write8(addr, self.regs.r[rt] as u8);
                1
            }
            0b011 => {
                // LDRSB Rt, [Rn, Rm]
                let val = bus.read8(addr) as i8 as i32 as u32;
                self.regs.r[rt] = val;
                2
            }
            0b100 => {
                // LDR Rt, [Rn, Rm]
                self.regs.r[rt] = bus.read32(addr);
                2
            }
            0b101 => {
                // LDRH Rt, [Rn, Rm]
                self.regs.r[rt] = bus.read16(addr) as u32;
                2
            }
            0b110 => {
                // LDRB Rt, [Rn, Rm]
                self.regs.r[rt] = bus.read8(addr) as u32;
                2
            }
            _ => {
                // 0b111: LDRSH Rt, [Rn, Rm]
                let val = bus.read16(addr) as i16 as i32 as u32;
                self.regs.r[rt] = val;
                2
            }
        }
    }

    // ========================================================================
    // Thumb-16: Load/store immediate offset
    // ========================================================================

    /// STR Rt, [Rn, #imm5*4] (01100_imm5_Rn_Rt).
    pub(crate) fn thumb16_str_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 2);
        bus.write32(addr, self.regs.r[rt]);
        1
    }

    /// LDR Rt, [Rn, #imm5*4] (01101_imm5_Rn_Rt).
    pub(crate) fn thumb16_ldr_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 2);
        self.regs.r[rt] = bus.read32(addr);
        2
    }

    /// STRB Rt, [Rn, #imm5] (01110_imm5_Rn_Rt).
    pub(crate) fn thumb16_strb_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5);
        bus.write8(addr, self.regs.r[rt] as u8);
        1
    }

    /// LDRB Rt, [Rn, #imm5] (01111_imm5_Rn_Rt).
    pub(crate) fn thumb16_ldrb_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5);
        self.regs.r[rt] = bus.read8(addr) as u32;
        2
    }

    /// STRH Rt, [Rn, #imm5*2] (10000_imm5_Rn_Rt).
    pub(crate) fn thumb16_strh_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 1);
        bus.write16(addr, self.regs.r[rt] as u16);
        1
    }

    /// LDRH Rt, [Rn, #imm5*2] (10001_imm5_Rn_Rt).
    pub(crate) fn thumb16_ldrh_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 1);
        self.regs.r[rt] = bus.read16(addr) as u32;
        2
    }

    // ========================================================================
    // Thumb-16: SP-relative load/store
    // ========================================================================

    /// STR Rt, [SP, #imm8*4] (10010_Rt_imm8).
    pub(crate) fn thumb16_str_sp(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let addr = self.regs.sp().wrapping_add(imm8 << 2);
        bus.write32(addr, self.regs.r[rt]);
        1
    }

    /// LDR Rt, [SP, #imm8*4] (10011_Rt_imm8).
    pub(crate) fn thumb16_ldr_sp(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let addr = self.regs.sp().wrapping_add(imm8 << 2);
        self.regs.r[rt] = bus.read32(addr);
        2
    }

    // ========================================================================
    // Thumb-16: ADR / ADD SP
    // ========================================================================

    /// ADR Rd, #imm8*4 (10100_Rd_imm8) — PC-relative address.
    pub(crate) fn thumb16_adr(&mut self, opcode: u16) -> u32 {
        let rd = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let base = self.read_pc() & !3; // Align(PC, 4)
        self.regs.r[rd] = base.wrapping_add(imm8 << 2);
        1
    }

    /// ADD Rd, SP, #imm8*4 (10101_Rd_imm8).
    pub(crate) fn thumb16_add_sp_imm(&mut self, opcode: u16) -> u32 {
        let rd = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        self.regs.r[rd] = self.regs.sp().wrapping_add(imm8 << 2);
        1
    }

    // ========================================================================
    // Thumb-16: Miscellaneous
    // ========================================================================

    /// Miscellaneous 16-bit instructions (bits[15:12] = 1011).
    pub(crate) fn thumb16_misc(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let op = (opcode >> 8) & 0xF;
        match op {
            0b0000 => {
                // Adjust SP
                let imm7 = (opcode & 0x7F) as u32;
                let offset = imm7 << 2;
                if opcode & (1 << 7) == 0 {
                    // ADD SP, SP, #imm7*4
                    self.regs.r[13] = self.regs.sp().wrapping_add(offset);
                } else {
                    // SUB SP, SP, #imm7*4
                    self.regs.r[13] = self.regs.sp().wrapping_sub(offset);
                }
                1
            }
            0b0010 => {
                // Sign/zero extend
                let rm = ((opcode >> 3) & 0x7) as usize;
                let rd = (opcode & 0x7) as usize;
                let val = self.regs.r[rm];
                match (opcode >> 6) & 0x3 {
                    0b00 => self.regs.r[rd] = val as i16 as i32 as u32,  // SXTH
                    0b01 => self.regs.r[rd] = val as i8 as i32 as u32,   // SXTB
                    0b10 => self.regs.r[rd] = val & 0xFFFF,              // UXTH
                    _    => self.regs.r[rd] = val & 0xFF,                // UXTB
                }
                1
            }
            0b0100 | 0b0101 => {
                // PUSH {reglist, LR?}
                let mut reglist = (opcode & 0xFF) as u32;
                if opcode & (1 << 8) != 0 {
                    reglist |= 1 << 14; // include LR
                }
                let count = reglist.count_ones();
                let mut addr = self.regs.sp().wrapping_sub(count * 4);
                self.regs.set_sp(addr);
                for i in 0..15 {
                    if reglist & (1 << i) != 0 {
                        bus.write32(addr, self.regs.r[i]);
                        addr = addr.wrapping_add(4);
                    }
                }
                1 + count
            }
            0b0110 => {
                // CPS — stub for Phase 1 (affects PRIMASK/FAULTMASK)
                1
            }
            0b1010 => {
                // REV/REV16/REVSH
                let rm = ((opcode >> 3) & 0x7) as usize;
                let rd = (opcode & 0x7) as usize;
                let val = self.regs.r[rm];
                match (opcode >> 6) & 0x3 {
                    0b00 => self.regs.r[rd] = val.swap_bytes(),          // REV
                    0b01 => {                                             // REV16
                        self.regs.r[rd] =
                            ((val >> 8) & 0x00FF_00FF) | ((val << 8) & 0xFF00_FF00);
                    }
                    0b11 => {                                             // REVSH
                        let half = (val & 0xFFFF) as u16;
                        let swapped = half.swap_bytes();
                        self.regs.r[rd] = swapped as i16 as i32 as u32;
                    }
                    _ => {} // 0b10 is undefined — treat as NOP for now
                }
                1
            }
            0b1100 | 0b1101 => {
                // POP {reglist, PC?}
                let mut reglist = (opcode & 0xFF) as u32;
                let pop_pc = opcode & (1 << 8) != 0;
                if pop_pc {
                    reglist |= 1 << 15; // include PC
                }
                let count = reglist.count_ones();
                let mut addr = self.regs.sp();
                for i in 0..16 {
                    if reglist & (1 << i) != 0 {
                        let val = bus.read32(addr);
                        if i == 15 {
                            // Loading PC: bit 0 → T bit (must be 1), clear for addr
                            self.regs.set_pc(val & !1);
                        } else {
                            self.regs.r[i] = val;
                        }
                        addr = addr.wrapping_add(4);
                    }
                }
                self.regs.set_sp(addr);
                if pop_pc { 1 + count + 3 } else { 1 + count }
            }
            0b1110 => {
                // BKPT #imm8 — stub: treat as NOP for Phase 1
                1
            }
            0b1111 => {
                let mask = opcode & 0xF;
                if mask != 0 {
                    // IT instruction: firstcond = bits[7:4], mask = bits[3:0]
                    self.it_state = (opcode & 0xFF) as u8;
                    1
                } else {
                    // Hints: NOP, YIELD, WFE, WFI, SEV
                    // All are NOPs for Phase 1
                    1
                }
            }
            // CBZ/CBNZ: bits[11:8] matches x0x1 pattern
            op if op & 0x5 == 0x1 => {
                let rn = (opcode & 0x7) as usize;
                let i = ((opcode >> 9) & 1) as u32;
                let imm5 = ((opcode >> 3) & 0x1F) as u32;
                let offset = (i << 6) | (imm5 << 1);
                let nonzero = opcode & (1 << 11) != 0;
                let rn_val = self.regs.r[rn];
                if (nonzero && rn_val != 0) || (!nonzero && rn_val == 0) {
                    let target = self.read_pc().wrapping_add(offset);
                    self.regs.set_pc(target);
                    2
                } else {
                    1
                }
            }
            _ => 1, // Other misc encodings — NOP
        }
    }

    // ========================================================================
    // Thumb-16: Load/store multiple
    // ========================================================================

    /// STM Rn!, {reglist} (11000_Rn_reglist).
    pub(crate) fn thumb16_stm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rn = ((opcode >> 8) & 0x7) as usize;
        let reglist = (opcode & 0xFF) as u32;
        let count = reglist.count_ones();
        let mut addr = self.regs.r[rn];

        for i in 0..8 {
            if reglist & (1 << i) != 0 {
                bus.write32(addr, self.regs.r[i]);
                addr = addr.wrapping_add(4);
            }
        }
        // Writeback
        self.regs.r[rn] = addr;
        1 + count
    }

    /// LDM Rn!, {reglist} (11001_Rn_reglist).
    /// Writeback only if Rn is NOT in reglist.
    pub(crate) fn thumb16_ldm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rn = ((opcode >> 8) & 0x7) as usize;
        let reglist = (opcode & 0xFF) as u32;
        let count = reglist.count_ones();
        let mut addr = self.regs.r[rn];

        for i in 0..8 {
            if reglist & (1 << i) != 0 {
                self.regs.r[i] = bus.read32(addr);
                addr = addr.wrapping_add(4);
            }
        }
        // Writeback if Rn not in reglist
        if reglist & (1 << rn) == 0 {
            self.regs.r[rn] = addr;
        }
        1 + count
    }

    // ========================================================================
    // Thumb-16: Conditional branch / SVC
    // ========================================================================

    /// B.cond and SVC (1101_cond_imm8).
    pub(crate) fn thumb16_cond_branch_svc(&mut self, opcode: u16) -> u32 {
        let cond = ((opcode >> 8) & 0xF) as u8;
        match cond {
            0xE => {
                // UDF — permanently undefined
                self.thumb16_undefined(opcode)
            }
            0xF => {
                // SVC — stub for Phase 1
                1
            }
            _ => {
                // Conditional branch
                if self.regs.condition_passed(cond) {
                    let imm8 = (opcode & 0xFF) as u32;
                    let offset = sign_extend(imm8 << 1, 9); // 8-bit imm, shifted left 1, sign-extended from bit 8
                    let target = self.read_pc().wrapping_add(offset);
                    self.regs.set_pc(target);
                    2 // branch taken
                } else {
                    1 // not taken
                }
            }
        }
    }

    // ========================================================================
    // Thumb-16: Unconditional branch
    // ========================================================================

    /// B label (11100_imm11).
    pub(crate) fn thumb16_branch(&mut self, opcode: u16) -> u32 {
        let imm11 = (opcode & 0x7FF) as u32;
        let offset = sign_extend(imm11 << 1, 12); // 11-bit imm, shifted left 1, sign-extended from bit 11
        let target = self.read_pc().wrapping_add(offset);
        self.regs.set_pc(target);
        2
    }

    // ========================================================================
    // Thumb-16: Undefined
    // ========================================================================

    /// Undefined instruction — UsageFault in Phase 3, NOP for now.
    pub(crate) fn thumb16_undefined(&mut self, _opcode: u16) -> u32 {
        // TODO: raise UsageFault (Phase 3)
        1
    }
}

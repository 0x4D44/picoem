//! ARMv6-M Thumb-16 executor.
//!
//! One method per opcode group (bits [15:11] dispatch from
//! [`super::decode`]). Semantics mirror the ARMv6-M ARM DDI 0419 spec —
//! flags and cycle counts follow the same pattern the mdrp2350 core
//! uses for encodings common to both ISAs, with the M33-only behaviour
//! stripped (no IT blocks, no CBZ/CBNZ, no wide-path handling, no
//! security state).
//!
//! Cycle counts are aligned with the mdrp2350 figures because Phase 4.A
//! does not yet have a bus-timing model — the Phase 5 RP2040 bus will
//! recalibrate against real-silicon measurements when it lands.

use super::{CortexM0Plus, Fault};
use crate::bus::Bus;

// ============================================================================
// Helpers
// ============================================================================

/// Add with carry. Returns (result, carry_out, overflow).
#[inline(always)]
pub(crate) fn add_with_carry(a: u32, b: u32, carry_in: bool) -> (u32, bool, bool) {
    let wide = (a as u64) + (b as u64) + (carry_in as u64);
    let result = wide as u32;
    let carry_out = wide > 0xFFFF_FFFF;
    let overflow = (((a ^ result) & (b ^ result)) >> 31) != 0;
    (result, carry_out, overflow)
}

/// Sign-extend `val` from `bits` width to 32 bits.
#[inline(always)]
pub(crate) fn sign_extend(val: u32, bits: u32) -> u32 {
    let shift = 32 - bits;
    ((val << shift) as i32 >> shift) as u32
}

/// Alignment check for word / halfword memory accesses. Returns `true`
/// when `addr` is aligned to `size` bytes — `size` must be a power of
/// two. Byte accesses (`size == 1`) are always legal and return `true`.
#[inline(always)]
pub(crate) fn is_aligned(addr: u32, size: u32) -> bool {
    addr & (size - 1) == 0
}

// ============================================================================
// Thumb-16: Shift (immediate)
// ============================================================================

impl CortexM0Plus {
    /// LSLS Rd, Rm, #imm5 — encoding T1 (`00000_imm5_Rm_Rd`).
    /// When imm5 == 0 this is `MOVS Rd, Rm` (carry unchanged).
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

    /// LSRS Rd, Rm, #imm5 — encoding T1 (`00001_imm5_Rm_Rd`).
    /// imm5 == 0 encodes shift-by-32.
    pub(crate) fn thumb16_lsr_imm(&mut self, opcode: u16) -> u32 {
        let rd = (opcode & 0x7) as usize;
        let rm = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let val = self.regs.r[rm];

        let (result, carry) = if imm5 == 0 {
            (0, val >> 31 != 0)
        } else {
            (val >> imm5, (val >> (imm5 - 1)) & 1 != 0)
        };
        self.regs.r[rd] = result;
        self.regs.set_nz(result);
        self.regs.set_flag_c(carry);
        1
    }

    /// ASRS Rd, Rm, #imm5 — encoding T1 (`00010_imm5_Rm_Rd`).
    /// imm5 == 0 encodes shift-by-32.
    pub(crate) fn thumb16_asr_imm(&mut self, opcode: u16) -> u32 {
        let rd = (opcode & 0x7) as usize;
        let rm = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let val = self.regs.r[rm] as i32;

        let (result, carry) = if imm5 == 0 {
            let r = val >> 31;
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

    /// `bits[15:11] = 00011`. Sub-decode on bits[10:9]:
    /// 00 = ADDS reg, 01 = SUBS reg, 10 = ADDS imm3, 11 = SUBS imm3.
    pub(crate) fn thumb16_add_sub(&mut self, opcode: u16) -> u32 {
        let rd = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let rn_val = self.regs.r[rn];

        match (opcode >> 9) & 0x3 {
            0b00 => {
                let rm = ((opcode >> 6) & 0x7) as usize;
                let rm_val = self.regs.r[rm];
                let (result, c, v) = add_with_carry(rn_val, rm_val, false);
                self.regs.r[rd] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0b01 => {
                let rm = ((opcode >> 6) & 0x7) as usize;
                let rm_val = self.regs.r[rm];
                let (result, c, v) = add_with_carry(rn_val, !rm_val, true);
                self.regs.r[rd] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0b10 => {
                let imm3 = ((opcode >> 6) & 0x7) as u32;
                let (result, c, v) = add_with_carry(rn_val, imm3, false);
                self.regs.r[rd] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            _ => {
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

    /// MOVS Rd, #imm8 (`00100_Rd_imm8`). Flags: N/Z (carry preserved).
    pub(crate) fn thumb16_mov_imm(&mut self, opcode: u16) -> u32 {
        let rd = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        self.regs.r[rd] = imm8;
        self.regs.set_nz(imm8);
        1
    }

    /// CMP Rn, #imm8 (`00101_Rn_imm8`). Updates N/Z/C/V.
    pub(crate) fn thumb16_cmp_imm(&mut self, opcode: u16) -> u32 {
        let rn = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let rn_val = self.regs.r[rn];
        let (result, c, v) = add_with_carry(rn_val, !imm8, true);
        self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
        1
    }

    /// ADDS Rdn, #imm8 (`00110_Rdn_imm8`).
    pub(crate) fn thumb16_add_imm8(&mut self, opcode: u16) -> u32 {
        let rdn = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let rdn_val = self.regs.r[rdn];
        let (result, c, v) = add_with_carry(rdn_val, imm8, false);
        self.regs.r[rdn] = result;
        self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
        1
    }

    /// SUBS Rdn, #imm8 (`00111_Rdn_imm8`).
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

    /// 16 low-register ALU ops. Opcode bits[9:6] select the operation.
    /// All operate on low registers (R0-R7) and all update flags.
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
                // SBCS Rdn, Rm
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
                // TST Rn, Rm
                let result = a & b;
                self.regs.set_nz(result);
            }
            0x9 => {
                // RSBS Rd, Rn, #0  (a.k.a. NEG)
                let (result, c, v) = add_with_carry(0, !b, true);
                self.regs.r[rdn] = result;
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0xA => {
                // CMP Rn, Rm (low registers)
                let (result, c, v) = add_with_carry(a, !b, true);
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
            }
            0xB => {
                // CMN Rn, Rm
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
                // MULS Rdn, Rm — low 32 bits of a * b. M0+ MUL cycle
                // count varies (1–32 cycles on early-termination
                // variants); use 1 here until Phase 5 bus timing
                // measures the RP2040 figure.
                let result = a.wrapping_mul(b);
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
                return 1;
            }
            0xE => {
                // BICS Rdn, Rm
                let result = a & !b;
                self.regs.r[rdn] = result;
                self.regs.set_nz(result);
            }
            _ => {
                // 0xF: MVNS Rdn, Rm
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
    ///
    /// `bits[15:10] = 010001`. Sub-op decoded from bits[9:8]:
    /// 00 = ADD, 01 = CMP, 10 = MOV, 11 = BX/BLX (bit 7 selects link).
    ///
    /// ADD and MOV do NOT update flags (unlike low-register variants).
    /// CMP updates flags. BX/BLX transfer control via register.
    pub(crate) fn thumb16_special_data_bx(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let op = (opcode >> 8) & 0x3;
        match op {
            0b00 => {
                // ADD Rd, Rm (high registers, no flags)
                let d = (((opcode >> 4) & 0x8) | (opcode & 0x7)) as usize; // DN:Rd
                let rm = ((opcode >> 3) & 0xF) as usize;
                let rm_val = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                let rd_val = if d == 15 { self.read_pc() } else { self.regs.r[d] };
                let result = rd_val.wrapping_add(rm_val);
                if d == 15 {
                    if result & 1 == 0 {
                        self.pending_fault = Some(Fault::InvalidEpsr);
                        return 1;
                    }
                    self.regs.set_pc(result & !1);
                    return 3; // pipeline flush
                }
                self.regs.r[d] = result;
                1
            }
            0b01 => {
                // CMP Rn, Rm (high registers)
                let n = (((opcode >> 4) & 0x8) | (opcode & 0x7)) as usize;
                let rm = ((opcode >> 3) & 0xF) as usize;
                let rn_val = if n == 15 { self.read_pc() } else { self.regs.r[n] };
                let rm_val = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                let (result, c, v) = add_with_carry(rn_val, !rm_val, true);
                self.regs.set_nzcv(result >> 31 != 0, result == 0, c, v);
                1
            }
            0b10 => {
                // MOV Rd, Rm (high registers, no flags)
                let d = (((opcode >> 4) & 0x8) | (opcode & 0x7)) as usize;
                let rm = ((opcode >> 3) & 0xF) as usize;
                let val = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                if d == 15 {
                    if val & 1 == 0 {
                        self.pending_fault = Some(Fault::InvalidEpsr);
                        return 1;
                    }
                    self.regs.set_pc(val & !1);
                    return 3; // pipeline flush
                }
                self.regs.r[d] = val;
                1
            }
            _ => {
                // BX / BLX Rm
                //
                // ARMv6-M: BX bits[7:0] are xxxx_x000 with Rm in bits[6:3];
                // BLX bits[7:0] are xxxx_x000 with bit 7 = 1. Bit 0 of the
                // target encodes Thumb state (must be 1 on M0+ — HardFault
                // if clear). BX to an EXC_RETURN magic value performs an
                // exception return instead of a branch.
                let rm = ((opcode >> 3) & 0xF) as usize;
                let target = if rm == 15 { self.read_pc() } else { self.regs.r[rm] };
                let link = opcode & (1 << 7) != 0;
                if link {
                    // BLX Rm — LR = address of next instruction | 1.
                    let next = self.current_instr_addr.wrapping_add(2) | 1;
                    self.regs.set_lr(next);
                    // BLX's target must have the Thumb bit set (it's a
                    // subroutine call, never an exception return).
                    if target & 1 == 0 {
                        self.pending_fault = Some(Fault::InvalidEpsr);
                        return 1;
                    }
                    self.regs.set_pc(target & !1);
                    return 3;
                }
                // BX in Handler mode with EXC_RETURN magic → exception exit.
                if self.regs.in_handler_mode() && Self::is_exc_return(target) {
                    self.exit_exception(target, bus);
                    return 3;
                }
                if target & 1 == 0 {
                    self.pending_fault = Some(Fault::InvalidEpsr);
                    return 1;
                }
                self.regs.set_pc(target & !1);
                3
            }
        }
    }

    // ========================================================================
    // Thumb-16: Load literal (PC-relative)
    // ========================================================================

    /// LDR Rt, [PC, #imm8*4] (`01001_Rt_imm8`).
    pub(crate) fn thumb16_ldr_literal(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let base = self.read_pc() & !3;
        let addr = base.wrapping_add(imm8 << 2);
        // PC-relative LDR is always word-aligned by construction (base
        // is force-aligned via & !3), but keep the check for symmetry —
        // cost is free.
        debug_assert!(is_aligned(addr, 4));
        self.regs.r[rt] = bus.read32(addr);
        2
    }

    // ========================================================================
    // Thumb-16: Load/store register offset
    // ========================================================================

    /// STR/STRH/STRB/LDRSB/LDR/LDRH/LDRB/LDRSH with register offset.
    /// Encoding: `0101_opc_Rm_Rn_Rt`.
    pub(crate) fn thumb16_load_store_reg(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let rm = ((opcode >> 6) & 0x7) as usize;
        let opc = (opcode >> 9) & 0x7;
        let addr = self.regs.r[rn].wrapping_add(self.regs.r[rm]);

        match opc {
            0b000 => {
                if !is_aligned(addr, 4) {
                    self.pending_fault = Some(Fault::Unaligned);
                    return 1;
                }
                bus.write32(addr, self.regs.r[rt]);
                2
            }
            0b001 => {
                if !is_aligned(addr, 2) {
                    self.pending_fault = Some(Fault::Unaligned);
                    return 1;
                }
                bus.write16(addr, self.regs.r[rt] as u16);
                2
            }
            0b010 => {
                bus.write8(addr, self.regs.r[rt] as u8);
                2
            }
            0b011 => {
                let val = bus.read8(addr) as i8 as i32 as u32;
                self.regs.r[rt] = val;
                2
            }
            0b100 => {
                if !is_aligned(addr, 4) {
                    self.pending_fault = Some(Fault::Unaligned);
                    return 1;
                }
                self.regs.r[rt] = bus.read32(addr);
                2
            }
            0b101 => {
                if !is_aligned(addr, 2) {
                    self.pending_fault = Some(Fault::Unaligned);
                    return 1;
                }
                self.regs.r[rt] = bus.read16(addr) as u32;
                2
            }
            0b110 => {
                self.regs.r[rt] = bus.read8(addr) as u32;
                2
            }
            _ => {
                // 0b111: LDRSH Rt, [Rn, Rm]
                if !is_aligned(addr, 2) {
                    self.pending_fault = Some(Fault::Unaligned);
                    return 1;
                }
                let val = bus.read16(addr) as i16 as i32 as u32;
                self.regs.r[rt] = val;
                2
            }
        }
    }

    // ========================================================================
    // Thumb-16: Load/store immediate offset
    // ========================================================================

    /// STR Rt, [Rn, #imm5*4] (`01100_imm5_Rn_Rt`).
    pub(crate) fn thumb16_str_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 2);
        if !is_aligned(addr, 4) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }
        bus.write32(addr, self.regs.r[rt]);
        2
    }

    /// LDR Rt, [Rn, #imm5*4] (`01101_imm5_Rn_Rt`).
    pub(crate) fn thumb16_ldr_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 2);
        if !is_aligned(addr, 4) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }
        self.regs.r[rt] = bus.read32(addr);
        2
    }

    /// STRB Rt, [Rn, #imm5] (`01110_imm5_Rn_Rt`).
    pub(crate) fn thumb16_strb_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5);
        bus.write8(addr, self.regs.r[rt] as u8);
        2
    }

    /// LDRB Rt, [Rn, #imm5] (`01111_imm5_Rn_Rt`).
    pub(crate) fn thumb16_ldrb_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5);
        self.regs.r[rt] = bus.read8(addr) as u32;
        2
    }

    /// STRH Rt, [Rn, #imm5*2] (`10000_imm5_Rn_Rt`).
    pub(crate) fn thumb16_strh_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 1);
        if !is_aligned(addr, 2) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }
        bus.write16(addr, self.regs.r[rt] as u16);
        2
    }

    /// LDRH Rt, [Rn, #imm5*2] (`10001_imm5_Rn_Rt`).
    pub(crate) fn thumb16_ldrh_imm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = (opcode & 0x7) as usize;
        let rn = ((opcode >> 3) & 0x7) as usize;
        let imm5 = ((opcode >> 6) & 0x1F) as u32;
        let addr = self.regs.r[rn].wrapping_add(imm5 << 1);
        if !is_aligned(addr, 2) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }
        self.regs.r[rt] = bus.read16(addr) as u32;
        2
    }

    // ========================================================================
    // Thumb-16: SP-relative load/store
    // ========================================================================

    /// STR Rt, [SP, #imm8*4] (`10010_Rt_imm8`).
    pub(crate) fn thumb16_str_sp(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let addr = self.regs.sp().wrapping_add(imm8 << 2);
        if !is_aligned(addr, 4) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }
        bus.write32(addr, self.regs.r[rt]);
        2
    }

    /// LDR Rt, [SP, #imm8*4] (`10011_Rt_imm8`).
    pub(crate) fn thumb16_ldr_sp(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rt = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let addr = self.regs.sp().wrapping_add(imm8 << 2);
        if !is_aligned(addr, 4) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }
        self.regs.r[rt] = bus.read32(addr);
        2
    }

    // ========================================================================
    // Thumb-16: ADR / ADD SP
    // ========================================================================

    /// ADR Rd, #imm8*4 (`10100_Rd_imm8`) — PC-relative address (no flags).
    pub(crate) fn thumb16_adr(&mut self, opcode: u16) -> u32 {
        let rd = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        let base = self.read_pc() & !3;
        self.regs.r[rd] = base.wrapping_add(imm8 << 2);
        1
    }

    /// ADD Rd, SP, #imm8*4 (`10101_Rd_imm8`) — no flags.
    pub(crate) fn thumb16_add_sp_imm(&mut self, opcode: u16) -> u32 {
        let rd = ((opcode >> 8) & 0x7) as usize;
        let imm8 = (opcode & 0xFF) as u32;
        self.regs.r[rd] = self.regs.sp().wrapping_add(imm8 << 2);
        1
    }

    // ========================================================================
    // Thumb-16: Miscellaneous 16-bit instructions
    // ========================================================================

    /// Misc group (`bits[15:12] = 1011`). Covers:
    /// * ADD/SUB SP, #imm7*4
    /// * SXTH/SXTB/UXTH/UXTB
    /// * PUSH {reglist, LR?}
    /// * CPS (CPSIE/CPSID)
    /// * REV/REV16/REVSH
    /// * POP {reglist, PC?}
    /// * BKPT #imm8 (stub: NOP until Phase 4.B wires HardFault delivery)
    /// * IT / hint (M0+: IT encodings are undefined; hints supported)
    ///
    /// Encodings that decode to CBZ/CBNZ on M33 are undefined on M0+
    /// and are routed through [`Self::thumb16_undefined`].
    pub(crate) fn thumb16_misc(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let op = (opcode >> 8) & 0xF;
        match op {
            0b0000 => {
                // ADD/SUB SP, #imm7*4
                let imm7 = (opcode & 0x7F) as u32;
                let offset = imm7 << 2;
                if opcode & (1 << 7) == 0 {
                    self.regs.r[13] = self.regs.sp().wrapping_add(offset);
                } else {
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
                    0b00 => self.regs.r[rd] = val as i16 as i32 as u32, // SXTH
                    0b01 => self.regs.r[rd] = val as i8 as i32 as u32,  // SXTB
                    0b10 => self.regs.r[rd] = val & 0xFFFF,             // UXTH
                    _ => self.regs.r[rd] = val & 0xFF,                  // UXTB
                }
                1
            }
            0b0100 | 0b0101 => {
                // PUSH {reglist, LR?}
                let mut reglist = (opcode & 0xFF) as u32;
                if opcode & (1 << 8) != 0 {
                    reglist |= 1 << 14; // LR
                }
                let count = reglist.count_ones();
                let base = self.regs.sp().wrapping_sub(count * 4);
                if !is_aligned(base, 4) {
                    self.pending_fault = Some(Fault::Unaligned);
                    return 1;
                }
                let mut addr = base;
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
                // CPS CPSIE/CPSID — affects PRIMASK only on M0+
                // (no FAULTMASK / BASEPRI).
                //
                // ARMv6-M ARM A6.7.38 (CPS T1): bits[1:0] are I:F.
                // Bit 1 = I (PRIMASK); bit 0 = F which is UNPREDICTABLE on
                // M0+ (no FAULTMASK), ignored.
                let im = ((opcode >> 4) & 1) as u32;
                let affect_i = opcode & (1 << 1) != 0;
                if affect_i {
                    self.regs.primask = im;
                }
                1
            }
            0b1010 => {
                // REV/REV16/REVSH
                let rm = ((opcode >> 3) & 0x7) as usize;
                let rd = (opcode & 0x7) as usize;
                let val = self.regs.r[rm];
                match (opcode >> 6) & 0x3 {
                    0b00 => self.regs.r[rd] = val.swap_bytes(), // REV
                    0b01 => {
                        // REV16
                        self.regs.r[rd] =
                            ((val >> 8) & 0x00FF_00FF) | ((val << 8) & 0xFF00_FF00);
                    }
                    0b11 => {
                        // REVSH
                        let half = (val & 0xFFFF) as u16;
                        let swapped = half.swap_bytes();
                        self.regs.r[rd] = swapped as i16 as i32 as u32;
                    }
                    _ => {
                        // 0b10 is UNDEFINED on ARMv6-M
                        return self.thumb16_undefined(opcode);
                    }
                }
                1
            }
            0b1100 | 0b1101 => {
                // POP {reglist, PC?}
                let mut reglist = (opcode & 0xFF) as u32;
                let pop_pc = opcode & (1 << 8) != 0;
                if pop_pc {
                    reglist |= 1 << 15;
                }
                let count = reglist.count_ones();
                let sp_start = self.regs.sp();
                if !is_aligned(sp_start, 4) {
                    self.pending_fault = Some(Fault::Unaligned);
                    return 1;
                }
                let mut addr = sp_start;
                let mut popped_pc: Option<u32> = None;
                for i in 0..16 {
                    if reglist & (1 << i) != 0 {
                        let val = bus.read32(addr);
                        if i == 15 {
                            popped_pc = Some(val);
                        } else {
                            self.regs.r[i] = val;
                        }
                        addr = addr.wrapping_add(4);
                    }
                }
                self.regs.set_sp(addr);
                if let Some(pc_val) = popped_pc {
                    // EXC_RETURN magic in Handler mode → exception return.
                    if self.regs.in_handler_mode() && Self::is_exc_return(pc_val) {
                        self.exit_exception(pc_val, bus);
                    } else if pc_val & 1 == 0 {
                        self.pending_fault = Some(Fault::InvalidEpsr);
                        return 1 + count;
                    } else {
                        self.regs.set_pc(pc_val & !1);
                    }
                }
                if pop_pc { 1 + count + 3 } else { 1 + count }
            }
            0b1110 => {
                // BKPT #imm8 — with no debugger attached this raises
                // HardFault (ARMv6-M ARM §A6.7.16). The Phase 4.B fault
                // path translates this into exception #3 entry.
                self.pending_fault = Some(Fault::HardFault);
                1
            }
            0b1111 => {
                // Hints only on M0+. IT-block encodings (mask != 0) are
                // UNDEFINED.
                let mask = opcode & 0xF;
                if mask != 0 {
                    return self.thumb16_undefined(opcode);
                }
                let hint_op = (opcode >> 4) & 0xF;
                match hint_op {
                    0x0 => 1, // NOP
                    0x1 => 1, // YIELD
                    0x2 => 1, // WFE — Phase 5 will wire up the SEV/WFE event flag
                    0x3 => 1, // WFI
                    0x4 => 1, // SEV
                    _ => 1,   // Reserved hints: execute as NOP
                }
            }
            _ => {
                // Remaining misc sub-ops on M33 are CBZ/CBNZ (op & 0x5 == 0x1)
                // — UNDEFINED on ARMv6-M. Anything else is also undefined.
                self.thumb16_undefined(opcode)
            }
        }
    }

    // ========================================================================
    // Thumb-16: Load/store multiple (LDMIA / STMIA)
    // ========================================================================

    /// STM Rn!, {reglist} (`11000_Rn_reglist`).
    pub(crate) fn thumb16_stm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rn = ((opcode >> 8) & 0x7) as usize;
        let reglist = (opcode & 0xFF) as u32;
        let count = reglist.count_ones();
        let mut addr = self.regs.r[rn];
        if !is_aligned(addr, 4) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }

        for i in 0..8 {
            if reglist & (1 << i) != 0 {
                bus.write32(addr, self.regs.r[i]);
                addr = addr.wrapping_add(4);
            }
        }
        self.regs.r[rn] = addr;
        1 + count
    }

    /// LDM Rn!, {reglist} (`11001_Rn_reglist`).
    /// Writeback only when Rn is NOT in the register list.
    pub(crate) fn thumb16_ldm(&mut self, opcode: u16, bus: &mut Bus) -> u32 {
        let rn = ((opcode >> 8) & 0x7) as usize;
        let reglist = (opcode & 0xFF) as u32;
        let count = reglist.count_ones();
        let mut addr = self.regs.r[rn];
        if !is_aligned(addr, 4) {
            self.pending_fault = Some(Fault::Unaligned);
            return 1;
        }

        for i in 0..8 {
            if reglist & (1 << i) != 0 {
                self.regs.r[i] = bus.read32(addr);
                addr = addr.wrapping_add(4);
            }
        }
        if reglist & (1 << rn) == 0 {
            self.regs.r[rn] = addr;
        }
        1 + count
    }

    // ========================================================================
    // Thumb-16: Conditional branch / SVC
    // ========================================================================

    /// B<cond> and SVC (`1101_cond_imm8`).
    /// cond == 0xE (AL) is UNDEFINED on ARMv6-M — the unconditional
    /// branch uses the separate `11100` encoding.
    /// cond == 0xF is SVC #imm8.
    pub(crate) fn thumb16_cond_branch_svc(&mut self, opcode: u16) -> u32 {
        let cond = ((opcode >> 8) & 0xF) as u8;
        match cond {
            0xE => self.thumb16_undefined(opcode),
            0xF => {
                // SVC #imm8 — deliver exception 11 via the fault path.
                self.pending_fault = Some(Fault::Svc);
                1
            }
            _ => {
                if self.regs.condition_passed(cond) {
                    let imm8 = (opcode & 0xFF) as u32;
                    let offset = sign_extend(imm8 << 1, 9);
                    let target = self.read_pc().wrapping_add(offset);
                    self.regs.set_pc(target);
                    3 // pipeline flush on taken branch
                } else {
                    1
                }
            }
        }
    }

    // ========================================================================
    // Thumb-16: Unconditional branch
    // ========================================================================

    /// B label (`11100_imm11`). 11-bit signed offset << 1.
    pub(crate) fn thumb16_branch(&mut self, opcode: u16) -> u32 {
        let imm11 = (opcode & 0x7FF) as u32;
        let offset = sign_extend(imm11 << 1, 12);
        let target = self.read_pc().wrapping_add(offset);
        self.regs.set_pc(target);
        3 // pipeline flush
    }

    // ========================================================================
    // Thumb-16: Undefined
    // ========================================================================

    /// Undefined instruction — raises HardFault on ARMv6-M.
    pub(crate) fn thumb16_undefined(&mut self, _opcode: u16) -> u32 {
        self.pending_fault = Some(Fault::Undefined);
        1
    }
}

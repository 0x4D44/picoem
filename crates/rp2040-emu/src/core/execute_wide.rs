//! ARMv6-M Thumb-32 executor — M0+ supports exactly six wide
//! encodings:
//!
//! * **BL** (immediate)   `11110 S imm10  :  11 J1 1 J2 imm11`
//! * **MRS** Rd, SYSm     `11110 0111 11 1 1111  :  10 00 Rd SYSm`
//! * **MSR** SYSm, Rn     `11110 0111 00 0 Rn    :  10 00 (1000) SYSm`
//! * **DSB** #option      `11110 0111 011 1111   :  10 00 1111 0100 option`
//! * **DMB** #option      `11110 0111 011 1111   :  10 00 1111 0101 option`
//! * **ISB** #option      `11110 0111 011 1111   :  10 00 1111 0110 option`
//!
//! Any other encoding reaching [`CortexM0Plus::execute_thumb32`] is
//! UNDEFINED → HardFault.
//!
//! Structure mirrors the rp2350_emu `execute_thumb32` module for
//! pattern-recognition but drops everything outside the M0+ subset (no
//! wide data-processing, no LDRD, no coprocessor, no IT-block awareness).

use super::{CoreBus, CortexM0Plus, Fault};

impl CortexM0Plus {
    /// Top-level Thumb-32 dispatch. `hw0` is the first halfword (with
    /// the `0b11110` prefix already validated by the decoder).
    pub(crate) fn execute_thumb32<B: CoreBus>(&mut self, hw0: u16, hw1: u16, _bus: &mut B) -> u32 {
        // Bits [15:11] are `0b11110`. The op[2] bit is hw1[15]; op[1:0]
        // from hw1[13:12]. On ARMv6-M only the BL / misc-control
        // encodings are defined (ARM DDI 0419 §A6.3).

        // BL: hw1[15:14] = 11, hw1[12] = 1 → the three-bit op field
        // (hw1[15]=1, hw1[14]=1, hw1[12]=1) is what ARMv7-M/v8-M call
        // "11_1" which is BL on all Cortex-M variants.
        if (hw1 & 0xD000) == 0xD000 {
            return self.thumb32_bl(hw0, hw1);
        }

        // Misc control / MRS / MSR / barriers: hw1[15:14] = 10 and the
        // op field in hw0[10:4] selects the sub-encoding.
        if (hw1 & 0xD000) == 0x8000 {
            return self.thumb32_misc_control(hw0, hw1);
        }

        // Anything else in the 0b11110 space is undefined on M0+.
        self.thumb32_undefined(hw0, hw1)
    }

    // =====================================================================
    // BL (immediate)
    // =====================================================================

    /// BL #imm24 — store return address in LR, jump to target.
    ///
    /// Encoding (ARM DDI 0419 §A6.7.13):
    ///   hw0 = `11110 S imm10`
    ///   hw1 = `11 J1 1 J2 imm11`
    /// Target offset = SignExtend(S:I1:I2:imm10:imm11:0, 25) where
    /// I1 = NOT(J1 XOR S), I2 = NOT(J2 XOR S).
    #[inline]
    pub(crate) fn thumb32_bl(&mut self, hw0: u16, hw1: u16) -> u32 {
        let s = ((hw0 >> 10) & 1) as u32;
        let imm10 = (hw0 & 0x3FF) as u32;
        let j1 = ((hw1 >> 13) & 1) as u32;
        let j2 = ((hw1 >> 11) & 1) as u32;
        let imm11 = (hw1 & 0x7FF) as u32;

        let i1 = (j1 ^ s) ^ 1;
        let i2 = (j2 ^ s) ^ 1;

        let imm25 = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
        let offset = super::execute::sign_extend(imm25, 25);

        // LR = address of the next instruction with the Thumb bit set.
        // decode_execute has already advanced PC past the wide encoding,
        // so `regs.pc()` is the return address.
        let next_instr = self.regs.pc() | 1;
        self.regs.set_lr(next_instr);

        let target = self.read_pc().wrapping_add(offset);
        // BL always targets Thumb code — bit 0 is the T bit in the
        // encoded address. On M0+ we clear it before writing PC.
        self.regs.set_pc(target & !1);
        4 // M0+ measured: branch-with-link pipeline flush
    }

    // =====================================================================
    // Misc control: MRS / MSR / DSB / DMB / ISB
    // =====================================================================

    /// Dispatch the misc-control group (hw1[15:14] = 10).
    #[inline]
    fn thumb32_misc_control(&mut self, hw0: u16, hw1: u16) -> u32 {
        // Barriers — DDI 0419 §A6.7.14: hw0 == 0xF3BF, hw1[15:12] == 0x8,
        // hw1[11:8] == 0xF.
        if hw0 == 0xF3BF && (hw1 & 0xFF00) == 0x8F00 {
            let barrier_op = (hw1 >> 4) & 0xF;
            return match barrier_op {
                0x4 => 1, // DSB
                0x5 => 1, // DMB
                // ISB: flush the per-core decode cache so any
                // self-modifying or cross-core writes that landed
                // before this barrier are re-fetched. Mirrors the
                // rp2350_emu ISB handler (commit 0c31479).
                0x6 => {
                    self.invalidate_decode_cache_all();
                    1
                }
                _ => self.thumb32_undefined(hw0, hw1),
            };
        }

        // MSR: hw0[10:4] = 0b011100_x and hw1[15:8] = 0b10001000.
        // hw1 bits [11:8] are reserved `1000` (mask) on M0+; any other
        // pattern is UNDEFINED. `x` = bit [4] is the register-select R
        // subfield on M0+ (reserved on some variants; accept either).
        let op_field = (hw0 >> 4) & 0x7F;
        if (op_field == 0b0111000 || op_field == 0b0111001) && (hw1 & 0xFF00) == 0x8800 {
            return self.thumb32_msr(hw0, hw1);
        }

        // MRS: hw0[10:4] = 0b011111_x, hw0[3:0] = 0b1111, hw1[15:12] = 0b1000.
        if (op_field == 0b0111110 || op_field == 0b0111111)
            && (hw0 & 0xF) == 0xF
            && (hw1 & 0xF000) == 0x8000
        {
            return self.thumb32_mrs(hw1);
        }

        self.thumb32_undefined(hw0, hw1)
    }

    /// MSR SYSm, Rn — write general register into a system register.
    ///
    /// SYSm values recognised on M0+ (ARMv6-M ARM §B5.2.3):
    ///   0  = APSR  (NZCV flags only — no Q, no GE on v6-M)
    ///   3  = xPSR  (same subset as APSR for writes)
    ///   5  = IPSR  (read-only, write ignored)
    ///   8  = MSP
    ///   9  = PSP
    ///   16 = PRIMASK
    ///   20 = CONTROL
    ///
    /// All other SYSm values (including 1, 2, 4, 6, 7, 10..15, 17..19,
    /// 21..255) are RESERVED on ARMv6-M and raise HardFault via the
    /// Undefined fault path.
    fn thumb32_msr(&mut self, hw0: u16, hw1: u16) -> u32 {
        let rn = (hw0 & 0xF) as usize;
        let sysm = (hw1 & 0xFF) as u8;
        let val = self.regs.r[rn];

        match sysm {
            // APSR / xPSR — write NZCV only (M0+ has no Q, no GE).
            0 | 3 => {
                self.regs.xpsr = (self.regs.xpsr & !0xF000_0000) | (val & 0xF000_0000);
            }
            // IPSR — read-only, write ignored.
            5 => {}
            // MSP — banked stack pointer. If we're currently executing
            // with MSP as the active SP, reflect the write into r[13].
            8 => {
                self.regs.msp = val;
                if !self.regs.active_sp_is_psp() {
                    self.regs.r[13] = val;
                }
            }
            // PSP — same pattern for PSP.
            9 => {
                self.regs.psp = val;
                if self.regs.active_sp_is_psp() {
                    self.regs.r[13] = val;
                }
            }
            // PRIMASK — only bit 0 is architected.
            16 => {
                self.regs.primask = val & 1;
            }
            // CONTROL — only SPSEL (bit 1) and nPRIV (bit 0) are
            // architected on M0+. Handler mode ignores SPSEL writes.
            20 => {
                let mut new_ctrl = val & 0x3;
                if self.regs.in_handler_mode() {
                    // Handler mode: SPSEL is RAZ/WI.
                    new_ctrl &= !0x2;
                    new_ctrl |= self.regs.control & 0x2;
                }
                self.regs.sync_sp_to_banked();
                self.regs.control = new_ctrl;
                self.regs.sync_sp_from_banked();
            }
            // Reserved SYSm — UNPREDICTABLE on M0+. Blueprint says
            // HardFault; surface as Undefined so the fault path delivers
            // as HardFault.
            _ => {
                self.pending_fault = Some(Fault::Undefined);
            }
        }
        2
    }

    /// MRS Rd, SYSm — read a system register into a general register.
    ///
    /// SYSm values accepted match [`thumb32_msr`]; every other value is
    /// RESERVED on ARMv6-M and raises HardFault.
    fn thumb32_mrs(&mut self, hw1: u16) -> u32 {
        let rd = ((hw1 >> 8) & 0xF) as usize;
        let sysm = (hw1 & 0xFF) as u8;

        let value = match sysm {
            // APSR / xPSR — NZCV flags only on M0+ (no Q, no GE).
            0 | 3 => self.regs.xpsr & 0xF000_0000,
            // IPSR — exception number in [8:0].
            5 => self.regs.xpsr & 0x1FF,
            // MSP — when MSP is the active SP the architectural value
            // lives in r[13]; the cached `regs.msp` is only authoritative
            // while MSP is the inactive bank. Mirrors the symmetric write
            // path on line 156.
            8 => {
                if self.regs.active_sp_is_psp() {
                    self.regs.msp
                } else {
                    self.regs.r[13]
                }
            }
            // PSP — symmetric: live in r[13] when PSP-active, else cached.
            9 => {
                if self.regs.active_sp_is_psp() {
                    self.regs.r[13]
                } else {
                    self.regs.psp
                }
            }
            // PRIMASK
            16 => self.regs.primask & 1,
            // CONTROL
            20 => self.regs.control & 0x3,
            // Reserved SYSm — HardFault via Undefined.
            _ => {
                self.pending_fault = Some(Fault::Undefined);
                return 2;
            }
        };
        self.regs.r[rd] = value;
        2
    }

    // =====================================================================
    // Undefined 32-bit encoding
    // =====================================================================

    /// Undefined Thumb-32 encoding → HardFault via pending-fault path.
    #[inline]
    pub(crate) fn thumb32_undefined(&mut self, _hw0: u16, _hw1: u16) -> u32 {
        self.pending_fault = Some(Fault::Undefined);
        1
    }
}

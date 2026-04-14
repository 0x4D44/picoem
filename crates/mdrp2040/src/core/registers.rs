//! Cortex-M0+ register file (ARMv6-M).
//!
//! Strict subset of the `mdrp2350::core::registers` M33 set: no banked
//! non-secure copies, no stack-pointer limits, no BASEPRI / FAULTMASK,
//! no GE flags, no FPU. `control` holds only two architecturally
//! meaningful bits (`SPSEL`, `nPRIV`).

/// xPSR flag bit positions (same layout as ARMv7-M / ARMv8-M).
pub const XPSR_N: u32 = 1 << 31;
pub const XPSR_Z: u32 = 1 << 30;
pub const XPSR_C: u32 = 1 << 29;
pub const XPSR_V: u32 = 1 << 28;
pub const XPSR_T: u32 = 1 << 24;

/// Cortex-M0+ register file.
///
/// `repr(C)` mirrors mdrp2350 for cache-line discipline. M0+ has no FPU,
/// so S0-S31 / FPSCR are absent — giving a much smaller footprint than
/// the M33 register file.
#[repr(C)]
pub struct Registers {
    /// R0-R12, SP (R13), LR (R14), PC (R15).
    pub r: [u32; 16],
    /// Combined APSR + IPSR + EPSR. On M0+ only N/Z/C/V/T and
    /// IPSR[8:0] are architecturally defined.
    pub xpsr: u32,
    /// Interrupt mask. M0+ has PRIMASK only — no BASEPRI or FAULTMASK.
    pub primask: u32,
    /// CONTROL register. M0+ defines two bits:
    ///   bit 0 — nPRIV (0 = privileged, 1 = unprivileged in thread mode)
    ///   bit 1 — SPSEL (0 = MSP, 1 = PSP) — only meaningful in thread mode
    pub control: u32,
    /// Main Stack Pointer (banked when `CONTROL.SPSEL == 1`).
    pub msp: u32,
    /// Process Stack Pointer (banked when `CONTROL.SPSEL == 1`).
    pub psp: u32,
}

impl Registers {
    /// Create a register file in reset state (Thumb bit set).
    pub fn new() -> Self {
        Self {
            r: [0; 16],
            xpsr: XPSR_T,
            primask: 0,
            control: 0,
            msp: 0,
            psp: 0,
        }
    }

    // --- Flag accessors ---

    #[inline(always)]
    pub fn flag_n(&self) -> bool {
        self.xpsr & XPSR_N != 0
    }

    #[inline(always)]
    pub fn flag_z(&self) -> bool {
        self.xpsr & XPSR_Z != 0
    }

    #[inline(always)]
    pub fn flag_c(&self) -> bool {
        self.xpsr & XPSR_C != 0
    }

    #[inline(always)]
    pub fn flag_v(&self) -> bool {
        self.xpsr & XPSR_V != 0
    }

    #[inline(always)]
    pub fn set_flag_n(&mut self, v: bool) {
        if v {
            self.xpsr |= XPSR_N;
        } else {
            self.xpsr &= !XPSR_N;
        }
    }

    #[inline(always)]
    pub fn set_flag_z(&mut self, v: bool) {
        if v {
            self.xpsr |= XPSR_Z;
        } else {
            self.xpsr &= !XPSR_Z;
        }
    }

    #[inline(always)]
    pub fn set_flag_c(&mut self, v: bool) {
        if v {
            self.xpsr |= XPSR_C;
        } else {
            self.xpsr &= !XPSR_C;
        }
    }

    #[inline(always)]
    pub fn set_flag_v(&mut self, v: bool) {
        if v {
            self.xpsr |= XPSR_V;
        } else {
            self.xpsr &= !XPSR_V;
        }
    }

    /// Set N and Z flags from a 32-bit result.
    #[inline(always)]
    pub fn set_nz(&mut self, result: u32) {
        self.set_flag_n(result & 0x8000_0000 != 0);
        self.set_flag_z(result == 0);
    }

    /// Set all four condition flags.
    #[inline(always)]
    pub fn set_nzcv(&mut self, n: bool, z: bool, c: bool, v: bool) {
        self.xpsr &= !(XPSR_N | XPSR_Z | XPSR_C | XPSR_V);
        if n {
            self.xpsr |= XPSR_N;
        }
        if z {
            self.xpsr |= XPSR_Z;
        }
        if c {
            self.xpsr |= XPSR_C;
        }
        if v {
            self.xpsr |= XPSR_V;
        }
    }

    // --- Named register accessors ---

    #[inline(always)]
    pub fn sp(&self) -> u32 {
        self.r[13]
    }

    #[inline(always)]
    pub fn set_sp(&mut self, v: u32) {
        self.r[13] = v;
    }

    #[inline(always)]
    pub fn lr(&self) -> u32 {
        self.r[14]
    }

    #[inline(always)]
    pub fn set_lr(&mut self, v: u32) {
        self.r[14] = v;
    }

    #[inline(always)]
    pub fn pc(&self) -> u32 {
        self.r[15]
    }

    #[inline(always)]
    pub fn set_pc(&mut self, v: u32) {
        self.r[15] = v;
    }

    /// IPSR field (exception number, bits [8:0]).
    #[inline(always)]
    pub fn ipsr(&self) -> u32 {
        self.xpsr & 0x1FF
    }

    /// True if the processor is in handler mode (IPSR != 0).
    #[inline(always)]
    pub fn in_handler_mode(&self) -> bool {
        self.ipsr() != 0
    }

    // --- SP banking helpers ---

    /// Returns true if the active SP is PSP (thread mode + `CONTROL.SPSEL == 1`).
    /// Handler mode always uses MSP regardless of SPSEL.
    pub fn active_sp_is_psp(&self) -> bool {
        !self.in_handler_mode() && self.control & 2 != 0
    }

    /// Sync R13 to the appropriate banked SP before switching.
    pub fn sync_sp_to_banked(&mut self) {
        if self.active_sp_is_psp() {
            self.psp = self.r[13];
        } else {
            self.msp = self.r[13];
        }
    }

    /// Sync R13 from the appropriate banked SP after switching.
    pub fn sync_sp_from_banked(&mut self) {
        self.r[13] = if self.active_sp_is_psp() { self.psp } else { self.msp };
    }

    /// Evaluate an ARM condition code (cond in `[0..0xE]`).
    ///
    /// ARMv6-M only uses condition codes on the `B<cond>` branch
    /// encoding. `0xF` is reserved (used as the SVC encoding) and
    /// never passed here; `0xE` (AL) is handled as a short-circuit.
    #[inline(always)]
    pub fn condition_passed(&self, cond: u8) -> bool {
        if cond >= 0xE {
            return true;
        }
        match cond & 0xF {
            0x0 => self.flag_z(),                                      // EQ
            0x1 => !self.flag_z(),                                     // NE
            0x2 => self.flag_c(),                                      // CS/HS
            0x3 => !self.flag_c(),                                     // CC/LO
            0x4 => self.flag_n(),                                      // MI
            0x5 => !self.flag_n(),                                     // PL
            0x6 => self.flag_v(),                                      // VS
            0x7 => !self.flag_v(),                                     // VC
            0x8 => self.flag_c() && !self.flag_z(),                    // HI
            0x9 => !self.flag_c() || self.flag_z(),                    // LS
            0xA => self.flag_n() == self.flag_v(),                     // GE
            0xB => self.flag_n() != self.flag_v(),                     // LT
            0xC => !self.flag_z() && (self.flag_n() == self.flag_v()), // GT
            0xD => self.flag_z() || (self.flag_n() != self.flag_v()),  // LE
            _ => true,
        }
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}

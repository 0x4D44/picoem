/// xPSR flag bit positions.
pub const XPSR_N: u32 = 1 << 31;
pub const XPSR_Z: u32 = 1 << 30;
pub const XPSR_C: u32 = 1 << 29;
pub const XPSR_V: u32 = 1 << 28;
pub const XPSR_Q: u32 = 1 << 27;
pub const XPSR_T: u32 = 1 << 24;

/// Cortex-M33 register file.
///
/// Layout is `repr(C)` for cache locality. The entire struct fits in 4 cache
/// lines (232 bytes < 256).
#[repr(C)]
pub struct Registers {
    /// R0-R12, SP (R13), LR (R14), PC (R15).
    pub r: [u32; 16],
    /// Combined APSR + IPSR + EPSR.
    pub xpsr: u32,
    pub primask: u32,
    pub basepri: u32,
    pub faultmask: u32,
    pub control: u32,
    /// Main Stack Pointer (banked).
    pub msp: u32,
    /// Process Stack Pointer (banked).
    pub psp: u32,
    /// Non-secure MSP (TrustZone — stub in v1).
    pub msp_ns: u32,
    /// Non-secure PSP (TrustZone — stub in v1).
    pub psp_ns: u32,
    /// FPU single-precision registers S0-S31.
    pub s: [f32; 32],
    /// FP status/control register.
    pub fpscr: u32,
}

impl Registers {
    /// Create a register file in reset state. T bit is set (Thumb mode).
    pub fn new() -> Self {
        let mut regs = Self {
            r: [0; 16],
            xpsr: XPSR_T, // Thumb mode always on
            primask: 0,
            basepri: 0,
            faultmask: 0,
            control: 0,
            msp: 0,
            psp: 0,
            msp_ns: 0,
            psp_ns: 0,
            s: [0.0; 32],
            fpscr: 0,
        };
        // R13 mirrors MSP at reset (CONTROL.SPSEL = 0)
        regs.r[13] = regs.msp;
        regs
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
    pub fn flag_q(&self) -> bool {
        self.xpsr & XPSR_Q != 0
    }

    /// Set the Q (saturation) flag. Q is sticky — once set it stays set
    /// until explicitly cleared via MSR.
    #[inline(always)]
    pub fn set_flag_q(&mut self) {
        self.xpsr |= XPSR_Q;
    }

    /// Read GE[3:0] flags from xPSR[19:16].
    #[inline(always)]
    pub fn ge_flags(&self) -> u32 {
        (self.xpsr >> 16) & 0xF
    }

    /// Write GE[3:0] flags into xPSR[19:16].
    #[inline(always)]
    pub fn set_ge_flags(&mut self, ge: u32) {
        self.xpsr = (self.xpsr & !0x000F_0000) | ((ge & 0xF) << 16);
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
        // Clear all four, then set the ones that are true.
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

    /// Returns true if the active SP is PSP (Thread mode + SPSEL=1).
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

    /// Evaluate an ARM condition code against current flags.
    pub fn condition_passed(&self, cond: u8) -> bool {
        match cond & 0xF {
            0x0 => self.flag_z(),                                     // EQ
            0x1 => !self.flag_z(),                                    // NE
            0x2 => self.flag_c(),                                     // CS/HS
            0x3 => !self.flag_c(),                                    // CC/LO
            0x4 => self.flag_n(),                                     // MI
            0x5 => !self.flag_n(),                                    // PL
            0x6 => self.flag_v(),                                     // VS
            0x7 => !self.flag_v(),                                    // VC
            0x8 => self.flag_c() && !self.flag_z(),                   // HI
            0x9 => !self.flag_c() || self.flag_z(),                   // LS
            0xA => self.flag_n() == self.flag_v(),                    // GE
            0xB => self.flag_n() != self.flag_v(),                    // LT
            0xC => !self.flag_z() && (self.flag_n() == self.flag_v()), // GT
            0xD => self.flag_z() || (self.flag_n() != self.flag_v()), // LE
            0xE => true,                                              // AL
            _ => true,                                                // unconditional
        }
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}

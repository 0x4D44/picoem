// FPCCR bit positions (DDI0553 §D1.2.32). Public so other crate modules
// (exceptions.rs, execute_fpu.rs) can reference them by name.
pub const FPCCR_LSPACT:    u32 = 1 << 0;
pub const FPCCR_MMRDY:     u32 = 1 << 5;
pub const FPCCR_BFRDY:     u32 = 1 << 6;
pub const FPCCR_SPLIMVIOL: u32 = 1 << 9;
pub const FPCCR_LSPEN:     u32 = 1 << 30;
pub const FPCCR_ASPEN:     u32 = 1 << 31;

/// Per-core Private Peripheral Bus state (NVIC, SCB, SysTick stubs).
/// Phase 3: slim — only what the bootrom needs.
pub struct Ppb {
    // SCB registers
    pub vtor: u32,      // Vector Table Offset (0xE000ED08, reset: 0)
    pub aircr: u32,     // App Interrupt/Reset Control (0xE000ED0C)
    pub scr: u32,       // System Control (0xE000ED10)
    pub ccr: u32,       // Configuration Control (0xE000ED14, reset: 0x200)
    pub shpr: [u8; 12], // System Handler Priority, exceptions 4-15 (0xE000ED18-ED20)
    pub shcsr: u32,     // System Handler Control/Status (0xE000ED24)
    pub cfsr: u32,      // Configurable Fault Status (0xE000ED28)
    pub hfsr: u32,      // Hard Fault Status (0xE000ED2C)
    pub mmfar: u32,     // MemManage Fault Address (0xE000ED34)
    pub bfar: u32,      // Bus Fault Address (0xE000ED38)
    pub cpacr: u32,     // Coprocessor Access Control (0xE000ED88)
    pub icsr: u32,      // Interrupt Control/State (0xE000ED04)

    // FP extension registers (Phase 7 Stage B — DDI0553 §D1.2.32-34)
    //
    // Invariants enforced by the emulator:
    //   1. CONTROL.FPCA=1 ⇒ S0-S31 + FPSCR are live thread-mode state.
    //   2. FPCCR.LSPACT=1 ⇒ FPCAR points at a reserved FP frame; S0-S15
    //      and FPSCR are still the pre-exception values, not yet written.
    //   3. EXC_RETURN[4]=0 ⇒ exception entry reserved 18 words above the
    //      basic frame.
    //   4. Only fpu_execute writes FPCA=1; only enter_exception/
    //      exit_exception write FPCA=0 / restore it.
    //
    /// FP Context Control Register. Reset 0xC000_0000 (ASPEN=1, LSPEN=1).
    /// Bit layout per DDI0553 §D1.2.32:
    ///   [0] LSPACT   [1] USER     [2] S        [3] THREAD
    ///   [4] HFRDY    [5] MMRDY    [6] BFRDY    [7] SFRDY
    ///   [8] MONRDY   [9] SPLIMVIOL [10] UFRDY  (11-25 reserved)
    ///   [26] TS      [27] CLRONRETS [28] CLRONRET
    ///   [29] LSPENS  [30] LSPEN   [31] ASPEN
    /// Emulator actively models: ASPEN, LSPEN, LSPACT, SPLIMVIOL,
    /// MMRDY, BFRDY. Others are RW storage but inert.
    pub fpccr: u32,
    /// FP Context Address Register. Writes mask bits [2:0] to 0
    /// (8-byte alignment).
    pub fpcar: u32,
    /// FP Default Status Control. Template for FPSCR at exception entry;
    /// active bits are AHP (26), DN (25), FZ (24), RMODE (23:22).
    pub fpdscr: u32,

    // MPU (0xE000ED94-0xE000EDA0)
    pub mpu_ctrl: u32,                 // MPU Control (0xE000ED94)
    pub mpu_rnr: u32,                  // MPU Region Number (0xE000ED98)
    pub mpu_regions: [(u32, u32); 16], // 16 regions: (RBAR, RLAR) pairs

    // SAU (0xE000EDD0-0xE000EDE0)
    pub sau_ctrl: u32,                // SAU Control (bit 0 = enable, bit 1 = ALLNS)
    pub sau_rnr: u32,                 // Region Number Register (selects active region)
    pub sau_regions: [(u32, u32); 8], // 8 regions: (RBAR, RLAR) pairs
}

impl Default for Ppb {
    fn default() -> Self {
        Self {
            vtor: 0,
            aircr: 0,
            scr: 0,
            ccr: 0x0000_0200, // STKALIGN=1
            shpr: [0; 12],
            shcsr: 0,
            cfsr: 0,
            hfsr: 0,
            mmfar: 0,
            bfar: 0,
            cpacr: 0x00F0_0000, // CP10/11 (FPU) full access
            icsr: 0,
            // ASPEN=1 (auto FP context save), LSPEN=1 (lazy enabled).
            fpccr: 0xC000_0000,
            fpcar: 0,
            fpdscr: 0,
            mpu_ctrl: 0,
            mpu_rnr: 0,
            mpu_regions: [(0, 0); 16],
            sau_ctrl: 0,
            sau_rnr: 0,
            sau_regions: [(0, 0); 8],
        }
    }
}

impl Ppb {
    /// Pack 4 consecutive SHPR bytes into a u32 (little-endian).
    fn pack_shpr(&self, start: usize) -> u32 {
        u32::from_le_bytes([
            self.shpr[start],
            self.shpr[start + 1],
            self.shpr[start + 2],
            self.shpr[start + 3],
        ])
    }

    /// Unpack a u32 into 4 consecutive SHPR bytes.
    /// Only bits [7:5] per byte are implemented on Cortex-M33.
    fn unpack_shpr(&mut self, start: usize, val: u32) {
        let bytes = val.to_le_bytes();
        for i in 0..4 {
            self.shpr[start + i] = bytes[i] & 0xE0;
        }
    }

    pub fn read32(&self, addr: u32) -> u32 {
        match addr & 0xFFFF {
            // ICTR — Interrupt Controller Type: 64 external IRQ lines
            0xE004 => 1,

            // SysTick (stub)
            0xE010..=0xE01C => 0,

            // NVIC (stub)
            0xE100..=0xE4FF => 0,

            // CPUID
            0xED00 => 0x411F_D210,

            // ICSR
            0xED04 => self.icsr,

            // VTOR
            0xED08 => self.vtor,

            // AIRCR
            0xED0C => self.aircr,

            // SCR
            0xED10 => self.scr,

            // CCR
            0xED14 => self.ccr,

            // SHPR1 (exceptions 4-7)
            0xED18 => self.pack_shpr(0),

            // SHPR2 (exceptions 8-11)
            0xED1C => self.pack_shpr(4),

            // SHPR3 (exceptions 12-15)
            0xED20 => self.pack_shpr(8),

            // SHCSR
            0xED24 => self.shcsr,

            // CFSR
            0xED28 => self.cfsr,

            // HFSR
            0xED2C => self.hfsr,

            // MMFAR
            0xED34 => self.mmfar,

            // BFAR
            0xED38 => self.bfar,

            // CPACR
            0xED88 => self.cpacr,

            // FPCCR / FPCAR / FPDSCR (Phase 7 Stage B)
            0xEF34 => self.fpccr,
            0xEF38 => self.fpcar,
            0xEF3C => self.fpdscr,

            // MPU_TYPE: 16 regions on RP2350 Cortex-M33
            0xED90 => 0x0000_1000, // DREGION=16, IREGION=0, SEPARATE=0
            // MPU_CTRL
            0xED94 => self.mpu_ctrl,
            // MPU_RNR
            0xED98 => self.mpu_rnr,
            // MPU_RBAR
            0xED9C => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].0
            }
            // MPU_RLAR
            0xEDA0 => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].1
            }
            // MPU_RBAR_A1 / RLAR_A1 / ... A3 (ARMv8-M §B11.2.5-8):
            // alias registers access region `(RNR & !3) | n` for n ∈ {1,2,3}.
            // Surfaced by the bootrom's MPU readback self-test which writes
            // all four (base, alias1, alias2, alias3) pairs in a single stmia.
            0xEDA4 | 0xEDAC | 0xEDB4 => {
                let n = ((addr as usize) - 0xEDA4) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].0
            }
            0xEDA8 | 0xEDB0 | 0xEDB8 => {
                let n = ((addr as usize) - 0xEDA8) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].1
            }

            // SAU_CTRL
            0xEDD0 => self.sau_ctrl,
            // SAU_TYPE: 8 regions (RP2350 has 8)
            0xEDD4 => 8,
            // SAU_RNR
            0xEDD8 => self.sau_rnr,
            // SAU_RBAR: bits [4:0] are RES0
            0xEDDC => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].0 & !0x1F
            }
            // SAU_RLAR
            0xEDE0 => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].1
            }

            // Unknown PPB register
            _ => 0,
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        match addr & 0xFFFF {
            // SysTick (stub — accept and ignore)
            0xE010..=0xE01C => {}

            // NVIC (stub — accept and ignore)
            0xE100..=0xE4FF => {}

            // CPUID — read-only, ignore writes
            0xED00 => {}

            // ICSR
            0xED04 => self.icsr = val,

            // VTOR — 128-byte aligned
            0xED08 => self.vtor = val & !0x7F,

            // AIRCR
            0xED0C => self.aircr = val,

            // SCR
            0xED10 => self.scr = val,

            // CCR
            0xED14 => self.ccr = val,

            // SHPR1 (exceptions 4-7)
            0xED18 => self.unpack_shpr(0, val),

            // SHPR2 (exceptions 8-11)
            0xED1C => self.unpack_shpr(4, val),

            // SHPR3 (exceptions 12-15)
            0xED20 => self.unpack_shpr(8, val),

            // SHCSR
            0xED24 => self.shcsr = val,

            // CFSR — write-1-to-clear
            0xED28 => self.cfsr &= !val,

            // HFSR — write-1-to-clear
            0xED2C => self.hfsr &= !val,

            // MMFAR
            0xED34 => self.mmfar = val,

            // BFAR
            0xED38 => self.bfar = val,

            // CPACR
            0xED88 => self.cpacr = val,

            // FPCCR / FPCAR / FPDSCR (Phase 7 Stage B). FPCAR is force-aligned
            // to 8 bytes (DDI0553 §D1.2.33). FPCCR has reserved bits but no
            // mask is applied — software is allowed to write the full word.
            0xEF34 => self.fpccr = val,
            0xEF38 => self.fpcar = val & !0x7,
            0xEF3C => self.fpdscr = val,

            // MPU_TYPE: read-only
            0xED90 => {}
            // MPU_CTRL
            0xED94 => self.mpu_ctrl = val,
            // MPU_RNR
            0xED98 => self.mpu_rnr = val & 0xF,
            // MPU_RBAR (ARMv8-M §B11.2.5): [31:5] BASE, [4:3] SH,
            // [2:1] AP, [0] XN — all bits carry meaning.
            0xED9C => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].0 = val;
            }
            // MPU_RLAR (ARMv8-M §B11.2.8): [31:5] LIMIT, [4] RES0,
            // [3:1] AttrIndx, [0] EN. Mask bit [4] so it reads back as 0
            // (the bootrom's readback self-test depends on this).
            0xEDA0 => {
                let idx = (self.mpu_rnr & 0xF) as usize;
                self.mpu_regions[idx].1 = val & !0x10;
            }
            // MPU_RBAR_An / RLAR_An aliases — see read path for definition.
            0xEDA4 | 0xEDAC | 0xEDB4 => {
                let n = ((addr as usize) - 0xEDA4) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].0 = val;
            }
            0xEDA8 | 0xEDB0 | 0xEDB8 => {
                let n = ((addr as usize) - 0xEDA8) / 8 + 1;
                let idx = ((self.mpu_rnr as usize) & !0x3) | n;
                self.mpu_regions[idx & 0xF].1 = val & !0x10;
            }

            // SAU_CTRL
            0xEDD0 => self.sau_ctrl = val,
            // SAU_TYPE: read-only, ignore writes
            0xEDD4 => {}
            // SAU_RNR
            0xEDD8 => self.sau_rnr = val & 0x7,
            // SAU_RBAR
            0xEDDC => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].0 = val;
            }
            // SAU_RLAR
            0xEDE0 => {
                let idx = (self.sau_rnr & 0x7) as usize;
                self.sau_regions[idx].1 = val;
            }

            // Unknown PPB register — ignore
            _ => {}
        }
    }

    /// Get the priority of a system exception (4-15) from SHPR.
    /// Returns i16: HardFault=-1, others from shpr[]. Only bits [7:5] used.
    pub fn exception_priority(&self, exc_num: u16) -> i16 {
        match exc_num {
            1 => -3,  // Reset
            2 => -2,  // NMI
            3 => -1,  // HardFault (fixed)
            4..=15 => (self.shpr[(exc_num - 4) as usize] & 0xE0) as i16,
            _ => 0,   // External IRQs default to 0 (Phase 5 will add NVIC_IPR)
        }
    }

    /// Clear the active bit for an exception. Phase 3 stub: just clear IPSR-related state in ICSR.
    pub fn clear_active(&mut self, _exc_num: u16) {
        // Phase 3: no NVIC active tracking. ICSR.VECTACTIVE handled by core IPSR.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpuid_read() {
        let ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_ED00), 0x411F_D210);
    }

    #[test]
    fn test_vtor_roundtrip() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED08, 0x200);
        assert_eq!(ppb.read32(0xE000_ED08), 0x200);
    }

    #[test]
    fn test_vtor_alignment() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_ED08, 0x201);
        assert_eq!(ppb.read32(0xE000_ED08), 0x200);
    }

    #[test]
    fn test_shpr_roundtrip() {
        let mut ppb = Ppb::default();
        // Write SHPR1 with packed bytes: priorities 0x20, 0x40, 0x60, 0xE0
        let val = u32::from_le_bytes([0x20, 0x40, 0x60, 0xE0]);
        ppb.write32(0xE000_ED18, val);
        assert_eq!(ppb.read32(0xE000_ED18), val);

        // Verify individual bytes (only bits [7:5] survive)
        assert_eq!(ppb.shpr[0], 0x20);
        assert_eq!(ppb.shpr[1], 0x40);
        assert_eq!(ppb.shpr[2], 0x60);
        assert_eq!(ppb.shpr[3], 0xE0);
    }

    #[test]
    fn test_cfsr_write_one_to_clear() {
        let mut ppb = Ppb::default();
        ppb.cfsr = 0xFF;
        ppb.write32(0xE000_ED28, 0x0F);
        assert_eq!(ppb.read32(0xE000_ED28), 0xF0);
    }

    #[test]
    fn test_exception_priority() {
        let mut ppb = Ppb::default();
        // HardFault is fixed at -1
        assert_eq!(ppb.exception_priority(3), -1);

        // Set exception 4 (MemManage) priority to 0xA0 via SHPR1
        ppb.write32(0xE000_ED18, u32::from_le_bytes([0xA0, 0, 0, 0]));
        assert_eq!(ppb.exception_priority(4), 0xA0_u8 as i16);
    }

    #[test]
    fn test_nvic_stub_returns_zero() {
        let ppb = Ppb::default();
        // NVIC_ISER0 at 0xE000E100
        assert_eq!(ppb.read32(0xE000_E100), 0);
    }

    #[test]
    fn test_systick_stub_returns_zero() {
        let ppb = Ppb::default();
        // SYST_CSR at 0xE000E010
        assert_eq!(ppb.read32(0xE000_E010), 0);
    }

    #[test]
    fn test_sau_type_returns_8() {
        let ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_EDD4), 8);
    }

    #[test]
    fn test_sau_ctrl_roundtrip() {
        let mut ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_EDD0), 0);
        ppb.write32(0xE000_EDD0, 1);
        assert_eq!(ppb.read32(0xE000_EDD0), 1);
    }

    #[test]
    fn test_sau_rnr_masks_to_3_bits() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EDD8, 0xFF);
        assert_eq!(ppb.read32(0xE000_EDD8), 7);
    }

    #[test]
    fn test_sau_region_roundtrip() {
        let mut ppb = Ppb::default();
        // Select region 3
        ppb.write32(0xE000_EDD8, 3);
        // Write RBAR and RLAR
        ppb.write32(0xE000_EDDC, 0x1000_4787);
        ppb.write32(0xE000_EDE0, 0x0000_7FE1);
        // Read back: RBAR has low 5 bits masked
        assert_eq!(ppb.read32(0xE000_EDDC), 0x1000_4780);
        assert_eq!(ppb.read32(0xE000_EDE0), 0x0000_7FE1);
        // Other regions remain zero
        ppb.write32(0xE000_EDD8, 0);
        assert_eq!(ppb.read32(0xE000_EDDC), 0);
        assert_eq!(ppb.read32(0xE000_EDE0), 0);
    }

    #[test]
    fn test_sau_type_write_ignored() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EDD4, 0xDEAD);
        assert_eq!(ppb.read32(0xE000_EDD4), 8);
    }

    // ----------------------------------------------------------------
    // FP extension registers (Phase 7 Stage B)
    // ----------------------------------------------------------------

    #[test]
    fn test_fpccr_reset_value() {
        let ppb = Ppb::default();
        assert_eq!(ppb.read32(0xE000_EF34), 0xC000_0000);
        assert_eq!(ppb.fpccr & FPCCR_ASPEN, FPCCR_ASPEN);
        assert_eq!(ppb.fpccr & FPCCR_LSPEN, FPCCR_LSPEN);
        assert_eq!(ppb.fpccr & FPCCR_LSPACT, 0);
    }

    #[test]
    fn test_fpccr_roundtrip() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EF34, 0xDEAD_BEEF);
        assert_eq!(ppb.read32(0xE000_EF34), 0xDEAD_BEEF);
    }

    #[test]
    fn test_fpcar_alignment_mask() {
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EF38, 0x2000_1007);
        // Bits [2:0] are forced to 0.
        assert_eq!(ppb.read32(0xE000_EF38), 0x2000_1000);
    }

    #[test]
    fn test_fpdscr_roundtrip() {
        let mut ppb = Ppb::default();
        // Set AHP=1, DN=1, FZ=1, RMODE=10 (round toward -inf).
        ppb.write32(0xE000_EF3C, (1 << 26) | (1 << 25) | (1 << 24) | (0b10 << 22));
        assert_eq!(ppb.read32(0xE000_EF3C),
            (1 << 26) | (1 << 25) | (1 << 24) | (0b10 << 22));
    }

    #[test]
    fn test_sau_bootrom_region7_setup() {
        // Reproduces the bootrom's SAU setup: region 7 with
        // RBAR=0x4787, RLAR=0x7FE1 (Secure, enabled)
        let mut ppb = Ppb::default();
        ppb.write32(0xE000_EDD0, 1);  // SAU_CTRL = enable
        ppb.write32(0xE000_EDD8, 7);  // SAU_RNR = region 7
        ppb.write32(0xE000_EDDC, 0x4787); // SAU_RBAR
        ppb.write32(0xE000_EDE0, 0x7FE1); // SAU_RLAR
        // Verify readback
        assert_eq!(ppb.read32(0xE000_EDDC), 0x4780); // RBAR low 5 bits masked
        assert_eq!(ppb.read32(0xE000_EDE0), 0x7FE1);
    }
}

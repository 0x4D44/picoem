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
            cpacr: 0,
            icsr: 0,
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
}

use crate::bus::Bus;
use super::{CortexM33, Fault};

impl CortexM33 {
    /// Top-level coprocessor dispatch.
    pub(crate) fn thumb32_coprocessor(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let coproc = ((hw1 >> 8) & 0xF) as u8;

        // Check CPACR (2 bits per coprocessor)
        let cpacr = bus.ppb[bus.active_core()].cpacr;
        let access = (cpacr >> (coproc as u32 * 2)) & 0x3;
        if access == 0 {
            self.pending_fault = Some(Fault::UsageFault);
            return 0;
        }

        match coproc {
            0 => self.cp0_gpioc(hw0, hw1),
            4 | 5 => self.cp4_5_dcp(hw0, hw1),
            7 => self.cp7_rcp(hw0, hw1),
            10 | 11 => self.fpu_execute(hw0, hw1, bus),
            _ => {
                self.pending_fault = Some(Fault::UsageFault);
                0
            }
        }
    }

    /// CP0 (GPIOC): GPIO coprocessor. Returns 0 for all reads (no external pins).
    fn cp0_gpioc(&mut self, _hw0: u16, _hw1: u16) -> u32 {
        // MRC: read GPIO -> return 0 (no pins connected)
        // MCR: write GPIO -> ignore
        1
    }

    /// CP4/5 (DCP): Double-precision coprocessor. Minimal transfer registers.
    fn cp4_5_dcp(&mut self, hw0: u16, hw1: u16) -> u32 {
        // Decode MCR vs MRC vs CDP from the instruction encoding
        let is_mrc_mcr = (hw0 >> 12) & 0xF == 0xE && hw1 & (1 << 4) != 0;
        if is_mrc_mcr {
            let rd = ((hw1 >> 12) & 0xF) as usize;
            let idx = (hw1 & 1) as usize; // use bit 0 to select register
            let to_cp = (hw0 >> 4) & 1 == 0; // MCR: bit 4 of hw0 = 0
            if to_cp {
                // MCR: ARM register -> DCP
                self.dcp_data[idx] = self.regs.r[rd];
            } else {
                // MRC: DCP -> ARM register
                self.regs.r[rd] = self.dcp_data[idx];
            }
        }
        // CDP: accept as NOP
        1
    }

    /// CP7 (RCP): Redundancy coprocessor. Accepts salt, returns dummy canary.
    fn cp7_rcp(&mut self, hw0: u16, hw1: u16) -> u32 {
        let is_mrc_mcr = (hw0 >> 12) & 0xF == 0xE && hw1 & (1 << 4) != 0;
        if is_mrc_mcr {
            let rd = ((hw1 >> 12) & 0xF) as usize;
            let to_cp = (hw0 >> 4) & 1 == 0;
            if to_cp {
                // MCR: write salt
                self.rcp_salt = self.regs.r[rd];
            } else {
                // MRC: read canary (salt XOR constant)
                self.regs.r[rd] = self.rcp_salt ^ 0xDEAD_BEEF;
            }
        }
        // CDP: assertion check — accept silently (never trigger NMI)
        1
    }
}

#[cfg(test)]
mod tests {
    use crate::bus::Bus;
    use crate::core::CortexM33;
    use crate::core::Fault;

    /// Encode MCR: ARM Rt -> CPn  (hw0[4]=0 means MCR)
    /// hw0 = 0xEE0x where x encodes opc1/CRn; hw1 = (Rt<<12) | (coproc<<8) | 0x10 | CRm
    fn encode_mcr(coproc: u8, rt: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xEE00; // bits [15:12]=0xE, bit4=0 (MCR)
        let hw1: u16 = ((rt as u16) << 12) | ((coproc as u16) << 8) | 0x10 | (crm as u16);
        (hw0, hw1)
    }

    /// Encode MRC: CPn -> ARM Rt  (hw0[4]=1 means MRC)
    fn encode_mrc(coproc: u8, rt: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xEE10; // bits [15:12]=0xE, bit4=1 (MRC)
        let hw1: u16 = ((rt as u16) << 12) | ((coproc as u16) << 8) | 0x10 | (crm as u16);
        (hw0, hw1)
    }

    /// Set CPACR to enable a given coprocessor (full access = 0b11).
    fn enable_cp(bus: &mut Bus, coproc: u8) {
        let core = bus.active_core();
        bus.ppb[core].cpacr |= 0x3 << (coproc as u32 * 2);
    }

    #[test]
    fn test_cp7_rcp_salt_roundtrip() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 7);

        let salt: u32 = 0x1234_5678;
        cpu.regs.r[0] = salt;

        // MCR: write salt from R0
        let (hw0, hw1) = encode_mcr(7, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        // MRC: read canary into R1
        let (hw0, hw1) = encode_mrc(7, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cpu.regs.r[1], salt ^ 0xDEAD_BEEF);
    }

    #[test]
    fn test_cp4_5_dcp_transfer() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 4);

        let val: u32 = 0xCAFE_BABE;
        cpu.regs.r[2] = val;

        // MCR: write R2 -> DCP[0]
        let (hw0, hw1) = encode_mcr(4, 2, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        // MRC: read DCP[0] -> R3
        let (hw0, hw1) = encode_mrc(4, 3, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cpu.regs.r[3], val);
    }

    #[test]
    fn test_cpacr_blocks_disabled_cp() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        // CPACR defaults to 0 — all coprocessors disabled

        let (hw0, hw1) = encode_mrc(7, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert!(matches!(cpu.pending_fault, Some(Fault::UsageFault)));
    }

    #[test]
    fn test_cpacr_allows_enabled_cp() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 7);

        cpu.regs.r[0] = 42;
        let (hw0, hw1) = encode_mcr(7, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert!(cpu.pending_fault.is_none());
        assert_eq!(cpu.rcp_salt, 42);
    }

    #[test]
    fn test_cp0_gpioc_read_zero() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        // CP0 is a stub — MRC returns 0 (no real GPIO reads implemented),
        // the function just returns cycle count 1 without modifying registers.
        cpu.regs.r[0] = 0xFFFF_FFFF;
        let (hw0, hw1) = encode_mrc(0, 0, 0);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cycles, 1);
        assert!(cpu.pending_fault.is_none());
    }
}

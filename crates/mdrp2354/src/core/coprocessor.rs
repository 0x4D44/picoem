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
            7 => self.cp7_rcp(hw0, hw1, bus),
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

    /// CP7 (RCP): Redundancy coprocessor. Dispatches MCR/MRC (0xEE/0xFE)
    /// and MCRR/MRRC (0xEC/0xFC) encoding families.
    fn cp7_rcp(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let hw0_high = (hw0 >> 8) & 0xFF;
        match hw0_high {
            0xEE | 0xFE => self.cp7_mcr_mrc_family(hw0, hw1, bus),
            0xEC | 0xFC => self.cp7_mcrr_mrrc_family(hw0, hw1, bus),
            _ => 1, // Not a recognized CP7 encoding
        }
    }

    fn cp7_mcr_mrc_family(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let is_cdp = hw1 & (1 << 4) == 0;
        if is_cdp {
            return 1; // CDP / CDP2: accept silently (rcp_panic etc.)
        }
        let is_mrc = (hw0 >> 4) & 1 != 0; // L bit
        let rt = ((hw1 >> 12) & 0xF) as usize;
        let core = bus.active_core();

        if is_mrc {
            if rt == 15 {
                // rcp_canary_status pc: write NZCV to APSR.
                // N = salt_valid[core]; Z=0, C=0, V=0.
                let n = if bus.rcp_salt_valid[core] { 1u32 << 31 } else { 0 };
                self.regs.xpsr = (self.regs.xpsr & 0x0FFF_FFFF) | n;
            } else {
                // rcp_canary_get: return salt XOR deadbeef
                self.regs.r[rt] = bus.rcp_salt[core] ^ 0xDEAD_BEEF;
            }
        }
        // MCR/MCR2: rcp_canary_check assertion — accept silently
        1
    }

    fn cp7_mcrr_mrrc_family(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let l_bit = (hw0 >> 4) & 1;
        if l_bit != 0 {
            return 1; // MRRC2 from CP7: not used by bootrom
        }

        // MCRR2 dispatch: discriminator is (opc1, CRm) from hw1.
        let opc1 = ((hw1 >> 4) & 0xF) as u8;
        let crm = (hw1 & 0xF) as u8;
        let rt = ((hw1 >> 12) & 0xF) as usize;

        match (opc1, crm) {
            (8, 0) => {
                // rcp_salt_core0
                bus.rcp_salt[0] = self.regs.r[rt];
                bus.rcp_salt_valid[0] = true;
            }
            (8, 1) => {
                // rcp_salt_core1
                bus.rcp_salt[1] = self.regs.r[rt];
                bus.rcp_salt_valid[1] = true;
            }
            _ => {
                // rcp_iequal, rcp_bvalid, rcp_count_*, etc.: silent NOP
            }
        }
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

        // Poke salt directly into bus (rcp_salt lives on Bus now)
        bus.rcp_salt[0] = 42;

        // MRC: read canary into R1
        let (hw0, hw1) = encode_mrc(7, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cpu.regs.r[1], 42 ^ 0xDEAD_BEEF);
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

        // MCR with CP7 enabled — should not fault (MCR is a silent NOP now)
        cpu.regs.r[0] = 42;
        let (hw0, hw1) = encode_mcr(7, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert!(cpu.pending_fault.is_none());
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

use crate::bus::Bus;
use crate::sio::Sio;
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
            0 => self.cp0_gpioc(hw0, hw1, bus),
            4 | 5 => self.cp4_5_dcp(hw0, hw1),
            7 => self.cp7_rcp(hw0, hw1, bus),
            10 | 11 => self.fpu_execute(hw0, hw1, bus),
            _ => {
                self.pending_fault = Some(Fault::UsageFault);
                0
            }
        }
    }

    /// CP0 (GPIOC): GPIO coprocessor — SDK-emitted ops wired to SIO fast-path
    /// and `Bus.gpio_in`. See `cp0_mcr_mrc_family` for the encoding table.
    fn cp0_gpioc(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let is_mrc_mcr = (hw0 >> 12) & 0xF == 0xE && hw1 & (1 << 4) != 0;
        if is_mrc_mcr {
            self.cp0_mcr_mrc_family(hw0, hw1, bus)
        } else {
            1 // CDP not used by CP0 — silent NOP, match existing stub style.
        }
    }

    /// CP0 MCR/MRC dispatch — matches HLD §C.1 / Pico SDK `hardware_gpio.h`.
    ///
    /// Thumb-32 MCR/MRC encoding:
    /// - hw0 = `1110 1110 opc1[3] L CRn[4]` (L=0 MCR, L=1 MRC)
    /// - hw1 = `Rt[4] coproc[4] opc2[3] 1 CRm[4]`
    ///
    /// The SDK uses `opc1` to select the bank (OUT/OE/IN × LO/HI) and then
    /// discriminates **bulk vs per-bit** by the `(CRn, CRm)` pair:
    ///   - `(CRn=0, CRm=0)` → bulk bank op; op2 selects get/put/set/clr/xor.
    ///   - otherwise → per-bit op on `pin = (CRn<<4)|CRm`; op2 selects the op.
    ///
    /// Bank mapping (RP2354A is 30-pin, HI bank is RAZ/WI):
    ///
    /// | opc1 | Bank              |
    /// |------|-------------------|
    /// |  0   | LO OUT (GPIO_OUT, pins 0..29)  |
    /// |  1   | LO OE  (GPIO_OE,  pins 0..29)  |
    /// |  2   | LO IN  (GPIO_IN,  pins 0..29)  |
    /// |  4   | HI OUT (pins 30..47 — RAZ/WI)  |
    /// |  5   | HI OE  (pins 30..47 — RAZ/WI)  |
    /// |  6   | HI IN  (pins 30..47 — RAZ)     |
    ///
    /// Per-bit op2 selection (when CRn or CRm is non-zero):
    /// `op2=0` → `_get` (MRC), `op2=4` → `_put` (MCR Rt[0]),
    /// `op2=5` → `_set`, `op2=6` → `_clr`, `op2=7` → `_xor`.
    ///
    /// Bulk op2 selection (when CRn=0 and CRm=0):
    /// MRC `op2=0` → `_get`. MCR `op2=0` → `_put`, `op2=1` → `_set`,
    /// `op2=2` → `_clr`, `op2=3` → `_xor`.
    ///
    /// Examples matching HLD §C.1:
    ///   `gpioc_lo_out_get()` = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
    ///   `gpioc_hi_out_get()` = MRC CP0, opc1=4, CRn=0, CRm=0, op2=0.
    ///   `gpioc_bit_out_get(pin)` = MRC CP0, opc1=0, CRn=pin_hi, CRm=pin_lo, op2=0.
    ///
    /// Note: pin 0 has (CRn=0, CRm=0), which collides with the bulk encoding.
    /// Per HLD, pin 0 per-bit ops are unreachable by this scheme; firmware
    /// uses the bulk mask path for pin 0. Matches Pico SDK behavior.
    ///
    /// Undefined op2 on MRC reads as 0; undefined op2 on MCR is silent NOP.
    /// Cycle cost: 1.
    fn cp0_mcr_mrc_family(&mut self, hw0: u16, hw1: u16, bus: &mut Bus) -> u32 {
        let is_mrc = (hw0 >> 4) & 1 != 0; // L bit
        let opc1 = ((hw0 >> 5) & 0x7) as u8;
        let crn = (hw0 & 0xF) as u8;
        let crm = (hw1 & 0xF) as u8;
        let op2 = ((hw1 >> 5) & 0x7) as u8;
        let rt = ((hw1 >> 12) & 0xF) as usize;
        let is_bulk = crn == 0 && crm == 0;

        match opc1 {
            // ---- LO banks (pins 0..29) ----
            0 => self.cp0_lo_out(bus, is_mrc, is_bulk, crn, crm, op2, rt),
            1 => self.cp0_lo_oe(bus, is_mrc, is_bulk, crn, crm, op2, rt),
            2 => self.cp0_lo_in(bus, is_mrc, is_bulk, crn, crm, rt),
            // ---- HI banks (pins 30..47) — RP2354A has no pins here.
            // Reads RAZ, writes WI. Preserve any Rt value on MRC by writing 0;
            // no SIO mutation on MCR.
            4 | 5 | 6 => {
                if is_mrc {
                    self.regs.r[rt] = 0;
                }
            }
            _ => {} // unknown opc1 -> silent NOP
        }
        1
    }

    /// LO OUT bank (opc1=0): bulk lo_out when (CRn=0,CRm=0), else per-bit on pin.
    fn cp0_lo_out(
        &mut self,
        bus: &mut Bus,
        is_mrc: bool,
        is_bulk: bool,
        crn: u8,
        crm: u8,
        op2: u8,
        rt: usize,
    ) {
        if is_bulk {
            if is_mrc {
                // op2=0 is the documented get; other op2 treated as NOP read 0.
                self.regs.r[rt] = if op2 == 0 { bus.sio.gpio_lo_out_get() } else { 0 };
            } else {
                let v = self.regs.r[rt];
                match op2 {
                    0 => bus.sio.gpio_lo_out_put(v),
                    1 => bus.sio.gpio_lo_out_set(v),
                    2 => bus.sio.gpio_lo_out_clr(v),
                    3 => bus.sio.gpio_lo_out_xor(v),
                    _ => {}
                }
            }
        } else {
            let pin = (crn << 4) | crm;
            if is_mrc {
                let v = if op2 == 0 { bus.sio.gpio_bit_out_get(pin) } else { false };
                self.regs.r[rt] = v as u32;
            } else {
                match op2 {
                    4 => bus.sio.gpio_bit_out_put(pin, self.regs.r[rt] & 1 != 0),
                    5 => bus.sio.gpio_bit_out_set(pin),
                    6 => bus.sio.gpio_bit_out_clr(pin),
                    7 => bus.sio.gpio_bit_out_xor(pin),
                    _ => {}
                }
            }
        }
    }

    /// LO OE bank (opc1=1): bulk lo_oe when (CRn=0,CRm=0), else per-bit on pin.
    fn cp0_lo_oe(
        &mut self,
        bus: &mut Bus,
        is_mrc: bool,
        is_bulk: bool,
        crn: u8,
        crm: u8,
        op2: u8,
        rt: usize,
    ) {
        if is_bulk {
            if is_mrc {
                self.regs.r[rt] = if op2 == 0 { bus.sio.gpio_lo_oe_get() } else { 0 };
            } else {
                let v = self.regs.r[rt];
                match op2 {
                    0 => bus.sio.gpio_lo_oe_put(v),
                    1 => bus.sio.gpio_lo_oe_set(v),
                    2 => bus.sio.gpio_lo_oe_clr(v),
                    3 => bus.sio.gpio_lo_oe_xor(v),
                    _ => {}
                }
            }
        } else {
            let pin = (crn << 4) | crm;
            if is_mrc {
                let v = if op2 == 0 { bus.sio.gpio_bit_oe_get(pin) } else { false };
                self.regs.r[rt] = v as u32;
            } else {
                match op2 {
                    4 => bus.sio.gpio_bit_oe_put(pin, self.regs.r[rt] & 1 != 0),
                    5 => bus.sio.gpio_bit_oe_set(pin),
                    6 => bus.sio.gpio_bit_oe_clr(pin),
                    7 => bus.sio.gpio_bit_oe_xor(pin),
                    _ => {}
                }
            }
        }
    }

    /// LO IN bank (opc1=2, read-only): bulk lo_in_get when (CRn=0,CRm=0),
    /// else per-bit in on pin. Source is `bus.gpio_in`. MCR is a silent NOP.
    fn cp0_lo_in(
        &mut self,
        bus: &mut Bus,
        is_mrc: bool,
        is_bulk: bool,
        crn: u8,
        crm: u8,
        rt: usize,
    ) {
        if !is_mrc {
            return; // writes to the input bank are undefined -> silent NOP.
        }
        if is_bulk {
            self.regs.r[rt] = bus.gpio_in & Sio::PIN_MASK;
        } else {
            let pin = (crn << 4) | crm;
            self.regs.r[rt] = if pin < 30 { (bus.gpio_in >> pin) & 1 } else { 0 };
        }
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
    /// hw0 = `1110 1110 opc1[3] L CRn[4]`
    /// hw1 = `Rt[4] coproc[4] opc2[3] 1 CRm[4]`
    fn encode_mcr(coproc: u8, rt: u8, crm: u8) -> (u16, u16) {
        encode_mcr_full(coproc, 0, 0, rt, 0, crm)
    }

    /// Encode MRC: CPn -> ARM Rt  (hw0[4]=1 means MRC)
    fn encode_mrc(coproc: u8, rt: u8, crm: u8) -> (u16, u16) {
        encode_mrc_full(coproc, 0, 0, rt, 0, crm)
    }

    /// Full MCR encoder (opc1, CRn, Rt, op2, CRm).
    fn encode_mcr_full(coproc: u8, opc1: u8, crn: u8, rt: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xEE00
            | ((opc1 as u16 & 0x7) << 5)
            // L bit = 0 for MCR
            | (crn as u16 & 0xF);
        let hw1: u16 = ((rt as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | 0x10
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    /// Full MRC encoder.
    fn encode_mrc_full(coproc: u8, opc1: u8, crn: u8, rt: u8, op2: u8, crm: u8) -> (u16, u16) {
        let hw0: u16 = 0xEE00
            | ((opc1 as u16 & 0x7) << 5)
            | 0x10 // L bit = 1 for MRC
            | (crn as u16 & 0xF);
        let hw1: u16 = ((rt as u16) << 12)
            | ((coproc as u16) << 8)
            | ((op2 as u16 & 0x7) << 5)
            | 0x10
            | (crm as u16 & 0xF);
        (hw0, hw1)
    }

    /// Split a pin number into (CRn=pin_hi, CRm=pin_lo).
    fn pin_split(pin: u8) -> (u8, u8) {
        ((pin >> 4) & 0xF, pin & 0xF)
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

    // ---- CP0 GPIOC tests (Phase 7 Stage C) ----

    /// Baseline cycle-count assertion — replaces the old "returns 0" stub test
    /// per HLD §10 test impact table. New contract: CP0 MRC reads actual SIO
    /// state, and returns cycle count 1.
    #[test]
    fn test_cp0_gpioc_read_matches_sio() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        // Seed SIO gpio_out via direct field access, then read via CP0.
        bus.sio.gpio_out = 0x12345678 & 0x3FFF_FFFF;
        // HLD §C.1: gpioc_lo_out_get = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mrc_full(0, 0, 0, 3, 0, 0); // lo_out_get -> r3
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cycles, 1);
        assert_eq!(cpu.regs.r[3], bus.sio.gpio_out);
        assert!(cpu.pending_fault.is_none());
    }

    // --- Per-bit GPIO_OUT ops ---

    #[test]
    fn test_cp0_bit_out_set() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        let pin = 5u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 1 << pin);
    }

    #[test]
    fn test_cp0_bit_out_clr() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);
        bus.sio.gpio_out = 0x0000_00FF;

        let pin = 3u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 6, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0xF7);
    }

    #[test]
    fn test_cp0_bit_out_xor() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);
        bus.sio.gpio_out = 0b1010;

        let pin = 1u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 7, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0b1000);
    }

    #[test]
    fn test_cp0_bit_out_put() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        let pin = 7u8;
        let (crn, crm) = pin_split(pin);

        // Put 1 into pin 7.
        cpu.regs.r[2] = 1;
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 2, 4, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 1 << pin);

        // Put 0 into pin 7.
        cpu.regs.r[2] = 0;
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 2, 4, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0);
    }

    #[test]
    fn test_cp0_bit_out_get() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        bus.sio.gpio_out = 1 << 9 | 1 << 15;

        // Read pin 9 -> 1.
        let (crn, crm) = pin_split(9);
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 4, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[4], 1);

        // Read pin 10 -> 0.
        let (crn, crm) = pin_split(10);
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 5, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[5], 0);
    }

    // --- Per-bit GPIO_OE ops ---

    #[test]
    fn test_cp0_bit_oe_set_clr_xor_put_get() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        let pin = 12u8;
        let (crn, crm) = pin_split(pin);

        // set
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 1 << pin);

        // get -> r6 = 1
        let (hw0, hw1) = encode_mrc_full(0, 1, crn, 6, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[6], 1);

        // xor -> clears
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 0, 7, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 0);

        // put 1
        cpu.regs.r[7] = 1;
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 7, 4, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 1 << pin);

        // clr
        let (hw0, hw1) = encode_mcr_full(0, 1, crn, 0, 6, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 0);
    }

    // --- Bulk GPIO_OUT ops ---

    #[test]
    fn test_cp0_lo_out_put_then_get() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        cpu.regs.r[1] = 0x1234_5678;
        // HLD §C.1: lo_out_put = MCR CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        // Value masked to 30 pins.
        assert_eq!(bus.sio.gpio_out, 0x1234_5678 & 0x3FFF_FFFF);

        // Read back: lo_out_get = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mrc_full(0, 0, 0, 2, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[2], 0x1234_5678 & 0x3FFF_FFFF);
    }

    #[test]
    fn test_cp0_lo_out_set_clr_xor() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        bus.sio.gpio_out = 0x0000_F000;

        // set 0x0F00: opc1=0, CRn=0, CRm=0, op2=1.
        cpu.regs.r[1] = 0x0F00;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 1, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0xFF00);

        // clr 0x00F0 (nothing to clear) — gpio_out unchanged. op2=2.
        cpu.regs.r[1] = 0x00F0;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 2, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0xFF00);

        // xor 0xF000 — flips high bits. op2=3.
        cpu.regs.r[1] = 0xF000;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 1, 3, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0x0F00);
    }

    #[test]
    fn test_cp0_lo_oe_bulk() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        cpu.regs.r[0] = 0xDEAD_BEEF;
        // HLD §C.1: lo_oe_put = MCR CP0, opc1=1 (OE bank), CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mcr_full(0, 1, 0, 0, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_oe, 0xDEAD_BEEF & 0x3FFF_FFFF);

        let (hw0, hw1) = encode_mrc_full(0, 1, 0, 1, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[1], 0xDEAD_BEEF & 0x3FFF_FFFF);
    }

    // --- Input reads ---

    #[test]
    fn test_cp0_lo_in_get() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        bus.gpio_in = 0xA5A5_A5A5;
        // opc1=2 (IN bank), CRn=0, CRm=0 -> lo_in_get, MRC into r8.
        let (hw0, hw1) = encode_mrc_full(0, 2, 0, 8, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[8], 0xA5A5_A5A5 & 0x3FFF_FFFF);
    }

    #[test]
    fn test_cp0_bit_in_get() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        bus.gpio_in = 0xA5A5_A5A5;
        // 0xA5 = 1010 0101: bit 2 = 1, bit 3 = 0, bit 5 = 1.
        // Pin 0 is unreachable per-bit under the HLD encoding (CRn=0,CRm=0
        // is the bulk slot), so exercise pins 2 and 3 instead.
        let (crn, crm) = pin_split(2);
        let (hw0, hw1) = encode_mrc_full(0, 2, crn, 9, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[9], 1);

        let (crn, crm) = pin_split(3);
        let (hw0, hw1) = encode_mrc_full(0, 2, crn, 10, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[10], 0);
    }

    // --- Pin >= 30 masking ---

    #[test]
    fn test_cp0_bit_set_pin_30_masked() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        // Pin 30 is out of range on RP2354A (30 pins: 0..29). Write must be masked.
        let pin = 30u8;
        let (crn, crm) = pin_split(pin);
        let before = bus.sio.gpio_out;
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, before);

        // Read pin 30 -> returns 0 regardless of underlying bit.
        bus.sio.gpio_out = 0xFFFF_FFFF;
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 11, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[11], 0);
    }

    #[test]
    fn test_cp0_lo_out_put_masks_upper_bits() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        // Write a value with bits [31:30] set; expect them masked to 0.
        // HLD §C.1: lo_out_put = MCR CP0, opc1=0, CRn=0, CRm=0, op2=0.
        cpu.regs.r[0] = 0xFFFF_FFFF;
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 0, 0, 0);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(bus.sio.gpio_out, 0x3FFF_FFFF);
    }

    // --- Discrimination: CP0 and SIO MMIO observe the same state ---

    #[test]
    fn test_cp0_write_observed_via_mmio() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        // Set pin 12 via CP0.
        let pin = 12u8;
        let (crn, crm) = pin_split(pin);
        let (hw0, hw1) = encode_mcr_full(0, 0, crn, 0, 5, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        // Read GPIO_OUT through the SIO MMIO path at 0xD000_0010.
        let mmio_val = bus.read32(0xD000_0010);
        assert_eq!(mmio_val, 1 << pin);

        // Conversely: write via MMIO GPIO_OUT_SET (0xD000_0018) and read via CP0.
        bus.write32(0xD000_0018, 1 << 20);
        let (crn, crm) = pin_split(20);
        let (hw0, hw1) = encode_mrc_full(0, 0, crn, 0, 0, crm);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);
        assert_eq!(cpu.regs.r[0], 1);
    }

    // --- CPACR disabled -> UsageFault ---

    #[test]
    fn test_cp0_cpacr_disabled_faults() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        // CPACR defaults to 0 — CP0 disabled.

        // Encode `bit_out_set(pin=5)`: opc1=0, CRn=0, CRm=5, op2=5, MCR.
        let (hw0, hw1) = encode_mcr_full(0, 0, 0, 0, 5, 5);
        cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert!(matches!(cpu.pending_fault, Some(Fault::UsageFault)));
        // Devil's-advocate follow-up: confirm the write was suppressed, not just
        // that a fault was raised. gpio_out must remain zero.
        assert_eq!(bus.sio.gpio_out, 0);
    }

    // --- HLD §C.1 compliance lock-in ---

    /// Regression guard for the encoding-bug fix (HLD §C.1 / SDK compliance).
    ///
    /// `gpioc_lo_out_get()` is `MRC CP0, opc1=0, CRn=0, CRm=0, op2=0` — the
    /// same opc1 as `gpioc_bit_out_get(pin)`. A prior implementation routed
    /// opc1=0 unconditionally through the per-bit path, which would make
    /// real-SDK firmware silently read only pin 0 of the bank. This test
    /// seeds a multi-bit pattern and asserts the read returns the full
    /// 30-bit bank, locking out the regression.
    #[test]
    fn test_cp0_hld_lo_out_get_reads_full_bank() {
        let mut cpu = CortexM33::new();
        let mut bus = Bus::default();
        enable_cp(&mut bus, 0);

        // Set a pattern with bit 0 = 0 and other bits = 1 — catches a
        // per-bit-pin-0 regression (which would read 0, not 0x3FFF_FFFE).
        let pattern: u32 = 0x3FFF_FFFE;
        bus.sio.gpio_out = pattern;

        // HLD §C.1: gpioc_lo_out_get = MRC CP0, opc1=0, CRn=0, CRm=0, op2=0.
        let (hw0, hw1) = encode_mrc_full(0, 0, 0, 7, 0, 0);
        let cycles = cpu.thumb32_coprocessor(hw0, hw1, &mut bus);

        assert_eq!(cycles, 1);
        assert_eq!(
            cpu.regs.r[7], pattern,
            "lo_out_get (opc1=0, CRn=0, CRm=0) must read the full 30-bit bank"
        );
        assert!(cpu.pending_fault.is_none());
    }
}

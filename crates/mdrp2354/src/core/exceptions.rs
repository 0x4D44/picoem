use crate::bus::Bus;
use super::{CortexM33, Fault};

impl CortexM33 {
    // --- EXC_RETURN detection ---

    /// Returns true if val is an Armv8-M EXC_RETURN magic value.
    pub(crate) fn is_exc_return(val: u32) -> bool {
        val & 0xFF00_0000 == 0xFF00_0000
    }

    // --- IT state encode/decode for exception stacking ---

    pub(crate) fn encode_it_to_xpsr(&self) -> u32 {
        let it = self.it_state as u32;
        ((it & 0xC0) << 19) | ((it & 0x3F) << 10) // bits [26:25] | [15:10]
    }

    pub(crate) fn decode_it_from_xpsr(xpsr: u32) -> u8 {
        (((xpsr >> 19) & 0xC0) | ((xpsr >> 10) & 0x3F)) as u8
    }

    // --- Return address selection ---

    /// Return address to stack during exception entry.
    /// Faults: return to faulting instruction (current_instr_addr) for retry.
    /// Calls (SVC) and async: return to next instruction (current PC).
    fn return_address(&self, exc_num: u16) -> u32 {
        match exc_num {
            // Synchronous faults (incl. escalated HardFault): retry the faulting instruction
            3 | 4 | 5 | 6 | 7 => self.current_instr_addr,
            // SVC (11), PendSV (14), SysTick (15), external IRQs (16+): next instruction
            _ => self.regs.pc(),
        }
    }

    // --- Exception entry ---

    /// Push exception frame, fetch vector, enter handler mode.
    /// Returns cycle cost (~12).
    pub(crate) fn enter_exception(&mut self, exc_num: u16, bus: &mut Bus) -> u32 {
        let use_psp = !self.regs.in_handler_mode() && self.regs.active_sp_is_psp();
        let original_sp = if use_psp { self.regs.psp } else { self.regs.msp };

        // 8-byte align with padding tracking (CCR.STKALIGN is RAO on M33)
        let aligned_sp = original_sp & !0x7;
        let frame_sp = aligned_sp.wrapping_sub(32);
        let was_padded = aligned_sp != original_sp;

        // Encode IT state and alignment padding into stacked xPSR.
        // Mask IT bits [26:25,15:10] from base xPSR first to avoid OR corruption
        // from stale bits left by a previous exit_exception.
        const IT_MASK: u32 = 0x0600_FC00;
        let mut stacked_xpsr = (self.regs.xpsr & !IT_MASK) | self.encode_it_to_xpsr();
        if was_padded {
            stacked_xpsr |= 1 << 9;
        }

        // Push exception frame: R0, R1, R2, R3, R12, LR, ReturnAddress, xPSR
        bus.write32(frame_sp, self.regs.r[0]);
        bus.write32(frame_sp.wrapping_add(4), self.regs.r[1]);
        bus.write32(frame_sp.wrapping_add(8), self.regs.r[2]);
        bus.write32(frame_sp.wrapping_add(12), self.regs.r[3]);
        bus.write32(frame_sp.wrapping_add(16), self.regs.r[12]);
        bus.write32(frame_sp.wrapping_add(20), self.regs.lr());
        bus.write32(frame_sp.wrapping_add(24), self.return_address(exc_num));
        bus.write32(frame_sp.wrapping_add(28), stacked_xpsr);

        // Update SP
        if use_psp {
            self.regs.psp = frame_sp;
        } else {
            self.regs.msp = frame_sp;
        }

        // Set LR to EXC_RETURN (Armv8-M, non-secure, no FP frame)
        self.regs.r[14] = if self.regs.in_handler_mode() {
            0xFFFF_FFF1 // return to Handler, MSP
        } else if use_psp {
            0xFFFF_FFFD // return to Thread, PSP
        } else {
            0xFFFF_FFF9 // return to Thread, MSP
        };

        // Fetch vector from table
        let vtor = bus.ppb[bus.active_core()].vtor;
        let vector = bus.read32(vtor.wrapping_add((exc_num as u32) * 4));
        self.regs.set_pc(vector & !1);

        // Enter handler mode: set IPSR, force MSP, clear IT
        self.regs.xpsr = (self.regs.xpsr & !0x1FF) | (exc_num as u32);
        self.regs.control &= !2; // handler always MSP
        self.regs.sync_sp_from_banked();
        self.it_state = 0;

        12
    }

    // --- Exception return ---

    /// Pop exception frame, restore mode. Returns cycle cost (~12).
    pub(crate) fn exit_exception(&mut self, exc_return: u32, bus: &mut Bus) -> u32 {
        let active_exc = self.regs.ipsr(); // capture BEFORE popping

        let return_to_psp = exc_return & 0x4 != 0;
        let sp = if return_to_psp { self.regs.psp } else { self.regs.msp };

        // Pop frame
        self.regs.r[0] = bus.read32(sp);
        self.regs.r[1] = bus.read32(sp.wrapping_add(4));
        self.regs.r[2] = bus.read32(sp.wrapping_add(8));
        self.regs.r[3] = bus.read32(sp.wrapping_add(12));
        self.regs.r[12] = bus.read32(sp.wrapping_add(16));
        self.regs.r[14] = bus.read32(sp.wrapping_add(20));
        let return_pc = bus.read32(sp.wrapping_add(24));
        let return_xpsr = bus.read32(sp.wrapping_add(28));

        self.regs.set_pc(return_pc & !1);

        // Alignment padding check (bit 9 of stacked xPSR)
        let frame_size: u32 = if return_xpsr & (1 << 9) != 0 { 36 } else { 32 };

        // Restore xPSR: clear bit 9 (frame metadata) and IT bits [26:25,15:10]
        // (IT state lives in the separate it_state field, not in xPSR)
        const IT_MASK: u32 = 0x0600_FC00;
        self.regs.xpsr = return_xpsr & !(1 << 9) & !IT_MASK;
        self.it_state = Self::decode_it_from_xpsr(return_xpsr);

        // Deallocate frame
        if return_to_psp {
            self.regs.psp = sp.wrapping_add(frame_size);
        } else {
            self.regs.msp = sp.wrapping_add(frame_size);
        }

        // Restore SPSEL
        self.regs.control = (self.regs.control & !2) | if return_to_psp { 2 } else { 0 };
        self.regs.sync_sp_from_banked();

        // Clear active exception
        bus.ppb[bus.active_core()].clear_active(active_exc as u16);

        12
    }

    // --- Priority evaluation ---

    /// Effective execution priority (lower = higher priority).
    pub(crate) fn execution_priority(&self, bus: &Bus) -> i16 {
        let mut prio: i16 = 256;
        if self.regs.faultmask & 1 != 0 {
            prio = -1;
        } else if self.regs.primask & 1 != 0 {
            prio = 0;
        }

        let ipsr = self.regs.ipsr();
        if ipsr > 0 {
            let exc_prio = bus.ppb[bus.active_core()].exception_priority(ipsr as u16);
            if exc_prio < prio {
                prio = exc_prio;
            }
        }
        prio
    }

    pub(crate) fn can_preempt(&self, exc_num: u16, bus: &Bus) -> bool {
        let exc_prio = bus.ppb[bus.active_core()].exception_priority(exc_num);
        exc_prio < self.execution_priority(bus)
    }

    // --- Fault delivery ---

    /// Deliver a pending fault. Returns cycle cost.
    pub(crate) fn deliver_fault(&mut self, fault: Fault, bus: &mut Bus) -> u32 {
        let core = bus.active_core();
        match fault {
            Fault::UsageFault => {
                // Set UFSR.UNDEFINSTR (bit 16 of CFSR)
                bus.ppb[core].cfsr |= 1 << 16;
                if bus.ppb[core].shcsr & (1 << 18) != 0 {
                    // USGFAULTENA
                    self.enter_exception(6, bus)
                } else {
                    bus.ppb[bus.active_core()].hfsr |= 1 << 30; // FORCED
                    self.enter_exception(3, bus) // escalate to HardFault
                }
            }
        }
    }

    // --- TT (Test Target) instruction -----------------------------------------

    /// Execute a TT instruction: look up SAU/IDAU region attributes for an address.
    /// Returns the TT result register value per ARMv8-M Architecture Reference.
    ///
    /// Result bits (per ARM DDI 0553):
    ///   [7:0]   MREGION — MPU region number (valid when MRVALID=1)
    ///   [15:8]  SREGION — SAU region number (valid when SRVALID=1)
    ///   [16]    MRVALID — MPU region match
    ///   [17]    SRVALID — SAU region match
    ///   [18]    R  — readable from current security state
    ///   [19]    RW — read-write from current security state
    ///   [20]    NSR  — NS readable
    ///   [21]    NSRW — NS read-write
    ///   [22]    S  — Secure
    ///   [23]    IRVALID — IDAU region valid
    ///   [25]    RP2350 IDAU exempt flag
    pub(crate) fn execute_tt(addr: u32, bus: &Bus) -> u32 {
        let ppb = &bus.ppb[bus.active_core()];

        // RP2350 IDAU: built-in security attribution for the address space.
        // Secure regions: ROM (0x0000_0000..0x0000_7FFF),
        //   secure SRAM/peripheral aliases, etc.
        let idau_result = Self::rp2350_idau(addr);

        // If SAU is disabled, everything is Secure with full access
        if ppb.sau_ctrl & 1 == 0 {
            // S=1, RW=1, R=1, no SAU region match; include IDAU bits
            return idau_result
                 | (1 << 22) | (1 << 19) | (1 << 18);
        }

        // Look up address in SAU regions
        for i in 0..8 {
            let (rbar, rlar) = ppb.sau_regions[i];
            let enabled = rlar & 1 != 0;
            if !enabled {
                continue;
            }

            let base = rbar & !0x1F; // bits [31:5]
            let limit = rlar | 0x1F; // bits [31:5] filled to 32-byte boundary
            let nsc = (rlar >> 1) & 1; // Non-Secure Callable

            if addr >= base && addr <= limit {
                let secure = nsc == 0;
                let region_num = i as u32;

                let mut result = idau_result;
                result |= (region_num & 0xFF) << 8;              // SREGION [15:8]
                result |= 1 << 17;                                // SRVALID
                result |= 1 << 18;                                // R (readable)
                result |= 1 << 19;                                // RW (read-write)
                if secure {
                    result |= 1 << 22;                            // S
                } else {
                    // Non-secure region: NS code can also access
                    result |= (1 << 20) | (1 << 21);             // NSR, NSRW
                }
                return result;
            }
        }

        // Address not in any SAU region
        let allns = (ppb.sau_ctrl >> 1) & 1;
        if allns != 0 {
            // ALLNS=1: unmatched addresses are Non-Secure
            idau_result | (1 << 20) | (1 << 21) | (1 << 19) | (1 << 18)
        } else {
            // ALLNS=0: unmatched addresses are Secure
            idau_result | (1 << 22) | (1 << 19) | (1 << 18)
        }
    }

    /// RP2350 Implementation-Defined Attribution Unit (IDAU).
    /// Returns the IDAU contribution to TT result bits.
    /// The RP2350 IDAU marks certain address ranges as secure/non-secure.
    fn rp2350_idau(addr: u32) -> u32 {
        // RP2350 address map (from datasheet):
        //   0x0000_0000..0x0000_7FFF: Secure ROM
        //   0x0000_8000..0x0000_FFFF: ROM (NS alias)
        //   0x1000_0000..0x1FFF_FFFF: XIP (secure)
        //   0x2000_0000..0x2007_FFFF: SRAM (secure)
        //   0x4000_0000..0x4FFF_FFFF: Peripherals (secure)
        //   0xD000_0000..0xD000_0FFF: SIO (secure)
        //   0xE000_0000..0xE00F_FFFF: PPB (secure, always)
        //
        // The IDAU on RP2350 provides a region number and secure/exempt flags.
        // For addresses the IDAU recognizes, it sets IRVALID (bit 23) and
        // the RP2350-specific exempt bit (bit 25).
        let idau_secure = match addr >> 28 {
            0x0 => addr < 0x0000_8000, // ROM: lower 32K is secure
            0x1 => true,                // XIP: secure
            0x2 => true,                // SRAM: secure
            0x3 => true,                // SRAM alias
            0x4 => true,                // APB peripherals: secure
            0x5 => true,                // AHB peripherals: secure
            0xD => true,                // SIO: secure
            0xE => true,                // PPB: always secure
            _ => false,
        };

        // IRVALID = 1, RP2350 exempt bit 25 = 1 for recognized secure regions
        if idau_secure {
            (1 << 23) | (1 << 25)
        } else {
            0
        }
    }
}

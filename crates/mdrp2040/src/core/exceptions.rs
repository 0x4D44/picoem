//! ARMv6-M exception model — Cortex-M0+ variant.
//!
//! Delta from mdrp2350 (ARMv8-M / M33):
//!
//! * Single HardFault vector — no separate MemManage / BusFault /
//!   UsageFault. All synchronous exceptions escalate to HardFault
//!   (exception #3) except SVC (exception #11) and NMI (exception #2).
//! * No FP frame — the stack frame is exactly 8 words.
//! * No security state (no EXC_RETURN bit [6]).
//! * No IT state — `xPSR` bits [26:25] and [15:10] are RAZ.
//! * No stack-limit registers (M/PSPLIM) — no stacking overflow check.
//! * EXC_RETURN magic values live in the top four bits (`0xFFFF_FFFx`):
//!     - `0xF1` — return to Handler mode, MSP (nested exception).
//!     - `0xF9` — return to Thread mode, MSP.
//!     - `0xFD` — return to Thread mode, PSP.
//!   Any other low-nibble → HardFault (InvalidExcReturn).
//!
//! Cycle counts: entry ≈ 16, exit ≈ 12 (M0+ TRM typical).

use crate::bus::Bus;
use super::{CortexM0Plus, Fault};

impl CortexM0Plus {
    /// Returns `true` if `val` matches the ARMv6-M EXC_RETURN magic
    /// pattern (top four bits all set).
    #[inline]
    pub(crate) fn is_exc_return(val: u32) -> bool {
        val & 0xF000_0000 == 0xF000_0000 && val & 0x0FFF_FFF0 == 0x0FFF_FFF0
    }

    /// Return address to push on exception entry. Synchronous faults
    /// return to the faulting instruction so the handler can retry;
    /// SVC / PendSV / async interrupts return to the next instruction.
    #[inline]
    fn return_address(&self, exc_num: u16) -> u32 {
        match exc_num {
            3 => self.current_instr_addr,     // HardFault — retry faulting instr
            11 => self.regs.pc(),             // SVC — return after the SVC
            _ => self.regs.pc(),              // async / NMI / PendSV / SysTick
        }
    }

    // -------------------------------------------------------------------
    // Exception entry
    // -------------------------------------------------------------------

    /// Push the 8-word frame, fetch the vector, enter Handler mode.
    ///
    /// On M0+ the stack frame is fixed at 8 words (no FP); we still
    /// apply the 8-byte-alignment pad per ARMv6-M ARM §B1.5.6.
    ///
    /// Cycle cost: ~16 (M0+ TRM).
    pub(crate) fn enter_exception(&mut self, exc_num: u16, bus: &mut Bus) -> u32 {
        // HardFault-in-HardFault → lockup (architecturally undefined on
        // v6-M but the common behaviour is to halt the core).
        if exc_num == 3 && self.regs.ipsr() == 3 {
            self.halted = true;
            return 0;
        }

        // Plain instructions (PUSH, POP, SUB SP #imm, ADD SP #imm) update
        // r[13] directly; they never touch the banked msp / psp fields.
        // Sync r[13] back into the currently-active banked SP before we
        // read it — otherwise the frame lands at a stale address.
        self.regs.sync_sp_to_banked();

        let use_psp = !self.regs.in_handler_mode() && self.regs.active_sp_is_psp();
        let original_sp = if use_psp { self.regs.psp } else { self.regs.msp };

        // 8-byte alignment: pre-decrement by 4 when SP is misaligned.
        let aligned_sp = original_sp & !0x7;
        let was_padded = aligned_sp != original_sp;
        let frame_sp = aligned_sp.wrapping_sub(32);

        // Encode alignment bit into stacked xPSR (bit 9). Other xPSR
        // bits are copied verbatim — on M0+ there's no IT state to mask.
        let mut stacked_xpsr = self.regs.xpsr & !(1 << 9);
        if was_padded {
            stacked_xpsr |= 1 << 9;
        }

        // Push in the order dictated by the ARMv6-M ARM: low-address
        // slot holds R0, high-address slot holds xPSR.
        bus.write32(frame_sp, self.regs.r[0]);
        bus.write32(frame_sp.wrapping_add(4), self.regs.r[1]);
        bus.write32(frame_sp.wrapping_add(8), self.regs.r[2]);
        bus.write32(frame_sp.wrapping_add(12), self.regs.r[3]);
        bus.write32(frame_sp.wrapping_add(16), self.regs.r[12]);
        bus.write32(frame_sp.wrapping_add(20), self.regs.lr());
        bus.write32(frame_sp.wrapping_add(24), self.return_address(exc_num));
        bus.write32(frame_sp.wrapping_add(28), stacked_xpsr);

        // Update the banked SP.
        if use_psp {
            self.regs.psp = frame_sp;
        } else {
            self.regs.msp = frame_sp;
        }

        // EXC_RETURN magic in LR — three valid values on M0+.
        self.regs.r[14] = if self.regs.in_handler_mode() {
            0xFFFF_FFF1 // return to Handler mode (nested), MSP
        } else if use_psp {
            0xFFFF_FFFD // return to Thread, PSP
        } else {
            0xFFFF_FFF9 // return to Thread, MSP
        };

        // Vector fetch.
        let vtor = bus.ppb[bus.active_core()].vtor;
        let vector = bus.read32(vtor.wrapping_add((exc_num as u32) * 4));

        // Architecturally, vector entries must have the T bit set.
        // ARMv6-M ARM §B1.5 treats a cleared T bit as an entry-path fault:
        //   * entering HardFault → HardFault-in-HardFault → lockup.
        //   * entering anything else → escalate to HardFault (as though
        //     the entry itself faulted). The frame we just pushed stays
        //     put; the outer step loop will pick up pending_fault and
        //     re-enter at vector #3.
        if vector & 1 == 0 {
            if exc_num == 3 {
                self.halted = true;
                return 16;
            }
            // Frame is already committed (push + SP update completed). The
            // new HardFault attempt will stack on top when the outer step
            // loop delivers it. Sync r[13] from the updated banked SP so
            // the follow-up entry sees the post-push stack pointer.
            self.regs.sync_sp_from_banked();
            self.pending_fault = Some(Fault::HardFault);
            return 16;
        }
        self.regs.set_pc(vector & !1);

        // Enter handler mode: IPSR = exc_num, CONTROL.SPSEL forced to 0.
        self.regs.xpsr = (self.regs.xpsr & !0x1FF) | (exc_num as u32);
        self.regs.control &= !0x2; // handler always uses MSP
        self.regs.sync_sp_from_banked();

        bus.ppb[bus.active_core()].mark_active(exc_num);
        16
    }

    // -------------------------------------------------------------------
    // Exception return
    // -------------------------------------------------------------------

    /// Pop the 8-word frame, restore mode. Returns cycle cost (~12).
    ///
    /// Called when a branch-to-PC instruction (BX, POP {PC}, LDM-to-PC)
    /// writes a value with the EXC_RETURN pattern. Validates the magic
    /// and selects the target stack.
    pub(crate) fn exit_exception(&mut self, exc_return: u32, bus: &mut Bus) -> u32 {
        let active_exc = self.regs.ipsr();

        // Handler-mode SP manipulation (SUB SP / ADD SP / PUSH / POP) writes
        // r[13] directly without syncing to the banked msp. Push r[13] back
        // into msp before we read it, so we unwind from the real stack
        // pointer rather than a stale snapshot taken at entry.
        self.regs.sync_sp_to_banked();

        // Validate the low nibble — only 0x1, 0x9, 0xD are legal.
        let return_to_psp = match exc_return & 0xF {
            0x1 => {
                // Return to Handler mode — the enclosing exception is
                // still active. Must use MSP.
                false
            }
            0x9 => false,
            0xD => true,
            _ => {
                self.pending_fault = Some(Fault::InvalidExcReturn);
                return 0;
            }
        };
        let return_to_handler = (exc_return & 0xF) == 0x1;

        // Integrity: returning to Handler mode requires another active
        // exception (otherwise IPSR would become 0 but we're still in
        // Handler mode per EXC_RETURN[3:0] = 0x1). ARMv6-M ARM §B1.5.8.
        if return_to_handler && !bus.ppb[bus.active_core()].any_active() {
            self.pending_fault = Some(Fault::InvalidExcReturn);
            return 0;
        }

        let sp = if return_to_psp { self.regs.psp } else { self.regs.msp };

        // Pop the 8-word frame.
        self.regs.r[0] = bus.read32(sp);
        self.regs.r[1] = bus.read32(sp.wrapping_add(4));
        self.regs.r[2] = bus.read32(sp.wrapping_add(8));
        self.regs.r[3] = bus.read32(sp.wrapping_add(12));
        self.regs.r[12] = bus.read32(sp.wrapping_add(16));
        self.regs.r[14] = bus.read32(sp.wrapping_add(20));
        let return_pc = bus.read32(sp.wrapping_add(24));
        let return_xpsr = bus.read32(sp.wrapping_add(28));

        self.regs.set_pc(return_pc & !1);

        // Alignment pad from bit 9 of stacked xPSR — reverse the
        // pre-decrement applied on entry.
        let frame_size: u32 = if return_xpsr & (1 << 9) != 0 { 36 } else { 32 };
        // Restore xPSR, clearing the alignment bit we borrowed.
        self.regs.xpsr = return_xpsr & !(1 << 9);

        // Deallocate frame on the selected stack.
        if return_to_psp {
            self.regs.psp = sp.wrapping_add(frame_size);
        } else {
            self.regs.msp = sp.wrapping_add(frame_size);
        }

        // Restore CONTROL.SPSEL based on EXC_RETURN and handler-vs-thread.
        // In Handler mode SPSEL is forced to 0; in Thread mode it
        // follows the EXC_RETURN selection.
        if return_to_handler {
            self.regs.control &= !0x2;
        } else {
            self.regs.control = (self.regs.control & !0x2) | if return_to_psp { 0x2 } else { 0 };
        }
        self.regs.sync_sp_from_banked();

        // Drop the outgoing exception from the active set.
        bus.ppb[bus.active_core()].clear_active(active_exc as u16);

        12
    }

    // -------------------------------------------------------------------
    // Fault delivery
    // -------------------------------------------------------------------

    /// Deliver a pending synchronous fault by entering the appropriate
    /// exception handler. Returns cycle cost (entry ≈ 16).
    pub(crate) fn deliver_fault(&mut self, fault: Fault, bus: &mut Bus) -> u32 {
        match fault {
            // SVCall lands in its own vector — but only if it can actually
            // preempt. Per ARMv6-M ARM §B1.5.8, if PRIMASK blocks SVCall
            // from running, the SVC escalates to HardFault. SHPR priorities
            // aren't fully wired in Phase 4.B, so we apply the simplified
            // rule: PRIMASK=1 → HardFault. PRIMASK=0 → deliver SVCall
            // normally (SVCall and execution priorities are both 0).
            Fault::Svc => {
                if self.regs.primask & 1 != 0 {
                    self.enter_exception(3, bus)
                } else {
                    self.enter_exception(11, bus)
                }
            }
            Fault::Undefined
            | Fault::Unaligned
            | Fault::HardFault
            | Fault::InvalidExcReturn
            | Fault::InvalidEpsr => self.enter_exception(3, bus),
        }
    }
}

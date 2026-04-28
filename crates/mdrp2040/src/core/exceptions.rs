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
//!     - any other low-nibble → HardFault (InvalidExcReturn).
//!
//! Cycle counts: entry ≈ 16, exit ≈ 12 (M0+ TRM typical).

use tracing::debug;

use super::{CoreBus, CortexM0Plus, Fault};

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
            3 => self.current_instr_addr, // HardFault — retry faulting instr
            11 => self.regs.pc(),         // SVC — return after the SVC
            _ => self.regs.pc(),          // async / NMI / PendSV / SysTick
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
    pub(crate) fn enter_exception<B: CoreBus>(&mut self, exc_num: u16, bus: &mut B) -> u32 {
        // Publish a sentinel "hardware-triggered exception stacking" PC so
        // the MMIO trace distinguishes the 8 stacking writes from the
        // faulting instruction's own access pattern. Value `0xFFFF_FFFE`
        // cannot collide with a real Thumb instruction PC (those are
        // even-aligned in the low 28 bits of the address map). Regular
        // PC publishing resumes at the handler's first `decode_execute`.
        bus.set_active_pc(0xFFFF_FFFE);

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
        let original_sp = if use_psp {
            self.regs.psp
        } else {
            self.regs.msp
        };

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
        let active_core = bus.active_core();
        let vtor = bus.ppb(active_core).vtor;
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

        // Priority lookup splits on whether this is a system exception or
        // an external IRQ — `Ppb::exception_priority` is for system
        // exceptions only (its debug_assert enforces this); external IRQ
        // priorities live in `Nvic::priority`.
        let priority_label: i16 = if exc_num < 16 {
            bus.ppb(active_core).exception_priority(exc_num)
        } else {
            bus.nvic(active_core).get_priority((exc_num - 16) as u8) as i16
        };
        bus.ppb_mut(active_core).mark_active(exc_num);

        debug!(
            exception_num = exc_num,
            priority = %priority_label,
            pc = format_args!("{:#010x}", vector & !1),
            lr = format_args!("{:#010x}", self.regs.lr()),
            "exception entry"
        );

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
    pub(crate) fn exit_exception<B: CoreBus>(&mut self, exc_return: u32, bus: &mut B) -> u32 {
        // Publish a sentinel "exception-return unstacking" PC so the
        // MMIO trace distinguishes the 8 unstacking reads from ordinary
        // instruction-driven access. Value `0xFFFF_FFFD` is paired with
        // the entry sentinel `0xFFFF_FFFE` and cannot collide with a
        // real Thumb instruction PC. Regular PC publishing resumes when
        // the returned-to instruction hits `decode_execute`.
        bus.set_active_pc(0xFFFF_FFFD);

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
        let active_core = bus.active_core();
        if return_to_handler && !bus.ppb(active_core).any_active() {
            self.pending_fault = Some(Fault::InvalidExcReturn);
            return 0;
        }

        let sp = if return_to_psp {
            self.regs.psp
        } else {
            self.regs.msp
        };

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
        bus.ppb_mut(active_core).clear_active(active_exc as u16);

        debug!(
            exc_return = format_args!("{:#010x}", exc_return),
            restored_pc = format_args!("{:#010x}", return_pc & !1),
            "exception return"
        );

        // Tail-chain poll: if any exception is still pending and
        // dispatchable now that we've exited handler mode, take it
        // without unwinding back to thread mode. Mirrors mdrp2350's
        // exit-path idiom (HLD V5 §5.3).
        //
        // Order matters on two counts:
        //   (a) we run *after* the SP unstack — so a tail-chain entry
        //       stacks onto a fresh thread-mode stack, not on top of
        //       the outgoing frame; and
        //   (b) we run *after* the active-flag clear above — so
        //       `can_dispatch_now` sees the outgoing handler as no
        //       longer active and permits the new dispatch.
        let chained = self.try_take_any_pending_exception(bus);
        12 + chained
    }

    // -------------------------------------------------------------------
    // Fault delivery
    // -------------------------------------------------------------------

    /// Deliver a pending synchronous fault by entering the appropriate
    /// exception handler. Returns cycle cost (entry ≈ 16).
    pub(crate) fn deliver_fault<B: CoreBus>(&mut self, fault: Fault, bus: &mut B) -> u32 {
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

#[cfg(test)]
mod tests {
    //! Unit tests for the unified exception dispatcher
    //! `try_take_any_pending_exception` and the tail-chain poll inside
    //! `exit_exception`. HLD V5 §5.3.

    use crate::bus::Bus;
    use crate::core::CortexM0Plus;

    /// Plant a simple vector table at `0x2000_0000` with all vectors
    /// pointing at distinct handler addresses (`0x2000_1000 + N*32`).
    /// Returns `(bus, handlers)` so callers can assert PC after entry.
    fn make_bus_with_vectors() -> (Bus, [u32; 17]) {
        let mut bus = Bus::default();
        let vtor: u32 = 0x2000_0000;
        let mut handlers = [0u32; 17];
        for i in 0..17 {
            let handler = 0x2000_1000 + (i as u32) * 32;
            bus.write32(vtor + (i as u32) * 4, handler | 1);
            handlers[i] = handler;
        }
        bus.ppb[0].vtor = vtor;
        (bus, handlers)
    }

    /// Build a CPU with stack pointer / PC primed for thread-mode entry.
    fn fresh_cpu() -> CortexM0Plus {
        let mut cpu = CortexM0Plus::new();
        cpu.regs.msp = 0x2000_8000;
        cpu.regs.set_sp(0x2000_8000);
        // PC at a NOP slide so any non-dispatching step decodes cleanly.
        cpu.regs.set_pc(0x2000_4000);
        cpu
    }

    /// Plant a `B .` (self-loop) at the given address so a `step()` that
    /// does NOT dispatch an exception executes a benign instruction.
    fn plant_self_loop(bus: &mut Bus, addr: u32) {
        bus.write16(addr, 0xE7FE);
    }

    // -------------------------------------------------------------------
    // PendSV dispatch
    // -------------------------------------------------------------------

    #[test]
    fn pendsv_dispatches_when_pending_and_primask_clear() {
        // ICSR.PENDSVSET (bit 28) latched, PRIMASK clear, no handler
        // active. The pre-fetch poll inside `step` must dispatch
        // exception #14 and leave the latch cleared.
        let (mut bus, handlers) = make_bus_with_vectors();
        plant_self_loop(&mut bus, 0x2000_4000);
        let mut cpu = fresh_cpu();
        bus.ppb[0].icsr |= 1 << 28;

        cpu.step(&mut bus);

        assert_eq!(cpu.regs.ipsr(), 14, "PendSV (#14) must dispatch");
        assert_eq!(cpu.regs.pc(), handlers[14], "PC at PendSV handler");
        assert_eq!(
            bus.ppb[0].icsr & (1 << 28),
            0,
            "PENDSVSET latch cleared on dispatch"
        );
    }

    #[test]
    fn pendsv_blocked_by_primask() {
        // PENDSVSET latched but PRIMASK=1 — dispatcher must defer; the
        // CPU executes the instruction at PC (the self-loop) instead.
        let (mut bus, _handlers) = make_bus_with_vectors();
        plant_self_loop(&mut bus, 0x2000_4000);
        let mut cpu = fresh_cpu();
        cpu.regs.primask = 1;
        bus.ppb[0].icsr |= 1 << 28;

        cpu.step(&mut bus);

        assert_eq!(cpu.regs.ipsr(), 0, "PRIMASK=1 must keep us in thread mode");
        assert_ne!(
            bus.ppb[0].icsr & (1 << 28),
            0,
            "PRIMASK leaves PENDSVSET latched"
        );
    }

    #[test]
    fn pendsv_blocked_by_active_handler() {
        // V1 limitation pinned: an already-active handler blocks
        // dispatch even if the new candidate is higher priority.
        // `can_dispatch_now` is stricter than ARMv6-M ARM §B1.5.4.
        let (mut bus, _handlers) = make_bus_with_vectors();
        plant_self_loop(&mut bus, 0x2000_4000);
        let mut cpu = fresh_cpu();
        // Mark an exception as active without entering it.
        bus.ppb[0].mark_active(11); // SVCall active
        // Fake handler-mode IPSR so a step inside the handler is a no-op
        // execute (PC at self-loop is fine; this test asserts the
        // dispatcher doesn't fire, not what runs instead).
        cpu.regs.xpsr = (cpu.regs.xpsr & !0x1FF) | 11;
        bus.ppb[0].icsr |= 1 << 28;

        cpu.step(&mut bus);

        assert_eq!(
            cpu.regs.ipsr(),
            11,
            "active handler must block PendSV dispatch"
        );
        assert_ne!(
            bus.ppb[0].icsr & (1 << 28),
            0,
            "PENDSVSET stays latched when blocked by active handler"
        );
    }

    // -------------------------------------------------------------------
    // Priority arbitration across system + external exceptions
    // -------------------------------------------------------------------

    #[test]
    fn external_irq_priority_above_pendsv_when_lower_value() {
        // PendSV configured priority 0xC0; IRQ 0 priority 0x40. Lower
        // numerical value wins, so IRQ 0 (#16) must dispatch before
        // PendSV (#14) even though 14 < 16 in the tie-break order.
        let (mut bus, handlers) = make_bus_with_vectors();
        plant_self_loop(&mut bus, 0x2000_4000);
        let mut cpu = fresh_cpu();
        // SHPR3 byte 2 → PendSV priority 0xC0.
        bus.ppb[0].shpr[10] = 0xC0;
        // IRQ 0 priority 0x40, enabled and pending.
        bus.nvics[0].set_priority(0, 0x40);
        bus.nvics[0].set_enabled(0);
        bus.nvics[0].set_pending(0);
        bus.ppb[0].icsr |= 1 << 28;

        cpu.step(&mut bus);

        assert_eq!(
            cpu.regs.ipsr(),
            16,
            "IRQ 0 (priority 0x40) wins over PendSV (0xC0)"
        );
        assert_eq!(cpu.regs.pc(), handlers[16]);
        assert!(
            !bus.nvics[0].is_pending(0),
            "dispatch clears the NVIC pending bit"
        );
        assert_ne!(
            bus.ppb[0].icsr & (1 << 28),
            0,
            "PENDSVSET stays latched — only the chosen candidate clears"
        );
    }

    #[test]
    fn pendsv_priority_above_systick_when_equal_tie_break_by_number() {
        // PendSV and SysTick both priority 0x40; both pending. Tie-break
        // rule: lower exception number wins → PendSV (#14) over
        // SysTick (#15).
        let (mut bus, handlers) = make_bus_with_vectors();
        plant_self_loop(&mut bus, 0x2000_4000);
        let mut cpu = fresh_cpu();
        bus.ppb[0].shpr[10] = 0x40; // PendSV
        bus.ppb[0].shpr[11] = 0x40; // SysTick
        bus.ppb[0].icsr |= (1 << 28) | (1 << 26);

        cpu.step(&mut bus);

        assert_eq!(
            cpu.regs.ipsr(),
            14,
            "tie-break by lower exception number → PendSV"
        );
        assert_eq!(cpu.regs.pc(), handlers[14]);
        assert_eq!(bus.ppb[0].icsr & (1 << 28), 0, "PENDSVSET cleared");
        assert_ne!(
            bus.ppb[0].icsr & (1 << 26),
            0,
            "PENDSTSET stays latched — the loser remains pending"
        );
    }

    // -------------------------------------------------------------------
    // Tail-chain
    // -------------------------------------------------------------------

    #[test]
    fn pendsv_systick_tail_chain_skips_thread_mode_resume() {
        // Both PendSV and SysTick pending. Step #1 dispatches the
        // higher-priority pair member (default priorities both 0 →
        // PendSV wins by tie-break). After the handler returns, the
        // tail-chain poll must dispatch SysTick *without* unwinding to
        // thread mode.
        let (mut bus, handlers) = make_bus_with_vectors();
        plant_self_loop(&mut bus, 0x2000_4000);
        let mut cpu = fresh_cpu();
        bus.ppb[0].icsr |= (1 << 28) | (1 << 26);

        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 14, "first step dispatches PendSV");
        assert_eq!(cpu.regs.pc(), handlers[14]);
        // Active flag for PendSV is set.
        assert_ne!(bus.ppb[0].active & (1 << 14), 0);

        // Simulate the handler exiting via EXC_RETURN to Thread+MSP.
        cpu.test_exit_exception(0xFFFF_FFF9, &mut bus);

        // Tail-chain must have fired SysTick during exit_exception.
        assert_eq!(
            cpu.regs.ipsr(),
            15,
            "tail-chain dispatched SysTick without thread-mode resume"
        );
        assert_eq!(cpu.regs.pc(), handlers[15]);
        // PENDSTSET cleared by the tail-chain dispatch.
        assert_eq!(bus.ppb[0].icsr & (1 << 26), 0);
        // SysTick is now active; PendSV no longer active.
        assert_ne!(bus.ppb[0].active & (1 << 15), 0, "SysTick active");
        assert_eq!(bus.ppb[0].active & (1 << 14), 0, "PendSV no longer active");
    }

    // -------------------------------------------------------------------
    // IPSR write-back invariants for system exceptions
    // -------------------------------------------------------------------

    #[test]
    fn pendsv_handler_observes_ipsr_equals_14() {
        // Cheap insurance against a code path that special-cases
        // `exc_num >= 16` for the IPSR writeback. The oracle's
        // HANDLER_TAIL routine reads IPSR via `MRS r2, IPSR; cmp r2, #14`
        // to disambiguate PendSV from SysTick — if PendSV ever leaves
        // IPSR at 0, that branch always falls through to the SysTick
        // increment and ctr_pendsv stays at 0 with no diagnostic.
        let (mut bus, _handlers) = make_bus_with_vectors();
        let mut cpu = fresh_cpu();
        cpu.test_enter_exception(14, &mut bus);
        assert_eq!(
            cpu.regs.xpsr & 0x1FF,
            14,
            "PendSV handler must see IPSR == 14"
        );
    }

    #[test]
    fn systick_handler_observes_ipsr_equals_15() {
        let (mut bus, _handlers) = make_bus_with_vectors();
        let mut cpu = fresh_cpu();
        cpu.test_enter_exception(15, &mut bus);
        assert_eq!(
            cpu.regs.xpsr & 0x1FF,
            15,
            "SysTick handler must see IPSR == 15"
        );
    }
}

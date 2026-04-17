use std::sync::atomic::Ordering;

use crate::bus::Bus;
use crate::bus::ppb::{FPCCR_BFRDY, FPCCR_LSPACT, FPCCR_LSPEN, FPCCR_MMRDY,
    FPCCR_SPLIMVIOL};
use super::{CortexM33, Fault};

/// CONTROL.FPCA bit position (bit 2). Owned exclusively by the three sites
/// in this file plus `fpu_execute`; see `Ppb` field doc invariants.
pub(crate) const CONTROL_FPCA: u32 = 1 << 2;

/// CFSR.UFSR.STKOF (bit 20) — stack overflow during hardware exception entry.
/// Set by the stack-limit check in `enter_exception` when the reserved frame
/// (basic + optional FP region) would underflow past M/PSPLIM. Phase 7
/// Stage B introduced the FP-region contribution.
const UFSR_STKOF: u32 = 1 << 20;

/// CFSR.UFSR.INVPC (bit 17) — integrity check failure on EXC_RETURN.
/// Set by `exit_exception` on an FP-frame mismatch: either EXC_RETURN[4]=0
/// claims an FP frame but no entry path ever reserved one (FPCAR=0 and
/// LSPACT=0), or EXC_RETURN[4]=1 claims no FP frame but FPCCR.LSPACT=1 is
/// still set from a lazy reserve at entry.
const UFSR_INVPC: u32 = 1 << 17;

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
    ///
    /// ARMv8-M §B3.4.1 lockup: if a HardFault is taken while already in the
    /// HardFault handler (IPSR==3), the processor enters lockup state. We
    /// emulate this by halting the core so tests observe the lockup rather
    /// than spinning on a crash loop.
    pub(crate) fn enter_exception(&mut self, exc_num: u16, bus: &mut Bus) -> u32 {
        // Publish a sentinel "hardware-triggered exception stacking" PC so
        // the MMIO trace distinguishes the stacking writes from the
        // faulting instruction's own access pattern. Value `0xFFFF_FFFE`
        // cannot collide with a real Thumb instruction PC (those are
        // even-aligned in the low 28 bits of the address map). Regular
        // PC publishing resumes at the handler's first `decode_execute`.
        bus.set_active_pc(0xFFFF_FFFE, self.core_id);

        // ARMv8-M B1.5.18: exception entry clears the local exclusive
        // monitor. Without this, a LDREX/STREX pair straddling an
        // exception would let STREX succeed on return — the monitor
        // must be reset so returning code re-issues LDREX.
        self.exclusive_address = None;

        if exc_num == 3 && self.regs.ipsr() == 3 {
            self.atomics.set_halted(self.core_id as usize);
            return 0;
        }
        let use_psp = !self.regs.in_handler_mode() && self.regs.active_sp_is_psp();
        let original_sp = if use_psp { self.regs.psp } else { self.regs.msp };

        // FP frame: thread mode with CONTROL.FPCA=1 forces FType=0 (FP frame
        // present). DDI0553 §B3.4.3. Per HLD §B.4 and Phase 7 stub: only
        // FPCA=1 → FP frame; we don't recompute via FPCCR.ASPEN since
        // fpu_execute is the sole FPCA-on writer.
        let had_fp = self.regs.control & CONTROL_FPCA != 0;

        // 8-byte align with padding tracking (CCR.STKALIGN is RAO on M33)
        let aligned_sp = original_sp & !0x7;
        let basic_frame: u32 = 32;
        // Extra FP region: 18 words = S0-S15 + FPSCR + 1 reserved.
        let fp_extra: u32 = if had_fp { 72 } else { 0 };
        let frame_sp = aligned_sp.wrapping_sub(basic_frame + fp_extra);
        let was_padded = aligned_sp != original_sp;

        // Stack-limit check (Armv8-M MSPLIM/PSPLIM, §B3.4.1). Compares
        // before any frame writes. On violation, raise UsageFault with
        // UFSR.STKOF (bit 20 of CFSR). Additionally, per DDI0553 §D1.2.32,
        // FPCCR.SPLIMVIOL is set iff the FP region specifically caused
        // the violation — i.e. the basic frame alone would have fit but
        // adding the FP region pushes the SP past the limit. If the
        // basic frame alone already underflows, SPLIMVIOL stays clear
        // (the violation is not attributable to FP context).
        let limit = if use_psp { self.regs.psplim } else { self.regs.msplim };
        if frame_sp < limit {
            if had_fp && aligned_sp.wrapping_sub(basic_frame) >= limit {
                // Basic frame alone would fit; the FP region drove the
                // underflow → SPLIMVIOL attributable to FP.
                self.ppb.fpccr |= FPCCR_SPLIMVIOL;
            }
            self.ppb.cfsr |= UFSR_STKOF;
            self.pending_fault = Some(Fault::UsageFault);
            return 0;
        }

        // Encode IT state and alignment padding into stacked xPSR.
        // Mask IT bits [26:25,15:10] from base xPSR first to avoid OR corruption
        // from stale bits left by a previous exit_exception.
        const IT_MASK: u32 = 0x0600_FC00;
        let mut stacked_xpsr = (self.regs.xpsr & !IT_MASK) | self.encode_it_to_xpsr();
        if was_padded {
            stacked_xpsr |= 1 << 9;
        }

        // Push exception frame: R0, R1, R2, R3, R12, LR, ReturnAddress, xPSR
        self.bus_write32(frame_sp, self.regs.r[0], bus);
        self.bus_write32(frame_sp.wrapping_add(4), self.regs.r[1], bus);
        self.bus_write32(frame_sp.wrapping_add(8), self.regs.r[2], bus);
        self.bus_write32(frame_sp.wrapping_add(12), self.regs.r[3], bus);
        self.bus_write32(frame_sp.wrapping_add(16), self.regs.r[12], bus);
        self.bus_write32(frame_sp.wrapping_add(20), self.regs.lr(), bus);
        self.bus_write32(frame_sp.wrapping_add(24), self.return_address(exc_num), bus);
        self.bus_write32(frame_sp.wrapping_add(28), stacked_xpsr, bus);

        // FP context — eager (LSPEN=0) writes S0-S15 + FPSCR now; lazy
        // (LSPEN=1, default) reserves the slots and sets FPCCR.LSPACT
        // so the first FP op in handler mode performs the flush.
        if had_fp {
            let fp_region_sp = frame_sp.wrapping_add(basic_frame);
            let lspen = self.ppb.fpccr & FPCCR_LSPEN != 0;
            // Always record FPCAR — needed by lazy path and by the
            // exit-pop path when LSPACT is cleared.
            self.ppb.fpcar = fp_region_sp;
            if lspen {
                // Lazy: do not write S0-S15. Mark LSPACT so the first
                // in-handler FP op flushes; clear any stale RDY bits we
                // leave behind from a prior fault.
                self.ppb.fpccr |= FPCCR_LSPACT;
                self.ppb.fpccr &= !(FPCCR_MMRDY | FPCCR_BFRDY);
            } else {
                // Eager: write S0-S15 + FPSCR + reserved word. Layout
                // per DDI0553 §B3.4.3 ExceptionEntry pseudocode.
                for i in 0..16 {
                    self.bus_write32(
                        fp_region_sp.wrapping_add((i as u32) * 4),
                        self.regs.s[i].to_bits(),
                        bus,
                    );
                }
                self.bus_write32(fp_region_sp.wrapping_add(64), self.regs.fpscr, bus);
                self.bus_write32(fp_region_sp.wrapping_add(68), 0, bus);
            }
            // Reset FPSCR from FPDSCR active bits: AHP[26], DN[25],
            // FZ[24], RMODE[23:22] (DDI0553 §B3.4.3). Cumulative
            // exception flags are NOT cleared by exception entry.
            let fpdscr_mask: u32 = (1 << 26) | (1 << 25) | (1 << 24) | (0b11 << 22);
            let fpdscr = self.ppb.fpdscr & fpdscr_mask;
            self.regs.fpscr = (self.regs.fpscr & !fpdscr_mask) | fpdscr;
        }

        // Update SP
        if use_psp {
            self.regs.psp = frame_sp;
        } else {
            self.regs.msp = frame_sp;
        }

        // FIXME(trustzone): these values don't encode the S bit — NS exceptions will claim Secure return
        // Set LR to EXC_RETURN (Armv8-M, non-secure). Bit [4] = FType: 0
        // means an FP frame is present (so 0xFFFF_FFE_) and 1 means no FP
        // frame (so 0xFFFF_FFF_, matching Phase 3 behavior). The S=1 stub
        // is preserved per HLD §2 non-goals.
        let base = if had_fp { 0xFFFF_FFE0_u32 } else { 0xFFFF_FFF0_u32 };
        self.regs.r[14] = base | if self.regs.in_handler_mode() {
            0x1 // return to Handler, MSP
        } else if use_psp {
            0xD // return to Thread, PSP
        } else {
            0x9 // return to Thread, MSP
        };

        // Fetch vector from table
        let vtor = self.ppb.vtor;
        let vector = self.bus_read32(vtor.wrapping_add((exc_num as u32) * 4), bus);
        self.regs.set_pc(vector & !1);

        // Enter handler mode: set IPSR, force MSP, clear IT.
        // Clear CONTROL.FPCA: handler enters as a non-FP-active context
        // (the saved/lazy frame remembers prior thread state).
        self.regs.xpsr = (self.regs.xpsr & !0x1FF) | (exc_num as u32);
        self.regs.control &= !2; // handler always MSP
        self.regs.control &= !CONTROL_FPCA;
        self.regs.sync_sp_from_banked();
        self.it_state = 0;

        12
    }

    // --- Exception return ---

    /// Pop exception frame, restore mode. Returns cycle cost (~12).
    pub(crate) fn exit_exception(&mut self, exc_return: u32, bus: &mut Bus) -> u32 {
        // Publish a sentinel "exception-return unstacking" PC so the
        // MMIO trace distinguishes the unstacking reads from ordinary
        // instruction-driven access. Value `0xFFFF_FFFD` is paired with
        // the entry sentinel `0xFFFF_FFFE` and cannot collide with a
        // real Thumb instruction PC. Regular PC publishing resumes when
        // the returned-to instruction hits `decode_execute`.
        bus.set_active_pc(0xFFFF_FFFD, self.core_id);

        // ARMv8-M B1.5.18: exception return clears the local exclusive
        // monitor, matching the entry-side clear.
        self.exclusive_address = None;

        let active_exc = self.regs.ipsr(); // capture BEFORE popping

        let return_to_psp = exc_return & 0x4 != 0;
        // FType=0 (bit 4 clear) ⇒ FP frame present and must be unwound.
        let had_fp_frame = exc_return & 0x10 == 0;

        // Integrity check (DDI0553 §B3.4.4 ExceptionReturn): EXC_RETURN[4]
        // must match the FP state reserved at entry. Two inconsistent-state
        // cases both raise UsageFault.INVPC:
        //
        //   1. FType=0 (had_fp_frame) but no entry path ever reserved an FP
        //      frame — FPCAR=0 and LSPACT=0. LR was fabricated.
        //   2. FType=1 (no_fp_frame) but FPCCR.LSPACT=1 — a lazy reservation
        //      from entry is still outstanding, so the handler is returning
        //      with an EXC_RETURN that claims no FP context despite one
        //      being architecturally pending. Silently clearing LSPACT would
        //      leave FPCAR pointing at (potentially) stale stack memory, so
        //      the next thread-mode FP op would flush into that region.
        //      The spec-correct response is to catch the mismatch.
        //
        // Rationale for not silent-clearing: the integrity check catches the
        // handler bug; a silent clear would mask it and allow a stale-FPCAR
        // flush to corrupt memory on the next FP op.
        {
            let fpccr = self.ppb.fpccr;
            let lspact = fpccr & FPCCR_LSPACT != 0;
            let bogus = if had_fp_frame {
                // Case 1: FType=0 but nothing reserved.
                !lspact && self.ppb.fpcar == 0
            } else {
                // Case 2: FType=1 but a lazy reservation is outstanding.
                lspact
            };
            if bogus {
                self.ppb.cfsr |= UFSR_INVPC;
                self.pending_fault = Some(Fault::UsageFault);
                return 0;
            }
        }

        let sp = if return_to_psp { self.regs.psp } else { self.regs.msp };

        // Pop basic frame
        self.regs.r[0] = self.bus_read32(sp, bus);
        self.regs.r[1] = self.bus_read32(sp.wrapping_add(4), bus);
        self.regs.r[2] = self.bus_read32(sp.wrapping_add(8), bus);
        self.regs.r[3] = self.bus_read32(sp.wrapping_add(12), bus);
        self.regs.r[12] = self.bus_read32(sp.wrapping_add(16), bus);
        self.regs.r[14] = self.bus_read32(sp.wrapping_add(20), bus);
        let return_pc = self.bus_read32(sp.wrapping_add(24), bus);
        let return_xpsr = self.bus_read32(sp.wrapping_add(28), bus);

        self.regs.set_pc(return_pc & !1);

        // FP frame restore (HLD §B.6).
        //   LSPACT=1 → handler never touched FP. S0-S15 still hold the
        //              pre-exception values; just skip the pop and clear
        //              LSPACT. FPSCR retains the in-handler value (which
        //              equals the pre-exception value — see fpu_execute).
        //   LSPACT=0 → an FP op in the handler triggered the lazy flush,
        //              or eager mode wrote the frame. Pop S0-S15 + FPSCR.
        if had_fp_frame {
            let fp_region_sp = sp.wrapping_add(32);
            let lspact = self.ppb.fpccr & FPCCR_LSPACT != 0;
            if lspact {
                self.ppb.fpccr &= !FPCCR_LSPACT;
            } else {
                for i in 0..16 {
                    let bits = self.bus_read32(fp_region_sp.wrapping_add((i as u32) * 4), bus);
                    self.regs.s[i] = f32::from_bits(bits);
                }
                self.regs.fpscr = self.bus_read32(fp_region_sp.wrapping_add(64), bus);
            }
        }

        // Alignment padding check (bit 9 of stacked xPSR)
        let mut frame_size: u32 = if return_xpsr & (1 << 9) != 0 { 36 } else { 32 };
        if had_fp_frame {
            // FP region is 18 words = 72 bytes.
            frame_size = frame_size.saturating_add(72);
        }

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

        // Restore SPSEL and FPCA. CONTROL.FPCA = NOT EXC_RETURN[4]: an FP
        // frame on entry implies FP-active thread state to resume.
        self.regs.control = (self.regs.control & !2) | if return_to_psp { 2 } else { 0 };
        if had_fp_frame {
            self.regs.control |= CONTROL_FPCA;
        } else {
            self.regs.control &= !CONTROL_FPCA;
        }
        self.regs.sync_sp_from_banked();

        // Clear active exception
        self.ppb.clear_active(active_exc as u16);

        12
    }

    // --- Priority evaluation ---

    /// Effective execution priority (lower = higher priority).
    ///
    /// Folds four contributions per ARMv8-M §B3.4:
    /// * FAULTMASK=1 clamps to -1 (HardFault priority).
    /// * PRIMASK=1 clamps to 0 (all configurable priorities masked).
    /// * **BASEPRI non-zero clamps to `basepri & 0xE0`** — pending IRQs
    ///   with priority value ≥ BASEPRI are masked. M33 implements 3 bits
    ///   of priority (bits [7:5]) so the stored byte is pre-masked to
    ///   `0xE0`; the compare against IRQ priorities is numeric.
    /// * IPSR > 0 pulls the currently-active exception's priority into
    ///   the running value.
    ///
    /// The lowest (most restrictive in architectural terms, numerically
    /// smallest) of these wins.
    pub(crate) fn execution_priority(&self) -> i16 {
        let mut prio: i16 = 256;
        if self.regs.faultmask & 1 != 0 {
            prio = -1;
        } else if self.regs.primask & 1 != 0 {
            prio = 0;
        }

        // BASEPRI: when non-zero, masks pending IRQs whose priority
        // value is >= BASEPRI. Folding as `prio = min(prio, basepri)`
        // preserves the ordering PendingPrio < ExecPrio ⇒ preempt.
        // BASEPRI is byte-wide; pre-mask to 0xE0 so callers who write
        // an unmasked byte observe the architectural fold.
        let basepri = (self.regs.basepri & 0xFF) as u8;
        if basepri != 0 {
            let bp = (basepri & 0xE0) as i16;
            if bp < prio {
                prio = bp;
            }
        }

        let ipsr = self.regs.ipsr();
        if ipsr > 0 {
            let exc_prio = self.ppb.exception_priority(ipsr as u16);
            if exc_prio < prio {
                prio = exc_prio;
            }
        }
        prio
    }

    /// True if an exception numbered `exc_num` would preempt the core's
    /// current execution priority. Called by the unified exception-
    /// arbitration path and by tests that probe architectural priority
    /// behaviour (BASEPRI / PRIMASK / FAULTMASK / active-exception
    /// interactions).
    pub(crate) fn can_preempt(&self, exc_num: u16) -> bool {
        let exc_prio = self.ppb.exception_priority(exc_num);
        exc_prio < self.execution_priority()
    }

    /// Attempt to take the highest-priority pending exception at this
    /// instruction boundary, unified across NMI, PendSV, SysTick, and
    /// external NVIC IRQs. ARMv8-M §B3.7 mandates a single priority
    /// ordering over all pending exceptions — when both an asynchronous
    /// system exception and an external IRQ are pending, the numerically-
    /// lower priority wins (NMI's fixed -2 beats any configurable IRQ;
    /// an IRQ at priority 0x20 beats a PendSV at 0x80). Ties resolve to
    /// the lower exception number.
    ///
    /// Called at the top of `CortexM33::step`. Returns `Some(cycles)`
    /// if an exception was entered, `None` otherwise.
    ///
    /// NMI bypasses PRIMASK/FAULTMASK and never consults `can_preempt`;
    /// every other candidate goes through `can_preempt` so PRIMASK /
    /// BASEPRI / FAULTMASK / active-exception priority all apply.
    pub(crate) fn try_take_any_pending_exception(&mut self, bus: &mut Bus) -> Option<u32> {
        let icsr = self.ppb.icsr;

        // NMI (exc 2, priority -2): non-maskable, highest fixed priority.
        // No preempt check — NMI preempts unconditionally per ARMv8-M.
        if icsr & crate::bus::ppb::ICSR_NMIPENDSET != 0 {
            self.ppb.icsr &= !crate::bus::ppb::ICSR_NMIPENDSET;
            return Some(self.enter_exception(2, bus));
        }

        // Collect the three remaining candidate exceptions and pick the
        // numerically-lowest priority; tie-break by lower exception number.
        // `best` holds `(priority, exc_num)`; `None` means no candidate.
        let mut best: Option<(i16, u16)> = None;
        let pendsv = icsr & crate::bus::ppb::ICSR_PENDSVSET != 0;
        let pendst = icsr & crate::bus::ppb::ICSR_PENDSTSET != 0;
        if pendsv {
            best = Some((self.ppb.exception_priority(14), 14));
        }
        if pendst {
            let prio = self.ppb.exception_priority(15);
            best = match best {
                None => Some((prio, 15)),
                Some((bp, be)) if prio < bp || (prio == bp && 15 < be) => Some((prio, 15)),
                other => other,
            };
        }
        if let Some(ext_exc) = self.ppb.highest_priority_pending_irq() {
            let prio = self.ppb.exception_priority(ext_exc);
            best = match best {
                None => Some((prio, ext_exc)),
                Some((bp, be)) if prio < bp || (prio == bp && ext_exc < be) => Some((prio, ext_exc)),
                other => other,
            };
        }

        let (_, candidate) = best?;
        if !self.can_preempt(candidate) {
            return None;
        }

        // Dispatch-path cleanup differs by exception class: ICSR SET bits
        // for system exceptions, NVIC_ISPR + IABR for external IRQs.
        //
        // ***DUAL-CLEAR INVARIANT (Phase 0b.1 Commit B)***
        //
        // For external IRQs, we clear BOTH `bus.irq_pending[core]` (the
        // peripheral-facing short-circuit mask used to gate the NVIC
        // walk on the common no-IRQ path) AND `self.ppb.nvic_ispr[word]`
        // (the architectural latch). Keeping these in lockstep is what
        // makes the `merge_irq_pending` union-merge safe: a stale clear
        // in one without the other would either cause a silent no-fire
        // (clear nvic_ispr but leave irq_pending set → union re-pends
        // it on the next step) or an untracked fire (clear irq_pending
        // but leave nvic_ispr set → short-circuit gate opens while
        // latch still drives). Do not change this without re-checking
        // `Bus::assert_irq_core/shared`, `Bus::clear_irq_core/shared`,
        // `CortexM33::sync_nvic_to_irq_pending`, and the
        // `irq_pending_dirty` merge points in `step`/`Emulator::step`.
        match candidate {
            14 => {
                self.ppb.icsr &= !crate::bus::ppb::ICSR_PENDSVSET;
            }
            15 => {
                self.ppb.icsr &= !crate::bus::ppb::ICSR_PENDSTSET;
            }
            _ => {
                // External IRQ. See DUAL-CLEAR INVARIANT above.
                let irq = candidate - 16;
                let word = (irq / 32) as usize;
                let bit = irq % 32;
                if word < crate::bus::ppb::NVIC_BIT_WORDS {
                    let core = self.core_id as usize;
                    bus.atomics.clear_irq(core, irq as u32);
                    self.ppb.nvic_ispr[word].fetch_and(!(1u32 << bit), Ordering::Relaxed);
                }
                self.ppb.set_irq_active(irq as u32);
            }
        }
        Some(self.enter_exception(candidate, bus))
    }

    // --- Fault delivery ---

    /// Deliver a pending fault. Returns cycle cost.
    pub(crate) fn deliver_fault(&mut self, fault: Fault, bus: &mut Bus) -> u32 {
        match fault {
            Fault::UsageFault => {
                // Set UFSR.UNDEFINSTR (bit 16 of CFSR)
                self.ppb.cfsr |= 1 << 16;
                if self.ppb.shcsr & (1 << 18) != 0 {
                    // USGFAULTENA
                    self.enter_exception(6, bus)
                } else {
                    self.ppb.hfsr |= 1 << 30; // FORCED
                    self.enter_exception(3, bus) // escalate to HardFault
                }
            }
            Fault::MemManage => {
                // Set MMFSR.DACCVIOL (bit 1 of CFSR). Data-side is the honest default:
                // Phase 7 Stage E's MPU-fault-during-lazy-flush use case is data-side.
                self.ppb.cfsr |= 1 << 1;
                if self.ppb.shcsr & (1 << 16) != 0 {
                    // MEMFAULTENA
                    self.enter_exception(4, bus)
                } else {
                    self.ppb.hfsr |= 1 << 30; // FORCED
                    self.enter_exception(3, bus) // escalate to HardFault
                }
            }
            // NMI (exception #2) has priority -2. ARMv8-M §B3.4.1 escalation:
            //   * NMI arriving while already in the NMI handler (IPSR==2):
            //     cannot preempt itself — escalate to HardFault (priority -1),
            //     set HFSR.FORCED, and deliver exception #3.
            //   * HardFault handler (IPSR==3) is NOT disturbed by NMI on
            //     Armv8-M — NMI at -2 is higher priority than HardFault at -1,
            //     so it WOULD preempt. But here the only way we synthesize an
            //     NMI is CP7 RCP via deliver_fault; if the HardFault handler
            //     itself triggered another HardFault the proper path is lockup
            //     (handled elsewhere by HardFault-in-HardFault detection at
            //     enter_exception time).
            //   * Any other context (Thread mode or a lower-priority handler):
            //     deliver NMI normally.
            Fault::Nmi => {
                let ipsr = self.regs.ipsr();
                if ipsr == 2 {
                    // NMI-in-NMI — cannot preempt itself. Escalate to HardFault.
                    self.ppb.hfsr |= 1 << 30; // FORCED
                    self.enter_exception(3, bus)
                } else {
                    self.enter_exception(2, bus)
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
    pub(crate) fn execute_tt(&self, addr: u32) -> u32 {
        let ppb = &self.ppb;

        // RP2350 IDAU: built-in security attribution for the address space.
        let idau_result = Self::rp2350_idau(addr);

        // --- MPU contribution (MRVALID / MREGION + R/RW if matched) ---
        //
        // The bootrom's MPU self-test (see `check_mpu_loop2` in
        // `roms/rp2350/arm-bootrom.dis`) performs `tt r4, r4` on an address that
        // must land inside one of the just-written MPU regions, and
        // checks the result has MRVALID=1, MREGION=<expected>, R=1 (but
        // NOT RW, since the region was written with AP requiring priv-
        // only writes).
        let mpu_enabled = ppb.mpu_ctrl & 1 != 0;
        let mut mpu_bits: u32 = 0;
        if mpu_enabled {
            for i in 0..16 {
                let (rbar, rlar) = ppb.mpu_regions[i];
                if rlar & 1 == 0 {
                    continue; // region disabled
                }
                let base = rbar & !0x1F;
                let limit = (rlar & !0x1F) | 0x1F;
                if addr >= base && addr <= limit {
                    mpu_bits |= 1 << 16;            // MRVALID
                    mpu_bits |= (i as u32) & 0xFF;  // MREGION [7:0]
                    // R is always granted if the region matches from
                    // privileged-S state (TT always runs from privileged
                    // S here — the bootrom's self-test issues TT from
                    // the secure entry path). ARMv8-M ARM §B11.2.5 says
                    // the RBAR AP[2:1] field (bits [2:1]) encodes:
                    //   AP=00 → priv RW,        AP=01 → any RW,
                    //   AP=10 → priv RO,        AP=11 → any RO.
                    // We read the full 2-bit field (see `let ap` below)
                    // and grant RW when AP[1]=0, i.e. AP ∈ {0, 1}.
                    mpu_bits |= 1 << 18;                // R
                    let ap = (rbar >> 1) & 0x3;
                    if ap == 0 || ap == 1 {
                        mpu_bits |= 1 << 19;            // RW (writable)
                    }
                    break;
                }
            }
        }

        // --- SAU contribution (SRVALID / SREGION / S / NSR / NSRW) ---
        if ppb.sau_ctrl & 1 == 0 {
            // SAU disabled → everything Secure, fully accessible. No
            // SRVALID. MPU bits still honored.
            let mut r = idau_result | (1 << 22) | (1 << 19) | (1 << 18) | mpu_bits;
            // If MPU didn't grant, fall back to universal R/RW from SAU-off.
            if mpu_bits == 0 {
                r |= (1 << 19) | (1 << 18);
            }
            return r;
        }

        // SAU enabled: find matching region.
        let mut sau_bits: u32 = 0;
        let mut sau_matched = false;
        for i in 0..8 {
            let (rbar, rlar) = ppb.sau_regions[i];
            if rlar & 1 == 0 {
                continue;
            }
            let base = rbar & !0x1F;
            let limit = rlar | 0x1F;
            let nsc = (rlar >> 1) & 1;
            if addr >= base && addr <= limit {
                let secure = nsc == 0;
                sau_bits |= (i as u32 & 0xFF) << 8;       // SREGION
                sau_bits |= 1 << 17;                      // SRVALID
                if secure {
                    sau_bits |= 1 << 22;                  // S
                } else {
                    sau_bits |= (1 << 20) | (1 << 21);   // NSR, NSRW
                }
                sau_matched = true;
                break;
            }
        }
        if !sau_matched {
            let allns = (ppb.sau_ctrl >> 1) & 1;
            if allns != 0 {
                sau_bits |= (1 << 20) | (1 << 21);
            } else {
                sau_bits |= 1 << 22;
            }
        }

        // Compose. If no MPU match, fall back to universal R/RW (pre-Stage-E
        // behavior) so callers that don't configure MPU still see accessible
        // addresses as readable. When MPU has matched the address, the MPU's
        // permission bits win.
        let mut result = idau_result | sau_bits | mpu_bits;
        if mpu_bits == 0 {
            result |= (1 << 19) | (1 << 18);
        }
        result
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

#[cfg(test)]
mod tests {
    use crate::bus::Bus;
    use crate::core::{CortexM33, Fault};

    /// Address of the synthetic vector table base (relocated into SRAM so
    /// we can populate it).
    const VT_BASE: u32 = 0x2000_0000;
    /// Address of the synthetic handler (BKPT 0). Vectors below point here
    /// with bit 0 set for Thumb state.
    const HANDLER_ADDR: u32 = 0x2000_0100;
    /// Vector value (HANDLER_ADDR | 1 for Thumb).
    const HANDLER_VEC: u32 = HANDLER_ADDR | 1;

    /// Build a core + bus with MSP pointing at valid SRAM so exception
    /// frame pushes don't trigger a bus fault during delivery. Also
    /// installs a synthetic vector table at VT_BASE and points VTOR at
    /// it so enter_exception lands on a known handler address rather
    /// than reading a zero vector from the default (zero-filled) table.
    fn core_and_bus() -> (CortexM33, Bus) {
        let mut cpu = CortexM33::for_test(0);
        cpu.regs.msp = 0x2000_1000;
        cpu.regs.r[13] = cpu.regs.msp;

        // Phase 3 Stage 1: share the Arc<CoreAtomics> between core and
        // bus so signal paths (IRQ assert → step consume) route correctly.
        // The trip-wire in `CortexM33::step` enforces this invariant.
        let mut bus = Bus::with_atomics(std::sync::Arc::clone(&cpu.atomics));

        // Relocate VTOR into writable SRAM and populate vectors for the
        // exceptions exercised by these tests: NMI (2), HardFault (3),
        // MemManage (4). Phase 0b.1 Commit B: per-core PPB now lives on
        // `CortexM33`, so VTOR is set on `cpu.ppb`.
        cpu.ppb.vtor = VT_BASE;
        bus.write32(VT_BASE + 8,  HANDLER_VEC, 0); // NMI       (exc 2)
        bus.write32(VT_BASE + 12, HANDLER_VEC, 0); // HardFault (exc 3)
        bus.write32(VT_BASE + 16, HANDLER_VEC, 0); // MemManage (exc 4)
        // `B .` (0xE7FE) at the handler address — keeps the handler
        // well-defined without halting (BKPT halts via the debugger
        // semantic). Tests that exercise step-after-entry rely on the
        // handler being a no-op-equivalent.
        bus.write32(HANDLER_ADDR, 0x0000_E7FE, 0);

        (cpu, bus)
    }

    #[test]
    fn test_nmi_delivered() {
        let (mut cpu, mut bus) = core_and_bus();
        cpu.pending_fault = Some(Fault::Nmi);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 2);
        assert_eq!(cpu.regs.pc(), HANDLER_ADDR);
    }

    #[test]
    fn test_memmanage_enabled() {
        let (mut cpu, mut bus) = core_and_bus();
        cpu.ppb.shcsr |= 1 << 16; // MEMFAULTENA
        cpu.pending_fault = Some(Fault::MemManage);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 4);
        assert_eq!(cpu.regs.pc(), HANDLER_ADDR);
    }

    #[test]
    fn test_memmanage_disabled_escalates() {
        let (mut cpu, mut bus) = core_and_bus();
        cpu.ppb.shcsr &= !(1 << 16); // MEMFAULTENA cleared
        cpu.pending_fault = Some(Fault::MemManage);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 3);
        assert_ne!(cpu.ppb.hfsr & (1 << 30), 0, "HFSR.FORCED should be set");
        assert_eq!(cpu.regs.pc(), HANDLER_ADDR);
    }

    #[test]
    fn test_memmanage_sets_mmfsr_daccviol() {
        let (mut cpu, mut bus) = core_and_bus();
        cpu.ppb.shcsr |= 1 << 16; // MEMFAULTENA
        cpu.pending_fault = Some(Fault::MemManage);
        cpu.step(&mut bus);
        assert_ne!(cpu.ppb.cfsr & 0x2, 0, "MMFSR.DACCVIOL (bit 1) should be set");
        // Pin this as the non-escalated path: IPSR must be MemManage (4),
        // not HardFault (3). MMFSR.DACCVIOL would also be set after
        // escalation, so asserting IPSR is what distinguishes the paths.
        assert_eq!(cpu.regs.ipsr(), 4);
        assert_eq!(cpu.regs.pc(), HANDLER_ADDR);
    }

    /// ARMv8-M §B3.4.1: an NMI taken while the NMI handler is already
    /// running cannot preempt itself and must escalate to HardFault with
    /// HFSR.FORCED set. This is NOT the lockup path — only a HardFault
    /// arriving during HardFault locks up.
    #[test]
    fn test_nmi_in_nmi_handler_escalates_to_hardfault() {
        let (mut cpu, mut bus) = core_and_bus();

        // 1. Enter NMI normally.
        cpu.pending_fault = Some(Fault::Nmi);
        cpu.step(&mut bus);
        assert_eq!(cpu.regs.ipsr(), 2, "should be in NMI handler");

        // 2. Deliver another NMI while IPSR==2.
        cpu.pending_fault = Some(Fault::Nmi);
        cpu.step(&mut bus);

        // 3. Should have escalated to HardFault with FORCED set; NOT halted.
        assert_eq!(cpu.regs.ipsr(), 3,
            "NMI-in-NMI must escalate to HardFault (IPSR=3), got {}", cpu.regs.ipsr());
        assert_ne!(cpu.ppb.hfsr & (1 << 30), 0,
            "HFSR.FORCED should be set on escalation");
        assert!(!cpu.is_halted(), "NMI-in-NMI must NOT halt the core");
        assert_eq!(cpu.regs.pc(), HANDLER_ADDR);
    }

    /// ARMv8-M §B3.4.1: a HardFault taken while the HardFault handler is
    /// already running is the actual lockup path. The only synchronous
    /// fault we emit is NMI/MemManage/UsageFault (none of which deliver
    /// HardFault directly when IPSR==3 — they all go through
    /// enter_exception(3)), so we verify the enter_exception guard by
    /// placing the core in the HardFault handler and invoking it again.
    #[test]
    fn test_hardfault_in_hardfault_halts() {
        let (mut cpu, mut bus) = core_and_bus();

        // Manually place the core in the HardFault handler: set IPSR=3.
        cpu.regs.xpsr = (cpu.regs.xpsr & !0x1FF) | 3;
        assert_eq!(cpu.regs.ipsr(), 3);

        // Attempt to re-enter HardFault — lockup path.
        let cycles = cpu.enter_exception(3, &mut bus);
        assert_eq!(cycles, 0, "lockup path returns 0 cycles (no frame push)");
        assert!(cpu.is_halted(), "HardFault-in-HardFault must halt the core");
    }

    // -----------------------------------------------------------------
    // TT (Test Target) — MPU contribution (Phase 7 Stage E)
    //
    // These tests exercise the MPU loop inside `execute_tt` directly.
    // Result bit convention (subset exercised here):
    //   [16] MRVALID — MPU region matched
    //   [18] R       — readable from current security state
    //   [19] RW      — writable from current security state
    //   [7:0] MREGION — matched region number
    // -----------------------------------------------------------------

    /// Helper: enable MPU and program a single region with given base,
    /// limit, and AP[2:1] value in RBAR bits [2:1]. AP=0 or 1 → RW; AP=2
    /// → RO. The EN bit in RLAR[0] controls region enable.
    ///
    /// Also enables SAU with one catch-all Secure region covering the
    /// full 32-bit address space. This is necessary because when SAU is
    /// disabled, `execute_tt`'s SAU-off path unconditionally returns
    /// universal R+RW (independent of MPU state), which would mask the
    /// MPU's AP restrictions we're trying to exercise.
    ///
    /// Phase 0b.1 Commit B: per-core MPU/SAU state lives on
    /// `CortexM33.ppb` rather than `Bus.ppb`; helper now targets a CPU.
    fn program_mpu_region(
        cpu: &mut CortexM33,
        idx: usize,
        base: u32,
        limit: u32,
        ap: u32,
        enable: bool,
    ) {
        // MPU on.
        cpu.ppb.mpu_ctrl |= 1;
        let rbar = (base & !0x1F) | ((ap & 0x3) << 1);
        let rlar = (limit & !0x1F) | if enable { 1 } else { 0 };
        cpu.ppb.mpu_regions[idx] = (rbar, rlar);

        // SAU on with a catch-all Secure region — required to avoid the
        // SAU-disabled universal-RW fallback. Region 0 covers everything.
        cpu.ppb.sau_ctrl |= 1; // SAU enable
        cpu.ppb.sau_regions[0] = (0x0000_0000, 0xFFFF_FFE1); // full range, NSC=0, EN=1
    }

    /// Region EN=0: TT must treat the region as absent — no MRVALID,
    /// no R, no RW from the MPU side.
    #[test]
    fn test_tt_mpu_disabled_region() {
        let mut cpu = CortexM33::for_test(0);
        // Program region 0 covering 0x2000_0000..0x2000_0FFF, EN=0.
        program_mpu_region(&mut cpu, 0, 0x2000_0000, 0x2000_0FFF, 0, false);

        let r = cpu.execute_tt(0x2000_0400);
        assert_eq!(r & (1 << 16), 0, "MRVALID must be 0 for disabled region");
        // Note: with MPU enabled but no match, execute_tt falls back to
        // universal R/RW — that's independent of the disabled-region
        // behavior. The critical contract here is MRVALID stays clear.
    }

    /// AP[2:1] = 00 (RW for privileged): TT must return R=1 and RW=1.
    #[test]
    fn test_tt_mpu_rw_access() {
        let mut cpu = CortexM33::for_test(0);
        // Region 2, AP=0 (RW), enabled.
        program_mpu_region(&mut cpu, 2, 0x2000_0000, 0x2000_0FFF, 0, true);

        let r = cpu.execute_tt(0x2000_0080);
        assert_ne!(r & (1 << 16), 0, "MRVALID must be set when region matches");
        assert_ne!(r & (1 << 18), 0, "R must be set for RW region");
        assert_ne!(r & (1 << 19), 0, "RW must be set for AP=00 (read-write)");
        assert_eq!(r & 0xFF, 2, "MREGION must be the matching region index");
    }

    /// AP[2:1] = 10 (RO): TT must return R=1 but RW=0.
    #[test]
    fn test_tt_mpu_ro_access() {
        let mut cpu = CortexM33::for_test(0);
        // Region 5, AP=2 (RO), enabled.
        program_mpu_region(&mut cpu, 5, 0x2000_1000, 0x2000_1FFF, 2, true);

        let r = cpu.execute_tt(0x2000_1500);
        assert_ne!(r & (1 << 16), 0, "MRVALID must be set for RO region");
        assert_ne!(r & (1 << 18), 0, "R must be set for RO region");
        assert_eq!(r & (1 << 19), 0, "RW must be clear for AP=10 (read-only)");
        assert_eq!(r & 0xFF, 5, "MREGION must be 5");
    }

    /// Two enabled regions both covering the target address: the first
    /// match (lowest index) must win. This pins the iteration order so
    /// it isn't silently reordered to something firmware doesn't expect
    /// — the bootrom's MPU self-test assumes first-match semantics.
    #[test]
    fn test_tt_mpu_overlapping_regions_first_match() {
        let mut cpu = CortexM33::for_test(0);
        // Region 1: AP=0 (RW), covers 0x2000_0000..0x2000_1FFF.
        program_mpu_region(&mut cpu, 1, 0x2000_0000, 0x2000_1FFF, 0, true);
        // Region 7: AP=2 (RO), covers 0x2000_0000..0x2000_0FFF — overlaps
        // the low half of region 1. First-match-wins means region 1 is
        // returned, with RW still set.
        program_mpu_region(&mut cpu, 7, 0x2000_0000, 0x2000_0FFF, 2, true);

        let r = cpu.execute_tt(0x2000_0400);
        assert_eq!(
            r & 0xFF,
            1,
            "first-match must win (region 1 programmed first)"
        );
        assert_ne!(r & (1 << 19), 0, "RW must reflect region 1's AP=00, not region 7's AP=10");
    }
}

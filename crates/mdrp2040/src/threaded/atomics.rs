//! `CoreAtomics` — cross-core, per-core atomic state shared between the
//! two CPU threads + coordinator on the RP2040 threaded path.
//!
//! Stage 3b.2 (dual-execution HLD V1 §6.4): mirrors the RP2350 shape but
//! drops M0+-irrelevant fields:
//! - No RCP (no coprocessor at all on M0+).
//! - No FPU-related state (M0+ has no FPU / FPCCR / FPCAR).
//!
//! RP2040 has 26 peripheral IRQs (`irq::IRQ_COUNT`) so the pending mask
//! fits in a `u32` — the RP2350 version uses `AtomicU64` for its 64-line
//! NVIC.
//!
//! Orderings follow the RP2350 LLD:
//!   * `sev_both` stores with `Release`, `event_flag_consume` swaps with
//!     `AcqRel` — the pair establishes the SEV-caller-side happens-before
//!     the WFE-waker-side.
//!   * `take_irq_pending` swaps the mask to zero with `AcqRel`; the
//!     non-zero return replaces a "pending dirty" flag.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// Cross-core, per-core atomics owned by `Arc<CoreAtomics>` inside
/// `SharedState`. See module docs for ordering rationale.
#[derive(Debug)]
pub struct CoreAtomics {
    /// Core is halted — will not execute until explicitly woken.
    pub halted: [AtomicBool; 2],
    /// Core is sleeping on WFE — will resume when event_flag is set.
    pub wfe_waiting: [AtomicBool; 2],
    /// Per-core event flag for WFE/SEV protocol.
    pub event_flag: [AtomicBool; 2],
    /// Per-core external-IRQ pending mask. Bit N = IRQ N latched on that
    /// core's NVIC. Consumed by the step path via
    /// [`Self::take_irq_pending`] (`swap(0, AcqRel)`).
    pub irq_pending: [AtomicU32; 2],
    /// Per-core precise bus-fault flag. Observed by `CortexM0Plus::step`
    /// after a data-side access; escalated to HardFault (M0+ has no
    /// configurable BusFault handler).
    pub bus_fault: [AtomicBool; 2],
    /// Per-core address that triggered the most recent bus fault.
    pub bus_fault_addr: [AtomicU32; 2],
}

impl Default for CoreAtomics {
    fn default() -> Self {
        Self {
            halted: [AtomicBool::new(false), AtomicBool::new(false)],
            wfe_waiting: [AtomicBool::new(false), AtomicBool::new(false)],
            event_flag: [AtomicBool::new(false), AtomicBool::new(false)],
            irq_pending: [AtomicU32::new(0), AtomicU32::new(0)],
            bus_fault: [AtomicBool::new(false), AtomicBool::new(false)],
            bus_fault_addr: [AtomicU32::new(0), AtomicU32::new(0)],
        }
    }
}

impl CoreAtomics {
    // --- IRQ pending ---

    /// Assert an IRQ on one core's pending mask. Idempotent.
    #[inline]
    pub fn assert_irq(&self, core: usize, irq: u32) {
        if core < 2 && irq < 32 {
            self.irq_pending[core].fetch_or(1u32 << irq, Ordering::Release);
        }
    }

    /// Assert an IRQ on every core (shared peripheral line).
    #[inline]
    pub fn assert_irq_shared(&self, irq: u32) {
        if irq < 32 {
            let bit = 1u32 << irq;
            self.irq_pending[0].fetch_or(bit, Ordering::Release);
            self.irq_pending[1].fetch_or(bit, Ordering::Release);
        }
    }

    /// Clear one core's pending bit.
    #[inline]
    pub fn clear_irq(&self, core: usize, irq: u32) {
        if core < 2 && irq < 32 {
            self.irq_pending[core].fetch_and(!(1u32 << irq), Ordering::Release);
        }
    }

    /// Non-consuming peek at the pending mask.
    #[inline]
    pub fn irq_pending_load(&self, core: usize) -> u32 {
        self.irq_pending[core].load(Ordering::Acquire)
    }

    /// Swap-to-zero consume of the pending mask with a load-first fast
    /// path: the steady-state case is "no IRQ pending", a plain Acquire
    /// load costs ~1 clock, only the non-zero case pays the LOCK XCHG.
    #[inline]
    pub fn take_irq_pending(&self, core: usize) -> u32 {
        if self.irq_pending[core].load(Ordering::Acquire) == 0 {
            return 0;
        }
        self.irq_pending[core].swap(0, Ordering::AcqRel)
    }

    /// Set the full IRQ pending mask for `core` (used by NVIC sync paths).
    #[inline]
    pub fn set_irq_pending(&self, core: usize, mask: u32) {
        self.irq_pending[core].store(mask, Ordering::Release);
    }

    // --- WFE/SEV ---

    /// SEV writes both cores' event_flag with Release so pre-SEV stores
    /// are visible after a peer's `event_flag_consume` returns true.
    #[inline]
    pub fn sev_both(&self) {
        self.event_flag[0].store(true, Ordering::Release);
        self.event_flag[1].store(true, Ordering::Release);
    }

    /// Consume one core's event_flag (swap-to-false, AcqRel).
    #[inline]
    pub fn event_flag_consume(&self, core: usize) -> bool {
        self.event_flag[core].swap(false, Ordering::AcqRel)
    }

    /// Direct set (used by FIFO push on the receiver side).
    #[inline]
    pub fn set_event_flag(&self, core: usize) {
        self.event_flag[core].store(true, Ordering::Release);
    }

    /// Non-consuming peek (used by WFE wake check before swap).
    #[inline]
    pub fn event_flag_load(&self, core: usize) -> bool {
        self.event_flag[core].load(Ordering::Acquire)
    }

    // --- Halt / WFE state ---

    #[inline]
    pub fn set_halted(&self, core: usize) {
        self.halted[core].store(true, Ordering::Release);
    }

    #[inline]
    pub fn clear_halted(&self, core: usize) {
        self.halted[core].store(false, Ordering::Release);
    }

    #[inline]
    pub fn is_halted(&self, core: usize) -> bool {
        self.halted[core].load(Ordering::Acquire)
    }

    #[inline]
    pub fn set_wfe_waiting(&self, core: usize) {
        self.wfe_waiting[core].store(true, Ordering::Release);
    }

    #[inline]
    pub fn clear_wfe_waiting(&self, core: usize) {
        self.wfe_waiting[core].store(false, Ordering::Release);
    }

    #[inline]
    pub fn is_wfe_waiting(&self, core: usize) -> bool {
        self.wfe_waiting[core].load(Ordering::Acquire)
    }

    // --- Bus fault ---

    #[inline]
    pub fn set_bus_fault(&self, core: usize, addr: u32) {
        self.bus_fault_addr[core].store(addr, Ordering::Release);
        self.bus_fault[core].store(true, Ordering::Release);
    }

    #[inline]
    pub fn clear_bus_fault(&self, core: usize) {
        self.bus_fault[core].store(false, Ordering::Release);
    }

    #[inline]
    pub fn is_bus_fault(&self, core: usize) -> bool {
        self.bus_fault[core].load(Ordering::Acquire)
    }

    #[inline]
    pub fn bus_fault_addr(&self, core: usize) -> u32 {
        self.bus_fault_addr[core].load(Ordering::Acquire)
    }

    // --- Bulk / setup helpers ---

    /// Reset all per-core state. Coordinator-phase only — not safe while
    /// workers are executing.
    pub fn reset(&self) {
        for c in 0..2 {
            self.halted[c].store(false, Ordering::Release);
            self.wfe_waiting[c].store(false, Ordering::Release);
            self.event_flag[c].store(false, Ordering::Release);
            self.irq_pending[c].store(0, Ordering::Release);
            self.bus_fault[c].store(false, Ordering::Release);
            self.bus_fault_addr[c].store(0, Ordering::Release);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_all_zero() {
        let a = CoreAtomics::default();
        for c in 0..2 {
            assert!(!a.is_halted(c));
            assert!(!a.is_wfe_waiting(c));
            assert!(!a.event_flag_load(c));
            assert_eq!(a.irq_pending_load(c), 0);
            assert!(!a.is_bus_fault(c));
            assert_eq!(a.bus_fault_addr(c), 0);
        }
    }

    #[test]
    fn sev_sets_both_event_flags() {
        let a = CoreAtomics::default();
        a.sev_both();
        assert!(a.event_flag_load(0));
        assert!(a.event_flag_load(1));
    }

    #[test]
    fn event_flag_consume_returns_prior_state() {
        let a = CoreAtomics::default();
        a.set_event_flag(0);
        assert!(a.event_flag_consume(0));
        assert!(!a.event_flag_consume(0));
    }

    #[test]
    fn assert_irq_sets_bit_per_core() {
        let a = CoreAtomics::default();
        a.assert_irq(0, 5);
        assert_eq!(a.irq_pending_load(0), 1u32 << 5);
        assert_eq!(a.irq_pending_load(1), 0);
    }

    #[test]
    fn assert_irq_shared_lands_on_both() {
        let a = CoreAtomics::default();
        a.assert_irq_shared(7);
        assert_eq!(a.irq_pending_load(0), 1u32 << 7);
        assert_eq!(a.irq_pending_load(1), 1u32 << 7);
    }

    #[test]
    fn take_irq_pending_swap_returns_prior() {
        let a = CoreAtomics::default();
        a.assert_irq(0, 5);
        let first = a.take_irq_pending(0);
        assert_ne!(first & (1u32 << 5), 0);
        let second = a.take_irq_pending(0);
        assert_eq!(second, 0);
    }

    #[test]
    fn take_irq_pending_zero_fast_path() {
        let a = CoreAtomics::default();
        assert_eq!(a.take_irq_pending(0), 0);
        assert_eq!(a.take_irq_pending(1), 0);
    }

    #[test]
    fn assert_irq_out_of_range_is_noop() {
        let a = CoreAtomics::default();
        a.assert_irq(2, 5); // bad core
        a.assert_irq(0, 32); // bad irq
        assert_eq!(a.irq_pending_load(0), 0);
        assert_eq!(a.irq_pending_load(1), 0);
    }

    #[test]
    fn clear_irq_clears_bit() {
        let a = CoreAtomics::default();
        a.assert_irq(0, 7);
        a.assert_irq(0, 3);
        a.clear_irq(0, 7);
        assert_eq!(a.irq_pending_load(0), 1u32 << 3);
    }

    #[test]
    fn bus_fault_is_per_core() {
        let a = CoreAtomics::default();
        a.set_bus_fault(0, 0xDEAD_BEEF);
        assert!(a.is_bus_fault(0));
        assert!(!a.is_bus_fault(1));
        assert_eq!(a.bus_fault_addr(0), 0xDEAD_BEEF);
        a.clear_bus_fault(0);
        assert!(!a.is_bus_fault(0));
    }

    #[test]
    fn reset_clears_all_fields() {
        let a = CoreAtomics::default();
        a.set_halted(0);
        a.set_wfe_waiting(1);
        a.set_event_flag(0);
        a.assert_irq(1, 3);
        a.set_bus_fault(0, 0xCAFE_F00D);

        a.reset();

        for c in 0..2 {
            assert!(!a.is_halted(c));
            assert!(!a.is_wfe_waiting(c));
            assert!(!a.event_flag_load(c));
            assert_eq!(a.irq_pending_load(c), 0);
            assert!(!a.is_bus_fault(c));
            assert_eq!(a.bus_fault_addr(c), 0);
        }
    }
}

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
    /// FIFO_ST.WOF — write-on-full sticky, one per sender core. Set when
    /// this core pushes to `sio_fifo_*` and the queue is full. Cleared
    /// by writing 1 to FIFO_ST bit 2 (W1C).
    pub fifo_wof: [AtomicBool; 2],
    /// FIFO_ST.ROE — read-on-empty sticky, one per reader core. Set
    /// when this core pops from the incoming `sio_fifo_*` while empty.
    /// Cleared by writing 1 to FIFO_ST bit 3 (W1C).
    pub fifo_roe: [AtomicBool; 2],
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
            fifo_wof: [AtomicBool::new(false), AtomicBool::new(false)],
            fifo_roe: [AtomicBool::new(false), AtomicBool::new(false)],
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

    // --- FIFO sticky flags (FIFO_ST WOF/ROE) ---

    /// Latch the FIFO_ST.WOF (write-on-full) sticky for `core`. Set when
    /// `core` pushed to a full SIO FIFO.
    #[inline]
    pub fn set_fifo_wof(&self, core: usize) {
        if core < 2 {
            self.fifo_wof[core].store(true, Ordering::Release);
        }
    }

    /// Latch the FIFO_ST.ROE (read-on-empty) sticky for `core`. Set when
    /// `core` popped from an empty SIO FIFO.
    #[inline]
    pub fn set_fifo_roe(&self, core: usize) {
        if core < 2 {
            self.fifo_roe[core].store(true, Ordering::Release);
        }
    }

    /// Read the FIFO_ST.WOF sticky for `core`.
    #[inline]
    pub fn fifo_wof(&self, core: usize) -> bool {
        if core < 2 {
            self.fifo_wof[core].load(Ordering::Acquire)
        } else {
            false
        }
    }

    /// Read the FIFO_ST.ROE sticky for `core`.
    #[inline]
    pub fn fifo_roe(&self, core: usize) -> bool {
        if core < 2 {
            self.fifo_roe[core].load(Ordering::Acquire)
        } else {
            false
        }
    }

    /// Clear the FIFO_ST.WOF sticky for `core` (W1C from FIFO_ST write).
    #[inline]
    pub fn clear_fifo_wof(&self, core: usize) {
        if core < 2 {
            self.fifo_wof[core].store(false, Ordering::Release);
        }
    }

    /// Clear the FIFO_ST.ROE sticky for `core` (W1C from FIFO_ST write).
    #[inline]
    pub fn clear_fifo_roe(&self, core: usize) {
        if core < 2 {
            self.fifo_roe[core].store(false, Ordering::Release);
        }
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
            self.fifo_wof[c].store(false, Ordering::Release);
            self.fifo_roe[c].store(false, Ordering::Release);
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

    // --- Out-of-range guards on per-core helpers ---------------------------

    #[test]
    fn assert_irq_bad_core_only() {
        // core >= 2 short-circuits — irq value is irrelevant.
        let a = CoreAtomics::default();
        a.assert_irq(2, 0);
        a.assert_irq(99, 5);
        assert_eq!(a.irq_pending_load(0), 0);
        assert_eq!(a.irq_pending_load(1), 0);
    }

    #[test]
    fn assert_irq_bad_irq_only() {
        // core valid but irq >= 32 — drops without panicking on shift.
        let a = CoreAtomics::default();
        a.assert_irq(0, 32);
        a.assert_irq(1, 100);
        assert_eq!(a.irq_pending_load(0), 0);
        assert_eq!(a.irq_pending_load(1), 0);
    }

    #[test]
    fn assert_irq_shared_out_of_range_is_noop() {
        // The shared variant guards `irq < 32` separately; neither core
        // should see a bit set.
        let a = CoreAtomics::default();
        a.assert_irq_shared(32);
        a.assert_irq_shared(64);
        assert_eq!(a.irq_pending_load(0), 0);
        assert_eq!(a.irq_pending_load(1), 0);
    }

    #[test]
    fn clear_irq_out_of_range_is_noop() {
        let a = CoreAtomics::default();
        a.assert_irq(0, 7);
        a.assert_irq(1, 7);
        // Bad core — should not affect either pending mask.
        a.clear_irq(2, 7);
        a.clear_irq(99, 7);
        assert_eq!(a.irq_pending_load(0), 1u32 << 7);
        assert_eq!(a.irq_pending_load(1), 1u32 << 7);
        // Bad irq — shift would otherwise overflow / wrap; guard drops it.
        a.clear_irq(0, 32);
        a.clear_irq(1, 200);
        assert_eq!(a.irq_pending_load(0), 1u32 << 7);
        assert_eq!(a.irq_pending_load(1), 1u32 << 7);
    }

    #[test]
    fn set_irq_pending_overwrites_mask() {
        let a = CoreAtomics::default();
        a.assert_irq(0, 1);
        a.set_irq_pending(0, 0xDEAD_BEEF);
        assert_eq!(a.irq_pending_load(0), 0xDEAD_BEEF);
        // Untouched core stays clean.
        assert_eq!(a.irq_pending_load(1), 0);
    }

    // --- FIFO sticky helpers (WOF / ROE) ----------------------------------

    #[test]
    fn fifo_wof_set_clear_round_trip_per_core() {
        let a = CoreAtomics::default();
        assert!(!a.fifo_wof(0));
        assert!(!a.fifo_wof(1));
        a.set_fifo_wof(0);
        assert!(a.fifo_wof(0));
        assert!(!a.fifo_wof(1));
        a.set_fifo_wof(1);
        assert!(a.fifo_wof(0));
        assert!(a.fifo_wof(1));
        a.clear_fifo_wof(0);
        assert!(!a.fifo_wof(0));
        assert!(a.fifo_wof(1));
        a.clear_fifo_wof(1);
        assert!(!a.fifo_wof(1));
    }

    #[test]
    fn fifo_roe_set_clear_round_trip_per_core() {
        let a = CoreAtomics::default();
        assert!(!a.fifo_roe(0));
        assert!(!a.fifo_roe(1));
        a.set_fifo_roe(0);
        assert!(a.fifo_roe(0));
        assert!(!a.fifo_roe(1));
        a.set_fifo_roe(1);
        assert!(a.fifo_roe(0));
        assert!(a.fifo_roe(1));
        a.clear_fifo_roe(0);
        assert!(!a.fifo_roe(0));
        assert!(a.fifo_roe(1));
        a.clear_fifo_roe(1);
        assert!(!a.fifo_roe(1));
    }

    #[test]
    fn fifo_sticky_helpers_ignore_out_of_range_core() {
        // All four set/clear paths take the `if core < 2` false branch
        // here; reads return false on bad core via the else arm.
        let a = CoreAtomics::default();
        a.set_fifo_wof(2);
        a.set_fifo_wof(99);
        a.set_fifo_roe(2);
        a.set_fifo_roe(99);
        // Real cores are still clean.
        assert!(!a.fifo_wof(0));
        assert!(!a.fifo_wof(1));
        assert!(!a.fifo_roe(0));
        assert!(!a.fifo_roe(1));
        // Reads on bad core return false (else arm).
        assert!(!a.fifo_wof(2));
        assert!(!a.fifo_wof(99));
        assert!(!a.fifo_roe(2));
        assert!(!a.fifo_roe(99));
        // Set legitimate flags then verify clear-on-bad-core leaves them alone.
        a.set_fifo_wof(0);
        a.set_fifo_roe(1);
        a.clear_fifo_wof(2);
        a.clear_fifo_wof(99);
        a.clear_fifo_roe(2);
        a.clear_fifo_roe(99);
        assert!(a.fifo_wof(0));
        assert!(a.fifo_roe(1));
    }

    // --- WFE / halt symmetry ----------------------------------------------

    #[test]
    fn halt_and_wfe_state_round_trip() {
        let a = CoreAtomics::default();
        a.set_halted(0);
        a.set_wfe_waiting(1);
        assert!(a.is_halted(0));
        assert!(!a.is_halted(1));
        assert!(!a.is_wfe_waiting(0));
        assert!(a.is_wfe_waiting(1));
        a.clear_halted(0);
        a.clear_wfe_waiting(1);
        assert!(!a.is_halted(0));
        assert!(!a.is_wfe_waiting(1));
    }

    #[test]
    fn take_irq_pending_returns_only_set_bits() {
        // Multi-bit mask path: confirms swap returns the full mask, not
        // just the most recently OR-ed bit.
        let a = CoreAtomics::default();
        a.assert_irq(1, 2);
        a.assert_irq(1, 9);
        a.assert_irq(1, 17);
        let mask = a.take_irq_pending(1);
        assert_eq!(mask, (1u32 << 2) | (1u32 << 9) | (1u32 << 17));
        assert_eq!(a.take_irq_pending(1), 0);
    }
}

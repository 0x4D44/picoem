//! `CoreAtomics` — cross-core, per-core atomic state shared between the
//! two CPU threads + coordinator.
//!
//! Phase 3 Stage 1 (LLD V7 §2): this owns every per-core field that used
//! to live directly on `Bus` or `CortexM33` and that will, in the threaded
//! runtime, be poked from more than one host thread. It is constructed
//! once at build time, wrapped in an `Arc`, and handed to the `Bus` + both
//! cores so the single-threaded call sites continue to function while the
//! threaded path can clone the `Arc` into worker closures.
//!
//! Orderings follow the LLD:
//!   * `sev_both` stores with `Release`, `event_flag_consume` swaps with
//!     `AcqRel` — the pair establishes the SEV-caller-side happens-before
//!     the WFE-waker-side.
//!   * `take_irq_pending` swaps the mask to zero with `AcqRel` — the
//!     non-zero return replaces V5's `irq_pending_dirty` flag.
//!   * `rcp_count_check` is a CAS loop guarding the shared counter;
//!     readers use `Acquire`, the CAS uses `AcqRel` on success and
//!     `Acquire` on failure.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

/// Cross-core, per-core atomics owned by `Arc<CoreAtomics>` on both the
/// `Bus` and each `CortexM33`. See module docs for ordering rationale.
#[derive(Debug)]
pub struct CoreAtomics {
    /// Core is halted — will not execute until explicitly woken.
    pub halted: [AtomicBool; 2],
    /// Core is sleeping on WFE — will resume when event_flag is set.
    pub wfe_waiting: [AtomicBool; 2],
    /// Per-core event flag for WFE/SEV protocol.
    pub event_flag: [AtomicBool; 2],
    /// Per-core external-IRQ pending mask. Bit N set means IRQ N is
    /// latched on that core's NVIC. The step-path consumes this with
    /// [`Self::take_irq_pending`] (`swap(0, AcqRel)`); the non-zero
    /// return replaces the old `irq_pending_dirty` flag.
    pub irq_pending: [AtomicU64; 2],
    /// Per-core RCP salt value (CP7 coprocessor, Phase 7 Stage E).
    pub rcp_salt: [AtomicU32; 2],
    /// Per-core RCP salt validity flag.
    pub rcp_salt_valid: [AtomicBool; 2],
    /// RCP redundancy counter — single shared counter across both cores.
    /// `rcp_count_set` initialises; [`Self::rcp_count_check`] asserts
    /// counter == expected then increments (CAS loop).
    pub rcp_count: AtomicU32,
    /// Per-core precise bus-fault flag. Observed by `CortexM33::step`
    /// after a data-side access; escalated to BusFault/HardFault.
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
            irq_pending: [AtomicU64::new(0), AtomicU64::new(0)],
            rcp_salt: [AtomicU32::new(0), AtomicU32::new(0)],
            rcp_salt_valid: [AtomicBool::new(false), AtomicBool::new(false)],
            rcp_count: AtomicU32::new(0),
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
        if core < 2 && irq < 64 {
            self.irq_pending[core].fetch_or(1u64 << irq, Ordering::Release);
        }
    }

    /// Assert an IRQ on every core (shared peripheral line).
    #[inline]
    pub fn assert_irq_shared(&self, irq: u32) {
        if irq < 64 {
            let bit = 1u64 << irq;
            self.irq_pending[0].fetch_or(bit, Ordering::Release);
            self.irq_pending[1].fetch_or(bit, Ordering::Release);
        }
    }

    /// Clear one core's pending bit.
    #[inline]
    pub fn clear_irq(&self, core: usize, irq: u32) {
        if core < 2 && irq < 64 {
            self.irq_pending[core].fetch_and(!(1u64 << irq), Ordering::Release);
        }
    }

    /// Non-consuming peek at the pending mask.
    #[inline]
    pub fn irq_pending_load(&self, core: usize) -> u64 {
        self.irq_pending[core].load(Ordering::Acquire)
    }

    /// Swap-to-zero consume of the pending mask — non-zero return
    /// replaces the V5 `irq_pending_dirty` flag.
    ///
    /// Load-first fast path: called every CPU emu-step, and the
    /// steady-state case is "no IRQ pending". A plain Acquire load on
    /// the shared cache line costs ~1 clock; only when the mask is
    /// non-zero do we pay the `LOCK XCHG` to atomically clear it.
    /// Skipping the RMW in the zero case avoids the per-instruction
    /// cache-line invalidation between CPU workers.
    ///
    /// Race behaviour: if a writer (coord's `assert_irq_shared` or a
    /// peer `sev`) stores non-zero between our load and a
    /// subsequent step's observation, the new bits are picked up on
    /// the next step — at most one emu-step of IRQ latency jitter,
    /// within the threaded-mode tolerance from HLD V7 §5.2.
    #[inline]
    pub fn take_irq_pending(&self, core: usize) -> u64 {
        if self.irq_pending[core].load(Ordering::Acquire) == 0 {
            return 0;
        }
        self.irq_pending[core].swap(0, Ordering::AcqRel)
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
    /// Pairs with [`Self::sev_both`]'s Release.
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

    // --- RCP ---

    #[inline]
    pub fn rcp_count_set(&self, value: u32) {
        self.rcp_count.store(value, Ordering::Release);
    }

    /// CAS-guarded check-and-increment. Returns `Ok(())` if the counter
    /// matched `expected` and was bumped to `expected+1`; returns
    /// `Err(actual)` if a mismatch was observed at any point. Retries
    /// only on spurious CAS failure where the reloaded value still
    /// matches the expectation.
    #[inline]
    pub fn rcp_count_check(&self, expected: u32) -> Result<(), u32> {
        loop {
            let cur = self.rcp_count.load(Ordering::Acquire);
            if cur != expected {
                return Err(cur);
            }
            match self.rcp_count.compare_exchange(
                cur,
                cur.wrapping_add(1),
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => return Ok(()),
                Err(_) => continue,
            }
        }
    }

    #[inline]
    pub fn rcp_count_load(&self) -> u32 {
        self.rcp_count.load(Ordering::Acquire)
    }

    #[inline]
    pub fn rcp_salt_set(&self, core: usize, value: u32) {
        self.rcp_salt[core].store(value, Ordering::Release);
        self.rcp_salt_valid[core].store(true, Ordering::Release);
    }

    #[inline]
    pub fn rcp_salt_load(&self, core: usize) -> u32 {
        self.rcp_salt[core].load(Ordering::Acquire)
    }

    #[inline]
    pub fn rcp_salt_is_valid(&self, core: usize) -> bool {
        self.rcp_salt_valid[core].load(Ordering::Acquire)
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

    /// Reset all per-core state (used during emulator reset). Coordinator-phase
    /// only; not safe to call while workers are executing.
    pub fn reset(&self) {
        for c in 0..2 {
            self.halted[c].store(false, Ordering::Release);
            self.wfe_waiting[c].store(false, Ordering::Release);
            self.event_flag[c].store(false, Ordering::Release);
            self.irq_pending[c].store(0, Ordering::Release);
            self.rcp_salt[c].store(0, Ordering::Release);
            self.rcp_salt_valid[c].store(false, Ordering::Release);
            self.bus_fault[c].store(false, Ordering::Release);
            self.bus_fault_addr[c].store(0, Ordering::Release);
        }
        self.rcp_count.store(0, Ordering::Release);
    }

    /// Set the full IRQ pending mask for `core` (used by NVIC sync paths).
    #[inline]
    pub fn set_irq_pending(&self, core: usize, mask: u64) {
        self.irq_pending[core].store(mask, Ordering::Release);
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
            assert!(!a.rcp_salt_is_valid(c));
            assert_eq!(a.rcp_salt_load(c), 0);
            assert!(!a.is_bus_fault(c));
            assert_eq!(a.bus_fault_addr(c), 0);
        }
        assert_eq!(a.rcp_count_load(), 0);
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
    fn assert_irq_sets_bit() {
        let a = CoreAtomics::default();
        a.assert_irq(0, 5);
        assert_eq!(a.irq_pending_load(0), 1u64 << 5);
        assert_eq!(a.irq_pending_load(1), 0);
    }

    #[test]
    fn assert_irq_shared_lands_on_both() {
        let a = CoreAtomics::default();
        a.assert_irq_shared(7);
        assert_eq!(a.irq_pending_load(0), 1u64 << 7);
        assert_eq!(a.irq_pending_load(1), 1u64 << 7);
    }

    #[test]
    fn core_atomics_swap_zero_returns_prior_pending() {
        let a = CoreAtomics::default();
        a.assert_irq(0, 5);
        let first = a.take_irq_pending(0);
        assert_ne!(first & (1u64 << 5), 0,
            "first take_irq_pending must return the mask with bit 5 set");
        let second = a.take_irq_pending(0);
        assert_eq!(second, 0,
            "second take_irq_pending must observe the zeroed mask");
    }

    #[test]
    fn rcp_count_check_success_increments() {
        let a = CoreAtomics::default();
        a.rcp_count_set(0xC0);
        assert_eq!(a.rcp_count_check(0xC0), Ok(()));
        assert_eq!(a.rcp_count_load(), 0xC1);
    }

    #[test]
    fn rcp_count_check_mismatch_returns_err() {
        let a = CoreAtomics::default();
        a.rcp_count_set(0xC0);
        assert_eq!(a.rcp_count_check(0xC5), Err(0xC0));
        assert_eq!(a.rcp_count_load(), 0xC0, "count unchanged on mismatch");
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
    fn core_atomics_reset_clears_all() {
        let a = CoreAtomics::default();
        // Seed every field with a non-default value.
        a.set_halted(0);
        a.set_halted(1);
        a.set_wfe_waiting(0);
        a.set_wfe_waiting(1);
        a.set_event_flag(0);
        a.set_event_flag(1);
        a.assert_irq(0, 3);
        a.assert_irq(1, 5);
        a.rcp_salt_set(0, 0xAAAA_1111);
        a.rcp_salt_set(1, 0x5555_BBBB);
        a.rcp_count_set(0x1234_5678);
        a.set_bus_fault(0, 0xCAFE_F00D);
        a.set_bus_fault(1, 0xDEAD_BEEF);

        a.reset();

        for c in 0..2 {
            assert!(!a.is_halted(c), "halted[{}] not cleared", c);
            assert!(!a.is_wfe_waiting(c), "wfe_waiting[{}] not cleared", c);
            assert!(!a.event_flag_load(c), "event_flag[{}] not cleared", c);
            assert_eq!(a.irq_pending_load(c), 0, "irq_pending[{}] not cleared", c);
            assert_eq!(a.rcp_salt_load(c), 0, "rcp_salt[{}] not cleared", c);
            assert!(!a.rcp_salt_is_valid(c), "rcp_salt_valid[{}] not cleared", c);
            assert!(!a.is_bus_fault(c), "bus_fault[{}] not cleared", c);
            assert_eq!(a.bus_fault_addr(c), 0, "bus_fault_addr[{}] not cleared", c);
        }
        assert_eq!(a.rcp_count_load(), 0, "rcp_count not cleared");
    }
}

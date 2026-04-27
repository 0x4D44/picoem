//! `SharedState` — the Arc bundle cloned into every worker in the
//! threaded runtime.
//!
//! Phase 3 Stage 4 (LLD V7 §3): scaffolding only. Stage 5 (`WorkerBus`)
//! consumes it; Stage 6's `ThreadedEmulator::from_emulator` constructs
//! it from the existing single-threaded `Emulator` / `Bus`.
//!
//! All inner state lives behind `Arc` so `SharedState: Clone` is a
//! cheap refcount bump. Cloning is the intended way to hand a view of
//! the shared state to each worker closure.
//!
//! The lock-free `master_cycle: Arc<AtomicU64>` lets the coordinator
//! publish cycle advancement without contending with the CPU workers'
//! PLL CS reads — see `bus/peripherals.rs` and LLD V7 §3/§4.

use std::sync::Arc;
use std::sync::atomic::AtomicU64;

use crate::threaded::{
    atomics::CoreAtomics, gpio::AtomicGpio, memory::SharedMemory, monitors::ExclusiveMonitors,
    peripherals::Peripherals, pio::ThreadedPio, sio::ThreadedSio,
};

/// Arc-bundled shared state handed to every worker in the threaded
/// runtime. Cloning is cheap (refcount bumps on the inner Arcs +
/// the AtomicU64) and intentional — each worker thread owns its own
/// clone and references the inner state lock-free (or Mutex-guarded
/// for `peripherals`).
#[derive(Clone)]
pub struct SharedState {
    /// Shared SRAM / ROM / XIP memory (atomic word + plain byte slices).
    pub memory: Arc<SharedMemory>,
    /// GPIO output / output-enable state (two-bank u32 atomics).
    pub gpio: Arc<AtomicGpio>,
    /// Cross-core SIO surface: FIFOs, spinlocks, doorbells, mtime.
    pub sio: Arc<ThreadedSio>,
    /// PIO TX/RX FIFOs + command queue + control atomics.
    pub pio: Arc<ThreadedPio>,
    /// LDREX/STREX exclusive-monitor tracking.
    pub monitors: Arc<ExclusiveMonitors>,
    /// Mutex-guarded peripheral storage (CLOCKS / QMI / RESETS / APB /
    /// TIMERS / DMA / legacy HashMap). See `peripherals.rs`.
    pub peripherals: Arc<Peripherals>,
    /// Cross-core per-core atomics (halted / WFE / event_flag /
    /// irq_pending / RCP / bus_fault). Also referenced directly from
    /// each `CortexM33`.
    pub atomics: Arc<CoreAtomics>,
    /// Global master-cycle counter. Coordinator advances via
    /// `fetch_add(step_q, Release)` at the top of each phase; CPU
    /// workers read with `load(Acquire)` for PLL CS LOCK-bit derivation
    /// (see `bus/peripherals.rs` callers at :273/:283/:294/:305).
    ///
    /// Lock-free on the hot read path — the snapshot is taken *before*
    /// taking the `peripherals.clocks` mutex, so a concurrent
    /// coordinator advance does not serialize with CPU reads.
    pub master_cycle: Arc<AtomicU64>,
}

impl SharedState {
    /// Construct a fresh `SharedState` with every inner component in
    /// its default / post-bootrom state. Stage 6's `from_emulator`
    /// consumes existing Bus fields instead; this constructor exists
    /// for unit tests and any future standalone use.
    pub fn new_default() -> Self {
        Self {
            memory: Arc::new(SharedMemory::new()),
            gpio: Arc::new(AtomicGpio::new()),
            sio: Arc::new(ThreadedSio::new()),
            pio: Arc::new(ThreadedPio::new()),
            monitors: Arc::new(ExclusiveMonitors::new()),
            peripherals: Arc::new(Peripherals::new_default()),
            atomics: Arc::new(CoreAtomics::default()),
            master_cycle: Arc::new(AtomicU64::new(0)),
        }
    }
}

impl Default for SharedState {
    fn default() -> Self {
        Self::new_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    /// Compile-time proof that `SharedState` is `Send + Sync + Clone`.
    /// Any of these trait bounds failing would kill the coordinator →
    /// worker handoff pattern before runtime, so enforcing them here
    /// gives us a fast-fail on accidental non-Sync field additions.
    fn _assert_send_sync_clone<T: Send + Sync + Clone>() {}
    #[test]
    fn shared_state_send_sync_clone() {
        _assert_send_sync_clone::<SharedState>();
    }

    #[test]
    fn shared_state_clone_shares_arcs() {
        // Cloning must not deep-copy — both clones observe the same
        // underlying atomic counter, which is what makes the
        // coordinator/worker handoff coherent.
        let a = SharedState::new_default();
        let b = a.clone();
        a.master_cycle.store(42, Ordering::Release);
        assert_eq!(b.master_cycle.load(Ordering::Acquire), 42);
        assert!(Arc::ptr_eq(&a.master_cycle, &b.master_cycle));
        assert!(Arc::ptr_eq(&a.memory, &b.memory));
        assert!(Arc::ptr_eq(&a.peripherals, &b.peripherals));
        assert!(Arc::ptr_eq(&a.atomics, &b.atomics));
    }
}

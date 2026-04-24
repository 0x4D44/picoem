//! `SharedState` — the Arc-bundle cloned into each worker in the RP2040
//! threaded runtime.
//!
//! Stage 3b.2 (dual-execution HLD V1 §6.4): scaffolding only. Stage 3b.3
//! fills in the MMIO peripheral routing surface; Stage 3b.4 wires this
//! bundle into `ThreadedEmulator`.
//!
//! All inner state lives behind `Arc` so `SharedState: Clone` is a cheap
//! refcount bump — cloning is the intended way to hand a view of the
//! shared state to each worker closure.
//!
//! Differences vs mdrp2350's `SharedState`:
//! - No FPU / RCP / coprocessor state (M0+ has none).
//! - 32 spinlocks (RP2040) vs 32 spinlocks (RP2350 also has 32 — same).
//! - 30 GPIOs (RP2040) — still fits `AtomicU32`.
//! - No XIP flash — `SharedMemory` is ROM + SRAM only.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64};
use std::sync::{Arc, Mutex};

use mdpicoem_common::threaded::SpscQueue;

use crate::WorkerName;
use crate::threaded::{CoreAtomics, SharedMemory};

/// Arc-bundled shared state handed to every worker in the threaded
/// runtime. Cloning is cheap (refcount bumps + bundle copy).
#[derive(Clone)]
pub struct SharedState {
    /// Shared SRAM / ROM memory (atomic words + read-only byte slice).
    pub memory: Arc<SharedMemory>,
    /// Per-core atomics (halted / WFE / event_flag / irq_pending /
    /// bus_fault).
    pub atomics: [Arc<CoreAtomics>; 2],
    /// Inter-core FIFO: core 0 pushes, core 1 pops.
    pub sio_fifo_0_to_1: Arc<SpscQueue>,
    /// Inter-core FIFO: core 1 pushes, core 0 pops.
    pub sio_fifo_1_to_0: Arc<SpscQueue>,
    /// 32 hardware spinlocks (RP2040 datasheet §2.3.1.5). Each cell
    /// stores 0 (unlocked) or the acquire-token (non-zero).
    pub spinlocks: Arc<[AtomicU32; 32]>,
    /// GPIO output state (low 30 bits = GPIO0..29). Coordinator merges
    /// SIO + PIO into this each quantum so cores can observe the pin
    /// state without holding a mutex.
    pub gpio_out: Arc<AtomicU32>,
    /// GPIO output-enable state (low 30 bits). Same ownership pattern
    /// as `gpio_out`.
    pub gpio_oe: Arc<AtomicU32>,
    /// Monotonic master-cycle counter published by the coordinator and
    /// read lock-free by the CPU workers (PLL LOCK timing, TIMER match,
    /// etc.).
    pub master_cycle: Arc<AtomicU64>,
    /// Sticky panic flag. Set by the first worker to unwind; subsequent
    /// workers observe it and exit their quantum early.
    pub poisoned: Arc<AtomicBool>,
    /// Structured panic info for attribution. Mutex-guarded because it
    /// is written on the cold panic path, read once by the coordinator
    /// at join time.
    pub panic_info: Arc<Mutex<Option<(WorkerName, String)>>>,
}

impl SharedState {
    /// Construct a fresh `SharedState` with every inner component in
    /// its default / post-boot state. Stage 3b.4 may grow a
    /// `from_emulator` / `from_parts` sibling once the `Emulator` bus
    /// is refactored to hand its memory over.
    pub fn new_default() -> Self {
        // 32 AtomicU32 cells. Array init via `core::array::from_fn`
        // keeps the constructor boilerplate contained.
        let spinlocks_array: [AtomicU32; 32] =
            std::array::from_fn(|_| AtomicU32::new(0));

        Self {
            memory: Arc::new(SharedMemory::new_zero()),
            atomics: [
                Arc::new(CoreAtomics::default()),
                Arc::new(CoreAtomics::default()),
            ],
            // Capacity 8 matches mdrp2350 / RP2040 SIO FIFO depth.
            sio_fifo_0_to_1: Arc::new(SpscQueue::new(8)),
            sio_fifo_1_to_0: Arc::new(SpscQueue::new(8)),
            spinlocks: Arc::new(spinlocks_array),
            gpio_out: Arc::new(AtomicU32::new(0)),
            gpio_oe: Arc::new(AtomicU32::new(0)),
            master_cycle: Arc::new(AtomicU64::new(0)),
            poisoned: Arc::new(AtomicBool::new(false)),
            panic_info: Arc::new(Mutex::new(None)),
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
    use std::thread;

    /// Compile-time proof that `SharedState` is `Send + Sync + Clone`.
    fn _assert_send_sync_clone<T: Send + Sync + Clone>() {}
    #[test]
    fn shared_state_is_send_sync_clone() {
        _assert_send_sync_clone::<SharedState>();
    }

    #[test]
    fn clone_shares_inner_arcs() {
        let a = SharedState::new_default();
        let b = a.clone();
        a.master_cycle.store(42, Ordering::Release);
        assert_eq!(b.master_cycle.load(Ordering::Acquire), 42);
        assert!(Arc::ptr_eq(&a.master_cycle, &b.master_cycle));
        assert!(Arc::ptr_eq(&a.memory, &b.memory));
        assert!(Arc::ptr_eq(&a.atomics[0], &b.atomics[0]));
        assert!(Arc::ptr_eq(&a.sio_fifo_0_to_1, &b.sio_fifo_0_to_1));
        assert!(Arc::ptr_eq(&a.spinlocks, &b.spinlocks));
    }

    #[test]
    fn fifo_cross_thread_push_pop() {
        // Sanity check that the SPSC FIFO actually carries a payload
        // across a thread boundary when passed through SharedState.
        let state = SharedState::new_default();
        let producer_state = state.clone();
        let producer = thread::spawn(move || {
            assert!(producer_state.sio_fifo_0_to_1.try_push(0xCAFE_F00D));
        });
        producer.join().unwrap();
        let got = state.sio_fifo_0_to_1.try_pop();
        assert_eq!(got, Some(0xCAFE_F00D));
    }

    #[test]
    fn panic_info_roundtrips_via_mutex() {
        let state = SharedState::new_default();
        assert!(!state.poisoned.load(Ordering::Acquire));
        {
            let mut guard = state.panic_info.lock().unwrap();
            *guard = Some((WorkerName::Core0, "boom".into()));
        }
        state.poisoned.store(true, Ordering::Release);
        assert!(state.poisoned.load(Ordering::Acquire));
        let info = state.panic_info.lock().unwrap().clone();
        assert_eq!(info, Some((WorkerName::Core0, "boom".to_string())));
    }
}

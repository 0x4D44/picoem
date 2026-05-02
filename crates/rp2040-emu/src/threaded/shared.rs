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
//! Differences vs rp2350_emu's `SharedState`:
//! - No FPU / RCP / coprocessor state (M0+ has none).
//! - 32 spinlocks (RP2040) vs 32 spinlocks (RP2350 also has 32 — same).
//! - 30 GPIOs (RP2040) — still fits `AtomicU32`.
//! - No XIP flash — `SharedMemory` is ROM + SRAM only.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64};
use std::sync::{Arc, Mutex};

use picoem_common::threaded::SpscQueue;

use crate::WorkerName;
use crate::bus::ppb::Ppb;
use crate::threaded::{CoreAtomics, Peripherals, SharedMemory, ThreadedPio};

/// Arc-bundled shared state handed to every worker in the threaded
/// runtime. Cloning is cheap (refcount bumps + bundle copy).
#[derive(Clone)]
pub struct SharedState {
    /// Shared SRAM / ROM memory (atomic words + read-only byte slice).
    pub memory: Arc<SharedMemory>,
    /// Cross-core atomics (halted / WFE / event_flag / irq_pending /
    /// bus_fault). `CoreAtomics` itself carries per-core arrays
    /// internally, so one `Arc` is sufficient — the previous
    /// `[Arc<CoreAtomics>; 2]` shape would have routed peer-core
    /// publishes to a sibling Arc instance that held the same per-core
    /// bits, which worked by accident when producer + consumer agreed
    /// on which slot to index. Collapsed to a single Arc after Stage
    /// 3b.3 review.
    pub atomics: Arc<CoreAtomics>,
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
    /// External GPIO input override value, mirrors
    /// `Bus::external_gpio_in_override`. Bits set in
    /// [`Self::external_gpio_in_mask`] take their value from this field
    /// in the coordinator's post-merge GPIO_IN snapshot. Harness drivers
    /// (e.g. `picogus_diff_rp2040`) store here to inject synthetic ISA
    /// bus waveforms.
    pub external_gpio_in_override: Arc<AtomicU32>,
    /// Mask of GPIO input bits driven externally, mirrors
    /// `Bus::external_gpio_in_mask`. Zero means the SIO + PIO merge
    /// drives every pin.
    pub external_gpio_in_mask: Arc<AtomicU32>,
    /// Monotonic master-cycle counter published by the coordinator and
    /// read lock-free by the CPU workers (PLL LOCK timing, TIMER match,
    /// etc.).
    pub master_cycle: Arc<AtomicU64>,
    /// Mutex-guarded bundle of cold-path peripheral registers (CLOCKS /
    /// PLL / XOSC / ROSC / RESETS / IO_BANK0 / PADS_BANK0 / legacy
    /// HashMap). See [`Peripherals`] for the layout.
    pub peripherals: Arc<Peripherals>,
    /// PIO shared state: per-block command queues + coordinator-refreshed
    /// register snapshot + `sm_enabled` atomics. Stage 3b.3 populates the
    /// queue write path; Stage 3b.4 wires the coordinator drain.
    pub pio: Arc<ThreadedPio>,
    /// Sticky panic flag. Set by the first worker to unwind; subsequent
    /// workers observe it and exit their quantum early.
    pub poisoned: Arc<AtomicBool>,
    /// Structured panic info for attribution. Mutex-guarded because it
    /// is written on the cold panic path, read once by the coordinator
    /// at join time.
    pub panic_info: Arc<Mutex<Option<(WorkerName, String)>>>,
    /// Initial per-core PPB state carried from `from_emulator` into the
    /// CPU workers' `WorkerBus`. Seeded once under the mutex by
    /// `from_emulator`; each core worker calls
    /// [`Self::take_initial_ppb`] on its first quantum entry to consume
    /// its slot. Wrapped in a `Mutex<Option<...>>` to avoid widening the
    /// `spawn_worker` signature while still preserving pre-run pokes
    /// (VTOR, NVIC pending/enable via ICSR, SHPR priorities).
    pub initial_ppb: Arc<Mutex<Option<[Ppb; 2]>>>,
}

impl SharedState {
    /// Construct a fresh `SharedState` with every inner component in
    /// its default / post-boot state. Stage 3b.4 may grow a
    /// `from_emulator` / `from_parts` sibling once the `Emulator` bus
    /// is refactored to hand its memory over.
    pub fn new_default() -> Self {
        // 32 AtomicU32 cells. Array init via `core::array::from_fn`
        // keeps the constructor boilerplate contained.
        let spinlocks_array: [AtomicU32; 32] = std::array::from_fn(|_| AtomicU32::new(0));

        Self {
            memory: Arc::new(SharedMemory::new_zero()),
            atomics: Arc::new(CoreAtomics::default()),
            // Capacity 8 matches rp2350_emu / RP2040 SIO FIFO depth.
            sio_fifo_0_to_1: Arc::new(SpscQueue::new(8)),
            sio_fifo_1_to_0: Arc::new(SpscQueue::new(8)),
            spinlocks: Arc::new(spinlocks_array),
            gpio_out: Arc::new(AtomicU32::new(0)),
            gpio_oe: Arc::new(AtomicU32::new(0)),
            external_gpio_in_override: Arc::new(AtomicU32::new(0)),
            external_gpio_in_mask: Arc::new(AtomicU32::new(0)),
            master_cycle: Arc::new(AtomicU64::new(0)),
            peripherals: Arc::new(Peripherals::new_default()),
            pio: Arc::new(ThreadedPio::new()),
            poisoned: Arc::new(AtomicBool::new(false)),
            panic_info: Arc::new(Mutex::new(None)),
            initial_ppb: Arc::new(Mutex::new(None)),
        }
    }

    /// Take a clone of the initial per-core PPB seed for `core_id`.
    /// Returns [`Ppb::new()`] if no seed was populated. The slot is
    /// populated by [`crate::threaded::ThreadedEmulator::from_emulator`]
    /// when the serial `Bus` carries non-default PPB state (VTOR,
    /// NVIC enable/pending via ICSR, SHPR priorities). Both workers
    /// may call this on their first quantum entry — the snapshot is
    /// shared (cheap `Clone`) and only cleared after both cores have
    /// read their slot by the caller choosing to `*guard = None`.
    pub fn take_initial_ppb(&self, core_id: usize) -> Ppb {
        debug_assert!(core_id < 2);
        let guard = self.initial_ppb.lock().expect("initial_ppb poisoned");
        match &*guard {
            Some(pair) => pair[core_id].clone(),
            None => Ppb::new(),
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
        assert!(Arc::ptr_eq(&a.atomics, &b.atomics));
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

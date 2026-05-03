//! `ThreadedEmulator` — 3-thread runtime entry point for the RP2040
//! dual-execution path (Stage 3b.4).
//!
//! Mirrors the rp2350_emu shape but simpler: three threads (core0, core1,
//! coordinator) instead of six, because RP2040 has no separate PIO-per-
//! block split — PIO steps on the coordinator in Stage 3b.4. If Stage 4
//! benchmarks find PIO to be a bottleneck, the coordinator can be
//! further split into per-block PIO workers.
//!
//! Gated behind `#[cfg(all(target_arch = "x86_64", any(target_os =
//! "windows", target_os = "linux")))]` (via the parent
//! `threaded/mod.rs`) because the thread-pinning path uses
//! `SetThreadAffinityMask` on Windows and `pthread_setaffinity_np` on
//! Linux. Other UNIX hosts stay on the existing single-threaded
//! `Emulator::run` path until `pin_to_host_core` grows a port.
//!
//! Lifecycle at a glance:
//!
//! 1. Caller drives an existing `Emulator` to the pre-run state (load
//!    ROM / flash, reset, seed GPIO stimulus, etc.).
//! 2. `ThreadedEmulator::from_emulator(emu)` destructures the serial
//!    `Bus` into the shared state bundle and takes ownership of both
//!    CPU cores and the two `PioBlock`s.
//! 3. `run_quanta_checked(n)` spawns three workers (core 0, core 1,
//!    coordinator), joins them, and surfaces panics via the `poisoned`
//!    flag so the instance cannot be reused after a worker panic.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use picoem_common::PioBlock;
use tracing::{error, warn};

use crate::bus::Bus;
use crate::core::CortexM0Plus;
use crate::{Emulator, WorkerName};

use super::memory as tmem;
use super::peripherals::{ClocksState, IoState, ResetsState, TimerState};
use super::{
    BarrierResult, CoreAtomics, Peripherals, PioCommand, SharedMemory, SharedState, SpinBarrier,
    ThreadedPio, panic_message, spawn_worker,
};

// =======================================================================
// ThreadedEmulator
// =======================================================================

/// Runtime-error payload returned from [`ThreadedEmulator::run_quanta_checked`].
/// Distinguishes a worker panic from a barrier-watchdog timeout so the
/// outer [`crate::EmulatorError`] surface can expose the two cases as
/// separate variants (HLD V1 §6.6 Stage 5).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RunError {
    /// One of the worker threads panicked. `message` is the downcast
    /// payload text; `which` is the first worker to return `Err` from
    /// `JoinHandle::join` scanning in worker-index order.
    Panic { which: WorkerName, message: String },
    /// The shared barrier's wall-clock deadline elapsed before all
    /// workers arrived. `which` names the first worker whose barrier
    /// call returned `TimedOut` (an observer, not the culprit); the
    /// barrier cannot identify the missing worker on its own.
    /// `elapsed_ms` is the wall-clock elapsed time recorded by the
    /// first waiter to trip the watchdog.
    Timeout { which: WorkerName, elapsed_ms: u32 },
}

/// 3-thread runtime handle over a seeded `SharedState` and both CPU
/// cores. Construction is via [`Self::from_emulator`].
pub struct ThreadedEmulator {
    shared: Arc<SharedState>,
    core0: Option<CortexM0Plus>,
    core1: Option<CortexM0Plus>,
    /// Coordinator-owned PIO blocks. CPU workers enqueue commands onto
    /// `shared.pio`; the coordinator drains them and steps the blocks.
    pio_blocks: Option<[PioBlock; 2]>,
    step_quantum: u32,
    thread_mask: [usize; 3],
    poisoned: bool,
    /// Test-only panic injection target. Consumed on the next
    /// `run_quanta_checked` entry: the matching worker panics on its
    /// first barrier wait.
    #[cfg(feature = "testing")]
    pending_panic_inject: Option<WorkerName>,
}

impl ThreadedEmulator {
    /// Consume a single-threaded `Emulator` and return a
    /// `ThreadedEmulator` with every piece of state hoisted onto the
    /// shared `SharedState`.
    ///
    /// Panics if `std::thread::available_parallelism()` reports fewer
    /// than 3 host cores — the runtime pins one thread per core.
    pub fn from_emulator(emu: Emulator) -> Self {
        let n = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        assert!(
            n >= 3,
            "ThreadedEmulator requires >= 3 host cores (found {n})"
        );

        let Emulator {
            cores,
            bus,
            clock,
            step_quantum,
            // Diagnostic counters are serial-path only — drop on handoff.
            pio_tick_count: _,
            pio_tick_iow_low_count: _,
            pio0_sm0_max_pc: _,
            pio0_sm0_pc_advances: _,
            pio0_sm0_last_pc: _,
            execution_model: _,
            threaded: _,
            panic_info: _,
            timeout_info: _,
            #[cfg(feature = "testing")]
                pending_panic_inject: _,
            bus_is_placeholder: _,
        } = emu;

        let [core0, core1] = cores;

        // PSRAM is not modelled on the threaded path — the PicoGUS
        // harness that uses it should stay on Serial (HLD §6.4 + Stage
        // 3b.3 tech_debt). Warn loudly if the caller attached one;
        // silent drop would strip the MISO feedback from `update_gpio`
        // and deadlock a PSRAM-reading firmware.
        if bus.psram.is_some() {
            warn!(
                "ThreadedEmulator::from_emulator: attached PSRAM device dropped \
                 — threaded path does not model sub-quantum SPI edge timing. \
                 Use ExecutionModel::Serial for PSRAM-driven firmware."
            );
        }

        // Build the shared memory by copying ROM + SRAM contents out of
        // the serial `Memory` struct. ROM is sealed read-only after this;
        // SRAM becomes atomic-word storage shared across workers.
        let shared_memory = Arc::new(clone_memory(&bus));

        // Seed `gpio_out` / `gpio_oe` from the SIO registers. The
        // coordinator overwrites these each quantum with the merged
        // (SIO | PIO) pin view, so this seed is only observed until the
        // first quantum's merge completes. External GPIO injection
        // (harness pokes to `Bus::external_gpio_in_{override,mask}`) is
        // carried on its own atomics below and applied in the WorkerBus
        // SIO_GPIO_IN read path. PSRAM MISO feedback is not modelled on
        // the threaded path — see warn above.
        let gpio_out = Arc::new(std::sync::atomic::AtomicU32::new(bus.sio.gpio_out));
        let gpio_oe = Arc::new(std::sync::atomic::AtomicU32::new(bus.sio.gpio_oe));
        // Carry `Bus::external_gpio_in_override` / `..._mask` onto the
        // threaded shared state so `picogus_diff_rp2040`-style harnesses
        // keep driving pins post-promotion.
        let external_gpio_in_override = Arc::new(std::sync::atomic::AtomicU32::new(
            bus.external_gpio_in_override,
        ));
        let external_gpio_in_mask =
            Arc::new(std::sync::atomic::AtomicU32::new(bus.external_gpio_in_mask));

        // CoreAtomics bundle — seeded with the serial bus's IRQ pending
        // mask broadcast to both cores, WFE / halted state, bus fault
        // state.
        let atomics = Arc::new(CoreAtomics::default());
        for c in 0..2usize {
            // Serial path uses a single `bus.irq_pending` — broadcast to
            // both cores so firmware that poked NVIC_ISPR pre-run sees
            // the pending bit on whichever core consumes it first.
            if bus.irq_pending != 0 {
                atomics.set_irq_pending(c, bus.irq_pending);
            }
            if bus.event_flag[c] {
                atomics.set_event_flag(c);
            }
            // WFE-park state survives promotion — see
            // `wrk_docs/2026.04.26 - HLD - RP2040 WFE-SEV Wake Mechanics
            // V1.md` §4.5. Without this, a core that parked in Serial
            // mode would silently un-park on promotion.
            if bus.wfe_waiting[c] {
                atomics.set_wfe_waiting(c);
            }
        }
        // Halted state follows the cores themselves after `take`; mirror
        // their `halted` flags now so the atomic view matches the core's
        // own bool.
        if core0.is_halted() {
            atomics.set_halted(0);
        }
        if core1.is_halted() {
            atomics.set_halted(1);
        }

        // Peripherals — take a ClocksState / ResetsState / IoState /
        // TimerState populated from the serial Bus's typed fields so the
        // first worker MMIO read sees firmware's pre-run pokes.
        let peripherals = Arc::new(Peripherals {
            clocks: std::sync::Mutex::new(ClocksState {
                clocks_regs: bus.clocks_regs,
                xosc_regs: bus.xosc_regs,
                rosc_regs: bus.rosc_regs,
                pll_sys_regs: bus.pll_sys_regs,
                pll_usb_regs: bus.pll_usb_regs,
                pll_sys_lock_at_cycle: bus.pll_sys_lock_at_cycle,
                pll_usb_lock_at_cycle: bus.pll_usb_lock_at_cycle,
                clock_tree: bus.clock_tree,
            }),
            resets: std::sync::Mutex::new(ResetsState { resets: bus.resets }),
            io: std::sync::Mutex::new(IoState {
                io_bank0: bus.io_bank0,
                pads_bank0: bus.pads_bank0,
            }),
            timer: std::sync::Mutex::new(TimerState { regs: bus.timer }),
            legacy: std::sync::Mutex::new(std::collections::HashMap::new()),
        });

        // Seed ThreadedPio sm_enabled from the incoming PioBlocks so a
        // caller that programmed CTRL.SM_ENABLE through the serial Bus
        // before `from_emulator` is honoured from the first quantum.
        let threaded_pio = Arc::new(ThreadedPio::new());
        let pio_blocks: [PioBlock; 2] = bus.pio;
        for (idx, block) in pio_blocks.iter().enumerate() {
            threaded_pio.publish_sm_enabled(idx, block.sm_enabled_mask());
        }

        // Seed the SPSC cross-core FIFOs from the serial SIO FIFO
        // snapshots so pre-run pushes (harness setup) survive the
        // handoff. Ordering matches `rp2350_emu::threaded::ThreadedSio::
        // seed`: head → tail. Capacity 8 on both ends, so the snapshot
        // (≤ 8 entries) cannot overflow. Also propagate the sticky
        // WOF / ROE flags onto `CoreAtomics` so FIFO_ST reads match
        // the serial behaviour from the first quantum.
        let sio_fifo_0_to_1 = Arc::new(picoem_common::threaded::SpscQueue::new(8));
        let sio_fifo_1_to_0 = Arc::new(picoem_common::threaded::SpscQueue::new(8));
        for val in bus.sio.fifo_0to1_snapshot() {
            let _ = sio_fifo_0_to_1.try_push(val);
        }
        for val in bus.sio.fifo_1to0_snapshot() {
            let _ = sio_fifo_1_to_0.try_push(val);
        }
        for core in 0..2 {
            if bus.sio.fifo_wof(core) {
                atomics.set_fifo_wof(core);
            }
            if bus.sio.fifo_roe(core) {
                atomics.set_fifo_roe(core);
            }
        }

        // Seed the 32 spinlock cells from the serial `Sio::spinlock_bits`
        // claim bitmap so harnesses that pre-claim a lock see the held
        // state from the first threaded quantum. Each cell stores 0
        // (unlocked) or a non-zero token (lock 1..=32 maps to token N+1
        // to avoid aliasing against a zero 'unlocked' default).
        let spinlocks_array: [std::sync::atomic::AtomicU32; 32] =
            std::array::from_fn(|_| std::sync::atomic::AtomicU32::new(0));
        let sl_bits = bus.sio.spinlock_bits();
        for n in 0..32 {
            if sl_bits & (1u32 << n) != 0 {
                spinlocks_array[n].store(n as u32 + 1, std::sync::atomic::Ordering::Relaxed);
            }
        }
        let spinlocks = Arc::new(spinlocks_array);

        // Carry the serial `Bus::ppb` (VTOR / SHPR / ICSR / active
        // bitmap) into `SharedState.initial_ppb`. WorkerBus::new()
        // consumes its slot on construction (see `take_initial_ppb`).
        let initial_ppb = Arc::new(std::sync::Mutex::new(Some([
            bus.ppb[0].clone(),
            bus.ppb[1].clone(),
        ])));

        let shared = Arc::new(SharedState {
            memory: shared_memory,
            atomics,
            sio_fifo_0_to_1,
            sio_fifo_1_to_0,
            spinlocks,
            gpio_out,
            gpio_oe,
            external_gpio_in_override,
            external_gpio_in_mask,
            master_cycle: Arc::new(AtomicU64::new(clock.cycles.max(bus.master_cycle))),
            peripherals,
            pio: threaded_pio,
            poisoned: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            panic_info: Arc::new(std::sync::Mutex::new(None)),
            initial_ppb,
        });

        Self {
            shared,
            core0: Some(core0),
            core1: Some(core1),
            pio_blocks: Some(pio_blocks),
            step_quantum,
            thread_mask: [0, 1, 2],
            poisoned: false,
            #[cfg(feature = "testing")]
            pending_panic_inject: None,
        }
    }

    /// Current shared master-cycle count. Lock-free `Acquire` load.
    pub fn master_cycle(&self) -> u64 {
        self.shared.master_cycle.load(Ordering::Acquire)
    }

    /// Cycle counter for core `idx` (0 or 1). Returns 0 while a
    /// `run_quanta_checked` call is in flight (cores are `take`n into
    /// worker threads).
    pub fn core_cycles(&self, idx: u8) -> u64 {
        match idx {
            0 => self.core0.as_ref().map_or(0, |c| c.cycles()),
            1 => self.core1.as_ref().map_or(0, |c| c.cycles()),
            _ => panic!("ThreadedEmulator::core_cycles: idx must be 0 or 1"),
        }
    }

    /// Borrow the `SharedState` for harness-side inspection (e.g. GPIO
    /// pin reads, shared-memory peeks).
    pub fn shared(&self) -> &Arc<SharedState> {
        &self.shared
    }

    /// Test-only: arm a panic injection for the next `run_quanta_checked`
    /// call. Panics the named worker on its first barrier entry.
    #[cfg(feature = "testing")]
    pub fn inject_panic_for_testing(&mut self, which: WorkerName) {
        self.pending_panic_inject = Some(which);
    }

    /// Run `n` quanta and surface worker panics as structured
    /// `Err((which, message))` instead of re-raising. On `Err`, the
    /// instance is poisoned — drop it and rebuild.
    pub fn run_quanta_checked(&mut self, n: u64) -> Result<(), RunError> {
        assert!(
            !self.poisoned,
            "ThreadedEmulator poisoned by prior worker panic; drop and rebuild"
        );

        let core0 = self.core0.take().expect("run_quanta reentry");
        let core1 = self.core1.take().expect("run_quanta reentry");
        let [block0, block1] = self
            .pio_blocks
            .take()
            .expect("run_quanta reentry (pio_blocks)");

        let barrier = Arc::new(SpinBarrier::new(3));
        let shared = Arc::clone(&self.shared);
        let step_q = self.step_quantum;
        let mask = self.thread_mask;

        // Test-only panic injection — each worker body checks a local
        // `inject` flag captured at spawn time.
        #[cfg(feature = "testing")]
        let (inject_core0, inject_core1, inject_coord) = match self.pending_panic_inject.take() {
            Some(WorkerName::Core0) => (true, false, false),
            Some(WorkerName::Core1) => (false, true, false),
            Some(WorkerName::Coord) => (false, false, true),
            None => (false, false, false),
        };
        #[cfg(not(feature = "testing"))]
        let (inject_core0, inject_core1, inject_coord) = (false, false, false);

        let h0 = spawn_worker(mask[0], barrier.clone(), {
            let s = Arc::clone(&shared);
            move |b| core_worker_body(0, core0, s, b, n, step_q, inject_core0)
        });
        let h1 = spawn_worker(mask[1], barrier.clone(), {
            let s = Arc::clone(&shared);
            move |b| core_worker_body(1, core1, s, b, n, step_q, inject_core1)
        });
        let hc = spawn_worker(mask[2], barrier.clone(), {
            let s = Arc::clone(&shared);
            move |b| coordinator_worker_body(s, [block0, block1], b, n, step_q, inject_coord)
        });

        let r0 = h0.join();
        let r1 = h1.join();
        let rc = hc.join();

        let msg0 = panic_message(r0.as_ref().err());
        let msg1 = panic_message(r1.as_ref().err());
        let msgc = panic_message(rc.as_ref().err());

        let r0_err = r0.is_err();
        let r1_err = r1.is_err();
        let rc_err = rc.is_err();

        // Restore owned state on the happy path.
        if let Ok(c) = r0 {
            self.core0 = Some(c);
        }
        if let Ok(c) = r1 {
            self.core1 = Some(c);
        }
        if !rc_err {
            if let Ok([b0, b1]) = rc {
                self.pio_blocks = Some([b0, b1]);
            }
        } else {
            // Coordinator panicked — drop the blocks (they're lost to
            // the joined Err payload anyway). The poisoned flag below
            // rejects further calls.
            self.pio_blocks = None;
        }

        // First panicked worker wins attribution, scanning in worker-
        // index order (core0, core1, coord).
        let first_panic: Option<(WorkerName, String)> = [
            (WorkerName::Core0, r0_err, msg0),
            (WorkerName::Core1, r1_err, msg1),
            (WorkerName::Coord, rc_err, msgc),
        ]
        .into_iter()
        .find_map(|(name, err, msg)| if err { Some((name, msg)) } else { None });

        if let Some((which, message)) = first_panic {
            self.poisoned = true;
            // Mirror into shared.panic_info so parallel reads see it.
            match self.shared.panic_info.lock() {
                Ok(mut guard) => {
                    *guard = Some((which, message.clone()));
                }
                Err(_) => {
                    // A prior panic already poisoned the mutex; the
                    // structured attribution is lost for this round but
                    // the sticky `self.poisoned` still rejects further
                    // use. Log once at `error!` level so the mismatch
                    // between a user-visible WorkerPanicked error and
                    // an empty `shared.panic_info` is diagnosable.
                    error!(
                        which = which.as_str(),
                        "panic_info mutex poisoned during panic handling — \
                         structured attribution lost; emulator remains \
                         sticky-poisoned"
                    );
                }
            }
            self.shared
                .poisoned
                .store(true, std::sync::atomic::Ordering::Release);
            return Err(RunError::Panic { which, message });
        }

        // Stage 5 (HLD V1 §6.6): watchdog-fired barrier exits all
        // workers cleanly via `TimedOut`, so no `JoinHandle::join`
        // returns Err. Inspect the barrier directly to distinguish a
        // timeout from an ordinary clean return. `WorkerName::Coord` is
        // the observer attribution — the barrier cannot identify the
        // missing worker.
        if barrier.timed_out() {
            self.poisoned = true;
            self.shared
                .poisoned
                .store(true, std::sync::atomic::Ordering::Release);
            return Err(RunError::Timeout {
                which: WorkerName::Coord,
                elapsed_ms: barrier.timeout_elapsed_ms(),
            });
        }

        Ok(())
    }
}

// =======================================================================
// Memory cloner — copies bytes out of serial Memory into SharedMemory.
// =======================================================================

/// Clone a serial `Bus`'s `memory` field into a fresh `SharedMemory`.
/// ROM contents become the read-only byte slice; SRAM contents become
/// atomic-word storage.
fn clone_memory(bus: &Bus) -> SharedMemory {
    let mut rom_bytes = vec![0u8; tmem::ROM_SIZE as usize];
    for i in 0..tmem::ROM_SIZE {
        rom_bytes[i as usize] = bus.memory.rom_read8(i);
    }
    let rom: Arc<[u8]> = rom_bytes.into();

    let mut sram_words: Vec<std::sync::atomic::AtomicU32> = Vec::with_capacity(tmem::SRAM_WORDS);
    for i in 0..tmem::SRAM_WORDS {
        let w = bus.memory.sram_read32((i as u32) * 4);
        sram_words.push(std::sync::atomic::AtomicU32::new(w));
    }
    let sram: Arc<[std::sync::atomic::AtomicU32]> = sram_words.into();
    SharedMemory::new(rom, sram)
}

// =======================================================================
// Worker-thread plumbing
// =======================================================================
//
// `panic_message`, `spawn_worker`, and `pin_to_host_core` were promoted
// to `picoem-common::threaded::worker` per the 2026-04-30 Threaded
// Helpers Pull-Up HLD V1. They reach this file via the `use super::{...}`
// import at the top of the file (re-exported from
// `crate::threaded::mod.rs`).

// =======================================================================
// Worker bodies
// =======================================================================

/// CPU-core worker. Owns a `CortexM0Plus` and drives `step` against a
/// per-core `WorkerBus`.
fn core_worker_body(
    core_id: usize,
    mut core: CortexM0Plus,
    shared: Arc<SharedState>,
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
    inject_panic: bool,
) -> CortexM0Plus {
    use super::WorkerBus;
    let mut bus = WorkerBus::new(Arc::clone(&shared), core_id);
    let mut target: u64 = core.cycles();

    for quantum in 0..n {
        // Test-only panic injection — panic on the first quantum so the
        // attribution test sees a deterministic panic point.
        if inject_panic && quantum == 0 {
            if core_id == 0 {
                panic!("core0 test panic");
            } else {
                panic!("core1 test panic");
            }
        }

        // WFE wake: consume event_flag and clear wfe_waiting.
        if shared.atomics.is_wfe_waiting(core_id) && shared.atomics.event_flag_consume(core_id) {
            shared.atomics.clear_wfe_waiting(core_id);
        }

        // Drain cross-core IRQ pending bits into the local NVIC at the
        // top of the quantum so peer-asserted IRQs (FIFO push,
        // peripheral drive) become visible before the first step.
        bus.drain_cross_core_irqs();

        target = target.wrapping_add(step_q as u64);
        if !shared.atomics.is_halted(core_id) {
            while core.cycles() < target && !core.is_halted() {
                let consumed = core.step(&mut bus);
                // HLD V5 §5.2: tick this worker's SysTick once per
                // master cycle, mirroring the serial slow-path tick at
                // lib.rs ~665. The threaded path has no per-cycle
                // `tick_peripherals` analogue (peripherals are bulk-
                // handled at the quantum boundary by the coordinator),
                // so this fires immediately after `core.step` and
                // before `drain_cross_core_irqs` so a SysTick-asserted
                // ICSR.PENDSTSET aligns with this cycle. Each worker
                // only ticks its own `systicks[core_id]` slot and ORs
                // into its own `ppb[core_id].icsr`.
                //
                // `core.step` returns the cycle count consumed by the
                // instruction (e.g. `LDM r0, {r1-r7}` = 8, `BL` = 4);
                // tick once per master cycle so SysTick rate stays
                // coupled to the master clock and matches the serial
                // path's per-cycle tick.
                let cid = bus.core_id;
                for _ in 0..consumed {
                    if bus.systicks[cid].tick() {
                        bus.ppb[cid].icsr |= 1 << 26;
                    }
                }
                // Drain any mid-step cross-core IRQ arrivals so the
                // next instruction observes them.
                bus.drain_cross_core_irqs();
                if shared.atomics.is_wfe_waiting(core_id) {
                    break;
                }
            }
        }

        let result = barrier.wait();
        if matches!(
            result,
            BarrierResult::Poisoned | BarrierResult::TimedOut { .. }
        ) {
            return core;
        }
    }
    core
}

/// Coordinator worker. Owns the `PioBlock`s, drains CPU-queued PIO
/// commands, steps the blocks, merges GPIO, and advances the shared
/// master-cycle counter. Rendezvouses on the shared barrier at the tail
/// of each quantum.
fn coordinator_worker_body(
    shared: Arc<SharedState>,
    mut pio_blocks: [PioBlock; 2],
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
    inject_panic: bool,
) -> [PioBlock; 2] {
    for quantum in 0..n {
        if inject_panic && quantum == 0 {
            panic!("coord test panic");
        }

        // 1. Drain per-block PIO commands and apply them.
        for block_idx in 0..2 {
            let commands = shared.pio.drain_commands(block_idx);
            for cmd in commands {
                apply_pio_command(&mut pio_blocks[block_idx], cmd);
            }
        }

        // 2. Advance PIO state machines for `step_q` sysclocks. PIO sees
        //    the latest merged GPIO pin state (initial value for quantum 0
        //    comes from from_emulator's seed; subsequent quanta see the
        //    previous quantum's merge result).
        let gpio_snapshot =
            shared.gpio_out.load(Ordering::Acquire) & shared.gpio_oe.load(Ordering::Acquire);
        for block_idx in 0..2 {
            if shared.pio.sm_enabled(block_idx) != 0 {
                pio_blocks[block_idx].step_n(step_q, gpio_snapshot);
            }
            shared
                .pio
                .publish_sm_enabled(block_idx, pio_blocks[block_idx].sm_enabled_mask());
        }

        // 3. Merge SIO + PIO outputs into gpio_out / gpio_oe atomics so
        //    worker-side GPIO_IN reads observe the post-merge state.
        //    SIO output is already in `shared.gpio_out`; PIO pad_oe/
        //    pad_out overrides it where a PIO block drives.
        {
            let sio_out = shared.gpio_out.load(Ordering::Acquire);
            let sio_oe = shared.gpio_oe.load(Ordering::Acquire);
            let mut merged_out = sio_out & sio_oe;
            let mut merged_oe = sio_oe;
            for block in &pio_blocks {
                let pio_mask = block.pad_oe;
                merged_out = (merged_out & !pio_mask) | (block.pad_out & pio_mask);
                merged_oe |= pio_mask;
            }
            // Mask to 30 GPIO pins (RP2040 has 0..29).
            merged_out &= 0x3FFF_FFFF;
            merged_oe &= 0x3FFF_FFFF;
            // Publish merged state back for worker SIO_GPIO_IN reads.
            // We overwrite gpio_out/gpio_oe with the merged view so the
            // WorkerBus SIO_GPIO_IN path (which uses `out & oe`) sees
            // the correct merged pin state. SIO writes that come in
            // during the next quantum stamp back onto these atomics.
            shared.gpio_out.store(merged_out, Ordering::Release);
            shared.gpio_oe.store(merged_oe, Ordering::Release);
        }

        // 4. Advance master_cycle by step_quantum. Release ordering
        //    pairs with CPU workers' Acquire load on the next quantum.
        let new_master = shared
            .master_cycle
            .fetch_add(step_q as u64, Ordering::Release)
            .wrapping_add(step_q as u64);

        // 5. Poll TIMER alarms against the freshly-advanced master
        //    cycle and route any latched IRQs onto both cores'
        //    cross-core pending mask (TIMER occupies NVIC lines 0..3,
        //    shared across cores on RP2040 silicon).
        //
        //    Stage 3b.4 scope: TIMER only. Other peripherals with
        //    per-cycle ticks (UART/SPI/I2C/ADC/PWM) remain un-advanced
        //    on the threaded path — tech_debt.md covers the escape
        //    for firmware that depends on them.
        {
            let sys_hz = {
                let clocks = shared
                    .peripherals
                    .clocks
                    .lock()
                    .expect("clocks mutex poisoned");
                clocks.clock_tree.sys_clk_hz
            };
            let nvic_bits = {
                let mut timer = shared
                    .peripherals
                    .timer
                    .lock()
                    .expect("timer mutex poisoned");
                timer.regs.poll_alarms(new_master, sys_hz) & 0xF
            };
            if nvic_bits != 0 {
                shared.atomics.set_irq_pending(0, nvic_bits);
                shared.atomics.set_irq_pending(1, nvic_bits);
            }
        }

        let result = barrier.wait();
        if matches!(
            result,
            BarrierResult::Poisoned | BarrierResult::TimedOut { .. }
        ) {
            return pio_blocks;
        }
    }
    pio_blocks
}

/// Apply a CPU-queued `PioCommand` to the coordinator's owned PioBlock.
fn apply_pio_command(block: &mut PioBlock, cmd: PioCommand) {
    match cmd {
        PioCommand::WriteCtrl {
            block: _,
            val,
            alias,
        } => {
            block.write32(0x000, val, alias as u32);
        }
        PioCommand::WriteInstrMem {
            block: _,
            addr,
            value,
            alias,
        } => {
            if addr < 32 {
                let offset = 0x048 + (addr as u32) * 4;
                block.write32(offset, value as u32, alias as u32);
            }
        }
        PioCommand::SetClkDiv {
            block: _,
            sm,
            int_div,
            frac_div,
            alias,
        } => {
            if sm < 4 {
                // RP2040 SMn_CLKDIV: stride 0x18 starting at 0x0C8.
                let offset = 0x0C8 + (sm as u32) * 0x18;
                let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
                block.write32(offset, val, alias as u32);
            }
        }
        PioCommand::WriteReg {
            block: _,
            offset,
            val,
            alias,
        } => {
            block.write32(offset as u32, val, alias as u32);
        }
    }
}

// =======================================================================
// Tests
// =======================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Config, EmulatorBuilder};

    #[test]
    fn from_emulator_builds_threaded() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let threaded = ThreadedEmulator::from_emulator(emu);
        // master_cycle starts at 0 for a fresh Emulator.
        assert_eq!(threaded.master_cycle(), 0);
    }

    #[test]
    fn run_quanta_halted_cores_advances_master_cycle() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        // Both cores default halted (core 1) / soon halted (core 0).
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        let step_q = threaded.step_quantum as u64;
        assert_eq!(threaded.master_cycle(), 0);

        threaded
            .run_quanta_checked(1)
            .expect("run_quanta_checked(1)");
        assert_eq!(threaded.master_cycle(), step_q);

        threaded
            .run_quanta_checked(5)
            .expect("run_quanta_checked(5)");
        assert_eq!(threaded.master_cycle(), 6 * step_q);
    }

    #[cfg(feature = "testing")]
    #[test]
    fn run_quanta_core0_panic_surfaces() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.inject_panic_for_testing(WorkerName::Core0);

        let result = threaded.run_quanta_checked(1);
        match result {
            Err(RunError::Panic {
                which: WorkerName::Core0,
                ref message,
            }) => {
                assert!(
                    message.contains("core0"),
                    "message should name core0: {message}"
                );
            }
            other => panic!("expected Err(RunError::Panic{{Core0,...}}), got {other:?}"),
        }
    }

    /// Test 14 (HLD §5): the serial → threaded handoff must lift
    /// `Bus::wfe_waiting[c]` into `CoreAtomics::wfe_waiting[c]`.
    /// Without this, a core parked on WFE in Serial mode would
    /// silently un-park on promotion. See
    /// `wrk_docs/2026.04.26 - HLD - RP2040 WFE-SEV Wake Mechanics
    /// V1.md` §4.5.
    #[test]
    fn from_emulator_preserves_wfe_waiting() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        emu.bus.wfe_waiting[1] = true;
        // Halt both cores so `from_emulator` can take ownership cleanly.
        emu.cores[0].halt();
        emu.cores[1].halt();

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert!(
            threaded.shared.atomics.is_wfe_waiting(1),
            "wfe_waiting[1] must be lifted into CoreAtomics on promotion"
        );
        assert!(
            !threaded.shared.atomics.is_wfe_waiting(0),
            "wfe_waiting[0] must remain false (Serial bus had it false)"
        );
    }

    /// Proof that the serial → threaded handoff carries
    /// `Bus::external_gpio_in_override` and `external_gpio_in_mask`
    /// forward. Without this transfer, harness pin injection
    /// (e.g. `picogus_diff_rp2040`) vanishes at promotion.
    #[test]
    fn from_emulator_preserves_external_gpio_override() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        emu.bus.external_gpio_in_override = 0xA5A5_0000;
        emu.bus.external_gpio_in_mask = 0xFFFF_0000;
        // Halt both cores so `run_quanta_checked` terminates cleanly
        // without running instructions (no program loaded).
        emu.cores[0].halt();
        emu.cores[1].halt();

        let mut threaded = ThreadedEmulator::from_emulator(emu);
        assert_eq!(
            threaded
                .shared
                .external_gpio_in_mask
                .load(Ordering::Acquire),
            0xFFFF_0000
        );
        assert_eq!(
            threaded
                .shared
                .external_gpio_in_override
                .load(Ordering::Acquire),
            0xA5A5_0000
        );

        // Run one quantum — both cores halted, coordinator merges GPIO.
        threaded
            .run_quanta_checked(1)
            .expect("run_quanta_checked(1)");

        // Read GPIO_IN through a fresh WorkerBus and confirm the
        // override bits win over the SIO merge (which is all zeros in
        // this fresh emulator).
        use crate::core::bus_trait::CoreBus;
        use crate::threaded::WorkerBus;
        let mut probe = WorkerBus::new(Arc::clone(&threaded.shared), 0);
        let gpio_in = probe.read32(0xD000_0004);
        assert_eq!(
            gpio_in & 0xFFFF_0000,
            0xA5A5_0000,
            "external override must survive from_emulator + one quantum"
        );
    }

    // ----- Builder + accessor coverage ----------------------------------

    /// `core_cycles(0|1)` returns the inner core's cycle counter while
    /// the cores are owned by `ThreadedEmulator` (i.e. between
    /// `run_quanta_checked` calls). Fresh emulator ⇒ both 0.
    #[test]
    fn core_cycles_returns_zero_for_fresh_emulator() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let threaded = ThreadedEmulator::from_emulator(emu);
        assert_eq!(threaded.core_cycles(0), 0);
        assert_eq!(threaded.core_cycles(1), 0);
    }

    /// `core_cycles` panics for any index other than 0 or 1.
    #[test]
    #[should_panic(expected = "idx must be 0 or 1")]
    fn core_cycles_panics_on_invalid_idx() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let threaded = ThreadedEmulator::from_emulator(emu);
        let _ = threaded.core_cycles(2);
    }

    // ----- run_quanta_checked drain-loop coverage -----------------------

    /// `run_quanta_checked(0)` is a no-op: workers spawn, take the n=0
    /// loop bound, return immediately. master_cycle stays put.
    #[test]
    fn run_quanta_zero_is_noop() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);
        let before = threaded.master_cycle();

        threaded
            .run_quanta_checked(0)
            .expect("run_quanta_checked(0) infallible");

        assert_eq!(
            threaded.master_cycle(),
            before,
            "n=0 must not advance master_cycle"
        );
    }

    /// One core halted, the other not: covers the
    /// `if !shared.atomics.is_halted(core_id)` branch in
    /// `core_worker_body` taking both true and false on the same run.
    /// Core 1 is halted by the builder default; core 0 is left running.
    /// Without a program loaded `core.is_halted()` becomes true after
    /// the first faulting fetch, so the inner step loop terminates
    /// quickly. The test simply asserts the call returns Ok.
    #[test]
    fn run_quanta_one_core_halted_other_runs() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        // Core 1 is halted by default (set in `EmulatorBuilder::build`).
        // Core 0 left running — its worker exercises the
        // `is_halted(0) == false` branch and the inner step loop, which
        // hardfaults out almost immediately on a no-firmware reset.
        threaded
            .run_quanta_checked(2)
            .expect("run_quanta_checked(2) with mixed halted state");

        let step_q = threaded.step_quantum as u64;
        assert_eq!(threaded.master_cycle(), 2 * step_q);
    }

    /// Re-entry after a successful run is allowed (non-poisoned path).
    /// Exercises the `Some(core)` restore arms at the tail of
    /// `run_quanta_checked`.
    #[test]
    fn run_quanta_repeat_invocations_restore_state() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        for _ in 0..3 {
            threaded
                .run_quanta_checked(1)
                .expect("each round restores cores + pio_blocks");
        }

        let step_q = threaded.step_quantum as u64;
        assert_eq!(threaded.master_cycle(), 3 * step_q);
    }

    // ----- WFE / SEV cross-core wake -----------------------------------

    /// SEV-on-WFE wake mirroring the rp2350_emu §11 item 15 test. Park
    /// core 0 on WFE then SEV both cores; after one quantum the core
    /// worker's top-of-loop hook must consume `event_flag` and clear
    /// `wfe_waiting`.
    #[test]
    fn wfe_sev_wake_clears_waiting_flag() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        // Halt core 1 so only core 0 exercises the WFE wake hook.
        threaded.shared.atomics.set_halted(1);
        threaded.shared.atomics.set_wfe_waiting(0);
        assert!(threaded.shared.atomics.is_wfe_waiting(0));

        // SEV both cores.
        threaded.shared.atomics.sev_both();
        threaded
            .run_quanta_checked(1)
            .expect("run_quanta_checked(1)");

        assert!(
            !threaded.shared.atomics.is_wfe_waiting(0),
            "WFE wake must clear wfe_waiting after SEV"
        );
        assert!(
            !threaded.shared.atomics.event_flag_load(0),
            "event_flag[0] must be consumed by the wake check"
        );
    }

    // ----- from_emulator state-carry coverage --------------------------

    /// Pre-promotion `Bus::irq_pending` is broadcast to both cores'
    /// `CoreAtomics::irq_pending` slots. Covers the `if bus.irq_pending
    /// != 0` branch on both iterations of the seed loop.
    #[test]
    fn from_emulator_broadcasts_irq_pending() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        emu.bus.irq_pending = 0b1010;
        emu.cores[0].halt();
        emu.cores[1].halt();

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert_eq!(threaded.shared.atomics.irq_pending_load(0), 0b1010);
        assert_eq!(threaded.shared.atomics.irq_pending_load(1), 0b1010);
    }

    /// `Bus::event_flag[c]` must lift into `CoreAtomics::event_flag[c]`
    /// at promotion. Covers both per-core branches of the loop.
    #[test]
    fn from_emulator_preserves_event_flag() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        emu.bus.event_flag[0] = true;
        emu.bus.event_flag[1] = false;
        emu.cores[0].halt();
        emu.cores[1].halt();

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert!(threaded.shared.atomics.event_flag_load(0));
        assert!(!threaded.shared.atomics.event_flag_load(1));
    }

    /// Halted-core promotion: when both serial cores are halted at
    /// handoff, both atomic `halted` slots must be set so the worker
    /// loop skips its inner step loop.
    #[test]
    fn from_emulator_carries_halted_flags() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        emu.cores[0].halt();
        emu.cores[1].halt();

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert!(threaded.shared.atomics.is_halted(0));
        assert!(threaded.shared.atomics.is_halted(1));
    }

    /// PIO `sm_enabled_mask` round-trips through promotion via
    /// `ThreadedPio::publish_sm_enabled`. Drive a non-default mask onto
    /// PIO0 / PIO1 via the serial bus before handoff and verify the
    /// shared snapshot mirrors it.
    #[test]
    fn from_emulator_seeds_pio_sm_enabled() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        // Write CTRL.SM_ENABLE = 0b0011 onto PIO0 (base 0x5020_0000)
        // and 0b0100 onto PIO1 (base 0x5030_0000) via serial MMIO.
        emu.bus.write32(0x5020_0000, 0b0011);
        emu.bus.write32(0x5030_0000, 0b0100);
        emu.cores[0].halt();
        emu.cores[1].halt();

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert_eq!(threaded.shared.pio.sm_enabled(0), 0b0011);
        assert_eq!(threaded.shared.pio.sm_enabled(1), 0b0100);
    }

    /// `master_cycle` initial value is `max(clock.cycles, bus.master_cycle)`.
    /// Force a non-zero `clock.cycles` and verify the threaded counter
    /// starts there.
    #[test]
    fn from_emulator_seeds_master_cycle_from_clock() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        emu.clock.cycles = 12_345;
        emu.cores[0].halt();
        emu.cores[1].halt();

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert_eq!(threaded.master_cycle(), 12_345);
    }

    // ----- Poison / re-entry guards ------------------------------------

    /// After a panic-induced poisoning, calling `run_quanta_checked`
    /// must trip the leading `assert!(!self.poisoned)` panic.
    #[cfg(feature = "testing")]
    #[test]
    #[should_panic(expected = "poisoned by prior worker panic")]
    fn run_quanta_after_poison_panics() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.inject_panic_for_testing(WorkerName::Core0);
        // First call panics core 0 and sets `self.poisoned = true`.
        let _ = threaded.run_quanta_checked(1);
        assert!(threaded.poisoned, "first call must poison");
        // Second call must trip the assert.
        let _ = threaded.run_quanta_checked(1);
    }

    /// `inject_panic_for_testing(Core1)` ⇒ core 1 worker panics and
    /// `RunError::Panic{Core1, ..}` is returned.
    #[cfg(feature = "testing")]
    #[test]
    fn run_quanta_core1_panic_surfaces() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.inject_panic_for_testing(WorkerName::Core1);

        let result = threaded.run_quanta_checked(1);
        match result {
            Err(RunError::Panic {
                which: WorkerName::Core1,
                ref message,
            }) => {
                assert!(
                    message.contains("core1"),
                    "message should name core1: {message}"
                );
            }
            other => panic!("expected Err(RunError::Panic{{Core1,...}}), got {other:?}"),
        }
        assert!(threaded.poisoned);
        // shared.panic_info must mirror.
        let info = threaded.shared.panic_info.lock().unwrap();
        let (which, msg) = info.as_ref().expect("panic_info must be populated");
        assert_eq!(*which, WorkerName::Core1);
        assert!(msg.contains("core1"));
    }

    /// `inject_panic_for_testing(Coord)` ⇒ coordinator worker panics
    /// and `RunError::Panic{Coord, ..}` is returned. Also covers the
    /// `pio_blocks = None` branch on the coord-panic path.
    #[cfg(feature = "testing")]
    #[test]
    fn run_quanta_coord_panic_surfaces() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.inject_panic_for_testing(WorkerName::Coord);

        let result = threaded.run_quanta_checked(1);
        match result {
            Err(RunError::Panic {
                which: WorkerName::Coord,
                ref message,
            }) => {
                assert!(
                    message.contains("coord"),
                    "message should name coord: {message}"
                );
            }
            other => panic!("expected Err(RunError::Panic{{Coord,...}}), got {other:?}"),
        }
        assert!(threaded.poisoned);
        assert!(
            threaded.pio_blocks.is_none(),
            "coordinator panic must drop pio_blocks"
        );
        assert!(
            threaded
                .shared
                .poisoned
                .load(std::sync::atomic::Ordering::Acquire),
            "shared.poisoned must mirror"
        );
    }

    // ----- Coordinator branches: PIO step + TIMER IRQ + GPIO merge -----

    /// Coordinator GPIO merge applies the SIO `gpio_out & gpio_oe` mask
    /// per quantum. Seed both atomics, run a quantum with cores halted,
    /// and verify the post-merge `gpio_out` is the `out & oe` mask
    /// (lines 720-739 in `coordinator_worker_body`).
    #[test]
    fn coordinator_merges_sio_gpio_mask() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        threaded.shared.gpio_out.store(0xFFFF, Ordering::Release);
        threaded.shared.gpio_oe.store(0x00FF, Ordering::Release);

        threaded
            .run_quanta_checked(1)
            .expect("run_quanta_checked(1)");

        // After merge: out = out & oe, oe = oe. PIO contributions are
        // zero (no SM enabled), so the result is the SIO mask.
        assert_eq!(threaded.shared.gpio_out.load(Ordering::Acquire), 0x00FF);
        assert_eq!(threaded.shared.gpio_oe.load(Ordering::Acquire), 0x00FF);
    }

    // ----- Helper coverage: apply_pio_command ---------------------------
    // `panic_message_extracts_all_payload_kinds` moved with `panic_message`
    // to `picoem-common::threaded::worker::tests` per the 2026-04-30
    // Threaded Helpers Pull-Up HLD V1 §5.4.

    /// `apply_pio_command` covers all four `PioCommand` arms. We feed
    /// each one into a fresh `PioBlock` and verify a non-default
    /// observable byte changed afterwards. The exact register layout
    /// is the contract's responsibility — this test only proves we
    /// hit each match arm.
    #[test]
    fn apply_pio_command_dispatches_all_variants() {
        let mut block = PioBlock::new();

        // CTRL: SM_ENABLE = 0b0001.
        apply_pio_command(
            &mut block,
            PioCommand::WriteCtrl {
                block: 0,
                val: 0b0001,
                alias: 0,
            },
        );
        assert_eq!(block.sm_enabled_mask(), 0b0001);

        // INSTR_MEM[5] = 0xABCD.
        apply_pio_command(
            &mut block,
            PioCommand::WriteInstrMem {
                block: 0,
                addr: 5,
                value: 0xABCD,
                alias: 0,
            },
        );
        // INSTR_MEM is write-only via MMIO, so use the test accessor.
        assert_eq!(block.instr_mem()[5], 0xABCD);

        // INSTR_MEM out-of-range (addr >= 32) is a no-op — covers the
        // `if addr < 32` false arm.
        apply_pio_command(
            &mut block,
            PioCommand::WriteInstrMem {
                block: 0,
                addr: 99,
                value: 0xDEAD,
                alias: 0,
            },
        );

        // SetClkDiv on SM 1.
        apply_pio_command(
            &mut block,
            PioCommand::SetClkDiv {
                block: 0,
                sm: 1,
                int_div: 0x1234,
                frac_div: 0x56,
                alias: 0,
            },
        );
        // SM1_CLKDIV at 0x0C8 + 1*0x18 = 0x0E0.
        let expected = (0x1234u32 << 16) | (0x56u32 << 8);
        assert_eq!(block.read32(0x0E0), expected);

        // SetClkDiv with sm >= 4 is a no-op (covers `if sm < 4` false).
        apply_pio_command(
            &mut block,
            PioCommand::SetClkDiv {
                block: 0,
                sm: 7,
                int_div: 0xFFFF,
                frac_div: 0xFF,
                alias: 0,
            },
        );

        // Generic WriteReg: hit FDEBUG @ 0x008.
        apply_pio_command(
            &mut block,
            PioCommand::WriteReg {
                block: 0,
                offset: 0x008,
                val: 0xFFFF_FFFF,
                alias: 0,
            },
        );
        // FDEBUG is W1C — read-back returns 0 after writing 1s.
        // Just exercise the dispatch; no assertion needed beyond
        // "didn't panic".
        let _ = block.read32(0x008);
    }

    // ----- Drop / cleanup -----------------------------------------------

    /// Drop without an explicit run — covers the no-active-workers
    /// drop path. Must not panic, must not leak Arcs.
    #[test]
    fn drop_without_running_quanta() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let threaded = ThreadedEmulator::from_emulator(emu);
        let shared = Arc::clone(threaded.shared());
        let pre_strong = Arc::strong_count(&shared);
        drop(threaded);
        // Strong count must drop by at least 1 once the emulator
        // releases its handle.
        let post_strong = Arc::strong_count(&shared);
        assert!(
            post_strong < pre_strong,
            "drop must release the emulator's Arc handle: {pre_strong} -> {post_strong}"
        );
    }

    /// Drop after a successful run cycle — covers the post-run drop
    /// path where cores + pio_blocks have been restored.
    #[test]
    fn drop_after_successful_run() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);
        threaded
            .run_quanta_checked(1)
            .expect("run_quanta_checked(1)");
        // Implicit drop here.
    }

    /// Drop after a panic-poisoned run — covers the drop path where
    /// `pio_blocks` may be `None` (coord panic) or `Some` (core panic).
    #[cfg(feature = "testing")]
    #[test]
    fn drop_after_poisoned_run() {
        let emu = EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build infallible");
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.inject_panic_for_testing(WorkerName::Coord);
        let _ = threaded.run_quanta_checked(1);
        // Implicit drop — must not double-fault on `pio_blocks = None`.
    }
}

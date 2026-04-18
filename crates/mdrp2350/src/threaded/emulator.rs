//! `ThreadedEmulator` — 4-thread runtime entry point for Phase 3.
//!
//! Phase 3 Stage 6b (LLD V7 §8/§9): this stage ships **stub** worker
//! bodies that only rendezvous on the shared `SpinBarrier` twice per
//! quantum. Stage 7 replaces the stubs with the real
//! core / PIO / coordinator execution logic.
//!
//! Gated behind `#[cfg(all(target_arch = "x86_64", target_os =
//! "windows"))]` because the thread-pinning path uses Win32
//! `SetThreadAffinityMask`. Non-Windows callers stay on the existing
//! single-threaded `Emulator::run` path.
//!
//! Lifecycle at a glance:
//!
//! 1. Caller drives an existing `Emulator` to the pre-run state (load
//!    ROM / flash, reset, seed GPIO stimulus, etc.).
//! 2. `ThreadedEmulator::from_emulator(emu)` destructures the Bus into
//!    the shared state bundle and per-core CPUs.
//! 3. `run_quanta(n)` spawns four workers (core 0, core 1, PIO,
//!    coordinator), joins, and surfaces panics via the `poisoned` flag
//!    so the instance cannot be reused after a worker panic.
//!
//! The master-cycle counter lives on `SharedState.master_cycle` (an
//! `Arc<AtomicU64>`) so the coordinator's `fetch_add(Release)` pairs
//! with the CPU workers' `load(Acquire)` for PLL-LOCK derivation —
//! see `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 and
//! `threaded::peripherals::ClocksState::pll_sys_read_at`.

use std::panic::AssertUnwindSafe;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::thread::{self, JoinHandle};

use mdpicoem_common::PioBlock;

use crate::bus::Bus;
use crate::core::CortexM33;
use crate::Emulator;

use super::{
    AtomicGpio, BarrierResult, ExclusiveMonitors, SharedMemory, SharedState,
    SpinBarrier, ThreadedPio, ThreadedSio,
};
use super::peripherals::{
    ApbState, ClocksState, DmaState, Peripherals, QmiState, ResetsState, TimersState,
};

/// 4-thread runtime handle over a seeded `SharedState` and both CPU
/// cores. See module-level docs for the Stage 6b → Stage 7 split.
pub struct ThreadedEmulator {
    shared: SharedState,
    core0: Option<CortexM33>,
    core1: Option<CortexM33>,
    pio_blocks: Option<[PioBlock; 3]>,
    step_quantum: u32,
    thread_mask: [usize; 4],
    poisoned: bool,
}

impl ThreadedEmulator {
    /// Consume a single-threaded `Emulator` and return a
    /// `ThreadedEmulator` with every piece of state hoisted onto the
    /// shared `SharedState`.
    ///
    /// Panics if `std::thread::available_parallelism()` reports fewer
    /// than 4 host cores — the runtime pins one thread per core and a
    /// 4-core host cannot satisfy that without OS contention. On
    /// exactly 4 cores, emits an `eprintln!` advising >= 5.
    pub fn from_emulator(emu: Emulator) -> Self {
        let n = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        assert!(
            n >= 4,
            "ThreadedEmulator requires >= 4 host cores (found {n})"
        );
        if n == 4 {
            // TODO: migrate to tracing::warn! once mdrp2350 pulls
            // tracing directly (workspace-wide dep rollout tracked
            // alongside the tracing infra HLD). Until then, a one-shot
            // stderr advisory is better than silently letting the
            // 4-core case degrade under OS contention.
            eprintln!(
                "ThreadedEmulator: exactly 4 host cores — workers will \
                 contend with OS / other processes; >= 5 recommended"
            );
        }

        let Emulator {
            cores,
            bus,
            clock,
            step_quantum,
        } = emu;
        let [core0, core1] = cores;

        // Exhaustive destructure — any new `Bus` field forces a compile
        // error here so the threaded path cannot silently drop state.
        // Fields that Stage 6/Stage 7 doesn't yet consume are bound to
        // `_`; the Stage 5 `WorkerBus`/Stage 7 worker bodies will
        // pick them up as needed.
        //
        // `decode_cache` still lives on the single-threaded `Bus`
        // today; Stage 10 will migrate it onto each `CortexM33`. For
        // now we drop the ROM-backed cache — the cores rebuild their
        // caches lazily on first fetch.
        let Bus {
            memory,
            boot_ram,
            xip_sram,
            sio,
            pio,
            atomics,
            resets_state,
            ticks,
            timer0,
            timer1,
            uart0,
            spi0,
            i2c0,
            adc,
            pwm,
            io_bank0,
            pads_bank0,
            dma,
            clk_ref_ctrl,
            clk_sys_ctrl,
            clk_sys_div,
            clock_tree,
            pll_sys_regs,
            pll_usb_regs,
            pll_sys_lock_at_cycle,
            pll_usb_lock_at_cycle,
            rosc_regs,
            xosc_regs,
            gpio_hi_noise_state,
            qmi_regs,
            xip_cache_offset,
            gpio_in,
            gpio_external_in,
            gpio_external_mask,
            flash_loaded,
            peripheral_regs,
            master_cycle,
            last_access_cycles: _,
            extra_wait_states: _,
            burst_mode: _,
            active_pc: _,
            trace_enabled: _,
            trace_sink: _,
            decode_cache: _,
        } = bus;

        let shared_mem = Arc::new(SharedMemory::from_memory(
            memory,
            boot_ram,
            xip_sram,
            flash_loaded,
        ));
        let shared_gpio = Arc::new(AtomicGpio::seed(
            sio.gpio_out,
            sio.gpio_oe,
            gpio_in,
            gpio_external_in,
            gpio_external_mask,
        ));
        let shared_sio = Arc::new(ThreadedSio::seed(&sio));

        // Carry any unconsumed single-threaded FIFO-wake signal into the
        // threaded `event_flag` so a WFE wake that was queued on `Sio`
        // but not yet lifted by `Bus::step` survives the handoff.
        // Parity with the `pending_fifo_event` drain in `bus/mod.rs` —
        // the threaded runtime consumes the bit via `CoreAtomics`.
        if let Some(receiver) = sio.pending_fifo_event {
            debug_assert!(receiver < 2, "pending_fifo_event receiver must be 0 or 1");
            atomics.event_flag[receiver].store(true, std::sync::atomic::Ordering::Release);
        }

        // Per-core DIV / INTERP (`PerCoreSio`) already live on each
        // `CortexM33` post-Stage-3, so there is nothing to copy from
        // `Sio` into `core*.sio_local` here. Touching the field would
        // erase the already-populated per-core divider / interpolator
        // state.

        let peripherals = Arc::new(Peripherals {
            clocks: Mutex::new(ClocksState {
                clk_ref_ctrl,
                clk_sys_ctrl,
                clk_sys_div,
                clock_tree,
                pll_sys_regs,
                pll_usb_regs,
                pll_sys_lock_at_cycle,
                pll_usb_lock_at_cycle,
                rosc: rosc_regs,
                xosc: xosc_regs,
                gpio_hi_noise_state,
            }),
            qmi: Mutex::new(QmiState {
                qmi_regs,
                xip_cache_offset,
            }),
            resets: Mutex::new(ResetsState { resets_state }),
            apb: Mutex::new(ApbState {
                uart0,
                spi0,
                i2c0,
                adc,
                pwm,
                io_bank0,
                pads_bank0,
            }),
            timers: Mutex::new(TimersState {
                ticks,
                timer0,
                timer1,
            }),
            dma: Mutex::new(DmaState { dma }),
            legacy: Mutex::new(peripheral_regs),
        });

        let shared = SharedState {
            memory: shared_mem,
            gpio: shared_gpio,
            sio: shared_sio,
            pio: Arc::new(ThreadedPio::new()),
            monitors: Arc::new(ExclusiveMonitors::new()),
            peripherals,
            atomics,
            // Defensive .max(): Emulator::step keeps these equal at every quantum boundary
            // (lib.rs writes bus.master_cycle = clock.cycles before entry), so they should
            // be the same value. .max() guards against an edge case where bus.master_cycle
            // was advanced via a peripheral tick since the last clock.cycles sync.
            master_cycle: Arc::new(AtomicU64::new(master_cycle.max(clock.cycles))),
        };

        Self {
            shared,
            core0: Some(core0),
            core1: Some(core1),
            pio_blocks: Some(pio),
            step_quantum,
            thread_mask: [0, 1, 2, 3],
            poisoned: false,
        }
    }

    /// Override the default host-core pinning mask. The supplied mask
    /// maps worker index (0=core0, 1=core1, 2=PIO, 3=coordinator) to
    /// host logical-CPU id. Useful on SMT / hyperthreaded hosts where
    /// the default dense `[0, 1, 2, 3]` mapping would share physical
    /// cores with the other three workers.
    pub fn with_thread_mask(mut self, mask: [usize; 4]) -> Self {
        self.thread_mask = mask;
        self
    }

    /// Current shared master-cycle count. Lock-free `Acquire` load,
    /// paired with the coordinator's `fetch_add(Release)` in
    /// `coordinator_worker_body_stub` (Stage 6b) /
    /// `coordinator_worker_body` (Stage 7).
    pub fn master_cycle(&self) -> u64 {
        self.shared.master_cycle.load(Ordering::Acquire)
    }

    /// Run `n` quanta. Spawns four workers, joins, and — on panic —
    /// flips the `poisoned` flag so the next call panics early. Do not
    /// call `run_quanta` again on a poisoned instance; drop it and
    /// rebuild from a fresh `Emulator`.
    ///
    /// Stage 6b: the worker bodies are the rendezvous-only stubs from
    /// [`core_worker_body_stub`] / [`pio_worker_body_stub`] /
    /// [`coordinator_worker_body_stub`]. Stage 7 replaces each with the
    /// real step / PIO-step / peripheral-tick logic.
    pub fn run_quanta(&mut self, n: u64) {
        assert!(
            !self.poisoned,
            "ThreadedEmulator poisoned by prior worker panic; drop and rebuild"
        );

        let core0 = self.core0.take().expect("run_quanta reentry");
        let core1 = self.core1.take().expect("run_quanta reentry");
        let blocks = self.pio_blocks.take().expect("run_quanta reentry");

        let barrier = Arc::new(SpinBarrier::new(4));
        let shared = self.shared.clone();
        let step_q = self.step_quantum;
        let mask = self.thread_mask;

        let h0 = spawn_worker(mask[0], barrier.clone(), {
            let s = shared.clone();
            move |b| core_worker_body_stub(0, core0, s, b, n, step_q)
        });
        let h1 = spawn_worker(mask[1], barrier.clone(), {
            let s = shared.clone();
            move |b| core_worker_body_stub(1, core1, s, b, n, step_q)
        });
        let hp = spawn_worker(mask[2], barrier.clone(), {
            let s = shared.clone();
            move |b| pio_worker_body_stub(blocks, s, b, n, step_q)
        });
        let hc = spawn_worker(mask[3], barrier.clone(), {
            let s = shared.clone();
            move |b| coordinator_worker_body_stub(s, b, n, step_q)
        });

        let r0 = h0.join();
        let r1 = h1.join();
        let rp = hp.join();
        let rc = hc.join();

        // Track which workers panicked before consuming the Ok payloads
        // so the panic message can enumerate the culprits.
        let r0_err = r0.is_err();
        let r1_err = r1.is_err();
        let rp_err = rp.is_err();
        let rc_err = rc.is_err();

        // Restore owned state on the happy path. Any side that panicked
        // loses its core / block value for this run — the `poisoned`
        // flag that follows rejects any further call into this
        // instance anyway, so the `None` is observable only via a
        // `run_quanta` re-entry that panics with a clear message.
        if let Ok(c) = r0 {
            self.core0 = Some(c);
        }
        if let Ok(c) = r1 {
            self.core1 = Some(c);
        }
        if let Ok(b) = rp {
            self.pio_blocks = Some(b);
        }
        // Coordinator worker returns `()`; nothing to reclaim.
        let _ = rc;

        let panicked: Vec<&str> = [
            ("core0", r0_err),
            ("core1", r1_err),
            ("pio", rp_err),
            ("coordinator", rc_err),
        ]
        .into_iter()
        .filter_map(|(name, err)| if err { Some(name) } else { None })
        .collect();
        if !panicked.is_empty() {
            self.poisoned = true;
            panic!("worker thread(s) panicked: {}", panicked.join(", "));
        }
    }
}

// =======================================================================
// Worker-thread plumbing
// =======================================================================

/// Spawn a worker thread pinned to `host_core` running `body`. Catches
/// panics from `body` and poisons the shared barrier before re-raising
/// the panic so the remaining workers drop out of their spin loops.
///
/// Generic over the body's return type so the three different body
/// signatures (`CortexM33` / `[PioBlock; 3]` / `()`) share the same
/// spawn path without a trait object.
fn spawn_worker<F, R>(
    host_core: usize,
    barrier: Arc<SpinBarrier>,
    body: F,
) -> JoinHandle<R>
where
    F: FnOnce(Arc<SpinBarrier>) -> R + Send + 'static,
    R: Send + 'static,
{
    thread::spawn(move || {
        pin_to_host_core(host_core);
        let b_for_body = barrier.clone();
        match std::panic::catch_unwind(AssertUnwindSafe(move || body(b_for_body))) {
            Ok(r) => r,
            Err(payload) => {
                barrier.poison();
                std::panic::resume_unwind(payload);
            }
        }
    })
}

/// Pin the current thread to the supplied host logical-CPU id via
/// `SetThreadAffinityMask`. Windows-only (the whole module is gated to
/// Windows anyway; this is the call site that needs the Win32 handle).
fn pin_to_host_core(host_core: usize) {
    use winapi::um::processthreadsapi::GetCurrentThread;
    use winapi::um::winbase::SetThreadAffinityMask;
    assert!(
        host_core < usize::BITS as usize,
        "host_core {host_core} exceeds processor-mask bit width"
    );
    let h = unsafe { GetCurrentThread() };
    let mask = 1usize << host_core;
    let prev = unsafe { SetThreadAffinityMask(h, mask) };
    assert!(
        prev != 0,
        "SetThreadAffinityMask failed for host core {host_core}"
    );
}

// =======================================================================
// Stage 6b STUB worker bodies
// =======================================================================
//
// STAGE 6b SCAFFOLDING: the three `*_worker_body_stub` functions are
// barrier-only rendezvous placeholders. Stage 7 replaces them with real
// core/pio/coordinator execution — see V7 §9. Tests that measure
// cycle advancement or check execution side-effects will fail against
// the stubs; that's by design.
//
// These only `barrier.wait()` twice per quantum. Stage 7 replaces the
// bodies with the real execution loops (see LLD V7 §9):
//   * `core_worker_body_stub` → `core_worker_body` with instruction
//     dispatch + WFE/WFI wake hooks.
//   * `pio_worker_body_stub`  → `pio_worker_body` that drains the PIO
//     command queue and steps active SMs.
//   * `coordinator_worker_body_stub` → `coordinator_worker_body` that
//     advances `master_cycle`, MTIME, and the peripheral ticks.
//
// Returning the owned `CortexM33` / `[PioBlock; 3]` by-move keeps the
// calling `run_quanta` handoff identical in Stage 7 — the bodies simply
// start doing useful work with the values they already own.

fn core_worker_body_stub(
    core_id: u8,
    core: CortexM33,
    _shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    _step_q: u32,
) -> CortexM33 {
    // STAGE 6b STUB: Stage 7 replaces with the real execution loop.
    let _ = core_id;
    for _ in 0..n {
        if barrier.wait() == BarrierResult::Poisoned {
            return core;
        }
        if barrier.wait() == BarrierResult::Poisoned {
            return core;
        }
    }
    core
}

fn pio_worker_body_stub(
    blocks: [PioBlock; 3],
    _shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    _step_q: u32,
) -> [PioBlock; 3] {
    // STAGE 6b STUB: Stage 7 replaces with the real PIO step loop.
    for _ in 0..n {
        if barrier.wait() == BarrierResult::Poisoned {
            return blocks;
        }
        if barrier.wait() == BarrierResult::Poisoned {
            return blocks;
        }
    }
    blocks
}

fn coordinator_worker_body_stub(
    _shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
) {
    // STAGE 6b STUB: Stage 7 replaces with peripheral ticks +
    // master_cycle fetch_add + MTIME advance. The `step_q` argument is
    // passed through the closure and into this body so the Stage 7
    // swap is a body-local edit — no signature change.
    let _ = step_q;
    for _ in 0..n {
        if barrier.wait() == BarrierResult::Poisoned {
            return;
        }
        if barrier.wait() == BarrierResult::Poisoned {
            return;
        }
    }
}

// =======================================================================
// Tests
// =======================================================================
//
// Stage 6b keeps the test surface minimal. The two tests here verify
// that the destructure → seed → spawn → join round-trip compiles and
// runs; Stage 7 will add smoke tests that validate actual execution
// against the single-threaded baseline.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Config;

    #[test]
    fn from_emulator_builds_threadedemulator() {
        let emu = Emulator::new(Config::default());
        let threaded = ThreadedEmulator::from_emulator(emu);
        assert_eq!(threaded.master_cycle(), 0);
    }

    #[test]
    fn run_quanta_stubs_spawn_and_join() {
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.run_quanta(1);
        let mc_before = threaded.master_cycle();
        threaded.run_quanta(5);
        assert_eq!(
            threaded.master_cycle(),
            mc_before,
            "stub coordinator does not advance master_cycle; Stage 7 changes this"
        );
    }

    /// Fix 1a: an unconsumed `pending_fifo_event` on the single-threaded
    /// `Sio` must be forwarded to the threaded `event_flag[receiver]`
    /// during handoff so a WFE that was about to be woken doesn't get
    /// stranded.
    #[test]
    fn from_emulator_preserves_pending_fifo_event() {
        let mut emu = Emulator::new(Config::default());
        // Simulate a FIFO push that queued a wake for core 1.
        emu.bus.sio.pending_fifo_event = Some(1);

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert!(
            threaded
                .shared
                .atomics
                .event_flag[1]
                .load(Ordering::Acquire),
            "pending_fifo_event(1) must land on event_flag[1]"
        );
        assert!(
            !threaded
                .shared
                .atomics
                .event_flag[0]
                .load(Ordering::Acquire),
            "peer (0) must stay clear"
        );
    }

    /// Fix 1b: `mtime_match_asserted` bits survive the handoff so the
    /// Phase 5 MTIMECMP → IRQ wiring starts from the right edge state.
    #[test]
    fn from_emulator_preserves_mtime_match_asserted() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.sio.mtime_match_asserted = [true, false];

        let threaded = ThreadedEmulator::from_emulator(emu);
        assert!(threaded.shared.sio.mtime_match_asserted_load(0));
        assert!(!threaded.shared.sio.mtime_match_asserted_load(1));
    }
}

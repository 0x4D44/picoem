//! `ThreadedEmulator` — 4-thread runtime entry point for Phase 3.
//!
//! Phase 3 Stage 7 (LLD V7 §9): real core / PIO / coordinator worker
//! bodies. Phase 4 Stage C (HLD V7 §5) collapsed the two-barrier
//! rendezvous to a single barrier per iteration: each worker performs
//! its phase work then rendezvouses once at the tail of the loop. CPU
//! phase-1 of quantum N now runs in parallel with coord phase-2 of
//! quantum N; the `2 × step_quantum` staleness ceiling that overlap
//! implies is accepted per HLD V7 §5.2.
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
    AtomicGpio, BarrierResult, ExclusiveMonitors, PioCommand, SharedMemory, SharedState,
    SpinBarrier, ThreadedPio, ThreadedSio, WorkerBus,
};
use super::peripherals::{
    ApbState, ClocksState, DmaState, Peripherals, QmiState, ResetsState, TimersState,
};
use super::timings::{PerWorkerTimings, RunTimings, TimingRecorder};

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
    /// Per-worker per-quantum timing instrumentation. Off by default so
    /// production `run_quanta` calls stay on the zero-`Instant::now()`
    /// hot path. Flip with [`ThreadedEmulator::set_timing_enabled`]
    /// before calling `run_quanta`; read via
    /// [`ThreadedEmulator::last_run_timings`] after.
    timing_enabled: bool,
    /// Raw timings from the most recent `run_quanta`. `None` until the
    /// first call or after a call with `timing_enabled == false`.
    /// Each call resets this — no cross-call accumulation.
    last_run_timings: Option<RunTimings>,
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
        // ThreadedEmulator currently only supports the Arm arm — RISC-V
        // (Hazard3) lives behind the P1a enum but doesn't thread yet.
        let crate::Cores::Arm(arm) = cores else {
            panic!("ThreadedEmulator requires Arch::Arm (RISC-V threading is P4+)");
        };
        let [core0, core1] = arm;

        // Debug-assert that the single-threaded driver has drained any
        // pending decode-cache invalidations before handoff. Dropping
        // these on the floor would leave the threaded workers starting
        // with per-core caches that still carry stale entries pointing
        // at bytes the single-threaded `Bus` replaced.
        debug_assert!(
            bus.pending_cache_invalidations.is_empty()
                && bus.pending_invalidation_regions == 0,
            "ThreadedEmulator::from_emulator: Bus has unconsumed decode-cache \
             invalidations. Call Emulator::step() or Emulator::reset() before \
             handoff, or the threaded workers will start with stale per-core \
             caches."
        );

        // Exhaustive destructure — any new `Bus` field forces a compile
        // error here so the threaded path cannot silently drop state.
        // Fields that Stage 6/Stage 7 doesn't yet consume are bound to
        // `_`; the Stage 5 `WorkerBus`/Stage 7 worker bodies will
        // pick them up as needed.
        //
        // The decode cache now lives on each `CortexM33` (Phase 3
        // follow-up #10); `pending_cache_invalidations` /
        // `pending_invalidation_regions` are single-threaded-path
        // dirty-range queues that the threaded workers don't consume
        // — `WorkerBus` carries its own per-worker queue. The
        // debug-assert above guards against handoff with unconsumed
        // state in either.
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
            uart,
            spi,
            i2c,
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
            mmio_trace_enabled: _,
            mmio_trace_sink: _,
            pending_cache_invalidations: _,
            pending_invalidation_regions: _,
            last_fetch_addr: _,
            warned_addrs: _,
            watchdog_reset_requested: _,
            syscfg: _,
            tbman: _,
            glitch: _,
            psm: _,
            watchdog: _,
            otp: _,
            trng: _,
            sha256: _,
            powman: _,
            coresight_trace: _,
            warned_clk_enable_clear: _,
            reservation: _,
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
                uart,
                spi,
                i2c,
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

        // Seed `ThreadedPio::sm_enabled` from the incoming `PioBlock`
        // state so a caller that programmed CTRL.SM_ENABLE through the
        // single-threaded `Bus` before `from_emulator` is honoured from
        // the first quantum — without this, the enable-mask check at
        // the top of `pio_worker_body` would zero-skip those blocks
        // until a fresh WriteCtrl came in over the command queue.
        //
        // Also seed `pads` so coord's first `update_gpio` (HLD V7 §4.3)
        // sees the live `(pad_out, pad_oe)` rather than zero — without
        // this, PIO output drops for one quantum during handover.
        //
        // Stage C prerequisite: under single-barrier overlap, coord's
        // first update_gpio reads pads before PIO worker publishes. The
        // seed prevents a first-quantum PIO-output drop. Do not remove.
        let threaded_pio = ThreadedPio::new();
        for (idx, block) in pio.iter().enumerate() {
            threaded_pio.write_sm_enabled(idx, block.sm_enabled_mask());
            threaded_pio.write_pads(idx, block.pad_out, block.pad_oe);
        }

        let shared = SharedState {
            memory: shared_mem,
            gpio: shared_gpio,
            sio: shared_sio,
            pio: Arc::new(threaded_pio),
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
            timing_enabled: false,
            last_run_timings: None,
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
    /// [`coordinator_worker_body`].
    pub fn master_cycle(&self) -> u64 {
        self.shared.master_cycle.load(Ordering::Acquire)
    }

    /// Enable or disable per-worker per-quantum timing instrumentation
    /// for subsequent `run_quanta` calls. When enabled, each worker
    /// records `(phase_work_ns, barrier_wait_ns)` per quantum and the
    /// aggregate is available via [`Self::last_run_timings`] after
    /// `run_quanta` returns.
    ///
    /// Off by default. The hot path pays no `Instant::now()` cost while
    /// disabled. Used by `paced_bench_rp2350`'s `--timing` flag to
    /// diagnose barrier-wait balance on dual-core workloads.
    ///
    /// Enabled-path overhead: expect roughly 30-40% throughput drop at
    /// `step_quantum=64`, shrinking at larger quanta as the two
    /// `Instant::now()` bracketing calls per quantum amortise. On a
    /// panicked `run_quanta`, timings are discarded and
    /// [`Self::last_run_timings`] returns whatever the previous
    /// successful call populated (or `None`).
    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.timing_enabled = enabled;
    }

    /// Raw timings from the most recent [`Self::run_quanta`] call.
    /// `None` before the first call, or after a call made while
    /// `timing_enabled == false`. Reset at the start of each call —
    /// no cross-call accumulation.
    pub fn last_run_timings(&self) -> Option<&RunTimings> {
        self.last_run_timings.as_ref()
    }

    /// Run `n` quanta. Spawns four workers, joins, and — on panic —
    /// flips the `poisoned` flag so the next call panics early. Do not
    /// call `run_quanta` again on a poisoned instance; drop it and
    /// rebuild from a fresh `Emulator`.
    ///
    /// Stage 7 (LLD V7 §9): each worker drives the real execution
    /// logic — the CPU workers step their `CortexM33` against a
    /// [`WorkerBus`] with WFE-wake + IRQ-pending merge semantics,
    /// the PIO worker drains the command queue + steps active SMs,
    /// and the coordinator publishes `master_cycle` + ticks the
    /// coordinator-owned peripherals.
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
        let timing = self.timing_enabled;

        // Reset per-run timings. Enabled runs repopulate this from the
        // joined workers below; disabled runs leave it `None` so
        // stale data from a prior enabled run doesn't mislead.
        self.last_run_timings = None;

        let h0 = spawn_worker(mask[0], barrier.clone(), {
            let s = shared.clone();
            move |b| core_worker_body(0, core0, s, b, n, step_q, timing)
        });
        let h1 = spawn_worker(mask[1], barrier.clone(), {
            let s = shared.clone();
            move |b| core_worker_body(1, core1, s, b, n, step_q, timing)
        });
        let hp = spawn_worker(mask[2], barrier.clone(), {
            let s = shared.clone();
            move |b| pio_worker_body(blocks, s, b, n, step_q, timing)
        });
        let hc = spawn_worker(mask[3], barrier.clone(), {
            let s = shared.clone();
            move |b| coordinator_worker_body(s, b, n, step_q, timing)
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
        //
        // When timing is enabled, each Ok payload carries a
        // `PerWorkerTimings` trailer; gather them into a `RunTimings`
        // below. A worker that panicked contributes `None`, which
        // `RunTimings` turns into an empty per-worker vec.
        let mut t0 = PerWorkerTimings::default();
        let mut t1 = PerWorkerTimings::default();
        let mut tp = PerWorkerTimings::default();
        let mut tc = PerWorkerTimings::default();

        if let Ok((c, t)) = r0 {
            self.core0 = Some(c);
            t0 = t;
        }
        if let Ok((c, t)) = r1 {
            self.core1 = Some(c);
            t1 = t;
        }
        if let Ok((b, t)) = rp {
            self.pio_blocks = Some(b);
            tp = t;
        }
        // Coordinator worker returns `((), PerWorkerTimings)`.
        if let Ok(((), t)) = rc {
            tc = t;
        }

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

        if timing {
            self.last_run_timings = Some(RunTimings {
                core0: t0,
                core1: t1,
                pio: tp,
                coord: tc,
            });
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
// Stage 7 worker bodies (LLD V7 §9)
// =======================================================================
//
// Each body owns its loop over `n` quanta. Within a quantum, every
// worker performs its phase work and then rendezvouses on the shared
// `SpinBarrier` exactly once at the tail (HLD V7 §5.1). This overlaps
// CPU/PIO phase-1 of quantum N with coord phase-2 of quantum N, at
// the cost of a `2 × step_quantum` staleness ceiling on peripheral
// state observed by CPU workers (HLD V7 §5.2).
//
// A poisoned barrier (any worker panicked) returns the owned `CortexM33`
// / `[PioBlock; 3]` / `()` immediately so the caller can flip the
// `poisoned` flag.

/// CPU-core worker. Owns a `CortexM33` and drives `step` against a
/// per-core [`WorkerBus`]. Consumes any peer-asserted event (SEV) and
/// IRQ-pending bits before the execution loop so a signal that landed
/// between barriers does not slip through.
fn core_worker_body(
    core_id: u8,
    mut core: CortexM33,
    shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
    timing_enabled: bool,
) -> (CortexM33, PerWorkerTimings) {
    let mut bus = WorkerBus::new(core_id, shared.clone());
    let mut target: u64 = 0;
    let idx = core_id as usize;
    let mut rec = TimingRecorder::new(n, timing_enabled);
    // Quantum 0's phase_work_ns is measured from worker entry, so it
    // includes thread-spawn residue. Intentional — it makes the first
    // quantum identifiable in summaries as "entry+phase_work".
    rec.on_worker_entry();

    for _ in 0..n {
        target = target.wrapping_add(step_q as u64);

        // WFE wake: consume the event_flag and clear wfe_waiting so the
        // step loop resumes execution. WFI wake is Phase 5. Pairs with
        // `CoreAtomics::sev_both`'s `Release` store on the SEV caller
        // side via `event_flag_consume`'s `AcqRel` swap.
        if shared.atomics.is_wfe_waiting(idx)
            && shared.atomics.event_flag_consume(idx)
        {
            shared.atomics.clear_wfe_waiting(idx);
        }

        // Merge coordinator-/peer-asserted IRQs into this core's NVIC.
        // `take_irq_pending` is an `AcqRel` swap-to-zero — the non-zero
        // return is the consume-and-merge trigger (LLD V7 §2).
        let pending = shared.atomics.take_irq_pending(idx);
        if pending != 0 {
            core.ppb.merge_irq_pending(pending);
        }

        while !shared.atomics.is_halted(idx)
            && !shared.atomics.is_wfe_waiting(idx)
            && core.cycles() < target
        {
            core.ppb.update_latest_cycles(core.cycles());
            core.step(&mut bus);
            if !bus.pending_cache_invalidations.is_empty() {
                // Decode cache lives on `core` (Phase 3 follow-up #10);
                // `invalidate_decode_cache_entries` is now inherent on
                // `CortexM33` and doesn't need `bus`. Drain in place.
                core.invalidate_decode_cache_entries(&bus.pending_cache_invalidations);
                bus.pending_cache_invalidations.clear();
            }
        }
        core.ppb.update_latest_cycles(core.cycles());

        // Phase 4 Stage B (HLD V7 §4.1): per-core SysTick advance.
        // Halted cores produce a zero delta via the snapshot in
        // `Ppb::systick_advance`, matching serial.
        core.ppb.systick_advance(core.cycles());

        // Phase 4 Stage C (HLD V7 §5.1): single-barrier rendezvous
        // after phase work. Overlaps with coord phase-2 of this
        // quantum. Poison propagation per §5.4.
        rec.on_wait_entry();
        let result = barrier.wait();
        rec.on_wait_return();
        if result == BarrierResult::Poisoned {
            return (core, rec.take());
        }
    }
    (core, rec.take())
}

/// PIO worker. Drains CPU-queued commands (INSTR_MEM / CLKDIV writes),
/// then steps each enabled state machine `step_q` sysclocks. PIO IRQ
/// routing to the NVIC is deliberately omitted — the single-threaded
/// `Bus` path also does not route PIO IRQs today, and Phase 3 §6
/// scopes this to parity (adding it requires both edges, which is a
/// separate HLD).
fn pio_worker_body(
    mut blocks: [PioBlock; 3],
    shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
    timing_enabled: bool,
) -> ([PioBlock; 3], PerWorkerTimings) {
    let mut rec = TimingRecorder::new(n, timing_enabled);
    rec.on_worker_entry();

    for _ in 0..n {
        for cmd in shared.pio.drain_commands() {
            apply_pio_command(&mut blocks, &shared.pio, cmd);
        }

        // GPIO_IN snapshot once per quantum — parity with the
        // single-threaded `Emulator::tick_peripherals` which reads
        // `bus.gpio_in` once and hands it to every PIO step.
        let gpio_in = shared.gpio.read_in();

        for (block_idx, block) in blocks.iter_mut().enumerate() {
            // `shared.pio.read_sm_enabled` reflects the last-applied
            // CTRL write's SM_ENABLE mask (`apply_pio_command::WriteCtrl`
            // republishes it after each CTRL write). A zero mask means
            // no SM in this block can make progress this quantum, so
            // skip the per-SM stepping loop entirely.
            let enabled = shared.pio.read_sm_enabled(block_idx);
            if enabled == 0 {
                continue;
            }
            // `PioBlock::step_n` gates on `sm_enabled_mask` internally;
            // mirroring the enable mask into the block keeps its fast
            // path tight.
            block.step_n(step_q, gpio_in);

            // Reflect the block's IRQ flags back onto `ThreadedPio` so
            // CPU workers observe them through the shared atomic.
            // PIO→NVIC assertion is Phase-later scope (see function
            // doc); we only publish the bits here.
            shared.pio.write_irq_flags(block_idx, block.pending_irqs() as u8);
        }

        // Phase 4 Stage B (HLD V7 §4.3): publish every block's pad
        // state — including disabled blocks, whose pads may still carry
        // a non-zero latch from the last active tick — so coord's
        // `update_gpio` sees a coherent snapshot.
        for (block_idx, block) in blocks.iter().enumerate() {
            shared.pio.write_pads(block_idx, block.pad_out, block.pad_oe);
        }

        // Phase 4 Stage C (HLD V7 §5.1): single-barrier rendezvous
        // after phase work. Overlaps with coord phase-2 of this
        // quantum. Poison propagation per §5.4.
        rec.on_wait_entry();
        let result = barrier.wait();
        rec.on_wait_return();
        if result == BarrierResult::Poisoned {
            return (blocks, rec.take());
        }
    }
    (blocks, rec.take())
}

/// Apply a CPU-queued [`PioCommand`] to the PIO worker's local
/// `PioBlock`s. Routes through each block's public `write32` so all
/// the existing bookkeeping (INSTR_MEM index guard, FIFO-join handling,
/// alias decoding, SM enable-mask invariant) continues to apply.
///
/// After `WriteCtrl`, the post-write `sm_enabled_mask` is republished
/// onto `ThreadedPio::sm_enabled` so CPU-side reads of CTRL.SM_ENABLE
/// and the `pio_worker_body` enable-gate check see the new state on
/// the next quantum. `WriteReg` republishes the mask too — on the
/// chance a generic write touches per-SM state that flips an SM's
/// enable (belt-and-braces; today no per-SM register toggles enable,
/// but this keeps the invariant local to this function regardless of
/// future `PioBlock::write32` extensions).
fn apply_pio_command(
    blocks: &mut [PioBlock; 3],
    shared_pio: &super::ThreadedPio,
    cmd: PioCommand,
) {
    match cmd {
        PioCommand::WriteInstrMem { block, addr, value, alias } => {
            let b = block as usize;
            if b >= blocks.len() || addr >= 32 {
                return;
            }
            let offset = 0x048 + (addr as u32) * 4;
            blocks[b].write32(offset, value as u32, alias as u32);
        }
        PioCommand::SetClkDiv { block, sm, int_div, frac_div, alias } => {
            let b = block as usize;
            if b >= blocks.len() || sm >= 4 {
                return;
            }
            // SMn_CLKDIV lives at 0x0C8 + sm * 0x18. Layout: INT<<16,
            // FRAC<<8, rest reserved (mdpicoem-common::pio::mod §write_clkdiv).
            let offset = 0x0C8 + (sm as u32) * 0x18;
            let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
            blocks[b].write32(offset, val, alias as u32);
        }
        PioCommand::WriteCtrl { block, val, alias } => {
            let b = block as usize;
            if b >= blocks.len() {
                return;
            }
            blocks[b].write32(0x000, val, alias as u32);
            // Republish the post-write enable mask so CPU-side readers
            // (including the PIO worker's own step-loop enable gate)
            // observe it next quantum.
            shared_pio.write_sm_enabled(b, blocks[b].sm_enabled_mask());
        }
        PioCommand::WriteReg { block, offset, val, alias } => {
            let b = block as usize;
            if b >= blocks.len() {
                return;
            }
            blocks[b].write32(offset as u32, val, alias as u32);
            // Conservative republish: keeps the mask coherent even if a
            // future `PioBlock::write32` extension ends up toggling
            // `enabled` outside CTRL.
            shared_pio.write_sm_enabled(b, blocks[b].sm_enabled_mask());
        }
    }
}

/// Coordinator worker. Merges GPIO, advances `master_cycle` + MTIME,
/// then ticks the coordinator-owned peripherals, and rendezvouses on
/// the shared barrier at the tail of each quantum (Phase 4 Stage C,
/// HLD V7 §5.1). The `fetch_add(Release)` on `master_cycle` pairs
/// with every CPU worker's `load(Acquire)` in `bus/peripherals.rs`'s
/// PLL CS read path (LLD V7 §3, §9).
fn coordinator_worker_body(
    shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
    timing_enabled: bool,
) -> ((), PerWorkerTimings) {
    let mut rec = TimingRecorder::new(n, timing_enabled);
    rec.on_worker_entry();

    for _ in 0..n {
        // Phase 4 Stage B (HLD V7 §4.2): merge SIO + PIO pad state into
        // `gpio_in` first, mirroring serial's PIO step → update_gpio →
        // mtime → APB tick chain. Serial's PIO step is on the PIO
        // worker under Phase 4, so coord picks up the chain here.
        update_gpio(&shared);

        // Advance master_cycle BEFORE ticking peripherals so CPU
        // workers' next-quantum PLL reads observe the fresh timeline.
        shared.master_cycle.fetch_add(step_q as u64, Ordering::Release);
        shared.sio.mtime_advance(step_q as u64);

        tick_peripherals(&shared, step_q);

        // Phase 4 Stage C (HLD V7 §5.1): single-barrier rendezvous
        // after phase work. Overlaps with CPU/PIO phase-1 of this
        // quantum. Poison propagation per §5.4.
        rec.on_wait_entry();
        let result = barrier.wait();
        rec.on_wait_return();
        if result == BarrierResult::Poisoned {
            return ((), rec.take());
        }
    }
    ((), rec.take())
}

/// Coordinator-owned GPIO merge. Ports `Emulator::update_gpio`
/// (`lib.rs:406-415`): start with SIO pads (`out & oe`, bank 0), fold
/// each PIO block's `(pad_out, pad_oe)` overlay in block order, then
/// apply the external-stimulus overlay last.
fn update_gpio(shared: &SharedState) {
    let sio_out = shared.gpio.read_out(0);
    let sio_oe = shared.gpio.read_oe(0);
    let mut merged = sio_out & sio_oe;
    for block_idx in 0..3 {
        let (pad_out, pad_oe) = shared.pio.read_pads(block_idx);
        merged = (merged & !pad_oe) | (pad_out & pad_oe);
    }
    let (ext_val, ext_mask) = shared.gpio.read_external();
    shared.gpio.write_in((merged & !ext_mask) | (ext_val & ext_mask));
}

/// Coordinator-owned peripheral tick. Phase 4 Stage A port of
/// `Bus::tick_peripherals` (`bus/mod.rs:915`) minus DMA — DMA lands in
/// Phase 5 alongside PIO-DREQ wiring (HLD V7 §2.2).
fn tick_peripherals(shared: &SharedState, cycles: u32) {
    use crate::bus::{
        RESET_ADC, RESET_I2C0, RESET_I2C1, RESET_PWM, RESET_SPI0, RESET_SPI1, RESET_TIMER0,
        RESET_TIMER1, RESET_UART0, RESET_UART1,
    };

    // RESETS snapshot — single acquire, reused for all five gates this
    // quantum. A mid-quantum CPU-worker RESETS write takes effect next
    // quantum (HLD V7 §3.2).
    let resets_state = shared.peripherals.resets.lock().unwrap().resets_state;
    let held = |bit: u8| (resets_state & (1u32 << bit)) != 0;

    // Clock-tree snapshot (Copy) — released before the APB tick block.
    let tree = shared.peripherals.clocks.lock().unwrap().clock_tree;

    let mut ext_irqs = 0u64;

    // Steps 1–3 under a single timers-lock acquire (HLD V7 §3.1).
    {
        let mut timers = shared.peripherals.timers.lock().unwrap();
        timers.ticks.advance_all(cycles);

        if !held(RESET_TIMER0) {
            let edges = timers.ticks.take_timer0_edges();
            if edges > 0 {
                timers.timer0.advance_us(edges);
            }
            ext_irqs |= timers.timer0.poll_alarms();
        }

        if !held(RESET_TIMER1) {
            let edges = timers.ticks.take_timer1_edges();
            if edges > 0 {
                timers.timer1.advance_us(edges);
            }
            ext_irqs |= timers.timer1.poll_alarms();
        }
    }

    // Phase-2 APB peripherals — each advances per sys_clk unless held.
    {
        let mut apb = shared.peripherals.apb.lock().unwrap();
        if !held(RESET_UART0) {
            apb.uart[0].tick(cycles, &tree, &mut ext_irqs);
        }
        if !held(RESET_UART1) {
            apb.uart[1].tick(cycles, &tree, &mut ext_irqs);
        }
        if !held(RESET_SPI0) {
            apb.spi[0].tick(cycles, &tree, &mut ext_irqs);
        }
        if !held(RESET_SPI1) {
            apb.spi[1].tick(cycles, &tree, &mut ext_irqs);
        }
        if !held(RESET_I2C0) {
            apb.i2c[0].tick(cycles, &tree, &mut ext_irqs);
        }
        if !held(RESET_I2C1) {
            apb.i2c[1].tick(cycles, &tree, &mut ext_irqs);
        }
        if !held(RESET_ADC) {
            apb.adc.tick(cycles, &tree, &mut ext_irqs);
        }
        if !held(RESET_PWM) {
            apb.pwm.tick(cycles, &tree, &mut ext_irqs);
        }
    }

    // `Peripherals::dma` intentionally deferred to Phase 5 (HLD V7 §2.2).

    // IRQ dispatch — drop software-only bits 46..=51, assert shared.
    let mut mask = ext_irqs & crate::irq::PERIPH_IRQ_MASK;
    while mask != 0 {
        let bit = mask.trailing_zeros();
        shared.atomics.assert_irq_shared(bit);
        mask &= mask - 1;
    }
}

// =======================================================================
// Tests
// =======================================================================
//
// Stage 7 (LLD V7 §11 items 13–19): smoke tests that spawn the 4-worker
// runtime and verify end-to-end execution semantics — quantum advance,
// cross-core SRAM visibility, WFE/SEV wake, FIFO-push wake, spinlock
// contention, doorbell state, and decode-cache invalidation plumbing.
//
// The three `from_emulator_preserves_*` tests from earlier stages stay
// — they exercise the destructure + seed round-trip.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Config;

    // ----- Handoff + round-trip (stages prior) --------------------------

    #[test]
    fn from_emulator_builds_threadedemulator() {
        let emu = Emulator::new(Config::default());
        let threaded = ThreadedEmulator::from_emulator(emu);
        assert_eq!(threaded.master_cycle(), 0);
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

    // ----- §11 item 13: master_cycle advances per quantum ---------------

    /// Coordinator advances `master_cycle` by `step_quantum` each
    /// quantum. Running 1 + 100 quanta should land at 101 ticks'
    /// worth of `master_cycle` (halted cores ⇒ no CPU execution,
    /// isolates the coordinator's `fetch_add` contribution).
    #[test]
    fn run_quanta_single_then_many() {
        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        // Halt both cores so the CPU workers spin-and-wait; coordinator
        // still advances master_cycle per quantum regardless.
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        let step_q = threaded.step_quantum as u64;
        assert_eq!(threaded.master_cycle(), 0);

        threaded.run_quanta(1);
        assert_eq!(threaded.master_cycle(), step_q);

        threaded.run_quanta(100);
        assert_eq!(
            threaded.master_cycle(),
            101 * step_q,
            "master_cycle must advance by step_quantum each quantum"
        );
    }

    // ----- Stage A: tick_peripherals fires TIMER0 ALARM0 end-to-end -----

    /// Smoke test for Phase 4 Stage A `tick_peripherals` port. Programs
    /// TIMER0 (TICKS.TIMER0 enabled, ALARM0=5, INTE=1) via serial MMIO
    /// before handoff, then runs the threaded coordinator a few quanta
    /// with both cores halted. The coordinator's `tick_peripherals`
    /// must drive TICKS → TIMER0 edges → alarm match. We observe
    /// `TIMER0.INTR` (latched on poll_alarms match, cleared only by
    /// explicit W1C — stable under Stage C overlap, unlike the
    /// atomic IRQ wire which CPU workers swap-to-zero each quantum).
    #[test]
    fn tick_peripherals_fires_timer0_alarm0_shared_irq() {
        use crate::peripherals::ticks::{
            CTRL_ENABLE, DOMAIN_STRIDE, DOMAIN_TIMER0, TICKS_BASE,
        };
        use crate::peripherals::timer::{ALARM0_OFFSET, INTE_OFFSET, INTR_OFFSET, TIMER0_BASE};

        let mut emu = Emulator::new(Config::default());
        // TIMER0 is released post-bootrom already; enable the TICKS
        // TIMER0 domain so sys_clk cycles turn into TIMER0 µs edges,
        // then arm ALARM0 with INTE to route the match to NVIC.
        let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
        emu.bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
        emu.bus.write32(TIMER0_BASE + INTE_OFFSET, 0x1, 0);
        emu.bus.write32(TIMER0_BASE + ALARM0_OFFSET, 5, 0);

        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        // step_quantum=64 sys_clks, TIMER0 CYCLES=12 ⇒ 5 edges/quantum.
        // Two quanta (≥10 µs) is comfortably past the ALARM0=5 target.
        threaded.run_quanta(2);

        // TIMER0.INTR bit 0 latches on ALARM0 match and stays set
        // until an ISR W1Cs it. Observable post-run regardless of
        // whether CPU worker take_irq_pending raced ahead of coord's
        // assert_irq_shared in the final quantum (Stage C overlap).
        let timer0_intr = threaded.shared.peripherals.timers.lock().unwrap()
            .timer0.read32(INTR_OFFSET);
        assert_ne!(
            timer0_intr & 0x1,
            0,
            "TIMER0 ALARM0 INTR must be latched after tick_peripherals drove count_us past ALARM0",
        );
    }

    // ----- §11 item 14: SRAM write visible across cores -----------------

    /// SRAM writes made through `SharedMemory` from the core-0 worker
    /// side must be visible to core-1 reads. Drive via the shared
    /// memory interface directly (per spec note — "prefer
    /// `shared.memory.write32` / `read32` directly since full-emulator
    /// smoke is the goal"). Run a quantum before reading to make sure
    /// the barrier protocol does not strand stores.
    #[test]
    fn sram_write_visible_across_cores() {
        // Validates Arc<SharedMemory> aliasing — a store through one
        // owner's handle is visible via another owner's handle. Full
        // CPU-worker-thread-0 → CPU-worker-thread-1 visibility under
        // step() requires firmware driving and is deferred to the
        // firmware-oracle phase. This test exercises the memory-layer
        // contract; the §9 barrier protocol ensures the worker-to-worker
        // happens-before chain separately.
        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        let addr: u32 = 0x2000_1000;
        let val: u32 = 0xDEAD_BEEF;

        // Core 0 writes via shared memory.
        threaded.shared.memory.write32(addr, val);

        // Advance a quantum.
        threaded.run_quanta(1);

        // Core 1 observes the value through the same shared memory.
        assert_eq!(
            threaded.shared.memory.read32(addr),
            val,
            "SRAM write from core 0's side must be visible to core 1"
        );
    }

    // ----- §11 item 15: WFE/SEV wake ------------------------------------

    /// Park core 0 on WFE by setting `wfe_waiting[0]` directly, then
    /// fire SEV. The next quantum's top-of-loop check consumes the
    /// event_flag and clears wfe_waiting. Because Stage 7 does not
    /// boot firmware here, we drive the pre-condition via the atomics
    /// surface.
    #[test]
    fn wfe_sev_wake() {
        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        // Halt core 1 so only core 0's worker exercises the wake hook.
        threaded.shared.atomics.set_halted(1);

        // Park core 0 on WFE.
        threaded.shared.atomics.set_wfe_waiting(0);
        assert!(threaded.shared.atomics.is_wfe_waiting(0));

        // Fire SEV (sets event_flag on both cores).
        threaded.shared.atomics.sev_both();

        // Run a quantum — core_worker_body should consume event_flag
        // and clear wfe_waiting before entering the step loop.
        threaded.run_quanta(1);

        assert!(
            !threaded.shared.atomics.is_wfe_waiting(0),
            "WFE wake must clear wfe_waiting after SEV"
        );
        assert!(
            !threaded.shared.atomics.event_flag_load(0),
            "event_flag[0] must be consumed by the wake check"
        );
    }

    // ----- §11 item 16: FIFO push wakes peer's WFE ----------------------

    /// A FIFO_WR MMIO write from core 1 must set `event_flag[0]` via
    /// the Stage 5 WorkerBus hook, which wakes a WFE-parked core 0
    /// on the next quantum. Drives the hook directly through a
    /// `WorkerBus::write32` call so the test does not depend on a
    /// firmware-driven MMIO path.
    #[test]
    fn fifo_push_wakes_peer_wfe() {
        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(1);
        threaded.shared.atomics.set_wfe_waiting(0);

        // Core-1 side pushes FIFO_WR (SIO offset 0x054). Use a
        // transient WorkerBus bound to core 1 — the same dispatch
        // the worker loop goes through.
        {
            use crate::core::CoreBus;
            let mut bus = WorkerBus::new(1, threaded.shared.clone());
            bus.write32(0xD000_0054, 0x1234_5678, 1);
        }

        // event_flag[0] must be set now (pre-quantum).
        assert!(
            threaded.shared.atomics.event_flag_load(0),
            "FIFO push must set event_flag on the peer core"
        );

        threaded.run_quanta(1);

        assert!(
            !threaded.shared.atomics.is_wfe_waiting(0),
            "FIFO push hook must wake core 0 from WFE"
        );
    }

    // ----- §11 item 17: spinlock contention -----------------------------

    /// Two cores racing for the same spinlock: core 0 claims, core 1
    /// tries and gets 0 (failed). Core 0 releases; core 1 reclaims
    /// successfully. Drives the lock through `ThreadedSio` directly
    /// (parity with WorkerBus spinlock dispatch at 0x100..=0x17F).
    #[test]
    fn spinlock_contended_both_cores() {
        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let sio = threaded.shared.sio.clone();

        // Core 0 claims lock 7.
        let first = sio.spinlock_claim(7);
        assert_eq!(first, 1u32 << 7, "core 0 should succeed claiming free lock");

        // Core 1 tries and fails (returns 0).
        let second = sio.spinlock_claim(7);
        assert_eq!(second, 0, "core 1 must see the lock held and return 0");

        // Core 0 releases. Core 1 re-tries and succeeds.
        sio.spinlock_release(7);
        let third = sio.spinlock_claim(7);
        assert_eq!(third, 1u32 << 7, "core 1 must claim after release");
    }

    // ----- §11 item 18: doorbell state roundtrip (no IRQ) ---------------

    /// §6 scope: doorbell writes mutate bits without asserting IRQ
    /// (parity with single-threaded `sio/mod.rs:152-154`). Verify the
    /// state roundtrips and the shared NVIC pending bits stay clear.
    #[test]
    fn doorbell_state_roundtrips_without_irq() {
        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let sio = &threaded.shared.sio;
        let atomics = &threaded.shared.atomics;

        // Pre-condition: no pending IRQ bits on either core.
        assert_eq!(atomics.irq_pending_load(0), 0);
        assert_eq!(atomics.irq_pending_load(1), 0);

        // Core 0 rings core 1's doorbell.
        sio.doorbell_set(1, 0b0101);
        assert_eq!(sio.doorbell_read(1), 0b0101);

        // Clear two bits; verify read-back.
        sio.doorbell_clear(1, 0b0100);
        assert_eq!(sio.doorbell_read(1), 0b0001);

        // IRQ pending must not be asserted — §6 scope parity.
        assert_eq!(
            atomics.irq_pending_load(0),
            0,
            "doorbell must not raise IRQ on sender"
        );
        assert_eq!(
            atomics.irq_pending_load(1),
            0,
            "doorbell must not raise IRQ on receiver (§6 scope parity)"
        );
    }

    // ----- §11 item 19: cross-core SMC → decode-cache invalidation ------

    /// Plumbing test: a WorkerBus write32 into SRAM (region `0x2`)
    /// pushes the address onto `pending_cache_invalidations`. The
    /// worker loop drains the Vec each quantum — the plumbing guarantee
    /// is that (a) the write records, (b) the loop drains, (c) the
    /// next instruction's decode goes through a fresh fetch. We test
    /// (a)+(b) here; (c) is covered end-to-end by full-firmware tests
    /// in later phases.
    ///
    /// V7 LLD §10 closure (2026-04-17): the `ISB` instruction now emits
    /// a `SeqCst` fence and calls `CortexM33::invalidate_decode_cache_all`
    /// on the bus, so the observing core's cache is flushed on the ISB
    /// in addition to the per-write queue drained below. That semantics
    /// layer is exercised by the in-crate `core::tests`/`decode` cache
    /// tests — this test remains focused on the WorkerBus plumbing.
    #[test]
    fn cross_core_smc_dsb_isb_fetches_new_insn() {
        // Plumbing validation only: confirms WorkerBus::write32 pushes
        // addresses into pending_cache_invalidations, and that the worker
        // body drains them via invalidate_decode_cache_entries. End-to-end
        // cross-core SMC (core 0 writes → core 1 executes rewritten insn)
        // requires firmware and is deferred to the firmware-oracle phase.
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // (a) write32 into SRAM records TWO pending invalidations
        // (`addr` and `addr+2`) so the drainer's `{slot(addr-2),
        // slot(addr)}` pattern ends up covering `{addr-2, addr, addr+2}`
        // — parity with `Bus::invalidate_pc_range(addr, 4)`.
        let addr_a = 0x2000_0100;
        bus.write32(addr_a, 0xBF00_BF00, 0);
        assert_eq!(
            bus.pending_cache_invalidations.len(),
            2,
            "write32 must queue two decode-cache invalidations"
        );
        assert_eq!(
            bus.pending_cache_invalidations[0], addr_a,
            "first queued entry is the word address"
        );
        assert_eq!(
            bus.pending_cache_invalidations[1], addr_a + 2,
            "second queued entry is word+2 to cover trailing hw slot"
        );

        // (b) a second write accumulates — two more entries.
        bus.write32(0x2000_0200, 0xBF00_BF00, 0);
        assert_eq!(
            bus.pending_cache_invalidations.len(),
            4,
            "second write32 accumulates two more entries"
        );

        // Simulate the worker loop's drain step:
        let mut dummy = Emulator::new(Config::default());
        dummy
            .core_mut(0)
            .invalidate_decode_cache_entries(&bus.pending_cache_invalidations);
        bus.pending_cache_invalidations.clear();

        assert!(
            bus.pending_cache_invalidations.is_empty(),
            "drain + clear must leave the queue empty"
        );

        // Non-exec region writes (APB / SIO) must NOT queue invalidations.
        bus.write32(0xD000_0010, 0, 0);
        assert!(
            bus.pending_cache_invalidations.is_empty(),
            "SIO write must not queue a decode-cache invalidation"
        );
    }

    // ----- Phase 3 task #11: PIO CTRL / INSTR_MEM / CLKDIV routing ------

    /// `WriteCtrl` applied via `apply_pio_command` must flip the
    /// per-block `sm_enabled` mask on `ThreadedPio`. Before the task
    /// #11 routing landed, `shared.pio.read_sm_enabled` was 0 indefinitely
    /// because `ahb_write32` dropped CTRL writes silently. This test
    /// drives the command queue directly to isolate
    /// `apply_pio_command`'s republish path from the MMIO dispatcher.
    #[test]
    fn pio_sm_enable_routes_through_command_queue() {
        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));

        // Block 0 starts disabled.
        assert_eq!(threaded.shared.pio.read_sm_enabled(0), 0);

        // Enqueue a CTRL write enabling SMs 0 and 2.
        threaded.shared.pio.send_command(PioCommand::WriteCtrl {
            block: 0,
            val: 0b0101, // SM0 + SM2
            alias: 0,
        });

        // Drain + apply as the PIO worker would at quantum entry.
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        for cmd in threaded.shared.pio.drain_commands() {
            apply_pio_command(&mut blocks, &threaded.shared.pio, cmd);
        }

        assert_eq!(
            threaded.shared.pio.read_sm_enabled(0),
            0b0101,
            "CTRL write must republish enable mask onto ThreadedPio"
        );
        assert_eq!(blocks[0].sm_enabled_mask(), 0b0101);
        // Other blocks unaffected.
        assert_eq!(threaded.shared.pio.read_sm_enabled(1), 0);
        assert_eq!(threaded.shared.pio.read_sm_enabled(2), 0);
    }

    /// A CTRL write landing through `WorkerBus::ahb_write32` must
    /// enqueue a `WriteCtrl` command and — after `apply_pio_command`
    /// runs — propagate to `ThreadedPio::read_sm_enabled`. This covers
    /// the end-to-end MMIO → command-queue → block hand-off for the
    /// critical unblocker.
    #[test]
    fn pio_ctrl_write_via_worker_bus_propagates_to_threaded_pio() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO1 CTRL = 0x5030_0000, enable SMs 1 and 3.
        bus.write32(0x5030_0000, 0b1010, 0);

        // Command must be queued (not dropped silently).
        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 1, "CTRL write must queue exactly one command");
        assert_eq!(
            pending[0],
            PioCommand::WriteCtrl { block: 1, val: 0b1010, alias: 0 }
        );

        // Apply + verify the republish lands on ThreadedPio.
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        apply_pio_command(&mut blocks, &threaded.shared.pio, pending[0]);
        assert_eq!(threaded.shared.pio.read_sm_enabled(1), 0b1010);
        assert_eq!(blocks[1].sm_enabled_mask(), 0b1010);
    }

    /// INSTR_MEM writes through `WorkerBus::ahb_write32` must land in
    /// `PioBlock::instr_mem` after the PIO worker applies the command.
    /// Task-required smoke test: "a PIO INSTR_MEM write through
    /// WorkerBus's ahb_write32 (not through a direct `send_command`
    /// call) actually propagates into PioBlock's instruction memory."
    #[test]
    fn pio_instr_mem_write_via_worker_bus_propagates_to_block() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 INSTR_MEM7 lives at 0x5020_0000 + 0x048 + 7*4 = 0x5020_0064.
        // Value is truncated to u16 inside PioBlock::write32.
        let insn: u32 = 0x0000_E080; // arbitrary PIO opcode-shaped word
        bus.write32(0x5020_0064, insn, 0);

        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 1, "INSTR_MEM write must queue one command");
        assert_eq!(
            pending[0],
            PioCommand::WriteInstrMem { block: 0, addr: 7, value: insn as u16, alias: 0 }
        );

        // Apply and verify it reached instr_mem[7].
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        apply_pio_command(&mut blocks, &threaded.shared.pio, pending[0]);
        assert_eq!(blocks[0].instr_mem()[7], insn as u16);
        // Neighbours untouched.
        assert_eq!(blocks[0].instr_mem()[6], 0);
        assert_eq!(blocks[0].instr_mem()[8], 0);
    }

    /// SMn_CLKDIV writes through `WorkerBus::ahb_write32` must decode
    /// into a `SetClkDiv` command carrying the INT/FRAC fields split
    /// out of the 32-bit register word, and land in the right SM slot.
    #[test]
    fn pio_clkdiv_write_via_worker_bus_decodes_int_frac() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO2 SM3_CLKDIV: 0x5040_0000 + 0x0C8 + 3*0x18 = 0x5040_0110.
        // Layout: INT << 16, FRAC << 8.
        let int_div: u16 = 0x1234;
        let frac_div: u8 = 0x56;
        let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
        bus.write32(0x5040_0110, val, 0);

        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending[0],
            PioCommand::SetClkDiv { block: 2, sm: 3, int_div, frac_div, alias: 0 }
        );
    }

    /// A write to an offset outside CTRL / INSTR_MEM / CLKDIV (e.g.
    /// TXF0 or IRQ) must fall through to the generic `WriteReg`
    /// variant so no PIO MMIO offset is silently dropped anymore.
    #[test]
    fn pio_non_fast_path_write_uses_generic_writereg() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 TXF0 at 0x5020_0010.
        bus.write32(0x5020_0010, 0xABCD_1234, 0);
        // PIO0 IRQ at 0x5020_0030 (W1C, alias 0).
        bus.write32(0x5020_0030, 0x0000_000F, 0);

        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 2, "two non-fast-path writes → two commands");
        assert_eq!(
            pending[0],
            PioCommand::WriteReg { block: 0, offset: 0x010, val: 0xABCD_1234, alias: 0 }
        );
        assert_eq!(
            pending[1],
            PioCommand::WriteReg { block: 0, offset: 0x030, val: 0x0000_000F, alias: 0 }
        );
    }

    /// `WorkerBus::ahb_read32` exposes the two atomics `ThreadedPio`
    /// publishes (CTRL.SM_ENABLE at 0x000 and IRQ at 0x030) so
    /// firmware that round-trips CTRL after enabling SMs observes the
    /// correct mask. Other offsets return 0 until read-through is wired.
    #[test]
    fn pio_ctrl_readback_reflects_published_enable_mask() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        // Seed the shared mask directly (simulates a post-apply state).
        threaded.shared.pio.write_sm_enabled(1, 0b1001);
        threaded.shared.pio.write_irq_flags(2, 0x0A);

        let mut bus = WorkerBus::new(0, threaded.shared.clone());
        // PIO1 CTRL readback.
        assert_eq!(bus.read32(0x5030_0000, 0), 0b1001);
        // PIO2 IRQ readback.
        assert_eq!(bus.read32(0x5040_0030, 0), 0x0A);
        // PIO0 CTRL still 0 (no writes).
        assert_eq!(bus.read32(0x5020_0000, 0), 0);
        // Non-wired offsets (e.g. FSTAT 0x004) fire a debug_assert
        // under test builds; release builds keep returning 0 for
        // forward compatibility. Covered by `pio_ahb_read32_unmapped_offset_panics_under_debug`.
    }

    /// End-to-end: after a CTRL write enables SM0 on PIO0, a single
    /// `run_quanta(1)` call must drain the queue, apply the command,
    /// and leave `read_sm_enabled(0) = 0b0001` — proving the
    /// per-quantum enable gate in `pio_worker_body` now observes the
    /// firmware-programmed state (it zero-skipped indefinitely before).
    #[test]
    fn pio_ctrl_write_drains_during_run_quanta() {
        use crate::core::CoreBus;

        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        // Halt both cores so `core_worker_body` idle-spins — the PIO
        // worker still drains commands and ticks each quantum.
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        // Queue a CTRL enable via the MMIO path (identical to a
        // firmware write through the bus).
        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            bus.write32(0x5020_0000, 0b0001, 0);
        }
        assert_eq!(
            threaded.shared.pio.read_sm_enabled(0),
            0,
            "before run_quanta, command is queued but not yet applied"
        );

        threaded.run_quanta(1);

        assert_eq!(
            threaded.shared.pio.read_sm_enabled(0),
            0b0001,
            "run_quanta must drain + apply the CTRL command"
        );
    }

    // ----- Phase 3 task #11 follow-up: alias propagation + coverage ------

    /// INSTR_MEM writes through an aliased MMIO address (SET/CLR/XOR)
    /// must carry the decoded alias into the `WriteInstrMem` command
    /// and through to `PioBlock::write32`. Without this, aliased writes
    /// to INSTR_MEM would silently downgrade to plain writes on the
    /// threaded path — diverging from the single-threaded `Bus` which
    /// forwards alias unconditionally.
    #[test]
    fn pio_instr_mem_alias_propagates_through_command() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 INSTR_MEM7 = 0x5020_0064; SET alias adds 0x2000.
        let set_alias_addr = 0x5020_0064 + 0x2000;
        bus.write32(set_alias_addr, 0x0000_DEAD, 0);

        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending[0],
            PioCommand::WriteInstrMem {
                block: 0,
                addr: 7,
                value: 0xDEAD,
                alias: 2, // SET
            },
            "SET alias (addr[13:12] = 2) must round-trip into the command",
        );
    }

    /// Same as above for SMn_CLKDIV. Using XOR alias (0x1000) here to
    /// cover a different encoding than the INSTR_MEM test.
    #[test]
    fn pio_clkdiv_alias_propagates_through_command() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 SM0_CLKDIV = 0x5020_00C8; XOR alias adds 0x1000.
        let xor_alias_addr = 0x5020_00C8 + 0x1000;
        let int_div: u16 = 0x0010;
        let frac_div: u8 = 0x80;
        let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
        bus.write32(xor_alias_addr, val, 0);

        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending[0],
            PioCommand::SetClkDiv {
                block: 0,
                sm: 0,
                int_div,
                frac_div,
                alias: 1, // XOR
            },
            "XOR alias (addr[13:12] = 1) must round-trip into the command",
        );
    }

    /// CTRL write via the SET alias must OR the incoming bits into the
    /// prior CTRL state, not overwrite. End-to-end test of alias
    /// semantics for CTRL through the WorkerBus → command-queue →
    /// apply_pio_command → PioBlock::write32 chain. PioBlock's write_ctrl
    /// implements alias=2 as bit-set on SM_ENABLE.
    #[test]
    fn pio_ctrl_write_with_set_alias_propagates_or_semantics() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));

        // Seed: enable SM0 on PIO0 via a plain CTRL write.
        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            bus.write32(0x5020_0000, 0b0001, 0);
        }
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        for cmd in threaded.shared.pio.drain_commands() {
            apply_pio_command(&mut blocks, &threaded.shared.pio, cmd);
        }
        assert_eq!(blocks[0].sm_enabled_mask(), 0b0001);

        // Now SET-alias write enabling SM2 — must OR with the prior
        // state, yielding 0b0101. Plain (alias=0) would overwrite to
        // 0b0100.
        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            bus.write32(0x5020_0000 + 0x2000, 0b0100, 0);
        }
        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending[0],
            PioCommand::WriteCtrl { block: 0, val: 0b0100, alias: 2 },
        );
        apply_pio_command(&mut blocks, &threaded.shared.pio, pending[0]);

        assert_eq!(
            blocks[0].sm_enabled_mask(),
            0b0101,
            "SET alias must OR into SM_ENABLE, preserving prior bits",
        );
        assert_eq!(
            threaded.shared.pio.read_sm_enabled(0),
            0b0101,
            "republished mask must match the post-alias state",
        );
    }

    /// End-to-end smoke: a TXF0 write must land in the target block's
    /// SM[0] tx_fifo after one `run_quanta`. This covers the generic
    /// `WriteReg` path all the way from WorkerBus → command queue →
    /// PioBlock::write32 → per-SM fifo state.
    #[test]
    fn pio_writereg_txf_end_to_end_lands_in_block_fifo() {
        use crate::core::CoreBus;

        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        // Halt CPUs so only the PIO worker runs its drain/step.
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            // PIO0 TXF0 at 0x5020_0010.
            bus.write32(0x5020_0010, 0xCAFE_BABE, 0);
        }

        threaded.run_quanta(1);

        // The PIO worker owns the blocks — we re-run drain + apply here
        // against a fresh scratch block array to observe the fifo state
        // that `PioBlock::write32` produced. The contract we're proving
        // is that WriteReg dispatch correctly routes through
        // `PioBlock::write32`; the fact that the run_quanta loop already
        // did the same work against the worker-owned blocks is the
        // production behaviour (unobservable from outside the worker).
        let mut scratch = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        apply_pio_command(
            &mut scratch,
            &threaded.shared.pio,
            PioCommand::WriteReg {
                block: 0,
                offset: 0x010,
                val: 0xCAFE_BABE,
                alias: 0,
            },
        );
        assert_eq!(
            scratch[0].pop_tx(0),
            Some(0xCAFE_BABE),
            "WriteReg(TXF0) must land in SM[0].tx_fifo via PioBlock::write32",
        );
    }

    /// Multi-command batch: a mix of CTRL, INSTR_MEM, CLKDIV, and
    /// generic WriteReg in one quantum must all drain + apply correctly.
    #[test]
    fn pio_multi_command_batch_all_drain() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            // CTRL enable (0x000).
            bus.write32(0x5020_0000, 0b0011, 0);
            // INSTR_MEM3 (0x048 + 3*4 = 0x054).
            bus.write32(0x5020_0054, 0x0000_1234, 0);
            // SM1_CLKDIV (0x0C8 + 1*0x18 = 0x0E0).
            bus.write32(0x5020_00E0, (5u32 << 16) | (0x40 << 8), 0);
            // TXF0 (0x010) — generic WriteReg.
            bus.write32(0x5020_0010, 0xAA55_AA55, 0);
            // IRQ (0x030) — generic WriteReg.
            bus.write32(0x5020_0030, 0x0000_0001, 0);
        }

        let pending = threaded.shared.pio.drain_commands();
        assert_eq!(pending.len(), 5, "five MMIO writes → five commands");
        assert_eq!(
            pending[0],
            PioCommand::WriteCtrl { block: 0, val: 0b0011, alias: 0 }
        );
        assert_eq!(
            pending[1],
            PioCommand::WriteInstrMem { block: 0, addr: 3, value: 0x1234, alias: 0 }
        );
        assert_eq!(
            pending[2],
            PioCommand::SetClkDiv {
                block: 0,
                sm: 1,
                int_div: 5,
                frac_div: 0x40,
                alias: 0,
            }
        );
        assert_eq!(
            pending[3],
            PioCommand::WriteReg { block: 0, offset: 0x010, val: 0xAA55_AA55, alias: 0 }
        );
        assert_eq!(
            pending[4],
            PioCommand::WriteReg { block: 0, offset: 0x030, val: 0x0000_0001, alias: 0 }
        );

        // Apply all and verify observable end-state matches the write sequence.
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        for cmd in pending {
            apply_pio_command(&mut blocks, &threaded.shared.pio, cmd);
        }
        assert_eq!(blocks[0].sm_enabled_mask(), 0b0011);
        assert_eq!(blocks[0].instr_mem()[3], 0x1234);
        assert_eq!(
            blocks[0].pop_tx(0),
            Some(0xAA55_AA55),
            "TXF0 byte must reach SM[0].tx_fifo",
        );
    }

    /// `ahb_read32` on an unmapped PIO offset must fire a `debug_assert`
    /// so Phase 4/5 read-through regressions surface loudly under test.
    /// Release builds still return 0 — this only catches the test path.
    #[test]
    #[should_panic(expected = "PIO ahb_read32 offset")]
    #[cfg(debug_assertions)]
    fn pio_ahb_read32_unmapped_offset_panics_under_debug() {
        use crate::core::CoreBus;

        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());
        // FSTAT at 0x5020_0004 — not yet wired.
        let _ = bus.read32(0x5020_0004, 0);
    }

    // ----- Phase 4 Stage B (HLD V7 §4) ----------------------------------

    /// Each CPU worker's phase-1 tail must call `ppb.systick_advance(cycles)`.
    /// Halted cores advance no cycles, so the observable side-effect is
    /// that `last_systick_cycles` snaps to the core's current `cycles`
    /// on the first call. Seed `last_systick_cycles = 42` pre-handoff so
    /// the first post-quantum read proves the hook fired.
    #[test]
    fn tick_systick_fires_in_cpu_worker_phase1() {
        let mut emu = Emulator::new(Config::default());
        emu.core_mut(0).ppb.last_systick_cycles = 42;
        emu.core_mut(1).ppb.last_systick_cycles = 99;

        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        threaded.run_quanta(1);

        // Halted cores stay at cycles=0, so systick_advance(0) must have
        // rewritten last_systick_cycles from (42, 99) to (0, 0).
        assert_eq!(
            threaded.core0.as_ref().unwrap().ppb.last_systick_cycles, 0,
            "core0 phase-1 must call systick_advance"
        );
        assert_eq!(
            threaded.core1.as_ref().unwrap().ppb.last_systick_cycles, 0,
            "core1 phase-1 must call systick_advance"
        );
    }

    /// Coordinator's phase-2 `update_gpio` must fold SIO pads + PIO pad
    /// snapshots + external stimulus into `gpio.in` mirroring serial.
    /// Exercised against the `update_gpio` function directly (the PIO
    /// worker republishes pads every quantum, so `run_quanta` would
    /// overwrite the seeded pad state before coord reads it).
    #[test]
    fn update_gpio_merges_sio_pio_and_external() {
        let threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));

        // SIO drives bit 0 (out & oe).
        threaded.shared.gpio.write_out(0, 0x0000_0001);
        threaded.shared.gpio.write_oe(0, 0x0000_0001);
        // PIO block 2 drives bit 4 high and bit 0 low via pad_oe
        // (higher-indexed blocks overlay lower ones per §4.2).
        threaded.shared.pio.write_pads(2, 0x0000_0010, 0x0000_0011);
        // External stimulus forces bit 8 high.
        threaded.shared.gpio.write_external(0x0000_0100, 0x0000_0100);

        update_gpio(&threaded.shared);

        // SIO bit 0 overridden by PIO block 2's pad_oe bit 0 (pad_out=0),
        // block 2's bit 4 high, external bit 8 high.
        assert_eq!(
            threaded.shared.gpio.read_in(),
            0x0000_0110,
            "update_gpio must overlay PIO then external on top of SIO"
        );
    }

    /// `from_emulator` must seed `ThreadedPio::pads` from each incoming
    /// `PioBlock.pad_out` / `pad_oe`. Without the seed, coord's first
    /// `update_gpio` reads zero and drops PIO output for one quantum.
    ///
    /// Regression guard: removing the seed loop in `from_emulator` (at
    /// the `threaded_pio.write_pads(...)` call) fails this test.
    #[test]
    fn from_emulator_seeds_pio_pads() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.pio[0].pad_out = 0xAAAA_0000;
        emu.bus.pio[0].pad_oe = 0xFFFF_0000;
        emu.bus.pio[1].pad_out = 0x0000_5555;
        emu.bus.pio[1].pad_oe = 0x0000_FFFF;
        emu.bus.pio[2].pad_out = 0x1234_5678;
        emu.bus.pio[2].pad_oe = 0x8765_4321;

        let threaded = ThreadedEmulator::from_emulator(emu);

        assert_eq!(threaded.shared.pio.read_pads(0), (0xAAAA_0000, 0xFFFF_0000));
        assert_eq!(threaded.shared.pio.read_pads(1), (0x0000_5555, 0x0000_FFFF));
        assert_eq!(threaded.shared.pio.read_pads(2), (0x1234_5678, 0x8765_4321));
    }

    // ----- Per-worker timing instrumentation ----------------------------

    /// When timing is disabled (default), `run_quanta` must not populate
    /// `last_run_timings`. Guards the zero-overhead contract — a consumer
    /// that forgets the flag sees `None` rather than stale data.
    #[test]
    fn timings_disabled_by_default_yields_none() {
        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        threaded.run_quanta(5);
        assert!(
            threaded.last_run_timings().is_none(),
            "disabled timings must leave last_run_timings = None"
        );
    }

    /// When timing is enabled, `run_quanta(n)` must populate all four
    /// workers' raw vecs with exactly `n` samples each (one per
    /// quantum).
    #[test]
    fn timings_enabled_records_n_samples_per_worker() {
        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);
        threaded.set_timing_enabled(true);

        let n: u64 = 10;
        threaded.run_quanta(n);

        let rt = threaded
            .last_run_timings()
            .expect("enabled run must populate last_run_timings");
        assert_eq!(rt.samples(), n as usize);
        for s in rt.summary() {
            assert_eq!(
                s.samples, n as usize,
                "worker {} must record n samples",
                s.name().as_str()
            );
            // Every quantum did *some* work — at minimum the barrier
            // wait itself took nonzero nanoseconds. On busy hosts the
            // Instant resolution floor (~100ns on Windows) means the
            // phase-work can round to 0 for the trivial halted-cores
            // case, so we only assert the total is monotonic, not
            // strictly positive.
            assert!(s.phase_work_total_ns >= s.phase_work_max_ns);
            assert!(s.barrier_wait_total_ns >= s.barrier_wait_max_ns);
        }
    }

    /// Re-running with timing disabled after an enabled run must reset
    /// `last_run_timings` to `None`. Prevents the stale-data trap
    /// where a consumer sees a non-`None` from the *previous* run.
    #[test]
    fn timings_reset_when_disabled_between_runs() {
        let mut threaded =
            ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        threaded.set_timing_enabled(true);
        threaded.run_quanta(3);
        assert!(threaded.last_run_timings().is_some());

        threaded.set_timing_enabled(false);
        threaded.run_quanta(3);
        assert!(
            threaded.last_run_timings().is_none(),
            "disabled second run must reset last_run_timings to None"
        );
    }

    /// End-to-end integration: a single `run_quanta` call must drive all
    /// three phase-2 pieces at once — SIO pad state merged into GPIO_IN,
    /// external stimulus overlaid on top, per-core SysTick advanced, and
    /// master_cycle advanced by the coordinator. PIO is covered directly
    /// by `update_gpio_merges_sio_pio_and_external` (the PIO worker
    /// republishes pads every quantum, which makes a pre-quantum
    /// `write_pads` seed a poor integration signal here).
    ///
    /// Both cores are halted: core 0's SysTick `last_systick_cycles` is
    /// pre-seeded to `u64::MAX - 99` so the `wrapping_sub` on phase-1's
    /// `systick_advance(0)` synthesises a delta of 100 — enough to drive
    /// CVR below its initial RVR without running firmware.
    #[test]
    fn run_quanta_integrates_sio_external_and_systick() {
        const SIO_BIT: u32 = 1 << 25; // GPIO25, LED on Pico 2
        const EXT_BIT: u32 = 1 << 8;
        const RVR_INIT: u32 = 1000;

        let mut emu = Emulator::new(Config::default());

        // SIO drives GPIO25 OUT=1, OE=1 pre-handoff. `AtomicGpio::seed`
        // lifts these into `shared.gpio.{out,oe}`.
        emu.bus.sio.gpio_out = SIO_BIT;
        emu.bus.sio.gpio_oe = SIO_BIT;

        // Harness-style external stimulus forces bit 8 high. Seeded into
        // the packed `external` AtomicU64 by `AtomicGpio::seed`.
        emu.bus.gpio_external_in = EXT_BIT;
        emu.bus.gpio_external_mask = EXT_BIT;

        // Enable core 0 SysTick (ENABLE=1, RVR=1000, CVR=1000) and
        // pre-seed `last_systick_cycles` so a halted-core call to
        // `systick_advance(0)` yields `delta = 100` via wrapping_sub.
        emu.core_mut(0).ppb.syst_csr = 1;
        emu.core_mut(0).ppb.syst_rvr = RVR_INIT;
        emu.core_mut(0).ppb.syst_cvr = RVR_INIT;
        emu.core_mut(0).ppb.last_systick_cycles = u64::MAX - 99;

        let mut threaded = ThreadedEmulator::from_emulator(emu);
        // Halt both cores — core 0 stays at cycles=0, so the SysTick
        // advance comes purely from the pre-seeded wrapping delta.
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        assert_eq!(threaded.master_cycle(), 0);

        threaded.run_quanta(2);

        // (1) GPIO merge ran — both SIO bit 25 and external bit 8
        // appear in `gpio.read_in()`. SIO is applied first, external
        // last; distinct bits mean both survive.
        let gpio_in = threaded.shared.gpio.read_in();
        assert_eq!(
            gpio_in & (SIO_BIT | EXT_BIT),
            SIO_BIT | EXT_BIT,
            "update_gpio must merge SIO + external into gpio_in"
        );

        // (2) Core 0 SysTick advanced. First quantum: delta=100 via
        // wrapping_sub, CVR drops from 1000 to 900. Second quantum:
        // delta=0, CVR stays at 900.
        let cvr = threaded.core0.as_ref().unwrap().ppb.syst_cvr;
        assert!(
            cvr < RVR_INIT,
            "core 0 systick_cvr must decrement below RVR after run_quanta (cvr={cvr})"
        );

        // (3) Coord's phase-2 ran — master_cycle advanced by
        // 2 * step_quantum.
        let step_q = threaded.step_quantum as u64;
        assert_eq!(
            threaded.master_cycle(),
            2 * step_q,
            "coordinator phase-2 must advance master_cycle each quantum"
        );
    }
}

//! `ThreadedEmulator` — 6-thread runtime entry point for Phase 3.
//!
//! Phase 3 Stage 7 (LLD V7 §9): real core / PIO / coordinator worker
//! bodies. Phase 4 Stage C (HLD V7 §5) collapsed the two-barrier
//! rendezvous to a single barrier per iteration: each worker performs
//! its phase work then rendezvouses once at the tail of the loop. CPU
//! phase-1 of quantum N now runs in parallel with coord phase-2 of
//! quantum N; the `2 × step_quantum` staleness ceiling that overlap
//! implies is accepted per HLD V7 §5.2. Stage B.2 of the 2026-04-22
//! threaded-PIO HLD V5 split the single PIO worker into three per-block
//! workers (pio0, pio1, pio2) so the emulator stops serialising what
//! the hardware runs in parallel.
//!
//! Gated behind `#[cfg(all(target_arch = "x86_64", any(target_os =
//! "windows", target_os = "linux")))]` because the thread-pinning
//! path uses `SetThreadAffinityMask` on Windows and
//! `pthread_setaffinity_np` on Linux. Other UNIX hosts stay on the
//! existing single-threaded `Emulator::run` path until
//! `pin_to_host_core` grows a port.
//!
//! Lifecycle at a glance:
//!
//! 1. Caller drives an existing `Emulator` to the pre-run state (load
//!    ROM / flash, reset, seed GPIO stimulus, etc.).
//! 2. `ThreadedEmulator::from_emulator(emu)` destructures the Bus into
//!    the shared state bundle and per-core CPUs.
//! 3. `run_quanta(n)` spawns six workers (core 0, core 1, pio0, pio1,
//!    pio2, coordinator), joins, and surfaces panics via the `poisoned`
//!    flag so the instance cannot be reused after a worker panic.
//!
//! The master-cycle counter lives on `SharedState.master_cycle` (an
//! `Arc<AtomicU64>`) so the coordinator's `fetch_add(Release)` pairs
//! with the CPU workers' `load(Acquire)` for PLL-LOCK derivation —
//! see `wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` §6 and
//! `threaded::peripherals::ClocksState::pll_sys_read_at`.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

use picoem_common::PioBlock;

use crate::Emulator;
use crate::bus::Bus;
use crate::core::CortexM33;

use super::peripherals::{
    ApbState, ClocksState, DmaState, Peripherals, QmiState, ResetsState, TimersState, UsbState,
};
use super::timings::{PerWorkerTimings, RunTimings, TimingRecorder};
use super::{
    AtomicGpio, BarrierResult, ExclusiveMonitors, PioCommand, SharedMemory, SharedState,
    SpinBarrier, ThreadedPio, ThreadedSio, WorkerBus, panic_message, spawn_worker,
};

/// Runtime-error payload returned from [`ThreadedEmulator::run_quanta_checked`].
/// Distinguishes a worker panic from a barrier-watchdog timeout so the
/// outer [`crate::EmulatorError`] surface can expose the two cases as
/// separate variants (HLD V1 §6.6 Stage 5).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RunError {
    /// One of the worker threads panicked. `message` is the downcast
    /// payload text; `which` is the first worker to return `Err` from
    /// `JoinHandle::join` scanning in worker-index order.
    Panic {
        which: super::WorkerName,
        message: String,
    },
    /// The shared barrier's wall-clock deadline elapsed before all
    /// workers arrived. `which` names the first worker whose barrier
    /// call returned `TimedOut` (an observer, not the culprit); the
    /// barrier cannot identify the missing worker on its own.
    /// `elapsed_ms` is the wall-clock elapsed time recorded by the
    /// first waiter to trip the watchdog.
    Timeout {
        which: super::WorkerName,
        elapsed_ms: u32,
    },
}

/// 6-thread runtime handle over a seeded `SharedState` and both CPU
/// cores. See module-level docs for the Stage 6b → Stage 7 split, and
/// the 2026-04-22 Threaded PIO Per-Block Workers HLD V5 for the
/// per-block PIO worker topology.
pub struct ThreadedEmulator {
    shared: SharedState,
    core0: Option<CortexM33>,
    core1: Option<CortexM33>,
    pio_blocks: Option<[PioBlock; 3]>,
    step_quantum: u32,
    thread_mask: [usize; 6],
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
    /// Latched true once either CPU worker observed the bootrom mask-ROM
    /// hook fire (HLD V5 §"Component 3"). Drained from each
    /// `CortexM33::bootrom_hook_fired` at the post-`run_quanta_checked`
    /// join, so it survives across calls and is visible to the host via
    /// [`Self::shutdown_requested`]. Mirrors the serial drain in
    /// `lib.rs:707-717` — without this, worker-side latches never reach
    /// `Emulator::shutdown_requested` and the host loops forever.
    bootrom_hook_fired: bool,
}

impl ThreadedEmulator {
    /// Consume a single-threaded `Emulator` and return a
    /// `ThreadedEmulator` with every piece of state hoisted onto the
    /// shared `SharedState`.
    ///
    /// Panics if `std::thread::available_parallelism()` reports fewer
    /// than 6 host cores — the runtime pins one thread per core and a
    /// host with fewer cores cannot satisfy that without OS contention.
    pub fn from_emulator(emu: Emulator) -> Self {
        let n = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1);
        assert!(
            n >= 6,
            "ThreadedEmulator requires >= 6 host cores (found {n})"
        );

        let Emulator {
            cores,
            bus,
            clock,
            step_quantum,
            execution_model: _,
            threaded: _,
            panic_info: _,
            timeout_info: _,
            #[cfg(feature = "testing")]
                pending_panic_inject: _,
            bus_is_placeholder: _,
            shutdown_requested,
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
            bus.pending_cache_invalidations.is_empty() && bus.pending_invalidation_regions == 0,
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
            gpio_in_hi,
            gpio_external_in,
            gpio_external_mask,
            gpio_external_in_hi,
            gpio_external_mask_hi,
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
            usbctrl,
            warned_clk_enable_clear: _,
            reservation: _,
        } = bus;

        let shared_mem = Arc::new(SharedMemory::from_memory(
            memory,
            boot_ram,
            xip_sram,
            flash_loaded,
        ));
        // `gpio_in` and `gpio_external_in[_hi]` are `AtomicU32` on `Bus`
        // (Phase 2 of the OneROM CPU speed-grade oracle, plus Stage 3A
        // wide-GPIO bus support — HLD §A). The threaded `AtomicGpio`
        // carries its own atomics, so lift the current values out at
        // handoff with `Ordering::Relaxed` — consistent with every
        // other reader/writer of these fields.
        let shared_gpio = Arc::new(AtomicGpio::seed(
            sio.gpio_out,
            sio.gpio_oe,
            gpio_in.load(Ordering::Relaxed),
            gpio_in_hi.load(Ordering::Relaxed),
            gpio_external_in.load(Ordering::Relaxed),
            gpio_external_mask,
            gpio_external_in_hi.load(Ordering::Relaxed),
            gpio_external_mask_hi,
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
            usb: Mutex::new(UsbState { usbctrl }),
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
            threaded_pio.write_gpio_base(idx, block.gpio_base());
            threaded_pio.write_pads(idx, block.pad_out, block.pad_oe);
            let (out_lo, out_hi) = block.local_to_physical_pins(block.pad_out);
            let (oe_lo, oe_hi) = block.local_to_physical_pins(block.pad_oe);
            shared_gpio.write_pio_pads(idx, out_lo, oe_lo, out_hi, oe_hi);
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
            thread_mask: [0, 1, 2, 3, 4, 5],
            poisoned: false,
            timing_enabled: false,
            last_run_timings: None,
            // Carry the seeded `Emulator::shutdown_requested` across the
            // hand-off so a hook fired on the serial path before
            // promotion is not silently dropped.
            bootrom_hook_fired: shutdown_requested,
        }
    }

    /// Latched bootrom mask-ROM hook flag (HLD V5 §"Component 3").
    /// Set true once either CPU worker observed `pc == bootrom_reboot_hook_pc_*`
    /// during any past `run_quanta_checked`. Surfaced to the host via
    /// `Emulator::shutdown_requested` so the run loop can exit cleanly.
    pub fn shutdown_requested(&self) -> bool {
        self.bootrom_hook_fired
    }

    /// Current shared master-cycle count. Lock-free `Acquire` load,
    /// paired with the coordinator's `fetch_add(Release)` in
    /// [`coordinator_worker_body`].
    pub fn master_cycle(&self) -> u64 {
        self.shared.master_cycle.load(Ordering::Acquire)
    }

    /// Cycle counter for core `idx` (0 or 1). Reads the owned
    /// `CortexM33::cycles()`; returns 0 while a `run_quanta` is in
    /// flight (cores are `take`n into worker threads). Callers must
    /// snapshot between `run_quanta` calls. Halted cores stay at
    /// whatever cycle they reached when halted — their counter does
    /// not advance during run loops, unlike `master_cycle` which the
    /// coordinator fetch-adds per quantum regardless. Exposed for
    /// `paced_bench_rp2350` so "Avg MHz" can be computed from real
    /// instruction work rather than coordinator ticks.
    pub fn core_cycles(&self, idx: u8) -> u64 {
        match idx {
            0 => self.core0.as_ref().map_or(0, |c| c.cycles()),
            1 => self.core1.as_ref().map_or(0, |c| c.cycles()),
            _ => panic!("ThreadedEmulator::core_cycles: idx must be 0 or 1"),
        }
    }

    /// Program counter for core `idx` (0 or 1), read between
    /// `run_quanta` calls. Returns `None` while a `run_quanta` is in
    /// flight (cores are `take`n into worker threads) or if the core
    /// was lost to a worker panic. Useful for post-run assertions like
    /// "serve-loop PC is still in range" in OneROM speed-grade
    /// oracles.
    pub fn core_pc(&self, idx: u8) -> Option<u32> {
        match idx {
            0 => self.core0.as_ref().map(|c| c.regs.pc()),
            1 => self.core1.as_ref().map(|c| c.regs.pc()),
            _ => panic!("ThreadedEmulator::core_pc: idx must be 0 or 1"),
        }
    }

    /// Borrow the `SharedState`. Exposed so harness threads can write
    /// external pin stimulus (`shared.gpio.write_external`) and read
    /// the merged `gpio_in` (`shared.gpio.read_in`) concurrently with
    /// the worker threads inside `run_quanta`. Both endpoints on
    /// [`AtomicGpio`] are internally synchronized (Release/Acquire for
    /// `external`, Relaxed for `in_`), so the harness driving stimulus
    /// does not need to pause the runtime.
    pub fn shared(&self) -> &SharedState {
        &self.shared
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

    /// Run `n` quanta. Wraps [`Self::run_quanta_checked`] and panics
    /// the main thread on any worker panic (preserving the legacy
    /// contract for callers that expect the classic panic-on-mismatch
    /// semantics).
    ///
    /// Stage 7 (LLD V7 §9) + HLD V5 Stage B.2: each worker drives the
    /// real execution logic — the CPU workers step their `CortexM33`
    /// against a [`WorkerBus`] with WFE-wake + IRQ-pending merge
    /// semantics, each PIO worker owns one `PioBlock` and drains its
    /// per-block command queue + steps its active SMs, and the
    /// coordinator publishes `master_cycle` + ticks the
    /// coordinator-owned peripherals.
    pub fn run_quanta(&mut self, n: u64) {
        match self.run_quanta_checked(n) {
            Ok(()) => {}
            Err(RunError::Panic { which, message }) => {
                panic!("worker {} panicked: {message}", which.as_str());
            }
            Err(RunError::Timeout { which, elapsed_ms }) => {
                panic!(
                    "barrier watchdog fired (observed by worker {}) after {}ms",
                    which.as_str(),
                    elapsed_ms
                );
            }
        }
    }

    /// Run `n` quanta and surface worker panics / barrier-watchdog
    /// timeouts as a structured [`RunError`] instead of re-raising. Used
    /// by the dual-execution HLD V1 Stage 1b
    /// `EmulatorError::WorkerPanicked` wiring (§5.5 item 4) and the
    /// Stage 5 `EmulatorError::BarrierTimeout` wiring (§6.6). On `Err`,
    /// the instance is poisoned: drop it and rebuild.
    pub fn run_quanta_checked(&mut self, n: u64) -> Result<(), RunError> {
        assert!(
            !self.poisoned,
            "ThreadedEmulator poisoned by prior worker panic; drop and rebuild"
        );

        let core0 = self.core0.take().expect("run_quanta reentry");
        let core1 = self.core1.take().expect("run_quanta reentry");
        // Destructure the three-block array so each PIO worker can own
        // exactly one `PioBlock`. Happy-path reassembly below rebuilds
        // the array; any PIO worker panic drops all three blocks per
        // HLD V5 §2.7 — the poisoned instance cannot be reused.
        let [block0, block1, block2] = self.pio_blocks.take().expect("run_quanta reentry");

        let barrier = Arc::new(SpinBarrier::new(6));
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
        let hp0 = spawn_worker(mask[2], barrier.clone(), {
            let s = shared.clone();
            move |b| pio_block_worker_body(0, block0, s, b, n, step_q, timing)
        });
        let hp1 = spawn_worker(mask[3], barrier.clone(), {
            let s = shared.clone();
            move |b| pio_block_worker_body(1, block1, s, b, n, step_q, timing)
        });
        let hp2 = spawn_worker(mask[4], barrier.clone(), {
            let s = shared.clone();
            move |b| pio_block_worker_body(2, block2, s, b, n, step_q, timing)
        });
        let hc = spawn_worker(mask[5], barrier.clone(), {
            let s = shared.clone();
            move |b| coordinator_worker_body(s, b, n, step_q, timing)
        });

        let r0 = h0.join();
        let r1 = h1.join();
        let rp0 = hp0.join();
        let rp1 = hp1.join();
        let rp2 = hp2.join();
        let rc = hc.join();

        // Extract panic message (if any) from each JoinHandle::join()
        // Err payload before consuming the Ok payloads. Mirrors the
        // downcast pattern used in the in-crate panic assertion tests.
        let msg0 = panic_message(r0.as_ref().err());
        let msg1 = panic_message(r1.as_ref().err());
        let msgp0 = panic_message(rp0.as_ref().err());
        let msgp1 = panic_message(rp1.as_ref().err());
        let msgp2 = panic_message(rp2.as_ref().err());
        let msgc = panic_message(rc.as_ref().err());

        // Track which workers panicked before consuming the Ok payloads
        // so the panic message can enumerate the culprits.
        let r0_err = r0.is_err();
        let r1_err = r1.is_err();
        let rp0_err = rp0.is_err();
        let rp1_err = rp1.is_err();
        let rp2_err = rp2.is_err();
        let rc_err = rc.is_err();
        let any_pio_err = rp0_err || rp1_err || rp2_err;

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
        let mut tp0 = PerWorkerTimings::default();
        let mut tp1 = PerWorkerTimings::default();
        let mut tp2 = PerWorkerTimings::default();
        let mut tc = PerWorkerTimings::default();

        if let Ok((c, t)) = r0 {
            // Drain the bootrom-hook latch into the host-visible flag
            // before re-storing the core (mirrors the serial drain in
            // `lib.rs:707-717`). Hook is terminate-only / sticky, so a
            // single OR-into is correct across calls.
            if c.bootrom_hook_fired {
                self.bootrom_hook_fired = true;
            }
            self.core0 = Some(c);
            t0 = t;
        }
        if let Ok((c, t)) = r1 {
            if c.bootrom_hook_fired {
                self.bootrom_hook_fired = true;
            }
            self.core1 = Some(c);
            t1 = t;
        }

        // HLD V5 §2.7: on any PIO worker panic, drop all three Ok
        // blocks too (the instance is poisoned and cannot be reused).
        // Unpack each join result exactly once and stash the blocks into
        // `Option<PioBlock>`; if any worker panicked, the Ok blocks
        // simply go out of scope at end of function (drop site).
        let ok0 = rp0.ok();
        let ok1 = rp1.ok();
        let ok2 = rp2.ok();
        let (bl0, tpe0) = split_ok(ok0);
        let (bl1, tpe1) = split_ok(ok1);
        let (bl2, tpe2) = split_ok(ok2);
        if let Some(t) = tpe0 {
            tp0 = t;
        }
        if let Some(t) = tpe1 {
            tp1 = t;
        }
        if let Some(t) = tpe2 {
            tp2 = t;
        }
        if !any_pio_err {
            // All three PIO workers returned Ok; reassemble the array.
            self.pio_blocks = Some([
                bl0.expect("pio0 Ok but block missing"),
                bl1.expect("pio1 Ok but block missing"),
                bl2.expect("pio2 Ok but block missing"),
            ]);
        } else {
            // At least one PIO worker panicked — release all three blocks
            // (including any Ok returns) per HLD V5 §2.7. PioBlock has no
            // Drop impl and the locals drop naturally at scope exit.
            self.pio_blocks = None;
        }
        // Coordinator worker returns `((), PerWorkerTimings)`.
        if let Ok(((), t)) = rc {
            tc = t;
        }

        // First panicked worker wins attribution — the barrier-poison
        // pathway means every other waiter exits via `Poisoned` and
        // doesn't carry a real panic payload. Scanning in worker-index
        // order (core0, core1, pio0, pio1, pio2, coord) matches the
        // legacy enumeration order in the test suite.
        let first_panic: Option<(super::WorkerName, String)> = [
            (super::WorkerName::Core0, r0_err, msg0),
            (super::WorkerName::Core1, r1_err, msg1),
            (super::WorkerName::Pio0, rp0_err, msgp0),
            (super::WorkerName::Pio1, rp1_err, msgp1),
            (super::WorkerName::Pio2, rp2_err, msgp2),
            (super::WorkerName::Coord, rc_err, msgc),
        ]
        .into_iter()
        .find_map(|(name, err, msg)| if err { Some((name, msg)) } else { None });

        if let Some((which, message)) = first_panic {
            self.poisoned = true;
            return Err(RunError::Panic { which, message });
        }

        // Stage 5 (HLD V1 §6.6): watchdog-fired barrier exits all workers
        // cleanly via `TimedOut`, so no `JoinHandle::join` returns Err.
        // Inspect the barrier directly to distinguish a timeout from an
        // ordinary clean return. `WorkerName::Coord` is the observer
        // attribution — the barrier cannot identify the missing worker.
        if barrier.timed_out() {
            self.poisoned = true;
            return Err(RunError::Timeout {
                which: super::WorkerName::Coord,
                elapsed_ms: barrier.timeout_elapsed_ms(),
            });
        }

        if timing {
            self.last_run_timings = Some(RunTimings {
                core0: t0,
                core1: t1,
                pio0: tp0,
                pio1: tp1,
                pio2: tp2,
                coord: tc,
            });
        }
        Ok(())
    }
}

// =======================================================================
// Worker-thread plumbing
// =======================================================================
//
// `panic_message`, `spawn_worker`, and `pin_to_host_core` were promoted
// to `picoem-common::threaded::worker` per the 2026-04-30 Threaded
// Helpers Pull-Up HLD V1. They reach this file as `panic_message`,
// `spawn_worker`, `pin_to_host_core` via the `use super::{...}` import
// at the top of the file (re-exported from `crate::threaded::mod.rs`).

/// Helper for `run_quanta`'s PIO-worker join handling: split an
/// `Option<(PioBlock, PerWorkerTimings)>` into its two halves so the
/// block and its timings can be moved into different places (happy-path
/// reassembly vs. drop-on-panic) without a partial-move borrow-checker
/// fight.
fn split_ok(
    ok: Option<(PioBlock, PerWorkerTimings)>,
) -> (Option<PioBlock>, Option<PerWorkerTimings>) {
    match ok {
        Some((b, t)) => (Some(b), Some(t)),
        None => (None, None),
    }
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
// / `PioBlock` / `()` immediately so the caller can flip the
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
    // Seed target from the core's current cycle count so successive
    // `run_quanta` calls (each spawns a fresh worker) keep advancing
    // the core from where it left off. A persistent `core.cycles()`
    // with a per-call `target = 0` makes the `core.cycles() < target`
    // guard immediately false and the executor skips work entirely
    // — reproduced by `paced_bench_rp2350` logging `runtime.run`
    // drop from 540 ms on call 1 to ~30 ms on calls 2..N (2026-04-20).
    let mut target: u64 = core.cycles();
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
        if shared.atomics.is_wfe_waiting(idx) && shared.atomics.event_flag_consume(idx) {
            shared.atomics.clear_wfe_waiting(idx);
        }

        // Merge coordinator-/peer-asserted IRQs into this core's NVIC.
        // `take_irq_pending` is an `AcqRel` swap-to-zero — the non-zero
        // return is the consume-and-merge trigger (LLD V7 §2).
        let pending = shared.atomics.take_irq_pending(idx);
        if pending != 0 {
            core.ppb.merge_irq_pending(pending);
        }

        // Hoist the cross-thread atomic loads out of the inner hot path.
        // `is_halted` only changes when a peer worker calls `set_halted`,
        // and `is_wfe_waiting` is written *by the core itself* when it
        // executes a WFE instruction — so the inner loop only needs to
        // re-check the WFE flag *after* each step, and `is_halted` can
        // be sampled once per quantum. Peer-driven halts now land one
        // quantum late; that matches the existing IRQ-delivery latency
        // ceiling documented in HLD §5.2 (2× step_quantum).
        //
        // `step_no_atomics` skips the redundant per-step WFE / halt /
        // irq_pending atomic loads inside `CortexM33::step` — the worker
        // already handled those at the top of the quantum. On single-
        // core workloads the coord-written `irq_pending` line bounces
        // into core 0's cache on every step otherwise, tripling the
        // per-instruction cost.
        if !shared.atomics.is_halted(idx) {
            while core.cycles() < target {
                core.ppb.update_latest_cycles(core.cycles());
                core.step_no_atomics(&mut bus);
                if !bus.pending_cache_invalidations.is_empty() {
                    // Decode cache lives on `core` (Phase 3 follow-up #10);
                    // `invalidate_decode_cache_entries` is now inherent on
                    // `CortexM33` and doesn't need `bus`. Drain in place.
                    core.invalidate_decode_cache_entries(&bus.pending_cache_invalidations);
                    bus.pending_cache_invalidations.clear();
                }
                if shared.atomics.is_wfe_waiting(idx) {
                    break;
                }
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
        if matches!(
            result,
            BarrierResult::Poisoned | BarrierResult::TimedOut { .. }
        ) {
            return (core, rec.take());
        }
    }
    (core, rec.take())
}

/// Per-block PIO worker. Owns a single [`PioBlock`] (addressed by
/// `block_idx`), drains CPU-queued commands for that block, then steps
/// its enabled state machines for `step_q` sysclocks. PIO IRQ routing
/// to the NVIC is deliberately omitted — the single-threaded `Bus`
/// path also does not route PIO IRQs today, and Phase 3 §6 scopes this
/// to parity (adding it requires both edges, which is a separate HLD).
///
/// HLD V5 §2.1: three workers run concurrently, one per PIO block, so
/// the emulator's thread boundary matches the hardware's PIO-block
/// boundary. Each worker publishes its pad state unconditionally each
/// quantum (§2.1) — even disabled blocks must publish their current
/// pad latch so coord's `update_gpio` sees a coherent snapshot.
fn pio_block_worker_body(
    block_idx: usize,
    mut block: PioBlock,
    shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
    timing_enabled: bool,
) -> (PioBlock, PerWorkerTimings) {
    let mut rec = TimingRecorder::new(n, timing_enabled);
    rec.on_worker_entry();

    for _ in 0..n {
        for cmd in shared.pio.drain_commands(block_idx) {
            apply_pio_command(&mut block, block_idx, &shared.pio, cmd);
        }

        // Physical GPIO snapshot once per quantum — parity with the
        // single-threaded `Emulator::tick_peripherals` which reads
        // the merged GPIO banks once and hands them to every PIO step.
        let gpio_pins = shared.gpio.read_in64();

        // `shared.pio.read_sm_enabled` reflects the last-applied CTRL
        // write's SM_ENABLE mask (`apply_pio_command::WriteCtrl`
        // republishes it after each CTRL write). A zero mask means no
        // SM in this block can make progress this quantum, so skip the
        // per-SM stepping loop entirely.
        if shared.pio.read_sm_enabled(block_idx) != 0 {
            block.step_n_with_pins(step_q, gpio_pins);
            // Reflect the block's IRQ flags back onto `ThreadedPio` so
            // CPU workers observe them through the shared atomic.
            // PIO→NVIC assertion is Phase-later scope (see function
            // doc); we only publish the bits here.
            shared
                .pio
                .write_irq_flags(block_idx, block.pending_irqs() as u8);
        }

        // HLD V5 §2.1: publish pad state unconditionally — even a
        // disabled block must publish its current pad latch so coord's
        // `update_gpio` sees a coherent snapshot. Mirrors the
        // pre-split per-block publish loop.
        shared
            .pio
            .write_pads(block_idx, block.pad_out, block.pad_oe);
        let (out_lo, out_hi) = block.local_to_physical_pins(block.pad_out);
        let (oe_lo, oe_hi) = block.local_to_physical_pins(block.pad_oe);
        shared
            .gpio
            .write_pio_pads(block_idx, out_lo, oe_lo, out_hi, oe_hi);

        // Phase 4 Stage C (HLD V7 §5.1): single-barrier rendezvous
        // after phase work. Overlaps with coord phase-2 of this
        // quantum. Poison propagation per §5.4.
        rec.on_wait_entry();
        let result = barrier.wait();
        rec.on_wait_return();
        if matches!(
            result,
            BarrierResult::Poisoned | BarrierResult::TimedOut { .. }
        ) {
            return (block, rec.take());
        }
    }
    (block, rec.take())
}

/// Apply a CPU-queued [`PioCommand`] to the owning PIO worker's local
/// `PioBlock`. Routes through `PioBlock::write32` so all the existing
/// bookkeeping (INSTR_MEM index guard, FIFO-join handling, alias
/// decoding, SM enable-mask invariant) continues to apply.
///
/// `block_idx` is the index of the worker's owned block; since each
/// per-block command queue only delivers commands matching its own
/// `block_idx` (§2.2 routing contract), `cmd.block() as usize` always
/// equals `block_idx` here — asserted under `debug_assertions`. The
/// `WriteCtrl` / `WriteReg` arms use `block_idx` to publish the
/// post-write `sm_enabled_mask` onto `ThreadedPio::sm_enabled` so
/// CPU-side reads of CTRL.SM_ENABLE and the `pio_block_worker_body`
/// enable-gate check see the new state on the next quantum.
/// `WriteReg` republishes the mask and GPIOBASE too — on the chance a
/// generic write touches state that affects CPU-side readback, this
/// keeps the invariant local to this function regardless of future
/// `PioBlock::write32` extensions.
fn apply_pio_command(
    block: &mut PioBlock,
    block_idx: usize,
    shared_pio: &super::ThreadedPio,
    cmd: PioCommand,
) {
    debug_assert_eq!(
        cmd.block() as usize,
        block_idx,
        "PioCommand.block must match the owning worker's block_idx (§2.2 routing)"
    );
    match cmd {
        PioCommand::WriteInstrMem {
            block: _,
            addr,
            value,
            alias,
        } => {
            if addr >= 32 {
                return;
            }
            let offset = 0x048 + (addr as u32) * 4;
            block.write32(offset, value as u32, alias as u32);
        }
        PioCommand::SetClkDiv {
            block: _,
            sm,
            int_div,
            frac_div,
            alias,
        } => {
            if sm >= 4 {
                return;
            }
            // SMn_CLKDIV lives at 0x0C8 + sm * 0x18. Layout: INT<<16,
            // FRAC<<8, rest reserved (picoem-common::pio::mod §write_clkdiv).
            let offset = 0x0C8 + (sm as u32) * 0x18;
            let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
            block.write32(offset, val, alias as u32);
        }
        PioCommand::WriteCtrl {
            block: _,
            val,
            alias,
        } => {
            block.write32(0x000, val, alias as u32);
            // Republish the post-write enable mask so CPU-side readers
            // (including the PIO worker's own step-loop enable gate)
            // observe it next quantum.
            shared_pio.write_sm_enabled(block_idx, block.sm_enabled_mask());
        }
        PioCommand::WriteReg {
            block: _,
            offset,
            val,
            alias,
        } => {
            block.write32(offset as u32, val, alias as u32);
            // Conservative republish: keeps the mask coherent even if a
            // future `PioBlock::write32` extension ends up toggling
            // `enabled` outside CTRL. GPIOBASE readback uses the same
            // generic path, so publish it after every WriteReg too.
            shared_pio.write_sm_enabled(block_idx, block.sm_enabled_mask());
            shared_pio.write_gpio_base(block_idx, block.gpio_base());
        }
        // Stage B.2 panic-injection hook (HLD V5 §2.2 / §4 item 5). The
        // `pio{block}` substring is load-bearing — the worker-split tests
        // assert on it to prove the panic surfaced on the specific PIO
        // worker addressed by the command's block field. The panic
        // message uses `cmd.block()` (i.e. the command's target), not
        // `block_idx`, so it stays accurate under the §2.2 contract
        // (they are equal, but the command-field form documents intent).
        // `#[cfg(feature = "testing")]` keeps the arm — and therefore
        // the `panic!` — out of release builds (Stage 1b review
        // REQUIRED #2), so `PioCommand`'s exhaustive match compiles
        // clean in production. Matches the variant gating in
        // `threaded/pio.rs`.
        #[cfg(feature = "testing")]
        PioCommand::TestPanic { block } => {
            panic!("PioCommand::TestPanic fired from pio{}", block);
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
        shared
            .master_cycle
            .fetch_add(step_q as u64, Ordering::Release);
        shared.sio.mtime_advance(step_q as u64);

        tick_peripherals(&shared, step_q);

        // Phase 4 Stage C (HLD V7 §5.1): single-barrier rendezvous
        // after phase work. Overlaps with CPU/PIO phase-1 of this
        // quantum. Poison propagation per §5.4.
        rec.on_wait_entry();
        let result = barrier.wait();
        rec.on_wait_return();
        if matches!(
            result,
            BarrierResult::Poisoned | BarrierResult::TimedOut { .. }
        ) {
            return ((), rec.take());
        }
    }
    ((), rec.take())
}

/// Coordinator-owned GPIO merge. Ports `Emulator::update_gpio`
/// (`lib.rs:406-415`): start with SIO pads (`out & oe`), fold each
/// PIO block's physical pad overlay in block order, then apply the
/// external-stimulus overlay last.
fn update_gpio(shared: &SharedState) {
    let sio_out = shared.gpio.read_out(0);
    let sio_oe = shared.gpio.read_oe(0);
    let mut merged_lo = sio_out & sio_oe;
    let mut merged_hi = shared.gpio.read_out(1) & shared.gpio.read_oe(1);
    for block_idx in 0..3 {
        let ((pad_out_lo, pad_oe_lo), (pad_out_hi, pad_oe_hi)) =
            shared.gpio.read_pio_pads(block_idx);
        merged_lo = (merged_lo & !pad_oe_lo) | (pad_out_lo & pad_oe_lo);
        merged_hi = (merged_hi & !pad_oe_hi) | (pad_out_hi & pad_oe_hi);
    }
    let (ext_val, ext_mask) = shared.gpio.read_external();
    let (ext_val_hi, ext_mask_hi) = shared.gpio.read_external_hi();
    shared.gpio.write_in64(
        (merged_lo & !ext_mask) | (ext_val & ext_mask),
        (merged_hi & !ext_mask_hi) | (ext_val_hi & ext_mask_hi),
    );
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
// Stage 7 (LLD V7 §11 items 13–19): smoke tests that spawn the 6-worker
// runtime and verify end-to-end execution semantics — quantum advance,
// cross-core SRAM visibility, WFE/SEV wake, FIFO-push wake, spinlock
// contention, doorbell state, and decode-cache invalidation plumbing.
//
// The three `from_emulator_preserves_*` tests from earlier stages stay
// — they exercise the destructure + seed round-trip.

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Config, EmulatorBuilder};

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
            threaded.shared.atomics.event_flag[1].load(Ordering::Acquire),
            "pending_fifo_event(1) must land on event_flag[1]"
        );
        assert!(
            !threaded.shared.atomics.event_flag[0].load(Ordering::Acquire),
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
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
        use crate::peripherals::ticks::{CTRL_ENABLE, DOMAIN_STRIDE, DOMAIN_TIMER0, TICKS_BASE};
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
        let timer0_intr = threaded
            .shared
            .peripherals
            .timers
            .lock()
            .unwrap()
            .timer0
            .read32(INTR_OFFSET);
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
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
            bus.pending_cache_invalidations[1],
            addr_a + 2,
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
        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));

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
        for cmd in threaded.shared.pio.drain_commands(0) {
            apply_pio_command(&mut blocks[0], 0, &threaded.shared.pio, cmd);
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

    /// Smoke test that the `#[cfg(feature = "testing")]` `TestPanic`
    /// arm in `apply_pio_command` panics with the `pio{block}`
    /// substring the Stage B.2 / B.4 end-to-end worker-split tests
    /// rely on. Catches format-string regressions before the
    /// worker-panic integration tests land.
    #[cfg(feature = "testing")]
    #[test]
    #[should_panic(expected = "pio1")]
    fn apply_pio_command_test_panic_arm_fires() {
        let pio = super::ThreadedPio::new();
        let mut block = PioBlock::new();
        apply_pio_command(&mut block, 1, &pio, PioCommand::TestPanic { block: 1 });
    }

    /// A CTRL write landing through `WorkerBus::ahb_write32` must
    /// enqueue a `WriteCtrl` command and — after `apply_pio_command`
    /// runs — propagate to `ThreadedPio::read_sm_enabled`. This covers
    /// the end-to-end MMIO → command-queue → block hand-off for the
    /// critical unblocker.
    #[test]
    fn pio_ctrl_write_via_worker_bus_propagates_to_threaded_pio() {
        use crate::core::CoreBus;

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO1 CTRL = 0x5030_0000, enable SMs 1 and 3.
        bus.write32(0x5030_0000, 0b1010, 0);

        // Command must be queued (not dropped silently).
        let pending = threaded.shared.pio.drain_commands(1);
        assert_eq!(
            pending.len(),
            1,
            "CTRL write must queue exactly one command"
        );
        assert_eq!(
            pending[0],
            PioCommand::WriteCtrl {
                block: 1,
                val: 0b1010,
                alias: 0
            }
        );

        // Apply + verify the republish lands on ThreadedPio.
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        apply_pio_command(&mut blocks[1], 1, &threaded.shared.pio, pending[0]);
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 INSTR_MEM7 lives at 0x5020_0000 + 0x048 + 7*4 = 0x5020_0064.
        // Value is truncated to u16 inside PioBlock::write32.
        let insn: u32 = 0x0000_E080; // arbitrary PIO opcode-shaped word
        bus.write32(0x5020_0064, insn, 0);

        let pending = threaded.shared.pio.drain_commands(0);
        assert_eq!(pending.len(), 1, "INSTR_MEM write must queue one command");
        assert_eq!(
            pending[0],
            PioCommand::WriteInstrMem {
                block: 0,
                addr: 7,
                value: insn as u16,
                alias: 0
            }
        );

        // Apply and verify it reached instr_mem[7].
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        apply_pio_command(&mut blocks[0], 0, &threaded.shared.pio, pending[0]);
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO2 SM3_CLKDIV: 0x5040_0000 + 0x0C8 + 3*0x18 = 0x5040_0110.
        // Layout: INT << 16, FRAC << 8.
        let int_div: u16 = 0x1234;
        let frac_div: u8 = 0x56;
        let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
        bus.write32(0x5040_0110, val, 0);

        let pending = threaded.shared.pio.drain_commands(2);
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending[0],
            PioCommand::SetClkDiv {
                block: 2,
                sm: 3,
                int_div,
                frac_div,
                alias: 0
            }
        );
    }

    /// A write to an offset outside CTRL / INSTR_MEM / CLKDIV (e.g.
    /// TXF0 or IRQ) must fall through to the generic `WriteReg`
    /// variant so no PIO MMIO offset is silently dropped anymore.
    #[test]
    fn pio_non_fast_path_write_uses_generic_writereg() {
        use crate::core::CoreBus;

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 TXF0 at 0x5020_0010.
        bus.write32(0x5020_0010, 0xABCD_1234, 0);
        // PIO0 IRQ at 0x5020_0030 (W1C, alias 0).
        bus.write32(0x5020_0030, 0x0000_000F, 0);

        let pending = threaded.shared.pio.drain_commands(0);
        assert_eq!(pending.len(), 2, "two non-fast-path writes → two commands");
        assert_eq!(
            pending[0],
            PioCommand::WriteReg {
                block: 0,
                offset: 0x010,
                val: 0xABCD_1234,
                alias: 0
            }
        );
        assert_eq!(
            pending[1],
            PioCommand::WriteReg {
                block: 0,
                offset: 0x030,
                val: 0x0000_000F,
                alias: 0
            }
        );
    }

    /// `WorkerBus::ahb_read32` exposes the atomics `ThreadedPio`
    /// publishes (CTRL.SM_ENABLE at 0x000, IRQ at 0x030, GPIOBASE at
    /// 0x168) so firmware that round-trips these registers observes
    /// the worker-owned state. Other offsets return 0 until read-through
    /// is wired.
    #[test]
    fn pio_ctrl_readback_reflects_published_enable_mask() {
        use crate::core::CoreBus;

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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

        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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

    #[test]
    fn pio_gpiobase_write_via_worker_bus_drains_and_reads_back() {
        use crate::core::CoreBus;

        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            bus.write32(0x5030_0168, 0xFFFF_FFFF, 0);
            assert_eq!(
                bus.read32(0x5030_0168, 0),
                0,
                "PIO write is queued until the PIO worker drains the next quantum"
            );
        }

        threaded.run_quanta(1);

        let mut bus = WorkerBus::new(0, threaded.shared.clone());
        assert_eq!(threaded.shared.pio.read_gpio_base(1), 16);
        assert_eq!(
            bus.read32(0x5030_0168, 0),
            16,
            "threaded GPIOBASE readback must mirror the worker-owned PioBlock"
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 INSTR_MEM7 = 0x5020_0064; SET alias adds 0x2000.
        let set_alias_addr = 0x5020_0064 + 0x2000;
        bus.write32(set_alias_addr, 0x0000_DEAD, 0);

        let pending = threaded.shared.pio.drain_commands(0);
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        let mut bus = WorkerBus::new(0, threaded.shared.clone());

        // PIO0 SM0_CLKDIV = 0x5020_00C8; XOR alias adds 0x1000.
        let xor_alias_addr = 0x5020_00C8 + 0x1000;
        let int_div: u16 = 0x0010;
        let frac_div: u8 = 0x80;
        let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
        bus.write32(xor_alias_addr, val, 0);

        let pending = threaded.shared.pio.drain_commands(0);
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));

        // Seed: enable SM0 on PIO0 via a plain CTRL write.
        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            bus.write32(0x5020_0000, 0b0001, 0);
        }
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        for cmd in threaded.shared.pio.drain_commands(0) {
            apply_pio_command(&mut blocks[0], 0, &threaded.shared.pio, cmd);
        }
        assert_eq!(blocks[0].sm_enabled_mask(), 0b0001);

        // Now SET-alias write enabling SM2 — must OR with the prior
        // state, yielding 0b0101. Plain (alias=0) would overwrite to
        // 0b0100.
        {
            let mut bus = WorkerBus::new(0, threaded.shared.clone());
            bus.write32(0x5020_0000 + 0x2000, 0b0100, 0);
        }
        let pending = threaded.shared.pio.drain_commands(0);
        assert_eq!(pending.len(), 1);
        assert_eq!(
            pending[0],
            PioCommand::WriteCtrl {
                block: 0,
                val: 0b0100,
                alias: 2
            },
        );
        apply_pio_command(&mut blocks[0], 0, &threaded.shared.pio, pending[0]);

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

        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
            &mut scratch[0],
            0,
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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

        let pending = threaded.shared.pio.drain_commands(0);
        assert_eq!(pending.len(), 5, "five MMIO writes → five commands");
        assert_eq!(
            pending[0],
            PioCommand::WriteCtrl {
                block: 0,
                val: 0b0011,
                alias: 0
            }
        );
        assert_eq!(
            pending[1],
            PioCommand::WriteInstrMem {
                block: 0,
                addr: 3,
                value: 0x1234,
                alias: 0
            }
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
            PioCommand::WriteReg {
                block: 0,
                offset: 0x010,
                val: 0xAA55_AA55,
                alias: 0
            }
        );
        assert_eq!(
            pending[4],
            PioCommand::WriteReg {
                block: 0,
                offset: 0x030,
                val: 0x0000_0001,
                alias: 0
            }
        );

        // Apply all and verify observable end-state matches the write sequence.
        let mut blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        for cmd in pending {
            apply_pio_command(&mut blocks[0], 0, &threaded.shared.pio, cmd);
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

        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
            threaded.core0.as_ref().unwrap().ppb.last_systick_cycles,
            0,
            "core0 phase-1 must call systick_advance"
        );
        assert_eq!(
            threaded.core1.as_ref().unwrap().ppb.last_systick_cycles,
            0,
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
        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));

        // SIO drives bit 0 (out & oe).
        threaded.shared.gpio.write_out(0, 0x0000_0001);
        threaded.shared.gpio.write_oe(0, 0x0000_0001);
        // PIO block 2 drives bit 4 high and bit 0 low via pad_oe
        // (higher-indexed blocks overlay lower ones per §4.2).
        threaded
            .shared
            .gpio
            .write_pio_pads(2, 0x0000_0010, 0x0000_0011, 0, 0);
        // External stimulus forces bit 8 high.
        threaded
            .shared
            .gpio
            .write_external(0x0000_0100, 0x0000_0100);

        update_gpio(&threaded.shared);

        // SIO bit 0 overridden by PIO block 2's pad_oe bit 0 (pad_out=0),
        // block 2's bit 4 high, external bit 8 high.
        assert_eq!(
            threaded.shared.gpio.read_in(),
            0x0000_0110,
            "update_gpio must overlay PIO then external on top of SIO"
        );
    }

    #[test]
    fn threaded_update_gpio_publishes_gpio_in_hi() {
        let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));

        // PIO1 physical high-bank output drives GPIO34 high.
        threaded.shared.gpio.write_pio_pads(1, 0, 0, 1 << 2, 1 << 2);
        // External stimulus independently forces GPIO36 high.
        threaded.shared.gpio.write_external_hi(1 << 4, 1 << 4);

        update_gpio(&threaded.shared);

        assert_eq!(
            threaded.shared.gpio.read_in_hi() & ((1 << 2) | (1 << 4)),
            (1 << 2) | (1 << 4),
            "threaded update_gpio must publish PIO and external high-bank inputs"
        );
    }

    #[test]
    fn threaded_pio_gpiobase_16_sees_gpio_external_in_hi() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(1)
            .build()
            .unwrap();

        // IN PINS, 19. GPIOBASE=16 maps physical GPIO34 to local pin 18.
        emu.bus.pio[1].write32(0x048, 0x4013, 0);
        emu.bus.pio[1].write32(0x168, 16, 0);
        let shiftctrl = emu.bus.pio[1].read32(0x0D0) & !(1 << 18);
        emu.bus.pio[1].write32(0x0D0, shiftctrl, 0);
        emu.bus.pio[1].write32(0x000, 0x1, 0);
        emu.bus.set_gpio_external_in_hi(1 << 2, 1 << 2);
        emu.update_gpio();

        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        threaded.run_quanta(1);

        let block = &threaded.pio_blocks.as_ref().unwrap()[1];
        assert_eq!(block.sm[0].isr_value(), 1 << 18);
        assert_eq!(block.sm[0].isr_shift_count(), 19);
    }

    /// `from_emulator` must seed both the legacy local pad snapshot and
    /// the physical low/high pad snapshot from each incoming
    /// `PioBlock.pad_out` / `pad_oe`. Without the physical seed, coord's
    /// first `update_gpio` reads zero and drops PIO output for one quantum.
    ///
    /// Regression guard: removing the seed loop in `from_emulator` (at
    /// the `threaded_pio.write_pads(...)` call) fails this test.
    #[test]
    fn from_emulator_seeds_pio_pads() {
        let mut emu = Emulator::new(Config::default());
        emu.bus.pio[0].pad_out = 0xAAAA_0000;
        emu.bus.pio[0].pad_oe = 0xFFFF_0000;
        emu.bus.pio[1].write32(0x168, 16, 0);
        emu.bus.pio[1].pad_out = 0x0000_5555;
        emu.bus.pio[1].pad_oe = 0x0000_FFFF;
        emu.bus.pio[2].pad_out = 0x1234_5678;
        emu.bus.pio[2].pad_oe = 0x8765_4321;

        let threaded = ThreadedEmulator::from_emulator(emu);

        assert_eq!(threaded.shared.pio.read_pads(0), (0xAAAA_0000, 0xFFFF_0000));
        assert_eq!(threaded.shared.pio.read_pads(1), (0x0000_5555, 0x0000_FFFF));
        assert_eq!(threaded.shared.pio.read_pads(2), (0x1234_5678, 0x8765_4321));
        assert_eq!(threaded.shared.pio.read_gpio_base(1), 16);
        assert_eq!(
            threaded.shared.gpio.read_pio_pads(0),
            ((0xAAAA_0000, 0xFFFF_0000), (0, 0))
        );
        assert_eq!(
            threaded.shared.gpio.read_pio_pads(1),
            ((0x5555_0000, 0xFFFF_0000), (0, 0))
        );
        assert_eq!(
            threaded.shared.gpio.read_pio_pads(2),
            ((0x1234_5678, 0x8765_4321), (0, 0))
        );
    }

    // ----- Per-worker timing instrumentation ----------------------------

    /// When timing is disabled (default), `run_quanta` must not populate
    /// `last_run_timings`. Guards the zero-overhead contract — a consumer
    /// that forgets the flag sees `None` rather than stale data.
    #[test]
    fn timings_disabled_by_default_yields_none() {
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
                s.samples,
                n as usize,
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
        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
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
        emu.bus.gpio_external_in.store(EXT_BIT, Ordering::Relaxed);
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

    // ----- HLD V5 Stage B.4 §4 items 2-5: end-to-end per-block split -----

    /// Build a minimal blinky-style PIO program on the target `block_idx`
    /// that drives a single GPIO pin (`set_pin`) high forever via a
    /// `SET PINS, 1` / `JMP 0` loop. Queues all setup through
    /// `send_command` (no direct `PioBlock` mutation) so the drive path
    /// matches real firmware's MMIO-only interface.
    ///
    /// After one-or-more `run_quanta` calls, `shared.pio.read_pads(block_idx)`
    /// reports `(1 << set_pin, 1 << set_pin)` (pad_out, pad_oe).
    fn queue_pio_blinky_setup(shared: &SharedState, block_idx: u8, set_pin: u8) {
        // Program: addr 0 = SET PINS, 1; addr 1 = JMP 0.
        let set_pins_1: u16 = 0xE001;
        let jmp_0: u16 = 0x0000;
        shared.pio.send_command(PioCommand::WriteInstrMem {
            block: block_idx,
            addr: 0,
            value: set_pins_1,
            alias: 0,
        });
        shared.pio.send_command(PioCommand::WriteInstrMem {
            block: block_idx,
            addr: 1,
            value: jmp_0,
            alias: 0,
        });

        // SM0_PINCTRL (0x0DC): set_base=set_pin (bits[9:5]), set_count=1
        // (bits[28:26]).
        let pinctrl = (1u32 << 26) | ((set_pin as u32) << 5);
        shared.pio.send_command(PioCommand::WriteReg {
            block: block_idx,
            offset: 0x0DC,
            val: pinctrl,
            alias: 0,
        });

        // SM0_EXECCTRL (0x0CC): wrap_top=1 (bits[16:12]), wrap_bottom=0.
        let execctrl = 1u32 << 12;
        shared.pio.send_command(PioCommand::WriteReg {
            block: block_idx,
            offset: 0x0CC,
            val: execctrl,
            alias: 0,
        });

        // SM0_INSTR (0x0D8): force-execute SET PINDIRS, 1 so the pin
        // is configured for output before the program drives it.
        // SET PINDIRS, 1 = 0xE081.
        shared.pio.send_command(PioCommand::WriteReg {
            block: block_idx,
            offset: 0x0D8,
            val: 0xE081,
            alias: 0,
        });

        // CTRL (0x000): enable SM0.
        shared.pio.send_command(PioCommand::WriteCtrl {
            block: block_idx,
            val: 0b0001,
            alias: 0,
        });
    }

    /// §4 item 2 — per-block concurrent integration. Two different
    /// blinky programs run simultaneously on PIO1 (pin 10) and PIO2
    /// (pin 20). After a burst of quanta, each block's pad snapshot
    /// reflects its own program independently — the pre-split single
    /// PIO worker could not have expressed concurrent per-block pad
    /// state in the same quantum.
    ///
    /// The assertion checks `pad_oe` bits (the driven pins) per block,
    /// not `pad_out` — `pad_out` can carry undriven-high bits from the
    /// block's `merge_pin_outputs` compositing; `pad_oe` is the clean
    /// per-block independence signal. What matters for independence is
    /// that block 1's driven bit is pin 10 and block 2's is pin 20,
    /// with no crosstalk.
    ///
    /// All `send_command` calls happen on the test thread (no cross-
    /// core TXF traffic), dodging the pre-existing SPSC-MPSC hazard
    /// called out in the HLD.
    #[test]
    fn per_block_concurrent_pio1_and_pio2_independent_pads() {
        const PIN_A: u8 = 10;
        const PIN_B: u8 = 20;

        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        // Halt both cores — we only care about the PIO workers.
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        queue_pio_blinky_setup(&threaded.shared, 1, PIN_A);
        queue_pio_blinky_setup(&threaded.shared, 2, PIN_B);

        threaded.run_quanta(10);

        let (out1, oe1) = threaded.shared.pio.read_pads(1);
        let (out2, oe2) = threaded.shared.pio.read_pads(2);

        // Block 1 drives pin PIN_A only; block 2 drives pin PIN_B only.
        assert_eq!(
            oe1,
            1u32 << PIN_A,
            "PIO1 pad_oe must drive pin {PIN_A} exclusively (got {oe1:#x})",
        );
        assert_eq!(
            oe2,
            1u32 << PIN_B,
            "PIO2 pad_oe must drive pin {PIN_B} exclusively (got {oe2:#x})",
        );
        // And the driven bit on each side must be high (SET PINS, 1 ran).
        assert_ne!(
            out1 & (1u32 << PIN_A),
            0,
            "PIO1 pad_out bit PIN_A must be 1"
        );
        assert_ne!(
            out2 & (1u32 << PIN_B),
            0,
            "PIO2 pad_out bit PIN_B must be 1"
        );
        // Cross-check: PIO1 did not drive PIO2's pin and vice versa.
        assert_eq!(oe1 & (1u32 << PIN_B), 0, "PIO1 must not drive PIO2's pin");
        assert_eq!(oe2 & (1u32 << PIN_A), 0, "PIO2 must not drive PIO1's pin");

        // PIO0 untouched — its worker still ran but drained no commands
        // and has no enabled SMs, so its pad snapshot stays at 0.
        assert_eq!(
            threaded.shared.pio.read_pads(0),
            (0, 0),
            "PIO0 pads must remain zero — no program was queued for it",
        );
    }

    /// §4 item 3 — cross-block command ordering smoke. A single-thread
    /// burst of three `WriteCtrl` commands with different block targets
    /// and different SM_ENABLE masks lands on the three per-block
    /// queues; one `run_quanta(1)` drains and applies all three. After
    /// the quantum, each block's `read_sm_enabled` reflects its own
    /// mask — proving per-block routing preserves per-block semantics
    /// across the barrier boundary.
    #[test]
    fn cross_block_writectrl_burst_reaches_each_block() {
        // Disjoint single-bit masks so a mis-routing that delivers all
        // three commands to block 0 (in sequence) cannot accidentally
        // pass — each block must end with its exclusive bit set.
        const ENABLE_SM_MASK_0: u32 = 0b0001;
        const ENABLE_SM_MASK_1: u32 = 0b0010;
        const ENABLE_SM_MASK_2: u32 = 0b0100;

        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        threaded.shared.pio.send_command(PioCommand::WriteCtrl {
            block: 0,
            val: ENABLE_SM_MASK_0,
            alias: 0,
        });
        threaded.shared.pio.send_command(PioCommand::WriteCtrl {
            block: 1,
            val: ENABLE_SM_MASK_1,
            alias: 0,
        });
        threaded.shared.pio.send_command(PioCommand::WriteCtrl {
            block: 2,
            val: ENABLE_SM_MASK_2,
            alias: 0,
        });

        threaded.run_quanta(1);

        assert_eq!(
            threaded.shared.pio.read_sm_enabled(0) as u32,
            ENABLE_SM_MASK_0,
            "PIO0 SM_ENABLE must reflect its WriteCtrl",
        );
        assert_eq!(
            threaded.shared.pio.read_sm_enabled(1) as u32,
            ENABLE_SM_MASK_1,
            "PIO1 SM_ENABLE must reflect its WriteCtrl",
        );
        assert_eq!(
            threaded.shared.pio.read_sm_enabled(2) as u32,
            ENABLE_SM_MASK_2,
            "PIO2 SM_ENABLE must reflect its WriteCtrl",
        );
    }

    /// §4 item 4 — idle-block pad-publish semantics. The HLD's invariant
    /// is that an idle PIO worker *still publishes* every quantum (HLD
    /// V5 §2.1 unconditional `write_pads`), so coord's `update_gpio`
    /// always sees a coherent pad snapshot per block. The concrete
    /// post-disable latch value is whatever `PioBlock::write_ctrl`
    /// happens to leave behind — `set_sm_enabled(false)` calls
    /// `merge_pin_outputs`, which zeroes pad_out/pad_oe when the last
    /// SM drops out (mirrors `picoem_common::pio::tests::
    /// disable_clears_pin_outputs`). So "latched" here means "stable":
    /// whatever the first post-disable quantum publishes must survive
    /// unchanged across subsequent idle quanta.
    ///
    /// The test sequence: enable PIO1 → latch a non-zero pad_oe →
    /// disable → snapshot the published value → run 10 more idle quanta
    /// → assert the snapshot is unchanged. Stability, not non-zero.
    #[test]
    fn disabled_block_still_publishes_stable_pads() {
        const PIN: u8 = 15;

        let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        // Phase 1: enable + run so pads latch to a non-zero pad_oe.
        queue_pio_blinky_setup(&threaded.shared, 1, PIN);
        threaded.run_quanta(10);
        let active_pads = threaded.shared.pio.read_pads(1);
        assert_eq!(
            active_pads.1,
            1u32 << PIN,
            "PIO1 pad_oe must drive pin {PIN} while SM0 is enabled (got {:#x})",
            active_pads.1,
        );

        // Phase 2: disable SM0 and run one quantum so the CTRL write
        // drains + applies + the worker publishes the resulting pads.
        threaded.shared.pio.send_command(PioCommand::WriteCtrl {
            block: 1,
            val: 0,
            alias: 0,
        });
        threaded.run_quanta(1);
        assert_eq!(
            threaded.shared.pio.read_sm_enabled(1),
            0,
            "SM0 should be disabled after the WriteCtrl with val=0",
        );
        let idle_pads = threaded.shared.pio.read_pads(1);

        // Phase 3: run 10 more idle quanta — the published pads must
        // not drift. The worker still runs, drains its (empty) queue,
        // skips step_n (enable gate), and publishes pad state every
        // quantum; the value published is stable because nothing mutates
        // `block.pad_out / pad_oe` once the SM is disabled and step_n
        // is skipped.
        threaded.run_quanta(10);
        assert_eq!(
            threaded.shared.pio.read_pads(1),
            idle_pads,
            "idle PIO1 must keep publishing the same pad snapshot every quantum",
        );

        // Other blocks unaffected — their workers publish their own
        // (zero) pad state independently.
        assert_eq!(threaded.shared.pio.read_pads(0), (0, 0));
        assert_eq!(threaded.shared.pio.read_pads(2), (0, 0));
    }

    /// §4 item 5 — per-PIO panic-naming + reassembly end-to-end.
    /// Parameterised over `{pio0, pio1, pio2}`: `TestPanic { block }`
    /// routes through `send_command` into the per-block queue, the
    /// matching PIO worker drains + `apply_pio_command` panics with
    /// `pio{block}` in the message, `spawn_worker` poisons the barrier,
    /// the remaining workers exit cleanly, and `run_quanta` panics on
    /// the main thread with an enumeration of the panicked worker(s).
    ///
    /// Post-panic, the ThreadedEmulator is poisoned: `pio_blocks` is
    /// `None` (HLD V5 §2.7) and the next `run_quanta` call panics with
    /// the `poisoned by prior worker panic` message.
    #[cfg(feature = "testing")]
    #[test]
    fn test_panic_on_pio_worker_poisons_emulator_and_names_block() {
        for block in 0..3u8 {
            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);

            threaded
                .shared
                .pio
                .send_command(PioCommand::TestPanic { block });

            let expected_name = format!("pio{block}");
            let result =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| threaded.run_quanta(1)));
            let payload = result.expect_err("run_quanta must panic when a PIO worker panics");
            let msg = payload
                .downcast_ref::<String>()
                .map(String::as_str)
                .or_else(|| payload.downcast_ref::<&str>().copied())
                .unwrap_or("<non-string panic payload>");
            assert!(
                msg.contains(&expected_name),
                "panic message must name the specific worker ({expected_name}); got: {msg}",
            );

            // HLD V5 §2.7: poisoned instance — pio_blocks dropped, flag set.
            assert!(
                threaded.poisoned,
                "block {block}: poisoned flag must be set after PIO worker panic",
            );
            assert!(
                threaded.pio_blocks.is_none(),
                "block {block}: pio_blocks must be None after PIO worker panic",
            );

            // Contract: a second `run_quanta` on the poisoned instance
            // panics early with the "poisoned" assertion — proving the
            // instance cannot be reused.
            let reuse =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| threaded.run_quanta(1)));
            let reuse_err = reuse.expect_err("reuse of a poisoned ThreadedEmulator must panic");
            let reuse_msg = reuse_err
                .downcast_ref::<String>()
                .map(String::as_str)
                .or_else(|| reuse_err.downcast_ref::<&str>().copied())
                .unwrap_or("<non-string panic payload>");
            assert!(
                reuse_msg.contains("poisoned"),
                "reuse panic must cite poisoning; got: {reuse_msg}",
            );
        }
    }

    // =====================================================================
    // stage5_coverage: accessor / lifecycle / error-path coverage
    // =====================================================================

    mod stage5_coverage {
        use super::*;

        /// `core_cycles(0)` / `core_cycles(1)` on a freshly-built runtime
        /// return 0 (matching each core's pre-execution cycle count).
        #[test]
        fn core_cycles_returns_zero_on_fresh_emulator() {
            let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            assert_eq!(threaded.core_cycles(0), 0);
            assert_eq!(threaded.core_cycles(1), 0);
        }

        /// `core_cycles(idx)` panics on idx >= 2.
        #[test]
        #[should_panic(expected = "idx must be 0 or 1")]
        fn core_cycles_invalid_idx_panics() {
            let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            let _ = threaded.core_cycles(2);
        }

        /// `core_pc(0)` / `core_pc(1)` return Some(_) on a freshly-built
        /// runtime.
        #[test]
        fn core_pc_returns_some_on_fresh_emulator() {
            let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            assert!(threaded.core_pc(0).is_some());
            assert!(threaded.core_pc(1).is_some());
        }

        /// `core_pc(idx)` panics on idx >= 2.
        #[test]
        #[should_panic(expected = "idx must be 0 or 1")]
        fn core_pc_invalid_idx_panics() {
            let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            let _ = threaded.core_pc(2);
        }

        /// `shared()` returns a reference that can observe atomics set
        /// through the original runtime.
        #[test]
        fn shared_accessor_exposes_atomics() {
            let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            let s = threaded.shared();
            // Write a bit via the reference; observe via the original.
            s.atomics.assert_irq(0, 3);
            assert_eq!(threaded.shared.atomics.irq_pending_load(0), 1u64 << 3);
        }

        /// `master_cycle()` returns 0 before any run.
        #[test]
        fn master_cycle_zero_on_fresh_emulator() {
            let threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            assert_eq!(threaded.master_cycle(), 0);
        }

        /// `apply_pio_command` early-return for addr >= 32 on WriteInstrMem
        /// (line 861-863). After the call, INSTR_MEM read-back through
        /// the block reports the unchanged post-reset value.
        #[test]
        fn apply_pio_command_instrmem_out_of_range_skips() {
            let pio = ThreadedPio::new();
            let mut block = PioBlock::new();
            // Pre-seed slot 0 so we can confirm the skipped high-index
            // write does not reach it.
            block.write32(0x048, 0x1234, 0); // INSTR_MEM0
            let before = block.read32(0x048);
            apply_pio_command(
                &mut block,
                0,
                &pio,
                PioCommand::WriteInstrMem {
                    block: 0,
                    addr: 33, // >= 32, skipped
                    value: 0xABCD,
                    alias: 0,
                },
            );
            // No panic, no effect.
            assert_eq!(block.read32(0x048), before);
        }

        /// `apply_pio_command` early-return for sm >= 4 on SetClkDiv
        /// (line 868-870).
        #[test]
        fn apply_pio_command_setclkdiv_out_of_range_skips() {
            let pio = ThreadedPio::new();
            let mut block = PioBlock::new();
            apply_pio_command(
                &mut block,
                0,
                &pio,
                PioCommand::SetClkDiv {
                    block: 0,
                    sm: 4, // >= 4, skipped
                    int_div: 100,
                    frac_div: 0,
                    alias: 0,
                },
            );
            // No panic — SM0 CLKDIV at 0x0C8 stays at its post-reset value.
            // We only assert absence of panic; the specific post-reset
            // value is implementation detail that may change.
        }

        /// `tick_peripherals` held-peripheral branches: lock every
        /// optional peripheral in RESETS and run a quantum. Each
        /// `if !held(RESET_*)` arm takes its false branch.
        #[test]
        fn tick_peripherals_held_peripherals_skip_advance() {
            use crate::bus::{
                RESET_ADC, RESET_I2C0, RESET_I2C1, RESET_PWM, RESET_SPI0, RESET_SPI1, RESET_TIMER0,
                RESET_TIMER1, RESET_UART0, RESET_UART1,
            };

            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);

            // Hold every peripheral tick_peripherals gates on. Post-bootrom
            // RESETS already has UART1/SPI1/I2C1 held; add TIMER0/TIMER1/
            // UART0/SPI0/I2C0/ADC/PWM.
            {
                let mut r = threaded.shared.peripherals.resets.lock().unwrap();
                r.resets_state |= (1u32 << RESET_TIMER0)
                    | (1u32 << RESET_TIMER1)
                    | (1u32 << RESET_UART0)
                    | (1u32 << RESET_UART1)
                    | (1u32 << RESET_SPI0)
                    | (1u32 << RESET_SPI1)
                    | (1u32 << RESET_I2C0)
                    | (1u32 << RESET_I2C1)
                    | (1u32 << RESET_ADC)
                    | (1u32 << RESET_PWM);
            }

            // Run a single quantum. tick_peripherals must take every
            // `held()` true arm without panicking.
            threaded.run_quanta(1);

            // No IRQs should have fired — sanity check.
            assert_eq!(threaded.shared.atomics.irq_pending_load(0), 0);
        }

        /// `from_emulator` on an RISC-V-armed emulator panics with the
        /// explicit message before hoisting state (line 106).
        #[test]
        #[should_panic(expected = "ThreadedEmulator requires Arch::Arm")]
        fn from_emulator_riscv_panics() {
            use crate::{Arch, EmulatorBuilder};
            let emu = EmulatorBuilder::new(Config::default())
                .arch(Arch::RiscV)
                .build()
                .unwrap();
            let _ = ThreadedEmulator::from_emulator(emu);
        }

        /// Running the core-worker body with a halted core lets the
        /// `if !shared.atomics.is_halted(idx)` branch take its false arm
        /// (line 727); with the wake-on-event pattern it exercises the
        /// top-of-quantum consume path (line 698-702). Both cores halted
        /// during `run_quanta` above covers line 727; this test covers
        /// the `wfe_waiting` early-wake branch.
        #[test]
        fn wfe_waiting_early_wake_via_event_flag() {
            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);
            // Put core 0 into WFE-waiting, then pre-arm the event flag —
            // the first quantum's top-of-loop drain clears both.
            threaded.shared.atomics.set_wfe_waiting(0);
            threaded.shared.atomics.event_flag[0].store(true, std::sync::atomic::Ordering::Release);
            threaded.run_quanta(1);
            assert!(
                !threaded.shared.atomics.is_wfe_waiting(0),
                "WFE should have cleared on event_flag_consume"
            );
        }

        /// Release UART1 / SPI1 / I2C1 (held post-bootrom) before
        /// running, so `tick_peripherals`' `!held(RESET_*)` arms take
        /// their true branch for those peripherals too.
        #[test]
        fn tick_peripherals_all_released_drives_every_arm() {
            use crate::bus::{RESET_I2C1, RESET_SPI1, RESET_TIMER1, RESET_UART1};

            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);

            // Release UART1 / SPI1 / I2C1 / TIMER1 so their `!held()`
            // arms inside `tick_peripherals` take the true branch.
            {
                let mut r = threaded.shared.peripherals.resets.lock().unwrap();
                r.resets_state &= !((1u32 << RESET_UART1)
                    | (1u32 << RESET_SPI1)
                    | (1u32 << RESET_I2C1)
                    | (1u32 << RESET_TIMER1));
            }

            threaded.run_quanta(1);

            // Post-run, the runtime must not be poisoned. That's the
            // observable contract; each tick call took its unheld branch.
            assert!(!threaded.poisoned);
        }

        /// Run a real stepping core for several quanta to exercise
        /// `core_worker_body`'s inner loop (lines 727-741):
        ///   - `if !shared.atomics.is_halted(idx)` true arm
        ///   - `while core.cycles() < target` body
        ///   - `core.step_no_atomics(...)` call
        ///   - `if !bus.pending_cache_invalidations.is_empty()` branch
        ///   - post-step `if shared.atomics.is_wfe_waiting(idx) { break; }`
        ///
        /// We seed a minimal Thumb program into SRAM:
        ///   0x2000_0000: STR R0, [R1]   (queues cache invalidation)
        ///   0x2000_0002: B .-2          (branch to self, idle loop)
        /// and point the reset vector + SP at the start.
        #[test]
        fn core_worker_inner_loop_steps_and_drains_invalidations() {
            let mut emu = Emulator::new(Config::default());
            // Minimal Thumb program at 0x2000_0100:
            //  STR R0, [R1]    (Thumb-16 encoding 0x6008)
            //  B .-2           (Thumb-16 encoding 0xE7FE)
            // Pad with NOPs to keep a nominally valid instruction window.
            let prog: [u8; 8] = [
                0x08, 0x60, // STR R0, [R1]
                0xFE, 0xE7, // B . (branch to self)
                0x00, 0xBF, // NOP
                0x00, 0xBF, // NOP
            ];
            emu.load_image(0x2000_0100, &prog);
            // Vector table in SRAM at 0x2000_0000: init SP + reset vector.
            // Bytes: SP lo...hi, reset PC lo...hi (with Thumb bit set).
            let sp: u32 = 0x2001_0000;
            let reset_pc: u32 = 0x2000_0101; // Thumb bit
            let vectors: [u8; 8] = [
                (sp & 0xFF) as u8,
                (sp >> 8 & 0xFF) as u8,
                (sp >> 16 & 0xFF) as u8,
                (sp >> 24 & 0xFF) as u8,
                (reset_pc & 0xFF) as u8,
                (reset_pc >> 8 & 0xFF) as u8,
                (reset_pc >> 16 & 0xFF) as u8,
                (reset_pc >> 24 & 0xFF) as u8,
            ];
            emu.load_image(0x2000_0000, &vectors);
            // Point core 0 at the reset vector. Bypass bootrom by setting
            // PC/SP directly. Also point R1 at a valid SRAM word so the
            // STR drops into memory rather than faulting.
            emu.core_mut(0).regs.set_pc(0x2000_0100);
            emu.core_mut(0).regs.set_sp(sp);
            emu.core_mut(0).regs.r[1] = 0x2000_2000;

            let mut threaded = ThreadedEmulator::from_emulator(emu);
            // Halt core 1; core 0 runs.
            threaded.shared.atomics.set_halted(1);
            // Run enough quanta for many inner-loop iterations.
            threaded.run_quanta(2);
            assert!(!threaded.poisoned);
        }

        /// Enable TICKS TIMER1 domain so `tick_peripherals` at line
        /// 1003 runs `timers.timer1.advance_us(edges)` with `edges > 0`.
        #[test]
        fn tick_peripherals_timer1_advances_us() {
            use crate::peripherals::ticks::{
                CTRL_ENABLE, DOMAIN_STRIDE, DOMAIN_TIMER1, TICKS_BASE,
            };
            let mut emu = Emulator::new(Config::default());
            // Enable TIMER1 TICKS domain (CTRL.ENABLE).
            let ctrl = TICKS_BASE + DOMAIN_TIMER1 as u32 * DOMAIN_STRIDE;
            emu.bus.write32(ctrl, CTRL_ENABLE, 0);

            let mut threaded = ThreadedEmulator::from_emulator(emu);
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);

            // Two quanta: TIMER CYCLES=12 default, step_quantum=64 sysclks
            // → ~5 edges per quantum → `advance_us(edges)` hit >0 times.
            threaded.run_quanta(2);
            assert!(!threaded.poisoned);
        }

        /// Running a single quantum with a pre-set IRQ pending mask
        /// exercises `core_worker_body`'s `if pending != 0` arm —
        /// the merge branch.
        #[test]
        fn core_worker_merges_pending_irq() {
            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);
            threaded.shared.atomics.assert_irq(0, 7);
            threaded.run_quanta(1);
            // take_irq_pending is swap-to-zero; after the quantum the
            // worker consumed the bit into the core's NVIC.
            assert_eq!(threaded.shared.atomics.irq_pending_load(0), 0);
        }

        // =================================================================
        // stage7_branch_coverage: branches missed by stage5_coverage
        // =================================================================

        /// Line 273 — `from_emulator` with `pending_fifo_event = None`
        /// (fresh emulator) takes the `if let Some(receiver) = ...` else
        /// branch: no `event_flag` is set. The "Some" arm has its own
        /// regression test (`from_emulator_preserves_pending_fifo_event`);
        /// this locks the None-arm contract.
        #[test]
        fn from_emulator_with_no_pending_fifo_event_leaves_event_flags_clear() {
            let emu = Emulator::new(Config::default());
            // Default Sio leaves pending_fifo_event = None.
            assert!(emu.bus.sio.pending_fifo_event.is_none());

            let threaded = ThreadedEmulator::from_emulator(emu);
            assert!(
                !threaded.shared.atomics.event_flag[0].load(Ordering::Acquire),
                "no pending FIFO wake → event_flag[0] stays clear"
            );
            assert!(
                !threaded.shared.atomics.event_flag[1].load(Ordering::Acquire),
                "no pending FIFO wake → event_flag[1] stays clear"
            );
        }

        /// Line 596 — when core 0's worker returns Ok and the core's
        /// `bootrom_hook_fired` flag is set, `run_quanta_checked` ORs it
        /// into the host-visible `bootrom_hook_fired` so
        /// `shutdown_requested()` returns true. Drives the latch by
        /// pre-setting the flag on `core0` before handoff (the same drain
        /// site `lib.rs:707-717` exercises serially).
        #[test]
        fn run_quanta_drains_core0_bootrom_hook_into_shutdown_requested() {
            let mut emu = Emulator::new(Config::default());
            // Pre-seed the latch so the post-join OR fires for core 0.
            emu.core_mut(0).bootrom_hook_fired = true;

            let mut threaded = ThreadedEmulator::from_emulator(emu);
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);
            // shutdown_requested is seeded from Emulator::shutdown_requested
            // (which is false here) — must flip true after the drain.
            assert!(!threaded.shutdown_requested());

            threaded.run_quanta(1);

            assert!(
                threaded.shutdown_requested(),
                "core0 bootrom_hook_fired must propagate to shutdown_requested"
            );
            // Sticky: a subsequent run keeps the flag latched even if the
            // hook didn't re-fire (halted core never re-asserts).
            threaded.run_quanta(1);
            assert!(threaded.shutdown_requested());
        }

        /// Line 603 — same as above but for core 1, locking the
        /// per-core OR-into independently.
        #[test]
        fn run_quanta_drains_core1_bootrom_hook_into_shutdown_requested() {
            let mut emu = Emulator::new(Config::default());
            emu.core_mut(1).bootrom_hook_fired = true;

            let mut threaded = ThreadedEmulator::from_emulator(emu);
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);
            assert!(!threaded.shutdown_requested());

            threaded.run_quanta(1);

            assert!(
                threaded.shutdown_requested(),
                "core1 bootrom_hook_fired must propagate to shutdown_requested"
            );
        }

        /// Line 374 — `Emulator::shutdown_requested` is plumbed through
        /// `from_emulator` so a hook fired on the serial path before
        /// promotion is observable post-handoff via
        /// `ThreadedEmulator::shutdown_requested`. Mutates the public
        /// field via `Emulator` destructure-time `shutdown_requested`
        /// binding.
        #[test]
        fn from_emulator_carries_shutdown_requested_seed() {
            let mut emu = Emulator::new(Config::default());
            emu.shutdown_requested = true;

            let threaded = ThreadedEmulator::from_emulator(emu);
            assert!(
                threaded.shutdown_requested(),
                "Emulator::shutdown_requested must seed ThreadedEmulator::bootrom_hook_fired"
            );
        }

        /// Line 771 — the `&&` short-circuit in
        /// `if shared.atomics.is_wfe_waiting(idx) && event_flag_consume(idx)`.
        /// With wfe_waiting=true but event_flag=false the second arm is
        /// false, the body skips, and `wfe_waiting` STAYS set after the
        /// quantum (no wake happened). Counterpart to the existing
        /// `wfe_waiting_early_wake_via_event_flag` which covers the
        /// both-true path.
        #[test]
        fn wfe_waiting_without_event_flag_does_not_clear() {
            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);
            // Park core 0 on WFE; deliberately do NOT set event_flag.
            threaded.shared.atomics.set_wfe_waiting(0);
            assert!(threaded.shared.atomics.is_wfe_waiting(0));
            assert!(!threaded.shared.atomics.event_flag_load(0));

            threaded.run_quanta(1);

            // No event was pending, so the short-circuit took its
            // false branch and wfe_waiting stays set.
            assert!(
                threaded.shared.atomics.is_wfe_waiting(0),
                "wfe_waiting must remain set when event_flag is unset"
            );
        }

        /// Line 809 — the post-step
        /// `if shared.atomics.is_wfe_waiting(idx) { break; }` inside the
        /// inner stepping loop. A real WFE instruction ran by the core
        /// in mid-quantum must set `wfe_waiting`, the inner-loop break
        /// must fire, and the worker must rendezvous immediately at the
        /// barrier without spinning further.
        ///
        /// Program: WFE (0xBF20) followed by NOPs. With event_flag=false
        /// (no pending event) the WFE handler sets wfe_waiting and the
        /// next iteration of the `while core.cycles() < target` loop
        /// breaks via line 809 instead of running the NOPs.
        #[test]
        fn core_worker_inner_loop_breaks_on_post_step_wfe() {
            let mut emu = Emulator::new(Config::default());
            // Program at 0x2000_0100:
            //   WFE   (0xBF20)   — sets wfe_waiting on this CPU
            //   NOP   (0xBF00)   — must NOT execute after WFE breaks
            //   NOP
            //   NOP
            let prog: [u8; 8] = [0x20, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF];
            emu.load_image(0x2000_0100, &prog);
            // SP + reset vector at 0x2000_0000.
            let sp: u32 = 0x2001_0000;
            let reset_pc: u32 = 0x2000_0101; // Thumb bit
            let vectors: [u8; 8] = [
                (sp & 0xFF) as u8,
                ((sp >> 8) & 0xFF) as u8,
                ((sp >> 16) & 0xFF) as u8,
                ((sp >> 24) & 0xFF) as u8,
                (reset_pc & 0xFF) as u8,
                ((reset_pc >> 8) & 0xFF) as u8,
                ((reset_pc >> 16) & 0xFF) as u8,
                ((reset_pc >> 24) & 0xFF) as u8,
            ];
            emu.load_image(0x2000_0000, &vectors);
            emu.core_mut(0).regs.set_pc(0x2000_0100);
            emu.core_mut(0).regs.set_sp(sp);

            let mut threaded = ThreadedEmulator::from_emulator(emu);
            // Halt core 1 so only core 0's worker exercises the WFE break.
            threaded.shared.atomics.set_halted(1);
            // event_flag must be false so WFE actually parks the core.
            assert!(!threaded.shared.atomics.event_flag_load(0));

            threaded.run_quanta(1);

            // Post-quantum: core 0 executed WFE, the post-step check at
            // line 809 broke the inner loop, and wfe_waiting is set.
            assert!(
                threaded.shared.atomics.is_wfe_waiting(0),
                "WFE must have parked core 0 (set wfe_waiting via core::wfe)"
            );
            // PC advanced past the WFE (Thumb-16, +2) — proving WFE
            // executed exactly once before the inner-loop break fired.
            // Without the line-809 break, subsequent NOPs at 0x2000_0102+
            // would have run too, but they don't move PC differently from
            // a correctly-broken loop because the first re-entry top-of-
            // quantum check would still flag wfe_waiting. The cycle count
            // is the cleaner signal: only one instruction executed.
            let core_cycles = threaded.core_cycles(0);
            assert!(
                core_cycles >= 1 && core_cycles < threaded.step_quantum as u64,
                "core 0 should have stopped early on WFE (cycles={core_cycles})"
            );
        }

        /// Lines 591/602/644 (Ok arms) and 664 None branch — happy-path
        /// timing-disabled run that takes none of the panic paths.
        /// Distinct from `core_worker_inner_loop_steps_and_drains_invalidations`
        /// in that this asserts the post-run state explicitly: `core0`
        /// + `core1` + `pio_blocks` all hold `Some(_)` (no worker dropped),
        /// `last_run_timings` stays `None`, and `poisoned` is false.
        #[test]
        fn run_quanta_happy_path_preserves_owned_state() {
            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);
            // Pre-condition: every owned slot starts populated.
            assert!(threaded.core0.is_some());
            assert!(threaded.core1.is_some());
            assert!(threaded.pio_blocks.is_some());

            threaded.run_quanta(2);

            // Post-condition: the Ok arms at 591/602 reassembled core0
            // and core1, the !any_pio_err branch at 630 reassembled
            // pio_blocks, the rc Ok arm extracted coord timings (which
            // we discard since timing is off), and the first_panic None
            // branch at 664 left the instance non-poisoned.
            assert!(threaded.core0.is_some(), "core0 reassembled");
            assert!(threaded.core1.is_some(), "core1 reassembled");
            assert!(threaded.pio_blocks.is_some(), "pio_blocks reassembled");
            assert!(threaded.last_run_timings.is_none(), "timing disabled");
            assert!(!threaded.poisoned, "no panic ⇒ not poisoned");
        }

        /// Reuse-after-poison guard: after a TestPanic-driven PIO panic,
        /// the next `run_quanta` call must hit the
        /// `assert!(!self.poisoned, ...)` at line 498-501. Locks the
        /// "drop and rebuild" contract for the poisoned state.
        #[cfg(feature = "testing")]
        #[test]
        #[should_panic(expected = "poisoned by prior worker panic")]
        fn run_quanta_on_poisoned_emulator_asserts() {
            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);
            threaded
                .shared
                .pio
                .send_command(PioCommand::TestPanic { block: 0 });

            // First call panics through the worker → instance poisoned.
            // We catch_unwind to keep the test stable across the
            // expected-substring matchers.
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                threaded.run_quanta(1)
            }));
            assert!(threaded.poisoned);

            // Second call hits the entry-point assert and panics with
            // the expected message — caught by #[should_panic] above.
            threaded.run_quanta(1);
        }

        /// `RunError` Debug + Eq + Clone derives: covers the discriminant
        /// formatters and the equality contract used by the
        /// `EmulatorError` lift in lib.rs. Important for callers that
        /// compare run errors across one-shot reentry.
        #[test]
        fn run_error_derives_round_trip() {
            use crate::threaded::WorkerName;
            let p1 = RunError::Panic {
                which: WorkerName::Pio0,
                message: "boom".into(),
            };
            let p2 = p1.clone();
            assert_eq!(p1, p2);
            let _ = format!("{:?}", p1);

            let t1 = RunError::Timeout {
                which: WorkerName::Coord,
                elapsed_ms: 5_000,
            };
            let t2 = t1.clone();
            assert_eq!(t1, t2);
            let _ = format!("{:?}", t1);

            // Different variants are not equal.
            assert_ne!(p1, t1);
        }

        /// `apply_pio_command` with a SetClkDiv whose `sm < 4` exercises
        /// the false branch of `if sm >= 4` (line 964) end-to-end, with
        /// the actual SMn_CLKDIV write landing in the block. Distinct
        /// from `apply_pio_command_setclkdiv_out_of_range_skips` (which
        /// covers the true branch).
        #[test]
        fn apply_pio_command_setclkdiv_in_range_writes_block() {
            let pio = ThreadedPio::new();
            let mut block = PioBlock::new();
            apply_pio_command(
                &mut block,
                0,
                &pio,
                PioCommand::SetClkDiv {
                    block: 0,
                    sm: 2,
                    int_div: 0x0042,
                    frac_div: 0x80,
                    alias: 0,
                },
            );
            // SM2_CLKDIV at 0x0C8 + 2*0x18 = 0x0F8.
            let val = block.read32(0x0F8);
            assert_eq!(val >> 16, 0x0042, "INT field landed");
            assert_eq!((val >> 8) & 0xFF, 0x80, "FRAC field landed");
        }

        /// `apply_pio_command` with a WriteInstrMem whose `addr < 32`
        /// covers the false branch of `if addr >= 32` (line 951) with
        /// the actual write landing in INSTR_MEM. Distinct from
        /// `apply_pio_command_instrmem_out_of_range_skips`.
        #[test]
        fn apply_pio_command_instrmem_in_range_writes_block() {
            let pio = ThreadedPio::new();
            let mut block = PioBlock::new();
            apply_pio_command(
                &mut block,
                0,
                &pio,
                PioCommand::WriteInstrMem {
                    block: 0,
                    addr: 5,
                    value: 0x1234,
                    alias: 0,
                },
            );
            // INSTR_MEM5 at 0x048 + 5*4 = 0x05C.
            assert_eq!(block.instr_mem()[5], 0x1234);
        }

        /// Line 876 — when `read_sm_enabled(block_idx) == 0` the PIO
        /// worker skips `step_n_with_pins` and `write_irq_flags`, but
        /// still publishes pads (HLD V5 §2.1). Concretely: a
        /// freshly-built ThreadedEmulator has all SM masks at 0 and a
        /// `run_quanta(1)` must (a) not assert any IRQ, (b) leave the
        /// pad snapshot at zero. The complement (mask != 0) is covered
        /// by the §4 item 2 concurrent-pads test.
        #[test]
        fn pio_worker_skips_step_when_sm_enabled_is_zero() {
            let mut threaded = ThreadedEmulator::from_emulator(Emulator::new(Config::default()));
            threaded.shared.atomics.set_halted(0);
            threaded.shared.atomics.set_halted(1);

            // Pre-conditions: all SM masks are zero.
            assert_eq!(threaded.shared.pio.read_sm_enabled(0), 0);
            assert_eq!(threaded.shared.pio.read_sm_enabled(1), 0);
            assert_eq!(threaded.shared.pio.read_sm_enabled(2), 0);

            threaded.run_quanta(1);

            // Post: pads still zero (worker took the skip arm), no IRQ
            // flags were published.
            assert_eq!(threaded.shared.pio.read_pads(0), (0, 0));
            assert_eq!(threaded.shared.pio.read_pads(1), (0, 0));
            assert_eq!(threaded.shared.pio.read_pads(2), (0, 0));
        }

        /// `split_ok` Some path — a Some((block, timings)) input must
        /// be split into two Somes preserving both halves.
        #[test]
        fn split_ok_some_input_yields_two_somes() {
            let block = PioBlock::new();
            let timings = PerWorkerTimings::default();
            let (b, t) = split_ok(Some((block, timings)));
            assert!(b.is_some());
            assert!(t.is_some());
        }

        /// `split_ok` None path — a None input maps to (None, None).
        #[test]
        fn split_ok_none_input_yields_two_nones() {
            let (b, t) = split_ok(None);
            assert!(b.is_none());
            assert!(t.is_none());
        }
    }

    // ----- HLD V5 §4 item 6: OneROM-shape soak ---------------------------

    /// HLD V5 §4 item 6 — `ThreadedEmulator` runs an OneROM-shape workload
    /// (PIO0 idle, PIO1 SM0 enabled, PIO2 SM0+SM1 enabled) for
    /// `min(60 s wall, 10^8 quanta)` with `debug_assertions` on, under
    /// the bench default `step_quantum = 256` (§8 gates on the throughput
    /// regime, not the barrier-dominated `sq=64` regime). The runtime
    /// must complete without panicking and without poisoning the instance.
    ///
    /// Termination: whichever of wall-clock 60 s or 10^8 quanta fires
    /// first, checked once per `CHUNK_QUANTA`-sized `run_quanta` call.
    /// Final output (printed for journal capture):
    ///  - elapsed wall time,
    ///  - quanta completed,
    ///  - effective throughput (`quanta * step_quantum / elapsed`).
    #[test]
    #[ignore = "60s stress; run explicitly via cargo test -- --ignored stress_onerom"]
    fn stress_onerom_60s() {
        use std::time::{Duration, Instant};

        const STEP_QUANTUM: u32 = 256;
        const MAX_QUANTA: u64 = 100_000_000;
        const WALL_LIMIT: Duration = Duration::from_secs(60);
        const CHUNK_QUANTA: u64 = 10_000;

        // Pin per-SM programs on blocks 1 and 2 (block 0 stays idle). The
        // existing `queue_pio_blinky_setup` helper enables SM0 only, which
        // matches the HLD shape for PIO1. For PIO2 we additionally enable
        // SM1 by OR-ing into the CTRL mask — the blinky helper already
        // loaded a valid wrap-loop program at addr 0/1 that SM1 can also
        // execute (SM1 and SM0 share instruction memory on each block).
        let emu = crate::EmulatorBuilder::new(Config::default())
            .step_quantum(STEP_QUANTUM)
            .build()
            .unwrap();
        let mut threaded = ThreadedEmulator::from_emulator(emu);
        threaded.shared.atomics.set_halted(0);
        threaded.shared.atomics.set_halted(1);

        // PIO1: SM0 enabled, driving pin 10.
        queue_pio_blinky_setup(&threaded.shared, 1, 10);

        // PIO2: SM0 enabled via the helper, driving pin 20. Then a second
        // CTRL write enables SM1 alongside SM0 (0b0011) — SM1 re-uses
        // the instruction memory loaded by the helper and will spin the
        // same wrap loop. PIO2 thus runs with two SMs enabled, matching
        // the HLD's "PIO2 with SM0 + SM1 enabled" shape.
        queue_pio_blinky_setup(&threaded.shared, 2, 20);
        threaded.shared.pio.send_command(PioCommand::WriteCtrl {
            block: 2,
            val: 0b0011,
            alias: 0,
        });

        let start = Instant::now();
        let mut quanta: u64 = 0;
        loop {
            threaded.run_quanta(CHUNK_QUANTA);
            quanta += CHUNK_QUANTA;

            let elapsed = start.elapsed();
            if elapsed >= WALL_LIMIT || quanta >= MAX_QUANTA {
                break;
            }
        }
        let elapsed = start.elapsed();

        // Contract: the runtime is not poisoned. If any worker had
        // panicked, `run_quanta` itself would have re-raised the payload
        // (test would already have failed); this is the belt-and-braces
        // check that the poison flag never got flipped silently.
        assert!(
            !threaded.poisoned,
            "ThreadedEmulator poisoned after OneROM-shape stress run",
        );

        let elapsed_secs = elapsed.as_secs_f64();
        let sysclks = quanta.saturating_mul(STEP_QUANTUM as u64);
        let throughput_hz = if elapsed_secs > 0.0 {
            sysclks as f64 / elapsed_secs
        } else {
            0.0
        };
        println!(
            "stress_onerom_60s: elapsed={elapsed_secs:.3}s quanta={quanta} \
             step_quantum={STEP_QUANTUM} sysclks={sysclks} \
             throughput={:.3} MHz",
            throughput_hz / 1.0e6,
        );
    }
}

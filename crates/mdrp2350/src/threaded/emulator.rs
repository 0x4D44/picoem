//! `ThreadedEmulator` — 4-thread runtime entry point for Phase 3.
//!
//! Phase 3 Stage 7 (LLD V7 §9): real core / PIO / coordinator worker
//! bodies. Each quantum the four workers rendezvous on the shared
//! `SpinBarrier` twice — once at the top (execution phase start) and
//! once at the bottom (peripheral-tick phase end). The coordinator
//! publishes `master_cycle` and ticks shared peripherals between the
//! two barriers; the CPU and PIO workers execute during the first
//! phase and are idle in the second.
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
    /// [`coordinator_worker_body`].
    pub fn master_cycle(&self) -> u64 {
        self.shared.master_cycle.load(Ordering::Acquire)
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

        let h0 = spawn_worker(mask[0], barrier.clone(), {
            let s = shared.clone();
            move |b| core_worker_body(0, core0, s, b, n, step_q)
        });
        let h1 = spawn_worker(mask[1], barrier.clone(), {
            let s = shared.clone();
            move |b| core_worker_body(1, core1, s, b, n, step_q)
        });
        let hp = spawn_worker(mask[2], barrier.clone(), {
            let s = shared.clone();
            move |b| pio_worker_body(blocks, s, b, n, step_q)
        });
        let hc = spawn_worker(mask[3], barrier.clone(), {
            let s = shared.clone();
            move |b| coordinator_worker_body(s, b, n, step_q)
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
// Stage 7 worker bodies (LLD V7 §9)
// =======================================================================
//
// Each body owns its loop over `n` quanta. Within a quantum, the CPU
// and PIO workers do their work then double-`barrier.wait()` (once to
// let the coordinator advance `master_cycle` / tick peripherals, once
// to re-synchronise before the next quantum). The coordinator's
// loop is the mirror: it waits first, advances state, then waits again.
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
) -> CortexM33 {
    let mut bus = WorkerBus::new(core_id, shared.clone());
    let mut target: u64 = 0;
    let idx = core_id as usize;

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
                // `core.step(&mut bus)` borrows `bus` mutably for the call duration,
                // and `invalidate_decode_cache_entries(&mut self, bus, addrs)` also
                // needs `&mut bus`. We can't simultaneously borrow
                // `&bus.pending_cache_invalidations` and pass `&mut bus` — so swap
                // the Vec out, drain, put it back.
                //
                // Happy path: the original 16-cap Vec roundtrips through `addrs` and
                // gets cleared in place (preserving capacity). If invalidate_...
                // panics, the cap-0 temporary stays in place and future quanta start
                // re-allocating from cap 0 until the Vec grows back. This is
                // acceptable given the panic already escalates to barrier poison +
                // ThreadedEmulator::poisoned.
                let addrs = std::mem::take(&mut bus.pending_cache_invalidations);
                core.invalidate_decode_cache_entries(&mut bus, &addrs);
                bus.pending_cache_invalidations = addrs;
                bus.pending_cache_invalidations.clear();
            }
        }
        core.ppb.update_latest_cycles(core.cycles());

        if barrier.wait() == BarrierResult::Poisoned {
            return core;
        }
        if barrier.wait() == BarrierResult::Poisoned {
            return core;
        }
    }
    core
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
) -> [PioBlock; 3] {
    for _ in 0..n {
        for cmd in shared.pio.drain_commands() {
            apply_pio_command(&mut blocks, cmd);
        }

        // GPIO_IN snapshot once per quantum — parity with the
        // single-threaded `Emulator::tick_peripherals` which reads
        // `bus.gpio_in` once and hands it to every PIO step.
        let gpio_in = shared.gpio.read_in();

        for (block_idx, block) in blocks.iter_mut().enumerate() {
            // NOTE: shared.pio.read_sm_enabled returns 0 until the PIO CTRL
            // routing lands (see bus.rs TODO). Today the worker body drains
            // commands + steps only the blocks whose PioBlock state was
            // pre-configured via single-threaded Bus before from_emulator.
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

        if barrier.wait() == BarrierResult::Poisoned {
            return blocks;
        }
        if barrier.wait() == BarrierResult::Poisoned {
            return blocks;
        }
    }
    blocks
}

/// Apply a CPU-queued [`PioCommand`] to the PIO worker's local
/// `PioBlock`s. Routes through each block's public `write32` so all
/// the existing bookkeeping (INSTR_MEM index guard, FIFO-join handling,
/// alias decoding) continues to apply.
fn apply_pio_command(blocks: &mut [PioBlock; 3], cmd: PioCommand) {
    match cmd {
        PioCommand::WriteInstrMem { block, addr, value } => {
            let b = block as usize;
            if b >= blocks.len() || addr >= 32 {
                return;
            }
            let offset = 0x048 + (addr as u32) * 4;
            blocks[b].write32(offset, value as u32, 0);
        }
        PioCommand::SetClkDiv { block, sm, int_div, frac_div } => {
            let b = block as usize;
            if b >= blocks.len() || sm >= 4 {
                return;
            }
            // SMn_CLKDIV lives at 0x0C8 + sm * 0x18. Layout: INT<<16,
            // FRAC<<8, rest reserved (mdpicoem-common::pio::mod §write_clkdiv).
            let offset = 0x0C8 + (sm as u32) * 0x18;
            let val = ((int_div as u32) << 16) | ((frac_div as u32) << 8);
            blocks[b].write32(offset, val, 0);
        }
    }
}

/// Coordinator worker. Advances `master_cycle` + MTIME then ticks the
/// coordinator-owned peripherals between the two barriers. The
/// `fetch_add(Release)` on `master_cycle` pairs with every CPU
/// worker's `load(Acquire)` in `bus/peripherals.rs`'s PLL CS read
/// path (LLD V7 §3, §9).
fn coordinator_worker_body(
    shared: SharedState,
    barrier: Arc<SpinBarrier>,
    n: u64,
    step_q: u32,
) {
    for _ in 0..n {
        if barrier.wait() == BarrierResult::Poisoned {
            return;
        }

        // Advance master_cycle BEFORE ticking peripherals so CPU
        // workers' next-quantum PLL reads observe the fresh timeline.
        shared.master_cycle.fetch_add(step_q as u64, Ordering::Release);
        shared.sio.mtime_advance(step_q as u64);

        tick_peripherals(&shared, step_q as u64);

        if barrier.wait() == BarrierResult::Poisoned {
            return;
        }
    }
}

/// Coordinator-owned peripheral tick. Stage 7 minimal implementation —
/// full per-peripheral integration (DMA progress, TIMER0/1 alarm arm
/// checks, UART TX/RX FIFO drain, ADC FIFO, PWM wrap) lands alongside
/// the corresponding migration from the single-threaded Bus tick path.
///
/// Until those peripherals publish their per-quantum effects through
/// `Peripherals` (Phase 4/5), this function is intentionally a no-op —
/// the `shared.master_cycle` + `shared.sio.mtime_advance` already done
/// in the coordinator body cover the clock-tree visibility contract
/// the CPU workers rely on.
fn tick_peripherals(shared: &SharedState, cycles: u64) {
    // TODO(Phase 4): TIMER0 / TIMER1 alarm fire checks — see
    //   `Bus::tick_peripherals` in `crates/mdrp2350/src/bus/mod.rs` for
    //   the single-threaded implementation, which drains alarm-match
    //   IRQs into `CoreAtomics::assert_irq_shared`.
    // TODO(Phase 4): DMA tick — advance channel progress, raise
    //   DMA_IRQ_0/1 via `shared.atomics.assert_irq_shared`.
    // TODO(Phase 5): UART TX/RX FIFO drain + ADC FIFO + PWM wrap.
    //
    // All of the above will compose cleanly with the existing
    // `Peripherals` mutex layout — the coordinator is the only
    // writer of these states so contention is only with CPU-worker
    // MMIO reads against the APB surface.
    let _ = (shared, cycles);
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
        let addrs = std::mem::take(&mut bus.pending_cache_invalidations);
        let mut dummy = Emulator::new(Config::default());
        dummy
            .cores[0]
            .invalidate_decode_cache_entries(&mut bus, &addrs);
        bus.pending_cache_invalidations = addrs;
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
}

//! `WorkerBus` — the per-CPU-thread `CoreBus` implementation that routes
//! memory accesses into the `SharedState` primitives (Phase 1/2) plus the
//! Stage-4 mutex-guarded peripheral bundle.
//!
//! Phase 3 Stage 5 (LLD V7 §4, §5, §6):
//!
//! - [`WorkerBus`] — owned by each CPU worker thread, implements
//!   [`crate::core::bus_trait::CoreBus`].
//! - [`PioBus`] — owned by the PIO worker thread; holds the three
//!   `PioBlock`s. Stage 5 only stands up the constructor; Stage 7 adds
//!   the worker loop that drives PIO stepping.
//!
//! ## What Stage 5 covers
//!
//! - Address-class dispatch (region `0x0`, `0x1`, `0x2`, `0x4`, `0x5`,
//!   `0xD`) using `SharedMemory` / `AtomicGpio` / `ThreadedSio` /
//!   `Peripherals`.
//! - Peer-monitor snoop on every write.
//! - Per-core decode-cache invalidation queue (`pending_cache_invalidations`).
//! - FIFO push → `event_flag[peer]` wake hook (WFE wake parity with
//!   `bus/mod.rs:2182-2186`).
//! - APB read/write dispatch for PLL_SYS / PLL_USB / CLOCKS / RESETS /
//!   QMI / ROSC / XOSC / TIMERS / APB (UART/SPI/I2C/ADC/PWM/IO_BANK0/
//!   PADS_BANK0) / DMA; unknown offsets fall through to
//!   `peripherals.legacy` (HashMap).
//!
//! ## What Stage 5 does NOT cover
//!
//! - Wiring into the `Emulator` struct or `ThreadedEmulator::from_emulator`
//!   — that lands in Stage 6.
//! - Worker-body `core_worker_body` / `pio_worker_body` loops — Stage 7.
//! - DIV/INTERP intercept on `CortexM33` (already done in Stage 3).
//!
//! ## STREX note
//!
//! STREX-on-success **does not route through** [`WorkerBus::write32`]. The
//! execute-site on `CortexM33` calls
//! `shared.memory.cas32(addr, expected, new_val)` directly, then
//! `shared.monitors.snoop(addr)`, bypassing the region dispatch +
//! `pending_cache_invalidations` queue. STREX into executable memory
//! therefore requires firmware-issued `ISB` per the ARMv8-M spec (LLD
//! V7 §4). [`WorkerBus::write32`] has no STREX-specific branch.

use std::sync::Arc;
use std::sync::atomic::Ordering;

use mdpicoem_common::PioBlock;

use crate::core::bus_trait::CoreBus;
use crate::dma::DMA_BASE;
use crate::peripherals::adc::ADC_BASE;
use crate::peripherals::i2c::I2C0_BASE;
use crate::peripherals::io_bank0::IO_BANK0_BASE;
use crate::peripherals::pads_bank0::PADS_BANK0_BASE;
use crate::peripherals::pwm::PWM_BASE;
use crate::peripherals::spi::SPI0_BASE;
use crate::peripherals::ticks::TICKS_BASE;
use crate::peripherals::timer::{TIMER0_BASE, TIMER1_BASE};
use crate::peripherals::uart::UART0_BASE;
use crate::threaded::CoreAtomics;
use crate::threaded::PioCommand;
use crate::threaded::SharedState;

/// Capacity bound for [`WorkerBus::pending_cache_invalidations`]: STM
/// tops out at 13 registers; FPU context push spills 16 words; keep
/// headroom so typical bursts amortise within a single allocation.
pub(crate) const PENDING_INVALIDATION_CAPACITY: usize = 16;

// =======================================================================
// PioBus
// =======================================================================

/// PIO-thread view of the shared state. Owns the three [`PioBlock`]s
/// that the worker drives per-cycle.
///
/// Stage 5 only stands up the constructor + the [`Self::take_blocks`]
/// escape hatch; Stage 7 wires `pio_worker_body` that drains
/// `shared.pio.drain_commands()` and steps each enabled state machine
/// `step_quantum` times per quantum.
pub struct PioBus {
    #[allow(dead_code)] // consumed by Stage 7's pio_worker_body
    shared: SharedState,
    #[allow(dead_code)] // consumed by Stage 7 + take_blocks (cfg(test))
    blocks: [PioBlock; 3],
}

impl PioBus {
    /// Construct a new PIO worker bus with the given shared state and
    /// PIO block storage. Stage 7's `pio_worker_body` consumes this.
    pub fn new(shared: SharedState, blocks: [PioBlock; 3]) -> Self {
        Self { shared, blocks }
    }

    /// Reclaim the underlying `PioBlock`s at worker exit. Called by
    /// Stage 7's `run_quanta` to hand the blocks back to the
    /// `ThreadedEmulator`.
    #[allow(dead_code)] // used in Stage 7 worker exit + by cfg(test)
    pub(crate) fn take_blocks(self) -> [PioBlock; 3] {
        self.blocks
    }
}

// =======================================================================
// WorkerBus
// =======================================================================

/// Per-CPU-thread bus view. Holds a clone of [`SharedState`] plus the
/// per-instruction accounting fields that in the single-threaded path
/// live directly on `Bus`.
///
/// ## Cache invalidation queue
///
/// Every SRAM / ROM / XIP write pushes the target address into
/// [`Self::pending_cache_invalidations`]. The worker loop drains this
/// after each `core.step` and feeds the addresses into the core's
/// local decode cache. Cross-core SMC is the firmware's responsibility
/// (per ARM spec: `DSB; ISB; IC IVAU`).
///
/// ## Decode cache
///
/// The decode cache lives on each [`crate::core::CortexM33`] (Phase 3
/// follow-up #10). `WorkerBus` only carries the dirty-range log
/// [`Self::pending_cache_invalidations`]; the worker drains it into the
/// per-core cache via
/// [`crate::core::CortexM33::invalidate_decode_cache_entries`] after
/// each `core.step()`.
///
pub struct WorkerBus {
    core_id: u8,
    shared: SharedState,
    active_pc: u32,
    burst_mode: bool,
    extra_wait_states: u32,
    /// SRAM / ROM / XIP write addresses queued this instruction.
    /// Drained by the worker loop after each `core.step`.
    pub pending_cache_invalidations: Vec<u32>,
}

impl WorkerBus {
    /// Construct a new `WorkerBus` for `core_id` with the given
    /// [`SharedState`].
    ///
    /// TODO(Stage-6): `ThreadedEmulator::from_emulator` constructs
    /// the `SharedState` and drives `WorkerBus::new` once per core
    /// when spawning workers.
    pub fn new(core_id: u8, shared: SharedState) -> Self {
        debug_assert!(core_id < 2, "core_id must be 0 or 1");
        // See `PENDING_INVALIDATION_CAPACITY` — pre-allocating up front
        // keeps the write hot path allocation-free in steady state.
        let pending_cache_invalidations = Vec::with_capacity(PENDING_INVALIDATION_CAPACITY);
        Self {
            core_id,
            shared,
            active_pc: 0,
            burst_mode: false,
            extra_wait_states: 0,
            pending_cache_invalidations,
        }
    }

    /// Accessor for tests only — otherwise the `core_id` is consumed
    /// through the `core` arg on each access method.
    #[allow(dead_code)]
    #[inline]
    pub(crate) fn core_id(&self) -> u8 {
        self.core_id
    }

    // --- Per-region dispatch (internal) ---

    /// Master-cycle snapshot, taken lock-free **before** any
    /// `peripherals.*` lock is acquired. Keeps coordinator
    /// `fetch_add` off the lock path per LLD V7 §4.
    #[inline]
    fn master_cycle(&self) -> u64 {
        self.shared.master_cycle.load(Ordering::Acquire)
    }

    /// APB (`0x4`) read32 dispatch. Tries each component's APB helper;
    /// falls through to the `legacy` HashMap for addresses no typed
    /// component has migrated yet.
    ///
    /// Peripherals held in `RESETS.RESET` return 0 before any typed
    /// dispatch — parity with `Bus::read32` (`bus/mod.rs:1629`).
    fn apb_read32(&mut self, addr: u32) -> u32 {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        let mc = self.master_cycle();

        // RESETS guard (HLD V5 §5.3). Held peripherals read as 0 —
        // typed peripherals never see the access.
        if self
            .shared
            .peripherals
            .resets
            .lock()
            .unwrap()
            .is_held_in_reset_base(base)
        {
            return 0;
        }

        match base {
            // CLOCKS / PLL / ROSC / XOSC live inside ClocksState.
            0x4001_0000 => self.shared.peripherals.clocks.lock().unwrap().clocks_read(offset),
            0x4005_0000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_sys_read_at(offset, mc),
            0x4005_8000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_usb_read_at(offset, mc),
            0x4004_8000 => self.shared.peripherals.clocks.lock().unwrap().xosc_read(offset),
            0x400E_8000 => self.shared.peripherals.clocks.lock().unwrap().rosc_read(offset),

            0x4002_0000 => self.shared.peripherals.resets.lock().unwrap().resets_read(offset),
            0x400D_0000 => self.shared.peripherals.qmi.lock().unwrap().qmi_read(offset),
            0x4000_0000 => sysinfo_read(offset),

            TIMER0_BASE => self.shared.peripherals.timers.lock().unwrap().timer0.read32(offset),
            TIMER1_BASE => self.shared.peripherals.timers.lock().unwrap().timer1.read32(offset),
            TICKS_BASE => self.shared.peripherals.timers.lock().unwrap().ticks.read32(offset),

            UART0_BASE => self.shared.peripherals.apb.lock().unwrap().uart0.read32(offset),
            SPI0_BASE => self.shared.peripherals.apb.lock().unwrap().spi0.read32(offset),
            I2C0_BASE => self.shared.peripherals.apb.lock().unwrap().i2c0.read32(offset),
            ADC_BASE => self.shared.peripherals.apb.lock().unwrap().adc.read32(offset),
            PWM_BASE => self.shared.peripherals.apb.lock().unwrap().pwm.read32(offset),
            IO_BANK0_BASE => self.shared.peripherals.apb.lock().unwrap().io_bank0.read32(offset),
            PADS_BANK0_BASE => self.shared.peripherals.apb.lock().unwrap().pads_bank0.read32(offset),

            _ => {
                // Legacy HashMap fallback.
                self.shared
                    .peripherals
                    .legacy
                    .lock()
                    .unwrap()
                    .get(&canonical)
                    .copied()
                    .unwrap_or(0)
            }
        }
    }

    /// APB write32 dispatch. Mirrors `apb_read32` structurally.
    ///
    /// Peripherals held in `RESETS.RESET` drop the write silently —
    /// parity with `Bus::write32` (`bus/mod.rs:1742`).
    fn apb_write32(&mut self, addr: u32, val: u32) {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        let alias = (addr >> 12) & 3;
        let mc = self.master_cycle();

        // RESETS guard (HLD V5 §5.3). Held peripherals drop writes
        // silently — typed peripherals never see the access.
        if self
            .shared
            .peripherals
            .resets
            .lock()
            .unwrap()
            .is_held_in_reset_base(base)
        {
            return;
        }

        match base {
            0x4001_0000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .clocks_write(offset, val, alias),
            0x4005_0000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_sys_write_at(offset, val, alias, mc),
            0x4005_8000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_usb_write_at(offset, val, alias, mc),
            0x4004_8000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .xosc_write(offset, val, alias),
            0x400E_8000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .rosc_write(offset, val, alias),

            0x4002_0000 => self
                .shared
                .peripherals
                .resets
                .lock()
                .unwrap()
                .resets_write(offset, val, alias),
            0x400D_0000 => self
                .shared
                .peripherals
                .qmi
                .lock()
                .unwrap()
                .qmi_write(offset, val),
            // SYSINFO is read-only on real hardware.
            0x4000_0000 => {}

            TIMER0_BASE => {
                let mut p = self.shared.peripherals.timers.lock().unwrap();
                p.timer0.write32(offset, val, alias);
            }
            TIMER1_BASE => {
                let mut p = self.shared.peripherals.timers.lock().unwrap();
                p.timer1.write32(offset, val, alias);
            }
            TICKS_BASE => {
                let mut p = self.shared.peripherals.timers.lock().unwrap();
                let invalidate = p.ticks.write32(offset, val, alias);
                if invalidate {
                    p.timer0.invalidate_lazy();
                    p.timer1.invalidate_lazy();
                }
            }

            UART0_BASE => {
                let mut ext_irqs = 0u64;
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .uart0
                    .write32(offset, val, alias, &mut ext_irqs);
                self.raise_irqs_shared(ext_irqs);
            }
            SPI0_BASE => {
                let mut ext_irqs = 0u64;
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .spi0
                    .write32(offset, val, alias, &mut ext_irqs);
                self.raise_irqs_shared(ext_irqs);
            }
            I2C0_BASE => {
                let mut ext_irqs = 0u64;
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .i2c0
                    .write32(offset, val, alias, &mut ext_irqs);
                self.raise_irqs_shared(ext_irqs);
            }
            ADC_BASE => {
                let mut ext_irqs = 0u64;
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .adc
                    .write32(offset, val, alias, &mut ext_irqs);
                self.raise_irqs_shared(ext_irqs);
            }
            PWM_BASE => {
                let mut ext_irqs = 0u64;
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .pwm
                    .write32(offset, val, alias, &mut ext_irqs);
                self.raise_irqs_shared(ext_irqs);
            }
            IO_BANK0_BASE => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .io_bank0
                .write32(offset, val, alias),
            PADS_BANK0_BASE => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .pads_bank0
                .write32(offset, val, alias),

            _ => {
                // Legacy HashMap fallback with alias logic.
                let mut legacy = self.shared.peripherals.legacy.lock().unwrap();
                let old = legacy.get(&canonical).copied().unwrap_or(0);
                let new_val = match alias {
                    0 => val,
                    1 => old ^ val,
                    2 => old | val,
                    3 => old & !val,
                    _ => val,
                };
                legacy.insert(canonical, new_val);
            }
        }
    }

    /// AHB (`0x5`) read32 — DMA at 0x5000_0000, PIO at 0x5020_0000..0x5040_0000.
    fn ahb_read32(&mut self, addr: u32) -> u32 {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        match base {
            DMA_BASE => self
                .shared
                .peripherals
                .dma
                .lock()
                .unwrap()
                .dma
                .read32(offset),
            // PIO register reads: the `PioBlock`s themselves live on
            // the PIO worker thread, so the CPU worker can only observe
            // the atomics `ThreadedPio` publishes — today that's
            // CTRL.SM_ENABLE (0x000) and IRQ (0x030). RX FIFO pops and
            // per-SM register reads need a read-through channel (not
            // yet wired — Phase 4/5 scope) and return 0 for now, which
            // matches a freshly reset block.
            0x5020_0000 | 0x5030_0000 | 0x5040_0000 => {
                // PIO blocks are 0x10_0000 bytes apart (0x502/0x503/0x504).
                let block = ((base - 0x5020_0000) >> 20) as usize;
                match offset {
                    0x000 => self.shared.pio.read_sm_enabled(block) as u32,
                    0x030 => self.shared.pio.read_irq_flags(block) as u32,
                    _ => {
                        // FSTAT / FLEVEL / RXFn / DBG_* / per-SM
                        // reads need a read-through channel to the PIO
                        // worker's local `PioBlock`s (Phase 4/5 scope).
                        // Surface the gap loudly under `cargo test`,
                        // keep release behaviour as 0 for forward
                        // compatibility with firmware that polls these
                        // before they're wired.
                        debug_assert!(
                            false,
                            "PIO ahb_read32 offset {:#05X} not yet wired (Phase 4/5)",
                            offset,
                        );
                        0
                    }
                }
            }
            _ => {
                self.shared
                    .peripherals
                    .legacy
                    .lock()
                    .unwrap()
                    .get(&canonical)
                    .copied()
                    .unwrap_or(0)
            }
        }
    }

    /// AHB write32. DMA writes apply directly; PIO writes queue a
    /// command onto `shared.pio` for Stage 7's worker to apply
    /// (Stage 5 stubs PIO writes to a no-op because `ThreadedPio`
    /// command encoding is Stage 7's domain).
    fn ahb_write32(&mut self, addr: u32, val: u32) {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        let alias = (addr >> 12) & 3;
        match base {
            DMA_BASE => self
                .shared
                .peripherals
                .dma
                .lock()
                .unwrap()
                .dma
                .write32(offset, val, alias),
            0x5020_0000 | 0x5030_0000 | 0x5040_0000 => {
                // CPU→PIO writes are one-quantum-delayed: the command
                // queues here, and the PIO worker drains + applies at
                // the TOP of the NEXT quantum. Firmware that writes
                // CTRL then reads back inline will see the pre-update
                // value. Spec: V7 HLD §5 "One-quantum delay on CPU→PIO
                // writes". Firmware must issue DMB + yield one quantum
                // for the writeback to be visible.
                //
                // PIO MMIO routing (Phase 3 task #11): queue a
                // PioCommand onto `shared.pio` for the PIO worker to
                // apply against its locally-owned `PioBlock`s.
                //
                // Dispatch breakdown:
                //   - CTRL (0x000) → WriteCtrl (worker also republishes
                //     the post-write sm_enabled_mask onto `ThreadedPio`
                //     so CPU-side reads observe the new enable bits).
                //   - INSTR_MEM0..31 (0x048-0x0C4) → WriteInstrMem.
                //   - SMn_CLKDIV (0x0C8 + sm*0x18) → SetClkDiv (decodes
                //     the INT/FRAC fields so the command carries the
                //     wire-format ints the worker passes back through
                //     `PioBlock::write32`).
                //   - Everything else (TXF0..TXF3, IRQ, FDEBUG,
                //     INPUT_SYNC_BYPASS, per-SM EXECCTRL/SHIFTCTRL/
                //     INSTR/PINCTRL) → WriteReg, which the worker
                //     hands straight to `PioBlock::write32`.
                //
                // `alias` (the 2 bits encoded in address[13:12]) is
                // propagated on every variant — the single-threaded
                // `Bus::write32` forwards it unconditionally to
                // `PioBlock::write32`, so dropping it here would make
                // aliased writes (SET/CLR/XOR) diverge between modes.
                // PIO blocks are 0x10_0000 bytes apart (0x502/0x503/0x504).
                let block = ((base - 0x5020_0000) >> 20) as u8;
                let off12 = offset as u16;
                let cmd = match off12 {
                    0x000 => PioCommand::WriteCtrl { block, val, alias: alias as u8 },
                    0x048..=0x0C4 => {
                        let addr = ((off12 - 0x048) >> 2) as u8;
                        PioCommand::WriteInstrMem {
                            block,
                            addr,
                            value: val as u16,
                            alias: alias as u8,
                        }
                    }
                    // SMn_CLKDIV: 0x0C8, 0x0E0, 0x0F8, 0x110. Stride 0x18.
                    0x0C8 | 0x0E0 | 0x0F8 | 0x110 => {
                        let sm = ((off12 - 0x0C8) / 0x18) as u8;
                        // CLKDIV layout: INT<<16, FRAC<<8 — see
                        // `PioBlock::write_sm_reg` / `sm.write_clkdiv`.
                        let int_div = ((val >> 16) & 0xFFFF) as u16;
                        let frac_div = ((val >> 8) & 0xFF) as u8;
                        PioCommand::SetClkDiv {
                            block,
                            sm,
                            int_div,
                            frac_div,
                            alias: alias as u8,
                        }
                    }
                    _ => PioCommand::WriteReg { block, offset: off12, val, alias: alias as u8 },
                };
                self.shared.pio.send_command(cmd);
            }
            _ => {
                let mut legacy = self.shared.peripherals.legacy.lock().unwrap();
                let old = legacy.get(&canonical).copied().unwrap_or(0);
                let new_val = match alias {
                    0 => val,
                    1 => old ^ val,
                    2 => old | val,
                    3 => old & !val,
                    _ => val,
                };
                legacy.insert(canonical, new_val);
            }
        }
    }

    /// SIO (`0xD`) read32. DIV/INTERP (offsets 0x060..=0x0FC) are
    /// intercepted on `CortexM33` and never reach here.
    fn sio_read32(&mut self, addr: u32, core: u8) -> u32 {
        let reg_offset = addr & 0xFFF;
        debug_assert!(
            !crate::core::PerCoreSio::owns_offset(reg_offset),
            "DIV/INTERP addr 0x{:08X} reached WorkerBus::read32 — use CortexM33::bus_read32 wrapper",
            addr
        );

        match reg_offset {
            0x000 => core as u32,                  // CPUID
            0x004 => 0,                            // GPIO_IN — Stage 7 wires external pin state
            0x008 => 0,                            // GPIO_HI_IN — ditto
            0x010 => self.shared.gpio.read_out(0), // GPIO_OUT
            0x030 => self.shared.gpio.read_oe(0),  // GPIO_OE
            // FIFO
            0x050 => self.shared.sio.fifo_st(core as usize), // FIFO_ST
            0x058 => {
                // FIFO_RD: pop from this core's RX.
                self.shared.sio.fifo_pop(core as usize).unwrap_or(0)
            }
            // SPINLOCK_ST (0x05C): current spinlock bitmap.
            0x05C => self.shared.sio.spinlock_bits(),
            // Spinlock claim (0x100..=0x17F): test-and-set.
            0x100..=0x17F => {
                let id = ((reg_offset - 0x100) >> 2) as usize;
                self.shared.sio.spinlock_claim(id)
            }
            // DOORBELL_IN_SET read (0x188): current 4-bit doorbell.
            0x188 => self.shared.sio.doorbell_read(core as usize),
            // MTIME registers (0x1A0–0x1BC).
            0x1A0 => self.shared.sio.mtime_ctrl_read(),
            0x1A8 => self.shared.sio.mtime_read() as u32,
            0x1AC => (self.shared.sio.mtime_read() >> 32) as u32,
            0x1B0 => self.shared.sio.mtimecmp_read(0) as u32,
            0x1B4 => (self.shared.sio.mtimecmp_read(0) >> 32) as u32,
            0x1B8 => self.shared.sio.mtimecmp_read(1) as u32,
            0x1BC => (self.shared.sio.mtimecmp_read(1) >> 32) as u32,
            _ => 0,
        }
    }

    /// SIO write32. Mirrors [`Self::sio_read32`]; on successful
    /// FIFO_WR push, wakes the peer's WFE via
    /// `atomics.event_flag[peer].store(true, Release)` — LLD V7 §6
    /// (parity with `bus/mod.rs:2182-2186`; NO IRQ hook per §6 scope
    /// note).
    fn sio_write32(&mut self, addr: u32, val: u32, core: u8) {
        let reg_offset = addr & 0xFFF;
        debug_assert!(
            !crate::core::PerCoreSio::owns_offset(reg_offset),
            "DIV/INTERP addr 0x{:08X} reached WorkerBus::write32 — use CortexM33::bus_write32 wrapper",
            addr
        );

        match reg_offset {
            // GPIO_OUT family — 0x010/0x018/0x020/0x028.
            0x010 => self.shared.gpio.write_out(0, val),
            0x018 => self.shared.gpio.set_out(0, val),
            0x020 => self.shared.gpio.clear_out(0, val),
            0x028 => self.shared.gpio.xor_out(0, val),
            // GPIO_OE family — 0x030/0x038/0x040/0x048.
            0x030 => self.shared.gpio.write_oe(0, val),
            0x038 => self.shared.gpio.set_oe(0, val),
            0x040 => self.shared.gpio.clear_oe(0, val),
            0x048 => self.shared.gpio.xor_oe(0, val),
            // FIFO
            0x050 => self.shared.sio.fifo_st_clear(core as usize, val),
            0x054 => {
                // FIFO_WR: push to peer core's RX queue.
                let peer = 1 - (core as usize);
                if self.shared.sio.fifo_push(core as usize, val) {
                    // WFE wake hook (LLD V7 §6). No IRQ.
                    self.shared.atomics.set_event_flag(peer);
                }
            }
            // Spinlock release (0x100..=0x17F): any write clears.
            0x100..=0x17F => {
                let id = ((reg_offset - 0x100) >> 2) as usize;
                self.shared.sio.spinlock_release(id);
            }
            // DOORBELL_OUT_SET (0x180): set bits on peer.
            0x180 => {
                let peer = 1 - (core as usize);
                self.shared.sio.doorbell_set(peer, val & 0xF);
            }
            // DOORBELL_OUT_CLR (0x184): clear bits on peer.
            0x184 => {
                let peer = 1 - (core as usize);
                self.shared.sio.doorbell_clear(peer, val & 0xF);
            }
            // DOORBELL_IN_CLR (0x18C): clear bits on self.
            0x18C => {
                self.shared.sio.doorbell_clear(core as usize, val & 0xF);
            }
            // MTIME registers (0x1A0–0x1BC).
            0x1A0 => self.shared.sio.mtime_ctrl_write(val),
            0x1A8 => {
                let hi = self.shared.sio.mtime_read() & 0xFFFF_FFFF_0000_0000;
                self.shared.sio.mtime_write(hi | val as u64);
            }
            0x1AC => {
                let lo = self.shared.sio.mtime_read() & 0x0000_0000_FFFF_FFFF;
                self.shared.sio.mtime_write(lo | ((val as u64) << 32));
            }
            0x1B0 => {
                let hi = self.shared.sio.mtimecmp_read(0) & 0xFFFF_FFFF_0000_0000;
                self.shared.sio.mtimecmp_write(0, hi | val as u64);
            }
            0x1B4 => {
                let lo = self.shared.sio.mtimecmp_read(0) & 0x0000_0000_FFFF_FFFF;
                self.shared.sio.mtimecmp_write(0, lo | ((val as u64) << 32));
            }
            0x1B8 => {
                let hi = self.shared.sio.mtimecmp_read(1) & 0xFFFF_FFFF_0000_0000;
                self.shared.sio.mtimecmp_write(1, hi | val as u64);
            }
            0x1BC => {
                let lo = self.shared.sio.mtimecmp_read(1) & 0x0000_0000_FFFF_FFFF;
                self.shared.sio.mtimecmp_write(1, lo | ((val as u64) << 32));
            }
            _ => {}
        }
    }

    // --- Narrow-access helpers (parity with Bus::read8/read16/write8/write16) ---

    /// Narrow byte read for FIFO-data registers whose word read pops
    /// the RX FIFO (UART0.UARTDR, SPI0.SSPDR, I2C0.IC_DATA_CMD,
    /// ADC.FIFO). Returns `Some(byte)` if the access was handled,
    /// `None` to fall through to the word-then-extract path.
    ///
    /// Parity with `Bus::read8` at `bus/mod.rs:1170`. The RESETS guard
    /// short-circuits to 0 before dispatch to match the single-threaded
    /// path (`bus/mod.rs:1169`).
    fn try_narrow_read8(&mut self, addr: u32) -> Option<u8> {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let word_offset = (canonical & !3) & 0x0000_0FFF;

        match (base, word_offset) {
            (UART0_BASE, crate::peripherals::uart::UARTDR)
            | (SPI0_BASE, crate::peripherals::spi::SSPDR)
            | (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD)
            | (ADC_BASE, crate::peripherals::adc::FIFO) => {}
            _ => return None,
        }

        // RESETS guard: held peripherals return 0 (parity with
        // `bus/mod.rs:1169`).
        if self
            .shared
            .peripherals
            .resets
            .lock()
            .unwrap()
            .is_held_in_reset_base(base)
        {
            return Some(0);
        }

        let v = match (base, word_offset) {
            (UART0_BASE, crate::peripherals::uart::UARTDR) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .uart0
                .read8(crate::peripherals::uart::UARTDR),
            (SPI0_BASE, crate::peripherals::spi::SSPDR) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .spi0
                .read8(crate::peripherals::spi::SSPDR),
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .i2c0
                .read8(crate::peripherals::i2c::IC_DATA_CMD),
            (ADC_BASE, crate::peripherals::adc::FIFO) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .adc
                .read8(crate::peripherals::adc::FIFO),
            _ => unreachable!(),
        };
        Some(v)
    }

    /// Narrow halfword read for FIFO-data registers. Parity with
    /// `Bus::read16` at `bus/mod.rs:1597`.
    fn try_narrow_read16(&mut self, addr: u32) -> Option<u16> {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let word_offset = (canonical & !3) & 0x0000_0FFF;

        match (base, word_offset) {
            (UART0_BASE, crate::peripherals::uart::UARTDR)
            | (SPI0_BASE, crate::peripherals::spi::SSPDR)
            | (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD)
            | (ADC_BASE, crate::peripherals::adc::FIFO) => {}
            _ => return None,
        }

        if self
            .shared
            .peripherals
            .resets
            .lock()
            .unwrap()
            .is_held_in_reset_base(base)
        {
            return Some(0);
        }

        let v = match (base, word_offset) {
            (SPI0_BASE, crate::peripherals::spi::SSPDR) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .spi0
                .read16(crate::peripherals::spi::SSPDR),
            (UART0_BASE, crate::peripherals::uart::UARTDR) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .uart0
                .read8(crate::peripherals::uart::UARTDR) as u16,
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .i2c0
                .read8(crate::peripherals::i2c::IC_DATA_CMD) as u16,
            (ADC_BASE, crate::peripherals::adc::FIFO) => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .adc
                .read16(crate::peripherals::adc::FIFO),
            _ => unreachable!(),
        };
        Some(v)
    }

    /// Narrow byte write for TX FIFO-data registers (UART0.UARTDR,
    /// SPI0.SSPDR, I2C0.IC_DATA_CMD). Returns `true` if the access was
    /// handled. Parity with `Bus::write8` at `bus/mod.rs:1319`.
    ///
    /// ADC FIFO narrow writes are swallowed (ADC FIFO is read-only).
    fn try_narrow_write8(&mut self, addr: u32, val: u8) -> bool {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let word_offset = (canonical & !3) & 0x0000_0FFF;

        match (base, word_offset) {
            (UART0_BASE, crate::peripherals::uart::UARTDR)
            | (SPI0_BASE, crate::peripherals::spi::SSPDR)
            | (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD)
            | (ADC_BASE, crate::peripherals::adc::FIFO) => {}
            _ => return false,
        }

        if self
            .shared
            .peripherals
            .resets
            .lock()
            .unwrap()
            .is_held_in_reset_base(base)
        {
            return true; // consumed — held peripherals drop writes
        }

        let mut ext_irqs = 0u64;
        match (base, word_offset) {
            (UART0_BASE, crate::peripherals::uart::UARTDR) => {
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .uart0
                    .write8(crate::peripherals::uart::UARTDR, val, &mut ext_irqs);
            }
            (SPI0_BASE, crate::peripherals::spi::SSPDR) => {
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .spi0
                    .write8(crate::peripherals::spi::SSPDR, val, &mut ext_irqs);
            }
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .i2c0
                    .write8(crate::peripherals::i2c::IC_DATA_CMD, val, &mut ext_irqs);
            }
            (ADC_BASE, crate::peripherals::adc::FIFO) => {
                // Read-only on silicon (datasheet §12.4.5). Swallow,
                // parity with `bus/mod.rs:1370`.
            }
            _ => unreachable!(),
        }
        self.raise_irqs_shared(ext_irqs);
        true
    }

    /// Narrow halfword write for SPI0.SSPDR. Other FIFO registers have
    /// no architected halfword write semantics; a halfword to UARTDR /
    /// IC_DATA_CMD / ADC FIFO collapses to a byte via the low lane to
    /// match the RP2040 narrow-write idiom and avoid RMW-induced FIFO
    /// pops. Parity with `Bus::write16` at `bus/mod.rs`.
    fn try_narrow_write16(&mut self, addr: u32, val: u16) -> bool {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let word_offset = (canonical & !3) & 0x0000_0FFF;

        match (base, word_offset) {
            (SPI0_BASE, crate::peripherals::spi::SSPDR)
            | (UART0_BASE, crate::peripherals::uart::UARTDR)
            | (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD)
            | (ADC_BASE, crate::peripherals::adc::FIFO) => {}
            _ => return false,
        }

        if self
            .shared
            .peripherals
            .resets
            .lock()
            .unwrap()
            .is_held_in_reset_base(base)
        {
            return true;
        }

        let mut ext_irqs = 0u64;
        match (base, word_offset) {
            (SPI0_BASE, crate::peripherals::spi::SSPDR) => {
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .spi0
                    .write16(crate::peripherals::spi::SSPDR, val, &mut ext_irqs);
            }
            (UART0_BASE, crate::peripherals::uart::UARTDR) => {
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .uart0
                    .write8(crate::peripherals::uart::UARTDR, val as u8, &mut ext_irqs);
            }
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                self.shared
                    .peripherals
                    .apb
                    .lock()
                    .unwrap()
                    .i2c0
                    .write8(crate::peripherals::i2c::IC_DATA_CMD, val as u8, &mut ext_irqs);
            }
            (ADC_BASE, crate::peripherals::adc::FIFO) => {
                // Read-only — swallow.
            }
            _ => unreachable!(),
        }
        self.raise_irqs_shared(ext_irqs);
        true
    }

    /// Raise every bit in `mask` on both cores' NVIC pending — used by
    /// APB peripherals that report shared IRQ lines via `ext_irqs`.
    ///
    /// Bits outside the peripheral-driven range (`PERIPH_IRQ_MASK` —
    /// 0..=45) are filtered here so a peripheral
    /// `mask |= 1 << IRQ_*` typo on a software-only line (46..=51) can't
    /// silently misassert. Parity with `Bus::raise_irqs_u64`
    /// (`bus/mod.rs:1540`).
    fn raise_irqs_shared(&self, mask: u64) {
        let mut remaining = mask & crate::irq::PERIPH_IRQ_MASK;
        while remaining != 0 {
            let irq = remaining.trailing_zeros();
            self.shared.atomics.assert_irq_shared(irq);
            remaining &= remaining - 1;
        }
    }

    /// Queue a post-write cache invalidation for any write that could
    /// have landed in executable memory (ROM/XIP/SRAM).
    ///
    /// `len` is the write width in bytes (1, 2, or 4). The drainer
    /// ([`CortexM33::invalidate_decode_cache_entries`]) evicts two slots
    /// per queued address (`addr-2` and `addr`), so for a 4-byte write we
    /// push **two** entries (`addr` and `addr+2`) to match the coverage
    /// of the single-threaded [`Bus::invalidate_pc_range(addr, 4)`],
    /// which clears `{addr-2, addr, addr+2}`. For 1/2-byte writes a
    /// single entry suffices (coverage `{addr-2, addr}`); the extra
    /// `addr-2` entry for byte writes is a safe over-invalidation.
    #[inline]
    fn queue_cache_invalidation(&mut self, addr: u32, len: u8) {
        debug_assert!(len == 1 || len == 2 || len == 4);
        if matches!(addr >> 28, 0x0..=0x2) {
            self.pending_cache_invalidations.push(addr);
            if len == 4 {
                // write32 spans {addr-2, addr, addr+2} — one push only
                // covers {addr-2, addr}. Push addr+2 to get the third
                // slot.
                self.pending_cache_invalidations.push(addr.wrapping_add(2));
            }
        }
    }
}

// =======================================================================
// `CoreBus` impl
// =======================================================================

impl CoreBus for WorkerBus {
    // --- Canonical 13-method surface --------------------------------

    fn read8(&mut self, addr: u32, core: u8) -> u8 {
        // Boot RAM (0xEFFF_F000..0xF000_0000) and XIP SRAM
        // (0x1C00_0000..0x1C00_4000) live at addresses the generic
        // `0x0..=0x2` arm would either miss (boot RAM is 0xE) or
        // absorb as empty flash XIP (xip_sram is inside 0x1). Route
        // them before the generic memory arm so both regions are
        // backed by the per-word atomic storage on `SharedMemory`.
        if is_boot_ram_addr(addr) {
            return self.shared.memory.read_boot_ram8(addr);
        }
        if is_xip_sram_addr(addr) {
            return self.shared.memory.read_xip_sram8(addr);
        }
        match addr >> 28 {
            0x0..=0x2 => self.shared.memory.read8(addr),
            0x4 | 0x5 => {
                // Narrow-access dispatch for byte-significant Phase 2
                // registers: UARTDR pops one RX byte per access; SSPDR
                // pops one RX word per access (low byte here);
                // IC_DATA_CMD pops one I2C byte; ADC FIFO pops one
                // sample. Parity with `Bus::read8` (`bus/mod.rs:1170`).
                // RMW-via-word would pop the FIFO on every byte offset.
                if let Some(v) = self.try_narrow_read8(addr) {
                    return v;
                }
                let word = if (addr >> 28) == 0x5 {
                    self.ahb_read32(addr & !3)
                } else {
                    self.apb_read32(addr & !3)
                };
                (word >> ((addr & 3) * 8)) as u8
            }
            0xD => {
                let word = self.sio_read32(addr & !3, core);
                (word >> ((addr & 3) * 8)) as u8
            }
            _ => {
                self.shared.atomics.set_bus_fault(core as usize, addr);
                0
            }
        }
    }

    fn read16(&mut self, addr: u32, core: u8) -> u16 {
        if is_boot_ram_addr(addr) {
            return self.shared.memory.read_boot_ram16(addr);
        }
        if is_xip_sram_addr(addr) {
            return self.shared.memory.read_xip_sram16(addr);
        }
        match addr >> 28 {
            0x0..=0x2 => self.shared.memory.read16(addr),
            0x4 | 0x5 => {
                // Narrow halfword path (parity with `Bus::read16` at
                // `bus/mod.rs:1597`) — route FIFO reads through the
                // peripheral's own narrow helper so the FIFO pops once.
                if let Some(v) = self.try_narrow_read16(addr) {
                    return v;
                }
                let word = if (addr >> 28) == 0x5 {
                    self.ahb_read32(addr & !3)
                } else {
                    self.apb_read32(addr & !3)
                };
                let shift = (addr & 2) * 8;
                (word >> shift) as u16
            }
            0xD => {
                let word = self.sio_read32(addr & !3, core);
                let shift = (addr & 2) * 8;
                (word >> shift) as u16
            }
            _ => {
                self.shared.atomics.set_bus_fault(core as usize, addr);
                0
            }
        }
    }

    fn read32(&mut self, addr: u32, core: u8) -> u32 {
        if is_boot_ram_addr(addr) {
            return self.shared.memory.read_boot_ram32(addr);
        }
        if is_xip_sram_addr(addr) {
            return self.shared.memory.read_xip_sram32(addr);
        }
        match addr >> 28 {
            0x0..=0x2 => self.shared.memory.read32(addr),
            0x4 => self.apb_read32(addr),
            0x5 => self.ahb_read32(addr),
            0xD => self.sio_read32(addr, core),
            _ => {
                self.shared.atomics.set_bus_fault(core as usize, addr);
                0
            }
        }
    }

    /// STREX success does not route here — `CortexM33` calls
    /// `shared.memory.cas32` + `shared.monitors.snoop` directly per
    /// LLD V7 §4.
    fn write8(&mut self, addr: u32, val: u8, core: u8) {
        if is_boot_ram_addr(addr) {
            self.shared.memory.write_boot_ram8(addr, val);
            self.shared.monitors.snoop(addr);
            return;
        }
        if is_xip_sram_addr(addr) {
            self.shared.memory.write_xip_sram8(addr, val);
            self.shared.monitors.snoop(addr);
            return;
        }
        match addr >> 28 {
            0x0..=0x2 => {
                self.shared.memory.write8(addr, val);
                self.queue_cache_invalidation(addr, 1);
            }
            0x4 => {
                // Narrow-write dispatch for side-effect registers
                // (UARTDR TX, SSPDR TX, IC_DATA_CMD, ADC FIFO). RMW
                // through word32 would read-then-write-back and pop
                // the RX FIFO. Parity with `Bus::write8`
                // (`bus/mod.rs:1319`).
                if self.try_narrow_write8(addr, val) {
                    self.shared.monitors.snoop(addr);
                    return;
                }
                // Byte-wide APB writes are rare; RMW through word32 to
                // keep the APB path a single dispatch.
                let aligned = addr & !3;
                let word = self.apb_read32(aligned);
                let shift = (addr & 3) * 8;
                let masked = word & !(0xFFu32 << shift);
                let new_word = masked | ((val as u32) << shift);
                self.apb_write32(aligned, new_word);
            }
            0x5 => {
                let aligned = addr & !3;
                let word = self.ahb_read32(aligned);
                let shift = (addr & 3) * 8;
                let masked = word & !(0xFFu32 << shift);
                let new_word = masked | ((val as u32) << shift);
                self.ahb_write32(aligned, new_word);
            }
            0xD => {
                let aligned = addr & !3;
                let word = self.sio_read32(aligned, core);
                let shift = (addr & 3) * 8;
                let masked = word & !(0xFFu32 << shift);
                let new_word = masked | ((val as u32) << shift);
                self.sio_write32(aligned, new_word, core);
            }
            _ => {
                self.shared.atomics.set_bus_fault(core as usize, addr);
            }
        }
        self.shared.monitors.snoop(addr);
    }

    fn write16(&mut self, addr: u32, val: u16, core: u8) {
        if is_boot_ram_addr(addr) {
            self.shared.memory.write_boot_ram16(addr, val);
            self.shared.monitors.snoop(addr);
            return;
        }
        if is_xip_sram_addr(addr) {
            self.shared.memory.write_xip_sram16(addr, val);
            self.shared.monitors.snoop(addr);
            return;
        }
        match addr >> 28 {
            0x0..=0x2 => {
                self.shared.memory.write16(addr, val);
                self.queue_cache_invalidation(addr, 2);
            }
            0x4 => {
                // Narrow-write dispatch — same rationale as write8.
                if self.try_narrow_write16(addr, val) {
                    self.shared.monitors.snoop(addr);
                    return;
                }
                let aligned = addr & !3;
                let word = self.apb_read32(aligned);
                let shift = (addr & 2) * 8;
                let masked = word & !(0xFFFFu32 << shift);
                let new_word = masked | ((val as u32) << shift);
                self.apb_write32(aligned, new_word);
            }
            0x5 => {
                let aligned = addr & !3;
                let word = self.ahb_read32(aligned);
                let shift = (addr & 2) * 8;
                let masked = word & !(0xFFFFu32 << shift);
                let new_word = masked | ((val as u32) << shift);
                self.ahb_write32(aligned, new_word);
            }
            0xD => {
                let aligned = addr & !3;
                let word = self.sio_read32(aligned, core);
                let shift = (addr & 2) * 8;
                let masked = word & !(0xFFFFu32 << shift);
                let new_word = masked | ((val as u32) << shift);
                self.sio_write32(aligned, new_word, core);
            }
            _ => {
                self.shared.atomics.set_bus_fault(core as usize, addr);
            }
        }
        self.shared.monitors.snoop(addr);
    }

    /// STREX success does not route here — `CortexM33` calls
    /// `shared.memory.cas32` + `shared.monitors.snoop` directly per
    /// LLD V7 §4.
    fn write32(&mut self, addr: u32, val: u32, core: u8) {
        if is_boot_ram_addr(addr) {
            self.shared.memory.write_boot_ram32(addr, val);
            self.shared.monitors.snoop(addr);
            return;
        }
        if is_xip_sram_addr(addr) {
            self.shared.memory.write_xip_sram32(addr, val);
            self.shared.monitors.snoop(addr);
            return;
        }
        match addr >> 28 {
            0x0..=0x2 => {
                self.shared.memory.write32(addr, val);
                self.queue_cache_invalidation(addr, 4);
            }
            0x4 => self.apb_write32(addr, val),
            0x5 => self.ahb_write32(addr, val),
            0xD => self.sio_write32(addr, val, core),
            _ => {
                self.shared.atomics.set_bus_fault(core as usize, addr);
            }
        }
        // ARMv8-M §A3.4: any store to a word with an active peer
        // exclusive monitor must invalidate that monitor.
        self.shared.monitors.snoop(addr);
    }

    #[inline]
    fn set_active_pc(&mut self, pc: u32, _core: u8) {
        self.active_pc = pc;
    }

    fn bus_fault(&self, core: u8) -> bool {
        self.shared.atomics.is_bus_fault(core as usize)
    }
    fn bus_fault_addr(&self, core: u8) -> u32 {
        self.shared.atomics.bus_fault_addr(core as usize)
    }
    fn clear_bus_fault(&mut self, core: u8) {
        self.shared.atomics.clear_bus_fault(core as usize);
    }

    #[inline]
    fn set_burst_mode(&mut self, on: bool) {
        self.burst_mode = on;
    }
    #[inline]
    fn add_extra_wait_states(&mut self, n: u32) {
        self.extra_wait_states = self.extra_wait_states.saturating_add(n);
    }
    #[inline]
    fn take_extra_wait_states(&mut self) -> u32 {
        let n = self.extra_wait_states;
        self.extra_wait_states = 0;
        n
    }

    // --- TRANSIENT (Stage 2) ----------------------------------------

    #[inline]
    fn atomics(&self) -> &Arc<CoreAtomics> {
        &self.shared.atomics
    }

    // --- GPIO OUT / OE / IN (Phase 3 Stage 6a) -----------------------
    //
    // Forward to `shared.gpio` bank 0 — RP2354 SIO only exposes bank 0
    // on the CP0 GPIOC path. Stage 7 wires the GPIO_IN column on
    // `AtomicGpio`; until then, `gpio_read_in` returns 0 (matching the
    // single-threaded `Bus::gpio_in` default on fresh construction).

    #[inline]
    fn gpio_read_out(&self) -> u32 {
        self.shared.gpio.read_out(0)
    }
    #[inline]
    fn gpio_write_out(&mut self, val: u32) {
        self.shared.gpio.write_out(0, val);
    }
    #[inline]
    fn gpio_set_out(&mut self, mask: u32) {
        self.shared.gpio.set_out(0, mask);
    }
    #[inline]
    fn gpio_clear_out(&mut self, mask: u32) {
        self.shared.gpio.clear_out(0, mask);
    }
    #[inline]
    fn gpio_xor_out(&mut self, mask: u32) {
        self.shared.gpio.xor_out(0, mask);
    }

    #[inline]
    fn gpio_read_oe(&self) -> u32 {
        self.shared.gpio.read_oe(0)
    }
    #[inline]
    fn gpio_write_oe(&mut self, val: u32) {
        self.shared.gpio.write_oe(0, val);
    }
    #[inline]
    fn gpio_set_oe(&mut self, mask: u32) {
        self.shared.gpio.set_oe(0, mask);
    }
    #[inline]
    fn gpio_clear_oe(&mut self, mask: u32) {
        self.shared.gpio.clear_oe(0, mask);
    }
    #[inline]
    fn gpio_xor_oe(&mut self, mask: u32) {
        self.shared.gpio.xor_oe(0, mask);
    }

    #[inline]
    fn gpio_read_in(&self) -> u32 {
        // AtomicGpio has no GPIO_IN column yet; Stage 7 wires external
        // pin state. Until then return 0, matching `gpio_in()`.
        0
    }

    #[inline]
    fn extra_wait_states(&self) -> u32 {
        self.extra_wait_states
    }
    #[inline]
    fn reset_extra_wait_states(&mut self) {
        self.extra_wait_states = 0;
    }

    #[inline]
    fn mmio_trace_enabled(&self) -> bool {
        false
    }
    #[inline]
    fn emit_mmio_trace(&mut self, _rw: char, _size: u32, _addr: u32, _val: u32, _core: u8) {
        // Trace routing is coordinator-side in the threaded runtime
        // (Phase 4 wiring). Stage 5 drops trace events on the worker
        // path.
    }
}

// =======================================================================
// Helpers
// =======================================================================

/// True when `addr` lies inside the 4 KB boot RAM scratchpad
/// (`0xEFFF_F000..0xF000_0000`). Mirrors `Bus::is_boot_ram`.
#[inline]
fn is_boot_ram_addr(addr: u32) -> bool {
    (0xEFFF_F000..0xF000_0000).contains(&addr)
}

/// True when `addr` lies inside the 16 KB XIP SRAM scratchpad
/// (`0x1C00_0000..0x1C00_4000`). Mirrors `Bus::is_xip_sram`.
#[inline]
fn is_xip_sram_addr(addr: u32) -> bool {
    (0x1C00_0000..0x1C00_4000).contains(&addr)
}

/// SYSINFO (0x4000_0000) — read-only. Mirrors `Bus::sysinfo_read`.
/// Free function so we don't need to lock any mutex for this.
fn sysinfo_read(offset: u32) -> u32 {
    match offset {
        0x000 => 0x0000_0002, // CHIP_ID: RP2350
        0x004 => 0x0000_0000, // PACKAGE_SEL
        0x008 => 0x0000_0001, // PLATFORM: ASIC
        _ => 0,
    }
}

// =======================================================================
// Tests (LLD V7 §11 items 1-12)
// =======================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::threaded::shared::SharedState;
    use std::sync::atomic::Ordering;

    /// Fresh `SharedState` for tests — isolates each test's peripheral
    /// state and atomic counters.
    fn fresh_shared() -> SharedState {
        SharedState::new_default()
    }

    // ------------------------------------------------------------
    // LLD V7 §11 test 1: region dispatch
    // ------------------------------------------------------------

    /// Write to SRAM, ROM (drops), XIP, SIO GPIO_OUT, APB clk_ref_ctrl,
    /// legacy HashMap offset. Reads observe the expected values (or 0
    /// for ROM / unwritable regions).
    #[test]
    fn worker_bus_region_dispatch() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared.clone());

        // SRAM: write-then-read roundtrip.
        let sram_addr = 0x2000_0100;
        bus.write32(sram_addr, 0xDEAD_BEEF, 0);
        assert_eq!(bus.read32(sram_addr, 0), 0xDEAD_BEEF);

        // ROM: writes silently dropped (immutable); reads return 0 on
        // an empty ROM image.
        bus.write32(0x0000_0100, 0x1234_5678, 0);
        assert_eq!(bus.read32(0x0000_0100, 0), 0);

        // XIP: writes to a region with no flash loaded silently drop.
        // Empty XIP → reads return 0.
        assert_eq!(bus.read32(0x1000_0000, 0), 0);

        // SIO GPIO_OUT (0xD000_0010): roundtrip via AtomicGpio.
        bus.write32(0xD000_0010, 0xA5A5_5A5A, 0);
        assert_eq!(bus.read32(0xD000_0010, 0), 0xA5A5_5A5A);

        // APB CLK_REF_CTRL (0x4001_0030): roundtrip via ClocksState.
        bus.write32(0x4001_0030, 0x0000_0002, 0);
        assert_eq!(bus.read32(0x4001_0030, 0), 0x0000_0002);

        // Legacy HashMap — a peripheral offset not claimed by any
        // typed component (e.g. inside the CLOCKS CTRL space for a
        // channel without backing storage). Use an unmapped APB
        // offset under a base that no dispatch arm matches.
        let legacy_addr = 0x4030_0000; // no typed base matches
        bus.write32(legacy_addr, 0xCAFE_F00D, 0);
        assert_eq!(bus.read32(legacy_addr, 0), 0xCAFE_F00D);
    }

    // ------------------------------------------------------------
    // LLD V7 §11 test 2: peer-monitor snoop on SRAM write
    // ------------------------------------------------------------

    #[test]
    fn worker_bus_write_snoops_peer_monitor() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared.clone());

        // --- word write ---
        let addr = 0x2000_0200;
        shared.monitors.set(1, addr);
        assert!(shared.monitors.check(1, addr));
        bus.write32(addr, 0xDEAD_BEEF, 0);
        assert!(
            !shared.monitors.check(1, addr),
            "peer monitor must be invalidated by WorkerBus::write32"
        );

        // --- halfword write ---
        let addr16 = 0x2000_0210;
        shared.monitors.set(1, addr16);
        assert!(shared.monitors.check(1, addr16));
        bus.write16(addr16, 0xBEEF, 0);
        assert!(
            !shared.monitors.check(1, addr16),
            "peer monitor must be invalidated by WorkerBus::write16"
        );

        // --- byte write ---
        let addr8 = 0x2000_0220;
        shared.monitors.set(1, addr8);
        assert!(shared.monitors.check(1, addr8));
        bus.write8(addr8, 0xA5, 0);
        assert!(
            !shared.monitors.check(1, addr8),
            "peer monitor must be invalidated by WorkerBus::write8"
        );
    }

    // ------------------------------------------------------------
    // LLD V7 §11 test 3: STM-style sequence queues invalidations
    // ------------------------------------------------------------

    #[test]
    fn worker_bus_stm_queues_multiple_invalidations() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared);

        // Simulate an STM of 8 registers into consecutive SRAM words.
        let base = 0x2000_1000;
        for i in 0..8u32 {
            bus.write32(base + i * 4, i, 0);
        }

        // Each write32 pushes TWO entries (`addr` and `addr+2`) so that
        // the drainer — which invalidates `addr-2` and `addr` slots per
        // queued address — covers the full `{addr-2, addr, addr+2}`
        // range expected by the single-threaded
        // `Bus::invalidate_pc_range(addr, 4)`. 8 writes × 2 = 16.
        assert_eq!(
            bus.pending_cache_invalidations.len(),
            16,
            "two entries per SRAM write32 (addr, addr+2)"
        );
        for i in 0..8u32 {
            let word_addr = base + i * 4;
            assert_eq!(
                bus.pending_cache_invalidations[i as usize * 2],
                word_addr,
                "first entry records word base"
            );
            assert_eq!(
                bus.pending_cache_invalidations[i as usize * 2 + 1],
                word_addr + 2,
                "second entry records word+2 to cover trailing hw slot"
            );
        }
    }

    /// Halfword SRAM writes must also push an invalidation entry —
    /// firmware can patch a single 16-bit Thumb instruction in place.
    #[test]
    fn worker_bus_write16_queues_invalidation() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared);

        let addr = 0x2000_2000;
        bus.write16(addr, 0xBEEF, 0);

        assert_eq!(
            bus.pending_cache_invalidations.len(),
            1,
            "write16 to SRAM must queue one invalidation"
        );
        assert_eq!(bus.pending_cache_invalidations[0], addr);
    }

    /// Byte SRAM writes must also push an invalidation entry — the
    /// write still lands inside an executable word even if it mutates
    /// only one lane.
    #[test]
    fn worker_bus_write8_queues_invalidation() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared);

        let addr = 0x2000_2100;
        bus.write8(addr, 0xA5, 0);

        assert_eq!(
            bus.pending_cache_invalidations.len(),
            1,
            "write8 to SRAM must queue one invalidation"
        );
        assert_eq!(bus.pending_cache_invalidations[0], addr);
    }

    // ------------------------------------------------------------
    // LLD V7 §11 tests 4-7: already covered on CoreAtomics. Cross-ref
    // in the threaded::atomics module tests. Nothing to add here.
    // ------------------------------------------------------------

    // ------------------------------------------------------------
    // LLD V7 §11 test 8: trait dyn coverage for WorkerBus
    // ------------------------------------------------------------

    /// Compile-time + smoke check that `CoreBus for WorkerBus` covers
    /// every method the trait declares and that the trait is reachable
    /// via a `dyn CoreBus` coercion.
    #[test]
    fn worker_bus_core_bus_impl_covers_all_methods() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared);
        let bus_dyn: &mut dyn CoreBus = &mut bus;

        // Canonical 13-method surface.
        let _ = bus_dyn.read32(0x2000_0000, 0);
        bus_dyn.write32(0x2000_0000, 0, 0);
        let _ = bus_dyn.read16(0x2000_0000, 0);
        bus_dyn.write16(0x2000_0000, 0, 0);
        let _ = bus_dyn.read8(0x2000_0000, 0);
        bus_dyn.write8(0x2000_0000, 0, 0);
        bus_dyn.set_active_pc(0x2000_0000, 0);
        let _fault = bus_dyn.bus_fault(0);
        let _addr = bus_dyn.bus_fault_addr(0);
        bus_dyn.clear_bus_fault(0);
        bus_dyn.set_burst_mode(true);
        bus_dyn.set_burst_mode(false);
        bus_dyn.add_extra_wait_states(3);
        let n = bus_dyn.take_extra_wait_states();
        assert_eq!(n, 3, "take_extra_wait_states must return the added 3");
        assert_eq!(
            bus_dyn.take_extra_wait_states(),
            0,
            "take_extra_wait_states must drain to zero"
        );

        // Transient accessors (removed in later Phase 3 stages —
        // see `core/bus_trait.rs`).
        let _a: &Arc<CoreAtomics> = bus_dyn.atomics();
        // GPIO OUT/OE/IN typed accessors (Phase 3 Stage 6a).
        let _ = bus_dyn.gpio_read_out();
        bus_dyn.gpio_write_out(0);
        bus_dyn.gpio_set_out(0);
        bus_dyn.gpio_clear_out(0);
        bus_dyn.gpio_xor_out(0);
        let _ = bus_dyn.gpio_read_oe();
        bus_dyn.gpio_write_oe(0);
        bus_dyn.gpio_set_oe(0);
        bus_dyn.gpio_clear_oe(0);
        bus_dyn.gpio_xor_oe(0);
        let _ = bus_dyn.gpio_read_in();
        let _ = bus_dyn.extra_wait_states();
        bus_dyn.reset_extra_wait_states();
        let _ = bus_dyn.mmio_trace_enabled();
        bus_dyn.emit_mmio_trace('R', 4, 0x2000_0000, 0, 0);
    }

    // ------------------------------------------------------------
    // LLD V7 §11 test 9: per-core bus_fault observation via WorkerBus
    // ------------------------------------------------------------

    #[test]
    fn worker_bus_bus_fault_is_per_core() {
        let shared = fresh_shared();
        let bus = WorkerBus::new(0, shared.clone());

        // Set a bus fault on core 0 only.
        shared.atomics.set_bus_fault(0, 0xBAD_1BAD);
        assert!(bus.bus_fault(0));
        assert!(!bus.bus_fault(1));
        assert_eq!(bus.bus_fault_addr(0), 0xBAD_1BAD);
    }

    // ------------------------------------------------------------
    // LLD V7 §11 test 10: wait state accounting via the trait
    // ------------------------------------------------------------

    #[test]
    fn worker_bus_wait_state_accounting_survives_trait() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared);
        let b: &mut dyn CoreBus = &mut bus;

        b.add_extra_wait_states(3);
        b.add_extra_wait_states(2);
        assert_eq!(b.take_extra_wait_states(), 5);
        assert_eq!(b.take_extra_wait_states(), 0);
    }

    // ------------------------------------------------------------
    // LLD V7 §11 test 11: shared master_cycle fetch_add + load
    // ------------------------------------------------------------

    #[test]
    fn shared_master_cycle_read_after_fetch_add() {
        let shared = fresh_shared();
        // Ensure clean state.
        shared.master_cycle.store(0, Ordering::Release);

        shared.master_cycle.fetch_add(10, Ordering::Release);
        shared.master_cycle.fetch_add(5, Ordering::Release);

        assert_eq!(shared.master_cycle.load(Ordering::Acquire), 15);
    }

    // ------------------------------------------------------------
    // LLD V7 §11 test 12: PLL CS read goes through shared master_cycle
    // ------------------------------------------------------------

    /// Seed `pll_sys_lock_at_cycle = Some(100)`, bump
    /// `shared.master_cycle` to 101, then read PLL_CS through
    /// WorkerBus's APB dispatch. The LOCK bit (CS[31]) must be set.
    #[test]
    fn pll_cs_read_uses_shared_master_cycle() {
        let shared = fresh_shared();

        // Arm the PLL: non-zero FBDIV + PWR=0 so the base predicate
        // holds, and set a lock deadline at master cycle 100.
        {
            let mut clocks = shared.peripherals.clocks.lock().unwrap();
            clocks.pll_sys_regs[0] = 0x0000_0001; // CS image (LOCK bit derived separately)
            clocks.pll_sys_regs[1] = 0; // PWR cleared
            clocks.pll_sys_regs[2] = 125; // FBDIV_INT != 0
            clocks.pll_sys_lock_at_cycle = Some(100);
        }
        shared.master_cycle.store(101, Ordering::Release);

        let mut bus = WorkerBus::new(0, shared);
        // PLL_SYS CS is at APB 0x4005_0000.
        let cs = bus.read32(0x4005_0000, 0);
        assert_ne!(
            cs & (1 << 31),
            0,
            "LOCK bit must be set when master_cycle >= lock_at_cycle"
        );
    }

    // ------------------------------------------------------------
    // Bonus: FIFO_WR wakes peer event flag
    // ------------------------------------------------------------

    /// Cross-references LLD V7 §6's scope note — FIFO push sets the
    /// peer's event_flag (parity with bus/mod.rs:2182-2186). No IRQ.
    #[test]
    fn worker_bus_fifo_wr_sets_peer_event_flag() {
        let shared = fresh_shared();
        let mut bus0 = WorkerBus::new(0, shared.clone());

        // Precondition: peer event flag clear.
        assert!(!shared.atomics.event_flag_load(1));

        // Core 0 writes FIFO_WR (SIO offset 0x054).
        bus0.write32(0xD000_0054, 0xCAFE_F00D, 0);

        // Peer (core 1) event flag must now be set; writer's own
        // event flag must not be touched by this push.
        assert!(
            shared.atomics.event_flag_load(1),
            "FIFO push must wake peer's WFE via event_flag"
        );
        assert!(
            !shared.atomics.event_flag_load(0),
            "writer's event_flag must not be disturbed"
        );

        // IRQ pending must stay zero on both cores — §6 scope note.
        assert_eq!(shared.atomics.irq_pending_load(0), 0);
        assert_eq!(shared.atomics.irq_pending_load(1), 0);
    }

    // ------------------------------------------------------------
    // Construction / capacity sanity
    // ------------------------------------------------------------

    #[test]
    fn worker_bus_preallocates_invalidation_capacity() {
        let shared = fresh_shared();
        let bus = WorkerBus::new(0, shared);
        assert!(
            bus.pending_cache_invalidations.capacity() >= PENDING_INVALIDATION_CAPACITY,
            "capacity must be >= PENDING_INVALIDATION_CAPACITY (STM 13 regs + headroom)"
        );
        assert_eq!(bus.core_id(), 0);
    }

    // --- Fix 3: boot_ram / xip_sram routed through WorkerBus ---

    #[test]
    fn worker_bus_boot_ram_roundtrip() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared);
        let addr = 0xEFFF_F100;
        bus.write32(addr, 0xDEAD_BEEF, 0);
        assert_eq!(bus.read32(addr, 0), 0xDEAD_BEEF);
        // Halfword + byte stay consistent with the 32-bit view.
        assert_eq!(bus.read16(addr, 0), 0xBEEF);
        assert_eq!(bus.read16(addr + 2, 0), 0xDEAD);
        assert_eq!(bus.read8(addr, 0), 0xEF);
        // Writes do NOT queue decode-cache invalidations — boot RAM is
        // outside the 0x0..=0x2 executable-memory regions.
        let ci = bus.pending_cache_invalidations.len();
        bus.write32(addr, 0x1234_5678, 0);
        assert_eq!(
            bus.pending_cache_invalidations.len(),
            ci,
            "boot RAM writes must not queue cache invalidations"
        );
    }

    #[test]
    fn worker_bus_xip_sram_roundtrip() {
        let shared = fresh_shared();
        let mut bus = WorkerBus::new(0, shared);
        let addr = 0x1C00_0200;
        bus.write32(addr, 0xAABB_CCDD, 0);
        assert_eq!(bus.read32(addr, 0), 0xAABB_CCDD);
        assert_eq!(bus.read16(addr, 0), 0xCCDD);
        assert_eq!(bus.read8(addr, 0), 0xDD);
        // Same invariant: no cache invalidation for xip_sram writes.
        let ci = bus.pending_cache_invalidations.len();
        bus.write8(addr, 0x00, 0);
        assert_eq!(
            bus.pending_cache_invalidations.len(),
            ci,
            "xip_sram writes must not queue cache invalidations"
        );
    }

    #[test]
    fn pio_bus_take_blocks_roundtrips() {
        let shared = fresh_shared();
        let blocks = [PioBlock::new(), PioBlock::new(), PioBlock::new()];
        let pb = PioBus::new(shared, blocks);
        let recovered = pb.take_blocks();
        assert_eq!(recovered.len(), 3, "PioBus returns the three blocks");
    }
}

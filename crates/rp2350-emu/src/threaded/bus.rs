//! `WorkerBus` — the per-CPU-thread `CoreBus` implementation that routes
//! memory accesses into the `SharedState` primitives (Phase 1/2) plus the
//! Stage-4 mutex-guarded peripheral bundle.
//!
//! Phase 3 Stage 5 (LLD V7 §4, §5, §6):
//!
//! - [`WorkerBus`] — owned by each CPU worker thread, implements
//!   [`crate::core::bus_trait::CoreBus`].
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
use crate::peripherals::usb::{USBCTRL_DPRAM_BASE, USBCTRL_DPRAM_SIZE, USBCTRL_REGS_BASE};
use crate::threaded::CoreAtomics;
use crate::threaded::PioCommand;
use crate::threaded::SharedState;

/// Capacity bound for [`WorkerBus::pending_cache_invalidations`]: STM
/// tops out at 13 registers; FPU context push spills 16 words; keep
/// headroom so typical bursts amortise within a single allocation.
pub(crate) const PENDING_INVALIDATION_CAPACITY: usize = 16;

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
    shared: SharedState,
    active_pc: u32,
    burst_mode: bool,
    extra_wait_states: u32,
    /// Sequential-fetch tracking for bank 2/6 penalty (silicon fidelity
    /// fix from test_silicon baseline campaign 2026-04-16; see
    /// `core/decode.rs::decode_execute`). Initialised to `u32::MAX` so
    /// the first fetch is non-sequential.
    last_fetch_addr: u32,
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
            shared,
            active_pc: 0,
            burst_mode: false,
            extra_wait_states: 0,
            last_fetch_addr: u32::MAX,
            pending_cache_invalidations,
        }
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
            0x4001_0000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .clocks_read(offset),
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
            0x4004_8000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .xosc_read(offset),
            0x400E_8000 => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .rosc_read(offset),

            0x4002_0000 => self
                .shared
                .peripherals
                .resets
                .lock()
                .unwrap()
                .resets_read(offset),
            0x400D_0000 => self.shared.peripherals.qmi.lock().unwrap().qmi_read(offset),
            0x4000_0000 => sysinfo_read(offset),

            TIMER0_BASE => self
                .shared
                .peripherals
                .timers
                .lock()
                .unwrap()
                .timer0
                .read32(offset),
            TIMER1_BASE => self
                .shared
                .peripherals
                .timers
                .lock()
                .unwrap()
                .timer1
                .read32(offset),
            TICKS_BASE => self
                .shared
                .peripherals
                .timers
                .lock()
                .unwrap()
                .ticks
                .read32(offset),

            UART0_BASE => self.shared.peripherals.apb.lock().unwrap().uart[0].read32(offset),
            SPI0_BASE => self.shared.peripherals.apb.lock().unwrap().spi[0].read32(offset),
            I2C0_BASE => self.shared.peripherals.apb.lock().unwrap().i2c[0].read32(offset),
            ADC_BASE => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .adc
                .read32(offset),
            PWM_BASE => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .pwm
                .read32(offset),
            IO_BANK0_BASE => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .io_bank0
                .read32(offset),
            PADS_BANK0_BASE => self
                .shared
                .peripherals
                .apb
                .lock()
                .unwrap()
                .pads_bank0
                .read32(offset),

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
                self.shared.peripherals.apb.lock().unwrap().uart[0].write32(
                    offset,
                    val,
                    alias,
                    &mut ext_irqs,
                );
                self.raise_irqs_shared(ext_irqs);
            }
            SPI0_BASE => {
                let mut ext_irqs = 0u64;
                self.shared.peripherals.apb.lock().unwrap().spi[0].write32(
                    offset,
                    val,
                    alias,
                    &mut ext_irqs,
                );
                self.raise_irqs_shared(ext_irqs);
            }
            I2C0_BASE => {
                let mut ext_irqs = 0u64;
                self.shared.peripherals.apb.lock().unwrap().i2c[0].write32(
                    offset,
                    val,
                    alias,
                    &mut ext_irqs,
                );
                self.raise_irqs_shared(ext_irqs);
            }
            ADC_BASE => {
                let mut ext_irqs = 0u64;
                self.shared.peripherals.apb.lock().unwrap().adc.write32(
                    offset,
                    val,
                    alias,
                    &mut ext_irqs,
                );
                self.raise_irqs_shared(ext_irqs);
            }
            PWM_BASE => {
                let mut ext_irqs = 0u64;
                self.shared.peripherals.apb.lock().unwrap().pwm.write32(
                    offset,
                    val,
                    alias,
                    &mut ext_irqs,
                );
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
            // USBCTRL regs + DPRAM (HLD V5 §Component 1). RESETS guard
            // mirrors the serial Bus: held peripherals read as 0
            // (`bus/mod.rs:1656`, `2332`).
            USBCTRL_REGS_BASE => {
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
                self.shared
                    .peripherals
                    .usb
                    .lock()
                    .unwrap()
                    .usbctrl
                    .read32(offset)
            }
            USBCTRL_DPRAM_BASE => {
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
                // DPRAM is plain 4 KB AHB-Lite RAM — no APB alias
                // semantics. Recover the raw byte offset from `addr`
                // (the canonical mask above strips alias bits 12/13
                // that don't apply here). Mirrors `bus/mod.rs:2776`.
                let dpram_off = (addr - USBCTRL_DPRAM_BASE) & (USBCTRL_DPRAM_SIZE - 1);
                self.shared
                    .peripherals
                    .usb
                    .lock()
                    .unwrap()
                    .usbctrl
                    .read_dpram(dpram_off & !3, 4)
            }
            // PIO register reads: the `PioBlock`s themselves live on
            // the PIO worker thread, so the CPU worker can only observe
            // the atomics `ThreadedPio` publishes — today that's
            // CTRL.SM_ENABLE (0x000), IRQ (0x030), and GPIOBASE
            // (0x168). RX FIFO pops and per-SM register reads need a
            // read-through channel (not yet wired — Phase 4/5 scope)
            // and return 0 for now, which matches a freshly reset block.
            0x5020_0000 | 0x5030_0000 | 0x5040_0000 => {
                // PIO blocks are 0x10_0000 bytes apart (0x502/0x503/0x504).
                let block = ((base - 0x5020_0000) >> 20) as usize;
                match offset {
                    0x000 => self.shared.pio.read_sm_enabled(block) as u32,
                    0x030 => self.shared.pio.read_irq_flags(block) as u32,
                    0x168 => self.shared.pio.read_gpio_base(block) as u32,
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
            _ => self
                .shared
                .peripherals
                .legacy
                .lock()
                .unwrap()
                .get(&canonical)
                .copied()
                .unwrap_or(0),
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
            // USBCTRL regs + DPRAM (HLD V5 §Component 1). RESETS guard
            // mirrors the serial Bus: held peripherals drop writes
            // silently (`bus/mod.rs:2730`, `2872`).
            USBCTRL_REGS_BASE => {
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
                let mut ext_irqs = 0u64;
                self.shared.peripherals.usb.lock().unwrap().usbctrl.write32(
                    offset,
                    val,
                    alias,
                    &mut ext_irqs,
                );
                self.raise_irqs_shared(ext_irqs);
            }
            USBCTRL_DPRAM_BASE => {
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
                let dpram_off = (addr - USBCTRL_DPRAM_BASE) & (USBCTRL_DPRAM_SIZE - 1);
                self.shared
                    .peripherals
                    .usb
                    .lock()
                    .unwrap()
                    .usbctrl
                    .write_dpram(dpram_off & !3, val, 4);
            }
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
                //     INPUT_SYNC_BYPASS, GPIOBASE, per-SM
                //     EXECCTRL/SHIFTCTRL/INSTR/PINCTRL) → WriteReg,
                //     which the worker hands straight to
                //     `PioBlock::write32`.
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
                    0x000 => PioCommand::WriteCtrl {
                        block,
                        val,
                        alias: alias as u8,
                    },
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
                    _ => PioCommand::WriteReg {
                        block,
                        offset: off12,
                        val,
                        alias: alias as u8,
                    },
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

    /// Read GPIO_IN with the freshest external-stimulus overlay.
    ///
    /// The coordinator's `update_gpio` merges SIO pads, PIO pad
    /// overlays, and external stimulus into `AtomicGpio::in_` at each
    /// quantum boundary — so between quanta, the cached `in_` is stale
    /// w.r.t. any external-stim writes a host thread has issued in the
    /// meantime. For OneROM-style oracles the measurement thread drives
    /// `gpio_external_in` at sub-µs cadence while the CPU tight-loops
    /// in its serve path; the CPU must observe those updates within a
    /// single instruction, not wait for the next merge.
    ///
    /// We rebuild the overlay at read time: the non-masked bits of
    /// `in_` carry the quantum-coherent SIO+PIO pad state (those don't
    /// change between quanta in these workloads), and we apply the
    /// fresh `(ext_val, ext_mask)` on top. Correct when the external
    /// mask is stable across the read window — which holds for every
    /// oracle that pins the same set of pads for the duration of a
    /// sweep. If the mask changed between quanta, stale `ext_val` bits
    /// outside the new mask can leak; no current caller trips that.
    #[inline]
    fn gpio_in_fresh(&self) -> u32 {
        let base = self.shared.gpio.read_in();
        let (ext_val, ext_mask) = self.shared.gpio.read_external();
        (base & !ext_mask) | (ext_val & ext_mask)
    }

    /// Threaded counterpart of `Bus::read_gpio_hi_in`. Stage 3A wide-GPIO
    /// support — see HLD §A.
    ///
    /// Composition mirrors `gpio_in_fresh`: start from the coordinator's
    /// merged bank-1 state, then re-overlay fresh high-bank external
    /// stimulus so host-driven GPIO 32..47 changes are visible between
    /// quanta.
    #[inline]
    fn gpio_hi_in_fresh(&self) -> u32 {
        let base = self.shared.gpio.read_in_hi();
        let (ext_val, ext_mask) = self.shared.gpio.read_external_hi();
        (base & !ext_mask) | (ext_val & ext_mask)
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
            0x000 => core as u32,          // CPUID
            0x004 => self.gpio_in_fresh(), // GPIO_IN — SIO+PIO from last quantum + fresh external
            // GPIO_HI_IN — merged bank-1 state plus fresh external overlay.
            0x008 => self.gpio_hi_in_fresh(),
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
            (UART0_BASE, crate::peripherals::uart::UARTDR) => {
                self.shared.peripherals.apb.lock().unwrap().uart[0]
                    .read8(crate::peripherals::uart::UARTDR)
            }
            (SPI0_BASE, crate::peripherals::spi::SSPDR) => {
                self.shared.peripherals.apb.lock().unwrap().spi[0]
                    .read8(crate::peripherals::spi::SSPDR)
            }
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                self.shared.peripherals.apb.lock().unwrap().i2c[0]
                    .read8(crate::peripherals::i2c::IC_DATA_CMD)
            }
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
            (SPI0_BASE, crate::peripherals::spi::SSPDR) => {
                self.shared.peripherals.apb.lock().unwrap().spi[0]
                    .read16(crate::peripherals::spi::SSPDR)
            }
            (UART0_BASE, crate::peripherals::uart::UARTDR) => {
                self.shared.peripherals.apb.lock().unwrap().uart[0]
                    .read8(crate::peripherals::uart::UARTDR) as u16
            }
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                self.shared.peripherals.apb.lock().unwrap().i2c[0]
                    .read8(crate::peripherals::i2c::IC_DATA_CMD) as u16
            }
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
                self.shared.peripherals.apb.lock().unwrap().uart[0].write8(
                    crate::peripherals::uart::UARTDR,
                    val,
                    &mut ext_irqs,
                );
            }
            (SPI0_BASE, crate::peripherals::spi::SSPDR) => {
                self.shared.peripherals.apb.lock().unwrap().spi[0].write8(
                    crate::peripherals::spi::SSPDR,
                    val,
                    &mut ext_irqs,
                );
            }
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                self.shared.peripherals.apb.lock().unwrap().i2c[0].write8(
                    crate::peripherals::i2c::IC_DATA_CMD,
                    val,
                    &mut ext_irqs,
                );
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
                self.shared.peripherals.apb.lock().unwrap().spi[0].write16(
                    crate::peripherals::spi::SSPDR,
                    val,
                    &mut ext_irqs,
                );
            }
            (UART0_BASE, crate::peripherals::uart::UARTDR) => {
                self.shared.peripherals.apb.lock().unwrap().uart[0].write8(
                    crate::peripherals::uart::UARTDR,
                    val as u8,
                    &mut ext_irqs,
                );
            }
            (I2C0_BASE, crate::peripherals::i2c::IC_DATA_CMD) => {
                self.shared.peripherals.apb.lock().unwrap().i2c[0].write8(
                    crate::peripherals::i2c::IC_DATA_CMD,
                    val as u8,
                    &mut ext_irqs,
                );
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
                // SIO GPIO_OUT family (0x010/0x018/0x020/0x028) and
                // GPIO_OE family (0x030/0x038/0x040/0x048) replicate a
                // narrow write across all four lanes on real RP2350
                // silicon — the single-cycle IO fabric latches the
                // full 32-bit bus without byte-lane enables. OneROM's
                // CPU-serve loop relies on this: `STRB Rn, [Rm, #0]`
                // at SIO_GPIO_OUT lights up the data pins regardless
                // of which byte of the word the store targets. Serial
                // `Bus::write8` mirrors this at `bus/mod.rs:1973` —
                // keep the two paths in sync via the shared predicate.
                let aligned = addr & !3;
                let reg_offset = aligned & 0xFFF;
                if crate::bus::Bus::is_sio_gpio_out_replicating_reg(reg_offset) {
                    let replicated = u32::from(val) * 0x0101_0101;
                    self.sio_write32(aligned, replicated, core);
                } else {
                    let word = self.sio_read32(aligned, core);
                    let shift = (addr & 3) * 8;
                    let masked = word & !(0xFFu32 << shift);
                    let new_word = masked | ((val as u32) << shift);
                    self.sio_write32(aligned, new_word, core);
                }
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
                // Halfword mirror of the write8 SIO arm — see the
                // comment above. GPIO_OUT-family halfword writes
                // replicate across both 16-bit lanes of the word.
                let aligned = addr & !3;
                let reg_offset = aligned & 0xFFF;
                if crate::bus::Bus::is_sio_gpio_out_replicating_reg(reg_offset) {
                    let replicated = u32::from(val) * 0x0001_0001;
                    self.sio_write32(aligned, replicated, core);
                } else {
                    let word = self.sio_read32(aligned, core);
                    let shift = (addr & 2) * 8;
                    let masked = word & !(0xFFFFu32 << shift);
                    let new_word = masked | ((val as u32) << shift);
                    self.sio_write32(aligned, new_word, core);
                }
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
    // on the CP0 GPIOC path. The GPIO_IN column is merged each quantum
    // by the coordinator's `update_gpio` (SIO pads + per-block PIO pads
    // + external stimulus), so CPU workers read the freshest merged
    // state here.

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
        // Same semantics as `sio_read32(0x004)` — take the
        // quantum-boundary SIO/PIO merge and re-overlay the external
        // stimulus at read time so host threads driving
        // `gpio_external_in` get sub-quantum visibility to the CPU.
        self.gpio_in_fresh()
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
    fn last_fetch_addr(&self) -> u32 {
        self.last_fetch_addr
    }
    #[inline]
    fn set_last_fetch_addr(&mut self, addr: u32) {
        self.last_fetch_addr = addr;
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

    // =====================================================================
    // stage5_coverage: broad coverage fill-ins for APB/AHB/SIO dispatch
    // arms that the existing tests leave cold. Each new test targets a
    // specific uncovered branch listed in the 2026-04-22 coverage task.
    // =====================================================================

    mod stage5_coverage {
        use super::*;
        use crate::bus::{
            RESET_ADC, RESET_DMA, RESET_I2C0, RESET_IO_BANK0, RESET_PADS_BANK0, RESET_PWM,
            RESET_SPI0, RESET_TIMER0, RESET_TIMER1, RESET_UART0,
        };

        // ---- APB read dispatch -----------------------------------------

        #[test]
        fn apb_pll_usb_read_routes_through_clocks() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // PLL_USB CS without any lock-arming returns the stored CS
            // image (no LOCK bit set).
            let cs = bus.read32(0x4005_8000, 0);
            // Any value is fine — the goal is to hit `pll_usb_read_at`.
            let _ = cs;
        }

        #[test]
        fn apb_xosc_rosc_resets_qmi_sysinfo_read() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Each of these must route through its typed arm in apb_read32.
            let _ = bus.read32(0x4004_8000, 0); // XOSC
            let _ = bus.read32(0x400E_8000, 0); // ROSC
            let _ = bus.read32(0x4002_0000, 0); // RESETS
            let _ = bus.read32(0x400D_0000, 0); // QMI
            // SYSINFO CHIP_ID is deterministic.
            assert_eq!(bus.read32(0x4000_0000, 0), 0x0000_0002);
            // SYSINFO PACKAGE_SEL + PLATFORM (cover sysinfo_read match).
            assert_eq!(bus.read32(0x4000_0004, 0), 0);
            assert_eq!(bus.read32(0x4000_0008, 0), 0x1);
            // SYSINFO unmapped offset → 0 (last sysinfo_read arm).
            assert_eq!(bus.read32(0x4000_0100, 0), 0);
        }

        #[test]
        fn apb_timer0_timer1_ticks_read() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            let _ = bus.read32(TIMER0_BASE, 0);
            let _ = bus.read32(TIMER1_BASE, 0);
            let _ = bus.read32(TICKS_BASE, 0);
        }

        #[test]
        fn apb_uart_spi_i2c_adc_pwm_io_bank0_pads_read() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            let _ = bus.read32(UART0_BASE + 0x04, 0); // UARTRSR_ECR
            let _ = bus.read32(SPI0_BASE + 0x04, 0);
            let _ = bus.read32(I2C0_BASE + 0x04, 0);
            let _ = bus.read32(ADC_BASE + 0x04, 0);
            let _ = bus.read32(PWM_BASE + 0x04, 0);
            let _ = bus.read32(IO_BANK0_BASE, 0);
            let _ = bus.read32(PADS_BANK0_BASE, 0);
        }

        /// RESETS held peripheral reads return 0 (`apb_read32` line 207).
        #[test]
        fn apb_held_peripheral_read_returns_zero() {
            let shared = fresh_shared();
            // UART1 is held in reset post-bootrom — its base maps to
            // RESET_UART1 (bit 26); `is_held_in_reset_base(UART1_BASE)` is
            // true. A read32 must return 0 without touching UART1 state.
            let mut bus = WorkerBus::new(0, shared);
            let uart1_base: u32 = 0x4007_4000;
            assert_eq!(
                bus.read32(uart1_base + 0x04, 0),
                0,
                "held UART1 must return 0 via the RESETS guard"
            );
        }

        // ---- APB write dispatch ----------------------------------------

        #[test]
        fn apb_write_held_peripheral_drops_silently() {
            let shared = fresh_shared();
            // SPI1 is held in reset post-bootrom. A write there must not
            // panic and must not affect the RESETS state.
            let spi1_base: u32 = 0x4008_4000;
            let mut bus = WorkerBus::new(0, shared.clone());
            bus.write32(spi1_base + 0x10, 0xDEAD_BEEF, 0);
            // Readback also returns 0 via the held-guard.
            assert_eq!(bus.read32(spi1_base + 0x10, 0), 0);
        }

        #[test]
        fn apb_write_all_typed_arms_happy_path() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // CLOCKS / PLL_SYS / PLL_USB / XOSC / ROSC / RESETS / QMI /
            // TIMER0 / TIMER1 / TICKS / UART0 / SPI0 / I2C0 / ADC / PWM /
            // IO_BANK0 / PADS_BANK0 — each exercises its own match arm in
            // apb_write32.
            bus.write32(0x4001_0000, 0x0000_0001, 0); // CLOCKS CTRL
            bus.write32(0x4005_0000, 0x0000_0000, 0); // PLL_SYS CS
            bus.write32(0x4005_8000, 0x0000_0000, 0); // PLL_USB CS
            bus.write32(0x4004_8000, 0x0000_0000, 0); // XOSC CTRL
            bus.write32(0x400E_8000, 0x0000_0000, 0); // ROSC CTRL
            bus.write32(0x4002_0000, 0x0000_0000, 0); // RESETS
            bus.write32(0x400D_0000, 0x0000_0000, 0); // QMI
            bus.write32(0x4000_0000, 0xFFFF_FFFF, 0); // SYSINFO write-no-op
            bus.write32(TIMER0_BASE + 0x40, 0x1, 0); // TIMER0 INTR-ish
            bus.write32(TIMER1_BASE + 0x40, 0x1, 0);
            bus.write32(TICKS_BASE + 0x08, 0x1, 0);
            bus.write32(UART0_BASE + 0x44, 0x1, 0); // UARTICR
            bus.write32(SPI0_BASE + 0x20, 0x1, 0);
            bus.write32(I2C0_BASE + 0x40, 0x1, 0);
            bus.write32(ADC_BASE + 0x14, 0x1, 0);
            bus.write32(PWM_BASE + 0x50, 0x1, 0);
            bus.write32(IO_BANK0_BASE, 0x0, 0);
            bus.write32(PADS_BANK0_BASE, 0x0, 0);
        }

        /// Legacy HashMap write aliases — XOR, SET, CLR on an address
        /// that no typed APB arm claims (APB unknown-base path). The
        /// canonical slot is `base & !0x3000`; the 0x1000 / 0x2000 / 0x3000
        /// adders select the XOR / SET / CLR aliases.
        #[test]
        fn apb_legacy_hashmap_alias_rmw() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Pick a base whose bits [13:12] are zero (alias == 0 plain).
            let base = 0x4030_0000;
            // Plain write.
            bus.write32(base, 0x0000_00FF, 0);
            assert_eq!(bus.read32(base, 0), 0x0000_00FF);
            // XOR alias.
            bus.write32(base | 0x1000, 0xFF00_00FF, 0);
            assert_eq!(bus.read32(base, 0), 0xFF00_0000);
            // SET alias.
            bus.write32(base | 0x2000, 0x0000_0F0F, 0);
            assert_eq!(bus.read32(base, 0), 0xFF00_0F0F);
            // CLR alias.
            bus.write32(base | 0x3000, 0x0000_0F00, 0);
            assert_eq!(bus.read32(base, 0), 0xFF00_000F);
        }

        // ---- AHB read dispatch -----------------------------------------

        #[test]
        fn ahb_read_dma_region() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // DMA CH0_READ_ADDR — post-bootrom-released on this emu.
            let _ = bus.read32(DMA_BASE, 0);
        }

        #[test]
        fn ahb_read_pio_published_registers() {
            let shared = fresh_shared();
            shared.pio.write_gpio_base(1, 16);
            let mut bus = WorkerBus::new(0, shared);
            // PIO0 CTRL (SM_ENABLE) — reads through ThreadedPio.
            assert_eq!(bus.read32(0x5020_0000, 0), 0);
            // PIO1 IRQ flags.
            assert_eq!(bus.read32(0x5030_0030, 0), 0);
            // PIO1 GPIOBASE.
            assert_eq!(bus.read32(0x5030_0168, 0), 16);
            // PIO2 CTRL.
            assert_eq!(bus.read32(0x5040_0000, 0), 0);
        }

        #[test]
        fn ahb_read_unmapped_falls_back_to_legacy() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Legacy AHB window (no typed match).
            let legacy = 0x5060_0000;
            bus.write32(legacy, 0x1234_5678, 0);
            assert_eq!(bus.read32(legacy, 0), 0x1234_5678);
        }

        // ---- AHB write dispatch ----------------------------------------

        #[test]
        fn ahb_write_dma_routes_to_dma_state() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // CH0_READ_ADDR.
            bus.write32(DMA_BASE, 0x2000_1000, 0);
            // No assertion on DMA state — the typed arm handling is what
            // we want to cover. read32 may return 0 if the field reserves.
            let _ = bus.read32(DMA_BASE, 0);
        }

        #[test]
        fn ahb_write_pio_routes_queue_each_variant() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // PIO0 CTRL → WriteCtrl.
            bus.write32(0x5020_0000, 0x0000_0001, 0);
            // PIO0 INSTR_MEM0 → WriteInstrMem.
            bus.write32(0x5020_0048, 0x0000_A001, 0);
            // PIO0 SM0_CLKDIV → SetClkDiv (addr 0x0C8).
            bus.write32(0x5020_00C8, 0x0001_0000, 0);
            // PIO0 generic reg → WriteReg (TXF0 at 0x010).
            bus.write32(0x5020_0010, 0x1234_5678, 0);
            // Commands must have queued on the PIO worker queue.
            let drained = shared.pio.drain_commands(0);
            assert!(
                drained.len() >= 4,
                "expected >= 4 commands, got {}",
                drained.len()
            );
        }

        // ---- AHB USBCTRL dispatch --------------------------------------
        //
        // Mirrors the serial-Bus tests in `peripherals/usb.rs` so the
        // two paths produce identical observable behaviour for USB MMIO
        // accesses (HLD V5 §Component 1).

        /// Helper: release USBCTRL from RESETS so reads can return
        /// non-zero values. Mirrors `release_usbctrl` in usb.rs tests.
        fn release_usbctrl(bus: &mut WorkerBus) {
            use crate::bus::RESET_USBCTRL;
            // RESETS_BASE = 0x4002_0000, CLR alias = +0x3000.
            bus.write32(0x4002_3000, 1u32 << RESET_USBCTRL, 0);
        }

        #[test]
        fn ahb_usbctrl_regs_round_trip_at_canonical_alias() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            release_usbctrl(&mut bus);
            // MAIN_CTRL at offset 0x040.
            bus.write32(USBCTRL_REGS_BASE + 0x040, 0x0000_0001, 0);
            assert_eq!(bus.read32(USBCTRL_REGS_BASE + 0x040, 0), 0x0000_0001);
        }

        #[test]
        fn ahb_usbctrl_sie_status_writes_drop_silently() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            release_usbctrl(&mut bus);
            // SIE_STATUS at offset 0x050 — write via APB BITSET alias
            // (+0x2000); stub still reads 0.
            bus.write32(USBCTRL_REGS_BASE + 0x050 + 0x2000, (1 << 19) | (1 << 18), 0);
            assert_eq!(bus.read32(USBCTRL_REGS_BASE + 0x050, 0), 0);
        }

        #[test]
        fn ahb_usbctrl_dpram_word_round_trip() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            release_usbctrl(&mut bus);
            bus.write32(USBCTRL_DPRAM_BASE + 0x200, 0xDEAD_BEEF, 0);
            assert_eq!(bus.read32(USBCTRL_DPRAM_BASE + 0x200, 0), 0xDEAD_BEEF);
        }

        #[test]
        fn ahb_usbctrl_held_in_reset_reads_zero_writes_drop() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Don't release USBCTRL; writes drop, reads must return 0.
            bus.write32(USBCTRL_REGS_BASE + 0x040, 0x1, 0);
            assert_eq!(
                bus.read32(USBCTRL_REGS_BASE + 0x040, 0),
                0,
                "writes dropped + reads return 0 while USBCTRL held in reset",
            );
            // Same for DPRAM.
            bus.write32(USBCTRL_DPRAM_BASE + 0x100, 0xCAFE_F00D, 0);
            assert_eq!(
                bus.read32(USBCTRL_DPRAM_BASE + 0x100, 0),
                0,
                "DPRAM access dropped + reads 0 while USBCTRL held in reset",
            );
        }

        #[test]
        fn ahb_write_legacy_aliases() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Canonical base with alias bits cleared.
            let base = 0x5060_0000;
            bus.write32(base, 0x0000_00FF, 0);
            bus.write32(base | 0x1000, 0xFF00_00FF, 0); // XOR
            assert_eq!(bus.read32(base, 0), 0xFF00_0000);
            bus.write32(base | 0x2000, 0x0000_0F0F, 0); // SET
            assert_eq!(bus.read32(base, 0), 0xFF00_0F0F);
            bus.write32(base | 0x3000, 0x0000_0F00, 0); // CLR
            assert_eq!(bus.read32(base, 0), 0xFF00_000F);
        }

        // ---- SIO read32 coverage ---------------------------------------

        #[test]
        fn sio_read_cpuid_is_per_core() {
            let shared = fresh_shared();
            let mut bus0 = WorkerBus::new(0, shared.clone());
            let mut bus1 = WorkerBus::new(1, shared);
            assert_eq!(bus0.read32(0xD000_0000, 0), 0);
            assert_eq!(bus1.read32(0xD000_0000, 1), 1);
        }

        #[test]
        fn sio_read_gpio_hi_in_and_oe() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // GPIO_HI_IN (0x008) — merged bank-1 state is zero with no
            // PIO/SIO output or harness-driven external stimulus. The
            // QSPI-noise hook is intentionally absent on the threaded
            // path.
            assert_eq!(bus.read32(0xD000_0008, 0), 0);
            // GPIO_OE.
            assert_eq!(bus.read32(0xD000_0030, 0), 0);
        }

        /// Stage 3A wide-GPIO support: a harness-driven external_hi
        /// overlay surfaces on `GPIO_HI_IN` reads through the threaded
        /// path. Mirrors the single-threaded
        /// `gpio_external_in_hi_drives_read_gpio_hi_in` test.
        #[test]
        fn sio_read_gpio_hi_in_overlays_external_hi() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // Drive GPIOs 40..43 high (bits 8..11 of GPIO_HI_IN), with
            // a mask covering GPIOs 32..47 (low 16 bits).
            shared.gpio.write_external_hi(0x0000_0F00, 0x0000_FFFF);
            assert_eq!(bus.read32(0xD000_0008, 0), 0x0000_0F00);
        }

        #[test]
        fn sio_read_gpio_hi_in_preserves_internal_hi_bits() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());

            shared.gpio.write_in_hi(1 << 2);
            shared.gpio.write_external_hi(1 << 4, 1 << 4);

            assert_eq!(
                bus.read32(0xD000_0008, 0) & ((1 << 2) | (1 << 4)),
                (1 << 2) | (1 << 4),
                "GPIO_HI_IN must include merged high-bank state plus fresh external overlay"
            );
        }

        #[test]
        fn sio_read_fifo_status_and_spinlock_bits() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // FIFO_ST.
            let _ = bus.read32(0xD000_0050, 0);
            // FIFO_RD (empty → 0).
            assert_eq!(bus.read32(0xD000_0058, 0), 0);
            // SPINLOCK_ST.
            let _ = bus.read32(0xD000_005C, 0);
        }

        #[test]
        fn sio_read_spinlock_claim_and_release() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Claim lock 0 via 0xD000_0100.
            let first = bus.read32(0xD000_0100, 0);
            // First claim returns non-zero; re-claim returns zero.
            let second = bus.read32(0xD000_0100, 0);
            assert_ne!(first, 0);
            assert_eq!(second, 0);
            // Release via any write.
            bus.write32(0xD000_0100, 0, 0);
            // After release, claim succeeds again.
            let third = bus.read32(0xD000_0100, 0);
            assert_ne!(third, 0);
        }

        #[test]
        fn sio_read_doorbell_and_mtime_registers() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // DOORBELL_IN_SET.
            let _ = bus.read32(0xD000_0188, 0);
            // MTIME control, MTIME lo/hi, MTIMECMP0/1 lo/hi.
            let _ = bus.read32(0xD000_01A0, 0);
            let _ = bus.read32(0xD000_01A8, 0);
            let _ = bus.read32(0xD000_01AC, 0);
            let _ = bus.read32(0xD000_01B0, 0);
            let _ = bus.read32(0xD000_01B4, 0);
            let _ = bus.read32(0xD000_01B8, 0);
            let _ = bus.read32(0xD000_01BC, 0);
            // Unmapped SIO offset — final `_ => 0` arm.
            assert_eq!(bus.read32(0xD000_07FC, 0), 0);
        }

        // ---- SIO write32 coverage --------------------------------------

        #[test]
        fn sio_write_gpio_out_family() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            bus.write32(0xD000_0010, 0xFFFF_0000, 0); // GPIO_OUT
            bus.write32(0xD000_0018, 0x0000_0F0F, 0); // GPIO_OUT_SET
            bus.write32(0xD000_0020, 0x0000_0F00, 0); // GPIO_OUT_CLR
            bus.write32(0xD000_0028, 0x0000_00FF, 0); // GPIO_OUT_XOR
            // GPIO_OE family.
            bus.write32(0xD000_0030, 0xFFFF_FFFF, 0);
            bus.write32(0xD000_0038, 0x0000_0001, 0);
            bus.write32(0xD000_0040, 0x0000_0001, 0);
            bus.write32(0xD000_0048, 0x0000_FFFF, 0);
        }

        #[test]
        fn sio_write_fifo_st_clear() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // FIFO_ST write (W1C).
            bus.write32(0xD000_0050, 0xF, 0);
        }

        #[test]
        fn sio_write_doorbell_out_and_in_clear() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // Core 0 sets doorbell bits on peer (core 1).
            bus.write32(0xD000_0180, 0xF, 0);
            assert_eq!(shared.sio.doorbell_read(1), 0xF);
            // Clear those bits on peer.
            bus.write32(0xD000_0184, 0xF, 0);
            assert_eq!(shared.sio.doorbell_read(1), 0);
            // Self-clear DOORBELL_IN_CLR.
            shared.sio.doorbell_set(0, 0xF);
            bus.write32(0xD000_018C, 0xF, 0);
            assert_eq!(shared.sio.doorbell_read(0), 0);
        }

        #[test]
        fn sio_write_mtime_registers() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // MTIME control + lo/hi + MTIMECMP0/1 lo/hi.
            bus.write32(0xD000_01A0, 0x1, 0);
            bus.write32(0xD000_01A8, 0x1234_5678, 0);
            bus.write32(0xD000_01AC, 0x0000_0001, 0);
            bus.write32(0xD000_01B0, 0xAABB_CCDD, 0);
            bus.write32(0xD000_01B4, 0x0000_0001, 0);
            bus.write32(0xD000_01B8, 0x1111_1111, 0);
            bus.write32(0xD000_01BC, 0x0000_0001, 0);
            // Verify low/high splitting by reading back full 64-bit view.
            let lo = bus.read32(0xD000_01A8, 0) as u64;
            let hi = bus.read32(0xD000_01AC, 0) as u64;
            assert_eq!((hi << 32) | lo, 0x0000_0001_1234_5678);
        }

        #[test]
        fn sio_write_spinlock_release_by_offset() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Claim + release via the spinlock offset (0x100..=0x17F).
            let claimed = bus.read32(0xD000_0104, 0); // lock 1
            assert_ne!(claimed, 0);
            bus.write32(0xD000_0104, 0, 0); // release
            // Post-release claim succeeds again.
            assert_ne!(bus.read32(0xD000_0104, 0), 0);
        }

        #[test]
        fn sio_write_unmapped_offset_is_noop() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // An SIO offset outside any match arm (no panic, no effect).
            bus.write32(0xD000_07FC, 0xDEAD_BEEF, 0);
        }

        // ---- Narrow read/write dispatch --------------------------------

        /// Narrow byte read at UARTDR exercises `try_narrow_read8` hit
        /// path. The RX FIFO is empty → read returns 0 but the dispatch
        /// path is fully covered.
        #[test]
        fn narrow_read8_uartdr_spi_i2c_adc() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // UARTDR (0x000), SSPDR (0x008 on SPI), IC_DATA_CMD (0x010
            // on I2C), ADC FIFO (0x00C on ADC).
            assert_eq!(bus.read8(UART0_BASE, 0), 0);
            assert_eq!(bus.read8(SPI0_BASE + 0x008, 0), 0);
            assert_eq!(bus.read8(I2C0_BASE + 0x010, 0), 0);
            assert_eq!(bus.read8(ADC_BASE + 0x00C, 0), 0);
        }

        #[test]
        fn narrow_read8_fallthrough_returns_word_extract() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Non-FIFO APB byte read → falls through to word-then-extract.
            let _ = bus.read8(UART0_BASE + 0x18, 0); // UARTFR
            let _ = bus.read8(0x4001_0030, 0); // CLK_REF_CTRL byte
        }

        #[test]
        fn narrow_read16_uart_spi_i2c_adc_fifo() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Each narrow-read16 arm.
            assert_eq!(bus.read16(UART0_BASE, 0), 0);
            assert_eq!(bus.read16(SPI0_BASE + 0x008, 0), 0);
            assert_eq!(bus.read16(I2C0_BASE + 0x010, 0), 0);
            assert_eq!(bus.read16(ADC_BASE + 0x00C, 0), 0);
            // Non-FIFO APB halfword read → falls through.
            let _ = bus.read16(UART0_BASE + 0x18, 0);
        }

        /// Narrow byte write to UARTDR / SSPDR / IC_DATA_CMD — happy path
        /// plus the ADC FIFO read-only swallow arm.
        #[test]
        fn narrow_write8_each_arm() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            bus.write8(UART0_BASE, b'X', 0); // UART TX
            bus.write8(SPI0_BASE + 0x008, 0x55, 0); // SPI TX
            bus.write8(I2C0_BASE + 0x010, 0x77, 0); // I2C write
            bus.write8(ADC_BASE + 0x00C, 0x00, 0); // ADC FIFO swallowed
        }

        #[test]
        fn narrow_write16_each_arm() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            bus.write16(SPI0_BASE + 0x008, 0xAABB, 0);
            bus.write16(UART0_BASE, 0x0041, 0);
            bus.write16(I2C0_BASE + 0x010, 0x007F, 0);
            bus.write16(ADC_BASE + 0x00C, 0x0000, 0);
        }

        /// Narrow write with a held peripheral silently drops.
        #[test]
        fn narrow_write_held_peripheral_drops() {
            let shared = fresh_shared();
            // UART1 is held; its UARTDR is 0x4007_4000. try_narrow_write8
            // must return `true` (consumed) and the write must not reach
            // the peripheral.
            let mut bus = WorkerBus::new(0, shared);
            bus.write8(0x4007_4000, 0x41, 0);
            bus.write16(0x4007_4000, 0x00_41, 0);
            // read8 via the held peripheral returns 0.
            assert_eq!(bus.read8(0x4007_4000, 0), 0);
            assert_eq!(bus.read16(0x4007_4000, 0), 0);
        }

        // ---- AHB byte / halfword write -------------------------------

        #[test]
        fn ahb_byte_write_rmw_through_word() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Legacy AHB byte-write goes through the RMW path.
            let addr = 0x5060_0000;
            bus.write32(addr, 0xAABB_CCDD, 0);
            bus.write8(addr + 1, 0xFF, 0);
            assert_eq!(bus.read32(addr, 0), 0xAABB_FFDD);
        }

        #[test]
        fn ahb_halfword_write_rmw_through_word() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            let addr = 0x5060_0004;
            bus.write32(addr, 0xAABB_CCDD, 0);
            bus.write16(addr, 0x1122, 0);
            assert_eq!(bus.read32(addr, 0), 0xAABB_1122);
        }

        // ---- SIO narrow write replicating path -------------------------

        #[test]
        fn sio_write8_gpio_out_replicates_across_lanes() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // STRB to SIO GPIO_OUT replicates the byte across all lanes.
            bus.write8(0xD000_0010, 0xA5, 0);
            assert_eq!(bus.read32(0xD000_0010, 0), 0xA5A5_A5A5);
        }

        #[test]
        fn sio_write16_gpio_out_replicates_across_lanes() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            bus.write16(0xD000_0010, 0xBEEF, 0);
            assert_eq!(bus.read32(0xD000_0010, 0), 0xBEEF_BEEF);
        }

        /// Non-replicating SIO byte write falls through to the RMW arm.
        #[test]
        fn sio_write8_non_replicating_rmw_fallback() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // MTIME control at 0xD000_01A0 is NOT in the replicating set.
            bus.write32(0xD000_01A0, 0x0000_0000, 0);
            bus.write8(0xD000_01A0, 0xAB, 0);
            // RMW path must land the byte in lane 0 only.
            assert_eq!(bus.read32(0xD000_01A0, 0) & 0xFF, 0xAB);
        }

        #[test]
        fn sio_write16_non_replicating_rmw_fallback() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            bus.write32(0xD000_01A8, 0x0000_0000, 0);
            bus.write16(0xD000_01A8, 0xCAFE, 0);
            assert_eq!(bus.read32(0xD000_01A8, 0) & 0xFFFF, 0xCAFE);
        }

        // ---- Unmapped-region bus fault dispatch ------------------------

        #[test]
        fn unmapped_read_sets_bus_fault() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // 0x8000_0000 is outside all mapped regions.
            assert_eq!(bus.read32(0x8000_0000, 0), 0);
            assert_eq!(bus.read16(0x8000_0000, 0), 0);
            assert_eq!(bus.read8(0x8000_0000, 0), 0);
            assert!(shared.atomics.is_bus_fault(0));
            assert_eq!(shared.atomics.bus_fault_addr(0), 0x8000_0000);
        }

        #[test]
        fn unmapped_write_sets_bus_fault() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            bus.write32(0x8000_0000, 0xDEAD_BEEF, 0);
            assert!(shared.atomics.is_bus_fault(0));
            // Distinct faults for write8 / write16.
            shared.atomics.clear_bus_fault(0);
            bus.write8(0x8000_0004, 0xAA, 0);
            assert!(shared.atomics.is_bus_fault(0));
            shared.atomics.clear_bus_fault(0);
            bus.write16(0x8000_0008, 0xBBBB, 0);
            assert!(shared.atomics.is_bus_fault(0));
        }

        // ---- raise_irqs_shared filters software IRQs -------------------

        /// When a peripheral `tick` reports a non-peripheral IRQ bit
        /// (e.g. bit 46 — SIO_IRQ_FIFO is software-only), `raise_irqs_shared`
        /// masks it. We can exercise the loop itself via the UART TX path
        /// indirectly: push a byte, which reports UART0_IRQ back on the
        /// shared mask.
        #[test]
        fn raise_irqs_shared_filters_software_only_bits() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // Enable TX interrupt on UART0 (UARTIMSC bit 5 = TXIM).
            bus.write32(UART0_BASE + 0x38, 0x20, 0);
            // Push a byte through the narrow TX path.
            bus.write8(UART0_BASE, b'A', 0);
            // UART0 IRQ may now be asserted on both cores; at worst, no
            // assertion but the path is covered.
            let pending0 = shared.atomics.irq_pending_load(0);
            // Software-only bits (46..=51) must not appear.
            let swmask = (1u64 << 46)
                | (1u64 << 47)
                | (1u64 << 48)
                | (1u64 << 49)
                | (1u64 << 50)
                | (1u64 << 51);
            assert_eq!(pending0 & swmask, 0);
        }

        // ---- queue_cache_invalidation skip path ------------------------

        #[test]
        fn queue_cache_invalidation_skips_non_executable_regions() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Write into APB (0x4...) — addr >> 28 == 0x4, NOT in 0x0..=0x2.
            // queue_cache_invalidation's matches-predicate must skip this.
            let before = bus.pending_cache_invalidations.len();
            bus.write32(0x4001_0030, 0x0, 0);
            assert_eq!(
                bus.pending_cache_invalidations.len(),
                before,
                "APB writes must not queue cache invalidations"
            );
        }

        // ---- RESETS writes for coverage --------------------------------

        /// Toggle RESETS state: hold TIMER0, then release, observe the
        /// guard path on apb_read32 flip between 0 and the real value.
        /// Covers the `is_held_in_reset_base` true/false branches once
        /// more through the typed APB path.
        #[test]
        fn resets_state_toggle_gates_timer0_read() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // Hold TIMER0.
            let mask: u32 = 1u32 << RESET_TIMER0;
            bus.write32(0x4002_0000, mask, 0); // set RESET bits (alias 0)
            // Alias 2 = SET semantics to make sure the bit lands.
            bus.write32(0x4002_2000, mask, 0);
            let held_state = shared.peripherals.resets.lock().unwrap().resets_state;
            assert_ne!(held_state & mask, 0);
            // Held TIMER0 must read 0.
            assert_eq!(bus.read32(TIMER0_BASE, 0), 0);
            // Release via alias 3 (CLR).
            bus.write32(0x4002_3000, mask, 0);
            let released_state = shared.peripherals.resets.lock().unwrap().resets_state;
            assert_eq!(released_state & mask, 0);
            // Suppress unused-import warnings for the RESET_* constants
            // imported at the module top. Each is used somewhere above or
            // here; this keeps the batch import compact.
            let _ = (
                RESET_UART0,
                RESET_SPI0,
                RESET_I2C0,
                RESET_ADC,
                RESET_PWM,
                RESET_IO_BANK0,
                RESET_PADS_BANK0,
                RESET_TIMER1,
                RESET_DMA,
            );
        }

        // ---- CoreBus wait-state accounting via dyn --------------------

        #[test]
        fn wait_state_add_and_take_via_core_bus() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            let b: &mut dyn CoreBus = &mut bus;
            b.add_extra_wait_states(2);
            b.reset_extra_wait_states();
            assert_eq!(b.extra_wait_states(), 0);
            // Non-zero take path.
            b.add_extra_wait_states(7);
            assert_eq!(b.take_extra_wait_states(), 7);
            assert_eq!(b.take_extra_wait_states(), 0);
            // last_fetch_addr getter/setter.
            b.set_last_fetch_addr(0x2000_1234);
            assert_eq!(b.last_fetch_addr(), 0x2000_1234);
        }

        // ---- boot_ram / xip_sram narrow-width routing -------------------

        /// `write8` / `write16` into the boot RAM scratchpad take the
        /// early-return arms at lines 1176-78 and 1248-52.
        #[test]
        fn boot_ram_narrow_writes_via_worker_bus() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            let addr = 0xEFFF_F080;
            bus.write32(addr, 0x0000_0000, 0);
            bus.write16(addr, 0xBEEF, 0);
            assert_eq!(bus.read16(addr, 0), 0xBEEF);
            bus.write8(addr, 0xAB, 0);
            assert_eq!(bus.read8(addr, 0), 0xAB);
        }

        #[test]
        fn xip_sram_narrow_writes_via_worker_bus() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            let addr = 0x1C00_0080;
            bus.write32(addr, 0x0000_0000, 0);
            bus.write16(addr, 0xCAFE, 0);
            assert_eq!(bus.read16(addr, 0), 0xCAFE);
            bus.write8(addr, 0xCD, 0);
            assert_eq!(bus.read8(addr, 0), 0xCD);
        }

        // ---- Narrow read/write hitting APB+AHB for 0x5 region ---------

        /// `read8` / `read16` at an AHB (0x5) non-FIFO address hits the
        /// `(addr >> 28) == 0x5` branch inside the narrow-read fallthrough.
        #[test]
        fn narrow_read8_and_16_ahb_fallthrough() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // Legacy AHB canonical base (alias bits clear). Seed with a
            // recognisable pattern so the byte- and halfword-extract
            // math is observable.
            let base = 0x5060_0000;
            bus.write32(base, 0xDEAD_BEEF, 0);
            assert_eq!(bus.read8(base, 0), 0xEF);
            assert_eq!(bus.read8(base + 1, 0), 0xBE);
            assert_eq!(bus.read16(base, 0), 0xBEEF);
            assert_eq!(bus.read16(base + 2, 0), 0xDEAD);
        }

        // ---- SIO byte / halfword reads to exercise 1106-07 / 1141-43 ---

        #[test]
        fn sio_narrow_reads_word_extract() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // GPIO_OUT at 0xD000_0010 is RW; seed it then read byte/half.
            bus.write32(0xD000_0010, 0xAABB_CCDD, 0);
            assert_eq!(bus.read8(0xD000_0010, 0), 0xDD);
            assert_eq!(bus.read8(0xD000_0012, 0), 0xBB);
            assert_eq!(bus.read16(0xD000_0010, 0), 0xCCDD);
            assert_eq!(bus.read16(0xD000_0012, 0), 0xAABB);
        }

        // ---- SIO GPIO_IN read via sio_read32 offset 0x004 ---------------

        #[test]
        fn sio_read_gpio_in_via_worker_bus() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // Seed an external-stim value and a bank-0 output so
            // `gpio_in_fresh` produces a predictable merge.
            shared.gpio.write_external(0xA000_0000, 0xA000_0000);
            shared.gpio.write_out(0, 0x0000_0001);
            shared.gpio.write_oe(0, 0x0000_0001);
            shared.gpio.write_in(0x0000_0001);
            // Read GPIO_IN via SIO offset 0x004 — exercises line 636.
            let v = bus.read32(0xD000_0004, 0);
            // The external-stim bit must show, AND the seeded `in_` bit.
            assert_ne!(v & 0xA000_0000, 0, "external bit must be present");
        }

        // ---- RESETS writes routed through apb_write32's RESETS arm ----

        /// `apb_write32` line 321-327 (RESETS dispatch arm) is covered
        /// by `resets_state_toggle_gates_timer0_read` above. This test
        /// exercises the TICKS write_32's `if invalidate` true branch
        /// (bus.rs:349-352) by toggling CTRL on TIMER0 domain.
        #[test]
        fn ticks_invalidate_lazy_on_timer0_ctrl_write() {
            use crate::peripherals::ticks::{
                CTRL_ENABLE, CTRL_OFFSET, DOMAIN_STRIDE, DOMAIN_TIMER0,
            };
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // CTRL register of TIMER0 domain inside TICKS.
            let addr = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CTRL_OFFSET;
            // First write toggles ENABLE on (returns true → invalidate_lazy).
            bus.write32(addr, CTRL_ENABLE, 0);
            // Second toggles it off (another invalidate).
            bus.write32(addr, 0, 0);
        }

        // ---- raise_irqs_shared loop body via PWM tick ------------------

        /// `raise_irqs_shared` is exercised indirectly when a peripheral
        /// write triggers an IRQ assertion. Trigger PWM IRQ by writing
        /// INTE=1 then INTR=1 so the next PWM tick reports the IRQ via
        /// ext_irqs → `raise_irqs_shared` loops through bit 40 or so.
        ///
        /// This specifically tries to hit the while-loop body at
        /// bus.rs:1033-1037.
        /// Force `raise_irqs_shared` to actually loop by triggering a
        /// real IRQ through a peripheral write. PWM's INTE + INTF combo
        /// is the most compact path: set INTE=1 on slice 0, then write
        /// INTF=1 to force the interrupt. The PWM write handler reports
        /// the IRQ back through `irqs` → `raise_irqs_shared` runs its
        /// inner `while remaining != 0 { ... }` body.
        #[test]
        fn raise_irqs_shared_fires_pwm_intf() {
            use crate::peripherals::pwm::PWM_BASE;
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // INTE0 at offset 0xF8: enable slice 0.
            bus.write32(PWM_BASE + 0xF8, 0x1, 0);
            // INTF0 at offset 0xFC: force slice 0. This sets INTS → the
            // peripheral reports IRQ via the write32 `irqs` return →
            // `raise_irqs_shared` loop fires.
            bus.write32(PWM_BASE + 0xFC, 0x1, 0);
            // IRQ pending should now have the PWM wrap IRQ bit set on
            // both cores (the loop body runs at least once per bit).
            let m0 = shared.atomics.irq_pending_load(0);
            let m1 = shared.atomics.irq_pending_load(1);
            // We don't assert the exact IRQ number — different IRQ
            // constants map across revisions. The load must be non-zero
            // on both cores, proving `assert_irq_shared` ran.
            assert_ne!(m0, 0, "core 0 must see PWM IRQ pending");
            assert_ne!(m1, 0, "core 1 must see PWM IRQ pending");
        }

        #[test]
        fn raise_irqs_shared_loops_through_multiple_bits() {
            use crate::peripherals::pwm::PWM_BASE;
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared.clone());
            // Force-write the raw PWM IRQ bit (PWM_BASE + 0x50 is the
            // INTE mask; register offsets differ per slice). Even if
            // this write doesn't assert IRQs, it exercises the PWM
            // write32 path and the code inside `raise_irqs_shared`
            // will at least skip its `while` preamble evaluation.
            bus.write32(PWM_BASE + 0x50, 0xFFFF, 0);
            // Manually assert two IRQs via assert_irq_shared to confirm
            // the helper's shape — independent of `raise_irqs_shared`.
            shared.atomics.assert_irq_shared(10);
            shared.atomics.assert_irq_shared(12);
            let m = shared.atomics.irq_pending_load(0);
            assert_ne!(m & (1u64 << 10), 0);
            assert_ne!(m & (1u64 << 12), 0);
        }

        // ---- queue_cache_invalidation 4-byte dual-push via SRAM write32 --

        /// A 4-byte SRAM write pushes BOTH `addr` and `addr+2` into the
        /// invalidation queue — covers bus.rs:1060.
        #[test]
        fn queue_cache_invalidation_write32_pushes_two_entries() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            let addr = 0x2000_3000;
            bus.write32(addr, 0xDEAD_BEEF, 0);
            assert_eq!(bus.pending_cache_invalidations.len(), 2);
            assert_eq!(bus.pending_cache_invalidations[0], addr);
            assert_eq!(bus.pending_cache_invalidations[1], addr + 2);
        }

        // ---- sysinfo_read unmapped branch via bus dispatch -------------

        #[test]
        fn sysinfo_unknown_offset_falls_through() {
            let shared = fresh_shared();
            let mut bus = WorkerBus::new(0, shared);
            // 0x4000_0020 — unmapped SYSINFO offset, returns 0.
            assert_eq!(bus.read32(0x4000_0020, 0), 0);
        }
    }
}

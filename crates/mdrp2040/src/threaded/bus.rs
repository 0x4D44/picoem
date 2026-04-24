//! `WorkerBus` — the per-CPU-thread `CoreBus` implementation for the
//! RP2040 threaded runtime.
//!
//! Stage 3b.3 (dual-execution HLD V1 §6.4): MMIO peripheral routing
//! landed. Every address region firmware can touch has a real
//! destination:
//!
//! - **RAM / ROM** (`0x0000_0000..0x0000_4000`, `0x2000_0000..0x2004_2000`
//!   and aliases) — `SharedMemory` atomic-word storage.
//! - **SIO** (`0xD000_0000`) — hot path: FIFOs, spinlocks, GPIO atomics,
//!   CPUID, per-core divider/interpolator. See [`WorkerBus::sio_read32`]
//!   / [`WorkerBus::sio_write32`].
//! - **APB** (`0x4`) — clocks, PLL, XOSC, ROSC, RESETS, IO_BANK0,
//!   PADS_BANK0 through `SharedState.peripherals` Mutex. Reset-gated
//!   peripherals short-circuit to 0. Everything else falls through to
//!   the `peripherals.legacy` HashMap (matches serial behaviour).
//! - **AHB** (`0x5`) — DMA (stub via legacy HashMap) + PIO via
//!   `shared.pio` typed `Mutex<Vec<PioCommand>>` command queue + a
//!   coordinator-refreshed register snapshot for reads.
//! - **PPB** (`0xE`) — per-worker local (same as serial inherent `Bus`).
//! - **XIP / XIP_CTRL / SSI** (`0x1`) — legacy HashMap stub.
//!
//! ## Drops vs serial path
//!
//! - **Bank contention** (HLD §6.4 step 3) — threaded path has no
//!   per-bank touched bitmap; contention cycles are virtual anyway.
//! - **Interp cross-core coherency** (HLD §6.4 step 5) — silicon
//!   doesn't coherence it either; each worker owns its own divider +
//!   interpolator snapshot.
//! - **TIMER/UART/SPI/I2C/ADC/PWM typed storage** — Stage 3b.3 routes
//!   these through the `legacy` HashMap; Stage 3b.4 will refactor
//!   individual hot-read paths (TIMER especially) to
//!   coordinator-refreshed snapshots. Serial RAW semantics are
//!   preserved via the alias-aware HashMap update.
//!
//! ## PPB and NVIC ownership
//!
//! Per-core PPB and NVIC live locally on the `WorkerBus`. Cross-core
//! IRQ delivery flows through `SharedState.atomics.irq_pending[core]`;
//! `WorkerBus::drain_cross_core_irqs` merges those bits into the local
//! NVIC at quantum / step entry. Stage 3b.4 calls this at the start of
//! each `CortexM0Plus::step`.

use std::sync::Arc;

use mdpicoem_common::clocks::ClockTree;

use crate::bus::ppb::Ppb;
use crate::core::Nvic;
use crate::core::bus_trait::CoreBus;
use crate::threaded::pio::PioCommand;
use crate::threaded::{SharedState, memory};

// =======================================================================
// Address region constants (RP2040 datasheet §2.2).
// =======================================================================

// APB peripheral bases we route by match in `apb_read32` / `apb_write32`.
const SYSINFO_BASE: u32 = 0x4000_0000;
const CLOCKS_BASE: u32 = 0x4000_8000;
const RESETS_BASE: u32 = 0x4000_C000;
const IO_BANK0_BASE: u32 = 0x4001_4000;
const PADS_BANK0_BASE: u32 = 0x4001_C000;
const XOSC_BASE: u32 = 0x4002_4000;
const PLL_SYS_BASE: u32 = 0x4002_8000;
const PLL_USB_BASE: u32 = 0x4002_C000;
const ROSC_BASE: u32 = 0x4006_0000;

// TIMER block — typed state (TIMELR read latches TIMEHR, so plain
// HashMap storage cannot model it). Routed through
// `SharedState.peripherals.timer`.
const TIMER_BASE: u32 = 0x4005_4000;

// PIO AHB bases (RP2040 has 2 PIO blocks, vs RP2350's 3).
const PIO0_BASE: u32 = 0x5020_0000;
const PIO1_BASE: u32 = 0x5030_0000;

// XIP_CTRL / SSI — region 0x1, stub storage on the legacy HashMap.
const XIP_CTRL_BASE: u32 = 0x1400_0000;
const SSI_BASE: u32 = 0x1800_0000;

// =======================================================================
// WorkerBus
// =======================================================================

/// Per-CPU-thread bus view. Carries a clone of [`SharedState`] plus the
/// per-instruction accounting fields that in the serial path live
/// directly on `Bus`.
pub struct WorkerBus {
    /// Core ID this worker drives (0 or 1).
    core_id: usize,
    /// Shared state bundle (cheap Arc clone per worker).
    shared: Arc<SharedState>,
    /// Per-core PPB. Only `[core_id]` is actually touched by this
    /// worker's core; the other slot is an inert placeholder so the
    /// trait's `&ppb[core]` indexing maps 1:1 with the serial `Bus`.
    ppb: [Ppb; 2],
    /// Per-core NVIC. Same layout + placeholder convention as `ppb`.
    nvic: [Nvic; 2],
    /// Sticky bus-fault flag.
    bus_fault: bool,
    /// Address that raised the most recent bus fault.
    bus_fault_addr: u32,
    /// PC of the currently-executing instruction.
    active_pc: u32,
    /// Per-worker integer divider snapshot (HLD §6.4 point 4 — divider
    /// is per-core-local; no cross-core coherency).
    divider: DividerLocal,
    /// Per-worker interpolator snapshot (HLD §6.4 point 5 — interp
    /// cross-core coherency is dropped).
    interp: [u32; 32],
}

impl WorkerBus {
    /// Construct a new `WorkerBus` for `core_id` with the given
    /// [`SharedState`] bundle.
    ///
    /// Seeds this worker's PPB slot from
    /// [`SharedState::take_initial_ppb`] so pre-run pokes to VTOR,
    /// ICSR (PENDSV/PENDST), and SHPR priorities are carried through
    /// the serial → threaded handoff. The peer core's slot is left at
    /// the default (never read by this worker — `ppb[peer]` is an
    /// inert placeholder kept for indexing parity with the serial
    /// `Bus`).
    pub fn new(shared: Arc<SharedState>, core_id: usize) -> Self {
        debug_assert!(core_id < 2, "core_id must be 0 or 1");
        let mut ppb = [Ppb::new(), Ppb::new()];
        ppb[core_id] = shared.take_initial_ppb(core_id);
        Self {
            core_id,
            shared,
            ppb,
            nvic: [Nvic::new(), Nvic::new()],
            bus_fault: false,
            bus_fault_addr: 0,
            active_pc: 0,
            divider: DividerLocal::default(),
            interp: [0u32; 32],
        }
    }

    /// Access the shared state bundle (used by the worker loop in
    /// Stage 3b.4 and by diagnostics).
    #[allow(dead_code)]
    pub(crate) fn shared(&self) -> &Arc<SharedState> {
        &self.shared
    }

    /// Drain the cross-core IRQ pending mask published by the
    /// coordinator / peer worker and merge it into this core's local
    /// NVIC.
    ///
    /// Called at step entry by the Stage 3b.4 worker loop — each
    /// `CortexM0Plus::step` call starts with a drain so a FIFO push
    /// from the peer (which sets `atomics.irq_pending[self]`) becomes
    /// visible to the local NVIC before the instruction dispatch.
    pub fn drain_cross_core_irqs(&mut self) {
        // `CoreAtomics` is a single `Arc` on `SharedState`; its
        // `irq_pending` array is indexed by this core's id. Swap-to-zero
        // with AcqRel so a peer push becomes visible here before the
        // NVIC merge.
        let bits = self.shared.atomics.take_irq_pending(self.core_id);
        if bits != 0 {
            self.nvic[self.core_id].pending |= bits;
        }
    }

    /// Lock-free snapshot of the coordinator-published master cycle
    /// counter. Taken *before* any `peripherals.*` lock is acquired so a
    /// concurrent coordinator `fetch_add` never serializes with CPU
    /// reads.
    #[inline]
    fn master_cycle(&self) -> u64 {
        use std::sync::atomic::Ordering;
        self.shared.master_cycle.load(Ordering::Acquire)
    }

    // ----------------------------------------------------------------
    // Region classifiers
    // ----------------------------------------------------------------

    /// True when `addr` is backed by the `SharedMemory` atomic-word
    /// store (ROM or any SRAM alias). Tightened per Stage 3b.2
    /// reviewer NIT — the previous classifier admitted the full
    /// `0x2???_????` region; real silicon only has 264 KB at
    /// `0x2000_0000..0x2004_2000` with three aliases at
    /// `0x2100_0000` / `0x2200_0000` / `0x2300_0000` (same 264 KB).
    /// Out-of-bounds accesses now fall through to MMIO routing and
    /// eventually the legacy HashMap / stub — firmware bus faults
    /// surface distinctly from SRAM reads-returning-zero.
    fn is_ram_or_rom_addr(addr: u32) -> bool {
        // Boot ROM: 0x0000_0000..0x0000_4000 (16 KB).
        if addr < memory::ROM_SIZE {
            return true;
        }
        // SRAM + 3 aliases. Each window is 264 KB wide; everything else
        // in 0x2??_???? is unmapped.
        let top_nibble = addr >> 28;
        if top_nibble != 0x2 {
            return false;
        }
        let alias = (addr >> 24) & 0xF; // 0x0 / 0x1 / 0x2 / 0x3
        if alias > 3 {
            return false;
        }
        let offset = addr & 0x00FF_FFFF;
        offset < memory::SRAM_SIZE
    }

    // ----------------------------------------------------------------
    // SIO dispatch (base 0xD000_0000)
    // ----------------------------------------------------------------
    //
    // See `bus/sio.rs` for the ground-truth offset layout. Cross-core
    // surface (FIFO / spinlocks / GPIO) routes through `SharedState`;
    // per-core divider + interpolator are worker-local.

    fn sio_read32(&mut self, addr: u32) -> u32 {
        use std::sync::atomic::Ordering;
        let offset = addr & 0xFFF;
        match offset {
            // CPUID returns this core's id.
            0x000 => self.core_id as u32,
            // GPIO_IN: the coordinator merges SIO + PIO outputs into
            // `gpio_out`/`gpio_oe` each quantum. We compute the effective
            // pin view as `out & oe` then apply the external-GPIO
            // override mask so harness pokes to
            // `Bus::external_gpio_in_override` (e.g. PicoGUS ISA
            // waveforms) survive the merge — parity with the serial
            // `Emulator::update_gpio` final override step.
            0x004 => {
                let merged = self.shared.gpio_out.load(Ordering::Acquire)
                    & self.shared.gpio_oe.load(Ordering::Acquire);
                let ext_mask = self.shared.external_gpio_in_mask.load(Ordering::Acquire);
                if ext_mask == 0 {
                    merged
                } else {
                    let ext_val = self.shared.external_gpio_in_override.load(Ordering::Acquire);
                    (merged & !ext_mask) | (ext_val & ext_mask)
                }
            }
            // GPIO_OUT / SET / CLR / XOR all read the same register.
            0x010 | 0x014 | 0x018 | 0x01C => self.shared.gpio_out.load(Ordering::Acquire),
            // GPIO_OE / SET / CLR / XOR all read the same register.
            0x020 | 0x024 | 0x028 | 0x02C => self.shared.gpio_oe.load(Ordering::Acquire),
            // FIFO_ST — vld / rdy / wof / roe. WOF/ROE live on the
            // shared `CoreAtomics.fifo_wof` / `fifo_roe` sticky flags,
            // set by FIFO_WR on push-to-full and FIFO_RD on pop-from-
            // empty respectively (serial `Sio::fifo_st_read` shape).
            0x050 => {
                let rx_fifo = if self.core_id == 0 {
                    &self.shared.sio_fifo_1_to_0
                } else {
                    &self.shared.sio_fifo_0_to_1
                };
                let tx_fifo = if self.core_id == 0 {
                    &self.shared.sio_fifo_0_to_1
                } else {
                    &self.shared.sio_fifo_1_to_0
                };
                let vld = if !rx_fifo.is_empty() { 1u32 } else { 0 };
                let rdy = if !tx_fifo.is_full() { 2u32 } else { 0 };
                let wof = if self.shared.atomics.fifo_wof(self.core_id) {
                    1u32 << 2
                } else {
                    0
                };
                let roe = if self.shared.atomics.fifo_roe(self.core_id) {
                    1u32 << 3
                } else {
                    0
                };
                vld | rdy | wof | roe
            }
            // FIFO_RD: pop from this core's RX queue. An empty pop
            // latches FIFO_ST.ROE on this core (matches serial
            // `Sio::fifo_rd`).
            0x058 => {
                let rx_fifo = if self.core_id == 0 {
                    &self.shared.sio_fifo_1_to_0
                } else {
                    &self.shared.sio_fifo_0_to_1
                };
                match rx_fifo.try_pop() {
                    Some(v) => v,
                    None => {
                        self.shared.atomics.set_fifo_roe(self.core_id);
                        0
                    }
                }
            }
            // Integer divider reads (per-core local).
            0x060 | 0x068 => self.divider.dividend,
            0x064 | 0x06C => self.divider.divisor,
            0x070 => self.divider.quotient,
            0x074 => self.divider.remainder,
            0x078 => {
                // CSR: bit 0 READY always 1; bit 1 DIRTY if results
                // are unread. Same shape as serial `Sio::read32`.
                let ready = 1u32;
                let dirty = if self.divider.dirty { 2 } else { 0 };
                ready | dirty
            }
            // Interpolators 0x080..0x0FC — per-worker local.
            0x080..=0x0FC => {
                let idx = ((offset - 0x080) >> 2) as usize;
                if idx < 32 {
                    self.interp[idx]
                } else {
                    0
                }
            }
            // Spinlock bank status at 0x05C.
            0x05C => {
                // Bitmap of currently-held spinlocks. Build from the
                // per-lock AtomicU32 storage on the fly.
                let mut bits = 0u32;
                for i in 0..32usize {
                    if self.shared.spinlocks[i].load(Ordering::Acquire) != 0 {
                        bits |= 1u32 << i;
                    }
                }
                bits
            }
            // Spinlocks 0x100..0x17F — test-and-set acquire.
            0x100..=0x17F => {
                let n = ((offset - 0x100) >> 2) as usize;
                debug_assert!(n < 32);
                let mask = 1u32 << n;
                // CAS 0 → mask; success returns the mask, failure 0.
                match self.shared.spinlocks[n].compare_exchange(
                    0,
                    mask,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => mask,
                    Err(_) => 0,
                }
            }
            _ => 0,
        }
    }

    fn sio_write32(&mut self, addr: u32, val: u32) {
        use std::sync::atomic::Ordering;
        const PIN_MASK: u32 = 0x3FFF_FFFF;
        let offset = addr & 0xFFF;
        match offset {
            // GPIO_OUT plain store / SET / CLR / XOR (RP2040 4-byte spacing).
            0x010 => self.shared.gpio_out.store(val & PIN_MASK, Ordering::Release),
            0x014 => {
                self.shared.gpio_out.fetch_or(val & PIN_MASK, Ordering::AcqRel);
            }
            0x018 => {
                self.shared
                    .gpio_out
                    .fetch_and(!(val & PIN_MASK), Ordering::AcqRel);
            }
            0x01C => {
                self.shared.gpio_out.fetch_xor(val & PIN_MASK, Ordering::AcqRel);
            }
            // GPIO_OE plain / SET / CLR / XOR.
            0x020 => self.shared.gpio_oe.store(val & PIN_MASK, Ordering::Release),
            0x024 => {
                self.shared.gpio_oe.fetch_or(val & PIN_MASK, Ordering::AcqRel);
            }
            0x028 => {
                self.shared.gpio_oe.fetch_and(!(val & PIN_MASK), Ordering::AcqRel);
            }
            0x02C => {
                self.shared.gpio_oe.fetch_xor(val & PIN_MASK, Ordering::AcqRel);
            }
            // FIFO_ST — W1C for WOF/ROE stickies (bits 2 and 3). Matches
            // serial `Sio::fifo_st_write`.
            0x050 => {
                if val & (1 << 2) != 0 {
                    self.shared.atomics.clear_fifo_wof(self.core_id);
                }
                if val & (1 << 3) != 0 {
                    self.shared.atomics.clear_fifo_roe(self.core_id);
                }
            }
            // FIFO_WR: push onto peer RX queue + raise cross-core IRQ
            // bit so the peer NVIC wakes (FIFO IRQs are SIO_PROC0/1_IRQ
            // on RP2040 = IRQ 15 / 16; Stage 3b.3 just publishes the
            // bit, Stage 3b.4 wires the actual IRQ lines). A push into
            // a full FIFO latches FIFO_ST.WOF on the sender core.
            0x054 => {
                let other = 1 - self.core_id;
                let tx_fifo = if self.core_id == 0 {
                    &self.shared.sio_fifo_0_to_1
                } else {
                    &self.shared.sio_fifo_1_to_0
                };
                if tx_fifo.try_push(val) {
                    // Wake peer via event_flag (WFE/SEV parity) + IRQ
                    // pending for NVIC dispatch.
                    self.shared.atomics.set_event_flag(other);
                    // SIO_PROC0_IRQ = IRQ 15, SIO_PROC1_IRQ = IRQ 16.
                    let irq = if other == 0 { 15 } else { 16 };
                    self.shared.atomics.assert_irq(other, irq as u32);
                } else {
                    // Push into full FIFO latches WOF on the sender core
                    // (matches serial `Sio::fifo_wr` on queue-full).
                    self.shared.atomics.set_fifo_wof(self.core_id);
                }
            }
            // Divider writes — per-core local.
            0x060 => {
                self.divider.dividend = val;
                self.divider.signed = false;
            }
            0x064 => {
                self.divider.divisor = val;
                self.divider.signed = false;
                self.divider.compute();
            }
            0x068 => {
                self.divider.dividend = val;
                self.divider.signed = true;
            }
            0x06C => {
                self.divider.divisor = val;
                self.divider.signed = true;
                self.divider.compute();
            }
            0x070 => {
                self.divider.quotient = val;
                self.divider.dirty = true;
            }
            0x074 => {
                self.divider.remainder = val;
                self.divider.dirty = true;
            }
            // Interpolators — plain storage per-worker.
            0x080..=0x0FC => {
                let idx = ((offset - 0x080) >> 2) as usize;
                if idx < 32 {
                    self.interp[idx] = val;
                }
            }
            // Spinlock release — any write clears.
            0x100..=0x17F => {
                let n = ((offset - 0x100) >> 2) as usize;
                debug_assert!(n < 32);
                self.shared.spinlocks[n].store(0, Ordering::Release);
            }
            _ => {}
        }
    }

    // ----------------------------------------------------------------
    // APB dispatch (region 0x4)
    // ----------------------------------------------------------------

    fn apb_read32(&mut self, addr: u32) -> u32 {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        let mc = self.master_cycle();

        // RESETS guard — held peripherals read as 0 (parity with
        // serial `Bus::peripheral_read32`).
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
            SYSINFO_BASE => sysinfo_read(offset),
            CLOCKS_BASE => self.shared.peripherals.clocks.lock().unwrap().clocks_read(offset),
            XOSC_BASE => self.shared.peripherals.clocks.lock().unwrap().xosc_read(offset),
            ROSC_BASE => self.shared.peripherals.clocks.lock().unwrap().rosc_read(offset),
            PLL_SYS_BASE => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_sys_read_at(offset, mc),
            PLL_USB_BASE => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_usb_read_at(offset, mc),
            RESETS_BASE => self.shared.peripherals.resets.lock().unwrap().read(offset),
            IO_BANK0_BASE => self
                .shared
                .peripherals
                .io
                .lock()
                .unwrap()
                .io_bank0
                .read32(offset),
            PADS_BANK0_BASE => self
                .shared
                .peripherals
                .io
                .lock()
                .unwrap()
                .pads_bank0
                .read32(offset),
            TIMER_BASE => {
                // Snapshot sys_hz from the cached ClockTree under the
                // clocks lock, then route through the typed TimerState.
                // TIMELR read latches TIMEHR — HashMap storage cannot
                // model that; hence the typed fallback.
                let sys_hz = self
                    .shared
                    .peripherals
                    .clocks
                    .lock()
                    .unwrap()
                    .clock_tree
                    .sys_clk_hz;
                self.shared
                    .peripherals
                    .timer
                    .lock()
                    .unwrap()
                    .read32(offset, mc, sys_hz)
            }
            _ => self.legacy_read(canonical),
        }
    }

    fn apb_write32(&mut self, addr: u32, val: u32) {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        let alias = (addr >> 12) & 3;
        let mc = self.master_cycle();

        // RESETS guard — held peripherals drop writes.
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
            SYSINFO_BASE => {} // read-only
            CLOCKS_BASE => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .clocks_write(offset, val, alias),
            XOSC_BASE => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .xosc_write(offset, val, alias),
            ROSC_BASE => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .rosc_write(offset, val, alias),
            PLL_SYS_BASE => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_sys_write_at(offset, val, alias, mc),
            PLL_USB_BASE => self
                .shared
                .peripherals
                .clocks
                .lock()
                .unwrap()
                .pll_usb_write_at(offset, val, alias, mc),
            RESETS_BASE => self
                .shared
                .peripherals
                .resets
                .lock()
                .unwrap()
                .write(offset, val, alias),
            IO_BANK0_BASE => self
                .shared
                .peripherals
                .io
                .lock()
                .unwrap()
                .io_bank0
                .write32(offset, val, alias),
            PADS_BANK0_BASE => self
                .shared
                .peripherals
                .io
                .lock()
                .unwrap()
                .pads_bank0
                .write32(offset, val, alias),
            TIMER_BASE => {
                let sys_hz = self
                    .shared
                    .peripherals
                    .clocks
                    .lock()
                    .unwrap()
                    .clock_tree
                    .sys_clk_hz;
                self.shared
                    .peripherals
                    .timer
                    .lock()
                    .unwrap()
                    .write32(offset, val, alias, mc, sys_hz);
            }
            _ => self.legacy_write(canonical, val, alias),
        }
    }

    // ----------------------------------------------------------------
    // AHB dispatch (region 0x5) — DMA + PIO
    // ----------------------------------------------------------------

    fn ahb_read32(&mut self, addr: u32) -> u32 {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        // PIO blocks are 0x10_0000 bytes apart; mask the block bit out
        // to get the per-block base.
        let block = match addr & 0xFFF0_0000 {
            x if x == PIO0_BASE => Some(0usize),
            x if x == PIO1_BASE => Some(1usize),
            _ => None,
        };
        if let Some(block) = block {
            // CTRL register synthesises SM_ENABLE from the coordinator-
            // published atomic; other offsets return the snapshot.
            if offset == 0x000 {
                return self.shared.pio.sm_enabled(block) as u32;
            }
            return self.shared.pio.snapshot_read32(block, offset);
        }
        // DMA, XIP_CTRL, SSI all fall through to legacy HashMap.
        match base {
            _ => self.legacy_read(canonical),
        }
    }

    fn ahb_write32(&mut self, addr: u32, val: u32) {
        let canonical = addr & !0x3000;
        let offset = canonical & 0x0000_0FFF;
        let alias = (addr >> 12) & 3;
        let block = match addr & 0xFFF0_0000 {
            x if x == PIO0_BASE => Some(0u8),
            x if x == PIO1_BASE => Some(1u8),
            _ => None,
        };
        if let Some(block) = block {
            // Enqueue a PioCommand for the coordinator to apply against
            // the real `PioBlock`. Matches mdrp2350's encoder.
            let off12 = offset as u16;
            let cmd = match off12 {
                0x000 => PioCommand::WriteCtrl {
                    block,
                    val,
                    alias: alias as u8,
                },
                // INSTR_MEM0..31 live at 0x048..0x0C4 on RP2040.
                0x048..=0x0C4 => {
                    let mem_addr = ((off12 - 0x048) >> 2) as u8;
                    PioCommand::WriteInstrMem {
                        block,
                        addr: mem_addr,
                        value: val as u16,
                        alias: alias as u8,
                    }
                }
                // SMn_CLKDIV: 0x0C8, 0x0E0, 0x0F8, 0x110. Stride 0x18.
                0x0C8 | 0x0E0 | 0x0F8 | 0x110 => {
                    let sm = ((off12 - 0x0C8) / 0x18) as u8;
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
            return;
        }
        // DMA / others → legacy HashMap.
        self.legacy_write(canonical, val, alias);
    }

    // ----------------------------------------------------------------
    // Region 0x1 (XIP window + XIP_CTRL + SSI)
    // ----------------------------------------------------------------
    //
    // XIP flash isn't modelled on the RP2040 threaded path (HLD §4.2
    // — RP2040 has no onboard flash). XIP_CTRL at 0x1400_0000 and SSI
    // at 0x1800_0000 are stubbed through the legacy HashMap; reads
    // return 0 unless firmware has written a value there, matching
    // serial `xip_ctrl_read` / `ssi_read` for the subset of offsets
    // those helpers special-case. Special-case the two offsets serial
    // code synthesises (XIP_CTRL.CTRL = 1, SSI.SR = TFE|BF) so
    // firmware init loops terminate.

    fn xip_region_read32(&mut self, addr: u32) -> u32 {
        let canonical = addr & !0x3000;
        let base = canonical & 0xFFFF_F000;
        let offset = canonical & 0x0000_0FFF;
        match base {
            XIP_CTRL_BASE => {
                // XIP_CTRL_CTRL (offset 0x00) reports EN=1 so the
                // bootrom's check for "XIP cache enabled" succeeds.
                if offset == 0x00 {
                    let legacy = self.shared.peripherals.legacy.lock().unwrap();
                    return legacy.get(&canonical).copied().unwrap_or(1);
                }
                self.legacy_read(canonical)
            }
            SSI_BASE => {
                // SSI_SR (offset 0x28) reports TFE|BF so firmware TX
                // wait loops terminate.
                if offset == 0x28 {
                    return 0x05;
                }
                self.legacy_read(canonical)
            }
            _ => self.legacy_read(canonical),
        }
    }

    fn xip_region_write32(&mut self, addr: u32, val: u32) {
        let canonical = addr & !0x3000;
        let alias = (addr >> 12) & 3;
        self.legacy_write(canonical, val, alias);
    }

    // ----------------------------------------------------------------
    // PPB dispatch (region 0xE) — per-core local, same as serial Bus.
    // ----------------------------------------------------------------

    fn ppb_read32(&self, addr: u32) -> u32 {
        self.ppb[self.core_id].read32(addr)
    }

    fn ppb_write32(&mut self, addr: u32, val: u32) {
        self.ppb[self.core_id].write32(addr, val);
    }

    // ----------------------------------------------------------------
    // Legacy HashMap helpers (untyped peripheral fallback).
    // ----------------------------------------------------------------

    fn legacy_read(&self, canonical: u32) -> u32 {
        self.shared
            .peripherals
            .legacy
            .lock()
            .unwrap()
            .get(&canonical)
            .copied()
            .unwrap_or(0)
    }

    fn legacy_write(&self, canonical: u32, val: u32, alias: u32) {
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

    // ----------------------------------------------------------------
    // Outer region dispatch
    // ----------------------------------------------------------------

    fn bus_read32(&mut self, addr: u32) -> u32 {
        // RAM / ROM fast path.
        if Self::is_ram_or_rom_addr(addr) {
            return self.shared.memory.read32(addr);
        }
        match addr >> 28 {
            0x1 => self.xip_region_read32(addr),
            0x4 => self.apb_read32(addr),
            0x5 => self.ahb_read32(addr),
            0xD => self.sio_read32(addr),
            0xE => self.ppb_read32(addr),
            _ => 0,
        }
    }

    fn bus_write32(&mut self, addr: u32, val: u32) {
        if Self::is_ram_or_rom_addr(addr) {
            self.shared.memory.write32(addr, val);
            return;
        }
        match addr >> 28 {
            0x1 => self.xip_region_write32(addr, val),
            0x4 => self.apb_write32(addr, val),
            0x5 => self.ahb_write32(addr, val),
            0xD => self.sio_write32(addr, val),
            0xE => self.ppb_write32(addr, val),
            _ => {}
        }
    }

    /// Expose the `ClockTree` cached inside `peripherals.clocks`.
    /// Cheap read; locks briefly. Used by Stage 3b.4's coordinator when
    /// it decides how many sub-cycles to advance per quantum.
    #[allow(dead_code)]
    pub fn clock_tree(&self) -> ClockTree {
        self.shared.peripherals.clocks.lock().unwrap().clock_tree
    }
}

// =======================================================================
// DividerLocal — per-worker snapshot of the SIO integer divider.
// =======================================================================
//
// HLD §6.4 point 4: divider is per-core-local, no cross-core coherency.
// The SIO divider on real silicon is per-core hardware; matching that
// shape in the threaded path avoids serializing divider writes on a
// shared mutex. Encoded inline here because `mdpicoem_common::Divider`
// is carried on the serial `Sio` type along with its own accounting
// that only the serial path needs.

#[derive(Debug, Clone, Copy, Default)]
struct DividerLocal {
    dividend: u32,
    divisor: u32,
    quotient: u32,
    remainder: u32,
    signed: bool,
    dirty: bool,
}

impl DividerLocal {
    fn compute(&mut self) {
        if self.divisor == 0 {
            // Match serial divide-by-zero semantics in
            // `bus::sio::Sio::compute_division`.
            if self.signed {
                let a = self.dividend as i32;
                self.quotient = if a < 0 { 1u32 } else { (-1i32) as u32 };
            } else {
                self.quotient = 0xFFFF_FFFF;
            }
            self.remainder = self.dividend;
        } else if self.signed {
            let a = self.dividend as i32;
            let b = self.divisor as i32;
            self.quotient = a.wrapping_div(b) as u32;
            self.remainder = a.wrapping_rem(b) as u32;
        } else {
            self.quotient = self.dividend.wrapping_div(self.divisor);
            self.remainder = self.dividend.wrapping_rem(self.divisor);
        }
        self.dirty = true;
    }
}

// =======================================================================
// SYSINFO stub — matches serial `Bus::sysinfo_read`.
// =======================================================================

fn sysinfo_read(offset: u32) -> u32 {
    match offset {
        0x000 => 0x0000_0001, // CHIP_ID placeholder
        0x004 => 0x0000_0000, // PLATFORM
        _ => 0,
    }
}

// =======================================================================
// CoreBus impl
// =======================================================================

impl CoreBus for WorkerBus {
    // --- Memory access ------------------------------------------------
    //
    // Byte / halfword accesses go through `SharedMemory`'s atomic CAS
    // retry loop for RAM/ROM, and through sub-word extracts + RMW for
    // MMIO regions (parity with serial `Bus::read8`/`read16` and
    // `write8`/`write16`, which both synthesise narrow MMIO access via
    // word32).

    fn read8(&mut self, addr: u32) -> u8 {
        if Self::is_ram_or_rom_addr(addr) {
            return self.shared.memory.read8(addr);
        }
        let word_addr = addr & !3;
        let word = self.bus_read32(word_addr);
        let shift = (addr & 3) * 8;
        ((word >> shift) & 0xFF) as u8
    }

    fn read16(&mut self, addr: u32) -> u16 {
        if Self::is_ram_or_rom_addr(addr) {
            return self.shared.memory.read16(addr);
        }
        let word_addr = addr & !3;
        let word = self.bus_read32(word_addr);
        let shift = (addr & 2) * 8;
        ((word >> shift) & 0xFFFF) as u16
    }

    fn read32(&mut self, addr: u32) -> u32 {
        self.bus_read32(addr)
    }

    fn write8(&mut self, addr: u32, val: u8) {
        if Self::is_ram_or_rom_addr(addr) {
            self.shared.memory.write8(addr, val);
            return;
        }
        // Sub-word MMIO write: serial `Bus` ignores narrow writes to
        // most peripherals (PIO / SIO GPIO) because RMW would trigger
        // side-effects on read-with-side-effects registers. Stage 3b.3
        // matches that shape by routing narrow writes through the same
        // word32 path — firmware that issues narrow writes outside the
        // serial supported path observes the same RMW artefact.
        let word_addr = addr & !3;
        let old = self.bus_read32(word_addr);
        let shift = (addr & 3) * 8;
        let new_word = (old & !(0xFFu32 << shift)) | ((val as u32) << shift);
        self.bus_write32(word_addr, new_word);
    }

    fn write16(&mut self, addr: u32, val: u16) {
        if Self::is_ram_or_rom_addr(addr) {
            self.shared.memory.write16(addr, val);
            return;
        }
        let word_addr = addr & !3;
        let old = self.bus_read32(word_addr);
        let shift = (addr & 2) * 8;
        let new_word = (old & !(0xFFFFu32 << shift)) | ((val as u32) << shift);
        self.bus_write32(word_addr, new_word);
    }

    fn write32(&mut self, addr: u32, val: u32) {
        self.bus_write32(addr, val);
    }

    // --- Instruction-boundary metadata --------------------------------

    fn set_active_pc(&mut self, pc: u32) {
        self.active_pc = pc;
    }

    // --- Bus fault ----------------------------------------------------

    fn bus_fault(&self) -> bool {
        self.bus_fault
    }

    fn bus_fault_addr(&self) -> u32 {
        self.bus_fault_addr
    }

    fn clear_bus_fault(&mut self) {
        self.bus_fault = false;
    }

    // --- Per-core PPB / NVIC ------------------------------------------
    //
    // Only `self.core_id` is a live slot on this `WorkerBus`. The other
    // slot is an inert placeholder kept for trait-signature compatibility
    // with the serial `Bus::ppb(core)` / `nvic(core)` accessor shape so
    // callers (exception entry, NVIC dispatch) don't have to branch on
    // the execution model. Any attempt to address the peer slot is a
    // misuse of the threaded path — cross-core exception delivery flows
    // through `SharedState.atomics`, not through a peer's inert PPB.

    fn ppb(&self, core: usize) -> &Ppb {
        debug_assert_eq!(
            core, self.core_id,
            "peer-core PPB slot is inert; cross-core exception delivery goes through shared atomics"
        );
        &self.ppb[core]
    }

    fn ppb_mut(&mut self, core: usize) -> &mut Ppb {
        debug_assert_eq!(
            core, self.core_id,
            "peer-core PPB slot is inert; cross-core exception delivery goes through shared atomics"
        );
        &mut self.ppb[core]
    }

    fn nvic(&self, core: usize) -> &Nvic {
        debug_assert_eq!(
            core, self.core_id,
            "peer-core NVIC slot is inert; cross-core IRQ delivery goes through shared atomics"
        );
        &self.nvic[core]
    }

    fn nvic_mut(&mut self, core: usize) -> &mut Nvic {
        debug_assert_eq!(
            core, self.core_id,
            "peer-core NVIC slot is inert; cross-core IRQ delivery goes through shared atomics"
        );
        &mut self.nvic[core]
    }

    // --- Scheduler plumbing -------------------------------------------

    fn active_core(&self) -> usize {
        self.core_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::threaded::memory::{ROM_BASE, SRAM_BASE};
    use std::sync::atomic::Ordering;

    fn fresh_worker(core_id: usize) -> (Arc<SharedState>, WorkerBus) {
        let shared = Arc::new(SharedState::new_default());
        let bus = WorkerBus::new(shared.clone(), core_id);
        (shared, bus)
    }

    // --- Classifier tightening ------------------------------------------------

    #[test]
    fn classifier_admits_rom_and_sram_aliases() {
        // Boot ROM 0x0000_0000..0x0000_4000.
        assert!(WorkerBus::is_ram_or_rom_addr(ROM_BASE));
        assert!(WorkerBus::is_ram_or_rom_addr(0x0000_3FFF));
        // SRAM base + 3 aliases.
        assert!(WorkerBus::is_ram_or_rom_addr(SRAM_BASE));
        assert!(WorkerBus::is_ram_or_rom_addr(0x2100_0100));
        assert!(WorkerBus::is_ram_or_rom_addr(0x2200_0200));
        assert!(WorkerBus::is_ram_or_rom_addr(0x2300_0300));
        // Last valid SRAM word in alias 3.
        assert!(WorkerBus::is_ram_or_rom_addr(0x2304_1FFC));
    }

    #[test]
    fn classifier_rejects_out_of_range_sram() {
        // 0x2004_2000 is one byte past the end of SRAM — must be
        // rejected so MMIO routing handles it.
        assert!(!WorkerBus::is_ram_or_rom_addr(0x2004_2000));
        assert!(!WorkerBus::is_ram_or_rom_addr(0x2004_FFFF));
        // Alias 4 (0x2400_0000) isn't a real SRAM alias.
        assert!(!WorkerBus::is_ram_or_rom_addr(0x2400_0000));
        assert!(!WorkerBus::is_ram_or_rom_addr(0x2FFF_FFFF));
        // ROM past 16 KB.
        assert!(!WorkerBus::is_ram_or_rom_addr(0x0000_4000));
        assert!(!WorkerBus::is_ram_or_rom_addr(0x0000_FFFF));
        // MMIO regions stay out.
        assert!(!WorkerBus::is_ram_or_rom_addr(0x4000_8000));
        assert!(!WorkerBus::is_ram_or_rom_addr(0xD000_0000));
    }

    // --- SIO ------------------------------------------------------------------

    #[test]
    fn cpuid_returns_core_id() {
        let (_s0, mut b0) = fresh_worker(0);
        let (_s1, mut b1) = fresh_worker(1);
        assert_eq!(b0.read32(0xD000_0000), 0);
        assert_eq!(b1.read32(0xD000_0000), 1);
    }

    #[test]
    fn gpio_out_write_is_observable_on_peer() {
        let (shared, mut b0) = fresh_worker(0);
        b0.write32(0xD000_0010, 0x1234_5678);
        // Peer sees the SET on the shared atomic (pin mask strips to 30 bits).
        assert_eq!(
            shared.gpio_out.load(Ordering::Acquire),
            0x1234_5678 & 0x3FFF_FFFF
        );
        // Read back through the bus.
        assert_eq!(b0.read32(0xD000_0010), 0x1234_5678 & 0x3FFF_FFFF);
    }

    #[test]
    fn gpio_out_set_clr_xor_round_trip() {
        let (_shared, mut b0) = fresh_worker(0);
        b0.write32(0xD000_0010, 0x0F); // plain store
        b0.write32(0xD000_0014, 0x10); // SET
        assert_eq!(b0.read32(0xD000_0010), 0x1F);
        b0.write32(0xD000_0018, 0x01); // CLR
        assert_eq!(b0.read32(0xD000_0010), 0x1E);
        b0.write32(0xD000_001C, 0xFF); // XOR
        assert_eq!(b0.read32(0xD000_0010), 0xE1);
    }

    #[test]
    fn gpio_oe_round_trip_distinct_from_out() {
        let (_shared, mut b0) = fresh_worker(0);
        b0.write32(0xD000_0010, 0xAA);
        b0.write32(0xD000_0020, 0x55);
        assert_eq!(b0.read32(0xD000_0010), 0xAA);
        assert_eq!(b0.read32(0xD000_0020), 0x55);
    }

    #[test]
    fn fifo_push_from_core0_visible_on_core1_read() {
        let shared = Arc::new(SharedState::new_default());
        let mut b0 = WorkerBus::new(shared.clone(), 0);
        let mut b1 = WorkerBus::new(shared.clone(), 1);
        b0.write32(0xD000_0054, 0xDEAD_BEEF);
        // Peer sees the push on its RX queue via its own FIFO_RD.
        assert_eq!(b1.read32(0xD000_0058), 0xDEAD_BEEF);
    }

    #[test]
    fn fifo_push_raises_peer_irq_pending_bit() {
        let shared = Arc::new(SharedState::new_default());
        let mut b0 = WorkerBus::new(shared.clone(), 0);
        // Core 0 pushes → peer (core 1) gets SIO_PROC1_IRQ (IRQ 16)
        // asserted on `atomics.irq_pending[1]`. The Stage 3b.4 worker
        // drains this via `drain_cross_core_irqs` into the local NVIC.
        b0.write32(0xD000_0054, 0x1);
        let bits = shared.atomics.irq_pending_load(1);
        assert_ne!(bits & (1 << 16), 0, "SIO_PROC1_IRQ must be latched");
    }

    #[test]
    fn fifo_status_reflects_queue_state() {
        let (_shared, mut b0) = fresh_worker(0);
        // Empty RX, non-full TX → vld=0, rdy=1 → bit 1 set.
        let st = b0.read32(0xD000_0050);
        assert_eq!(st & 0x3, 0b10);
    }

    #[test]
    fn fifo_push_to_full_latches_wof_and_w1c_clears() {
        // Matches serial `Sio::fifo_wr` + `fifo_st_read`/`fifo_st_write`
        // semantics. Push until the queue is full then push once more —
        // WOF bit 2 of FIFO_ST must latch; W1C clears it.
        let shared = Arc::new(SharedState::new_default());
        let mut b0 = WorkerBus::new(shared.clone(), 0);
        // SIO FIFO depth is 8. Fill it, then overflow.
        for i in 0..8u32 {
            b0.write32(0xD000_0054, i);
        }
        // No WOF yet (all pushes succeeded).
        assert_eq!(b0.read32(0xD000_0050) & (1 << 2), 0);
        // One more push overflows → WOF latches on core 0 (sender).
        b0.write32(0xD000_0054, 0xDEAD);
        let st = b0.read32(0xD000_0050);
        assert_ne!(st & (1 << 2), 0, "WOF must latch on push-to-full");
        // W1C clears WOF (bit 2).
        b0.write32(0xD000_0050, 1 << 2);
        assert_eq!(
            b0.read32(0xD000_0050) & (1 << 2),
            0,
            "WOF must clear after W1C"
        );
    }

    #[test]
    fn fifo_pop_from_empty_latches_roe_and_w1c_clears() {
        let (_shared, mut b0) = fresh_worker(0);
        // RX queue is empty. A pop returns 0 and latches ROE (bit 3).
        let v = b0.read32(0xD000_0058);
        assert_eq!(v, 0);
        let st = b0.read32(0xD000_0050);
        assert_ne!(st & (1 << 3), 0, "ROE must latch on pop-from-empty");
        // W1C clears ROE.
        b0.write32(0xD000_0050, 1 << 3);
        assert_eq!(
            b0.read32(0xD000_0050) & (1 << 3),
            0,
            "ROE must clear after W1C"
        );
    }

    #[test]
    fn spinlock_claim_cas_release_round_trip() {
        let (_shared, mut b0) = fresh_worker(0);
        // First claim returns the bit.
        assert_eq!(b0.read32(0xD000_0100), 0x1);
        // Second claim fails.
        assert_eq!(b0.read32(0xD000_0100), 0);
        // Release and re-claim.
        b0.write32(0xD000_0100, 0);
        assert_eq!(b0.read32(0xD000_0100), 0x1);
    }

    #[test]
    fn spinlock_cross_core_contention() {
        let shared = Arc::new(SharedState::new_default());
        let mut b0 = WorkerBus::new(shared.clone(), 0);
        let mut b1 = WorkerBus::new(shared.clone(), 1);
        // Core 0 claims lock 5 → core 1 observes held.
        let lock_addr = 0xD000_0100 + 5 * 4;
        assert_eq!(b0.read32(lock_addr), 1 << 5);
        assert_eq!(b1.read32(lock_addr), 0);
        // Core 0 releases → core 1 can claim.
        b0.write32(lock_addr, 0);
        assert_eq!(b1.read32(lock_addr), 1 << 5);
    }

    #[test]
    fn divider_local_write_read_back() {
        let (_shared, mut b0) = fresh_worker(0);
        // Unsigned 100 / 7 = 14 rem 2.
        b0.write32(0xD000_0060, 100); // DIV_UDIVIDEND
        b0.write32(0xD000_0064, 7); // DIV_UDIVISOR
        assert_eq!(b0.read32(0xD000_0070), 14); // QUOTIENT
        assert_eq!(b0.read32(0xD000_0074), 2); // REMAINDER
    }

    #[test]
    fn divider_is_per_worker_not_shared() {
        let shared = Arc::new(SharedState::new_default());
        let mut b0 = WorkerBus::new(shared.clone(), 0);
        let mut b1 = WorkerBus::new(shared.clone(), 1);
        b0.write32(0xD000_0060, 100);
        b0.write32(0xD000_0064, 7);
        // Core 1 divider is untouched.
        assert_eq!(b1.read32(0xD000_0070), 0);
    }

    #[test]
    fn divider_signed_div_by_zero_matches_serial() {
        let (_shared, mut b0) = fresh_worker(0);
        b0.write32(0xD000_0068, (-42i32) as u32); // SDIVIDEND
        b0.write32(0xD000_006C, 0); // SDIVISOR
        assert_eq!(b0.read32(0xD000_0070), 1); // quotient = 1 (matches serial)
        assert_eq!(b0.read32(0xD000_0074), (-42i32) as u32);
    }

    #[test]
    fn interp_is_per_worker_not_shared() {
        let shared = Arc::new(SharedState::new_default());
        let mut b0 = WorkerBus::new(shared.clone(), 0);
        let mut b1 = WorkerBus::new(shared.clone(), 1);
        b0.write32(0xD000_0080, 0xAAAA_AAAA);
        b1.write32(0xD000_0080, 0xBBBB_BBBB);
        assert_eq!(b0.read32(0xD000_0080), 0xAAAA_AAAA);
        assert_eq!(b1.read32(0xD000_0080), 0xBBBB_BBBB);
    }

    // --- APB / clock tree ------------------------------------------------

    #[test]
    fn clocks_ctrl_read_after_write() {
        let (_shared, mut b0) = fresh_worker(0);
        // CLK_SYS_CTRL is at 0x4000_8000 + 0x3C.
        b0.write32(0x4000_803C, 0x1);
        assert_eq!(b0.read32(0x4000_803C), 0x1);
        // CLK_SYS_SELECTED at 0x44 mirrors 1<<SRC (here SRC=1 → 2).
        assert_eq!(b0.read32(0x4000_8044), 0x2);
    }

    #[test]
    fn resets_blocks_writes_and_reads_until_released() {
        let (_shared, mut b0) = fresh_worker(0);
        // Release UART0 (bit 22) so the write lands in legacy storage.
        b0.write32(0x4000_C000 + 0x3000, 1u32 << 22);
        // Write into a UART0 offset (UARTDR at +0x00).
        b0.write32(0x4003_4000 + 0x04, 0xAA);
        // Confirm read-back works while released.
        assert_eq!(b0.read32(0x4003_4000 + 0x04), 0xAA);
        // Re-hold UART0 (bit 22) via SET alias at +0x2000.
        b0.write32(0x4000_C000 + 0x2000, 1u32 << 22);
        // Blocked read returns 0 without touching the peripheral,
        // even though legacy still stores 0xAA.
        assert_eq!(b0.read32(0x4003_4000 + 0x04), 0);
        // Release again: the stored value must still be present.
        b0.write32(0x4000_C000 + 0x3000, 1u32 << 22);
        assert_eq!(
            b0.read32(0x4003_4000 + 0x04),
            0xAA,
            "stored value survives the reset-held window"
        );
    }

    #[test]
    fn pll_sys_pwr_write_arms_lock() {
        let (shared, mut b0) = fresh_worker(0);
        // PWR write with FBDIV set triggers lock_at arming.
        b0.write32(0x4002_8004, 0); // PWR cleared
        b0.write32(0x4002_8008, 125); // FBDIV
        let lock = shared
            .peripherals
            .clocks
            .lock()
            .unwrap()
            .pll_sys_lock_at_cycle;
        assert!(lock.is_some(), "PLL_SYS lock must be armed after PWR+FBDIV");
    }

    #[test]
    fn xosc_status_reads_stable_enabled() {
        let (_shared, mut b0) = fresh_worker(0);
        // XOSC STATUS at 0x4002_4004 should always report STABLE|ENABLED.
        let status = b0.read32(0x4002_4004);
        assert_ne!(status & (1 << 31), 0);
        assert_ne!(status & (1 << 12), 0);
    }

    #[test]
    fn rosc_ctrl_round_trip() {
        let (_shared, mut b0) = fresh_worker(0);
        b0.write32(0x4006_0000, 0xFABC_0FA0);
        assert_eq!(b0.read32(0x4006_0000), 0xFABC_0FA0);
    }

    #[test]
    fn io_bank0_ctrl_round_trip() {
        let (_shared, mut b0) = fresh_worker(0);
        // GPIO0_CTRL at offset 0x004 inside IO_BANK0.
        let addr = 0x4001_4000 + 0x004;
        b0.write32(addr, 0x5);
        assert_eq!(b0.read32(addr), 0x5);
    }

    #[test]
    fn pads_bank0_write_alias_or() {
        let (_shared, mut b0) = fresh_worker(0);
        // Pad GPIO0 at offset 0x04. Default PAD_RESET = 0x56.
        let base = 0x4001_C000 + 0x04;
        // BITSET alias (offset +0x2000) sets bit 0x80.
        b0.write32(base + 0x2000, 0x80);
        assert_eq!(b0.read32(base), 0x56 | 0x80);
    }

    // --- AHB / PIO --------------------------------------------------------

    #[test]
    fn pio_ctrl_write_enqueues_command() {
        let (shared, mut b0) = fresh_worker(0);
        // PIO0 CTRL at 0x5020_0000.
        b0.write32(0x5020_0000, 0x0F);
        // The coordinator's drain sees a typed WriteCtrl with the exact
        // payload the worker wrote.
        let drained = shared.pio.drain_commands(0);
        assert_eq!(drained.len(), 1);
        assert!(
            matches!(
                drained[0],
                PioCommand::WriteCtrl {
                    block: 0,
                    val: 0x0F,
                    alias: 0,
                }
            ),
            "expected WriteCtrl{{block:0, val:0x0F, alias:0}}, got {:?}",
            drained[0]
        );
        // Second drain on block 0 must be empty; block 1 untouched.
        assert!(shared.pio.drain_commands(0).is_empty());
        assert!(shared.pio.drain_commands(1).is_empty());
    }

    #[test]
    fn pio_ctrl_read_reports_sm_enabled_snapshot() {
        let (shared, mut b0) = fresh_worker(0);
        // Stage 3b.3: CTRL read returns sm_enabled atomic (0 until
        // coordinator publishes).
        assert_eq!(b0.read32(0x5020_0000), 0);
        shared.pio.publish_sm_enabled(0, 0x5);
        assert_eq!(b0.read32(0x5020_0000), 0x5);
    }

    #[test]
    fn pio_generic_reg_write_enqueues_command() {
        let (shared, mut b0) = fresh_worker(0);
        // TXF0 at 0x010 — routes through WriteReg variant.
        b0.write32(0x5020_0010, 0xDEAD_BEEF);
        let drained = shared.pio.drain_commands(0);
        assert_eq!(drained.len(), 1);
        assert!(
            matches!(
                drained[0],
                PioCommand::WriteReg {
                    block: 0,
                    offset: 0x010,
                    val: 0xDEAD_BEEF,
                    alias: 0,
                }
            ),
            "expected WriteReg{{block:0, offset:0x010, val:0xDEAD_BEEF, alias:0}}, got {:?}",
            drained[0]
        );
    }

    #[test]
    fn pio_instr_mem_write_enqueues_typed_command() {
        let (shared, mut b0) = fresh_worker(0);
        // PIO0 INSTR_MEM5 at 0x5020_0000 + 0x048 + 5*4 = 0x5020_005C.
        b0.write32(0x5020_005C, 0x0000_E020);
        let drained = shared.pio.drain_commands(0);
        assert_eq!(drained.len(), 1);
        assert!(
            matches!(
                drained[0],
                PioCommand::WriteInstrMem {
                    block: 0,
                    addr: 5,
                    value: 0xE020,
                    alias: 0,
                }
            ),
            "expected WriteInstrMem{{block:0, addr:5, value:0xE020, alias:0}}, got {:?}",
            drained[0]
        );
    }

    #[test]
    fn pio_clkdiv_write_enqueues_typed_command() {
        let (shared, mut b0) = fresh_worker(0);
        // PIO0 SM0 CLKDIV at 0x5020_0000 + 0x0C8.
        // int_div=0x00C8, frac_div=0x34 → packed: int<<16 | frac<<8.
        b0.write32(0x5020_00C8, (0x00C8u32 << 16) | (0x34u32 << 8));
        let drained = shared.pio.drain_commands(0);
        assert_eq!(drained.len(), 1);
        assert!(
            matches!(
                drained[0],
                PioCommand::SetClkDiv {
                    block: 0,
                    sm: 0,
                    int_div: 0x00C8,
                    frac_div: 0x34,
                    alias: 0,
                }
            ),
            "expected SetClkDiv{{block:0, sm:0, int_div:0x00C8, frac_div:0x34, alias:0}}, got {:?}",
            drained[0]
        );
    }

    // --- TIMER typed state (preserves TIMELR→TIMEHR latching) ------------

    #[test]
    fn timer_timelr_read_latches_timehr() {
        let (shared, mut b0) = fresh_worker(0);
        // Release TIMER from reset (bit 21 at CLR alias).
        b0.write32(0x4000_C000 + 0x3000, 1u32 << 21);
        // Set sys_hz to a known value by triggering clock-tree recompute
        // — default CLK_SYS tree uses ref_clk; a sys_hz of exactly
        // 1 MHz keeps master_cycle == microseconds which simplifies
        // the assertion. Instead, seed master_cycle and derive us from
        // it: master_cycle * 1e6 / sys_hz = us. With the default XOSC
        // clock at 12 MHz and clk_ref = 12_000_000, sys_hz stays 12 MHz
        // post-bootrom; cycles_to_us divides by sys_hz/1_000_000 = 12,
        // so to express a precise high/low split we pre-compute.
        let sys_hz = shared
            .peripherals
            .clocks
            .lock()
            .unwrap()
            .clock_tree
            .sys_clk_hz
            .max(1);
        // Aim for 'now_us' = (3 << 32) | 42. cycles = us * (sys_hz / 1e6).
        let target_us: u64 = (3u64 << 32) | 42;
        let divisor = (sys_hz / 1_000_000).max(1) as u64;
        let cycles = target_us.saturating_mul(divisor);
        // master_cycle is an Arc<AtomicU64>.
        shared
            .master_cycle
            .store(cycles, std::sync::atomic::Ordering::Release);
        // TIMELR (0x0C) read returns low 32 bits + latches TIMEHR.
        let lo = b0.read32(0x4005_4000 + 0x0C);
        assert_eq!(lo, 42, "TIMELR must return low 32 bits of now_us");
        // Advance master_cycle after the latch — TIMEHR must still
        // return the latched value, not the live one.
        shared.master_cycle.store(
            cycles + (9u64 << 32) * divisor,
            std::sync::atomic::Ordering::Release,
        );
        let hi = b0.read32(0x4005_4000 + 0x08);
        assert_eq!(
            hi, 3,
            "TIMEHR must return the latched high half, not the live value"
        );
    }

    // --- Cross-core IRQ drain --------------------------------------------

    #[test]
    fn drain_cross_core_irqs_merges_pending_mask_into_local_nvic() {
        let (shared, mut b0) = fresh_worker(0);
        // Peer sets IRQ 15 (SIO_PROC0_IRQ) on core 0.
        shared.atomics.assert_irq(0, 15);
        b0.drain_cross_core_irqs();
        assert_ne!(b0.nvic[0].pending & (1 << 15), 0);
        // Shared mask is drained.
        assert_eq!(shared.atomics.irq_pending_load(0), 0);
    }

    #[test]
    fn drain_cross_core_irqs_preserves_existing_local_pending() {
        let (shared, mut b0) = fresh_worker(0);
        // Locally-latched IRQ 3 must survive the drain-merge.
        b0.nvic[0].pending = 1 << 3;
        shared.atomics.assert_irq(0, 7);
        b0.drain_cross_core_irqs();
        assert_eq!(b0.nvic[0].pending & (1 << 3), 1 << 3);
        assert_ne!(b0.nvic[0].pending & (1 << 7), 0);
    }

    #[test]
    fn drain_cross_core_irqs_is_noop_when_empty() {
        let (_shared, mut b0) = fresh_worker(0);
        b0.drain_cross_core_irqs();
        assert_eq!(b0.nvic[0].pending, 0);
    }

    // --- Narrow access ---------------------------------------------------

    #[test]
    fn narrow_read_ram_round_trip() {
        let (_shared, mut b0) = fresh_worker(0);
        b0.write32(SRAM_BASE + 0x100, 0x1122_3344);
        assert_eq!(b0.read8(SRAM_BASE + 0x100), 0x44);
        assert_eq!(b0.read8(SRAM_BASE + 0x101), 0x33);
        assert_eq!(b0.read16(SRAM_BASE + 0x102), 0x1122);
    }

    // --- Basic smoke test from Stage 3b.2 ---------------------------------

    #[test]
    fn construct_and_basic_sram_roundtrip() {
        let shared = Arc::new(SharedState::new_default());
        let mut bus = WorkerBus::new(shared.clone(), 0);

        assert_eq!(bus.active_core(), 0);

        bus.write32(SRAM_BASE + 0x100, 0xDEAD_BEEF);
        assert_eq!(bus.read32(SRAM_BASE + 0x100), 0xDEAD_BEEF);
        assert_eq!(shared.memory.read32(SRAM_BASE + 0x100), 0xDEAD_BEEF);

        // ROM reads back zero on a fresh memory.
        assert_eq!(bus.read32(ROM_BASE), 0);

        // PPB / NVIC hand out live references.
        bus.ppb_mut(0).vtor = 0x1000_0100;
        assert_eq!(bus.ppb(0).vtor, 0x1000_0100);
        bus.nvic_mut(0).pending = 0xFF;
        assert_eq!(bus.nvic(0).pending, 0xFF);

        // Bus-fault slot.
        assert!(!bus.bus_fault());
        bus.clear_bus_fault();
        assert_eq!(bus.bus_fault_addr(), 0);

        bus.set_active_pc(0x1000_0200);
    }
}

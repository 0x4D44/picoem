//! RP2350 DMA controller — Phase 3 (HLD V5 §5.6).
//!
//! 16-channel DMA bus master with ring/chain/abort/DREQ matrix. Drives
//! `hello_dma` (mem -> mem), `dma_uart` (DREQ + UART), and DMA chain
//! scenarios from the corpus.
//!
//! ### Scope (V1)
//!
//! * 16 channels (0..=15), transfer sizes 1 / 2 / 4 bytes.
//! * Per-channel registers: `CTRL`, `READ_ADDR`, `WRITE_ADDR`,
//!   `TRANS_COUNT`, plus the read-back aliases (`CTRL_TRIG`,
//!   `AL1_*`, `AL2_*`, `AL3_*`) that share state but trigger on the
//!   write to their trigger variant.
//! * `RING_SIZE` + `RING_SEL` address masking for circular buffers.
//! * `CHAIN_TO` with full chained triggering — `TRANS_COUNT` hits 0 ->
//!   enable target channel (if not self).
//! * `CH_ABORT`: writing 1-bits clears `BUSY` immediately on those
//!   channels.
//! * `INTE0`/`INTE1`/`INTS0`/`INTS1`/`INTR`: per-channel enable masks.
//!   `INTR` latches on transfer completion; `INTS0`/`INTS1` are W1C on
//!   `INTR` bits. `DMA_IRQ_0` / `DMA_IRQ_1` on NVIC lines 10 / 11.
//! * Fixed-priority arbitration: lowest channel index wins.
//!
//! ### Not in V1 (per HLD V5 §5.6)
//!
//! * CRC (`SNIFF_CTRL` registers — storage-only).
//! * Sniff (`SNIFF_DATA` — storage-only).
//! * Byte-swap (`BSWAP` bit) — field stored but ignored.
//! * `HIGH_PRIORITY` two-tier arbitration.
//! * `DMA_IRQ_2` / `DMA_IRQ_3`.
//! * Read-error / write-error IRQs.
//!
//! ### Ordering contract
//!
//! Per V5 §5.6: peripherals tick first (produce DREQ), then `tick_dma`
//! consumes the DREQ snapshot via [`Bus::collect_dreqs`]. DMA writes
//! take effect the cycle they issue — real AHB is N+1 due to address /
//! data phases, but no corpus scenario distinguishes.

use crate::bus::Bus;
use crate::dreq::{DREQ_FORCE, DREQ_TIMER0, DREQ_TIMER3};
use crate::irq::{IRQ_DMA_IRQ_0, IRQ_DMA_IRQ_1, IRQ_DMA_IRQ_2, IRQ_DMA_IRQ_3};

/// Total number of DMA channels on RP2350 (datasheet §12.6.1).
pub const NUM_CHANNELS: usize = 16;

/// DMA base address (AHB-Lite, not APB).
pub const DMA_BASE: u32 = 0x5000_0000;

/// Size of the DMA register aperture as decoded by `Bus::read32`/`write32`
/// (16 KB — one 4 KB page at 0x5000_0000 plus three APB alias mirrors at
/// `+0x1000`/`+0x2000`/`+0x3000`). Used by [`Dma::issue_transfer`] to
/// detect DMA-to-DMA transfers and route them through `self` instead of
/// the bus, which would otherwise land on the empty stand-in left by
/// [`crate::bus::Bus::tick_dma`]'s `mem::take`.
const DMA_APERTURE_SIZE: u32 = 0x4000;

// Per-channel register offsets inside one 0x40-byte channel stride.
const CH_READ_ADDR: u32 = 0x00;
const CH_WRITE_ADDR: u32 = 0x04;
const CH_TRANS_COUNT: u32 = 0x08;
const CH_CTRL_TRIG: u32 = 0x0C;
const CH_AL1_CTRL: u32 = 0x10;
const CH_AL1_READ_ADDR: u32 = 0x14;
const CH_AL1_WRITE_ADDR_TRIG: u32 = 0x18;
const CH_AL1_TRANS_COUNT: u32 = 0x1C;
const CH_AL2_CTRL: u32 = 0x20;
const CH_AL2_TRANS_COUNT_TRIG: u32 = 0x24;
const CH_AL2_READ_ADDR: u32 = 0x28;
const CH_AL2_WRITE_ADDR: u32 = 0x2C;
const CH_AL3_CTRL: u32 = 0x30;
const CH_AL3_WRITE_ADDR: u32 = 0x34;
const CH_AL3_TRANS_COUNT: u32 = 0x38;
const CH_AL3_READ_ADDR_TRIG: u32 = 0x3C;

// Per-channel debug registers — read-only (datasheet §12.6.6).
const CH_DBG_CTDREQ_OFFSET: u32 = 0x800;

// Global registers (RP2350 datasheet §12.6.6).
//
// RP2350 inserts two new IRQ groups vs RP2040 — INTE2/INTF2/INTS2 at
// 0x424..0x42C and INTE3/INTF3/INTS3 at 0x434..0x43C — which shifts
// every register from 0x420 onward up by 0x20 bytes relative to the
// RP2040 layout.  Pre-fix this file inherited the RP2040 offsets
// wholesale (commit 4cc7906) and `dma_timer_paced` ran green on the
// emulator while failing on silicon: the sled's write to 0x5000_0420
// hit reserved padding, the real TIMER0 at 0x5000_0440 stayed at
// reset, DREQ_TIMER0 never asserted, BUSY never cleared (Residual
// C.2.1, 2026-04-17).
const REG_INTR: u32 = 0x400;
const REG_INTE0: u32 = 0x404;
const REG_INTF0: u32 = 0x408;
const REG_INTS0: u32 = 0x40C;
// 0x410 reserved.
const REG_INTE1: u32 = 0x414;
const REG_INTF1: u32 = 0x418;
const REG_INTS1: u32 = 0x41C;
// 0x420 reserved (RP2040 TIMER0 lived here — shifted to 0x440 on RP2350).
const REG_INTE2: u32 = 0x424;
const REG_INTF2: u32 = 0x428;
const REG_INTS2: u32 = 0x42C;
// 0x430 reserved.
const REG_INTE3: u32 = 0x434;
const REG_INTF3: u32 = 0x438;
const REG_INTS3: u32 = 0x43C;
const REG_TIMER0: u32 = 0x440;
const REG_TIMER1: u32 = 0x444;
const REG_TIMER2: u32 = 0x448;
const REG_TIMER3: u32 = 0x44C;
const REG_MULTI_CHAN_TRIGGER: u32 = 0x450;
const REG_SNIFF_CTRL: u32 = 0x454;
const REG_SNIFF_DATA: u32 = 0x458;
// 0x45C reserved.
const REG_FIFO_LEVELS: u32 = 0x460;
const REG_CHAN_ABORT: u32 = 0x464;
const REG_N_CHANNELS: u32 = 0x468;

// CTRL bit fields (RP2350 datasheet §12.6.6 CH0_CTRL_TRIG).
//
// RP2350 adds INCR_READ_REV [5] and INCR_WRITE_REV [7] vs RP2040, which
// shifts RING_SIZE, RING_SEL, CHAIN_TO, TREQ_SEL, and IRQ_QUIET each up
// by 2 bits relative to RP2040.  The post-V5 fix below corrects these.
//
// Full RP2350 field map:
//  bit 0      EN
//  bit 1      HIGH_PRIORITY
//  bits[3:2]  DATA_SIZE
//  bit 4      INCR_READ
//  bit 5      INCR_READ_REV
//  bit 6      INCR_WRITE
//  bit 7      INCR_WRITE_REV
//  bits[11:8] RING_SIZE
//  bit 12     RING_SEL
//  bits[16:13] CHAIN_TO
//  bits[22:17] TREQ_SEL
//  bit 23     IRQ_QUIET
//  bit 24     BSWAP
//  bit 25     SNIFF_EN
//  bit 26     BUSY (RO)
//  bits[28:27] reserved
//  bit 29     WRITE_ERROR (W1C)
//  bit 30     READ_ERROR  (W1C)
//  bit 31     AHB_ERROR   (RO, OR of READ_ERROR | WRITE_ERROR)
const CTRL_EN: u32 = 1 << 0;
/// `HIGH_PRIORITY` flag — not modelled in V1 (flat priority; HLD V5 §5.6
/// "Not in V1"). Used by [`Dma::check_inert_ctrl_bits`] for warn-once.
const CTRL_HIGH_PRIORITY: u32 = 1 << 1;
const CTRL_DATA_SIZE_SHIFT: u32 = 2;
const CTRL_DATA_SIZE_MASK: u32 = 0x3 << CTRL_DATA_SIZE_SHIFT;
const CTRL_INCR_READ: u32 = 1 << 4;
/// `INCR_READ_REV` — stored but not modelled in V1 (reverse-increment not
/// exercised by any V1 corpus scenario).
#[allow(dead_code)]
const CTRL_INCR_READ_REV: u32 = 1 << 5;
const CTRL_INCR_WRITE: u32 = 1 << 6;
/// `INCR_WRITE_REV` — stored but not modelled in V1.
#[allow(dead_code)]
const CTRL_INCR_WRITE_REV: u32 = 1 << 7;
const CTRL_RING_SIZE_SHIFT: u32 = 8;
const CTRL_RING_SIZE_MASK: u32 = 0xF << CTRL_RING_SIZE_SHIFT;
const CTRL_RING_SEL: u32 = 1 << 12;
const CTRL_CHAIN_TO_SHIFT: u32 = 13;
const CTRL_CHAIN_TO_MASK: u32 = 0xF << CTRL_CHAIN_TO_SHIFT;
const CTRL_TREQ_SEL_SHIFT: u32 = 17;
const CTRL_TREQ_SEL_MASK: u32 = 0x3F << CTRL_TREQ_SEL_SHIFT;
const CTRL_IRQ_QUIET: u32 = 1 << 23;
/// `BSWAP` (byte-swap) flag — not modelled in V1 (HLD V5 §5.6 "Not in
/// V1"). Stored through CTRL RMW but ignored on transfer. Used by
/// [`Dma::check_inert_ctrl_bits`] for warn-once.
const CTRL_BSWAP: u32 = 1 << 24;
/// `SNIFF_EN` — not modelled in V1 (no CRC). Stored but ignored.
#[allow(dead_code)]
const CTRL_SNIFF_EN: u32 = 1 << 25;
const CTRL_BUSY: u32 = 1 << 26;
// Bits [28:27] are reserved per RP2350 datasheet §12.6.6.
const CTRL_WRITE_ERROR: u32 = 1 << 29;
const CTRL_READ_ERROR: u32 = 1 << 30;
const CTRL_AHB_ERROR: u32 = 1u32 << 31;
// Mask of writable bits in CTRL: exclude BUSY (RO), the three error bits
// (status / W1C), and the reserved bits [28:27].
const CTRL_WRITABLE_MASK: u32 =
    !(CTRL_BUSY | CTRL_WRITE_ERROR | CTRL_READ_ERROR | CTRL_AHB_ERROR | (0x3 << 27));

// Channel mask: 16 channels = bits [15:0].
const CHANNEL_MASK: u32 = 0xFFFF;

/// One DMA channel. Tracks the live transfer state and the program
/// registers. `trans_count_reload` snapshots the value written to
/// `TRANS_COUNT` so a chained reloader channel can pre-program the
/// count without it being consumed by the preceding transfer.
#[derive(Clone, Copy, Default)]
pub struct DmaChannel {
    /// Current source address (increments on transfer per `INCR_READ`
    /// or ring-wraps per `RING_SEL`).
    pub read_addr: u32,
    /// Current destination address.
    pub write_addr: u32,
    /// Remaining transfers. Latches to `trans_count_reload` when the
    /// channel fires and decrements toward 0. `BUSY` clears when this
    /// hits 0.
    pub trans_count: u32,
    /// Original count written by firmware — used to reload after chain
    /// or multi-trigger.
    pub trans_count_reload: u32,
    /// CTRL register. `BUSY` is derived on read from [`Self::busy`].
    pub ctrl: u32,
    /// Transfer-in-progress flag. Decoupled from `ctrl` so a
    /// read-through-CTRL still surfaces the live state correctly.
    pub busy: bool,
}

impl DmaChannel {
    /// Byte size of one transfer per `CTRL.DATA_SIZE`. 0 -> 1 byte,
    /// 1 -> 2 bytes, 2 -> 4 bytes, 3 -> reserved (fallback: 4 bytes, same
    /// as pico-sdk's safety fallback).
    #[inline]
    fn transfer_size(&self) -> u32 {
        match (self.ctrl & CTRL_DATA_SIZE_MASK) >> CTRL_DATA_SIZE_SHIFT {
            0 => 1,
            1 => 2,
            2 => 4,
            _ => 4,
        }
    }

    /// `CHAIN_TO` field — index of the channel to enable when this one
    /// completes. Self-chain means "no chain" per datasheet.
    #[inline]
    fn chain_to(&self) -> u32 {
        (self.ctrl & CTRL_CHAIN_TO_MASK) >> CTRL_CHAIN_TO_SHIFT
    }

    /// `TREQ_SEL` field — DREQ source index. `0x3F` is `FORCE` (always
    /// ready).
    #[inline]
    fn treq_sel(&self) -> u8 {
        ((self.ctrl & CTRL_TREQ_SEL_MASK) >> CTRL_TREQ_SEL_SHIFT) as u8
    }

    /// `RING_SIZE` field — number of low-order address bits to preserve
    /// when wrapping. 0 means "no ring". A value of N means the ring is
    /// `1 << N` bytes wide.
    #[inline]
    fn ring_size(&self) -> u32 {
        (self.ctrl & CTRL_RING_SIZE_MASK) >> CTRL_RING_SIZE_SHIFT
    }

    /// `RING_SEL`: 0 -> ring the read address, 1 -> ring the write
    /// address.
    #[inline]
    fn ring_on_write(&self) -> bool {
        (self.ctrl & CTRL_RING_SEL) != 0
    }

    /// Ring-wrap `addr` after bumping by `size` — preserves the top bits
    /// outside the ring mask and wraps the low bits within
    /// `(1 << ring)` bytes.
    #[inline]
    fn apply_ring(addr: u32, ring: u32, size: u32) -> u32 {
        if ring == 0 {
            return addr.wrapping_add(size);
        }
        let mask = (1u32 << ring).wrapping_sub(1);
        let base = addr & !mask;
        let low = (addr.wrapping_add(size)) & mask;
        base | low
    }
}

/// Test-only observable: snapshot of the most recent transfer issued
/// on a channel. Updated atomically with the bus write inside
/// [`Dma::issue_transfer`]. Behind `#[cfg(feature = "testing")]` so
/// release builds don't ship the field.
///
/// The reader contract: pair `push_count` (a monotonically-increasing
/// edge detector) with `last_src_addr` (the source address that
/// produced the just-completed transfer, captured before any
/// post-transfer increment). Sampling both in the same observation
/// window yields the address that actually fed the most recent
/// transfer — equivalent to the `(ch1_pushes, last_pushed_read_addr)`
/// pair that the harness-side `GlueDma` previously exposed.
///
/// Default is `{ push_count: 0, last_src_addr: 0 }`; meaningful only
/// after `push_count > 0`.
#[cfg(feature = "testing")]
#[derive(Clone, Copy, Debug, Default)]
pub struct ChannelTransferEvent {
    /// Monotonic count of completed transfers on this channel since
    /// `Dma::reset` (or construction). Wraps on overflow.
    pub push_count: u32,
    /// `read_addr` value used as the source of the most recent
    /// transfer, captured BEFORE the post-transfer increment-read
    /// step bumps it. Reading this paired with a `push_count` edge
    /// yields the address that produced the byte/word now visible
    /// downstream.
    pub last_src_addr: u32,
}

/// DMA controller state — 16 channels + global registers.
pub struct Dma {
    channels: [DmaChannel; NUM_CHANNELS],
    /// Raw interrupt status. Bit N latches when channel N's
    /// `trans_count` hits 0 (or via `INTF` force). Low 16 bits used.
    intr: u32,
    inte0: u32,
    inte1: u32,
    intf0: u32,
    intf1: u32,
    /// INTE2/INTF2/INTE3/INTF3: added by Residual C.2.1.  Reads of
    /// INTS2/INTS3 return `(intr | intfN) & inteN` so firmware's
    /// read-modify-write sequences round-trip correctly.
    inte2: u32,
    inte3: u32,
    intf2: u32,
    intf3: u32,
    timer: [u32; 4],
    /// Fractional accumulators for the four DMA-internal pacing timers
    /// (TREQ 59..62). Each timer's register is `X[31:16]:Y[15:0]`;
    /// every `tick()` adds X to the accumulator and fires when it >= Y.
    timer_accum: [u32; 4],
    /// One-cycle assertion flags set by the timer accumulator overflow.
    /// Consumed by the DREQ readiness check during the same `tick()`.
    timer_dreq_asserted: [bool; 4],
    sniff_ctrl: u32,
    sniff_data: u32,
    /// Warn-once latch for `CHn_CTRL.BSWAP` bit set on a channel
    /// (HLD V5 §4.A2 site 1). Byte-swap is not modelled.
    warned_bswap: [bool; NUM_CHANNELS],
    /// Warn-once latch for `CHn_CTRL.HIGH_PRIORITY` bit set on a channel
    /// (HLD V5 §4.A2 site 4). Two-tier priority is not modelled.
    warned_high_priority: [bool; NUM_CHANNELS],
    /// Warn-once latch for `SNIFF_CTRL.EN` first set (HLD V5 §4.A2 site 2).
    warned_sniff_ctrl_en: bool,
    /// Warn-once latch for first `SNIFF_DATA` write (HLD V5 §4.A2 site 3).
    warned_sniff_data: bool,
    /// Test-only per-channel observable. Updated inside
    /// [`Self::issue_transfer`] atomically with the bus write that
    /// completes a transfer. See [`ChannelTransferEvent`] for the
    /// reader contract; this exists solely so harness oracles can
    /// observe push edges + the source address that produced each
    /// push without polling MMIO between cycles (which races the
    /// in-flight DMA pipeline). Gated behind `testing` so release
    /// crates don't carry the bookkeeping.
    #[cfg(feature = "testing")]
    last_transfer_event: [ChannelTransferEvent; NUM_CHANNELS],
}

impl Default for Dma {
    fn default() -> Self {
        Self::new()
    }
}

impl Dma {
    /// Construct a DMA controller at power-on defaults.
    pub fn new() -> Self {
        Self {
            channels: [DmaChannel::default(); NUM_CHANNELS],
            intr: 0,
            inte0: 0,
            inte1: 0,
            intf0: 0,
            intf1: 0,
            inte2: 0,
            inte3: 0,
            intf2: 0,
            intf3: 0,
            timer: [0; 4],
            timer_accum: [0; 4],
            timer_dreq_asserted: [false; 4],
            sniff_ctrl: 0,
            sniff_data: 0,
            warned_bswap: [false; NUM_CHANNELS],
            warned_high_priority: [false; NUM_CHANNELS],
            warned_sniff_ctrl_en: false,
            warned_sniff_data: false,
            #[cfg(feature = "testing")]
            last_transfer_event: [ChannelTransferEvent {
                push_count: 0,
                last_src_addr: 0,
            }; NUM_CHANNELS],
        }
    }

    /// Reset all state to power-on defaults.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// True iff no channel is currently transferring (no `BUSY`) and no
    /// IRQ is latched.
    #[inline]
    pub fn is_idle(&self) -> bool {
        !self.channels.iter().any(|c| c.busy) && self.intr == 0
    }

    /// True iff `tick()` could observably change state on this cycle.
    ///
    /// Used by `Bus::tick_peripherals` to skip the per-sysclk
    /// `tick_dma` loop entirely when nothing is going to advance —
    /// closes the remaining 5x perf gap from Stage 4 of the DMA
    /// pacing HLD (2026.05.06 §4.5).
    ///
    /// Returns `true` if either:
    /// * any channel currently has `BUSY` set (an in-flight transfer
    ///   that needs to advance), OR
    /// * any of the four pacing timers is programmed (`X != 0` AND
    ///   `Y != 0` in `Xreg = X[31:16]:Y[15:0]`) so its accumulator
    ///   must keep ticking even with no channel armed against it.
    ///
    /// Note: deliberately weaker than `!is_idle()` — `is_idle` also
    /// gates on `intr == 0`, but a latched IRQ does not need
    /// per-cycle advancement to keep state consistent. The bus-level
    /// fast path can safely skip ticking in that case.
    #[inline]
    pub fn needs_tick(&self) -> bool {
        if self.channels.iter().any(|c| c.busy) {
            return true;
        }
        self.timer.iter().any(|&reg| {
            let x = (reg >> 16) & 0xFFFF;
            let y = reg & 0xFFFF;
            x != 0 && y != 0
        })
    }

    /// Borrow a channel read-only (exposed for tests / observability).
    pub fn channel(&self, i: usize) -> &DmaChannel {
        &self.channels[i]
    }

    /// Test-only: snapshot the most recent transfer-completion event
    /// for channel `ch_idx`. Returns
    /// `ChannelTransferEvent { push_count: 0, last_src_addr: 0 }`
    /// before the channel has issued any transfers.
    ///
    /// The returned `(push_count, last_src_addr)` pair is updated
    /// atomically with the bus write inside [`Self::issue_transfer`];
    /// callers that observe a `push_count` edge see the source
    /// address that fed exactly that transfer (as opposed to whatever
    /// `CH_n.READ_ADDR` MMIO holds by the time of observation, which
    /// in chained pumps may have already advanced).
    ///
    /// Gated behind `testing` so release builds don't expose the
    /// per-channel bookkeeping (matches the [`crate::Emulator::
    /// inject_panic_for_testing`] precedent).
    #[cfg(feature = "testing")]
    pub fn channel_transfer_event(&self, ch_idx: usize) -> ChannelTransferEvent {
        self.last_transfer_event[ch_idx]
    }

    /// Current raw interrupt-status register.
    #[inline]
    pub fn intr(&self) -> u32 {
        self.intr
    }

    // -------------------------------------------------------------
    // Register dispatch
    // -------------------------------------------------------------

    /// Read a DMA register at the given 4 KB-relative offset.
    ///
    /// CTRL reads return the stored CTRL value OR'd with `BUSY` from
    /// the live `channel.busy` flag — firmware polls this to determine
    /// when a transfer has completed.
    pub fn read32(&self, offset: u32) -> u32 {
        if offset < (NUM_CHANNELS as u32) * 0x40 {
            let ch_idx = (offset / 0x40) as usize;
            let inner = offset % 0x40;
            return self.channel_read32(ch_idx, inner);
        }
        match offset {
            REG_INTR => self.intr,
            REG_INTE0 => self.inte0,
            REG_INTE1 => self.inte1,
            REG_INTF0 => self.intf0,
            REG_INTF1 => self.intf1,
            REG_INTS0 => (self.intr | self.intf0) & self.inte0,
            REG_INTS1 => (self.intr | self.intf1) & self.inte1,
            // IRQ2/IRQ3 (Residual C.2.1).  Read-side produces the
            // same pattern as IRQ0/IRQ1 so firmware read-modify-write
            // sequences round-trip.
            REG_INTE2 => self.inte2,
            REG_INTE3 => self.inte3,
            REG_INTF2 => self.intf2,
            REG_INTF3 => self.intf3,
            REG_INTS2 => (self.intr | self.intf2) & self.inte2,
            REG_INTS3 => (self.intr | self.intf3) & self.inte3,
            REG_TIMER0 => self.timer[0],
            REG_TIMER1 => self.timer[1],
            REG_TIMER2 => self.timer[2],
            REG_TIMER3 => self.timer[3],
            REG_MULTI_CHAN_TRIGGER => 0, // W1-only side-effect
            REG_SNIFF_CTRL => self.sniff_ctrl,
            REG_SNIFF_DATA => self.sniff_data,
            REG_FIFO_LEVELS => 0, // no bus-FIFO model
            REG_CHAN_ABORT => {
                // Datasheet: reads return 1 while abort is in progress;
                // we abort immediately so the field reads 0.
                0
            }
            REG_N_CHANNELS => NUM_CHANNELS as u32,
            _ => {
                if offset >= CH_DBG_CTDREQ_OFFSET
                    && offset < CH_DBG_CTDREQ_OFFSET + 0x40 * NUM_CHANNELS as u32
                {
                    let ch = ((offset - CH_DBG_CTDREQ_OFFSET) / 0x40) as usize;
                    let inner = (offset - CH_DBG_CTDREQ_OFFSET) % 0x40;
                    match inner {
                        0 => 0, // CTDREQ — not modelled
                        4 => self.channels[ch].trans_count,
                        _ => 0,
                    }
                } else {
                    0
                }
            }
        }
    }

    /// Write a DMA register.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        if offset < (NUM_CHANNELS as u32) * 0x40 {
            let ch_idx = (offset / 0x40) as usize;
            let inner = offset % 0x40;
            self.channel_write32(ch_idx, inner, value, alias);
            return;
        }
        match offset {
            REG_INTR => {
                // INTR is primarily RO but W1C per datasheet.
                let stored = apply_alias(self.intr, value, alias);
                self.intr &= !stored;
            }
            REG_INTE0 => self.inte0 = apply_alias(self.inte0, value, alias) & CHANNEL_MASK,
            REG_INTE1 => self.inte1 = apply_alias(self.inte1, value, alias) & CHANNEL_MASK,
            REG_INTF0 => self.intf0 = apply_alias(self.intf0, value, alias) & CHANNEL_MASK,
            REG_INTF1 => self.intf1 = apply_alias(self.intf1, value, alias) & CHANNEL_MASK,
            REG_INTS0 => {
                // INTS is W1C on INTR bits (datasheet §12.6.6).
                let bits = apply_alias(0, value, alias);
                self.intr &= !bits;
            }
            REG_INTS1 => {
                let bits = apply_alias(0, value, alias);
                self.intr &= !bits;
            }
            // IRQ2/IRQ3 storage-only writes (Residual C.2.1).  Same RMW
            // semantics as IRQ0/IRQ1 so firmware round-trips but no
            // NVIC fan-out (tracked in tech_debt.md).
            REG_INTE2 => self.inte2 = apply_alias(self.inte2, value, alias) & CHANNEL_MASK,
            REG_INTE3 => self.inte3 = apply_alias(self.inte3, value, alias) & CHANNEL_MASK,
            REG_INTF2 => self.intf2 = apply_alias(self.intf2, value, alias) & CHANNEL_MASK,
            REG_INTF3 => self.intf3 = apply_alias(self.intf3, value, alias) & CHANNEL_MASK,
            REG_INTS2 => {
                let bits = apply_alias(0, value, alias);
                self.intr &= !bits;
            }
            REG_INTS3 => {
                let bits = apply_alias(0, value, alias);
                self.intr &= !bits;
            }
            REG_TIMER0 => self.timer[0] = apply_alias(self.timer[0], value, alias),
            REG_TIMER1 => self.timer[1] = apply_alias(self.timer[1], value, alias),
            REG_TIMER2 => self.timer[2] = apply_alias(self.timer[2], value, alias),
            REG_TIMER3 => self.timer[3] = apply_alias(self.timer[3], value, alias),
            REG_MULTI_CHAN_TRIGGER => {
                let mask = apply_alias(0, value, alias) & CHANNEL_MASK;
                for i in 0..NUM_CHANNELS {
                    if (mask >> i) & 1 != 0 {
                        self.trigger_channel(i);
                    }
                }
            }
            REG_SNIFF_CTRL => {
                let new = apply_alias(self.sniff_ctrl, value, alias);
                // Warn-once on first SNIFF_CTRL.EN set (HLD V5 §4.A2 site 2).
                if (new & 1) != 0 && !self.warned_sniff_ctrl_en {
                    self.warned_sniff_ctrl_en = true;
                    tracing::warn!("DMA SNIFF_CTRL.EN set; CRC sniff not modelled");
                }
                self.sniff_ctrl = new;
            }
            REG_SNIFF_DATA => {
                // Warn-once on first SNIFF_DATA write (HLD V5 §4.A2 site 3).
                if !self.warned_sniff_data {
                    self.warned_sniff_data = true;
                    tracing::warn!("DMA SNIFF_DATA written; CRC sniff not modelled");
                }
                self.sniff_data = apply_alias(self.sniff_data, value, alias);
            }
            REG_CHAN_ABORT => {
                let mask = apply_alias(0, value, alias) & CHANNEL_MASK;
                for i in 0..NUM_CHANNELS {
                    if (mask >> i) & 1 != 0 {
                        self.channels[i].busy = false;
                    }
                }
            }
            _ => {}
        }
    }

    // -------------------------------------------------------------
    // Channel register dispatch
    // -------------------------------------------------------------

    fn channel_read32(&self, ch_idx: usize, inner: u32) -> u32 {
        let ch = &self.channels[ch_idx];
        let ctrl_image = (ch.ctrl & !CTRL_BUSY) | (if ch.busy { CTRL_BUSY } else { 0 });
        match inner {
            CH_READ_ADDR | CH_AL1_READ_ADDR | CH_AL2_READ_ADDR | CH_AL3_READ_ADDR_TRIG => {
                ch.read_addr
            }
            CH_WRITE_ADDR | CH_AL1_WRITE_ADDR_TRIG | CH_AL2_WRITE_ADDR | CH_AL3_WRITE_ADDR => {
                ch.write_addr
            }
            CH_TRANS_COUNT | CH_AL1_TRANS_COUNT | CH_AL2_TRANS_COUNT_TRIG | CH_AL3_TRANS_COUNT => {
                ch.trans_count
            }
            CH_CTRL_TRIG | CH_AL1_CTRL | CH_AL2_CTRL | CH_AL3_CTRL => ctrl_image,
            _ => 0,
        }
    }

    fn channel_write32(&mut self, ch_idx: usize, inner: u32, value: u32, alias: u32) {
        match inner {
            CH_READ_ADDR | CH_AL1_READ_ADDR | CH_AL2_READ_ADDR => {
                let new = apply_alias(self.channels[ch_idx].read_addr, value, alias);
                self.channels[ch_idx].read_addr = new;
            }
            CH_AL3_READ_ADDR_TRIG => {
                let new = apply_alias(self.channels[ch_idx].read_addr, value, alias);
                self.channels[ch_idx].read_addr = new;
                self.trigger_channel(ch_idx);
            }
            CH_WRITE_ADDR | CH_AL2_WRITE_ADDR | CH_AL3_WRITE_ADDR => {
                let new = apply_alias(self.channels[ch_idx].write_addr, value, alias);
                self.channels[ch_idx].write_addr = new;
            }
            CH_AL1_WRITE_ADDR_TRIG => {
                let new = apply_alias(self.channels[ch_idx].write_addr, value, alias);
                self.channels[ch_idx].write_addr = new;
                self.trigger_channel(ch_idx);
            }
            CH_TRANS_COUNT | CH_AL1_TRANS_COUNT | CH_AL3_TRANS_COUNT => {
                let new = apply_alias(self.channels[ch_idx].trans_count, value, alias);
                self.channels[ch_idx].trans_count = new;
                self.channels[ch_idx].trans_count_reload = new;
            }
            CH_AL2_TRANS_COUNT_TRIG => {
                let new = apply_alias(self.channels[ch_idx].trans_count, value, alias);
                self.channels[ch_idx].trans_count = new;
                self.channels[ch_idx].trans_count_reload = new;
                self.trigger_channel(ch_idx);
            }
            CH_CTRL_TRIG => {
                let new = apply_alias(self.channels[ch_idx].ctrl, value, alias);
                self.channels[ch_idx].ctrl = new & CTRL_WRITABLE_MASK;
                self.check_inert_ctrl_bits(ch_idx, new);
                self.trigger_channel(ch_idx);
            }
            CH_AL1_CTRL | CH_AL2_CTRL | CH_AL3_CTRL => {
                let new = apply_alias(self.channels[ch_idx].ctrl, value, alias);
                self.channels[ch_idx].ctrl = new & CTRL_WRITABLE_MASK;
                self.check_inert_ctrl_bits(ch_idx, new);
            }
            _ => {}
        }
    }

    /// Warn-once per channel when firmware sets `CTRL.BSWAP` (bit 24)
    /// or `CTRL.HIGH_PRIORITY` (bit 1) — inert registers inside the
    /// modelled DMA peripheral (HLD V5 §4.A2 sites 1 & 4). Byte-swap
    /// and two-tier priority are not modelled; storage round-trips
    /// but transfer behaviour ignores these bits.
    #[inline]
    fn check_inert_ctrl_bits(&mut self, ch_idx: usize, ctrl: u32) {
        if (ctrl & CTRL_BSWAP) != 0 && !self.warned_bswap[ch_idx] {
            self.warned_bswap[ch_idx] = true;
            tracing::warn!(
                channel = ch_idx,
                "DMA CHn_CTRL.BSWAP set; byte-swap not modelled"
            );
        }
        if (ctrl & CTRL_HIGH_PRIORITY) != 0 && !self.warned_high_priority[ch_idx] {
            self.warned_high_priority[ch_idx] = true;
            tracing::warn!(
                channel = ch_idx,
                "DMA CHn_CTRL.HIGH_PRIORITY set; two-tier priority not modelled"
            );
        }
    }

    /// Arm a channel: if `CTRL.EN` is set and `TRANS_COUNT > 0`, mark
    /// `BUSY`. Otherwise no-op.
    fn trigger_channel(&mut self, ch_idx: usize) {
        let ch = &mut self.channels[ch_idx];
        if (ch.ctrl & CTRL_EN) == 0 {
            return;
        }
        if ch.trans_count == 0 {
            return;
        }
        ch.trans_count_reload = ch.trans_count;
        ch.busy = true;
    }

    // -------------------------------------------------------------
    // Per-cycle tick
    // -------------------------------------------------------------

    /// Advance DMA by one system clock. Fires **every** ready channel
    /// in low-to-high index order within this tick — not just the
    /// lowest-index ready one.
    ///
    /// This is the silicon-correct semantic for workloads where
    /// multiple channels are paced on a shared DREQ source (e.g.
    /// OneROM programs CH0 and CH1 with the same `TREQ_SEL=12`,
    /// `DREQ_PIO1_RX0`). Pre-Stage-7 the V1 fixed-priority
    /// (lowest-index-wins) loop `break`'d after the first ready
    /// channel; CH0 monopolised every DREQ pulse and CH1 starved
    /// indefinitely. The GlueDma harness shim was masking the
    /// regression by aborting both channels every cycle and pumping
    /// CH1 manually; deleting GlueDma in Stage 5 exposed the bug.
    ///
    /// DREQ snapshot semantics: `bus.collect_dreqs()` is taken **once**
    /// at tick start. Every channel decides "ready" against this
    /// start-of-tick state, even if an earlier channel's bus access
    /// in the same tick would have changed the underlying peripheral
    /// (e.g. CH0 draining an RX FIFO that CH1 is also paced on).
    /// `busy` is re-fetched per iteration so a chain-fired channel
    /// armed mid-tick by an earlier `issue_transfer` is observed
    /// correctly.
    ///
    /// Rationale: `wrk_journals/2026.05.06 - JRN - DMA Pacing Within
    /// Step Quantum Implementation.md` § "Phase 2 — silicon
    /// validation".
    pub fn tick(&mut self, bus: &mut Bus) {
        // DMA-internal timers (TREQ 59–62). Format: X[31:16]:Y[15:0],
        // rate = X/Y of sys_clk. Accumulator fires when accum >= Y.
        for i in 0..4 {
            let reg = self.timer[i];
            let x = (reg >> 16) & 0xFFFF;
            let y = reg & 0xFFFF;
            if x == 0 || y == 0 {
                self.timer_dreq_asserted[i] = false;
                continue;
            }
            self.timer_accum[i] += x;
            if self.timer_accum[i] >= y {
                self.timer_accum[i] -= y;
                self.timer_dreq_asserted[i] = true;
            } else {
                self.timer_dreq_asserted[i] = false;
            }
        }

        // HLD §4.5: skip arbitration when no channel is armed. Saves
        // `collect_dreqs()` + 16-channel scan on quanta with no DMA work.
        // Timers above must still tick because a future trigger arms a
        // channel observing the running accumulator.
        if self.channels.iter().all(|ch| !ch.busy) {
            return;
        }

        let dreqs = bus.collect_dreqs();
        for i in 0..NUM_CHANNELS {
            // Re-fetch `busy` each iteration: an earlier channel's
            // `issue_transfer` may have chain-armed a higher-index
            // channel (CHAIN_TO), and that newly-armed channel is
            // eligible to fire in the same tick if the start-of-tick
            // DREQ snapshot says so.
            if !self.channels[i].busy {
                continue;
            }
            let treq = self.channels[i].treq_sel();
            let ready = treq == DREQ_FORCE
                || ((DREQ_TIMER0..=DREQ_TIMER3).contains(&treq)
                    && self.timer_dreq_asserted[(treq - DREQ_TIMER0) as usize])
                || (treq < 64 && (dreqs >> treq) & 1 != 0);
            if ready {
                self.issue_transfer(i, bus);
            }
        }
    }

    fn issue_transfer(&mut self, ch_idx: usize, bus: &mut Bus) {
        let (read_addr, write_addr, size, incr_read, incr_write, ring, ring_on_write) = {
            let ch = &self.channels[ch_idx];
            (
                ch.read_addr,
                ch.write_addr,
                ch.transfer_size(),
                (ch.ctrl & CTRL_INCR_READ) != 0,
                (ch.ctrl & CTRL_INCR_WRITE) != 0,
                ch.ring_size(),
                ch.ring_on_write(),
            )
        };

        // Issue one transfer. DMA-to-DMA transfers (read_addr or
        // write_addr falling inside the DMA register aperture) must
        // bypass `bus.read*` / `bus.write*` and route through `self`
        // directly: `Bus::tick_dma` `mem::take`s `self.dma` for the
        // duration of the tick, so the bus's `DMA_BASE` dispatch arm
        // would land on the empty stand-in and the access would be
        // silently dropped (read returns 0, write goes to /dev/null).
        // Real silicon's AHB carries DMA self-accesses to the DMA
        // peripheral the same as any other master would; this branch
        // emulates that.
        let value = self.dma_routed_read(read_addr, size, bus);
        self.dma_routed_write(write_addr, value, size, bus);

        // Test-only push-event observable. Recorded inside
        // `issue_transfer` synchronously with the bus write that
        // completes the transfer, BEFORE the increment-read step
        // bumps `read_addr`. Harness oracles read the pair
        // `(push_count, last_src_addr)` to detect a push edge and
        // recover the source address atomically — a guarantee that
        // polling `CH_n.READ_ADDR` MMIO from outside the DMA cannot
        // make, because CH0 in a chained pump may already have
        // advanced READ_ADDR by the time the harness samples.
        #[cfg(feature = "testing")]
        {
            let prev = self.last_transfer_event[ch_idx].push_count;
            self.last_transfer_event[ch_idx] = ChannelTransferEvent {
                push_count: prev.wrapping_add(1),
                last_src_addr: read_addr,
            };
        }

        // Update addresses.
        let ch = &mut self.channels[ch_idx];
        if incr_read {
            ch.read_addr = if !ring_on_write {
                DmaChannel::apply_ring(read_addr, ring, size)
            } else {
                read_addr.wrapping_add(size)
            };
        }
        if incr_write {
            ch.write_addr = if ring_on_write {
                DmaChannel::apply_ring(write_addr, ring, size)
            } else {
                write_addr.wrapping_add(size)
            };
        }

        // Consume one unit of trans_count.
        ch.trans_count = ch.trans_count.saturating_sub(1);
        if ch.trans_count == 0 {
            ch.busy = false;
            if (ch.ctrl & CTRL_IRQ_QUIET) == 0 {
                self.intr |= 1u32 << ch_idx;
            }
            let chain_to = ch.chain_to() as usize;
            if chain_to != ch_idx && chain_to < NUM_CHANNELS {
                let reload = self.channels[chain_to].trans_count_reload;
                if reload > 0 {
                    self.channels[chain_to].trans_count = reload;
                }
                self.trigger_channel(chain_to);
            }
        }
    }

    /// True iff `addr` falls in the DMA register aperture decoded by
    /// `Bus::read32`/`write32`. See [`DMA_APERTURE_SIZE`] for the
    /// rationale and exact bound.
    #[inline]
    fn is_dma_aperture(addr: u32) -> bool {
        addr.wrapping_sub(DMA_BASE) < DMA_APERTURE_SIZE
    }

    /// Issue one DMA-master read of `size` bytes at `addr`. Falls back
    /// to the bus for normal addresses; routes DMA-aperture reads
    /// through `self` so a self-read observes the live DMA state, not
    /// the empty stand-in left by `Bus::tick_dma`'s `mem::take`.
    #[inline]
    fn dma_routed_read(&self, addr: u32, size: u32, bus: &mut Bus) -> u32 {
        if Self::is_dma_aperture(addr) {
            // DMA registers are 32-bit; sub-word reads pick the bytes
            // out of the underlying word the same way `Bus::read8/16`
            // do for word-only peripherals (LE byte select).
            let canonical = addr & !0x3000;
            let word_offset = canonical & 0x0000_0FFC;
            let word = self.read32(word_offset);
            match size {
                1 => (word >> ((addr & 3) * 8)) & 0xFF,
                2 => (word >> ((addr & 2) * 8)) & 0xFFFF,
                _ => word,
            }
        } else {
            match size {
                1 => bus.read8(addr, 0) as u32,
                2 => bus.read16(addr, 0) as u32,
                _ => bus.read32(addr, 0),
            }
        }
    }

    /// Issue one DMA-master write of `size` bytes (`value`) at `addr`.
    /// Falls back to the bus for normal addresses; routes DMA-aperture
    /// writes through `self` so the update lands on the live DMA, not
    /// the empty stand-in. Preserves the bus-level cross-cutting
    /// concerns (LR/SC reservation invalidation, MMIO trace) that
    /// `Bus::write*` would have applied on the silicon path.
    #[inline]
    fn dma_routed_write(&mut self, addr: u32, value: u32, size: u32, bus: &mut Bus) {
        if Self::is_dma_aperture(addr) {
            // V1 DMA-to-DMA writes are word-only — every channel
            // register is 32-bit and sub-word access on real silicon
            // is unspecified for AHB-Lite slaves. If a corpus ever
            // exercises narrow self-writes, expand here with the
            // matching RMW path the peripheral bus uses.
            debug_assert_eq!(
                size, 4,
                "DMA-to-DMA narrow transfer not modelled (size={size}, addr=0x{addr:08X})"
            );
            let canonical = addr & !0x3000;
            let offset = canonical & 0x0000_0FFF;
            let alias = (addr >> 12) & 3;
            self.write32(offset, value, alias);
            // Mirror the cross-cutting work `Bus::write32` would have
            // done before reaching the DMA aperture dispatch.
            bus.invalidate_reservation_at(addr);
            if bus.mmio_trace_enabled {
                bus.emit_mmio_trace('W', size, addr, value, 0);
            }
        } else {
            match size {
                1 => bus.write8(addr, value as u8, 0),
                2 => bus.write16(addr, value as u16, 0),
                _ => bus.write32(addr, value, 0),
            }
        }
    }

    /// OR DMA IRQ lines into the shared `CoreAtomics` wire. Call this
    /// after [`Self::tick`] so both cores' NVICs latch any just-completed
    /// transfer. Phase 3 Stage 1 migrated the storage off `Bus`.
    pub fn route_irqs(&self, atomics: &crate::threaded::CoreAtomics) {
        if (self.intr | self.intf0) & self.inte0 != 0 {
            // DMA IRQs are shared — both cores see them.
            atomics.assert_irq_shared(IRQ_DMA_IRQ_0);
        }
        if (self.intr | self.intf1) & self.inte1 != 0 {
            atomics.assert_irq_shared(IRQ_DMA_IRQ_1);
        }
        if (self.intr | self.intf2) & self.inte2 != 0 {
            atomics.assert_irq_shared(IRQ_DMA_IRQ_2);
        }
        if (self.intr | self.intf3) & self.inte3 != 0 {
            atomics.assert_irq_shared(IRQ_DMA_IRQ_3);
        }
    }
}

/// Apply one of the four RP2350 alias write semantics:
///   * alias 0 (base): plain write
///   * alias 1 (XOR): `old ^ value`
///   * alias 2 (SET): `old | value`
///   * alias 3 (CLR): `old & !value`
#[inline]
fn apply_alias(old: u32, value: u32, alias: u32) -> u32 {
    match alias {
        0 => value,
        1 => old ^ value,
        2 => old | value,
        3 => old & !value,
        _ => value,
    }
}

#[cfg(test)]
mod tests {
    //! Phase 3 DMA tests (HLD V5 §5.6).

    use super::*;

    // ----------------------------------------------------------------
    // Unit tests — field decoding & apply_ring
    // ----------------------------------------------------------------

    #[test]
    fn dma_is_idle_at_construction() {
        assert!(Dma::new().is_idle());
        assert!(Dma::default().is_idle());
    }

    #[test]
    fn n_channels_returns_sixteen() {
        let dma = Dma::new();
        assert_eq!(dma.read32(REG_N_CHANNELS), NUM_CHANNELS as u32);
    }

    #[test]
    fn ring_wrap_preserves_top_bits() {
        let ring = 4; // 1 << 4 = 16 bytes.
        let mut a = 0x2000_0000;
        for _ in 0..4 {
            a = DmaChannel::apply_ring(a, ring, 4);
        }
        assert_eq!(a, 0x2000_0000);
    }

    #[test]
    fn ring_zero_is_plain_increment() {
        assert_eq!(DmaChannel::apply_ring(0x2000_0000, 0, 4), 0x2000_0004);
    }

    #[test]
    fn chain_to_zero_means_no_chain_when_self() {
        // CHAIN_TO field = 0 — "chain to self" = no chain.
        let ch = DmaChannel {
            ctrl: 0,
            ..DmaChannel::default()
        };
        assert_eq!(ch.chain_to(), 0);
    }

    #[test]
    fn treq_force_is_sixty_three() {
        let ch = DmaChannel {
            ctrl: 0x3F << CTRL_TREQ_SEL_SHIFT,
            ..DmaChannel::default()
        };
        assert_eq!(ch.treq_sel(), DREQ_FORCE);
    }

    #[test]
    fn transfer_size_decodes_correctly() {
        // DATA_SIZE=0 -> 1 byte
        let ch = DmaChannel {
            ctrl: 0 << CTRL_DATA_SIZE_SHIFT,
            ..DmaChannel::default()
        };
        assert_eq!(ch.transfer_size(), 1);
        // DATA_SIZE=1 -> 2 bytes
        let ch = DmaChannel {
            ctrl: 1 << CTRL_DATA_SIZE_SHIFT,
            ..DmaChannel::default()
        };
        assert_eq!(ch.transfer_size(), 2);
        // DATA_SIZE=2 -> 4 bytes
        let ch = DmaChannel {
            ctrl: 2 << CTRL_DATA_SIZE_SHIFT,
            ..DmaChannel::default()
        };
        assert_eq!(ch.transfer_size(), 4);
        // DATA_SIZE=3 -> fallback 4
        let ch = DmaChannel {
            ctrl: 3 << CTRL_DATA_SIZE_SHIFT,
            ..DmaChannel::default()
        };
        assert_eq!(ch.transfer_size(), 4);
    }

    // ----------------------------------------------------------------
    // RP2350 DMA global-register offsets: regression guard.
    //
    // RP2350 §12.6.6 shifts the entire global-register block up by
    // 0x20 bytes vs RP2040 because it inserts INTE2/INTF2/INTS2 at
    // 0x424..0x42C and INTE3/INTF3/INTS3 at 0x434..0x43C.  TIMER0..3
    // therefore moves from `0x420..0x42C` (RP2040) to `0x440..0x44C`
    // (RP2350), MULTI_CHAN_TRIGGER from 0x430 to 0x450, and so on.
    //
    // Pre-fix the emulator inherited the RP2040 offsets wholesale
    // during Phase 3 (commit 4cc7906), which made `dma_timer_paced`
    // run green here while failing on silicon (BKPT timeout with
    // BUSY never clearing, because the sled's write to 0x5000_0420
    // landed in reserved padding and the real TIMER0 at 0x5000_0440
    // stayed at reset 0).  Residual C.2.1 (2026-04-17).
    // ----------------------------------------------------------------

    #[test]
    fn dma_timer0_register_is_at_rp2350_offset_not_rp2040() {
        assert_eq!(REG_TIMER0, 0x440, "TIMER0 must be at RP2350 offset 0x440");
        assert_eq!(REG_TIMER1, 0x444);
        assert_eq!(REG_TIMER2, 0x448);
        assert_eq!(REG_TIMER3, 0x44C);
        assert_eq!(REG_MULTI_CHAN_TRIGGER, 0x450);
        assert_eq!(REG_SNIFF_CTRL, 0x454);
        assert_eq!(REG_SNIFF_DATA, 0x458);
        assert_eq!(REG_FIFO_LEVELS, 0x460);
        assert_eq!(REG_CHAN_ABORT, 0x464);
        assert_eq!(REG_N_CHANNELS, 0x468);
        // New RP2350 IRQ2/IRQ3 slots: storage-only in the V1 model.
        assert_eq!(REG_INTE2, 0x424);
        assert_eq!(REG_INTF2, 0x428);
        assert_eq!(REG_INTS2, 0x42C);
        assert_eq!(REG_INTE3, 0x434);
        assert_eq!(REG_INTF3, 0x438);
        assert_eq!(REG_INTS3, 0x43C);
    }

    // ----------------------------------------------------------------
    // Integration tests (Bus-level)
    // ----------------------------------------------------------------

    /// Build CTRL as a single value. `data_size` is 0/1/2 for 1/2/4-byte.
    fn make_ctrl(
        en: bool,
        data_size: u32,
        incr_read: bool,
        incr_write: bool,
        treq_sel: u8,
        chain_to: u32,
        ring_size: u32,
        ring_sel: bool,
    ) -> u32 {
        let mut v = 0u32;
        if en {
            v |= CTRL_EN;
        }
        v |= (data_size & 0x3) << CTRL_DATA_SIZE_SHIFT;
        if incr_read {
            v |= CTRL_INCR_READ;
        }
        if incr_write {
            v |= CTRL_INCR_WRITE;
        }
        v |= (treq_sel as u32 & 0x3F) << CTRL_TREQ_SEL_SHIFT;
        v |= (chain_to & 0xF) << CTRL_CHAIN_TO_SHIFT;
        v |= (ring_size & 0xF) << CTRL_RING_SIZE_SHIFT;
        if ring_sel {
            v |= CTRL_RING_SEL;
        }
        v
    }

    /// Release DMA from RESETS so bus-level dispatch reaches the DMA.
    fn release_dma(bus: &mut Bus) {
        use crate::bus::RESET_DMA;
        // CLR alias at RESETS: offset 0x3000.
        bus.write32(0x4002_0000 + 0x3000, 1u32 << RESET_DMA, 0);
    }

    // ----------------------------------------------------------------
    // Channel config: write READ_ADDR/WRITE_ADDR/TRANS_COUNT/CTRL,
    // verify readback.
    // ----------------------------------------------------------------

    #[test]
    fn channel_register_readback() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // Write ch0 registers via DMA_BASE
        bus.write32(DMA_BASE, 0x2000_1000, 0); // READ_ADDR
        bus.write32(DMA_BASE + 0x04, 0x2000_2000, 0); // WRITE_ADDR
        bus.write32(DMA_BASE + 0x08, 42, 0); // TRANS_COUNT

        assert_eq!(bus.read32(DMA_BASE, 0), 0x2000_1000);
        assert_eq!(bus.read32(DMA_BASE + 0x04, 0), 0x2000_2000);
        assert_eq!(bus.read32(DMA_BASE + 0x08, 0), 42);
    }

    // ----------------------------------------------------------------
    // CTRL_TRIG write starts transfer (BUSY=1)
    // ----------------------------------------------------------------

    #[test]
    fn ctrl_trig_starts_transfer() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        // Seed source memory
        bus.write32(src, 0xDEAD_BEEF, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        // CTRL_TRIG: EN=1, DATA_SIZE=2 (word), INCR_READ=1, INCR_WRITE=1,
        // TREQ_SEL=63 (FORCE), CHAIN_TO=0 (self = no chain)
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Read CTRL_TRIG — BUSY should be set
        let readback = bus.read32(DMA_BASE + 0x0C, 0);
        assert_ne!(
            readback & CTRL_BUSY,
            0,
            "BUSY must be set after CTRL_TRIG write"
        );
    }

    // ----------------------------------------------------------------
    // CTRL_BUSY bit position: must be bit 26, not bit 24 (BSWAP).
    // Pins the datasheet fix so a silent revert would fail immediately.
    // ----------------------------------------------------------------

    #[test]
    fn ctrl_busy_is_at_bit_26_not_bit_24() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        bus.write32(DMA_BASE, 0x2000_0100, 0);
        bus.write32(DMA_BASE + 0x04, 0x2000_0200, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        bus.write32(DMA_BASE, 0x2000_0100, 0); // seed source addr
        // Write a dummy source word.
        bus.write32(0x2000_0100, 0xA5A5_A5A5, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0); // triggers transfer

        let raw = bus.read32(DMA_BASE + 0x0C, 0);
        // BUSY must be asserted immediately after CTRL_TRIG write.
        assert_ne!(raw & (1 << 26), 0, "BUSY is at bit 26");
        // Bit 24 is BSWAP — must not be set (we did not request byte-swap).
        assert_eq!(raw & (1 << 24), 0, "bit 24 is BSWAP, not BUSY");
    }

    // ----------------------------------------------------------------
    // One transfer per tick (mem-to-mem with DREQ_FORCE)
    // ----------------------------------------------------------------

    #[test]
    fn mem_to_mem_single_word_transfer() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0xCAFE_BABE, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Tick once — should complete the transfer
        bus.tick_dma();

        assert_eq!(bus.read32(dst, 0), 0xCAFE_BABE);
        // BUSY should be clear
        let readback = bus.read32(DMA_BASE + 0x0C, 0);
        assert_eq!(
            readback & CTRL_BUSY,
            0,
            "BUSY must clear after transfer completes"
        );
    }

    // ----------------------------------------------------------------
    // TRANS_COUNT decrement, reaches 0 -> INTR bit set
    // ----------------------------------------------------------------

    #[test]
    fn trans_count_decrement_and_intr() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        for i in 0..4u32 {
            bus.write32(src + i * 4, i + 1, 0);
        }

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 4, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Tick 4 times
        for _ in 0..4 {
            bus.tick_dma();
        }

        // Verify destination
        for i in 0..4u32 {
            assert_eq!(bus.read32(dst + i * 4, 0), i + 1, "word {i} mismatch");
        }
        // INTR bit 0 should be set
        assert_ne!(
            bus.read32(DMA_BASE + REG_INTR, 0) & 1,
            0,
            "INTR bit 0 must latch"
        );
        // BUSY should be clear
        let ctrl_read = bus.read32(DMA_BASE + 0x0C, 0);
        assert_eq!(ctrl_read & CTRL_BUSY, 0, "BUSY must be clear");
    }

    // ----------------------------------------------------------------
    // DREQ gating: channel BUSY but DREQ not asserted -> no transfer
    // ----------------------------------------------------------------

    #[test]
    fn dreq_gating_prevents_transfer_when_source_not_ready() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0xAAAA_BBBB, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        // TREQ_SEL = UART0_TX (28) — UART0 is empty so DREQ should be
        // deasserted (TX not enabled).
        let ctrl = make_ctrl(true, 2, true, true, 28, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Tick — DREQ not asserted, so no transfer should happen
        bus.tick_dma();

        assert_eq!(
            bus.read32(dst, 0),
            0,
            "no transfer should occur when DREQ is not asserted"
        );
        // Channel should still be BUSY
        let readback = bus.read32(DMA_BASE + 0x0C, 0);
        assert_ne!(readback & CTRL_BUSY, 0, "channel must remain BUSY");
    }

    // ----------------------------------------------------------------
    // Ring: RING_SIZE=4, address wraps at 16-byte boundary
    // ----------------------------------------------------------------

    #[test]
    fn ring_buffer_write_wrap() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        // 16-byte ring buffer at 0x2000_0200 (aligned to 16 bytes)
        let dst: u32 = 0x2000_0200;
        for i in 0..8u32 {
            bus.write32(src + i * 4, 0x1000 + i, 0);
        }

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 8, 0);
        // RING_SIZE=4 (16 bytes), RING_SEL=1 (ring on write address)
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 4, true);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Tick 8 times — writes wrap around the 16-byte ring
        for _ in 0..8 {
            bus.tick_dma();
        }

        // After 8 words, the write pointer should have wrapped twice.
        // The last 4 words overwrite the first 4.
        assert_eq!(bus.read32(dst, 0), 0x1004, "ring wrap word 0");
        assert_eq!(bus.read32(dst + 4, 0), 0x1005, "ring wrap word 1");
        assert_eq!(bus.read32(dst + 8, 0), 0x1006, "ring wrap word 2");
        assert_eq!(bus.read32(dst + 12, 0), 0x1007, "ring wrap word 3");
    }

    // ----------------------------------------------------------------
    // Chain: CHAIN_TO=1, channel 0 completes -> channel 1 starts
    // ----------------------------------------------------------------

    #[test]
    fn chain_trigger_activates_next_channel() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // Channel 0: copy one word from src0 to dst0, chain to ch 1
        let src0: u32 = 0x2000_0100;
        let dst0: u32 = 0x2000_0200;
        bus.write32(src0, 0xAAAA_0000, 0);

        bus.write32(DMA_BASE, src0, 0);
        bus.write32(DMA_BASE + 0x04, dst0, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        // CHAIN_TO=1 (chain to channel 1)
        let ctrl0 = make_ctrl(true, 2, true, true, 63, 1, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl0, 0);

        // Channel 1: pre-program (no trigger yet). Use TRANS_COUNT=2
        // so post-Stage-7 chain-fires-in-same-tick still leaves CH1
        // BUSY for the second-tick BUSY observation below.
        let src1: u32 = 0x2000_0300;
        let dst1: u32 = 0x2000_0400;
        for i in 0..2u32 {
            bus.write32(src1 + i * 4, 0xBBBB_1110 + i, 0);
        }

        // Use AL1_CTRL (no trigger) to write ch1 CTRL
        bus.write32(DMA_BASE + 0x40, src1, 0); // ch1 READ_ADDR
        bus.write32(DMA_BASE + 0x40 + 0x04, dst1, 0); // ch1 WRITE_ADDR
        bus.write32(DMA_BASE + 0x40 + 0x08, 2, 0); // ch1 TRANS_COUNT=2
        let ctrl1 = make_ctrl(true, 2, true, true, 63, 1, 0, false);
        bus.write32(DMA_BASE + 0x40 + 0x10, ctrl1, 0); // ch1 AL1_CTRL (no trigger)

        // Tick once: CH0 completes, chains to CH1. Stage-7 onwards CH1
        // also fires its first transfer in the same tick (low-to-high
        // iteration with start-of-tick DREQ snapshot, TREQ=63 force =
        // always ready). CH1 still has 1 transfer left → BUSY.
        bus.tick_dma();
        assert_eq!(bus.read32(dst0, 0), 0xAAAA_0000, "CH0 transfer 1 lands");
        assert_eq!(
            bus.read32(dst1, 0),
            0xBBBB_1110,
            "CH1 chain-fires its first transfer in the same tick (Stage 7 semantics)"
        );

        // ch1 should still be BUSY (1 transfer left of 2).
        let ch1_ctrl = bus.read32(DMA_BASE + 0x40 + 0x0C, 0);
        assert_ne!(
            ch1_ctrl & CTRL_BUSY,
            0,
            "channel 1 must remain BUSY (TRANS_COUNT=2, 1 left)"
        );

        // Tick again: ch1 completes its second/final transfer.
        bus.tick_dma();
        assert_eq!(bus.read32(dst1 + 4, 0), 0xBBBB_1111, "CH1 transfer 2 lands");

        // Both INTR bits should be set
        let intr = bus.read32(DMA_BASE + REG_INTR, 0);
        assert_ne!(intr & (1 << 0), 0, "ch0 INTR");
        assert_ne!(intr & (1 << 1), 0, "ch1 INTR");
    }

    // ----------------------------------------------------------------
    // CH_ABORT: write abort mask -> BUSY clears immediately
    // ----------------------------------------------------------------

    #[test]
    fn ch_abort_clears_busy() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 100, 0); // large count
        // TREQ_SEL=28 (UART0_TX) — DREQ gated so no transfers happen
        let ctrl = make_ctrl(true, 2, true, true, 28, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Verify BUSY
        let r = bus.read32(DMA_BASE + 0x0C, 0);
        assert_ne!(r & CTRL_BUSY, 0);

        // Abort ch0
        bus.write32(DMA_BASE + REG_CHAN_ABORT, 0x0001, 0);

        // BUSY should be clear
        let r = bus.read32(DMA_BASE + 0x0C, 0);
        assert_eq!(r & CTRL_BUSY, 0, "abort must clear BUSY");
    }

    // ----------------------------------------------------------------
    // IRQ routing: INTE0 bit 0 set + INTR bit 0 set -> DMA_IRQ_0
    // ----------------------------------------------------------------

    #[test]
    fn irq_routing_dma_irq0() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0x12345678, 0);

        // Enable INTE0 bit 0
        bus.write32(DMA_BASE + REG_INTE0, 0x0001, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();

        // INTS0 should be nonzero
        let ints0 = bus.read32(DMA_BASE + REG_INTS0, 0);
        assert_ne!(
            ints0, 0,
            "INTS0 must be set after transfer completes with INTE0 enabled"
        );

        // irq_pending should have DMA_IRQ_0 set
        assert_ne!(
            bus.atomics.irq_pending_load(0) & (1u64 << IRQ_DMA_IRQ_0),
            0,
            "DMA_IRQ_0 must be pending on core 0"
        );
    }

    #[test]
    fn irq_routing_dma_irq1() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0xABCD_EF01, 0);

        // Enable INTE1 bit 0
        bus.write32(DMA_BASE + REG_INTE1, 0x0001, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();

        let ints1 = bus.read32(DMA_BASE + REG_INTS1, 0);
        assert_ne!(ints1, 0, "INTS1 must be set");
        assert_ne!(
            bus.atomics.irq_pending_load(0) & (1u64 << IRQ_DMA_IRQ_1),
            0,
            "DMA_IRQ_1 must be pending"
        );
    }

    #[test]
    fn irq_routing_dma_irq2() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0xCAFE_0002, 0);

        // Enable INTE2 bit 0
        bus.write32(DMA_BASE + REG_INTE2, 0x0001, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();

        let ints2 = bus.read32(DMA_BASE + REG_INTS2, 0);
        assert_ne!(ints2, 0, "INTS2 must be set");
        assert_ne!(
            bus.atomics.irq_pending_load(0) & (1u64 << IRQ_DMA_IRQ_2),
            0,
            "DMA_IRQ_2 must be pending"
        );
    }

    #[test]
    fn irq_routing_dma_irq3() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0xCAFE_0003, 0);

        // Enable INTE3 bit 0
        bus.write32(DMA_BASE + REG_INTE3, 0x0001, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();

        let ints3 = bus.read32(DMA_BASE + REG_INTS3, 0);
        assert_ne!(ints3, 0, "INTS3 must be set");
        assert_ne!(
            bus.atomics.irq_pending_load(0) & (1u64 << IRQ_DMA_IRQ_3),
            0,
            "DMA_IRQ_3 must be pending"
        );
    }

    #[test]
    fn irq_routing_dma_all_four_lines() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0xDEAD_BEEF, 0);

        // Enable bit 0 in all four INTE registers simultaneously
        bus.write32(DMA_BASE + REG_INTE0, 0x0001, 0);
        bus.write32(DMA_BASE + REG_INTE1, 0x0001, 0);
        bus.write32(DMA_BASE + REG_INTE2, 0x0001, 0);
        bus.write32(DMA_BASE + REG_INTE3, 0x0001, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();

        let pending = bus.atomics.irq_pending_load(0);
        assert_ne!(
            pending & (1u64 << IRQ_DMA_IRQ_0),
            0,
            "DMA_IRQ_0 must be pending"
        );
        assert_ne!(
            pending & (1u64 << IRQ_DMA_IRQ_1),
            0,
            "DMA_IRQ_1 must be pending"
        );
        assert_ne!(
            pending & (1u64 << IRQ_DMA_IRQ_2),
            0,
            "DMA_IRQ_2 must be pending"
        );
        assert_ne!(
            pending & (1u64 << IRQ_DMA_IRQ_3),
            0,
            "DMA_IRQ_3 must be pending"
        );
    }

    // ----------------------------------------------------------------
    // W1C on INTR / INTS
    // ----------------------------------------------------------------

    #[test]
    fn intr_w1c_clears_bits() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0x11111111, 0);
        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);
        bus.tick_dma();

        assert_ne!(bus.read32(DMA_BASE + REG_INTR, 0) & 1, 0);
        // W1C: write 1 to bit 0 to clear
        bus.write32(DMA_BASE + REG_INTR, 1, 0);
        assert_eq!(
            bus.read32(DMA_BASE + REG_INTR, 0) & 1,
            0,
            "INTR bit 0 must be cleared"
        );
    }

    // ----------------------------------------------------------------
    // collect_dreqs: UART0 TX not full -> DREQ asserted
    // ----------------------------------------------------------------

    #[test]
    fn collect_dreqs_uart0_tx() {
        let mut bus = Bus::new();
        // Release UART0 from RESETS
        bus.write32(0x4002_0000 + 0x3000, 1u32 << crate::bus::RESET_UART0, 0);

        // Enable UART0: UARTEN=1, TXE=1 in UARTCR
        use crate::peripherals::uart::UART0_BASE;
        bus.write32(UART0_BASE + 0x030, (1 << 0) | (1 << 8), 0); // UARTEN | TXE

        let dreqs = bus.collect_dreqs();
        // UART0 TX DREQ is at bit 28 on RP2350
        assert_ne!(
            dreqs & (1u64 << crate::dreq::DREQ_UART0_TX),
            0,
            "UART0 TX DREQ must be asserted when TX FIFO has room and UART enabled"
        );
    }

    // ----------------------------------------------------------------
    // Byte-width DMA transfer
    // ----------------------------------------------------------------

    #[test]
    fn mem_to_mem_byte_transfer() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write8(src, 0xAB, 0);
        bus.write8(src + 1, 0xCD, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 2, 0);
        // DATA_SIZE=0 (byte), INCR_READ=1, INCR_WRITE=1
        let ctrl = make_ctrl(true, 0, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();
        bus.tick_dma();

        assert_eq!(bus.read8(dst, 0), 0xAB);
        assert_eq!(bus.read8(dst + 1, 0), 0xCD);
    }

    // ----------------------------------------------------------------
    // Halfword-width DMA transfer
    // ----------------------------------------------------------------

    #[test]
    fn mem_to_mem_halfword_transfer() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write16(src, 0x1234, 0);
        bus.write16(src + 2, 0x5678, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 2, 0);
        // DATA_SIZE=1 (halfword)
        let ctrl = make_ctrl(true, 1, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();
        bus.tick_dma();

        assert_eq!(bus.read16(dst, 0), 0x1234);
        assert_eq!(bus.read16(dst + 2, 0), 0x5678);
    }

    // ----------------------------------------------------------------
    // IRQ_QUIET suppresses INTR
    // ----------------------------------------------------------------

    #[test]
    fn irq_quiet_suppresses_intr() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0200;
        bus.write32(src, 0x11111111, 0);
        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let mut ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        ctrl |= CTRL_IRQ_QUIET;
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_dma();

        assert_eq!(
            bus.read32(DMA_BASE + REG_INTR, 0),
            0,
            "IRQ_QUIET must suppress INTR"
        );
    }

    // ----------------------------------------------------------------
    // Multi-word transfer (4 words, mem-to-mem, DREQ_FORCE)
    // ----------------------------------------------------------------

    #[test]
    fn dma_mem_to_mem_32bit_4_words() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0300;
        for i in 0..4u32 {
            bus.write32(src + i * 4, 0xA000_0000 + i, 0);
        }

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 4, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        for _ in 0..4 {
            bus.tick_dma();
        }

        for i in 0..4u32 {
            assert_eq!(bus.read32(dst + i * 4, 0), 0xA000_0000 + i, "word {i}");
        }
        // INTR bit 0 set
        assert_ne!(bus.read32(DMA_BASE + REG_INTR, 0) & 1, 0);
    }

    // ----------------------------------------------------------------
    // Tick ordering: DMA sees fresh peripheral state
    // ----------------------------------------------------------------

    #[test]
    fn tick_ordering_dma_after_peripherals() {
        // This test verifies the structural ordering: tick_peripherals
        // runs before tick_dma in the Emulator::step loop. We verify
        // by checking that collect_dreqs sees the FORCE bit.
        let bus = Bus::new();
        let dreqs = bus.collect_dreqs();
        assert_ne!(dreqs & (1u64 << 63), 0, "FORCE DREQ must always be set");
    }

    // ----------------------------------------------------------------
    // DMA timer pacing: TREQ_SEL=59 (TIMER0), rate X/Y = 1/10
    // ----------------------------------------------------------------

    #[test]
    fn dma_timer_paced_transfer() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0800;
        let dst: u32 = 0x2000_0900;
        for i in 0..4u32 {
            bus.write32(src + i * 4, 0xF000_0000 + i, 0);
        }

        // Residual C.2.1 regression guard: the RP2040 legacy TIMER0 offset
        // (0x420) must be inert on RP2350.  A write there must not programme
        // any pacing timer, and the canonical TIMER0 at 0x440 must still read
        // back zero until explicitly programmed.
        bus.write32(DMA_BASE + 0x420, 0xDEAD_BEEF, 0);
        assert_eq!(
            bus.read32(DMA_BASE + REG_TIMER0, 0),
            0,
            "write to RP2040-legacy offset 0x420 must not leak into RP2350 TIMER0 at 0x440"
        );

        // Program DMA TIMER0: X=1, Y=10 → fires every 10 sysclks.
        bus.write32(DMA_BASE + REG_TIMER0, (1u32 << 16) | 10, 0);
        assert_eq!(
            bus.read32(DMA_BASE + REG_TIMER0, 0),
            (1u32 << 16) | 10,
            "TIMER0 at RP2350 offset 0x440 must take the programmed value"
        );

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 4, 0);
        // TREQ_SEL=59 (0x3B) = DREQ_TIMER0
        let ctrl = make_ctrl(true, 2, true, true, 59, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // At rate 1/10, the timer fires on tick 10, 20, 30, 40.
        // After 9 ticks nothing should have transferred.
        for _ in 0..9 {
            bus.tick_dma();
        }
        assert_eq!(bus.read32(dst, 0), 0, "no transfer before first timer fire");
        let readback = bus.read32(DMA_BASE + 0x0C, 0);
        assert_ne!(readback & CTRL_BUSY, 0, "channel must still be BUSY");

        // Tick 10: first timer fire → first transfer.
        bus.tick_dma();
        assert_eq!(bus.read32(dst, 0), 0xF000_0000, "first word after tick 10");

        // Complete remaining 3 transfers (30 more ticks).
        for _ in 0..30 {
            bus.tick_dma();
        }
        for i in 0..4u32 {
            assert_eq!(bus.read32(dst + i * 4, 0), 0xF000_0000 + i, "word {i}");
        }
        // BUSY should be clear, INTR bit 0 set.
        let readback = bus.read32(DMA_BASE + 0x0C, 0);
        assert_eq!(readback & CTRL_BUSY, 0, "BUSY must clear after completion");
        assert_ne!(
            bus.read32(DMA_BASE + REG_INTR, 0) & 1,
            0,
            "INTR bit 0 must latch"
        );
    }

    // ----------------------------------------------------------------
    // CTRL field positions: regression against RP2040 bit layout.
    //
    // The RP2350 DMA CTRL register adds INCR_READ_REV [5] and
    // INCR_WRITE_REV [7] vs RP2040, shifting RING_SIZE, RING_SEL,
    // CHAIN_TO, TREQ_SEL, and IRQ_QUIET each up by 2 bits. If the
    // emulator reverts to RP2040 positions:
    //   - TREQ_SEL=63 lands at bits[20:15] instead of [22:17]
    //   - INCR_WRITE=1 at bit 5 instead of bit 6
    //   - CHAIN_TO=N at bits[14:11] instead of [16:13]
    // A CTRL value built with wrong positions would have TREQ_SEL in
    // the wrong field (e.g. 0x001F_8039 decodes to TREQ_SEL=15 on RP2350),
    // causing the channel to stall waiting for a peripheral DREQ that
    // never fires.  Concretely: BUSY never clears, any busy-poll loop spins
    // forever, which is exactly the silicon hang observed by the silicon
    // oracle on the Stage C DMA sleds.
    //
    // This test builds a correct RP2350 CTRL value via `make_ctrl` (which
    // uses the named constants), runs a 4-word mem-to-mem transfer, and
    // asserts both that BUSY clears AND that the correct data was written.
    // If the constants revert to RP2040 positions, TREQ_SEL will not be
    // FORCE and the channel will stall — making this test fail.
    // ----------------------------------------------------------------

    #[test]
    fn ctrl_field_positions_rp2350_not_rp2040() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0A00;
        let dst: u32 = 0x2000_0B00;
        for i in 0..4u32 {
            bus.write32(src + i * 4, 0xF00D_0000 + i, 0);
        }

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 4, 0);
        // TREQ_SEL=63 (FORCE), CHAIN_TO=0 (ch0=self=no chain on RP2350).
        // With correct RP2350 positions this should be 0x007E_0059.
        // With RP2040 positions it would be 0x001F_8039 — TREQ_SEL=15
        // (PIO0_RX3), which is never asserted, so BUSY would never clear.
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        assert_eq!(
            ctrl, 0x007E_0059,
            "make_ctrl must produce RP2350 field positions (not RP2040)"
        );
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // With FORCE TREQ, 4 ticks must be enough for 4 transfers.
        for _ in 0..4 {
            bus.tick_dma();
        }

        // Data must have been transferred.
        for i in 0..4u32 {
            assert_eq!(
                bus.read32(dst + i * 4, 0),
                0xF00D_0000 + i,
                "word {i} not transferred: TREQ_SEL may be wrong (RP2040 positions?)"
            );
        }
        // BUSY must clear — if TREQ was wrong, channel stalls and BUSY stays.
        let readback = bus.read32(DMA_BASE + 0x0C, 0);
        assert_eq!(
            readback & CTRL_BUSY,
            0,
            "BUSY did not clear: likely RP2040 CTRL field positions used instead of RP2350"
        );
        // INTR bit 0 must be set (transfer completed).
        assert_ne!(
            bus.read32(DMA_BASE + REG_INTR, 0) & 1,
            0,
            "INTR bit 0 must latch on completion"
        );
    }

    // ----------------------------------------------------------------
    // Inert-register warn-once tests (HLD V5 §4.A2 sites 1..=4).
    // ----------------------------------------------------------------

    use std::sync::{Arc, Mutex};
    use tracing::span::{Attributes, Id, Record};
    use tracing::{Event, Metadata, Subscriber};

    #[derive(Default)]
    struct CaptureSubscriber {
        events: Arc<Mutex<Vec<String>>>,
    }

    struct FieldRecorder(String);
    impl tracing::field::Visit for FieldRecorder {
        fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
            use std::fmt::Write;
            if !self.0.is_empty() {
                self.0.push(' ');
            }
            let _ = write!(self.0, "{}={:?}", field.name(), value);
        }
    }

    impl Subscriber for CaptureSubscriber {
        fn enabled(&self, _metadata: &Metadata<'_>) -> bool {
            true
        }
        fn new_span(&self, _span: &Attributes<'_>) -> Id {
            Id::from_u64(1)
        }
        fn record(&self, _span: &Id, _values: &Record<'_>) {}
        fn record_follows_from(&self, _span: &Id, _follows: &Id) {}
        fn event(&self, event: &Event<'_>) {
            let mut v = FieldRecorder(String::new());
            event.record(&mut v);
            let meta = event.metadata();
            let line = format!("{} {} {}", meta.level(), meta.target(), v.0);
            self.events.lock().unwrap().push(line);
        }
        fn enter(&self, _span: &Id) {}
        fn exit(&self, _span: &Id) {}
    }

    fn count_warns_containing(events: &[String], needle: &str) -> usize {
        events
            .iter()
            .filter(|line| line.starts_with("WARN"))
            .filter(|line| line.contains(needle))
            .count()
    }

    #[test]
    fn bswap_warn_fires_once_per_channel() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber {
            events: captured.clone(),
        };
        tracing::subscriber::with_default(subscriber, || {
            let mut dma = Dma::new();
            // Two BSWAP-set writes on channel 3 — one warn.
            dma.write32(3 * 0x40 + CH_AL1_CTRL, CTRL_BSWAP, 0);
            dma.write32(3 * 0x40 + CH_AL1_CTRL, CTRL_BSWAP, 0);
            // One BSWAP-set write on channel 7 — separate warn.
            dma.write32(7 * 0x40 + CH_AL1_CTRL, CTRL_BSWAP, 0);
        });
        let events = captured.lock().unwrap();
        let matches = count_warns_containing(&events, "BSWAP");
        assert_eq!(
            matches, 2,
            "expected one BSWAP warn per affected channel; got {} in {:?}",
            matches, *events
        );
    }

    #[test]
    fn high_priority_warn_fires_once_per_channel() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber {
            events: captured.clone(),
        };
        tracing::subscriber::with_default(subscriber, || {
            let mut dma = Dma::new();
            dma.write32(2 * 0x40 + CH_AL1_CTRL, CTRL_HIGH_PRIORITY, 0);
            dma.write32(2 * 0x40 + CH_AL1_CTRL, CTRL_HIGH_PRIORITY, 0);
        });
        let events = captured.lock().unwrap();
        let matches = count_warns_containing(&events, "HIGH_PRIORITY");
        assert_eq!(matches, 1, "got {:?}", *events);
    }

    #[test]
    fn sniff_ctrl_en_warn_fires_once() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber {
            events: captured.clone(),
        };
        tracing::subscriber::with_default(subscriber, || {
            let mut dma = Dma::new();
            // Two SNIFF_CTRL.EN-set writes — one warn.
            dma.write32(REG_SNIFF_CTRL, 0x1, 0);
            dma.write32(REG_SNIFF_CTRL, 0x1, 0);
        });
        let events = captured.lock().unwrap();
        let matches = count_warns_containing(&events, "SNIFF_CTRL.EN");
        assert_eq!(matches, 1, "got {:?}", *events);
    }

    #[test]
    fn sniff_data_warn_fires_once() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber {
            events: captured.clone(),
        };
        tracing::subscriber::with_default(subscriber, || {
            let mut dma = Dma::new();
            dma.write32(REG_SNIFF_DATA, 0xDEAD_BEEF, 0);
            dma.write32(REG_SNIFF_DATA, 0xCAFE_BABE, 0);
        });
        let events = captured.lock().unwrap();
        let matches = count_warns_containing(&events, "SNIFF_DATA");
        assert_eq!(matches, 1, "got {:?}", *events);
    }

    // ----------------------------------------------------------------
    // DMA pacing within a step quantum (HLD 2026.05.06 §4.1).
    //
    // `Bus::tick_peripherals(sys_clks)` advances every other peripheral
    // by `sys_clks` cycles in one batch.  Pre-fix DMA was ticked exactly
    // once regardless of `sys_clks`, capping DMA throughput at 1/quantum
    // sysclks.  These six tests gate the §3 fix: tests 1–4 must FAIL on
    // unchanged code; 5–6 must PASS as guards against future regressions.
    // ----------------------------------------------------------------

    /// Test 1: FORCE pacing throughput — N transfers in N cycles.
    /// Catches "DMA still ticked once per quantum" — the basic regression.
    #[test]
    fn tick_peripherals_drains_one_transfer_per_sysclk_force() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0300;
        for i in 0..64u32 {
            bus.write32(src + i * 4, 0xCAFE_0000 + i, 0);
        }

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 64, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_peripherals(64);

        for i in 0..64u32 {
            assert_eq!(bus.read32(dst + i * 4, 0), 0xCAFE_0000 + i, "word {i}");
        }
        assert_eq!(bus.read32(DMA_BASE + 0x0C, 0) & CTRL_BUSY, 0);
    }

    /// Test 2: Timer pacing rate accuracy.  TIMER0 X=1, Y=10 produces ~10
    /// transfers per 100 cycles.  Catches accumulator-state caching bugs
    /// (e.g. "someone hoisted timer advance outside the loop"); without
    /// the fix this completes 1 transfer in 100 cycles, not 10.
    #[test]
    fn tick_peripherals_advances_dma_timer_per_sysclk() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        bus.write32(DMA_BASE + 0x440, (1 << 16) | 10, 0); // TIMER0: X=1 Y=10

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0300;
        for i in 0..16u32 {
            bus.write32(src + i * 4, 0x1000 + i, 0);
        }
        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 16, 0);
        // TREQ_SEL=59 (0x3B) = DREQ_TIMER0
        let ctrl = make_ctrl(true, 2, true, true, 59, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_peripherals(100);

        // 100 sysclks at 1/10 rate ⇒ 10 transfers fired. 6 remaining.
        let intr = bus.read32(DMA_BASE + REG_INTR, 0);
        assert_eq!(intr & 1, 0, "channel must still be in flight");
        let busy = bus.read32(DMA_BASE + 0x0C, 0) & CTRL_BUSY;
        assert_ne!(busy, 0);
        for i in 0..10u32 {
            assert_eq!(bus.read32(dst + i * 4, 0), 0x1000 + i, "word {i}");
        }
        assert_eq!(bus.read32(dst + 10 * 4, 0), 0, "word 10 must be untouched");
    }

    /// Test 3: PIO RX pacing — DREQ feedback inside the loop.  Pre-fill
    /// PIO0 SM0 RX FIFO with 4 words (RX FIFO depth = 4 by default), arm
    /// CH0 with TREQ_SEL=DREQ_PIO0_RX0 and TRANS_COUNT=16, run
    /// `tick_peripherals(64)`.  PIO0 SM0 stays disabled (SM-disabled is
    /// the default at `PioBlock::new()`) so the FIFO does not refill.
    /// Expect exactly 4 transfers (FIFO empty after the 4th); BUSY remains.
    /// Catches `collect_dreqs` caching outside the loop — without
    /// per-cycle re-snapshot DMA would either drain past empty (reading
    /// zeros for 16 transfers) or stop on cycle 1.
    #[test]
    fn tick_peripherals_pio_rx_dreq_feedback_per_sysclk() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // Pre-fill PIO0 SM0 RX FIFO with 4 words (default depth).
        for i in 0..4u32 {
            assert!(
                bus.pio[0].push_rx(0, 0xBEEF_0000 + i),
                "RX FIFO must accept 4 words at default depth"
            );
        }

        // Source addr = PIO0 RXF0 MMIO at PIO0_BASE + 0x020 (no incr).
        // PIO0_BASE on RP2350 is 0x5020_0000 (from existing harness sleds).
        let pio0_rxf0: u32 = 0x5020_0000 + 0x020;
        let dst: u32 = 0x2000_0300;
        bus.write32(DMA_BASE, pio0_rxf0, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 16, 0);
        // TREQ_SEL=4 = DREQ_PIO0_RX0; INCR_READ=0 (sourced from FIFO MMIO).
        let ctrl = make_ctrl(true, 2, false, true, 4, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_peripherals(64);

        // Exactly 4 transfers should have happened (FIFO emptied; SM
        // disabled so no refill).
        for i in 0..4u32 {
            assert_eq!(
                bus.read32(dst + i * 4, 0),
                0xBEEF_0000 + i,
                "drained word {i}"
            );
        }
        assert_eq!(
            bus.read32(dst + 4 * 4, 0),
            0,
            "no transfer past the 4th — DREQ must gate cycle 5"
        );
        // BUSY remains: 16 - 4 = 12 transfers still pending.
        let busy = bus.read32(DMA_BASE + 0x0C, 0) & CTRL_BUSY;
        assert_ne!(busy, 0, "channel must remain BUSY (12 transfers left)");
        // INTR must NOT be latched yet.
        let intr = bus.read32(DMA_BASE + REG_INTR, 0);
        assert_eq!(intr & 1, 0, "INTR bit 0 must not be set yet");
    }

    /// Test 4: Chain trigger fires mid-quantum.  CH0 TRANS_COUNT=4
    /// CHAIN_TO=1; CH1 pre-programmed via AL1 (no trigger), TRANS_COUNT=4.
    /// Run `tick_peripherals(8)` once.  Both channels complete, both
    /// INTR bits set, both destinations populated.  Catches "chain only
    /// fires on the next quantum boundary."
    #[test]
    fn tick_peripherals_chain_fires_mid_quantum() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // CH0: copy 4 words from src0 to dst0, chain to CH1.
        let src0: u32 = 0x2000_0100;
        let dst0: u32 = 0x2000_0200;
        for i in 0..4u32 {
            bus.write32(src0 + i * 4, 0xAAAA_0000 + i, 0);
        }
        bus.write32(DMA_BASE, src0, 0);
        bus.write32(DMA_BASE + 0x04, dst0, 0);
        bus.write32(DMA_BASE + 0x08, 4, 0);
        let ctrl0 = make_ctrl(true, 2, true, true, 63, 1, 0, false);

        // CH1: pre-program via AL1 (offset 0x40 + 0x10 for AL1_CTRL — does
        // NOT trigger).  Self-chain so CH1 doesn't chain back into another
        // channel.
        let src1: u32 = 0x2000_0300;
        let dst1: u32 = 0x2000_0400;
        for i in 0..4u32 {
            bus.write32(src1 + i * 4, 0xBBBB_0000 + i, 0);
        }
        bus.write32(DMA_BASE + 0x40, src1, 0); // CH1 READ_ADDR
        bus.write32(DMA_BASE + 0x40 + 0x04, dst1, 0); // CH1 WRITE_ADDR
        bus.write32(DMA_BASE + 0x40 + 0x08, 4, 0); // CH1 TRANS_COUNT
        let ctrl1 = make_ctrl(true, 2, true, true, 63, 1, 0, false);
        bus.write32(DMA_BASE + 0x40 + 0x10, ctrl1, 0); // CH1 AL1_CTRL (no trig)

        // Now arm CH0.
        bus.write32(DMA_BASE + 0x0C, ctrl0, 0);

        // 8 sysclks ⇒ 4 (CH0) + 4 (CH1 via chain) transfers.
        bus.tick_peripherals(8);

        for i in 0..4u32 {
            assert_eq!(bus.read32(dst0 + i * 4, 0), 0xAAAA_0000 + i, "ch0 word {i}");
            assert_eq!(bus.read32(dst1 + i * 4, 0), 0xBBBB_0000 + i, "ch1 word {i}");
        }
        // Both BUSY bits clear.
        assert_eq!(bus.read32(DMA_BASE + 0x0C, 0) & CTRL_BUSY, 0, "ch0 BUSY clear");
        assert_eq!(
            bus.read32(DMA_BASE + 0x40 + 0x0C, 0) & CTRL_BUSY,
            0,
            "ch1 BUSY clear"
        );
        // Both INTR bits latched.
        let intr = bus.read32(DMA_BASE + REG_INTR, 0);
        assert_ne!(intr & (1 << 0), 0, "ch0 INTR latched");
        assert_ne!(intr & (1 << 1), 0, "ch1 INTR latched");
    }

    /// Test 5: `sys_clks=0` does nothing.
    ///
    /// Pre-fix this failed because `Bus::tick_peripherals` called
    /// `tick_dma()` unconditionally — `sys_clks=0` still issued one
    /// transfer. Post-fix the `for _ in 0..0` empty loop is what makes
    /// this assertion hold. Not a "no-op guard"; a real fix-path gate.
    #[test]
    fn tick_peripherals_zero_sysclks_does_nothing() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0300;
        bus.write32(src, 0xCAFE_BABE, 0);

        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        bus.tick_peripherals(0);

        // No transfer: dst still zero.
        assert_eq!(bus.read32(dst, 0), 0, "no transfer at sys_clks=0");
        // BUSY still set.
        assert_ne!(
            bus.read32(DMA_BASE + 0x0C, 0) & CTRL_BUSY,
            0,
            "BUSY must remain set"
        );
    }

    /// Test 6: DMA in reset is silent.  RESETS_RESET bit 2 set + armed
    /// channel + `tick_peripherals(64)` ⇒ no transfers, BUSY unchanged.
    /// Guards the `is_held_in_reset_bit` gate.
    #[test]
    fn tick_peripherals_dma_in_reset_is_silent() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        let src: u32 = 0x2000_0100;
        let dst: u32 = 0x2000_0300;
        for i in 0..4u32 {
            bus.write32(src + i * 4, 0xDEAD_0000 + i, 0);
        }
        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 4, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Verify channel is BUSY before holding DMA in reset.
        assert_ne!(
            bus.read32(DMA_BASE + 0x0C, 0) & CTRL_BUSY,
            0,
            "channel must be BUSY before reset"
        );

        // Hold DMA in reset via the SET alias (offset 0x2000) on RESETS.
        // RESETS base = 0x4002_0000; RESET_DMA = bit 2.
        bus.write32(0x4002_0000 + 0x2000, 1u32 << crate::bus::RESET_DMA, 0);

        bus.tick_peripherals(64);

        // No transfer: dst untouched.
        for i in 0..4u32 {
            assert_eq!(
                bus.read32(dst + i * 4, 0),
                0,
                "no transfer while DMA held in reset (word {i})"
            );
        }
    }

    /// Test 7: INTF0-driven force-IRQ propagates through tick_peripherals
    /// even when DMA is otherwise idle (no busy channel, no timer).
    ///
    /// Pre-`route_irqs`-hoist: the bus-level fast path skipped tick_dma,
    /// route_irqs never fired, IRQ_DMA_IRQ_0 stayed deasserted indefinitely.
    /// Post-hoist: route_irqs runs unconditionally per quantum.
    #[test]
    fn tick_peripherals_routes_force_irq_when_dma_idle() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // No channel armed, no timer programmed, but force IRQ_0 via INTF0|INTE0.
        bus.write32(DMA_BASE + REG_INTF0, 0x0001, 0);
        bus.write32(DMA_BASE + REG_INTE0, 0x0001, 0);

        // Sanity: DMA reports idle (no busy channel, no programmed timer)
        // — this is the path that previously skipped route_irqs entirely.
        assert!(
            !bus.dma.needs_tick(),
            "precondition: DMA must be idle so the fast path is exercised"
        );

        bus.tick_peripherals(64);

        // The shared IRQ line should now be asserted on both cores.
        assert_ne!(
            bus.atomics.irq_pending_load(0) & (1u64 << IRQ_DMA_IRQ_0),
            0,
            "IRQ_DMA_IRQ_0 must be asserted via INTF0 force-IRQ even when DMA is otherwise idle (core 0)"
        );
        assert_ne!(
            bus.atomics.irq_pending_load(1) & (1u64 << IRQ_DMA_IRQ_0),
            0,
            "IRQ_DMA_IRQ_0 must be asserted via INTF0 force-IRQ even when DMA is otherwise idle (core 1)"
        );
    }

    /// Stage 7: two channels paced on `DREQ_PIO0_RX0` must both fire in
    /// a single tick when the shared source asserts. Pre-Stage-7 the
    /// V1 fixed-priority (lowest-index-wins) arbitration meant CH0
    /// monopolised every DREQ pulse and CH1 starved forever — the
    /// regression that surfaced after the GlueDma cleanup against the
    /// OneROM full-system smoke (CH0 and CH1 both programmed with
    /// `TREQ_SEL=12` for `DREQ_PIO1_RX0`). Post-Stage-7 the DREQ is
    /// snapshotted at tick start and every ready channel fires in
    /// low-to-high order within the same tick.
    #[test]
    fn two_channels_share_dreq_both_fire() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // Single word in PIO0 SM0 RX FIFO is enough: CH0 drains it,
        // CH1 only uses the start-of-tick DREQ snapshot for pacing
        // and reads from a scratch SRAM word, so its bus access
        // doesn't depend on the FIFO state at the moment of issue.
        bus.pio[0].push_rx(0, 0xCAFE_F00D);

        // Scratch addresses (clear of fixture pre-fills used elsewhere).
        let scratch_src: u32 = 0x2000_0500;
        let scratch1: u32 = 0x2000_0600;
        let scratch2: u32 = 0x2000_0700;
        bus.write32(scratch_src, 0xDEAD_BEEF, 0);

        // CH0: paced on DREQ_PIO0_RX0 (TREQ=4), reads PIO0 RXF0
        // (offset 0x020 from PIO0_BASE 0x5020_0000), writes scratch1.
        // No incr (FIFO MMIO + single-word transfer).
        let pio0_rxf0: u32 = 0x5020_0000 + 0x020;
        bus.write32(DMA_BASE + 0 * 0x40 + 0x00, pio0_rxf0, 0);
        bus.write32(DMA_BASE + 0 * 0x40 + 0x04, scratch1, 0);
        bus.write32(DMA_BASE + 0 * 0x40 + 0x08, 1, 0);
        let ctrl = make_ctrl(true, 2, false, false, 4, 0, 0, false);
        bus.write32(DMA_BASE + 0 * 0x40 + 0x0C, ctrl, 0);

        // CH1: paced on DREQ_PIO0_RX0 (same TREQ=4), reads scratch_src,
        // writes scratch2.
        bus.write32(DMA_BASE + 1 * 0x40 + 0x00, scratch_src, 0);
        bus.write32(DMA_BASE + 1 * 0x40 + 0x04, scratch2, 0);
        bus.write32(DMA_BASE + 1 * 0x40 + 0x08, 1, 0);
        bus.write32(DMA_BASE + 1 * 0x40 + 0x0C, ctrl, 0);

        // ONE tick — DREQ snapshot taken at tick start should arm both.
        bus.tick_dma();

        // Both channels must have fired exactly once.
        assert_eq!(
            bus.read32(scratch1, 0),
            0xCAFE_F00D,
            "CH0 must transfer (drained the RX FIFO word)"
        );
        assert_eq!(
            bus.read32(scratch2, 0),
            0xDEAD_BEEF,
            "CH1 must transfer in the same tick (start-of-tick DREQ snapshot)"
        );
        let intr = bus.read32(DMA_BASE + REG_INTR, 0);
        assert_eq!(
            intr & 0b11,
            0b11,
            "INTR bits 0 and 1 must both latch (both channels completed in one tick)"
        );
    }

    /// Regression: a DMA-issued write whose destination is another
    /// channel's `READ_ADDR` register (i.e. inside the DMA aperture)
    /// must land on the live `Dma`, not on the empty stand-in left by
    /// `Bus::tick_dma`'s `mem::take`.
    ///
    /// The OneROM SDRR firmware idiom is precisely this: CH0 reads an
    /// address from a PIO RX FIFO and writes it to `CH1.READ_ADDR`,
    /// then CH1 reads from the address CH0 just deposited and pushes
    /// the byte to PIO2's TX FIFO. Pre-fix, the write to
    /// `0x5000_0040` (CH1.READ_ADDR) dispatched through `Bus::write32`
    /// → `self.dma.write32(...)` where `self.dma` was the empty
    /// `Dma::default()` stand-in for the duration of `dma.tick(bus)`.
    /// CH1.READ_ADDR therefore stayed at whatever the firmware
    /// originally programmed; CH1 read from the wrong place; the bug
    /// surfaced as a stuck `last_src_addr` in the OneROM full-system
    /// smoke (which only checked "land somewhere in SHADOW", not
    /// "track the address pin pattern").
    #[test]
    fn dma_to_dma_write_during_tick_lands_on_live_dma() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // The address CH0 will deposit, and the byte sitting at that
        // address. If CH1.READ_ADDR updates correctly, CH1 reads from
        // here and the scratch cell holds `expected`.
        let address_to_deposit: u32 = 0x2000_0DEA;
        let expected: u32 = 0xCAFE_F00D;
        bus.write32(address_to_deposit, expected, 0);

        // Where CH1 will write the byte it reads.
        let scratch: u32 = 0x2000_0700;
        bus.write32(scratch, 0, 0);

        // Push the address into PIO0 SM0 RX FIFO so CH0 (paced on
        // DREQ_PIO0_RX0) has something to drain.
        bus.pio[0].push_rx(0, address_to_deposit);

        // CH0: read PIO0 RXF0, write CH1.READ_ADDR
        // (DMA_BASE + 1*0x40 + 0x00 = 0x5000_0040). No incr; one
        // transfer; paced on DREQ_PIO0_RX0 (TREQ=4).
        let pio0_rxf0: u32 = 0x5020_0000 + 0x020;
        let ch1_read_addr_reg: u32 = DMA_BASE + 1 * 0x40 + 0x00;
        bus.write32(DMA_BASE + 0 * 0x40 + 0x00, pio0_rxf0, 0);
        bus.write32(DMA_BASE + 0 * 0x40 + 0x04, ch1_read_addr_reg, 0);
        bus.write32(DMA_BASE + 0 * 0x40 + 0x08, 1, 0);
        let ctrl0 = make_ctrl(true, 2, false, false, 4, 0, 0, false);
        bus.write32(DMA_BASE + 0 * 0x40 + 0x0C, ctrl0, 0);

        // CH1: read from a placeholder address, write `scratch`. The
        // placeholder is meaningfully different from
        // `address_to_deposit` so the assertion can distinguish "CH1
        // read from where CH0 deposited" from "CH1 read from its
        // original placeholder". Paced on the same DREQ; one transfer.
        let placeholder: u32 = 0x2000_0500;
        bus.write32(placeholder, 0xBAAD_BAAD, 0);
        bus.write32(DMA_BASE + 1 * 0x40 + 0x00, placeholder, 0);
        bus.write32(DMA_BASE + 1 * 0x40 + 0x04, scratch, 0);
        bus.write32(DMA_BASE + 1 * 0x40 + 0x08, 1, 0);
        let ctrl1 = make_ctrl(true, 2, false, false, 4, 0, 0, false);
        bus.write32(DMA_BASE + 1 * 0x40 + 0x0C, ctrl1, 0);

        // One tick. CH0 fires first (lowest index), updates
        // CH1.READ_ADDR; CH1 then fires using the just-written value.
        bus.tick_dma();

        // The smoking-gun assertion: CH1 must have read from the
        // address CH0 deposited, not from its original placeholder.
        assert_eq!(
            bus.read32(scratch, 0),
            expected,
            "CH1 must read from the address CH0 deposited into CH1.READ_ADDR \
             (got 0x{:08X}, expected 0x{:08X} at SRAM[0x{:08X}]). If this \
             reads as 0xBAAD_BAAD, CH0's write to CH1.READ_ADDR was \
             swallowed by the empty `Dma::default()` stand-in left by \
             `Bus::tick_dma`'s `mem::take` — the very bug this test \
             guards against.",
            bus.read32(scratch, 0),
            expected,
            address_to_deposit,
        );

        // Cross-check the live CH1.READ_ADDR observable too: it should
        // hold the deposited address (not the original placeholder).
        let ch1_read_addr_observed = bus.read32(ch1_read_addr_reg, 0);
        assert_eq!(
            ch1_read_addr_observed, address_to_deposit,
            "CH1.READ_ADDR must reflect the value CH0 wrote to it"
        );
    }

    // ----------------------------------------------------------------
    // Test-only: per-channel push-event observable.
    //
    // Replaces the old harness-side `GlueDma::ch1_pushes()` /
    // `GlueDma::last_pushed_read_addr()` pair with an in-DMA hook
    // that records the source address atomically with the bus write
    // that completes a transfer. See `ChannelTransferEvent` doc for
    // the reader contract; the harness oracle pairs `push_count` and
    // `last_src_addr` to identify which transfer fed the byte
    // currently visible downstream.
    // ----------------------------------------------------------------

    #[cfg(feature = "testing")]
    #[test]
    fn channel_transfer_event_default_is_zero() {
        let dma = Dma::new();
        for ch_idx in 0..NUM_CHANNELS {
            let ev = dma.channel_transfer_event(ch_idx);
            assert_eq!(ev.push_count, 0);
            assert_eq!(ev.last_src_addr, 0);
        }
    }

    #[cfg(feature = "testing")]
    #[test]
    fn channel_transfer_event_records_push_and_source_address() {
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // 3-word mem-to-mem transfer with INCR_READ — we want to
        // observe that `last_src_addr` is the PRE-increment address
        // for the most recent transfer.
        let src: u32 = 0x2000_1000;
        let dst: u32 = 0x2000_2000;
        for i in 0..3u32 {
            bus.write32(src + i * 4, 0xC0DE_0000 | i, 0);
        }
        bus.write32(DMA_BASE, src, 0);
        bus.write32(DMA_BASE + 0x04, dst, 0);
        bus.write32(DMA_BASE + 0x08, 3, 0);
        let ctrl = make_ctrl(true, 2, true, true, 63, 0, 0, false);
        bus.write32(DMA_BASE + 0x0C, ctrl, 0);

        // Pre-tick: still no transfers issued.
        let ev = bus.dma_channel_transfer_event(0);
        assert_eq!(ev.push_count, 0, "no transfers before tick");
        assert_eq!(ev.last_src_addr, 0);

        // First tick: one transfer issued, source = src.
        bus.tick_dma();
        let ev = bus.dma_channel_transfer_event(0);
        assert_eq!(ev.push_count, 1);
        assert_eq!(
            ev.last_src_addr, src,
            "last_src_addr must be the PRE-increment source of the transfer"
        );

        // Second tick: source = src + 4.
        bus.tick_dma();
        let ev = bus.dma_channel_transfer_event(0);
        assert_eq!(ev.push_count, 2);
        assert_eq!(ev.last_src_addr, src + 4);

        // Third tick: source = src + 8.
        bus.tick_dma();
        let ev = bus.dma_channel_transfer_event(0);
        assert_eq!(ev.push_count, 3);
        assert_eq!(ev.last_src_addr, src + 8);

        // Fourth tick: TRANS_COUNT exhausted, no transfer issued.
        bus.tick_dma();
        let ev = bus.dma_channel_transfer_event(0);
        assert_eq!(
            ev.push_count, 3,
            "push_count must not advance once TRANS_COUNT is exhausted"
        );
    }
}

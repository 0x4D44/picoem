//! RP2040 DMA controller — Phase 4 (HLD V7 §5.6).
//!
//! 12-channel DMA bus master with ring/chain/abort/DREQ matrix. Drives
//! `hello_dma` (mem → mem), `dma_uart` (DREQ + UART), and `audio_i2s`
//! (chain + ring + PIO DREQ) from the corpus.
//!
//! ### Scope (V1)
//!
//! * 12 channels (0..=11), transfer sizes 1 / 2 / 4 bytes.
//! * Per-channel registers: `CTRL`, `READ_ADDR`, `WRITE_ADDR`,
//!   `TRANS_COUNT`, plus the read-back aliases (`CTRL_TRIG`,
//!   `AL1_*`, `AL2_*`, `AL3_*`) that share state but trigger on the
//!   write to their trigger variant.
//! * `RING_SIZE` + `RING_SEL` address masking for circular buffers
//!   (`audio_i2s`).
//! * `CHAIN_TO` with full chained triggering — `TRANS_COUNT` hits 0 →
//!   enable target channel (if not self).
//! * `CH_ABORT`: writing 1-bits clears `BUSY` immediately on those
//!   channels.
//! * `INTE0`/`INTE1`/`INTS0`/`INTS1`/`INTR`: per-channel enable masks.
//!   `INTR` latches on transfer completion; `INTS0`/`INTS1` are W1C on
//!   `INTR` bits. `DMA_IRQ_0` / `DMA_IRQ_1` on NVIC lines 11 / 12.
//! * Fixed-priority arbitration: lowest channel index wins.
//!
//! ### Not in V1 (per HLD §5.6.1)
//!
//! * CRC (`SNIFF_CTRL` registers — storage-only).
//! * Sniff (`SNIFF_DATA` — storage-only).
//! * Byte-swap (`BSWAP` bit) — field stored but ignored.
//! * `HIGH_PRIORITY` two-tier arbitration.
//! * Ring across non-aligned base address.
//! * Read-error / write-error IRQs.
//!
//! ### Ordering contract
//!
//! Per V7 §5.6: peripherals tick first (produce DREQ), then `tick_dma`
//! consumes the DREQ snapshot via [`Bus::collect_dreqs`]. DMA writes
//! take effect the cycle they issue — real AHB is N+1 due to address /
//! data phases, but no corpus scenario distinguishes.

use crate::bus::Bus;
use crate::dreq::DREQ_FORCE;
use crate::irq::{IRQ_DMA_IRQ_0, IRQ_DMA_IRQ_1};

/// Total number of DMA channels on RP2040 (datasheet §2.5).
pub const NUM_CHANNELS: usize = 12;

// Per-channel register offsets inside one 0x40-byte channel stride.
// `_TRIG` variants latch the channel's state and set `BUSY = 1` on write.
// Non-`_TRIG` aliases update state without triggering.
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

// Per-channel debug registers — read-only (datasheet §2.5.7).
const CH_DBG_CTDREQ_OFFSET: u32 = 0x800;
/// Per-channel `DBG_TCR` offset (0x804 within the per-channel DBG block).
/// Kept as a named constant for datasheet fidelity even though dispatch
/// computes it as `base + 4`.
#[allow(dead_code)]
const CH_DBG_TCR_OFFSET: u32 = 0x804;

// Global registers (RP2040 datasheet §2.5.7 Table 123).
const REG_INTR: u32 = 0x400;
const REG_INTE0: u32 = 0x404;
const REG_INTF0: u32 = 0x408;
const REG_INTS0: u32 = 0x40C;
const REG_INTE1: u32 = 0x414;
const REG_INTF1: u32 = 0x418;
const REG_INTS1: u32 = 0x41C;
const REG_TIMER0: u32 = 0x420;
const REG_TIMER1: u32 = 0x424;
const REG_TIMER2: u32 = 0x428;
const REG_TIMER3: u32 = 0x42C;
const REG_MULTI_CHAN_TRIGGER: u32 = 0x430;
const REG_SNIFF_CTRL: u32 = 0x434;
const REG_SNIFF_DATA: u32 = 0x438;
const REG_FIFO_LEVELS: u32 = 0x440;
const REG_CHAN_ABORT: u32 = 0x444;
const REG_N_CHANNELS: u32 = 0x448;

// CTRL bit fields (datasheet §2.5.7 Table 126).
const CTRL_EN: u32 = 1 << 0;
/// `HIGH_PRIORITY` flag — not modelled in V1 (flat priority; HLD §5.6.1
/// "Not in V1"). Kept for datasheet fidelity / future promotion.
#[allow(dead_code)]
const CTRL_HIGH_PRIORITY: u32 = 1 << 1;
const CTRL_DATA_SIZE_SHIFT: u32 = 2;
const CTRL_DATA_SIZE_MASK: u32 = 0x3 << CTRL_DATA_SIZE_SHIFT;
const CTRL_INCR_READ: u32 = 1 << 4;
const CTRL_INCR_WRITE: u32 = 1 << 5;
const CTRL_RING_SIZE_SHIFT: u32 = 6;
const CTRL_RING_SIZE_MASK: u32 = 0xF << CTRL_RING_SIZE_SHIFT;
const CTRL_RING_SEL: u32 = 1 << 10;
const CTRL_CHAIN_TO_SHIFT: u32 = 11;
const CTRL_CHAIN_TO_MASK: u32 = 0xF << CTRL_CHAIN_TO_SHIFT;
const CTRL_TREQ_SEL_SHIFT: u32 = 15;
const CTRL_TREQ_SEL_MASK: u32 = 0x3F << CTRL_TREQ_SEL_SHIFT;
const CTRL_IRQ_QUIET: u32 = 1 << 21;
/// `BSWAP` (byte-swap) flag — not modelled in V1 (HLD §5.6.1 "Not in
/// V1"). Stored through CTRL RMW but ignored on transfer.
#[allow(dead_code)]
const CTRL_BSWAP: u32 = 1 << 22;
/// `SNIFF_EN` — not modelled in V1 (no CRC). Stored but ignored.
#[allow(dead_code)]
const CTRL_SNIFF_EN: u32 = 1 << 23;
const CTRL_BUSY: u32 = 1 << 24;
// Bits [30:25] are reserved.
const CTRL_WRITE_ERROR: u32 = 1 << 29;
const CTRL_READ_ERROR: u32 = 1 << 30;
const CTRL_AHB_ERROR: u32 = 1 << 31;
// Mask of writable bits in CTRL (everything except BUSY + the three
// error bits, which are status-only / W1C).
const CTRL_WRITABLE_MASK: u32 = !(CTRL_BUSY | CTRL_WRITE_ERROR | CTRL_READ_ERROR | CTRL_AHB_ERROR);

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
    // ---------------------------------------------------------------
    // PicoGUS DMA-dispatch diagnostic counters (HLD Rev. 1 §3).
    // Pure observation — increment beside existing logic and never
    // feed back into control flow. All default-zero so the `Copy +
    // Default` derive above keeps holding.
    // ---------------------------------------------------------------
    /// Sticky bit per PIO1 TXF target this channel has ever pointed
    /// at. Bit N set iff a WRITE_ADDR-family write (any of the four
    /// aliases) landed the post-alias `write_addr` in
    /// `0x5030_0010 + N*4`, i.e. PIO1 TXF`N`.
    pub ever_wrote_pio1_txf_mask: u8,
    /// How many times this channel's `CTRL_TRIG` (offset 0x0C) arm
    /// fired — the "CTRL write that triggers" idiom.
    pub trig_ctrl: u32,
    /// Trigger-writes via `AL1_WRITE_ADDR_TRIG` (offset 0x18).
    pub trig_write_addr: u32,
    /// Trigger-writes via `AL1_TRANS_COUNT` (offset 0x1C). Despite
    /// lacking `_TRIG` in its name this alias *does* trigger — the
    /// Phase-D bug the diagnostic is meant to surface.
    pub trig_trans_count: u32,
    /// Trigger-writes via `AL2_TRANS_COUNT_TRIG` (offset 0x24). Kept
    /// distinct from `trig_trans_count` because the two share a
    /// match arm in `channel_write32` but mean different things.
    pub trig_al2_trans: u32,
    /// Trigger-writes via `AL3_READ_ADDR_TRIG` (offset 0x3C).
    pub trig_al3_read_addr: u32,
    /// Trigger-writes via `MULTI_CHAN_TRIGGER` with this channel's
    /// bit set. Increments even when `CTRL.EN=0` (captures firmware
    /// intent).
    pub trig_multi: u32,
    /// Monotonic count of bus transfers issued for this channel
    /// (bumped at the top of `issue_transfer`).
    pub transfers_issued: u64,
    /// Sticky bitmap of TREQ indices for which this channel was
    /// `ready` at least once inside `Dma::tick`. Bit 63 is the
    /// `DREQ_FORCE` alias. Non-zero means firmware's `CTRL.TREQ_SEL`
    /// pointed at a DREQ line that the emulator asserted at least
    /// once while this channel was armed — distinguishes "DREQ seen
    /// but engine didn't serve" from "TREQ never asserted".
    pub dreq_observed_mask: u64,
}

impl DmaChannel {
    /// Byte size of one transfer per `CTRL.DATA_SIZE`. 0 → 1 byte,
    /// 1 → 2 bytes, 2 → 4 bytes, 3 → reserved (fallback: 4 bytes, same
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

    /// `RING_SEL`: 0 → ring the read address, 1 → ring the write
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

    /// Diagnostic: set the sticky `ever_wrote_pio1_txf_mask` bit
    /// corresponding to `addr` when it lies in the PIO1 TXF window
    /// (`0x5030_0010..=0x5030_001C`, word-aligned, one bit per TXF). No
    /// effect for addresses outside that window. Called on every post-
    /// `apply_alias` `write_addr` update so AL2 (SET) / AL3 (CLR)
    /// constructions are also captured.
    #[inline]
    fn mark_if_pio1_txf(&mut self, addr: u32) {
        const PIO1_TXF_BASE: u32 = 0x5030_0010;
        const PIO1_TXF_LAST: u32 = 0x5030_001C;
        if (PIO1_TXF_BASE..=PIO1_TXF_LAST).contains(&addr) && (addr & 3) == 0 {
            let n = ((addr - PIO1_TXF_BASE) >> 2) & 0x3;
            self.ever_wrote_pio1_txf_mask |= 1u8 << n;
        }
    }
}

/// DMA controller state — 12 channels + global registers.
pub struct Dma {
    channels: [DmaChannel; NUM_CHANNELS],
    /// Raw interrupt status. Bit N latches when channel N's
    /// `trans_count` hits 0 (or via `INTF` force). Low 12 bits used.
    intr: u32,
    inte0: u32,
    inte1: u32,
    intf0: u32,
    intf1: u32,
    timer: [u32; 4],
    sniff_ctrl: u32,
    sniff_data: u32,
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
            timer: [0; 4],
            sniff_ctrl: 0,
            sniff_data: 0,
        }
    }

    /// Reset all state to power-on defaults.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// True iff no channel is currently transferring (no `BUSY`) and no
    /// IRQ is latched. Consulted by the fast-path gate in
    /// [`crate::Emulator::step`] — when false, the slow path runs so
    /// `tick_dma` can issue transfers.
    #[inline]
    pub fn is_idle(&self) -> bool {
        !self.channels.iter().any(|c| c.busy) && self.intr == 0
    }

    /// Borrow a channel read-only (exposed for tests / observability).
    pub fn channel(&self, i: usize) -> &DmaChannel {
        &self.channels[i]
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
                // INTR is primarily RO but W1C per datasheet Table 123.
                let stored = apply_alias(self.intr, value, alias);
                // W1C on direct write: clear matching bits.
                self.intr &= !stored;
            }
            REG_INTE0 => self.inte0 = apply_alias(self.inte0, value, alias) & 0xFFF,
            REG_INTE1 => self.inte1 = apply_alias(self.inte1, value, alias) & 0xFFF,
            REG_INTF0 => self.intf0 = apply_alias(self.intf0, value, alias) & 0xFFF,
            REG_INTF1 => self.intf1 = apply_alias(self.intf1, value, alias) & 0xFFF,
            REG_INTS0 => {
                // INTS is W1C on INTR bits (datasheet §2.5.7).
                let bits = apply_alias(0, value, alias);
                self.intr &= !bits;
            }
            REG_INTS1 => {
                let bits = apply_alias(0, value, alias);
                self.intr &= !bits;
            }
            REG_TIMER0 => self.timer[0] = apply_alias(self.timer[0], value, alias),
            REG_TIMER1 => self.timer[1] = apply_alias(self.timer[1], value, alias),
            REG_TIMER2 => self.timer[2] = apply_alias(self.timer[2], value, alias),
            REG_TIMER3 => self.timer[3] = apply_alias(self.timer[3], value, alias),
            REG_MULTI_CHAN_TRIGGER => {
                // Write a bitmask of channels to trigger — sets BUSY on
                // each bit, only if the channel is configured (`CTRL.EN`
                // set). Ignores self-chain / write-to-self logic.
                let mask = apply_alias(0, value, alias) & 0xFFF;
                for i in 0..NUM_CHANNELS {
                    if (mask >> i) & 1 != 0 {
                        // Diagnostic counter bumps even when EN=0 —
                        // captures firmware intent regardless of
                        // whether `trigger_channel` would arm.
                        self.channels[i].trig_multi = self.channels[i].trig_multi.wrapping_add(1);
                        self.trigger_channel(i);
                    }
                }
            }
            REG_SNIFF_CTRL => self.sniff_ctrl = apply_alias(self.sniff_ctrl, value, alias),
            REG_SNIFF_DATA => self.sniff_data = apply_alias(self.sniff_data, value, alias),
            REG_CHAN_ABORT => {
                // W1 to abort. Clears BUSY on each selected channel
                // immediately.
                let mask = apply_alias(0, value, alias) & 0xFFF;
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
        // CTRL reads splice live BUSY into the stored value.
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
                self.channels[ch_idx].trig_al3_read_addr =
                    self.channels[ch_idx].trig_al3_read_addr.wrapping_add(1);
                self.trigger_channel(ch_idx);
            }
            CH_WRITE_ADDR | CH_AL2_WRITE_ADDR | CH_AL3_WRITE_ADDR => {
                let new = apply_alias(self.channels[ch_idx].write_addr, value, alias);
                self.channels[ch_idx].write_addr = new;
                self.channels[ch_idx].mark_if_pio1_txf(new);
            }
            CH_AL1_WRITE_ADDR_TRIG => {
                let new = apply_alias(self.channels[ch_idx].write_addr, value, alias);
                self.channels[ch_idx].write_addr = new;
                self.channels[ch_idx].mark_if_pio1_txf(new);
                self.channels[ch_idx].trig_write_addr =
                    self.channels[ch_idx].trig_write_addr.wrapping_add(1);
                self.trigger_channel(ch_idx);
            }
            CH_TRANS_COUNT | CH_AL3_TRANS_COUNT => {
                let new = apply_alias(self.channels[ch_idx].trans_count, value, alias);
                self.channels[ch_idx].trans_count = new;
                self.channels[ch_idx].trans_count_reload = new;
            }
            CH_AL1_TRANS_COUNT | CH_AL2_TRANS_COUNT_TRIG => {
                let new = apply_alias(self.channels[ch_idx].trans_count, value, alias);
                self.channels[ch_idx].trans_count = new;
                self.channels[ch_idx].trans_count_reload = new;
                // Split the shared arm: AL1 at 0x1C is the Phase-D
                // "triggers-despite-no-_TRIG-in-name" alias; AL2 at 0x24
                // is the canonical _TRIG variant. Both trigger, but we
                // track them separately for the diagnostic.
                if inner == CH_AL1_TRANS_COUNT {
                    self.channels[ch_idx].trig_trans_count =
                        self.channels[ch_idx].trig_trans_count.wrapping_add(1);
                } else {
                    self.channels[ch_idx].trig_al2_trans =
                        self.channels[ch_idx].trig_al2_trans.wrapping_add(1);
                }
                self.trigger_channel(ch_idx);
            }
            CH_CTRL_TRIG => {
                let new = apply_alias(self.channels[ch_idx].ctrl, value, alias);
                self.channels[ch_idx].ctrl = new & CTRL_WRITABLE_MASK;
                self.channels[ch_idx].trig_ctrl = self.channels[ch_idx].trig_ctrl.wrapping_add(1);
                self.trigger_channel(ch_idx);
            }
            CH_AL1_CTRL | CH_AL2_CTRL | CH_AL3_CTRL => {
                let new = apply_alias(self.channels[ch_idx].ctrl, value, alias);
                self.channels[ch_idx].ctrl = new & CTRL_WRITABLE_MASK;
            }
            _ => {}
        }
    }

    /// Arm a channel: if `CTRL.EN` is set and `TRANS_COUNT > 0`, mark
    /// `BUSY`. Otherwise no-op. This is the shared "write to trigger
    /// alias" path; the trigger aliases call through here.
    fn trigger_channel(&mut self, ch_idx: usize) {
        let ch = &mut self.channels[ch_idx];
        if (ch.ctrl & CTRL_EN) == 0 {
            return;
        }
        if ch.trans_count == 0 {
            return;
        }
        // Latch the reload count so chain retriggers reload from a
        // known value.
        ch.trans_count_reload = ch.trans_count;
        ch.busy = true;
    }

    // -------------------------------------------------------------
    // Per-cycle tick
    // -------------------------------------------------------------

    /// Advance DMA by one system clock. Issues at most one transfer
    /// across all channels (fixed-priority, lowest index wins).
    ///
    /// Snapshots DREQ lines before issuing any bus access so peripheral
    /// state changes produced by the transfer don't feed back into
    /// same-cycle DREQ arbitration.
    pub fn tick(&mut self, bus: &mut Bus) {
        let dreqs = bus.collect_dreqs();
        // Lowest-index channel wins arbitration.
        let mut selected: Option<usize> = None;
        for i in 0..NUM_CHANNELS {
            let ch = &self.channels[i];
            if !ch.busy {
                continue;
            }
            let treq = ch.treq_sel();
            let ready = treq == DREQ_FORCE || (treq < 64 && (dreqs >> treq) & 1 != 0);
            if ready {
                // Diagnostic: record that this channel's TREQ_SEL was
                // satisfied at least once. Kept sticky so per-channel
                // verdicts survive arbitration loss to a lower-indexed
                // peer. Set before `issue_transfer` picks one.
                self.channels[i].dreq_observed_mask |= 1u64 << treq;
                if selected.is_none() {
                    selected = Some(i);
                }
            }
        }
        let Some(idx) = selected else {
            return;
        };
        self.issue_transfer(idx, bus);
    }

    fn issue_transfer(&mut self, ch_idx: usize, bus: &mut Bus) {
        // Diagnostic — bump before the bus access so aborts / bus faults
        // mid-transfer still register the engine's intent. Pure
        // observation; does not alter control flow.
        self.channels[ch_idx].transfers_issued =
            self.channels[ch_idx].transfers_issued.wrapping_add(1);
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

        // Issue one transfer. Real AHB would split address / data
        // phases; emulator collapses into one cycle.
        let value = match size {
            1 => bus.read8(read_addr) as u32,
            2 => bus.read16(read_addr) as u32,
            _ => bus.read32(read_addr),
        };
        match size {
            1 => bus.write8(write_addr, value as u8),
            2 => bus.write16(write_addr, value as u16),
            _ => bus.write32(write_addr, value),
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
            // Latch INTR unless IRQ_QUIET is set.
            if (ch.ctrl & CTRL_IRQ_QUIET) == 0 {
                self.intr |= 1u32 << ch_idx;
            }
            // Chain-trigger target.
            let chain_to = ch.chain_to() as usize;
            if chain_to != ch_idx && chain_to < NUM_CHANNELS {
                // Refill the chain target's counter from its stored
                // reload, then arm it.
                let reload = self.channels[chain_to].trans_count_reload;
                if reload > 0 {
                    self.channels[chain_to].trans_count = reload;
                }
                self.trigger_channel(chain_to);
            }
        }
    }

    /// OR DMA IRQ lines into the bus `irq_pending` wire. Call this after
    /// [`Self::tick`] so the NVIC latches any just-completed transfer.
    pub fn route_irqs(&self, irq_pending: &mut u32) {
        if (self.intr | self.intf0) & self.inte0 != 0 {
            *irq_pending |= 1u32 << IRQ_DMA_IRQ_0;
        }
        if (self.intr | self.intf1) & self.inte1 != 0 {
            *irq_pending |= 1u32 << IRQ_DMA_IRQ_1;
        }
    }
}

/// Apply one of the four RP2040 alias write semantics:
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
    //! Phase 4 DMA tests (HLD V7 §5.6).
    //!
    //! The unit tests cover field-decoding helpers on `DmaChannel` in
    //! isolation. The integration tests construct a full `Bus` with
    //! DMA released from RESETS and exercise each V1 capability:
    //! mem → mem, ring, chain-trigger, abort, INTE/INTS routing, DREQ
    //! gating (UART TX + FORCE), and the RESETS bus-level guard.

    use super::*;
    use crate::bus::peripheral_dispatch::{RESET_DMA, RESET_TIMER, RESET_UART0, RESET_UART1};
    use crate::bus::{Bus, DMA_BASE, RESETS_BASE};
    use crate::dreq::{DREQ_FORCE, DREQ_UART0_TX};
    use crate::irq::{IRQ_DMA_IRQ_0, IRQ_DMA_IRQ_1};

    // ------------------------------------------------------------
    // Unit tests — field decoding & apply_ring
    // ------------------------------------------------------------

    #[test]
    fn stub_dma_is_idle_at_construction() {
        assert!(Dma::new().is_idle());
        assert!(Dma::default().is_idle());
    }

    #[test]
    fn n_channels_returns_twelve() {
        let dma = Dma::new();
        assert_eq!(dma.read32(REG_N_CHANNELS), NUM_CHANNELS as u32);
    }

    #[test]
    fn ring_wrap_preserves_top_bits() {
        // 16-byte ring at 0x2000_0000: after 4 transfers of 4 bytes the
        // address wraps back to the ring base (0x2000_0000).
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

    // ------------------------------------------------------------
    // Integration tests (Bus-level)
    // ------------------------------------------------------------

    /// Release the DMA and (optionally) TIMER / UART bits from RESETS
    /// so peripheral dispatch actually reaches the peripheral. DMA
    /// shares the Bus-level RESETS guard — without a CLR it's gated
    /// and every read returns 0.
    fn release_dma(bus: &mut Bus) {
        // CLR alias at RESETS: offset 0x3000.
        bus.write32(RESETS_BASE + 0x3000, 1u32 << RESET_DMA);
        // Also release TIMER (some tests observe idle flags) — harmless.
        bus.write32(RESETS_BASE + 0x3000, 1u32 << RESET_TIMER);
    }

    /// Build CTRL as a single value. `data_size` is 0/1/2 for 1/2/4-byte.
    fn make_ctrl(
        en: bool,
        incr_read: bool,
        incr_write: bool,
        data_size: u32,
        chain_to: u32,
        treq_sel: u8,
        ring_size: u32,
        ring_sel: bool,
        irq_quiet: bool,
    ) -> u32 {
        let mut c = 0u32;
        if en {
            c |= CTRL_EN;
        }
        if incr_read {
            c |= CTRL_INCR_READ;
        }
        if incr_write {
            c |= CTRL_INCR_WRITE;
        }
        c |= (data_size & 0x3) << CTRL_DATA_SIZE_SHIFT;
        c |= (chain_to & 0xF) << CTRL_CHAIN_TO_SHIFT;
        c |= ((treq_sel as u32) & 0x3F) << CTRL_TREQ_SEL_SHIFT;
        c |= (ring_size & 0xF) << CTRL_RING_SIZE_SHIFT;
        if ring_sel {
            c |= CTRL_RING_SEL;
        }
        if irq_quiet {
            c |= CTRL_IRQ_QUIET;
        }
        c
    }

    fn program_channel(
        bus: &mut Bus,
        ch: u32,
        read_addr: u32,
        write_addr: u32,
        trans_count: u32,
        ctrl: u32,
    ) {
        let base = DMA_BASE + ch * 0x40;
        bus.write32(base + CH_READ_ADDR, read_addr);
        bus.write32(base + CH_WRITE_ADDR, write_addr);
        bus.write32(base + CH_TRANS_COUNT, trans_count);
        // Non-trigger CTRL first, so a subsequent CTRL_TRIG write can
        // arm with a single definitive value.
        bus.write32(base + CH_AL1_CTRL, ctrl);
    }

    fn trigger_channel_via_ctrl_trig(bus: &mut Bus, ch: u32, ctrl: u32) {
        bus.write32(DMA_BASE + ch * 0x40 + CH_CTRL_TRIG, ctrl);
    }

    #[test]
    fn reset_holds_dma() {
        // Fresh bus: DMA is held in RESETS. Writes are dropped,
        // reads return 0 — N_CHANNELS would report 12 once released.
        let mut bus = Bus::new();
        assert_eq!(bus.read32(DMA_BASE + REG_N_CHANNELS), 0);
        release_dma(&mut bus);
        assert_eq!(bus.read32(DMA_BASE + REG_N_CHANNELS), NUM_CHANNELS as u32);
    }

    #[test]
    fn mem_to_mem_transfer() {
        // 4 words from 0x2000_0100 to 0x2000_0200 via CH0 with FORCE DREQ.
        let mut bus = Bus::new();
        release_dma(&mut bus);

        // Seed source.
        for i in 0..4u32 {
            bus.write32(0x2000_0100 + i * 4, 0xDEAD_0000 | i);
        }

        let ctrl = make_ctrl(
            true, // EN
            true, // INCR_READ
            true, // INCR_WRITE
            2,    // DATA_SIZE = 32-bit
            0,    // CHAIN_TO = self → no chain
            DREQ_FORCE, 0, // no ring
            false, false,
        );
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 4, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);

        // BUSY should now be asserted.
        assert!(bus.dma.channel(0).busy);

        // Tick the DMA 4 cycles — one transfer per cycle with FORCE.
        for _ in 0..4 {
            bus.tick_dma();
        }
        assert!(!bus.dma.channel(0).busy);
        assert!((bus.dma.intr() & 1) != 0, "INTR[0] must latch");

        // Destination mirrors source.
        for i in 0..4u32 {
            assert_eq!(
                bus.read32(0x2000_0200 + i * 4),
                0xDEAD_0000 | i,
                "word {i} mismatched",
            );
        }
    }

    #[test]
    fn ring_mode_wraps_write_address() {
        // 16-byte write ring at 0x2000_0200. Transfer 8 words (32 bytes)
        // → the low 16 bytes of the ring overwrite twice.
        let mut bus = Bus::new();
        release_dma(&mut bus);
        for i in 0..8u32 {
            bus.write32(0x2000_0100 + i * 4, i);
        }
        // RING_SIZE = 4 → 16-byte ring; RING_SEL = 1 (write ring).
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 4, true, false);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 8, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        for _ in 0..8 {
            bus.tick_dma();
        }
        // After 8 × 4-byte writes with a 16-byte ring, the final four
        // source words (4,5,6,7) overwrite the first four.
        assert_eq!(bus.read32(0x2000_0200), 4);
        assert_eq!(bus.read32(0x2000_0204), 5);
        assert_eq!(bus.read32(0x2000_0208), 6);
        assert_eq!(bus.read32(0x2000_020C), 7);
        assert!(!bus.dma.channel(0).busy);
    }

    #[test]
    fn chain_trigger_starts_target_channel() {
        // CH0 → CHAIN_TO = 1. When CH0 completes, CH1 should kick off.
        let mut bus = Bus::new();
        release_dma(&mut bus);
        bus.write32(0x2000_0100, 0xAAAA_0001);
        bus.write32(0x2000_0200, 0xBBBB_0002);

        // Pre-program CH1 (don't trigger it).
        let ctrl1 = make_ctrl(true, false, false, 2, 1, DREQ_FORCE, 0, false, false);
        program_channel(&mut bus, 1, 0x2000_0200, 0x2000_0400, 1, ctrl1);

        // CH0 with CHAIN_TO = 1.
        let ctrl0 = make_ctrl(true, false, false, 2, 1, DREQ_FORCE, 0, false, false);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0300, 1, ctrl0);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl0);

        // Tick: first cycle serves CH0 (lowest idx), second should serve CH1.
        bus.tick_dma(); // CH0 xfer — hits 0 — triggers CH1.
        assert!(!bus.dma.channel(0).busy);
        assert!(bus.dma.channel(1).busy, "chain must arm CH1");
        bus.tick_dma(); // CH1 xfer.
        assert!(!bus.dma.channel(1).busy);

        assert_eq!(bus.read32(0x2000_0300), 0xAAAA_0001);
        assert_eq!(bus.read32(0x2000_0400), 0xBBBB_0002);
    }

    #[test]
    fn chan_abort_clears_busy() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 0, false, false);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 100, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        for _ in 0..5 {
            bus.tick_dma();
        }
        assert!(bus.dma.channel(0).busy);
        // Abort channel 0.
        bus.write32(DMA_BASE + REG_CHAN_ABORT, 1);
        assert!(!bus.dma.channel(0).busy);
    }

    #[test]
    fn inte0_routes_completion_to_dma_irq_0() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        // Enable INTE0 bit 0 — CH0 completion routes to DMA_IRQ_0 (NVIC 11).
        bus.write32(DMA_BASE + REG_INTE0, 1);
        let ctrl = make_ctrl(true, false, false, 2, 0, DREQ_FORCE, 0, false, false);
        bus.write32(0x2000_0100, 0x1234_5678);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 1, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        bus.tick_dma();
        assert!((bus.irq_pending() & (1 << IRQ_DMA_IRQ_0)) != 0);
        assert_eq!(bus.irq_pending() & (1 << IRQ_DMA_IRQ_1), 0);
    }

    #[test]
    fn ints0_is_w1c_on_intr() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        // Force a completion on CH0 so INTR[0] latches.
        let ctrl = make_ctrl(true, false, false, 2, 0, DREQ_FORCE, 0, false, false);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 1, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        bus.tick_dma();
        assert_eq!(bus.read32(DMA_BASE + REG_INTR) & 1, 1);

        // W1C via INTS0.
        bus.write32(DMA_BASE + REG_INTS0, 1);
        assert_eq!(bus.read32(DMA_BASE + REG_INTR) & 1, 0);
    }

    #[test]
    fn dreq_gating_prevents_transfer_when_source_not_ready() {
        // TREQ_SEL = UART0_TX but UART0 is held in RESETS → tx_dreq=false.
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, false, false, 2, 0, DREQ_UART0_TX, 0, false, false);
        bus.write32(0x2000_0100, 0xC0FF_EE00);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 1, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        // UART0 not released — tx_dreq is false. Five DMA ticks: nothing
        // transferred, BUSY still set.
        for _ in 0..5 {
            bus.tick_dma();
        }
        assert!(bus.dma.channel(0).busy);
        assert_eq!(bus.read32(0x2000_0200), 0);

        // Release UART0 + enable it. `tx_dreq` should assert (TX FIFO empty).
        bus.write32(RESETS_BASE + 0x3000, 1u32 << RESET_UART0);
        // Enable UART: write UARTCR UARTEN (bit 0). UART0 base = 0x4003_4000;
        // UARTCR at offset 0x030.
        bus.write32(0x4003_4030, 1);
        bus.tick_dma();
        assert_eq!(bus.read32(0x2000_0200), 0xC0FF_EE00);
        assert!(!bus.dma.channel(0).busy);

        // Housekeeping: silence unused-release warning for UART1 in this test.
        let _ = RESET_UART1;
    }

    #[test]
    fn force_dreq_always_runs() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 0, false, false);
        for i in 0..2 {
            bus.write32(0x2000_0100 + i * 4, 0x1000 + i);
        }
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 2, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        for _ in 0..2 {
            bus.tick_dma();
        }
        assert_eq!(bus.read32(0x2000_0200), 0x1000);
        assert_eq!(bus.read32(0x2000_0204), 0x1001);
    }

    #[test]
    fn resets_gating_blocks_all_access() {
        let mut bus = Bus::new();
        // DMA held in reset: all reads return 0, writes are dropped.
        bus.write32(DMA_BASE + REG_INTE0, 0xFFF);
        assert_eq!(bus.read32(DMA_BASE + REG_INTE0), 0);
        // Release.
        release_dma(&mut bus);
        bus.write32(DMA_BASE + REG_INTE0, 0xFFF);
        assert_eq!(bus.read32(DMA_BASE + REG_INTE0), 0xFFF);
    }

    #[test]
    fn is_idle_transitions_with_busy() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        assert!(bus.dma.is_idle());
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 0, false, false);
        bus.write32(0x2000_0100, 0xCAFE_BABE);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 1, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        assert!(!bus.dma.is_idle(), "BUSY must close the gate");
        bus.tick_dma();
        // After completion INTR[0] is set → still not idle until ack'd.
        assert!(!bus.dma.is_idle());
        bus.write32(DMA_BASE + REG_INTS0, 1); // ack INTR[0] by writing
        // INTE0 wasn't set, so INTS0 W1C mask (value=1) clears INTR[0].
        assert!(bus.dma.is_idle());
    }

    #[test]
    fn ctrl_trig_trigger_via_ctrl_trig_alias() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 0, false, false);
        bus.write32(0x2000_0100, 0xABCD);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 1, ctrl);
        // Writing AL1_CTRL didn't trigger — BUSY still 0.
        assert!(!bus.dma.channel(0).busy);
        // Triggering via CTRL_TRIG does.
        bus.write32(DMA_BASE + CH_CTRL_TRIG, ctrl);
        assert!(bus.dma.channel(0).busy);
    }

    #[test]
    fn al3_read_addr_trig_arms_channel() {
        // Per datasheet: writing AL3_READ_ADDR_TRIG latches READ_ADDR
        // and triggers the channel atomically — common idiom for
        // chained reloader channels.
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 0, false, false);
        bus.write32(0x2000_0100, 0xF00D);
        // Pre-program everything except READ_ADDR via CTRL_TRIG path.
        bus.write32(DMA_BASE + CH_WRITE_ADDR, 0x2000_0200);
        bus.write32(DMA_BASE + CH_TRANS_COUNT, 1);
        bus.write32(DMA_BASE + CH_AL1_CTRL, ctrl);
        assert!(!bus.dma.channel(0).busy);
        // The trigger-variant write arms the channel.
        bus.write32(DMA_BASE + CH_AL3_READ_ADDR_TRIG, 0x2000_0100);
        assert!(bus.dma.channel(0).busy);
        bus.tick_dma();
        assert_eq!(bus.read32(0x2000_0200), 0xF00D);
    }

    #[test]
    fn multi_chan_trigger_arms_selected_bits() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, false, false, 2, 0, DREQ_FORCE, 0, false, false);
        for ch in 0..3 {
            program_channel(
                &mut bus,
                ch,
                0x2000_0100 + ch * 4,
                0x2000_0300 + ch * 4,
                1,
                ctrl,
            );
        }
        bus.write32(DMA_BASE + REG_MULTI_CHAN_TRIGGER, 0b111);
        assert!(bus.dma.channel(0).busy);
        assert!(bus.dma.channel(1).busy);
        assert!(bus.dma.channel(2).busy);
    }

    // ------------------------------------------------------------
    // PicoGUS DMA-dispatch diagnostic counters (HLD Rev. 1)
    // ------------------------------------------------------------

    #[test]
    fn mark_if_pio1_txf_sets_bit_for_each_txf() {
        let mut ch = DmaChannel::default();
        ch.mark_if_pio1_txf(0x5030_0010);
        assert_eq!(ch.ever_wrote_pio1_txf_mask, 0b0001, "TXF0 → bit 0");
        ch.mark_if_pio1_txf(0x5030_0014);
        assert_eq!(ch.ever_wrote_pio1_txf_mask, 0b0011, "TXF0+TXF1");
        ch.mark_if_pio1_txf(0x5030_0018);
        assert_eq!(ch.ever_wrote_pio1_txf_mask, 0b0111, "TXF0..2");
        ch.mark_if_pio1_txf(0x5030_001C);
        assert_eq!(ch.ever_wrote_pio1_txf_mask, 0b1111, "TXF0..3");
    }

    #[test]
    fn mark_if_pio1_txf_ignores_out_of_range() {
        let mut ch = DmaChannel::default();
        // Just below the window and on a non-word-aligned address.
        ch.mark_if_pio1_txf(0x5030_000F);
        ch.mark_if_pio1_txf(0x5030_0020);
        // Byte-misaligned inside the window — still not a TXF write.
        ch.mark_if_pio1_txf(0x5030_0011);
        assert_eq!(ch.ever_wrote_pio1_txf_mask, 0);
    }

    #[test]
    fn trig_counters_bump_per_alias_write() {
        // Exercise each trigger-alias write path and confirm the
        // matching per-channel counter is exactly 1. The counters bump
        // unconditionally — even when `CTRL.EN=0` — so firmware intent
        // is still captured in bring-up traces.
        let mut dma = Dma::new();

        // AL1_WRITE_ADDR_TRIG (0x18).
        dma.write32(CH_AL1_WRITE_ADDR_TRIG, 0x2000_0000, 0);
        assert_eq!(dma.channel(0).trig_write_addr, 1);

        // AL1_TRANS_COUNT (0x1C) — the Phase-D alias.
        dma.write32(CH_AL1_TRANS_COUNT, 1, 0);
        assert_eq!(dma.channel(0).trig_trans_count, 1);
        assert_eq!(dma.channel(0).trig_al2_trans, 0, "must not cross over");

        // AL2_TRANS_COUNT_TRIG (0x24).
        dma.write32(CH_AL2_TRANS_COUNT_TRIG, 1, 0);
        assert_eq!(dma.channel(0).trig_al2_trans, 1);
        assert_eq!(dma.channel(0).trig_trans_count, 1, "must not cross over");

        // CTRL_TRIG (0x0C).
        dma.write32(CH_CTRL_TRIG, 0, 0);
        assert_eq!(dma.channel(0).trig_ctrl, 1);

        // AL3_READ_ADDR_TRIG (0x3C).
        dma.write32(CH_AL3_READ_ADDR_TRIG, 0x2000_0100, 0);
        assert_eq!(dma.channel(0).trig_al3_read_addr, 1);

        // MULTI_CHAN_TRIGGER — bit 0 set bumps channel 0's counter.
        dma.write32(REG_MULTI_CHAN_TRIGGER, 1, 0);
        assert_eq!(dma.channel(0).trig_multi, 1);
    }

    #[test]
    fn pio1_txf_sticky_mask_captures_ctrl_only_channel_program() {
        // The sticky mask must catch plain (non-trigger) WRITE_ADDR
        // writes too — firmware that programs CTRL via AL1_CTRL and
        // sets WRITE_ADDR via CH_WRITE_ADDR without using the _TRIG
        // alias must still be visible as "ever targeted TXF".
        let mut dma = Dma::new();
        dma.write32(CH_WRITE_ADDR, 0x5030_0010, 0);
        assert_eq!(dma.channel(0).ever_wrote_pio1_txf_mask, 0b0001);
    }

    #[test]
    fn transfers_issued_bumps_on_issue_transfer() {
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 0, false, false);
        bus.write32(0x2000_0100, 0xBEEF);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 3, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        for _ in 0..3 {
            bus.tick_dma();
        }
        assert_eq!(bus.dma.channel(0).transfers_issued, 3);
    }

    #[test]
    fn dreq_observed_mask_captures_treq_index() {
        // FORCE TREQ = 63 → bit 63 sticks after the first ready tick.
        let mut bus = Bus::new();
        release_dma(&mut bus);
        let ctrl = make_ctrl(true, true, true, 2, 0, DREQ_FORCE, 0, false, false);
        bus.write32(0x2000_0100, 0xC0DE);
        program_channel(&mut bus, 0, 0x2000_0100, 0x2000_0200, 1, ctrl);
        trigger_channel_via_ctrl_trig(&mut bus, 0, ctrl);
        bus.tick_dma();
        assert!(
            (bus.dma.channel(0).dreq_observed_mask >> DREQ_FORCE as u64) & 1 != 0,
            "FORCE TREQ must latch bit 63"
        );

        // Non-FORCE TREQ: program CH1 with UART0_TX and release UART0
        // so `tx_dreq` asserts. After the tick, bit DREQ_UART0_TX must
        // be set in the mask.
        bus.write32(RESETS_BASE + 0x3000, 1u32 << RESET_UART0);
        bus.write32(0x4003_4030, 1); // UARTEN
        let ctrl_uart = make_ctrl(true, false, false, 2, 0, DREQ_UART0_TX, 0, false, false);
        program_channel(&mut bus, 1, 0x2000_0100, 0x2000_0300, 1, ctrl_uart);
        trigger_channel_via_ctrl_trig(&mut bus, 1, ctrl_uart);
        bus.tick_dma();
        assert!(
            (bus.dma.channel(1).dreq_observed_mask >> DREQ_UART0_TX as u64) & 1 != 0,
            "UART0_TX TREQ must latch bit {}",
            DREQ_UART0_TX,
        );
        let _ = RESET_UART1;
    }
}

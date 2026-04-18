//! Harness-side two-channel glue DMA (F.4).
//!
//! Our RP2350 emulator stubs the DMA peripheral — writes to the DMA
//! aperture (`0x5000_0000`) fall through to the peripheral-register
//! HashMap and do nothing else. OneROM's serving loop however *depends*
//! on DMA: CH0 moves an address word from `PIO1.RX0` to `CH1.READ_ADDR`,
//! CH1 reads a byte from SRAM at that address and writes it to `PIO2.TX0`.
//!
//! This module fills that gap. The harness calls [`GlueDma::tick`] once
//! per emulator cycle; the pump observes `CHn_CTRL_TRIG` writes (via
//! `bus.read32`) and latches fresh configs, then drives the two-stage
//! chain with the documented 4+4-cycle read+write latency.
//!
//! Design: `wrk_docs/2026.04.14 - HLD - OneROM Full-System Oracle.md` §6.

use mdrp2350::Bus;

/// Base address of the DMA peripheral aperture.
pub const DMA_BASE: u32 = 0x5000_0000;

/// Channel register stride.
pub const DMA_CH_STRIDE: u32 = 0x40;

pub const DMA_CH_READ_ADDR: u32 = 0x00;
pub const DMA_CH_WRITE_ADDR: u32 = 0x04;
pub const DMA_CH_TRANS_COUNT: u32 = 0x08;
pub const DMA_CH_CTRL_TRIG: u32 = 0x0C;

/// Read latency (address-fetch or SRAM-fetch stage).
pub const DMA_READ_CYCLES: u8 = 4;
/// Write latency (sink stage).
pub const DMA_WRITE_CYCLES: u8 = 4;

/// MMIO base of each PIO block (duplicated here to avoid a cross-module
/// dependency on [`crate::onerom_sync`]; the numbers are a chip-level
/// constant).
const PIO_BASES: [u32; 3] = [0x5020_0000, 0x5030_0000, 0x5040_0000];

/// Global `CHAN_ABORT` register offset inside the DMA aperture.
///
/// RP2350 §12.6.6: CHAN_ABORT is at 0x464, not 0x444 (RP2040). The block
/// shift comes from the new IRQ2/IRQ3 groups inserted at 0x424..0x42C and
/// 0x434..0x43C — see Residual C.2.1 (2026-04-17).
const DMA_CHAN_ABORT: u32 = 0x464;

/// Abort mask: CH0 + CH1 — the two channels the glue pump owns.
const GLUE_DMA_CHAN_MASK: u32 = (1 << 0) | (1 << 1);

/// Rx FIFO 0 offset inside a PIO block. Reading it **pops**.
const PIO_RXF0: u32 = 0x020;
/// Tx FIFO 0 offset — writes push into SM0's TX FIFO. Only used by
/// legacy tests that target SM0 directly; the production path now
/// routes per-channel via `decode_pio_tx_addr`.
#[cfg_attr(not(test), allow(dead_code))]
const PIO_TXF0: u32 = 0x010;
/// FSTAT register.
const PIO_FSTAT: u32 = 0x004;

// FSTAT bit positions. Each is an LSB index: add the SM index (0..3) to
// get the actual bit number. See RP2350 datasheet §12.6.4.6.
const FSTAT_TXFULL_SM0:  u32 = 16;
const FSTAT_RXEMPTY_SM0: u32 = 8;
/// Only used by unit tests that observe the TX FIFO non-empty flag.
#[cfg_attr(not(test), allow(dead_code))]
const FSTAT_TXEMPTY_SM0: u32 = 24;

/// Last-seen trigger snapshot for one channel. `None` before we've seen
/// a trigger value (i.e. either pre-sync or config still at reset).
#[derive(Clone, Copy, Debug)]
struct ChannelCfg {
    read_addr: u32,
    write_addr: u32,
    trans_count: u32,
    ctrl: u32,
    armed: bool,
    last_trig: u32,
}

impl ChannelCfg {
    fn new() -> Self {
        Self {
            read_addr: 0,
            write_addr: 0,
            trans_count: 0,
            ctrl: 0,
            armed: false,
            last_trig: 0,
        }
    }
}

/// Two-channel glue DMA pump. Stores pipeline state for CH0 and CH1.
#[derive(Debug)]
pub struct GlueDma {
    /// Per-channel config latches.
    ch: [ChannelCfg; 2],

    /// CH0 pending address read from PIO1 RX0. While non-zero, `ch0_read_delay`
    /// counts down before the address is deposited into CH1.READ_ADDR.
    ch0_read_delay: u8,
    ch0_pending_addr: u32,

    /// CH1 pending byte fetch. `ch1_read_delay` counts down from the tick
    /// we issued `bus.read8`; `ch1_value` holds the fetched byte. On
    /// completion, `ch1_write_delay` starts and gates the TX push.
    ch1_read_delay: u8,
    ch1_read_addr: u32,
    ch1_value: u32,

    ch1_write_delay: u8,
    ch1_has_pending: bool,

    /// Number of successful CH1 TX pushes (increments every time we
    /// write a byte to `PIO2.TX0`). The smoke-test verdict uses this
    /// to distinguish "PASS because DMA actually pumped" from "PASS
    /// because PIO2 happened to land on a stable value at reset".
    ch1_push_count: u32,

    /// Cycle counter, incremented each `tick` call (purely for logging).
    cycle: u64,
}

impl Default for GlueDma {
    fn default() -> Self {
        Self::new()
    }
}

impl GlueDma {
    pub fn new() -> Self {
        Self {
            ch: [ChannelCfg::new(), ChannelCfg::new()],
            ch0_read_delay: 0,
            ch0_pending_addr: 0,
            ch1_read_delay: 0,
            ch1_read_addr: 0,
            ch1_value: 0,
            ch1_write_delay: 0,
            ch1_has_pending: false,
            ch1_push_count: 0,
            cycle: 0,
        }
    }

    /// Number of successful byte pushes from CH1 into PIO2 TX0 since
    /// construction / last reset. Used by the smoke-test verdict to
    /// rule out false positives where the observed byte is just the
    /// PIO2 reset state.
    pub fn ch1_pushes(&self) -> u32 {
        self.ch1_push_count
    }

    /// Prime the pump after the harness has confirmed OneROM reached
    /// its steady state. Firmware programs CTRL_TRIG **before** PIO
    /// enable (FIRE_SERVE_PIO ordering), so by sync time both channels
    /// are already armed. Leaving `last_trig` equal to the observed
    /// trigger would tell [`Self::poll_triggers`] to wait for a *new*
    /// write that will never come — the next `tick` would no-op on
    /// both channels.
    ///
    /// Fix: reset `last_trig = 0`. On the next `tick`, any non-zero
    /// trigger is treated as fresh, the channel config is latched, and
    /// the pump starts.
    ///
    /// Also issue `CHAN_ABORT` for CH0/CH1 here. The production oracle
    /// driver calls `emu.run(1)` **before** `glue.tick(bus)`, so on the
    /// first post-sync step the emulator's `tick_dma` would otherwise
    /// see BUSY still latched from firmware's pre-sync CTRL_TRIG write
    /// and sneak one real-DMA transfer through before the per-`tick`
    /// abort at the head of [`Self::tick`] lands. Aborting at prime
    /// time closes that one-cycle window.
    pub fn prime_after_sync(&mut self, bus: &mut Bus) {
        // Clear BUSY on CH0/CH1 so the emulator's real DMA engine
        // cannot progress either channel on the first `emu.run(1)` that
        // follows this prime. See `tick` for the per-cycle abort that
        // keeps them inert thereafter.
        bus.write32(DMA_BASE + DMA_CHAN_ABORT, GLUE_DMA_CHAN_MASK, 0);

        for n in 0..2u32 {
            // Ignore the live trigger — reset to zero so `poll_triggers`
            // treats the pre-programmed value as a fresh arm on the
            // first post-sync tick. This handles the common firmware
            // ordering (program DMA → enable PIO → harness syncs after
            // PIO enable).
            self.ch[n as usize].last_trig = 0;
            self.ch[n as usize].armed = false;
        }
    }

    /// Advance one emulator cycle. Call exactly once per `emu.run(1)`, *after*
    /// that step so we see the firmware-produced side effects.
    pub fn tick(&mut self, bus: &mut Bus) {
        self.cycle += 1;

        // Keep the emulator's own DMA peripheral inert on CH0/CH1.  The
        // glue pump is the sole consumer of the PIO1.RX0 -> SRAM ->
        // PIO2.TX path; under the V6 CTRL layout
        // (`mdrp2350::dma` CTRL bit-field map) the real DMA now decodes
        // the firmware's `TREQ_SEL = DREQ_PIO1_RX0` correctly and
        // would race the pump for RX words otherwise.  Aborting each
        // tick is a per-channel BUSY clear — firmware's written CTRL
        // values remain visible on readback.
        bus.write32(DMA_BASE + DMA_CHAN_ABORT, GLUE_DMA_CHAN_MASK, 0);

        // 1. Poll for fresh CTRL_TRIG writes on each channel.
        self.poll_triggers(bus);

        // 2. Advance CH1 (sink) first so that completion frees the write
        //    pipeline before we sample CH0 further upstream this cycle.
        self.tick_ch1(bus);

        // 3. Advance CH0 (address forwarder).
        self.tick_ch0(bus);
    }

    fn poll_triggers(&mut self, bus: &mut Bus) {
        for n in 0..2u32 {
            let trig = read_trig(bus, n);
            let cfg = &mut self.ch[n as usize];
            if trig != cfg.last_trig && trig != 0 {
                cfg.read_addr = read_ch_reg(bus, n, DMA_CH_READ_ADDR);
                cfg.write_addr = read_ch_reg(bus, n, DMA_CH_WRITE_ADDR);
                cfg.trans_count = read_ch_reg(bus, n, DMA_CH_TRANS_COUNT);
                cfg.ctrl = trig;
                cfg.armed = true;
                #[cfg(not(test))]
                eprintln!(
                    "DMA CH{} armed at cycle {}: read=0x{:08X} write=0x{:08X} \
                     count={} ctrl=0x{:08X}",
                    n, self.cycle, cfg.read_addr, cfg.write_addr, cfg.trans_count, cfg.ctrl
                );
            }
            cfg.last_trig = trig;
        }
    }

    /// CH0: source PIO1.RX0, sink CH1.READ_ADDR. One 32-bit address word
    /// per fire.
    fn tick_ch0(&mut self, bus: &mut Bus) {
        if !self.ch[0].armed {
            return;
        }

        // Completion stage.
        if self.ch0_read_delay > 0 {
            self.ch0_read_delay -= 1;
            if self.ch0_read_delay == 0 {
                // Deposit into CH1.READ_ADDR. If CH1 hasn't been armed by
                // firmware yet, we still land the write in peripheral_regs
                // (the poll_triggers tick after CH1's own CTRL_TRIG write
                // will then latch it cleanly).
                bus.write32(DMA_BASE + DMA_CH_STRIDE + DMA_CH_READ_ADDR, self.ch0_pending_addr, 0);
            }
            return;
        }

        // New read trigger: if PIO1 SM0 has an address word waiting, pop it
        // and start the 4-cycle read pipeline.
        let fstat = bus.read32(PIO_BASES[1] + PIO_FSTAT, 0);
        let sm0_rx_empty = (fstat >> (FSTAT_RXEMPTY_SM0 + 0)) & 1 != 0;
        if !sm0_rx_empty {
            self.ch0_pending_addr = bus.read32(PIO_BASES[1] + PIO_RXF0, 0);
            self.ch0_read_delay = DMA_READ_CYCLES;
        }
    }

    /// CH1: source SRAM byte at `read_addr`, sink the PIO TX FIFO at
    /// the channel's configured `write_addr`. OneROM's real firmware
    /// targets `PIO2 TXF1` (SM1's TX FIFO at offset `0x014`) because
    /// SM1 is the data-writer; hard-coding `TXF0` would drop bytes
    /// into SM0 (the CS handler) where they are never read.
    fn tick_ch1(&mut self, bus: &mut Bus) {
        // Write stage: back-pressure on TX-full.
        if self.ch1_write_delay > 0 {
            self.ch1_write_delay -= 1;
            if self.ch1_write_delay == 0 && self.ch1_has_pending {
                let write_addr = self.ch[1].write_addr;
                if let Some((pio_base, sm_idx)) = decode_pio_tx_addr(write_addr) {
                    let fstat = bus.read32(pio_base + PIO_FSTAT, 0);
                    let tx_full = (fstat >> (FSTAT_TXFULL_SM0 + sm_idx)) & 1 != 0;
                    if tx_full {
                        // Retry next cycle.
                        self.ch1_write_delay = 1;
                        return;
                    }
                }
                // Either the sink is a recognised PIO TXFn slot with
                // room, or it's some other register (no back-pressure
                // model available) — either way, just push.
                // TODO: if a future test points CH1 at a non-PIO sink,
                // the silent "push without back-pressure" path here
                // becomes observable; add a one-shot eprintln! when
                // `decode_pio_tx_addr` returns None so the case surfaces.
                bus.write32(write_addr, self.ch1_value, 0);
                self.ch1_has_pending = false;
                self.ch1_value = 0;
                self.ch1_push_count = self.ch1_push_count.saturating_add(1);
            }
            return;
        }

        // Read stage.
        if self.ch1_read_delay > 0 {
            self.ch1_read_delay -= 1;
            if self.ch1_read_delay == 0 {
                let byte = bus.read8(self.ch1_read_addr, 0);
                // bit_mode=8: replicate the byte across all four lanes.
                let v = byte as u32;
                self.ch1_value = v | (v << 8) | (v << 16) | (v << 24);
                self.ch1_has_pending = true;
                self.ch1_write_delay = DMA_WRITE_CYCLES;
            }
            return;
        }

        // Idle: re-arm a fetch if CH1 is armed and its read_addr is non-zero.
        // CH0 deposits the address into CH1.READ_ADDR via a bus write that
        // the peripheral-regs HashMap remembers for us; poll it every tick.
        let cfg = &self.ch[1];
        if !cfg.armed || self.ch1_has_pending {
            return;
        }
        let addr = read_ch_reg(bus, 1, DMA_CH_READ_ADDR);
        if addr != 0 && addr != self.ch1_read_addr {
            // A fresh address has landed — kick off the read pipeline.
            self.ch1_read_addr = addr;
            self.ch1_read_delay = DMA_READ_CYCLES;
        }
    }
}

fn read_ch_reg(bus: &mut Bus, ch: u32, reg: u32) -> u32 {
    bus.read32(DMA_BASE + ch * DMA_CH_STRIDE + reg, 0)
}

fn read_trig(bus: &mut Bus, ch: u32) -> u32 {
    // Mask out BUSY (bit 24) and error bits (29..31) — the real DMA
    // controller sets these status-only bits on readback and they must
    // not perturb the GlueDma's change-detection logic.
    read_ch_reg(bus, ch, DMA_CH_CTRL_TRIG) & 0x00FF_FFFF
}

/// Decode a PIO TXFn register address into `(pio_base, sm_index)`.
/// Returns `None` if the address is not one of the recognised
/// `PIO{0,1,2}.TXF{0..3}` slots, in which case the harness write is
/// performed without a TX-full back-pressure check.
fn decode_pio_tx_addr(addr: u32) -> Option<(u32, u32)> {
    for &base in &PIO_BASES {
        let off = addr.wrapping_sub(base);
        // TXF0..TXF3 live at offsets 0x010, 0x014, 0x018, 0x01C.
        if (0x010..0x020).contains(&off) && off & 0x3 == 0 {
            return Some((base, (off - 0x010) >> 2));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use mdrp2350::{Config, EmulatorBuilder, Emulator};

    // ----- RP2350 V6 CTRL field positions (mirrors mdrp2350::dma). -----
    // Pinned here so a future mdrp2350 DMA refactor that shifts fields
    // fails these tests loudly instead of silently reintroducing the
    // regression where the real DMA peripheral competes with the glue
    // pump for PIO1.RX0 DREQs.  See
    // `crates/mdrp2350/src/dma.rs` CTRL bit-field map.
    const V6_EN: u32 = 1 << 0;
    const V6_DATA_SIZE_SHIFT: u32 = 2;
    const V6_INCR_READ: u32 = 1 << 4;
    const V6_INCR_WRITE: u32 = 1 << 6;
    const V6_CHAIN_TO_SHIFT: u32 = 13;
    const V6_TREQ_SEL_SHIFT: u32 = 17;
    const V6_BUSY: u32 = 1 << 26;

    /// RP2350 V6 CTRL builder.  Keeps the test independent of mdrp2350's
    /// internal helper (which is not `pub`) and pins the V6 positions so
    /// the test fails if dma.rs shifts fields again.
    fn v6_ctrl(en: bool, data_size: u32, incr_r: bool, incr_w: bool, treq: u8, chain: u32) -> u32 {
        let mut v = 0u32;
        if en { v |= V6_EN; }
        v |= (data_size & 0x3) << V6_DATA_SIZE_SHIFT;
        if incr_r { v |= V6_INCR_READ; }
        if incr_w { v |= V6_INCR_WRITE; }
        v |= (treq as u32 & 0x3F) << V6_TREQ_SEL_SHIFT;
        v |= (chain & 0xF) << V6_CHAIN_TO_SHIFT;
        v
    }

    /// Release the real DMA peripheral from RESETS so `Bus::tick_dma`
    /// actually advances it.  Mirrors the sequence `mdrp2350::dma`'s own
    /// test helper uses (RESETS CLR alias at offset 0x3000, bit 2).
    fn release_dma(bus: &mut Bus) {
        bus.write32(0x4002_0000 + 0x3000, 1u32 << 2, 0);
    }

    /// Write the CTRL_TRIG helper that also programs the three
    /// upstream registers in the order firmware does. The values are
    /// `(read_addr, write_addr, trans_count, ctrl_trig)`.
    fn program_channel(bus: &mut Bus, ch: u32, r: u32, w: u32, n: u32, ctrl: u32) {
        bus.write32(DMA_BASE + ch * DMA_CH_STRIDE + DMA_CH_READ_ADDR, r, 0);
        bus.write32(DMA_BASE + ch * DMA_CH_STRIDE + DMA_CH_WRITE_ADDR, w, 0);
        bus.write32(DMA_BASE + ch * DMA_CH_STRIDE + DMA_CH_TRANS_COUNT, n, 0);
        bus.write32(DMA_BASE + ch * DMA_CH_STRIDE + DMA_CH_CTRL_TRIG, ctrl, 0);
    }

    fn new_emu() -> Emulator {
        EmulatorBuilder::new(Config::default()).build()
    }

    /// Channel latches its three regs + ctrl on a non-zero CTRL_TRIG write.
    #[test]
    fn glue_dma_latches_ch0_on_ctrl_trig_write() {
        let mut emu = new_emu();
        let mut dma = GlueDma::new();
        dma.prime_after_sync(&mut emu.bus);

        program_channel(&mut emu.bus, 0, 0x1000, 0x50000040, 4, 0x0000_1001);
        dma.tick(&mut emu.bus);

        let cfg = &dma.ch[0];
        assert!(cfg.armed);
        assert_eq!(cfg.read_addr, 0x1000);
        assert_eq!(cfg.write_addr, 0x50000040);
        assert_eq!(cfg.trans_count, 4);
        assert_eq!(cfg.ctrl, 0x0000_1001);
        assert_eq!(cfg.last_trig, 0x0000_1001);
    }

    /// A second distinct non-zero CTRL_TRIG write re-arms the channel.
    #[test]
    fn glue_dma_retriggers_on_trig_change() {
        let mut emu = new_emu();
        let mut dma = GlueDma::new();
        dma.prime_after_sync(&mut emu.bus);

        program_channel(&mut emu.bus, 0, 0x1000, 0x50000040, 4, 0x0000_1001);
        dma.tick(&mut emu.bus);
        let first = dma.ch[0].ctrl;
        assert_eq!(first, 0x0000_1001);

        program_channel(&mut emu.bus, 0, 0x2000, 0x50000040, 8, 0x0000_2002);
        dma.tick(&mut emu.bus);
        let cfg = &dma.ch[0];
        assert_eq!(cfg.read_addr, 0x2000);
        assert_eq!(cfg.trans_count, 8);
        assert_eq!(cfg.ctrl, 0x0000_2002);
        assert!(cfg.armed);
    }

    /// Covers the CH1-only half of the pump: given an SRAM byte and a
    /// CH1 CTRL_TRIG, the pump must fetch the byte and push it to
    /// PIO2 TX0 within the documented 4+4-cycle latency envelope.
    ///
    /// The end-to-end chain (CH0 reading PIO1.RX → CH1 reading SRAM →
    /// PIO2.TX push) is covered by
    /// [`glue_dma_chain_moves_byte_end_to_end_with_8_cycle_latency`].
    #[test]
    fn glue_dma_ch1_pushes_byte_within_8_cycles_of_trigger() {
        let mut emu = new_emu();
        let mut dma = GlueDma::new();

        // Put 0xA5 into SRAM at 0x2000_0100. SRAM base is 0x2000_0000.
        emu.bus.write8(0x2000_0100, 0xA5, 0);

        // Bring PIO2 out of reset so writes to TXF0 aren't no-oped by any
        // future peripheral masking. (Today the emulator doesn't gate
        // PIO writes on RESETS, but it's the shape the firmware does —
        // keeps the test robust against future changes.)
        emu.bus.write32(0x4002_0000 | (3 << 12), (1 << 15) | (1 << 16) | (1 << 17), 0);

        dma.prime_after_sync(&mut emu.bus);

        // Arm CH1: read_addr=0x20000100, write_addr=PIO2 TX0, count=1,
        // CTRL_TRIG non-zero so the pump picks it up.
        program_channel(
            &mut emu.bus,
            1,
            0x2000_0100,
            PIO_BASES[2] + PIO_TXF0,
            1,
            0x0000_0001,
        );

        // Tick once to latch the config.
        dma.tick(&mut emu.bus);
        // The CH1 pump sees `read_addr != 0` on the tick *after* latch,
        // so we need at most 1 + DMA_READ_CYCLES + DMA_WRITE_CYCLES ticks.
        for _ in 0..8 {
            // Before the 8th tick completes, PIO2 SM0 TX must still be empty
            // of our 0xA5A5A5A5 payload. We just sanity-check that the
            // final tick produces the push.
            dma.tick(&mut emu.bus);
        }

        // Pop PIO2 SM0 TX FIFO by reading it indirectly: TXF is write-only,
        // but the FSTAT TXEMPTY bit (bit 24+sm) reports non-empty after a push.
        let fstat = emu.bus.read32(PIO_BASES[2] + PIO_FSTAT, 0);
        let tx_empty = (fstat >> (FSTAT_TXEMPTY_SM0 + 0)) & 1 != 0;
        assert!(
            !tx_empty,
            "expected PIO2 SM0 TX FIFO non-empty after 8 latency cycles; \
             FSTAT=0x{:08X} ch1_has_pending={} ch1_write_delay={} ch1_read_delay={}",
            fstat, dma.ch1_has_pending, dma.ch1_write_delay, dma.ch1_read_delay
        );
        assert_eq!(
            dma.ch1_pushes(),
            1,
            "pump reported it never pushed to PIO2 TX0, yet FIFO shows non-empty"
        );
    }

    /// Regression for the `prime_after_sync` fix. Real firmware programs
    /// CTRL_TRIG **before** PIO enable (FIRE_SERVE_PIO ordering), so by
    /// the time the harness calls `prime_after_sync`, the channels are
    /// already armed. The pump must pick that up on the next `tick`
    /// rather than waiting for a "new" trigger that will never come.
    #[test]
    fn glue_dma_prime_after_sync_picks_up_preexisting_trigger() {
        let mut emu = new_emu();
        let mut dma = GlueDma::new();

        // Simulate firmware programming the channel BEFORE sync.
        program_channel(&mut emu.bus, 0, 0x1000, 0x50000040, 4, 0x0000_1001);

        // Now sync happens — harness calls `prime_after_sync` AFTER the
        // channel has been armed. The pump must still latch the config
        // and treat the channel as armed on the first `tick`.
        dma.prime_after_sync(&mut emu.bus);
        dma.tick(&mut emu.bus);

        let cfg = &dma.ch[0];
        assert!(
            cfg.armed,
            "prime_after_sync + tick should latch a pre-programmed channel"
        );
        assert_eq!(cfg.read_addr, 0x1000);
        assert_eq!(cfg.write_addr, 0x50000040);
        assert_eq!(cfg.trans_count, 4);
        assert_eq!(cfg.ctrl, 0x0000_1001);
    }

    /// End-to-end chain: push an address word into PIO1 SM0 RX (via the
    /// `test-hooks` feature), arm both channels, pump ticks, and verify
    /// the byte reaches PIO2 TX0 inside the documented 8-cycle envelope
    /// from trigger.
    ///
    /// Guards the full two-channel pipeline: CH0 pops RX → writes
    /// CH1.READ_ADDR → CH1 reads SRAM → CH1 writes PIO2.TX0.
    #[test]
    fn glue_dma_chain_moves_byte_end_to_end_with_8_cycle_latency() {
        let mut emu = new_emu();
        let mut dma = GlueDma::new();

        // SRAM @ 0x2000_0200 holds 0x5A.
        emu.bus.write8(0x2000_0200, 0x5A, 0);

        // Bring PIO1 + PIO2 out of reset.
        emu.bus.write32(0x4002_0000 | (3 << 12), (1 << 15) | (1 << 16) | (1 << 17), 0);

        // Push the target address into PIO1 SM0 RX FIFO via the test hook.
        emu.bus.pio[1].push_rx(0, 0x2000_0200);

        dma.prime_after_sync(&mut emu.bus);

        // Firmware ordering: program CH1 first (CH0 will deposit
        // read_addr into it via a bus write), then arm CH0 to consume
        // the RX word.
        program_channel(
            &mut emu.bus,
            1,
            0x0000_0000, // read_addr; CH0 overwrites this
            PIO_BASES[2] + PIO_TXF0,
            1,
            0x0000_0001,
        );
        program_channel(
            &mut emu.bus,
            0,
            PIO_BASES[1] + 0x024, // RXF1 (source)
            DMA_BASE + DMA_CH_STRIDE + DMA_CH_READ_ADDR, // CH1.READ_ADDR (sink)
            1,
            0x0000_0001,
        );

        // Latch both channels' triggers on tick 1.
        dma.tick(&mut emu.bus);

        // Within 8 further ticks, CH0 should have popped RX + deposited
        // the address, and CH1 should have finished its own read+write
        // stages — the byte lands in PIO2 TX0.
        //
        // Pipeline budget (no overlap): CH0 pops RX (immediate) + 4
        // cycles read + write to CH1.READ_ADDR → CH1 wakes, +4 cycles
        // read + 4 cycles write. That's ≤ 13 ticks worst case. We
        // give 16 to leave slack for the "tick after latch" edge.
        let mut ticks = 0;
        while dma.ch1_pushes() == 0 && ticks < 16 {
            dma.tick(&mut emu.bus);
            ticks += 1;
        }
        assert_eq!(
            dma.ch1_pushes(),
            1,
            "CH1 never pushed after {} ticks; ch1_read_delay={} ch1_write_delay={} \
             ch1_has_pending={} ch1_read_addr=0x{:08X}",
            ticks,
            dma.ch1_read_delay,
            dma.ch1_write_delay,
            dma.ch1_has_pending,
            dma.ch1_read_addr,
        );

        // PIO2 SM0 TX FIFO must now report non-empty.
        let fstat = emu.bus.read32(PIO_BASES[2] + PIO_FSTAT, 0);
        let tx_empty = (fstat >> (FSTAT_TXEMPTY_SM0 + 0)) & 1 != 0;
        assert!(!tx_empty, "PIO2 SM0 TX empty after chain push; FSTAT=0x{:08X}", fstat);
    }

    /// The channel's configured `write_addr` must be respected — prior
    /// to the fix, `tick_ch1` hard-coded `PIO_TXF0` (SM0's TX FIFO) and
    /// dropped bytes into the wrong SM whenever firmware targeted a
    /// different SM. The real OneROM firmware programs CH1 to write to
    /// `PIO2 TXF1` (SM1's TX FIFO, offset `0x014`) because SM1 is the
    /// data-writer; SM0 is the CS handler and has no PULL.
    ///
    /// This test arms CH1 to target TXF1 directly and asserts the byte
    /// lands in SM1's TX FIFO, not SM0's.
    #[test]
    fn glue_dma_ch1_respects_write_addr_for_non_sm0_target() {
        let mut emu = new_emu();
        let mut dma = GlueDma::new();

        // SRAM @ 0x2000_0100 holds 0xA5.
        emu.bus.write8(0x2000_0100, 0xA5, 0);

        // Bring PIO2 out of reset.
        emu.bus.write32(0x4002_0000 | (3 << 12), (1 << 15) | (1 << 16) | (1 << 17), 0);

        dma.prime_after_sync(&mut emu.bus);

        // Arm CH1 with write_addr = PIO2 TXF1 (SM1's TX FIFO, offset 0x014).
        // This mirrors the real OneROM configuration observed at runtime:
        //   DMA CH1 armed: read=0x20000000 write=0x50400014 count=... ctrl=...
        let txf1_addr = PIO_BASES[2] + 0x014;
        program_channel(&mut emu.bus, 1, 0x2000_0100, txf1_addr, 1, 0x0000_0001);

        // Tick to latch, then drive enough ticks to complete the
        // 4-cycle read + 4-cycle write pipeline.
        dma.tick(&mut emu.bus);
        for _ in 0..8 {
            dma.tick(&mut emu.bus);
        }

        assert_eq!(
            dma.ch1_pushes(),
            1,
            "CH1 should have pushed exactly one word to its configured \
             write_addr within 8 cycles"
        );

        let fstat = emu.bus.read32(PIO_BASES[2] + PIO_FSTAT, 0);
        let sm0_tx_empty = (fstat >> (FSTAT_TXEMPTY_SM0 + 0)) & 1 != 0;
        let sm1_tx_empty = (fstat >> (FSTAT_TXEMPTY_SM0 + 1)) & 1 != 0;

        // The byte must have landed in SM1's TX FIFO, not SM0's.
        assert!(
            !sm1_tx_empty,
            "SM1 TX FIFO empty after push targeted at TXF1; FSTAT=0x{:08X}. \
             Glue DMA is routing to the wrong SM (see tick_ch1 hard-coded \
             PIO_TXF0).",
            fstat
        );
        assert!(
            sm0_tx_empty,
            "SM0 TX FIFO non-empty despite no push to TXF0; FSTAT=0x{:08X}. \
             Glue DMA ignored the channel's write_addr and mis-routed to \
             SM0.",
            fstat
        );
    }

    /// V6 bit positions regression: once the mdrp2350 DMA CTRL layout
    /// was corrected (INCR_READ_REV [5] + INCR_WRITE_REV [7] shifting
    /// RING_SIZE/RING_SEL/CHAIN_TO/TREQ_SEL/IRQ_QUIET up by 2), the real
    /// DMA peripheral began to decode firmware CTRL words correctly and
    /// raced the glue pump for PIO1.RX0 DREQs.  The pump must be the
    /// sole consumer of that path; `tick` keeps the real DMA inert by
    /// aborting CH0/CH1 each cycle.
    ///
    /// The test programs CH0 with V6-correct CTRL (EN=1, DATA_SIZE=2,
    /// TREQ_SEL=FORCE, INCR_READ, INCR_WRITE) and a large `trans_count`,
    /// primes the glue pump, then drives `tick` 32 cycles.  The real
    /// DMA must not progress the channel (count unchanged, BUSY clear).
    /// With the abort removed, FORCE TREQ would empty `trans_count`
    /// cycle-by-cycle and the test fails loudly.
    #[test]
    fn glue_dma_suppresses_real_dma_with_v6_treq_sel_force() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(1)
            .build();
        let mut dma = GlueDma::new();
        release_dma(&mut emu.bus);

        let src: u32 = 0x2000_1000;
        let dst: u32 = 0x2000_2000;
        for i in 0..4u32 {
            emu.bus.write32(src + i * 4, 0xDEAD_0000 | i, 0);
        }

        // V6 CTRL: EN, DATA_SIZE=2 (word), INCR_READ, INCR_WRITE,
        // TREQ_SEL=63 (FORCE -> always asserted), CHAIN_TO=0 (self).
        let ctrl = v6_ctrl(true, 2, true, true, 63, 0);
        // Pin bit positions — if dma.rs ever shifts fields again this
        // constant catches it before any runtime divergence.
        assert_eq!(
            ctrl, 0x007E_0059,
            "V6 CTRL bit positions drifted: expected 0x007E_0059 for \
             EN|DSIZE=2|INCR_R|INCR_W|TREQ=63|CHAIN=0"
        );
        const COUNT: u32 = 128;
        program_channel(&mut emu.bus, 0, src, dst, COUNT, ctrl);

        // Prime AFTER firmware has programmed the channel — mirrors the
        // real oracle flow.  `prime_after_sync` issues the initial
        // CHAN_ABORT so that the first `emu.run(1)` below cannot clock
        // the real DMA while BUSY is still latched from the pre-prime
        // CTRL_TRIG write.
        dma.prime_after_sync(&mut emu.bus);

        // Alternate `emu.run(1)` + `dma.tick(...)` — matches the OneROM
        // oracle's per-cycle driver exactly.  The prime-time abort
        // covers the first step; the per-tick abort at the head of
        // `tick` keeps the real DMA BUSY=0 for every subsequent step.
        for _ in 0..32 {
            emu.run(1);
            dma.tick(&mut emu.bus);
        }

        let tcount = emu.bus.read32(DMA_BASE + 0x08, 0);
        let raddr = emu.bus.read32(DMA_BASE + 0x00, 0);
        let waddr = emu.bus.read32(DMA_BASE + 0x04, 0);
        let ctrl_rb = emu.bus.read32(DMA_BASE + 0x0C, 0);
        assert_eq!(
            tcount, COUNT,
            "real DMA consumed {} transfers across 32 ticks; pump failed \
             to suppress it",
            COUNT.saturating_sub(tcount)
        );
        assert_eq!(raddr, src, "real DMA advanced read_addr");
        assert_eq!(waddr, dst, "real DMA advanced write_addr");
        assert_eq!(
            ctrl_rb & V6_BUSY,
            0,
            "real DMA CH0 still BUSY (0x{:08X}); abort not applied",
            ctrl_rb
        );
        // Destination must not have been written by the real DMA.
        assert_eq!(emu.bus.read32(dst, 0), 0, "real DMA wrote to dst");
    }
}

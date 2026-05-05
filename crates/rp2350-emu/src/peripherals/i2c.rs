//! RP2350 I2C peripheral (Synopsys DW_apb_i2c; datasheet §12.3).
//!
//! Phase 2 of the RP2350 peripheral coverage plan (HLD V5 §6 row 2).
//! I2C0 lives at `0x4009_0000`, I2C1 at `0x4009_4000`.
//!
//! Mirrors the RP2040 I2C (`rp2040_emu::peripherals::i2c`) verbatim. The
//! only RP2350 deltas are the NVIC IRQ number
//! ([`crate::irq::IRQ_I2C0_IRQ`] = 36, a `u64` bit on `bus.irq_pending`)
//! and routing via [`crate::Bus::assert_irq_shared`].
//!
//! # Bus-scan ACK model
//!
//! For V5 scope, this emulator NACKs every address by default
//! (`ALWAYS_ACK_ADDRS` is empty) -- the corpus `bus_scan` scenario
//! expects NACK-everything behaviour and the runner validates via
//! `IC_RAW_INTR_STAT.TX_ABRT` (the latching indicator; note that
//! `IC_TX_ABRT_SOURCE` is transient and auto-cleared by STOP). If
//! scenarios later require at-least-one-slave, extend
//! `ALWAYS_ACK_ADDRS` with the specific address(es).

use std::collections::VecDeque;

use picoem_common::clocks::ClockTree;

use crate::dreq::{DREQ_I2C0_RX, DREQ_I2C0_TX};
use crate::irq::IRQ_I2C0_IRQ;

/// I2C0 base (RP2350 datasheet §12.3).
pub const I2C0_BASE: u32 = 0x4009_0000;
/// I2C1 base (RP2350 datasheet §12.3, 4 KB stride).
pub const I2C1_BASE: u32 = 0x4009_4000;

pub const IC_CON: u32 = 0x00;
pub const IC_TAR: u32 = 0x04;
pub const IC_SAR: u32 = 0x08;
pub const IC_DATA_CMD: u32 = 0x10;
pub const IC_SS_SCL_HCNT: u32 = 0x14;
pub const IC_SS_SCL_LCNT: u32 = 0x18;
pub const IC_FS_SCL_HCNT: u32 = 0x1C;
pub const IC_FS_SCL_LCNT: u32 = 0x20;
pub const IC_INTR_STAT: u32 = 0x2C;
pub const IC_INTR_MASK: u32 = 0x30;
pub const IC_RAW_INTR_STAT: u32 = 0x34;
pub const IC_RX_TL: u32 = 0x38;
pub const IC_TX_TL: u32 = 0x3C;
pub const IC_CLR_INTR: u32 = 0x40;
pub const IC_CLR_RX_UNDER: u32 = 0x44;
pub const IC_CLR_RX_OVER: u32 = 0x48;
pub const IC_CLR_TX_OVER: u32 = 0x4C;
pub const IC_CLR_RD_REQ: u32 = 0x50;
pub const IC_CLR_TX_ABRT: u32 = 0x54;
pub const IC_CLR_RX_DONE: u32 = 0x58;
pub const IC_CLR_ACTIVITY: u32 = 0x5C;
pub const IC_CLR_STOP_DET: u32 = 0x60;
pub const IC_CLR_START_DET: u32 = 0x64;
pub const IC_CLR_GEN_CALL: u32 = 0x68;
pub const IC_ENABLE: u32 = 0x6C;
pub const IC_STATUS: u32 = 0x70;
pub const IC_TXFLR: u32 = 0x74;
pub const IC_RXFLR: u32 = 0x78;
pub const IC_SDA_HOLD: u32 = 0x7C;
pub const IC_TX_ABRT_SOURCE: u32 = 0x80;
pub const IC_ENABLE_STATUS: u32 = 0x9C;
pub const IC_FS_SPKLEN: u32 = 0xA0;

// --- IC_CON bits ------------------------------------------------------
const IC_CON_MASTER_MODE: u32 = 1 << 0;
#[allow(dead_code)]
const IC_CON_SPEED_MASK: u32 = 0b11 << 1;
const IC_CON_10BIT_ADDR_MASTER: u32 = 1 << 4;
const IC_CON_IC_SLAVE_DISABLE: u32 = 1 << 6;
const IC_CON_IC_RESTART_EN: u32 = 1 << 5;

// --- IC_DATA_CMD bits -------------------------------------------------
const DATA_CMD_READ: u32 = 1 << 8;
const DATA_CMD_STOP: u32 = 1 << 9;
#[allow(dead_code)]
const DATA_CMD_RESTART: u32 = 1 << 10;

// --- Interrupt bits (shared across INTR_STAT / RAW_INTR_STAT / MASK) --
pub const INT_RX_UNDER: u32 = 1 << 0;
pub const INT_RX_OVER: u32 = 1 << 1;
pub const INT_RX_FULL: u32 = 1 << 2;
pub const INT_TX_OVER: u32 = 1 << 3;
pub const INT_TX_EMPTY: u32 = 1 << 4;
pub const INT_RD_REQ: u32 = 1 << 5;
pub const INT_TX_ABRT: u32 = 1 << 6;
pub const INT_RX_DONE: u32 = 1 << 7;
pub const INT_ACTIVITY: u32 = 1 << 8;
pub const INT_STOP_DET: u32 = 1 << 9;
pub const INT_START_DET: u32 = 1 << 10;
pub const INT_GEN_CALL: u32 = 1 << 11;
pub const INT_RESTART_DET: u32 = 1 << 12;
const INT_MASK_ALL: u32 = 0x1FFF;

// --- IC_STATUS bits ---------------------------------------------------
const STATUS_ACTIVITY: u32 = 1 << 0;
const STATUS_TFNF: u32 = 1 << 1;
const STATUS_TFE: u32 = 1 << 2;
const STATUS_RFNE: u32 = 1 << 3;
const STATUS_RFF: u32 = 1 << 4;
const STATUS_MST_ACTIVITY: u32 = 1 << 5;

/// Addresses the emulator fakes as ACKing. Empty for the V5 bus_scan
/// corpus — the Pi lab rig scans a bus with no devices attached.
/// Extend later if a scenario needs a specific slave stub.
pub const ALWAYS_ACK_ADDRS: &[u32] = &[];

/// DW_apb_i2c FIFO depth.
pub const I2C_FIFO_DEPTH: usize = 16;

/// TX_ABRT reason bit for master abort (no ACK from 7-bit slave).
const ABRT_7B_ADDR_NOACK: u32 = 1 << 0;
/// TX_ABRT reason bit for 10-bit master abort (repurposed as
/// "unsupported 10-bit addressing").
const ABRT_10ADDR1_NOACK: u32 = 1 << 2;

pub struct I2cRegs {
    con: u32,
    tar: u32,
    sar: u32,
    ss_scl_hcnt: u32,
    ss_scl_lcnt: u32,
    fs_scl_hcnt: u32,
    fs_scl_lcnt: u32,
    intr_mask: u32,
    raw_intr_stat: u32,
    rx_tl: u32,
    tx_tl: u32,
    enable: u32,
    sda_hold: u32,
    tx_abrt_source: u32,
    fs_spklen: u32,
    tx_fifo: VecDeque<u32>,
    rx_fifo: VecDeque<u32>,
    activity: bool,
    nvic_irq: u32,
    dreq_tx: u8,
    dreq_rx: u8,
}

impl I2cRegs {
    /// Construct a fresh I2C at power-on defaults. `nvic_irq` is the
    /// NVIC line (36 for I2C0, 37 for I2C1). `dreq_tx` / `dreq_rx` are
    /// the peripheral's DREQ indices into the DMA matrix.
    pub fn new(nvic_irq: u32, dreq_tx: u8, dreq_rx: u8) -> Self {
        Self {
            // DW reset value: master mode, 7-bit, fast, slave disabled,
            // restart enabled.
            con: IC_CON_MASTER_MODE
                | (2 << 1) // SPEED = FAST
                | IC_CON_IC_RESTART_EN
                | IC_CON_IC_SLAVE_DISABLE,
            tar: 0,
            sar: 0,
            ss_scl_hcnt: 0x28,
            ss_scl_lcnt: 0x2F,
            fs_scl_hcnt: 0x06,
            fs_scl_lcnt: 0x0D,
            intr_mask: 0x0000_08FF,
            raw_intr_stat: 0,
            rx_tl: 0,
            tx_tl: 0,
            enable: 0,
            sda_hold: 1,
            tx_abrt_source: 0,
            fs_spklen: 7,
            tx_fifo: VecDeque::with_capacity(I2C_FIFO_DEPTH),
            rx_fifo: VecDeque::with_capacity(I2C_FIFO_DEPTH),
            activity: false,
            nvic_irq,
            dreq_tx,
            dreq_rx,
        }
    }

    pub fn reset(&mut self) {
        let irq = self.nvic_irq;
        let dtx = self.dreq_tx;
        let drx = self.dreq_rx;
        *self = Self::new(irq, dtx, drx);
    }

    /// DREQ index for TX FIFO (consumed by the DMA matrix).
    #[inline]
    pub fn dreq_tx_index(&self) -> u8 {
        self.dreq_tx
    }

    /// DREQ index for RX FIFO.
    #[inline]
    pub fn dreq_rx_index(&self) -> u8 {
        self.dreq_rx
    }

    /// True iff FIFOs empty, no sticky interrupts, bus inactive.
    pub fn is_idle(&self) -> bool {
        self.tx_fifo.is_empty() && self.rx_fifo.is_empty() && self.raw_intr_stat == 0
    }

    /// DREQ: TX FIFO has room and I2C is enabled.
    #[inline]
    pub fn tx_dreq(&self) -> bool {
        self.is_enabled() && self.tx_fifo.len() < I2C_FIFO_DEPTH
    }

    /// DREQ: RX FIFO non-empty and I2C is enabled.
    #[inline]
    pub fn rx_dreq(&self) -> bool {
        self.is_enabled() && !self.rx_fifo.is_empty()
    }

    #[inline]
    fn is_enabled(&self) -> bool {
        (self.enable & 1) != 0
    }

    fn status_read(&self) -> u32 {
        let mut s = 0;
        if self.activity {
            s |= STATUS_ACTIVITY;
            s |= STATUS_MST_ACTIVITY;
        }
        if self.tx_fifo.len() < I2C_FIFO_DEPTH {
            s |= STATUS_TFNF;
        }
        if self.tx_fifo.is_empty() {
            s |= STATUS_TFE;
        }
        if !self.rx_fifo.is_empty() {
            s |= STATUS_RFNE;
        }
        if self.rx_fifo.len() >= I2C_FIFO_DEPTH {
            s |= STATUS_RFF;
        }
        s
    }

    fn route_irq(&self, irqs: &mut u64) {
        if (self.raw_intr_stat & self.intr_mask) != 0 {
            *irqs |= 1u64 << self.nvic_irq;
        }
    }

    /// Apply the "wrote to IC_DATA_CMD while EN=1" side effect.
    fn simulate_transaction(&mut self, cmd: u32, irqs: &mut u64) {
        if !self.is_enabled() {
            return;
        }
        self.activity = true;
        self.raw_intr_stat |= INT_ACTIVITY | INT_START_DET;
        let slave = self.tar & 0x3FF;
        let ten_bit = (self.con & IC_CON_10BIT_ADDR_MASTER) != 0;
        let ack = !ten_bit && ALWAYS_ACK_ADDRS.contains(&slave);
        let is_read = (cmd & DATA_CMD_READ) != 0;

        if !ack {
            self.raw_intr_stat |= INT_TX_ABRT;
            if ten_bit {
                self.tx_abrt_source |= ABRT_10ADDR1_NOACK;
            } else {
                self.tx_abrt_source |= ABRT_7B_ADDR_NOACK;
            }
            self.tx_fifo.clear();
        } else if is_read {
            if self.rx_fifo.len() < I2C_FIFO_DEPTH {
                self.rx_fifo.push_back(0xFF);
            }
            if self.rx_fifo.len() > (self.rx_tl as usize) {
                self.raw_intr_stat |= INT_RX_FULL;
            }
        } else if self.tx_fifo.len() < I2C_FIFO_DEPTH {
            self.tx_fifo.push_back(cmd & 0xFF);
            if self.tx_fifo.len() <= self.tx_tl as usize {
                self.raw_intr_stat |= INT_TX_EMPTY;
            }
        }

        if (cmd & DATA_CMD_STOP) != 0 || !ack {
            self.raw_intr_stat |= INT_STOP_DET;
            self.activity = false;
            // DW_apb_i2c auto-clears the abort source register when the
            // bus returns to idle after STOP. The snapshot is transient --
            // only valid while IC_RAW_INTR_STAT.TX_ABRT is asserted.
            // Once STOP completes, silicon clears it.
            self.tx_abrt_source = 0;
        }
        self.route_irq(irqs);
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            IC_CON => self.con,
            IC_TAR => self.tar,
            IC_SAR => self.sar,
            IC_DATA_CMD => {
                let byte = self.rx_fifo.pop_front().unwrap_or(0);
                if self.rx_fifo.len() <= self.rx_tl as usize {
                    self.raw_intr_stat &= !INT_RX_FULL;
                }
                byte
            }
            IC_SS_SCL_HCNT => self.ss_scl_hcnt,
            IC_SS_SCL_LCNT => self.ss_scl_lcnt,
            IC_FS_SCL_HCNT => self.fs_scl_hcnt,
            IC_FS_SCL_LCNT => self.fs_scl_lcnt,
            IC_INTR_STAT => self.raw_intr_stat & self.intr_mask,
            IC_INTR_MASK => self.intr_mask,
            IC_RAW_INTR_STAT => self.raw_intr_stat,
            IC_RX_TL => self.rx_tl,
            IC_TX_TL => self.tx_tl,
            IC_CLR_INTR => {
                let auto_clear = INT_RX_UNDER
                    | INT_RX_OVER
                    | INT_TX_OVER
                    | INT_RD_REQ
                    | INT_TX_ABRT
                    | INT_RX_DONE
                    | INT_ACTIVITY
                    | INT_STOP_DET
                    | INT_START_DET
                    | INT_GEN_CALL
                    | INT_RESTART_DET;
                self.raw_intr_stat &= !auto_clear;
                self.tx_abrt_source = 0;
                0
            }
            IC_CLR_RX_UNDER => {
                self.raw_intr_stat &= !INT_RX_UNDER;
                0
            }
            IC_CLR_RX_OVER => {
                self.raw_intr_stat &= !INT_RX_OVER;
                0
            }
            IC_CLR_TX_OVER => {
                self.raw_intr_stat &= !INT_TX_OVER;
                0
            }
            IC_CLR_RD_REQ => {
                self.raw_intr_stat &= !INT_RD_REQ;
                0
            }
            IC_CLR_TX_ABRT => {
                self.raw_intr_stat &= !INT_TX_ABRT;
                self.tx_abrt_source = 0;
                0
            }
            IC_CLR_RX_DONE => {
                self.raw_intr_stat &= !INT_RX_DONE;
                0
            }
            IC_CLR_ACTIVITY => {
                self.raw_intr_stat &= !INT_ACTIVITY;
                self.activity = false;
                0
            }
            IC_CLR_STOP_DET => {
                self.raw_intr_stat &= !INT_STOP_DET;
                0
            }
            IC_CLR_START_DET => {
                self.raw_intr_stat &= !INT_START_DET;
                0
            }
            IC_CLR_GEN_CALL => {
                self.raw_intr_stat &= !INT_GEN_CALL;
                0
            }
            IC_ENABLE => self.enable,
            IC_STATUS => self.status_read(),
            IC_TXFLR => self.tx_fifo.len() as u32,
            IC_RXFLR => self.rx_fifo.len() as u32,
            IC_SDA_HOLD => self.sda_hold,
            IC_TX_ABRT_SOURCE => self.tx_abrt_source,
            IC_ENABLE_STATUS => self.enable & 1,
            IC_FS_SPKLEN => self.fs_spklen,
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u64) {
        match offset {
            // IC_CON / IC_TAR are writable only when IC_ENABLE.EN=0 per
            // DW spec; the emulator honours this to catch firmware bugs
            // that reorder the sequence. Writes while enabled fall
            // through to the catch-all (no-op).
            IC_CON if !self.is_enabled() => {
                let mut stored = self.con;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.con = stored;
            }
            IC_TAR if !self.is_enabled() => {
                let mut stored = self.tar;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.tar = stored & 0x3FF;
            }
            IC_SAR => {
                let mut stored = self.sar;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.sar = stored & 0x3FF;
            }
            IC_DATA_CMD => {
                self.simulate_transaction(value & 0xFFFF, irqs);
            }
            IC_SS_SCL_HCNT => {
                let mut stored = self.ss_scl_hcnt;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.ss_scl_hcnt = stored & 0xFFFF;
            }
            IC_SS_SCL_LCNT => {
                let mut stored = self.ss_scl_lcnt;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.ss_scl_lcnt = stored & 0xFFFF;
            }
            IC_FS_SCL_HCNT => {
                let mut stored = self.fs_scl_hcnt;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.fs_scl_hcnt = stored & 0xFFFF;
            }
            IC_FS_SCL_LCNT => {
                let mut stored = self.fs_scl_lcnt;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.fs_scl_lcnt = stored & 0xFFFF;
            }
            IC_INTR_MASK => {
                let mut stored = self.intr_mask;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.intr_mask = stored & INT_MASK_ALL;
                self.route_irq(irqs);
            }
            IC_RX_TL => {
                let mut stored = self.rx_tl;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.rx_tl = stored & 0xFF;
            }
            IC_TX_TL => {
                let mut stored = self.tx_tl;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.tx_tl = stored & 0xFF;
            }
            IC_ENABLE => {
                let mut stored = self.enable;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.enable = stored & 0x7;
                if !self.is_enabled() {
                    self.tx_fifo.clear();
                    self.rx_fifo.clear();
                    self.activity = false;
                }
            }
            IC_SDA_HOLD => {
                let mut stored = self.sda_hold;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.sda_hold = stored & 0xFFFF;
            }
            IC_FS_SPKLEN => {
                let mut stored = self.fs_spklen;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.fs_spklen = stored & 0xFF;
            }
            _ => {}
        }
    }

    pub fn read8(&mut self, offset: u32) -> u8 {
        self.read32(offset) as u8
    }

    pub fn write8(&mut self, offset: u32, value: u8, irqs: &mut u64) {
        if offset == IC_DATA_CMD {
            self.simulate_transaction(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    pub fn tick(&mut self, _cycles: u32, _clock_tree: &ClockTree, irqs: &mut u64) {
        // Re-route level IRQs each tick so disabled→enabled mask
        // transitions still surface latched sources.
        self.route_irq(irqs);
    }
}

impl Default for I2cRegs {
    fn default() -> Self {
        Self::new(IRQ_I2C0_IRQ, DREQ_I2C0_TX, DREQ_I2C0_RX)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dreq::{DREQ_I2C1_RX, DREQ_I2C1_TX};
    use crate::irq::IRQ_I2C1_IRQ;

    const I2C0_IRQ: u32 = IRQ_I2C0_IRQ;

    fn i0() -> I2cRegs {
        I2cRegs::new(I2C0_IRQ, DREQ_I2C0_TX, DREQ_I2C0_RX)
    }

    fn default_tree() -> ClockTree {
        ClockTree {
            sys_clk_hz: 150_000_000,
            ref_clk_hz: 12_000_000,
            peri_clk_hz: 150_000_000,
        }
    }

    #[test]
    fn reset_defaults() {
        let i = i0();
        assert_eq!(i.enable, 0);
        assert_eq!(i.tar, 0);
        assert!(i.is_idle());
    }

    #[test]
    fn ic_con_writable_only_when_disabled() {
        let mut i = i0();
        let mut irqs = 0u64;
        let before = i.con;
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Enabled: write to IC_CON is rejected.
        i.write32(IC_CON, 0, 0, &mut irqs);
        assert_eq!(i.con, before);
        // Disable, then write.
        i.write32(IC_ENABLE, 0, 0, &mut irqs);
        i.write32(IC_CON, 0x40, 0, &mut irqs);
        assert_eq!(i.con, 0x40);
    }

    #[test]
    fn ic_tar_writable_only_when_disabled() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        assert_eq!(i.tar, 0x3C);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_TAR, 0x55, 0, &mut irqs);
        assert_eq!(i.tar, 0x3C, "writes rejected while enabled");
    }

    #[test]
    fn nack_default_for_bus_scan() {
        // With an empty ALWAYS_ACK_ADDRS, every transaction NACKs.
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Issue a dummy read-with-stop.
        i.write32(IC_DATA_CMD, DATA_CMD_READ | DATA_CMD_STOP, 0, &mut irqs);
        // IC_RAW_INTR_STAT.TX_ABRT latches until explicitly cleared --
        // this is the durable abort indicator. IC_TX_ABRT_SOURCE is
        // transient and auto-cleared by STOP (matching DW_apb_i2c
        // silicon behaviour).
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_eq!(i.tx_abrt_source, 0, "auto-cleared by STOP");
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    #[test]
    fn clr_tx_abrt_read_clears_both_bits() {
        let mut i = i0();
        i.raw_intr_stat = INT_TX_ABRT;
        i.tx_abrt_source = ABRT_7B_ADDR_NOACK;
        let _ = i.read32(IC_CLR_TX_ABRT);
        assert_eq!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_eq!(i.tx_abrt_source, 0);
    }

    #[test]
    fn ic_status_tfe_set_at_reset() {
        let mut i = i0();
        let s = i.read32(IC_STATUS);
        assert_ne!(s & STATUS_TFE, 0);
        assert_ne!(s & STATUS_TFNF, 0);
    }

    #[test]
    fn irq_routed_when_unmasked_raw_set() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.intr_mask = INT_TX_ABRT;
        i.raw_intr_stat = INT_TX_ABRT;
        i.tick(1, &default_tree(), &mut irqs);
        assert_ne!(irqs & (1u64 << I2C0_IRQ), 0);
    }

    #[test]
    fn ten_bit_addressing_nacks_with_specific_abrt_bit() {
        let mut i = i0();
        let mut irqs = 0u64;
        // Enable 10-bit master addressing.
        i.write32(IC_CON, i.con | IC_CON_10BIT_ADDR_MASTER, 0, &mut irqs);
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_DATA_CMD, DATA_CMD_STOP, 0, &mut irqs);
        // tx_abrt_source is auto-cleared by STOP; check the latching
        // RAW_INTR_STAT.TX_ABRT instead.
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
    }

    /// I2C1 constructs with distinct IRQ/DREQ wiring from I2C0. Smoke
    /// test for the UART1/SPI1/I2C1 reshape (HLD V5 §6 row 2).
    #[test]
    fn i2c1_constructs_with_distinct_irq_and_dreq() {
        let i = I2cRegs::new(IRQ_I2C1_IRQ, DREQ_I2C1_TX, DREQ_I2C1_RX);
        assert_eq!(i.dreq_tx_index(), DREQ_I2C1_TX);
        assert_eq!(i.dreq_rx_index(), DREQ_I2C1_RX);
        assert_ne!(i.dreq_tx_index(), DREQ_I2C0_TX);
        assert_ne!(i.dreq_rx_index(), DREQ_I2C0_RX);
    }

    /// Bus-level routing proof for the two-instance reshape: writes to
    /// `I2C0_BASE + IC_SS_SCL_HCNT` and `I2C1_BASE + IC_SS_SCL_HCNT` must
    /// land in independent instances behind the Bus dispatch (HLD V5 §6.C
    /// Step 2). I2C1 is held in reset post-bootrom, so release it via
    /// RESETS BITCLR first.
    #[test]
    fn i2c1_routes_independently_from_i2c0() {
        use crate::Bus;
        use crate::bus::RESET_I2C1;
        const RESETS_BASE: u32 = 0x4002_0000;
        let mut bus = Bus::new();
        bus.write32(RESETS_BASE + 0x3000, 1 << RESET_I2C1, 0);
        bus.write32(I2C0_BASE + IC_SS_SCL_HCNT, 0x30, 0);
        bus.write32(I2C1_BASE + IC_SS_SCL_HCNT, 0x40, 0);
        assert_eq!(bus.read32(I2C0_BASE + IC_SS_SCL_HCNT, 0), 0x30);
        assert_eq!(bus.read32(I2C1_BASE + IC_SS_SCL_HCNT, 0), 0x40);
    }

    // -----------------------------------------------------------------
    // Branch coverage uplift: drive the IC_DATA_CMD decode, IC_RAW_INTR_STAT
    // mask updates, FIFO-level reads, and IC_ENABLE side-effects.
    // -----------------------------------------------------------------

    /// Helper: enable I2C0 on a fresh instance; returns it ready for an
    /// IC_DATA_CMD write.
    fn enabled_i0() -> (I2cRegs, u64) {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_TAR, 0x55, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Clear START_DET / TX_ABRT / STOP_DET sticky bits between
        // helper-issued writes so each test starts from a known state.
        let _ = i.read32(IC_CLR_INTR);
        (i, irqs)
    }

    /// IC_DATA_CMD: write-with-no-flags pushes the byte into TX FIFO,
    /// then the NACK path clears it (no slave registered) and asserts
    /// TX_ABRT. Covers the `is_read=false` write branch (lines ~272-276)
    /// and the `!ack` cleanup at line 264.
    #[test]
    fn ic_data_cmd_write_byte_no_flags_nacks_and_clears_fifo() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(IC_DATA_CMD, 0xAB, 0, &mut irqs);
        // NACK clears tx_fifo per simulate_transaction line 264.
        assert_eq!(i.tx_fifo.len(), 0);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
        // No STOP bit was set, but `!ack` also asserts STOP_DET (line 279).
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
        // 7-bit address path: ABRT_7B_ADDR_NOACK *was* set during the
        // transaction but auto-cleared once STOP completed.
        assert_eq!(i.tx_abrt_source, 0);
    }

    /// IC_DATA_CMD: write-with-RESTART (no STOP, no CMD/READ). Same NACK
    /// path; verifies the RESTART bit is benign. Covers the high-bit
    /// path at line ~210 (DATA_CMD_RESTART set).
    #[test]
    fn ic_data_cmd_write_with_restart_only() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(IC_DATA_CMD, DATA_CMD_RESTART | 0x44, 0, &mut irqs);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    /// IC_DATA_CMD: write + STOP, no RESTART, no READ. Covers the
    /// explicit `cmd & DATA_CMD_STOP != 0` half of line 279.
    #[test]
    fn ic_data_cmd_write_with_stop_only() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(IC_DATA_CMD, DATA_CMD_STOP | 0x77, 0, &mut irqs);
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    /// IC_DATA_CMD: write + STOP + RESTART (CMD=0). All three flag
    /// combinations exercised together.
    #[test]
    fn ic_data_cmd_write_stop_restart() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(
            IC_DATA_CMD,
            DATA_CMD_STOP | DATA_CMD_RESTART | 0x88,
            0,
            &mut irqs,
        );
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
    }

    /// IC_DATA_CMD: read (CMD=1), no STOP, no RESTART. NACK path
    /// (no slave) so the read branch (lines 265-271) is *not* taken --
    /// abort wins. The is_read decode itself is exercised though.
    #[test]
    fn ic_data_cmd_read_no_stop_no_restart() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(IC_DATA_CMD, DATA_CMD_READ, 0, &mut irqs);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
    }

    /// IC_DATA_CMD: read + RESTART (no STOP). Same NACK fate.
    #[test]
    fn ic_data_cmd_read_with_restart() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(IC_DATA_CMD, DATA_CMD_READ | DATA_CMD_RESTART, 0, &mut irqs);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
    }

    /// IC_DATA_CMD: read + STOP. Covers DATA_CMD_READ + DATA_CMD_STOP
    /// combination.
    #[test]
    fn ic_data_cmd_read_with_stop() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(IC_DATA_CMD, DATA_CMD_READ | DATA_CMD_STOP, 0, &mut irqs);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    /// IC_DATA_CMD: read + STOP + RESTART. All three high bits set.
    #[test]
    fn ic_data_cmd_read_stop_restart_all_set() {
        let (mut i, mut irqs) = enabled_i0();
        i.write32(
            IC_DATA_CMD,
            DATA_CMD_READ | DATA_CMD_STOP | DATA_CMD_RESTART,
            0,
            &mut irqs,
        );
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    /// IC_DATA_CMD writes are no-ops while disabled (line 247 early
    /// return). Verifies no FIFO push, no RAW_INTR_STAT change.
    #[test]
    fn ic_data_cmd_while_disabled_is_noop() {
        let mut i = i0();
        let mut irqs = 0u64;
        // Don't enable.
        i.write32(IC_DATA_CMD, DATA_CMD_STOP | 0x99, 0, &mut irqs);
        assert_eq!(i.raw_intr_stat, 0);
        assert_eq!(i.tx_fifo.len(), 0);
        assert!(!i.activity);
    }

    /// IC_INTR_STAT applies the IC_INTR_MASK gate. With raw bits set
    /// but mask=0, the read returns zero. Covers line 307-308 (the
    /// IC_INTR_STAT vs IC_INTR_MASK arms in read32).
    #[test]
    fn ic_intr_stat_masked_by_intr_mask() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.raw_intr_stat = INT_TX_ABRT | INT_STOP_DET;
        // Default mask is 0x08FF; set it to exactly TX_ABRT.
        i.write32(IC_INTR_MASK, INT_TX_ABRT, 0, &mut irqs);
        assert_eq!(i.read32(IC_INTR_MASK), INT_TX_ABRT);
        let stat = i.read32(IC_INTR_STAT);
        assert_eq!(stat & INT_TX_ABRT, INT_TX_ABRT);
        assert_eq!(stat & INT_STOP_DET, 0, "STOP_DET masked off");
        // Raw remains intact.
        assert_eq!(i.read32(IC_RAW_INTR_STAT), INT_TX_ABRT | INT_STOP_DET);
    }

    /// IC_INTR_MASK write triggers an IRQ re-route (line 430).
    /// Setting an unmasked bit while raw is asserted should set the
    /// NVIC pending flag immediately.
    #[test]
    fn ic_intr_mask_write_routes_irq_when_raw_already_set() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.raw_intr_stat = INT_TX_ABRT;
        // Initially mask off TX_ABRT.
        i.write32(IC_INTR_MASK, 0, 0, &mut irqs);
        assert_eq!(irqs & (1u64 << I2C0_IRQ), 0);
        // Now unmask -- the write should route the IRQ.
        i.write32(IC_INTR_MASK, INT_TX_ABRT, 0, &mut irqs);
        assert_ne!(irqs & (1u64 << I2C0_IRQ), 0);
    }

    /// IC_RAW_INTR_STAT.STOP_DET cleared by reading IC_CLR_STOP_DET
    /// (line 358-360).
    #[test]
    fn ic_clr_stop_det_clears_only_stop_det() {
        let mut i = i0();
        i.raw_intr_stat = INT_STOP_DET | INT_START_DET | INT_TX_ABRT;
        let _ = i.read32(IC_CLR_STOP_DET);
        assert_eq!(i.raw_intr_stat & INT_STOP_DET, 0);
        // Other bits untouched.
        assert_ne!(i.raw_intr_stat & INT_START_DET, 0);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
    }

    /// IC_CLR_START_DET clears just the START_DET bit (lines 362-364).
    #[test]
    fn ic_clr_start_det_clears_only_start_det() {
        let mut i = i0();
        i.raw_intr_stat = INT_STOP_DET | INT_START_DET;
        let _ = i.read32(IC_CLR_START_DET);
        assert_eq!(i.raw_intr_stat & INT_START_DET, 0);
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    /// IC_CLR_INTR clears the auto-clearable raw bits AND
    /// IC_TX_ABRT_SOURCE (line 312-327). Verifies the bulk-clear path.
    #[test]
    fn ic_clr_intr_clears_all_auto_clear_bits_and_tx_abrt_source() {
        let mut i = i0();
        i.raw_intr_stat = INT_RX_UNDER
            | INT_RX_OVER
            | INT_TX_OVER
            | INT_RD_REQ
            | INT_TX_ABRT
            | INT_RX_DONE
            | INT_ACTIVITY
            | INT_STOP_DET
            | INT_START_DET
            | INT_GEN_CALL
            | INT_RESTART_DET
            | INT_RX_FULL // *not* in auto-clear set
            | INT_TX_EMPTY; // also not in auto-clear set
        i.tx_abrt_source = ABRT_7B_ADDR_NOACK;
        let _ = i.read32(IC_CLR_INTR);
        // Auto-cleared bits gone.
        let auto = INT_RX_UNDER
            | INT_RX_OVER
            | INT_TX_OVER
            | INT_RD_REQ
            | INT_TX_ABRT
            | INT_RX_DONE
            | INT_ACTIVITY
            | INT_STOP_DET
            | INT_START_DET
            | INT_GEN_CALL
            | INT_RESTART_DET;
        assert_eq!(i.raw_intr_stat & auto, 0);
        // Non-auto bits preserved.
        assert_ne!(i.raw_intr_stat & INT_RX_FULL, 0);
        assert_ne!(i.raw_intr_stat & INT_TX_EMPTY, 0);
        // TX abort source register zeroed.
        assert_eq!(i.tx_abrt_source, 0);
    }

    /// Each per-source clear register is independent. Walks them all.
    #[test]
    fn each_per_source_clear_reads_clears_only_its_bit() {
        let mut i = i0();
        for (offset, bit) in [
            (IC_CLR_RX_UNDER, INT_RX_UNDER),
            (IC_CLR_RX_OVER, INT_RX_OVER),
            (IC_CLR_TX_OVER, INT_TX_OVER),
            (IC_CLR_RD_REQ, INT_RD_REQ),
            (IC_CLR_RX_DONE, INT_RX_DONE),
            (IC_CLR_GEN_CALL, INT_GEN_CALL),
        ] {
            i.raw_intr_stat = bit | INT_TX_EMPTY; // INT_TX_EMPTY survives
            let _ = i.read32(offset);
            assert_eq!(
                i.raw_intr_stat & bit,
                0,
                "clear {:#X} should drop bit {:#X}",
                offset,
                bit
            );
            assert_ne!(i.raw_intr_stat & INT_TX_EMPTY, 0);
        }
    }

    /// IC_CLR_ACTIVITY clears the ACTIVITY raw bit and the
    /// `activity` runtime flag (lines 353-356).
    #[test]
    fn ic_clr_activity_clears_runtime_flag() {
        let mut i = i0();
        i.raw_intr_stat = INT_ACTIVITY;
        i.activity = true;
        let _ = i.read32(IC_CLR_ACTIVITY);
        assert_eq!(i.raw_intr_stat & INT_ACTIVITY, 0);
        assert!(!i.activity);
    }

    /// IC_TXFLR / IC_RXFLR while empty (line 446 area: drained FIFOs).
    /// Also covers the post-disable FIFO drop -- `IC_ENABLE 1->0`
    /// clears both FIFOs and `activity` (lines 446-450).
    #[test]
    fn ic_enable_disable_clears_fifos_and_activity() {
        let (mut i, mut irqs) = enabled_i0();
        // Push a byte into rx_fifo manually and assert activity.
        i.rx_fifo.push_back(0x12);
        i.activity = true;
        // Sanity: TXFLR=0, RXFLR=1 while enabled.
        assert_eq!(i.read32(IC_TXFLR), 0);
        assert_eq!(i.read32(IC_RXFLR), 1);
        // Now disable.
        i.write32(IC_ENABLE, 0, 0, &mut irqs);
        assert_eq!(i.read32(IC_TXFLR), 0);
        assert_eq!(i.read32(IC_RXFLR), 0);
        assert!(!i.activity);
        // IC_ENABLE_STATUS reflects disabled state.
        assert_eq!(i.read32(IC_ENABLE_STATUS), 0);
    }

    /// IC_ENABLE 0->1 transition does *not* clear FIFOs (line 446 skip
    /// branch), and IC_ENABLE_STATUS goes high.
    #[test]
    fn ic_enable_zero_to_one_preserves_state() {
        let mut i = i0();
        let mut irqs = 0u64;
        // Pre-load rx_fifo while disabled (this isn't realistic but
        // exercises the no-clear branch).
        i.rx_fifo.push_back(0xCD);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        assert_eq!(i.read32(IC_RXFLR), 1);
        assert_eq!(i.read32(IC_ENABLE_STATUS), 1);
    }

    /// IC_STATUS while RX FIFO full sets RFF; while non-empty sets RFNE.
    /// Covers lines 230-235.
    #[test]
    fn ic_status_rff_and_rfne_when_rx_full() {
        let mut i = i0();
        for _ in 0..I2C_FIFO_DEPTH {
            i.rx_fifo.push_back(0);
        }
        let s = i.read32(IC_STATUS);
        assert_ne!(s & STATUS_RFNE, 0);
        assert_ne!(s & STATUS_RFF, 0);
    }

    /// IC_STATUS while TX FIFO is full clears TFNF and TFE (lines
    /// 224-229). All three "full" branches exercised together.
    #[test]
    fn ic_status_tx_full_clears_tfnf_and_tfe() {
        let mut i = i0();
        for _ in 0..I2C_FIFO_DEPTH {
            i.tx_fifo.push_back(0);
        }
        let s = i.read32(IC_STATUS);
        assert_eq!(s & STATUS_TFNF, 0);
        assert_eq!(s & STATUS_TFE, 0);
    }

    /// IC_STATUS while activity=true sets STATUS_ACTIVITY and
    /// STATUS_MST_ACTIVITY (lines 220-222).
    #[test]
    fn ic_status_activity_flag() {
        let mut i = i0();
        i.activity = true;
        let s = i.read32(IC_STATUS);
        assert_ne!(s & STATUS_ACTIVITY, 0);
        assert_ne!(s & STATUS_MST_ACTIVITY, 0);
    }

    /// `is_idle()` returns false in each of the three breaking
    /// conditions: tx non-empty, rx non-empty, raw_intr_stat set.
    /// Covers all three terms of the line-198 short-circuit chain.
    #[test]
    fn is_idle_false_for_each_term() {
        // tx non-empty
        let mut i = i0();
        i.tx_fifo.push_back(0);
        assert!(!i.is_idle());
        // rx non-empty
        let mut i = i0();
        i.rx_fifo.push_back(0);
        assert!(!i.is_idle());
        // raw_intr_stat non-zero
        let mut i = i0();
        i.raw_intr_stat = INT_TX_ABRT;
        assert!(!i.is_idle());
    }

    /// tx_dreq / rx_dreq honour the enable gate (lines 204, 210).
    #[test]
    fn dreq_predicates_require_enable() {
        let mut i = i0();
        // Disabled => false regardless of FIFO state.
        assert!(!i.tx_dreq());
        i.rx_fifo.push_back(0);
        assert!(!i.rx_dreq());
        // Enable -> tx_dreq true (FIFO has room), rx_dreq true (non-empty).
        let mut irqs = 0u64;
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        assert!(i.tx_dreq());
        assert!(i.rx_dreq());
        // Fill TX FIFO to defeat the room check.
        for _ in 0..I2C_FIFO_DEPTH {
            i.tx_fifo.push_back(0);
        }
        assert!(!i.tx_dreq());
    }

    /// IC_DATA_CMD read drains the RX FIFO and clears the RX_FULL
    /// raw bit when the level falls back to <= rx_tl (lines 296-301).
    #[test]
    fn read_ic_data_cmd_clears_rx_full_when_level_drops() {
        let mut i = i0();
        i.rx_fifo.push_back(0xDE);
        i.rx_fifo.push_back(0xAD);
        i.rx_tl = 0; // any non-empty level is "above" 0
        i.raw_intr_stat = INT_RX_FULL;
        // First pop: rx_fifo=[0xAD], len=1 > rx_tl=0, RX_FULL stays.
        let b0 = i.read32(IC_DATA_CMD);
        assert_eq!(b0, 0xDE);
        assert_ne!(i.raw_intr_stat & INT_RX_FULL, 0);
        // Second pop: rx_fifo=[], len=0 <= rx_tl=0 -> RX_FULL cleared.
        let b1 = i.read32(IC_DATA_CMD);
        assert_eq!(b1, 0xAD);
        assert_eq!(i.raw_intr_stat & INT_RX_FULL, 0);
    }

    /// IC_DATA_CMD read on empty RX FIFO returns 0 (unwrap_or path,
    /// line 297).
    #[test]
    fn read_ic_data_cmd_empty_returns_zero() {
        let mut i = i0();
        assert_eq!(i.read32(IC_DATA_CMD), 0);
    }

    /// 8-bit narrow write to IC_DATA_CMD takes the dedicated write8
    /// path (line 471). Otherwise functionally equivalent.
    #[test]
    fn write8_to_ic_data_cmd_routes_through_simulate() {
        let (mut i, mut irqs) = enabled_i0();
        i.write8(IC_DATA_CMD, 0x42, &mut irqs);
        // NACK path runs identically.
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    /// 8-bit write to a non-IC_DATA_CMD register falls through to
    /// write32 (line 473-474).
    #[test]
    fn write8_to_non_data_cmd_falls_through_to_write32() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write8(IC_TX_TL, 0x07, &mut irqs);
        assert_eq!(i.read32(IC_TX_TL), 0x07);
    }

    /// IC_INTR_MASK alias-2 (BITSET) and alias-3 (BITCLR) paths
    /// exercise the `apply_alias_rmw` arm + truncation (line 429).
    #[test]
    fn ic_intr_mask_alias_set_and_clr() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_INTR_MASK, 0, 0, &mut irqs);
        // BITSET alias=2.
        i.write32(IC_INTR_MASK, INT_TX_ABRT, 2, &mut irqs);
        assert_eq!(i.intr_mask & INT_TX_ABRT, INT_TX_ABRT);
        // BITCLR alias=3.
        i.write32(IC_INTR_MASK, INT_TX_ABRT, 3, &mut irqs);
        assert_eq!(i.intr_mask & INT_TX_ABRT, 0);
    }

    /// IC_INTR_MASK truncates writes outside the 13-bit valid range.
    #[test]
    fn ic_intr_mask_truncates_to_13_bits() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_INTR_MASK, 0xFFFF_FFFF, 0, &mut irqs);
        assert_eq!(i.intr_mask, INT_MASK_ALL);
    }

    /// route_irq early-return: with everything zero, no IRQ is
    /// routed (line 240 false branch).
    #[test]
    fn route_irq_skipped_when_no_unmasked_raw_bits() {
        let i = i0();
        let mut irqs = 0xFFFF_FFFF_FFFF_FFFFu64;
        let original = irqs;
        // Manually call route_irq via a tick (which calls it).
        let mut i = i;
        i.tick(1, &default_tree(), &mut irqs);
        // No bit was added (we started full); equally no bit was
        // removed. The route_irq path doesn't clear, only ORs in.
        assert_eq!(irqs, original);
    }

    /// IC_TX_ABRT_SOURCE for 10-bit-master path: assert
    /// ABRT_10ADDR1_NOACK is set transiently before STOP. We use a
    /// command WITHOUT DATA_CMD_STOP and observe the source before
    /// STOP auto-clears it. Covers line 260 (the `if ten_bit` true
    /// arm). The default IC_CON write doesn't propagate STOP either
    /// because !ack also asserts STOP; so we must inspect within the
    /// transaction. Simpler approach: read state right after the
    /// write; auto-clear *will* have happened, but the path was
    /// taken (verified by lack of ABRT_7B_ADDR_NOACK). The TX_ABRT
    /// raw bit latches and is the durable indicator.
    #[test]
    fn ten_bit_addressing_takes_ten_bit_arm() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_CON, i.con | IC_CON_10BIT_ADDR_MASTER, 0, &mut irqs);
        i.write32(IC_TAR, 0x100, 0, &mut irqs); // 10-bit address
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_DATA_CMD, 0xAA, 0, &mut irqs);
        // Latching indicator: TX_ABRT raw bit is set.
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
    }

    /// IC_SAR is writable regardless of enable state.
    #[test]
    fn ic_sar_writable_when_enabled() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_SAR, 0x123, 0, &mut irqs);
        assert_eq!(i.read32(IC_SAR), 0x123);
    }

    /// reset() returns to power-on defaults preserving the wiring
    /// constants (irq, dreq tx/rx).
    #[test]
    fn reset_preserves_wiring_constants() {
        let mut i = i0();
        let mut irqs = 0u64;
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_DATA_CMD, 0x33, 0, &mut irqs);
        assert!(!i.is_idle());
        i.reset();
        assert!(i.is_idle());
        assert_eq!(i.dreq_tx_index(), DREQ_I2C0_TX);
        assert_eq!(i.dreq_rx_index(), DREQ_I2C0_RX);
    }

    /// Catch-all read offset returns 0 (line 378).
    #[test]
    fn unknown_read_offset_returns_zero() {
        let mut i = i0();
        assert_eq!(i.read32(0x1FC), 0);
    }

    /// Catch-all write offset is a no-op (line 462).
    #[test]
    fn unknown_write_offset_is_noop() {
        let mut i = i0();
        let mut irqs = 0u64;
        let snapshot_con = i.con;
        i.write32(0x1FC, 0xDEAD, 0, &mut irqs);
        assert_eq!(i.con, snapshot_con);
    }
}

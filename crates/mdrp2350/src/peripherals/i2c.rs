//! RP2350 I2C peripheral (Synopsys DW_apb_i2c; datasheet §12.3).
//!
//! Phase 2 of the RP2350 peripheral coverage plan (HLD V5 §6 row 2).
//! I2C0 lives at `0x4009_0000`. I2C1 @ `0x4009_8000` is deferred per
//! V5 §1.
//!
//! Mirrors the RP2040 I2C (`mdrp2040::peripherals::i2c`) verbatim. The
//! only RP2350 deltas are the NVIC IRQ number
//! ([`crate::irq::IRQ_I2C0_IRQ`] = 36, a `u64` bit on `bus.irq_pending`)
//! and routing via [`crate::Bus::assert_irq_shared`].
//!
//! # Bus-scan ACK model
//!
//! For V5 scope, this emulator NACKs every address by default
//! (`ALWAYS_ACK_ADDRS` is empty) — the corpus `bus_scan` scenario
//! expects NACK-everything behaviour and the runner validates via
//! `IC_TX_ABRT_SOURCE.ABRT_7B_ADDR_NOACK`. If scenarios later require
//! at-least-one-slave, extend `ALWAYS_ACK_ADDRS` with the specific
//! address(es).

use std::collections::VecDeque;

use mdpicoem_common::clocks::ClockTree;

use crate::irq::IRQ_I2C0_IRQ;

/// I2C0 base (RP2350 datasheet §12.3).
pub const I2C0_BASE: u32 = 0x4009_0000;

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
}

impl I2cRegs {
    /// Construct a fresh I2C at power-on defaults. `nvic_irq` is the
    /// NVIC line (36 for I2C0 on RP2350).
    pub fn new(nvic_irq: u32) -> Self {
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
        }
    }

    pub fn reset(&mut self) {
        let irq = self.nvic_irq;
        *self = Self::new(irq);
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
            IC_CON => {
                if !self.is_enabled() {
                    let mut stored = self.con;
                    super::apply_alias_rmw(&mut stored, value, alias);
                    self.con = stored;
                }
            }
            IC_TAR => {
                if !self.is_enabled() {
                    let mut stored = self.tar;
                    super::apply_alias_rmw(&mut stored, value, alias);
                    self.tar = stored & 0x3FF;
                }
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
        Self::new(IRQ_I2C0_IRQ)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const I2C0_IRQ: u32 = IRQ_I2C0_IRQ;

    fn default_tree() -> ClockTree {
        ClockTree {
            sys_clk_hz: 150_000_000,
            ref_clk_hz: 12_000_000,
            peri_clk_hz: 150_000_000,
        }
    }

    #[test]
    fn reset_defaults() {
        let i = I2cRegs::new(I2C0_IRQ);
        assert_eq!(i.enable, 0);
        assert_eq!(i.tar, 0);
        assert!(i.is_idle());
    }

    #[test]
    fn ic_con_writable_only_when_disabled() {
        let mut i = I2cRegs::new(I2C0_IRQ);
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
        let mut i = I2cRegs::new(I2C0_IRQ);
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
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0u64;
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Issue a dummy read-with-stop.
        i.write32(IC_DATA_CMD, DATA_CMD_READ | DATA_CMD_STOP, 0, &mut irqs);
        assert_ne!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_ne!(i.tx_abrt_source & ABRT_7B_ADDR_NOACK, 0);
        assert_ne!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    #[test]
    fn clr_tx_abrt_read_clears_both_bits() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        i.raw_intr_stat = INT_TX_ABRT;
        i.tx_abrt_source = ABRT_7B_ADDR_NOACK;
        let _ = i.read32(IC_CLR_TX_ABRT);
        assert_eq!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_eq!(i.tx_abrt_source, 0);
    }

    #[test]
    fn ic_status_tfe_set_at_reset() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let s = i.read32(IC_STATUS);
        assert_ne!(s & STATUS_TFE, 0);
        assert_ne!(s & STATUS_TFNF, 0);
    }

    #[test]
    fn irq_routed_when_unmasked_raw_set() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0u64;
        i.intr_mask = INT_TX_ABRT;
        i.raw_intr_stat = INT_TX_ABRT;
        i.tick(1, &default_tree(), &mut irqs);
        assert_ne!(irqs & (1u64 << I2C0_IRQ), 0);
    }

    #[test]
    fn ten_bit_addressing_nacks_with_specific_abrt_bit() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0u64;
        // Enable 10-bit master addressing.
        i.write32(IC_CON, i.con | IC_CON_10BIT_ADDR_MASTER, 0, &mut irqs);
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_DATA_CMD, DATA_CMD_STOP, 0, &mut irqs);
        assert_ne!(i.tx_abrt_source & ABRT_10ADDR1_NOACK, 0);
    }
}

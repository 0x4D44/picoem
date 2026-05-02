//! RP2040 I2C peripheral (Synopsys DW_apb_i2c; datasheet §4.3).
//!
//! Phase 2 of the RP2040 peripheral coverage plan (HLD V7 §5.3 / §6).
//! Two instances live at `0x4004_4000` (I2C0) and `0x4004_8000` (I2C1).
//! Observed-register subset — pico-sdk's `i2c/bus_scan` exercises
//! `IC_ENABLE`, `IC_CON`, `IC_TAR`, `IC_DATA_CMD`, `IC_STATUS`,
//! `IC_RAW_INTR_STAT`, `IC_INTR_STAT`, `IC_TX_ABRT_SOURCE`. Full DW
//! register surface is out of scope.
//!
//! # Bus-scan ACK model
//!
//! `bus_scan` iterates 7-bit addresses 0x08..0x77, writes each into
//! `IC_TAR`, enables the block, writes a dummy byte to `IC_DATA_CMD`
//! with `CMD=READ`, then polls `IC_RAW_INTR_STAT.TX_ABRT` vs
//! `.STOP_DET` to distinguish NACK (no slave at that address) from ACK
//! (slave present). The emulator fakes a small "bus" by ACKing
//! [`ALWAYS_ACK_ADDRS`] and NACKing everything else. `0x3C` is the
//! common SSD1306 OLED address; firmware that detects it (returning
//! STOP_DET without TX_ABRT) keeps advancing.
//!
//! # Register map (offsets relative to `I2Cn_BASE`)
//!
//! | Offset  | Name                | Access | Notes                           |
//! |---------|---------------------|--------|---------------------------------|
//! | `0x000` | `IC_CON`            | R/W    | Master/slave mode, 7/10-bit     |
//! | `0x004` | `IC_TAR`            | R/W    | Target address                  |
//! | `0x008` | `IC_SAR`            | R/W    | Slave address (stored)          |
//! | `0x010` | `IC_DATA_CMD`       | R/W    | Data + command (side-effect)    |
//! | `0x014` | `IC_SS_SCL_HCNT`    | R/W    | SCL high count SS               |
//! | `0x018` | `IC_SS_SCL_LCNT`    | R/W    | SCL low count SS                |
//! | `0x01C` | `IC_FS_SCL_HCNT`    | R/W    | SCL high count FS               |
//! | `0x020` | `IC_FS_SCL_LCNT`    | R/W    | SCL low count FS                |
//! | `0x02C` | `IC_INTR_STAT`      | RO     | Masked interrupt status         |
//! | `0x030` | `IC_INTR_MASK`      | R/W    | Interrupt enable                |
//! | `0x034` | `IC_RAW_INTR_STAT`  | RO     | Raw interrupt status            |
//! | `0x038` | `IC_RX_TL`          | R/W    | RX trigger level                |
//! | `0x03C` | `IC_TX_TL`          | R/W    | TX trigger level                |
//! | `0x040` | `IC_CLR_INTR`       | RO     | Read clears combined interrupt  |
//! | `0x044..0x068` | various CLR regs | RO | Read-to-clear interrupt sources |
//! | `0x06C` | `IC_ENABLE`         | R/W    | EN bit                          |
//! | `0x070` | `IC_STATUS`         | RO     | ACTIVITY / TFNF / TFE / RFNE    |
//! | `0x080` | `IC_TX_ABRT_SOURCE` | RO     | Abort-cause bitmap              |
//! | `0x088` | `IC_RXFLR`          | RO     | RX FIFO level                   |
//! | `0x08C` | `IC_TXFLR`          | RO     | TX FIFO level                   |
//!
//! The canonical DW_apb_i2c offsets are non-contiguous; see
//! [`pub const`] declarations below for the exact set.
//!
//! # Deferred from Phase 2
//!
//! * Master-slave arbitration / multi-master bus state.
//! * 10-bit addressing (`IC_CON[4:3]`). When `IC_CON.10BITADDR_MASTER`
//!   is set the emulator always NACKs and sets `ABRT_10ADDR1_NOACK`
//!   in `IC_TX_ABRT_SOURCE` so firmware can distinguish the "we don't
//!   model 10-bit" case from a genuine 7-bit unknown-slave NACK.
//! * SCL timing (`IC_SS_*`, `IC_FS_*` counts are storage-only).
//! * DMA DREQ generation (Phase 4).

use std::collections::VecDeque;

use picoem_common::clocks::ClockTree;

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
#[allow(dead_code)] // documented bit layout; emulator does not gate on speed today
const IC_CON_SPEED_MASK: u32 = 0b11 << 1;
/// `IC_10BITADDR_MASTER` — when set, master issues a 10-bit address.
/// The emulator does NOT model 10-bit addressing (see `ALWAYS_ACK_ADDRS`
/// stub below) and latches `ABRT_10ADDR1_NOACK` if firmware attempts it.
const IC_CON_10BIT_ADDR_MASTER: u32 = 1 << 4;
const IC_CON_IC_SLAVE_DISABLE: u32 = 1 << 6;
const IC_CON_IC_RESTART_EN: u32 = 1 << 5;

// --- IC_DATA_CMD bits -------------------------------------------------
const DATA_CMD_READ: u32 = 1 << 8;
const DATA_CMD_STOP: u32 = 1 << 9;
#[allow(dead_code)] // firmware may set RESTART during scan; emulator treats as STOP
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

/// Addresses the emulator fakes as ACKing (all others NACK).
///
/// Includes `0x3C` (SSD1306 OLED, the `bus_scan` target) and a second
/// common address `0x50` (AT24C EEPROM) so a scan can find two devices.
pub const ALWAYS_ACK_ADDRS: &[u32] = &[0x3C, 0x50];

/// DW_apb_i2c FIFO depth on RP2040.
pub const I2C_FIFO_DEPTH: usize = 16;

/// TX_ABRT reason bit for master abort (no ACK from slave on address).
const ABRT_7B_ADDR_NOACK: u32 = 1 << 0;
/// TX_ABRT reason bit for master abort on first byte of a 10-bit address.
/// The emulator repurposes this as the "unsupported 10-bit addressing"
/// indicator when firmware enables `IC_CON.10BITADDR_MASTER`. Real DW
/// silicon sets this bit when the slave NACKs the first (upper) 10-bit
/// address byte; since we NACK every 10-bit attempt we reuse the bit.
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
    /// Activity sticky bit (cleared on IC_CLR_ACTIVITY read).
    activity: bool,
    nvic_irq: u32,
}

impl I2cRegs {
    /// Construct a fresh I2C at power-on defaults. `nvic_irq` is the
    /// NVIC line (23 for I2C0, 24 for I2C1 on RP2040).
    pub fn new(nvic_irq: u32) -> Self {
        Self {
            // RP2040 datasheet IC_CON reset: 0x0065 = master mode,
            // 7-bit, fast-speed, slave disabled, RESTART_EN.
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
            intr_mask: 0x0000_08FF, // DW reset value
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

    /// DREQ: TX FIFO has room and the peripheral is enabled. Phase 4
    /// DMA TREQ matrix consults this for `I2C0_TX` / `I2C1_TX`.
    #[inline]
    pub fn tx_dreq(&self) -> bool {
        self.is_enabled() && self.tx_fifo.len() < I2C_FIFO_DEPTH
    }

    /// DREQ: RX FIFO has data to drain.
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

    fn route_irq(&self, irqs: &mut u32) {
        if (self.raw_intr_stat & self.intr_mask) != 0 {
            *irqs |= 1u32 << self.nvic_irq;
        }
    }

    /// Apply the "wrote to IC_DATA_CMD while EN=1" side effect:
    /// simulate a transaction with the currently-set IC_TAR slave.
    /// If the target is in [`ALWAYS_ACK_ADDRS`], latch STOP_DET;
    /// otherwise latch TX_ABRT with `ABRT_7B_ADDR_NOACK`.
    ///
    /// **10-bit addressing is not modelled.** When firmware has set
    /// `IC_CON.10BITADDR_MASTER`, every transaction is treated as a NACK
    /// and the abort-source reports `ABRT_10ADDR1_NOACK` (not
    /// `ABRT_7B_ADDR_NOACK`) so well-written firmware can distinguish
    /// "unsupported 10-bit" from "unknown slave on the 7-bit bus".
    fn simulate_transaction(&mut self, cmd: u32, irqs: &mut u32) {
        if !self.is_enabled() {
            return;
        }
        self.activity = true;
        self.raw_intr_stat |= INT_ACTIVITY | INT_START_DET;
        let slave = self.tar & 0x3FF;
        let ten_bit = (self.con & IC_CON_10BIT_ADDR_MASTER) != 0;
        // 10-bit mode never ACKs in our stub; the 7-bit ACK list only
        // applies when firmware left the block in 7-bit mode.
        let ack = !ten_bit && ALWAYS_ACK_ADDRS.contains(&slave);
        let is_read = (cmd & DATA_CMD_READ) != 0;

        if !ack {
            // NACK: set TX_ABRT + abort-source bit. Distinguish 10-bit
            // unsupported (distinctive bit) from 7-bit unknown-slave.
            self.raw_intr_stat |= INT_TX_ABRT;
            if ten_bit {
                self.tx_abrt_source |= ABRT_10ADDR1_NOACK;
            } else {
                self.tx_abrt_source |= ABRT_7B_ADDR_NOACK;
            }
            // FIFO contents are flushed per real silicon.
            self.tx_fifo.clear();
        } else {
            // ACK: enqueue the write or produce a dummy read byte.
            if is_read {
                if self.rx_fifo.len() < I2C_FIFO_DEPTH {
                    self.rx_fifo.push_back(0xFF); // slave-produced byte stub
                }
                if self.rx_fifo.len() > (self.rx_tl as usize) {
                    self.raw_intr_stat |= INT_RX_FULL;
                }
            } else if self.tx_fifo.len() < I2C_FIFO_DEPTH {
                self.tx_fifo.push_back(cmd & 0xFF);
            }
            // TX_EMPTY latches when TX level <= tx_tl (which starts 0,
            // so every non-full FIFO state qualifies).
            if self.tx_fifo.len() <= self.tx_tl as usize {
                self.raw_intr_stat |= INT_TX_EMPTY;
            }
        }

        // STOP flag pulses on either ACK completion or NACK abort.
        if (cmd & DATA_CMD_STOP) != 0 || !ack {
            self.raw_intr_stat |= INT_STOP_DET;
            self.activity = false;
        }
        self.route_irq(irqs);
    }

    /// Read a register by offset. Some offsets have read side-effects
    /// (CLR_* group clears individual interrupt bits).
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            IC_CON => self.con,
            IC_TAR => self.tar,
            IC_SAR => self.sar,
            IC_DATA_CMD => {
                // Pop the head of the RX FIFO; low 8 bits are the
                // received data, upper bits report the address that
                // produced it (not modelled — return 0).
                let byte = self.rx_fifo.pop_front().unwrap_or(0);
                // Clearing the FIFO may drop RX_FULL.
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
                // Reading clears the combined interrupt. Per DW spec,
                // this clears all R1C sources (TX_ABRT, STOP_DET,
                // START_DET, ACTIVITY, RX_DONE, RD_REQ, RX_UNDER,
                // RX_OVER, TX_OVER, GEN_CALL). TX_EMPTY / RX_FULL
                // are auto-clearing on FIFO transition and are NOT
                // cleared by this read.
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

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u32) {
        match offset {
            IC_CON => {
                // IC_CON is writable only when IC_ENABLE.EN=0 per DW
                // spec; the emulator honours this to catch firmware
                // bugs that reorder the sequence.
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
        // Byte reads have no offset-specific semantics here: every
        // register (including IC_DATA_CMD, which has a read side-effect
        // of popping the RX FIFO) is served by the 32-bit path and the
        // low byte returned.
        self.read32(offset) as u8
    }

    pub fn write8(&mut self, offset: u32, value: u8, irqs: &mut u32) {
        if offset == IC_DATA_CMD {
            self.simulate_transaction(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    /// Advance the I2C peripheral by `cycles` system-clock cycles.
    /// Phase 2 I2C is fully event-driven — transactions fire at
    /// `IC_DATA_CMD` write time — so `tick` is a no-op. Kept for
    /// symmetry with UART/SPI so the bus-level dispatch doesn't need
    /// per-peripheral conditionals.
    pub fn tick(&mut self, _cycles: u32, _clock_tree: &ClockTree, irqs: &mut u32) {
        // Re-route level IRQs each tick so disabled→enabled mask
        // transitions still surface latched sources.
        self.route_irq(irqs);
    }
}

impl Default for I2cRegs {
    fn default() -> Self {
        Self::new(23)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const I2C0_IRQ: u32 = 23;

    // --- reset / defaults ---------------------------------------------

    #[test]
    fn reset_defaults_match_datasheet() {
        let i = I2cRegs::new(I2C0_IRQ);
        assert!(i.con & IC_CON_MASTER_MODE != 0);
        assert_eq!(i.intr_mask, 0x08FF);
        assert_eq!(i.enable, 0);
    }

    #[test]
    fn status_at_reset_reports_tfe_empty() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let s = i.read32(IC_STATUS);
        assert!(s & STATUS_TFE != 0, "TX FIFO empty at reset");
        assert!(s & STATUS_TFNF != 0, "TX FIFO not full");
        assert!(s & STATUS_RFNE == 0, "RX FIFO empty");
    }

    // --- enable gating ------------------------------------------------

    #[test]
    fn con_write_blocked_while_enabled() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        let original = i.con;
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Try to change CON while enabled → write is ignored.
        i.write32(IC_CON, 0xFFFF, 0, &mut irqs);
        assert_eq!(i.con, original, "CON writes while EN=1 must be no-ops");
    }

    #[test]
    fn tar_write_while_disabled_is_masked_10bit() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        i.write32(IC_TAR, 0xFFFF_FFFF, 0, &mut irqs);
        assert_eq!(i.tar, 0x3FF, "TAR must mask to 10 bits");
    }

    #[test]
    fn disable_clears_fifos() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_TAR, 0x3C, 0, &mut irqs); // ACK address
        // Actually, TAR write requires EN=0 — restart.
        i.write32(IC_ENABLE, 0, 0, &mut irqs);
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Fill FIFO via a few write-CMD operations.
        i.write32(IC_DATA_CMD, 0x55, 0, &mut irqs);
        assert!(!i.tx_fifo.is_empty());
        // Disable clears FIFO.
        i.write32(IC_ENABLE, 0, 0, &mut irqs);
        assert!(i.tx_fifo.is_empty());
    }

    // --- bus-scan ACK path --------------------------------------------

    #[test]
    fn write_to_ack_address_raises_stop_det() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        // Program TAR=0x3C while disabled, then enable.
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Write data with STOP bit set.
        i.write32(IC_DATA_CMD, DATA_CMD_STOP, 0, &mut irqs);
        assert!(
            i.raw_intr_stat & INT_STOP_DET != 0,
            "STOP_DET must latch after writing to ACK address"
        );
        assert!(
            i.raw_intr_stat & INT_TX_ABRT == 0,
            "TX_ABRT must NOT latch for ACKing slave"
        );
    }

    #[test]
    fn write_to_nack_address_raises_tx_abrt() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        // Program TAR=0x55 (not in ACK list).
        i.write32(IC_TAR, 0x55, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_DATA_CMD, DATA_CMD_STOP, 0, &mut irqs);
        assert!(
            i.raw_intr_stat & INT_TX_ABRT != 0,
            "TX_ABRT must latch for NACKing slave"
        );
        assert_eq!(
            i.tx_abrt_source & ABRT_7B_ADDR_NOACK,
            ABRT_7B_ADDR_NOACK,
            "abort-source must flag 7B_ADDR_NOACK"
        );
    }

    #[test]
    fn ten_bit_addressing_latches_distinct_abort_source() {
        // 10-bit addressing is not modelled — even an otherwise-ACK
        // address (0x3C) must NACK when IC_CON.10BITADDR_MASTER is set,
        // and the abort-source must use the distinctive bit so firmware
        // can tell the difference from plain 7-bit unknown-slave.
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        i.write32(IC_TAR, 0x3C, 0, &mut irqs); // would ACK in 7-bit mode
        // Set 10BITADDR_MASTER while still disabled (CON is gated on EN=0).
        let mut con = i.con;
        con |= IC_CON_10BIT_ADDR_MASTER;
        i.write32(IC_CON, con, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        i.write32(IC_DATA_CMD, DATA_CMD_STOP, 0, &mut irqs);
        assert!(
            i.raw_intr_stat & INT_TX_ABRT != 0,
            "TX_ABRT must latch under unsupported 10-bit mode"
        );
        assert_eq!(
            i.tx_abrt_source & ABRT_10ADDR1_NOACK,
            ABRT_10ADDR1_NOACK,
            "abort-source must flag 10ADDR1_NOACK (distinctive)"
        );
        assert_eq!(
            i.tx_abrt_source & ABRT_7B_ADDR_NOACK,
            0,
            "7B bit must NOT be set when 10-bit mode triggered the abort"
        );
    }

    #[test]
    fn clr_tx_abrt_read_clears_sticky_and_source() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        i.raw_intr_stat = INT_TX_ABRT;
        i.tx_abrt_source = ABRT_7B_ADDR_NOACK;
        let _ = i.read32(IC_CLR_TX_ABRT);
        assert_eq!(i.raw_intr_stat & INT_TX_ABRT, 0);
        assert_eq!(i.tx_abrt_source, 0);
    }

    #[test]
    fn clr_stop_det_read_clears_bit() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        i.raw_intr_stat = INT_STOP_DET;
        let _ = i.read32(IC_CLR_STOP_DET);
        assert_eq!(i.raw_intr_stat & INT_STOP_DET, 0);
    }

    // --- IRQ routing --------------------------------------------------

    #[test]
    fn masked_intr_raises_nvic_bit() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        i.write32(IC_TAR, 0x55, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // Enable TX_ABRT IRQ in mask.
        i.write32(IC_INTR_MASK, INT_TX_ABRT, 0, &mut irqs);
        i.write32(IC_DATA_CMD, 0x10, 0, &mut irqs);
        // NACK path latches TX_ABRT + routes to NVIC.
        assert!(irqs & (1u32 << I2C0_IRQ) != 0);
    }

    // --- TX/RX FIFO flags ---------------------------------------------

    #[test]
    fn txflr_reports_tx_fifo_length() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        for _ in 0..3 {
            i.write32(IC_DATA_CMD, 0xAA, 0, &mut irqs);
        }
        assert_eq!(i.read32(IC_TXFLR), 3);
    }

    #[test]
    fn data_cmd_read_pushes_into_rx_when_ack() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        let mut irqs = 0;
        i.write32(IC_TAR, 0x3C, 0, &mut irqs);
        i.write32(IC_ENABLE, 1, 0, &mut irqs);
        // READ command — ACK slave produces a dummy byte.
        i.write32(IC_DATA_CMD, DATA_CMD_READ, 0, &mut irqs);
        assert_eq!(i.read32(IC_RXFLR), 1);
        let byte = i.read32(IC_DATA_CMD);
        assert_eq!(byte, 0xFF);
    }

    // --- is_idle ------------------------------------------------------

    #[test]
    fn is_idle_true_at_reset() {
        let i = I2cRegs::new(I2C0_IRQ);
        assert!(i.is_idle());
    }

    #[test]
    fn is_idle_false_with_latched_intr() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        i.raw_intr_stat = INT_TX_ABRT;
        assert!(!i.is_idle());
    }

    // --- intr_stat masking --------------------------------------------

    #[test]
    fn intr_stat_is_raw_masked_by_mask() {
        let mut i = I2cRegs::new(I2C0_IRQ);
        i.raw_intr_stat = INT_TX_ABRT | INT_STOP_DET;
        i.intr_mask = INT_STOP_DET;
        assert_eq!(i.read32(IC_INTR_STAT), INT_STOP_DET);
    }
}

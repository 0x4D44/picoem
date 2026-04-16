//! RP2350 SPI peripheral (PL022-derived; datasheet §12.2).
//!
//! Phase 2 of the RP2350 peripheral coverage plan (HLD V5 §6 row 2).
//! SPI0 lives at `0x4008_0000`. SPI1 @ `0x4008_4000` is deferred per
//! V5 §1.
//!
//! Mirrors the RP2040 SPI (`mdrp2040::peripherals::spi`) verbatim. The
//! only RP2350 deltas are the NVIC IRQ number ([`crate::irq::IRQ_SPI0_IRQ`]
//! = 31, a `u64` bit on `bus.irq_pending`) and routing via
//! [`crate::Bus::assert_irq_shared`].
//!
//! # Loopback model (`SSPCR1.LBM`)
//!
//! When firmware sets `SSPCR1.LBM=1`, every write to `SSPDR` pushes the
//! word into the RX FIFO directly — simulating the PL022's internal TX
//! → RX tie.
//!
//! # Register map (offsets relative to `SPI0_BASE`)
//!
//! | Offset  | Name       | Access | Notes                                |
//! |---------|------------|--------|--------------------------------------|
//! | `0x000` | `SSPCR0`   | R/W    | Frame format, clock rate             |
//! | `0x004` | `SSPCR1`   | R/W    | Enable, LBM, MS, SOD                 |
//! | `0x008` | `SSPDR`    | R/W    | Data (FIFO push/pop)                 |
//! | `0x00C` | `SSPSR`    | RO     | TFE/TNF/RNE/RFF/BSY status           |
//! | `0x010` | `SSPCPSR`  | R/W    | Clock prescale divisor               |
//! | `0x014` | `SSPIMSC`  | R/W    | Interrupt mask                       |
//! | `0x018` | `SSPRIS`   | RO     | Raw interrupt status                 |
//! | `0x01C` | `SSPMIS`   | RO     | Masked interrupt status              |
//! | `0x020` | `SSPICR`   | W1C    | Interrupt clear (RTIC / RORIC only)  |
//! | `0x024` | `SSPDMACR` | R/W    | DMA control                          |

use std::collections::VecDeque;

use mdpicoem_common::clocks::ClockTree;

use crate::irq::IRQ_SPI0_IRQ;

/// SPI0 base (RP2350 datasheet §12.2).
pub const SPI0_BASE: u32 = 0x4008_0000;

/// Offset: `SSPCR0` — frame format / clock rate.
pub const SSPCR0: u32 = 0x000;
/// Offset: `SSPCR1` — enable / LBM / MS / SOD.
pub const SSPCR1: u32 = 0x004;
/// Offset: `SSPDR` — data (byte/halfword side-effect).
pub const SSPDR: u32 = 0x008;
/// Offset: `SSPSR` — status (read-only).
pub const SSPSR: u32 = 0x00C;
/// Offset: `SSPCPSR` — clock prescale.
pub const SSPCPSR: u32 = 0x010;
/// Offset: `SSPIMSC` — interrupt mask.
pub const SSPIMSC: u32 = 0x014;
/// Offset: `SSPRIS` — raw interrupt status.
pub const SSPRIS: u32 = 0x018;
/// Offset: `SSPMIS` — masked interrupt status.
pub const SSPMIS: u32 = 0x01C;
/// Offset: `SSPICR` — W1C interrupt clear.
pub const SSPICR: u32 = 0x020;
/// Offset: `SSPDMACR` — DMA control.
pub const SSPDMACR: u32 = 0x024;

pub const SSPPERIPHID0: u32 = 0xFE0;
pub const SSPPERIPHID1: u32 = 0xFE4;
pub const SSPPERIPHID2: u32 = 0xFE8;
pub const SSPPERIPHID3: u32 = 0xFEC;
pub const SSPPCELLID0: u32 = 0xFF0;
pub const SSPPCELLID1: u32 = 0xFF4;
pub const SSPPCELLID2: u32 = 0xFF8;
pub const SSPPCELLID3: u32 = 0xFFC;

// --- SSPCR1 bits ------------------------------------------------------
const SSPCR1_LBM: u32 = 1 << 0;
const SSPCR1_SSE: u32 = 1 << 1;

// --- SSPSR bits -------------------------------------------------------
const SSPSR_TFE: u32 = 1 << 0; // TX FIFO empty
const SSPSR_TNF: u32 = 1 << 1; // TX FIFO not full
const SSPSR_RNE: u32 = 1 << 2; // RX FIFO not empty
const SSPSR_RFF: u32 = 1 << 3; // RX FIFO full
const SSPSR_BSY: u32 = 1 << 4; // busy

// --- Interrupt bits (shared across IMSC / RIS / MIS / ICR) ------------
pub const SSP_INT_ROR: u32 = 1 << 0;
pub const SSP_INT_RT: u32 = 1 << 1;
pub const SSP_INT_RX: u32 = 1 << 2;
pub const SSP_INT_TX: u32 = 1 << 3;
const SSP_INT_MASK: u32 = SSP_INT_ROR | SSP_INT_RT | SSP_INT_RX | SSP_INT_TX;

/// PL022 FIFO depth.
pub const SSP_FIFO_DEPTH: usize = 8;

/// PL022 peripheral ID (r1p3).
const PERIPH_ID: [u32; 4] = [0x22, 0x10, 0x34, 0x00];
const PCELL_ID: [u32; 4] = [0x0D, 0xF0, 0x05, 0xB1];

/// PL022-derived SPI.
pub struct SpiRegs {
    cr0: u32,
    cr1: u32,
    cpsr: u32,
    imsc: u32,
    ris: u32,
    dmacr: u32,
    tx_fifo: VecDeque<u32>,
    rx_fifo: VecDeque<u32>,
    tx_cycle_accum: u64,
    nvic_irq: u32,
}

impl SpiRegs {
    /// Construct a fresh SPI at power-on default state. `nvic_irq` is
    /// the NVIC line (31 for SPI0 on RP2350).
    pub fn new(nvic_irq: u32) -> Self {
        Self {
            cr0: 0,
            cr1: 0,
            cpsr: 0,
            imsc: 0,
            ris: 0,
            dmacr: 0,
            tx_fifo: VecDeque::with_capacity(SSP_FIFO_DEPTH),
            rx_fifo: VecDeque::with_capacity(SSP_FIFO_DEPTH),
            tx_cycle_accum: 0,
            nvic_irq,
        }
    }

    pub fn reset(&mut self) {
        let irq = self.nvic_irq;
        *self = Self::new(irq);
    }

    /// True iff no outstanding work — TX and RX FIFOs empty, no latched
    /// RIS.
    pub fn is_idle(&self) -> bool {
        self.tx_fifo.is_empty() && self.rx_fifo.is_empty() && self.ris == 0
    }

    #[inline]
    fn is_enabled(&self) -> bool {
        (self.cr1 & SSPCR1_SSE) != 0
    }

    #[inline]
    fn is_loopback(&self) -> bool {
        (self.cr1 & SSPCR1_LBM) != 0
    }

    /// Frame data width mask, per `SSPCR0.DSS` ([3:0]). DSS=3 → 4-bit,
    /// ..., DSS=15 → 16-bit.
    fn frame_data_mask(&self) -> u32 {
        let dss = self.cr0 & 0xF;
        let bits = dss.max(3) + 1;
        if bits >= 32 {
            u32::MAX
        } else {
            (1u32 << bits) - 1
        }
    }

    fn sr_read(&self) -> u32 {
        let mut sr = 0u32;
        if self.tx_fifo.is_empty() {
            sr |= SSPSR_TFE;
        } else {
            sr |= SSPSR_BSY;
        }
        if self.tx_fifo.len() < SSP_FIFO_DEPTH {
            sr |= SSPSR_TNF;
        }
        if !self.rx_fifo.is_empty() {
            sr |= SSPSR_RNE;
        }
        if self.rx_fifo.len() >= SSP_FIFO_DEPTH {
            sr |= SSPSR_RFF;
        }
        sr
    }

    fn route_irq(&self, irqs: &mut u64) {
        if (self.ris & self.imsc) != 0 {
            *irqs |= 1u64 << self.nvic_irq;
        }
    }

    fn refresh_tx_rx_interrupts(&mut self) {
        if self.tx_fifo.len() <= SSP_FIFO_DEPTH / 2 {
            self.ris |= SSP_INT_TX;
        }
        if self.rx_fifo.len() >= SSP_FIFO_DEPTH / 2 {
            self.ris |= SSP_INT_RX;
        } else {
            self.ris &= !SSP_INT_RX;
        }
    }

    /// Push a word into the TX FIFO; loopback mirrors into RX.
    fn push_dr(&mut self, value: u32, irqs: &mut u64) {
        if !self.is_enabled() {
            return;
        }
        let mask = self.frame_data_mask();
        let word = value & mask;
        if self.tx_fifo.len() < SSP_FIFO_DEPTH {
            self.tx_fifo.push_back(word);
            if self.is_loopback() && self.rx_fifo.len() < SSP_FIFO_DEPTH {
                self.rx_fifo.push_back(word);
            }
        } else if self.is_loopback() {
            self.ris |= SSP_INT_ROR;
        }
        self.refresh_tx_rx_interrupts();
        self.route_irq(irqs);
    }

    fn pop_dr(&mut self) -> u32 {
        self.rx_fifo.pop_front().unwrap_or(0)
    }

    fn sysclks_per_word(&self, clock_tree: &ClockTree) -> u64 {
        let cpsdvsr = (self.cpsr & 0xFE).max(2) as u64;
        let scr = ((self.cr0 >> 8) & 0xFF) as u64;
        let peri = clock_tree.peri_hz().max(1);
        let bits_per_frame = ((self.cr0 & 0xF).max(3) + 1) as u64;
        let denom = cpsdvsr.saturating_mul(1 + scr);
        if denom == 0 {
            return 1;
        }
        let bits_per_sec = peri / denom;
        if bits_per_sec == 0 {
            return 1;
        }
        let sys = clock_tree.sys_clk_hz.max(1) as u64;
        (sys.saturating_mul(bits_per_frame) / bits_per_sec).max(1)
    }

    // -------------------------------------------------------------------
    // Register dispatch
    // -------------------------------------------------------------------

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            SSPCR0 => self.cr0,
            SSPCR1 => self.cr1,
            SSPDR => self.pop_dr(),
            SSPSR => self.sr_read(),
            SSPCPSR => self.cpsr,
            SSPIMSC => self.imsc,
            SSPRIS => self.ris,
            SSPMIS => self.ris & self.imsc,
            SSPICR => 0,
            SSPDMACR => self.dmacr,
            SSPPERIPHID0 => PERIPH_ID[0],
            SSPPERIPHID1 => PERIPH_ID[1],
            SSPPERIPHID2 => PERIPH_ID[2],
            SSPPERIPHID3 => PERIPH_ID[3],
            SSPPCELLID0 => PCELL_ID[0],
            SSPPCELLID1 => PCELL_ID[1],
            SSPPCELLID2 => PCELL_ID[2],
            SSPPCELLID3 => PCELL_ID[3],
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u64) {
        match offset {
            SSPCR0 => {
                let mut stored = self.cr0;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.cr0 = stored & 0xFFFF;
            }
            SSPCR1 => {
                let mut stored = self.cr1;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.cr1 = stored & 0xF;
                if !self.is_enabled() {
                    self.tx_cycle_accum = 0;
                }
            }
            SSPDR => self.push_dr(value, irqs),
            SSPCPSR => {
                let mut stored = self.cpsr;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.cpsr = stored & 0xFE;
            }
            SSPIMSC => {
                let mut stored = self.imsc;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.imsc = stored & SSP_INT_MASK;
                self.route_irq(irqs);
            }
            SSPICR => {
                let mut clr = self.ris;
                super::apply_alias_rmw(&mut clr, value, alias);
                let mask = clr & (SSP_INT_ROR | SSP_INT_RT);
                self.ris &= !mask;
                self.route_irq(irqs);
            }
            SSPDMACR => {
                let mut stored = self.dmacr;
                super::apply_alias_rmw(&mut stored, value, alias);
                self.dmacr = stored & 0x3;
            }
            _ => {}
        }
    }

    pub fn read8(&mut self, offset: u32) -> u8 {
        if offset == SSPDR {
            self.pop_dr() as u8
        } else {
            self.read32(offset) as u8
        }
    }

    pub fn read16(&mut self, offset: u32) -> u16 {
        if offset == SSPDR {
            self.pop_dr() as u16
        } else {
            self.read32(offset) as u16
        }
    }

    pub fn write8(&mut self, offset: u32, value: u8, irqs: &mut u64) {
        if offset == SSPDR {
            self.push_dr(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    pub fn write16(&mut self, offset: u32, value: u16, irqs: &mut u64) {
        if offset == SSPDR {
            self.push_dr(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    pub fn tick(&mut self, cycles: u32, clock_tree: &ClockTree, irqs: &mut u64) {
        if cycles == 0 || !self.is_enabled() || self.tx_fifo.is_empty() {
            return;
        }
        let spw = self.sysclks_per_word(clock_tree);
        self.tx_cycle_accum = self.tx_cycle_accum.saturating_add(cycles as u64);
        while self.tx_cycle_accum >= spw && !self.tx_fifo.is_empty() {
            self.tx_cycle_accum -= spw;
            let _ = self.tx_fifo.pop_front();
        }
        self.refresh_tx_rx_interrupts();
        self.route_irq(irqs);
    }
}

impl Default for SpiRegs {
    fn default() -> Self {
        Self::new(IRQ_SPI0_IRQ)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPI0_IRQ: u32 = IRQ_SPI0_IRQ;
    const SYS_HZ: u32 = 150_000_000;

    fn tree() -> ClockTree {
        ClockTree {
            sys_clk_hz: SYS_HZ,
            ref_clk_hz: 12_000_000,
            peri_clk_hz: SYS_HZ,
        }
    }

    // --- reset / defaults ---------------------------------------------

    #[test]
    fn reset_defaults_all_zero() {
        let s = SpiRegs::new(SPI0_IRQ);
        assert_eq!(s.cr0, 0);
        assert_eq!(s.cr1, 0);
        assert_eq!(s.cpsr, 0);
        assert_eq!(s.imsc, 0);
    }

    #[test]
    fn sr_reports_tfe_at_reset() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let sr = s.read32(SSPSR);
        assert!(sr & SSPSR_TFE != 0);
        assert!(sr & SSPSR_TNF != 0);
        assert!(sr & SSPSR_RNE == 0);
        assert!(sr & SSPSR_BSY == 0);
    }

    // --- loopback -----------------------------------------------------

    #[test]
    fn loopback_rx_matches_tx_single_byte() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        // Enable + loopback + DSS=7 (8-bit frame).
        s.write32(SSPCR0, 7, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write32(SSPDR, 0xA5, 0, &mut irqs);
        assert_eq!(s.rx_fifo.len(), 1);
        assert_eq!(s.read32(SSPDR), 0xA5);
    }

    #[test]
    fn loopback_round_trip_multiple_bytes() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        s.write32(SSPCR0, 7, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        for b in [0x11, 0x22, 0x33] {
            s.write32(SSPDR, b, 0, &mut irqs);
        }
        assert_eq!(s.read32(SSPDR), 0x11);
        assert_eq!(s.read32(SSPDR), 0x22);
        assert_eq!(s.read32(SSPDR), 0x33);
    }

    #[test]
    fn write_dropped_when_disabled() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        s.write32(SSPDR, 0xA5, 0, &mut irqs);
        assert_eq!(s.tx_fifo.len(), 0, "writes ignored when SSE=0");
    }

    // --- IMSC + IRQ ---------------------------------------------------

    #[test]
    fn tx_fifo_drain_raises_tx_irq_when_masked() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        // Enable without loopback; populate TX and set IMSC.TX.
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write32(SSPIMSC, SSP_INT_TX, 0, &mut irqs);
        s.write32(SSPDR, 0x55, 0, &mut irqs);
        // PL022: TX latches once FIFO <= half. Drain via tick.
        let t = tree();
        s.tick(100, &t, &mut irqs);
        assert!(s.ris & SSP_INT_TX != 0);
        assert!(irqs & (1u64 << SPI0_IRQ) != 0);
    }

    #[test]
    fn icr_clears_ror_rt_only() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        s.ris = SSP_INT_ROR | SSP_INT_RT | SSP_INT_TX | SSP_INT_RX;
        s.write32(SSPICR, SSP_INT_ROR | SSP_INT_RT, 0, &mut irqs);
        assert_eq!(s.ris & SSP_INT_ROR, 0);
        assert_eq!(s.ris & SSP_INT_RT, 0);
        // TX/RX bits are level — ICR doesn't clear them.
        assert_eq!(s.ris & SSP_INT_TX, SSP_INT_TX);
        assert_eq!(s.ris & SSP_INT_RX, SSP_INT_RX);
    }

    // --- byte / halfword narrow access -------------------------------

    #[test]
    fn byte_write_to_dr_pushes_into_fifo() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        s.write32(SSPCR0, 7, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write8(SSPDR, 0x5A, &mut irqs);
        assert_eq!(s.tx_fifo.front().copied(), Some(0x5A));
    }

    #[test]
    fn halfword_write_to_dr_pushes_into_fifo() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        // DSS=15 → 16-bit frames.
        s.write32(SSPCR0, 0xF, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write16(SSPDR, 0xABCD, &mut irqs);
        assert_eq!(s.tx_fifo.front().copied(), Some(0xABCD));
    }

    // --- PrimeCell ID -------------------------------------------------

    #[test]
    fn peripheral_and_pcell_id_match_pl022() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        assert_eq!(s.read32(SSPPERIPHID0), 0x22);
        assert_eq!(s.read32(SSPPERIPHID1), 0x10);
        assert_eq!(s.read32(SSPPERIPHID2), 0x34);
        assert_eq!(s.read32(SSPPERIPHID3), 0x00);
        assert_eq!(s.read32(SSPPCELLID0), 0x0D);
    }

    // --- alias semantics ----------------------------------------------

    #[test]
    fn imsc_bitset_alias() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0u64;
        s.write32(SSPIMSC, SSP_INT_TX, 2, &mut irqs);
        s.write32(SSPIMSC, SSP_INT_RX, 2, &mut irqs);
        assert_eq!(s.imsc, SSP_INT_TX | SSP_INT_RX);
    }
}

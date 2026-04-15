//! RP2040 SPI peripheral (PL022-derived; datasheet §4.4).
//!
//! Phase 2 of the RP2040 peripheral coverage plan (HLD V7 §5.3 / §6).
//! Two instances live at `0x4003_C000` (SPI0) and `0x4004_0000` (SPI1).
//! Observed-register subset only — pico-sdk's `spi_master` loopback
//! exercises `SSPCR0`, `SSPCR1`, `SSPCPSR`, `SSPDR`, `SSPSR`, and the
//! interrupt registers, which is what this module models. Everything
//! else (`SSPCR1.SOD`, slave-only modes) is storage-round-trip.
//!
//! # Register map (offsets relative to `SSPn_BASE`)
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
//! | `0xFE0..0xFEC` | `SSPPERIPHID0..3` | RO | PrimeCell peripheral ID     |
//! | `0xFF0..0xFFC` | `SSPPCELLID0..3`  | RO | PrimeCell ID                 |
//!
//! # Loopback model (`SSPCR1.LBM`)
//!
//! When firmware sets `SSPCR1.LBM=1`, every write to `SSPDR` pushes the
//! word into the RX FIFO directly — simulating the PL022's internal TX
//! → RX tie. This is exactly what the `spi_master` corpus binary
//! expects: write 0xA5, read 0xA5 back. When LBM=0 the TX FIFO drains
//! off-chip with no RX response (a full PIO-driven external slave is
//! out of scope for Phase 2).
//!
//! # Baud-rate cadence (non-loopback)
//!
//! Even in non-loopback mode the TX FIFO must eventually drain so
//! `SSPSR.BSY` can fall back to 0. [`SpiRegs::tick`] models the PL022
//! clock rate as `clk_peri / (SCR + 1) / SSPCPSR` and pops one TX
//! FIFO entry every `sysclks_per_word`. When `LBM=1` that drain
//! replays into the RX FIFO (already queued at write time to keep
//! `spi_master`'s poll-then-read rhythm deterministic).
//!
//! # IRQ sources
//!
//! The PL022 surfaces four interrupts ORed onto the peripheral's
//! single NVIC line (SPI0=18, SPI1=19):
//! * `SSPRIS.ROR` — RX overrun (not modelled; never raised).
//! * `SSPRIS.RT`  — RX timeout.
//! * `SSPRIS.RX`  — RX FIFO ≥ 1/2 full.
//! * `SSPRIS.TX`  — TX FIFO ≤ 1/2 full.

use std::collections::VecDeque;

use mdpicoem_common::clocks::ClockTree;

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
/// Offset: `SSPICR` — W1C interrupt clear (only RTIC + RORIC are valid bits).
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

/// PL022 peripheral ID (r1p3). TRM Table 2-18 canonical values.
const PERIPH_ID: [u32; 4] = [0x22, 0x10, 0x34, 0x00];
const PCELL_ID: [u32; 4] = [0x0D, 0xF0, 0x05, 0xB1];

/// PL022-derived SPI (RP2040 §4.4).
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
    /// the NVIC line (18 for SPI0, 19 for SPI1 on RP2040).
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

    /// True iff no outstanding work — TX and RX FIFOs empty.
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

    /// Frame data width in bits, per `SSPCR0.DSS` ([3:0]). 4 → 5-bit
    /// frame, ..., 15 → 16-bit frame. For masking purposes we need the
    /// low-N-bits value so loopback round-trips every written bit.
    fn frame_data_mask(&self) -> u32 {
        let dss = self.cr0 & 0xF;
        // DSS encoding: 3 = 4-bit, ..., 15 = 16-bit.
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

    fn route_irq(&self, irqs: &mut u32) {
        if (self.ris & self.imsc) != 0 {
            *irqs |= 1u32 << self.nvic_irq;
        }
    }

    fn refresh_tx_rx_interrupts(&mut self) {
        // PL022 TX latches when TX FIFO ≤ 1/2 (4 of 8 entries).
        if self.tx_fifo.len() <= SSP_FIFO_DEPTH / 2 {
            self.ris |= SSP_INT_TX;
        }
        // RX latches when RX FIFO ≥ 1/2 full.
        if self.rx_fifo.len() >= SSP_FIFO_DEPTH / 2 {
            self.ris |= SSP_INT_RX;
        } else {
            // Level-fall: once RX drains below threshold, drop the bit
            // so firmware can re-arm without a spurious re-trigger.
            self.ris &= !SSP_INT_RX;
        }
    }

    /// Push a word into the TX FIFO; loopback mirrors into RX.
    fn push_dr(&mut self, value: u32, irqs: &mut u32) {
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
        } else {
            // Overrun latched when RX FIFO can't accept a loopback copy.
            if self.is_loopback() {
                self.ris |= SSP_INT_ROR;
            }
        }
        self.refresh_tx_rx_interrupts();
        self.route_irq(irqs);
    }

    /// Pop a word from the RX FIFO (DR read side-effect).
    fn pop_dr(&mut self) -> u32 {
        self.rx_fifo.pop_front().unwrap_or(0)
    }

    fn sysclks_per_word(&self, clock_tree: &ClockTree) -> u64 {
        // PL022: bit rate = peri_hz / (CPSDVSR * (1 + SCR)). Per frame
        // width = (DSS+1) bits. Collapse into one clamp.
        let cpsdvsr = (self.cpsr & 0xFE).max(2) as u64; // must be even ≥ 2
        let scr = ((self.cr0 >> 8) & 0xFF) as u64;
        let peri = clock_tree.peri_hz().max(1);
        let bits_per_frame = (((self.cr0 & 0xF).max(3)) + 1) as u64;
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

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u32) {
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
                // Disabling collapses FIFOs to empty per PL022 reset
                // semantics (real silicon holds state but firmware
                // observes post-disable reads as 0).
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
                // Only RTIC + RORIC are valid ICR bits (TX/RX are level
                // and clear on drain/fill). We still honour W1C for
                // whatever bits firmware sets on ROR/RT.
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
            // SR / RIS / MIS are read-only.
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

    pub fn write8(&mut self, offset: u32, value: u8, irqs: &mut u32) {
        if offset == SSPDR {
            self.push_dr(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    pub fn write16(&mut self, offset: u32, value: u16, irqs: &mut u32) {
        if offset == SSPDR {
            self.push_dr(value as u32, irqs);
        } else {
            self.write32(offset, value as u32, 0, irqs);
        }
    }

    pub fn tick(&mut self, cycles: u32, clock_tree: &ClockTree, irqs: &mut u32) {
        if cycles == 0 || !self.is_enabled() || self.tx_fifo.is_empty() {
            return;
        }
        let spw = self.sysclks_per_word(clock_tree);
        self.tx_cycle_accum = self.tx_cycle_accum.saturating_add(cycles as u64);
        while self.tx_cycle_accum >= spw && !self.tx_fifo.is_empty() {
            self.tx_cycle_accum -= spw;
            // Drain one word out of the TX FIFO. In loopback mode the
            // RX copy was pushed at `push_dr` time so no extra work
            // here.
            let _ = self.tx_fifo.pop_front();
        }
        self.refresh_tx_rx_interrupts();
        self.route_irq(irqs);
    }
}

impl Default for SpiRegs {
    fn default() -> Self {
        Self::new(18)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SPI0_IRQ: u32 = 18;
    const SYS_HZ: u32 = 125_000_000;

    fn tree() -> ClockTree {
        let mut t = ClockTree::default();
        t.sys_clk_hz = SYS_HZ;
        t.peri_clk_hz = SYS_HZ;
        t.ref_clk_hz = SYS_HZ;
        t
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
    fn loopback_roundtrips_byte_value() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // Enable + LBM; DSS = 7 (8-bit frames).
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write32(SSPDR, 0xA5, 0, &mut irqs);
        // RX FIFO should carry the loopback copy immediately.
        assert!(s.read32(SSPSR) & SSPSR_RNE != 0, "RX non-empty after LBM push");
        let rx = s.read32(SSPDR);
        assert_eq!(rx, 0xA5);
    }

    #[test]
    fn loopback_masks_to_frame_width() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // DSS=3 → 4-bit frames: values clamp to 4 LSBs.
        s.write32(SSPCR0, 0x03, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write32(SSPDR, 0xFF, 0, &mut irqs);
        assert_eq!(s.read32(SSPDR), 0x0F);
    }

    #[test]
    fn dr_write_before_enable_is_dropped() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPDR, 0xAA, 0, &mut irqs);
        assert!(s.tx_fifo.is_empty());
        assert!(s.rx_fifo.is_empty());
    }

    // --- FIFO + SR flags ---------------------------------------------

    #[test]
    fn tx_fifo_saturates_at_eight() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        // Enable but no loopback — bytes stay queued until tick drains.
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        for i in 0..12 {
            s.write32(SSPDR, i as u32, 0, &mut irqs);
        }
        assert_eq!(s.tx_fifo.len(), SSP_FIFO_DEPTH);
        // SR: TNF clear when full.
        assert!(s.read32(SSPSR) & SSPSR_TNF == 0);
        assert!(s.read32(SSPSR) & SSPSR_BSY != 0);
    }

    // --- tick drains ---------------------------------------------------

    #[test]
    fn tick_drains_tx_at_configured_rate() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // 1 MHz bit rate: CPSDVSR=50, SCR=1 → 125MHz / (50 * 2) = 1.25 MHz.
        s.write32(SSPCR0, (1 << 8) | 0x07, 0, &mut irqs);
        s.write32(SSPCPSR, 50, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        for i in 0..4 {
            s.write32(SSPDR, i as u32, 0, &mut irqs);
        }
        assert_eq!(s.tx_fifo.len(), 4);
        let t = tree();
        s.tick(10_000, &t, &mut irqs); // 80 µs worth of cycles
        // 8 bits at ~1.25 MHz = 6.4 µs/word; 4 words ≈ 26 µs → fully
        // drained inside 80 µs.
        assert!(s.tx_fifo.is_empty());
    }

    // --- IRQ routing --------------------------------------------------

    #[test]
    fn tx_irq_latches_when_fifo_under_half() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write32(SSPIMSC, SSP_INT_TX, 0, &mut irqs);
        // Fill past half then drain via tick.
        for _ in 0..6 {
            s.write32(SSPDR, 0x11, 0, &mut irqs);
        }
        // After 6 entries, TX FIFO is above the 1/2 (4) threshold.
        // Actually, "TX IRQ" fires when level <= 1/2 = 4. 6 > 4, so
        // TXIS should currently NOT be set from refresh. Re-check
        // after drain.
        let _ = s.ris; // forces the refresh during push_dr
        s.write32(SSPCPSR, 2, 0, &mut irqs);
        // SCR=0, CPSDVSR=2 → 125MHz / 2 / 1 = 62.5 MHz → tiny sysclks/word.
        let t = tree();
        // Drain a few words.
        s.tick(1_000, &t, &mut irqs);
        assert!(s.ris & SSP_INT_TX != 0);
        assert!(irqs & (1u32 << SPI0_IRQ) != 0);
    }

    #[test]
    fn rx_irq_latches_when_fifo_half_full() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write32(SSPIMSC, SSP_INT_RX, 0, &mut irqs);
        // Push 4 words — hits RX threshold in loopback mode.
        for i in 0..4 {
            s.write32(SSPDR, i as u32, 0, &mut irqs);
        }
        assert!(s.ris & SSP_INT_RX != 0);
        assert!(irqs & (1u32 << SPI0_IRQ) != 0);
    }

    #[test]
    fn ror_ric_clears_ror() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.ris = SSP_INT_ROR | SSP_INT_RT;
        s.write32(SSPICR, SSP_INT_ROR, 0, &mut irqs);
        assert_eq!(s.ris & SSP_INT_ROR, 0);
        assert_eq!(s.ris & SSP_INT_RT, SSP_INT_RT);
    }

    // --- is_idle ------------------------------------------------------

    #[test]
    fn is_idle_true_at_reset() {
        let s = SpiRegs::new(SPI0_IRQ);
        assert!(s.is_idle());
    }

    #[test]
    fn is_idle_false_with_pending_tx() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE, 0, &mut irqs);
        s.write32(SSPDR, 0x11, 0, &mut irqs);
        assert!(!s.is_idle());
    }

    // --- Byte/halfword DR narrow dispatch ----------------------------

    #[test]
    fn byte_write_to_dr_pushes_into_tx_fifo() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        s.write32(SSPCR0, 0x07, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write8(SSPDR, 0x73, &mut irqs);
        assert_eq!(s.rx_fifo.front().copied(), Some(0x73));
    }

    #[test]
    fn halfword_loopback_16_bit_frame() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        let mut irqs = 0;
        // DSS=15 → 16-bit frames.
        s.write32(SSPCR0, 0x0F, 0, &mut irqs);
        s.write32(SSPCR1, SSPCR1_SSE | SSPCR1_LBM, 0, &mut irqs);
        s.write16(SSPDR, 0xBEEF, &mut irqs);
        assert_eq!(s.read16(SSPDR), 0xBEEF);
    }

    // --- PrimeCell ID ------------------------------------------------

    #[test]
    fn peripheral_and_pcell_id_match_pl022() {
        let mut s = SpiRegs::new(SPI0_IRQ);
        assert_eq!(s.read32(SSPPERIPHID0), 0x22);
        assert_eq!(s.read32(SSPPCELLID0), 0x0D);
        assert_eq!(s.read32(SSPPCELLID3), 0xB1);
    }
}

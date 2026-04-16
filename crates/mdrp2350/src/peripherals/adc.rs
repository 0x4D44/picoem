//! RP2350 ADC peripheral (datasheet §12.4).
//!
//! Phase 2 of the RP2350 peripheral coverage plan (HLD V5 §6 row 2).
//! Single instance at `0x400A_0000`. Reset-gated on RESETS bit 0.
//!
//! Mirrors the RP2040 ADC (`mdrp2040::peripherals::adc`) verbatim —
//! same CS/RESULT/FCS/FIFO/DIV/INTR/INTE/INTF/INTS register surface,
//! same fixed-point `clk_adc`/`clk_sys` scaling. The RP2350 deltas are
//! the NVIC IRQ number ([`crate::irq::IRQ_ADC_IRQ_FIFO`] = 35, a `u64`
//! bit) and the nominal `clk_adc` / `clk_sys` frequencies used by the
//! accumulator:
//!
//! * `ADC_HZ` uses [`mdpicoem_common::clocks::RP2350_ADC_CLK_HZ`]
//!   (48 MHz on RP2350, same as RP2040 in post-bootrom state).
//! * `sys_clk_hz` comes from the current `ClockTree` (seeded to
//!   150 MHz at emulator reset per HLD V5 §5.7).
//!
//! # Clock scaling
//!
//! Per HLD V5 §6 row 2 and the RP2040 V7 idiom: the ADC `tick`
//! accumulator advances each `clk_sys` cycle:
//!
//! ```text
//! adc_phase += ADC_HZ;
//! while adc_phase >= SYS_HZ { adc_phase -= SYS_HZ; adc_subtick(); }
//! ```
//!
//! Each `adc_subtick` decrements the in-flight conversion counter;
//! when it hits zero a sample is pushed.

use std::collections::VecDeque;

use mdpicoem_common::clocks::{ClockTree, RP2350_ADC_CLK_HZ};

use crate::irq::IRQ_ADC_IRQ_FIFO;

use super::apply_alias_rmw;

/// ADC base (RP2350 datasheet §12.4).
pub const ADC_BASE: u32 = 0x400A_0000;

// --- Register offsets -------------------------------------------------

pub const CS: u32 = 0x00;
pub const RESULT: u32 = 0x04;
pub const FCS: u32 = 0x08;
pub const FIFO: u32 = 0x0C;
pub const DIV: u32 = 0x10;
pub const INTR: u32 = 0x14;
pub const INTE: u32 = 0x18;
pub const INTF: u32 = 0x1C;
pub const INTS: u32 = 0x20;

// --- CS bits ---------------------------------------------------------

pub const CS_EN: u32 = 1 << 0;
pub const CS_TS_EN: u32 = 1 << 1;
pub const CS_START_ONCE: u32 = 1 << 2;
pub const CS_START_MANY: u32 = 1 << 3;
pub const CS_READY: u32 = 1 << 8;
pub const CS_ERR: u32 = 1 << 9;
pub const CS_ERR_STICKY: u32 = 1 << 10;
const CS_AINSEL_SHIFT: u32 = 12;
const CS_AINSEL_MASK: u32 = 0x7 << CS_AINSEL_SHIFT;
const CS_RROBIN_SHIFT: u32 = 16;
const CS_RROBIN_MASK: u32 = 0x1F << CS_RROBIN_SHIFT;

const CS_WRITE_MASK: u32 = CS_EN
    | CS_TS_EN
    | CS_START_ONCE
    | CS_START_MANY
    | CS_ERR_STICKY
    | CS_AINSEL_MASK
    | CS_RROBIN_MASK;

// --- FCS bits --------------------------------------------------------

pub const FCS_EN: u32 = 1 << 0;
pub const FCS_SHIFT: u32 = 1 << 1;
pub const FCS_ERR: u32 = 1 << 2;
pub const FCS_DREQ_EN: u32 = 1 << 3;
const FCS_EMPTY: u32 = 1 << 8;
const FCS_FULL: u32 = 1 << 9;
pub const FCS_UNDER: u32 = 1 << 10;
pub const FCS_OVER: u32 = 1 << 11;
const FCS_LEVEL_SHIFT: u32 = 16;
const FCS_LEVEL_MASK: u32 = 0xF << FCS_LEVEL_SHIFT;
pub const FCS_THRESH_SHIFT: u32 = 24;
pub const FCS_THRESH_MASK: u32 = 0xF << FCS_THRESH_SHIFT;

const FCS_WRITE_MASK: u32 = FCS_EN | FCS_SHIFT | FCS_ERR | FCS_DREQ_EN | FCS_THRESH_MASK;

// --- INTR bits -------------------------------------------------------

pub const INTR_FIFO: u32 = 1 << 0;

// --- Conversion timing ----------------------------------------------

/// ADC clock frequency — 48 MHz on RP2350 (USB PLL ÷ 10).
pub const ADC_HZ: u32 = RP2350_ADC_CLK_HZ;

/// Number of `clk_adc` ticks per conversion (datasheet §12.4.1).
pub const CONVERSION_ADC_TICKS: u32 = 96;

/// FIFO depth — four 12/16-bit entries (datasheet §12.4.1).
pub const ADC_FIFO_DEPTH: usize = 4;

/// ADC register storage.
pub struct AdcRegs {
    cs: u32,
    fcs: u32,
    div: u32,
    intr: u32,
    inte: u32,
    intf: u32,
    fifo: VecDeque<u16>,
    last_sample: u16,
    adc_phase: u64,
    conversion_remaining: Option<u32>,
    conversion_counter: u32,
    nvic_irq: u32,
}

impl AdcRegs {
    /// Construct a fresh ADC at power-on defaults. `nvic_irq` is the
    /// NVIC line (35 for ADC_IRQ_FIFO on RP2350).
    pub fn new(nvic_irq: u32) -> Self {
        Self {
            cs: 0,
            fcs: 0,
            div: 0,
            intr: 0,
            inte: 0,
            intf: 0,
            fifo: VecDeque::with_capacity(ADC_FIFO_DEPTH),
            last_sample: 0,
            adc_phase: 0,
            conversion_remaining: None,
            conversion_counter: 0,
            nvic_irq,
        }
    }

    pub fn reset(&mut self) {
        let irq = self.nvic_irq;
        *self = Self::new(irq);
    }

    pub fn fifo_len(&self) -> usize {
        self.fifo.len()
    }

    /// True iff no conversion is running and the FIFO is empty.
    pub fn is_idle(&self) -> bool {
        self.conversion_remaining.is_none()
            && (self.cs & (CS_START_ONCE | CS_START_MANY)) == 0
            && self.fifo.is_empty()
            && self.intr == 0
    }

    /// DREQ: ADC FIFO non-empty and `FCS.DREQ_EN` is set. Consumed by
    /// the DMA DREQ matrix (Phase 3).
    #[inline]
    pub fn dreq(&self) -> bool {
        (self.fcs & FCS_DREQ_EN) != 0 && !self.fifo.is_empty()
    }

    #[inline]
    fn is_enabled(&self) -> bool {
        (self.cs & CS_EN) != 0
    }

    #[inline]
    fn ainsel(&self) -> u32 {
        (self.cs & CS_AINSEL_MASK) >> CS_AINSEL_SHIFT
    }

    #[inline]
    fn fcs_enabled(&self) -> bool {
        (self.fcs & FCS_EN) != 0
    }

    #[inline]
    fn fcs_thresh(&self) -> u32 {
        (self.fcs & FCS_THRESH_MASK) >> FCS_THRESH_SHIFT
    }

    fn fcs_read(&self) -> u32 {
        let base = self.fcs & !(FCS_LEVEL_MASK | FCS_EMPTY | FCS_FULL);
        let level = (self.fifo.len() as u32) << FCS_LEVEL_SHIFT;
        let mut extras = level & FCS_LEVEL_MASK;
        if self.fifo.is_empty() {
            extras |= FCS_EMPTY;
        }
        if self.fifo.len() >= ADC_FIFO_DEPTH {
            extras |= FCS_FULL;
        }
        base | extras
    }

    /// Deterministic 12-bit sample for a given channel. Firmware needs
    /// non-zero, varying data without a modelled analog frontend.
    ///
    /// Returns a raw 12-bit result (bits [11:0] only). The caller is
    /// responsible for packing this into the hardware FIFO format
    /// `ERR[15]:AINSEL[14:12]:RESULT[11:0]`.
    #[inline]
    fn make_sample(&self, channel: u32) -> u16 {
        let payload = ((channel & 0xF) << 8) | (self.conversion_counter & 0xFF);
        (payload & 0xFFF) as u16
    }

    /// Pack a raw 12-bit sample into the hardware FIFO word format:
    /// `ERR[15]:AINSEL[14:12]:RESULT[11:0]` (datasheet §12.4.5 FIFO
    /// register). ERR is always 0 (no error modelling).
    #[inline]
    fn hw_fifo_word(channel: u32, raw_sample: u16) -> u16 {
        ((channel as u16 & 0x7) << 12) | (raw_sample & 0xFFF)
    }

    /// Fire a single completed conversion.
    fn complete_conversion(&mut self) -> bool {
        let ch = self.ainsel();
        let raw = self.make_sample(ch);
        let hw_word = Self::hw_fifo_word(ch, raw);
        // RESULT register holds the raw 12-bit sample (datasheet §12.4.2).
        self.last_sample = raw;
        self.conversion_counter = self.conversion_counter.wrapping_add(1);
        self.cs |= CS_READY;

        let mut fifo_edge = false;
        if self.fcs_enabled() {
            if self.fifo.len() >= ADC_FIFO_DEPTH {
                self.fcs |= FCS_OVER;
            } else {
                self.fifo.push_back(hw_word);
                fifo_edge = true;
            }
        }

        // Round-robin channel advancement (datasheet §12.4.3 CS.RROBIN).
        let rrobin = (self.cs & CS_RROBIN_MASK) >> CS_RROBIN_SHIFT;
        if rrobin != 0 {
            let current = ch;
            let mut next = (current + 1) % 5; // 5 ADC channels on RP2350
            while rrobin & (1 << next) == 0 {
                next = (next + 1) % 5;
            }
            self.cs = (self.cs & !CS_AINSEL_MASK) | (next << CS_AINSEL_SHIFT);
        }

        self.cs &= !CS_START_ONCE;
        self.conversion_remaining = None;
        fifo_edge
    }

    fn refresh_intr(&mut self) {
        let thresh = self.fcs_thresh();
        if self.fcs_enabled()
            && thresh > 0
            && (self.fifo.len() as u32) >= thresh
        {
            self.intr |= INTR_FIFO;
        } else {
            self.intr &= !INTR_FIFO;
        }
    }

    fn route_irq(&self, irqs: &mut u64) {
        if ((self.intr | self.intf) & self.inte) != 0 {
            *irqs |= 1u64 << self.nvic_irq;
        }
    }

    fn maybe_start(&mut self) {
        if !self.is_enabled() {
            return;
        }
        if self.conversion_remaining.is_some() {
            return;
        }
        let should_start = (self.cs & (CS_START_ONCE | CS_START_MANY)) != 0;
        if should_start {
            self.conversion_remaining = Some(CONVERSION_ADC_TICKS);
            self.cs &= !CS_READY;
        }
    }

    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            CS => self.cs,
            RESULT => self.last_sample as u32,
            FCS => self.fcs_read(),
            FIFO => self.fifo_pop_word(),
            DIV => self.div,
            INTR => self.intr,
            INTE => self.inte,
            INTF => self.intf,
            INTS => (self.intr | self.intf) & self.inte,
            _ => 0,
        }
    }

    pub fn read16(&mut self, offset: u32) -> u16 {
        if offset == FIFO {
            self.fifo_pop_sample()
        } else {
            self.read32(offset) as u16
        }
    }

    fn fifo_pop_sample(&mut self) -> u16 {
        if let Some(sample) = self.fifo.pop_front() {
            self.refresh_intr();
            if (self.fcs & FCS_SHIFT) != 0 {
                sample >> 4
            } else {
                sample
            }
        } else {
            self.fcs |= FCS_UNDER;
            0
        }
    }

    fn fifo_pop_word(&mut self) -> u32 {
        self.fifo_pop_sample() as u32
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u64) {
        match offset {
            CS => {
                let old_cs = self.cs;
                let mut stored = self.cs;
                apply_alias_rmw(&mut stored, value, alias);
                self.cs = (stored & CS_WRITE_MASK) | (self.cs & (CS_READY | CS_ERR));
                let en_before = (old_cs & CS_EN) != 0;
                let en_after = (self.cs & CS_EN) != 0;
                if !en_before && en_after && self.conversion_remaining.is_none() {
                    self.cs |= CS_READY;
                } else if en_before && !en_after {
                    self.conversion_remaining = None;
                    self.cs &= !CS_READY;
                }
                self.maybe_start();
            }
            RESULT => {}
            FCS => {
                let sticky = self.fcs & (FCS_UNDER | FCS_OVER);
                let mut new_ctrl = self.fcs & FCS_WRITE_MASK;
                apply_alias_rmw(&mut new_ctrl, value, alias);
                self.fcs = (new_ctrl & FCS_WRITE_MASK) | sticky;
                // FCS.UNDER/OVER are W1C with-a-twist: datasheet §12.4.5
                // lists them as write-1-to-clear, but only normal stores
                // (alias=0) and BITSET stores (alias=2, writing 1s) can
                // land a 1 in the sticky bit. BITCLR (alias=3, writing
                // `clear_mask`) and XOR (alias=1) would paradoxically
                // clear on their reverse semantics — we drop them.
                if alias == 0 || alias == 2 {
                    let w1c_mask = value & (FCS_UNDER | FCS_OVER);
                    self.fcs &= !w1c_mask;
                }
                if !self.fcs_enabled() {
                    self.fifo.clear();
                }
                self.refresh_intr();
                self.route_irq(irqs);
            }
            FIFO => {}
            DIV => {
                let mut stored = self.div;
                apply_alias_rmw(&mut stored, value, alias);
                self.div = stored & 0x00FF_FFFF;
            }
            INTR => {}
            INTE => {
                let mut stored = self.inte;
                apply_alias_rmw(&mut stored, value, alias);
                self.inte = stored & INTR_FIFO;
                self.route_irq(irqs);
            }
            INTF => {
                let mut stored = self.intf;
                apply_alias_rmw(&mut stored, value, alias);
                self.intf = stored & INTR_FIFO;
                self.route_irq(irqs);
            }
            INTS => {}
            _ => {}
        }
    }

    pub fn read8(&mut self, offset: u32) -> u8 {
        if offset == FIFO {
            self.fifo_pop_sample() as u8
        } else {
            self.read32(offset) as u8
        }
    }

    pub fn write8(&mut self, _offset: u32, _value: u8, _irqs: &mut u64) {
        // No byte-significant writes on the ADC surface.
    }

    /// Advance the ADC by `sys_cycles` `clk_sys` ticks.
    pub fn tick(&mut self, sys_cycles: u32, clock_tree: &ClockTree, irqs: &mut u64) {
        if sys_cycles == 0 {
            return;
        }
        self.maybe_start();

        if self.conversion_remaining.is_none() && (self.cs & CS_START_MANY) == 0 {
            self.route_irq(irqs);
            return;
        }

        let sys_hz = clock_tree.sys_clk_hz.max(1) as u64;
        self.adc_phase = self.adc_phase.saturating_add((ADC_HZ as u64) * (sys_cycles as u64));

        let mut fired = false;
        while self.adc_phase >= sys_hz {
            self.adc_phase -= sys_hz;
            self.maybe_start();
            if let Some(rem) = self.conversion_remaining.as_mut() {
                if *rem > 1 {
                    *rem -= 1;
                } else {
                    let _ = self.complete_conversion();
                    fired = true;
                }
            } else if (self.cs & CS_START_MANY) == 0 {
                break;
            }
        }

        if fired {
            self.refresh_intr();
        }
        self.route_irq(irqs);
    }
}

impl Default for AdcRegs {
    fn default() -> Self {
        Self::new(IRQ_ADC_IRQ_FIFO)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ADC_IRQ: u32 = IRQ_ADC_IRQ_FIFO;
    const SYS_HZ: u32 = 150_000_000;

    fn default_tree() -> ClockTree {
        ClockTree {
            sys_clk_hz: SYS_HZ,
            ref_clk_hz: 12_000_000,
            peri_clk_hz: SYS_HZ,
        }
    }

    // --- Reset / defaults ------------------------------------------------

    #[test]
    fn reset_defaults_all_zero() {
        let a = AdcRegs::new(ADC_IRQ);
        assert_eq!(a.cs, 0);
        assert_eq!(a.last_sample, 0);
        assert!(a.fifo.is_empty());
        assert!(a.is_idle());
    }

    // --- CS write sanitisation ------------------------------------------

    #[test]
    fn cs_write_cannot_set_ready_directly() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(CS, CS_READY, 0, &mut irqs);
        assert_eq!(a.cs & CS_READY, 0);
    }

    #[test]
    fn cs_en_alone_latches_ready() {
        // Mirrors RP2040 ADC Phase A idiom: setting CS.EN from 0→1
        // with no START_* pending latches READY immediately.
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(CS, CS_EN, 0, &mut irqs);
        assert_ne!(a.cs & CS_READY, 0);
    }

    // --- One-shot conversion --------------------------------------------

    #[test]
    fn start_once_requires_enable() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(CS, CS_START_ONCE, 0, &mut irqs);
        assert!(a.conversion_remaining.is_none());
    }

    #[test]
    fn start_once_arms_conversion() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        assert_eq!(a.conversion_remaining, Some(CONVERSION_ADC_TICKS));
        assert_eq!(a.cs & CS_READY, 0);
    }

    #[test]
    fn start_once_completes_sets_ready() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(CS, CS_EN | CS_START_ONCE | (3 << CS_AINSEL_SHIFT), 0, &mut irqs);
        // 96 adc ticks @ 48 MHz → 96*150/48 = 300 sys_clks.
        a.tick(500, &default_tree(), &mut irqs);
        assert_eq!(a.cs & CS_READY, CS_READY);
        assert_eq!(a.cs & CS_START_ONCE, 0, "START_ONCE auto-clears");
        assert_ne!(a.read32(RESULT), 0);
    }

    #[test]
    fn start_many_keeps_converting() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(FCS, FCS_EN, 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_MANY, 0, &mut irqs);
        a.tick(2_500, &default_tree(), &mut irqs);
        assert!(a.fifo.len() >= 2);
    }

    // --- Clock scaling ---------------------------------------------------

    #[test]
    fn clk_adc_scaling_matches_ratio() {
        // 150 sys_clks * 48/150 = 48 adc sub-ticks.
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        a.tick(150, &default_tree(), &mut irqs);
        assert_eq!(a.conversion_remaining, Some(CONVERSION_ADC_TICKS - 48));
    }

    // --- FIFO + threshold IRQ --------------------------------------------

    #[test]
    fn fifo_level_meets_thresh_raises_irq() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(FCS, FCS_EN | (4 << FCS_THRESH_SHIFT), 0, &mut irqs);
        a.write32(INTE, INTR_FIFO, 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_MANY, 0, &mut irqs);
        a.tick(3_000, &default_tree(), &mut irqs);
        assert_eq!(a.fifo.len(), ADC_FIFO_DEPTH);
        assert_eq!(a.intr & INTR_FIFO, INTR_FIFO);
        assert_ne!(irqs & (1u64 << ADC_IRQ), 0);
    }

    #[test]
    fn fifo_pop_drops_intr() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(FCS, FCS_EN | (1 << FCS_THRESH_SHIFT), 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        a.tick(500, &default_tree(), &mut irqs);
        assert_eq!(a.fifo.len(), 1);
        assert_eq!(a.intr & INTR_FIFO, INTR_FIFO);
        let _ = a.read32(FIFO);
        assert_eq!(a.intr & INTR_FIFO, 0);
    }

    #[test]
    fn fifo_pop_empty_sets_under() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let v = a.read32(FIFO);
        assert_eq!(v, 0);
        assert_ne!(a.fcs_read() & FCS_UNDER, 0);
    }

    // --- Alias semantics -------------------------------------------------

    #[test]
    fn cs_bitset_alias() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(CS, CS_EN, 0, &mut irqs);
        a.write32(CS, CS_START_ONCE, 2, &mut irqs);
        assert!(a.cs & CS_EN != 0);
    }

    // --- DIV storage -----------------------------------------------------

    #[test]
    fn div_round_trip() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u64;
        a.write32(DIV, 0x0012_3456, 0, &mut irqs);
        assert_eq!(a.read32(DIV), 0x0012_3456);
    }
}

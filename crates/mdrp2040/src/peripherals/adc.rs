//! RP2040 ADC peripheral (datasheet §4.9).
//!
//! Phase 3 of the RP2040 peripheral coverage plan (HLD V7 §5.3 / §6).
//! Single instance at `0x4004_C000`. Reset-gated on RESETS bit 0.
//! pico-sdk's `adc/hello_adc` exercises `CS`, `RESULT`, `FCS`, `FIFO`,
//! `DIV`, and the INTR/INTE/INTF/INTS group. That register set is what
//! this module models; extras (round-robin + channel-select details,
//! temperature-sensor compensation, DREQ wiring) are deferred.
//!
//! # Clock scaling
//!
//! ADC runs on `clk_adc` (48 MHz nominal) — different from `clk_sys`
//! (125 MHz default). Per HLD V7 §5.3 the emulator scales with a
//! fixed-point accumulator that advances each `clk_sys` tick:
//!
//! ```text
//! adc_phase += ADC_HZ;
//! while adc_phase >= SYS_HZ { adc_phase -= SYS_HZ; adc_subtick(); }
//! ```
//!
//! Each `adc_subtick` decrements the in-flight conversion counter;
//! when it hits zero a sample is pushed.
//!
//! # Conversion model
//!
//! Hardware takes 96 adc_clk cycles per conversion (≈2 µs at 48 MHz).
//! We model that as a deterministic countdown in `adc_clk` ticks
//! ([`CONVERSION_ADC_TICKS`]). The sample itself is a deterministic
//! pattern keyed to channel + sample index so firmware sees varying,
//! plausible values without our having to model the analog frontend.
//!
//! # Register map (offsets relative to `ADC_BASE`)
//!
//! | Offset | Name     | Access  | Notes                             |
//! |--------|----------|---------|-----------------------------------|
//! | `0x00` | `CS`     | R/W     | EN, TS_EN, START_*, READY, AINSEL |
//! | `0x04` | `RESULT` | R       | Latest 12-bit sample.             |
//! | `0x08` | `FCS`    | R/W     | FIFO control + status.            |
//! | `0x0C` | `FIFO`   | R       | Pop one sample (side-effect).     |
//! | `0x10` | `DIV`    | R/W     | 16.8 clock divisor (storage).     |
//! | `0x14` | `INTR`   | R       | Raw interrupt status.             |
//! | `0x18` | `INTE`   | R/W     | Interrupt enable.                 |
//! | `0x1C` | `INTF`   | R/W     | Interrupt force.                  |
//! | `0x20` | `INTS`   | R       | (INTR \| INTF) & INTE.            |
//!
//! # Deferred from Phase 3
//!
//! * DREQ wiring for DMA paced reads (Phase 4).
//! * Analog frontend — samples are a deterministic `channel | index`
//!   pattern, not a modelled voltage curve.
//! * Temperature-sensor `TS_EN` path — TS_EN is storage-round-trip.
//! * `AINSEL` round-robin scheduling beyond the single currently-
//!   selected channel (RROBIN bits stored but not advanced).

use std::collections::VecDeque;

use mdpicoem_common::clocks::ClockTree;

use super::apply_alias_rmw;

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
const FCS_THRESH_SHIFT: u32 = 24;
const FCS_THRESH_MASK: u32 = 0xF << FCS_THRESH_SHIFT;

/// Bits that may be written by firmware (EMPTY/FULL/LEVEL are derived;
/// UNDER/OVER are W1C).
const FCS_WRITE_MASK: u32 = FCS_EN | FCS_SHIFT | FCS_ERR | FCS_DREQ_EN | FCS_THRESH_MASK;

// --- INTR bits -------------------------------------------------------

/// Raw interrupt: FIFO level >= FCS.THRESH.
pub const INTR_FIFO: u32 = 1 << 0;

// --- Conversion timing ----------------------------------------------

/// Nominal ADC clock frequency (RP2040 datasheet §4.9.2.1). Used for
/// the fixed-point accumulator that scales `clk_sys` cycles into
/// `clk_adc` sub-ticks.
pub const ADC_HZ: u32 = 48_000_000;

/// Number of `clk_adc` ticks per conversion. Datasheet §4.9.1:
/// each conversion takes 96 cycles of `clk_adc` (≈2 µs at 48 MHz).
pub const CONVERSION_ADC_TICKS: u32 = 96;

/// FIFO depth — four 12/16-bit entries (datasheet §4.9.1 Figure 38).
pub const ADC_FIFO_DEPTH: usize = 4;

/// ADC register storage.
pub struct AdcRegs {
    cs: u32,
    fcs: u32,
    div: u32,
    intr: u32,
    inte: u32,
    intf: u32,
    /// 12-bit samples; FIFO depth four.
    fifo: VecDeque<u16>,
    /// Latest 12-bit sample (RESULT register mirrors this).
    last_sample: u16,
    /// Fixed-point clk_adc accumulator. See module docstring.
    adc_phase: u64,
    /// Running conversion's remaining adc_clk ticks. `None` = idle.
    conversion_remaining: Option<u32>,
    /// Monotonic conversion counter used to vary the deterministic
    /// sample pattern (low 8 bits per-channel).
    conversion_counter: u32,
    nvic_irq: u32,
}

impl AdcRegs {
    /// Construct a fresh ADC at power-on defaults. `nvic_irq` is the
    /// NVIC line (22 for ADC_IRQ_FIFO on RP2040).
    pub fn new(nvic_irq: u32) -> Self {
        Self {
            cs: 0,
            fcs: 0,
            // Datasheet §4.9.6: DIV reset value is 0 (no clock division).
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

    /// Current FIFO occupancy. Useful for bus-level integration tests
    /// that verify the narrow-dispatch path doesn't double-pop.
    pub fn fifo_len(&self) -> usize {
        self.fifo.len()
    }

    /// True iff no conversion is running and the FIFO is empty
    /// (HLD V7 §5.3: START_ONCE == 0 && START_MANY == 0).
    pub fn is_idle(&self) -> bool {
        self.conversion_remaining.is_none()
            && (self.cs & (CS_START_ONCE | CS_START_MANY)) == 0
            && self.fifo.is_empty()
            && self.intr == 0
    }

    /// DREQ: FIFO level crosses `FCS.THRESH` with `FCS.DREQ_EN` set.
    /// Phase 4 DMA TREQ `DREQ_ADC` consults this — firmware that wants
    /// DMA'd samples writes `FCS.EN=1, DREQ_EN=1, THRESH=n` and the DMA
    /// kicks whenever the FIFO is at or above that level. If `DREQ_EN`
    /// is clear the DREQ never asserts even while samples are waiting.
    #[inline]
    pub fn dreq(&self) -> bool {
        if !self.fcs_enabled() || (self.fcs & FCS_DREQ_EN) == 0 {
            return false;
        }
        let thresh = ((self.fcs & FCS_THRESH_MASK) >> FCS_THRESH_SHIFT) as usize;
        // THRESH = 0 → datasheet says "DREQ every sample"; use >= 1.
        let effective = thresh.max(1);
        self.fifo.len() >= effective
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

    /// Compose the FCS register with live LEVEL (FIFO occupancy)
    /// and EMPTY/FULL status bits.
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

    /// Produce a deterministic 12-bit sample for the given channel.
    /// Firmware needs non-zero, varying data — a modelled analog
    /// frontend is out of scope for Phase 3.
    #[inline]
    fn make_sample(&self, channel: u32) -> u16 {
        // channel occupies high 4 bits, counter low 8 — keeps adjacent
        // channels distinguishable while giving the counter room to
        // vary through the FIFO.
        let payload = ((channel & 0xF) << 8) | (self.conversion_counter & 0xFF);
        (payload & 0xFFF) as u16
    }

    /// Fire a single completed conversion — push to FIFO if enabled,
    /// update RESULT, set READY, clear START_ONCE if that was the
    /// trigger. Returns `true` if the FIFO level crossed the threshold
    /// (level IRQ edge should re-route).
    fn complete_conversion(&mut self) -> bool {
        let ch = self.ainsel();
        let sample = self.make_sample(ch);
        self.last_sample = sample;
        self.conversion_counter = self.conversion_counter.wrapping_add(1);
        self.cs |= CS_READY;

        let mut fifo_edge = false;
        if self.fcs_enabled() {
            if self.fifo.len() >= ADC_FIFO_DEPTH {
                // Overrun: drop the new sample, set OVER sticky.
                self.fcs |= FCS_OVER;
            } else {
                self.fifo.push_back(sample);
                fifo_edge = true;
            }
        }
        // START_ONCE is auto-cleared after one conversion; START_MANY
        // keeps going until firmware clears it.
        self.cs &= !CS_START_ONCE;
        self.conversion_remaining = None;
        fifo_edge
    }

    /// Raise INTR bits based on current FIFO level vs FCS.THRESH.
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

    /// Route pending interrupt lines onto the NVIC pending bitmap.
    fn route_irq(&self, irqs: &mut u32) {
        if ((self.intr | self.intf) & self.inte) != 0 {
            *irqs |= 1u32 << self.nvic_irq;
        }
    }

    /// Start a new conversion if enabled and not already running.
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
            // READY clears for the duration of the conversion.
            self.cs &= !CS_READY;
        }
    }

    /// Read a register. `FIFO` has a side-effect (pop).
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

    /// Halfword read — currently only meaningful for `FIFO` (SHIFT mode
    /// yields an 8-bit payload in the low byte; we expose the full
    /// 12-bit sample in either case).
    pub fn read16(&mut self, offset: u32) -> u16 {
        if offset == FIFO {
            self.fifo_pop_sample()
        } else {
            self.read32(offset) as u16
        }
    }

    /// Pop one FIFO entry as a u16. If FCS.SHIFT is set the sample is
    /// right-shifted to an 8-bit payload (datasheet §4.9.6 "right-
    /// justified 8-bit sample"). When the FIFO is empty we latch
    /// `FCS.UNDER` and return 0 — matches silicon, lets the corpus's
    /// drain loops terminate gracefully.
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

    /// Write a register with an APB alias (0=normal, 1=XOR, 2=BITSET,
    /// 3=BITCLR). `irqs` is ORed with `1 << ADC_IRQ_FIFO` when the
    /// write leaves FIFO level >= THRESH and INTE is set.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u32) {
        match offset {
            CS => {
                let mut stored = self.cs;
                apply_alias_rmw(&mut stored, value, alias);
                // Clear bits that firmware isn't allowed to set
                // directly (READY is driven by the peripheral; ERR is
                // a sticky derived from conversion overruns).
                self.cs = (stored & CS_WRITE_MASK) | (self.cs & (CS_READY | CS_ERR));
                self.maybe_start();
            }
            RESULT => {} // read-only
            FCS => {
                // Split FCS into the writable control slice (EN/SHIFT/
                // ERR/DREQ_EN/THRESH) and the sticky W1C bits (UNDER/
                // OVER). LEVEL/EMPTY/FULL are derived on read.
                let sticky = self.fcs & (FCS_UNDER | FCS_OVER);
                let mut new_ctrl = self.fcs & FCS_WRITE_MASK;
                apply_alias_rmw(&mut new_ctrl, value, alias);
                self.fcs = (new_ctrl & FCS_WRITE_MASK) | sticky;
                // UNDER/OVER are W1C. For normal writes and BITSET any
                // bit set in `value` clears the matching sticky (mirrors
                // TIMER_INTR). XOR/BITCLR leave sticky bits untouched.
                if alias == 0 || alias == 2 {
                    let w1c_mask = value & (FCS_UNDER | FCS_OVER);
                    self.fcs &= !w1c_mask;
                }
                // Clearing FCS_EN drains the FIFO (datasheet §4.9.5).
                if !self.fcs_enabled() {
                    self.fifo.clear();
                }
                self.refresh_intr();
                self.route_irq(irqs);
            }
            FIFO => {} // read-only
            DIV => {
                let mut stored = self.div;
                apply_alias_rmw(&mut stored, value, alias);
                // 16.8 fixed-point: integer[31:16] (RP2040 datasheet
                // encodes 20 usable bits here but we store the full
                // 32 for firmware round-trip).
                self.div = stored & 0x00FF_FFFF;
            }
            INTR => {
                // INTR is read-only on silicon (datasheet §4.9.6);
                // firmware clears the FIFO condition by draining. We
                // keep the write path a no-op for correctness.
            }
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
            INTS => {} // read-only
            _ => {}
        }
    }

    /// Advance the ADC by `sys_cycles` `clk_sys` ticks.
    ///
    /// `clk_sys` is scaled into `clk_adc` sub-ticks via the fixed-point
    /// accumulator pattern from HLD V7 §5.3. Each sub-tick decrements
    /// the running conversion counter; when it hits zero the sample is
    /// committed, INTR refreshed, and (on edge) the NVIC line routed.
    pub fn tick(&mut self, sys_cycles: u32, clock_tree: &ClockTree, irqs: &mut u32) {
        if sys_cycles == 0 {
            return;
        }
        // Pick up START_MANY retriggers that happen between ticks.
        self.maybe_start();

        if self.conversion_remaining.is_none() && (self.cs & CS_START_MANY) == 0 {
            // Idle: still route INTF-forced IRQs so disabled→enabled
            // mask transitions surface latched sources.
            self.route_irq(irqs);
            return;
        }

        let sys_hz = clock_tree.sys_clk_hz.max(1) as u64;
        self.adc_phase = self.adc_phase.saturating_add((ADC_HZ as u64) * (sys_cycles as u64));

        let mut fired = false;
        while self.adc_phase >= sys_hz {
            self.adc_phase -= sys_hz;
            // Re-arm on START_MANY after a prior conversion committed.
            self.maybe_start();
            if let Some(rem) = self.conversion_remaining.as_mut() {
                if *rem > 1 {
                    *rem -= 1;
                } else {
                    let _ = self.complete_conversion();
                    fired = true;
                    // START_MANY immediately re-arms on the next
                    // iteration via `maybe_start`.
                }
            } else if (self.cs & CS_START_MANY) == 0 {
                // Nothing else to do this quantum.
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
        Self::new(crate::irq::IRQ_ADC_IRQ_FIFO)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ADC_IRQ: u32 = 22;
    const SYS_HZ: u32 = 125_000_000;

    fn default_tree() -> ClockTree {
        ClockTree {
            sys_clk_hz: SYS_HZ,
            ref_clk_hz: 12_000_000,
            peri_clk_hz: SYS_HZ,
        }
    }

    // --- Reset / defaults -----------------------------------------------

    #[test]
    fn reset_defaults_all_zero() {
        let a = AdcRegs::new(ADC_IRQ);
        assert_eq!(a.cs, 0);
        assert_eq!(a.last_sample, 0);
        assert_eq!(a.div, 0);
        assert_eq!(a.intr, 0);
        assert_eq!(a.inte, 0);
        assert_eq!(a.intf, 0);
        assert!(a.fifo.is_empty());
        assert!(a.is_idle());
    }

    #[test]
    fn reset_restores_state_after_activity() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_EN | CS_START_MANY, 0, &mut irqs);
        a.tick(1_000, &default_tree(), &mut irqs);
        a.reset();
        assert_eq!(a.cs, 0);
        assert!(a.fifo.is_empty());
        assert!(a.is_idle());
    }

    // --- CS write sanitisation -----------------------------------------

    #[test]
    fn cs_write_cannot_set_ready_directly() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_READY, 0, &mut irqs);
        assert_eq!(a.cs & CS_READY, 0, "READY must be peripheral-driven");
    }

    #[test]
    fn cs_readback_tracks_writable_fields() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        // EN + TS_EN + AINSEL=3.
        let v = CS_EN | CS_TS_EN | (3 << CS_AINSEL_SHIFT);
        a.write32(CS, v, 0, &mut irqs);
        assert_eq!(a.cs & v, v);
    }

    // --- One-shot conversion --------------------------------------------

    #[test]
    fn start_once_requires_enable() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_START_ONCE, 0, &mut irqs);
        // EN=0 → no conversion arms.
        assert!(a.conversion_remaining.is_none());
    }

    #[test]
    fn start_once_arms_conversion() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        assert_eq!(a.conversion_remaining, Some(CONVERSION_ADC_TICKS));
        assert_eq!(a.cs & CS_READY, 0);
    }

    #[test]
    fn start_once_completes_sets_ready_and_result() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        // Select channel 3 so the deterministic pattern produces
        // non-zero bits (channel 0 + first counter=0 would yield 0).
        a.write32(CS, CS_EN | CS_START_ONCE | (3 << CS_AINSEL_SHIFT), 0, &mut irqs);
        // 96 adc ticks @ 48 MHz vs 125 MHz sys_clk. Needed sys_clk
        // ticks ≈ 96 * 125/48 = 250. Tick a safe overshoot.
        let tree = default_tree();
        a.tick(400, &tree, &mut irqs);
        assert_eq!(a.cs & CS_READY, CS_READY, "READY must be set after conversion");
        assert_eq!(a.cs & CS_START_ONCE, 0, "START_ONCE must auto-clear");
        assert_eq!(a.read32(RESULT), a.last_sample as u32);
        assert_ne!(a.read32(RESULT), 0, "deterministic sample should be non-zero");
    }

    #[test]
    fn start_many_keeps_converting() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        // Enable FIFO so completed conversions accumulate.
        a.write32(FCS, FCS_EN, 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_MANY, 0, &mut irqs);

        // Tick enough sys cycles to complete multiple conversions.
        // 96 adc_clk per conversion → ~250 sys_clk at 125 MHz / 48 MHz.
        // FIFO depth is 4 → run for 4 * 250 + margin.
        let tree = default_tree();
        a.tick(2_000, &tree, &mut irqs);
        assert!(
            a.fifo.len() >= 2,
            "multiple conversions should have latched, got {}",
            a.fifo.len()
        );
    }

    // --- Clock scaling --------------------------------------------------

    #[test]
    fn clk_adc_scaling_matches_ratio() {
        // With SYS_HZ=125e6 and ADC_HZ=48e6, a 1 sys_clk tick moves
        // adc_phase by ADC_HZ. After N ticks, adc_phase residue should
        // be N * ADC_HZ mod SYS_HZ.
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        let tree = default_tree();
        // 125 sys_clks = 48 adc_clks (exact integer ratio at these HZ).
        a.tick(125, &tree, &mut irqs);
        // After 125 sys, we expect 48 adc sub-ticks to have fired —
        // decrementing `conversion_remaining` from 96 to 48.
        assert_eq!(
            a.conversion_remaining,
            Some(CONVERSION_ADC_TICKS - 48),
            "125 sys_clk @ (125/48) = 48 adc_clk"
        );
    }

    // --- FIFO + threshold IRQ ------------------------------------------

    #[test]
    fn fifo_level_below_thresh_does_not_raise() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        // FCS.EN=1, THRESH=4 (full FIFO).
        a.write32(FCS, FCS_EN | (4 << FCS_THRESH_SHIFT), 0, &mut irqs);
        a.write32(INTE, INTR_FIFO, 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        a.tick(400, &default_tree(), &mut irqs);
        // One sample in FIFO, thresh=4 → no IRQ.
        assert_eq!(a.fifo.len(), 1);
        assert_eq!(a.intr & INTR_FIFO, 0);
        assert_eq!(irqs & (1u32 << ADC_IRQ), 0);
    }

    #[test]
    fn fifo_level_meets_thresh_raises_irq() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        // Drive FIFO full; thresh=4 triggers.
        a.write32(FCS, FCS_EN | (4 << FCS_THRESH_SHIFT), 0, &mut irqs);
        a.write32(INTE, INTR_FIFO, 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_MANY, 0, &mut irqs);
        a.tick(2_000, &default_tree(), &mut irqs);
        assert_eq!(a.fifo.len(), ADC_FIFO_DEPTH);
        assert_eq!(a.intr & INTR_FIFO, INTR_FIFO);
        assert_ne!(irqs & (1u32 << ADC_IRQ), 0, "NVIC line 22 must fire");
    }

    #[test]
    fn fifo_pop_drops_level() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(FCS, FCS_EN | (1 << FCS_THRESH_SHIFT), 0, &mut irqs);
        a.write32(INTE, INTR_FIFO, 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        a.tick(400, &default_tree(), &mut irqs);
        assert_eq!(a.fifo.len(), 1);
        assert_eq!(a.intr & INTR_FIFO, INTR_FIFO);
        // Drain via FIFO read.
        let _ = a.read32(FIFO);
        assert!(a.fifo.is_empty());
        assert_eq!(a.intr & INTR_FIFO, 0, "INTR must drop when FIFO below THRESH");
    }

    #[test]
    fn fifo_pop_empty_sets_under() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let v = a.read32(FIFO);
        assert_eq!(v, 0);
        assert_ne!(a.fcs_read() & FCS_UNDER, 0, "empty pop must latch UNDER");
    }

    #[test]
    fn fifo_fcs_read_exposes_level_empty_full() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(FCS, FCS_EN | (1 << FCS_THRESH_SHIFT), 0, &mut irqs);
        assert_ne!(a.fcs_read() & FCS_EMPTY, 0);
        a.write32(CS, CS_EN | CS_START_MANY, 0, &mut irqs);
        a.tick(2_000, &default_tree(), &mut irqs);
        let fcs = a.fcs_read();
        let level = (fcs & FCS_LEVEL_MASK) >> FCS_LEVEL_SHIFT;
        assert_eq!(level as usize, a.fifo.len());
        assert_ne!(fcs & FCS_FULL, 0);
        assert_eq!(fcs & FCS_EMPTY, 0);
    }

    // --- INTE / INTF ----------------------------------------------------

    #[test]
    fn intf_forces_ints_without_match() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(INTE, INTR_FIFO, 0, &mut irqs);
        a.write32(INTF, INTR_FIFO, 0, &mut irqs);
        assert_eq!(a.read32(INTS), INTR_FIFO);
        assert_ne!(irqs & (1u32 << ADC_IRQ), 0);
    }

    #[test]
    fn inte_gated_when_intr_latched_but_disabled() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        // Raise the condition without enabling INTE → no NVIC fire.
        a.write32(FCS, FCS_EN | (1 << FCS_THRESH_SHIFT), 0, &mut irqs);
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        a.tick(400, &default_tree(), &mut irqs);
        assert_eq!(a.intr & INTR_FIFO, INTR_FIFO);
        assert_eq!(irqs & (1u32 << ADC_IRQ), 0, "INTE=0 must not route");
    }

    // --- Alias semantics -----------------------------------------------

    #[test]
    fn cs_bitset_alias_ors_in_bits() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_EN, 0, &mut irqs);
        a.write32(CS, CS_START_ONCE, 2, &mut irqs); // BITSET
        assert!(a.cs & CS_EN != 0);
        assert!(a.is_enabled());
    }

    #[test]
    fn cs_bitclr_alias_clears_bits() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_EN | (4 << CS_AINSEL_SHIFT), 0, &mut irqs);
        a.write32(CS, CS_AINSEL_MASK, 3, &mut irqs); // BITCLR on AINSEL
        assert_eq!(a.ainsel(), 0);
        assert!(a.is_enabled(), "BITCLR on AINSEL must leave EN intact");
    }

    // --- is_idle --------------------------------------------------------

    #[test]
    fn is_idle_false_during_conversion() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        assert!(!a.is_idle());
    }

    #[test]
    fn is_idle_true_after_completion_and_drain() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        // One-shot without FIFO enable: no queued samples.
        a.write32(CS, CS_EN | CS_START_ONCE, 0, &mut irqs);
        a.tick(400, &default_tree(), &mut irqs);
        // Drop EN so no re-arm is considered.
        a.write32(CS, CS_EN, 3, &mut irqs);
        assert!(a.is_idle());
    }

    // --- DIV storage roundtrip -----------------------------------------

    #[test]
    fn div_is_round_trip_storage() {
        let mut a = AdcRegs::new(ADC_IRQ);
        let mut irqs = 0u32;
        a.write32(DIV, 0x0012_3456, 0, &mut irqs);
        assert_eq!(a.read32(DIV), 0x0012_3456);
    }
}

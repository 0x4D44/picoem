//! RP2040 PWM peripheral (datasheet §4.5).
//!
//! Phase 3 of the RP2040 peripheral coverage plan (HLD V7 §5.3 / §6).
//! Single instance at `0x4005_0000` exposing eight independent slices,
//! each with its own counter + wrap + compare registers. Reset-gated on
//! RESETS bit 14. pico-sdk's `pwm/hello_pwm` exercises `CH0_CSR`,
//! `CH0_DIV`, `CH0_CTR`, `CH0_CC`, `CH0_TOP`, the global `EN`, and the
//! `INTR`/`INTE`/`INTF`/`INTS` group on the wrap interrupt. That
//! register set is what this module models; slice-level phase-advance /
//! phase-retard pulses (`PH_ADV`/`PH_RET`) and phase-correct mode are
//! storage-only in Phase 3.
//!
//! # Counter cadence
//!
//! Per HLD V7 §5.3 Phase 3 simplification: each enabled slice increments
//! its CTR by 1 per `clk_sys` cycle, ignoring `CH_DIV`. The corpus
//! `hello_pwm` programs small TOP values so this is faithful enough for
//! the corpus to reach the wrap-interrupt handler. Fractional divider
//! support is a Phase 4 concern.
//!
//! # IRQ model
//!
//! Single NVIC line `PWM_IRQ_WRAP` (4) shared across all eight slices.
//! `INTR` is a bitmap — bit N latches when slice N wraps (CTR == TOP
//! rolls over to 0). `INTE` and `INTF` mask individual slices onto the
//! shared NVIC line. Firmware W1Cs bits via `INTR` to dismiss each
//! slice's wrap event.
//!
//! # Register map (offsets relative to `PWM_BASE`)
//!
//! | Offset | Name    | Access | Notes                                |
//! |--------|---------|--------|--------------------------------------|
//! | `0x00` | `CHn_CSR` | R/W  | Per-slice control.                   |
//! | `0x04` | `CHn_DIV` | R/W  | Per-slice divisor (storage-only).    |
//! | `0x08` | `CHn_CTR` | R/W  | Counter. Writes load the counter.    |
//! | `0x0C` | `CHn_CC`  | R/W  | Channel A (low16) + B (high16) cmp.  |
//! | `0x10` | `CHn_TOP` | R/W  | Wrap value.                          |
//! | `0xA0` | `EN`      | R/W  | **Alias** of the eight per-slice     |
//! |        |           |      | `CHn_CSR.EN` bits — same physical    |
//! |        |           |      | storage (datasheet §4.5.3.18).       |
//! | `0xA4` | `INTR`    | W1C  | Per-slice wrap latch.                |
//! | `0xA8` | `INTE`    | R/W  | Interrupt enable per slice.          |
//! | `0xAC` | `INTF`    | R/W  | Interrupt force per slice.           |
//! | `0xB0` | `INTS`    | R    | (INTR \| INTF) & INTE.               |
//!
//! # Deferred from Phase 3
//!
//! * `CH_DIV` fractional cadence (Phase 4).
//! * `PH_CORRECT` phase-correct counter mode (triangle wave).
//! * `A_INV` / `B_INV` output inversion.
//! * Output pin fan-out to GPIO (pads still driven by SIO / PIO paths).
//! * DMA DREQ generation on wrap (Phase 4).

use picoem_common::clocks::ClockTree;

use super::apply_alias_rmw;

/// Number of PWM slices on RP2040.
pub const PWM_SLICE_COUNT: usize = 8;

/// Offset stride between consecutive slice register banks.
pub const SLICE_STRIDE: u32 = 0x14;

// Per-slice register offsets (within a slice's 0x14-byte bank).
const SLICE_CSR: u32 = 0x00;
const SLICE_DIV: u32 = 0x04;
const SLICE_CTR: u32 = 0x08;
const SLICE_CC: u32 = 0x0C;
const SLICE_TOP: u32 = 0x10;

// Global register offsets.
pub const EN: u32 = 0xA0;
pub const INTR: u32 = 0xA4;
pub const INTE: u32 = 0xA8;
pub const INTF: u32 = 0xAC;
pub const INTS: u32 = 0xB0;

// --- CSR bits --------------------------------------------------------

pub const CSR_EN: u32 = 1 << 0;
pub const CSR_PH_CORRECT: u32 = 1 << 1;
pub const CSR_A_INV: u32 = 1 << 2;
pub const CSR_B_INV: u32 = 1 << 3;
const CSR_DIVMODE_SHIFT: u32 = 4;
const CSR_DIVMODE_MASK: u32 = 0x3 << CSR_DIVMODE_SHIFT;
pub const CSR_PH_RETARD: u32 = 1 << 6;
pub const CSR_PH_ADVANCE: u32 = 1 << 7;

/// Writable bits of CH_CSR.
const CSR_WRITE_MASK: u32 = CSR_EN
    | CSR_PH_CORRECT
    | CSR_A_INV
    | CSR_B_INV
    | CSR_DIVMODE_MASK
    | CSR_PH_RETARD
    | CSR_PH_ADVANCE;

/// `TOP` reset value (RP2040 datasheet §4.5.2.4): `0xFFFF` — the full
/// 16-bit counter range. A freshly-reset slice with `EN=1` would wrap
/// every 65 536 sysclks; `hello_pwm` programs a smaller TOP before
/// enabling.
pub const TOP_RESET: u32 = 0xFFFF;

/// `CH_DIV` reset value (16.4 fixed-point `1.0` = division-by-one,
/// i.e. `0x0010`).
pub const DIV_RESET: u32 = 0x0010;

/// Per-slice storage.
#[derive(Clone, Copy)]
pub struct PwmSlice {
    pub csr: u32,
    pub div: u32,
    pub ctr: u16,
    pub cc: u32,
    pub top: u16,
}

impl PwmSlice {
    pub const fn new() -> Self {
        Self {
            csr: 0,
            div: DIV_RESET,
            ctr: 0,
            cc: 0,
            top: TOP_RESET as u16,
        }
    }
}

impl Default for PwmSlice {
    fn default() -> Self {
        Self::new()
    }
}

/// PWM register storage.
pub struct PwmRegs {
    slices: [PwmSlice; PWM_SLICE_COUNT],
    intr: u8,
    inte: u8,
    intf: u8,
    nvic_irq: u32,
}

impl PwmRegs {
    /// Construct a fresh PWM at power-on defaults. `nvic_irq` is the
    /// shared NVIC line (4 for PWM_IRQ_WRAP on RP2040).
    pub fn new(nvic_irq: u32) -> Self {
        Self {
            slices: [PwmSlice::new(); PWM_SLICE_COUNT],
            intr: 0,
            inte: 0,
            intf: 0,
            nvic_irq,
        }
    }

    pub fn reset(&mut self) {
        let irq = self.nvic_irq;
        *self = Self::new(irq);
    }

    /// Synthetic view of the `PWM_EN` register at offset 0xA0. Each bit
    /// `i` is `CHi_CSR.EN` — the two registers share physical storage
    /// per datasheet §4.5.3.18. Exposed for tests and internal use.
    fn pwm_en_view(&self) -> u8 {
        let mut v: u8 = 0;
        for (i, s) in self.slices.iter().enumerate() {
            if (s.csr & CSR_EN) != 0 {
                v |= 1u8 << i;
            }
        }
        v
    }

    /// True iff no slice is enabled and no wrap-latch is pending.
    /// Phase 3 simplification per HLD V7 §5.3: "idle iff no slice has
    /// CSR.EN == 1". We additionally require INTR is clear so that a
    /// latched wrap from before a global disable still surfaces via
    /// the slow path.
    pub fn is_idle(&self) -> bool {
        self.pwm_en_view() == 0 && self.intr == 0 && (self.intf & self.inte) == 0
    }

    // --- Offset decoding -----------------------------------------------

    /// Decode a register offset into `(slice, inner_offset)` if it
    /// falls within a slice bank; otherwise `None`. The global
    /// registers (EN / INTR / INTE / INTF / INTS) start at 0xA0.
    fn decode_slice_offset(offset: u32) -> Option<(usize, u32)> {
        if offset >= (PWM_SLICE_COUNT as u32) * SLICE_STRIDE {
            return None;
        }
        let slice = (offset / SLICE_STRIDE) as usize;
        let inner = offset % SLICE_STRIDE;
        Some((slice, inner))
    }

    // --- Interrupt plumbing --------------------------------------------

    fn route_irq(&self, irqs: &mut u32) {
        if ((self.intr | self.intf) & self.inte) != 0 {
            *irqs |= 1u32 << self.nvic_irq;
        }
    }

    /// Raise `intr` bit `slice` (the wrap latch) and route the NVIC
    /// line if INTE or INTF gated it open.
    fn latch_wrap(&mut self, slice: usize) {
        self.intr |= 1u8 << slice;
    }

    // --- Register reads ------------------------------------------------

    pub fn read32(&mut self, offset: u32) -> u32 {
        if let Some((slice, inner)) = Self::decode_slice_offset(offset) {
            return match inner {
                SLICE_CSR => self.slices[slice].csr,
                SLICE_DIV => self.slices[slice].div,
                SLICE_CTR => self.slices[slice].ctr as u32,
                SLICE_CC => self.slices[slice].cc,
                SLICE_TOP => self.slices[slice].top as u32,
                _ => 0,
            };
        }
        match offset {
            EN => self.pwm_en_view() as u32,
            INTR => self.intr as u32,
            INTE => self.inte as u32,
            INTF => self.intf as u32,
            INTS => ((self.intr | self.intf) & self.inte) as u32,
            _ => 0,
        }
    }

    // --- Register writes -----------------------------------------------

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u32) {
        if let Some((slice, inner)) = Self::decode_slice_offset(offset) {
            match inner {
                SLICE_CSR => {
                    let mut stored = self.slices[slice].csr;
                    apply_alias_rmw(&mut stored, value, alias);
                    self.slices[slice].csr = stored & CSR_WRITE_MASK;
                    // Datasheet §4.5.2.3: clearing `PH_ADVANCE` /
                    // `PH_RETARD` is done by hardware after the pulse —
                    // we emulate the pulse as a 1-cycle transient.
                    self.slices[slice].csr &= !(CSR_PH_ADVANCE | CSR_PH_RETARD);
                }
                SLICE_DIV => {
                    let mut stored = self.slices[slice].div;
                    apply_alias_rmw(&mut stored, value, alias);
                    // 16.4 fixed-point: low 12 bits used by real silicon
                    // (4 fractional + 8 integer). Mask and store.
                    self.slices[slice].div = stored & 0x0FFF;
                }
                SLICE_CTR => {
                    let mut stored = self.slices[slice].ctr as u32;
                    apply_alias_rmw(&mut stored, value, alias);
                    self.slices[slice].ctr = stored as u16;
                }
                SLICE_CC => {
                    let mut stored = self.slices[slice].cc;
                    apply_alias_rmw(&mut stored, value, alias);
                    self.slices[slice].cc = stored;
                }
                SLICE_TOP => {
                    let mut stored = self.slices[slice].top as u32;
                    apply_alias_rmw(&mut stored, value, alias);
                    self.slices[slice].top = stored as u16;
                }
                _ => {}
            }
            return;
        }
        match offset {
            EN => {
                // PWM_EN is an alias of the eight CHn_CSR.EN bits
                // (datasheet §4.5.3.18) — same physical storage. Fan out
                // the RMW through the per-slice CSRs so that whichever
                // register firmware touches, the other view updates in
                // lock-step.
                let mut stored = self.pwm_en_view() as u32;
                apply_alias_rmw(&mut stored, value, alias);
                let new_en = stored as u8;
                for i in 0..PWM_SLICE_COUNT {
                    if (new_en & (1u8 << i)) != 0 {
                        self.slices[i].csr |= CSR_EN;
                    } else {
                        self.slices[i].csr &= !CSR_EN;
                    }
                }
                self.route_irq(irqs);
            }
            INTR => {
                // W1C — every bit set in the resolved value clears the
                // matching latch. Match TIMER's alias handling.
                let mut stored = self.intr as u32;
                apply_alias_rmw(&mut stored, value, alias);
                let clr = stored as u8;
                self.intr &= !clr;
                self.route_irq(irqs);
            }
            INTE => {
                let mut stored = self.inte as u32;
                let before = stored;
                apply_alias_rmw(&mut stored, value, alias);
                self.inte = stored as u8;
                tracing::debug!(
                    target: "rp2040_emu::peripherals::pwm",
                    alias,
                    "INTE write val=0x{:02x} {:#04x} -> {:#04x}",
                    value, before, stored,
                );
                self.route_irq(irqs);
            }
            INTF => {
                let mut stored = self.intf as u32;
                apply_alias_rmw(&mut stored, value, alias);
                self.intf = stored as u8;
                self.route_irq(irqs);
            }
            INTS => {} // read-only
            _ => {}
        }
    }

    /// Advance the PWM peripheral by `cycles` `clk_sys` ticks.
    ///
    /// Phase 3 simplification: CTR advances by 1 per sys_clk on every
    /// enabled slice, ignoring CH_DIV. When CTR is at TOP the next
    /// sys_clk wraps it to 0 and latches the wrap bit in INTR.
    ///
    /// The counter period is `TOP + 1` sys_clks: `0, 1, ..., TOP, 0, ...`.
    /// If `cycles` spans more than one period only a single wrap bit is
    /// latched — INTR is a bitmap firmware W1Cs, it can't count wraps.
    pub fn tick(&mut self, cycles: u32, _clock_tree: &ClockTree, irqs: &mut u32) {
        if cycles == 0 {
            self.route_irq(irqs);
            return;
        }
        for slice_idx in 0..PWM_SLICE_COUNT {
            let slice = &mut self.slices[slice_idx];
            // PWM_EN and CHn_CSR.EN share physical storage (datasheet
            // §4.5.3.18), so CSR.EN is the single gate here.
            if (slice.csr & CSR_EN) == 0 {
                continue;
            }
            let top = slice.top as u64;
            let period = top + 1;
            let old_ctr = slice.ctr as u64;
            // New counter position after `cycles` advances, modulo one
            // period. With TOP=0 the period is 1 and every cycle wraps.
            let new_ctr = (old_ctr + cycles as u64) % period;
            // The first wrap happens at offset `top + 1 - old_ctr`
            // sys_clks from now. Any `cycles >= that offset` triggers
            // at least one wrap.
            let to_first_wrap = period - old_ctr;
            let wrapped = (cycles as u64) >= to_first_wrap;
            slice.ctr = new_ctr as u16;
            if wrapped {
                self.latch_wrap(slice_idx);
            }
        }
        self.route_irq(irqs);
    }
}

impl Default for PwmRegs {
    fn default() -> Self {
        Self::new(crate::irq::IRQ_PWM_IRQ_WRAP)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PWM_IRQ: u32 = 4;
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
    fn reset_defaults_all_slices_idle() {
        let p = PwmRegs::new(PWM_IRQ);
        assert_eq!(p.pwm_en_view(), 0);
        assert_eq!(p.intr, 0);
        for s in &p.slices {
            assert_eq!(s.csr, 0);
            assert_eq!(s.ctr, 0);
            assert_eq!(s.cc, 0);
            // TOP reset is 0xFFFF per datasheet.
            assert_eq!(s.top, 0xFFFF);
            assert_eq!(s.div, DIV_RESET);
        }
        assert!(p.is_idle());
    }

    #[test]
    fn reset_clears_post_state() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        p.tick(1_000, &default_tree(), &mut irqs);
        p.reset();
        assert!(p.is_idle());
        assert_eq!(p.slices[0].ctr, 0);
    }

    // --- Register round-trips ------------------------------------------

    #[test]
    fn slice_offset_decode_spans_all_eight() {
        for slice in 0..PWM_SLICE_COUNT {
            let base = slice as u32 * SLICE_STRIDE;
            assert_eq!(
                PwmRegs::decode_slice_offset(base + SLICE_CSR),
                Some((slice, SLICE_CSR))
            );
            assert_eq!(
                PwmRegs::decode_slice_offset(base + SLICE_TOP),
                Some((slice, SLICE_TOP))
            );
        }
        // 0xA0 (EN) is outside the slice range.
        assert_eq!(PwmRegs::decode_slice_offset(EN), None);
    }

    #[test]
    fn slice_register_roundtrip() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        // Slice 2 (base 0x28).
        let base = 2 * SLICE_STRIDE;
        p.write32(base + SLICE_TOP, 0x1234, 0, &mut irqs);
        p.write32(base + SLICE_CC, 0xDEAD_BEEF, 0, &mut irqs);
        assert_eq!(p.read32(base + SLICE_TOP), 0x1234);
        assert_eq!(p.read32(base + SLICE_CC), 0xDEAD_BEEF);
    }

    #[test]
    fn en_write_roundtrip() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(EN, 0xFF, 0, &mut irqs);
        assert_eq!(p.read32(EN) & 0xFF, 0xFF);
    }

    // --- CTR advance + wrap --------------------------------------------

    #[test]
    fn enabled_slice_ctr_advances_one_per_sys_clk() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.tick(50, &default_tree(), &mut irqs);
        assert_eq!(p.slices[0].ctr, 50);
    }

    #[test]
    fn wrap_at_top_latches_intr_bit() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        // TOP=100 means wrap after 101 increments.
        p.tick(101, &default_tree(), &mut irqs);
        assert_eq!(p.intr & 0x1, 0x1, "slice 0 wrap must latch INTR bit 0");
        assert_eq!(p.slices[0].ctr, 0);
    }

    #[test]
    fn inte_gates_nvic_fire_on_wrap() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 50, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.write32(INTE, 1, 0, &mut irqs);
        p.tick(51, &default_tree(), &mut irqs);
        assert_ne!(irqs & (1u32 << PWM_IRQ), 0, "PWM_IRQ_WRAP must fire");
    }

    #[test]
    fn wrap_without_inte_does_not_raise_nvic() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 50, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.tick(51, &default_tree(), &mut irqs);
        assert_eq!(p.intr & 1, 1, "INTR still latches regardless of INTE");
        assert_eq!(irqs & (1u32 << PWM_IRQ), 0, "no NVIC fire without INTE");
    }

    #[test]
    fn disabled_slice_does_not_advance() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        // A slice whose CSR.EN is 0 must not run. PWM_EN is an alias of
        // CSR.EN, so writing 0 to PWM_EN after enabling via CSR should
        // also clear the slice.
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        p.write32(EN, 0, 0, &mut irqs); // clears CSR.EN via the alias
        assert_eq!(
            p.slices[0].csr & CSR_EN,
            0,
            "writing 0 to PWM_EN must clear CSR.EN on every slice"
        );
        p.tick(500, &default_tree(), &mut irqs);
        assert_eq!(p.slices[0].ctr, 0);
        assert_eq!(p.intr, 0);
    }

    #[test]
    fn pwm_en_clearing_a_bit_clears_matching_csr_en() {
        // The old `globally_enabled_without_csr_en_does_not_advance`
        // test predates datasheet §4.5.3.18 — it asserted that PWM_EN
        // and CSR.EN were independent gates. Under the alias model
        // they aren't: writing PWM_EN with a bit clear clears the
        // matching slice's CSR.EN. Confirm that + the slice then halts.
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        // Enable slice 0 via CSR.
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        p.tick(10, &default_tree(), &mut irqs);
        assert_eq!(p.slices[0].ctr, 10);
        // Clear slice 0 via a PWM_EN BITCLR. CSR.EN must mirror.
        p.write32(EN, 0x01, 3, &mut irqs);
        assert_eq!(p.slices[0].csr & CSR_EN, 0);
        let ctr_after_clear = p.slices[0].ctr;
        p.tick(500, &default_tree(), &mut irqs);
        assert_eq!(
            p.slices[0].ctr, ctr_after_clear,
            "slice halts once PWM_EN clears its bit"
        );
    }

    // --- INTR W1C -------------------------------------------------------

    #[test]
    fn intr_is_w1c() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 10, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.tick(11, &default_tree(), &mut irqs);
        assert_eq!(p.intr & 1, 1);
        // W1C.
        p.write32(INTR, 1, 0, &mut irqs);
        assert_eq!(p.intr & 1, 0);
    }

    #[test]
    fn intr_write_zero_does_not_clear() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.intr = 0xFF;
        p.write32(INTR, 0, 0, &mut irqs);
        assert_eq!(p.intr, 0xFF);
    }

    // --- INTS combine ---------------------------------------------------

    #[test]
    fn ints_combines_intr_intf_gated_by_inte() {
        let mut p = PwmRegs::new(PWM_IRQ);
        p.intr = 0b0000_0011; // slices 0,1 pending
        p.intf = 0b0000_1100; // slices 2,3 forced
        p.inte = 0b0000_0101; // enable 0 + 2
        let v = p.read32(INTS);
        assert_eq!(v, 0b0000_0101);
    }

    // --- Multi-slice concurrent wrap ----------------------------------

    #[test]
    fn multiple_slices_wrap_independently() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        // Slice 0 top=10, slice 1 top=20, slice 2 disabled.
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 10, 0, &mut irqs);
        p.write32(SLICE_STRIDE + SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_STRIDE + SLICE_TOP, 20, 0, &mut irqs);
        p.write32(EN, 0b0000_0011, 0, &mut irqs);
        // After 21 ticks both slices have wrapped at least once.
        p.tick(21, &default_tree(), &mut irqs);
        assert_eq!(
            p.intr & 0b11,
            0b11,
            "both slices 0 and 1 must latch wrap bits"
        );
        assert_eq!(p.intr & 0b100, 0, "slice 2 must not latch — not enabled");
    }

    // --- is_idle --------------------------------------------------------

    #[test]
    fn is_idle_false_when_any_slice_enabled() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(EN, 0x01, 0, &mut irqs);
        assert!(!p.is_idle());
    }

    #[test]
    fn is_idle_true_with_no_enable_no_pending() {
        let p = PwmRegs::new(PWM_IRQ);
        assert!(p.is_idle());
    }

    // --- Alias semantics -----------------------------------------------

    #[test]
    fn en_bitset_alias() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(EN, 0x03, 0, &mut irqs);
        p.write32(EN, 0x0C, 2, &mut irqs); // BITSET
        assert_eq!(p.pwm_en_view(), 0x0F);
    }

    #[test]
    fn en_bitclr_alias() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.write32(EN, 0xFF, 0, &mut irqs);
        p.write32(EN, 0xF0, 3, &mut irqs); // BITCLR
        assert_eq!(p.pwm_en_view(), 0x0F);
    }

    #[test]
    fn csr_bitset_alias_on_slice_4() {
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        let base = 4 * SLICE_STRIDE;
        p.write32(base + SLICE_CSR, CSR_A_INV, 0, &mut irqs);
        p.write32(base + SLICE_CSR, CSR_B_INV, 2, &mut irqs); // BITSET
        assert_eq!(
            p.slices[4].csr & (CSR_A_INV | CSR_B_INV),
            CSR_A_INV | CSR_B_INV
        );
    }

    // --- PWM_EN is an alias of the eight CSR.EN bits (datasheet §4.5.3.18) ---

    #[test]
    fn csr_en_alone_advances_slice_and_mirrors_into_pwm_en() {
        // Pico-SDK's `pwm_set_enabled()` writes only CSR.EN — not PWM_EN at
        // 0xA0. Per RP2040 §4.5.3.18 PWM_EN is a *view* of the eight per-
        // slice CSR.EN bits; writing CSR.EN[4] must therefore be observable
        // as bit 4 of PWM_EN, and slice 4 must run without any write to
        // 0xA0.
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        let base = 4 * SLICE_STRIDE;
        // Program slice 4 TOP then enable via CSR only.
        p.write32(base + SLICE_TOP, 100, 0, &mut irqs);
        p.write32(base + SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.tick(50, &default_tree(), &mut irqs);
        assert_eq!(
            p.slices[4].ctr, 50,
            "slice 4 CTR must advance with only CSR.EN set"
        );
        assert_eq!(
            p.read32(EN) & 0xFF,
            1u32 << 4,
            "PWM_EN must read back as the OR of the eight CSR.EN bits"
        );
    }

    #[test]
    fn intr_selective_w1c_clears_only_matching_bits() {
        // Plain write with a mask only clears bits set in `value`;
        // other latched bits survive. Used by firmware that dismisses
        // one slice's IRQ while keeping another's pending.
        let mut p = PwmRegs::new(PWM_IRQ);
        let mut irqs = 0u32;
        p.intr = 0b1111_0000;
        p.write32(INTR, 0b0101_0000, 0, &mut irqs);
        assert_eq!(p.intr, 0b1010_0000, "only bits set in value get W1C'd");
    }
}

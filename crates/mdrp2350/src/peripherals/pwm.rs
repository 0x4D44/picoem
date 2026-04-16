//! RP2350 PWM peripheral (datasheet §12.5).
//!
//! Phase 2 of the RP2350 peripheral coverage plan (HLD V5 §6 row 2).
//! Single instance at `0x400A_8000`. Reset-gated on RESETS bit 16.
//!
//! **Address note**: V5 §6 Phase 2 table quotes `0x4005_0000` (the
//! RP2040 PWM base). That address is in fact RP2350 PLL_SYS per
//! datasheet §2.3 Table 6 and the pico-sdk-pico2 `addressmap.h`. The
//! correct RP2350 PWM base is `0x400A_8000`; we use that here to avoid
//! aliasing with PLL_SYS. Documented in the Phase 2 implementation
//! journal for Arthur to reconcile with the HLD.
//!
//! RP2350 has **12 slices** (RP2040 had 8 — the extra 4 support the
//! additional GPIOs). Slices 0..=7 route their wrap IRQ to
//! [`crate::irq::IRQ_PWM_IRQ_WRAP_0`] (8); slices 8..=11 route to
//! [`crate::irq::IRQ_PWM_IRQ_WRAP_1`] (9). The corpus `hello_pwm` uses
//! slice 0 via `PWM_IRQ_WRAP_0` — slice-8+ IRQs exist for completeness.
//!
//! Register map grows linearly: 12 slices × 5 registers = 60 words
//! (0x00..0xEC), then the global `EN/INTR/INTE/INTF/INTS` block starts
//! at `0xF0` (matching pico-sdk `hardware/regs/pwm.h` for RP2350).
//!
//! # Register map (offsets relative to `PWM_BASE`)
//!
//! | Offset | Name       | Access | Notes                              |
//! |--------|------------|--------|------------------------------------|
//! | `0x00+` | `CHn_CSR..TOP` | R/W | Per-slice 5-word bank (stride 0x14) |
//! | `0xF0` | `EN`       | R/W    | 12-bit mask; 1 = slice N runs       |
//! | `0xF4` | `INTR`     | W1C    | Per-slice wrap latch (12 bits)      |
//! | `0xF8` | `INTE0`    | R/W    | Interrupt enable (slices 0..7)      |
//! | `0xFC` | `INTF0`    | R/W    | Interrupt force (slices 0..7)       |
//! | `0x100`| `INTS0`    | R      | `(INTR | INTF) & INTE` for 0..7     |
//! | `0x104`| `INTE1`    | R/W    | Interrupt enable (slices 8..11)     |
//! | `0x108`| `INTF1`    | R/W    | Interrupt force (slices 8..11)      |
//! | `0x10C`| `INTS1`    | R      | `(INTR | INTF) & INTE` for 8..11    |
//!
//! # Counter cadence
//!
//! Each enabled slice uses its `CH_DIV` 8.4 fixed-point divider
//! (INT[11:4]:FRAC[3:0]) to convert `clk_sys` ticks into counter
//! advances. The O(1) closed-form fractional accumulator avoids
//! per-cycle iteration. At the reset value DIV=0x0010 (1.0), advance
//! equals cycles — identical to the Phase 2 simplification.

use mdpicoem_common::clocks::ClockTree;

use crate::irq::{IRQ_PWM_IRQ_WRAP_0, IRQ_PWM_IRQ_WRAP_1};

use super::apply_alias_rmw;

/// PWM base (RP2350 datasheet §12.5 / §2.3 Table 6 / pico-sdk-pico2
/// `addressmap.h`). Note that this is **not** `0x4005_0000` (that is
/// RP2350 PLL_SYS); the HLD V5 §6 Phase 2 row's "0x4005_0000" appears
/// to be copied from the RP2040 map and is corrected here.
pub const PWM_BASE: u32 = 0x400A_8000;

/// Number of PWM slices on RP2350.
pub const PWM_SLICE_COUNT: usize = 12;

/// Offset stride between consecutive slice register banks.
pub const SLICE_STRIDE: u32 = 0x14;

/// Boundary of the slice-bank range. Offsets >= this hit the global
/// register block (EN/INTR/INTE0/..).
const SLICE_BLOCK_END: u32 = (PWM_SLICE_COUNT as u32) * SLICE_STRIDE;

// Per-slice register offsets (within a slice's 0x14-byte bank).
const SLICE_CSR: u32 = 0x00;
const SLICE_DIV: u32 = 0x04;
const SLICE_CTR: u32 = 0x08;
const SLICE_CC: u32 = 0x0C;
const SLICE_TOP: u32 = 0x10;

// Global register offsets (post-slice-bank).
pub const EN: u32 = 0xF0;
pub const INTR: u32 = 0xF4;
pub const INTE0: u32 = 0xF8;
pub const INTF0: u32 = 0xFC;
pub const INTS0: u32 = 0x100;
pub const INTE1: u32 = 0x104;
pub const INTF1: u32 = 0x108;
pub const INTS1: u32 = 0x10C;

// --- CSR bits --------------------------------------------------------

pub const CSR_EN: u32 = 1 << 0;
pub const CSR_PH_CORRECT: u32 = 1 << 1;
pub const CSR_A_INV: u32 = 1 << 2;
pub const CSR_B_INV: u32 = 1 << 3;
const CSR_DIVMODE_SHIFT: u32 = 4;
const CSR_DIVMODE_MASK: u32 = 0x3 << CSR_DIVMODE_SHIFT;
pub const CSR_PH_RETARD: u32 = 1 << 6;
pub const CSR_PH_ADVANCE: u32 = 1 << 7;

const CSR_WRITE_MASK: u32 = CSR_EN
    | CSR_PH_CORRECT
    | CSR_A_INV
    | CSR_B_INV
    | CSR_DIVMODE_MASK
    | CSR_PH_RETARD
    | CSR_PH_ADVANCE;

/// `TOP` reset value (datasheet §12.5.2.4): full 16-bit counter range.
pub const TOP_RESET: u32 = 0xFFFF;

/// `CH_DIV` reset value (8.4 fixed-point (INT[11:4]:FRAC[3:0]) 1.0).
pub const DIV_RESET: u32 = 0x0010;

/// Mask covering all 12 slice bits.
const SLICE_MASK: u16 = 0x0FFF;
/// Mask for slices 0..=7 (PWM_IRQ_WRAP_0 group).
const WRAP0_MASK: u16 = 0x00FF;
/// Mask for slices 8..=11 (PWM_IRQ_WRAP_1 group).
const WRAP1_MASK: u16 = 0x0F00;

#[derive(Clone, Copy)]
pub struct PwmSlice {
    pub csr: u32,
    pub div: u32,
    pub ctr: u16,
    pub cc: u32,
    pub top: u16,
    /// Fractional accumulator for the 8.4 fixed-point divider.
    pub frac_accum: u16,
}

impl PwmSlice {
    pub const fn new() -> Self {
        Self {
            csr: 0,
            div: DIV_RESET,
            ctr: 0,
            cc: 0,
            top: TOP_RESET as u16,
            frac_accum: 0,
        }
    }
}

impl Default for PwmSlice {
    fn default() -> Self {
        Self::new()
    }
}

pub struct PwmRegs {
    slices: [PwmSlice; PWM_SLICE_COUNT],
    en: u16,
    intr: u16,
    inte: u16,
    intf: u16,
    /// NVIC IRQ for slices 0..=7 (`IRQ_PWM_IRQ_WRAP_0`).
    nvic_irq_wrap0: u32,
    /// NVIC IRQ for slices 8..=11 (`IRQ_PWM_IRQ_WRAP_1`).
    nvic_irq_wrap1: u32,
}

impl PwmRegs {
    pub fn new(nvic_irq_wrap0: u32, nvic_irq_wrap1: u32) -> Self {
        Self {
            slices: [PwmSlice::new(); PWM_SLICE_COUNT],
            en: 0,
            intr: 0,
            inte: 0,
            intf: 0,
            nvic_irq_wrap0,
            nvic_irq_wrap1,
        }
    }

    pub fn reset(&mut self) {
        let (i0, i1) = (self.nvic_irq_wrap0, self.nvic_irq_wrap1);
        *self = Self::new(i0, i1);
    }

    pub fn is_idle(&self) -> bool {
        self.en == 0 && self.intr == 0 && (self.intf & self.inte) == 0
    }

    fn decode_slice_offset(offset: u32) -> Option<(usize, u32)> {
        if offset >= SLICE_BLOCK_END {
            return None;
        }
        let slice = (offset / SLICE_STRIDE) as usize;
        let inner = offset % SLICE_STRIDE;
        Some((slice, inner))
    }

    fn route_irq(&self, irqs: &mut u64) {
        let pending = (self.intr | self.intf) & self.inte;
        if (pending & WRAP0_MASK) != 0 {
            *irqs |= 1u64 << self.nvic_irq_wrap0;
        }
        if (pending & WRAP1_MASK) != 0 {
            *irqs |= 1u64 << self.nvic_irq_wrap1;
        }
    }

    fn latch_wrap(&mut self, slice: usize) {
        self.intr |= 1u16 << slice;
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
            EN => self.en as u32,
            INTR => self.intr as u32,
            INTE0 => (self.inte & WRAP0_MASK) as u32,
            INTF0 => (self.intf & WRAP0_MASK) as u32,
            INTS0 => ((self.intr | self.intf) & self.inte & WRAP0_MASK) as u32,
            INTE1 => ((self.inte & WRAP1_MASK) >> 8) as u32,
            INTF1 => ((self.intf & WRAP1_MASK) >> 8) as u32,
            INTS1 => (((self.intr | self.intf) & self.inte & WRAP1_MASK) >> 8) as u32,
            _ => 0,
        }
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, irqs: &mut u64) {
        if let Some((slice, inner)) = Self::decode_slice_offset(offset) {
            match inner {
                SLICE_CSR => {
                    let mut stored = self.slices[slice].csr;
                    apply_alias_rmw(&mut stored, value, alias);
                    self.slices[slice].csr = stored & CSR_WRITE_MASK;
                    // PH_ADVANCE / PH_RETARD auto-clear after the pulse.
                    self.slices[slice].csr &= !(CSR_PH_ADVANCE | CSR_PH_RETARD);
                }
                SLICE_DIV => {
                    let mut stored = self.slices[slice].div;
                    apply_alias_rmw(&mut stored, value, alias);
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
                let mut stored = self.en as u32;
                apply_alias_rmw(&mut stored, value, alias);
                self.en = (stored as u16) & SLICE_MASK;
                self.route_irq(irqs);
            }
            INTR => {
                let mut stored = self.intr as u32;
                apply_alias_rmw(&mut stored, value, alias);
                let clr = stored as u16 & SLICE_MASK;
                self.intr &= !clr;
                self.route_irq(irqs);
            }
            INTE0 => {
                let mut stored = (self.inte & WRAP0_MASK) as u32;
                apply_alias_rmw(&mut stored, value, alias);
                self.inte = (self.inte & !WRAP0_MASK) | ((stored as u16) & WRAP0_MASK);
                self.route_irq(irqs);
            }
            INTF0 => {
                let mut stored = (self.intf & WRAP0_MASK) as u32;
                apply_alias_rmw(&mut stored, value, alias);
                self.intf = (self.intf & !WRAP0_MASK) | ((stored as u16) & WRAP0_MASK);
                self.route_irq(irqs);
            }
            INTE1 => {
                let hi_current = ((self.inte & WRAP1_MASK) >> 8) as u32;
                let mut stored = hi_current;
                apply_alias_rmw(&mut stored, value, alias);
                let new_hi = ((stored as u16) << 8) & WRAP1_MASK;
                self.inte = (self.inte & !WRAP1_MASK) | new_hi;
                self.route_irq(irqs);
            }
            INTF1 => {
                let hi_current = ((self.intf & WRAP1_MASK) >> 8) as u32;
                let mut stored = hi_current;
                apply_alias_rmw(&mut stored, value, alias);
                let new_hi = ((stored as u16) << 8) & WRAP1_MASK;
                self.intf = (self.intf & !WRAP1_MASK) | new_hi;
                self.route_irq(irqs);
            }
            INTS0 | INTS1 => {} // read-only
            _ => {}
        }
    }

    /// Advance the PWM peripheral by `cycles` `clk_sys` ticks.
    ///
    /// Each enabled slice uses its `CH_DIV` 8.4 fixed-point divider to
    /// convert sys_clk ticks into counter advances via an O(1)
    /// closed-form fractional accumulator.
    pub fn tick(&mut self, cycles: u32, _clock_tree: &ClockTree, irqs: &mut u64) {
        if cycles == 0 {
            self.route_irq(irqs);
            return;
        }
        for slice_idx in 0..PWM_SLICE_COUNT {
            let globally_enabled = (self.en & (1u16 << slice_idx)) != 0;
            let slice = &mut self.slices[slice_idx];
            let locally_enabled = (slice.csr & CSR_EN) != 0;
            if !globally_enabled || !locally_enabled {
                continue;
            }

            // O(1) fractional accumulator: DIV is 8.4 fixed-point
            // (INT[11:4]:FRAC[3:0]).
            let int_part = (slice.div >> 4) as u32;   // INT[11:4]
            let frac_part = (slice.div & 0xF) as u32; // FRAC[3:0]
            let total_frac = slice.frac_accum as u32 + frac_part * cycles;
            let carry = total_frac / 16;
            slice.frac_accum = (total_frac % 16) as u16;
            let advance = int_part * cycles + carry;

            if advance == 0 {
                continue;
            }

            let top = slice.top as u64;
            let period = top + 1;
            let old_ctr = slice.ctr as u64;
            let new_ctr = (old_ctr + advance as u64) % period;
            let to_first_wrap = period - old_ctr;
            let wrapped = (advance as u64) >= to_first_wrap;
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
        Self::new(IRQ_PWM_IRQ_WRAP_0, IRQ_PWM_IRQ_WRAP_1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_tree() -> ClockTree {
        ClockTree {
            sys_clk_hz: 150_000_000,
            ref_clk_hz: 12_000_000,
            peri_clk_hz: 150_000_000,
        }
    }

    fn new_pwm() -> PwmRegs {
        PwmRegs::new(IRQ_PWM_IRQ_WRAP_0, IRQ_PWM_IRQ_WRAP_1)
    }

    #[test]
    fn twelve_slices_at_reset() {
        let p = new_pwm();
        assert_eq!(p.slices.len(), 12);
        for s in &p.slices {
            assert_eq!(s.top, 0xFFFF);
            assert_eq!(s.div, DIV_RESET);
        }
    }

    #[test]
    fn is_idle_at_reset() {
        let p = new_pwm();
        assert!(p.is_idle());
    }

    #[test]
    fn slice_bank_decodes_across_twelve_slices() {
        for slice in 0..PWM_SLICE_COUNT {
            let base = slice as u32 * SLICE_STRIDE;
            assert_eq!(
                PwmRegs::decode_slice_offset(base + SLICE_TOP),
                Some((slice, SLICE_TOP))
            );
        }
        // 0xF0 is the EN register — above the slice block.
        assert_eq!(PwmRegs::decode_slice_offset(EN), None);
        assert_eq!(PwmRegs::decode_slice_offset(INTR), None);
    }

    #[test]
    fn en_roundtrip_masks_12_bits() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(EN, 0xFFFF, 0, &mut irqs);
        // Only 12 slice bits — high nibble of the 16-bit mask must clear.
        assert_eq!(p.read32(EN) & 0xFFFF, 0x0FFF);
    }

    #[test]
    fn enabled_slice_ctr_advances_one_per_sys_clk() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.tick(50, &default_tree(), &mut irqs);
        assert_eq!(p.slices[0].ctr, 50);
    }

    #[test]
    fn wrap_at_top_latches_intr_bit() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.tick(101, &default_tree(), &mut irqs);
        assert_eq!(p.intr & 0x1, 0x1);
        assert_eq!(p.slices[0].ctr, 0);
    }

    #[test]
    fn inte0_gates_wrap0_nvic_fire() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 50, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.write32(INTE0, 1, 0, &mut irqs);
        p.tick(51, &default_tree(), &mut irqs);
        assert_ne!(irqs & (1u64 << IRQ_PWM_IRQ_WRAP_0), 0);
        assert_eq!(irqs & (1u64 << IRQ_PWM_IRQ_WRAP_1), 0);
    }

    #[test]
    fn inte1_gates_wrap1_nvic_fire() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        // Slice 8 — upper-group.
        let base = 8 * SLICE_STRIDE;
        p.write32(base + SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(base + SLICE_TOP, 50, 0, &mut irqs);
        p.write32(EN, 1u32 << 8, 0, &mut irqs);
        p.write32(INTE1, 1, 0, &mut irqs); // bit 0 of INTE1 = slice 8
        p.tick(51, &default_tree(), &mut irqs);
        // Slice 8 wraps, WRAP_1 fires, WRAP_0 stays clear.
        assert_ne!(p.intr & (1u16 << 8), 0);
        assert_ne!(irqs & (1u64 << IRQ_PWM_IRQ_WRAP_1), 0);
        assert_eq!(irqs & (1u64 << IRQ_PWM_IRQ_WRAP_0), 0);
    }

    #[test]
    fn slice_enable_mask_controls_advance() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 100, 0, &mut irqs);
        // Global mask clear → counter frozen.
        p.write32(EN, 0, 0, &mut irqs);
        p.tick(500, &default_tree(), &mut irqs);
        assert_eq!(p.slices[0].ctr, 0);
    }

    #[test]
    fn intr_is_w1c() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 10, 0, &mut irqs);
        p.write32(EN, 1, 0, &mut irqs);
        p.tick(11, &default_tree(), &mut irqs);
        assert_eq!(p.intr & 1, 1);
        p.write32(INTR, 1, 0, &mut irqs);
        assert_eq!(p.intr & 1, 0);
    }

    #[test]
    fn multiple_slices_wrap_independently() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_TOP, 10, 0, &mut irqs);
        p.write32(SLICE_STRIDE + SLICE_CSR, CSR_EN, 0, &mut irqs);
        p.write32(SLICE_STRIDE + SLICE_TOP, 20, 0, &mut irqs);
        p.write32(EN, 0b0000_0011, 0, &mut irqs);
        p.tick(21, &default_tree(), &mut irqs);
        assert_eq!(p.intr & 0b11, 0b11);
    }

    #[test]
    fn en_bitset_alias() {
        let mut p = new_pwm();
        let mut irqs = 0u64;
        p.write32(EN, 0x03, 0, &mut irqs);
        p.write32(EN, 0x0C, 2, &mut irqs);
        assert_eq!(p.en, 0x0F);
    }
}

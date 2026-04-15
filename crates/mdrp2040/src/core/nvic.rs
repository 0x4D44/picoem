// Phase 1 Wave 2 TODO: expose ISER/ICER/ICPR/IPR via PPB; add enabled mask;
// have CortexM0Plus::step poll pending && enabled to deliver external IRQ
// exceptions. silicon_isr_diff_rp2040::isr_m0_timer_cold cannot pass without
// this.

//! Minimal NVIC pending-latch for Cortex-M0+.
//!
//! Phase 1 Wave 1 of the RP2040 peripheral coverage plan (HLD V7 §5.2)
//! needs a target for peripheral-asserted external interrupts. ARMv6-M
//! has a 32-line NVIC with per-line enable / pending / active
//! registers; RP2040 routes 26 lines into it (see
//! [`crate::irq`](crate::irq)). Phase 1 only needs the pending-latch so
//! [`Emulator::drain_pending_irqs_to_cores`] has somewhere to stash
//! peripheral-asserted interrupts until the exception dispatcher (tech_
//! debt: `CortexM0Plus::step` not polling external IRQs yet — HLD V7
//! Phase 1 landing also gates on `silicon_isr_diff_rp2040` which will
//! surface the issue) picks them up.
//!
//! Why a separate struct rather than a bare `u32` on `CortexM0Plus`:
//! later waves add NVIC_ISER / NVIC_ICER / NVIC_IPR register decode,
//! and a struct keeps that growth local. Today it's just `pending: u32`.
//!
//! [`Emulator::drain_pending_irqs_to_cores`]: crate::Emulator::drain_pending_irqs_to_cores

/// Cortex-M0+ NVIC — pending latch only (Phase 1 scope).
///
/// One bit per external IRQ line (bit N = IRQ #N pending). RP2040 uses
/// lines 0..=25; bits 26..=31 are unused and never asserted.
#[derive(Default, Clone, Copy)]
pub struct Nvic {
    /// Pending external interrupts — bit N set iff line N is pending.
    pub pending: u32,
}

impl Nvic {
    /// Construct an NVIC with no interrupts pending.
    pub fn new() -> Self {
        Self { pending: 0 }
    }

    /// Reset to power-on defaults.
    pub fn reset(&mut self) {
        self.pending = 0;
    }

    /// Mark IRQ line `irq` as pending. No-op if `irq >= 32` (the RP2040
    /// datasheet pins the NVIC at 32 lines).
    ///
    /// This is a set operation, not a toggle — level peripherals
    /// re-assert every cycle the condition holds, so repeated calls
    /// with the same line are idempotent.
    #[inline]
    pub fn set_pending(&mut self, irq: u8) {
        if irq < 32 {
            self.pending |= 1u32 << irq;
        }
    }

    /// Clear the pending bit for IRQ line `irq`. No-op if `irq >= 32`.
    #[inline]
    pub fn clear_pending(&mut self, irq: u8) {
        if irq < 32 {
            self.pending &= !(1u32 << irq);
        }
    }

    /// True iff IRQ line `irq` is currently pending. Always `false`
    /// when `irq >= 32` (the NVIC is 32 lines wide on ARMv6-M).
    #[inline]
    pub fn is_pending(&self, irq: u8) -> bool {
        irq < 32 && (self.pending & (1u32 << irq)) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_nvic_has_nothing_pending() {
        let n = Nvic::new();
        assert_eq!(n.pending, 0);
    }

    #[test]
    fn set_pending_latches_bit() {
        let mut n = Nvic::new();
        n.set_pending(7);
        assert!(n.is_pending(7));
        assert_eq!(n.pending, 1u32 << 7);
    }

    #[test]
    fn set_pending_is_idempotent() {
        let mut n = Nvic::new();
        n.set_pending(5);
        n.set_pending(5);
        n.set_pending(5);
        assert_eq!(n.pending, 1u32 << 5);
    }

    #[test]
    fn set_pending_oob_is_noop() {
        let mut n = Nvic::new();
        n.set_pending(32);
        n.set_pending(255);
        assert_eq!(n.pending, 0);
    }

    #[test]
    fn clear_pending_drops_bit() {
        let mut n = Nvic::new();
        n.set_pending(3);
        n.set_pending(9);
        n.clear_pending(3);
        assert!(!n.is_pending(3));
        assert!(n.is_pending(9));
    }

    #[test]
    fn reset_drops_all_pending() {
        let mut n = Nvic::new();
        n.set_pending(0);
        n.set_pending(15);
        n.reset();
        assert_eq!(n.pending, 0);
    }
}

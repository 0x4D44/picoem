//! Minimal NVIC for Cortex-M0+.
//!
//! Phase 1 Wave 2 of the RP2040 peripheral coverage plan (HLD V7 §5.2)
//! needs enough NVIC register surface for pico-sdk's `irq_set_enabled`
//! / `irq_set_priority` to land on something live, and for the CPU step
//! path to actually deliver external IRQs. ARMv6-M has a 32-line NVIC
//! with per-line enable / pending / priority; RP2040 routes 26 of those
//! lines (see [`crate::irq`]). This struct tracks:
//!
//! * `pending` — bit N set iff line N is pending.
//! * `enabled` — bit N set iff line N is unmasked at the NVIC.
//! * `priority[0..32]` — one byte per IRQ. Only bits [7:6] are
//!   implemented on M0+ → 4 priority levels (`0x00`, `0x40`, `0x80`,
//!   `0xC0`). Lower numeric value = higher architectural priority.
//!
//! The PPB layer (`crates/mdrp2040/src/bus/ppb.rs`) maps the five
//! NVIC-register aliases onto these fields:
//!
//! | Register      | Address         | Shape |
//! |---------------|-----------------|-------|
//! | `NVIC_ISER0`  | `0xE000_E100`   | read `enabled`, W1S                    |
//! | `NVIC_ICER0`  | `0xE000_E180`   | read `enabled`, W1C                    |
//! | `NVIC_ISPR0`  | `0xE000_E200`   | read `pending`, W1S                    |
//! | `NVIC_ICPR0`  | `0xE000_E280`   | read `pending`, W1C                    |
//! | `NVIC_IPR0..7`| `0xE000_E400 + 4N` | 4×u8 priority bytes (mask `0xC0`)   |
//!
//! No IPSR / execution-priority logic here — that's on the CPU.
//!
//! # Pending-bit semantics (level vs edge)
//!
//! We clear the NVIC pending bit on exception dispatch. On real M0+
//! silicon, this matches pulse-IRQ behaviour. For level-triggered
//! sources (a peripheral with INTR latched AND inte/intf set), the
//! source itself is expected to re-raise via `poll_alarms` /
//! `tick_peripherals` on subsequent cycles. Peripherals MUST implement
//! `poll_alarms` such that a still-raised condition
//! (`intr & (inte | intf)`) re-asserts into `bus.irq_pending` on every
//! poll, not only on fresh match edges. Otherwise the emulator will
//! drop the level re-assert and silently diverge from silicon.
//!
//! The Phase 1 Wave 2 TIMER (`peripherals::timer::TimerRegs::poll_alarms`)
//! satisfies this contract: after an alarm fires and auto-disarms, the
//! INTR bit stays latched, and each subsequent poll re-ORs
//! `(intr & inte)` into the returned NVIC bitmap until the ISR W1Cs
//! INTR. See the `poll_alarms_re_asserts_latched_level_until_w1c` test.

/// Priority byte mask — only bits [7:6] are implemented on M0+.
/// Firmware reads of `NVIC_IPRn` observe stored bytes masked by this
/// constant; writes are pre-masked before storage. Four distinct
/// architectural priority levels: `0x00` (highest), `0x40`, `0x80`,
/// `0xC0` (lowest).
pub const PRIORITY_MASK: u8 = 0xC0;

/// Cortex-M0+ NVIC.
///
/// One bit per external IRQ line. RP2040 uses lines 0..=25; bits
/// 26..=31 are unused and never asserted. Each field is word-wide so
/// register-level read/write is a direct memory-to-MMIO mirror.
#[derive(Clone, Copy)]
pub struct Nvic {
    /// Pending external interrupts — bit N set iff line N is pending.
    pub pending: u32,
    /// Enabled external interrupts — bit N set iff line N is unmasked.
    pub enabled: u32,
    /// Per-line priority bytes. Lower numeric value = higher priority.
    pub priority: [u8; 32],
}

impl Default for Nvic {
    fn default() -> Self {
        Self::new()
    }
}

impl Nvic {
    /// Construct an NVIC with no interrupts pending, everything masked,
    /// all priorities at `0x00` (highest configurable level).
    pub fn new() -> Self {
        Self {
            pending: 0,
            enabled: 0,
            priority: [0; 32],
        }
    }

    /// Reset to power-on defaults.
    pub fn reset(&mut self) {
        self.pending = 0;
        self.enabled = 0;
        self.priority = [0; 32];
    }

    // --- Pending ----------------------------------------------------------

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

    // --- Enable mask ------------------------------------------------------

    /// Unmask IRQ line `irq`. No-op if `irq >= 32`.
    #[inline]
    pub fn set_enabled(&mut self, irq: u8) {
        if irq < 32 {
            self.enabled |= 1u32 << irq;
        }
    }

    /// Mask IRQ line `irq` (clear the enable bit). No-op if `irq >= 32`.
    #[inline]
    pub fn clear_enabled(&mut self, irq: u8) {
        if irq < 32 {
            self.enabled &= !(1u32 << irq);
        }
    }

    /// True iff IRQ line `irq` is currently unmasked.
    #[inline]
    pub fn is_enabled(&self, irq: u8) -> bool {
        irq < 32 && (self.enabled & (1u32 << irq)) != 0
    }

    // --- Priority ---------------------------------------------------------

    /// Assign a priority byte to IRQ line `irq`. Input is pre-masked to
    /// [`PRIORITY_MASK`] so only the implemented bits land in storage.
    /// No-op if `irq >= 32`.
    #[inline]
    pub fn set_priority(&mut self, irq: u8, prio: u8) {
        if irq < 32 {
            self.priority[irq as usize] = prio & PRIORITY_MASK;
        }
    }

    /// Read the priority byte for IRQ line `irq`. The stored byte is
    /// pre-masked, so this always returns a value in
    /// `{0x00, 0x40, 0x80, 0xC0}`. Returns 0 if `irq >= 32`.
    #[inline]
    pub fn get_priority(&self, irq: u8) -> u8 {
        if irq < 32 {
            self.priority[irq as usize]
        } else {
            0
        }
    }

    // --- Dispatch helper --------------------------------------------------

    /// Bitmap of IRQs that are both pending AND enabled. The CPU step
    /// path checks this before instruction fetch; a non-zero result
    /// means at least one IRQ is eligible for dispatch (subject to
    /// priority + PRIMASK + handler-mode gating).
    #[inline]
    pub fn pending_and_enabled(&self) -> u32 {
        self.pending & self.enabled
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Reset / defaults ------------------------------------------------

    #[test]
    fn new_nvic_has_nothing_pending() {
        let n = Nvic::new();
        assert_eq!(n.pending, 0);
        assert_eq!(n.enabled, 0);
        assert!(n.priority.iter().all(|&p| p == 0));
    }

    #[test]
    fn reset_drops_all_state() {
        let mut n = Nvic::new();
        n.set_pending(0);
        n.set_enabled(15);
        n.set_priority(7, 0xC0);
        n.reset();
        assert_eq!(n.pending, 0);
        assert_eq!(n.enabled, 0);
        assert_eq!(n.priority[7], 0);
    }

    // --- Pending ---------------------------------------------------------

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

    // --- Enable ----------------------------------------------------------

    #[test]
    fn set_enabled_latches_bit() {
        let mut n = Nvic::new();
        n.set_enabled(20);
        assert!(n.is_enabled(20));
        assert_eq!(n.enabled, 1u32 << 20);
    }

    #[test]
    fn clear_enabled_drops_bit() {
        let mut n = Nvic::new();
        n.set_enabled(0);
        n.set_enabled(25);
        n.clear_enabled(0);
        assert!(!n.is_enabled(0));
        assert!(n.is_enabled(25));
    }

    #[test]
    fn enable_oob_is_noop() {
        let mut n = Nvic::new();
        n.set_enabled(32);
        n.set_enabled(200);
        assert_eq!(n.enabled, 0);
    }

    // --- Priority --------------------------------------------------------

    #[test]
    fn set_priority_masks_to_top_two_bits() {
        let mut n = Nvic::new();
        // 0x3F is in the implemented bits' complement — must be dropped.
        n.set_priority(10, 0x3F);
        assert_eq!(n.get_priority(10), 0x00);
        // 0x7F has bit [6] set — survives masking as 0x40.
        n.set_priority(10, 0x7F);
        assert_eq!(n.get_priority(10), 0x40);
        // 0xC0 is a fully-implemented value — round-trips.
        n.set_priority(10, 0xC0);
        assert_eq!(n.get_priority(10), 0xC0);
        // 0xFF masked becomes 0xC0.
        n.set_priority(10, 0xFF);
        assert_eq!(n.get_priority(10), 0xC0);
    }

    #[test]
    fn set_priority_oob_is_noop() {
        let mut n = Nvic::new();
        n.set_priority(32, 0xC0);
        n.set_priority(99, 0x80);
        assert!(n.priority.iter().all(|&p| p == 0));
    }

    #[test]
    fn get_priority_oob_returns_zero() {
        let n = Nvic::new();
        assert_eq!(n.get_priority(32), 0);
        assert_eq!(n.get_priority(255), 0);
    }

    // --- pending_and_enabled --------------------------------------------

    #[test]
    fn pending_and_enabled_is_bitwise_and() {
        let mut n = Nvic::new();
        n.set_pending(0);
        n.set_pending(3);
        n.set_pending(7);
        n.set_enabled(3);
        n.set_enabled(7);
        n.set_enabled(10);
        // Intersection: only 3 and 7 are both pending and enabled.
        assert_eq!(n.pending_and_enabled(), (1u32 << 3) | (1u32 << 7));
    }

    #[test]
    fn pending_without_enable_is_masked() {
        let mut n = Nvic::new();
        n.set_pending(12);
        assert_eq!(n.pending_and_enabled(), 0);
    }

    #[test]
    fn enable_without_pending_is_masked() {
        let mut n = Nvic::new();
        n.set_enabled(8);
        assert_eq!(n.pending_and_enabled(), 0);
    }
}

//! RESETS peripheral (base 0x4000_C000).
//!
//! RP2040 datasheet §2.14. Simple three-register block:
//!
//! * `RESET` (0x00) — W1S / W1C / XOR via APB alias. Bits set = peripheral
//!   held in reset. Default 0x01FF_FFFF (all 25 peripherals in reset at
//!   boot).
//! * `WDSEL` (0x04) — watchdog reset select (stubbed storage).
//! * `RESET_DONE` (0x08) — bitmask of peripherals out of reset and ready.
//!   Reads as `!reset_state` (instant completion — nothing is gated on a
//!   simulated reset cycle count).
//!
//! Phase 5.A stores only what firmware reads/writes. Full per-peripheral
//! behaviour is out of scope; routing the register faithfully is enough
//! for firmware to believe its reset sequence worked.

/// Number of valid bits in the RESET register (25 peripherals on RP2040).
pub(crate) const RESET_MASK: u32 = 0x01FF_FFFF;

/// RESETS register storage.
pub struct Resets {
    /// Per-peripheral reset assertion bits. Bit N = 1 means peripheral N
    /// is held in reset. Default `RESET_MASK` — everything in reset.
    pub state: u32,
    /// Watchdog reset select — storage-only.
    pub wdsel: u32,
}

impl Resets {
    pub fn new() -> Self {
        Self {
            state: RESET_MASK,
            wdsel: 0,
        }
    }

    /// Reset to power-on defaults. Called from `Emulator::reset()`.
    pub fn reset(&mut self) {
        self.state = RESET_MASK;
        self.wdsel = 0;
    }

    /// True iff the peripheral at bit `bit` is currently held in
    /// reset. Bit numbering matches RP2040 datasheet §2.14 Table 26
    /// (TIMER = bit 21, WATCHDOG = bit 24, etc.).
    ///
    /// The Bus-level dispatch in
    /// [`super::peripheral_dispatch::is_held_in_reset`] calls this
    /// before routing MMIO to the peripheral — reset-gated access
    /// returns 0 / no-ops without ever reaching the peripheral module.
    ///
    /// Out-of-range `bit` values (≥25, the field width on RP2040)
    /// return `false` — the RESETS register doesn't carry state for
    /// them, so treating them as "not held" lets dispatch fall
    /// through to the non-reset-gated path. Today this affects no
    /// peripheral we route; a future `bit = 26` caller would be a
    /// programming error the unit tests catch at the dispatch table.
    #[inline]
    pub fn is_held(&self, bit: u8) -> bool {
        if bit >= 25 {
            return false;
        }
        (self.state & (1u32 << bit)) != 0
    }

    /// Read a RESETS register by offset.
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.state,
            0x04 => self.wdsel,
            0x08 => !self.state & RESET_MASK, // RESET_DONE = 1 for ready
            _ => 0,
        }
    }

    /// Write a RESETS register with an alias-aware update.
    ///
    /// `alias` follows the APB convention: 0=normal, 1=XOR, 2=SET, 3=CLR.
    /// Only the 25 peripheral bits are writable; bits [31:25] are masked
    /// off so `RESET_DONE` never reports ghost peripherals.
    pub fn write32(&mut self, offset: u32, val: u32, alias: u32) {
        let apply = |cur: u32, v: u32| match alias {
            0 => v,
            1 => cur ^ v,
            2 => cur | v,
            3 => cur & !v,
            _ => v,
        };
        match offset {
            0x00 => self.state = apply(self.state, val) & RESET_MASK,
            0x04 => self.wdsel = apply(self.wdsel, val) & RESET_MASK,
            _ => {} // RESET_DONE is read-only
        }
    }
}

impl Default for Resets {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_defaults_all_in_reset() {
        let r = Resets::new();
        assert_eq!(r.state, RESET_MASK);
        assert_eq!(r.read32(0x00), RESET_MASK);
        assert_eq!(r.read32(0x08), 0);
    }

    #[test]
    fn clr_alias_deasserts_reset() {
        let mut r = Resets::new();
        // CLR alias releases peripheral 0 from reset.
        r.write32(0x00, 0x1, 3);
        assert_eq!(r.state & 0x1, 0);
        assert_eq!(r.read32(0x08) & 0x1, 0x1); // RESET_DONE[0] set
    }

    #[test]
    fn set_alias_asserts_reset() {
        let mut r = Resets::new();
        r.state = 0;
        r.write32(0x00, 0x4, 2);
        assert_eq!(r.state, 0x4);
    }

    #[test]
    fn xor_alias_toggles() {
        let mut r = Resets::new();
        r.state = 0xAAAA;
        r.write32(0x00, 0xFFFF, 1);
        assert_eq!(r.state, 0x5555);
    }

    #[test]
    fn normal_write_replaces_preserving_mask() {
        let mut r = Resets::new();
        r.write32(0x00, 0xFFFF_FFFF, 0);
        // Upper bits above 25 should be masked off.
        assert_eq!(r.state, RESET_MASK);
    }

    #[test]
    fn reset_done_is_read_only() {
        let mut r = Resets::new();
        let before = r.state;
        r.write32(0x08, 0x1234, 0);
        assert_eq!(r.state, before);
    }

    #[test]
    fn is_held_reflects_reset_state_bit() {
        let mut r = Resets::new();
        // Default: every peripheral in reset → every valid bit is held.
        assert!(r.is_held(0));
        assert!(r.is_held(21)); // TIMER
        assert!(r.is_held(24)); // WATCHDOG
        // Release TIMER (bit 21) via CLR alias.
        r.write32(0x00, 1 << 21, 3);
        assert!(!r.is_held(21));
        // WATCHDOG still held.
        assert!(r.is_held(24));
    }

    #[test]
    fn is_held_out_of_range_returns_false() {
        // RP2040 has 25 peripherals; bits ≥ 25 are not modelled.
        let r = Resets::new();
        assert!(!r.is_held(25));
        assert!(!r.is_held(31));
        assert!(!r.is_held(200));
    }
}

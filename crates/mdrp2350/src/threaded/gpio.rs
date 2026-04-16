//! Atomic GPIO state for cross-thread visibility.
//!
//! RP2354 has 48 GPIO pins across two banks:
//! - Bank 0: pins 0..31
//! - Bank 1: pins 32..47
//!
//! All operations use `Relaxed` ordering — GPIO pins are observed
//! asynchronously by the outside world, so no acquire/release
//! fencing is needed.

use std::sync::atomic::{AtomicU32, Ordering::Relaxed};

/// Thread-safe GPIO output and output-enable state.
///
/// Two 32-bit banks cover the full 48-pin space.  Bank 1 bits 16..31
/// are unused on current silicon but allocated per the HLD.
pub struct AtomicGpio {
    out: [AtomicU32; 2],
    oe: [AtomicU32; 2],
}

impl AtomicGpio {
    pub fn new() -> Self {
        Self {
            out: [AtomicU32::new(0), AtomicU32::new(0)],
            oe: [AtomicU32::new(0), AtomicU32::new(0)],
        }
    }

    // ---- OUT (output value) ------------------------------------------------

    /// Read the full 32-bit OUT register for `bank` (0 or 1).
    #[inline]
    pub fn read_out(&self, bank: usize) -> u32 {
        debug_assert!(bank < 2);
        self.out[bank].load(Relaxed)
    }

    /// Overwrite the full 32-bit OUT register for `bank`.
    #[inline]
    pub fn write_out(&self, bank: usize, val: u32) {
        debug_assert!(bank < 2);
        self.out[bank].store(val, Relaxed);
    }

    /// SET: `out[bank] |= mask`.
    #[inline]
    pub fn set_out(&self, bank: usize, mask: u32) {
        debug_assert!(bank < 2);
        self.out[bank].fetch_or(mask, Relaxed);
    }

    /// CLR: `out[bank] &= !mask`.
    #[inline]
    pub fn clear_out(&self, bank: usize, mask: u32) {
        debug_assert!(bank < 2);
        self.out[bank].fetch_and(!mask, Relaxed);
    }

    /// XOR: `out[bank] ^= mask`.
    #[inline]
    pub fn xor_out(&self, bank: usize, mask: u32) {
        debug_assert!(bank < 2);
        self.out[bank].fetch_xor(mask, Relaxed);
    }

    // ---- OE (output enable) ------------------------------------------------

    /// Read the full 32-bit OE register for `bank` (0 or 1).
    #[inline]
    pub fn read_oe(&self, bank: usize) -> u32 {
        debug_assert!(bank < 2);
        self.oe[bank].load(Relaxed)
    }

    /// Overwrite the full 32-bit OE register for `bank`.
    #[inline]
    pub fn write_oe(&self, bank: usize, val: u32) {
        debug_assert!(bank < 2);
        self.oe[bank].store(val, Relaxed);
    }

    /// SET: `oe[bank] |= mask`.
    #[inline]
    pub fn set_oe(&self, bank: usize, mask: u32) {
        debug_assert!(bank < 2);
        self.oe[bank].fetch_or(mask, Relaxed);
    }

    /// CLR: `oe[bank] &= !mask`.
    #[inline]
    pub fn clear_oe(&self, bank: usize, mask: u32) {
        debug_assert!(bank < 2);
        self.oe[bank].fetch_and(!mask, Relaxed);
    }

    /// XOR: `oe[bank] ^= mask`.
    #[inline]
    pub fn xor_oe(&self, bank: usize, mask: u32) {
        debug_assert!(bank < 2);
        self.oe[bank].fetch_xor(mask, Relaxed);
    }

    // ---- Pin-level helpers -------------------------------------------------

    /// Read the output level of a single pin (0..47).
    #[inline]
    pub fn read_pin(&self, pin: u32) -> bool {
        let bank = (pin / 32) as usize;
        let bit = pin % 32;
        self.out[bank].load(Relaxed) & (1 << bit) != 0
    }
}

impl Default for AtomicGpio {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_write_roundtrip() {
        let gpio = AtomicGpio::new();

        // Bank 0 starts at zero.
        assert_eq!(gpio.read_out(0), 0);
        assert_eq!(gpio.read_oe(0), 0);

        // Write a pattern and read it back.
        gpio.write_out(0, 0xDEAD_BEEF);
        assert_eq!(gpio.read_out(0), 0xDEAD_BEEF);

        gpio.write_oe(0, 0x0000_FFFF);
        assert_eq!(gpio.read_oe(0), 0x0000_FFFF);

        // Bank 1 is independent.
        assert_eq!(gpio.read_out(1), 0);
        gpio.write_out(1, 0xCAFE);
        assert_eq!(gpio.read_out(1), 0xCAFE);
        assert_eq!(gpio.read_out(0), 0xDEAD_BEEF); // bank 0 unchanged
    }

    #[test]
    fn set_clear_xor() {
        let gpio = AtomicGpio::new();

        // SET: turn on bits 0 and 4.
        gpio.set_out(0, 0b1_0001);
        assert_eq!(gpio.read_out(0), 0b1_0001);

        // SET again: idempotent for already-set bits, additive for new.
        gpio.set_out(0, 0b1_0010);
        assert_eq!(gpio.read_out(0), 0b1_0011);

        // CLR: clear bit 0, leave bit 1 and 4 alone.
        gpio.clear_out(0, 0b0_0001);
        assert_eq!(gpio.read_out(0), 0b1_0010);

        // XOR: toggle bit 4 off, toggle bit 0 on.
        gpio.xor_out(0, 0b1_0001);
        assert_eq!(gpio.read_out(0), 0b0_0011);

        // OE follows the same pattern.
        gpio.set_oe(0, 0xFF);
        gpio.clear_oe(0, 0x0F);
        assert_eq!(gpio.read_oe(0), 0xF0);
        gpio.xor_oe(0, 0xA0);
        assert_eq!(gpio.read_oe(0), 0x50);
    }

    #[test]
    fn read_pin() {
        let gpio = AtomicGpio::new();

        // Pin 0 is low.
        assert!(!gpio.read_pin(0));

        // Set pin 25 (LED on many Pico boards).
        gpio.set_out(0, 1 << 25);
        assert!(gpio.read_pin(25));
        assert!(!gpio.read_pin(24));
        assert!(!gpio.read_pin(26));

        // Cross-bank: pin 33 lives in bank 1, bit 1.
        assert!(!gpio.read_pin(33));
        gpio.set_out(1, 1 << 1);
        assert!(gpio.read_pin(33));

        // Bank 0 is unaffected.
        assert_eq!(gpio.read_out(0), 1 << 25);
    }
}

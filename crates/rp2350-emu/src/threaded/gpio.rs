//! Atomic GPIO state for cross-thread visibility.
//!
//! RP2354 has 48 GPIO pins across two banks:
//! - Bank 0: pins 0..31
//! - Bank 1: pins 32..47
//!
//! Most operations use `Relaxed` ordering — GPIO pins are observed
//! asynchronously by the outside world, so no acquire/release
//! fencing is needed. Packed external stimulus and PIO pad snapshots
//! carry multi-field invariants across threads, so those paths use
//! `Acquire`/`Release` to preserve atomicity.

use std::sync::atomic::{
    AtomicU32, AtomicU64,
    Ordering::{Acquire, Relaxed, Release},
};

struct AtomicPadSnapshot {
    seq: AtomicU32,
    out: AtomicU64,
    oe: AtomicU64,
}

impl AtomicPadSnapshot {
    fn new() -> Self {
        Self {
            seq: AtomicU32::new(0),
            out: AtomicU64::new(0),
            oe: AtomicU64::new(0),
        }
    }

    #[inline]
    fn store(&self, out_lo: u32, oe_lo: u32, out_hi: u32, oe_hi: u32) {
        let seq = self.seq.load(Relaxed).wrapping_add(1) | 1;
        self.seq.store(seq, Release);
        self.out.store(pack_banks(out_lo, out_hi), Relaxed);
        self.oe.store(pack_banks(oe_lo, oe_hi), Relaxed);
        self.seq.store(seq.wrapping_add(1), Release);
    }

    #[inline]
    fn load(&self) -> ((u32, u32), (u32, u32)) {
        loop {
            let before = self.seq.load(Acquire);
            if before & 1 != 0 {
                std::hint::spin_loop();
                continue;
            }
            let out = self.out.load(Relaxed);
            let oe = self.oe.load(Relaxed);
            let after = self.seq.load(Acquire);
            if before == after {
                let (out_lo, out_hi) = unpack_banks(out);
                let (oe_lo, oe_hi) = unpack_banks(oe);
                return ((out_lo, oe_lo), (out_hi, oe_hi));
            }
            std::hint::spin_loop();
        }
    }
}

#[inline]
fn pack_banks(lo: u32, hi: u32) -> u64 {
    (lo as u64) | ((hi as u64) << 32)
}

#[inline]
fn unpack_banks(packed: u64) -> (u32, u32) {
    (packed as u32, (packed >> 32) as u32)
}

/// Thread-safe GPIO output and output-enable state.
///
/// Two 32-bit banks cover the full 48-pin space.  Bank 1 bits 16..31
/// are unused on current silicon but allocated per the HLD.
///
/// `in_pins` and `external` / `external_hi` carry the serial
/// `Bus` GPIO input and harness-stimulus fields into the threaded
/// runtime. PIO pad snapshots are published after projection into
/// physical low/high banks so `GPIOBASE=16` never gets treated as a
/// low-bank-only local mask by the coordinator.
///
/// `external` and `external_hi` each pack `(val, mask)` as
/// `(high32 = val, low32 = mask)` so the harness writer and coord's
/// `update_gpio` reader see an atomic pair — two independent
/// `AtomicU32`s would allow a torn read where `val` and `mask` come
/// from different writes.
pub struct AtomicGpio {
    out: [AtomicU32; 2],
    oe: [AtomicU32; 2],
    /// Packed merged input state: low32 = GPIO0..31, high32 = GPIO32..47.
    in_pins: AtomicU64,
    pio_pads: [AtomicPadSnapshot; 3],
    /// Packed external stimulus for GPIOs 0..31: `(val << 32) | mask`.
    external: AtomicU64,
    /// Packed external stimulus for GPIOs 32..47 (low 16 bits used):
    /// `(val << 32) | mask`. Companion to `external` — bit `i` of the
    /// `val`/`mask` halves corresponds to GPIO `32 + i`. Layered on top
    /// of the legacy QSPI-noise model in `Bus::read_gpio_hi_in` /
    /// `WorkerBus::sio_read32(0x008)` so the firmware sees harness-
    /// driven bits and noise on everything else.
    external_hi: AtomicU64,
}

impl AtomicGpio {
    pub fn new() -> Self {
        Self {
            out: [AtomicU32::new(0), AtomicU32::new(0)],
            oe: [AtomicU32::new(0), AtomicU32::new(0)],
            in_pins: AtomicU64::new(0),
            pio_pads: std::array::from_fn(|_| AtomicPadSnapshot::new()),
            external: AtomicU64::new(0),
            external_hi: AtomicU64::new(0),
        }
    }

    /// Seed a freshly-allocated `AtomicGpio` with values lifted from an
    /// existing single-threaded `Bus`.
    ///
    /// Phase 3 Stage 6b (LLD V7 §6/§8): `ThreadedEmulator::from_emulator`
    /// calls this to carry the live GPIO OUT/OE/IN + harness-controlled
    /// external-stimulus state (both halves) into the threaded runtime.
    /// Consumes the plain `u32` snapshots — callers take them out of the
    /// Bus destructure.
    ///
    /// SIO OUT/OE seed only bank 0. The merged input seed carries both
    /// `gpio_in` and `gpio_in_hi`; PIO physical pad snapshots are seeded
    /// separately by `ThreadedEmulator::from_emulator` after each
    /// `PioBlock` maps its local pads through its own `GPIOBASE`.
    pub fn seed(
        gpio_out: u32,
        gpio_oe: u32,
        gpio_in: u32,
        gpio_in_hi: u32,
        gpio_external_in: u32,
        gpio_external_mask: u32,
        gpio_external_in_hi: u32,
        gpio_external_mask_hi: u32,
    ) -> Self {
        let external = ((gpio_external_in as u64) << 32) | (gpio_external_mask as u64);
        let external_hi = ((gpio_external_in_hi as u64) << 32) | (gpio_external_mask_hi as u64);
        Self {
            out: [AtomicU32::new(gpio_out), AtomicU32::new(0)],
            oe: [AtomicU32::new(gpio_oe), AtomicU32::new(0)],
            in_pins: AtomicU64::new(pack_banks(gpio_in, gpio_in_hi)),
            pio_pads: std::array::from_fn(|_| AtomicPadSnapshot::new()),
            external: AtomicU64::new(external),
            external_hi: AtomicU64::new(external_hi),
        }
    }

    // ---- GPIO_IN (merged pin state) ----------------------------------------

    /// Read the merged GPIO_IN value (low 32 pins).
    #[inline]
    pub fn read_in(&self) -> u32 {
        self.in_pins.load(Relaxed) as u32
    }

    /// Overwrite GPIO_IN. Stage 7 wires this to the GPIO-merge step
    /// that runs at the quantum boundary (coordinator-owned).
    #[inline]
    pub fn write_in(&self, val: u32) {
        let mut current = self.in_pins.load(Relaxed);
        loop {
            let new = (current & 0xFFFF_FFFF_0000_0000) | val as u64;
            match self
                .in_pins
                .compare_exchange_weak(current, new, Release, Relaxed)
            {
                Ok(_) => return,
                Err(next) => current = next,
            }
        }
    }

    /// Read the merged GPIO_IN value for physical GPIOs 32..47.
    #[inline]
    pub fn read_in_hi(&self) -> u32 {
        (self.in_pins.load(Relaxed) >> 32) as u32
    }

    /// Overwrite GPIO_IN for physical GPIOs 32..47.
    #[inline]
    pub fn write_in_hi(&self, val: u32) {
        let mut current = self.in_pins.load(Relaxed);
        loop {
            let new = (current & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32);
            match self
                .in_pins
                .compare_exchange_weak(current, new, Release, Relaxed)
            {
                Ok(_) => return,
                Err(next) => current = next,
            }
        }
    }

    /// Publish a coherent physical GPIO input snapshot.
    #[inline]
    pub fn write_in64(&self, lo: u32, hi: u32) {
        self.in_pins.store(pack_banks(lo, hi), Release);
    }

    /// Read the merged physical GPIO sample used by PIO blocks.
    #[inline]
    pub fn read_in64(&self) -> u64 {
        let base = self.in_pins.load(Acquire);
        let (base_lo, base_hi) = unpack_banks(base);
        let (ext_val, ext_mask) = self.read_external();
        let lo = (base_lo & !ext_mask) | (ext_val & ext_mask);
        let (ext_val_hi, ext_mask_hi) = self.read_external_hi();
        let hi = (base_hi & !ext_mask_hi) | (ext_val_hi & ext_mask_hi);
        pack_banks(lo, hi)
    }

    /// Publish one PIO block's already-physical pad snapshot.
    #[inline]
    pub fn write_pio_pads(&self, block: usize, out_lo: u32, oe_lo: u32, out_hi: u32, oe_hi: u32) {
        debug_assert!(block < 3);
        self.pio_pads[block].store(out_lo, oe_lo, out_hi, oe_hi);
    }

    /// Read one PIO block's already-physical pad snapshot.
    #[inline]
    pub fn read_pio_pads(&self, block: usize) -> ((u32, u32), (u32, u32)) {
        debug_assert!(block < 3);
        self.pio_pads[block].load()
    }

    // ---- External-input stimulus (harness pin forcing) ---------------------

    /// Read the external-input stimulus `(val, mask)` atomically.
    ///
    /// Uses `Acquire` ordering — this carries a multi-field invariant
    /// across threads (harness writer → coord's `update_gpio` reader),
    /// unlike the Relaxed accessors on `out`/`oe`.
    #[inline]
    pub fn read_external(&self) -> (u32, u32) {
        let packed = self.external.load(Acquire);
        let val = (packed >> 32) as u32;
        let mask = packed as u32;
        (val, mask)
    }

    /// Overwrite the external-input stimulus `(val, mask)` atomically.
    ///
    /// Uses `Release` ordering — pairs with `read_external`'s
    /// `Acquire` load so the coordinator sees a coherent pair.
    #[inline]
    pub fn write_external(&self, val: u32, mask: u32) {
        let packed = ((val as u64) << 32) | (mask as u64);
        self.external.store(packed, Release);
    }

    /// Read the high-half external-input stimulus `(val, mask)`
    /// atomically. Bit `i` of either half corresponds to GPIO `32 + i`.
    ///
    /// Uses `Acquire` ordering for the same reason as
    /// [`Self::read_external`] — the harness writer and the coord's
    /// `GPIO_HI_IN` read path observe a coherent pair.
    #[inline]
    pub fn read_external_hi(&self) -> (u32, u32) {
        let packed = self.external_hi.load(Acquire);
        let val = (packed >> 32) as u32;
        let mask = packed as u32;
        (val, mask)
    }

    /// Overwrite the high-half external-input stimulus `(val, mask)`
    /// atomically. Bit `i` of either half corresponds to GPIO `32 + i`.
    ///
    /// Uses `Release` ordering — pairs with `read_external_hi`'s
    /// `Acquire` load so the coordinator sees a coherent pair.
    #[inline]
    pub fn write_external_hi(&self, val: u32, mask: u32) {
        let packed = ((val as u64) << 32) | (mask as u64);
        self.external_hi.store(packed, Release);
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

    /// External-stim accessors round-trip independently for the low and
    /// high halves. Stage 3A wide-GPIO bus support — see HLD §A.
    #[test]
    fn external_low_and_high_independent() {
        let gpio = AtomicGpio::new();

        // Low half default: zero.
        assert_eq!(gpio.read_external(), (0, 0));
        assert_eq!(gpio.read_external_hi(), (0, 0));

        // Drive the low half: bit 5 stim-high, mask covers bits 0..7.
        gpio.write_external(1 << 5, 0xFF);
        assert_eq!(gpio.read_external(), (1 << 5, 0xFF));
        // High half is untouched.
        assert_eq!(gpio.read_external_hi(), (0, 0));

        // Drive the high half: GPIO 33 (= bit 1) stim-high, mask covers
        // GPIOs 32..47 (low 16 bits of the high-half word).
        gpio.write_external_hi(1 << 1, 0xFFFF);
        assert_eq!(gpio.read_external_hi(), (1 << 1, 0xFFFF));
        // Low half is preserved.
        assert_eq!(gpio.read_external(), (1 << 5, 0xFF));
    }

    #[test]
    fn in_hi_and_in64_roundtrip() {
        let gpio = AtomicGpio::seed(0, 0, 0xAAAA_5555, 0x0000_00F0, 0, 0, 0, 0);

        assert_eq!(gpio.read_in(), 0xAAAA_5555);
        assert_eq!(gpio.read_in_hi(), 0x0000_00F0);
        assert_eq!(gpio.read_in64(), 0x0000_00F0_AAAA_5555);

        gpio.write_in_hi(0x0000_1234);
        assert_eq!(gpio.read_in64(), 0x0000_1234_AAAA_5555);

        gpio.write_in64(0xCAFE_BABE, 0x0000_5678);
        assert_eq!(gpio.read_in(), 0xCAFE_BABE);
        assert_eq!(gpio.read_in_hi(), 0x0000_5678);
        assert_eq!(gpio.read_in64(), 0x0000_5678_CAFE_BABE);

        gpio.write_external(0x0000_0001, 0x0000_0001);
        gpio.write_external_hi(0x0000_0002, 0x0000_0002);
        assert_eq!(
            gpio.read_in64() & 0x0000_0002_0000_0001,
            0x0000_0002_0000_0001
        );
    }

    #[test]
    fn pio_physical_pads_roundtrip_per_block() {
        let gpio = AtomicGpio::new();

        gpio.write_pio_pads(1, 0x0001_0000, 0x0003_0000, 0x0000_0004, 0x0000_000C);

        assert_eq!(gpio.read_pio_pads(0), ((0, 0), (0, 0)));
        assert_eq!(
            gpio.read_pio_pads(1),
            ((0x0001_0000, 0x0003_0000), (0x0000_0004, 0x0000_000C))
        );
        assert_eq!(gpio.read_pio_pads(2), ((0, 0), (0, 0)));
    }
}

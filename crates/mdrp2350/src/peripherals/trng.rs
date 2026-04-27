//! RP2350 TRNG — true-random-number generator model, HLD V5 §7.D.5.
//!
//! Models the Synopsys TRNG block as a **32-bit counter**. Each read
//! of the `RNG_OUT` register returns the current counter value, then
//! increments (wrapping). `STATUS` always reports "ready". All other
//! registers round-trip plain storage.
//!
//! # Intentional simplifications
//!
//! - **Counter, not entropy.** A linear counter passes the trivial
//!   "consecutive reads differ" check but fails any statistical test
//!   of randomness. No current pico-sdk path checks randomness
//!   properties; accepted risk (HLD V5 §7.D.5).
//! - **Ready-always.** `STATUS.EHR_VALID` (and any analogue) read as
//!   1 so firmware poll loops complete immediately.
//! - **No RNG_CTRL interpretation.** RNG_CTRL / RNG_ISR / IMR etc.
//!   are plain storage.
//! - **Counter preserved across [`Emulator::reset`](crate::Emulator::reset).**
//!   Rationale: simplicity, and a post-reset firmware that reads a
//!   different value than pre-reset still sees monotonic behaviour.
//!
//! # Base address
//!
//! `TRNG_BASE = 0x400F_0000` — RP2350 datasheet §6 (ARM TrustZone
//! TRNG). The aperture fits in a single 4 KB window. Datasheet
//! warn-once catches mismatch.

/// TRNG base. Datasheet §6. See module doc.
pub const TRNG_BASE: u32 = 0x400F_0000;

/// Synopsys DesignWare TRNG register offsets (datasheet §6 Table).
/// Only the ones with modelled behaviour are named; the rest fall into
/// the plain-storage `regs` HashMap.
const RNG_IMR_OFFSET: u32 = 0x100;
const TRNG_ISR_OFFSET: u32 = 0x104;
const TRNG_ICR_OFFSET: u32 = 0x108;
const TRNG_CONFIG_OFFSET: u32 = 0x10C;
const TRNG_VALID_OFFSET: u32 = 0x110;
const EHR_DATA0_OFFSET: u32 = 0x114;
const RND_SOURCE_ENABLE_OFFSET: u32 = 0x12C;

/// Bit indicating a 192-bit random value is ready to read
/// (datasheet §6 — `TRNG_VALID.EHR_VALID`). We report always-ready.
const EHR_VALID_BIT: u32 = 1 << 0;

/// TRNG register block.
pub struct Trng {
    /// Counter source — advances on every `EHR_DATA0` read.
    counter: u32,
    /// Plain-storage fallback for unmodelled registers (IMR, ISR,
    /// CONFIG, RND_SOURCE_ENABLE, …).
    regs: std::collections::HashMap<u32, u32>,
}

impl Trng {
    pub fn new() -> Self {
        Self {
            counter: 0,
            regs: std::collections::HashMap::new(),
        }
    }

    /// Read a word. `EHR_DATA0` returns and advances the counter;
    /// `TRNG_VALID` reports ready. Other offsets return stored value
    /// (or 0 if unwritten).
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            EHR_DATA0_OFFSET => {
                let v = self.counter;
                self.counter = self.counter.wrapping_add(1);
                v
            }
            TRNG_VALID_OFFSET => EHR_VALID_BIT,
            _ => *self.regs.get(&offset).unwrap_or(&0),
        }
    }

    /// Write a word. All registers except `EHR_DATA0` / `TRNG_VALID`
    /// round-trip through the HashMap under canonical alias semantics.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        match offset {
            EHR_DATA0_OFFSET | TRNG_VALID_OFFSET => {
                // RO — ignore.
            }
            RNG_IMR_OFFSET
            | TRNG_ISR_OFFSET
            | TRNG_ICR_OFFSET
            | TRNG_CONFIG_OFFSET
            | RND_SOURCE_ENABLE_OFFSET => {
                let stored = self.regs.entry(offset).or_insert(0);
                super::apply_alias_rmw(stored, value, alias);
            }
            _ => {
                let stored = self.regs.entry(offset).or_insert(0);
                super::apply_alias_rmw(stored, value, alias);
            }
        }
    }
}

impl Default for Trng {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_out_reads_are_monotonically_increasing() {
        let mut t = Trng::new();
        let a = t.read32(EHR_DATA0_OFFSET);
        let b = t.read32(EHR_DATA0_OFFSET);
        let c = t.read32(EHR_DATA0_OFFSET);
        assert_eq!(a, 0);
        assert_eq!(b, 1);
        assert_eq!(c, 2);
        // Continues across many reads.
        for expected in 3..20u32 {
            assert_eq!(t.read32(EHR_DATA0_OFFSET), expected);
        }
    }

    #[test]
    fn status_is_always_ready() {
        let mut t = Trng::new();
        // TRNG_VALID.EHR_VALID == 1 immediately.
        assert_eq!(t.read32(TRNG_VALID_OFFSET) & EHR_VALID_BIT, EHR_VALID_BIT);
        // After draining several words, still ready.
        for _ in 0..8 {
            let _ = t.read32(EHR_DATA0_OFFSET);
        }
        assert_eq!(t.read32(TRNG_VALID_OFFSET) & EHR_VALID_BIT, EHR_VALID_BIT);
        // Write to TRNG_VALID is ignored (RO).
        t.write32(TRNG_VALID_OFFSET, 0, 0);
        assert_eq!(t.read32(TRNG_VALID_OFFSET) & EHR_VALID_BIT, EHR_VALID_BIT);
    }
}

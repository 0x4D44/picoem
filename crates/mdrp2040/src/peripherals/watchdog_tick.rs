//! WATCHDOG_TICK register model — minimal Phase 1 scope.
//!
//! RP2040 datasheet §4.7. The watchdog block sits at base
//! `0x4005_8000`; this file models only the `TICK` register at offset
//! `0x2C`, which is the sole register TIMER reads to derive its 1 µs
//! cadence. The rest of the watchdog (timeout counter, reset select,
//! scratch registers, feed / pause state) is out of scope for Phase 1
//! — out-of-range offsets read as 0 and write as no-op.
//!
//! # `TICK` register layout (offset `0x2C`)
//!
//! | Bits   | Field   | Access | Reset | Meaning                                         |
//! |--------|---------|--------|-------|-------------------------------------------------|
//! | `8:0`  | CYCLES  | R/W    | 12    | cycles of `clk_ref` per 1 µs tick               |
//! | `9`    | ENABLE  | R/W    | 0     | tick generator enable                           |
//! | `10`   | RUNNING | RO     | 0     | mirrors `ENABLE` one cycle later (tick running) |
//! | `19:11`| COUNT   | RO     | 0     | current countdown (reset-value counters)        |
//!
//! `CYCLES` defaults to `12` because pico-sdk programs `clk_ref` to
//! 12 MHz before releasing the TIMER from reset, giving the required 1
//! MHz / 1 µs TIMER cadence. Firmware frequently reads the register
//! back after writing (see `hardware_ticks_set_cycles`); the backing
//! store round-trips every writable bit. COUNT is a compact count-down
//! field that we do not advance — Phase 1 stops short of a cycle-
//! accurate tick generator because nothing in the corpus distinguishes
//! RUNNING / COUNT behaviour from "value stored on last write".
//!
//! `ENABLE` and `RUNNING` collapse on read (RUNNING echoes ENABLE with
//! no per-cycle delay). This matches silicon behaviour closely enough
//! for the `hello_timer` corpus check that both bits appear set shortly
//! after enable. A full cycle-accurate transition lives in `tech_debt`
//! if Phase 4 surfaces a corpus binary that cares.

use super::apply_alias_rmw;

/// `TICK` register offset within the WATCHDOG block (datasheet §4.7.3).
pub const TICK_OFFSET: u32 = 0x2C;

/// Reset value for `CYCLES` — 12 cycles of `clk_ref` per microsecond.
/// pico-sdk writes this explicitly but real silicon resets to 0; the
/// default models the post-init state so a freshly-built [`Bus`] can
/// host `hello_timer` without firmware having to initialise the TICK
/// register first.
///
/// [`Bus`]: crate::bus::Bus
pub const CYCLES_RESET: u16 = 12;

/// WATCHDOG_TICK register storage.
///
/// Only the `TICK` register (offset `0x2C`) is modelled. All other
/// offsets within the watchdog block are Phase 1 no-ops (read 0, write
/// ignored) and are decoded at `Bus::peripheral_*32` dispatch time.
pub struct WatchdogTickRegs {
    /// `CYCLES[8:0]` — cycles of `clk_ref` per 1 µs TIMER tick.
    pub cycles: u16,
    /// `ENABLE[9]` — tick-generator enable.
    pub enable: bool,
    /// `RUNNING[10]` — mirrors `ENABLE` on read. Kept as a separate
    /// field so a future cycle-accurate model can drop the collapse
    /// without refactoring callers.
    pub running: bool,
}

impl WatchdogTickRegs {
    /// Construct in the post-init state (CYCLES = 12, ENABLE/RUNNING = 0).
    pub fn new() -> Self {
        Self {
            cycles: CYCLES_RESET,
            enable: false,
            running: false,
        }
    }

    /// Reset to power-on defaults. Called from `Emulator::reset()`.
    pub fn reset(&mut self) {
        self.cycles = CYCLES_RESET;
        self.enable = false;
        self.running = false;
    }

    /// Read a register by canonical offset within the watchdog block.
    /// Only `TICK` (`0x2C`) returns meaningful state; other offsets are
    /// 0 in Phase 1.
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            TICK_OFFSET => {
                let mut v = (self.cycles as u32) & 0x1FF;
                if self.enable {
                    v |= 1 << 9;
                }
                if self.running {
                    v |= 1 << 10;
                }
                v
            }
            _ => 0,
        }
    }

    /// Write a register with an APB alias in the normalised 2-bit form
    /// (`0` plain / `1` XOR / `2` BITSET / `3` BITCLR). Only `TICK` is
    /// writable in Phase 1; other offsets are no-ops.
    ///
    /// RUNNING mirrors ENABLE — any transition on bit 9 transitions
    /// bit 10 on the same cycle. This collapses the "takes effect one
    /// cycle later" silicon delay into an instant transition, which is
    /// sufficient for firmware that polls `RUNNING` after `ENABLE`
    /// (there's no corpus binary distinguishing the two cadences).
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        if offset != TICK_OFFSET {
            return;
        }
        // Rebuild the stored word, apply the alias RMW, then decode
        // back into fields. This keeps the alias math in a single
        // place (the shared helper) rather than re-implementing it
        // per bit field.
        let mut word = (self.cycles as u32) & 0x1FF;
        if self.enable {
            word |= 1 << 9;
        }
        if self.running {
            word |= 1 << 10;
        }
        apply_alias_rmw(&mut word, value, alias);
        self.cycles = (word & 0x1FF) as u16;
        self.enable = (word & (1 << 9)) != 0;
        // RUNNING mirrors ENABLE on the same cycle — see doc comment.
        self.running = self.enable;
    }
}

impl Default for WatchdogTickRegs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_defaults_cycles_12_enable_off() {
        let t = WatchdogTickRegs::new();
        assert_eq!(t.cycles, 12);
        assert!(!t.enable);
        assert!(!t.running);
        assert_eq!(t.read32(TICK_OFFSET), 12);
    }

    #[test]
    fn read_non_tick_offset_is_zero() {
        let t = WatchdogTickRegs::new();
        assert_eq!(t.read32(0x00), 0);
        assert_eq!(t.read32(0x04), 0);
        assert_eq!(t.read32(0x30), 0);
    }

    #[test]
    fn plain_write_cycles_roundtrips() {
        let mut t = WatchdogTickRegs::new();
        t.write32(TICK_OFFSET, 0x0000_0041, 0); // CYCLES = 65, no ENABLE
        assert_eq!(t.cycles, 65);
        assert!(!t.enable);
        // Full-word readback (ENABLE off so bit 9/10 are zero).
        assert_eq!(t.read32(TICK_OFFSET), 0x0000_0041);
    }

    #[test]
    fn plain_write_enable_sets_running() {
        let mut t = WatchdogTickRegs::new();
        // CYCLES = 12 (default) + ENABLE bit 9 = 0x200 | 0x0C
        t.write32(TICK_OFFSET, 0x0000_020C, 0);
        assert_eq!(t.cycles, 12);
        assert!(t.enable);
        assert!(t.running);
        // Read-back surfaces RUNNING bit 10 as well.
        let v = t.read32(TICK_OFFSET);
        assert_eq!(v & (1 << 9), 1 << 9);
        assert_eq!(v & (1 << 10), 1 << 10);
        assert_eq!(v & 0x1FF, 12);
    }

    #[test]
    fn bitset_alias_flips_enable_only() {
        let mut t = WatchdogTickRegs::new();
        // BITSET alias (2): assert ENABLE (bit 9) without disturbing CYCLES.
        t.write32(TICK_OFFSET, 1 << 9, 2);
        assert_eq!(t.cycles, CYCLES_RESET);
        assert!(t.enable);
        assert!(t.running);
    }

    #[test]
    fn bitclr_alias_clears_enable_only() {
        let mut t = WatchdogTickRegs::new();
        t.write32(TICK_OFFSET, 1 << 9, 2); // BITSET: enable on
        assert!(t.enable);
        t.write32(TICK_OFFSET, 1 << 9, 3); // BITCLR: clear bit 9
        assert!(!t.enable);
        assert!(!t.running);
        assert_eq!(t.cycles, CYCLES_RESET);
    }

    #[test]
    fn non_tick_offset_write_is_noop() {
        let mut t = WatchdogTickRegs::new();
        t.write32(0x04, 0xDEAD_BEEF, 0);
        assert_eq!(t.cycles, CYCLES_RESET);
        assert!(!t.enable);
    }

    #[test]
    fn reset_restores_post_init_state() {
        let mut t = WatchdogTickRegs::new();
        t.write32(TICK_OFFSET, 0x0000_03FF, 0); // CYCLES=511, ENABLE=1
        assert_eq!(t.cycles, 0x1FF);
        assert!(t.enable);
        t.reset();
        assert_eq!(t.cycles, CYCLES_RESET);
        assert!(!t.enable);
        assert!(!t.running);
    }
}

//! RP2350 TIMER peripheral (datasheet §12.8).
//!
//! RP2350 has two TIMER blocks (`TIMER0` at `0x400B_0000`, `TIMER1` at
//! `0x400B_8000`), each with a 64-bit microsecond counter, four 32-bit
//! alarms, and four IRQ lines. Both timers draw their 1 µs cadence from
//! the dedicated [`super::ticks`] block rather than the combined
//! WATCHDOG_TICK on RP2040.
//!
//! # Register map (offsets relative to `TIMERn_BASE`)
//!
//! | Offset | Name       | Access | Notes                                   |
//! |--------|------------|--------|-----------------------------------------|
//! | `0x00` | `TIMEHW`   | W      | V5 no-op (software-poke of time value). |
//! | `0x04` | `TIMELW`   | W      | V5 no-op.                               |
//! | `0x08` | `TIMEHR`   | R      | Latched by prior `TIMELR` read.         |
//! | `0x0C` | `TIMELR`   | R      | Low 32b of counter; also latches hi.    |
//! | `0x10` | `ALARM0`   | R/W    | Writing ARMS and schedules.             |
//! | `0x14` | `ALARM1`   | R/W    |                                         |
//! | `0x18` | `ALARM2`   | R/W    |                                         |
//! | `0x1C` | `ALARM3`   | R/W    |                                         |
//! | `0x20` | `ARMED`    | R/W    | Writing 1 DISARMS (inverse-W1C).        |
//! | `0x24` | `TIMERAWH` | R      | High 32b of counter (no latch).         |
//! | `0x28` | `TIMERAWL` | R      | Low 32b of counter (no latch).          |
//! | `0x2C` | `DBGPAUSE` | R/W    | Plain storage (V5).                     |
//! | `0x30` | `PAUSE`    | R/W    | Plain storage (V5).                     |
//! | `0x34` | `LOCKED`   | R      | RAZ in V5 (unmodelled; 0).              |
//! | `0x38` | `SOURCE`   | R      | `0x2` — TICKS domain (hardcoded).       |
//! | `0x3C` | `INTR`     | R/W1C  | Writing 1 clears.                       |
//! | `0x40` | `INTE`     | R/W    | Plain storage (alias-aware).            |
//! | `0x44` | `INTF`     | R/W    | Plain storage (alias-aware).            |
//! | `0x48` | `INTS`     | R      | `(intr \| intf) & inte`.                |
//!
//! # Tick source and lazy scheduling
//!
//! Each TIMER draws 1 µs edges from its matching TICKS domain
//! ([`super::ticks::DOMAIN_TIMER0`] / `DOMAIN_TIMER1`). On every
//! `Bus::tick_peripherals`, the bus drains pending edges from the
//! TICKS domain and calls [`TimerRegs::advance_us`] with the count.
//!
//! Lazy scheduling: alarms are scheduled in **microsecond space** at
//! write time. [`TimerRegs::poll_alarms`] compares `count_us` to each
//! scheduled fire-microsecond and emits NVIC bits for matches. Cached
//! fire-microseconds invalidate when firmware reprograms the TICKS
//! rate (HLD V5 §5.4) — the bus calls [`TimerRegs::invalidate_lazy`]
//! for both timers after any TICKS write that can shift the rate.

use super::apply_alias_rmw;

/// TIMER0 base (RP2350 datasheet §12.8).
pub const TIMER0_BASE: u32 = 0x400B_0000;
/// TIMER1 base (RP2350 datasheet §12.8).
pub const TIMER1_BASE: u32 = 0x400B_8000;

/// Offset: `TIMEHW` (write latch for TIMEHR) — 0x00.
pub const TIMEHW_OFFSET: u32 = 0x00;
/// Offset: `TIMELW` (commits TIMEHW/TIMELW pair on write) — 0x04.
pub const TIMELW_OFFSET: u32 = 0x04;
/// Offset: `TIMEHR` (read-only, latched) — 0x08.
pub const TIMEHR_OFFSET: u32 = 0x08;
/// Offset: `TIMELR` (read; latches TIMEHR) — 0x0C.
pub const TIMELR_OFFSET: u32 = 0x0C;
/// Offset: `ALARM0` — 0x10.
pub const ALARM0_OFFSET: u32 = 0x10;
/// Offset: `ARMED` (write-1-disarms) — 0x20.
pub const ARMED_OFFSET: u32 = 0x20;
/// Offset: `TIMERAWH` — 0x24.
pub const TIMERAWH_OFFSET: u32 = 0x24;
/// Offset: `TIMERAWL` — 0x28.
pub const TIMERAWL_OFFSET: u32 = 0x28;
/// Offset: `DBGPAUSE` — 0x2C.
pub const DBGPAUSE_OFFSET: u32 = 0x2C;
/// Offset: `PAUSE` — 0x30.
pub const PAUSE_OFFSET: u32 = 0x30;
/// Offset: `LOCKED` — 0x34. RP2350-only (RAZ in V5).
pub const LOCKED_OFFSET: u32 = 0x34;
/// Offset: `SOURCE` — 0x38. RP2350-only; reports the tick source index.
pub const SOURCE_OFFSET: u32 = 0x38;
/// Offset: `INTR` (W1C) — 0x3C (RP2350 — shifted from RP2040's 0x34 to
/// make room for LOCKED/SOURCE).
pub const INTR_OFFSET: u32 = 0x3C;
/// Offset: `INTE` — 0x40.
pub const INTE_OFFSET: u32 = 0x40;
/// Offset: `INTF` — 0x44.
pub const INTF_OFFSET: u32 = 0x44;
/// Offset: `INTS` — 0x48.
pub const INTS_OFFSET: u32 = 0x48;

/// `DBGPAUSE` occupies 3 bits.
const DBGPAUSE_MASK: u32 = 0b111;
/// `PAUSE` occupies 1 bit.
const PAUSE_MASK: u32 = 1;
/// INTR / INTE / INTF / ARMED occupy 4 bits (one per alarm).
const ALARM_MASK_4BITS: u32 = 0xF;

/// `SOURCE` register value reported by V5 — the TIMER is driven from
/// the TICKS block (index 2 per datasheet §12.8.4).
const SOURCE_TICKS: u32 = 0x2;

/// TIMER register storage.
pub struct TimerRegs {
    /// 64-bit microsecond counter. Advanced by [`Self::advance_us`]
    /// when TICKS emits edges.
    count_us: u64,
    /// Alarm target microsecond values (low 32 bits of the modular
    /// match space, per silicon).
    alarm_target_us: [u32; 4],
    /// Cached fire-microsecond (absolute, 64-bit) per alarm. `None`
    /// when the alarm is not armed. Recomputed on ALARM write and
    /// invalidated on TICKS rate changes (though since we work in
    /// µs space rather than sys_clk space, invalidation is defensive
    /// against future refactors that move the cache to sys_clk).
    alarm_fire_us: [Option<u64>; 4],
    /// Armed bit per alarm (bit N = alarm N armed).
    armed: u8,
    /// Latched pending bits per alarm.
    intr: u8,
    /// Interrupt enable mask.
    inte: u8,
    /// Interrupt force mask.
    intf: u8,
    /// PAUSE[0] — plain storage.
    pause: bool,
    /// DBGPAUSE[2:0] — plain storage.
    dbgpause: u8,
    /// High 32 bits of TIMER latched by the prior TIMELR read.
    timehr_latched: u32,
    /// Base NVIC IRQ number. TIMER0 uses
    /// [`crate::irq::IRQ_TIMER0_IRQ_0`] (0); TIMER1 uses
    /// [`crate::irq::IRQ_TIMER1_IRQ_0`] (4). Stored per-instance so
    /// `poll_alarms` can return the right bitmap.
    irq_base: u32,
}

impl TimerRegs {
    /// Create a post-reset TIMER register block. `irq_base` is the
    /// IRQ number of alarm 0 — 0 for TIMER0, 4 for TIMER1.
    pub fn new(irq_base: u32) -> Self {
        Self {
            count_us: 0,
            alarm_target_us: [0; 4],
            alarm_fire_us: [None; 4],
            armed: 0,
            intr: 0,
            inte: 0,
            intf: 0,
            pause: false,
            dbgpause: 0,
            timehr_latched: 0,
            irq_base,
        }
    }

    /// Reset to power-on defaults, preserving the IRQ base.
    pub fn reset(&mut self) {
        let base = self.irq_base;
        *self = Self::new(base);
    }

    /// Advance the 64-bit microsecond counter by `edges` (one per
    /// TICKS-domain edge). Called from `Bus::tick_peripherals`.
    ///
    /// Skips advance if `PAUSE` or `DBGPAUSE` bits are set (firmware
    /// convention: DBGPAUSE[1] halts on proc0-halted, DBGPAUSE[2] on
    /// proc1-halted, PAUSE[0] is a software halt). V5 models DBGPAUSE
    /// as plain storage — we only honour `PAUSE`, matching V5's
    /// "DBGPAUSE plain storage" policy.
    pub fn advance_us(&mut self, edges: u32) {
        if self.pause {
            return;
        }
        self.count_us = self.count_us.wrapping_add(edges as u64);
    }

    /// Poll all armed alarms and fire any whose target microsecond
    /// timestamp has been reached.
    ///
    /// Returns the NVIC IRQ bitmap in **absolute IRQ numbers** — bit
    /// `irq_base + n` is set if alarm n fires or remains latched with
    /// INTE enabled. The caller OR's the result into `bus.irq_pending`
    /// / `bus.assert_irq_shared` per HLD V5 §5.3.
    pub fn poll_alarms(&mut self) -> u64 {
        let mut nvic_bits = 0u64;
        for n in 0..4 {
            if self.armed & (1 << n) == 0 {
                continue;
            }
            if let Some(fire_us) = self.alarm_fire_us[n]
                && self.count_us >= fire_us
            {
                self.intr |= 1 << n;
                self.armed &= !(1 << n);
                self.alarm_fire_us[n] = None;
            }
        }
        // Level re-assert: any alarm whose INTR bit is still latched
        // AND whose INTE is set re-raises the NVIC line on every poll.
        // The CPU clears the NVIC pending bit on dispatch; level-triggered
        // sources are expected to re-assert until the ISR W1Cs INTR.
        let live = (self.intr & self.inte) as u64 & ALARM_MASK_4BITS as u64;
        nvic_bits |= live << self.irq_base;
        // INTF forces the NVIC line even without a match.
        let forced = (self.intf & self.inte) as u64 & ALARM_MASK_4BITS as u64;
        nvic_bits |= forced << self.irq_base;
        nvic_bits
    }

    /// No-op cache invalidation for TIMER's lazy schedule. Retained as
    /// a public entry point because the bus signals rate-change touches
    /// to both TIMER0 and TIMER1 (HLD V5 §5.4), but the TIMER cache is
    /// **rate-invariant**: `alarm_fire_us[n]` is the absolute
    /// microsecond timestamp at which the alarm must fire, and `count_us`
    /// is also microseconds, so neither depends on the TICKS divider
    /// setting. A rate change cancels no pending alarm; firmware need
    /// not re-arm.
    ///
    /// Silicon matches this behaviour (RP2350 datasheet §12.8 — an
    /// armed alarm whose target has not yet been reached continues
    /// running at whatever new cadence TICKS is reprogrammed to; the
    /// counter keeps advancing until the match point lands).
    ///
    /// The public `pub fn` shape stays so future changes that move the
    /// cache back into sys_clk space (if we ever cache the sys_clk
    /// count instead of the µs count) have a pre-wired invalidate path
    /// without touching every Bus call site. R1 / V5 §5.4 comment trail.
    pub fn invalidate_lazy(&mut self) {
        // Deliberately empty — see doc comment.
    }

    // -------------------------------------------------------------------
    // Register dispatch
    // -------------------------------------------------------------------

    /// Read a TIMER register.
    pub fn read32(&mut self, offset: u32) -> u32 {
        match offset {
            TIMEHW_OFFSET | TIMELW_OFFSET => 0, // write-only; reads RAZ
            TIMEHR_OFFSET => self.timehr_latched,
            TIMELR_OFFSET => {
                // Latch high half so a subsequent TIMEHR read is
                // consistent with this snapshot (datasheet §12.8.2).
                self.timehr_latched = (self.count_us >> 32) as u32;
                self.count_us as u32
            }
            ALARM0_OFFSET..=0x1C => {
                let idx = ((offset - ALARM0_OFFSET) >> 2) as usize;
                if idx < 4 {
                    self.alarm_target_us[idx]
                } else {
                    0
                }
            }
            ARMED_OFFSET => (self.armed & 0xF) as u32,
            TIMERAWH_OFFSET => (self.count_us >> 32) as u32,
            TIMERAWL_OFFSET => self.count_us as u32,
            DBGPAUSE_OFFSET => self.dbgpause as u32,
            PAUSE_OFFSET => {
                if self.pause {
                    1
                } else {
                    0
                }
            }
            LOCKED_OFFSET => 0, // V5: unmodelled (RAZ)
            SOURCE_OFFSET => SOURCE_TICKS,
            INTR_OFFSET => (self.intr & 0xF) as u32,
            INTE_OFFSET => (self.inte & 0xF) as u32,
            INTF_OFFSET => (self.intf & 0xF) as u32,
            INTS_OFFSET => ((self.intr | self.intf) & self.inte) as u32 & ALARM_MASK_4BITS,
            _ => 0,
        }
    }

    /// Write a TIMER register. `alias` is the 2-bit normalised alias
    /// form (0 plain / 1 XOR / 2 BITSET / 3 BITCLR).
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        match offset {
            // V5: software-poke of TIMER value is a no-op.
            TIMEHW_OFFSET | TIMELW_OFFSET | TIMEHR_OFFSET => {}
            TIMELR_OFFSET => {}
            ALARM0_OFFSET..=0x1C => {
                let idx = ((offset - ALARM0_OFFSET) >> 2) as usize;
                if idx >= 4 {
                    return;
                }
                let mut stored = self.alarm_target_us[idx];
                apply_alias_rmw(&mut stored, value, alias);
                self.alarm_target_us[idx] = stored;
                // Arm + schedule. Compute the fire-microsecond: target
                // is the low 32 bits of a future 64-bit `count_us`. If
                // the target is in the past relative to `count_us`,
                // wrap to the next modular 32-bit match per pico-sdk
                // semantics for short delays (identical to RP2040).
                self.armed |= 1 << idx;
                let target_us = stored as u64;
                let count_lo = (self.count_us as u32) as u64;
                let delta = target_us.wrapping_sub(count_lo) & 0xFFFF_FFFF;
                let fire_us = self.count_us.wrapping_add(delta);
                self.alarm_fire_us[idx] = Some(fire_us);
            }
            ARMED_OFFSET => {
                // Writing 1 to an ARMED bit DISARMS the alarm
                // (datasheet §12.8.3 — inverse W1C). Respect alias for
                // consistency; the final resolved word becomes the
                // disarm mask.
                let mut stored = self.armed as u32;
                apply_alias_rmw(&mut stored, value, alias);
                let disarm = stored as u8 & 0xF;
                self.armed &= !disarm;
                for n in 0..4 {
                    if disarm & (1 << n) != 0 {
                        self.alarm_fire_us[n] = None;
                    }
                }
            }
            TIMERAWH_OFFSET | TIMERAWL_OFFSET => {} // read-only
            DBGPAUSE_OFFSET => {
                let mut stored = self.dbgpause as u32;
                apply_alias_rmw(&mut stored, value, alias);
                self.dbgpause = (stored & DBGPAUSE_MASK) as u8;
            }
            PAUSE_OFFSET => {
                let mut stored = if self.pause { 1u32 } else { 0u32 };
                apply_alias_rmw(&mut stored, value, alias);
                self.pause = (stored & PAUSE_MASK) != 0;
            }
            LOCKED_OFFSET | SOURCE_OFFSET => {} // read-only (V5)
            INTR_OFFSET => {
                // W1C on post-alias-resolution bits.
                let mut stored = self.intr as u32;
                apply_alias_rmw(&mut stored, value, alias);
                let clr = (stored as u8) & 0xF;
                self.intr &= !clr;
            }
            INTE_OFFSET => {
                let mut stored = self.inte as u32;
                apply_alias_rmw(&mut stored, value, alias);
                self.inte = (stored & ALARM_MASK_4BITS) as u8;
            }
            INTF_OFFSET => {
                let mut stored = self.intf as u32;
                apply_alias_rmw(&mut stored, value, alias);
                self.intf = (stored & ALARM_MASK_4BITS) as u8;
            }
            INTS_OFFSET => {} // read-only
            _ => {}
        }
    }

    /// True iff no INTR bit is latched and no INTF-forced line is live.
    /// Analogous to RP2040's `is_idle`. Kept on the public surface for
    /// a future fast-path gate (HLD V5 §5.5 — deferred in V5).
    pub fn is_idle(&self) -> bool {
        self.intr == 0 && (self.intf & self.inte) == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::irq::{IRQ_TIMER0_IRQ_0, IRQ_TIMER1_IRQ_0};

    fn timer0() -> TimerRegs {
        TimerRegs::new(IRQ_TIMER0_IRQ_0)
    }

    fn timer1() -> TimerRegs {
        TimerRegs::new(IRQ_TIMER1_IRQ_0)
    }

    // --- Defaults + reset ------------------------------------------------

    #[test]
    fn new_defaults() {
        let t = timer0();
        assert_eq!(t.count_us, 0);
        assert_eq!(t.armed, 0);
        assert_eq!(t.intr, 0);
        assert_eq!(t.inte, 0);
        assert_eq!(t.intf, 0);
        assert!(!t.pause);
        assert_eq!(t.dbgpause, 0);
        assert!(t.alarm_fire_us.iter().all(|x| x.is_none()));
    }

    #[test]
    fn reset_preserves_irq_base() {
        let mut t = timer1();
        t.advance_us(100);
        t.intr = 0xF;
        t.reset();
        assert_eq!(t.count_us, 0);
        assert_eq!(t.intr, 0);
        assert_eq!(t.irq_base, IRQ_TIMER1_IRQ_0);
    }

    // --- Counter advance -------------------------------------------------

    #[test]
    fn advance_us_bumps_counter() {
        let mut t = timer0();
        t.advance_us(42);
        assert_eq!(t.count_us, 42);
        t.advance_us(8);
        assert_eq!(t.count_us, 50);
    }

    #[test]
    fn pause_halts_advance() {
        let mut t = timer0();
        t.pause = true;
        t.advance_us(100);
        assert_eq!(t.count_us, 0);
        t.pause = false;
        t.advance_us(100);
        assert_eq!(t.count_us, 100);
    }

    #[test]
    fn counter_wraps_at_u64_boundary() {
        let mut t = timer0();
        t.count_us = u64::MAX - 3;
        t.advance_us(10);
        assert_eq!(t.count_us, 6);
    }

    // --- TIMELR reads + TIMEHR latch -------------------------------------

    #[test]
    fn timelr_read_latches_timehr() {
        let mut t = timer0();
        // 64-bit value: high=5, low=7.
        t.count_us = (5u64 << 32) | 7;
        let lo = t.read32(TIMELR_OFFSET);
        assert_eq!(lo, 7);
        // TIMEHR now returns the latched high half.
        assert_eq!(t.read32(TIMEHR_OFFSET), 5);
    }

    #[test]
    fn timehr_returns_latched_even_after_advance() {
        let mut t = timer0();
        t.count_us = (1u64 << 32) | 10;
        t.read32(TIMELR_OFFSET);
        // Advance past the latched snapshot.
        t.advance_us(100);
        // TIMEHR must still return the old snapshot.
        assert_eq!(t.read32(TIMEHR_OFFSET), 1);
    }

    #[test]
    fn timerawl_reads_live_without_latch() {
        let mut t = timer0();
        t.count_us = 123;
        assert_eq!(t.read32(TIMERAWL_OFFSET), 123);
        assert_eq!(t.read32(TIMEHR_OFFSET), 0, "TIMERAWL does not latch");
    }

    #[test]
    fn timerawh_reads_live_high_without_latch() {
        let mut t = timer0();
        t.count_us = 5u64 << 32;
        assert_eq!(t.read32(TIMERAWH_OFFSET), 5);
    }

    // --- Alarms + IRQ ----------------------------------------------------

    #[test]
    fn alarm_write_arms_and_schedules() {
        let mut t = timer0();
        t.write32(ALARM0_OFFSET, 100, 0);
        assert_eq!(t.armed & 1, 1, "ALARM write must arm");
        assert_eq!(t.alarm_target_us[0], 100);
        assert_eq!(t.alarm_fire_us[0], Some(100));
    }

    #[test]
    fn poll_alarms_fires_on_match_without_inte() {
        let mut t = timer0();
        t.write32(ALARM0_OFFSET, 100, 0);
        t.advance_us(99);
        assert_eq!(t.poll_alarms(), 0, "no fire before target");
        t.advance_us(1);
        let bits = t.poll_alarms();
        // INTR latches; INTE is 0 so no NVIC line raised.
        assert_eq!(bits, 0);
        assert_eq!(t.intr & 1, 1, "INTR bit 0 must latch on match");
        assert_eq!(t.armed & 1, 0, "alarm auto-disarms after fire");
    }

    #[test]
    fn poll_alarms_raises_nvic_when_inte_set() {
        let mut t = timer0();
        t.write32(INTE_OFFSET, 1, 0);
        t.write32(ALARM0_OFFSET, 100, 0);
        t.advance_us(100);
        let bits = t.poll_alarms();
        assert_eq!(
            bits & (1u64 << IRQ_TIMER0_IRQ_0),
            1u64 << IRQ_TIMER0_IRQ_0,
            "INTE=1 routes alarm 0 to NVIC IRQ_TIMER0_IRQ_0"
        );
    }

    #[test]
    fn timer1_fires_on_its_own_irq_base() {
        let mut t = timer1();
        t.write32(INTE_OFFSET, 1, 0);
        t.write32(ALARM0_OFFSET, 50, 0);
        t.advance_us(50);
        let bits = t.poll_alarms();
        // TIMER1 alarm 0 fires on IRQ 4.
        assert_eq!(bits & (1u64 << IRQ_TIMER1_IRQ_0), 1u64 << IRQ_TIMER1_IRQ_0);
        // No TIMER0 IRQ bits set.
        assert_eq!(bits & (1u64 << IRQ_TIMER0_IRQ_0), 0);
    }

    #[test]
    fn poll_alarms_re_asserts_latched_level_until_w1c() {
        // Level-IRQ re-assert: after fire, INTR stays latched and
        // every subsequent poll must re-raise the NVIC line while
        // INTE remains set.
        let mut t = timer0();
        t.write32(INTE_OFFSET, 1, 0);
        t.write32(ALARM0_OFFSET, 100, 0);
        t.advance_us(100);
        let n1 = t.poll_alarms();
        assert_eq!(n1 & (1u64 << IRQ_TIMER0_IRQ_0), 1u64 << IRQ_TIMER0_IRQ_0);
        // Second poll without clearing INTR → NVIC re-asserted.
        let n2 = t.poll_alarms();
        assert_eq!(n2 & (1u64 << IRQ_TIMER0_IRQ_0), 1u64 << IRQ_TIMER0_IRQ_0);
        // After W1C of INTR, the level drops.
        t.write32(INTR_OFFSET, 1, 0);
        let n3 = t.poll_alarms();
        assert_eq!(n3 & (1u64 << IRQ_TIMER0_IRQ_0), 0);
    }

    #[test]
    fn intr_write_is_w1c() {
        let mut t = timer0();
        t.write32(ALARM0_OFFSET, 50, 0);
        t.advance_us(50);
        t.poll_alarms();
        assert_eq!(t.intr, 1);
        t.write32(INTR_OFFSET, 1, 0);
        assert_eq!(t.intr, 0);
    }

    #[test]
    fn intr_write_zero_does_not_clear() {
        let mut t = timer0();
        t.intr = 0xF;
        t.write32(INTR_OFFSET, 0, 0);
        assert_eq!(t.intr, 0xF);
    }

    #[test]
    fn armed_write_disarms() {
        let mut t = timer0();
        t.write32(ALARM0_OFFSET, 100, 0);
        assert_eq!(t.armed & 1, 1);
        t.write32(ARMED_OFFSET, 1, 0);
        assert_eq!(t.armed & 1, 0);
        assert!(t.alarm_fire_us[0].is_none());
    }

    #[test]
    fn ints_reads_latched_and_inte_gated() {
        let mut t = timer0();
        t.intr = 0x3;
        t.inte = 0x1;
        assert_eq!(t.read32(INTS_OFFSET), 0x1);
    }

    #[test]
    fn intf_forces_ints_even_without_match() {
        let mut t = timer0();
        t.inte = 0x4;
        t.intf = 0x4;
        assert_eq!(t.read32(INTS_OFFSET), 0x4);
    }

    // --- Multi-alarm independence ---------------------------------------

    #[test]
    fn four_alarms_fire_independently() {
        let mut t = timer0();
        t.write32(INTE_OFFSET, 0xF, 0);
        t.write32(ALARM0_OFFSET, 10, 0);
        t.write32(ALARM0_OFFSET + 4, 20, 0);
        t.write32(ALARM0_OFFSET + 8, 30, 0);
        t.write32(ALARM0_OFFSET + 12, 40, 0);
        t.advance_us(10);
        t.poll_alarms();
        assert_eq!(t.intr, 0x1);
        assert_eq!(t.armed & 0xF, 0xE);
        t.advance_us(30);
        t.poll_alarms();
        assert_eq!(t.intr, 0xF);
        assert_eq!(t.armed & 0xF, 0);
    }

    // --- Invalidate ------------------------------------------------------

    #[test]
    fn invalidate_lazy_is_no_op_preserving_armed_and_cache() {
        // R1: TICKS-rate-change invalidation is a no-op. TIMER state is
        // rate-invariant (µs-based), so firmware does not need to re-arm
        // after reprogramming TICKS — silicon matches this (datasheet
        // §12.8). Armed bits AND the cached fire-microseconds survive.
        let mut t = timer0();
        t.write32(ALARM0_OFFSET, 100, 0);
        t.write32(ALARM0_OFFSET + 4, 200, 0);
        let armed_before = t.armed;
        let cache_before = t.alarm_fire_us;
        t.invalidate_lazy();
        assert_eq!(
            t.armed, armed_before,
            "armed bits must survive a TICKS rate change (R1)"
        );
        assert_eq!(
            t.alarm_fire_us, cache_before,
            "cache is rate-invariant; invalidate_lazy is a no-op"
        );
        // Sanity: alarm still fires at its original target.
        t.advance_us(100);
        t.poll_alarms();
        assert_eq!(
            t.intr & 0x1,
            0x1,
            "armed alarm must fire after invalidate_lazy (no-op)"
        );
    }

    // --- LOCKED / SOURCE ------------------------------------------------

    #[test]
    fn source_reads_ticks_index() {
        let mut t = timer0();
        assert_eq!(t.read32(SOURCE_OFFSET), SOURCE_TICKS);
    }

    #[test]
    fn locked_reads_zero_in_v5() {
        let mut t = timer0();
        assert_eq!(t.read32(LOCKED_OFFSET), 0);
    }

    #[test]
    fn locked_source_writes_are_noop() {
        let mut t = timer0();
        t.write32(LOCKED_OFFSET, 0xFFFF_FFFF, 0);
        t.write32(SOURCE_OFFSET, 0xFFFF_FFFF, 0);
        assert_eq!(t.read32(LOCKED_OFFSET), 0);
        assert_eq!(t.read32(SOURCE_OFFSET), SOURCE_TICKS);
    }

    // --- is_idle --------------------------------------------------------

    #[test]
    fn is_idle_false_when_intr_latched() {
        let mut t = timer0();
        t.intr = 0x1;
        assert!(!t.is_idle());
    }

    #[test]
    fn is_idle_true_with_armed_but_no_pending() {
        let mut t = timer0();
        t.write32(ALARM0_OFFSET, 100, 0);
        assert!(t.is_idle());
    }

    // --- Alias semantics ------------------------------------------------

    #[test]
    fn inte_bitset_alias() {
        let mut t = timer0();
        t.write32(INTE_OFFSET, 0x2, 2);
        assert_eq!(t.inte, 0x2);
        t.write32(INTE_OFFSET, 0x4, 2);
        assert_eq!(t.inte, 0x6);
    }

    #[test]
    fn inte_bitclr_alias() {
        let mut t = timer0();
        t.inte = 0xF;
        t.write32(INTE_OFFSET, 0x5, 3);
        assert_eq!(t.inte, 0xA);
    }
}

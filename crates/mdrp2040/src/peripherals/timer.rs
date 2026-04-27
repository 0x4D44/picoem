//! RP2040 TIMER peripheral (datasheet §4.6).
//!
//! Phase 1 Wave 2 of the RP2040 peripheral coverage plan (HLD V7 §5.3).
//! The TIMER block sits at `0x4005_4000`. On real silicon it counts
//! microseconds driven by WATCHDOG_TICK (`clk_ref / WATCHDOG_TICK.CYCLES`
//! cycles per tick). The emulator models TIMER *lazily*: a call to
//! [`TimerRegs::now_us`] computes the current microsecond count from
//! `master_cycle` and `sys_clk_hz`, and [`TimerRegs::poll_alarms`]
//! surfaces alarm-match IRQs whose fire cycle falls at or before the
//! current cycle. No per-cycle advance — the peripheral stays in the
//! fast-path "idle" set.
//!
//! # Simplifications carried into Phase 1
//!
//! * The software-poke path (`TIMEHW`/`TIMELW` writes that commit a new
//!   64-bit time value when `TIMELW` lands) is a no-op. pico-sdk's
//!   `busy_wait_us_32` / `hello_timer` corpus does not write the time
//!   registers; we'll revisit when a corpus binary demands it.
//! * Time conversion assumes one microsecond per `sys_hz / 1_000_000`
//!   sysclk cycles — i.e. we collapse WATCHDOG_TICK's divider out of
//!   the formula. Firmware reprogramming `clk_peri` or `WATCHDOG_TICK.
//!   CYCLES` is a Phase 2+ issue.
//! * `DBGPAUSE` and `PAUSE` are plain storage — firmware round-trip
//!   only; they do not gate alarm scheduling. `busy_wait_us_32` never
//!   touches either.
//! * 32-bit ALARM wrap math: an alarm scheduled across the 32-bit low
//!   boundary of `now` (i.e. `target_us < now_lo` with the arming
//!   firmware intending the *next* modular match) is handled at write
//!   time by `fire_cycle = now + ((target - now_lo) & 0xFFFF_FFFF)`
//!   in master-cycle space, but `poll_alarms` itself does not re-check
//!   the wrap across a 32-bit boundary while the alarm waits — the
//!   wrap-aware match path is deferred to Phase 2+.
//!
//! # Register map (offsets relative to `TIMER_BASE`)
//!
//! | Offset | Name       | Access | Notes                             |
//! |--------|------------|--------|-----------------------------------|
//! | `0x00` | `TIMEHW`   | W      | Phase 1: no-op.                   |
//! | `0x04` | `TIMELW`   | W      | Phase 1: no-op.                   |
//! | `0x08` | `TIMEHR`   | R      | `timehr_latched` from prior read. |
//! | `0x0C` | `TIMELR`   | R      | Low 32b of now; also latches hi.  |
//! | `0x10` | `ALARM0`   | R/W    | Writing ARMs and schedules.       |
//! | `0x14` | `ALARM1`   | R/W    | "                                 |
//! | `0x18` | `ALARM2`   | R/W    | "                                 |
//! | `0x1C` | `ALARM3`   | R/W    | "                                 |
//! | `0x20` | `ARMED`    | R/W    | Writing 1 DISARMS the bit.        |
//! | `0x24` | `TIMERAWH` | R      | High 32b of now (no latch).       |
//! | `0x28` | `TIMERAWL` | R      | Low 32b of now (no latch).        |
//! | `0x2C` | `DBGPAUSE` | R/W    | Plain storage.                    |
//! | `0x30` | `PAUSE`    | R/W    | Plain storage.                    |
//! | `0x34` | `INTR`     | R/W1C  | Writing 1 clears.                 |
//! | `0x38` | `INTE`     | R/W    | Plain storage (alias-aware).      |
//! | `0x3C` | `INTF`     | R/W    | Plain storage (alias-aware).      |
//! | `0x40` | `INTS`     | R      | `(intr \| intf) & inte`.          |
//!
//! # IRQ delivery
//!
//! Each alarm has one NVIC line (TIMER_IRQ_0..3 = IRQs 0..3). When
//! [`TimerRegs::poll_alarms`] finds `fire_cycle[n] <= now_cycles && armed
//! & (1 << n) != 0`:
//!
//! 1. Set `intr |= 1 << n` (latched pending).
//! 2. Clear `armed[n]` (alarm must be re-armed by firmware).
//! 3. If `(inte | intf) & (1 << n) != 0`, raise `IRQ_TIMER_IRQ_0 + n` in
//!    `bus.irq_pending`.
//!
//! The INTS view combines `intr | intf` masked by `inte` — matches real
//! silicon.

use super::apply_alias_rmw;

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
/// Offset: `ARMED` (write-1-clears) — 0x20.
pub const ARMED_OFFSET: u32 = 0x20;
/// Offset: `TIMERAWH` — 0x24.
pub const TIMERAWH_OFFSET: u32 = 0x24;
/// Offset: `TIMERAWL` — 0x28.
pub const TIMERAWL_OFFSET: u32 = 0x28;
/// Offset: `DBGPAUSE` — 0x2C.
pub const DBGPAUSE_OFFSET: u32 = 0x2C;
/// Offset: `PAUSE` — 0x30.
pub const PAUSE_OFFSET: u32 = 0x30;
/// Offset: `INTR` (W1C) — 0x34.
pub const INTR_OFFSET: u32 = 0x34;
/// Offset: `INTE` — 0x38.
pub const INTE_OFFSET: u32 = 0x38;
/// Offset: `INTF` — 0x3C.
pub const INTF_OFFSET: u32 = 0x3C;
/// Offset: `INTS` — 0x40.
pub const INTS_OFFSET: u32 = 0x40;

/// DBGPAUSE occupies 3 bits.
const DBGPAUSE_MASK: u32 = 0b111;
/// PAUSE occupies 1 bit.
const PAUSE_MASK: u32 = 1;
/// INTR / INTE / INTF / ARMED occupy 4 bits (one per alarm).
const ALARM_MASK_4BITS: u32 = 0xF;

/// TIMER register storage (Phase 1 Wave 2).
pub struct TimerRegs {
    // Scheduled alarm fire-cycles in sys_clk master-cycle space. `None`
    // = alarm not armed. Recomputed whenever `ALARM[n]` is written.
    alarm_target_us: [u32; 4],
    alarm_fire_cycle: [Option<u64>; 4],
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
}

impl TimerRegs {
    /// Create in the post-init state (all fields zero / disarmed).
    pub fn new() -> Self {
        Self {
            alarm_target_us: [0; 4],
            alarm_fire_cycle: [None; 4],
            armed: 0,
            intr: 0,
            inte: 0,
            intf: 0,
            pause: false,
            dbgpause: 0,
            timehr_latched: 0,
        }
    }

    /// Reset to power-on defaults. Called from `Emulator::reset`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Convert a master sys-clock cycle count to microseconds given the
    /// current `sys_hz`. See the module docstring's simplification note
    /// — Phase 1 assumes `micros = master_cycle / (sys_hz / 1_000_000)`.
    /// Guards against div-by-zero (returns 0).
    #[inline]
    fn cycles_to_us(master_cycle: u64, sys_hz: u32) -> u64 {
        let divisor = (sys_hz / 1_000_000).max(1) as u64;
        master_cycle / divisor
    }

    /// Convert microseconds back to master sys-clock cycles. Used to
    /// schedule alarm fire cycles at write time.
    #[inline]
    fn us_to_cycles(us: u64, sys_hz: u32) -> u64 {
        let multiplier = (sys_hz / 1_000_000).max(1) as u64;
        us.saturating_mul(multiplier)
    }

    /// Current TIMER value in microseconds, given the bus's master-
    /// cycle count and system-clock frequency.
    pub fn now_us(&self, master_cycle: u64, sys_hz: u32) -> u64 {
        Self::cycles_to_us(master_cycle, sys_hz)
    }

    /// Poll all armed alarms and fire any whose target microsecond
    /// timestamp has been reached.
    ///
    /// Returns the NVIC IRQ bitmap to OR into `bus.irq_pending` (bit N
    /// corresponds to `IRQ_TIMER_IRQ_0 + N` at N in `0..4`). The caller
    /// is expected to fold this into `bus.irq_pending` under the
    /// ownership rules in HLD V7 §5.5.
    pub fn poll_alarms(&mut self, master_cycle: u64, _sys_hz: u32) -> u32 {
        let mut nvic_bits = 0u32;
        for n in 0..4 {
            if self.armed & (1 << n) == 0 {
                continue;
            }
            // "fire now?" — for the Phase 1 model we fire whenever the
            // stored `fire_cycle` <= `master_cycle` so corpus
            // `busy_wait_us` semantics hold. Wrap-around across the
            // 32-bit ALARM boundary is a Phase 2+ concern (see the
            // module doc).
            if let Some(fc) = self.alarm_fire_cycle[n]
                && master_cycle >= fc
            {
                self.intr |= 1 << n;
                self.armed &= !(1 << n);
                self.alarm_fire_cycle[n] = None;
                // Only fire the NVIC line if INTE has this alarm enabled.
                // INTF allows firmware to force the line without an actual
                // alarm match.
                if (self.inte | self.intf) & (1 << n) != 0 {
                    nvic_bits |= 1u32 << n;
                }
            }
        }
        // Level re-assert: any alarm whose INTR bit is still latched
        // AND whose INTE is set must re-raise on every poll, not only
        // on the fresh match edge. See the level-vs-edge note in
        // `crate::core::nvic`: the NVIC pending bit is cleared on
        // dispatch, and for level-triggered sources the peripheral
        // itself is expected to re-assert while the condition holds
        // (INTR latched AND enabled) until the ISR W1Cs INTR.
        nvic_bits |= (self.intr & self.inte) as u32 & ALARM_MASK_4BITS;
        // INTF contributions that aren't tied to an armed alarm still
        // raise the NVIC line each poll — firmware may set INTF to test
        // handlers without arming.
        nvic_bits |= (self.intf & self.inte) as u32 & ALARM_MASK_4BITS;
        nvic_bits
    }

    /// True iff the TIMER has "nothing observable pending" — i.e. no
    /// INTR bit latched. TIMER is always fast-path-idle from the per-
    /// cycle tick perspective (alarms fire through `poll_alarms`), but
    /// an already-latched INTR may need to route to the NVIC on the
    /// next step; callers that fold this into a gate should OR it with
    /// INTR != 0.
    pub fn is_idle(&self) -> bool {
        self.intr == 0 && (self.intf & self.inte) == 0
    }

    // -------------------------------------------------------------------
    // Register dispatch
    // -------------------------------------------------------------------

    /// Read a TIMER register. `master_cycle` + `sys_hz` let TIMELR /
    /// TIMERAWL / TIMERAWH compute the live value.
    pub fn read32(&mut self, offset: u32, master_cycle: u64, sys_hz: u32) -> u32 {
        let now = Self::cycles_to_us(master_cycle, sys_hz);
        match offset {
            TIMEHW_OFFSET | TIMELW_OFFSET => 0, // write-only; reads RAZ
            TIMEHR_OFFSET => self.timehr_latched,
            TIMELR_OFFSET => {
                // Latch high half so a subsequent TIMEHR read is
                // consistent with this snapshot.
                self.timehr_latched = (now >> 32) as u32;
                now as u32
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
            TIMERAWH_OFFSET => (now >> 32) as u32,
            TIMERAWL_OFFSET => now as u32,
            DBGPAUSE_OFFSET => self.dbgpause as u32,
            PAUSE_OFFSET => {
                if self.pause {
                    1
                } else {
                    0
                }
            }
            INTR_OFFSET => (self.intr & 0xF) as u32,
            INTE_OFFSET => (self.inte & 0xF) as u32,
            INTF_OFFSET => (self.intf & 0xF) as u32,
            INTS_OFFSET => ((self.intr | self.intf) & self.inte) as u32 & ALARM_MASK_4BITS,
            _ => 0,
        }
    }

    /// Write a TIMER register with an APB alias (normalised 2-bit form:
    /// 0 plain / 1 XOR / 2 BITSET / 3 BITCLR). `sys_hz` lets ALARM writes
    /// convert the target microsecond value to a fire-cycle.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32, master_cycle: u64, sys_hz: u32) {
        match offset {
            // Phase 1: software-poke of TIMER value is a no-op.
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
                // Arm + schedule.
                self.armed |= 1 << idx;
                let now = Self::cycles_to_us(master_cycle, sys_hz);
                // Compute fire-cycle: target microsecond is the low 32
                // bits of a future "now". If the target is in the past
                // relative to `now`, wrap to the next modular match
                // (`now + (target - now_lo)` into 32 bits). This matches
                // pico-sdk's 32-bit TIMER semantics for short delays.
                let target_us = stored as u64;
                let now_lo = (now as u32) as u64;
                let delta = target_us.wrapping_sub(now_lo) & 0xFFFF_FFFF;
                let fire_us = now.wrapping_add(delta);
                self.alarm_fire_cycle[idx] = Some(Self::us_to_cycles(fire_us, sys_hz));
            }
            ARMED_OFFSET => {
                // Writing 1 to an ARMED bit DISARMS the alarm.
                // (Datasheet §4.6.5 — inverse W1C.) We honour alias
                // semantics by first resolving to a 32-bit value then
                // using it as the disarm mask.
                let mut stored = self.armed as u32;
                apply_alias_rmw(&mut stored, value, alias);
                // Interpret the resulting stored value as a disarm mask:
                // every bit that is 1 disarms the matching alarm.
                let disarm = stored as u8 & 0xF;
                self.armed &= !disarm;
                for n in 0..4 {
                    if disarm & (1 << n) != 0 {
                        self.alarm_fire_cycle[n] = None;
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
            INTR_OFFSET => {
                // W1C regardless of alias — per datasheet. Still let the
                // alias shape the mask so BITSET on INTR isn't a
                // surprise no-op.
                let mut stored = self.intr as u32;
                apply_alias_rmw(&mut stored, value, alias);
                // After alias resolution, every bit that is 1 clears
                // the corresponding INTR pending bit.
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
}

impl Default for TimerRegs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Simulated 125 MHz sys_clk — pico-sdk's default runtime state.
    const SYS_HZ: u32 = 125_000_000;

    // --- Reset / defaults ------------------------------------------------

    #[test]
    fn reset_defaults_all_fields_zero() {
        let t = TimerRegs::new();
        assert_eq!(t.armed, 0);
        assert_eq!(t.intr, 0);
        assert_eq!(t.inte, 0);
        assert_eq!(t.intf, 0);
        assert!(!t.pause);
        assert_eq!(t.dbgpause, 0);
        assert!(t.alarm_fire_cycle.iter().all(|x| x.is_none()));
    }

    #[test]
    fn reset_clears_post_state() {
        let mut t = TimerRegs::new();
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        t.intr = 0xF;
        t.inte = 0xF;
        t.reset();
        assert_eq!(t.armed, 0);
        assert_eq!(t.intr, 0);
        assert_eq!(t.inte, 0);
    }

    // --- Time reads ------------------------------------------------------

    #[test]
    fn timerawl_returns_low_32_of_now_no_latch() {
        let mut t = TimerRegs::new();
        // 250 sysclks at 125 MHz = 2 µs.
        assert_eq!(t.read32(TIMERAWL_OFFSET, 250, SYS_HZ), 2);
        // TIMEHR is NOT latched by a TIMERAWL read.
        assert_eq!(t.read32(TIMEHR_OFFSET, 250, SYS_HZ), 0);
    }

    #[test]
    fn timerawh_returns_high_32_no_latch() {
        let mut t = TimerRegs::new();
        // To push into the high half we need ≥ 2^32 µs = 4.29 × 10^9 µs.
        // At 125 MHz: 4.29e9 × 125 = 5.36e11 cycles.
        let cycles = (5u64 << 32) * 125; // 5 × 2^32 sysclks as a u64 count
        let hi = t.read32(TIMERAWH_OFFSET, cycles, SYS_HZ);
        assert_eq!(hi, 5);
    }

    #[test]
    fn timelr_read_latches_timehr() {
        let mut t = TimerRegs::new();
        // Place 'now' at 5 µs of high-half significance, 7 µs low.
        let us: u64 = (5u64 << 32) | 7;
        let cycles = us * 125;
        let lo = t.read32(TIMELR_OFFSET, cycles, SYS_HZ);
        assert_eq!(lo, 7);
        // TIMEHR now returns the latched high half.
        assert_eq!(t.read32(TIMEHR_OFFSET, cycles, SYS_HZ), 5);
    }

    #[test]
    fn timehr_returns_latched_value_not_live() {
        // Read TIMELR at one instant, then advance master_cycle; TIMEHR
        // still returns the latched snapshot.
        let mut t = TimerRegs::new();
        let us1: u64 = (1u64 << 32) | 10;
        t.read32(TIMELR_OFFSET, us1 * 125, SYS_HZ);
        // Bump master_cycle so now > latched.
        let us2: u64 = (9u64 << 32) | 99;
        assert_eq!(
            t.read32(TIMEHR_OFFSET, us2 * 125, SYS_HZ),
            1,
            "TIMEHR must return the latched snapshot, not the live value"
        );
    }

    // --- Alarms + IRQ ----------------------------------------------------

    #[test]
    fn alarm_write_arms_and_schedules() {
        let mut t = TimerRegs::new();
        // Write ALARM0 = 100 µs.
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        assert_eq!(t.armed & 1, 1, "ALARM write must arm the alarm");
        assert_eq!(t.alarm_target_us[0], 100);
        assert_eq!(
            t.alarm_fire_cycle[0],
            Some(100 * 125),
            "fire cycle = 100 µs × 125 MHz"
        );
    }

    #[test]
    fn poll_alarms_fires_on_match() {
        let mut t = TimerRegs::new();
        // Write ALARM0 = 100 µs at master_cycle 0.
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        assert_eq!(t.poll_alarms(99 * 125, SYS_HZ), 0, "no fire before target");
        let nvic_bits = t.poll_alarms(100 * 125, SYS_HZ);
        assert_eq!(nvic_bits, 0, "INTE not set => no NVIC line raised");
        // But INTR bit is latched even without INTE.
        assert_eq!(t.intr & 1, 1, "INTR bit 0 must latch");
        // Armed bit must clear after fire.
        assert_eq!(t.armed & 1, 0, "alarm must auto-disarm after fire");
    }

    #[test]
    fn poll_alarms_raises_nvic_when_inte_set() {
        let mut t = TimerRegs::new();
        t.write32(INTE_OFFSET, 1, 0, 0, SYS_HZ);
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        let nvic_bits = t.poll_alarms(100 * 125, SYS_HZ);
        assert_eq!(nvic_bits & 1, 1, "INTE=1 routes alarm 0 to NVIC bit 0");
    }

    #[test]
    fn poll_alarms_re_asserts_latched_level_until_w1c() {
        // Level-IRQ re-assert: after an alarm fires and disarms,
        // INTR bit stays latched until firmware W1Cs it. While INTE
        // is set, every subsequent poll must re-raise the NVIC line
        // — the CPU clears the NVIC pending bit on dispatch, and
        // level sources are expected to keep raising until the ISR
        // clears INTR. Otherwise a tail-chained ISR would see no
        // pending and silently diverge from silicon.
        let mut t = TimerRegs::new();
        t.write32(INTE_OFFSET, 1, 0, 0, SYS_HZ);
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        // First poll at match: fires and latches INTR.
        let n1 = t.poll_alarms(100 * 125, SYS_HZ);
        assert_eq!(n1 & 1, 1, "first poll raises NVIC bit 0");
        assert_eq!(t.intr & 1, 1, "INTR bit 0 latched");
        assert_eq!(t.armed & 1, 0, "alarm auto-disarmed after fire");
        // Second poll with INTR still latched: MUST re-raise.
        let n2 = t.poll_alarms(101 * 125, SYS_HZ);
        assert_eq!(
            n2 & 1,
            1,
            "level re-assert: INTR latched + INTE set => NVIC bit 0 re-raised"
        );
        // After W1C of INTR, the re-assert stops.
        t.write32(INTR_OFFSET, 1, 0, 0, SYS_HZ);
        let n3 = t.poll_alarms(102 * 125, SYS_HZ);
        assert_eq!(
            n3 & 1,
            0,
            "after firmware W1Cs INTR, the level condition drops"
        );
    }

    #[test]
    fn intr_write_is_w1c() {
        let mut t = TimerRegs::new();
        // Fire alarm 0 to latch INTR bit 0.
        t.write32(ALARM0_OFFSET, 50, 0, 0, SYS_HZ);
        t.poll_alarms(50 * 125, SYS_HZ);
        assert_eq!(t.intr, 1);
        // Write 1 to clear.
        t.write32(INTR_OFFSET, 1, 0, 0, SYS_HZ);
        assert_eq!(t.intr, 0, "INTR must be W1C");
    }

    #[test]
    fn intr_write_zero_does_not_clear() {
        let mut t = TimerRegs::new();
        t.intr = 0xF;
        t.write32(INTR_OFFSET, 0, 0, 0, SYS_HZ);
        assert_eq!(t.intr, 0xF, "writing 0 to INTR must not clear any bit");
    }

    #[test]
    fn armed_write_disarms() {
        let mut t = TimerRegs::new();
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        assert_eq!(t.armed & 1, 1);
        // Writing 1 to ARMED bit 0 disarms alarm 0.
        t.write32(ARMED_OFFSET, 1, 0, 0, SYS_HZ);
        assert_eq!(t.armed & 1, 0);
        assert!(t.alarm_fire_cycle[0].is_none());
    }

    #[test]
    fn ints_reads_latched_and_inte_gated() {
        let mut t = TimerRegs::new();
        t.intr = 0x3; // alarms 0 + 1 pending
        t.inte = 0x1; // only alarm 0 enabled
        let v = t.read32(INTS_OFFSET, 0, SYS_HZ);
        assert_eq!(v, 0x1, "INTS = (intr | intf) & inte, alarm 1 masked");
    }

    #[test]
    fn intf_forces_ints_even_without_match() {
        let mut t = TimerRegs::new();
        t.inte = 0x4;
        t.intf = 0x4;
        let v = t.read32(INTS_OFFSET, 0, SYS_HZ);
        assert_eq!(v, 0x4);
    }

    #[test]
    fn inte_bitset_alias_works() {
        let mut t = TimerRegs::new();
        t.write32(INTE_OFFSET, 0x2, 2, 0, SYS_HZ); // BITSET alias
        assert_eq!(t.inte, 0x2);
        t.write32(INTE_OFFSET, 0x4, 2, 0, SYS_HZ);
        assert_eq!(t.inte, 0x6);
    }

    #[test]
    fn inte_bitclr_alias_works() {
        let mut t = TimerRegs::new();
        t.inte = 0xF;
        t.write32(INTE_OFFSET, 0x5, 3, 0, SYS_HZ); // BITCLR alias
        assert_eq!(t.inte, 0xA);
    }

    // --- is_idle ---------------------------------------------------------

    #[test]
    fn is_idle_false_when_intr_latched() {
        let mut t = TimerRegs::new();
        t.intr = 0x1;
        assert!(!t.is_idle());
    }

    #[test]
    fn is_idle_true_with_armed_but_no_pending() {
        let mut t = TimerRegs::new();
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        assert!(
            t.is_idle(),
            "alarms armed but not yet fired are fast-path idle"
        );
    }

    // --- Multiple alarms --------------------------------------------------

    #[test]
    fn two_alarms_independent_fire() {
        let mut t = TimerRegs::new();
        t.write32(ALARM0_OFFSET, 100, 0, 0, SYS_HZ);
        t.write32(ALARM0_OFFSET + 4, 200, 0, 0, SYS_HZ);
        // At 100 µs alarm 0 fires; alarm 1 still armed.
        t.poll_alarms(100 * 125, SYS_HZ);
        assert_eq!(t.intr, 0x1);
        assert_eq!(t.armed & 0x3, 0x2);
        // At 200 µs alarm 1 fires too.
        t.poll_alarms(200 * 125, SYS_HZ);
        assert_eq!(t.intr, 0x3);
        assert_eq!(t.armed & 0x3, 0);
    }
}

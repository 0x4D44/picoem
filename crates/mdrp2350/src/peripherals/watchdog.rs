//! RP2350 WATCHDOG peripheral — HLD V5 §7.D.3.
//!
//! # Register map (datasheet §4.7)
//!
//! | Offset | Name        | Access | Notes                              |
//! |--------|-------------|--------|------------------------------------|
//! | `0x00` | `CTRL`      | RW     | `ENABLE` (bit 30), `PAUSE_JTAG` (bit 24), `PAUSE_DBG0` (bit 25), `PAUSE_DBG1` (bit 26), `TRIGGER` (bit 31), `TIME[23:0]` countdown (RO mirror). |
//! | `0x04` | `LOAD`      | W      | Sets countdown. Reads as the last LOAD value (harmless). |
//! | `0x08` | `REASON`    | RO     | Force / timer reason bits. Stubbed; V5 does not decode. |
//! | `0x0C` | `SCRATCH0`  | RW     | Persistent across reset.           |
//! | `0x10` | `SCRATCH1`  | RW     |                                    |
//! | `0x14` | `SCRATCH2`  | RW     |                                    |
//! | `0x18` | `SCRATCH3`  | RW     |                                    |
//! | `0x1C` | `SCRATCH4`  | RW     |                                    |
//! | `0x20` | `SCRATCH5`  | RW     |                                    |
//! | `0x24` | `SCRATCH6`  | RW     |                                    |
//! | `0x28` | `SCRATCH7`  | RW     |                                    |
//!
//! There is **no `TICK` register** on RP2350 — that moved to the
//! TICKS peripheral. Firmware reaching for the RP2040 `WATCHDOG_TICK`
//! offset hits the Bus HashMap fallthrough (warn-once).
//!
//! # Countdown model
//!
//! Simplification: one [`WatchdogRegs::tick`] call advances the
//! countdown by one cycle. The Bus calls `tick()` once per
//! `Bus::tick_peripherals` invocation — i.e. once per quantum's
//! `sys_clks` advance. Good enough for the LOAD=100 fire-within-130-
//! cycles test; firmware that cares about precise countdown rate
//! programs TICKS explicitly and the warn-once will flag divergence.
//!
//! When `ENABLE == 1`, no `PAUSE_*` bit is set, and `TIME` reaches 0,
//! the peripheral latches a reset request via
//! [`crate::bus::Bus::set_watchdog_reset`]. Pausing: if **any** of
//! `PAUSE_JTAG` / `PAUSE_DBG0` / `PAUSE_DBG1` is set, the countdown
//! holds (the emulator has no debug state, so "pause bit set" == "hold").
//!
//! # Base address
//!
//! `WATCHDOG_BASE = 0x400D_8000` — pico-sdk RP2350 layout
//! (`hardware_regs/watchdog.h`). This differs from RP2040's
//! `0x4005_8000` (which is PLL_USB on RP2350). If silicon differs, the
//! warn-once at the Bus dispatch catches it.

use super::apply_alias_rmw;

/// WATCHDOG base. See module doc for the pico-sdk layout reference.
pub const WATCHDOG_BASE: u32 = 0x400D_8000;

const CTRL_OFFSET: u32 = 0x00;
const LOAD_OFFSET: u32 = 0x04;
const REASON_OFFSET: u32 = 0x08;
/// First SCRATCH offset. SCRATCH0..7 span `0x0C..=0x28`.
const SCRATCH0_OFFSET: u32 = 0x0C;
const SCRATCH_COUNT: usize = 8;

/// `CTRL.ENABLE` bit.
const CTRL_ENABLE_BIT: u32 = 1 << 30;
/// `CTRL.TRIGGER` (write-1 to force reset).
const CTRL_TRIGGER_BIT: u32 = 1 << 31;
/// Combined pause mask (`PAUSE_JTAG | PAUSE_DBG0 | PAUSE_DBG1`).
const CTRL_PAUSE_MASK: u32 = (1 << 24) | (1 << 25) | (1 << 26);
/// `TIME[23:0]` mirror field in CTRL (read-only).
const CTRL_TIME_MASK: u32 = 0x00FF_FFFF;

/// Mask of CTRL bits that firmware owns (everything except TIME[23:0],
/// which the peripheral overlays on read).
const CTRL_WRITABLE_MASK: u32 = !CTRL_TIME_MASK;

/// WATCHDOG register block.
pub struct WatchdogRegs {
    /// Stored CTRL bits — writable subset only (firmware can't set TIME).
    ctrl: u32,
    /// Current countdown value (24-bit).
    time: u32,
    /// Last LOAD value (for completeness; writing LOAD reloads `time`).
    load: u32,
    /// REASON register. Stubbed to 0 in V5.
    reason: u32,
    /// SCRATCH0..7 — persist across watchdog-reset per the datasheet.
    scratch: [u32; SCRATCH_COUNT],
}

impl WatchdogRegs {
    pub fn new() -> Self {
        Self {
            ctrl: 0,
            time: 0,
            load: 0,
            reason: 0,
            scratch: [0u32; SCRATCH_COUNT],
        }
    }

    /// Read a WATCHDOG register word.
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            CTRL_OFFSET => (self.ctrl & CTRL_WRITABLE_MASK) | (self.time & CTRL_TIME_MASK),
            LOAD_OFFSET => self.load,
            REASON_OFFSET => self.reason,
            _ if Self::is_scratch(offset) => {
                let idx = Self::scratch_index(offset);
                self.scratch[idx]
            }
            _ => 0,
        }
    }

    /// Write a WATCHDOG register word. Returns `true` if firmware
    /// requested an immediate reset via `CTRL.TRIGGER`.
    #[must_use]
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) -> bool {
        match offset {
            CTRL_OFFSET => {
                // Only writable bits land; TIME is peripheral-owned.
                let mut staged = self.ctrl;
                apply_alias_rmw(&mut staged, value & CTRL_WRITABLE_MASK, alias);
                // TRIGGER is write-1-to-fire, self-clearing semantically.
                let trigger = (staged & CTRL_TRIGGER_BIT) != 0;
                self.ctrl = staged & !CTRL_TRIGGER_BIT;
                trigger
            }
            LOAD_OFFSET => {
                // LOAD always plain-writes; reloads countdown into TIME.
                self.load = value & CTRL_TIME_MASK;
                self.time = self.load;
                false
            }
            REASON_OFFSET => {
                // RO in practice; accept alias-clear for firmware W1C if any.
                apply_alias_rmw(&mut self.reason, value, alias);
                false
            }
            _ if Self::is_scratch(offset) => {
                let idx = Self::scratch_index(offset);
                apply_alias_rmw(&mut self.scratch[idx], value, alias);
                false
            }
            _ => false,
        }
    }

    /// Advance the countdown by one cycle. Returns `true` when the
    /// countdown has just hit zero while `ENABLE` was set and no
    /// `PAUSE_*` bit is held — the caller (`Bus::tick_peripherals`)
    /// converts this into a [`crate::bus::Bus::set_watchdog_reset`] call.
    #[must_use]
    pub fn tick(&mut self) -> bool {
        if self.ctrl & CTRL_ENABLE_BIT == 0 {
            return false;
        }
        if self.ctrl & CTRL_PAUSE_MASK != 0 {
            return false;
        }
        if self.time == 0 {
            // Already fired / not loaded. No re-fire until reloaded.
            return false;
        }
        self.time = self.time.saturating_sub(1);
        self.time == 0
    }

    /// Clear the reset-firing state after the Bus has processed a
    /// watchdog-reset. Preserves SCRATCH0..7 (datasheet §4.7 — they
    /// survive a WDT reset). Clears countdown and `ENABLE` so the
    /// post-reset firmware sees a known-quiescent watchdog.
    pub fn post_reset(&mut self) {
        self.ctrl = 0;
        self.time = 0;
        self.load = 0;
        self.reason = 0;
        // scratch[] preserved intentionally.
    }

    #[inline]
    fn is_scratch(offset: u32) -> bool {
        offset >= SCRATCH0_OFFSET
            && offset < SCRATCH0_OFFSET + 4 * SCRATCH_COUNT as u32
            && (offset - SCRATCH0_OFFSET) % 4 == 0
    }

    #[inline]
    fn scratch_index(offset: u32) -> usize {
        ((offset - SCRATCH0_OFFSET) / 4) as usize
    }
}

impl Default for WatchdogRegs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_seeds_countdown() {
        let mut wd = WatchdogRegs::new();
        let _ = wd.write32(LOAD_OFFSET, 100, 0);
        // TIME visible in CTRL[23:0].
        let ctrl = wd.read32(CTRL_OFFSET);
        assert_eq!(ctrl & CTRL_TIME_MASK, 100);
    }

    #[test]
    fn disabled_watchdog_does_not_advance() {
        let mut wd = WatchdogRegs::new();
        let _ = wd.write32(LOAD_OFFSET, 100, 0);
        for _ in 0..200 {
            assert!(!wd.tick(), "fire should not occur while ENABLE=0");
        }
        assert_eq!(wd.read32(CTRL_OFFSET) & CTRL_TIME_MASK, 100);
    }

    #[test]
    fn enabled_countdown_fires_within_130_cycles_of_load_100() {
        let mut wd = WatchdogRegs::new();
        let _ = wd.write32(LOAD_OFFSET, 100, 0);
        let _ = wd.write32(CTRL_OFFSET, CTRL_ENABLE_BIT, 0);
        let mut fired_at = None;
        for i in 1..=130 {
            if wd.tick() {
                fired_at = Some(i);
                break;
            }
        }
        let fired_at = fired_at.expect("watchdog must fire within 130 cycles at LOAD=100");
        assert!(
            (100..=130).contains(&fired_at),
            "fired at cycle {fired_at}, expected 100..=130"
        );
    }

    #[test]
    fn pause_bit_halts_countdown() {
        let mut wd = WatchdogRegs::new();
        let _ = wd.write32(LOAD_OFFSET, 100, 0);
        let _ = wd.write32(CTRL_OFFSET, CTRL_ENABLE_BIT | (1 << 25), 0); // PAUSE_DBG0
        for _ in 0..200 {
            assert!(!wd.tick(), "fire should not occur while a PAUSE_* bit is set");
        }
        assert_eq!(wd.read32(CTRL_OFFSET) & CTRL_TIME_MASK, 100);
    }

    #[test]
    fn scratch_preserved_across_post_reset() {
        let mut wd = WatchdogRegs::new();
        for i in 0..SCRATCH_COUNT as u32 {
            let _ = wd.write32(SCRATCH0_OFFSET + 4 * i, 0x1000 + i, 0);
        }
        // Fire the watchdog.
        let _ = wd.write32(LOAD_OFFSET, 1, 0);
        let _ = wd.write32(CTRL_OFFSET, CTRL_ENABLE_BIT, 0);
        assert!(wd.tick());
        // Bus would clear flag + call post_reset.
        wd.post_reset();
        for i in 0..SCRATCH_COUNT as u32 {
            assert_eq!(
                wd.read32(SCRATCH0_OFFSET + 4 * i),
                0x1000 + i,
                "SCRATCH{} must survive watchdog reset",
                i
            );
        }
        // ENABLE cleared.
        assert_eq!(wd.read32(CTRL_OFFSET) & CTRL_ENABLE_BIT, 0);
    }

    #[test]
    fn trigger_write_fires_immediately() {
        let mut wd = WatchdogRegs::new();
        let fired = wd.write32(CTRL_OFFSET, CTRL_TRIGGER_BIT, 0);
        assert!(fired, "CTRL.TRIGGER write should request an immediate reset");
        // TRIGGER is self-clearing.
        assert_eq!(wd.read32(CTRL_OFFSET) & CTRL_TRIGGER_BIT, 0);
    }
}

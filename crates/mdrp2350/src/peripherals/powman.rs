//! RP2350 POWMAN peripheral — Coverage Gap Fill V11 §3.2.
//!
//! Models the POWMAN AON timer sufficient to drive `POWMAN_IRQ_TIMER`
//! (NVIC line 45) when a programmed alarm matches the running count.
//! Also retains the Stage 1 ARCHSEL fire-once tripwire (§10) and the
//! VREG / CTRL storage for firmware round-trip reads.
//!
//! # ARCHSEL tripwire — emulator-only
//!
//! The ARCHSEL tripwire is emulator-only (silicon POWMAN offset 0x20 is
//! not ARCHSEL — the real silicon register at that offset serves a
//! different purpose, so real firmware may write 0x20 for its
//! silicon-intended reason). Downgraded to trace-level to avoid noise
//! if firmware touches the real silicon register here. The tripwire
//! still fires once via the `warned_archsel` struct flag; unit tests
//! assert that behaviour directly rather than via log-level capture.
//! See HLD V11 §10 for the RISC-V track rationale (runtime
//! ARCHSEL-driven core selection is currently build-time via
//! `Cores::RiscV`, but the tripwire is kept for the day that changes).
//!
//! # Register map (pico-sdk pinned commit
//! `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779` — `powman.h`)
//!
//! | Offset  | Name                    | Access | Notes                    |
//! |---------|-------------------------|--------|--------------------------|
//! | `0x00`  | `BADPASSWD` (= `CTRL`*) | R/W    | Plain storage            |
//! | `0x04`  | `VREG_CTRL`             | R/W    | Plain storage            |
//! | `0x08`  | `VREG_STS`              | R/W    | Plain storage            |
//! | `0x0C`  | `VREG`                  | R/W    | Plain storage            |
//! | `0x60..0x6C` | `SET_TIME_*`      | W      | Writes seed 64-bit COUNT |
//! | `0x70`  | `READ_TIME_UPPER`       | R      | High 32 of running COUNT |
//! | `0x74`  | `READ_TIME_LOWER`       | R      | Low  32 of running COUNT |
//! | `0x78..0x84` | `ALARM_TIME_*`    | R/W    | 64-bit match target      |
//! | `0x88`  | `TIMER`                 | R/W    | RUN (bit 1), ALARM_ENAB  |
//! |         |                         |        | (bit 4), ALARM W1C (6)   |
//! | `0xE0`  | `INTR`                  | RO     | TIMER = bit 1            |
//! | `0xE4`  | `INTE`                  | R/W    | TIMER = bit 1            |
//! | `0xE8`  | `INTF`                  | R/W    | TIMER = bit 1            |
//! | `0xEC`  | `INTS`                  | RO     | `(INTR & INTE) | INTF`   |
//! | `0x20`  | `ARCHSEL` (emu-only)    | R/W    | Warn-once on non-Arm (§10) |
//! | other   | —                       | R/W    | HashMap fallthrough      |
//!
//! *Offset 0x00 is `BADPASSWD` on silicon but `CTRL` in the Stage 1
//! model; it is plain-storage for firmware round-trip and never observed
//! by the IRQ path. The `ctrl` field retains the Stage 1 name.
//!
//! # HLD §3.2 logical-name mapping
//!
//! The HLD uses logical names (`AON_COUNT_LO/HI`, `AON_MATCH_LO/HI`,
//! `MATCH_EN`); silicon does not. The mapping this module implements:
//!
//! - `AON_COUNT_LO/HI` → `READ_TIME_LOWER`/`UPPER` (RO). Firmware that
//!   previously wrote COUNT should use `SET_TIME_*` instead; for back-
//!   compat, writes to the old `AON_COUNT_LO/HI` offsets `0x08`/`0x0C`
//!   now fall through to `VREG_STS`/`VREG` (plain storage) — any
//!   firmware actually programming those offsets would have been
//!   non-functional on silicon anyway.
//! - `AON_MATCH_LO/HI` → `ALARM_TIME_15TO0`..`63TO48`. The HLD's
//!   "AON_MATCH_LO = 100" translates to "write 100 to offset 0x84"
//!   (since 100 fits in 16 bits, the low 16-bit register is sufficient).
//! - `MATCH_EN` → `TIMER.ALARM_ENAB` (bit 4 at offset 0x88).
//!
//! # COUNT advancement
//!
//! POWMAN's tick source on RP2354 is XOSC / 4 ≈ 3 MHz. At the post-
//! bootrom system clock of 150 MHz this gives **50 sys_clks per POWMAN
//! tick**. [`POWMAN_SYS_PER_TICK`] codifies this ratio; Stage 5
//! pre-flight (`smoke_powman_pacing_rp2350`) measures the real ratio
//! on silicon. The emulator recomputes the ratio from the live
//! [`ClockTree::sys_clk_hz`] on each `tick` call to stay correct when
//! firmware reprograms PLL_SYS.
//!
//! # Alarm semantics
//!
//! When [`advance`] observes `count >= alarm` **and** `TIMER.ALARM_ENAB`
//! is set, it:
//! 1. Sets `INTR.TIMER` (bit 1) and `TIMER.ALARM` (bit 6).
//! 2. Clears `TIMER.ALARM_ENAB` so the alarm is one-shot per HLD.
//! 3. Returns the NVIC raise mask for [`IRQ_POWMAN_IRQ_TIMER`].
//!
//! The `Bus::tick_peripherals` caller folds the mask into
//! `assert_irq_shared` via `raise_irqs_u64`. `POWMAN_IRQ_POW` (line
//! 44) is never driven by the emulator.

use std::collections::HashMap;
use tracing::{debug, trace};

use mdpicoem_common::clocks::{ClockTree, XOSC_FREQ_HZ};

use super::apply_alias_rmw;

/// POWMAN base address. Verified against pico-sdk `addressmap.h` at the
/// pinned commit: `POWMAN_BASE = 0x4010_0000`.
pub const POWMAN_BASE: u32 = 0x4010_0000;

/// POWMAN `BADPASSWD` / stage-1 CTRL storage slot.
pub const CTRL_OFFSET: u32 = 0x00;
/// POWMAN `VREG_CTRL`.
pub const VREG_CTRL_OFFSET: u32 = 0x04;
/// POWMAN `VREG_STS`.
pub const VREG_STS_OFFSET: u32 = 0x08;
/// POWMAN `VREG`.
pub const VREG_OFFSET: u32 = 0x0C;
/// POWMAN `ARCHSEL` — emulator-only warn-once tripwire (HLD §10).
pub const ARCHSEL_OFFSET: u32 = 0x20;

/// `SET_TIME_15TO0` — writes to `SET_TIME_*` seed the running COUNT.
pub const SET_TIME_15TO0_OFFSET: u32 = 0x6C;
/// `SET_TIME_31TO16`.
pub const SET_TIME_31TO16_OFFSET: u32 = 0x68;
/// `SET_TIME_47TO32`.
pub const SET_TIME_47TO32_OFFSET: u32 = 0x64;
/// `SET_TIME_63TO48`.
pub const SET_TIME_63TO48_OFFSET: u32 = 0x60;

/// `READ_TIME_UPPER` — high 32 of running COUNT.
pub const READ_TIME_UPPER_OFFSET: u32 = 0x70;
/// `READ_TIME_LOWER` — low 32 of running COUNT.
pub const READ_TIME_LOWER_OFFSET: u32 = 0x74;

/// `ALARM_TIME_63TO48`.
pub const ALARM_TIME_63TO48_OFFSET: u32 = 0x78;
/// `ALARM_TIME_47TO32`.
pub const ALARM_TIME_47TO32_OFFSET: u32 = 0x7C;
/// `ALARM_TIME_31TO16`.
pub const ALARM_TIME_31TO16_OFFSET: u32 = 0x80;
/// `ALARM_TIME_15TO0` — HLD §3.2 `AON_MATCH_LO`. Low 16 bits of the
/// 64-bit alarm target.
pub const ALARM_TIME_15TO0_OFFSET: u32 = 0x84;

/// `TIMER` control register. Carries ALARM_ENAB (bit 4), ALARM W1C
/// (bit 6), RUN (bit 1).
pub const TIMER_OFFSET: u32 = 0x88;
/// `TIMER.RUN` — bit 1. When clear, COUNT does not advance.
pub const TIMER_RUN_BIT: u32 = 1 << 1;
/// `TIMER.ALARM_ENAB` — bit 4. HLD §3.2 "MATCH_EN". One-shot: cleared
/// automatically when the alarm fires.
pub const TIMER_ALARM_ENAB_BIT: u32 = 1 << 4;
/// `TIMER.ALARM` — bit 6. W1C interrupt flag mirroring `INTR.TIMER`.
pub const TIMER_ALARM_BIT: u32 = 1 << 6;
/// `TIMER` register RW mask — bits writable by firmware. `TIMER.RUN`
/// and `TIMER.ALARM_ENAB` are RW; `TIMER.ALARM` is W1C; the
/// `TIMER.USE_*` SC bits are treated as plain storage since the
/// emulator does not distinguish clock sources.
pub const TIMER_RW_MASK: u32 = 0x000F_2777;

/// `INTR` — TIMER bit position (bit 1). Matches pico-sdk
/// `POWMAN_INTR_TIMER_BITS = 0x2`.
pub const INT_TIMER_BIT: u32 = 1 << 1;
/// `INTR` offset (RO latched interrupt status).
pub const INTR_OFFSET: u32 = 0xE0;
/// `INTE` offset (interrupt enable).
pub const INTE_OFFSET: u32 = 0xE4;
/// `INTF` offset (force interrupt).
pub const INTF_OFFSET: u32 = 0xE8;
/// `INTS` offset — `(INTR & INTE) | INTF`.
pub const INTS_OFFSET: u32 = 0xEC;

/// NVIC input line for the POWMAN TIMER IRQ. Verified against pico-sdk
/// `intctrl.h`: `POWMAN_IRQ_TIMER = 45`.
pub const IRQ_POWMAN_IRQ_TIMER: u32 = 45;

/// Arm default selection in [`ARCHSEL_OFFSET`]; non-Arm writes warn once
/// (HLD §10 RISC-V tripwire).
const ARCHSEL_ARM: u32 = 0;

/// Sys-clks per POWMAN tick at the post-bootrom default clock tree
/// (sys_clk = 150 MHz, XOSC = 12 MHz, POWMAN tick = XOSC/4 = 3 MHz,
/// 150e6 / 3e6 = 50). Recomputed live from [`ClockTree::sys_clk_hz`] if
/// firmware reprograms PLL_SYS — see [`PowmanRegs::advance`]. Exposed as
/// a `pub const` so the silicon scenario catalogue can size its
/// `max_sysclks` budget from the same number the emulator uses.
///
/// Stage 5 pre-flight (`smoke_powman_pacing_rp2350`) measures the real
/// ratio on silicon.
pub const POWMAN_SYS_PER_TICK: u64 = 50;

/// Default POWMAN tick frequency. XOSC / 4 with the pico-sdk default
/// 12 MHz XOSC: 12e6 / 4 = 3_000_000.
const POWMAN_TICK_HZ: u32 = XOSC_FREQ_HZ / 4;

/// POWMAN register block.
pub struct PowmanRegs {
    /// Plain-storage `BADPASSWD`/CTRL slot.
    ctrl: u32,
    /// `VREG_CTRL` storage.
    vreg_ctrl: u32,
    /// `VREG_STS` storage.
    vreg_sts: u32,
    /// `VREG` storage.
    vreg: u32,
    /// Emulator-only `ARCHSEL` (see module doc).
    archsel: u32,
    /// 64-bit running AON count — HLD §3.2 `AON_COUNT`.
    aon_count: u64,
    /// 64-bit alarm target — HLD §3.2 `AON_MATCH`.
    aon_match: u64,
    /// `TIMER` control register (RUN, ALARM_ENAB, ALARM, + plain-storage
    /// bits). `TIMER.ALARM_ENAB` is HLD §3.2 `MATCH_EN`.
    timer: u32,
    /// `INTE` — interrupt enable. Currently unused by the emulator IRQ
    /// path (we gate on `TIMER.ALARM_ENAB` directly, matching silicon's
    /// behaviour of also setting `INTR.TIMER` but routing NVIC via a
    /// separate path); kept here for firmware round-trip.
    inte: u32,
    /// `INTF` — force-interrupt. Plain storage; not routed to NVIC.
    intf: u32,
    /// `INTR` — latched interrupt status. `TIMER` bit set when alarm
    /// fires; W1C on write.
    intr: u32,
    /// Sub-tick accumulator: sys_clks that have arrived since the last
    /// COUNT increment. Resets modulo [`sys_per_tick`].
    sys_tick_accum: u64,
    /// HashMap fallthrough for offsets outside the modelled set.
    /// Round-trip only — no side effects.
    other: HashMap<u32, u32>,
    /// Warn-once latch: first `ARCHSEL` write changing value to non-Arm
    /// (HLD §10 RISC-V tripwire).
    warned_archsel: bool,
}

impl PowmanRegs {
    pub fn new() -> Self {
        Self {
            ctrl: 0,
            vreg_ctrl: 0,
            vreg_sts: 0,
            vreg: 0,
            archsel: ARCHSEL_ARM,
            aon_count: 0,
            aon_match: 0,
            timer: 0,
            inte: 0,
            intf: 0,
            intr: 0,
            sys_tick_accum: 0,
            other: HashMap::new(),
            warned_archsel: false,
        }
    }

    /// Read a POWMAN register word.
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            CTRL_OFFSET => self.ctrl,
            VREG_CTRL_OFFSET => self.vreg_ctrl,
            VREG_STS_OFFSET => self.vreg_sts,
            VREG_OFFSET => self.vreg,
            ARCHSEL_OFFSET => self.archsel,

            READ_TIME_LOWER_OFFSET => self.aon_count as u32,
            READ_TIME_UPPER_OFFSET => (self.aon_count >> 32) as u32,

            ALARM_TIME_15TO0_OFFSET => (self.aon_match & 0xFFFF) as u32,
            ALARM_TIME_31TO16_OFFSET => ((self.aon_match >> 16) & 0xFFFF) as u32,
            ALARM_TIME_47TO32_OFFSET => ((self.aon_match >> 32) & 0xFFFF) as u32,
            ALARM_TIME_63TO48_OFFSET => ((self.aon_match >> 48) & 0xFFFF) as u32,

            TIMER_OFFSET => self.timer,
            INTR_OFFSET => self.intr,
            INTE_OFFSET => self.inte,
            INTF_OFFSET => self.intf,
            INTS_OFFSET => (self.intr & self.inte) | self.intf,

            _ => *self.other.get(&offset).unwrap_or(&0),
        }
    }

    /// Write a POWMAN register word.
    ///
    /// `alias` is the APB alias selector (0 = RMW, 2 = SET, 3 = CLR, 1 =
    /// XOR) extracted by the caller via the standard 0x2000/0x3000 alias
    /// bits. All register paths use [`apply_alias_rmw`] so SET/CLR
    /// semantics match silicon.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        match offset {
            CTRL_OFFSET => apply_alias_rmw(&mut self.ctrl, value, alias),
            VREG_CTRL_OFFSET => apply_alias_rmw(&mut self.vreg_ctrl, value, alias),
            VREG_STS_OFFSET => apply_alias_rmw(&mut self.vreg_sts, value, alias),
            VREG_OFFSET => apply_alias_rmw(&mut self.vreg, value, alias),

            ARCHSEL_OFFSET => {
                apply_alias_rmw(&mut self.archsel, value, alias);
                if self.archsel != ARCHSEL_ARM && !self.warned_archsel {
                    self.warned_archsel = true;
                    // Trace-level, not warn: silicon has no ARCHSEL at
                    // offset 0x20, so real firmware hitting this path
                    // does so for whatever the real silicon register at
                    // 0x20 is, not for RISC-V selection. The tripwire
                    // survives as the `warned_archsel` struct flag
                    // (exercised by unit tests) and as a trace event
                    // visible under `RUST_LOG=trace`.
                    trace!(
                        archsel = format_args!("{:#X}", self.archsel),
                        "POWMAN ARCHSEL tripwire (emulator-only) — non-Arm value written"
                    );
                }
            }

            // SET_TIME_* — writing any SET_TIME_* lane seeds the
            // corresponding 16 bits of COUNT. The full silicon protocol
            // requires all four lanes to be written in sequence; we
            // accept partial writes for test convenience.
            //
            // Password strip: silicon requires the `0x5AFE` password in
            // bits [31:16] on every password-gated POWMAN write (per
            // pico-sdk `powman.h` commit a1438dff); wrong-password
            // writes are dropped and BADPASSWD latches. We don't
            // enforce the password (no BADPASSWD latch) but we do mask
            // bits [31:16] off on store — matching the silicon-visible
            // state where stored values are 16-bit and password-
            // prefixed firmware writes round-trip as their low 16 bits.
            // Since each SET_TIME_* / ALARM_TIME_* lane is already a
            // 16-bit field, `value as u16` (= `value & 0xFFFF`) applies
            // the strip implicitly below.
            SET_TIME_15TO0_OFFSET => {
                let v = (value as u64) & 0xFFFF;
                self.aon_count = (self.aon_count & !0xFFFF) | v;
            }
            SET_TIME_31TO16_OFFSET => {
                let v = ((value as u64) & 0xFFFF) << 16;
                self.aon_count = (self.aon_count & !(0xFFFF << 16)) | v;
            }
            SET_TIME_47TO32_OFFSET => {
                let v = ((value as u64) & 0xFFFF) << 32;
                self.aon_count = (self.aon_count & !(0xFFFFu64 << 32)) | v;
            }
            SET_TIME_63TO48_OFFSET => {
                let v = ((value as u64) & 0xFFFF) << 48;
                self.aon_count = (self.aon_count & !(0xFFFFu64 << 48)) | v;
            }

            ALARM_TIME_15TO0_OFFSET => {
                let v = (value as u64) & 0xFFFF;
                self.aon_match = (self.aon_match & !0xFFFF) | v;
            }
            ALARM_TIME_31TO16_OFFSET => {
                let v = ((value as u64) & 0xFFFF) << 16;
                self.aon_match = (self.aon_match & !(0xFFFF << 16)) | v;
            }
            ALARM_TIME_47TO32_OFFSET => {
                let v = ((value as u64) & 0xFFFF) << 32;
                self.aon_match = (self.aon_match & !(0xFFFFu64 << 32)) | v;
            }
            ALARM_TIME_63TO48_OFFSET => {
                let v = ((value as u64) & 0xFFFF) << 48;
                self.aon_match = (self.aon_match & !(0xFFFFu64 << 48)) | v;
            }

            TIMER_OFFSET => {
                // Strip the upper 16 bits (POWMAN password — see the
                // SET_TIME_* comment above) *before* applying the RW
                // mask. TIMER_RW_MASK covers bits [19:0], so a raw AND
                // against `value` alone would leak the password nibble
                // at [19:16] through. Masking to low 16 bits first
                // discards the password, then `TIMER_RW_MASK` restricts
                // storage to RW fields. `TIMER.ALARM` (bit 6) is W1C,
                // handled explicitly below.
                let masked_in = (value & 0xFFFF) & TIMER_RW_MASK;
                // Pre-compute the new value according to the alias,
                // then apply W1C for ALARM.
                let old = self.timer;
                let new = match alias & 0x3 {
                    0 => masked_in,
                    1 => old ^ masked_in,
                    2 => old | masked_in,
                    3 => old & !masked_in,
                    _ => masked_in,
                };
                // `ALARM` bit is W1C: a set bit in the write clears it.
                let alarm_clear = if (value & TIMER_ALARM_BIT) != 0 {
                    TIMER_ALARM_BIT
                } else {
                    0
                };
                // Preserve old ALARM state unless W1C clears it.
                let alarm_bit = (old & TIMER_ALARM_BIT) & !alarm_clear;
                self.timer = (new & !TIMER_ALARM_BIT) | alarm_bit;
                // INTR.TIMER follows TIMER.ALARM: if firmware W1C'd
                // ALARM, clear INTR.TIMER too.
                if alarm_clear != 0 {
                    self.intr &= !INT_TIMER_BIT;
                }
            }

            INTR_OFFSET => {
                // INTR is W1C — bits set in the write clear in storage.
                // Ignore `alias` (firmware uses raw INTR writes per
                // pico-sdk's hw_clear_bits pattern).
                self.intr &= !value;
                // Mirror the W1C on TIMER.ALARM.
                if (value & INT_TIMER_BIT) != 0 {
                    self.timer &= !TIMER_ALARM_BIT;
                }
            }
            INTE_OFFSET => apply_alias_rmw(&mut self.inte, value, alias),
            INTF_OFFSET => apply_alias_rmw(&mut self.intf, value, alias),
            INTS_OFFSET => {
                // Read-only on silicon — ignore writes.
            }

            _ => {
                let stored = self.other.entry(offset).or_insert(0);
                apply_alias_rmw(stored, value, alias);
            }
        }
    }

    /// Advance AON COUNT by `sys_clks` sys-clocks and, if the alarm
    /// fires, return the NVIC raise mask. Caller folds the mask into
    /// [`Bus::raise_irqs_u64`].
    ///
    /// No-op when `TIMER.RUN` is clear. The sys-per-tick divisor is
    /// derived from the current [`ClockTree::sys_clk_hz`] so firmware
    /// that reprograms PLL_SYS keeps a correct POWMAN cadence.
    pub fn advance(&mut self, sys_clks: u32, clock_tree: &ClockTree) -> u64 {
        if (self.timer & TIMER_RUN_BIT) == 0 || sys_clks == 0 {
            return 0;
        }
        // Note: `TIMER.RUN` 0→1 transitions do NOT clear
        // `sys_tick_accum`. This is a minor divergence from silicon —
        // real POWMAN resets its sub-tick phase when RUN asserts, so a
        // quick stop/start that straddles a half-tick can skew the next
        // tick by up to (sys_per_tick - 1) sys_clks vs silicon. No
        // known scenario observes this; if a future scenario diverges
        // on the first tick after a RUN re-assertion, zero
        // `sys_tick_accum` on the 0→1 transition in `TIMER_OFFSET`'s
        // write path.

        let sys_per_tick = sys_per_tick(clock_tree);
        if sys_per_tick == 0 {
            // Pathological: sys_clk slower than 1 Hz. Bail out rather
            // than divide by zero.
            return 0;
        }

        self.sys_tick_accum += sys_clks as u64;
        let ticks = self.sys_tick_accum / sys_per_tick;
        self.sys_tick_accum %= sys_per_tick;
        if ticks == 0 {
            return 0;
        }
        self.aon_count = self.aon_count.saturating_add(ticks);

        // Alarm check — fire iff count has reached match AND
        // ALARM_ENAB is asserted. One-shot: clear ENAB on fire.
        //
        // `>=` (rather than `==`) tolerates the case where a single
        // batch of sys_clks bumps COUNT past MATCH without stopping on
        // the exact value — e.g. a slow test steps 100 POWMAN ticks in
        // one call when MATCH = 50. The one-shot semantics below
        // (`timer &= !ALARM_ENAB`) prevent re-fire on subsequent
        // advances; `alarm_fires_and_is_one_shot` asserts that contract.
        if (self.timer & TIMER_ALARM_ENAB_BIT) != 0 && self.aon_count >= self.aon_match {
            debug!(
                count = self.aon_count,
                alarm = self.aon_match,
                "POWMAN alarm fired"
            );
            self.timer &= !TIMER_ALARM_ENAB_BIT;
            self.timer |= TIMER_ALARM_BIT;
            self.intr |= INT_TIMER_BIT;
            return 1u64 << IRQ_POWMAN_IRQ_TIMER;
        }

        0
    }

    /// Reset to post-power-on state. Called from [`crate::Emulator::reset`]
    /// to quiesce COUNT/MATCH/TIMER/INTR on warm reset. Mirrors the
    /// Stage 3 GLITCH_DETECTOR reset pattern.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for PowmanRegs {
    fn default() -> Self {
        Self::new()
    }
}

/// Sys-clocks per POWMAN tick, derived from the live clock tree. Returns
/// [`POWMAN_SYS_PER_TICK`] for the default configuration (sys_clk =
/// 150 MHz, POWMAN tick = XOSC/4 = 3 MHz).
fn sys_per_tick(clock_tree: &ClockTree) -> u64 {
    let sys_hz = clock_tree.sys_clk_hz as u64;
    let tick_hz = POWMAN_TICK_HZ as u64;
    if tick_hz == 0 {
        0
    } else {
        (sys_hz / tick_hz).max(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use tracing::span::{Attributes, Id, Record};
    use tracing::{Event, Metadata, Subscriber};

    // --- shared capture subscriber -----------------------------------

    #[derive(Default)]
    struct CaptureSubscriber {
        events: Arc<Mutex<Vec<String>>>,
    }

    struct FieldRecorder(String);
    impl tracing::field::Visit for FieldRecorder {
        fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
            use std::fmt::Write;
            if !self.0.is_empty() {
                self.0.push(' ');
            }
            let _ = write!(self.0, "{}={:?}", field.name(), value);
        }
    }

    impl Subscriber for CaptureSubscriber {
        fn enabled(&self, _metadata: &Metadata<'_>) -> bool {
            true
        }
        fn new_span(&self, _span: &Attributes<'_>) -> Id {
            Id::from_u64(1)
        }
        fn record(&self, _span: &Id, _values: &Record<'_>) {}
        fn record_follows_from(&self, _span: &Id, _follows: &Id) {}
        fn event(&self, event: &Event<'_>) {
            let mut v = FieldRecorder(String::new());
            event.record(&mut v);
            let meta = event.metadata();
            let line = format!("{} {} {}", meta.level(), meta.target(), v.0);
            self.events.lock().unwrap().push(line);
        }
        fn enter(&self, _span: &Id) {}
        fn exit(&self, _span: &Id) {}
    }

    /// Count events at any level whose body contains `needle`. The
    /// ARCHSEL tripwire was previously a warn-level event but is now
    /// trace-level (silicon has no ARCHSEL register — see module doc);
    /// callers assert on the *count* of tripwire events, not the level.
    fn count_events_containing(events: &[String], needle: &str) -> usize {
        events.iter().filter(|line| line.contains(needle)).count()
    }

    #[test]
    fn ctrl_roundtrip() {
        let mut p = PowmanRegs::new();
        p.write32(CTRL_OFFSET, 0xDEAD_BEEF, 0);
        assert_eq!(p.read32(CTRL_OFFSET), 0xDEAD_BEEF);
    }

    #[test]
    fn vreg_roundtrip() {
        let mut p = PowmanRegs::new();
        p.write32(VREG_CTRL_OFFSET, 0xA5A5_A5A5, 0);
        assert_eq!(p.read32(VREG_CTRL_OFFSET), 0xA5A5_A5A5);
        p.write32(VREG_OFFSET, 0x12_3456, 0);
        assert_eq!(p.read32(VREG_OFFSET), 0x12_3456);
    }

    #[test]
    fn count_does_not_advance_without_run() {
        let mut p = PowmanRegs::new();
        let tree = ClockTree::default();
        assert_eq!(p.advance(10_000, &tree), 0);
        assert_eq!(p.read32(READ_TIME_LOWER_OFFSET), 0);
    }

    #[test]
    fn count_advances_one_tick_per_fifty_sys_clks_at_default_clocks() {
        let mut p = PowmanRegs::new();
        let mut tree = ClockTree::default();
        // Force sys_clk = 150 MHz so sys_per_tick = 50.
        tree.sys_clk_hz = 150_000_000;
        p.write32(TIMER_OFFSET, TIMER_RUN_BIT, 0);
        // Exactly 50 sys_clks => 1 POWMAN tick.
        let _ = p.advance(50, &tree);
        assert_eq!(p.read32(READ_TIME_LOWER_OFFSET), 1);
        // Another 49 => still 1.
        let _ = p.advance(49, &tree);
        assert_eq!(p.read32(READ_TIME_LOWER_OFFSET), 1);
        // One more sys_clk => 2 ticks total.
        let _ = p.advance(1, &tree);
        assert_eq!(p.read32(READ_TIME_LOWER_OFFSET), 2);
    }

    #[test]
    fn alarm_fires_and_is_one_shot() {
        let mut p = PowmanRegs::new();
        let mut tree = ClockTree::default();
        tree.sys_clk_hz = 150_000_000;
        p.write32(ALARM_TIME_15TO0_OFFSET, 2, 0);
        p.write32(TIMER_OFFSET, TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT, 0);
        // 100 sys_clks = 2 POWMAN ticks = count reaches 2.
        let mask = p.advance(100, &tree);
        assert_eq!(mask, 1u64 << IRQ_POWMAN_IRQ_TIMER);
        // ALARM_ENAB cleared after fire.
        assert_eq!(p.read32(TIMER_OFFSET) & TIMER_ALARM_ENAB_BIT, 0);
        // ALARM bit (W1C) set.
        assert_eq!(p.read32(TIMER_OFFSET) & TIMER_ALARM_BIT, TIMER_ALARM_BIT);
        // INTR.TIMER set.
        assert_eq!(p.read32(INTR_OFFSET) & INT_TIMER_BIT, INT_TIMER_BIT);
        // Subsequent advance does not re-fire.
        let mask = p.advance(1000, &tree);
        assert_eq!(mask, 0);
    }

    #[test]
    fn alarm_w1c_clears_status() {
        let mut p = PowmanRegs::new();
        let mut tree = ClockTree::default();
        tree.sys_clk_hz = 150_000_000;
        p.write32(ALARM_TIME_15TO0_OFFSET, 1, 0);
        p.write32(TIMER_OFFSET, TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT, 0);
        let _ = p.advance(50, &tree);
        assert_ne!(p.read32(INTR_OFFSET) & INT_TIMER_BIT, 0);
        // W1C via TIMER.ALARM
        p.write32(TIMER_OFFSET, TIMER_ALARM_BIT, 0);
        assert_eq!(p.read32(TIMER_OFFSET) & TIMER_ALARM_BIT, 0);
        assert_eq!(p.read32(INTR_OFFSET) & INT_TIMER_BIT, 0);
    }

    #[test]
    fn archsel_arm_default_and_no_tripwire_on_arm_write() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber { events: captured.clone() };
        tracing::subscriber::with_default(subscriber, || {
            let mut p = PowmanRegs::new();
            assert_eq!(p.read32(ARCHSEL_OFFSET), ARCHSEL_ARM);
            p.write32(ARCHSEL_OFFSET, ARCHSEL_ARM, 0);
            assert!(
                !p.warned_archsel,
                "writing Arm default must not fire the tripwire"
            );
        });
        let events = captured.lock().unwrap();
        assert_eq!(
            count_events_containing(&events, "ARCHSEL"),
            0,
            "writing Arm default must not emit any ARCHSEL events; got {:?}",
            *events
        );
    }

    /// Tripwire fires exactly once on the first non-Arm ARCHSEL write.
    /// Renamed from `archsel_non_arm_warns_once` — the event is now
    /// trace-level (silicon has no ARCHSEL register; see module doc),
    /// so "warn" no longer matches the level. The behaviour is still a
    /// one-shot tripwire latched by the `warned_archsel` struct flag.
    ///
    /// Tests only the struct flag (not the emitted `trace!` event) —
    /// `trace!` is compiled out in `--release` by the workspace's
    /// `release_max_level_info` setting, so a capture-based assertion
    /// would green in debug and red in release.
    #[test]
    fn powman_archsel_non_arm_write_fires_tripwire_once() {
        let mut p = PowmanRegs::new();
        assert!(!p.warned_archsel, "tripwire must start latched-low");
        p.write32(ARCHSEL_OFFSET, 1, 0);
        assert!(
            p.warned_archsel,
            "first non-Arm write must latch the tripwire"
        );
        p.write32(ARCHSEL_OFFSET, 2, 0);
        assert_eq!(p.read32(ARCHSEL_OFFSET), 2);
        assert!(
            p.warned_archsel,
            "tripwire stays latched on subsequent non-Arm writes"
        );
    }

    #[test]
    fn unknown_offset_roundtrip() {
        let mut p = PowmanRegs::new();
        p.write32(0xF00, 0x1234_5678, 0);
        assert_eq!(p.read32(0xF00), 0x1234_5678);
        assert_eq!(p.read32(0xF04), 0);
    }
}

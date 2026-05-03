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
//! | Offset  | Name                  | Access | Notes                      |
//! |---------|-----------------------|--------|----------------------------|
//! | `0x00`  | `BADPASSWD`           | R/W1C  | Bit 0 sticky latch (§V13)  |
//! | `0x04`  | `VREG_CTRL`           | R/W    | Plain storage              |
//! | `0x08`  | `VREG_STS`            | R/W    | Plain storage              |
//! | `0x0C`  | `VREG`                | R/W    | Plain storage              |
//! | `0x60..0x6C` | `SET_TIME_*`    | W      | Writes seed 64-bit COUNT   |
//! | `0x70`  | `READ_TIME_UPPER`     | R      | High 32 of running COUNT   |
//! | `0x74`  | `READ_TIME_LOWER`     | R      | Low  32 of running COUNT   |
//! | `0x78..0x84` | `ALARM_TIME_*`  | R/W    | 64-bit match target        |
//! | `0x88`  | `TIMER`               | R/W    | RUN (bit 1), ALARM_ENAB    |
//! |         |                       |        | (bit 4), ALARM W1C (6)     |
//! | `0xE0`  | `INTR`                | RO     | TIMER = bit 1              |
//! | `0xE4`  | `INTE`                | R/W    | TIMER = bit 1              |
//! | `0xE8`  | `INTF`                | R/W    | TIMER = bit 1              |
//! | `0xEC`  | `INTS`                | RO     | `(INTR & INTE) | INTF`     |
//! | `0x20`  | `ARCHSEL` (emu-only)  | R/W    | Warn-once on non-Arm (§10) |
//! | other   | —                     | R/W    | HashMap fallthrough        |
//!
//! # Password protection (V13 Stage 1)
//!
//! Password-gated offsets require bits [31:16] == `0x5AFE` on every
//! write. Currently gated: `SET_TIME_*`, `ALARM_TIME_*`, `TIMER`,
//! `INTE`, `INTF`. A wrong-password write is dropped (no target
//! register mutation) and latches `BADPASSWD` bit 0 at offset 0x00.
//! `BADPASSWD` is itself not password-gated: any write with bit 0 set
//! clears the latch (W1C), any other write is inert. V13 scope does
//! not gate `VREG_CTRL`/`VREG_STS`/`VREG`/`ARCHSEL` — see `tech_debt.md`
//! for the broader password-audit follow-up.
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
//! 1. Sets `INTR.TIMER` (bit 1) and `TIMER.ALARM` (bit 6) **un-
//!    conditionally** — these are silicon's latch behaviour regardless
//!    of `INTE`.
//! 2. Clears `TIMER.ALARM_ENAB` so the alarm is one-shot per HLD.
//! 3. Returns the NVIC raise mask for [`IRQ_POWMAN_IRQ_TIMER`] **only
//!    if** `INTE.TIMER` is also set. Silicon gates the NVIC line on
//!    `(INTR & INTE) | INTF` (the `INTS` view); without `INTE.TIMER`
//!    the alarm latches in `INTR` but the NVIC line never asserts.
//!
//! Because `INTS` is level-sensitive on silicon, firmware that sets
//! `INTE.TIMER` *after* `INTR.TIMER` has latched must see the NVIC
//! line re-assert. [`PowmanRegs::write32`] returns the same NVIC raise
//! mask when a write to `INTE` (or `INTF`) transitions
//! `(intr & inte) | intf` from 0 to non-zero on the TIMER bit; the
//! `Bus::tick_peripherals` caller folds the mask into
//! `assert_irq_shared` via `raise_irqs_u64`. `POWMAN_IRQ_POW` (line
//! 44) is never driven by the emulator.

use std::collections::HashMap;
use tracing::{debug, trace};

use picoem_common::clocks::{ClockTree, XOSC_FREQ_HZ};

use super::apply_alias_rmw;

/// POWMAN base address. Verified against pico-sdk `addressmap.h` at the
/// pinned commit: `POWMAN_BASE = 0x4010_0000`.
pub const POWMAN_BASE: u32 = 0x4010_0000;

/// POWMAN `BADPASSWD` status/W1C slot (V13 Stage 1). Bit 0 latches on
/// any wrong-password write to a password-gated offset. Writing 1 to
/// bit 0 clears the latch. V11 aliased this offset to a plain-storage
/// `CTRL` slot; V13 replaces that with the silicon-correct BADPASSWD
/// semantics. The constant name is retained for call-site stability.
pub const CTRL_OFFSET: u32 = 0x00;
/// Alias for [`CTRL_OFFSET`] — the silicon name of offset 0x00.
pub const BADPASSWD_OFFSET: u32 = CTRL_OFFSET;
/// Sticky bit set on wrong-password writes; W1C to clear.
pub const BADPASSWD_BIT: u32 = 1 << 0;

/// Password value required in bits [31:16] of every password-gated
/// write. Silicon drops the target mutation on mismatch and latches
/// [`BADPASSWD_BIT`] at [`BADPASSWD_OFFSET`].
pub const POWMAN_PASSWORD: u32 = 0x5AFE_0000;
/// Mask covering the password field.
pub const POWMAN_PASSWORD_MASK: u32 = 0xFFFF_0000;
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
    /// `BADPASSWD` status/W1C latch at offset 0x00 (V13 Stage 1). Bit 0
    /// set on any wrong-password write to a password-gated offset.
    badpasswd: u32,
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
    /// `INTE` — interrupt enable. Gates the NVIC raise: [`PowmanRegs::
    /// advance`] sets `INTR.TIMER` unconditionally on alarm match but
    /// only returns the NVIC raise mask if `INTE.TIMER` is also set.
    /// Writes via [`PowmanRegs::write32`] also re-pend NVIC if they
    /// transition `(intr & inte) | intf` from 0 → 1 on the TIMER bit
    /// (level-sensitive `INTS` semantics). See module doc § "Alarm
    /// semantics".
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
            badpasswd: 0,
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
            CTRL_OFFSET => self.badpasswd,
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
    ///
    /// Returns a NVIC raise mask (`1u64 << IRQ_POWMAN_IRQ_TIMER`) when a
    /// write transitions the level-sensitive `INTS` view of the TIMER
    /// bit from 0 → 1 — i.e. enabling `INTE.TIMER` while `INTR.TIMER`
    /// is already latched, or setting `INTF.TIMER`. Returns 0 for all
    /// other writes. Caller folds the mask into
    /// [`crate::bus::Bus::raise_irqs_u64`].
    #[must_use]
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) -> u64 {
        // V13 Stage 1 — password check. Password-gated offsets drop the
        // target write on mismatch and latch BADPASSWD. Handled up-front
        // so each storage branch below can assume either (a) this offset
        // is not password-gated, or (b) the password is correct.
        if is_password_gated(offset) && (value & POWMAN_PASSWORD_MASK) != POWMAN_PASSWORD {
            self.badpasswd |= BADPASSWD_BIT;
            trace!(
                offset = format_args!("{:#X}", offset),
                value = format_args!("{:#X}", value),
                "POWMAN BADPASSWD latch — wrong-password write"
            );
            return 0;
        }

        // Snapshot the pre-write INTS.TIMER state so we can detect a
        // 0 → 1 transition for INTE / INTF writes.
        let pre_ints_timer = self.ints_timer_asserted();

        match offset {
            // BADPASSWD W1C — any set bit in bit 0 clears the latch.
            // Not password-gated. Bits [31:1] are ignored on write
            // (register is 1 bit wide on silicon).
            CTRL_OFFSET => {
                self.badpasswd &= !(value & BADPASSWD_BIT);
            }
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
            // INTE/INTF are password-gated like TIMER and SET_TIME_*: silicon
            // drops the upper-16 0x5AFE before storing, so a firmware read
            // observes only the low-16. Strip here so emulator round-trip
            // reads match silicon. Defined fields are bit 1 (TIMER) only;
            // upper-16 storage would otherwise confuse `INTS` reads via a
            // password write side-channel.
            INTE_OFFSET => {
                let stripped = value & 0xFFFF;
                apply_alias_rmw(&mut self.inte, stripped, alias);
            }
            INTF_OFFSET => {
                let stripped = value & 0xFFFF;
                apply_alias_rmw(&mut self.intf, stripped, alias);
            }
            INTS_OFFSET => {
                // Read-only on silicon — ignore writes.
            }

            _ => {
                let stored = self.other.entry(offset).or_insert(0);
                apply_alias_rmw(stored, value, alias);
            }
        }

        // Level-sensitive INTS: any write that transitions the TIMER
        // bit of `(INTR & INTE) | INTF` from 0 → 1 must (re-)assert
        // NVIC line 45. In practice this catches:
        //   * `INTE.TIMER` set while `INTR.TIMER` is already latched.
        //   * `INTF.TIMER` directly forcing the line.
        // Writes that lower INTS (INTR W1C, INTE clear) do not return
        // a mask — NVIC pending bits stick until the handler runs.
        let post_ints_timer = self.ints_timer_asserted();
        if post_ints_timer && !pre_ints_timer {
            1u64 << IRQ_POWMAN_IRQ_TIMER
        } else {
            0
        }
    }

    /// True iff the TIMER bit of the level-sensitive `INTS` view —
    /// `(INTR & INTE) | INTF` — is currently asserted. Helper for
    /// detecting write-induced transitions in [`PowmanRegs::write32`].
    #[inline]
    fn ints_timer_asserted(&self) -> bool {
        ((self.intr & self.inte) | self.intf) & INT_TIMER_BIT != 0
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
            // INTR.TIMER and TIMER.ALARM latch unconditionally —
            // silicon raises these regardless of INTE. ALARM_ENAB
            // self-clears (one-shot per HLD §3.2).
            self.timer &= !TIMER_ALARM_ENAB_BIT;
            self.timer |= TIMER_ALARM_BIT;
            self.intr |= INT_TIMER_BIT;
            // Only raise NVIC if INTE.TIMER is set. Without INTE, the
            // event latches in INTR but the NVIC line stays low —
            // matching silicon's `INTS = (INTR & INTE) | INTF` gating
            // (V11 Stage 6 silicon finding; V12 §3.2).
            if (self.inte & INT_TIMER_BIT) != 0 {
                return 1u64 << IRQ_POWMAN_IRQ_TIMER;
            }
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

/// Offsets requiring the POWMAN password in bits [31:16] (V13 Stage 1).
/// Writes with a wrong password to these offsets are silently dropped
/// and latch [`BADPASSWD_BIT`]. V13 scope is conservative — only the
/// offsets previously masking off their upper 16 bits are gated. A
/// future stage can widen this to VREG / ARCHSEL once silicon confirms.
#[inline]
fn is_password_gated(offset: u32) -> bool {
    matches!(
        offset,
        SET_TIME_15TO0_OFFSET
            | SET_TIME_31TO16_OFFSET
            | SET_TIME_47TO32_OFFSET
            | SET_TIME_63TO48_OFFSET
            | ALARM_TIME_15TO0_OFFSET
            | ALARM_TIME_31TO16_OFFSET
            | ALARM_TIME_47TO32_OFFSET
            | ALARM_TIME_63TO48_OFFSET
            | TIMER_OFFSET
            | INTE_OFFSET
            | INTF_OFFSET
    )
}

/// Sys-clocks per POWMAN tick, derived from the live clock tree. Returns
/// [`POWMAN_SYS_PER_TICK`] for the default configuration (sys_clk =
/// 150 MHz, POWMAN tick = XOSC/4 = 3 MHz).
fn sys_per_tick(clock_tree: &ClockTree) -> u64 {
    let sys_hz = clock_tree.sys_clk_hz as u64;
    let tick_hz = POWMAN_TICK_HZ as u64;
    sys_hz.checked_div(tick_hz).map(|q| q.max(1)).unwrap_or(0)
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

    /// Helper: arm POWMAN with `INTE.TIMER` enabled so `advance` will
    /// raise NVIC line 45 on alarm match. Most tests want this; the
    /// INTE-gating tests below intentionally skip it. V13 Stage 1 —
    /// password required on every gated write.
    fn arm_for_nvic_raise(p: &mut PowmanRegs, alarm_low16: u32) {
        let _ = p.write32(ALARM_TIME_15TO0_OFFSET, POWMAN_PASSWORD | alarm_low16, 0);
        let _ = p.write32(INTE_OFFSET, POWMAN_PASSWORD | INT_TIMER_BIT, 0);
        let _ = p.write32(
            TIMER_OFFSET,
            POWMAN_PASSWORD | TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
            0,
        );
    }

    #[test]
    fn vreg_roundtrip() {
        let mut p = PowmanRegs::new();
        let _ = p.write32(VREG_CTRL_OFFSET, 0xA5A5_A5A5, 0);
        assert_eq!(p.read32(VREG_CTRL_OFFSET), 0xA5A5_A5A5);
        let _ = p.write32(VREG_OFFSET, 0x12_3456, 0);
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
        // Force sys_clk = 150 MHz so sys_per_tick = 50.
        let tree = ClockTree {
            sys_clk_hz: 150_000_000,
            ..ClockTree::default()
        };
        let _ = p.write32(TIMER_OFFSET, POWMAN_PASSWORD | TIMER_RUN_BIT, 0);
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
        let tree = ClockTree {
            sys_clk_hz: 150_000_000,
            ..ClockTree::default()
        };
        // INTE.TIMER must be set for the NVIC raise to propagate; see
        // V12 §3.2 INTE-gating fix.
        arm_for_nvic_raise(&mut p, 2);
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
        let tree = ClockTree {
            sys_clk_hz: 150_000_000,
            ..ClockTree::default()
        };
        arm_for_nvic_raise(&mut p, 1);
        let _ = p.advance(50, &tree);
        assert_ne!(p.read32(INTR_OFFSET) & INT_TIMER_BIT, 0);
        // W1C via TIMER.ALARM (password required)
        let _ = p.write32(TIMER_OFFSET, POWMAN_PASSWORD | TIMER_ALARM_BIT, 0);
        assert_eq!(p.read32(TIMER_OFFSET) & TIMER_ALARM_BIT, 0);
        assert_eq!(p.read32(INTR_OFFSET) & INT_TIMER_BIT, 0);
    }

    /// V12 §3.2: with `INTE.TIMER = 0`, an alarm match must STILL latch
    /// `INTR.TIMER` and `TIMER.ALARM` (silicon's unconditional latch
    /// behaviour) but must NOT raise the NVIC line (silicon gates the
    /// line on `(INTR & INTE) | INTF`).
    #[test]
    fn powman_match_does_not_pend_nvic_when_inte_timer_clear() {
        let mut p = PowmanRegs::new();
        let tree = ClockTree {
            sys_clk_hz: 150_000_000,
            ..ClockTree::default()
        };
        // No INTE write — INTE.TIMER stays 0.
        let _ = p.write32(ALARM_TIME_15TO0_OFFSET, POWMAN_PASSWORD | 2, 0);
        let _ = p.write32(
            TIMER_OFFSET,
            POWMAN_PASSWORD | TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
            0,
        );

        let mask = p.advance(100, &tree);
        assert_eq!(mask, 0, "INTE.TIMER clear must suppress NVIC raise");

        // INTR.TIMER and TIMER.ALARM still latch unconditionally.
        assert_eq!(
            p.read32(INTR_OFFSET) & INT_TIMER_BIT,
            INT_TIMER_BIT,
            "INTR.TIMER must latch even when INTE.TIMER is clear"
        );
        assert_eq!(
            p.read32(TIMER_OFFSET) & TIMER_ALARM_BIT,
            TIMER_ALARM_BIT,
            "TIMER.ALARM must latch even when INTE.TIMER is clear"
        );
    }

    /// V12 §3.2: with `INTE.TIMER = 1` set BEFORE alarm match, the NVIC
    /// raise mask must equal `1u64 << 45`.
    #[test]
    fn powman_match_pends_nvic_when_inte_timer_set() {
        let mut p = PowmanRegs::new();
        let tree = ClockTree {
            sys_clk_hz: 150_000_000,
            ..ClockTree::default()
        };
        // Enable INTE.TIMER first, then arm and run TIMER.
        let _ = p.write32(INTE_OFFSET, POWMAN_PASSWORD | INT_TIMER_BIT, 0);
        let _ = p.write32(ALARM_TIME_15TO0_OFFSET, POWMAN_PASSWORD | 2, 0);
        let _ = p.write32(
            TIMER_OFFSET,
            POWMAN_PASSWORD | TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
            0,
        );

        let mask = p.advance(100, &tree);
        assert_eq!(
            mask,
            1u64 << IRQ_POWMAN_IRQ_TIMER,
            "alarm match with INTE.TIMER set must raise NVIC line 45"
        );
    }

    /// V12 §3.2: late-enable case. Drive alarm fire with INTE clear (no
    /// NVIC raise from `advance`); then write `INTE.TIMER = 1` and
    /// assert the write itself returns the NVIC raise mask. This models
    /// silicon's level-sensitive `INTS` view re-asserting NVIC line 45
    /// the moment the gate opens.
    #[test]
    fn powman_inte_set_after_intr_re_pends_nvic() {
        let mut p = PowmanRegs::new();
        let tree = ClockTree {
            sys_clk_hz: 150_000_000,
            ..ClockTree::default()
        };
        let _ = p.write32(ALARM_TIME_15TO0_OFFSET, POWMAN_PASSWORD | 2, 0);
        let _ = p.write32(
            TIMER_OFFSET,
            POWMAN_PASSWORD | TIMER_RUN_BIT | TIMER_ALARM_ENAB_BIT,
            0,
        );

        let advance_mask = p.advance(100, &tree);
        assert_eq!(advance_mask, 0, "INTE clear: advance must not raise");
        assert_eq!(
            p.read32(INTR_OFFSET) & INT_TIMER_BIT,
            INT_TIMER_BIT,
            "INTR.TIMER must be latched after the alarm match"
        );

        // Late INTE enable — the write itself should re-pend NVIC 45
        // because INTS.TIMER transitions 0 → 1.
        let write_mask = p.write32(INTE_OFFSET, POWMAN_PASSWORD | INT_TIMER_BIT, 0);
        assert_eq!(
            write_mask,
            1u64 << IRQ_POWMAN_IRQ_TIMER,
            "INTE.TIMER set with INTR.TIMER latched must (re-)raise NVIC 45"
        );
    }

    #[test]
    fn archsel_arm_default_and_no_tripwire_on_arm_write() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber {
            events: captured.clone(),
        };
        tracing::subscriber::with_default(subscriber, || {
            let mut p = PowmanRegs::new();
            assert_eq!(p.read32(ARCHSEL_OFFSET), ARCHSEL_ARM);
            let _ = p.write32(ARCHSEL_OFFSET, ARCHSEL_ARM, 0);
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
        let _ = p.write32(ARCHSEL_OFFSET, 1, 0);
        assert!(
            p.warned_archsel,
            "first non-Arm write must latch the tripwire"
        );
        let _ = p.write32(ARCHSEL_OFFSET, 2, 0);
        assert_eq!(p.read32(ARCHSEL_OFFSET), 2);
        assert!(
            p.warned_archsel,
            "tripwire stays latched on subsequent non-Arm writes"
        );
    }

    #[test]
    fn unknown_offset_roundtrip() {
        let mut p = PowmanRegs::new();
        let _ = p.write32(0xF00, 0x1234_5678, 0);
        assert_eq!(p.read32(0xF00), 0x1234_5678);
        assert_eq!(p.read32(0xF04), 0);
    }

    // --- V13 Stage 1 — BADPASSWD semantics --------------------------
    //
    // On silicon, every password-gated POWMAN register (SET_TIME_*,
    // ALARM_TIME_*, TIMER, INTE, INTF) requires bits [31:16] == 0x5AFE.
    // A wrong-password write silently drops the target mutation AND
    // latches BADPASSWD at offset 0x00 (sticky W1C, bit 0). Reads of
    // offset 0x00 return the latch state.
    //
    // These tests are the red contract for V13. The layout (bit 0, W1C)
    // is silicon-plausible but unverified — future silicon-diff runs
    // may require a bit-position adjustment here; fix in-stage (V12
    // Stage 1 CHIP_ID precedent).

    #[test]
    fn badpasswd_wrong_password_timer_write_is_dropped() {
        let mut p = PowmanRegs::new();
        // Upper16 = 0x0000 is *not* 0x5AFE — write must be dropped.
        let _ = p.write32(TIMER_OFFSET, TIMER_RUN_BIT, 0);
        assert_eq!(
            p.read32(TIMER_OFFSET) & TIMER_RUN_BIT,
            0,
            "wrong-password TIMER write must not mutate TIMER"
        );
    }

    #[test]
    fn badpasswd_wrong_password_latches_status_bit() {
        let mut p = PowmanRegs::new();
        assert_eq!(
            p.read32(CTRL_OFFSET) & 0x1,
            0,
            "BADPASSWD must be latched-low after reset"
        );
        let _ = p.write32(TIMER_OFFSET, TIMER_RUN_BIT, 0);
        assert_eq!(
            p.read32(CTRL_OFFSET) & 0x1,
            0x1,
            "wrong-password write must latch BADPASSWD bit 0"
        );
    }

    #[test]
    fn badpasswd_w1c_clears_latch() {
        let mut p = PowmanRegs::new();
        let _ = p.write32(TIMER_OFFSET, TIMER_RUN_BIT, 0);
        assert_eq!(p.read32(CTRL_OFFSET) & 0x1, 0x1);
        // W1C — writing 1 to bit 0 clears the latch. Password not
        // required for the clear itself (BADPASSWD is a status latch,
        // not itself password-gated).
        let _ = p.write32(CTRL_OFFSET, 0x1, 0);
        assert_eq!(
            p.read32(CTRL_OFFSET) & 0x1,
            0,
            "BADPASSWD must clear via W1C"
        );
    }

    #[test]
    fn badpasswd_correct_password_does_not_latch() {
        let mut p = PowmanRegs::new();
        // Correct-password write: upper16 = 0x5AFE, low16 carries RUN.
        let _ = p.write32(TIMER_OFFSET, 0x5AFE_0000 | TIMER_RUN_BIT, 0);
        assert_eq!(
            p.read32(CTRL_OFFSET) & 0x1,
            0,
            "correct-password writes must not latch BADPASSWD"
        );
        assert_eq!(
            p.read32(TIMER_OFFSET) & TIMER_RUN_BIT,
            TIMER_RUN_BIT,
            "correct-password TIMER write must apply"
        );
    }

    #[test]
    fn badpasswd_wrong_password_on_alarm_time_dropped() {
        let mut p = PowmanRegs::new();
        let _ = p.write32(ALARM_TIME_15TO0_OFFSET, 0x1234, 0);
        assert_eq!(
            p.read32(ALARM_TIME_15TO0_OFFSET),
            0,
            "wrong-password ALARM_TIME_15TO0 write must be dropped"
        );
        assert_eq!(
            p.read32(CTRL_OFFSET) & 0x1,
            0x1,
            "wrong-password ALARM_TIME write must latch BADPASSWD"
        );
    }

    #[test]
    fn badpasswd_wrong_password_on_inte_dropped() {
        let mut p = PowmanRegs::new();
        let _ = p.write32(INTE_OFFSET, INT_TIMER_BIT, 0);
        assert_eq!(
            p.read32(INTE_OFFSET) & INT_TIMER_BIT,
            0,
            "wrong-password INTE write must be dropped"
        );
        assert_eq!(p.read32(CTRL_OFFSET) & 0x1, 0x1);
    }
}

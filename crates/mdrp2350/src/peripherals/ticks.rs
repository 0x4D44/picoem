//! RP2350 TICKS peripheral (datasheet §8.5, base `0x4010_8000`).
//!
//! RP2350 factors the 1 µs tick generator — on RP2040 it lived inside
//! WATCHDOG as the `WATCHDOG_TICK` register — out into a dedicated
//! `TICKS` block with **six independent domains**: PROC0, PROC1, TIMER0,
//! TIMER1, WATCHDOG, RISCV. Each domain carries three registers:
//!
//! | Offset  | Register | Notes                                       |
//! |---------|----------|---------------------------------------------|
//! | `+0x00` | `CTRL`   | bit 0 = ENABLE, bit 1 = RUNNING (RO mirror) |
//! | `+0x04` | `CYCLES` | u32 divider. Reset = 0. SDK programs 12     |
//! | `+0x08` | `COUNT`  | current countdown (RO)                      |
//!
//! Domain base offsets within the TICKS block (prompt §3 / datasheet):
//!
//! | Domain    | Base offset |
//! |-----------|-------------|
//! | PROC0     | `0x00`      |
//! | PROC1     | `0x0C`      |
//! | TIMER0    | `0x18`      |
//! | TIMER1    | `0x24`      |
//! | WATCHDOG  | `0x30`      |
//! | RISCV     | `0x3C`      |
//!
//! # Tick semantics
//!
//! Per `sys_clk` cycle, each enabled domain accumulates `1` sys_clk.
//! When the accumulator reaches `CYCLES` the divider emits one tick
//! edge and the accumulator drops back by `CYCLES`. A write of
//! `CYCLES = 0` **halts** the domain — no edges ever fire until a
//! non-zero `CYCLES` lands. This matches datasheet §8.5 ("the divider
//! does not advance when CYCLES is 0") and the prompt §3 clarification.
//!
//! # Consumer interface
//!
//! TIMER0 / TIMER1 each consume tick edges from the matching TICKS
//! domain. After `Bus::tick_peripherals` advances TICKS, the caller
//! drains accumulated edges per domain via [`TicksRegs::take_timer0_edges`]
//! / [`TicksRegs::take_timer1_edges`] and hands them to the TIMER model.
//!
//! # Reset / post-bootrom state
//!
//! HLD V5 §5.7 pins `TICKS.TIMERn.CYCLES = 12` at emulator reset (the
//! SDK-programmed post-bootrom value), even though real silicon resets
//! to `CYCLES = 0`. [`TicksRegs::post_bootrom`] constructs that state
//! directly; [`TicksRegs::new_hardware_reset`] is the honest reset (all
//! zero) kept for tests that want to exercise the firmware-programs-it
//! path.

use super::apply_alias_rmw;

/// TICKS peripheral base address (RP2350 datasheet §8.5).
pub const TICKS_BASE: u32 = 0x4010_8000;

/// Number of independent tick domains: PROC0, PROC1, TIMER0, TIMER1,
/// WATCHDOG, RISCV.
pub const DOMAIN_COUNT: usize = 6;

/// Domain index: PROC0.
pub const DOMAIN_PROC0: usize = 0;
/// Domain index: PROC1.
pub const DOMAIN_PROC1: usize = 1;
/// Domain index: TIMER0 — drives the TIMER0 peripheral's 1 µs counter.
pub const DOMAIN_TIMER0: usize = 2;
/// Domain index: TIMER1 — drives the TIMER1 peripheral's 1 µs counter.
pub const DOMAIN_TIMER1: usize = 3;
/// Domain index: WATCHDOG (not yet modelled — stored for firmware round-trip).
pub const DOMAIN_WATCHDOG: usize = 4;
/// Domain index: RISCV (not yet modelled).
pub const DOMAIN_RISCV: usize = 5;

/// Per-domain base offset within the TICKS MMIO window.
/// `DOMAIN_STRIDE = 0x0C` (three u32 registers per domain).
pub const DOMAIN_STRIDE: u32 = 0x0C;

/// Per-domain offsets within the 0x0C-byte stride.
pub const CTRL_OFFSET: u32 = 0x00;
pub const CYCLES_OFFSET: u32 = 0x04;
pub const COUNT_OFFSET: u32 = 0x08;

/// `CTRL.ENABLE` bit mask.
pub const CTRL_ENABLE: u32 = 1 << 0;
/// `CTRL.RUNNING` bit mask (read-only mirror of ENABLE when CYCLES > 0).
pub const CTRL_RUNNING: u32 = 1 << 1;

/// SDK-programmed post-bootrom CYCLES value for TIMER0 / TIMER1 domains.
/// `clk_ref = 12 MHz` ÷ 12 = 1 MHz (1 µs per edge).
pub const CYCLES_POST_BOOTROM: u32 = 12;

/// Per-domain state.
///
/// `enable` + `cycles` are firmware-visible (via [`TicksRegs::read32`]).
/// `running` mirrors `enable` at read-time when `cycles > 0`. `sys_clks`
/// is the internal fixed-point accumulator (hidden — firmware never
/// reads it directly; `COUNT` reads report the down-count view).
#[derive(Clone, Copy, Debug, Default)]
pub struct Domain {
    pub enable: bool,
    pub cycles: u32,
    /// Internal sys_clk accumulator. On each tick this climbs by the
    /// number of sys_clks consumed; when it reaches `cycles` an edge
    /// fires and it wraps back by `cycles`.
    pub sys_clks: u32,
    /// Pending edge count for this domain since the consumer last
    /// drained it. Only the TIMER0 / TIMER1 domains have a real
    /// consumer today; others leave their edges to accumulate until
    /// saturating (see [`Domain::add_edges`]).
    pub pending_edges: u32,
}

impl Domain {
    /// Advance this domain by `cycles` sys_clk ticks. No-op if disabled
    /// or if `self.cycles == 0` (divider halted per datasheet §8.5).
    /// Returns the number of edges emitted (also latched into
    /// `pending_edges`).
    fn advance(&mut self, cycles: u32) -> u32 {
        if !self.enable || self.cycles == 0 {
            return 0;
        }
        self.sys_clks = self.sys_clks.saturating_add(cycles);
        let divisor = self.cycles;
        let edges = self.sys_clks / divisor;
        self.sys_clks -= edges * divisor;
        self.add_edges(edges);
        edges
    }

    fn add_edges(&mut self, n: u32) {
        self.pending_edges = self.pending_edges.saturating_add(n);
    }

    /// Down-count view for `COUNT` — reports `cycles - sys_clks` so
    /// firmware reading sees a countdown rather than a count-up.
    /// Matches datasheet §8.5 "COUNT reads as the remaining count
    /// before the next edge." Returns 0 if disabled or `cycles == 0`.
    fn count_view(&self) -> u32 {
        if !self.enable || self.cycles == 0 {
            return 0;
        }
        self.cycles.saturating_sub(self.sys_clks)
    }
}

/// TICKS register storage — six independent domains.
pub struct TicksRegs {
    pub domains: [Domain; DOMAIN_COUNT],
}

impl TicksRegs {
    /// Honest hardware reset — every domain CYCLES = 0, disabled. The
    /// bootrom transitions from here to the SDK-programmed CYCLES = 12
    /// before handing to firmware.
    pub fn new_hardware_reset() -> Self {
        Self {
            domains: [Domain::default(); DOMAIN_COUNT],
        }
    }

    /// Post-bootrom state (HLD V5 §5.7). TIMER0/1 domains carry
    /// `CYCLES = 12` but are **not** enabled — firmware's
    /// `runtime_init_ticks` writes `CTRL.ENABLE = 1` to start the
    /// divider. pico-sdk-pico2 follows this sequence (see
    /// `crates/mdpicoem-harness/src/silicon_scenarios.rs` comments in
    /// the TICKS retarget scenario).
    pub fn post_bootrom() -> Self {
        let mut regs = Self::new_hardware_reset();
        regs.domains[DOMAIN_TIMER0].cycles = CYCLES_POST_BOOTROM;
        regs.domains[DOMAIN_TIMER1].cycles = CYCLES_POST_BOOTROM;
        regs
    }

    /// Reset back to the post-bootrom state. Called from
    /// `Emulator::reset`.
    pub fn reset(&mut self) {
        *self = Self::post_bootrom();
    }

    /// Advance every enabled domain by `cycles` sys_clk ticks. Per
    /// HLD V5 §5.5 the tick runs every cycle unconditionally (no
    /// fast-path gate in V5).
    pub fn advance_all(&mut self, cycles: u32) {
        for d in self.domains.iter_mut() {
            let _ = d.advance(cycles);
        }
    }

    /// Drain and return pending edges from the TIMER0 domain.
    pub fn take_timer0_edges(&mut self) -> u32 {
        std::mem::take(&mut self.domains[DOMAIN_TIMER0].pending_edges)
    }

    /// Drain and return pending edges from the TIMER1 domain.
    pub fn take_timer1_edges(&mut self) -> u32 {
        std::mem::take(&mut self.domains[DOMAIN_TIMER1].pending_edges)
    }

    /// Map an offset within the TICKS window to `(domain_idx, reg_offset)`
    /// where `reg_offset` is one of [`CTRL_OFFSET`], [`CYCLES_OFFSET`],
    /// [`COUNT_OFFSET`]. Returns `None` for out-of-window offsets.
    #[inline]
    fn decode_offset(offset: u32) -> Option<(usize, u32)> {
        // RP2350 APB does not fully decode the 12-bit page offset for
        // the TICKS block. Only the low 7 bits (selecting domain +
        // register) are meaningful; upper bits are not connected on the
        // APB bus fabric. Mask to the power-of-two decode width before
        // dispatch.
        let offset = offset & 0x7F;
        let domain = (offset / DOMAIN_STRIDE) as usize;
        if domain >= DOMAIN_COUNT {
            return None;
        }
        let reg = offset % DOMAIN_STRIDE;
        Some((domain, reg))
    }

    /// Read a TICKS register by offset.
    pub fn read32(&self, offset: u32) -> u32 {
        let Some((d, reg)) = Self::decode_offset(offset) else {
            return 0;
        };
        let dom = &self.domains[d];
        match reg {
            CTRL_OFFSET => {
                let mut v = 0u32;
                if dom.enable {
                    v |= CTRL_ENABLE;
                }
                // RUNNING mirrors ENABLE when the divider is actually
                // running (CYCLES > 0). Per datasheet §8.5 "RUNNING is
                // set when the tick generator is running — enabled and
                // CYCLES != 0."
                if dom.enable && dom.cycles > 0 {
                    v |= CTRL_RUNNING;
                }
                v
            }
            CYCLES_OFFSET => dom.cycles,
            COUNT_OFFSET => dom.count_view(),
            _ => 0,
        }
    }

    /// Write a TICKS register with alias semantics. Returns `true` if
    /// the write touched a TIMER0 or TIMER1 domain register in a way
    /// that can shift the tick rate (CTRL, CYCLES, or COUNT). The
    /// caller uses this signal to invalidate TIMER0/1 cached match
    /// cycles per HLD V5 §5.4.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) -> bool {
        let Some((d, reg)) = Self::decode_offset(offset) else {
            return false;
        };
        let dom = &mut self.domains[d];
        match reg {
            CTRL_OFFSET => {
                // Rebuild the CTRL word for the alias RMW, then decode
                // back. RUNNING (bit 1) is read-only and therefore
                // stripped before storage.
                let mut word = 0u32;
                if dom.enable {
                    word |= CTRL_ENABLE;
                }
                apply_alias_rmw(&mut word, value, alias);
                let new_enable = (word & CTRL_ENABLE) != 0;
                if new_enable != dom.enable {
                    // Starting or stopping the divider — zero the
                    // accumulator so the next edge fires `CYCLES`
                    // sys_clks from the transition (matches silicon
                    // behaviour described in §8.5: a fresh ENABLE
                    // edge restarts the count from zero).
                    dom.sys_clks = 0;
                }
                dom.enable = new_enable;
            }
            CYCLES_OFFSET => {
                let mut word = dom.cycles;
                apply_alias_rmw(&mut word, value, alias);
                if word != dom.cycles {
                    // Rate change — reset the accumulator so we don't
                    // carry partial sys_clks across a divider swap.
                    dom.sys_clks = 0;
                }
                dom.cycles = word;
            }
            COUNT_OFFSET => {
                // COUNT is nominally read-only per datasheet; ignore
                // writes but still report the touch as invalidating
                // (V5 §5.4 explicitly lists COUNT as a phase-reset
                // trigger). Matches silicon where a write to the
                // debug-only COUNT port would force-reset the divider.
                dom.sys_clks = 0;
            }
            _ => return false,
        }
        // Invalidation is required for TIMER0/TIMER1 domains only;
        // PROC0/PROC1/WATCHDOG/RISCV domains do not feed the TIMER
        // lazy-schedule cache.
        d == DOMAIN_TIMER0 || d == DOMAIN_TIMER1
    }
}

impl Default for TicksRegs {
    fn default() -> Self {
        Self::post_bootrom()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Decode sanity ---------------------------------------------------

    #[test]
    fn decode_offset_covers_all_six_domains() {
        // Each domain starts at k*0x0C; the CTRL/CYCLES/COUNT triplet
        // must be decodable at the expected domain index.
        for d in 0..DOMAIN_COUNT {
            let base = (d as u32) * DOMAIN_STRIDE;
            assert_eq!(TicksRegs::decode_offset(base), Some((d, CTRL_OFFSET)));
            assert_eq!(
                TicksRegs::decode_offset(base + CYCLES_OFFSET),
                Some((d, CYCLES_OFFSET))
            );
            assert_eq!(
                TicksRegs::decode_offset(base + COUNT_OFFSET),
                Some((d, COUNT_OFFSET))
            );
        }
    }

    #[test]
    fn decode_offset_rejects_out_of_range() {
        // Offset 0x48 (domain 6) is within the 0x7F mask but beyond the
        // 6-domain window -- must return None.
        assert_eq!(
            TicksRegs::decode_offset(DOMAIN_STRIDE * DOMAIN_COUNT as u32),
            None
        );
        // Offset 0x80 aliases to 0x00 (domain 0, CTRL) via the & 0x7F
        // mask, matching silicon's incomplete APB decode.
        assert_eq!(
            TicksRegs::decode_offset(0x80),
            Some((0, CTRL_OFFSET))
        );
    }

    // --- Reset state -----------------------------------------------------

    #[test]
    fn hardware_reset_all_zero() {
        let t = TicksRegs::new_hardware_reset();
        for d in t.domains.iter() {
            assert!(!d.enable);
            assert_eq!(d.cycles, 0);
            assert_eq!(d.sys_clks, 0);
            assert_eq!(d.pending_edges, 0);
        }
    }

    #[test]
    fn post_bootrom_programs_timer_domains_to_12() {
        let t = TicksRegs::post_bootrom();
        assert_eq!(t.domains[DOMAIN_TIMER0].cycles, 12);
        assert_eq!(t.domains[DOMAIN_TIMER1].cycles, 12);
        // Still disabled — firmware writes CTRL.ENABLE to start.
        assert!(!t.domains[DOMAIN_TIMER0].enable);
        assert!(!t.domains[DOMAIN_TIMER1].enable);
        // Other domains unaffected.
        assert_eq!(t.domains[DOMAIN_PROC0].cycles, 0);
    }

    // --- Cycles = 0 halts the domain (prompt §3 clarification) -----------

    #[test]
    fn cycles_zero_stops_the_tick() {
        let mut t = TicksRegs::new_hardware_reset();
        // Enable TIMER0 with CYCLES=0. No edges must fire.
        t.domains[DOMAIN_TIMER0].enable = true;
        t.domains[DOMAIN_TIMER0].cycles = 0;
        t.advance_all(1_000_000);
        assert_eq!(t.take_timer0_edges(), 0,
            "CYCLES=0 must halt the divider, no edges");
    }

    #[test]
    fn disabled_domain_does_not_tick() {
        let mut t = TicksRegs::new_hardware_reset();
        t.domains[DOMAIN_TIMER0].cycles = 12;
        // Enable=false: no edges.
        t.advance_all(1_000);
        assert_eq!(t.take_timer0_edges(), 0);
    }

    // --- Edge emission ---------------------------------------------------

    #[test]
    fn cycles_12_emits_one_edge_per_12_sys_clks() {
        let mut t = TicksRegs::new_hardware_reset();
        t.domains[DOMAIN_TIMER0].enable = true;
        t.domains[DOMAIN_TIMER0].cycles = 12;
        // 12 sys_clks → exactly 1 edge.
        t.advance_all(12);
        assert_eq!(t.take_timer0_edges(), 1);
        // Another 11 — no edge yet, 11 sys_clks accumulated.
        t.advance_all(11);
        assert_eq!(t.take_timer0_edges(), 0);
        assert_eq!(t.domains[DOMAIN_TIMER0].sys_clks, 11);
        // One more sys_clk → edge fires.
        t.advance_all(1);
        assert_eq!(t.take_timer0_edges(), 1);
    }

    #[test]
    fn large_advance_batches_many_edges() {
        let mut t = TicksRegs::new_hardware_reset();
        t.domains[DOMAIN_TIMER0].enable = true;
        t.domains[DOMAIN_TIMER0].cycles = 12;
        // 150 MHz / 12 = 12.5M edges per second. Feed 12000 sys_clks;
        // expect 1000 edges.
        t.advance_all(12_000);
        assert_eq!(t.take_timer0_edges(), 1_000);
    }

    #[test]
    fn domains_tick_independently() {
        let mut t = TicksRegs::new_hardware_reset();
        t.domains[DOMAIN_TIMER0].enable = true;
        t.domains[DOMAIN_TIMER0].cycles = 12;
        t.domains[DOMAIN_TIMER1].enable = true;
        t.domains[DOMAIN_TIMER1].cycles = 24;
        t.advance_all(48);
        // TIMER0: 48 / 12 = 4 edges. TIMER1: 48 / 24 = 2 edges.
        assert_eq!(t.take_timer0_edges(), 4);
        assert_eq!(t.take_timer1_edges(), 2);
    }

    // --- Register-level read/write round-trip ----------------------------

    #[test]
    fn write_timer0_cycles_sets_storage() {
        let mut t = TicksRegs::new_hardware_reset();
        let off_cycles = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
        let invalidates = t.write32(off_cycles, 24, 0);
        assert!(invalidates, "CYCLES write on TIMER0 must request invalidation");
        assert_eq!(t.domains[DOMAIN_TIMER0].cycles, 24);
        assert_eq!(t.read32(off_cycles), 24);
    }

    #[test]
    fn write_timer0_ctrl_enable_sets_running_when_cycles_nonzero() {
        let mut t = TicksRegs::post_bootrom();
        let off_ctrl = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CTRL_OFFSET;
        assert_eq!(t.read32(off_ctrl), 0, "disabled at post_bootrom");
        t.write32(off_ctrl, CTRL_ENABLE, 0);
        assert_eq!(
            t.read32(off_ctrl),
            CTRL_ENABLE | CTRL_RUNNING,
            "ENABLE set + CYCLES=12 => RUNNING mirrors"
        );
    }

    #[test]
    fn ctrl_running_not_set_when_cycles_zero() {
        // A rare but legal state: firmware writes ENABLE=1 before
        // CYCLES is programmed. RUNNING must remain 0 until CYCLES > 0.
        let mut t = TicksRegs::new_hardware_reset();
        let off_ctrl = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CTRL_OFFSET;
        t.write32(off_ctrl, CTRL_ENABLE, 0);
        assert_eq!(t.read32(off_ctrl) & CTRL_RUNNING, 0);
        // Now write CYCLES — RUNNING latches.
        let off_cycles = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
        t.write32(off_cycles, 12, 0);
        assert_eq!(t.read32(off_ctrl) & CTRL_RUNNING, CTRL_RUNNING);
    }

    #[test]
    fn write_count_reports_invalidation_and_resets_accumulator() {
        let mut t = TicksRegs::post_bootrom();
        t.domains[DOMAIN_TIMER0].enable = true;
        t.domains[DOMAIN_TIMER0].sys_clks = 7; // partial accumulator
        let off_count = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + COUNT_OFFSET;
        let invalidates = t.write32(off_count, 0, 0);
        assert!(invalidates);
        assert_eq!(t.domains[DOMAIN_TIMER0].sys_clks, 0);
    }

    #[test]
    fn non_timer_domain_write_does_not_request_invalidation() {
        let mut t = TicksRegs::new_hardware_reset();
        // PROC0 CYCLES write — TIMER cache does not depend on PROC
        // domains.
        let off = DOMAIN_PROC0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
        assert!(!t.write32(off, 12, 0));
        // WATCHDOG domain likewise.
        let off = DOMAIN_WATCHDOG as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
        assert!(!t.write32(off, 12, 0));
    }

    #[test]
    fn count_read_returns_countdown_view() {
        let mut t = TicksRegs::post_bootrom();
        t.domains[DOMAIN_TIMER0].enable = true;
        let off_count = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + COUNT_OFFSET;
        // 0 sys_clks accumulated + cycles=12 → COUNT=12.
        assert_eq!(t.read32(off_count), 12);
        // Advance 5 sys_clks → COUNT=7.
        t.advance_all(5);
        assert_eq!(t.read32(off_count), 7);
    }

    #[test]
    fn count_read_zero_when_disabled() {
        let t = TicksRegs::post_bootrom();
        // Disabled domain — COUNT reads 0 regardless of stored cycles.
        let off_count = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + COUNT_OFFSET;
        assert_eq!(t.read32(off_count), 0);
    }

    // --- Mid-run rate change (prompt §3 / V5 §5.4) -----------------------

    #[test]
    fn rate_change_resets_accumulator() {
        let mut t = TicksRegs::new_hardware_reset();
        t.domains[DOMAIN_TIMER0].enable = true;
        t.domains[DOMAIN_TIMER0].cycles = 12;
        t.advance_all(7); // partial — 7 sys_clks accumulated
        assert_eq!(t.domains[DOMAIN_TIMER0].sys_clks, 7);
        // Rate change to 24 via register write. Accumulator resets.
        let off_cycles = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
        t.write32(off_cycles, 24, 0);
        assert_eq!(t.domains[DOMAIN_TIMER0].sys_clks, 0);
    }

    #[test]
    fn enable_edge_resets_accumulator() {
        let mut t = TicksRegs::new_hardware_reset();
        t.domains[DOMAIN_TIMER0].cycles = 12;
        t.domains[DOMAIN_TIMER0].sys_clks = 99;
        let off_ctrl = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CTRL_OFFSET;
        t.write32(off_ctrl, CTRL_ENABLE, 0);
        assert_eq!(t.domains[DOMAIN_TIMER0].sys_clks, 0,
            "ENABLE 0→1 transition must zero the accumulator");
    }

    // --- Alias semantics (BITSET / BITCLR) -------------------------------

    #[test]
    fn bitset_alias_sets_enable_only() {
        let mut t = TicksRegs::post_bootrom();
        let off_ctrl = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CTRL_OFFSET;
        // Alias 2 (BITSET) with ENABLE bit.
        t.write32(off_ctrl, CTRL_ENABLE, 2);
        assert!(t.domains[DOMAIN_TIMER0].enable);
    }

    #[test]
    fn alias2_at_0x81c_lands_on_timer0_cycles() {
        // HLD V5 §4.A3 regression guard: the failing silicon scenario
        // `ticks_timer0_retarget_halves_rate` writes to
        // `TICKS_BASE + 0x81C` (the alias-2/BITSET form of TIMER0.CYCLES
        // at base offset 0x1C). The 12-bit APB offset `0x81C` must mask
        // through `& 0x7F` to `0x1C` so the write lands on TIMER0.CYCLES.
        let mut t = TicksRegs::new_hardware_reset();
        // alias=2 (BITSET) with bitmask 24 => CYCLES OR= 24.
        let invalidates = t.write32(0x81C, 24, 2);
        assert!(invalidates, "write to TIMER0.CYCLES must invalidate caches");
        assert_eq!(t.domains[DOMAIN_TIMER0].cycles, 24);
    }

    #[test]
    fn bitclr_alias_clears_enable() {
        let mut t = TicksRegs::post_bootrom();
        t.domains[DOMAIN_TIMER0].enable = true;
        let off_ctrl = DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CTRL_OFFSET;
        t.write32(off_ctrl, CTRL_ENABLE, 3);
        assert!(!t.domains[DOMAIN_TIMER0].enable);
    }

    // --- Drain clears pending --------------------------------------------

    #[test]
    fn take_edges_clears_pending() {
        let mut t = TicksRegs::new_hardware_reset();
        t.domains[DOMAIN_TIMER0].enable = true;
        t.domains[DOMAIN_TIMER0].cycles = 12;
        t.advance_all(24);
        assert_eq!(t.take_timer0_edges(), 2);
        assert_eq!(t.take_timer0_edges(), 0, "drained state is empty");
    }

    // --- Out-of-window reads/writes --------------------------------------

    #[test]
    fn read_past_block_returns_zero() {
        let t = TicksRegs::post_bootrom();
        // 0x80 aliases to domain 0 CTRL (offset 0x00) via & 0x7F --
        // domain 0 is disabled at post_bootrom, so CTRL reads 0.
        assert_eq!(t.read32(0x80), 0);
        // 0xFF & 0x7F = 0x7F -> domain 0x7F/0x0C = 10, which is
        // >= DOMAIN_COUNT (6), so decode_offset returns None -> 0.
        assert_eq!(t.read32(0xFF), 0);
    }

    #[test]
    fn write_past_block_is_noop() {
        let mut t = TicksRegs::post_bootrom();
        // 0x4C & 0x7F = 0x4C -> domain 0x4C/0x0C = 6, which is
        // >= DOMAIN_COUNT, so the write is rejected (returns false).
        let before = t.domains;
        assert!(!t.write32(0x4C, 0xFFFF_FFFF, 0));
        assert_eq!(t.domains.map(|d| (d.enable, d.cycles)),
                   before.map(|d| (d.enable, d.cycles)));
    }
}

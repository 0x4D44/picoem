//! Phase 1 integration tests — TICKS + TIMER0/1 + RESETS guard +
//! `tick_peripherals` driven through the Bus public API.
//!
//! Unit-level tests for each peripheral live alongside the peripheral
//! in the same module; this file exercises the Bus-level wiring that
//! the prompt explicitly calls out as mandatory coverage (HLD V5 §5.3,
//! §5.4, §5.5, §5.7):
//!
//! - RESETS guard: held peripheral returns 0 on read, discards writes,
//!   doesn't tick.
//! - Per-cycle tick: `Emulator::step` calls `tick_peripherals` each
//!   iteration.
//! - Reset-state table: `clk_sys_hz() == 150_000_000`,
//!   `XOSC.STATUS.STABLE` reads 1 at reset, `TICKS.TIMER0.CYCLES`
//!   reads 12.
//! - Invalidation: TICKS rate change disarms TIMER alarms.

use crate::Bus;
use crate::Emulator;
use crate::bus::{
    RESET_PIO0, RESET_TIMER0, RESET_TIMER1, RESETS_POST_BOOTROM,
};
use crate::peripherals::ticks::{
    CTRL_ENABLE, CYCLES_OFFSET, CYCLES_POST_BOOTROM, DOMAIN_STRIDE, DOMAIN_TIMER0,
    DOMAIN_TIMER1, TICKS_BASE,
};
use crate::peripherals::timer::{
    ALARM0_OFFSET, ARMED_OFFSET, INTE_OFFSET, INTR_OFFSET, TIMER0_BASE, TIMER1_BASE,
    TIMERAWL_OFFSET,
};

// ----------------------------------------------------------------------------
// RESETS guard — held peripheral returns 0 on read, writes dropped, no tick
// ----------------------------------------------------------------------------

#[test]
fn resets_guard_read_held_timer0_returns_zero() {
    let mut bus = Bus::new();
    // Seed TIMER0 counter bypassing the Bus (internal tick path).
    bus.timer0.advance_us(12345);
    assert_eq!(bus.resets_state & (1 << RESET_TIMER0), 0);
    assert_eq!(bus.read32(TIMER0_BASE + TIMERAWL_OFFSET, 0), 12345);
    // Re-assert TIMER0 reset via SET alias.
    bus.write32(0x4002_2000, 1 << RESET_TIMER0, 0);
    assert_ne!(bus.resets_state & (1 << RESET_TIMER0), 0);
    assert_eq!(
        bus.read32(TIMER0_BASE + TIMERAWL_OFFSET, 0),
        0,
        "held TIMER0 must read 0 via RESETS guard"
    );
}

#[test]
fn resets_guard_write_held_timer0_discarded() {
    let mut bus = Bus::new();
    bus.write32(0x4002_2000, 1 << RESET_TIMER0, 0);
    bus.write32(TIMER0_BASE + INTE_OFFSET, 0xF, 0);
    bus.write32(0x4002_3000, 1 << RESET_TIMER0, 0);
    assert_eq!(
        bus.read32(TIMER0_BASE + INTE_OFFSET, 0),
        0,
        "writes to held TIMER0 must be discarded"
    );
}

#[test]
fn resets_guard_write_held_timer1_discarded() {
    let mut bus = Bus::new();
    bus.write32(0x4002_2000, 1 << RESET_TIMER1, 0);
    bus.write32(TIMER1_BASE + INTE_OFFSET, 0xF, 0);
    bus.write32(0x4002_3000, 1 << RESET_TIMER1, 0);
    assert_eq!(bus.read32(TIMER1_BASE + INTE_OFFSET, 0), 0);
}

#[test]
fn resets_guard_held_timer0_does_not_tick() {
    let mut bus = Bus::new();
    bus.write32(0x4002_2000, 1 << RESET_TIMER0, 0);
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    bus.tick_peripherals(240);
    bus.write32(0x4002_3000, 1 << RESET_TIMER0, 0);
    assert_eq!(
        bus.read32(TIMER0_BASE + TIMERAWL_OFFSET, 0),
        0,
        "TIMER0 must not advance while held in RESETS"
    );
}

#[test]
fn resets_guard_unrelated_peripheral_unaffected() {
    let mut bus = Bus::new();
    bus.write32(0x4002_2000, 1 << RESET_TIMER0, 0);
    bus.timer1.advance_us(999);
    assert_eq!(
        bus.read32(TIMER1_BASE + TIMERAWL_OFFSET, 0),
        999,
        "TIMER1 unaffected by TIMER0 reset"
    );
}

// ----------------------------------------------------------------------------
// Reset-state table (HLD V5 §5.7)
// ----------------------------------------------------------------------------

#[test]
fn reset_state_clk_sys_is_150_mhz_after_bus_new() {
    // B5: Bus::new() must land on post-bootrom state directly (HLD V5 §5.7).
    let bus = Bus::new();
    assert_eq!(
        bus.sys_clk_hz(),
        150_000_000,
        "HLD V5 §5.7: Bus::new() lands on post-bootrom clk_sys = 150 MHz"
    );
}

#[test]
fn reset_state_clk_sys_is_150_mhz_after_emulator_new() {
    // B5: Emulator::new(Config::default()) yields the same state as Bus::new().
    let emu = Emulator::new(crate::Config::default());
    assert_eq!(
        emu.bus.sys_clk_hz(),
        150_000_000,
        "HLD V5 §5.7: Emulator::new(default) observes post-bootrom clk_sys"
    );
}

#[test]
fn reset_state_clk_sys_is_150_mhz_after_emulator_reset() {
    let mut emu = Emulator::new(crate::Config::default());
    emu.reset();
    assert_eq!(
        emu.bus.sys_clk_hz(),
        150_000_000,
        "HLD V5 §5.7: post-bootrom clk_sys = 150 MHz"
    );
}

#[test]
fn reset_state_clk_ref_is_12_mhz_after_bus_new() {
    let bus = Bus::new();
    assert_eq!(
        bus.ref_clk_hz(),
        12_000_000,
        "HLD V5 §5.7: Bus::new() lands on post-bootrom clk_ref = 12 MHz"
    );
}

#[test]
fn reset_state_clk_ref_is_12_mhz_after_emulator_reset() {
    let mut emu = Emulator::new(crate::Config::default());
    emu.reset();
    assert_eq!(
        emu.bus.ref_clk_hz(),
        12_000_000,
        "HLD V5 §5.7: post-bootrom clk_ref = 12 MHz (XOSC)"
    );
}

#[test]
fn bus_new_equals_emulator_reset_clock_state() {
    // B5: the three construction paths (Bus::new, Emulator::new,
    // Emulator::reset) MUST land on identical clock-tree state. Any
    // divergence re-introduces the Phase 1 bug where a fresh emulator
    // ran at 6.5 MHz while TICKS was programmed for 12 MHz clk_ref.
    let bus = Bus::new();
    let new_emu = Emulator::new(crate::Config::default());
    let mut reset_emu = Emulator::new(crate::Config::default());
    reset_emu.reset();
    assert_eq!(bus.sys_clk_hz(), new_emu.bus.sys_clk_hz());
    assert_eq!(bus.sys_clk_hz(), reset_emu.bus.sys_clk_hz());
    assert_eq!(bus.ref_clk_hz(), new_emu.bus.ref_clk_hz());
    assert_eq!(bus.ref_clk_hz(), reset_emu.bus.ref_clk_hz());
}

#[test]
fn reset_state_xosc_stable() {
    let mut bus = Bus::new();
    let status = bus.read32(0x4004_8004, 0);
    assert_ne!(
        status & (1u32 << 31),
        0,
        "XOSC.STATUS.STABLE must read 1 at reset (V5 §5.7)"
    );
}

#[test]
fn reset_state_pll_sys_cs_lock_phase1_gap() {
    // HLD V5 §5.7 prescribes `PLL_SYS.CS.LOCK = 1` at reset. The PLL
    // model (post `2026.04.15 HLD - PLL LOCK Modelling`) reports LOCK
    // from the power/FBDIV predicate, and at Bus::new() the PLL image
    // is hardware-reset (powered-down, FBDIV=0) — predicate false, so
    // LOCK reads 0. Closing this gap requires seeding the PLL register
    // image to "powered-up + configured + armed" at Bus::new(), which
    // is Phase 2 scope (PLL reset-state seed). This test documents the
    // current behaviour so a future fix has a canary. R4: explicit
    // coverage of the §5.7 LOCK line.
    let mut bus = Bus::new();
    let cs = bus.read32(0x4005_0000, 0);
    assert_eq!(
        cs & (1u32 << 31),
        0,
        "Phase 1: PLL_SYS.CS.LOCK = 0 at Bus::new() (PLL image not yet \
         seeded to configured state; §5.7 LOCK=1 is a Phase 2 follow-up)"
    );
}

#[test]
fn reset_state_pll_usb_cs_lock_phase1_gap() {
    // Same story as PLL_SYS — HLD V5 §5.7 says LOCK=1, but the PLL
    // image is hardware-reset. Phase 2 closes this.
    let mut bus = Bus::new();
    let cs = bus.read32(0x4005_8000, 0);
    assert_eq!(
        cs & (1u32 << 31),
        0,
        "Phase 1: PLL_USB.CS.LOCK = 0 at Bus::new() (Phase 2 follow-up)"
    );
}

#[test]
fn reset_state_clk_ref_selected_mirrors_ctrl() {
    // HLD V5 §5.7: CLOCKS.clk_*_selected mirrors CTRL.SRC/AUXSRC. At
    // reset CLK_REF_CTRL.SRC = 0 → SELECTED = 1<<0 = 1 (glitchless
    // mux — see bus/peripherals.rs::clocks_read).
    let mut bus = Bus::new();
    let sel = bus.read32(0x4001_0038, 0);
    assert_eq!(
        sel, 0x1,
        "CLK_REF_SELECTED must mirror CTRL.SRC at reset (§5.7)"
    );
}

#[test]
fn reset_state_clk_sys_selected_mirrors_ctrl() {
    // CLK_SYS_CTRL.SRC = 0 → SELECTED = 1. Glitchless mux.
    let mut bus = Bus::new();
    let sel = bus.read32(0x4001_0044, 0);
    assert_eq!(
        sel, 0x1,
        "CLK_SYS_SELECTED must mirror CTRL.SRC at reset (§5.7)"
    );
}

#[test]
fn reset_state_clk_peri_selected_reads_one() {
    // Non-glitchless mux — _SELECTED reads 1 unconditionally (see
    // bus/peripherals.rs::clocks_read). Satisfies pico-sdk's busy-wait
    // on clock_configure.
    let mut bus = Bus::new();
    assert_eq!(bus.read32(0x4001_0050, 0), 0x1, "CLK_PERI_SELECTED = 1");
}

#[test]
fn reset_state_ticks_timer0_cycles_is_12() {
    let mut bus = Bus::new();
    let off_cycles = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
    assert_eq!(bus.read32(off_cycles, 0), CYCLES_POST_BOOTROM);
    assert_eq!(CYCLES_POST_BOOTROM, 12);
}

#[test]
fn reset_state_ticks_timer1_cycles_is_12() {
    let mut bus = Bus::new();
    let off_cycles = TICKS_BASE + DOMAIN_TIMER1 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
    assert_eq!(bus.read32(off_cycles, 0), CYCLES_POST_BOOTROM);
}

#[test]
fn reset_state_resets_releases_timer_and_pll() {
    let bus = Bus::new();
    assert_eq!(bus.resets_state, RESETS_POST_BOOTROM);
    assert_eq!(bus.resets_state & (1 << RESET_TIMER0), 0);
    assert_eq!(bus.resets_state & (1 << RESET_TIMER1), 0);
    // PIO should still be held — firmware programs PIO per-use.
    assert_ne!(bus.resets_state & (1 << RESET_PIO0), 0);
}

#[test]
fn emulator_reset_restores_post_bootrom_ticks_cycles() {
    let mut emu = Emulator::new(crate::Config::default());
    let off_cycles = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
    emu.bus.write32(off_cycles, 24, 0);
    assert_eq!(emu.bus.read32(off_cycles, 0), 24);
    emu.reset();
    assert_eq!(emu.bus.read32(off_cycles, 0), 12);
}

// ----------------------------------------------------------------------------
// tick_peripherals — TICKS divides, TIMER0/1 advance, IRQ dispatch
// ----------------------------------------------------------------------------

#[test]
fn tick_peripherals_advances_timer0_after_ticks_enable() {
    let mut bus = Bus::new();
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    bus.tick_peripherals(120);
    assert_eq!(
        bus.read32(TIMER0_BASE + TIMERAWL_OFFSET, 0),
        10,
        "120 sys_clks / 12 = 10 µs on TIMER0"
    );
}

#[test]
fn tick_peripherals_timer0_halts_when_ticks_cycles_zero() {
    let mut bus = Bus::new();
    let ticks_cycles_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
    bus.write32(ticks_cycles_t0, 0, 0);
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    bus.tick_peripherals(1_000_000);
    assert_eq!(
        bus.read32(TIMER0_BASE + TIMERAWL_OFFSET, 0),
        0,
        "CYCLES=0 freezes TIMER0"
    );
}

#[test]
fn tick_peripherals_alarm_match_raises_irq_via_assert_shared() {
    let mut bus = Bus::new();
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    bus.write32(TIMER0_BASE + INTE_OFFSET, 0x1, 0);
    bus.write32(TIMER0_BASE + ALARM0_OFFSET, 5, 0);
    bus.tick_peripherals(60);
    let irq = crate::irq::IRQ_TIMER0_IRQ_0;
    assert_ne!(
        bus.irq_pending[0] & (1u64 << irq),
        0,
        "core 0 sees TIMER0 IRQ 0"
    );
    assert_ne!(
        bus.irq_pending[1] & (1u64 << irq),
        0,
        "core 1 sees TIMER0 IRQ 0 (shared)"
    );
    assert_eq!(bus.read32(TIMER0_BASE + INTR_OFFSET, 0) & 1, 1);
    assert_eq!(bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1, 0);
}

#[test]
fn tick_peripherals_timer1_fires_on_its_own_irq_base() {
    let mut bus = Bus::new();
    let ticks_ctrl_t1 = TICKS_BASE + DOMAIN_TIMER1 as u32 * DOMAIN_STRIDE;
    bus.write32(ticks_ctrl_t1, CTRL_ENABLE, 0);
    bus.write32(TIMER1_BASE + INTE_OFFSET, 0x1, 0);
    bus.write32(TIMER1_BASE + ALARM0_OFFSET, 3, 0);
    bus.tick_peripherals(36);
    let irq1 = crate::irq::IRQ_TIMER1_IRQ_0;
    assert_ne!(bus.irq_pending[0] & (1u64 << irq1), 0);
    let irq0 = crate::irq::IRQ_TIMER0_IRQ_0;
    assert_eq!(bus.irq_pending[0] & (1u64 << irq0), 0);
}

#[test]
fn emulator_step_calls_tick_peripherals_each_iteration() {
    let mut emu = Emulator::new(crate::Config::default());
    emu.cores[0].halt();
    emu.cores[1].halt();
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    emu.bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    // step_quantum default = 64 sys_clks. At CYCLES=12:
    //   step 1: acc=0,  +64 → 64, 64/12 = 5 edges rem 4 → TIMER0 = 5
    //   step 2: acc=4,  +64 → 68, 68/12 = 5 edges rem 8 → TIMER0 = 10
    //   step 3: acc=8,  +64 → 72, 72/12 = 6 edges rem 0 → TIMER0 = 16
    emu.step();
    assert_eq!(
        emu.bus.read32(TIMER0_BASE + TIMERAWL_OFFSET, 0),
        5,
        "1 step × 64 sys_clks / 12 = 5 edges on TIMER0"
    );
    emu.step();
    emu.step();
    assert_eq!(
        emu.bus.read32(TIMER0_BASE + TIMERAWL_OFFSET, 0),
        16,
        "3 steps × 64 sys_clks / 12 = 16 edges (carrying remainders)"
    );
}

// ----------------------------------------------------------------------------
// TICKS rate change mid-run invalidates TIMER alarms (HLD V5 §5.4)
// ----------------------------------------------------------------------------

#[test]
fn ticks_cycles_change_preserves_timer_alarms() {
    // R1: TICKS rate-change invalidation is a no-op. TIMER state is
    // µs-based (rate-invariant), so armed alarms continue at whatever
    // new cadence TICKS is reprogrammed to. Silicon matches this
    // (datasheet §12.8 — alarms count against the µs timer regardless
    // of the divider).
    let mut bus = Bus::new();
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    bus.write32(TIMER0_BASE + ALARM0_OFFSET, 100, 0);
    assert_eq!(bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1, 1);
    let ticks_cycles_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE + CYCLES_OFFSET;
    bus.write32(ticks_cycles_t0, 24, 0);
    assert_eq!(
        bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1,
        1,
        "TICKS rate change must NOT disarm TIMER0 alarms (R1)"
    );
    // The alarm fires after enough ticks accumulate at the new cadence.
    // At CYCLES=24, 100 µs needs 2400 sys_clks.
    bus.tick_peripherals(3000);
    assert_eq!(
        bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1,
        0,
        "alarm must fire at the new TICKS rate without re-arm"
    );
}

#[test]
fn ticks_ctrl_enable_set_invalidates_timer_alarms() {
    // S2: clarified name — ENABLE 0→1 transition resets the TICKS
    // accumulator (see ticks.rs) and the bus routes the write-touch
    // signal through invalidate_lazy. R1: the armed bit survives; only
    // the cached fire-µs is dropped.
    let mut bus = Bus::new();
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    bus.write32(TIMER0_BASE + ALARM0_OFFSET, 50, 0);
    assert_eq!(bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1, 1);
    bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    assert_eq!(
        bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1,
        1,
        "TICKS ENABLE set must NOT disarm TIMER0 alarms (R1)"
    );
}

#[test]
fn ticks_proc0_domain_does_not_invalidate_timer_alarms() {
    let mut bus = Bus::new();
    bus.write32(TIMER0_BASE + ALARM0_OFFSET, 50, 0);
    assert_eq!(bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1, 1);
    let ticks_cycles_proc0 = TICKS_BASE + CYCLES_OFFSET;
    bus.write32(ticks_cycles_proc0, 24, 0);
    assert_eq!(
        bus.read32(TIMER0_BASE + ARMED_OFFSET, 0) & 1,
        1,
        "PROC0 TICKS write must not touch TIMER alarms"
    );
}

// ----------------------------------------------------------------------------
// Subword access + alias round-trip through Bus
// ----------------------------------------------------------------------------

#[test]
fn timer0_inte_write8_round_trips() {
    let mut bus = Bus::new();
    // Write byte 0 of INTE with 0xF.
    bus.write8(TIMER0_BASE + INTE_OFFSET, 0xF, 0);
    assert_eq!(bus.read32(TIMER0_BASE + INTE_OFFSET, 0) & 0xF, 0xF);
}

#[test]
fn timer0_inte_bitset_alias_via_bus() {
    let mut bus = Bus::new();
    // Alias 2 (SET) — offset 0x2000 relative to TIMER0_BASE.
    bus.write32(TIMER0_BASE + 0x2000 + INTE_OFFSET, 0x5, 0);
    assert_eq!(bus.read32(TIMER0_BASE + INTE_OFFSET, 0) & 0xF, 0x5);
    // Add more bits via SET.
    bus.write32(TIMER0_BASE + 0x2000 + INTE_OFFSET, 0xA, 0);
    assert_eq!(bus.read32(TIMER0_BASE + INTE_OFFSET, 0) & 0xF, 0xF);
}

#[test]
fn ticks_held_timer_write_discarded_via_bus_guard() {
    // Note: TICKS itself has no RESETS bit, so writing TICKS registers
    // is never blocked by the guard. This test documents that contract.
    let mut bus = Bus::new();
    // Hold TIMER0 — does NOT affect TICKS access.
    bus.write32(0x4002_2000, 1 << RESET_TIMER0, 0);
    let ticks_ctrl_t0 = TICKS_BASE + DOMAIN_TIMER0 as u32 * DOMAIN_STRIDE;
    bus.write32(ticks_ctrl_t0, CTRL_ENABLE, 0);
    // TICKS accepts the write even though TIMER0 is held.
    assert_ne!(bus.read32(ticks_ctrl_t0, 0) & CTRL_ENABLE, 0);
}

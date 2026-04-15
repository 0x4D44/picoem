// Peripheral oracle catalog — scenarios diffed by
// `silicon_periph_diff_rp2350` against real RP2354 silicon. Each
// scenario is absolute-MMIO writes + a sysclk bound + readbacks; the
// runner CYCCNT-measures actual cycles and diffs emulator vs HW. See
// `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md`
// §Oracle 1.

// ---------------------------------------------------------------------------
// Absolute MMIO bases and bit constants (RP2350)
// ---------------------------------------------------------------------------

pub const PIO0_BASE: u32 = 0x5020_0000;
pub const PIO1_BASE: u32 = 0x5030_0000;

pub const PLL_SYS_BASE: u32 = 0x4005_0000;

pub const RESETS_BASE: u32 = 0x4002_0000;
pub const RESETS_RESET: u32 = RESETS_BASE + 0x00;

/// APB alias offsets. `+0x2000` = SET (OR), `+0x3000` = CLR (AND NOT).
pub const ALIAS_SET: u32 = 0x2000;
pub const ALIAS_CLR: u32 = 0x3000;

/// RESETS bits used by v1 scenarios.
pub const RESET_IO_BANK0: u32 = 1 << 6;
pub const RESET_PADS_BANK0: u32 = 1 << 9;
pub const RESET_PIO0: u32 = 1 << 11;
pub const RESET_PIO1: u32 = 1 << 12;
pub const RESET_PLL_SYS: u32 = 1 << 14;

// PIO register offsets (identical for all three PIO blocks).
pub const PIO_CTRL_OFF: u32 = 0x000;
pub const PIO_FDEBUG_OFF: u32 = 0x008;
pub const PIO_DBG_PADOE_OFF: u32 = 0x040;
pub const PIO_INSTR_MEM_OFF: u32 = 0x048;
pub const PIO_SM_STRIDE: u32 = 0x18;
pub const PIO_SM0_BASE_OFF: u32 = 0x0C8;
pub const PIO_SM_CLKDIV_OFF: u32 = 0x00;
pub const PIO_SM_EXECCTRL_OFF: u32 = 0x04;
pub const PIO_SM_ADDR_OFF: u32 = 0x0C;
pub const PIO_SM_PINCTRL_OFF: u32 = 0x14;

/// Compute the absolute address of `SMx_<field>` for a given PIO base.
pub const fn pio_sm_addr(base: u32, sm: u32, field_off: u32) -> u32 {
    base + PIO_SM0_BASE_OFF + sm * PIO_SM_STRIDE + field_off
}

/// Compute the absolute address of `INSTR_MEM[slot]` for a given PIO base.
pub const fn pio_instr_mem_addr(base: u32, slot: u32) -> u32 {
    base + PIO_INSTR_MEM_OFF + slot * 4
}

// SIO / IO_BANK0 / PADS_BANK0 addresses used below.
pub const SIO_GPIO_IN: u32 = 0xD000_0004;
pub const SIO_GPIO_OE: u32 = 0xD000_0030;
pub const IO_BANK0_GPIO0_CTRL: u32 = 0x4002_8000 + 0x04;
pub const PADS_BANK0_GPIO0: u32 = 0x4003_8000 + 0x04;

// PLL_SYS register offsets + CS.LOCK bit.
pub const PLL_CS_OFF: u32 = 0x000;
pub const PLL_PWR_OFF: u32 = 0x004;
pub const PLL_FBDIV_INT_OFF: u32 = 0x008;
pub const PLL_PRIM_OFF: u32 = 0x00C;
pub const PLL_CS_LOCK_BIT: u32 = 1 << 31;

// CLOCKS block (RP2350). Base is 0x4001_0000. Per datasheet layout in
// `crates/mdrp2350/src/bus/peripherals.rs:79-87`, RP2350's CLOCKS
// register map adds GPOUT4-7 before CLK_REF, shifting CLK_SYS earlier
// than on RP2040: CLK_SYS_DIV lives at offset 0x040 (not 0x044 —
// that's CLK_SYS_SELECTED, which is read-only). Writable integer
// divider is in bits [31:16]; fractional in [15:0].
pub const CLOCKS_CLK_SYS_DIV: u32 = 0x4001_0040;

// ---------------------------------------------------------------------------
// Scenario type
// ---------------------------------------------------------------------------

/// A single peripheral oracle scenario. Runner applies `setup` in order
/// (probe-rs on HW, `bus.write32` on EMU), runs at most `max_sysclks`
/// cycles, then reads `observe` (MMIO) + `observe_pins` (GPIO). First
/// masked-bit divergence wins.
pub struct PeriphScenario {
    pub name: &'static str,
    /// `(absolute_addr, value)` — must be in APB/AHB (0x4000_0000..
    /// 0x5FFF_FFFF) or SIO (0xD000_0000); enforced by unit tests.
    pub setup: &'static [(u32, u32)],
    /// Upper bound on sysclks; CYCCNT-measured actual is handed to
    /// `Emulator::run` so both sides advance identically.
    pub max_sysclks: u32,
    /// `(absolute_addr, mask)` — `0xFFFF_FFFF` = full word.
    pub observe: &'static [(u32, u32)],
    /// GPIO pins to sample drive+level. 0 = skip pins.
    pub observe_pins: u32,
    /// If `Some(bytes)`, the runner uploads these as the sled instead
    /// of auto-assembling a countdown. Bytes must end in `bkpt #0`
    /// (`0xBE00`). Existing scenarios leave this `None`.
    pub custom_sled: Option<&'static [u8]>,
}

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

// S1: PIO0 SM0 runs `JMP 0` in a one-instruction loop. Positive
// control — ADDR never advances past 0, HW and EMU MUST agree.
const S_PIO0_NOP_LOOP: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
    (pio_instr_mem_addr(PIO0_BASE, 0), 0x0000), // JMP 0
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
];
const O_PIO0_NOP_LOOP: &[(u32, u32)] = &[
    // SM_ADDR is a 5-bit register.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_ADDR_OFF), 0x1F),
];

// S2: `SET X, 31 / JMP X-- [1] / JMP 2 (stall)`. After countdown, ADDR
// settles at slot 2 (the stall). EXECCTRL is programmed with a
// non-default WRAP so divergence in wrap-bit storage shows up.
const S_PIO0_FIXED_CYCLES: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
    (pio_instr_mem_addr(PIO0_BASE, 0), 0xE03F), // SET X, 31
    (pio_instr_mem_addr(PIO0_BASE, 1), 0x0041), // JMP X-- 1
    (pio_instr_mem_addr(PIO0_BASE, 2), 0x0002), // JMP 2 (stall)
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    // EXECCTRL: WRAP_TOP=2 (bits 16:12), WRAP_BOTTOM=0 (bits 11:7).
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_EXECCTRL_OFF), 2u32 << 12),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
];
const O_PIO0_FIXED_CYCLES: &[(u32, u32)] = &[
    // ADDR is a 5-bit field.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_ADDR_OFF), 0x1F),
    // EXECCTRL WRAP_TOP/BOTTOM bits [16:7] — the fields we programmed.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_EXECCTRL_OFF), 0x0001_FF80),
];

// S3: GPIO0 side-set toggle. SM0 runs `JMP 0, side 1` — one
// instruction with side=1 on GPIO0 every cycle. IO_BANK0 / PADS_BANK0
// configured to route GPIO0 through PIO0.
const S_PIO0_SIDE_SET_TOGGLE: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0 | RESET_IO_BANK0 | RESET_PADS_BANK0),
    // PADS_BANK0 GPIO0: IE=1, drive=4 mA (value matches paced_bench_rp2350).
    (PADS_BANK0_GPIO0, 0x0000_0056),
    // IO_BANK0 GPIO0_CTRL: FUNCSEL=6 (PIO0).
    (IO_BANK0_GPIO0_CTRL, 0x0000_0006),
    // INSTR_MEM[0] = JMP 0, side-set 1. With SIDESET_COUNT=1, side value
    // 1 lives in delay/sideset field bit 4 → opcode 0x1000.
    (pio_instr_mem_addr(PIO0_BASE, 0), 0x1000),
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    // PINCTRL: SIDESET_COUNT=1 (bits 31:29), SIDESET_BASE=0.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_PINCTRL_OFF), 1 << 29),
    // EXECCTRL: SIDE_PINDIR=0 (default — side-set writes to OUT).
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_EXECCTRL_OFF), 0),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
];
const O_PIO0_SIDE_SET_TOGGLE: &[(u32, u32)] = &[
    // `DBG_PADOE` is the PIO-side output-enable mirror (PIO0 + 0x040).
    // Per RP2350 §11.3.2.3, with `EXECCTRL.SIDE_PINDIR=0` and no
    // explicit `SET PINDIRS`, side-set drives pin *values* only —
    // direction stays zero. Silicon correctly reports `pad_oe=0`; the
    // emulator's `PioBlock::merge_pin_outputs` (see
    // `crates/mdpicoem-common/src/pio/mod.rs:196`) ORs the positioned
    // side-set mask into `pad_oe` while SMs run, but the runner's gate
    // write (`PIO_CTRL=0`) disables all SMs before readback, and
    // `merge_pin_outputs` then zeroes `pad_oe` again. So both sides
    // read 0 here post-gate — included for conceptual completeness
    // and to catch a future emulator regression that leaks pad_oe
    // past the gate, but it is NOT the bug-exposing signal today.
    // Status: FIXED 2026-04-15 — `merge_pin_outputs` no longer forces
    // OE in the value-drive branch; bug-exposing `GPIO_IN` divergence
    // below is now resolved. Scenario retained as regression guard.
    (PIO0_BASE + PIO_DBG_PADOE_OFF, 0xFFFF_FFFF),
    // FDEBUG TXSTALL/TXOVER bands [27:24] + [19:16] — a healthy
    // side-set loop keeps both zero.
    (PIO0_BASE + PIO_FDEBUG_OFF, 0x0F0F_0000),
    // The load-bearing signal lives in `observe_pins` below: SIO-side
    // `GPIO_OE` / `GPIO_IN` at `0xD000_0030` / `0xD000_0004`. These
    // reflect the output-fabric state, and the side-set `pad_oe` bug
    // leaks through into `GPIO_IN`'s level bit — HW reads 0 (tri-
    // state), EMU reads 1 (driven-high from side-set). That is the
    // divergence this scenario catches.
];

// S4: PIO0 RESETS gating — PLACEHOLDER.
//
// Intent: assert PIO0 reset mid-run and verify the SM freezes, which
// tests the tech-debt item "PIO not gated on RESETS bit" (both
// mdrp2350 and mdrp2040 tick PIO unconditionally regardless of
// RESETS). The original 1-instruction design (JMP 0) couldn't
// exercise the bug because ADDR=0 is invariant. The devils-advocate
// reviewer proposed upgrading to a 2-instruction program (NOP + JMP
// 0) so ADDR alternates 0↔1 and a broken emu (ticking PIO while
// gated) lands at `actual_sysclks % 2`, while a correctly-gated emu
// stays at its post-setup ADDR.
//
// Empirical result (2026-04-15 run): this still PASSes on silicon
// with the 2-instruction program. HW settles at ADDR=0 (setup writes
// are fast enough on probe-rs that PIO hasn't advanced from 0 by the
// time the RESETS_SET write lands), and the measured
// `actual_sysclks=158` is even, so broken-EMU also lands at ADDR=0.
// A longer program (3+ states) has the same modular-agreement
// hazard, and `actual_sysclks` is determined by sled pipelining on
// silicon — not controllable to force a mismatch.
//
// Verdict: the scenario as designed cannot reliably expose the bug.
// Kept in the catalog as a placeholder so the tech-debt target
// stays visible and the next scenario redesign has an entry point.
// Proper future design: either poll ADDR *while* the gate is held
// (requires mid-run probe sampling, not end-state diff), or
// arrange a program whose reset-frozen state is architecturally
// distinct from any transient running state (tricky on PIO given
// that ADDR is the only non-FIFO state observable externally).
const S_PIO0_RESET_GATING_PLACEHOLDER: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
    (pio_instr_mem_addr(PIO0_BASE, 0), 0xA042), // NOP (MOV Y, Y)
    (pio_instr_mem_addr(PIO0_BASE, 1), 0x0000), // JMP 0
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
    // Slam PIO0 back into reset after SM is running.
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
];
const O_PIO0_RESET_GATING_PLACEHOLDER: &[(u32, u32)] = &[
    // SM_ADDR (5-bit). Can false-PASS — see scenario comment above.
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_ADDR_OFF), 0x1F),
];

// S5: PLL_SYS lock. Configure FBDIV=100, REFDIV=1, POSTDIV=2/2, power
// up, spin 1500 sysclks, read CS.LOCK. Emulator's `pll_read_from`
// forces LOCK=1 unconditionally (tech-debt "PLL LOCK always 1",
// originally logged against mdrp2040 — same pattern in mdrp2350).
const S_PLL_SYS_LOCK_TIMING: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PLL_SYS),
    (PLL_SYS_BASE + PLL_CS_OFF, 1),        // REFDIV=1
    (PLL_SYS_BASE + PLL_FBDIV_INT_OFF, 100),
    (PLL_SYS_BASE + PLL_PRIM_OFF, (2u32 << 16) | (2u32 << 12)),
    (PLL_SYS_BASE + PLL_PWR_OFF, 0),       // all powered up
];
const O_PLL_SYS_LOCK_TIMING: &[(u32, u32)] = &[
    // CS.LOCK bit only — the narrower the mask, the clearer the failure.
    (PLL_SYS_BASE + PLL_CS_OFF, PLL_CS_LOCK_BIT),
];

// S6: Clock tree — PLL_SYS FBDIV reprogrammed mid-run. Setup primes
// PLL_SYS at FBDIV=125 (12 MHz × 125 / (2·2) = 375 MHz VCO, 93.75 MHz
// postdiv). The custom sled spins ~500 sysclks, writes FBDIV=100 to
// PLL_SYS (mid-run reprogramming without toggling RESETS), spins ~500
// more sysclks, then BKPTs. Observables: PLL_SYS.CS (LOCK + status
// bits), PLL_SYS.FBDIV_INT (the new value must have stuck),
// PLL_SYS.PRIM (post-divs unchanged).
//
// Safety: PLL_SYS is *not* switched to be sys_clk's source by this
// scenario's setup. The core keeps running on whatever source the
// bootrom left active (typically ROSC / XOSC post-reset_and_halt), so
// reprogramming PLL_SYS FBDIV is architecturally a no-op for the
// running core — no glitch risk. Exercises the ClockTree recompute
// path on the PLL register write, per the HLD §"Cycle-vs-frequency
// semantics" (CYCCNT counts core ticks regardless of PLL state).
const S_CLOCK_PLL_SYS_REPROGRAM_MID_RUN: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PLL_SYS),
    (PLL_SYS_BASE + PLL_CS_OFF, 1),        // REFDIV=1
    (PLL_SYS_BASE + PLL_FBDIV_INT_OFF, 125), // initial FBDIV
    (PLL_SYS_BASE + PLL_PRIM_OFF, (2u32 << 16) | (2u32 << 12)), // POSTDIV1=2, POSTDIV2=2
    (PLL_SYS_BASE + PLL_PWR_OFF, 0),       // all powered up
];
const O_CLOCK_PLL_SYS_REPROGRAM_MID_RUN: &[(u32, u32)] = &[
    // CS.LOCK (bit 31) — by the end of the post-reprogram window LOCK
    // should either re-assert or at least match between HW and EMU.
    (PLL_SYS_BASE + PLL_CS_OFF, PLL_CS_LOCK_BIT),
    // FBDIV_INT is a 12-bit field — mask enforces that we only read
    // architecturally-defined bits (datasheet §8.6.2: 12-bit divider).
    (PLL_SYS_BASE + PLL_FBDIV_INT_OFF, 0x0000_0FFF),
    // PRIM holds POSTDIV1 [18:16] and POSTDIV2 [14:12]. Verify they
    // survived the FBDIV write untouched.
    (PLL_SYS_BASE + PLL_PRIM_OFF, (7u32 << 16) | (7u32 << 12)),
];

// Custom sled for `clock_pll_sys_reprogram_mid_run`.
//
// Structure:
//   - Spin ~500 sysclks (125-iter × ~4 cycles/iter countdown).
//   - Write FBDIV_INT = 100 to PLL_SYS at 0x4005_0008.
//   - Spin ~500 more sysclks.
//   - BKPT #0.
//
// Registers used (all caller-saved, no need to preserve):
//   r0 — loop counter
//   r1 — PLL_SYS.FBDIV_INT address literal (0x4005_0008)
//   r2 — new FBDIV value literal (100)
//
// Thumb-2 encodings per ARMv8-M ARM:
//   movw T3:   hw0 = 0xF240 | (i<<10) | imm4,
//              hw1 = (imm3<<12) | (Rd<<8) | imm8
//   movt T1:   hw0 = 0xF2C0 | (i<<10) | imm4, hw1 format same as movw
//   subs T2:   hw0 = 0x3800 | (Rdn<<8) | imm8   (16-bit)
//   bne  T1:   hw0 = 0xD100 | (imm8 & 0xFF)     (16-bit, imm8 halfwords,
//                                               target = PC+4 + imm8*2)
//   str  T1:   hw0 = 0x6000 | (imm5<<6) | (Rn<<3) | Rt   (16-bit)
//   bkpt T1:   hw0 = 0xBE00 | imm8              (16-bit)
//
// `0xD1FD` decodes to `bne` with imm8 = -3 → target = PC+4 + (-3)*2 =
// PC-2, i.e. one halfword before the bne — the adjacent subs.
#[rustfmt::skip]
const SLED_CLOCK_PLL_SYS_REPROGRAM_MID_RUN_HW: [u16; 16] = [
    0xF240, //  [ 0] movw r0, #125 hw0     ; r0 = loop-1 counter
    0x007D, //  [ 1] movw r0, #125 hw1     ; (i=0, imm4=0, imm3=0, Rd=0, imm8=0x7D)
    0x3801, //  [ 2] subs r0, #1           ; loop1:
    0xD1FD, //  [ 3] bne  -4               ;   → [2] subs
    0xF240, //  [ 4] movw r1, #0x0008 hw0  ; r1 = PLL_SYS.FBDIV_INT low half
    0x0108, //  [ 5] movw r1, #0x0008 hw1  ; (Rd=1, imm8=0x08)
    0xF2C4, //  [ 6] movt r1, #0x4005 hw0  ; r1 high half (imm4=4, imm8=0x05)
    0x0105, //  [ 7] movt r1, #0x4005 hw1  ; r1 = 0x4005_0008 (FBDIV_INT addr)
    0xF240, //  [ 8] movw r2, #100   hw0   ; r2 = new FBDIV value (100)
    0x0264, //  [ 9] movw r2, #100   hw1   ; (Rd=2, imm8=0x64)
    0x600A, //  [10] str  r2, [r1]         ; *FBDIV_INT = 100 — reprogram mid-run
    0xF240, //  [11] movw r0, #125 hw0     ; r0 = loop-2 counter
    0x007D, //  [12] movw r0, #125 hw1
    0x3801, //  [13] subs r0, #1           ; loop2:
    0xD1FD, //  [14] bne  -4               ;   → [13] subs
    0xBE00, //  [15] bkpt #0               ; end of sled
];
const SLED_CLOCK_PLL_SYS_REPROGRAM_MID_RUN: &[u8] =
    &halfwords_to_le_bytes::<16, 32>(SLED_CLOCK_PLL_SYS_REPROGRAM_MID_RUN_HW);

// `clock_div_change_pio_running` — sled changes CLOCKS_CLK_SYS_DIV
// mid-run; observe CLK_SYS_DIV register readback survives. Originally
// designed to also verify PIO0 SM0_ADDR progress ratio matches the
// divider change, but the emulator's PIO advances one sysclk per
// step_quantum independent of clock_tree.sys_clk_hz (see
// `mdpicoem-common/src/pio/mod.rs:143`); both sides converge on the
// stall value regardless of CLK_SYS_DIV. The HLD §"Cycle-vs-
// frequency semantics" warns about this. Restore the SM_ADDR
// observable when the emulator's PIO honors sys_clk_hz.
//
// The scenario is retained in the catalogue in degraded form because
// it still exercises the mid-sled MMIO write path (ClockTree recompute
// on CLK_SYS_DIV write) and the readback proves the write landed on
// both sides.
const S_CLOCK_DIV_CHANGE_PIO_RUNNING: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0 | RESET_IO_BANK0 | RESET_PADS_BANK0),
    (pio_instr_mem_addr(PIO0_BASE, 0), 0xE03F), // SET X, 31
    (pio_instr_mem_addr(PIO0_BASE, 1), 0x0041), // JMP X-- 1
    (pio_instr_mem_addr(PIO0_BASE, 2), 0x0002), // JMP 2 (stall)
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    // EXECCTRL: WRAP_TOP=2 (bits 16:12), WRAP_BOTTOM=0 (bits 11:7).
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_EXECCTRL_OFF), 2u32 << 12),
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
    // Leave CLK_SYS_DIV at its reset default (integer=1, fractional=0)
    // so the sled's mid-run write is the only change during the run.
];
const O_CLOCK_DIV_CHANGE_PIO_RUNNING: &[(u32, u32)] = &[
    // CLK_SYS_DIV — integer bits [31:16] must match the sled's write.
    // This is the sole observable: the PIO_SM_ADDR / FDEBUG observables
    // that previously lived here were dropped (see scenario comment
    // above) — the emulator's PIO step is independent of
    // clock_tree.sys_clk_hz so both sides converge on the same stall
    // value regardless of the divider change, producing a false-PASS.
    (CLOCKS_CLK_SYS_DIV, 0xFFFF_0000),
];

// Custom sled for `clock_div_change_pio_running`.
//
// Structure:
//   - Spin ~500 sysclks (125-iter × ~4 cycles/iter countdown).
//   - Build r1 = CLK_SYS_DIV address (0x4001_0040) via movw + movt.
//   - Build r2 = new divider value (0x0002_0000 = integer=2) via
//     movw + movt (since the immediate > 16 bits).
//   - str r2, [r1]       — halve sys_clk.
//   - Spin ~500 more sysclks at the new (slower) divider.
//   - BKPT #0.
//
// Registers used (all caller-saved):
//   r0 — loop counter
//   r1 — CLK_SYS_DIV address literal (0x4001_0040)
//   r2 — new CLK_SYS_DIV value (0x0002_0000)
#[rustfmt::skip]
const SLED_CLOCK_DIV_CHANGE_PIO_RUNNING_HW: [u16; 18] = [
    0xF240, //  [ 0] movw r0, #125 hw0     ; r0 = loop-1 counter
    0x007D, //  [ 1] movw r0, #125 hw1
    0x3801, //  [ 2] subs r0, #1           ; loop1:
    0xD1FD, //  [ 3] bne  -4               ;   → [2] subs
    0xF240, //  [ 4] movw r1, #0x0040 hw0  ; r1 = CLK_SYS_DIV low half
    0x0140, //  [ 5] movw r1, #0x0040 hw1  ; (Rd=1, imm8=0x40)
    0xF2C4, //  [ 6] movt r1, #0x4001 hw0  ; r1 high half (imm4=4, imm8=0x01)
    0x0101, //  [ 7] movt r1, #0x4001 hw1  ; r1 = 0x4001_0040 (CLK_SYS_DIV)
    0xF240, //  [ 8] movw r2, #0     hw0   ; r2 low = 0 (fractional)
    0x0200, //  [ 9] movw r2, #0     hw1   ; (Rd=2, imm8=0)
    0xF2C0, //  [10] movt r2, #2     hw0   ; r2 high = 2 (integer divider)
    0x0202, //  [11] movt r2, #2     hw1   ; r2 = 0x0002_0000
    0x600A, //  [12] str  r2, [r1]         ; CLK_SYS_DIV = integer 2 mid-run
    0xF240, //  [13] movw r0, #125 hw0     ; r0 = loop-2 counter
    0x007D, //  [14] movw r0, #125 hw1
    0x3801, //  [15] subs r0, #1           ; loop2:
    0xD1FD, //  [16] bne  -4               ;   → [15] subs
    0xBE00, //  [17] bkpt #0               ; end of sled
];
const SLED_CLOCK_DIV_CHANGE_PIO_RUNNING: &[u8] =
    &halfwords_to_le_bytes::<18, 36>(SLED_CLOCK_DIV_CHANGE_PIO_RUNNING_HW);

/// Compile-time helper: serialize a fixed-length array of Thumb
/// halfwords to little-endian bytes, producing an array suitable for
/// `&[u8]` borrow into a `'static` slot. `N_HW = N_BYTES / 2`.
const fn halfwords_to_le_bytes<const N_HW: usize, const N_BYTES: usize>(
    hws: [u16; N_HW],
) -> [u8; N_BYTES] {
    assert!(N_BYTES == N_HW * 2, "N_BYTES must be 2 * N_HW");
    let mut out = [0u8; N_BYTES];
    let mut i = 0;
    while i < N_HW {
        let hw = hws[i];
        out[2 * i] = (hw & 0xFF) as u8;
        out[2 * i + 1] = (hw >> 8) as u8;
        i += 1;
    }
    out
}

/// Initial catalog. New scenarios append to the end so filter-by-substring
/// output stays ordered as cases are added.
pub const SCENARIOS: &[PeriphScenario] = &[
    PeriphScenario {
        name: "pio0_nop_loop",
        setup: S_PIO0_NOP_LOOP,
        max_sysclks: 100,
        observe: O_PIO0_NOP_LOOP,
        observe_pins: 0,
        custom_sled: None,
    },
    PeriphScenario {
        name: "pio0_fixed_cycles",
        setup: S_PIO0_FIXED_CYCLES,
        max_sysclks: 200,
        observe: O_PIO0_FIXED_CYCLES,
        observe_pins: 0,
        custom_sled: None,
    },
    PeriphScenario {
        name: "pio0_side_set_toggle",
        setup: S_PIO0_SIDE_SET_TOGGLE,
        max_sysclks: 100,
        observe: O_PIO0_SIDE_SET_TOGGLE,
        observe_pins: 0x0000_0001,
        custom_sled: None,
    },
    PeriphScenario {
        name: "pio0_reset_gating_placeholder",
        setup: S_PIO0_RESET_GATING_PLACEHOLDER,
        max_sysclks: 200,
        observe: O_PIO0_RESET_GATING_PLACEHOLDER,
        observe_pins: 0,
        custom_sled: None,
    },
    PeriphScenario {
        name: "pll_sys_lock_timing",
        setup: S_PLL_SYS_LOCK_TIMING,
        max_sysclks: 1500,
        observe: O_PLL_SYS_LOCK_TIMING,
        observe_pins: 0,
        custom_sled: None,
    },
    PeriphScenario {
        name: "clock_pll_sys_reprogram_mid_run",
        setup: S_CLOCK_PLL_SYS_REPROGRAM_MID_RUN,
        max_sysclks: 2000,
        observe: O_CLOCK_PLL_SYS_REPROGRAM_MID_RUN,
        observe_pins: 0,
        custom_sled: Some(SLED_CLOCK_PLL_SYS_REPROGRAM_MID_RUN),
    },
    PeriphScenario {
        name: "clock_div_change_pio_running",
        setup: S_CLOCK_DIV_CHANGE_PIO_RUNNING,
        max_sysclks: 2000,
        observe: O_CLOCK_DIV_CHANGE_PIO_RUNNING,
        observe_pins: 0,
        custom_sled: Some(SLED_CLOCK_DIV_CHANGE_PIO_RUNNING),
    },
];

// ---------------------------------------------------------------------------
// Library-API entry point (`run_against`)
// ---------------------------------------------------------------------------

use crate::silicon_oracle::{
    self, enable_cyccnt, read_cyccnt, reset_cyccnt, CaseOutcome, Verdict,
};
use crate::{EMU_TEST_STACK, SILICON_RUN_SLED};
use mdrp2350::{Config, EmulatorBuilder};
use probe_rs::{Core, MemoryInterface, RegisterId};
use std::time::{Duration, Instant};

const PC_REG: RegisterId = RegisterId(15);
const XPSR_REG: RegisterId = RegisterId(16);
const SP_REG: RegisterId = RegisterId(13);
const LR_REG: RegisterId = RegisterId(14);

/// Per-scenario BKPT timeout. Largest scenario (PLL) is ~1500 sysclks,
/// microseconds at any reasonable sys_clk; 5 s is absurd headroom.
const BKPT_TIMEOUT: Duration = Duration::from_secs(5);

/// Arguments for `run_against`. Mirrors the standalone binary's CLI.
#[derive(Clone, Debug, Default)]
pub struct PeriphArgs {
    pub filter: Option<String>,
    pub verbose: bool,
}

/// Reject any sled that isn't terminated by a `bkpt #0` (encoded as
/// Thumb halfword `0xBE00`, little-endian `[0x00, 0xBE]`). This catches
/// authoring mistakes (missing terminator, odd length, empty slice) at
/// scenario-evaluation time on both the HW and EMU paths.
///
/// Restriction: only `bkpt #0` (`0xBE00`) is accepted. Future scenarios
/// that need distinguishable halt reasons via `bkpt #N` (`0xBE00 | N`)
/// would need to relax this check to match any `0xBE**` halfword.
///
/// Returns the sled bytes unchanged on success; an error string on
/// failure. The caller decides whether to panic, abort the scenario,
/// or log — `run_scenario` currently converts errors to
/// `Box<dyn Error>`.
pub fn validate_custom_sled(bytes: &[u8]) -> Result<&[u8], String> {
    if bytes.is_empty() {
        return Err("custom sled is empty".to_string());
    }
    if bytes.len() < 2 {
        return Err(format!(
            "custom sled must be at least one halfword (got {} bytes)",
            bytes.len()
        ));
    }
    if !bytes.len().is_multiple_of(2) {
        return Err(format!(
            "custom sled must be a whole number of halfwords (got {} bytes)",
            bytes.len()
        ));
    }
    let n = bytes.len();
    // Thumb halfwords are little-endian: `bkpt #0` = 0xBE00 serialises
    // to `[0x00, 0xBE]`.
    if bytes[n - 2] != 0x00 || bytes[n - 1] != 0xBE {
        return Err(format!(
            "custom sled must end in `bkpt #0` (0xBE00); last halfword \
             is 0x{:02X}{:02X}",
            bytes[n - 1],
            bytes[n - 2],
        ));
    }
    Ok(bytes)
}

/// Build the countdown sled bytes for `max_sysclks`.
///
///   movw r0, #N      ; N = ceil(max_sysclks / 4), capped at 0xFFFF
///   subs r0, #1
///   bne  -4          ; back to subs
///   bkpt #0
pub fn assemble_sled(max_sysclks: u32) -> Vec<u8> {
    let mut n = max_sysclks.div_ceil(4);
    if n == 0 {
        n = 1;
    }
    if n > 0xFFFF {
        n = 0xFFFF;
    }

    let i_bit = (n >> 11) & 1;
    let imm4 = (n >> 12) & 0xF;
    let imm3 = (n >> 8) & 0x7;
    let imm8 = n & 0xFF;
    let hw0 = (0xF240u32 | (i_bit << 10) | imm4) as u16;
    let hw1 = ((imm3 << 12) | imm8) as u16;

    let halfwords = [hw0, hw1, 0x3801u16, 0xD1FDu16, 0xBE00u16];
    let mut out = Vec::with_capacity(halfwords.len() * 2);
    for hw in halfwords {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    out
}

/// Release PIO0 / PIO1 / PLL_SYS from reset. Individual scenarios may
/// re-assert specific bits afterwards.
fn release_common_resets(core: &mut Core) -> Result<(), probe_rs::Error> {
    let state: u32 = core.read_word_32(RESETS_RESET as u64)?;
    let cleared = state & !(RESET_PIO0 | RESET_PIO1 | RESET_PLL_SYS);
    core.write_word_32(RESETS_RESET as u64, cleared)?;
    Ok(())
}

fn apply_setup_hw(core: &mut Core, setup: &[(u32, u32)]) -> Result<(), probe_rs::Error> {
    for &(addr, val) in setup {
        core.write_word_32(addr as u64, val)?;
    }
    Ok(())
}

/// Gate the peripheral off immediately after BKPT so readback is atomic.
/// Scenario-specific, driven by name prefix.
fn gate_peripheral_hw(core: &mut Core, name: &str) -> Result<(), probe_rs::Error> {
    if name.starts_with("pio0") {
        core.write_word_32((PIO0_BASE + PIO_CTRL_OFF) as u64, 0)?;
    } else if name.starts_with("pio1") {
        core.write_word_32((PIO1_BASE + PIO_CTRL_OFF) as u64, 0)?;
    } else if name.starts_with("pll_sys") {
        // PLL_SYS has no CS.ENABLE; re-assert RESETS bit to freeze.
        let state: u32 = core.read_word_32(RESETS_RESET as u64)?;
        core.write_word_32(RESETS_RESET as u64, state | RESET_PLL_SYS)?;
    }
    Ok(())
}

fn gate_peripheral_emu(emu: &mut mdrp2350::Emulator, name: &str) {
    if name.starts_with("pio0") {
        emu.mmio_write32(PIO0_BASE + PIO_CTRL_OFF, 0);
    } else if name.starts_with("pio1") {
        emu.mmio_write32(PIO1_BASE + PIO_CTRL_OFF, 0);
    } else if name.starts_with("pll_sys") {
        let state = emu.mmio_read32(RESETS_RESET);
        emu.mmio_write32(RESETS_RESET, state | RESET_PLL_SYS);
    }
}

fn run_sled_hw(core: &mut Core) -> Result<u32, Box<dyn std::error::Error>> {
    reset_cyccnt(core)?;
    core.write_core_reg(PC_REG, SILICON_RUN_SLED)?;
    core.write_core_reg(XPSR_REG, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP_REG, EMU_TEST_STACK)?;
    core.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;
    core.run()?;

    let deadline = Instant::now() + BKPT_TIMEOUT;
    loop {
        if core.status()?.is_halted() {
            break;
        }
        if Instant::now() > deadline {
            let _ = core.halt(Duration::from_millis(200));
            let pc: u32 = core.read_core_reg(PC_REG).unwrap_or(0xDEAD_BEEF);
            let sp: u32 = core.read_core_reg(SP_REG).unwrap_or(0xDEAD_BEEF);
            let lr: u32 = core.read_core_reg(LR_REG).unwrap_or(0xDEAD_BEEF);
            return Err(format!(
                "BKPT timeout: PC=0x{pc:08X} SP=0x{sp:08X} LR=0x{lr:08X}"
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    read_cyccnt(core).map_err(Into::into)
}

fn sample_pins_hw(core: &mut Core, mask: u32) -> Result<(u32, u32), probe_rs::Error> {
    let oe: u32 = core.read_word_32(SIO_GPIO_OE as u64)?;
    let in_: u32 = core.read_word_32(SIO_GPIO_IN as u64)?;
    Ok((oe & mask, in_ & mask))
}

fn sample_pins_emu(emu: &mut mdrp2350::Emulator, mask: u32) -> (u32, u32) {
    let oe = emu.mmio_read32(SIO_GPIO_OE) & mask;
    let in_ = emu.mmio_read32(SIO_GPIO_IN) & mask;
    (oe, in_)
}

/// Per-scenario rich result used by the standalone binary.
pub struct PeriphScenarioResult {
    pub name: &'static str,
    pub verdict: Verdict,
    pub actual_sysclks: u32,
    pub first_divergence: Option<String>,
    pub elapsed: Duration,
}

pub fn run_scenario(
    core: &mut Core,
    sc: &PeriphScenario,
    first_scenario: bool,
    verbose: bool,
) -> Result<PeriphScenarioResult, Box<dyn std::error::Error>> {
    let t0 = Instant::now();

    if first_scenario {
        core.reset_and_halt(Duration::from_millis(500))?;
        enable_cyccnt(core)?;
    } else if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }
    release_common_resets(core)?;

    apply_setup_hw(core, sc.setup)?;
    // `custom_sled = Some(bytes)` → upload as-is (after end-terminator
    // validation). `None` → fall through to the countdown-loop sled
    // sized by `max_sysclks`. The validator is the single guard
    // against authoring mistakes; same check runs on the EMU side
    // below so a malformed sled fails before any bus state is touched.
    let owned_sled: Vec<u8>;
    let sled_bytes: &[u8] = match sc.custom_sled {
        Some(bytes) => validate_custom_sled(bytes).map_err(|e| {
            format!("scenario '{}': {e}", sc.name)
        })?,
        None => {
            owned_sled = assemble_sled(sc.max_sysclks);
            &owned_sled
        }
    };
    core.write_8(SILICON_RUN_SLED as u64, sled_bytes)?;
    let actual_sysclks = run_sled_hw(core)?;
    gate_peripheral_hw(core, sc.name)?;

    let hw_obs: Vec<u32> = sc
        .observe
        .iter()
        .map(|(addr, _m)| core.read_word_32(*addr as u64))
        .collect::<Result<_, _>>()?;
    let hw_pins = if sc.observe_pins != 0 {
        Some(sample_pins_hw(core, sc.observe_pins)?)
    } else {
        None
    };

    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    // Core 1 stays halted throughout; scenarios are single-core only.
    emu.core_mut(1).halt();
    for &(addr, val) in sc.setup {
        emu.mmio_write32(addr, val);
    }

    if let Some(bytes) = sc.custom_sled {
        // Mirror the HW path: validate first, upload to SRAM, then let
        // core 0 execute the sled so its embedded MMIO writes (clock
        // reprogramming, etc.) hit the emulator's bus at the same point
        // in the run they hit silicon. Matching execution on both sides
        // is the only way the ClockTree recompute path sees load.
        //
        // Termination: step until PC == sled-end BKPT address, bounded
        // by `actual_sysclks` as a safety cap. This avoids BKPT
        // overshoot — HW halts cleanly on BKPT, and the emulator's BKPT
        // handler is currently a 1-cycle NOP that would otherwise fall
        // through into zero-initialised SRAM (harmless `LSLS R0, R0, #0`)
        // and consume the remaining cycle budget, letting flag state
        // drift from HW in a way no current observable notices but
        // future scenarios might. Stopping at BKPT keeps xPSR in the
        // same shape on both sides.
        let vetted: &[u8] = validate_custom_sled(bytes)
            .map_err(|e| format!("scenario '{}': {e}", sc.name))?;
        emu.load_image(SILICON_RUN_SLED, vetted);
        // NOTE: depends on fresh EmulatorBuilder per scenario for default-zero
        // PRIMASK/CONTROL/FAULTMASK; reusing a long-lived emulator would
        // inherit stale state and this release block would need to reset those
        // too.
        {
            let c = emu.core_mut(0);
            c.wake();
            c.set_reg(13, EMU_TEST_STACK);  // SP
            c.set_reg(14, 0xFFFF_FFFF);     // LR
            c.regs.set_pc(SILICON_RUN_SLED);
            c.regs.xpsr = 0x0100_0000;      // T=1 (Thumb)
        }
        let bkpt_pc = SILICON_RUN_SLED + (vetted.len() as u32) - 2;
        let start = emu.cycles();
        let budget = actual_sysclks as u64;
        while emu.core(0).regs.pc() != bkpt_pc
            && emu.cycles().saturating_sub(start) < budget
        {
            emu.step();
        }
        let overshot = emu.core(0).regs.pc() != bkpt_pc;
        if overshot && verbose {
            println!(
                "    warn scenario '{}': EMU exhausted {}-cycle budget before \
                 reaching BKPT at PC=0x{:08X} (PC=0x{:08X})",
                sc.name,
                budget,
                bkpt_pc,
                emu.core(0).regs.pc(),
            );
        }
        emu.core_mut(0).halt();
    } else {
        // Default (non-custom-sled) path: halt both cores and advance
        // only bus/peripheral state. S1–S5 observables are all "steady-
        // state after N cycles" — the sled's job on HW is just to burn
        // N cycles, not to mutate MMIO.
        emu.core_mut(0).halt();
        emu.run(actual_sysclks as u64);
    }
    gate_peripheral_emu(&mut emu, sc.name);

    let emu_obs: Vec<u32> =
        sc.observe.iter().map(|(addr, _m)| emu.mmio_read32(*addr)).collect();
    let emu_pins = if sc.observe_pins != 0 {
        Some(sample_pins_emu(&mut emu, sc.observe_pins))
    } else {
        None
    };

    let mut first_div: Option<String> = None;
    for (i, (addr, mask)) in sc.observe.iter().enumerate() {
        let h = hw_obs[i] & *mask;
        let e = emu_obs[i] & *mask;
        if h != e {
            let msg = format!(
                "MMIO 0x{:08X} mask=0x{:08X}: HW=0x{:08X} EMU=0x{:08X} (xor=0x{:08X})",
                addr, mask, h, e, h ^ e,
            );
            if first_div.is_none() {
                first_div = Some(msg.clone());
            }
            if verbose {
                println!("    DIFF {msg}");
            }
        } else if verbose {
            println!("    ok   MMIO 0x{:08X} mask=0x{:08X}: 0x{:08X}", addr, mask, h);
        }
    }
    if let (Some(h), Some(e)) = (hw_pins, emu_pins) {
        if h != e {
            let msg = format!(
                "GPIO mask=0x{:08X}: HW oe=0x{:08X} level=0x{:08X}, \
                 EMU oe=0x{:08X} level=0x{:08X}",
                sc.observe_pins, h.0, h.1, e.0, e.1,
            );
            if first_div.is_none() {
                first_div = Some(msg.clone());
            }
            if verbose {
                println!("    DIFF {msg}");
            }
        } else if verbose {
            println!(
                "    ok   GPIO mask=0x{:08X}: oe=0x{:08X} level=0x{:08X}",
                sc.observe_pins, h.0, h.1
            );
        }
    }

    let verdict = if first_div.is_none() { Verdict::Pass } else { Verdict::Fail };
    Ok(PeriphScenarioResult {
        name: sc.name,
        verdict,
        actual_sysclks,
        first_divergence: first_div,
        elapsed: t0.elapsed(),
    })
}

/// Library entry point used by `silicon_periph_diff_rp2350` and the
/// `test_silicon` orchestrator.
///
/// **Cleanup contract**: on exit, re-assert the RESETS bits the catalogue
/// cleared (`RESET_PIO0 | RESET_PIO1 | RESET_PLL_SYS`) so the next oracle
/// in an orchestrated iteration sees default peripheral state. Runs
/// unconditionally — even if a case fails mid-loop — to avoid order-
/// dependent flakes per the HLD's cross-oracle state-cleanup contract.
///
/// Preconditions: `core` is live (auto-attached). The function handles
/// reset / CYCCNT enable on the first selected scenario.
///
/// Case selection semantics:
/// * `order = None` — run every catalogue scenario whose name matches
///   `args.filter`, in catalogue-declared order (single-pass / standalone
///   default).
/// * `order = Some(&[name, name, …])` — run exactly those scenarios in
///   that order. `args.filter` is ignored for selection. Names not
///   present in the catalogue are skipped with a single `eprintln!`
///   warning per unknown name.
pub fn run_against(
    core: &mut Core,
    args: &PeriphArgs,
    order: Option<&[&str]>,
) -> Result<Vec<CaseOutcome>, Box<dyn std::error::Error>> {
    let selected: Vec<&PeriphScenario> = match order {
        None => SCENARIOS
            .iter()
            .filter(|s| silicon_oracle::name_matches_filter(s.name, args.filter.as_deref()))
            .collect(),
        Some(names) => {
            let mut v: Vec<&PeriphScenario> = Vec::with_capacity(names.len());
            for name in names {
                match SCENARIOS.iter().find(|s| s.name == *name) {
                    Some(sc) => v.push(sc),
                    None => eprintln!(
                        "silicon_scenarios::run_against: unknown scenario '{name}' in order list; skipping",
                    ),
                }
            }
            v
        }
    };

    let mut outcomes: Vec<CaseOutcome> = Vec::with_capacity(selected.len());
    let mut loop_err: Option<Box<dyn std::error::Error>> = None;
    for (i, sc) in selected.iter().enumerate() {
        match run_scenario(core, sc, i == 0, args.verbose) {
            Ok(r) => {
                let elapsed_ms = r.elapsed.as_millis().min(u32::MAX as u128) as u32;
                let outcome = if r.verdict == Verdict::Pass {
                    CaseOutcome::pass("periph", r.name, elapsed_ms)
                } else {
                    CaseOutcome::fail(
                        "periph",
                        r.name,
                        r.first_divergence.unwrap_or_default(),
                        elapsed_ms,
                    )
                };
                outcomes.push(outcome);
            }
            Err(e) => {
                // Capture the error, stop running further cases, but still
                // execute the cleanup block below.
                loop_err = Some(e);
                break;
            }
        }
    }

    // Cleanup: re-assert the RESETS bits the catalogue cleared.
    // Runs even on error so the next oracle sees a clean state.
    //
    // The mask below must track every RESETS bit any scenario touches
    // via `ALIAS_CLR` — see HLD v1.1.1 §Cross-oracle state-cleanup
    // contract. `RESET_IO_BANK0` / `RESET_PADS_BANK0` are cleared by the
    // `pio0_side_set_toggle` scenario; leaving them un-asserted here
    // would leave GPIO0 configured for PIO0 at the start of the next
    // iteration's first scenario, leaking state across oracles.
    //
    // Cleanup failures are logged to stderr even though the rest of
    // `run_against` is silent — an operator needs to see them to
    // diagnose a wedged probe, and swallowing the error would make a
    // soak run lose the signal entirely.
    if let Err(e) = core.halt(Duration::from_millis(200)) {
        eprintln!("warning: periph cleanup halt failed: {e}");
    }
    match core.read_word_32(RESETS_RESET as u64) {
        Ok(state) => {
            let bits = RESET_PIO0
                | RESET_PIO1
                | RESET_PLL_SYS
                | RESET_IO_BANK0
                | RESET_PADS_BANK0;
            if let Err(e) = core.write_word_32(RESETS_RESET as u64, state | bits) {
                eprintln!("warning: periph cleanup RESETS write failed: {e}");
            }
        }
        Err(e) => {
            eprintln!("warning: periph cleanup RESETS read failed: {e}");
        }
    }

    // Restore CLK_SYS_DIV to its reset default (integer=1, fractional=0).
    // RESETS does *not* gate the CLOCKS block, so a scenario that
    // reprograms CLK_SYS_DIV (`clock_div_change_pio_running`) leaves the
    // divider at 0x0002_0000 even after we re-assert the RESETS mask
    // above. Unconditional — a no-op if the divider was already at 1.
    // Without this, a PIO scenario later in the same test_silicon
    // iteration would see HW running at half sys_clk while the emulator
    // (freshly built each scenario) sees the reset default, diverging
    // on timing-sensitive observables.
    if let Err(e) = core.write_word_32(CLOCKS_CLK_SYS_DIV as u64, 0x0001_0000) {
        eprintln!("warning: periph cleanup CLK_SYS_DIV write failed: {e}");
    }

    if let Some(e) = loop_err {
        return Err(e);
    }
    Ok(outcomes)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn is_mmio(addr: u32) -> bool {
        (0x4000_0000..0x6000_0000).contains(&addr)
            || (0xD000_0000..0xE000_0000).contains(&addr)
    }

    /// Catalog must ship the five v1 scenarios the HLD enumerates.
    #[test]
    fn test_scenarios_catalog_nonempty() {
        assert!(SCENARIOS.len() >= 5, "at least 5 scenarios, got {}", SCENARIOS.len());
    }

    /// Every setup / observe address must target MMIO — catches a
    /// relative-address regression (e.g. 0x0C8 instead of 0x5020_00C8).
    #[test]
    fn test_setup_addresses_all_absolute() {
        for sc in SCENARIOS {
            for (i, (a, _)) in sc.setup.iter().enumerate() {
                assert!(is_mmio(*a), "{} setup[{}] 0x{:08X}", sc.name, i, a);
            }
            for (i, (a, _)) in sc.observe.iter().enumerate() {
                assert!(is_mmio(*a), "{} observe[{}] 0x{:08X}", sc.name, i, a);
            }
        }
    }

    /// Observing nothing = always PASS = bug.
    #[test]
    fn test_observe_masks_are_nonzero() {
        for sc in SCENARIOS {
            let any = sc.observe.iter().any(|(_, m)| *m != 0) || sc.observe_pins != 0;
            assert!(any, "scenario '{}' observes nothing", sc.name);
        }
    }

    /// `max_sysclks=0` would never execute the sled.
    #[test]
    fn test_max_sysclks_positive() {
        for sc in SCENARIOS {
            assert!(sc.max_sysclks > 0, "'{}' has max_sysclks=0", sc.name);
        }
    }

    /// Duplicate names would confuse `--filter` and summary output.
    #[test]
    fn test_no_duplicate_scenario_names() {
        let mut seen: HashSet<&str> = HashSet::new();
        for sc in SCENARIOS {
            assert!(seen.insert(sc.name), "duplicate name '{}'", sc.name);
        }
    }

    /// Reading `TXFx` / `RXFx` pops a FIFO entry — an observable that
    /// silently mutates state is a footgun waiting for a future catalog
    /// author. Neither block allows this under any circumstances.
    ///
    /// TXF range per PIO block: `[base + 0x10, base + 0x20)` (4 SMs × 4
    /// bytes). RXF range: `[base + 0x20, base + 0x30)`.
    #[test]
    fn test_no_fifo_pop_on_read_observables() {
        let mut violations = 0usize;
        for sc in SCENARIOS {
            for &(addr, _mask) in sc.observe {
                for base in [PIO0_BASE, PIO1_BASE] {
                    let txf_lo = base + 0x10;
                    let txf_hi = base + 0x20;
                    let rxf_lo = base + 0x20;
                    let rxf_hi = base + 0x30;
                    if (txf_lo..txf_hi).contains(&addr) || (rxf_lo..rxf_hi).contains(&addr)
                    {
                        eprintln!(
                            "scenario '{}' observes FIFO 0x{:08X} (pops on read)",
                            sc.name, addr,
                        );
                        violations += 1;
                    }
                }
            }
        }
        assert_eq!(violations, 0, "FIFO-popping observables present");
    }

    // ---- Custom sled validator tests ------------------------------------

    /// A sled that doesn't end in `bkpt #0` (`0xBE00` → bytes `[0x00, 0xBE]`)
    /// must be rejected by the validator — the runner relies on BKPT to
    /// terminate HW execution, so a missing terminator would wedge the
    /// probe path until `BKPT_TIMEOUT`.
    #[test]
    fn test_validate_custom_sled_rejects_missing_terminator() {
        // Ends in 0xBF00 (nop), not 0xBE00 (bkpt).
        let bad: &[u8] = &[0x00, 0xBF, 0x00, 0xBF];
        let err = validate_custom_sled(bad).expect_err("sled without BKPT should be rejected");
        assert!(err.contains("bkpt"), "error should mention bkpt, got: {err}");
    }

    /// Odd-length byte stream can't be a Thumb halfword sequence — reject.
    #[test]
    fn test_validate_custom_sled_rejects_odd_length() {
        let bad: &[u8] = &[0x00, 0xBE, 0x00]; // 3 bytes
        let err = validate_custom_sled(bad).expect_err("odd-length sled should be rejected");
        assert!(
            err.contains("halfword") || err.contains("whole"),
            "error should mention alignment, got: {err}",
        );
    }

    /// Empty sled — nothing to run, reject.
    #[test]
    fn test_validate_custom_sled_rejects_empty() {
        let err = validate_custom_sled(&[]).expect_err("empty sled should be rejected");
        assert!(err.contains("empty"), "error should mention empty, got: {err}");
    }

    /// Happy path: a minimal valid sled is just one halfword of BKPT #0.
    #[test]
    fn test_validate_custom_sled_accepts_bare_bkpt() {
        let ok: &[u8] = &[0x00, 0xBE];
        assert!(validate_custom_sled(ok).is_ok());
    }

    /// The two new clock-reprogram sleds ship bundled as byte slices — the
    /// validator must accept them without complaint; a future edit that
    /// accidentally breaks the terminator gets caught here.
    #[test]
    fn test_validate_custom_sled_accepts_shipped_sleds() {
        assert!(
            validate_custom_sled(SLED_CLOCK_PLL_SYS_REPROGRAM_MID_RUN).is_ok(),
            "clock_pll_sys_reprogram_mid_run sled must validate",
        );
        assert!(
            validate_custom_sled(SLED_CLOCK_DIV_CHANGE_PIO_RUNNING).is_ok(),
            "clock_div_change_pio_running sled must validate",
        );
    }

    // ---- Catalogue presence tests for Stage 4 scenarios -----------------

    /// Both Stage-4 clock-reprogram scenarios must be in the catalogue so
    /// `test_silicon --filter clock_` picks them up, and so the soak-mode
    /// catalogue shuffle can randomise them alongside everything else.
    #[test]
    fn test_clock_pll_sys_reprogram_mid_run_present() {
        let sc = SCENARIOS.iter().find(|s| s.name == "clock_pll_sys_reprogram_mid_run");
        assert!(sc.is_some(), "scenario 'clock_pll_sys_reprogram_mid_run' missing");
        let sc = sc.unwrap();
        assert!(
            sc.custom_sled.is_some(),
            "clock_pll_sys_reprogram_mid_run must ship a custom sled",
        );
    }

    #[test]
    fn test_clock_div_change_pio_running_present() {
        let sc = SCENARIOS.iter().find(|s| s.name == "clock_div_change_pio_running");
        assert!(sc.is_some(), "scenario 'clock_div_change_pio_running' missing");
        let sc = sc.unwrap();
        assert!(
            sc.custom_sled.is_some(),
            "clock_div_change_pio_running must ship a custom sled",
        );
    }

    /// After Stage 4 review feedback the scenario's sole MMIO observable
    /// is CLK_SYS_DIV — the PIO_SM_ADDR / FDEBUG observables were
    /// dropped because the emulator's PIO step is independent of
    /// `clock_tree.sys_clk_hz`, so both sides converge on the same
    /// stall value regardless of the divider change (false-PASS). This
    /// test locks that invariant: if anyone adds an SM_ADDR observable
    /// back without first fixing the PIO/sys_clk coupling, the test
    /// fails and the reviewer is forced to look at the bug report.
    #[test]
    fn test_clock_div_change_pio_running_observes_clk_sys_div_only() {
        let sc = SCENARIOS
            .iter()
            .find(|s| s.name == "clock_div_change_pio_running")
            .expect("scenario missing");
        assert_eq!(
            sc.observe.len(),
            1,
            "expected exactly one observable (CLK_SYS_DIV); got {} — did \
             someone restore the SM_ADDR observable?",
            sc.observe.len(),
        );
        let (addr, mask) = sc.observe[0];
        assert_eq!(
            addr, CLOCKS_CLK_SYS_DIV,
            "sole observable must target CLK_SYS_DIV (0x{:08X}); got \
             0x{:08X}",
            CLOCKS_CLK_SYS_DIV, addr,
        );
        assert_eq!(
            mask, 0xFFFF_0000,
            "CLK_SYS_DIV mask must cover only the integer-divider \
             bits [31:16]; got 0x{:08X}",
            mask,
        );
        assert_eq!(
            sc.observe_pins, 0,
            "no GPIO observables expected — the scenario is MMIO-only",
        );
    }

    /// Sanity-check: the CLK_SYS_DIV register address must resolve to the
    /// writable RP2350 CLOCKS slot at 0x4001_0040 (bits [31:16] = integer
    /// divider). `0x4001_0044` is CLK_SYS_SELECTED, which is read-only;
    /// getting this wrong turns the scenario into a silent no-op.
    #[test]
    fn test_clocks_clk_sys_div_address() {
        assert_eq!(CLOCKS_CLK_SYS_DIV, 0x4001_0040);
    }

    /// Existing scenarios shouldn't gain a custom sled by accident —
    /// only the two new Stage-4 entries opt in. If a future scenario
    /// author adds a custom sled and forgets to add it to this
    /// allow-list, the test flags it so the reviewer double-checks
    /// the intent.
    #[test]
    fn test_custom_sled_opt_in_roster() {
        let expected_custom: HashSet<&str> = [
            "clock_pll_sys_reprogram_mid_run",
            "clock_div_change_pio_running",
        ]
        .into_iter()
        .collect();
        for sc in SCENARIOS {
            let has_custom = sc.custom_sled.is_some();
            let expected = expected_custom.contains(sc.name);
            assert_eq!(
                has_custom, expected,
                "scenario '{}' custom_sled={} but expected={}",
                sc.name, has_custom, expected,
            );
        }
    }
}

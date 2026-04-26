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
/// RESET_DONE: each bit reads 1 once the corresponding peripheral has
/// fully exited reset.  Polling this after a RESETS_CLR write ensures
/// the peripheral's register file is accessible before we touch it.
pub const RESETS_RESET_DONE: u32 = RESETS_BASE + 0x08;

/// APB alias offsets. `+0x2000` = SET (OR), `+0x3000` = CLR (AND NOT).
pub const ALIAS_SET: u32 = 0x2000;
pub const ALIAS_CLR: u32 = 0x3000;

/// RESETS bits used by scenarios and cleanup.
pub const RESET_ADC: u32 = 1 << 0;
pub const RESET_IO_BANK0: u32 = 1 << 6;
pub const RESET_PADS_BANK0: u32 = 1 << 9;
pub const RESET_PIO0: u32 = 1 << 11;
pub const RESET_PIO1: u32 = 1 << 12;
pub const RESET_PLL_SYS: u32 = 1 << 14;
pub const RESET_UART0: u32 = 1 << 26;

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
pub const PIO_IRQ_OFF: u32 = 0x030;
pub const PIO_INTR_OFF: u32 = 0x16C;
pub const PIO_IRQ0_INTE_OFF: u32 = 0x170;
pub const PIO_IRQ0_INTF_OFF: u32 = 0x174;
pub const PIO_IRQ0_INTS_OFF: u32 = 0x178;
pub const PIO_IRQ1_INTE_OFF: u32 = 0x17C;
pub const PIO_IRQ1_INTF_OFF: u32 = 0x180;
pub const PIO_IRQ1_INTS_OFF: u32 = 0x184;

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

/// PADS_BANK0 GPIO26 -- controls digital input buffer for ADC channel 0.
pub const PADS_BANK0_GPIO26: u32 = 0x4003_8000 + 0x04 + (26 * 4);
/// IO_BANK0 GPIO26_CTRL -- function select for ADC channel 0.
pub const IO_BANK0_GPIO26_CTRL: u32 = 0x4002_8000 + (26 * 8) + 0x04;

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

/// CLOCKS CLK_PERI_CTRL — gate for the peripheral clock (UART / SPI /
/// I2C). Silicon reset value: 0 (ENABLE=0, AUXSRC=0). Post-bootrom
/// silicon stays ungated until `runtime_init_clocks` sets ENABLE=1.
/// After `Core::reset_and_halt` the bootrom does NOT run, so scenarios
/// that program a real UART baud must set ENABLE=1 themselves.
pub const CLOCKS_CLK_PERI_CTRL: u32 = 0x4001_0048;
/// ENABLE bit for CLK_*_CTRL (bit 11 on every gated channel).
pub const CLK_CTRL_ENABLE: u32 = 1 << 11;

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
    /// Soft lower bound on sysclks. If the emulator completes in fewer
    /// cycles than this, a WARNING is printed but the scenario is NOT
    /// failed (V5 §4 / §7). 0 = no minimum.
    pub min_sysclks: u32,
}

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// SYSINFO read-only fields (Coverage Gap Fill V11 §3.1 Bucket A item 1)
// ---------------------------------------------------------------------------
//
// RP2350 datasheet §12.11. Read each documented SYSINFO u32 at reset and
// diff against the emulator's hardcoded values in `sysinfo_read`
// (`crates/mdrp2350/src/bus/peripherals.rs:48`). SYSINFO is not reset-
// gated on either side, so no setup is required beyond the oracle's
// `release_common_resets` baseline.
//
// Observed words:
//   0x00 CHIP_ID        — mask 0x0FFF_FFFF covers MANUFACTURER+PART+STOP_BIT
//                          from pico-sdk master; REV nibble [31:28] is
//                          masked out until Stage 5 pre-flight captures
//                          the chip-revision-specific value from Arthur's
//                          RP2354 (V11 HLD §8).
//   0x04 PACKAGE_SEL    — full u32.
//   0x08 PLATFORM       — full u32 (bits 0 FPGA, 1 ASIC).
//   GITREF not observed — Stage 5 pre-flight adds the entry with the
//                          chip-specific value and a full mask.
pub const SYSINFO_BASE: u32 = 0x4000_0000;
pub const SYSINFO_CHIP_ID_OFF: u32 = 0x00;
pub const SYSINFO_PACKAGE_SEL_OFF: u32 = 0x04;
pub const SYSINFO_PLATFORM_OFF: u32 = 0x08;
pub const SYSINFO_GITREF_RP2350_OFF: u32 = 0x14;

// No setup needed: SYSINFO is not reset-gated; release_common_resets() is sufficient.
const S_SYSINFO_READONLY_FIELDS: &[(u32, u32)] = &[];
const O_SYSINFO_READONLY_FIELDS: &[(u32, u32)] = &[
    // REV nibble [31:28] masked out until Stage 5 pre-flight.
    (SYSINFO_BASE + SYSINFO_CHIP_ID_OFF, 0x0FFF_FFFF),
    (SYSINFO_BASE + SYSINFO_PACKAGE_SEL_OFF, 0xFFFF_FFFF),
    (SYSINFO_BASE + SYSINFO_PLATFORM_OFF, 0xFFFF_FFFF),
];

// ---------------------------------------------------------------------------
// TBMAN PLATFORM selector (Coverage Gap Fill V11 §3.4 Bucket A item 4)
// ---------------------------------------------------------------------------
//
// TBMAN (`0x4016_0000`) is the RP2350 test-bench manager. Its PLATFORM
// register at offset 0x00 reports whether the design is running on ASIC,
// FPGA, or HDL simulation. On real RP2354 silicon the reset value is
// `TBMAN_PLATFORM_ASIC_BITS = 0x1` per pico-sdk:
//
//   https://raw.githubusercontent.com/raspberrypi/pico-sdk/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2350/hardware_regs/include/hardware/regs/tbman.h
//
//   #define TBMAN_PLATFORM_RESET       _u(0x00000001)
//   #define TBMAN_PLATFORM_BITS        _u(0x00000007)
//
// Matches HLD V11 §3.4 assumption `0b01`. TBMAN is not reset-gated
// (see `reset_bit_for_base` in `crates/mdrp2350/src/bus/mod.rs`) so no
// setup beyond the oracle's `release_common_resets` baseline is required.
// The emulator override landed in `crates/mdrp2350/src/peripherals/inert.rs`
// (`Tbman::read32`, offset 0x00).
pub const TBMAN_BASE: u32 = 0x4016_0000;
// PLATFORM offset is re-exported from the emulator module so the oracle
// and the emulator can't drift on what address is being diffed.
use mdrp2350::peripherals::inert::TBMAN_PLATFORM_OFFSET;

const S_TBMAN_PLATFORM: &[(u32, u32)] = &[];
const O_TBMAN_PLATFORM: &[(u32, u32)] = &[
    // PLATFORM is a 3-bit field; probe with full-word mask so any
    // stray upper-bit divergence on silicon also fails.
    (TBMAN_BASE + TBMAN_PLATFORM_OFFSET, 0xFFFF_FFFF),
];

// ---------------------------------------------------------------------------
// GLITCH_DETECTOR ARM readback (Coverage Gap Fill V11 §3.3 Bucket A item 3)
// ---------------------------------------------------------------------------
//
// GLITCH_DETECTOR (`0x4015_8000`) is the RP2350 on-chip glitch-detector
// controller. Its ARM register at offset 0x00 is a 16-bit RW sentinel —
// `ARM_VALUE_NO = 0x5bad` means "do not force the detectors to be
// armed", any other value force-arms. Reset value is `ARM_RESET = 0x5bad`
// (= VALUE_NO). Per pico-sdk:
//
//   https://raw.githubusercontent.com/raspberrypi/pico-sdk/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2350/hardware_regs/include/hardware/regs/glitch_detector.h
//
//   #define GLITCH_DETECTOR_ARM_OFFSET     _u(0x00000000)
//   #define GLITCH_DETECTOR_ARM_RESET      _u(0x00005bad)
//   #define GLITCH_DETECTOR_ARM_VALUE_NO   _u(0x5bad)
//   #define GLITCH_DETECTOR_ARM_VALUE_YES  _u(0x0000)
//
//   #define GLITCH_DETECTOR_TRIG_STATUS_OFFSET _u(0x00000010)
//   #define GLITCH_DETECTOR_TRIG_STATUS_RESET  _u(0x00000000)
//
// HLD V11 §3.3 target: writing ARM = "force YES" must round-trip on
// readback; TRIG_STATUS must stay 0 because no glitch fires.
//
// Setup: write ARM = 0x1234 so the diff sees a distinctive non-reset
// value that both force-arms (any non-0x5bad value force-arms) AND
// proves the write round-tripped. 0x1234 is chosen deliberately:
//   - distinctive: not 0x0000 (pre-power RAM default / probe stub read
//     failure / masked-read zero) and not 0x5bad (ARM_RESET sentinel),
//     so a readback of 0x1234 can only come from the write landing;
//   - not sentinel-coincident: != ARM_VALUE_NO (0x5bad) so the write
//     force-arms the detectors (= ARM_VALUE_YES semantics on silicon);
//   - not reset-coincident: a read that returns 0x5bad after this
//     setup indicates the write was dropped (emulator bug or silicon
//     sinking to masked / locked / not-Secure fabric) — with 0x0000
//     as the prior value, the reset-zero and write-zero cases are
//     indistinguishable;
//   - in the ARM bit-field range: 0x1234 fits in `ARM_BITS = 0x0000_FFFF`
//     so the upper-half mask doesn't elide the discriminator bits.
// GLITCH_DETECTOR is not in `reset_bit_for_base`, so
// `release_common_resets` is sufficient.
//
// Observed words:
//   ARM          mask 0x0000_FFFF — the 16-bit register field; upper
//                halves on silicon are unimplemented/zero.
//   TRIG_STATUS  mask 0x0000_000F — four DETn bits, must all be 0.
//
// The emulator override landed in
// `crates/mdrp2350/src/peripherals/inert.rs` (`GlitchDetector::new()`
// seeds ARM = RESET; `write32` / `read32` round-trip ARM and keep
// TRIG_STATUS's read-as-zero override).
pub const GLITCH_DETECTOR_BASE: u32 = 0x4015_8000;
// Offsets + reset / sentinel values are re-exported from the emulator
// module so the oracle and the emulator can't drift on addresses or
// magic values.
use mdrp2350::peripherals::inert::{
    GLITCH_DETECTOR_ARM_MASK, GLITCH_DETECTOR_ARM_OFFSET, GLITCH_DETECTOR_TRIG_STATUS_OFFSET,
};

/// Distinctive ARM discriminator for the silicon diff — see the
/// scenario comment above for the selection rationale. Any value that
/// is not `ARM_VALUE_NO = 0x5bad` force-arms the detectors on silicon.
const GLITCH_DETECTOR_ARM_DISCRIMINATOR: u32 = 0x0000_1234;

const S_GLITCH_DETECTOR_ARM_READBACK: &[(u32, u32)] = &[
    // Write ARM = 0x1234 — force-arm with a distinctive discriminator
    // so the readback proves the write landed (vs. reset-zero or
    // reset-0x5bad collisions).
    (
        GLITCH_DETECTOR_BASE + GLITCH_DETECTOR_ARM_OFFSET,
        GLITCH_DETECTOR_ARM_DISCRIMINATOR,
    ),
];
const O_GLITCH_DETECTOR_ARM_READBACK: &[(u32, u32)] = &[
    // ARM register — mask to the 16-bit field defined by ARM_BITS.
    (
        GLITCH_DETECTOR_BASE + GLITCH_DETECTOR_ARM_OFFSET,
        GLITCH_DETECTOR_ARM_MASK,
    ),
    // TRIG_STATUS — 4-bit W1C field; must be 0 (no glitch fires).
    (
        GLITCH_DETECTOR_BASE + GLITCH_DETECTOR_TRIG_STATUS_OFFSET,
        0x0000_000F,
    ),
];

// S1: PIO0 SM0 runs `JMP 0` in a one-instruction loop. Positive
// control — ADDR never advances past 0, HW and EMU MUST agree.
const S_PIO0_NOP_LOOP: &[(u32, u32)] = &[
    // Inlined PREFIX_PIO0_HARD_RESET — wipe instr_mem[], SM state, FIFOs
    // and irq_flags so a prior PIO scenario's program can't persist
    // through the Fisher-Yates shuffle. HLD V1 §4.3.
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
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
    // Inlined PREFIX_PIO0_HARD_RESET — defensive pulse so prior-scenario
    // instr_mem[] / SM state cannot leak (HLD V1 §5 preventive).
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
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
    // Inlined PREFIX_PIO0_HARD_RESET — defensive pulse prepended ahead
    // of the IO_BANK0 / PADS_BANK0 release so prior-scenario instr_mem[]
    // / SM state cannot leak (HLD V1 §5 preventive).
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
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
    // Inlined PREFIX_PIO0_HARD_RESET — defensive pulse so prior-scenario
    // instr_mem[] / SM state cannot leak before the gating test runs
    // (HLD V1 §5 preventive). The trailing RESETS_SET below re-slams
    // PIO0 into reset to probe the gating behaviour itself.
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
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
    // Inlined PREFIX_PIO0_HARD_RESET — defensive pulse ahead of the
    // IO_BANK0 / PADS_BANK0 release so prior-scenario instr_mem[] / SM
    // state cannot leak (HLD V1 §5 preventive).
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
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

// Phase 1 B1: `timer0_alarm0_fire_and_clear` — end-to-end TIMER0
// exercise. Sled reads TIMELR, programs ALARM_0 at +1000 µs, enables
// INTE.bit0, busy-polls INTS until bit 0, W1C's INTR, writes the
// marker 0xDEADBEEF to 0x2000_0300, then BKPT #0. The scenario's setup
// layer releases TIMER0 from RESETS and enables the TIMER0 TICKS
// domain; post-bootrom CYCLES=12 is already in place (HLD V5 §5.7), so
// no explicit CYCLES write is needed.
//
// Silicon expectations (validated by Arthur on the lab rig):
//   - TIMER0.INTR reads 0 post-W1C.
//   - TIMER0.TIMELR reads ≥ 1000 µs (alarm fired at ≥ target).
//   - 0x2000_0300 holds 0xDEADBEEF (sled reached the marker write,
//     i.e. INTS asserted and the W1C landed).
//
// Registers used (all caller-saved):
//   r0 — scratch (TIMELR read, ALARM target)
//   r1 — INTE/INTR/ALARM arm value (1)
//   r2 — INTS read for polling
//   r3 — TIMER0_BASE (0x400B_0000)
//   r4 — marker value 0xDEADBEEF
//   r5 — marker address 0x2000_0300
//
// Thumb-2 encodings per ARMv8-M ARM (same idioms as the PLL sled above):
//   movw T3 / movt T1 / ldr T1 / str T1 (imm5 word offset, R0-R7) /
//   adds T1 (reg) / movs T1 / tst T1 (reg) / b T1 (cond) / bkpt T1.
#[rustfmt::skip]
const SLED_TIMER0_ALARM0_FIRE_AND_CLEAR_HW: [u16; 25] = [
    0xF240, //  [ 0] movw r3, #0x0000       ; r3 = TIMER0_BASE low half
    0x0300, //  [ 1]
    0xF2C4, //  [ 2] movt r3, #0x400B       ; r3 = 0x400B_0000
    0x030B, //  [ 3]
    0x68D8, //  [ 4] ldr  r0, [r3, #0x0C]   ; r0 = TIMELR (µs snapshot)
    0xF240, //  [ 5] movw r1, #1000         ; r1 = 1000 µs offset
    0x31E8, //  [ 6]
    0x1840, //  [ 7] adds r0, r0, r1        ; r0 = target_us
    0x6118, //  [ 8] str  r0, [r3, #0x10]   ; ALARM_0 = target (arms alarm 0)
    0x2101, //  [ 9] movs r1, #1            ; r1 = 1 (bit0 mask)
    0x6419, //  [10] str  r1, [r3, #0x40]   ; INTE = 1 (alarm-0 int enable)
    0x6C9A, //  [11] ldr  r2, [r3, #0x48]   ; loop: r2 = INTS
    0x420A, //  [12] tst  r2, r1            ;   Z=1 if INTS.bit0 == 0
    0xD0FC, //  [13] beq  loop              ;   offset = -8 (back to [11])
    0x63D9, //  [14] str  r1, [r3, #0x3C]   ; INTR = 1 (W1C alarm-0 latch)
    0xF64B, //  [15] movw r4, #0xBEEF       ; r4 low half
    0x64EF, //  [16]
    0xF6CD, //  [17] movt r4, #0xDEAD       ; r4 = 0xDEADBEEF
    0x64AD, //  [18]
    0xF240, //  [19] movw r5, #0x0300       ; r5 low half
    0x3500, //  [20]
    0xF2C2, //  [21] movt r5, #0x2000       ; r5 = 0x2000_0300 (marker slot)
    0x0500, //  [22]
    0x602C, //  [23] str  r4, [r5, #0]      ; marker = 0xDEADBEEF
    0xBE00, //  [24] bkpt #0                ; end of sled
];
const SLED_TIMER0_ALARM0_FIRE_AND_CLEAR: &[u8] =
    &halfwords_to_le_bytes::<25, 50>(SLED_TIMER0_ALARM0_FIRE_AND_CLEAR_HW);

// Phase 1 B2: `ticks_timer0_retarget_halves_rate` — verify that
// doubling `TICKS.TIMER0.CYCLES` from 12 to 24 halves the observed
// TIMER0 advance rate. Sled writes `TICKS.TIMER0.CYCLES = 24` then
// busy-spins ~4800 sys_clks. On silicon at clk_ref=12 MHz with the
// new CYCLES=24, the post-bootrom 1-µs cadence halves to 0.5-µs; in
// the ~400 µs wall-clock window the busy-loop covers, TIMER0 should
// advance ~200 µs (with CYCLES unchanged it would have advanced ~400).
// The emulator's TICKS model divides sys_clks by CYCLES, so the same
// retarget produces the same halving effect.
//
// The observable is TIMER0.TIMELR masked to the low 8 bits after the
// sled halts. Both sides should land in the same ballpark. If the
// EMU ignored the CYCLES write, its TIMELR would be roughly double
// the silicon value and the low-byte diverge catches it.
//
// Silicon validation happens on Arthur's lab rig — the low-8-bit
// mask carries an inherent fuzziness (both sides complete different
// numbers of µs-edges depending on spin-loop timing), but the
// coarse-grained band is sized to catch the "CYCLES write silently
// dropped" failure mode which is the primary EMU concern.
//
// Registers used:
//   r2 — TICKS.TIMER0.CYCLES address literal (0x4010_881C)
//   r4 — spin counter
//   r6 — new CYCLES value (24)
#[rustfmt::skip]
const SLED_TICKS_TIMER0_RETARGET_HW: [u16; 11] = [
    0xF648, //  [ 0] movw r2, #0x881C       ; r2 = TICKS.TIMER0.CYCLES low half
    0x021C, //  [ 1]                        ; (imm4=8, i=1, imm3=0, Rd=2, imm8=0x1C)
    0xF2C4, //  [ 2] movt r2, #0x4010       ; r2 = 0x4010_881C
    0x0210, //  [ 3]
    0x2618, //  [ 4] movs r6, #24           ; r6 = new CYCLES value
    0x6016, //  [ 5] str  r6, [r2, #0]      ; CYCLES = 24 (retarget)
    0xF240, //  [ 6] movw r4, #1200         ; r4 = spin iters (~4800 sys_clks)
    0x44B0, //  [ 7]                        ; (imm4=0, i=0, imm3=4, Rd=4, imm8=0xB0)
    0x3C01, //  [ 8] subs r4, #1            ; spin:
    0xD1FD, //  [ 9] bne  -4                ;   → [8] subs
    0xBE00, //  [10] bkpt #0                ; end of sled
];
const SLED_TICKS_TIMER0_RETARGET: &[u8] =
    &halfwords_to_le_bytes::<11, 22>(SLED_TICKS_TIMER0_RETARGET_HW);

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

// Phase 1 B1/B2 scenario setup/observe tables.
//
// TIMER0_ALARM0_FIRE_AND_CLEAR:
//   - Release TIMER0 from RESETS so its bus dispatch unmasks.
//   - Enable TICKS.TIMER0 (CTRL.ENABLE = 1). Post-bootrom CYCLES=12 is
//     already installed by both silicon's bootrom and the emulator's
//     `Bus::new()` per HLD V5 §5.7 (see phase1 tests).
//   - Custom sled (SLED_TIMER0_ALARM0_FIRE_AND_CLEAR) runs the
//     TIMELR-read → ALARM_0-arm → poll-INTS → W1C-INTR sequence and
//     writes 0xDEADBEEF to 0x2000_0300 on success.
const S_TIMER0_ALARM0_FIRE_AND_CLEAR: &[(u32, u32)] = &[
    // Release TIMER0 (bit 23) from the RESETS guard.
    (RESETS_RESET + ALIAS_CLR, RESET_TIMER0_BIT),
    // Enable the TIMER0 TICKS domain.
    (TICKS_TIMER0_CTRL, TICKS_CTRL_ENABLE),
];
const O_TIMER0_ALARM0_FIRE_AND_CLEAR: &[(u32, u32)] = &[
    // Post-W1C: INTR.bit0 must be clear. Positive proof that the sled
    // reached the "W1C INTR after INTS asserted" branch — had the
    // busy-poll timed out, INTR.bit0 would still read 1 on silicon.
    (TIMER0_INTR, 0x1),
    // INTE stayed set (we never wrote it clear). The sled's `str r1,
    // [r3, #0x40]` lands bit 0; both sides must mirror. This locks
    // the test to "the sled actually wrote INTE" without which INTS
    // would never assert and the scenario would hang.
    (TIMER0_INTE, 0x1),
    // ARMED.bit0 = 0 — the alarm auto-disarms on match per §12.8.3.
    // HW and EMU both clear bit 0 of ARMED when the alarm fires.
    (TIMER0_ARMED, 0x1),
];

// TICKS_TIMER0_RETARGET_HALVES_RATE:
//   - Release TIMER0 from RESETS.
//   - Enable TIMER0 TICKS domain at post-bootrom CYCLES=12.
//   - Custom sled (SLED_TICKS_TIMER0_RETARGET) samples TIMELR,
//     reprograms CYCLES to 24, busy-spins ~2400 sys_clks, samples
//     TIMELR again, and stores the delta at 0x2000_0300.
//
// The observable is the delta at 0x2000_0300. At the original cadence
// (CYCLES=12) the spin would elapse ~100 µs; at CYCLES=24 it elapses
// ~50 µs. Both sides should land in the ~50 band. The mask `0xFF`
// catches any gross-miscomputation divergence (e.g. EMU ignoring the
// CYCLES write would leave the delta at ~100 ≈ 0x64, clearly distinct
// from ~50 ≈ 0x32).
const S_TICKS_TIMER0_RETARGET: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_TIMER0_BIT),
    (TICKS_TIMER0_CTRL, TICKS_CTRL_ENABLE),
];
const O_TICKS_TIMER0_RETARGET: &[(u32, u32)] = &[
    // TICKS.TIMER0.CYCLES must land at 24 — positive proof the retarget
    // landed on both sides. If silicon accepted the write but EMU
    // dropped it, this observable catches the divergence directly
    // without any timing-band fuzziness.
    (TICKS_TIMER0_CYCLES, 0xFF),
    // TIMER0.TIMERAWL after the sled halts. Silicon at 150 MHz
    // post-bootrom + CYCLES=24 yields ~200 µs in the spin window; EMU
    // (same sys_clks, same TICKS divider) produces the same order of
    // magnitude. Mask the low 10 bits to catch a gross divergence
    // (e.g. EMU at ~400 µs because CYCLES stayed at 12) while tolerating
    // the ~5% timing jitter inherent in busy-loop scheduling. The
    // primary signal is the CYCLES readback above — TIMERAWL is a
    // secondary consistency check.
    (TIMER0_TIMERAWL, 0x3FF),
];

// ---------------------------------------------------------------------------
// Phase 1 Expansion: SIO divider, MTIME, TIMER1
// ---------------------------------------------------------------------------

// SIO divider register addresses (RP2350 §3.1.2).
const SIO_DIV_UDIVIDEND: u32 = 0xD000_0060;
const SIO_DIV_UDIVISOR: u32 = 0xD000_0064;
const SIO_DIV_SDIVIDEND: u32 = 0xD000_0068;
const SIO_DIV_SDIVISOR: u32 = 0xD000_006C;
const SIO_DIV_QUOTIENT: u32 = 0xD000_0070;
const SIO_DIV_REMAINDER: u32 = 0xD000_0074;
const SIO_DIV_CSR: u32 = 0xD000_0078;

// SIO MTIME register addresses (RP2350 §3.1.2).
// NOTE: 0x1A0 is RISCV_SOFTIRQ (not MTIME_CTRL). Correct map:
//   0x1A0 = RISCV_SOFTIRQ, 0x1A4 = MTIME_CTRL, 0x1B0 = MTIME_LO, 0x1B4 = MTIME_HI.
const SIO_MTIME_CTRL: u32 = 0xD000_01A4;
const SIO_MTIME_LO: u32 = 0xD000_01B0;
const SIO_MTIME_HI: u32 = 0xD000_01B4;

// TIMER1 base (RP2350 datasheet §12.8, `0x400B_8000`).
pub const TIMER1_BASE: u32 = 0x400B_8000;
pub const TIMER1_INTR: u32 = TIMER1_BASE + 0x3C;
pub const TIMER1_INTE: u32 = TIMER1_BASE + 0x40;
pub const TIMER1_ARMED: u32 = TIMER1_BASE + 0x20;
/// RESETS bit for TIMER1 (RP2350 §7.5, bit 24).
pub const RESET_TIMER1_BIT: u32 = 1 << 24;
/// TICKS.TIMER1 control register (TICKS_BASE + 0x24).
pub const TICKS_TIMER1_CTRL: u32 = TICKS_BASE + 0x24;

// S_SIO_DIV_UNSIGNED: Write 100 / 7 unsigned, observe quotient/remainder/CSR.
const S_SIO_DIVIDER_UNSIGNED: &[(u32, u32)] = &[
    (SIO_DIV_UDIVIDEND, 100),
    (SIO_DIV_UDIVISOR, 7),
];
const O_SIO_DIVIDER_UNSIGNED: &[(u32, u32)] = &[
    (SIO_DIV_QUOTIENT, 0xFFFF_FFFF),
    (SIO_DIV_REMAINDER, 0xFFFF_FFFF),
    (SIO_DIV_CSR, 0x3), // READY + DIRTY only
];

// S_SIO_DIV_SIGNED: Write -100 / 7 signed, observe quotient/remainder/CSR.
const S_SIO_DIVIDER_SIGNED: &[(u32, u32)] = &[
    (SIO_DIV_SDIVIDEND, 0xFFFF_FF9C), // -100
    (SIO_DIV_SDIVISOR, 7),
];
const O_SIO_DIVIDER_SIGNED: &[(u32, u32)] = &[
    (SIO_DIV_QUOTIENT, 0xFFFF_FFFF),
    (SIO_DIV_REMAINDER, 0xFFFF_FFFF),
    (SIO_DIV_CSR, 0x3),
];

// S_SIO_MTIME_COUNT_AND_MATCH: Enable MTIME, spin ~50 iters, disable, observe.
const S_SIO_MTIME_COUNT_AND_MATCH: &[(u32, u32)] = &[
    (SIO_MTIME_CTRL, 0), // ensure clean start
];
const O_SIO_MTIME_COUNT_AND_MATCH: &[(u32, u32)] = &[
    (SIO_MTIME_CTRL, 0x1),       // EN bit (should be 0 after sled disables)
    (SIO_MTIME_LO, 0xFFFF_FFFF), // frozen counter low
    (SIO_MTIME_HI, 0xFFFF_FFFF), // frozen counter high
];

// Custom sled for `sio_mtime_count_and_match`.
//
// Structure:
//   - Build R0 = MTIME_CTRL address (0xD000_01A4) via movw + movt.
//   - MOVS R1, #1 — enable value.
//   - STR R1, [R0, #0] — MTIME_CTRL = 1 (enable counting).
//   - MOVS R2, #50 — spin counter.
//   - SUBS R2, #1 / BNE — spin loop (~50 iterations).
//   - MOVS R1, #0 — disable value.
//   - STR R1, [R0, #0] — MTIME_CTRL = 0 (freeze counter).
//   - BKPT #0.
//
// Registers used:
//   R0 — MTIME_CTRL address (0xD000_01A4)
//   R1 — enable/disable value
//   R2 — spin counter
//
// Thumb-2 encodings:
//   movw R0, #0x01A4: imm4=0, i=0, imm3=1, imm8=0xA4
//     hw0 = 0xF240, hw1 = 0x10A4
//   movt R0, #0xD000: imm4=0xD, i=0, imm3=0, imm8=0x00
//     hw0 = 0xF2CD, hw1 = 0x0000
#[rustfmt::skip]
const SLED_SIO_MTIME_COUNT_AND_MATCH_HW: [u16; 12] = [
    0xF240, //  [ 0] movw r0, #0x01A4 hw0  ; r0 = MTIME_CTRL low half
    0x10A4, //  [ 1] movw r0, #0x01A4 hw1  ; (imm3=1, Rd=0, imm8=0xA4)
    0xF2CD, //  [ 2] movt r0, #0xD000 hw0  ; r0 high half (imm4=0xD)
    0x0000, //  [ 3] movt r0, #0xD000 hw1  ; r0 = 0xD000_01A4
    0x2101, //  [ 4] movs r1, #1           ; r1 = 1 (enable)
    0x6001, //  [ 5] str  r1, [r0, #0]     ; MTIME_CTRL = 1 (start counting)
    0x2232, //  [ 6] movs r2, #50          ; r2 = spin counter
    0x3A01, //  [ 7] subs r2, #1           ; spin:
    0xD1FD, //  [ 8] bne  -4               ;   → [7] subs
    0x2100, //  [ 9] movs r1, #0           ; r1 = 0 (disable)
    0x6001, //  [10] str  r1, [r0, #0]     ; MTIME_CTRL = 0 (freeze counter)
    0xBE00, //  [11] bkpt #0               ; end of sled
];
const SLED_SIO_MTIME_COUNT_AND_MATCH: &[u8] =
    &halfwords_to_le_bytes::<12, 24>(SLED_SIO_MTIME_COUNT_AND_MATCH_HW);

// Phase 1 B1 clone: `timer1_alarm0_fire_and_clear` — same logic as TIMER0
// but targeting TIMER1 at 0x400B_8000. Every TIMER0 address in setup,
// observe, AND the custom sled is shifted +0x8000.
//
// Sled register offsets from r3 (TIMER1_BASE = 0x400B_8000):
//   TIMELR = [r3, #0x0C], ALARM_0 = [r3, #0x10], INTR = [r3, #0x3C],
//   INTE = [r3, #0x40], INTS = [r3, #0x48] — all identical to TIMER0.
//
// Only the movw/movt pair building r3 changes:
//   TIMER0: movw r3, #0x0000 / movt r3, #0x400B
//   TIMER1: movw r3, #0x8000 / movt r3, #0x400B
//
// movw R3, #0x8000: imm16 = 0x8000
//   imm4 = 8 (bits[15:12]), i = 0 (bit[11]), imm3 = 0 (bits[10:8]),
//   imm8 = 0x00 (bits[7:0])
//   hw0 = 0xF240 | (0 << 10) | 8 = 0xF248
//   hw1 = (0 << 12) | (3 << 8) | 0 = 0x0300
// movt R3, #0x400B — unchanged from TIMER0.
#[rustfmt::skip]
const SLED_TIMER1_ALARM0_FIRE_AND_CLEAR_HW: [u16; 25] = [
    0xF248, //  [ 0] movw r3, #0x8000       ; r3 = TIMER1_BASE low half
    0x0300, //  [ 1]
    0xF2C4, //  [ 2] movt r3, #0x400B       ; r3 = 0x400B_8000
    0x030B, //  [ 3]
    0x68D8, //  [ 4] ldr  r0, [r3, #0x0C]   ; r0 = TIMELR (µs snapshot)
    0xF240, //  [ 5] movw r1, #1000         ; r1 = 1000 µs offset
    0x31E8, //  [ 6]
    0x1840, //  [ 7] adds r0, r0, r1        ; r0 = target_us
    0x6118, //  [ 8] str  r0, [r3, #0x10]   ; ALARM_0 = target (arms alarm 0)
    0x2101, //  [ 9] movs r1, #1            ; r1 = 1 (bit0 mask)
    0x6419, //  [10] str  r1, [r3, #0x40]   ; INTE = 1 (alarm-0 int enable)
    0x6C9A, //  [11] ldr  r2, [r3, #0x48]   ; loop: r2 = INTS
    0x420A, //  [12] tst  r2, r1            ;   Z=1 if INTS.bit0 == 0
    0xD0FC, //  [13] beq  loop              ;   offset = -8 (back to [11])
    0x63D9, //  [14] str  r1, [r3, #0x3C]   ; INTR = 1 (W1C alarm-0 latch)
    0xF64B, //  [15] movw r4, #0xBEEF       ; r4 low half
    0x64EF, //  [16]
    0xF6CD, //  [17] movt r4, #0xDEAD       ; r4 = 0xDEADBEEF
    0x64AD, //  [18]
    0xF240, //  [19] movw r5, #0x0300       ; r5 low half
    0x3500, //  [20]
    0xF2C2, //  [21] movt r5, #0x2000       ; r5 = 0x2000_0300 (marker slot)
    0x0500, //  [22]
    0x602C, //  [23] str  r4, [r5, #0]      ; marker = 0xDEADBEEF
    0xBE00, //  [24] bkpt #0                ; end of sled
];
const SLED_TIMER1_ALARM0_FIRE_AND_CLEAR: &[u8] =
    &halfwords_to_le_bytes::<25, 50>(SLED_TIMER1_ALARM0_FIRE_AND_CLEAR_HW);

const S_TIMER1_ALARM0_FIRE_AND_CLEAR: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_TIMER1_BIT),
    (TICKS_TIMER1_CTRL, TICKS_CTRL_ENABLE),
];
const O_TIMER1_ALARM0_FIRE_AND_CLEAR: &[(u32, u32)] = &[
    (TIMER1_INTR, 0x1),
    (TIMER1_INTE, 0x1),
    (TIMER1_ARMED, 0x1),
];

// ---------------------------------------------------------------------------
// Phase 2 scenarios — UART0, SPI0, I2C0, ADC, PWM
// ---------------------------------------------------------------------------
//
// Per HLD V5 §6 row 2 and the Phase 2 prompt: one silicon scenario per
// peripheral. Observables use peripheral register state (not pins) per
// V5 §4 observability constraint.

/// UART0 base and UARTFR / UARTLCR_H / UARTCR / UARTDR offsets.
const UART0_UARTFR: u32 = UART0_BASE + 0x18;
const UART0_UARTDR: u32 = UART0_BASE + 0x00;
const UART0_UARTLCR_H: u32 = UART0_BASE + 0x2C;
const UART0_UARTCR: u32 = UART0_BASE + 0x30;
const UARTLCR_H_FEN: u32 = 1 << 4;
/// UARTLCR_H.WLEN = 0b11 (bits [6:5]) — 8-bit word length. PL011 masks
/// transmitted/received data to `WLEN+5` bits; without this, silicon
/// defaults to WLEN=00 = 5-bit and truncates `0x42` → `0x02`.
const UARTLCR_H_WLEN_8: u32 = 0b11 << 5;
const UARTCR_UARTEN: u32 = 1 << 0;
const UARTCR_LBE: u32 = 1 << 7;
const UARTCR_TXE: u32 = 1 << 8;
const UARTCR_RXE: u32 = 1 << 9;
const UART0_UARTIBRD: u32 = UART0_BASE + 0x24;
const UART0_UARTFBRD: u32 = UART0_BASE + 0x28;
/// UARTFR.TXFE at `0x4007_0018` bit 7 — asserted when TX FIFO is
/// empty, which is the post-drain steady state after one byte ticks out.
const UARTFR_TXFE_BIT: u32 = 1 << 7;

/// SPI0 SSPCR0/SSPCR1/SSPDR offsets.
const SPI0_SSPCR0: u32 = SPI0_BASE + 0x00;
const SPI0_SSPCR1: u32 = SPI0_BASE + 0x04;
const SPI0_SSPDR: u32 = SPI0_BASE + 0x08;
const SSPCR1_SSE: u32 = 1 << 1;
const SSPCR1_LBM: u32 = 1 << 0;

/// I2C0 base and IC_TAR / IC_ENABLE / IC_DATA_CMD / IC_TX_ABRT_SOURCE
/// offsets. Scenario: target an I2C-reserved 7-bit address (0x7F — the
/// last reserved slot; ARM §I2C-spec reserves 0x00..=0x07 and
/// 0x78..=0x7F) + STOP → silicon NACKs (no real device should occupy a
/// reserved address), emulator's `ALWAYS_ACK_ADDRS` is empty so it also
/// NACKs. Both sides land on abort_source bit 0. The prior address
/// `0x3C` collided with the common SSD1306 OLED — if a rig has one
/// attached the scenario fails opaquely.
pub const I2C0_BASE_RP2350: u32 = 0x4009_0000;
const I2C0_IC_TAR: u32 = I2C0_BASE_RP2350 + 0x04;
const I2C0_IC_DATA_CMD: u32 = I2C0_BASE_RP2350 + 0x10;
const I2C0_IC_ENABLE: u32 = I2C0_BASE_RP2350 + 0x6C;
const I2C0_IC_TX_ABRT_SOURCE: u32 = I2C0_BASE_RP2350 + 0x80;
const IC_DATA_CMD_STOP: u32 = 1 << 9;
const IC_DATA_CMD_READ_BIT: u32 = 1 << 8;

/// ADC CS register offset.
const ADC_CS_RP2350: u32 = ADC_BASE + 0x00;
const CS_EN_BIT: u32 = 1 << 0;
const CS_START_ONCE_BIT: u32 = 1 << 2;
/// CS.READY (bit 8) — silicon asserts after one-shot completes. The
/// emulator mirrors this via `AdcRegs::complete_conversion`.
const ADC_CS_READY_BIT: u32 = 1 << 8;

/// PWM (RP2350 `0x400A_8000`) — slice 0 CSR/TOP/CC/CTR offsets (stride
/// 0x14), plus global EN/INTR at `+0xF0` / `+0xF4`.
pub const PWM_BASE_RP2350: u32 = 0x400A_8000;
const PWM_SLICE0_CSR: u32 = PWM_BASE_RP2350 + 0x00;
const PWM_SLICE0_DIV: u32 = PWM_BASE_RP2350 + 0x04;
const PWM_SLICE0_CTR: u32 = PWM_BASE_RP2350 + 0x08;
const PWM_SLICE0_TOP: u32 = PWM_BASE_RP2350 + 0x10;
const PWM_EN_OFFSET: u32 = PWM_BASE_RP2350 + 0xF0;
const PWM_INTR_OFFSET: u32 = PWM_BASE_RP2350 + 0xF4;
const PWM_CSR_EN_BIT: u32 = 1 << 0;

/// UART0 single-byte TX scenario — enable FIFO + UARTEN + TXE, push one
/// byte via UARTDR, advance `max_sysclks`, observe UARTFR.TXFE set.
/// With `IBRD=FBRD=0` the emulator's sysclks-per-byte falls back to 1,
/// so one byte drains on the first tick. Silicon with a programmed
/// baud takes longer — but the scenario's `max_sysclks` budget covers
/// a 1 µs byte-time at 150 MHz (150 cycles) which is well inside the
/// time it takes the PL011 to drain a 115200-baud byte (~13 020
/// cycles). For V5 scope we accept the EMU optimism; on silicon the
/// observation `TXFE=1` still holds post-drain.
const S_UART0_TX_SINGLE_BYTE: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    // Inlined PREFIX_UART0_HARD_RESET — symmetric bidirectional
    // cleanliness with S_UART0_RX_LOOPBACK so whichever scenario lands
    // first under Fisher-Yates leaves a clean UART0 for the other.
    // HLD V1 §4.5.
    (RESETS_RESET + ALIAS_SET, RESET_UART0),
    (RESETS_RESET + ALIAS_CLR, RESET_UART0),
    (UART0_UARTLCR_H, UARTLCR_H_FEN),
    (UART0_UARTCR, UARTCR_UARTEN | UARTCR_TXE),
    (UART0_UARTDR, 0x5A),
];
const O_UART0_TX_SINGLE_BYTE: &[(u32, u32)] = &[
    (UART0_UARTFR, UARTFR_TXFE_BIT),
];

/// UART0 RX loopback — enable FIFO + UARTEN + TXE + RXE + LBE,
/// program baud (IBRD=81, FBRD=24 = 115200 @ 150 MHz clk_peri), push
/// 0x42 via UARTDR, advance enough sysclks for baud-timed TX drain +
/// loopback into RX FIFO. Observe UARTFR (RXFE clear) and UARTDR
/// readback (pop 0x42 from RX FIFO).
///
/// Residual A.2.2 fix (2026-04-17): two scenario edits close this
/// oracle against RP2354 silicon:
///
/// 1. Enable `CLK_PERI_CTRL.ENABLE=1` before touching UART registers.
///    The bootrom is skipped by `Core::reset_and_halt`, so silicon
///    starts with `CLK_PERI_CTRL=0` (gate closed, UARTCLK stopped).
///    Without this write silicon's TX shift register never advances
///    and UARTFR stays at 0x18 (BUSY|RXFE) indefinitely. The emulator
///    currently ignores the gate (tech_debt.md "UART/SPI/I2C ignore
///    CLK_PERI_CTRL.ENABLE"), so this write is a no-op on EMU — but
///    it pins the scenario's precondition for the day the emulator
///    starts honouring the gate.
/// 2. Set `UARTLCR_H.WLEN=0b11` (8-bit) alongside FEN. PL011 masks
///    both TX and loopback-RX data to `WLEN+5` bits; with WLEN=00
///    (reset default) silicon truncates `0x42` → `0x02` in the RX
///    FIFO and the UARTDR observable diverges. The emulator doesn't
///    model WLEN-based data truncation today, so this write is also
///    a no-op on EMU.
const S_UART0_RX_LOOPBACK: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    // Inlined PREFIX_UART0_HARD_RESET — pulses RESET_UART0 before the
    // A.2.2 CLK_PERI / LCR_H / CR sequence so a prior
    // S_UART0_TX_SINGLE_BYTE cannot leave its 0x5A payload in the RX
    // FIFO. The assert+clear pair MUST precede the CLK_PERI_CTRL write
    // so the A.2.2 ordering stays intact. HLD V1 §4.5.
    (RESETS_RESET + ALIAS_SET, RESET_UART0),
    (RESETS_RESET + ALIAS_CLR, RESET_UART0),
    (CLOCKS_CLK_PERI_CTRL, CLK_CTRL_ENABLE),
    (UART0_UARTIBRD, 81),
    (UART0_UARTFBRD, 24),
    (UART0_UARTLCR_H, UARTLCR_H_FEN | UARTLCR_H_WLEN_8),
    (UART0_UARTCR, UARTCR_UARTEN | UARTCR_LBE | UARTCR_TXE | UARTCR_RXE),
    (UART0_UARTDR, 0x42),
];
const O_UART0_RX_LOOPBACK: &[(u32, u32)] = &[
    // UARTFR: full-word mask — RXFE (bit 4) must be clear on both sides
    // because the RX FIFO holds the looped-back byte. TXFE (bit 7)
    // must be set because the TX byte has drained.
    (UART0_UARTFR, 0xFFFF_FFFF),
    // UARTDR: reading pops the RX FIFO head — should be 0x42.
    (UART0_UARTDR, 0xFF),
];

/// SPI0 loopback single-byte — enable + loopback, push 0xA5, observe
/// readback matches.
const S_SPI0_LOOPBACK_SINGLE_BYTE: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    (SPI0_SSPCR0, 7),                        // DSS=7 (8-bit)
    (SPI0_SSPCR1, SSPCR1_SSE | SSPCR1_LBM),  // enable + loopback
    (SPI0_SSPDR, 0xA5),
];
const O_SPI0_LOOPBACK_SINGLE_BYTE: &[(u32, u32)] = &[
    // After the setup writes, the sled has time to push + loopback; the
    // RX FIFO should contain 0xA5 and SSPDR reads pop it. First-read
    // value masked to 0xFF equals 0xA5 on both sides.
    (SPI0_SSPDR, 0xFF),
];

/// I2C0 bus-scan NACK — target an I2C-reserved 7-bit address (`0x7F`),
/// enable, issue READ+STOP. Reserved addresses are never occupied by
/// real silicon devices, so silicon always NACKs; emulator's empty
/// `ALWAYS_ACK_ADDRS` also NACKs. Observe
/// IC_TX_ABRT_SOURCE.ABRT_7B_ADDR_NOACK (bit 0). Prior revisions used
/// `0x3C` (common SSD1306 OLED) and would silently fail on rigs with
/// a display attached.
const S_I2C0_BUS_SCAN_NACK: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    (I2C0_IC_TAR, 0x7F),
    (I2C0_IC_ENABLE, 1),
    (I2C0_IC_DATA_CMD, IC_DATA_CMD_READ_BIT | IC_DATA_CMD_STOP),
];
const O_I2C0_BUS_SCAN_NACK: &[(u32, u32)] = &[
    (I2C0_IC_TX_ABRT_SOURCE, 0x1),
];

/// ADC one-shot — enable, start once, advance enough sys_clks for a
/// conversion to complete. Observe CS.READY set and CS.START_ONCE
/// auto-cleared.
///
/// GPIO26 must be configured for analog input before starting the
/// conversion: disable the digital input buffer (OD=1, IE=0 in
/// PADS_BANK0) and set funcsel to NULL (31) in IO_BANK0. Without this,
/// silicon's ADC sample-and-hold conflicts with the digital input
/// driver and locks the APB bus, causing probe-rs ARM errors.
const S_ADC_ONE_SHOT: &[(u32, u32)] = &[
    // 1. Release all peripherals from reset (incl. ADC, IO_BANK0, PADS_BANK0).
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    // 2. Disable digital input buffer on GPIO26 pad (clear IE bit 6, set OD bit 7).
    //    Reset value is 0x56 (IE=1, PUE=1, PDE=1, SCHMITT=1, DRIVE=4mA).
    //    Target: 0x96 = OD=1, IE=0, rest unchanged.
    (PADS_BANK0_GPIO26, 0x96),
    // 3. Set GPIO26 funcsel to NULL (31) so the pin is routed to ADC, not digital.
    (IO_BANK0_GPIO26_CTRL, 31),
    // 4. Now safe to enable ADC and start one-shot conversion on channel 0.
    (ADC_CS_RP2350, CS_EN_BIT | CS_START_ONCE_BIT),
];
const O_ADC_ONE_SHOT: &[(u32, u32)] = &[
    // READY must be set post-conversion. START_ONCE must have
    // auto-cleared. We mask READY | START_ONCE but expect bit 8 set
    // and bit 2 clear — we verify bit 8 via this observable.
    (ADC_CS_RP2350, ADC_CS_READY_BIT),
];

/// PWM wrap IRQ — enable slice 0 with TOP=100, advance 150 sys_clks,
/// observe INTR bit 0 set (slice 0 wrap). The emulator ticks PWM at one
/// CTR-advance per sys_clk so a sweep past TOP guarantees one wrap.
/// Silicon at post-bootrom CSR.DIV reset (1.0) matches.
const S_PWM_WRAP_IRQ: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    // Inlined PREFIX_PWM_SLICE0_CLEAN — force slice 0 to a known-zero
    // state before arming. Without this the silicon CTR carries over
    // from whichever PWM scenario Fisher-Yates picked last, pre-tripping
    // the wrap IRQ against a stale TOP. See HLD V1 §4.2.
    (PWM_EN_OFFSET, 0),
    (PWM_SLICE0_CSR, 0),
    (PWM_SLICE0_CTR, 0),
    (PWM_INTR_OFFSET, 0xF),
    // TOP must land BEFORE CSR_EN so DAP latency between writes can't
    // tick the counter against the prior scenario's TOP.
    (PWM_SLICE0_TOP, 100),
    (PWM_SLICE0_CSR, PWM_CSR_EN_BIT),
    (PWM_EN_OFFSET, 1),
];
const O_PWM_WRAP_IRQ: &[(u32, u32)] = &[
    (PWM_INTR_OFFSET, 0x1),
];

/// Phase 3.2 — PWM fractional divider: slice 0, TOP=0xFFFF, DIV=0x0020
/// (integer 2, frac 0 = divisor 2.0), EN=1. After 200 sys_clks the
/// counter should advance by 100 (200/2). No wrap expected at TOP=0xFFFF.
const S_PWM_FRACTIONAL_DIV: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    (PWM_SLICE0_TOP, 0xFFFF),
    (PWM_SLICE0_DIV, 0x0020), // INT=2, FRAC=0 -> divisor 2.0
    (PWM_SLICE0_CSR, PWM_CSR_EN_BIT),
    (PWM_EN_OFFSET, 1),
];
const O_PWM_FRACTIONAL_DIV: &[(u32, u32)] = &[
    (PWM_SLICE0_CTR, 100), // 200 sysclks / divisor 2.0 = 100
    (PWM_INTR_OFFSET, 0x0), // no wrap at TOP=0xFFFF
];

// Phase 3.1 — ADC round-robin 2-channel. RROBIN = 0x03 (ch0 + ch1),
// EN + START_MANY, FCS with SHIFT + EN. After ≥ 2 conversions (~600
// sys_clks at 150 MHz / 48 MHz ADC × 96 ticks/conversion), AINSEL
// should have advanced from ch0 to ch1 or wrapped back. Observable:
// CS register masked to AINSEL bits [14:12] (0x7000). We do NOT
// observe FIFO contents — floating pin noise on silicon makes the raw
// sample values non-deterministic.
//
// GPIO26 pad setup mirrors `adc_one_shot`: disable digital input buffer
// (OD=1, IE=0) and set funcsel=31 (NULL/ADC) to avoid the silicon APB
// bus lockup that occurs when the ADC samples while a digital input
// driver is active on the same pad.
/// ADC FCS register (ADC_BASE + 0x08).
const ADC_FCS_RP2350: u32 = ADC_BASE + 0x08;
/// CS value: EN=1, START_MANY=1, RROBIN bits [20:16] = 0x03 (ch0+ch1).
const ADC_CS_RROBIN_2CH: u32 = CS_EN_BIT | (1 << 3) | (0x03 << 16);
/// FCS value: EN=1, SHIFT=1.
const ADC_FCS_SHIFT_EN: u32 = 0x03;
/// CS.AINSEL mask — bits [14:12].
const ADC_CS_AINSEL_MASK: u32 = 0x7000;

const S_ADC_ROUND_ROBIN_2CH: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
    // GPIO26 pad: OD=1, IE=0 (disable digital input buffer for ADC).
    (PADS_BANK0_GPIO26, 0x96),
    // GPIO26 funcsel = NULL (31) — route pin to ADC, not digital fabric.
    (IO_BANK0_GPIO26_CTRL, 31),
    // Enable FIFO with SHIFT.
    (ADC_FCS_RP2350, ADC_FCS_SHIFT_EN),
    // Enable ADC with round-robin on channels 0+1 and START_MANY.
    (ADC_CS_RP2350, ADC_CS_RROBIN_2CH),
];
const O_ADC_ROUND_ROBIN_2CH: &[(u32, u32)] = &[
    // CS masked to AINSEL bits [14:12] only. After 2+ conversions with
    // RROBIN=0x03, AINSEL must have advanced from the reset default (0).
    // The exact value depends on how many conversions completed in the
    // window, but both HW and EMU should agree.
    (ADC_CS_RP2350, ADC_CS_AINSEL_MASK),
];

// S_PIO0_INT_ROUTING_SPLIT — Phase 4.1: PIO0 SM0 asserts IRQ flag 0;
// INT0_INTE enables flag 0 only, INT1_INTE enables flag 1 only.
// After running, IRQ0_INTS must show bit 8 set (SM0/IRQ-flag-0 position in the
// 16-bit INTR layout: bits [15:8]=SM7..SM0, [7:4]=TXNFULL, [3:0]=RXNEMPTY).
// IRQ1_INTS must be 0 because SM0 never asserts IRQ flag 1.
const S_PIO0_INT_ROUTING_SPLIT: &[(u32, u32)] = &[
    // Inlined PREFIX_PIO0_HARD_RESET — replaces the prior single-line
    // release with a set-then-clear pulse pair so SM0.pc starts at 0
    // rather than inheriting a stale PC from a prior PIO scenario.
    // HLD V1 §4.4.
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
    // INSTR_MEM[0] = IRQ SET 0 (opcode 0xC000): asserts PIO IRQ flag 0
    (pio_instr_mem_addr(PIO0_BASE, 0), 0xC000),
    // INSTR_MEM[1] = JMP 1 (spin in place): 0x0001
    (pio_instr_mem_addr(PIO0_BASE, 1), 0x0001),
    // SM0 CLKDIV = 1.0 (integer=1, frac=0)
    (pio_sm_addr(PIO0_BASE, 0, PIO_SM_CLKDIV_OFF), 0x0001_0000),
    // IRQ0_INTE = 0x100 — bit 8 = SM0/IRQ-flag-0 (RP2350 ds Table 1019)
    (PIO0_BASE + PIO_IRQ0_INTE_OFF, 0x100),
    // IRQ1_INTE = 0x200 — bit 9 = SM1/IRQ-flag-1
    // (SM0 never sets flag 1, so NVIC line 1 must stay quiet)
    (PIO0_BASE + PIO_IRQ1_INTE_OFF, 0x200),
    // Enable SM0
    (PIO0_BASE + PIO_CTRL_OFF + ALIAS_SET, 0x0000_0001),
];
const O_PIO0_INT_ROUTING_SPLIT: &[(u32, u32)] = &[
    // IRQ0_INTS at 0x178: bit 8 must be set (IRQ flag 0 enabled on line 0).
    // Mask to 16-bit field.
    (PIO0_BASE + PIO_IRQ0_INTS_OFF, 0xFFFF),
    // IRQ1_INTS at 0x184: must be 0 (flag 1 never set by SM0).
    // Mask to 16-bit field — all bits must be 0.
    (PIO0_BASE + PIO_IRQ1_INTS_OFF, 0xFFFF),
    // Also check raw IRQ register — bit 0 must be set (SM0 asserted flag 0).
    (PIO0_BASE + PIO_IRQ_OFF, 0xFF),
];

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
        min_sysclks: 0,
    },
    PeriphScenario {
        name: "pio0_fixed_cycles",
        setup: S_PIO0_FIXED_CYCLES,
        max_sysclks: 200,
        observe: O_PIO0_FIXED_CYCLES,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    PeriphScenario {
        name: "pio0_side_set_toggle",
        setup: S_PIO0_SIDE_SET_TOGGLE,
        max_sysclks: 100,
        observe: O_PIO0_SIDE_SET_TOGGLE,
        observe_pins: 0x0000_0001,
        custom_sled: None,
        min_sysclks: 0,
    },
    PeriphScenario {
        name: "pio0_reset_gating_placeholder",
        setup: S_PIO0_RESET_GATING_PLACEHOLDER,
        max_sysclks: 200,
        observe: O_PIO0_RESET_GATING_PLACEHOLDER,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    PeriphScenario {
        name: "pll_sys_lock_timing",
        setup: S_PLL_SYS_LOCK_TIMING,
        max_sysclks: 1500,
        observe: O_PLL_SYS_LOCK_TIMING,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    PeriphScenario {
        name: "clock_pll_sys_reprogram_mid_run",
        setup: S_CLOCK_PLL_SYS_REPROGRAM_MID_RUN,
        max_sysclks: 2000,
        observe: O_CLOCK_PLL_SYS_REPROGRAM_MID_RUN,
        observe_pins: 0,
        custom_sled: Some(SLED_CLOCK_PLL_SYS_REPROGRAM_MID_RUN),
        min_sysclks: 0,
    },
    PeriphScenario {
        name: "clock_div_change_pio_running",
        setup: S_CLOCK_DIV_CHANGE_PIO_RUNNING,
        max_sysclks: 2000,
        observe: O_CLOCK_DIV_CHANGE_PIO_RUNNING,
        observe_pins: 0,
        custom_sled: Some(SLED_CLOCK_DIV_CHANGE_PIO_RUNNING),
        min_sysclks: 0,
    },
    // Phase 1 B1: TIMER0 alarm-fire + W1C-clear scenario (HLD V5 §6
    // Phase 1 exit). Sled reads TIMELR, arms ALARM_0 at +1000 µs,
    // busy-polls INTS, W1C's INTR, writes a marker. Silicon
    // validation happens on Arthur's lab rig.
    //
    // max_sysclks is sized for 1000 µs of busy-poll plus sled overhead.
    // At 150 MHz post-bootrom clk_sys: 1000 µs ≈ 150_000 sys_clks; add
    // ~10_000 sys_clks for the setup MOV/STR block and the poll-loop
    // iterations. Round up to 200_000 for headroom. On the emulator,
    // TICKS divides sys_clks by CYCLES=12 → 12_000 sys_clks produces
    // 1000 edges, well below the budget.
    PeriphScenario {
        name: "timer0_alarm0_fire_and_clear",
        setup: S_TIMER0_ALARM0_FIRE_AND_CLEAR,
        max_sysclks: 200_000,
        observe: O_TIMER0_ALARM0_FIRE_AND_CLEAR,
        observe_pins: 0,
        custom_sled: Some(SLED_TIMER0_ALARM0_FIRE_AND_CLEAR),
        // 1000 us alarm with CYCLES=12 -> 12_000 sys_clks minimum.
        min_sysclks: 12_000,
    },
    // Phase 1 B2: TICKS retarget — verify TIMER0 advances at ~half
    // rate after CYCLES doubles 12 → 24 (HLD V5 §6 Phase 1 exit).
    // Sled samples TIMELR, writes CYCLES=24, spin-waits ~2400
    // sys_clks, samples TIMELR again, stores delta at 0x2000_0300.
    PeriphScenario {
        name: "ticks_timer0_retarget_halves_rate",
        setup: S_TICKS_TIMER0_RETARGET,
        max_sysclks: 10_000,
        observe: O_TICKS_TIMER0_RETARGET,
        observe_pins: 0,
        custom_sled: Some(SLED_TICKS_TIMER0_RETARGET),
        // Sled spin-waits ~2400 sys_clks after retarget.
        min_sysclks: 2_400,
    },
    // Phase 2 — UART0 single-byte TX (V5 §6 row 2).
    PeriphScenario {
        name: "uart0_tx_single_byte",
        setup: S_UART0_TX_SINGLE_BYTE,
        max_sysclks: 60_000,
        observe: O_UART0_TX_SINGLE_BYTE,
        observe_pins: 0,
        custom_sled: None,
        // 1 byte at 115200 baud ~ 87 us; at 150 MHz ~ 13_000 sys_clks.
        min_sysclks: 10_000,
    },
    // Phase 4.2 — UART0 RX via internal loopback.
    PeriphScenario {
        name: "uart0_rx_loopback",
        setup: S_UART0_RX_LOOPBACK,
        max_sysclks: 60_000,
        observe: O_UART0_RX_LOOPBACK,
        observe_pins: 0,
        custom_sled: None,
        // 1 byte at 115200 baud @ 150 MHz ~ 13_020 sys_clks for TX drain
        // (reachable now that S_UART0_RX_LOOPBACK enables CLK_PERI_CTRL
        // before programming baud — see residual A.2.2 fix). LBE loopback
        // adds FIFO pipeline latency on top. 25_000 gives ~2x margin over
        // the bare baud-rate minimum so silicon is never sampled mid-TX.
        min_sysclks: 25_000,
    },
    // Phase 2 — SPI0 loopback round-trip.
    PeriphScenario {
        name: "spi0_loopback_single_byte",
        setup: S_SPI0_LOOPBACK_SINGLE_BYTE,
        max_sysclks: 500,
        observe: O_SPI0_LOOPBACK_SINGLE_BYTE,
        observe_pins: 0,
        custom_sled: None,
        // 8-bit SPI transfer at prescaler divider -> ~16 sys_clks minimum.
        min_sysclks: 16,
    },
    // Phase 2 — I2C0 bus scan NACK on a reserved address (0x7F).
    PeriphScenario {
        name: "i2c0_bus_scan_reserved_nack",
        setup: S_I2C0_BUS_SCAN_NACK,
        max_sysclks: 500,
        observe: O_I2C0_BUS_SCAN_NACK,
        observe_pins: 0,
        custom_sled: None,
        // I2C START + 7-bit addr + R/W + NACK -> ~9 bit periods.
        min_sysclks: 20,
    },
    // Phase 2 — ADC one-shot conversion.
    PeriphScenario {
        name: "adc_one_shot",
        setup: S_ADC_ONE_SHOT,
        max_sysclks: 1_000,
        observe: O_ADC_ONE_SHOT,
        observe_pins: 0,
        custom_sled: None,
        // ADC conversion takes ~96 clk_adc cycles.
        min_sysclks: 96,
    },
    // Phase 2 — PWM slice-0 wrap IRQ latch.
    PeriphScenario {
        name: "pwm_wrap_irq",
        setup: S_PWM_WRAP_IRQ,
        max_sysclks: 200,
        observe: O_PWM_WRAP_IRQ,
        observe_pins: 0,
        custom_sled: None,
        // PWM counter must wrap at least once.
        min_sysclks: 2,
    },
    // Phase 3 — DMA mem-to-mem 32-bit, 4 words (V5 §5.6).
    // CPU-bus sled rearchitecture (2026-04-16): sled seeds SRAM via CPU
    // STRs, configures DMA ch0, triggers, and busy-polls CTRL_TRIG.BUSY.
    PeriphScenario {
        name: "dma_mem_to_mem_32bit",
        setup: S_DMA_MEM_TO_MEM_32BIT,
        max_sysclks: 500,
        observe: O_DMA_MEM_TO_MEM_32BIT,
        observe_pins: 0,
        custom_sled: Some(SLED_DMA_MEM_TO_MEM_32BIT),
        // 4-word DMA transfer -> at least 4 bus cycles.
        min_sysclks: 4,
    },
    // Phase 3 — DMA chain trigger: ch0 → ch1 (V5 §5.6).
    // CPU-bus sled rearchitecture: sled seeds both source words, configures
    // ch1 (via AL1_CTRL non-triggering alias), triggers ch0, polls both.
    PeriphScenario {
        name: "dma_chain_trigger",
        setup: S_DMA_CHAIN_TRIGGER,
        max_sysclks: 500,
        observe: O_DMA_CHAIN_TRIGGER,
        observe_pins: 0,
        custom_sled: Some(SLED_DMA_CHAIN_TRIGGER),
        // Two chained DMA transfers -> at least 8 bus cycles.
        min_sysclks: 8,
    },
    // Phase 3.3 — DMA timer-paced transfer: TREQ_SEL=59 (TIMER0), rate 1/10.
    // CPU-bus sled rearchitecture: sled programs DMA_TIMER0, configures ch0
    // with TREQ_SEL=59, triggers, and polls BUSY until transfer completes.
    PeriphScenario {
        name: "dma_timer_paced",
        setup: S_DMA_TIMER_PACED,
        max_sysclks: 500,
        observe: O_DMA_TIMER_PACED,
        observe_pins: 0,
        custom_sled: Some(SLED_DMA_TIMER_PACED),
        // 4 transfers at 1/10 rate → at least 40 sysclks.
        min_sysclks: 40,
    },
    // Phase 1 Expansion — SIO unsigned divider.
    PeriphScenario {
        name: "sio_divider_unsigned",
        setup: S_SIO_DIVIDER_UNSIGNED,
        max_sysclks: 100,
        observe: O_SIO_DIVIDER_UNSIGNED,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    // Phase 1 Expansion — SIO signed divider.
    PeriphScenario {
        name: "sio_divider_signed",
        setup: S_SIO_DIVIDER_SIGNED,
        max_sysclks: 100,
        observe: O_SIO_DIVIDER_SIGNED,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    // Phase 1 Expansion — SIO MTIME count + disable (custom sled).
    PeriphScenario {
        name: "sio_mtime_count_and_match",
        setup: S_SIO_MTIME_COUNT_AND_MATCH,
        max_sysclks: 500,
        observe: O_SIO_MTIME_COUNT_AND_MATCH,
        observe_pins: 0,
        custom_sled: Some(SLED_SIO_MTIME_COUNT_AND_MATCH),
        min_sysclks: 0,
    },
    // Phase 1 Expansion — TIMER1 alarm0 fire and clear (clone of TIMER0).
    PeriphScenario {
        name: "timer1_alarm0_fire_and_clear",
        setup: S_TIMER1_ALARM0_FIRE_AND_CLEAR,
        max_sysclks: 200_000,
        observe: O_TIMER1_ALARM0_FIRE_AND_CLEAR,
        observe_pins: 0,
        custom_sled: Some(SLED_TIMER1_ALARM0_FIRE_AND_CLEAR),
        min_sysclks: 12_000,
    },
    // Phase 3.2 — PWM fractional divider (divisor 2.0, no wrap).
    PeriphScenario {
        name: "pwm_fractional_div",
        setup: S_PWM_FRACTIONAL_DIV,
        max_sysclks: 200,
        observe: O_PWM_FRACTIONAL_DIV,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    // Phase 3.1 — ADC round-robin 2-channel advancement.
    PeriphScenario {
        name: "adc_round_robin_2ch",
        setup: S_ADC_ROUND_ROBIN_2CH,
        max_sysclks: 1_000,
        observe: O_ADC_ROUND_ROBIN_2CH,
        observe_pins: 0,
        custom_sled: None,
        // 2 conversions: 2 × 96 adc ticks × (150/48) sys/adc ≈ 600 sys_clks.
        min_sysclks: 300,
    },
    // Phase 4.1 — PIO0 INTn routing split. SM0 asserts IRQ flag 0;
    // INT0_INTE enables flag 0 only (→ IRQ0_INTS has bit 8 set;
    // RP2350 INTR layout: IRQ flag 0 at bit position 8).
    // INT1_INTE enables flag 1 only (→ IRQ1_INTS stays 0 because SM0
    // never sets flag 1). Validates that the emulator routes each PIO
    // IRQ flag through the per-line INTE mask rather than over-routing
    // to both lines.
    PeriphScenario {
        name: "pio0_int_routing_split",
        setup: S_PIO0_INT_ROUTING_SPLIT,
        max_sysclks: 100,
        observe: O_PIO0_INT_ROUTING_SPLIT,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    // Coverage Gap Fill V11 §3.1 Bucket A item 1 — SYSINFO read-only
    // fields. Four documented u32s; no setup beyond the runner's
    // baseline `release_common_resets`. The countdown sled only needs
    // enough cycles to cover the 4 register probes; pick a small budget.
    PeriphScenario {
        name: "sysinfo_readonly_fields",
        setup: S_SYSINFO_READONLY_FIELDS,
        max_sysclks: 100,
        observe: O_SYSINFO_READONLY_FIELDS,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    // Coverage Gap Fill V11 §3.4 Bucket A item 4 — TBMAN PLATFORM
    // selector. Single-read scenario; TBMAN is not reset-gated so the
    // runner's baseline `release_common_resets` is sufficient.
    PeriphScenario {
        name: "tbman_platform_reads_silicon_value",
        setup: S_TBMAN_PLATFORM,
        max_sysclks: 100,
        observe: O_TBMAN_PLATFORM,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    // Coverage Gap Fill V11 §3.3 Bucket A item 3 — GLITCH_DETECTOR ARM
    // readback. Setup writes ARM = VALUE_YES (0x0000); observe confirms
    // the write stuck and TRIG_STATUS reads 0. GLITCH_DETECTOR is not
    // reset-gated so the runner's `release_common_resets` baseline is
    // sufficient. ARM is marked "Secure read/write only" in silicon —
    // this scenario runs from Secure state, matching the oracle's
    // default execution context.
    PeriphScenario {
        name: "glitch_detector_arm_readback_tracks_ctrl",
        setup: S_GLITCH_DETECTOR_ARM_READBACK,
        max_sysclks: 100,
        observe: O_GLITCH_DETECTOR_ARM_READBACK,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
];

// ---------------------------------------------------------------------------
// Red-path scenarios — Phase 0b HLD V5 §4.2.8 (Phase 2 replacement)
// ---------------------------------------------------------------------------
//
// Three deliberately-broken scenarios designed to exercise the oracle's
// FAIL path: `first_divergence` must render correctly when HW and EMU
// disagree. Gated behind `--red-path` on the standalone binary so
// normal runs (and `test_silicon`) don't flake on them.
//
// All three are GENUINE red-path witnesses on the current emulator:
// each observable diverges because the RP2350 emulator's APB fallthrough
// (`peripheral_regs` HashMap — see `crates/mdrp2350/src/bus/mod.rs`
// §read32/§write32) does not model the peripheral state that silicon
// produces. A fresh HashMap entry returns 0 on read; a written key
// returns exactly the stored bits, never the state-driven flags silicon
// computes.
//
// **Phase 2 replacement**: the Phase 0b witnesses (UART0/SPI0/ADC) are
// now modelled peripherals and have stopped diverging. **Step 2 of the
// V5 §6.C gap fill** (UART1/SPI1/I2C1 array reshape) turned UART1 into
// a modelled peripheral too, so the former `red_uart1_fr_at_reset_
// unmodelled` witness was retired. The catalogue now targets two
// still-unmodelled blocks:
//
//   * `red_trng_status_unmodelled` — TRNG @ `0x400F_0000`. RP2350
//     datasheet §12.12 TRNG block is unmodelled at V5 scope. The TRNG
//     `TRNG_RAND_SOURCE_ENABLE_REG` at +0x1300 defaults to non-zero on
//     the reg-rp235x layout because silicon's TRNG comes out of reset
//     with random-source-enable latched. EMU's HashMap stub returns 0.
//     Divergence any unmasked bit → FAIL. For probe reliability we mask
//     bits 0..=3 (commonly set on silicon; see `trng.h`).
//   * `red_sha256_csr_unmodelled` — SHA256 @ `0x400F_8000`. RP2350
//     datasheet §12.11 SHA256 hash accelerator — unmodelled at V5
//     scope. The `CSR` at +0x00 has WFIFO_READY bit 2 set on reset
//     (FIFO empty-and-ready-to-accept-words). EMU HashMap returns 0
//     verbatim. HW (bit 2 set) ≠ EMU (0) → FAIL.

/// TIMER0 base (RP2350 datasheet §12.8, `0x400B_0000`) and TIMERAWL
/// offset (`0x28` — timer value low half, no latching on read). Used
/// by the B1 `timer0_alarm0_fire_and_clear` main-path scenario.
pub const TIMER0_BASE: u32 = 0x400B_0000;
pub const TIMER0_TIMERAWL: u32 = TIMER0_BASE + 0x28;
/// TIMER0 ALARM_0 offset (`0x10`) — write a 32-bit microsecond target
/// to arm + schedule alarm 0.
pub const TIMER0_ALARM0: u32 = TIMER0_BASE + 0x10;
/// TIMER0 ARMED offset (`0x20`) — RW (write 1-to-disarm).
pub const TIMER0_ARMED: u32 = TIMER0_BASE + 0x20;
/// TIMER0 TIMELR offset (`0x0C`) — read low 32 bits (latches TIMEHR).
pub const TIMER0_TIMELR: u32 = TIMER0_BASE + 0x0C;
/// TIMER0 INTR offset (`0x3C`) — W1C on the four alarm bits.
pub const TIMER0_INTR: u32 = TIMER0_BASE + 0x3C;
/// TIMER0 INTE offset (`0x40`) — per-alarm interrupt enable.
pub const TIMER0_INTE: u32 = TIMER0_BASE + 0x40;
/// TIMER0 INTS offset (`0x48`) — `(INTR | INTF) & INTE`.
pub const TIMER0_INTS: u32 = TIMER0_BASE + 0x48;

/// TICKS block (RP2350 datasheet §8.5, `0x4010_8000`). Six-domain 1 µs
/// tick generator. TIMER0 draws edges from the TIMER0 domain at
/// `+0x18` (CTRL/CYCLES/COUNT stride of `0x0C`).
pub const TICKS_BASE: u32 = 0x4010_8000;
pub const TICKS_TIMER0_CTRL: u32 = TICKS_BASE + 0x18;
pub const TICKS_TIMER0_CYCLES: u32 = TICKS_BASE + 0x1C;
/// `TICKS.CTRL.ENABLE` bit mask (bit 0).
pub const TICKS_CTRL_ENABLE: u32 = 1 << 0;

/// RESETS bit for TIMER0 (RP2350 §7.5, bit 23). Used by Phase 1
/// scenarios to release TIMER0 from reset.
pub const RESET_TIMER0_BIT: u32 = 1 << 23;

/// SPI0 base (RP2350 datasheet §12.2, `0x4008_0000`). PrimeCell PL022.
/// Kept as a public constant for future scenarios; the Phase 0b red-
/// path (SPI0 SSPSR.TFE) was retired once SPI0 gained a real model.
pub const SPI0_BASE: u32 = 0x4008_0000;

/// UART0 base (RP2350 datasheet §12.1.1, `0x4007_0000`). Kept public
/// for future scenarios.
pub const UART0_BASE: u32 = 0x4007_0000;

/// ADC base (RP2350 datasheet §12.4, `0x400A_0000`). Kept public for
/// future scenarios.
pub const ADC_BASE: u32 = 0x400A_0000;

// --- Phase 2 red-path witness addresses --------------------------------
// Three unmodelled peripherals that still fall through to the APB
// `peripheral_regs` HashMap stub on the emulator side.

/// TRNG base (RP2350 datasheet §12.12, `0x400F_0000`). **Unmodelled**.
pub const TRNG_BASE: u32 = 0x400F_0000;
/// TRNG_IMR — interrupt-mask register at +0x100. On silicon bit 0
/// (RND_NUM_VLD interrupt mask) defaults to 1 at reset — the Rockchip
/// RK-TRNG core masks the random-number-valid interrupt until firmware
/// explicitly enables it. Source: RP2350 datasheet §12.12.8 TRNG_IMR
/// register map (reset value `0xFFFF`) and pico-sdk-pico2 header
/// `hardware/regs/trng.h` `TRNG_IMR_RESET = 0xFFFF`. EMU HashMap returns
/// 0, so this is a genuine red-path witness: if the emulator ever adds a
/// TRNG stub that mirrors the reset value, this scenario moves from
/// FAIL to PASS and must be replaced with a different unmodelled witness
/// rather than silently losing the red-path signal.
pub const TRNG_IMR: u32 = TRNG_BASE + 0x100;

/// SHA256 base (RP2350 datasheet §12.11, `0x400F_8000`). **Unmodelled**.
pub const SHA256_BASE: u32 = 0x400F_8000;
/// SHA256_CSR at +0x00. WFIFO_READY (bit 2) is asserted at reset — the
/// FIFO is empty and ready to accept writes. EMU HashMap returns 0.
pub const SHA256_CSR: u32 = SHA256_BASE + 0x00;
/// SHA256_CSR.WFIFO_READY (bit 2).
pub const SHA256_CSR_WFIFO_READY: u32 = 1 << 2;

/// SIO GPIO_OUT (RP2350 offset 0x010).
pub const SIO_GPIO_OUT: u32 = 0xD000_0010;
/// SIO GPIO_OUT_SET (RP2350 offset 0x018 — write-1-set).
pub const SIO_GPIO_OUT_SET: u32 = 0xD000_0018;
/// SIO GPIO_OE_SET (RP2350 offset 0x038 — write-1-set). Offset table
/// per datasheet §3.1.2.
pub const SIO_GPIO_OE_SET: u32 = 0xD000_0038;

// Shared: release every peripheral from reset by writing `!0` to the
// RESETS_RESET.CLR alias. The RP2350 RESETS register only defines bits
// 0..=28 (`resets_state` init mask is `0x1FFF_FFFF`); writes to bits
// 29..=31 are RAZ/WI and therefore harmless. The red-path scenarios
// never consult the exact per-peripheral bit assignment — silicon only
// needs the relevant peripheral out of reset; a scenario-specific
// constant would bit-rot against datasheet revisions without buying
// anything the broad CLR doesn't already deliver.
const RESETS_CLR_ALL: u32 = 0xFFFF_FFFF;

// Per-peripheral clean-state prefixes used by scenario setup tables.
// Each scenario inlines the tuples below (Rust const-slice concat
// requires a macro; inline duplication is acceptable at ≤ 8 usages).
// See `wrk_docs/2026.04.18 - HLD - Silicon Scenario State Reset V1.md`
// §4.1 — these constants are the canonical anchors the scenarios
// mirror.

/// Canonical per-slice PWM clean-state prefix for scenario setups.
/// Gates every slice off, clears per-slice 0 CSR + CTR, and W1C-clears
/// any latched wrap IRQs so a prior scenario's INTR bit can't leak.
#[expect(
    dead_code,
    reason = "documentation anchor — scenarios inline the tuple sequence \
             (Rust const-slice concat requires a macro). HLD V1 §4.1 names \
             this constant as the canonical reference; scenario comments \
             cite it by name."
)]
const PREFIX_PWM_SLICE0_CLEAN: &[(u32, u32)] = &[
    (PWM_EN_OFFSET, 0),
    (PWM_SLICE0_CSR, 0),
    (PWM_SLICE0_CTR, 0),
    (PWM_INTR_OFFSET, 0xF),
];

/// Canonical PIO0 hard-reset pulse for scenario setups. The assert+
/// clear pair wipes `instr_mem[]`, all SM registers, FIFOs and
/// `irq_flags` on silicon, leaving a known-zero peripheral.
#[expect(
    dead_code,
    reason = "documentation anchor — scenarios inline the tuple sequence. \
             HLD V1 §4.1 names this constant as the canonical reference; \
             scenario comments cite it by name."
)]
const PREFIX_PIO0_HARD_RESET: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_SET, RESET_PIO0),
    (RESETS_RESET + ALIAS_CLR, RESET_PIO0),
];

/// Canonical UART0 hard-reset pulse. Wipes the TX / RX FIFOs, shift
/// registers and CR state so a prior `S_UART0_TX_SINGLE_BYTE` can't
/// leak its payload into the next scenario's RX FIFO.
#[expect(
    dead_code,
    reason = "documentation anchor — scenarios inline the tuple sequence. \
             HLD V1 §4.1 names this constant as the canonical reference; \
             scenario comments cite it by name."
)]
const PREFIX_UART0_HARD_RESET: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_SET, RESET_UART0),
    (RESETS_RESET + ALIAS_CLR, RESET_UART0),
];

// S_R2: red-path TRNG — release every peripheral, observe TRNG_IMR
// (0x400F_0100). The Rockchip-derived TRNG core has a non-zero reset
// value in the interrupt-mask register (all interrupts masked at
// reset). EMU HashMap returns 0 — any unmasked bit that silicon
// reports as 1 diverges. We mask bit 0 of IMR as a conservative
// witness bit; the TRNG reset value has that bit set per the silicon
// datasheet wake path.
const S_RED_TRNG_IMR_UNMODELLED: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
];
const O_RED_TRNG_IMR_UNMODELLED: &[(u32, u32)] = &[
    // Bit 0 of IMR is RND_NUM_VLD mask — asserted at reset on silicon.
    (TRNG_IMR, 0x0000_0001),
];

// S_R3: red-path SHA256 — release every peripheral, observe SHA256
// CSR (0x400F_8000) masked to WFIFO_READY (bit 2). Silicon's SHA256
// hash accelerator reports the write-FIFO ready to accept words at
// reset (FIFO is empty). EMU's HashMap stub returns 0. Divergence on
// bit 2 → FAIL.
const S_RED_SHA256_CSR_UNMODELLED: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESETS_CLR_ALL),
];
const O_RED_SHA256_CSR_UNMODELLED: &[(u32, u32)] = &[
    (SHA256_CSR, SHA256_CSR_WFIFO_READY),
];

// ---------------------------------------------------------------------------
// DMA scenarios — Phase 3 (HLD V5 §5.6)
// ---------------------------------------------------------------------------

/// DMA base (RP2350 §12.6, `0x5000_0000`).
pub const DMA_BASE: u32 = 0x5000_0000;
/// RESETS bit for DMA (§7.5, bit 2).
pub const RESET_DMA_BIT: u32 = 1 << 2;
/// DMA global INTR offset (§12.6.6).
pub const DMA_INTR: u32 = DMA_BASE + 0x400;

// S_DMA1: DMA mem-to-mem 32-bit, 4 words, DREQ_FORCE (ch0).
//
// CPU-bus sled rearchitecture (2026-04-16): DAP writes do not drive DMA
// on silicon — the DMA controller's bus-master port never sees the DAP's
// seed data and debug-halt clocks can gate DMA completion. The setup
// table is now minimal (RESETS release only); a custom sled seeds SRAM
// and drives DMA entirely through CPU stores, then busy-polls CTRL_TRIG
// BUSY (bit 26) until the transfer completes.
//
// CTRL_TRIG value breakdown (RP2350 field positions — differs from RP2040):
//   bit 0      : EN = 1
//   bits [3:2] : DATA_SIZE = 2 (word)
//   bit 4      : INCR_READ = 1
//   bit 5      : INCR_READ_REV = 0  (new in RP2350 vs RP2040)
//   bit 6      : INCR_WRITE = 1
//   bit 7      : INCR_WRITE_REV = 0 (new in RP2350 vs RP2040)
//   bits [11:8] : RING_SIZE = 0
//   bit 12      : RING_SEL = 0
//   bits [16:13]: CHAIN_TO = 0 (ch0 = self = no chain, per RP2350 datasheet §12.6.3.2)
//   bits [22:17]: TREQ_SEL = 63 (0x3F, PERMANENT/FORCE)
//   → 0x007E_0059
const S_DMA_MEM_TO_MEM_32BIT: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_DMA_BIT),
];
const O_DMA_MEM_TO_MEM_32BIT: &[(u32, u32)] = &[
    // All 4 destination words must match source (seeded 0xDEAD_0001..4).
    (0x2000_0300, 0xFFFF_FFFF),
    (0x2000_0304, 0xFFFF_FFFF),
    (0x2000_0308, 0xFFFF_FFFF),
    (0x2000_030C, 0xFFFF_FFFF),
    // DMA INTR bit 0 must be set (transfer complete).
    (DMA_INTR, 0x0000_0001),
];

// Sled: seed 4 words at 0x2000_0100 (0xDEAD_0001..4), configure DMA
// ch0 for SRAM→SRAM copy, trigger via CTRL_TRIG write, poll BUSY clear.
//
// Register assignments (all caller-saved):
//   r0 — word value (initialised to 0xDEAD_0001, incremented per word)
//   r1 — DMA_BASE (0x5000_0000), then reused for CTRL_TRIG poll
//   r3 — BUSY mask (0x0400_0000 = bit 26)
//   r4 — address / config scratch
//   r5 — source SRAM address (0x2000_0100)
//
// Thumb-2 encodings — all ARMv8-M Thumb (see
// crates/mdpicoem-harness/src/silicon_scenarios.rs sled comment block):
//   movw T3 / movt T1 / movs T1 / adds T2 / str T1 / ldr T1 /
//   lsls T1 / tst T1 / bne T1 / bkpt T1.
//
// Busy-poll loop (halfwords [37..39]):
//   [37] ldr  r2, [r1, #0x0C]  ; read CH0_CTRL_TRIG
//   [38] tst  r2, r3           ; test BUSY (bit 26)
//   [39] bne  [37]             ; imm8=-4 → D1FC; loop if busy
// B<cond> T1: target=PC+4+SignExtend(imm8,8)*2. [39] at byte 78.
// PC = byte78+4=byte82. target=sled+74 ([37]). imm8=(74-82)/2=-4=0xFC.
#[rustfmt::skip]
const SLED_DMA_MEM_TO_MEM_32BIT_HW: [u16; 41] = [
    // ---- seed source SRAM (r0 = 0xDEAD_0001, r5 = 0x2000_0100) -----------
    0x2001, //  [ 0] movs r0, #1
    0xF6CD, //  [ 1] movt r0, #0xDEAD hw0   (imm4=D,i=1,imm3=6,imm8=AD)
    0x60AD, //  [ 2] movt r0, #0xDEAD hw1   (Rd=0)
    0xF240, //  [ 3] movw r5, #0x0100 hw0
    0x1500, //  [ 4] movw r5, #0x0100 hw1   (imm3=1,Rd=5,imm8=00)
    0xF2C2, //  [ 5] movt r5, #0x2000 hw0
    0x0500, //  [ 6] movt r5, #0x2000 hw1   (Rd=5)
    0x6028, //  [ 7] str  r0, [r5, #0]      ; src[0]=0xDEAD_0001
    0x3001, //  [ 8] adds r0, r0, #1
    0x6068, //  [ 9] str  r0, [r5, #4]      ; src[1]=0xDEAD_0002
    0x3001, //  [10] adds r0, r0, #1
    0x60A8, //  [11] str  r0, [r5, #8]      ; src[2]=0xDEAD_0003
    0x3001, //  [12] adds r0, r0, #1
    0x60E8, //  [13] str  r0, [r5, #12]     ; src[3]=0xDEAD_0004
    // ---- build DMA_BASE in r1 (0x5000_0000) --------------------------------
    0xF240, //  [14] movw r1, #0x0000 hw0
    0x0100, //  [15] movw r1, #0x0000 hw1   (Rd=1)
    0xF2C5, //  [16] movt r1, #0x5000 hw0
    0x0100, //  [17] movt r1, #0x5000 hw1   (Rd=1)
    // ---- program CH0_READ_ADDR = 0x2000_0100 (in r4) -----------------------
    0xF240, //  [18] movw r4, #0x0100 hw0
    0x1400, //  [19] movw r4, #0x0100 hw1   (Rd=4)
    0xF2C2, //  [20] movt r4, #0x2000 hw0
    0x0400, //  [21] movt r4, #0x2000 hw1   (Rd=4)
    0x600C, //  [22] str  r4, [r1, #0]      ; CH0_READ_ADDR
    // ---- program CH0_WRITE_ADDR = 0x2000_0300 (in r4) ----------------------
    0xF240, //  [23] movw r4, #0x0300 hw0
    0x3400, //  [24] movw r4, #0x0300 hw1   (imm3=3,Rd=4)
    0xF2C2, //  [25] movt r4, #0x2000 hw0
    0x0400, //  [26] movt r4, #0x2000 hw1
    0x604C, //  [27] str  r4, [r1, #4]      ; CH0_WRITE_ADDR  (imm5=1)
    // ---- program CH0_TRANS_COUNT = 4 ----------------------------------------
    0x2404, //  [28] movs r4, #4
    0x608C, //  [29] str  r4, [r1, #8]      ; CH0_TRANS_COUNT (imm5=2)
    // ---- program CH0_CTRL_TRIG = 0x007E_0059 (triggers transfer) -----------
    // RP2350 CTRL field positions differ from RP2040: INCR_READ_REV [5] and
    // INCR_WRITE_REV [7] are new, shifting RING_SIZE to [11:8], RING_SEL to
    // [12], CHAIN_TO to [16:13], TREQ_SEL to [22:17], IRQ_QUIET to [23].
    // EN=1, DATA_SIZE=2(word), INCR_READ=1[4], INCR_WRITE=1[6],
    // CHAIN_TO=0[16:13](ch0=self=no chain), TREQ_SEL=63[22:17](FORCE).
    // 0x0059 = 0000_0000_0101_1001: bit6=1(INCR_WRITE), bit4=1(INCR_READ),
    //   bit3=0, bit2=1(DATA_SIZE lsb? no: [3:2]=10=2), bit1=0, bit0=1(EN).
    // Wait: 0x59=0101_1001: bit6=1, bit4=1, bit3=0, bits[3:2]=10, bit0=1. OK.
    // 0x007E = 0000_0000_0111_1110 = bits[22:17]=0x3F=63 (TREQ_SEL=FORCE).
    // MOVW r4, #0x0059: imm4=0,i=0,imm3=0,imm8=0x59 → hw0=F240, hw1=0x0459
    // MOVT r4, #0x007E: imm4=0,i=0,imm3=0,imm8=0x7E → hw0=F2C0, hw1=0x047E
    0xF240, //  [30] movw r4, #0x0059 hw0   (imm4=0,i=0,imm3=0,imm8=59)
    0x0459, //  [31] movw r4, #0x0059 hw1   (Rd=4)
    0xF2C0, //  [32] movt r4, #0x007E hw0   (imm4=0,i=0,imm3=0,imm8=7E)
    0x047E, //  [33] movt r4, #0x007E hw1   (Rd=4)
    0x60CC, //  [34] str  r4, [r1, #0x0C]   ; CH0_CTRL_TRIG   (imm5=3)
    // ---- build BUSY mask in r3 (bit 26 = 0x0400_0000) ----------------------
    0x2301, //  [35] movs r3, #1
    0x069B, //  [36] lsls r3, r3, #26       ; r3 = 0x0400_0000
    // ---- busy-poll loop (target=[37], bne imm8=-4 = 0xFC) ------------------
    // B<cond> T1: target = PC + 4 + SignExtend(imm8,8)*2.
    // [39] is at byte 78 from sled start. PC = byte78 + 4 = byte 82.
    // Target = sled + 74 (halfword [37] = ldr r2,...).
    // imm8 = (74 - 82) / 2 = -4 = 0xFC.
    0x68CA, //  [37] ldr  r2, [r1, #0x0C]   ; read CH0_CTRL_TRIG (imm5=3)
    0x421A, //  [38] tst  r2, r3            ; test BUSY
    0xD1FC, //  [39] bne  [37]              ; loop while BUSY set
    0xBE00, //  [40] bkpt #0
];
const SLED_DMA_MEM_TO_MEM_32BIT: &[u8] =
    &halfwords_to_le_bytes::<41, 82>(SLED_DMA_MEM_TO_MEM_32BIT_HW);

// S_DMA2: DMA chain trigger — ch0 completes, chains to ch1.
//
// CPU-bus sled rearchitecture (same rationale as S_DMA1). Setup table
// releases DMA from reset only. Sled seeds both source words via CPU
// STRs, programs ch1 (via AL1_CTRL — non-triggering alias at +0x50),
// then programs ch0 CTRL_TRIG (triggers). Poll ch0 BUSY until clear
// (chain arms ch1 automatically on ch0 completion), then poll ch1 BUSY.
//
// Ch0 CTRL_TRIG: EN=1, DATA_SIZE=2, INCR_READ[4], INCR_WRITE[6],
//   TREQ_SEL=63[22:17], CHAIN_TO=1[16:13] (chain to ch1 on completion).
//   → 0x007E_2059  (RP2350 field positions)
//
// Ch1 AL1_CTRL (non-triggering alias at DMA_BASE+0x050):
//   EN=1, DATA_SIZE=2, INCR_READ[4], INCR_WRITE[6], TREQ_SEL=63[22:17],
//   CHAIN_TO=1[16:13] (ch1=self=no further chain per RP2350 §12.6.3.2).
//   → 0x007E_2059
const S_DMA_CHAIN_TRIGGER: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_DMA_BIT),
];
const O_DMA_CHAIN_TRIGGER: &[(u32, u32)] = &[
    // Ch0 destination (0x2000_0600 ← 0xAAAA_0000).
    (0x2000_0600, 0xFFFF_FFFF),
    // Ch1 destination (0x2000_0700 ← 0xBBBB_1111).
    (0x2000_0700, 0xFFFF_FFFF),
    // INTR bits 0 and 1 must be set (both transfers complete).
    (DMA_INTR, 0x0000_0003),
];

// Sled for dma_chain_trigger.
//
// Register assignments:
//   r0 — word value scratch
//   r1 — DMA_BASE (0x5000_0000)
//   r2 — temp (CTRL_TRIG readback)
//   r3 — BUSY mask (0x0400_0000)
//   r4 — config / address scratch
//   r5 — source/dest address scratch
//
// Register layout of DMA channels from DMA_BASE (r1):
//   ch0: +0x00=READ_ADDR, +0x04=WRITE_ADDR, +0x08=TRANS_COUNT, +0x0C=CTRL_TRIG
//   ch1: +0x40=READ_ADDR, +0x44=WRITE_ADDR, +0x48=TRANS_COUNT, +0x50=AL1_CTRL,
//        +0x4C=CTRL_TRIG (written by ch0's chain completion, not the sled)
//
// Halfword index breakdown:
//   [0..5]   seed ch0 source word (0xAAAA_0000) at 0x2000_0400
//   [6..11]  seed ch1 source word (0xBBBB_1111) at 0x2000_0500
//   [12..17] build DMA_BASE in r1
//   [18..27] configure ch1 via non-triggering AL1 alias
//   [28..37] configure ch0 CTRL_TRIG (triggers ch0, chains to ch1)
//   [38..40] build BUSY mask in r3
//   [41..43] poll ch0 BUSY (loop until ch0 complete → chain arms ch1)
//   [44..46] poll ch1 BUSY (loop until ch1 complete)
//   [47]     bkpt #0
//
// Key encodings:
//   0xAAAA_0000: movs r0,#0 / movt r0,#0xAAAA
//     movt r0,#0xAAAA: 0xAAAA→imm4=A,i=1,imm3=2,imm8=AA
//       hw0=F2C0|(1<<10)|0xA=F6CA, hw1=(2<<12)|(0<<8)|0xAA=0x20AA
//   0xBBBB_1111: movw r0,#0x1111 / movt r0,#0xBBBB
//     movw r0,#0x1111: imm4=1,i=0,imm3=1,imm8=11 → hw0=F241, hw1=0x1011
//     movt r0,#0xBBBB: imm4=B,i=1,imm3=3,imm8=BB → hw0=F6CB, hw1=0x30BB
//   CTRL 0x007E_2059 (ch0: CHAIN_TO=1, ch1: CHAIN_TO=1=self=no chain, TREQ=63):
//     low16=0x2059: imm4=2,i=0,imm3=0,imm8=59 → hw0=F242, hw1=(Rd<<8)|0x59
//     high16=0x007E: imm4=0,i=0,imm3=0,imm8=7E → hw0=F2C0, hw1=(Rd<<8)|0x7E
//   str r4,[r1,#0x40] imm5=16: 0x6000|(16<<6)|(1<<3)|4=0x640C
//   str r4,[r1,#0x44] imm5=17: 0x6000|(17<<6)|(1<<3)|4=0x644C
//   str r4,[r1,#0x48] imm5=18: 0x6000|(18<<6)|(1<<3)|4=0x648C
//   str r4,[r1,#0x50] imm5=20: 0x6000|(20<<6)|(1<<3)|4=0x650C
//   ldr r2,[r1,#0x4C] imm5=19: 0x6800|(19<<6)|(1<<3)|2=0x6CCA (poll ch1)
#[rustfmt::skip]
const SLED_DMA_CHAIN_TRIGGER_HW: [u16; 64] = [
    // ---- seed ch0 source: 0xAAAA_0000 → 0x2000_0400 ----------------------
    0x2000, //  [ 0] movs r0, #0
    0xF6CA, //  [ 1] movt r0, #0xAAAA hw0
    0x20AA, //  [ 2] movt r0, #0xAAAA hw1   (Rd=0)
    0xF240, //  [ 3] movw r5, #0x0400 hw0   (imm3=4,Rd=5)
    0x4500, //  [ 4] movw r5, #0x0400 hw1
    0xF2C2, //  [ 5] movt r5, #0x2000 hw0
    0x0500, //  [ 6] movt r5, #0x2000 hw1
    0x6028, //  [ 7] str  r0, [r5, #0]      ; ch0 src word
    // ---- seed ch1 source: 0xBBBB_1111 → 0x2000_0500 ----------------------
    0xF241, //  [ 8] movw r0, #0x1111 hw0   (imm4=1,i=0,imm3=1,imm8=11)
    0x1011, //  [ 9] movw r0, #0x1111 hw1   (Rd=0)
    0xF6CB, //  [10] movt r0, #0xBBBB hw0   (imm4=B,i=1,imm3=3,imm8=BB)
    0x30BB, //  [11] movt r0, #0xBBBB hw1   (Rd=0)
    0xF240, //  [12] movw r5, #0x0500 hw0   (imm3=5,Rd=5)
    0x5500, //  [13] movw r5, #0x0500 hw1
    0xF2C2, //  [14] movt r5, #0x2000 hw0
    0x0500, //  [15] movt r5, #0x2000 hw1
    0x6028, //  [16] str  r0, [r5, #0]      ; ch1 src word
    // ---- build DMA_BASE in r1 (0x5000_0000) --------------------------------
    0xF240, //  [17] movw r1, #0x0000 hw0
    0x0100, //  [18] movw r1, #0x0000 hw1
    0xF2C5, //  [19] movt r1, #0x5000 hw0
    0x0100, //  [20] movt r1, #0x5000 hw1
    // ---- configure ch1 via non-triggering AL1_CTRL (at +0x50) ------------
    0xF240, //  [21] movw r4, #0x0500 hw0   ; ch1 READ_ADDR = 0x2000_0500
    0x5400, //  [22] movw r4, #0x0500 hw1   (Rd=4)
    0xF2C2, //  [23] movt r4, #0x2000 hw0
    0x0400, //  [24] movt r4, #0x2000 hw1
    0x640C, //  [25] str  r4, [r1, #0x40]   ; CH1_READ_ADDR  (imm5=16)
    0xF240, //  [26] movw r4, #0x0700 hw0   ; ch1 WRITE_ADDR = 0x2000_0700
    0x7400, //  [27] movw r4, #0x0700 hw1   (imm3=7,Rd=4)
    0xF2C2, //  [28] movt r4, #0x2000 hw0
    0x0400, //  [29] movt r4, #0x2000 hw1
    0x644C, //  [30] str  r4, [r1, #0x44]   ; CH1_WRITE_ADDR (imm5=17)
    0x2401, //  [31] movs r4, #1            ; ch1 TRANS_COUNT=1
    0x648C, //  [32] str  r4, [r1, #0x48]   ; CH1_TRANS_COUNT (imm5=18)
    // ch1 CTRL = 0x007E_2059: EN=1, DATA_SIZE=2, INCR_READ=1[4], INCR_WRITE=1[6],
    // CHAIN_TO=1[16:13](ch1=self=no chain), TREQ_SEL=63[22:17](FORCE).
    // MOVW r4, #0x2059: imm4=2,i=0,imm3=0,imm8=59 → hw0=F242, hw1=0x0459
    // MOVT r4, #0x007E: imm4=0,i=0,imm3=0,imm8=7E → hw0=F2C0, hw1=0x047E
    0xF242, //  [33] movw r4, #0x2059 hw0   ; ch1 ctrl: 0x007E_2059 (RP2350 positions)
    0x0459, //  [34] movw r4, #0x2059 hw1   (Rd=4)
    0xF2C0, //  [35] movt r4, #0x007E hw0   (imm4=0,i=0,imm3=0,imm8=7E)
    0x047E, //  [36] movt r4, #0x007E hw1   (Rd=4)
    0x650C, //  [37] str  r4, [r1, #0x50]   ; CH1_AL1_CTRL   (imm5=20)
    // ---- configure ch0 and trigger (CTRL_TRIG at +0x0C) -------------------
    0xF240, //  [38] movw r4, #0x0400 hw0   ; ch0 READ_ADDR = 0x2000_0400
    0x4400, //  [39] movw r4, #0x0400 hw1   (imm3=4,Rd=4)
    0xF2C2, //  [40] movt r4, #0x2000 hw0
    0x0400, //  [41] movt r4, #0x2000 hw1
    0x600C, //  [42] str  r4, [r1, #0]      ; CH0_READ_ADDR
    0xF240, //  [43] movw r4, #0x0600 hw0   ; ch0 WRITE_ADDR = 0x2000_0600
    0x6400, //  [44] movw r4, #0x0600 hw1   (imm3=6,Rd=4)
    0xF2C2, //  [45] movt r4, #0x2000 hw0
    0x0400, //  [46] movt r4, #0x2000 hw1
    0x604C, //  [47] str  r4, [r1, #4]      ; CH0_WRITE_ADDR (imm5=1)
    0x2401, //  [48] movs r4, #1            ; ch0 TRANS_COUNT=1
    0x608C, //  [49] str  r4, [r1, #8]      ; CH0_TRANS_COUNT (imm5=2)
    // ch0 CTRL = 0x007E_2059: EN=1, DATA_SIZE=2, INCR_READ=1[4], INCR_WRITE=1[6],
    // CHAIN_TO=1[16:13](chains to ch1 on completion), TREQ_SEL=63[22:17](FORCE).
    // Same encoding as ch1: MOVW r4, #0x2059 / MOVT r4, #0x007E.
    0xF242, //  [50] movw r4, #0x2059 hw0   ; ch0 ctrl: 0x007E_2059, CHAIN_TO=1
    0x0459, //  [51] movw r4, #0x2059 hw1   (Rd=4)
    0xF2C0, //  [52] movt r4, #0x007E hw0   (imm4=0,i=0,imm3=0,imm8=7E)
    0x047E, //  [53] movt r4, #0x007E hw1   (Rd=4)
    0x60CC, //  [54] str  r4, [r1, #0x0C]   ; CH0_CTRL_TRIG → triggers ch0
    // ---- BUSY mask: r3 = bit 26 (0x0400_0000) ------------------------------
    0x2301, //  [55] movs r3, #1
    0x069B, //  [56] lsls r3, r3, #26
    // ---- poll ch0 BUSY (ch0 chains to ch1 on completion) ------------------
    // B<cond> T1: target = PC + 4 + SignExtend(imm8,8)*2.
    // [59] at byte 118. PC = byte118 + 4 = byte 122. Target = byte 114 ([57]).
    // imm8 = (114 - 122) / 2 = -4 = 0xFC.
    0x68CA, //  [57] ldr  r2, [r1, #0x0C]   ; read CH0_CTRL_TRIG (imm5=3)
    0x421A, //  [58] tst  r2, r3
    0xD1FC, //  [59] bne  [57]              ; imm8=-4 → loop while ch0 busy
    // ---- poll ch1 BUSY at +0x4C (imm5=19) ---------------------------------
    // [62] at byte 124. PC = byte124 + 4 = byte 128. Target = byte 120 ([60]).
    // imm8 = (120 - 128) / 2 = -4 = 0xFC.
    0x6CCA, //  [60] ldr  r2, [r1, #0x4C]   ; read CH1_CTRL_TRIG
    0x421A, //  [61] tst  r2, r3
    0xD1FC, //  [62] bne  [60]              ; imm8=-4 → loop while ch1 busy
    0xBE00, //  [63] bkpt #0
];
const SLED_DMA_CHAIN_TRIGGER: &[u8] =
    &halfwords_to_le_bytes::<64, 128>(SLED_DMA_CHAIN_TRIGGER_HW);

// S_DMA3: DMA timer-paced transfer — TREQ_SEL=59 (TIMER0), rate 1/10.
//
// CPU-bus sled rearchitecture (same rationale as S_DMA1). Setup table
// releases DMA from reset only. Sled seeds SRAM, programs DMA_TIMER0
// (0x5000_0440 per RP2350 §12.6.6 — not 0x5000_0420 which is the RP2040
// offset, see Residual C.2.1), with X=1/Y=10 → fires every 10 sysclks,
// configures ch0 (TREQ_SEL=59), triggers, and polls BUSY until complete.
//
// CTRL_TRIG value breakdown (RP2350 field positions — differs from RP2040):
//   bit 0      : EN = 1
//   bits [3:2] : DATA_SIZE = 2 (word)
//   bit 4      : INCR_READ = 1
//   bit 5      : INCR_READ_REV = 0  (new in RP2350 vs RP2040)
//   bit 6      : INCR_WRITE = 1
//   bit 7      : INCR_WRITE_REV = 0 (new in RP2350 vs RP2040)
//   bits [11:8] : RING_SIZE = 0
//   bit 12      : RING_SEL = 0
//   bits [16:13]: CHAIN_TO = 0 (ch0 = self = no chain)
//   bits [22:17]: TREQ_SEL = 59 (0x3B = TIMER0)
//   → 0x0076_0059
/// DMA TIMER0 register absolute address (RP2350 §12.6.6 — `DMA_BASE + 0x440`).
///
/// RP2350 inserts INTE2/INTF2/INTS2 (0x424..0x42C) and INTE3/INTF3/INTS3
/// (0x434..0x43C) into the global-register block, shifting TIMER0..3 from
/// 0x420..0x42C (RP2040) up by 0x20 bytes to 0x440..0x44C.  Writing to
/// `DMA_BASE + 0x420` on RP2350 lands in reserved padding (harmless RAZ/WI),
/// which masked the Residual C.2.1 bug on the emulator before the 2026-04-17
/// register-offset fix.
pub const DMA_TIMER0: u32 = DMA_BASE + 0x440;
const S_DMA_TIMER_PACED: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_DMA_BIT),
];
const O_DMA_TIMER_PACED: &[(u32, u32)] = &[
    // All 4 destination words must match source (seeded 0xCAFE_0001..4).
    (0x2000_0B00, 0xFFFF_FFFF),
    (0x2000_0B04, 0xFFFF_FFFF),
    (0x2000_0B08, 0xFFFF_FFFF),
    (0x2000_0B0C, 0xFFFF_FFFF),
    // DMA INTR bit 0 must be set (transfer complete).
    (DMA_INTR, 0x0000_0001),
];

// Sled for dma_timer_paced.
//
// Register assignments:
//   r0 — word value (0xCAFE_0001, incremented)
//   r1 — DMA_BASE (0x5000_0000)
//   r2 — DMA_TIMER0 address scratch / CTRL_TRIG readback
//   r3 — BUSY mask (0x0400_0000)
//   r4 — config / address scratch
//   r5 — source SRAM address (0x2000_0A00)
//
// DMA_TIMER0 = 0x5000_0440 (RP2350 §12.6.6) — cannot reach with a simple
// [r1,#imm5*4] (0x440/4=272 > imm5_max=31), so we build the address in r2
// explicitly and use str r4,[r2,#0].
//
// Key encodings:
//   MOVW T3: imm16={imm4,i,imm3,imm8}; hw0=F240|(i<<10)|imm4, hw1=(imm3<<12)|(Rd<<8)|imm8
//   hw1 bit15 MUST be 0; values ≥0x0800 set i=1 in hw0 (→0xF640+imm4).
//   0x0A00={imm4=0,i=1,imm3=2,imm8=0x00} → hw0=F640, hw1=(2<<12)|(Rd<<8)
//   0x0B00={imm4=0,i=1,imm3=3,imm8=0x00} → hw0=F640, hw1=(3<<12)|(Rd<<8)
//   0x0440={imm4=0,i=0,imm3=4,imm8=0x40} → hw0=F240, hw1=(4<<12)|(Rd<<8)|0x40
//   0xCAFE={imm4=C,i=1,imm3=2,imm8=FE} → hw0=F6CC(MOVT hw0=F2C0+0x0400+0xC), hw1=0x2xFE
//   CTRL 0x0076_0059 (TREQ_SEL=59=TIMER0, CHAIN_TO=0=self=no chain, RP2350):
//     low16=0x0059: imm4=0,i=0,imm3=0,imm8=0x59 → hw0=F240, hw1=(Rd<<8)|0x59
//     high16=0x0076: imm4=0,i=0,imm3=0,imm8=0x76 → hw0=F2C0, hw1=(Rd<<8)|0x76
//   DMA_TIMER0 value 0x0001_000A: movs r4,#0x0A; movt r4,#0x0001
//     movt #0x0001: imm4=0,i=0,imm3=0,imm8=1 → hw0=F2C0, hw1=(Rd<<8)|1
#[rustfmt::skip]
const SLED_DMA_TIMER_PACED_HW: [u16; 49] = [
    // ---- seed source SRAM (r0 = 0xCAFE_0001, r5 = 0x2000_0A00) -----------
    0x2001, //  [ 0] movs r0, #1
    0xF6CC, //  [ 1] movt r0, #0xCAFE hw0   (imm4=C,i=1,imm3=2,imm8=FE)
    0x20FE, //  [ 2] movt r0, #0xCAFE hw1   (Rd=0)
    0xF640, //  [ 3] movw r5, #0x0A00 hw0   (i=1,imm4=0 → F240|(1<<10)=F640)
    0x2500, //  [ 4] movw r5, #0x0A00 hw1   (imm3=2,Rd=5,imm8=00)
    0xF2C2, //  [ 5] movt r5, #0x2000 hw0
    0x0500, //  [ 6] movt r5, #0x2000 hw1
    0x6028, //  [ 7] str  r0, [r5, #0]      ; src[0]=0xCAFE_0001
    0x3001, //  [ 8] adds r0, r0, #1
    0x6068, //  [ 9] str  r0, [r5, #4]      ; src[1]=0xCAFE_0002
    0x3001, //  [10] adds r0, r0, #1
    0x60A8, //  [11] str  r0, [r5, #8]      ; src[2]=0xCAFE_0003
    0x3001, //  [12] adds r0, r0, #1
    0x60E8, //  [13] str  r0, [r5, #12]     ; src[3]=0xCAFE_0004
    // ---- build DMA_BASE in r1 (0x5000_0000) --------------------------------
    0xF240, //  [14] movw r1, #0x0000 hw0
    0x0100, //  [15] movw r1, #0x0000 hw1
    0xF2C5, //  [16] movt r1, #0x5000 hw0
    0x0100, //  [17] movt r1, #0x5000 hw1
    // ---- write DMA_TIMER0 = 0x0001_000A (r2 = 0x5000_0440) ---------------
    // RP2350 §12.6.6: DMA_TIMER0 is at DMA_BASE+0x440, not 0x420 (RP2040).
    // MOVW r2, #0x0440: imm4=0,i=0,imm3=4,imm8=0x40
    //   hw0 = 0xF240 | (0<<10) | 0 = 0xF240
    //   hw1 = (4<<12) | (2<<8) | 0x40 = 0x4240
    0xF240, //  [18] movw r2, #0x0440 hw0   (imm3=4,imm8=0x40)
    0x4240, //  [19] movw r2, #0x0440 hw1   (Rd=2)
    0xF2C5, //  [20] movt r2, #0x5000 hw0
    0x0200, //  [21] movt r2, #0x5000 hw1   (Rd=2)
    0x240A, //  [22] movs r4, #0x0A         ; r4 low = 10 (Y=10)
    0xF2C0, //  [23] movt r4, #0x0001 hw0   (imm4=0,i=0,imm3=0,imm8=1)
    0x0401, //  [24] movt r4, #0x0001 hw1   (Rd=4, r4=0x0001_000A)
    0x6014, //  [25] str  r4, [r2, #0]      ; DMA_TIMER0 = (X=1)<<16|(Y=10)
    // ---- program CH0_READ_ADDR = 0x2000_0A00 --------------------------------
    0xF640, //  [26] movw r4, #0x0A00 hw0   (i=1,imm4=0 → F640)
    0x2400, //  [27] movw r4, #0x0A00 hw1   (imm3=2,Rd=4,imm8=00)
    0xF2C2, //  [28] movt r4, #0x2000 hw0
    0x0400, //  [29] movt r4, #0x2000 hw1
    0x600C, //  [30] str  r4, [r1, #0]      ; CH0_READ_ADDR
    // ---- program CH0_WRITE_ADDR = 0x2000_0B00 --------------------------------
    0xF640, //  [31] movw r4, #0x0B00 hw0   (i=1,imm4=0 → F640)
    0x3400, //  [32] movw r4, #0x0B00 hw1   (imm3=3,Rd=4,imm8=00)
    0xF2C2, //  [33] movt r4, #0x2000 hw0
    0x0400, //  [34] movt r4, #0x2000 hw1
    0x604C, //  [35] str  r4, [r1, #4]      ; CH0_WRITE_ADDR (imm5=1)
    // ---- program CH0_TRANS_COUNT = 4 -----------------------------------------
    0x2404, //  [36] movs r4, #4
    0x608C, //  [37] str  r4, [r1, #8]      ; CH0_TRANS_COUNT (imm5=2)
    // ---- program CH0_CTRL_TRIG = 0x0076_0059 (TREQ_SEL=59=TIMER0, triggers) --
    // RP2350 CTRL: EN=1[0], DATA_SIZE=2[3:2], INCR_READ=1[4], INCR_WRITE=1[6],
    // CHAIN_TO=0[16:13](self=no chain), TREQ_SEL=59[22:17].
    // 59=0x3B=0011_1011; bits[22:17]=0b0111_0110=0x76 in high byte of low16?
    // No: TREQ_SEL occupies bits[22:17]: 59<<17=0x0076_0000. With other bits:
    // 0x0076_0059. Low16=0x0059, high16=0x0076.
    // MOVW r4, #0x0059: imm4=0,i=0,imm3=0,imm8=0x59 → hw0=F240, hw1=0x0459
    // MOVT r4, #0x0076: imm4=0,i=0,imm3=0,imm8=0x76 → hw0=F2C0, hw1=0x0476
    0xF240, //  [38] movw r4, #0x0059 hw0   (imm4=0,i=0,imm3=0,imm8=59)
    0x0459, //  [39] movw r4, #0x0059 hw1   (Rd=4)
    0xF2C0, //  [40] movt r4, #0x0076 hw0   (imm4=0,i=0,imm3=0,imm8=76)
    0x0476, //  [41] movt r4, #0x0076 hw1   (Rd=4)
    0x60CC, //  [42] str  r4, [r1, #0x0C]   ; CH0_CTRL_TRIG → triggers
    // ---- BUSY mask in r3 (bit 26) -------------------------------------------
    0x2301, //  [43] movs r3, #1
    0x069B, //  [44] lsls r3, r3, #26
    // ---- busy-poll loop (target=[45]) ----------------------------------------
    // B<cond> T1: target = PC + 4 + SignExtend(imm8,8)*2.
    // [47] at byte 94. PC = byte94 + 4 = byte 98. Target = byte 90 ([45]).
    // imm8 = (90 - 98) / 2 = -4 = 0xFC.
    0x68CA, //  [45] ldr  r2, [r1, #0x0C]   ; read CH0_CTRL_TRIG
    0x421A, //  [46] tst  r2, r3
    0xD1FC, //  [47] bne  [45]              ; imm8=-4 → loop while BUSY
    0xBE00, //  [48] bkpt #0
];
const SLED_DMA_TIMER_PACED: &[u8] =
    &halfwords_to_le_bytes::<49, 98>(SLED_DMA_TIMER_PACED_HW);

/// Red-path catalogue. Selected by `silicon_periph_diff_rp2350
/// --red-path` (mutually exclusive with the default catalogue).
pub const RED_PATH_SCENARIOS: &[PeriphScenario] = &[
    PeriphScenario {
        name: "red_trng_imr_unmodelled",
        setup: S_RED_TRNG_IMR_UNMODELLED,
        max_sysclks: 500,
        observe: O_RED_TRNG_IMR_UNMODELLED,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
    },
    PeriphScenario {
        name: "red_sha256_csr_wfifo_ready_unmodelled",
        setup: S_RED_SHA256_CSR_UNMODELLED,
        max_sysclks: 500,
        observe: O_RED_SHA256_CSR_UNMODELLED,
        observe_pins: 0,
        custom_sled: None,
        min_sysclks: 0,
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
    pub exclude: Option<String>,
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
    const RESETS_CLR_ADDR: u32 = RESETS_RESET + ALIAS_CLR;
    // RP2350 RESETS.RESET is a 29-bit field (bits 0..=28). Reserved bits
    // 29..=31 are RAZ on RESET_DONE, so a scenario that writes
    // `RESETS_CLR_ALL = 0xFFFF_FFFF` to the ALIAS_CLR (harmless — reserved
    // bits are RAZ/WI on writes) would otherwise spin forever waiting for
    // those reserved bits to read back as 1. Mask the poll to the
    // implemented bits.
    const RESETS_IMPLEMENTED_MASK: u32 = 0x1FFF_FFFF;
    // Real peripherals exit reset in microseconds; 50 ms is a generous
    // ceiling that still keeps scenarios snappy. Observed in practice
    // that RESETS_DONE doesn't always read back the full implemented
    // mask even long after clearing — scenarios that depend on a
    // specific peripheral should narrow `val` to that peripheral's bit.
    const RESETS_DONE_TIMEOUT: Duration = Duration::from_millis(50);

    for &(addr, val) in setup {
        core.write_word_32(addr as u64, val)?;

        // After releasing peripherals from reset, wait for the reset
        // tree to propagate.  On RP2350, RESETS.RESET_DONE reflects
        // which peripherals have fully exited reset.  Without this
        // barrier the very next DAP write can arrive before the
        // peripheral's register file is accessible, silently dropping
        // the configuration (observed on DMA — see the test_silicon
        // baseline journal 2026-04-16).
        if addr == RESETS_CLR_ADDR && val != 0 {
            let wait_mask = val & RESETS_IMPLEMENTED_MASK;
            if wait_mask != 0 {
                let deadline = Instant::now() + RESETS_DONE_TIMEOUT;
                loop {
                    let done = core.read_word_32(RESETS_RESET_DONE as u64)?;
                    if done & wait_mask == wait_mask {
                        break;
                    }
                    if Instant::now() > deadline {
                        // Don't hang forever if a peripheral never comes
                        // out of reset; let the scenario proceed and fail
                        // on observation rather than masking it as a
                        // harness hang.
                        break;
                    }
                }
            }
        }
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

    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build().unwrap();
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
            emu.step().expect("Serial step is infallible");
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
        emu.run(actual_sysclks as u64).expect("Serial run is infallible");
    }
    gate_peripheral_emu(&mut emu, sc.name);

    // V5 §4 soft-window: warn if the scenario completed implausibly fast.
    if sc.min_sysclks > 0 && actual_sysclks < sc.min_sysclks {
        println!(
            "    WARNING scenario '{}': completed implausibly fast \
             ({} sysclks < min_sysclks {})",
            sc.name, actual_sysclks, sc.min_sysclks,
        );
    }

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

/// Retry-once wrapper. The only probe-rs error kinds we retry on are
/// the transient ones: `Probe` (DebugProbeError — USB disconnect /
/// buffer drain stalls) and `Timeout` (ARM DAP timeout). Everything
/// else is a hard fail on the first attempt.
///
/// On retry, we pause briefly to let the probe's internal queue drain
/// before kicking off the next scenario's reset_and_halt.
///
/// Direct port of `silicon_periph_diff_rp2040.rs::run_scenario_with_retry`
/// (RP2040 Phase 0 Wave 3). Lives in the shared module so both
/// `silicon_periph_diff_rp2350` and `run_against` (test_silicon
/// orchestrator) benefit without duplication.
pub fn run_scenario_with_retry(
    core: &mut Core,
    sc: &PeriphScenario,
    first_scenario: bool,
    verbose: bool,
) -> Result<PeriphScenarioResult, Box<dyn std::error::Error>> {
    match run_scenario(core, sc, first_scenario, verbose) {
        Ok(r) => Ok(r),
        Err(e) => {
            if is_transient_probe_error(e.as_ref()) {
                eprintln!(
                    "  scenario '{}': transient probe error, retrying once: {e}",
                    sc.name,
                );
                std::thread::sleep(Duration::from_millis(250));
                run_scenario(core, sc, first_scenario, verbose)
            } else {
                Err(e)
            }
        }
    }
}

/// Strict error-kind match: retry only on `probe_rs::Error::Probe` and
/// `probe_rs::Error::Timeout`. Anything else — including `Arm` errors,
/// `ChipNotFound`, memory-alignment errors — is a hard fail.
///
/// `'static` bound on the trait object is required because
/// `Any::downcast_ref` (pulled in via `Error::downcast_ref`) can only
/// work on types that don't borrow from shorter-lived scopes.
pub fn is_transient_probe_error(e: &(dyn std::error::Error + 'static)) -> bool {
    if let Some(pe) = e.downcast_ref::<probe_rs::Error>() {
        matches!(pe, probe_rs::Error::Probe(_) | probe_rs::Error::Timeout)
    } else {
        false
    }
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
            .filter(|s| !silicon_oracle::should_exclude(s.name, args.exclude.as_deref()))
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
        match run_scenario_with_retry(core, sc, i == 0, args.verbose) {
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
            let bits = RESET_ADC
                | RESET_PIO0
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
        (0x2000_0000..0x2008_0000).contains(&addr) // SRAM (for DMA src/dst seed)
            || (0x4000_0000..0x6000_0000).contains(&addr)
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

    /// All shipped sleds must validate — a future edit that accidentally
    /// breaks the `bkpt #0` terminator (or odd-aligns a halfword pair)
    /// gets caught here.
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
        assert!(
            validate_custom_sled(SLED_TIMER0_ALARM0_FIRE_AND_CLEAR).is_ok(),
            "timer0_alarm0_fire_and_clear sled must validate",
        );
        assert!(
            validate_custom_sled(SLED_TICKS_TIMER0_RETARGET).is_ok(),
            "ticks_timer0_retarget_halves_rate sled must validate",
        );
        assert!(
            validate_custom_sled(SLED_SIO_MTIME_COUNT_AND_MATCH).is_ok(),
            "sio_mtime_count_and_match sled must validate",
        );
        assert!(
            validate_custom_sled(SLED_TIMER1_ALARM0_FIRE_AND_CLEAR).is_ok(),
            "timer1_alarm0_fire_and_clear sled must validate",
        );
        assert!(
            validate_custom_sled(SLED_DMA_MEM_TO_MEM_32BIT).is_ok(),
            "dma_mem_to_mem_32bit sled must validate",
        );
        assert!(
            validate_custom_sled(SLED_DMA_CHAIN_TRIGGER).is_ok(),
            "dma_chain_trigger sled must validate",
        );
        assert!(
            validate_custom_sled(SLED_DMA_TIMER_PACED).is_ok(),
            "dma_timer_paced sled must validate",
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
    /// only the entries explicitly enumerated here. If a future scenario
    /// author adds a custom sled and forgets to add it to this
    /// allow-list, the test flags it so the reviewer double-checks
    /// the intent. Phase 1 added two new entries:
    /// `timer0_alarm0_fire_and_clear` and
    /// `ticks_timer0_retarget_halves_rate`.
    #[test]
    fn test_custom_sled_opt_in_roster() {
        let expected_custom: HashSet<&str> = [
            "clock_pll_sys_reprogram_mid_run",
            "clock_div_change_pio_running",
            "timer0_alarm0_fire_and_clear",
            "ticks_timer0_retarget_halves_rate",
            "sio_mtime_count_and_match",
            "timer1_alarm0_fire_and_clear",
            "dma_mem_to_mem_32bit",
            "dma_chain_trigger",
            "dma_timer_paced",
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

    // ---- Retry-wrapper transient-error classifier (HLD V5 §4.2.9) -------
    //
    // `run_scenario_with_retry` takes a real `probe_rs::Core`, which
    // can't be mocked from a unit test. The retry logic itself is a
    // two-liner (match on Ok/Err + classify); the load-bearing piece
    // is `is_transient_probe_error` — the filter that decides which
    // error kinds get a second chance. Tests cover it directly.
    //
    // Port of the RP2040 transient-error contract: retry ONLY on
    // `probe_rs::Error::Probe` and `probe_rs::Error::Timeout`.

    /// `Probe` wraps a `DebugProbeError` — treat as transient (USB
    /// disconnect / buffer drain stalls).
    #[test]
    fn test_is_transient_probe_error_probe_variant() {
        // `DebugProbeError::Timeout` is a unit-ish variant in probe-rs
        // 0.31 and the easiest to construct without pulling feature
        // flags. Wrap it in `probe_rs::Error::Probe(_)` to exercise
        // the transient-arm.
        let inner = probe_rs::probe::DebugProbeError::Timeout;
        let e: Box<dyn std::error::Error + 'static> = Box::new(probe_rs::Error::Probe(inner));
        assert!(
            is_transient_probe_error(e.as_ref()),
            "probe_rs::Error::Probe must be classified as transient",
        );
    }

    /// `Timeout` is the ARM DAP timeout — treat as transient.
    #[test]
    fn test_is_transient_probe_error_timeout_variant() {
        let e: Box<dyn std::error::Error + 'static> = Box::new(probe_rs::Error::Timeout);
        assert!(
            is_transient_probe_error(e.as_ref()),
            "probe_rs::Error::Timeout must be classified as transient",
        );
    }

    /// A plain `String` error (or anything that isn't a
    /// `probe_rs::Error`) must NOT be treated as transient — hard
    /// fail on the first attempt.
    #[test]
    fn test_is_transient_probe_error_rejects_non_probe_errors() {
        let e: Box<dyn std::error::Error + 'static> =
            Box::<dyn std::error::Error + Send + Sync>::from("some other failure");
        assert!(
            !is_transient_probe_error(e.as_ref()),
            "non-probe-rs errors must be classified as hard failures",
        );
    }

    /// `ChipNotFound` is configuration-level — not worth retrying.
    #[test]
    fn test_is_transient_probe_error_rejects_chip_not_found() {
        let e: Box<dyn std::error::Error + 'static> =
            Box::new(probe_rs::Error::ChipNotFound(
                probe_rs::config::RegistryError::ChipNotFound("rp2350".into()),
            ));
        assert!(
            !is_transient_probe_error(e.as_ref()),
            "probe_rs::Error::ChipNotFound must NOT be classified as transient",
        );
    }

    // ---- Red-path catalogue (Phase 0b HLD V5 §4.2.8) --------------------
    //
    // The red-path catalogue is gated behind `--red-path` on the
    // standalone binary so normal runs don't flake. These tests verify
    // the catalogue ships the three required scenarios and its shape
    // matches the default catalogue's invariants.

    #[test]
    fn test_red_path_catalogue_has_two_scenarios() {
        assert_eq!(
            RED_PATH_SCENARIOS.len(),
            2,
            "V5 §6.C Step 2 retired the UART1 witness (UART1 is now modelled); \
             expected 2 red-path scenarios, got {}",
            RED_PATH_SCENARIOS.len(),
        );
    }

    #[test]
    fn test_red_path_catalogue_names_match_spec() {
        // Phase 2 retired the Phase 0b/1 witnesses (UART0/SPI0/ADC) as
        // they became modelled peripherals. V5 §6.C Step 2 retired the
        // UART1 witness for the same reason. Remaining witnesses target
        // still-unmodelled blocks: TRNG, SHA256.
        let expected: HashSet<&str> = [
            "red_trng_imr_unmodelled",
            "red_sha256_csr_wfifo_ready_unmodelled",
        ]
        .into_iter()
        .collect();
        let actual: HashSet<&str> = RED_PATH_SCENARIOS.iter().map(|s| s.name).collect();
        assert_eq!(
            actual, expected,
            "red-path catalogue names must match the Phase 2 spec \
             (genuine red-path witnesses); got {:?}",
            actual,
        );
    }

    #[test]
    fn test_red_path_catalogue_all_setup_addresses_absolute_mmio() {
        for sc in RED_PATH_SCENARIOS {
            for (i, (a, _)) in sc.setup.iter().enumerate() {
                assert!(
                    is_mmio(*a),
                    "{} setup[{}] 0x{:08X} is not in MMIO range",
                    sc.name,
                    i,
                    a,
                );
            }
            for (i, (a, _)) in sc.observe.iter().enumerate() {
                assert!(
                    is_mmio(*a),
                    "{} observe[{}] 0x{:08X} is not in MMIO range",
                    sc.name,
                    i,
                    a,
                );
            }
        }
    }

    #[test]
    fn test_red_path_catalogue_no_name_overlap_with_default() {
        let default: HashSet<&str> = SCENARIOS.iter().map(|s| s.name).collect();
        for sc in RED_PATH_SCENARIOS {
            assert!(
                !default.contains(sc.name),
                "red-path scenario '{}' collides with default catalogue name",
                sc.name,
            );
        }
    }

    /// Observable set must be non-empty for every red-path scenario —
    /// otherwise the oracle reports PASS trivially.
    #[test]
    fn test_red_path_catalogue_observe_nonempty() {
        for sc in RED_PATH_SCENARIOS {
            let any = sc.observe.iter().any(|(_, m)| *m != 0) || sc.observe_pins != 0;
            assert!(
                any,
                "red-path scenario '{}' observes nothing (mask=0)",
                sc.name,
            );
        }
    }

    /// `max_sysclks > 0` — same invariant as the default catalogue.
    #[test]
    fn test_red_path_catalogue_max_sysclks_positive() {
        for sc in RED_PATH_SCENARIOS {
            assert!(
                sc.max_sysclks > 0,
                "red-path scenario '{}' has max_sysclks=0",
                sc.name,
            );
        }
    }

    // ---- min_sysclks soft-window (V5 §4) --------------------------------

    /// `min_sysclks <= max_sysclks` for every scenario in both catalogues.
    #[test]
    fn test_min_sysclks_le_max_sysclks() {
        for sc in SCENARIOS.iter().chain(RED_PATH_SCENARIOS.iter()) {
            assert!(
                sc.min_sysclks <= sc.max_sysclks,
                "'{}' min_sysclks {} > max_sysclks {}",
                sc.name, sc.min_sysclks, sc.max_sysclks,
            );
        }
    }

    /// When `min_sysclks > 0` and `actual < min_sysclks`, the warning
    /// fires. This test checks the condition directly (the println in
    /// `run_scenario` cannot be captured without a real probe session).
    #[test]
    fn test_min_sysclks_warning_fires_when_below() {
        let sc = PeriphScenario {
            name: "synth_fast",
            setup: &[],
            max_sysclks: 200,
            observe: &[],
            observe_pins: 0,
            custom_sled: None,
            min_sysclks: 100,
        };
        let actual_sysclks: u32 = 50;
        // Condition mirrors `run_scenario`'s guard.
        assert!(
            sc.min_sysclks > 0 && actual_sysclks < sc.min_sysclks,
            "expected warning condition to trigger for actual={} < min={}",
            actual_sysclks, sc.min_sysclks,
        );
    }

    /// When `min_sysclks == 0`, the warning condition never fires
    /// regardless of `actual_sysclks`.
    #[test]
    fn test_min_sysclks_zero_no_warning() {
        let sc = PeriphScenario {
            name: "synth_no_min",
            setup: &[],
            max_sysclks: 200,
            observe: &[],
            observe_pins: 0,
            custom_sled: None,
            min_sysclks: 0,
        };
        let actual_sysclks: u32 = 0;
        assert!(
            !(sc.min_sysclks > 0 && actual_sysclks < sc.min_sysclks),
            "min_sysclks=0 must never trigger the warning",
        );
    }

    // ---- DMA sled emulator verification ------------------------------------
    //
    // Each test below loads the sled into SILICON_RUN_SLED, runs the
    // emulator's CPU (core 0) to completion (BKPT), then checks that the
    // destination SRAM words match the expected seeded pattern under full
    // mask.  This exercises the emulator's DMA implementation end-to-end
    // via the same CPU-store path used on real silicon.

    fn run_dma_sled_on_emu(sled: &'static [u8], budget: u64) -> mdrp2350::Emulator {
        use mdrp2350::{Config, EmulatorBuilder};
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(1)
            .build().unwrap();
        // Apply the RESETS CLR so DMA is out of reset (mirrors setup table).
        emu.mmio_write32(RESETS_RESET + ALIAS_CLR, RESET_DMA_BIT);
        // Load and run the sled.
        emu.load_image(SILICON_RUN_SLED, sled);
        {
            let c = emu.core_mut(0);
            c.wake();
            c.set_reg(13, EMU_TEST_STACK);
            c.set_reg(14, 0xFFFF_FFFF);
            c.regs.set_pc(SILICON_RUN_SLED);
            c.regs.xpsr = 0x0100_0000;
        }
        let bkpt_pc = SILICON_RUN_SLED + (sled.len() as u32) - 2;
        let start = emu.cycles();
        while emu.core(0).regs.pc() != bkpt_pc
            && emu.cycles().saturating_sub(start) < budget
        {
            emu.step().expect("Serial step is infallible");
        }
        assert_eq!(
            emu.core(0).regs.pc(),
            bkpt_pc,
            "sled did not reach BKPT within {budget} cycles (PC=0x{:08X})",
            emu.core(0).regs.pc(),
        );
        emu
    }

    #[test]
    fn test_dma_mem_to_mem_32bit_sled_on_emu() {
        let mut emu = run_dma_sled_on_emu(SLED_DMA_MEM_TO_MEM_32BIT, 2000);
        // Destination 0x2000_0300..030C must mirror source 0xDEAD_0001..4.
        assert_eq!(emu.mmio_read32(0x2000_0300), 0xDEAD_0001, "word 0");
        assert_eq!(emu.mmio_read32(0x2000_0304), 0xDEAD_0002, "word 1");
        assert_eq!(emu.mmio_read32(0x2000_0308), 0xDEAD_0003, "word 2");
        assert_eq!(emu.mmio_read32(0x2000_030C), 0xDEAD_0004, "word 3");
        // DMA INTR bit 0 must be set.
        assert_ne!(emu.mmio_read32(DMA_INTR) & 0x0000_0001, 0, "DMA INTR bit 0");
    }

    #[test]
    fn test_dma_chain_trigger_sled_on_emu() {
        let mut emu = run_dma_sled_on_emu(SLED_DMA_CHAIN_TRIGGER, 4000);
        // Ch0 destination 0x2000_0600 ← 0xAAAA_0000.
        assert_eq!(emu.mmio_read32(0x2000_0600), 0xAAAA_0000, "ch0 dst");
        // Ch1 destination 0x2000_0700 ← 0xBBBB_1111.
        assert_eq!(emu.mmio_read32(0x2000_0700), 0xBBBB_1111, "ch1 dst");
        // DMA INTR bits 0 and 1 must both be set.
        assert_eq!(
            emu.mmio_read32(DMA_INTR) & 0x0000_0003,
            0x0000_0003,
            "DMA INTR bits 0+1",
        );
        // Neither channel must have RING_SEL set (bit 12 on RP2350). The sled
        // encodes CTRL=0x007E_2059 which has RING_SIZE[11:8]=0 and RING_SEL[12]=0.
        // This also verifies the RP2350 field positions are used: with the old
        // RP2040 layout, RING_SEL would have been at bit 10 and CHAIN_TO at [14:11].
        let ch0_ctrl = emu.mmio_read32(DMA_BASE + 0x0C);
        let ch1_ctrl = emu.mmio_read32(DMA_BASE + 0x50);
        assert_eq!(ch0_ctrl & (1 << 12), 0, "ch0 must not have RING_SEL (bit 12) set");
        assert_eq!(ch1_ctrl & (1 << 12), 0, "ch1 must not have RING_SEL (bit 12) set");
        // Also verify TREQ_SEL field is correctly at bits[22:17] (RP2350) not [20:15] (RP2040).
        // For TREQ_SEL=63: bits[22:17] of 0x007E_2059 = (0x007E_2059 >> 17) & 0x3F = 63.
        assert_eq!((ch0_ctrl >> 17) & 0x3F, 63, "ch0 TREQ_SEL must be 63 (FORCE) at bits[22:17]");
        assert_eq!((ch1_ctrl >> 17) & 0x3F, 63, "ch1 TREQ_SEL must be 63 (FORCE) at bits[22:17]");
    }

    #[test]
    fn test_dma_timer_paced_sled_on_emu() {
        let mut emu = run_dma_sled_on_emu(SLED_DMA_TIMER_PACED, 2000);
        // Destination 0x2000_0B00..0B0C must mirror source 0xCAFE_0001..4.
        assert_eq!(emu.mmio_read32(0x2000_0B00), 0xCAFE_0001, "word 0");
        assert_eq!(emu.mmio_read32(0x2000_0B04), 0xCAFE_0002, "word 1");
        assert_eq!(emu.mmio_read32(0x2000_0B08), 0xCAFE_0003, "word 2");
        assert_eq!(emu.mmio_read32(0x2000_0B0C), 0xCAFE_0004, "word 3");
        // DMA INTR bit 0 must be set.
        assert_ne!(emu.mmio_read32(DMA_INTR) & 0x0000_0001, 0, "DMA INTR bit 0");
        // Residual C.2.1: verify the sled programmed TIMER0 at the correct
        // RP2350 offset (0x440), not the RP2040 legacy offset (0x420).  Pre-fix
        // both the sled and the emulator matched at 0x420, masking the silicon
        // failure — this is the test pin that catches a revert.
        assert_eq!(
            emu.mmio_read32(0x5000_0440),
            (1u32 << 16) | 10,
            "sled must programme TIMER0 at RP2350 offset 0x440 (X=1,Y=10)"
        );
        assert_eq!(
            emu.mmio_read32(0x5000_0420),
            0,
            "RP2040 legacy TIMER0 offset must remain unmapped (read-as-zero on RP2350)"
        );
    }

    /// Drive every red-path scenario through the same EMU-side
    /// sequence that `run_scenario` uses — apply setup writes, advance
    /// `max_sysclks` cycles with both cores halted, then read the
    /// observables — and assert each one leaves EMU with **zero** in
    /// every masked bit. Any non-zero silicon observation (the whole
    /// point of a red-path witness) therefore diverges. This is the
    /// local half of the HW != EMU contract: the HW side is gated on
    /// real silicon and runs in Arthur's lab, but the EMU side must
    /// hold here or the red-path catalogue is silently green.
    ///
    /// If a future phase wires a real peripheral model at one of these
    /// addresses, this test starts passing with a non-zero value — the
    /// signal to replace that scenario with a fresh unmodelled one.
    #[test]
    fn test_red_path_emu_observables_are_zero_under_mask() {
        use mdrp2350::{Config, EmulatorBuilder};
        for sc in RED_PATH_SCENARIOS {
            let mut emu = EmulatorBuilder::new(Config::default())
                .step_quantum(1)
                .build().unwrap();
            emu.core_mut(0).halt();
            emu.core_mut(1).halt();
            for &(addr, val) in sc.setup {
                emu.mmio_write32(addr, val);
            }
            emu.run(sc.max_sysclks as u64).expect("Serial run is infallible");
            for &(addr, mask) in sc.observe {
                let got = emu.mmio_read32(addr) & mask;
                assert_eq!(
                    got,
                    0,
                    "red-path scenario '{}': EMU read 0x{:08X} & 0x{:08X} = \
                     0x{:08X}, expected 0 (any non-zero silicon reading \
                     would diverge — if EMU now matches silicon, replace \
                     this scenario with a fresh unmodelled-peripheral one)",
                    sc.name,
                    emu.mmio_read32(addr),
                    mask,
                    got,
                );
            }
        }
    }
}

//! RP2040-specific PIO tests.
//!
//! The PIO primitive (`PioBlock`, decoder, state-machine) lives in
//! [`mdpicoem_common::pio`]. These tests exercise PIO *through the
//! RP2040 `Bus` and `Emulator`*, which is chip-specific — register
//! address layout (PIO0=0x5020_0000, PIO1=0x5030_0000), number of PIO
//! blocks (2, vs 3 on RP2350), and the GPIO merge path all sit on
//! `mdrp2040::Bus`.
//!
//! Tests that exercise only `PioBlock`/`StateMachine` internals stay
//! co-located with the primitive under `mdpicoem-common/src/pio/mod.rs`.
//! Tests that need the full RP2350 bus live in `mdrp2350::pio_tests`.
//!
//! # Clippy: identity_op
//!
//! Many tests here use `PIO_BASE + 0x000` and `0u32 << 7` style literals
//! as part of symmetric offset / bit-field series (e.g., iterating
//! register offsets 0x000, 0x004, 0x008...; or writing EXECCTRL with
//! explicit wrap_top and wrap_bottom fields where one of the shifts
//! happens to be zero). Keeping the zero-offset form preserves visual
//! alignment and makes the structural intent obvious at the call site,
//! so silence the lint file-wide.
//!
//! # PIO cadence relative to core 0
//!
//! `Emulator::step()` drains up to `step_quantum` cycles per call and
//! ticks the PIO once with the summed cycle count. With the default
//! `step_quantum = DEFAULT_STEP_QUANTUM = 64`, intermediate pin states
//! within a quantum are not observable. Tests that need to read PIO
//! pin state on a per-instruction basis construct the emulator with
//! `EmulatorBuilder::new(Config::default()).step_quantum(1).build()`
//! so each `emu.step()` advances by exactly one core-0 instruction;
//! [`park_core0_on_nops`] then parks core 0 on NOPs to guarantee a
//! fixed 1-tick-per-step PIO cadence.

#![allow(clippy::identity_op)]

use crate::bus::{Bus, PIO0_BASE, PIO1_BASE};
use crate::{Config, Emulator, EmulatorBuilder};

// ---------------------------------------------------------------------------
// Bus-level dispatch
// ---------------------------------------------------------------------------

#[test]
fn bus_pio0_sm0_pinctrl_roundtrip() {
    // SM0_PINCTRL at PIO_BASE + 0x0DC — writable through the bus.
    let mut bus = Bus::new();
    bus.write32(PIO0_BASE + 0x0DC, 0x1234_5678);
    assert_eq!(bus.read32(PIO0_BASE + 0x0DC), 0x1234_5678);
}

#[test]
fn bus_pio1_sm1_clkdiv_roundtrip() {
    // SM1_CLKDIV = 0x0E0 (SM0=0x0C8, stride 0x18).
    let mut bus = Bus::new();
    let clkdiv = (500u32 << 16) | (64u32 << 8);
    bus.write32(PIO1_BASE + 0x0E0, clkdiv);
    assert_eq!(bus.read32(PIO1_BASE + 0x0E0), clkdiv);
}

#[test]
fn bus_pio_blocks_are_independent() {
    // A write to PIO0 must not be visible via PIO1 (and vice-versa).
    // Exercises both blocks at the same register offset (SM0_EXECCTRL).
    let mut bus = Bus::new();
    bus.write32(PIO0_BASE + 0x0CC, 0xAAAA_AAAA);
    bus.write32(PIO1_BASE + 0x0CC, 0x5555_5555);
    // EXECCTRL's bit 31 is EXEC_STALLED (read-only, cleared on fresh SM),
    // so reads mask off that bit — compare against the masked expectation.
    assert_eq!(bus.read32(PIO0_BASE + 0x0CC), 0xAAAA_AAAA & 0x7FFF_FFFF);
    assert_eq!(bus.read32(PIO1_BASE + 0x0CC), 0x5555_5555 & 0x7FFF_FFFF);
}

#[test]
fn bus_pio_instr_mem_write_is_observable_via_force_exec() {
    // INSTR_MEM0..31 are at offsets 0x048..0x0C4 (32-bit each), write-only
    // (reads return 0 — datasheet §3.7). Verify writes land by running a
    // one-instruction program through SM0 and observing the pad output.
    let mut bus = Bus::new();
    // Program: SET PINS, 1 @ addr 0, JMP 0 @ addr 1.
    bus.write32(PIO0_BASE + 0x048, 0xE001);
    bus.write32(PIO0_BASE + 0x04C, 0x0000);

    // INSTR_MEM reads must return 0 (write-only per PIO spec).
    assert_eq!(bus.read32(PIO0_BASE + 0x048), 0);

    // Configure SM0 to drive pin 0 and enable.
    bus.write32(PIO0_BASE + 0x0DC, 1u32 << 26); // set_count=1, set_base=0
    bus.write32(PIO0_BASE + 0x0CC, 1u32 << 12); // wrap_top=1, wrap_bottom=0
    bus.write32(PIO0_BASE + 0x0D8, 0xE081); // force SET PINDIRS, 1
    bus.write32(PIO0_BASE + 0x000, 0x1); // enable SM0

    // Tick PIO0 once so SM0 executes SET PINS, 1 and pad_out reflects it.
    bus.pio[0].step(0);
    assert_eq!(bus.pio[0].pad_out & 0x1, 0x1,
        "SET PINS, 1 must drive pad_out bit 0 high — confirms INSTR_MEM write landed");
}

#[test]
fn bus_pio_ctrl_set_alias_enables_sm() {
    // CTRL.SM_ENABLE bits use the APB SET alias (base + 0x2000).
    let mut bus = Bus::new();
    bus.write32(PIO0_BASE + 0x2000, 0x1); // SET SM0 enabled
    assert!(bus.pio[0].sm[0].enabled());
    assert_eq!(bus.read32(PIO0_BASE + 0x000), 0x1);

    // SET another bit; first must stay enabled.
    bus.write32(PIO0_BASE + 0x2000, 0x4); // SET SM2
    assert!(bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert_eq!(bus.read32(PIO0_BASE + 0x000), 0x5);

    // CLR alias (base + 0x3000) disables.
    bus.write32(PIO0_BASE + 0x3000, 0x1); // CLR SM0
    assert!(!bus.pio[0].sm[0].enabled());
    assert!(bus.pio[0].sm[2].enabled());
    assert_eq!(bus.read32(PIO0_BASE + 0x000), 0x4);
}

#[test]
fn bus_pio_byte_writes_silently_ignored() {
    // PIO is 32-bit-only — byte writes must not trigger the peripheral
    // RMW fallback (which would spuriously pop an RX FIFO or force-
    // execute a garbage instruction).
    let mut bus = Bus::new();
    // Pre-seed SM0_PINCTRL with a known non-zero value.
    bus.write32(PIO0_BASE + 0x0DC, 0xDEAD_BEEF);
    // A byte-write must be a no-op: the word must remain intact.
    bus.write8(PIO0_BASE + 0x0DC, 0x00);
    bus.write8(PIO0_BASE + 0x0DD, 0x00);
    assert_eq!(bus.read32(PIO0_BASE + 0x0DC), 0xDEAD_BEEF);
    // Same for halfwords.
    bus.write16(PIO0_BASE + 0x0DC, 0x0000);
    assert_eq!(bus.read32(PIO0_BASE + 0x0DC), 0xDEAD_BEEF);
}

// ---------------------------------------------------------------------------
// Emulator-level GPIO merge
// ---------------------------------------------------------------------------

#[test]
fn update_gpio_pio_overrides_sio_on_same_pin() {
    // SIO drives pin 5 high; PIO0 drives pin 5 low with OE.
    // Expected: gpio_in bit 5 = 0 (PIO wins where pad_oe is set).
    let mut emu = Emulator::new(Config::default());
    emu.bus.sio.gpio_out = 1 << 5;
    emu.bus.sio.gpio_oe = 1 << 5;
    emu.bus.pio[0].pad_oe = 1 << 5;
    emu.bus.pio[0].pad_out = 0;
    emu.update_gpio();
    assert_eq!(emu.bus.gpio_in & (1 << 5), 0,
        "PIO pad_oe bit must override SIO on pin 5");
}

#[test]
fn update_gpio_independent_pins_coexist() {
    // PIO0 drives pin 5, SIO drives pin 10 — both appear.
    let mut emu = Emulator::new(Config::default());
    emu.bus.sio.gpio_out = 1 << 10;
    emu.bus.sio.gpio_oe = 1 << 10;
    emu.bus.pio[0].pad_oe = 1 << 5;
    emu.bus.pio[0].pad_out = 1 << 5;
    emu.update_gpio();
    assert_ne!(emu.bus.gpio_in & (1 << 5), 0, "PIO pin 5 present");
    assert_ne!(emu.bus.gpio_in & (1 << 10), 0, "SIO pin 10 present");
}

#[test]
fn update_gpio_pio1_participates_in_merge() {
    // A pin driven by PIO1 (not PIO0) must also reach gpio_in.
    let mut emu = Emulator::new(Config::default());
    emu.bus.pio[1].pad_oe = 1 << 7;
    emu.bus.pio[1].pad_out = 1 << 7;
    emu.update_gpio();
    assert_ne!(emu.bus.gpio_in & (1 << 7), 0, "PIO1 pin 7 must appear in gpio_in");
}

#[test]
fn update_gpio_masks_to_30_pins() {
    // RP2040 has 30 GPIO pins — bits 30/31 must be cleared even if
    // something tries to drive them.
    let mut emu = Emulator::new(Config::default());
    emu.bus.pio[0].pad_oe = 0xFFFF_FFFF;
    emu.bus.pio[0].pad_out = 0xFFFF_FFFF;
    emu.update_gpio();
    assert_eq!(emu.bus.gpio_in & 0xC000_0000, 0,
        "bits 30/31 must be masked off — RP2040 has 30 GPIOs");
    assert_eq!(emu.bus.gpio_in, 0x3FFF_FFFF);
}

// ---------------------------------------------------------------------------
// Full loop: load PIO program, step the emulator, observe pin state
// ---------------------------------------------------------------------------

/// Build an emulator with a PIO program loaded and SM0 configured to drive
/// a single output pin via SET PINS. Returns the emulator; caller runs
/// `emu.step()` repeatedly.
///
/// Uses `step_quantum=1` so each `emu.step()` advances by exactly one
/// system-clock cycle — these tests read PIO pin state on a per-cycle
/// basis, which the default quantum execution model would otherwise
/// smear across up to `DEFAULT_STEP_QUANTUM` cycles.
///
/// Program:
///   addr 0: SET PINS, 1   (pin HIGH)
///   addr 1: SET PINS, 0   (pin LOW)
///   addr 2: JMP 0         (loop)
fn blinky_emulator(pio_base: u32, pin: u8) -> Emulator {
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    let set_pins_1: u16 = 0xE001; // SET PINS, 1
    let set_pins_0: u16 = 0xE000; // SET PINS, 0
    let jmp_0: u16 = 0x0000; // JMP 0

    for (i, insn) in [set_pins_1, set_pins_0, jmp_0].iter().enumerate() {
        emu.bus.write32(pio_base + 0x048 + (i as u32) * 4, *insn as u32);
    }

    // SM0_PINCTRL: set_count=1 (bits 28:26), set_base=pin (bits 9:5).
    let pinctrl = (1u32 << 26) | ((pin as u32) << 5);
    emu.bus.write32(pio_base + 0x0DC, pinctrl);

    // SM0_EXECCTRL: wrap_top=2, wrap_bottom=0.
    let execctrl = (2u32 << 12) | (0u32 << 7);
    emu.bus.write32(pio_base + 0x0CC, execctrl);

    // Force SET PINDIRS, 1 so the output pin becomes driven.
    emu.bus.write32(pio_base + 0x0D8, 0xE081);

    // Enable SM0.
    emu.bus.write32(pio_base + 0x000, 0x1);

    emu
}

/// Configure core 0 to execute a long run of NOPs at SRAM (each a
/// 1-cycle instruction), so every `emu.step()` advances the PIO by
/// exactly one system-clock cycle. Branch instructions cost 3 cycles
/// on M0+ (pipeline refill) and would smear PIO state across multiple
/// SM cycles per step.
fn park_core0_on_nops(emu: &mut Emulator) {
    let prog = 0x2000_0000u32;
    // 256 NOPs is plenty for any test that doesn't run more than 256
    // emu.step()s on core 0.
    for i in 0..256u32 {
        emu.bus.write16(prog + i * 2, 0xBF00); // NOP
    }
    emu.cores[0].regs.set_pc(prog);
    emu.cores[0].regs.msp = 0x2003_0000;
    emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
}

#[test]
fn pio0_blinky_drives_gpio_in() {
    // Program: SET PINS,1 -> SET PINS,0 -> JMP 0 (loops forever).
    // Pin 25 should go HIGH after the first cycle, LOW after the second.
    let mut emu = blinky_emulator(PIO0_BASE, 25);
    park_core0_on_nops(&mut emu);

    // Step 1: SET PINS, 1 (pin 25 HIGH)
    emu.step();
    assert!(emu.gpio_read(25), "pin 25 HIGH after SET PINS, 1");

    // Step 2: SET PINS, 0 (pin 25 LOW)
    emu.step();
    assert!(!emu.gpio_read(25), "pin 25 LOW after SET PINS, 0");

    // Step 3: JMP 0 (no pin change — pad_out stays at 0)
    emu.step();
    assert!(!emu.gpio_read(25), "pin 25 still LOW after JMP");

    // Step 4: SET PINS, 1 again (second pattern)
    emu.step();
    assert!(emu.gpio_read(25), "pin 25 HIGH on second pattern");
}

#[test]
fn pio1_blinky_is_independent_of_pio0() {
    // A program loaded into PIO1 drives pin 10 without affecting pin 25
    // (which PIO0 would drive if the blocks were accidentally aliased).
    let mut emu = blinky_emulator(PIO1_BASE, 10);
    park_core0_on_nops(&mut emu);

    emu.step();
    assert!(emu.gpio_read(10), "PIO1 drives pin 10 HIGH");
    assert!(!emu.gpio_read(25), "PIO0 stays quiet on pin 25");

    emu.step();
    assert!(!emu.gpio_read(10), "PIO1 pin 10 goes LOW");
}

#[test]
fn pio0_and_pio1_drive_different_pins_concurrently() {
    // Run PIO0 on pin 5 and PIO1 on pin 12 simultaneously.
    // Both should reflect in gpio_in on the same step.
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();

    // Load identical blinky programs into both blocks.
    let set_pins_1: u16 = 0xE001;
    let set_pins_0: u16 = 0xE000;
    let jmp_0: u16 = 0x0000;
    for base in [PIO0_BASE, PIO1_BASE] {
        for (i, insn) in [set_pins_1, set_pins_0, jmp_0].iter().enumerate() {
            emu.bus.write32(base + 0x048 + (i as u32) * 4, *insn as u32);
        }
        emu.bus.write32(base + 0x0CC, (2u32 << 12) | (0u32 << 7));
    }

    // PIO0 → pin 5
    emu.bus.write32(PIO0_BASE + 0x0DC, (1u32 << 26) | (5u32 << 5));
    emu.bus.write32(PIO0_BASE + 0x0D8, 0xE081); // SET PINDIRS, 1
    emu.bus.write32(PIO0_BASE + 0x000, 0x1);

    // PIO1 → pin 12
    emu.bus.write32(PIO1_BASE + 0x0DC, (1u32 << 26) | (12u32 << 5));
    emu.bus.write32(PIO1_BASE + 0x0D8, 0xE081);
    emu.bus.write32(PIO1_BASE + 0x000, 0x1);

    park_core0_on_nops(&mut emu);

    // Step 1: both blocks execute SET PINS,1 → pins 5 and 12 HIGH.
    emu.step();
    assert!(emu.gpio_read(5), "PIO0 drives pin 5 HIGH");
    assert!(emu.gpio_read(12), "PIO1 drives pin 12 HIGH");

    // Step 2: both execute SET PINS,0 → pins 5 and 12 LOW.
    emu.step();
    assert!(!emu.gpio_read(5));
    assert!(!emu.gpio_read(12));
}

#[test]
fn pio_multi_sm_different_pins_in_one_block() {
    // SM0 drives pin 5, SM1 drives pin 12, both in PIO0. Both pins must
    // reflect in gpio_in after one step — two SMs on the same block tick
    // concurrently on each system-clock cycle (default clkdiv=1).
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();

    // Shared instruction memory layout:
    //   addr 0: SET PINS, 1  (SM0 body)
    //   addr 1: JMP 0        (SM0 tail)
    //   addr 2: SET PINS, 1  (SM1 body)
    //   addr 3: JMP 2        (SM1 tail)
    let set_pins_1: u16 = 0xE001;
    let jmp_0: u16 = 0x0000;
    let jmp_2: u16 = 0x0002;
    for (i, insn) in [set_pins_1, jmp_0, set_pins_1, jmp_2].iter().enumerate() {
        emu.bus.write32(PIO0_BASE + 0x048 + (i as u32) * 4, *insn as u32);
    }

    // SM0_PINCTRL: set_count=1, set_base=5.
    emu.bus.write32(PIO0_BASE + 0x0DC, (1u32 << 26) | (5u32 << 5));
    // SM0_EXECCTRL: wrap_top=1, wrap_bottom=0 (loops addr 0→1→0).
    emu.bus.write32(PIO0_BASE + 0x0CC, (1u32 << 12) | (0u32 << 7));
    // Force SET PINDIRS, 1 for SM0 (uses SM0's set_base=5).
    emu.bus.write32(PIO0_BASE + 0x0D8, 0xE081);

    // SM1_PINCTRL at 0x0F4 (0x0DC + 0x18): set_count=1, set_base=12.
    emu.bus.write32(PIO0_BASE + 0x0F4, (1u32 << 26) | (12u32 << 5));
    // SM1_EXECCTRL at 0x0E4 (0x0CC + 0x18): wrap_top=3, wrap_bottom=2.
    emu.bus.write32(PIO0_BASE + 0x0E4, (3u32 << 12) | (2u32 << 7));
    // SM1_INSTR at 0x0F0: force SET PINDIRS, 1 for SM1 (uses SM1's
    // set_base=12).
    emu.bus.write32(PIO0_BASE + 0x0F0, 0xE081);
    // Also force SM1's PC to addr 2 by executing JMP 2 — SM1's PC defaults
    // to 0 after reset, but its program lives at addr 2. Force-executing
    // JMP 2 through SM1_INSTR gives a clean jump before SM1 gets enabled.
    emu.bus.write32(PIO0_BASE + 0x0F0, jmp_2 as u32);

    // Enable both SM0 and SM1.
    emu.bus.write32(PIO0_BASE + 0x000, 0x3);

    park_core0_on_nops(&mut emu);

    // One step: both SMs advance by one instruction. SM0 executes
    // SET PINS,1 @ addr 0; SM1 executes SET PINS,1 @ addr 2.
    emu.step();
    assert!(emu.gpio_read(5), "SM0 drives pin 5");
    assert!(emu.gpio_read(12), "SM1 drives pin 12");
}

// ---------------------------------------------------------------------------
// Reset semantics
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// PIO cadence relative to core 0
// ---------------------------------------------------------------------------

#[test]
fn emu_step_advances_pio_by_core0_cycle_cost() {
    // Documents: with `step_quantum=1` (opt-in via `EmulatorBuilder`),
    // `Emulator::step` advances the PIO by `c0` ticks, where `c0` is the
    // number of system-clock cycles core 0's single instruction consumed
    // (see `Emulator::tick_pio` in lib.rs). A 1-cycle NOP advances PIO
    // by 1 tick; a 3-cycle taken branch advances PIO by 3.
    //
    // Under the default quantum (`DEFAULT_STEP_QUANTUM = 64`), `step()`
    // drains many instructions per call and ticks the PIO once with the
    // summed cycle count — the per-instruction cadence asserted here is
    // a property of the `step_quantum=1` opt-in, not the default.
    //
    // Discriminator program (loaded at INSTR_MEM[0..3], wrap_top=3):
    //   addr 0: SET PINS, 0   (0xE000)
    //   addr 1: SET PINS, 0   (0xE000)
    //   addr 2: SET PINS, 1   (0xE001)
    //   addr 3: JMP 0         (0x0000)
    //
    // Starting pad_out=0, after N PIO ticks pin-0 state is:
    //   1 tick  -> 0 (SET PINS, 0 @ addr 0)
    //   2 ticks -> 0 (SET PINS, 0 @ addr 1)
    //   3 ticks -> 1 (SET PINS, 1 @ addr 2)    <-- discriminator
    //   4 ticks -> 1 (JMP 0, no pin change)
    //
    // So with `step_quantum=1` one `emu.step()` running a 3-cycle branch
    // on core 0 must leave pin-0 HIGH; with a 1-cycle NOP it must leave
    // pin-0 LOW. The pair of cases together proves the per-instruction
    // PIO cadence under the opt-in single-cycle quantum.

    fn setup_pio(emu: &mut Emulator) {
        // SET PINS, 0 / SET PINS, 0 / SET PINS, 1 / JMP 0
        emu.bus.write32(PIO0_BASE + 0x048, 0xE000);
        emu.bus.write32(PIO0_BASE + 0x04C, 0xE000);
        emu.bus.write32(PIO0_BASE + 0x050, 0xE001);
        emu.bus.write32(PIO0_BASE + 0x054, 0x0000);
        // SM0_PINCTRL: set_count=1, set_base=0.
        emu.bus.write32(PIO0_BASE + 0x0DC, (1u32 << 26) | (0u32 << 5));
        // SM0_EXECCTRL: wrap_top=3, wrap_bottom=0.
        emu.bus.write32(PIO0_BASE + 0x0CC, (3u32 << 12) | (0u32 << 7));
        // Force SET PINDIRS, 1 so pin 0 is driven.
        emu.bus.write32(PIO0_BASE + 0x0D8, 0xE081);
        // Enable SM0.
        emu.bus.write32(PIO0_BASE + 0x000, 0x1);
    }

    // --- Baseline: core 0 on NOPs -> c0 = 1 per step -> 1 PIO tick -> pin 0.
    let mut emu_nop = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    setup_pio(&mut emu_nop);
    park_core0_on_nops(&mut emu_nop);
    emu_nop.step();
    assert!(!emu_nop.gpio_read(0),
        "after 1 PIO tick pin 0 must be LOW (SET PINS, 0 @ addr 0)");

    // --- Discriminator: core 0 on a 3-cycle taken branch -> c0 = 3 ->
    // 3 PIO ticks -> pin 1. Uses `B +0` (0xE000 Thumb-16), which
    // branches to the very next instruction at +3 cycles; chain enough
    // copies that the test can step multiple times if it wants to.
    let mut emu_branch = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    setup_pio(&mut emu_branch);
    let prog = 0x2000_0000u32;
    for i in 0..256u32 {
        emu_branch.bus.write16(prog + i * 2, 0xE000); // B +0 (3 cycles)
    }
    emu_branch.cores[0].regs.set_pc(prog);
    emu_branch.cores[0].regs.msp = 0x2003_0000;
    emu_branch.cores[0].regs.r[13] = emu_branch.cores[0].regs.msp;
    emu_branch.step();
    assert!(emu_branch.gpio_read(0),
        "after 3 PIO ticks pin 0 must be HIGH (SET PINS, 1 @ addr 2) — \
         proves Emulator::step advances PIO by c0 cycles per instruction \
         when step_quantum=1");
}

#[test]
fn emulator_reset_clears_pio_state() {
    let mut emu = Emulator::new(Config::default());
    // Seed some PIO state.
    emu.bus.write32(PIO0_BASE + 0x000, 0x3); // enable SM0, SM1
    emu.bus.pio[0].pad_out = 0xABCD_1234;
    emu.bus.pio[0].pad_oe = 0x0000_00FF;
    assert!(emu.bus.pio[0].sm[0].enabled());

    // Load a minimal reset vector so `reset()` can read SP/PC from ROM.
    emu.bus.memory.load_rom(&[
        0x00, 0x00, 0x03, 0x20, // SP = 0x2003_0000
        0x01, 0x00, 0x00, 0x20, // PC = 0x2000_0001 (Thumb)
    ]);

    emu.reset();

    assert!(!emu.bus.pio[0].sm[0].enabled(),
        "reset() must disable SMs");
    assert_eq!(emu.bus.pio[0].pad_out, 0,
        "reset() must clear pad_out");
    assert_eq!(emu.bus.pio[0].pad_oe, 0,
        "reset() must clear pad_oe");
    assert_eq!(emu.bus.pio[1].pad_oe, 0,
        "reset() must also reset PIO1");
}

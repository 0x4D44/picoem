//! End-to-end smoke test — a tiny hand-assembled M0+ program writes to
//! SIO GPIO_OE + GPIO_OUT via the real bus path, and we observe the
//! GPIO state change via `emu.gpio_read_all()` / `emu.gpio_read(0)`.
//!
//! Hand-assembled Thumb-16 program (loaded at 0x2000_0000):
//!
//! ```text
//! 0x00: 20 01        MOVS r0, #1           ; r0 = 1
//! 0x02: 49 02        LDR  r1, [PC, #8]     ; r1 = SIO_BASE (from literal)
//! 0x04: 62 08        STR  r0, [r1, #32]    ; GPIO_OE   (SIO + 0x20) = 1
//! 0x06: 61 08        STR  r0, [r1, #16]    ; GPIO_OUT  (SIO + 0x10) = 1
//! 0x08: E7 FE        B    .                ; loop forever
//! 0x0A: BF 00        NOP                    ; align literal pool
//! 0x0C: 00 00 00 D0  .word 0xD000_0000      ; SIO_BASE
//! ```

use mdrp2040::{Config, Emulator, EmulatorBuilder};

const SIO_BASE: u32 = 0xD000_0000;
const GPIO_OUT_OFFSET: u32 = 0x010;

fn assemble_gpio_blink() -> Vec<u8> {
    // Little-endian halfwords + one literal word.
    let halfwords: &[u16] = &[
        0x2001, // MOVS r0, #1
        0x4902, // LDR  r1, [PC, #8]
        0x6208, // STR  r0, [r1, #32]  (GPIO_OE  = 1)
        0x6108, // STR  r0, [r1, #16]  (GPIO_OUT = 1)
        0xE7FE, // B    .
        0xBF00, // NOP (pad)
    ];
    let mut out = Vec::with_capacity(halfwords.len() * 2 + 4);
    for hw in halfwords {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    out.extend_from_slice(&SIO_BASE.to_le_bytes());
    out
}

#[test]
fn gpio_blink_program_drives_pin0_high() {
    // Use step_quantum=1 so each step advances by one instruction —
    // the loop count below counts instructions, not quanta.
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    let prog = assemble_gpio_blink();
    let load_addr = 0x2000_0000u32;
    emu.load_image(load_addr, &prog);

    // Boot core 0 manually: SP to top of SRAM, PC to the program. Reset
    // vector fetch from ROM would return zero (no bootrom loaded).
    emu.cores[0].regs.msp = 0x2002_0000;
    emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    emu.cores[0].regs.set_pc(load_addr);
    emu.cores[0].regs.xpsr = 1 << 24; // Thumb bit

    // Run a handful of instructions — the program hits its B . self-loop
    // after 4 instructions (MOVS, LDR, STR, STR, then B).
    for _ in 0..16 {
        emu.step();
    }

    // Confirm GPIO_OUT bit 0 is set via the raw SIO register.
    assert_eq!(emu.bus.sio.gpio_out & 1, 1, "SIO GPIO_OUT bit 0 should be set");
    assert_eq!(emu.bus.sio.gpio_oe & 1, 1, "SIO GPIO_OE bit 0 should be set");
    // And the merged pin state reflects SIO drive + OE.
    assert!(emu.gpio_read(0), "GPIO pin 0 should read high");
    assert_eq!(emu.gpio_read_all() & 1, 1);
}

#[test]
fn gpio_write_api_reflects_in_pin_state() {
    let mut emu = Emulator::new(Config::default());
    // Direct API call — no firmware required.
    emu.gpio_write(7, true);
    assert!(emu.gpio_read(7));
    emu.gpio_write(7, false);
    assert!(!emu.gpio_read(7));
}

#[test]
fn sio_gpio_out_set_via_bus_write() {
    let mut emu = Emulator::new(Config::default());
    // Enable pin 5 via bulk register write through the bus.
    emu.bus.write32(SIO_BASE + 0x020, 1 << 5); // GPIO_OE
    emu.bus.write32(SIO_BASE + GPIO_OUT_OFFSET, 1 << 5); // GPIO_OUT
    // Merge runs at `step` time; poke the merge directly for this test
    // (same as what step does without advancing any instructions).
    let sio_out = emu.bus.sio.gpio_out & emu.bus.sio.gpio_oe;
    assert_eq!(sio_out & (1 << 5), 1 << 5);
}

#[test]
fn core1_stays_halted_until_fifo_wake() {
    // Regression for the placeholder "wake-on-any-non-zero push" behaviour
    // — replaced by the full 6-word SDK handshake (HLD 2026.04.16).
    // A single non-zero push must NOT wake core 1; only a complete valid
    // handshake does.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build();
    assert!(emu.cores[1].is_halted(), "core 1 should be halted at boot");

    // Pre-seed core 0 with a NOP so step() never faults during the probe.
    let nop_addr = 0x2001_0000u32;
    emu.bus.write16(nop_addr, 0xBF00);
    emu.cores[0].regs.set_pc(nop_addr);
    emu.cores[0].regs.msp = 0x2002_0000;
    emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    emu.cores[0].regs.xpsr = 1 << 24;

    // Single non-zero push — at seq=0 the FSM expects a 0; the mismatch
    // resets seq to 0 and echoes a 0, BUT does not produce a launch.
    emu.bus.set_active_core(0);
    emu.bus.write32(SIO_BASE + 0x054, 0xAA);
    emu.step();
    assert!(
        emu.cores[1].is_halted(),
        "single non-zero push must not wake core 1 — placeholder semantics gone"
    );

    // Now the full valid handshake — this MUST wake core 1.
    const VTOR: u32 = 0x2004_0000;
    const SP: u32 = 0x2001_0000;
    const ENTRY: u32 = 0x2000_1001;
    let seq = [0u32, 0, 1, VTOR, SP, ENTRY];
    emu.bus.set_active_core(0);
    for &w in &seq {
        emu.bus.write32(SIO_BASE + 0x054, w);
        // Drain the echo so the FSM can run the next slot with a clean
        // RX queue view (avoids conflating echoes with IPC traffic).
        let _ = emu.bus.read32(SIO_BASE + 0x058);
    }
    // Plant a `B .` self-loop at the entry so that when `emu.step()`
    // wakes core 1 and (with step_quantum=1) runs one core-1
    // instruction in the same quantum, PC stays pinned to entry for
    // the asserts below.
    emu.bus.write16(ENTRY & !1, 0xE7FE);
    emu.step();
    assert!(
        !emu.cores[1].is_halted(),
        "full 6-word handshake must wake core 1"
    );
    assert_eq!(emu.cores[1].regs.pc(), ENTRY & !1);
    assert_eq!(emu.cores[1].regs.msp, SP);
    assert_eq!(emu.bus.ppb[1].vtor, VTOR);
}

// ---------------------------------------------------------------------------
// Stage 1 (PicoGUS Integration HLD): XIP flash
// ---------------------------------------------------------------------------

#[test]
fn emulator_load_flash_roundtrips_through_xip_window() {
    // `Emulator::load_flash` must land bytes at flash offset 0, visible
    // at the canonical XIP base 0x1000_0000 and each of the three
    // aliases (0x11/0x12/0x13).
    let mut emu = Emulator::new(Config::default());
    emu.load_flash(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
    assert_eq!(emu.bus.read32(0x1000_0000), 0x44332211);
    assert_eq!(emu.bus.read32(0x1000_0004), 0x88776655);
    assert_eq!(emu.bus.read32(0x1100_0000), 0x44332211);
    assert_eq!(emu.bus.read32(0x1200_0000), 0x44332211);
    assert_eq!(emu.bus.read32(0x1300_0000), 0x44332211);
}

#[test]
fn emulator_load_flash_clamps_oversize_image() {
    // Stage 1 HLD: "copies bytes into flash starting at offset 0,
    // clamps/errors if too large." RP2040 flash window is 2 MB.
    // Clamp silently at the emulator API boundary.
    let mut emu = Emulator::new(Config::default());
    let big = vec![0xABu8; 3 * 1024 * 1024]; // 3 MB > 2 MB window
    emu.load_flash(&big);
    // First 2 MB must all be 0xAB, aliases mirror.
    assert_eq!(emu.bus.read8(0x1000_0000), 0xAB);
    assert_eq!(emu.bus.read8(0x101F_FFFF), 0xAB);
    assert_eq!(emu.bus.read8(0x1100_0000), 0xAB);
}

#[test]
fn emulator_builder_flash_seeds_xip() {
    // `EmulatorBuilder::flash(Vec<u8>)` lets callers pre-load flash
    // before `build()`, matching the stage-1 CLI pattern
    // `--flash <blinky.bin>`.
    let flash = vec![0xDE, 0xAD, 0xBE, 0xEF];
    let emu = EmulatorBuilder::new(Config::default()).flash(flash).build();
    // Builder seeds before reset; bus peek observes the bytes directly.
    assert_eq!(emu.bus.peek32(0x1000_0000), 0xEFBEADDE);
}

#[test]
fn load_image_to_sram_still_works_after_flash_plumbing() {
    // Regression: the existing `load_image` → SRAM path must keep
    // working untouched by Stage 1 flash changes.
    let mut emu = Emulator::new(Config::default());
    emu.load_image(0x2000_0000, &[0x01, 0x02, 0x03, 0x04]);
    assert_eq!(emu.bus.read32(0x2000_0000), 0x04030201);
}

// ---------------------------------------------------------------------------
// Phase A (PicoGUS Bring-up): ADC CS.READY init-gate
// ---------------------------------------------------------------------------

/// Hand-assembled Thumb-16 mirror of pico-sdk's `adc_init()` wait-for-
/// READY loop. Success sentinel is the `B .` self-loop at 0x2000_0010
/// — asserting `PC == 0x2000_0010` rules out "still spinning at the
/// poll BEQ target 0x2000_000A" and "faulted into 0x0000_0000".
fn assemble_adc_init_poll() -> Vec<u8> {
    let halfwords: &[u16] = &[
        0x4B04, // 0x00: LDR  r3, [PC, #16]   ; r3 = ADC_BASE
        0x2101, // 0x02: MOVS r1, #1          ; r1 = CS_EN
        0x2280, // 0x04: MOVS r2, #0x80
        0x0052, // 0x06: LSLS r2, r2, #1      ; r2 = 0x100 (CS_READY)
        0x6019, // 0x08: STR  r1, [r3, #0]    ; adc_hw->cs = EN
        0x681C, // 0x0A: LDR  r4, [r3, #0]    ; poll: r4 = adc_hw->cs
        0x4214, // 0x0C: TST  r4, r2          ; r4 & CS_READY
        0xD0FC, // 0x0E: BEQ  poll            ; back to 0x2000_000A
        0xE7FE, // 0x10: B    .               ; exit sentinel
        0xBF00, // 0x12: NOP                  ; align literal pool
    ];
    let mut out = Vec::with_capacity(halfwords.len() * 2 + 4);
    for hw in halfwords {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    // Literal pool at 0x2000_0014: ADC_BASE = 0x4004_C000.
    out.extend_from_slice(&0x4004_C000u32.to_le_bytes());
    out
}

#[test]
fn adc_init_sdk_pattern_exits_ready_poll() {
    const RESETS_BASE: u32 = 0x4000_C000;
    const RESETS_CLR_ALIAS: u32 = 0x3000;

    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    let prog = assemble_adc_init_poll();
    let load_addr = 0x2000_0000u32;
    emu.load_image(load_addr, &prog);

    // Default Bus holds ADC in reset; release it so CS writes land.
    emu.bus.write32(RESETS_BASE + RESETS_CLR_ALIAS, 1u32 << 0);

    emu.cores[0].regs.msp = 0x2002_0000;
    emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    emu.cores[0].regs.set_pc(load_addr);
    emu.cores[0].regs.xpsr = 1 << 24; // Thumb bit

    for _ in 0..64 {
        emu.step();
    }

    assert_eq!(
        emu.cores[0].regs.pc(),
        0x2000_0010,
        "adc_init poll must exit to B . sentinel; PC={:#010x}",
        emu.cores[0].regs.pc()
    );
}

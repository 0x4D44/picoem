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
    // Use step_quantum=1 so the wake-on-FIFO observation happens in a
    // single, well-defined step rather than after a 64-instruction drain.
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    assert!(emu.cores[1].is_halted(), "core 1 should be halted at boot");
    // Core 0 pushes through the SIO FIFO → core 1 wakes.
    emu.bus.set_active_core(0);
    emu.bus.write32(SIO_BASE + 0x054, 0xAA);
    // Step once so `maybe_wake_core1` observes the pending event.
    // Pre-seed core 0 with a NOP at its current PC so step doesn't fault.
    let nop_addr = 0x2001_0000u32;
    emu.bus.write16(nop_addr, 0xBF00);
    emu.cores[0].regs.set_pc(nop_addr);
    emu.cores[0].regs.msp = 0x2002_0000;
    emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    emu.cores[0].regs.xpsr = 1 << 24;
    emu.step();
    assert!(!emu.cores[1].is_halted(), "FIFO push should wake core 1");
}

//! PSRAM integration tests — bus and PIO driven.
//!
//! The PSRAM model itself lives in `mdpicoem-devices::psram`. These
//! tests exercise the emulator's `update_gpio()` hook and PIO-driven
//! SPI interleave against the device model wired into `Bus.psram`.

// =============================================================================
// Bus-integration tests — drive PSRAM via the Emulator's GPIO state directly
// (no PIO program). Proves the update_gpio() hook actually calls psram.tick
// and splices MISO back into gpio_in.
// =============================================================================

#[cfg(test)]
mod bus_integration {
    const PIN_MISO: u8 = 0;
    const PIN_CS: u8 = 1;
    const PIN_SCK: u8 = 2;
    const PIN_MOSI: u8 = 3;

    use mdpicoem_devices::Psram;
    use crate::{Config, EmulatorBuilder, Emulator};

    /// Drive the PSRAM's CS/SCK/MOSI pins by poking SIO directly, then
    /// call update_gpio() so the PSRAM observes the change.
    fn drive_pins(emu: &mut Emulator, cs: bool, sck: bool, mosi: bool) {
        // Use GPIO1/2/3 (CS/SCK/MOSI) on SIO with OE asserted.
        let mask = (1u32 << PIN_CS) | (1u32 << PIN_SCK) | (1u32 << PIN_MOSI);
        emu.bus.sio.gpio_oe |= mask;
        let mut out = emu.bus.sio.gpio_out & !mask;
        if cs {
            out |= 1 << PIN_CS;
        }
        if sck {
            out |= 1 << PIN_SCK;
        }
        if mosi {
            out |= 1 << PIN_MOSI;
        }
        emu.bus.sio.gpio_out = out;
        emu.update_gpio();
    }

    /// Clock a single byte out to the PSRAM with CS already low. Returns
    /// the 8 MISO bits sampled on rising edges (MSB first).
    fn clock_byte_via_bus(emu: &mut Emulator, byte: u8) -> u8 {
        let mut out: u8 = 0;
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1;
            drive_pins(emu, false, false, bit != 0);
            drive_pins(emu, false, true, bit != 0);
            // MISO appears as GPIO0 after update_gpio — read it back.
            let miso = ((emu.bus.gpio_in >> PIN_MISO) & 1) as u8;
            out = (out << 1) | miso;
        }
        drive_pins(emu, false, false, false);
        out
    }

    #[test]
    fn bus_hook_write_round_trip() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .psram(Psram::picogus())
            .build();
        // Idle: CS high.
        drive_pins(&mut emu, true, false, false);

        drive_pins(&mut emu, false, false, false); // CS fall
        clock_byte_via_bus(&mut emu, 0x02);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x20);
        clock_byte_via_bus(&mut emu, 0xCA);
        clock_byte_via_bus(&mut emu, 0xFE);
        drive_pins(&mut emu, true, false, false); // CS rise

        assert_eq!(emu.bus.psram.as_ref().unwrap().buffer[0x20], 0xCA);
        assert_eq!(emu.bus.psram.as_ref().unwrap().buffer[0x21], 0xFE);
    }

    #[test]
    fn bus_hook_miso_drives_gpio_in_bit_zero() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .psram(Psram::picogus())
            .build();
        // Seed the buffer so read returns a known non-zero byte.
        emu.bus.psram.as_mut().unwrap().buffer[0x00] = 0xFF; // all 1s — every MISO bit is 1

        drive_pins(&mut emu, true, false, false);
        drive_pins(&mut emu, false, false, false); // CS fall
        clock_byte_via_bus(&mut emu, 0x0B);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        // Dummy byte.
        clock_byte_via_bus(&mut emu, 0x00);
        // Read one data byte — every bit must come back as 1.
        let got = clock_byte_via_bus(&mut emu, 0x00);
        drive_pins(&mut emu, true, false, false);

        assert_eq!(got, 0xFF,
            "PSRAM must drive GPIO0 (MISO) high for each '1' bit in the read byte");
    }

    #[test]
    fn bus_hook_miso_pio_merge_does_not_clobber_psram() {
        // If PIO1 is NOT asserting OE on GPIO0, the PSRAM's MISO drive
        // must land intact in gpio_in. (In real PicoGUS hardware, PIO1
        // configures GPIO0 as an input for its SPI SM; it doesn't drive
        // GPIO0.) This test just confirms the merge order: psram.tick
        // runs after the SIO+PIO merge, so no PIO OE on GPIO0 means MISO
        // wins.
        let mut emu = EmulatorBuilder::new(Config::default())
            .psram(Psram::picogus())
            .build();
        emu.bus.psram.as_mut().unwrap().buffer[0x00] = 0xAA;

        // PIO1 drives a different pin (not GPIO0) — ensure no collision.
        emu.bus.pio[1].pad_oe = 1 << 5;
        emu.bus.pio[1].pad_out = 0;

        drive_pins(&mut emu, true, false, false);
        drive_pins(&mut emu, false, false, false);
        clock_byte_via_bus(&mut emu, 0x0B);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        clock_byte_via_bus(&mut emu, 0x00);
        let got = clock_byte_via_bus(&mut emu, 0x00);
        drive_pins(&mut emu, true, false, false);

        assert_eq!(got, 0xAA);
    }

    #[test]
    fn bus_hook_reset_clears_psram_state() {
        let mut emu = EmulatorBuilder::new(Config::default())
            .psram(Psram::picogus())
            .build();
        // Get the PSRAM into a non-idle state.
        drive_pins(&mut emu, true, false, false);
        drive_pins(&mut emu, false, false, false);
        clock_byte_via_bus(&mut emu, 0x02); // WRITE cmd — partial frame
        // Leave the frame in-progress (no CS rise yet).

        // Seed a reset vector so reset() can run.
        emu.bus.memory.load_rom(&[
            0x00, 0x00, 0x03, 0x20,
            0x01, 0x00, 0x00, 0x20,
        ]);
        emu.reset();

        // After reset, PSRAM state machine must be idle again.
        assert!(emu.bus.psram.as_ref().unwrap().phase_is_idle(),
            "Emulator::reset() must propagate to psram.reset_state()");
    }
}

// =============================================================================
// PIO-driven integration tests — drive SCK from a PIO program and let
// Emulator::step()'s per-cycle PIO/PSRAM interleave deliver every edge.
//
// What these tests prove that the `bus_integration` tests above don't:
//
//   * Multiple SCK edges happen **inside a single `emu.step()` quantum**
//     (PIO toggles SCK every 2 sysclks; step_quantum=4, so one full SCK
//     period per step). The old pre-fix code — `tick_pio(consumed)`
//     followed by a single `update_gpio()` — would only surface the
//     quantum-end pin snapshot to the PSRAM, missing the SCK edges
//     between quantum start and end. The test therefore fails without
//     the interleave fix.
//
//   * No manual `emu.update_gpio()` call anywhere in the test body —
//     the step loop's per-cycle interleave is solely responsible for
//     feeding the PSRAM its pin view.
// =============================================================================

#[cfg(test)]
mod pio_integration {
    const PIN_MISO: u8 = 0;
    const PIN_CS: u8 = 1;
    const PIN_SCK: u8 = 2;
    const PIN_MOSI: u8 = 3;

    use mdpicoem_devices::Psram;
    use crate::bus::{PIO1_BASE, SIO_BASE};
    use crate::{Config, Emulator, EmulatorBuilder};

    /// Step quantum for these tests. Chosen so one full SCK period
    /// (4 sysclks: rise, hold, fall, hold) exactly matches one quantum.
    /// Each `emu.step()` therefore presents the PSRAM with exactly one
    /// SCK rising edge and one falling edge — provided the interleave
    /// fix is in place.
    const STEP_QUANTUM: u32 = 4;

    /// Install a minimal SCK-generator program into PIO1 SM0 on pin
    /// [`PIN_SCK`], running at system clock (clkdiv = 1). The program:
    ///
    ///   addr 0: SET PINS, 1 [delay=1]   ; SCK rises, 2 cycles total
    ///   addr 1: SET PINS, 0 [delay=1]   ; SCK falls, 2 cycles total
    ///   (wrap addr 1 -> 0)
    ///
    /// Total period: 4 sysclks. One rising edge per 4-cycle quantum.
    fn install_sck_toggler(emu: &mut Emulator) {
        // Instruction encoding (no side-set): [SET 111][delay 00001][dst 000][data 00001]
        const SET_PINS_1_D1: u16 = 0xE101; // SET PINS, 1 with delay=1
        const SET_PINS_0_D1: u16 = 0xE100; // SET PINS, 0 with delay=1

        // INSTR_MEM0 / INSTR_MEM1.
        emu.bus.write32(PIO1_BASE + 0x048, SET_PINS_1_D1 as u32);
        emu.bus.write32(PIO1_BASE + 0x04C, SET_PINS_0_D1 as u32);

        // SM0_PINCTRL: SET count=1, SET base=PIN_SCK (=2).
        let pinctrl = (1u32 << 26) | ((PIN_SCK as u32) << 5);
        emu.bus.write32(PIO1_BASE + 0x0DC, pinctrl);

        // SM0_EXECCTRL: wrap_top=1, wrap_bottom=0.
        let execctrl = (1u32 << 12) | (0u32 << 7);
        emu.bus.write32(PIO1_BASE + 0x0CC, execctrl);

        // Force-execute SET PINDIRS, 1 to mark SCK as an output.
        emu.bus.write32(PIO1_BASE + 0x0D8, 0xE081);

        // NB: not enabled yet — caller enables after CS/MOSI are set up.
    }

    fn enable_sm0(emu: &mut Emulator) {
        // CTRL.SM_ENABLE bit 0 = SM0.
        emu.bus.write32(PIO1_BASE + 0x000, 0x1);
    }

    /// Park core 0 on a long chain of NOPs at 0x2000_0000 so each
    /// `emu.step()` quantum advances exactly `STEP_QUANTUM` sysclks on
    /// the PIO side (each M0+ NOP is a 1-cycle instruction — branches
    /// are 3, so no JMPs here).
    fn park_core0_on_nops(emu: &mut Emulator) {
        let prog = 0x2000_0000u32;
        for i in 0..256u32 {
            emu.bus.write16(prog + i * 2, 0xBF00); // NOP
        }
        emu.cores[0].regs.set_pc(prog);
        emu.cores[0].regs.msp = 0x2003_0000;
        emu.cores[0].regs.r[13] = emu.cores[0].regs.msp;
    }

    /// Configure CS, MOSI, (and later we'll sample MISO at GPIO0) via
    /// SIO. Both pins start as outputs; CS starts high (idle), MOSI
    /// starts low.
    fn configure_sio_bits(emu: &mut Emulator) {
        let cs_mask = 1u32 << PIN_CS;
        let mosi_mask = 1u32 << PIN_MOSI;
        // GPIO_OE_SET — enable CS + MOSI outputs (leave MISO as input).
        emu.bus.write32(SIO_BASE + 0x024, cs_mask | mosi_mask);
        // GPIO_OUT_SET — CS high initially.
        emu.bus.write32(SIO_BASE + 0x014, cs_mask);
    }

    fn sio_set_mosi(emu: &mut Emulator, bit: bool) {
        let mask = 1u32 << PIN_MOSI;
        if bit {
            emu.bus.write32(SIO_BASE + 0x014, mask); // GPIO_OUT_SET
        } else {
            emu.bus.write32(SIO_BASE + 0x018, mask); // GPIO_OUT_CLR
        }
    }

    fn sio_set_cs(emu: &mut Emulator, high: bool) {
        let mask = 1u32 << PIN_CS;
        if high {
            emu.bus.write32(SIO_BASE + 0x014, mask); // GPIO_OUT_SET
        } else {
            emu.bus.write32(SIO_BASE + 0x018, mask); // GPIO_OUT_CLR
        }
    }

    /// Clock one MSB-first byte out: set MOSI per bit and call
    /// `emu.step()` once — exactly one SCK rising edge per step when
    /// `STEP_QUANTUM == 4` matches the PIO program's period.
    ///
    /// MISO is sampled **before** each step, matching real SPI timing:
    /// the PSRAM updates MISO on SCK falling edges, and the master
    /// samples it on the next rising edge (i.e., at the start of the
    /// next quantum). Sampling after the step would be one bit ahead.
    ///
    /// Returns the MISO byte assembled from bit 0 of `gpio_in`.
    fn clock_out_byte(emu: &mut Emulator, byte: u8) -> u8 {
        let mut miso_byte: u8 = 0;
        for i in 0..8 {
            let bit = ((byte >> (7 - i)) & 1) != 0;
            sio_set_mosi(emu, bit);
            // Sample MISO *before* stepping — this is the bit the PSRAM
            // loaded on the previous quantum's falling edge, which the
            // real master samples on the current rising edge.
            let miso = ((emu.bus.gpio_in >> PIN_MISO) & 1) as u8;
            miso_byte = (miso_byte << 1) | miso;
            emu.step();
        }
        miso_byte
    }

    fn fresh_emu() -> Emulator {
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(STEP_QUANTUM)
            .psram(Psram::picogus())
            .build();
        configure_sio_bits(&mut emu);
        install_sck_toggler(&mut emu);
        park_core0_on_nops(&mut emu);
        // Let initial pin state propagate through one step before SM runs,
        // so the PSRAM's prev_cs latches to CS=high.
        emu.step();
        emu
    }

    #[test]
    fn pio_driven_write_then_read_round_trip() {
        let mut emu = fresh_emu();
        enable_sm0(&mut emu);

        // Write frame: drop CS, clock 0x02, 3 addr bytes (0x00,0x01,0x00
        // for address 0x100), 4 data bytes, raise CS.
        //
        // CS-fall and the first SCK rising edge land in the same quantum —
        // `psram::tick` handles that correctly (begin_frame runs before
        // the clock-edge work on the same tick), so the cmd byte's MSB
        // is the first bit captured.
        sio_set_cs(&mut emu, false);

        clock_out_byte(&mut emu, 0x02); // WRITE cmd
        clock_out_byte(&mut emu, 0x00); // addr [23:16]
        clock_out_byte(&mut emu, 0x01); // addr [15:8]
        clock_out_byte(&mut emu, 0x00); // addr [7:0]
        clock_out_byte(&mut emu, 0xDE);
        clock_out_byte(&mut emu, 0xAD);
        clock_out_byte(&mut emu, 0xBE);
        clock_out_byte(&mut emu, 0xEF);

        sio_set_cs(&mut emu, true);
        emu.step(); // propagate CS-rise to PSRAM.

        assert_eq!(
            &emu.bus.psram.as_ref().unwrap().buffer[0x100..0x104],
            &[0xDE, 0xAD, 0xBE, 0xEF],
            "PIO-driven SCK must deliver every rising edge to the PSRAM; \
             missing edges would leave buffer[0x100..0x104] at zero."
        );
    }

    #[test]
    fn pio_driven_fast_read_returns_written_bytes() {
        // Same plumbing, but exercises the fast-read path: 0x0B cmd + 3
        // addr bytes + 1 dummy byte + read MISO for N bytes.
        let mut emu = fresh_emu();
        enable_sm0(&mut emu);

        // Seed the buffer with a known pattern at address 0x200.
        emu.bus.psram.as_mut().unwrap().buffer[0x200] = 0x11;
        emu.bus.psram.as_mut().unwrap().buffer[0x201] = 0x22;
        emu.bus.psram.as_mut().unwrap().buffer[0x202] = 0x33;
        emu.bus.psram.as_mut().unwrap().buffer[0x203] = 0x44;

        sio_set_cs(&mut emu, false);

        clock_out_byte(&mut emu, 0x0B); // Fast Read cmd
        clock_out_byte(&mut emu, 0x00); // addr [23:16]
        clock_out_byte(&mut emu, 0x02); // addr [15:8]
        clock_out_byte(&mut emu, 0x00); // addr [7:0]
        clock_out_byte(&mut emu, 0x00); // 8 dummy cycles (one byte)
        let b0 = clock_out_byte(&mut emu, 0x00);
        let b1 = clock_out_byte(&mut emu, 0x00);
        let b2 = clock_out_byte(&mut emu, 0x00);
        let b3 = clock_out_byte(&mut emu, 0x00);

        sio_set_cs(&mut emu, true);
        emu.step();

        assert_eq!([b0, b1, b2, b3], [0x11, 0x22, 0x33, 0x44],
            "PIO-driven fast-read must return the seeded buffer bytes — \
             a single-edge-per-quantum interleave fix is required.");
    }
}

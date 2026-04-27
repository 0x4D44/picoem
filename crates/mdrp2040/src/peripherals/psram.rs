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

    use crate::{Config, Emulator, EmulatorBuilder};
    use mdpicoem_devices::Psram;

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
            .build()
            .expect("Serial build is infallible");
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
            .build()
            .expect("Serial build is infallible");
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

        assert_eq!(
            got, 0xFF,
            "PSRAM must drive GPIO0 (MISO) high for each '1' bit in the read byte"
        );
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
            .build()
            .expect("Serial build is infallible");
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
            .build()
            .expect("Serial build is infallible");
        // Get the PSRAM into a non-idle state.
        drive_pins(&mut emu, true, false, false);
        drive_pins(&mut emu, false, false, false);
        clock_byte_via_bus(&mut emu, 0x02); // WRITE cmd — partial frame
        // Leave the frame in-progress (no CS rise yet).

        // Seed a reset vector so reset() can run.
        emu.bus
            .memory
            .load_rom(&[0x00, 0x00, 0x03, 0x20, 0x01, 0x00, 0x00, 0x20]);
        emu.reset();

        // After reset, PSRAM state machine must be idle again.
        assert!(
            emu.bus.psram.as_ref().unwrap().phase_is_idle(),
            "Emulator::reset() must propagate to psram.reset_state()"
        );
    }
}

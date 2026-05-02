//! PicoGUS v4.0.0 pin-mapping constants.
//!
//! Single source of truth for every RP2040 GPIO the PicoGUS v4.0.0
//! firmware pins to an external signal. The harness replayer
//! (`picogus_diff_rp2040`) and the I2S capture module
//! (`i2s_capture`) both import from here so the mapping cannot drift
//! between them.
//!
//! All values are sourced from the firmware tag `v4.0.0` of
//! <https://github.com/polpo/picogus>:
//!
//! | Signal        | GPIO | Source                                   |
//! | ------------- | ---- | ---------------------------------------- |
//! | `PSRAM MISO`  | 0    | `sw/CMakeLists.txt:261` (`PSRAM_PIN_MISO`) |
//! | `PSRAM CS`    | 1    | `sw/CMakeLists.txt:258` (`PSRAM_PIN_CS`)   |
//! | `PSRAM SCK`   | 2    | `sw/CMakeLists.txt:259` (`PSRAM_PIN_SCK`)  |
//! | `PSRAM MOSI`  | 3    | `sw/CMakeLists.txt:260` (`PSRAM_PIN_MOSI`) |
//! | `ISA IOW#`    | 4    | `sw/isa/isa_io.pio:21` (`IOW_PIN`)         |
//! | `ISA IOR#`    | 5    | `sw/isa/isa_io.pio:22` (`IOR_PIN`)         |
//! | `ISA AD0..9`  | 6..15| `sw/isa/isa_io.pio:19` (`AD0_PIN`, 10 pins) |
//! | `I2S DOUT`    | 16   | `sw/CMakeLists.txt:85` (`PICO_AUDIO_I2S_DATA_PIN`) |
//! | `I2S BCLK`    | 17   | `sw/CMakeLists.txt:86` (`PICO_AUDIO_I2S_CLOCK_PIN_BASE`) |
//! | `I2S LRCLK`   | 18   | `clock_pin_base + 1` (sideset bit 1 of `audio_i2s.pio`) |
//! | `ISA DACK`    | 19   | `sw/isa/isa_io.pio:26` (`DACK_PIN`)         |
//! | `ISA IRQ`     | 21   | `sw/isa/isa_io.pio:20` (`IRQ_PIN`)          |
//! | `ISA IOCHRDY` | 26   | `sw/isa/isa_io.pio:23` (`IOCHRDY_PIN`)      |
//! | `ISA ADS`     | 27   | `sw/isa/isa_io.pio:24` (`ADS_PIN`)          |
//! | `UART TX`     | 28   | `sw/isa/isa_io.pio:25` (`UART_TX_PIN`)      |
//!
//! There is no overlap between the ISA bank (GPIO 4..28 with gaps)
//! and the I2S bank (GPIO 16..18). The PSRAM bank (GPIO 0..3) is
//! likewise disjoint.

// ---------------------------------------------------------------------------
// PSRAM (external SPI PSRAM — Stage 2 model observes these pins)
// ---------------------------------------------------------------------------

/// PSRAM MISO — firmware reads the chip's data-out on this input.
pub const PSRAM_MISO: u8 = 0;
/// PSRAM chip-select (active low).
pub const PSRAM_CS: u8 = 1;
/// PSRAM serial clock.
pub const PSRAM_SCK: u8 = 2;
/// PSRAM MOSI — firmware drives command/data bits here.
pub const PSRAM_MOSI: u8 = 3;

// ---------------------------------------------------------------------------
// ISA bus (Stage 4 harness drives or observes these)
// ---------------------------------------------------------------------------

/// I/O write strobe, active low.
pub const ISA_IOW: u8 = 4;
/// I/O read strobe, active low.
pub const ISA_IOR: u8 = 5;
/// Lowest pin of the 10-bit multiplexed address/data bus (AD0..AD9
/// occupy `ISA_AD0 .. ISA_AD0 + ISA_AD_COUNT`).
pub const ISA_AD0: u8 = 6;
/// Number of consecutive GPIOs that carry the AD bus.
pub const ISA_AD_COUNT: u8 = 10;
/// DMA acknowledge (firmware-driven input to the ISA bus).
pub const ISA_DACK: u8 = 19;
/// IRQ line driven by firmware (output).
pub const ISA_IRQ: u8 = 21;
/// IOCHRDY wait-state handshake (firmware sideset output).
pub const ISA_IOCHRDY: u8 = 26;
/// Address/data mux select (firmware sideset output).
pub const ISA_ADS: u8 = 27;
/// UART TX pin (diagnostic stdio).
pub const UART_TX: u8 = 28;

// ---------------------------------------------------------------------------
// I2S (Stage 5 capture observes these)
// ---------------------------------------------------------------------------

/// I2S serial data output — DOUT from the RP2040 to the PCM5102 DAC.
pub const I2S_DOUT: u8 = 16;
/// I2S bit clock — firmware-driven, runs at 32 * sample_rate (stereo
/// 16-bit). Sideset bit 0 of the PIO I2S program.
pub const I2S_BCLK: u8 = 17;
/// I2S word-select / LRCLK. Low = left channel, high = right channel
/// (standard Philips I2S). Sideset bit 1 of the PIO I2S program.
pub const I2S_LRCLK: u8 = 18;

// ---------------------------------------------------------------------------
// Convenience masks
// ---------------------------------------------------------------------------

/// Bitmask covering IOW#, IOR#, and the 10-bit AD bus.
pub const ISA_EXTERNAL_PIN_MASK: u32 =
    (1u32 << ISA_IOW) | (1u32 << ISA_IOR) | (((1u32 << ISA_AD_COUNT) - 1) << ISA_AD0);

/// Bitmask covering the three I2S pins (DOUT + BCLK + LRCLK).
pub const I2S_PIN_MASK: u32 = (1u32 << I2S_DOUT) | (1u32 << I2S_BCLK) | (1u32 << I2S_LRCLK);

#[cfg(test)]
mod tests {
    use super::*;

    /// Pin assignments must not overlap across functional groups. Guards
    /// against a copy/paste slip that silently aliases an ISA pin to an
    /// I2S pin (or similar).
    #[test]
    fn pin_assignments_are_disjoint() {
        let psram =
            (1u32 << PSRAM_MISO) | (1u32 << PSRAM_CS) | (1u32 << PSRAM_SCK) | (1u32 << PSRAM_MOSI);
        let isa = ISA_EXTERNAL_PIN_MASK
            | (1u32 << ISA_DACK)
            | (1u32 << ISA_IRQ)
            | (1u32 << ISA_IOCHRDY)
            | (1u32 << ISA_ADS)
            | (1u32 << UART_TX);
        let i2s = I2S_PIN_MASK;
        assert_eq!(psram & isa, 0, "PSRAM overlaps ISA");
        assert_eq!(psram & i2s, 0, "PSRAM overlaps I2S");
        assert_eq!(isa & i2s, 0, "ISA overlaps I2S");
    }

    /// Sanity-check the expected numeric values so a future refactor
    /// that bumps a constant fails loudly.
    #[test]
    fn i2s_pin_numbers_match_firmware() {
        assert_eq!(I2S_DOUT, 16);
        assert_eq!(I2S_BCLK, 17);
        assert_eq!(I2S_LRCLK, 18);
    }

    #[test]
    fn isa_pin_numbers_match_firmware() {
        assert_eq!(ISA_IOW, 4);
        assert_eq!(ISA_IOR, 5);
        assert_eq!(ISA_AD0, 6);
        assert_eq!(ISA_AD_COUNT, 10);
        assert_eq!(ISA_IOCHRDY, 26);
        assert_eq!(ISA_ADS, 27);
        assert_eq!(UART_TX, 28);
    }

    #[test]
    fn psram_pin_numbers_match_firmware() {
        assert_eq!(PSRAM_MISO, 0);
        assert_eq!(PSRAM_CS, 1);
        assert_eq!(PSRAM_SCK, 2);
        assert_eq!(PSRAM_MOSI, 3);
    }
}

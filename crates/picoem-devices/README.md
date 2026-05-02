# picoem-devices

[![Crates.io](https://img.shields.io/crates/v/picoem-devices.svg)](https://crates.io/crates/picoem-devices)
[![Docs.rs](https://docs.rs/picoem-devices/badge.svg)](https://docs.rs/picoem-devices)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/picoem)

Off-chip device models — PSRAM, LCD, I2S — for the
[picoem](https://github.com/0x4D44/picoem) RP2350 / RP2040 emulator
workspace.

This is a support crate for the Pico emulators. Most users want
[`rp2350-emu`](https://crates.io/crates/rp2350-emu) or
[`rp2040-emu`](https://crates.io/crates/rp2040-emu) directly; those crates
embed the device models from here when they are needed (e.g. `rp2040-emu`
uses the HyperRAM-style PSRAM model for `test_psram` compatibility).

## What's in here

- **HyperRAM-style external PSRAM model** — drives the SPI-side of the
  RP2040 PicoGUS PSRAM dispatch path.
- **LCD device model** — frame-buffered display backend used by the
  RP2350 TUI app's LCD demo.
- **I2S capture model** — sampling sink that decodes BCLK/LRCLK/DOUT
  from the RP2040 emulator's PIO pad output and produces stereo PCM.

See the [workspace README](https://github.com/0x4D44/picoem) for the
broader context.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

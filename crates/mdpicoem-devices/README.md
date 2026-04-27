# mdpicoem-devices

[![Crates.io](https://img.shields.io/crates/v/mdpicoem-devices.svg)](https://crates.io/crates/mdpicoem-devices)
[![Docs.rs](https://docs.rs/mdpicoem-devices/badge.svg)](https://docs.rs/mdpicoem-devices)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/mdpicoem)

Off-chip device models — PSRAM, LCD, I2S — for the
[mdpicoem](https://github.com/0x4D44/mdpicoem) RP2350 / RP2040 emulator
workspace.

This is a support crate for the Pico emulators. Most users want
[`mdrp2350`](https://crates.io/crates/mdrp2350) or
[`mdrp2040`](https://crates.io/crates/mdrp2040) directly; those crates
embed the device models from here when they are needed (e.g. `mdrp2040`
uses the HyperRAM-style PSRAM model for `test_psram` compatibility).

## What's in here

- **HyperRAM-style external PSRAM model** — drives the SPI-side of the
  RP2040 PicoGUS PSRAM dispatch path.
- **LCD device model** — frame-buffered display backend used by the
  RP2350 TUI app's LCD demo.
- **I2S capture model** — sampling sink that decodes BCLK/LRCLK/DOUT
  from the RP2040 emulator's PIO pad output and produces stereo PCM.

See the [workspace README](https://github.com/0x4D44/mdpicoem) for the
broader context.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

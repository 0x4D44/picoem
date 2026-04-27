# mdpicoem-common

[![Crates.io](https://img.shields.io/crates/v/mdpicoem-common.svg)](https://crates.io/crates/mdpicoem-common)
[![Docs.rs](https://docs.rs/mdpicoem-common/badge.svg)](https://docs.rs/mdpicoem-common)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/mdpicoem)

Shared primitive types for the [mdpicoem](https://github.com/0x4D44/mdpicoem)
RP2350 / RP2354 / RP2040 emulator workspace.

This crate is a low-level building block for the chip emulators in the
mdpicoem workspace; it is unlikely to be useful as a stand-alone library.
**If you want to embed a Pico emulator in your project, depend on
[`mdrp2350`](https://crates.io/crates/mdrp2350) or
[`mdrp2040`](https://crates.io/crates/mdrp2040) instead** — they re-export
the parts of `mdpicoem-common` that consumers need.

## What's in here

- `Memory` — the untimed RAM/ROM/flash backing store used by both chip
  emulators. Configurable per-chip via `with_sizes(rom, sram)`.
- `ClockTree` — the chip-clock math used by the RP2350 and RP2040 clock
  registers. Recomputed on PLL/divider register writes.
- `Pacer` — atomic cycle/nanosecond accounting for wall-clock pacing.
  `x86_64`-only.
- PIO building blocks — `PioBlock`, `StateMachine`, `Divider`, `Fifo`.
- Threading primitives — `threaded::SpinBarrier`, `threaded::SpscQueue`.
  `x86_64` Windows only.

## Features

- `test-hooks` — exposes test-only PIO state mutators (`PioBlock::push_rx`,
  `pop_tx`) for cross-crate unit tests. **Do not enable in production
  builds.**
- `pio-pad-diag` — optional PIO PSRAM-SPI pad-edge diagnostic counters.
  Off by default; enable when running PicoGUS-style PSRAM-SPI diff work.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

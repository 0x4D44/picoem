# mdrp2040

[![Crates.io](https://img.shields.io/crates/v/mdrp2040.svg)](https://crates.io/crates/mdrp2040)
[![Docs.rs](https://docs.rs/mdrp2040/badge.svg)](https://docs.rs/mdrp2040)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/mdpicoem)

A cycle-accurate emulator library for the **Raspberry Pi RP2040**
(dual Arm Cortex-M0+ @ 133 MHz, 264 KB SRAM, PIO).

`mdrp2040` is the RP2040-side of the [mdpicoem](https://github.com/0x4D44/mdpicoem)
workspace. It runs ARMv6-M firmware and is differentially validated
against both QEMU's `cortex-m0` and real RP2040 silicon via SWD.

## Quick start

Add to `Cargo.toml`:

```toml
[dependencies]
mdrp2040 = "0.1"
```

Minimal usage:

```rust,no_run
use mdrp2040::{EmulatorBuilder, ExecutionModel};

let bootrom = std::fs::read("bootrom-rp2040-b2.bin")?;
let firmware = std::fs::read("my_firmware.bin")?;

let mut emu = EmulatorBuilder::new()
    .execution(ExecutionModel::Serial)
    .build()?;

emu.load_bootrom(&bootrom)?;
emu.load_image(&firmware)?;

// Step the dual-core machine for 1M master-clock cycles.
emu.run(1_000_000);

# Ok::<(), Box<dyn std::error::Error>>(())
```

The Raspberry Pi RP2040 B2 bootrom is published by Raspberry Pi at
<https://github.com/raspberrypi/pico-bootrom-rp2040> under BSD-3-Clause.

## What's modelled

- **Dual Cortex-M0+ cores** (ARMv6-M). All Thumb-16, the supported
  Thumb-32 subset (`BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`), banked
  MSP/PSP, exception entry/return.
- **AHB-Lite bus fabric** with cycle accounting and a deprecated-in-place
  bank-contention model on the Serial execution path.
- **264 KB SRAM** across 4 striped + 2 scratch banks; 16 KB bootrom.
  RP2040 has no onboard flash — firmware loads into SRAM via
  `load_image`.
- **Single-cycle IO** (SIO) — GPIO, CPUID, FIFO, 32 spinlocks, hardware
  divider, interpolators.
- **Clocks** — ROSC / XOSC / PLL_SYS / PLL_USB / dividers, all
  reprogrammable at runtime.
- **Two PIO blocks** with state machines, FIFOs, dividers.
- **PPB** with sticky `bus_fault` flag escalating to HardFault.

## Execution models

- **`ExecutionModel::Serial`** (default) — single host thread runs both
  cores interleaved per `step_quantum`. The oracle-validated reference
  path. Recommended for most uses.
- **`ExecutionModel::Threaded`** — three-thread worker runtime,
  barrier-synchronised at the quantum boundary. Faster for some
  workloads. Currently supported on **x86_64 Windows and x86_64 Linux**;
  other platforms get `ConfigError::ThreadingUnavailable`.

## Features

- `threading` — feature-gates the threaded runtime. Opt-in for V1 so
  `cargo add mdrp2040` works cross-platform out of the box; on x86_64
  Windows or x86_64 Linux, enable with
  `cargo add mdrp2040 --features threading` to use `ThreadedEmulator`.
- `testing` — opt-in panic-injection APIs. **Do not enable in
  production builds.**
- `test-hooks` — exposes test-only PIO hooks for cross-crate testing.

## Workspace context

This crate is part of the `mdpicoem` workspace; the project also publishes:

- [`mdrp2350`](https://crates.io/crates/mdrp2350) — RP2350 / RP2354 (Cortex-M33) emulator.
- [`mdpicoem-common`](https://crates.io/crates/mdpicoem-common) — shared primitives.
- [`mdpicoem-devices`](https://crates.io/crates/mdpicoem-devices) — off-chip device models (PSRAM, LCD, I2S).

The full workspace, including TUI applications, the test harness, the
QEMU + silicon differential oracles, and design documents, lives at
<https://github.com/0x4D44/mdpicoem>.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

*Raspberry Pi*, *RP2040*, and *Pico* are trademarks of Raspberry Pi Ltd.
*Arm* and *Cortex-M0+* are trademarks or registered trademarks of Arm
Limited. This project is independent and not affiliated with or endorsed
by either company.

# rp2350-emu

> **Status:** Personal research project — no maintenance commitments.
> See the [project repository](https://github.com/0x4D44/picoem).

[![Crates.io](https://img.shields.io/crates/v/rp2350-emu.svg)](https://crates.io/crates/rp2350-emu)
[![Docs.rs](https://docs.rs/rp2350-emu/badge.svg)](https://docs.rs/rp2350-emu)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/picoem)

A cycle-accurate emulator library for the **Raspberry Pi RP2350 / RP2354**
(dual Arm Cortex-M33 @ 150 MHz, 520 KB SRAM, FPU, coprocessors, PIO).

`rp2350-emu` is the RP2350-side of the [picoem](https://github.com/0x4D44/picoem)
workspace. It boots the real Raspberry Pi bootrom, runs ARMv8-M Mainline
firmware, and is differentially validated against both QEMU's Cortex-M33 and
real RP2354 silicon via SWD.

## Quick start

Add to `Cargo.toml`:

```toml
[dependencies]
rp2350-emu = "0.1"
```

Minimal usage:

```rust,no_run
use rp2350_emu::{EmulatorBuilder, ExecutionModel};

let bootrom = std::fs::read("bootrom-combined.bin")?;
let firmware = std::fs::read("my_firmware.bin")?;

let mut emu = EmulatorBuilder::new()
    .execution(ExecutionModel::Serial)
    .build()?;

emu.load_bootrom(&bootrom)?;
emu.load_flash(&firmware)?;

// Step the dual-core machine for 1M master-clock cycles.
emu.run(1_000_000);

# Ok::<(), Box<dyn std::error::Error>>(())
```

The Raspberry Pi RP2350 bootrom is published by Raspberry Pi at
<https://github.com/raspberrypi/pico-bootrom-rp2350> under BSD-3-Clause.

## What's modelled

- **Dual Cortex-M33 cores** (ARMv8-M Mainline). All Thumb-16 and the
  modelled Thumb-32 subset, IT blocks, exception entry/return, lazy FP
  context save, NVIC, MPU seams.
- **VFPv5 single-precision FPU** with lazy FP context save (FPCCR/FPCAR).
- **Coprocessors** — GPIO/CP0 → SIO; DCP on CP4/CP5; RCP on CP7.
- **AHB5 bus fabric** with cycle accounting and APB-bridge latency.
- **520 KB SRAM** across 10 banks; 32 KB bootrom; XIP flash.
- **Single-cycle IO** (SIO) — GPIO, spinlocks, FIFOs, interpolators.
- **16-channel DMA** with `CTRL_TRIG` aliases, `RING_*`, `CHAIN_TO`,
  `CH_ABORT`, fixed-priority arbitration, and the full DREQ matrix.
- **Two PIO blocks** (`PioBlock`) with state machines, FIFOs, dividers.
- **Clocks** — ROSC / XOSC / PLL_SYS / PLL_USB / dividers, all
  reprogrammable at runtime.
- **Peripherals** — TIMER0/1, TICKS, POWMAN, UART0/1, SPI0/1, I2C0/1,
  PWM, ADC, WATCHDOG, OTP, TRNG, SHA256, PSM, IO_BANK0, PADS_BANK0,
  CoreSight trace.

## Execution models

`rp2350-emu` ships two interchangeable execution backends, selectable per
emulator instance:

- **`ExecutionModel::Serial`** (default) — single host thread runs both
  cores interleaved per `step_quantum`. The oracle-validated reference
  path (QEMU diff, silicon diff). Recommended for most uses.
- **`ExecutionModel::Threaded`** — six-thread worker runtime,
  barrier-synchronised at the quantum boundary. Faster for compute-heavy
  workloads. Currently supported on **x86_64 Windows and x86_64 Linux**;
  on other platforms `Builder::build()` returns
  `ConfigError::ThreadingUnavailable`.

## Features

- `threading` — feature-gates the threaded runtime. Opt-in for V1 so
  `cargo add rp2350-emu` works cross-platform out of the box; on x86_64
  Windows or x86_64 Linux, enable with
  `cargo add rp2350-emu --features threading` to use `ThreadedEmulator`.
- `testing` — opt-in panic-injection APIs for the panic-containment test
  suite. **Do not enable in production builds.**
- `test-hooks` — exposes test-only PIO hooks for cross-crate testing.

## Workspace context

This crate is part of the `picoem` workspace; the project also publishes:

- [`rp2040-emu`](https://crates.io/crates/rp2040-emu) — RP2040 (Cortex-M0+) emulator.
- [`picoem-common`](https://crates.io/crates/picoem-common) — shared primitives.
- [`picoem-devices`](https://crates.io/crates/picoem-devices) — off-chip device models.

The full workspace, including TUI applications, the test harness, the
QEMU + silicon differential oracles, and design documents, lives at
<https://github.com/0x4D44/picoem>.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

*Raspberry Pi*, *RP2350*, and *RP2354* are trademarks of Raspberry Pi Ltd.
*Arm* and *Cortex-M33* are trademarks or registered trademarks of Arm
Limited. This project is independent and not affiliated with or endorsed
by either company.

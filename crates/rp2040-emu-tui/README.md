# rp2040-emu-tui

[![Crates.io](https://img.shields.io/crates/v/rp2040-emu-tui.svg)](https://crates.io/crates/rp2040-emu-tui)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/picoem)

Interactive terminal UI for the
[`rp2040-emu`](https://crates.io/crates/rp2040-emu) Raspberry Pi
RP2040 emulator (dual Arm Cortex-M0+ + PIO).

Built with [ratatui](https://ratatui.rs/) and
[crossterm](https://github.com/crossterm-rs/crossterm). Loads the real
Raspberry Pi RP2040 B2 bootrom + an ARMv6-M firmware image into SRAM
(RP2040 has no onboard flash) and lets you step the dual-core machine,
inspect register / memory / trace state, and watch GPIO from a plain
terminal.

## Install

```bash
cargo install rp2040-emu-tui
```

Then point it at a bootrom + firmware image. See the
[picoem repo](https://github.com/0x4D44/picoem) for ready-to-run
bootrom and demo firmware (`roms/rp2040/blinky.bin` is the smallest
useful target).

## What it shows

- Per-core Cortex-M0+ register file (R0..R15, xPSR, banked MSP/PSP,
  CONTROL, PRIMASK).
- ISA panel with M0+-specific cycle counts (MULS = 1, LDR = 2,
  LDM N = 1+N, B = 1–3, BL = 4).
- Memory dump windows (ROM, SRAM banks, XIP region) and a streaming
  instruction trace.
- PIO state (state machines, FIFO depth, pin output).

## Workspace context

Part of the [picoem](https://github.com/0x4D44/picoem) workspace, which
also publishes:

- [`rp2040-emu`](https://crates.io/crates/rp2040-emu) — the underlying RP2040 emulator library.
- [`rp2350-emu`](https://crates.io/crates/rp2350-emu) — RP2350 / RP2354 (Cortex-M33) emulator.
- [`rp2350-emu-tui`](https://crates.io/crates/rp2350-emu-tui) — RP2350 sibling of this TUI.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

*Raspberry Pi*, *RP2040*, and *Pico* are trademarks of Raspberry Pi Ltd.
*Arm* and *Cortex-M0+* are trademarks or registered trademarks of Arm
Limited. This project is independent and not affiliated with or endorsed
by either company.

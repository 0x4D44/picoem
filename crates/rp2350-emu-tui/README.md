# rp2350-emu-tui

[![Crates.io](https://img.shields.io/crates/v/rp2350-emu-tui.svg)](https://crates.io/crates/rp2350-emu-tui)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/picoem)

Interactive terminal UI for the
[`rp2350-emu`](https://crates.io/crates/rp2350-emu) Raspberry Pi
RP2350 / RP2354 emulator (dual Arm Cortex-M33 + PIO + FPU).

Built with [ratatui](https://ratatui.rs/) and
[crossterm](https://github.com/crossterm-rs/crossterm). Boots a real
Raspberry Pi bootrom + ARMv8-M firmware image and lets you step the
machine, inspect register / memory / trace state, and watch GPIO from a
plain terminal.

## Install

```bash
cargo install rp2350-emu-tui
```

Then point it at a bootrom + firmware image. See the
[picoem repo](https://github.com/0x4D44/picoem) for ready-to-run
bootrom and demo firmware (blinky, LCD demo, benchmark stubs).

## What it shows

- Per-core Cortex-M33 register file (R0..R15, xPSR, banked SP, CONTROL).
- VFPv5 single-precision FPU register file (S0..S31), with FPCCR/FPCAR
  diagnostics for lazy FP context save.
- DCP / RCP coprocessor state (CP4/CP5/CP7).
- ARMv8-M exception entry diagnostics — vector table, EXC_RETURN,
  Secure / Non-Secure context, NVIC priority.
- Memory dump windows (ROM, SRAM, XIP flash) and a streaming
  instruction trace.

## RISC-V (Hazard3)

The underlying `rp2350-emu` library can construct a RISC-V Hazard3
emulator (per Phase 5 of the Hazard3 HLD). **This TUI is Arm-specific** —
its panels render Cortex-M33 state that has no Hazard3 analogue. If you
need an interactive RISC-V driver today, use the library API directly.

## Workspace context

Part of the [picoem](https://github.com/0x4D44/picoem) workspace, which
also publishes:

- [`rp2350-emu`](https://crates.io/crates/rp2350-emu) — the underlying RP2350 emulator library.
- [`rp2040-emu`](https://crates.io/crates/rp2040-emu) — RP2040 (Cortex-M0+) emulator.
- [`rp2040-emu-tui`](https://crates.io/crates/rp2040-emu-tui) — RP2040 sibling of this TUI.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

*Raspberry Pi*, *RP2350*, and *RP2354* are trademarks of Raspberry Pi
Ltd. *Arm* and *Cortex-M33* are trademarks or registered trademarks of
Arm Limited. This project is independent and not affiliated with or
endorsed by either company.

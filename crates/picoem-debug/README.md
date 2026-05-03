# picoem-debug

[![Crates.io](https://img.shields.io/crates/v/picoem-debug.svg)](https://crates.io/crates/picoem-debug)
[![Docs.rs](https://docs.rs/picoem-debug/badge.svg)](https://docs.rs/picoem-debug)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](https://github.com/0x4D44/picoem)

Placeholder crate for the future GDB RSP server, instruction trace, and
DWT-driven tooling in the [picoem](https://github.com/0x4D44/picoem)
RP2350 / RP2354 / RP2040 emulator workspace.

**This crate currently has no public API.** It is reserved on crates.io
for the project's GDB / debug tooling, which is slated for a later phase
of the workspace HLD roadmap. Until then it builds successfully but
exposes nothing useful.

If you want to embed a Pico emulator in your project today, depend on
[`rp2350-emu`](https://crates.io/crates/rp2350-emu) or
[`rp2040-emu`](https://crates.io/crates/rp2040-emu) directly.

## License

Dual-licensed under either:

- Apache License, Version 2.0
- MIT license

at your option.

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
for published crates.

Per-crate version numbers reflect the workspace's pre-public iteration
history rather than restarting at `0.1.0`. Each crate's own `[package]
version` was bumped in line with the user CLAUDE.md per-file
semantic-versioning convention as the workspace evolved; the inaugural
public release simply ships those current versions.

## [Unreleased]

## [2026-05-06]

Patch release for `rp2350-emu`. DMA pacing within the step quantum: the DMA
controller now ticks per master-clock cycle inside `step()` (instead of
once per quantum at the boundary), and multiple shared-DREQ channels can
fire on a single tick. Test-only push-event hook added on `Dma`/`Bus`
behind `cfg(feature = "testing")`. Bus-level fast path + `route_irqs`
hoist preserve no-DMA-armed performance.

Reference HLD: `wrk_docs/2026.05.06 - HLD - DMA Pacing Within Step Quantum.md`.

### Crates published to crates.io

| Crate | Version | Change |
|---|---|---|
| `rp2350-emu` | `0.2.2` | Silicon-correct DMA pacing within step quantum; multi-channel-per-tick arbitration for shared DREQs; `cfg(feature = "testing")` push-event hook on `Dma`/`Bus`; bus fast path + `route_irqs` hoist (no perf regression when no DMA channels are armed). |

## [2026-05-04]

Second publication round. Picks up the wide-GPIO bus work (RP2354 high-half
GPIOs 32..47) and the PIO `GPIOBASE` high-bank sampling support.

### Crates published to crates.io

| Crate | Version | Change |
|---|---|---|
| `picoem-common` | `0.2.0` | New public PIO API: `PioBlock::gpio_base`, `local_to_physical_pins`, `step_with_pins`, `step_n_with_pins`. Existing `step` / `step_n` retained as wrappers. |
| `picoem-devices` | `0.1.2` | README polish; no API change. |
| `rp2350-emu` | `0.2.0` | New public API on `Bus` and `Emulator` for GPIOs 32..47 (`gpio_in_hi`, `gpio_external_in_hi`, `gpio_external_mask_hi`); `Emulator::gpio_read` extended to pin range 0..47. PIO `GPIOBASE` register honoured for SM input/output windows. Picks up `picoem-common` 0.2. |
| `rp2040-emu` | `0.1.3` | Internal-only clippy 1.95 lint sweep; picks up `picoem-common` 0.2. No public API change. |
| `picoem-debug` | `0.1.1` | Metadata refresh; placeholder crate. |
| `rp2350-emu-tui` | `0.1.2` | README polish; tracks `rp2350-emu` 0.2. |
| `rp2040-emu-tui` | `0.1.2` | README polish; tracks `rp2040-emu` 0.1. |

## [Initial public release] — 2026-05-03

First publication of the picoem workspace as open source under the
dual MIT OR Apache-2.0 license.

### Crates published to crates.io

| Crate | Version | Notes |
|---|---|---|
| `picoem-common` | `0.1.2` | Shared primitives: `Memory`, `ClockTree`, `Pacer`, PIO building blocks, threading helpers. |
| `picoem-devices` | `0.1.1` | Off-chip device models: PSRAM, LCD, I2S capture. |
| `rp2350-emu` | `0.1.3` | RP2350 / RP2354 emulator library (dual Cortex-M33 + PIO + FPU). |
| `rp2040-emu` | `0.1.2` | RP2040 emulator library (dual Cortex-M0+ + PIO). |
| `picoem-debug` | `0.1.0` | Placeholder for the future GDB RSP server / debug tooling. |
| `rp2350-emu-tui` | `0.1.1` | Interactive ratatui/crossterm TUI for `rp2350-emu`. |
| `rp2040-emu-tui` | `0.1.1` | Interactive ratatui/crossterm TUI for `rp2040-emu`. |

### Not published

- `picoem-harness` — internal differential-test binaries; depends on a
  patched probe-rs and uses path-only deps with crate-private features.
  `publish = false` in its manifest. Namespace squat handled separately
  per OSS-release HLD §13.7.
- `epio-sys` — `-sys` belongs to the upstream `piersfinlayson/epio`
  project; not squatted.

### What's in scope for V1

See the workspace [README](README.md) and per-crate READMEs for the
modelled feature set. Differential validation against QEMU (Cortex-M33,
Cortex-M0, RV32IMC-Zba-Zbb-Zbs) and against real RP2354 / RP2040 silicon
via probe-rs. Phases 1–7 of the workspace restructure complete.

### Acknowledgements

- Raspberry Pi Ltd for the RP2350 and RP2040 bootroms (BSD-3-Clause).
- The `probe-rs` project — vendored fork carrying a small DPv1
  cache-upgrade workaround for upstream issue #3872.
- The Rust embedded ecosystem — `rp235x-hal`, `rp2040-hal`, and the
  Cortex-M tooling crates that informed our naming and API choices.

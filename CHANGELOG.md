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

## [2026-05-07] — third release

Patch release for `rp2350-emu`. Three narrow-write fixes plus an
observation-based smoke verdict in the OneROM full-system harness.

RESETS narrow-write widening: `Bus::write8` / `Bus::write16` for
`0x4002_0000` previously dropped byte/halfword writes (`=> {}` with the
comment "RESETS: only word-aligned writes meaningful"). Real silicon's
AHB widens narrow writes to 32-bit before the peripheral sees them.
RESETS has a single writable register (`RESET` at offset `0x000`) and
no side-effect-on-read, so the standard subword-alias RMW dispatch via
`resets_read` / `resets_write` composes cleanly. `STRB` / `STRH` to
any RESETS alias (plain / XOR / SET / CLR) now lands.

PIO TXF-only narrow-write widening: this morning's earlier 0.2.4
attempt routed every PIO narrow write through the standard
subword-alias RMW used by UART/SPI/I2C. That pattern doesn't transfer
to PIO — FDEBUG / IRQ are W1C with **live read state** (RMW splice
zeros bits outside the targeted byte), `SMn_INSTR` write32 calls
`force_execute` (RMW would re-execute prior opcodes ORed with the
narrow value), CTRL byte 1 carries self-clearing `SM_RESTART` actions,
`SHIFTCTRL` byte 3 carries `FJOIN_TX/RX` FIFO-flush bits, and RXF
**pops the FIFO on read**. After review the design changed to
TXF-only widen (offsets `0x010..=0x01C`) with explicit drop for every
other PIO register — matches `rp2040-emu`'s design philosophy and
sidesteps every side-effect hazard above. Non-TXF narrow writes are
dropped (matching pre-0.2.4 behaviour for those registers); only TXF
gets widened. The widen now uses zero-extension `(val as u32) <<
(byte_idx * 8)` directly instead of the brief subword-alias RMW that
0.2.4 carried on disk — RP2350 is AHB5 (byte-strobed, no
replication), distinct from rp2040-emu's AHB-Lite byte-replication.
Both produce identical observables for OneROM's `OUT PINS, 8`
(bottom byte only).

OneROM full-system smoke harness verdict tightened: criterion 5 was
rewritten from a wrong-formula computation
(`((SHADOW_BASE + addr_word) & 0xFF) ^ 0x55` — assumed pin-to-SHADOW
lift is identity, which it isn't) to an observation-based criterion:
collect the CH1 push-byte set during each dwell window, require at
least one obs cycle in that dwell with `oe == 0xFF` and `data_byte ∈
push_bytes`. Empty dwells are inconclusive (skipped). Post-loop guard
tightened from `pin_matches == 0` to `pin_matches < 2` to mirror
criterion 4's distinct-src-addrs cardinality requirement. Smoke
verdict feeds the harness exit code; sync-without-serve now exits
FAILURE.

Regression coverage in `crates/rp2350-emu/src/pio_tests.rs`:
`pio_narrow_writes_widen_to_32_bit` (extended to lanes 2/3 byte +
halfword lane 1 across PIO0/1/2),
`pio_narrow_writes_to_rxf_dont_pop_fifo`,
`pio_narrow_writes_to_fdebug_dont_corrupt`,
`pio_narrow_writes_to_ctrl_dont_trigger_sm_restart`,
`pio_narrow_writes_to_sm_instr_dont_force_execute`,
plus `tests::resets_narrow_writes_widen_to_32_bit`.

### Crates published to crates.io

| Crate | Version | Change |
|---|---|---|
| `rp2350-emu` | `0.2.5` | RESETS narrow-write widening (subword-alias RMW); PIO TXF-only narrow-write widening with zero-extension for AHB5 byte-strobe semantics (non-TXF PIO narrow writes still dropped — silicon-correct given W1C/force-execute/SM_RESTART/FJOIN/RXF-pop side effects). OneROM full-system smoke harness verdict criterion 5 rewritten observation-based; `pin_matches < 2` now fails. |

## [2026-05-07]

Patch release for `rp2350-emu`. DMA-to-DMA correctness fix: `Bus::tick_dma`
swaps the live `Dma` out of `Bus` for the duration of `dma.tick(bus)` to
avoid a cross-borrow, but pre-fix any DMA-issued bus write whose
destination fell in the DMA register aperture (`0x5000_0000..0x5000_3FFF`)
dispatched through the bus to the empty `Dma::default()` stand-in and was
silently dropped. Chains that update one channel's registers from another
(e.g. `CH0.WRITE_ADDR = CH1.READ_ADDR` — the OneROM SDRR firmware idiom)
appeared to fire and decremented `TRANS_COUNT` but never updated the
target register. Real silicon's AHB carries DMA self-accesses to the DMA
peripheral the same as any other master would; this release routes
DMA-aperture transfers through the live `Dma` directly inside
`Dma::issue_transfer` to match.

Regression covered by the new `dma::tests::dma_to_dma_write_during_tick_lands_on_live_dma`
unit test, plus a tightened `onerom_full_system_rp2350` smoke harness that
sweeps the external address pins through several distinct values and
requires `last_src_addr` to take more than one distinct value across CH1
push edges (the pre-fix harness drove a single all-zero address, which
made a stuck `CH1.READ_ADDR` indistinguishable from a working pipe).

### Crates published to crates.io

| Crate | Version | Change |
|---|---|---|
| `rp2350-emu` | `0.2.3` | Fix DMA-to-DMA write drop in `Bus::tick_dma` borrow trap. DMA-aperture self-accesses now route through the live `Dma` instead of the `mem::take` stand-in. Adds a regression unit test; tightens the OneROM full-system smoke harness with an address-pin sweep. |

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

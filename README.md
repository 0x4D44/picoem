# mdrp2354

A cycle-accurate emulator for the Raspberry Pi **RP2354** microcontroller (dual Arm Cortex-M33 @ 150 MHz + 520 KB SRAM + PIO), written in Rust.

The goal is a small, clean, verifiable emulator core that can boot the real Pi 2350/2354 bootrom, run Thumb-2 firmware with accurate cycle timing, and serve as a reusable library crate for downstream projects.

```
mdrp2354 (this repo)          — RP2354 emulator crate + TUI + test harness
  └─► onerom-emu              — OneROM firmware running on mdrp2354
        └─► mddosem           — DOS emulator, uses OneROM as BIOS
```

## Status

Actively developed. Arm-mode-only; Hazard3 RISC-V cores are out of scope.

| Subsystem | State |
|---|---|
| Cortex-M33 integer ISA (Thumb-16 + Thumb-32) | Working; differential-tested against QEMU |
| FPU (VFPv5 single-precision subset) | Working; lazy context save |
| Coprocessors (GPIO/CP0, DCP, RCP) | Working |
| Dual-core + SIO (spinlocks, FIFOs, interpolators) | Working |
| Bus fabric (AHB5 decode, APB bridge, bank-conflict stalls) | Working; some timing edge cases open (see `tech_debt.md`) |
| Clock tree (ROSC / XOSC / PLL / dividers) | Working |
| Exceptions / NVIC / fault delivery | Working |
| Memory (32 KB ROM, 520 KB SRAM, XIP flash) | Working |
| Pacer (wall-clock real-time pacing) | Working (`x86_64` only) |
| UART / SPI / I2C / DMA / timers | Stubs |
| PIO blocks | Stubs |
| GDB RSP debug server | Stubs |
| TrustZone (SAU / ACCESSCTRL) | Design seams only — v1 treats everything as Secure |

## Quick Start

```bash
# Build everything (release profile is strongly recommended — debug builds are slow)
cargo build --release

# Launch the interactive TUI with the blinky demo firmware
cargo run -p mdrp2354-app --release

# Run a different bundled firmware preset
cargo run -p mdrp2354-app --release -- lcd        # LCD demo
cargo run -p mdrp2354-app --release -- benchmark  # throughput benchmark
cargo run -p mdrp2354-app --release -- blinky     # (default)

# Load your own firmware
cargo run -p mdrp2354-app --release -- path/to/firmware.bin
```

The TUI has panels for CPU status, GPIO state, an LCD device emulator, an ISA trace view, and a live benchmark panel.

Bundled ROMs under `roms/` (`blinky.bin`, `benchmark.bin`, `lcd_demo.bin`, `dualcore.bin`) are generated from the `gen_*.py` scripts in the same directory. The real Pi bootrom is checked in as `roms/bootrom-combined.bin`.

## Workspace Layout

Six crates under `crates/`:

- **`mdrp2354`** — the emulator core library (CPUs, bus, memory, clocks, SIO, pacer). All the hot path lives here.
- **`mdrp2354-app`** — the interactive TUI (ratatui + crossterm) with panels and a device frontend (LCD, benchmark).
- **`mdrp2354-test-harness`** — all differential and hardware-in-the-loop test binaries.
- **`mdrp2354-periph`** — peripheral implementations (UART/SPI/I2C/DMA) injected via the `Peripheral` trait. Stubbed.
- **`mdrp2354-pio`** — PIO block emulation. Stubbed.
- **`mdrp2354-debug`** — GDB RSP server and trace tooling. Stubbed.

The top-level `src/main.rs` is a one-line sanity binary that prints config; the real UI is `mdrp2354-app`.

## Testing

The emulator is validated by three independent oracles, each catching different bug classes:

### 1. Unit tests

```bash
cargo test                      # all crates
cargo test -p mdrp2354          # core only
cargo test <name_substring>     # filtered
```

Instruction semantics, decode edge cases, exception mechanics, and clock-tree config live in `crates/mdrp2354/src/tests.rs`.

### 2. QEMU differential harness

Spawns a QEMU Cortex-M33, connects over GDB on `localhost:3333`, runs the same instruction in both QEMU and the emulator, then diffs R0–R15 and xPSR (masking architecturally unpredictable flag fields).

```bash
# Targeted edge-case suite (fast)
cargo run -p mdrp2354-test-harness --release --bin qemu_diff

# Random fuzz — N cases per instruction class
cargo run -p mdrp2354-test-harness --release --bin qemu_diff -- --fuzz 100000

# Reproduce a specific failure
cargo run -p mdrp2354-test-harness --release --bin qemu_diff -- --fuzz 100000 --seed <S>
```

Requires `qemu-system-arm` on `PATH`.

### 3. Hardware-in-the-loop (real RP2354 silicon)

Drive a real RP2354 board over SWD via a Pi Pico debug probe, single-step it, and diff against the emulator. Catches behaviours QEMU doesn't model correctly — e.g. SRAM bank contention, pipeline effects.

```bash
# Same test suite as qemu_diff but against silicon
cargo run -p mdrp2354-test-harness --release --bin probe_diff

# Register / DWT cycle-counter sanity checks
cargo run -p mdrp2354-test-harness --release --bin probe_verify

# SRAM bank-conflict timing characterisation
cargo run -p mdrp2354-test-harness --release --bin bank_conflict_test
```

Requires a Pi Pico configured as a `probe-rs`-compatible debug probe wired to an RP2354 target.

### 4. Paced benchmark

```bash
cargo run -p mdrp2354-test-harness --release --bin paced_bench
```

Measures real-time throughput with wall-clock pacing, useful for regression-checking performance work.

### Coverage

```bash
cargo llvm-cov
```

## Design Documents

Phase HLDs live under `wrk_docs/`. Filenames follow `YYYY.MM.DD - HLD - <topic> V<N>.md`. Start with `2026.04.12 - RP2350 Emulator HLD.md` for the master design, then the phase docs (bus fabric, interrupts, dual-core, PIO, coprocessors/FPU) for subsystem detail. Newer dated versions supersede earlier drafts of the same phase.

Per-session journals (investigations, performance work, review cycles) live under `wrk_journals/`. Known cycle-timing gaps tracked against real silicon are in `tech_debt.md`.

## Requirements

- Rust (edition 2024, stable)
- `qemu-system-arm` for `qemu_diff` (any reasonably recent QEMU release)
- A Pi Pico debug probe + RP2354 target board for the `probe_*` harnesses (optional)

The `pacer` module uses `x86_64`-only atomics; builds on other hosts will work as a library but won't get wall-clock pacing.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

This repository also redistributes the Raspberry Pi RP2350 bootrom
(`roms/bootrom-combined.bin`) under BSD-3-Clause — see [NOTICE](NOTICE) for
attribution.

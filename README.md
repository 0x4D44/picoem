# picoem

Cycle-accurate emulators for the Raspberry Pi **RP2354 / RP2350** (dual Arm Cortex-M33 @ 150 MHz, 520 KB SRAM, FPU, coprocessors, PIO) and **RP2040** (dual Arm Cortex-M0+ @ 133 MHz, 264 KB SRAM, PIO), written in Rust.

The goal is a small, clean, verifiable emulator core that can boot the real Pi bootroms, run ARMv8-M (M33) or ARMv6-M (M0+) firmware with accurate cycle timing, and serve as a reusable library crate for downstream projects.

```
picoem (this repo)           — RP2350/RP2354 + RP2040 emulators, TUIs, test harness
  ├─► rp2350-emu                 — RP2350 / RP2354 emulator library (Cortex-M33)
  ├─► rp2040-emu                 — RP2040 emulator library (Cortex-M0+)
  └─► onerom-emu               — OneROM firmware running on rp2350-emu
        └─► mddosem            — DOS emulator, uses OneROM as BIOS
```

## Status

A personal research project, published in case it's useful to others.
I use these crates in my own work and will likely keep pushing fixes
as I hit them, but I'm not committing to issue triage, PR review SLAs,
or ongoing feature development. No promises — if you need certainty
around a bugfix or feature, fork freely under MIT/Apache-2.0.

Feature coverage as of the published versions — Arm-mode only; Hazard3
RISC-V cores on RP2350 are out of scope:

| Subsystem | RP2350 / RP2354 | RP2040 |
|---|---|---|
| CPU core ISA | Cortex-M33 (ARMv8-M Mainline); differential-tested vs QEMU | Cortex-M0+ (ARMv6-M); differential-tested vs QEMU `cortex-m0` |
| FPU (VFPv5 single-precision) | Working; lazy context save | N/A |
| Coprocessors (GPIO/CP0, DCP, RCP) | Working | N/A |
| Dual-core + SIO (spinlocks, FIFOs, interpolators) | Working | Working (+ hardware divider) |
| Bus fabric | Working; some timing edge cases open | Working; simplifications open (see `tech_debt.md`) |
| Clock tree (ROSC / XOSC / PLL / dividers) | Working | Working |
| Exceptions / NVIC / fault delivery | Working | Working |
| Memory | 32 KB ROM, 520 KB SRAM, XIP flash | 16 KB ROM, 264 KB SRAM (no onboard flash) |
| PIO blocks | Working | Working |
| Pacer (wall-clock real-time pacing) | Working | Working |
| UART / SPI / I2C / DMA / timers | Stubs | Stubs |
| GDB RSP debug server | Stub | Stub |
| TrustZone (SAU / ACCESSCTRL) | Design seams only — v1 treats everything as Secure | N/A |

Open cycle-timing gaps and post-Phase-7 residuals are tracked in `tech_debt.md`.

## Quick Start

Clone with submodules — the workspace member `epio-sys` references vendored
upstream sources via git submodules. A normal clone works for everything else,
but `cargo build -p epio-sys` requires submodules to be initialised:

```bash
git clone --recursive https://github.com/0x4D44/picoem.git
# or, after a non-recursive clone:
git submodule update --init
```

`epio-sys` is excluded from the workspace's `default-members` and additionally
requires `clang` to be on `PATH`, so a plain `cargo build --release` at the
workspace root works on hosts without `clang` or initialised submodules — opt
in explicitly with `cargo build -p epio-sys` once those prerequisites are in
place.

```bash
# Build everything (release profile is strongly recommended — debug is slow)
cargo build --release

# RP2350 / RP2354 interactive TUI (dual Cortex-M33)
cargo run -p rp2350-emu-tui --release              # blinky (default)
cargo run -p rp2350-emu-tui --release -- lcd       # LCD demo
cargo run -p rp2350-emu-tui --release -- benchmark # throughput benchmark
cargo run -p rp2350-emu-tui --release -- blinky    # (explicit)

# RP2040 interactive TUI (dual Cortex-M0+)
cargo run -p rp2040-emu-tui --release              # blinky (default)

# Load your own firmware
cargo run -p rp2350-emu-tui --release -- path/to/firmware.bin
cargo run -p rp2040-emu-tui --release -- path/to/firmware.bin
```

The RP2350 TUI has panels for CPU status, GPIO state, an LCD device emulator, an ISA trace view, and a live benchmark panel. The RP2040 TUI has the same shape minus the FPU / DCP / RCP / NS panels, and its ISA panel carries M0+-specific cycle numbers.

Bundled ROMs under `roms/rp2350/` (`blinky.bin`, `benchmark.bin`, `lcd_demo.bin`, `dualcore.bin`) and `roms/rp2040/` (`blinky.bin`) are generated from the `gen_*.py` scripts in the same directories. The real Pi bootroms are checked in as `roms/rp2350/bootrom-combined.bin` and `roms/rp2040/bootrom.bin`.

## Workspace Layout

Seven crates under `crates/`:

- **`picoem-common`** — shared primitives: `Memory`, `ClockTree`, `Pacer`, PIO primitive types (`PioBlock` / `StateMachine`), divider/FIFO, `Peripheral` trait. Both chip crates depend on this.
- **`rp2350-emu`** — the RP2350 / RP2354 emulator core library (CPUs, bus, memory, clocks, SIO, PIO, FPU, coprocessors, pacer).
- **`rp2350-emu-tui`** — interactive TUI (ratatui + crossterm) for `rp2350-emu`, with panels and a device frontend (LCD, benchmark).
- **`rp2040-emu`** — the RP2040 emulator core library (dual Cortex-M0+, bus, memory, clocks, SIO, PIO).
- **`rp2040-emu-tui`** — interactive TUI for `rp2040-emu`.
- **`picoem-harness`** — all differential and hardware-in-the-loop test binaries. Binaries are chip-suffixed (`qemu_diff_m33` / `qemu_diff_m0plus`, `probe_diff_rp2350` / `probe_diff_rp2040`, etc.).
- **`picoem-debug`** — GDB RSP server and trace tooling. Stubbed.

The real UIs are `rp2350-emu-tui` and `rp2040-emu-tui`; run them with `cargo run -p rp2350-emu-tui` or `cargo run -p rp2040-emu-tui`. The workspace has no top-level binary.

## Testing

The emulators are validated by independent oracles, each catching different bug classes.

### 1. Unit tests

```bash
cargo test                      # all crates
cargo test -p rp2350-emu          # RP2350 / RP2354 core only
cargo test -p rp2040-emu          # RP2040 core only
cargo test <name_substring>     # filtered
```

Instruction semantics, decode edge cases, exception mechanics, PIO, and clock-tree config live in each core crate's `tests.rs` / `pio_tests.rs` / `tests/firmware.rs`.

### 2. QEMU differential harness (per chip)

Each oracle spawns a QEMU reference CPU, connects over GDB, runs the same instruction in both QEMU and the emulator, then diffs R0–R15 + xPSR (masking architecturally unpredictable flag fields).

```bash
# RP2350 / Cortex-M33 oracle (GDB port 3333)
cargo run -p picoem-harness --release --bin qemu_diff_m33                        # edge-case suite
cargo run -p picoem-harness --release --bin qemu_diff_m33 -- --fuzz 100000       # random fuzz
cargo run -p picoem-harness --release --bin qemu_diff_m33 -- --fuzz 100000 --seed <S>

# RP2040 / Cortex-M0+ oracle (GDB port 3334, uses QEMU `cortex-m0` — see `tech_debt.md`)
cargo run -p picoem-harness --release --bin qemu_diff_m0plus
cargo run -p picoem-harness --release --bin qemu_diff_m0plus -- --fuzz 100000
cargo run -p picoem-harness --release --bin qemu_diff_m0plus -- --fuzz 100000 --seed <S>
```

Requires `qemu-system-arm` on `PATH`.

On Windows, a running `.exe` is locked against overwrite, so a long fuzz session will block concurrent `cargo build --release` (or any build that relinks the harness). Copy the binary out before a long run — e.g. `cp target/release/qemu_diff_m33.exe /tmp/fuzzer.exe && /tmp/fuzzer.exe --fuzz 100000`. The overnight drivers under `fuzz-runs/` handle this automatically. See `CLAUDE.md` for the full note.

### 3. Hardware-in-the-loop (real silicon)

Drive a real RP2354 board over SWD via a Pi Pico debug probe, single-step it, and diff against the emulator. Catches behaviours QEMU doesn't model correctly — e.g. pipeline effects, peripheral timing. `bank_conflict_test_rp2350` characterises SRAM bank-contention timing on silicon for reference; the emulator itself does **not** model bank contention on RP2350 (see `CLAUDE.md` / `wrk_journals/2026.04.15 - JRN - Contention Modelling Declined.md` for rationale).

```bash
# Same instruction-level test suite as qemu_diff_m33 but against silicon
cargo run -p picoem-harness --release --bin probe_diff_rp2350

# Register / DWT cycle-counter sanity checks
cargo run -p picoem-harness --release --bin probe_verify_rp2350

# SRAM bank-conflict timing characterisation
cargo run -p picoem-harness --release --bin bank_conflict_test_rp2350
```

Requires a Pi Pico configured as a `probe-rs`-compatible debug probe wired to an RP2354 target (for the `*_rp2350` binaries) or a Pico V1 / RP2040 target (for `probe_diff_rp2040`). On a host with both probes attached, disambiguate with `--probe <VID:PID:SERIAL>` — `probe-rs list` shows the available serials.

### 4. Paced benchmark and full integration

```bash
cargo run -p picoem-harness --release --bin paced_bench_rp2350
cargo run -p picoem-harness --release --bin full_test_rp2350
```

Measures real-time throughput with wall-clock pacing and runs a larger integration smoke. Useful for regression-checking performance work.

### Coverage

```bash
cargo llvm-cov
```

## Design Documents

Phase HLDs live under `wrk_docs/`. Filenames follow `YYYY.MM.DD - HLD - <topic> V<N>.md`. Start with `2026.04.12 - RP2350 Emulator HLD.md` for the original design (with an errata note at the top tracking the post-restructure crate renames), then the phase docs (bus fabric, interrupts, dual-core, PIO, coprocessors/FPU) for subsystem detail. Newer dated versions supersede earlier drafts of the same phase. The workspace restructure is documented in `2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

Per-session journals (investigations, performance work, review cycles) live under `wrk_journals/`. Known cycle-timing gaps against real silicon and post-Phase-7 residuals are in `tech_debt.md`.

## Requirements

- Rust (edition 2024, stable)
- `qemu-system-arm` for `qemu_diff_m33` / `qemu_diff_m0plus` (any reasonably recent release; the M0+ oracle uses `cortex-m0` because QEMU 10.2 doesn't ship a `cortex-m0plus` model — see `tech_debt.md`)
- A Pi Pico debug probe + RP2354 target board for the `probe_*_rp2350` harnesses (optional)

The `pacer` module uses an `rdtsc` backend on `x86_64` and an `Instant` backend elsewhere, so serial emulator builds and TUIs support macOS, Linux, and Windows. The threaded worker runtime remains limited to `x86_64` Windows/Linux.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <https://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

This repository redistributes third-party content under their respective
licenses — Raspberry Pi RP2350 and RP2040 bootroms (BSD-3-Clause), the
PicoGUS firmware (GPL-2.0-or-later), and a vendored fork of probe-rs
(MIT OR Apache-2.0). See [NOTICE](NOTICE) for the full list and attribution.

## Trademarks

*Raspberry Pi*, *RP2350*, *RP2354*, *RP2040*, and *Pico* are trademarks
of Raspberry Pi Ltd. *Arm*, *Cortex-M0+*, *Cortex-M33*, *Armv6-M*,
*Armv8-M*, and *NEON* are trademarks or registered trademarks of Arm
Limited (or its subsidiaries) in the US and/or elsewhere. *Sound
Blaster* is a trademark of Creative Technology Ltd. *AdLib*, *Gravis
Ultrasound*, and *MT-32* are trademarks of their respective owners.
*Monkey Island* is a trademark of Lucasfilm Entertainment Company Ltd.
LLC.

This project is an independent emulator and is not affiliated with,
endorsed by, or sponsored by any of the above trademark holders. All
trademarks are used for identification purposes only.

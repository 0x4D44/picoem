# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## mdpicoem — RP2350 / RP2040 Emulator Workspace

Cycle-accurate emulator for the Raspberry Pi RP2354 / RP2350 family (dual Cortex-M33 + PIO), with planned RP2040 support. Rust workspace with four crates under `crates/` after the Phase 1 restructure:

- `mdrp2350` — the RP2350/RP2354 emulator library (core, bus, memory, SIO, PIO, clocks, pacer).
- `mdrp2350app` — the TUI demo app driving `mdrp2350`.
- `mdpicoem-harness` — differential test binaries (QEMU diff, probe-rs diff, softfloat diff, paced benchmark, full-test runner).
- `mdpicoem-debug` — GDB RSP scaffolding (stub).

Later phases add `mdpicoem-common`, `mdrp2040`, and `mdrp2040app` — see `wrk_docs/2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

## Build & Test

```bash
# Build everything
cargo build --release

# Run all unit tests
cargo test

# Run a single test (substring match)
cargo test <test_name_substr>

# Run tests in one crate
cargo test -p mdrp2350

# Code coverage
cargo llvm-cov
```

## Differential Fuzz Testing (QEMU harness)

The QEMU harness spawns a QEMU Cortex-M33, connects over GDB on `localhost:3333`, runs the same instruction in both QEMU and our emulator, and diffs R0–R15 + xPSR (with masking for architecturally unpredictable flag fields).

```bash
# Run N random fuzz tests per instruction class
cargo run -p mdpicoem-harness --release --bin qemu_diff -- --fuzz <N>

# Reproducible run with a specific seed
cargo run -p mdpicoem-harness --release --bin qemu_diff -- --fuzz <N> --seed <S>

# Targeted edge-case tests only (default, no args)
cargo run -p mdpicoem-harness --release --bin qemu_diff
```

### Typical fuzz sessions

| Goal | Command |
|---|---|
| Quick smoke test | `--fuzz 1000` |
| Standard session | `--fuzz 100000` |
| Extended soak | `--fuzz 1000000` (or more) |

When asked to "fuzz test" or "do some fuzzing", default to `--fuzz 100000` unless a different count or duration is specified. For time-based requests ("fuzz for 2 hours"), estimate iterations from prior run throughput and adjust.

### Handling failures

When the harness reports a mismatch:
1. Note the seed and instruction class from the failure output.
2. Reproduce with `--seed <S>` to get a deterministic repro.
3. Investigate the specific instruction's decode/execute path in our emulator.
4. Fix and re-run the same seed to confirm.

## Workspace Layout

- **`crates/mdrp2350`** — the emulator core (CPUs, bus, memory, clocks, SIO, pacer). All the hot path lives here.
- **`crates/mdrp2350app`** — interactive TUI (ratatui/crossterm) for register/memory/trace inspection and firmware loading.
- **`crates/mdpicoem-debug`** — GDB RSP server + trace scaffolding (currently a stub).
- **`crates/mdpicoem-harness`** — all test binaries (see "Testing Topology" below).

The top-level `src/main.rs` is a sanity-check stub that prints emulator config. **The real entry point is `mdrp2350app`, not this binary.**

## Core Emulator Architecture (`crates/mdrp2350/src/`)

- **`lib.rs`** — `Emulator` aggregates two `CortexM33` cores, `Bus`, `Clock`. Public API: `step`/`run`/`load_bootrom`/`load_flash`. Builder pattern for construction.
- **`core/`** — CPU implementation:
  - `mod.rs` — `CortexM33` struct, fetch-decode-execute loop, multi-cycle stall tracking, IT-block state, exception entry/return.
  - `decode.rs` — Thumb-16 / Thumb-32 decoder → operation enum.
  - `execute.rs` + `execute_thumb32.rs` — instruction semantics (hot path; `execute_thumb32.rs` is large, search by instruction mnemonic).
  - `execute_fpu.rs` — VFPv5 single-precision subset with lazy FP context save (FPCCR/FPCAR).
  - `exceptions.rs` — vector table, stacking, `EXC_RETURN`, NVIC integration, fault handlers.
  - `coprocessor.rs` — CP dispatch (GPIO/CP0 → SIO; DCP on CP4/5; RCP on CP7).
- **`bus/`** — AHB5 address decode, cycle accounting, APB bridge latency, peripheral register backing store, and contention tracking (see below).
- **`bus/clocks.rs`** — ROSC / XOSC / PLL_SYS / PLL_USB / divider model. Recomputes the cached `ClockTree` on register writes.
- **`memory/`** — untimed ROM (32 KB) / SRAM (520 KB across 10 banks) / XIP flash backing storage.
- **`sio/`** — single-cycle IO: GPIO, spinlocks, FIFOs, interpolators, coprocessor interface.
- **`pacer.rs`** — atomic cycle/nanosecond accounting for wall-clock pacing. **`x86_64`-only** (gated at module level).
- **`tests.rs`** — massive in-crate unit test file for instruction semantics, decode edge cases, exception mechanics, clock tree config.

## Testing Topology

Four independent oracles, each catching different bugs:

1. **Unit tests** (`crates/mdrp2350/src/tests.rs`) — instruction semantics, decode, exceptions, clock tree.
2. **`qemu_diff`** — QEMU Cortex-M33 reference vs. emulator, via GDB single-step. Catches architectural mistakes (flag computation, wide-instruction decode, PC-relative addressing).
3. **`probe_diff`** + **`probe_verify`** + **`bank_conflict_test`** — same idea but against **real RP2354 silicon** via SWD (probe-rs 0.31). Catches behaviours QEMU gets wrong or doesn't model (e.g. SRAM bank contention timing). Requires a Pico debug probe attached to RP2354 hardware.
4. **`paced_bench`** and **`full_test`** — real-time paced throughput / integration smoke test.

## High-Level Design Documents

Under `wrk_docs/`. HLDs are **phase-based and dated** (`YYYY.MM.DD - HLD - <topic> V<N>.md`). The original master HLD is `2026.04.12 - RP2350 Emulator HLD.md`, but subsequent phase HLDs (Phase 2 bus, Phase 3 interrupts, Phase 4 flash boot, Phase 5 dual-core SIO, Phase 6 PIO, Phase 7 coprocessors/FPU) supersede relevant sections. The workspace restructure itself lives in `2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

When working on a specific subsystem, **read the latest HLD for that phase** — not the master HLD. Later-dated versions (e.g. V5 over V2) supersede earlier drafts of the same phase.

Per-session notes live in `wrk_journals/`. Open technical debt is tracked in `tech_debt.md`.

## Non-Obvious Conventions

- **Bank contention model**: dual-core `step` runs core 0 first (recording which downstream port it touched in `core0_port`), then runs core 1 with `contention_check_active` — any same-port access adds +1 cycle. If core 1 is halted, contention checking is skipped entirely. This is why the `Bus` struct carries both flags.
- **Pacer is `x86_64` only.** Don't assume it's available on other targets; gate usage accordingly.
- **Clock tree is mutable at runtime.** Firmware can reprogram PLL/dividers via CLOCKS registers; the `ClockTree` cache on `Bus` is recomputed on each relevant register write. Don't hardcode frequencies.
- **Hardware harness needs real silicon.** `probe_diff` / `probe_verify` / `bank_conflict_test` will not run in CI — they require a Pico debug probe attached to an RP2354 board.
- **The `bin/` directory under `crates/mdpicoem-harness/src/` is tracked intentionally** — don't re-add a broad `bin/` rule to `.gitignore`; it silently hides test binaries.
- **ROMs live under `roms/rp2350/`** (and `roms/rp2040/` once Phase 5 populates it). Pre-restructure code referenced bare `roms/...`; post-restructure, paths are `roms/rp2350/<file>`.

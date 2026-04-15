# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## mdpicoem — RP2350 / RP2040 Emulator Workspace

Cycle-accurate emulators for the Raspberry Pi RP2354 / RP2350 family (dual Cortex-M33 + PIO) and the RP2040 (dual Cortex-M0+ + PIO). Rust workspace under `crates/` after the post-restructure state (Phases 1–7 complete):

- `mdpicoem-common` — shared primitives crate: `Memory`, `ClockTree` math, `Pacer`, PIO primitive types (`PioBlock`/`StateMachine`), divider/FIFO, `Peripheral` trait.
- `mdrp2350` — the RP2350/RP2354 emulator library (dual Cortex-M33 cores, bus, memory, SIO, PIO, clocks, pacer).
- `mdrp2350app` — the TUI demo app driving `mdrp2350`.
- `mdrp2040` — the RP2040 emulator library (dual Cortex-M0+ cores, bus, memory, SIO, PIO, clocks).
- `mdrp2040app` — the TUI demo app driving `mdrp2040`.
- `mdpicoem-harness` — differential test binaries (QEMU diff + probe-rs diff variants per chip, softfloat diff, paced benchmark, full-test runner).
- `mdpicoem-debug` — GDB RSP scaffolding (stub).

See `wrk_docs/2026.04.14 - HLD - mdpicoem Workspace Restructure.md` for the phase-by-phase restructure plan.

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

Two QEMU differential oracles, one per chip:

- **`qemu_diff_m33`** (the RP2350/RP2354 oracle) spawns a QEMU Cortex-M33 on GDB port `3333`, runs the same instruction in both QEMU and `mdrp2350`, and diffs R0–R15 + xPSR (with masking for architecturally unpredictable flag fields).
- **`qemu_diff_m0plus`** (the RP2040 oracle) spawns a QEMU `cortex-m0` on GDB port `3334` (QEMU 10.2 has no `cortex-m0plus` model; M0+ is a strict ISA superset of M0 for the subset under test — see `tech_debt.md`), runs the same instruction in both QEMU and `mdrp2040`, and diffs the same state.

```bash
# RP2350 / Cortex-M33 oracle
cargo run -p mdpicoem-harness --release --bin qemu_diff_m33 -- --fuzz <N>
cargo run -p mdpicoem-harness --release --bin qemu_diff_m33 -- --fuzz <N> --seed <S>
cargo run -p mdpicoem-harness --release --bin qemu_diff_m33             # edge cases only

# RP2040 / Cortex-M0+ oracle
cargo run -p mdpicoem-harness --release --bin qemu_diff_m0plus -- --fuzz <N>
cargo run -p mdpicoem-harness --release --bin qemu_diff_m0plus -- --fuzz <N> --seed <S>
cargo run -p mdpicoem-harness --release --bin qemu_diff_m0plus          # edge cases only
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

### Running alongside concurrent builds (Windows)

Windows holds an exclusive lock on a running `.exe`. While `qemu_diff_m33.exe` / `qemu_diff_m0plus.exe` is fuzzing, any link step that tries to overwrite *that specific binary* — workspace-wide `cargo build --release`, or `-p mdpicoem-harness` — will fail with an access-denied linker error. This blocks other agents rebuilding the harness.

Scope is narrow: builds and tests that don't touch the harness binary (e.g. `cargo build -p mdrp2350`, `cargo test -p mdrp2040`) are unaffected.

When starting a long fuzz run, copy the binary first so `target/release/<bin>.exe` stays free:

```bash
cp target/release/qemu_diff_m33.exe /tmp/fuzzer.exe
/tmp/fuzzer.exe --fuzz 100000
```

The overnight drivers under `fuzz-runs/` (`run-m33.sh`, `run-m0plus.sh`, `run-probe.sh`) already do this — they copy the harness to `fuzz-runs/<bin>.<pid>.exe` at startup and delete it on exit.

## Workspace Layout

- **`crates/mdpicoem-common`** — shared primitives pulled out in Phase 2: `Memory` (with `new()` for RP2350 sizes and `with_sizes(rom, sram)` for RP2040 / future chips), `ClockTree` math, `Pacer` (`x86_64`-only), `PioBlock`/`StateMachine`, `Divider`, `Fifo`, and the `Peripheral` trait. Both chip crates depend on this.
- **`crates/mdrp2350`** — the RP2350/RP2354 emulator core (dual Cortex-M33 / ARMv8-M Mainline, bus, 520 KB SRAM, 32 KB bootrom, XIP flash, clocks, SIO, PIO, FPU, coprocessors). All the RP2350 hot path lives here.
- **`crates/mdrp2350app`** — interactive TUI (ratatui/crossterm) for the RP2350 emulator: register/memory/trace inspection and firmware loading.
- **`crates/mdrp2040`** — the RP2040 emulator core (dual Cortex-M0+ / ARMv6-M, bus, 264 KB SRAM across 4 striped + 2 scratch banks, 16 KB bootrom, **no onboard flash**, clocks, SIO, PIO). No FPU/coprocessors/secure world.
- **`crates/mdrp2040app`** — interactive TUI for the RP2040 emulator. Same shape as `mdrp2350app` minus the FPU/DCP/RCP/NS panels; its ISA panel carries M0+-specific cycle numbers.
- **`crates/mdpicoem-debug`** — GDB RSP server + trace scaffolding (currently a stub).
- **`crates/mdpicoem-harness`** — all test binaries (see "Testing Topology" below). Binaries are chip-suffixed: `qemu_diff_m33` vs `qemu_diff_m0plus`, `probe_diff_rp2350` vs `probe_diff_rp2040` (stub), etc.

The top-level `src/main.rs` is a sanity-check stub that prints emulator config. **The real entry points are `mdrp2350app` and `mdrp2040app`, not this binary.**

## Core Emulator Architecture (`crates/mdrp2350/src/`)

- **`lib.rs`** — `Emulator` aggregates two `CortexM33` cores, `Bus`, `Clock`. Public API: `step`/`run`/`load_bootrom`/`load_flash`. Builder pattern for construction.
- **`core/`** — CPU implementation:
  - `mod.rs` — `CortexM33` struct, fetch-decode-execute loop, multi-cycle stall tracking, IT-block state, exception entry/return.
  - `decode.rs` — Thumb-16 / Thumb-32 decoder → operation enum.
  - `execute.rs` + `execute_thumb32.rs` — instruction semantics (hot path; `execute_thumb32.rs` is large, search by instruction mnemonic).
  - `execute_fpu.rs` — VFPv5 single-precision subset with lazy FP context save (FPCCR/FPCAR).
  - `exceptions.rs` — vector table, stacking, `EXC_RETURN`, NVIC integration, fault handlers.
  - `coprocessor.rs` — CP dispatch (GPIO/CP0 → SIO; DCP on CP4/5; RCP on CP7).
- **`bus/`** — AHB5 address decode, cycle accounting, APB bridge latency, peripheral register backing store. No bank-contention accounting in the production path (see "Bank contention model" below).
- **`bus/clocks.rs`** — ROSC / XOSC / PLL_SYS / PLL_USB / divider model. Recomputes the cached `ClockTree` on register writes.
- **`memory/`** — untimed ROM (32 KB) / SRAM (520 KB across 10 banks) / XIP flash backing storage.
- **`sio/`** — single-cycle IO: GPIO, spinlocks, FIFOs, interpolators, coprocessor interface.
- **`pacer.rs`** — atomic cycle/nanosecond accounting for wall-clock pacing. **`x86_64`-only** (gated at module level).
- **`tests.rs`** — massive in-crate unit test file for instruction semantics, decode edge cases, exception mechanics, clock tree config.

## Core Emulator Architecture (`crates/mdrp2040/src/`)

- **`lib.rs`** — `Emulator` aggregates two `CortexM0Plus` cores and a `Bus`. Public API mirrors mdrp2350 (`step`/`run`/`load_bootrom`/`load_image`/`gpio_*`). Builder pattern with a `step_quantum` field (currently **unused** by `step()` — see `tech_debt.md`; per-instruction cadence, not quantum).
- **`core/`** — ARMv6-M CPU implementation:
  - `mod.rs` — `CortexM0Plus` struct, fetch-decode-execute loop, 2-bit CONTROL, banked MSP/PSP (with explicit `sync_sp_to_banked`/`sync_sp_from_banked` around exception entry/exit), `pending_fault`. No IT blocks, no FAULTMASK, no BASEPRI.
  - `registers.rs` — ARMv6-M register set.
  - `decode.rs` — group dispatch + `is_wide` that only accepts the `0b11110` prefix (other wide forms fault on this CPU).
  - `execute.rs` — Thumb-16 executor. CBZ/CBNZ/IT rejected as UNDEFINED. Cycle counts hardcoded per M0+ r0p1 (`MULS=1`, `LDR=2`, `LDM N`=1+N, `B`=1–3, `BL`=4).
  - `execute_wide.rs` — the small Thumb-32 subset M0+ actually supports: `BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`. **Not currently exercised by `qemu_diff_m0plus`** (see `tech_debt.md`).
  - `exceptions.rs` — vector table, stacking, `EXC_RETURN` `0xF1`/`0xF9`/`0xFD`, invalid EXC_RETURN → HardFault, PRIMASK-escalates-SVC-to-HardFault.
- **`bus/`** — RP2040 address decode, SRAM bank striping + contention, CLOCKS / RESETS / XOSC / ROSC / PLL_SYS / PLL_USB register model, SIO (GPIO, CPUID, FIFO, 32 spinlocks, divider, interp storage), IO_BANK0, PADS_BANK0, XIP_CTRL/SSI stubs, two `PioBlock`s, minimal PPB. `bus_fault` sticky flag observed by `CortexM0Plus::step` and escalated to HardFault.
- **`memory.rs`** — thin wrapper around `mdpicoem_common::Memory::with_sizes(16 KB ROM, 264 KB SRAM)`. **No onboard flash** on RP2040 — firmware images load into SRAM via `load_image`.
- **`tests.rs`** / **`pio_tests.rs`** / **`tests/firmware.rs`** — unit tests for instruction semantics, PIO through the bus, and end-to-end firmware smoke paths.

## Testing Topology

Per-chip independent oracles, each catching different bugs:

1. **Unit tests**
   - `crates/mdrp2350/src/tests.rs` (+ `pio_tests.rs`) — M33 instruction semantics, decode, exceptions, clock tree.
   - `crates/mdrp2040/src/tests.rs` (+ `pio_tests.rs`, `tests/firmware.rs`) — M0+ instruction semantics, decode, exceptions, PIO, firmware smoke.
   - `crates/mdrp2040app/` smoke test — launches the emulator, loads `roms/rp2040/blinky.bin`, asserts GPIO25 flips within 2 seconds.
2. **`qemu_diff_m33`** — QEMU Cortex-M33 reference vs. `mdrp2350`, via GDB single-step on port `3333`. Catches M33 architectural mistakes (flag computation, wide-instruction decode, PC-relative addressing).
3. **`qemu_diff_m0plus`** — QEMU `cortex-m0` reference vs. `mdrp2040`, via GDB single-step on port `3334`. Same idea for the M0+ ISA subset. **Caveat**: filters out all 32-bit-wide Thumb encodings today (`is_m0plus_safe`), so the Thumb-32 subset — `BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB` — is unit-test-only. See `tech_debt.md`.
4. **`probe_diff_rp2350`** + **`probe_verify_rp2350`** + **`bank_conflict_test_rp2350`** — RP2350-specific probe-rs 0.31 differentials against **real RP2354 silicon** via SWD. Catches behaviours QEMU gets wrong. `bank_conflict_test_rp2350` characterises real-silicon SRAM bank contention timing for reference; the emulator does **not** model contention on RP2350 by design (see "Bank contention model" in Non-Obvious Conventions). Requires a Pico debug probe attached to RP2354 hardware.
5. **`probe_diff_rp2040`** — **stub**. Exits 2 with a rationale; no probe-rs wiring exists because the lab rig only carries an RP2354 board. When real RP2040 silicon becomes available, mirror the RP2350 runner and share the `is_m0plus_safe` fuzz filter. For now, the ISA-level differential oracle for mdrp2040 is `qemu_diff_m0plus` only.
6. **`paced_bench_rp2350`** and **`full_test_rp2350`** — RP2350 real-time paced throughput / integration smoke test.
7. **`silicon_cycle_oracle_rp2350`** — measures true instruction-sequence cycle cost on real RP2354 silicon at native speed via a Thumb measurement stub plus a mailbox handshake over SWD. Each case is run twice at different iteration counts (K=101 vs K=201) and the K-delta cancels per-invocation framing (BLX/BX, stub entry) to isolate steady-state per-iteration cost. The emulator side runs the same sequence through the standard step path and reports HW/EMU/delta per case. Requires a Pico debug probe attached to an RP2354 board (same prerequisite as the other `probe_*` / `bank_conflict_*` runners) — not for CI. `cargo run -p mdpicoem-harness --release --bin silicon_cycle_oracle_rp2350`; see `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md` §Oracle 2 for the catalog.
8. **`silicon_periph_diff_rp2350`** — end-state differential oracle for peripheral state against live RP2354 silicon. Each scenario applies an identical MMIO setup sequence and a CYCCNT-measured sysclk window to both silicon (via probe-rs) and a `step_quantum=1` emulator (via `Emulator::run` with halted cores), then diffs a scenario-declared set of observable registers and pins. Covers PIO (register + FIFO + pad state), PLL LOCK timing, and clock-tree reprogramming under load via the `custom_sled` extension. Same HW prerequisite as the cycle oracle — Pico probe + RP2354, not for CI. `cargo run -p mdpicoem-harness --release --bin silicon_periph_diff_rp2350`; see `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md` §Oracle 1 for the catalog.
9. **`silicon_dualcore_diff_rp2350`** — cross-core contention oracle. Reuses the cycle oracle's K-delta `MEASUREMENT_STUB` on core 0 while a per-case antagonist sequence runs on core 1 (released with a custom PC into an infinite loop in SRAM bank 5 to keep I-fetch contention out of the data-bank signal). Catalogue: bank-thrash same/diff control pair, spinlock churn, FIFO transfer. Validates the emulator's `Bus::contention_check_active` model against real concurrent-execution timing. Same HW prerequisite. **Depends on Assumption 1** (per-core CYCCNT alias on RP2354) — verify with `smoke_per_core_cyccnt_rp2350` first. See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage Expansion.md` §Component 3.
10. **`silicon_isr_diff_rp2350`** — end-state differential oracle for ARMv8-M exception entry, lazy FP context save, and tail-chained ISRs. Each scenario uploads a per-image vector table + handler stub + main routine into SRAM, programs CPU init state (R0..R12 + MSP/PSP/CONTROL/CPACR/VTOR with both Secure and Non-Secure aliases written), triggers the exception, halts in the handler at BKPT #0, and diffs observables (MMIO, stacked-frame slots, CYCCNT mailbox). Catalogue: cold PendSV, lazy FP save, eager FP save, PendSV+SysTick tail-chain. **Designed to expose `tech_debt.md:295`** — `CortexM33::step` does not poll `ICSR.PENDSVSET`/`PENDSTSET`, so all four scenarios are EXPECTED to FAIL on the EMU side until the dispatch path is wired; the oracle is the surfacing tool for it. Same HW prerequisite. See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage Expansion.md` §Component 2.
11. **`test_silicon`** — orchestrator that wraps oracles 4, 7, 8, 9, 10 (probe_diff/cycle/periph/dualcore/isr — bank_conflict_test rolled into the cycle catalogue's K-delta protocol) under one shared probe session. Single-pass mode is the day-to-day driver; `--soak <duration>` runs continuously with per-iteration Fisher-Yates shuffling of each oracle's case order, prints failures immediately + hourly heartbeat, and survives transient probe errors via per-case 60s watchdog + drop-and-reattach. Designed for unattended multi-day runs on real RP2354. CLI: `--soak <duration> --seed <u64> --filter <substr> --verbose`. Same HW prerequisite. See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage Expansion.md` §Component 1.
12. **`smoke_per_core_cyccnt_rp2350`** — one-shot disposable smoke binary that verifies Assumption 1 of the test_silicon HLD (per-core DWT CYCCNT on RP2354). Differentiator design: core 0 spins in an infinite NOP loop with DWT enabled; core 1 is halted (NOT reset) and its CYCCNT alias is read WITHOUT enabling DWT on core 1. Distinguishes per-core (N1 == 0) from aliased (N1 ≈ N0) DWT. Run once before relying on `silicon_dualcore_diff_rp2350`'s cycle measurements; delete the binary after.

## High-Level Design Documents

Under `wrk_docs/`. HLDs are **phase-based and dated** (`YYYY.MM.DD - HLD - <topic> V<N>.md`). The original master HLD is `2026.04.12 - RP2350 Emulator HLD.md`, but subsequent phase HLDs (Phase 2 bus, Phase 3 interrupts, Phase 4 flash boot, Phase 5 dual-core SIO, Phase 6 PIO, Phase 7 coprocessors/FPU) supersede relevant sections. The workspace restructure itself lives in `2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

When working on a specific subsystem, **read the latest HLD for that phase** — not the master HLD. Later-dated versions (e.g. V5 over V2) supersede earlier drafts of the same phase.

Per-session notes live in `wrk_journals/`. Open technical debt is tracked in `tech_debt.md`.

## Non-Obvious Conventions

- **Bank contention model (RP2040 only, deprecated-in-place)**: dual-core `step` runs core 0 first (recording which downstream port it touched in `core0_port`), then runs core 1 with `contention_check_active` — any same-port access adds +1 cycle. If core 1 is halted, contention checking is skipped entirely. This is why the `Bus` struct carries both flags. **Do not extend this model to mdrp2350, and do not invest further on mdrp2040**: the perf gap against real silicon (see `paced_bench_rp2*`) swamps the contention-accuracy gain by an order of magnitude, contention cycles are virtual (they don't consume host compute), and threaded cores on the roadmap will obsolete the serial-interleave path entirely. mdrp2350 `contention`/`stress` bench workloads are dual-core-compute-only by design. Rationale: `wrk_journals/2026.04.15 - JRN - Contention Modelling Declined.md`.
- **Pacer is `x86_64` only.** Don't assume it's available on other targets; gate usage accordingly.
- **Clock tree is mutable at runtime.** Firmware can reprogram PLL/dividers via CLOCKS registers; the `ClockTree` cache on `Bus` is recomputed on each relevant register write. Don't hardcode frequencies.
- **Hardware harness needs real silicon.** `probe_diff_rp2350` / `probe_verify_rp2350` / `bank_conflict_test_rp2350` will not run in CI — they require a Pico debug probe attached to an RP2354 board. `probe_diff_rp2040` is currently a stub (no RP2040 board on the lab rig); the ISA oracle for mdrp2040 is `qemu_diff_m0plus` only.
- **The `bin/` directory under `crates/mdpicoem-harness/src/` is tracked intentionally** — don't re-add a broad `bin/` rule to `.gitignore`; it silently hides test binaries.
- **ROMs live under `roms/rp2350/` and `roms/rp2040/`.** Pre-restructure code referenced bare `roms/...`; post-restructure, paths are `roms/rp2350/<file>` (blinky, bootrom, LCD demo, benchmark firmware) and `roms/rp2040/<file>` (blinky + 16 KB bootrom generated by `roms/rp2040/gen_blinky.py`).
- **Windows-only: hung oracle process trees.** When a fuzz oracle child tree zombies and `taskkill` doesn't terminate it, see [`RUNBOOK.md`](RUNBOOK.md) for the `kill -9 <POSIX_PID>` recipe using `ps -W` PPID walking.

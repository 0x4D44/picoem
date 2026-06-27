# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## picoem — RP2350 / RP2040 Emulator Workspace

Cycle-accurate emulators for the Raspberry Pi RP2354 / RP2350 family (dual Cortex-M33 + PIO) and the RP2040 (dual Cortex-M0+ + PIO). Rust workspace under `crates/` after the post-restructure state (Phases 1–7 complete):

- `picoem-common` — shared primitives crate: `Memory`, `ClockTree` math, portable `Pacer`, PIO primitive types (`PioBlock`/`StateMachine`), divider/FIFO, and portable `threaded::{SpinBarrier, SpscQueue}` primitives.
- `picoem-devices` — off-chip device models used by harnesses and apps: `psram.rs`, `lcd.rs`, `i2s_capture.rs`. Depended on by `rp2040-emu` (PSRAM compat).
- `rp2350-emu` — the RP2350/RP2354 emulator library (dual Cortex-M33 + RISC-V Hazard3 cores, bus, memory, SIO, PIO, DMA, full peripheral suite, clocks, pacer, threaded runtime).
- `rp2350-emu-tui` — the TUI demo app driving `rp2350-emu`.
- `rp2040-emu` — the RP2040 emulator library (dual Cortex-M0+ cores, bus, memory, SIO, PIO, clocks, threaded runtime).
- `rp2040-emu-tui` — the TUI demo app driving `rp2040-emu`.
- `picoem-harness` — differential test binaries (QEMU diff + probe-rs diff variants per chip, softfloat diff, paced benchmark, silicon oracles, PicoGUS replay, OneROM oracles, test_silicon orchestrator).
- `picoem-debug` — GDB RSP scaffolding (stub).
- `epio-sys` — native FFI for the reference PIO simulator. **Excluded from workspace `default-members`** because it requires `clang` and vendored submodules; build explicitly with `cargo build -p epio-sys`.

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
cargo test -p rp2350-emu

# Code coverage
cargo llvm-cov
```

## Differential Fuzz Testing (QEMU harness)

Two QEMU differential oracles, one per chip:

- **`qemu_diff_m33`** (the RP2350/RP2354 oracle) spawns a QEMU Cortex-M33 on GDB port `3333`, runs the same instruction in both QEMU and `rp2350-emu`, and diffs R0–R15 + xPSR (with masking for architecturally unpredictable flag fields).
- **`qemu_diff_m0plus`** (the RP2040 oracle) spawns a QEMU `cortex-m0` on GDB port `3334` (QEMU 10.2 has no `cortex-m0plus` model; M0+ is a strict ISA superset of M0 for the subset under test — see `tech_debt.md`), runs the same instruction in both QEMU and `rp2040-emu`, and diffs the same state.

```bash
# RP2350 / Cortex-M33 oracle
cargo run -p picoem-harness --release --bin qemu_diff_m33 -- --fuzz <N>
cargo run -p picoem-harness --release --bin qemu_diff_m33 -- --fuzz <N> --seed <S>
cargo run -p picoem-harness --release --bin qemu_diff_m33             # edge cases only

# RP2040 / Cortex-M0+ oracle
cargo run -p picoem-harness --release --bin qemu_diff_m0plus -- --fuzz <N>
cargo run -p picoem-harness --release --bin qemu_diff_m0plus -- --fuzz <N> --seed <S>
cargo run -p picoem-harness --release --bin qemu_diff_m0plus          # edge cases only
```

### Typical fuzz sessions

| Goal | Command |
|---|---|
| Quick smoke test | `--fuzz 1000` |
| Standard session | `--fuzz 100000` |
| Extended soak | `--fuzz 1000000` (or more) |

When asked to "fuzz test" or "do some fuzzing", default to `--fuzz 100000` unless a different count or duration is specified. For time-based requests ("fuzz for 2 hours"), estimate iterations from prior run throughput and adjust.

On `test_rp2350_qemu_diff_riscv32`, `--fuzz N --class X` now means "N cases of class X" (pre-dispatch); pre-2026-04-18 it meant "N mixed cases post-filtered to class X" (so low-weight classes produced far fewer than N). Seed reproducibility for `--fuzz N` without `--class` (the regression-gate path) is byte-identical across this change.

### Handling failures

When the harness reports a mismatch:
1. Note the seed and instruction class from the failure output.
2. Reproduce with `--seed <S>` to get a deterministic repro.
3. Investigate the specific instruction's decode/execute path in our emulator.
4. Fix and re-run the same seed to confirm.

### Running alongside concurrent builds (Windows)

Windows holds an exclusive lock on a running `.exe`. While `qemu_diff_m33.exe` / `qemu_diff_m0plus.exe` is fuzzing, any link step that tries to overwrite *that specific binary* — workspace-wide `cargo build --release`, or `-p picoem-harness` — will fail with an access-denied linker error. This blocks other agents rebuilding the harness.

Scope is narrow: builds and tests that don't touch the harness binary (e.g. `cargo build -p rp2350-emu`, `cargo test -p rp2040-emu`) are unaffected.

When starting a long fuzz run, copy the binary first so `target/release/<bin>.exe` stays free:

```bash
cp target/release/qemu_diff_m33.exe /tmp/fuzzer.exe
/tmp/fuzzer.exe --fuzz 100000
```

The overnight drivers under `fuzz-runs/` (`run-m33.sh`, `run-m0plus.sh`, `run-probe.sh`) already do this — they copy the harness to `fuzz-runs/<bin>.<pid>.exe` at startup and delete it on exit.

## Workspace Layout

- **`crates/picoem-common`** — shared primitives pulled out in Phase 2: `Memory` (with `new()` for RP2350 sizes and `with_sizes(rom, sram)` for RP2040 / future chips), `ClockTree` math, portable `Pacer` (`rdtsc` on `x86_64`, `Instant` elsewhere), `PioBlock`/`StateMachine`, `Divider`, `Fifo`. `threaded::{SpinBarrier, SpscQueue}` are portable cross-chip thread-coordination primitives used by each chip's `threaded/` runtime; worker affinity helpers remain OS-gated. Both chip crates depend on this. Note: a former `Peripheral` trait was removed (zero impls workspace-wide); peripherals now use inherent methods invoked from `Bus` per `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V7.md` §5.1.
- **`crates/picoem-devices`** — off-chip device emulation shared by harnesses and apps: `psram.rs` (HyperRAM-style external PSRAM, consumed by `rp2040-emu` for `test_psram` compat), `lcd.rs`, `i2s_capture.rs`.
- **`crates/rp2350-emu`** — the RP2350/RP2354 emulator core (dual Cortex-M33 / ARMv8-M Mainline, bus, 520 KB SRAM, 32 KB bootrom, XIP flash, clocks, SIO, PIO, FPU, coprocessors, 16-channel DMA, and the peripheral suite: TIMER0/1, TICKS, POWMAN, UART0/1, SPI0/1, I2C0/1, PWM, ADC, WATCHDOG, OTP, TRNG, SHA256, PSM, IO_BANK0, PADS_BANK0, CoreSight). A parallel RISC-V Hazard3 core (`core_riscv/`) shares the same bus. The `threaded/` subtree (feature `threading`, `x86_64` Windows/Linux only) provides a 6-thread worker runtime selected by `ExecutionModel::Threaded`. All the RP2350 hot path lives here.
- **`crates/rp2350-emu-tui`** — interactive TUI (ratatui/crossterm) for the RP2350 emulator: register/memory/trace inspection and firmware loading.
- **`crates/rp2040-emu`** — the RP2040 emulator core (dual Cortex-M0+ / ARMv6-M, bus, 264 KB SRAM across 4 striped + 2 scratch banks, 16 KB bootrom, **no onboard flash**, clocks, SIO, PIO). No FPU/coprocessors/secure world. The `threaded/` subtree (same platform/feature gating as rp2350-emu) provides a 3-thread runtime selected by `ExecutionModel::Threaded`. Depends on `picoem-devices` for PSRAM.
- **`crates/rp2040-emu-tui`** — interactive TUI for the RP2040 emulator. Same shape as `rp2350-emu-tui` minus the FPU/DCP/RCP/NS panels; its ISA panel carries M0+-specific cycle numbers.
- **`crates/picoem-debug`** — GDB RSP server + trace scaffolding (currently a stub).
- **`crates/picoem-harness`** — all test binaries (see "Testing Topology" below). Binaries are chip-suffixed: `qemu_diff_m33` vs `qemu_diff_m0plus`, `probe_diff_rp2350` vs `probe_diff_rp2040`, `silicon_periph_diff_rp2350` vs `silicon_periph_diff_rp2040`, etc.
- **`crates/epio-sys`** — native FFI binding to the reference PIO simulator (`links = "epio"`). Excluded from `default-members` so the root `cargo build` still works on hosts without clang; build explicitly with `cargo build -p epio-sys`.

**The real entry points are `rp2350-emu-tui` and `rp2040-emu-tui`.** The workspace has no top-level binary; build/run via `cargo run -p rp2350-emu-tui` or `cargo run -p rp2040-emu-tui`.

## Core Emulator Architecture (`crates/rp2350-emu/src/`)

- **`lib.rs`** — `Emulator` aggregates two `CortexM33` cores, `Bus`, `Clock`. Public API: `step`/`run`/`load_bootrom`/`load_flash`. `EmulatorBuilder::build() -> Result<Emulator, ConfigError>` with an `execution(ExecutionModel)` selector (`Serial` / `Threaded`); see "Execution Models" below.
- **`core/`** — CPU implementation:
  - `mod.rs` — `CortexM33` struct, fetch-decode-execute loop, multi-cycle stall tracking, IT-block state, exception entry/return.
  - `decode.rs` — Thumb-16 / Thumb-32 decoder → operation enum.
  - `execute.rs` + `execute_thumb32.rs` — instruction semantics (hot path; `execute_thumb32.rs` is large, search by instruction mnemonic).
  - `execute_fpu.rs` — VFPv5 single-precision subset with lazy FP context save (FPCCR/FPCAR).
  - `exceptions.rs` — vector table, stacking, `EXC_RETURN`, NVIC integration, fault handlers.
  - `coprocessor.rs` — CP dispatch (GPIO/CP0 → SIO; DCP on CP4/5; RCP on CP7).
- **`bus/`** — AHB5 address decode, cycle accounting, APB bridge latency, peripheral dispatch. No bank-contention accounting in the production path (see "Bank contention model" below).
- **`bus/clocks.rs`** — ROSC / XOSC / PLL_SYS / PLL_USB / divider model. Recomputes the cached `ClockTree` on register writes.
- **`memory/`** — untimed ROM (32 KB) / SRAM (520 KB across 10 banks) / XIP flash backing storage.
- **`sio/`** — single-cycle IO: GPIO, spinlocks, FIFOs, interpolators, coprocessor interface.
- **`dma.rs`** (+ **`dreq.rs`**) — 16-channel DMA bus master with `CTRL_TRIG`/`AL1`/`AL2`/`AL3` register aliases, `RING_SIZE`+`RING_SEL` circular buffers, `CHAIN_TO` chained triggering, `CH_ABORT`, fixed-priority (lowest-index-wins) arbitration, and the full DREQ matrix. `DMA_IRQ_0`/`DMA_IRQ_1` on NVIC lines 10/11. Silicon-verified via `dma_chain_trigger` + `dma_timer_paced` scenarios in `silicon_periph_diff_rp2350`. Not in V1: CRC sniff, byte-swap, `HIGH_PRIORITY`, `DMA_IRQ_2`/`DMA_IRQ_3`, read/write-error IRQs.
- **`peripherals/`** — per-peripheral modules with inherent methods invoked from `Bus` (per `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V7.md` §5.1; the earlier `Peripheral` trait was removed): `timer.rs` (TIMER0/1 + alarms), `ticks.rs` (per-source tick generators), `powman.rs` (always-on timer, password-gated writes + BADPASSWD), `uart.rs` (PL011-derived, byte-lane narrow-access on UARTDR), `spi.rs` (PL022 with LBM loopback timing), `i2c.rs` (DW_apb_i2c), `pwm.rs`, `adc.rs` (clk_adc/clk_sys fixed-point accumulator), `watchdog.rs`, `otp.rs`, `trng.rs`, `sha256.rs`, `psm.rs`, `io_bank0.rs`, `pads_bank0.rs`, `coresight_trace.rs`. `inert.rs` stubs the unmodelled APB holes. Phase 1/2 tests live alongside (`phase1_tests.rs` / `phase2_tests.rs`).
- **`pacer.rs`** — atomic cycle/nanosecond accounting for wall-clock pacing. Uses `rdtsc` on `x86_64` and an `Instant` spin-wait backend elsewhere.
- **`tests.rs`** — massive in-crate unit test file for instruction semantics, decode edge cases, exception mechanics, clock tree config.

## Core Emulator Architecture (`crates/rp2040-emu/src/`)

- **`lib.rs`** — `Emulator` aggregates two `CortexM0Plus` cores and a `Bus`. Public API mirrors rp2350-emu (`step`/`run`/`load_bootrom`/`load_image`/`gpio_*`). `EmulatorBuilder::build() -> Result<Emulator, ConfigError>` with `step_quantum` (now actively honoured — drains up to N master-clock cycles per `step()` call, default 64) and an `execution(ExecutionModel)` selector (`Serial` / `Threaded`, where `Threaded` requires the `threading` feature and `x86_64` Windows/Linux).
- **`core/`** — ARMv6-M CPU implementation:
  - `mod.rs` — `CortexM0Plus` struct, fetch-decode-execute loop, 2-bit CONTROL, banked MSP/PSP (with explicit `sync_sp_to_banked`/`sync_sp_from_banked` around exception entry/exit), `pending_fault`. No IT blocks, no FAULTMASK, no BASEPRI.
  - `registers.rs` — ARMv6-M register set.
  - `decode.rs` — group dispatch + `is_wide` that only accepts the `0b11110` prefix (other wide forms fault on this CPU).
  - `execute.rs` — Thumb-16 executor. CBZ/CBNZ/IT rejected as UNDEFINED. Cycle counts hardcoded per M0+ r0p1 (`MULS=1`, `LDR=2`, `LDM N`=1+N, `B`=1–3, `BL`=4).
  - `execute_wide.rs` — the small Thumb-32 subset M0+ actually supports: `BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB`. **Not currently exercised by `qemu_diff_m0plus`** (see `tech_debt.md`).
  - `exceptions.rs` — vector table, stacking, `EXC_RETURN` `0xF1`/`0xF9`/`0xFD`, invalid EXC_RETURN → HardFault, PRIMASK-escalates-SVC-to-HardFault.
- **`bus/`** — RP2040 address decode, SRAM bank striping + contention, CLOCKS / RESETS / XOSC / ROSC / PLL_SYS / PLL_USB register model, SIO (GPIO, CPUID, FIFO, 32 spinlocks, divider, interp storage), IO_BANK0, PADS_BANK0, XIP_CTRL/SSI stubs, two `PioBlock`s, minimal PPB. `bus_fault` sticky flag observed by `CortexM0Plus::step` and escalated to HardFault.
- **`memory.rs`** — thin wrapper around `picoem_common::Memory::with_sizes(16 KB ROM, 264 KB SRAM)`. **No onboard flash** on RP2040 — firmware images load into SRAM via `load_image`.
- **`tests.rs`** / **`pio_tests.rs`** / **`tests/firmware.rs`** — unit tests for instruction semantics, PIO through the bus, and end-to-end firmware smoke paths.

## Execution Models (Serial vs Threaded)

Both chip crates expose an `ExecutionModel` selector on the builder: `Serial` (the oracle-validated reference path — a single host thread runs both cores interleaved per `step_quantum`) or `Threaded` (a worker-pool runtime: 3 threads on RP2040, 6 threads on RP2350, barrier-synchronised at the quantum boundary). Design: `wrk_docs/2026.04.24 - HLD - Dual Serial and Threaded Execution Models V1.md`.

- **`threading` feature** — opt-in, and the `Threaded` variant is further `cfg`-gated to `x86_64 + Windows/Linux`. On any other target, including macOS, `Builder::build()` with `Threaded` returns `ConfigError::ThreadingUnavailable`. Keep Serial as the default for new harnesses and oracles; opt into Threaded only where you specifically want to exercise the runtime.
- **Oracles validate Serial.** QEMU diff, probe diff, silicon cycle/periph/isr/dualcore — all run Serial. Use the `dual_model` integration test (`cargo test -p rp2350-emu --test dual_model --features threading`, same shape on rp2040-emu) to assert Serial/Threaded end-state parity.
- **Testing-only APIs** live behind the `testing` feature (`Emulator::inject_panic_for_testing`, `threaded::PioCommand::TestPanic`). Never enable `testing` in production builds — it's a "brick your emulator" hook that only exists for the panic-containment suite.
- **Tuning.** `paced_bench_rp2040` / `paced_bench_rp2350` accept `--model {serial,threaded,both}` and `--step-quantum N` (larger quantum amortises the barrier cost in threaded mode; serial is insensitive to quantum). `--model both` is the A/B harness (HLD V1 §7.3 — implies `--unpaced`). `threaded_probe` and `threading_micro` are micro-benchmarks for the worker pool itself.

## Testing Topology

Per-chip independent oracles, each catching different bugs:

1. **Unit tests**
   - `crates/rp2350-emu/src/tests.rs` (+ `pio_tests.rs`) — M33 instruction semantics, decode, exceptions, clock tree.
   - `crates/rp2040-emu/src/tests.rs` (+ `pio_tests.rs`, `tests/firmware.rs`) — M0+ instruction semantics, decode, exceptions, PIO, firmware smoke.
   - `crates/rp2040-emu-tui/` smoke test — launches the emulator, loads `roms/rp2040/blinky.bin`, asserts GPIO25 flips within 2 seconds.
   - **Threaded-runtime integration tests** — `crates/rp2350-emu/tests/dual_model.rs` + `crates/rp2040-emu/tests/dual_model.rs` assert Serial/Threaded end-state parity (require `--features threading`); `crates/rp2350-emu/tests/execution_model.rs` + `crates/rp2040-emu/tests/execution_model.rs` cover panic-containment via `ExecutionModel::Threaded` + `inject_panic_for_testing` (require `--features threading,testing`). `crates/rp2350-emu/tests/hello_riscv_blinky.rs` is the Hazard3 firmware smoke.
2. **`qemu_diff_m33`** — QEMU Cortex-M33 reference vs. `rp2350-emu`, via GDB single-step on port `3333`. Catches M33 architectural mistakes (flag computation, wide-instruction decode, PC-relative addressing).
3. **`qemu_diff_m0plus`** — QEMU `cortex-m0` reference vs. `rp2040-emu`, via GDB single-step on port `3334`. Same idea for the M0+ ISA subset. **Caveat**: filters out all 32-bit-wide Thumb encodings today (`is_m0plus_safe`), so the Thumb-32 subset — `BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB` — is unit-test-only. See `tech_debt.md`.
4. **`probe_diff_rp2350`** + **`probe_verify_rp2350`** + **`bank_conflict_test_rp2350`** — RP2350-specific probe-rs 0.31 differentials against **real RP2354 silicon** via SWD. Catches behaviours QEMU gets wrong. `bank_conflict_test_rp2350` characterises real-silicon SRAM bank contention timing for reference; the emulator does **not** model contention on RP2350 by design (see "Bank contention model" in Non-Obvious Conventions). Requires a Pico debug probe attached to RP2354 hardware.
5. **`probe_diff_rp2040`** — M0+ ISA differential oracle against live RP2040 silicon via SWD (probe-rs 0.31). Unlike `qemu_diff_m0plus`, its `is_m0plus_silicon_safe` filter admits the M0+ Thumb-32 subset (`BL`, `MRS`/`MSR` for `PRIMASK`/`CONTROL`, `DSB`/`DMB`/`ISB`), so silicon is the only oracle for those encodings. No cycle comparison — M0+ has no DWT CYCCNT, so `--cycles` is intentionally absent. A post-step `UndefOnSilicon` diagnostic flags filter-gap escapes (PC landed in the RP2040 bootrom range `< 0x0000_4000` after the step) distinctly from `[FAIL]`/`[SKIP]`. `--probe <VID:PID:SERIAL>` disambiguates on hosts with multiple probes attached. Requires a Pico debug probe on a Pico V1 / RP2040 board — not for CI. `cargo run -p picoem-harness --release --bin probe_diff_rp2040`.
6. **`paced_bench_rp2350`** and **`full_test_rp2350`** — RP2350 real-time paced throughput / integration smoke test.
7. **`silicon_cycle_oracle_rp2350`** — measures true instruction-sequence cycle cost on real RP2354 silicon at native speed via a Thumb measurement stub plus a mailbox handshake over SWD. Each case is run twice at different iteration counts (K=101 vs K=201) and the K-delta cancels per-invocation framing (BLX/BX, stub entry) to isolate steady-state per-iteration cost. The emulator side runs the same sequence through the standard step path and reports HW/EMU/delta per case. Requires a Pico debug probe attached to an RP2354 board (same prerequisite as the other `probe_*` / `bank_conflict_*` runners) — not for CI. `cargo run -p picoem-harness --release --bin silicon_cycle_oracle_rp2350`; see `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md` §Oracle 2 for the catalog.
8. **`silicon_periph_diff_rp2350`** — end-state differential oracle for peripheral state against live RP2354 silicon. Each scenario applies an identical MMIO setup sequence and a CYCCNT-measured sysclk window to both silicon (via probe-rs) and a `step_quantum=1` emulator (via `Emulator::run` with halted cores), then diffs a scenario-declared set of observable registers and pins. Covers PIO (register + FIFO + pad state), PLL LOCK timing, clock-tree reprogramming under load, and DMA (`dma_chain_trigger`, `dma_timer_paced`) via the `custom_sled` extension. Same HW prerequisite as the cycle oracle — Pico probe + RP2354, not for CI. `cargo run -p picoem-harness --release --bin silicon_periph_diff_rp2350`; see `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md` §Oracle 1 for the catalog.
9. **`silicon_dualcore_diff_rp2350`** — cross-core contention oracle. Reuses the cycle oracle's K-delta `MEASUREMENT_STUB` on core 0 while a per-case antagonist sequence runs on core 1 (released with a custom PC into an infinite loop in SRAM bank 5 to keep I-fetch contention out of the data-bank signal). Catalogue: bank-thrash same/diff control pair, spinlock churn, FIFO transfer. Validates the emulator's `Bus::contention_check_active` model against real concurrent-execution timing. Same HW prerequisite. **Depends on Assumption 1** (per-core CYCCNT alias on RP2354) — verify with `smoke_per_core_cyccnt_rp2350` first. See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage Expansion.md` §Component 3.
10. **`silicon_isr_diff_rp2350`** — end-state differential oracle for ARMv8-M exception entry, lazy FP context save, and tail-chained ISRs. Each scenario uploads a per-image vector table + handler stub + main routine into SRAM, programs CPU init state (R0..R12 + MSP/PSP/CONTROL/CPACR/VTOR with both Secure and Non-Secure aliases written), triggers the exception, halts in the handler at BKPT #0, and diffs observables (MMIO, stacked-frame slots, CYCCNT mailbox). Catalogue: cold PendSV, lazy FP save, eager FP save, PendSV+SysTick tail-chain, external IRQ (TIMER0 cold, masked-pending, priority-preempt). ICSR dispatch is wired (`try_take_any_pending_exception` at `exceptions.rs:422`, called every `step()`); the banked SP staleness bug was fixed (2026-04-16); remaining FAILs surface real divergences in stacked-frame layout, FPCCR state, or CYCCNT delta. See `tech_debt.md` § "Exception entry/exit not differentially validated" for the broader validation roadmap. Same HW prerequisite. See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage Expansion.md` §Component 2.
11. **`test_silicon`** — orchestrator that wraps oracles 4, 7, 8, 9, 10 (probe_diff/cycle/periph/dualcore/isr — bank_conflict_test rolled into the cycle catalogue's K-delta protocol) under one shared probe session. Single-pass mode is the day-to-day driver; `--soak <duration>` runs continuously with per-iteration Fisher-Yates shuffling of each oracle's case order, prints failures immediately + hourly heartbeat, and survives transient probe errors via per-case 60s watchdog + drop-and-reattach. Designed for unattended multi-day runs on real RP2354. CLI: `--soak <duration> --seed <u64> --filter <substr> --verbose`. Same HW prerequisite. See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage Expansion.md` §Component 1.
12. **`smoke_per_core_cyccnt_rp2350`** — one-shot disposable smoke binary that verifies Assumption 1 of the test_silicon HLD (per-core DWT CYCCNT on RP2354). Differentiator design: core 0 spins in an infinite NOP loop with DWT enabled; core 1 is halted (NOT reset) and its CYCCNT alias is read WITHOUT enabling DWT on core 1. Distinguishes per-core (N1 == 0) from aliased (N1 ≈ N0) DWT. Run once before relying on `silicon_dualcore_diff_rp2350`'s cycle measurements; delete the binary after.
13. **`silicon_periph_diff_rp2040`** + **`silicon_isr_diff_rp2040`** — RP2040 siblings of oracles 8 and 10. Peripheral oracle uses a SysTick-based timing window (M0+ has no DWT CYCCNT, so `min/max_sysclks` is a soft assertion); ISR oracle uploads a 17-entry ARMv6-M vector table + handler stub to SRAM and diffs stacked-frame + counter observables. Same HW prerequisite as `probe_diff_rp2040` (Pico probe + RP2040 board). **`silicon_isr_diff_rp2040` ships ahead of silicon hookup** — the oracle exists so the Phase-1 RP2040 IRQ-plumbing exit criterion has something to point at, but scenarios will FAIL on the EMU side until `Bus::irq_pending` + `tick_peripherals` + pending-exception dispatch land in `CortexM0Plus::step` (per the binary's own header comment). See `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V7.md`.
14. **`paced_bench_rp2040`** — RP2040 sibling of `paced_bench_rp2350` (minus the FPU workload). Accepts `--model {serial,threaded,both}`, `--workload {basic,peripheral,contention,stress}`, `--step-quantum N`, `--seconds`/`--cycles`, `--unpaced`. Used for A/B comparison between the Serial and Threaded runtimes and for the RP2040 threading crossover measurement (stage 4 of the dual-execution HLD).
15. **`picogus_diff_rp2040`** (+ **`picogus_probe_pc`**) — PicoGUS ISA-bus replayer. Reads a CSV trace captured from a patched DOSBox-X and drives synthetic ISA cycles into an `rp2040_emu::Emulator` running the real PicoGUS firmware, sampling the I2S pins per cycle and writing decoded stereo PCM to a WAV at the end. Integration goal is "does Monkey Island sound right". See `wrk_docs/2026.04.14 - HLD - PicoGUS Integration.md` and the 2026-04-2x journals for audio-quality progression. `picogus_probe_pc` is the live-silicon variant.
16. **OneROM oracles** — `onerom_cpu_probe`, `onerom_cpu_speed_grade_rp2350` (+ `_serial`), `onerom_pio_diff_rp2350`, `onerom_pio_speed_grade_serial_rp2350`, `onerom_full_system_rp2350`, `onerom_serving_oracle_rp2350` (+ `_cpu`), `onerom_stress_cpu_rp2350`, `onerom_stress_pio_rp2350`. Targets the OneROM card (RP2350 PIO emulating a ROM chip). Real hardware oracles against the OneROM rig; HLDs live under `wrk_docs/2026.04.14 - HLD - OneROM *.md` and subsequent dates. Not for CI.
17. **RISC-V Hazard3 oracles** — `qemu_diff_riscv32` (QEMU RV32IMC-Zba-Zbb-Zbs diff for rp2350-emu's Hazard3 core), `riscv_probe_spike` (spike reference diff). Same role as the M33 oracles but for the parallel RISC-V path. `probe_csrrw_riscv32` is a one-shot Stage-6 probe (now resolved) — see its file header; not an oracle.
18. **`mmio_trace_rp2040`** / **`mmio_trace_rp2350`** — drive an emulator to emit the wire-format MMIO trace used by downstream consumers (see "Logging & Tracing" — don't duplicate this with `tracing`).
19. **`threaded_probe`** / **`threading_micro`** / **`pio_microbench`** / **`workload_study_rp2350`** — host-side microbenchmarks for the threaded runtime, PIO engine, and scheduler.

## High-Level Design Documents

Under `wrk_docs/`. HLDs are **phase-based and dated** (`YYYY.MM.DD - HLD - <topic> V<N>.md`). The original master HLD is `2026.04.12 - RP2350 Emulator HLD.md`, but subsequent phase HLDs (Phase 2 bus, Phase 3 interrupts, Phase 4 flash boot, Phase 5 dual-core SIO, Phase 6 PIO, Phase 7 coprocessors/FPU) supersede relevant sections. The workspace restructure itself lives in `2026.04.14 - HLD - mdpicoem Workspace Restructure.md`.

When working on a specific subsystem, **read the latest HLD for that phase** — not the master HLD. Later-dated versions (e.g. V5 over V2) supersede earlier drafts of the same phase.

Per-session notes live in `wrk_journals/`. Open technical debt is tracked in `tech_debt.md`.

## Logging & Tracing

Use the `tracing` crate for all diagnostic output in library crates. **Never add raw `eprintln!` or `println!` for debugging** — use the appropriate tracing level instead. See `wrk_docs/2026.04.16 - HLD - Workspace Tracing Infrastructure.md` for the full design.

### Level guide
- `error!` — emulation-breaking conditions (internal invariant violations)
- `warn!` — suspicious but non-fatal (unexpected clock config, repeated fault loops)
- `info!` — lifecycle events (init, firmware load, clock config, HardFault escalation)
- `debug!` — subsystem events (exception entry/return, clock reprogram, bus faults)
- `trace!` — high-frequency cold-path events (spinlock acquire/release)

Currently instrumented: exception entry/return, HardFault escalation, bus faults, clock tree recompute, emulator construction, spinlock ops. DMA, PIO, and peripheral subsystems will be instrumented incrementally as those areas are worked on.

### What NOT to trace
- **Instruction decode/execute hot path** (`execute.rs`, `decode.rs`, `execute_thumb32.rs`, `execute_fpu.rs`, `execute_wide.rs`) — use differential oracles, not logging. Even in debug builds, tracing here would be too slow.
- **The MMIO bus trace** (`emit_mmio_trace`) — this is a purpose-built wire-format trace with downstream consumers. Don't duplicate it with `tracing`.
- **Harness structured output** (PASS/FAIL, DIFF lines, progress counters) — this is program output, not logging. Keep as `println!`/`eprintln!`.
- **Test-only `eprintln!`** inside `#[test]` — test runner captures this already.

### Zero overhead guarantee
Binary crates (`picoem-harness`, `rp2350-emu-tui`, `rp2040-emu-tui`) set the `release_max_level_info` feature on `tracing`. Cargo's feature unification propagates that cap down to our libraries when they're built into those binaries, so `trace!()` and `debug!()` compile to **nothing** in `--release`. No performance impact on the hot path. Only `info!`/`error!` survive in release, and those are on cold paths only. The cap is deliberately *not* set on library crates' own dependency entries — doing so would leak through to external consumers (a `tracing` footgun the maintainers explicitly warn against).

### Runtime filtering
All binaries initialise a `tracing-subscriber` (harness: `harness_tracing_init()`, apps: inline). Default level is `warn`. Override with `RUST_LOG`:
```bash
RUST_LOG=rp2350_emu::bus=debug cargo run -p rp2350-emu-tui
RUST_LOG=rp2040_emu::core::exceptions=debug cargo run -p rp2040-emu-tui
RUST_LOG=debug cargo run -p picoem-harness --bin qemu_diff_m33
```

## Non-Obvious Conventions

- **Bank contention model (RP2040 Serial only, deprecated-in-place)**: in the Serial `step` path, core 0 runs first (recording which downstream port it touched in `core0_port`), then core 1 runs with `contention_check_active` — any same-port access adds +1 cycle. If core 1 is halted, contention checking is skipped entirely. This is why the `Bus` struct carries both flags. **Do not extend this model to rp2350-emu, and do not invest further on rp2040-emu**: the perf gap against real silicon (see `paced_bench_rp2*`) swamps the contention-accuracy gain by an order of magnitude, contention cycles are virtual (they don't consume host compute), and the Threaded `ExecutionModel` bypasses the serial-interleave path entirely (each core runs on its own worker thread, barrier-synchronised at the quantum boundary — no `contention_check_active` in that path). rp2350-emu `contention`/`stress` bench workloads are dual-core-compute-only by design. Rationale: `wrk_journals/2026.04.15 - JRN - Contention Modelling Declined.md`.
- **Pacer is portable.** It uses `rdtsc` on `x86_64` and `Instant` elsewhere. Do not gate serial apps or TUIs on `x86_64` just to use `Pacer`; only the threaded worker runtime is still platform-limited.
- **Clock tree is mutable at runtime.** Firmware can reprogram PLL/dividers via CLOCKS registers; the `ClockTree` cache on `Bus` is recomputed on each relevant register write. Don't hardcode frequencies.
- **Hardware harness needs real silicon.** `probe_diff_rp2350` / `probe_verify_rp2350` / `bank_conflict_test_rp2350` will not run in CI — they require a Pico debug probe attached to an RP2354 board. `probe_diff_rp2040` also needs a Pico debug probe, attached to a Pico V1 / RP2040 board, and is likewise not for CI. When both probes are attached to one host, disambiguate with `--probe <VID:PID:SERIAL>`.
- **Probe selector.** On a host with both an RP2354 probe and an RP2040 probe attached, `probe-rs auto_attach` picks the first enumerated probe regardless of target type, which fails half the time. Pass `--probe <VID:PID:SERIAL>` to `probe_diff_rp2040` / `probe_diff_rp2350` explicitly. `probe-rs list` shows the available serials.
- **Probe serial → DUT mapping** lives in [`docs/probe_serials.md`](docs/probe_serials.md). Use that table when a harness binary needs `--probe <VID:PID:SERIAL>`.
- **The `bin/` directory under `crates/picoem-harness/src/` is tracked intentionally** — don't re-add a broad `bin/` rule to `.gitignore`; it silently hides test binaries.
- **ROMs live under `roms/rp2350/` and `roms/rp2040/`.** Pre-restructure code referenced bare `roms/...`; post-restructure, paths are `roms/rp2350/<file>` (blinky, bootrom, LCD demo, benchmark firmware) and `roms/rp2040/<file>` (blinky + 16 KB bootrom generated by `roms/rp2040/gen_blinky.py`).
- **Windows-only: hung oracle process trees.** When a fuzz oracle child tree zombies and `taskkill` doesn't terminate it, see [`RUNBOOK.md`](RUNBOOK.md) for the `kill -9 <POSIX_PID>` recipe using `ps -W` PPID walking.
- **`debug_assert!` policy.** Use `debug_assert!` for invariants the type system cannot encode and that no caller can reach with bad input. **Public methods that take user input** (e.g. builder setters such as `EmulatorBuilder::step_quantum`) must validate with a typed `Result` or a saturating clamp — `debug_assert!` is elided in release builds and silently dangerous on those paths.

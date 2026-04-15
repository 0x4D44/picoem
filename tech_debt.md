# Technical Debt

Items discovered during development that need addressing in later phases.

## Cycle Timing — Phase 2 (Bus Fabric)

Measured on real RP2354 silicon via DWT CYCCNT (probe_diff --cycles).
Current emulator uses flat per-instruction cycle counts. These effects
require the bus fabric (HLD Phase 2) to model correctly.

### SRAM Bank Contention (+1 cycle on some load/store)

14 load/store tests show 3 cycles instead of the expected 2. All have
raw CYCCNT=7 (vs normal 6). Pattern: specific register/offset combinations
that likely cause the data access to hit the same SRAM bank as the
instruction fetch.

The RP2350 has 10 SRAM banks (SRAM0-7 striped, SRAM8-9 non-striped).
When two bus masters (I-bus fetch + D-bus data) access the same bank on
the same cycle, one stalls for 1 cycle. This is bus arbitration, not
instruction cost — must be modelled in the fabric, not execute().

Examples:
```
STR R7, [R6, #8]              HW=3  EMU=2  (raw=7)
LDR R2, [R1, #8]              HW=3  EMU=2  (raw=7)
LDRB R3, [R2, #10]            HW=3  EMU=2  (raw=7)
STR R0, [SP, #8]              HW=3  EMU=2  (raw=7)
```

### Backward Branch Pipeline Penalty

3 large backward branches show 6 cycles instead of 1. Small backward
branches (offset < ~256 bytes) show 1 cycle, same as forward branches.

The M33 prefetch buffer can serve forward branch targets (already fetched
or being fetched) but not far backward targets. A large backward branch
requires a full pipeline flush + refetch from the new address.

Threshold appears to be around 256-500 bytes backward. Need more data
points to determine the exact cutoff.

Examples:
```
B -500                         HW=6  EMU=1  (raw=10)
B -1000                        HW=6  EMU=1  (raw=10)
B -2048                        HW=6  EMU=1  (raw=10)
B -100                         HW=1  EMU=1  (raw=5, OK)
B -6                           HW=1  EMU=1  (raw=5, OK)
```

### PUSH Minimum Cost

PUSH with 2 registers shows 4 cycles (HW), but 1+N formula gives 3.
Single-register PUSH = 2 (correct: 1+1). Three-register PUSH = 4
(correct: 1+3). Eight-register PUSH = 9 (correct: 1+8).

The 2-register case suggests a minimum cost floor or non-linear
formula for small N. Possibly related to the store buffer or stack
pointer update timing. Need more data points across different register
counts to determine the exact formula.

Examples:
```
PUSH {R0, R1}                  HW=4  EMU=3  (1+2=3, but HW=4)
PUSH {R0, LR}                  HW=4  EMU=3
PUSH {R6, LR}                  HW=4  EMU=3
PUSH {R0}                      HW=2  EMU=2  (1+1=2, OK)
PUSH {R0, R1, LR}              HW=4  EMU=4  (1+3=4, OK)
PUSH {R0-R7}                   HW=9  EMU=9  (1+8=9, OK)
```

## Cycle Timing — Halt-Step Measurement Limitations

The DWT CYCCNT measurements via probe-rs halt-step include a constant
5-cycle debug overhead (calibrated out). This works for isolated
instruction cost but cannot capture:

- Pipeline overlap (I-bus/D-bus parallel access)
- Back-to-back forwarding between consecutive instructions
- Cache effects (XIP flash vs SRAM fetch latency)
- Multi-instruction timing interactions

For these, the firmware mailbox mode (HLD Phase B of Oracle Layer 3)
is needed — measures CYCCNT in a tight loop without debug overhead.

## Test Harness — Address-Space Dependent Tests

18 Thumb-16 tests removed from the QEMU differential harness because
they produce address-space-dependent results (different QEMU vs emulator
address spaces):

- 7 ADR tests (writes PC-relative address to register)
- 10 ADD Rd, SP, #imm tests (writes SP-relative address to register)
- 1 POP {PC} test (loads absolute address from memory into PC)

These are testable via probe_diff (same address space) but not via
qemu_diff. Could be restored with address-aware comparison logic in
the QEMU harness if needed.

## Core Correctness

### CPS bit-swap in mdrp2350 (matches obsolete LLD docs)

`crates/mdrp2350/src/core/execute.rs` implements the CPS encoding with bit 0 = I and bit 1 = F. ARMv6-M/v7-M ARM A6.7.38 specifies the reverse (bit 1 = I, bit 0 = F). Canonical assembler output (`CPSIE i` = 0xB662, `CPSID i` = 0xB672) is currently silent on PRIMASK. The LLD docs under `wrk_docs/` that claim the bits are swapped are wrong — they inherit the same error.

Fix: swap the bit check; update tests that used the wrong canonical encoding. Trace references:
- `crates/mdrp2350/src/core/execute.rs` — CPS decode site.
- `wrk_docs/` — any LLD section mentioning CPS bit ordering. Correct to match ARM ARM.

mdrp2040 Phase 4.A fixed the bug in its own code (2026-04-14).

### mdrp2350 banked SP staleness in `enter_exception`/`exit_exception`

mdrp2040 Phase 4.B uncovered (and fixed in its own tree) a banked-SP staleness hazard in the shared exception-entry/-exit pattern: `enter_exception` reads `self.regs.msp`/`psp` directly, but plain instructions (`SUB SP,#imm`, `ADD SP,#imm`, `PUSH`, `POP`) update `r[13]` without syncing back to the banked field. Handlers that allocate stack locals then return to unwind from a stale banked SP, corrupting the frame pointer. mdrp2350's `enter_exception`/`exit_exception` has the same shape and was not touched during Phase 4 per review scope. Fix: insert `sync_sp_to_banked()` at the top of both entry/exit and `sync_sp_from_banked()` after SP swaps, mirroring the mdrp2040 Phase 4.B apply-feedback change. Any Pico SDK handler stack-allocating locals will exhibit corruption today.

## Phase 5.A Simplifications (RP2040 bus)

These surfaced during Phase 5.A code review. The emulator compiles and Phase 5.A unit tests pass, but firmware exercising any of these paths will see incorrect behaviour. All are Phase 6+ work.

### RP2040 WFE/SEV not wired on M0+

`crates/mdrp2040/src/core/execute.rs` treats WFE and WFI as 1-cycle NOPs. `Emulator::step` clears `event_flag[0]` each step without a corresponding wait-state on core 0. Firmware using `__wfe`/SEV protocol will busy-loop rather than suspend. Needs a proper `wfe_waiting` flag per the mdrp2350 pattern (core suspends until a SEV, interrupt, or FIFO-rx event pending). Blocker before any multicore firmware with `__wfe()` idle loops (and any SDK `sev()`/`wfe()` helpers) can run correctly.

### RP2040 SIO divider 8-cycle latency not modelled

`crates/mdrp2040/src/bus/sio.rs` `DIV_CSR` reports `READY=1` immediately after a divider write. Real hardware requires 8 cycles for the DIV result to become available. Pico SDK `hw_divider_delay` uses inline-asm hard-coded NOPs rather than polling `CSR.READY`, so most SDK-using firmware is unaffected, but any firmware that busy-polls `CSR.READY` will read a stale result. Low priority — fix with a cycle counter on the divider state.

### RP2040 PLL LOCK always 1

`crates/mdrp2040/src/bus/clocks.rs` forces `PLL_SYS_CS[31]` (LOCK) to 1 on read so firmware wait-for-lock loops fall through on the first poll. If firmware writes `FBDIV_INT=0` and then polls LOCK, it will observe LOCK=1 but the derived `pll_output_hz` returns 0 (so the clock tree is 0 Hz). Partial mitigation: callers reading `sys_clk_hz` see the zero propagation. Proper modelling: LOCK=1 only when `pll_output_hz` > 0 (and/or only after a configured lock-delay). Low priority.

### RP2040 core 1 SDK handshake not parsed

`crates/mdrp2040/src/lib.rs` `maybe_wake_core1` wakes core 1 at its current reset-vector PC on any non-zero FIFO push by core 0. The real Pico SDK `multicore_launch_core1` sends a six-word handshake (0, 0, 1, VTOR, SP, entry) which the bootrom on core 1 parses before jumping. Our emulator ignores VTOR/SP/entry — any real SDK-based multicore firmware will either fault (bad SP) or land at the wrong entry. Fix in Phase 6: parse the six-word handshake in the SIO FIFO path and drive `core 1 wake + SP/PC/VTOR` from the handshake words.

### RP2040 per-instruction dual-core cadence

`crates/mdrp2040/src/lib.rs` `Emulator::step` runs one instruction per core per call — unlike mdrp2350's quantum (N-instructions-per-quantum) scheduler. `update_gpio()` and `wake_checks()` also run per-instruction, which adds measurable per-Hz overhead and makes paced-throughput numbers worse than mdrp2350. Should converge to the quantum model before Phase 7 app work so `paced_bench` numbers are comparable across the two chips.

**Status (2026-04-14, post-Phase-7):** not fixed. Phase 7 landed with
`Emulator::step` still running one instruction per core per call. The
`step_quantum` field on the builder is assigned but unused by `step()`,
so configuring it is a no-op. Firmware correctness is unaffected (the
blinky smoke test runs end-to-end in ~290ms, 7x headroom vs the 2s
budget), but `paced_bench` for mdrp2040 is not directly comparable to
mdrp2350's quantum-mode numbers until convergence happens. Still a real
improvement to make, just not a blocker for firmware correctness.

**Resolved (2026-04-14):** `Emulator::step` now drains both cores up to
`step_quantum` master cycles per call and ticks PIO / GPIO / wake-checks
once at quantum end, mirroring `mdrp2350::Emulator::step`. Per-instruction
core-0/core-1 interleaving (and `maybe_wake_core1`) preserved so bank
contention timing and intra-quantum FIFO wakes still fire. Tests that
need single-instruction granularity opt in via
`EmulatorBuilder::new(Config::default()).step_quantum(1).build()`. See
`wrk_docs/2026.04.14 - HLD - mdrp2040 Quantum Step.md` (v1.2.0).

### RP2040 pacer MHz panel undercounts consumed cycles

`crates/mdrp2040app/src/sim.rs` calls `emu.run(pacer.quantum_cycles())`
and `Pacer` reports MHz from cycles *asked for*. `run()` overshoots by
up to `step_quantum - 1` cycles (quantum-step landed in
`wrk_docs/2026.04.14 - HLD - mdrp2040 Quantum Step.md` v1.2.0), so the
app's MHz panel systematically undercounts by up to ~22% at default
settings — surfaced during the punchlist review (see
`wrk_docs/2026.04.14 - HLD - mdrp2040 Quantum Step Punchlist.md`).
Fix requires a `Pacer` API extension to feed consumed cycles back
(replace `begin_quantum`/`end_quantum` with a form that takes the
actual cycle count from `emu.run`'s return). Low priority — firmware
runs correctly; only the displayed MHz figure is wrong.

### RP2040 SIO divider 2-read dirty clear heuristic

`crates/mdrp2040/src/bus/sio.rs` clears the divider `dirty` flag after exactly two result reads. Real hardware clears `dirty` on any result read (per-register). The two-read heuristic happens to match the canonical `__aeabi_idivmod` pattern (quotient + remainder read in pairs), but misbehaves for firmware that reads only one result (e.g., modulo-only code paths leave `dirty` set until the next write). Low priority — fix by clearing on each read of `QUOTIENT`/`REMAINDER`.

### PIO not gated on RESETS bit

Both mdrp2350 and mdrp2040 tick their PIO blocks unconditionally each
step, regardless of the RESETS register state. Real hardware holds the
PIO block inert while its RESETS bit is asserted. In practice an SM
disabled before RESETS is de-asserted stays disabled anyway, so this is
a safe simplification — but firmware that expects a mid-execution SM to
freeze on RESETS assert will diverge. mdrp2350 carries the same
behaviour.

## Phase 6 Simplifications (Harness split)

These surfaced during Phase 6 (the `mdpicoem-harness` binary split into
chip-suffixed runners). The workspace compiles and both `qemu_diff_m33`
and `qemu_diff_m0plus` oracles pass their smoke runs, but the following
corners are deferred to later phases.

### `probe_diff_rp2040` is a stub

`crates/mdpicoem-harness/src/bin/probe_diff_rp2040.rs` is a placeholder
that exits 2 with a rationale; no probe-rs wiring exists. The lab rig
only carries an RP2354, so there is no hardware to diff against. Future
work: mirror `probe_diff_rp2350` with a chip-pack of `"RP2040"` and
extract the `is_m0plus_safe` filter out of `qemu_diff_m0plus` into
`mdpicoem-harness::lib` so both runners can share it.

### QEMU M0+ oracle uses `cortex-m0`, not `cortex-m0plus`

QEMU 10.2 does not expose a `cortex-m0plus` CPU model, so
`qemu_diff_m0plus` pins the oracle CPU to `cortex-m0`. The M0+ is a
strict ISA superset of the M0 for the Thumb-16 / Thumb-32 subset under
test (MUL cycle counts differ, but the harness does not compare cycle
counts), so the M0 reference is safe for architectural (register /
memory / xPSR) diffs. Switch to `cortex-m0plus` once a future QEMU
release exposes it.

## Thumb-32 Test Generators

Three Thumb-32 generator functions are stubbed out in lib.rs
(commented out in generate_all):

```rust
// all.extend(thumb32_gen::gen_t32_dp_mod_imm());
// all.extend(thumb32_gen::gen_t32_load_store_single());
// all.extend(thumb32_gen::gen_t32_multiply_divide());
```

Uncomment and implement as Thumb-32 instruction classes are completed
in the emulator.

## Phase 6/7 Residuals

These surfaced during the final conformance pass after Phase 7 shipped.
None are firmware-correctness blockers; they are oracle-coverage and
calibration gaps.

### MULS cycle-count hardcode on mdrp2040 (not silicon-calibrated)

`crates/mdrp2040/src/core/execute.rs` (~line 339) returns `1` cycle for
`MULS`. Real Cortex-M0+ ships in two multiplier variants: a single-cycle
"fast" multiplier (the RP2040's choice per the datasheet) and a
32-cycle multi-cycle variant. `1` is defensible for the Pico's M0+ r0p1
implementation, but the number is hardcoded with **no silicon
calibration**. The `isa.rs` panel in `mdrp2040app` consumes this as
ground truth (`MULS=1`). Not currently oracle-validated — the
QEMU `cortex-m0` oracle does not compare cycle counts, and the probe
oracle for RP2040 is a stub. Low priority — fix when a Pico probe
harness is available to measure the real cycle count. Same caveat
applies to the other hardcoded M0+ cycle counts in the same file
(`LDR`, `LDM`, `B`, `BL`, `ADDS`).

### Thumb-32 subset not QEMU-differentially validated

`qemu_diff_m0plus` uses the `is_m0plus_safe` filter in
`crates/mdpicoem-harness/src/lib.rs` which rejects every `TestCase` with
`hw1.is_some()` — i.e., all 32-bit-wide Thumb encodings. Phase 4.B
shipped ~700 lines of Thumb-32 executor code (`execute_wide.rs`) for
`BL`, `MRS`, `MSR`, `DSB`, `DMB`, `ISB` with **unit-test-only
coverage**; the QEMU differential oracle never exercises these paths.

Fix: extend the fuzz generator (or add a new M0+-specific wide-subset
generator) to produce valid `BL`/`MRS`/`MSR`/`DSB`/`DMB`/`ISB` test
cases, then relax `is_m0plus_safe` to allow them through. Medium
priority — unit tests cover the known happy paths but a
differential oracle would catch decode/flag/SYSm corner cases that
unit tests tend to miss. This entry supersedes the older "Thumb-32
Test Generators" section above for the M0+-specific subset; the
mdrp2350 T32 generator work remains pending separately.

### Exception entry/exit not differentially validated

`qemu_diff_m33` and `qemu_diff_m0plus` single-step individual
instructions; neither fuzzer exercises asynchronous exception entry
(external interrupt, SysTick tick) or any of its corners. Exception
entry is the fattest remaining code path in the emulator that is
unit-test-only — covered by targeted tests in `mdrp2350/src/tests.rs`
and `mdrp2040/src/tests.rs` but not fuzzed against any reference.

For M33 in particular, the combinatorial surface is large: stacking
(8 regs plus FPU lazy save via FPCCR.LSPACT), SP switching
(MSP/PSP × S/NS), xPSR/EPSR update, EXC_RETURN encoding, security-
state transitions, stack-limit (MSPLIM/PSPLIM) checks, tail-chaining,
late-arriving preemption. Unit tests cover known corners but not the
cross-product.

Fix (three-stage plan):
1. Add `--workload isr` to `paced_bench_*` — exercises the path;
   bench-level regression signal. Cheap first step.
2. Add targeted unit tests for the known-hairy corners (FPU lazy
   save on entry, stack-limit fault, security transition).
3. Add a dedicated ISR diff fuzzer (`qemu_diff_isr_*`) that treats
   entry as an atomic unit — compare state *after* entry completes,
   not cycle-by-cycle.

**Caveat for M33:** QEMU's M33 NVIC/SCB modelling is believed to be
less mature than its integer-ops support (needs confirmation when we
get there); if true, a meaningful fraction of findings from a
QEMU-based ISR fuzzer will be QEMU bugs, not ours. For M33,
`probe_diff_rp2350` against real RP2354 silicon is likely the
higher-yield ISR oracle once the infrastructure is in place. For
M0+, QEMU is probably fine.

Medium priority. The path is correct for current firmware (unit
tests gate it) but has no regression safety net at the breadth a
fuzzer provides.

## PicoGUS Integration — Stage 1 follow-ups

Surfaced by the devils-advocate review of Stage 1 (XIP flash in
mdrp2040). None block Stage 2; logged here so they don't get lost.

### `mdrp2040app` CLI does not expose `--flash`

The PicoGUS HLD Stage 1 acceptance criterion reads
`cargo run -p mdrp2040app -- --flash roms/rp2040/blinky.bin`. The
functionality works — `mdrp2040app` loads its positional argument via
`Emulator::load_flash` — but there is no `--flash` named flag. Either
fix the HLD wording to match the positional-argument invocation, or
add proper flag parsing to `mdrp2040app`. Low priority (cosmetic /
docs drift).

### `Memory::load_flash` branching on `xip.is_empty()` is a footgun

`crates/mdpicoem-common/src/memory.rs` branches on `self.xip.is_empty()`
to choose resize-vs-clamp semantics. On the mdrp2350 path
(`with_sizes(rom, sram)` leaves `xip` empty), the first `load_flash`
call resizes the buffer and subsequent calls fall into the clamp
branch — so a follow-up `load_flash` with a larger image silently
drops its tail. No current call site hits this, but it will bite when
someone first reloads mdrp2350 flash with a different size. Fix:
either always resize, or split into `load_flash_clamped` (mdrp2040
fixed-window) and `load_flash_resize` (mdrp2350 dynamic). Medium
priority — latent bug, not blocking.

### XIP reads past the loaded image don't mirror within the 2 MB alias

Each of the four RP2040 XIP aliases covers a 16 MB address range with
a 2 MB physical flash — real hardware mirrors the image every 2 MB
inside each alias. Our implementation returns 0 for reads in
`0x10200000..0x11000000` (and the equivalent gaps in the other three
aliases). Low priority — firmware that addresses past 2 MB is already
buggy; current tests don't depend on the mirroring. One-line fix if
we care: fold offset modulo `FLASH_SIZE` before indexing.

## PicoGUS Integration — Stage 2 follow-ups

### PSRAM PIO-integration tests cover only 1 edge/quantum

`pio_integration::pio_driven_write_then_read_round_trip` and
`pio_driven_fast_read_returns_written_bytes` use `step_quantum=4` with
SCK toggling every 2 sysclks — one rising edge per `emu.step()`. That
means the test would pass even if `update_gpio()` ran twice per step
instead of `consumed` times. Add a stress test at `step_quantum=64`
with PIO toggling SCK every sysclk (32 rising edges per step) to
actually prove the interleave fix catches every edge. Without this,
a future regression to a narrower fast-path predicate would not be
detected. Medium priority (insurance for Stage 6 firmware boot).

### Enable-then-disable mid-quantum drops PSRAM edges

`Emulator::step` checks `pio_idle` (`!any_sm_enabled`) at the *end*
of the core loop. If a CPU instruction enables an SM at cycle C1 and
another instruction disables it at C2 (both within the same quantum),
the final state is "disabled" → `pio_idle=true` → fast-path runs
`tick_pio(consumed)` which short-circuits → edges between C1..C2 are
dropped. Unrealistic in firmware (SM-enable/disable pairs in a 64-
cycle window is pathological) but a real semantic gap of the fast
path. Low priority. Fix: OR the pre-loop enabled mask into the
predicate.

## PicoGUS Integration — Stage 3 follow-ups (build-time only)

Neither item affects first-party Rust code. Both fire only when
someone actually applies the DOSBox-X patch and builds it.

### GUS DMA-channel traffic not captured by the tap

The patch hooks `read_gus` / `write_gus`, which capture direct GUS
register I/O but *not* DMA transfers. Real GUS ("DRAM DMA" via the
GF1 register set) pumps patch samples from DOS RAM into GUS DRAM via
ISA DMA cycles. Depending on DOSBox-X's DMA implementation, these
may flow through `write_gus` (captured) or bypass via
`GUS::DMA_Callback` (NOT captured). If bypassed, no samples ever
reach PicoGUS's PSRAM in the replay and Stage 6 produces silence.

Mitigation options:
1. Extend the patch to hook `GUS::DMA_Callback` emitting synthetic
   `write8` records at the GUS DRAM data port (0x247).
2. Pre-load PSRAM from a known patch-bank dump before replay starts
   (requires parsing GUS .pat or .ult formats).
3. Use a MIDI demo that relies only on built-in GM patches baked
   into firmware (may not exist for PicoGUS).

Must resolve before Stage 6 demo. Medium priority.

### Tap reentrant guard is not exception-safe

The `picogus_tap_reentrant` static bool in the DOSBox-X patch is
set/cleared around the `iolen==2` recursion in `read_gus`. If a
C++ exception unwinds through `read_gus` (DOSBox-X uses `E_Exit` in
some paths), the flag stays `true` and all subsequent tap entries
silently skip. Fix: RAII guard struct (ctor sets, dtor clears).
Five-line change. Low priority — unlikely in steady-state MIDI
playback.

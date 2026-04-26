# Technical Debt

Items discovered during development that need addressing in later phases.

## SpinBarrier watchdog tuning follow-ups (2026-04-24, Stage 5)

Stage 5 added a wall-clock watchdog to `SpinBarrier::wait` in
`crates/mdpicoem-common/src/threaded/barrier.rs` with a flat 5 s
default deadline. Several refinements deferred:

1. **Deadline formula.** HLD §6.6 prescribed
   `quantum_cycles × 256 / min_clock_hz + 5 s`. Stage 5 shipped a
   flat 5 s for simplicity. Edge case: a test-constructed 1 kHz
   clock with `step_quantum = 1_000_000` → one quantum = 1000 s wall
   time, watchdog misfires. No such config exists today; add the
   formula when a user hits it.
2. **`threading_micro` per-wait overhead ~6% (HLD target was <2%).**
   Source: single `Instant::now()` at `wait()` entry (~25 ns per
   QPC call on Windows). Fast-path fix: lazy-capture on first spin-
   stride check instead of wait-entry. Most spins complete before
   SPIN_BUDGET=512 iterations, so `Instant::now()` would rarely be
   called. Measured vs documented-target mismatch acknowledged;
   correctness > 4 % perf for v1.
3. **`WATCHDOG_STRIDE = 1024` > `SPIN_BUDGET = 512`.** The spin-path
   watchdog check `i & (WATCHDOG_STRIDE-1) == 0` never triggers
   because SPIN_BUDGET elapses first and we transition to condvar
   park. Condvar path does check the deadline, so correctness is
   fine; spin-path check is dead code. Either lower stride to 256
   or delete the spin-path check and document the condvar-path-only
   semantics.
4. **`EmulatorError::BarrierTimeout { which }` attributes to first
   observer, not culprit.** The barrier cannot identify the
   never-arriving worker (no arrival signal). `which` is currently
   hardcoded `WorkerName::Coord` at both chip coordinators. Misleading
   for a reader. Fix: either drop `which`, or plumb `WorkerName` into
   `SpinBarrier::wait` so the first tripper records itself.
5. **`EmulatorError::BarrierTimeout` cfg-gate asymmetry.** mdrp2350
   cfg-gates the variant on `all(feature = "threading", x86_64,
   windows)`; mdrp2040 defines it unconditionally (field
   `timeout_info` IS cfg-gated). Align.

None block Stage 6; all are polish. Low priority — file a cleanup
pass when touching `barrier.rs` next.

## Emulator direct-field access is Serial-only but not type-enforced (2026-04-24)

**Current state:** `mdrp2350::Emulator` and `mdrp2040::Emulator`
expose `pub cores`, `pub bus: Bus`, and (on mdrp2350) `pub clock:
Clock`. After a Threaded run (`run_quantum` / `run` on an
`ExecutionModel::Threaded` builder), the dual-execution HLD V1 Stage
1b / 3b.4 `promote_to_threaded` path moves the live state into
`self.threaded` and replaces the flat fields with zero-cost
placeholders. A `pub(crate) bus_is_placeholder` flag plus
`Self::assert_not_placeholder()` debug_asserts fire a clear panic
when a caller reaches through a guarded API in debug builds. Release
builds elide the assertion entirely.

Guarded accessors (Stage 3b.4 added the mutators):

- Read-side (both crates): `core`, `core_mut`, `peek`, `gpio_read`,
  `gpio_read_all`, `cycles`, `mmio_read32` (+ mdrp2350-only:
  `core_riscv`, `core_riscv_mut`, `core_counters`, `reset_counters`).
- Write-side / construction-time (mdrp2040): `reset`, `load_image`,
  `load_bootrom`, `load_flash`, `direct_boot_from_flash`,
  `gpio_write`, `halt_core1`, `wake_core1`, `drain_uart0_tx_log`,
  `mmio_write32`, `poke`.

**Known escape:** raw field access — `emu.bus.sio.gpio_out`,
`emu.cores[0].regs.r[0] = 42`, `emu.clock.cycles` — bypasses the
guarded accessors and silently reads/writes the dead placeholder
state with no diagnostic in debug or release. Grep of the workspace
as of this write confirms no live caller does this after
`promote_to_threaded`, but the footgun is primed for any future
caller.

**Recommended follow-up:** migrate `cores` / `bus` / `clock` to
`pub(crate)` with typed accessors in a dedicated stage. Estimated
cost: ~1287 call-site updates across roughly 40 files. The debug
guard today only covers the typed API surface, so the compiler will
not surface raw-field regressions.

**Risk if deferred:** new code added to `ExecutionModel::Threaded`
paths may silently corrupt state by poking the placeholder fields
instead of the live worker state. The current guards catch every
typed-API caller, which is the majority of the workspace, but not
direct `.bus.` / `.cores[` / `.clock.` reaches.

**Status:** guards in place; wider migration deferred to a future
stage of the dual-execution rollout.

## Stage 3b.4: RP2040 Threaded — PSRAM MISO not modelled (2026-04-24)

`mdrp2040::threaded::emulator::ThreadedEmulator::from_emulator` emits
a `tracing::warn!` when the source `Bus::psram` is attached and drops
the device. The threaded coordinator has no sub-quantum SPI edge
model — SIO / PIO writes publish their merged pad state to
`shared.gpio_out`/`gpio_oe` at the end of each quantum, so a PSRAM
device driven by PIO-toggled SCK would see only the quantum-end pin
snapshot. That is always wrong for PSRAM reads (MISO feedback into
GPIO0 would miss every SCK rising edge inside the quantum).

**Mitigation:** harnesses using PSRAM — notably `picogus_diff_rp2040`
— must build with `ExecutionModel::Serial`. The Serial path still
exercises the per-cycle `update_gpio` interleave that feeds every
pin edge to the PSRAM.

**Fix (Stage 4+):** either model PSRAM as a coordinator-resident
device ticked per cycle inside the merge loop, or split the PSRAM
timing onto a dedicated worker with its own quantum cadence. Both
options need dual-model oracle coverage because the current Serial
behaviour is the ground truth.

## Stage 3b.4: RP2040 Threaded — coordinator peripheral ticks limited to TIMER (2026-04-24)

`mdrp2040::threaded::emulator::coordinator_worker_body` polls
`TimerRegs::poll_alarms` each quantum and routes bits 0..3 onto
`CoreAtomics.irq_pending` for both cores. Other peripherals with
per-cycle tick methods — UART0/UART1, SPI0/SPI1, I2C0/I2C1, ADC,
PWM, DMA — are **not** advanced on the threaded path. Their state
lives in `SharedState.peripherals.legacy` (HashMap) from Stage 3b.3
and is only read/written through MMIO. Firmware that depends on
TX shift-register drains (UART flush IRQs), ADC conversion timing,
PWM wrap IRQs, or DMA transfer completion will diverge between
models.

**Mitigation:** Stage 4 dual-model tests that hit those peripherals
should run Serial-only until the corresponding advance path lands.
Firmware-level harnesses (`paced_bench_rp2040`, overnight fuzz)
should pick `--model serial` for peripheral-heavy workloads.

**Fix (Stage 4):** promote each typed peripheral into
`SharedState.peripherals.<name>: Mutex<..State>` (mirroring the
existing TIMER) and tick them in the coordinator loop. Audit each
peripheral's `tick` method for Send-safety under Mutex first — some
reach back into the Bus for IRQ OR-in; that coupling must be
factored out before the threaded wiring is safe.

## Vendored probe-rs fork for issue #3872 (Track A workaround)

**Context:** `third_party/probe-rs-0.31.0-mdrp-patched/` carries a single
surgical patch to `select_ap_and_ap_bank` in
`src/architecture/arm/communication_interface.rs` to recover from a
first-DPIDR-read version glitch that trips probe-rs's `unreachable!()`
panic on RP2354 silicon oracles (`silicon_periph_diff_rp2350`,
`silicon_isr_diff_rp2350`, `silicon_dualcore_diff_rp2350`,
`bank_conflict_test_rp2350`, `probe_verify_rp2350`, and
`test_silicon` when any of the above runs). Routed via
`[patch.crates-io]` in the workspace `Cargo.toml`. See
`wrk_docs/2026.04.21 - HLD - Track A Probe-rs Attach Fix.md` and
`RUNBOOK.md` → "probe-rs patch notes".

**Upstream issue:** [probe-rs/probe-rs#3872](https://github.com/probe-rs/probe-rs/issues/3872)
(open; companion report #3257 open since April 2025). Master still
contains the `unreachable!()` at the equivalent location as of
2026-04-21. A PR draft targeting #3872 lives at
`wrk_docs/2026.04.21 - DRAFT - probe-rs PR body.md`.

**Risk:** every probe-rs bump needs the patch re-ported until upstream
merges a fix; our `Cargo.lock` also unpins the crates.io checksum for
`probe-rs` for as long as the patch is live. The sentinel WARN
(`ApV2 access on DPv1 cache; upgrading to DPv3 (mdrp-patched
workaround)`) is expected at most once per session; if it fires
repeatedly, the patch is masking a deeper DP bug and should be
escalated rather than ignored.

**Fix:** drop the vendored fork and the `[patch.crates-io]` entry once
upstream ships a release we can bump to. Smoke-test the silicon
oracle catalogue on the bump commit to confirm the panic no longer
reproduces.

**Status:** live workaround; tracked until upstream resolves #3872.

## RP2350 DMA IRQ2/IRQ3 routing not modelled

**Context:** Residual C.2.1 (`wrk_docs/2026.04.17 - HLD - Residual C.2.1 DMA
Timer Paced Fix.md`) shifted the DMA global-register block to the correct
RP2350 offsets and added `INTE2/INTF2/INTS2` and `INTE3/INTF3/INTS3` as
read/write storage in `crates/mdrp2350/src/dma.rs`. Reads return the same
pattern as IRQ0/IRQ1 (`(intr | intfN) & inteN`) so firmware read-modify-write
sequences round-trip, but the controller does **not** fan these out to the
NVIC lines `IRQ_DMA_IRQ_2` (12) / `IRQ_DMA_IRQ_3` (13) — `Dma::route_irqs`
still only dispatches IRQ0 and IRQ1.

**Risk:** any firmware that enables `INTE2` / `INTE3` will silently never
observe the corresponding NVIC pend. No V1 corpus scenario uses IRQ2/IRQ3
(`qemu_diff_m33`, silicon oracles, and OneROM glue all route via IRQ0), so
there is no current test signal. The existing `dma_timer_paced_transfer` /
sibling DMA silicon scenarios exercise the storage-side (reads/writes
round-trip) but nothing verifies the NVIC fan-out.

**Fix sketch:** extend `Dma::route_irqs` to pend `IRQ_DMA_IRQ_2` when
`(intr | intf2) & inte2 != 0` and `IRQ_DMA_IRQ_3` when
`(intr | intf3) & inte3 != 0`, mirroring the IRQ0/IRQ1 pattern.  Add a
unit test paralleling `irq_routing_dma_irq0` / `irq_routing_dma_irq1`.

**Status:** deferred. The storage-only stance keeps the blast radius
bounded for Residual C.2.1 (scope: pacing-timer offset only).  Land this
alongside the first scenario that actually wires IRQ2/IRQ3.

## Silicon oracle scenario `pwm_fractional_div` — self-gated sled design
fails silicon verify (Residual A.2.3 incomplete)

**Context:** Residual A.2.3 attempted to close `pwm_fractional_div` on RP2354
silicon (HLD: `wrk_docs/2026.04.17 - HLD - Residual A.2.3 PWM Fractional
Divider Fix.md`). The HLD's diagnosis is sound: the emulator's fractional
divider formula is correct for the declared scenario window
(152 sysclks / divisor 2.0 = 76 = 0x4C, unit test
`fractional_div_integer_2_per_cycle_dispatch_matches_bulk` pins this), and
the divergence is a scenario-design defect (measurement-window asymmetry
between silicon's DAP-open window and emulator's `actual_sysclks` window).

**Proposed fix (attempted, reverted):** custom sled that flips `CSR_EN=1`
as its first instruction and `CSR_EN=0` before BKPT, plus observe mask
widened from broken `0x64` (non-contiguous) to `0xFFFF`. Cross-executed
correctly on emulator (post-sled CTR = 76). **Silicon FAILs**:
`HW=0x0000FE82 EMU=0x0000004C` (xor=0xFECE). HW=65154 implies silicon's
PWM-enabled window is ~130k sysclks, not 152 — the self-gated sled is
not actually stopping the counter on silicon.

**Hypotheses not yet disproven** (needs empirical silicon probing):
1. Silicon's `CSR_EN=0` write via Thumb STR doesn't propagate before BKPT
   halts the core (unlikely — APB writes complete before next-instr retire).
2. Silicon holds PWM-enabled state across the `gate_peripheral_hw` →
   CTR readback path that occurs after BKPT, with the DAP-readback latency
   contributing to the high HW count.
3. Split-storage tech-debt in emulator (Table 1137 says CSR_EN / EN are
   one physical bit; emulator stores them as two independent fields)
   interacts with sled ordering in a way that the emulator's `AND` model
   masks but silicon exposes.
4. Something in the scenario runner's post-halt sequence re-enables PWM
   (unlikely — no gate_peripheral_hw PWM branch exists; setup table no
   longer writes enable).

**Recommended next steps:**
1. Add a diagnostic scenario variant that reads `PWM_SLICE0_CSR` and
   `PWM_EN_OFFSET` post-BKPT — confirms whether enable bits are actually
   0 at observe time.
2. Consider adding a PWM branch to `gate_peripheral_hw` that writes
   `CSR_EN=0` to all active slices before readback — belt-and-braces.
3. Audit what silicon does between BKPT-reached and CTR-read. If
   non-trivial PWM ticking happens there, the fix needs to be at the
   runner level, not the sled.

**Status:** Residual A.2.3 open pending follow-up wave. Residual leaves
the scenario as it was before the wave (setup-time `CSR_EN=1` + default
countdown sled + broken `0x64` observe mask + HW=0x64 EMU=0x44 coincidence-
PASS-style fail). Emulator fractional-divider formula from commit `5eac6a1`
remains correct and protected by existing unit tests; no regression risk.

## HLD/LLD alignment

### Test-Oracles HLD V4 §4 Phase 2 — QEMU invocation deviation

Core HLD (`wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support V6.md`
§6 P2.5) pinned `-kernel <bin>`. Phase 2 LLD
(`wrk_docs/2026.04.17 - LLD - QEMU Diff RISC-V V1.md` §2) empirically found
`-kernel` is rejected with `-machine none` ("The -kernel parameter is not
supported (use the generic 'loader' device instead)") on QEMU 10.2. LLD uses
`-device loader,file=<bin>,addr=0x20000000,cpu-num=0` instead. Core HLD should
be amended or superseded with a V7 noting this, or the LLD's resolution should
be folded back. Not blocking.

Owner: whoever next edits the test-oracles HLD.

## RISC-V QEMU oracle — Stage 6 residuals (2026-04-18, superseding prior entry)

**The previous entry claimed csrrw silently no-ops on QEMU virt rv32. That was
wrong** — the actual fault was that QEMU's `-machine virt` maps `VIRT_FLASH` at
`0x2000_0000` (the address the oracle originally used to match the RP2350's
native SRAM base). CFI flash accepts GDB `M`-packet writes (debugger bypass)
but silently drops CPU `sw` instructions that don't do the CFI unlock dance.
Every "csrrw no-op" observation was actually `csrrw` working fine then the
subsequent `sw t1, 0(gp)` spill dropping the result into the flash bit bucket.
Proof: a minimal `csrrw mtvec, t0; csrrs t1, mtvec, x0; sw t1, off(gp);
ebreak` probe showed `misa = 0x00000000`, which is impossible on a running
rv32 CPU (misa is hardwired). Moving the same probe to `0x8000_0000` (virt
DRAM) immediately returned misa = 0x40141185 and showed csrrw taking effect
on mscratch / mtvec / mepc. Diagnostic tool:
`crates/mdpicoem-harness/src/bin/probe_csrrw_riscv32.rs`.

**Fix landed**: test-mode alias in `mdrp2350::bus::canon_oracle_addr` that
maps `0x8xxx_xxxx` → the existing SRAM backing. All oracle addresses moved
to `0x8000_0000`-based so QEMU virt's DRAM and the emulator run at the same
absolute PC. Stage-6 class filters (`Rv32iBranch`, `CsrSideEffect`, `Zicsr`,
`sc_w*`, etc.) REMOVED — they were hiding this root cause, not solving it.

**Genuine residuals now visible** (these were hidden before because the bug
masked them as "all zeros == all zeros"):

1. **Misaligned mem tests** (11 edge cases + ~4% of mem fuzz at seed 42).
   QEMU virt rv32 handles misaligned accesses transparently; Hazard3 traps
   with mcause=4/6 per HLD §4.5. **This is HLD §11-declared silicon-only
   scope** — the QEMU oracle cannot validate these, they're the silicon
   oracle's job. Consider adding a `Platform::SiliconOnly` marker on the
   `RiscvTestCase` so the QEMU runner skips them cleanly rather than
   reporting noisy diffs.

2. **Far branches / JAL with out-of-RAM targets** (~8 edge cases).
   Branch/jump targets land outside virt's DRAM region. QEMU reads 0 from
   unmapped → illegal-inst trap, mcause=2. Emulator reads 0 with bus_fault
   → access-fault trap, mcause=1. Both implementations are spec-legal
   (different trap priority interpretation); neither is wrong.

3. **sc.w without preceding lr.w**. `sc_w_plain` / `sc_w_aqrl` diff:
   QEMU=0 (success) vs emu=1 (failure). Emu is spec-correct — a `sc.w`
   with no reservation must fail. QEMU's generic rv32 appears to be
   lenient here. Real QEMU quirk, not an emulator bug.

4. **`rvc_c_jr_x1` / `c_jalr_x1`**. Jump to x1=0 → PC=0 → unmapped fetch
   → same unmapped-fetch divergence as (2). Not a core bug.

5. **Two ALU fuzz divergences at seed 42** (`fuzz_alu_107`: x26 low-byte
   `0x6b` vs `0xeb`, `fuzz_alu_129`: x19 `0x80` vs `0x00`). Previously
   listed as residuals; STILL present after the fix. Real emulator bugs
   worth investigating — the fix just uncovered them, didn't cause them.

6. **Handful of rv32m multiply/div and rv32a atomic fuzz divergences**
   at seed 42 (~5 failures per 100 tests). Real emulator bugs in
   overflow/corner-case semantics.

Owner for (1): QEMU oracle maintainer (annotate test cases; follow HLD §11).
Owner for (2), (3), (4): documentation (platform-divergence notes in the
LLD). Owner for (5), (6): Hazard3 core developers.

## Corpus reproducibility caveat

First-build binary SHA256s recorded during Phase 0 corpus pinning are NOT
byte-reproducible without `SOURCE_DATE_EPOCH`, `-Wl,--build-id=none`, and
`-no-canonical-prefixes`. V1 treats the SHA as an artefact identifier, not
a reproducibility guarantee. Tracked per V7 HLD §3.

## Resolved

### PIO side-set drives pad_oe without PINDIRS — Resolved (2026-04-15)

Fixed per `wrk_docs/2026.04.15 - HLD - PIO Side-Set Pad OE.md` (Option A):
dropped the `oe |= positioned_mask` in `PioBlock::merge_pin_outputs`'s
value-drive branch. `shared_pin_dirs` (populated by SET/OUT/MOV PINDIRS)
now solely owns pad_oe for side-set pins, matching RP2350 §11.3.2.3.
New unit tests T1–T4 in `crates/mdpicoem-common/src/pio/mod.rs` pin the
correct behaviour.

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

mdrp2040 Phase 4.B uncovered (and fixed in its own tree) a banked-SP staleness hazard in the shared exception-entry/-exit pattern: `enter_exception` reads `self.regs.msp`/`psp` directly, but plain instructions (`SUB SP,#imm`, `ADD SP,#imm`, `PUSH`, `POP`) update `r[13]` without syncing back to the banked field. Handlers that allocate stack locals then return to unwind from a stale banked SP, corrupting the frame pointer.

**Resolved (2026-04-16):** `sync_sp_to_banked()` inserted at the top of both `enter_exception` (after lockup check) and `exit_exception` (before unstack address read) in `crates/mdrp2350/src/core/exceptions.rs`, mirroring the mdrp2040 Phase 4.B fix. Unit test `test_pendsv_stacks_at_post_sub_sp_not_stale_banked_msp` confirms the frame is stacked at the correct post-SUB address.

## Phase 5.A Simplifications (RP2040 bus)

These surfaced during Phase 5.A code review. The emulator compiles and Phase 5.A unit tests pass, but firmware exercising any of these paths will see incorrect behaviour. All are Phase 6+ work.

### RP2040 WFE/SEV not wired on M0+

`crates/mdrp2040/src/core/execute.rs` treats WFE and WFI as 1-cycle NOPs. `Emulator::step` clears `event_flag[0]` each step without a corresponding wait-state on core 0. Firmware using `__wfe`/SEV protocol will busy-loop rather than suspend. Needs a proper `wfe_waiting` flag per the mdrp2350 pattern (core suspends until a SEV, interrupt, or FIFO-rx event pending). Blocker before any multicore firmware with `__wfe()` idle loops (and any SDK `sev()`/`wfe()` helpers) can run correctly.

### RP2040 SIO divider 8-cycle latency not modelled

`crates/mdrp2040/src/bus/sio.rs` `DIV_CSR` reports `READY=1` immediately after a divider write. Real hardware requires 8 cycles for the DIV result to become available. Pico SDK `hw_divider_delay` uses inline-asm hard-coded NOPs rather than polling `CSR.READY`, so most SDK-using firmware is unaffected, but any firmware that busy-polls `CSR.READY` will read a stale result. Low priority — fix with a cycle counter on the divider state.

### RP2040 PLL LOCK always 1

`crates/mdrp2040/src/bus/clocks.rs` forces `PLL_SYS_CS[31]` (LOCK) to 1 on read so firmware wait-for-lock loops fall through on the first poll. If firmware writes `FBDIV_INT=0` and then polls LOCK, it will observe LOCK=1 but the derived `pll_output_hz` returns 0 (so the clock tree is 0 Hz). Partial mitigation: callers reading `sys_clk_hz` see the zero propagation. Proper modelling: LOCK=1 only when `pll_output_hz` > 0 (and/or only after a configured lock-delay). Low priority.

**Confirmed present on mdrp2350** (2026-04-15) via `silicon_periph_diff_rp2350` `pll_sys_lock_timing` scenario — `crates/mdrp2350/src/bus/peripherals.rs:21` forces CS[31]=1 unconditionally regardless of CS.ENABLE, PWR, or settle time. Same fix applies.

**Resolved (2026-04-15):** both chips now derive CS[31] from the PLL
register image, a `Bus::pll_*_lock_at_cycle: Option<u64>` arm state, and
the current master cycle count, via three pure helpers in
`mdpicoem_common::clocks` (`PLL_LOCK_DELAY_SYSCLKS = 2_000`,
`pll_is_locked_base`, `pll_cs_read_with_lock`, `pll_should_arm_lock`).
The write path implements **Option B** — PWR transitions back through
"not powered / not configured" drop the arm, and FBDIV / REFDIV changes
while still powered re-arm per silicon behaviour (PRIM / POSTDIV writes
deliberately do not rearm). See
`wrk_docs/2026.04.15 - HLD - PLL LOCK Modelling.md` for the design;
twelve per-chip integration tests (`test_pll_cs_*` / `test_pll_usb_*`)
plus sixteen common-side helper tests cover the blast radius.

### ~~RP2040 core 1 SDK handshake not parsed~~ (resolved 2026-04-16)

Resolved by `wrk_docs/2026.04.16 - HLD - RP2040 Core 1 Multicore Launch Handshake.md`. `Sio::fifo_wr` now runs a 6-state FSM for the full `multicore_launch_core1_raw` protocol (0, 0, 1, VTOR, SP, entry) while core 1 is halted; `Emulator::maybe_wake_core1` consumes the emitted `Core1Launch` token and applies VTOR/MSP/PC + `reset_control_for_launch` before waking the core. Covered by T1..T9 in `crates/mdrp2040/tests/multicore.rs` (including the SDK-sender-scripted T9).

### RP2040 multicore launch: entry with Thumb bit clear silently stripped

`Emulator::maybe_wake_core1` and `Emulator::direct_boot_from_flash` both land core 1 with `pc = entry & !1`. On real silicon a BLX target with bit 0 clear raises a UsageFault (escalated to HardFault on M0+). Our emulator silently strips the bit, so malformed vector tables get the wrong diagnostic. Low risk — pico-sdk always sets the Thumb bit on reset-vector words — but if real PicoGUS-like firmware miswrites the handshake `entry` field, our emulator will run where silicon would fault. Fix: validate bit 0 on entry and raise `Fault::InvalidEpsr` / `HardFault` instead. Applies to both sites symmetrically.

### RP2040 multicore launch: SCR.SLEEPDEEP not cleared on core 1 wake

The real RP2040 bootrom at `bootrom_rt0.S:366-368` clears `SCR.SLEEPDEEP` immediately before `BLX` on the freshly-launched core 1. Our `maybe_wake_core1` shortcut skips that write — consistent with `direct_boot_from_flash` which also doesn't touch SCR. Low risk: firmware expects SCR=0 on a fresh launch and a fresh core boots with SCR=0, so the real bootrom's clear is defensive. Fix for parity: in `maybe_wake_core1`, clear `ppb[1].scr & !0x4` (SLEEPDEEP is bit 2) before wake.

### RP2040 SIO address-mask quirk: atomic aliases hit unmapped offsets

`Bus::sio_write32` does `offset = addr & 0xFFF` before dispatch. That strips the atomic-alias bits (bits 12-13), but it also folds `0xD000_2054` (which is outside the SIO window on real silicon — SIO is 4 KB at 0xD000_0000..0xD000_0FFF) down onto `fifo_wr`. Effect: firmware that inadvertently writes to the second SIO-sized page sees our FIFO respond when real silicon would bus-fault. Pre-existing, surfaced while auditing `fifo_wr` for the multicore handshake HLD. Fix: validate `addr` is within `SIO_BASE..SIO_BASE+0x1000` before dispatch, or preserve the alias bits and use proper alias semantics.

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

### PIO INTn_INTE routing — RESOLVED 2026-04-16

`Emulator::tick_pio_and_route_irqs_single` in mdrp2040 now routes via
`PioBlock::int0_ints` / `int1_ints` (i.e. `(INTR & INTE) | INTF`),
matching the mdrp2350 implementation landed in commit `8bb7614`. The
shared register surface (offsets `0x12C`..`0x140` on `PioBlock`) is
already wired through the bus dispatch. Resolves the PicoGUS audio
blocker — firmware ISA handlers fire on PIO0 RX FIFO RXNEMPTY as
intended.

Note: `pio_all_idle()` still keys on `irq_flags` only, not on
`int0_ints` / `int1_ints`. Firmware that enables `INTn_INTE` for an
RXNEMPTY/TXNFULL bit while leaving all SMs disabled (an unusual
pattern) will miss the IRQ on the fast path. PicoGUS keeps SM0
enabled whenever the IRQ matters, so this is not on the critical
path. Update `pio_all_idle()` to consult `int0_ints`/`int1_ints` if
a future workload needs the disabled-SM IRQ behaviour.

## Phase 6 Simplifications (Harness split)

These surfaced during Phase 6 (the `mdpicoem-harness` binary split into
chip-suffixed runners). The workspace compiles and both `qemu_diff_m33`
and `qemu_diff_m0plus` oracles pass their smoke runs, but the following
corners are deferred to later phases.

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

## ISR Oracle Residual Cycle Deltas (mdrp2350)

Measured by `silicon_isr_diff_rp2350` against real RP2354 silicon.
State observables (stacked frame, FPCCR, IPSR) all match; only
`cyccnt_delta` diverges on the two cases below.

### `isr_pendsv_cold` — EMU overcounts by 6 (HW=19, EMU=25)

Cold PendSV entry: main pends PendSV, single handler fires, reads
CYCCNT into the mailbox, BKPTs. No EXC_RETURN, no tail-chain. The
EMU+6 delta on the cold path is the remaining gap once load-use
latency and write-buffer drain aren't modelled.

Contributing factors, all documented in
`wrk_docs/2026.04.16 - HLD - Cycle and DualCore Timing Accuracy.md`
§9 "Future Work":

1. **LDR load-use pipeline overlap** — handler's two `LDR` (CYCCNT
   address, CYCCNT value) return 2+2=4 cycles in EMU; silicon can
   fold one to 1 when the destination isn't consumed by the next
   instruction. Worth ~1-2 cycles.
2. **Write-buffer drain overlap on stacking stores** — the 8-word
   basic-frame push on exception entry drains through a write buffer
   on silicon, overlapping with the vector fetch. EMU charges a flat
   12 cycles. Worth ~2-4 cycles.
3. **Handler prologue fetch from a mid-image offset** — the handler
   starts at 0x044 (bank depends on image_base); exception-entry
   vector fetch lands on a non-sequential PC that `decode.rs` will
   correctly penalise at bank 2/6, but the flat 12-cycle
   `enter_exception` cost may already include / exclude this
   inconsistently.

Fixing any single one of these is scope-creep against the HLD that
already landed (5 of 10 oracle cases fixed, 5 improved-but-residual).
Treat as a follow-up HLD for exception-entry cycle fidelity when the
residual causes a firmware-observable timing bug.

### `isr_tail_chain_pendsv_systick` — scenario mis-named (RESOLVED to cold parity)

The scenario as catalogued in v1 does NOT exercise tail-chain
transitions. Shared `HANDLER_BASELINE` ends in `bkpt #0` at halfword
[4] — no EXC_RETURN is ever issued, so the emulator's tail-chain
path (landed 2026-04-16) is never entered by this scenario.

**Root cause of the original HW=19 / EMU=15 delta (investigated 2026-04-16):**

The preamble writes `SYST_CVR=0` then `SYST_CSR=ENABLE|TICKINT|CLKSOURCE`.
`Ppb::systick_advance` at `crates/mdrp2350/src/bus/ppb.rs:743` had an
off-by-RVR bug at CVR=0 startup:

```rust
// Pre-fix: cvr=0, rem=1 → "rem -= cvr+1 = 1; cvr=RVR; FIRE."
// Consumed 1 cycle to fire, wrong by RVR cycles vs silicon.
```

ARMv8-M §B11.2.1 counter operation: when CVR=0 at start of a tick,
the counter LOADS RVR into CVR on that tick (reload, no fire).
Pending only asserts on the subsequent cvr→0 decrement transition.
So CVR=0 with RVR=4 should take 5 ticks to the first fire (1 reload
+ 4 decrements). EMU fired after just 1 tick.

**Fix landed (2026-04-16):** `systick_advance` now handles the CVR=0
start as a reload-without-fire step, then falls through to the
normal countdown loop. Regression test
`test_systick_cvr_zero_reloads_without_fire_on_first_tick` pins the
behaviour. Scratch investigation test
`test_investigate_cold_vs_tail_chain_emu_cyccnt` in
`crates/mdpicoem-harness/src/isr_scenarios.rs` confirms both
scenarios now report mailbox CYCCNT = 22 (HW=19 for both, so EMU=+3
matching the cold-ISR residual above). The scenarios are no longer
divergent between themselves on EMU.

### `Ppb::systick_advance` — cvr→0 via subtraction is silent (bug 2)

Separate systick bug discovered during the investigation. The
`rem <= cvr` branch does `cvr -= rem` and, when the result is
exactly 0, does NOT fire COUNTFLAG / pend the exception. Silicon
fires on the transition to 0 regardless of whether it's reached by
decrement or by a multi-cycle subtraction.

**Scope:** one if-block in `systick_advance`:

```rust
if rem <= self.syst_cvr {
    self.syst_cvr -= rem;
    // TODO: if cvr==0 here, set COUNTFLAG + pend_systick (if TICKINT).
    break;
}
```

**Why deferred:** The fix is silicon-accurate in isolation but
interacts with the cold-ISR cycle residual (HLD §9 Future Work:
main instruction cycle model over-counts by ~3 cycles on EMU vs
silicon). Applying it makes the `isr_tail_chain_pendsv_systick`
scenario fire SysTick one EMU step earlier than silicon would,
splitting the ISR oracle's unified +3 residual into +3 (cold) /
−3 (tail-chain) — a worse oracle signal. Once the cold-ISR
residual is closed, re-apply bug 2 fix to regain silicon semantics
without signal degradation.

Priority: low (latent, no current scenario exercises the rem=cvr
boundary in a way that observably diverges from silicon).

### Tail-chain fast path landed (2026-04-16)

`exit_exception` now speculates post-pop priority against pending
exceptions; on tail-chain, skips the unstack + re-stack and jumps
directly to the new handler at 6 cycles (vs ~24 for the old
two-step exit-then-reentry). See
`crates/mdrp2350/src/core/exceptions.rs` `activate_tail_chain`, and
unit tests `test_tail_chain_pendsv_to_systick_preserves_frame` +
`test_tail_chain_cycle_cost_is_discounted` in
`crates/mdrp2350/src/tests.rs`. Architecturally correct; does not
close the `isr_pendsv_cold` residual (separate cold-entry gap) and
did not directly close `isr_tail_chain_pendsv_systick` either —
that scenario's delta was caused by the systick CVR=0 bug above,
closed by a separate fix to `Ppb::systick_advance`.

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

## PicoGUS Integration — Stage 4 follow-ups

Surfaced by Stage 4 devils-advocate. Some items are blockers for
Stage 6 and must be resolved before the end-to-end MIDI demo lands.

### ISA pin mapping not cross-checked against PicoGUS v4.0.0 firmware

Stage 4's replayer hardcodes pin assignments (IOW=GPIO4, IOR=GPIO5,
AD0..9=GPIO6..15) in the top-of-file comment. These came from the
original research summary but have NOT been cross-checked line-for-
line against `github.com/polpo/picogus/blob/v4.0.0/sw/isa_io.pio`
and `sw/CMakeLists.txt`. Two concrete risks:

1. A pin number mismatch means the firmware's PIO program never sees
   the waveform we drive, Stage 6 produces silence.
2. If any ISA pin collides with the I2S pins Stage 5 needs
   (PCM5102-style I2S typically lives on GPIO26/27/28 in PicoGUS),
   we'll have a collision at Stage 5/6 integration time.

Fix: before Stage 5 coding starts, vendor `polpo/picogus@v4.0.0`'s
`isa_io.pio` + `CMakeLists.txt` snapshots under `third_party/` and
extract the authoritative pin constants into a shared module (e.g.
`mdpicoem-common::picogus_pins`). Stage 4's replayer and Stage 5's
I2S decoder should both import from there. **Must resolve before
Stage 6.**

### `write16` → two `write8`s is semantically wrong for GUS 16-bit ports

The replayer splits a `write16` event at port P into two write8s at
P and P+1. Real GUS has 16-bit registers (e.g. voice-start-high)
that decode as a single 16-bit port, not two 8-bit ports. Splitting
can write to the wrong register. Real firmware may not trip this
(DOSBox-X's tap preserves the width, and GUS MIDI playback may not
actually use 16-bit port accesses), but traces from some drivers
could. Fix: either (a) extend the synthetic waveform to drive SBHE#
and a second pin block for D8..D15, or (b) emit a warning on first
`write16` and document as a caveat. Defer until Stage 6 surfaces the
need (if a MIDI file fails to replay correctly). Low priority.

### Stage 4 misleading comment after B1 fix

Comment in `picogus_diff_rp2040.rs` near the drive_pins path still
refers to preserving "firmware-driven pin state" (PSRAM MISO etc.).
The B1 fix (external override on Bus) makes this a lie — the mask
preserves bit-position-wise, but `update_gpio` always rebuilds
`bus.gpio_in` from scratch and re-applies the override. The test
mirroring into `bus.gpio_in` within `drive_pins` is decorative for
testability, not load-bearing for correctness. Tidy the comment.
Trivial (1 minute).

## Cycle-Timing — Sequence-in-Loop Measurements (2026-04-15)

Entries below come from `silicon_cycle_oracle_rp2350` — a sequence-in-
loop oracle that measures one `BLX seq / seq body / BX LR` round-trip
per iteration inside a steady-state K-delta measurement loop
(K_low=101, K_high=201; per_iter = (m_high − m_low) / (K_high − K_low)).

**These entries are NOT directly comparable to the halt-step per-
instruction entries above** (under "Cycle Timing — Phase 2 (Bus
Fabric)" and "Cycle Timing — Halt-Step Measurement Limitations").
The halt-step entries isolate one instruction's cost plus a fixed
5-cycle debug overhead; the entries here measure a *bundle* (BLX +
seq body + BX LR + loop overhead) at native speed with pipeline
effects fully engaged. Deltas of a few cycles between HW and EMU in
one measurement mode do not imply the other mode is wrong — the two
modes answer different questions. Do not fold these numbers into
tech-debt estimates framed in the halt-step context, and do not
"close" halt-step entries based on sequence-in-loop results.

Measured on the RP2354 attached via Pico debug probe, 2026-04-15,
with per-case emu baselines seeded from the current mdrp2350 cycle
model.

### Resolved (known-delta) — Cycle Oracle per-case tolerances (2026-04-21)

Four sequence-in-loop cases previously FAIL at tol=0 are now PASS under
per-case tolerances that encode the known pipeline-overlap residuals.
See `wrk_docs/2026.04.21 - HLD - Track B Cycle Oracle Fidelity.md` for
the full root-cause analysis, option comparison (Option A: emulator
cycle-accounting fixes vs. Option B: tolerance widening), and lead
sign-off. Option B (Track B) landed.

Current deltas (2026-04-21) and per-case tolerances:

| case                             | HW/iter | EMU/iter | delta | tol | verdict |
|----------------------------------|--------:|---------:|------:|----:|:--------|
| nop_chain_8                      |      11 |       14 |    −3 |   3 | known Δ |
| push_2_min_cost                  |      10 |       12 |    −2 |   2 | known Δ |
| backward_branch_small            |      13 |       13 |     0 |   0 | PASS    |
| backward_branch_large            |      13 |       13 |     0 |   0 | PASS    |
| bank_contention_fetch_data_same  |       9 |       10 |    −1 |   1 | known Δ |
| bank_contention_fetch_data_diff  |       9 |       10 |    −1 |   1 | known Δ |
| ldm_8_reg                        |      17 |       17 |     0 |   0 | PASS    |
| single_adds                      |       7 |        7 |     0 |   0 | PASS    |
| back_to_back_alu                 |      14 |       14 |     0 |   0 | PASS    |

Root causes per case (full analysis in the HLD §3):

- **`nop_chain_8` (Δ=−3, tol=3 — positive control)**: M33 folds
  consecutive NOPs in the prefetch/issue path (≈−1 to −2) + silicon
  absorbs the stub's BLX/BX-LR framing into prefetch state in a way our
  "sequential vs non-sequential" fetch abstraction cannot model (≈−1).
- **`push_2_min_cost` (Δ=−2, tol=2)**: M33 write buffer partially
  forwards the two PUSH stores to the subsequent POP loads at the same
  addresses, collapsing the 6-cycle body to ≈4.
- **`bank_contention_fetch_data_same` / `_diff` (Δ=−1, tol=1)**:
  not contention-sourced — silicon exhibits no observable bank
  contention at sequence-in-loop scale. The residual is LDR→LDR
  load-to-use forwarding (first LDR's result forwarded to second LDR's
  address phase, saving one cycle).

All four were explicitly flagged as "Phase-2 future work" in the prior
2026.04.16 HLD; this HLD formalises the tolerance-widening approach the
earlier HLD already recommended. The previous "Positive-control case —
nop_chain_8 FAIL" and "Sequence-in-loop deltas per case" sub-entries
are replaced by the table above; the `emu_baseline` column constants in
`cycle_cases.rs` were already updated to match today's EMU per-iter
values before Track B landed, so `NOTE: emu per-iter differs from
emu_baseline` no longer fires on these cases.

### Phase-2 pipeline-model roadmap

The route to tightening tolerance back to 0 on the four known-delta
cases requires emulator-side pipeline state. All three features were
considered and deferred by the Track B Cycle Oracle Fidelity HLD (§4
Option A, §5) because the 1–3 cycle residuals do not justify the 2–3
week engineering cost + wide unit-test blast radius. If a future phase
revisits this, the features are:

1. **NOP fold heuristic** — detect runs of `BF00` at fetch
   classification time and charge `ceil(N * 0.75)` instead of `N`.
   Covers `nop_chain_8`'s body contribution (≈−1 to −2 cycles).
2. **Write-buffer forwarding** — track a small write-buffer state on
   `Bus` (address + data of last ≤2 stores); when a load matches a
   pending store's address, return 1-cycle load cost instead of 2.
   Covers `push_2_min_cost`.
3. **Load-to-use forwarding** — track destination register of the
   most-recent LDR on `CortexM33`; when the next instruction uses that
   register as an address base, credit −1 cycle to the LDR's cost.
   Covers `bank_contention_fetch_data_same/diff`.

Blast radius: feature (3) is the riskiest — dozens of `last_access_cycles`/
`extra_wait_states` unit-test sites in `crates/mdrp2350/src/tests.rs` rely
on single-instruction isolation that inter-instruction state tracking
would break. Features (1) and (2) are narrower but still touch the
fetch-classification / bus hot path.

## PicoGUS Integration — Stage 6 follow-ups

### ~~Real RP2040 bootrom not vendored~~ (resolved 2026-04-15)

Fixed by vendoring the upstream B2 bootrom as
`roms/rp2040/bootrom-rp2040-b2.bin` (SHA256
`9c19b46f068c21f90d200c514faad4a0d5cecfc978f155b8c9d25cb6bc2efd81`,
BSD-3-Clause). `picogus_diff_rp2040` gained a `--bootrom` flag that
default-searches this path when `--flash` is supplied. Journal:
`wrk_journals/2026.04.15 - JRN - PicoGUS RP2040 Bootrom + Boot Smoke.md`.

Superseded by the two follow-ups below: the bootrom alone isn't enough
to boot real SDK firmware, because our peripheral model is incomplete,
and the SDK runtime itself also trips an assertion we don't yet
understand.

### ~~PicoGUS v4.0.0 firmware panics early in pico-sdk runtime~~ (resolved 2026-04-15)

Resolved across four phases. Full narrative in
`wrk_journals/2026.04.15 - JRN - PicoGUS SDK Panic Debug.md`. The
"alarm 1 already claimed" hypothesis from the original diagnosis
turned out to be a red herring — the r0=1 panic argument was the
IRQ number passed to `irq_set_exclusive_handler`, not a hardware
alarm index.

- **Phase 1** (`22135ff`, `eadda71`) — `direct_boot_from_flash`
  didn't write VTOR. Real silicon's `exit_from_boot2` writes SP +
  VTOR + PC before jumping to the SDK reset handler; we were only
  setting SP + PC. Added the VTOR write, both cores.
- **Phase 2** (`34d7d6a`) — `crates/mdrp2040/src/bus/ppb.rs` had a
  broken mask/pattern pair. `match addr & 0x0FFF_FFFF { 0x000E_D008
  => self.vtor }` never matched because for `addr = 0xE000_ED08` the
  mask yields `0x0000_ED08`, not `0x000E_D008`. Every SCB register
  read returned 0, every write was silently dropped. Existing tests
  all used direct field assignment (`bus.ppb[0].vtor = val`) which
  bypassed the match, so the bug was invisible. Rewrote using the
  correct mdrp2350 idiom (`match addr & 0xFFFF { 0xED08 =>
  self.vtor }`) + added 5 regression tests + bus-level integration
  assertion.
- **Phase 4** (`5e113dc`) — `execute.rs` incorrectly raised
  `Fault::InvalidEpsr` on `MOV PC, Rm` and `ADD PC, Rm` with even
  destination. Per ARMv6-M ARM DDI 0419E §A5.1.2, `ALUWritePC`
  (used by those two instructions when Rd=15) just masks the LSB —
  only `BXWritePC` / `LoadWritePC` correctly fault. gcc emits `mov
  pc, rN` for switch jump tables with even-aligned label targets;
  this tripped the bogus fault in `_vfprintf`. Removed the check,
  added positive regression tests. mdrp2350 already had the correct
  behaviour.

**Result**: real PicoGUS v4.0.0 firmware now runs through
`picogus_diff_rp2040` for 62.7 M cycles (2.002 s simulated time)
with zero stall events. All SDK panic paths are cleared.

Three new blockers surfaced during Phase 5 diagnosis (no audio yet)
— see entries below.

### PicoGUS: no I2S output — blocked on remaining DMA gate

**Impact: HIGH** for the end-to-end PicoGUS ear-test acceptance.
None blocks the oracle itself — the SDK panic is cleared, tests are
green, chime firmware still produces audio.

Two of three emulator peripheral gates are now resolved; the
remaining gate is DMA.

1. ~~**RP2040 TIMER peripheral model** (blocker-1)~~ — RESOLVED.
   TIMERAWH/TIMERAWL + ALARM0..3 + NVIC IRQ routing landed.

2. ~~**RP2040 core 1 SDK launch handshake** (blocker-2)~~ — RESOLVED
   2026-04-16 by the multicore-launch HLD. `Sio::fifo_wr` now parses
   the full `multicore_launch_core1` 6-word handshake; core 1 wakes
   at the supplied entry with the supplied MSP/VTOR. See
   `crates/mdrp2040/tests/multicore.rs` T1..T9.

3. **RP2040 DMA block model** (remaining blocker, MEDIUM impact).
   I2S output is DMA → PIO TX FIFO. No DMA = no PIO FIFO samples
   ever get loaded = no BCLK/LRCLK/DOUT output even if PIO were
   programmed. Our emulator's `bus/mod.rs` has a generic
   `peripheral_regs` HashMap fall-through at 0x50000000; DMA writes
   land there and do nothing. Scope: ~3-5 days.

4. (Non-emulator) Real DOSBox-X trace capture to drive audio,
   tracked as an external dependency in the PicoGUS Integration HLD
   Stage 6.

After gate-3 lands, audio should finally reach the I2S pins.

### Secondary finding — ISA-pin idle default matters for diagnostic probes

`picogus_diff_rp2040` primes ISA pins to idle (IOW#=IOR#=HIGH)
between replay events via `CapturingSink`. Without that priming
(plain `picogus_probe_pc` before the Phase 5 edit), firmware
observes phantom ISA bus cycles and faults at a different site
early in init. Not a regression — the oracle's priming is
correct-by-design — but it means downstream probes/oracles must
mirror the idle-pin convention to reproduce `picogus_diff_rp2040`'s
observed behaviour. Document in any new PicoGUS-adjacent oracle
binary.

### RP2040 bootrom's QSPI flash detection needs SSI+pads model to pass

**Impact: MEDIUM.** With `direct_boot_from_flash` in place, not
currently blocking anything — but worth fixing if we want to run the
bootrom through the actual boot flow for correctness testing.

RP2040 bootrom `main()` at ROM `0x24d0` detects an attached QSPI
flash by reading `SIO.GPIO_HI_IN` (offset `0x008`) 9 times and
counting how often bit 1 (QSPI_SS) samples high. If ≥ 5 samples have
SS high, it proceeds with `connect_internal_flash` → SSI-based
read / CRC check of boot2 → jump to `0x10000100`. Our
`mdrp2040::bus::sio::read32` returns 0 for offset `0x008` (no QSPI
pad model), so the bootrom always fails the check and enters USB MSC
boot (`async_task_worker` at `0x20d8`).

Minimum fix: make SIO `GPIO_HI_IN` return `0x3E` (SS high + SD0..SD3
pulled up, SCLK low — the idle state with a flash chip attached), and
teach the SSI register model to serve JEDEC ID (`0x9F` → `EF 40 15`
for a W25Q16JV-like device) and READ (`0x03` + 24-bit addr) commands
well enough for the CRC check to pass. ~2 hours.

### Firmware + upstream assets not committed (by design, worth re-reviewing)

`third_party/picogus/` holds `VERSION` + `README.md` in git; the
actual UF2 / bin / zip / exe are `.gitignore`d and fetched by
`scripts/picogus_demo.sh --prepare`. Rationale in
`third_party/picogus/README.md`. If a hermetic CI build ever needs
to run the demo offline, revisit: option A is committing the 900 KB
bin; option B is a git-lfs slot; option C is a private mirror.

### Demo runbook assumes Arthur picks a DOS MIDI player

`third_party/picogus-demo-runbook.md` lists three candidate MIDI
players (CLM.EXE, MIDPLAY.EXE, JMPLAY.EXE) but doesn't pin one —
because none of them are redistributable under clear licences we've
verified. When Stage 6 acceptance runs end-to-end, record the
specific player used + its version in `wrk_journals/` so the trace
is reproducible.

### `Emulator::reset()` clobbers the clock tree to ROSC (~6.5 MHz)

`mdrp2040::Emulator::reset()` resets the clock tree to power-on-ROSC
state, discarding whatever `Config.sys_clk_hz` was seeded with at
construction. Harness tests that mix `reset()` with cycle-accurate
timing must follow up with `bus.seed_sys_clk_hz(N)` (see
`crates/mdpicoem-harness/src/bin/picogus_diff_rp2040.rs` tests
`replay_advances_emulator_to_target_cycles` / `replay_end_to_end_post_roll_reports_cycles`).
Consider an `Emulator::reset_at(sys_clk_hz)` helper to avoid the
copy-paste re-seed pattern, and verify the ROSC-on-warm-reset
behaviour matches silicon (HLD follow-up).

## Phase 1 known limitations (mdrp2040 IRQ / TIMER)

Closed-out from Phase 1 Wave 2 (`HLD V7 §5.2`/`§5.3`) code review.
All four items are by design for Phase 1 and have explicit deferral
owners below.

- **32-bit alarm wrap math** — `TimerRegs::poll_alarms`
  (`crates/mdrp2040/src/peripherals/timer.rs`) does not re-check the
  wrap across a 32-bit boundary for alarms scheduled near the low-word
  rollover. Arming computes `fire_cycle = now + (target - now_lo)` in
  master-cycle space at write time, but a firmware that arms then
  reprograms the time register could mis-fire. Phase 2+.
- **Fixed `sys_hz/1_000_000` tick derivation** — TIMER's
  `cycles_to_us` / `us_to_cycles` collapse the WATCHDOG_TICK.CYCLES
  divider out of the formula and assume one microsecond per
  `sys_hz / 1_000_000` sysclk cycles. Firmware that reprograms
  `clk_peri` or `WATCHDOG_TICK.CYCLES` mid-run will see TIMER drift.
  Phase 2+.
- **Dual-core preemption under the 4-priority collapsed model** —
  `CortexM0Plus::maybe_dispatch_external_irq`
  (`crates/mdrp2040/src/core/mod.rs`) blocks all higher-priority IRQs
  on any core while any core is in handler mode. ARMv6-M real silicon
  preempts per-core: a higher-priority IRQ on core 1 should preempt a
  core-1 handler running at lower priority even while core 0 is in
  handler mode. Our simplified model suffices for corpus firmware that
  uses a single priority level per IRQ; correct per-core nesting is a
  later-phase item.
- **Halted-core IRQ wake (WFE/WFI)** — if core N is halted (WFE/WFI)
  and an IRQ becomes pending+enabled on core N's NVIC, the early-return
  on `is_halted` in `maybe_dispatch_external_irq` means nothing wakes
  the core. Real silicon wakes via the IRQ-pending line even from WFE.
  Phase 2+ wake path needs to re-check `nvic.pending_and_enabled()` on
  peripheral tick and clear `is_halted` when a deliverable IRQ appears.

## Phase 2 known limitations (mdrp2040 UART / SPI / I2C)

Closed-out from Phase 2 Wave 1 (`HLD V7 §5.3`/`§6`) code review. All
five items are by design for Phase 2 and documented in the relevant
peripheral module.

- **UART RX stimulus path not wired** —
  `crates/mdrp2040/src/peripherals/uart.rs` models the TX side (FIFO
  drain + baud-timed cycle accumulator) but does not inject RX bytes
  from any external source. The Phase 2 corpus (`hello_uart`) only
  exercises TX. Firmware that reads `UARTFR.RXFE` or attempts
  `UARTDR` reads will see `RXFE=1` forever. Phase 3+ will need a loop-
  back or scripted stimulus hook.
- **UART modem flow control tied high** — `UARTFR` modem-status bits are
  driven from the nUART* modem pins via IO_BANK0 mux, but the emulator
  doesn't propagate that. CTS-hardwired-high removed for mdrp2350 in
  commit `4243695` (silicon oracle drove the fix). DCD/DSR/RI on
  mdrp2350 + the same CTS/DCD/DSR/RI pattern on mdrp2040 are still
  hardwired and have not been silicon-validated. `UARTCR.RTS`/
  `CTSEn`/`RTSEn` are stored but have no effect on TX gating.
  Firmware that relies on handshake runs ungated.
- **SPI master-slave arbitration: loopback-only** —
  `crates/mdrp2040/src/peripherals/spi.rs` implements `SSPCR1.LBM=1`
  (master/loopback) to round-trip TX→RX so the `hello_spi` corpus can
  verify baud-rate math. Off-chip slave interaction is not modelled;
  any non-LBM transaction drains TX but produces no RX data.
- **I2C 10-bit addressing not modelled** —
  `crates/mdrp2040/src/peripherals/i2c.rs` silently NACKs every
  transaction when `IC_CON.10BITADDR_MASTER=1`, latching TX_ABRT with
  the distinctive `ABRT_10ADDR1_NOACK` bit (not `ABRT_7B_ADDR_NOACK`)
  so firmware can distinguish "unsupported 10-bit" from "7-bit unknown
  slave".
- **I2C SCL timing not modelled** — `IC_SS_SCL_*` / `IC_FS_SCL_*` /
  `IC_SDA_HOLD` / `IC_FS_SPKLEN` are storage-only. Transactions fire
  synchronously at `IC_DATA_CMD` write time (instant ACK/NACK +
  STOP_DET), so firmware that spin-checks `IC_STATUS.ACTIVITY` or
  raw-IRQ ordering expecting bus-cycle-paced events may see different
  interleavings than real silicon.

## Phase 3 known limitations (mdrp2040 ADC / PWM)

Closed-out from Phase 3 (`HLD V7 §6`) code review. All five items are
by design for Phase 3 and documented in the relevant peripheral module.

- **PWM fractional `CH_DIV` (16.4 fixed-point divisor)** — slices
  advance CTR one per sys_clk regardless of DIV. `hello_pwm` corpus
  unaffected (uses DIV=1). See `crates/mdrp2040/src/peripherals/pwm.rs:17`.
- **PWM `PH_CORRECT` triangle mode and `A_INV`/`B_INV` output
  inversion** — storage-only; no behavioural effect.
- **ADC round-robin channel advancement** — `RROBIN` bits stored but
  AINSEL never advances between samples; multi-channel firmware sees
  single-channel behaviour. See
  `crates/mdrp2040/src/peripherals/adc.rs:7`.
- **ADC DREQ emission (DREQ source 36 per V7 Appendix C)** —
  FCS.DREQ_EN stored but no DREQ signal emitted to DMA today. Phase 4
  DMA doesn't consume this lane.
- **PWM wrap DREQs (sources 24..31)** — unmodelled; `collect_dreqs`
  leaves the band zero. `audio_i2s` uses PIO DREQ so the corpus is
  unaffected.

## Phase 4 known limitations (mdrp2040 DMA)

Closed-out from Phase 4 (`HLD V7 §7`) code review. All items are by
design for Phase 4 and documented in the relevant DMA module.

- **DMA `CTRL.BSWAP` (byte-swap) bit** — stored in CTRL but transfer
  ignores it. No corpus firmware uses it.
- **DMA `SNIFF_EN` and `SNIFF_CTRL`/`SNIFF_DATA` registers** —
  storage-only. CRC not implemented.
- **DMA `HIGH_PRIORITY` tier arbitration** — stored in CTRL but
  ignored; flat lowest-channel arbitration used. `audio_i2s` does not
  rely on priority.
- **DMA XIP DREQ sources (37..39, XIP_STREAM / XIP_SSITX /
  XIP_SSIRX)** — not modelled (XIP MMIO stub predates Phase 4).
- **DMA Timer pacing (`TIMER0..3` registers at `DMA_BASE + 0x440`)** —
  storage-only, pacing not applied.
- **Per-channel `DBG_CTDREQ` / `DBG_TCR` debug registers** — read as
  zero. No corpus consults them.
- **`DmaChannel.trans_count_reload` field is redundant with
  `trans_count` today** — overwritten on every TRANS_COUNT write.
  Audit pending to either remove it or capture reload at trigger time.
  See review M1.
- **DMA `mem::take` swap: zero-read window if DMA self-targets its own
  registers during a transfer** — unreachable in corpus firmware;
  documented as known anomaly.

### test_silicon residual failures (2026-04-16 baseline)

- ~~**Cycle timing residuals (3 cases)**: `push_2_min_cost` (delta=-2),
  `bank_contention_fetch_data_same` and `_diff` (delta=-1 each).~~
  **Resolved (known-delta, 2026-04-21)** — accepted as bounded pipeline
  residuals with per-case tolerances in
  `crates/mdpicoem-harness/src/cycle_cases.rs`. See
  `wrk_docs/2026.04.21 - HLD - Track B Cycle Oracle Fidelity.md` and the
  "Resolved (known-delta) — Cycle Oracle per-case tolerances
  (2026-04-21)" section above. Closing these at tol=0 requires Phase-2
  pipeline-model work (write-buffer forwarding + load-to-use forwarding);
  deferred.

- **TICKS TIMER0 CYCLES readback**: `ticks_timer0_retarget_halves_rate`
  fails with EMU=0x18 (correctly accepts aliased write), HW=0x00.
  Pre-existing: the scenario was failing before the aliasing fix too
  (EMU=0x0C, HW=0x00).  The CYCLES register on silicon may not be a
  simple static reload value, or the domain tick logic modifies it.
  Needs investigation on real silicon (probe-read CYCLES at multiple
  points during the scenario to characterise the actual register
  behaviour).

- **DMA oracle scenarios**: `dma_mem_to_mem_32bit` and `dma_chain_trigger`
  diverge because the probe-based setup (DAP writes) doesn't produce a
  valid DMA transfer on silicon.  Emulator DMA is correct (destination
  contains seed data).  RESET_DONE polling was added but didn't resolve
  the issue — likely a DAP write-buffer coherency or debug-halt clock
  gating issue.  Fix: rearchitect DMA scenarios to use a custom sled
  that performs SRAM seeding + DMA configuration + busy-wait at runtime
  through the CPU bus interface, not through the debug port.

- **test_silicon orchestrator**: `session.core(0)` fails in worker
  thread even with `--probe` explicit selector.  Root cause is cross-
  thread session transfer in probe-rs, not probe selection.  Standalone
  oracle binaries work fine.  The `--probe` flag is still valuable for
  multi-probe disambiguation.  Needs probe-rs investigation or
  restructuring the orchestrator to call `session.core(0)` from the
  main thread before moving to the worker.

- **adc_one_shot**: crashes probe with "An ARM specific error" even
  with GPIO26 pad configuration (OD=1, IE=0, funcsel=NULL).  ADC
  analog subsystem appears fundamentally hostile to halted-core probe
  access.  Recommend gating behind `--include-adc` flag or moving to
  `RED_PATH_SCENARIOS`.

- **`hello_dma.bin` generator drift** — the checked-in binary is generated
  by `roms/rp2350/gen_hello_dma.py`; any datasheet correction (e.g. the
  CTRL_BUSY bit-24 → bit-26 fix, 2026-04-16) silently invalidates the blob
  until the script is updated and regenerated.  Fix candidates: make it a
  `build.rs` artefact, or add CI that diffs generator output against the
  committed binary.

- **`#[allow(dead_code)]` on DMA flag constants masks bit-position bugs** —
  `CTRL_BSWAP`, `CTRL_SNIFF_EN`, `CTRL_HIGH_PRIORITY` in
  `crates/mdrp2350/src/dma.rs` are stored but ignored.  When promoted to
  active use in a future phase, the promotion path must include a
  bit-position assertion test (see `ctrl_busy_is_at_bit_26_not_bit_24` for
  the pattern).

- **`uart0_tx_single_byte` scenario `min_sysclks: 10_000` is below the
  byte-time floor** — at 150 MHz `clk_peri` and 115200 baud, one byte
  needs ~13,020 sysclks for the TX shift register to drain. Scenario's
  comment claims "~13_000 sysclks" but the literal is 10,000. Currently
  PASSes on silicon by luck (the actual run takes longer than min); will
  flake or fail under timing variance. Bump to ~20,000 in a follow-up
  pass. Surfaced by the scenario-fixes agent during the Stage A fidelity
  fix wave (2026-04-16) but deferred to keep that wave's scope tight.

- **UART/SPI/I2C ignore `CLK_PERI_CTRL.ENABLE`** — peripheral `tick`
  paths in `crates/mdrp2350/src/peripherals/{uart,spi,i2c}.rs` advance
  their state machines regardless of the `CLK_PERI_CTRL.ENABLE` gate.
  Silicon post-`Core::reset_and_halt` starts with `CLK_PERI_CTRL=0`
  (the bootrom's `runtime_init_clocks` didn't run), so silicon's UART
  shift register sits idle until firmware flips ENABLE. The emulator
  happily drains at its seeded 150 MHz peri_clk in the same window.
  Evidence: residual A.2.2 — `uart0_rx_loopback` reported
  `HW=0x18 EMU=0x80` at `0x4007_0018` until the scenario started
  writing `CLK_PERI_CTRL=0x800` as its second setup step (2026-04-17,
  `wrk_docs/2026.04.17 - HLD - Residual A.2.2 UART RX Loopback BUSY
  Fix.md`). Adding gate-aware tick paths is tempting for fidelity but
  carries wide blast radius: every scenario that currently passes does
  so because the emulator runs clk_peri unconditionally
  (`spi0_loopback_single_byte`, `i2c0_bus_scan_reserved_nack`,
  `uart0_tx_single_byte` all skip the ENABLE write). A co-ordinated
  audit of peri-clock consumers plus every scenario that implicitly
  relies on "clk_peri always live" is needed before flipping the
  switch. Defer until a firmware scenario genuinely exercises dynamic
  `CLK_PERI_CTRL` enable/disable. Related emitter:
  `crates/mdrp2350/src/bus/peripherals.rs:194` warn-once
  "CLOCKS CLK_*_CTRL.ENABLE cleared; clock-gate behaviour not modelled".

### `Verdict::ResolvedAddrOutOfRange` unreachable at runtime

In `crates/mdpicoem-harness/src/onerom_serving_oracle.rs`, the stim-pattern
predicate already restricts resolved addresses to `hi16 == 0x2000` before
the `SHADOW_BASE..SHADOW_BASE + SHADOW_SIZE` bounds check runs. With
`SHADOW_SIZE = 0x1_0000` the shadow spans the full u16 low-half, so every
stim-matching push is architecturally in-range and the
`Verdict::ResolvedAddrOutOfRange` arm cannot fire. The variant is kept
intentionally as a belt-and-braces guard: if a future shrink of
`SHADOW_SIZE` drops below `0x1_0000`, the check catches the regression
instead of indexing past the shadow. Document-only; no code change.

### `CpuServingOracle` pin map is hardcoded (blocks CPU-mode stress on non-`test-sdrr-0` fixtures)

In `crates/mdpicoem-harness/src/onerom_serving_oracle_cpu.rs`, `run_case`
drives CS via `GPIO_CS1 = 13` (plus `GPIO_CS2 = 12`, `GPIO_CS3 = 15`) and
uses the fixed `ADDR_PINS` permutation. Those constants match the
`test-sdrr-0-cpu` fixture, where the CPU firmware was baked to read CS
from GPIO13. The `1541-cpu` fixture was baked with its "chip select"
wired to GPIO0 instead (firmware-side metadata difference; both use the
same `fire-24-a` hardware pin mapping, but the firmware's runtime gating
register is different). Net effect: `onerom_stress_cpu_rp2350` against
`1541-cpu.bin` only passes ~140/2048 cases — every case where the
stim-pattern happens to set GPIO0 high passes; the rest report
`NoResolve` because the firmware's wait-loop poll never takes the "CS
asserted" branch.

Fix: parameterise `CpuServingOracle::run_case` over a `PinProfile` read
from the fixture's SDRR metadata (see HLD
`wrk_docs/2026.04.17 - HLD - OneROM Stress Harness.md` §Open questions
for the shape). Low-risk change once scoped — ~1 day. Out of scope for
the initial stress-harness wave (2026-04-17). The stress CPU binary is
retained as a latent regression target: when the fix lands, it'll flip
to full 2048/2048 PASS without any binary-side change.

**Caveat on the current 140 "passes":** they are a double-coincidence,
not real serve-path coverage. Each passing case has (a) a stim pattern
that incidentally sets GPIO0 high AND (b) an expected shadow byte of
`0x00`, so the oracle's `ZERO_BYTE_TRUST_TIMEOUT_CPU` fallback declares
pass after 40 cycles of dead pins. No case currently verifies that the
CPU actually drove the right byte onto the data pins via the 1541
serve loop. The pass count should be read as "0 of 2048 cases are real
serve-path verifications" until the pin-profile fix lands.

## Stage 2 dual-model test gap: no cross-core IRQ routing test (2026-04-24)

`crates/mdrp2350/tests/dual_model.rs` has 19 tests but none of them
assert that Core 0 IRQ routes to its NVIC while Core 1's NVIC stays
clean (or vice versa). Cross-core IRQ delivery is dual-core-specific
scheduler behaviour; Threaded quantum boundaries could silently drift
on this without oracle coverage. The existing
`nvic_pre_run_enable_write_accepted` test only covers the enable-
write accept path, not post-IRQ routing asymmetry.

Pick up in Stage 3/4 or a follow-up `dual_model.rs` extension. Risk:
low — unit-level NVIC tests cover per-core routing in isolation
(`crates/mdrp2350/src/tests.rs` covers `assert_irq_*` → ISPR/NVIC_PEND
for each core individually), but the Threaded scheduler could in
principle introduce ordering violations (e.g. an IRQ asserted on
core 0 becoming visible to core 1's `merge_irq_pending` first) that
single-core oracles wouldn't detect. Landing a dual-model test for
this fills the gap.

Sketch: seed an IRQ-handler stub in SRAM for both cores, program
NVIC_ISER to enable a vector on only core 0, assert the IRQ via
`Bus::assert_irq_*` from the harness, run one quantum, and diff
`core(0).ppb.nvic_ispr` vs `core(1).ppb.nvic_ispr` — the former
must flip, the latter must stay zero — on both Serial and Threaded.
The Serial-only assertions (PPB reads debug-assert `not_placeholder`)
mean the observable has to be captured via a scoped pre/post
`both_models_compare` pattern, or through the ThreadedEmulator's
`shared()` state on the Threaded branch.

## Stage 3b.3: RP2040 threaded UART/SPI/I2C/ADC/PWM/DMA route through legacy HashMap (2026-04-24)

Serial `Bus::peripheral_write32` dispatches UART / SPI / I2C / ADC / PWM
/ DMA through typed peripheral modules (`self.uart0.write32(..., &mut
self.irq_pending)` etc.) with register-level side effects — IRQ RIS
bits for UART/SPI/I2C, trigger-channel transfer kickoff for DMA. Stage
3b.3 routes these same regions through
`SharedState.peripherals.legacy: Mutex<HashMap<u32, u32>>` as raw
register storage, preserving RAW (read-after-write) semantics through
the alias-aware update rule (plain / XOR / OR / AND-NOT) but losing the
side effects.

Risk: firmware using these peripherals on the threaded path sees an
IRQ-less UART / SPI / I2C and a non-ticking DMA. pico-sdk code that
polls for TX-fifo-ready via bit-bash will still work (the register
round-trips); code that waits on an ISR will hang.

Mitigation for Stage 4 crossover measurement: treat these peripherals
as Serial-only. `paced_bench_rp2040 --model serial` remains the path
for any firmware that exercises UART/SPI/I2C/DMA. If Stage 4 benchmarks
demand threaded coverage of these peripherals, follow up with typed
state on `Peripherals` (same pattern as the TIMER fix in Stage 3b.3 —
reuse the existing serial type inside a thin `Mutex<...>` wrapper).

TIMER is already typed on the threaded path — the TIMELR→TIMEHR read-
latching side effect cannot be modelled by the HashMap.

## RP2040 SysTick CLKSOURCE simplification (2026-04-25, V5 §9.1)

V1 of the RP2040 IRQ Plumbing oracle (`silicon_isr_diff_rp2040`) ticks
SysTick once per master cycle for both `SYST_CSR.CLKSOURCE=0` (external
1-µs watchdog tick on real silicon) and `CLKSOURCE=1` (processor clock).
The V1 oracle scenarios all program `CLKSOURCE=1`, so the model is
exact for the cases under test. A future scenario relying on the 1-µs
external cadence will see CLKSOURCE=0 ticking faster on EMU than on
silicon by a factor of `sys_hz / 1_000_000` (≈125× at the default
125 MHz sysclk).

Mitigation: small change to `crates/mdrp2040/src/bus/systick.rs::tick`
(or its threaded sibling) to consult `CLKSOURCE` and gate the master-
cycle tick against the 1-µs boundary derived from the WATCHDOG_TICK
divider. Revisit when the first such scenario surfaces — no firmware in
the V1 corpus exercises CLKSOURCE=0.

## RP2040 no-preemption rule + asymmetric multi-core SysTick (2026-04-25, V5 §9.2)

Two related simplifications in the V1 RP2040 IRQ plumbing, neither
exercised by V1 oracle scenarios:

1. **No-preemption rule.** `try_take_any_pending_exception::can_
   dispatch_now` (in `crates/mdrp2040/src/core/exceptions.rs`) returns
   `true` iff `!ppb.any_active()` — i.e. it refuses to preempt *any*
   running handler, regardless of priority. This is stricter than
   ARMv6-M ARM §B1.5.4, which permits a higher-priority pending
   exception to preempt a lower-priority active handler. V1 oracle
   scenarios all dispatch from thread mode (no nested handlers), so
   the gap is invisible today.

2. **Asymmetric multi-core SysTick.** V1 ticks SysTick on the active
   core only — i.e. `bus.systick.tick(core_id, ...)` advances the per-
   core CVR for whichever core is currently executing in
   `Emulator::step`. Correct when at most one core is unhalted (the V1
   oracle case) OR when both cores' SysTicks are programmed
   identically. Asymmetric multi-core SysTick programming (different
   RVRs, different CLKSOURCEs, or only one core's SysTick enabled)
   would tick the secondary core's SysTick at half-rate relative to
   silicon, since the primary core consumes half the wall-clock cycles
   and only its CVR ticks during that window.

Mitigation: when the first scenario needs preemption or asymmetric
multi-core SysTick, wire priority-aware preemption into
`can_dispatch_now` (compare pending exception priority against the
active-handler-stack top in PPB) and per-core SysTick scheduling (tick
both cores' CVRs every master cycle, gated by per-core enable).

## RP2040 step_quantum boundary effects with NVIC MMIO (2026-04-25, V5 §9.4)

The V1 oracle integration test (`crates/mdpicoem-harness/tests/
isr_scenarios_rp2040_emu.rs`) pins `step_quantum=1` to force a clean
per-cycle dispatch decision. The existing dual_model parity test at
`crates/mdrp2040/tests/dual_model.rs:444` uses default quantum (64) but
does not touch NVIC MMIO during its pacing window — its IRQ source is
the SIO FIFO, which dispatches via the bus tick path rather than via
software-driven `ISER`/`ISPR` writes. So no exposure today.

A future scenario combining `step_quantum > 1` + threaded execution +
NVIC MMIO writes (e.g. firmware that pends an IRQ via `ISPR` and
expects single-cycle latency) may surface a quantum-boundary IRQ-drain
race: the NVIC write lands mid-quantum, but the dispatch check only
fires at the quantum boundary, so observable IRQ latency depends on
where in the quantum the write landed.

Mitigation: add such a scenario when motivated. Likely fix is to make
the threaded-path NVIC MMIO write trigger a same-tick re-check of
`try_take_any_pending_exception` rather than waiting for the next
quantum boundary. Verify behaviour against silicon (`probe_diff_rp2040`
or a new `silicon_isr_diff_rp2040` scenario) before changing the
production path.

## RP2040 NVIC ISER/ISPR/ICER/ICPR bits 26..31 latch instead of RAZ/WI (2026-04-25, V5 §9.6)

The current NVIC MMIO write path
(`crates/mdrp2040/src/bus/mod.rs:1422-1439`) has no mask on `n.enabled
|= val` / `n.pending |= val`, so a write of `0xFFFF_FFFF` to ISER0 /
ISPR0 latches all 32 enable / pending bits. Only IPR is gated by the
`if irq < 32` check at `bus/mod.rs:1448`, which on M0+ is a no-op
since the NVIC array is 32-wide anyway.

Bits beyond IRQ 25 are unobservable in normal operation: no peripheral
source asserts them, and no exception vector slot exists for them
(the RP2040 vector table tops out at IRQ 25 / vector index 41), so
they cannot dispatch through `try_take_any_pending_exception`. V1
keeps this behaviour intentionally — masking writes adds cost on the
hot path for no observable correctness gain in any V1 scenario.

Mitigation: on real RP2040 silicon, bits 26..31 may be RAZ/WI per the
ARMv6-M ARM (implementation-defined for unimplemented IRQ slots). If
`probe_diff_rp2040` later proves silicon RAZ/WI's those bits, the fix
is a one-line `& 0x03FF_FFFF` mask in `nvic_mmio_write32` and the
threaded sibling. Documented here so a future probe-diff failure can
be triaged quickly.

## TrustZone EXC_RETURN missing the S bit (2026-04-26)

`crates/mdrp2350/src/core/exceptions.rs:191` carries an in-source
FIXME(trustzone): the EXC_RETURN value pushed on exception entry does
not encode the Secure / Non-Secure (S) bit. Non-Secure exceptions
will therefore claim a Secure return on resume, mismatching the
ARMv8-M Mainline behaviour described in the master HLD §2 (TrustZone
non-goals).

Status: not yet observed because no current oracle exercises a NS
exception entry path — `silicon_isr_diff_rp2350` runs in Secure
state. Surfacing here so the next NS-state work has the gap on its
radar; promote to a sized fix when Phase 8 (TrustZone) lands.

## SysTick CLKSOURCE=0 ref-clock scaling not modelled (2026-04-26)

Two TODO sites: `crates/mdrp2350/src/bus/ppb.rs:392` (write path) and
`:772` (`systick_advance`). Both note that all cycles tick the
SysTick counter regardless of `SYST_CSR.CLKSOURCE`. Real silicon
divides the reference clock when `CLKSOURCE=0`; the current model
treats it as `CLKSOURCE=1` (processor clock).

Risk: any firmware that reprograms SysTick to use the reference clock
(rare on RP2350 in practice — bootrom and most SDK paths use the
processor clock) will see incorrect interrupt cadence. No oracle
catches this today.

Fix path: when `SYST_CSR.CLKSOURCE` becomes 0, scale the delta in
`systick_advance` by the cached `clk_ref / clk_sys` ratio from
`ClockTree`. ~10 LOC; gated on `SYST_CSR.CLKSOURCE`.

## FPSCR.AHP not honoured in f16 conversions (2026-04-26)

Two TODO(phase-7.1) sites in `crates/mdrp2350/src/core/execute_fpu.rs`:
`:1378` (`f16_bits_to_f32`) and `:1429` (`f32_to_f16_bits`). Both
ignore `FPSCR.AHP` (Alternative Half-Precision encoding). When AHP=1,
ARMv8-M uses the alternative-half-precision encoding (no Inf/NaN; max
exp encodes large normals); when AHP=0 (default), it uses IEEE 754-2008
half-precision.

Current behaviour: always IEEE 754-2008. Firmware that flips
`FPSCR.AHP` to use the alternative encoding will produce incorrect
half-float results on the boundary cases (Inf, NaN, max-exp normals).

No oracle catches this; AHP is rarely used in practice. Fix is
localised to the two functions and gated on a single FPSCR bit.

## tests_narrow.rs IO_BANK0 INTR W1C tests #[ignore]'d (2026-04-26)

`crates/mdrp2350/src/tests_narrow.rs:1005` (`s65_io_bank0_intr0_byte_write_no_fault`)
and `:1020` (`s65_io_bank0_intr5_byte_write_no_fault`) are marked
`#[ignore]` until `io_bank0` INTR W1C semantics land per HLD §4.7.
Current model is plain RW storage at `peripherals/io_bank0.rs:192-200`
("W1C is a future enhancement"); a no-fault-only test would not
distinguish correct from broken behaviour and was deemed worse than an
ignored one.

Re-enable as soon as the W1C peripheral path lands. Until then, the
narrow-write paths into IO_BANK0 INTR are tested only by the bus-side
mask, which is not equivalent to silicon W1C.

## Master clock does not advance when both cores are WFE/WFI-blocked (RP2040 + RP2350) (2026-04-26)

`crates/mdrp2040/src/lib.rs:595-625` (post-WFE/SEV wiring) and
`crates/mdrp2350/src/lib.rs:1339-1342` both use the same step-loop
shape: each core that is `is_halted() || is_wfe_waiting()` contributes
zero cycles to the quantum. When both cores are blocked the loop body
hits `if c0 == 0 && c1 == 0 { break; }` and the master clock does not
advance.

**The gap.** On real silicon, with both cores in WFI, a TIMER alarm
can still fire and assert IRQ on the NVIC; the wake propagates via
`wake_checks` → `pending_and_enabled() != 0` → `cores[c].wake()`,
unblocking the WFI'd core for the next quantum. In the emulator that
chain is intact for the *wake* — `wake_checks` runs at the quantum
tail unconditionally — but the *trigger* is missing: lazy peripherals
(TIMER alarms) advance through `Bus::advance_lazy_scheduled(consumed)`
where `consumed` is the per-quantum delta. Both-cores-blocked → zero
delta → no alarm tick → no IRQ assert → no wake.

**Same shape on mdrp2350.** The `step_pair_arm` skip predicate at
`crates/mdrp2350/src/lib.rs:1339-1342` exhibits the same behaviour:
`while !cs[core_id].is_halted() && !cs[core_id].is_wfe_waiting() && cs[core_id].cycles < target`
guarantees the cycle counter doesn't advance, and the higher-level
clock counter follows the cores. SysTick on M33 runs off the same
master cycle — same shape, same gap.

**Risk classification.** Theoretical. No current corpus scenario
exercises a "both cores enter WFI together, expect TIMER alarm to
wake one" pattern. The Pico SDK's `multicore_launch_core1` waits
core 1 with WFE on a SIO event (covered by the FIFO-rx event wake
path). The closest real-firmware shape would be a power-management
RTOS that idles both cores into WFI between time-slices and relies
on a TIMER tick to schedule next — but that pattern isn't in the
PicoGUS / MonkeyIsland / blinky / multicore-bench workloads we run
today.

**Resolution path when the scenario lands.** Two options:
1. Detect "both blocked + clock would otherwise stall" in the step
   loop, advance the clock by the lesser of `step_quantum` or "time
   to the next scheduled lazy event" (TIMER alarm match), then call
   `advance_lazy_scheduled` once to fire any IRQs in that window,
   then re-run `wake_checks` to unblock cores. Symmetric on both
   chips.
2. Accept the gap and document it as "WFI-blocked cores resume only
   on cross-core IRQ, not on internal-peripheral IRQ" — viable if
   a future user-space contract treats this as a feature rather
   than a bug.

Linked HLD: `wrk_docs/2026.04.26 - HLD - RP2040 WFE-SEV Wake
Mechanics V1.md` §8 Q3 (deferred per supervisor adjudication).

# Technical Debt

Items discovered during development that need addressing in later phases.

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

### Positive-control case — nop_chain_8 FAIL

Case: 8× Thumb NOP (`0xBF00`) inside the BLX/BX LR frame. At
per-iter=8 this would indicate HW and EMU agree on "1-cycle NOP
plus frame overhead that matches". They do NOT agree.

HW per-iter: 11    EMU per-iter: 16    delta: −5

Measured as BLX / 8×NOP / BX LR round-trip in a steady-state loop
— NOT directly comparable to the halt-step per-instruction
entries above.

Interpretation options (unresolved — flagged for Arthur):

1. Emulator's pipeline model is pessimistic on the per-iter
   framing overhead: BLX (emu 2) + BX LR (emu 2) + SUBS (1) +
   BNE-taken (emu 3) = 8 cycles of overhead, on top of 8 cycles of
   NOPs, matches the observed EMU=16. Silicon folds some of this
   into the pipeline (likely branch prediction on BX LR and the
   BNE) and observes 11.
2. The oracle's per-iter definition (BLX round-trip included) does
   not isolate "NOP cost" — the author's expectation that HW==EMU
   at per-iter=8 silently assumed zero framing overhead, which
   does not match either side.

Either way, **do not infer from this that every delta below is
"5 cycles of bias" and subtract it**. The framing overhead is real
on both sides and genuinely models differently in HW vs EMU; this
is an emulator-fidelity finding, not an oracle-calibration finding.

### Sequence-in-loop deltas per case

All nine cases at tol=0. HW is the silicon measurement; EMU is the
mdrp2350 cycle model's measurement through the same stub. `measured
as BLX / seq / BX LR round-trip in a steady-state loop — NOT
directly comparable to the halt-step per-instruction entries above`
applies to every row.

| case                             | HW/iter | EMU/iter | delta |
|----------------------------------|--------:|---------:|------:|
| nop_chain_8                      |      11 |       16 |    −5 |
| push_2_min_cost                  |      10 |       13 |    −3 |
| backward_branch_small            |      13 |       11 |    +2 |
| backward_branch_large            |      13 |       14 |    −1 |
| bank_contention_fetch_data_same  |       9 |       11 |    −2 |
| bank_contention_fetch_data_diff  |       9 |       11 |    −2 |
| ldm_8_reg                        |      17 |       18 |    −1 |
| single_adds                      |       7 |        7 |     0 |
| back_to_back_alu                 |      14 |       16 |    −2 |

Notable observations:

- `single_adds` is the only PASS at tol=0. Whatever the emulator's
  per-iter framing cost is, it matches silicon exactly for the
  1-instruction case. That constrains option (1) above: the framing
  overhead is the *same* on HW and EMU for a minimal sequence body,
  and the HW−EMU divergence in other cases grows with the specific
  instructions in the body.
- `bank_contention_fetch_data_same` (bank 0 fetch vs bank 0 data)
  and `bank_contention_fetch_data_diff` (bank 0 fetch vs bank 1
  data) measure *identically* on both HW (9) and EMU (11). No
  observable bank-contention signal in this measurement mode.
  This does not invalidate the halt-step "SRAM Bank Contention"
  entry above — that measures the contention on a single instruction
  in isolation, and the effect may be masked at sequence-in-loop
  scale by other pipeline effects. Treat the halt-step entry as
  authoritative for its own context.
- `backward_branch_large` (302-byte span) and `backward_branch_small`
  (22-byte span) measure *identically* on HW (13 each). No
  observable large-branch penalty in this measurement mode.
  Again, this does not invalidate the halt-step "Backward Branch
  Pipeline Penalty" entry above; that was measured at isolation,
  not in a tight loop where branch target buffering has warmed up.

### What this does not do

- Does not close any of the halt-step entries above. Sequence-in-
  loop and halt-step are disjoint measurement modalities; closing a
  halt-step entry requires a halt-step measurement confirming the
  emulator matches silicon in that mode.
- Does not confirm or refute individual instruction cycle costs. It
  diffs *bundle* costs only.
- Does not resolve the `nop_chain_8` positive-control failure.
  Flagged for Arthur.

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

- **Cycle timing residuals (3 cases)**: `push_2_min_cost` (delta=-2),
  `bank_contention_fetch_data_same` and `_diff` (delta=-1 each).  All
  are pipeline-overlap effects: M33 store-buffer drain overlapping with
  POP loads (PUSH case), and load-use latency hiding where LDR costs 1
  when the next instruction doesn't consume the loaded register (bank
  contention cases).  Requires register-dependency tracking between
  consecutive instructions — Phase 2 pipeline model work.

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

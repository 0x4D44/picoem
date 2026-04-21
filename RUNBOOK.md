# RUNBOOK

Operator recipes for the mdpicoem repo. Pairs with `tech_debt.md` (which
tracks bugs), `CLAUDE.md` (which tracks conventions), and the HLDs under
`wrk_docs/`.

## Killing a hung oracle process tree on Windows (Git Bash)

### Symptom

- Zombie `qemu-system-arm.exe` (or `probe_diff_*`) bound to port 3333 or
  3334 after a fuzz batch dies abnormally.
- `taskkill //f //pid <WINPID>` hangs for more than 30 s with no output.
- `kill -9 <WINPID>` returns `kill: (<WINPID>) - No such process`.
- New oracles spawned by the bash loop connect to the dead QEMU and then
  fail with `fatal: A connection attempt failed ... (os error 10060)`.
- Bash loop re-spawns immediately, every batch dies in ~1 s — a hot
  failure loop.

### Root cause

- Git Bash `kill` is the POSIX tool from `procps-ng`; it expects POSIX
  PIDs, not Windows WINPIDs. Passing a WINPID gives "No such process".
- `taskkill` and `powershell Stop-Process` traverse the Windows process
  table via WMI; under AV / Defender contention or heavy system load
  they can stall indefinitely instead of failing fast.

### Recipe

1. Locate the target's POSIX PID and WINPID. The first column in
   `ps -W` is the POSIX PID; `WINPID` is a separate column:

   ```bash
   ps -W | grep <exe-name>          # e.g. qemu_diff_m0plus.exe
   netstat -ano | grep ':3334 '     # cross-check via the listening port
   ```

2. Walk the PPID tree upward from the target to the owning bash loop.
   Two or three hops are typical (oracle -> inner bash -> outer bash):

   ```bash
   ps -W | awk 'NR==1 || $1==<TARGET_POSIX_PID> || $2==<TARGET_POSIX_PID>'
   # take the PPID from the matching row, repeat until you reach the
   # outermost bash that owns the loop.
   ```

3. Kill all collected POSIX PIDs in one invocation (not WINPIDs):

   ```bash
   kill -9 <POSIX_PID_outer_bash> <POSIX_PID_inner_bash> <POSIX_PID_oracle>
   ```

4. Verify the tree is gone:

   ```bash
   ps -W | grep <exe-name>          # expect no output
   netstat -ano | grep ':3334 '     # expect no LISTEN on the port
   ```

### Worked example

The 2026-04-14 06:09 incident, lifted from the campaign journal
(`wrk_journals/2026.04.14 - JRN - Overnight QEMU Fuzz Campaign.md`,
sections "00:05–00:15" and "06:09"):

- Target: `probe_diff_rp2350.exe` POSIX PID **114565**, started
  06:08:47 by the run-probe loop.
- `taskkill /F /IM probe_diff_rp2350.exe` had already timed out.
- `ps -W | awk` walked the PPID chain: oracle 114565 -> inner bash
  **103703** -> outer bash **103698**.
- `kill -9 114565 103703 103698` terminated the entire tree on the
  first try. `ps -W` confirmed empty.

The same sequence is the only thing that worked during the earlier
"00:05–00:15 — Cleanup attempts failed on Windows" debugging window,
where `TaskStop` returned success but left orphaned children, and
`kill -9 <WINPID>`, two flavours of `taskkill`, and
`powershell Stop-Process` all hung or returned "No such process".

### When to reach for it

Decision tree, gentlest first — **do not start with `kill -9`**:

1. Stop the bash loop via its task handle (`TaskStop <id>`) or Ctrl-C
   if you launched it interactively.
2. Try `taskkill //f //pid <WINPID>` once with a short timeout (≤ 30 s).
3. **Only if** step 1 leaves orphaned children **or** step 2 hangs,
   fall back to this recipe.

The recipe is a fallback. Skipping the gentler steps loses the chance
to observe `trap` handlers and any clean-shutdown behaviour the oracle
or driver might run.

### Scope and limits

- **Sanity check first:** `ps -W | head -1` must show both `WINPID`
  and `PPID` columns. If the header is missing either, your `ps`
  flavour is not the Git-Bash / MSYS2 `procps-ng` build the recipe
  assumes — stop and reassess.
- **Git Bash / MSYS2 only.** WSL's `ps` is Linux-native and does not
  expose a `WINPID` column at all. From a WSL prompt, drop to
  PowerShell or `cmd.exe` and use `taskkill /F /IM <exe>` instead.
- **If Defender is actively scanning the target `.exe`**, both
  `taskkill` and `kill -9` may stall until the scan completes. Wait,
  don't hammer the system with retries.
- This recipe terminates a tree that has already gone wrong; it does
  **not** address why the zombies appeared. The real fix for the
  source bug is Agent A's child-process cleanup —
  see `wrk_docs/2026.04.15 - HLD - QEMU Child Cleanup on Exit.md`.

## Putting RP2354 into RISC-V mode

### Background

RP2354 ships two complete CPU complexes (2× Cortex-M33, 2× Hazard3
RISC-V). Only one runs at a time. The selection — ARCHSEL — is latched
at reset from two inputs, evaluated in order:

1. **POWMAN `CHIP_RESET.ARCH_SEL`** soft-override in sticky always-on
   storage — revertible on next reset if written again.
2. **`CHIP_INFO.ARCH_SEL`** OTP fuse — one-way, permanent once burned.

Default silicon (no OTP burn, no soft-override) boots into Arm.

For spike and oracle runs where we want to flip between Arm and RV
without reburning fuses, use the POWMAN soft-override.

### Mechanism 1 — POWMAN soft-override (preferred for spike runs)

Revertible; persists across soft reset via always-on storage but is
cleared on a fresh cold power-up. Sequence:

1. Attach via SWD with the probe in its current arch (Arm usually).
2. Write `CHIP_RESET.ARCH_SEL` via the POWMAN register window.
   - **ASSUMPTION — not yet pinned from datasheet.** `POWMAN_BASE` and
     the `CHIP_RESET` offset *must* be pinned from RP2350 datasheet
     §5.10 before `riscv_probe_spike --attempt-archsel-flip` attempts
     a write. The emulator's current values are **assumptions**:
     `POWMAN_BASE = 0x4010_0000`, `CHIP_RESET` offset `0x20`, both
     explicitly flagged ASSUMPTION in
     `crates/mdrp2350/src/peripherals/powman.rs` module doc (offset
     `0x20` is an educated guess inside the 4 KB POWMAN aperture; the
     emulator's storage model round-trips regardless of whether the
     offset matches silicon exactly, so the in-tree offset itself is
     not evidence of the datasheet value).
   - Until the datasheet pin lands here, Row A2 of the Phase 1 spike
     (`--attempt-archsel-flip`) will SKIP with the reason "A2 skipped:
     POWMAN CHIP_RESET offset not pinned in RUNBOOK" even when the
     flag is passed. Row A1 (read-only probe) runs unconditionally and
     prints whatever is at `POWMAN_BASE + 0x20` — that output is a
     starting data point for triaging the real offset, not a
     confirmation that `0x20` is correct.
   - **Do not blindly adopt other offsets proposed in review without
     primary-source evidence.** Another reviewer has asserted the
     datasheet value is `0x08`; until that is confirmed against the
     actual RP2350 datasheet §5.10 (or an alternative primary source
     such as `one-rom/sdrr/include/reg-rp235x.h`), neither `0x20` nor
     `0x08` should be trusted on silicon.
3. Issue a reset (e.g. `probe-rs reset`, or re-cycle nRST).
4. Re-attach. Next boot comes up in the selected arch.

#### Pinning the offset (procedure)

1. Locate RP2350 datasheet §5.10 (POWMAN register map) or
   `one-rom/sdrr/include/reg-rp235x.h` if available.
2. Read off `POWMAN_BASE` (expect `0x4010_0000`; confirm) and the
   `CHIP_RESET` register offset.
3. Update this section to replace the ASSUMPTION block with a pinned
   value, citing the source.
4. Update `POWMAN_OFFSET_PINNED` in
   `crates/mdpicoem-harness/src/bin/riscv_probe_spike.rs` from
   `false` to `true` and fix the `POWMAN_BASE` / `CHIP_RESET_OFFSET`
   constants to match.
5. Update the emulator's `crates/mdrp2350/src/peripherals/powman.rs`
   module doc to drop the ASSUMPTION flag on the matching offset (if
   the datasheet value equals the emulator's current guess) or to
   correct the guess (if not — will require a storage-model audit,
   but the round-trip tests should remain green regardless).

### Mechanism 2 — OTP fuse burn (permanent)

Burn `CHIP_INFO.ARCH_SEL` via `picotool otp set-default-boot ...`
(exact sub-command — see picotool help). One-way: once burned you
cannot un-burn, though a subsequent POWMAN soft-override in the
opposite direction still works.

Use only if:

- The board is dedicated to one arch for the foreseeable future.
- The OTP-fuse escape path (opposite-direction POWMAN override) is
  documented for the operator who will later revive the board.

### Checking current mode

After attach:

- `probe-rs list` shows the probe but **does not** report current
  CPU arch — the probe YAML is ARM-only and probe-rs routes through
  the Arm debug sequence regardless of silicon state.
- Run `riscv_probe_spike` (the Phase 1 spike binary in
  `crates/mdpicoem-harness/src/bin/riscv_probe_spike.rs`). It
  calls `Core::architecture()` and reads `mhartid`; the summary row
  "1. Attach + RV core enumeration" tells you which arch the silicon
  booted into. PASS = RV, FAIL with "architectures = [Arm, Arm]" =
  still Arm.

### Notes and caveats

- A cold power cycle (full power removal, not just nRST) can clear
  the POWMAN soft-override depending on the board's always-on rail.
  If the override unexpectedly reverts, check whether the board cut
  AON power during the "reset".
- probe-rs 0.31's embedded `RP235x.yaml` has no RV core stanza. Even
  when silicon is in RV mode, probe-rs hands out ARM `Core`s and
  `architecture()` may still report `Arm` — this is the probe-rs
  side of the gap, not a silicon bug. See
  `wrk_docs/2026.04.17 - LLD - RISC-V Probe-rs Attach Spike V1.md`
  §2 for the implication.
- Two-probe + two-board setups can run Arm and RV in parallel; the
  POWMAN override is a one-board recipe.

## probe-rs patch notes

### Patched path

`third_party/probe-rs-0.31.0-mdrp-patched/` — a verbatim copy of
probe-rs 0.31.0 (MIT OR Apache-2.0) with one surgical change in
`src/architecture/arm/communication_interface.rs::select_ap_and_ap_bank`.
The workspace `Cargo.toml` pins it via `[patch.crates-io]`. Harness code
is unchanged.

### What the patch does

Adds a new match arm for `(ApAddress::V2, SelectCache::DPv1)` that
upgrades the DP state cache to `DPv3` in place and performs the
ADIv6 V2 register write, instead of hitting the upstream
`unreachable!()`. This recovers RP2354 silicon oracles from a
first-DPIDR-read version glitch — see upstream issues
[#3872](https://github.com/probe-rs/probe-rs/issues/3872) and
[#3257](https://github.com/probe-rs/probe-rs/issues/3257), and
`wrk_docs/2026.04.21 - HLD - Track A Probe-rs Attach Fix.md`.

### Sentinel WARN to watch for

```
WARN ... ApV2 access on DPv1 cache; upgrading to DPv3 (mdrp-patched workaround)
```

Expected: **at most once per session** when the initial DPIDR read
mis-reports the DP version. If you see this line firing repeatedly
within a single oracle run, the patch is masking a deeper DP state
bug — stop and escalate rather than ignoring it.

### When bumping probe-rs

1. Check whether the new release contains a fix for issue #3872
   (the `unreachable!()` at `select_ap_and_ap_bank`).
2. If fixed upstream: drop `third_party/probe-rs-0.31.0-mdrp-patched/`
   and the `[patch.crates-io]` entry, then rerun the silicon oracle
   catalogue (`probe_verify_rp2350`, `silicon_periph_diff_rp2350`,
   `test_silicon`) to confirm the panic no longer reproduces.
3. If not fixed upstream: re-vendor at the new version, re-apply the
   patch to `select_ap_and_ap_bank`, and smoke-test the same
   oracles. Keep the sentinel WARN text identical so dashboards that
   grep for it keep working.

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
where `TaskStop`, `kill -9 <WINPID>`, two flavours of `taskkill`, and
`powershell Stop-Process` had all hung or no-op'd.

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

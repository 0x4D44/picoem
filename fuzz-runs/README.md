<!-- Updated 2026-04-15; contents reflect fuzz-runs/ state post-reconciliation -->
# fuzz-runs/

Drivers and logs for the overnight QEMU and probe-rs differential fuzz
campaigns. See `CLAUDE.md` ("Differential Fuzz Testing (QEMU harness)")
for how to launch individual batches.

## Contents

- `run-m33.sh` — RP2350 / Cortex-M33 oracle (port 3333)
- `run-m0plus.sh` — RP2040 / Cortex-M0+ oracle (port 3334)
- `run-probe.sh` — RP2354 silicon oracle (via Pico probe)
- `deadline` — epoch-seconds end time, read once at driver start
- `*.log` — per-oracle batch output

## If the loop hangs

See `RUNBOOK.md` for the Windows Git-Bash `kill -9 <POSIX_PID>` recipe
when `taskkill` won't terminate a zombie process tree.

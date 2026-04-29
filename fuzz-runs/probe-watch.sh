#!/bin/bash
# Probe-availability watcher. Both probes' WinUSB endpoints wedged at
# launch time on 2026-04-28; this watcher polls every 15 min and, if a
# probe responds, kicks off its silicon driver for the rest of the
# 24h window. Coexists with the QEMU oracles already running on this
# host — silicon drivers ride alongside, do not replace.
#
# Logs to fuzz-runs/probe-watch.log.

LOG="fuzz-runs/probe-watch.log"
: > "$LOG"

DEADLINE="$(cat fuzz-runs/deadline 2>/dev/null)"
if [ -z "$DEADLINE" ]; then
  DEADLINE=$(( $(date +%s) + 28800 ))
fi
echo "=== probe-watch started $(date -Iseconds), deadline $(date -d @$DEADLINE -Iseconds) ===" >> "$LOG"

RP2354_PROBE="2e8a:000c:E46410955F614129"
RP2040_PROBE="2e8a:000c:E46410955F3C5C27"

# Track whether each silicon driver is already running so we don't
# double-launch on a flaky reattach.
RP2354_LAUNCHED=0
RP2040_LAUNCHED=0

probe_alive() {
  local sel="$1" chip="$2"
  # `probe-rs info` returns rc=0 even when both JTAG and SWD probing
  # fail — it considers "I tried and reported the error" a success.
  # So we capture the output and grep for the wedge fingerprint.
  local out
  out="$(timeout 15 probe-rs info --probe "$sel" --chip "$chip" 2>&1)"
  if printf "%s" "$out" | grep -qE "Could not determine a suitable packet size|Failed to open the debug probe|An error which is specific to the debug probe"; then
    return 1
  fi
  # Sanity check: a healthy attach prints "ARM Chip" or "Found ARM CPU"
  # or similar real chip-info output. If we see neither the error
  # signature nor a plausible chip line, treat as wedged-ish.
  if printf "%s" "$out" | grep -qE "ARM Chip|Cortex-|Hazard3|RISC-V|RP2040|RP235|Found"; then
    return 0
  fi
  # Unknown state — assume wedged to avoid spurious launches.
  return 1
}

silicon_driver_running() {
  local pattern="$1"
  ps -W 2>/dev/null | grep -q "$pattern"
  return $?
}

while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  NOW=$(date -Iseconds)
  echo "" >> "$LOG"
  echo "[$NOW] poll" >> "$LOG"

  if [ "$RP2354_LAUNCHED" -eq 0 ]; then
    if probe_alive "$RP2354_PROBE" "RP235x"; then
      echo "[$NOW] RP2354 probe responsive — launching test_silicon driver" >> "$LOG"
      nohup ./fuzz-runs/run-test-silicon.sh "$RP2354_PROBE" >/dev/null 2>&1 &
      RP2354_LAUNCHED=1
      echo "[$NOW] test_silicon driver PID $!" >> "$LOG"
    else
      echo "[$NOW] RP2354 probe still wedged" >> "$LOG"
    fi
  else
    # Verify the wrapper bash is still alive; if it died, allow relaunch.
    if ! silicon_driver_running "run-test-silicon"; then
      echo "[$NOW] run-test-silicon wrapper exited — clearing flag for next poll" >> "$LOG"
      RP2354_LAUNCHED=0
    fi
  fi

  if [ "$RP2040_LAUNCHED" -eq 0 ]; then
    if probe_alive "$RP2040_PROBE" "RP2040"; then
      echo "[$NOW] RP2040 probe responsive — launching probe_diff_rp2040 driver" >> "$LOG"
      nohup ./fuzz-runs/run-m0plus-probe.sh "$RP2040_PROBE" >/dev/null 2>&1 &
      RP2040_LAUNCHED=1
      echo "[$NOW] run-m0plus-probe driver PID $!" >> "$LOG"
    else
      echo "[$NOW] RP2040 probe still wedged" >> "$LOG"
    fi
  else
    if ! silicon_driver_running "run-m0plus-probe"; then
      echo "[$NOW] run-m0plus-probe wrapper exited — clearing flag for next poll" >> "$LOG"
      RP2040_LAUNCHED=0
    fi
  fi

  # 15 min between polls. Don't sleep past the deadline.
  REMAINING=$(( DEADLINE - $(date +%s) ))
  if [ "$REMAINING" -le 0 ]; then
    break
  fi
  SLEEP_FOR=900
  if [ "$REMAINING" -lt "$SLEEP_FOR" ]; then
    SLEEP_FOR="$REMAINING"
  fi
  sleep "$SLEEP_FOR"
done

echo "=== probe-watch ending at $(date -Iseconds), deadline reached ===" >> "$LOG"

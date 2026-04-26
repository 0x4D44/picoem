#!/bin/bash
# Overnight hardware probe fuzz driver against real RP2354 silicon.
# No --cycles flag: known cycle mismatches (bank contention, backward branch,
# PUSH minimum cost) are catalogued in tech_debt.md and would be noise.
#
# Optional argument $1: probe selector (VID:PID:SERIAL) for when two probes
# are attached. When omitted, auto_attach picks the only visible probe.
LOG="fuzz-runs/probe.log"
BATCH=0
: > "$LOG"
# Read deadline once at start. If the file vanishes mid-run we still
# keep going for the full window we captured here.
DEADLINE="$(cat "fuzz-runs/deadline" 2>/dev/null)"
if [ -z "$DEADLINE" ]; then
  DEADLINE=$(( $(date +%s) + 28800 ))
fi
PROBE_ARGS=()
if [ -n "$1" ]; then
  PROBE_ARGS=(--probe "$1")
fi
echo "=== PROBE driver deadline=$DEADLINE ($(date -d @$DEADLINE -Iseconds)) ===" >> "$LOG"
# Run from a PID-qualified copy so concurrent cargo builds can still
# overwrite target/release/probe_diff_rp2350.exe while we fuzz (Windows
# holds an exclusive lock on a running .exe).
BIN="fuzz-runs/probe_diff_rp2350.$$.exe"
cp -f target/release/probe_diff_rp2350.exe "$BIN" || {
  echo "=== FATAL: target/release/probe_diff_rp2350.exe missing or unreadable ===" >> "$LOG"
  exit 1
}
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== PROBE batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" \
    --fuzz 200 --seed "$SEED" "${PROBE_ARGS[@]}" >> "$LOG" 2>&1
  rc=$?
  echo "=== PROBE batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
  # rc=2 means probe creation failed — almost always a transient WinUSB
  # endpoint-busy state after a USB blip. Retrying instantly burns dozens
  # of seeds per minute through dead attaches, so back off and let the
  # endpoint clear before the next attempt.
  if [ "$rc" -eq 2 ]; then
    echo "=== PROBE rc=2 detected; backing off 30s before next batch ===" >> "$LOG"
    sleep 30
  fi
done
echo "=== PROBE deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

#!/bin/bash
# Overnight hardware probe fuzz driver against real RP2354 silicon.
# No --cycles flag: known cycle mismatches (bank contention, backward branch,
# PUSH minimum cost) are catalogued in tech_debt.md and would be noise.
LOG="fuzz-runs/probe.log"
DEADLINE_FILE="fuzz-runs/deadline"
BATCH=0
: > "$LOG"
# Read deadline once at start. If the file vanishes mid-run we still
# keep going for the full window we captured here.
DEADLINE="$(cat "$DEADLINE_FILE" 2>/dev/null)"
if [ -z "$DEADLINE" ]; then
  DEADLINE=$(( $(date +%s) + 28800 ))
fi
echo "=== PROBE driver deadline=$DEADLINE ($(date -d @$DEADLINE -Iseconds)) ===" >> "$LOG"
# Run from a PID-qualified copy so concurrent cargo builds can still
# overwrite target/release/probe_diff_rp2350.exe while we fuzz (Windows
# holds an exclusive lock on a running .exe).
BIN="fuzz-runs/probe_diff_rp2350.$$.exe"
cp -f target/release/probe_diff_rp2350.exe "$BIN"
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== PROBE batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" \
    --fuzz 200 --seed "$SEED" >> "$LOG" 2>&1
  rc=$?
  echo "=== PROBE batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done
echo "=== PROBE deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

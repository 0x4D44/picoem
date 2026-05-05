#!/bin/bash
# Overnight M33 fuzz driver. Rotates seeds per batch, writes to m33.log.
LOG="fuzz-runs/m33.log"
BATCH=0
: > "$LOG"
# Read deadline once at start. If the file vanishes mid-run we still
# keep going for the full window we captured here.
DEADLINE="$(cat "fuzz-runs/deadline" 2>/dev/null)"
if [ -z "$DEADLINE" ]; then
  DEADLINE=$(( $(date +%s) + 28800 ))
fi
echo "=== M33 driver deadline=$DEADLINE ($(date -d @$DEADLINE -Iseconds)) ===" >> "$LOG"
# Run from a PID-qualified copy so concurrent cargo builds can still
# overwrite target/release/qemu_diff_m33.exe while we fuzz (Windows
# holds an exclusive lock on a running .exe).
BIN="fuzz-runs/qemu_diff_m33.$$.exe"
cp -f target/release/qemu_diff_m33.exe "$BIN" || {
  echo "=== FATAL: target/release/qemu_diff_m33.exe missing or unreadable ===" >> "$LOG"
  exit 1
}
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  NOW=$(date +%s)
  REMAINING=$(( DEADLINE - NOW ))
  # If we have less than a minute left, don't start a new batch — a
  # full --fuzz 30000 run takes well over an hour and would overrun
  # the deadline by a large margin (mirrors run-test-silicon.sh).
  if [ "$REMAINING" -lt 60 ]; then
    echo "=== M33 only ${REMAINING}s left, skipping last batch ===" >> "$LOG"
    break
  fi
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== M33 batch=$BATCH seed=$SEED remaining=${REMAINING}s start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" \
    --fuzz 30000 --seed "$SEED" >> "$LOG" 2>&1
  rc=$?
  echo "=== M33 batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done
echo "=== M33 deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

#!/bin/bash
# Overnight M0+ fuzz driver. Rotates seeds per batch, writes to m0plus.log.
LOG="fuzz-runs/m0plus.log"
DEADLINE_FILE="fuzz-runs/deadline"
BATCH=0
: > "$LOG"
# Run from a PID-qualified copy so concurrent cargo builds can still
# overwrite target/release/qemu_diff_m0plus.exe while we fuzz (Windows
# holds an exclusive lock on a running .exe).
BIN="fuzz-runs/qemu_diff_m0plus.$$.exe"
cp -f target/release/qemu_diff_m0plus.exe "$BIN"
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$(cat "$DEADLINE_FILE")" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== M0+ batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" \
    --fuzz 50000 --seed "$SEED" >> "$LOG" 2>&1
  rc=$?
  echo "=== M0+ batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done
echo "=== M0+ deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

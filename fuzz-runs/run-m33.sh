#!/bin/bash
# Overnight M33 fuzz driver. Rotates seeds per batch, writes to m33.log.
LOG="fuzz-runs/m33.log"
DEADLINE_FILE="fuzz-runs/deadline"
BATCH=0
: > "$LOG"
# Run from a PID-qualified copy so concurrent cargo builds can still
# overwrite target/release/qemu_diff_m33.exe while we fuzz (Windows
# holds an exclusive lock on a running .exe).
BIN="fuzz-runs/qemu_diff_m33.$$.exe"
cp -f target/release/qemu_diff_m33.exe "$BIN"
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$(cat "$DEADLINE_FILE")" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== M33 batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" \
    --fuzz 30000 --seed "$SEED" >> "$LOG" 2>&1
  rc=$?
  echo "=== M33 batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done
echo "=== M33 deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

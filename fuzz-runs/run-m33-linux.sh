#!/bin/bash
# Linux-adapted overnight M33 fuzz driver. Rotates seeds per batch.
# Differs from run-m33.sh: drops `.exe`, no Windows lock-avoidance copy
# is necessary on Linux (we still PID-tag the binary for parity with the
# original so concurrent rebuilds don't clobber a running fuzzer).
set -u
LOG="fuzz-runs/m33.log"
BATCH=0
: > "$LOG"
DEADLINE="$(cat fuzz-runs/deadline 2>/dev/null)"
if [ -z "$DEADLINE" ]; then
  DEADLINE=$(( $(date +%s) + 28800 ))
fi
echo "=== M33 driver deadline=$DEADLINE ($(date -d @$DEADLINE -Iseconds)) ===" >> "$LOG"
BIN="fuzz-runs/qemu_diff_m33.$$"
cp -f target/release/qemu_diff_m33 "$BIN" || {
  echo "=== FATAL: target/release/qemu_diff_m33 missing or unreadable ===" >> "$LOG"
  exit 1
}
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== M33 batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" --fuzz 30000 --seed "$SEED" --classes=base >> "$LOG" 2>&1
  rc=$?
  echo "=== M33 batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done
echo "=== M33 deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

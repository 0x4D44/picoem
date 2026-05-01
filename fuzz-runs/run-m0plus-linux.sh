#!/bin/bash
# Linux-adapted overnight M0+ fuzz driver. See run-m33-linux.sh header.
set -u
LOG="fuzz-runs/m0plus.log"
BATCH=0
: > "$LOG"
DEADLINE="$(cat fuzz-runs/deadline 2>/dev/null)"
if [ -z "$DEADLINE" ]; then
  DEADLINE=$(( $(date +%s) + 28800 ))
fi
echo "=== M0+ driver deadline=$DEADLINE ($(date -d @$DEADLINE -Iseconds)) ===" >> "$LOG"
BIN="fuzz-runs/qemu_diff_m0plus.$$"
cp -f target/release/qemu_diff_m0plus "$BIN" || {
  echo "=== FATAL: target/release/qemu_diff_m0plus missing or unreadable ===" >> "$LOG"
  exit 1
}
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== M0+ batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" --fuzz 50000 --seed "$SEED" >> "$LOG" 2>&1
  rc=$?
  echo "=== M0+ batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done
echo "=== M0+ deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

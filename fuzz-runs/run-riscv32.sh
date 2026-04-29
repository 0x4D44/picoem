#!/bin/bash
# Overnight RISC-V Hazard3 fuzz driver. Rotates seeds per batch, writes
# to riscv32.log. Mirrors run-m33.sh / run-m0plus.sh structure.
LOG="fuzz-runs/riscv32.log"
BATCH=0
: > "$LOG"
# Read deadline once at start. If the file vanishes mid-run we still
# keep going for the full window we captured here.
DEADLINE="$(cat "fuzz-runs/deadline" 2>/dev/null)"
if [ -z "$DEADLINE" ]; then
  DEADLINE=$(( $(date +%s) + 28800 ))
fi
echo "=== RISCV32 driver deadline=$DEADLINE ($(date -d @$DEADLINE -Iseconds)) ===" >> "$LOG"
# Run from a PID-qualified copy so concurrent cargo builds can still
# overwrite target/release/qemu_diff_riscv32.exe while we fuzz (Windows
# holds an exclusive lock on a running .exe).
BIN="fuzz-runs/qemu_diff_riscv32.$$.exe"
cp -f target/release/qemu_diff_riscv32.exe "$BIN" || {
  echo "=== FATAL: target/release/qemu_diff_riscv32.exe missing or unreadable ===" >> "$LOG"
  exit 1
}
trap 'rm -f "$BIN"' EXIT
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== RISCV32 batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" \
    --fuzz 30000 --seed "$SEED" >> "$LOG" 2>&1
  rc=$?
  echo "=== RISCV32 batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done
echo "=== RISCV32 deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

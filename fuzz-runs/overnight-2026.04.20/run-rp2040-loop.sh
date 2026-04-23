#!/bin/bash
# Overnight RP2040 probe_diff loop driver for 2026-04-18 session.
# Loops test_rp2040_probe_diff --fuzz 200 with random seeds until the
# deadline passed in argv[1] (epoch seconds) is reached.
#
# Probe-selector argv[2] is a VID:PID:SERIAL string (required — two probes
# attached, auto-attach is unreliable per CLAUDE.md).
#
# Log written to argv[3]. Binary path passed as argv[4] (PID-qualified copy
# so concurrent cargo builds don't get blocked on the .exe lock).

set -u
DEADLINE="$1"
PROBE="$2"
LOG="$3"
BIN="$4"

BATCH=0
: > "$LOG"
echo "=== RP2040 driver deadline=$DEADLINE ($(date -d @"$DEADLINE" -Iseconds)) probe=$PROBE bin=$BIN ===" >> "$LOG"

while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  BATCH=$((BATCH+1))
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== RP2040 batch=$BATCH seed=$SEED start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" --fuzz 200 --seed "$SEED" --probe "$PROBE" >> "$LOG" 2>&1
  rc=$?
  echo "=== RP2040 batch=$BATCH seed=$SEED end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
done

echo "=== RP2040 deadline reached at $(date -Iseconds), batches=$BATCH ===" >> "$LOG"

#!/bin/bash
# Overnight RP2354 silicon orchestrator for 2026-04-18 session.
# Runs test_rp2350_silicon --soak for the deadline window, on the specified
# probe, writing to the log.
#
# argv[1]: deadline (epoch seconds; used to compute remaining soak duration)
# argv[2]: probe selector (VID:PID:SERIAL)
# argv[3]: log path
# argv[4]: binary path (PID-qualified copy)
# argv[5]: seed (u64)
# argv[6]: optional exclude substring (passed as --exclude); empty to skip.

set -u
DEADLINE="$1"
PROBE="$2"
LOG="$3"
BIN="$4"
SEED="$5"
EXCLUDE="${6:-}"

NOW=$(date +%s)
REMAIN=$(( DEADLINE - NOW ))
if [ "$REMAIN" -lt 60 ]; then
  echo "deadline already reached or <1min away, bailing" > "$LOG"
  exit 1
fi

: > "$LOG"
{
  echo "=== RP2354 silicon deadline=$DEADLINE ($(date -d @"$DEADLINE" -Iseconds)) ==="
  echo "=== remaining=${REMAIN}s probe=$PROBE seed=$SEED bin=$BIN ==="
  echo "=== start=$(date -Iseconds) ==="
} >> "$LOG"

# Use the remaining window in seconds; the orchestrator accepts s/m/h/d suffixes.
EXTRA=()
if [ -n "$EXCLUDE" ]; then
  EXTRA+=(--exclude "$EXCLUDE")
fi
"$BIN" --soak "${REMAIN}s" --seed "$SEED" --probe "$PROBE" "${EXTRA[@]}" >> "$LOG" 2>&1
rc=$?

echo "=== RP2354 silicon end=$(date -Iseconds) rc=$rc ===" >> "$LOG"

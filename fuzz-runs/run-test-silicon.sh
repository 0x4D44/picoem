#!/bin/bash
# Overnight test_silicon orchestrator soak against real RP2354 silicon.
# Wraps probe_diff/cycle/periph/dualcore/isr under one shared probe
# session with per-iteration Fisher-Yates case shuffle, hourly heartbeat,
# per-case 60s watchdog with drop-and-reattach. Designed for unattended
# multi-day runs.
#
# Reads fuzz-runs/deadline (epoch seconds) and runs --soak for the
# remaining window. If the orchestrator exits early, the outer loop
# restarts it (with a fresh seed) until the deadline is reached.
#
# Optional argument $1: probe selector (VID:PID:SERIAL) for when two
# probes are attached. When omitted, auto_attach picks the only
# visible probe.
LOG="fuzz-runs/test-silicon.log"
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

echo "=== TEST_SILICON driver deadline=$DEADLINE ($(date -d @$DEADLINE -Iseconds)) ===" >> "$LOG"

# PID-qualified copy so concurrent cargo builds can still overwrite
# target/release/test_silicon.exe while we soak (Windows holds an
# exclusive lock on a running .exe).
BIN="fuzz-runs/test_silicon.$$.exe"
cp -f target/release/test_silicon.exe "$BIN" || {
  echo "=== FATAL: target/release/test_silicon.exe missing or unreadable ===" >> "$LOG"
  exit 1
}
trap 'rm -f "$BIN"' EXIT

ATTEMPT=0
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  ATTEMPT=$((ATTEMPT+1))
  NOW=$(date +%s)
  REMAINING=$(( DEADLINE - NOW ))
  # If we have less than a minute left, don't bother spinning up
  # another orchestrator instance — the case-startup cost would
  # dominate and the iteration would not produce meaningful coverage.
  if [ "$REMAINING" -lt 60 ]; then
    echo "=== TEST_SILICON only ${REMAINING}s left, skipping last restart ===" >> "$LOG"
    break
  fi
  SEED="$RANDOM$RANDOM$RANDOM"
  {
    echo ""
    echo "=== TEST_SILICON attempt=$ATTEMPT seed=$SEED remaining=${REMAINING}s start=$(date -Iseconds) ==="
  } >> "$LOG"
  "$BIN" \
    --soak "${REMAINING}s" \
    --seed "$SEED" \
    "${PROBE_ARGS[@]}" >> "$LOG" 2>&1
  rc=$?
  echo "=== TEST_SILICON attempt=$ATTEMPT end=$(date -Iseconds) rc=$rc ===" >> "$LOG"
  # If the orchestrator returns non-zero before the deadline (panic,
  # probe disappeared, etc.), back off briefly to let the USB stack
  # settle before the next attempt.
  if [ "$(date +%s)" -lt "$DEADLINE" ]; then
    echo "=== TEST_SILICON exited early; sleeping 30s before restart ===" >> "$LOG"
    sleep 30
  fi
done
echo "=== TEST_SILICON deadline reached at $(date -Iseconds), attempts=$ATTEMPT ===" >> "$LOG"

#!/usr/bin/env bash
# Replay the Monkey Island trace against the rebuilt PicoGUS firmware
# with the knobs that actually make the PIO capture all trace events.
#
# Background: the stock trace has same-ns clusters of tens of thousands
# of ISA writes. The default `WRITE_IDLE_CYCLES=12` between events is
# too tight for the emulated PIO RX FIFO (4 deep) + firmware to keep
# up; events drop. `PICOGUS_IDLE_CYCLES=5000` paces each event at
# ~13.5 µs of sim-time which is enough for firmware to drain.
#
# Firmware comes from wrk_scratch/picogus-rebuild/pg-gus.bin — rebuilt
# with `pico_enable_stdio_uart(target 1)` so the UART TX tap picks up
# firmware `printf`/`puts`. The `test_psram` function address is
# `0x2001_309C` in this build (stock firmware is `0x2001_2FA4`).

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

FLASH="${FLASH:-$REPO_ROOT/wrk_scratch/picogus-rebuild/pg-gus.bin}"
TRACE="${TRACE:-$REPO_ROOT/crates/mdpicoem-harness/fixtures/monkey_island_theme.trace}"
OUT_WAV="${OUT_WAV:-/tmp/picogus_replay.wav}"
PRE_ROLL="${PRE_ROLL:-5.0}"
DURATION="${DURATION:-0.5}"
POST_ROLL="${POST_ROLL:-0.3}"

export PICOGUS_STUB_TEST_PSRAM="${PICOGUS_STUB_TEST_PSRAM:-0x2001309c}"
export PICOGUS_IDLE_CYCLES="${PICOGUS_IDLE_CYCLES:-5000}"
export PICOGUS_BACKPRESSURE_THRESHOLD="${PICOGUS_BACKPRESSURE_THRESHOLD:-0}"
export PICOGUS_PROBE="${PICOGUS_PROBE:-0x2001d976}"

echo "firmware:    $FLASH"
echo "trace:       $TRACE"
echo "out wav:     $OUT_WAV"
echo "pre-roll:    $PRE_ROLL s"
echo "duration:    $DURATION s"
echo "post-roll:   $POST_ROLL s"
echo "idle cycles: $PICOGUS_IDLE_CYCLES"
echo

cd "$REPO_ROOT"
cargo run -p mdpicoem-harness --release --bin picogus_diff_rp2040 -- \
    --flash "$FLASH" \
    --trace "$TRACE" \
    --pre-roll "$PRE_ROLL" \
    --duration "$DURATION" \
    --post-roll "$POST_ROLL" \
    --out "$OUT_WAV"

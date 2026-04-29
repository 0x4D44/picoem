#!/usr/bin/env bash
# V2 oracle-runner launcher.
#
# Usage:
#   ./scripts/v2_mutation_run.sh first-pass  [extra args]
#   ./scripts/v2_mutation_run.sh deep-pass   [extra args]
#   ./scripts/v2_mutation_run.sh status      # peek at progress
#
# first-pass:
#   Reads mutation/v2/sweep/mutants.out/missed.txt (from cargo-mutants),
#   runs each mutant through the appropriate oracle at --fuzz 200, appends
#   to mutation/v2/results.jsonl. --skip-done makes this restart-friendly.
#
# deep-pass:
#   Re-tests the oracle_survived rows at --fuzz 2000. Drops the survivor
#   rows from results.jsonl first (so re-tests overwrite, don't duplicate).
#
# status:
#   Quick summary of cargo-mutants sweep + V2 runner progress so far.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

V2_DIR="mutation/v2"
SWEEP_DIR="$V2_DIR/sweep/mutants.out"

case "${1:-}" in
    first-pass)
        shift
        if [ ! -f "$SWEEP_DIR/missed.txt" ]; then
            echo "missing $SWEEP_DIR/missed.txt — run cargo-mutants first" >&2
            exit 1
        fi
        echo "V2 first-pass: $(wc -l < "$SWEEP_DIR/missed.txt") missed mutants"
        echo "starting at $(date -Iseconds)"
        nohup setsid python3 scripts/v2_mutation_runner.py \
            --missed "$SWEEP_DIR/missed.txt" \
            --skip-done \
            --fuzz 200 \
            --timeout 180 \
            "$@" \
            > "$V2_DIR/v2_run.log" 2>&1 < /dev/null &
        disown
        echo "$!" > "$V2_DIR/v2_pid.txt"
        echo "launched V2 first-pass pid=$!"
        ;;

    deep-pass)
        shift
        echo "V2 deep-pass: re-testing oracle_survived rows at --fuzz 2000"
        echo "starting at $(date -Iseconds)"
        nohup setsid python3 scripts/v2_mutation_runner.py \
            --retry-survivors \
            --fuzz 2000 \
            --timeout 600 \
            "$@" \
            > "$V2_DIR/v2_deep.log" 2>&1 < /dev/null &
        disown
        echo "$!" > "$V2_DIR/v2_pid.txt"
        echo "launched V2 deep-pass pid=$!"
        ;;

    status)
        echo "=== cargo-mutants sweep ==="
        if [ -d "$SWEEP_DIR" ]; then
            for f in caught.txt missed.txt timeout.txt unviable.txt; do
                if [ -f "$SWEEP_DIR/$f" ]; then
                    printf "  %-15s %d\n" "$f" "$(wc -l < "$SWEEP_DIR/$f")"
                fi
            done
            if pgrep -f "/cargo-mutants" >/dev/null 2>&1; then
                echo "  STATUS: running"
            else
                echo "  STATUS: not running"
            fi
        else
            echo "  (no sweep dir)"
        fi
        echo
        echo "=== V2 runner ==="
        if [ -f "$V2_DIR/results.jsonl" ]; then
            n=$(wc -l < "$V2_DIR/results.jsonl")
            echo "  results: $n records"
            python3 scripts/v2_mutation_summary.py 2>/dev/null | tail -n +2 | head -25
        else
            echo "  (no results yet)"
        fi
        if [ -f "$V2_DIR/v2_pid.txt" ]; then
            pid=$(cat "$V2_DIR/v2_pid.txt")
            if kill -0 "$pid" 2>/dev/null; then
                echo "  STATUS: running (pid=$pid)"
            else
                echo "  STATUS: not running"
            fi
        fi
        ;;

    *)
        echo "usage: $0 {first-pass|deep-pass|status} [args...]" >&2
        exit 2
        ;;
esac

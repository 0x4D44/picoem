#!/usr/bin/env bash
# V2 oracle-runner — N-way parallel launcher via git worktrees.
#
# Usage:
#   ./scripts/v2_mutation_run_parallel.sh start [N=4] [extra-runner-args...]
#   ./scripts/v2_mutation_run_parallel.sh status
#   ./scripts/v2_mutation_run_parallel.sh wait
#   ./scripts/v2_mutation_run_parallel.sh merge
#   ./scripts/v2_mutation_run_parallel.sh stop
#   ./scripts/v2_mutation_run_parallel.sh clean
#
# Why: sequential V2 on 795 mutants @ ~50 s each is ~11 h. Splitting into N
# git worktrees (each a fully isolated checkout with its own target/) and
# running N V2 instances in parallel cuts wall-clock by ~Nx, bounded by
# host CPU/IO contention. On this 28-core/62-GB host, N=4 is a comfortable
# operating point.
#
# Layout:
#   /tmp/v2_workspaces/w<i>/             — git worktree for worker i
#   mutation/v2/parallel/missed.shard.<i> — per-worker mutant list
#   mutation/v2/parallel/results.<i>.jsonl — per-worker results
#   mutation/v2/parallel/log.<i>          — per-worker stdout
#   mutation/v2/parallel/pid.<i>          — per-worker PID
#
# After all workers exit, `merge` concatenates the per-worker results into
# the canonical mutation/v2/results.jsonl (de-duped on mutant name).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

V2_DIR="$REPO_ROOT/mutation/v2"
PARA_DIR="$V2_DIR/parallel"
WORKSPACES="/tmp/v2_workspaces"
SWEEP_MISSED="$V2_DIR/sweep/mutants.out/missed.txt"
CATALOG="$V2_DIR/mutants_catalog.json"

mkdir -p "$PARA_DIR"

start() {
    shift || true  # consume the unused legacy N argument if present
    local extra=("$@")

    if [ ! -f "$SWEEP_MISSED" ]; then
        echo "missing $SWEEP_MISSED" >&2; exit 1
    fi
    if [ ! -f "$CATALOG" ]; then
        echo "missing $CATALOG" >&2; exit 1
    fi

    echo "Splitting $(wc -l < "$SWEEP_MISSED") missed mutants by oracle"
    rm -f "$PARA_DIR"/missed.shard.* "$PARA_DIR"/results.*.jsonl \
          "$PARA_DIR"/log.* "$PARA_DIR"/pid.*
    # Three shards, one per oracle. This avoids the port-3333 collision
    # that hits when two parallel M33 workers each try to spawn QEMU.
    # Shard 0 → qemu_diff_m33 (mdrp2350: decode.rs, execute.rs, execute_thumb32.rs)
    # Shard 1 → qemu_diff_m0plus (all mdrp2040 files)
    # Shard 2 → softfloat_diff (mdrp2350/execute_fpu.rs)
    awk '
      /crates\/mdrp2350\/src\/core\/decode\.rs:/         { print > "'"$PARA_DIR"'/missed.shard.0"; next }
      /crates\/mdrp2350\/src\/core\/execute\.rs:/        { print > "'"$PARA_DIR"'/missed.shard.0"; next }
      /crates\/mdrp2350\/src\/core\/execute_thumb32\.rs:/{ print > "'"$PARA_DIR"'/missed.shard.0"; next }
      /crates\/mdrp2040\/src\/core\//                    { print > "'"$PARA_DIR"'/missed.shard.1"; next }
      /crates\/mdrp2350\/src\/core\/execute_fpu\.rs:/    { print > "'"$PARA_DIR"'/missed.shard.2"; next }
    ' "$SWEEP_MISSED"
    local n=3

    mkdir -p "$WORKSPACES"
    for i in $(seq 0 $((n-1))); do
        local w="$WORKSPACES/w$i"
        if [ -d "$w" ]; then
            echo "  worker $i: existing worktree at $w (reusing)"
        else
            echo "  worker $i: creating worktree at $w"
            git worktree add --detach "$w" HEAD >/dev/null
        fi
        # Each worker needs the catalog. Copy it (cheap; 7.4 MB) so each
        # worktree has a self-contained mutation/v2/ tree.
        mkdir -p "$w/mutation/v2"
        cp -f "$CATALOG" "$w/mutation/v2/mutants_catalog.json"

        local shard="$PARA_DIR/missed.shard.$i"
        local results="$PARA_DIR/results.$i.jsonl"
        local logf="$PARA_DIR/log.$i"

        # Pre-existing results in the canonical mutation/v2/results.jsonl
        # are also valid skip-done records for any worker — copy a
        # filtered view of them into each worker's results.<i>.jsonl
        # before launching, so --skip-done covers prototype + earlier
        # partial runs.
        if [ -f "$V2_DIR/results.jsonl" ]; then
            cp "$V2_DIR/results.jsonl" "$results"
        else
            : > "$results"
        fi

        echo "  worker $i: shard=$(wc -l < "$shard") mutants → $results"
        (
            cd "$w"
            nohup setsid python3 scripts/v2_mutation_runner.py \
                --missed "$shard" \
                --skip-done \
                --fuzz 200 \
                --timeout 180 \
                --catalog-path "$w/mutation/v2/mutants_catalog.json" \
                --results-path "$results" \
                --log-path "$logf" \
                "${extra[@]}" \
                >> "$logf" 2>&1 < /dev/null &
            echo "$!" > "$PARA_DIR/pid.$i"
        )
        sleep 1
    done
    echo "Launched $n workers."
}

status() {
    local total=0 caught=0 survived=0 errored=0
    for f in "$PARA_DIR"/results.*.jsonl; do
        [ -f "$f" ] || continue
        local i=${f##*results.}; i=${i%.jsonl}
        local n_records=$(wc -l < "$f" 2>/dev/null || echo 0)
        local pid=$(cat "$PARA_DIR/pid.$i" 2>/dev/null || echo "?")
        local status="?"
        if [ -f "$PARA_DIR/pid.$i" ] && kill -0 "$pid" 2>/dev/null; then
            status="running"
        else
            status="exited"
        fi
        local last=$(tail -1 "$f" 2>/dev/null \
            | python3 -c "import json,sys; r=json.loads(sys.stdin.read() or '{}'); print(r.get('classification', '?'))" 2>/dev/null)
        printf "  worker %d: pid=%s [%s] %d records, last=%s\n" \
            "$i" "$pid" "$status" "$n_records" "$last"
        total=$((total + n_records))
    done
    echo "  total records (incl. dedup carry-overs): $total"
}

wait_workers() {
    while :; do
        local any=0
        for f in "$PARA_DIR"/pid.*; do
            [ -f "$f" ] || continue
            local pid=$(cat "$f")
            if kill -0 "$pid" 2>/dev/null; then any=1; break; fi
        done
        [ "$any" -eq 0 ] && break
        sleep 30
    done
    echo "all workers exited at $(date -Iseconds)"
}

merge() {
    # Concatenate per-worker results, de-dup by name (keep last entry
    # per name — most recent oracle verdict wins). Output to canonical
    # mutation/v2/results.jsonl.
    local out="$V2_DIR/results.jsonl"
    python3 - "$PARA_DIR" "$out" << 'PYEOF'
import json
import sys
from pathlib import Path

para_dir = Path(sys.argv[1])
out_path = Path(sys.argv[2])

# Load all per-worker results, latest wins.
by_name = {}
for f in sorted(para_dir.glob("results.*.jsonl")):
    with f.open() as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            by_name[r["name"]] = r

# Preserve any rows already in the canonical results.jsonl that aren't
# in by_name (e.g. older prototype rows for files outside the parallel
# missed set).
if out_path.exists():
    with out_path.open() as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if r["name"] not in by_name:
                by_name[r["name"]] = r

with out_path.open("w") as fp:
    for r in by_name.values():
        fp.write(json.dumps(r) + "\n")

print(f"merged {len(by_name)} unique rows → {out_path}")
PYEOF
}

stop() {
    for f in "$PARA_DIR"/pid.*; do
        [ -f "$f" ] || continue
        local pid=$(cat "$f")
        if kill -0 "$pid" 2>/dev/null; then
            echo "killing worker pid=$pid"
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    sleep 5
    for f in "$PARA_DIR"/pid.*; do
        [ -f "$f" ] || continue
        local pid=$(cat "$f")
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    # Restore any worktree source files
    for w in "$WORKSPACES"/w*; do
        [ -d "$w" ] || continue
        (cd "$w" && git checkout -- crates/ 2>/dev/null || true)
    done
}

clean() {
    stop
    for w in "$WORKSPACES"/w*; do
        [ -d "$w" ] || continue
        echo "removing worktree $w"
        git worktree remove --force "$w" 2>/dev/null || rm -rf "$w"
    done
    git worktree prune
    rm -rf "$PARA_DIR"
}

case "${1:-}" in
    start)   shift; start "$@" ;;
    status)  status ;;
    wait)    wait_workers ;;
    merge)   merge ;;
    stop)    stop ;;
    clean)   clean ;;
    *)
        echo "usage: $0 {start [N=4] [extra args] | status | wait | merge | stop | clean}" >&2
        exit 2
        ;;
esac

#!/usr/bin/env python3
"""
V2 Mutation Summary: aggregate scripts/v2_mutation_runner.py output
(`mutation/v2/results.jsonl`) into per-file / per-classification counts
plus a survivor list suitable for triage.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = REPO_ROOT / "mutation" / "v2" / "results.jsonl"


def load(path: Path) -> list[dict]:
    out: list[dict] = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            out.append(json.loads(line))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--results", default=str(DEFAULT_RESULTS),
                    help="path to results.jsonl")
    ap.add_argument("--survivors-out",
                    help="write oracle_survived mutant names to this file")
    args = ap.parse_args()

    path = Path(args.results)
    if not path.exists():
        sys.exit(f"missing: {path}")
    rows = load(path)
    if not rows:
        sys.exit(f"empty: {path}")

    print(f"Total V2 records: {len(rows)}")
    print()
    cls = Counter(r["classification"] for r in rows)
    print("By classification:")
    for k in ("oracle_caught", "oracle_survived", "oracle_unavailable",
              "build_failed", "skip_no_oracle", "error"):
        print(f"  {k:<18} {cls.get(k, 0):>5}")
    print()

    by_file: dict[str, Counter] = defaultdict(Counter)
    for r in rows:
        by_file[r["file"]][r["classification"]] += 1
    print("By file (caught / survived / other):")
    for f in sorted(by_file):
        c = by_file[f]
        caught = c.get("oracle_caught", 0)
        survived = c.get("oracle_survived", 0)
        other = sum(c.values()) - caught - survived
        total = caught + survived + other
        print(f"  {f}")
        print(f"    caught={caught:>4}  survived={survived:>4}  "
              f"other={other:>3}  total={total}")
    print()

    by_oracle: dict[str, Counter] = defaultdict(Counter)
    for r in rows:
        by_oracle[r["oracle"] or "(none)"][r["classification"]] += 1
    print("By oracle:")
    for o in sorted(by_oracle):
        c = by_oracle[o]
        print(f"  {o:<18} caught={c.get('oracle_caught', 0):>4}  "
              f"survived={c.get('oracle_survived', 0):>4}  "
              f"errored={c.get('build_failed', 0) + c.get('error', 0):>3}")
    print()

    walls = [r["wall_seconds"] for r in rows if r.get("wall_seconds")]
    if walls:
        print(f"Wall-clock per mutant: median={sorted(walls)[len(walls)//2]:.1f}s, "
              f"p90={sorted(walls)[int(len(walls) * 0.9)]:.1f}s, "
              f"max={max(walls):.1f}s, total={sum(walls)/3600:.2f}h")

    survivors = [r["name"] for r in rows if r["classification"] == "oracle_survived"]
    print(f"\nOracle survivors: {len(survivors)}")

    if args.survivors_out and survivors:
        Path(args.survivors_out).write_text("\n".join(survivors) + "\n")
        print(f"Wrote survivors to {args.survivors_out}")


if __name__ == "__main__":
    main()

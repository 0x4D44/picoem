#!/usr/bin/env python3
"""
V2 Survivor Inspector — for each oracle_survived mutant in
mutation/v2/results.jsonl, print the source context (the line being
mutated) plus the function it lives in. Used during triage to bucket
each survivor as Bucket-2 (real oracle gap) or Bucket-4 (equivalent /
unreachable).

Usage:
  scripts/v2_survivor_inspector.py [--results <path>] [--limit N]

Output: one block per survivor:
  -- name: <mutant name>
     file: <path>:<line>
     function: <fn> (genre <genre>)
     context: <source line, with ↑ pointing at start_col>
     replacement: <replacement string>
     notes: <oracle output summary>
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = REPO_ROOT / "mutation" / "v2" / "results.jsonl"
CATALOG = REPO_ROOT / "mutation" / "v2" / "mutants_catalog.json"


def load_results(path: Path) -> list[dict]:
    out: list[dict] = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def load_catalog() -> dict[str, dict]:
    with CATALOG.open() as f:
        d = json.load(f)
    return {m["name"]: m for m in d}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--results", default=str(DEFAULT_RESULTS))
    ap.add_argument("--limit", type=int, default=0,
                    help="show only first N survivors (0 = all)")
    ap.add_argument("--by-file", action="store_true",
                    help="group output by file")
    args = ap.parse_args()

    results = load_results(Path(args.results))
    survivors = [r for r in results if r["classification"] == "oracle_survived"]
    print(f"# {len(survivors)} oracle_survived rows in {args.results}\n")

    catalog = load_catalog()

    if args.by_file:
        survivors.sort(key=lambda r: (r["file"], r["name"]))

    n = 0
    for r in survivors:
        if args.limit and n >= args.limit:
            break
        m = catalog.get(r["name"])
        if not m:
            continue
        line_no = m["span"]["start"]["line"]
        col = m["span"]["start"]["column"]
        end_col = m["span"]["end"]["column"]
        file_path = REPO_ROOT / r["file"]
        try:
            src_line = file_path.read_text().splitlines()[line_no - 1]
        except Exception:
            src_line = "(?? source read failed)"
        fn = (m["function"] or {}).get("function_name", "<top-level>")
        print(f"-- name: {r['name']}")
        print(f"   file: {r['file']}:{line_no}")
        print(f"   fn:   {fn}  (genre {m['genre']})")
        print(f"   src:  {src_line}")
        # arrow under the start_col
        print(f"   col:  {' ' * (col - 1 + 9)}^")
        if m["span"]["end"]["line"] == line_no and end_col - col > 1:
            print(f"   span: cols {col}..{end_col-1}")
        print(f"   repl: {m['replacement'][:80]!r}")
        print()
        n += 1


if __name__ == "__main__":
    main()

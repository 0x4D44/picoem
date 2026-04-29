#!/usr/bin/env python3
"""
V2 Oracle Runner for Mutation Testing

Given a list of "missed" mutants (those that survived `cargo test` in V1),
apply each mutation to the source tree, build the appropriate differential
oracle, run it briefly, and classify each mutant as oracle-caught or
oracle-survived.

Inputs:
  - mutation/v2/mutants_catalog.json  (full mutant catalogue with diffs)
  - mutation/v2/missed.txt            (one mutant name per line, V1 output)
  - or --names "<name>" args for prototype runs

Outputs:
  - mutation/v2/results.jsonl         (one JSON record per mutant tested)
  - mutation/v2/runner.log            (combined stdout/stderr)

Per-mutant pipeline:
  1. Save original file content.
  2. Apply mutation by splicing [start_col-1, end_col-1) on the source file.
  3. Build the appropriate oracle binary (cargo build --release ...).
  4. Run oracle for a bounded fuzz/time budget.
  5. Restore original file (always; finally block).
  6. Record outcome.

Oracle routing:
  mdrp2350/core/execute_thumb32.rs  -> qemu_diff_m33
  mdrp2350/core/execute_fpu.rs      -> softfloat_diff
  mdrp2350/core/execute.rs          -> qemu_diff_m33
  mdrp2350/core/decode.rs           -> qemu_diff_m33
  mdrp2040/core/execute.rs          -> qemu_diff_m0plus
  mdrp2040/core/execute_wide.rs     -> qemu_diff_m0plus
  mdrp2040/core/decode.rs           -> qemu_diff_m0plus
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
V2_DIR = REPO_ROOT / "mutation" / "v2"
CATALOG_PATH = V2_DIR / "mutants_catalog.json"
RESULTS_PATH = V2_DIR / "results.jsonl"
LOG_PATH = V2_DIR / "runner.log"

ORACLE_FOR_FILE = {
    "crates/mdrp2350/src/core/execute_thumb32.rs": "qemu_diff_m33",
    "crates/mdrp2350/src/core/execute_fpu.rs":     "softfloat_diff",
    "crates/mdrp2350/src/core/execute.rs":         "qemu_diff_m33",
    "crates/mdrp2350/src/core/decode.rs":          "qemu_diff_m33",
    "crates/mdrp2040/src/core/execute.rs":         "qemu_diff_m0plus",
    "crates/mdrp2040/src/core/execute_wide.rs":    "qemu_diff_m0plus",
    "crates/mdrp2040/src/core/decode.rs":          "qemu_diff_m0plus",
}

# QEMU oracles take a --fuzz N flag. softfloat_diff takes --fuzz too.
DEFAULT_FUZZ = 2000


@dataclass
class Result:
    name: str
    file: str
    oracle: str
    classification: str   # "oracle_caught", "oracle_survived",
                          # "build_failed", "skip_no_oracle", "error"
    fuzz_count: int
    wall_seconds: float
    exit_code: Optional[int]
    notes: str


def log(msg: str) -> None:
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with LOG_PATH.open("a") as f:
        f.write(line + "\n")


def load_catalog() -> dict[str, dict]:
    with CATALOG_PATH.open() as f:
        d = json.load(f)
    return {m["name"]: m for m in d}


def compute_line_starts(text: bytes) -> list[int]:
    """Return list of byte offsets for the start of each line (1-based)."""
    starts = [0]
    for i, b in enumerate(text):
        if b == 0x0A:  # newline
            starts.append(i + 1)
    return starts


def apply_mutation(file_path: Path, mutant: dict) -> bytes:
    """Apply mutation to file, return original content for revert."""
    original = file_path.read_bytes()
    line_starts = compute_line_starts(original)

    span = mutant["span"]
    start_off = line_starts[span["start"]["line"] - 1] + span["start"]["column"] - 1
    end_off = line_starts[span["end"]["line"] - 1] + span["end"]["column"] - 1

    replacement = mutant["replacement"].encode("utf-8")
    mutated = original[:start_off] + replacement + original[end_off:]
    file_path.write_bytes(mutated)
    return original


def revert(file_path: Path, original: bytes) -> None:
    file_path.write_bytes(original)


def build_oracle(oracle: str) -> tuple[bool, str]:
    """Run cargo build for the oracle binary. Returns (success, stderr)."""
    cmd = [
        "cargo", "build", "--release",
        "-p", "mdpicoem-harness",
        "--bin", oracle,
    ]
    proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    return proc.returncode == 0, proc.stderr[-2000:] if proc.stderr else ""


def run_oracle(oracle: str, fuzz: int, timeout_s: int) -> tuple[int, str]:
    """Run the oracle. Return (exit_code, last 1000 chars of stderr).

    Per-oracle CLI quirks:
    - qemu_diff_m33: FPU class spawns mps2-an505 FPU which is broken on
      QEMU 8.2 (all cases SKIP, run hangs). Pin --classes base.
    - softfloat_diff: --mode all to cover both FPU + DCP coprocessor.
    - qemu_diff_m0plus: no class flag, runs the whole corpus.
    """
    binary = REPO_ROOT / "target" / "release" / oracle
    cmd = [str(binary), "--fuzz", str(fuzz)]
    if oracle == "qemu_diff_m33":
        cmd += ["--classes", "base"]
    elif oracle == "softfloat_diff":
        cmd += ["--mode", "all"]
    try:
        proc = subprocess.run(
            cmd, cwd=REPO_ROOT, capture_output=True, text=True,
            timeout=timeout_s,
        )
        return proc.returncode, (proc.stdout[-500:] + proc.stderr[-500:])
    except subprocess.TimeoutExpired:
        return -1, "[timeout]"


def classify(exit_code: int) -> str:
    """
    qemu_diff_m33 / qemu_diff_m0plus / softfloat_diff exit:
      0 → all cases passed (mutation NOT caught — survived oracle)
      non-zero → at least one case failed (mutation caught)
    """
    if exit_code == 0:
        return "oracle_survived"
    return "oracle_caught"


def process_mutant(mutant: dict, fuzz: int, timeout_s: int) -> Result:
    name = mutant["name"]
    file_rel = mutant["file"].replace("\\", "/")
    file_path = REPO_ROOT / file_rel

    oracle = ORACLE_FOR_FILE.get(file_rel)
    if oracle is None:
        return Result(
            name=name, file=file_rel, oracle="", classification="skip_no_oracle",
            fuzz_count=0, wall_seconds=0.0, exit_code=None,
            notes=f"no oracle mapping for {file_rel}",
        )

    start = time.time()
    original: Optional[bytes] = None
    try:
        original = apply_mutation(file_path, mutant)
        ok, stderr = build_oracle(oracle)
        if not ok:
            return Result(
                name=name, file=file_rel, oracle=oracle,
                classification="build_failed",
                fuzz_count=0, wall_seconds=time.time() - start,
                exit_code=None, notes=f"build error: {stderr[-300:]}",
            )

        exit_code, tail = run_oracle(oracle, fuzz, timeout_s)
        return Result(
            name=name, file=file_rel, oracle=oracle,
            classification=classify(exit_code),
            fuzz_count=fuzz, wall_seconds=time.time() - start,
            exit_code=exit_code, notes=tail[-300:].replace("\n", " | "),
        )
    except Exception as e:
        return Result(
            name=name, file=file_rel, oracle=oracle or "",
            classification="error",
            fuzz_count=0, wall_seconds=time.time() - start,
            exit_code=None, notes=f"{type(e).__name__}: {e}",
        )
    finally:
        if original is not None:
            revert(file_path, original)


def iter_target_names(args, catalog: dict[str, dict]) -> Iterable[str]:
    if args.names:
        for n in args.names:
            yield n
    elif args.missed:
        with open(args.missed) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    yield line
    elif args.sample:
        # Hand-picked sample from V1 triage HLD §3 examples.
        # We can't know the exact V1 missed names without the missed.txt,
        # so for the prototype we'll pick a few that match the patterns
        # the HLD called out and let measurement tell us what they really do.
        candidates = []
        for name, m in catalog.items():
            f = m["file"].replace("\\", "/")
            # Pick from each top-survivor file, prefer simple binary-op mutants.
            if m["genre"] == "BinaryOperator" and f in ORACLE_FOR_FILE:
                candidates.append(name)
        # Take first 2 per file.
        per_file = {}
        for n in candidates:
            f = catalog[n]["file"].replace("\\", "/")
            per_file.setdefault(f, []).append(n)
        for f, ns in per_file.items():
            for n in ns[:2]:
                yield n


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--missed", help="path to V1 missed.txt; one mutant name per line")
    ap.add_argument("--names", nargs="*", help="specific mutant names to process")
    ap.add_argument("--sample", action="store_true",
                    help="run a small hand-picked sample for prototyping")
    ap.add_argument("--fuzz", type=int, default=DEFAULT_FUZZ,
                    help=f"fuzz iterations per oracle run (default {DEFAULT_FUZZ})")
    ap.add_argument("--timeout", type=int, default=180,
                    help="per-oracle timeout in seconds (default 180)")
    ap.add_argument("--max", type=int, default=0,
                    help="stop after N mutants (0 = no limit)")
    args = ap.parse_args()

    if not (args.missed or args.names or args.sample):
        ap.error("specify --missed, --names, or --sample")

    V2_DIR.mkdir(parents=True, exist_ok=True)
    log(f"V2 runner starting: fuzz={args.fuzz} timeout={args.timeout}s max={args.max}")
    catalog = load_catalog()
    log(f"loaded catalog: {len(catalog)} mutants")

    results_f = RESULTS_PATH.open("a")
    n = 0
    for name in iter_target_names(args, catalog):
        if args.max and n >= args.max:
            break
        m = catalog.get(name)
        if m is None:
            log(f"SKIP unknown mutant: {name}")
            continue
        log(f"[{n+1}] {name}")
        result = process_mutant(m, args.fuzz, args.timeout)
        log(f"    -> {result.classification} (exit={result.exit_code} "
            f"wall={result.wall_seconds:.1f}s)")
        results_f.write(json.dumps(asdict(result)) + "\n")
        results_f.flush()
        n += 1

    results_f.close()
    log(f"V2 runner done: {n} mutants processed")


if __name__ == "__main__":
    main()

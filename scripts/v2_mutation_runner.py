#!/usr/bin/env python3
"""
V2 Oracle Runner for Mutation Testing

Given a list of "missed" mutants (those that survived `cargo test` in V1),
apply each mutation to the source tree, build the appropriate differential
oracle, run it briefly, and classify each mutant as oracle-caught,
oracle-survived, or oracle-unavailable.

Inputs:
  - mutation/v2/mutants_catalog.json  (full mutant catalogue with diffs)
  - mutation/v2/missed.txt            (one mutant name per line, V1 output)
  - or --names "<name>" args for prototype runs
  - scripts/v2_oracle_routing.json    (per-function oracle routing sidecar;
                                       falls back to in-code ORACLE_FOR_FILE
                                       if absent / malformed / disabled)

Outputs:
  - mutation/v2/results.jsonl         (one JSON record per mutant tested)
  - mutation/v2/runner.log            (combined stdout/stderr)

Per-mutant pipeline:
  1. Save original file content.
  2. Apply mutation by splicing [start_col-1, end_col-1) on the source file.
  3. Resolve routes (per-function → per-file fallback) per HLD V1 §3.3.
  4. For each route in order, build the oracle, run it briefly, record
     a RouteResult. Short-circuit on first oracle_caught (HLD §3.4).
  5. Restore original file (always; finally block).
  6. Aggregate per HLD §3.7 and append a Result record.

Oracle routing (default per-file fallback; per-function overrides live in
scripts/v2_oracle_routing.json):
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
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Iterable, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
V2_DIR = REPO_ROOT / "mutation" / "v2"
CATALOG_PATH = V2_DIR / "mutants_catalog.json"
RESULTS_PATH = V2_DIR / "results.jsonl"
LOG_PATH = V2_DIR / "runner.log"

DEFAULT_ROUTING_PATH = REPO_ROOT / "scripts" / "v2_oracle_routing.json"

# These paths are mutable: --results-path / --catalog-path / --log-path
# override them at startup so parallel workers can each write to a
# private file. The bound is global because process_mutant() and
# friends use them. Unfashionable but pragmatic for a sequential
# Python runner.
def _set_paths(catalog: Optional[str], results: Optional[str],
               log: Optional[str]) -> None:
    global CATALOG_PATH, RESULTS_PATH, LOG_PATH
    if catalog:
        CATALOG_PATH = Path(catalog)
    if results:
        RESULTS_PATH = Path(results)
    if log:
        LOG_PATH = Path(log)

# Legacy per-file routing. Kept as the fallback when the JSON sidecar
# is absent / disabled / malformed (HLD §3.5). Updates here flow through
# load_routing() into the synthesised routing dict.
ORACLE_FOR_FILE = {
    "crates/mdrp2350/src/core/execute_thumb32.rs": "qemu_diff_m33",
    "crates/mdrp2350/src/core/execute_fpu.rs":     "softfloat_diff",
    "crates/mdrp2350/src/core/execute.rs":         "qemu_diff_m33",
    "crates/mdrp2350/src/core/decode.rs":          "qemu_diff_m33",
    "crates/mdrp2040/src/core/execute.rs":         "qemu_diff_m0plus",
    "crates/mdrp2040/src/core/execute_wide.rs":    "qemu_diff_m0plus",
    "crates/mdrp2040/src/core/decode.rs":          "qemu_diff_m0plus",
}

# Per-oracle CLI arg defaults used when load_routing() synthesises the
# fallback routing (no sidecar). Keep aligned with run_oracle() below.
_LEGACY_ORACLE_ARGS = {
    "qemu_diff_m33":    ["--classes", "base"],
    "qemu_diff_m0plus": [],
    "softfloat_diff":   ["--mode", "all"],
}

# QEMU oracles take a --fuzz N flag. softfloat_diff takes --fuzz too.
DEFAULT_FUZZ = 2000

# FPU-class smoke probe budget. HLD §5.1 (supervisor decision Q4): 10 s.
# Broken-QEMU EAGAIN path returns in <3 s, so fail-fast is preserved.
FPU_SMOKE_TIMEOUT_S = 10.0


# ---------------------------------------------------------------------
# Result types (HLD §3.6 — additive with legacy back-compat fields)
# ---------------------------------------------------------------------

@dataclass
class RouteResult:
    oracle: str
    args: list[str]
    classification: str   # "oracle_caught", "oracle_survived",
                          # "oracle_unavailable", "build_failed",
                          # "skipped", "error"
    fuzz_count: int
    wall_seconds: float
    exit_code: Optional[int]
    notes: str


@dataclass
class Result:
    name: str
    file: str
    function: Optional[str]   # function.function_name from catalog (or None)
    classification: str       # AGGREGATE: see aggregate_classification()
    routes: list[dict]        # one entry per route attempted (RouteResult dicts)
    wall_seconds: float       # total across all routes
    # Legacy back-compat fields (kept so old consumers don't break):
    oracle: str               # = routes[0].oracle (or "" if no routes)
    fuzz_count: int           # = routes[0].fuzz_count
    exit_code: Optional[int]  # = routes[0].exit_code
    notes: str                # = routes[0].notes


def log(msg: str) -> None:
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with LOG_PATH.open("a") as f:
        f.write(line + "\n")


def load_catalog() -> dict[str, dict]:
    with CATALOG_PATH.open() as f:
        d = json.load(f)
    return {m["name"]: m for m in d}


# ---------------------------------------------------------------------
# Routing: load + resolve + aggregate (HLD §3.2 / §3.3 / §3.7)
# ---------------------------------------------------------------------

def _legacy_routing_dict() -> dict:
    """Synthesise the routing structure from in-code ORACLE_FOR_FILE.

    Used when the JSON sidecar is absent / disabled / malformed.
    Preserves today's per-file behaviour byte-identically.
    """
    return {
        "version": 1,
        "default_oracles_by_file": {
            file_rel: [{
                "oracle": oracle,
                "args": list(_LEGACY_ORACLE_ARGS.get(oracle, [])),
            }]
            for file_rel, oracle in ORACLE_FOR_FILE.items()
        },
        "by_function": {},
    }


def load_routing(path: Optional[Path]) -> dict:
    """Load the routing JSON sidecar.

    `path=None` → fallback to in-code ORACLE_FOR_FILE (HLD §3.5).
    Missing file or invalid JSON also falls back, with a stderr warning.
    """
    if path is None:
        return _legacy_routing_dict()
    p = Path(path)
    if not p.exists():
        # Quiet fallback: many call sites just want "use whatever's there".
        return _legacy_routing_dict()
    try:
        with p.open() as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError) as e:
        print(f"WARN: routing sidecar {p} unreadable ({e}); "
              f"falling back to ORACLE_FOR_FILE", file=sys.stderr)
        return _legacy_routing_dict()
    # Defensive: ensure required keys exist.
    data.setdefault("version", 1)
    data.setdefault("default_oracles_by_file", {})
    data.setdefault("by_function", {})
    return data


def extract_function_name(mutant: dict) -> Optional[str]:
    """Return the catalog `function.function_name` field, or None.

    cargo-mutants populates this for ~99.6 % of mutants. Module-scope
    mutants (top-level `const`s, free helpers) have no `function` field
    and fall through to file-default routing.
    """
    fn = mutant.get("function")
    if not isinstance(fn, dict):
        return None
    name = fn.get("function_name")
    return name if isinstance(name, str) and name else None


def _normalise_path(p: str) -> str:
    return p.replace("\\", "/")


def resolve_routes(
    mutant: dict, routing: dict, capabilities: set[str],
) -> list[dict]:
    """Return the ordered list of route dicts for this mutant.

    Per HLD §3.3:
      1. Look up by_function[file][function_name] if both keys present.
      2. Else fall through to default_oracles_by_file[file].
      3. Filter out any route whose `requires` capability is missing.
    """
    file_rel = _normalise_path(mutant.get("file", ""))
    fn = extract_function_name(mutant)
    by_fn = routing.get("by_function", {}).get(file_rel, {})

    if fn and fn in by_fn:
        candidates = by_fn[fn]
    else:
        candidates = routing.get("default_oracles_by_file", {}).get(
            file_rel, [],
        )

    out: list[dict] = []
    for r in candidates:
        req = r.get("requires")
        if req is None or req in capabilities:
            out.append(r)
    return out


def aggregate_classification(routes: list[RouteResult]) -> str:
    """Aggregate per HLD §3.7 table.

    Order of precedence:
      1. Empty list → skip_no_oracle.
      2. Any oracle_caught → oracle_caught.
      3. Any build_failed → build_failed.
      4. Any oracle_survived → oracle_survived (covers mixed
         survived+unavailable per §3.7's subtle rule).
      5. All oracle_unavailable → oracle_unavailable.
      6. Otherwise → error.
    """
    if not routes:
        return "skip_no_oracle"
    classifications = [r.classification for r in routes]
    if any(c == "oracle_caught" for c in classifications):
        return "oracle_caught"
    if any(c == "build_failed" for c in classifications):
        return "build_failed"
    if any(c == "oracle_survived" for c in classifications):
        return "oracle_survived"
    if all(c == "oracle_unavailable" for c in classifications):
        return "oracle_unavailable"
    return "error"


# ---------------------------------------------------------------------
# Mutation apply / revert
# ---------------------------------------------------------------------

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


# ---------------------------------------------------------------------
# Build + run
# ---------------------------------------------------------------------

CARGO_EXTRA_ARGS = []  # populated from --cargo-arg in main()


def build_oracle(oracle: str) -> tuple[bool, str]:
    """Run cargo build for the oracle binary. Returns (success, stderr)."""
    cmd = [
        "cargo", "build", "--release",
        *CARGO_EXTRA_ARGS,
        "-p", "mdpicoem-harness",
        "--bin", oracle,
    ]
    proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    return proc.returncode == 0, proc.stderr[-2000:] if proc.stderr else ""


def run_oracle(
    oracle: str, args: list[str], fuzz: int, timeout_s: int,
) -> tuple[int, str]:
    """Run the oracle. Return (exit_code, last 1000 chars of stderr).

    `args` comes verbatim from the routing-table route (e.g.
    `["--classes", "base"]` or `["--mode", "all"]`). The runner appends
    `--fuzz N`. Per-oracle CLI quirks now live in the routing JSON,
    not hardcoded here.
    """
    binary = REPO_ROOT / "target" / "release" / oracle
    cmd = [str(binary), "--fuzz", str(fuzz), *args]
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
      -1 → timeout (we report as "error" so triage notices)
    """
    if exit_code == -1:
        return "error"
    if exit_code == 0:
        return "oracle_survived"
    return "oracle_caught"


# ---------------------------------------------------------------------
# Capability detection (HLD §5.1)
# ---------------------------------------------------------------------

def smoke_probe_fpu(timeout: float = FPU_SMOKE_TIMEOUT_S) -> bool:
    """Probe whether `qemu_diff_m33 --classes fpu` is healthy.

    Pass criteria (per HLD §5.1): process exits within `timeout`
    seconds AND exit code is 0 or 1 (oracle ran cleanly OR caught a
    diff — both prove the FPU class isn't EAGAIN-stuck).
    Fail criteria: timeout, exit code outside {0,1}, or spawn error.
    """
    binary = REPO_ROOT / "target" / "release" / "qemu_diff_m33"
    if not binary.exists():
        return False
    try:
        proc = subprocess.run(
            [str(binary), "--classes", "fpu", "--fuzz", "1"],
            cwd=REPO_ROOT, capture_output=True, text=True,
            timeout=timeout,
        )
    except (subprocess.TimeoutExpired, OSError):
        return False
    return proc.returncode in (0, 1)


def detect_capabilities(
    routing: dict, force_allow_fpu: bool, force_no_fpu: bool,
) -> set[str]:
    """Determine which capabilities are healthy on this host.

    The smoke probe runs only when at least one route in the loaded
    table requires a capability. Today only "fpu_class" is gated.

    Operator overrides (mutually exclusive):
      --allow-fpu  → force-on regardless of probe.
      --no-fpu     → force-off regardless of probe.
    """
    caps: set[str] = set()
    needs_fpu = _routing_uses_capability(routing, "fpu_class")

    if force_allow_fpu:
        caps.add("fpu_class")
        log("FPU-class capability force-enabled (--allow-fpu)")
    elif force_no_fpu:
        log("FPU-class capability force-disabled (--no-fpu)")
    elif needs_fpu:
        log(f"running FPU-class smoke probe (timeout={FPU_SMOKE_TIMEOUT_S}s)")
        if smoke_probe_fpu():
            caps.add("fpu_class")
            log("FPU-class smoke probe passed; routes with "
                "'requires: fpu_class' will run")
        else:
            log(f"FPU-class smoke probe failed (timeout="
                f"{FPU_SMOKE_TIMEOUT_S}s, env may need QEMU 10.2). "
                f"Routes with 'requires: fpu_class' will record "
                f"'oracle_unavailable'.")
    return caps


def _routing_uses_capability(routing: dict, cap: str) -> bool:
    for fns in routing.get("by_function", {}).values():
        for routes in fns.values():
            for r in routes:
                if r.get("requires") == cap:
                    return True
    for routes in routing.get("default_oracles_by_file", {}).values():
        for r in routes:
            if r.get("requires") == cap:
                return True
    return False


# ---------------------------------------------------------------------
# Per-mutant orchestration
# ---------------------------------------------------------------------

def _full_candidates(mutant: dict, routing: dict) -> list[dict]:
    """Return the unfiltered candidate route list for this mutant.

    Mirrors `resolve_routes` minus the capability filter. Used by
    `_make_unavailable_routes` and `process_mutant` to record what
    would have run (so triage sees `oracle_unavailable` rows even when
    every candidate is capability-gated out).
    """
    file_rel = _normalise_path(mutant.get("file", ""))
    fn = extract_function_name(mutant)
    by_fn = routing.get("by_function", {}).get(file_rel, {})
    if fn:
        cands = by_fn.get(fn)
        if cands is not None:
            return cands
    return routing.get("default_oracles_by_file", {}).get(file_rel, [])


def _make_unavailable_routes(
    mutant: dict, routing: dict,
) -> list[RouteResult]:
    """When resolve_routes() returns [] because every candidate was
    capability-gated out, we still want to record what would have run
    (so triage can see the oracle_unavailable verdicts per route)."""
    return [
        RouteResult(
            oracle=r["oracle"],
            args=list(r.get("args", [])),
            classification="oracle_unavailable",
            fuzz_count=0,
            wall_seconds=0.0,
            exit_code=None,
            notes=f"requires={r.get('requires')!r} (capability missing)",
        )
        for r in _full_candidates(mutant, routing)
        if r.get("requires") is not None
    ]


def process_mutant(
    mutant: dict, fuzz: int, timeout_s: int, routing: dict,
    capabilities: set[str],
) -> Result:
    name = mutant["name"]
    file_rel = _normalise_path(mutant["file"])
    file_path = REPO_ROOT / file_rel
    fn_name = extract_function_name(mutant)

    routes_to_run = resolve_routes(mutant, routing, capabilities)

    # No usable route: either file unmapped (skip_no_oracle) or every
    # candidate gated out (oracle_unavailable per HLD §5.2).
    if not routes_to_run:
        unavail = _make_unavailable_routes(mutant, routing)
        if unavail:
            agg = aggregate_classification(unavail)
            return _result_from_routes(
                name, file_rel, fn_name, agg, unavail, 0.0,
            )
        return Result(
            name=name, file=file_rel, function=fn_name,
            classification="skip_no_oracle",
            routes=[], wall_seconds=0.0,
            oracle="", fuzz_count=0, exit_code=None,
            notes=f"no oracle mapping for {file_rel}",
        )

    overall_start = time.time()
    original: Optional[bytes] = None
    route_results: list[RouteResult] = []
    try:
        original = apply_mutation(file_path, mutant)

        # Pre-pad with oracle_unavailable rows for any gated-out routes
        # so the result reflects everything the table specified, not
        # just what survived the capability filter.
        full_candidates = _full_candidates(mutant, routing)

        # TODO V2.x: cross-port coordination when fpu_class becomes
        # healthy — see HLD V1 §10.6. Today, when multiple routes for
        # one mutant resolve and one of them is qemu_diff_m33 --classes
        # fpu, two parallel workers can race on GDB port 3333. The
        # parallel launcher currently shards by oracle; per-function
        # routing in execute_fpu.rs may put a vfp_sd mutant on the
        # softfloat worker that then internally spawns qemu_diff_m33.
        # When the FPU class becomes healthy, the launcher needs a
        # `--owned-oracle <name>` flag (or per-port mutex) to coordinate.
        # Deferred for V1: the smoke probe currently fails on this host
        # so the gated routes are oracle_unavailable and no race.
        for route in full_candidates:
            if route.get("requires") and route["requires"] not in capabilities:
                route_results.append(RouteResult(
                    oracle=route["oracle"],
                    args=list(route.get("args", [])),
                    classification="oracle_unavailable",
                    fuzz_count=0,
                    wall_seconds=0.0,
                    exit_code=None,
                    notes=f"requires={route['requires']!r} "
                          "(capability missing)",
                ))
                continue

            r_start = time.time()
            ok, stderr = build_oracle(route["oracle"])
            if not ok:
                route_results.append(RouteResult(
                    oracle=route["oracle"],
                    args=list(route.get("args", [])),
                    classification="build_failed",
                    fuzz_count=0,
                    wall_seconds=time.time() - r_start,
                    exit_code=None,
                    notes=f"build error: {stderr[-300:]}",
                ))
                # Build failure on one route is terminal — don't try
                # subsequent routes; the source tree state matters.
                break

            exit_code, tail = run_oracle(
                route["oracle"], list(route.get("args", [])),
                fuzz, timeout_s,
            )
            route_results.append(RouteResult(
                oracle=route["oracle"],
                args=list(route.get("args", [])),
                classification=classify(exit_code),
                fuzz_count=fuzz,
                wall_seconds=time.time() - r_start,
                exit_code=exit_code,
                notes=tail[-300:].replace("\n", " | "),
            ))

            # Short-circuit on first catch (HLD §3.4).
            if route_results[-1].classification == "oracle_caught":
                break

        agg = aggregate_classification(route_results)
        return _result_from_routes(
            name, file_rel, fn_name, agg, route_results,
            time.time() - overall_start,
        )

    except Exception as e:
        # Synthesise a minimal error result; preserve any per-route
        # results we already collected so triage can see how far we got.
        if not route_results:
            route_results = [RouteResult(
                oracle="", args=[], classification="error",
                fuzz_count=0, wall_seconds=time.time() - overall_start,
                exit_code=None, notes=f"{type(e).__name__}: {e}",
            )]
        return _result_from_routes(
            name, file_rel, fn_name, "error", route_results,
            time.time() - overall_start,
        )
    finally:
        if original is not None:
            revert(file_path, original)


def _result_from_routes(
    name: str, file_rel: str, fn_name: Optional[str], aggregate: str,
    routes: list[RouteResult], total_wall: float,
) -> Result:
    """Build a Result with both new and legacy back-compat fields populated."""
    first = routes[0] if routes else None
    return Result(
        name=name,
        file=file_rel,
        function=fn_name,
        classification=aggregate,
        routes=[asdict(r) for r in routes],
        wall_seconds=total_wall,
        oracle=first.oracle if first else "",
        fuzz_count=first.fuzz_count if first else 0,
        exit_code=first.exit_code if first else None,
        notes=first.notes if first else "",
    )


# ---------------------------------------------------------------------
# CLI plumbing
# ---------------------------------------------------------------------

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


def load_already_done(only_class: Optional[str] = None) -> set[str]:
    """Return mutant names already present in results.jsonl (for dedup)."""
    done: set[str] = set()
    if not RESULTS_PATH.exists():
        return done
    with RESULTS_PATH.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if only_class is None or r.get("classification") == only_class:
                done.add(r["name"])
    return done


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--missed", help="path to V1 missed.txt; one mutant name per line")
    ap.add_argument("--names", nargs="*", help="specific mutant names to process")
    ap.add_argument("--sample", action="store_true",
                    help="run a small hand-picked sample for prototyping")
    ap.add_argument("--retry-survivors", action="store_true",
                    help="re-test all oracle_survived entries from results.jsonl")
    ap.add_argument("--fuzz", type=int, default=DEFAULT_FUZZ,
                    help=f"fuzz iterations per oracle run (default {DEFAULT_FUZZ})")
    ap.add_argument("--timeout", type=int, default=180,
                    help="per-oracle timeout in seconds (default 180)")
    ap.add_argument("--max", type=int, default=0,
                    help="stop after N mutants (0 = no limit)")
    ap.add_argument("--skip-done", action="store_true",
                    help="skip mutants already present in results.jsonl")
    ap.add_argument("--catalog-path", help="override catalog JSON path")
    ap.add_argument("--results-path", help="override results.jsonl path")
    ap.add_argument("--log-path", help="override runner.log path")
    ap.add_argument("--cargo-arg", action="append", default=[],
                    help="extra arg for the cargo build invocation (repeat).")
    ap.add_argument("--routing", default=str(DEFAULT_ROUTING_PATH),
                    help=f"path to oracle routing JSON sidecar "
                         f"(default {DEFAULT_ROUTING_PATH})")
    ap.add_argument("--no-routing", action="store_true",
                    help="disable JSON sidecar; use in-code "
                         "ORACLE_FOR_FILE only")
    ap.add_argument("--allow-fpu", action="store_true",
                    help="force fpu_class capability ON regardless of "
                         "smoke probe (mutually exclusive with --no-fpu)")
    ap.add_argument("--no-fpu", action="store_true",
                    help="force fpu_class capability OFF regardless of "
                         "smoke probe (mutually exclusive with --allow-fpu)")
    args = ap.parse_args()

    if args.allow_fpu and args.no_fpu:
        ap.error("--allow-fpu and --no-fpu are mutually exclusive")

    _set_paths(args.catalog_path, args.results_path, args.log_path)
    global CARGO_EXTRA_ARGS
    CARGO_EXTRA_ARGS = args.cargo_arg

    if not (args.missed or args.names or args.sample or args.retry_survivors):
        ap.error("specify --missed, --names, --sample, or --retry-survivors")

    V2_DIR.mkdir(parents=True, exist_ok=True)
    log(f"V2 runner starting: fuzz={args.fuzz} timeout={args.timeout}s "
        f"max={args.max} skip_done={args.skip_done}")
    catalog = load_catalog()
    log(f"loaded catalog: {len(catalog)} mutants")

    routing_path: Optional[Path]
    if args.no_routing:
        routing_path = None
        log("routing sidecar disabled (--no-routing); using ORACLE_FOR_FILE")
    else:
        routing_path = Path(args.routing)
        if routing_path.exists():
            log(f"loading routing from {routing_path}")
        else:
            log(f"routing sidecar {routing_path} absent; "
                f"falling back to ORACLE_FOR_FILE")
    routing = load_routing(routing_path)
    capabilities = detect_capabilities(
        routing, args.allow_fpu, args.no_fpu,
    )

    if args.retry_survivors:
        survivors: list[str] = []
        keep: list[dict] = []
        if RESULTS_PATH.exists():
            with RESULTS_PATH.open() as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    r = json.loads(line)
                    if r["classification"] == "oracle_survived":
                        survivors.append(r["name"])
                    else:
                        keep.append(r)
        log(f"retry-survivors: {len(survivors)} survivors to re-test "
            f"(keeping {len(keep)} non-survivor rows)")
        with RESULTS_PATH.open("w") as f:
            for r in keep:
                f.write(json.dumps(r) + "\n")
        names_iter: Iterable[str] = iter(survivors)
    elif args.skip_done:
        done = load_already_done()
        log(f"--skip-done: {len(done)} mutants already in results.jsonl")
        names_iter = (n for n in iter_target_names(args, catalog) if n not in done)
    else:
        names_iter = iter_target_names(args, catalog)

    results_f = RESULTS_PATH.open("a")
    n = 0
    for name in names_iter:
        if args.max and n >= args.max:
            break
        m = catalog.get(name)
        if m is None:
            log(f"SKIP unknown mutant: {name}")
            continue
        log(f"[{n+1}] {name}")
        result = process_mutant(
            m, args.fuzz, args.timeout, routing, capabilities,
        )
        log(f"    -> {result.classification} "
            f"(routes={len(result.routes)} wall={result.wall_seconds:.1f}s)")
        results_f.write(json.dumps(asdict(result)) + "\n")
        results_f.flush()
        n += 1

    results_f.close()
    log(f"V2 runner done: {n} mutants processed")


if __name__ == "__main__":
    main()

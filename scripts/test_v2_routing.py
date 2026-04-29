#!/usr/bin/env python3
"""
Unit tests for V2 per-function oracle routing.

Covers HLD §6.1:
  - Function-name extraction (with / without `function` field).
  - Route resolution (in-table function, out-of-table function,
    out-of-table file).
  - Capability gating (`requires` filter).
  - Aggregation rules per HLD §3.7 (5 cases).
  - Backward-compat (sidecar absent → in-code fallback to
    `ORACLE_FOR_FILE`).

Run with:
    python3 -m unittest scripts/test_v2_routing.py -v
or with pytest if installed.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

# Make scripts/ importable as a module dir.
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import v2_mutation_runner as runner  # noqa: E402


# ---------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------

def make_routing(by_function=None, default_oracles_by_file=None):
    return {
        "version": 1,
        "default_oracles_by_file": default_oracles_by_file or {},
        "by_function": by_function or {},
    }


def make_mutant(name="m", file="crates/mdrp2350/src/core/execute_fpu.rs",
                function_name=None, genre="BinaryOperator",
                replacement="0", span=None):
    m = {
        "name": name,
        "file": file,
        "genre": genre,
        "replacement": replacement,
        "span": span or {
            "start": {"line": 1, "column": 1},
            "end": {"line": 1, "column": 2},
        },
    }
    if function_name is not None:
        m["function"] = {"function_name": function_name}
    return m


SOFTFLOAT_ROUTE = {"oracle": "softfloat_diff", "args": ["--mode", "all"]}
QEMU_FPU_ROUTE = {
    "oracle": "qemu_diff_m33",
    "args": ["--classes", "fpu"],
    "requires": "fpu_class",
}
QEMU_BASE_ROUTE = {"oracle": "qemu_diff_m33", "args": ["--classes", "base"]}


# ---------------------------------------------------------------------
# Function-name extraction
# ---------------------------------------------------------------------

class FunctionNameExtractionTests(unittest.TestCase):
    def test_function_name_present(self):
        m = make_mutant(function_name="fp_add")
        self.assertEqual(runner.extract_function_name(m), "fp_add")

    def test_qualified_method_name(self):
        m = make_mutant(function_name="CortexM33::fpu_unary")
        self.assertEqual(
            runner.extract_function_name(m), "CortexM33::fpu_unary",
        )

    def test_function_field_absent(self):
        m = make_mutant(function_name=None)  # no `function` key
        self.assertIsNone(runner.extract_function_name(m))

    def test_function_field_malformed(self):
        m = make_mutant()
        m["function"] = {"return_type": "-> usize"}  # no function_name
        self.assertIsNone(runner.extract_function_name(m))


# ---------------------------------------------------------------------
# Route resolution
# ---------------------------------------------------------------------

class RouteResolutionTests(unittest.TestCase):
    def test_in_table_function(self):
        routing = make_routing(
            by_function={
                "crates/mdrp2350/src/core/execute_fpu.rs": {
                    "fp_add": [SOFTFLOAT_ROUTE],
                },
            },
        )
        m = make_mutant(function_name="fp_add")
        routes = runner.resolve_routes(m, routing, capabilities=set())
        self.assertEqual(len(routes), 1)
        self.assertEqual(routes[0]["oracle"], "softfloat_diff")

    def test_out_of_table_function_falls_back_to_file_default(self):
        # Function `unknown_helper` is not in by_function; file has
        # default_oracles_by_file → use file default.
        routing = make_routing(
            by_function={
                "crates/mdrp2350/src/core/execute_fpu.rs": {
                    "fp_add": [SOFTFLOAT_ROUTE],
                },
            },
            default_oracles_by_file={
                "crates/mdrp2350/src/core/execute_fpu.rs": [SOFTFLOAT_ROUTE],
            },
        )
        m = make_mutant(function_name="unknown_helper")
        routes = runner.resolve_routes(m, routing, capabilities=set())
        self.assertEqual(len(routes), 1)
        self.assertEqual(routes[0]["oracle"], "softfloat_diff")

    def test_module_scope_mutant_uses_file_default(self):
        # No function_name (the 16 module-scope mutants in HLD §3.3).
        routing = make_routing(
            default_oracles_by_file={
                "crates/mdrp2350/src/core/execute_fpu.rs": [SOFTFLOAT_ROUTE],
            },
        )
        m = make_mutant(function_name=None)
        routes = runner.resolve_routes(m, routing, capabilities=set())
        self.assertEqual(len(routes), 1)
        self.assertEqual(routes[0]["oracle"], "softfloat_diff")

    def test_out_of_table_file_returns_empty(self):
        routing = make_routing()
        m = make_mutant(
            file="crates/mdrp2350/src/bus/mod.rs",
            function_name="some_fn",
        )
        routes = runner.resolve_routes(m, routing, capabilities=set())
        self.assertEqual(routes, [])

    def test_windows_path_separator_normalised(self):
        routing = make_routing(
            by_function={
                "crates/mdrp2350/src/core/execute_fpu.rs": {
                    "fp_add": [SOFTFLOAT_ROUTE],
                },
            },
        )
        # cargo-mutants on Windows occasionally emits backslashes.
        m = make_mutant(
            file="crates\\mdrp2350\\src\\core\\execute_fpu.rs",
            function_name="fp_add",
        )
        routes = runner.resolve_routes(m, routing, capabilities=set())
        self.assertEqual(len(routes), 1)


# ---------------------------------------------------------------------
# Capability gating
# ---------------------------------------------------------------------

class CapabilityGatingTests(unittest.TestCase):
    def test_required_capability_present(self):
        routing = make_routing(
            by_function={
                "crates/mdrp2350/src/core/execute_fpu.rs": {
                    "vfp_sd": [QEMU_FPU_ROUTE],
                },
            },
        )
        m = make_mutant(function_name="vfp_sd")
        routes = runner.resolve_routes(
            m, routing, capabilities={"fpu_class"},
        )
        self.assertEqual(len(routes), 1)

    def test_required_capability_missing(self):
        routing = make_routing(
            by_function={
                "crates/mdrp2350/src/core/execute_fpu.rs": {
                    "vfp_sd": [QEMU_FPU_ROUTE],
                },
            },
        )
        m = make_mutant(function_name="vfp_sd")
        routes = runner.resolve_routes(m, routing, capabilities=set())
        # Route is dropped → empty list. Caller emits oracle_unavailable.
        self.assertEqual(routes, [])

    def test_mixed_routes_keeps_ungated(self):
        routing = make_routing(
            by_function={
                "crates/mdrp2350/src/core/execute_fpu.rs": {
                    "fpu_v8m_dp": [SOFTFLOAT_ROUTE, QEMU_FPU_ROUTE],
                },
            },
        )
        m = make_mutant(function_name="fpu_v8m_dp")
        routes = runner.resolve_routes(m, routing, capabilities=set())
        self.assertEqual(len(routes), 1)
        self.assertEqual(routes[0]["oracle"], "softfloat_diff")


# ---------------------------------------------------------------------
# Aggregation (HLD §3.7)
# ---------------------------------------------------------------------

def _rr(classification, oracle="softfloat_diff", args=None):
    return runner.RouteResult(
        oracle=oracle,
        args=args or [],
        classification=classification,
        fuzz_count=0,
        wall_seconds=0.0,
        exit_code=None,
        notes="",
    )


class AggregationTests(unittest.TestCase):
    def test_any_caught(self):
        routes = [_rr("oracle_caught"), _rr("oracle_survived")]
        self.assertEqual(
            runner.aggregate_classification(routes), "oracle_caught",
        )

    def test_all_survived(self):
        routes = [_rr("oracle_survived"), _rr("oracle_survived")]
        self.assertEqual(
            runner.aggregate_classification(routes), "oracle_survived",
        )

    def test_mixed_survived_unavailable_picks_survived(self):
        # The HLD §3.7 subtle case: at least one route measured.
        routes = [_rr("oracle_survived"), _rr("oracle_unavailable")]
        self.assertEqual(
            runner.aggregate_classification(routes), "oracle_survived",
        )

    def test_all_unavailable(self):
        routes = [_rr("oracle_unavailable"), _rr("oracle_unavailable")]
        self.assertEqual(
            runner.aggregate_classification(routes), "oracle_unavailable",
        )

    def test_caught_and_unavailable(self):
        routes = [_rr("oracle_caught"), _rr("oracle_unavailable")]
        self.assertEqual(
            runner.aggregate_classification(routes), "oracle_caught",
        )

    def test_build_failed_dominates(self):
        routes = [_rr("oracle_survived"), _rr("build_failed")]
        self.assertEqual(
            runner.aggregate_classification(routes), "build_failed",
        )

    def test_empty_routes_is_skip(self):
        # No routes at all (e.g. file not in routing table).
        self.assertEqual(
            runner.aggregate_classification([]), "skip_no_oracle",
        )


# ---------------------------------------------------------------------
# Backward compat (sidecar absent / disabled)
# ---------------------------------------------------------------------

class BackwardCompatTests(unittest.TestCase):
    def test_load_routing_missing_path_returns_fallback(self):
        # When the sidecar doesn't exist on disk, load_routing returns
        # a routing dict synthesised from ORACLE_FOR_FILE, so legacy
        # per-file behaviour is preserved.
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "no_such_file.json"
            routing = runner.load_routing(missing)
        self.assertEqual(routing["version"], 1)
        # Each file in ORACLE_FOR_FILE must resolve to a single route
        # naming the legacy oracle.
        for file_rel, oracle in runner.ORACLE_FOR_FILE.items():
            routes = routing["default_oracles_by_file"].get(file_rel)
            self.assertIsNotNone(
                routes, f"missing default for {file_rel}",
            )
            self.assertEqual(len(routes), 1)
            self.assertEqual(routes[0]["oracle"], oracle)
        # by_function should be empty in the fallback.
        self.assertEqual(routing["by_function"], {})

    def test_legacy_per_file_behaviour_preserved(self):
        # With sidecar fallback, every file in ORACLE_FOR_FILE
        # resolves to its legacy oracle, regardless of function name.
        with tempfile.TemporaryDirectory() as tmp:
            missing = Path(tmp) / "absent.json"
            routing = runner.load_routing(missing)

        for file_rel, legacy_oracle in runner.ORACLE_FOR_FILE.items():
            m = make_mutant(file=file_rel, function_name="some_fn")
            routes = runner.resolve_routes(
                m, routing, capabilities=set(),
            )
            self.assertEqual(len(routes), 1)
            self.assertEqual(routes[0]["oracle"], legacy_oracle)

    def test_load_routing_real_sidecar(self):
        # The shipped sidecar must parse cleanly, version == 1, and
        # contain entries for every file ORACLE_FOR_FILE knows about.
        sidecar = HERE / "v2_oracle_routing.json"
        if not sidecar.exists():
            self.skipTest(f"sidecar not present at {sidecar}")
        routing = runner.load_routing(sidecar)
        self.assertEqual(routing["version"], 1)
        for file_rel in runner.ORACLE_FOR_FILE:
            self.assertIn(
                file_rel, routing["default_oracles_by_file"],
                f"sidecar missing default for {file_rel}",
            )

    def test_load_routing_invalid_json_falls_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "bad.json"
            bad.write_text("{ not valid json")
            routing = runner.load_routing(bad)
        # Falls back to in-code ORACLE_FOR_FILE.
        self.assertEqual(routing["version"], 1)
        self.assertIn(
            "crates/mdrp2350/src/core/execute_fpu.rs",
            routing["default_oracles_by_file"],
        )


if __name__ == "__main__":
    unittest.main()

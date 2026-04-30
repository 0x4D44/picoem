# V2 Mutation Testing — Operator Notes

Companion to `wrk_docs/2026.04.27 - HLD - Mutation Testing V1 Triage.md` §5
(V1 follow-up: build the second-pass oracle runner) and
`wrk_docs/2026.04.29 - HLD - Mutation Testing V2 Triage.md`.

## What this is

V1 ran `cargo-mutants` against seven decode/execute files (4480 mutants in
the V2 codebase) and bucketed the survivors by hand-inspection. V2 takes
each missed mutant, applies the patch, runs the matching differential
oracle (`qemu_diff_m33` / `qemu_diff_m0plus` / `softfloat_diff`) for a
bounded fuzz window, and records oracle_caught vs oracle_survived per
mutant.

## Files

- `scripts/v2_mutation_runner.py` — main per-mutant runner.
- `scripts/v2_mutation_summary.py` — aggregate `results.jsonl` into
  per-file / per-classification / per-oracle counts.
- `scripts/v2_mutation_run.sh` — high-level launcher
  (`first-pass | deep-pass | status`).
- `mutation/` — gitignored working dir:
  - `mutants_catalog.json` — full catalog from `cargo mutants --list --json --diff`.
  - `sweep/mutants.out/` — cargo-mutants sweep output (caught/missed/timeout).
  - `results.jsonl` — V2 per-mutant outcomes (one JSON per line).
  - `v2_run.log`, `v2_deep.log` — runner stdout/stderr.

## Typical run

```bash
# 1) Regenerate V1 sweep (or use existing mutants.out)
cargo mutants --jobs 8 --output mutation/v2/sweep/ \
    -- -- --skip <test1> --skip <test2>

# 2) V2 first-pass: bounded fuzz window per missed mutant
./scripts/v2_mutation_run.sh first-pass

# 3) Status / summary while running
./scripts/v2_mutation_run.sh status
python3 scripts/v2_mutation_summary.py

# 4) After first-pass completes, deep-pass survivors at higher fuzz
./scripts/v2_mutation_run.sh deep-pass
```

## Per-mutant pipeline

1. Look up the mutant by `name` in `mutation/v2/mutants_catalog.json`.
2. Compute byte offsets `[start_col-1, end_col-1)` from the JSON span;
   splice the source file with `replacement` from the JSON.
3. `cargo build --release -p mdpicoem-harness --bin <oracle>` —
   incremental compile of the chip lib + relink (~18 s).
4. Run the oracle with `--fuzz N` plus per-oracle CLI quirks
   (`qemu_diff_m33 --classes base`, `softfloat_diff --mode all`).
5. Revert the source file (always; `try`/`finally`).
6. Append JSON record to `mutation/v2/results.jsonl`.

## Oracle routing

Routing is now driven by **`scripts/v2_oracle_routing.json`** — a JSON
sidecar supporting per-file defaults plus per-function overrides. See
`wrk_docs/2026.04.29 - HLD - V2 Per-Function Oracle Routing V1.md` for
the full schema and rationale.

Per-file defaults (the fallback when no per-function entry exists):

| File                                 | Oracle             | Args                |
|---                                   |---                 |---                  |
| `mdrp2350/core/execute_thumb32.rs`   | `qemu_diff_m33`    | `--classes base`    |
| `mdrp2350/core/execute_fpu.rs`       | `softfloat_diff`   | `--mode all`        |
| `mdrp2350/core/execute.rs`           | `qemu_diff_m33`    | `--classes base`    |
| `mdrp2350/core/decode.rs`            | `qemu_diff_m33`    | `--classes base`    |
| `mdrp2040/core/execute.rs`           | `qemu_diff_m0plus` | (none)              |
| `mdrp2040/core/execute_wide.rs`      | `qemu_diff_m0plus` | (none)              |
| `mdrp2040/core/decode.rs`            | `qemu_diff_m0plus` | (none)              |

Per-function overrides for `execute_fpu.rs`: IEEE-754 helpers (`fp_add`,
`fp_sub`, `fp_mul`, `fp_div`, `fp_fma`, `fp_sqrt`, `fpu_unary`, the f16
/ f32 conversions, the rounding-mode helpers, NaN canonicalisers) route
to `softfloat_diff` only. VFP-encoding helpers (`vfp_sd`, `vfp_sn`,
`vfp_sm`, `fpscr_set_nzcv`, `vfp_expand_imm_f32`) route to
`qemu_diff_m33 --classes fpu` only — these are observable only when an
actual VFP instruction executes. Dispatch / public-API entries
(`fpu_v8m_dp`, `fpu_data_processing`, `fpu_reg_transfer`, etc.) route to
**both** (caught-if-any).

### Routing-related CLI flags

- `--routing <path>` — path to the JSON sidecar (default
  `scripts/v2_oracle_routing.json`).
- `--no-routing` — disable the sidecar; use the in-code
  `ORACLE_FOR_FILE` fallback only.
- `--allow-fpu` — force `fpu_class` capability ON regardless of the
  smoke probe.
- `--no-fpu` — force `fpu_class` capability OFF regardless of the smoke
  probe.
- `--allow-fpu` and `--no-fpu` are mutually exclusive.

### FPU-class smoke probe

When the loaded routing table has any route with `requires:
"fpu_class"`, the runner spawns one `qemu_diff_m33 --classes fpu --fuzz
1` at startup with a **10-second wall-clock cap**. Pass criteria: exit
in {0, 1} within the cap (oracle ran cleanly OR caught a diff — both
prove the FPU class isn't EAGAIN-stuck). Fail criteria: timeout, spawn
error, or exit code outside {0, 1}.

On pass, `fpu_class` is in the capabilities set and the relevant routes
run as configured. On fail, those routes record a new outcome
**`oracle_unavailable`** (distinct from `oracle_survived` — the gap is
on the oracle side, not the mutation). The smoke probe runs once per
runner invocation.

If the FPU class is broken in your environment (QEMU 8.2 + mps2-an505
has been observed to hang on FPU cases), expect `oracle_unavailable`
rows for the encoding-helper mutants. They count as "needs better
oracle", not "equivalent mutation" — the V2 triage should subtract
them from the Bucket 4 total.

### Aggregation rules (multi-route mutants)

| Route mix                              | Aggregate          |
|---                                     |---                 |
| Any route is `oracle_caught`           | `oracle_caught`    |
| Any route is `build_failed`            | `build_failed`     |
| Any route is `oracle_survived`         | `oracle_survived`  |
| All routes are `oracle_unavailable`    | `oracle_unavailable` |
| Otherwise                              | `error`            |

The mixed `oracle_unavailable + oracle_survived → oracle_survived` rule:
if at least one oracle measured the mutant and reported survived,
that's a real measurement. The unavailable route is recorded inside
`routes[]` for triage when the oracle becomes available.

### Cross-port coordination caveat

When the FPU-class capability is healthy AND the parallel launcher
(`scripts/v2_mutation_run_parallel.sh`) is in use, the per-function
routing in `execute_fpu.rs` may push a mutant from the softfloat worker
(W2) onto a `qemu_diff_m33 --classes fpu` invocation that races with
W0 on GDB port 3333. **This is not an issue today** because the FPU
smoke probe fails on the current host and routes degrade to
`oracle_unavailable`. When the FPU env is repaired (QEMU 10.2 / pinned
host), a separate work-package will add launcher-level coordination.
See HLD V1 §10.6 for the deferred design.

## Cancellation safety

The runner uses `try/finally` around every mutation+build+oracle cycle.
`SIGINT` propagates through Python's KeyboardInterrupt and the `finally`
runs — source file reverts. `SIGKILL` skips the finally; if the runner
is force-killed, run `git checkout -- crates/...` to clean up the
mutated file.

## Restart-friendly

`--skip-done` reads `results.jsonl` and dedups already-tested mutant
names. Pause / restart / re-launch with the same flags safely.

## Known limitations

1. **QEMU 8.2 mps2-an505 FPU class is broken in this env**. FPU cases
   all `[SKIP]` with EAGAIN on the GDB pipe; the run hangs waiting for
   timeout. `qemu_diff_m33 --classes base` pinned in the runner. FPU
   mutations land in `softfloat_diff`, which doesn't run the encoding
   helpers (`vfp_sd`, `vfp_sn`, `vfp_sm`, `fpscr_set_nzcv`) — those
   appear `oracle_survived` even when the mutation is meaningful. Note
   such cases as Bucket 4 by manual inspection or wait for a
   QEMU-10.2 environment.

2. **Per-mutant rebuild cost**. ~18 s baked-in; cannot be amortised
   without per-worker scratch trees (cargo-mutants-style). Sequential
   795 × 35 s ≈ 8 h on this host. Future improvement: 4-way parallel
   via `git worktree`.

3. **Single-shot fuzz budget**. First-pass `--fuzz 200` may miss
   rarely-triggered paths. The `--retry-survivors` deep-pass at
   `--fuzz 2000` is the safety net.

4. **Cargo-mutants `--` separator quirk**. `cargo mutants -- --skip X`
   passes `--skip X` to cargo (which rejects it). Use `cargo mutants
   -- -- --skip X` so libtest gets the flag.

5. **Two cargo-mutants baseline-blocking failing tests** as of
   2026-04-29: `bfc_full_width_clears_word` and
   `coresight_trace_halfword_read_dispatches_through_byte_path`. Per
   `tech_debt.md` § "Residue-test failures discovered by V2 mutation
   sweep". `--skip` them at cargo-mutants invocation until fixed.

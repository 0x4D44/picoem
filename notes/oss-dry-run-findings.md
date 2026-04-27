# Open-source dry-run findings

**Date:** 2026-04-26

## Setup

Ran the dry run inside the workspace, under:

`D:\language\mdrp2354\wrk_scratch\oss-dry-run-20260426-232206\mdpicoem-test`

Command shape:

```bash
git clone --recursive file:///d/language/mdrp2354 mdpicoem-test
cargo build --release
cargo test
```

Cloned commit:

`1cb26ee mutation_testing: add cargo-mutants config + HLD V1 (Stage 1)`

Clone status after checkout: clean.

## Important limitation

The source workspace currently has many uncommitted and untracked release-prep
files. A local `git clone file:///...` only includes committed Git state, so
this dry run did **not** include the newly-written release-prep files such as
`CONTRIBUTING.md`, `SECURITY.md`, `CODE_OF_CONDUCT.md`, `CHANGELOG.md`, or the
latest NOTICE / README edits.

That means this dry run validates the committed build/test baseline, not the
full intended open-source release tree.

## Results

`cargo build --release`: passed.

Observed warning:

- `crates/mdpicoem-harness/src/bin/probe_csrrw_riscv32.rs:30` has an unused
  import: `OPC_LOAD`.

`cargo test`: passed.

Observed warnings:

- `crates/mdrp2040/src/tests.rs:5181` uses non-snake-case helper
  `SSPICR_OFFSET`.
- `crates/mdrp2350/src/core_riscv/tests_p5.rs:16` has an unused `Bus` import.
- `crates/mdrp2350/src/tests.rs:4975` has an unused `Memory` import.
- `crates/mdrp2350/src/core_riscv/tests_p4.rs:230` has an unnecessary `mut`.
- `crates/mdrp2350/src/tests.rs:20100` has an unnecessary `mut`.
- `crates/mdrp2350app/tests/smoke.rs:159` ignores the `Result` from
  `emu.step()`.
- `crates/mdpicoem-harness/src/onerom_glue_dma.rs:771` ignores the `Result`
  from `emu.run(1)`.
- The same `OPC_LOAD` unused import appears in the
  `probe_csrrw_riscv32` test build.

## README gaps

No README-only build gap was encountered for the requested clone/build/test
path. The recursive clone initialized submodules and both Cargo commands ran
without needing `CLAUDE.md` or an out-of-tree convention.

The limitation above still matters: the release-prep documentation itself was
not present in the cloned tree because it is not committed.

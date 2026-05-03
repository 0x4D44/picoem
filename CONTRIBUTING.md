# Contributing to mdpicoem

Thanks for your interest in mdpicoem. This is a personal research
project, not a sponsored or commercial codebase, so PR review and
issue triage are best-effort — please be patient.

## Project status and scope

mdpicoem is an actively-developed cycle-accurate emulator workspace
for the Raspberry Pi RP2350 / RP2354 (dual Cortex-M33 + PIO) and
RP2040 (dual Cortex-M0+ + PIO). The Arm-mode cores are the focus; the
RP2350 RISC-V Hazard3 cores are present and partially functional but
out of scope for active development at the moment.

If you're adding a new peripheral or substantial subsystem, please
open an issue first to discuss the approach. For bug fixes, small
docs improvements, and clearly-scoped enhancements, a pull request
straight away is fine.

## Building and testing

Standard Rust workflow. The workspace pins MSRV at `1.88` and edition
`2024` — older toolchains will not compile.

```bash
# Build everything (release profile is strongly recommended).
cargo build --release

# Unit tests.
cargo test
cargo test -p rp2350-emu      # RP2350 only
cargo test -p rp2040-emu      # RP2040 only

# Code coverage.
cargo llvm-cov
```

`cargo build -p epio-sys` is opt-in: it requires `clang` and
initialised git submodules. A plain `cargo build` at the workspace
root skips it via `default-members`.

### Differential oracles

Several oracles validate the emulator against external references.
The QEMU oracles (`qemu_diff_m33`, `qemu_diff_m0plus`) need a QEMU
build with Cortex-M33 and `cortex-m0` machine support. They are the
ones an outside contributor is most likely to be able to run; see the
"Testing" section of `README.md` for the catalogue and the
"Differential Fuzz Testing" section for typical session shapes.

### Hardware-only oracles

The `probe_*`, `silicon_*`, `bank_conflict_*`, and `onerom_*` binaries
require a physical Raspberry Pi debug probe attached to an RP2354,
RP2040, or OneROM rig. They will not run in CI and they will not run
on a host without the required hardware. If you don't have the rig,
that's fine — these oracles are not gating for most contributions.

The probe-serial → DUT mapping for the developer's rig is in
`docs/probe_serials.md`; that file documents the canonical mapping
used in the codebase but is specific to the maintainer's hardware.

## Commit messages

Use plain present-tense imperative summaries (e.g. "fix dma channel
arbitration", not "fixed" or "fixes"). For commits that touch a
specific subsystem, prefix the summary line with that subsystem's
short name where it helps readability — e.g. `rp2350-emu: ...`,
`harness: ...`, `picogus: ...`. Don't go out of your way to retrofit
a prefix when a commit spans subsystems.

Co-author trailers are welcomed (`Co-Authored-By: Name <email>`) but
not required.

## Coding style

- Standard `cargo fmt`. CI doesn't enforce it because we have no CI;
  please run it locally before submitting.
- `cargo clippy --all-targets` should be clean. There are a few
  known warnings on the existing tree that we have not yet quietened;
  don't introduce new ones if you can avoid it.
- Prefer `tracing` over `eprintln!` / `println!` for diagnostic output
  in library crates. The level guidance is in `CLAUDE.md` under
  "Logging & Tracing"; same conventions apply to contributors.
- We do **not** put SPDX license headers on individual source files.
  The workspace `LICENSE-MIT` and `LICENSE-APACHE` files cover all
  source files in this repository.

## Tests

New code should come with tests. Unit tests live in-crate
(`crates/<name>/src/tests.rs` etc.); integration tests live under
`crates/<name>/tests/`. For instruction-semantics work, the
differential oracles (QEMU diff, silicon diff) are the canonical
backstop — be ready to run those before claiming "this is correct."

## Licensing of contributions

Unless you explicitly state otherwise, any contribution you intentionally
submit for inclusion in this repository, as defined in the Apache-2.0
license, will be dual-licensed as **MIT OR Apache-2.0** without any
additional terms or conditions. See `LICENSE-MIT` and `LICENSE-APACHE`.

# Local patches against upstream `probe-rs` 0.31.0

This directory is a **vendored fork** of [probe-rs](https://github.com/probe-rs/probe-rs)
crate `probe-rs` version `0.31.0` (published 2026-01-17 on crates.io).

Upstream is dual-licensed MIT OR Apache-2.0; the local copies of those
licenses are at `LICENSE-MIT` and `LICENSE-APACHE` in this directory and
travel with the source. Our local patches are likewise MIT OR Apache-2.0
under the same dual-licensing terms used by the rest of `mdpicoem`.

The fork is wired in from the workspace root via:

```toml
[patch.crates-io]
probe-rs = { path = "third_party/probe-rs-0.31.0-mdrp-patched" }
```

so any workspace crate depending on `probe-rs = "0.31"` resolves to this
copy. Only the harness binaries actually take this dependency; the
`mdrp2350` and `mdrp2040` library crates do not depend on probe-rs and
are unaffected by this fork when they are published to crates.io.

## Patch list

### P1 — DPv3 cache upgrade fallback for ADIv6 targets

**File:** `src/architecture/arm/communication_interface.rs`
**Function:** `select_ap_and_ap_bank`
**Upstream issues:** [probe-rs#3872](https://github.com/probe-rs/probe-rs/issues/3872),
[probe-rs#3257](https://github.com/probe-rs/probe-rs/issues/3257)
**Local design doc:** `wrk_docs/2026.04.21 - HLD - Track A Probe-rs Attach Fix.md`

**Symptom upstream produces.** When the very first DPIDR read on an ADIv6
target reports anything other than `version == DPv3` (typically because of a
stale pipelined read or a transient SWD line glitch on the alert sequence),
the `SelectCache` is left in `DPv1` form. Any subsequent access to a V2 AP
(e.g. RP2354 core 0 at `ApV2Address(0x2000)`) then matches the `(V2, DPv1)`
arm of the cross-product `match` in `select_ap_and_ap_bank`, which upstream
implements as `unreachable!()`, panicking the caller. The issue is open
upstream as of probe-rs 0.31.0 (no fix merged on `master`) and is reproduced
by at least one other vendor (Nordic nRF9151) per #3257.

**What the patch does.** Replaces the panicking arm with an in-place
upgrade of the `SelectCache` to `DPv3` and proceeds along the V2 path.
The outer `previous_select != current_select` check immediately below
flushes the freshly-built SELECT / SELECT1 registers to the DP on the
same call, so the upgrade has the same on-the-wire effect as a fresh DP
insertion that read DPv3 cleanly the first time.

**Diff size.** Approximately 30 lines added to a single file; no other
files are modified. The added block carries an in-source comment block
pointing back to upstream issue #3872 and to our HLD.

**Behavioural impact when the fork is removed.** If `[patch.crates-io]`
is dropped and the workspace builds against stock `probe-rs 0.31.0`, the
hardware oracles `silicon_periph_diff_rp2350`, `silicon_dualcore_diff_rp2350`,
`silicon_isr_diff_rp2350`, `bank_conflict_test_rp2350`, and `probe_verify_rp2350`
will panic on attach approximately one run in N (depending on probe and
host timing). `probe_diff_rp2350` and `probe_diff_rp2040` are unaffected
because their attach path does not access the V2 AP that triggers the
panic.

## Removing this fork in the future

We expect to drop the fork once probe-rs ships an upstream fix for
issue #3872. The upgrade path is:

1. Bump probe-rs to the upstream-fixed release in
   `crates/mdpicoem-harness/Cargo.toml`.
2. Remove the `[patch.crates-io]` block from the workspace root
   `Cargo.toml`.
3. Delete this directory.

No API surface is changed by the patch, so the rest of the workspace
should not need code changes for the upgrade.

# Security policy

## Reporting a vulnerability

picoem is a research-grade emulator: it executes potentially
untrusted firmware images, but inside a userspace simulator on a
developer's host. The realistic security surface is small —
predominantly:

- Memory-safety issues in the emulator itself when fed adversarial
  firmware images (a crash, sandbox escape, or arbitrary host code
  execution from inside emulated code).
- Bugs in the harness binaries that talk to real hardware via
  `probe-rs`, where a maliciously-crafted target could in principle
  affect the host.

If you believe you have found a security issue in either of those
categories, please email **martin@tollens.ai** with the details
rather than opening a public GitHub issue. Plain-text email is fine;
PGP is not used.

This is a personal project maintained best-effort. A reasonable
expectation for a first response is **a few business days**, with
remediation to follow on whatever timeline matches the severity.

## What is in scope

- The `rp2350-emu`, `rp2040-emu`, `picoem-common`, and `picoem-devices`
  library crates.
- The `rp2350-emu-tui` and `rp2040-emu-tui` TUI applications.
- The `picoem-harness` binaries, including the QEMU and probe-rs
  differential oracles.
- The vendored `probe-rs` fork
  (`third_party/probe-rs-0.31.0-mdrp-patched/`) **only insofar as the
  local patch is concerned** — see that directory's `PATCHES.md`.
  Pre-existing probe-rs issues should be reported upstream at
  <https://github.com/probe-rs/probe-rs>.

## What is out of scope

- Firmware images and trace fixtures redistributed under
  `roms/`, `third_party/picogus/`, and
  `crates/picoem-harness/fixtures/`. Issues in upstream firmware
  should be reported to those upstream projects.
- Issues found in the `epio-sys` vendored upstream sources. Report
  those to the upstream `epio` / `apio` repositories.
- Anything requiring physical access to the developer's hardware
  rig (e.g. RP2354 silicon-level attacks). Those are interesting but
  not in the threat model of an emulator project.

## Coordinated disclosure

If a security issue is reported and fixed, the fix will land in a
public commit on the main branch. There is no separate security
release branch. CVE assignment is at the discretion of the maintainer
based on severity.

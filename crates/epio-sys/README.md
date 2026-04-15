# epio-sys

Low-level Rust bindings for Piers Finlayson's [`epio`] cycle-accurate
RP2350 PIO emulator, vendored under `third_party/`. Used exclusively by
the OneROM PIO differential oracle under `mdpicoem-harness`.

This crate is **excluded from `workspace.default-members`** — it requires
clang and vendored submodules. Build it explicitly:

```
cargo build -p epio-sys --release
```

## First-time setup

```
git submodule update --init --recursive
```

## Required toolchain

- `clang` on PATH (Windows: typically `C:\Program Files\LLVM\bin\`).
  Rust's `cc` crate is invoked with `.compiler("clang")` — there is no
  MSVC or gcc fallback.

## Design

See `wrk_docs/2026.04.14 - HLD - OneROM PIO Differential.md`.

[`epio`]: https://github.com/piersfinlayson/epio

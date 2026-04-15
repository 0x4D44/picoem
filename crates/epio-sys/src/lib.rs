//! Low-level Rust bindings to Piers Finlayson's `epio` — a cycle-accurate
//! RP2350 PIO emulator. Used only by the OneROM PIO differential oracle.
//!
//! See `wrk_docs/2026.04.14 - HLD - OneROM PIO Differential.md` for the
//! design of the test harness that consumes this crate.
//!
//! This crate is intentionally outside `workspace.default-members` because
//! it requires clang and vendored submodules under `third_party/`.

#![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]

// Low-level epio API bindings.
include!(concat!(env!("OUT_DIR"), "/epio_bindings.rs"));

// First-party C shim — bridges Rust to apio + epio for the OneROM scenario.
pub mod shim {
    #![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]
    include!(concat!(env!("OUT_DIR"), "/shim_bindings.rs"));
}

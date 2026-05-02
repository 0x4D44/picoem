#![cfg(target_arch = "x86_64")]
//! Thread-coordination primitives shared across chip emulators.
//!
//! Platform-gated on x86_64 because `SpscQueue` relies on x86 TSO for
//! its `Relaxed` atomics to be free of fences. The chip-crate threaded
//! runtimes (`rp2350_emu::threaded::emulator`, `rp2040_emu::threaded::emulator`)
//! layer Windows + Linux thread-affinity pinning on top of the
//! primitives here; macOS / other UNIX hosts still need a portable
//! `pin_to_host_core` before they can light up.
//!
//! Promoted from `rp2350_emu::threaded::{barrier,spsc}` as part of
//! Stage 3a of the dual-execution HLD (see
//! `wrk_docs/2026.04.24 - HLD - Dual Serial and Threaded Execution Models V1.md`
//! §6.4 step 1). Chip-specific bundles (`CoreAtomics`, `WorkerBus`,
//! `SharedState`, `ExclusiveMonitors`) stay in the chip crates.

pub mod barrier;
pub mod spsc;
// Worker-thread helpers (`panic_message`, `spawn_worker`,
// `pin_to_host_core`) — promoted from the chip emulators per the
// 2026-04-30 Threaded Helpers Pull-Up HLD V1. The affinity FFI inside
// `pin_to_host_core` only resolves on Windows or Linux, so the module
// itself is gated on those operating systems; the parent module is
// already x86_64-gated by the inner `#![cfg(...)]` above.
#[cfg(any(target_os = "windows", target_os = "linux"))]
pub mod worker;

pub use barrier::{BarrierResult, SpinBarrier};
pub use spsc::SpscQueue;
#[cfg(any(target_os = "windows", target_os = "linux"))]
pub use worker::{panic_message, pin_to_host_core, spawn_worker};

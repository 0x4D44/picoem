#![cfg(target_arch = "x86_64")]
//! Thread-coordination primitives shared across chip emulators.
//!
//! Platform-gated on x86_64 because `SpscQueue` relies on x86 TSO for
//! its `Relaxed` atomics to be free of fences. The chip-crate threaded
//! runtimes (`mdrp2350::threaded::emulator`, `mdrp2040::threaded::emulator`)
//! layer Windows + Linux thread-affinity pinning on top of the
//! primitives here; macOS / other UNIX hosts still need a portable
//! `pin_to_host_core` before they can light up.
//!
//! Promoted from `mdrp2350::threaded::{barrier,spsc}` as part of
//! Stage 3a of the dual-execution HLD (see
//! `wrk_docs/2026.04.24 - HLD - Dual Serial and Threaded Execution Models V1.md`
//! §6.4 step 1). Chip-specific bundles (`CoreAtomics`, `WorkerBus`,
//! `SharedState`, `ExclusiveMonitors`) stay in the chip crates.

pub mod barrier;
pub mod spsc;

pub use barrier::{BarrierResult, SpinBarrier};
pub use spsc::SpscQueue;

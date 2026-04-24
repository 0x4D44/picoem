#![cfg(all(target_arch = "x86_64", target_os = "windows"))]
//! Thread-coordination primitives shared across chip emulators.
//!
//! Platform-gated on x86_64 Windows per HLD §3 non-goals — mirrors the
//! gate on `mdrp2350::threaded::emulator` / `mdrp2350::threaded::timings`
//! and the upcoming `mdrp2040::threaded` surface. Non-Windows builds see
//! no symbols from this module and stay on the serial `Emulator::run`
//! path.
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

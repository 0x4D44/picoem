//! Threaded primitives for Phase 2 of the dual-core emulation effort.
//!
//! Types in this module are standalone building blocks — they will be
//! composed into `SharedState` by Phase 3's `ThreadedEmulator`. None of
//! them are wired into the existing serial-interleave step path yet.
//!
//! See `wrk_docs/2026.04.17 - LLD - Threaded Dual-Core Phase 2 V4.md`.

pub mod atomics;
pub mod gpio;
pub mod memory;
pub mod monitors;
// `barrier` + `spsc` were promoted to `mdpicoem-common::threaded` as
// Stage 3a of the dual-execution HLD V1 (§6.4 step 1). The re-exports
// below keep every existing `crate::threaded::{SpinBarrier, SpscQueue,
// BarrierResult}` call site source-compatible.
pub mod bus;
pub mod peripherals;
pub mod pio;
pub mod shared;
pub mod sio;
// Stage 6b (LLD V7 §8/§9): `ThreadedEmulator` pins one thread per
// worker via `SetThreadAffinityMask`, so the whole module is gated to
// x86_64 Windows. Non-Windows callers continue on the serial
// `Emulator::run` path. Dual-execution HLD V1 (Stage 1b) layered the
// `threading` cargo feature on top: both gates must be satisfied for
// the threaded runtime to exist.
#[cfg(all(feature = "threading", target_arch = "x86_64", target_os = "windows"))]
pub mod emulator;
// Per-worker per-quantum timing instrumentation (HLD V7 §8 follow-up).
// Gated to the same target as `emulator` because only the threaded
// runtime produces timings.
#[cfg(all(feature = "threading", target_arch = "x86_64", target_os = "windows"))]
pub mod timings;

pub use atomics::CoreAtomics;
pub use gpio::AtomicGpio;
pub use memory::SharedMemory;
pub use monitors::ExclusiveMonitors;
// Re-exported from `mdpicoem-common::threaded` (Stage 3a). Chip-local
// call sites keep using `crate::threaded::{SpinBarrier, BarrierResult,
// SpscQueue}` unchanged.
pub use bus::{PioBus, WorkerBus};
#[cfg(all(feature = "threading", target_arch = "x86_64", target_os = "windows"))]
pub use emulator::{RunError, ThreadedEmulator};
pub use mdpicoem_common::threaded::{BarrierResult, SpinBarrier, SpscQueue};
pub use peripherals::{
    ApbState, ClocksState, DmaState, Peripherals, QmiState, ResetsState, TimersState, UsbState,
};
pub use pio::{PioCommand, ThreadedPio};
pub use shared::SharedState;
pub use sio::ThreadedSio;
#[cfg(all(feature = "threading", target_arch = "x86_64", target_os = "windows"))]
pub use timings::{RunTimings, WorkerName, WorkerSummary};

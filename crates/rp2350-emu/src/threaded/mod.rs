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
// `barrier` + `spsc` were promoted to `picoem-common::threaded` as
// Stage 3a of the dual-execution HLD V1 (§6.4 step 1). The re-exports
// below keep every existing `crate::threaded::{SpinBarrier, SpscQueue,
// BarrierResult}` call site source-compatible.
pub mod bus;
pub mod peripherals;
pub mod pio;
pub mod shared;
pub mod sio;
// Stage 6b (LLD V7 §8/§9): `ThreadedEmulator` pins one thread per
// worker via the host's affinity API (`SetThreadAffinityMask` on
// Windows; `pthread_setaffinity_np` on Linux). Other UNIX hosts
// (macOS, FreeBSD, …) stay on the serial `Emulator::run` path until
// `pin_to_host_core` grows a port. Dual-execution HLD V1 (Stage 1b)
// layered the `threading` cargo feature on top: both gates must be
// satisfied for the threaded runtime to exist.
#[cfg(all(
    feature = "threading",
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
pub mod emulator;
// Per-worker per-quantum timing instrumentation (HLD V7 §8 follow-up).
// Gated to the same target as `emulator` because only the threaded
// runtime produces timings.
#[cfg(all(
    feature = "threading",
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
pub mod timings;

pub use atomics::CoreAtomics;
pub use gpio::AtomicGpio;
pub use memory::SharedMemory;
pub use monitors::ExclusiveMonitors;
// Re-exported from `picoem-common::threaded` (Stage 3a). Chip-local
// call sites keep using `crate::threaded::{SpinBarrier, BarrierResult,
// SpscQueue}` unchanged.
pub use bus::WorkerBus;
#[cfg(all(
    feature = "threading",
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
pub use emulator::{RunError, ThreadedEmulator};
pub use picoem_common::threaded::{BarrierResult, SpinBarrier, SpscQueue};
// Worker-thread helpers (panic_message, spawn_worker, pin_to_host_core)
// were promoted from `threaded::emulator` to `picoem-common::threaded`
// per the 2026-04-30 Threaded Helpers Pull-Up HLD V1. These re-exports
// keep the chip-local call sites that reach them via `super::{...}`
// from `threaded::emulator` source-compatible.
#[cfg(all(
    feature = "threading",
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
pub use picoem_common::threaded::{panic_message, pin_to_host_core, spawn_worker};
pub use peripherals::{
    ApbState, ClocksState, DmaState, Peripherals, QmiState, ResetsState, TimersState, UsbState,
};
pub use pio::{PioCommand, ThreadedPio};
pub use shared::SharedState;
pub use sio::ThreadedSio;
#[cfg(all(
    feature = "threading",
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
pub use timings::{RunTimings, WorkerName, WorkerSummary};

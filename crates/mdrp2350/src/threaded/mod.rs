//! Threaded primitives for Phase 2 of the dual-core emulation effort.
//!
//! Types in this module are standalone building blocks — they will be
//! composed into `SharedState` by Phase 3's `ThreadedEmulator`. None of
//! them are wired into the existing serial-interleave step path yet.
//!
//! See `wrk_docs/2026.04.17 - LLD - Threaded Dual-Core Phase 2 V4.md`.

pub mod atomics;
pub mod memory;
pub mod gpio;
pub mod monitors;
pub mod spsc;
pub mod barrier;
pub mod sio;
pub mod pio;
pub mod peripherals;
pub mod shared;
pub mod bus;
// Stage 6b (LLD V7 §8/§9): `ThreadedEmulator` pins one thread per
// worker via `SetThreadAffinityMask`, so the whole module is gated to
// x86_64 Windows. Non-Windows callers continue on the serial
// `Emulator::run` path.
#[cfg(all(target_arch = "x86_64", target_os = "windows"))]
pub mod emulator;

pub use atomics::CoreAtomics;
pub use memory::SharedMemory;
pub use gpio::AtomicGpio;
pub use monitors::ExclusiveMonitors;
pub use spsc::SpscQueue;
pub use barrier::{SpinBarrier, BarrierResult};
pub use sio::ThreadedSio;
pub use pio::{ThreadedPio, PioCommand};
pub use peripherals::{
    ApbState, ClocksState, DmaState, Peripherals, QmiState, ResetsState, TimersState,
};
pub use shared::SharedState;
pub use bus::{PioBus, WorkerBus};
#[cfg(all(target_arch = "x86_64", target_os = "windows"))]
pub use emulator::ThreadedEmulator;

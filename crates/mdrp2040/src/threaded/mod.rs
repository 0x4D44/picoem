#![cfg(all(target_arch = "x86_64", target_os = "windows", feature = "threading"))]
//! RP2040 threaded dual-core execution runtime.
//!
//! Mirrors the mdrp2350 threaded tree but adapted for M0+ — no FPU, no
//! secure world, no coprocessors. 3 workers (core0, core1, coordinator)
//! rather than RP2350's 6, because M0+ has no PIO-as-worker split at this
//! stage (Stage 4 may split if benchmarks reveal PIO as a bottleneck).
//!
//! Stage 3b.2 scope: atomics + shared state + memory + `WorkerBus`
//! skeleton. MMIO peripheral routing arrives in Stage 3b.3;
//! `ThreadedEmulator` runtime arrives in Stage 3b.4.
//!
//! See `wrk_docs/2026.04.24 - HLD - Dual Serial and Threaded Execution
//! Models V1.md` §6.4.

pub mod atomics;
pub mod shared;
pub mod memory;
pub mod bus;

// Stage 3b.4 will add emulator.rs (runtime).

pub use mdpicoem_common::threaded::{BarrierResult, SpinBarrier, SpscQueue};

pub use atomics::CoreAtomics;
pub use shared::SharedState;
pub use memory::SharedMemory;
pub use bus::WorkerBus;

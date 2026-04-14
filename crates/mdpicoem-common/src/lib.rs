//! Chip-agnostic primitives shared by `mdrp2040` and `mdrp2350`.
//!
//! See `wrk_docs/2026.04.14 - HLD - mdpicoem Workspace Restructure.md` for
//! the split policy: common owns *primitives* (types and pure functions);
//! chip crates own *composed structures* that mix chip-specific state with
//! those primitives.

pub mod clock;
pub mod clocks;
pub mod divider;
pub mod fifo;
pub mod memory;
pub mod pacer;
pub mod pio;

pub use self::clock::Clock;
pub use self::clocks::{ClockTree, ROSC_FREQ_HZ, XOSC_FREQ_HZ, pll_output_hz};
pub use self::divider::Divider;
pub use self::fifo::Fifo;
pub use self::memory::{Memory, ROM_SIZE, SRAM_SIZE};
pub use self::pacer::{PacerSnapshot, PacerStats};
pub use self::pio::PioBlock;
#[cfg(target_arch = "x86_64")]
pub use self::pacer::Pacer;

/// Trait for memory-mapped peripherals. Implemented by chip-specific
/// peripheral crates; the common crate defines only the interface.
pub trait Peripheral {
    fn read32(&mut self, offset: u32) -> u32;
    fn write32(&mut self, offset: u32, value: u32);
    /// Called once per system clock. Return true if interrupt asserted.
    fn step(&mut self) -> bool;
}

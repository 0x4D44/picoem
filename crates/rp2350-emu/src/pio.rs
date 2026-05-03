//! Chip-side PIO re-exports.
//!
//! The PIO primitive (block, state machine, ISA decoder) lives in
//! [`picoem_common::pio`]. RP2350 embeds three of these blocks in its
//! `Bus`; the block count is chip-specific state, the type is shared.

pub use picoem_common::pio::*;

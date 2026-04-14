//! Chip-side clock-tree re-exports.
//!
//! The pure-math primitives (`ClockTree`, `pll_output_hz`, `ROSC_FREQ_HZ`,
//! `XOSC_FREQ_HZ`) live in [`mdpicoem_common::clocks`]. This module
//! re-exports them so the `Bus` register-write path and any tests that
//! reference the old `crate::bus::clocks::*` paths keep compiling.
//!
//! The chip-specific register-storage and `recompute_clock_tree` logic
//! stays on [`crate::bus::Bus`].

pub use mdpicoem_common::clocks::{ClockTree, ROSC_FREQ_HZ, XOSC_FREQ_HZ, pll_output_hz};

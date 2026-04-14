//! Off-chip peripheral models — devices physically outside the RP2040
//! die that the emulator needs to simulate for a specific board target.
//!
//! Today this carries the PicoGUS v2 SPI PSRAM (`psram`); if/when a
//! second off-chip device becomes needed, we extract shared scaffolding
//! then, not now (premature abstraction resists the correct design).

pub mod psram;

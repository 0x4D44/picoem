//! Off-chip device models for the picoem emulator workspace.
//!
//! This crate holds external (board-level) devices that compose with the
//! chip emulators (`rp2040_emu`, `rp2350_emu`) but are not part of the chips
//! themselves. Each device is a concrete struct with its own API — no
//! trait polymorphism.
//!
//! Devices fall into two categories:
//!
//! * **Pin-driving** (e.g. PSRAM) — participates in the per-cycle GPIO
//!   merge inside the emulator's `update_gpio()`. Hosted on `Bus` as a
//!   typed field.
//! * **Observer** (e.g. LCD decoder, I2S capture) — reads GPIO state from
//!   outside the step loop. Called by the app or harness, not by the
//!   emulator core.

pub mod i2s_capture;
pub mod lcd;
pub mod psram;

pub use i2s_capture::I2sCapture;
pub use lcd::{LcdDecoder, LcdState};
pub use psram::Psram;

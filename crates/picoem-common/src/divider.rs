//! Per-core integer divider state — shared between RP2040 and RP2350.
//!
//! Same 32-bit signed/unsigned divider semantics on both chips
//! (including division-by-zero behaviour). Compute routine stays on the
//! chip-side `Sio` composition; this file lifts the data struct only.
//! Fields were `pub` within the private `sio::mod` module; crossing the
//! crate boundary promotes them to `pub` for cross-crate field access
//! from chip SIO implementations.

/// Per-core integer divider state (HLD §2.4).
#[derive(Default, Clone, Copy)]
pub struct Divider {
    pub dividend: u32,
    pub divisor: u32,
    pub quotient: u32,
    pub remainder: u32,
    pub signed: bool,
    pub dirty: bool,
    pub reads_pending: u8,
}

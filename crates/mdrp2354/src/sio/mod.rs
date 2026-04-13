/// Single-cycle IO block — stub for Phase 1.
///
/// Phase 5 adds spinlocks, FIFOs, doorbells, divider, interpolators.
pub struct Sio {
    /// 32 hardware spinlocks.
    pub spinlocks: [u32; 32],
}

impl Sio {
    pub fn new() -> Self {
        Self {
            spinlocks: [0; 32],
        }
    }
}

impl Default for Sio {
    fn default() -> Self {
        Self::new()
    }
}

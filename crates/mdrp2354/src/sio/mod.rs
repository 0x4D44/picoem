/// Single-cycle IO block.
///
/// GPIO output/OE/input registers are on Bus (region 0xD dispatch).
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

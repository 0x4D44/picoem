//! RP2350 CORESIGHT_TRACE peripheral — HLD V5 §8.E.2.
//!
//! Storage-only model at `CORESIGHT_TRACE_BASE = 0xE004_1000`, inside
//! the ARM CoreSight aperture. No side effects, no trace data produced.
//! The block exists so firmware that programs trace configuration
//! round-trips as expected — otherwise boot-time SWO / TPIU enable
//! paths would hit the HashMap fallthrough with a noisy warn-once.
//!
//! # Storage
//!
//! Simple `HashMap<u32, u32>` keyed on word offset. `apply_alias_rmw`
//! preserves APB SET / CLR / XOR semantics, matching the inert-cluster
//! pattern in `peripherals/inert.rs`.

use std::collections::HashMap;

use super::apply_alias_rmw;

/// CORESIGHT_TRACE base (HLD V5 §8.E.2).
pub const CORESIGHT_TRACE_BASE: u32 = 0xE004_1000;

/// CORESIGHT_TRACE register block — storage only.
pub struct CoresightTraceRegs {
    regs: HashMap<u32, u32>,
}

impl CoresightTraceRegs {
    pub fn new() -> Self {
        Self {
            regs: HashMap::new(),
        }
    }

    /// Read a word. Unwritten offsets read 0.
    pub fn read32(&self, offset: u32) -> u32 {
        *self.regs.get(&offset).unwrap_or(&0)
    }

    /// Write a word using the canonical 2-bit APB alias encoding (see
    /// [`super::apply_alias_rmw`]).
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        let stored = self.regs.entry(offset).or_insert(0);
        apply_alias_rmw(stored, value, alias);
    }
}

impl Default for CoresightTraceRegs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_default_and_aliases() {
        let mut c = CoresightTraceRegs::new();
        // Unwritten reads 0.
        assert_eq!(c.read32(0x10), 0);
        // Plain write.
        c.write32(0x10, 0xDEAD_BEEF, 0);
        assert_eq!(c.read32(0x10), 0xDEAD_BEEF);
        // SET alias ORs in.
        c.write32(0x10, 0x1, 2);
        assert_eq!(c.read32(0x10), 0xDEAD_BEEF | 0x1);
        // CLR alias masks out.
        c.write32(0x10, 0xFFFF_0000, 3);
        assert_eq!(c.read32(0x10), (0xDEAD_BEEF | 0x1) & !0xFFFF_0000);
        // XOR alias.
        c.write32(0x10, 0x000F_000F, 1);
        assert_eq!(
            c.read32(0x10),
            ((0xDEAD_BEEF | 0x1) & !0xFFFF_0000) ^ 0x000F_000F
        );
    }
}

// Placeholder Hazard3 RISC-V core. P1a ships the enum + plumbing only;
// P1b replaces this with the real ISA implementation per
// `wrk_docs/2026.04.17 - HLD - RP2350 RISC-V Hazard3 Core Support.md`.

use crate::Bus;

pub struct Hazard3 {
    cycles: u64,
}

impl Hazard3 {
    pub fn new() -> Self {
        Self { cycles: 0 }
    }

    pub fn step(&mut self, _bus: &mut Bus) {
        self.cycles += 1;
    }

    pub fn cycles(&self) -> u64 {
        self.cycles
    }

    pub fn is_halted(&self) -> bool {
        false
    }
}

impl Default for Hazard3 {
    fn default() -> Self {
        Self::new()
    }
}

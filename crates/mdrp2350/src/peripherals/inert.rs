//! RP2350 "inert" peripheral cluster — SYSCFG, TBMAN, GLITCH_DETECTOR.
//!
//! HLD V5 §7.D.1. Three small APB peripherals that firmware touches for
//! chip identification / debug configuration / glitch-detect status, but
//! whose modelled behaviour reduces to register round-trip plus a
//! handful of always-zero read overrides. Consolidated into one file to
//! avoid three two-page modules that all do the same thing.
//!
//! # Base addresses (assumptions flagged in-code)
//!
//! - `SYSCFG_BASE = 0x4000_8000` — confirmed against
//!   `one-rom/sdrr/include/reg-rp235x.h` (SYSCFG_BASE).
//! - `TBMAN_BASE  = 0x4016_0000` — HLD V5 §7.D.1 pick; datasheet §6
//!   reference unverified in tree. The warn-once infrastructure from
//!   step 1 will catch a firmware hit if this is off.
//! - `GLITCH_DETECTOR_BASE = 0x4015_8000` — HLD V5 §7.D.1 pick;
//!   datasheet §4.4 reference unverified in tree. Same warn-once caveat.
//!
//! # Storage model
//!
//! Each struct carries a `HashMap<u32, u32>` keyed by canonical word
//! offset (alias bits stripped). Write path uses
//! [`super::apply_alias_rmw`] so SET / CLR / XOR land consistently.
//! Read path returns the stored value unless a register has special
//! semantics (see below).
//!
//! # Special semantics
//!
//! - `GLITCH_DETECTOR.STATUS`: `ARM` bit is forced to 0 on read (no
//!   glitch event in emulation). Bit position assumed at bit 0 — if the
//!   datasheet §4.4 assigns a different bit, the warn-once MMIO trace
//!   will flag firmware that polls the wrong bit.
//! - `GLITCH_DETECTOR.TRIG_STATUS`: W1C semantics via
//!   [`super::apply_alias_rmw`] alias=3 (BITCLR). Writes with `alias=0`
//!   simply overwrite (firmware rarely does this but storage mirrors
//!   the silicon behaviour).

use std::collections::HashMap;

use super::apply_alias_rmw;

/// SYSCFG base (one-rom `reg-rp235x.h`).
pub const SYSCFG_BASE: u32 = 0x4000_8000;
/// TBMAN base — HLD V5 §7.D.1 pick. See module-level caveat.
pub const TBMAN_BASE: u32 = 0x4016_0000;
/// GLITCH_DETECTOR base — HLD V5 §7.D.1 pick. See module-level caveat.
pub const GLITCH_DETECTOR_BASE: u32 = 0x4015_8000;

/// GLITCH_DETECTOR register offsets. Layout assumption — see module doc.
const GLITCH_STATUS_OFFSET: u32 = 0x00;
const GLITCH_TRIG_STATUS_OFFSET: u32 = 0x08;
/// `ARM` bit within STATUS — assumption. Datasheet §4.4 was not
/// verified in-tree; if silicon uses a different bit, firmware
/// polling for `(STATUS & (1 << N)) == 0` still sees "disarmed" only
/// for `N == 0`. Warn-once catches the rest.
const GLITCH_STATUS_ARM_BIT: u32 = 0;

/// SYSCFG — storage-only APB peripheral at `0x4000_8000`.
pub struct SysCfg {
    regs: HashMap<u32, u32>,
}

impl SysCfg {
    pub fn new() -> Self {
        Self { regs: HashMap::new() }
    }

    /// Read a word from SYSCFG. Unwritten offsets read 0.
    pub fn read32(&self, offset: u32) -> u32 {
        *self.regs.get(&offset).unwrap_or(&0)
    }

    /// Write a word to SYSCFG with the canonical APB alias encoding
    /// (`alias` in 0..=3 — see [`super::apply_alias_rmw`]).
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        let stored = self.regs.entry(offset).or_insert(0);
        apply_alias_rmw(stored, value, alias);
    }
}

impl Default for SysCfg {
    fn default() -> Self {
        Self::new()
    }
}

/// TBMAN — test-bench manager, storage-only at `0x4016_0000`.
pub struct Tbman {
    regs: HashMap<u32, u32>,
}

impl Tbman {
    pub fn new() -> Self {
        Self { regs: HashMap::new() }
    }

    pub fn read32(&self, offset: u32) -> u32 {
        *self.regs.get(&offset).unwrap_or(&0)
    }

    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        let stored = self.regs.entry(offset).or_insert(0);
        apply_alias_rmw(stored, value, alias);
    }
}

impl Default for Tbman {
    fn default() -> Self {
        Self::new()
    }
}

/// GLITCH_DETECTOR — storage plus `STATUS.ARM == 0` and `TRIG_STATUS`
/// W1C. See module doc for the caveats on base and bit layout.
pub struct GlitchDetector {
    regs: HashMap<u32, u32>,
}

impl GlitchDetector {
    pub fn new() -> Self {
        Self { regs: HashMap::new() }
    }

    /// Read a word. `STATUS.ARM` is always reported as 0 in emulation.
    pub fn read32(&self, offset: u32) -> u32 {
        let stored = *self.regs.get(&offset).unwrap_or(&0);
        if offset == GLITCH_STATUS_OFFSET {
            stored & !(1u32 << GLITCH_STATUS_ARM_BIT)
        } else {
            stored
        }
    }

    /// Write a word. `TRIG_STATUS` uses W1C semantics — a plain write
    /// (`alias == 0`) is reinterpreted as BITCLR (`alias == 3`) so that
    /// firmware writing `1` to a bit clears it. Alias-addressed writes
    /// (SET / CLR / XOR) pass through unchanged.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        let effective_alias = if offset == GLITCH_TRIG_STATUS_OFFSET && alias == 0 {
            3 // W1C
        } else {
            alias
        };
        let stored = self.regs.entry(offset).or_insert(0);
        apply_alias_rmw(stored, value, effective_alias);
    }
}

impl Default for GlitchDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn syscfg_roundtrip() {
        let mut s = SysCfg::new();
        s.write32(0x0C, 0xDEAD_BEEF, 0);
        assert_eq!(s.read32(0x0C), 0xDEAD_BEEF);
        // SET alias ORs in
        s.write32(0x0C, 0x0000_0001, 2);
        assert_eq!(s.read32(0x0C), 0xDEAD_BEEF | 0x1);
        // CLR alias masks out
        s.write32(0x0C, 0xFFFF_0000, 3);
        assert_eq!(s.read32(0x0C), (0xDEAD_BEEF | 0x1) & !0xFFFF_0000);
        // Unwritten offset reads 0
        assert_eq!(s.read32(0x40), 0);
    }

    #[test]
    fn tbman_roundtrip() {
        let mut t = Tbman::new();
        t.write32(0x00, 0x0000_0001, 0);
        assert_eq!(t.read32(0x00), 0x0000_0001);
        t.write32(0x00, 0x0000_0003, 1); // XOR
        assert_eq!(t.read32(0x00), 0x0000_0001 ^ 0x0000_0003);
        assert_eq!(t.read32(0x10), 0);
    }

    #[test]
    fn glitch_detector_roundtrip_non_status() {
        let mut g = GlitchDetector::new();
        // Arbitrary non-STATUS/non-TRIG offset stores plainly.
        g.write32(0x04, 0x1234_5678, 0);
        assert_eq!(g.read32(0x04), 0x1234_5678);
    }

    #[test]
    fn glitch_detector_status_arm_reads_zero() {
        let mut g = GlitchDetector::new();
        // Force ARM bit on via plain write, confirm read masks it.
        g.write32(GLITCH_STATUS_OFFSET, 0xFFFF_FFFF, 0);
        let status = g.read32(GLITCH_STATUS_OFFSET);
        assert_eq!(
            status & (1u32 << GLITCH_STATUS_ARM_BIT),
            0,
            "ARM bit must read 0 even when stored value has it set"
        );
        // Other bits survive.
        assert_eq!(status | (1u32 << GLITCH_STATUS_ARM_BIT), 0xFFFF_FFFF);
    }

    #[test]
    fn glitch_detector_trig_status_w1c() {
        let mut g = GlitchDetector::new();
        // Seed TRIG_STATUS via SET alias (explicit).
        g.write32(GLITCH_TRIG_STATUS_OFFSET, 0x0000_000F, 2);
        assert_eq!(g.read32(GLITCH_TRIG_STATUS_OFFSET), 0x0000_000F);
        // Plain write of 0x3 clears those bits (W1C).
        g.write32(GLITCH_TRIG_STATUS_OFFSET, 0x0000_0003, 0);
        assert_eq!(g.read32(GLITCH_TRIG_STATUS_OFFSET), 0x0000_000C);
    }
}

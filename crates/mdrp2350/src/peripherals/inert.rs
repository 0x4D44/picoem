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
//! `SysCfg` and `GlitchDetector` each carry a `HashMap<u32, u32>` keyed
//! by canonical word offset (alias bits stripped); their write path uses
//! [`super::apply_alias_rmw`] so SET / CLR / XOR land consistently, and
//! their read path returns the stored value unless a register has special
//! semantics (see below).
//!
//! `Tbman` is storage-free: the pico-sdk header documents exactly one
//! register (`PLATFORM` at 0x00, a silicon strap — architecturally RO),
//! so reads dispatch directly on `offset` and writes are no-ops.
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

/// TBMAN.PLATFORM offset. Register layout per pico-sdk
/// `src/rp2350/hardware_regs/include/hardware/regs/tbman.h`
/// (`TBMAN_PLATFORM_OFFSET`). `pub` so the harness (`silicon_scenarios`)
/// can import the same symbol instead of redeclaring.
pub const TBMAN_PLATFORM_OFFSET: u32 = 0x00;
/// TBMAN.PLATFORM reset value on real RP2354 silicon: ASIC bit (bit 0)
/// set, FPGA (bit 1) and HDLSIM (bit 2) clear. Source:
///
///   https://raw.githubusercontent.com/raspberrypi/pico-sdk/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2350/hardware_regs/include/hardware/regs/tbman.h
///
///   #define TBMAN_PLATFORM_RESET       _u(0x00000001)
///   #define TBMAN_PLATFORM_ASIC_BITS   _u(0x00000001)
///
/// Matches HLD Coverage Gap Fill V11 §3.4 "assumption 0b01".
const TBMAN_PLATFORM_RESET: u32 = 0x0000_0001;

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

/// TBMAN — test-bench manager at `0x4016_0000`. The pico-sdk header
/// documents exactly one register (`PLATFORM` at offset 0x00, a 3-bit
/// RO selector distinguishing ASIC / FPGA / HDLSIM). Every other offset
/// in the block is unmapped fabric and reads as 0 on real silicon.
///
/// Storage-free: reads dispatch directly on `offset`, writes are no-ops.
/// PLATFORM is architecturally read-only (silicon strap, not a register),
/// so accepting writes anywhere in the TBMAN window would diverge from
/// hardware and there's no meaningful state to retain.
pub struct Tbman;

impl Tbman {
    pub fn new() -> Self {
        Self
    }

    /// Read a word. `PLATFORM` (offset 0x00) returns the silicon-observed
    /// reset value — pico-sdk `TBMAN_PLATFORM_RESET = 0x1` (ASIC bit set,
    /// FPGA + HDLSIM clear). All other offsets read 0 (unmapped fabric).
    pub fn read32(&self, offset: u32) -> u32 {
        if offset == TBMAN_PLATFORM_OFFSET {
            TBMAN_PLATFORM_RESET
        } else {
            0
        }
    }

    /// Write a word. TBMAN has no writable state on real silicon —
    /// PLATFORM is a strap, not a register — so all writes are silently
    /// discarded. `_alias` is accepted to match the peripheral dispatch
    /// contract (`Bus::write32` always passes the alias bits) but has
    /// no effect.
    pub fn write32(&mut self, _offset: u32, _value: u32, _alias: u32) {
        // No-op: TBMAN exposes no writable state.
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
    fn tbman_platform_reads_silicon_reset() {
        // PLATFORM (offset 0x00) returns `TBMAN_PLATFORM_RESET = 0x1`
        // from pico-sdk's tbman.h — ASIC bit set, FPGA + HDLSIM clear.
        // Writes anywhere in the TBMAN window are no-ops, so writing
        // garbage to PLATFORM *and* to a non-PLATFORM offset must leave
        // both the silicon-reset override and the unmapped-reads-as-0
        // contract intact.
        let mut t = Tbman::new();
        assert_eq!(t.read32(TBMAN_PLATFORM_OFFSET), TBMAN_PLATFORM_RESET);
        // PLATFORM is architecturally RO — write must not alter read.
        t.write32(TBMAN_PLATFORM_OFFSET, 0xFFFF_FFFF, 0);
        assert_eq!(
            t.read32(TBMAN_PLATFORM_OFFSET),
            TBMAN_PLATFORM_RESET,
            "PLATFORM is silicon-RO; write must not alter read value"
        );
        // Unmapped offsets must read 0 regardless of prior writes — no
        // stray state leaks from `write32` into subsequent reads.
        t.write32(0x04, 0xDEAD_BEEF, 0);
        assert_eq!(t.read32(0x04), 0, "unmapped TBMAN offsets read 0");
        assert_eq!(t.read32(0x10), 0, "unmapped TBMAN offsets read 0");
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

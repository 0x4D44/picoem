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
//! - `GLITCH_DETECTOR.ARM` (offset 0x00): plain RW storage. On first
//!   read (before any write) the register returns the silicon reset
//!   value `GLITCH_DETECTOR_ARM_RESET = 0x5bad` (= `ARM_VALUE_NO` —
//!   "do not force the detectors to be armed"). After a write, readback
//!   returns the stored value, matching silicon: firmware that writes
//!   `ARM_VALUE_YES = 0x0000` (force-armed) or any non-`0x5bad` sentinel
//!   sees that same value on the next read.
//!
//!   Per pico-sdk `glitch_detector.h`:
//!
//!   ```text
//!   https://raw.githubusercontent.com/raspberrypi/pico-sdk/a1438dff1d38bd9c65dbd693f0e5db4b9ae91779/src/rp2350/hardware_regs/include/hardware/regs/glitch_detector.h
//!
//!   #define GLITCH_DETECTOR_ARM_OFFSET    _u(0x00000000)
//!   #define GLITCH_DETECTOR_ARM_BITS      _u(0x0000ffff)
//!   #define GLITCH_DETECTOR_ARM_RESET     _u(0x00005bad)
//!   #define GLITCH_DETECTOR_ARM_VALUE_NO  _u(0x5bad)
//!   #define GLITCH_DETECTOR_ARM_VALUE_YES _u(0x0000)
//!   ```
//!
//!   The ARM register is **not** protected by a separate password field —
//!   a single 16-bit value serves as both the arm command and its own
//!   integrity check (any value other than `0x5bad` counts as "force
//!   arm"). The emulator therefore accepts any write as silicon does;
//!   there is no password to model. The HLD §3.3 draft used the
//!   shorthand "CTRL.ARM bit" / "STATUS.ARM bit" — on real silicon ARM
//!   is one RW register that reads back what was written, so "ARM
//!   readback tracks CTRL" reduces to "ARM is plain storage with a
//!   non-zero reset value".
//!
//!   Note: the header marks ARM as "Secure read/write only". The
//!   emulator does not enforce Secure-only access at this level (the
//!   ACCESSCTRL / SAU model is out of V11 scope, Bucket C); a future
//!   Non-Secure write that silicon would RAZ/WI will currently sink
//!   into the HashMap. Coverage gap logged in HLD §5 (Bucket C).
//! - `GLITCH_DETECTOR.TRIG_STATUS` (offset 0x10): W1C semantics via
//!   [`super::apply_alias_rmw`] alias=3 (BITCLR). Writes with `alias=0`
//!   are reinterpreted as BITCLR so firmware writing `1` to a bit
//!   clears it (per `GLITCH_DETECTOR_TRIG_STATUS_*_ACCESS = "WC"`).
//!   Reads return 0 in emulation — no glitch ever fires, consistent
//!   with HLD §3.3 ("TRIG_STATUS stays at 0").

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

/// GLITCH_DETECTOR register offsets (pico-sdk
/// `src/rp2350/hardware_regs/include/hardware/regs/glitch_detector.h` —
/// pin commit `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`).
///
/// `pub` so the harness (`silicon_scenarios`) can import the same
/// symbols instead of redeclaring them — Stage 2 (TBMAN) precedent.
pub const GLITCH_DETECTOR_ARM_OFFSET: u32 = 0x00;
pub const GLITCH_DETECTOR_DISARM_OFFSET: u32 = 0x04;
pub const GLITCH_DETECTOR_SENSITIVITY_OFFSET: u32 = 0x08;
pub const GLITCH_DETECTOR_LOCK_OFFSET: u32 = 0x0C;
pub const GLITCH_DETECTOR_TRIG_STATUS_OFFSET: u32 = 0x10;
pub const GLITCH_DETECTOR_TRIG_FORCE_OFFSET: u32 = 0x14;

/// ARM register field mask and named values. `ARM` is a 16-bit field;
/// writing `VALUE_NO = 0x5bad` disarms, any other value force-arms.
/// This is the silicon's "password-like" integrity check — see module
/// doc. No upper-word password; the sentinel IS the protection.
pub const GLITCH_DETECTOR_ARM_MASK: u32 = 0x0000_FFFF;
pub const GLITCH_DETECTOR_ARM_VALUE_NO: u32 = 0x0000_5BAD;
pub const GLITCH_DETECTOR_ARM_VALUE_YES: u32 = 0x0000_0000;
/// Reset value of ARM on real silicon — "force-arm NO". Firmware that
/// reads ARM before touching it sees this; firmware that writes any
/// value other than `ARM_VALUE_NO` observes it on readback.
pub const GLITCH_DETECTOR_ARM_RESET: u32 = GLITCH_DETECTOR_ARM_VALUE_NO;

/// SYSCFG — storage-only APB peripheral at `0x4000_8000`.
pub struct SysCfg {
    regs: HashMap<u32, u32>,
}

impl SysCfg {
    pub fn new() -> Self {
        Self {
            regs: HashMap::new(),
        }
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

/// GLITCH_DETECTOR — ARM readback + TRIG_STATUS W1C-reads-zero. See
/// module doc for the full register map and the silicon-reset rationale.
pub struct GlitchDetector {
    regs: HashMap<u32, u32>,
}

impl GlitchDetector {
    pub fn new() -> Self {
        // Seed the silicon reset value for ARM so the first read (before
        // any firmware write) reports "do not force-arm" — matching
        // `GLITCH_DETECTOR_ARM_RESET = 0x5bad`. Other registers reset to
        // 0 (the HashMap default via `unwrap_or(&0)` in `read32`).
        let mut regs = HashMap::new();
        regs.insert(GLITCH_DETECTOR_ARM_OFFSET, GLITCH_DETECTOR_ARM_RESET);
        Self { regs }
    }

    /// Reset all GLITCH_DETECTOR state — used by [`crate::Emulator::reset`].
    /// After a watchdog-driven warm reset on real silicon, ARM returns to
    /// `GLITCH_DETECTOR_ARM_RESET = 0x5bad` regardless of what firmware
    /// last wrote. Drop the backing HashMap and re-seed the reset value
    /// by reconstructing the peripheral from `new()`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Read a word.
    ///
    /// - `ARM`, `DISARM`, `SENSITIVITY`, `LOCK`, `TRIG_FORCE` — return
    ///   stored value. ARM reads `GLITCH_DETECTOR_ARM_RESET` if
    ///   firmware has not written since reset (seeded by `new()`).
    /// - `TRIG_STATUS` — always reads 0: no glitch ever fires in
    ///   emulation, so there are no pending-trigger bits to report.
    pub fn read32(&self, offset: u32) -> u32 {
        if offset == GLITCH_DETECTOR_TRIG_STATUS_OFFSET {
            0
        } else {
            *self.regs.get(&offset).unwrap_or(&0)
        }
    }

    /// Write a word.
    ///
    /// - `TRIG_STATUS` uses W1C semantics — a plain write (`alias == 0`)
    ///   is reinterpreted as BITCLR (`alias == 3`) so that firmware
    ///   writing `1` to a bit clears it. Alias-addressed writes (SET /
    ///   CLR / XOR) pass through unchanged. TRIG_STATUS storage is
    ///   never observable via `read32` (which returns 0) but is kept in
    ///   the HashMap for consistency with other registers.
    /// - All other offsets: plain alias RMW into backing storage. ARM
    ///   writes therefore round-trip exactly on readback, per HLD §3.3.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        let effective_alias = if offset == GLITCH_DETECTOR_TRIG_STATUS_OFFSET && alias == 0 {
            3 // W1C
        } else {
            alias
        };
        let stored = self.regs.entry(offset).or_insert(0);
        apply_alias_rmw(stored, value, effective_alias);
        // ARM is a 16-bit field (GLITCH_DETECTOR_ARM_BITS = 0x0000_FFFF);
        // upper bits are RAZ/WI on silicon. `apply_alias_rmw` has no field
        // knowledge, so mask junk upper bits out post-write so they don't
        // persist in backing storage and resurface on readback.
        if offset == GLITCH_DETECTOR_ARM_OFFSET {
            *stored &= GLITCH_DETECTOR_ARM_MASK;
        }
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
    fn glitch_detector_arm_reset_value() {
        // Before any write, ARM reads the silicon reset value 0x5bad
        // (force-arm NO). This is seeded in `new()`; if the seed is
        // ever removed, firmware that polls ARM before writing will
        // see 0 instead of 0x5bad and silently diverge from silicon.
        let g = GlitchDetector::new();
        assert_eq!(
            g.read32(GLITCH_DETECTOR_ARM_OFFSET),
            GLITCH_DETECTOR_ARM_RESET,
            "ARM reset value must be 0x5bad (ARM_VALUE_NO)"
        );
    }

    #[test]
    fn glitch_detector_arm_roundtrip() {
        let mut g = GlitchDetector::new();
        // Write "force-arm YES" (0x0000) — must read back unchanged.
        g.write32(GLITCH_DETECTOR_ARM_OFFSET, GLITCH_DETECTOR_ARM_VALUE_YES, 0);
        assert_eq!(
            g.read32(GLITCH_DETECTOR_ARM_OFFSET),
            GLITCH_DETECTOR_ARM_VALUE_YES
        );
        // Write "force-arm NO" (0x5bad) — must read back unchanged.
        g.write32(GLITCH_DETECTOR_ARM_OFFSET, GLITCH_DETECTOR_ARM_VALUE_NO, 0);
        assert_eq!(
            g.read32(GLITCH_DETECTOR_ARM_OFFSET),
            GLITCH_DETECTOR_ARM_VALUE_NO
        );
    }

    #[test]
    fn glitch_detector_other_offsets_roundtrip() {
        let mut g = GlitchDetector::new();
        // DISARM, SENSITIVITY, LOCK: plain RW storage. Probe one value.
        g.write32(GLITCH_DETECTOR_DISARM_OFFSET, 0x0000_DCAF, 0);
        assert_eq!(g.read32(GLITCH_DETECTOR_DISARM_OFFSET), 0x0000_DCAF);
        g.write32(GLITCH_DETECTOR_SENSITIVITY_OFFSET, 0xDE00_C3CC, 0);
        assert_eq!(g.read32(GLITCH_DETECTOR_SENSITIVITY_OFFSET), 0xDE00_C3CC);
    }

    #[test]
    fn glitch_detector_trig_status_reads_zero() {
        let mut g = GlitchDetector::new();
        // TRIG_STATUS reads 0 in emulation regardless of any stored
        // bits — no glitch ever fires. Seed via explicit SET alias,
        // then confirm the read-override still reports 0.
        g.write32(GLITCH_DETECTOR_TRIG_STATUS_OFFSET, 0x0000_000F, 2);
        assert_eq!(g.read32(GLITCH_DETECTOR_TRIG_STATUS_OFFSET), 0);
    }

    #[test]
    fn glitch_detector_trig_status_w1c_on_plain_write() {
        let mut g = GlitchDetector::new();
        // Seed TRIG_STATUS via SET alias (explicit).
        g.write32(GLITCH_DETECTOR_TRIG_STATUS_OFFSET, 0x0000_000F, 2);
        // Plain write of 0x3 is reinterpreted as BITCLR (W1C).
        // TRIG_STATUS itself reads 0, so verify via the storage map
        // directly — a regression that dropped W1C would show up when
        // firmware later aliased the bits back on (a real scenario).
        g.write32(GLITCH_DETECTOR_TRIG_STATUS_OFFSET, 0x0000_0003, 0);
        let stored = *g
            .regs
            .get(&GLITCH_DETECTOR_TRIG_STATUS_OFFSET)
            .unwrap_or(&0);
        assert_eq!(
            stored, 0x0000_000C,
            "W1C must clear DET0+DET1; DET2+DET3 survive"
        );
    }
}

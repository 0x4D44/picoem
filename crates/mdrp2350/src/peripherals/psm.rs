//! RP2350 PSM (Power-on State Machine) — HLD V5 §7.D.2.
//!
//! PSM sequences the bring-up of on-chip subsystems (PROC0/PROC1, BUSFAB,
//! XIP, PIO, etc.). Real silicon asserts `DONE` bits as each subsystem
//! completes its ramp. The emulator models this as an instant handshake:
//! `DONE` mirrors `FRCE_ON` on read. Firmware that waits
//! `while ((PSM_DONE & mask) != mask)` sees completion on the next read
//! after it writes `FRCE_ON`. This is a documented limitation — real
//! silicon has per-subsystem ramp latency (on the order of µs).
//!
//! # Register map
//!
//! | Offset | Name      | Access | Notes                          |
//! |--------|-----------|--------|--------------------------------|
//! | `0x00` | `FRCE_ON` | RW     | Plain storage (alias-aware).   |
//! | `0x04` | `FRCE_OFF`| RW     | Plain storage.                 |
//! | `0x08` | `WDSEL`   | RW     | Plain storage.                 |
//! | `0x0C` | `DONE`    | RO     | Mirrors FRCE_ON (instant).     |
//!
//! Offsets are taken from `one-rom/sdrr/include/reg-rp235x.h` (PSM_BASE
//! plus offsets for FRCE_OFF at `0x004` etc.) — FRCE_ON at `0x000`,
//! FRCE_OFF at `0x004`, WDSEL at `0x008`, DONE at `0x00C`. If the
//! datasheet §13 layout differs, warn-once catches firmware hits on the
//! missing offset.
//!
//! Base address `PSM_BASE = 0x4001_8000` — confirmed against
//! `reg-rp235x.h`.

use super::apply_alias_rmw;

/// PSM base (one-rom `reg-rp235x.h`).
pub const PSM_BASE: u32 = 0x4001_8000;

const FRCE_ON_OFFSET: u32 = 0x00;
const FRCE_OFF_OFFSET: u32 = 0x04;
const WDSEL_OFFSET: u32 = 0x08;
const DONE_OFFSET: u32 = 0x0C;

/// PSM register block.
pub struct Psm {
    frce_on: u32,
    frce_off: u32,
    wdsel: u32,
}

impl Psm {
    pub fn new() -> Self {
        Self {
            frce_on: 0,
            frce_off: 0,
            wdsel: 0,
        }
    }

    /// Read a word. `DONE` mirrors `FRCE_ON` (instant handshake).
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            FRCE_ON_OFFSET => self.frce_on,
            FRCE_OFF_OFFSET => self.frce_off,
            WDSEL_OFFSET => self.wdsel,
            DONE_OFFSET => self.frce_on, // instant mirror
            _ => 0,
        }
    }

    /// Write a word with canonical APB alias encoding.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        match offset {
            FRCE_ON_OFFSET => apply_alias_rmw(&mut self.frce_on, value, alias),
            FRCE_OFF_OFFSET => apply_alias_rmw(&mut self.frce_off, value, alias),
            WDSEL_OFFSET => apply_alias_rmw(&mut self.wdsel, value, alias),
            DONE_OFFSET => { /* RO — ignore */ }
            _ => { /* unmodelled — drop */ }
        }
    }
}

impl Default for Psm {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn done_mirrors_frce_on() {
        let mut psm = Psm::new();
        assert_eq!(psm.read32(DONE_OFFSET), 0);
        psm.write32(FRCE_ON_OFFSET, 0x0000_00FF, 0);
        assert_eq!(psm.read32(FRCE_ON_OFFSET), 0x0000_00FF);
        assert_eq!(psm.read32(DONE_OFFSET), 0x0000_00FF);
        // SET alias adds bits; DONE follows.
        psm.write32(FRCE_ON_OFFSET, 0x0000_0F00, 2);
        assert_eq!(psm.read32(DONE_OFFSET), 0x0000_0FFF);
    }

    #[test]
    fn frce_off_and_wdsel_roundtrip() {
        let mut psm = Psm::new();
        psm.write32(FRCE_OFF_OFFSET, 0xAAAA_5555, 0);
        psm.write32(WDSEL_OFFSET, 0x5555_AAAA, 0);
        assert_eq!(psm.read32(FRCE_OFF_OFFSET), 0xAAAA_5555);
        assert_eq!(psm.read32(WDSEL_OFFSET), 0x5555_AAAA);
        // DONE is independent of FRCE_OFF / WDSEL.
        assert_eq!(psm.read32(DONE_OFFSET), 0);
    }

    #[test]
    fn done_is_read_only() {
        let mut psm = Psm::new();
        psm.write32(DONE_OFFSET, 0xDEAD_BEEF, 0);
        // DONE write ignored — still mirrors FRCE_ON (== 0).
        assert_eq!(psm.read32(DONE_OFFSET), 0);
    }
}

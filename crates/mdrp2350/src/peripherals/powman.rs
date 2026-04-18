//! RP2350 POWMAN peripheral — HLD V5 §8.E.1.
//!
//! Storage-only model covering three sub-areas:
//!
//! 1. **AON timer — storage-only.** `COUNT` (64-bit) and `MATCH` (64-bit)
//!    are plain-storage register pairs. `STATUS.READY` always reads 1.
//!    The timer does not advance in emulation — firmware that programs
//!    `MATCH` and waits for a wake event will never fire. The warn-once
//!    on first `MATCH` write and first `COUNT` write (HLD V5 §4.A2 sites
//!    7 & 8) is the signal to implement clocked advancement if real
//!    firmware hits it.
//! 2. **VREG.** Plain storage.
//! 3. **ARCHSEL.** Plain storage. Reset default is Arm (0); warn-once on
//!    first write that changes the stored value to a non-Arm selection
//!    (HLD V5 §4.A2 site 6).
//!
//! # Base address
//!
//! `POWMAN_BASE = 0x4010_0000` — **assumption**. RP2350 datasheet §12
//! POWMAN was not verified in-tree; `one-rom/sdrr/include/reg-rp235x.h`
//! is not present on the build host. If the assumption is wrong, the
//! Step 1 warn-once MMIO trace flags the real base on first firmware
//! hit. Base updates here + in `crate::bus` match arms only — storage
//! model is base-independent.
//!
//! # Register map (assumptions flagged)
//!
//! | Offset   | Name            | Access | Notes                        |
//! |----------|-----------------|--------|------------------------------|
//! | `0x00`   | `CTRL`          | R/W    | Plain storage                |
//! | `0x04`   | `STATUS`        | R      | `READY` bit forced 1         |
//! | `0x08`   | `AON_COUNT_LO`  | R/W    | Low 32 of 64-bit COUNT       |
//! | `0x0C`   | `AON_COUNT_HI`  | R/W    | High 32                      |
//! | `0x10`   | `AON_MATCH_LO`  | R/W    | Low 32 of 64-bit MATCH       |
//! | `0x14`   | `AON_MATCH_HI`  | R/W    | High 32                      |
//! | `0x18`   | `VREG_CTRL`     | R/W    | Plain storage                |
//! | `0x1C`   | `VREG_STS`      | R/W    | Plain storage                |
//! | `0x20`   | `ARCHSEL`       | R/W    | `ARM=0`; warn on non-Arm     |
//! | other    | —               | R/W    | `HashMap<u32,u32>` fallthrough |
//!
//! Offsets are educated-guess placements inside the POWMAN 4 KB
//! aperture. Firmware round-trip accesses at those offsets work
//! regardless of whether they match silicon exactly; the warn-once
//! sites are triggered on register identity (COUNT vs MATCH vs
//! ARCHSEL) rather than specific bit layouts.
//!
//! # STATUS.READY bit
//!
//! Assumption: bit 0. RP2350 datasheet §12 POWMAN STATUS layout
//! unverified in tree; firmware that polls a different bit for "ready"
//! will hit the HashMap fallthrough behaviour (read returns 0 for
//! unwritten registers). The warn-once MMIO trace is the escape hatch.

use std::collections::HashMap;

use super::apply_alias_rmw;

/// POWMAN base (assumption — see module doc).
pub const POWMAN_BASE: u32 = 0x4010_0000;

const CTRL_OFFSET: u32 = 0x00;
const STATUS_OFFSET: u32 = 0x04;
const AON_COUNT_LO_OFFSET: u32 = 0x08;
const AON_COUNT_HI_OFFSET: u32 = 0x0C;
const AON_MATCH_LO_OFFSET: u32 = 0x10;
const AON_MATCH_HI_OFFSET: u32 = 0x14;
const VREG_CTRL_OFFSET: u32 = 0x18;
const VREG_STS_OFFSET: u32 = 0x1C;
const ARCHSEL_OFFSET: u32 = 0x20;

/// `STATUS.READY` bit position — assumption (see module doc).
const STATUS_READY_BIT: u32 = 1 << 0;

/// Arm default selection in `ARCHSEL` — any non-zero write warns once.
const ARCHSEL_ARM: u32 = 0;

/// POWMAN register block.
pub struct PowmanRegs {
    /// `CTRL` register — plain storage.
    ctrl: u32,
    /// AON timer COUNT low / high 32 bits. No advancement.
    aon_count_lo: u32,
    aon_count_hi: u32,
    /// AON timer MATCH low / high 32 bits. No advancement.
    aon_match_lo: u32,
    aon_match_hi: u32,
    /// `VREG_CTRL` / `VREG_STS` — plain storage.
    vreg_ctrl: u32,
    vreg_sts: u32,
    /// `ARCHSEL` — reset default `ARM=0`.
    archsel: u32,
    /// HashMap fallthrough for offsets outside the modelled set.
    /// Round-trip only — no side effects.
    other: HashMap<u32, u32>,
    /// Warn-once latch: first AON `COUNT` write (HLD V5 §4.A2 site 8).
    warned_aon_count: bool,
    /// Warn-once latch: first AON `MATCH` write (HLD V5 §4.A2 site 7).
    warned_aon_match: bool,
    /// Warn-once latch: first `ARCHSEL` write changing value to non-Arm
    /// (HLD V5 §4.A2 site 6).
    warned_archsel: bool,
}

impl PowmanRegs {
    pub fn new() -> Self {
        Self {
            ctrl: 0,
            aon_count_lo: 0,
            aon_count_hi: 0,
            aon_match_lo: 0,
            aon_match_hi: 0,
            vreg_ctrl: 0,
            vreg_sts: 0,
            archsel: ARCHSEL_ARM,
            other: HashMap::new(),
            warned_aon_count: false,
            warned_aon_match: false,
            warned_archsel: false,
        }
    }

    /// Read a POWMAN register word.
    pub fn read32(&self, offset: u32) -> u32 {
        match offset {
            CTRL_OFFSET => self.ctrl,
            STATUS_OFFSET => STATUS_READY_BIT,
            AON_COUNT_LO_OFFSET => self.aon_count_lo,
            AON_COUNT_HI_OFFSET => self.aon_count_hi,
            AON_MATCH_LO_OFFSET => self.aon_match_lo,
            AON_MATCH_HI_OFFSET => self.aon_match_hi,
            VREG_CTRL_OFFSET => self.vreg_ctrl,
            VREG_STS_OFFSET => self.vreg_sts,
            ARCHSEL_OFFSET => self.archsel,
            _ => *self.other.get(&offset).unwrap_or(&0),
        }
    }

    /// Write a POWMAN register word.
    pub fn write32(&mut self, offset: u32, value: u32, alias: u32) {
        match offset {
            CTRL_OFFSET => apply_alias_rmw(&mut self.ctrl, value, alias),
            STATUS_OFFSET => {
                // Read-only: accept alias-clear for any W1C firmware,
                // otherwise storage round-trip so firmware reads don't
                // surprise. READY is regenerated on read.
                // (No stored state; ignore.)
            }
            AON_COUNT_LO_OFFSET => {
                apply_alias_rmw(&mut self.aon_count_lo, value, alias);
                if !self.warned_aon_count {
                    self.warned_aon_count = true;
                    tracing::warn!(
                        "POWMAN AON COUNT written; timer does not advance"
                    );
                }
            }
            AON_COUNT_HI_OFFSET => {
                apply_alias_rmw(&mut self.aon_count_hi, value, alias);
                if !self.warned_aon_count {
                    self.warned_aon_count = true;
                    tracing::warn!(
                        "POWMAN AON COUNT written; timer does not advance"
                    );
                }
            }
            AON_MATCH_LO_OFFSET => {
                apply_alias_rmw(&mut self.aon_match_lo, value, alias);
                if !self.warned_aon_match {
                    self.warned_aon_match = true;
                    tracing::warn!(
                        "POWMAN AON MATCH programmed; emulator does not advance the timer"
                    );
                }
            }
            AON_MATCH_HI_OFFSET => {
                apply_alias_rmw(&mut self.aon_match_hi, value, alias);
                if !self.warned_aon_match {
                    self.warned_aon_match = true;
                    tracing::warn!(
                        "POWMAN AON MATCH programmed; emulator does not advance the timer"
                    );
                }
            }
            VREG_CTRL_OFFSET => apply_alias_rmw(&mut self.vreg_ctrl, value, alias),
            VREG_STS_OFFSET => apply_alias_rmw(&mut self.vreg_sts, value, alias),
            ARCHSEL_OFFSET => {
                apply_alias_rmw(&mut self.archsel, value, alias);
                if self.archsel != ARCHSEL_ARM && !self.warned_archsel {
                    self.warned_archsel = true;
                    tracing::warn!(
                        archsel = format_args!("{:#X}", self.archsel),
                        "POWMAN ARCHSEL set to non-Arm value; emulator only models Arm"
                    );
                }
            }
            _ => {
                let stored = self.other.entry(offset).or_insert(0);
                apply_alias_rmw(stored, value, alias);
            }
        }
    }
}

impl Default for PowmanRegs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use tracing::{Event, Metadata, Subscriber};
    use tracing::span::{Attributes, Id, Record};

    // --- shared capture subscriber (copied from `bus::bus_observability`,
    // since that module is `#[cfg(test)]` private). ---------------------

    #[derive(Default)]
    struct CaptureSubscriber {
        events: Arc<Mutex<Vec<String>>>,
    }

    struct FieldRecorder(String);
    impl tracing::field::Visit for FieldRecorder {
        fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
            use std::fmt::Write;
            if !self.0.is_empty() {
                self.0.push(' ');
            }
            let _ = write!(self.0, "{}={:?}", field.name(), value);
        }
    }

    impl Subscriber for CaptureSubscriber {
        fn enabled(&self, _metadata: &Metadata<'_>) -> bool { true }
        fn new_span(&self, _span: &Attributes<'_>) -> Id { Id::from_u64(1) }
        fn record(&self, _span: &Id, _values: &Record<'_>) {}
        fn record_follows_from(&self, _span: &Id, _follows: &Id) {}
        fn event(&self, event: &Event<'_>) {
            let mut v = FieldRecorder(String::new());
            event.record(&mut v);
            let meta = event.metadata();
            let line = format!("{} {} {}", meta.level(), meta.target(), v.0);
            self.events.lock().unwrap().push(line);
        }
        fn enter(&self, _span: &Id) {}
        fn exit(&self, _span: &Id) {}
    }

    fn count_warns_containing(events: &[String], needle: &str) -> usize {
        events
            .iter()
            .filter(|line| line.starts_with("WARN"))
            .filter(|line| line.contains(needle))
            .count()
    }

    #[test]
    fn status_ready_always_reads_one() {
        let p = PowmanRegs::new();
        assert_eq!(p.read32(STATUS_OFFSET) & STATUS_READY_BIT, STATUS_READY_BIT);
    }

    #[test]
    fn ctrl_roundtrip() {
        let mut p = PowmanRegs::new();
        p.write32(CTRL_OFFSET, 0xDEAD_BEEF, 0);
        assert_eq!(p.read32(CTRL_OFFSET), 0xDEAD_BEEF);
    }

    #[test]
    fn vreg_roundtrip() {
        let mut p = PowmanRegs::new();
        p.write32(VREG_CTRL_OFFSET, 0xA5A5_A5A5, 0);
        assert_eq!(p.read32(VREG_CTRL_OFFSET), 0xA5A5_A5A5);
        p.write32(VREG_STS_OFFSET, 0x12_3456, 0);
        assert_eq!(p.read32(VREG_STS_OFFSET), 0x12_3456);
    }

    #[test]
    fn aon_count_roundtrip_and_warns_once() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber { events: captured.clone() };
        tracing::subscriber::with_default(subscriber, || {
            let mut p = PowmanRegs::new();
            p.write32(AON_COUNT_LO_OFFSET, 0x1111_2222, 0);
            p.write32(AON_COUNT_HI_OFFSET, 0x3333_4444, 0);
            assert_eq!(p.read32(AON_COUNT_LO_OFFSET), 0x1111_2222);
            assert_eq!(p.read32(AON_COUNT_HI_OFFSET), 0x3333_4444);
        });
        let events = captured.lock().unwrap();
        assert_eq!(
            count_warns_containing(&events, "AON COUNT"),
            1,
            "expected exactly one AON COUNT warn; got {:?}", *events
        );
    }

    #[test]
    fn aon_match_roundtrip_and_warns_once() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber { events: captured.clone() };
        tracing::subscriber::with_default(subscriber, || {
            let mut p = PowmanRegs::new();
            p.write32(AON_MATCH_LO_OFFSET, 0xCAFE_BABE, 0);
            p.write32(AON_MATCH_HI_OFFSET, 0xDEAD_BEEF, 0);
            assert_eq!(p.read32(AON_MATCH_LO_OFFSET), 0xCAFE_BABE);
            assert_eq!(p.read32(AON_MATCH_HI_OFFSET), 0xDEAD_BEEF);
        });
        let events = captured.lock().unwrap();
        assert_eq!(
            count_warns_containing(&events, "AON MATCH"),
            1,
            "expected exactly one AON MATCH warn; got {:?}", *events
        );
    }

    #[test]
    fn archsel_arm_default_and_no_warn_on_arm_write() {
        let captured: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let subscriber = CaptureSubscriber { events: captured.clone() };
        tracing::subscriber::with_default(subscriber, || {
            let mut p = PowmanRegs::new();
            // Reset default reads Arm (0).
            assert_eq!(p.read32(ARCHSEL_OFFSET), ARCHSEL_ARM);
            // Writing Arm again must NOT warn.
            p.write32(ARCHSEL_OFFSET, ARCHSEL_ARM, 0);
        });
        let events = captured.lock().unwrap();
        assert_eq!(
            count_warns_containing(&events, "ARCHSEL"),
            0,
            "writing Arm default must not warn; got {:?}", *events
        );
    }

    /// Tripwire fires exactly once on the first non-Arm ARCHSEL write.
    /// Renamed from `archsel_non_arm_warns_once` — the event is now
    /// trace-level (silicon has no ARCHSEL register; see module doc),
    /// so "warn" no longer matches the level. The behaviour is still a
    /// one-shot tripwire latched by the `warned_archsel` struct flag.
    ///
    /// Tests only the struct flag (not the emitted `trace!` event) —
    /// `trace!` is compiled out in `--release` by the workspace's
    /// `release_max_level_info` setting, so a capture-based assertion
    /// would green in debug and red in release.
    #[test]
    fn powman_archsel_non_arm_write_fires_tripwire_once() {
        let mut p = PowmanRegs::new();
        assert!(!p.warned_archsel, "tripwire must start latched-low");
        p.write32(ARCHSEL_OFFSET, 1, 0);
        assert!(
            p.warned_archsel,
            "first non-Arm write must latch the tripwire"
        );
        p.write32(ARCHSEL_OFFSET, 2, 0);
        assert_eq!(p.read32(ARCHSEL_OFFSET), 2);
        assert!(
            p.warned_archsel,
            "tripwire stays latched on subsequent non-Arm writes"
        );
    }

    #[test]
    fn unknown_offset_roundtrip() {
        let mut p = PowmanRegs::new();
        p.write32(0xF00, 0x1234_5678, 0);
        assert_eq!(p.read32(0xF00), 0x1234_5678);
        assert_eq!(p.read32(0xF04), 0);
    }
}

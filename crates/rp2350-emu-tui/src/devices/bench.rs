//! Benchmark sentinel poller.
//!
//! The benchmark firmware writes a phase value to SRAM address
//! `0x2003_FF00` at the start and end of each timed section (LLD §8.1):
//!
//! | Phase | Meaning                                          |
//! |-------|--------------------------------------------------|
//! | 0x00  | Not yet started                                  |
//! | 0x1N  | Section N started (N = 1..=REFERENCE_TABLE.len())|
//! | 0x2N  | Section N done                                   |
//! | 0xFF  | All sections complete (firmware halts)           |
//!
//! The sim thread calls [`BenchmarkPoller::poll`] once per quantum; the
//! poller samples the sentinel via `emu.peek()` and, on transition,
//! either records a section start or finalises a section by subtracting
//! the recorded start from the current `emu.cycles()`. Populated entries
//! reference names and reference-cycle counts from [`REFERENCE_TABLE`]
//! by zero-based index (`phase & 0x0F) - 1`).
//!
//! The poll is O(1) per quantum — one memory read, one match, one
//! comparison for the stall guard.

use std::time::Duration;

use rp2350_emu::Emulator;

use crate::firmware::REFERENCE_TABLE;
use crate::snapshot::{BenchmarkReport, BenchmarkSection};

/// Sentinel address — see LLD §8.1.
pub const PHASE_ADDR: u32 = 0x2003_FF00;

/// Any phase the firmware holds longer than this before reaching 0xFF is
/// reported as a stall in the bench panel. Since we wrote the firmware
/// this is effectively a bug indicator, not a normal operating condition.
pub const STALL_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Default)]
pub struct BenchmarkPoller {
    last_phase: u32,
    section_start: u64,
    sections: Vec<BenchmarkSection>,
    complete: bool,
    stall: Option<u32>,
}

impl BenchmarkPoller {
    pub fn new() -> Self {
        Self::default()
    }

    /// Sample the sentinel once. Call once per sim-thread quantum.
    ///
    /// `wall` is the wall-clock elapsed since the sim thread started —
    /// used solely to trigger stall detection after [`STALL_TIMEOUT`].
    /// Reference numbers and iteration counts come from
    /// [`REFERENCE_TABLE`]; the poller is agnostic to the actual
    /// instruction under test.
    pub fn poll(&mut self, emu: &Emulator, wall: Duration) {
        let phase = emu.peek(PHASE_ADDR);

        if phase != self.last_phase {
            let now = emu.cycles();
            match phase & 0xF0 {
                0x10 => {
                    // Start-of-section marker: record the master cycle
                    // count right before the loop body begins.
                    self.section_start = now;
                }
                0x20 => {
                    // End-of-section marker: look up the reference row
                    // by section index and push a complete record. If
                    // the firmware drifts out of sync with the table
                    // (e.g. extra section, missing reference) the extra
                    // marker is silently dropped — the complete flag
                    // and stall guard still tell the right story.
                    let idx = ((phase & 0x0F) as usize).saturating_sub(1);
                    if let Some(reference) = REFERENCE_TABLE.get(idx) {
                        self.sections.push(BenchmarkSection {
                            name: reference.name,
                            emu_cycles: now.saturating_sub(self.section_start),
                            ref_cycles: reference.cycles,
                            iterations: reference.iterations,
                        });
                    }
                }
                0xF0 => {
                    // Final halt marker.
                    self.complete = true;
                }
                _ => {}
            }
            self.last_phase = phase;
        }

        // Stall guard: if we're past the timeout and haven't hit 0xFF,
        // latch the current phase so the panel can surface the error.
        if !self.complete && self.stall.is_none() && wall > STALL_TIMEOUT {
            self.stall = Some(phase);
        }
    }

    /// Returns a snapshot of the current benchmark state, or `None` if
    /// nothing has been observed yet (no sections finalised *and* no
    /// stall latched). Called by the sim thread once per quantum to
    /// populate the shared snapshot.
    pub fn report(&self) -> Option<BenchmarkReport> {
        if self.sections.is_empty() && self.stall.is_none() && !self.complete {
            return None;
        }
        Some(BenchmarkReport {
            sections: self.sections.clone(),
            complete: self.complete,
            stall: self.stall,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rp2350_emu::{Config, Emulator};

    /// Spin up a fresh emulator and poker — the tests exercise the
    /// poller against a real `emu.peek()`/`emu.cycles()` surface rather
    /// than a mock, per LLD §12 (no mocking the emulator).
    fn fresh() -> (Emulator, BenchmarkPoller) {
        let emu = Emulator::new(Config::default());
        let poller = BenchmarkPoller::new();
        (emu, poller)
    }

    #[test]
    fn initial_state_yields_no_report() {
        let (emu, mut poller) = fresh();
        poller.poll(&emu, Duration::from_secs(0));
        assert!(
            poller.report().is_none(),
            "before any sentinel transition the report must be None"
        );
    }

    #[test]
    fn section_start_then_done_records_one_section() {
        let (mut emu, mut poller) = fresh();

        // Section 1 start (phase = 0x11).
        emu.poke(PHASE_ADDR, 0x11);
        poller.poll(&emu, Duration::from_secs(0));
        assert!(
            poller.report().is_none(),
            "a start marker alone should not yield a finished section"
        );

        // Section 1 done (phase = 0x21).
        emu.poke(PHASE_ADDR, 0x21);
        poller.poll(&emu, Duration::from_secs(0));

        let report = poller.report().expect("a section should be recorded");
        assert_eq!(report.sections.len(), 1, "exactly one section recorded");
        assert!(
            !report.complete,
            "report must not be marked complete until 0xFF is observed"
        );
        assert!(
            report.stall.is_none(),
            "no stall expected under the timeout"
        );

        let row = &report.sections[0];
        assert_eq!(row.name, REFERENCE_TABLE[0].name);
        assert_eq!(row.iterations, REFERENCE_TABLE[0].iterations);
        assert_eq!(row.ref_cycles, REFERENCE_TABLE[0].cycles);
    }

    #[test]
    fn final_halt_phase_marks_report_complete() {
        let (mut emu, mut poller) = fresh();

        emu.poke(PHASE_ADDR, 0x11);
        poller.poll(&emu, Duration::from_secs(0));
        emu.poke(PHASE_ADDR, 0x21);
        poller.poll(&emu, Duration::from_secs(0));

        emu.poke(PHASE_ADDR, 0xFF);
        poller.poll(&emu, Duration::from_secs(0));

        let report = poller.report().expect("report should be present");
        assert!(
            report.complete,
            "observing phase=0xFF must mark the report complete"
        );
        assert_eq!(report.sections.len(), 1, "existing sections preserved");
    }

    #[test]
    fn stall_latches_when_wall_exceeds_timeout_without_halt() {
        let (mut emu, mut poller) = fresh();

        // Start section 1 and then never finish it; report stall.
        emu.poke(PHASE_ADDR, 0x14);
        poller.poll(&emu, Duration::from_secs(0));
        // Just below the timeout — no stall yet, and still no report
        // (no section complete, no stall latched).
        poller.poll(&emu, STALL_TIMEOUT);
        assert!(poller.report().is_none());

        // Past the timeout — stall should latch with the current phase.
        poller.poll(&emu, STALL_TIMEOUT + Duration::from_millis(1));
        let report = poller.report().expect("stall path should now be populated");
        assert_eq!(
            report.stall,
            Some(0x14),
            "the stall field should capture the phase we were stuck at"
        );
        assert!(!report.complete);
    }

    #[test]
    fn stall_does_not_latch_once_complete() {
        let (mut emu, mut poller) = fresh();

        emu.poke(PHASE_ADDR, 0xFF);
        poller.poll(&emu, Duration::from_secs(0));
        // Even way past the timeout, a completed run should not latch a stall.
        poller.poll(&emu, STALL_TIMEOUT + Duration::from_secs(30));

        let report = poller.report().expect("complete report is visible");
        assert!(report.complete);
        assert!(
            report.stall.is_none(),
            "completed runs must not be marked stalled"
        );
    }

    #[test]
    fn out_of_range_section_index_does_not_panic() {
        // Firmware drift: phase encodes a section index that isn't in
        // the reference table. The poller must drop the update silently
        // rather than panic or push garbage.
        let (mut emu, mut poller) = fresh();
        let too_big = (REFERENCE_TABLE.len() as u32) + 5;
        assert!(too_big <= 0x0F, "table fits in a nibble for this test");
        emu.poke(PHASE_ADDR, 0x10 | too_big);
        poller.poll(&emu, Duration::from_secs(0));
        emu.poke(PHASE_ADDR, 0x20 | too_big);
        poller.poll(&emu, Duration::from_secs(0));
        // No report (no sections, no stall) — the drift is silently eaten.
        assert!(poller.report().is_none());
    }
}

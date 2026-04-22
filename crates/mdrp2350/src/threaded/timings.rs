//! Per-worker per-quantum timing instrumentation for the 6-thread runtime.
//!
//! When enabled on a [`super::ThreadedEmulator`] via
//! [`super::ThreadedEmulator::set_timing_enabled`], each worker records
//! two nanosecond timestamps per quantum:
//!
//!   - `phase_work_ns`  — wall time doing the worker's actual phase
//!                        work (CPU step / PIO step / coord tick), i.e.
//!                        from the previous `barrier.wait()` return to
//!                        the next `barrier.wait()` entry.
//!   - `barrier_wait_ns` — wall time blocked in `barrier.wait()`, i.e.
//!                         from wait entry to wait return.
//!
//! Interpretation: the highest `phase_work_ns` each quantum is the
//! bottleneck worker; a worker with high `barrier_wait_ns` finished
//! early and was waiting for a peer. If all six workers show high
//! `barrier_wait_ns` at once, the barrier spin-wait itself is where
//! the cycles are going.
//!
//! When the flag is off the workers skip the `Instant::now()` brackets
//! entirely, so the production hot path pays nothing. A disabled
//! `PerWorkerTimings` is a zero-capacity wrapper (no allocation).
//!
//! `RunTimings::last_run_timings` on `ThreadedEmulator` reflects the
//! most recent `run_quanta` call only; each call resets.

use std::time::Instant;

/// Which of the six worker threads the timings belong to. Matches the
/// worker-index convention used elsewhere in `emulator.rs` (core0 is
/// worker 0, core1 is worker 1, pio0/pio1/pio2 are workers 2..=4, coord
/// is worker 5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkerName {
    Core0,
    Core1,
    Pio0,
    Pio1,
    Pio2,
    Coord,
}

impl WorkerName {
    /// Short label for summary tables. Kept stable so tooling can scrape
    /// `--timing` output lines.
    pub fn as_str(self) -> &'static str {
        match self {
            WorkerName::Core0 => "core0",
            WorkerName::Core1 => "core1",
            WorkerName::Pio0 => "pio0",
            WorkerName::Pio1 => "pio1",
            WorkerName::Pio2 => "pio2",
            WorkerName::Coord => "coord",
        }
    }
}

/// Raw per-quantum timings for a single worker. Two parallel `Vec`s:
/// `phase_work_ns[i]` and `barrier_wait_ns[i]` are from the same
/// quantum `i`.
///
/// Kept private to this module. Aggregated into a [`WorkerSummary`]
/// before it leaves `ThreadedEmulator`.
#[derive(Debug, Default)]
pub(super) struct PerWorkerTimings {
    pub(super) phase_work_ns: Vec<u64>,
    pub(super) barrier_wait_ns: Vec<u64>,
}

impl PerWorkerTimings {
    /// Pre-allocated storage for `n` quanta when timing is enabled, or a
    /// zero-capacity placeholder when it's not. On the disabled path
    /// the worker skips every `push` and `Instant::now()` call, so the
    /// zero-cap vec never grows.
    pub(super) fn new(n: u64, enabled: bool) -> Self {
        if enabled {
            // Bound the up-front allocation. Very long runs (billions of
            // quanta) would otherwise request gigabytes. 10 M quanta
            // caps us at 160 MB per worker, which is plenty for any
            // bench run we'd actually stare at.
            let cap = n.min(10_000_000) as usize;
            Self {
                phase_work_ns: Vec::with_capacity(cap),
                barrier_wait_ns: Vec::with_capacity(cap),
            }
        } else {
            Self::default()
        }
    }
}

/// Summary statistics over one worker's quanta. Emitted by
/// [`RunTimings::summary`]; the raw vecs stay private.
///
/// `total_ns` is the plain sum over every quantum recorded. It's what
/// you'd compare against wall time for a utilisation figure.
#[derive(Clone, Copy, Debug, Default)]
pub struct WorkerSummary {
    pub name_idx: usize,
    pub samples: usize,
    pub phase_work_mean_ns: u64,
    pub phase_work_p50_ns: u64,
    pub phase_work_p99_ns: u64,
    pub phase_work_max_ns: u64,
    pub phase_work_total_ns: u64,
    pub barrier_wait_mean_ns: u64,
    pub barrier_wait_p50_ns: u64,
    pub barrier_wait_p99_ns: u64,
    pub barrier_wait_max_ns: u64,
    pub barrier_wait_total_ns: u64,
}

impl WorkerSummary {
    /// Resolve the worker this summary belongs to from its ordinal.
    pub fn name(&self) -> WorkerName {
        debug_assert!(self.name_idx < 6, "WorkerSummary.name_idx out of range");
        match self.name_idx {
            0 => WorkerName::Core0,
            1 => WorkerName::Core1,
            2 => WorkerName::Pio0,
            3 => WorkerName::Pio1,
            4 => WorkerName::Pio2,
            _ => WorkerName::Coord,
        }
    }
}

/// All six workers' raw timings from the most recent `run_quanta`.
/// Constructed on [`super::ThreadedEmulator::run_quanta`] completion;
/// read via [`super::ThreadedEmulator::last_run_timings`].
///
/// The raw vecs are private. Consumers call [`RunTimings::summary`] to
/// get the six per-worker [`WorkerSummary`] entries in fixed order
/// `[core0, core1, pio0, pio1, pio2, coord]`.
#[derive(Debug)]
pub struct RunTimings {
    pub(super) core0: PerWorkerTimings,
    pub(super) core1: PerWorkerTimings,
    pub(super) pio0: PerWorkerTimings,
    pub(super) pio1: PerWorkerTimings,
    pub(super) pio2: PerWorkerTimings,
    pub(super) coord: PerWorkerTimings,
}

impl RunTimings {
    /// Compute mean / p50 / p99 / max / total for each worker. Returned
    /// in fixed `[core0, core1, pio0, pio1, pio2, coord]` order so
    /// callers can index by [`WorkerName`] as-is.
    pub fn summary(&self) -> [WorkerSummary; 6] {
        [
            summarise(0, &self.core0),
            summarise(1, &self.core1),
            summarise(2, &self.pio0),
            summarise(3, &self.pio1),
            summarise(4, &self.pio2),
            summarise(5, &self.coord),
        ]
    }

    /// Number of quanta recorded. Uses the core0 worker's count; every
    /// worker records one sample per quantum so all six are equal.
    /// Zero when timing was disabled for the run.
    pub fn samples(&self) -> usize {
        self.core0.phase_work_ns.len()
    }
}

fn summarise(name_idx: usize, raw: &PerWorkerTimings) -> WorkerSummary {
    let n = raw.phase_work_ns.len();
    if n == 0 {
        return WorkerSummary {
            name_idx,
            ..WorkerSummary::default()
        };
    }

    let (pw_mean, pw_p50, pw_p99, pw_max, pw_total) = stats(&raw.phase_work_ns);
    let (bw_mean, bw_p50, bw_p99, bw_max, bw_total) = stats(&raw.barrier_wait_ns);

    WorkerSummary {
        name_idx,
        samples: n,
        phase_work_mean_ns: pw_mean,
        phase_work_p50_ns: pw_p50,
        phase_work_p99_ns: pw_p99,
        phase_work_max_ns: pw_max,
        phase_work_total_ns: pw_total,
        barrier_wait_mean_ns: bw_mean,
        barrier_wait_p50_ns: bw_p50,
        barrier_wait_p99_ns: bw_p99,
        barrier_wait_max_ns: bw_max,
        barrier_wait_total_ns: bw_total,
    }
}

/// Compute (mean, p50, p99, max, total) over a non-empty sample vec.
/// Sorts a clone so the caller's vec stays in quantum-order for other
/// consumers. p50 / p99 use nearest-rank (no interpolation) — good
/// enough for bench diagnostics, and monotone under concatenation.
fn stats(samples: &[u64]) -> (u64, u64, u64, u64, u64) {
    debug_assert!(!samples.is_empty());
    let total: u64 = samples.iter().sum();
    let mean = total / samples.len() as u64;

    let mut sorted = samples.to_vec();
    sorted.sort_unstable();

    let p50_idx = (sorted.len().saturating_sub(1)) / 2;
    // Nearest-rank p99: ceil(0.99 * n) - 1, clamped to last index.
    let p99_idx = {
        let raw = (sorted.len() as u64 * 99).div_ceil(100);
        (raw.saturating_sub(1) as usize).min(sorted.len() - 1)
    };
    (mean, sorted[p50_idx], sorted[p99_idx], sorted[sorted.len() - 1], total)
}

/// Runtime helper a worker uses to bracket its wait / phase-work spans.
/// One instance is constructed per worker body; `enabled == false`
/// collapses every bracket call into a branch-only no-op so the hot
/// path pays nothing when the bench harness didn't ask for timings.
pub(super) struct TimingRecorder {
    enabled: bool,
    /// Last timestamp captured. Flips meaning between "wait started"
    /// and "wait ended" — each call to `wait_returned` / `phase_ended`
    /// closes one span and (implicitly) opens the next.
    last: Option<Instant>,
    pub(super) timings: PerWorkerTimings,
}

impl TimingRecorder {
    pub(super) fn new(n: u64, enabled: bool) -> Self {
        Self {
            enabled,
            last: None,
            timings: PerWorkerTimings::new(n, enabled),
        }
    }

    /// Call at the top of the worker, before the main loop. Anchors
    /// the "quantum 0 phase_work" span at worker entry so the first
    /// quantum's phase_work_ns includes any thread-spawn residue. A
    /// follow-up comment at the call site flags this caveat.
    #[inline]
    pub(super) fn on_worker_entry(&mut self) {
        if !self.enabled {
            return;
        }
        self.last = Some(Instant::now());
    }

    /// Call immediately before `barrier.wait()` — closes the
    /// phase-work span that started at the last `on_worker_entry` /
    /// wait return.
    #[inline]
    pub(super) fn on_wait_entry(&mut self) {
        if !self.enabled {
            return;
        }
        let now = Instant::now();
        let prev = self.last.expect("on_wait_entry without prior anchor");
        let ns = now.saturating_duration_since(prev).as_nanos() as u64;
        self.timings.phase_work_ns.push(ns);
        self.last = Some(now);
    }

    /// Call immediately after `barrier.wait()` returns — closes the
    /// barrier-wait span and anchors the next phase-work span.
    #[inline]
    pub(super) fn on_wait_return(&mut self) {
        if !self.enabled {
            return;
        }
        let now = Instant::now();
        let prev = self.last.expect("on_wait_return without prior anchor");
        let ns = now.saturating_duration_since(prev).as_nanos() as u64;
        self.timings.barrier_wait_ns.push(ns);
        self.last = Some(now);
    }

    pub(super) fn take(self) -> PerWorkerTimings {
        self.timings
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disabled_recorder_allocates_nothing() {
        let r = TimingRecorder::new(1_000_000, false);
        assert_eq!(r.timings.phase_work_ns.capacity(), 0);
        assert_eq!(r.timings.barrier_wait_ns.capacity(), 0);
    }

    #[test]
    fn enabled_recorder_records_pairs() {
        let mut r = TimingRecorder::new(3, true);
        r.on_worker_entry();
        for _ in 0..3 {
            r.on_wait_entry();
            r.on_wait_return();
        }
        assert_eq!(r.timings.phase_work_ns.len(), 3);
        assert_eq!(r.timings.barrier_wait_ns.len(), 3);
    }

    #[test]
    fn disabled_recorder_records_nothing() {
        let mut r = TimingRecorder::new(3, false);
        r.on_worker_entry();
        for _ in 0..3 {
            r.on_wait_entry();
            r.on_wait_return();
        }
        assert!(r.timings.phase_work_ns.is_empty());
        assert!(r.timings.barrier_wait_ns.is_empty());
    }

    #[test]
    fn summary_handles_empty() {
        let s = summarise(0, &PerWorkerTimings::default());
        assert_eq!(s.samples, 0);
        assert_eq!(s.phase_work_mean_ns, 0);
        assert_eq!(s.phase_work_max_ns, 0);
        assert_eq!(s.barrier_wait_total_ns, 0);
    }

    #[test]
    fn summary_computes_mean_p50_p99_max() {
        // 100 samples: 1..=100. mean=50, p50=50, p99=99, max=100.
        let phase: Vec<u64> = (1..=100).collect();
        let raw = PerWorkerTimings {
            phase_work_ns: phase.clone(),
            barrier_wait_ns: phase,
        };
        let s = summarise(1, &raw);
        assert_eq!(s.samples, 100);
        assert_eq!(s.phase_work_mean_ns, 50);
        // p50: nearest-rank on index 49 → sorted[49] = 50.
        assert_eq!(s.phase_work_p50_ns, 50);
        // p99 nearest-rank: ceil(100*99/100)=99 → index 98 → sorted[98]=99.
        assert_eq!(s.phase_work_p99_ns, 99);
        assert_eq!(s.phase_work_max_ns, 100);
        assert_eq!(s.phase_work_total_ns, (1..=100).sum::<u64>());
    }

    #[test]
    fn run_timings_samples_reports_core0_count() {
        let rt = RunTimings {
            core0: PerWorkerTimings {
                phase_work_ns: vec![1, 2, 3],
                barrier_wait_ns: vec![4, 5, 6],
            },
            core1: PerWorkerTimings::default(),
            pio0: PerWorkerTimings::default(),
            pio1: PerWorkerTimings::default(),
            pio2: PerWorkerTimings::default(),
            coord: PerWorkerTimings::default(),
        };
        assert_eq!(rt.samples(), 3);
    }

    #[test]
    fn worker_name_labels_are_stable() {
        assert_eq!(WorkerName::Core0.as_str(), "core0");
        assert_eq!(WorkerName::Core1.as_str(), "core1");
        assert_eq!(WorkerName::Pio0.as_str(), "pio0");
        assert_eq!(WorkerName::Pio1.as_str(), "pio1");
        assert_eq!(WorkerName::Pio2.as_str(), "pio2");
        assert_eq!(WorkerName::Coord.as_str(), "coord");
    }
}

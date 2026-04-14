use std::sync::atomic::{AtomicU64, AtomicBool, Ordering};
use std::sync::Arc;

/// Shared monitoring state. Atomic counters updated on the hot path,
/// safe to read from any thread without locking.
pub struct PacerStats {
    /// Total emulated cycles since pacer started.
    emulated_cycles: AtomicU64,
    /// Host nanoseconds spent emulating (doing useful work).
    emulation_ns: AtomicU64,
    /// Host nanoseconds spent spinning (waiting for real-time to catch up).
    spin_ns: AtomicU64,
    /// Number of quanta where emulation couldn't keep up with real-time.
    behind_count: AtomicU64,
    /// Cumulative wall-clock nanoseconds since first begin_quantum().
    /// This is set (not added) to absolute elapsed time, unlike the
    /// other counters.
    wall_ns: AtomicU64,
    /// Whether pacing is currently active. Caller-managed — the Pacer does
    /// not set this automatically. Monitoring consumers can check this to
    /// know if data is flowing.
    running: AtomicBool,
}

impl PacerStats {
    pub fn new() -> Self {
        Self {
            emulated_cycles: AtomicU64::new(0),
            emulation_ns: AtomicU64::new(0),
            spin_ns: AtomicU64::new(0),
            behind_count: AtomicU64::new(0),
            wall_ns: AtomicU64::new(0),
            running: AtomicBool::new(false),
        }
    }

    /// Read all atomic counters and return a point-in-time snapshot.
    pub fn snapshot(&self) -> PacerSnapshot {
        PacerSnapshot {
            emulated_cycles: self.emulated_cycles.load(Ordering::Relaxed),
            emulation_ns: self.emulation_ns.load(Ordering::Relaxed),
            spin_ns: self.spin_ns.load(Ordering::Relaxed),
            behind_count: self.behind_count.load(Ordering::Relaxed),
            wall_ns: self.wall_ns.load(Ordering::Relaxed),
        }
    }

    pub(crate) fn add_emulated_cycles(&self, n: u64) {
        self.emulated_cycles.fetch_add(n, Ordering::Relaxed);
    }

    pub(crate) fn add_emulation_ns(&self, n: u64) {
        self.emulation_ns.fetch_add(n, Ordering::Relaxed);
    }

    pub(crate) fn add_spin_ns(&self, n: u64) {
        self.spin_ns.fetch_add(n, Ordering::Relaxed);
    }

    pub(crate) fn increment_behind(&self) {
        self.behind_count.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn set_wall_ns(&self, ns: u64) {
        self.wall_ns.store(ns, Ordering::Relaxed);
    }

    pub fn set_running(&self, val: bool) {
        self.running.store(val, Ordering::Relaxed);
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }
}

impl Default for PacerStats {
    fn default() -> Self {
        Self::new()
    }
}

/// Point-in-time snapshot of pacer stats. All values are plain integers
/// copied from the atomic counters. Derived metrics are computed here
/// to keep the hot path (atomic updates) minimal.
///
/// Snapshots are approximate: individual fields are read with Relaxed ordering,
/// so a snapshot may see cycles from quantum N but spin_ns from quantum N-1.
/// For monitoring/dashboard purposes this is fine — all values are monotonically
/// increasing and converge quickly.
///
/// Preemption caveat: `wall_ns` is derived from TSC at quantum boundaries, so any
/// OS preemption that lands *during* an in-budget quantum's emulation phase is
/// folded into `emulation_ns` (we only timestamp at begin/end). `utilization()`
/// is therefore an upper bound on actual emulation-work fraction — real
/// utilization is lower by the preemption fraction.
#[derive(Debug)]
pub struct PacerSnapshot {
    pub emulated_cycles: u64,
    pub emulation_ns: u64,
    pub spin_ns: u64,
    pub behind_count: u64,
    pub wall_ns: u64,
}

impl PacerSnapshot {
    /// Upper bound on fraction of wall time spent emulating. Note: any OS
    /// preemption that lands during an in-budget quantum's emulation phase
    /// is counted as emulation time (we only timestamp at quantum boundaries).
    /// Real utilization is lower by the preemption fraction.
    pub fn utilization(&self) -> f64 {
        if self.wall_ns == 0 {
            return 0.0;
        }
        self.emulation_ns as f64 / self.wall_ns as f64
    }

    /// Fraction of wall time not spent emulating (1.0 - utilization).
    pub fn headroom(&self) -> f64 {
        1.0 - self.utilization()
    }

    /// Effective emulated clock rate in MHz.
    /// Formula: cycles/ns = GHz; multiply by 10^3 to convert to MHz.
    pub fn emulated_mhz(&self) -> f64 {
        if self.wall_ns == 0 {
            return 0.0;
        }
        self.emulated_cycles as f64 / self.wall_ns as f64 * 1000.0
    }
}

// ---------------------------------------------------------------------------
// Pacer — real-time pacing via rdtsc spin-wait
// ---------------------------------------------------------------------------

/// Non-serializing timestamp read. Lower overhead, used only inside
/// spin-wait polling loops where convergence handles any reordering.
#[cfg(target_arch = "x86_64")]
#[inline(always)]
fn rdtsc() -> u64 {
    unsafe { std::arch::x86_64::_rdtsc() }
}

/// Serializing timestamp read. Waits for prior instructions to retire
/// before reading TSC. Used for measurement points where accuracy matters.
#[cfg(target_arch = "x86_64")]
#[inline(always)]
fn rdtscp() -> u64 {
    let mut _aux = 0u32;
    unsafe { std::arch::x86_64::__rdtscp(&mut _aux) }
}

/// Verify the CPU supports invariant TSC (constant_tsc). Panics with a
/// clear message if not. All modern x86_64 CPUs (since ~2008) support this.
#[cfg(target_arch = "x86_64")]
fn require_constant_tsc() {
    let result = std::arch::x86_64::__cpuid(0x80000007);
    let has_invariant_tsc = (result.edx >> 8) & 1 != 0;
    assert!(
        has_invariant_tsc,
        "CPU does not support invariant TSC (constant_tsc). \
         Required for rdtsc-based real-time pacing."
    );
}

/// Calibrate the TSC frequency by measuring rdtsc ticks over a short sleep.
/// Assumes invariant TSC (verified by `require_constant_tsc`).
#[cfg(target_arch = "x86_64")]
fn calibrate_tsc() -> u64 {
    require_constant_tsc();
    let t0 = std::time::Instant::now();
    let tsc0 = rdtscp();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let tsc1 = rdtscp();
    let elapsed_ns = t0.elapsed().as_nanos() as u64;
    (tsc1 - tsc0) * 1_000_000_000 / elapsed_ns
}

/// Measure per-quantum overhead empirically. Runs 5 batches of 2000 no-op
/// quanta each and returns the minimum measured overhead (least disturbed
/// by OS preemption). Clamps to nominal/4 maximum to survive pathological
/// calibration runs.
#[cfg(target_arch = "x86_64")]
fn calibrate_overhead(nominal_quantum_tsc: u64, tsc_freq_hz: u64) -> u64 {
    const BATCHES: usize = 5;
    const PER_BATCH: u64 = 2000;

    let tsc_to_ns = |ticks: u64| -> u64 {
        (ticks as u128 * 1_000_000_000 / tsc_freq_hz as u128) as u64
    };

    // Throwaway stats to exercise the same atomic updates production does.
    let stats = PacerStats::new();

    let mut min_overhead = u64::MAX;
    for _ in 0..BATCHES {
        let first = rdtscp();
        let mut quantum_start = first;
        for _ in 0..PER_BATCH {
            // Mimic end_quantum: measurement + spin + stats updates.
            let emu_end = rdtscp();
            let emulation_tsc = emu_end - quantum_start;
            let final_tsc = if emulation_tsc < nominal_quantum_tsc {
                let target = quantum_start + nominal_quantum_tsc;
                let mut now = emu_end;
                while now < target {
                    std::hint::spin_loop();
                    now = rdtsc();
                }
                let total_tsc = now - quantum_start;
                let spin_tsc = total_tsc - emulation_tsc;
                stats.add_emulation_ns(tsc_to_ns(emulation_tsc));
                stats.add_spin_ns(tsc_to_ns(spin_tsc));
                now
            } else {
                stats.add_emulation_ns(tsc_to_ns(emulation_tsc));
                stats.increment_behind();
                emu_end
            };
            stats.add_emulated_cycles(150);
            let wall_tsc = final_tsc - first;
            stats.set_wall_ns(tsc_to_ns(wall_tsc));

            // Mimic next begin_quantum.
            quantum_start = rdtscp();
        }
        let elapsed = quantum_start - first;
        let per_quantum = elapsed / PER_BATCH;
        let overhead = per_quantum.saturating_sub(nominal_quantum_tsc);
        if overhead < min_overhead {
            min_overhead = overhead;
        }
    }

    // Safety clamp: if calibration is catastrophically wrong, cap the
    // correction at 25% of nominal rather than panicking on zero ticks.
    let max_overhead = nominal_quantum_tsc / 4;
    min_overhead.min(max_overhead)
}

/// Real-time pacer that spin-waits to keep emulation at the target clock rate.
///
/// Usage:
/// ```ignore
/// let mut pacer = Pacer::new(150_000_000);
/// loop {
///     pacer.begin_quantum();
///     emulator.run(pacer.quantum_cycles());
///     pacer.end_quantum();
/// }
/// ```
#[cfg(target_arch = "x86_64")]
pub struct Pacer {
    /// Shared monitoring stats.
    stats: Arc<PacerStats>,
    /// Emulated cycles per quantum. Default 150 (= 1 us at 150 MHz).
    quantum_cycles: u64,
    /// Host rdtsc ticks per quantum (derived from calibration).
    quantum_tsc_ticks: u64,
    /// rdtsc value at start of current quantum.
    quantum_start_tsc: u64,
    /// TSC at first begin_quantum call. Set once on the first call.
    first_begin_tsc: Option<u64>,
    /// Calibrated TSC frequency in Hz.
    tsc_freq_hz: u64,
    /// Emulator system clock in Hz (e.g. 150_000_000). Mutable via
    /// [`Pacer::update_sys_clk_hz`] so pacing follows firmware clock
    /// reconfiguration — see LLD V2 §4.7.
    sys_clk_hz: u64,
    /// Per-quantum measurement/spin overhead in TSC ticks. Subtracted
    /// from the nominal quantum length so spin targets compensate for
    /// the fixed cost of `end_quantum`'s bookkeeping. Computed once in
    /// [`Pacer::with_quantum`] via [`calibrate_overhead`] and stored so
    /// [`Pacer::update_sys_clk_hz`] can reuse it when the system clock
    /// changes without re-running the 50 ms calibration sweep.
    overhead: u64,
}

#[cfg(target_arch = "x86_64")]
impl Pacer {
    /// Create a new pacer for the given emulator clock frequency.
    /// Calibrates TSC and measures per-quantum overhead (~60 ms one-time cost).
    pub fn new(sys_clk_hz: u32) -> Self {
        Self::with_quantum(sys_clk_hz, 150)
    }

    /// Create a pacer with a custom quantum size.
    pub fn with_quantum(sys_clk_hz: u32, quantum_cycles: u64) -> Self {
        assert!(quantum_cycles > 0, "quantum_cycles must be non-zero");
        let tsc_freq_hz = calibrate_tsc();
        let nominal = (tsc_freq_hz as u128 * quantum_cycles as u128 / sys_clk_hz as u128) as u64;
        assert!(
            nominal >= 100,
            "quantum too small for TSC resolution (nominal = {} ticks)",
            nominal
        );
        let overhead = calibrate_overhead(nominal, tsc_freq_hz);
        let quantum_tsc_ticks = nominal.saturating_sub(overhead);
        assert!(quantum_tsc_ticks > 0, "quantum_tsc_ticks is zero");

        Self {
            stats: Arc::new(PacerStats::new()),
            quantum_cycles,
            quantum_tsc_ticks,
            quantum_start_tsc: 0,
            first_begin_tsc: None,
            tsc_freq_hz,
            sys_clk_hz: sys_clk_hz as u64,
            overhead,
        }
    }

    /// Get a shared handle to the monitoring stats.
    pub fn stats(&self) -> Arc<PacerStats> {
        Arc::clone(&self.stats)
    }

    /// Number of emulator cycles per quantum.
    pub fn quantum_cycles(&self) -> u64 {
        self.quantum_cycles
    }

    /// Calibrated TSC frequency.
    pub fn tsc_freq_hz(&self) -> u64 {
        self.tsc_freq_hz
    }

    /// Host rdtsc ticks per quantum after overhead compensation. Exposed
    /// `pub(crate)` for unit tests that exercise [`Self::update_sys_clk_hz`];
    /// not part of the public API.
    #[cfg(test)]
    pub(crate) fn quantum_tsc_ticks(&self) -> u64 {
        self.quantum_tsc_ticks
    }

    /// Update the effective emulator system clock and recompute
    /// `quantum_tsc_ticks` so spin-wait targets track the new frequency.
    ///
    /// Called by the sim thread after each quantum — see LLD V2 §4.7.
    /// Zero-cost when the frequency is unchanged (the fast path is a
    /// single compare + early return).
    ///
    /// `new_hz == 0` is a guard path: if firmware misconfigures CLK_SYS
    /// to point at an unconfigured PLL (which now honestly reports 0 Hz),
    /// keep the previous quantum rather than divide-by-zero and crash.
    /// The emulator keeps running at the last known pace until firmware
    /// reconfigures the clock.
    #[inline]
    pub fn update_sys_clk_hz(&mut self, new_hz: u32) {
        if new_hz == 0 {
            return;
        }
        let new = new_hz as u64;
        if new == self.sys_clk_hz {
            return;
        }
        self.sys_clk_hz = new;
        let nominal = (self.tsc_freq_hz as u128
            * self.quantum_cycles as u128
            / new as u128) as u64;
        self.quantum_tsc_ticks = nominal.saturating_sub(self.overhead);
    }

    /// Mark the start of a quantum. Call before stepping the emulator.
    #[inline(always)]
    pub fn begin_quantum(&mut self) {
        let tsc = rdtscp();
        self.first_begin_tsc.get_or_insert(tsc);
        self.quantum_start_tsc = tsc;
    }

    /// End a quantum. Spin-waits if we're ahead of real-time, updates stats.
    /// Call after stepping the emulator for `quantum_cycles()` cycles.
    #[inline(always)]
    pub fn end_quantum(&mut self) {
        debug_assert!(self.quantum_start_tsc != 0, "begin_quantum() must be called before end_quantum()");

        // TSC wraparound: at 5 GHz, u64 wraps after ~117 years. If it does wrap
        // (or a VM offsets the TSC), unsigned subtraction produces a large value
        // and we take the "behind" path — safe degradation.
        let emu_end = rdtscp();
        let emulation_tsc = emu_end - self.quantum_start_tsc;

        let final_tsc = if emulation_tsc < self.quantum_tsc_ticks {
            // Ahead of real-time — spin wait.
            // Capture exit TSC inside the loop to avoid post-loop measurement skew.
            let target_tsc = self.quantum_start_tsc + self.quantum_tsc_ticks;
            let mut now = emu_end;
            while now < target_tsc {
                std::hint::spin_loop();
                now = rdtsc();
            }
            let total_tsc = now - self.quantum_start_tsc;
            let spin_tsc = total_tsc - emulation_tsc;

            self.stats.add_emulation_ns(self.tsc_to_ns(emulation_tsc));
            self.stats.add_spin_ns(self.tsc_to_ns(spin_tsc));
            now
        } else {
            // Behind real-time — skip spin, record it
            self.stats.add_emulation_ns(self.tsc_to_ns(emulation_tsc));
            self.stats.increment_behind();
            emu_end
        };

        // Update cumulative wall time since first begin_quantum.
        // Set wall_ns BEFORE cycles so snapshots see wall slightly ahead of
        // cycles (biases MHz low, bounded) rather than the reverse.
        let first = self.first_begin_tsc.expect("begin_quantum() must be called before end_quantum()");
        let wall_tsc = final_tsc - first;
        self.stats.set_wall_ns(self.tsc_to_ns(wall_tsc));
        self.stats.add_emulated_cycles(self.quantum_cycles);
    }

    /// Convert TSC ticks to nanoseconds.
    #[inline(always)]
    fn tsc_to_ns(&self, tsc_ticks: u64) -> u64 {
        (tsc_ticks as u128 * 1_000_000_000 / self.tsc_freq_hz as u128) as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pacer_stats_new() {
        let stats = PacerStats::new();
        let snap = stats.snapshot();
        assert_eq!(snap.emulated_cycles, 0);
        assert_eq!(snap.emulation_ns, 0);
        assert_eq!(snap.spin_ns, 0);
        assert_eq!(snap.behind_count, 0);
        assert_eq!(snap.wall_ns, 0);
        assert!(!stats.is_running());
    }

    #[test]
    fn test_pacer_stats_add_cycles() {
        let stats = PacerStats::new();
        stats.add_emulated_cycles(100);
        stats.add_emulated_cycles(50);
        assert_eq!(stats.snapshot().emulated_cycles, 150);
    }

    #[test]
    fn test_pacer_stats_snapshot() {
        let stats = PacerStats::new();
        stats.add_emulated_cycles(1000);
        stats.add_emulation_ns(500);
        stats.add_spin_ns(300);
        stats.increment_behind();
        stats.increment_behind();

        let snap = stats.snapshot();
        assert_eq!(snap.emulated_cycles, 1000);
        assert_eq!(snap.emulation_ns, 500);
        assert_eq!(snap.spin_ns, 300);
        assert_eq!(snap.behind_count, 2);
        // set_wall_ns hasn't been called, so wall_ns remains 0.
        assert_eq!(snap.wall_ns, 0);
    }

    #[test]
    fn test_pacer_stats_running() {
        let stats = PacerStats::new();
        assert!(!stats.is_running());
        stats.set_running(true);
        assert!(stats.is_running());
        stats.set_running(false);
        assert!(!stats.is_running());
    }

    #[test]
    fn test_snapshot_utilization_zero() {
        let snap = PacerSnapshot {
            emulated_cycles: 0,
            emulation_ns: 0,
            spin_ns: 0,
            behind_count: 0,
            wall_ns: 0,
        };
        assert_eq!(snap.utilization(), 0.0);
    }

    #[test]
    fn test_snapshot_utilization_half() {
        let snap = PacerSnapshot {
            emulated_cycles: 0,
            emulation_ns: 500,
            spin_ns: 500,
            behind_count: 0,
            wall_ns: 1000,
        };
        assert!((snap.utilization() - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_snapshot_utilization_full() {
        let snap = PacerSnapshot {
            emulated_cycles: 0,
            emulation_ns: 1000,
            spin_ns: 0,
            behind_count: 0,
            wall_ns: 1000,
        };
        assert!((snap.utilization() - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_snapshot_headroom() {
        let snap = PacerSnapshot {
            emulated_cycles: 0,
            emulation_ns: 300,
            spin_ns: 700,
            behind_count: 0,
            wall_ns: 1000,
        };
        assert!((snap.headroom() - 0.7).abs() < f64::EPSILON);
    }

    #[test]
    fn test_snapshot_emulated_mhz() {
        let snap = PacerSnapshot {
            emulated_cycles: 150_000,
            emulation_ns: 500_000,
            spin_ns: 500_000,
            behind_count: 0,
            wall_ns: 1_000_000,
        };
        assert!((snap.emulated_mhz() - 150.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_snapshot_emulated_mhz_zero() {
        let snap = PacerSnapshot {
            emulated_cycles: 100,
            emulation_ns: 0,
            spin_ns: 0,
            behind_count: 0,
            wall_ns: 0,
        };
        assert_eq!(snap.emulated_mhz(), 0.0);
    }

    // -----------------------------------------------------------------------
    // Pacer tests (x86_64 only — uses rdtsc)
    // -----------------------------------------------------------------------

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_pacer_creation() {
        let pacer = Pacer::new(150_000_000);
        assert_eq!(pacer.quantum_cycles(), 150);
        assert!(pacer.tsc_freq_hz() > 0, "TSC frequency must be non-zero");
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_pacer_with_quantum() {
        let pacer = Pacer::with_quantum(150_000_000, 300);
        assert_eq!(pacer.quantum_cycles(), 300);
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_pacer_stats_shared() {
        let pacer = Pacer::new(150_000_000);
        let stats1 = pacer.stats();
        let stats2 = pacer.stats();
        // Both Arcs point to the same allocation.
        assert!(Arc::ptr_eq(&stats1, &stats2));
        // Mutation through one is visible through the other.
        stats1.add_emulated_cycles(42);
        assert_eq!(stats2.snapshot().emulated_cycles, 42);
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_pacer_begin_end_quantum() {
        let mut pacer = Pacer::new(150_000_000);
        pacer.begin_quantum();
        // Do a tiny bit of work so emulation_ns is non-zero.
        let mut dummy = 0u64;
        for i in 0..1000 {
            dummy = dummy.wrapping_add(i);
        }
        std::hint::black_box(dummy);
        pacer.end_quantum();

        let snap = pacer.stats().snapshot();
        assert!(snap.emulation_ns > 0, "emulation_ns should be non-zero");
        assert_eq!(snap.emulated_cycles, 150);
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_pacer_behind_detection() {
        // Using u32::MAX as sys_clk_hz makes quantum_tsc_ticks extremely small,
        // so any real work between begin/end will be "behind".
        let mut pacer = Pacer::new(u32::MAX);
        pacer.begin_quantum();
        // Burn some time.
        let mut dummy = 0u64;
        for i in 0..10_000 {
            dummy = dummy.wrapping_add(i);
        }
        std::hint::black_box(dummy);
        pacer.end_quantum();

        let snap = pacer.stats().snapshot();
        assert!(snap.behind_count > 0, "should detect being behind real-time");
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_tsc_to_ns_via_pacer() {
        // Indirectly test tsc_to_ns: run a quantum and verify the stats
        // report sensible nanosecond values (> 0, < 1 second).
        let mut pacer = Pacer::new(150_000_000);
        pacer.begin_quantum();
        pacer.end_quantum();

        let snap = pacer.stats().snapshot();
        let total = snap.emulation_ns + snap.spin_ns;
        assert!(total > 0, "total ns should be non-zero after a quantum");
        assert!(total < 1_000_000_000, "a single quantum should take < 1 second");
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_pacer_wall_ns_after_quantum() {
        let mut pacer = Pacer::new(150_000_000);
        pacer.begin_quantum();
        pacer.end_quantum();
        let snap = pacer.stats().snapshot();
        assert!(snap.wall_ns > 0, "wall_ns should be non-zero after a quantum");
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_pacer_wall_ns_monotonic() {
        let mut pacer = Pacer::new(150_000_000);
        let mut last = 0u64;
        for _ in 0..5 {
            pacer.begin_quantum();
            pacer.end_quantum();
            let wall = pacer.stats().snapshot().wall_ns;
            assert!(wall > last, "wall_ns should grow each quantum: {} > {}", wall, last);
            last = wall;
        }
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_calibrate_overhead_clamped_tiny_nominal() {
        // With nominal = 200 ticks (~65ns on 3 GHz TSC), real overhead will
        // vastly exceed nominal/4 = 50, so the clamp kicks in and we return 50.
        let overhead = calibrate_overhead(200, 3_000_000_000);
        assert!(overhead <= 50, "clamp should limit overhead to nominal/4");
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_calibrate_overhead_reasonable() {
        // At nominal = 3000 ticks (~1µs on 3 GHz), overhead should be
        // measurable (>0) but well below nominal.
        let overhead = calibrate_overhead(3000, 3_000_000_000);
        assert!(overhead < 3000, "overhead should be below nominal");
        // Lower bound is hard to guarantee on all machines; just check clamp.
        assert!(overhead <= 750, "overhead should be clamped to nominal/4");
    }

    // -----------------------------------------------------------------------
    // Phase C: dynamic sys_clk_hz updates
    // -----------------------------------------------------------------------

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_update_sys_clk_hz_zero_keeps_previous() {
        // If firmware misconfigures CLK_SYS to an unconfigured PLL (0 Hz),
        // the Pacer must keep the previous quantum rather than divide-by-zero.
        let mut pacer = Pacer::new(6_500_000);
        let before = pacer.quantum_tsc_ticks();
        pacer.update_sys_clk_hz(0);
        assert_eq!(
            pacer.quantum_tsc_ticks(),
            before,
            "zero-Hz update must preserve previous quantum_tsc_ticks"
        );
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_update_sys_clk_hz_changes_quantum() {
        // Jumping from ROSC (6.5 MHz) to a 150 MHz PLL is a ~23× speedup,
        // so the host-tick budget per quantum should shrink by ~23×. We
        // allow a generous tolerance because the stored overhead is
        // subtracted from both (which biases the ratio) and calibration
        // noise is environment-dependent.
        let mut pacer = Pacer::new(6_500_000);
        let old_ticks = pacer.quantum_tsc_ticks();
        assert!(old_ticks > 0, "baseline quantum must be non-zero");

        pacer.update_sys_clk_hz(150_000_000);
        let new_ticks = pacer.quantum_tsc_ticks();
        assert!(new_ticks > 0, "new quantum must be non-zero");
        assert!(
            new_ticks < old_ticks,
            "150 MHz quantum should be smaller than 6.5 MHz quantum (old={}, new={})",
            old_ticks,
            new_ticks
        );

        // Expected ratio ≈ 150M / 6.5M ≈ 23.08. Allow 15×..35× to absorb
        // overhead-subtraction bias and TSC calibration jitter.
        let ratio = old_ticks as f64 / new_ticks as f64;
        assert!(
            (15.0..=35.0).contains(&ratio),
            "ratio should be ~23x, got {:.2} (old={}, new={})",
            ratio,
            old_ticks,
            new_ticks
        );
    }

    #[test]
    #[cfg(target_arch = "x86_64")]
    fn test_update_sys_clk_hz_noop_when_same() {
        // Calling update with the current frequency is the expected
        // per-quantum hot path when firmware hasn't touched the clock
        // tree. It must be a no-op: quantum_tsc_ticks unchanged.
        let mut pacer = Pacer::new(6_500_000);
        let before = pacer.quantum_tsc_ticks();
        pacer.update_sys_clk_hz(6_500_000);
        pacer.update_sys_clk_hz(6_500_000);
        assert_eq!(
            pacer.quantum_tsc_ticks(),
            before,
            "same-frequency update must be a no-op"
        );
    }
}

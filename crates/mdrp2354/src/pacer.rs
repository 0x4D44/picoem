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
    /// Whether pacing is currently active.
    running: AtomicBool,
}

impl PacerStats {
    pub fn new() -> Self {
        Self {
            emulated_cycles: AtomicU64::new(0),
            emulation_ns: AtomicU64::new(0),
            spin_ns: AtomicU64::new(0),
            behind_count: AtomicU64::new(0),
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
        }
    }

    pub fn add_emulated_cycles(&self, n: u64) {
        self.emulated_cycles.fetch_add(n, Ordering::Relaxed);
    }

    pub fn add_emulation_ns(&self, n: u64) {
        self.emulation_ns.fetch_add(n, Ordering::Relaxed);
    }

    pub fn add_spin_ns(&self, n: u64) {
        self.spin_ns.fetch_add(n, Ordering::Relaxed);
    }

    pub fn increment_behind(&self) {
        self.behind_count.fetch_add(1, Ordering::Relaxed);
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
pub struct PacerSnapshot {
    pub emulated_cycles: u64,
    pub emulation_ns: u64,
    pub spin_ns: u64,
    pub behind_count: u64,
}

impl PacerSnapshot {
    /// Total host nanoseconds (emulation + spin).
    pub fn total_ns(&self) -> u64 {
        self.emulation_ns + self.spin_ns
    }

    /// Fraction of time spent emulating (0.0..=1.0).
    pub fn utilization(&self) -> f64 {
        let total = self.total_ns();
        if total == 0 {
            return 0.0;
        }
        self.emulation_ns as f64 / total as f64
    }

    /// Fraction of time spent spinning (1.0 - utilization).
    pub fn headroom(&self) -> f64 {
        1.0 - self.utilization()
    }

    /// Effective emulated clock rate in MHz.
    pub fn emulated_mhz(&self) -> f64 {
        let total = self.total_ns();
        if total == 0 {
            return 0.0;
        }
        self.emulated_cycles as f64 / total as f64 * 1000.0
    }
}

// ---------------------------------------------------------------------------
// Pacer — real-time pacing via rdtsc spin-wait
// ---------------------------------------------------------------------------

#[cfg(target_arch = "x86_64")]
#[inline(always)]
fn rdtsc() -> u64 {
    unsafe { std::arch::x86_64::_rdtsc() }
}

/// Calibrate the TSC frequency by measuring rdtsc ticks over a short sleep.
#[cfg(target_arch = "x86_64")]
fn calibrate_tsc() -> u64 {
    let t0 = std::time::Instant::now();
    let tsc0 = rdtsc();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let tsc1 = rdtsc();
    let elapsed_ns = t0.elapsed().as_nanos() as u64;
    (tsc1 - tsc0) * 1_000_000_000 / elapsed_ns
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
    /// Calibrated TSC frequency in Hz.
    tsc_freq_hz: u64,
    /// Emulator system clock in Hz (e.g. 150_000_000).
    /// Retained for potential runtime quantum reconfiguration.
    #[allow(dead_code)]
    sys_clk_hz: u64,
}

#[cfg(target_arch = "x86_64")]
impl Pacer {
    /// Create a new pacer for the given emulator clock frequency.
    /// Calibrates the TSC at construction time (~50 ms one-time cost).
    pub fn new(sys_clk_hz: u32) -> Self {
        let tsc_freq_hz = calibrate_tsc();
        let quantum_cycles: u64 = 150;
        let quantum_tsc_ticks =
            (tsc_freq_hz as u128 * quantum_cycles as u128 / sys_clk_hz as u128) as u64;

        Self {
            stats: Arc::new(PacerStats::new()),
            quantum_cycles,
            quantum_tsc_ticks,
            quantum_start_tsc: 0,
            tsc_freq_hz,
            sys_clk_hz: sys_clk_hz as u64,
        }
    }

    /// Create a pacer with a custom quantum size.
    pub fn with_quantum(sys_clk_hz: u32, quantum_cycles: u64) -> Self {
        let mut pacer = Self::new(sys_clk_hz);
        pacer.quantum_cycles = quantum_cycles;
        pacer.quantum_tsc_ticks =
            (pacer.tsc_freq_hz as u128 * quantum_cycles as u128 / sys_clk_hz as u128) as u64;
        pacer
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

    /// Mark the start of a quantum. Call before stepping the emulator.
    #[inline(always)]
    pub fn begin_quantum(&mut self) {
        self.quantum_start_tsc = rdtsc();
    }

    /// End a quantum. Spin-waits if we're ahead of real-time, updates stats.
    /// Call after stepping the emulator for `quantum_cycles()` cycles.
    #[inline(always)]
    pub fn end_quantum(&mut self) {
        let emulation_tsc = rdtsc() - self.quantum_start_tsc;

        if emulation_tsc < self.quantum_tsc_ticks {
            // Ahead of real-time — spin wait
            let target_tsc = self.quantum_start_tsc + self.quantum_tsc_ticks;
            while rdtsc() < target_tsc {
                std::hint::spin_loop();
            }
            let total_tsc = rdtsc() - self.quantum_start_tsc;
            let spin_tsc = total_tsc - emulation_tsc;

            self.stats.add_emulation_ns(self.tsc_to_ns(emulation_tsc));
            self.stats.add_spin_ns(self.tsc_to_ns(spin_tsc));
        } else {
            // Behind real-time — skip spin, log it
            self.stats.add_emulation_ns(self.tsc_to_ns(emulation_tsc));
            self.stats.increment_behind();
        }

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
}

use std::sync::atomic::{AtomicU64, AtomicBool, Ordering};

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
}

//! Flat spin barrier for worker-thread synchronisation.
//!
//! A fixed-party rendezvous point: each `wait()` caller blocks until
//! `parties` threads have arrived, then all release simultaneously and
//! the barrier is ready for the next round. Promoted from the Phase 0c
//! prototype (`threading_micro.rs`).
//!
//! See `wrk_docs/2026.04.17 - LLD - Threaded Dual-Core Phase 2 V4.md` §2.
//!
//! ## Mechanism
//!
//! A generation counter distinguishes barrier rounds. The last arriver
//! resets `count` and bumps `generation`; earlier arrivers spin until
//! they observe a new generation. There is no fallback to futexes —
//! callers are worker threads that will hit the barrier within
//! microseconds, so wasted spinning is bounded.
//!
//! ## Poisoning
//!
//! If a worker panics before reaching the barrier, the remaining
//! threads would spin forever. The coordinator catches the panic
//! (Phase 4) and calls [`SpinBarrier::poison`], which unblocks all
//! current and future waiters with [`BarrierResult::Poisoned`]. Phase 2
//! provides the primitive; the `catch_unwind` wiring lands in Phase 4.
//!
//! Phase 0c measured ~425 ns mean round-trip (4-way), within the
//! <500 ns threshold.
//!
//! ## Cross-chip reuse
//!
//! This type is chip-agnostic and may move to `mdpicoem-common` in
//! Phase 3 when the RP2040 threaded path lands.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering::*};

/// Outcome of a [`SpinBarrier::wait`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BarrierResult {
    /// All `parties` arrived; this waiter has been released.
    Released,
    /// The barrier was poisoned before release; caller should unwind.
    Poisoned,
}

/// Fixed-party flat barrier with panic-safe poisoning.
///
/// Constructed once and shared (typically behind an `Arc`) across the
/// worker threads. Both `wait()` and `poison()` take `&self`, so no
/// external locking is needed.
pub struct SpinBarrier {
    generation: AtomicU32,
    count: AtomicU32,
    parties: u32,
    poisoned: AtomicBool,
}

impl SpinBarrier {
    /// Create a barrier that releases when `parties` threads arrive.
    ///
    /// Panics if `parties < 2` — a single-party barrier is degenerate
    /// and almost always indicates a bug at the call site.
    pub fn new(parties: u32) -> Self {
        assert!(parties >= 2);
        Self {
            generation: AtomicU32::new(0),
            count: AtomicU32::new(0),
            parties,
            poisoned: AtomicBool::new(false),
        }
    }

    /// Block until all `parties` threads have arrived at this barrier.
    ///
    /// Returns [`BarrierResult::Released`] on normal release, or
    /// [`BarrierResult::Poisoned`] if [`Self::poison`] was called
    /// while this thread was waiting (or before it entered `wait`).
    pub fn wait(&self) -> BarrierResult {
        if self.poisoned.load(Acquire) {
            return BarrierResult::Poisoned;
        }
        let cur_gen = self.generation.load(Acquire);
        let n = self.count.fetch_add(1, AcqRel) + 1;
        if n == self.parties {
            // Last arrival: reset count and bump generation, releasing
            // all earlier arrivals in one Release store.
            self.count.store(0, Relaxed);
            self.generation.store(cur_gen.wrapping_add(1), Release);
        } else {
            loop {
                if self.poisoned.load(Acquire) {
                    return BarrierResult::Poisoned;
                }
                if self.generation.load(Acquire) != cur_gen {
                    break;
                }
                std::hint::spin_loop();
            }
        }
        BarrierResult::Released
    }

    /// Abort all current and future waiters with [`BarrierResult::Poisoned`].
    ///
    /// One-way switch: once poisoned, the barrier stays poisoned for
    /// its lifetime. Intended for use by a panic-recovery coordinator.
    pub fn poison(&self) {
        self.poisoned.store(true, Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{
        AtomicU32, AtomicUsize,
        Ordering::{self, SeqCst},
    };
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn all_threads_released() {
        let barrier = Arc::new(SpinBarrier::new(4));
        let flags: [Arc<AtomicU32>; 4] = [
            Arc::new(AtomicU32::new(0)),
            Arc::new(AtomicU32::new(0)),
            Arc::new(AtomicU32::new(0)),
            Arc::new(AtomicU32::new(0)),
        ];

        let handles: Vec<_> = (0..4)
            .map(|i| {
                let b = Arc::clone(&barrier);
                let f = Arc::clone(&flags[i]);
                thread::spawn(move || {
                    match b.wait() {
                        BarrierResult::Released => f.store(1, SeqCst),
                        BarrierResult::Poisoned => panic!("unexpected poison"),
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().expect("thread panicked");
        }

        for f in &flags {
            assert_eq!(f.load(SeqCst), 1, "thread did not set released flag");
        }
    }

    #[test]
    fn multiple_rounds() {
        const PARTIES: u32 = 4;
        const ROUNDS: u32 = 10;

        let barrier = Arc::new(SpinBarrier::new(PARTIES));
        let counter = Arc::new(AtomicUsize::new(0));

        let handles: Vec<_> = (0..PARTIES)
            .map(|_| {
                let b = Arc::clone(&barrier);
                let c = Arc::clone(&counter);
                thread::spawn(move || {
                    for _ in 0..ROUNDS {
                        match b.wait() {
                            BarrierResult::Released => {
                                c.fetch_add(1, SeqCst);
                            }
                            BarrierResult::Poisoned => panic!("unexpected poison"),
                        }
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().expect("thread panicked");
        }

        assert_eq!(
            counter.load(SeqCst),
            (PARTIES * ROUNDS) as usize,
            "counter should equal parties * rounds"
        );
    }

    #[test]
    fn poison_breaks_waiters() {
        // 4-party barrier but only 3 waiters: without poisoning they
        // would spin forever. Main thread waits until all three workers
        // have entered the barrier (observable via `entered`), then
        // poisons. The small trailing sleep gives each worker time to
        // reach the spin loop after incrementing the counter — still
        // technically racy but far more robust than a flat 50ms wait.
        let barrier = Arc::new(SpinBarrier::new(4));
        let entered = Arc::new(AtomicU32::new(0));

        let handles: Vec<_> = (0..3)
            .map(|_| {
                let b = Arc::clone(&barrier);
                let e = Arc::clone(&entered);
                thread::spawn(move || {
                    e.fetch_add(1, Ordering::Release);
                    b.wait()
                })
            })
            .collect();

        while entered.load(Ordering::Acquire) < 3 {
            thread::sleep(Duration::from_millis(1));
        }
        thread::sleep(Duration::from_millis(10));
        barrier.poison();

        for h in handles {
            let result = h.join().expect("thread panicked");
            assert_eq!(
                result,
                BarrierResult::Poisoned,
                "waiter should have returned Poisoned"
            );
        }
    }
}

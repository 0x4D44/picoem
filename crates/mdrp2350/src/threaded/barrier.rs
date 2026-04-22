//! Hybrid spin+park barrier for worker-thread synchronisation.
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
//! resets `count`, bumps `generation`, and broadcasts via `Condvar`.
//! Earlier arrivers first spin for a short budget (`SPIN_BUDGET`) — the
//! fast path when all workers converge within a few hundred ns — and
//! only then fall back to `Condvar::wait` so they don't burn CPU while
//! one productive worker runs a long quantum alone. This matters under
//! single-core workloads where three of four workers have nothing to
//! do: the pure-spin variant had them pegging `pause` for the entire
//! quantum, bouncing cache lines against the productive core.
//!
//! ## Poisoning
//!
//! If a worker panics before reaching the barrier, the remaining
//! threads would wait forever. The coordinator catches the panic
//! (Phase 4) and calls [`SpinBarrier::poison`], which unblocks all
//! current and future waiters with [`BarrierResult::Poisoned`] via the
//! same `Condvar` broadcast.
//!
//! Phase 0c measured ~425 ns mean round-trip (4-way) on the pure-spin
//! variant; the hybrid keeps that fast path when all workers arrive in
//! close succession.
//!
//! ## Cross-chip reuse
//!
//! This type is chip-agnostic and may move to `mdpicoem-common` in
//! Phase 3 when the RP2040 threaded path lands.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering::*};
use std::sync::{Condvar, Mutex};

/// Spin iterations before falling back to `Condvar::wait`. `spin_loop()`
/// hints take ~20 ns on current x86, so 512 iterations is ~10 μs of
/// spin headroom. The previous value (128, ~2.5 μs) was tuned for a
/// general-purpose rendezvous where early arrivers should yield
/// quickly to the productive worker. That tuning is wrong for the
/// ThreadedEmulator's actual per-quantum shape: on OneROM-class
/// peripheral-heavy workloads, worker-to-worker arrival stagger
/// routinely hits 2 μs+ (PIO2 finishes ~2.5 μs after PIO0/core0 in
/// the §1.1 critical-path model), so a 2.5 μs budget forces every
/// barrier round through `park_cv.wait` / `notify_all` — a pair of
/// kernel transitions costing several microseconds each and erasing
/// the win from parallelising the blocks in the first place.
///
/// At 512 iterations (~10 μs) no parking occurs in realistic OneROM
/// workloads — measured via `threading_micro` §9 late-arriver sampler:
/// p50 400-600 ns, p99 ≤ 1000 ns round-trip. The cost ceiling rises
/// symmetrically: worst-case burn is ~160 μs per barrier if every
/// worker is idle while one runs 100 μs+ alone. On the dedicated
/// pinned host cores the ThreadedEmulator targets (§2.5), that burn
/// is dissipating host CPU that nothing else wants; on a general-
/// purpose host sharing cores with other workloads, 160 μs of hot
/// spin per quantum would be unacceptable, but that configuration is
/// out of scope for this runtime.
///
/// See `wrk_journals/2026.04.22 - JRN - Threaded PIO Split
/// Implementation.md` for the measurement data backing these numbers.
const SPIN_BUDGET: u32 = 512;

/// Outcome of a [`SpinBarrier::wait`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BarrierResult {
    /// All `parties` arrived; this waiter has been released.
    Released,
    /// The barrier was poisoned before release; caller should unwind.
    Poisoned,
}

/// Fixed-party hybrid barrier with panic-safe poisoning.
///
/// Constructed once and shared (typically behind an `Arc`) across the
/// worker threads. Both `wait()` and `poison()` take `&self`, so no
/// external locking is needed. Name retained from the Phase 0c
/// spin-only prototype (`SpinBarrier`) for call-site compatibility;
/// the implementation is now spin-then-park.
pub struct SpinBarrier {
    generation: AtomicU32,
    count: AtomicU32,
    parties: u32,
    poisoned: AtomicBool,
    /// Held briefly by the last arriver (around the `generation` store)
    /// and by earlier arrivers once their spin budget is exhausted
    /// (across a `Condvar::wait`). Uncontended in the fast path.
    park_mu: Mutex<()>,
    park_cv: Condvar,
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
            park_mu: Mutex::new(()),
            park_cv: Condvar::new(),
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
            // Last arrival: bump generation under the park mutex so any
            // waiter currently inside `park_cv.wait_while` — which holds
            // `park_mu` around its predicate check — linearises on the
            // old-gen side or the new-gen side, never in a window that
            // could miss the broadcast.
            {
                let _g = self.park_mu.lock().unwrap();
                self.count.store(0, Relaxed);
                self.generation.store(cur_gen.wrapping_add(1), Release);
            }
            self.park_cv.notify_all();
            return BarrierResult::Released;
        }

        // Earlier arriver — spin briefly on the fast path.
        for _ in 0..SPIN_BUDGET {
            if self.poisoned.load(Acquire) {
                return BarrierResult::Poisoned;
            }
            if self.generation.load(Acquire) != cur_gen {
                return BarrierResult::Released;
            }
            std::hint::spin_loop();
        }

        // Fast path exhausted — sleep on the condvar. Idle-for-most-of-
        // the-quantum workers hit this path and stop burning CPU cycles
        // that the productive worker would otherwise lose to cache-
        // coherence traffic on the shared barrier lines.
        let mut g = self.park_mu.lock().unwrap();
        while !self.poisoned.load(Acquire) && self.generation.load(Acquire) == cur_gen {
            g = self.park_cv.wait(g).unwrap();
        }
        if self.poisoned.load(Acquire) {
            BarrierResult::Poisoned
        } else {
            BarrierResult::Released
        }
    }

    /// Abort all current and future waiters with [`BarrierResult::Poisoned`].
    ///
    /// One-way switch: once poisoned, the barrier stays poisoned for
    /// its lifetime. Intended for use by a panic-recovery coordinator.
    /// Broadcasts on the park condvar so any sleeping waiter wakes up
    /// immediately rather than on the next timeout.
    pub fn poison(&self) {
        // Take/drop the mutex to linearise with `park_cv.wait` predicate
        // checks on the sleeping-waiter side, same reasoning as the
        // generation store in `wait`.
        {
            let _g = self.park_mu.lock().unwrap();
            self.poisoned.store(true, Release);
        }
        self.park_cv.notify_all();
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

    /// HLD V5 §4 item 12 — `ThreadedEmulator` now rendezvouses six
    /// workers (core0, core1, pio0, pio1, pio2, coord) per quantum, so
    /// the primitive's own unit-test surface should cover that arity
    /// explicitly. Sibling of `multiple_rounds` with `PARTIES = 6`.
    #[test]
    fn multiple_rounds_6way() {
        const PARTIES: u32 = 6;
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

    /// Asymmetric-arrival sibling to `multiple_rounds_6way`. One worker
    /// busy-waits ~2 μs past the others before entering `wait()`. With
    /// `SPIN_BUDGET=512` (~10 μs spin ceiling) no worker should park.
    /// A regression that lowers `SPIN_BUDGET` below the ~2 μs stagger
    /// ceiling would cause `Condvar::wait`/`notify_all` cycles —
    /// measurable as a huge p99 spike in the microbench but previously
    /// undetected by `cargo test`.
    ///
    /// This test doesn't measure timing; it just confirms 10 rounds
    /// complete (no deadlock, no poison), exercising the late-arriver
    /// code path that pure-symmetric `multiple_rounds` misses.
    #[test]
    fn parties_6_asymmetric_arrival_does_not_park() {
        const PARTIES: u32 = 6;
        const ROUNDS: u32 = 10;
        let barrier = Arc::new(SpinBarrier::new(PARTIES));
        let counter = Arc::new(AtomicU32::new(0));

        let handles: Vec<_> = (0..PARTIES)
            .map(|tid| {
                let b = Arc::clone(&barrier);
                let c = Arc::clone(&counter);
                thread::spawn(move || {
                    for _ in 0..ROUNDS {
                        if tid == PARTIES - 1 {
                            // Late arriver busy-waits ~2 μs.
                            let t0 = std::time::Instant::now();
                            while t0.elapsed() < Duration::from_micros(2) {
                                std::hint::spin_loop();
                            }
                        }
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
        assert_eq!(counter.load(SeqCst), PARTIES * ROUNDS);
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

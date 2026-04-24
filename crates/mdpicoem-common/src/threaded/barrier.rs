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
//! ## Watchdog (Stage 5, HLD V1 §6.6)
//!
//! A worker that deadlocks *without* panicking — a PIO block stuck in
//! an infinite `JMP` loop, a CPU core polling an MMIO read that never
//! resolves — leaves the poison path untouched. Every `wait` call
//! therefore carries its own wall-clock deadline
//! ([`DEFAULT_DEADLINE`] unless constructed via [`SpinBarrier::with_deadline`]).
//! The spin loop checks `Instant::now()` every [`WATCHDOG_STRIDE`]
//! iterations and the park loop uses
//! [`Condvar::wait_timeout`](std::sync::Condvar::wait_timeout) with the
//! remaining budget, so expiry is detected promptly on both the hot
//! and cold paths. On expiry, the first waiter records the elapsed
//! time, flips [`SpinBarrier::timed_out`], and calls
//! [`SpinBarrier::poison`] so peer waiters wake into a
//! [`BarrierResult::TimedOut`] exit instead of a latent hang.
//!
//! Phase 0c measured ~425 ns mean round-trip (4-way) on the pure-spin
//! variant; the hybrid keeps that fast path when all workers arrive in
//! close succession.
//!
//! ## Cross-chip reuse
//!
//! This type is chip-agnostic and may move to `mdpicoem-common` in
//! Phase 3 when the RP2040 threaded path lands.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering::*};
use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

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

/// Watchdog wall-clock check cadence. The spin loop re-reads `Instant::now()`
/// every `WATCHDOG_STRIDE` iterations to bound per-iteration overhead. At
/// `~20 ns` per spin hint, `1024` iterations is `~20 μs` between checks —
/// comfortably below the default 5-second deadline but cheap enough to
/// keep the fast path clean (see `threading_micro` regression target
/// <2%). The stride is a power of two so the test is a bitwise `and`.
const WATCHDOG_STRIDE: u32 = 1024;

/// Default wall-clock deadline for [`SpinBarrier::wait`]. Picked as a
/// simple upper bound that catches pathological deadlocks without firing
/// on any realistic per-quantum rendezvous (Phase 0c measured p99 ≤ 1 μs
/// for a 4-way barrier; the ThreadedEmulator's 6-way case runs quanta
/// well under a millisecond even on slow hosts). HLD V1 §6.6 suggested a
/// clock-derived formula (`quantum_cycles × 256 / min_clock_hz + 5 s`);
/// a fixed 5 s is strictly simpler and leaves room to refine later.
pub const DEFAULT_DEADLINE: Duration = Duration::from_secs(5);

/// Outcome of a [`SpinBarrier::wait`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BarrierResult {
    /// All `parties` arrived; this waiter has been released.
    Released,
    /// The barrier was poisoned before release; caller should unwind.
    Poisoned,
    /// The per-barrier wall-clock deadline expired before all `parties`
    /// arrived. The waiter that observes this value has already poisoned
    /// the barrier so the remaining waiters wake on their next check.
    /// `elapsed_ms` is the wall-clock time between the waiter's own
    /// [`SpinBarrier::wait`] entry and the expiry detection, for
    /// diagnostics only (capped at `u32::MAX` ms ≈ 49 days).
    TimedOut { elapsed_ms: u32 },
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
    /// Set when any waiter observes deadline expiry (distinct from
    /// ordinary panic-triggered poisoning). The watchdog sets this
    /// flag *and* calls [`Self::poison`] so other waiters wake out of
    /// park, but the coordinator can distinguish a watchdog-fired exit
    /// from a panic-fired exit by reading [`Self::timed_out`].
    timed_out: AtomicBool,
    /// Elapsed milliseconds from the first timing-out waiter's entry to
    /// its detection of expiry. Written once by the first waiter to
    /// trip the watchdog, zero otherwise. Surfaced to callers via the
    /// `TimedOut { elapsed_ms }` variant and a getter.
    timeout_elapsed_ms: AtomicU64,
    /// Wall-clock deadline applied to every waiter individually (not a
    /// shared round deadline). Each [`Self::wait`] call times from its
    /// own entry.
    deadline: Duration,
    /// Held briefly by the last arriver (around the `generation` store)
    /// and by earlier arrivers once their spin budget is exhausted
    /// (across a `Condvar::wait`). Uncontended in the fast path.
    park_mu: Mutex<()>,
    park_cv: Condvar,
}

impl SpinBarrier {
    /// Create a barrier that releases when `parties` threads arrive.
    /// Uses the default wall-clock deadline ([`DEFAULT_DEADLINE`]).
    ///
    /// Panics if `parties < 2` — a single-party barrier is degenerate
    /// and almost always indicates a bug at the call site.
    pub fn new(parties: u32) -> Self {
        Self::with_deadline(parties, DEFAULT_DEADLINE)
    }

    /// Create a barrier with an explicit per-waiter wall-clock deadline.
    ///
    /// Primarily intended for tests that need a short deadline to
    /// exercise the watchdog path; production code should use
    /// [`Self::new`] and inherit [`DEFAULT_DEADLINE`].
    pub fn with_deadline(parties: u32, deadline: Duration) -> Self {
        assert!(parties >= 2);
        Self {
            generation: AtomicU32::new(0),
            count: AtomicU32::new(0),
            parties,
            poisoned: AtomicBool::new(false),
            timed_out: AtomicBool::new(false),
            timeout_elapsed_ms: AtomicU64::new(0),
            deadline,
            park_mu: Mutex::new(()),
            park_cv: Condvar::new(),
        }
    }

    /// Block until all `parties` threads have arrived at this barrier.
    ///
    /// Returns [`BarrierResult::Released`] on normal release,
    /// [`BarrierResult::Poisoned`] if [`Self::poison`] was called
    /// while this thread was waiting (or before it entered `wait`),
    /// or [`BarrierResult::TimedOut`] if the per-waiter wall-clock
    /// deadline elapsed before all parties arrived. On timeout the
    /// observing waiter poisons the barrier so remaining waiters exit
    /// promptly; the barrier is single-use after that.
    pub fn wait(&self) -> BarrierResult {
        let start = Instant::now();
        if self.poisoned.load(Acquire) {
            return self.poisoned_or_timed_out_result(start);
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

        // Earlier arriver — spin briefly on the fast path. The watchdog
        // check is amortised across `WATCHDOG_STRIDE` iterations so the
        // fast-path cost of reading `Instant::now()` lands well under
        // 1% of the per-barrier-round figure measured by
        // `threading_micro`.
        let mut i: u32 = 0;
        for _ in 0..SPIN_BUDGET {
            if self.poisoned.load(Acquire) {
                return self.poisoned_or_timed_out_result(start);
            }
            if self.generation.load(Acquire) != cur_gen {
                return BarrierResult::Released;
            }
            i = i.wrapping_add(1);
            if i & (WATCHDOG_STRIDE - 1) == 0 && start.elapsed() >= self.deadline {
                return self.trip_watchdog(start);
            }
            std::hint::spin_loop();
        }

        // Fast path exhausted — sleep on the condvar with a remaining-
        // budget timeout so the watchdog still fires even if every
        // waiter parked before expiry. Idle-for-most-of-the-quantum
        // workers hit this path and stop burning CPU cycles that the
        // productive worker would otherwise lose to cache-coherence
        // traffic on the shared barrier lines.
        let mut g = self.park_mu.lock().unwrap();
        loop {
            // Check release first so a late-spin-path generation bump
            // we missed still exits as `Released` rather than falling
            // into the poisoned-or-timed-out result path.
            if self.generation.load(Acquire) != cur_gen {
                return BarrierResult::Released;
            }
            if self.poisoned.load(Acquire) {
                return self.poisoned_or_timed_out_result(start);
            }
            let remaining = self.deadline.saturating_sub(start.elapsed());
            if remaining.is_zero() {
                drop(g);
                return self.trip_watchdog(start);
            }
            let (gg, _timeout) = self.park_cv.wait_timeout(g, remaining).unwrap();
            g = gg;
            // The top-of-loop predicate handles both the "notified"
            // and the "spurious / timeout" returns uniformly, so we
            // don't need to branch on `timeout.timed_out()` here.
        }
    }

    /// Return either `Poisoned` or `TimedOut` depending on whether the
    /// poisoning was caused by watchdog expiry. Shared by every early-
    /// exit path in `wait`. `start` is only consulted when the calling
    /// waiter is reporting its own elapsed time and no prior waiter has
    /// written `timeout_elapsed_ms`.
    #[inline]
    fn poisoned_or_timed_out_result(&self, start: Instant) -> BarrierResult {
        if self.timed_out.load(Acquire) {
            let recorded = self.timeout_elapsed_ms.load(Acquire);
            let elapsed_ms = if recorded != 0 {
                recorded.min(u32::MAX as u64) as u32
            } else {
                start.elapsed().as_millis().min(u32::MAX as u128) as u32
            };
            BarrierResult::TimedOut { elapsed_ms }
        } else {
            BarrierResult::Poisoned
        }
    }

    /// First waiter to detect deadline expiry records the elapsed time,
    /// flips `timed_out`, and poisons the barrier so other waiters wake
    /// up immediately. Subsequent callers re-enter via
    /// [`Self::poisoned_or_timed_out_result`] and observe the recorded
    /// elapsed time.
    #[inline]
    fn trip_watchdog(&self, start: Instant) -> BarrierResult {
        let elapsed = start.elapsed();
        let elapsed_ms = elapsed.as_millis().min(u32::MAX as u128) as u32;
        // Race: two waiters can both elapse before the first CAS. The
        // first to store wins; the second observes the recorded value
        // via `poisoned_or_timed_out_result`.
        let _ = self.timeout_elapsed_ms.compare_exchange(
            0,
            elapsed_ms as u64,
            AcqRel,
            Acquire,
        );
        self.timed_out.store(true, Release);
        self.poison();
        BarrierResult::TimedOut { elapsed_ms }
    }

    /// `true` if the barrier has transitioned to the timed-out state
    /// (any waiter observed the deadline expire). Distinct from
    /// ordinary [`Self::poison`] so the coordinator can attribute a
    /// runtime exit to a watchdog rather than a panic.
    pub fn timed_out(&self) -> bool {
        self.timed_out.load(Acquire)
    }

    /// Elapsed-milliseconds snapshot recorded when the watchdog first
    /// fired; zero if the barrier never timed out. `u32` because a
    /// `Duration` isn't `Clone`-friendly inside the emulator error
    /// variant that surfaces this value.
    pub fn timeout_elapsed_ms(&self) -> u32 {
        self.timeout_elapsed_ms
            .load(Acquire)
            .min(u32::MAX as u64) as u32
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
                        BarrierResult::TimedOut { .. } => panic!("unexpected timeout"),
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
                            BarrierResult::TimedOut { .. } => panic!("unexpected timeout"),
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
                            BarrierResult::TimedOut { .. } => panic!("unexpected timeout"),
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
                            BarrierResult::TimedOut { .. } => panic!("unexpected timeout"),
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

    /// Stage 5 (HLD V1 §6.6): a stalled worker must no longer deadlock
    /// the rendezvous. Two parties, only one arrives; the other waiter
    /// must observe `TimedOut` within the configured deadline + a small
    /// wake-up cushion. Uses a short 150 ms deadline so the test runs
    /// in well under a second; the production default is 5 s.
    #[test]
    fn barrier_watchdog_fires_when_worker_stalls() {
        const DEADLINE: Duration = Duration::from_millis(150);
        let barrier = Arc::new(SpinBarrier::with_deadline(2, DEADLINE));
        let b = Arc::clone(&barrier);

        let start = std::time::Instant::now();
        let handle = thread::spawn(move || b.wait());
        // Main thread never arrives — the spawned waiter should trip
        // the watchdog all by itself.
        let result = handle.join().expect("worker panicked");

        let elapsed = start.elapsed();
        // Watchdog should fire between the deadline and the deadline
        // plus a generous host-scheduler cushion. The lower bound is
        // the interesting correctness check; the upper bound catches
        // regressions where the stride-based check forgets to also
        // honour the condvar-timeout path.
        assert!(
            elapsed >= DEADLINE,
            "watchdog fired too early: {elapsed:?} < {DEADLINE:?}"
        );
        assert!(
            elapsed < DEADLINE + Duration::from_millis(500),
            "watchdog took too long: {elapsed:?} > {:?}",
            DEADLINE + Duration::from_millis(500)
        );

        match result {
            BarrierResult::TimedOut { elapsed_ms } => {
                assert!(
                    elapsed_ms >= DEADLINE.as_millis() as u32,
                    "reported elapsed_ms {elapsed_ms} < deadline {}",
                    DEADLINE.as_millis()
                );
            }
            other => panic!("expected TimedOut, got {other:?}"),
        }

        // One-shot poison semantics: every subsequent wait returns the
        // same variant without blocking. Without this property a
        // panicking coordinator could re-enter a watchdog-fired barrier
        // and hang again.
        let reentry = barrier.wait();
        match reentry {
            BarrierResult::TimedOut { .. } => {}
            other => panic!("re-entry after timeout should yield TimedOut, got {other:?}"),
        }
        assert!(barrier.timed_out());
        assert!(barrier.timeout_elapsed_ms() >= DEADLINE.as_millis() as u32);
    }
}

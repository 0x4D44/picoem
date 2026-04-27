//! Threading Micro-Prototype (Phase 0c + Stage 0 measurement gate)
//!
//! Standalone benchmark measuring the cost of threading primitives that the
//! threaded dual-core emulation design depends on.  Results feed the go/no-go
//! decision before committing to a full threaded architecture.
//!
//! Measures:
//!   1. SpinBarrier 4-way round-trip (local spin-only, historical row)
//!   2. AtomicU32 contended vs uncontended throughput
//!   3. Masked CAS loop cost (byte-granularity writes on word atomics)
//!   4. SPSC bounded-queue throughput
//!   5. Mutex contention cycle time
//!   6. Production `SpinBarrier` sweep at parties ∈ {2, 4, 6, 8}
//!      (symmetric arrival, mean / p50 / p99 round-trip).
//!   7. Production `SpinBarrier` asymmetric-arrival at parties=6 — one
//!      worker sleeps ~2 µs past the others before calling `wait()`,
//!      exercising the spin-budget-exhaustion → `Condvar::wait` → `notify_all`
//!      path that OneROM's real-workload pattern triggers (PIO2 finishes
//!      well after the rest). Mean / p50 / p99 round-trip.
//!   8. Parameterised-SPIN_BUDGET barrier sweep — the same 6-way
//!      asymmetric-2µs case repeated across SPIN_BUDGET ∈ {128, 256, 512,
//!      1024, 2048, 4096, 8192} to locate the knee where early arrivers
//!      stay on the spin path through the 2 µs stagger and nobody parks.
//!      Uses a local `ParamSpinBarrier` (logic cloned from the production
//!      `SpinBarrier` minus the poison path) because the real primitive's
//!      SPIN_BUDGET is a `const`.
//!   9. Late-arriver tail sweep — same 6-way asymmetric-2µs pattern and
//!      SPIN_BUDGET sweep as §8, but the sampled thread is now the *late*
//!      arriver rather than an early arriver. This measures pure release
//!      propagation (no stagger, no park on the sampler's side) — the
//!      metric that actually bounds per-quantum throughput in the
//!      ThreadedEmulator, since a round cannot finish until the late
//!      arriver's `wait()` returns.
//!
//! Sections 6–9 back the §7 pre-implementation measurement gate in
//! `wrk_docs/2026.04.22 - HLD - Threaded PIO Per-Block Workers V5.md`.
//! They use the production hybrid (spin-then-park) barrier from
//! `mdrp2350::threaded::barrier` — not the local spin-only shim used by
//! sections 1–5 — because the §7 asymmetric case depends on the parked-
//! waiter wakeup path that only the production primitive implements.
//! Section 8 uses a local clone of the production barrier so SPIN_BUDGET
//! can be parameterised at runtime; the clone mirrors the production
//! fences and park path exactly (see `ParamSpinBarrier`).
//! Threads are intentionally **not pinned** in sections 6–8: we want to
//! measure the barrier primitive, not the OS scheduler overlay, and the
//! HLD's §1.1 model captures scheduler effects separately via its
//! conservative band.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering::*};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use mdrp2350::threaded::SpinBarrier as ProdSpinBarrier;

// ─────────────────────────────────────────────────────────────────────────────
// Windows thread-affinity shim (matches paced_bench pattern)
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(target_os = "windows")]
mod affinity {
    #[allow(non_camel_case_types)]
    type HANDLE = *mut std::ffi::c_void;
    #[allow(non_camel_case_types)]
    type DWORD_PTR = usize;

    unsafe extern "system" {
        fn GetCurrentThread() -> HANDLE;
        fn SetThreadAffinityMask(hThread: HANDLE, dwThreadAffinityMask: DWORD_PTR) -> DWORD_PTR;
    }

    /// Pin the calling thread to a specific logical core.
    pub fn pin_to_core(core: usize) {
        unsafe {
            let mask: DWORD_PTR = 1 << core;
            SetThreadAffinityMask(GetCurrentThread(), mask);
        }
    }
}

#[cfg(not(target_os = "windows"))]
mod affinity {
    pub fn pin_to_core(_core: usize) {
        // No-op on non-Windows; could use libc sched_setaffinity on Linux.
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. SpinBarrier
// ─────────────────────────────────────────────────────────────────────────────

#[allow(dead_code)]
enum BarrierResult {
    Released,
    Poisoned,
}

struct SpinBarrier {
    generation: AtomicU32,
    count: AtomicU32,
    parties: u32,
    poisoned: AtomicBool,
}

impl SpinBarrier {
    fn new(parties: u32) -> Self {
        Self {
            generation: AtomicU32::new(0),
            count: AtomicU32::new(0),
            parties,
            poisoned: AtomicBool::new(false),
        }
    }

    fn wait(&self) -> BarrierResult {
        if self.poisoned.load(Acquire) {
            return BarrierResult::Poisoned;
        }
        let cur_gen = self.generation.load(Acquire);
        let n = self.count.fetch_add(1, AcqRel) + 1;
        if n == self.parties {
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

    #[allow(dead_code)]
    fn poison(&self) {
        self.poisoned.store(true, Release);
    }
}

fn bench_spin_barrier() {
    const PARTIES: u32 = 4;
    const ROUNDS: u32 = 1_000_000;

    let barrier = Arc::new(SpinBarrier::new(PARTIES));

    // Warmup
    {
        let handles: Vec<_> = (0..PARTIES)
            .map(|i| {
                let b = Arc::clone(&barrier);
                std::thread::spawn(move || {
                    affinity::pin_to_core(i as usize);
                    for _ in 0..1000 {
                        b.wait();
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    // Reset barrier state after warmup
    let barrier = Arc::new(SpinBarrier::new(PARTIES));

    let start = Instant::now();
    let handles: Vec<_> = (0..PARTIES)
        .map(|i| {
            let b = Arc::clone(&barrier);
            std::thread::spawn(move || {
                affinity::pin_to_core(i as usize);
                for _ in 0..ROUNDS {
                    b.wait();
                }
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let elapsed = start.elapsed();

    let mean_ns = elapsed.as_nanos() as f64 / ROUNDS as f64;
    println!("1. SpinBarrier (4-way, {}M rounds)", ROUNDS / 1_000_000);
    println!("   Mean round-trip:   {:.0} ns", mean_ns);
    println!("   Total time:        {:.3}s", elapsed.as_secs_f64());
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. AtomicU32 throughput (same vs different cacheline)
// ─────────────────────────────────────────────────────────────────────────────

#[repr(align(64))]
struct CacheLinePadded {
    value: AtomicU32,
}

impl CacheLinePadded {
    fn new(v: u32) -> Self {
        Self {
            value: AtomicU32::new(v),
        }
    }
}

fn bench_atomic_throughput() {
    const OPS_PER_THREAD: u64 = 10_000_000;
    const THREADS: usize = 2;

    // --- Same cacheline (contended) ---
    let shared = Arc::new(AtomicU32::new(0));

    // Warmup
    {
        let s = Arc::clone(&shared);
        for _ in 0..10_000 {
            s.fetch_add(1, Relaxed);
        }
        s.store(0, Relaxed);
    }

    let start = Instant::now();
    let handles: Vec<_> = (0..THREADS)
        .map(|i| {
            let s = Arc::clone(&shared);
            std::thread::spawn(move || {
                affinity::pin_to_core(i);
                for _ in 0..OPS_PER_THREAD {
                    s.fetch_add(1, Relaxed);
                }
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let contended_elapsed = start.elapsed();
    let contended_mops =
        (THREADS as f64 * OPS_PER_THREAD as f64) / contended_elapsed.as_secs_f64() / 1e6;

    // --- Different cacheline (uncontended) ---
    let padded = Arc::new([CacheLinePadded::new(0), CacheLinePadded::new(0)]);

    let start = Instant::now();
    let handles: Vec<_> = (0..THREADS)
        .map(|i| {
            let p = Arc::clone(&padded);
            std::thread::spawn(move || {
                affinity::pin_to_core(i);
                for _ in 0..OPS_PER_THREAD {
                    p[i].value.fetch_add(1, Relaxed);
                }
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let uncontended_elapsed = start.elapsed();
    let uncontended_mops =
        (THREADS as f64 * OPS_PER_THREAD as f64) / uncontended_elapsed.as_secs_f64() / 1e6;

    let ratio = uncontended_mops / contended_mops;

    println!(
        "\n2. AtomicU32 throughput ({}M ops/thread)",
        OPS_PER_THREAD / 1_000_000
    );
    println!(
        "   Same cacheline:    {:.1} Mops/s  (contended)",
        contended_mops
    );
    println!(
        "   Diff cacheline:    {:.1} Mops/s  (uncontended)",
        uncontended_mops
    );
    println!("   Ratio:             {:.1}x", ratio);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Masked CAS loop (byte-granularity writes on word atomics)
// ─────────────────────────────────────────────────────────────────────────────

fn cas_write_byte(word: &AtomicU32, byte_offset: u32, value: u8) -> u32 {
    let shift = (byte_offset & 3) * 8;
    let mask = 0xFFu32 << shift;
    let new_bits = (value as u32) << shift;
    let mut retries = 0u32;
    loop {
        let old = word.load(Relaxed);
        let new_val = (old & !mask) | new_bits;
        match word.compare_exchange_weak(old, new_val, AcqRel, Relaxed) {
            Ok(_) => return retries,
            Err(_) => {
                retries += 1;
                continue;
            }
        }
    }
}

fn bench_masked_cas() {
    const OPS_PER_THREAD: u64 = 1_000_000;
    const THREADS: usize = 4;

    let word = Arc::new(AtomicU32::new(0));
    let total_retries = Arc::new(AtomicU64::new(0));

    // Warmup
    for i in 0..THREADS {
        cas_write_byte(&word, i as u32, 0xAA);
    }
    word.store(0, Relaxed);

    let start = Instant::now();
    let handles: Vec<_> = (0..THREADS)
        .map(|i| {
            let w = Arc::clone(&word);
            let r = Arc::clone(&total_retries);
            std::thread::spawn(move || {
                affinity::pin_to_core(i);
                let mut local_retries = 0u64;
                for iter in 0..OPS_PER_THREAD {
                    local_retries += cas_write_byte(&w, i as u32, (iter & 0xFF) as u8) as u64;
                }
                r.fetch_add(local_retries, Relaxed);
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let elapsed = start.elapsed();

    let total_ops = THREADS as f64 * OPS_PER_THREAD as f64;
    let mean_ns = elapsed.as_nanos() as f64 / total_ops;
    let retries = total_retries.load(Relaxed) as f64;
    let retry_rate = retries / (retries + total_ops) * 100.0;

    println!(
        "\n3. Masked CAS loop ({} threads, {}M ops/thread)",
        THREADS,
        OPS_PER_THREAD / 1_000_000
    );
    println!("   Mean CAS time:     {:.0} ns", mean_ns);
    println!("   Retry rate:        {:.1}%", retry_rate);
}

use std::sync::atomic::AtomicU64;

// ─────────────────────────────────────────────────────────────────────────────
// 4. SPSC queue (bounded, lock-free, single-producer single-consumer)
// ─────────────────────────────────────────────────────────────────────────────

/// Cache-line-padded atomic index to avoid false sharing between producer
/// and consumer.
#[repr(align(64))]
struct PaddedAtomicUsize(AtomicUsize);

impl PaddedAtomicUsize {
    fn new(v: usize) -> Self {
        Self(AtomicUsize::new(v))
    }
}

struct SpscQueue {
    buffer: Box<[AtomicU32]>,
    head: PaddedAtomicUsize, // next slot to write (producer only)
    tail: PaddedAtomicUsize, // next slot to read  (consumer only)
    capacity: usize,
}

impl SpscQueue {
    fn new(capacity: usize) -> Self {
        let mut buf = Vec::with_capacity(capacity);
        for _ in 0..capacity {
            buf.push(AtomicU32::new(0));
        }
        Self {
            buffer: buf.into_boxed_slice(),
            head: PaddedAtomicUsize::new(0),
            tail: PaddedAtomicUsize::new(0),
            capacity,
        }
    }

    /// Try to push a value. Returns false if full.
    fn try_push(&self, val: u32) -> bool {
        let head = self.head.0.load(Relaxed);
        let next_head = (head + 1) % self.capacity;
        if next_head == self.tail.0.load(Acquire) {
            return false; // full
        }
        self.buffer[head].store(val, Relaxed);
        self.head.0.store(next_head, Release);
        true
    }

    /// Try to pop a value. Returns None if empty.
    fn try_pop(&self) -> Option<u32> {
        let tail = self.tail.0.load(Relaxed);
        if tail == self.head.0.load(Acquire) {
            return None; // empty
        }
        let val = self.buffer[tail].load(Relaxed);
        self.tail.0.store((tail + 1) % self.capacity, Release);
        Some(val)
    }
}

// SAFETY: single-producer single-consumer — head is only written by producer,
// tail is only written by consumer.  The AtomicU32 buffer slots plus
// Acquire/Release on head/tail ensure correct visibility.
unsafe impl Send for SpscQueue {}
unsafe impl Sync for SpscQueue {}

fn bench_spsc_queue() {
    const ELEMENTS: u64 = 10_000_000;

    // 4a. Lock-free SPSC with capacity 256 (amortises cache-line bounces).
    {
        const CAPACITY: usize = 256;
        let queue = Arc::new(SpscQueue::new(CAPACITY + 1));

        // Warmup
        {
            let q = Arc::clone(&queue);
            for i in 0..100u32 {
                while !q.try_push(i) {
                    std::hint::spin_loop();
                }
            }
            for _ in 0..100 {
                while q.try_pop().is_none() {
                    std::hint::spin_loop();
                }
            }
        }

        let queue = Arc::new(SpscQueue::new(CAPACITY + 1));
        let start = Instant::now();

        let q_prod = Arc::clone(&queue);
        let producer = std::thread::spawn(move || {
            affinity::pin_to_core(0);
            for i in 0..ELEMENTS as u32 {
                while !q_prod.try_push(i) {
                    std::hint::spin_loop();
                }
            }
        });

        let q_cons = Arc::clone(&queue);
        let consumer = std::thread::spawn(move || {
            affinity::pin_to_core(2); // different physical core
            for _ in 0..ELEMENTS {
                while q_cons.try_pop().is_none() {
                    std::hint::spin_loop();
                }
            }
        });

        producer.join().unwrap();
        consumer.join().unwrap();
        let elapsed = start.elapsed();
        let mops = ELEMENTS as f64 / elapsed.as_secs_f64() / 1e6;

        println!("\n4. SPSC queue ({}M elements)", ELEMENTS / 1_000_000);
        println!("   Lock-free (cap={}): {:.1} Mops/s", CAPACITY, mops);
    }

    // 4b. std::sync::mpsc::sync_channel(8) as a baseline comparison.
    {
        let (tx, rx) = std::sync::mpsc::sync_channel::<u32>(8);

        let start = Instant::now();
        let producer = std::thread::spawn(move || {
            affinity::pin_to_core(0);
            for i in 0..ELEMENTS as u32 {
                tx.send(i).unwrap();
            }
        });

        let consumer = std::thread::spawn(move || {
            affinity::pin_to_core(2);
            for _ in 0..ELEMENTS {
                rx.recv().unwrap();
            }
        });

        producer.join().unwrap();
        consumer.join().unwrap();
        let elapsed = start.elapsed();
        let mops = ELEMENTS as f64 / elapsed.as_secs_f64() / 1e6;

        println!("   sync_channel(8):   {:.1} Mops/s", mops);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Mutex contention cycle time
// ─────────────────────────────────────────────────────────────────────────────

fn bench_mutex_contention() {
    const OPS_PER_THREAD: u64 = 1_000_000;
    const THREADS: usize = 4;

    let counter = Arc::new(Mutex::new(0u32));

    // Warmup
    for _ in 0..1000 {
        let mut g = counter.lock().unwrap();
        *g += 1;
    }
    *counter.lock().unwrap() = 0;

    let start = Instant::now();
    let handles: Vec<_> = (0..THREADS)
        .map(|i| {
            let c = Arc::clone(&counter);
            std::thread::spawn(move || {
                affinity::pin_to_core(i);
                for _ in 0..OPS_PER_THREAD {
                    let mut g = c.lock().unwrap();
                    *g += 1;
                }
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let elapsed = start.elapsed();

    let total_ops = THREADS as f64 * OPS_PER_THREAD as f64;
    let mean_ns = elapsed.as_nanos() as f64 / total_ops;

    println!(
        "\n5. Mutex contention ({} threads, {}M ops/thread)",
        THREADS,
        OPS_PER_THREAD / 1_000_000
    );
    println!("   Mean lock cycle:   {:.0} ns", mean_ns);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Production SpinBarrier (hybrid spin+park) — parties ∈ {2,4,6,8} sweep
// 7. Production SpinBarrier — asymmetric arrival at parties=6
//
// Back the HLD §7 pre-implementation measurement gate:
//   wrk_docs/2026.04.22 - HLD - Threaded PIO Per-Block Workers V5.md §7
//
// Per-round sampling strategy: one designated "sampler" worker times its
// own `wait()` call with `Instant::now()`. Each round produces one sample
// — a u64 nanosecond count — which is pushed into a Vec. p50/p99 are read
// off the sorted vector post-hoc. At ~500 ns per round and ~25 ns per
// `Instant::now()` on Windows, the sampling overhead is ~5% on the mean
// and negligible on tail percentiles; the figure we read off is a faithful
// upper bound on the primitive's actual cost.
//
// For the asymmetric case, the "late" worker busy-waits ~2 µs past the
// others before calling `wait()` (see `busy_wait_approx`). Busy-wait over
// `thread::sleep` because Windows' default timer granularity is ~15.6 ms
// — `thread::sleep(Duration::from_micros(2))` can return 15 ms later and
// destroy the measurement. Busy-wait is deterministic at µs granularity.
//
// The sampler is an *early* arriver (not the late one) so its measured
// `wait()` includes the full park → notify_all → wake path under load.
// ─────────────────────────────────────────────────────────────────────────────

/// Per-configuration latency summary captured by the §7 gate.
struct BarrierStats {
    config: &'static str,
    parties: u32,
    mean_ns: f64,
    p50_ns: u64,
    p99_ns: u64,
    samples: usize,
}

/// Busy-wait approximately `target` long by polling `Instant::now()`.
///
/// Deterministic at µs granularity, unlike `thread::sleep` on Windows
/// which rounds up to the system timer tick (~15.6 ms by default).
#[inline]
fn busy_wait_approx(target: Duration) {
    let start = Instant::now();
    while start.elapsed() < target {
        std::hint::spin_loop();
    }
}

/// Compute percentile from a *sorted* slice of nanosecond samples.
///
/// Uses the conventional nearest-rank (round-up) definition. `samples`
/// MUST be pre-sorted ascending.
fn percentile_ns(sorted_samples: &[u64], pct: f64) -> u64 {
    if sorted_samples.is_empty() {
        return 0;
    }
    let n = sorted_samples.len();
    // Nearest-rank: rank = ceil(pct/100 * n), clamped to [1, n].
    let rank = ((pct / 100.0) * n as f64).ceil() as usize;
    let idx = rank.saturating_sub(1).min(n - 1);
    sorted_samples[idx]
}

/// Run a symmetric-arrival round-trip benchmark on the production barrier.
///
/// Spawns `parties` threads that each loop `rounds` times calling
/// `wait()`. One designated thread (id 0) samples its own `wait()`
/// latency each round. All threads arrive as fast as they can (symmetric
/// arrival) — this exercises the hot spin-release path.
fn bench_prod_barrier_symmetric(parties: u32, rounds: usize) -> BarrierStats {
    let barrier = Arc::new(ProdSpinBarrier::new(parties));

    // Warmup — matches the style of section 1 and lets the OS JIT the
    // thread-creation + condvar paths before we start timing.
    {
        let warm = Arc::new(ProdSpinBarrier::new(parties));
        let handles: Vec<_> = (0..parties)
            .map(|_| {
                let b = Arc::clone(&warm);
                std::thread::spawn(move || {
                    for _ in 0..1_000 {
                        let _ = b.wait();
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    // Sampler thread (id 0) records one u64 per round; others just wait.
    // Pre-allocated so no sample-path allocation shows up in the tail.
    let samples: Arc<Mutex<Vec<u64>>> = Arc::new(Mutex::new(Vec::with_capacity(rounds)));

    let mut handles = Vec::with_capacity(parties as usize);
    for tid in 0..parties {
        let b = Arc::clone(&barrier);
        let s = Arc::clone(&samples);
        handles.push(std::thread::spawn(move || {
            if tid == 0 {
                // Sampler — time each wait(), push the nanosecond delta.
                let mut local = Vec::with_capacity(rounds);
                for _ in 0..rounds {
                    let t0 = Instant::now();
                    let _ = b.wait();
                    local.push(t0.elapsed().as_nanos() as u64);
                }
                let mut g = s.lock().unwrap();
                *g = local;
            } else {
                // Peer — no sampling, just arrive.
                for _ in 0..rounds {
                    let _ = b.wait();
                }
            }
        }));
    }
    for h in handles {
        h.join().unwrap();
    }

    let mut samples = Arc::try_unwrap(samples)
        .ok()
        .expect("sampler arc should have only one strong ref after join")
        .into_inner()
        .unwrap();
    samples.sort_unstable();

    let total: u64 = samples.iter().sum();
    let mean_ns = total as f64 / samples.len() as f64;
    let p50_ns = percentile_ns(&samples, 50.0);
    let p99_ns = percentile_ns(&samples, 99.0);

    BarrierStats {
        config: "symmetric",
        parties,
        mean_ns,
        p50_ns,
        p99_ns,
        samples: samples.len(),
    }
}

/// Run an asymmetric-arrival round-trip benchmark on the production
/// barrier at the specified arity.
///
/// One designated "late" worker busy-waits `late_delay` each round before
/// calling `wait()`. The sampler thread is an *early* arriver — its
/// measured `wait()` latency captures the time from its own arrival
/// through park → `notify_all` wake after the late worker finally shows
/// up. This mirrors OneROM's hot-path pattern (PIO2 finishes well after
/// PIO0/PIO1/core0/coord).
fn bench_prod_barrier_asymmetric(
    parties: u32,
    rounds: usize,
    late_delay: Duration,
) -> BarrierStats {
    assert!(
        parties >= 3,
        "asymmetric case needs ≥ sampler + late + peer"
    );
    let barrier = Arc::new(ProdSpinBarrier::new(parties));

    // Warmup — run a symmetric warmup (no staggering) just to prime
    // thread-local caches and the OS scheduler.
    {
        let warm = Arc::new(ProdSpinBarrier::new(parties));
        let handles: Vec<_> = (0..parties)
            .map(|_| {
                let b = Arc::clone(&warm);
                std::thread::spawn(move || {
                    for _ in 0..1_000 {
                        let _ = b.wait();
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    let samples: Arc<Mutex<Vec<u64>>> = Arc::new(Mutex::new(Vec::with_capacity(rounds)));

    // Thread-id convention:
    //   tid == 0          : sampler (early arriver, times its own wait())
    //   tid == parties-1  : late arriver (busy-waits `late_delay` first)
    //   others            : plain early arrivers
    let late_id = parties - 1;

    let mut handles = Vec::with_capacity(parties as usize);
    for tid in 0..parties {
        let b = Arc::clone(&barrier);
        let s = Arc::clone(&samples);
        handles.push(std::thread::spawn(move || {
            if tid == 0 {
                let mut local = Vec::with_capacity(rounds);
                for _ in 0..rounds {
                    let t0 = Instant::now();
                    let _ = b.wait();
                    local.push(t0.elapsed().as_nanos() as u64);
                }
                let mut g = s.lock().unwrap();
                *g = local;
            } else if tid == late_id {
                for _ in 0..rounds {
                    busy_wait_approx(late_delay);
                    let _ = b.wait();
                }
            } else {
                for _ in 0..rounds {
                    let _ = b.wait();
                }
            }
        }));
    }
    for h in handles {
        h.join().unwrap();
    }

    let mut samples = Arc::try_unwrap(samples)
        .ok()
        .expect("sampler arc should have only one strong ref after join")
        .into_inner()
        .unwrap();
    samples.sort_unstable();

    let total: u64 = samples.iter().sum();
    let mean_ns = total as f64 / samples.len() as f64;
    let p50_ns = percentile_ns(&samples, 50.0);
    let p99_ns = percentile_ns(&samples, 99.0);

    BarrierStats {
        config: "asymmetric-2us",
        parties,
        mean_ns,
        p50_ns,
        p99_ns,
        samples: samples.len(),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. Parameterised-SPIN_BUDGET barrier sweep (6-way asymmetric 2µs)
//
// The production `SpinBarrier` in `mdrp2350::threaded::barrier` has a
// hardcoded `const SPIN_BUDGET: u32 = 128`. Section 7 showed the 6-way
// asymmetric-2µs case exhausts that budget in five of six workers and
// pays the park → `notify_all` wake cost. To find the knee where everyone
// stays on the spin path through a 2 µs stagger, we clone the production
// barrier's essential logic here with a runtime-parameterised budget.
//
// The clone mirrors the production barrier *exactly* on the hot path:
//
//   - Same Release/Acquire pairing on `generation` and `count` so the
//     last-arriver's reset (count=0; gen+=1) publishes "ahead of" the
//     early arrivers' spin-loop gen load.
//   - Same `park_mu` held around the generation store on the last-
//     arriver branch so any worker already inside `park_cv.wait` linearises
//     on old-gen or new-gen sides — never in a window that could miss
//     the broadcast.
//   - Same park-on-budget-exhaust path: re-check under `park_mu` and
//     `wait()` on the condvar.
//   - Same `notify_all()` outside the lock.
//
// The poison path is omitted (bench doesn't panic; no coordinator). The
// `poisoned` load inside the spin loop is also removed since there's
// nothing to poison it from — this shaves one atomic load per spin
// iteration but does not change the timing the sweep is trying to
// measure (budget-exhaust vs stay-on-spin). Everything else is a
// line-for-line mirror of `SpinBarrier::wait`.
// ─────────────────────────────────────────────────────────────────────────────

/// Local clone of the production `SpinBarrier` with a runtime-parameterised
/// spin budget. See module-level §8 comment for semantics guarantee.
struct ParamSpinBarrier {
    generation: AtomicU32,
    count: AtomicU32,
    parties: u32,
    spin_budget: u32,
    park_mu: Mutex<()>,
    park_cv: Condvar,
}

impl ParamSpinBarrier {
    fn new(parties: u32, spin_budget: u32) -> Self {
        assert!(parties >= 2);
        Self {
            generation: AtomicU32::new(0),
            count: AtomicU32::new(0),
            parties,
            spin_budget,
            park_mu: Mutex::new(()),
            park_cv: Condvar::new(),
        }
    }

    fn wait(&self) {
        let cur_gen = self.generation.load(Acquire);
        let n = self.count.fetch_add(1, AcqRel) + 1;
        if n == self.parties {
            // Last arriver — bump generation under park_mu so any sleeper
            // linearises cleanly with its predicate re-check.
            {
                let _g = self.park_mu.lock().unwrap();
                self.count.store(0, Relaxed);
                self.generation.store(cur_gen.wrapping_add(1), Release);
            }
            self.park_cv.notify_all();
            return;
        }

        // Early arriver — spin budget.
        for _ in 0..self.spin_budget {
            if self.generation.load(Acquire) != cur_gen {
                return;
            }
            std::hint::spin_loop();
        }

        // Budget exhausted — park on condvar.
        let mut g = self.park_mu.lock().unwrap();
        while self.generation.load(Acquire) == cur_gen {
            g = self.park_cv.wait(g).unwrap();
        }
    }
}

/// One row of the §8 sweep — a single SPIN_BUDGET configuration under the
/// 6-way asymmetric-2µs pattern.
struct SweepStats {
    spin_budget: u32,
    mean_ns: f64,
    p50_ns: u64,
    p99_ns: u64,
    samples: usize,
}

/// Asymmetric-arrival benchmark on `ParamSpinBarrier`. Same thread-id
/// convention as `bench_prod_barrier_asymmetric`: tid 0 samples, tid
/// `parties-1` busy-waits `late_delay`, others are plain early arrivers.
fn bench_param_barrier_asymmetric(
    parties: u32,
    rounds: usize,
    late_delay: Duration,
    spin_budget: u32,
) -> SweepStats {
    assert!(
        parties >= 3,
        "asymmetric case needs ≥ sampler + late + peer"
    );
    let barrier = Arc::new(ParamSpinBarrier::new(parties, spin_budget));

    // Warmup — prime OS scheduler and caches.
    {
        let warm = Arc::new(ParamSpinBarrier::new(parties, spin_budget));
        let handles: Vec<_> = (0..parties)
            .map(|_| {
                let b = Arc::clone(&warm);
                std::thread::spawn(move || {
                    for _ in 0..1_000 {
                        b.wait();
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    let samples: Arc<Mutex<Vec<u64>>> = Arc::new(Mutex::new(Vec::with_capacity(rounds)));
    let late_id = parties - 1;

    let mut handles = Vec::with_capacity(parties as usize);
    for tid in 0..parties {
        let b = Arc::clone(&barrier);
        let s = Arc::clone(&samples);
        handles.push(std::thread::spawn(move || {
            if tid == 0 {
                let mut local = Vec::with_capacity(rounds);
                for _ in 0..rounds {
                    let t0 = Instant::now();
                    b.wait();
                    local.push(t0.elapsed().as_nanos() as u64);
                }
                let mut g = s.lock().unwrap();
                *g = local;
            } else if tid == late_id {
                for _ in 0..rounds {
                    busy_wait_approx(late_delay);
                    b.wait();
                }
            } else {
                for _ in 0..rounds {
                    b.wait();
                }
            }
        }));
    }
    for h in handles {
        h.join().unwrap();
    }

    let mut samples = Arc::try_unwrap(samples)
        .ok()
        .expect("sampler arc should have only one strong ref after join")
        .into_inner()
        .unwrap();
    samples.sort_unstable();

    let total: u64 = samples.iter().sum();
    let mean_ns = total as f64 / samples.len() as f64;
    let p50_ns = percentile_ns(&samples, 50.0);
    let p99_ns = percentile_ns(&samples, 99.0);

    SweepStats {
        spin_budget,
        mean_ns,
        p50_ns,
        p99_ns,
        samples: samples.len(),
    }
}

fn print_sweep_table(rows: &[SweepStats]) {
    println!(
        "   {:>11} {:>10} {:>10} {:>10} {:>10}",
        "spin_budget", "mean_ns", "p50_ns", "p99_ns", "samples"
    );
    println!("   {}", "-".repeat(11 + 1 + 10 + 1 + 10 + 1 + 10 + 1 + 10));
    for r in rows {
        println!(
            "   {:>11} {:>10.0} {:>10} {:>10} {:>10}",
            r.spin_budget, r.mean_ns, r.p50_ns, r.p99_ns, r.samples
        );
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. Late-arriver tail sweep (6-way asymmetric 2µs)
//
// §§7-8 sample an *early* arriver's `wait()` — that's `stagger +
// release_propagation`, which is useful for understanding the parked-
// waiter wake cost but over-reports what actually bounds round-trip
// throughput. The ThreadedEmulator can't end a quantum until the slowest
// worker's `wait()` returns, so what we need is the *late* arriver's wait
// tail: the interval from when the last-arriving thread calls `wait()` to
// when that same thread's `wait()` returns.
//
// For the late arriver this is pure release propagation. It never parks
// on its own side (it's the last arriver — it takes the `n == parties`
// branch, bumps the generation, and returns immediately). Its cost is
// dominated by the `notify_all()` kernel call that has to wake up to
// `parties-1` parked threads before `wait()` can return — so at b=128
// (everyone parked), we pay `notify_all` on 5 threads; at b=1024+
// (nobody parks), notify_all is a near-free no-op path.
//
// Thread-id convention (mirrors §8 but swaps sampler ↔ late):
//   tid == 0            : late arriver / sampler — busy-waits `late_delay`,
//                         then times its own `wait()`.
//   tid == 1..parties-1 : early arrivers, no sampling, no stagger.
//
// Everything else (warmup, percentile math, barrier, round count) mirrors
// §8 exactly so the two sweeps are comparable side-by-side.
// ─────────────────────────────────────────────────────────────────────────────

/// Late-arriver asymmetric benchmark on `ParamSpinBarrier`. The sampled
/// thread is the *late* arriver — it busy-waits `late_delay` then times
/// its own `wait()` call. That timing window is pure release propagation
/// (no stagger inside it, and this thread never parks).
fn bench_param_barrier_late_tail(
    parties: u32,
    rounds: usize,
    late_delay: Duration,
    spin_budget: u32,
) -> SweepStats {
    assert!(parties >= 3, "late-tail case needs ≥ late + 2 peers");
    let barrier = Arc::new(ParamSpinBarrier::new(parties, spin_budget));

    // Warmup — prime OS scheduler and caches (same shape as §8).
    {
        let warm = Arc::new(ParamSpinBarrier::new(parties, spin_budget));
        let handles: Vec<_> = (0..parties)
            .map(|_| {
                let b = Arc::clone(&warm);
                std::thread::spawn(move || {
                    for _ in 0..1_000 {
                        b.wait();
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
    }

    let samples: Arc<Mutex<Vec<u64>>> = Arc::new(Mutex::new(Vec::with_capacity(rounds)));

    let mut handles = Vec::with_capacity(parties as usize);
    for tid in 0..parties {
        let b = Arc::clone(&barrier);
        let s = Arc::clone(&samples);
        handles.push(std::thread::spawn(move || {
            if tid == 0 {
                // Late arriver *and* sampler. Stagger happens OUTSIDE the
                // timed window, so the sample is pure release propagation.
                let mut local = Vec::with_capacity(rounds);
                for _ in 0..rounds {
                    busy_wait_approx(late_delay);
                    let t0 = Instant::now();
                    b.wait();
                    local.push(t0.elapsed().as_nanos() as u64);
                }
                let mut g = s.lock().unwrap();
                *g = local;
            } else {
                // Early arrivers — no stagger, no sampling.
                for _ in 0..rounds {
                    b.wait();
                }
            }
        }));
    }
    for h in handles {
        h.join().unwrap();
    }

    let mut samples = Arc::try_unwrap(samples)
        .ok()
        .expect("sampler arc should have only one strong ref after join")
        .into_inner()
        .unwrap();
    samples.sort_unstable();

    let total: u64 = samples.iter().sum();
    let mean_ns = total as f64 / samples.len() as f64;
    let p50_ns = percentile_ns(&samples, 50.0);
    let p99_ns = percentile_ns(&samples, 99.0);

    SweepStats {
        spin_budget,
        mean_ns,
        p50_ns,
        p99_ns,
        samples: samples.len(),
    }
}

fn print_barrier_stats_table(
    all: &[BarrierStats],
    sweep_early: &[SweepStats],
    sweep_late: &[SweepStats],
) {
    println!();
    println!("=== Production SpinBarrier sweep (HLD §7 gate) ===");
    println!(
        "{:<22} {:>7} {:>7} {:>10} {:>9} {:>10} {:>10}",
        "config", "parties", "arriver", "mean_ns", "p50_ns", "p99_ns", "samples"
    );
    println!(
        "{}",
        "-".repeat(22 + 1 + 7 + 1 + 7 + 1 + 10 + 1 + 9 + 1 + 10 + 1 + 10)
    );
    for s in all {
        // §§6-7 all sample an early arriver.
        println!(
            "{:<22} {:>7} {:>7} {:>10.0} {:>9} {:>10} {:>10}",
            s.config, s.parties, "early", s.mean_ns, s.p50_ns, s.p99_ns, s.samples
        );
    }
    // Append §§8-9 sweep rows so the gate decision is visible in one place.
    // `parties` is always 6 for the sweeps (the asymmetric-2µs case).
    // Print each budget's early row followed immediately by its late row
    // so the two are directly comparable. The two sweeps may cover
    // slightly different budget sets (§8 includes 8192 for the historical
    // knee search; §9 stops at 4096 per the HLD gate's target range), so
    // print the union keyed by budget.
    let mut budgets: Vec<u32> = sweep_early
        .iter()
        .map(|r| r.spin_budget)
        .chain(sweep_late.iter().map(|r| r.spin_budget))
        .collect();
    budgets.sort_unstable();
    budgets.dedup();
    for b in budgets {
        let label = format!("sweep-b={}", b);
        if let Some(e) = sweep_early.iter().find(|r| r.spin_budget == b) {
            println!(
                "{:<22} {:>7} {:>7} {:>10.0} {:>9} {:>10} {:>10}",
                label, 6u32, "early", e.mean_ns, e.p50_ns, e.p99_ns, e.samples
            );
        }
        if let Some(l) = sweep_late.iter().find(|r| r.spin_budget == b) {
            println!(
                "{:<22} {:>7} {:>7} {:>10.0} {:>9} {:>10} {:>10}",
                label, 6u32, "late", l.mean_ns, l.p50_ns, l.p99_ns, l.samples
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Go / No-Go summary
// ─────────────────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Threading Micro-Prototype (Phase 0c) ===");
    println!();

    bench_spin_barrier();
    let barrier_ns = {
        // Re-run a short measurement just for the summary value.
        // (We already printed the full result above; grab a quick number.)
        const PARTIES: u32 = 4;
        const ROUNDS: u32 = 100_000;
        let barrier = Arc::new(SpinBarrier::new(PARTIES));
        let start = Instant::now();
        let handles: Vec<_> = (0..PARTIES)
            .map(|i| {
                let b = Arc::clone(&barrier);
                std::thread::spawn(move || {
                    affinity::pin_to_core(i as usize);
                    for _ in 0..ROUNDS {
                        b.wait();
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
        start.elapsed().as_nanos() as f64 / ROUNDS as f64
    };

    bench_atomic_throughput();
    let contended_mops = {
        // Quick re-measurement for summary.
        const OPS: u64 = 1_000_000;
        let shared = Arc::new(AtomicU32::new(0));
        let start = Instant::now();
        let handles: Vec<_> = (0..2)
            .map(|i| {
                let s = Arc::clone(&shared);
                std::thread::spawn(move || {
                    affinity::pin_to_core(i);
                    for _ in 0..OPS {
                        s.fetch_add(1, Relaxed);
                    }
                })
            })
            .collect();
        for h in handles {
            h.join().unwrap();
        }
        2.0 * OPS as f64 / start.elapsed().as_secs_f64() / 1e6
    };

    bench_masked_cas();
    bench_spsc_queue();
    bench_mutex_contention();

    // --- Sections 6 & 7: production SpinBarrier sweep + asymmetric case ---
    //
    // Sample count: 100 000 rounds per configuration. At ~500 ns/round
    // symmetric and ~a few µs/round asymmetric, total wall time for the
    // five configurations below is on the order of a few minutes — well
    // within the HLD §7 "~20 min bench work" budget alongside the
    // `contention` baseline capture. p99 from 100 000 samples is the
    // 1 000th-worst sample, stable enough for the §7 gate decision.
    const GATE_ROUNDS: usize = 100_000;

    println!("\n6. Production SpinBarrier — symmetric arrival sweep");
    let mut results: Vec<BarrierStats> = Vec::new();
    for &parties in &[2u32, 4, 6, 8] {
        let stats = bench_prod_barrier_symmetric(parties, GATE_ROUNDS);
        println!(
            "   parties={}  mean={:.0} ns  p50={} ns  p99={} ns  ({} samples)",
            stats.parties, stats.mean_ns, stats.p50_ns, stats.p99_ns, stats.samples
        );
        results.push(stats);
    }

    println!("\n7. Production SpinBarrier — asymmetric arrival (parties=6, late=2µs)");
    let asym = bench_prod_barrier_asymmetric(6, GATE_ROUNDS, Duration::from_micros(2));
    println!(
        "   parties={}  mean={:.0} ns  p50={} ns  p99={} ns  ({} samples)",
        asym.parties, asym.mean_ns, asym.p50_ns, asym.p99_ns, asym.samples
    );
    results.push(asym);

    // --- Section 8: SPIN_BUDGET sweep under the same 6-way asymmetric-2µs
    // pattern, on a local parameterised-budget clone of the production
    // barrier. Goal: locate the knee where early arrivers stay on the
    // spin path through the 2 µs stagger and nobody parks.
    println!(
        "\n8. Parameterised SpinBarrier SPIN_BUDGET sweep (6-way asymmetric 2µs, EARLY sampler)"
    );
    let mut sweep_early: Vec<SweepStats> = Vec::new();
    for &budget in &[128u32, 256, 512, 1024, 2048, 4096, 8192] {
        let row = bench_param_barrier_asymmetric(6, GATE_ROUNDS, Duration::from_micros(2), budget);
        sweep_early.push(row);
    }
    print_sweep_table(&sweep_early);

    // --- Section 9: late-arriver tail sweep. Same 6-way asymmetric-2µs
    // pattern and SPIN_BUDGET axis as §8, but the sampled thread is now
    // the late arriver. Its timed window is pure release propagation —
    // no stagger, no park on its side — which is the metric that
    // actually bounds per-quantum throughput in the ThreadedEmulator.
    //
    // Expected shape: at b=128 every peer parks, so the late arriver's
    // wait() pays notify_all-on-5-parked (~1-3 µs mean). At b=1024+ no
    // peer has parked — they're all on the spin path waiting for the
    // generation store — and notify_all is essentially free.
    println!(
        "\n9. Parameterised SpinBarrier SPIN_BUDGET sweep (6-way asymmetric 2µs, LATE sampler)"
    );
    let mut sweep_late: Vec<SweepStats> = Vec::new();
    for &budget in &[128u32, 256, 512, 1024, 2048, 4096] {
        let row = bench_param_barrier_late_tail(6, GATE_ROUNDS, Duration::from_micros(2), budget);
        sweep_late.push(row);
    }
    print_sweep_table(&sweep_late);

    print_barrier_stats_table(&results, &sweep_early, &sweep_late);

    // --- Go / No-Go ---
    println!("\n=== Go/No-Go ===");

    let barrier_pass = barrier_ns < 500.0;
    println!(
        "SpinBarrier:  {:.0} ns  [{}] (threshold: <500 ns)",
        barrier_ns,
        if barrier_pass { "PASS" } else { "FAIL" }
    );

    let atomic_pass = contended_mops > 50.0;
    println!(
        "AtomicU32:    {:.1} Mops/s [{}] (threshold: >50 Mops/s contended)",
        contended_mops,
        if atomic_pass { "PASS" } else { "FAIL" }
    );
}

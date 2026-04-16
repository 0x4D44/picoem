//! Threading Micro-Prototype (Phase 0c)
//!
//! Standalone benchmark measuring the cost of threading primitives that the
//! threaded dual-core emulation design depends on.  Results feed the go/no-go
//! decision before committing to a full threaded architecture.
//!
//! Measures:
//!   1. SpinBarrier 4-way round-trip
//!   2. AtomicU32 contended vs uncontended throughput
//!   3. Masked CAS loop cost (byte-granularity writes on word atomics)
//!   4. SPSC bounded-queue throughput
//!   5. Mutex contention cycle time

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering::*};
use std::sync::{Arc, Mutex};
use std::time::Instant;

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

        println!(
            "\n4. SPSC queue ({}M elements)",
            ELEMENTS / 1_000_000
        );
        println!(
            "   Lock-free (cap={}): {:.1} Mops/s",
            CAPACITY, mops
        );
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

//! Lock-free single-producer single-consumer ring buffer for u32 payloads.
//!
//! Non-blocking `try_push` / `try_pop` (no spinning on full/empty). Used by
//! ThreadedSio (capacity 8) and ThreadedPio (capacity 4).
//!
//! See `wrk_docs/2026.04.17 - LLD - Threaded Dual-Core Phase 2 V4.md` §1.
//!
//! ## Memory ordering
//!
//! - Producer loads own `head` (Relaxed), peer's `tail` (Acquire); writes
//!   buffer (Relaxed), Release-stores head.
//! - Consumer loads own `tail` (Relaxed), peer's `head` (Acquire); reads
//!   buffer (Relaxed), Release-stores tail.
//! - `len()`: Relaxed both — approximate count for status register.
//! - `clear()`: Relaxed — only called during coordinator phase or reset
//!   (no concurrent access).
//!
//! Fullness check uses unsigned wrap-around arithmetic, correct for all
//! u32 values because capacity is a power of 2 in [2, 2^31].
//!
//! The release/acquire edge on `head` publishes the producer's payload
//! store before the consumer reads it, so the queue is valid on weakly
//! ordered targets as well as x86 TSO.
//!
//! ## Cross-chip reuse
//!
//! This type is chip-agnostic and may move to `picoem-common` in Phase 3
//! when the RP2040 threaded path lands. No dependency on rp2350_emu-specific
//! types.

use std::sync::atomic::{AtomicU32, Ordering::*};

pub struct SpscQueue {
    buffer: Box<[AtomicU32]>,
    head: AtomicU32, // next write position (producer owns)
    tail: AtomicU32, // next read position (consumer owns)
    capacity: u32,   // power of 2
    mask: u32,       // capacity - 1
}

impl SpscQueue {
    /// Create a new SPSC queue with the given capacity.
    ///
    /// `capacity` must be a power of two and at least 2.
    pub fn new(capacity: u32) -> Self {
        assert!(capacity.is_power_of_two() && capacity >= 2);
        let mut buf = Vec::with_capacity(capacity as usize);
        for _ in 0..capacity {
            buf.push(AtomicU32::new(0));
        }
        Self {
            buffer: buf.into_boxed_slice(),
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            capacity,
            mask: capacity - 1,
        }
    }

    /// Attempt to push a value. Returns true on success, false if full.
    pub fn try_push(&self, val: u32) -> bool {
        let head = self.head.load(Relaxed);
        let tail = self.tail.load(Acquire);
        if head.wrapping_sub(tail) >= self.capacity {
            return false;
        }
        self.buffer[(head & self.mask) as usize].store(val, Relaxed);
        self.head.store(head.wrapping_add(1), Release);
        true
    }

    /// Attempt to pop a value. Returns `Some(val)` on success, `None` if empty.
    pub fn try_pop(&self) -> Option<u32> {
        let tail = self.tail.load(Relaxed);
        let head = self.head.load(Acquire);
        if tail == head {
            return None;
        }
        let val = self.buffer[(tail & self.mask) as usize].load(Relaxed);
        self.tail.store(tail.wrapping_add(1), Release);
        Some(val)
    }

    /// Approximate queue occupancy. Relaxed loads — value may be stale.
    pub fn len(&self) -> u32 {
        let head = self.head.load(Relaxed);
        let tail = self.tail.load(Relaxed);
        head.wrapping_sub(tail)
    }

    /// True if the queue appears empty. Relaxed — may be stale.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// True if the queue appears full. Relaxed — may be stale.
    pub fn is_full(&self) -> bool {
        self.len() >= self.capacity
    }

    /// Reset to empty. Only safe when no concurrent push/pop (coordinator
    /// phase or reset).
    pub fn clear(&self) {
        self.head.store(0, Relaxed);
        self.tail.store(0, Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;

    #[test]
    fn push_pop_roundtrip() {
        let q = SpscQueue::new(8);
        for i in 0..8u32 {
            assert!(q.try_push(i), "push {i} should succeed");
        }
        for i in 0..8u32 {
            assert_eq!(q.try_pop(), Some(i), "pop index {i}");
        }
        assert!(q.is_empty());
    }

    #[test]
    fn empty_pop_returns_none() {
        let q = SpscQueue::new(4);
        assert_eq!(q.try_pop(), None);
    }

    #[test]
    fn full_push_returns_false() {
        let q = SpscQueue::new(4);
        for i in 0..4u32 {
            assert!(q.try_push(i));
        }
        assert!(!q.try_push(99), "push into full queue must fail");
        assert_eq!(q.len(), 4);
    }

    #[test]
    fn wraparound() {
        // cap=8, push/pop 100 items so head/tail wrap past capacity many times.
        let q = SpscQueue::new(8);
        for i in 0..100u32 {
            assert!(q.try_push(i), "push {i}");
            assert_eq!(q.try_pop(), Some(i), "pop {i}");
        }
        assert!(q.is_empty());
    }

    #[test]
    fn len_tracks_occupancy() {
        let q = SpscQueue::new(8);
        assert_eq!(q.len(), 0);
        assert!(q.is_empty());
        assert!(!q.is_full());

        q.try_push(10);
        q.try_push(20);
        q.try_push(30);
        assert_eq!(q.len(), 3);
        assert!(!q.is_empty());
        assert!(!q.is_full());
    }

    #[test]
    fn clear_resets_queue() {
        let q = SpscQueue::new(4);
        q.try_push(1);
        q.try_push(2);
        q.try_push(3);
        assert_eq!(q.len(), 3);

        q.clear();
        assert_eq!(q.len(), 0);
        assert!(q.is_empty());
        assert_eq!(q.try_pop(), None);

        // Reusable after clear.
        assert!(q.try_push(42));
        assert_eq!(q.try_pop(), Some(42));
    }

    #[test]
    fn concurrent_push_pop() {
        const N: u32 = 100_000;
        let q = Arc::new(SpscQueue::new(1024));
        let producer_q = Arc::clone(&q);
        let consumer_q = Arc::clone(&q);

        let producer = thread::spawn(move || {
            let mut i = 0u32;
            while i < N {
                if producer_q.try_push(i) {
                    i += 1;
                } else {
                    std::hint::spin_loop();
                }
            }
        });

        let consumer = thread::spawn(move || {
            let mut received = Vec::with_capacity(N as usize);
            while received.len() < N as usize {
                match consumer_q.try_pop() {
                    Some(v) => received.push(v),
                    None => std::hint::spin_loop(),
                }
            }
            received
        });

        producer.join().expect("producer thread");
        let received = consumer.join().expect("consumer thread");

        assert_eq!(received.len(), N as usize);
        for (idx, v) in received.iter().enumerate() {
            assert_eq!(*v, idx as u32, "value at index {idx} must equal index");
        }
    }
}

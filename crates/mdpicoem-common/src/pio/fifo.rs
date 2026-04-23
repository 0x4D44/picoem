/// PIO FIFO with configurable depth (4 normal, 8 joined).
pub struct PioFifo {
    buf: [u32; 8],
    head: u8,
    tail: u8,
    count: u8,
    depth: u8,
    /// Diagnostic counter — number of `push` calls that successfully
    /// stored a value (FIFO had room). Pure observation, never read by
    /// FIFO logic. Wraps on `u64` overflow (in practice unreachable
    /// within a single emulator session). Used by the PicoGUS bring-up
    /// harness to confirm the DMA→TXF path is actually landing bytes.
    pub push_success: u64,
    /// Diagnostic counter — number of `push` calls that found the FIFO
    /// full and silently dropped the value. Paired with `push_success`;
    /// non-zero in production code indicates either DMA outpacing the
    /// SM (back-pressure intended) or a missing DREQ throttle.
    pub push_drop: u64,
}

impl PioFifo {
    pub fn new(depth: u8) -> Self {
        Self {
            buf: [0; 8],
            head: 0,
            tail: 0,
            count: 0,
            depth,
            push_success: 0,
            push_drop: 0,
        }
    }

    /// Push a value. Returns false if the FIFO is full (value dropped).
    pub fn push(&mut self, val: u32) -> bool {
        if self.count >= self.depth {
            self.push_drop = self.push_drop.wrapping_add(1);
            return false;
        }
        self.buf[self.tail as usize] = val;
        self.tail = (self.tail + 1) % self.depth;
        self.count += 1;
        self.push_success = self.push_success.wrapping_add(1);
        true
    }

    /// Pop a value. Returns None if the FIFO is empty.
    pub fn pop(&mut self) -> Option<u32> {
        if self.count == 0 {
            return None;
        }
        let val = self.buf[self.head as usize];
        self.head = (self.head + 1) % self.depth;
        self.count -= 1;
        Some(val)
    }

    pub fn is_full(&self) -> bool {
        self.depth > 0 && self.count >= self.depth
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn level(&self) -> u8 {
        self.count
    }

    pub fn flush(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.count = 0;
    }

    pub fn set_depth(&mut self, d: u8) {
        self.depth = d;
        self.flush();
    }
}

#[cfg(test)]
mod tests {
    //! Push success / drop counter tests.
    //!
    //! Wired up before the counter fields existed (TDD RED phase) — the
    //! original failures were on `assert_eq!(fifo.push_success, …)` /
    //! `assert_eq!(fifo.push_drop, …)` because the fields weren't
    //! defined. They go GREEN once `push_success` / `push_drop` land in
    //! `PioFifo` and `push` increments them.
    use super::*;
    #[test]
    fn counters_initialize_to_zero() {
        let fifo = PioFifo::new(4);
        assert_eq!(fifo.push_success, 0);
        assert_eq!(fifo.push_drop, 0);
    }

    #[test]
    fn push_success_count_bumps_on_successful_push() {
        let mut fifo = PioFifo::new(4);
        assert!(fifo.push(0xAA));
        assert!(fifo.push(0xBB));
        assert!(fifo.push(0xCC));
        assert_eq!(fifo.push_success, 3);
        assert_eq!(fifo.push_drop, 0);
    }

    #[test]
    fn push_drop_count_bumps_when_full() {
        let mut fifo = PioFifo::new(4);
        // Fill it.
        for v in 0..4u32 {
            assert!(fifo.push(v));
        }
        assert_eq!(fifo.push_success, 4);
        assert_eq!(fifo.push_drop, 0);
        // 5th push drops.
        assert!(!fifo.push(0xDEAD));
        assert_eq!(fifo.push_success, 4);
        assert_eq!(fifo.push_drop, 1);
        // Three more pushes while full keep dropping.
        for _ in 0..3 {
            assert!(!fifo.push(0xBEEF));
        }
        assert_eq!(fifo.push_success, 4);
        assert_eq!(fifo.push_drop, 4);
    }

    #[test]
    fn counters_survive_pop_and_push_mix() {
        let mut fifo = PioFifo::new(4);
        for v in 0..4u32 {
            assert!(fifo.push(v));
        }
        assert_eq!(fifo.pop(), Some(0));
        assert_eq!(fifo.pop(), Some(1));
        // Two free slots — two more pushes succeed.
        assert!(fifo.push(0x10));
        assert!(fifo.push(0x11));
        assert_eq!(fifo.push_success, 6);
        assert_eq!(fifo.push_drop, 0);
        // Now full again — 3rd push drops.
        assert!(!fifo.push(0x12));
        assert_eq!(fifo.push_success, 6);
        assert_eq!(fifo.push_drop, 1);
    }

    /// `is_full` returns false for a depth=0 FIFO (the "joined-away" side
    /// of a FIFO-join pair — see `PioBlock::apply_fifo_join`). Covers the
    /// short-circuit left arm of `depth > 0 && count >= depth` at line 59.
    #[test]
    fn is_full_is_false_on_depth_zero_fifo() {
        let mut fifo = PioFifo::new(0);
        assert!(!fifo.is_full(), "depth=0 FIFO must not report full");
        assert!(fifo.is_empty(), "depth=0 FIFO is trivially empty");
        // And push on a depth=0 FIFO always drops (covers line 36 drop arm).
        assert!(!fifo.push(0xAA));
        assert_eq!(fifo.push_drop, 1);
        assert_eq!(fifo.push_success, 0);
        // pop returns None — covers the count==0 guard of `pop`.
        assert_eq!(fifo.pop(), None);
    }

    /// Cover `set_depth` and `flush`: after expanding depth the FIFO is
    /// usable; after `flush` it drops buffered entries and reports empty.
    #[test]
    fn set_depth_flushes_and_allows_reuse() {
        let mut fifo = PioFifo::new(4);
        assert!(fifo.push(1));
        assert!(fifo.push(2));
        assert_eq!(fifo.level(), 2);
        fifo.flush();
        assert!(fifo.is_empty());
        assert_eq!(fifo.level(), 0);
        // Grow to 8 (FJOIN case) — push 5 values succeeds.
        fifo.set_depth(8);
        for v in 0..5u32 {
            assert!(fifo.push(v));
        }
        assert_eq!(fifo.level(), 5);
    }
}

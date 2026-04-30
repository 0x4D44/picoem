//! 8-entry circular FIFO for inter-processor communication.
//!
//! Same shape on RP2040 (2-core FIFO pair at `SIO_BASE + 0x050`) and
//! RP2350 (same pair, plus the coprocessor bridge uses a second 8-entry
//! FIFO in `sio/mod.rs`). Lifted out of `mdrp2350::sio::mod` verbatim.

/// 8-entry circular FIFO for inter-processor communication.
pub struct Fifo {
    buf: [u32; 8],
    head: u8,
    tail: u8,
    count: u8,
}

impl Fifo {
    pub fn new() -> Self {
        Self {
            buf: [0; 8],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Push a value. Returns false if the FIFO is full (value dropped).
    pub fn push(&mut self, val: u32) -> bool {
        if self.count >= 8 {
            return false;
        }
        self.buf[self.tail as usize] = val;
        self.tail = (self.tail + 1) % 8;
        self.count += 1;
        true
    }

    /// Pop a value. Returns None if the FIFO is empty.
    pub fn pop(&mut self) -> Option<u32> {
        if self.count == 0 {
            return None;
        }
        let val = self.buf[self.head as usize];
        self.head = (self.head + 1) % 8;
        self.count -= 1;
        Some(val)
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn is_full(&self) -> bool {
        self.count >= 8
    }

    /// Non-consuming snapshot of the FIFO contents in head → tail order.
    ///
    /// Used by `mdrp2350::threaded::ThreadedSio::seed` to copy the
    /// single-threaded inter-core FIFO state into the SPSC ring without
    /// mutating the source.
    pub fn snapshot(&self) -> Vec<u32> {
        let mut out = Vec::with_capacity(self.count as usize);
        let mut idx = self.head as usize;
        for _ in 0..self.count {
            out.push(self.buf[idx]);
            idx = (idx + 1) % 8;
        }
        out
    }
}

impl Default for Fifo {
    fn default() -> Self {
        Self::new()
    }
}

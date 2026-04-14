/// PIO FIFO with configurable depth (4 normal, 8 joined).
pub struct PioFifo {
    buf: [u32; 8],
    head: u8,
    tail: u8,
    count: u8,
    depth: u8,
}

impl PioFifo {
    pub fn new(depth: u8) -> Self {
        Self {
            buf: [0; 8],
            head: 0,
            tail: 0,
            count: 0,
            depth,
        }
    }

    /// Push a value. Returns false if the FIFO is full (value dropped).
    pub fn push(&mut self, val: u32) -> bool {
        if self.count >= self.depth {
            return false;
        }
        self.buf[self.tail as usize] = val;
        self.tail = (self.tail + 1) % self.depth;
        self.count += 1;
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

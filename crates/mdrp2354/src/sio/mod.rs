/// Per-core integer divider state (§2.4).
#[derive(Default, Clone, Copy)]
struct Divider {
    dividend: u32,
    divisor: u32,
    quotient: u32,
    remainder: u32,
    signed: bool,
    dirty: bool,
    reads_pending: u8,
}

/// 8-entry circular FIFO for inter-processor communication.
struct Fifo {
    buf: [u32; 8],
    head: u8,
    tail: u8,
    count: u8,
}

impl Fifo {
    fn new() -> Self {
        Self {
            buf: [0; 8],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Push a value. Returns false if the FIFO is full (value dropped).
    fn push(&mut self, val: u32) -> bool {
        if self.count >= 8 {
            return false;
        }
        self.buf[self.tail as usize] = val;
        self.tail = (self.tail + 1) % 8;
        self.count += 1;
        true
    }

    /// Pop a value. Returns None if the FIFO is empty.
    fn pop(&mut self) -> Option<u32> {
        if self.count == 0 {
            return None;
        }
        let val = self.buf[self.head as usize];
        self.head = (self.head + 1) % 8;
        self.count -= 1;
        Some(val)
    }

    fn is_empty(&self) -> bool {
        self.count == 0
    }

    fn is_full(&self) -> bool {
        self.count >= 8
    }

    #[allow(dead_code)]
    fn len(&self) -> u8 {
        self.count
    }
}

/// Single-cycle IO block.
///
/// GPIO output/OE/input registers + CPUID dispatch.
/// Phase 5 adds spinlocks, FIFOs, doorbells, divider, interpolators.
pub struct Sio {
    /// SIO GPIO output register (offset 0x010).
    pub gpio_out: u32,
    /// SIO GPIO output enable register (offset 0x030).
    pub gpio_oe: u32,
    /// SIO GPIO input register (offset 0x004, always 0 — no external pin model).
    pub gpio_in: u32,
    /// Inter-processor FIFO: Core 0 writes -> Core 1 reads.
    fifo_to_core1: Fifo,
    /// Inter-processor FIFO: Core 1 writes -> Core 0 reads.
    fifo_to_core0: Fifo,
    /// Sticky write-overflow flag, per core.
    fifo_wof: [bool; 2],
    /// Sticky read-underflow flag, per core.
    fifo_roe: [bool; 2],
    /// 32 hardware spinlocks as a bitmask (bit N = SPINLOCK<N> claimed).
    spinlock_bits: u32,
    /// Set by FIFO_WR on successful push; Bus reads and clears this to
    /// set `event_flag[other_core]`. Value is the receiver core index,
    /// or `None` if no event pending.
    pub pending_fifo_event: Option<usize>,
    /// Per-core integer divider (§2.4).
    divider: [Divider; 2],
    /// Doorbell pending bits — 4 bits per core (§2.5).
    pub doorbell_pending: [u8; 2],
    /// 64-bit platform timer counter (§2.6).
    pub mtime: u64,
    /// MTIME control register — bit 0 = enable (§2.6).
    pub mtime_ctrl: u32,
    /// Per-core 64-bit compare value (§2.6).
    pub mtimecmp: [u64; 2],
    /// Per-core edge-triggered match flag (§2.6).
    pub mtime_match_asserted: [bool; 2],
    /// Per-core interpolator register backing store (§2.7).
    /// 2 cores x 32 words (INTERP0 at 0x080–0x0BC, INTERP1 at 0x0C0–0x0FC).
    interp: [[u32; 32]; 2],
}

impl Sio {
    pub fn new() -> Self {
        Self {
            gpio_out: 0,
            gpio_oe: 0,
            gpio_in: 0,
            fifo_to_core1: Fifo::new(),
            fifo_to_core0: Fifo::new(),
            fifo_wof: [false; 2],
            fifo_roe: [false; 2],
            spinlock_bits: 0,
            pending_fifo_event: None,
            divider: [Divider::default(); 2],
            doorbell_pending: [0; 2],
            mtime: 0,
            mtime_ctrl: 0,
            mtimecmp: [0; 2],
            mtime_match_asserted: [false; 2],
            interp: [[0; 32]; 2],
        }
    }

    /// Explicitly reset all SIO state. Called from `Emulator::reset()`.
    pub fn reset(&mut self) {
        self.gpio_out = 0;
        self.gpio_oe = 0;
        self.gpio_in = 0;
        self.fifo_to_core1 = Fifo::new();
        self.fifo_to_core0 = Fifo::new();
        self.fifo_wof = [false; 2];
        self.fifo_roe = [false; 2];
        self.spinlock_bits = 0;
        self.pending_fifo_event = None;
        self.divider = [Divider::default(); 2];
        self.doorbell_pending = [0; 2];
        self.mtime = 0;
        self.mtime_ctrl = 0;
        self.mtimecmp = [0; 2];
        self.mtime_match_asserted = [false; 2];
        self.interp = [[0; 32]; 2];
    }

    /// 32-bit register read. `offset` is already masked to 12 bits by Bus.
    /// GPIO_HI_IN (0x008) is handled by Bus before calling this.
    pub fn read32(&mut self, offset: u32, core: usize) -> u32 {
        match offset {
            0x000 => core as u32,   // CPUID
            0x004 => self.gpio_in,  // GPIO_IN
            0x010 => self.gpio_out, // GPIO_OUT
            0x030 => self.gpio_oe,  // GPIO_OE
            // FIFO
            0x050 => self.fifo_st_read(core),
            0x058 => self.fifo_rd(core),
            // Spinlocks
            0x05C => self.spinlock_bits,  // SPINLOCK_ST
            // Integer divider (0x060–0x078)
            0x060 | 0x068 => self.divider[core].dividend,  // DIV_UDIVIDEND / DIV_SDIVIDEND
            0x064 | 0x06C => self.divider[core].divisor,   // DIV_UDIVISOR / DIV_SDIVISOR
            0x070 | 0x074 => self.divider_result_read(offset, core),
            0x078 => {  // DIV_CSR
                let ready = 1u32;
                let dirty = if self.divider[core].dirty { 2 } else { 0 };
                ready | dirty
            }
            // Interpolators (0x080–0x0FC)
            0x080..=0x0FC => {
                let idx = ((offset - 0x080) >> 2) as usize;
                self.interp[core][idx]
            }
            0x100..=0x17F => self.spinlock_read(offset),
            // Doorbells
            0x188 => self.doorbell_pending[core] as u32,  // DOORBELL_IN_SET read
            // MTIME registers (0x1A0–0x1BC)
            0x1A0 => self.mtime_ctrl,
            0x1A8 => self.mtime as u32,
            0x1AC => (self.mtime >> 32) as u32,
            0x1B0 => self.mtimecmp[0] as u32,
            0x1B4 => (self.mtimecmp[0] >> 32) as u32,
            0x1B8 => self.mtimecmp[1] as u32,
            0x1BC => (self.mtimecmp[1] >> 32) as u32,
            _ => 0,
        }
    }

    /// 32-bit register write. `offset` is already masked to 12 bits by Bus.
    pub fn write32(&mut self, offset: u32, val: u32, core: usize) {
        match offset {
            // GPIO_OUT: RP2350 offsets (8-byte spacing)
            0x010 => self.gpio_out = val,
            0x018 => self.gpio_out |= val,    // GPIO_OUT_SET
            0x020 => self.gpio_out &= !val,   // GPIO_OUT_CLR
            0x028 => self.gpio_out ^= val,    // GPIO_OUT_XOR
            // GPIO_OE: RP2350 offsets (8-byte spacing)
            0x030 => self.gpio_oe = val,
            0x038 => self.gpio_oe |= val,     // GPIO_OE_SET
            0x040 => self.gpio_oe &= !val,    // GPIO_OE_CLR
            0x048 => self.gpio_oe ^= val,     // GPIO_OE_XOR
            // FIFO
            0x050 => self.fifo_st_write(val, core),
            0x054 => self.fifo_wr(val, core),
            // Integer divider (0x060–0x078)
            0x060..=0x078 => self.divider_write(offset, val, core),
            // Interpolators (0x080–0x0FC)
            0x080..=0x0FC => {
                let idx = ((offset - 0x080) >> 2) as usize;
                if idx < 32 {
                    self.interp[core][idx] = val;
                }
            }
            // Spinlocks
            0x100..=0x17F => self.spinlock_write(offset),
            // Doorbells
            0x180 => self.doorbell_pending[1 - core] |= (val & 0xF) as u8,   // DOORBELL_OUT_SET
            0x184 => self.doorbell_pending[1 - core] &= !((val & 0xF) as u8), // DOORBELL_OUT_CLR
            0x18C => self.doorbell_pending[core] &= !((val & 0xF) as u8),     // DOORBELL_IN_CLR
            // MTIME registers (0x1A0–0x1BC)
            0x1A0 => self.mtime_ctrl = val,
            0x1A8 => self.mtime = (self.mtime & 0xFFFF_FFFF_0000_0000) | val as u64,
            0x1AC => self.mtime = (self.mtime & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
            0x1B0 => self.mtimecmp[0] = (self.mtimecmp[0] & 0xFFFF_FFFF_0000_0000) | val as u64,
            0x1B4 => self.mtimecmp[0] = (self.mtimecmp[0] & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
            0x1B8 => self.mtimecmp[1] = (self.mtimecmp[1] & 0xFFFF_FFFF_0000_0000) | val as u64,
            0x1BC => self.mtimecmp[1] = (self.mtimecmp[1] & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
            _ => {}
        }
    }

    // --- FIFO helpers ---

    /// Read FIFO_ST: status register from the calling core's perspective.
    fn fifo_st_read(&self, core: usize) -> u32 {
        // Bit 0: VLD -- this core's RX queue has data
        let rx_fifo = if core == 0 { &self.fifo_to_core0 } else { &self.fifo_to_core1 };
        let vld = !rx_fifo.is_empty();
        // Bit 1: RDY -- other core's RX queue has space
        let tx_fifo = if core == 0 { &self.fifo_to_core1 } else { &self.fifo_to_core0 };
        let rdy = !tx_fifo.is_full();
        // Bit 2: WOF (sticky write overflow)
        let wof = self.fifo_wof[core];
        // Bit 3: ROE (sticky read underflow)
        let roe = self.fifo_roe[core];

        (vld as u32) | ((rdy as u32) << 1) | ((wof as u32) << 2) | ((roe as u32) << 3)
    }

    /// Write FIFO_ST: W1C for WOF and ROE bits.
    fn fifo_st_write(&mut self, val: u32, core: usize) {
        if val & 0x4 != 0 {
            self.fifo_wof[core] = false;
        }
        if val & 0x8 != 0 {
            self.fifo_roe[core] = false;
        }
    }

    /// Write FIFO_WR: push to OTHER core's RX queue.
    fn fifo_wr(&mut self, val: u32, core: usize) {
        let other = 1 - core;
        let tx_fifo = if core == 0 {
            &mut self.fifo_to_core1
        } else {
            &mut self.fifo_to_core0
        };
        if tx_fifo.push(val) {
            // Successful push -- signal event to receiver core.
            self.pending_fifo_event = Some(other);
        } else {
            // Full -- drop data, set WOF for writer.
            self.fifo_wof[core] = true;
        }
    }

    /// Read FIFO_RD: pop from THIS core's RX queue.
    fn fifo_rd(&mut self, core: usize) -> u32 {
        let rx_fifo = if core == 0 {
            &mut self.fifo_to_core0
        } else {
            &mut self.fifo_to_core1
        };
        match rx_fifo.pop() {
            Some(val) => val,
            None => {
                self.fifo_roe[core] = true;
                0
            }
        }
    }

    // --- Spinlock helpers ---

    /// Read SPINLOCK<N>: test-and-set. Returns 1<<N on success, 0 if already claimed.
    fn spinlock_read(&mut self, offset: u32) -> u32 {
        let n = ((offset - 0x100) >> 2) as u32;
        debug_assert!(n < 32);
        let mask = 1u32 << n;
        if self.spinlock_bits & mask == 0 {
            self.spinlock_bits |= mask;
            mask
        } else {
            0
        }
    }

    /// Write SPINLOCK<N>: release (clear bit N, any value).
    fn spinlock_write(&mut self, offset: u32) {
        let n = ((offset - 0x100) >> 2) as u32;
        debug_assert!(n < 32);
        self.spinlock_bits &= !(1u32 << n);
    }

    // --- Integer divider helpers (§2.4) ---

    /// Read quotient or remainder, advancing the reads_pending counter.
    /// Clears DIRTY after both quotient and remainder have been read.
    fn divider_result_read(&mut self, offset: u32, core: usize) -> u32 {
        let d = &mut self.divider[core];
        let val = match offset {
            0x070 => d.quotient,
            0x074 => d.remainder,
            _ => return 0,
        };
        if d.dirty {
            d.reads_pending += 1;
            if d.reads_pending >= 2 {
                d.dirty = false;
                d.reads_pending = 0;
            }
        }
        val
    }

    fn divider_write(&mut self, offset: u32, val: u32, core: usize) {
        let d = &mut self.divider[core];
        match offset {
            0x060 => { // DIV_UDIVIDEND
                d.dividend = val;
                d.signed = false;
            }
            0x064 => { // DIV_UDIVISOR — triggers unsigned computation
                d.divisor = val;
                d.signed = false;
                Self::compute_division(d);
            }
            0x068 => { // DIV_SDIVIDEND
                d.dividend = val;
                d.signed = true;
            }
            0x06C => { // DIV_SDIVISOR — triggers signed computation
                d.divisor = val;
                d.signed = true;
                Self::compute_division(d);
            }
            0x070 => { // DIV_QUOTIENT (direct set)
                d.quotient = val;
                d.dirty = true;
                d.reads_pending = 0;
            }
            0x074 => { // DIV_REMAINDER (direct set)
                d.remainder = val;
                d.dirty = true;
                d.reads_pending = 0;
            }
            _ => {}
        }
    }

    fn compute_division(d: &mut Divider) {
        if d.divisor == 0 {
            // Division by zero (RP2350 behavior)
            if d.signed {
                let dividend_signed = d.dividend as i32;
                d.quotient = if dividend_signed < 0 { 1u32 } else { (-1i32) as u32 };
            } else {
                d.quotient = 0xFFFF_FFFF;
            }
            d.remainder = d.dividend;
        } else if d.signed {
            let a = d.dividend as i32;
            let b = d.divisor as i32;
            d.quotient = a.wrapping_div(b) as u32;
            d.remainder = a.wrapping_rem(b) as u32;
        } else {
            d.quotient = d.dividend.wrapping_div(d.divisor);
            d.remainder = d.dividend.wrapping_rem(d.divisor);
        }
        d.dirty = true;
        d.reads_pending = 0;
    }

    // --- MTIME helpers (§2.6) ---

    /// Tick the MTIME counter. Called once per `Emulator::step()` after
    /// both cores have stepped and the wake check has run.
    pub fn tick_mtime(&mut self) {
        if self.mtime_ctrl & 1 != 0 {
            let new_mtime = self.mtime.wrapping_add(1);
            self.mtime = new_mtime;
            for core in 0..2 {
                let match_now = new_mtime >= self.mtimecmp[core];
                if match_now && !self.mtime_match_asserted[core] {
                    self.mtime_match_asserted[core] = true;
                }
                if !match_now {
                    self.mtime_match_asserted[core] = false;
                }
            }
        }
    }
}

impl Default for Sio {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Integer divider tests (Stage B3) ----

    #[test]
    fn divider_unsigned_basic() {
        let mut sio = Sio::new();
        let core = 0;
        // 100 / 7 = 14 remainder 2
        sio.write32(0x060, 100, core); // DIV_UDIVIDEND
        sio.write32(0x064, 7, core);   // DIV_UDIVISOR
        assert_eq!(sio.read32(0x070, core), 14);
        assert_eq!(sio.read32(0x074, core), 2);
    }

    #[test]
    fn divider_signed_basic() {
        let mut sio = Sio::new();
        let core = 0;
        // -100 / 7 = -14 remainder -2
        sio.write32(0x068, (-100i32) as u32, core); // DIV_SDIVIDEND
        sio.write32(0x06C, 7, core);                // DIV_SDIVISOR
        assert_eq!(sio.read32(0x070, core) as i32, -14);
        assert_eq!(sio.read32(0x074, core) as i32, -2);
    }

    #[test]
    fn divider_signed_negative_divisor() {
        let mut sio = Sio::new();
        let core = 0;
        // 100 / -7 = -14 remainder 2
        sio.write32(0x068, 100, core);
        sio.write32(0x06C, (-7i32) as u32, core);
        assert_eq!(sio.read32(0x070, core) as i32, -14);
        assert_eq!(sio.read32(0x074, core) as i32, 2);
    }

    #[test]
    fn divider_unsigned_div_by_zero() {
        let mut sio = Sio::new();
        let core = 0;
        sio.write32(0x060, 42, core); // dividend = 42
        sio.write32(0x064, 0, core);  // divisor = 0
        assert_eq!(sio.read32(0x070, core), 0xFFFF_FFFF);
        assert_eq!(sio.read32(0x074, core), 42);
    }

    #[test]
    fn divider_signed_div_by_zero_positive() {
        let mut sio = Sio::new();
        let core = 0;
        sio.write32(0x068, 42, core); // dividend = 42 (positive)
        sio.write32(0x06C, 0, core);  // divisor = 0
        // positive dividend / 0 → quotient = -1
        assert_eq!(sio.read32(0x070, core) as i32, -1);
        assert_eq!(sio.read32(0x074, core), 42);
    }

    #[test]
    fn divider_signed_div_by_zero_negative() {
        let mut sio = Sio::new();
        let core = 0;
        sio.write32(0x068, (-42i32) as u32, core); // dividend = -42
        sio.write32(0x06C, 0, core);               // divisor = 0
        // negative dividend / 0 → quotient = 1
        assert_eq!(sio.read32(0x070, core), 1);
        assert_eq!(sio.read32(0x074, core), (-42i32) as u32);
    }

    #[test]
    fn divider_dirty_flag_clear_after_both_reads() {
        let mut sio = Sio::new();
        let core = 0;
        sio.write32(0x060, 100, core);
        sio.write32(0x064, 7, core);
        // CSR should show DIRTY (bit 1) and READY (bit 0)
        assert_eq!(sio.read32(0x078, core) & 0x3, 0x3);
        // Read quotient — still dirty
        sio.read32(0x070, core);
        assert_eq!(sio.read32(0x078, core) & 0x2, 0x2);
        // Read remainder — dirty should clear
        sio.read32(0x074, core);
        assert_eq!(sio.read32(0x078, core) & 0x2, 0x0);
        // READY always 1
        assert_eq!(sio.read32(0x078, core) & 0x1, 0x1);
    }

    #[test]
    fn divider_per_core_isolation() {
        let mut sio = Sio::new();
        sio.write32(0x060, 100, 0);
        sio.write32(0x064, 10, 0);
        sio.write32(0x060, 200, 1);
        sio.write32(0x064, 20, 1);
        assert_eq!(sio.read32(0x070, 0), 10);
        assert_eq!(sio.read32(0x070, 1), 10);
    }

    #[test]
    fn divider_direct_write_quotient_remainder() {
        let mut sio = Sio::new();
        let core = 0;
        sio.write32(0x070, 0xDEAD, core);
        sio.write32(0x074, 0xBEEF, core);
        assert_eq!(sio.read32(0x070, core), 0xDEAD);
        assert_eq!(sio.read32(0x074, core), 0xBEEF);
    }

    // ---- Doorbell tests (Stage C1) ----

    #[test]
    fn doorbell_set_and_read() {
        let mut sio = Sio::new();
        // Core 0 sets doorbell for core 1
        sio.write32(0x180, 0x5, 0); // DOORBELL_OUT_SET
        assert_eq!(sio.read32(0x188, 1), 0x5); // Core 1 reads DOORBELL_IN_SET
        assert_eq!(sio.read32(0x188, 0), 0x0); // Core 0 sees nothing
    }

    #[test]
    fn doorbell_clr() {
        let mut sio = Sio::new();
        // Core 0 sets all doorbells for core 1
        sio.write32(0x180, 0xF, 0);
        assert_eq!(sio.read32(0x188, 1), 0xF);
        // Core 0 clears bit 1 on core 1
        sio.write32(0x184, 0x2, 0); // DOORBELL_OUT_CLR
        assert_eq!(sio.read32(0x188, 1), 0xD);
    }

    #[test]
    fn doorbell_in_clr() {
        let mut sio = Sio::new();
        // Core 1 sets doorbell for core 0
        sio.write32(0x180, 0xA, 1);
        assert_eq!(sio.read32(0x188, 0), 0xA);
        // Core 0 clears its own doorbell via DOORBELL_IN_CLR
        sio.write32(0x18C, 0x8, 0);
        assert_eq!(sio.read32(0x188, 0), 0x2);
    }

    #[test]
    fn doorbell_masks_to_4_bits() {
        let mut sio = Sio::new();
        sio.write32(0x180, 0xFF, 0);
        assert_eq!(sio.read32(0x188, 1), 0xF); // Only lower 4 bits
    }

    // ---- MTIME tests (Stage C2) ----

    #[test]
    fn mtime_counting() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1; // Enable
        sio.tick_mtime();
        assert_eq!(sio.mtime, 1);
        sio.tick_mtime();
        assert_eq!(sio.mtime, 2);
    }

    #[test]
    fn mtime_disabled_no_count() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 0; // Disabled
        sio.tick_mtime();
        assert_eq!(sio.mtime, 0);
    }

    #[test]
    fn mtime_compare_match_edge() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1;
        sio.mtimecmp[0] = 3;
        sio.tick_mtime(); // mtime = 1
        assert!(!sio.mtime_match_asserted[0]);
        sio.tick_mtime(); // mtime = 2
        assert!(!sio.mtime_match_asserted[0]);
        sio.tick_mtime(); // mtime = 3 → match fires
        assert!(sio.mtime_match_asserted[0]);
        sio.tick_mtime(); // mtime = 4 → still asserted (level)
        assert!(sio.mtime_match_asserted[0]);
    }

    #[test]
    fn mtime_compare_rewrite_above_clears() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1;
        sio.mtimecmp[0] = 2;
        sio.tick_mtime(); // 1
        sio.tick_mtime(); // 2 → match
        assert!(sio.mtime_match_asserted[0]);
        // Rewrite compare to value above current mtime
        sio.mtimecmp[0] = 100;
        sio.tick_mtime(); // 3 < 100 → clears
        assert!(!sio.mtime_match_asserted[0]);
    }

    #[test]
    fn mtime_wraparound() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1;
        sio.mtime = u64::MAX;
        sio.mtimecmp[0] = 5;
        sio.tick_mtime(); // wraps to 0
        assert_eq!(sio.mtime, 0);
        // 0 < 5 → not matched
        assert!(!sio.mtime_match_asserted[0]);
    }

    #[test]
    fn mtime_register_read_write() {
        let mut sio = Sio::new();
        // Write MTIME_CTRL
        sio.write32(0x1A0, 0x1, 0);
        assert_eq!(sio.read32(0x1A0, 0), 0x1);
        // Write MTIME low + high
        sio.write32(0x1A8, 0xDEAD_BEEF, 0);
        sio.write32(0x1AC, 0x0000_0042, 0);
        assert_eq!(sio.mtime, 0x0000_0042_DEAD_BEEF);
        assert_eq!(sio.read32(0x1A8, 0), 0xDEAD_BEEF);
        assert_eq!(sio.read32(0x1AC, 0), 0x42);
        // Write MTIMECMP0
        sio.write32(0x1B0, 0x1111, 0);
        sio.write32(0x1B4, 0x2222, 0);
        assert_eq!(sio.mtimecmp[0], 0x0000_2222_0000_1111);
        // Write MTIMECMP1
        sio.write32(0x1B8, 0x3333, 0);
        sio.write32(0x1BC, 0x4444, 0);
        assert_eq!(sio.mtimecmp[1], 0x0000_4444_0000_3333);
    }

    // ---- Interpolator stub tests (Stage C3) ----

    #[test]
    fn interp_register_round_trip() {
        let mut sio = Sio::new();
        let core = 0;
        // Write to INTERP0_ACCUM0 (offset 0x080)
        sio.write32(0x080, 0xCAFE_BABE, core);
        assert_eq!(sio.read32(0x080, core), 0xCAFE_BABE);
        // Write to last register (offset 0x0FC)
        sio.write32(0x0FC, 0xDEAD_BEEF, core);
        assert_eq!(sio.read32(0x0FC, core), 0xDEAD_BEEF);
        // First register unchanged
        assert_eq!(sio.read32(0x080, core), 0xCAFE_BABE);
    }

    #[test]
    fn interp_per_core_isolation() {
        let mut sio = Sio::new();
        sio.write32(0x080, 0x1111, 0);
        sio.write32(0x080, 0x2222, 1);
        assert_eq!(sio.read32(0x080, 0), 0x1111);
        assert_eq!(sio.read32(0x080, 1), 0x2222);
    }

    #[test]
    fn interp_all_registers() {
        let mut sio = Sio::new();
        let core = 0;
        // Write all 32 words
        for i in 0u32..32 {
            sio.write32(0x080 + i * 4, i + 1, core);
        }
        // Read them all back
        for i in 0u32..32 {
            assert_eq!(sio.read32(0x080 + i * 4, core), i + 1);
        }
    }
}

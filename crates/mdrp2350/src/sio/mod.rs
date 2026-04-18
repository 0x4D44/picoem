use mdpicoem_common::Fifo;

/// Single-cycle IO block.
///
/// GPIO output/OE/input registers + CPUID dispatch, FIFOs, spinlocks,
/// doorbells, MTIME. Phase 3 Stage 3 (LLD V7 §6): the **per-core**
/// DIV and INTERP register files have moved off `Sio` onto each
/// `CortexM33` as `PerCoreSio`, because cores see distinct register
/// state there. Intercept lives in `CortexM33::bus_read32/write32` at
/// SIO offsets 0x060..=0x0FC — those offsets never reach `Sio`.
pub struct Sio {
    /// SIO GPIO output register (offset 0x010).
    pub gpio_out: u32,
    /// SIO GPIO output enable register (offset 0x030).
    pub gpio_oe: u32,
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
}

impl Sio {
    pub fn new() -> Self {
        Self {
            gpio_out: 0,
            gpio_oe: 0,
            fifo_to_core1: Fifo::new(),
            fifo_to_core0: Fifo::new(),
            fifo_wof: [false; 2],
            fifo_roe: [false; 2],
            spinlock_bits: 0,
            pending_fifo_event: None,
            doorbell_pending: [0; 2],
            mtime: 0,
            mtime_ctrl: 0,
            mtimecmp: [0; 2],
            mtime_match_asserted: [false; 2],
        }
    }

    /// Non-consuming snapshot of the `core0 → core1` FIFO in head→tail
    /// order. Used by `threaded::ThreadedSio::seed` to carry the
    /// single-threaded inter-core FIFO state into the threaded SPSC
    /// ring. Empty when the FIFO is empty.
    pub fn fifo_0to1_snapshot(&self) -> Vec<u32> {
        self.fifo_to_core1.snapshot()
    }

    /// Non-consuming snapshot of the `core1 → core0` FIFO in head→tail
    /// order. See [`Self::fifo_0to1_snapshot`].
    pub fn fifo_1to0_snapshot(&self) -> Vec<u32> {
        self.fifo_to_core0.snapshot()
    }

    /// Read the sticky FIFO write-overflow flag for `core`.
    pub fn fifo_wof(&self, core: usize) -> bool {
        debug_assert!(core < 2);
        self.fifo_wof[core]
    }

    /// Read the sticky FIFO read-underflow flag for `core`.
    pub fn fifo_roe(&self, core: usize) -> bool {
        debug_assert!(core < 2);
        self.fifo_roe[core]
    }

    /// Read the 32-lock spinlock claim bitmask. Bit N set = SPINLOCK<N>
    /// is currently claimed.
    pub fn spinlock_bits(&self) -> u32 {
        self.spinlock_bits
    }

    /// Explicitly reset all SIO state. Called from `Emulator::reset()`.
    /// Per-core DIV/INTERP state (`PerCoreSio`) is cleared on the
    /// individual `CortexM33`s in `Emulator::reset` — not here.
    pub fn reset(&mut self) {
        self.gpio_out = 0;
        self.gpio_oe = 0;
        self.fifo_to_core1 = Fifo::new();
        self.fifo_to_core0 = Fifo::new();
        self.fifo_wof = [false; 2];
        self.fifo_roe = [false; 2];
        self.spinlock_bits = 0;
        self.pending_fifo_event = None;
        self.doorbell_pending = [0; 2];
        self.mtime = 0;
        self.mtime_ctrl = 0;
        self.mtimecmp = [0; 2];
        self.mtime_match_asserted = [false; 2];
    }

    /// 32-bit register read. `offset` is already masked to 12 bits by Bus.
    /// GPIO_IN (0x004) and GPIO_HI_IN (0x008) are handled by Bus before
    /// calling this; DIV and INTERP (0x060..=0x0FC) are intercepted on
    /// `CortexM33` into `sio_local` and never reach here (Phase 3
    /// Stage 3, LLD V7 §6).
    pub fn read32(&mut self, offset: u32, core: usize) -> u32 {
        match offset {
            0x000 => core as u32,   // CPUID
            0x010 => self.gpio_out, // GPIO_OUT
            0x030 => self.gpio_oe,  // GPIO_OE
            // FIFO
            0x050 => self.fifo_st_read(core),
            0x058 => self.fifo_rd(core),
            // Spinlocks
            0x05C => self.spinlock_bits,  // SPINLOCK_ST
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
    /// DIV and INTERP (0x060..=0x0FC) are intercepted on `CortexM33` and
    /// never reach here — see [`Self::read32`] for the Phase 3 Stage 3
    /// split.
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

    // --- CP0 GPIOC fast-path methods (Phase 7 Stage C) ---
    //
    // These expose the SIO output/OE state to the CP0 coprocessor without a
    // bus round-trip. Input state lives on `Bus` per HLD §C.3, so no
    // `gpio_bit_in_get` method is provided here — CP0 reads `bus.gpio_in`
    // directly.
    //
    // RP2354A target: 30 pins. Bits [31:30] are masked on writes and read
    // back as zero. The `PIN_MASK` constant encodes this.

    /// Mask of valid GPIO pin bits for RP2354A (30 pins, bits [29:0]).
    pub(crate) const PIN_MASK: u32 = 0x3FFF_FFFF;

    // Per-bit output (GPIO_OUT) operations.

    pub fn gpio_bit_out_get(&self, pin: u8) -> bool {
        if pin >= 30 {
            return false;
        }
        (self.gpio_out >> pin) & 1 != 0
    }

    pub fn gpio_bit_out_put(&mut self, pin: u8, v: bool) {
        if pin >= 30 {
            return;
        }
        let mask = 1u32 << pin;
        if v {
            self.gpio_out |= mask;
        } else {
            self.gpio_out &= !mask;
        }
    }

    pub fn gpio_bit_out_set(&mut self, pin: u8) {
        if pin >= 30 {
            return;
        }
        self.gpio_out |= 1u32 << pin;
    }

    pub fn gpio_bit_out_clr(&mut self, pin: u8) {
        if pin >= 30 {
            return;
        }
        self.gpio_out &= !(1u32 << pin);
    }

    pub fn gpio_bit_out_xor(&mut self, pin: u8) {
        if pin >= 30 {
            return;
        }
        self.gpio_out ^= 1u32 << pin;
    }

    // Per-bit output-enable (GPIO_OE) operations.

    pub fn gpio_bit_oe_get(&self, pin: u8) -> bool {
        if pin >= 30 {
            return false;
        }
        (self.gpio_oe >> pin) & 1 != 0
    }

    pub fn gpio_bit_oe_put(&mut self, pin: u8, v: bool) {
        if pin >= 30 {
            return;
        }
        let mask = 1u32 << pin;
        if v {
            self.gpio_oe |= mask;
        } else {
            self.gpio_oe &= !mask;
        }
    }

    pub fn gpio_bit_oe_set(&mut self, pin: u8) {
        if pin >= 30 {
            return;
        }
        self.gpio_oe |= 1u32 << pin;
    }

    pub fn gpio_bit_oe_clr(&mut self, pin: u8) {
        if pin >= 30 {
            return;
        }
        self.gpio_oe &= !(1u32 << pin);
    }

    pub fn gpio_bit_oe_xor(&mut self, pin: u8) {
        if pin >= 30 {
            return;
        }
        self.gpio_oe ^= 1u32 << pin;
    }

    // Bulk GPIO_OUT operations — whole-bank (30 valid pins on RP2354A).

    pub fn gpio_lo_out_get(&self) -> u32 {
        self.gpio_out & Self::PIN_MASK
    }

    pub fn gpio_lo_out_put(&mut self, v: u32) {
        self.gpio_out = v & Self::PIN_MASK;
    }

    pub fn gpio_lo_out_set(&mut self, v: u32) {
        self.gpio_out |= v & Self::PIN_MASK;
    }

    pub fn gpio_lo_out_clr(&mut self, v: u32) {
        self.gpio_out &= !(v & Self::PIN_MASK);
    }

    pub fn gpio_lo_out_xor(&mut self, v: u32) {
        self.gpio_out ^= v & Self::PIN_MASK;
    }

    // Bulk GPIO_OE operations.

    pub fn gpio_lo_oe_get(&self) -> u32 {
        self.gpio_oe & Self::PIN_MASK
    }

    pub fn gpio_lo_oe_put(&mut self, v: u32) {
        self.gpio_oe = v & Self::PIN_MASK;
    }

    pub fn gpio_lo_oe_set(&mut self, v: u32) {
        self.gpio_oe |= v & Self::PIN_MASK;
    }

    pub fn gpio_lo_oe_clr(&mut self, v: u32) {
        self.gpio_oe &= !(v & Self::PIN_MASK);
    }

    pub fn gpio_lo_oe_xor(&mut self, v: u32) {
        self.gpio_oe ^= v & Self::PIN_MASK;
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

    // Integer divider helpers moved to `core::PerCoreSio` in Phase 3
    // Stage 3 (LLD V7 §6). See `crates/mdrp2350/src/core/mod.rs`.

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

    /// Bulk-advance MTIME by `n` cycles. Quantum-end variant of
    /// [`Self::tick_mtime`]. Match-asserted flags are updated once against
    /// the final post-advance value — interrupt edges that land mid-quantum
    /// are still observed, but with up-to-one-quantum latency, consistent
    /// with the quantum execution model.
    pub fn tick_mtime_n(&mut self, n: u32) {
        if n == 0 || self.mtime_ctrl & 1 == 0 {
            return;
        }
        let new_mtime = self.mtime.wrapping_add(n as u64);
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

impl Default for Sio {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Integer divider and interpolator tests moved to `core::mod`'s
    // `tests` module alongside `PerCoreSio` in Phase 3 Stage 3 (LLD V7
    // §6). DIV / INTERP are now per-core state on `CortexM33`, not
    // shared SIO state.

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

    // Interpolator tests moved to `core::mod`'s `tests` module — see
    // the head of this test module.
}

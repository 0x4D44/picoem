use tracing::trace;

use mdpicoem_common::Fifo;

mod interp;
pub use interp::Interp;

/// `MTIME_CTRL.EN` bit mask (datasheet §3.1 Table 80, bit 0).
/// When 0, the MTIME counter is frozen regardless of tick source.
pub(crate) const MTIME_CTRL_EN: u32 = 1 << 0;

/// `MTIME_CTRL.FULLSPEED` bit mask (datasheet §3.1 Table 80, bit 1).
/// When 1, MTIME increments every sys_clk, bypassing `TICKS.RISCV`;
/// when 0, MTIME advances only on `TICKS.RISCV` edges.
pub(crate) const MTIME_CTRL_FULLSPEED: u32 = 1 << 1;

/// `MTIME_CTRL` reset value per datasheet §3.1 Table 80.
///
/// Bits: EN=1 (bit 0), FULLSPEED=0 (bit 1), DBGPAUSE_CORE0=1 (bit 2),
/// DBGPAUSE_CORE1=1 (bit 3) → `0b1101 = 0x0D`.
///
/// Residual A.2.1 corrected this from the pre-existing Stage A value of
/// 0x0F, which incorrectly asserted FULLSPEED=1 at reset. See
/// `wrk_docs/2026.04.17 - HLD - Residual A.2.1 MTIME WATCHDOG_TICK Fix.md`.
pub(crate) const MTIME_CTRL_RESET: u32 = 0x0D;

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
    /// RISCV_SOFTIRQ register (offset 0x1A0, §3.1, Table 79). Split SET/CLR
    /// register: bits [1:0] (CORE0_SET/CORE1_SET) write-1-to-set; bits [9:8]
    /// (CORE0_CLR/CORE1_CLR) write-1-to-clear. Read returns only bits [1:0]
    /// (status); bits [9:8] are write-only and always read as 0.
    riscv_softirq: u32,
    /// 64-bit platform timer counter (§2.6).
    pub mtime: u64,
    /// MTIME control register at offset 0x1A4 — bit 0 = enable (§2.6).
    pub mtime_ctrl: u32,
    /// Per-core 64-bit compare value (§2.6).
    pub mtimecmp: [u64; 2],
    /// Per-core edge-triggered match flag (§2.6).
    pub mtime_match_asserted: [bool; 2],
    // Phase 3 Stage 3 (LLD V7 §6): INTERP0/INTERP1 moved off `Sio` onto
    // each `CortexM33` as `PerCoreSio::interp` — see `core/mod.rs`.
    // Dispatch at 0x080..=0x0FC is intercepted in `CortexM33::bus_read32`
    // / `bus_write32` and never reaches `Sio`. Live semantics (SHIFT,
    // MASK_LSB/MSB, SIGNED, CROSS_INPUT/RESULT, ADD_RAW, FORCE_MSB,
    // BLEND, CLAMP, OVERF) live in `sio::interp::Interp`.
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
            riscv_softirq: 0,
            mtime: 0,
            mtime_ctrl: MTIME_CTRL_RESET,
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

    /// Read the RISCV_SOFTIRQ register state (bits [1:0] = per-hart
    /// software-interrupt flags). Exposed for P4's `fan_out_riscv_irqs`
    /// which drives `mip[3]` (MSIP) from these bits — HLD §4.6.
    pub fn riscv_softirq(&self) -> u32 {
        self.riscv_softirq & 0x3
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
        self.riscv_softirq = 0;
        self.mtime = 0;
        self.mtime_ctrl = MTIME_CTRL_RESET;
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
            // DIV (0x060..0x078) and INTERP (0x080..0x0FC) are per-core
            // state on `PerCoreSio`; intercepted by `CortexM33::bus_read32`
            // and never reach here. (Phase 3 Stage 3 — LLD V7 §6.)
            0x100..=0x17F => self.spinlock_read(offset),
            // Doorbells
            0x188 => self.doorbell_pending[core] as u32,  // DOORBELL_IN_SET read
            // RISCV_SOFTIRQ (0x1A0, §3.1): per-core RISC-V SW interrupt flags.
            // Bits [1:0] = {CORE1_SET, CORE0_SET}. Write sets; read returns state.
            0x1A0 => self.riscv_softirq & 0x3,
            // MTIME registers (0x1A4–0x1BC, §2.6)
            0x1A4 => self.mtime_ctrl,
            0x1B0 => self.mtime as u32,
            0x1B4 => (self.mtime >> 32) as u32,
            0x1B8 => self.mtimecmp[core] as u32,
            0x1BC => (self.mtimecmp[core] >> 32) as u32,
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
            // DIV (0x060..0x078) and INTERP (0x080..0x0FC) are per-core
            // state on `PerCoreSio`; intercepted by `CortexM33::bus_write32`
            // and never reach here. (Phase 3 Stage 3 — LLD V7 §6.)
            // Spinlocks
            0x100..=0x17F => self.spinlock_write(offset),
            // Doorbells
            0x180 => self.doorbell_pending[1 - core] |= (val & 0xF) as u8,   // DOORBELL_OUT_SET
            0x184 => self.doorbell_pending[1 - core] &= !((val & 0xF) as u8), // DOORBELL_OUT_CLR
            0x18C => self.doorbell_pending[core] &= !((val & 0xF) as u8),     // DOORBELL_IN_CLR
            // RISCV_SOFTIRQ (0x1A0, §3.1, Table 79): split SET/CLR register.
            // Bits [1:0]  = CORE0_SET/CORE1_SET: write 1 sets the IRQ flag.
            // Bits [9:8]  = CORE0_CLR/CORE1_CLR: write 1 clears the IRQ flag.
            // Per datasheet: if a flag is both set and cleared on the same cycle,
            // only the set takes effect (apply clear first, then set).
            // Bits [9:8] are write-only control; they always read as 0.
            0x1A0 => {
                let set_bits = val & 0x3;
                let clr_bits = (val >> 8) & 0x3;
                self.riscv_softirq &= !clr_bits;
                self.riscv_softirq |= set_bits;
            }
            // MTIME registers (0x1A4–0x1BC, §2.6)
            0x1A4 => self.mtime_ctrl = val,
            0x1B0 => self.mtime = (self.mtime & 0xFFFF_FFFF_0000_0000) | val as u64,
            0x1B4 => self.mtime = (self.mtime & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
            0x1B8 => self.mtimecmp[core] = (self.mtimecmp[core] & 0xFFFF_FFFF_0000_0000) | val as u64,
            0x1BC => self.mtimecmp[core] = (self.mtimecmp[core] & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
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
            trace!(lock_id = n, "spinlock acquired");
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
        trace!(lock_id = n, "spinlock released");
    }

    // Integer divider helpers moved to `core::PerCoreSio` in Phase 3
    // Stage 3 (LLD V7 §6). See `crates/mdrp2350/src/core/mod.rs`.

    // --- MTIME helpers (§2.6 / §3.1.8) ---

    /// Advance MTIME using tick-generator edges and/or sys_clks.
    ///
    /// Called once per quantum from `Bus::tick_peripherals` after
    /// `TicksRegs::advance_all` has populated `TICKS.RISCV` edges.
    ///
    /// Semantics (RP2350 datasheet §3.1.8 and Table 80):
    /// - `MTIME_CTRL.EN` = 0 → counter frozen, no-op.
    /// - `MTIME_CTRL.FULLSPEED` = 1 → increment by `sys_clks` (bypass TICKS).
    /// - `MTIME_CTRL.FULLSPEED` = 0 → increment by `riscv_edges`.
    ///
    /// Exactly one source is consumed per call: `sys_clks` is ignored when
    /// FULLSPEED=0, and `riscv_edges` is ignored when FULLSPEED=1. Callers
    /// should still pass the real quantum values for both — the mode
    /// selection happens here, not at the call site.
    ///
    /// Match-asserted flags are updated once against the final post-advance
    /// value — interrupt edges that land mid-quantum are still observed,
    /// but with up-to-one-quantum latency, consistent with the quantum
    /// execution model.
    pub fn tick_mtime_from_ticks(&mut self, riscv_edges: u32, sys_clks: u32) {
        if self.mtime_ctrl & MTIME_CTRL_EN == 0 {
            return;
        }
        let n = if self.mtime_ctrl & MTIME_CTRL_FULLSPEED != 0 {
            sys_clks
        } else {
            riscv_edges
        };
        if n == 0 {
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

    // ---- Integer divider tests (RP2350 RAZ/WI — §3.1.7) ----
    //
    // Per-core divider state lives on `PerCoreSio` (Phase 3 Stage 3 —
    // LLD V7 §6). `CortexM33::bus_read32`/`bus_write32` intercepts the
    // 0x060..=0x078 window, so writes/reads direct to `Sio` fall through
    // to the catch-all `_ => 0` / `_ => {}` arms: the tests below
    // exercise that Sio itself never stores anything at divider offsets.

    /// All divider-range offsets read as 0 regardless of prior writes.
    #[test]
    fn div_csr_all_reserved_offsets_raz() {
        let mut sio = Sio::new();
        let core = 0;
        let offsets = [0x060u32, 0x064, 0x068, 0x06C, 0x070, 0x074, 0x078];
        // Cold read — all must be 0.
        for &off in &offsets {
            assert_eq!(sio.read32(off, core), 0,
                "offset 0x{off:03X} must RAZ on cold read");
        }
        // Write arbitrary values, then read back — still 0 (WI).
        for &off in &offsets {
            sio.write32(off, 0xDEAD_BEEF, core);
        }
        for &off in &offsets {
            assert_eq!(sio.read32(off, core), 0,
                "offset 0x{off:03X} must RAZ after arbitrary write (WI)");
        }
    }

    /// Writing operand values and reading DIV_CSR / QUOTIENT / REMAINDER all
    /// return 0 on RP2350 — the divider does not compute anything via Sio.
    #[test]
    fn div_csr_post_write_is_still_zero() {
        let mut sio = Sio::new();
        let core = 0;
        // Simulate what a program would do with the RP2040 divider.
        sio.write32(0x060, 5, core);   // DIV_UDIVIDEND
        sio.write32(0x064, 2, core);   // DIV_UDIVISOR
        assert_eq!(sio.read32(0x078, core), 0, "DIV_CSR must be 0 on RP2350 via Sio");
        assert_eq!(sio.read32(0x070, core), 0, "DIV_QUOTIENT must be 0 on RP2350 via Sio");
        assert_eq!(sio.read32(0x074, core), 0, "DIV_REMAINDER must be 0 on RP2350 via Sio");
    }

    /// Cold-read DIV_CSR is 0 before any writes.
    #[test]
    fn div_csr_cold_read_is_zero() {
        let mut sio = Sio::new();
        assert_eq!(sio.read32(0x078, 0), 0, "DIV_CSR must be 0 on cold read");
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

    // The legacy per-edge `tick_mtime()` helper was retired in Residual A.2.1
    // (HLD `2026.04.17 - HLD - Residual A.2.1 MTIME WATCHDOG_TICK Fix.md`).
    // These tests exercise the single-edge path by calling
    // `tick_mtime_from_ticks(1, 0)` with `MTIME_CTRL.FULLSPEED = 0`, which
    // is the semantically identical invocation.

    #[test]
    fn mtime_counting() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1; // EN=1, FULLSPEED=0
        sio.tick_mtime_from_ticks(1, 0);
        assert_eq!(sio.mtime, 1);
        sio.tick_mtime_from_ticks(1, 0);
        assert_eq!(sio.mtime, 2);
    }

    #[test]
    fn mtime_disabled_no_count() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 0; // Disabled
        sio.tick_mtime_from_ticks(1, 0);
        assert_eq!(sio.mtime, 0);
    }

    #[test]
    fn mtime_compare_match_edge() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1;
        sio.mtimecmp[0] = 3;
        sio.tick_mtime_from_ticks(1, 0); // mtime = 1
        assert!(!sio.mtime_match_asserted[0]);
        sio.tick_mtime_from_ticks(1, 0); // mtime = 2
        assert!(!sio.mtime_match_asserted[0]);
        sio.tick_mtime_from_ticks(1, 0); // mtime = 3 → match fires
        assert!(sio.mtime_match_asserted[0]);
        sio.tick_mtime_from_ticks(1, 0); // mtime = 4 → still asserted (level)
        assert!(sio.mtime_match_asserted[0]);
    }

    #[test]
    fn mtime_compare_rewrite_above_clears() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1;
        sio.mtimecmp[0] = 2;
        sio.tick_mtime_from_ticks(1, 0); // 1
        sio.tick_mtime_from_ticks(1, 0); // 2 → match
        assert!(sio.mtime_match_asserted[0]);
        // Rewrite compare to value above current mtime
        sio.mtimecmp[0] = 100;
        sio.tick_mtime_from_ticks(1, 0); // 3 < 100 → clears
        assert!(!sio.mtime_match_asserted[0]);
    }

    #[test]
    fn mtime_wraparound() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 1;
        sio.mtime = u64::MAX;
        sio.mtimecmp[0] = 5;
        sio.tick_mtime_from_ticks(1, 0); // wraps to 0
        assert_eq!(sio.mtime, 0);
        // 0 < 5 → not matched
        assert!(!sio.mtime_match_asserted[0]);
    }

    /// MTIME_CTRL resets to 0x0D per datasheet Table 80.
    /// Bits: EN=1, FULLSPEED=0, DBGPAUSE_CORE0=1, DBGPAUSE_CORE1=1 -> 0b1101.
    /// Residual A.2.1 corrected this from the pre-existing 0x0F value.
    #[test]
    fn mtime_ctrl_reset_value_is_0x0d() {
        let sio = Sio::new();
        assert_eq!(sio.mtime_ctrl, 0x0D,
            "MTIME_CTRL must reset to 0x0D (EN + DBGPAUSE_CORE0 + DBGPAUSE_CORE1)");
        // Verify reset() also restores 0x0D.
        let mut sio2 = Sio::new();
        sio2.mtime_ctrl = 0;
        sio2.reset();
        assert_eq!(sio2.mtime_ctrl, 0x0D, "MTIME_CTRL must be 0x0D after reset()");
    }

    #[test]
    fn mtime_register_read_write() {
        let mut sio = Sio::new();
        // MTIME_CTRL is at 0x1A4 on RP2350 (0x1A0 is RISCV_SOFTIRQ)
        sio.write32(0x1A4, 0x1, 0);
        assert_eq!(sio.read32(0x1A4, 0), 0x1);
        // Write MTIME low + high (0x1B0/0x1B4 on RP2350)
        sio.write32(0x1B0, 0xDEAD_BEEF, 0);
        sio.write32(0x1B4, 0x0000_0042, 0);
        assert_eq!(sio.mtime, 0x0000_0042_DEAD_BEEF);
        assert_eq!(sio.read32(0x1B0, 0), 0xDEAD_BEEF);
        assert_eq!(sio.read32(0x1B4, 0), 0x42);
        // Write MTIMECMP for core 0 (core-local at 0x1B8/0x1BC)
        sio.write32(0x1B8, 0x1111, 0);
        sio.write32(0x1BC, 0x2222, 0);
        assert_eq!(sio.mtimecmp[0], 0x0000_2222_0000_1111);
        // Write MTIMECMP for core 1 (same offsets, different core)
        sio.write32(0x1B8, 0x3333, 1);
        sio.write32(0x1BC, 0x4444, 1);
        assert_eq!(sio.mtimecmp[1], 0x0000_4444_0000_3333);
    }

    // Interp dispatch via SIO bus write/read is intercepted at
    // `CortexM33::bus_write32` / `bus_read32` (Phase 3 Stage 3 — LLD
    // V7 §6), so the "round-trip via Sio" style of test can't be done
    // here. The detailed arithmetic tests live in `sio::interp::tests`;
    // per-core isolation + bus dispatch tests live in `core::tests`
    // exercising the `PerCoreSio` route.

    // Interp per-core + dispatch tests live in `core::tests` — they
    // must go through `CortexM33::bus_read32/write32` intercept to hit
    // PerCoreSio, not via `sio.write32(0x080, ...)` which falls through
    // to `_ => {}` in threading's model.

    // `interp_all_registers` passive round-trip removed — threading's
    // PerCoreSio intercept + live Interp semantics mean not every offset
    // is a round-trip storage register. Arithmetic coverage lives in
    // `sio::interp::tests`.

    // ---- Silicon oracle regression tests (silicon_periph_diff_rp2350) ----

    /// `sio_divider_unsigned` oracle: DIV_CSR.READY (bit 0) must be 0 on RP2350.
    ///
    /// RP2350 datasheet §3.1.7 — the RP2040 memory-mapped integer divider is
    /// not present; address range 0x060–0x078 is reserved. Silicon reads 0x00
    /// for DIV_CSR, so READY (bit 0) must be 0 in the emulator too.
    #[test]
    fn div_csr_ready_is_zero_on_rp2350() {
        let mut sio = Sio::new();
        let core = 0;
        // Trigger an unsigned divide — same writes as the silicon scenario.
        sio.write32(0x060, 100, core); // DIV_UDIVIDEND
        sio.write32(0x064, 7, core);   // DIV_UDIVISOR
        // READY (bit 0) must be 0; DIRTY (bit 1) may be set.
        assert_eq!(sio.read32(0x078, core) & 0x1, 0x0,
            "DIV_CSR.READY must be 0 on RP2350 (reserved register)");
    }

    /// `sio_divider_signed` oracle: same as unsigned — READY=0.
    #[test]
    fn div_csr_ready_is_zero_after_signed_divide() {
        let mut sio = Sio::new();
        let core = 0;
        sio.write32(0x068, 0xFFFF_FF9C, core); // DIV_SDIVIDEND (-100)
        sio.write32(0x06C, 7, core);            // DIV_SDIVISOR
        assert_eq!(sio.read32(0x078, core) & 0x1, 0x0,
            "DIV_CSR.READY must be 0 on RP2350 (reserved register)");
    }

    /// `sio_mtime_count_and_match` oracle: RISCV_SOFTIRQ at 0x1A0 is
    /// write-to-set; writing 1 sets bit 0 (CORE0_SET) permanently until
    /// cleared via CORE0_CLR (bit 1). Writing 0 is a no-op (no bits set).
    #[test]
    fn riscv_softirq_write_to_set_semantics() {
        let mut sio = Sio::new();
        // Initial state: both flags clear.
        assert_eq!(sio.read32(0x1A0, 0), 0x0);
        // Write 1 — sets CORE0_SET (bit 0).
        sio.write32(0x1A0, 1, 0);
        assert_eq!(sio.read32(0x1A0, 0), 0x1,
            "RISCV_SOFTIRQ: writing 1 must set CORE0_SET bit");
        // Write 0 — no-op; bit 0 stays set.
        sio.write32(0x1A0, 0, 0);
        assert_eq!(sio.read32(0x1A0, 0), 0x1,
            "RISCV_SOFTIRQ: writing 0 must not clear any flag");
    }

    /// RISCV_SOFTIRQ write 1 then write 0: bit 0 remains 1.
    /// This matches what silicon observes in the `sio_mtime_count_and_match`
    /// sled (STR r1=1 → STR r1=0 → BKPT → read back 0x1).
    #[test]
    fn riscv_softirq_set_then_zero_stays_set() {
        let mut sio = Sio::new();
        sio.write32(0x1A0, 1, 0);
        sio.write32(0x1A0, 0, 0);
        assert_eq!(sio.read32(0x1A0, 0) & 0x1, 0x1,
            "RISCV_SOFTIRQ.CORE0_SET must stay 1 after writing 0 (only CORE0_CLR=bit1 clears it)");
    }

    /// RISCV_SOFTIRQ: write 0x100 (CORE0_CLR bit [8]) clears CORE0_SET (bit [0]).
    /// Per RP2350 datasheet Table 79 §3.1: bits [9:8] are write-1-to-clear control.
    #[test]
    fn riscv_softirq_write_to_clear_via_bit_8_clears_core0() {
        let mut sio = Sio::new();
        // Step 1: set CORE0 via bit [0].
        sio.write32(0x1A0, 0x01, 0);
        assert_eq!(sio.read32(0x1A0, 0), 0x01, "CORE0_SET should be 1 after write 0x01");
        // Step 2: clear CORE0 via bit [8] (CORE0_CLR).
        sio.write32(0x1A0, 0x100, 0);
        assert_eq!(sio.read32(0x1A0, 0), 0x00, "CORE0_SET should be 0 after CORE0_CLR write 0x100");
    }

    /// RISCV_SOFTIRQ simultaneous SET and CLR: set wins per datasheet.
    /// Writing 0x101 (bit [0] = CORE0_SET, bit [8] = CORE0_CLR) must leave
    /// CORE0_SET = 1 (set takes priority over clear on the same cycle).
    #[test]
    fn riscv_softirq_simultaneous_set_and_clear_set_wins() {
        let mut sio = Sio::new();
        // Write with both CORE0_SET (bit 0) and CORE0_CLR (bit 8) asserted.
        sio.write32(0x1A0, 0x101, 0);
        assert_eq!(sio.read32(0x1A0, 0), 0x01,
            "RISCV_SOFTIRQ: simultaneous SET+CLR must leave the flag set (set wins)");
    }

    // ---- MTIME / FULLSPEED vs TICKS.RISCV tests (Residual A.2.1) ----
    //
    // HLD `2026.04.17 - HLD - Residual A.2.1 MTIME WATCHDOG_TICK Fix.md`:
    // MTIME must not advance from sys_clks in the default FULLSPEED=0 mode
    // unless TICKS.RISCV is configured to emit edges. These three tests
    // pin the MTIME_CTRL.FULLSPEED and RISCV-edges semantics of the new
    // `tick_mtime_from_ticks` entry point.

    /// Post-reset MTIME_CTRL=0x0D (EN=1, FULLSPEED=0, DBGPAUSE*=1).
    /// Pumping sys_clks with zero RISCV edges must leave MTIME at zero —
    /// matches silicon.
    #[test]
    fn mtime_post_reset_does_not_count_from_sys_clks_alone() {
        let mut sio = Sio::new();
        assert_eq!(sio.mtime_ctrl, 0x0D);
        sio.tick_mtime_from_ticks(0, 1000);
        assert_eq!(sio.mtime, 0,
            "FULLSPEED=0 + zero RISCV edges must not advance MTIME");
    }

    /// In the default TICKS mode (FULLSPEED=0), MTIME advances by one per
    /// RISCV edge and ignores sys_clks entirely.
    #[test]
    fn mtime_advances_one_per_riscv_edge_in_ticks_mode() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 0x01; // EN=1, FULLSPEED=0.
        sio.tick_mtime_from_ticks(5, 0);
        assert_eq!(sio.mtime, 5, "5 RISCV edges -> MTIME += 5 in TICKS mode");
        sio.tick_mtime_from_ticks(0, 1000);
        assert_eq!(sio.mtime, 5, "sys_clks alone never advance in TICKS mode");
    }

    /// FULLSPEED=1 counts sys_clks directly and ignores RISCV edges
    /// entirely (no double-count).
    #[test]
    fn mtime_fullspeed_counts_sys_clks_directly() {
        let mut sio = Sio::new();
        sio.mtime_ctrl = 0x03; // EN=1 + FULLSPEED=1.
        sio.tick_mtime_from_ticks(0, 100);
        assert_eq!(sio.mtime, 100, "FULLSPEED=1 -> MTIME counts sys_clks");
        // RISCV edges ignored in FULLSPEED mode.
        sio.tick_mtime_from_ticks(9, 50);
        assert_eq!(sio.mtime, 150, "edges ignored when FULLSPEED=1");
    }
}

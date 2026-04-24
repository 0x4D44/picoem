//! Shared SIO state for threaded execution.
//!
//! Exposes the cross-core SIO surface — inter-core FIFOs, spinlocks,
//! doorbells, machine timer — as a set of atomic-backed accessors. Per-core
//! state (divider, interpolators) is NOT carried here: those live on the
//! per-core bus view in Phase 3 and never need inter-thread synchronisation.
//!
//! See `wrk_docs/2026.04.17 - LLD - Threaded Dual-Core Phase 2 V4.md` §3.
//!
//! ## Phase split
//!
//! Phase 2 publishes the primitive: methods return/mutate state without
//! reaching into the NVIC or the `event_flag` used by WFE wake-up. Phase 3
//! is responsible for:
//! - signalling `event_flag` after a successful FIFO push (WFE wake-up),
//! - asserting MTIME-match → NVIC IRQ,
//! - propagating doorbell writes to NVIC at the quantum boundary.
//!
//! ## Memory ordering
//!
//! Spinlock claim/release use Acquire/Release so lock acquisition
//! publishes subsequent critical-section writes to the peer; everything
//! else is Relaxed (per-field reads/writes have no cross-field ordering
//! requirement, and the enclosing barrier in Phase 3 provides the
//! happens-before edge at quantum boundaries).

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering::*};

use super::SpscQueue;

/// Inter-core FIFO depth on real RP2350 silicon.
const FIFO_CAPACITY: u32 = 8;

// FIFO_ST register layout (matches RP2350 datasheet §3.1.4.4).
const FIFO_ST_VLD: u32 = 1 << 0; // RX FIFO not empty
const FIFO_ST_RDY: u32 = 1 << 1; // TX FIFO has room
const FIFO_ST_WOF: u32 = 1 << 2; // sticky: TX write when full
const FIFO_ST_ROE: u32 = 1 << 3; // sticky: RX read when empty

/// Doorbell register is 4 bits wide (one bit per doorbell, 4 per core).
const DOORBELL_MASK: u32 = 0xF;

/// Shared SIO state for the two CPU cores.
///
/// Constructed once, shared via `Arc` across worker threads. All methods
/// take `&self`.
pub struct ThreadedSio {
    // Inter-core FIFOs (capacity 8, matching hardware).
    fifo_0to1: SpscQueue, // core 0 produces, core 1 consumes
    fifo_1to0: SpscQueue, // core 1 produces, core 0 consumes

    // Sticky FIFO error flags per core.
    fifo_roe: [AtomicBool; 2], // read-overflow (pop empty)
    fifo_wof: [AtomicBool; 2], // write-overflow (push full)

    // Spinlocks: single 32-bit bitmask, one bit per lock.
    spinlocks: AtomicU32,

    // Doorbells: 4-bit pending field per core.
    doorbells: [AtomicU32; 2],

    // Machine timer (coordinator advances, cores read via MMIO).
    mtime: AtomicU64,
    mtime_ctrl: AtomicU32, // bit 0 = enable
    mtimecmp: [AtomicU64; 2],

    /// Per-core edge-triggered MTIMECMP match latch.
    ///
    /// Phase 3 seeds this from `Sio::mtime_match_asserted` so the edge
    /// state survives the single → threaded handoff, but does **not**
    /// consume it — MTIMECMP → IRQ wiring is deferred to Phase 5 per
    /// LLD V7 §14. Exposed via `mtime_match_asserted_load` /
    /// `_store` for the forthcoming Phase 5 coordinator tick.
    mtime_match_asserted: [AtomicBool; 2],
}

impl ThreadedSio {
    /// Construct a zero-initialised `ThreadedSio`.
    pub fn new() -> Self {
        Self {
            fifo_0to1: SpscQueue::new(FIFO_CAPACITY),
            fifo_1to0: SpscQueue::new(FIFO_CAPACITY),
            fifo_roe: [AtomicBool::new(false), AtomicBool::new(false)],
            fifo_wof: [AtomicBool::new(false), AtomicBool::new(false)],
            spinlocks: AtomicU32::new(0),
            doorbells: [AtomicU32::new(0), AtomicU32::new(0)],
            mtime: AtomicU64::new(0),
            mtime_ctrl: AtomicU32::new(0),
            mtimecmp: [AtomicU64::new(0), AtomicU64::new(0)],
            mtime_match_asserted: [AtomicBool::new(false), AtomicBool::new(false)],
        }
    }

    /// Read the per-core MTIMECMP match-latch flag. Phase 5 coordinator
    /// tick will observe this; Phase 3 only seeds the initial bit.
    pub fn mtime_match_asserted_load(&self, core: usize) -> bool {
        debug_assert!(core < 2);
        self.mtime_match_asserted[core].load(Relaxed)
    }

    /// Write the per-core MTIMECMP match-latch flag. Phase 3 uses this
    /// only during `seed`; Phase 5 wires it to the coordinator.
    pub fn mtime_match_asserted_store(&self, core: usize, val: bool) {
        debug_assert!(core < 2);
        self.mtime_match_asserted[core].store(val, Relaxed);
    }

    // --- FIFO ---

    /// Push to the peer core's RX FIFO. Sets WOF on failure.
    ///
    /// Returns true on success, false if the peer's RX FIFO is full.
    pub fn fifo_push(&self, writing_core: usize, val: u32) -> bool {
        debug_assert!(writing_core < 2);
        let q = match writing_core {
            0 => &self.fifo_0to1,
            _ => &self.fifo_1to0,
        };
        if q.try_push(val) {
            // NOTE: event_flag signalling for WFE wake-up is Phase 3's
            // responsibility — the caller (WorkerBus) sets it.
            true
        } else {
            self.fifo_wof[writing_core].store(true, Relaxed);
            false
        }
    }

    /// Pop from the reading core's own RX FIFO. Sets ROE on failure.
    pub fn fifo_pop(&self, reading_core: usize) -> Option<u32> {
        debug_assert!(reading_core < 2);
        let q = match reading_core {
            0 => &self.fifo_1to0,
            _ => &self.fifo_0to1,
        };
        match q.try_pop() {
            Some(val) => Some(val),
            None => {
                self.fifo_roe[reading_core].store(true, Relaxed);
                None
            }
        }
    }

    /// Build FIFO_ST register value for the given core.
    ///
    /// Bit 0: VLD (own RX has data), bit 1: RDY (TX has room),
    /// bit 2: WOF (sticky), bit 3: ROE (sticky).
    pub fn fifo_st(&self, core: usize) -> u32 {
        debug_assert!(core < 2);
        let (read_q, write_q) = match core {
            0 => (&self.fifo_1to0, &self.fifo_0to1),
            _ => (&self.fifo_0to1, &self.fifo_1to0),
        };
        let vld = if !read_q.is_empty() { FIFO_ST_VLD } else { 0 };
        let rdy = if !write_q.is_full() { FIFO_ST_RDY } else { 0 };
        let wof = if self.fifo_wof[core].load(Relaxed) { FIFO_ST_WOF } else { 0 };
        let roe = if self.fifo_roe[core].load(Relaxed) { FIFO_ST_ROE } else { 0 };
        vld | rdy | wof | roe
    }

    /// Clear sticky FIFO_ST bits (write-1-to-clear).
    ///
    /// Bits 0/1 (VLD/RDY) are read-only and silently ignored, matching
    /// hardware. Bits 2 (WOF) and 3 (ROE) are cleared when set in `mask`.
    pub fn fifo_st_clear(&self, core: usize, mask: u32) {
        debug_assert!(core < 2);
        if mask & FIFO_ST_WOF != 0 {
            self.fifo_wof[core].store(false, Relaxed);
        }
        if mask & FIFO_ST_ROE != 0 {
            self.fifo_roe[core].store(false, Relaxed);
        }
    }

    // --- Spinlocks ---

    /// Claim a spinlock via atomic test-and-set (hardware READ semantics).
    ///
    /// Returns `1 << id` on success, `0` if the lock was already held.
    pub fn spinlock_claim(&self, id: usize) -> u32 {
        debug_assert!(id < 32);
        let mask = 1u32 << id;
        let prev = self.spinlocks.fetch_or(mask, Acquire);
        if prev & mask == 0 {
            mask
        } else {
            0
        }
    }

    /// Release a spinlock. Hardware ignores the written value, so the API
    /// omits it.
    pub fn spinlock_release(&self, id: usize) {
        debug_assert!(id < 32);
        self.spinlocks.fetch_and(!(1u32 << id), Release);
    }

    /// Read the raw 32-lock bitmask (serves SPINLOCK_ST MMIO reads).
    pub fn spinlock_bits(&self) -> u32 {
        self.spinlocks.load(Relaxed)
    }

    // --- Doorbells ---

    /// Set one or more doorbell bits for `core`.
    ///
    /// Bits outside the 4-bit doorbell field are silently dropped.
    pub fn doorbell_set(&self, core: usize, bits: u32) {
        debug_assert!(core < 2);
        self.doorbells[core].fetch_or(bits & DOORBELL_MASK, Relaxed);
    }

    /// Clear one or more doorbell bits for `core`.
    pub fn doorbell_clear(&self, core: usize, bits: u32) {
        debug_assert!(core < 2);
        self.doorbells[core].fetch_and(!(bits & DOORBELL_MASK), Relaxed);
    }

    /// Read the 4-bit doorbell field for `core`.
    pub fn doorbell_read(&self, core: usize) -> u32 {
        debug_assert!(core < 2);
        self.doorbells[core].load(Relaxed) & DOORBELL_MASK
    }

    // --- MTIME ---

    /// Read the 64-bit machine timer.
    pub fn mtime_read(&self) -> u64 {
        self.mtime.load(Relaxed)
    }

    /// Write the machine timer directly (firmware init via MMIO).
    pub fn mtime_write(&self, val: u64) {
        self.mtime.store(val, Relaxed);
    }

    /// Advance MTIME by `cycles`. MUST be called only by the coordinator
    /// thread. No-op when `mtime_ctrl` bit 0 is clear.
    pub fn mtime_advance(&self, cycles: u64) {
        if self.mtime_ctrl.load(Relaxed) & 1 != 0 {
            self.mtime.fetch_add(cycles, Relaxed);
        }
    }

    /// Read the MTIME_CTRL register.
    pub fn mtime_ctrl_read(&self) -> u32 {
        self.mtime_ctrl.load(Relaxed)
    }

    /// Write the MTIME_CTRL register.
    pub fn mtime_ctrl_write(&self, val: u32) {
        self.mtime_ctrl.store(val, Relaxed);
    }

    /// Read per-core MTIMECMP.
    pub fn mtimecmp_read(&self, core: usize) -> u64 {
        debug_assert!(core < 2);
        self.mtimecmp[core].load(Relaxed)
    }

    /// Write per-core MTIMECMP.
    pub fn mtimecmp_write(&self, core: usize, val: u64) {
        debug_assert!(core < 2);
        self.mtimecmp[core].store(val, Relaxed);
    }

    /// True when `mtime >= mtimecmp[core]` — the match condition that
    /// Phase 3 will latch into the NVIC.
    pub fn mtime_check_match(&self, core: usize) -> bool {
        debug_assert!(core < 2);
        self.mtime.load(Relaxed) >= self.mtimecmp[core].load(Relaxed)
    }

    // --- Reset ---

    /// Reset all shared SIO state.
    ///
    /// Only safe during emulator reset (coordinator phase, no concurrent
    /// access). Clears both FIFOs, all sticky flags, doorbells,
    /// spinlocks, MTIME, MTIME_CTRL, and both MTIMECMP registers.
    pub fn reset(&self) {
        self.fifo_0to1.clear();
        self.fifo_1to0.clear();
        for i in 0..2 {
            self.fifo_roe[i].store(false, Relaxed);
            self.fifo_wof[i].store(false, Relaxed);
            self.doorbells[i].store(0, Relaxed);
            self.mtimecmp[i].store(0, Relaxed);
            self.mtime_match_asserted[i].store(false, Relaxed);
        }
        self.spinlocks.store(0, Relaxed);
        self.mtime.store(0, Relaxed);
        self.mtime_ctrl.store(0, Relaxed);
    }
}

impl ThreadedSio {
    /// Seed a fresh `ThreadedSio` from an existing single-threaded
    /// `Sio` — copying FIFO contents, sticky FIFO-error flags, the 32
    /// spinlock claim bitmask, per-core doorbell pending bits, MTIME /
    /// MTIME_CTRL / both MTIMECMP. Does NOT seed `gpio_out` / `gpio_oe`
    /// — those live on `AtomicGpio` in the threaded runtime.
    ///
    /// Phase 3 Stage 6b (LLD V7 §6/§8): called from
    /// `ThreadedEmulator::from_emulator` so cross-core state survives
    /// the single → threaded-runtime handoff.
    pub fn seed(sio: &crate::sio::Sio) -> Self {
        let out = Self::new();
        // FIFOs: re-push the single-threaded Fifo contents (head→tail
        // order) into the SPSC rings. The snapshots are bounded by the
        // 8-entry `Fifo` capacity so the pushes cannot overflow the
        // SPSC queues (which are the same capacity).
        for val in sio.fifo_0to1_snapshot() {
            let _ = out.fifo_0to1.try_push(val);
        }
        for val in sio.fifo_1to0_snapshot() {
            let _ = out.fifo_1to0.try_push(val);
        }
        for core in 0..2 {
            out.fifo_wof[core].store(sio.fifo_wof(core), Relaxed);
            out.fifo_roe[core].store(sio.fifo_roe(core), Relaxed);
            out.doorbells[core].store(sio.doorbell_pending[core] as u32, Relaxed);
            out.mtimecmp[core].store(sio.mtimecmp[core], Relaxed);
            // Phase 3 preserves the edge-latch state for the Phase 5
            // MTIMECMP → IRQ wiring (LLD V7 §14); not consumed yet.
            out.mtime_match_asserted[core].store(sio.mtime_match_asserted[core], Relaxed);
        }
        out.spinlocks.store(sio.spinlock_bits(), Relaxed);
        out.mtime.store(sio.mtime, Relaxed);
        out.mtime_ctrl.store(sio.mtime_ctrl, Relaxed);
        out
    }
}

impl Default for ThreadedSio {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_push_pop() {
        let sio = ThreadedSio::new();
        assert!(sio.fifo_push(0, 0xDEAD_BEEF));
        assert!(sio.fifo_push(0, 0xCAFE_F00D));
        assert_eq!(sio.fifo_pop(1), Some(0xDEAD_BEEF));
        assert_eq!(sio.fifo_pop(1), Some(0xCAFE_F00D));
    }

    #[test]
    fn fifo_reverse() {
        let sio = ThreadedSio::new();
        assert!(sio.fifo_push(1, 0x1111_2222));
        assert!(sio.fifo_push(1, 0x3333_4444));
        assert_eq!(sio.fifo_pop(0), Some(0x1111_2222));
        assert_eq!(sio.fifo_pop(0), Some(0x3333_4444));
    }

    #[test]
    fn fifo_full_sets_wof() {
        let sio = ThreadedSio::new();
        // Fill the 8-entry FIFO from core 0 -> core 1.
        for i in 0..8u32 {
            assert!(sio.fifo_push(0, i), "push {i} must succeed");
        }
        // Ninth push must fail and set WOF on the writing core.
        assert!(!sio.fifo_push(0, 99));
        assert_eq!(sio.fifo_st(0) & FIFO_ST_WOF, FIFO_ST_WOF);
        // The reading side's WOF is unaffected.
        assert_eq!(sio.fifo_st(1) & FIFO_ST_WOF, 0);
    }

    #[test]
    fn fifo_empty_sets_roe() {
        let sio = ThreadedSio::new();
        // Core 0 pops from its own (empty) RX.
        assert_eq!(sio.fifo_pop(0), None);
        assert_eq!(sio.fifo_st(0) & FIFO_ST_ROE, FIFO_ST_ROE);
        // The non-reading side's ROE is unaffected.
        assert_eq!(sio.fifo_st(1) & FIFO_ST_ROE, 0);
    }

    #[test]
    fn fifo_st_reflects_state() {
        let sio = ThreadedSio::new();
        // Nothing pushed: VLD=0 on both sides, RDY=1 on both sides.
        let st0 = sio.fifo_st(0);
        let st1 = sio.fifo_st(1);
        assert_eq!(st0 & FIFO_ST_VLD, 0);
        assert_eq!(st1 & FIFO_ST_VLD, 0);
        assert_eq!(st0 & FIFO_ST_RDY, FIFO_ST_RDY);
        assert_eq!(st1 & FIFO_ST_RDY, FIFO_ST_RDY);

        // Core 0 pushes once: core 1's RX has data (VLD), core 0 still
        // has room (RDY).
        assert!(sio.fifo_push(0, 42));
        let st0 = sio.fifo_st(0);
        let st1 = sio.fifo_st(1);
        assert_eq!(st0 & FIFO_ST_VLD, 0, "core 0 RX still empty");
        assert_eq!(st1 & FIFO_ST_VLD, FIFO_ST_VLD, "core 1 RX has one entry");
        assert_eq!(st0 & FIFO_ST_RDY, FIFO_ST_RDY, "core 0 TX has room");

        // Fill the FIFO from core 0. Core 0's TX (fifo_0to1) becomes full
        // so RDY drops; core 1's RX is VLD.
        for _ in 1..8 {
            assert!(sio.fifo_push(0, 0));
        }
        let st0 = sio.fifo_st(0);
        let st1 = sio.fifo_st(1);
        assert_eq!(st0 & FIFO_ST_RDY, 0, "core 0 TX full so RDY cleared");
        assert_eq!(st1 & FIFO_ST_VLD, FIFO_ST_VLD);
    }

    #[test]
    fn fifo_st_clear() {
        let sio = ThreadedSio::new();

        // Produce both sticky conditions on core 0.
        assert_eq!(sio.fifo_pop(0), None); // sets ROE on core 0
        for _ in 0..8 {
            sio.fifo_push(0, 0);
        }
        assert!(!sio.fifo_push(0, 0)); // sets WOF on core 0

        let st = sio.fifo_st(0);
        assert_eq!(st & FIFO_ST_WOF, FIFO_ST_WOF);
        assert_eq!(st & FIFO_ST_ROE, FIFO_ST_ROE);

        // Clear ROE only — WOF must remain.
        sio.fifo_st_clear(0, FIFO_ST_ROE);
        let st = sio.fifo_st(0);
        assert_eq!(st & FIFO_ST_ROE, 0);
        assert_eq!(st & FIFO_ST_WOF, FIFO_ST_WOF);

        // Clear WOF too.
        sio.fifo_st_clear(0, FIFO_ST_WOF);
        let st = sio.fifo_st(0);
        assert_eq!(st & FIFO_ST_WOF, 0);
        assert_eq!(st & FIFO_ST_ROE, 0);

        // Writing the VLD/RDY bits is silently ignored even if the user
        // asks for it (bits 0/1 are RO).
        sio.fifo_st_clear(0, FIFO_ST_VLD | FIFO_ST_RDY);
    }

    #[test]
    fn spinlock_claim_release() {
        let sio = ThreadedSio::new();
        // First claim of lock 7 succeeds.
        assert_eq!(sio.spinlock_claim(7), 1 << 7);
        // Repeated claim fails while held.
        assert_eq!(sio.spinlock_claim(7), 0);
        // Spinlock 0 is independent.
        assert_eq!(sio.spinlock_claim(0), 1);
    }

    #[test]
    fn spinlock_release_allows_reclaim() {
        let sio = ThreadedSio::new();
        assert_eq!(sio.spinlock_claim(5), 1 << 5);
        assert_eq!(sio.spinlock_claim(5), 0);
        sio.spinlock_release(5);
        assert_eq!(sio.spinlock_claim(5), 1 << 5);
    }

    #[test]
    fn doorbell_set_read_clear() {
        let sio = ThreadedSio::new();

        // Bits outside the 4-bit mask are dropped.
        sio.doorbell_set(0, 0xFF);
        assert_eq!(sio.doorbell_read(0), 0xF);
        // Core 1 untouched.
        assert_eq!(sio.doorbell_read(1), 0);

        // Partial clear.
        sio.doorbell_clear(0, 0x5);
        assert_eq!(sio.doorbell_read(0), 0xA);

        // Clearing upper bits is a no-op on the 4-bit field.
        sio.doorbell_clear(0, 0xF0);
        assert_eq!(sio.doorbell_read(0), 0xA);

        // Full clear.
        sio.doorbell_clear(0, 0xF);
        assert_eq!(sio.doorbell_read(0), 0);
    }

    #[test]
    fn mtime_advance_respects_enable() {
        let sio = ThreadedSio::new();

        // Disabled: advance is a no-op.
        sio.mtime_advance(10);
        assert_eq!(sio.mtime_read(), 0);

        // Enabled: advance increments.
        sio.mtime_ctrl_write(1);
        sio.mtime_advance(10);
        assert_eq!(sio.mtime_read(), 10);
        sio.mtime_advance(25);
        assert_eq!(sio.mtime_read(), 35);

        // Disabling freezes the counter.
        sio.mtime_ctrl_write(0);
        sio.mtime_advance(100);
        assert_eq!(sio.mtime_read(), 35);
    }

    #[test]
    fn mtime_compare() {
        let sio = ThreadedSio::new();

        // mtimecmp defaults to 0, mtime is 0 → match.
        assert!(sio.mtime_check_match(0));
        assert!(sio.mtime_check_match(1));

        // Raise mtimecmp past current mtime — no match.
        sio.mtimecmp_write(0, 100);
        assert!(!sio.mtime_check_match(0));
        // Core 1's compare is independent.
        assert!(sio.mtime_check_match(1));

        // Advance mtime to the compare value: match fires.
        sio.mtime_ctrl_write(1);
        sio.mtime_advance(100);
        assert!(sio.mtime_check_match(0));

        // Overshoot also matches.
        sio.mtime_advance(50);
        assert!(sio.mtime_check_match(0));
    }

    #[test]
    fn reset_clears_all_state() {
        let sio = ThreadedSio::new();

        // Push into both FIFOs.
        sio.fifo_push(0, 1);
        sio.fifo_push(1, 2);

        // Explicitly trigger WOF on core 0 by pushing past capacity (8).
        // Iterate enough times that the FIFO fills and further pushes
        // set WOF.
        for _ in 0..20 {
            sio.fifo_push(0, 0);
        }
        // Explicitly trigger WOF on core 1 the same way.
        for _ in 0..20 {
            sio.fifo_push(1, 0);
        }

        // Trigger ROE on both cores by popping from empty... but since
        // we've pushed data above, drain first to get to empty. Simpler:
        // bypass via a direct sticky-flag check after reset. To force
        // ROE pre-reset, drain both FIFOs then pop again.
        while sio.fifo_pop(0).is_some() {}
        while sio.fifo_pop(1).is_some() {}
        let _ = sio.fifo_pop(0); // empty → ROE on core 0
        let _ = sio.fifo_pop(1); // empty → ROE on core 1

        // Confirm sticky bits are set pre-reset so we know reset has
        // real work to do.
        assert_eq!(sio.fifo_st(0) & FIFO_ST_WOF, FIFO_ST_WOF);
        assert_eq!(sio.fifo_st(1) & FIFO_ST_WOF, FIFO_ST_WOF);
        assert_eq!(sio.fifo_st(0) & FIFO_ST_ROE, FIFO_ST_ROE);
        assert_eq!(sio.fifo_st(1) & FIFO_ST_ROE, FIFO_ST_ROE);

        // Populate the remaining shared state.
        sio.spinlock_claim(3);
        sio.spinlock_claim(17);
        sio.doorbell_set(0, 0xF);
        sio.doorbell_set(1, 0xF);
        sio.mtime_ctrl_write(1);
        sio.mtime_write(999);
        sio.mtimecmp_write(0, 111);
        sio.mtimecmp_write(1, 222);

        // Push something back into both FIFOs so reset has data to drop.
        sio.fifo_push(0, 7);
        sio.fifo_push(1, 8);

        sio.reset();

        // FIFOs are empty (VLD=0), have room (RDY=1), sticky bits cleared.
        assert_eq!(sio.fifo_st(0), FIFO_ST_RDY, "only RDY is set after reset");
        assert_eq!(sio.fifo_st(1), FIFO_ST_RDY);
        assert_eq!(sio.spinlock_bits(), 0);
        assert_eq!(sio.doorbell_read(0), 0);
        assert_eq!(sio.doorbell_read(1), 0);
        assert_eq!(sio.mtime_read(), 0);
        assert_eq!(sio.mtime_ctrl_read(), 0);
        assert_eq!(sio.mtimecmp_read(0), 0);
        assert_eq!(sio.mtimecmp_read(1), 0);
    }

    #[test]
    fn mmio_accessors_roundtrip() {
        let sio = ThreadedSio::new();

        // MTIME direct write/read.
        sio.mtime_write(0xDEAD_BEEF_CAFE_F00D);
        assert_eq!(sio.mtime_read(), 0xDEAD_BEEF_CAFE_F00D);

        // MTIME_CTRL write/read.
        sio.mtime_ctrl_write(0xA5);
        assert_eq!(sio.mtime_ctrl_read(), 0xA5);

        // MTIMECMP per-core.
        sio.mtimecmp_write(0, 0x1111_2222_3333_4444);
        sio.mtimecmp_write(1, 0x5555_6666_7777_8888);
        assert_eq!(sio.mtimecmp_read(0), 0x1111_2222_3333_4444);
        assert_eq!(sio.mtimecmp_read(1), 0x5555_6666_7777_8888);

        // spinlock_bits reflects the current claim state.
        assert_eq!(sio.spinlock_bits(), 0);
        sio.spinlock_claim(0);
        sio.spinlock_claim(31);
        assert_eq!(sio.spinlock_bits(), (1 << 0) | (1u32 << 31));
        sio.spinlock_release(0);
        assert_eq!(sio.spinlock_bits(), 1u32 << 31);
        sio.spinlock_release(31);
        assert_eq!(sio.spinlock_bits(), 0);
    }
}

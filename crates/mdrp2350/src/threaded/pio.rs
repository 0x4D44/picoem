//! Shared PIO state for threaded execution.
//!
//! The PioBlock execution engine (SM stepping) stays on the PIO worker
//! thread — only communication interfaces are shared across threads here.
//!
//! See `wrk_docs/2026.04.17 - LLD - Threaded Dual-Core Phase 2 V4.md` §4.
//!
//! ## Layout
//!
//! - **TX FIFOs** `tx[BLOCK][SM]`: CPU pushes (`tx_push`), PIO thread pops
//!   (`tx_pop`). SPSC direction is CPU → PIO.
//! - **RX FIFOs** `rx[BLOCK][SM]`: PIO thread pushes (`rx_push`), CPU pops
//!   (`rx_pop`). SPSC direction is PIO → CPU.
//! - **Atomic control**: `sm_enabled`, `irq_flags`, `dreq` — one byte per
//!   PIO block, touched from multiple threads with Relaxed ordering.
//! - **Command queue**: Mutex-guarded `Vec<PioCommand>` for cold-path
//!   firmware setup (CPU → PIO).
//!
//! All SpscQueues are constructed at capacity 4. FIFO join (depth 8) is
//! deferred to Phase 3 — join is a setup-time operation and the Phase 3
//! coordinator can construct new queues during a barrier-protected pause.
//!
//! ## Cross-core ordering
//!
//! Single-core MMIO ordering is preserved (each CPU thread calls
//! `send_command` sequentially). Cross-core ordering is NOT guaranteed —
//! concurrent writes to the same PIO register from two cores serialize
//! arbitrarily through the Mutex, which matches real hardware semantics.

use super::spsc::SpscQueue;
use std::sync::atomic::{AtomicU8, Ordering::Relaxed};
use std::sync::Mutex;

pub const PIO_BLOCKS: usize = 3;
pub const SMS_PER_BLOCK: usize = 4;
pub const PIO_FIFO_DEPTH: u32 = 4;

pub struct ThreadedPio {
    // TX FIFOs: CPU pushes, PIO thread pops
    tx: [[SpscQueue; SMS_PER_BLOCK]; PIO_BLOCKS],

    // RX FIFOs: PIO thread pushes, CPU pops
    rx: [[SpscQueue; SMS_PER_BLOCK]; PIO_BLOCKS],

    // Atomic control
    sm_enabled: [AtomicU8; PIO_BLOCKS],
    irq_flags: [AtomicU8; PIO_BLOCKS],
    dreq: [AtomicU8; PIO_BLOCKS],

    // Cold-path command queue (CPU → PIO thread)
    commands: Mutex<Vec<PioCommand>>,
}

/// Cold-path commands sent from CPU workers to the PIO thread.
///
/// Phase 2 seeded the queue with `WriteInstrMem` / `SetClkDiv`; Phase 3
/// task #11 added `WriteCtrl` (SM enable / restart / clkdiv-restart —
/// the critical unblocker so `ThreadedPio::read_sm_enabled` reflects
/// firmware-programmed state) and a general-purpose `WriteReg` arm that
/// covers every remaining PIO register offset the single-threaded
/// `Bus::write32` hands to `PioBlock::write32`: TXF0..TXF3, FDEBUG,
/// IRQ, IRQ_FORCE, INPUT_SYNC_BYPASS, per-SM EXECCTRL/SHIFTCTRL/
/// INSTR/PINCTRL.
///
/// `WriteInstrMem` and `SetClkDiv` are kept as purpose-built variants
/// for backward compatibility with existing tests and for the slightly
/// cheaper dispatch path (no sub-offset decode in the worker). The
/// generic `WriteReg` variant is the fallback used by `WorkerBus` for
/// anything outside those two fast paths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PioCommand {
    /// INSTR_MEM slot write. `alias` (0=normal, 1=XOR, 2=OR, 3=AND-NOT)
    /// is propagated to `PioBlock::write32` so firmware that uses the
    /// aliased MMIO regions (e.g. SET/CLR/XOR ROM patching) produces
    /// the same memory contents as the single-threaded `Bus` path.
    WriteInstrMem { block: u8, addr: u8, value: u16, alias: u8 },
    /// SMn_CLKDIV write. `alias` is propagated through to
    /// `PioBlock::write32` for parity with the single-threaded `Bus`
    /// path, which forwards the 2-bit alias encoded in the upper MMIO
    /// address bits to `PioBlock::write32` unconditionally.
    SetClkDiv { block: u8, sm: u8, int_div: u16, frac_div: u8, alias: u8 },
    /// CTRL (0x000) write: SM_ENABLE / SM_RESTART / CLKDIV_RESTART.
    /// After applying, the PIO worker publishes the resulting
    /// `sm_enabled_mask` onto `ThreadedPio::sm_enabled` so CPU-side
    /// reads observe the new state.
    WriteCtrl { block: u8, val: u32, alias: u8 },
    /// Generic register write — dispatched to `PioBlock::write32` as-is.
    /// Covers TXF0..TXF3, FDEBUG, IRQ, IRQ_FORCE, INPUT_SYNC_BYPASS,
    /// per-SM EXECCTRL / SHIFTCTRL / INSTR / PINCTRL, and any PIO offset
    /// the two purpose-built variants above do not route.
    WriteReg { block: u8, offset: u16, val: u32, alias: u8 },
}

impl ThreadedPio {
    pub fn new() -> Self {
        Self {
            tx: std::array::from_fn(|_| {
                std::array::from_fn(|_| SpscQueue::new(PIO_FIFO_DEPTH))
            }),
            rx: std::array::from_fn(|_| {
                std::array::from_fn(|_| SpscQueue::new(PIO_FIFO_DEPTH))
            }),
            sm_enabled: std::array::from_fn(|_| AtomicU8::new(0)),
            irq_flags: std::array::from_fn(|_| AtomicU8::new(0)),
            dreq: std::array::from_fn(|_| AtomicU8::new(0)),
            // Preallocate for the common setup-heavy case (INSTR_MEM 32
            // slots × up to 3 blocks = 96 writes + per-SM setup). Keeps
            // the first-quantum firmware init path from thrashing the
            // allocator through the push path. Subsequent quanta recycle
            // this capacity via `drain_commands`.
            commands: Mutex::new(Vec::with_capacity(64)),
        }
    }

    // --- CPU-side FIFO ---

    /// CPU pushes to a TX FIFO. Returns false if the FIFO is full.
    pub fn tx_push(&self, block: usize, sm: usize, val: u32) -> bool {
        debug_assert!(block < PIO_BLOCKS);
        debug_assert!(sm < SMS_PER_BLOCK);
        self.tx[block][sm].try_push(val)
    }

    /// CPU pops from an RX FIFO. Returns `None` if empty.
    pub fn rx_pop(&self, block: usize, sm: usize) -> Option<u32> {
        debug_assert!(block < PIO_BLOCKS);
        debug_assert!(sm < SMS_PER_BLOCK);
        self.rx[block][sm].try_pop()
    }

    /// TX FIFO occupancy (for FSTAT / FDEBUG MMIO).
    pub fn tx_level(&self, block: usize, sm: usize) -> u32 {
        debug_assert!(block < PIO_BLOCKS);
        debug_assert!(sm < SMS_PER_BLOCK);
        self.tx[block][sm].len()
    }

    /// RX FIFO occupancy (for FSTAT / FDEBUG MMIO).
    pub fn rx_level(&self, block: usize, sm: usize) -> u32 {
        debug_assert!(block < PIO_BLOCKS);
        debug_assert!(sm < SMS_PER_BLOCK);
        self.rx[block][sm].len()
    }

    // --- PIO-thread-side FIFO ---

    /// PIO thread pops a word pushed by CPU on the TX side. `None` if empty.
    pub fn tx_pop(&self, block: usize, sm: usize) -> Option<u32> {
        debug_assert!(block < PIO_BLOCKS);
        debug_assert!(sm < SMS_PER_BLOCK);
        self.tx[block][sm].try_pop()
    }

    /// PIO thread pushes a word to the CPU-side RX FIFO. Returns false if full.
    pub fn rx_push(&self, block: usize, sm: usize, val: u32) -> bool {
        debug_assert!(block < PIO_BLOCKS);
        debug_assert!(sm < SMS_PER_BLOCK);
        self.rx[block][sm].try_push(val)
    }

    // --- Atomic control ---

    /// Read the 4-bit state-machine enable mask for `block` (one bit per SM).
    pub fn read_sm_enabled(&self, block: usize) -> u8 {
        debug_assert!(block < PIO_BLOCKS);
        self.sm_enabled[block].load(Relaxed)
    }

    /// Write the 4-bit state-machine enable mask for `block` (one bit per SM).
    pub fn write_sm_enabled(&self, block: usize, mask: u8) {
        debug_assert!(block < PIO_BLOCKS);
        self.sm_enabled[block].store(mask, Relaxed);
    }

    /// Read the 8-bit PIO IRQ-flag register for `block` (4 user IRQs + 4 spare).
    pub fn read_irq_flags(&self, block: usize) -> u8 {
        debug_assert!(block < PIO_BLOCKS);
        self.irq_flags[block].load(Relaxed)
    }

    /// Overwrite the 8-bit PIO IRQ-flag register for `block`.
    pub fn write_irq_flags(&self, block: usize, flags: u8) {
        debug_assert!(block < PIO_BLOCKS);
        self.irq_flags[block].store(flags, Relaxed);
    }

    /// Clear IRQ flag bits indicated by `mask` (write-1-to-clear semantics).
    pub fn clear_irq_flags(&self, block: usize, mask: u8) {
        debug_assert!(block < PIO_BLOCKS);
        self.irq_flags[block].fetch_and(!mask, Relaxed);
    }

    /// Read the 8-bit DREQ signal byte for `block` (one bit per TX/RX DREQ).
    pub fn read_dreq(&self, block: usize) -> u8 {
        debug_assert!(block < PIO_BLOCKS);
        self.dreq[block].load(Relaxed)
    }

    /// Overwrite the 8-bit DREQ signal byte for `block`.
    pub fn write_dreq(&self, block: usize, val: u8) {
        debug_assert!(block < PIO_BLOCKS);
        self.dreq[block].store(val, Relaxed);
    }

    // --- Command queue ---

    /// Queue a cold-path command for the PIO thread to drain. Used for
    /// firmware setup operations (instr memory writes, clock divider
    /// reprogramming).
    pub fn send_command(&self, cmd: PioCommand) {
        self.commands
            .lock()
            .expect("PIO command mutex poisoned")
            .push(cmd);
    }

    /// Drain all pending commands. Intended for the PIO thread to call
    /// during the coordinator phase.
    ///
    /// Preserves the queue's allocated capacity across drains: `mem::take`
    /// would replace the guarded `Vec` with `Vec::new()` (cap 0), which
    /// makes the next quantum's push path reallocate from scratch. For
    /// firmware doing heavy setup (INSTR_MEM 32×3 = 96 writes plus per-SM
    /// configuration) the reallocation cost adds up — `mem::replace` with
    /// a same-capacity `Vec` recycles the prior allocation so the push
    /// path is steady-state allocation-free after the first warm-up.
    pub fn drain_commands(&self) -> Vec<PioCommand> {
        let mut guard = self.commands.lock().expect("PIO command mutex poisoned");
        let cap = guard.capacity();
        std::mem::replace(&mut *guard, Vec::with_capacity(cap))
    }

    // --- Reset ---

    /// Reset all shared PIO state. Called during emulator reset
    /// (coordinator phase, no concurrent access).
    pub fn reset(&self) {
        for block in 0..PIO_BLOCKS {
            for sm in 0..SMS_PER_BLOCK {
                self.tx[block][sm].clear();
                self.rx[block][sm].clear();
            }
            self.sm_enabled[block].store(0, Relaxed);
            self.irq_flags[block].store(0, Relaxed);
            self.dreq[block].store(0, Relaxed);
        }
        self.commands
            .lock()
            .expect("PIO command mutex poisoned")
            .clear();
    }
}

impl Default for ThreadedPio {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_push_pop() {
        let pio = ThreadedPio::new();
        assert!(pio.tx_push(0, 0, 0xDEAD_BEEF));
        assert_eq!(pio.tx_pop(0, 0), Some(0xDEAD_BEEF));
        assert_eq!(pio.tx_pop(0, 0), None);
    }

    #[test]
    fn rx_push_pop() {
        let pio = ThreadedPio::new();
        assert!(pio.rx_push(0, 0, 0xCAFE_BABE));
        assert_eq!(pio.rx_pop(0, 0), Some(0xCAFE_BABE));
        assert_eq!(pio.rx_pop(0, 0), None);
    }

    #[test]
    fn fifo_full_at_depth() {
        let pio = ThreadedPio::new();
        for i in 0..PIO_FIFO_DEPTH {
            assert!(pio.tx_push(1, 2, i), "push {i} should succeed");
        }
        assert!(
            !pio.tx_push(1, 2, 0xFFFF),
            "push into full FIFO must fail"
        );
        assert_eq!(pio.tx_level(1, 2), PIO_FIFO_DEPTH);
    }

    #[test]
    fn sm_enabled_atomic() {
        let pio = ThreadedPio::new();
        pio.write_sm_enabled(2, 0xF);
        assert_eq!(pio.read_sm_enabled(2), 0xF);
        // Other blocks unaffected.
        assert_eq!(pio.read_sm_enabled(0), 0);
        assert_eq!(pio.read_sm_enabled(1), 0);
    }

    #[test]
    fn irq_flags_set_clear() {
        let pio = ThreadedPio::new();
        pio.write_irq_flags(1, 0x5);
        assert_eq!(pio.read_irq_flags(1), 0x5);
        pio.clear_irq_flags(1, 0x1);
        assert_eq!(pio.read_irq_flags(1), 0x4);
    }

    #[test]
    fn dreq_set_read() {
        let pio = ThreadedPio::new();
        pio.write_dreq(0, 0xAA);
        assert_eq!(pio.read_dreq(0), 0xAA);
    }

    #[test]
    fn command_send_drain() {
        let pio = ThreadedPio::new();
        pio.send_command(PioCommand::WriteInstrMem {
            block: 0,
            addr: 5,
            value: 0x1234,
            alias: 0,
        });
        pio.send_command(PioCommand::SetClkDiv {
            block: 1,
            sm: 2,
            int_div: 100,
            frac_div: 7,
            alias: 0,
        });

        let drained = pio.drain_commands();
        assert_eq!(drained.len(), 2);
        assert_eq!(
            drained[0],
            PioCommand::WriteInstrMem {
                block: 0,
                addr: 5,
                value: 0x1234,
                alias: 0,
            }
        );
        assert_eq!(
            drained[1],
            PioCommand::SetClkDiv {
                block: 1,
                sm: 2,
                int_div: 100,
                frac_div: 7,
                alias: 0,
            }
        );

        // After drain, queue is empty.
        assert!(pio.drain_commands().is_empty());
    }

    #[test]
    fn drain_empty() {
        let pio = ThreadedPio::new();
        let drained = pio.drain_commands();
        assert!(drained.is_empty());
    }

    /// `drain_commands` must preserve the queue's allocated capacity —
    /// `mem::take` would replace with `Vec::new()` (cap 0), forcing the
    /// next quantum to reallocate from scratch. Setup-heavy firmware
    /// (INSTR_MEM 32×3 + per-SM config) pushes 100+ commands per quantum,
    /// so this matters for steady-state performance.
    #[test]
    fn drain_preserves_capacity() {
        let pio = ThreadedPio::new();
        // Push enough commands to force at least one grow past the initial
        // Vec::with_capacity(64). The actual capacity can be >= 128 after
        // grow — we only care that drain doesn't reset it to 0.
        for i in 0..128u32 {
            pio.send_command(PioCommand::WriteReg {
                block: 0,
                offset: 0x010,
                val: i,
                alias: 0,
            });
        }
        let cap_before = pio.commands.lock().unwrap().capacity();
        assert!(cap_before >= 128, "capacity should have grown to hold 128 entries");

        let drained = pio.drain_commands();
        assert_eq!(drained.len(), 128);

        let cap_after = pio.commands.lock().unwrap().capacity();
        assert_eq!(
            cap_after, cap_before,
            "drain must preserve capacity ({} -> {})",
            cap_before, cap_after
        );
    }
}

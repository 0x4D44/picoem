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

/// Cold-path commands sent from CPU workers to the PIO thread. Phase 2
/// implements only the two variants exercised by tests; Phase 3 will add
/// the remaining variants (SetExecCtrl, SetShiftCtrl, SetPinCtrl, ForceExec,
/// SmRestart, ClkdivRestart) as MMIO dispatch demands them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PioCommand {
    WriteInstrMem { block: u8, addr: u8, value: u16 },
    SetClkDiv { block: u8, sm: u8, int_div: u16, frac_div: u8 },
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
            commands: Mutex::new(Vec::new()),
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
    pub fn drain_commands(&self) -> Vec<PioCommand> {
        let mut guard = self.commands.lock().expect("PIO command mutex poisoned");
        std::mem::take(&mut *guard)
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
        });
        pio.send_command(PioCommand::SetClkDiv {
            block: 1,
            sm: 2,
            int_div: 100,
            frac_div: 7,
        });

        let drained = pio.drain_commands();
        assert_eq!(drained.len(), 2);
        assert_eq!(
            drained[0],
            PioCommand::WriteInstrMem {
                block: 0,
                addr: 5,
                value: 0x1234,
            }
        );
        assert_eq!(
            drained[1],
            PioCommand::SetClkDiv {
                block: 1,
                sm: 2,
                int_div: 100,
                frac_div: 7,
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
}

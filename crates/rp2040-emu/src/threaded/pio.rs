//! `ThreadedPio` — shared PIO state for the RP2040 threaded runtime.
//!
//! Stage 3b.3 (dual-execution HLD V1 §6.4): CPU workers enqueue typed
//! [`PioCommand`]s onto a per-block `Mutex<Vec<PioCommand>>`; the
//! coordinator (Stage 3b.4) drains them, applies them against the
//! coordinator-owned `PioBlock`s, and refreshes a per-block register
//! snapshot that worker-side reads consume.
//!
//! ## What this module does
//!
//! - Defines the [`PioCommand`] wire format covering every MMIO write
//!   shape firmware issues: CTRL, INSTR_MEM, SMn_CLKDIV, and a generic
//!   `WriteReg` fallback that the coordinator dispatches through
//!   `PioBlock::write32`.
//! - Provides per-block typed command queues ([`ThreadedPio::send_command`]
//!   / [`ThreadedPio::drain_commands`]) so worker → coordinator traffic
//!   carries full `PioCommand` payloads without an encode / decode step.
//!   The enclosing `Mutex<Vec<PioCommand>>` mirrors rp2350_emu's shape.
//! - Exposes a coordinator-refreshed register snapshot
//!   ([`ThreadedPio::snapshot_read32`] / [`ThreadedPio::publish_snapshot`])
//!   for CPU-side reads. Stage 3b.3 always reads zero until Stage 3b.4
//!   wires the coordinator refresh — WorkerBus reads therefore observe
//!   the post-reset register value.
//!
//! ## What this module does NOT do
//!
//! - Drive the `PioBlock` state machines. The actual stepping happens
//!   on the coordinator thread in Stage 3b.4.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};

/// Size of the per-block register snapshot (in u32 words). The RP2040
/// PIO register window is 0x140 bytes — round to 0x200 (128 words) for
/// headroom and cache-line alignment. Accessed by offset/4.
const PIO_SNAPSHOT_WORDS: usize = 128;

/// Preallocated capacity of each per-block command queue. Covers the
/// common firmware init shape (32 INSTR_MEM slots + per-SM setup) so
/// the push path is allocation-free after warm-up.
const PIO_CMD_QUEUE_INITIAL_CAP: usize = 64;

/// Cold-path commands sent from CPU workers to the coordinator for
/// application to the actual `PioBlock` state machines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PioCommand {
    /// CTRL (0x000) write: SM_ENABLE / SM_RESTART / CLKDIV_RESTART.
    WriteCtrl { block: u8, val: u32, alias: u8 },
    /// INSTR_MEM slot write. Offset 0x048..0x0C4, stride 4.
    WriteInstrMem {
        block: u8,
        addr: u8,
        value: u16,
        alias: u8,
    },
    /// SMn_CLKDIV write. RP2040 offsets 0x0C8 / 0x0E0 / 0x0F8 / 0x110.
    SetClkDiv {
        block: u8,
        sm: u8,
        int_div: u16,
        frac_div: u8,
        alias: u8,
    },
    /// Generic register write — coordinator dispatches to
    /// `PioBlock::write32` after the RP2040-to-internal offset
    /// translation.
    WriteReg {
        block: u8,
        offset: u16,
        val: u32,
        alias: u8,
    },
}

impl PioCommand {
    /// Which PIO block this command targets (0 or 1).
    #[inline]
    pub fn block(&self) -> u8 {
        match *self {
            PioCommand::WriteCtrl { block, .. }
            | PioCommand::WriteInstrMem { block, .. }
            | PioCommand::SetClkDiv { block, .. }
            | PioCommand::WriteReg { block, .. } => block,
        }
    }
}

/// Shared PIO state. Constructed once per `ThreadedEmulator`, referenced
/// by every worker's `SharedState` clone.
///
/// Each block carries its own `Mutex<Vec<PioCommand>>`. Multiple CPU
/// workers can push concurrently; the Mutex serialises them. The
/// coordinator drains with `std::mem::replace` to preserve the
/// allocated capacity across quanta (matches rp2350_emu's pattern).
pub struct ThreadedPio {
    /// Per-block typed command queue (CPU → coordinator).
    commands: [Mutex<Vec<PioCommand>>; 2],
    /// Coordinator-refreshed register snapshot. Worker reads index by
    /// `offset / 4`. Stage 3b.3: never refreshed (all zero).
    /// Stage 3b.4: coordinator publishes at quantum tail.
    snapshot: [Mutex<[u32; PIO_SNAPSHOT_WORDS]>; 2],
    /// Coordinator-published per-block `SM_ENABLE` bitmap (bits 0..3).
    /// Read lock-free on the CPU side — firmware's CTRL read-after-write
    /// depends on this.
    sm_enabled: [AtomicU8; 2],
}

impl ThreadedPio {
    /// Construct a fresh `ThreadedPio` with empty queues and zero
    /// snapshots.
    pub fn new() -> Self {
        Self {
            commands: [
                Mutex::new(Vec::with_capacity(PIO_CMD_QUEUE_INITIAL_CAP)),
                Mutex::new(Vec::with_capacity(PIO_CMD_QUEUE_INITIAL_CAP)),
            ],
            snapshot: [
                Mutex::new([0u32; PIO_SNAPSHOT_WORDS]),
                Mutex::new([0u32; PIO_SNAPSHOT_WORDS]),
            ],
            sm_enabled: [AtomicU8::new(0), AtomicU8::new(0)],
        }
    }

    /// Push a command onto `block`'s queue. The coordinator will drain
    /// it on the next quantum. Out-of-range blocks are silently dropped
    /// (firmware bug — not a panic vector).
    pub fn send_command(&self, cmd: PioCommand) {
        let block = cmd.block() as usize;
        if block >= 2 {
            return;
        }
        self.commands[block]
            .lock()
            .expect("PIO command mutex poisoned")
            .push(cmd);
    }

    /// Drain all pending commands for `block` (coordinator-side).
    /// Preserves allocated capacity via `mem::replace` so the next
    /// quantum's push path is allocation-free after warm-up. Same
    /// pattern as rp2350_emu's `ThreadedPio::drain_commands`.
    pub fn drain_commands(&self, block: usize) -> Vec<PioCommand> {
        if block >= 2 {
            return Vec::new();
        }
        let mut guard = self.commands[block]
            .lock()
            .expect("PIO command mutex poisoned");
        let cap = guard.capacity();
        std::mem::replace(&mut *guard, Vec::with_capacity(cap))
    }

    /// Read a word from the coordinator-refreshed register snapshot.
    /// Returns 0 until Stage 3b.4 wires the coordinator refresh.
    pub fn snapshot_read32(&self, block: usize, offset: u32) -> u32 {
        if block >= 2 {
            return 0;
        }
        let idx = (offset >> 2) as usize;
        if idx >= PIO_SNAPSHOT_WORDS {
            return 0;
        }
        let snap = self.snapshot[block].lock().unwrap();
        snap[idx]
    }

    /// Publish the register snapshot for `block` — coordinator-side
    /// (Stage 3b.4). Callers must ensure `words.len() <= PIO_SNAPSHOT_WORDS`.
    #[allow(dead_code)] // consumed by Stage 3b.4's coordinator
    pub fn publish_snapshot(&self, block: usize, words: &[u32]) {
        if block >= 2 {
            return;
        }
        let mut snap = self.snapshot[block].lock().unwrap();
        let n = words.len().min(PIO_SNAPSHOT_WORDS);
        snap[..n].copy_from_slice(&words[..n]);
    }

    /// CPU-side read of the per-block `SM_ENABLE` bitmap (CTRL[3:0]).
    pub fn sm_enabled(&self, block: usize) -> u8 {
        if block >= 2 {
            return 0;
        }
        self.sm_enabled[block].load(Ordering::Acquire)
    }

    /// Coordinator-side publish of the `SM_ENABLE` bitmap.
    #[allow(dead_code)] // consumed by Stage 3b.4's coordinator
    pub fn publish_sm_enabled(&self, block: usize, mask: u8) {
        if block >= 2 {
            return;
        }
        self.sm_enabled[block].store(mask & 0xF, Ordering::Release);
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
    fn pio_command_block_dispatch() {
        assert_eq!(
            PioCommand::WriteCtrl {
                block: 0,
                val: 0,
                alias: 0
            }
            .block(),
            0
        );
        assert_eq!(
            PioCommand::WriteInstrMem {
                block: 1,
                addr: 0,
                value: 0,
                alias: 0
            }
            .block(),
            1
        );
        assert_eq!(
            PioCommand::SetClkDiv {
                block: 1,
                sm: 0,
                int_div: 0,
                frac_div: 0,
                alias: 0
            }
            .block(),
            1
        );
        assert_eq!(
            PioCommand::WriteReg {
                block: 0,
                offset: 0,
                val: 0,
                alias: 0
            }
            .block(),
            0
        );
    }

    #[test]
    fn send_command_routes_to_correct_block_queue() {
        let pio = ThreadedPio::new();
        // Push one to block 0.
        pio.send_command(PioCommand::WriteCtrl {
            block: 0,
            val: 0x1,
            alias: 0,
        });
        // Coordinator-side drain sees it on block 0 only; block 1 empty.
        let drained0 = pio.drain_commands(0);
        assert_eq!(drained0.len(), 1);
        assert_eq!(
            drained0[0],
            PioCommand::WriteCtrl {
                block: 0,
                val: 0x1,
                alias: 0,
            }
        );
        assert!(pio.drain_commands(0).is_empty());
        assert!(pio.drain_commands(1).is_empty());
    }

    /// Capacity preservation smoke test — mirrors rp2350_emu's
    /// `drain_preserves_capacity`. After a push→drain cycle the queue's
    /// allocated capacity must survive so steady-state is alloc-free.
    #[test]
    fn drain_preserves_capacity() {
        let pio = ThreadedPio::new();
        for i in 0..128u32 {
            pio.send_command(PioCommand::WriteReg {
                block: 0,
                offset: 0x010,
                val: i,
                alias: 0,
            });
        }
        let cap_before = pio.commands[0].lock().unwrap().capacity();
        assert!(
            cap_before >= 128,
            "capacity should have grown to hold 128 entries"
        );
        let drained = pio.drain_commands(0);
        assert_eq!(drained.len(), 128);
        let cap_after = pio.commands[0].lock().unwrap().capacity();
        assert_eq!(
            cap_after, cap_before,
            "drain must preserve capacity ({cap_before} -> {cap_after})"
        );
    }

    /// Two commands on the same block must drain in push order (FIFO).
    #[test]
    fn drain_preserves_push_order() {
        let pio = ThreadedPio::new();
        pio.send_command(PioCommand::WriteInstrMem {
            block: 1,
            addr: 3,
            value: 0xABCD,
            alias: 1,
        });
        pio.send_command(PioCommand::SetClkDiv {
            block: 1,
            sm: 2,
            int_div: 125,
            frac_div: 32,
            alias: 0,
        });
        let drained = pio.drain_commands(1);
        assert_eq!(drained.len(), 2);
        assert_eq!(
            drained[0],
            PioCommand::WriteInstrMem {
                block: 1,
                addr: 3,
                value: 0xABCD,
                alias: 1,
            }
        );
        assert_eq!(
            drained[1],
            PioCommand::SetClkDiv {
                block: 1,
                sm: 2,
                int_div: 125,
                frac_div: 32,
                alias: 0,
            }
        );
    }

    #[test]
    fn sm_enabled_round_trip() {
        let pio = ThreadedPio::new();
        assert_eq!(pio.sm_enabled(0), 0);
        pio.publish_sm_enabled(0, 0xA);
        assert_eq!(pio.sm_enabled(0), 0xA);
        assert_eq!(pio.sm_enabled(1), 0);
        pio.publish_sm_enabled(1, 0x5);
        assert_eq!(pio.sm_enabled(1), 0x5);
    }

    #[test]
    fn snapshot_read_zero_until_refreshed() {
        let pio = ThreadedPio::new();
        assert_eq!(pio.snapshot_read32(0, 0x000), 0);
        assert_eq!(pio.snapshot_read32(1, 0x130), 0);
        // Publish and re-read.
        let words = [0xABCD_1234u32; 16];
        pio.publish_snapshot(0, &words);
        assert_eq!(pio.snapshot_read32(0, 0x00), 0xABCD_1234);
        assert_eq!(pio.snapshot_read32(0, 0x3C), 0xABCD_1234);
        // Out-of-range word returns zero.
        assert_eq!(pio.snapshot_read32(0, 0x2000), 0);
    }

    #[test]
    fn out_of_range_block_is_noop() {
        let pio = ThreadedPio::new();
        // Blocks ≥ 2 silently drop — firmware bug, not a panic.
        pio.send_command(PioCommand::WriteCtrl {
            block: 3,
            val: 0,
            alias: 0,
        });
        // Nothing arrived on the real blocks.
        assert!(pio.drain_commands(0).is_empty());
        assert!(pio.drain_commands(1).is_empty());
        // Out-of-range reads also safe.
        assert!(pio.drain_commands(5).is_empty());
        assert_eq!(pio.snapshot_read32(5, 0), 0);
        assert_eq!(pio.sm_enabled(7), 0);
    }
}

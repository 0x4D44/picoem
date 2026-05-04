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

use super::SpscQueue;
use std::sync::Mutex;
use std::sync::atomic::{
    AtomicU8, AtomicU64,
    Ordering::{Acquire, Relaxed, Release},
};

pub const PIO_BLOCKS: usize = 3;
pub const SMS_PER_BLOCK: usize = 4;
pub const PIO_FIFO_DEPTH: u32 = 4;

/// Cache-line-aligned wrapper for per-block atomics.
///
/// HLD V5 §2.3 — once the PIO worker splits into three per-block
/// workers (Stage B), each block's `pads` / `irq_flags` / `sm_enabled`
/// / `dreq` byte would otherwise share a cache line with its siblings,
/// creating a textbook 3-writer false-sharing hotspot on hot atomics.
/// Aligning each element to 64 bytes puts each block's atomic on its
/// own line; `Deref` keeps existing accessor bodies
/// (`self.pads[block].store(...)`) unchanged — the atomic methods all
/// take `&self`, so no `DerefMut` is needed.
///
/// Stage A applies the padding up-front while the worker split is
/// still in flight; any perf delta observed with the single PIO worker
/// is pure false-sharing win from the coord↔worker `pads` line bounce.
#[repr(align(64))]
pub(super) struct Aligned<T>(T);

impl<T> std::ops::Deref for Aligned<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.0
    }
}

pub struct ThreadedPio {
    // TX FIFOs: CPU pushes, PIO thread pops
    tx: [[SpscQueue; SMS_PER_BLOCK]; PIO_BLOCKS],

    // RX FIFOs: PIO thread pushes, CPU pops
    rx: [[SpscQueue; SMS_PER_BLOCK]; PIO_BLOCKS],

    // Atomic control — each block's byte on its own cache line so the
    // three PIO workers (post-split) don't false-share.
    sm_enabled: [Aligned<AtomicU8>; PIO_BLOCKS],
    irq_flags: [Aligned<AtomicU8>; PIO_BLOCKS],
    gpio_base: [Aligned<AtomicU8>; PIO_BLOCKS],
    dreq: [Aligned<AtomicU8>; PIO_BLOCKS],

    // Packed pad snapshot: high32 = pad_out, low32 = pad_oe. PIO worker
    // publishes once per quantum; coordinator reads in `update_gpio`.
    // Padded onto its own line per block: coord reads all three
    // sequentially, so cross-block writes must not invalidate a peer's
    // cached line.
    pads: [Aligned<AtomicU64>; PIO_BLOCKS],

    // Cold-path command queue (CPU → PIO thread). Per-block so the
    // Stage B PIO worker split drains its own queue without contending
    // on a shared mutex against the other two PIO workers.
    commands: [Mutex<Vec<PioCommand>>; PIO_BLOCKS],
}

/// Cold-path commands sent from CPU workers to the PIO thread.
///
/// Phase 2 seeded the queue with `WriteInstrMem` / `SetClkDiv`; Phase 3
/// task #11 added `WriteCtrl` (SM enable / restart / clkdiv-restart —
/// the critical unblocker so `ThreadedPio::read_sm_enabled` reflects
/// firmware-programmed state) and a general-purpose `WriteReg` arm that
/// covers every remaining PIO register offset the single-threaded
/// `Bus::write32` hands to `PioBlock::write32`: TXF0..TXF3, FDEBUG,
/// IRQ, IRQ_FORCE, INPUT_SYNC_BYPASS, GPIOBASE, per-SM EXECCTRL/
/// SHIFTCTRL/INSTR/PINCTRL.
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
    WriteInstrMem {
        block: u8,
        addr: u8,
        value: u16,
        alias: u8,
    },
    /// SMn_CLKDIV write. `alias` is propagated through to
    /// `PioBlock::write32` for parity with the single-threaded `Bus`
    /// path, which forwards the 2-bit alias encoded in the upper MMIO
    /// address bits to `PioBlock::write32` unconditionally.
    SetClkDiv {
        block: u8,
        sm: u8,
        int_div: u16,
        frac_div: u8,
        alias: u8,
    },
    /// CTRL (0x000) write: SM_ENABLE / SM_RESTART / CLKDIV_RESTART.
    /// After applying, the PIO worker publishes the resulting
    /// `sm_enabled_mask` onto `ThreadedPio::sm_enabled` so CPU-side
    /// reads observe the new state.
    WriteCtrl { block: u8, val: u32, alias: u8 },
    /// Generic register write — dispatched to `PioBlock::write32` as-is.
    /// Covers TXF0..TXF3, FDEBUG, IRQ, IRQ_FORCE, INPUT_SYNC_BYPASS,
    /// GPIOBASE, per-SM EXECCTRL / SHIFTCTRL / INSTR / PINCTRL, and any
    /// PIO offset the two purpose-built variants above do not route.
    WriteReg {
        block: u8,
        offset: u16,
        val: u32,
        alias: u8,
    },
    /// Test-only panic-injection variant (HLD V5 §2.2 + dual-
    /// execution HLD V1 §5.5). The `apply_pio_command` arm for this
    /// variant unconditionally panics with a message containing
    /// `pio{block}` so Stage B.2 per-block worker split tests and the
    /// dual-execution Stage 1b `worker_panic_surfaces_as_error`
    /// integration test can route a panic to a specific PIO worker
    /// via the block field.
    ///
    /// Feature-gated behind `testing` (Stage 1b review REQUIRED #2);
    /// callers should use [`crate::Emulator::inject_panic_for_testing`]
    /// — itself `testing`-gated — to arm the injection, rather than
    /// constructing this variant directly.
    #[cfg(feature = "testing")]
    TestPanic { block: u8 },
}

impl PioCommand {
    /// Which PIO block this command targets. `send_command` uses this
    /// to route into the per-block command queue so each Stage B.2 PIO
    /// worker drains only the commands addressed to its own block.
    pub fn block(&self) -> u8 {
        match *self {
            PioCommand::WriteInstrMem { block, .. }
            | PioCommand::SetClkDiv { block, .. }
            | PioCommand::WriteCtrl { block, .. }
            | PioCommand::WriteReg { block, .. } => block,
            #[cfg(feature = "testing")]
            PioCommand::TestPanic { block } => block,
        }
    }
}

impl ThreadedPio {
    pub fn new() -> Self {
        Self {
            tx: std::array::from_fn(|_| std::array::from_fn(|_| SpscQueue::new(PIO_FIFO_DEPTH))),
            rx: std::array::from_fn(|_| std::array::from_fn(|_| SpscQueue::new(PIO_FIFO_DEPTH))),
            sm_enabled: std::array::from_fn(|_| Aligned(AtomicU8::new(0))),
            irq_flags: std::array::from_fn(|_| Aligned(AtomicU8::new(0))),
            gpio_base: std::array::from_fn(|_| Aligned(AtomicU8::new(0))),
            dreq: std::array::from_fn(|_| Aligned(AtomicU8::new(0))),
            pads: std::array::from_fn(|_| Aligned(AtomicU64::new(0))),
            // Preallocate for the common setup-heavy case (INSTR_MEM 32
            // slots per block + per-SM setup). Keeps the first-quantum
            // firmware init path from thrashing the allocator through
            // the push path. Subsequent quanta recycle this capacity
            // via `drain_commands`. Per-block so the Stage B.2 PIO
            // worker split drains its own queue uncontended.
            commands: std::array::from_fn(|_| Mutex::new(Vec::with_capacity(64))),
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

    /// Read the RP2350 GPIOBASE value for `block` (0 or 16).
    pub fn read_gpio_base(&self, block: usize) -> u8 {
        debug_assert!(block < PIO_BLOCKS);
        self.gpio_base[block].load(Relaxed)
    }

    /// Publish the RP2350 GPIOBASE value for `block`.
    pub fn write_gpio_base(&self, block: usize, base: u8) {
        debug_assert!(block < PIO_BLOCKS);
        debug_assert!(matches!(base, 0 | 16));
        self.gpio_base[block].store(base, Relaxed);
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

    /// Publish the PIO block's `(pad_out, pad_oe)` pair as a single
    /// atomic so the coordinator never observes a torn snapshot.
    pub fn write_pads(&self, block: usize, out: u32, oe: u32) {
        debug_assert!(block < PIO_BLOCKS);
        self.pads[block].store(((out as u64) << 32) | oe as u64, Release);
    }

    /// Read the `(pad_out, pad_oe)` snapshot for `block`.
    pub fn read_pads(&self, block: usize) -> (u32, u32) {
        debug_assert!(block < PIO_BLOCKS);
        let p = self.pads[block].load(Acquire);
        ((p >> 32) as u32, p as u32)
    }

    // --- Command queue ---

    /// Queue a cold-path command for the PIO thread to drain. Used for
    /// firmware setup operations (instr memory writes, clock divider
    /// reprogramming). Routes to the per-block queue addressed by
    /// `cmd.block()` so each Stage B.2 PIO worker drains only its own
    /// traffic.
    pub fn send_command(&self, cmd: PioCommand) {
        debug_assert!(
            (cmd.block() as usize) < PIO_BLOCKS,
            "PioCommand.block out of range"
        );
        let block_idx = cmd.block() as usize;
        self.commands[block_idx]
            .lock()
            .expect("PIO command mutex poisoned")
            .push(cmd);
    }

    /// Drain all pending commands for one PIO block. Intended for each
    /// per-block PIO worker (Stage B.2) to call at quantum entry.
    ///
    /// Preserves the queue's allocated capacity across drains: `mem::take`
    /// would replace the guarded `Vec` with `Vec::new()` (cap 0), which
    /// makes the next quantum's push path reallocate from scratch. For
    /// firmware doing heavy setup (INSTR_MEM 32 writes plus per-SM
    /// configuration per block) the reallocation cost adds up — `mem::replace`
    /// with a same-capacity `Vec` recycles the prior allocation so the
    /// push path is steady-state allocation-free after the first warm-up.
    pub fn drain_commands(&self, block_idx: usize) -> Vec<PioCommand> {
        debug_assert!(block_idx < PIO_BLOCKS);
        let mut guard = self.commands[block_idx]
            .lock()
            .expect("PIO command mutex poisoned");
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
            self.gpio_base[block].store(0, Relaxed);
            self.dreq[block].store(0, Relaxed);
            self.pads[block].store(0, Release);
            self.commands[block]
                .lock()
                .expect("PIO command mutex poisoned")
                .clear();
        }
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
        assert!(!pio.tx_push(1, 2, 0xFFFF), "push into full FIFO must fail");
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
    fn gpio_base_atomic() {
        let pio = ThreadedPio::new();
        pio.write_gpio_base(1, 16);
        assert_eq!(pio.read_gpio_base(1), 16);
        assert_eq!(pio.read_gpio_base(0), 0);
        assert_eq!(pio.read_gpio_base(2), 0);
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
        // Both commands target block 0 so they land in the same per-block
        // queue and retain their relative ordering on drain.
        pio.send_command(PioCommand::WriteInstrMem {
            block: 0,
            addr: 5,
            value: 0x1234,
            alias: 0,
        });
        pio.send_command(PioCommand::SetClkDiv {
            block: 0,
            sm: 2,
            int_div: 100,
            frac_div: 7,
            alias: 0,
        });

        let drained = pio.drain_commands(0);
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
                block: 0,
                sm: 2,
                int_div: 100,
                frac_div: 7,
                alias: 0,
            }
        );

        // After drain, queue is empty.
        assert!(pio.drain_commands(0).is_empty());
    }

    #[test]
    fn drain_empty() {
        let pio = ThreadedPio::new();
        let drained = pio.drain_commands(0);
        assert!(drained.is_empty());
    }

    /// HLD V5 §4 — `send_command` must route each command into the
    /// queue addressed by `cmd.block()` so each per-block PIO worker
    /// (Stage B.2) drains only its own traffic.
    #[test]
    fn send_routes_by_block() {
        let pio = ThreadedPio::new();
        pio.send_command(PioCommand::WriteInstrMem {
            block: 0,
            addr: 0,
            value: 0x0001,
            alias: 0,
        });
        pio.send_command(PioCommand::WriteInstrMem {
            block: 0,
            addr: 1,
            value: 0x0002,
            alias: 0,
        });
        pio.send_command(PioCommand::WriteInstrMem {
            block: 2,
            addr: 0,
            value: 0x0003,
            alias: 0,
        });

        assert_eq!(pio.drain_commands(0).len(), 2);
        assert_eq!(pio.drain_commands(1).len(), 0);
        assert_eq!(pio.drain_commands(2).len(), 1);
    }

    /// HLD V5 §4 — each per-block queue must independently preserve its
    /// allocated capacity across drains. The existing
    /// `drain_preserves_capacity` covers block 0; this exercises 1 and 2.
    #[test]
    fn drain_preserves_capacity_per_block() {
        for block_idx in [1usize, 2] {
            let pio = ThreadedPio::new();
            // Push enough to force at least one grow past the initial 64.
            for i in 0..128u32 {
                pio.send_command(PioCommand::WriteReg {
                    block: block_idx as u8,
                    offset: 0x010,
                    val: i,
                    alias: 0,
                });
            }
            let cap_before = pio.commands[block_idx].lock().unwrap().capacity();
            assert!(
                cap_before >= 128,
                "block {block_idx}: capacity should have grown to hold 128 entries"
            );

            let drained = pio.drain_commands(block_idx);
            assert_eq!(drained.len(), 128);

            let cap_after = pio.commands[block_idx].lock().unwrap().capacity();
            assert_eq!(
                cap_after, cap_before,
                "block {block_idx}: drain must preserve capacity ({cap_before} -> {cap_after})",
            );
        }
    }

    /// HLD V5 §4 — `reset` must clear every per-block queue, not just
    /// block 0.
    #[test]
    fn reset_clears_all_block_queues() {
        let pio = ThreadedPio::new();
        for block in 0..PIO_BLOCKS as u8 {
            pio.send_command(PioCommand::WriteReg {
                block,
                offset: 0x010,
                val: u32::from(block),
                alias: 0,
            });
        }
        // Pre-reset: each queue has one command.
        for block_idx in 0..PIO_BLOCKS {
            assert_eq!(
                pio.commands[block_idx].lock().unwrap().len(),
                1,
                "block {block_idx} should have one queued command pre-reset"
            );
        }

        pio.reset();

        for block_idx in 0..PIO_BLOCKS {
            assert!(
                pio.drain_commands(block_idx).is_empty(),
                "block {block_idx} queue must be empty after reset"
            );
        }
    }

    /// HLD V5 §4 — `PioCommand::block()` must report the target block
    /// for every production variant.
    #[test]
    fn pio_command_block_accessor() {
        let cases = [
            PioCommand::WriteInstrMem {
                block: 7,
                addr: 0,
                value: 0,
                alias: 0,
            },
            PioCommand::SetClkDiv {
                block: 7,
                sm: 0,
                int_div: 1,
                frac_div: 0,
                alias: 0,
            },
            PioCommand::WriteCtrl {
                block: 7,
                val: 0,
                alias: 0,
            },
            PioCommand::WriteReg {
                block: 7,
                offset: 0,
                val: 0,
                alias: 0,
            },
        ];
        for cmd in &cases {
            assert_eq!(cmd.block(), 7, "block() accessor must return 7 for {cmd:?}");
        }
    }

    /// HLD V5 §4 item 11 — regression guard for the `Aligned<T>`
    /// padding applied to `pads` / `irq_flags` / `sm_enabled` / `dreq`.
    /// Catches a future "let's drop the padding or the Deref" mistake.
    /// The Deref smoke at the bottom proves existing accessor bodies
    /// (`self.pads[block].store(...)`) still compile unchanged.
    #[test]
    fn aligned_pio_atomics_on_own_lines() {
        use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};
        assert_eq!(std::mem::align_of::<Aligned<AtomicU64>>(), 64);
        assert_eq!(std::mem::size_of::<[Aligned<AtomicU8>; 3]>(), 192);
        // Deref-path smoke — proves the accessor pattern still compiles
        // and the forwarded method call works.
        let a = Aligned(AtomicU8::new(0));
        a.store(7, Ordering::Relaxed);
        assert_eq!(a.load(Ordering::Relaxed), 7);
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
            "drain must preserve capacity ({} -> {})",
            cap_before, cap_after
        );
    }
}

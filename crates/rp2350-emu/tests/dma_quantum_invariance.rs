//! DMA quantum-invariance integration test (HLD V0.1.0 §4.2).
//!
//! For each workload class, asserts that the end-state after running a
//! fixed total of master cycles is independent of `step_quantum`. This
//! is the strongest gate the §3 fix can have — it catches any bug
//! whose mechanism produces a quantum-dependent result, regardless of
//! whether the bug is in DMA pacing, DREQ snapshotting, chain
//! triggering, IRQ latching, or anywhere else in the per-quantum tick
//! contract.
//!
//! The §3 fix loops `tick_dma()` once per advanced sysclk inside
//! `Bus::tick_peripherals`. Without that loop, DMA progresses by 1
//! transfer per `step_quantum` cycles instead of 1 per sysclk — a
//! divergence between `quantum=1` (which by accident gives DMA one
//! tick per sysclk) and `quantum=N`. With the fix, the end state
//! after `TOTAL_CYCLES` master clocks is identical at every quantum.
//!
//! Five workloads × four non-reference quanta = 20 invariance
//! assertions per `cargo test` run.
//!
//! ## Scope
//!
//! - **Serial only.** The §3 fix targets the Serial `tick_peripherals`
//!   path (HLD §3 "Scope: Serial only"). Threaded DMA is intentionally
//!   not ticked at all by the Threaded coordinator
//!   (`threaded/emulator.rs:1159`) and is out of scope here.
//! - **Cores halted.** All workloads halt both cores at the start; the
//!   workload programs DMA via direct `bus.write32` writes and never
//!   runs CPU instructions, so the only thing advancing per-quantum is
//!   `tick_peripherals`.
//! - **Public API only.** Integration tests live outside the crate, so
//!   `pub(crate)` items (the `dma` field, internal helpers like the
//!   in-crate `make_ctrl`) are not in scope. We reach DMA via
//!   `bus.read32`/`bus.write32`, PIO RX state via the cross-crate
//!   `PioBlock::push_rx` test-hook (gated under
//!   `cfg(any(test, feature = "test-hooks"))`; integration tests are
//!   `cfg(test)` automatically), and inspect TX/RX FIFO occupancy via
//!   the `PioBlock::FLEVEL` register (offset `0x00C`, side-effect
//!   free).

use rp2350_emu::{Config, Emulator, EmulatorBuilder};

// ---------------------------------------------------------------------------
// MMIO offsets (RP2350 datasheet §12.6.6 / §12.6.1)
// ---------------------------------------------------------------------------

const SRAM_BASE: u32 = 0x2000_0000;

/// DMA controller base.
const DMA_BASE: u32 = 0x5000_0000;
/// Per-channel stride.
const CH_STRIDE: u32 = 0x40;
/// Per-channel register offsets within a 0x40-byte channel block.
const CH_READ_ADDR: u32 = 0x00;
const CH_WRITE_ADDR: u32 = 0x04;
const CH_TRANS_COUNT: u32 = 0x08;
const CH_CTRL_TRIG: u32 = 0x0C;
const CH_AL1_CTRL: u32 = 0x10;

/// Global DMA register offsets.
const REG_INTR: u32 = 0x400;
const REG_INTE0: u32 = 0x404;
const REG_INTS0: u32 = 0x40C;
const REG_INTE1: u32 = 0x414;
const REG_INTS1: u32 = 0x41C;
const REG_TIMER0: u32 = 0x440;

/// Three PIO blocks on RP2350 (PIO0/1/2). Source for the FLEVEL
/// register read used to capture per-SM TX/RX FIFO occupancy.
const PIO_BASES: [u32; 3] = [0x5020_0000, 0x5030_0000, 0x5040_0000];
const PIO_FLEVEL_OFFSET: u32 = 0x00C;
/// PIO RXFn MMIO offset for SM `n` (0x020, 0x024, 0x028, 0x02C). Used
/// as the DMA `READ_ADDR` source for the `pio_rx_paced` workload.
const fn pio_rxf_addr(block: usize, sm: usize) -> u32 {
    PIO_BASES[block] + 0x020 + (sm as u32) * 4
}

/// CTRL field bit positions (RP2350 V6 datasheet §12.6.6 CH0_CTRL_TRIG).
/// Mirrored from `crates/rp2350-emu/src/dma.rs` (the in-crate
/// `make_ctrl` helper is private to the dma.rs `mod tests`; this
/// reimplementation keeps the integration test self-contained while
/// citing the same datasheet field map).
///
/// Field layout:
///   bit 0      EN
///   bit 1      HIGH_PRIORITY (not modelled in V1)
///   bits[3:2]  DATA_SIZE (0=byte, 1=halfword, 2=word)
///   bit 4      INCR_READ
///   bit 5      INCR_READ_REV (not modelled in V1)
///   bit 6      INCR_WRITE
///   bit 7      INCR_WRITE_REV (not modelled in V1)
///   bits[11:8] RING_SIZE
///   bit 12     RING_SEL (0=ring read addr, 1=ring write addr)
///   bits[16:13] CHAIN_TO
///   bits[22:17] TREQ_SEL
///   bit 23     IRQ_QUIET
///   bit 24     BSWAP (not modelled in V1)
///   bits[28:25] reserved / SNIFF_EN / BUSY (RO) / reserved
const CTRL_EN: u32 = 1 << 0;
const CTRL_DATA_SIZE_SHIFT: u32 = 2;
const CTRL_INCR_READ: u32 = 1 << 4;
const CTRL_INCR_WRITE: u32 = 1 << 6;
const CTRL_RING_SIZE_SHIFT: u32 = 8;
const CTRL_RING_SEL: u32 = 1 << 12;
const CTRL_CHAIN_TO_SHIFT: u32 = 13;
const CTRL_TREQ_SEL_SHIFT: u32 = 17;
const CTRL_BUSY: u32 = 1 << 26;

/// TREQ_SEL constants (datasheet §12.6.4).
const TREQ_PIO0_RX0: u8 = 4;
const TREQ_TIMER0: u8 = 59;
const TREQ_FORCE: u8 = 63;

/// Build a CTRL value. Mirror of the private `make_ctrl` in
/// `crates/rp2350-emu/src/dma.rs::tests`; we reimplement here because
/// integration tests can't reach in-crate test helpers.
#[allow(clippy::too_many_arguments)]
fn make_ctrl(
    en: bool,
    data_size: u32,
    incr_read: bool,
    incr_write: bool,
    treq_sel: u8,
    chain_to: u32,
    ring_size: u32,
    ring_sel: bool,
) -> u32 {
    let mut v = 0u32;
    if en {
        v |= CTRL_EN;
    }
    v |= (data_size & 0x3) << CTRL_DATA_SIZE_SHIFT;
    if incr_read {
        v |= CTRL_INCR_READ;
    }
    if incr_write {
        v |= CTRL_INCR_WRITE;
    }
    v |= (treq_sel as u32 & 0x3F) << CTRL_TREQ_SEL_SHIFT;
    v |= (chain_to & 0xF) << CTRL_CHAIN_TO_SHIFT;
    v |= (ring_size & 0xF) << CTRL_RING_SIZE_SHIFT;
    if ring_sel {
        v |= CTRL_RING_SEL;
    }
    v
}

/// Per-channel base address.
const fn ch_base(idx: u32) -> u32 {
    DMA_BASE + idx * CH_STRIDE
}

// ---------------------------------------------------------------------------
// Test parameters (HLD V0.1.0 §4.2)
// ---------------------------------------------------------------------------

/// Quanta to compare. `1` is the reference (the silicon oracle pins
/// quantum=1 — see HLD §4 "Argument for sufficiency").
const QUANTA: &[u32] = &[1, 4, 16, 64, 256];
/// Total master cycles per workload run. `768 = 3 × 256`, divisible by
/// every quantum in `QUANTA`. Required because `Emulator::run(N)`
/// drives `while self.clock.cycles < target` (`lib.rs:843-848`), so a
/// non-aligned target overshoots by up to `quantum-1` cycles — that
/// would produce different actual master-cycle counts per quantum and
/// trivially break invariance even with a correct fix. The 3× factor
/// gives workloads enough room to either complete (FORCE 64 words) or
/// settle into a steady-state stall (PIO RX drained).
const TOTAL_CYCLES: u64 = 768;

// ---------------------------------------------------------------------------
// Snapshot — the diff target at each quantum
// ---------------------------------------------------------------------------

/// End-state observation. Compared via `PartialEq` after running each
/// workload at every quantum. A divergence at any field on any
/// non-reference quantum fails the test with a message naming both
/// the workload and the offending quantum.
#[derive(Debug, PartialEq, Eq)]
struct Snapshot {
    /// 4 KB SRAM window covering all workload src/dst regions
    /// (0x2000_0000..0x2000_1000). Captured byte-wise via
    /// `bus.memory.peek32` so DMA writes that landed in this window
    /// are diffable.
    sram_window: Vec<u8>,
    /// Global DMA IRQ registers.
    intr: u32,
    inte0: u32,
    inte1: u32,
    ints0: u32,
    ints1: u32,
    /// Per-channel observables: (busy, trans_count, read_addr,
    /// write_addr, ctrl). All 16 channels — most idle in any one
    /// workload, but capturing all keeps the spec literal and avoids
    /// any "we forgot to check channel N" regressions.
    chans: [(bool, u32, u32, u32, u32); 16],
    /// Per-(block, sm) RX FIFO occupancy levels. PIO has 3 blocks × 4
    /// SMs; each SM's RX FIFO is 0..=4 (or 0..=8 if FJOIN_RX, not
    /// used here). Decoded from FLEVEL: bits [4:7] within each SM's
    /// 8-bit lane = RX level.
    pio_rx_levels: [[u8; 4]; 3],
    /// Per-(block, sm) TX FIFO occupancy. Decoded from FLEVEL bits
    /// [0:3] within each SM's 8-bit lane.
    pio_tx_levels: [[u8; 4]; 3],
}

impl Snapshot {
    fn capture(emu: &mut Emulator) -> Self {
        // SRAM window: 0x2000_0000..0x2000_1000 (4 KB) — covers the
        // src/dst arrangements of every workload (see workload setup
        // comments). Use `bus.memory.peek32` to bypass any peripheral
        // dispatch (SRAM has no read-side effects, but peek is the
        // cheaper path).
        let mut sram_window = Vec::with_capacity(0x1000);
        for off in 0..0x1000u32 {
            sram_window.push(emu.bus.memory.peek8(SRAM_BASE + off));
        }

        // DMA global registers via Bus::read32. None of these have
        // read-side effects on the DMA model (the W1C INTSn just
        // returns `(intr | intfn) & inten` on read).
        let intr = emu.bus.read32(DMA_BASE + REG_INTR, 0);
        let inte0 = emu.bus.read32(DMA_BASE + REG_INTE0, 0);
        let inte1 = emu.bus.read32(DMA_BASE + REG_INTE1, 0);
        let ints0 = emu.bus.read32(DMA_BASE + REG_INTS0, 0);
        let ints1 = emu.bus.read32(DMA_BASE + REG_INTS1, 0);

        // Per-channel observables. Reading CTRL_TRIG returns the live
        // CTRL with BUSY merged in from the channel's `busy` flag, so
        // the BUSY bit reflects in-flight state.
        let mut chans = [(false, 0u32, 0u32, 0u32, 0u32); 16];
        for i in 0..16u32 {
            let base = ch_base(i);
            let read_addr = emu.bus.read32(base + CH_READ_ADDR, 0);
            let write_addr = emu.bus.read32(base + CH_WRITE_ADDR, 0);
            let trans_count = emu.bus.read32(base + CH_TRANS_COUNT, 0);
            let ctrl = emu.bus.read32(base + CH_CTRL_TRIG, 0);
            let busy = (ctrl & CTRL_BUSY) != 0;
            chans[i as usize] = (busy, trans_count, read_addr, write_addr, ctrl);
        }

        // PIO FIFO occupancy via FLEVEL (read-only, side-effect free).
        // FLEVEL layout per `picoem-common/src/pio/mod.rs::flevel`:
        //   for each SM i in 0..4:
        //     bits[(i*8)..(i*8+4)] = TX level (0..=8)
        //     bits[(i*8+4)..(i*8+8)] = RX level (0..=8)
        let mut pio_tx_levels = [[0u8; 4]; 3];
        let mut pio_rx_levels = [[0u8; 4]; 3];
        for (block, base) in PIO_BASES.iter().enumerate() {
            let flevel = emu.bus.read32(base + PIO_FLEVEL_OFFSET, 0);
            for sm in 0..4 {
                let lane = (flevel >> (sm * 8)) & 0xFF;
                pio_tx_levels[block][sm] = (lane & 0xF) as u8;
                pio_rx_levels[block][sm] = ((lane >> 4) & 0xF) as u8;
            }
        }

        Self {
            sram_window,
            intr,
            inte0,
            inte1,
            ints0,
            ints1,
            chans,
            pio_rx_levels,
            pio_tx_levels,
        }
    }
}

// ---------------------------------------------------------------------------
// Workload runner
// ---------------------------------------------------------------------------

/// Build a fresh Serial emulator at the requested quantum, halt both
/// cores, run `setup` to program DMA, run `TOTAL_CYCLES` master
/// cycles, then capture the end-state. The default `EmulatorBuilder`
/// produces a Serial emulator (`ExecutionModel::Serial`); we don't
/// need to call `.execution(...)` explicitly.
fn run_workload(quantum: u32, setup: fn(&mut Emulator)) -> Snapshot {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(quantum)
        .build()
        .expect("Serial build is infallible");
    emu.core_mut(0).halt();
    emu.core_mut(1).halt();
    setup(&mut emu);
    emu.run(TOTAL_CYCLES)
        .expect("Serial Emulator::run is infallible");
    Snapshot::capture(&mut emu)
}

// ---------------------------------------------------------------------------
// Workloads
// ---------------------------------------------------------------------------

/// Workload 1 — `force_64_words`. CH0 FORCE-paced (TREQ_SEL=63) memcpy,
/// 64 words from src to dst. With the §3 fix, FORCE delivers one
/// transfer per sysclk; 64 transfers fit comfortably in 768 cycles, so
/// the channel completes well before the run ends. End state: dst
/// fully populated, INTR bit 0 latched, BUSY clear, TRANS_COUNT=0.
fn setup_force_64_words(emu: &mut Emulator) {
    let src: u32 = SRAM_BASE + 0x100;
    let dst: u32 = SRAM_BASE + 0x300;
    for i in 0..64u32 {
        emu.bus.write32(src + i * 4, 0xCAFE_0000 + i, 0);
    }
    emu.bus.write32(ch_base(0) + CH_READ_ADDR, src, 0);
    emu.bus.write32(ch_base(0) + CH_WRITE_ADDR, dst, 0);
    emu.bus.write32(ch_base(0) + CH_TRANS_COUNT, 64, 0);
    let ctrl = make_ctrl(
        true, 2, true, true, TREQ_FORCE, 0, 0, false,
    );
    emu.bus.write32(ch_base(0) + CH_CTRL_TRIG, ctrl, 0);
}

/// Workload 2 — `timer_paced`. TIMER0 X=1, Y=10 (one transfer every
/// 10 sysclks); CH0 with TRANS_COUNT=200 so the channel does *not*
/// complete within 768 cycles. End state: ~76 transfers fired (768/10
/// rounded), BUSY remains, INTR bit 0 not yet latched, dst populated
/// up to ~word 76. The exact transfer count is what we're locking
/// across quanta — it must be the same at quantum 1 and quantum 256.
fn setup_timer_paced(emu: &mut Emulator) {
    // TIMER0 register: X[31:16] = 1, Y[15:0] = 10.
    emu.bus
        .write32(DMA_BASE + REG_TIMER0, (1u32 << 16) | 10, 0);

    let src: u32 = SRAM_BASE + 0x100;
    let dst: u32 = SRAM_BASE + 0x500;
    // Pre-fill 200 source words so even a fast quantum can't run off
    // the end of the source region (every word's preimage is in the
    // sram_window so divergence in `read_addr` is fully diffable).
    for i in 0..200u32 {
        emu.bus.write32(src + i * 4, 0x1000 + i, 0);
    }
    emu.bus.write32(ch_base(0) + CH_READ_ADDR, src, 0);
    emu.bus.write32(ch_base(0) + CH_WRITE_ADDR, dst, 0);
    // 200 is comfortably more than (TOTAL_CYCLES/Y) = 76, so the
    // channel never completes during the run.
    emu.bus.write32(ch_base(0) + CH_TRANS_COUNT, 200, 0);
    let ctrl = make_ctrl(
        true, 2, true, true, TREQ_TIMER0, 0, 0, false,
    );
    emu.bus.write32(ch_base(0) + CH_CTRL_TRIG, ctrl, 0);
}

/// Workload 3 — `pio_rx_paced`. CH0 paced on DREQ_PIO0_RX0
/// (TREQ_SEL=4); pre-fill PIO0 SM0 RX FIFO with 4 words via
/// `push_rx`. SM0 stays disabled at PioBlock::new defaults so the
/// FIFO doesn't refill. CH0 drains 4 words then stalls (DREQ
/// de-asserts when RX FIFO is empty). End state: 4 transfers
/// completed, dst[0..4] populated, RX FIFO empty, BUSY remains,
/// TRANS_COUNT=12 (16 - 4).
///
/// This is the workload most directly stressing "DREQ snapshot
/// freshness inside the per-cycle loop" (HLD §3). Without per-cycle
/// `collect_dreqs` re-snapshot, the DMA would either drain past
/// empty (reading zero from PIO RXF and writing 16 words) or stop
/// after 1 transfer. Either deviation diverges from quantum=1 and
/// fails the assertion.
fn setup_pio_rx_paced(emu: &mut Emulator) {
    // Pre-fill PIO0 SM0 RX FIFO with 4 words (default depth = 4).
    // `push_rx` is gated under `cfg(any(test, feature = "test-hooks"))`;
    // integration tests are `cfg(test)` automatically AND the dev-
    // dependency on picoem-common enables `test-hooks`, so this
    // resolves on both gates.
    for i in 0..4u32 {
        let ok = emu.bus.pio[0].push_rx(0, 0xBEEF_0000 + i);
        assert!(ok, "RX FIFO must accept 4 words at default depth");
    }

    // Source = PIO0 RXF0 MMIO (no read-incr); dst = SRAM.
    let dst: u32 = SRAM_BASE + 0x700;
    emu.bus
        .write32(ch_base(0) + CH_READ_ADDR, pio_rxf_addr(0, 0), 0);
    emu.bus.write32(ch_base(0) + CH_WRITE_ADDR, dst, 0);
    emu.bus.write32(ch_base(0) + CH_TRANS_COUNT, 16, 0);
    // INCR_READ=false (FIFO MMIO), INCR_WRITE=true.
    let ctrl = make_ctrl(
        true, 2, false, true, TREQ_PIO0_RX0, 0, 0, false,
    );
    emu.bus.write32(ch_base(0) + CH_CTRL_TRIG, ctrl, 0);
}

/// Workload 4 — `chain_4_then_4`. CH0 TRANS_COUNT=4 with CHAIN_TO=1.
/// CH1 pre-armed via the AL1_CTRL alias (writing AL1_CTRL does NOT
/// trigger the channel — the trigger variants are AL1_WRITE_ADDR,
/// AL2_TRANS_COUNT, AL3_READ_ADDR), TRANS_COUNT=4, self-chain (so
/// the chain doesn't loop). Both channels FORCE-paced so they
/// complete within 8 cycles each.
///
/// With the §3 fix, the chain trigger fires mid-quantum: CH0
/// completes around cycle 4, that completion arms CH1 inside the
/// same `tick_peripherals` call, CH1 then takes over arbitration for
/// cycles 5..=8. End state: both INTR bits set, both BUSY clear,
/// both destinations populated.
fn setup_chain_4_then_4(emu: &mut Emulator) {
    let src0: u32 = SRAM_BASE + 0x100;
    let dst0: u32 = SRAM_BASE + 0x200;
    let src1: u32 = SRAM_BASE + 0x800;
    let dst1: u32 = SRAM_BASE + 0x900;
    for i in 0..4u32 {
        emu.bus.write32(src0 + i * 4, 0xAAAA_0000 + i, 0);
        emu.bus.write32(src1 + i * 4, 0xBBBB_0000 + i, 0);
    }

    // CH1: pre-program via AL1_CTRL — this does NOT trigger CH1.
    // Self-chain (chain_to = 1 = CH1's own index = "no chain" per
    // datasheet).
    emu.bus.write32(ch_base(1) + CH_READ_ADDR, src1, 0);
    emu.bus.write32(ch_base(1) + CH_WRITE_ADDR, dst1, 0);
    emu.bus.write32(ch_base(1) + CH_TRANS_COUNT, 4, 0);
    let ctrl1 = make_ctrl(
        true, 2, true, true, TREQ_FORCE, 1, 0, false,
    );
    emu.bus.write32(ch_base(1) + CH_AL1_CTRL, ctrl1, 0);

    // CH0: program and trigger. CHAIN_TO=1 → CH1 fires when CH0's
    // trans_count hits 0.
    emu.bus.write32(ch_base(0) + CH_READ_ADDR, src0, 0);
    emu.bus.write32(ch_base(0) + CH_WRITE_ADDR, dst0, 0);
    emu.bus.write32(ch_base(0) + CH_TRANS_COUNT, 4, 0);
    let ctrl0 = make_ctrl(
        true, 2, true, true, TREQ_FORCE, 1, 0, false,
    );
    emu.bus.write32(ch_base(0) + CH_CTRL_TRIG, ctrl0, 0);
}

/// Workload 5 — `two_channels_armed`. CH0 + CH2 both FORCE-paced,
/// independent src/dst regions. Picking CH2 (not CH1) keeps the
/// chain workload's CH1 use distinct, so a "captured the wrong
/// channel state" bug in either workload doesn't accidentally
/// pass on the other.
///
/// Fixed-priority arbitration is lowest-index-wins, so per cycle
/// CH0 wins until it completes (cycle 16, since TRANS_COUNT=16),
/// then CH2 takes over for the next 16 cycles. Both complete
/// comfortably within 768 cycles. End state: both BUSY clear, both
/// INTR bits set, both destinations populated.
fn setup_two_channels_armed(emu: &mut Emulator) {
    let src0: u32 = SRAM_BASE + 0x100;
    let dst0: u32 = SRAM_BASE + 0x200;
    let src2: u32 = SRAM_BASE + 0xA00;
    let dst2: u32 = SRAM_BASE + 0xB00;
    for i in 0..16u32 {
        emu.bus.write32(src0 + i * 4, 0x1111_0000 + i, 0);
        emu.bus.write32(src2 + i * 4, 0x2222_0000 + i, 0);
    }

    // CH0.
    emu.bus.write32(ch_base(0) + CH_READ_ADDR, src0, 0);
    emu.bus.write32(ch_base(0) + CH_WRITE_ADDR, dst0, 0);
    emu.bus.write32(ch_base(0) + CH_TRANS_COUNT, 16, 0);
    let ctrl0 = make_ctrl(
        true, 2, true, true, TREQ_FORCE, 0, 0, false,
    );

    // CH2 (skip CH1 to avoid overlap with the chain workload's CH1).
    emu.bus.write32(ch_base(2) + CH_READ_ADDR, src2, 0);
    emu.bus.write32(ch_base(2) + CH_WRITE_ADDR, dst2, 0);
    emu.bus.write32(ch_base(2) + CH_TRANS_COUNT, 16, 0);
    let ctrl2 = make_ctrl(
        true, 2, true, true, TREQ_FORCE, 2, 0, false,
    );

    // Order: arm CH2 first, then CH0. Once CH0 is armed it takes
    // priority anyway (lowest-index-wins), but starting CH2 first
    // means both are in `busy` state at the moment CH0 trigger
    // returns — covers the "two simultaneously armed" arbitration
    // path rather than "CH0 fires then CH2 fires" sequentially.
    emu.bus.write32(ch_base(2) + CH_CTRL_TRIG, ctrl2, 0);
    emu.bus.write32(ch_base(0) + CH_CTRL_TRIG, ctrl0, 0);
}

// ---------------------------------------------------------------------------
// The test
// ---------------------------------------------------------------------------

/// HLD V0.1.0 §4.2: for each workload, `Snapshot::capture` after
/// `TOTAL_CYCLES` master clocks must equal across all `QUANTA`. The
/// reference is `quantum=1` (the silicon oracle's pinned quantum).
///
/// Five workloads × four non-reference quanta = 20 invariance
/// assertions. A failure message names both the workload and the
/// offending quantum, so the supervisor's review can map any
/// regression directly back to the responsible cycle-loop bug.
#[test]
fn dma_end_state_invariant_across_step_quantum() {
    let workloads: &[(&str, fn(&mut Emulator))] = &[
        ("force_64_words", setup_force_64_words),
        ("timer_paced", setup_timer_paced),
        ("pio_rx_paced", setup_pio_rx_paced),
        ("chain_4_then_4", setup_chain_4_then_4),
        ("two_channels_armed", setup_two_channels_armed),
    ];
    for (name, setup) in workloads {
        let reference = run_workload(QUANTA[0], *setup);
        for &q in &QUANTA[1..] {
            let result = run_workload(q, *setup);
            assert_eq!(
                result, reference,
                "workload '{}' diverged at quantum={} (reference quantum={})",
                name, q, QUANTA[0],
            );
        }
    }
}

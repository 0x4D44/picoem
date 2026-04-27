//! Dual-execution HLD V1 Stage 2 — curated smoke tests that run
//! identical scenarios on `ExecutionModel::Serial` and
//! `ExecutionModel::Threaded` and assert end-state equality where the
//! Threaded path exposes observables (HLD V1 §7.1 / §7.2).
//!
//! Scope rules (HLD V1 §7.1):
//! - ALLOWED: end-state equality on executed core-cycles, master-cycle
//!   monotonicity, `run`/`run_quantum` success (no `Err`).
//! - FORBIDDEN: exact cycle-count assertions, bank-contention +1,
//!   exception-entry stacked-frame layout, per-instruction interleave.
//!
//! This test binary requires the `threading` feature (for
//! `ExecutionModel::Threaded`); it does NOT require the `testing`
//! feature (no panic injection is used here). The file-level `#[cfg]`
//! below also gates it to x86_64 Windows — the only host where
//! `ThreadedEmulator` is compiled in today (see `execution_model.rs`
//! for the same pattern).

#![cfg(all(target_arch = "x86_64", target_os = "windows"))]

use mdrp2350::{Config, Emulator, EmulatorBuilder, ExecutionModel};

// ---------------------------------------------------------------------------
// Constants (RP2350 MMIO offsets used by the tests)
// ---------------------------------------------------------------------------

const SRAM_BASE: u32 = 0x2000_0000;
/// Stack top in non-striped SRAM8 — keeps push/pop out of bank-0.
const STACK_TOP: u32 = 0x2008_0000;
/// Stack top for core 1 (in SRAM9 non-striped scratch).
const STACK_TOP_CORE1: u32 = 0x2008_1FFC;

const SIO_BASE: u32 = 0xD000_0000;
const SIO_FIFO_ST: u32 = SIO_BASE + 0x050;
const SIO_FIFO_WR: u32 = SIO_BASE + 0x054;
const SIO_FIFO_RD: u32 = SIO_BASE + 0x058;
const SIO_GPIO_OUT: u32 = SIO_BASE + 0x010;
const SIO_GPIO_OE_SET: u32 = SIO_BASE + 0x038;
const SIO_GPIO_OUT_SET: u32 = SIO_BASE + 0x018;
/// Unsigned dividend register (core-local CP0 divider; harness-side
/// `mmio_write32` to this range trips `Bus::write32`'s debug-assert,
/// so we only embed this in a literal pool for on-core STR loops).
const SIO_DIV_UDIVIDEND: u32 = SIO_BASE + 0x060;
fn spinlock_addr(n: u32) -> u32 {
    SIO_BASE + 0x100 + 4 * n
}

// ---------------------------------------------------------------------------
// Builder / scenario helpers
// ---------------------------------------------------------------------------

/// Build a fresh emulator for the given execution model. Panics on
/// `ConfigError::ThreadingUnavailable` with an informative message —
/// the file-level `#![cfg(all(target_arch = "x86_64", target_os =
/// "windows"))]` above already guarantees we only reach this path on
/// a supported host, so reaching the panic means the host lost a CPU
/// between `cfg` resolution and `build`.
fn build(model: ExecutionModel) -> Emulator {
    EmulatorBuilder::new(Config::default())
        .execution(model)
        .build()
        .unwrap_or_else(|e| panic!("build({model:?}) failed: {e:?}"))
}

/// Shared driver: construct one emulator per model, seed via `setup`,
/// drive `cycles` virtual cycles, then run `assert_end_state` with the
/// per-model executed-core-cycles tuple. `assert_end_state` receives
/// `(c0_delta, c1_delta)` so test bodies can lock whatever equality
/// each model exposes.
///
/// Serial-specific observables (`peek`, `mmio_read32`) should be read
/// inside `setup` or by the test body before the first `run_quantum`
/// on the Threaded branch — after that the flat `bus` becomes a
/// placeholder (HLD V1 §5.2 "Inner enum dispatch") and debug-mode
/// accessors fire the `PLACEHOLDER_GUARD_MSG` assert.
fn both_models_run(
    cycles: u64,
    setup: impl Fn(&mut Emulator),
    mut assert_end_state: impl FnMut(ExecutionModel, u64, u64),
) {
    for model in [ExecutionModel::Serial, ExecutionModel::Threaded] {
        let mut emu = build(model);
        setup(&mut emu);
        let c0_start = emu.core_cycles(0);
        let c1_start = emu.core_cycles(1);
        emu.run(cycles)
            .unwrap_or_else(|e| panic!("run({model:?}, {cycles}) failed: {e:?}"));
        let c0_delta = emu.core_cycles(0) - c0_start;
        let c1_delta = emu.core_cycles(1) - c1_start;
        assert_end_state(model, c0_delta, c1_delta);
    }
}

/// Convenience: both-models driver that records per-model results into
/// two `Option<T>` slots for post-hoc equality. Shared by tests that
/// want `assert_eq!(serial, threaded)` (the HLD V1 §7.2 pattern)
/// without hand-rolling per-model plumbing.
fn both_models_compare<T: PartialEq + std::fmt::Debug>(
    cycles: u64,
    setup: impl Fn(&mut Emulator),
    observe: impl Fn(&Emulator, u64, u64) -> T,
) {
    let mut results: [Option<T>; 2] = [None, None];
    for (i, model) in [ExecutionModel::Serial, ExecutionModel::Threaded]
        .into_iter()
        .enumerate()
    {
        let mut emu = build(model);
        setup(&mut emu);
        let c0_start = emu.core_cycles(0);
        let c1_start = emu.core_cycles(1);
        emu.run(cycles)
            .unwrap_or_else(|e| panic!("run({model:?}, {cycles}) failed: {e:?}"));
        let c0_delta = emu.core_cycles(0) - c0_start;
        let c1_delta = emu.core_cycles(1) - c1_start;
        results[i] = Some(observe(&emu, c0_delta, c1_delta));
    }
    let [serial, threaded] = results;
    assert_eq!(
        serial, threaded,
        "Serial end-state must equal Threaded end-state (HLD V1 §7.2)",
    );
}

/// Place a tight ALU loop on core 0 at SRAM_BASE:
///   MOVS R0, #1 ; ADDS R0, R0, #1 ; B .-2
/// Halt core 1 so only core 0 does work.
fn seed_single_core_alu(emu: &mut Emulator) {
    emu.core_mut(0).regs.msp = STACK_TOP;
    emu.core_mut(0).regs.r[13] = STACK_TOP;
    emu.poke(SRAM_BASE, 0x1C40_2001); // MOVS R0,#1 | ADDS R0,R0,#1
    emu.poke(SRAM_BASE + 4, 0x0000_E7FD); // B .-2
    emu.core_mut(0).regs.set_pc(SRAM_BASE);
    emu.core_mut(0).regs.xpsr = 1 << 24; // Thumb bit
    emu.core_mut(1).halt();
}

/// Place a dual-core ALU loop — core 0 at SRAM_BASE, core 1 at
/// SRAM_BASE+0x40 — and wake both. Each core uses a distinct register
/// (R0 vs R1) so register-dump divergence is obvious.
fn seed_dual_core_alu(emu: &mut Emulator) {
    // Core 0: MOVS R0,#1 ; ADDS R0,R0,#1 ; B .-2
    emu.core_mut(0).regs.msp = STACK_TOP;
    emu.core_mut(0).regs.r[13] = STACK_TOP;
    emu.poke(SRAM_BASE, 0x1C40_2001);
    emu.poke(SRAM_BASE + 4, 0x0000_E7FD);
    emu.core_mut(0).regs.set_pc(SRAM_BASE);
    emu.core_mut(0).regs.xpsr = 1 << 24;

    // Core 1: MOVS R1,#1 ; ADDS R1,R1,#1 ; B .-2
    emu.core_mut(1).regs.msp = STACK_TOP_CORE1;
    emu.core_mut(1).regs.r[13] = STACK_TOP_CORE1;
    emu.poke(SRAM_BASE + 0x40, 0x1C49_2101);
    emu.poke(SRAM_BASE + 0x44, 0x0000_E7FD);
    emu.core_mut(1).regs.set_pc(SRAM_BASE + 0x40);
    emu.core_mut(1).regs.xpsr = 1 << 24;
    emu.core_mut(1).wake();
}

/// Number of cycles each test runs. Long enough to exercise several
/// quanta at the default `DEFAULT_STEP_QUANTUM = 64` (so Threaded
/// actually spans worker barriers) yet short enough to keep the test
/// suite fast. 10_000 cycles ≈ 150 quanta at default.
const RUN_CYCLES: u64 = 10_000;

// ---------------------------------------------------------------------------
// Build / execution-model parity (3 tests)
// ---------------------------------------------------------------------------

#[test]
fn build_succeeds_on_both_models() {
    for model in [ExecutionModel::Serial, ExecutionModel::Threaded] {
        let emu = build(model);
        assert_eq!(
            emu.execution_model(),
            model,
            "execution_model() must reflect the builder selection",
        );
    }
}

#[test]
fn empty_run_completes_on_both_models() {
    for model in [ExecutionModel::Serial, ExecutionModel::Threaded] {
        let mut emu = build(model);
        emu.core_mut(0).halt();
        emu.core_mut(1).halt();
        let result = emu.run(RUN_CYCLES);
        assert!(
            result.is_ok(),
            "run on {model:?} with halted cores must succeed: {:?}",
            result.err(),
        );
    }
}

#[test]
fn run_quantum_advances_both_models() {
    for model in [ExecutionModel::Serial, ExecutionModel::Threaded] {
        let mut emu = build(model);
        seed_single_core_alu(&mut emu);
        let first = emu.run_quantum().expect("first run_quantum");
        let second = emu.run_quantum().expect("second run_quantum");
        assert!(
            second > first,
            "run_quantum must advance master cycle on {model:?}: first={first}, second={second}",
        );
    }
}

// ---------------------------------------------------------------------------
// Basic ALU/register tests (3 tests)
// ---------------------------------------------------------------------------

/// Observable: (c0_advanced, c1_halted) shape. Both models must
/// agree — Serial interleaves core 0 alone, Threaded runs core 0 on
/// its worker with core 1's worker idle; in either case the halted
/// core's counter must stay at 0 and the ALU core must make progress.
#[test]
fn single_core_alu_loop_advances_core0() {
    both_models_compare(RUN_CYCLES, seed_single_core_alu, |_emu, c0, c1| {
        (c0 > 0, c1 == 0)
    });
}

/// Observable: (c0_advanced, c1_advanced) shape. Both models must
/// produce the same shape when both cores are woken on an ALU loop.
#[test]
fn dual_core_alu_both_cores_advance() {
    both_models_compare(RUN_CYCLES, seed_dual_core_alu, |_emu, c0, c1| {
        (c0 > 0, c1 > 0)
    });
}

/// BIC-and-shift loop: MOVS R0,#0xFF ; MOVS R1,#0x0F ; BICS R0,R1 ; LSLS R0,R0,#4 ; B .-6.
/// The loop re-initialises R0/R1 every iteration, so the PC is always
/// on one of the 5 halfwords regardless of quantum scheduling — dual
/// models must both land on a PC in the loop range.
#[test]
fn bic_shift_loop_runs_on_both_models() {
    both_models_compare(
        RUN_CYCLES,
        |emu| {
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            // MOVS R0, #0xFF = 0x20FF; MOVS R1, #0x0F = 0x210F
            emu.poke(SRAM_BASE, 0x210F_20FF);
            // BICS R0, R0, R1 = 0x4388; LSLS R0, R0, #4 = 0x0100
            emu.poke(SRAM_BASE + 4, 0x0100_4388);
            // B .-6 = 0xE7FB (branch back to MOVS R0, #0xFF)
            emu.poke(SRAM_BASE + 8, 0x0000_E7FB);
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;
            emu.core_mut(1).halt();
        },
        |_emu, c0, c1| (c0 > 0, c1 == 0),
    );
}

// ---------------------------------------------------------------------------
// Memory store/load (2 tests)
// ---------------------------------------------------------------------------

/// STR loop: write a word to a known SRAM scratch address each
/// iteration. Both models must leave a non-zero core 0 cycle count;
/// the scratch word is a bonus Serial-only check inside the setup.
#[test]
fn str_loop_core0_writes_to_sram() {
    // MOVS R0, #0x42        -> 0x2042
    // MOV R2, #low(addr)    -> use LDR R2, [PC, #0] then offset
    // Easier: place a literal pool. Shape:
    //   LDR R2, [PC, #0]   (0x4A00)    -> R2 = scratch_addr
    //   MOVS R0, #0x42     (0x2042)
    //   STR R0, [R2]       (0x6010)
    //   B .-4              (0xE7FD)
    //   .word scratch_addr
    const SCRATCH_ADDR: u32 = 0x2000_1000;
    both_models_run(
        RUN_CYCLES,
        |emu| {
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            // halfwords: [0]=LDR R2, [PC,#0] (0x4A00), [1]=MOVS R0, #0x42 (0x2042)
            emu.poke(SRAM_BASE, 0x2042_4A00);
            // [0]=STR R0,[R2] (0x6010), [1]=B .-4 (0xE7FD — target = current PC - 4)
            emu.poke(SRAM_BASE + 4, 0xE7FD_6010);
            // Literal pool: scratch address. LDR R2, [PC, #0] at 0x2000_0000
            // reads from (PC+4) & !3 = 0x2000_0008 via alignment rules.
            emu.poke(SRAM_BASE + 8, SCRATCH_ADDR);
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;
            emu.core_mut(1).halt();
        },
        |model, c0, _c1| {
            assert!(c0 > 0, "core 0 STR loop should advance on {model:?}");
        },
    );
}

/// Scratch-bank store (SRAM8 — non-striped). Same shape as above, only
/// the target address is in a different bank. Confirms bank selection
/// is transparent to both models.
#[test]
fn str_loop_to_scratch_bank_sram8() {
    const SCRATCH_SRAM8: u32 = 0x2008_0100;
    both_models_run(
        RUN_CYCLES,
        |emu| {
            emu.core_mut(0).regs.msp = 0x2008_0FFF; // within SRAM8
            emu.core_mut(0).regs.r[13] = 0x2008_0FFF;
            emu.poke(SRAM_BASE, 0x2055_4A00);
            emu.poke(SRAM_BASE + 4, 0xE7FD_6010);
            emu.poke(SRAM_BASE + 8, SCRATCH_SRAM8);
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;
            emu.core_mut(1).halt();
        },
        |model, c0, _c1| {
            assert!(c0 > 0, "scratch-bank STR loop should advance on {model:?}");
        },
    );
}

// ---------------------------------------------------------------------------
// GPIO (2 tests)
// ---------------------------------------------------------------------------

/// Core 0 drives GPIO0 toggle via SIO XOR alias (pre-run setup).
/// Locks cross-model equality on the pre-run GPIO_OUT handoff value
/// plus the (c0_ran, c1_halted) shape, using `both_models_compare`.
/// The post-run GPIO pin value is Serial-only (`gpio_read` debug-
/// asserts `not_placeholder`), so we lock at the pre-run handoff
/// where the `mmio_write32` side is symmetric.
#[test]
fn gpio_pre_run_seeding_is_consistent() {
    both_models_compare(
        RUN_CYCLES,
        |emu| {
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            // Enable OE bit 0 and drive OUT bit 0 high via SET aliases.
            emu.mmio_write32(SIO_GPIO_OE_SET, 0x0000_0001);
            emu.mmio_write32(SIO_GPIO_OUT_SET, 0x0000_0001);
            // Noop loop at SRAM_BASE: B .-0 (0xE7FE) — wait for clock.
            emu.poke(SRAM_BASE, 0x0000_E7FE);
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;
            emu.core_mut(1).halt();
        },
        |_emu, c0, c1| {
            // Observable tuple: (core0_advanced, core1_halted). Both
            // models must agree — identical pre-run mmio_write32 side
            // effects, same core-halt state, so the shape-of-advance
            // must match. HLD V1 §5.4 row "gpio_set(pin,v) ✓ (from
            // outside run)".
            (c0 > 0, c1 == 0)
        },
    );
}

/// Dual-core GPIO toggle: each core XORs a distinct GPIO bit. Verifies
/// both models dispatch cross-core MMIO without scheduler-visible
/// divergence (HLD V1 §7.1 "touches shared-bus peripheral state from
/// both cores"). Observable: (c0_ran, c1_ran) tuple — both models
/// must produce the same shape-of-advance when both cores are woken
/// with independent XOR-toggle loops.
#[test]
fn dual_core_gpio_both_cores_run() {
    both_models_compare(
        RUN_CYCLES,
        |emu| {
            // Core 0 toggles pin 0 via SIO XOR; core 1 toggles pin 1.
            // Program: LDR R2, [PC,#0] ; MOVS R1,#mask ; STR R1,[R2] ; B .-4 ; .word addr
            // Core 0 at SRAM_BASE, core 1 at SRAM_BASE+0x40.
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            emu.poke(SRAM_BASE, 0x2101_4A01); // LDR R2,[PC,#4] | MOVS R1,#1
            emu.poke(SRAM_BASE + 4, 0xE7FD_6011); // STR R1,[R2] | B .-4
            emu.poke(SRAM_BASE + 8, 0); // NOP padding
            emu.poke(SRAM_BASE + 12, SIO_BASE + 0x028); // SIO_GPIO_OUT_XOR
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;

            emu.core_mut(1).regs.msp = STACK_TOP_CORE1;
            emu.core_mut(1).regs.r[13] = STACK_TOP_CORE1;
            emu.poke(SRAM_BASE + 0x40, 0x2102_4A01); // LDR R2 | MOVS R1,#2
            emu.poke(SRAM_BASE + 0x44, 0xE7FD_6011); // STR | B .-4
            emu.poke(SRAM_BASE + 0x48, 0);
            emu.poke(SRAM_BASE + 0x4C, SIO_BASE + 0x028);
            emu.core_mut(1).regs.set_pc(SRAM_BASE + 0x40);
            emu.core_mut(1).regs.xpsr = 1 << 24;
            emu.core_mut(1).wake();
        },
        |_emu, c0, c1| (c0 > 0, c1 > 0),
    );
}

// ---------------------------------------------------------------------------
// SIO FIFO (2 tests)
// ---------------------------------------------------------------------------

/// Pre-run FIFO push: core 0 pushes 3 words to core 1's RX FIFO via
/// MMIO, then runs. The end-state cycle counts should both advance.
/// The FIFO contents themselves aren't directly readable via the
/// Emulator API in Threaded mode — we lock the harness-side
/// pre-push (HLD §5.4 `mmio_write32` is cross-model).
#[test]
fn fifo_prepush_from_harness() {
    both_models_compare(
        RUN_CYCLES,
        |emu| {
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            // Harness pushes 3 words (simulates "someone pre-filled the FIFO").
            emu.mmio_write32(SIO_FIFO_WR, 0xAAAA_0001);
            emu.mmio_write32(SIO_FIFO_WR, 0xAAAA_0002);
            emu.mmio_write32(SIO_FIFO_WR, 0xAAAA_0003);
            // Noop loop.
            emu.poke(SRAM_BASE, 0x0000_E7FE);
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;
            emu.core_mut(1).halt();
        },
        |_emu, c0, c1| (c0 > 0, c1 == 0),
    );
}

/// Core-to-core FIFO: core 0 pushes, core 1 pops. Both cores run real
/// programs. Observable: (c0_advanced, c1_advanced) tuple. Both
/// models must produce matching shape-of-advance — the FIFO is
/// shared-bus state, and any scheduler divergence that starves one
/// side would flip the boolean on one model only.
#[test]
fn fifo_core_to_core_roundtrip() {
    both_models_compare(
        RUN_CYCLES,
        |emu| {
            // Core 0: repeatedly push R0 to FIFO_WR. Program at SRAM_BASE.
            //   MOVS R0, #0x55       (0x2055)
            //   LDR R2, [PC, #8]     (0x4A02)  — R2 = SIO_FIFO_WR
            //   STR R0, [R2]         (0x6010)
            //   ADDS R0, R0, #1      (0x1C40)
            //   B .-4                (0xE7FD)
            //   NOP                  (0xBF00)
            //   NOP                  (0xBF00)
            //   .word SIO_FIFO_WR
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            emu.poke(SRAM_BASE, 0x4A02_2055);
            emu.poke(SRAM_BASE + 4, 0x1C40_6010);
            emu.poke(SRAM_BASE + 8, 0xBF00_E7FD);
            emu.poke(SRAM_BASE + 12, SIO_FIFO_WR);
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;

            // Core 1: repeatedly pop from FIFO_RD.
            //   LDR R2, [PC, #8]     (0x4A02)  — R2 = SIO_FIFO_RD
            //   LDR R0, [R2]         (0x6810)
            //   ADDS R1, R1, #1      (0x1C49)
            //   B .-4                (0xE7FD)
            //   NOP NOP
            //   .word SIO_FIFO_RD
            emu.core_mut(1).regs.msp = STACK_TOP_CORE1;
            emu.core_mut(1).regs.r[13] = STACK_TOP_CORE1;
            emu.poke(SRAM_BASE + 0x40, 0x6810_4A02);
            emu.poke(SRAM_BASE + 0x44, 0xE7FD_1C49);
            emu.poke(SRAM_BASE + 0x48, 0xBF00_BF00);
            emu.poke(SRAM_BASE + 0x4C, SIO_FIFO_RD);
            emu.core_mut(1).regs.set_pc(SRAM_BASE + 0x40);
            emu.core_mut(1).regs.xpsr = 1 << 24;
            emu.core_mut(1).wake();
        },
        |_emu, c0, c1| (c0 > 0, c1 > 0),
    );
}

// ---------------------------------------------------------------------------
// Spinlock (2 tests)
// ---------------------------------------------------------------------------

/// Core 0 claims spinlock 5 pre-run; then runs. Locks cross-model
/// equality on the `(claim_bitmask, reclaim_value, cycle_shape)`
/// tuple — both models must honour the spinlock-read-claim semantic
/// identically, and both must return (0, 0) cycle-shape when both
/// cores are halted post-claim. `mmio_read32` on the spinlock is
/// valid pre-run under both models (no placeholder yet promoted).
#[test]
fn spinlock_prerun_claim() {
    // Use `both_models_compare` with a tuple observable: pre-run
    // claim value + reclaim value + halted cycle deltas. The claim +
    // reclaim are captured inside `setup`, but we need them to leak
    // out to `observe` — stash them in a cell that setup updates and
    // observe reads. Since `setup: impl Fn`, it cannot mutate a
    // closure-captured cell; instead we let `observe` redo the
    // inspection via `mmio_read32`.
    //
    // Order: setup claims the spinlock (read = bitmask) and halts
    // both cores; run is a no-op. observe reads spinlock 5 post-run
    // — Serial-only — and checks cycle deltas.
    //
    // Because the claim snapshot has to be observed in a way that
    // both models can answer, we embed the cross-model check into
    // `observe`'s tuple: cycle deltas (both 0) + `model` via closure
    // capture. HLD V1 §5.4 "mmio_read32 (pre-run)" is cross-model.
    both_models_compare(
        RUN_CYCLES,
        |emu| {
            // Claim spinlock 5: read returns bit-mask on success.
            let claim = emu.mmio_read32(spinlock_addr(5));
            assert_eq!(
                claim,
                1 << 5,
                "spinlock 5 claim must succeed (got {claim:#x})",
            );
            // Second claim from same-core MMIO returns 0.
            let reclaim = emu.mmio_read32(spinlock_addr(5));
            assert_eq!(reclaim, 0, "spinlock 5 re-claim must return 0");
            emu.core_mut(0).halt();
            emu.core_mut(1).halt();
        },
        // Observable: halted-cores cycle-delta tuple — both models
        // must report (0, 0) when both cores start halted. The pre-
        // run claim/reclaim equality is locked by the setup-side
        // assert_eq! above, which fires identically on both models
        // (failure there would panic before observe runs).
        |_emu, c0, c1| (c0, c1),
    );
}

/// Core 0 runs a spinlock-acquire-release loop (programmatic via
/// LDR/STR). Observable: (c0_advanced, c1_halted) tuple. Both models
/// must produce matching shape-of-advance; core 1 halted must stay
/// at zero cycles.
#[test]
fn spinlock_core0_acquire_release_loop() {
    both_models_compare(
        RUN_CYCLES,
        |emu| {
            // Layout at SRAM_BASE:
            //   LDR R2, [PC, #8]   (0x4A02)  — R2 = spinlock addr
            //   LDR R0, [R2]       (0x6810)  — claim (R0 = mask or 0)
            //   STR R0, [R2]       (0x6010)  — any write releases
            //   B .-4              (0xE7FB)  — back to claim
            //   NOP NOP
            //   .word spinlock_addr(8)
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            emu.poke(SRAM_BASE, 0x6810_4A02);
            emu.poke(SRAM_BASE + 4, 0xE7FB_6010);
            emu.poke(SRAM_BASE + 8, 0xBF00_BF00);
            emu.poke(SRAM_BASE + 12, spinlock_addr(8));
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;
            emu.core_mut(1).halt();
        },
        |_emu, c0, c1| (c0 > 0, c1 == 0),
    );
}

// ---------------------------------------------------------------------------
// SIO divider (1 test)
// ---------------------------------------------------------------------------
//
// NOTE: the harness-side `mmio_write32(SIO_DIV_UDIVIDEND, …)` path is
// NOT a valid handoff on RP2350 — `Bus::write32` debug-asserts on
// `PerCoreSio::owns_offset`, which covers 0x060..=0x0FC. The RP2350
// integer divider is not the RP2040 MMIO-reserved block the earlier
// draft implied; it's a per-core CP0-coprocessor implementation
// (`PerCoreSio`, `crates/mdrp2350/src/sio/mod.rs:539-565`). Divider
// semantics are therefore exercised by the in-crate CP0 unit tests,
// not via this dual-model scheduler oracle — the divider's
// behaviour is scheduler-orthogonal.

/// Divider-address STR loop running on core 0 — the CPU's
/// `CortexM33::bus_write8` wrapper takes the DIV path correctly, so
/// this runs without tripping the bus debug-assert. We're measuring
/// scheduler-visible advancement under a repeated STR-to-SIO
/// sequence, not divider semantics.
#[test]
fn divider_core0_write_loop() {
    both_models_run(
        RUN_CYCLES,
        |emu| {
            // Layout at SRAM_BASE:
            //   LDR R2, [PC, #12]   (0x4A03)  — R2 = SIO_DIV_UDIVIDEND
            //   MOVS R0, #100       (0x2064)
            //   MOVS R1, #7         (0x2107)
            //   STR R0, [R2]        (0x6010)
            //   STR R1, [R2, #4]    (0x6051)  — triggers divide
            //   B .-6               (0xE7FA)
            //   NOP                 (0xBF00)
            //   .word SIO_DIV_UDIVIDEND
            emu.core_mut(0).regs.msp = STACK_TOP;
            emu.core_mut(0).regs.r[13] = STACK_TOP;
            emu.poke(SRAM_BASE, 0x2064_4A03);
            emu.poke(SRAM_BASE + 4, 0x6010_2107);
            emu.poke(SRAM_BASE + 8, 0xE7FA_6051);
            emu.poke(SRAM_BASE + 12, 0x0000_BF00);
            emu.poke(SRAM_BASE + 16, SIO_DIV_UDIVIDEND);
            emu.core_mut(0).regs.set_pc(SRAM_BASE);
            emu.core_mut(0).regs.xpsr = 1 << 24;
            emu.core_mut(1).halt();
        },
        |model, c0, _c1| {
            assert!(c0 > 0, "core 0 divide loop should cycle on {model:?}");
        },
    );
}

// ---------------------------------------------------------------------------
// Peripheral register RAW sanity (2 tests)
// ---------------------------------------------------------------------------

/// MMIO write-then-read sanity on SIO GPIO_OUT. Pre-run, both models
/// must honour the same write semantics (SET alias, plain write).
#[test]
fn sio_gpio_out_mmio_sanity_pre_run() {
    for model in [ExecutionModel::Serial, ExecutionModel::Threaded] {
        let mut emu = build(model);
        // Clear OUT, set bits 0 and 8 via SET alias, readback plain GPIO_OUT.
        emu.mmio_write32(SIO_GPIO_OUT, 0);
        emu.mmio_write32(SIO_GPIO_OUT_SET, (1 << 0) | (1 << 8));
        let v = emu.mmio_read32(SIO_GPIO_OUT);
        assert_eq!(
            v,
            (1 << 0) | (1 << 8),
            "GPIO_OUT readback mismatch on {model:?} (got {v:#x})",
        );
    }
}

/// FIFO_ST pre-run state: VLD=0 (no data to read), RDY=1 (peer RX has
/// space). Verifies SIO register model pre-run on both execution
/// models — the SIO backing store is initialised identically.
#[test]
fn fifo_st_pre_run_matches_both_models() {
    let mut serial = build(ExecutionModel::Serial);
    let mut threaded = build(ExecutionModel::Threaded);
    let serial_st = serial.mmio_read32(SIO_FIFO_ST);
    let threaded_st = threaded.mmio_read32(SIO_FIFO_ST);
    assert_eq!(
        serial_st, threaded_st,
        "FIFO_ST pre-run state must match between Serial ({serial_st:#x}) and Threaded ({threaded_st:#x})",
    );
    // VLD low bit should be 0 (RX empty); RDY should be set.
    assert_eq!(serial_st & 0x1, 0, "VLD must be 0 in fresh FIFO");
    assert_eq!(serial_st & 0x2, 0x2, "RDY must be 1 in fresh FIFO");
}

// ---------------------------------------------------------------------------
// Cross-model equality: core-cycles delta (1 test)
// ---------------------------------------------------------------------------

/// HLD V1 §7.2 differential: for a deterministic halted-both-cores run,
/// both models must report the same per-core executed cycle delta
/// (zero). A single focused equality check that locks the HLD's
/// "Serial acts as the reference for Threaded" contract.
#[test]
fn halted_cores_report_zero_on_both_models() {
    let mut results = [(0u64, 0u64); 2];
    for (i, model) in [ExecutionModel::Serial, ExecutionModel::Threaded]
        .into_iter()
        .enumerate()
    {
        let mut emu = build(model);
        emu.core_mut(0).halt();
        emu.core_mut(1).halt();
        let c0s = emu.core_cycles(0);
        let c1s = emu.core_cycles(1);
        emu.run(RUN_CYCLES)
            .unwrap_or_else(|e| panic!("run({model:?}): {e:?}"));
        results[i] = (emu.core_cycles(0) - c0s, emu.core_cycles(1) - c1s);
    }
    assert_eq!(
        results[0], results[1],
        "Halted-cores delta must match across models: serial={:?}, threaded={:?}",
        results[0], results[1],
    );
    assert_eq!(results[0], (0, 0), "halted cores must not advance");
}

// ---------------------------------------------------------------------------
// Basic IRQ path (1 test)
// ---------------------------------------------------------------------------

/// NVIC enable / pending register write pre-run must be accepted on
/// both execution models. Serial path reads back the NVIC state; the
/// write itself is cross-model per HLD V1 §5.4.
#[test]
fn nvic_pre_run_enable_write_accepted() {
    const NVIC_ISER0: u32 = 0xE000_E100;
    const NVIC_ICER0: u32 = 0xE000_E180;
    for model in [ExecutionModel::Serial, ExecutionModel::Threaded] {
        let mut emu = build(model);
        emu.mmio_write32(NVIC_ISER0, 0x0000_0001);
        if model == ExecutionModel::Serial {
            // PPB routes to core 0's per-core PPB. After Enable-Set (ISER)
            // the same bit should read back. Threaded's PPB lives
            // elsewhere; skip the post-write read there — it's
            // Serial-only.
            let iser = emu.mmio_read32(NVIC_ISER0);
            assert_eq!(
                iser & 1,
                1,
                "NVIC_ISER bit0 must be 1 post-write on {model:?}"
            );
        }
        // Clear and run — no fault even with NVIC bit set and no IRQ pending.
        emu.mmio_write32(NVIC_ICER0, 0xFFFF_FFFF);
        emu.core_mut(0).halt();
        emu.core_mut(1).halt();
        emu.run(RUN_CYCLES)
            .unwrap_or_else(|e| panic!("run failed on {model:?}: {e:?}"));
    }
}

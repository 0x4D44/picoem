// Dual-core contention oracle catalogue — measures per-iter cycle cost on
// core 0 while core 1 runs an antagonist sequence, diffs HW vs EMU.
//
// The single-core cycle oracle (`cycle_cases`) and the single-core bank-
// conflict oracle (`bank_conflict_cases`) both pin core 1 halted. This
// oracle is the cross-core complement: `Bus::contention_check_active` in
// `mdrp2350/src/bus/mod.rs` implements the +1 cycle penalty when two
// cores' downstream ports collide, but that mechanism is unit-test-only
// today. This oracle releases core 1 on both silicon and emulator and
// compares HW vs EMU per-iter cycles end-to-end.
//
// Each case runs the cycle oracle's K-delta measurement stub on core 0.
// The antagonist body runs on core 1 as an infinite loop, which the
// runner releases before core 0's measurement begins and halts after it
// ends. The runner's per-core stack slot for core 1 keeps its frame
// separate from core 0's.
//
// See `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage
// Expansion.md` §Component 3 for the catalogue and measurement contract.

use crate::cycle_cases::{
    self, fresh_emulator as fresh_emulator_cycle, measure_emu as measure_emu_cycle, pack_seq,
    pack_stub, CYCLE_SEQ_SLOT, STUB_START,
};
use crate::silicon_oracle::{self, enable_cyccnt, CaseOutcome, Verdict};
use crate::{
    CYCLE_MAILBOX_BASE, DUALCORE_ANTAGONIST_SLOT, DUALCORE_CORE1_DATA, DUALCORE_CORE1_STACK,
    EMU_TEST_STACK,
};
use mdrp2350::Emulator;
use probe_rs::{Core, MemoryInterface, RegisterId, Session};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Register IDs — mirror the style used elsewhere in the harness.
// ---------------------------------------------------------------------------

const PC_REG: RegisterId = RegisterId(15);
const XPSR_REG: RegisterId = RegisterId(16);
const SP_REG: RegisterId = RegisterId(13);
const LR_REG: RegisterId = RegisterId(14);
/// Packed register holding {CONTROL, FAULTMASK, BASEPRI, PRIMASK}
/// (ARM Cortex-M debug register id 0b10100 = 20). Writing zero zeroes
/// all four — thread-mode, MSP, privileged, no FPCA/SFPA — shedding
/// any bootrom CONTROL state core 1 inherits on reset_and_halt.
const EXTRA_REG: RegisterId = RegisterId(0b10100);

/// Map register index 0..15 to probe-rs `RegisterId` for `init_core*`
/// slots. Caller validates that the index is in-range (0..16) before
/// calling; this helper just widens the u8 for the probe API.
fn reg_id(n: u8) -> RegisterId {
    RegisterId(n as u16)
}

/// Tight timeout for core-0 stub's DONE flag in this oracle. The dualcore
/// path is comparable to the cycle oracle's (same stub, same K-range),
/// with a touch of slack for the concurrent core-1 traffic.
const DONE_TIMEOUT: Duration = Duration::from_secs(2);

// ---------------------------------------------------------------------------
// Case type + catalogue
// ---------------------------------------------------------------------------

/// A single dual-core contention case.
///
/// `seq_core0` is the Thumb halfword stream run on core 0 inside the
/// K-iteration measurement loop — same contract as `CycleCase::seq`: MUST
/// NOT end in `bx lr` (0x4770); the runner appends that at upload time.
///
/// `seq_core1` is the antagonist body run on core 1. The runner appends
/// an infinite-backward-branch halfword (`b -2` = 0xE7FE) so the core 1
/// body loops forever; the runner halts core 1 when core 0's measurement
/// completes.
///
/// `init_core0` and `init_core1` are `(reg_idx, value)` slots applied to
/// core 0 / core 1 before the measurement begins. Only registers 0..=15
/// are valid indices — the unit tests assert this. Core 0's R4-R7 are
/// owned by the measurement stub; cases MUST set only R0..=R3 and R8..=R12.
///
/// `emu_baseline` is the emulator's per-iter cycle cost as of the last
/// run; the runner prints the live value on each invocation so drift is
/// visible. Update this field when a case's emulator value legitimately
/// changes — not to "make tests pass", which defeats the oracle.
///
/// `tolerance_override` lets race-dependent cases (spinlock churn, FIFO
/// xfer) opt into a wider band than `DualCoreArgs::tolerance`. K-delta
/// cancels per-invocation framing but NOT per-iteration race outcomes —
/// which core wins each contended cycle is non-deterministic, so the
/// per-iter number has intrinsic variance for those cases. `None` falls
/// back to `args.tolerance` (the strict default). Placeholder values
/// should be tuned from silicon data in a follow-up pass.
pub struct DualCoreCase {
    pub name: &'static str,
    pub seq_core0: &'static [u16],
    pub seq_core1: &'static [u16],
    pub init_core0: &'static [(u8, u32)],
    pub init_core1: &'static [(u8, u32)],
    pub emu_baseline: u32,
    pub tolerance_override: Option<u32>,
}

// ---------------------------------------------------------------------------
// Antagonist sequences — hand-assembled Thumb halfwords. Each halfword
// annotated with its mnemonic per the project convention.
// ---------------------------------------------------------------------------

/// Core 0's LDR sequence — `ldr r0, [r1]`.
///
/// Paired with bank-0 vs bank-1 antagonists below to isolate the
/// cross-core contention signal. r1 is primed to `DUALCORE_CORE1_DATA`
/// (bank 0 — `(0x200 >> 2) & 7 = 0`) by the same-bank case; and to
/// `DUALCORE_CORE1_DATA + 4` (bank 1) by the diff-bank case. Core 1's
/// antagonist thrashes its own bank and the diff bank from the other
/// direction, so the oracle sees two near-identical patterns where only
/// the bank-match bit differs.
const SEQ_CORE0_LDR_B0: &[u16] = &[
    0x6808, // ldr r0, [r1]     — core 0 load from bank 0
];

const SEQ_CORE0_LDR_B0_DIFF: &[u16] = &[
    0x6808, // ldr r0, [r1]     — core 0 load from bank 0 (same as above; control)
];

/// Core 1 thrashes bank 0: `ldr r0, [r2]; str r0, [r2]`.
///
/// r2 is primed to `DUALCORE_CORE1_DATA` (bank 0). The runner appends
/// `b -2` (0xE7FE) after these two halfwords so core 1 spins the
/// `ldr; str` pair forever until the runner halts it.
const SEQ_CORE1_THRASH_B0: &[u16] = &[
    0x6810, // ldr r0, [r2]     — core 1 load from same bank
    0x6010, // str r0, [r2]     — core 1 store to same bank
];

/// Core 1 thrashes bank 1: `ldr r0, [r2]; str r0, [r2]` with r2 pointing
/// at `DUALCORE_CORE1_DATA + 4` (bank 1) — the control case where core 0
/// (bank 0) and core 1 (bank 1) do NOT contend.
const SEQ_CORE1_THRASH_B1: &[u16] = &[
    0x6810, // ldr r0, [r2]     — core 1 load from different bank
    0x6010, // str r0, [r2]     — core 1 store to different bank
];

/// Both cores churn on SIO SPINLOCK 0: `ldr r0, [r1]` (test-and-set
/// claims on read — returns 1<<N on success, 0 on collision) followed
/// by `str r0, [r1]` (any write releases). r1 points at SPINLOCK0
/// (`0xD000_0100`).
///
/// Note: SPINLOCK reads have side effects — a successful claim sets the
/// bit. This case's pair of operations models the "claim then release"
/// discipline firmware uses; the test-and-set atomicity and the
/// throughput-under-contention number are what the oracle diffs.
const SEQ_CORE0_SPINLOCK: &[u16] = &[
    0x6808, // ldr r0, [r1]     — core 0 test-and-set claim on SPINLOCK0
    0x6008, // str r0, [r1]     — core 0 release
];

const SEQ_CORE1_SPINLOCK: &[u16] = &[
    0x680A, // ldr r2, [r1]     — core 1 test-and-set claim (into r2 to stay clear of r0)
    0x600A, // str r2, [r1]     — core 1 release
];

/// Core 0 pushes 4 words to FIFO_WR at SIO+0x54. r1 is primed to
/// `0xD000_0054`; r0 is primed to a 32-bit data word the runner loops
/// push-store.
const SEQ_CORE0_FIFO_PUSH: &[u16] = &[
    0x6008, // str r0, [r1]     — push word 1 to FIFO_WR
    0x6008, // str r0, [r1]     — push word 2 to FIFO_WR
    0x6008, // str r0, [r1]     — push word 3 to FIFO_WR
    0x6008, // str r0, [r1]     — push word 4 to FIFO_WR
];

/// Core 1 pops 4 words from FIFO_RD at SIO+0x58. r1 is primed to
/// `0xD000_0058`; reads destructively pop the core's RX queue.
const SEQ_CORE1_FIFO_POP: &[u16] = &[
    0x680A, // ldr r2, [r1]     — pop word 1 from FIFO_RD
    0x680A, // ldr r2, [r1]     — pop word 2 from FIFO_RD
    0x680A, // ldr r2, [r1]     — pop word 3 from FIFO_RD
    0x680A, // ldr r2, [r1]     — pop word 4 from FIFO_RD
];

// Register-prime tables.
//
// Core 0 r1 / r2 etc. are primed OUTSIDE the measurement stub — the stub
// enters with a fresh blx to seq, and seq trusts r0..r3 have been set.
// The stub reserves r4..r7 (callee-saved loop state) so cases must not
// touch r4..r7 on core 0.

const INIT_CORE0_LDR_B0: &[(u8, u32)] = &[
    (1, DUALCORE_CORE1_DATA),       // r1 = bank-0 data slot
];

const INIT_CORE1_LDR_B0: &[(u8, u32)] = &[
    (2, DUALCORE_CORE1_DATA),       // r2 = bank-0 data slot (same bank as core 0)
];

const INIT_CORE1_LDR_B1: &[(u8, u32)] = &[
    (2, DUALCORE_CORE1_DATA + 4),   // r2 = bank-1 data slot (different bank)
];

const INIT_CORE0_SPINLOCK: &[(u8, u32)] = &[
    (1, 0xD000_0100),               // r1 = SPINLOCK0 register
];

const INIT_CORE1_SPINLOCK: &[(u8, u32)] = &[
    (1, 0xD000_0100),               // r1 = SPINLOCK0 register
];

const INIT_CORE0_FIFO: &[(u8, u32)] = &[
    (0, 0xA5A5_A5A5),               // r0 = data payload
    (1, 0xD000_0054),               // r1 = SIO_FIFO_WR
];

const INIT_CORE1_FIFO: &[(u8, u32)] = &[
    (1, 0xD000_0058),               // r1 = SIO_FIFO_RD
];

/// Initial catalogue. `emu_baseline` values are seeded conservatively;
/// the runner prints the live emulator value each run so drift is
/// visible. Update the values here when a case's emulator cost
/// legitimately changes.
pub const CASES: &[DualCoreCase] = &[
    DualCoreCase {
        name: "dualcore_load_same_bank",
        seq_core0: SEQ_CORE0_LDR_B0,
        seq_core1: SEQ_CORE1_THRASH_B0,
        init_core0: INIT_CORE0_LDR_B0,
        init_core1: INIT_CORE1_LDR_B0,
        emu_baseline: 7,
        tolerance_override: None,
    },
    DualCoreCase {
        name: "dualcore_load_diff_bank",
        seq_core0: SEQ_CORE0_LDR_B0_DIFF,
        seq_core1: SEQ_CORE1_THRASH_B1,
        init_core0: INIT_CORE0_LDR_B0,
        init_core1: INIT_CORE1_LDR_B1,
        emu_baseline: 7,
        tolerance_override: None,
    },
    DualCoreCase {
        name: "dualcore_spinlock_churn",
        seq_core0: SEQ_CORE0_SPINLOCK,
        seq_core1: SEQ_CORE1_SPINLOCK,
        init_core0: INIT_CORE0_SPINLOCK,
        init_core1: INIT_CORE1_SPINLOCK,
        emu_baseline: 10,
        // Race-dependent: which core wins each contended SPINLOCK0 claim
        // is non-deterministic. Placeholder — tune from silicon data.
        tolerance_override: Some(3),
    },
    DualCoreCase {
        name: "dualcore_fifo_xfer",
        seq_core0: SEQ_CORE0_FIFO_PUSH,
        seq_core1: SEQ_CORE1_FIFO_POP,
        init_core0: INIT_CORE0_FIFO,
        init_core1: INIT_CORE1_FIFO,
        emu_baseline: 12,
        // Race-dependent: FIFO push/pop interleaving depends on each
        // core's arrival-time at SIO. Placeholder — tune from silicon.
        tolerance_override: Some(3),
    },
];

// ---------------------------------------------------------------------------
// Args + richer result
// ---------------------------------------------------------------------------

/// Arguments for `run_against`. Mirrors `CycleArgs` — K-delta protocol is
/// identical. `iter_low` / `iter_high` drive the stub's K counter.
#[derive(Clone, Debug)]
pub struct DualCoreArgs {
    pub filter: Option<String>,
    pub iter_low: u32,
    pub iter_high: u32,
    pub tolerance: u32,
}

impl Default for DualCoreArgs {
    fn default() -> Self {
        Self {
            filter: None,
            iter_low: 101,
            iter_high: 201,
            tolerance: 0,
        }
    }
}

/// Rich per-case result retained so the standalone binary can print a
/// detailed table (HW vs EMU m_low / m_high / per_iter, delta, verdict).
///
/// The EMU side always releases core 1 (see `fresh_emulator_dualcore`);
/// a single-core fallback is not modelled.
#[derive(Debug)]
pub struct DualCoreCaseResult {
    pub name: &'static str,
    pub hw_low: u32,
    pub hw_high: u32,
    pub hw_per_iter: u32,
    pub emu_low: u32,
    pub emu_high: u32,
    pub emu_per_iter: u32,
    pub emu_baseline: u32,
    pub delta: i64,
    pub verdict: Verdict,
    pub elapsed_ms: u32,
}

// ---------------------------------------------------------------------------
// Antagonist bytes — pack `seq_core1` + infinite-branch sentinel.
// ---------------------------------------------------------------------------

/// Pack a core-1 antagonist sequence into a byte stream with an infinite
/// `b -2` (0xE7FE) appended. That tail halfword is the "branch to self"
/// that makes core 1 loop forever until the runner halts it explicitly.
///
/// `b -2` (T2 unconditional): halfword offset = -2 bytes from PC, PC
/// being instruction address + 4. `imm11_hw = target - PC = 0 - 2 = -2`
/// → two's-complement 11-bit = 0x7FE. Encoding 0xE000 | 0x7FE = 0xE7FE.
pub fn pack_antagonist(seq: &[u16]) -> Vec<u8> {
    let mut out = Vec::with_capacity((seq.len() + 1) * 2);
    for hw in seq {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    out.extend_from_slice(&0xE7FEu16.to_le_bytes()); // b -2 (loop-to-self)
    out
}

// ---------------------------------------------------------------------------
// Hardware-side orchestration
// ---------------------------------------------------------------------------

/// Release core 1 at the antagonist sequence. The antagonist has its
/// own infinite tail so no further supervision is needed on silicon; the
/// runner halts core 1 after core 0's measurement completes.
///
/// **Assumption 1 dependency.** This oracle relies on per-core CYCCNT
/// being routed independently on RP2354 (ARMv8-M DWT is per-core in
/// spec). `smoke_per_core_cyccnt_rp2350` is the standalone check. Core 0
/// reads CYCCNT while core 1 runs; if CYCCNT is aliased, core 0's
/// cycles get polluted by core 1's workload and the per_iter delta
/// becomes meaningless. See HLD v1.1.1 §"Critical: Assumption 1".
fn release_core1_hw(
    session: &mut Session,
    init_core1: &[(u8, u32)],
) -> Result<(), Box<dyn std::error::Error>> {
    let mut c1 = session.core(1)?;
    // Halt first — probe-rs needs the core halted to write registers.
    // If it's already halted (reset_and_halt pins it), this is a no-op
    // modulo a timeout.
    if !c1.status()?.is_halted() {
        c1.halt(Duration::from_millis(200))?;
    }
    // Prime per-case registers.
    for &(idx, val) in init_core1 {
        c1.write_core_reg(reg_id(idx), val)?;
    }
    // Prime PC / SP / xPSR / LR.
    c1.write_core_reg(PC_REG, DUALCORE_ANTAGONIST_SLOT)?;
    c1.write_core_reg(XPSR_REG, 0x0100_0000u32)?; // T=1
    c1.write_core_reg(SP_REG, DUALCORE_CORE1_STACK)?;
    c1.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;
    // Shed bootrom-inherited CONTROL state (PSP mode, unprivileged,
    // FPCA/SFPA). See EXTRA_REG doc; mirrors the Stage 1 smoke binary.
    c1.write_core_reg(EXTRA_REG, 0u32)?;
    // Release.
    c1.run()?;
    Ok(())
}

/// Halt core 1 post-measurement. Idempotent — a double-halt just returns
/// the same halted state.
fn halt_core1_hw(session: &mut Session) -> Result<(), Box<dyn std::error::Error>> {
    let mut c1 = session.core(1)?;
    let _ = c1.halt(Duration::from_millis(200));
    Ok(())
}

/// Run core 0's K-delta measurement for this case. Shape mirrors
/// `cycle_cases::measure_hw` but inlined here because we need to own
/// the pre-measurement core-0 register prime for `init_core0` (which
/// wouldn't otherwise be honoured: the stub's BLX to seq is what
/// consumes r0..r3).
fn measure_core0_hw(
    core: &mut Core,
    init_core0: &[(u8, u32)],
    k: u32,
) -> Result<u32, Box<dyn std::error::Error>> {
    // Prime stub entry point and frame.
    core.write_core_reg(PC_REG, STUB_START)?;
    core.write_core_reg(XPSR_REG, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP_REG, EMU_TEST_STACK)?;
    core.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;

    // Per-case core-0 register primes. These values are the seq body's
    // r0..r3 view — the stub enters with BLX so r0..r3 passthrough.
    for &(idx, val) in init_core0 {
        core.write_core_reg(reg_id(idx), val)?;
    }

    // Kick the mailbox and wait for DONE. Mailbox protocol matches
    // cycle_cases — GO=1, DONE=0, SEQ_PTR=CYCLE_SEQ_SLOT|1, ITER=K.
    core.write_word_32(
        (CYCLE_MAILBOX_BASE + cycle_cases::MBX_DONE) as u64,
        0,
    )?;
    core.write_word_32(
        (CYCLE_MAILBOX_BASE + cycle_cases::MBX_CYCLES) as u64,
        0,
    )?;
    core.write_word_32(
        (CYCLE_MAILBOX_BASE + cycle_cases::MBX_SEQ_PTR) as u64,
        CYCLE_SEQ_SLOT | 1,
    )?;
    core.write_word_32(
        (CYCLE_MAILBOX_BASE + cycle_cases::MBX_ITER) as u64,
        k,
    )?;
    core.write_word_32((CYCLE_MAILBOX_BASE + cycle_cases::MBX_GO) as u64, 1)?;

    // NOTE: Assumption 1 dependency — this CYCCNT read assumes the ARMv8-M
    // DWT is per-core on RP2354 (core 0's CYCCNT is not polluted by core
    // 1's concurrent execution). If the assumption fails, the measured
    // delta is indeterminate and the oracle reports accordingly; the
    // `smoke_per_core_cyccnt_rp2350` standalone check validates this
    // before the catalogue runs in anger.
    core.run()?;
    let deadline = Instant::now() + DONE_TIMEOUT;
    loop {
        let done: u32 =
            core.read_word_32((CYCLE_MAILBOX_BASE + cycle_cases::MBX_DONE) as u64)?;
        if done == 1 {
            break;
        }
        if Instant::now() > deadline {
            let _ = core.halt(Duration::from_millis(200));
            return Err(format!("timeout waiting for stub DONE=1 (k={k})").into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    core.halt(Duration::from_millis(200))?;
    let cycles: u32 =
        core.read_word_32((CYCLE_MAILBOX_BASE + cycle_cases::MBX_CYCLES) as u64)?;
    Ok(cycles)
}

// ---------------------------------------------------------------------------
// EMU-side orchestration
// ---------------------------------------------------------------------------

/// Build a fresh emulator with the cycle-oracle stub + core-0 sequence
/// already loaded (via `cycle_cases::fresh_emulator`), then additionally
/// upload the core-1 antagonist and release core 1 with per-case init.
/// `cores[1].wake()` after setting PC/SP/xPSR is all the emulator needs
/// to put core 1 into the antagonist's infinite loop.
fn fresh_emulator_dualcore(
    seq_core0_bytes: &[u8],
    antagonist_bytes: &[u8],
    init_core0: &[(u8, u32)],
    init_core1: &[(u8, u32)],
) -> Emulator {
    let mut emu = fresh_emulator_cycle(seq_core0_bytes);

    // Core-0 per-case register primes — the stub's BLX passes r0..r3 to
    // seq unmodified.
    for &(idx, val) in init_core0 {
        if (idx as usize) < 13 {
            emu.cores[0].regs.r[idx as usize] = val;
        }
    }

    // Upload the antagonist at DUALCORE_ANTAGONIST_SLOT.
    let slot_off = DUALCORE_ANTAGONIST_SLOT - 0x2000_0000;
    for (i, &b) in antagonist_bytes.iter().enumerate() {
        emu.bus.memory.sram_write8(slot_off + i as u32, b);
    }

    // Core-1 register primes.
    for &(idx, val) in init_core1 {
        if (idx as usize) < 13 {
            emu.cores[1].regs.r[idx as usize] = val;
        }
    }
    emu.cores[1].regs.set_pc(DUALCORE_ANTAGONIST_SLOT);
    emu.cores[1].regs.r[13] = DUALCORE_CORE1_STACK;
    emu.cores[1].regs.msp = DUALCORE_CORE1_STACK;
    emu.cores[1].regs.r[14] = 0xFFFF_FFFF;
    emu.cores[1].regs.xpsr = 0x0100_0000; // T=1

    // Release core 1. `wake()` only clears the halted flag — caller must
    // have set PC/SP/xpsr first, which we just did.
    emu.cores[1].wake();

    emu
}

// ---------------------------------------------------------------------------
// Public runner
// ---------------------------------------------------------------------------

/// Library entry point for `silicon_dualcore_diff_rp2350` and future
/// `test_silicon` orchestrator integration.
///
/// **Assumption 1 dependency.** This runner reads DWT CYCCNT from core 0
/// to measure the core-0 instruction window while core 1 runs the
/// antagonist. That is only meaningful if CYCCNT on RP2354 is per-core
/// (ARMv8-M DWT spec). If CYCCNT is instead aliased across cores, core 0's
/// readings are polluted by core 1's concurrent execution and EVERY case
/// will report HW != EMU. Run `smoke_per_core_cyccnt_rp2350` first to
/// confirm; see `release_core1_hw` / `measure_core0_hw` comments.
///
/// **Cleanup contract**: halts core 1 and releases SIO spinlock 0 on
/// exit, regardless of pass or fail — per HLD v1.1.1 §Cross-oracle
/// state-cleanup contract (dualcore row).
///
/// Preconditions: `core` is halted. `Session` is accessible through the
/// caller for core-1 operations — the runner takes `&mut Session` rather
/// than just `&mut Core` because core 1 lives behind
/// `session.core(1)`.
///
/// Case selection:
/// * `order = None` → catalogue order, `args.filter` applied.
/// * `order = Some(&[name, name, …])` → exact order; `args.filter`
///   ignored for selection. Unknown names are skipped with a single
///   `eprintln!` per name.
///
/// Tolerance: `args.tolerance` is the default; per-case
/// `tolerance_override` (set in the catalogue) takes precedence for
/// race-dependent cases.
pub fn run_against(
    session: &mut Session,
    args: &DualCoreArgs,
    order: Option<&[&str]>,
) -> Result<Vec<CaseOutcome>, Box<dyn std::error::Error>> {
    debug_assert!(args.iter_high > args.iter_low, "iter_high must exceed iter_low");

    // Enable DWT on core 0 once. Idempotent; lets the binary wrapper be
    // minimal and lets future orchestrator integration call us without
    // assuming upstream setup.
    {
        let mut c0 = session.core(0)?;
        enable_cyccnt(&mut c0)?;

        // Write the shared measurement stub + zero the mailbox.
        let stub_bytes = pack_stub();
        c0.write_8(STUB_START as u64, &stub_bytes)?;
        for off in [
            cycle_cases::MBX_GO,
            cycle_cases::MBX_DONE,
            cycle_cases::MBX_SEQ_PTR,
            cycle_cases::MBX_ITER,
            cycle_cases::MBX_CYCLES,
            cycle_cases::MBX_RESERVED,
        ] {
            c0.write_word_32((CYCLE_MAILBOX_BASE + off) as u64, 0)?;
        }
    }

    let selected: Vec<&DualCoreCase> = match order {
        None => CASES
            .iter()
            .filter(|c| silicon_oracle::name_matches_filter(c.name, args.filter.as_deref()))
            .collect(),
        Some(names) => {
            let mut v: Vec<&DualCoreCase> = Vec::with_capacity(names.len());
            for name in names {
                match CASES.iter().find(|c| c.name == *name) {
                    Some(c) => v.push(c),
                    None => eprintln!(
                        "dualcore_cases::run_against: unknown case '{name}' in order list; skipping",
                    ),
                }
            }
            v
        }
    };

    let mut outcomes: Vec<CaseOutcome> = Vec::with_capacity(selected.len());
    for case in selected {
        // Per-case tolerance: race-dependent cases (spinlock / FIFO)
        // can declare their own wider band via `tolerance_override`;
        // strict cases fall back to the CLI-default `args.tolerance`.
        let effective_tol = case.tolerance_override.unwrap_or(args.tolerance);
        match run_dualcore_case(session, case, args.iter_low, args.iter_high, effective_tol) {
            Ok(r) => {
                let detail = if r.verdict == Verdict::Pass {
                    String::new()
                } else {
                    format!(
                        "hw={} emu={} delta={:+} tol={}",
                        r.hw_per_iter,
                        r.emu_per_iter,
                        r.delta,
                        effective_tol,
                    )
                };
                outcomes.push(CaseOutcome {
                    oracle: "dualcore",
                    case: r.name,
                    verdict: r.verdict,
                    detail,
                    elapsed_ms: r.elapsed_ms,
                });
            }
            Err(e) => {
                // Per-case error: log and mark fail. Still clean up core 1
                // / spinlock at the end.
                outcomes.push(CaseOutcome::fail(
                    "dualcore",
                    case.name,
                    format!("error: {e}"),
                    0,
                ));
                // Best-effort halt so the next case starts fresh.
                let _ = halt_core1_hw(session);
            }
        }
    }

    // Cleanup (HLD v1.1.1 §Cross-oracle state-cleanup contract: dualcore
    // row). Halt core 1 unconditionally; release SPINLOCK 0. The
    // catalogue only touches spinlock 0 in v1; if a future case touches
    // more, extend this list.
    let _ = halt_core1_hw(session);
    if let Ok(mut c0) = session.core(0) {
        // Any write releases a SIO spinlock. 1 is a canonical value.
        let _ = c0.write_word_32(0xD000_0100u64, 1);
    }

    Ok(outcomes)
}

/// Run one dualcore case end-to-end (HW + EMU) and build the rich result.
fn run_dualcore_case(
    session: &mut Session,
    case: &DualCoreCase,
    iter_low: u32,
    iter_high: u32,
    tolerance: u32,
) -> Result<DualCoreCaseResult, Box<dyn std::error::Error>> {
    let t0 = Instant::now();

    // Upload the core-0 sequence to CYCLE_SEQ_SLOT.
    let seq_bytes = pack_seq(case.seq_core0);
    let antagonist_bytes = pack_antagonist(case.seq_core1);

    {
        let mut c0 = session.core(0)?;
        c0.write_8(CYCLE_SEQ_SLOT as u64, &seq_bytes)?;
        // Upload core-1 antagonist as a byte stream.
        c0.write_8(DUALCORE_ANTAGONIST_SLOT as u64, &antagonist_bytes)?;
    }

    // Release core 1 BEFORE core 0's measurement begins.
    release_core1_hw(session, case.init_core1)?;

    // Measure core 0 at K_low and K_high. Brackets release the &mut Core
    // handle between measurements so core 1 can be re-checked / halted.
    let hw_low = {
        let mut c0 = session.core(0)?;
        measure_core0_hw(&mut c0, case.init_core0, iter_low)?
    };
    let hw_high = {
        let mut c0 = session.core(0)?;
        measure_core0_hw(&mut c0, case.init_core0, iter_high)?
    };
    let hw_per_iter = (hw_high - hw_low) / (iter_high - iter_low);

    // Halt core 1 before moving to the EMU side.
    halt_core1_hw(session)?;

    // EMU side — same stub, same sequence, core 1 also released.
    let mut emu = fresh_emulator_dualcore(
        &seq_bytes,
        &antagonist_bytes,
        case.init_core0,
        case.init_core1,
    );
    let emu_low = measure_emu_cycle(&mut emu, CYCLE_SEQ_SLOT, iter_low)?;
    let emu_high = measure_emu_cycle(&mut emu, CYCLE_SEQ_SLOT, iter_high)?;
    let emu_per_iter = (emu_high - emu_low) / (iter_high - iter_low);

    let delta = hw_per_iter as i64 - emu_per_iter as i64;
    let verdict = if (delta.unsigned_abs() as u32) <= tolerance {
        Verdict::Pass
    } else {
        Verdict::Fail
    };

    let elapsed_ms = t0.elapsed().as_millis().min(u32::MAX as u128) as u32;
    Ok(DualCoreCaseResult {
        name: case.name,
        hw_low,
        hw_high,
        hw_per_iter,
        emu_low,
        emu_high,
        emu_per_iter,
        emu_baseline: case.emu_baseline,
        delta,
        verdict,
        elapsed_ms,
    })
}

// ---------------------------------------------------------------------------
// Standalone per-case runner (exposed for the binary's diagnostic table)
// ---------------------------------------------------------------------------

/// Run one case and produce a `DualCoreCaseResult`. Thin wrapper that
/// lets the binary print HW m_low/m_high + EMU m_low/m_high per case
/// without going through the `Vec<CaseOutcome>` shape of `run_against`.
///
/// `args_tolerance` is the CLI-default tolerance; if the case declares
/// a `tolerance_override`, that takes precedence (same policy as
/// `run_against`).
///
/// Caller must have:
///   * Attached and reset the session (`core.reset_and_halt`).
///   * Enabled CYCCNT on core 0.
///   * Uploaded the stub + zeroed the mailbox (`run_against` does this;
///     the binary mirrors it).
pub fn run_case_rich(
    session: &mut Session,
    case: &DualCoreCase,
    iter_low: u32,
    iter_high: u32,
    args_tolerance: u32,
) -> Result<DualCoreCaseResult, Box<dyn std::error::Error>> {
    let effective_tol = case.tolerance_override.unwrap_or(args_tolerance);
    run_dualcore_case(session, case, iter_low, iter_high, effective_tol)
}

/// Resolve the effective tolerance for a case: the case's
/// `tolerance_override` if set, otherwise the CLI-default `args_tolerance`.
/// Exposed so the binary can show the per-case tolerance column.
pub fn effective_tolerance(case: &DualCoreCase, args_tolerance: u32) -> u32 {
    case.tolerance_override.unwrap_or(args_tolerance)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    // (1) Catalogue presence: 4 cases, all start with `dualcore_` prefix.
    #[test]
    fn test_catalogue_size_and_prefix() {
        assert_eq!(CASES.len(), 4, "catalogue must carry 4 cases in v1");
        for c in CASES {
            assert!(
                c.name.starts_with("dualcore_"),
                "case '{}' must start with 'dualcore_' prefix (substring-uniqueness)",
                c.name,
            );
        }
    }

    // (2) Register indices in init tables are valid (0..=15).
    //
    // The stub reserves r4..r7 as callee-saved loop state on core 0; the
    // catalogue therefore must not set those indices on core 0. Core 1
    // runs the antagonist in isolation (no stub framing), so r0..=r12 is
    // the full valid range there.
    #[test]
    fn test_init_register_indices_are_valid() {
        for c in CASES {
            for &(idx, _) in c.init_core0 {
                assert!(
                    idx <= 15,
                    "case '{}' init_core0 has out-of-range register r{idx}",
                    c.name,
                );
                assert!(
                    !(4..=7).contains(&idx),
                    "case '{}' init_core0 touches r{idx}; stub reserves r4..r7",
                    c.name,
                );
            }
            for &(idx, _) in c.init_core1 {
                assert!(
                    idx <= 15,
                    "case '{}' init_core1 has out-of-range register r{idx}",
                    c.name,
                );
            }
        }
    }

    // (3) Antagonist tails are well-formed. `pack_antagonist` MUST append
    //     `0xE7FE` (B -2 = branch-to-self) so core 1 loops forever until
    //     explicitly halted.
    #[test]
    fn test_antagonist_packing_appends_infinite_branch() {
        for c in CASES {
            let bytes = pack_antagonist(c.seq_core1);
            let last = u16::from_le_bytes([bytes[bytes.len() - 2], bytes[bytes.len() - 1]]);
            assert_eq!(
                last, 0xE7FE,
                "case '{}' antagonist must end in B -2 (0xE7FE); got 0x{:04X}",
                c.name, last,
            );
            // Also: total length is (seq_core1.len() + 1) * 2 bytes.
            assert_eq!(
                bytes.len(),
                (c.seq_core1.len() + 1) * 2,
                "case '{}' antagonist byte stream wrong size",
                c.name,
            );
        }
    }

    // (4) `b -2` encoding is computed correctly. Proof against a future
    //     refactor of `pack_antagonist`.
    #[test]
    fn test_infinite_branch_encoding() {
        // Thumb-16 B T2: `0b11100_imm11`. For target=instruction_address
        // (i.e. loop to self), `PC = instr_addr + 4`, so `imm11_hw = (0 -
        // 4) / 2 = -2`. Two's-complement 11-bit of -2 = 0x7FE.
        // Encoding: 0xE000 | 0x7FE = 0xE7FE.
        let imm11_hw: i32 = -2;
        let encoded = 0xE000u16 | ((imm11_hw as i16) as u16 & 0x07FF);
        assert_eq!(encoded, 0xE7FE, "b -2 encoding must be 0xE7FE");
    }

    // (5) Substring-uniqueness within the catalogue. The orchestrator's
    //     whole-catalogue validator (`test_silicon.rs`) runs over all
    //     oracles; this test fires within the dualcore catalogue alone so
    //     a bad rename inside this module fails before the orchestrator
    //     ever sees it.
    #[test]
    fn test_catalogue_names_substring_unique() {
        let mut seen: HashSet<&str> = HashSet::new();
        for c in CASES {
            assert!(
                seen.insert(c.name),
                "duplicate dualcore case name '{}'",
                c.name,
            );
        }
        // Short-in-long check.
        for c1 in CASES {
            for c2 in CASES {
                if c1.name == c2.name {
                    continue;
                }
                assert!(
                    !c1.name.contains(c2.name),
                    "case '{}' is a substring of '{}'; filter aliasing",
                    c2.name, c1.name,
                );
            }
        }
    }

    // (6) `seq_core0` follows the same "no trailing bx lr" contract as
    //     `CycleCase::seq` — the runner appends `0x4770` at upload time.
    #[test]
    fn test_seq_core0_no_trailing_bxlr() {
        for c in CASES {
            let last = *c.seq_core0.last().expect("seq_core0 must be non-empty");
            assert_ne!(
                last, 0x4770,
                "case '{}' seq_core0 must not end in bx lr (runner appends it)",
                c.name,
            );
        }
    }

    // (7) Core 1 antagonist bodies are non-empty (otherwise the runner's
    //     appended `b -2` would land on a stub-inherited halfword and the
    //     "antagonist" isn't antagonising anything).
    #[test]
    fn test_seq_core1_non_empty() {
        for c in CASES {
            assert!(
                !c.seq_core1.is_empty(),
                "case '{}' seq_core1 must be non-empty",
                c.name,
            );
        }
    }

    // (8) Race-dependent cases must declare a `tolerance_override`. K-delta
    //     cancels per-invocation framing but NOT the per-iteration race
    //     outcome (which core wins each contended cycle). Strict cases
    //     (non-race) opt out with `None`.
    #[test]
    fn test_race_cases_have_tolerance_override() {
        for c in CASES {
            let is_race =
                c.name.contains("spinlock") || c.name.contains("fifo");
            if is_race {
                assert!(
                    c.tolerance_override.is_some(),
                    "race-dependent case '{}' must declare a tolerance_override",
                    c.name,
                );
            }
        }
    }

    // (9) `effective_tolerance` honours the override when set, and
    //     otherwise falls back to the CLI default.
    #[test]
    fn test_effective_tolerance_override_vs_fallback() {
        for c in CASES {
            let with_default = effective_tolerance(c, 0);
            match c.tolerance_override {
                Some(v) => assert_eq!(
                    with_default, v,
                    "case '{}' override ({}) should beat CLI default (0)",
                    c.name, v,
                ),
                None => assert_eq!(
                    with_default, 0,
                    "case '{}' has no override; effective tolerance must fall back to CLI default",
                    c.name,
                ),
            }
            // CLI override should beat the None case.
            let fallback = effective_tolerance(c, 42);
            match c.tolerance_override {
                Some(v) => assert_eq!(fallback, v),
                None => assert_eq!(fallback, 42),
            }
        }
    }
}

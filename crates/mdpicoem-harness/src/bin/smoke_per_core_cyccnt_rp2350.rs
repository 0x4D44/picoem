// smoke_per_core_cyccnt_rp2350 — verifier for Assumption 1 of
// `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage
// Expansion.md`: DWT CYCCNT on RP2354 is independently available on core 1
// at the standard `0xE000_1004` alias.
//
// Differentiator design: a symmetric "enable + zero + run + read on each
// core" shape cannot tell per-core CYCCNT apart from a CYCCNT alias — both
// produce a PASS. We instead put the two cores in visibly different counter
// states before reading: (1) core 0 enables DWT, loads an infinite busy-loop
// stub, zeroes CYCCNT, verifies the zero stuck, and runs for 100 ms;
// (2) core 1 is halted (NOT reset_and_halt — that would clobber core 0)
// and its CYCCNT alias is read WITHOUT ever enabling DWT on core 1;
// (3) core 0 is halted and its CYCCNT read.
//
// Verdicts:
//   N1 == 0                     — Assumption 1 HOLDS (per-core DWT).
//   N1 within ~5% of N0         — Assumption 1 FAILS (aliased DWT).
//   N1 non-zero and far from N0 — INDETERMINATE; investigate.
//
// Stage 1 scaffolding, not a permanent oracle. Delete after Arthur runs it
// once and we either confirm core-1 CYCCNT works or fall back to the
// wall-clock plan noted in the HLD.

use mdpicoem_harness::{EMU_TEST_SLOT, EMU_TEST_STACK};
use probe_rs::{MemoryInterface, RegisterId, Session, SessionConfig};
use std::time::Duration;

const DEMCR: u64 = 0xE000_EDFC;
const DWT_CTRL: u64 = 0xE000_1000;
const DWT_CYCCNT: u64 = 0xE000_1004;
const CPUID: u64 = 0xE000_ED00;
const TRCENA: u32 = 1 << 24;
const CYCCNTENA: u32 = 1 << 0;

// ARM core register IDs. EXTRA (0b10100) packs {CONTROL, FAULTMASK, BASEPRI,
// PRIMASK}; writing 0 zeroes all four (thread-mode / MSP / privileged).
const PC: RegisterId = RegisterId(15);
const XPSR: RegisterId = RegisterId(16);
const SP: RegisterId = RegisterId(13);
const LR: RegisterId = RegisterId(14);
const EXTRA: RegisterId = RegisterId(0b10100);

// 10 × NOP (0xBF00) + B -22 back to the first NOP. B imm11 encoding is
// `11100 imm11` with target = PC_of_B + 4 + (imm11 << 1); B at offset 20
// (PC=24) to target 0 gives imm11 = -12 = 0x7F4 → halfword 0xE7F4.
const STUB: [u8; 22] = [
    0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF,
    0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF,
    0xF4, 0xE7,
];

const SPIN_DURATION: Duration = Duration::from_millis(100);
const HALT_TIMEOUT: Duration = Duration::from_millis(500);
// Aliased-counter tolerance: if N1 is within ±5% of N0, both reads are
// observing the same hardware counter.
const ALIAS_REL_TOL: f64 = 0.05;

fn run() -> Result<i32, Box<dyn std::error::Error>> {
    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;

    // Phase 1: enable DWT on core 0, prime the busy-loop stub, verify the
    // CYCCNT-zero write stuck, then release core 0 to spin.
    let cpuid0 = {
        let mut c0 = session.core(0)?;
        c0.reset_and_halt(HALT_TIMEOUT)?;
        let demcr = c0.read_word_32(DEMCR)?;
        c0.write_word_32(DEMCR, demcr | TRCENA)?;
        let ctrl = c0.read_word_32(DWT_CTRL)?;
        c0.write_word_32(DWT_CTRL, ctrl | CYCCNTENA)?;
        c0.write_8(EMU_TEST_SLOT as u64, &STUB)?;
        c0.write_core_reg(PC, EMU_TEST_SLOT)?;
        c0.write_core_reg(XPSR, 0x0100_0000u32)?; // T=1
        c0.write_core_reg(SP, EMU_TEST_STACK)?;
        c0.write_core_reg(LR, EMU_TEST_SLOT | 1)?; // not EXC_RETURN; re-enter stub
        c0.write_core_reg(EXTRA, 0u32)?; // CONTROL/FAULTMASK/BASEPRI/PRIMASK = 0
        c0.write_word_32(DWT_CYCCNT, 0)?;
        let zeroed = c0.read_word_32(DWT_CYCCNT)?;
        if zeroed != 0 {
            return Err(format!("core 0: CYCCNT did not zero (read 0x{zeroed:08X})").into());
        }
        let cpuid = c0.read_word_32(CPUID)?;
        c0.run()?;
        cpuid
    };
    std::thread::sleep(SPIN_DURATION);

    // Phase 2: halt core 1 (NOT reset_and_halt — that resets the chip and
    // clobbers core 0), sanity-check CPUID routing, read its CYCCNT WITHOUT
    // ever enabling DWT on core 1.
    let (n1, cpuid1) = {
        let mut c1 = session.core(1)?;
        if !c1.status()?.is_halted() {
            c1.halt(HALT_TIMEOUT)?;
        }
        let cpuid = c1.read_word_32(CPUID)?;
        let n1 = c1.read_word_32(DWT_CYCCNT)?;
        (n1, cpuid)
    };

    // Phase 3: halt core 0 and read its CYCCNT.
    let n0 = {
        let mut c0 = session.core(0)?;
        c0.halt(HALT_TIMEOUT)?;
        c0.read_word_32(DWT_CYCCNT)?
    };

    println!("CPUID core 0 = 0x{cpuid0:08X}, CPUID core 1 = 0x{cpuid1:08X}");
    println!("core 0 cyccnt = {n0} (0x{n0:08X})");
    println!("core 1 cyccnt = {n1} (0x{n1:08X})");

    if n1 == 0 {
        println!("Assumption 1 HOLDS — per-core CYCCNT confirmed.");
        return Ok(0);
    }
    if n0 != 0 {
        let rel = (n1 as f64 - n0 as f64).abs() / n0 as f64;
        if rel <= ALIAS_REL_TOL {
            println!(
                "Assumption 1 FAILS — CYCCNT appears aliased (|ΔN|/N0 = {rel:.3} ≤ \
                 {ALIAS_REL_TOL:.3}); dualcore oracle must use a different measurement strategy."
            );
            return Ok(1);
        }
    }
    println!("Assumption 1 INDETERMINATE — N1=0x{n1:08X} unexpected relative to N0=0x{n0:08X}.");
    Ok(1)
}

fn main() {
    mdpicoem_harness::harness_tracing_init();
    match run() {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("fatal: {e}");
            std::process::exit(2);
        }
    }
}

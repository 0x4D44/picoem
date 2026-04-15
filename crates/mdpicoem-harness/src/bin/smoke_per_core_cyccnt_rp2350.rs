// smoke_per_core_cyccnt_rp2350 — one-shot verifier for Assumption 1 of
// `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage
// Expansion.md` (§Assumptions): DWT CYCCNT on RP2354 is independently
// available on core 1 at the standard `0xE000_1004` alias.
//
// For each core (0 then 1): reset_and_halt, enable DWT, write a tiny
// Thumb stub (6 NOPs + BKPT #0) at `EMU_TEST_SLOT`, zero CYCCNT, prime
// PC/XPSR/SP/LR, resume, wait for the BKPT halt, read CYCCNT. A value
// in 5..=20 is a PASS; anything else is a FAIL. Exits 0 iff both PASS.
//
// This is Stage 1 scaffolding, not a permanent oracle. Delete after
// Arthur runs it once and we either confirm core-1 CYCCNT works or fall
// back to the wall-clock plan noted in the HLD.

use mdpicoem_harness::EMU_TEST_STACK;
use probe_rs::{Core, MemoryInterface, RegisterId, Session, SessionConfig};
use std::time::{Duration, Instant};

// DWT / CoreDebug MMIO (match `silicon_periph_diff_rp2350.rs:35-39`).
const DEMCR: u64 = 0xE000_EDFC;
const DWT_CTRL: u64 = 0xE000_1000;
const DWT_CYCCNT: u64 = 0xE000_1004;
const TRCENA: u32 = 1 << 24;
const CYCCNTENA: u32 = 1 << 0;

// ARM core register IDs.
const PC: RegisterId = RegisterId(15);
const XPSR: RegisterId = RegisterId(16);
const SP: RegisterId = RegisterId(13);
const LR: RegisterId = RegisterId(14);

// Stub: 6 × NOP (0xBF00) + BKPT #0 (0xBE00), all 16-bit Thumb halfwords.
const STUB: [u8; 14] = [
    0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBE,
];
const STUB_ADDR: u32 = mdpicoem_harness::EMU_TEST_SLOT;

const BKPT_TIMEOUT: Duration = Duration::from_secs(5);
// 6 NOPs (1 cycle each) + BKPT/halt framing is well under 20; anything
// well below 5 would suggest the counter is stuck or not ticking.
const CYCCNT_SANE_LOW: u32 = 5;
const CYCCNT_SANE_HIGH: u32 = 20;

fn enable_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    let demcr: u32 = core.read_word_32(DEMCR)?;
    core.write_word_32(DEMCR, demcr | TRCENA)?;
    let ctrl: u32 = core.read_word_32(DWT_CTRL)?;
    core.write_word_32(DWT_CTRL, ctrl | CYCCNTENA)?;
    Ok(())
}

fn run_one_core(session: &mut Session, id: usize) -> Result<u32, Box<dyn std::error::Error>> {
    let mut core = session.core(id)?;
    core.reset_and_halt(Duration::from_millis(500))?;
    enable_cyccnt(&mut core)?;
    core.write_8(STUB_ADDR as u64, &STUB)?;
    core.write_word_32(DWT_CYCCNT, 0)?;
    core.write_core_reg(PC, STUB_ADDR)?;
    core.write_core_reg(XPSR, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP, EMU_TEST_STACK)?;
    core.write_core_reg(LR, 0xFFFF_FFFFu32)?;
    core.run()?;
    let deadline = Instant::now() + BKPT_TIMEOUT;
    loop {
        if core.status()?.is_halted() {
            break;
        }
        if Instant::now() > deadline {
            let _ = core.halt(Duration::from_millis(200));
            let pc: u32 = core.read_core_reg(PC).unwrap_or(0xDEAD_BEEF);
            return Err(format!("core {id}: BKPT timeout, PC=0x{pc:08X}").into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    let cyccnt: u32 = core.read_word_32(DWT_CYCCNT)?;
    Ok(cyccnt)
}

fn main() {
    let mut session = match Session::auto_attach("rp2350", SessionConfig::default()) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("fatal: auto_attach failed: {e}");
            std::process::exit(2);
        }
    };
    let mut all_pass = true;
    for id in 0..=1usize {
        match run_one_core(&mut session, id) {
            Ok(n) => {
                let pass = (CYCCNT_SANE_LOW..=CYCCNT_SANE_HIGH).contains(&n);
                let verdict = if pass { "PASS" } else { "FAIL" };
                println!("core {id}: CYCCNT after 6 NOPs + BKPT = {n}  [{verdict}]");
                if !pass {
                    all_pass = false;
                }
            }
            Err(e) => {
                eprintln!("core {id}: error: {e}  [FAIL]");
                all_pass = false;
            }
        }
    }
    // Halt both cores on exit so the board is in a known state.
    for id in 0..=1usize {
        if let Ok(mut core) = session.core(id) {
            let _ = core.halt(Duration::from_millis(200));
        }
    }
    std::process::exit(if all_pass { 0 } else { 1 });
}

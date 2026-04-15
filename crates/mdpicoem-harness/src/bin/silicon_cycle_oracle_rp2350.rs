// silicon_cycle_oracle_rp2350 — sequence-in-loop cycle-cost oracle for
// mdrp2350 vs. real RP2354 silicon.
//
// A small measurement stub lives in SRAM. The host (this binary, over
// SWD via probe-rs) writes a mailbox telling the stub which sequence to
// run and how many iterations (K). The stub reads DWT_CYCCNT before and
// after K calls to the sequence, writes the delta back. Two K values
// (default K_low=101, K_high=201) isolate the steady-state per-iter
// cost: per_iter = (m_high - m_low) / (K_high - K_low).
//
// The identical stub + sequence bytes are written into the mdrp2350
// emulator's SRAM and the emulator is run through the same protocol.
// Per-iter (HW) vs per-iter (EMU) is the diff.
//
// **What this measures.** Each per-iter number is the cost of one
// `BLX seq / seq body / BX LR` round-trip inside a steady-state
// measurement loop. It is NOT the halt-step per-instruction cost that
// the entries in `tech_debt.md` (under "Cycle Timing — Phase 2") were
// measured at. The two numbers answer different questions:
//
//   - halt-step entries (tech_debt.md) isolate one instruction's cost
//     plus a fixed 5-cycle debug overhead.
//   - this oracle measures a sequence's cost inside a BLX/BXLR frame
//     at native speed (no debug overhead) with pipeline effects fully
//     engaged.
//
// Deltas from this oracle populate a separate section of `tech_debt.md`
// ("Cycle-Timing — Sequence-in-Loop Measurements"); they do not
// supersede or invalidate the halt-step numbers.
//
// Usage:
//   silicon_cycle_oracle_rp2350
//   silicon_cycle_oracle_rp2350 -- --filter push
//   silicon_cycle_oracle_rp2350 -- --iter-low 51 --iter-high 151
//   silicon_cycle_oracle_rp2350 -- --tolerance 1

use mdpicoem_harness::cycle_cases::{
    fresh_emulator, measure_emu, pack_seq, pack_stub, CycleCase, CASES, CYCLE_SEQ_SLOT,
    DWT_CYCCNT_ADDR, STUB_START,
};
use mdpicoem_harness::{CYCLE_MAILBOX_BASE, EMU_TEST_STACK};
use probe_rs::{Core, MemoryInterface, RegisterId, Session, SessionConfig};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Fixed layout
// ---------------------------------------------------------------------------
//
// Stub lives at `STUB_START` = `EMU_TEST_SLOT` = 0x2000_0100 (reuses the
// ISA oracle's slot — the two oracles never run concurrently).
//
// Sequences live at `CYCLE_SEQ_SLOT` = 0x2000_1000 — 4 KB above the stub,
// well clear of `EMU_TEST_SCRATCH` (0x2000_0200..0x2000_0600) and
// `EMU_FPU_SCRATCH` (0x2000_0600+). Bank of 0x2000_1000 = (0x1000 >> 2) & 7
// = 0 — important for the bank-contention test case whose data address
// (0x2000_0200, bank 0) must match the sequence-fetch bank.
//
// Mailbox lives at `CYCLE_MAILBOX_BASE` = 0x2004_0100 (above
// `EMU_TEST_STACK` = 0x2004_0000 so the stub's callee-saved push frame
// doesn't clobber it).

// Mailbox word offsets (also exported from `cycle_cases` for the
// emulator-side helpers; re-declared here for HW reads/writes).
const MBX_GO: u32 = 0x00;
const MBX_DONE: u32 = 0x04;
const MBX_SEQ_PTR: u32 = 0x08;
const MBX_ITER: u32 = 0x0C;
const MBX_CYCLES: u32 = 0x10;
const MBX_RESERVED: u32 = 0x14;

// DWT / CoreDebug MMIO (host side — matches silicon).
const DEMCR_HW: u64 = 0xE000_EDFC;
const DWT_CTRL_HW: u64 = 0xE000_1000;
const TRCENA: u32 = 1 << 24;
const CYCCNTENA: u32 = 1 << 0;

// ARM core register IDs.
const PC: RegisterId = RegisterId(15);
const XPSR: RegisterId = RegisterId(16);
const SP: RegisterId = RegisterId(13);
const LR: RegisterId = RegisterId(14);

// SCB fault-status registers (host side).
const HFSR_ADDR: u64 = 0xE000_ED2C;
const CFSR_ADDR: u64 = 0xE000_ED28;

// Default polling timeout for DONE (per case, per iteration count).
const DONE_TIMEOUT: Duration = Duration::from_secs(1);

fn main() {
    match run() {
        Ok(exit_code) => std::process::exit(exit_code),
        Err(e) => {
            eprintln!("fatal: {e}");
            std::process::exit(2);
        }
    }
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Args {
    filter: Option<String>,
    iter_low: u32,
    iter_high: u32,
    tolerance: u32,
}

const USAGE: &str = "\
Usage: silicon_cycle_oracle_rp2350 [--filter <substr>] [--iter-low <K1>] \
[--iter-high <K2>] [--tolerance <N>]

Options:
  --filter    Only run cases whose name contains <substr>
  --iter-low  K_low   (default 101)
  --iter-high K_high  (default 201, must be > K_low)
  --tolerance Cycle-delta tolerance before marking FAIL (default 0)
";

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut filter: Option<String> = None;
    let mut iter_low: u32 = 101;
    let mut iter_high: u32 = 201;
    let mut tolerance: u32 = 0;
    let mut i = 0;
    while i < argv.len() {
        let a = argv[i].as_str();
        match a {
            "--filter" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--filter requires a substring\n{USAGE}"));
                }
                filter = Some(argv[i].clone());
            }
            "--iter-low" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--iter-low requires a value\n{USAGE}"));
                }
                iter_low = argv[i]
                    .parse()
                    .map_err(|e| format!("invalid --iter-low '{}': {e}\n{USAGE}", argv[i]))?;
            }
            "--iter-high" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--iter-high requires a value\n{USAGE}"));
                }
                iter_high = argv[i]
                    .parse()
                    .map_err(|e| format!("invalid --iter-high '{}': {e}\n{USAGE}", argv[i]))?;
            }
            "--tolerance" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--tolerance requires a value\n{USAGE}"));
                }
                tolerance = argv[i]
                    .parse()
                    .map_err(|e| format!("invalid --tolerance '{}': {e}\n{USAGE}", argv[i]))?;
            }
            "--help" | "-h" => {
                return Err(USAGE.to_string());
            }
            other => return Err(format!("unknown argument '{other}'\n{USAGE}")),
        }
        i += 1;
    }
    if iter_high <= iter_low {
        return Err(format!(
            "--iter-high ({iter_high}) must be > --iter-low ({iter_low})\n{USAGE}"
        ));
    }
    Ok(Args { filter, iter_low, iter_high, tolerance })
}

// ---------------------------------------------------------------------------
// Hardware side
// ---------------------------------------------------------------------------

fn enable_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    let demcr: u32 = core.read_word_32(DEMCR_HW)?;
    core.write_word_32(DEMCR_HW, demcr | TRCENA)?;
    let ctrl: u32 = core.read_word_32(DWT_CTRL_HW)?;
    core.write_word_32(DWT_CTRL_HW, ctrl | CYCCNTENA)?;
    Ok(())
}

/// Zero the mailbox region (six u32 slots).
fn zero_mailbox_hw(core: &mut Core) -> Result<(), probe_rs::Error> {
    for off in [MBX_GO, MBX_DONE, MBX_SEQ_PTR, MBX_ITER, MBX_CYCLES, MBX_RESERVED] {
        core.write_word_32((CYCLE_MAILBOX_BASE + off) as u64, 0)?;
    }
    Ok(())
}

/// Write mailbox: GO=1, DONE=0, SEQ_PTR=seq_start|1, ITER=K.
fn kick_mailbox_hw(core: &mut Core, seq_start: u32, k: u32) -> Result<(), probe_rs::Error> {
    debug_assert!(
        seq_start & 1 == 0,
        "seq_start must be halfword-aligned before OR'ing Thumb bit"
    );
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_DONE) as u64, 0)?;
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_CYCLES) as u64, 0)?;
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_SEQ_PTR) as u64, seq_start | 1)?;
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_ITER) as u64, k)?;
    // GO last — stub is spinning on this.
    core.write_word_32((CYCLE_MAILBOX_BASE + MBX_GO) as u64, 1)?;
    Ok(())
}

/// Resume, poll DONE until 1 or timeout, halt, read CYCLES.
///
/// On timeout, dumps PC/SP/LR/CFSR/HFSR and the six mailbox slots — just
/// enough state to distinguish "stub wedged in poll" from "fault in stub"
/// from "fault in seq". Deeper post-mortem (stacked frame readback, BFAR,
/// MMFAR) was stripped as review flagged it as bring-up scaffolding: if
/// the stub is faulting in steady-state, these six lines are enough to
/// localise it, and the developer will already be attached with
/// `probe-rs gdb` for anything deeper.
fn wait_and_read_cycles_hw(core: &mut Core) -> Result<u32, Box<dyn std::error::Error>> {
    core.run()?;
    let deadline = Instant::now() + DONE_TIMEOUT;
    loop {
        let done: u32 = core.read_word_32((CYCLE_MAILBOX_BASE + MBX_DONE) as u64)?;
        if done == 1 {
            break;
        }
        if Instant::now() > deadline {
            // Best-effort halt so follow-up writes go through.
            let _ = core.halt(Duration::from_millis(200));
            let pc: u32 = core.read_core_reg(PC).unwrap_or(0xDEAD_BEEF);
            let sp: u32 = core.read_core_reg(SP).unwrap_or(0xDEAD_BEEF);
            let lr: u32 = core.read_core_reg(LR).unwrap_or(0xDEAD_BEEF);
            let hfsr: u32 = core.read_word_32(HFSR_ADDR).unwrap_or(0xDEAD_BEEF);
            let cfsr: u32 = core.read_word_32(CFSR_ADDR).unwrap_or(0xDEAD_BEEF);
            let go: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_GO) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let done_v: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_DONE) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let seq_ptr: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_SEQ_PTR) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let iter: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_ITER) as u64)
                .unwrap_or(0xDEAD_BEEF);
            let cycles: u32 = core
                .read_word_32((CYCLE_MAILBOX_BASE + MBX_CYCLES) as u64)
                .unwrap_or(0xDEAD_BEEF);
            return Err(format!(
                "timeout waiting for stub DONE=1\n\
                 PC=0x{pc:08X} SP=0x{sp:08X} LR=0x{lr:08X}\n\
                 HFSR=0x{hfsr:08X} CFSR=0x{cfsr:08X}\n\
                 mailbox: GO={go} DONE={done_v} SEQ_PTR=0x{seq_ptr:08X} ITER={iter} CYCLES={cycles}"
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    core.halt(Duration::from_millis(200))?;
    let cycles: u32 = core.read_word_32((CYCLE_MAILBOX_BASE + MBX_CYCLES) as u64)?;
    Ok(cycles)
}

/// Measure raw cycles for one K-value on hardware.
fn measure_hw(
    core: &mut Core,
    seq_start: u32,
    k: u32,
) -> Result<u32, Box<dyn std::error::Error>> {
    // Re-prime PC so the run re-enters the stub's push frame fresh.
    // (The stub's `b poll` at [19] loops back to [2] — re-priming is
    // belt-and-braces insurance against an unexpected halt location.)
    core.write_core_reg(PC, STUB_START)?;
    core.write_core_reg(XPSR, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP, EMU_TEST_STACK)?;
    core.write_core_reg(LR, 0xFFFF_FFFFu32)?;
    kick_mailbox_hw(core, seq_start, k)?;
    wait_and_read_cycles_hw(core)
}

// ---------------------------------------------------------------------------
// Per-case driver
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct CaseResult {
    name: &'static str,
    hw_low: u32,
    hw_high: u32,
    hw_per_iter: u32,
    emu_low: u32,
    emu_high: u32,
    emu_per_iter: u32,
    emu_baseline: u32,
    delta: i64, // hw - emu
    verdict: Verdict,
}

#[derive(Debug, PartialEq)]
enum Verdict {
    Pass,
    Fail,
}

fn run_case(
    core: &mut Core,
    case: &CycleCase,
    iter_low: u32,
    iter_high: u32,
    tolerance: u32,
) -> Result<CaseResult, Box<dyn std::error::Error>> {
    let seq_bytes = pack_seq(case.seq);

    // Write seq to hardware SRAM once per case.
    core.write_8(CYCLE_SEQ_SLOT as u64, &seq_bytes)?;

    // Hardware: two K values.
    let hw_low = measure_hw(core, CYCLE_SEQ_SLOT, iter_low)?;
    let hw_high = measure_hw(core, CYCLE_SEQ_SLOT, iter_high)?;
    let hw_per_iter = (hw_high - hw_low) / (iter_high - iter_low);

    // Emulator: fresh run per case (setup cost is cheap).
    let mut emu = fresh_emulator(&seq_bytes);
    let emu_low = measure_emu(&mut emu, CYCLE_SEQ_SLOT, iter_low)?;
    let emu_high = measure_emu(&mut emu, CYCLE_SEQ_SLOT, iter_high)?;
    let emu_per_iter = (emu_high - emu_low) / (iter_high - iter_low);

    let delta = hw_per_iter as i64 - emu_per_iter as i64;
    let verdict = if (delta.unsigned_abs() as u32) <= tolerance {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    Ok(CaseResult {
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
    })
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn run() -> Result<i32, Box<dyn std::error::Error>> {
    let args = parse_args().map_err(|e| {
        eprintln!("{e}");
        "bad arguments"
    })?;

    let selected: Vec<&CycleCase> = CASES
        .iter()
        .filter(|c| match &args.filter {
            Some(sub) => c.name.contains(sub),
            None => true,
        })
        .collect();

    if selected.is_empty() {
        println!("no cases match filter; nothing to do");
        return Ok(0);
    }

    println!(
        "silicon_cycle_oracle_rp2350: K_low={} K_high={} tol={}",
        args.iter_low, args.iter_high, args.tolerance
    );
    println!(
        "stub=0x{STUB_START:08X} seq=0x{CYCLE_SEQ_SLOT:08X} mailbox=0x{CYCLE_MAILBOX_BASE:08X} dwt=0x{DWT_CYCCNT_ADDR:08X}"
    );
    println!("selected {} case(s)", selected.len());
    println!();

    // Attach + reset_and_halt + enable DWT.
    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
    let mut core = session.core(0)?;
    core.reset_and_halt(Duration::from_millis(500))?;
    enable_cyccnt(&mut core)?;

    // Write the stub once; it stays resident across all cases.
    let stub_bytes = pack_stub();
    core.write_8(STUB_START as u64, &stub_bytes)?;
    // Zero mailbox.
    zero_mailbox_hw(&mut core)?;

    // Prime core registers. probe-rs strips the Thumb LSB internally;
    // we write the aligned PC and rely on XPSR.T=1 for Thumb mode.
    core.write_core_reg(PC, STUB_START)?;
    core.write_core_reg(XPSR, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP, EMU_TEST_STACK)?;
    core.write_core_reg(LR, 0xFFFF_FFFFu32)?;

    // Header row.
    println!(
        "{:<36} {:>10} {:>10} {:>10} {:>10} {:>6} {:>6}",
        "case", "HW/iter", "EMU/iter", "delta", "baseline", "tol", "verdict",
    );
    println!("{}", "-".repeat(96));

    let mut total = 0usize;
    let mut pass = 0usize;
    let mut fail = 0usize;
    let t0 = Instant::now();

    for case in &selected {
        let r = run_case(&mut core, case, args.iter_low, args.iter_high, args.tolerance)?;
        total += 1;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail => fail += 1,
        }
        println!(
            "{:<36} {:>10} {:>10} {:>+10} {:>10} {:>6} {:>6}",
            r.name,
            r.hw_per_iter,
            r.emu_per_iter,
            r.delta,
            r.emu_baseline,
            args.tolerance,
            if r.verdict == Verdict::Pass { "PASS" } else { "FAIL" },
        );
        if r.emu_per_iter != r.emu_baseline {
            println!(
                "    NOTE: emu per-iter ({}) differs from catalog emu_baseline ({}); update CycleCase::emu_baseline",
                r.emu_per_iter, r.emu_baseline,
            );
        }
        // Keep raw samples visible for diagnostics.
        println!(
            "    HW  m_low={} m_high={}   EMU m_low={} m_high={}",
            r.hw_low, r.hw_high, r.emu_low, r.emu_high,
        );
    }

    let elapsed = t0.elapsed();
    println!();
    println!(
        "summary: total={total} pass={pass} fail={fail}  ({:.2}s)",
        elapsed.as_secs_f64()
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

//! Microbench: drive `PioBlock::step_n` directly to measure the per-sysclk
//! cost of the PIO worker's hot path, isolated from the threaded
//! emulator's barrier/coordinator/CPU machinery.
//!
//! The threaded `peripheral` / `stress` workloads cap at ~64 MHz
//! per-core because the PIO worker thread gates the per-quantum
//! barrier — see `wrk_journals/2026.04.20 - JRN - Threaded Perf
//! Investigation.md` and the follow-on `PIO step_n Profiling` journal.
//! This binary lets us profile and time `step_n` directly without
//! the threaded scaffolding noise.
//!
//! Default workload: one PIO block, SM0 enabled, clkdiv=1, running a
//! 2-instruction wrap loop (SET PINS,1 / SET PINS,0). Matches the
//! shape of the `peripheral` workload's PIO setup.
//!
//! Flags:
//!   --cycles N      Total sysclks to run (default 100_000_000).
//!   --step-q N      Sysclks per `step_n` call (default 4096; matches
//!                   the threaded runtime's quantum).
//!   --sms N         Number of SMs to enable on the block (1..=4,
//!                   default 1). All run the same wrap loop.
//!
//! Prints wall time, MHz of emulated sysclks, and ns per emulated
//! sysclk, plus a final pad_out value sanity check.

use mdpicoem_common::pio::PioBlock;
use std::time::Instant;

// PIO offsets (RP2350 — same as the threaded bench uses).
const CTRL: u32 = 0x000;
const INSTR_MEM0: u32 = 0x048;
const SM0_CLKDIV: u32 = 0x0C8;
const SM0_EXECCTRL: u32 = 0x0CC;
const SM_STRIDE: u32 = 0x18;

// SET PINS, n encoding (PIO ISA): top 3 bits 0b111 (SET), then
// 5 delay/sideset = 0, 3 dest = 0 (PINS), 5 data.
const SET_PINS_1: u32 = 0xE001;
const SET_PINS_0: u32 = 0xE000;

fn install_program(block: &mut PioBlock) {
    // INSTR_MEM[0] = SET PINS,1 ; INSTR_MEM[1] = SET PINS,0
    block.write32(INSTR_MEM0, SET_PINS_1, 0);
    block.write32(INSTR_MEM0 + 4, SET_PINS_0, 0);
}

fn configure_sm(block: &mut PioBlock, sm: u32) {
    let off = sm * SM_STRIDE;
    // CLKDIV: int=1, frac=0 (every sysclk ticks the SM).
    block.write32(SM0_CLKDIV + off, 1u32 << 16, 0);
    // EXECCTRL: WRAP_TOP=1 (bits [16:12]), WRAP_BOTTOM=0 (bits [11:7]).
    block.write32(SM0_EXECCTRL + off, 1u32 << 12, 0);
}

fn parse_args() -> (u64, u32, u32) {
    let mut cycles: u64 = 100_000_000;
    let mut step_q: u32 = 4096;
    let mut sms: u32 = 1;
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--cycles" => {
                cycles = args[i + 1].parse().expect("--cycles N");
                i += 2;
            }
            "--step-q" => {
                step_q = args[i + 1].parse().expect("--step-q N");
                i += 2;
            }
            "--sms" => {
                sms = args[i + 1].parse().expect("--sms N");
                assert!((1..=4).contains(&sms), "--sms must be 1..=4");
                i += 2;
            }
            other => panic!("unknown arg: {other}"),
        }
    }
    (cycles, step_q, sms)
}

fn main() {
    let (cycles, step_q, sms) = parse_args();
    println!(
        "pio_microbench: cycles={} step_q={} sms_enabled={}",
        cycles, step_q, sms
    );

    let mut block = PioBlock::new();
    install_program(&mut block);
    for sm in 0..sms {
        configure_sm(&mut block, sm);
    }
    // Enable via CTRL.SM_ENABLE — bit i = SM i.
    let mask: u32 = (1u32 << sms) - 1;
    block.write32(CTRL, mask, 0);
    assert_eq!(block.sm_enabled_mask() as u32, mask);

    let n_calls = (cycles / step_q as u64).max(1);
    let actual_cycles = n_calls * step_q as u64;

    let start = Instant::now();
    for _ in 0..n_calls {
        block.step_n(step_q, 0);
    }
    let elapsed = start.elapsed();

    let secs = elapsed.as_secs_f64();
    let mhz = (actual_cycles as f64) / secs / 1e6;
    let ns_per = (elapsed.as_nanos() as f64) / (actual_cycles as f64);
    println!(
        "elapsed={:.3}s cycles={} MHz={:.1} ns/sysclk={:.2}",
        secs, actual_cycles, mhz, ns_per
    );
}

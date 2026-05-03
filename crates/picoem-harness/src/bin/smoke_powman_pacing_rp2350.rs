// smoke_powman_pacing_rp2350 — POWMAN pre-flight for RP2354 (V13 Stage 3).
//
// Purpose (HLD V11 §8 / V13 Stage 3): validate the XOSC/4 assumption baked
// into `PowmanRegs::advance` by measuring POWMAN's tick rate on Arthur's
// RP2354 silicon. V11/V12 wrote this as a run-mode smoke
// (`core.run()` + probe-rs `read_word_32` in a loop), which faulted on
// RP2354 with "An ARM specific error occurred" — see V12 journal §3.
// V13 reworks the smoke to stay halted throughout: POWMAN COUNT
// advances from XOSC/4, independent of core run state, so halted reads
// are sufficient to derive its frequency.
//
// Design differentiator: the emulator computes `sys_per_tick` from
// `ClockTree::sys_clk_hz / (XOSC_FREQ_HZ / 4) = 150e6 / 3e6 = 50`. If
// silicon reports a materially different POWMAN tick rate, the constant
// (and the ISR_SCENARIOS MATCH budget that inherits it) needs re-scoping
// before POWMAN ships. The smoke prints the derived rate in Hz — an
// operator compares to the expected ~3 MHz for XOSC/4 @ 12 MHz.
//
// Not for CI — requires a Pico debug probe attached to an RP2354 board.
// Precedent: follows the `--probe VID:PID:SERIAL` pattern of
// `probe_diff_rp2350.rs` and the halted-read pattern of the
// silicon_periph_diff / silicon_isr_diff oracles.

use probe_rs::probe::{DebugProbeSelector, list::Lister};
use probe_rs::{MemoryInterface, Permissions, Session, SessionConfig};
use std::time::{Duration, Instant};

// RESETS_RESET alias addresses. Base = 0x4002_0000; ALIAS_CLR = +0x3000.
// `RESET_POWMAN = 17` per pico-sdk `resets.h`.
const RESETS_RESET_CLR: u64 = 0x4002_3000;
const RESET_POWMAN_BIT: u32 = 1 << 17;

// POWMAN register map (pico-sdk `powman.h`, pinned commit
// a1438dff1d38bd9c65dbd693f0e5db4b9ae91779).
const POWMAN_BASE: u64 = 0x4010_0000;
const POWMAN_READ_TIME_LOWER: u64 = POWMAN_BASE + 0x74;
const POWMAN_TIMER: u64 = POWMAN_BASE + 0x88;
// POWMAN password-protected writes require upper 16 bits = 0x5AFE on
// every write; bare writes (no password) are silently dropped and
// latch BADPASSWD (V13 Stage 1 emulator semantics; silicon parity).
const POWMAN_PASSWD: u32 = 0x5AFE_0000;
const TIMER_RUN_BIT: u32 = 1 << 1;
// Per pico-sdk powman.h: TIMER.USE_LPOSC = bit 8 (0x0100). Selecting a
// clock source is mandatory for COUNT to advance — bare TIMER.RUN
// without USE_LPOSC / USE_XOSC leaves the timer with no input clock.
const TIMER_USE_LPOSC_BIT: u32 = 1 << 8;

// Sampling budget. Each iteration does one halted `read_word_32` (few
// hundred µs over SWD) plus a short host-side sleep; POWMAN ticks at
// ~3 MHz, so even a sparse sampling cadence observes dozens of
// transitions per second.
const SAMPLE_ITERATIONS: usize = 200;
const TARGET_TRANSITIONS: usize = 10;
const INTER_SAMPLE_SLEEP: Duration = Duration::from_micros(500);
const HALT_TIMEOUT: Duration = Duration::from_millis(500);

// Default probe selector (CLAUDE.md — "hard-wired probe serial → DUT
// mapping"): Arthur's RP2354 Pico 2 debug probe.
const DEFAULT_PROBE: &str = "2e8a:000c:E46410955F614129";

struct Args {
    probe: Option<DebugProbeSelector>,
}

use picoem_harness::cli::parse_probe_selector;

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    // Default to the hard-wired RP2354 serial; users override with
    // `--probe <VID:PID:SERIAL>` or `--probe auto` to fall back to
    // probe-rs `auto_attach`.
    let mut probe = Some(parse_probe_selector(DEFAULT_PROBE)?);
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--probe" => {
                i += 1;
                if i >= argv.len() {
                    return Err("--probe requires a VID:PID:SERIAL argument (or 'auto')".into());
                }
                probe = if argv[i] == "auto" {
                    None
                } else {
                    Some(parse_probe_selector(&argv[i])?)
                };
            }
            other => {
                return Err(format!(
                    "unknown argument '{other}'\n\
                     Usage:\n  \
                     smoke_powman_pacing_rp2350                         Use default RP2354 probe\n  \
                     smoke_powman_pacing_rp2350 --probe VID:PID:SERIAL  Select a specific probe\n  \
                     smoke_powman_pacing_rp2350 --probe auto            probe-rs auto_attach"
                ));
            }
        }
        i += 1;
    }
    Ok(Args { probe })
}

fn run() -> Result<i32, Box<dyn std::error::Error>> {
    let args = parse_args()?;

    println!("POWMAN pre-flight — tick rate measurement (halted-read mode)");
    match args.probe.as_ref() {
        None => println!("Probe: auto_attach"),
        Some(sel) => println!("Probe: {sel}"),
    }

    // Attach via explicit selector when provided; else auto_attach.
    let mut session = match args.probe.as_ref() {
        None => Session::auto_attach("rp2350", SessionConfig::default())?,
        Some(selector) => {
            let probe = Lister::new().open(selector.clone())?;
            probe.attach("rp2350", Permissions::default())?
        }
    };

    let mut core = session.core(0)?;
    core.reset_and_halt(HALT_TIMEOUT)?;

    // Release POWMAN from reset (RESETS_RESET_CLR = RESET_POWMAN bit).
    // RESETS is not password-gated; plain write is fine.
    core.write_word_32(RESETS_RESET_CLR, RESET_POWMAN_BIT)?;
    println!("Released POWMAN from reset; starting timer.");

    // Start POWMAN timer: USE_LPOSC = 1 (bit 8) selects the LPOSC
    // clock source, RUN = 1 (bit 1) starts counting. Password
    // required in bits [31:16]. ALARM_ENAB left clear so COUNT
    // free-runs. Writing with the core halted is safe in probe-rs.
    core.write_word_32(
        POWMAN_TIMER,
        POWMAN_PASSWD | TIMER_USE_LPOSC_BIT | TIMER_RUN_BIT,
    )?;

    // -- Sampling --------------------------------------------------
    //
    // POWMAN's COUNT advances on XOSC/4 regardless of core run state,
    // so halted-reads suffice. Each iteration reads COUNT + captures a
    // host `Instant`; we only record a pair when COUNT has changed so
    // the reported span is tick-bounded. This avoids the V11/V12
    // fault: run-mode probe reads on RP2354 throw "An ARM specific
    // error occurred" on the first `read_word_32` after `core.run()`.
    // Staying halted sidesteps the issue entirely.
    println!("Sample pairs (µs since first sample, COUNT):");
    let mut pairs: Vec<(Instant, u32)> = Vec::new();
    let mut last_count: Option<u32> = None;
    for _ in 0..SAMPLE_ITERATIONS {
        let count = core.read_word_32(POWMAN_READ_TIME_LOWER)?;
        let now = Instant::now();
        if Some(count) != last_count {
            pairs.push((now, count));
            last_count = Some(count);
            if pairs.len() > TARGET_TRANSITIONS {
                break;
            }
        }
        std::thread::sleep(INTER_SAMPLE_SLEEP);
    }

    if pairs.len() < 2 {
        println!(
            "  (only {} unique COUNT value(s) observed in {} samples; \
             probe read cadence is slower than the POWMAN tick — \
             unexpected at default clocks, investigate reset release)",
            pairs.len(),
            SAMPLE_ITERATIONS
        );
    } else {
        let t0 = pairs[0].0;
        for (i, (t, count)) in pairs.iter().enumerate() {
            let dt_us = t.duration_since(t0).as_micros();
            println!("  {i:3}: t=+{dt_us:7} µs  count={count}");
        }

        // Derive tick rate from first-to-last span. Single intervals
        // are vulnerable to probe read-back noise; the wide span
        // integrates over many ticks.
        let (t_first, c_first) = pairs[0];
        let (t_last, c_last) = *pairs.last().unwrap();
        let dt_s = t_last.duration_since(t_first).as_secs_f64();
        let dc = c_last.wrapping_sub(c_first) as f64;
        if dt_s <= 0.0 || dc == 0.0 {
            println!("Derived: insufficient span to compute rate.");
        } else {
            let hz = dc / dt_s;
            let ratio_vs_3mhz = hz / 3_000_000.0;
            println!(
                "Derived: POWMAN tick rate ≈ {hz:.0} Hz — \
                 {ratio_vs_3mhz:.3}× the XOSC/4 @ 12 MHz expected \
                 value (3 MHz). Note: sampling via SWD caps the \
                 observable rate; an under-read here does not imply \
                 POWMAN is actually slow, only that the probe \
                 round-trip integrates multiple ticks."
            );
        }
    }

    // Dump POWMAN_BASE + 0x00..0x24 for cross-verification with the
    // pico-sdk / datasheet.
    println!("Register dump POWMAN_BASE + 0x00..0x24:");
    for off in (0x00u64..0x24).step_by(4) {
        let v = core.read_word_32(POWMAN_BASE + off)?;
        println!("  0x{off:02X}: 0x{v:08X}");
    }

    Ok(0)
}

fn main() {
    picoem_harness::harness_tracing_init();
    match run() {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("fatal: {e}");
            std::process::exit(2);
        }
    }
}

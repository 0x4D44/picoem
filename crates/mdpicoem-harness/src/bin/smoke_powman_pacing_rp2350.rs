// smoke_powman_pacing_rp2350 — Stage 5 pre-flight for POWMAN on RP2354.
//
// Purpose (HLD V11 §8): validate the XOSC/4 assumption baked into
// `PowmanRegs::advance` by measuring the real sys_clks-per-POWMAN-tick
// ratio on Arthur's RP2354 silicon. Emits raw (CYCCNT, COUNT) sample
// pairs across ~5–10 COUNT transitions, prints the derived ratio, and
// dumps `POWMAN_BASE + 0x00..0x24` for cross-verification with the
// pico-sdk / datasheet.
//
// Differentiator design: the emulator computes `sys_per_tick` from
// `ClockTree::sys_clk_hz / (XOSC_FREQ_HZ / 4) = 150e6 / 3e6 = 50`. If
// silicon reports a materially different ratio, the constant (and the
// ISR_SCENARIOS MATCH budget that inherits it) needs re-scoping before
// POWMAN ships. A compile-only "it built" result is not closure.
//
// Stage 5 scaffolding — re-run if the clock tree changes; otherwise
// leave in `src/bin/` as precedent (mirrors `test_rp2350_smoke_per_core_cyccnt`).
//
// Precedent: follows the probe-attach / core-halt / memory-read/write
// pattern of `test_rp2350_smoke_per_core_cyccnt.rs` and the `--probe
// VID:PID:SERIAL` option of `test_rp2350_probe_diff.rs` (disambiguates
// when both the RP2354 and RP2040 probes are attached to one host).

use probe_rs::probe::{list::Lister, DebugProbeSelector};
use probe_rs::{MemoryInterface, Permissions, Session, SessionConfig};
use std::time::Duration;

// DWT cycle counter — sys_clk reference clock (1 sys_clk per CYCCNT tick
// while the core is running at sys_clk).
const DEMCR: u64 = 0xE000_EDFC;
const DWT_CTRL: u64 = 0xE000_1000;
const DWT_CYCCNT: u64 = 0xE000_1004;
const TRCENA: u32 = 1 << 24;
const CYCCNTENA: u32 = 1 << 0;

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
// latch BADPASSWD.
const POWMAN_PASSWD: u32 = 0x5AFE_0000;
const TIMER_RUN_BIT: u32 = 1 << 1;
// Per pico-sdk powman.h: TIMER.USE_LPOSC = bit 8 (0x0100). Selecting a
// clock source is mandatory for COUNT to advance — bare TIMER.RUN
// without USE_LPOSC / USE_XOSC leaves the timer with no input clock
// and the bus access to READ_TIME_* faults (V11 Stage 6 smoke
// reproduced this with `An ARM specific error occurred`).
const TIMER_USE_LPOSC_BIT: u32 = 1 << 8;

// Pacing: sample loop budget. `READ_TIME_LOWER` ticks at ~3 MHz
// (XOSC/4 = 12 MHz / 4). Each loop iteration costs one probe read
// round-trip (~ms scale over SWD @ default clock), so a modest
// iteration cap easily observes 5–10 COUNT transitions.
const MAX_SAMPLES: usize = 2000;
const TARGET_TRANSITIONS: usize = 10;
const HALT_TIMEOUT: Duration = Duration::from_millis(500);

// Default probe selector (CLAUDE.md — "hard-wired probe serial → DUT
// mapping"): Arthur's RP2354 Pico 2 debug probe.
const DEFAULT_PROBE: &str = "2e8a:000c:E46410955F614129";

struct Args {
    probe: Option<DebugProbeSelector>,
}

fn parse_probe_selector(s: &str) -> Result<DebugProbeSelector, String> {
    DebugProbeSelector::try_from(s).map_err(|e| format!("invalid probe selector '{s}': {e}"))
}

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

    println!("POWMAN pre-flight — sys_clks/tick ratio measurement");
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

    // Enable DWT CYCCNT — our sys_clk reference. 1 CYCCNT tick = 1
    // sys_clk while the core runs.
    let demcr = core.read_word_32(DEMCR)?;
    core.write_word_32(DEMCR, demcr | TRCENA)?;
    let ctrl = core.read_word_32(DWT_CTRL)?;
    core.write_word_32(DWT_CTRL, ctrl | CYCCNTENA)?;
    core.write_word_32(DWT_CYCCNT, 0)?;

    // Release POWMAN from reset (RESETS_RESET_CLR = RESET_POWMAN bit).
    // No password required on RESETS — this is the RESETS peripheral,
    // not POWMAN itself.
    core.write_word_32(RESETS_RESET_CLR, RESET_POWMAN_BIT)?;
    println!("Released POWMAN from reset; starting timer.");

    // Start POWMAN timer: USE_LPOSC = 1 (bit 8) selects the LPOSC
    // clock source — without it COUNT has no input clock and READ_TIME
    // bus accesses fault. RUN = 1 (bit 1) starts counting. ALARM_ENAB
    // left clear so we just free-run COUNT. Password required.
    core.write_word_32(
        POWMAN_TIMER,
        POWMAN_PASSWD | TIMER_USE_LPOSC_BIT | TIMER_RUN_BIT,
    )?;

    // Release the core so CYCCNT ticks; we'll read both CYCCNT and
    // POWMAN COUNT via SWD while the core is running. probe-rs handles
    // the halt/resume under the hood on each memory access.
    core.run()?;

    // Reference sys_clk from DWT: measure by spinning a fixed
    // wall-clock interval and seeing how much CYCCNT advanced. Uses
    // the probe-read of CYCCNT as the reference since we don't have
    // another timebase here; the ratio printed below is sys_clks per
    // POWMAN tick regardless of absolute sys_clk_hz.
    //
    // Sample: poll (CYCCNT, COUNT) until we've seen TARGET_TRANSITIONS
    // or hit MAX_SAMPLES. Print only the first pair per unique COUNT
    // value — "first CYCCNT at which COUNT == n" — which is what the
    // sys_per_tick derivation needs.
    println!("Sample pairs (CYCCNT, COUNT):");

    let mut pairs: Vec<(u32, u32)> = Vec::new();
    let mut last_count: Option<u32> = None;
    for _ in 0..MAX_SAMPLES {
        let cyccnt = core.read_word_32(DWT_CYCCNT)?;
        let count = core.read_word_32(POWMAN_READ_TIME_LOWER)?;
        if Some(count) != last_count {
            // First CYCCNT at which we saw this COUNT value.
            pairs.push((cyccnt, count));
            last_count = Some(count);
            if pairs.len() > TARGET_TRANSITIONS {
                break;
            }
        }
    }

    if pairs.len() < 2 {
        println!(
            "  (only {} unique COUNT value(s) observed in {} samples; \
             probe round-trip may be slower than POWMAN tick — increase \
             MAX_SAMPLES or slow sys_clk to recover)",
            pairs.len(),
            MAX_SAMPLES
        );
    } else {
        let mut last_c: Option<u32> = None;
        for (i, (cyccnt, count)) in pairs.iter().enumerate() {
            match last_c {
                None => println!("  {i}: cyccnt={cyccnt} count={count}"),
                Some(prev) => {
                    let d = cyccnt.wrapping_sub(prev);
                    println!("  {i}: cyccnt={cyccnt} count={count}  →  ΔCYCCNT = {d}");
                }
            }
            last_c = Some(*cyccnt);
        }

        // Derive sys_clks/tick from the first-to-last span rather than
        // a single interval — single intervals are vulnerable to probe
        // read-back noise. total_cyccnt_delta / total_tick_delta.
        let (first_cyccnt, first_count) = pairs[0];
        let (last_cyccnt, last_count) = *pairs.last().unwrap();
        let d_cyc = last_cyccnt.wrapping_sub(first_cyccnt) as u64;
        let d_count = last_count.wrapping_sub(first_count) as u64;
        if d_count == 0 {
            println!("Derived: insufficient tick spread to compute ratio.");
        } else {
            let ratio = d_cyc / d_count;
            println!(
                "Derived: sys_clks per POWMAN tick ≈ {ratio} \
                 (expected 50 if XOSC/4@3MHz & sys@150MHz)"
            );
        }
    }

    // Dump POWMAN_BASE + 0x00..0x24 for cross-verification. Writing
    // through the POWMAN password filter is not needed for reads.
    println!("Register dump POWMAN_BASE + 0x00..0x24:");
    for off in (0x00u64..0x24).step_by(4) {
        let v = core.read_word_32(POWMAN_BASE + off)?;
        println!("  0x{off:02X}: 0x{v:08X}");
    }

    Ok(0)
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

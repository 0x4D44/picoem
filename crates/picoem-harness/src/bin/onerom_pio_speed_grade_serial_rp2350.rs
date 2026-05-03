//! OneROM PIO Serial Speed-Grade Oracle.
//!
//! Per-rung verdict over a **nanosecond** ladder — EPROM speed-grade
//! convention — measured by reusing [`ServingOracle::run_case`] across
//! the full 2048-case A11=A12=1 sweep on the 1541 $E000 kernal (ROM set
//! 0) fixture in PIO-serve mode.
//!
//! HLD / journal: `wrk_journals/2026.04.24 - JRN - OneROM PIO Serial
//! Speed-Grade Oracle.md`. Companion to the CPU-mode ladder
//! (`onerom_cpu_speed_grade_serial_rp2350`), but:
//!
//! - **ns rungs** instead of cycles (EPROM speed-grade classification).
//! - PIO serve path, i.e. `ServingOracle` + `GlueDma` instead of
//!   `CpuServingOracle`.
//! - `is_synced` sync criterion (PIO1+PIO2 SM-enable) instead of the
//!   CPU serve-loop PC check.
//! - No `force_rom_set_index_via_sel_pins` — the PIO firmware writes
//!   `rom_set_index` to SRAM by sync time, so the oracle lifts the
//!   shadow for the correct set automatically.
//!
//! This binary is a **self-characterisation** of the emulator's PIO
//! mode — it does not validate against real-silicon OneROM timing. See
//! `onerom_serving_oracle.rs:58-78` for the emulator-bounded envelope.
//!
//! CLI:
//!   --ladder <csv>     ns thresholds, strictly decreasing
//!                      (default 300,250,200,150,120,100,90)
//!   --all-rungs        Report every rung (default: stop at first FAIL)
//!   --help             Print this message and exit.
//!
//! Usage:
//!   cargo run -p picoem-harness --release \
//!     --bin onerom_pio_speed_grade_serial_rp2350

use std::collections::BTreeMap;
use std::process::ExitCode;
use std::time::Instant;

use picoem_harness::{onerom_glue_dma, onerom_serving_oracle, onerom_stress, onerom_sync};
use rp2350_emu::{Config, Emulator, EmulatorBuilder};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str = "crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-1541.bin";

/// ROM set index parsed from the fixture. `0` = 1541 $E000 kernal
/// (901229-06AA). The PIO firmware writes this into SRAM by sync time;
/// this constant is only used for the report header.
const ROM_SET_INDEX: u8 = 0;

/// Boot cycle cap — sync normally arrives around cycle 7k; 10M is
/// generous. Matches `onerom_stress_pio_rp2350`.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// Default ns-ladder. Rationale (see journal 2026-04-24, stage B):
/// stress_pio reports min=126 / p50=233 / mean=226 / p95=233 / max=233
/// ns on the emulator. The ladder spans that observed range from a
/// slack top (300 ns) through the p95 (250 ns) down to known-impossible
/// emulator rungs (90 ns) so the cliff is visible in the per-rung
/// output. Strictly decreasing — harder targets lie later.
///
/// The 90 ns floor matches the silicon CPU claim (13 cy @ 150 MHz) and
/// acts as the "PIO mode is not silicon-fast on this emulator" canary.
const DEFAULT_NS_LADDER: &[u32] = &[300, 250, 200, 150, 120, 100, 90];

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Cli {
    ladder_ns: Vec<u32>,
    all_rungs: bool,
}

fn print_help() {
    println!(
        "onerom_pio_speed_grade_serial_rp2350 — serial-mode ns speed-grade oracle (PIO)\n\
         \n\
         Measures stim-to-byte serving latency over the full 2048-case\n\
         A11=A12=1 sweep on the 1541 ROM set 0 fixture, through the PIO\n\
         serve path. Reports per-rung PASS/FAIL over a nanosecond ladder\n\
         (EPROM speed-grade convention) plus an emulator speed-grade\n\
         classification.\n\
         \n\
         This is a self-characterisation of the emulator's PIO model; it\n\
         does NOT validate against real-silicon OneROM timing.\n\
         \n\
         FLAGS\n\
         \u{20}\u{20}--ladder <csv>   ns thresholds, strictly decreasing (default 300,250,200,150,120,100,90).\n\
         \u{20}\u{20}--all-rungs      Continue past first failing rung.\n\
         \u{20}\u{20}--help           Print this message."
    );
}

fn parse_cli() -> Result<Cli, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut ladder_csv: Option<String> = None;
    let mut all_rungs = false;

    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--help" | "-h" => {
                print_help();
                std::process::exit(0);
            }
            "--ladder" => {
                i += 1;
                let v = args.get(i).ok_or("--ladder requires a CSV value")?;
                ladder_csv = Some(v.clone());
            }
            "--all-rungs" => {
                all_rungs = true;
            }
            other => return Err(format!("unknown argument: {}", other)),
        }
        i += 1;
    }

    let ladder_ns: Vec<u32> = match ladder_csv {
        None => DEFAULT_NS_LADDER.to_vec(),
        Some(csv) => csv
            .split(',')
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .map(|s| {
                s.parse::<u32>()
                    .map_err(|e| format!("--ladder entry '{}': {}", s, e))
            })
            .collect::<Result<Vec<_>, _>>()?,
    };
    if ladder_ns.is_empty() {
        return Err("--ladder cannot be empty".to_string());
    }
    // Strictly decreasing — harder (lower-ns) targets lie later.
    for win in ladder_ns.windows(2) {
        if win[0] <= win[1] {
            return Err(format!(
                "--ladder must be strictly decreasing: {} !> {}",
                win[0], win[1]
            ));
        }
    }

    Ok(Cli {
        ladder_ns,
        all_rungs,
    })
}

// ---------------------------------------------------------------------------
// Boot-sync (verbatim pattern from `onerom_stress_pio_rp2350`, inlined
// per HLD scope discipline — no shared helper module).
// ---------------------------------------------------------------------------

/// Load bootrom / flash, reset, halt core 1, step serially until the
/// `is_synced` predicate (both PIO1 and PIO2 SMs enabled) fires. Then
/// prime the glue DMA and populate SRAM from the lifted shadow. Returns
/// the synced emulator, the primed glue, and the oracle.
fn boot_sync(
    bootrom: &[u8],
    flash: &[u8],
) -> Result<
    (
        Emulator,
        onerom_glue_dma::GlueDma,
        onerom_serving_oracle::ServingOracle,
    ),
    String,
> {
    // Up-front shadow-lift sanity check: confirm the hardcoded ROM set
    // parses out of the fixture. None here is a loud early signal that
    // the fixture / ROM_SET_INDEX pair is wrong.
    if onerom_serving_oracle::lift_shadow_from_flash_pub(flash, ROM_SET_INDEX).is_none() {
        return Err(format!(
            "failed to lift ROM set {} from fixture — wrong index or malformed flash",
            ROM_SET_INDEX
        ));
    }

    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();
    emu.load_bootrom(bootrom);
    emu.load_flash(flash);
    emu.reset();

    // Bootrom bypass — OneROM flash is not an IMAGE_DEF block.
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);

    // OneROM's serving loop is single-core.
    emu.core_mut(1).halt();

    // Step to PIO sync (both PIO blocks' SMs enabled).
    let mut glue = onerom_glue_dma::GlueDma::new();
    let mut sync_cycle: Option<u64> = None;
    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            return Err(format!("cycle counter stalled at {}", before));
        }
        if onerom_sync::is_synced(&mut emu.bus) {
            sync_cycle = Some(after);
            break;
        }
    }
    if sync_cycle.is_none() {
        return Err(format!(
            "boot did not reach PIO1+PIO2 SM-enable sync within {} cycles",
            BOOT_CYCLE_CAP
        ));
    }

    // Prime DMA + oracle after sync.
    glue.prime_after_sync(&mut emu.bus);
    let oracle = onerom_serving_oracle::ServingOracle::new_at_sync(&mut emu.bus, flash);
    oracle.populate_sram_from_shadow(&mut emu.bus);

    Ok((emu, glue, oracle))
}

// ---------------------------------------------------------------------------
// Stats (computed directly from the collected latency Vec in cycles;
// ns is reported alongside by plain integer conversion).
// ---------------------------------------------------------------------------

struct CycleStats {
    min: u32,
    p50: u32,
    mean: u32,
    p95: u32,
    max: u32,
}

fn compute_stats(latencies: &[u32]) -> Option<CycleStats> {
    if latencies.is_empty() {
        return None;
    }
    let mut sorted = latencies.to_vec();
    sorted.sort_unstable();
    let n = sorted.len();
    let min = sorted[0];
    let max = sorted[n - 1];
    // Nearest-rank percentile (ceil): index = ceil(p * n) - 1, clamped.
    let p50_idx = (50 * n).div_ceil(100).saturating_sub(1).min(n - 1);
    let p95_idx = (95 * n).div_ceil(100).saturating_sub(1).min(n - 1);
    let p50 = sorted[p50_idx];
    let p95 = sorted[p95_idx];
    // Mean as plain integer (truncating).
    let sum: u64 = sorted.iter().map(|&c| c as u64).sum();
    let mean = (sum / n as u64) as u32;
    Some(CycleStats {
        min,
        p50,
        mean,
        p95,
        max,
    })
}

/// Truncating cy→ns at `sys_clk_hz`.
#[inline]
fn cycles_to_ns(cycles: u32, sys_clk_hz: u32) -> u64 {
    (cycles as u64) * 1_000_000_000 / (sys_clk_hz as u64)
}

/// Truncating ns→cy threshold at `sys_clk_hz`. A case whose
/// `latency_cycles <= ns_to_cy_threshold(rung_ns)` "meets the rung".
#[inline]
fn ns_to_cy_threshold(ns: u32, sys_clk_hz: u32) -> u32 {
    ((ns as u64) * (sys_clk_hz as u64) / 1_000_000_000) as u32
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> ExitCode {
    picoem_harness::harness_tracing_init();

    let cli = match parse_cli() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: {}", e);
            eprintln!("hint: pass --help for usage.");
            return ExitCode::from(2);
        }
    };

    let t_start = Instant::now();

    let bootrom = match std::fs::read(BOOTROM_PATH) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom at {}: {}", BOOTROM_PATH, e);
            return ExitCode::from(2);
        }
    };
    let flash = match std::fs::read(FLASH_PATH) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read flash image at {}: {}", FLASH_PATH, e);
            return ExitCode::from(2);
        }
    };

    let (mut emu, mut glue, mut oracle) = match boot_sync(&bootrom, &flash) {
        Ok(trio) => trio,
        Err(e) => {
            eprintln!("boot-sync failed: {}", e);
            return ExitCode::FAILURE;
        }
    };

    let sys_clk_hz = emu.bus.sys_clk_hz();
    if sys_clk_hz == 0 {
        eprintln!("sys_clk_hz is 0 at sync — PLL not settled; cannot convert ns rungs");
        return ExitCode::FAILURE;
    }

    let cases = onerom_stress::generate_sweep_cases();
    let mut latencies: Vec<u32> = Vec::with_capacity(cases.len());
    let mut wrong_byte: u32 = 0;
    let mut no_resolve: u32 = 0;
    let mut no_stable: u32 = 0;
    let mut out_of_range: u32 = 0;
    let mut out_of_envelope: u32 = 0;

    for case in &cases {
        // `run_case` returns a borrow into the oracle's internal results
        // vec; copy the fields out before the next call invalidates it.
        // Only `Verdict::Pass` counts as "served" for the ladder — the
        // other verdicts (WrongByte carries a latency, LatencyOutOfEnvelope
        // carries one too) must be treated as errors at every rung.
        let (verdict, latency_cycles) = {
            let result = oracle.run_case(&mut emu, &mut glue, *case);
            (result.verdict, result.latency_cycles)
        };
        match verdict {
            onerom_serving_oracle::Verdict::Pass => match latency_cycles {
                Some(cy) => latencies.push(cy),
                // Pass with no latency would be a library invariant
                // violation; bucket as out_of_envelope rather than silently
                // dropping. (Keeps the ladder error count honest.)
                None => out_of_envelope += 1,
            },
            onerom_serving_oracle::Verdict::WrongByte { .. } => wrong_byte += 1,
            onerom_serving_oracle::Verdict::NoResolve => no_resolve += 1,
            onerom_serving_oracle::Verdict::NoStableByte => no_stable += 1,
            onerom_serving_oracle::Verdict::ResolvedAddrOutOfRange { .. } => out_of_range += 1,
            onerom_serving_oracle::Verdict::LatencyOutOfEnvelope { .. } => out_of_envelope += 1,
        }
    }

    let total_cases = cases.len();
    let elapsed = t_start.elapsed();

    // -------------------- Report ------------------------------------------
    println!(
        "OneROM PIO Serial Speed-Grade Oracle — 1541 $E000 kernal (ROM set {})",
        ROM_SET_INDEX
    );
    println!("fixture:  {}", FLASH_PATH);
    println!("cases:    {} (full address sweep, A11=A12=1)", total_cases);
    {
        let mhz = sys_clk_hz as f64 / 1_000_000.0;
        let ns_per_cy = 1_000_000_000.0 / sys_clk_hz as f64;
        println!("sysclk:   {:.0} MHz ({:.2} ns/cy)", mhz, ns_per_cy);
    }
    println!();
    println!("NOTE: This characterises the emulator's PIO model under steady-state");
    println!("stimulus; it does not validate against real-silicon OneROM timing. See");
    println!("ENVELOPE_CYCLES derivation in onerom_serving_oracle.rs:58-78.");
    println!();
    println!("   threshold     errors    verdict");
    println!("        (ns)");
    println!();

    let mut top_rung_errors: Option<u32> = None;
    // Track the tightest rung at which zero errors were recorded so the
    // speed-grade line reflects the whole ladder, not just pre-break
    // state when --all-rungs is off. Stored in ns.
    let mut tightest_pass_ns: Option<u32> = None;
    for (i, &rung_ns) in cli.ladder_ns.iter().enumerate() {
        let rung_cy = ns_to_cy_threshold(rung_ns, sys_clk_hz);
        let served_in_rung = latencies.iter().filter(|&&cy| cy <= rung_cy).count();
        // Only Pass entries populate `latencies`, so all other error
        // buckets inherently count as errors at every rung
        // (latencies.len() <= total_cases).
        let errors = (total_cases - served_in_rung) as u32;
        if i == 0 {
            top_rung_errors = Some(errors);
        }
        if errors == 0 {
            tightest_pass_ns = Some(rung_ns);
        }
        let verdict = if errors == 0 { "PASS" } else { "FAIL" };
        println!("   {:>9}   {:>8}     {}", rung_ns, errors, verdict);
        if !cli.all_rungs && errors != 0 {
            break;
        }
    }

    println!();
    match compute_stats(&latencies) {
        Some(s) => println!(
            "cycles across all cases: min={} p50={} mean={} p95={} max={}  ({} / {} / {} / {} / {} ns)",
            s.min,
            s.p50,
            s.mean,
            s.p95,
            s.max,
            cycles_to_ns(s.min, sys_clk_hz),
            cycles_to_ns(s.p50, sys_clk_hz),
            cycles_to_ns(s.mean, sys_clk_hz),
            cycles_to_ns(s.p95, sys_clk_hz),
            cycles_to_ns(s.max, sys_clk_hz),
        ),
        None => println!("cycles across all cases: (no measurable cases)"),
    }
    if !latencies.is_empty() {
        let mut buckets: BTreeMap<u32, usize> = BTreeMap::new();
        for &cy in &latencies {
            *buckets.entry(cy).or_insert(0) += 1;
        }
        let rendered: Vec<String> = buckets
            .iter()
            .map(|(cy, n)| format!("{}: {} cases", cy, n))
            .collect();
        println!("unique cycle buckets:     {}", rendered.join(", "));
    }
    println!("wrong-byte:               {}", wrong_byte);
    println!("no-resolve:               {}", no_resolve);
    println!("no-stable-byte:           {}", no_stable);
    println!("addr-out-of-range:        {}", out_of_range);
    println!("out-of-envelope:          {}", out_of_envelope);
    println!();
    match tightest_pass_ns {
        Some(ns) => println!(
            "Emulator speed-grade for PIO mode on 1541 $E000 fixture: -{} ns (tightest rung all cases served)",
            ns
        ),
        None => println!(
            "Emulator speed-grade for PIO mode on 1541 $E000 fixture: UNGRADED (fails at all tested rungs)"
        ),
    }
    println!();
    println!("elapsed: {:.1} s wall-clock", elapsed.as_secs_f64());

    match top_rung_errors {
        Some(0) => ExitCode::SUCCESS,
        _ => ExitCode::FAILURE,
    }
}

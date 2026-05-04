//! OneROM CPU Serial Speed-Grade Oracle.
//!
//! Per-rung verdict over a cycle-count ladder, measured in **simulated
//! cycles** by reusing [`CpuServingOracle::run_case`] across the full
//! 2048-case A11=A12=1 sweep on the 1541 $E000 kernal (ROM set 0)
//! fixture.
//!
//! HLD: `wrk_docs/2026.04.23 - HLD - OneROM CPU Serial Speed-Grade
//! Oracle V2.md`. Companion to the threaded wall-clock oracle
//! (`onerom_cpu_speed_grade_rp2350`) — same fixture, same walk, but
//! decoupled from host-threading overhead. This is the honest answer
//! to "does our emulator serve in cycle-count parity with silicon?".
//!
//! CLI:
//!   --ladder <csv>     Cycle thresholds, strictly decreasing
//!                      (default 50,30,20,13,10)
//!   --all-rungs        Report every rung (default: stop at first FAIL)
//!   --help             Print this message and exit.
//!
//! Usage:
//!   cargo run -p picoem-harness --release \
//!     --bin onerom_cpu_speed_grade_serial_rp2350

use std::process::ExitCode;
use std::time::Instant;

use picoem_harness::onerom_fixture::{FixtureSpec, lift_shadow_from_flash};
use picoem_harness::{
    onerom_serving_oracle_cpu::{self, CpuServingOracle, CpuVerdict},
    onerom_stress,
};
use rp2350_emu::{Config, Emulator, EmulatorBuilder};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str = "crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

/// ROM set index parsed from the fixture. `0` = 1541 $E000 kernal
/// (901229-06AA, 2364 bake) — matches the library's hardcoded pin
/// constants. Forced at boot via the image_sel helper.
const ROM_SET_INDEX: u32 = 0;

/// Boot cycle cap — mirrors the stress and threaded speed-grade
/// binaries. Enough for any plausible CPU-mode sync; if we don't sync
/// in 10M cycles the fixture is broken.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// CPU serve-loop PC range for the 1541 fixture's ROM set 0 (2364
/// bake). Same offset as `test-sdrr-0-cpu` and the threaded speed-grade
/// binary; kept as a local constant so a future bake shift doesn't
/// ripple through the shared library.
const SERVE_LOOP_PC_LO: u32 = 0x1000_0926;
const SERVE_LOOP_PC_HI: u32 = 0x1000_0930;

/// Default cycle-count ladder. Band chosen to cover the observed
/// envelope `CPU_ENVELOPE_CYCLES = 7..=60`:
///   50 — top of envelope minus slack; PASS on golden fixture.
///   30 — above p95 (~40 cycles); tail misses.
///   20 — around p90.
///   13 — silicon-claim floor (90 ns @ 150 MHz = 13 cycles).
///   10 — below silicon; tests distribution below best-case.
const DEFAULT_CYCLE_LADDER: &[u32] = &[50, 30, 20, 13, 10];

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Cli {
    ladder: Vec<u32>,
    all_rungs: bool,
}

fn print_help() {
    println!(
        "onerom_cpu_speed_grade_serial_rp2350 — serial-mode cycle-count speed-grade oracle\n\
         \n\
         Measures stim-to-byte serving latency in simulated cycles over the\n\
         full 2048-case A11=A12=1 sweep on the 1541 ROM set 0 fixture.\n\
         Reports per-rung PASS/FAIL over a cycle-count ladder.\n\
         \n\
         FLAGS\n\
         \u{20}\u{20}--ladder <csv>   Cycle thresholds, strictly decreasing (default 50,30,20,13,10).\n\
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

    let ladder: Vec<u32> = match ladder_csv {
        None => DEFAULT_CYCLE_LADDER.to_vec(),
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
    if ladder.is_empty() {
        return Err("--ladder cannot be empty".to_string());
    }
    // Strictly decreasing — harder targets lie later in the list.
    for win in ladder.windows(2) {
        if win[0] <= win[1] {
            return Err(format!(
                "--ladder must be strictly decreasing: {} !> {}",
                win[0], win[1]
            ));
        }
    }

    Ok(Cli { ladder, all_rungs })
}

// ---------------------------------------------------------------------------
// Boot-sync (verbatim from `onerom_cpu_speed_grade_rp2350::boot_sync`,
// with `ROM_SET_INDEX` resolved to the local `u32` constant).
// ---------------------------------------------------------------------------

/// Load bootrom / flash, reset, halt core 1, force ROM set 0 via
/// `force_rom_set_index_via_sel_pins`, run the emulator serially until
/// core 0's PC enters the serve-loop range AND the shadow tripwire
/// fires. Returns the synced emulator.
fn boot_sync(bootrom: &[u8], flash: &[u8], spec: &FixtureSpec) -> Result<Emulator, String> {
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

    // CPU-serve is single-core.
    emu.core_mut(1).halt();

    // Force ROM set 0 via the image_sel helper so the firmware boots
    // the 2364 bake matching the library pin constants.
    onerom_serving_oracle_cpu::force_rom_set_index_via_sel_pins(&mut emu, flash, ROM_SET_INDEX)?;

    // Phase 1: step until PC enters the serve-loop range.
    let mut phase1_cycle: Option<u64> = None;
    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            return Err(format!("cycle counter stalled at {}", before));
        }
        if is_in_serve_loop(&emu) {
            phase1_cycle = Some(after);
            break;
        }
    }
    if phase1_cycle.is_none() {
        return Err(format!(
            "boot did not reach CPU serve-loop PC (0x{:08X}..=0x{:08X}) within {} cycles",
            SERVE_LOOP_PC_LO, SERVE_LOOP_PC_HI, BOOT_CYCLE_CAP
        ));
    }

    // Shadow sentinel (same convention as the CPU oracle).
    const RUNTIME_INFO_SRAM_OFF: u32 = 0x0008_0000;
    const ROM_SET_INDEX_OFFSET: u32 = 6;
    let rom_set_index_live = emu
        .bus
        .memory
        .sram_read8(RUNTIME_INFO_SRAM_OFF + ROM_SET_INDEX_OFFSET);
    let sentinel: Option<(u32, u8)> =
        match lift_shadow_from_flash(flash, rom_set_index_live, spec) {
            Some(shadow) => onerom_serving_oracle_cpu::find_shadow_sentinel(&shadow),
            None => None,
        };

    // Phase 2: PC + sentinel.
    let sentinel_ok = |emu: &Emulator| match sentinel {
        None => true,
        Some((offset, expected)) => emu.bus.memory.sram_read8(offset) == expected,
    };
    let mut synced = is_in_serve_loop(&emu) && sentinel_ok(&emu);
    while !synced && emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            return Err(format!("cycle counter stalled at {}", before));
        }
        synced = is_in_serve_loop(&emu) && sentinel_ok(&emu);
    }
    if !synced {
        return Err(format!(
            "boot did not reach CPU serve-loop sync (PC + sentinel) within {} cycles",
            BOOT_CYCLE_CAP
        ));
    }

    Ok(emu)
}

#[inline]
fn is_in_serve_loop(emu: &Emulator) -> bool {
    let pc = emu.core(0).regs.pc();
    (SERVE_LOOP_PC_LO..=SERVE_LOOP_PC_HI).contains(&pc)
}

// ---------------------------------------------------------------------------
// Stats (computed directly from the collected latency Vec in cycles —
// avoids the ns conversion that `onerom_stress::compute_histogram`
// performs, which would introduce rounding error on small cycle counts).
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
    // Mean as plain integer (truncating) — matches "mean=15" style.
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

    let spec = match FixtureSpec::from_flash(&flash) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("failed to parse fixture spec: {}", e);
            return ExitCode::FAILURE;
        }
    };

    let mut emu = match boot_sync(&bootrom, &flash, &spec) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("boot-sync failed: {}", e);
            return ExitCode::FAILURE;
        }
    };

    let mut oracle = CpuServingOracle::new_at_sync(&mut emu.bus, spec.clone(), &flash);

    let cases = onerom_stress::generate_sweep_cases(&spec);
    let mut latencies: Vec<u32> = Vec::with_capacity(cases.len());
    let mut wedged: u32 = 0;
    let mut wrong_byte: u32 = 0;
    let mut out_of_envelope: u32 = 0;
    for case in &cases {
        // `run_case` returns a borrow into the oracle's internal results
        // vec; copy the fields out before the next call invalidates it.
        // Only `CpuVerdict::Pass` counts as "served" for the ladder —
        // `WrongByte` and `LatencyOutOfEnvelope` also carry a
        // `latency_cycles: Some(_)` but must be treated as errors at
        // every rung (a fast-but-wrong byte is still a regression).
        let (verdict, latency_cycles) = {
            let result = oracle.run_case(&mut emu, *case);
            (result.verdict, result.latency_cycles)
        };
        match verdict {
            CpuVerdict::Pass => match latency_cycles {
                Some(cy) => latencies.push(cy),
                // Defensive: Pass with no latency should not occur, but
                // bucket as wedged rather than silently dropping.
                None => wedged += 1,
            },
            CpuVerdict::WrongByte { .. } => wrong_byte += 1,
            CpuVerdict::LatencyOutOfEnvelope { .. } => out_of_envelope += 1,
            CpuVerdict::DataPinsNotDriven | CpuVerdict::NoStableByte => wedged += 1,
        }
    }

    let total_cases = cases.len();
    let elapsed = t_start.elapsed();

    // -------------------- Report (HLD §4.6) -------------------------------
    println!(
        "OneROM CPU Serial Speed-Grade Oracle — 1541 $E000 kernal (ROM set {})",
        ROM_SET_INDEX
    );
    println!("fixture:  {}", FLASH_PATH);
    println!("cases:    {} (full address sweep, A11=A12=1)", total_cases);
    println!();
    println!("   threshold     errors    verdict");
    println!("      (cy)");
    println!();

    let mut top_rung_errors: Option<u32> = None;
    for (i, &rung) in cli.ladder.iter().enumerate() {
        let served_in_rung = latencies.iter().filter(|&&cy| cy <= rung).count();
        // Only `Pass` entries populate `latencies`, so wedged /
        // wrong-byte / out-of-envelope cases inherently count as errors
        // at every rung (latencies.len() <= total_cases).
        let errors = (total_cases - served_in_rung) as u32;
        // Capture the top-rung result BEFORE deciding whether to break,
        // so a future refactor can't accidentally exit early without
        // recording the exit-code-relevant verdict.
        if i == 0 {
            top_rung_errors = Some(errors);
        }
        let verdict = if errors == 0 { "PASS" } else { "FAIL" };
        println!("   {:>8}   {:>8}     {}", rung, errors, verdict);
        if !cli.all_rungs && errors != 0 {
            break;
        }
    }

    println!();
    match compute_stats(&latencies) {
        Some(s) => println!(
            "cycles across all cases: min={} p50={} mean={} p95={} max={}",
            s.min, s.p50, s.mean, s.p95, s.max
        ),
        None => println!("cycles across all cases: (no measurable cases)"),
    }
    println!(
        "wedged:                   {} (cases with no latency_cycles measurable)",
        wedged
    );
    println!(
        "wrong-byte:               {} (cases that produced an incorrect byte — counted as FAIL)",
        wrong_byte
    );
    println!(
        "out-of-envelope:          {} (cases served outside CPU_ENVELOPE_CYCLES — counted as FAIL)",
        out_of_envelope
    );
    println!();
    println!("elapsed: {:.1} s wall-clock", elapsed.as_secs_f64());

    match top_rung_errors {
        Some(0) => ExitCode::SUCCESS,
        _ => ExitCode::FAILURE,
    }
}

//! OneROM stress driver — PIO-mode sweep for the 1541 $E000 kernal fixture.
//!
//! Loads the 1541 OneROM firmware image, boots to PIO sync (same scaffolding
//! as [`onerom_serving_oracle_rp2350`]), then runs the full 2048-case
//! `addr_bits ∈ 0x1800..=0x1FFF` sweep through the serving oracle's
//! `run_case`. The sweep is silent — no per-case output — and ends with a
//! latency histogram + first-20-fails block rendered by
//! [`mdpicoem_harness::onerom_stress::format_report`].
//!
//! Hardcoded fixture path + `ROM_SET_INDEX` — change and recompile to target
//! a different fixture or ROM set.
//!
//! Design: `wrk_docs/2026.04.17 - HLD - OneROM Stress Harness.md`.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin onerom_stress_pio_rp2350 --release

use std::process::ExitCode;
use std::time::{Duration, Instant};

use mdpicoem_harness::{
    onerom_glue_dma, onerom_serving_oracle, onerom_stress, onerom_sync,
};
use mdrp2350::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-1541.bin";

/// ROM set index parsed from the fixture. `0` = 1541 $E000 kernal
/// (901229-06AA). Change + recompile to sweep a different set in the same
/// fixture; no env-var override by design.
const ROM_SET_INDEX: u8 = 0;

/// Human-readable label for the report header.
const LABEL: &str = "1541 $E000 kernal (901229-06AA), PIO mode";

/// Boot cycle cap — sync normally arrives around cycle 7k; 10M is generous.
/// Matches `onerom_serving_oracle_rp2350`.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// First N failures to inline in the report (HLD §Output format).
const FIRST_FAILS_CAP: usize = 20;

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

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

    // Up-front shadow-lift sanity check: confirm the hardcoded ROM set
    // parses out of the fixture. If this returns None the sweep cannot
    // meaningfully PASS — every case would see the fallback zero shadow.
    // We still boot below (the oracle lifts its own copy via
    // `new_at_sync` reading the SRAM rom_set_index), but a None here is
    // a loud early signal that the fixture / ROM_SET_INDEX pair is wrong.
    if onerom_serving_oracle::lift_shadow_from_flash_pub(&flash, ROM_SET_INDEX)
        .is_none()
    {
        eprintln!(
            "failed to lift ROM set {} from fixture — wrong index or malformed flash",
            ROM_SET_INDEX
        );
        return ExitCode::from(2);
    }

    // step_quantum=1 matches the serving-oracle binary — `run_case` needs
    // cycle-level granularity for its per-cycle observation loop.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build().unwrap();
    emu.load_bootrom(&bootrom);
    emu.load_flash(&flash);
    emu.reset();

    // Bootrom bypass — OneROM's `.bin` is raw flash, not an IMAGE_DEF block,
    // so jump straight to the reset vector.
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);

    // OneROM's serving loop is single-core.
    emu.core_mut(1).halt();

    // Step to PIO sync.
    let mut sync_cycle: Option<u64> = None;
    let mut glue = onerom_glue_dma::GlueDma::new();

    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            eprintln!("cycle counter stalled at {}", before);
            return ExitCode::FAILURE;
        }
        if onerom_sync::is_synced(&mut emu.bus) {
            sync_cycle = Some(after);
            break;
        }
    }

    if sync_cycle.is_none() {
        eprintln!(
            "FAILURE — boot did not reach PIO1+PIO2 SM-enable sync within {} cycles",
            BOOT_CYCLE_CAP
        );
        return ExitCode::FAILURE;
    }

    // Prime DMA + oracle. `new_at_sync` lifts its shadow by reading the
    // SRAM-encoded rom_set_index; the up-front `lift_shadow_from_flash_pub`
    // above confirms the hardcoded `ROM_SET_INDEX` matches a real set.
    glue.prime_after_sync(&mut emu.bus);
    let mut oracle =
        onerom_serving_oracle::ServingOracle::new_at_sync(&mut emu.bus, &flash);
    oracle.populate_sram_from_shadow(&mut emu.bus);

    // Silent sweep: 2048 cases, no per-case output.
    // `run_case` accumulates into `oracle.results()` — the per-call
    // return value is just a convenience reference we don't need here.
    // Wall-clock is measured per case via `Instant::now()` so the
    // report can show host elapsed time alongside the emulated-cycle
    // model latency. Both cases contribute to the throughput
    // denominator (failing cases still consume host time).
    let cases = onerom_stress::generate_sweep_cases();
    let mut wall_durations: Vec<Duration> = Vec::with_capacity(cases.len());
    for case in &cases {
        let t0 = Instant::now();
        let _ = oracle.run_case(&mut emu, &mut glue, *case);
        wall_durations.push(t0.elapsed());
    }

    // Histogram + first-N failures.
    let sys_clk_hz = emu.bus.sys_clk_hz();
    let results = oracle.results();
    let hist = onerom_stress::compute_histogram(results, sys_clk_hz);
    let wall = onerom_stress::compute_wall_clock_stats(&wall_durations);

    let fails: Vec<onerom_serving_oracle::CaseResult> = results
        .iter()
        .filter(|r| r.verdict != onerom_serving_oracle::Verdict::Pass)
        .take(FIRST_FAILS_CAP)
        .cloned()
        .collect();

    print!(
        "{}",
        onerom_stress::format_report(
            LABEL,
            FLASH_PATH,
            ROM_SET_INDEX,
            sys_clk_hz,
            &hist,
            &wall,
            &fails,
        )
    );

    if hist.fail == 0 {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

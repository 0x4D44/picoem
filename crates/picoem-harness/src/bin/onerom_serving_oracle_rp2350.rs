//! OneROM serving oracle — byte + timing end-to-end validator (Stage G).
//!
//! Boots the real OneROM firmware, waits for PIO sync, snapshots the
//! SRAM-base ROM table, then runs the Stage G case set through the
//! serving pipeline. For each case, drives the pin stimulus, watches the
//! glue DMA pump, and proves the byte observed on D0..D7 matches the
//! shadow byte at the resolved CH1.READ_ADDR.
//!
//! Stage G.3 — full 15-case walking-1s + pattern sweep through the
//! serving pipeline, followed by the `format_report` render. One line
//! per case still scrolls as the run proceeds so a stuck or crashing
//! case is obvious from the partial output; the full report (table,
//! summary, latency stats, ROM speed class, emulator-bounded caveat)
//! prints after the sweep completes. Envelope post-processing
//! (`apply_envelope`) is applied per-case inside
//! `ServingOracle::run_case`, so anything outside the
//! `ENVELOPE_CYCLES` window is reclassified to `LatencyOutOfEnvelope`
//! (see §5.3 in the HLD) before the report renders.
//!
//! Design: `wrk_docs/2026.04.15 - HLD - OneROM Serving Oracle (Stage G).md`.
//!
//! Usage:
//!   cargo run -p picoem-harness --bin onerom_serving_oracle_rp2350 --release

use std::collections::HashSet;
use std::process::ExitCode;
use std::time::Instant;

use picoem_harness::onerom_fixture::FixtureSpec;
use picoem_harness::{onerom_glue_dma, onerom_serving_oracle, onerom_sync};
use rp2350_emu::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str = "crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0.bin";

/// Cycle cap for boot. Same budget as Stage F's binary — sync normally
/// arrives around cycle 7k; 10M is generous.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

fn main() -> ExitCode {
    picoem_harness::harness_tracing_init();

    let bootrom_path = BOOTROM_PATH;
    let flash_path = FLASH_PATH;

    let bootrom = match std::fs::read(bootrom_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom at {}: {}", bootrom_path, e);
            return ExitCode::from(2);
        }
    };

    let flash = match std::fs::read(flash_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read flash image at {}: {}", flash_path, e);
            return ExitCode::from(2);
        }
    };

    println!(
        "loaded bootrom ({} bytes) and flash ({} bytes)",
        bootrom.len(),
        flash.len()
    );

    // step_quantum=1 so the per-cycle observation loop inside
    // `ServingOracle::run_case` sees a single CPU instruction per tick.
    // This matches Stage F's tracing cadence; the oracle's run_case
    // depends on cycle-level granularity for the push-anchored stability
    // check to work (HLD §4.4).
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();
    emu.load_bootrom(&bootrom);
    emu.load_flash(&flash);
    emu.reset();

    // Bootrom bypass — OneROM's `.bin` is raw flash, not an IMAGE_DEF
    // block, so we jump straight to the reset vector. Same as Stage F.
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32; // clear Thumb LSB
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);
    println!(
        "bypassing bootrom: SP=0x{:08X} PC=0x{:08X}",
        initial_sp, initial_pc
    );

    // OneROM's serving loop is single-core.
    emu.core_mut(1).halt();

    // Step to sync.
    let mut sync_cycle: Option<u64> = None;
    let mut sync_report: Option<onerom_sync::SyncReport> = None;
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
            sync_report = Some(onerom_sync::capture_snapshot(&mut emu.bus, after));
            break;
        }
    }

    let sync_at = match sync_cycle {
        Some(c) => c,
        None => {
            eprintln!(
                "FAILURE — boot did not reach PIO1+PIO2 SM-enable sync within {} cycles",
                BOOT_CYCLE_CAP
            );
            return ExitCode::FAILURE;
        }
    };

    println!("sync reached at cycle {}", sync_at);
    if let Some(r) = &sync_report {
        println!(
            "  PIO1.CTRL = 0x{:08X}, PIO2.CTRL = 0x{:08X}",
            r.pio1.ctrl, r.pio2.ctrl
        );
    }

    // Parse the FixtureSpec from the loaded flash before constructing
    // the oracle. The same flash bytes are reused for the shadow lift.
    let spec = match FixtureSpec::from_flash(&flash) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("failed to parse fixture spec: {}", e);
            return ExitCode::FAILURE;
        }
    };
    println!("fixture: {} ({}-pin)", spec.label, spec.chip_pins);

    // Prime the DMA and capture the SRAM shadow.
    glue.prime_after_sync(&mut emu.bus);

    let mut oracle =
        onerom_serving_oracle::ServingOracle::new_at_sync(&mut emu.bus, spec.clone(), &flash);

    // Mirror the flash-parsed shadow into SRAM — emulates what the
    // firmware's `preload_rom_image` DMA would have done. Without this
    // the glue DMA CH1 reads back 0x00 for every resolved address.
    oracle.populate_sram_from_shadow(&mut emu.bus);

    // Shadow-integrity tripwire (devil's-advocate Attack 2, Stage G.2 +
    // Stage G shadow-source investigation, 2026-04-15):
    // `ServingOracle::new_at_sync` now lifts the shadow from the flash
    // bytes via SDRR struct parsing (the SRAM-at-sync assumption from
    // HLD §3.1 was false — preload DMA hasn't landed by our sync
    // criterion). If the resulting shadow is uniform, either the parse
    // failed (fell back to zeros) or the selected ROM set is a genuine
    // null fixture like `zero8192.rom`. Either way, no address-decode
    // bug can be caught — every resolved_addr would look up the same
    // byte. Surface the unique count so the operator sees the tripwire
    // before misreading a false PASS.
    let unique: HashSet<u8> = oracle.shadow().iter().copied().collect();
    println!(
        "shadow @ 0x{:08X}..+0x{:04X}: {} unique bytes",
        onerom_serving_oracle::SHADOW_BASE,
        spec.shadow_size,
        unique.len(),
    );
    if unique.len() == 1 {
        println!("WARNING: shadow is uniform — oracle cannot distinguish between addresses.");
        println!("         See the Shadow Source Investigation journal (2026-04-15).");
    }

    // No pre-case external-input setup needed: `run_case` authoritatively
    // sets `gpio_external_mask` and drives `gpio_external_in` (init seed
    // on first call, stimulus level thereafter). Mirroring it here would
    // be dead writes overwritten on the first iteration below.

    // G.2 caveat inherited from G.1: the very first case (`walk1
    // baseline`, 0x1800) runs before CH0 has had a chance to overwrite
    // CH1.READ_ADDR with a PIO1-decoded address, so its resolved_addr
    // reflects CH1's pre-loaded READ_ADDR=0x20000000 rather than a
    // genuine pin→PIO1→CH0→CH1 round trip. The subsequent 14 cases do
    // exercise the full pipeline as READ_ADDR moves. See HLD §9 and
    // the Stage G journal for details.
    //
    // Per-case one-line output: operator sees progress even if a later
    // case hangs or crashes. Keep the format narrow enough that 15
    // lines fit on one screen.
    let cases = onerom_serving_oracle::default_cases(&spec);
    let total = cases.len();
    let mut wall_us: Vec<f64> = Vec::with_capacity(total);
    for (idx, case) in cases.iter().enumerate() {
        let t0 = Instant::now();
        let result = oracle.run_case(&mut emu, &mut glue, *case);
        let elapsed_us = t0.elapsed().as_nanos() as f64 / 1000.0;
        wall_us.push(elapsed_us);
        let verdict_str = format_verdict_short(&result.verdict);
        let expected = result
            .expected_byte
            .map(|b| format!("0x{:02X}", b))
            .unwrap_or_else(|| "—".to_string());
        let observed = result
            .observed_byte
            .map(|b| format!("0x{:02X}", b))
            .unwrap_or_else(|| "—".to_string());
        let cycles = result
            .latency_cycles
            .map(|c| format!("{}", c))
            .unwrap_or_else(|| "—".to_string());
        println!(
            "[{:>2}/{:>2}] {:<16} pat=0x{:08X} verdict={:<12} expected={} observed={} cycles={} wall={:.3} us",
            idx + 1,
            total,
            result.case.label,
            result.case.pin_pattern as u32,
            verdict_str,
            expected,
            observed,
            cycles,
            elapsed_us,
        );
    }

    // Full Stage G.3 report: header (sys_clk_hz), per-case table,
    // summary (pass/fail counts + latency stats + ROM speed class),
    // and the emulator-bounded caveat. `format_report` is the single
    // canonical renderer — the G.2 inline summary tally has been
    // retired in favor of it.
    println!();
    let sys_clk_hz = emu.bus.sys_clk_hz();
    print!("{}", oracle.format_report(sys_clk_hz));

    // Wall-clock summary.
    let n = wall_us.len() as f64;
    let total_us: f64 = wall_us.iter().sum();
    let mean_us = total_us / n;
    let min_us = wall_us.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_us = wall_us.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    println!();
    println!("wall-clock per case (host-measured, this run) (us):");
    println!("  min  : {:>8.3}", min_us);
    println!("  mean : {:>8.3}", mean_us);
    println!("  max  : {:>8.3}", max_us);
    println!("  total: {:.3} ms over {} cases", total_us / 1000.0, total);

    // Exit code: pass iff every case is Pass.
    let results = oracle.results();
    let pass_count = results
        .iter()
        .filter(|r| matches!(r.verdict, onerom_serving_oracle::Verdict::Pass))
        .count();

    if pass_count == results.len() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// Compact verdict label for the per-case progress line. We want every
/// class to fit in a 12-character column so the table aligns, and we
/// want the reader to distinguish the verdict at a glance without
/// wading through the full `Debug` form.
fn format_verdict_short(v: &onerom_serving_oracle::Verdict) -> String {
    match v {
        onerom_serving_oracle::Verdict::Pass => "Pass".to_string(),
        onerom_serving_oracle::Verdict::WrongByte { .. } => "WrongByte".to_string(),
        onerom_serving_oracle::Verdict::NoResolve => "NoResolve".to_string(),
        onerom_serving_oracle::Verdict::NoStableByte => "NoStableByte".to_string(),
        onerom_serving_oracle::Verdict::ResolvedAddrOutOfRange { .. } => "AddrOOR".to_string(),
        onerom_serving_oracle::Verdict::LatencyOutOfEnvelope { .. } => "LatencyOOE".to_string(),
    }
}

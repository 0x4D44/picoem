//! OneROM serving oracle — byte + timing end-to-end validator (Stage G).
//!
//! Boots the real OneROM firmware, waits for PIO sync, snapshots the
//! SRAM-base ROM table, then runs the Stage G case set through the
//! serving pipeline. For each case, drives the pin stimulus, watches the
//! glue DMA pump, and proves the byte observed on D0..D7 matches the
//! shadow byte at the resolved CH1.READ_ADDR.
//!
//! Stage G.2 — full 15-case walking-1s + pattern sweep through the
//! serving pipeline. One line per case scrolls as the run proceeds so
//! a stuck or crashing case is obvious from the partial output. G.3
//! adds the latency report + ns conversion + envelope check.
//!
//! Design: `wrk_docs/2026.04.15 - HLD - OneROM Serving Oracle (Stage G).md`.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin onerom_serving_oracle_rp2350 --release

use std::collections::HashSet;
use std::process::ExitCode;

use mdpicoem_harness::{onerom_glue_dma, onerom_serving_oracle, onerom_sync};
use mdrp2350::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0.bin";

/// Cycle cap for boot. Same budget as Stage F's binary — sync normally
/// arrives around cycle 7k; 10M is generous.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

fn main() -> ExitCode {
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
        .build();
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
        emu.run(1);
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

    // Prime the DMA and capture the SRAM shadow.
    glue.prime_after_sync(&mut emu.bus);
    let mut oracle = onerom_serving_oracle::ServingOracle::new_at_sync(&mut emu.bus);

    // Shadow-integrity tripwire (devil's-advocate Attack 2, Stage G.2):
    // `ServingOracle::new_at_sync` captures the 8 KB shadow from SRAM at
    // SHADOW_BASE. If that region is uniform (all zeros, all 0xFF, etc.),
    // no address-decode bug can possibly be caught — every resolved_addr
    // would look up the same byte. We sample the same SRAM region the
    // oracle did and surface the unique-byte count so the operator sees
    // the tripwire before reading a false PASS.
    let mut shadow_sample = [0u8; onerom_serving_oracle::SHADOW_SIZE];
    for i in 0..onerom_serving_oracle::SHADOW_SIZE {
        shadow_sample[i] = emu.bus.memory.sram_read8(i as u32);
    }
    let unique: HashSet<u8> = shadow_sample.iter().copied().collect();
    println!(
        "shadow @ 0x{:08X}..+0x{:04X}: {} unique bytes",
        onerom_serving_oracle::SHADOW_BASE,
        onerom_serving_oracle::SHADOW_SIZE,
        unique.len(),
    );
    if unique.len() == 1 {
        println!(
            "WARNING: shadow is uniform — oracle cannot distinguish between addresses."
        );
        println!(
            "         See \"Shadow source investigation\" in the Stage G journal."
        );
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
    let total = onerom_serving_oracle::DEFAULT_CASES.len();
    for (idx, case) in onerom_serving_oracle::DEFAULT_CASES.iter().enumerate() {
        let result = oracle.run_case(&mut emu, &mut glue, *case);
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
        // TODO(G.3): include `resolved=0x...` column in per-case output
        // (see onerom_serving_oracle format_report). Once the parallel
        // PIO2 fix lands, knowing *where* WrongByte read from is
        // diagnostic gold; G.3's full report formatter will fold it in.
        println!(
            "[{:>2}/{:>2}] {:<16} addr=0x{:04X} verdict={:<12} expected={} observed={} cycles={}",
            idx + 1,
            total,
            result.case.label,
            result.case.addr_bits,
            verdict_str,
            expected,
            observed,
            cycles,
        );
    }

    // Summary tally — bucket verdicts by class so the operator sees at
    // a glance what failed and why. `WrongByte` is expected to dominate
    // until the parallel PIO2 pad_out team lands their fix; other
    // variants are real findings worth surfacing separately.
    let results = oracle.results();
    let mut pass = 0usize;
    let mut wrong_byte = 0usize;
    let mut no_resolve = 0usize;
    let mut no_stable = 0usize;
    let mut out_of_range = 0usize;
    let mut latency_out = 0usize;
    for r in results {
        match r.verdict {
            onerom_serving_oracle::Verdict::Pass => pass += 1,
            onerom_serving_oracle::Verdict::WrongByte { .. } => wrong_byte += 1,
            onerom_serving_oracle::Verdict::NoResolve => no_resolve += 1,
            onerom_serving_oracle::Verdict::NoStableByte => no_stable += 1,
            onerom_serving_oracle::Verdict::ResolvedAddrOutOfRange { .. } => {
                out_of_range += 1
            }
            onerom_serving_oracle::Verdict::LatencyOutOfEnvelope { .. } => {
                latency_out += 1
            }
        }
    }
    let fail = total - pass;
    println!(
        "{}/{} PASS, {}/{} FAIL ({} wrong byte, {} no-resolve, {} no-stable-byte, {} addr-out-of-range, {} latency-out-of-envelope)",
        pass, total, fail, total,
        wrong_byte, no_resolve, no_stable, out_of_range, latency_out,
    );

    if pass == total {
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
        onerom_serving_oracle::Verdict::ResolvedAddrOutOfRange { .. } => {
            "AddrOOR".to_string()
        }
        onerom_serving_oracle::Verdict::LatencyOutOfEnvelope { .. } => {
            "LatencyOOE".to_string()
        }
    }
}

//! OneROM serving oracle — byte + timing end-to-end validator (Stage G).
//!
//! Boots the real OneROM firmware, waits for PIO sync, snapshots the
//! SRAM-base ROM table, then runs the Stage G case set through the
//! serving pipeline. For each case, drives the pin stimulus, watches the
//! glue DMA pump, and proves the byte observed on D0..D7 matches the
//! shadow byte at the resolved CH1.READ_ADDR.
//!
//! Stage G.1 — one baseline case (`0x1800`). G.2 lights up the full
//! walking-1s + pattern sweep. G.3 adds the latency report.
//!
//! Design: `wrk_docs/2026.04.15 - HLD - OneROM Serving Oracle (Stage G).md`.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin onerom_serving_oracle_rp2350 --release

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use mdpicoem_harness::{onerom_glue_dma, onerom_serving_oracle, onerom_sync};
use mdrp2350::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0.bin";

/// Cycle cap for boot. Same budget as Stage F's binary — sync normally
/// arrives around cycle 7k; 10M is generous.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// Data bus + CS lanes + address pins. Mirrored from Stage F.
const GPIO_CS1: u8 = 13;
const GPIO_CS2: u8 = 12;
const GPIO_CS3: u8 = 15;
const ADDR_PINS: [u8; 13] = [7, 6, 5, 4, 3, 2, 1, 0, 10, 11, 14, 15, 12];

fn repo_root_relative(rel: &str) -> PathBuf {
    Path::new(rel).to_path_buf()
}

fn main() -> ExitCode {
    let bootrom_path = repo_root_relative(BOOTROM_PATH);
    let flash_path = repo_root_relative(FLASH_PATH);

    let bootrom = match std::fs::read(&bootrom_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom at {}: {}", bootrom_path.display(), e);
            return ExitCode::from(2);
        }
    };

    let flash = match std::fs::read(&flash_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read flash image at {}: {}", flash_path.display(), e);
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

    // Establish the external-input overlay exactly like Stage F's
    // post-sync hook: CS1 high (for the init seed inside `run_case`),
    // CS2/CS3 high, address pins = 0. The mask covers CS1/CS2/CS3 and
    // all address pins; D0..D7 stay PIO-driven.
    let ext_mask: u32 = (1u32 << GPIO_CS1)
        | (1u32 << GPIO_CS2)
        | (1u32 << GPIO_CS3)
        | ADDR_PINS.iter().fold(0u32, |a, &p| a | (1u32 << p));
    let seed_level: u32 = (1u32 << GPIO_CS1) | (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
    emu.bus.gpio_external_mask = ext_mask;
    emu.bus.gpio_external_in = seed_level;

    // Run each case.
    for case in onerom_serving_oracle::DEFAULT_CASES {
        let result = oracle.run_case(&mut emu, &mut glue, *case);
        println!(
            "CASE {:<28} addr=0x{:04X}  verdict={:?}  resolved={}  expected={}  observed={}  cycles={}",
            result.case.label,
            result.case.addr_bits,
            result.verdict,
            result
                .resolved_addr
                .map(|a| format!("0x{:08X}", a))
                .unwrap_or_else(|| "—".to_string()),
            result
                .expected_byte
                .map(|b| format!("0x{:02X}", b))
                .unwrap_or_else(|| "—".to_string()),
            result
                .observed_byte
                .map(|b| format!("0x{:02X}", b))
                .unwrap_or_else(|| "—".to_string()),
            result
                .latency_cycles
                .map(|c| format!("{}", c))
                .unwrap_or_else(|| "—".to_string()),
        );
    }

    let sys_hz = emu.bus.sys_clk_hz();
    println!();
    println!("{}", oracle.format_report(sys_hz));

    let all_pass = oracle
        .results()
        .iter()
        .all(|r| r.verdict == onerom_serving_oracle::Verdict::Pass);

    if all_pass {
        println!("SERVING ORACLE PASS — all {} case(s) matched the SRAM shadow",
                 oracle.results().len());
        ExitCode::SUCCESS
    } else {
        println!("SERVING ORACLE FAIL — see per-case verdicts above");
        ExitCode::FAILURE
    }
}

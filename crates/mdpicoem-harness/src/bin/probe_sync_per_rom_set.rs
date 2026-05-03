//! Disposable diagnostic: sync-probe each ROM set in a fixture.
//!
//! Given a fixture path (defaults to the SeaBIOS fixture's 1541
//! template), iterate `rom_set_count` (read from the metadata header)
//! and attempt boot-sync for each set with no byte serving — just
//! report the cycle at which sync succeeds, or that the boot timed
//! out. Lets us tell quickly whether an alternative template (e.g.
//! `test-sdrr-0`) has more usable ROM-set slots than the 1541 one.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin probe_sync_per_rom_set --release -- \
//!       --fixture <path>
//!
//! Output is one line per ROM set:
//!   set N: SYNCED at cycle <c>
//!   set N: TIMEOUT after <cap> cycles (boot did not reach serve loop)

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use mdpicoem_harness::{onerom_serving_oracle, onerom_serving_oracle_cpu};
use mdrp2350::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const DEFAULT_FIXTURE: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

/// 5M cycles is the per-set budget called for in the supervisor task —
/// half the byte-correct binary's 10M cap, but still well above the
/// observed sync times (~25k cycles for sets that work).
const BOOT_CYCLE_CAP: u64 = 5_000_000;

struct Cli {
    fixture: PathBuf,
}

fn parse_cli() -> Result<Cli, String> {
    let mut fixture = PathBuf::from(DEFAULT_FIXTURE);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--fixture" => {
                fixture = PathBuf::from(args.next().ok_or("--fixture needs a value")?)
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: probe_sync_per_rom_set [--fixture <path>]\n\
                     defaults to {DEFAULT_FIXTURE}"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unrecognised argument: {other}")),
        }
    }
    Ok(Cli { fixture })
}

fn probe_one(bootrom: &[u8], flash: &[u8], rom_set_index: u32) -> Result<u64, String> {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .map_err(|e| format!("emulator build failed: {e:?}"))?;
    emu.load_bootrom(bootrom);
    emu.load_flash(flash);
    emu.reset();

    // Bootrom bypass — OneROM flash is not an IMAGE_DEF block.
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);

    // CPU-serve mode is single-core.
    emu.core_mut(1).halt();

    onerom_serving_oracle_cpu::force_rom_set_index_via_sel_pins(&mut emu, flash, rom_set_index)?;

    // Phase 1: PC enters the serve-loop range.
    let mut phase1_cycle: Option<u64> = None;
    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            return Err(format!("cycle counter stalled at {before}"));
        }
        if onerom_serving_oracle_cpu::is_synced_cpu(&emu, None) {
            phase1_cycle = Some(after);
            break;
        }
    }
    let phase1 = phase1_cycle.ok_or_else(|| {
        format!(
            "boot did not reach CPU serve-loop PC within {} cycles",
            BOOT_CYCLE_CAP
        )
    })?;

    // Phase 2: PC + shadow tripwire.
    const RUNTIME_INFO_SRAM_OFF: u32 = 0x0008_0000;
    const ROM_SET_INDEX_OFFSET: u32 = 6;
    let live_index = emu
        .bus
        .memory
        .sram_read8(RUNTIME_INFO_SRAM_OFF + ROM_SET_INDEX_OFFSET);
    let sentinel: Option<(u32, u8)> =
        match onerom_serving_oracle::lift_shadow_from_flash_pub(flash, live_index) {
            Some(shadow) => onerom_serving_oracle_cpu::find_shadow_sentinel(&shadow),
            None => None,
        };

    while !onerom_serving_oracle_cpu::is_synced_cpu(&emu, sentinel) && emu.cycles() < BOOT_CYCLE_CAP
    {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            return Err(format!("cycle counter stalled at {before}"));
        }
    }
    if !onerom_serving_oracle_cpu::is_synced_cpu(&emu, sentinel) {
        return Err(format!(
            "boot did not reach CPU serve-loop sync (PC + sentinel) within {} cycles \
             (Phase 1 reached PC at {})",
            BOOT_CYCLE_CAP, phase1
        ));
    }

    Ok(emu.cycles())
}

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

    let cli = match parse_cli() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    };

    let flash = match std::fs::read(&cli.fixture) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read fixture {}: {}", cli.fixture.display(), e);
            return ExitCode::from(2);
        }
    };
    let bootrom = match std::fs::read(BOOTROM_PATH) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom {}: {}", BOOTROM_PATH, e);
            return ExitCode::from(2);
        }
    };

    let layout = match onerom_serving_oracle::parse_rom_set_layout(&flash) {
        Some(v) => v,
        None => {
            eprintln!("parse_rom_set_layout returned None — fixture metadata mismatch");
            return ExitCode::from(3);
        }
    };
    let rom_set_count = layout.len();
    println!("fixture: {} ({} bytes)", cli.fixture.display(), flash.len());
    println!("rom_set_count: {}", rom_set_count);
    println!("budget: {} cycles per set", BOOT_CYCLE_CAP);
    println!();

    let mut had_failure = false;
    for k in 0..rom_set_count as u32 {
        let t0 = Instant::now();
        match probe_one(&bootrom, &flash, k) {
            Ok(c) => println!(
                "set {}: SYNCED at cycle {} ({} ms wall)",
                k,
                c,
                t0.elapsed().as_millis()
            ),
            Err(e) => {
                println!(
                    "set {}: TIMEOUT — {} ({} ms wall)",
                    k,
                    e,
                    t0.elapsed().as_millis()
                );
                had_failure = true;
            }
        }
    }

    if had_failure {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

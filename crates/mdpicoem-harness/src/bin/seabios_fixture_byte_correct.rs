//! Validates that the OneROM-served bytes match the SeaBIOS image for every
//! reachable pin state. Per-fixture max byte-correct coverage is 128 KiB
//! (32 KiB × 4 ROM sets at CS1=low) — the firmware tristates D0..D7 when
//! CS1 is high, so the upper half of each ROM set's shadow is unservable
//! through this firmware path.
//!
//! ## Architectural ceiling
//!
//! The fire-24-a SDRR firmware tristates D0..D7 whenever CS1 (bit 13) is
//! high. This was confirmed empirically on 2026-05-03; see the journal
//! `wrk_journals/2026.05.03 - JRN - SDRR SeaBIOS fixture.md`.
//!
//! Consequence: 32 KiB byte-correct verifiable per ROM set × 4 sets =
//! 128 KiB max coverage per fixture. The upper 128 KiB of the SeaBIOS
//! image is laid into the shadow but is unservable through this firmware.
//! Serving all 256 KiB requires either a different SDRR firmware variant
//! with a wider pin map (the README mentions 27C-series EPROMs up to
//! 512 KiB), or a two-fixture topology where Stream B's worker reloads
//! the emulator with a different fixture for the upper 128 KiB.
//!
//! For each of the 4 ROM sets in
//! `onerom-fire-24-a-rp2350-seabios-cpu.bin`, boot the firmware,
//! force the ROM set selection, sync the CPU into the serve loop, then
//! drive every 16-bit GPIO pin pattern (skipping CS1=high cases as
//! unservable) through [`CpuServingOracle::run_case`] and confirm the
//! served byte matches the corresponding byte in the SeaBIOS image.
//!
//! Usage:
//!   cargo run --release -p mdpicoem-harness --bin seabios_fixture_byte_correct
//!   cargo run --release -p mdpicoem-harness --bin seabios_fixture_byte_correct -- --smoke
//!   cargo run --release -p mdpicoem-harness --bin seabios_fixture_byte_correct -- \
//!       --fixture <path> --seabios <path>

use std::io::Write as _;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use mdpicoem_harness::{
    onerom_serving_oracle,
    onerom_serving_oracle_cpu::{self, CpuServingOracle, CpuVerdict},
};
use mdrp2350::{Config, Emulator, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const DEFAULT_FIXTURE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/onerom-fire-24-a-rp2350-seabios-cpu.bin"
);
const DEFAULT_SEABIOS: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/sources/seabios-256k.bin"
);

const SHADOW_SIZE: usize = onerom_serving_oracle::SHADOW_SIZE;
const SEABIOS_SIZE: usize = 4 * SHADOW_SIZE;
const NUM_ROM_SETS: u32 = 4;

/// Boot-sync cycle cap. Sets 0/1/2 sync within ~25k cycles in practice;
/// 10M is generous. NOTE: rom_set 3 in the existing 1541 fixture also
/// fails to sync within this budget (verified independently with the
/// unmodified 1541 fixture) — a pre-existing emulator/firmware quirk
/// surfaced by this validator, not something we can fix here.
const BOOT_CYCLE_CAP: u64 = 10_000_000;
const PROGRESS_INTERVAL: usize = 4096;

/// Bit 13 in the GPIO_IN word is CS1; firmware tristates the data pins
/// while it's high, so we never drive that combination.
const CS1_BIT: u16 = 1 << 13;

struct Cli {
    fixture: PathBuf,
    seabios: PathBuf,
    smoke: bool,
}

fn parse_cli() -> Result<Cli, String> {
    let mut fixture = PathBuf::from(DEFAULT_FIXTURE);
    let mut seabios = PathBuf::from(DEFAULT_SEABIOS);
    let mut smoke = false;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--fixture" => fixture = PathBuf::from(args.next().ok_or("--fixture needs a value")?),
            "--seabios" => seabios = PathBuf::from(args.next().ok_or("--seabios needs a value")?),
            "--smoke" => smoke = true,
            "-h" | "--help" => {
                eprintln!(
                    "usage: seabios_fixture_byte_correct [--fixture <path>] [--seabios <path>] [--smoke]\n\
                     --smoke      runs all 4 ROM sets with the first 256 pin states each (fast spot check)."
                );
                std::process::exit(0);
            }
            other => return Err(format!("unrecognised argument: {other}")),
        }
    }
    Ok(Cli {
        fixture,
        seabios,
        smoke,
    })
}

/// Boot-sync helper: load bootrom + flash, reset, halt core 1, force
/// the requested ROM-set index via the image-select GPIOs, then run
/// the emulator until core 0's PC is in the CPU serve loop and the
/// shadow tripwire fires. Mirrors `onerom_cpu_speed_grade_serial_rp2350`.
fn boot_sync(bootrom: &[u8], flash: &[u8], rom_set_index: u32) -> Result<Emulator, String> {
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
    let mut phase1: Option<u64> = None;
    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            return Err(format!("cycle counter stalled at {before}"));
        }
        if onerom_serving_oracle_cpu::is_synced_cpu(&emu, None) {
            phase1 = Some(after);
            break;
        }
    }
    if phase1.is_none() {
        return Err(format!(
            "boot did not reach CPU serve-loop PC within {} cycles (rom_set={})",
            BOOT_CYCLE_CAP, rom_set_index
        ));
    }

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
            "boot did not reach CPU serve-loop sync (PC + sentinel) within {} cycles (rom_set={})",
            BOOT_CYCLE_CAP, rom_set_index
        ));
    }

    Ok(emu)
}

#[derive(Default)]
struct SetTally {
    pass: usize,
    wrong: usize,
    no_stable: usize,
    not_driven: usize,
    latency_oor: usize,
    unservable_cs1: usize,
    first_wrong: Option<(u16, u8, u8)>,
}

fn run_set(
    flash: &[u8],
    seabios: &[u8],
    rom_set_index: u32,
    pin_lo: u32,
    pin_hi: u32,
) -> Result<SetTally, String> {
    println!(
        "=== rom_set {} (pin states 0x{:04X}..0x{:04X}, {} cases) ===",
        rom_set_index,
        pin_lo,
        pin_hi - 1,
        pin_hi - pin_lo
    );
    let _ = std::io::stdout().flush();

    let bootrom = std::fs::read(BOOTROM_PATH).map_err(|e| format!("bootrom: {e}"))?;
    let mut emu = boot_sync(&bootrom, flash, rom_set_index)?;
    println!("  synced at cycle {}", emu.cycles());
    let _ = std::io::stdout().flush();

    let mut oracle = CpuServingOracle::new_at_sync(&mut emu.bus, flash);
    let shadow = oracle.shadow();
    let unique = shadow.iter().copied().collect::<std::collections::HashSet<u8>>().len();
    println!("  shadow: {} unique bytes", unique);

    // Cross-check: the lifted shadow MUST match the corresponding 64 KiB
    // chunk of seabios. If not, something went wrong with the build —
    // bail out before running 65k useless cases.
    let chunk_lo = (rom_set_index as usize) * SHADOW_SIZE;
    let chunk_hi = chunk_lo + SHADOW_SIZE;
    let chunk = &seabios[chunk_lo..chunk_hi];
    let mut shadow_mismatch = 0usize;
    for i in 0..SHADOW_SIZE {
        if shadow[i] != chunk[i] {
            shadow_mismatch += 1;
        }
    }
    if shadow_mismatch != 0 {
        return Err(format!(
            "rom_set {}: lifted shadow differs from seabios chunk in {} bytes — fixture build is broken",
            rom_set_index, shadow_mismatch
        ));
    }

    let mut tally = SetTally::default();
    let t0 = Instant::now();

    for pin_state in pin_lo..pin_hi {
        if (pin_state as u16) & CS1_BIT != 0 {
            tally.unservable_cs1 += 1;
            continue;
        }
        let case = onerom_serving_oracle::Case::raw_pin_state("seabios", pin_state as u16);
        let result = oracle.run_case(&mut emu, case);
        let expected = chunk[pin_state as usize];

        match result.verdict {
            CpuVerdict::Pass => {
                if result.observed_byte == Some(expected) {
                    tally.pass += 1;
                } else {
                    tally.wrong += 1;
                    if tally.first_wrong.is_none() {
                        tally.first_wrong = Some((
                            pin_state as u16,
                            expected,
                            result.observed_byte.unwrap_or(0xAA),
                        ));
                    }
                }
            }
            CpuVerdict::WrongByte { expected: e, observed } => {
                tally.wrong += 1;
                if tally.first_wrong.is_none() {
                    tally.first_wrong = Some((pin_state as u16, e, observed));
                }
            }
            CpuVerdict::NoStableByte => tally.no_stable += 1,
            CpuVerdict::DataPinsNotDriven => tally.not_driven += 1,
            CpuVerdict::LatencyOutOfEnvelope { .. } => {
                // Latency-only deviation; if the byte was correct we still
                // count it as a pass for byte-correctness purposes.
                if result.observed_byte == Some(expected) {
                    tally.pass += 1;
                } else {
                    tally.latency_oor += 1;
                }
            }
        }

        let done = (pin_state - pin_lo + 1) as usize;
        if done % PROGRESS_INTERVAL == 0 {
            let total = (pin_hi - pin_lo) as usize;
            let elapsed_ms = t0.elapsed().as_millis();
            println!(
                "    progress: {}/{}  pass={} wrong={} no_stable={} not_driven={} unservable_cs1={}  ({} ms)",
                done,
                total,
                tally.pass,
                tally.wrong,
                tally.no_stable,
                tally.not_driven,
                tally.unservable_cs1,
                elapsed_ms
            );
            let _ = std::io::stdout().flush();
        }
    }

    let elapsed_ms = t0.elapsed().as_millis();
    println!(
        "  done in {} ms: pass={} (32 KiB byte-correct) wrong={} no_stable={} not_driven={} latency_oor={} unservable_cs1={} (CS1=high → firmware tristates D0..D7)",
        elapsed_ms,
        tally.pass,
        tally.wrong,
        tally.no_stable,
        tally.not_driven,
        tally.latency_oor,
        tally.unservable_cs1
    );
    if let Some((p, e, o)) = tally.first_wrong {
        println!(
            "  first wrong: pin_state=0x{:04X} expected=0x{:02X} observed=0x{:02X}",
            p, e, o
        );
    }
    let _ = std::io::stdout().flush();
    Ok(tally)
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

    eprintln!(
        "seabios_fixture_byte_correct: bootrom={} fixture={} seabios={}",
        std::path::Path::new(BOOTROM_PATH).display(),
        cli.fixture.display(),
        cli.seabios.display()
    );

    println!("fixture: {}", cli.fixture.display());
    println!("seabios: {}", cli.seabios.display());
    println!(
        "mode:    {}",
        if cli.smoke { "smoke" } else { "full" }
    );
    let _ = std::io::stdout().flush();

    let flash = match std::fs::read(&cli.fixture) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read fixture: {e}");
            return ExitCode::from(2);
        }
    };
    let seabios = match std::fs::read(&cli.seabios) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read seabios: {e}");
            return ExitCode::from(2);
        }
    };
    if seabios.len() != SEABIOS_SIZE {
        eprintln!("seabios image must be {} bytes; got {}", SEABIOS_SIZE, seabios.len());
        return ExitCode::from(3);
    }

    // Smoke runs all 4 sets with the first 256 pin states each — fast
    // spot check that exercises every set's serve path against (often)
    // non-zero shadow data; the full run drives every 16-bit pin state
    // for an exhaustive byte-correctness sweep.
    let (sets, pin_hi): (std::ops::Range<u32>, u32) = if cli.smoke {
        (0..NUM_ROM_SETS, 256)
    } else {
        (0..NUM_ROM_SETS, 0x1_0000)
    };

    let mut grand = SetTally::default();
    let mut had_failure = false;
    for k in sets {
        match run_set(&flash, &seabios, k, 0, pin_hi) {
            Ok(t) => {
                grand.pass += t.pass;
                grand.wrong += t.wrong;
                grand.no_stable += t.no_stable;
                grand.not_driven += t.not_driven;
                grand.latency_oor += t.latency_oor;
                grand.unservable_cs1 += t.unservable_cs1;
                if grand.first_wrong.is_none() {
                    grand.first_wrong = t.first_wrong;
                }
                if t.wrong != 0 || t.no_stable != 0 || t.not_driven != 0 || t.latency_oor != 0 {
                    had_failure = true;
                }
            }
            Err(e) => {
                eprintln!("rom_set {} FAILED: {}", k, e);
                had_failure = true;
            }
        }
    }

    let total_cases = grand.pass
        + grand.wrong
        + grand.no_stable
        + grand.not_driven
        + grand.latency_oor
        + grand.unservable_cs1;
    println!();
    println!("=== grand total ===");
    println!("  pass            = {}  (= {} KiB byte-correct verified)", grand.pass, grand.pass / 1024);
    println!("  wrong           = {}", grand.wrong);
    println!("  no_stable       = {}", grand.no_stable);
    println!("  not_driven      = {}", grand.not_driven);
    println!("  latency_oor     = {}", grand.latency_oor);
    println!(
        "  unservable_cs1  = {}  (firmware tristates D0..D7 at CS1=high — bytes are present in the fixture but unreachable through the existing serve loop)",
        grand.unservable_cs1
    );
    println!(
        "fixture capacity  = {} bytes ({} KiB), serve coverage = {} bytes ({} KiB)",
        total_cases,
        total_cases / 1024,
        grand.pass,
        grand.pass / 1024
    );
    if let Some((p, e, o)) = grand.first_wrong {
        println!(
            "  first wrong overall: pin_state=0x{:04X} expected=0x{:02X} observed=0x{:02X}",
            p, e, o
        );
    }

    if had_failure {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

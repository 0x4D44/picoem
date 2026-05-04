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
//!   cargo run --release -p picoem-harness --bin seabios_fixture_byte_correct
//!   cargo run --release -p picoem-harness --bin seabios_fixture_byte_correct -- --smoke
//!   cargo run --release -p picoem-harness --bin seabios_fixture_byte_correct -- \
//!       --fixture <path> --seabios <path>

use std::io::Write as _;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use picoem_harness::onerom_fixture::{FixtureSpec, lift_shadow_from_flash};
use picoem_harness::{
    onerom_serving_oracle,
    onerom_serving_oracle_cpu::{self, CpuServingOracle, CpuVerdict},
};
use rp2350_emu::{Config, Emulator, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const DEFAULT_FIXTURE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/onerom-fire-24-a-rp2350-seabios-cpu.bin"
);
const DEFAULT_SEABIOS: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/sources/seabios-256k.bin"
);

/// Hardcoded fire-24-a SeaBIOS shadow size (one ROM set = 64 KiB).
/// The fixture-aware spec parse below gives the same value via
/// `spec.shadow_size`; this constant is kept so the SEABIOS_SIZE
/// computation has a `const` to multiply against.
const FIRE24A_SHADOW_SIZE: usize = 0x1_0000;
const SEABIOS_SIZE: usize = 4 * FIRE24A_SHADOW_SIZE;
const NUM_ROM_SETS: u32 = 4;

// 10M is generous; observed sync cycles in practice are ~25K. The
// unmodified 1541 fixture has rom_set 3 failing to sync (broken
// roms[] pointer in the template). build_seabios_fixture patches
// that descriptor field, so all 4 SeaBIOS-fixture sets sync inside
// ~25K cycles.
const BOOT_CYCLE_CAP: u64 = 10_000_000;
const PROGRESS_INTERVAL: usize = 4096;

// CS1 mask (the "unservable when high" pin pattern) is read from the
// FixtureSpec at run time — see `run_set` and `run_probe_cs1_thorough`.
// The legacy hardcoded `1 << 13` constant is gone; for fire-24-a it
// resolves to the same value via `spec.unservable_when_high`.

struct Cli {
    fixture: PathBuf,
    seabios: PathBuf,
    smoke: bool,
    probe_cs1_thorough: bool,
}

fn parse_cli() -> Result<Cli, String> {
    let mut fixture = PathBuf::from(DEFAULT_FIXTURE);
    let mut seabios = PathBuf::from(DEFAULT_SEABIOS);
    let mut smoke = false;
    let mut probe_cs1_thorough = false;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--fixture" => fixture = PathBuf::from(args.next().ok_or("--fixture needs a value")?),
            "--seabios" => seabios = PathBuf::from(args.next().ok_or("--seabios needs a value")?),
            "--smoke" => smoke = true,
            "--probe-cs1-thorough" => probe_cs1_thorough = true,
            "-h" | "--help" => {
                eprintln!(
                    "usage: seabios_fixture_byte_correct [--fixture <path>] [--seabios <path>] [--smoke | --probe-cs1-thorough]\n\
                     --smoke                 runs all 4 ROM sets with the first 256 pin states each (fast spot check).\n\
                     --probe-cs1-thorough    drives 256 deterministic LCG-seeded random pin states with CS1=high\n\
                                             on rom_set 1 (varied content) and reports the verdict distribution.\n\
                                             Used to gather empirical evidence for the firmware-tristates-at-CS1=high\n\
                                             claim. The regular full-sweep skip path stays unchanged."
                );
                std::process::exit(0);
            }
            other => return Err(format!("unrecognised argument: {other}")),
        }
    }
    if smoke && probe_cs1_thorough {
        return Err("--smoke and --probe-cs1-thorough are mutually exclusive".into());
    }
    Ok(Cli {
        fixture,
        seabios,
        smoke,
        probe_cs1_thorough,
    })
}

/// Boot-sync helper: load bootrom + flash, reset, halt core 1, force
/// the requested ROM-set index via the image-select GPIOs, then run
/// the emulator until core 0's PC is in the CPU serve loop and the
/// shadow tripwire fires. Mirrors `onerom_cpu_speed_grade_serial_rp2350`.
fn boot_sync(
    bootrom: &[u8],
    flash: &[u8],
    spec: &FixtureSpec,
    rom_set_index: u32,
) -> Result<Emulator, String> {
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
    let sentinel: Option<(u32, u8)> = match lift_shadow_from_flash(flash, live_index, spec) {
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
    /// Subset of `pass` where the expected byte is NOT equal to the
    /// most-common (modal) expected byte for the set's CS1-low pin
    /// states. For chunk 0 (all zeros) `discriminating_pass = 0` because
    /// every served byte is the modal byte — the pass count proves
    /// nothing about per-pin-state correctness. For chunks with varied
    /// data this approaches the full pass count.
    discriminating_pass: usize,
    /// Number of unique expected-byte values across the set's CS1-low
    /// pin states (i.e. across the lower 32 KiB of the set's chunk that
    /// the firmware can serve). Surfaced so the human can see at a
    /// glance whether the chunk is trivial (1 unique byte) or varied.
    unique_expected_bytes: usize,
    wrong: usize,
    no_stable: usize,
    not_driven: usize,
    latency_oor: usize,
    unservable_cs1: usize,
    first_wrong: Option<(u16, u8, u8)>,
}

/// Returns the most-common byte across the supplied slice. Ties broken
/// by lowest byte value (stable). Used to identify the "trivial pass"
/// byte for the discriminating-pass metric.
fn modal_byte(bytes: &[u8]) -> u8 {
    let mut counts = [0u32; 256];
    for &b in bytes {
        counts[b as usize] += 1;
    }
    let mut best_byte = 0u8;
    let mut best_count = 0u32;
    for (i, &c) in counts.iter().enumerate() {
        if c > best_count {
            best_count = c;
            best_byte = i as u8;
        }
    }
    best_byte
}

fn run_set(
    flash: &[u8],
    spec: &FixtureSpec,
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
    let mut emu = boot_sync(&bootrom, flash, spec, rom_set_index)?;
    println!("  synced at cycle {}", emu.cycles());
    let _ = std::io::stdout().flush();

    let mut oracle = CpuServingOracle::new_at_sync(&mut emu.bus, spec.clone(), flash);
    let shadow_size = spec.shadow_size;
    // CS1 mask: the pattern bit(s) that, when high, make the chip
    // unservable. For fire-24-a this is `1 << 13` (the legacy literal);
    // we use the FixtureSpec field so other fixtures plug in unchanged.
    // The cs1_mask narrowed to u16 is what the loop below tests against
    // the 16-bit pin pattern.
    let cs1_mask = spec.unservable_when_high as u16;
    let unique = oracle
        .shadow()
        .iter()
        .copied()
        .collect::<std::collections::HashSet<u8>>()
        .len();
    println!("  shadow: {} unique bytes", unique);

    // Cross-check: the lifted shadow MUST match the corresponding chunk
    // of seabios. If not, something went wrong with the build — bail
    // out before running 65k useless cases.
    let chunk_lo = (rom_set_index as usize) * shadow_size;
    let chunk_hi = chunk_lo + shadow_size;
    let chunk = &seabios[chunk_lo..chunk_hi];
    let mut shadow_mismatch = 0usize;
    for i in 0..shadow_size {
        if oracle.shadow()[i] != chunk[i] {
            shadow_mismatch += 1;
        }
    }
    if shadow_mismatch != 0 {
        return Err(format!(
            "rom_set {}: lifted shadow differs from seabios chunk in {} bytes — fixture build is broken",
            rom_set_index, shadow_mismatch
        ));
    }

    // Pre-compute the set's "servable" expected bytes so we can derive
    // the modal byte and the unique-byte count BEFORE the sweep.
    let mut servable_bytes = Vec::with_capacity((pin_hi - pin_lo) as usize);
    for pin_state in pin_lo..pin_hi {
        if (pin_state as u16) & cs1_mask == 0 {
            servable_bytes.push(chunk[pin_state as usize]);
        }
    }
    let set_modal = modal_byte(&servable_bytes);
    let unique_count = servable_bytes
        .iter()
        .copied()
        .collect::<std::collections::HashSet<u8>>()
        .len();

    let mut tally = SetTally {
        unique_expected_bytes: unique_count,
        ..SetTally::default()
    };
    let t0 = Instant::now();

    for pin_state in pin_lo..pin_hi {
        if (pin_state as u16) & cs1_mask != 0 {
            tally.unservable_cs1 += 1;
            continue;
        }
        let case = onerom_serving_oracle::Case::from_raw("seabios", pin_state as u64);
        let result = oracle.run_case(&mut emu, case);
        let expected = chunk[pin_state as usize];

        let mut count_pass = || {
            tally.pass += 1;
            if expected != set_modal {
                tally.discriminating_pass += 1;
            }
        };

        match result.verdict {
            CpuVerdict::Pass => {
                if result.observed_byte == Some(expected) {
                    count_pass();
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
            CpuVerdict::WrongByte {
                expected: e,
                observed,
            } => {
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
                    count_pass();
                } else {
                    tally.latency_oor += 1;
                }
            }
        }

        let done = (pin_state - pin_lo + 1) as usize;
        if done.is_multiple_of(PROGRESS_INTERVAL) {
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
        "  done in {} ms: pass={} ({} discriminating-pass, {} unique expected bytes; CS1-low half = 32 KiB) wrong={} no_stable={} not_driven={} latency_oor={} unservable_cs1={} (CS1=high → firmware tristates D0..D7)",
        elapsed_ms,
        tally.pass,
        tally.discriminating_pass,
        tally.unique_expected_bytes,
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

/// Constant LCG seed for `--probe-cs1-thorough`. Reproducible across
/// runs — no thread-rng, no clock-based seed (per the journal's
/// determinism requirement). Picked arbitrarily.
const PROBE_CS1_LCG_SEED: u64 = 0xC51E_C51E_C51E_C51E;
const PROBE_CS1_NUM_CASES: usize = 256;

/// Run the `--probe-cs1-thorough` mode: drive 256 deterministic-LCG-
/// seeded random pin states with CS1=high on rom_set 1 (chunk 1 has
/// varied non-zero content, so any silent OEN-asserts-with-zero-output
/// firmware bug would be visible). Reports verdict distribution.
fn run_probe_cs1_thorough(flash: &[u8], spec: &FixtureSpec, seabios: &[u8]) -> Result<(), String> {
    const ROM_SET_INDEX: u32 = 1;

    println!(
        "=== --probe-cs1-thorough (rom_set {}, {} CS1=high cases, LCG seed 0x{:016X}) ===",
        ROM_SET_INDEX, PROBE_CS1_NUM_CASES, PROBE_CS1_LCG_SEED
    );
    let _ = std::io::stdout().flush();

    let bootrom = std::fs::read(BOOTROM_PATH).map_err(|e| format!("bootrom: {e}"))?;
    let mut emu = boot_sync(&bootrom, flash, spec, ROM_SET_INDEX)?;
    println!("  synced at cycle {}", emu.cycles());

    let mut oracle = CpuServingOracle::new_at_sync(&mut emu.bus, spec.clone(), flash);
    let shadow_size = spec.shadow_size;
    let cs1_mask = spec.unservable_when_high as u16;
    // Cross-check the lifted shadow vs seabios chunk for ROM_SET_INDEX.
    let chunk_lo = (ROM_SET_INDEX as usize) * shadow_size;
    let chunk = &seabios[chunk_lo..chunk_lo + shadow_size];
    let mut shadow_mismatch = 0usize;
    for i in 0..shadow_size {
        if oracle.shadow()[i] != chunk[i] {
            shadow_mismatch += 1;
        }
    }
    if shadow_mismatch != 0 {
        return Err(format!(
            "rom_set {} (probe): lifted shadow differs from seabios chunk in {} bytes",
            ROM_SET_INDEX, shadow_mismatch
        ));
    }

    // Numerical Recipes LCG (Knuth's MMIX constants) — deterministic,
    // small, no dependencies. We only need 256 16-bit values with the
    // CS1 bit forced high.
    let mut state: u64 = PROBE_CS1_LCG_SEED;
    let lcg_next = |s: &mut u64| -> u16 {
        *s = s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        // High bits have better distribution than low bits in a Knuth LCG.
        ((*s >> 33) & 0xFFFF) as u16
    };

    let mut pass = 0usize;
    let mut wrong = 0usize;
    let mut no_stable = 0usize;
    let mut not_driven = 0usize;
    let mut latency_oor = 0usize;
    let mut surprises: Vec<(u16, &'static str, Option<u8>)> = Vec::new();

    for _ in 0..PROBE_CS1_NUM_CASES {
        let pin_state = lcg_next(&mut state) | cs1_mask;
        let case = onerom_serving_oracle::Case::from_raw("probe-cs1", pin_state as u64);
        let result = oracle.run_case(&mut emu, case);
        match result.verdict {
            CpuVerdict::Pass => {
                pass += 1;
                surprises.push((pin_state, "Pass", result.observed_byte));
            }
            CpuVerdict::WrongByte { observed, .. } => {
                wrong += 1;
                surprises.push((pin_state, "WrongByte", Some(observed)));
            }
            CpuVerdict::NoStableByte => {
                no_stable += 1;
                surprises.push((pin_state, "NoStableByte", None));
            }
            CpuVerdict::DataPinsNotDriven => not_driven += 1,
            CpuVerdict::LatencyOutOfEnvelope { .. } => {
                latency_oor += 1;
                surprises.push((pin_state, "LatencyOutOfEnvelope", result.observed_byte));
            }
        }
    }

    println!(
        "  verdict distribution over {} CS1=high cases:",
        PROBE_CS1_NUM_CASES
    );
    println!("    DataPinsNotDriven    = {}", not_driven);
    println!("    Pass                 = {}", pass);
    println!("    WrongByte            = {}", wrong);
    println!("    NoStableByte         = {}", no_stable);
    println!("    LatencyOutOfEnvelope = {}", latency_oor);

    if not_driven == PROBE_CS1_NUM_CASES {
        println!(
            "  verdict: ALL {} cases reported DataPinsNotDriven — the firmware-tristates-at-CS1=high claim is strengthened.",
            PROBE_CS1_NUM_CASES
        );
        Ok(())
    } else {
        // Surface up to the first 8 surprises so the human sees what diverged.
        println!(
            "  SURPRISE: {} of {} cases produced a non-DataPinsNotDriven verdict.",
            PROBE_CS1_NUM_CASES - not_driven,
            PROBE_CS1_NUM_CASES
        );
        for (i, (p, v, b)) in surprises.iter().take(8).enumerate() {
            println!(
                "    [{}] pin_state=0x{:04X}  verdict={}  observed_byte={:?}",
                i, p, v, b
            );
        }
        Err(format!(
            "expected all {} cases to report DataPinsNotDriven; got {} non-DataPinsNotDriven",
            PROBE_CS1_NUM_CASES,
            PROBE_CS1_NUM_CASES - not_driven
        ))
    }
}

fn main() -> ExitCode {
    picoem_harness::harness_tracing_init();

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
    let mode_str = if cli.smoke {
        "smoke"
    } else if cli.probe_cs1_thorough {
        "probe-cs1-thorough"
    } else {
        "full"
    };
    println!("mode:    {}", mode_str);
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
        eprintln!(
            "seabios image must be {} bytes; got {}",
            SEABIOS_SIZE,
            seabios.len()
        );
        return ExitCode::from(3);
    }

    let spec = match FixtureSpec::from_flash(&flash) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("failed to parse fixture spec: {}", e);
            return ExitCode::from(3);
        }
    };
    println!("fixture: {} ({}-pin)", spec.label, spec.chip_pins);

    // Guard: this binary is fire-24-a-only (per HLD §4.5). Two latent
    // failure modes break silently if pointed at a fire-32-a fixture:
    //  - `cs1_mask = spec.unservable_when_high as u16` zeroes out for
    //    fire-32-a (`(1u64 << 16) as u16 == 0`), so every CS1=high case
    //    in the sweep would slip through unmasked.
    //  - The full sweep upper bound `pin_hi = 0x1_0000` covers only 16
    //    of fire-32-a's 19 address pins — 7/8ths of the 32-pin address
    //    space would never be tested.
    // The fire-32-a path lands in a separate Stage 3 binary;
    // hard-fail here so an accidental fixture swap is loud.
    if spec.chip_pins != 24 {
        eprintln!(
            "ERROR: seabios_fixture_byte_correct is fire-24-a-only (chip_pins=24); \
             got chip_pins={} from {}. Use seabios32_fixture_byte_correct for fire-32-a fixtures.",
            spec.chip_pins,
            cli.fixture.display(),
        );
        return ExitCode::from(2);
    }
    // Tight invariant for the `as u16` cast: fire-24-a's CS1 lives at
    // GPIO13, so the mask fits in u16. Other 24-pin layouts that move
    // CS1 to GPIO ≥ 16 would silently zero the mask under the same
    // narrowing — assert here so we catch it loud.
    debug_assert_eq!(
        spec.unservable_when_high,
        1u64 << 13,
        "fire-24-a CS1 expected at GPIO13"
    );

    if cli.probe_cs1_thorough {
        return match run_probe_cs1_thorough(&flash, &spec, &seabios) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("--probe-cs1-thorough failed: {}", e);
                ExitCode::FAILURE
            }
        };
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
        match run_set(&flash, &spec, &seabios, k, 0, pin_hi) {
            Ok(t) => {
                grand.pass += t.pass;
                grand.discriminating_pass += t.discriminating_pass;
                // unique_expected_bytes is per-set; do not sum into grand
                // (a sum across sets is not meaningful — see per-set lines).
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
    println!(
        "  pass                  = {}  (= {} KiB byte-correct, but see discriminating count below)",
        grand.pass,
        grand.pass / 1024
    );
    println!(
        "  discriminating_pass   = {}     (pass count where the expected byte differed from the set's modal byte; trivial passes from all-zero chunks excluded)",
        grand.discriminating_pass
    );
    println!("  wrong                 = {}", grand.wrong);
    println!("  no_stable             = {}", grand.no_stable);
    println!("  not_driven            = {}", grand.not_driven);
    println!("  latency_oor           = {}", grand.latency_oor);
    println!(
        "  unservable_cs1        = {}  (firmware tristates D0..D7 at CS1=high)",
        grand.unservable_cs1
    );
    println!(
        "fixture content   = {} bytes ({} KiB) of seabios laid into 4 ROM-set shadows",
        SEABIOS_SIZE,
        SEABIOS_SIZE / 1024
    );
    println!(
        "serve coverage    = {} bytes ({} KiB) reachable at CS1=low; the upper {} KiB written into CS1=high shadow positions is unservable",
        grand.pass,
        grand.pass / 1024,
        grand.unservable_cs1 / 1024
    );
    let _ = total_cases; // retained for readability of the running totals above
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

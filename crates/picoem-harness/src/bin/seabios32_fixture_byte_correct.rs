//! Validates that the OneROM PIO-served bytes match the SeaBIOS image for
//! every address in the 19-bit fire-32-a 27C-series sweep.
//!
//! This is the fire-32-a sibling of `seabios_fixture_byte_correct`: where
//! that binary drives `CpuServingOracle` against the 24-pin SeaBIOS CPU-
//! serve fixture, this one drives the PIO `ServingOracle` against the
//! 32-pin SeaBIOS PIO-serve fixture (`onerom-fire-32-a-rp2350-seabios.bin`).
//! See `wrk_docs/2026.05.04 - HLD - OneROM Serving Oracle Fixture
//! Generalization.md` §4.5 / §4.6 for why we fork rather than unify the
//! two.
//!
//! ## Sweep shape
//!
//! Fire-32-a's socket metadata names GPIO16 as `cs2`, but for
//! 27C010/27C020/27C040 the OneROM runtime gates reads with the separate
//! active-low `/CE` and `/OE` lines. GPIO16 remains A16. The full run
//! therefore exercises all 2^19 socket address patterns; unused high
//! socket bits collapse through the SDRR-baked permutation table, so the
//! expected byte is `seabios[(decoded_addr) % seabios.len()]`.
//!
//! Per HLD §7.5, byte-correct + LatencyOOE counts as PASS — the
//! envelope is empirical and per-fixture; an out-of-window served byte
//! that nevertheless matches is a pipeline-model regression, not a
//! byte-correctness failure.
//!
//! ## CLI
//!
//! ```text
//! cargo run --release -p picoem-harness --bin seabios32_fixture_byte_correct
//! cargo run --release -p picoem-harness --bin seabios32_fixture_byte_correct -- --smoke
//! cargo run --release -p picoem-harness --bin seabios32_fixture_byte_correct -- --stride 16
//! cargo run --release -p picoem-harness --bin seabios32_fixture_byte_correct -- \
//!     --fixture <path> --seabios <path>
//! ```
//!
//! - `--fixture <path>`: defaults to the bundled fire-32-a SeaBIOS
//!   PIO-serve fixture.
//! - `--seabios <path>`: defaults to `seabios-256k.bin` co-located with
//!   the fire-24-a SeaBIOS path (HLD §7.7).
//! - `--smoke`: cap servable cases at 4096 — fast spot check (HLD §7.4).
//! - `--stride <N>`: step the address sweep by N (HLD §7.4 — for the
//!   dev loop on the full sweep).
//! - `--continue-on-fail`: don't bail on first FAIL (default: bail).
//!
//! # Status (2026-05-04)
//!
//! Stage 4C originally treated 32-pin `cs2` as an A16-aliased deselect.
//! That was too pessimistic for 27C-series EPROMs. The current parser uses
//! the embedded ROM type and treats 27C010/27C020/27C040 as `/CE`/`/OE`
//! gated fixtures with full source-image coverage.

use std::io::Write as _;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use picoem_harness::onerom_fixture::FixtureSpec;
use picoem_harness::onerom_serving_oracle::{Case, CaseResult, ServingOracle, Verdict};
use picoem_harness::onerom_sync;
use rp2350_emu::{Config, Emulator, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const DEFAULT_FIXTURE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/onerom-fire-32-a-rp2350-seabios.bin"
);
const DEFAULT_SEABIOS: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/sources/seabios-256k.bin"
);

const RUNTIME_INFO_BASE: u32 = 0x2008_0000;
const RUNTIME_PIOROM_CONFIG_OFFSET: u32 = 60;

/// Servable-case cap when `--smoke` is set (HLD §7.4).
const SMOKE_SERVABLE_CAP: u64 = 4096;

/// Cycle cap for boot-to-sync. The fire-24-a PIO oracle's binary uses
/// 10M; the fire-32-a fixture is the same firmware family, so the same
/// budget applies. Sync normally arrives well under 1M.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// Print a progress line every N servable cases run.
const PROGRESS_INTERVAL: u64 = 16384;

fn should_process_next_servable_case(servable_processed: u64, smoke_cap: Option<u64>) -> bool {
    match smoke_cap {
        Some(cap) => servable_processed < cap,
        None => true,
    }
}

struct Cli {
    fixture: PathBuf,
    seabios: PathBuf,
    smoke: bool,
    stride: u64,
    continue_on_fail: bool,
}

fn parse_cli() -> Result<Cli, String> {
    let mut fixture = PathBuf::from(DEFAULT_FIXTURE);
    let mut seabios = PathBuf::from(DEFAULT_SEABIOS);
    let mut smoke = false;
    let mut stride: u64 = 1;
    let mut continue_on_fail = false;

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--fixture" => fixture = PathBuf::from(args.next().ok_or("--fixture needs a value")?),
            "--seabios" => seabios = PathBuf::from(args.next().ok_or("--seabios needs a value")?),
            "--smoke" => smoke = true,
            "--stride" => {
                let v = args.next().ok_or("--stride needs a value")?;
                stride = v.parse::<u64>().map_err(|e| format!("--stride: {e}"))?;
                if stride == 0 {
                    return Err("--stride must be >= 1".into());
                }
            }
            "--continue-on-fail" => continue_on_fail = true,
            "-h" | "--help" => {
                eprintln!(
                    "usage: seabios32_fixture_byte_correct [--fixture <path>] [--seabios <path>] \
                     [--smoke] [--stride <N>] [--continue-on-fail]\n\
                     --smoke              cap servable cases at {} for fast spot-check\n\
                     --stride <N>         step the address sweep by N (sub-sample for dev loop)\n\
                     --continue-on-fail   keep going past the first FAIL\n",
                    SMOKE_SERVABLE_CAP,
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
        stride,
        continue_on_fail,
    })
}

/// Boot-sync helper: load bootrom + flash, reset, halt core 1, run the
/// firmware until PIO1+PIO2 sync is reached. Returns the synced
/// `Emulator`. Mirrors the boot-sync block in
/// `onerom_serving_oracle_rp2350` — kept inline rather than extracted
/// because each PIO-serve binary has slightly different setup needs and
/// the boilerplate is below the abstraction threshold (HLD §4.6).
fn boot_to_sync(bootrom: &[u8], flash: &[u8]) -> Result<Emulator, String> {
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .map_err(|e| format!("emulator build failed: {e:?}"))?;
    emu.load_bootrom(bootrom);
    emu.load_flash(flash);
    emu.reset();

    // Bootrom bypass — OneROM's `.bin` is raw flash, not an IMAGE_DEF
    // block, so jump straight to the reset vector.
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);

    // OneROM PIO serve loop is single-core.
    emu.core_mut(1).halt();

    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            return Err(format!("cycle counter stalled at {before}"));
        }
        if onerom_sync::is_synced(&mut emu.bus) {
            return Ok(emu);
        }
    }
    Err(format!(
        "boot did not reach PIO1+PIO2 SM-enable sync within {} cycles",
        BOOT_CYCLE_CAP
    ))
}

/// Convert a 19-bit address index into the canonical `Case` for the
/// sweep. The label is a `&'static str` — using a single shared label
/// keeps allocation off the hot path (262K cases × per-case allocs is
/// real time on a tight CPU).
fn make_case(addr_idx: u32, spec: &FixtureSpec) -> Case {
    Case::from_addr("sweep", addr_idx, spec)
}

fn expected_seabios_offset(addr_idx: u64, seabios_len: usize) -> usize {
    debug_assert!(seabios_len > 0);
    (addr_idx as usize) % seabios_len
}

fn validate_seabios_len(len: usize) -> Result<(), String> {
    match len {
        0x1_0000 | 0x2_0000 | 0x4_0000 => Ok(()),
        _ => Err(format!(
            "seabios image must be 64 KiB, 128 KiB, or 256 KiB; got {len} bytes"
        )),
    }
}

#[derive(Debug)]
struct RuntimePioromConfig {
    invert_cs: [u8; 4],
    num_cs_pins: u8,
    cs_base_pin: u8,
    data_base_pin: u8,
    num_data_pins: u8,
    addr_base_pin: u8,
    num_addr_pins: u8,
    addr_read_irq: u8,
    rom_table_addr: u32,
    contiguous_cs_pins: u8,
    multi_rom_mode: u8,
    cs_pin_2nd_match: u32,
}

fn runtime_piorom_config(emu: &mut Emulator) -> Option<RuntimePioromConfig> {
    let magic = [
        emu.bus.read8(RUNTIME_INFO_BASE, 0),
        emu.bus.read8(RUNTIME_INFO_BASE + 1, 0),
        emu.bus.read8(RUNTIME_INFO_BASE + 2, 0),
        emu.bus.read8(RUNTIME_INFO_BASE + 3, 0),
    ];
    if magic != *b"sdrr" {
        return None;
    }
    let base = RUNTIME_INFO_BASE.checked_add(RUNTIME_PIOROM_CONFIG_OFFSET)?;

    let read8 = |emu: &mut Emulator, off: u32| emu.bus.read8(base + off, 0);
    let read32 = |emu: &mut Emulator, off: u32| emu.bus.read32(base + off, 0);

    Some(RuntimePioromConfig {
        invert_cs: [read8(emu, 0), read8(emu, 1), read8(emu, 2), read8(emu, 3)],
        num_cs_pins: read8(emu, 4),
        cs_base_pin: read8(emu, 5),
        data_base_pin: read8(emu, 6),
        num_data_pins: read8(emu, 7),
        addr_base_pin: read8(emu, 8),
        num_addr_pins: read8(emu, 9),
        addr_read_irq: read8(emu, 10),
        rom_table_addr: read32(emu, 20),
        contiguous_cs_pins: read8(emu, 40),
        multi_rom_mode: read8(emu, 41),
        cs_pin_2nd_match: read32(emu, 44),
    })
}

fn pio_in_base(pinctrl: u32) -> u8 {
    ((pinctrl >> 15) & 0x1f) as u8
}

fn pio_out_base(pinctrl: u32) -> u8 {
    (pinctrl & 0x1f) as u8
}

fn unique_servable_offset_count(spec: &FixtureSpec, seabios_len: usize) -> usize {
    let sweep_size: u64 = 1u64 << spec.addr_pins.len();
    let mut offsets = std::collections::HashSet::new();
    for addr_idx in 0..sweep_size {
        let case = make_case(addr_idx as u32, spec);
        if (case.pin_pattern & spec.unservable_when_high) != 0 {
            continue;
        }
        offsets.insert(expected_seabios_offset(addr_idx, seabios_len));
    }
    offsets.len()
}

/// Decide whether a `CaseResult` counts as PASS for byte-correctness
/// purposes.
///
/// Per HLD §7.5, a stable byte that matches the expected SeaBIOS byte
/// is a PASS regardless of latency-envelope verdict — `LatencyOutOfEnvelope`
/// with the correct byte is still a byte-correctness PASS (the envelope
/// is empirical and per-fixture; the fire-24-a binary handles it the
/// same way).
fn classify(result: &CaseResult, expected: u8) -> ResultKind {
    match result.verdict {
        Verdict::Pass => {
            if result.observed_byte == Some(expected) {
                ResultKind::Pass
            } else {
                ResultKind::Wrong {
                    observed: result.observed_byte,
                }
            }
        }
        Verdict::LatencyOutOfEnvelope { .. } => {
            if result.observed_byte == Some(expected) {
                ResultKind::Pass
            } else {
                ResultKind::Wrong {
                    observed: result.observed_byte,
                }
            }
        }
        Verdict::WrongByte { observed, .. } => ResultKind::Wrong {
            observed: Some(observed),
        },
        Verdict::NoResolve => ResultKind::NoResolve,
        Verdict::NoStableByte => ResultKind::NoStableByte,
        Verdict::ResolvedAddrOutOfRange { addr } => ResultKind::AddrOOR { addr },
    }
}

/// Boil a verdict + expected/observed comparison down to one of four
/// human-meaningful outcomes for the sweep tally.
#[derive(Clone, Copy, Debug)]
enum ResultKind {
    Pass,
    Wrong { observed: Option<u8> },
    NoResolve,
    NoStableByte,
    AddrOOR { addr: u32 },
}

#[derive(Default)]
struct Tally {
    pass: u64,
    wrong: u64,
    no_resolve: u64,
    no_stable: u64,
    addr_oor: u64,
    /// First failure recorded (addr_idx, expected, observed_or_None).
    first_fail: Option<(u32, u8, Option<u8>)>,
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
        "seabios32_fixture_byte_correct: bootrom={} fixture={} seabios={}",
        std::path::Path::new(BOOTROM_PATH).display(),
        cli.fixture.display(),
        cli.seabios.display()
    );

    println!("fixture: {}", cli.fixture.display());
    println!("seabios: {}", cli.seabios.display());
    let mode_str = if cli.smoke {
        format!("smoke (cap={})", SMOKE_SERVABLE_CAP)
    } else if cli.stride > 1 {
        format!("stride={}", cli.stride)
    } else {
        "full".to_string()
    };
    println!("mode:    {}", mode_str);
    let _ = std::io::stdout().flush();

    let bootrom = match std::fs::read(BOOTROM_PATH) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom at {}: {}", BOOTROM_PATH, e);
            return ExitCode::from(2);
        }
    };
    let flash = match std::fs::read(&cli.fixture) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read fixture at {}: {}", cli.fixture.display(), e);
            return ExitCode::from(2);
        }
    };
    let seabios = match std::fs::read(&cli.seabios) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read seabios at {}: {}", cli.seabios.display(), e);
            return ExitCode::from(2);
        }
    };
    if let Err(e) = validate_seabios_len(seabios.len()) {
        eprintln!("{e}");
        return ExitCode::from(3);
    }

    let spec = match FixtureSpec::from_flash(&flash) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("failed to parse fixture spec: {e}");
            return ExitCode::from(3);
        }
    };
    println!("fixture: {} ({}-pin)", spec.label, spec.chip_pins);
    println!("  rom_type:                  {}", spec.rom_type);
    println!("  addr_pins:                 {:?}", spec.addr_pins);
    println!("  data_pins:                 {:?}", spec.data_pins);
    println!(
        "  cs1:                       GPIO{}  (socket-generic slot on 32-pin)",
        spec.cs1
    );
    println!(
        "  asserted_low_during_read:  {:?}",
        spec.asserted_low_during_read
    );
    println!(
        "  deasserted_high_during_read: {:?}",
        spec.deasserted_high_during_read
    );
    println!(
        "  unservable_when_high:      {:#018x}",
        spec.unservable_when_high
    );

    // Hard guard: this binary is fire-32-a-only. The 24-pin sibling
    // (`seabios_fixture_byte_correct`) drives `CpuServingOracle` and
    // owns CPU-serve fixtures; fire-32-a is PIO-served. See HLD §4.6.
    if spec.chip_pins != 32 {
        eprintln!(
            "ERROR: seabios32 binary is fire-32-a-only (chip_pins=32); \
             got chip_pins={} from {}. Use seabios_fixture_byte_correct \
             for fire-24-a fixtures.",
            spec.chip_pins,
            cli.fixture.display(),
        );
        return ExitCode::from(2);
    }
    if spec.shadow_size != 524_288 {
        eprintln!(
            "ERROR: fire-32-a expects 512 KiB SDRR shadow; got {}",
            spec.shadow_size
        );
        return ExitCode::from(3);
    }

    // Boot to PIO sync. This drives the firmware all the way to the
    // PIO1+PIO2 SM-enable handshake; from here `ServingOracle::run_case`
    // can drive cases through the serving pipeline.
    let t_boot = Instant::now();
    let mut emu = match boot_to_sync(&bootrom, &flash) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("FAILURE — {e}");
            return ExitCode::FAILURE;
        }
    };
    println!(
        "synced at cycle {} ({} ms)",
        emu.cycles(),
        t_boot.elapsed().as_millis()
    );
    if let Some(cfg) = runtime_piorom_config(&mut emu) {
        println!(
            "runtime piorom_config: cs_base={} num_cs={} invert={:?} data_base={} num_data={} \
             addr_base={} num_addr={} addr_irq={} contiguous_cs={} multi_rom={} cs_second=0x{:08x} \
             rom_table=0x{:08x}",
            cfg.cs_base_pin,
            cfg.num_cs_pins,
            cfg.invert_cs,
            cfg.data_base_pin,
            cfg.num_data_pins,
            cfg.addr_base_pin,
            cfg.num_addr_pins,
            cfg.addr_read_irq,
            cfg.contiguous_cs_pins,
            cfg.multi_rom_mode,
            cfg.cs_pin_2nd_match,
            cfg.rom_table_addr
        );
    }
    let cycle = emu.cycles();
    let sync_report = onerom_sync::capture_snapshot(&mut emu.bus, cycle);
    let pio1_fstat = emu.bus.read32(onerom_sync::PIO_BASES[1] + 0x004, 0);
    let pio2_fstat = emu.bus.read32(onerom_sync::PIO_BASES[2] + 0x004, 0);
    println!(
        "PIO1 CTRL=0x{:08x} SM0 CLKDIV=0x{:08x} EXECCTRL=0x{:08x} SHIFTCTRL=0x{:08x} \
         PINCTRL=0x{:08x} in_base={} out_base={} gpio_base={} ADDR={} LAST=0x{:04x} FSTAT=0x{:08x}",
        sync_report.pio1.ctrl,
        sync_report.pio1.sms[0].clkdiv,
        sync_report.pio1.sms[0].execctrl,
        sync_report.pio1.sms[0].shiftctrl,
        sync_report.pio1.sms[0].pinctrl,
        pio_in_base(sync_report.pio1.sms[0].pinctrl),
        pio_out_base(sync_report.pio1.sms[0].pinctrl),
        emu.bus.pio[1].gpio_base(),
        sync_report.pio1.sms[0].addr,
        sync_report.pio1.sms[0].last_insn,
        pio1_fstat
    );
    println!(
        "PIO2 CTRL=0x{:08x} SM0 CLKDIV=0x{:08x} EXECCTRL=0x{:08x} SHIFTCTRL=0x{:08x} \
         PINCTRL=0x{:08x} in_base={} out_base={} gpio_base={} ADDR={} LAST=0x{:04x} FSTAT=0x{:08x}",
        sync_report.pio2.ctrl,
        sync_report.pio2.sms[0].clkdiv,
        sync_report.pio2.sms[0].execctrl,
        sync_report.pio2.sms[0].shiftctrl,
        sync_report.pio2.sms[0].pinctrl,
        pio_in_base(sync_report.pio2.sms[0].pinctrl),
        pio_out_base(sync_report.pio2.sms[0].pinctrl),
        emu.bus.pio[2].gpio_base(),
        sync_report.pio2.sms[0].addr,
        sync_report.pio2.sms[0].last_insn,
        pio2_fstat
    );

    let mut oracle = ServingOracle::new_at_sync(&mut emu.bus, spec.clone(), &flash);
    oracle.populate_sram_from_shadow(&mut emu.bus);

    let unique_count: usize = oracle
        .shadow()
        .iter()
        .copied()
        .collect::<std::collections::HashSet<u8>>()
        .len();
    println!(
        "shadow @ +0x{:05X}: {} unique bytes",
        spec.shadow_size, unique_count
    );
    if unique_count == 1 {
        eprintln!("WARNING: shadow is uniform — oracle cannot distinguish between addresses.");
    }

    // Sweep parameters.
    let n_addr = spec.addr_pins.len() as u32;
    if n_addr != 19 {
        eprintln!(
            "ERROR: fire-32-a expects 19 address pins; got {} (spec.label = {})",
            n_addr, spec.label,
        );
        return ExitCode::from(3);
    }
    let sweep_size: u64 = 1u64 << n_addr;
    let smoke_cap: Option<u64> = cli.smoke.then_some(SMOKE_SERVABLE_CAP);
    let unique_servable_offsets = unique_servable_offset_count(&spec, seabios.len());

    println!(
        "sweep: {} addresses (stride={}, smoke_cap={:?})",
        sweep_size, cli.stride, smoke_cap
    );
    println!(
        "acceptance scope: servable pin-pattern byte checks; full stride=1 covers \
         {} unique SeaBIOS offsets out of {} bytes",
        unique_servable_offsets,
        seabios.len()
    );
    let _ = std::io::stdout().flush();

    let mut tally = Tally::default();
    let mut servable_processed: u64 = 0;
    let mut unservable_skipped: u64 = 0;
    let mut bailed_early = false;
    let t_sweep = Instant::now();

    let mut addr_idx: u64 = 0;
    while addr_idx < sweep_size {
        let case = make_case(addr_idx as u32, &spec);

        // Skip patterns only when the fixture parser identifies a genuine
        // address-aliased deselect gate. 27C010/020/040 fire-32-a fixtures
        // have none (`unservable_when_high == 0`).
        if (case.pin_pattern & spec.unservable_when_high) != 0 {
            unservable_skipped += 1;
            addr_idx += cli.stride;
            continue;
        }

        if !should_process_next_servable_case(servable_processed, smoke_cap) {
            break;
        }
        servable_processed += 1;

        // Expected byte: SeaBIOS at addr_idx modulo the source image. Higher
        // socket-only address bits are SDRR-table duplicates for smaller EPROM
        // content.
        let expected = seabios[expected_seabios_offset(addr_idx, seabios.len())];

        let result = oracle.run_case(&mut emu, case);
        let result_copy = *result;
        match classify(&result_copy, expected) {
            ResultKind::Pass => tally.pass += 1,
            ResultKind::Wrong { observed } => {
                tally.wrong += 1;
                if tally.first_fail.is_none() {
                    tally.first_fail = Some((addr_idx as u32, expected, observed));
                }
                println!(
                    "FAIL addr=0x{:05x} expected=0x{:02x} verdict={:?} observed={:?}",
                    addr_idx, expected, result_copy.verdict, observed,
                );
                if !cli.continue_on_fail {
                    bailed_early = true;
                    break;
                }
            }
            ResultKind::NoResolve => {
                tally.no_resolve += 1;
                if tally.first_fail.is_none() {
                    tally.first_fail = Some((addr_idx as u32, expected, None));
                }
                println!(
                    "FAIL addr=0x{:05x} expected=0x{:02x} verdict=NoResolve",
                    addr_idx, expected,
                );
                if !cli.continue_on_fail {
                    bailed_early = true;
                    break;
                }
            }
            ResultKind::NoStableByte => {
                tally.no_stable += 1;
                if tally.first_fail.is_none() {
                    tally.first_fail = Some((addr_idx as u32, expected, None));
                }
                println!(
                    "FAIL addr=0x{:05x} expected=0x{:02x} verdict=NoStableByte",
                    addr_idx, expected,
                );
                if !cli.continue_on_fail {
                    bailed_early = true;
                    break;
                }
            }
            ResultKind::AddrOOR { addr } => {
                tally.addr_oor += 1;
                if tally.first_fail.is_none() {
                    tally.first_fail = Some((addr_idx as u32, expected, None));
                }
                println!(
                    "FAIL addr=0x{:05x} expected=0x{:02x} verdict=AddrOOR resolved=0x{:08x}",
                    addr_idx, expected, addr,
                );
                if !cli.continue_on_fail {
                    bailed_early = true;
                    break;
                }
            }
        }

        if servable_processed.is_multiple_of(PROGRESS_INTERVAL) {
            let elapsed_ms = t_sweep.elapsed().as_millis();
            println!(
                "  progress: servable={} pass={} wrong={} no_resolve={} no_stable={} addr_oor={} ({} ms)",
                servable_processed,
                tally.pass,
                tally.wrong,
                tally.no_resolve,
                tally.no_stable,
                tally.addr_oor,
                elapsed_ms,
            );
            let _ = std::io::stdout().flush();
        }

        addr_idx += cli.stride;
    }

    let elapsed_ms = t_sweep.elapsed().as_millis();
    println!();
    println!("=== sweep complete ===");
    println!("addresses-in-sweep:    {}", sweep_size);
    println!("servable processed:    {}", servable_processed);
    println!("unservable (skipped):  {}", unservable_skipped);
    println!("PASS:                  {}", tally.pass);
    println!("wrong:                 {}", tally.wrong);
    println!("no_resolve:            {}", tally.no_resolve);
    println!("no_stable:             {}", tally.no_stable);
    println!("addr_oor:              {}", tally.addr_oor);
    println!("elapsed:               {} ms", elapsed_ms);
    if let Some((a, e, o)) = tally.first_fail {
        println!(
            "first fail: addr=0x{:05x} expected=0x{:02x} observed={:?}",
            a, e, o
        );
    }

    let full_stride_one_accounting_error = if !cli.smoke && cli.stride == 1 && !bailed_early {
        if servable_processed + unservable_skipped != sweep_size {
            eprintln!(
                "INTERNAL: full sweep accounting expected servable_processed + \
                 unservable_skipped = {}, got servable_processed={} unservable_skipped={}",
                sweep_size, servable_processed, unservable_skipped
            );
            true
        } else {
            false
        }
    } else {
        false
    };

    let total_failures = tally.wrong + tally.no_resolve + tally.no_stable + tally.addr_oor;
    if bailed_early || total_failures != 0 || full_stride_one_accounting_error {
        ExitCode::FAILURE
    } else if tally.pass != servable_processed {
        // Belt-and-braces: every servable case must be classified as PASS.
        eprintln!(
            "INTERNAL: pass count {} != servable_processed {}",
            tally.pass, servable_processed
        );
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use picoem_harness::onerom_fixture::FixtureSpec;

    #[test]
    fn smoke_cap_allows_exactly_requested_servable_cases() {
        let cap = Some(2);

        assert!(should_process_next_servable_case(0, cap));
        assert!(should_process_next_servable_case(1, cap));
        assert!(!should_process_next_servable_case(2, cap));
        assert!(should_process_next_servable_case(2, None));
    }

    #[test]
    fn seabios_len_accepts_eprom_sized_images_only() {
        assert!(validate_seabios_len(0x1_0000).is_ok());
        assert!(validate_seabios_len(0x2_0000).is_ok());
        assert!(validate_seabios_len(0x4_0000).is_ok());
        assert!(validate_seabios_len(0).is_err());
        assert!(validate_seabios_len(0x3_0000).is_err());
    }

    /// The fire-32-a 27C020 sweep keeps GPIO16 as A16 and uses /CE + /OE
    /// as the active-low read gates, so all 2^19 socket address patterns
    /// are servable.
    #[test]
    fn fire32a_27c020_sweep_has_no_a16_unservable_split() {
        let flash = std::fs::read(DEFAULT_FIXTURE)
            .expect("fire-32-a fixture must be present at the bundled path");
        let spec = FixtureSpec::from_flash(&flash).expect("fire-32-a parse");
        assert_eq!(spec.chip_pins, 32);
        assert_eq!(
            spec.rom_type,
            picoem_harness::onerom_fixture::CHIP_TYPE_27C020
        );
        assert_eq!(spec.addr_pins.len(), 19);
        assert_eq!(spec.unservable_when_high, 0);
        assert_eq!(spec.asserted_low_during_read, vec![15, 14]);
        assert!(spec.deasserted_high_during_read.is_empty());
        assert_eq!(
            spec.addr_pins[16], 16,
            "fire-32-a A16 must map to GPIO16 and remain addressable"
        );

        let sweep_size: u64 = 1u64 << 19;
        let mut servable: u64 = 0;
        let mut unservable: u64 = 0;
        for addr_idx in 0..sweep_size {
            let case = make_case(addr_idx as u32, &spec);
            if (case.pin_pattern & spec.unservable_when_high) != 0 {
                unservable += 1;
            } else {
                servable += 1;
            }
        }
        assert_eq!(servable, sweep_size, "all 2^19 patterns must be servable");
        assert_eq!(unservable, 0, "27C020 must not deselect on A16 high");
        assert_eq!(servable + unservable, sweep_size);
    }

    #[test]
    fn fire32a_full_sweep_expected_offsets_cover_whole_256k_source() {
        let flash = std::fs::read(DEFAULT_FIXTURE)
            .expect("fire-32-a fixture must be present at the bundled path");
        let spec = FixtureSpec::from_flash(&flash).expect("fire-32-a parse");
        assert_eq!(spec.addr_pins[16], 16);

        let seabios_size = 0x4_0000;
        let sweep_size: u64 = 1u64 << spec.addr_pins.len();
        let mut offsets = std::collections::HashSet::new();
        for addr_idx in 0..sweep_size {
            let case = make_case(addr_idx as u32, &spec);
            if (case.pin_pattern & spec.unservable_when_high) != 0 {
                continue;
            }
            offsets.insert(expected_seabios_offset(addr_idx, seabios_size));
        }

        assert_eq!(
            offsets.len(),
            0x4_0000,
            "full fire-32-a sweep covers all 262144 SeaBIOS offsets"
        );
        for offset in 0..seabios_size {
            assert!(
                offsets.contains(&offset),
                "missing SeaBIOS offset 0x{offset:05x}"
            );
        }
    }

    #[test]
    fn fire32a_128k_source_covers_whole_128k_source() {
        let flash = std::fs::read(DEFAULT_FIXTURE)
            .expect("fire-32-a fixture must be present at the bundled path");
        let spec = FixtureSpec::from_flash(&flash).expect("fire-32-a parse");
        assert_eq!(unique_servable_offset_count(&spec, 0x2_0000), 0x2_0000);
    }
}

//! OneROM stress driver — CPU-mode sweep for the 1541 $E000 kernal fixture.
//!
//! Mirrors [`onerom_stress_pio_rp2350`] but targets the CPU-serve variant
//! of the 1541 fixture. Same 2048-case sweep, same silent / first-20-fails /
//! exit-code contract.
//!
//! CPU vs PIO differences:
//! - No glue DMA — the CPU reads SRAM and drives pins directly. We call
//!   [`onerom_serving_oracle_cpu::CpuServingOracle::run_case`] once per
//!   case (no `GlueDma` parameter).
//! - Sync detection uses PC + shadow-readiness tripwire, not PIO CTRLs.
//! - The 1541 fixture's serve loop lives at a **different PC range** than
//!   `test-sdrr-0` — empirically `0x1000_09A4..=0x1000_09B0` per
//!   `verify_1541_cpu` (vs. `0x1000_0926..=0x1000_0930` for test-sdrr-0).
//!   Per HLD §Risk 1, we sync on the local 1541 range here rather than
//!   mutating the shipped [`onerom_serving_oracle_cpu::is_synced_cpu`]
//!   constants.
//!
//! `CpuCaseResult` is converted to [`onerom_serving_oracle::CaseResult`]
//! before feeding into [`onerom_stress::compute_histogram`] +
//! [`onerom_stress::format_report`] — keeps the reporting library
//! single-typed per HLD §Data flow.
//!
//! Hardcoded fixture path + `ROM_SET_INDEX` — change and recompile to
//! target a different fixture or ROM set.
//!
//! **Boot-time ROM-set forcing**. The 1541 CPU-mode fixture bundles four
//! ROM sets. With floating image-select jumpers the firmware decodes
//! `sel_value = 7` (a combination of pad pull-ups and per-pin flip bits)
//! and picks `7 % 4 = 3` — a 27C301 EPROM image with a completely
//! different pin layout (CS on GPIO0, data on GPIO26) from what the
//! shared [`onerom_serving_oracle_cpu::CpuServingOracle`] library
//! drives. Pre-fix runs reported ~140/2048 PASS, and those 140 were
//! spurious: the emulator was watching ROM set 3's idle pins while the
//! stim happened to produce an `expected == 0x00` shadow lookup that
//! the oracle's `ZERO_BYTE_TRUST_TIMEOUT_CPU` fallback trusted.
//!
//! ROM set 0 of this fixture is `1541-e000.901229-06AA.bin` — a 2364
//! mask ROM with `CS1=GPIO13`, identical to `test-sdrr-0-cpu`'s
//! default set and matching the library's hardcoded pin constants. We
//! therefore force the firmware to boot ROM set 0 by driving the
//! image-select GPIOs via
//! [`onerom_serving_oracle_cpu::force_rom_set_index_via_sel_pins`]
//! before the first emulator step, and run the sweep against set 0
//! without touching the library constants.
//!
//! Design: `wrk_docs/2026.04.17 - HLD - OneROM Stress Harness.md`
//! (original); `wrk_docs/2026.04.22 - HLD - OneROM CPU Speed Grade
//! Oracle.md` §Phase 1' for the image_sel forcing helper that unblocks
//! this binary.
//!
//! Usage:
//!   cargo run -p picoem-harness --bin onerom_stress_cpu_rp2350 --release

use std::process::ExitCode;
use std::time::{Duration, Instant};

use picoem_harness::{
    onerom_serving_oracle::{self, Case, CaseResult, Verdict},
    onerom_serving_oracle_cpu::{self, CpuCaseResult, CpuServingOracle, CpuVerdict},
    onerom_stress,
};
use rp2350_emu::{Config, Emulator, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
const FLASH_PATH: &str = "crates/picoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

/// ROM set index parsed from the fixture. `0` = 1541 $E000 kernal
/// (901229-06AA). Change + recompile to sweep a different set; no
/// env-var override by design (mirrors the PIO stress binary).
const ROM_SET_INDEX: u8 = 0;

/// Human-readable label for the report header.
const LABEL: &str = "1541 $E000 kernal (901229-06AA), CPU mode";

/// Boot cycle cap — CPU-mode sync usually arrives in a handful of
/// thousands of cycles; 10M is generous (mirrors the PIO stress binary
/// and the CPU serving oracle binary).
const BOOT_CYCLE_CAP: u64 = 10_000_000;

/// First N failures to inline in the report (HLD §Output format).
const FIRST_FAILS_CAP: usize = 20;

/// CPU serve-loop PC range for the 1541 fixture's **ROM set 0** (2364
/// mask ROM, `1541-e000.901229-06AA.bin`). With ROM set 0 forced at
/// boot via the image_sel helper, the 5-instruction tight loop lives
/// at the same offsets as the `test-sdrr-0` fixture's default set —
/// both are 2364 bakes, so they share the serve-loop code layout.
/// Empirically verified with `_probe_1541_cpu_romset0` (PC histogram
/// 28.57/28.57/14.28/14.28/14.28% = the expected 5-instruction loop
/// distribution).
///
/// Aliases [`onerom_serving_oracle_cpu::CPU_SERVE_LOOP_PC_LO`]/`_HI`
/// today; kept as local constants rather than `use`-imports so that
/// if a future 1541 fixture bake shifts the serve loop, the two can
/// diverge without touching the shared library. The pre-Phase-1' value
/// (`0x1000_09A4..=0x1000_09B0`) corresponded to the 1541 fixture's
/// floating-jumper default (ROM set 3, a 27C301 image with a different
/// serve loop); forcing ROM set 0 brings this back in line with set 0.
const SERVE_LOOP_PC_LO_1541: u32 = 0x1000_0926;
const SERVE_LOOP_PC_HI_1541: u32 = 0x1000_0930;

fn main() -> ExitCode {
    picoem_harness::harness_tracing_init();

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

    // Up-front shadow-lift sanity check (mirrors the PIO stress binary):
    // confirm `ROM_SET_INDEX` parses out of the fixture. If None, the
    // sweep is meaningless.
    if onerom_serving_oracle::lift_shadow_from_flash_pub(&flash, ROM_SET_INDEX).is_none() {
        eprintln!(
            "failed to lift ROM set {} from fixture — wrong index or malformed flash",
            ROM_SET_INDEX
        );
        return ExitCode::from(2);
    }

    // step_quantum=1 for per-cycle observation fidelity.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build()
        .unwrap();
    emu.load_bootrom(&bootrom);
    emu.load_flash(&flash);
    emu.reset();

    // Bootrom bypass — OneROM's flash image is not an IMAGE_DEF block.
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);

    // CPU-serve mode is single-core.
    emu.core_mut(1).halt();

    // Pin the image-select GPIOs so the firmware's `check_sel_pins()`
    // decodes `rom_set_index = ROM_SET_INDEX` instead of its default
    // floating-jumper fallback (which lands on ROM set 3 in this
    // 4-set fixture — a 27C301 image with an incompatible pin layout).
    // Must happen after `emu.reset()` (which clears external stimulus)
    // and before the first run-step below.
    if let Err(e) = onerom_serving_oracle_cpu::force_rom_set_index_via_sel_pins(
        &mut emu,
        &flash,
        ROM_SET_INDEX as u32,
    ) {
        eprintln!("failed to force rom_set_index {}: {}", ROM_SET_INDEX, e);
        return ExitCode::from(2);
    }

    // Two-phase sync (mirrors `onerom_serving_oracle_cpu_rp2350`):
    // phase 1 waits for core 0's PC to enter the 1541-specific serve
    // loop range; phase 2 gates on a shadow-readiness sentinel so we
    // don't declare sync while the firmware's SRAM copy is still
    // running.
    const RUNTIME_INFO_SRAM_OFF: u32 = 0x0008_0000;
    const ROM_SET_INDEX_OFFSET: u32 = 6;

    // Phase 1: PC enters the 1541 serve-loop range.
    let mut phase1_cycle: Option<u64> = None;
    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            eprintln!("cycle counter stalled at {}", before);
            return ExitCode::FAILURE;
        }
        if is_in_serve_loop_1541(&emu) {
            phase1_cycle = Some(after);
            break;
        }
    }
    if phase1_cycle.is_none() {
        eprintln!(
            "FAILURE — boot did not reach 1541 CPU serve-loop PC (0x{:08X}..=0x{:08X}) within {} cycles",
            SERVE_LOOP_PC_LO_1541, SERVE_LOOP_PC_HI_1541, BOOT_CYCLE_CAP
        );
        return ExitCode::FAILURE;
    }

    // Lift shadow via SRAM-reported rom_set_index + pick tripwire
    // sentinel from the flash-lifted shadow.
    let rom_set_index_live = emu
        .bus
        .memory
        .sram_read8(RUNTIME_INFO_SRAM_OFF + ROM_SET_INDEX_OFFSET);
    let sentinel: Option<(u32, u8)> =
        match onerom_serving_oracle::lift_shadow_from_flash_pub(&flash, rom_set_index_live) {
            Some(shadow) => onerom_serving_oracle_cpu::find_shadow_sentinel(&shadow),
            None => None,
        };

    // Phase 2: PC + sentinel sync. If sentinel is None (window
    // all-zero) this degrades to PC-only — same contract as the CPU
    // oracle binary.
    let mut sync_cycle: Option<u64> = if is_in_serve_loop_1541(&emu) && sentinel_ok(&emu, sentinel)
    {
        phase1_cycle
    } else {
        None
    };
    while sync_cycle.is_none() && emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1).expect("Serial run is infallible");
        let after = emu.cycles();
        if after == before {
            eprintln!("cycle counter stalled at {}", before);
            return ExitCode::FAILURE;
        }
        if is_in_serve_loop_1541(&emu) && sentinel_ok(&emu, sentinel) {
            sync_cycle = Some(after);
            break;
        }
    }
    if sync_cycle.is_none() {
        eprintln!(
            "FAILURE — boot did not reach CPU serve-loop sync (PC + sentinel) within {} cycles",
            BOOT_CYCLE_CAP
        );
        return ExitCode::FAILURE;
    }

    // Build the CPU oracle. Unlike PIO, no DMA preload — CPU-mode
    // firmware populates SRAM itself, so we skip any
    // `populate_sram_from_shadow` call (would race the firmware).
    let mut oracle = CpuServingOracle::new_at_sync(&mut emu.bus, &flash);

    // Silent sweep.
    let cases = onerom_stress::generate_sweep_cases();
    // Wall-clock per case: measured around each run_case invocation so
    // the report can show host elapsed alongside the emulated-cycle
    // model latency. Failing cases still contribute to throughput.
    let mut wall_durations: Vec<Duration> = Vec::with_capacity(cases.len());
    for case in &cases {
        let t0 = Instant::now();
        let _ = oracle.run_case(&mut emu, *case);
        wall_durations.push(t0.elapsed());
    }

    // Convert CpuCaseResult → CaseResult for the shared histogram /
    // report formatter. The verdict mapping is straight across; see
    // `cpu_to_pio_result` below.
    let cpu_results = oracle.results();
    let pio_results: Vec<CaseResult> = cpu_results.iter().map(cpu_to_pio_result).collect();

    let sys_clk_hz = emu.bus.sys_clk_hz();
    let hist = onerom_stress::compute_histogram(&pio_results, sys_clk_hz);
    let wall = onerom_stress::compute_wall_clock_stats(&wall_durations);
    let fails: Vec<CaseResult> = pio_results
        .iter()
        .filter(|r| r.verdict != Verdict::Pass)
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

/// Local sync helper: PC in the **1541** serve-loop range. Mirrors the
/// oracle's `is_synced_cpu` PC check but with the 1541-specific range;
/// the oracle's hardcoded `CPU_SERVE_LOOP_PC_LO`/`_HI` targets
/// test-sdrr-0 and changing it would break that fixture.
#[inline]
fn is_in_serve_loop_1541(emu: &Emulator) -> bool {
    let pc = emu.core(0).regs.pc();
    (SERVE_LOOP_PC_LO_1541..=SERVE_LOOP_PC_HI_1541).contains(&pc)
}

/// Local sentinel tripwire: returns `true` iff no sentinel or SRAM byte
/// at the sentinel offset matches. Mirrors the private
/// `shadow_tripwire_ok` helper (which isn't `pub`).
#[inline]
fn sentinel_ok(emu: &Emulator, sentinel: Option<(u32, u8)>) -> bool {
    match sentinel {
        None => true,
        Some((offset, expected)) => emu.bus.memory.sram_read8(offset) == expected,
    }
}

/// Map a [`CpuCaseResult`] to the PIO-typed [`CaseResult`] so we can
/// reuse [`onerom_stress::compute_histogram`] and
/// [`onerom_stress::format_report`] unchanged. CPU-specific verdicts
/// map as follows:
///
/// | CPU verdict                  | PIO verdict                    |
/// |------------------------------|--------------------------------|
/// | `Pass`                        | `Pass`                        |
/// | `WrongByte { e, o }`          | `WrongByte { e, o }`          |
/// | `DataPinsNotDriven`           | `NoResolve`                    |
/// | `NoStableByte`                | `NoStableByte`                 |
/// | `LatencyOutOfEnvelope { c }`  | `LatencyOutOfEnvelope { c }`   |
///
/// `DataPinsNotDriven` collapses to `NoResolve` because both mean "the
/// serve pipeline never produced a stable output at all" — the
/// downstream stress report doesn't distinguish between "PIO never
/// pushed" and "CPU never drove OEN" (they're both "no bytes served").
///
/// `resolved_addr` is left `None` — there is no PIO-style
/// `CH1.READ_ADDR` intermediate in CPU mode (the CPU computes the
/// shadow offset internally).
fn cpu_to_pio_result(cpu: &CpuCaseResult) -> CaseResult {
    let verdict = match cpu.verdict {
        CpuVerdict::Pass => Verdict::Pass,
        CpuVerdict::WrongByte { expected, observed } => Verdict::WrongByte { expected, observed },
        CpuVerdict::DataPinsNotDriven => Verdict::NoResolve,
        CpuVerdict::NoStableByte => Verdict::NoStableByte,
        CpuVerdict::LatencyOutOfEnvelope { cycles } => Verdict::LatencyOutOfEnvelope { cycles },
    };
    CaseResult {
        case: Case::new(cpu.case.label, cpu.case.addr_bits),
        resolved_addr: None,
        expected_byte: cpu.expected_byte,
        observed_byte: cpu.observed_byte,
        latency_cycles: cpu.latency_cycles,
        verdict,
    }
}

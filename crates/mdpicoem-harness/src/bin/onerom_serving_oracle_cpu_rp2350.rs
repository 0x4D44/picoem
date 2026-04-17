//! OneROM CPU-serve oracle — byte-correctness + timing envelope for the
//! on-core (CPU-serve) fixture variant.
//!
//! Mirrors `onerom_serving_oracle_rp2350` but targets the CPU-mode
//! variant of the test-sdrr-0 fixture. PIO1/PIO2 stay inert for the
//! entire run; core 0 sits in a 5-instruction tight serve loop at
//! `0x1000_0926..=0x1000_0930` that reads pin state, looks up the
//! byte from SRAM, and drives it back out via SIO_GPIO_OUT +
//! SIO_GPIO_OE. No glue DMA is required.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin onerom_serving_oracle_cpu_rp2350 --release
//!
//! Optional env-var:
//!   ONEROM_FIXTURE_PATH=<path>  Override the default CPU-mode fixture
//!                               (default:
//!                               `crates/mdpicoem-harness/fixtures/\
//!                                onerom-fire-24-a-rp2350-test-sdrr-0-cpu.bin`).

use std::collections::HashSet;
use std::path::PathBuf;
use std::process::ExitCode;

use mdpicoem_harness::{onerom_serving_oracle, onerom_serving_oracle_cpu};
use mdrp2350::{Config, EmulatorBuilder};

const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
/// Default fixture — the CPU-serve-mode variant. The PIO oracle's
/// `-test-sdrr-0.bin` exercises PIO serving; this file is the same
/// ROM image but with the firmware_overrides block patched to force
/// CPU-mode serving at runtime.
const DEFAULT_FLASH_PATH: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-test-sdrr-0-cpu.bin";

/// Boot cycle budget — same as the PIO oracle. CPU-mode sync (PC
/// entering the serve loop) takes roughly the same order of magnitude
/// as PIO sync; 10M cycles is generous.
const BOOT_CYCLE_CAP: u64 = 10_000_000;

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

    let flash_path = match std::env::var("ONEROM_FIXTURE_PATH") {
        Ok(p) => PathBuf::from(p),
        Err(_) => PathBuf::from(DEFAULT_FLASH_PATH),
    };

    let bootrom = match std::fs::read(BOOTROM_PATH) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read bootrom at {}: {}", BOOTROM_PATH, e);
            return ExitCode::from(2);
        }
    };
    let flash = match std::fs::read(&flash_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read flash at {}: {}", flash_path.display(), e);
            return ExitCode::from(2);
        }
    };

    println!(
        "loaded bootrom ({} bytes) and flash ({} bytes from {})",
        bootrom.len(),
        flash.len(),
        flash_path.display()
    );

    // step_quantum=1 for per-cycle observation fidelity.
    let mut emu = EmulatorBuilder::new(Config::default())
        .step_quantum(1)
        .build();
    emu.load_bootrom(&bootrom);
    emu.load_flash(&flash);
    emu.reset();

    // Bootrom bypass: OneROM's flash image is not an IMAGE_DEF block.
    // Jump straight to the reset vector (same as the PIO oracle).
    let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
    let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
    let initial_pc = initial_pc_raw & !1u32;
    emu.core_mut(0).regs.set_sp(initial_sp);
    emu.core_mut(0).regs.set_pc(initial_pc);
    println!(
        "bypassing bootrom: SP=0x{:08X} PC=0x{:08X}",
        initial_sp, initial_pc
    );

    // CPU-serve mode is single-core.
    emu.core_mut(1).halt();

    // Two-phase sync:
    //   Phase 1: step until core 0's PC first enters the serve-loop
    //            range. At that moment the firmware's init has run far
    //            enough that `sdrr_runtime_info_t.rom_set_index` is
    //            populated in SRAM — but the shadow copy may or may
    //            not yet be complete (the false-sync bug fired here).
    //   Phase 2: lift the shadow from flash using the SRAM-reported
    //            `rom_set_index`, pick a readiness sentinel, and keep
    //            stepping with the full `is_synced_cpu(emu, sentinel)`
    //            check until the sentinel byte lands in SRAM.
    //
    // The `rom_set_index` is read from SRAM at phase-1 boundary (PC
    // first hit) because it's set by the same firmware init that
    // runs before the serve loop — reading it earlier would get
    // pre-init zero. We can't hardcode 0 at startup: the fixture's
    // runtime rom_set_index is not always 0 (e.g. `test-sdrr-0-cpu`
    // reports set-index such that set 0 is all-zero and sets 1..4 are
    // the populated ones).
    const RUNTIME_INFO_SRAM_OFF: u32 = 0x0008_0000;
    const ROM_SET_INDEX_OFFSET: u32 = 6;

    // Phase 1: bare-PC sync only.
    let mut phase1_cycle: Option<u64> = None;
    while emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1);
        let after = emu.cycles();
        if after == before {
            eprintln!("cycle counter stalled at {}", before);
            return ExitCode::FAILURE;
        }
        if onerom_serving_oracle_cpu::is_synced_cpu(&emu, None) {
            phase1_cycle = Some(after);
            break;
        }
    }
    let phase1_at = match phase1_cycle {
        Some(c) => c,
        None => {
            eprintln!(
                "FAILURE — boot did not reach CPU serve-loop PC within {} cycles",
                BOOT_CYCLE_CAP
            );
            return ExitCode::FAILURE;
        }
    };
    println!(
        "phase-1 PC hit at cycle {} (PC=0x{:08X}) — lifting shadow + sentinel",
        phase1_at,
        emu.core(0).regs.pc(),
    );

    // Lift shadow via the SRAM-reported rom_set_index.
    let rom_set_index = emu
        .bus
        .memory
        .sram_read8(RUNTIME_INFO_SRAM_OFF + ROM_SET_INDEX_OFFSET);
    let sentinel: Option<(u32, u8)> = match onerom_serving_oracle::lift_shadow_from_flash_pub(
        &flash,
        rom_set_index,
    ) {
        Some(shadow) => {
            let s = onerom_serving_oracle_cpu::find_shadow_sentinel(&shadow);
            match s {
                Some((off, val)) => {
                    println!(
                        "sync tripwire: SRAM[0x{:04X}] == 0x{:02X} (first non-zero byte in shadow[..{}], rom_set_index={})",
                        off,
                        val,
                        onerom_serving_oracle_cpu::SENTINEL_SCAN_WINDOW,
                        rom_set_index,
                    );
                }
                None => {
                    println!(
                        "WARNING: shadow[..{}] is all zero for rom_set_index={} — sync falls back to phase-1 PC-only hit",
                        onerom_serving_oracle_cpu::SENTINEL_SCAN_WINDOW,
                        rom_set_index,
                    );
                }
            }
            s
        }
        None => {
            println!(
                "WARNING: failed to lift shadow at phase-1 (rom_set_index={}) — sync falls back to phase-1 PC-only hit",
                rom_set_index
            );
            None
        }
    };

    // Phase 2: PC + sentinel sync. If the sentinel is already
    // satisfied at the phase-1 boundary, the loop falls straight
    // through on the first check.
    let mut sync_cycle: Option<u64> = if onerom_serving_oracle_cpu::is_synced_cpu(&emu, sentinel) {
        Some(phase1_at)
    } else {
        None
    };
    while sync_cycle.is_none() && emu.cycles() < BOOT_CYCLE_CAP {
        let before = emu.cycles();
        emu.run(1);
        let after = emu.cycles();
        if after == before {
            eprintln!("cycle counter stalled at {}", before);
            return ExitCode::FAILURE;
        }
        if onerom_serving_oracle_cpu::is_synced_cpu(&emu, sentinel) {
            sync_cycle = Some(after);
            break;
        }
    }

    let sync_at = match sync_cycle {
        Some(c) => c,
        None => {
            eprintln!(
                "FAILURE — boot did not reach CPU serve-loop sync within {} cycles",
                BOOT_CYCLE_CAP
            );
            return ExitCode::FAILURE;
        }
    };

    let sync_pc = emu.core(0).regs.pc();
    println!(
        "sync reached at cycle {} (PC=0x{:08X}, in serve-loop range 0x{:08X}..=0x{:08X})",
        sync_at,
        sync_pc,
        onerom_serving_oracle_cpu::CPU_SERVE_LOOP_PC_LO,
        onerom_serving_oracle_cpu::CPU_SERVE_LOOP_PC_HI,
    );

    // Diagnostic: confirm PIO CTRLs are 0 (CPU-mode shouldn't touch PIO).
    let pio1_ctrl = emu.bus.read32(0x5030_0000);
    let pio2_ctrl = emu.bus.read32(0x5040_0000);
    println!(
        "  PIO1.CTRL = 0x{:08X}, PIO2.CTRL = 0x{:08X} (both 0 expected for CPU mode)",
        pio1_ctrl, pio2_ctrl
    );
    if pio1_ctrl != 0 || pio2_ctrl != 0 {
        println!("WARNING: PIO CTRLs non-zero — is this really a CPU-mode fixture?");
    }

    // Build the oracle. Shadow is lifted from flash (canonical ground
    // truth — mirrors the PIO oracle's approach).
    let mut oracle =
        onerom_serving_oracle_cpu::CpuServingOracle::new_at_sync(&mut emu.bus, &flash);

    // Shadow-integrity tripwire (same as the PIO oracle).
    let unique: HashSet<u8> = oracle.shadow().iter().copied().collect();
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
    }

    // In CPU mode the firmware populates SRAM itself via a CPU copy
    // (no DMA preload), so SRAM at [SHADOW_BASE] is already mirrored
    // by the time we reach sync. Skip `populate_sram_from_shadow`
    // here — if we wrote over the CPU's live shadow we'd risk a TOCTOU
    // between our writes and a still-running copy routine. The flash-
    // lifted shadow is used only as the verdict-comparison reference.

    // Per-case sweep.
    let total = onerom_serving_oracle_cpu::CPU_DEFAULT_CASES.len();
    for (idx, case) in onerom_serving_oracle_cpu::CPU_DEFAULT_CASES.iter().enumerate() {
        let result = oracle.run_case(&mut emu, *case);
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
            "[{:>2}/{:>2}] {:<16} addr=0x{:04X} verdict={:<18} expected={} observed={} cycles={}",
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

    // Full report.
    println!();
    let sys_clk_hz = emu.bus.sys_clk_hz();
    print!("{}", oracle.format_report(sys_clk_hz));

    // Exit code: pass iff every case is Pass.
    let results = oracle.results();
    let pass_count = results
        .iter()
        .filter(|r| matches!(r.verdict, onerom_serving_oracle_cpu::CpuVerdict::Pass))
        .count();

    if pass_count == results.len() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

fn format_verdict_short(v: &onerom_serving_oracle_cpu::CpuVerdict) -> String {
    match v {
        onerom_serving_oracle_cpu::CpuVerdict::Pass => "Pass".to_string(),
        onerom_serving_oracle_cpu::CpuVerdict::WrongByte { .. } => "WrongByte".to_string(),
        onerom_serving_oracle_cpu::CpuVerdict::DataPinsNotDriven => {
            "DataPinsNotDriven".to_string()
        }
        onerom_serving_oracle_cpu::CpuVerdict::NoStableByte => "NoStableByte".to_string(),
        onerom_serving_oracle_cpu::CpuVerdict::LatencyOutOfEnvelope { .. } => {
            "LatencyOOE".to_string()
        }
    }
}

//! MMIO trace runner — rp2040_emu (Cortex-M0+) bus access logger.
//!
//! Phase 0 sub-task 0.C per `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral
//! Coverage V7.md` §4.3 / §4.4: load firmware, enable `Bus::mmio_trace_enabled`,
//! run for a caller-supplied cycle budget, and emit one line per bus
//! access to stdout in the format
//!
//!     TRACE <R/W> <size> 0x<addr> val=0x<val> core=<N> pc=0x<pc>
//!
//! The trace's **tail** — the last registers firmware reads before it
//! spins or faults — identifies the next peripheral gap to close.
//! Phase 0 exit criterion 4: `--cycles 1_000_000 hello_timer.elf` must
//! emit ≥500 peripheral MMIO entries.
//!
//! CLI:
//!
//!     mmio_trace_rp2040 --cycles <N> <firmware>
//!
//! Firmware is loaded as a flash image (region `0x1000_0000`). If a
//! sibling `roms/rp2040/bootrom-rp2040-b2.bin` exists we also load the
//! bootrom so SDK-style firmware reaches its reset handler through a
//! direct-boot handoff (mirrors `picogus_diff_rp2040`). ELF loading is
//! deferred — the spec accepts `.bin` for the Phase 0 exit criterion and
//! the Cargo workspace does not yet depend on an ELF crate; we print a
//! clear error if an ELF is supplied.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use rp2040_emu::{Config, EmulatorBuilder};

/// Drop guard that flushes `stdout` when it goes out of scope. Guarantees
/// that any buffered trace lines reach the terminal / redirected file even
/// if `run()` panics mid-loop — without this, a panic inside `emu.run()`
/// could discard the tail of the trace. Pairs with `println!` in
/// `Bus::emit_mmio_trace` which writes through line-buffered stdout.
struct StdoutFlushGuard;

impl Drop for StdoutFlushGuard {
    fn drop(&mut self) {
        let _ = std::io::stdout().flush();
    }
}

/// Default bootrom location searched when a flash image is supplied and
/// no explicit `--bootrom` is given. Matches `picogus_diff_rp2040`.
const DEFAULT_BOOTROM_PATH: &str = "roms/rp2040/bootrom-rp2040-b2.bin";

/// SDK convention: vector table sits at offset 0x100 within flash (boot2
/// occupies the first 256 bytes).
const SDK_VTOR_FLASH_OFFSET: u32 = 0x100;

/// System clock for the pacer math. Matches `picogus_diff_rp2040` — any
/// value is fine; we don't wall-clock pace here, but the clock tree
/// seeds PLL / divider calculations consistently with that binary.
const DEFAULT_SYS_CLK_HZ: u32 = 125_000_000;

struct Args {
    cycles: u64,
    firmware: PathBuf,
    bootrom: Option<PathBuf>,
}

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut cycles: Option<u64> = None;
    let mut firmware: Option<PathBuf> = None;
    let mut bootrom: Option<PathBuf> = None;
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--cycles" => {
                i += 1;
                if i >= argv.len() {
                    return Err("--cycles requires a positive integer".into());
                }
                cycles = Some(
                    argv[i]
                        .parse::<u64>()
                        .map_err(|e| format!("invalid --cycles '{}': {e}", argv[i]))?,
                );
            }
            "--bootrom" => {
                i += 1;
                if i >= argv.len() {
                    return Err("--bootrom requires a path".into());
                }
                bootrom = Some(PathBuf::from(&argv[i]));
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other if other.starts_with("--") => {
                return Err(format!("unknown flag '{other}'"));
            }
            _ => {
                if firmware.is_some() {
                    return Err(format!(
                        "unexpected positional argument '{}': firmware path already set",
                        argv[i]
                    ));
                }
                firmware = Some(PathBuf::from(&argv[i]));
            }
        }
        i += 1;
    }

    let cycles = cycles.ok_or_else(|| "missing required --cycles <N>".to_string())?;
    let firmware = firmware.ok_or_else(|| "missing required firmware path".to_string())?;
    Ok(Args {
        cycles,
        firmware,
        bootrom,
    })
}

fn print_usage() {
    eprintln!(
        "Usage:\n  \
         mmio_trace_rp2040 --cycles <N> <firmware.bin>\n                   \
         [--bootrom <path>]\n\
         \n\
         --cycles    Required. Number of virtual cycles to run the emulator.\n\
         <firmware>  Required. 2 MB XIP flash image (.bin). ELF not yet\n              \
                     supported — rebuild as .bin with llvm-objcopy.\n\
         --bootrom   Optional 16 KB RP2040 bootrom image. Default-searches\n              \
                     `roms/rp2040/bootrom-rp2040-b2.bin` when the file exists.\n\
         \n\
         The trace goes straight to stdout (one line per access). Redirect\n\
         to a file to capture it; pipe to `head`/`tail` to inspect head or\n\
         tail of the run."
    );
}

fn resolve_bootrom_path(explicit: Option<&Path>) -> Option<PathBuf> {
    if let Some(p) = explicit {
        return Some(p.to_path_buf());
    }
    let default = PathBuf::from(DEFAULT_BOOTROM_PATH);
    if default.is_file() {
        Some(default)
    } else {
        None
    }
}

fn load_firmware(path: &Path) -> Result<Vec<u8>, String> {
    // ELF check: ELF magic is `0x7F 'E' 'L' 'F'`. We only handle raw
    // flash images for Phase 0; an ELF file needs `llvm-objcopy -O binary`
    // first. Be explicit so the user doesn't silently get garbage.
    let bytes = std::fs::read(path).map_err(|e| format!("reading {}: {e}", path.display()))?;
    if bytes.len() >= 4 && bytes[0..4] == [0x7F, b'E', b'L', b'F'] {
        return Err(format!(
            "{} is an ELF file — convert with `llvm-objcopy -O binary in.elf out.bin`",
            path.display()
        ));
    }
    Ok(bytes)
}

fn main() -> ExitCode {
    picoem_harness::harness_tracing_init();

    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("fatal: {e}");
            print_usage();
            ExitCode::from(2)
        }
    }
}

fn run() -> Result<(), String> {
    // Ensure stdout is flushed on any exit path — including a panic inside
    // `emu.run()` — so trailing trace lines are never truncated.
    let _flush_guard = StdoutFlushGuard;

    let args = parse_args()?;
    let flash_bytes = load_firmware(&args.firmware)?;
    eprintln!(
        "mmio_trace_rp2040: loaded {} bytes from {}",
        flash_bytes.len(),
        args.firmware.display()
    );

    // step_quantum(1) — per-cycle precision so `run(cycles)` overshoots
    // by at most one cycle. Matches the intent of the --cycles budget.
    let mut emu = EmulatorBuilder::new(Config {
        sys_clk_hz: DEFAULT_SYS_CLK_HZ,
    })
    .step_quantum(1)
    .flash(flash_bytes)
    .build()
    .expect("Serial build is infallible");

    // Load a bootrom if we can find one — SDK-style images need it to
    // resolve `rom_func_lookup` at runtime. Missing bootrom is only fatal
    // for firmware that actually exercises those paths.
    let resolved_bootrom = resolve_bootrom_path(args.bootrom.as_deref());
    if let Some(path) = &resolved_bootrom {
        let bytes =
            std::fs::read(path).map_err(|e| format!("reading bootrom {}: {e}", path.display()))?;
        eprintln!(
            "mmio_trace_rp2040: loaded bootrom {} bytes from {}",
            bytes.len(),
            path.display()
        );
        emu.load_bootrom(&bytes);
    } else {
        eprintln!("mmio_trace_rp2040: no bootrom loaded (not found at {DEFAULT_BOOTROM_PATH})");
    }

    emu.reset();

    // If flash looks like a pico-sdk image (vector table at offset 0x100
    // with an SP in SRAM and a PC in flash), direct-boot into the app
    // reset handler — the vendored bootrom would otherwise wait on USB
    // MSC forever because we don't model QSPI flash detection. Same
    // heuristic as `picogus_diff_rp2040`.
    let sp = emu.bus.memory.xip_read32(SDK_VTOR_FLASH_OFFSET);
    let pc = emu.bus.memory.xip_read32(SDK_VTOR_FLASH_OFFSET + 4);
    let sp_in_sram = (0x2000_0000..=0x2004_2000).contains(&sp);
    let pc_in_flash = (0x1000_0000..0x1020_0000).contains(&(pc & !1));
    if sp_in_sram && pc_in_flash {
        emu.direct_boot_from_flash(SDK_VTOR_FLASH_OFFSET);
        eprintln!(
            "mmio_trace_rp2040: direct-boot SP={:#010x} PC={:#010x}",
            emu.cores[0].regs.sp(),
            emu.cores[0].regs.pc()
        );
    } else {
        eprintln!(
            "mmio_trace_rp2040: direct-boot skipped (flash+0x{:x} is not an SDK vector table — booting via bootrom reset vector)",
            SDK_VTOR_FLASH_OFFSET
        );
    }

    eprintln!(
        "mmio_trace_rp2040: running for {} cycles (trace on stdout)",
        args.cycles
    );
    emu.bus.mmio_trace_enabled = true;
    let ran = emu.run(args.cycles).expect("Serial run is infallible");
    emu.bus.mmio_trace_enabled = false;
    eprintln!(
        "mmio_trace_rp2040: ran {} cycles (requested {})",
        ran, args.cycles
    );

    Ok(())
}

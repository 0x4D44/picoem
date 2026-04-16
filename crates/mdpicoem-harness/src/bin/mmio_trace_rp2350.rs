//! MMIO trace runner — mdrp2350 (Cortex-M33) bus access logger.
//!
//! Phase 0b per `wrk_docs/2026.04.15 - HLD - RP2350 Peripheral Coverage V5.md`
//! §4 / §4.2.7: load firmware, enable `Bus::mmio_trace_enabled`, run for a
//! caller-supplied cycle budget, and emit one line per bus access to
//! stdout in the format
//!
//!     TRACE <R/W> <size> 0x<addr> val=0x<val> core=<N> pc=0x<pc>
//!
//! The trace's **tail** — the last registers firmware reads before it
//! spins or faults — identifies the next peripheral gap to close.
//! Phase 0b exit criterion §4.2.7: `--cycles 1_000_000 hello_timer.elf`
//! must emit ≥500 peripheral MMIO entries (hardware-gated — Arthur
//! runs this on the lab rig).
//!
//! CLI:
//!
//!     mmio_trace_rp2350 --cycles <N> <firmware>
//!
//! `<firmware>` is loaded as a 2 MB XIP flash image at `0x1000_0000`.
//! If a sibling `roms/rp2350/bootrom-combined.bin` exists we also load
//! it at ROM base so SDK-style firmware reaches its reset handler
//! through the bootrom. ELF loading is deferred — the Cargo workspace
//! does not yet depend on an ELF crate; we print a clear error if an
//! ELF is supplied.
//!
//! Mirrors the RP2040 sibling at `mmio_trace_rp2040.rs`. Same runtime
//! flag idiom; see `crates/mdrp2350/src/bus/mod.rs::Bus::emit_mmio_trace`
//! for the coverage rationale.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use mdrp2350::{Config, Emulator};

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
/// no explicit `--bootrom` is given. Matches `mdrp2350app` / `paced_bench_rp2350`.
const DEFAULT_BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";

/// System clock for the pacer math. Any value is fine; we don't
/// wall-clock pace here, but the clock tree seeds PLL / divider
/// calculations consistently. `Config::default()` matches
/// `mdrp2350app`.
fn default_config() -> Config {
    Config::default()
}

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
    Ok(Args { cycles, firmware, bootrom })
}

fn print_usage() {
    eprintln!(
        "Usage:\n  \
         mmio_trace_rp2350 --cycles <N> <firmware.bin>\n                   \
         [--bootrom <path>]\n\
         \n\
         --cycles    Required. Number of virtual cycles to run the emulator.\n\
         <firmware>  Required. 2 MB XIP flash image (.bin). ELF not yet\n              \
                     supported — rebuild as .bin with llvm-objcopy.\n\
         --bootrom   Optional 32 KB RP2350 bootrom image. Default-searches\n              \
                     `roms/rp2350/bootrom-combined.bin` when the file exists.\n\
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
    // flash images for Phase 0b; an ELF file needs `llvm-objcopy -O binary`
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
    mdpicoem_harness::harness_tracing_init();

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
        "mmio_trace_rp2350: loaded {} bytes from {}",
        flash_bytes.len(),
        args.firmware.display(),
    );

    let mut emu = Emulator::new(default_config());

    // Load a bootrom if we can find one — SDK-style firmware needs it
    // to resolve `rom_func_lookup` at runtime and to execute the reset
    // vector at ROM word 1. Missing bootrom is only fatal for firmware
    // that actually exercises those paths; we log and proceed.
    let resolved_bootrom = resolve_bootrom_path(args.bootrom.as_deref());
    if let Some(path) = &resolved_bootrom {
        let bytes = std::fs::read(path)
            .map_err(|e| format!("reading bootrom {}: {e}", path.display()))?;
        eprintln!(
            "mmio_trace_rp2350: loaded bootrom {} bytes from {}",
            bytes.len(),
            path.display(),
        );
        emu.load_bootrom(&bytes);
    } else {
        eprintln!(
            "mmio_trace_rp2350: no bootrom loaded (not found at {DEFAULT_BOOTROM_PATH})",
        );
    }

    emu.load_flash(&flash_bytes);
    emu.reset();

    // Core 1 stays halted — the trace is single-core by design. Dual-
    // core scenarios need separate tooling (cross-core FIFO timing
    // disambiguation, SIO_BELL/SEV, etc.) not covered by this oracle.
    emu.core_mut(1).halt();

    eprintln!(
        "mmio_trace_rp2350: running for {} cycles (trace on stdout)",
        args.cycles,
    );
    emu.bus.mmio_trace_enabled = true;
    let ran = emu.run(args.cycles);
    emu.bus.mmio_trace_enabled = false;
    eprintln!(
        "mmio_trace_rp2350: ran {} cycles (requested {})",
        ran, args.cycles,
    );

    Ok(())
}

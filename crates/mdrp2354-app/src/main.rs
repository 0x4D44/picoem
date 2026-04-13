use std::io::stdout;
use std::panic;
use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, RwLock};
use std::thread::Builder;

use anyhow::Context;
use crossterm::cursor;
use crossterm::execute;
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};

use mdrp2354_app::sim::{self, FirmwareBytes};
use mdrp2354_app::snapshot::Snapshot;
use mdrp2354_app::ui;

/// Resolves a preset alias (e.g. `"blinky"`) to the repo-relative ROM path,
/// or passes an arbitrary user-supplied path through unchanged. Preset aliases
/// are anchored to the crate's manifest directory so invocation works from any
/// CWD; custom paths remain CWD-relative because they are user-supplied.
fn resolve_firmware_path(arg: &str) -> PathBuf {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    match arg {
        "blinky" => manifest_dir.join("../../roms/blinky.bin"),
        "benchmark" => manifest_dir.join("../../roms/benchmark.bin"),
        "lcd" => manifest_dir.join("../../roms/lcd_demo.bin"),
        path => PathBuf::from(path),
    }
}

fn bootrom_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms/bootrom-combined.bin")
}

fn install_panic_hook() {
    let default_hook = panic::take_hook();
    panic::set_hook(Box::new(move |info| {
        let _ = execute!(stdout(), cursor::Show, LeaveAlternateScreen);
        let _ = disable_raw_mode();
        default_hook(info);
    }));
}

struct TerminalGuard;

impl TerminalGuard {
    fn new() -> anyhow::Result<Self> {
        enable_raw_mode()?;
        if let Err(e) = execute!(stdout(), EnterAlternateScreen, cursor::Hide) {
            let _ = disable_raw_mode();
            return Err(e.into());
        }
        Ok(Self)
    }
}

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        let _ = execute!(stdout(), cursor::Show, LeaveAlternateScreen);
        let _ = disable_raw_mode();
    }
}

fn main() -> anyhow::Result<()> {
    let arg = std::env::args().nth(1).unwrap_or_else(|| "blinky".into());
    let fw_path = resolve_firmware_path(&arg);
    let bootrom_path = bootrom_path();

    let fw = FirmwareBytes {
        bootrom: std::fs::read(&bootrom_path)
            .with_context(|| format!("reading {}", bootrom_path.display()))?,
        flash: std::fs::read(&fw_path).with_context(|| format!("reading {}", fw_path.display()))?,
    };

    install_panic_hook();
    let guard = TerminalGuard::new()?;

    let snapshot = Arc::new(RwLock::new(Snapshot::default()));
    let shutdown = Arc::new(AtomicBool::new(false));

    let sim_handle = {
        let snapshot = Arc::clone(&snapshot);
        let shutdown = Arc::clone(&shutdown);
        Builder::new()
            .name("sim".into())
            .spawn(move || sim::run(fw, snapshot, shutdown))?
    };

    let ui_result = ui::run(snapshot, Arc::clone(&shutdown), &sim_handle);

    drop(guard);

    match sim_handle.join() {
        Ok(Ok(())) => ui_result,
        Ok(Err(e)) => {
            eprintln!("sim thread error: {e:#}");
            std::process::exit(1);
        }
        Err(payload) => {
            let msg = payload
                .downcast_ref::<&str>()
                .copied()
                .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
                .unwrap_or("unknown panic");
            eprintln!("sim thread panicked: {msg}");
            std::process::exit(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::resolve_firmware_path;
    use std::path::{Path, PathBuf};

    fn manifest_dir() -> &'static Path {
        Path::new(env!("CARGO_MANIFEST_DIR"))
    }

    #[test]
    fn preset_aliases_resolve_under_manifest_dir() {
        assert_eq!(
            resolve_firmware_path("blinky"),
            manifest_dir().join("../../roms/blinky.bin")
        );
        assert_eq!(
            resolve_firmware_path("benchmark"),
            manifest_dir().join("../../roms/benchmark.bin")
        );
        assert_eq!(
            resolve_firmware_path("lcd"),
            manifest_dir().join("../../roms/lcd_demo.bin")
        );
    }

    #[test]
    fn arbitrary_path_passes_through_unchanged() {
        let custom = "some/user/path/firmware.bin";
        assert_eq!(resolve_firmware_path(custom), PathBuf::from(custom));
    }
}

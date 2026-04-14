//! Smoke tests for `mdrp2040app`.
//!
//! Loads `roms/rp2040/bootrom.bin` + `roms/rp2040/blinky.bin` through the
//! sim thread and polls the shared `Snapshot`, mirroring the
//! mdrp2350app integration harness.
//!
//! These tests exercise the full stack the showcase app depends on:
//!   * sim-thread construction via `sim::run`
//!   * firmware loading through the real Emulator API
//!   * GPIO state becoming visible in the snapshot
//!
//! A blinky run at ROSC (~6.5 MHz) with a ~64K delay loop toggles the
//! LED once every ~40 ms of simulated time. We watch for the `SET`
//! (initial drive HIGH) that the reset handler performs before its
//! first delay — that alone is enough to prove the plumbing is live.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};
use std::thread::{Builder, JoinHandle};
use std::time::{Duration, Instant};

use mdrp2040app::sim::{self, FirmwareBytes};
use mdrp2040app::snapshot::Snapshot;

fn roms_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms/rp2040")
}

fn load_firmware(name: &str) -> FirmwareBytes {
    let roms = roms_dir();
    FirmwareBytes {
        bootrom: std::fs::read(roms.join("bootrom.bin"))
            .expect("roms/rp2040/bootrom.bin missing"),
        flash: std::fs::read(roms.join(name))
            .unwrap_or_else(|_| panic!("roms/rp2040/{name} missing")),
    }
}

fn spawn_sim(
    fw: FirmwareBytes,
    snapshot: Arc<RwLock<Snapshot>>,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<anyhow::Result<()>> {
    Builder::new()
        .name("sim-test".into())
        .spawn(move || sim::run(fw, snapshot, shutdown))
        .expect("spawn sim thread")
}

#[test]
fn blinky_drives_gpio25_high_within_2s() {
    let fw = load_firmware("blinky.bin");

    let snapshot = Arc::new(RwLock::new(Snapshot::default()));
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_sim(fw, Arc::clone(&snapshot), Arc::clone(&shutdown));

    // Poll the shared snapshot until we observe GPIO25 set or the
    // window elapses. The reset handler drives GPIO25 HIGH before its
    // first delay loop — that should be visible well under a second.
    let deadline = Instant::now() + Duration::from_millis(2000);
    let mut seen_high = false;
    while Instant::now() < deadline {
        let snap = snapshot.read().unwrap().clone();
        let merged_out = snap.gpio_out & snap.gpio_oe;
        if merged_out & (1 << 25) != 0 {
            seen_high = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    let final_snapshot = snapshot.read().unwrap().clone();

    shutdown.store(true, Ordering::Relaxed);
    handle
        .join()
        .expect("sim thread join")
        .expect("sim thread result");

    assert!(final_snapshot.cycles > 0, "emulator should have run");
    assert!(
        seen_high,
        "GPIO 25 should have been driven HIGH within 2 s (gpio_out={:#010x}, gpio_oe={:#010x})",
        final_snapshot.gpio_out, final_snapshot.gpio_oe,
    );
}

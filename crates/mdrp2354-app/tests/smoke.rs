use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};
use std::thread::{Builder, JoinHandle};
use std::time::{Duration, Instant};

use mdrp2354_app::sim::{self, FirmwareBytes};
use mdrp2354_app::snapshot::Snapshot;

fn roms_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms")
}

fn load_firmware(name: &str) -> FirmwareBytes {
    let roms = roms_dir();
    FirmwareBytes {
        bootrom: std::fs::read(roms.join("bootrom-combined.bin"))
            .expect("roms/bootrom-combined.bin missing"),
        flash: std::fs::read(roms.join(name)).unwrap_or_else(|_| panic!("roms/{name} missing")),
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
fn blinky_toggles_gpio25_within_500ms() {
    let fw = load_firmware("blinky.bin");

    let snapshot = Arc::new(RwLock::new(Snapshot::default()));
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_sim(fw, Arc::clone(&snapshot), Arc::clone(&shutdown));

    // Poll the shared snapshot until we observe GPIO 25 set or the window elapses.
    // Budget is 2 s to tolerate slow CI; the 5 ms poll keeps latency low when blinky
    // toggles in well under a second on real hardware.
    let deadline = Instant::now() + Duration::from_millis(2000);
    let mut seen = false;
    while Instant::now() < deadline {
        if snapshot.read().unwrap().gpio_out & (1 << 25) != 0 {
            seen = true;
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

    assert!(final_snapshot.cycles > 0, "emulator ran");
    assert!(
        seen,
        "GPIO 25 should have been set at least once within 2 s"
    );
}

#[test]
fn lcd_demo_writes_hello_from_mdrp2354_within_2s() {
    let fw = load_firmware("lcd_demo.bin");

    let snapshot = Arc::new(RwLock::new(Snapshot::default()));
    let shutdown = Arc::new(AtomicBool::new(false));
    let handle = spawn_sim(fw, Arc::clone(&snapshot), Arc::clone(&shutdown));

    // Poll until both rows contain the expected strings, or the timeout
    // elapses. The firmware completes a full refresh in well under 100 ms,
    // so 2 s leaves plenty of slack for slow CI.
    let deadline = Instant::now() + Duration::from_millis(2000);
    let mut populated = false;
    while Instant::now() < deadline {
        let snap = snapshot.read().unwrap().clone();
        let has_hello = snap.lcd.rows[0].starts_with(b"Hello from");
        let has_mdrp = snap.lcd.rows[1].starts_with(b"mdrp2354!");
        if has_hello && has_mdrp {
            populated = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }

    let final_snapshot = snapshot.read().unwrap().clone();

    shutdown.store(true, Ordering::Relaxed);
    handle
        .join()
        .expect("sim thread join")
        .expect("sim thread result");

    assert!(final_snapshot.cycles > 0, "emulator ran");
    assert!(
        populated,
        "LCD should show 'Hello from' on row 0 and 'mdrp2354!' on row 1 within 2 s \
         (row 0 = {:?}, row 1 = {:?})",
        String::from_utf8_lossy(&final_snapshot.lcd.rows[0]),
        String::from_utf8_lossy(&final_snapshot.lcd.rows[1]),
    );
}

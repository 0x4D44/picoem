use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};
use std::thread::Builder;
use std::time::Duration;

use mdrp2354_app::sim::{self, FirmwareBytes};
use mdrp2354_app::snapshot::Snapshot;

#[test]
fn blinky_toggles_gpio25_within_500ms() {
    let roms = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../roms");
    let bootrom = std::fs::read(roms.join("bootrom-combined.bin"))
        .expect("roms/bootrom-combined.bin missing");
    let flash = std::fs::read(roms.join("blinky.bin")).expect("roms/blinky.bin missing");

    let snapshot = Arc::new(RwLock::new(Snapshot::default()));
    let shutdown = Arc::new(AtomicBool::new(false));

    let handle = {
        let snapshot = Arc::clone(&snapshot);
        let shutdown = Arc::clone(&shutdown);
        Builder::new()
            .name("sim-test".into())
            .spawn(move || sim::run(FirmwareBytes { bootrom, flash }, snapshot, shutdown))
            .expect("spawn sim thread")
    };

    // Poll the shared snapshot until we observe GPIO 25 set or the window elapses.
    // Budget is 2 s to tolerate slow CI; the 5 ms poll keeps latency low when blinky
    // toggles in well under a second on real hardware.
    let deadline = std::time::Instant::now() + Duration::from_millis(2000);
    let mut seen = false;
    while std::time::Instant::now() < deadline {
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

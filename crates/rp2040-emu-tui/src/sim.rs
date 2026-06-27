use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};
use std::time::Instant;

use rp2040_emu::{Config, Emulator, Pacer};

use crate::devices::bench::BenchmarkPoller;
use crate::snapshot::Snapshot;
use picoem_devices::LcdDecoder;

pub struct FirmwareBytes {
    pub bootrom: Vec<u8>,
    pub flash: Vec<u8>,
}

pub fn run(
    fw: FirmwareBytes,
    snapshot: Arc<RwLock<Snapshot>>,
    shutdown: Arc<AtomicBool>,
) -> anyhow::Result<()> {
    let config = Config::default();
    let mut emu = Emulator::new(config);

    emu.load_bootrom(&fw.bootrom);
    emu.load_flash(&fw.flash);
    emu.reset();
    emu.core_mut(1).halt();

    // Initialise the pacer from the Bus clock tree — the same zero-cost
    // follow-firmware model as `rp2350_emu_tui`. At reset the tree reports
    // ROSC (~6.5 MHz); firmware that configures a PLL will speed the
    // emulator up via the per-quantum update below.
    let mut pacer = Pacer::with_quantum(emu.bus.sys_clk_hz(), 150);
    let qc = pacer.quantum_cycles();
    let start = Instant::now();

    let mut lcd = LcdDecoder::new(14, 15, 16);
    let mut bench = BenchmarkPoller::new();

    // Windowed MHz measurement: update every ~500ms for a responsive reading
    // that reflects current throughput rather than a lifetime cumulative average.
    let mut mhz_prev_cycles = 0u64;
    let mut mhz_prev_time = start;
    let mut effective_mhz = 0.0f64;

    while !shutdown.load(Ordering::Relaxed) {
        pacer.begin_quantum();
        let _ = emu.run(qc); // Serial-mode app: infallible
        pacer.end_quantum();

        // Follow firmware clock reconfiguration (PLL bring-up, mux
        // switches). Zero-cost when sys_clk_hz is unchanged.
        pacer.update_sys_clk_hz(emu.bus.sys_clk_hz());

        let gpio_out = emu.bus.sio.gpio_out;
        let gpio_oe = emu.bus.sio.gpio_oe;
        lcd.sample(gpio_out);

        let elapsed = start.elapsed();
        let cycles = emu.cycles();
        bench.poll(&emu, elapsed);

        let now = Instant::now();
        let dt = now.duration_since(mhz_prev_time);
        if dt.as_millis() >= 500 {
            let dc = cycles - mhz_prev_cycles;
            let secs = dt.as_secs_f64();
            effective_mhz = (dc as f64) / secs / 1e6;
            mhz_prev_cycles = cycles;
            mhz_prev_time = now;
        }

        let mut s = snapshot.write().unwrap_or_else(|e| e.into_inner());
        s.cycles = cycles;
        s.wall_ms = elapsed.as_millis() as u64;
        s.effective_mhz = effective_mhz;
        s.pc = emu.core(0).regs.pc();
        s.gpio_out = gpio_out;
        s.gpio_oe = gpio_oe;
        s.lcd = lcd.state();
        s.benchmark = bench.report();
    }

    Ok(())
}

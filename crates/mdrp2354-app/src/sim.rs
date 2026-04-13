// Note: this module uses `mdrp2354::Pacer`, which is gated on
// `#[cfg(target_arch = "x86_64")]` upstream. The whole showcase app therefore
// inherits that constraint (same as the existing `paced_bench` binary).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};
use std::time::Instant;

use mdrp2354::{Config, Emulator, Pacer};

use crate::devices::bench::BenchmarkPoller;
use crate::devices::lcd::LcdDecoder;
use crate::snapshot::Snapshot;

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
    let sys_clk_hz = config.sys_clk_hz;
    let mut emu = Emulator::new(config);

    emu.load_bootrom(&fw.bootrom);
    emu.load_flash(&fw.flash);
    emu.reset();
    emu.core_mut(1).halt();

    let mut pacer = Pacer::with_quantum(sys_clk_hz, 150);
    let qc = pacer.quantum_cycles();
    let start = Instant::now();

    let mut lcd = LcdDecoder::new();
    let mut bench = BenchmarkPoller::new();

    while !shutdown.load(Ordering::Relaxed) {
        pacer.begin_quantum();
        emu.run(qc);
        pacer.end_quantum();

        let gpio_out = emu.bus.sio.gpio_out;
        let gpio_oe = emu.bus.sio.gpio_oe;
        lcd.sample(gpio_out);

        let elapsed = start.elapsed();
        let cycles = emu.cycles();
        bench.poll(&emu, elapsed);

        let secs = elapsed.as_secs_f64();
        let effective_mhz = if secs > 0.0 {
            (cycles as f64) / secs / 1e6
        } else {
            0.0
        };

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

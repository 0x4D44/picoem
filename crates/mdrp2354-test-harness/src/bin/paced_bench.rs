//! Paced benchmark: measures whether the emulator can sustain real-time at 150 MHz.

use mdrp2354::{Config, Emulator, Pacer, PacerStats};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[cfg(target_os = "windows")]
#[allow(non_camel_case_types)]
mod win {
    use std::os::raw::c_void;

    type HANDLE = *mut c_void;
    type DWORD = u32;
    type BOOL = i32;
    type DWORD_PTR = usize;

    const HIGH_PRIORITY_CLASS: DWORD = 0x0000_0080;
    const THREAD_PRIORITY_TIME_CRITICAL: i32 = 15;

    unsafe extern "system" {
        fn GetCurrentProcess() -> HANDLE;
        fn GetCurrentThread() -> HANDLE;
        fn SetPriorityClass(hProcess: HANDLE, dwPriorityClass: DWORD) -> BOOL;
        fn SetThreadPriority(hThread: HANDLE, nPriority: i32) -> BOOL;
        fn SetThreadAffinityMask(hThread: HANDLE, dwThreadAffinityMask: DWORD_PTR) -> DWORD_PTR;
    }

    /// Raise process to HIGH_PRIORITY_CLASS, raise current thread to
    /// TIME_CRITICAL, and pin to the given core. Uses HIGH rather than
    /// REALTIME to avoid blocking kernel threads on Windows.
    pub fn boost_and_pin(core: usize) -> Result<(), &'static str> {
        unsafe {
            if SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS) == 0 {
                return Err("SetPriorityClass failed");
            }
            if SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) == 0 {
                return Err("SetThreadPriority failed");
            }
            let mask: DWORD_PTR = 1 << core;
            if SetThreadAffinityMask(GetCurrentThread(), mask) == 0 {
                return Err("SetThreadAffinityMask failed");
            }
        }
        Ok(())
    }
}

fn main() {
    let seconds = parse_arg("--seconds").unwrap_or(5);
    let quantum = parse_arg("--quantum").unwrap_or(150);
    let clock_mhz = parse_arg("--clock-mhz").unwrap_or(150);
    let sys_clk_hz = clock_mhz * 1_000_000;
    let core = parse_arg("--core").unwrap_or(2) as usize;

    if seconds == 0 || clock_mhz == 0 {
        eprintln!("error: --seconds and --clock-mhz must be > 0");
        std::process::exit(1);
    }

    // Raise priority and pin to a specific core to minimise OS preemption.
    // Uses HIGH_PRIORITY_CLASS (not REALTIME) to stay safe — won't block kernel threads.
    #[cfg(target_os = "windows")]
    match win::boost_and_pin(core) {
        Ok(()) => println!("Pinned to core {} at HIGH priority / TIME_CRITICAL", core),
        Err(e) => eprintln!("warning: failed to boost priority: {} (continuing with default)", e),
    }
    #[cfg(not(target_os = "windows"))]
    let _ = core;

    // --- Set up emulator with a tight SRAM loop ---
    let mut emu = Emulator::new(Config {
        sys_clk_hz,
        ..Default::default()
    });

    // Write workload: MOVS R0,#1 | ADDS R0,R0,#1 then B .-2 (back to ADDS)
    emu.poke(0x20000000, 0x1C40_2001);
    emu.poke(0x20000004, 0x0000_E7FD);

    // Core 0: run from SRAM
    emu.core_mut(0).regs.set_pc(0x20000000);
    emu.core_mut(0).regs.xpsr = 1 << 24; // Thumb bit
    emu.core_mut(0).regs.msp = 0x2008_0000;
    emu.core_mut(0).regs.r[13] = 0x2008_0000;

    // Core 1: halted
    emu.core_mut(1).halt();

    // --- Set up pacer ---
    let mut pacer = Pacer::with_quantum(sys_clk_hz, quantum.into());
    let stats = pacer.stats();

    println!("mdrp2354 paced benchmark — target {} MHz, quantum {} cycles", clock_mhz, quantum);
    println!("TSC calibrated: {} MHz\n", pacer.tsc_freq_hz() / 1_000_000);
    println!("{:>6} {:>14} {:>10} {:>8} {:>10} {:>8}",
        "time", "emu_cycles", "emu_MHz", "util%", "headroom%", "behind");

    // --- Monitoring thread ---
    stats.set_running(true);
    let mon_stats = Arc::clone(&stats);
    let monitor = std::thread::spawn(move || monitor_loop(mon_stats));

    // --- Paced execution ---
    let start = Instant::now();
    let duration = Duration::from_secs(seconds.into());
    let qc = pacer.quantum_cycles();

    while start.elapsed() < duration {
        pacer.begin_quantum();
        emu.run(qc);
        pacer.end_quantum();
    }

    stats.set_running(false);
    monitor.join().unwrap();

    // --- Summary ---
    let snap = stats.snapshot();
    let wall_secs = start.elapsed().as_secs_f64();
    println!("\n--- summary ---");
    println!("Duration:       {:.1} s", wall_secs);
    println!("Total cycles:   {}", snap.emulated_cycles);
    println!("Avg MHz:        {:.1}", snap.emulated_mhz());
    println!("Avg util:       {:.1}%", snap.utilization() * 100.0);
    println!("Behind count:   {}", snap.behind_count);

    let total_quanta = snap.emulated_cycles / quantum as u64;
    let behind_rate = snap.behind_count as f64 / total_quanta.max(1) as f64;
    let mhz_ratio = snap.emulated_mhz() / clock_mhz as f64;

    if mhz_ratio >= 0.99 && behind_rate < 0.001 {
        println!("Verdict:        REAL-TIME OK ({:.1}% of target, {:.2}% headroom, {:.3}% behind)",
                 mhz_ratio * 100.0, snap.headroom() * 100.0, behind_rate * 100.0);
    } else if mhz_ratio >= 0.95 && behind_rate < 0.01 {
        println!("Verdict:        REAL-TIME MARGINAL ({:.1}% of target, {:.2}% behind)",
                 mhz_ratio * 100.0, behind_rate * 100.0);
    } else {
        println!("Verdict:        CANNOT SUSTAIN REAL-TIME ({:.1}% of target, {:.2}% behind)",
                 mhz_ratio * 100.0, behind_rate * 100.0);
    }
}

fn monitor_loop(stats: Arc<PacerStats>) {
    let start = Instant::now();
    while stats.is_running() {
        std::thread::sleep(Duration::from_secs(1));
        if !stats.is_running() {
            break;
        }
        let snap = stats.snapshot();
        let elapsed = start.elapsed().as_secs();
        println!("{:>6} {:>14} {:>10.1} {:>7.1}% {:>9.1}% {:>8}",
            elapsed,
            snap.emulated_cycles,
            snap.emulated_mhz(),
            snap.utilization() * 100.0,
            snap.headroom() * 100.0,
            snap.behind_count);
    }
}

fn parse_arg(name: &str) -> Option<u32> {
    let args: Vec<String> = std::env::args().collect();
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
}

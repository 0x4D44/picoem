//! Paced benchmark: measures whether the mdrp2040 emulator can sustain
//! real-time at the RP2040's stock 125 MHz.
//!
//! Mirrors `paced_bench_rp2350` minus the FPU workload — the M0+ has no
//! FPU and no coprocessors, so there's only one workload (a tight ALU
//! loop). The host-cycles-per-emulated-cycle figure is reported for
//! info; there is no equivalent of the RP2350 HLD §12 budget gate yet.
//!
//! Flags:
//!   --seconds N        Wall-clock duration (default 5; ignored with --cycles).
//!   --cycles N         Unpaced mode: run exactly this many emulated cycles
//!                      (rounded up to a whole quantum). Useful for fixed-size
//!                      micro-benchmarks and CI gate checks.
//!   --quantum N        Emulated cycles per pacing quantum (default 125).
//!   --clock-mhz N      Target sys_clk frequency in MHz (default 125).
//!   --core N           Pin benchmark thread to host core N (default 2).
//!   --unpaced          Run flat-out, no real-time pacing; also emits the
//!                      host-cycles-per-emulated-cycle figure.
//!   --dual-core        Run both M0+ cores in parallel (single-threaded).

use mdrp2040::{Config, Emulator, Pacer, PacerStats};
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
    let cycles_target = parse_arg_u64("--cycles");
    let quantum = parse_arg("--quantum").unwrap_or(125);
    let clock_mhz = parse_arg("--clock-mhz").unwrap_or(125);
    let sys_clk_hz = clock_mhz * 1_000_000;
    let core = parse_arg("--core").unwrap_or(2) as usize;
    let unpaced = std::env::args().any(|a| a == "--unpaced");
    let dual_core = std::env::args().any(|a| a == "--dual-core");

    if seconds == 0 || clock_mhz == 0 {
        eprintln!("error: --seconds and --clock-mhz must be > 0");
        std::process::exit(1);
    }
    if cycles_target.is_some() && !unpaced {
        eprintln!("error: --cycles requires --unpaced (paced mode is duration-driven)");
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

    // --- Set up emulator ---
    let mut emu = Emulator::new(Config { sys_clk_hz });

    // Core 0 workload at 0x2000_0000 (striped SRAM, bank 0 stripe entry):
    //   MOVS R0,#1 | ADDS R0,R0,#1 then B .-2 (back to ADDS).
    emu.poke(0x2000_0000, 0x1C40_2001);
    emu.poke(0x2000_0004, 0x0000_E7FD);

    emu.core_mut(0).regs.set_pc(0x2000_0000);
    emu.core_mut(0).regs.xpsr = 1 << 24; // Thumb bit
    // Top of striped SRAM (256 KB at 0x2000_0000); first push targets 0x2003_FFFC.
    emu.core_mut(0).regs.msp = 0x2004_0000;
    emu.core_mut(0).regs.r[13] = 0x2004_0000;

    if dual_core {
        // Place core 1 in SRAM5 scratch (bank 5) — fully separate from
        // core 0's striped accesses, so dual-core measurements don't pick
        // up bus contention noise. Stack at top of SRAM5.
        emu.poke(0x2004_1000, 0x1C49_2101); // MOVS R1,#1 | ADDS R1,R1,#1
        emu.poke(0x2004_1004, 0x0000_E7FD); // B .-2 (back to ADDS)
        emu.core_mut(1).regs.set_pc(0x2004_1000);
        emu.core_mut(1).regs.xpsr = 1 << 24;
        emu.core_mut(1).regs.msp = 0x2004_2000;
        emu.core_mut(1).regs.r[13] = 0x2004_2000;
        // RP2040 boots with core 1 halted (Pico SDK wake handshake);
        // wake it directly here since the benchmark bypasses the SDK.
        emu.core_mut(1).wake();
    }

    // --- Set up pacer ---
    let mut pacer = Pacer::with_quantum(sys_clk_hz, quantum.into());
    let stats = pacer.stats();

    let core_mode = if dual_core { "dual-core" } else { "single-core" };
    println!(
        "mdrp2040 paced benchmark — target {} MHz, quantum {} cycles, {}",
        clock_mhz, quantum, core_mode
    );
    println!("TSC calibrated: {} MHz\n", pacer.tsc_freq_hz() / 1_000_000);
    println!("{:>6} {:>14} {:>10} {:>8} {:>10} {:>8}",
        "time", "emu_cycles", "emu_MHz", "util%", "headroom%", "behind");

    // --- Monitoring thread ---
    stats.set_running(true);
    let mon_stats = Arc::clone(&stats);
    let monitor = std::thread::spawn(move || monitor_loop(mon_stats));

    // --- Execution ---
    let start = Instant::now();
    let duration = Duration::from_secs(seconds.into());
    let qc = pacer.quantum_cycles();

    let unpaced_cycles: u64 = if unpaced {
        if let Some(target) = cycles_target {
            println!(
                "(unpaced mode — running flat-out until {} emulated cycles)",
                target
            );
        } else {
            println!("(unpaced mode — running flat-out, no real-time pacing)");
        }
        let mut n: u64 = 0;
        loop {
            if let Some(target) = cycles_target {
                if n >= target { break; }
            } else if start.elapsed() >= duration {
                break;
            }
            let consumed = emu.run(qc);
            n += consumed;
        }
        n
    } else {
        while start.elapsed() < duration {
            pacer.begin_quantum();
            emu.run(qc);
            pacer.end_quantum();
        }
        0 // unused
    };

    stats.set_running(false);
    monitor.join().unwrap();

    // --- Summary ---
    let wall_secs = start.elapsed().as_secs_f64();
    println!("\n--- summary ---");
    println!("Duration:       {:.1} s", wall_secs);

    if unpaced {
        let mhz = unpaced_cycles as f64 / wall_secs / 1_000_000.0;
        let host_cycles_per_emu = pacer.tsc_freq_hz() as f64 * wall_secs / unpaced_cycles as f64;
        println!("Total cycles:   {}", unpaced_cycles);
        println!("Avg MHz:        {:.1}", mhz);
        println!("Host/emu cycle: {:.2}", host_cycles_per_emu);
        println!("Verdict:        UNPACED (profiling mode)");
        return;
    }

    let snap = stats.snapshot();
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

fn parse_arg_u64(name: &str) -> Option<u64> {
    let args: Vec<String> = std::env::args().collect();
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok())
}

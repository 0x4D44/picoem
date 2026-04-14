//! Paced benchmark: measures whether the emulator can sustain real-time at 150 MHz.
//!
//! Flags:
//!   --seconds N        Wall-clock duration (default 5; ignored with --cycles).
//!   --cycles N         Unpaced mode: run exactly this many emulated cycles
//!                      (rounded up to a whole quantum). Useful for fixed-size
//!                      micro-benchmarks and CI gate checks.
//!   --quantum N        Emulated cycles per pacing quantum (default 150).
//!   --clock-mhz N      Target sys_clk frequency in MHz (default 150).
//!   --core N           Pin benchmark thread to host core N (default 2).
//!   --unpaced          Run flat-out, no real-time pacing; also emits the
//!                      host-cycles-per-emulated-cycle figure for the HLD §12
//!                      performance budget check.
//!   --dual-core        Run both M33 cores in parallel (single-threaded).
//!   --workload basic   Tight MOVS/ADDS/B loop (default).
//!   --workload fpu-heavy
//!                      VADD/VMUL/VDIV/VSQRT loop exercising the FPU hot path.

use mdrp2354::{Config, Emulator, Pacer, PacerStats};
use std::sync::Arc;
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Workload selection + VFP encoders
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Workload {
    /// Default: MOVS R0,#1 / ADDS R0,R0,#1 / B .-2 — tight ALU loop.
    Basic,
    /// FPU-heavy: VADD S3,S1,S2 / VMUL S3,S3,S1 / VDIV S3,S3,S2 / VSQRT S3,S3 / B.
    /// Used by the HLD §12 performance budget check.
    FpuHeavy,
}

/// Encode VFP data-processing (VADD/VSUB/VMUL/VDIV) single-precision.
///
/// Mirrors the private `vfp_dp` helper in the test harness (kept local to
/// this binary so `paced_bench` stays self-contained).
fn vfp_dp(op_hi: u16, op_lo: u16, op2_lo: u16, sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vn = (sn >> 1) & 0xF;
    let n = sn & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xEE00 | (op_hi << 7) | (d << 6) | (op_lo << 4) | vn;
    let hw1 = (vd << 12) | 0x0A00 | (n << 7) | (op2_lo << 6) | (m << 5) | vm;
    (hw0, hw1)
}

fn enc_vadd(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b11, 0, sd, sn, sm) }
fn enc_vmul(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(0, 0b10, 0, sd, sn, sm) }
fn enc_vdiv(sd: u16, sn: u16, sm: u16) -> (u16, u16) { vfp_dp(1, 0b00, 0, sd, sn, sm) }

fn enc_vsqrt(sd: u16, sm: u16) -> (u16, u16) {
    // VSQRT.F32: unary with opc3=0b0001, t=1 (F32).
    let vd = (sd >> 1) & 0xF;
    let d = sd & 1;
    let vm = (sm >> 1) & 0xF;
    let m = sm & 1;
    let hw0 = 0xEE00 | (1 << 7) | (d << 6) | (0b11 << 4) | 0b0001;
    let hw1 = (vd << 12) | 0x0A00 | (1 << 7) | (1 << 6) | (m << 5) | vm;
    (hw0, hw1)
}

/// Pack a (hw0, hw1) Thumb-32 pair into a little-endian u32 for `poke`.
fn pair(hw0: u16, hw1: u16) -> u32 {
    (hw1 as u32) << 16 | (hw0 as u32)
}

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
    let quantum = parse_arg("--quantum").unwrap_or(150);
    let clock_mhz = parse_arg("--clock-mhz").unwrap_or(150);
    let sys_clk_hz = clock_mhz * 1_000_000;
    let core = parse_arg("--core").unwrap_or(2) as usize;
    let unpaced = std::env::args().any(|a| a == "--unpaced");
    let dual_core = std::env::args().any(|a| a == "--dual-core");
    let workload = parse_workload().unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1);
    });

    if seconds == 0 || clock_mhz == 0 {
        eprintln!("error: --seconds and --clock-mhz must be > 0");
        std::process::exit(1);
    }
    if cycles_target.is_some() && !unpaced {
        eprintln!("error: --cycles requires --unpaced (paced mode is duration-driven)");
        std::process::exit(1);
    }
    if dual_core && workload == Workload::FpuHeavy {
        // Keep the FPU-heavy workload single-core for a clean host-cycles/emu-cycle
        // reading; dual-core contends on the shared bus and muddies the metric.
        eprintln!("error: --workload fpu-heavy is not supported with --dual-core");
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

    // --- Set up emulator with the selected SRAM workload ---
    let mut emu = Emulator::new(Config {
        sys_clk_hz,
        ..Default::default()
    });

    match workload {
        Workload::Basic => {
            // Write workload: MOVS R0,#1 | ADDS R0,R0,#1 then B .-2 (back to ADDS)
            emu.poke(0x20000000, 0x1C40_2001);
            emu.poke(0x20000004, 0x0000_E7FD);
        }
        Workload::FpuHeavy => {
            // FPU hot-path workload — exercises VADD/VMUL/VDIV/VSQRT plus the
            // loop branch. Total loop length: 4 T32 FPU ops (16 bytes) + 1 T16
            // backward branch (2 bytes) = 18 bytes = 9 halfwords.
            let (va0, va1) = enc_vadd(3, 1, 2);   // VADD  S3, S1, S2
            let (vm0, vm1) = enc_vmul(3, 3, 1);   // VMUL  S3, S3, S1
            let (vd0, vd1) = enc_vdiv(3, 3, 2);   // VDIV  S3, S3, S2
            let (vs0, vs1) = enc_vsqrt(3, 3);     // VSQRT S3, S3
            emu.poke(0x20000000, pair(va0, va1));
            emu.poke(0x20000004, pair(vm0, vm1));
            emu.poke(0x20000008, pair(vd0, vd1));
            emu.poke(0x2000000C, pair(vs0, vs1));
            // B .-20 back to 0x20000000 (PC at B is 0x20000010, PC+4 = 0x20000014,
            // target = 0x20000000, imm11 = -10 = 0x7F6 → hw = 0xE7F6).
            emu.poke(0x20000010, 0x0000_E7F6);
        }
    }

    // Core 0: run from SRAM
    emu.core_mut(0).regs.set_pc(0x20000000);
    emu.core_mut(0).regs.xpsr = 1 << 24; // Thumb bit
    emu.core_mut(0).regs.msp = 0x2008_0000;
    emu.core_mut(0).regs.r[13] = 0x2008_0000;

    if workload == Workload::FpuHeavy {
        // Seed the FPU sources so the steady-state is finite and well-behaved
        // (no NaN/Inf churn that would distort timing). Pick values that
        // cycle without overflow:
        //   S1 = 2.0, S2 = 3.0
        //   VADD S3 = 5.0 ; VMUL S3 = 10.0 ; VDIV S3 ≈ 3.333 ; VSQRT S3 ≈ 1.826
        emu.core_mut(0).regs.s[1] = 2.0;
        emu.core_mut(0).regs.s[2] = 3.0;
    }

    if dual_core {
        // Core 1: running its own loop at a different SRAM address
        // (different bank to avoid confounding bus contention with perf measurement)
        emu.poke(0x20001000, 0x1C49_2101); // MOVS R1, #1 | ADDS R1, R1, #1
        emu.poke(0x20001004, 0x0000_E7FD); // B .-2 (back to ADDS)
        emu.core_mut(1).regs.set_pc(0x20001000);
        emu.core_mut(1).regs.xpsr = 1 << 24;
        emu.core_mut(1).regs.msp = 0x2007_0000;
        emu.core_mut(1).regs.r[13] = 0x2007_0000;
    } else {
        // Core 1: halted
        emu.core_mut(1).halt();
    }

    // --- Set up pacer ---
    let mut pacer = Pacer::with_quantum(sys_clk_hz, quantum.into());
    let stats = pacer.stats();

    let core_mode = if dual_core { "dual-core" } else { "single-core" };
    let workload_str = match workload {
        Workload::Basic => "basic",
        Workload::FpuHeavy => "fpu-heavy",
    };
    println!(
        "mdrp2354 paced benchmark — target {} MHz, quantum {} cycles, {}, workload {}",
        clock_mhz, quantum, core_mode, workload_str
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
        // Unpaced: run emulator as fast as possible. Used both for profiling
        // (flamegraph) — isolates the hot path from the pacer — and for the
        // HLD §12 performance budget check (host cycles per emulated cycle).
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
            emu.run(qc);
            n += qc;
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
        // Host TSC ticks are a reasonable proxy for host core cycles on modern
        // x86_64 (invariant TSC runs at a fixed base close to the CPU nominal
        // clock). Under HIGH_PRIORITY_CLASS / TIME_CRITICAL this gives a
        // stable-enough signal for the HLD §12 budget gate.
        let host_cycles_per_emu = pacer.tsc_freq_hz() as f64 * wall_secs / unpaced_cycles as f64;
        println!("Total cycles:   {}", unpaced_cycles);
        println!("Avg MHz:        {:.1}", mhz);
        println!("Workload:       {}", workload_str);
        println!("Host/emu cycle: {:.2} (target: <33 per HLD §12)", host_cycles_per_emu);
        if host_cycles_per_emu < 33.0 {
            println!("Budget:         OK ({:.2} < 33)", host_cycles_per_emu);
        } else {
            println!("Budget:         OVER ({:.2} >= 33) — investigate regression", host_cycles_per_emu);
        }
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

fn parse_workload() -> Result<Workload, String> {
    let args: Vec<String> = std::env::args().collect();
    // Accept both `--workload X` and `--workload=X` forms.
    for (i, a) in args.iter().enumerate() {
        if let Some(v) = a.strip_prefix("--workload=") {
            return match_workload(v);
        }
        if a == "--workload" {
            let v = args.get(i + 1).ok_or("--workload requires basic|fpu-heavy")?;
            return match_workload(v);
        }
    }
    Ok(Workload::Basic)
}

fn match_workload(s: &str) -> Result<Workload, String> {
    match s {
        "basic" => Ok(Workload::Basic),
        "fpu-heavy" => Ok(Workload::FpuHeavy),
        other => Err(format!(
            "invalid --workload '{other}' (expected basic|fpu-heavy)"
        )),
    }
}

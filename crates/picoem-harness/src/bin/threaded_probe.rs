//! Minimal serial-vs-threaded throughput probe for the RP2350
//! executor. Strips everything that's not the executor: no Pacer,
//! no pacing loop, no workload variety — just the basic tight-loop
//! (ADDS + B) and a measurement of instructions per wall second on
//! each runtime.
//!
//! Purpose: isolate the ~6× per-core gap observed in
//! `paced_bench_rp2350` (serial basic = 154 MHz, threaded basic =
//! 25 MHz) from everything else. If this probe shows the same gap
//! on the same workload with a far simpler shell, the gap is in
//! `core.step` / bus path, not in pacing overhead.

use rp2350_emu::{Config, DEFAULT_STEP_QUANTUM, Emulator, EmulatorBuilder, ExecutionModel};
use std::time::Instant;

#[cfg(target_os = "windows")]
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
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

/// Basic tight-loop workload. Loops forever at PC=0x2000_0002.
/// Matches `paced_bench_rp2350::setup_basic_core0`.
fn setup_basic_core0(emu: &mut Emulator) {
    emu.poke(0x2000_0000, 0x1C40_2001); // MOVS R0,#1 | ADDS R0,R0,#1
    emu.poke(0x2000_0004, 0x0000_E7FD); // B .-2
    emu.core_mut(0).regs.set_pc(0x2000_0000);
    emu.core_mut(0).regs.xpsr = 1 << 24; // T bit
}

fn main() {
    let target_cycles: u64 = 150_000_000; // 1s at 150 MHz equivalent

    // Mirror paced_bench_rp2350's main-thread setup: core 2, HIGH
    // priority class, TIME_CRITICAL thread priority. Toggled via
    // --boost arg. If these cause the paced_bench regression, this
    // will reproduce it here.
    let boost = std::env::args().any(|a| a == "--boost");
    if boost {
        #[cfg(target_os = "windows")]
        {
            match win::boost_and_pin(2) {
                Ok(()) => println!("(main thread: pinned core 2, HIGH + TIME_CRITICAL)"),
                Err(e) => println!("(boost_and_pin failed: {e})"),
            }
        }
    }

    // --- Serial ---
    {
        let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        setup_basic_core0(&mut emu);
        emu.core_mut(1).halt();

        // Warm up decode cache
        emu.run(10_000).expect("Serial run is infallible");
        let c0_start = emu.core(0).cycles();
        let t0 = Instant::now();
        emu.run(target_cycles).expect("Serial run is infallible");
        let wall_ns = t0.elapsed().as_nanos() as u64;
        let c0_delta = emu.core(0).cycles() - c0_start;
        let mhz = (c0_delta as f64) / (wall_ns as f64) * 1000.0;
        let ns_per_cyc = (wall_ns as f64) / (c0_delta as f64);
        println!(
            "serial:   {:>10} cycles in {:>10} ns = {:>6.1} MHz ({:.2} ns/cyc)",
            c0_delta, wall_ns, mhz, ns_per_cyc
        );
    }

    // --- Threaded (Windows + Linux x86_64) ---
    #[cfg(all(
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    {
        let mut emu = EmulatorBuilder::new(Config::default())
            .execution(ExecutionModel::Threaded)
            .build()
            .expect("Threaded build on x86_64 Windows / Linux");
        setup_basic_core0(&mut emu);
        emu.core_mut(1).halt();

        let step_q = DEFAULT_STEP_QUANTUM as u64;

        // Warm up decode cache + pay thread-spawn startup
        emu.run(step_q * (10_000 / step_q + 1))
            .expect("Threaded warm-up run");

        let c0_start = emu.core_cycles(0);
        let t0 = Instant::now();
        emu.run(target_cycles).expect("Threaded run");
        let wall_ns = t0.elapsed().as_nanos() as u64;
        let c0_delta = emu.core_cycles(0) - c0_start;
        let mhz = (c0_delta as f64) / (wall_ns as f64) * 1000.0;
        let ns_per_cyc = (wall_ns as f64) / (c0_delta as f64);
        println!(
            "threaded: {:>10} cycles in {:>10} ns = {:>6.1} MHz ({:.2} ns/cyc)   step_q={}",
            c0_delta, wall_ns, mhz, ns_per_cyc, step_q
        );

        // Large-step_q variants — must set step_quantum on the builder
        // BEFORE building; the Threaded runtime inherits it from the
        // Emulator and keeps it for all `run` calls.
        // Variants: cold (no warmup) vs warm (one run warmup to
        // populate decode cache + amortise first thread spawn).
        for &sq in &[256u32, 1024, 4096, 16384] {
            for warm in &[false, true] {
                let mut emu = EmulatorBuilder::new(Config::default())
                    .step_quantum(sq)
                    .execution(ExecutionModel::Threaded)
                    .build()
                    .expect("Threaded build on x86_64 Windows");
                setup_basic_core0(&mut emu);
                emu.core_mut(1).halt();
                if *warm {
                    emu.run(sq as u64 * (10_000u64 / sq as u64 + 1))
                        .expect("Threaded warm-up run");
                }
                let c0_start = emu.core_cycles(0);
                let t0 = Instant::now();
                emu.run(target_cycles).expect("Threaded run");
                let wall_ns = t0.elapsed().as_nanos() as u64;
                let c0_delta = emu.core_cycles(0) - c0_start;
                let mhz = (c0_delta as f64) / (wall_ns as f64) * 1000.0;
                let tag = if *warm { "warm" } else { "cold" };
                println!(
                    "threaded {}: {:>10} cycles in {:>10} ns = {:>6.1} MHz   step_q={}",
                    tag, c0_delta, wall_ns, mhz, sq
                );
            }
        }
    }
}

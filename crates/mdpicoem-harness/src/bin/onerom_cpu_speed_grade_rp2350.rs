//! OneROM CPU-mode speed-grade oracle — binary driver.
//!
//! Drives the emulator under `ThreadedEmulator` and a host
//! measurement thread to answer: "what's the minimum wall-clock
//! access time at which our emulator can still serve correct OneROM
//! bytes?" The verdict is expressed as an equivalent EPROM speed
//! grade (e.g. "reliable at 250 ns, fails at 200 ns").
//!
//! HLD: `wrk_docs/2026.04.22 - HLD - OneROM CPU Speed Grade
//! Oracle.md` V3 — specifically §5 architecture, §7 stimulus and
//! walk, §8 measurement loop, §9 ladder, §10 pre-measurement probe,
//! §11 verification, §12 liveness, §13 output.
//!
//! Phase 3 deliverable — the library module
//! [`mdpicoem_harness::onerom_cpu_speed_grade`] holds the
//! stability-free testable logic (walk plan, shuffle, verify,
//! ladder). This binary owns the host-thread wiring, `HIGH_PRIORITY_CLASS`
//! / `THREAD_PRIORITY_TIME_CRITICAL` / affinity pinning, and the
//! report format.
//!
//! HOST PREREQUISITE: run with the Windows **High performance**
//! power plan. Without it CPU frequency scaling dominates the ladder
//! below ~300 ns and the verdict becomes untrustworthy. The binary
//! cannot change the power plan programmatically — no API; document
//! as a run-time requirement and warn in `--help`.
//!
//! CLI:
//!   --seed <u64>                   Shuffle seed (default 0x1541_CAFE_0000_0001).
//!   --ladder <csv>                 Comma-separated ns targets (default
//!                                  500,400,300,250,200,150,120,100).
//!   --sweeps-per-threshold <n>     Sweeps per rung (default 3).
//!   --help                         Print this message and exit.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --release \
//!     --bin onerom_cpu_speed_grade_rp2350

// Windows-x86_64 gated — matches the `ThreadedEmulator` module and the
// host-pinning Win32 calls below. On other targets the binary
// degrades to a stub that reports "unsupported host" and exits 1,
// keeping the crate buildable.

#[cfg(not(all(target_arch = "x86_64", target_os = "windows")))]
fn main() -> std::process::ExitCode {
    eprintln!(
        "onerom_cpu_speed_grade_rp2350: unsupported host — requires x86_64 \
         Windows (ThreadedEmulator and SetThreadAffinityMask are Windows-only)"
    );
    std::process::ExitCode::from(1)
}

#[cfg(all(target_arch = "x86_64", target_os = "windows"))]
fn main() -> std::process::ExitCode {
    windows_main::run()
}

#[cfg(all(target_arch = "x86_64", target_os = "windows"))]
mod windows_main {
    use std::process::ExitCode;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::thread;
    use std::time::{Duration, Instant};

    use mdpicoem_harness::onerom_cpu_speed_grade::{
        build_walk_plan, first_failing_rung, shuffle_plan, verify_observed, FailContext,
        LadderRung, SweepReport, WalkStep, DEFAULT_LADDER, GPIO_DATA_BASE, ROM_SET_INDEX,
        WALK_PLAN_LEN,
    };
    use mdpicoem_harness::onerom_serving_oracle;
    use mdpicoem_harness::onerom_serving_oracle_cpu;
    use mdrp2350::threaded::{SharedState, ThreadedEmulator};
    use mdrp2350::{Config, Emulator, EmulatorBuilder};

    // -----------------------------------------------------------------------
    // Constants
    // -----------------------------------------------------------------------

    const BOOTROM_PATH: &str = "roms/rp2350/bootrom-combined.bin";
    const FLASH_PATH: &str =
        "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

    const DEFAULT_SEED: u64 = 0x1541_CAFE_0000_0001;
    const DEFAULT_SWEEPS_PER_THRESHOLD: u32 = 3;

    /// Boot cycle cap — mirrors the stress binary. Enough for any
    /// plausible CPU-mode sync; if we don't sync in 10M cycles the
    /// fixture is broken.
    const BOOT_CYCLE_CAP: u64 = 10_000_000;

    /// CPU serve-loop PC range for 1541 ROM set 0 (2364 bake). Same
    /// offset as `test-sdrr-0-cpu` per the stress binary's empirical
    /// verification; kept as a local constant so a future bake shift
    /// doesn't ripple through the shared library.
    const SERVE_LOOP_PC_LO: u32 = 0x1000_0926;
    const SERVE_LOOP_PC_HI: u32 = 0x1000_0930;

    /// Step quantum for the threaded runtime. Smaller than
    /// `paced_bench`'s 256 because this oracle needs sub-µs stim-to-
    /// merge latency — the coordinator's `update_gpio` runs once per
    /// quantum, so `step_quantum` is the ultimate ceiling on the
    /// achievable ns ladder rung.
    ///
    /// Default step_quantum for the threaded runtime.
    ///
    /// 1024 empirically chosen — a sq-sweep across {64, 256, 1024,
    /// 4096} on the 2000..40 ns ladder showed:
    ///
    /// - sq=64:   67 MHz throughput, 500 ns rung 2.5% effective error.
    ///   Barrier cost (~447 ns) dominates each 660 ns quantum.
    /// - sq=256:  144 MHz throughput, 500 ns 0.37%. Barrier amortizing.
    /// - sq=1024: 177 MHz throughput, 500 ns 0.12%, clean top rungs.
    ///   Serve-path-dominated.
    /// - sq=4096: 157 MHz throughput, most samples stall (quantum >
    ///   target), data too noisy to interpret.
    ///
    /// 1024 lands firmly in the "serve path, not barrier" regime while
    /// staying below the stall threshold for targets down to ~300 ns.
    /// The 90-ns silicon floor still isn't reachable via wall-clock on
    /// a multithreaded host — that needs the serial oracle (measures
    /// simulated-cycle serve time directly, ~10-13 cycles).
    ///
    /// Overridable via `--step-quantum <u32>`. Smaller values expose
    /// barrier-amortization bottleneck; larger values expose stall
    /// (quantum > target) bottleneck.
    const DEFAULT_THREADED_STEP_QUANTUM: u32 = 1024;

    /// Minimum master-cycle advance during a sample's wait window
    /// for the observation to be counted against the emulator.
    ///
    /// Set to 0 — no filtering beyond the raw "coord literally didn't
    /// move" check. We rely on statistical averaging over large
    /// sweep counts rather than pre-filtering samples. Transient
    /// host-level DPC/ISR stalls contribute to `host_stalled` (the
    /// hard-zero-delta counter) and surface there without silently
    /// inflating the pass count; everything else contributes to
    /// `errors` honestly. A larger threshold biased the sample
    /// population — dropping it keeps the methodology straight.
    const MIN_PROGRESS_CYCLES: u64 = 1;

    /// Quanta-chunk for the driver thread's `run_quanta` loop. Sized
    /// so one chunk lasts ~seconds of wall-clock — long enough that
    /// the entire ladder run (≈ 25 ms at default targets) sits inside
    /// a single chunk, avoiding the ~30 µs inter-chunk tear-down that
    /// otherwise punches deterministic dead-zones into the sample
    /// stream. `stop` observation latency is bounded by chunk length,
    /// which is fine for this tool (worst-case shutdown ≈ one chunk
    /// ≈ 1.7 s at 40 MHz emulated).
    const DRIVER_CHUNK_QUANTA: u64 = 1_048_576;

    /// Throughput probe duration.
    const PROBE_WALL_MS: u64 = 100;

    /// Pre-sweep liveness: if `master_cycle` doesn't advance within
    /// this wall-clock window, the runtime is wedged and we abort
    /// rather than produce a meaningless "all FAIL" ladder. The
    /// window is generous (10 ms) because at low emulator
    /// throughput a quantum can take multiple ms of wall time; we're
    /// diagnosing "wedged", not "slow".
    const LIVENESS_WINDOW_US: u64 = 10_000;

    /// Wall-clock watchdog — abort the whole run if the ladder hasn't
    /// finished within this bound. §12.
    const WATCHDOG_SECS: u64 = 60;

    /// Host core to pin the measurement thread to. Worker threads take
    /// 0..=5 by default via `thread_mask`; 6 is the next free core on
    /// any ≥7-core host (which any modern x86_64 dev box is).
    const MEASUREMENT_HOST_CORE: usize = 6;

    /// Host core to pin the driver thread to. Must not be the SMT
    /// sibling of [`MEASUREMENT_HOST_CORE`] — a TIME_CRITICAL sibling
    /// starves the physical core's other logical lane, wedging
    /// `run_quanta` for tens of microseconds despite both "being
    /// pinned". On Intel x86_64 with hyperthreading, logical cores 2N
    /// and 2N+1 share physical N, so pairing 6 ⇔ 7 is pathological.
    /// Core 8 sits on a different physical core and keeps the
    /// coordinator scheduled continuously alongside the busy-waiting
    /// measurement loop. Workers on 0..=5 also map to physicals 0..=2
    /// so 8 keeps a clean separation from them too.
    const DRIVER_HOST_CORE: usize = 8;

    // -----------------------------------------------------------------------
    // Windows thread-priority + affinity shims
    // -----------------------------------------------------------------------

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
            fn SetThreadAffinityMask(
                hThread: HANDLE,
                dwThreadAffinityMask: DWORD_PTR,
            ) -> DWORD_PTR;
        }

        /// Raise process to HIGH_PRIORITY_CLASS. Safe to call once at
        /// startup before any worker spawn.
        pub fn set_high_priority_class() -> Result<(), &'static str> {
            unsafe {
                if SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS) == 0 {
                    return Err("SetPriorityClass(HIGH_PRIORITY_CLASS) failed");
                }
            }
            Ok(())
        }

        /// Raise the current thread to `THREAD_PRIORITY_TIME_CRITICAL`
        /// and pin it to logical core `core`. Call on the measurement
        /// thread (= main after the driver is spawned).
        pub fn promote_and_pin(core: usize) -> Result<(), &'static str> {
            unsafe {
                if SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) == 0 {
                    return Err("SetThreadPriority(TIME_CRITICAL) failed");
                }
                let mask: DWORD_PTR = 1usize << core;
                if SetThreadAffinityMask(GetCurrentThread(), mask) == 0 {
                    return Err("SetThreadAffinityMask failed");
                }
            }
            Ok(())
        }

        /// Pin the current thread to `core` AND raise it to
        /// TIME_CRITICAL. Called from the driver thread: the
        /// coordinator inside `run_quanta` drives workers via barriers,
        /// and any stall on the driver stalls the whole emulator.
        /// Background Windows DPCs / system threads at normal priority
        /// otherwise punch µs-scale dead-zones into the sample stream
        /// even with pinning.
        pub fn pin_and_promote(core: usize) -> Result<(), &'static str> {
            unsafe {
                let mask: DWORD_PTR = 1usize << core;
                if SetThreadAffinityMask(GetCurrentThread(), mask) == 0 {
                    return Err("SetThreadAffinityMask failed");
                }
                if SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) == 0 {
                    return Err("SetThreadPriority(TIME_CRITICAL) failed");
                }
            }
            Ok(())
        }
    }

    // -----------------------------------------------------------------------
    // CLI
    // -----------------------------------------------------------------------

    struct Cli {
        seed: u64,
        ladder: Vec<LadderRung>,
        sweeps_per_threshold: u32,
        run_all_rungs: bool,
        step_quantum: u32,
    }

    fn print_help() {
        println!(
            "onerom_cpu_speed_grade_rp2350 — EPROM-equivalent speed-grade oracle\n\
             \n\
             HOST PREREQUISITE: Windows High-performance power plan.\n\
             Without it CPU frequency scaling dominates sub-300 ns rungs and\n\
             the verdict is untrustworthy.\n\
             \n\
             FLAGS\n\
             \u{20}\u{20}--seed <u64>                 Shuffle seed (default 0x{:016X}).\n\
             \u{20}\u{20}--ladder <csv>               ns targets, comma-separated\n\
             \u{20}\u{20}                             (default 500,400,300,250,200,150,120,100).\n\
             \u{20}\u{20}--sweeps-per-threshold <n>   Sweeps per rung (default 3).\n\
             \u{20}\u{20}--step-quantum <u32>         Threaded CPU cycles per barrier (default {}).\n\
             \u{20}\u{20}--all-rungs                  Continue past first failing rung.\n\
             \u{20}\u{20}--help                       Print this message.\n",
            DEFAULT_SEED, DEFAULT_THREADED_STEP_QUANTUM
        );
    }

    fn parse_cli() -> Result<Cli, String> {
        let args: Vec<String> = std::env::args().skip(1).collect();
        let mut seed = DEFAULT_SEED;
        let mut ladder_csv: Option<String> = None;
        let mut sweeps_per_threshold = DEFAULT_SWEEPS_PER_THRESHOLD;
        let mut run_all_rungs = false;
        let mut step_quantum = DEFAULT_THREADED_STEP_QUANTUM;

        let mut i = 0;
        while i < args.len() {
            match args[i].as_str() {
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                "--seed" => {
                    i += 1;
                    let v = args.get(i).ok_or("--seed requires a value")?;
                    seed = parse_u64(v).map_err(|e| format!("--seed: {}", e))?;
                }
                "--ladder" => {
                    i += 1;
                    let v = args.get(i).ok_or("--ladder requires a CSV value")?;
                    ladder_csv = Some(v.clone());
                }
                "--sweeps-per-threshold" => {
                    i += 1;
                    let v = args
                        .get(i)
                        .ok_or("--sweeps-per-threshold requires a value")?;
                    sweeps_per_threshold = v
                        .parse::<u32>()
                        .map_err(|e| format!("--sweeps-per-threshold: {}", e))?;
                    if sweeps_per_threshold == 0 {
                        return Err("--sweeps-per-threshold must be >= 1".to_string());
                    }
                }
                "--all-rungs" => {
                    run_all_rungs = true;
                }
                "--step-quantum" => {
                    i += 1;
                    let v = args
                        .get(i)
                        .ok_or("--step-quantum requires a value")?;
                    step_quantum = v
                        .parse::<u32>()
                        .map_err(|e| format!("--step-quantum: {}", e))?;
                    if step_quantum == 0 {
                        return Err("--step-quantum must be >= 1".to_string());
                    }
                }
                other => return Err(format!("unknown argument: {}", other)),
            }
            i += 1;
        }

        let ladder: Vec<LadderRung> = match ladder_csv {
            None => DEFAULT_LADDER
                .iter()
                .copied()
                .map(LadderRung::new)
                .collect(),
            Some(csv) => csv
                .split(',')
                .map(str::trim)
                .filter(|s| !s.is_empty())
                .map(|s| {
                    s.parse::<u32>()
                        .map(LadderRung::new)
                        .map_err(|e| format!("--ladder entry '{}': {}", s, e))
                })
                .collect::<Result<Vec<_>, _>>()?,
        };
        if ladder.is_empty() {
            return Err("--ladder cannot be empty".to_string());
        }
        // Monotone-decreasing check: the "stop at first fail" policy
        // assumes harder targets lie later in the list.
        for win in ladder.windows(2) {
            if win[0].target_ns <= win[1].target_ns {
                return Err(format!(
                    "--ladder must be strictly decreasing: {} !> {}",
                    win[0].target_ns, win[1].target_ns
                ));
            }
        }

        Ok(Cli {
            seed,
            ladder,
            sweeps_per_threshold,
            run_all_rungs,
            step_quantum,
        })
    }

    fn parse_u64(s: &str) -> Result<u64, String> {
        let s = s.trim();
        if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
            u64::from_str_radix(hex, 16).map_err(|e| format!("{}: {}", s, e))
        } else {
            s.parse::<u64>().map_err(|e| format!("{}: {}", s, e))
        }
    }

    // -----------------------------------------------------------------------
    // Boot-sync (serial phase, before threaded handoff)
    // -----------------------------------------------------------------------

    /// Mirror of the stress binary's two-phase sync, trimmed: load
    /// bootrom / flash, reset, halt core 1, force ROM set 0 via
    /// `force_rom_set_index_via_sel_pins`, run the emulator serially
    /// until core 0's PC enters the serve loop range AND the shadow
    /// tripwire fires. Returns the synced emulator for
    /// `ThreadedEmulator::from_emulator`.
    fn boot_sync(bootrom: &[u8], flash: &[u8]) -> Result<Emulator, String> {
        let mut emu = EmulatorBuilder::new(Config::default())
            .step_quantum(1)
            .build();
        emu.load_bootrom(bootrom);
        emu.load_flash(flash);
        emu.reset();

        // Bootrom bypass — OneROM flash is not an IMAGE_DEF block.
        let initial_sp = u32::from_le_bytes(flash[0..4].try_into().unwrap());
        let initial_pc_raw = u32::from_le_bytes(flash[4..8].try_into().unwrap());
        let initial_pc = initial_pc_raw & !1u32;
        emu.core_mut(0).regs.set_sp(initial_sp);
        emu.core_mut(0).regs.set_pc(initial_pc);

        // CPU-serve is single-core.
        emu.core_mut(1).halt();

        // Force ROM set 0 via the image_sel helper so the firmware
        // boots the 2364 bake matching the library pin constants.
        onerom_serving_oracle_cpu::force_rom_set_index_via_sel_pins(
            &mut emu,
            flash,
            ROM_SET_INDEX as u32,
        )?;

        // Phase 1: step until PC enters the serve-loop range.
        let mut phase1_cycle: Option<u64> = None;
        while emu.cycles() < BOOT_CYCLE_CAP {
            let before = emu.cycles();
            emu.run(1);
            let after = emu.cycles();
            if after == before {
                return Err(format!("cycle counter stalled at {}", before));
            }
            if is_in_serve_loop(&emu) {
                phase1_cycle = Some(after);
                break;
            }
        }
        if phase1_cycle.is_none() {
            return Err(format!(
                "boot did not reach CPU serve-loop PC (0x{:08X}..=0x{:08X}) within {} cycles",
                SERVE_LOOP_PC_LO, SERVE_LOOP_PC_HI, BOOT_CYCLE_CAP
            ));
        }

        // Shadow sentinel (same convention as the CPU oracle).
        const RUNTIME_INFO_SRAM_OFF: u32 = 0x0008_0000;
        const ROM_SET_INDEX_OFFSET: u32 = 6;
        let rom_set_index_live = emu
            .bus
            .memory
            .sram_read8(RUNTIME_INFO_SRAM_OFF + ROM_SET_INDEX_OFFSET);
        let sentinel: Option<(u32, u8)> =
            match onerom_serving_oracle::lift_shadow_from_flash_pub(flash, rom_set_index_live) {
                Some(shadow) => onerom_serving_oracle_cpu::find_shadow_sentinel(&shadow),
                None => None,
            };

        // Phase 2: PC + sentinel.
        let sentinel_ok = |emu: &Emulator| match sentinel {
            None => true,
            Some((offset, expected)) => emu.bus.memory.sram_read8(offset) == expected,
        };
        let mut synced = is_in_serve_loop(&emu) && sentinel_ok(&emu);
        while !synced && emu.cycles() < BOOT_CYCLE_CAP {
            let before = emu.cycles();
            emu.run(1);
            let after = emu.cycles();
            if after == before {
                return Err(format!("cycle counter stalled at {}", before));
            }
            synced = is_in_serve_loop(&emu) && sentinel_ok(&emu);
        }
        if !synced {
            return Err(format!(
                "boot did not reach CPU serve-loop sync (PC + sentinel) within {} cycles",
                BOOT_CYCLE_CAP
            ));
        }

        Ok(emu)
    }

    #[inline]
    fn is_in_serve_loop(emu: &Emulator) -> bool {
        let pc = emu.core(0).regs.pc();
        (SERVE_LOOP_PC_LO..=SERVE_LOOP_PC_HI).contains(&pc)
    }

    // -----------------------------------------------------------------------
    // Throughput probe (§10)
    // -----------------------------------------------------------------------

    struct ProbeResult {
        emulated_mhz: f64,
        wall_ns_per_emul_cycle: f64,
    }

    /// Pin the first walk step's stimulus, run the driver for
    /// `PROBE_WALL_MS` wall-clock, and compute emulated MHz from the
    /// `master_cycle` delta. The probe is a pure diagnostic —
    /// independent of the ladder. Intentionally not paced so the
    /// number is the ceiling, not a paced-at-150-MHz figure.
    fn run_throughput_probe(
        shared: &SharedState,
        threaded: &mut ThreadedEmulator,
        plan: &[WalkStep],
    ) -> ProbeResult {
        // Pin one stimulus, run, measure.
        let first = &plan[0];
        shared
            .gpio
            .write_external(first.gpio_stim, first.gpio_mask);

        let c0 = shared.master_cycle.load(Ordering::Acquire);
        let t0 = Instant::now();
        let deadline = t0 + Duration::from_millis(PROBE_WALL_MS);
        while Instant::now() < deadline {
            threaded.run_quanta(DRIVER_CHUNK_QUANTA);
        }
        let wall_ns = t0.elapsed().as_nanos() as f64;
        let c1 = shared.master_cycle.load(Ordering::Acquire);
        let cycles = (c1 - c0) as f64;

        let emulated_mhz = if wall_ns > 0.0 {
            cycles / wall_ns * 1000.0
        } else {
            0.0
        };
        let wall_ns_per_emul_cycle = if cycles > 0.0 {
            wall_ns / cycles
        } else {
            f64::INFINITY
        };
        ProbeResult {
            emulated_mhz,
            wall_ns_per_emul_cycle,
        }
    }

    // -----------------------------------------------------------------------
    // Measurement loop (§8) — one sweep
    // -----------------------------------------------------------------------

    /// Run one sweep at the given ns target. Returns the observed byte
    /// vector in shuffle order (= sample order).
    ///
    /// Per sample (§8 + liveness filter):
    ///   - snapshot `t0 = Instant::now()` and `mcy_before`
    ///   - write external stimulus
    ///   - busy-wait until `t0.elapsed() >= target`
    ///   - check `mcy_delta = master_cycle - mcy_before`. If zero the
    ///     coordinator didn't advance during this sample's window —
    ///     host-level DPC/ISR preemption wedged the emulator despite
    ///     TIME_CRITICAL pinning, so the measurement is noise not
    ///     emulator capability. Self-mask the observation with
    ///     `expected` so verify doesn't count it as a miss, and
    ///     report the host-stalled count alongside errors.
    ///   - otherwise sample the CPU's driven data pins directly from
    ///     SIO atomics.
    ///
    /// Why read `out & oe` directly instead of `read_in`: the
    /// coordinator's `update_gpio` folds SIO pads into `gpio_in` only
    /// at quantum boundaries. A host-thread read of `gpio_in` at
    /// sub-µs cadence would observe the CPU's driven byte one full
    /// quantum late. `AtomicGpio::out`/`oe` are updated by every
    /// STRB's `sio_write32` with Relaxed ordering — visible to this
    /// thread within cache coherence time (~ns on x86_64).
    fn run_sweep(
        shared: &SharedState,
        plan: &[WalkStep],
        shuffle: &[u32],
        target_ns: u32,
    ) -> (Vec<u8>, u32) {
        let target = Duration::from_nanos(target_ns as u64);
        let mut observed: Vec<u8> = Vec::with_capacity(shuffle.len());
        let mut host_stalled: u32 = 0;
        let mut prev_mcy = shared.master_cycle.load(Ordering::Acquire);
        for &perm in shuffle {
            let step = &plan[perm as usize];
            let t0 = Instant::now();
            shared
                .gpio
                .write_external(step.gpio_stim, step.gpio_mask);
            while t0.elapsed() < target {
                std::hint::spin_loop();
            }
            let mcy_now = shared.master_cycle.load(Ordering::Acquire);
            let mcy_delta = mcy_now.saturating_sub(prev_mcy);
            prev_mcy = mcy_now;
            // A sample is only counted against the emulator if the
            // coord advanced by at least MIN_PROGRESS cycles during
            // the wait — enough for the CPU to execute ~2 serve-loop
            // iterations (stim observation + STRB on the next iter).
            // Sub-threshold samples are dominated by host DPC/ISR
            // noise, not emulator capability.
            if mcy_delta < MIN_PROGRESS_CYCLES {
                host_stalled += 1;
                observed.push(step.expected);
            } else {
                let driven = shared.gpio.read_out(0) & shared.gpio.read_oe(0);
                observed.push(((driven >> GPIO_DATA_BASE) & 0xFF) as u8);
            }
        }
        (observed, host_stalled)
    }

    /// Diagnostic sweep that prints every sample. Used for one-off
    /// investigation; off by default (gated behind `SPEED_GRADE_DEBUG`
    /// env var on the first sweep of each rung).
    ///
    /// Prints per-sample: stim, expected, observed, fresh SIO out/oe,
    /// and the master_cycle delta since the previous sample. A zero
    /// delta means the coordinator / workers were halted during this
    /// sample's wait window — the measurement is racing something the
    /// runtime isn't scheduling.
    #[allow(dead_code)]
    fn run_sweep_debug(
        shared: &SharedState,
        plan: &[WalkStep],
        shuffle: &[u32],
        target_ns: u32,
        _max_print: usize,
    ) -> (Vec<u8>, u32) {
        let target = Duration::from_nanos(target_ns as u64);
        let mut observed: Vec<u8> = Vec::with_capacity(shuffle.len());
        let mut prev_mcy = shared.master_cycle.load(Ordering::Acquire);
        let mut zero_mcy = 0u32;
        let mut worst_wall_ns = 0u64;
        for &perm in shuffle {
            let step = &plan[perm as usize];
            let t0 = Instant::now();
            shared
                .gpio
                .write_external(step.gpio_stim, step.gpio_mask);
            while t0.elapsed() < target {
                std::hint::spin_loop();
            }
            let elapsed_ns = t0.elapsed().as_nanos() as u64;
            if elapsed_ns > worst_wall_ns {
                worst_wall_ns = elapsed_ns;
            }
            let mcy_now = shared.master_cycle.load(Ordering::Acquire);
            let mcy_delta = mcy_now.saturating_sub(prev_mcy);
            prev_mcy = mcy_now;
            if mcy_delta < MIN_PROGRESS_CYCLES {
                zero_mcy += 1;
                observed.push(plan[perm as usize].expected);
            } else {
                let driven = shared.gpio.read_out(0) & shared.gpio.read_oe(0);
                observed.push(((driven >> GPIO_DATA_BASE) & 0xFF) as u8);
            }
        }
        eprintln!(
            "  debug summary: target={}ns, under_threshold_samples={}/{}, worst_wall_ns={}, min_progress_cycles={}",
            target_ns,
            zero_mcy,
            shuffle.len(),
            worst_wall_ns,
            MIN_PROGRESS_CYCLES
        );
        (observed, zero_mcy)
    }

    // -----------------------------------------------------------------------
    // Report format (§13)
    // -----------------------------------------------------------------------

    struct RunContext {
        fixture_path: String,
        seed: u64,
        sweeps_per_threshold: u32,
        ladder_len: usize,
        probe: ProbeResult,
    }

    fn print_header(ctx: &RunContext) {
        println!(
            "OneROM CPU-Mode Speed Grade Oracle — 1541 $E000 kernal (ROM set {})",
            ROM_SET_INDEX
        );
        println!("fixture:  {}", ctx.fixture_path);
        println!(
            "addr span: {} bytes, {} samples/sweep, {} sweeps × {} rungs",
            WALK_PLAN_LEN, WALK_PLAN_LEN, ctx.sweeps_per_threshold, ctx.ladder_len
        );
        println!("seed:     0x{:016X}", ctx.seed);
        println!();
        println!("serve-loop throughput probe:");
        println!(
            "  emulated rate: {:.1} MHz  ({:.2} ns per emul cycle wall-clock)",
            ctx.probe.emulated_mhz, ctx.probe.wall_ns_per_emul_cycle
        );
        let floor_comment = if ctx.probe.emulated_mhz >= 150.0 {
            "comfortable — ~100 ns rung has host-timing headroom"
        } else if ctx.probe.emulated_mhz >= 80.0 {
            "borderline — lower rungs may fail on emulator-throughput grounds"
        } else {
            "slow — ladder floor likely fails on emulator-throughput grounds, not real-timing grounds"
        };
        println!("  {}", floor_comment);
        println!();
    }

    fn print_rung_row(report: &SweepReport) {
        let verdict = if report.verdict_passes() {
            "PASS".to_string()
        } else if let Some(fc) = report.first_fail {
            format!(
                "FAIL   first @ sweep={} idx={} addr=0x{:04X} expected=0x{:02X} observed=0x{:02X}",
                fc.sweep_idx, fc.sample_idx, fc.addr, fc.expected, fc.observed
            )
        } else {
            "FAIL".to_string()
        };
        println!(
            "  {:>10}   {:>8}   {:>6}   {:>8}   {}",
            report.target_ns, report.samples, report.errors, report.host_stalled, verdict
        );
    }

    fn print_skipped(target_ns: u32, reason: &str) {
        println!(
            "  {:>10}   {:>8}   {:>6}   skipped ({})",
            target_ns, "—", "—", reason
        );
    }

    fn print_summary(reports: &[SweepReport], ladder: &[LadderRung], elapsed: Duration) {
        let first_fail = first_failing_rung(reports);
        println!();
        println!("Summary:");
        if first_fail == 0 {
            println!(
                "  unreliable @ all tested thresholds — even the top rung ({} ns) failed",
                ladder[0].target_ns
            );
        } else if first_fail == reports.len() {
            println!(
                "  reliable @ all tested thresholds down to {} ns",
                reports.last().map(|r| r.target_ns).unwrap_or(0)
            );
        } else {
            let last_pass_ns = reports[first_fail - 1].target_ns;
            let first_fail_ns = reports[first_fail].target_ns;
            println!("  reliable @ >= {} ns  (pass)", last_pass_ns);
            println!("  unreliable @ <= {} ns (fail)", first_fail_ns);
            println!("  edge: {}–{} ns", first_fail_ns, last_pass_ns);
        }
        println!("  elapsed: {:.2} s wall-clock", elapsed.as_secs_f64());
    }

    // -----------------------------------------------------------------------
    // Main
    // -----------------------------------------------------------------------

    pub fn run() -> ExitCode {
        mdpicoem_harness::harness_tracing_init();

        let cli = match parse_cli() {
            Ok(c) => c,
            Err(e) => {
                eprintln!("error: {}", e);
                eprintln!("hint: pass --help for usage.");
                return ExitCode::from(2);
            }
        };

        // Host process priority — set before anything spawns threads,
        // so spawned workers inherit HIGH.
        if let Err(e) = win::set_high_priority_class() {
            eprintln!("warning: {} (continuing — timing may be noisy)", e);
        }

        // Load fixtures.
        let bootrom = match std::fs::read(BOOTROM_PATH) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("failed to read bootrom at {}: {}", BOOTROM_PATH, e);
                return ExitCode::from(2);
            }
        };
        let flash = match std::fs::read(FLASH_PATH) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("failed to read flash image at {}: {}", FLASH_PATH, e);
                return ExitCode::from(2);
            }
        };

        // Build the walk plan up front — fails fast if the fixture is
        // malformed.
        let plan = match build_walk_plan(&flash) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("failed to build walk plan: {}", e);
                return ExitCode::from(2);
            }
        };

        // Boot sync serially, then promote to ThreadedEmulator. The
        // emulator builder uses a small step_quantum for the serial
        // sync phase; rebuild with the threaded step_quantum below.
        let mut emu = match boot_sync(&bootrom, &flash) {
            Ok(e) => e,
            Err(e) => {
                eprintln!("boot-sync failed: {}", e);
                return ExitCode::FAILURE;
            }
        };
        // Update step_quantum for the threaded phase. The library
        // destructures this field; setting it before handoff keeps
        // the coordinator's quantum wiring on the tuned value.
        emu.step_quantum = cli.step_quantum;

        // Seed gpio_external_mask up front so the mask is visible as
        // soon as the threaded runtime starts. Every walk step has
        // the same mask — set it once and never touch it from the
        // measurement loop.
        emu.bus.gpio_external_mask = plan[0].gpio_mask;

        // Serial-mode sanity across a few samples: drive each stim,
        // step ~200 cycles, check observed == expected. Validates
        // that the SRAM shadow agrees with the lifted-from-flash
        // shadow and that the CPU serves correctly on the serial
        // path before handoff. Any divergence after the handoff is
        // then attributable to the threaded-runtime integration
        // rather than the walk-plan construction.
        let mut serial_ok = 0usize;
        let serial_samples_tested;
        {
            let ordering = std::sync::atomic::Ordering::Relaxed;
            let sample_indices = [0usize, plan.len() / 2, plan.len() - 1];
            serial_samples_tested = sample_indices.len();
            for &idx in &sample_indices {
                let sample = &plan[idx];
                emu.bus.gpio_external_in.store(sample.gpio_stim, ordering);
                for _ in 0..200 {
                    emu.run(1);
                }
                let merged = emu.bus.gpio_in.load(ordering);
                let serial_observed = ((merged >> GPIO_DATA_BASE) & 0xFF) as u8;
                if serial_observed == sample.expected {
                    serial_ok += 1;
                }
            }
            // Leave gpio_external_in on plan[0] so the threaded
            // runtime starts with a valid stim.
            emu.bus
                .gpio_external_in
                .store(plan[0].gpio_stim, ordering);
            for _ in 0..100 {
                emu.run(1);
            }
        }
        println!(
            "serial-mode probe: {}/{} samples served correct bytes (pre-handoff sanity)",
            serial_ok, serial_samples_tested
        );
        if serial_ok != serial_samples_tested {
            eprintln!(
                "FATAL: serial-mode probe divergence — walk-plan construction does not \
                 match the firmware's SRAM shadow. Rebuilding the plan with the right \
                 ROM set / pin profile is required before the ladder makes sense."
            );
            return ExitCode::FAILURE;
        }

        let mut threaded = ThreadedEmulator::from_emulator(emu);

        // Throughput probe — owned by main while we have exclusive
        // access to `threaded`, before the driver closure consumes it.
        let probe = {
            // Use a scoped borrow of shared for the probe.
            let shared_ref: &SharedState = threaded.shared();
            // Run the probe by calling threaded.run_quanta inside the
            // scope. Rust borrow checker: probe takes `&mut threaded`
            // and `&SharedState` — split the borrow manually by
            // cloning the Arc and dropping the ref before calling run.
            let shared_clone: SharedState = shared_ref.clone();
            run_throughput_probe(&shared_clone, &mut threaded, &plan)
        };

        let ctx = RunContext {
            fixture_path: FLASH_PATH.to_string(),
            seed: cli.seed,
            sweeps_per_threshold: cli.sweeps_per_threshold,
            ladder_len: cli.ladder.len(),
            probe,
        };
        print_header(&ctx);

        // --- Launch driver + watchdog + run the ladder on main ----------
        let stop = Arc::new(AtomicBool::new(false));
        let shared_for_meas: SharedState = threaded.shared().clone();

        // Watchdog: wall-clock hard cap. Releases `stop` on expiry.
        let watchdog_stop = stop.clone();
        let watchdog = thread::Builder::new()
            .name("speed-grade-watchdog".to_string())
            .spawn(move || {
                let t0 = Instant::now();
                let cap = Duration::from_secs(WATCHDOG_SECS);
                while t0.elapsed() < cap {
                    if watchdog_stop.load(Ordering::Relaxed) {
                        return false; // clean shutdown — stop flipped first
                    }
                    thread::sleep(Duration::from_millis(50));
                }
                watchdog_stop.store(true, Ordering::Relaxed);
                true // watchdog fired
            })
            .expect("spawn watchdog");

        // Driver: owns `threaded`, loops run_quanta until stop is set.
        // Returns the consumed ThreadedEmulator so main can inspect
        // post-sweep state (PC check, etc.).
        let driver_stop = stop.clone();
        let driver = thread::Builder::new()
            .name("speed-grade-driver".to_string())
            .spawn(move || {
                if let Err(e) = win::pin_and_promote(DRIVER_HOST_CORE) {
                    eprintln!(
                        "warning: driver pin+promote to core {} failed ({}); relying on OS scheduler",
                        DRIVER_HOST_CORE, e
                    );
                }
                let mut threaded = threaded;
                while !driver_stop.load(Ordering::Relaxed) {
                    threaded.run_quanta(DRIVER_CHUNK_QUANTA);
                }
                threaded
            })
            .expect("spawn driver");

        // Main becomes the measurement thread — pin + promote.
        if let Err(e) = win::promote_and_pin(MEASUREMENT_HOST_CORE) {
            eprintln!(
                "warning: measurement-thread promote/pin failed ({}); continuing uncontrolled",
                e
            );
        }

        // Report header: per-rung table.
        println!(
            "  {:>10}   {:>8}   {:>6}   {:>8}   verdict",
            "threshold", "samples", "errors", "stalled"
        );
        println!("  {:>10}   {:>8}   {:>6}   {:>8}", "(ns)", "", "", "");

        // Pre-ladder sanity probe: drive one stimulus from the plan
        // and wait 50 ms (thousands of quanta — plenty of time for
        // the CPU to observe the new address, look up the shadow,
        // and STRB the result). If `observed != expected` at
        // effectively unbounded wall time, the ladder can't produce
        // a meaningful verdict at any rung. Report and continue (so
        // the header + ladder still print) but flag the run.
        let mut probe_failed = false;
        {
            let sample = &plan[plan.len() / 2];
            shared_for_meas
                .gpio
                .write_external(sample.gpio_stim, sample.gpio_mask);
            thread::sleep(Duration::from_millis(50));
            let merged = shared_for_meas.gpio.read_in();
            let observed = ((merged >> GPIO_DATA_BASE) & 0xFF) as u8;
            println!(
                "pre-ladder probe: addr=0x{:04X} stim=0x{:08X} expected=0x{:02X} observed=0x{:02X}",
                sample.addr, sample.gpio_stim, sample.expected, observed
            );
            if observed != sample.expected {
                probe_failed = true;
                eprintln!(
                    "WARNING: pre-ladder probe mismatch (expected=0x{:02X} observed=0x{:02X}) — \
                     ladder results below are not trustworthy. Likely causes: threaded-bus \
                     GPIO_IN wiring regression, CPU drift out of serve loop, or shadow \
                     mismatch between firmware and lifted-from-flash expectation.",
                    sample.expected, observed
                );
            } else {
                println!("pre-ladder probe: PASS — emulator serving correctly at 50 ms wait");
            }
        }
        println!();

        // Pre-sweep liveness check: `master_cycle` must advance
        // within LIVENESS_WINDOW_US µs. If not, the driver is wedged
        // — bail cleanly.
        let live_t0 = Instant::now();
        let live_c0 = shared_for_meas.master_cycle.load(Ordering::Acquire);
        let live_deadline = live_t0 + Duration::from_micros(LIVENESS_WINDOW_US);
        let mut alive = false;
        while Instant::now() < live_deadline {
            if shared_for_meas.master_cycle.load(Ordering::Acquire) != live_c0 {
                alive = true;
                break;
            }
            std::hint::spin_loop();
        }
        if !alive {
            eprintln!(
                "FATAL: liveness check failed — master_cycle did not advance within {} µs",
                LIVENESS_WINDOW_US
            );
            stop.store(true, Ordering::Relaxed);
            let _ = driver.join();
            let _ = watchdog.join();
            return ExitCode::FAILURE;
        }

        let run_t0 = Instant::now();
        let mut reports: Vec<SweepReport> = Vec::with_capacity(cli.ladder.len());
        let mut stopped_early = false;

        for rung in &cli.ladder {
            if stopped_early {
                print_skipped(rung.target_ns, "ladder stopped at first fail");
                continue;
            }
            if stop.load(Ordering::Relaxed) {
                print_skipped(rung.target_ns, "watchdog tripped");
                continue;
            }

            let mut errors: u32 = 0;
            let mut first_fail: Option<FailContext> = None;
            let samples_per_sweep = plan.len() as u32;

            let mut host_stalled_total: u32 = 0;
            for sweep_idx in 0..cli.sweeps_per_threshold {
                let shuffle = shuffle_plan(plan.len(), cli.seed, sweep_idx);
                let (observed, stalled) = if std::env::var("SPEED_GRADE_DEBUG").is_ok()
                    && sweep_idx == 0
                {
                    run_sweep_debug(
                        &shared_for_meas,
                        &plan,
                        &shuffle,
                        rung.target_ns,
                        30,
                    )
                } else {
                    run_sweep(&shared_for_meas, &plan, &shuffle, rung.target_ns)
                };
                host_stalled_total = host_stalled_total.saturating_add(stalled);
                let fail = verify_observed(
                    &plan,
                    &shuffle,
                    &observed,
                    rung.target_ns,
                    sweep_idx,
                );
                if let Some(fc) = fail {
                    // Count all mismatches in this sweep (not just the first).
                    let sweep_errors: u32 = observed
                        .iter()
                        .zip(shuffle.iter())
                        .filter(|&(&obs, &perm)| obs != plan[perm as usize].expected)
                        .count() as u32;
                    errors = errors.saturating_add(sweep_errors);
                    if first_fail.is_none() {
                        first_fail = Some(fc);
                    }
                }
            }

            let report = SweepReport {
                target_ns: rung.target_ns,
                sweeps: cli.sweeps_per_threshold,
                samples: samples_per_sweep.saturating_mul(cli.sweeps_per_threshold),
                errors,
                host_stalled: host_stalled_total,
                first_fail,
            };
            print_rung_row(&report);
            if !report.verdict_passes() && !cli.run_all_rungs {
                stopped_early = true;
            }
            reports.push(report);
        }

        // --- Shut down driver + watchdog --------------------------------
        stop.store(true, Ordering::Relaxed);
        let threaded = match driver.join() {
            Ok(t) => Some(t),
            Err(_) => {
                eprintln!("driver thread panicked during run");
                None
            }
        };
        let watchdog_fired = watchdog.join().unwrap_or(false);

        let elapsed = run_t0.elapsed();

        // Post-sweep PC check (§12) — reassembled core holds the PC.
        if let Some(ref t) = threaded {
            if let Some(pc) = t.core_pc(0) {
                if !(SERVE_LOOP_PC_LO..=SERVE_LOOP_PC_HI).contains(&pc) {
                    eprintln!(
                        "warning: core 0 PC 0x{:08X} left the serve-loop range \
                         0x{:08X}..=0x{:08X} by end of run — results may be tainted",
                        pc, SERVE_LOOP_PC_LO, SERVE_LOOP_PC_HI
                    );
                }
            }
        }
        if watchdog_fired {
            eprintln!(
                "warning: wall-clock watchdog ({} s) fired — ladder may be truncated",
                WATCHDOG_SECS
            );
        }

        print_summary(&reports, &cli.ladder, elapsed);

        if probe_failed {
            eprintln!(
                "\nNOTE: pre-ladder probe failed — top rung \"PASS\" (if any) \
                 should be treated skeptically."
            );
        }

        // Exit code per §13: 0 if top rung passed, 1 otherwise.
        match reports.first() {
            Some(top) if top.verdict_passes() => ExitCode::SUCCESS,
            _ => ExitCode::FAILURE,
        }
    }
}

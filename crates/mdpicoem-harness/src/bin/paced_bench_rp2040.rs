//! Paced benchmark: measures whether the mdrp2040 emulator can sustain
//! real-time at the RP2040's stock 125 MHz.
//!
//! Mirrors `paced_bench_rp2350` minus the FPU workload — the M0+ has no
//! FPU and no coprocessors. The `--workload` flag picks from a set of
//! synthetic workloads that span the cost envelope from single-core
//! ALU floor to "both cores active + peripherals singing" realistic
//! worst case. See `wrk_docs/2026.04.15 - HLD - Paced Bench Workload
//! Spread.md` for rationale.
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
//!   --workload <name>  One of: basic (default), peripheral, contention, stress.
//!                      Core count is implied by the workload: basic and
//!                      peripheral are single-core, contention and stress are
//!                      dual-core.

use mdrp2040::{Config, Emulator, Pacer, PacerStats};
use std::sync::Arc;
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Workload selection
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Workload {
    /// Baseline: single-core ALU loop in striped SRAM.
    Basic,
    /// Single-core ALU + SIO GPIO toggle, with PIO0 SM0 blinking a pin
    /// in the background (keeps `PioBlock::execute_cycle` on the hot
    /// path).
    Peripheral,
    /// Both cores running the ALU loop in striped SRAM bank 0 (16-byte
    /// apart) — exercises the +1-cycle contention accounting on core 1.
    Contention,
    /// Composite: core 0 runs the peripheral loop, core 1 runs the basic
    /// ALU loop, both in bank 0; PIO0 SM0 running. Realistic worst case.
    Stress,
}

impl Workload {
    fn is_dual_core(self) -> bool {
        matches!(self, Workload::Contention | Workload::Stress)
    }

    fn needs_pio(self) -> bool {
        matches!(self, Workload::Peripheral | Workload::Stress)
    }

    fn as_str(self) -> &'static str {
        match self {
            Workload::Basic => "basic",
            Workload::Peripheral => "peripheral",
            Workload::Contention => "contention",
            Workload::Stress => "stress",
        }
    }
}

// ---------------------------------------------------------------------------
// RP2040 MMIO register addresses (RP2040 datasheet §2.2, §2.19, §3).
// ---------------------------------------------------------------------------

const RESETS_BASE: u32 = 0x4000_C000;
const RESETS_RESET_OFFSET: u32 = 0x00;
const RESETS_PIO0_BIT: u32 = 1 << 10; // RESETS.RESET bit 10 = PIO0

const IO_BANK0_BASE: u32 = 0x4001_4000;
const PADS_BANK0_BASE: u32 = 0x4001_C000;

const SIO_BASE: u32 = 0xD000_0000;
const SIO_GPIO_OE_SET: u32 = SIO_BASE + 0x024;
const SIO_GPIO_OUT_XOR: u32 = SIO_BASE + 0x01C;

const PIO0_BASE: u32 = 0x5020_0000;
const PIO_CTRL: u32 = 0x000;
const PIO_INSTR_MEM0: u32 = 0x048;
const PIO_SM0_CLKDIV: u32 = 0x0C8;
const PIO_SM0_EXECCTRL: u32 = 0x0CC;
const PIO_SM0_SHIFTCTRL: u32 = 0x0D0;
const PIO_SM0_INSTR: u32 = 0x0D8;
const PIO_SM0_PINCTRL: u32 = 0x0DC;

/// FUNCSEL for PIO0 (see RP2040 datasheet §2.19.6: 6 = PIO0).
const FUNCSEL_PIO0: u8 = 6;

/// PIO pin number driven by the wrap-loop program.
const PIO_PIN: u8 = 2;
/// SIO GPIO pin toggled by the core's STR in the peripheral loop. Bit 0
/// so the pin mask fits in a `MOVS R1, #1` (T16 immediate), matching the
/// HLD loop shape exactly.
const SIO_TOGGLE_PIN: u8 = 0;

// ---------------------------------------------------------------------------
// Setup helpers
// ---------------------------------------------------------------------------

/// De-assert the RESETS bit for PIO0. The emulator doesn't currently gate
/// PIO activity on the RESETS bit, but matching real firmware bring-up
/// keeps us forward-compatible the day that tech-debt entry gets closed.
fn resets_deassert_pio0(emu: &mut Emulator) {
    // APB CLR alias (+0x3000): writing 1s clears the corresponding RESET
    // bits, bringing those peripherals out of reset.
    emu.mmio_write32(
        RESETS_BASE + RESETS_RESET_OFFSET + 0x3000,
        RESETS_PIO0_BIT,
    );
}

/// Configure a GPIO pin for output on `funcsel`: program the IO_BANK0
/// GPIO_CTRL.FUNCSEL and set the PADS_BANK0 entry so the pad is driven.
fn setup_gpio_output(emu: &mut Emulator, pin: u8, funcsel: u8) {
    // IO_BANK0_GPIO<pin>_CTRL is at offset 0x04 + pin*8 (pair: STATUS+CTRL
    // at 8-byte stride).
    let ctrl_offset = 0x04 + (pin as u32) * 8;
    emu.mmio_write32(IO_BANK0_BASE + ctrl_offset, funcsel as u32);

    // PADS_BANK0_GPIO<pin> is at offset 0x04 + pin*4. Clear the output
    // disable (OD, bit 7) and enable the input (IE, bit 6) matching
    // Pico SDK defaults for an output pin.
    let pad_offset = 0x04 + (pin as u32) * 4;
    // SCHMITT=1 (bit 1), DRIVE=01=4mA (bits 5:4), IE=1 (bit 6);
    // PUE=0, PDE=0, OD=0. SDK-style default for an output pin with the
    // pull-down explicitly disabled.
    emu.mmio_write32(PADS_BANK0_BASE + pad_offset, 0x0000_0052);
}

/// Install a minimal two-instruction wrap loop on PIO0 SM0:
///   addr 0: SET PINS, 1
///   addr 1: SET PINS, 0
///   .wrap (via EXECCTRL wrap_top=1, wrap_bottom=0)
/// Force a `SET PINDIRS, 1` via SM0_INSTR so the pin is driven, set
/// CLKDIV=1 (one SM cycle per sys_clk), and enable SM0 via CTRL.
/// Also configures IO_BANK0 / PADS_BANK0 for the target pin.
fn setup_pio0_sm0_wrap(emu: &mut Emulator, pin: u8) {
    resets_deassert_pio0(emu);

    // Route the pin through PIO0.
    setup_gpio_output(emu, pin, FUNCSEL_PIO0);

    // INSTR_MEM[0]: SET PINS, 1  (0xE001)
    // INSTR_MEM[1]: SET PINS, 0  (0xE000)
    emu.mmio_write32(PIO0_BASE + PIO_INSTR_MEM0, 0xE001);
    emu.mmio_write32(PIO0_BASE + PIO_INSTR_MEM0 + 4, 0xE000);

    // SM0_PINCTRL: SET_COUNT=1 (bits 28:26), SET_BASE=pin (bits 9:5).
    let pinctrl = (1u32 << 26) | ((pin as u32) << 5);
    emu.mmio_write32(PIO0_BASE + PIO_SM0_PINCTRL, pinctrl);

    // SM0_EXECCTRL: wrap_top=1 (bits 16:12), wrap_bottom=0 (bits 11:7).
    // All other bits zero — no JMP pin, no SIDE_EN, STATUS_N=0.
    let execctrl = 1u32 << 12;
    emu.mmio_write32(PIO0_BASE + PIO_SM0_EXECCTRL, execctrl);

    // SM0_SHIFTCTRL: leave at reset (autopush/autopull off, thresholds 32).
    // Reset value on the emulator side is 0x000C_0000; re-emit explicitly
    // so we don't rely on the global reset state.
    emu.mmio_write32(PIO0_BASE + PIO_SM0_SHIFTCTRL, 0x000C_0000);

    // SM0_CLKDIV: INT=1 (bits 31:16), FRAC=0. One SM cycle per sys_clk.
    emu.mmio_write32(PIO0_BASE + PIO_SM0_CLKDIV, 1u32 << 16);

    // Force `SET PINDIRS, 1` through SM0_INSTR so the pin becomes driven.
    // Encoding: opcode=111 (SET), dest=PINDIRS (4), data=1 → 0xE081.
    emu.mmio_write32(PIO0_BASE + PIO_SM0_INSTR, 0xE081);

    // CTRL.SM_ENABLE bit 0 — enable SM0.
    emu.mmio_write32(PIO0_BASE + PIO_CTRL, 0x1);
}

// ---------------------------------------------------------------------------
// Workload dispatch
// ---------------------------------------------------------------------------

/// Core 0 basic ALU loop at 0x2000_0000:
///   MOVS R0, #1 / ADDS R0, R0, #1 / B .-2
fn setup_basic_core0(emu: &mut Emulator) {
    // halfwords[0]=0x2001 MOVS R0,#1 | halfwords[1]=0x1C40 ADDS R0,R0,#1
    emu.poke(0x2000_0000, 0x1C40_2001);
    // halfwords[0]=0xE7FD B .-2 (back to ADDS)
    emu.poke(0x2000_0004, 0x0000_E7FD);

    emu.core_mut(0).regs.set_pc(0x2000_0000);
    emu.core_mut(0).regs.xpsr = 1 << 24;
}

/// Core 0 peripheral loop at 0x2000_0000, matching the HLD shape:
///   prologue: LDR R2, [PC, #lit] | MOVS R1, #1
///   loop:     ADDS R0, R0, #1 | STR R1, [R2] | B loop
///
/// Layout:
///   0x0: LDR  R2, [PC, #8]  (0x4A02) — R2 = SIO_GPIO_OUT_XOR
///   0x2: MOVS R1, #1        (0x2101) — R1 = pin mask (GPIO0)
///   0x4: ADDS R0, R0, #1    (0x1C40) — loop start
///   0x6: STR  R1, [R2]      (0x6011) — SIO XOR write
///   0x8: B    loop          (0xE7FC) — target = 0x4
///   0xA: NOP                (0xBF00) — alignment halfword
///   0xC: .word SIO_GPIO_OUT_XOR
///
/// `LDR R2, [PC, #8]`: PC (as seen by the instruction at 0x0) =
/// instruction_addr + 4 = 0x4, rounded down to word = 0x4. Offset 8 →
/// literal address 0xC. Encoding: `0x4800 | (Rd<<8) | (imm8)` with
/// Rd=2, imm8=2 → `0x4A02`. Three instructions per loop iteration.
fn setup_peripheral_core0(emu: &mut Emulator) {
    emu.poke(0x2000_0000, 0x2101_4A02);           // LDR R2 | MOVS R1
    emu.poke(0x2000_0004, 0x6011_1C40);           // ADDS R0,R0,#1 | STR R1,[R2]
    emu.poke(0x2000_0008, 0xBF00_E7FC);           // B .-8 | NOP
    emu.poke(0x2000_000C, SIO_GPIO_OUT_XOR);      // literal: SIO XOR addr

    // Enter at the prologue (LDR R2).
    emu.core_mut(0).regs.set_pc(0x2000_0000);
    emu.core_mut(0).regs.xpsr = 1 << 24;
}

/// Core 1 basic ALU loop at a caller-supplied address. Uses R1 as the
/// accumulator to differentiate from core 0 in register dumps.
fn setup_basic_core1_at(emu: &mut Emulator, addr: u32) {
    // halfwords[0]=0x2101 MOVS R1,#1 | halfwords[1]=0x1C49 ADDS R1,R1,#1
    emu.poke(addr, 0x1C49_2101);
    // halfwords[0]=0xE7FD B .-2
    emu.poke(addr + 4, 0x0000_E7FD);

    emu.core_mut(1).regs.set_pc(addr);
    emu.core_mut(1).regs.xpsr = 1 << 24;
}

/// Dispatch: set up the emulator for the chosen workload.
fn setup(emu: &mut Emulator, workload: Workload) {
    // Stack placement. Single-core workloads park core 0's stack top at
    // 0x2004_0000 (SRAM3/SRAM4 boundary — first push lands at
    // 0x2003_FFFC in striped SRAM3). Dual-core workloads move core 0's
    // stack top to 0x2004_2000 (top of SRAM5 scratch — first push at
    // 0x2004_1FFC) and give core 1 the 0x2004_1800 mid-point of SRAM5
    // so push/pop traffic stays off the bank-0 fetch-contention signal.
    let core0_stack_top: u32 = if workload.is_dual_core() {
        0x2004_2000
    } else {
        0x2004_0000
    };
    emu.core_mut(0).regs.msp = core0_stack_top;
    emu.core_mut(0).regs.r[13] = core0_stack_top;

    match workload {
        Workload::Basic => {
            setup_basic_core0(emu);
        }
        Workload::Peripheral => {
            setup_peripheral_core0(emu);
            // SIO drives pin 25 directly; PIO handles its own pin
            // (PIO_PIN). Both pins must be routed.
            setup_gpio_output(emu, SIO_TOGGLE_PIN, 5); // FUNCSEL=5 = SIO
            // Set OE bit for SIO pin so the toggle is observable.
            emu.mmio_write32(SIO_GPIO_OE_SET, 1u32 << SIO_TOGGLE_PIN);
            setup_pio0_sm0_wrap(emu, PIO_PIN);
        }
        Workload::Contention => {
            setup_basic_core0(emu);
            // Core 1 in striped bank 0 at 0x2000_0040 (word 16, 16 % 4 = 0).
            setup_basic_core1_at(emu, 0x2000_0040);
            // Core 1 stack in scratch SRAM5 — avoid polluting the bank-0
            // fetch-contention signal with push/pop traffic.
            let core1_stack_top: u32 = 0x2004_1800;
            emu.core_mut(1).regs.msp = core1_stack_top;
            emu.core_mut(1).regs.r[13] = core1_stack_top;
            emu.core_mut(1).wake();
        }
        Workload::Stress => {
            setup_peripheral_core0(emu);
            setup_gpio_output(emu, SIO_TOGGLE_PIN, 5);
            emu.mmio_write32(SIO_GPIO_OE_SET, 1u32 << SIO_TOGGLE_PIN);
            setup_pio0_sm0_wrap(emu, PIO_PIN);

            // Core 1 placed at 0x2000_0044 (word 17 = bank 1) so its
            // two-halfword ALU loop fetches land on core 0's peripheral
            // hot-path banks — core 0's ADDS@0x4 / STR@0x6 both live in
            // bank 1, core 0's B@0x8 in bank 2; core 1's ADDS@0x46 in
            // bank 1 and B@0x48 in bank 2. Contention fires whenever
            // both cores happen to fetch bank 1 or bank 2 on the same
            // cycle. (Core 0's `Contention` workload uses 0x2000_0040
            // — bank 0 — which works because core 0's basic ALU loop is
            // in bank 0. The peripheral shape here spans banks 1 and 2,
            // so stress needs a different offset.)
            setup_basic_core1_at(emu, 0x2000_0044);
            let core1_stack_top: u32 = 0x2004_1800;
            emu.core_mut(1).regs.msp = core1_stack_top;
            emu.core_mut(1).regs.r[13] = core1_stack_top;
            emu.core_mut(1).wake();
        }
    }
}

// ---------------------------------------------------------------------------
// Windows thread-priority + affinity shims
// ---------------------------------------------------------------------------

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
    let workload = parse_workload().unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1);
    });

    // `--dual-core` was removed in the workload-spread refactor: core
    // count is now a property of the workload (Basic/Peripheral → single
    // core; Contention/Stress → dual core). Reject it explicitly so
    // stale scripts get a helpful nudge instead of silently running the
    // wrong workload.
    if std::env::args().any(|a| a == "--dual-core") {
        eprintln!(
            "error: --dual-core has been removed. Use --workload {{contention,stress}} \
             for dual-core workloads."
        );
        std::process::exit(1);
    }

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

    // --- Set up emulator + selected workload ---
    let mut emu = Emulator::new(Config { sys_clk_hz });
    setup(&mut emu, workload);

    // --- Set up pacer ---
    let mut pacer = Pacer::with_quantum(sys_clk_hz, quantum.into());
    let stats = pacer.stats();

    let core_mode = if workload.is_dual_core() { "dual-core" } else { "single-core" };
    let pio_mode = if workload.needs_pio() { " + PIO0 SM0 wrap" } else { "" };
    println!(
        "mdrp2040 paced benchmark — target {} MHz, quantum {} cycles, {}, workload {}{}",
        clock_mhz, quantum, core_mode, workload.as_str(), pio_mode,
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
    println!("Workload:       {}", workload.as_str());

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

fn parse_workload() -> Result<Workload, String> {
    let args: Vec<String> = std::env::args().collect();
    for (i, a) in args.iter().enumerate() {
        if let Some(v) = a.strip_prefix("--workload=") {
            return match_workload(v);
        }
        if a == "--workload" {
            let v = args
                .get(i + 1)
                .ok_or("--workload requires basic|peripheral|contention|stress")?;
            return match_workload(v);
        }
    }
    Ok(Workload::Basic)
}

fn match_workload(s: &str) -> Result<Workload, String> {
    match s {
        "basic" => Ok(Workload::Basic),
        "peripheral" => Ok(Workload::Peripheral),
        "contention" => Ok(Workload::Contention),
        "stress" => Ok(Workload::Stress),
        other => Err(format!(
            "invalid --workload '{other}' (expected basic|peripheral|contention|stress)"
        )),
    }
}

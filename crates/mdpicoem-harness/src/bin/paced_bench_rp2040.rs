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
//!   --step-quantum N   Cycles per emulator step (default 256 — see
//!                      `BENCH_DEFAULT_STEP_QUANTUM`). In threaded mode
//!                      this is the cycles-per-barrier-rendezvous, so
//!                      larger values amortise the 3-thread barrier
//!                      cost. Serial runs honour this too for A/B
//!                      comparability (serial path is insensitive to
//!                      quantum size in this range).
//!   --model <name>     Dual-execution HLD V1 Stage 4 A/B harness.
//!                      One of `serial|threaded|both`. `serial` is the
//!                      default; `threaded` selects
//!                      `ExecutionModel::Threaded`; `both` runs N reps
//!                      on each model and prints a comparative
//!                      Serial-vs-Threaded table with median MHz, IQR,
//!                      delta% and per-workload verdict. `both` implies
//!                      `--unpaced` per HLD V1 §7.3. Paired with
//!                      `--reps`.
//!   --reps N           Replication count for `--model both` (also for
//!                      `--model serial|threaded` stats). Default 5
//!                      (HLD V1 §7.3 minimum). One warm-up rep per
//!                      model is run and discarded; N measured reps
//!                      follow.

use mdrp2040::{
    Config, ConfigError, Emulator, EmulatorBuilder, EmulatorError, ExecutionModel, Pacer,
    PacerStats,
};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Bench CLI default for `--step-quantum`. Diverges from the library
/// `DEFAULT_STEP_QUANTUM = 64` because larger values amortise the
/// threaded-barrier cost over more useful work. Matches the RP2350
/// bench default for A/B comparability across chips. Serial runs
/// honour this too for within-chip Serial-vs-Threaded A/B diffing;
/// the serial path is insensitive to quantum size in this range.
const BENCH_DEFAULT_STEP_QUANTUM: u32 = 256;

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
    emu.mmio_write32(RESETS_BASE + RESETS_RESET_OFFSET + 0x3000, RESETS_PIO0_BIT);
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
    emu.poke(0x2000_0000, 0x2101_4A02); // LDR R2 | MOVS R1
    emu.poke(0x2000_0004, 0x6011_1C40); // ADDS R0,R0,#1 | STR R1,[R2]
    emu.poke(0x2000_0008, 0xBF00_E7FC); // B .-8 | NOP
    emu.poke(0x2000_000C, SIO_GPIO_OUT_XOR); // literal: SIO XOR addr

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

// ---------------------------------------------------------------------------
// Parsed CLI state — shared across the A/B and single-model reps paths
// so each rep is configured identically.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct RunConfig {
    seconds: u32,
    cycles_target: Option<u64>,
    quantum: u32,
    clock_mhz: u32,
    sys_clk_hz: u32,
    unpaced: bool,
    step_quantum: u32,
    workload: Workload,
    model: ExecutionModel,
}

fn main() {
    mdpicoem_harness::harness_tracing_init();
    let seconds = parse_arg("--seconds").unwrap_or(5);
    let cycles_target = parse_arg_u64("--cycles");
    let quantum = parse_arg("--quantum").unwrap_or(125);
    let clock_mhz = parse_arg("--clock-mhz").unwrap_or(125);
    let sys_clk_hz = clock_mhz * 1_000_000;
    let core = parse_arg("--core").unwrap_or(2) as usize;
    let unpaced_flag = std::env::args().any(|a| a == "--unpaced");
    let step_quantum = parse_arg("--step-quantum").unwrap_or(BENCH_DEFAULT_STEP_QUANTUM);
    let model_sel = parse_model().unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1);
    });
    let reps = parse_arg("--reps").unwrap_or(5);
    let workload = parse_workload().unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1);
    });

    // `--model both` implies `--unpaced` per HLD V1 §7.3 — paced mode
    // measures real-time sustainability, which is only defined for a
    // single model at a time.
    let unpaced = unpaced_flag || matches!(model_sel, ModelSel::Both);

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
    if step_quantum == 0 {
        eprintln!("error: --step-quantum must be > 0");
        std::process::exit(1);
    }
    if reps == 0 {
        eprintln!("error: --reps must be > 0");
        std::process::exit(1);
    }
    if cycles_target.is_some() && !unpaced {
        eprintln!("error: --cycles requires --unpaced (paced mode is duration-driven)");
        std::process::exit(1);
    }

    // Threading-availability preflight. Mirrors the RP2350 bench's
    // guard: on non-Windows / non-Linux / non-x86_64 the threaded
    // `build()` returns `Err(ConfigError::ThreadingUnavailable)`. Per
    // task brief, print a skip message + exit 0 rather than failing.
    #[cfg(not(all(
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    )))]
    if matches!(model_sel, ModelSel::Threaded | ModelSel::Both) {
        println!(
            "(skip) --model threaded|both requires x86_64 Windows or Linux with \
             the `threading` cargo feature enabled; exiting cleanly."
        );
        std::process::exit(0);
    }

    // Raise priority and pin to a specific core to minimise OS preemption.
    // Uses HIGH_PRIORITY_CLASS (not REALTIME) to stay safe — won't block kernel threads.
    #[cfg(target_os = "windows")]
    match win::boost_and_pin(core) {
        Ok(()) => println!("Pinned to core {} at HIGH priority / TIME_CRITICAL", core),
        Err(e) => eprintln!(
            "warning: failed to boost priority: {} (continuing with default)",
            e
        ),
    }
    #[cfg(not(target_os = "windows"))]
    let _ = core;

    // `--model both` A/B harness: N reps per model, print comparative
    // table with median / IQR / delta% / verdict.
    if matches!(model_sel, ModelSel::Both) {
        let base_cfg = RunConfig {
            seconds,
            cycles_target,
            quantum,
            clock_mhz,
            sys_clk_hz,
            unpaced,
            step_quantum,
            workload,
            model: ExecutionModel::Serial, // per-rep override inside A/B loop
        };
        run_ab_harness(&base_cfg, reps);
        return;
    }

    // `--model serial|threaded`: single-model replication with stats.
    let exec_model = match model_sel {
        ModelSel::Serial => ExecutionModel::Serial,
        ModelSel::Threaded => ExecutionModel::Threaded,
        ModelSel::Both => unreachable!("handled above"),
    };
    let cfg = RunConfig {
        seconds,
        cycles_target,
        quantum,
        clock_mhz,
        sys_clk_hz,
        unpaced,
        step_quantum,
        workload,
        model: exec_model,
    };
    run_single_model_reps(&cfg, reps);
}

/// Run the full bench once with the given configuration. Returns the
/// headline throughput (Avg MHz): in unpaced mode this is the
/// executed-cycles-per-wall-second figure (per-core peak); in paced
/// mode this is `PacerStats::emulated_mhz()`.
fn run_once(cfg: &RunConfig) -> f64 {
    let seconds = cfg.seconds;
    let cycles_target = cfg.cycles_target;
    let quantum = cfg.quantum;
    let clock_mhz = cfg.clock_mhz;
    let sys_clk_hz = cfg.sys_clk_hz;
    let unpaced = cfg.unpaced;
    let step_quantum = cfg.step_quantum;
    let workload = cfg.workload;
    let model = cfg.model;

    // --- Set up emulator + selected workload ---
    // Workload setup must run on the Serial handle regardless — every
    // `mmio_write32` / `poke` / `core_mut` touch is cheap-and-direct on
    // the serial path, and the threaded path lazily promotes on the
    // first `run(cycles)` call (see `lib.rs` §Emulator::run).
    let mut emu = match EmulatorBuilder::new(Config { sys_clk_hz })
        .step_quantum(step_quantum)
        .execution(model)
        .build()
    {
        Ok(emu) => emu,
        Err(ConfigError::ThreadingUnavailable) => {
            eprintln!(
                "error: ExecutionModel::Threaded unavailable in this build \
                 (requires x86_64 Windows + `threading` feature)"
            );
            std::process::exit(1);
        }
    };
    setup(&mut emu, workload);

    // --- Set up pacer ---
    let mut pacer = Pacer::with_quantum(sys_clk_hz, quantum.into());
    let stats = pacer.stats();

    let core_mode = if workload.is_dual_core() {
        "dual-core"
    } else {
        "single-core"
    };
    let pio_mode = if workload.needs_pio() {
        " + PIO0 SM0 wrap"
    } else {
        ""
    };
    let runtime_mode = match model {
        ExecutionModel::Serial => "serial",
        ExecutionModel::Threaded => "threaded",
    };
    println!(
        "mdrp2040 paced benchmark — target {} MHz, quantum {} cycles, step_quantum {}, {}, workload {}{}, runtime {}",
        clock_mhz,
        quantum,
        step_quantum,
        core_mode,
        workload.as_str(),
        pio_mode,
        runtime_mode,
    );
    println!("TSC calibrated: {} MHz\n", pacer.tsc_freq_hz() / 1_000_000);
    println!(
        "{:>6} {:>14} {:>10} {:>8} {:>10} {:>8}",
        "time", "emu_cycles", "emu_MHz", "util%", "headroom%", "behind"
    );

    // --- Monitoring thread ---
    stats.set_running(true);
    let mon_stats = Arc::clone(&stats);
    let monitor = std::thread::spawn(move || monitor_loop(mon_stats));

    // --- Execution ---
    // Snapshot per-core cycle counters *before* the run so the bench
    // can report "Avg MHz" from real instruction work rather than
    // cycles-requested. In threaded mode `Emulator::run` advances the
    // requested cycle budget regardless of whether cores execute (a
    // halted core stays at 0), so requested-based MHz would mis-report
    // dual-core workloads where only one core is active.
    let c0_start = emu.core_cycles(0);
    let c1_start = emu.core_cycles(1);
    let start = Instant::now();
    let duration = Duration::from_secs(seconds.into());
    let qc = pacer.quantum_cycles();

    // Threaded chunk size: `run(cycles)` in threaded mode dispatches
    // one barrier rendezvous per `step_quantum` cycles. Pacing at the
    // 125-cycle `--quantum` default would burn the budget on barrier
    // overhead. Use 1 virtual second per chunk in unpaced mode (like
    // RP2350), and honour `--quantum` in paced mode (the threaded path
    // is not recommended for paced — it's legal, just inefficient).
    let threaded_chunk_cycles: u64 = sys_clk_hz as u64;

    let mut unpaced_cycles: u64 = 0;
    if unpaced {
        if let Some(target) = cycles_target {
            println!(
                "(unpaced mode — running flat-out until {} emulated cycles)",
                target
            );
        } else {
            println!("(unpaced mode — running flat-out, no real-time pacing)");
        }
        loop {
            if let Some(target) = cycles_target {
                if unpaced_cycles >= target {
                    break;
                }
            } else if start.elapsed() >= duration {
                break;
            }
            let chunk = if matches!(model, ExecutionModel::Threaded) {
                // Final iteration under --cycles: shrink the chunk so we
                // don't over-run the target by an entire virtual second.
                if let Some(target) = cycles_target {
                    threaded_chunk_cycles.min(target - unpaced_cycles)
                } else {
                    threaded_chunk_cycles
                }
            } else {
                qc
            };
            let consumed = run_with_model(&mut emu, chunk);
            unpaced_cycles += consumed;
        }
    } else {
        while start.elapsed() < duration {
            pacer.begin_quantum();
            let _ = run_with_model(&mut emu, qc);
            pacer.end_quantum();
        }
    }

    stats.set_running(false);
    monitor.join().unwrap();
    let c0_end = emu.core_cycles(0);
    let c1_end = emu.core_cycles(1);

    // --- Summary ---
    let wall_secs = start.elapsed().as_secs_f64();
    println!("\n--- summary ---");
    println!("Duration:       {:.1} s", wall_secs);
    println!("Workload:       {}", workload.as_str());
    println!("Runtime:        {}", runtime_mode);

    if unpaced {
        // "Cycles executed" = max of per-core deltas. Single-core
        // workloads halt core 1, so its delta is 0 and max reduces to
        // the running core's rate. Dual-core workloads run both cores
        // to roughly the same cycle count, so max ≈ either. This is
        // the real-time gate signal ("can a core sustain 125 MHz?"),
        // not an aggregate throughput number.
        let c0_delta = c0_end.saturating_sub(c0_start);
        let c1_delta = c1_end.saturating_sub(c1_start);
        let executed = c0_delta.max(c1_delta);
        let mhz = executed as f64 / wall_secs / 1_000_000.0;
        let host_cycles_per_emu = pacer.tsc_freq_hz() as f64 * wall_secs / executed.max(1) as f64;
        println!(
            "Executed cyc:   {} (c0={}, c1={})",
            executed, c0_delta, c1_delta
        );
        println!("Requested cyc:  {}", unpaced_cycles);
        println!(
            "Avg MHz:        {:.1} (per-core peak, from executed cycles)",
            mhz
        );
        println!("Host/emu cycle: {:.2}", host_cycles_per_emu);
        println!("Verdict:        UNPACED (profiling mode)");
        return mhz;
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
        println!(
            "Verdict:        REAL-TIME OK ({:.1}% of target, {:.2}% headroom, {:.3}% behind)",
            mhz_ratio * 100.0,
            snap.headroom() * 100.0,
            behind_rate * 100.0
        );
    } else if mhz_ratio >= 0.95 && behind_rate < 0.01 {
        println!(
            "Verdict:        REAL-TIME MARGINAL ({:.1}% of target, {:.2}% behind)",
            mhz_ratio * 100.0,
            behind_rate * 100.0
        );
    } else {
        println!(
            "Verdict:        CANNOT SUSTAIN REAL-TIME ({:.1}% of target, {:.2}% behind)",
            mhz_ratio * 100.0,
            behind_rate * 100.0
        );
    }

    snap.emulated_mhz()
}

/// Advance the emulator by at least `cycles` virtual cycles. Serial
/// `run` is infallible (panics on internal bug); Threaded `run` can
/// return `Err(EmulatorError::WorkerPanicked)` which we surface as a
/// hard bench failure — a worker panic is not a valid perf data point.
fn run_with_model(emu: &mut Emulator, cycles: u64) -> u64 {
    match emu.execution_model() {
        ExecutionModel::Serial => emu.run(cycles).expect("Serial run is infallible"),
        ExecutionModel::Threaded => match emu.run(cycles) {
            Ok(n) => n,
            Err(EmulatorError::WorkerPanicked { which, message }) => {
                eprintln!("fatal: threaded worker '{:?}' panicked: {}", which, message);
                std::process::exit(1);
            }
            Err(e) => {
                eprintln!("fatal: threaded run failed: {}", e);
                std::process::exit(1);
            }
        },
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
        println!(
            "{:>6} {:>14} {:>10.1} {:>7.1}% {:>9.1}% {:>8}",
            elapsed,
            snap.emulated_cycles,
            snap.emulated_mhz(),
            snap.utilization() * 100.0,
            snap.headroom() * 100.0,
            snap.behind_count
        );
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

/// Execution-model selector parsed from `--model`. Defaults to
/// `Serial` when absent (bench-historical behaviour on RP2040).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ModelSel {
    Serial,
    Threaded,
    Both,
}

fn parse_model() -> Result<ModelSel, String> {
    let args: Vec<String> = std::env::args().collect();
    for (i, a) in args.iter().enumerate() {
        if let Some(v) = a.strip_prefix("--model=") {
            return match_model(v);
        }
        if a == "--model" {
            let v = args
                .get(i + 1)
                .ok_or("--model requires serial|threaded|both")?;
            return match_model(v);
        }
    }
    Ok(ModelSel::Serial)
}

fn match_model(s: &str) -> Result<ModelSel, String> {
    match s {
        "serial" => Ok(ModelSel::Serial),
        "threaded" => Ok(ModelSel::Threaded),
        "both" => Ok(ModelSel::Both),
        other => Err(format!(
            "invalid --model '{other}' (expected serial|threaded|both)"
        )),
    }
}

/// A/B harness: run `reps` measured reps on Serial, then on Threaded,
/// and print a comparative table with median MHz / IQR / delta% /
/// verdict (HLD V1 §7.3). One warm-up rep per model is run and
/// discarded. Winner declared only when `|delta| > max(3 * IQR /
/// median_serial, 5%)` — anything inside the noise floor is "TIED".
fn run_ab_harness(base_cfg: &RunConfig, reps: u32) {
    println!(
        "[A/B bench] reps={} (+1 warmup/model discarded) --unpaced --workload {} --step-quantum {}",
        reps,
        base_cfg.workload.as_str(),
        base_cfg.step_quantum,
    );

    let serial_cfg = RunConfig {
        model: ExecutionModel::Serial,
        ..*base_cfg
    };
    let threaded_cfg = RunConfig {
        model: ExecutionModel::Threaded,
        ..*base_cfg
    };

    println!("\n=== Serial reps ===");
    let serial_mhz = collect_reps(&serial_cfg, reps);
    println!("\n=== Threaded reps ===");
    let threaded_mhz = collect_reps(&threaded_cfg, reps);

    let s_stats = Stats::from(&serial_mhz);
    let t_stats = Stats::from(&threaded_mhz);
    let delta_pct = if s_stats.median > 0.0 {
        100.0 * (t_stats.median - s_stats.median) / s_stats.median
    } else {
        0.0
    };
    // Noise floor per HLD V1 §7.3: max(3 × IQR/median_serial, 5%).
    let iqr_ratio = if s_stats.median > 0.0 {
        100.0 * (s_stats.p75 - s_stats.p25) / s_stats.median
    } else {
        0.0
    };
    let noise_floor = (3.0 * iqr_ratio).max(5.0);
    let verdict = if delta_pct.abs() <= noise_floor {
        "TIED (within noise)".to_string()
    } else if delta_pct > 0.0 {
        format!("WINNER: Threaded (+{:.1}%)", delta_pct)
    } else {
        format!("WINNER: Serial ({:.1}%)", delta_pct)
    };

    println!(
        "\n=== A/B summary — workload: {} ===",
        base_cfg.workload.as_str()
    );
    println!(
        "{:>9} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "model", "median", "p25", "p75", "min", "max"
    );
    print_stats_row("serial", &s_stats);
    print_stats_row("threaded", &t_stats);
    println!(
        "\ndelta: {:+.1}% (noise floor: ±{:.1}%)",
        delta_pct, noise_floor
    );
    println!("verdict: {}", verdict);
}

/// Single-model replication harness: `--model serial|threaded` plus
/// `--reps N` runs N measured reps (+ 1 discarded warm-up) and prints
/// a stats-only table with no delta / verdict (single-model mode has
/// no opposite to compare against).
fn run_single_model_reps(cfg: &RunConfig, reps: u32) {
    let label = match cfg.model {
        ExecutionModel::Serial => "serial",
        ExecutionModel::Threaded => "threaded",
    };
    println!(
        "[single-model bench] reps={} (+1 warmup discarded) --model {} --workload {} --step-quantum {}",
        reps,
        label,
        cfg.workload.as_str(),
        cfg.step_quantum,
    );
    let samples = collect_reps(cfg, reps);
    let s = Stats::from(&samples);
    println!(
        "\n=== stats — workload: {} (model: {}) ===",
        cfg.workload.as_str(),
        label
    );
    println!(
        "{:>9} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "model", "median", "p25", "p75", "min", "max"
    );
    print_stats_row(label, &s);
}

/// Run `reps` measured reps (+ 1 discarded warm-up) under `cfg` and
/// return the measured MHz values.
fn collect_reps(cfg: &RunConfig, reps: u32) -> Vec<f64> {
    let mut out = Vec::with_capacity(reps as usize);
    for i in 0..=reps {
        let tag = if i == 0 { "warmup" } else { "rep" };
        println!("\n--- {} {}/{} ---", tag, i, reps);
        let mhz = run_once(cfg);
        if i == 0 {
            println!("(warmup: {:.1} MHz, discarded)", mhz);
        } else {
            out.push(mhz);
        }
    }
    out
}

/// Nearest-rank percentile index for a sorted population of size
/// `n`: `ceil(pct/100 * n)` clamped to `[1, n]` then converted to a
/// zero-based index.
fn pct_idx(n: usize, pct: u8) -> usize {
    debug_assert!(n > 0, "pct_idx: n must be > 0");
    debug_assert!(pct <= 100, "pct_idx: pct must be in 0..=100");
    let r = ((pct as f64) / 100.0 * n as f64).ceil() as usize;
    r.clamp(1, n) - 1
}

/// Summary stats for a `Vec<f64>`: median, 25th/75th percentile, min,
/// max. Nearest-rank percentiles — simple and sufficient for N≥5.
struct Stats {
    median: f64,
    p25: f64,
    p75: f64,
    min: f64,
    max: f64,
}

impl Stats {
    fn from(samples: &[f64]) -> Self {
        assert!(!samples.is_empty(), "Stats::from: empty samples");
        let mut sorted = samples.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted.len();
        let pct = |p: u8| -> f64 { sorted[pct_idx(n, p)] };
        Self {
            median: pct(50),
            p25: pct(25),
            p75: pct(75),
            min: sorted[0],
            max: sorted[n - 1],
        }
    }
}

fn print_stats_row(label: &str, s: &Stats) {
    println!(
        "{:>9} {:>10.1} {:>10.1} {:>10.1} {:>10.1} {:>10.1}",
        label, s.median, s.p25, s.p75, s.min, s.max
    );
}

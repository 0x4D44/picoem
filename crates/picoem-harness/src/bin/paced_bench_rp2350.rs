//! Paced benchmark: measures whether the rp2350_emu emulator can sustain
//! real-time at the RP2354's stock 150 MHz.
//!
//! The `--workload` flag picks from a set of synthetic workloads that
//! span the cost envelope from single-core ALU floor to "both cores
//! active + peripherals singing" realistic worst case, plus the
//! existing `fpu-heavy` workload (RP2350-specific, used by the HLD §12
//! performance budget check). See `wrk_docs/2026.04.15 - HLD - Paced
//! Bench Workload Spread.md` for rationale.
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
//!   --workload <name>  One of: basic (default), peripheral, contention,
//!                      stress, fpu-heavy, onerom-shape. Core count is
//!                      implied by the workload: basic / peripheral /
//!                      fpu-heavy / onerom-shape are single-core,
//!                      contention / stress are dual-core. `onerom-shape`
//!                      configures PIO1 (1 SM) + PIO2 (2 SMs) — no PIO0
//!                      — to mirror Pier's OneROM block/SM enable mask;
//!                      used to decide whether splitting our single PIO
//!                      worker thread into per-block workers would help.
//!                      NB: `onerom-shape` is a topology mirror, NOT the
//!                      real OneROM CPU-mode serve pipeline — the real
//!                      benchmark lives at `onerom_cpu_speed_grade_*`.
//!                      The legacy `onerom` spelling is still accepted
//!                      as a deprecated alias.
//!   --threaded         Route execution through `ThreadedEmulator` (4
//!                      pinned workers: core0, core1, PIO, coordinator)
//!                      instead of the serial-interleave `Emulator::run`
//!                      path. Windows x86_64 only — matches the
//!                      `#[cfg]` gate on the threaded module. Workload
//!                      setup, pacing, stats and output format are
//!                      identical to the serial path so A/B diffs are
//!                      direct.
//!   --step-quantum N   Cycles per emulator step (default 256 — see
//!                      `BENCH_DEFAULT_STEP_QUANTUM`). In threaded mode
//!                      this is also the cycles-per-barrier-rendezvous,
//!                      so larger values amortise the 4-thread barrier
//!                      cost while smaller values improve inter-worker
//!                      latency (PIO seeing a CPU TX FIFO write etc.).
//!                      256 sits at the knee: peripheral/stress PIO
//!                      workloads plateau here and larger values buy
//!                      no further throughput — just 16× worse latency
//!                      at sq=4096. See
//!                      `wrk_journals/2026.04.20 - JRN - PIO step_n
//!                      Profiling.md`. For A/B comparability between
//!                      serial and threaded, both paths honour the
//!                      same value.
//!   --timing           Threaded-mode only. Enable per-worker
//!                      per-quantum timing instrumentation and print
//!                      a summary table at end of run showing, for
//!                      each of core0/core1/pio/coord, mean/p50/p99/
//!                      max phase_work_ns (work done before the
//!                      barrier) and barrier_wait_ns (time blocked
//!                      waiting for peers). Off by default; when off
//!                      the workers skip every `Instant::now()` call.
//!   --runs N           Repeat the full bench N+1 times: one warm-up
//!                      run (output labelled, but discarded from
//!                      stats) plus N measured runs. Prints a per-run
//!                      table of throughput (Avg MHz) and a summary
//!                      line reporting `mean / stdev / cv% / p50 /
//!                      p99 / min / max` across the N measured runs.
//!                      `cv%` (= 100 * stdev / mean) is the
//!                      machine-readable noise floor for the HLD V5
//!                      §4 item 8 ±3 % regression check. Default
//!                      (flag omitted) is the pre-existing single-run
//!                      behaviour. `--runs 1` degenerates to
//!                      warm-up-only with zero stats runs; falls
//!                      back silently to single-run mode.
//!   --model <name>     Dual-execution HLD V1 Stage 2 A/B harness.
//!                      One of `serial|threaded|both`. `serial` is
//!                      the default; `threaded` is equivalent to the
//!                      legacy `--threaded` flag; `both` runs N reps
//!                      on each model and prints a comparative
//!                      Serial-vs-Threaded table with median MHz,
//!                      IQR, delta% and per-workload verdict. `both`
//!                      implies `--unpaced` per HLD V1 §7.3. Paired
//!                      with `--reps`.
//!   --reps N           Replication count for `--model both` (also
//!                      for `--model serial|threaded` stats).
//!                      Default 5 (HLD V1 §7.3 minimum). One warm-up
//!                      rep per model is run and discarded; N
//!                      measured reps follow.

use rp2350_emu::{Config, Emulator, EmulatorBuilder, Pacer, PacerStats};

/// Bench CLI default for `--step-quantum`. Diverges from the library
/// `DEFAULT_STEP_QUANTUM = 64` because the bench is overwhelmingly
/// run in threaded mode, where the library default makes the barrier
/// dominate. 256 hits the peripheral/stress plateau with ~1.7 μs
/// inter-worker latency at 150 MHz sys_clk — larger values buy no
/// throughput, smaller values cost it. Serial runs honour this too
/// for A/B comparability; the serial path is insensitive to quantum
/// size in this range so there's no downside.
const BENCH_DEFAULT_STEP_QUANTUM: u32 = 256;
#[cfg(all(
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
use rp2350_emu::threaded::ThreadedEmulator;
use std::sync::Arc;
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// Workload selection + VFP encoders
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Workload {
    /// Default: MOVS R0,#1 / ADDS R0,R0,#1 / B .-2 — tight ALU loop.
    Basic,
    /// Single-core ALU + SIO GPIO toggle, with PIO0 SM0 blinking a pin
    /// in the background (keeps `PioBlock::execute_cycle` on the hot
    /// path).
    Peripheral,
    /// Both cores running the ALU loop (core 1 placed at bank-0 offset
    /// for CLI symmetry with RP2040). rp2350_emu runs cores sequentially
    /// per quantum with no production-path contention hooks, so this
    /// workload measures dual-core compute cost only — no bank-contention
    /// accounting. See HLD "Chip asymmetry" note.
    Contention,
    /// Composite: core 0 runs the peripheral loop, core 1 runs the basic
    /// ALU loop, PIO0 SM0 running. On RP2350 this is dual-core compute +
    /// peripheral cost (no bank contention — see `Contention` above).
    Stress,
    /// FPU-heavy: VADD/VMUL/VDIV/VSQRT loop exercising the FPU hot path.
    /// Used by the HLD §12 performance budget check.
    FpuHeavy,
    /// OneROM-shape: no PIO0 activity, PIO1 with 1 SM enabled (address
    /// reader analogue), PIO2 with 2 SMs enabled (data writer + CS
    /// handler analogues). Mirrors the block/SM enable mask Pier's
    /// OneROM firmware programs — see agent analysis in
    /// `wrk_journals/2026.04.20 - JRN - PIO step_n Profiling.md`.
    /// Core 0 runs the peripheral loop to keep CPU-side MMIO traffic
    /// on the hot path; core 1 halted (OneROM is single-core).
    /// Does NOT reproduce real serving semantics (emulator DMA is
    /// stubbed and the harness `GlueDma` helper can't hook into
    /// threaded mode) — the point is to measure whether PIO or CPU
    /// is the critical-path worker under the OneROM block/SM shape.
    ///
    /// **Name note:** this variant is `OneromShape` (CLI token
    /// `onerom-shape`), not bare `onerom`, to distinguish it from
    /// the real OneROM CPU-mode benchmark at
    /// `onerom_cpu_speed_grade_rp2350.rs`. Arthur's real-OneROM
    /// Serial>Threaded observation anchors HLD §1; this shape mirror
    /// is a different data point and the two are not expected to
    /// agree on the Serial-vs-Threaded verdict.
    OneromShape,
}

impl Workload {
    fn is_dual_core(self) -> bool {
        matches!(self, Workload::Contention | Workload::Stress)
    }

    fn needs_pio(self) -> bool {
        matches!(
            self,
            Workload::Peripheral | Workload::Stress | Workload::OneromShape
        )
    }

    fn as_str(self) -> &'static str {
        match self {
            Workload::Basic => "basic",
            Workload::Peripheral => "peripheral",
            Workload::Contention => "contention",
            Workload::Stress => "stress",
            Workload::FpuHeavy => "fpu-heavy",
            Workload::OneromShape => "onerom-shape",
        }
    }
}

/// Encode VFP data-processing (VADD/VSUB/VMUL/VDIV) single-precision.
///
/// Mirrors the private `vfp_dp` helper in the test harness (kept local to
/// this binary so `paced_bench_rp2350` stays self-contained).
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

fn enc_vadd(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(0, 0b11, 0, sd, sn, sm)
}
fn enc_vmul(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(0, 0b10, 0, sd, sn, sm)
}
fn enc_vdiv(sd: u16, sn: u16, sm: u16) -> (u16, u16) {
    vfp_dp(1, 0b00, 0, sd, sn, sm)
}

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

// ---------------------------------------------------------------------------
// RP2350 MMIO register addresses (RP2350 datasheet).
// ---------------------------------------------------------------------------

/// RESETS base on RP2350 (see `rp2350_emu/src/bus/peripherals.rs`).
const RESETS_BASE: u32 = 0x4002_0000;
const RESETS_RESET_OFFSET: u32 = 0x00;
/// RESETS.RESET bit 15 = PIO0 on RP2350.
const RESETS_PIO0_BIT: u32 = 1 << 15;

const IO_BANK0_BASE: u32 = 0x4002_8000;
const PADS_BANK0_BASE: u32 = 0x4003_8000;

const SIO_BASE: u32 = 0xD000_0000;
/// SIO GPIO_OE_SET on RP2350 (8-byte spacing — SET is offset 0x038).
const SIO_GPIO_OE_SET: u32 = SIO_BASE + 0x038;
/// SIO GPIO_OUT_XOR on RP2350 (8-byte spacing — XOR is offset 0x028).
const SIO_GPIO_OUT_XOR: u32 = SIO_BASE + 0x028;

const PIO0_BASE: u32 = 0x5020_0000;
const PIO1_BASE: u32 = 0x5030_0000;
const PIO2_BASE: u32 = 0x5040_0000;
/// RESETS.RESET bits for PIO1/PIO2 on RP2350 (see `rp2350_emu/src/bus/mod.rs`
/// `RESET_PIO1`/`RESET_PIO2`). PIO0 bit in the existing
/// `RESETS_PIO0_BIT = 1 << 15` is wrong per the datasheet (the correct
/// bit is 11) but harmless — the emulator doesn't gate PIO activity on
/// RESETS. Left unchanged here to avoid scope creep.
const RESETS_PIO1_BIT: u32 = 1 << 12;
const RESETS_PIO2_BIT: u32 = 1 << 13;
/// FUNCSEL values for PIO1/PIO2. PIO0 uses 6 per `FUNCSEL_PIO0`; PIO1
/// is 7, PIO2 is 8 on RP2350.
const FUNCSEL_PIO1: u8 = 7;
const FUNCSEL_PIO2: u8 = 8;
const PIO_CTRL: u32 = 0x000;
const PIO_INSTR_MEM0: u32 = 0x048;
const PIO_SM0_CLKDIV: u32 = 0x0C8;
const PIO_SM0_EXECCTRL: u32 = 0x0CC;
const PIO_SM0_SHIFTCTRL: u32 = 0x0D0;
const PIO_SM0_INSTR: u32 = 0x0D8;
const PIO_SM0_PINCTRL: u32 = 0x0DC;

/// FUNCSEL for PIO0 (RP2350: 6 = PIO0, same as RP2040).
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
///
/// RP2350 IO_BANK0 / PADS_BANK0 aren't deeply modelled (writes land in
/// the bus's catch-all peripheral_regs map), but emit the writes anyway
/// so the workload setup matches real firmware bring-up.
fn setup_gpio_output(emu: &mut Emulator, pin: u8, funcsel: u8) {
    // IO_BANK0_GPIO<pin>_CTRL is at offset 0x04 + pin*8 (per-pin pair:
    // STATUS at 0x00 + 8*pin, CTRL at 0x04 + 8*pin).
    let ctrl_offset = 0x04 + (pin as u32) * 8;
    emu.mmio_write32(IO_BANK0_BASE + ctrl_offset, funcsel as u32);

    // PADS_BANK0_GPIO<pin> at offset 0x04 + pin*4. SCHMITT=1 (bit 1),
    // DRIVE=01=4mA (bits 5:4), IE=1 (bit 6); PUE=0, PDE=0, OD=0.
    let pad_offset = 0x04 + (pin as u32) * 4;
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
    let execctrl = 1u32 << 12;
    emu.mmio_write32(PIO0_BASE + PIO_SM0_EXECCTRL, execctrl);

    // SM0_SHIFTCTRL: re-emit the reset value (autopush/autopull off,
    // thresholds 32 — encoded as 0).
    emu.mmio_write32(PIO0_BASE + PIO_SM0_SHIFTCTRL, 0x000C_0000);

    // SM0_CLKDIV: INT=1 (bits 31:16), FRAC=0. One SM cycle per sys_clk.
    emu.mmio_write32(PIO0_BASE + PIO_SM0_CLKDIV, 1u32 << 16);

    // Force `SET PINDIRS, 1` through SM0_INSTR so the pin becomes driven.
    // Encoding: opcode=111 (SET), dest=PINDIRS (4), data=1 → 0xE081.
    emu.mmio_write32(PIO0_BASE + PIO_SM0_INSTR, 0xE081);

    // CTRL.SM_ENABLE bit 0 — enable SM0.
    emu.mmio_write32(PIO0_BASE + PIO_CTRL, 0x1);
}

/// SM register stride (SMx_CLKDIV at SM0_CLKDIV + x*0x18, etc.).
const PIO_SM_STRIDE: u32 = 0x18;

/// Program one SM on a given PIO block with the minimal 2-instruction
/// wrap loop, CLKDIV=1, no pin output. Used by the OneROM-shape
/// workload to keep N SMs saturating `step_n` without FIFO stalls.
/// Program content is irrelevant to per-sysclk dispatch cost — all
/// that matters is which SMs are enabled.
fn program_sm_wrap_no_pins(emu: &mut Emulator, pio_base: u32, sm: u32) {
    let off = sm * PIO_SM_STRIDE;
    // INSTR_MEM[2*sm] = SET X, 0 (0xE020); INSTR_MEM[2*sm+1] = SET X, 1
    // (0xE021). Independent wrap window per SM so they don't collide.
    let prog_base = PIO_INSTR_MEM0 + (2 * sm) * 4;
    emu.mmio_write32(pio_base + prog_base, 0xE020);
    emu.mmio_write32(pio_base + prog_base + 4, 0xE021);
    // EXECCTRL: WRAP_TOP=2*sm+1, WRAP_BOTTOM=2*sm.
    let wrap_top = 2 * sm + 1;
    let wrap_bottom = 2 * sm;
    let execctrl = (wrap_top << 12) | (wrap_bottom << 7);
    emu.mmio_write32(pio_base + PIO_SM0_EXECCTRL + off, execctrl);
    // SHIFTCTRL: reset defaults.
    emu.mmio_write32(pio_base + PIO_SM0_SHIFTCTRL + off, 0x000C_0000);
    // CLKDIV: INT=1, FRAC=0 (1 SM cycle per sys_clk).
    emu.mmio_write32(pio_base + PIO_SM0_CLKDIV + off, 1u32 << 16);
    // Force PC to the SM's wrap window so it starts executing immediately.
    // JMP to wrap_bottom: JMP = 0x0000 | addr (5 bits).
    let jmp = wrap_bottom & 0x1F;
    emu.mmio_write32(pio_base + PIO_SM0_INSTR + off, jmp);
}

/// Set up the OneROM block/SM shape: PIO1 with SM0 only, PIO2 with
/// SM0 + SM1. PIO0 is left untouched (unused in OneROM serving mode).
/// All SMs run a minimal wrap loop at CLKDIV=1 — program content is
/// irrelevant to the per-sysclk dispatch cost we're measuring.
fn setup_pio1_pio2_onerom_shape(emu: &mut Emulator) {
    // RESETS de-assert for PIO1 and PIO2 via APB CLR alias (+0x3000).
    // Matches real firmware bring-up; the emulator doesn't currently
    // gate PIO activity on RESETS so this is a formality.
    emu.mmio_write32(
        RESETS_BASE + RESETS_RESET_OFFSET + 0x3000,
        RESETS_PIO1_BIT | RESETS_PIO2_BIT,
    );
    // Route GPIOs for each block — again formality, but matches
    // real-firmware bring-up sequencing. GPIO numbers match OneROM's
    // observed pin assignment (agent analysis, not verified against
    // silicon — purely for MMIO-write symmetry).
    setup_gpio_output(emu, 10, FUNCSEL_PIO1); // PIO1 address-reader pin stand-in
    setup_gpio_output(emu, 16, FUNCSEL_PIO2); // PIO2 D0-D7 stand-in
    setup_gpio_output(emu, 17, FUNCSEL_PIO2); // PIO2 CS-handler stand-in

    // PIO1: SM0 only.
    program_sm_wrap_no_pins(emu, PIO1_BASE, 0);
    emu.mmio_write32(PIO1_BASE + PIO_CTRL, 0x1);

    // PIO2: SM0 + SM1.
    program_sm_wrap_no_pins(emu, PIO2_BASE, 0);
    program_sm_wrap_no_pins(emu, PIO2_BASE, 1);
    emu.mmio_write32(PIO2_BASE + PIO_CTRL, 0x3);
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

/// Build the FPU-heavy workload at 0x2000_0000:
///   VADD S3, S1, S2
///   VMUL S3, S3, S1
///   VDIV S3, S3, S2
///   VSQRT S3, S3
///   B .-20
fn setup_fpu_heavy_core0(emu: &mut Emulator) {
    let (va0, va1) = enc_vadd(3, 1, 2); // VADD  S3, S1, S2
    let (vm0, vm1) = enc_vmul(3, 3, 1); // VMUL  S3, S3, S1
    let (vd0, vd1) = enc_vdiv(3, 3, 2); // VDIV  S3, S3, S2
    let (vs0, vs1) = enc_vsqrt(3, 3); // VSQRT S3, S3
    emu.poke(0x2000_0000, pair(va0, va1));
    emu.poke(0x2000_0004, pair(vm0, vm1));
    emu.poke(0x2000_0008, pair(vd0, vd1));
    emu.poke(0x2000_000C, pair(vs0, vs1));
    // B .-20 back to 0x2000_0000 (PC at B is 0x2000_0010, PC+4 = 0x2000_0014,
    // target = 0x2000_0000, imm11 = -10 = 0x7F6 → hw = 0xE7F6).
    emu.poke(0x2000_0010, 0x0000_E7F6);

    emu.core_mut(0).regs.set_pc(0x2000_0000);
    emu.core_mut(0).regs.xpsr = 1 << 24;

    // Seed the FPU sources so the steady-state is finite and well-behaved
    // (no NaN/Inf churn that would distort timing). Pick values that
    // cycle without overflow:
    //   S1 = 2.0, S2 = 3.0
    //   VADD S3 = 5.0 ; VMUL S3 = 10.0 ; VDIV S3 ≈ 3.333 ; VSQRT S3 ≈ 1.826
    emu.core_mut(0).regs.s[1] = 2.0;
    emu.core_mut(0).regs.s[2] = 3.0;
}

/// Dispatch: set up the emulator for the chosen workload.
fn setup(emu: &mut Emulator, workload: Workload) {
    // Stack placement for RP2350. SRAM8 (0x2008_0000) and SRAM9
    // (0x2008_1000) are non-striped scratch banks — keeping stacks
    // there keeps push/pop traffic out of the bank-0 fetch-contention
    // signal in the dual-core workloads.
    let core0_stack_top: u32 = if workload.is_dual_core() {
        0x2008_1000 // top of SRAM8
    } else {
        0x2008_0000 // top of striped region (workloads use a low-touch stack)
    };
    emu.core_mut(0).regs.msp = core0_stack_top;
    emu.core_mut(0).regs.r[13] = core0_stack_top;

    match workload {
        Workload::Basic => {
            setup_basic_core0(emu);
            emu.core_mut(1).halt();
        }
        Workload::Peripheral => {
            setup_peripheral_core0(emu);
            // SIO drives pin 25 directly; PIO drives PIO_PIN.
            setup_gpio_output(emu, SIO_TOGGLE_PIN, 5); // FUNCSEL=5 = SIO
            // Set OE bit for SIO pin so the toggle is observable.
            emu.mmio_write32(SIO_GPIO_OE_SET, 1u32 << SIO_TOGGLE_PIN);
            setup_pio0_sm0_wrap(emu, PIO_PIN);
            emu.core_mut(1).halt();
        }
        Workload::Contention => {
            setup_basic_core0(emu);
            // Core 1 placed 16 words past core 0 (RP2040 bank-0 layout).
            // No timing effect on RP2350 — cores run sequentially per
            // quantum, no production-path contention model. Kept for
            // CLI symmetry. See HLD "Chip asymmetry" note.
            setup_basic_core1_at(emu, 0x2000_0040);
            // Core 1 stack in SRAM9 (non-striped scratch). SRAM9 ends at
            // 0x2008_2000; first push lands at 0x2008_1FFC.
            let core1_stack_top: u32 = 0x2008_1FFC;
            emu.core_mut(1).regs.msp = core1_stack_top;
            emu.core_mut(1).regs.r[13] = core1_stack_top;
            emu.core_mut(1).wake();
        }
        Workload::Stress => {
            setup_peripheral_core0(emu);
            setup_gpio_output(emu, SIO_TOGGLE_PIN, 5);
            emu.mmio_write32(SIO_GPIO_OE_SET, 1u32 << SIO_TOGGLE_PIN);
            setup_pio0_sm0_wrap(emu, PIO_PIN);

            // Core 1 runs basic ALU loop at 0x2000_0040. No cross-core
            // bank contention on RP2350 — dual-core compute + peripheral
            // cost only. See HLD "Chip asymmetry" note.
            setup_basic_core1_at(emu, 0x2000_0040);
            let core1_stack_top: u32 = 0x2008_1FFC;
            emu.core_mut(1).regs.msp = core1_stack_top;
            emu.core_mut(1).regs.r[13] = core1_stack_top;
            emu.core_mut(1).wake();
        }
        Workload::FpuHeavy => {
            setup_fpu_heavy_core0(emu);
            emu.core_mut(1).halt();
        }
        Workload::OneromShape => {
            // Core 0 runs the peripheral loop (STR to GPIO_OUT_XOR) — same
            // CPU-side shape as `Peripheral` so the CPU worker does real
            // MMIO work every quantum, mirroring OneROM's CPU doing bus
            // stimulus alongside its PIO pipeline.
            setup_peripheral_core0(emu);
            setup_gpio_output(emu, SIO_TOGGLE_PIN, 5);
            emu.mmio_write32(SIO_GPIO_OE_SET, 1u32 << SIO_TOGGLE_PIN);
            // PIO1 (1 SM) + PIO2 (2 SMs) — the OneROM shape.
            setup_pio1_pio2_onerom_shape(emu);
            emu.core_mut(1).halt();
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

/// Execution backend. The serial path stays unchanged from the
/// pre-`--threaded` binary; the threaded variant consumes the same
/// `Emulator` after workload setup and replaces `emu.run(cycles)` with
/// `ThreadedEmulator::run_quanta(n)`. Quanta count derives from the
/// emulator's `step_quantum` so both backends advance identical
/// cycle counts per pacing interval — no accounting drift between A/B
/// runs.
enum Runtime {
    Serial(Emulator),
    #[cfg(all(
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    Threaded {
        inner: ThreadedEmulator,
        step_q: u32,
    },
}

impl Runtime {
    /// Advance the emulator by at least `cycles` virtual cycles,
    /// matching the serial `Emulator::run` overshoot contract.
    fn run(&mut self, cycles: u64) {
        match self {
            Runtime::Serial(emu) => {
                emu.run(cycles).expect("Serial run is infallible");
            }
            #[cfg(all(
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            Runtime::Threaded { inner, step_q } => {
                let sq = *step_q as u64;
                let n = cycles.div_ceil(sq);
                inner.run_quanta(n);
            }
        }
    }

    /// Per-core cycle counters `(c0, c1)`. Reads the real work done
    /// by each CPU core; halted cores stay at 0 across a run, and in
    /// threaded mode this is independent of the coordinator-driven
    /// `master_cycle`. Used by the bench to compute "Avg MHz" from
    /// instructions-executed rather than cycles-requested.
    fn core_cycles(&self) -> (u64, u64) {
        match self {
            Runtime::Serial(emu) => (emu.core(0).cycles(), emu.core(1).cycles()),
            #[cfg(all(
                target_arch = "x86_64",
                any(target_os = "windows", target_os = "linux")
            ))]
            Runtime::Threaded { inner, .. } => (inner.core_cycles(0), inner.core_cycles(1)),
        }
    }
}

/// Parsed CLI state, shared across all runs in a `--runs N` session so
/// each run is configured identically. All fields are `Copy`, so this
/// derives `Copy`; the A/B harness uses struct-update (`..base`) on
/// the value rather than a hand-rolled `clone_cfg`.
#[derive(Clone, Copy)]
struct RunConfig {
    seconds: u32,
    cycles_target: Option<u64>,
    quantum: u32,
    clock_mhz: u32,
    sys_clk_hz: u32,
    unpaced: bool,
    threaded: bool,
    timing: bool,
    step_quantum: u32,
    workload: Workload,
}

fn main() {
    picoem_harness::harness_tracing_init();
    let seconds = parse_arg("--seconds").unwrap_or(5);
    let cycles_target = parse_arg_u64("--cycles");
    let quantum = parse_arg("--quantum").unwrap_or(150);
    let clock_mhz = parse_arg("--clock-mhz").unwrap_or(150);
    let sys_clk_hz = clock_mhz * 1_000_000;
    let core = parse_arg("--core").unwrap_or(2) as usize;
    let unpaced_flag = std::env::args().any(|a| a == "--unpaced");
    let threaded_flag = std::env::args().any(|a| a == "--threaded");
    let timing = std::env::args().any(|a| a == "--timing");
    let step_quantum = parse_arg("--step-quantum").unwrap_or(BENCH_DEFAULT_STEP_QUANTUM);
    let runs_arg = parse_arg("--runs");
    let model = parse_model().unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1);
    });
    let reps = parse_arg("--reps").unwrap_or(5);
    let workload = parse_workload().unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1);
    });

    // `--model` reconciliation with legacy `--threaded`: when `--model`
    // is explicitly set, it wins. Legacy `--threaded` still maps to
    // `Model::Threaded`. `--model both` is the new A/B mode; it
    // implies `--unpaced` per HLD V1 §7.3.
    let unpaced = unpaced_flag || matches!(model, ModelSel::Both);
    let threaded = match model {
        ModelSel::Legacy => threaded_flag,
        ModelSel::Serial => false,
        ModelSel::Threaded => true,
        ModelSel::Both => false, // per-rep override inside A/B loop
    };

    #[cfg(not(all(
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    )))]
    if threaded || matches!(model, ModelSel::Threaded | ModelSel::Both) {
        eprintln!(
            "error: --threaded / --model threaded|both requires x86_64 Windows or \
             Linux (ThreadedEmulator is #[cfg]-gated to those targets — \
             pin_to_host_core uses SetThreadAffinityMask / pthread_setaffinity_np)"
        );
        std::process::exit(1);
    }

    if reps == 0 {
        eprintln!("error: --reps must be > 0");
        std::process::exit(1);
    }

    // `--dual-core` was removed in the workload-spread refactor: core
    // count is now a property of the workload (Basic/Peripheral/FpuHeavy
    // → single core; Contention/Stress → dual core). Reject it
    // explicitly so stale scripts get a helpful nudge instead of
    // silently running the wrong workload.
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
    if cycles_target.is_some() && !unpaced {
        eprintln!("error: --cycles requires --unpaced (paced mode is duration-driven)");
        std::process::exit(1);
    }
    // `--threaded` spawns 4 pinned workers per `run_quanta` call. Driving
    // it at 150-cycle pacer granularity would burn the entire budget on
    // thread spawn/join — measured at ~0 MHz during bring-up. Paced-mode
    // real-time accounting is also undefined for a model that rendezvous
    // on a barrier across cores. Require --unpaced so the threaded run
    // uses a single large `run_quanta` call and the MHz figure is
    // comparable against the serial --unpaced baseline.
    if threaded && !unpaced {
        eprintln!(
            "error: --threaded requires --unpaced (thread-spawn overhead per \
             pacer quantum would dominate; a single large run_quanta call \
             is the intended use — compare against serial --unpaced)"
        );
        std::process::exit(1);
    }
    if timing && !threaded {
        eprintln!(
            "error: --timing requires --threaded (the flag surfaces per-worker \
             barrier timings; the serial path has no workers to instrument)"
        );
        std::process::exit(1);
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

    let cfg = RunConfig {
        seconds,
        cycles_target,
        quantum,
        clock_mhz,
        sys_clk_hz,
        unpaced,
        threaded,
        timing,
        step_quantum,
        workload,
    };

    // A/B harness: `--model both` runs `reps` measured reps per model
    // (+ 1 warm-up per model, discarded) and prints a comparative
    // table. HLD V1 §7.3: declare a winner only when the delta
    // magnitude exceeds `max(3 × IQR/median_serial, 5%)`.
    if matches!(model, ModelSel::Both) {
        run_ab_harness(&cfg, reps);
        return;
    }

    // Single-model reps: `--model serial|threaded` with `--reps` runs
    // N measured reps (+ 1 discarded warm-up) on the selected model
    // and prints a stats-only table (no delta / verdict). Preserves
    // the A/B methodology's replication protocol for single-model
    // regression capture.
    if matches!(model, ModelSel::Serial | ModelSel::Threaded) {
        run_single_model_reps(&cfg, reps);
        return;
    }

    // `--runs N`: repeat the bench N+1 times (1 warmup discarded + N
    // measured). Default (flag absent) is the historical single-run
    // behaviour. `--runs 1` degenerates to warmup-only with zero
    // measured runs — fall back silently to single-run mode so the
    // user doesn't wait for a warmup they can't do stats on.
    let measured_runs: u32 = match runs_arg {
        None => 0,
        Some(n) if n <= 1 => {
            if n == 1 {
                println!(
                    "(note: --runs 1 degenerates to warmup-only; falling back to single-run mode)"
                );
            }
            0
        }
        Some(n) => n,
    };

    if measured_runs == 0 {
        run_once(&cfg);
        return;
    }

    let total_runs = measured_runs + 1;
    println!(
        "[bench] --runs {} ({} warmup + {} measured){}{}{} --workload {}",
        measured_runs,
        1,
        measured_runs,
        if cfg.threaded { " --threaded" } else { "" },
        if cfg.unpaced { " --unpaced" } else { "" },
        if cfg.timing { " --timing" } else { "" },
        cfg.workload.as_str(),
    );

    let mut throughputs: Vec<f64> = Vec::with_capacity(measured_runs as usize);
    for run_idx in 1..=total_runs {
        let label = if run_idx == 1 { "warmup" } else { "run" };
        println!("\n=== run {}/{} ({}) ===", run_idx, total_runs, label);
        let mhz = run_once(&cfg);
        if run_idx == 1 {
            println!(
                "(run {}: {:.1} MHz — warmup, discarded from stats)",
                run_idx, mhz
            );
        } else {
            println!("(run {}: {:.1} MHz)", run_idx, mhz);
            throughputs.push(mhz);
        }
    }

    print_runs_summary(&cfg, &throughputs);
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
    let threaded = cfg.threaded;
    #[cfg(all(
        target_arch = "x86_64",
        any(target_os = "windows", target_os = "linux")
    ))]
    let timing = cfg.timing;
    let step_quantum = cfg.step_quantum;
    let workload = cfg.workload;

    // --- Set up emulator + selected workload ---
    let mut emu = EmulatorBuilder::new(Config { sys_clk_hz })
        .step_quantum(step_quantum)
        .build()
        .unwrap();
    setup(&mut emu, workload);

    // Promote into threaded mode after setup — `ThreadedEmulator::from_emulator`
    // consumes the `Emulator`, so every workload poke / mmio_write32 must run
    // on the serial handle first. Workload setup uses only `poke` (memory,
    // bypasses cache-invalidation) and `mmio_write32` to non-executable
    // peripheral addresses, so the `from_emulator` debug-assert on drained
    // pending_cache_invalidations / pending_invalidation_regions is
    // trivially satisfied — no step or manual invalidate needed.
    #[allow(unused_mut)]
    let mut runtime = {
        #[cfg(all(
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        {
            if threaded {
                let step_q = emu.step_quantum;
                let mut inner = ThreadedEmulator::from_emulator(emu);
                inner.set_timing_enabled(timing);
                Runtime::Threaded { inner, step_q }
            } else {
                Runtime::Serial(emu)
            }
        }
        #[cfg(not(all(
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        )))]
        {
            // `threaded` guaranteed false by the earlier non-Windows
            // rejection above.
            let _ = threaded;
            Runtime::Serial(emu)
        }
    };

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
    let runtime_mode = if threaded { "threaded" } else { "serial" };
    println!(
        "rp2350_emu paced benchmark — target {} MHz, quantum {} cycles, step_quantum {}, {}, workload {}{}, runtime {}",
        clock_mhz,
        quantum,
        step_quantum,
        core_mode,
        workload.as_str(),
        pio_mode,
        runtime_mode,
    );
    #[cfg(target_arch = "x86_64")]
    println!("TSC calibrated: {} MHz\n", pacer.tsc_freq_hz() / 1_000_000);
    #[cfg(not(target_arch = "x86_64"))]
    println!(
        "Pacer timer:    Instant backend ({} MHz tick scale)\n",
        pacer.tsc_freq_hz() / 1_000_000
    );
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
    // cycles-requested. The threaded coordinator advances
    // `master_cycle` by `step_q` per quantum regardless of whether
    // cores execute (`threaded::emulator::run_quanta_single_then_many`
    // demonstrates this with both cores halted), so requested-based
    // MHz is unreliable at large `--step-quantum`.
    let (c0_start, c1_start) = runtime.core_cycles();
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
        // Threaded batch size. `ThreadedEmulator::run_quanta` spawns 4
        // pinned workers per call, so we amortise that startup cost over
        // a large chunk. 1 second of emulated time at the target clock
        // (150 MHz default) is ~150M cycles — well above the few-hundred-
        // microsecond spawn cost, while still letting duration-mode
        // re-check `start.elapsed()` every ~1 virtual second.
        let threaded_chunk_cycles: u64 = sys_clk_hz as u64;
        let mut n: u64 = 0;
        loop {
            if let Some(target) = cycles_target {
                if n >= target {
                    break;
                }
            } else if start.elapsed() >= duration {
                break;
            }
            let chunk = if threaded {
                // Final iteration under --cycles: shrink the chunk so we
                // don't over-run the target by an entire virtual second.
                if let Some(target) = cycles_target {
                    threaded_chunk_cycles.min(target - n)
                } else {
                    threaded_chunk_cycles
                }
            } else {
                qc
            };
            runtime.run(chunk);
            n += chunk;
        }
        n
    } else {
        while start.elapsed() < duration {
            pacer.begin_quantum();
            runtime.run(qc);
            pacer.end_quantum();
        }
        0 // unused
    };

    stats.set_running(false);
    monitor.join().unwrap();
    let (c0_end, c1_end) = runtime.core_cycles();

    // --- Summary ---
    let wall_secs = start.elapsed().as_secs_f64();
    println!("\n--- summary ---");
    println!("Duration:       {:.1} s", wall_secs);
    println!("Workload:       {}", workload.as_str());

    if unpaced {
        // "Cycles executed" = max of per-core deltas. Single-core
        // workloads halt core 1, so its delta is 0 and max reduces to
        // the running core's rate. Dual-core workloads run both cores
        // to roughly the same cycle count, so max ≈ either. This is
        // the real-time gate signal ("can a core sustain 150 MHz?"),
        // not an aggregate throughput number — use `c0+c1` if you
        // want aggregate.
        let c0_delta = c0_end.saturating_sub(c0_start);
        let c1_delta = c1_end.saturating_sub(c1_start);
        let executed = c0_delta.max(c1_delta);
        let requested = unpaced_cycles;
        let mhz = executed as f64 / wall_secs / 1_000_000.0;
        let mhz_requested = requested as f64 / wall_secs / 1_000_000.0;
        // Host TSC ticks are a reasonable proxy for host core cycles on modern
        // x86_64 (invariant TSC runs at a fixed base close to the CPU nominal
        // clock). Under HIGH_PRIORITY_CLASS / TIME_CRITICAL this gives a
        // stable-enough signal for the HLD §12 budget gate.
        let host_ticks_per_emu = pacer.tsc_freq_hz() as f64 * wall_secs / executed.max(1) as f64;
        println!(
            "Executed cyc:   {} (c0={}, c1={})",
            executed, c0_delta, c1_delta
        );
        println!(
            "Requested cyc:  {} (coord-tick advancement; not a perf signal in threaded)",
            requested
        );
        println!(
            "Avg MHz:        {:.1}  (per-core peak, from executed cycles)",
            mhz
        );
        println!(
            "Requested MHz:  {:.1}  (informational — what the run loop asked for)",
            mhz_requested
        );
        // HLD §12's <33 host-cycles/emu-cycle budget was calibrated
        // against `basic` and `fpu-heavy`. The peripheral / contention /
        // stress workloads deliberately do more work per master cycle
        // (PIO tick, GPIO merge, dual-core dispatch) and will always
        // exceed 33 — that's the cost the bench is designed to reveal,
        // not a regression. Show the budget verdict only for the two
        // workloads the gate was calibrated for; emit the raw number
        // informationally for the rest.
        #[cfg(target_arch = "x86_64")]
        {
            let budget_gated = matches!(workload, Workload::Basic | Workload::FpuHeavy);
            if budget_gated {
                println!(
                    "Host/emu cycle: {:.2} (target: <33 per HLD §12)",
                    host_ticks_per_emu
                );
                if host_ticks_per_emu < 33.0 {
                    println!("Budget:         OK ({:.2} < 33)", host_ticks_per_emu);
                } else {
                    println!(
                        "Budget:         OVER ({:.2} >= 33) — investigate regression",
                        host_ticks_per_emu
                    );
                }
            } else {
                println!(
                    "Host/emu cycle: {:.2} (informational; HLD §12 budget only gates basic/fpu-heavy)",
                    host_ticks_per_emu
                );
            }
        }
        #[cfg(not(target_arch = "x86_64"))]
        println!(
            "Host ns/emu cyc:{:>7.2} (informational; HLD §12 x86 TSC budget not applicable)",
            host_ticks_per_emu
        );
        println!("Verdict:        UNPACED (profiling mode)");

        // Threaded-mode --timing table. Runtime must outlive the
        // `last_run_timings()` borrow, so we print before returning.
        #[cfg(all(
            target_arch = "x86_64",
            any(target_os = "windows", target_os = "linux")
        ))]
        if timing
            && let Runtime::Threaded { ref inner, .. } = runtime
            && let Some(rt) = inner.last_run_timings()
        {
            print_timing_summary(rt);
        }
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

/// Print the `--runs N` per-run table + summary line (mean / stdev /
/// cv% / p50 / p99 / min / max). Stats are computed across the
/// measured runs only; the warmup throughput is printed for
/// transparency but excluded from the aggregate.
fn print_runs_summary(_cfg: &RunConfig, throughputs: &[f64]) {
    let n = throughputs.len();
    if n == 0 {
        return;
    }

    println!("\n=== per-run table ===");
    println!("{:>4} {:>8} {:>10}", "run", "label", "MHz");
    println!("{:>4} {:>8} {:>10}", 1, "warmup", "(see above)");
    for (i, mhz) in throughputs.iter().enumerate() {
        println!("{:>4} {:>8} {:>10.1}", i + 2, "run", mhz,);
    }

    let mean = throughputs.iter().copied().sum::<f64>() / n as f64;
    let variance = if n > 1 {
        // Population variance — n, not n-1. The bench is measuring an
        // already-averaged per-run throughput; treating the N samples
        // as the population of "runs we decided to do" matches the
        // HLD's cv% noise-floor usage. For N ≥ 10 the difference is
        // under 5 %; at N=3 Bessel's correction would bias the
        // reported spread upward by 22 %, hiding low-noise runs.
        throughputs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64
    } else {
        0.0
    };
    let stdev = variance.sqrt();
    let cv_pct = if mean > 0.0 {
        100.0 * stdev / mean
    } else {
        0.0
    };

    let mut sorted = throughputs.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p50 = sorted[pct_idx(n, 50)];
    let p99 = sorted[pct_idx(n, 99)];
    let min = sorted[0];
    let max = sorted[n - 1];

    println!("\n=== summary (n={} measured runs, warmup excluded) ===", n);
    println!("mean:  {:>7.1} MHz", mean);
    println!("stdev: {:>7.1} MHz  (cv%: {:.2}%)", stdev, cv_pct);
    println!("p50:   {:>7.1} MHz", p50);
    if n < 100 {
        println!("p99:   {:>7.1} MHz  (n < 100 — approximates max)", p99);
    } else {
        println!("p99:   {:>7.1} MHz", p99);
    }
    println!("min:   {:>7.1} MHz", min);
    println!("max:   {:>7.1} MHz", max);
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
    // Accept both `--workload X` and `--workload=X` forms.
    for (i, a) in args.iter().enumerate() {
        if let Some(v) = a.strip_prefix("--workload=") {
            return match_workload(v);
        }
        if a == "--workload" {
            let v = args.get(i + 1).ok_or(
                "--workload requires basic|peripheral|contention|stress|fpu-heavy|onerom-shape",
            )?;
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
        "fpu-heavy" => Ok(Workload::FpuHeavy),
        // `onerom-shape` is the canonical form; `onerom` is accepted
        // as a deprecated alias so pre-2026-04-24 scripts still run,
        // but emit a warning to stderr pointing at the new name.
        "onerom-shape" => Ok(Workload::OneromShape),
        "onerom" => {
            eprintln!(
                "warning: --workload onerom is deprecated; use onerom-shape. \
                 This variant is a block/SM topology mirror of OneROM, NOT \
                 the real OneROM CPU-mode serve pipeline."
            );
            Ok(Workload::OneromShape)
        }
        other => Err(format!(
            "invalid --workload '{other}' (expected basic|peripheral|contention|stress|fpu-heavy|onerom-shape)"
        )),
    }
}

/// Execution-model selector parsed from `--model`. `Legacy` is the
/// sentinel for "flag absent — defer to the legacy `--threaded`
/// flag". When explicitly set, Serial / Threaded / Both override any
/// `--threaded` presence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ModelSel {
    Legacy,
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
    Ok(ModelSel::Legacy)
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
        threaded: false,
        ..*base_cfg
    };
    let threaded_cfg = RunConfig {
        threaded: true,
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
    let label = if cfg.threaded { "threaded" } else { "serial" };
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
/// zero-based index. Shared by `Stats::from` and
/// `print_runs_summary` so the two can't drift. `pct` is an integer
/// 0–100; callers pass literals like 25, 50, 75, 99.
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

/// Print the per-worker per-quantum timing table populated by
/// `ThreadedEmulator::run_quanta` under `--timing`. Two rows per worker:
/// phase_work (time doing actual phase-1 work before the barrier) and
/// barrier_wait (time blocked in `barrier.wait()`). Ratio column shows
/// barrier_wait / (phase_work + barrier_wait) — high values imply the
/// worker finished early and was waiting for peers.
///
/// Quantum 0's phase_work includes thread-spawn residue (the first
/// `on_wait_entry` closes the span started at worker entry); over a
/// multi-million-quantum run the distortion is noise, but be aware for
/// very short runs.
#[cfg(all(
    target_arch = "x86_64",
    any(target_os = "windows", target_os = "linux")
))]
fn print_timing_summary(rt: &rp2350_emu::threaded::RunTimings) {
    println!("\n--- per-worker timings (ns) ---");
    println!(
        "{:>7} {:>9} {:>10} {:>10} {:>10} {:>10} {:>14} {:>8}",
        "worker", "samples", "mean", "p50", "p99", "max", "total_ns", "wait%"
    );
    for s in rt.summary() {
        let sum = s.phase_work_total_ns + s.barrier_wait_total_ns;
        let wait_pct = if sum > 0 {
            100.0 * s.barrier_wait_total_ns as f64 / sum as f64
        } else {
            0.0
        };
        println!(
            "{:>7} {:>9} {:>10} {:>10} {:>10} {:>10} {:>14}",
            format!("{}:work", s.name().as_str()),
            s.samples,
            s.phase_work_mean_ns,
            s.phase_work_p50_ns,
            s.phase_work_p99_ns,
            s.phase_work_max_ns,
            s.phase_work_total_ns,
        );
        println!(
            "{:>7} {:>9} {:>10} {:>10} {:>10} {:>10} {:>14} {:>7.1}%",
            format!("{}:wait", s.name().as_str()),
            s.samples,
            s.barrier_wait_mean_ns,
            s.barrier_wait_p50_ns,
            s.barrier_wait_p99_ns,
            s.barrier_wait_max_ns,
            s.barrier_wait_total_ns,
            wait_pct,
        );
    }
}

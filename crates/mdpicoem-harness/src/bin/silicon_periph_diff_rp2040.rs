// silicon_periph_diff_rp2040 — peripheral-state oracle for mdrp2040 vs
// real RP2040 silicon.
//
// Phase 0 sub-task 0.E per
// `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V7.md` §4.2 / §4.4.
//
// Same shape as `silicon_periph_diff_rp2350` but:
//
//   * Probe-rs target is `rp2040` (chip-pack name).
//   * Emulator is `mdrp2040::Emulator` (Cortex-M0+).
//   * Timing window uses **SysTick** (Cortex-M0+ has no DWT CYCCNT per
//     `probe_diff_rp2040.rs`). `[min_sysclks, max_sysclks]` is a soft
//     assertion per HLD §4.2 — outside the window is a FAIL with a
//     "window" message, inside the window the observables determine the
//     verdict.
//   * `PeriphScenario` is defined locally (not reused from
//     `silicon_scenarios`) — HLD §4.2 calls for `min_sysclks` which the
//     RP2350 struct lacks; defining a local type avoids cross-chip
//     coupling for a shape that nearly matches but is not identical.
//
// Running without a probe attached falls out of `Session::auto_attach`
// as a fatal error with the usual probe-rs message — same behaviour as
// `silicon_periph_diff_rp2350`.

use mdpicoem_harness::silicon_oracle::Verdict;
use mdpicoem_harness::{EMU_TEST_STACK, SILICON_RUN_SLED};
use mdrp2040::{Config, EmulatorBuilder};
use probe_rs::{Core, MemoryInterface, RegisterId, Session, SessionConfig};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// RP2040 absolute MMIO bases and bit constants (datasheet §2.2 / §2.14)
// ---------------------------------------------------------------------------

const RESETS_BASE: u32 = 0x4000_C000;
const RESETS_RESET: u32 = RESETS_BASE + 0x00;

/// APB alias offset. `+0x3000` = CLR (AND NOT). SET (`+0x2000`) is
/// unused by Phase 0 — only RESETS *release* (CLR) matters here.
const ALIAS_CLR: u32 = 0x3000;

/// RESETS bits (RP2040 datasheet §2.14 Table 26 ordering — differs from
/// RP2350). Only the subset this binary touches is enumerated.
const RESET_IO_BANK0: u32 = 1 << 5;
const RESET_PADS_BANK0: u32 = 1 << 8;
const RESET_PIO0: u32 = 1 << 10;
const RESET_PIO1: u32 = 1 << 11;
const RESET_PLL_SYS: u32 = 1 << 12;
const RESET_TIMER: u32 = 1 << 21;

/// SIO block (0xD000_0000). RP2040 uses 4-byte spacing for OUT/OE SET /
/// CLR / XOR, unlike RP2350's 8-byte spacing.
const SIO_GPIO_IN: u32 = 0xD000_0004;
const SIO_GPIO_OUT: u32 = 0xD000_0010;
const SIO_GPIO_OUT_SET: u32 = 0xD000_0014;
const SIO_GPIO_OUT_XOR: u32 = 0xD000_001C;
const SIO_GPIO_OE: u32 = 0xD000_0020;
const SIO_GPIO_OE_SET: u32 = 0xD000_0024;

/// IO_BANK0 GPIO25 CTRL register (SDK LED pin on Pico 1 boards).
const IO_BANK0_GPIO25_CTRL: u32 = 0x4001_4000 + 25 * 0x08 + 0x04;
/// PADS_BANK0 GPIO25 pad control (pad enable + drive strength).
const PADS_BANK0_GPIO25: u32 = 0x4001_C000 + (25 + 1) * 0x04;

/// TIMER block — `TIMERAWL` at offset 0x28 (read-only free-running lower
/// 32 bits). Used by the `GAP_TIMER_UNMODELLED` scenario.
const TIMER_BASE: u32 = 0x4005_4000;
const TIMER_TIMERAWL: u32 = TIMER_BASE + 0x28;

/// XOSC block. `CTRL` at offset 0, writable by firmware — used by the
/// `SMOKE_XOSC_CTRL_WRITE_ROUND_TRIP` scenario.
const XOSC_BASE: u32 = 0x4002_4000;
const XOSC_CTRL: u32 = XOSC_BASE + 0x00;

/// SysTick (ARMv6-M SCS). The same PPB constants apply on M0+ and M33.
/// `_CSR.CLKSOURCE|ENABLE` = bit 2 | bit 0; TICKINT intentionally off so
/// the count-to-zero event does not pend the SysTick exception.
const SYST_CSR: u32 = 0xE000_E010;
const SYST_RVR: u32 = 0xE000_E014;
const SYST_CVR: u32 = 0xE000_E018;
const SYST_CSR_CLKSOURCE_ENABLE: u32 = 0x5; // CLKSOURCE=1, ENABLE=1, TICKINT=0
/// SysTick reload = 2^24 - 1 (the hardware maximum for a 24-bit
/// downcounter). The widest Phase 0 scenario settles in well under 10k
/// sysclks, so a single 24-bit window is never exhausted.
const SYST_RVR_MAX: u32 = 0x00FF_FFFF;

// ---------------------------------------------------------------------------
// Scenario type (local — shape differs from RP2350's via `min_sysclks`)
// ---------------------------------------------------------------------------

/// A peripheral oracle scenario for the RP2040 variant. Runner applies
/// `setup` in order on both the HW `Core` and a fresh `mdrp2040::Emulator`,
/// runs until BKPT (or cycle budget), then reads `observe` and the GPIO
/// bitmask `observe_pins`. Timing window `[min_sysclks, max_sysclks]` is
/// a soft assertion — reported, and a FAIL verdict if the HW-measured
/// window falls outside it (HLD §4.2).
pub struct PeriphScenario {
    pub name: &'static str,
    /// `(absolute_addr, value)` — applied with zero-cycle bus writes on
    /// EMU and via probe-rs `write_word_32` on HW.
    pub setup: &'static [(u32, u32)],
    /// Upper bound on sysclks (inclusive).
    pub max_sysclks: u32,
    /// Lower bound on sysclks (inclusive). HLD §4.2 addition vs RP2350.
    pub min_sysclks: u32,
    /// `(absolute_addr, mask)` — `0xFFFF_FFFF` = full word.
    pub observe: &'static [(u32, u32)],
    /// GPIO pins to sample drive+level. 0 = skip pins.
    pub observe_pins: u32,
    /// If `Some(bytes)`, the runner uploads these as the sled instead
    /// of auto-assembling a countdown. Must end in `bkpt #0` (`0xBE00`).
    pub custom_sled: Option<&'static [u8]>,
}

// ---------------------------------------------------------------------------
// Scenario catalogue — 3 smoke + 1 genuine red-path. V7 HLD §4.4.3
// advertised "three deliberately-broken scenarios"; devil's-advocate
// review in Phase 0 Wave 3 found only `GAP_TIMER_UNMODELLED` is
// genuinely red today. The other two are green round-trips that
// validate the oracle's PASS path — still useful coverage, but not
// red-path witnesses. Phase 0 exit criterion 3 ("oracle CAN report
// FAIL") is satisfied by `GAP_TIMER_UNMODELLED`.
// ---------------------------------------------------------------------------

// SIO_GPIO_TOGGLE_BASIC — release RESETS for the pad fabric and SIO,
// configure GPIO25 as a SIO output, drive it high, then XOR its value
// four times so the final state is `1`. Known-good — both HW and EMU
// must agree on the final SIO_GPIO_OUT and the sampled pin level.
const GPIO25: u32 = 1 << 25;
const S_SIO_GPIO_TOGGLE_BASIC: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_IO_BANK0 | RESET_PADS_BANK0),
    // PADS_BANK0 GPIO25: IE=1 (bit 6), drive=4 mA (bits 5:4 = 01), no PU/PD.
    (PADS_BANK0_GPIO25, 0x0000_0056),
    // IO_BANK0 GPIO25_CTRL: FUNCSEL=5 (SIO), everything else default.
    (IO_BANK0_GPIO25_CTRL, 0x0000_0005),
    // Enable output, drive high, then XOR four times → final = 1.
    (SIO_GPIO_OE_SET, GPIO25),
    (SIO_GPIO_OUT_SET, GPIO25),
    (SIO_GPIO_OUT_XOR, GPIO25), // toggle 1→0
    (SIO_GPIO_OUT_XOR, GPIO25), // 0→1
    (SIO_GPIO_OUT_XOR, GPIO25), // 1→0
    (SIO_GPIO_OUT_XOR, GPIO25), // 0→1 (final)
];
const O_SIO_GPIO_TOGGLE_BASIC: &[(u32, u32)] = &[
    // GPIO_OUT must match the four-toggle result (bit 25 set).
    (SIO_GPIO_OUT, GPIO25),
    // GPIO_OE must match the explicit OE_SET write (bit 25 set).
    (SIO_GPIO_OE, GPIO25),
];

// SMOKE_GPIO_BIT24_ROUND_TRIP — baseline smoke that exercises the SIO
// OE_SET/OUT_SET path at bit 24 instead of bit 25, then reads back
// GPIO_OUT masked at bit 24. Both HW and EMU apply the same writes and
// both read the same value back — this is a *green round-trip*, not a
// red path. It sits in the catalogue as a second, independent smoke of
// the SIO_GPIO_OUT dispatch (GPIO25 is covered by SIO_GPIO_TOGGLE_BASIC)
// so a bit-dropping or bit-routing emulator regression at a different
// pin index would surface even if GPIO25 still worked.
//
// HLD note (V7 §4.4.3 called this "deliberately broken"): the original
// intent was bit-24 writes diverging from a bit-25 observe. Once both
// sides applied the same write the round-trip is symmetric and
// therefore green. Renamed to reflect reality — Phase 0's red-path
// witness is `GAP_TIMER_UNMODELLED` below.
const GPIO24: u32 = 1 << 24;
const S_SMOKE_GPIO_BIT24_ROUND_TRIP: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_IO_BANK0 | RESET_PADS_BANK0),
    (SIO_GPIO_OE_SET, GPIO24),
    (SIO_GPIO_OUT_SET, GPIO24),
];
const O_SMOKE_GPIO_BIT24_ROUND_TRIP: &[(u32, u32)] = &[
    (SIO_GPIO_OUT, GPIO24),
];

// GAP_TIMER_UNMODELLED — divergence-inducing scenario expected to FAIL
// today; will PASS after Phase 1 lands TIMER. Serves as Phase 0's
// red-path-validation witness: proves the oracle can report FAIL when
// HW and EMU genuinely disagree.
//
// Setup releases TIMER from reset so silicon starts counting. The
// emulator returns 0 from TIMERAWL (the `TimerRegs` peripheral is
// unimplemented on RP2040 per the Phase 1 HLD). On real silicon
// TIMERAWL advances between the setup write and the observe read, so
// HW reads non-zero while EMU reads zero → FAIL.
//
// The observe mask covers bits [23:0] to tolerate the upper-8-bit-
// ignore convention silicon sometimes uses for partial TIMERAWL reads;
// any non-zero value in the low 24 bits on silicon diverges from the
// emulator's zero.
const S_GAP_TIMER_UNMODELLED: &[(u32, u32)] = &[
    (RESETS_RESET + ALIAS_CLR, RESET_TIMER),
];
const O_GAP_TIMER_UNMODELLED: &[(u32, u32)] = &[
    (TIMER_TIMERAWL, 0x00FF_FFFF),
];

// SMOKE_XOSC_CTRL_WRITE_ROUND_TRIP — baseline smoke that writes a
// sentinel value into XOSC_CTRL and reads it back. Both HW and EMU
// honour the write (EMU dispatches to `xosc_regs.write32` per
// `mdrp2040/src/bus/mod.rs:569`), so this is a *green round-trip*, not
// a red path. It validates the XOSC register dispatch end-to-end on
// both sides.
//
// The sentinel `0x00FA_BABE` picks an arbitrary-but-distinctive pattern
// in the writable bits of XOSC_CTRL (the low 24 bits — ENABLE,
// FREQ_RANGE and the rest of the field). The observe mask is
// `0x00FF_FFFF` so the STATUS block at bits [31:24] — which neither
// emulator nor silicon guarantees under partial-reset — does not
// contribute to the diff.
//
// HLD note (V7 §4.4.3 called this "deliberately broken"): the original
// intent was that the emulator might swallow XOSC_CTRL writes. It does
// not — the dispatch path landed in Phase 0 Wave 2. Renamed to reflect
// reality.
const S_SMOKE_XOSC_CTRL_WRITE_ROUND_TRIP: &[(u32, u32)] = &[
    (XOSC_CTRL, 0x00FA_BABE),
];
const O_SMOKE_XOSC_CTRL_WRITE_ROUND_TRIP: &[(u32, u32)] = &[
    (XOSC_CTRL, 0x00FF_FFFF),
];

/// Phase 0 catalogue. V7 HLD §4.4.3 required four scenarios; devil's-
/// advocate review classified three as green round-trip smoke and one
/// (`GAP_TIMER_UNMODELLED`) as a genuine red-path witness. See the
/// per-scenario comments above for the re-classification rationale.
pub const SCENARIOS: &[PeriphScenario] = &[
    PeriphScenario {
        name: "SIO_GPIO_TOGGLE_BASIC",
        setup: S_SIO_GPIO_TOGGLE_BASIC,
        max_sysclks: 500,
        min_sysclks: 50,
        observe: O_SIO_GPIO_TOGGLE_BASIC,
        observe_pins: GPIO25,
        custom_sled: None,
    },
    PeriphScenario {
        name: "SMOKE_GPIO_BIT24_ROUND_TRIP",
        setup: S_SMOKE_GPIO_BIT24_ROUND_TRIP,
        max_sysclks: 500,
        min_sysclks: 20,
        observe: O_SMOKE_GPIO_BIT24_ROUND_TRIP,
        observe_pins: GPIO24,
        custom_sled: None,
    },
    PeriphScenario {
        name: "GAP_TIMER_UNMODELLED",
        setup: S_GAP_TIMER_UNMODELLED,
        max_sysclks: 500,
        min_sysclks: 20,
        observe: O_GAP_TIMER_UNMODELLED,
        observe_pins: 0,
        custom_sled: None,
    },
    PeriphScenario {
        name: "SMOKE_XOSC_CTRL_WRITE_ROUND_TRIP",
        setup: S_SMOKE_XOSC_CTRL_WRITE_ROUND_TRIP,
        max_sysclks: 500,
        min_sysclks: 20,
        observe: O_SMOKE_XOSC_CTRL_WRITE_ROUND_TRIP,
        observe_pins: 0,
        custom_sled: None,
    },
];

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

const USAGE: &str = "\
Usage: silicon_periph_diff_rp2040 [--filter <substr>] [--verbose]

Options:
  --filter   Only run scenarios whose name contains <substr>
  --verbose  Print per-observable diffs, not just the first divergence
";

#[derive(Clone, Debug, Default)]
struct Args {
    filter: Option<String>,
    verbose: bool,
}

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut args = Args::default();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--filter" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--filter requires a substring\n{USAGE}"));
                }
                args.filter = Some(argv[i].clone());
            }
            "--verbose" => args.verbose = true,
            "--help" | "-h" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument '{other}'\n{USAGE}")),
        }
        i += 1;
    }
    Ok(args)
}

// ---------------------------------------------------------------------------
// Runner
// ---------------------------------------------------------------------------

const PC_REG: RegisterId = RegisterId(15);
const XPSR_REG: RegisterId = RegisterId(16);
const SP_REG: RegisterId = RegisterId(13);
const LR_REG: RegisterId = RegisterId(14);

const BKPT_TIMEOUT: Duration = Duration::from_secs(5);

/// Reject any sled that isn't terminated by a `bkpt #0` (encoded as
/// Thumb halfword `0xBE00`, little-endian `[0x00, 0xBE]`). Same contract
/// as the RP2350 `validate_custom_sled` — kept local to avoid a cross-
/// chip import that would pull in the RP2350 scenario catalogue.
fn validate_custom_sled(bytes: &[u8]) -> Result<&[u8], String> {
    if bytes.is_empty() {
        return Err("custom sled is empty".to_string());
    }
    if bytes.len() < 2 {
        return Err(format!(
            "custom sled must be at least one halfword (got {} bytes)",
            bytes.len()
        ));
    }
    if !bytes.len().is_multiple_of(2) {
        return Err(format!(
            "custom sled must be a whole number of halfwords (got {} bytes)",
            bytes.len()
        ));
    }
    let n = bytes.len();
    if bytes[n - 2] != 0x00 || bytes[n - 1] != 0xBE {
        return Err(format!(
            "custom sled must end in `bkpt #0` (0xBE00); last halfword \
             is 0x{:02X}{:02X}",
            bytes[n - 1],
            bytes[n - 2],
        ));
    }
    Ok(bytes)
}

/// Build the default countdown sled bytes for `max_sysclks`.
///
/// Thumb-16 only (M0+ ISA): `movs r0, #N / subs r0, #1 / bne -4 / bkpt #0`.
/// N = ceil(max_sysclks / 4), clamped to `[1, 255]` because `movs rd, #imm8`
/// (T1) is the only immediate-move encoding guaranteed on Cortex-M0+; for
/// the small windows this catalogue uses (≤ 1020) that is always enough.
fn assemble_sled(max_sysclks: u32) -> Vec<u8> {
    let mut n = max_sysclks.div_ceil(4);
    if n == 0 {
        n = 1;
    }
    if n > 0xFF {
        n = 0xFF;
    }
    // movs r0, #n      — T2 encoding: 0x2000 | (Rd << 8) | imm8. Rd=0.
    let movs = 0x2000u16 | (n as u16 & 0xFF);
    // subs r0, #1      — T2 encoding: 0x3800 | (Rdn << 8) | imm8.  Rdn=0.
    let subs = 0x3800u16 | 0x0001;
    // bne -6           — T1 encoding: 0xD100 | imm8.  imm8 is signed and
    // scaled by 2: target = PC + imm8*2, where PC = branch_addr + 4. The
    // bne sits at byte 4 (hw[2]), so PC = 8; we want to branch back to
    // the subs at byte 2 (hw[1]). target - PC = 2 - 8 = -6, so imm8 = -3
    // = 0xFD. Matches the canonical sled pattern in `silicon_scenarios.rs`
    // (see SLED_CLOCK_PLL_SYS_REPROGRAM_MID_RUN_HW at lines 307, 318, 380,
    // 393, 571). Previous `0xD1FE` (imm8 = -2) branched to the bne itself
    // and hung HW forever.
    let bne = 0xD1FDu16;
    // bkpt #0
    let bkpt = 0xBE00u16;

    let halfwords = [movs, subs, bne, bkpt];
    let mut out = Vec::with_capacity(halfwords.len() * 2);
    for hw in halfwords {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    out
}

/// Release RESETS for the peripherals Phase 0 scenarios touch. Matches
/// the RP2350 `release_common_resets` shape.
fn release_common_resets(core: &mut Core) -> Result<(), probe_rs::Error> {
    let state: u32 = core.read_word_32(RESETS_RESET as u64)?;
    let cleared = state
        & !(RESET_PIO0
            | RESET_PIO1
            | RESET_PLL_SYS
            | RESET_IO_BANK0
            | RESET_PADS_BANK0
            | RESET_TIMER);
    core.write_word_32(RESETS_RESET as u64, cleared)?;
    Ok(())
}

/// Program SysTick for the scenario timing window. CLKSOURCE=processor,
/// ENABLE=1, TICKINT=0 so count-to-zero does not pend the SysTick
/// exception. Reload = 0x00FF_FFFF (max 24-bit). A single 24-bit window
/// comfortably covers every Phase 0 scenario (widest = 500 sysclks).
fn program_systick_hw(core: &mut Core) -> Result<(), probe_rs::Error> {
    // Disable first so a subsequent RVR write is guaranteed to land.
    core.write_word_32(SYST_CSR as u64, 0)?;
    core.write_word_32(SYST_RVR as u64, SYST_RVR_MAX)?;
    // Writing any value to CVR clears it to zero.
    core.write_word_32(SYST_CVR as u64, 0)?;
    core.write_word_32(SYST_CSR as u64, SYST_CSR_CLKSOURCE_ENABLE)?;
    Ok(())
}

fn read_systick_cvr(core: &mut Core) -> Result<u32, probe_rs::Error> {
    core.read_word_32(SYST_CVR as u64)
}

fn apply_setup_hw(core: &mut Core, setup: &[(u32, u32)]) -> Result<(), probe_rs::Error> {
    for &(addr, val) in setup {
        core.write_word_32(addr as u64, val)?;
    }
    Ok(())
}

fn sample_pins_hw(core: &mut Core, mask: u32) -> Result<(u32, u32), probe_rs::Error> {
    let oe: u32 = core.read_word_32(SIO_GPIO_OE as u64)?;
    let in_: u32 = core.read_word_32(SIO_GPIO_IN as u64)?;
    Ok((oe & mask, in_ & mask))
}

fn sample_pins_emu(emu: &mut mdrp2040::Emulator, mask: u32) -> (u32, u32) {
    let oe = emu.mmio_read32(SIO_GPIO_OE) & mask;
    let in_ = emu.mmio_read32(SIO_GPIO_IN) & mask;
    (oe, in_)
}

fn run_sled_hw(core: &mut Core) -> Result<(), Box<dyn std::error::Error>> {
    core.write_core_reg(PC_REG, SILICON_RUN_SLED)?;
    core.write_core_reg(XPSR_REG, 0x0100_0000u32)?; // T=1 (Thumb)
    core.write_core_reg(SP_REG, EMU_TEST_STACK)?;
    core.write_core_reg(LR_REG, 0xFFFF_FFFFu32)?;
    core.run()?;

    let deadline = Instant::now() + BKPT_TIMEOUT;
    loop {
        if core.status()?.is_halted() {
            return Ok(());
        }
        if Instant::now() > deadline {
            let _ = core.halt(Duration::from_millis(200));
            let pc: u32 = core.read_core_reg(PC_REG).unwrap_or(0xDEAD_BEEF);
            let sp: u32 = core.read_core_reg(SP_REG).unwrap_or(0xDEAD_BEEF);
            let lr: u32 = core.read_core_reg(LR_REG).unwrap_or(0xDEAD_BEEF);
            return Err(format!(
                "BKPT timeout: PC=0x{pc:08X} SP=0x{sp:08X} LR=0x{lr:08X}"
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// Per-scenario rich result.
struct ScenarioResult {
    name: &'static str,
    verdict: Verdict,
    window_sysclks: u32,
    first_divergence: Option<String>,
    elapsed: Duration,
}

fn run_scenario(
    core: &mut Core,
    sc: &PeriphScenario,
    first_scenario: bool,
    verbose: bool,
) -> Result<ScenarioResult, Box<dyn std::error::Error>> {
    let t0 = Instant::now();

    if first_scenario {
        core.reset_and_halt(Duration::from_millis(500))?;
    } else if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }
    release_common_resets(core)?;

    // Program SysTick once the peripherals are released, then sample
    // CVR_start right before the setup writes. Anything that happens
    // after this sample — setup writes, sled execution — counts toward
    // the window.
    program_systick_hw(core)?;
    let cvr_start = read_systick_cvr(core)?;

    apply_setup_hw(core, sc.setup)?;

    let owned_sled: Vec<u8>;
    let sled_bytes: &[u8] = match sc.custom_sled {
        Some(bytes) => {
            validate_custom_sled(bytes).map_err(|e| format!("scenario '{}': {e}", sc.name))?
        }
        None => {
            owned_sled = assemble_sled(sc.max_sysclks);
            &owned_sled
        }
    };
    core.write_8(SILICON_RUN_SLED as u64, sled_bytes)?;
    run_sled_hw(core)?;
    // Sample CVR the moment the core is halted. SysTick stops counting
    // while the core is halted (DAP-halt freezes the downcounter) so
    // this captures the steady-state end-of-window value.
    let cvr_end = read_systick_cvr(core)?;

    // SysTick counts *down*, so window = start - end (unwrapped — Phase 0
    // scenarios are well below the 24-bit reload so no wrap expected).
    let window_sysclks = cvr_start.saturating_sub(cvr_end);

    let hw_obs: Vec<u32> = sc
        .observe
        .iter()
        .map(|(addr, _m)| core.read_word_32(*addr as u64))
        .collect::<Result<_, _>>()?;
    let hw_pins = if sc.observe_pins != 0 {
        Some(sample_pins_hw(core, sc.observe_pins)?)
    } else {
        None
    };

    // Emulator side — fresh builder per scenario so PRIMASK/CONTROL/
    // pending-fault state cannot leak between cases.
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build().expect("Serial build is infallible");
    emu.core_mut(1).halt();
    for &(addr, val) in sc.setup {
        emu.mmio_write32(addr, val);
    }

    if let Some(bytes) = sc.custom_sled {
        let vetted: &[u8] = validate_custom_sled(bytes)
            .map_err(|e| format!("scenario '{}': {e}", sc.name))?;
        emu.load_image(SILICON_RUN_SLED, vetted);
        {
            let c = emu.core_mut(0);
            c.wake();
            c.set_reg(13, EMU_TEST_STACK); // SP
            c.set_reg(14, 0xFFFF_FFFF); // LR
            c.regs.set_pc(SILICON_RUN_SLED);
            c.regs.xpsr = 0x0100_0000; // T=1 (Thumb)
        }
        let bkpt_pc = SILICON_RUN_SLED + (vetted.len() as u32) - 2;
        let start = emu.cycles();
        let budget = window_sysclks as u64;
        while emu.core(0).regs.pc() != bkpt_pc
            && emu.cycles().saturating_sub(start) < budget
        {
            emu.step().expect("Serial step is infallible");
        }
        let overshot = emu.core(0).regs.pc() != bkpt_pc;
        if overshot && verbose {
            println!(
                "    warn scenario '{}': EMU exhausted {}-cycle budget before \
                 reaching BKPT at PC=0x{:08X} (PC=0x{:08X})",
                sc.name,
                budget,
                bkpt_pc,
                emu.core(0).regs.pc(),
            );
        }
        emu.core_mut(0).halt();
    } else {
        // Default path: halt cores, advance bus/peripheral state by the
        // HW-measured window so both sides see the same cycle count.
        emu.core_mut(0).halt();
        emu.run(window_sysclks as u64).expect("Serial run is infallible");
    }

    let emu_obs: Vec<u32> =
        sc.observe.iter().map(|(addr, _m)| emu.mmio_read32(*addr)).collect();
    let emu_pins = if sc.observe_pins != 0 {
        Some(sample_pins_emu(&mut emu, sc.observe_pins))
    } else {
        None
    };

    // First-divergence accumulation — stop at first hit but keep scanning
    // in verbose mode so every observable is logged.
    let mut first_div: Option<String> = None;

    // Soft-assertion timing window per HLD §4.2. Checked first so a
    // timing-violation divergence takes precedence over MMIO diffs the
    // wrong window could have produced transitively.
    if window_sysclks < sc.min_sysclks || window_sysclks > sc.max_sysclks {
        let msg = format!(
            "window {} sysclks outside [{}, {}]",
            window_sysclks, sc.min_sysclks, sc.max_sysclks,
        );
        first_div = Some(msg.clone());
        if verbose {
            println!("    DIFF {msg}");
        }
    } else if verbose {
        println!(
            "    ok   window {} sysclks in [{}, {}]",
            window_sysclks, sc.min_sysclks, sc.max_sysclks,
        );
    }

    for (i, (addr, mask)) in sc.observe.iter().enumerate() {
        let h = hw_obs[i] & *mask;
        let e = emu_obs[i] & *mask;
        if h != e {
            let msg = format!(
                "MMIO 0x{:08X} mask=0x{:08X}: HW=0x{:08X} EMU=0x{:08X} (xor=0x{:08X})",
                addr,
                mask,
                h,
                e,
                h ^ e,
            );
            if first_div.is_none() {
                first_div = Some(msg.clone());
            }
            if verbose {
                println!("    DIFF {msg}");
            }
        } else if verbose {
            println!("    ok   MMIO 0x{:08X} mask=0x{:08X}: 0x{:08X}", addr, mask, h);
        }
    }
    if let (Some(h), Some(e)) = (hw_pins, emu_pins) {
        if h != e {
            let msg = format!(
                "GPIO mask=0x{:08X}: HW oe=0x{:08X} level=0x{:08X}, \
                 EMU oe=0x{:08X} level=0x{:08X}",
                sc.observe_pins, h.0, h.1, e.0, e.1,
            );
            if first_div.is_none() {
                first_div = Some(msg.clone());
            }
            if verbose {
                println!("    DIFF {msg}");
            }
        } else if verbose {
            println!(
                "    ok   GPIO mask=0x{:08X}: oe=0x{:08X} level=0x{:08X}",
                sc.observe_pins, h.0, h.1
            );
        }
    }

    let verdict = if first_div.is_none() { Verdict::Pass } else { Verdict::Fail };
    Ok(ScenarioResult {
        name: sc.name,
        verdict,
        window_sysclks,
        first_divergence: first_div,
        elapsed: t0.elapsed(),
    })
}

/// Retry-once wrapper. The only probe-rs error kinds we retry on are the
/// transient ones: `Probe` (DebugProbeError — USB disconnect / buffer
/// drain stalls) and `Timeout` (ARM DAP timeout). Everything else is a
/// hard fail on the first attempt.
///
/// On retry, we pause briefly to let the probe's internal queue drain
/// before kicking off the next scenario's reset_and_halt.
fn run_scenario_with_retry(
    core: &mut Core,
    sc: &PeriphScenario,
    first_scenario: bool,
    verbose: bool,
) -> Result<ScenarioResult, Box<dyn std::error::Error>> {
    match run_scenario(core, sc, first_scenario, verbose) {
        Ok(r) => Ok(r),
        Err(e) => {
            if is_transient_probe_error(e.as_ref()) {
                eprintln!(
                    "  scenario '{}': transient probe error, retrying once: {e}",
                    sc.name,
                );
                std::thread::sleep(Duration::from_millis(250));
                run_scenario(core, sc, first_scenario, verbose)
            } else {
                Err(e)
            }
        }
    }
}

/// Strict error-kind match: retry only on `probe_rs::Error::Probe` and
/// `probe_rs::Error::Timeout`. Anything else — including `Arm` errors,
/// `ChipNotFound`, memory-alignment errors — is a hard fail.
///
/// `'static` bound on the trait object is required because `Any::downcast_ref`
/// (pulled in via `Error::downcast_ref`) can only work on types that don't
/// borrow from shorter-lived scopes.
fn is_transient_probe_error(e: &(dyn std::error::Error + 'static)) -> bool {
    if let Some(pe) = e.downcast_ref::<probe_rs::Error>() {
        matches!(pe, probe_rs::Error::Probe(_) | probe_rs::Error::Timeout)
    } else {
        false
    }
}

fn main() {
    mdpicoem_harness::harness_tracing_init();
    match run() {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("fatal: {e}");
            std::process::exit(2);
        }
    }
}

fn run() -> Result<i32, Box<dyn std::error::Error>> {
    let args = parse_args().map_err(|e| {
        eprintln!("{e}");
        "bad arguments"
    })?;

    let selected: Vec<&PeriphScenario> = SCENARIOS
        .iter()
        .filter(|s| args.filter.as_deref().is_none_or(|sub| s.name.contains(sub)))
        .collect();

    let skipped = SCENARIOS.len() - selected.len();
    if selected.is_empty() {
        println!(
            "silicon_periph_diff_rp2040: no scenarios match filter '{}'; nothing to do",
            args.filter.as_deref().unwrap_or(""),
        );
        return Ok(0);
    }

    println!(
        "silicon_periph_diff_rp2040: {} scenario(s) selected ({} skipped by filter)",
        selected.len(),
        skipped,
    );
    println!(
        "sled=0x{:08X} resets=0x{:08X} timer=0x{:08X} xosc=0x{:08X}",
        SILICON_RUN_SLED, RESETS_BASE, TIMER_BASE, XOSC_BASE,
    );
    println!();

    let mut session = Session::auto_attach("rp2040", SessionConfig::default())?;
    let mut core = session.core(0)?;

    println!(
        "{:<28} {:>6} {:>10} {:>7}  {}",
        "scenario", "sysclk", "runtime_ms", "verdict", "first_divergence",
    );
    println!("{}", "-".repeat(98));

    let mut pass = 0usize;
    let mut fail = 0usize;
    let t_total = Instant::now();
    for (i, sc) in selected.iter().enumerate() {
        let r = run_scenario_with_retry(&mut core, sc, i == 0, args.verbose)?;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail => fail += 1,
        }
        println!(
            "{:<28} {:>6} {:>10.1} {:>7}  {}",
            r.name,
            r.window_sysclks,
            r.elapsed.as_secs_f64() * 1000.0,
            r.verdict.as_str(),
            r.first_divergence.as_deref().unwrap_or("-"),
        );
    }

    // Cleanup contract — mirror RP2350 version: re-assert RESETS for the
    // peripherals the catalogue cleared so a subsequent invocation sees
    // default state. Cleanup failures go to stderr; they don't change the
    // exit code since the test results are already reported.
    if let Err(e) = core.halt(Duration::from_millis(200)) {
        eprintln!("warning: periph cleanup halt failed: {e}");
    }
    match core.read_word_32(RESETS_RESET as u64) {
        Ok(state) => {
            let bits = RESET_PIO0
                | RESET_PIO1
                | RESET_PLL_SYS
                | RESET_IO_BANK0
                | RESET_PADS_BANK0
                | RESET_TIMER;
            if let Err(e) = core.write_word_32(RESETS_RESET as u64, state | bits) {
                eprintln!("warning: periph cleanup RESETS write failed: {e}");
            }
        }
        Err(e) => {
            eprintln!("warning: periph cleanup RESETS read failed: {e}");
        }
    }

    println!();
    println!(
        "summary: total={} pass={} fail={} skipped={}  ({:.2}s)",
        selected.len(),
        pass,
        fail,
        skipped,
        t_total.elapsed().as_secs_f64(),
    );
    Ok(if fail > 0 { 1 } else { 0 })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn is_mmio(addr: u32) -> bool {
        (0x4000_0000..0x6000_0000).contains(&addr)
            || (0xD000_0000..0xE000_0000).contains(&addr)
    }

    #[test]
    fn catalogue_has_four_scenarios() {
        // V7 HLD §4.4.3 called for four scenarios. Post-review composition:
        // three smoke / round-trip + one genuine red path (GAP_TIMER_UNMODELLED).
        assert_eq!(SCENARIOS.len(), 4);
    }

    #[test]
    fn catalogue_has_baseline_smoke_and_gap_witness() {
        let names: Vec<&str> = SCENARIOS.iter().map(|s| s.name).collect();
        assert!(names.iter().any(|n| n.starts_with("SIO_GPIO_TOGGLE")));
        assert!(names.iter().any(|n| n.starts_with("SMOKE_GPIO")));
        assert!(names.iter().any(|n| n.starts_with("GAP_TIMER")));
        assert!(names.iter().any(|n| n.starts_with("SMOKE_XOSC")));
    }

    #[test]
    fn setup_addresses_all_mmio() {
        for sc in SCENARIOS {
            for (i, (a, _)) in sc.setup.iter().enumerate() {
                assert!(is_mmio(*a), "{} setup[{}] 0x{:08X}", sc.name, i, a);
            }
            for (i, (a, _)) in sc.observe.iter().enumerate() {
                assert!(is_mmio(*a), "{} observe[{}] 0x{:08X}", sc.name, i, a);
            }
        }
    }

    #[test]
    fn observe_masks_or_pins_nonzero() {
        for sc in SCENARIOS {
            let any = sc.observe.iter().any(|(_, m)| *m != 0) || sc.observe_pins != 0;
            assert!(any, "scenario '{}' observes nothing", sc.name);
        }
    }

    #[test]
    fn sysclk_bounds_positive_and_ordered() {
        for sc in SCENARIOS {
            assert!(sc.max_sysclks > 0, "'{}' has max_sysclks=0", sc.name);
            assert!(sc.min_sysclks <= sc.max_sysclks,
                "'{}' min_sysclks {} > max_sysclks {}",
                sc.name, sc.min_sysclks, sc.max_sysclks);
        }
    }

    #[test]
    fn scenario_names_unique() {
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        for sc in SCENARIOS {
            assert!(seen.insert(sc.name), "duplicate name '{}'", sc.name);
        }
    }

    #[test]
    fn sio_addresses_match_rp2040_layout() {
        // RP2040 has 4-byte spacing for OUT/OE SET/CLR/XOR (not RP2350's
        // 8-byte). Catch a copy-paste regression at test time.
        assert_eq!(SIO_GPIO_IN, 0xD000_0004);
        assert_eq!(SIO_GPIO_OUT, 0xD000_0010);
        assert_eq!(SIO_GPIO_OUT_SET, 0xD000_0014);
        assert_eq!(SIO_GPIO_OUT_XOR, 0xD000_001C);
        assert_eq!(SIO_GPIO_OE, 0xD000_0020);
        assert_eq!(SIO_GPIO_OE_SET, 0xD000_0024);
    }

    #[test]
    fn resets_bits_match_rp2040_layout() {
        // RP2040 datasheet §2.14 Table 26 ordering — guard against accidental
        // reuse of RP2350's RESETS bit layout.
        assert_eq!(RESET_IO_BANK0, 1 << 5);
        assert_eq!(RESET_PADS_BANK0, 1 << 8);
        assert_eq!(RESET_PIO0, 1 << 10);
        assert_eq!(RESET_PIO1, 1 << 11);
        assert_eq!(RESET_PLL_SYS, 1 << 12);
        assert_eq!(RESET_TIMER, 1 << 21);
    }

    #[test]
    fn validate_custom_sled_rejects_empty() {
        assert!(validate_custom_sled(&[]).is_err());
    }

    #[test]
    fn validate_custom_sled_rejects_missing_terminator() {
        let bad: &[u8] = &[0x00, 0xBF, 0x00, 0xBF];
        assert!(validate_custom_sled(bad).is_err());
    }

    #[test]
    fn validate_custom_sled_accepts_bare_bkpt() {
        let ok: &[u8] = &[0x00, 0xBE];
        assert!(validate_custom_sled(ok).is_ok());
    }

    #[test]
    fn assemble_sled_ends_in_bkpt() {
        let sled = assemble_sled(100);
        assert!(sled.len() >= 2, "sled too short: {}", sled.len());
        let n = sled.len();
        assert_eq!(sled[n - 2], 0x00, "expected BKPT low byte 0x00");
        assert_eq!(sled[n - 1], 0xBE, "expected BKPT high byte 0xBE");
    }

    #[test]
    fn assemble_sled_shape() {
        // movs r0, #N / subs r0, #1 / bne -6 / bkpt #0 = 8 bytes.
        let sled = assemble_sled(40);
        assert_eq!(sled.len(), 8);
        // Reconstruct halfwords for shape check.
        let hw0 = u16::from_le_bytes([sled[0], sled[1]]);
        let hw1 = u16::from_le_bytes([sled[2], sled[3]]);
        let hw2 = u16::from_le_bytes([sled[4], sled[5]]);
        let hw3 = u16::from_le_bytes([sled[6], sled[7]]);
        assert_eq!(hw0 & 0xFF00, 0x2000, "expected movs r0, #imm8");
        assert_eq!(hw1, 0x3801, "expected subs r0, #1");
        // bne T1: 0xD100 | (imm8 & 0xFF). imm8 = -3 → 0xFD. Branch at byte
        // 4 has PC = 8, target = PC + imm8*2 = 2 (the subs at hw[1]). The
        // earlier `0xD1FE` was imm8 = -2 and looped at the bne itself.
        assert_eq!(hw2, 0xD1FD, "expected bne back to subs (imm8 = -3)");
        assert_eq!(hw3, 0xBE00, "expected bkpt #0");
    }

    #[test]
    fn assemble_sled_clamps_large_counts() {
        // 1 << 20 cycles would otherwise overflow the 8-bit immediate.
        // Should clamp to 0xFF (255).
        let sled = assemble_sled(1 << 20);
        let hw0 = u16::from_le_bytes([sled[0], sled[1]]);
        assert_eq!(hw0 & 0x00FF, 0xFF);
    }

    /// Dry-run the assembled sled through `mdrp2040::Emulator` and assert
    /// the PC converges to the terminating BKPT halfword within a cycle
    /// budget — proof the bne loop is bounded rather than branching to
    /// itself. Guards against a regression of the `0xD1FE` bug that would
    /// cause silicon to hang in the bne forever.
    #[test]
    fn assemble_sled_converges_to_bkpt() {
        let sled = assemble_sled(40); // N = ceil(40/4) = 10 iterations
        let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build().expect("Serial build is infallible");
        emu.core_mut(1).halt();
        emu.load_image(SILICON_RUN_SLED, &sled);
        {
            let c = emu.core_mut(0);
            c.wake();
            c.set_reg(13, EMU_TEST_STACK);
            c.set_reg(14, 0xFFFF_FFFF);
            c.regs.set_pc(SILICON_RUN_SLED);
            c.regs.xpsr = 0x0100_0000; // T=1 (Thumb)
        }
        let bkpt_pc = SILICON_RUN_SLED + (sled.len() as u32) - 2;
        // 10 iterations * ~4 cycles/iter + framing ≈ 50; budget 500 so a
        // bounded loop always halts, an infinite loop never does.
        let budget: u64 = 500;
        let start = emu.cycles();
        while emu.core(0).regs.pc() != bkpt_pc && emu.cycles().saturating_sub(start) < budget
        {
            emu.step().expect("Serial step is infallible");
        }
        assert_eq!(
            emu.core(0).regs.pc(),
            bkpt_pc,
            "sled never reached BKPT — bne likely loops to itself again"
        );
    }
}

// silicon_periph_diff_rp2350 — peripheral-state oracle for mdrp2350 vs
// real RP2354 silicon.
//
// For each scenario in `silicon_scenarios::SCENARIOS` (filterable):
//   1. Halt core 0. First scenario resets the core; later ones just halt.
//   2. Release PIO0/PIO1/PLL_SYS from RESETS, then apply the scenario's
//      `setup` via probe-rs MMIO (HW) and `bus.write32` (EMU).
//   3. Upload a countdown sled sized for `max_sysclks` to
//      `SILICON_RUN_SLED`. Zero CYCCNT, set PC=sled, resume.
//   4. Poll for BKPT halt. First action: gate the peripheral (write
//      PIO.CTRL=0 or re-assert PLL_SYS RESETS bit) so readback is
//      atomic. Read CYCCNT → `actual_sysclks`.
//   5. Read observables (MMIO words + optional GPIO drive/level) from HW.
//   6. EMU side: fresh `Emulator` with both cores halted; apply setup;
//      `emu.run(actual_sysclks)`; gate; read the same observables.
//   7. Diff. First divergence wins.
//
// See `wrk_docs/2026.04.15 - HLD - Silicon Peripheral and Cycle Oracles.md`
// §Oracle 1.

use mdpicoem_harness::silicon_scenarios::{
    PeriphScenario, PIO0_BASE, PIO1_BASE, PIO_CTRL_OFF, PLL_SYS_BASE, RESETS_BASE,
    RESETS_RESET, RESET_PIO0, RESET_PIO1, RESET_PLL_SYS, SCENARIOS, SIO_GPIO_IN,
    SIO_GPIO_OE,
};
use mdpicoem_harness::{EMU_TEST_STACK, SILICON_RUN_SLED};
use mdrp2350::{Config, EmulatorBuilder};
use probe_rs::{Core, MemoryInterface, RegisterId, Session, SessionConfig};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// DWT / ARM core constants
// ---------------------------------------------------------------------------

const DEMCR: u64 = 0xE000_EDFC;
const DWT_CTRL: u64 = 0xE000_1000;
const DWT_CYCCNT: u64 = 0xE000_1004;
const TRCENA: u32 = 1 << 24;
const CYCCNTENA: u32 = 1 << 0;

const PC: RegisterId = RegisterId(15);
const XPSR: RegisterId = RegisterId(16);
const SP: RegisterId = RegisterId(13);
const LR: RegisterId = RegisterId(14);

/// Per-scenario BKPT timeout. Largest scenario (PLL) is ~1500 sysclks,
/// microseconds at any reasonable sys_clk; 5 s is absurd headroom.
const BKPT_TIMEOUT: Duration = Duration::from_secs(5);

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Args {
    filter: Option<String>,
    verbose: bool,
}

const USAGE: &str = "\
Usage: silicon_periph_diff_rp2350 [--filter <substr>] [--verbose]

Options:
  --filter   Only run scenarios whose name contains <substr>
  --verbose  Print per-observable diffs, not just the first divergence
";

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut filter: Option<String> = None;
    let mut verbose = false;
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--filter" => {
                i += 1;
                if i >= argv.len() {
                    return Err(format!("--filter requires a substring\n{USAGE}"));
                }
                filter = Some(argv[i].clone());
            }
            "--verbose" => verbose = true,
            "--help" | "-h" => return Err(USAGE.to_string()),
            other => return Err(format!("unknown argument '{other}'\n{USAGE}")),
        }
        i += 1;
    }
    Ok(Args { filter, verbose })
}

// ---------------------------------------------------------------------------
// Sled assembly
// ---------------------------------------------------------------------------

/// Build the countdown sled bytes for `max_sysclks`.
///
///   movw r0, #N      ; N = ceil(max_sysclks / 4), capped at 0xFFFF
///   subs r0, #1
///   bne  -4          ; back to subs
///   bkpt #0
///
/// Per-iter cost ≈ 4 sysclks (subs + bne-taken). MOVW T3 covers any
/// 16-bit immediate; MOVS T1 would cap at 255.
fn assemble_sled(max_sysclks: u32) -> Vec<u8> {
    let mut n = max_sysclks.div_ceil(4);
    if n == 0 {
        n = 1;
    }
    if n > 0xFFFF {
        n = 0xFFFF;
    }

    // MOVW r0, #N  (T3): hw0 = 0xF240 | (i<<10) | imm4,
    //                    hw1 = (imm3<<12) | (Rd<<8) | imm8; Rd=0.
    let i_bit = (n >> 11) & 1;
    let imm4 = (n >> 12) & 0xF;
    let imm3 = (n >> 8) & 0x7;
    let imm8 = n & 0xFF;
    let hw0 = (0xF240u32 | (i_bit << 10) | imm4) as u16;
    let hw1 = ((imm3 << 12) | imm8) as u16;

    let halfwords = [hw0, hw1, 0x3801u16, 0xD1FDu16, 0xBE00u16];
    let mut out = Vec::with_capacity(halfwords.len() * 2);
    for hw in halfwords {
        out.extend_from_slice(&hw.to_le_bytes());
    }
    out
}

// ---------------------------------------------------------------------------
// Hardware glue
// ---------------------------------------------------------------------------

fn enable_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    let demcr: u32 = core.read_word_32(DEMCR)?;
    core.write_word_32(DEMCR, demcr | TRCENA)?;
    let ctrl: u32 = core.read_word_32(DWT_CTRL)?;
    core.write_word_32(DWT_CTRL, ctrl | CYCCNTENA)?;
    Ok(())
}

fn reset_cyccnt(core: &mut Core) -> Result<(), probe_rs::Error> {
    core.write_word_32(DWT_CYCCNT, 0)
}

fn read_cyccnt(core: &mut Core) -> Result<u32, probe_rs::Error> {
    core.read_word_32(DWT_CYCCNT)
}

/// Release PIO0 / PIO1 / PLL_SYS from reset. Individual scenarios may
/// re-assert specific bits afterwards.
fn release_common_resets(core: &mut Core) -> Result<(), probe_rs::Error> {
    let state: u32 = core.read_word_32(RESETS_RESET as u64)?;
    let cleared = state & !(RESET_PIO0 | RESET_PIO1 | RESET_PLL_SYS);
    core.write_word_32(RESETS_RESET as u64, cleared)?;
    Ok(())
}

fn apply_setup_hw(core: &mut Core, setup: &[(u32, u32)]) -> Result<(), probe_rs::Error> {
    for &(addr, val) in setup {
        core.write_word_32(addr as u64, val)?;
    }
    Ok(())
}

/// Gate the peripheral off immediately after BKPT so readback is
/// atomic. Scenario-specific, driven by name prefix.
fn gate_peripheral_hw(core: &mut Core, name: &str) -> Result<(), probe_rs::Error> {
    if name.starts_with("pio0") {
        core.write_word_32((PIO0_BASE + PIO_CTRL_OFF) as u64, 0)?;
    } else if name.starts_with("pio1") {
        core.write_word_32((PIO1_BASE + PIO_CTRL_OFF) as u64, 0)?;
    } else if name.starts_with("pll_sys") {
        // PLL_SYS has no CS.ENABLE; re-assert RESETS bit to freeze.
        let state: u32 = core.read_word_32(RESETS_RESET as u64)?;
        core.write_word_32(RESETS_RESET as u64, state | RESET_PLL_SYS)?;
    }
    Ok(())
}

fn gate_peripheral_emu(emu: &mut mdrp2350::Emulator, name: &str) {
    if name.starts_with("pio0") {
        emu.mmio_write32(PIO0_BASE + PIO_CTRL_OFF, 0);
    } else if name.starts_with("pio1") {
        emu.mmio_write32(PIO1_BASE + PIO_CTRL_OFF, 0);
    } else if name.starts_with("pll_sys") {
        let state = emu.mmio_read32(RESETS_RESET);
        emu.mmio_write32(RESETS_RESET, state | RESET_PLL_SYS);
    }
}

/// Resume the core, poll until BKPT halts it or the timeout fires.
/// Returns the CYCCNT delta. On timeout, a one-line PC/SP/LR dump.
fn run_sled_hw(core: &mut Core) -> Result<u32, Box<dyn std::error::Error>> {
    reset_cyccnt(core)?;
    core.write_core_reg(PC, SILICON_RUN_SLED)?;
    core.write_core_reg(XPSR, 0x0100_0000u32)?; // T=1
    core.write_core_reg(SP, EMU_TEST_STACK)?;
    core.write_core_reg(LR, 0xFFFF_FFFFu32)?;
    core.run()?;

    let deadline = Instant::now() + BKPT_TIMEOUT;
    loop {
        if core.status()?.is_halted() {
            break;
        }
        if Instant::now() > deadline {
            let _ = core.halt(Duration::from_millis(200));
            let pc: u32 = core.read_core_reg(PC).unwrap_or(0xDEAD_BEEF);
            let sp: u32 = core.read_core_reg(SP).unwrap_or(0xDEAD_BEEF);
            let lr: u32 = core.read_core_reg(LR).unwrap_or(0xDEAD_BEEF);
            return Err(format!(
                "BKPT timeout: PC=0x{pc:08X} SP=0x{sp:08X} LR=0x{lr:08X}"
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    read_cyccnt(core).map_err(Into::into)
}

fn sample_pins_hw(core: &mut Core, mask: u32) -> Result<(u32, u32), probe_rs::Error> {
    let oe: u32 = core.read_word_32(SIO_GPIO_OE as u64)?;
    let in_: u32 = core.read_word_32(SIO_GPIO_IN as u64)?;
    Ok((oe & mask, in_ & mask))
}

fn sample_pins_emu(emu: &mut mdrp2350::Emulator, mask: u32) -> (u32, u32) {
    let oe = emu.mmio_read32(SIO_GPIO_OE) & mask;
    let in_ = emu.mmio_read32(SIO_GPIO_IN) & mask;
    (oe, in_)
}

// ---------------------------------------------------------------------------
// Per-scenario driver
// ---------------------------------------------------------------------------

#[derive(Debug, PartialEq)]
enum Verdict {
    Pass,
    Fail,
}

struct ScenarioResult {
    name: &'static str,
    verdict: Verdict,
    actual_sysclks: u32,
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

    // Fresh core state.
    if first_scenario {
        core.reset_and_halt(Duration::from_millis(500))?;
        enable_cyccnt(core)?;
    } else if !core.status()?.is_halted() {
        core.halt(Duration::from_millis(200))?;
    }
    release_common_resets(core)?;

    // Hardware: setup → upload sled → run → gate → observe.
    apply_setup_hw(core, sc.setup)?;
    let sled = assemble_sled(sc.max_sysclks);
    core.write_8(SILICON_RUN_SLED as u64, &sled)?;
    let actual_sysclks = run_sled_hw(core)?;
    gate_peripheral_hw(core, sc.name)?;

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

    // Emulator: fresh build with cores halted → setup → run → gate → observe.
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    emu.core_mut(0).halt();
    emu.core_mut(1).halt();
    for &(addr, val) in sc.setup {
        emu.mmio_write32(addr, val);
    }
    emu.run(actual_sysclks as u64);
    gate_peripheral_emu(&mut emu, sc.name);

    let emu_obs: Vec<u32> =
        sc.observe.iter().map(|(addr, _m)| emu.mmio_read32(*addr)).collect();
    let emu_pins = if sc.observe_pins != 0 {
        Some(sample_pins_emu(&mut emu, sc.observe_pins))
    } else {
        None
    };

    // Diff.
    let mut first_div: Option<String> = None;
    for (i, (addr, mask)) in sc.observe.iter().enumerate() {
        let h = hw_obs[i] & *mask;
        let e = emu_obs[i] & *mask;
        if h != e {
            let msg = format!(
                "MMIO 0x{:08X} mask=0x{:08X}: HW=0x{:08X} EMU=0x{:08X} (xor=0x{:08X})",
                addr, mask, h, e, h ^ e,
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
        actual_sysclks,
        first_divergence: first_div,
        elapsed: t0.elapsed(),
    })
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
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
            "silicon_periph_diff_rp2350: no scenarios match filter '{}'; nothing to do",
            args.filter.as_deref().unwrap_or(""),
        );
        return Ok(0);
    }

    println!(
        "silicon_periph_diff_rp2350: {} scenario(s) selected ({} skipped by filter)",
        selected.len(),
        skipped,
    );
    println!("sled=0x{SILICON_RUN_SLED:08X} resets=0x{RESETS_BASE:08X} pll_sys=0x{PLL_SYS_BASE:08X}");
    println!();

    let mut session = Session::auto_attach("rp2350", SessionConfig::default())?;
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
        let r = run_scenario(&mut core, sc, i == 0, args.verbose)?;
        match r.verdict {
            Verdict::Pass => pass += 1,
            Verdict::Fail => fail += 1,
        }
        println!(
            "{:<28} {:>6} {:>10.1} {:>7}  {}",
            r.name,
            r.actual_sysclks,
            r.elapsed.as_secs_f64() * 1000.0,
            if r.verdict == Verdict::Pass { "PASS" } else { "FAIL" },
            r.first_divergence.as_deref().unwrap_or("-"),
        );
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

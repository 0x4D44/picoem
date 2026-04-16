// Bank-conflict catalogue — halt-step single-step measurement for mdrp2350
// vs real RP2354 silicon.
//
// Each case writes one Thumb instruction at a fixed SRAM slot in the same
// bank as the sequence fetch (bank 0), primes R1 at a data address whose
// bank is chosen by the case, then measures:
//
//   HW  — `core.step()` (SWD halt-step), read DWT CYCCNT, subtract the
//         per-run NOP baseline (the 5-ish-cycle debug-stop overhead).
//   EMU — `emu.cores[0].step(&mut emu.bus)`, read the core's own
//         `cycles()` counter delta.
//
// Diff = `(HW_observed - debug_overhead) - emu_cycles`, pass iff
// `|diff| <= tolerance`.
//
// **Why halt-step, not K-delta.** The K-delta sequence-in-loop protocol
// (`cycle_cases`) averages framing cost over many iterations, which masks
// the +1 cycle SRAM bank-contention signal this oracle exists to catch —
// see `tech_debt.md:545-554` for the empirical demonstration and the HLD
// at `wrk_docs/2026.04.15 - HLD - test_silicon Orchestrator and Coverage
// Expansion.md` §Library-API extraction for the reversal.
//
// The runner mirrors the shape of the pre-orchestrator
// `bank_conflict_test_rp2350.rs` binary (committed as `42d9f2a`), with the
// per-case measurement loop hoisted out into a library entry point the
// orchestrator shares.

use crate::silicon_oracle::{
    self, enable_cyccnt, read_cyccnt, reset_cyccnt, CaseOutcome, Verdict,
};
use mdrp2350::{Config, Emulator, EmulatorBuilder};
use probe_rs::{Core, MemoryInterface, RegisterId};
use std::time::Instant;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// SRAM slot for the one-instruction test body — always bank 0 by choice.
/// `(0x100 >> 2) & 7 = 0`.
pub const TEST_SLOT: u32 = 0x2000_0100;

/// ARM Cortex-M register IDs for probe-rs.
const R0: RegisterId = RegisterId(0);
const R1: RegisterId = RegisterId(1);
const PC: RegisterId = RegisterId(15);

// Thumb-16 encodings (halfword, passed as u32 to `write_thumb_hw`).
const NOP: u32 = 0xBF00;
const LDR_R0_R1: u32 = 0x6808; // LDR R0, [R1, #0]
const STR_R0_R1: u32 = 0x6008; // STR R0, [R1, #0]

// ---------------------------------------------------------------------------
// Bank addressing
// ---------------------------------------------------------------------------
//
// SRAM bank formula: `bank = (byte_address >> 2) & 7` (bits [4:2]). The
// eight 4-byte slots from 0x2000_0200 to 0x2000_021C cover banks 0..7.

/// Compute the SRAM bank index for a byte address (bits [4:2]).
pub const fn bank_of(addr: u32) -> u32 {
    (addr >> 2) & 7
}

/// Data-slot address for bank `b` in [0..8). All slots live in the
/// `EMU_TEST_SCRATCH` region.
pub const fn data_addr_for_bank(b: u32) -> u32 {
    0x2000_0200 + b * 4
}

// ---------------------------------------------------------------------------
// Catalog
// ---------------------------------------------------------------------------

/// A single halt-step bank-conflict case.
///
/// `instr` is written to `fetch_addr` before measurement; `data_addr` is
/// loaded into R1 so the memory access lands in the intended bank.
/// `fetch_addr` is where the instruction is fetched from — defaults to
/// `TEST_SLOT` (bank 0); the fetch-bank sweep cases move this to
/// 0x2000_0100 .. 0x2000_011C so every bank takes a turn as the fetch
/// port.
///
/// `emu_baseline` is the emulator's last recorded cycle value for this
/// case. The runner prints the live value on every invocation so drift
/// is visible — update this field when the emulator legitimately
/// changes (NOT to silence a failure; that's the signal).
#[derive(Clone, Debug)]
pub struct BankCase {
    pub name: &'static str,
    pub instr: u32,
    pub fetch_addr: u32,
    pub data_addr: u32,
    pub emu_baseline: u32,
}

impl BankCase {
    /// Fetch lives at `TEST_SLOT` (bank 0); data bank varies.
    fn data_bank(name: &'static str, instr: u32, bank: u32, emu_baseline: u32) -> Self {
        Self {
            name,
            instr,
            fetch_addr: TEST_SLOT,
            data_addr: data_addr_for_bank(bank),
            emu_baseline,
        }
    }
    /// Data lives at a fixed bank-0 address; fetch bank varies 0..7.
    fn fetch_bank(name: &'static str, fetch_bank: u32) -> Self {
        Self {
            name,
            instr: LDR_R0_R1,
            fetch_addr: 0x2000_0100 + fetch_bank * 4,
            // Fixed data address in bank 0 (bits [4:2] of 0x300 = 0).
            data_addr: 0x2000_0300,
            emu_baseline: 2, // M33 LDR base cycle cost
        }
    }
}

/// Build the halt-step catalogue. Returns an owned `Vec` so every
/// iteration of a soak run gets a fresh slice — the orchestrator's
/// Fisher-Yates shuffle mutates the selection in place.
///
/// Layout (mirroring the original `bank_conflict_test`):
///   - 8× LDR with data bank = 0..7 (fetch always bank 0).
///   - 8× STR with data bank = 0..7 (fetch always bank 0).
///   - 8× fetch-bank sweep (LDR, data fixed at bank 0, fetch bank 0..7).
///   - 4× near-neighbour controls (LDR at bank-2 data slots in different
///     256-byte regions — sanity check that the bank hazard depends on
///     bank bits alone, not the full address).
pub fn build_catalog() -> Vec<BankCase> {
    let mut out: Vec<BankCase> = Vec::with_capacity(28);
    // LDR sweep (data bank 0..7).
    for b in 0..8u32 {
        out.push(BankCase::data_bank(LDR_NAMES[b as usize], LDR_R0_R1, b, 2));
    }
    // STR sweep (data bank 0..7).
    for b in 0..8u32 {
        // STR is architecturally 2 cycles on M33 (one cycle + one store).
        out.push(BankCase::data_bank(STR_NAMES[b as usize], STR_R0_R1, b, 2));
    }
    // Fetch-bank sweep.
    for b in 0..8u32 {
        out.push(BankCase::fetch_bank(FETCH_NAMES[b as usize], b));
    }
    // Near-neighbour controls: LDR at bank-2 data slots in different
    // 256-byte regions. Expected identical to LDR_b2_diff if bank bits
    // alone decide contention.
    for (name, addr) in [
        ("bankcfl_near_b2_228", 0x2000_0228u32),
        ("bankcfl_near_b2_248", 0x2000_0248u32),
        ("bankcfl_near_b2_308", 0x2000_0308u32),
        ("bankcfl_near_b2_408", 0x2000_0408u32),
    ] {
        debug_assert_eq!(bank_of(addr), 2, "near-neighbour addr must be bank 2");
        out.push(BankCase {
            name,
            instr: LDR_R0_R1,
            fetch_addr: TEST_SLOT,
            data_addr: addr,
            emu_baseline: 2,
        });
    }
    out
}

// Static name tables — `BankCase.name: &'static str` so `CaseOutcome`
// concatenation in the orchestrator stays allocation-free.
//
// All bank-conflict case names use the `bankcfl_` prefix so they can
// never be a strict substring of any cycle-oracle sequence name that
// happens to contain the word "bank" (e.g. `bank_contention_*`). The
// orchestrator's substring-uniqueness validator asserts this at startup;
// keep the prefix consistent here so the validator does not fire.
const LDR_NAMES: [&str; 8] = [
    "bankcfl_ldr_b0_same",
    "bankcfl_ldr_b1_diff",
    "bankcfl_ldr_b2_diff",
    "bankcfl_ldr_b3_diff",
    "bankcfl_ldr_b4_diff",
    "bankcfl_ldr_b5_diff",
    "bankcfl_ldr_b6_diff",
    "bankcfl_ldr_b7_diff",
];
const STR_NAMES: [&str; 8] = [
    "bankcfl_str_b0_same",
    "bankcfl_str_b1_diff",
    "bankcfl_str_b2_diff",
    "bankcfl_str_b3_diff",
    "bankcfl_str_b4_diff",
    "bankcfl_str_b5_diff",
    "bankcfl_str_b6_diff",
    "bankcfl_str_b7_diff",
];
const FETCH_NAMES: [&str; 8] = [
    "bankcfl_fetch_b0_data_b0",
    "bankcfl_fetch_b1_data_b0",
    "bankcfl_fetch_b2_data_b0",
    "bankcfl_fetch_b3_data_b0",
    "bankcfl_fetch_b4_data_b0",
    "bankcfl_fetch_b5_data_b0",
    "bankcfl_fetch_b6_data_b0",
    "bankcfl_fetch_b7_data_b0",
];

// ---------------------------------------------------------------------------
// Args + per-case result
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct BankArgs {
    pub filter: Option<String>,
    /// How many raw HW samples to take per case; returned as the median.
    /// The original binary used 20.
    pub num_samples: usize,
    /// Pass tolerance on `|(hw - overhead) - emu|`. Default 1 per HLD
    /// v1.1.1 (absorbs debug-overhead variation across instruction / bus
    /// paths). Set to 0 for strict same-as-emulator cases.
    pub tolerance: u32,
}

impl Default for BankArgs {
    fn default() -> Self {
        Self {
            filter: None,
            num_samples: 20,
            tolerance: 1,
        }
    }
}

/// One case's rich per-run result (kept so the standalone binary can
/// reproduce its per-bank cycle table).
#[derive(Clone, Debug)]
pub struct BankCaseResult {
    pub name: &'static str,
    pub fetch_addr: u32,
    pub fetch_bank: u32,
    pub data_addr: u32,
    pub data_bank: u32,
    pub hw_median: u32,
    pub hw_samples: Vec<u32>,
    /// `hw_median - (nop_baseline - 1)` — the cycle count attributable
    /// to the test instruction after debug-overhead cancellation.
    pub hw_adjusted: u32,
    pub nop_baseline: u32,
    pub emu_cycles: u32,
    pub emu_baseline: u32,
    pub delta: i64,
    pub verdict: Verdict,
    pub elapsed_ms: u32,
}

// ---------------------------------------------------------------------------
// HW-side halt-step measurement
// ---------------------------------------------------------------------------

fn write_thumb_hw(core: &mut Core, addr: u32, hw: u32) -> Result<(), probe_rs::Error> {
    let bytes = (hw as u16).to_le_bytes();
    core.write_8(addr as u64, &bytes)?;
    Ok(())
}

/// Measure one HW sample: write the instruction, set registers, step,
/// read CYCCNT.
fn measure_hw_one(
    core: &mut Core,
    instr: u32,
    fetch_addr: u32,
    data_addr: u32,
) -> Result<u32, probe_rs::Error> {
    write_thumb_hw(core, fetch_addr, instr)?;
    // Point R1 at the data address and write a deterministic pattern —
    // STR needs R0 set too.
    core.write_core_reg(R1, data_addr)?;
    core.write_word_32(data_addr as u64, 0xCAFE_BABE)?;
    core.write_core_reg(R0, 0x1234_5678u32)?;
    core.write_core_reg(PC, fetch_addr)?;
    reset_cyccnt(core)?;
    core.step()?;
    read_cyccnt(core)
}

fn collect_hw_samples(
    core: &mut Core,
    instr: u32,
    fetch_addr: u32,
    data_addr: u32,
    n: usize,
) -> Result<Vec<u32>, probe_rs::Error> {
    let mut samples = Vec::with_capacity(n);
    for _ in 0..n {
        samples.push(measure_hw_one(core, instr, fetch_addr, data_addr)?);
    }
    Ok(samples)
}

/// Median of a sample vector (n/2 after sort — cheap for n=20).
pub fn median(samples: &[u32]) -> u32 {
    let mut s = samples.to_vec();
    s.sort_unstable();
    s[s.len() / 2]
}

/// Calibrate the per-run debug-stop overhead: halt-step one NOP at
/// `TEST_SLOT` and take the median of `num_samples`. The original
/// `bank_conflict_test` used this exact trick — the returned value
/// includes fetch + NOP (1 cycle) + debug stop. Subtracting it from a
/// bank measurement isolates the bank-contention delta because both
/// measurements share the same debug overhead.
pub fn measure_nop_baseline_hw(
    core: &mut Core,
    num_samples: usize,
) -> Result<u32, probe_rs::Error> {
    let samples = collect_hw_samples(core, NOP, TEST_SLOT, 0x2000_0200, num_samples)?;
    Ok(median(&samples))
}

/// Compute the diff given the HW median, HW NOP baseline, and the EMU
/// per-instruction cycle count. Pulled out into a free function so unit
/// tests can hammer the math without an emulator or a probe.
pub fn compute_diff(hw_median: u32, nop_baseline: u32, emu_cycles: u32) -> (u32, i64) {
    // The emulator counts a NOP as 1 cycle with no debug overhead, so the
    // HW measurement — one NOP plus the debug stop — gives us the
    // overhead minus one emulated cycle. The fair comparison is
    // `hw_median - (nop_baseline - emu_nop_cycles)`. For M33 NOP the
    // emulated cost is 1, so `overhead_adjustment = nop_baseline - 1`.
    //
    // Saturating arithmetic everywhere: an absurdly low baseline
    // (can't happen on real silicon, but wrap-around in a signed path
    // would make the delta sign confusing) just clips to 0.
    let overhead = nop_baseline.saturating_sub(1);
    let hw_adjusted = hw_median.saturating_sub(overhead);
    let delta = hw_adjusted as i64 - emu_cycles as i64;
    (hw_adjusted, delta)
}

// ---------------------------------------------------------------------------
// EMU-side halt-step measurement
// ---------------------------------------------------------------------------

/// Build a fresh emulator with `step_quantum=1`, core 1 halted, DWT on,
/// and the single test instruction primed at `fetch_addr`.
fn fresh_emulator(instr: u32, fetch_addr: u32, data_addr: u32) -> Emulator {
    let mut emu = EmulatorBuilder::new(Config::default()).step_quantum(1).build();
    emu.cores[1].halt();

    // Write the instruction (two bytes) at `fetch_addr`. The decoded-op
    // cache is empty at this point so no explicit invalidation is
    // required.
    let fetch_off = fetch_addr - 0x2000_0000;
    emu.bus.memory.sram_write8(fetch_off, (instr & 0xFF) as u8);
    emu.bus.memory.sram_write8(fetch_off + 1, ((instr >> 8) & 0xFF) as u8);

    // Seed the data slot so LDR yields a deterministic value and STR has
    // somewhere to land.
    emu.bus.write32(data_addr, 0xCAFE_BABE, 0);

    // Enable DWT on the emulator side too, for parity even though we do
    // not sample CYCCNT here — the emulator's cycle accounting lives on
    // `core.cycles()`.
    let demcr = emu.bus.read32(silicon_oracle::DEMCR_U32, 0);
    emu.bus.write32(silicon_oracle::DEMCR_U32, demcr | silicon_oracle::TRCENA, 0);
    let ctrl = emu.bus.read32(silicon_oracle::DWT_CTRL_U32, 0);
    emu.bus
        .write32(silicon_oracle::DWT_CTRL_U32, ctrl | silicon_oracle::CYCCNTENA, 0);

    // Prime core 0 registers to execute exactly one instruction.
    emu.cores[0].wake();
    emu.cores[0].regs.set_pc(fetch_addr);
    emu.cores[0].regs.r[0] = 0x1234_5678;
    emu.cores[0].regs.r[1] = data_addr;
    emu.cores[0].regs.r[13] = crate::EMU_TEST_STACK;
    emu.cores[0].regs.msp = crate::EMU_TEST_STACK;
    emu.cores[0].regs.r[14] = 0xFFFF_FFFF;
    emu.cores[0].regs.xpsr = 0x0100_0000; // T=1

    emu
}

/// Run one instruction on core 0 and return the cycles it reported.
fn measure_emu_one(instr: u32, fetch_addr: u32, data_addr: u32) -> u32 {
    let mut emu = fresh_emulator(instr, fetch_addr, data_addr);
    // Phase 0b.1 Commit B: no `set_active_core` needed — CortexM33::step
    // takes a `&mut Bus` and self-supplies its core id for bus routing.
    let before = emu.cores[0].cycles();
    // Step exactly one instruction on core 0. We bypass `Emulator::step`
    // (which advances both cores and peripherals) because halt-step cares
    // only about the single-instruction cycle accounting on core 0 —
    // peripheral ticks and clock housekeeping would just add noise.
    emu.cores[0].step(&mut emu.bus);
    let after = emu.cores[0].cycles();
    (after - before) as u32
}

// ---------------------------------------------------------------------------
// Per-case runner
// ---------------------------------------------------------------------------

/// Run one case and produce a full diagnostic record.
///
/// `nop_baseline` is passed in rather than remeasured per case — the
/// original `bank_conflict_test` measured it once at the top of the run,
/// and the tolerance semantics depend on a single per-session overhead
/// constant.
pub fn run_bank_case(
    core: &mut Core,
    case: &BankCase,
    num_samples: usize,
    tolerance: u32,
    nop_baseline: u32,
) -> Result<BankCaseResult, Box<dyn std::error::Error>> {
    let t0 = Instant::now();

    let hw_samples = collect_hw_samples(
        core,
        case.instr,
        case.fetch_addr,
        case.data_addr,
        num_samples,
    )?;
    let hw_median = median(&hw_samples);
    let emu_cycles = measure_emu_one(case.instr, case.fetch_addr, case.data_addr);
    let (hw_adjusted, delta) = compute_diff(hw_median, nop_baseline, emu_cycles);
    let verdict = if (delta.unsigned_abs() as u32) <= tolerance {
        Verdict::Pass
    } else {
        Verdict::Fail
    };
    let elapsed_ms = t0.elapsed().as_millis().min(u32::MAX as u128) as u32;
    Ok(BankCaseResult {
        name: case.name,
        fetch_addr: case.fetch_addr,
        fetch_bank: bank_of(case.fetch_addr),
        data_addr: case.data_addr,
        data_bank: bank_of(case.data_addr),
        hw_median,
        hw_samples,
        hw_adjusted,
        nop_baseline,
        emu_cycles,
        emu_baseline: case.emu_baseline,
        delta,
        verdict,
        elapsed_ms,
    })
}

// ---------------------------------------------------------------------------
// Library entry point
// ---------------------------------------------------------------------------

/// Library entry point used by `bank_conflict_test_rp2350` and the
/// `test_silicon` orchestrator.
///
/// **Cleanup contract**: none. This oracle only writes SRAM scratch
/// (one instruction at `TEST_SLOT`, one data word per case) and enables
/// DWT — both defaulted by `core.reset_and_halt`, so the next oracle in
/// an orchestrated run sees a clean slate without any explicit teardown.
///
/// Preconditions: `core` is halted. DWT enable is idempotent and done
/// here, matching the shape of `cycle_cases::run_against`.
///
/// Case selection semantics:
/// * `order = None` — run every catalogue case whose name matches
///   `args.filter`, in catalogue-declared order (single-pass / standalone
///   default).
/// * `order = Some(&[name, name, …])` — run exactly those cases in that
///   order. `args.filter` is ignored for selection. Names not present in
///   the catalogue are skipped with a single `eprintln!` warning per
///   unknown name.
///
/// The NOP baseline is calibrated once per call (20 halt-step samples
/// ≈ 1 second). Passing a single-element `order` list therefore means
/// paying the baselining cost per call — the orchestrator amortises this
/// by passing the full shuffled order list in one call per iteration.
pub fn run_against(
    core: &mut Core,
    args: &BankArgs,
    order: Option<&[&str]>,
) -> Result<Vec<CaseOutcome>, Box<dyn std::error::Error>> {
    debug_assert!(args.num_samples >= 1, "num_samples must be >= 1");

    enable_cyccnt(core)?;
    let nop_baseline = measure_nop_baseline_hw(core, args.num_samples)?;

    let catalog = build_catalog();
    let selected: Vec<BankCase> = match order {
        None => catalog
            .into_iter()
            .filter(|c| silicon_oracle::name_matches_filter(c.name, args.filter.as_deref()))
            .collect(),
        Some(names) => {
            let mut v: Vec<BankCase> = Vec::with_capacity(names.len());
            for name in names {
                match catalog.iter().find(|c| c.name == *name) {
                    Some(c) => v.push(c.clone()),
                    None => eprintln!(
                        "bank_conflict_cases::run_against: unknown case '{name}' in order list; skipping",
                    ),
                }
            }
            v
        }
    };

    let mut outcomes: Vec<CaseOutcome> = Vec::with_capacity(selected.len());
    for case in &selected {
        let r = run_bank_case(core, case, args.num_samples, args.tolerance, nop_baseline)?;
        let detail = if r.verdict == Verdict::Pass {
            String::new()
        } else {
            format!(
                "hw_med={} nop_base={} hw_adj={} emu={} delta={:+} tol={} fetch_bank={} data_bank={}",
                r.hw_median,
                r.nop_baseline,
                r.hw_adjusted,
                r.emu_cycles,
                r.delta,
                args.tolerance,
                r.fetch_bank,
                r.data_bank,
            )
        };
        outcomes.push(CaseOutcome {
            oracle: "bank",
            case: r.name,
            verdict: r.verdict,
            detail,
            elapsed_ms: r.elapsed_ms,
        });
    }
    Ok(outcomes)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bank_of_formula() {
        assert_eq!(bank_of(0x2000_0200), 0);
        assert_eq!(bank_of(0x2000_0204), 1);
        assert_eq!(bank_of(0x2000_0208), 2);
        assert_eq!(bank_of(0x2000_021C), 7);
        // TEST_SLOT lives in bank 0 — every bank-0 case relies on this
        // precondition for the "same-bank" hazard to fire.
        assert_eq!(bank_of(TEST_SLOT), 0);
    }

    #[test]
    fn test_data_addr_for_bank_roundtrips() {
        for b in 0..8u32 {
            let addr = data_addr_for_bank(b);
            assert_eq!(bank_of(addr), b, "bank {b}: addr=0x{addr:08X}");
        }
    }

    #[test]
    fn test_build_catalog_shape() {
        let cat = build_catalog();
        // 8 LDR + 8 STR + 8 fetch-bank + 4 near-neighbour = 28.
        assert_eq!(cat.len(), 28, "catalogue size mismatch");
        assert!(cat.iter().any(|c| c.name == "bankcfl_ldr_b0_same"));
        assert!(cat.iter().any(|c| c.name == "bankcfl_str_b7_diff"));
        assert!(cat.iter().any(|c| c.name == "bankcfl_fetch_b5_data_b0"));
        assert!(cat.iter().any(|c| c.name == "bankcfl_near_b2_248"));
        // All names unique — orchestrator filter + report depend on it.
        let mut seen: std::collections::HashSet<&str> = Default::default();
        for c in &cat {
            assert!(seen.insert(c.name), "duplicate '{}'", c.name);
        }
    }

    #[test]
    fn test_fetch_bank_addresses_cover_all_banks() {
        let cat = build_catalog();
        let fetch_cases: Vec<&BankCase> = cat
            .iter()
            .filter(|c| c.name.starts_with("bankcfl_fetch_"))
            .collect();
        assert_eq!(fetch_cases.len(), 8);
        let mut banks_seen: std::collections::HashSet<u32> = Default::default();
        for c in &fetch_cases {
            banks_seen.insert(bank_of(c.fetch_addr));
        }
        assert_eq!(banks_seen.len(), 8, "fetch-bank sweep must cover 0..8");
    }

    /// Core of the diff math: HW median minus NOP baseline (adjusted so
    /// one emulated NOP cycle is not double-subtracted) vs emulator
    /// cycles. This test locks the arithmetic so a refactor that moves
    /// the `-1` out of `compute_diff` gets caught.
    #[test]
    fn test_compute_diff_nop_baseline_subtraction() {
        // HW measured 8 cycles for a NOP (baseline = 8).
        // Case measured 9 cycles for an LDR.
        // Emulator says LDR costs 2 cycles (M33 base).
        // overhead = 8 - 1 (emulator NOP cost) = 7.
        // hw_adjusted = 9 - 7 = 2.
        // delta = 2 - 2 = 0.
        let (adj, delta) = compute_diff(9, 8, 2);
        assert_eq!(adj, 2);
        assert_eq!(delta, 0);

        // Same-bank case: HW=10 (+1 for contention), EMU=2 (no contention).
        // overhead = 7.
        // hw_adjusted = 10 - 7 = 3.
        // delta = 3 - 2 = +1 — the bank-contention signal.
        let (adj, delta) = compute_diff(10, 8, 2);
        assert_eq!(adj, 3);
        assert_eq!(delta, 1);

        // EMU-heavy case: the emulator over-counts relative to silicon
        // by 1. Expect delta = -1.
        let (adj, delta) = compute_diff(9, 8, 3);
        assert_eq!(adj, 2);
        assert_eq!(delta, -1);

        // Saturating behaviour: if baseline is absurdly small, we must
        // not underflow. (Can't happen on real silicon but the math
        // promise is load-bearing; easier to test here than chase down
        // an overflow panic at 3am.)
        let (adj, delta) = compute_diff(5, 0, 2);
        assert_eq!(adj, 5);
        assert_eq!(delta, 3);
    }

    /// Verdict threshold around the +1 contention signal.
    #[test]
    fn test_verdict_tolerance_at_one_absorbs_plusone_signal() {
        // hw - overhead = 3, emu = 2, delta = +1. At tol=1 this PASSes;
        // at tol=0 it FAILs. Exactly the policy HLD v1.1.1 specifies.
        let (_, delta) = compute_diff(10, 8, 2);
        let tol_one_pass = (delta.unsigned_abs() as u32) <= 1;
        let tol_zero_pass = (delta.unsigned_abs() as u32) == 0;
        assert!(tol_one_pass, "delta={delta} must PASS at tol=1");
        assert!(!tol_zero_pass, "delta={delta} must FAIL at tol=0");
    }

    #[test]
    fn test_median_picks_middle_of_odd_and_even() {
        // n=5 → index 2.
        assert_eq!(median(&[1, 3, 5, 7, 9]), 5);
        // n=20 → index 10. (20 is what the binary actually uses.)
        let mut v: Vec<u32> = (1..=20).collect();
        v.reverse();
        assert_eq!(median(&v), 11);
    }

    #[test]
    fn test_filter_substring_semantics() {
        let cat = build_catalog();
        let ldr: Vec<&BankCase> = cat
            .iter()
            .filter(|c| silicon_oracle::name_matches_filter(c.name, Some("bankcfl_ldr_")))
            .collect();
        assert_eq!(ldr.len(), 8, "filter 'bankcfl_ldr_' must match all 8 LDR cases");
        let b0: Vec<&BankCase> = cat
            .iter()
            .filter(|c| silicon_oracle::name_matches_filter(c.name, Some("_b0_")))
            .collect();
        // `_b0_` bracketed by underscores matches:
        //   - bankcfl_ldr_b0_same
        //   - bankcfl_str_b0_same
        //   - bankcfl_fetch_b0_data_b0  (has "_b0_" in the middle as
        //     well as the trailing "_b0", but filter semantics are
        //     substring, one match per name is enough)
        // The other fetch-bank names end in "_b0" (not "_b0_") and do
        // NOT match; near-neighbour names contain "_b2_".
        assert_eq!(b0.len(), 3);
        let none: Vec<&BankCase> = cat
            .iter()
            .filter(|c| silicon_oracle::name_matches_filter(c.name, Some("nope")))
            .collect();
        assert!(none.is_empty());
    }
}

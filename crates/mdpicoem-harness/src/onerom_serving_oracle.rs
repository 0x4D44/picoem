//! OneROM serving oracle — byte-correctness + timing envelope (Stage G).
//!
//! Stage F validated that *some* stable byte appears on D0..D7 after sync;
//! it did not validate the byte's value, sweep multiple addresses, or
//! measure the CS-to-valid-data latency. This module is the Stage G
//! pipeline: snapshot SRAM at sync (the SRAM-base shadow, §3.1), drive
//! pin stimuli, and for each case prove the observed byte matches the
//! shadow byte that CH1.READ_ADDR actually resolved to.
//!
//! Stage G.1 wires the state machine and the trace-driven verdict logic
//! for a single baseline case (`0x1800`, A11=A12=1 with all low bits 0).
//! The full 15-case walking-1s + pattern sweep lands in G.2; the timing
//! report in G.3.
//!
//! Design: `wrk_docs/2026.04.15 - HLD - OneROM Serving Oracle (Stage G).md`.
//!
//! Key invariants enforced by this module (see HLD §4.3, §4.4):
//! - `addr_bits & 0x1800 == 0x1800` for every case — pin-map collision
//!   means CS2/CS3 share GPIOs with A12/A11; CS2=CS3=high requires
//!   A11=A12=1.
//! - Stability is anchored *after* the CH1 push cycle. A trace whose
//!   `data_byte` happens to hold the prior case's byte at cs_low must
//!   not report a fictional zero-latency PASS. See §4.4.
//! - `resolved_addr` must fall in `[SHADOW_BASE, SHADOW_BASE + SHADOW_SIZE)`
//!   before we look up `expected_byte`. Outside → `ResolvedAddrOutOfRange`.

use std::fmt::Write as _;

use mdrp2350::{Bus, Emulator};

use crate::onerom_glue_dma::{
    GlueDma, DMA_BASE, DMA_CH_READ_ADDR, DMA_CH_STRIDE,
};

// ---------------------------------------------------------------------------
// Public constants
// ---------------------------------------------------------------------------

/// Mask every case must set to keep CS2/CS3 deasserted during reads.
/// See HLD §4.1 + the CAUTION block in `onerom_full_system_rp2350.rs`.
pub const ADDR_A11_A12_HIGH: u16 = 0x1800;

/// Base of the SRAM shadow — SRAM region origin on RP2350.
pub const SHADOW_BASE: u32 = 0x2000_0000;

/// Shadow size: one 2364-class ROM.
pub const SHADOW_SIZE: usize = 0x2000;

/// Acceptable CS-low-to-stable-byte cycle envelope per `piorom.c`.
pub const ENVELOPE_CYCLES: std::ops::RangeInclusive<u32> = 11..=14;

// ---------------------------------------------------------------------------
// Internal constants
// ---------------------------------------------------------------------------

/// Minimum consecutive cycles of the same byte (with `pad_oe == 0xFF`)
/// required to declare a value "stable".
const MIN_STABLE_CYCLES: usize = 3;

/// Cycles of CS1-high + CS2/CS3-high + addr=0 we drive on the first
/// case to guarantee a clean high-to-low edge on CS1. Stage F ends
/// with CS1 low, so without this seed case 1 would never see a
/// transition. Applied once, in the `init` transition (HLD §4.3).
const SEED_CYCLES: u32 = 4;

/// CS-high settle between consecutive cases.
const GAP_CYCLES: u32 = 3;

/// Cycle budget per case (envelope is 11..=14; 60 gives ~4× slack).
const PER_CASE_TIMEOUT: u32 = 60;

/// Data bus base — D0..D7 on GPIO 16..23. Mirrors Stage F.
const GPIO_DATA_BASE: u8 = 16;

/// CS lanes on the `test-sdrr-0` fixture. Mirrors Stage F.
const GPIO_CS1: u8 = 13;
const GPIO_CS2: u8 = 12;
const GPIO_CS3: u8 = 15;

/// A0..A12 pin map. Mirrors Stage F.
const ADDR_PINS: [u8; 13] = [7, 6, 5, 4, 3, 2, 1, 0, 10, 11, 14, 15, 12];

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One address stimulus for the sweep.
///
/// `addr_bits` must have A11=A12=1 (`addr_bits & 0x1800 == 0x1800`) —
/// the `Case::new` constructor and the `DEFAULT_CASES` initializer
/// both assert this.
#[derive(Clone, Copy, Debug)]
pub struct Case {
    pub label: &'static str,
    pub addr_bits: u16,
}

impl Case {
    /// Build a case, asserting the A11=A12=1 invariant in debug builds.
    pub const fn new(label: &'static str, addr_bits: u16) -> Self {
        debug_assert!(
            addr_bits & ADDR_A11_A12_HIGH == ADDR_A11_A12_HIGH,
            "Case::new: addr_bits must have A11=A12=1 (addr_bits & 0x1800 == 0x1800)"
        );
        Self { label, addr_bits }
    }
}

/// Default address-case set.
///
/// G.1 ships only the baseline. G.2 will extend to the full
/// walking-1s over A0..A10 plus patterns from HLD §4.2.
// TODO(G.2): populate full walking-1s + pattern set (15 cases)
pub const DEFAULT_CASES: &[Case] = &[
    Case::new("walk1 A0 (baseline)", 0x1800),
];

/// Outcome for one case.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Byte observed on D0..D7 matched the SRAM shadow at resolved_addr,
    /// and the latency was within the documented envelope (if checked).
    Pass,
    /// Stable byte observed but it does not match the shadow.
    WrongByte { expected: u8, observed: u8 },
    /// `wait_push` timed out — PIO1 never pushed an address to CH0's RX.
    NoResolve,
    /// `wait_stable` timed out — CH1 pushed, but D0..D7 never stabilised
    /// with `pad_oe == 0xFF` for `MIN_STABLE_CYCLES` cycles.
    NoStableByte,
    /// CH1.READ_ADDR resolved to an address outside the shadow region.
    ResolvedAddrOutOfRange { addr: u32 },
    /// Stable byte matched the shadow but measured latency fell outside
    /// the `ENVELOPE_CYCLES` range. Reserved for G.3.
    LatencyOutOfEnvelope { cycles: u32 },
}

/// Per-case diagnostic result. Every field except `verdict` is
/// `Option` so partial runs (e.g. timeouts before the push arrives)
/// can still be reported cleanly.
#[derive(Clone, Copy, Debug)]
pub struct CaseResult {
    pub case: Case,
    pub resolved_addr: Option<u32>,
    pub expected_byte: Option<u8>,
    pub observed_byte: Option<u8>,
    pub latency_cycles: Option<u32>,
    pub verdict: Verdict,
}

/// One cycle's worth of observable serving-pipeline state. Used by the
/// trace-driven verdict evaluator and (implicitly) by `run_case`, which
/// builds a vector of these from live emulator state before invoking
/// the evaluator. Keeping `Observation` `pub(crate)` rather than `pub`
/// matches HLD §6.1 — it's a testing aid, not part of the binary
/// contract.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Observation {
    /// Cycles elapsed since this case's `cs_low_cycle` (the `cs_assert`
    /// transition). The first observation has `cycle == 0`.
    pub cycle: u64,
    /// `GlueDma::ch1_pushes()` sampled this cycle.
    pub ch1_pushes: u32,
    /// `bus.read32(CH1.READ_ADDR)` sampled this cycle.
    pub resolved_addr: u32,
    /// Byte currently exposed on D0..D7 (`(gpio_in >> 16) & 0xFF`).
    pub data_byte: u8,
    /// PIO2's output-enable mask over D0..D7 (`(pio[2].pad_oe >> 16) & 0xFF`).
    pub pio2_pad_oe_data: u8,
}

/// Oracle state. Owns the SRAM shadow captured at sync and the vector
/// of per-case results.
pub struct ServingOracle {
    rom_shadow: Box<[u8; SHADOW_SIZE]>,
    results: Vec<CaseResult>,
    /// Tracks whether we've driven the `init` seed yet. `run_case`
    /// does so exactly once — on the first call — to produce a clean
    /// high-to-low CS1 edge for case 1.
    seed_done: bool,
}

impl ServingOracle {
    /// Capture the SRAM shadow. Called once after the harness confirms
    /// OneROM has reached steady state (`onerom_sync::is_synced` true).
    ///
    /// Uses `bus.memory.sram_read8` rather than `bus.read8` to bypass the
    /// bus fabric: an 8192-iteration SRAM read loop through `read8` would
    /// accumulate `sram_bank_wait` contention into `bus.extra_wait_states`,
    /// perturbing the cycle accounting for the next CPU instruction (which
    /// Stage G.3 will trust for latency measurement). SHADOW_BASE is the
    /// SRAM origin (0x2000_0000), so SRAM offset `i` maps directly.
    pub fn new_at_sync(bus: &mut Bus) -> Self {
        let mut shadow = Box::new([0u8; SHADOW_SIZE]);
        for i in 0..SHADOW_SIZE {
            shadow[i] = bus.memory.sram_read8(i as u32);
        }
        Self {
            rom_shadow: shadow,
            results: Vec::new(),
            seed_done: false,
        }
    }

    /// Drive one case end-to-end. Steps the emulator and pumps the glue
    /// DMA; records per-cycle observations; runs the verdict evaluator.
    ///
    /// State machine per HLD §4.3:
    /// - `init` (first call only): drive CS1+CS2+CS3 high with addr=0
    ///   for `SEED_CYCLES`. Guarantees the next CS1-low stimulus produces
    ///   a high→low edge PIO1 can detect, regardless of what state Stage
    ///   F's external-input mask left behind.
    /// - `idle → cs_assert`: apply the case stimulus (CS1 low, CS2/CS3
    ///   high, A-bus = `case.addr_bits`). Record `cs_low_cycle` and
    ///   `ch1_pushes_before`.
    /// - `cs_assert → wait_push → wait_stable → record → cs_release → idle`:
    ///   the per-tick loop builds up an `Observation` vector for at
    ///   most `PER_CASE_TIMEOUT` cycles. At the end (or on early stable-
    ///   byte detection), the trace is fed to [`evaluate_case_trace`].
    pub fn run_case(
        &mut self,
        emu: &mut Emulator,
        glue: &mut GlueDma,
        case: Case,
    ) -> &CaseResult {
        debug_assert!(
            case.addr_bits & ADDR_A11_A12_HIGH == ADDR_A11_A12_HIGH,
            "run_case: case.addr_bits must have A11=A12=1"
        );

        // External-input mask covers CS1/CS2/CS3 and all address pins.
        // D0..D7 are PIO-driven; never mask them.
        let ext_mask: u32 = (1u32 << GPIO_CS1)
            | (1u32 << GPIO_CS2)
            | (1u32 << GPIO_CS3)
            | ADDR_PINS.iter().fold(0u32, |a, &p| a | (1u32 << p));
        emu.bus.gpio_external_mask = ext_mask;

        // 1. init seed (first call only).
        if !self.seed_done {
            let seed_level = (1u32 << GPIO_CS1) | (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
            emu.bus.gpio_external_in = seed_level;
            self.tick_cycles(emu, glue, SEED_CYCLES);
            self.seed_done = true;
        }

        // 2. cs_assert: apply the case stimulus.
        let stim_level = stimulus_level(case.addr_bits);
        emu.bus.gpio_external_in = stim_level;

        // Snapshot the push counter *before* any ticks at this stimulus.
        // HLD §4.4: the push counter is how we distinguish a fresh byte
        // arriving from residual data left by a prior case.
        //
        // `pushes_before` snapshot must happen before any run_case tick
        // advances the glue DMA. `glue.tick` is only invoked by this
        // module (here and in `tick_cycles`), so after the
        // `gpio_external_in` assignment above and before the per-cycle
        // loop below, `ch1_pushes()` is stable.
        let pushes_before = glue.ch1_pushes();

        // 3. wait_push → wait_stable: tick up to PER_CASE_TIMEOUT cycles,
        //    recording an Observation per cycle.
        let mut trace: Vec<Observation> = Vec::with_capacity(PER_CASE_TIMEOUT as usize);
        for c in 0..PER_CASE_TIMEOUT {
            emu.run(1);
            glue.tick(&mut emu.bus);

            // Plain subtraction — `glue.ch1_pushes()` is monotonic on a
            // single GlueDma, so an underflow here is a true invariant
            // violation we want to surface, not silently mask.
            let pushes = glue.ch1_pushes() - pushes_before;
            let resolved = emu
                .bus
                .read32(DMA_BASE + DMA_CH_STRIDE + DMA_CH_READ_ADDR);
            let data_byte = ((emu.bus.gpio_in >> GPIO_DATA_BASE) & 0xFF) as u8;
            let pad_oe = ((emu.bus.pio[2].pad_oe >> GPIO_DATA_BASE) & 0xFF) as u8;

            trace.push(Observation {
                cycle: c as u64,
                ch1_pushes: pushes,
                resolved_addr: resolved,
                data_byte,
                pio2_pad_oe_data: pad_oe,
            });

            // Early-exit if the verdict for the trace so far is already
            // conclusive — no need to tick out the full 60-cycle budget
            // once we've seen stability.
            if let Some(result) = try_evaluate_conclusive(case, &self.rom_shadow, &trace) {
                // 4. cs_release: drive CS1 high to settle the pipeline.
                let gap_level =
                    (1u32 << GPIO_CS1) | (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
                emu.bus.gpio_external_in = gap_level;
                self.tick_cycles(emu, glue, GAP_CYCLES);

                self.results.push(result);
                return self.results.last().unwrap();
            }
        }

        // 5. Budget exhausted — run the evaluator one last time; it'll
        //    report NoResolve / NoStableByte based on where the state
        //    machine stopped.
        let result = evaluate_case_trace(case, &self.rom_shadow, &trace);

        let gap_level = (1u32 << GPIO_CS1) | (1u32 << GPIO_CS2) | (1u32 << GPIO_CS3);
        emu.bus.gpio_external_in = gap_level;
        self.tick_cycles(emu, glue, GAP_CYCLES);

        self.results.push(result);
        self.results.last().unwrap()
    }

    /// Accessor for the full results vector.
    pub fn results(&self) -> &[CaseResult] {
        &self.results
    }

    /// Minimal per-case report. G.3 will replace this with the full
    /// table + aggregate stats + `sys_clk_hz` conversion + the
    /// emulator-bounded-ns caveat from HLD §5.4.
    // TODO(G.3): latency stats, ns conversion, ROM-speed class, caveat
    pub fn format_report(&self, sys_clk_hz: u32) -> String {
        let mut out = String::new();
        let _ = writeln!(
            out,
            "OneROM serving oracle — {} case(s), sys_clk_hz = {}",
            self.results.len(),
            sys_clk_hz
        );
        let _ = writeln!(
            out,
            "  {:<24} {:>10} {:>10} {:>8} {:>8} {:>8}  verdict",
            "case", "addr", "resolved", "expect", "observed", "cycles"
        );
        for r in &self.results {
            let addr = format!("0x{:04X}", r.case.addr_bits);
            let resolved = r
                .resolved_addr
                .map(|a| format!("0x{:08X}", a))
                .unwrap_or_else(|| "—".to_string());
            let expected = r
                .expected_byte
                .map(|b| format!("0x{:02X}", b))
                .unwrap_or_else(|| "—".to_string());
            let observed = r
                .observed_byte
                .map(|b| format!("0x{:02X}", b))
                .unwrap_or_else(|| "—".to_string());
            let cycles = r
                .latency_cycles
                .map(|c| format!("{}", c))
                .unwrap_or_else(|| "—".to_string());
            let _ = writeln!(
                out,
                "  {:<24} {:>10} {:>10} {:>8} {:>8} {:>8}  {:?}",
                r.case.label, addr, resolved, expected, observed, cycles, r.verdict
            );
        }
        out
    }

    /// Advance `emu` by `n` cycles, pumping the glue DMA each cycle.
    /// Used for the seed and gap phases, which don't need to record
    /// observations.
    fn tick_cycles(&self, emu: &mut Emulator, glue: &mut GlueDma, n: u32) {
        for _ in 0..n {
            emu.run(1);
            glue.tick(&mut emu.bus);
        }
    }
}

// ---------------------------------------------------------------------------
// Stimulus helpers
// ---------------------------------------------------------------------------

/// Build the `gpio_external_in` bitmask for a case's address stimulus.
///
/// CS1 low (asserted), CS2/CS3 high (deasserted — forced by A11/A12 = 1
/// per the pin-map collision); A0..A12 reflect `addr_bits`.
fn stimulus_level(addr_bits: u16) -> u32 {
    let mut level: u32 = 0;
    // CS2 (GPIO12)/CS3 (GPIO15) double as A12/A11 — driven high by the
    // A11=A12=1 case invariant (asserted in `Case::new`).
    for (i, &pin) in ADDR_PINS.iter().enumerate() {
        if (addr_bits >> i) & 1 != 0 {
            level |= 1u32 << pin;
        }
    }
    // CS1 stays low — do not set bit 13.
    level
}

// ---------------------------------------------------------------------------
// Trace-driven verdict evaluator (the testable core of the state machine)
// ---------------------------------------------------------------------------

/// State-machine stage used while scanning the observation trace.
///
/// `WaitStable::stable_run` tracks the in-flight stability candidate as
/// `(start_index, byte, length)`; it's `None` when we're between runs.
#[derive(Debug)]
enum EvalState {
    WaitPush,
    WaitStable {
        push_cycle: u64,
        resolved_addr: u32,
        expected: u8,
        stable_run: Option<(usize, u8, usize)>,
    },
}

/// Return a conclusive `CaseResult` if the trace is already sufficient
/// to decide the case, or `None` if more ticks are needed.
///
/// Used by `run_case` to early-exit the per-case loop once a stable byte
/// has been observed for `MIN_STABLE_CYCLES` cycles after the push. If
/// the state machine determines the case is definitively unresolvable
/// (e.g. `ResolvedAddrOutOfRange`), that also counts as conclusive.
// TODO(G.2): re-runs `evaluate_case_trace` from scratch every tick (O(N²)
// per case, N ≤ 60). Acceptable at the G.1 N=60 × single-case scale. If
// the 15-case sweep shows measurable cost, refactor to a streaming
// evaluator that carries state across ticks.
fn try_evaluate_conclusive(
    case: Case,
    shadow: &[u8; SHADOW_SIZE],
    trace: &[Observation],
) -> Option<CaseResult> {
    let result = evaluate_case_trace(case, shadow, trace);
    match result.verdict {
        // Timeouts are only meaningful after the budget is exhausted —
        // keep ticking.
        Verdict::NoResolve | Verdict::NoStableByte => None,
        _ => Some(result),
    }
}

/// Pure trace-driven verdict evaluator. Executes the HLD §4.3 state
/// machine over a synthetic `&[Observation]` sequence and returns the
/// resulting [`CaseResult`].
///
/// This is the testable core of [`ServingOracle::run_case`]: the unit
/// tests drive it with hand-crafted traces to exercise every verdict
/// variant without an emulator in the loop.
pub(crate) fn evaluate_case_trace(
    case: Case,
    shadow: &[u8; SHADOW_SIZE],
    trace: &[Observation],
) -> CaseResult {
    let mut state = EvalState::WaitPush;

    for (i, obs) in trace.iter().enumerate() {
        match &mut state {
            EvalState::WaitPush => {
                if obs.ch1_pushes > 0 {
                    // ch1_pushes is the delta from pushes_before, so > 0
                    // means a fresh push landed at or before this cycle.
                    let resolved = obs.resolved_addr;

                    if !(SHADOW_BASE..SHADOW_BASE + SHADOW_SIZE as u32).contains(&resolved) {
                        return CaseResult {
                            case,
                            resolved_addr: Some(resolved),
                            expected_byte: None,
                            observed_byte: None,
                            latency_cycles: None,
                            verdict: Verdict::ResolvedAddrOutOfRange { addr: resolved },
                        };
                    }

                    let offset = (resolved - SHADOW_BASE) as usize;
                    let expected = shadow[offset];

                    state = EvalState::WaitStable {
                        push_cycle: obs.cycle,
                        resolved_addr: resolved,
                        expected,
                        stable_run: None,
                    };
                    // Fall through: this same observation might also be
                    // the first stable cycle. But we require
                    // `first_stable_cycle > push_cycle` (HLD §3.1, §4.4),
                    // so the push cycle itself is excluded from stability.
                    continue;
                }
            }
            EvalState::WaitStable {
                push_cycle,
                resolved_addr,
                expected,
                stable_run,
            } => {
                // Stability requires pad_oe=0xFF AND we're strictly after
                // the push cycle (HLD §4.4 — the residue-rejection rule).
                let after_push = obs.cycle > *push_cycle;
                let drives_all = obs.pio2_pad_oe_data == 0xFF;

                if after_push && drives_all {
                    match stable_run {
                        Some((_start, byte, len)) if *byte == obs.data_byte => {
                            *len += 1;
                            if *len >= MIN_STABLE_CYCLES {
                                // Stable! Latency = stable_cycle - cs_low_cycle.
                                // stable_cycle is the cycle of the FIRST
                                // observation in the stable run: current
                                // index i, run length len, so start = i+1-len.
                                // cs_low_cycle is trace[0].cycle (== 0 by
                                // construction of `run_case`).
                                let stable_start_idx = i + 1 - *len;
                                let stable_cycle = trace[stable_start_idx].cycle;
                                // `trace[0]` is safe here: we only reach this
                                // arm after consuming at least MIN_STABLE_CYCLES
                                // observations, so the trace is non-empty.
                                let cs_low_cycle = trace[0].cycle;
                                let latency = (stable_cycle - cs_low_cycle) as u32;
                                let observed = *byte;

                                let verdict = if observed == *expected {
                                    Verdict::Pass
                                } else {
                                    Verdict::WrongByte {
                                        expected: *expected,
                                        observed,
                                    }
                                };

                                return CaseResult {
                                    case,
                                    resolved_addr: Some(*resolved_addr),
                                    expected_byte: Some(*expected),
                                    observed_byte: Some(observed),
                                    latency_cycles: Some(latency),
                                    verdict,
                                };
                            }
                        }
                        _ => {
                            *stable_run = Some((i, obs.data_byte, 1));
                        }
                    }
                } else {
                    // Break the stable run — either pad_oe dropped or
                    // we're still at/before push_cycle.
                    *stable_run = None;
                }
            }
        }
    }

    // Trace ran out before the state machine resolved.
    match state {
        EvalState::WaitPush => CaseResult {
            case,
            resolved_addr: None,
            expected_byte: None,
            observed_byte: None,
            latency_cycles: None,
            verdict: Verdict::NoResolve,
        },
        EvalState::WaitStable { resolved_addr, expected, .. } => CaseResult {
            case,
            resolved_addr: Some(resolved_addr),
            expected_byte: Some(expected),
            observed_byte: None,
            latency_cycles: None,
            verdict: Verdict::NoStableByte,
        },
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use mdrp2350::{Config, EmulatorBuilder};

    fn mk_case() -> Case {
        Case::new("test", 0x1800)
    }

    fn empty_shadow() -> Box<[u8; SHADOW_SIZE]> {
        Box::new([0u8; SHADOW_SIZE])
    }

    /// 1. `new_at_sync` captures SRAM[SHADOW_BASE..+SHADOW_SIZE] byte-for-byte.
    ///
    /// This is the only test that touches a real emulator: everything
    /// else is trace-driven.
    #[test]
    fn rom_shadow_captures_sram_at_build_time() {
        let mut emu = EmulatorBuilder::new(Config::default()).build();
        // Write a walking pattern byte-i = i-as-u8 across the full shadow.
        for i in 0..SHADOW_SIZE {
            emu.bus.write8(SHADOW_BASE + i as u32, i as u8);
        }
        let oracle = ServingOracle::new_at_sync(&mut emu.bus);
        for i in 0..SHADOW_SIZE {
            assert_eq!(
                oracle.rom_shadow[i], i as u8,
                "shadow[{}] = 0x{:02X}, expected 0x{:02X}",
                i, oracle.rom_shadow[i], i as u8
            );
        }
    }

    /// 2. Happy-path PASS: push at cycle 5, stable 0x42 from cycle 12 for 3 cycles.
    #[test]
    fn verdict_pass_when_byte_matches_shadow_after_push() {
        let case = mk_case();
        let mut shadow = empty_shadow();
        let offset = 0x10usize;
        shadow[offset] = 0x42;
        let resolved = SHADOW_BASE + offset as u32;

        // Trace: cycles 0..=14. push at cycle 5, data stable at 0x42
        // with pad_oe=0xFF at cycles 12, 13, 14.
        let mut trace = Vec::new();
        for c in 0..15u64 {
            let pushes = if c >= 5 { 1 } else { 0 };
            let data_byte = if c >= 12 { 0x42 } else { 0x00 };
            let pad_oe = if c >= 12 { 0xFF } else { 0x00 };
            trace.push(Observation {
                cycle: c,
                ch1_pushes: pushes,
                resolved_addr: resolved,
                data_byte,
                pio2_pad_oe_data: pad_oe,
            });
        }

        let result = evaluate_case_trace(case, &shadow, &trace);
        assert_eq!(result.verdict, Verdict::Pass);
        assert_eq!(result.latency_cycles, Some(12));
        assert_eq!(result.resolved_addr, Some(resolved));
        assert_eq!(result.expected_byte, Some(0x42));
        assert_eq!(result.observed_byte, Some(0x42));
    }

    /// 3. WrongByte when observed ≠ shadow.
    #[test]
    fn verdict_wrong_byte_when_observed_mismatches_shadow() {
        let case = mk_case();
        let mut shadow = empty_shadow();
        shadow[0x10] = 0x42;
        let resolved = SHADOW_BASE + 0x10;

        let mut trace = Vec::new();
        for c in 0..15u64 {
            let pushes = if c >= 5 { 1 } else { 0 };
            let data_byte = if c >= 12 { 0x00 } else { 0x55 };
            let pad_oe = if c >= 12 { 0xFF } else { 0x00 };
            trace.push(Observation {
                cycle: c,
                ch1_pushes: pushes,
                resolved_addr: resolved,
                data_byte,
                pio2_pad_oe_data: pad_oe,
            });
        }

        let result = evaluate_case_trace(case, &shadow, &trace);
        assert_eq!(
            result.verdict,
            Verdict::WrongByte { expected: 0x42, observed: 0x00 }
        );
        assert_eq!(result.latency_cycles, Some(12));
    }

    /// 4. Prior-case residue: data is already 0xAA at cycle 0, but the
    /// push only happens at cycle 5. The stable run must not start at
    /// cycle 0 — the first_stable_cycle > push_cycle rule forces the
    /// latency measurement to anchor after cycle 5. The earliest
    /// stable cycle is 6 (pad_oe=0xFF + cycle > 5), so 3-cycle run
    /// completes at cycle 8 → stable_cycle=6 → latency=6.
    ///
    // Validates: residue rejection — the push-anchored latency anchors
    // after the push cycle, not at cycle 0 of a residual byte.
    // Does NOT directly validate: the `>` vs `>=` distinction on line 497.
    // The `continue;` after the WaitPush→WaitStable transition already
    // prevents the push cycle from entering the WaitStable arm, so either
    // mutation of that comparator survives this test. The `>` guard is
    // belt-and-braces alongside the `continue;`. If either the `continue;`
    // OR the `>` guard is removed the other still protects; remove with
    // caution.
    #[test]
    fn verdict_rejects_prior_case_residue() {
        let case = mk_case();
        let mut shadow = empty_shadow();
        shadow[0x20] = 0xAA;
        let resolved = SHADOW_BASE + 0x20;

        // Data byte 0xAA and pad_oe 0xFF from cycle 0 onward (residue).
        // Push happens at cycle 5.
        let mut trace = Vec::new();
        for c in 0..15u64 {
            let pushes = if c >= 5 { 1 } else { 0 };
            trace.push(Observation {
                cycle: c,
                ch1_pushes: pushes,
                resolved_addr: resolved,
                data_byte: 0xAA,
                pio2_pad_oe_data: 0xFF,
            });
        }

        let result = evaluate_case_trace(case, &shadow, &trace);
        assert_eq!(
            result.verdict,
            Verdict::Pass,
            "residue rejection should still PASS once anchored after push"
        );
        // cs_low_cycle = 0; first cycle strictly after push_cycle=5 is
        // cycle 6; that starts the 3-cycle stable run (6,7,8); stable_cycle=6;
        // latency = 6 - 0 = 6.
        assert_eq!(
            result.latency_cycles,
            Some(6),
            "latency must anchor after push, not at cycle 0 of residual byte"
        );
    }

    /// 5. `resolved_addr` outside the shadow range → ResolvedAddrOutOfRange.
    #[test]
    fn verdict_resolved_addr_out_of_range() {
        let case = mk_case();
        let shadow = empty_shadow();
        let bad_addr = 0x2100_0000u32;

        let trace = vec![
            Observation {
                cycle: 0,
                ch1_pushes: 0,
                resolved_addr: 0,
                data_byte: 0,
                pio2_pad_oe_data: 0,
            },
            Observation {
                cycle: 5,
                ch1_pushes: 1,
                resolved_addr: bad_addr,
                data_byte: 0,
                pio2_pad_oe_data: 0,
            },
        ];

        let result = evaluate_case_trace(case, &shadow, &trace);
        assert_eq!(
            result.verdict,
            Verdict::ResolvedAddrOutOfRange { addr: bad_addr }
        );
        assert_eq!(result.resolved_addr, Some(bad_addr));
        assert!(result.expected_byte.is_none());
    }
}

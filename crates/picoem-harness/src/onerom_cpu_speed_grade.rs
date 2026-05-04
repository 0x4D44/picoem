//! OneROM CPU-mode speed-grade oracle — stability-free, testable core.
//!
//! Phase 3 of the 2026-04-22 HLD V3 (`wrk_docs/2026.04.22 - HLD -
//! OneROM CPU Speed Grade Oracle.md`). Companion to the binary
//! `onerom_cpu_speed_grade_rp2350`, which owns the host-thread /
//! driver / measurement / reporting wiring; everything that doesn't
//! need a live `ThreadedEmulator` lives here so it can be unit-tested
//! on any host.
//!
//! Responsibilities:
//!
//! - Walk-plan generation: given a loaded `1541-cpu` fixture, produce
//!   the full 8 KB address walk (8192 `WalkStep`s) encoded as
//!   `(gpio_stim, gpio_mask, expected, addr)` tuples. The expected
//!   byte is looked up against the pre-processed ROM-table shadow
//!   lifted by [`crate::onerom_serving_oracle::lift_shadow_from_flash_pub`],
//!   mirroring `onerom_stress_cpu_rp2350` so the two binaries agree
//!   on ground truth.
//! - Per-sweep shuffle: deterministic `StdRng::seed_from_u64(seed ^
//!   sweep_idx)` Fisher-Yates over `0..plan.len()`, returning a
//!   permutation index vector (not a rebuilt plan — walk plans are
//!   ~200 KB and reshuffling indices is ~32 KB).
//! - Ladder definition + verification: the default ns-target ladder
//!   and a pure `verify_observed` that walks the observation buffer
//!   and returns the first mismatch (if any) as a `FailContext`.
//!
//! Pin layout flows through a [`FixtureSpec`] supplied by the caller —
//! the binary parses it once at startup via
//! [`FixtureSpec::from_flash`] and threads it through `build_walk_plan`
//! and the measurement helpers. Stage 2 of the fixture-generalization
//! HLD eliminated the pin-map duplicates that lived here pre-restructure.
//!
//! Unit tests cover the pure bits below. The end-to-end gate is the
//! binary run; this module owns none of the threading, timing, or
//! report-format surface.

use crate::onerom_fixture::{FixtureSpec, lift_shadow_from_flash};
use crate::onerom_serving_oracle::Case;

use rand::RngCore;
use rand::SeedableRng;
use rand::rngs::StdRng;

// ---------------------------------------------------------------------------
// Ladder defaults
// ---------------------------------------------------------------------------

/// Default access-time ladder (ns). HLD §9: 8 rungs from 500 ns down
/// to 100 ns. `Instant::now()` on Windows is ~20 ns/call so 100 ns is
/// the realistic floor — below that the timing source overhead
/// dominates the budget.
pub const DEFAULT_LADDER: &[u32] = &[500, 400, 300, 250, 200, 150, 120, 100];

/// One rung of the speed-grade ladder. Thin newtype so the binary can
/// build a typed `Vec<LadderRung>` from CLI input without the caller
/// juggling raw `u32`s.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LadderRung {
    pub target_ns: u32,
}

impl LadderRung {
    pub const fn new(target_ns: u32) -> Self {
        Self { target_ns }
    }
}

// ---------------------------------------------------------------------------
// Walk plan
// ---------------------------------------------------------------------------

/// One step of the walk plan. Captures the exact external stimulus to
/// apply (`gpio_stim` with `gpio_mask` identifying which bits are
/// externally driven) plus the expected byte at that address. `addr`
/// is a diagnostic carry — the 13-bit address encoded by this step,
/// used to populate `FailContext::addr` without re-decoding
/// `gpio_stim`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WalkStep {
    /// External-input stimulus for `shared.gpio.write_external`. CS1
    /// is held low (not set in this value); CS2/CS3 follow A11/A12 on
    /// this pin map (`test-sdrr-0` / 1541-set-0 convention).
    pub gpio_stim: u32,
    /// Union of all externally-driven pins. Stays constant across all
    /// walk steps — declared on each step for caller ergonomics (the
    /// measurement loop doesn't have to special-case the first step).
    pub gpio_mask: u32,
    /// Expected data byte. Looked up against the pre-processed shadow
    /// (same path `CpuServingOracle` uses), so a matching observation
    /// is a true PASS — not an accidental match.
    pub expected: u8,
    /// Decoded 13-bit address (0..8192). Diagnostic only; the byte
    /// lookup already bakes the address into `gpio_stim`.
    pub addr: u16,
}

// Pin constants live on the supplied `FixtureSpec`. `build_walk_plan`
// composes the same `gpio_mask` shape (gate CS + every deasserted-high
// CS pin + every asserted-low pin + every address pin) that the
// `CpuServingOracle::run_case` external-input mask uses.

/// Base pin index for the 8-bit data bus on the legacy fire-24-a
/// fixture (D0..D7 → GPIO 16..23). Kept as a public constant so the
/// 1541-targeted speed-grade binary can extract the observed byte from
/// the bus word without re-deriving the offset every time. Stage 2
/// notes: a fixture-aware version would query `spec.data_pins[0]` —
/// the binary still does it this way because its measurement loop
/// pre-dates the spec plumbing; a follow-up will switch it over.
pub const GPIO_DATA_BASE: u8 = 16;

/// Walk-plan length: 2048 cases — the A11=A12=1 subspace where the
/// stimulus keeps both double-duty pins (CS2/CS3 on GPIO12/GPIO15)
/// at their "selected" level.
///
/// HLD V3 §3 asked for "8192 samples / full 8 KB coverage"; that's
/// not reachable through the 1541-cpu firmware's pin bake. The SDRR
/// pre-processed shadow at raw-pattern offsets where A11 or A12 is
/// zero is either "deselected" output (not ROM content) or masked;
/// the existing `onerom_stress_cpu_rp2350` binary tests exactly this
/// A11=A12=1 subspace (2048 cases) and reports 2048/2048 PASS, so
/// that's our known-valid ground-truth range. Walking outside it
/// produces legitimate-but-unhelpful mismatches (firmware serves
/// byte X from the shadow, our lifted shadow predicts byte Y from
/// the same raw flash slot, because the runtime effectively
/// overrides the raw bytes for deselected patterns).
///
/// Tradeoff: the ladder's effective address-space coverage halves
/// against what the HLD envisioned, but each rung still sees 6144
/// samples/sweep (2048 × 3), so statistical confidence remains very
/// strong — a single-byte emulator bug at any one of 8192 addresses
/// would still show up as an error rate > 0 in verification.
pub const WALK_PLAN_LEN: usize = 2048;

/// The A11=A12 bits in the 13-bit case-address space. Cases in the
/// walk satisfy `(addr & ADDR_A11_A12_HIGH) == ADDR_A11_A12_HIGH`.
/// Matches [`crate::onerom_serving_oracle::ADDR_A11_A12_HIGH`]; the
/// stress sweep uses the same constant, so the walk plan covers
/// exactly the stress binary's address space.
pub const ADDR_A11_A12_HIGH: u16 = 0x1800;

/// Hardcoded ROM-set index for the 1541 CPU fixture. Set 0 is the
/// 2364 `1541-e000.901229-06AA.bin` image (the $E000 kernal), which
/// matches the `onerom_stress_cpu_rp2350` target post-Phase-1'. The
/// binary forces this index via
/// [`crate::onerom_serving_oracle_cpu::force_rom_set_index_via_sel_pins`]
/// before boot-sync so the firmware lifts the same shadow this
/// function reads from flash.
pub const ROM_SET_INDEX: u8 = 0;

/// Build the full walk plan against the shadow lifted from the
/// fixture's ROM set 0. Returns a `WALK_PLAN_LEN`-entry vector, one per
/// 13-bit address in the A11=A12=1 subspace.
///
/// Each step:
/// - `gpio_stim` is composed by [`Case::from_addr`] (address bits
///   permuted through `spec.addr_pins`) ORed with the deasserted-high
///   CS pins from `spec.deasserted_high_during_read`. The gate CS
///   (`spec.cs1`) stays LOW — that's what selects the chip.
/// - `gpio_mask` covers gate CS, every deasserted-high CS pin, every
///   asserted-low pin, and every address pin. Mirrors
///   `CpuServingOracle::run_case`'s external-input mask.
/// - `expected` is `shadow[gpio_stim & 0xFFFF]` — the same shadow
///   lookup the CPU performs in its serve loop.
///
/// Returns `Err` on any shadow-lift parse failure.
pub fn build_walk_plan(flash: &[u8], spec: &FixtureSpec) -> Result<Vec<WalkStep>, String> {
    let shadow = lift_shadow_from_flash(flash, ROM_SET_INDEX, spec)
        .ok_or_else(|| "failed to lift ROM set 0 shadow from flash".to_string())?;

    // Compose the external-input mask from the spec.
    let mut mask_u64: u64 = 1u64 << spec.cs1;
    for &p in &spec.deasserted_high_during_read {
        mask_u64 |= 1u64 << p;
    }
    for &p in &spec.asserted_low_during_read {
        mask_u64 |= 1u64 << p;
    }
    for &p in &spec.addr_pins {
        mask_u64 |= 1u64 << p;
    }
    debug_assert!(
        mask_u64 >> 32 == 0,
        "build_walk_plan: gpio_mask uses GPIOs >= 32; widen Bus interface for fire-32-a (Stage 3)"
    );
    let mask = mask_u64 as u32;

    // Pre-compute the deasserted-high contribution (this is what the
    // oracle's stim composition adds before ORing in the case pattern).
    let mut deasserted_high: u64 = 0;
    for &p in &spec.deasserted_high_during_read {
        deasserted_high |= 1u64 << p;
    }

    let mut plan = Vec::with_capacity(WALK_PLAN_LEN);
    for low11 in 0..WALK_PLAN_LEN as u16 {
        let addr_bits = ADDR_A11_A12_HIGH | low11;
        let case = Case::from_addr("walk", addr_bits as u32, spec);
        let stim_u64 = deasserted_high | case.pin_pattern;
        debug_assert!(
            stim_u64 >> 32 == 0,
            "build_walk_plan: gpio_stim uses GPIOs >= 32"
        );
        let stim = stim_u64 as u32;
        let shadow_offset = (stim & 0xFFFF) as usize;
        let expected = shadow[shadow_offset];
        plan.push(WalkStep {
            gpio_stim: stim,
            gpio_mask: mask,
            expected,
            addr: addr_bits,
        });
    }
    Ok(plan)
}

// ---------------------------------------------------------------------------
// Shuffle
// ---------------------------------------------------------------------------

/// Build a Fisher-Yates permutation of `0..plan_len` seeded from
/// `seed ^ sweep_idx`. Returning indices (rather than a rebuilt
/// plan) saves ~200 KB of allocation per sweep — the plan itself
/// is immutable across sweeps.
///
/// The per-sweep XOR decorrelates adjacent sweeps even when the base
/// seed repeats across invocations; `StdRng::seed_from_u64` is
/// deterministic so the full run is reproducible from `(seed,
/// sweep_idx)`.
pub fn shuffle_plan(plan_len: usize, seed: u64, sweep_idx: u32) -> Vec<u32> {
    let mut out: Vec<u32> = (0..plan_len as u32).collect();
    let mut rng = StdRng::seed_from_u64(seed ^ sweep_idx as u64);
    // Fisher-Yates: pick j in 0..=i, swap i with j, for i in len-1..=1.
    for i in (1..out.len()).rev() {
        let j = (rng.next_u64() % (i as u64 + 1)) as usize;
        out.swap(i, j);
    }
    out
}

// ---------------------------------------------------------------------------
// Verification
// ---------------------------------------------------------------------------

/// Diagnostic tuple for the first observed mismatch in a sweep. Seeded
/// by [`verify_observed`] so the binary's report can print exactly
/// which byte diverged.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FailContext {
    pub sweep_idx: u32,
    pub sample_idx: u32,
    pub addr: u16,
    pub expected: u8,
    pub observed: u8,
}

/// Summary of one rung's result. Populated by the binary's main loop
/// after all sweeps for a given `target_ns` complete. `samples` is
/// the total byte observations across every sweep at this rung;
/// `errors` counts how many disagreed with the shadow. `host_stalled`
/// counts samples where the coordinator didn't advance during the
/// wait window (host-level DPC/ISR preemption) — these are excluded
/// from `errors` because they measure host noise, not emulator
/// capability.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SweepReport {
    pub target_ns: u32,
    pub sweeps: u32,
    pub samples: u32,
    pub errors: u32,
    pub host_stalled: u32,
    pub first_fail: Option<FailContext>,
}

impl SweepReport {
    pub fn verdict_passes(&self) -> bool {
        self.errors == 0
    }
}

/// Walk through one sweep's observations and return the first
/// mismatch, if any. Pure over `plan` / `shuffle` / `observed`, so
/// the binary can run this off the timed path (after `stop` is set
/// and workers have drained).
///
/// Contract: `observed.len() == shuffle.len()`; each
/// `observed[i]` is the data byte sampled after driving the stimulus
/// at `plan[shuffle[i]]`. Panics in debug builds on length mismatch
/// — the binary must hand in consistent buffers.
pub fn verify_observed(
    plan: &[WalkStep],
    shuffle: &[u32],
    observed: &[u8],
    _target_ns: u32,
    sweep_idx: u32,
) -> Option<FailContext> {
    debug_assert_eq!(
        shuffle.len(),
        observed.len(),
        "verify_observed: shuffle/observed length mismatch"
    );
    for (i, (&perm, &obs)) in shuffle.iter().zip(observed.iter()).enumerate() {
        let step = &plan[perm as usize];
        if obs != step.expected {
            return Some(FailContext {
                sweep_idx,
                sample_idx: i as u32,
                addr: step.addr,
                expected: step.expected,
                observed: obs,
            });
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Ladder iteration helper
// ---------------------------------------------------------------------------

/// Iterate `reports` in order; if any `report.errors > 0`, stop
/// there. Returns the index of the first failing rung (or
/// `reports.len()` if all passed). Pure helper extracted so the
/// "stop at first fail" policy can be unit-tested without a live
/// emulator.
pub fn first_failing_rung(reports: &[SweepReport]) -> usize {
    for (i, r) in reports.iter().enumerate() {
        if !r.verdict_passes() {
            return i;
        }
    }
    reports.len()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Fixture path hardcoded to the committed 1541-cpu fixture — the
    /// same one `onerom_stress_cpu_rp2350` targets. Tests that need the
    /// real fixture load it relative to the workspace cwd.
    const FIXTURE_PATH: &str = "fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

    fn load_fixture() -> Vec<u8> {
        // Tests run from `crates/picoem-harness/`; fixture lives
        // under that crate's `fixtures/`.
        std::fs::read(FIXTURE_PATH)
            .unwrap_or_else(|e| panic!("failed to load {}: {}", FIXTURE_PATH, e))
    }

    fn load_spec() -> FixtureSpec {
        FixtureSpec::from_flash(&load_fixture()).expect("FixtureSpec parse must succeed")
    }

    /// The walk plan covers every A11=A12=1 case (addr_bits ∈
    /// 0x1800..=0x1FFF), each exactly once. Validates the core shape
    /// invariant: no duplicates, no gaps.
    #[test]
    fn walk_plan_covers_2kb_a11_a12_high_subspace() {
        let flash = load_fixture();
        let spec = FixtureSpec::from_flash(&flash).expect("spec parse");
        let plan = build_walk_plan(&flash, &spec).expect("build_walk_plan");
        assert_eq!(plan.len(), WALK_PLAN_LEN, "walk plan must be 2048 steps");

        let mut seen = vec![false; WALK_PLAN_LEN];
        for step in &plan {
            assert_eq!(
                step.addr & ADDR_A11_A12_HIGH,
                ADDR_A11_A12_HIGH,
                "addr 0x{:04X} violates A11=A12=1 invariant",
                step.addr
            );
            let idx = (step.addr & !ADDR_A11_A12_HIGH) as usize;
            assert!(
                idx < WALK_PLAN_LEN,
                "low11 {} out of range for 0x{:04X}",
                idx,
                step.addr
            );
            assert!(
                !seen[idx],
                "addr 0x{:04X} (low11={}) appears twice",
                step.addr, idx
            );
            seen[idx] = true;
        }
        assert!(
            seen.iter().all(|&b| b),
            "at least one A11=A12=1 addr missing from walk plan"
        );
    }

    /// For a sampling of addresses, re-extracting the 13-bit address
    /// from `gpio_stim` via `spec.addr_pins` must yield the same
    /// address. Load-bearing property for the measurement path.
    #[test]
    fn walk_plan_stim_encodes_address_correctly() {
        let flash = load_fixture();
        let spec = load_spec();
        let plan = build_walk_plan(&flash, &spec).expect("build_walk_plan");

        let samples: Vec<u16> = (0..11u16)
            .map(|bit| ADDR_A11_A12_HIGH | (1u16 << bit))
            .chain([
                ADDR_A11_A12_HIGH,
                ADDR_A11_A12_HIGH | 0x07FF,
                ADDR_A11_A12_HIGH | 0x02AA,
            ])
            .collect();
        for &addr in &samples {
            let step = plan
                .iter()
                .find(|s| s.addr == addr)
                .unwrap_or_else(|| panic!("addr 0x{:04X} not in plan", addr));
            // Rebuild the 13-bit address from the stim word by
            // walking spec.addr_pins.
            let mut decoded: u16 = 0;
            for (i, &pin) in spec.addr_pins.iter().enumerate().take(13) {
                if (step.gpio_stim >> pin) & 1 != 0 {
                    decoded |= 1u16 << i;
                }
            }
            assert_eq!(
                decoded, addr,
                "addr 0x{:04X} stim 0x{:08X} decoded to 0x{:04X}",
                addr, step.gpio_stim, decoded
            );
            // Gate CS (cs1) must stay low.
            assert_eq!(
                step.gpio_stim & (1u32 << spec.cs1),
                0,
                "gate CS must be low in every stim (addr {:#06x})",
                addr
            );
            // Mask must include gate CS, every deasserted/asserted CS pin,
            // and every address pin.
            let mut expected_mask: u64 = 1u64 << spec.cs1;
            for &p in &spec.deasserted_high_during_read {
                expected_mask |= 1u64 << p;
            }
            for &p in &spec.asserted_low_during_read {
                expected_mask |= 1u64 << p;
            }
            for &p in &spec.addr_pins {
                expected_mask |= 1u64 << p;
            }
            assert_eq!(
                step.gpio_mask as u64, expected_mask,
                "gpio_mask must cover gate CS + dehigh/aslow CS pins + ADDR_PINS"
            );
        }
    }

    /// `first_failing_rung` returns the first index whose `errors >
    /// 0`, or `len()` if all pass. Validates the ladder's "stop at
    /// first fail" contract.
    #[test]
    fn ladder_stops_at_first_fail() {
        let reports = vec![
            SweepReport {
                target_ns: 500,
                sweeps: 3,
                samples: 24576,
                errors: 0,
                host_stalled: 0,
                first_fail: None,
            },
            SweepReport {
                target_ns: 400,
                sweeps: 3,
                samples: 24576,
                errors: 0,
                host_stalled: 0,
                first_fail: None,
            },
            SweepReport {
                target_ns: 300,
                sweeps: 3,
                samples: 24576,
                errors: 7,
                host_stalled: 0,
                first_fail: Some(FailContext {
                    sweep_idx: 1,
                    sample_idx: 42,
                    addr: 0x0A91,
                    expected: 0x2B,
                    observed: 0x00,
                }),
            },
            SweepReport {
                target_ns: 250,
                sweeps: 3,
                samples: 24576,
                errors: 99,
                host_stalled: 0,
                first_fail: None,
            },
        ];
        assert_eq!(first_failing_rung(&reports), 2);

        // All-pass → len().
        let all_pass: Vec<SweepReport> = reports
            .iter()
            .cloned()
            .map(|mut r| {
                r.errors = 0;
                r.first_fail = None;
                r
            })
            .collect();
        assert_eq!(first_failing_rung(&all_pass), all_pass.len());

        // First-rung fail → 0.
        let mut first_fail = reports.clone();
        first_fail[0].errors = 1;
        assert_eq!(first_failing_rung(&first_fail), 0);
    }

    /// `verify_observed` reports the first mismatched sample in scan
    /// order with the decoded `addr` + expected/observed bytes.
    #[test]
    fn verify_reports_first_mismatch() {
        let flash = load_fixture();
        let spec = load_spec();
        let plan = build_walk_plan(&flash, &spec).expect("build_walk_plan");

        // Trivial shuffle: identity.
        let shuffle: Vec<u32> = (0..plan.len() as u32).collect();

        // All-correct observed → None.
        let observed: Vec<u8> = plan.iter().map(|s| s.expected).collect();
        assert!(verify_observed(&plan, &shuffle, &observed, 500, 0).is_none());

        // Inject a mismatch at sample 17. The step at shuffle[17] is
        // plan[17]; flip its expected byte.
        let mut bad = observed.clone();
        let target = &plan[17];
        let injected = target.expected.wrapping_add(1);
        bad[17] = injected;
        let fail = verify_observed(&plan, &shuffle, &bad, 500, 4)
            .expect("must detect the injected mismatch");
        assert_eq!(fail.sweep_idx, 4);
        assert_eq!(fail.sample_idx, 17);
        assert_eq!(fail.addr, target.addr);
        assert_eq!(fail.expected, target.expected);
        assert_eq!(fail.observed, injected);

        // Earlier mismatch wins — inject another at sample 3.
        let mut bad2 = bad.clone();
        let earlier = &plan[3];
        bad2[3] = earlier.expected.wrapping_add(2);
        let fail = verify_observed(&plan, &shuffle, &bad2, 500, 4)
            .expect("must detect the earlier mismatch first");
        assert_eq!(fail.sample_idx, 3);
        assert_eq!(fail.addr, earlier.addr);
    }

    /// Fisher-Yates output is a permutation of `0..plan_len` — every
    /// index appears exactly once.
    #[test]
    fn shuffle_is_permutation_of_indices() {
        let len = WALK_PLAN_LEN;
        let perm = shuffle_plan(len, 0x1541_CAFE_0000_0001, 7);
        assert_eq!(perm.len(), len);

        let mut seen = vec![false; len];
        for &idx in &perm {
            let i = idx as usize;
            assert!(i < len, "permutation entry {} out of range", i);
            assert!(!seen[i], "permutation repeats index {}", i);
            seen[i] = true;
        }
        assert!(seen.iter().all(|&b| b));

        // Tiny plan to exercise the loop at a different length.
        let small = shuffle_plan(4, 0xABCD, 3);
        assert_eq!(small.len(), 4);
        let mut small_seen = [false; 4];
        for &idx in &small {
            small_seen[idx as usize] = true;
        }
        assert!(small_seen.iter().all(|&b| b));
    }

    /// Shuffle is deterministic: same `(seed, sweep_idx)` → same
    /// permutation. Different `sweep_idx` → (very likely) different
    /// permutation (we just assert non-equality — collision
    /// probability is 1/8192! ≈ 0).
    #[test]
    fn shuffle_deterministic_and_sweep_distinct() {
        let a = shuffle_plan(WALK_PLAN_LEN, 0x42, 0);
        let b = shuffle_plan(WALK_PLAN_LEN, 0x42, 0);
        assert_eq!(a, b, "same (seed, sweep_idx) must produce same permutation");

        let c = shuffle_plan(WALK_PLAN_LEN, 0x42, 1);
        assert_ne!(
            a, c,
            "different sweep_idx must produce a different permutation \
             (collision is astronomically unlikely)"
        );
    }

    /// The default ladder is monotonically decreasing (500 → 100),
    /// which the binary relies on to stop at first fail without
    /// scanning backwards.
    #[test]
    fn default_ladder_monotone_decreasing() {
        for win in DEFAULT_LADDER.windows(2) {
            assert!(
                win[0] > win[1],
                "ladder must be strictly decreasing: {} !> {}",
                win[0],
                win[1]
            );
        }
        assert_eq!(*DEFAULT_LADDER.first().unwrap(), 500);
        assert_eq!(*DEFAULT_LADDER.last().unwrap(), 100);
        assert_eq!(DEFAULT_LADDER.len(), 8);
    }
}

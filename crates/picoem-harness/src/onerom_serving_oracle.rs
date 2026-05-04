//! OneROM serving oracle — byte-correctness + timing envelope (Stage G).
//!
//! Stage F validated that *some* stable byte appears on D0..D7 after sync;
//! it did not validate the byte's value, sweep multiple addresses, or
//! measure the CS-to-valid-data latency. This module is the Stage G
//! pipeline: snapshot SRAM at sync (the SRAM-base shadow, §3.1), drive
//! pin stimuli, and for each case prove the observed byte matches the
//! shadow byte that CH1.READ_ADDR actually resolved to.
//!
//! Stage 2 of the fixture-generalization HLD (`wrk_docs/2026.05.04 - HLD -
//! OneROM Serving Oracle Fixture Generalization.md`) drops the hardcoded
//! 24-pin pin-map / 64 KiB shadow and consumes a [`crate::onerom_fixture::FixtureSpec`]
//! as the single source of truth for pin numbering, deassert/assert
//! levels, and shadow size. fire-24-a behaviour is preserved bit-for-bit;
//! fire-32-a (Stage 3) plugs into the same `FixtureSpec` parsing path.
//!
//! Design: `wrk_docs/2026.04.15 - HLD - OneROM Serving Oracle (Stage G).md`
//! plus the 2026.05.04 fixture-generalization HLD.
//!
//! Key invariants enforced by this module (see HLD §4.3, §4.4):
//! - Stability is anchored *after* the CH1 push cycle. A trace whose
//!   `data_byte` happens to hold the prior case's byte at cs_low must
//!   not report a fictional zero-latency PASS. See §4.4.
//! - `resolved_addr` must fall in `[SHADOW_BASE, SHADOW_BASE + spec.shadow_size)`
//!   before we look up `expected_byte`. Outside → `ResolvedAddrOutOfRange`.

use std::fmt::Write as _;
use std::sync::atomic::Ordering;

use rp2350_emu::{Bus, Emulator};

use crate::onerom_fixture::{FixtureError, FixtureSpec, lift_shadow_from_flash};
use crate::onerom_glue_dma::{DMA_READ_CYCLES, DMA_WRITE_CYCLES, GlueDma};

// ---------------------------------------------------------------------------
// Public constants
// ---------------------------------------------------------------------------

/// Base of the SRAM shadow — matches the OneROM runtime's `rom_table`
/// destination on RP2350. `preload_rom_image` copies the per-set
/// pre-processed ROM image to `_ram_rom_image_start`, which this firmware
/// build places at SRAM origin (confirmed via `sdrr_runtime_info` at
/// `0x20080000`: `rom_table = 0x20000000`).
pub const SHADOW_BASE: u32 = 0x2000_0000;

/// Acceptable CS-low-to-stable-byte cycle envelope.
///
/// This envelope is the emulator-model's observed steady-state window
/// for the `test-sdrr-0` fixture plus the glue DMA implementation in
/// `onerom_glue_dma.rs`. It is *emulator-bounded*, not
/// silicon-calibrated (see HLD §5.4) — silicon-tracked timing remains
/// a future pass via `silicon_cycle_oracle_rp2350`, which will measure
/// CS-to-valid-data latency on real RP2354 hardware.
///
/// The previous `11..=14` window was an aspirational ideal-pipeline
/// target taken verbatim from Piers' `piorom.c`. Once address and byte
/// correctness were closed (post `last_pushed_read_addr` + stim-
/// predicate fix on 2026-04-17), the real emulator steady-state ranges
/// 19..=38 cycles, driven by drain residue from continuous PIO1 SM0
/// background activity competing with the per-case stim push for the
/// glue DMA pipeline. The widened envelope accommodates that residue.
///
/// A case that resolves to the correct address and serves the correct
/// byte but sits outside this window is a *pipeline-model regression*,
/// not a functional failure — byte/address correctness are enforced
/// upstream via `Verdict::WrongByte` / `Verdict::ResolvedAddrOutOfRange`.
pub const ENVELOPE_CYCLES: std::ops::RangeInclusive<u32> = 15..=45;

// ---------------------------------------------------------------------------
// Internal constants
// ---------------------------------------------------------------------------

/// Minimum consecutive cycles of the same byte (with `pad_oe == 0xFF`)
/// required to declare a value "stable".
const MIN_STABLE_CYCLES: usize = 3;

/// Cycles of CS-high + addr=0 we drive on the first case to guarantee a
/// clean high-to-low edge on the gate CS. Stage F ends with the gate CS
/// low, so without this seed case 1 would never see a transition.
/// Applied once, in the `init` transition (HLD §4.3).
const SEED_CYCLES: u32 = 4;

/// Cycles of gap-level (gate CS + deasserted-high CS pins high, addr=0)
/// we drive at the start of each case to put PIO in a known CS-high
/// state before stimulus.
///
/// Earlier versions (H2 in the Stage G fix-wave) attempted an
/// invariant-based drain that spun until the glue DMA pipeline and
/// PIO1.SM0 RX FIFO were all simultaneously empty. That proved
/// counter-productive: during OneROM's steady-state background
/// pipeline activity, CH0/CH1 are rarely simultaneously idle, and any
/// pushes issued during the drain flooded the pipeline with gap-level
/// addresses — so the first "post-stimulus" push the oracle saw was
/// typically still gap-level, not stim-level. Every case then reported
/// `resolved = 0x2000_B000` (the gap-level pin pattern) regardless of
/// what stimulus was driven.
///
/// The H3 fix (2026-04-17) replaces the invariant-based drain with a
/// short fixed gap drive and switches the evaluator to match pushes
/// by **stim pin pattern** rather than by "first push after window
/// start". Gap pushes that slip into the pipeline after stimulus are
/// now skipped by the scan, so a small gap is enough to guarantee
/// PIO sees a clean CS-high state before stimulus.
///
/// Empirical tuning: 12 sysclks is enough to let one in-flight gap
/// push traverse CH0 (4 cycles) + CH1 (4+4 cycles) without blocking
/// the next stim push. Shorter gaps (≤ 8) leave the pipeline busy at
/// stim time so the stim push queues behind in-flight gap pushes and
/// the observed latency inflates far beyond the steady-state
/// envelope. Longer gaps (≥ 16) don't help further — PIO1's
/// background activity keeps pushing gap addresses during the gap
/// phase, so the pipeline never drains fully.
const GAP_CYCLES: u32 = 12;

/// Cycle budget per case. Must exceed the high end of `ENVELOPE_CYCLES`
/// by enough slack that transient drain residue on a correct serve
/// doesn't hit the timeout before the envelope check has a chance to
/// classify it. Current envelope is 15..=45; 60 gives ~1.3× slack over
/// the cap and is still well below any observed-in-practice latency.
const PER_CASE_TIMEOUT: u32 = 60;

/// Minimum sysclks before a CH1 byte-push could POSSIBLY have propagated
/// to PIO2's output pads. Derived from the glue DMA pipeline depth:
/// CH0-read (`DMA_READ_CYCLES`) + CH1-read/write (`DMA_WRITE_CYCLES`)
/// = 4 + 4 = 8 sysclks from CS-low to a fresh byte reaching PIO2.TX0.
///
/// This is a STRICT FLOOR: before this cycle, any observed stable run
/// is definitionally pipeline residue from a prior case whose
/// `data_byte` happens to match for `MIN_STABLE_CYCLES` consecutive
/// observations. PIO2 SM1's `OUT PINS, 8` shift adds additional
/// propagation cycles on top, but those land inside the steady-state
/// envelope (`ENVELOPE_CYCLES`) and are correctly classified by the
/// envelope check — no benefit to raising the gate past the DMA
/// floor, and doing so would risk false `NoStableByte` verdicts on
/// correct serves near the envelope boundary.
///
/// Gating stability on `obs.cycle >= MIN_FRESH_ARRIVAL_CYCLE` (alongside
/// the Phase D.2 `baseline_pushes` edge gate) rejects the pipeline-
/// propagation-lag false-positive class. See Phase D.2b in
/// `wrk_journals/2026.04.17 - JRN - OneROM Serving Oracle Fix Wave.md`
/// for the A7 (case 9) live-sweep trace that motivated this gate.
const MIN_FRESH_ARRIVAL_CYCLE: u64 = (DMA_READ_CYCLES as u64) + (DMA_WRITE_CYCLES as u64);

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One address stimulus for the sweep.
///
/// Stage 2 of the fixture-generalization HLD collapses the legacy
/// permutation/raw modes into a single `pin_pattern: u64` that's the
/// authoritative GPIO bitmask the fixture's stim composition will OR
/// onto the deasserted-high level. Build via [`Case::from_addr`] (which
/// permutes a chip-internal address through `spec.addr_pins`) or
/// [`Case::from_raw`] (which takes a literal pattern).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Case {
    pub label: &'static str,
    /// GPIO pin pattern for the case stimulus, as a u64 so future
    /// fire-32-a fixtures with GPIOs ≥ 32 can be expressed without
    /// re-typing. Stage 2 keeps fire-24-a working unchanged (max GPIO
    /// 23 → fits in u32); Stage 3 widens the bus interface so this
    /// value can flow through to the GPIO atomic without an assertion.
    pub pin_pattern: u64,
}

impl Case {
    /// Build a case from a chip-internal address by permuting bit `i`
    /// of `addr` onto `spec.addr_pins[i]`. The resulting `pin_pattern`
    /// captures only the address bits — chip-select levels are added
    /// later by the oracle's stim composition (HLD §4.4).
    pub fn from_addr(label: &'static str, addr: u32, spec: &FixtureSpec) -> Self {
        let mut pat = 0u64;
        for (bit, &gpio) in spec.addr_pins.iter().enumerate() {
            if (addr >> bit) & 1 != 0 {
                pat |= 1u64 << gpio;
            }
        }
        Case {
            label,
            pin_pattern: pat,
        }
    }

    /// Build a case that drives `pin_pattern` directly. Caller is
    /// responsible for the pin map; used by larger-than-default fixtures
    /// (e.g. the 256 KiB SeaBIOS validator) that enumerate every 16-bit
    /// GPIO pattern verbatim.
    pub const fn from_raw(label: &'static str, pin_pattern: u64) -> Self {
        Case {
            label,
            pin_pattern,
        }
    }
}

/// Default address-case set — walking-1s over A0..A10 plus three
/// high-coverage patterns. Translated through `spec.addr_pins`.
///
/// Structure (15 entries):
/// - 1 baseline case (`0x1800` — all low bits clear, A11=A12=1 only).
/// - 11 walking-1s cases: one per A0..A10 bit, labelled `walk1 A<n>`.
/// - 3 pattern cases: `0x1AAA`, `0x1D55`, `0x1FFF`.
///
/// The address values still carry the legacy fire-24-a A11=A12=1
/// invariant (CS2/CS3 share GPIOs with A11/A12 on that pin map, so
/// keeping those high is what deselects the chip). `Case::from_addr`
/// translates each address into a pin pattern through the supplied
/// `FixtureSpec`, so the same catalogue works on any fixture whose
/// `addr_pins` covers ≥ 13 lines.
pub fn default_cases(spec: &FixtureSpec) -> Vec<Case> {
    const TABLE: &[(&str, u32)] = &[
        ("walk1 baseline", 0x1800),
        ("walk1 A0", 0x1801),
        ("walk1 A1", 0x1802),
        ("walk1 A2", 0x1804),
        ("walk1 A3", 0x1808),
        ("walk1 A4", 0x1810),
        ("walk1 A5", 0x1820),
        ("walk1 A6", 0x1840),
        ("walk1 A7", 0x1880),
        ("walk1 A8", 0x1900),
        ("walk1 A9", 0x1A00),
        ("walk1 A10", 0x1C00),
        ("pattern AAA", 0x1AAA),
        ("pattern D55", 0x1D55),
        ("pattern FFF", 0x1FFF),
    ];
    TABLE
        .iter()
        .map(|(label, addr)| Case::from_addr(label, *addr, spec))
        .collect()
}

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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
    /// `bus.read32(CH1.READ_ADDR, 0)` sampled this cycle.
    pub resolved_addr: u32,
    /// Byte currently exposed on D0..D7 (`(gpio_in >> spec.data_pins[0]) & 0xFF` —
    /// the data pins are contiguous on supported fixtures).
    pub data_byte: u8,
    /// PIO2's output-enable mask over D0..D7.
    pub pio2_pad_oe_data: u8,
}

/// Errors surfaced by [`ServingOracle::new_at_sync`]. Stage 2 widens
/// the constructor to consume a [`FixtureSpec`] derived from the same
/// flash image; on parse failure the typed error propagates here so
/// drivers can render a useful diagnostic.
#[derive(Debug)]
pub enum OracleNewError {
    /// Underlying [`FixtureSpec::from_flash`] failed.
    FixtureParse(FixtureError),
}

impl std::fmt::Display for OracleNewError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::FixtureParse(e) => write!(f, "ServingOracle::new_at_sync: {e}"),
        }
    }
}

impl std::error::Error for OracleNewError {}

/// Oracle state. Owns the per-fixture pin map + capacity and the SRAM
/// shadow captured at sync.
pub struct ServingOracle {
    spec: FixtureSpec,
    rom_shadow: Box<[u8]>,
    results: Vec<CaseResult>,
    /// Tracks whether we've driven the `init` seed yet. `run_case`
    /// does so exactly once — on the first call — to produce a clean
    /// high-to-low edge on the gate CS for case 1.
    seed_done: bool,
}

impl ServingOracle {
    /// Capture the ROM-table shadow. Called once after the harness confirms
    /// OneROM has reached steady state (`onerom_sync::is_synced` true).
    ///
    /// Lifts the shadow from the loaded **flash image** rather than SRAM,
    /// because at the current sync criterion (`PIO1.CTRL.SM_ENABLE &&
    /// PIO2.CTRL.SM_ENABLE`) the DMA-driven `preload_rom_image` copy has
    /// NOT yet populated `rom_table` in SRAM. Observed: even stepping
    /// 1 M cycles past sync leaves SRAM[0x20000000..+spec.shadow_size]
    /// entirely zero on our emulator (the preload DMA program is not
    /// executed). See `wrk_journals/2026.04.15 - JRN - OneROM Shadow
    /// Source Investigation.md` for the evidence trail.
    ///
    /// Stage 2: takes a `FixtureSpec` (parsed from the same `flash`
    /// image by the caller) so the per-fixture shadow size is honoured.
    /// Reads `rom_set_index` from the SRAM-resident `sdrr_runtime_info`
    /// (the one field the firmware does populate by sync), then walks
    /// the flash structs to locate the selected set's `data` pointer
    /// and copies its `spec.shadow_size` bytes into the shadow.
    ///
    /// Fall-backs: on any parse failure (bad magic, out-of-range pointer,
    /// index out of range) the shadow is zero-filled. The binary-level
    /// "shadow-integrity tripwire" reports `unique bytes == 1` and warns,
    /// so a silently-wrong shadow still surfaces to the operator.
    pub fn new_at_sync(bus: &mut Bus, spec: FixtureSpec, flash: &[u8]) -> Self {
        // Offset of `sdrr_runtime_info.rom_set_index` within SRAM:
        // `_sdrr_runtime_info_location = _end - 8192 = 0x20080000` (per
        // `sdrr/link/common.ld`); `rom_set_index` is at runtime-info
        // offset +6 (see `sdrr_runtime_info_t` in
        // `sdrr/include/config_base.h`).
        const RUNTIME_INFO_SRAM_OFF: u32 = 0x0008_0000;
        const ROM_SET_INDEX_OFFSET: u32 = 6;
        let rom_set_index = bus
            .memory
            .sram_read8(RUNTIME_INFO_SRAM_OFF + ROM_SET_INDEX_OFFSET);

        let shadow = lift_shadow_from_flash(flash, rom_set_index, &spec)
            .unwrap_or_else(|| vec![0u8; spec.shadow_size].into_boxed_slice());

        Self {
            spec,
            rom_shadow: shadow,
            results: Vec::new(),
            seed_done: false,
        }
    }

    /// Test-only constructor that skips the emulator-side SRAM capture
    /// and accepts a caller-provided shadow. Used by
    /// `format_report_has_required_sections` (and future trace-free
    /// report tests) so the formatter can be exercised without spinning
    /// up an emulator just to seed SRAM.
    pub fn new_with_shadow(spec: FixtureSpec, shadow: Box<[u8]>) -> Self {
        debug_assert_eq!(
            shadow.len(),
            spec.shadow_size,
            "new_with_shadow: shadow length must match spec.shadow_size"
        );
        Self {
            spec,
            rom_shadow: shadow,
            results: Vec::new(),
            seed_done: false,
        }
    }

    /// Test-only push to seed the results vector for report tests.
    /// Not wired through `run_case`'s envelope post-processing: the
    /// caller is responsible for passing already-post-processed results
    /// when that matters.
    #[cfg(test)]
    pub(crate) fn push_result_for_test(&mut self, r: CaseResult) {
        self.results.push(r);
    }

    /// Drive one case end-to-end. Steps the emulator and pumps the glue
    /// DMA; records per-cycle observations; runs the verdict evaluator.
    ///
    /// State machine per HLD §4.3:
    /// - `init` (first call only): drive the gate CS + every
    ///   deasserted-high CS pin high, with addr=0, for `SEED_CYCLES`.
    ///   Guarantees the next stim produces a clean high→low edge on the
    ///   gate CS regardless of what state Stage F's external-input mask
    ///   left behind.
    /// - `idle → cs_assert`: apply the case stimulus (gate CS low,
    ///   deasserted-high CS pins high, A-bus = `case.pin_pattern`).
    ///   Record `cs_low_cycle` and `ch1_pushes_before`.
    /// - `cs_assert → wait_push → wait_stable → record → cs_release → idle`:
    ///   the per-tick loop builds up an `Observation` vector for at
    ///   most `PER_CASE_TIMEOUT` cycles. At the end (or on early stable-
    ///   byte detection), the trace is fed to [`evaluate_case_trace`].
    pub fn run_case(&mut self, emu: &mut Emulator, glue: &mut GlueDma, case: Case) -> &CaseResult {
        // External-input mask covers gate CS (cs1 today on fire-24-a),
        // every deasserted-high CS pin, every asserted-low pin, and all
        // address pins. Data pins are PIO-driven; never mask them.
        // fire-32-a (Stage 3) will widen the bus to u64; today we assert
        // that no fixture uses GPIOs >= 32 to make the Stage 3 follow-up
        // loud.
        let ext_mask: u64 = self.compose_ext_mask();
        debug_assert!(
            ext_mask >> 32 == 0,
            "ext_mask uses GPIOs >= 32; widen Bus interface for fire-32-a (Stage 3)"
        );
        emu.bus.gpio_external_mask = ext_mask as u32;

        // 1. init seed (first call only). Drive the gate CS + every
        //    deasserted-high CS pin high.
        if !self.seed_done {
            let seed_level: u64 = self.compose_seed_level();
            debug_assert!(seed_level >> 32 == 0, "seed_level uses GPIOs >= 32");
            emu.bus
                .gpio_external_in
                .store(seed_level as u32, Ordering::Relaxed);
            self.tick_cycles(emu, glue, SEED_CYCLES);
            self.seed_done = true;
        }

        // 2. Short fixed gap drive (H3 fix, 2026-04-17): put PIO into a
        // known CS-high state before applying stimulus. Earlier versions
        // (H2 fix) attempted an invariant-based drain until the DMA
        // pipeline was idle, but OneROM's background pipeline activity
        // meant the drain rarely terminated early — and pushes issued
        // during the drain flooded the pipeline with gap-level addresses.
        //
        // H3 relies on the evaluator's stim-pattern matching instead of
        // "first push after window start": gap pushes that land inside
        // the observation window are now skipped by the scan, so a
        // short fixed-duration gap is sufficient to seed CS-high before
        // stimulus.
        let gap_level: u64 = self.compose_gap_level();
        debug_assert!(gap_level >> 32 == 0, "gap_level uses GPIOs >= 32");
        emu.bus
            .gpio_external_in
            .store(gap_level as u32, Ordering::Relaxed);
        self.tick_cycles(emu, glue, GAP_CYCLES);

        // 3. cs_assert: apply the case stimulus.
        let stim_level: u64 = self.compose_stim_level(case.pin_pattern);
        debug_assert!(stim_level >> 32 == 0, "stim_level uses GPIOs >= 32");
        let expected_pin_bits: u16 = (stim_level & 0xFFFF) as u16;
        emu.bus
            .gpio_external_in
            .store(stim_level as u32, Ordering::Relaxed);

        // Snapshot the push counter *before* stimulus-time ticks.
        // The observation loop records ch1_pushes as a delta relative
        // to this snapshot; the evaluator then uses the delta to detect
        // per-cycle push edges (for skipping gap pushes that slip in
        // during the observation window).
        let pushes_before = glue.ch1_pushes();

        // Data-pin base — the data pins are contiguous on every supported
        // fixture (fire-24-a: GPIO 16..23; fire-32-a: GPIO 0..7). Use
        // data_pins[0] as the shift offset.
        let data_base = self.spec.data_pins[0];

        // 4. wait_push → wait_stable: tick up to PER_CASE_TIMEOUT cycles,
        //    recording an Observation per cycle.
        let mut trace: Vec<Observation> = Vec::with_capacity(PER_CASE_TIMEOUT as usize);
        for c in 0..PER_CASE_TIMEOUT {
            emu.run(1).expect("Serial run is infallible");
            glue.tick(&mut emu.bus);

            // Plain subtraction — `glue.ch1_pushes()` is monotonic on a
            // single GlueDma, so an underflow here is a true invariant
            // violation we want to surface, not silently mask.
            let pushes = glue.ch1_pushes() - pushes_before;
            // H1 fix: `resolved_addr` comes from the glue DMA's saved
            // `last_pushed_read_addr`, updated atomically with the push
            // counter in `tick_ch1`. Reading `CH1.READ_ADDR` MMIO here
            // raced CH0's subsequent writes — by the time the oracle
            // observed a push edge, CH0 may already have deposited the
            // NEXT address, so the MMIO value reported an address that
            // never produced the observed byte. See H1 in the Stage G
            // fix-wave brief (2026-04-17).
            let resolved = glue.last_pushed_read_addr();
            let data_byte =
                ((emu.bus.gpio_in.load(Ordering::Relaxed) >> data_base) & 0xFF) as u8;
            let pad_oe = ((emu.bus.pio[2].pad_oe >> data_base) & 0xFF) as u8;

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
            if let Some(result) = try_evaluate_conclusive(
                case,
                &self.rom_shadow,
                self.spec.shadow_size,
                expected_pin_bits,
                &trace,
            ) {
                // Leave the bus in gap-level state for the next case.
                emu.bus
                    .gpio_external_in
                    .store(gap_level as u32, Ordering::Relaxed);

                self.results.push(apply_envelope(result));
                return self.results.last().unwrap();
            }
        }

        // 5. Budget exhausted — run the evaluator one last time; it'll
        //    report NoResolve / NoStableByte based on where the state
        //    machine stopped.
        let result =
            evaluate_case_trace(case, &self.rom_shadow, self.spec.shadow_size, expected_pin_bits, &trace);

        emu.bus
            .gpio_external_in
            .store(gap_level as u32, Ordering::Relaxed);

        self.results.push(apply_envelope(result));
        self.results.last().unwrap()
    }

    /// Copy the flash-parsed shadow into emulator SRAM at [`SHADOW_BASE`],
    /// emulating what the firmware's `preload_rom_image` DMA would have
    /// done. Call exactly once, after [`ServingOracle::new_at_sync`] and
    /// before running cases — the glue DMA CH1 reads bytes from the bus
    /// at the PIO1-resolved address, and without this mirror SRAM is
    /// still zero-filled at sync, so every `observed_byte` collapses to
    /// 0x00.
    ///
    /// Uses alias-0 writes (`SHADOW_BASE = 0x2000_0000`). A future
    /// refactor must keep the destination on alias 0 — SRAM aliases
    /// 1..3 are XOR/SET/CLR-on-write, not plain stores, and would
    /// silently corrupt the populate.
    pub fn populate_sram_from_shadow(&self, bus: &mut Bus) {
        // fire-32-a's 524288-byte shadow lands in RP2350 SRAM with ~8 KiB
        // headroom before colliding with sdrr_runtime_info at
        // 0x2008_0000. Stage 3 will validate this empirically when the
        // wide-bus path lands.
        for offset in 0..self.spec.shadow_size {
            bus.write8(SHADOW_BASE + offset as u32, self.rom_shadow[offset], 0);
        }
    }

    /// Accessor for the full results vector.
    pub fn results(&self) -> &[CaseResult] {
        &self.results
    }

    /// Accessor for the ROM-table shadow. Used by the binary's
    /// shadow-integrity tripwire (the `unique bytes` diagnostic) so it
    /// checks the authoritative shadow the verdict evaluator will
    /// consult, rather than sampling SRAM — which is no longer the
    /// shadow source on this build.
    pub fn shadow(&self) -> &[u8] {
        &self.rom_shadow
    }

    /// Accessor for the per-fixture pin map + capacity. Useful for
    /// drivers that want to render the fixture label or check pin
    /// numbers in their own diagnostics.
    pub fn spec(&self) -> &FixtureSpec {
        &self.spec
    }

    /// Full report formatter (HLD §5 + §5.4).
    ///
    /// Sections: header (sys_clk_hz + shadow stats + case count),
    /// per-case table, summary (pass/fail counts + latency stats +
    /// ROM speed class), and the emulator-bounded caveat.
    ///
    /// If `sys_clk_hz == 0` (PLL not settled at sync — see HLD §5.2),
    /// prints an `UNAVAILABLE` marker in the header and omits all ns
    /// columns from the table and the summary.
    pub fn format_report(&self, sys_clk_hz: u32) -> String {
        let mut out = String::new();
        let ns_available = sys_clk_hz != 0;

        // --- Header -------------------------------------------------------
        let _ = writeln!(out, "OneROM Serving Oracle — Report");
        if ns_available {
            let mhz = sys_clk_hz as f64 / 1_000_000.0;
            let _ = writeln!(out, "sys_clk_hz: {} Hz ({:.3} MHz)", sys_clk_hz, mhz);
        } else {
            let _ = writeln!(out, "sys_clk_hz: UNAVAILABLE (PLL not settled at sync)");
        }
        let unique_shadow: std::collections::HashSet<u8> =
            self.rom_shadow.iter().copied().collect();
        let _ = writeln!(
            out,
            "shadow: 0x{:08X} + 0x{:04X} bytes, {} unique",
            SHADOW_BASE,
            self.spec.shadow_size,
            unique_shadow.len()
        );
        let _ = writeln!(out, "cases: {}", self.results.len());
        let _ = writeln!(out);

        // --- Per-case table ----------------------------------------------
        // Columns: idx, label, pattern, resolved, expected, observed, cycles,
        // [ns,] verdict. ns column is omitted when sys_clk_hz == 0.
        if ns_available {
            let _ = writeln!(
                out,
                " {:>5}  {:<20} {:<10} {:<10} {:<8} {:<8} {:>6} {:>6}  verdict",
                "idx", "label", "pattern", "resolved", "expected", "observed", "cycles", "ns"
            );
        } else {
            let _ = writeln!(
                out,
                " {:>5}  {:<20} {:<10} {:<10} {:<8} {:<8} {:>6}  verdict",
                "idx", "label", "pattern", "resolved", "expected", "observed", "cycles"
            );
        }

        let total = self.results.len();
        for (i, r) in self.results.iter().enumerate() {
            let idx = format!("{}/{}", i + 1, total);
            let pattern = format!("0x{:08X}", r.case.pin_pattern as u32);
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
            let verdict = format_verdict_full(&r.verdict);

            if ns_available {
                let ns = r
                    .latency_cycles
                    .map(|c| format!("{}", cycles_to_ns(c, sys_clk_hz)))
                    .unwrap_or_else(|| "—".to_string());
                let _ = writeln!(
                    out,
                    " {:>5}  {:<20} {:<10} {:<10} {:<8} {:<8} {:>6} {:>6}  {}",
                    idx, r.case.label, pattern, resolved, expected, observed, cycles, ns, verdict
                );
            } else {
                let _ = writeln!(
                    out,
                    " {:>5}  {:<20} {:<10} {:<10} {:<8} {:<8} {:>6}  {}",
                    idx, r.case.label, pattern, resolved, expected, observed, cycles, verdict
                );
            }
        }
        let _ = writeln!(out);

        // --- Summary -----------------------------------------------------
        let mut pass = 0usize;
        let mut wrong_byte = 0usize;
        let mut no_resolve = 0usize;
        let mut no_stable = 0usize;
        let mut out_of_range = 0usize;
        let mut latency_oor = 0usize;
        let mut pass_latencies: Vec<u32> = Vec::new();
        for r in &self.results {
            match r.verdict {
                Verdict::Pass => {
                    pass += 1;
                    if let Some(c) = r.latency_cycles {
                        pass_latencies.push(c);
                    }
                }
                Verdict::WrongByte { .. } => wrong_byte += 1,
                Verdict::NoResolve => no_resolve += 1,
                Verdict::NoStableByte => no_stable += 1,
                Verdict::ResolvedAddrOutOfRange { .. } => out_of_range += 1,
                Verdict::LatencyOutOfEnvelope { .. } => latency_oor += 1,
            }
        }
        let fail = total - pass;

        let _ = writeln!(out, "Summary:");
        let _ = writeln!(out, "  {} cases total", total);
        let _ = writeln!(out, "  {} PASS", pass);
        let _ = writeln!(
            out,
            "  {} FAIL  ({} wrong-byte, {} no-resolve, {} no-stable-byte, {} addr-out-of-range, {} latency-out-of-envelope)",
            fail, wrong_byte, no_resolve, no_stable, out_of_range, latency_oor
        );

        if pass_latencies.is_empty() {
            let _ = writeln!(
                out,
                "  latency stats: — no Pass cases, latency stats unavailable"
            );
        } else {
            let min = *pass_latencies.iter().min().unwrap();
            let max = *pass_latencies.iter().max().unwrap();
            let sum: u32 = pass_latencies.iter().sum();
            let mean = sum / pass_latencies.len() as u32;
            if ns_available {
                let min_ns = cycles_to_ns(min, sys_clk_hz);
                let max_ns = cycles_to_ns(max, sys_clk_hz);
                let mean_ns = cycles_to_ns(mean, sys_clk_hz);
                let _ = writeln!(
                    out,
                    "  latency stats (Pass cases only): min={} max={} mean={} cycles ({} ns / {} ns / {} ns)",
                    min, max, mean, min_ns, max_ns, mean_ns
                );
                let _ = writeln!(
                    out,
                    "  ROM speed class: {} (mean={} ns)",
                    rom_speed_class(mean_ns),
                    mean_ns
                );
            } else {
                let _ = writeln!(
                    out,
                    "  latency stats (Pass cases only): min={} max={} mean={} cycles (ns unavailable)",
                    min, max, mean
                );
                let _ = writeln!(out, "  ROM speed class: unavailable (sys_clk_hz == 0)");
            }
        }

        let _ = writeln!(out);

        // --- Emulator-bounded caveat (HLD §5.4, verbatim) ----------------
        let _ = writeln!(
            out,
            "  Latency measured against the emulator's glue DMA + PIO model"
        );
        let _ = writeln!(
            out,
            "  (4+4 cycle DMA latency; PIO timing per picoem-common::pio)."
        );
        let _ = writeln!(
            out,
            "  Silicon-calibrated timing is a future pass via the silicon oracle"
        );
        let _ = writeln!(out, "  rig (see silicon_cycle_oracle_rp2350).");

        out
    }

    /// Advance `emu` by `n` cycles, pumping the glue DMA each cycle.
    /// Used for the seed and gap phases, which don't need to record
    /// observations.
    fn tick_cycles(&self, emu: &mut Emulator, glue: &mut GlueDma, n: u32) {
        for _ in 0..n {
            emu.run(1).expect("Serial run is infallible");
            glue.tick(&mut emu.bus);
        }
    }

    // ---------------------------------------------------------------------
    // Stim/gap level composition (HLD §4.4)
    //
    // Every output is a u64 so fire-32-a (max GPIO 47) can be expressed
    // without re-typing. The bus interface is u32 today; `run_case`
    // downcasts at the boundary with a `debug_assert!` that the high
    // 32 bits are zero. Stage 3 widens the bus.
    // ---------------------------------------------------------------------

    /// Combined external-input mask covering the gate CS, every
    /// deasserted-high CS pin, every asserted-low pin, and every
    /// address pin. Data pins are PIO-driven and excluded.
    fn compose_ext_mask(&self) -> u64 {
        let mut mask: u64 = 1u64 << self.spec.cs1;
        for &p in &self.spec.deasserted_high_during_read {
            mask |= 1u64 << p;
        }
        for &p in &self.spec.asserted_low_during_read {
            mask |= 1u64 << p;
        }
        for &p in &self.spec.addr_pins {
            mask |= 1u64 << p;
        }
        mask
    }

    /// Init seed level. Same shape as the gap level: gate CS high +
    /// every deasserted-high CS pin high. (Asserted-low pins stay LOW
    /// — they're driven low during reads, so the init seed leaves them
    /// in their inactive state which is "not driven" / 0.)
    fn compose_seed_level(&self) -> u64 {
        let mut level: u64 = 1u64 << self.spec.cs1;
        for &p in &self.spec.deasserted_high_during_read {
            level |= 1u64 << p;
        }
        level
    }

    /// Gap-level (CS-high, addr=0, asserted-low pins LEFT high so the
    /// chip is fully deselected during the inter-case quiet period).
    fn compose_gap_level(&self) -> u64 {
        let mut level: u64 = 1u64 << self.spec.cs1;
        for &p in &self.spec.deasserted_high_during_read {
            level |= 1u64 << p;
        }
        for &p in &self.spec.asserted_low_during_read {
            level |= 1u64 << p;
        }
        level
    }

    /// Stim-level for a case: deasserted-high CS pins high, asserted-low
    /// pins LOW (the chip is now selected for reading), gate CS LOW (the
    /// case pattern doesn't include it — pin_pattern is address bits
    /// only), and the case's `pin_pattern` ORed in for the address bus.
    /// Note that the gate CS staying LOW is the desired behaviour: the
    /// stim composition does NOT set bit `cs1`.
    fn compose_stim_level(&self, case_pattern: u64) -> u64 {
        let mut level: u64 = 0;
        for &p in &self.spec.deasserted_high_during_read {
            level |= 1u64 << p;
        }
        // Asserted-low pins stay LOW during stim (they're the active-
        // assertion pins for reads — driving them high would deselect
        // the chip). They're already 0 in `level`.
        level | case_pattern
    }
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
    shadow: &[u8],
    shadow_size: usize,
    expected_pin_bits: u16,
    trace: &[Observation],
) -> Option<CaseResult> {
    let result = evaluate_case_trace(case, shadow, shadow_size, expected_pin_bits, trace);
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
/// The `expected_pin_bits` argument is the low-16 of the case's stim
/// level — the pin pattern PIO1 will latch and push into CH1.READ_ADDR
/// when the stimulus reaches the DUT. The evaluator uses it to
/// distinguish **stim-matching pushes** (the ones this case cares
/// about) from gap-level pushes that leak through the pipeline
/// (`resolved = 0x2000_B000`) or other background activity. Only
/// stim-matching pushes transition `WaitPush → WaitStable`.
///
/// This is the testable core of [`ServingOracle::run_case`]: the unit
/// tests drive it with hand-crafted traces to exercise every verdict
/// variant without an emulator in the loop.
pub(crate) fn evaluate_case_trace(
    case: Case,
    shadow: &[u8],
    shadow_size: usize,
    expected_pin_bits: u16,
    trace: &[Observation],
) -> CaseResult {
    let mut state = EvalState::WaitPush;
    // Per-cycle edge tracker: when `obs.ch1_pushes` increases vs. the
    // prior observation, a new push has landed this cycle. We start at
    // 0 because `run_case` stores ch1_pushes as a delta relative to the
    // pre-window `pushes_before` snapshot, so the first tick with a
    // push already inside the window (delta=1) correctly registers as
    // an edge.
    //
    // The H3 fix (2026-04-17) replaces the earlier "first push after
    // window start" rule with a per-edge scan: gap pushes that slip
    // into the window during background pipeline activity now increment
    // ch1_pushes but are filtered out by the stim-pattern match below
    // (gap-level resolved = 0x2000_B000 rarely collides with a case's
    // stimulus low-16). Only the push whose `resolved_addr` matches
    // this case's `expected_pin_bits` transitions to `WaitStable`.
    let mut prev_pushes: u32 = 0;

    for (i, obs) in trace.iter().enumerate() {
        match &mut state {
            EvalState::WaitPush => {
                let new_push = obs.ch1_pushes > prev_pushes;
                prev_pushes = obs.ch1_pushes;
                if new_push {
                    let resolved = obs.resolved_addr;
                    let hi16 = (resolved >> 16) as u16;
                    let low16 = (resolved & 0xFFFF) as u16;

                    // Gap / non-stim push — scan past it. `hi16 == 0x2000` is architecturally guaranteed:
                    // `setup_onerom.pio` PIO1 SM0 composes pushes `IN X, 16; IN PINS, 16` with `X = ROM_BASE >> 16`.
                    if hi16 != 0x2000 || low16 != expected_pin_bits {
                        continue;
                    }

                    // Stim-matching push. The hi16==0x2000 check above
                    // already guarantees in-range for shadow_size ==
                    // 0x10000 (which spans the full u16 low-half), but
                    // we still validate the bound explicitly so future
                    // resizes (e.g. fire-32-a's 512 KiB shadow) can
                    // surface out-of-range pushes.
                    if !(SHADOW_BASE..SHADOW_BASE + shadow_size as u32).contains(&resolved) {
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
                // the push cycle (HLD §4.4 — the residue-rejection rule)
                // AND the observation cycle is past the glue DMA pipeline
                // floor (Phase D.2b — `MIN_FRESH_ARRIVAL_CYCLE`). The
                // floor rules out stable runs that form before a fresh
                // byte could possibly have propagated, which would
                // otherwise latch stale residue that matches for
                // MIN_STABLE_CYCLES by coincidence.
                let after_push = obs.cycle > *push_cycle;
                let drives_all = obs.pio2_pad_oe_data == 0xFF;
                let past_pipeline = obs.cycle >= MIN_FRESH_ARRIVAL_CYCLE;

                if after_push && drives_all && past_pipeline {
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
                    // Break the stable run — either pad_oe dropped, we're
                    // still at/before push_cycle, or we're below the
                    // fresh-arrival cycle floor.
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
        EvalState::WaitStable {
            resolved_addr,
            expected,
            ..
        } => CaseResult {
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
// Envelope post-processing
// ---------------------------------------------------------------------------

/// Applies the emulator-bounded latency envelope (`ENVELOPE_CYCLES`)
/// to a [`CaseResult`]. If the verdict is [`Verdict::Pass`] and
/// `latency_cycles` is out of envelope, rewrites the verdict to
/// [`Verdict::LatencyOutOfEnvelope`]. All other verdicts pass through
/// unchanged. Silicon-tracked timing remains a future pass via
/// `silicon_cycle_oracle_rp2350`.
///
/// Separating this from [`evaluate_case_trace`] keeps the evaluator pure
/// over the trace (no policy) and lets unit tests exercise the envelope
/// rule without synthesizing traces. Wired into
/// [`ServingOracle::run_case`] as the final post-processing step.
pub fn apply_envelope(result: CaseResult) -> CaseResult {
    match result.verdict {
        Verdict::Pass => match result.latency_cycles {
            Some(cycles) if !ENVELOPE_CYCLES.contains(&cycles) => CaseResult {
                verdict: Verdict::LatencyOutOfEnvelope { cycles },
                ..result
            },
            _ => result,
        },
        _ => result,
    }
}

// ---------------------------------------------------------------------------
// Report helpers
// ---------------------------------------------------------------------------

/// Convert a cycle count to nanoseconds via the provided sysclk.
/// Panics are impossible because callers guard on `sys_clk_hz != 0`
/// before invoking. Integer-divide truncates toward zero; intentional —
/// ROM speed-class bands use inclusive `<=` upper bounds so truncation
/// doesn't mis-classify.
fn cycles_to_ns(cycles: u32, sys_clk_hz: u32) -> u64 {
    (cycles as u64) * 1_000_000_000 / (sys_clk_hz as u64)
}

/// ROM speed classification table from HLD §5.3. Label-only; never a
/// pass/fail criterion (the envelope check is).
fn rom_speed_class(ns: u64) -> &'static str {
    if ns <= 55 {
        "fast"
    } else if ns <= 70 {
        "standard fast"
    } else if ns <= 100 {
        "standard"
    } else if ns <= 120 {
        "slow standard"
    } else if ns <= 150 {
        "slow"
    } else {
        "very slow"
    }
}

/// Full verdict label for the report's per-case table. Longer than the
/// binary's compact `format_verdict_short` — the report can afford the
/// space and a reader looking at a table wants to see the parameters
/// (e.g. `LatencyOutOfEnvelope(5)`).
fn format_verdict_full(v: &Verdict) -> String {
    match v {
        Verdict::Pass => "Pass".to_string(),
        Verdict::WrongByte { expected, observed } => {
            format!("WrongByte(exp=0x{:02X}, obs=0x{:02X})", expected, observed)
        }
        Verdict::NoResolve => "NoResolve".to_string(),
        Verdict::NoStableByte => "NoStableByte".to_string(),
        Verdict::ResolvedAddrOutOfRange { addr } => {
            format!("AddrOOR(0x{:08X})", addr)
        }
        Verdict::LatencyOutOfEnvelope { cycles } => {
            format!("LatencyOutOfEnvelope({})", cycles)
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// The numbered-list comment style used inside this test module
// (`/// 1. foo bar...\n/// continuation...`) trips clippy's
// `doc_lazy_continuation` lint when the wrapped continuation lacks the
// 3-space indent under the bullet. The continuation is intentional
// prose, not list-continuation markdown — this module's docs render
// nowhere (it's a `#[cfg(test)]` block) so the lint is pure noise here.
#[cfg(test)]
#[allow(clippy::doc_lazy_continuation)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// Path to the fire-24-a CPU SeaBIOS fixture used by the in-crate
    /// tests as their canonical fixture source. Loaded via
    /// `CARGO_MANIFEST_DIR` so `cargo test` works regardless of cwd.
    fn fire24a_fixture_path() -> PathBuf {
        let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        p.push("fixtures");
        p.push("onerom-fire-24-a-rp2350-seabios-cpu.bin");
        p
    }

    /// Read the fire-24-a SeaBIOS-CPU fixture from disk.
    fn fire24a_fixture_bytes() -> Vec<u8> {
        let p = fire24a_fixture_path();
        std::fs::read(&p)
            .unwrap_or_else(|e| panic!("read {} failed: {}", p.display(), e))
    }

    /// Parse a `FixtureSpec` from the fire-24-a SeaBIOS-CPU fixture.
    fn fire24a_spec() -> FixtureSpec {
        let flash = fire24a_fixture_bytes();
        FixtureSpec::from_flash(&flash).expect("fire-24-a parse must succeed")
    }

    /// Shadow size for the fire-24-a fixture (64 KiB).
    fn fire24a_shadow_size() -> usize {
        fire24a_spec().shadow_size
    }

    /// Build a baseline 0x1800 case under the fire-24-a fixture (the
    /// common fixture-aware analogue of the legacy `mk_case()`).
    fn mk_case() -> Case {
        let spec = fire24a_spec();
        Case::from_addr("test", 0x1800, &spec)
    }

    /// Compute the low-16 stim-pattern bits PIO1 would push for
    /// `mk_case` under the fire-24-a fixture. Used as
    /// `expected_pin_bits` by the evaluator in tests that synthesise
    /// stim-matching pushes.
    fn mk_case_pin_bits() -> u16 {
        let spec = fire24a_spec();
        // The stim composition is gate CS LOW + deasserted-high CS pins
        // HIGH + case pin_pattern ORed in. Compute it here so tests
        // don't depend on `compose_stim_level` being public.
        let mut level: u64 = 0;
        for &p in &spec.deasserted_high_during_read {
            level |= 1u64 << p;
        }
        let case_pat = mk_case().pin_pattern;
        let total = level | case_pat;
        (total & 0xFFFF) as u16
    }

    /// Build a `resolved_addr` that matches `mk_case()`'s stim-pattern.
    /// Since the pin-bits (u16) occupy the low-16 of `resolved_addr`,
    /// and the stim-pattern uniquely identifies the case, every
    /// `resolved_addr` for `mk_case()` equals `0x2000_0000 | mk_case_pin_bits()`.
    fn mk_case_resolved() -> u32 {
        SHADOW_BASE | (mk_case_pin_bits() as u32)
    }

    fn empty_shadow() -> Box<[u8]> {
        vec![0u8; fire24a_shadow_size()].into_boxed_slice()
    }

    /// Single load-bearing equivalence proof for Stage 2: the new
    /// `Case::from_addr` output must be byte-identical to the legacy
    /// `stimulus_level()` output for every entry in the historic
    /// 15-case default catalogue, against the fire-24-a fixture.
    ///
    /// The legacy `stimulus_level` body is reproduced here verbatim
    /// (TEMPORARY copy — Stage 2 is the only test that asserts this).
    /// The legacy production code itself was deleted; if this test
    /// ever fails, either the fire-24-a `addr_pins` parse drifted or
    /// `Case::from_addr`'s permutation logic regressed. Both are
    /// load-bearing for fire-24-a behaviour preservation.
    #[test]
    fn case_from_addr_matches_legacy_stimulus_level() {
        // The pre-Stage-2 ADDR_PINS map for fire-24-a — copied from
        // the deleted production constant. The FixtureSpec parser
        // rebuilds this from the firmware's `sdrr_pins_t` so we
        // assert equality against both pathways below.
        const LEGACY_ADDR_PINS: [u8; 13] = [7, 6, 5, 4, 3, 2, 1, 0, 10, 11, 14, 15, 12];

        // The legacy `stimulus_level` body, reproduced verbatim. CS1
        // stays low; CS2 (GPIO12) and CS3 (GPIO15) double as A12 and
        // A11 respectively under this pin map and are driven by the
        // A11=A12=1 case invariant.
        fn legacy_stimulus_level(addr_bits: u16) -> u32 {
            let mut level: u32 = 0;
            for (i, &pin) in LEGACY_ADDR_PINS.iter().enumerate() {
                if (addr_bits >> i) & 1 != 0 {
                    level |= 1u32 << pin;
                }
            }
            level
        }

        let spec = fire24a_spec();

        // Sanity: the parsed addr_pins matches the legacy literal.
        assert_eq!(
            spec.addr_pins,
            LEGACY_ADDR_PINS.to_vec(),
            "fire-24-a addr_pins parse drifted from the legacy literal"
        );

        const LEGACY_TABLE: &[(&str, u16)] = &[
            ("walk1 baseline", 0x1800),
            ("walk1 A0", 0x1801),
            ("walk1 A1", 0x1802),
            ("walk1 A2", 0x1804),
            ("walk1 A3", 0x1808),
            ("walk1 A4", 0x1810),
            ("walk1 A5", 0x1820),
            ("walk1 A6", 0x1840),
            ("walk1 A7", 0x1880),
            ("walk1 A8", 0x1900),
            ("walk1 A9", 0x1A00),
            ("walk1 A10", 0x1C00),
            ("pattern AAA", 0x1AAA),
            ("pattern D55", 0x1D55),
            ("pattern FFF", 0x1FFF),
        ];

        for (label, addr_bits) in LEGACY_TABLE {
            let new_case = Case::from_addr(label, *addr_bits as u32, &spec);
            let legacy_level = legacy_stimulus_level(*addr_bits);
            // Legacy `stimulus_level` returned the pin pattern for the
            // address bits only (no chip-select levels). `Case::from_addr`
            // produces the same — chip-select levels are added later by
            // the oracle's stim composition.
            assert_eq!(
                new_case.pin_pattern as u32, legacy_level,
                "case `{}` (addr_bits=0x{:04X}): from_addr=0x{:08X}, legacy=0x{:08X}",
                label, addr_bits, new_case.pin_pattern as u32, legacy_level
            );
        }
    }

    /// 2. Happy-path PASS: push at cycle 5, stable 0x42 from cycle 12 for 3 cycles.
    #[test]
    fn verdict_pass_when_byte_matches_shadow_after_push() {
        let case = mk_case();
        let pin_bits = mk_case_pin_bits();
        let mut shadow = empty_shadow();
        shadow[pin_bits as usize] = 0x42;
        let resolved = mk_case_resolved();

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

        let result = evaluate_case_trace(case, &shadow, fire24a_shadow_size(), pin_bits, &trace);
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
        let pin_bits = mk_case_pin_bits();
        let mut shadow = empty_shadow();
        shadow[pin_bits as usize] = 0x42;
        let resolved = mk_case_resolved();

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

        let result = evaluate_case_trace(case, &shadow, fire24a_shadow_size(), pin_bits, &trace);
        assert_eq!(
            result.verdict,
            Verdict::WrongByte {
                expected: 0x42,
                observed: 0x00
            }
        );
        assert_eq!(result.latency_cycles, Some(12));
    }

    /// 4. Prior-case residue: data is already 0xAA at cycle 0, but the
    /// push only happens at cycle 5. Latency must anchor after the push
    /// + fresh-arrival floor, not at cycle 0.
    #[test]
    fn verdict_rejects_prior_case_residue() {
        let case = mk_case();
        let pin_bits = mk_case_pin_bits();
        let mut shadow = empty_shadow();
        shadow[pin_bits as usize] = 0xAA;
        let resolved = mk_case_resolved();

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

        let result = evaluate_case_trace(case, &shadow, fire24a_shadow_size(), pin_bits, &trace);
        assert_eq!(
            result.verdict,
            Verdict::Pass,
            "residue rejection should still PASS once anchored after push + floor"
        );
        assert_eq!(
            result.latency_cycles,
            Some(MIN_FRESH_ARRIVAL_CYCLE as u32),
            "latency must anchor after push AND after fresh-arrival floor"
        );
    }

    /// 4b. Gap-push rejection (H3 fix): a push whose `resolved` is the
    /// gap-level pattern is skipped as non-stim; verdict is `NoResolve`
    /// when no stim-matching push arrives.
    #[test]
    fn stability_rejects_stale_byte_without_fresh_push() {
        let case = mk_case();
        let pin_bits = mk_case_pin_bits();
        let shadow = empty_shadow();
        let resolved = SHADOW_BASE + 0xB000;
        assert_ne!(
            (resolved & 0xFFFF) as u16,
            pin_bits,
            "test precondition: gap resolve must differ from stim pin-bits",
        );

        let mut trace = Vec::new();
        for c in 0..(PER_CASE_TIMEOUT as u64) {
            trace.push(Observation {
                cycle: c,
                ch1_pushes: 1,
                resolved_addr: resolved,
                data_byte: 0x20,
                pio2_pad_oe_data: 0xFF,
            });
        }

        let result = evaluate_case_trace(case, &shadow, fire24a_shadow_size(), pin_bits, &trace);

        assert!(
            !matches!(result.verdict, Verdict::WrongByte { observed: 0x20, .. }),
            "stale byte must not surface as WrongByte(observed=0x20); got {:?}",
            result.verdict
        );
        assert_ne!(
            result.verdict,
            Verdict::Pass,
            "stale byte must not PASS (got observed={:?})",
            result.observed_byte
        );
        assert!(
            !matches!(result.verdict, Verdict::LatencyOutOfEnvelope { .. }),
            "stale byte must not surface as LatencyOutOfEnvelope; got {:?}",
            result.verdict
        );
        assert_eq!(
            result.verdict,
            Verdict::NoResolve,
            "gap-only pushes must resolve as NoResolve (no stim-matching push)"
        );
        assert!(result.observed_byte.is_none());
    }

    /// 4c. All-zero push-count trace: returns `NoResolve` via the
    /// `WaitPush` gate.
    #[test]
    fn stability_rejects_stale_byte_with_zero_pushes_throughout() {
        let case = mk_case();
        let pin_bits = mk_case_pin_bits();
        let shadow = empty_shadow();
        let resolved = SHADOW_BASE + 0xB000;

        let mut trace = Vec::new();
        for c in 0..(PER_CASE_TIMEOUT as u64) {
            trace.push(Observation {
                cycle: c,
                ch1_pushes: 0,
                resolved_addr: resolved,
                data_byte: 0x20,
                pio2_pad_oe_data: 0xFF,
            });
        }

        let result = evaluate_case_trace(case, &shadow, fire24a_shadow_size(), pin_bits, &trace);
        assert_eq!(result.verdict, Verdict::NoResolve);
        assert!(result.observed_byte.is_none());
    }

    /// 4d. Early-exit gate (H3): `try_evaluate_conclusive` must not
    /// declare a case conclusive while the trace-so-far contains no
    /// stim-matching push.
    #[test]
    fn try_evaluate_conclusive_requires_fresh_push_edge() {
        let case = mk_case();
        let pin_bits = mk_case_pin_bits();
        let shadow = empty_shadow();
        let resolved = SHADOW_BASE + 0xB000;

        let mut trace = Vec::new();
        for c in 0..10u64 {
            trace.push(Observation {
                cycle: c,
                ch1_pushes: 1,
                resolved_addr: resolved,
                data_byte: 0x20,
                pio2_pad_oe_data: 0xFF,
            });
        }
        assert!(
            try_evaluate_conclusive(case, &shadow, fire24a_shadow_size(), pin_bits, &trace).is_none(),
            "no stim-matching push → not conclusive"
        );
    }

    /// 4e. Fresh-arrival-cycle gate (Phase D.2b): even with a fresh push
    /// edge inside the window, stability declared before the glue DMA
    /// pipeline could possibly have delivered the new byte must be
    /// rejected.
    #[test]
    fn stability_rejects_stale_byte_under_min_fresh_arrival_cycle() {
        let case = mk_case();
        let pin_bits = mk_case_pin_bits();
        let shadow = empty_shadow();
        let resolved = mk_case_resolved();

        let mut trace = Vec::new();
        trace.push(Observation {
            cycle: 0,
            ch1_pushes: 0,
            resolved_addr: resolved,
            data_byte: 0x20,
            pio2_pad_oe_data: 0xFF,
        });
        for c in 1..(PER_CASE_TIMEOUT as u64) {
            trace.push(Observation {
                cycle: c,
                ch1_pushes: 1,
                resolved_addr: resolved,
                data_byte: 0x20,
                pio2_pad_oe_data: 0xFF,
            });
        }

        let result = evaluate_case_trace(case, &shadow, fire24a_shadow_size(), pin_bits, &trace);

        if let Some(cycles) = result.latency_cycles {
            assert!(
                (cycles as u64) >= MIN_FRESH_ARRIVAL_CYCLE,
                "stable run declared at cycles={} (< MIN_FRESH_ARRIVAL_CYCLE={}); \
                 verdict={:?}",
                cycles,
                MIN_FRESH_ARRIVAL_CYCLE,
                result.verdict
            );
        }
        assert_ne!(
            result.verdict,
            Verdict::Pass,
            "stale 0x20 must not PASS against expected 0x00"
        );
    }

    /// 5. Push with non-0x2000 hi16 is a non-stim push and skipped.
    #[test]
    fn verdict_non_stim_push_skipped_as_no_resolve() {
        let case = mk_case();
        let pin_bits = mk_case_pin_bits();
        let shadow = empty_shadow();
        let non_stim_addr = 0x2100_0000u32;

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
                resolved_addr: non_stim_addr,
                data_byte: 0,
                pio2_pad_oe_data: 0,
            },
        ];

        let result = evaluate_case_trace(case, &shadow, fire24a_shadow_size(), pin_bits, &trace);
        assert_eq!(
            result.verdict,
            Verdict::NoResolve,
            "non-stim push must be skipped, leaving verdict as NoResolve"
        );
        assert!(result.resolved_addr.is_none());
        assert!(result.expected_byte.is_none());
    }

    /// 6. Default-cases shape: 15 cases, each with the same shape under
    /// `from_addr` (label, computed pin_pattern). Validates length and
    /// distinct labels.
    #[test]
    fn default_cases_full_sweep_shape() {
        let spec = fire24a_spec();
        let cases = default_cases(&spec);
        assert_eq!(cases.len(), 15, "expected 15 cases");

        // Distinct labels.
        let mut seen_labels: std::collections::HashSet<&str> = std::collections::HashSet::new();
        for c in &cases {
            assert!(
                seen_labels.insert(c.label),
                "duplicate label `{}` in default_cases",
                c.label
            );
        }

        // Distinct pin patterns.
        let mut seen_patterns: std::collections::HashSet<u64> = std::collections::HashSet::new();
        for c in &cases {
            assert!(
                seen_patterns.insert(c.pin_pattern),
                "duplicate pin_pattern 0x{:X} in default_cases (label `{}`)",
                c.pin_pattern,
                c.label
            );
        }
    }

    // --- G.3 tests: envelope post-processing + report formatter ----------

    /// 7. Envelope pass-through.
    #[test]
    fn apply_envelope_passes_through_in_range_latency() {
        let case = mk_case();
        let in_range = *ENVELOPE_CYCLES.start() + 5;
        assert!(ENVELOPE_CYCLES.contains(&in_range));
        let result = CaseResult {
            case,
            resolved_addr: Some(SHADOW_BASE + 0x10),
            expected_byte: Some(0x42),
            observed_byte: Some(0x42),
            latency_cycles: Some(in_range),
            verdict: Verdict::Pass,
        };
        let out = apply_envelope(result);
        assert_eq!(out.verdict, Verdict::Pass);
        assert_eq!(out.latency_cycles, Some(in_range));
    }

    /// 8. Envelope rewrite.
    #[test]
    fn apply_envelope_rewrites_out_of_range_latency() {
        let case = mk_case();
        let out_of_range = *ENVELOPE_CYCLES.end() + 10;
        assert!(!ENVELOPE_CYCLES.contains(&out_of_range));
        let result = CaseResult {
            case,
            resolved_addr: Some(SHADOW_BASE + 0x10),
            expected_byte: Some(0x42),
            observed_byte: Some(0x42),
            latency_cycles: Some(out_of_range),
            verdict: Verdict::Pass,
        };
        let out = apply_envelope(result);
        assert_eq!(
            out.verdict,
            Verdict::LatencyOutOfEnvelope {
                cycles: out_of_range
            }
        );
        assert_eq!(out.latency_cycles, Some(out_of_range));
        assert_eq!(out.observed_byte, Some(0x42));
    }

    /// 9. Non-Pass verdicts are never rewritten by the envelope check.
    #[test]
    fn apply_envelope_leaves_non_pass_verdicts_alone() {
        let case = mk_case();

        let in_range = *ENVELOPE_CYCLES.start() + 5;
        let wrong_byte = CaseResult {
            case,
            resolved_addr: Some(SHADOW_BASE + 0x10),
            expected_byte: Some(0x42),
            observed_byte: Some(0xFF),
            latency_cycles: Some(in_range),
            verdict: Verdict::WrongByte {
                expected: 0x42,
                observed: 0xFF,
            },
        };
        let out = apply_envelope(wrong_byte);
        assert_eq!(
            out.verdict,
            Verdict::WrongByte {
                expected: 0x42,
                observed: 0xFF,
            }
        );

        let no_resolve = CaseResult {
            case,
            resolved_addr: None,
            expected_byte: None,
            observed_byte: None,
            latency_cycles: None,
            verdict: Verdict::NoResolve,
        };
        let out = apply_envelope(no_resolve);
        assert_eq!(out.verdict, Verdict::NoResolve);

        let bad_addr = 0x2100_0000u32;
        let addr_oor = CaseResult {
            case,
            resolved_addr: Some(bad_addr),
            expected_byte: None,
            observed_byte: None,
            latency_cycles: None,
            verdict: Verdict::ResolvedAddrOutOfRange { addr: bad_addr },
        };
        let out = apply_envelope(addr_oor);
        assert_eq!(
            out.verdict,
            Verdict::ResolvedAddrOutOfRange { addr: bad_addr }
        );
    }

    /// 10. `format_report` sections smoke-check.
    #[test]
    fn format_report_has_required_sections() {
        let spec = fire24a_spec();
        let mut shadow = empty_shadow();
        shadow[0x10] = 0x42;
        let mut oracle = ServingOracle::new_with_shadow(spec.clone(), shadow);

        let case = Case::from_addr("test", 0x1800, &spec);
        let in_range = *ENVELOPE_CYCLES.start() + 5;

        oracle.push_result_for_test(CaseResult {
            case,
            resolved_addr: Some(SHADOW_BASE + 0x10),
            expected_byte: Some(0x42),
            observed_byte: Some(0x42),
            latency_cycles: Some(in_range),
            verdict: Verdict::Pass,
        });

        oracle.push_result_for_test(CaseResult {
            case,
            resolved_addr: Some(SHADOW_BASE + 0x10),
            expected_byte: Some(0x42),
            observed_byte: Some(0xFF),
            latency_cycles: Some(in_range),
            verdict: Verdict::WrongByte {
                expected: 0x42,
                observed: 0xFF,
            },
        });

        oracle.push_result_for_test(CaseResult {
            case,
            resolved_addr: Some(SHADOW_BASE + 0x10),
            expected_byte: Some(0x42),
            observed_byte: Some(0x42),
            latency_cycles: Some(5),
            verdict: Verdict::LatencyOutOfEnvelope { cycles: 5 },
        });

        let report = oracle.format_report(150_000_000);

        assert!(
            report.contains("sys_clk_hz: 150000000"),
            "header missing sys_clk_hz: {}",
            report
        );
        assert!(
            report.contains("cases: 3"),
            "header missing cases count: {}",
            report
        );

        assert!(report.contains("Pass"), "missing Pass row: {}", report);
        assert!(
            report.contains("WrongByte"),
            "missing WrongByte row: {}",
            report
        );
        assert!(
            report.contains("LatencyOutOfEnvelope"),
            "missing LatencyOutOfEnvelope row: {}",
            report
        );

        assert!(
            report.contains("Summary:"),
            "missing Summary section: {}",
            report
        );
        assert!(
            report.contains("ROM speed class:"),
            "missing ROM speed class line: {}",
            report
        );

        assert!(
            report.contains("glue DMA + PIO model"),
            "missing emulator-bounded caveat: {}",
            report
        );
    }

    /// 11. Envelope wiring invariant: every `CaseResult` stored in
    /// `ServingOracle::results` must be a fixed point of `apply_envelope`.
    #[test]
    fn run_case_applies_envelope_before_pushing_result() {
        let spec = fire24a_spec();
        let cases = default_cases(&spec);
        let mut oracle = ServingOracle::new_with_shadow(spec, empty_shadow());

        let pre = CaseResult {
            case: cases[0],
            verdict: Verdict::Pass,
            latency_cycles: Some(5),
            resolved_addr: Some(SHADOW_BASE),
            expected_byte: Some(0),
            observed_byte: Some(0),
        };
        let post = apply_envelope(pre);
        assert_eq!(
            post.verdict,
            Verdict::LatencyOutOfEnvelope { cycles: 5 },
            "sanity: apply_envelope should rewrite Pass+5 to LatencyOOE(5)"
        );
        oracle.push_result_for_test(post);

        for r in oracle.results() {
            assert_eq!(
                *r,
                apply_envelope(*r),
                "stored result should be envelope-fixed-point: {:?}",
                r
            );
        }
    }

    // --- Phase C tests: `populate_sram_from_shadow` --------------------------

    use rp2350_emu::{Config, EmulatorBuilder};

    fn mk_emu() -> Emulator {
        EmulatorBuilder::new(Config::default())
            .build()
            .expect("Serial build is infallible")
    }

    /// Walking-1 SDRR offsets must appear in SRAM with the shadow's
    /// bytes after `populate_sram_from_shadow`.
    #[test]
    fn populate_sram_from_shadow_writes_bus_at_walking_1_offsets() {
        let mut emu = mk_emu();
        let spec = fire24a_spec();
        let mut shadow = empty_shadow();
        shadow[0x9010] = 0x08;
        shadow[0x9020] = 0x04;
        shadow[0x9040] = 0x02;
        shadow[0x9080] = 0x01;
        let oracle = ServingOracle::new_with_shadow(spec, shadow);

        oracle.populate_sram_from_shadow(&mut emu.bus);

        assert_eq!(emu.bus.read8(SHADOW_BASE + 0x9010, 0), 0x08);
        assert_eq!(emu.bus.read8(SHADOW_BASE + 0x9020, 0), 0x04);
        assert_eq!(emu.bus.read8(SHADOW_BASE + 0x9040, 0), 0x02);
        assert_eq!(emu.bus.read8(SHADOW_BASE + 0x9080, 0), 0x01);
        assert_ne!(
            emu.bus.read8(SHADOW_BASE + 0x9010, 0),
            0,
            "silent-revert guard: SHADOW_BASE+0x9010 must not read back 0x00"
        );
    }

    /// Off-by-one guard on the populate loop.
    #[test]
    fn populate_sram_from_shadow_covers_full_shadow_range() {
        let mut emu = mk_emu();
        let spec = fire24a_spec();
        let shadow_size = spec.shadow_size;
        let mut shadow = empty_shadow();
        shadow[0] = 0xAA;
        shadow[shadow_size - 1] = 0x55;
        let oracle = ServingOracle::new_with_shadow(spec, shadow);

        oracle.populate_sram_from_shadow(&mut emu.bus);

        assert_eq!(emu.bus.read8(SHADOW_BASE, 0), 0xAA);
        assert_eq!(emu.bus.read8(SHADOW_BASE + shadow_size as u32 - 1, 0), 0x55);
    }
}

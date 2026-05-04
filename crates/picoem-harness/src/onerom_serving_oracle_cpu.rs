//! OneROM serving oracle — CPU-serve mode byte-correctness + timing envelope.
//!
//! This is the on-core (CPU-serve) counterpart to [`crate::onerom_serving_oracle`].
//! The PIO oracle drives a 2-stage PIO + glue-DMA pipeline; this oracle
//! targets firmware builds that serve ROM from the CPU directly, with
//! core 0 sitting in a tight 5-instruction loop at
//! `0x1000_0926..=0x1000_0930` that:
//! 1. STRBs R1 → SIO_GPIO_OUT (drive data pins)
//! 2. LDRHs SIO_GPIO_IN → R0 (sample CS + addr pins)
//! 3. TSTs R0 against the CS1 mask in R9
//! 4. LDRBs shadow\[R0] → R1 (prefetch next byte from SRAM)
//! 5. BEQs back if CS1 stayed low
//!
//! Key differences from the PIO oracle:
//! - **No PIO**: PIO1.CTRL and PIO2.CTRL are both 0 for the whole run.
//!   Sync is detected via the CPU's PC entering the serve loop range.
//! - **No glue DMA**: the CPU reads SRAM directly and drives pins via
//!   SIO writes. There is no pipeline to pump and no [`GlueDma`] to
//!   thread through.
//! - **Direct pin observation**: the byte we care about is whatever the
//!   CPU has driven onto `gpio_in[16..23]` via SIO_GPIO_OUT + SIO_GPIO_OE.
//!   No "resolved address" intermediate — the CPU computes the address
//!   internally from the sampled pins and looks up the byte from its
//!   own SRAM shadow.
//! - **Envelope**: measured empirically; CPU-serve steady-state latency
//!   differs from PIO pipeline latency.
//!
//! Design mirrors [`crate::onerom_serving_oracle`] — we share [`Case`]
//! and [`SHADOW_BASE`] from there to keep the stimulus surface aligned.
//! Stage 2 of the fixture-generalization HLD migrates this oracle off
//! the legacy hardcoded pin map onto the per-fixture
//! [`crate::onerom_fixture::FixtureSpec`]; everything that touches
//! PIO/DMA is re-implemented here for CPU-serve semantics.

use std::fmt::Write as _;
use std::sync::atomic::Ordering;

use rp2350_emu::{Bus, Emulator};

use crate::onerom_fixture::{FixtureSpec, lift_shadow_from_flash};
use crate::onerom_serving_oracle::{Case, SHADOW_BASE};

// ---------------------------------------------------------------------------
// Public constants
// ---------------------------------------------------------------------------

/// Default case catalogue for the CPU oracle — wraps
/// [`crate::onerom_serving_oracle::default_cases`] verbatim. Stage 2
/// drops the legacy `CPU_DEFAULT_CASES` constant in favour of this
/// per-fixture function.
pub fn cpu_default_cases(spec: &FixtureSpec) -> Vec<Case> {
    crate::onerom_serving_oracle::default_cases(spec)
}

/// CPU serve-loop PC range (RP2350, CPU-mode fixture). The hot loop is
/// five instructions (one 32-bit TST.W plus four 16-bit halfwords),
/// spanning 10 bytes at `0x1000_0926..=0x1000_0930`. Empirically verified
/// via `onerom_cpu_probe` (5000-instruction PC histogram, 2026-04-17 run
/// of `test-sdrr-0-cpu.bin`): 100% of observed PCs land in this range,
/// with the 5 hot PCs distributing 28/14/14/14/28% — a balanced trace of
/// the 5-instruction inner loop.
///
/// If a future fixture shifts the serve loop, the `onerom_cpu_probe`
/// diagnostic binary will detect the new range and the constants here
/// are the single place to update.
pub const CPU_SERVE_LOOP_PC_LO: u32 = 0x1000_0926;
pub const CPU_SERVE_LOOP_PC_HI: u32 = 0x1000_0930;

/// Acceptable CS-low-to-stable-byte cycle envelope for CPU-serve mode.
///
/// Populated from the empirical wide-envelope run described in the HLD;
/// CPU latency is shorter than PIO pipeline latency because there's no
/// DMA pipeline to clock through — the CPU reads a pin, looks up a
/// byte, and drives it, all in one iteration of a 5-instruction loop.
///
/// This window is emulator-bounded (same caveat as the PIO oracle —
/// silicon-calibrated timing remains a future pass). A case outside
/// this window but correct byte-wise is reclassified as
/// [`Verdict::LatencyOutOfEnvelope`] rather than a true FAIL.
///
/// Floor widened from 9 to 7 after the image_sel helper unblocked the
/// `onerom_stress_cpu_rp2350` sweep: across 2045 legitimate serves, the
/// minimum observed was 7 cycles (3 cases, correct byte), matching the
/// theoretical best-path through the 5-instruction loop when CS is
/// already low on loop entry.
pub const CPU_ENVELOPE_CYCLES: std::ops::RangeInclusive<u32> = 7..=60;

// ---------------------------------------------------------------------------
// Internal constants
// ---------------------------------------------------------------------------

/// Minimum consecutive cycles the data byte must hold steady (with OEN
/// = 0xFF on pins 16..23) to declare the byte "stable".
const MIN_STABLE_CYCLES: usize = 4;

/// Cycles of gap-level (CS1/CS2/CS3 high, addr=0) we drive at the start
/// of each case to put the CPU back into the "deselected" state before
/// applying the case stimulus. Small — the CPU only needs enough cycles
/// to loop back around and observe CS1 high, exit the inner loop, clear
/// OEN, and park in the outer wait.
const GAP_CYCLES: u32 = 40;

/// Cycle budget per case. Must exceed the high end of
/// [`CPU_ENVELOPE_CYCLES`] by enough slack that a correct serve with
/// transient latency inflation doesn't spuriously hit the timeout.
const PER_CASE_TIMEOUT: u32 = 400;

/// Minimum tick (cycles elapsed since stimulus applied) at which the
/// stability counter may begin counting. The CPU serve loop is 5
/// Thumb instructions; on RP2350 one full iteration (including the
/// `STRB` that drives a fresh byte onto the pins) takes on the order
/// of 6 cycles. Before this floor, any stable run we observe is
/// definitionally the *previous* byte the CPU happened to be driving
/// — the CPU simply hasn't yet sampled the new pin state, looked up
/// the shadow, and issued the fresh STRB. Rejecting these pre-floor
/// runs is the CPU-oracle analogue of the PIO oracle's
/// `MIN_FRESH_ARRIVAL_CYCLE` gate; without it, every case locks onto
/// the stale 0x00 the CPU was driving before stimulus application.
const MIN_FRESH_ARRIVAL_CYCLE_CPU: u32 = 6;

/// Timeout after which, if no byte transition has been observed since
/// stimulus application, we trust whatever byte the CPU is holding
/// steady. This handles the legitimate "expected byte == 0x00" case:
/// the CPU performs its loop iteration and stores 0x00 (same as the
/// prior steady-state), so no transition is visible on the wire.
/// Without this fallback, those cases would time out as
/// `NoStableByte` despite the CPU serving correctly. The value must
/// comfortably exceed `MIN_FRESH_ARRIVAL_CYCLE_CPU` plus
/// `MIN_STABLE_CYCLES` so the fallback only fires after the CPU has
/// had multiple full loop iterations to act.
const ZERO_BYTE_TRUST_TIMEOUT_CPU: u32 = 40;

/// SIO GPIO_OE MMIO (RP2350 8-byte offsets).
const SIO_GPIO_OE_ADDR: u32 = 0xD000_0030;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Outcome for one case under the CPU-serve oracle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CpuVerdict {
    /// Byte observed on D0..D7 matched the SRAM shadow at the stimulus
    /// address, and the latency was within the documented envelope.
    Pass,
    /// Stable byte observed but it does not match the shadow.
    WrongByte { expected: u8, observed: u8 },
    /// The CPU never drove the data pins (OEN stayed 0 for the full
    /// per-case budget). Distinct from `NoStableByte` because it
    /// diagnoses "CPU is stuck / not serving" rather than "CPU is
    /// serving but output jitters".
    DataPinsNotDriven,
    /// CPU drove OEN but the data byte never held steady for
    /// [`MIN_STABLE_CYCLES`] consecutive cycles.
    NoStableByte,
    /// Stable byte matched the shadow but measured latency fell outside
    /// [`CPU_ENVELOPE_CYCLES`].
    LatencyOutOfEnvelope { cycles: u32 },
}

/// Per-case diagnostic result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuCaseResult {
    pub case: Case,
    pub expected_byte: Option<u8>,
    pub observed_byte: Option<u8>,
    pub latency_cycles: Option<u32>,
    pub verdict: CpuVerdict,
}

/// CPU-serve oracle state.
pub struct CpuServingOracle {
    spec: FixtureSpec,
    rom_shadow: Box<[u8]>,
    results: Vec<CpuCaseResult>,
}

impl CpuServingOracle {
    /// Capture the ROM-table shadow at sync.
    ///
    /// On CPU-mode fixtures the firmware actually populates SRAM via a
    /// CPU copy (no DMA-based preload), so by the time `is_synced_cpu`
    /// trips SRAM at [`SHADOW_BASE`] is already mirrored. We still lift
    /// the shadow from flash (not SRAM) because it's the canonical
    /// ground truth and the implementation is already plumbed in the
    /// PIO oracle — using flash keeps the two paths symmetric.
    ///
    /// Stage 2: takes a `FixtureSpec` (parsed from the same `flash`
    /// image by the caller) so the per-fixture shadow size + pin map
    /// is honoured.
    pub fn new_at_sync(bus: &mut Bus, spec: FixtureSpec, flash: &[u8]) -> Self {
        // `sdrr_runtime_info.rom_set_index` offset within SRAM — mirror
        // of the PIO oracle. Constants from `sdrr/link/common.ld` +
        // `sdrr_runtime_info_t`.
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
        }
    }

    /// Test-only constructor accepting a pre-built shadow.
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
        }
    }

    /// Drive one case end-to-end.
    ///
    /// 1. Gap drive: CS1/CS2/CS3 all high (deselected), addr=0, for
    ///    [`GAP_CYCLES`] cycles. Gets the CPU back into the "deselected"
    ///    state (outer wait loop + OEN cleared).
    /// 2. Stimulus drive: CS1 low, CS2/CS3 high (A11=A12=1), addr=case
    ///    pattern. Step the emulator until either:
    ///    - the data byte holds steady at the same value for
    ///      [`MIN_STABLE_CYCLES`] consecutive cycles with OEN=0xFF on
    ///      data pins → record the byte + latency, compare against
    ///      shadow, classify verdict.
    ///    - OEN stays 0 for the full budget → `DataPinsNotDriven`.
    ///    - OEN goes high but no stable run forms → `NoStableByte`.
    /// 3. Envelope check: reclassify `Pass` with out-of-envelope latency
    ///    to `LatencyOutOfEnvelope`.
    pub fn run_case(&mut self, emu: &mut Emulator, case: Case) -> &CpuCaseResult {
        // External-input mask covers gate CS, every deasserted-high CS
        // pin, every asserted-low pin, and all address pins. Data pins
        // are CPU-driven — never mask them.
        //
        // The mask and levels are split into low (GPIO 0..31) and high
        // (GPIO 32..47) halves and applied to both `Bus::gpio_external_*`
        // and `Bus::gpio_external_*_hi` (Stage 3A wide-GPIO support;
        // HLD §A). The CPU oracle is currently fire-24-a-only — the
        // shadow-lookup index below is `(stim_level & 0xFFFF)`, a
        // 16-bit u16 that does not capture address bits beyond GPIO15
        // — but the bus-write half is still kept symmetric with the PIO
        // oracle so a future fire-32-a CPU-serve oracle drops in
        // without re-touching this site. When `case.is_literal`, the
        // mask widens to every low-16 bit minus the data pins so the
        // SeaBIOS validator's raw 16-bit pin sweep drives its full
        // pattern verbatim (matches the Stage 1 baseline `0x0000_FFFF`).
        let ext_mask: u64 = self.compose_ext_mask(&case);
        emu.bus.gpio_external_mask = ext_mask as u32;
        emu.bus.gpio_external_mask_hi = (ext_mask >> 32) as u32;

        // 1. Gap drive — gate CS + every deasserted-high + every
        //    asserted-low pin all HIGH so the chip is fully deselected.
        let gap_level: u64 = self.compose_gap_level();
        emu.bus
            .gpio_external_in
            .store(gap_level as u32, Ordering::Relaxed);
        emu.bus
            .gpio_external_in_hi
            .store((gap_level >> 32) as u32, Ordering::Relaxed);
        for _ in 0..GAP_CYCLES {
            emu.run(1).expect("Serial run is infallible");
        }

        // 2. Apply stimulus: gate CS LOW, deasserted-high pins HIGH,
        //    asserted-low pins LOW, case pin_pattern ORed in. When
        //    `case.is_literal` (the SeaBIOS validator path), the
        //    overlay is skipped so the bus carries `pin_pattern`
        //    verbatim — see `compose_stim_level`.
        let stim_level: u64 = self.compose_stim_level(&case);
        emu.bus
            .gpio_external_in
            .store(stim_level as u32, Ordering::Relaxed);
        emu.bus
            .gpio_external_in_hi
            .store((stim_level >> 32) as u32, Ordering::Relaxed);

        // The CPU's serve loop looks up shadow[pins_low_16]. The pins
        // the CPU samples are the 16-bit pattern the stim applies — which
        // is just the low 16 bits of `stim_level`. Shadow lookup offset =
        // pins_low_16.
        let shadow_offset = (stim_level & 0xFFFF) as usize;
        let expected_byte = self.rom_shadow[shadow_offset];

        // Data-pin base — pins are contiguous on supported fixtures.
        let data_base = self.spec.data_pins[0];

        // 3. Tick and observe.
        let mut state = StabilityState::default();
        let mut decision: Option<StabilityDecision> = None;

        for tick in 0..PER_CASE_TIMEOUT {
            emu.run(1).expect("Serial run is infallible");

            let sio_oe = emu.bus.read32(SIO_GPIO_OE_ADDR, 0);
            let oe_data = ((sio_oe >> data_base) & 0xFF) as u8;
            // Observe byte from `gpio_in` — this is the composite of
            // external-input stimulus + SIO/PIO outputs after the
            // bus's `update_gpio` merge. Data bits are CPU-driven so
            // they reflect whatever the CPU's STRB has pushed via
            // SIO_GPIO_OUT.
            let data_byte = ((emu.bus.gpio_in.load(Ordering::Relaxed) >> data_base) & 0xFF) as u8;

            if let Some(d) = observe_tick(&mut state, tick, oe_data, data_byte) {
                decision = Some(d);
                break;
            }
        }

        // 4. Map decision → verdict, classifying timeouts as needed.
        let (verdict, observed_byte, latency_cycles) = match decision {
            Some(StabilityDecision::Stable { byte, at_tick }) => {
                let v = if byte == expected_byte {
                    CpuVerdict::Pass
                } else {
                    CpuVerdict::WrongByte {
                        expected: expected_byte,
                        observed: byte,
                    }
                };
                (v, Some(byte), Some(at_tick))
            }
            None => {
                // Loop fell off the end without a decision. Diagnose.
                if !state.oen_ever_set {
                    (CpuVerdict::DataPinsNotDriven, None, None)
                } else {
                    (CpuVerdict::NoStableByte, None, None)
                }
            }
        };

        let raw = CpuCaseResult {
            case,
            expected_byte: Some(expected_byte),
            observed_byte,
            latency_cycles,
            verdict,
        };

        // 5. Envelope post-process.
        let post = apply_envelope(raw);

        // Leave the bus in gap-level state for the next case. Mirror
        // both halves (low + high) for symmetry with the PIO oracle —
        // see the forward-compat block above and
        // `ServingOracle::run_case` at `onerom_serving_oracle.rs:552-557,
        // 570-575`. Today's CPU oracle is fire-24-a-only (`gap_level`
        // fits in u32) so the high half is a write-of-zero, but
        // keeping the symmetric pair here means a future fire-32-a
        // CPU-serve oracle won't re-introduce the missing-store bug.
        emu.bus
            .gpio_external_in
            .store(gap_level as u32, Ordering::Relaxed);
        emu.bus
            .gpio_external_in_hi
            .store((gap_level >> 32) as u32, Ordering::Relaxed);

        self.results.push(post);
        self.results.last().unwrap()
    }

    /// Accessor for the results vector.
    pub fn results(&self) -> &[CpuCaseResult] {
        &self.results
    }

    /// Accessor for the shadow (for tripwire diagnostics in the binary).
    pub fn shadow(&self) -> &[u8] {
        &self.rom_shadow
    }

    /// Accessor for the per-fixture pin map + capacity.
    pub fn spec(&self) -> &FixtureSpec {
        &self.spec
    }

    // ---------------------------------------------------------------------
    // Stim/gap level composition (mirrors the PIO oracle).
    // ---------------------------------------------------------------------

    /// External-input mask. Mirrors
    /// [`crate::onerom_serving_oracle::ServingOracle::compose_ext_mask`]
    /// — including the `case.is_literal` short-circuit that widens the
    /// mask to every low-16 bit minus the data pins so SeaBIOS-
    /// validator-style raw 16-bit pin sweeps drive their declared
    /// pattern through to the firmware verbatim. The narrow per-spec
    /// mask only covers `addr_pins ∪ CS-gates`, which silently
    /// truncates literal patterns whose bits fall outside that set
    /// (e.g. fire-24-a literal sweep bits 8 + 9).
    fn compose_ext_mask(&self, case: &Case) -> u64 {
        if case.is_literal {
            // Literal: drive all low-16 bits except the data pins. See
            // the PIO-oracle sibling for fixture-by-fixture values.
            let data_mask: u64 = 0xFFu64 << self.spec.data_pins[0];
            return 0x0000_FFFFu64 & !data_mask;
        }
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

    fn compose_gap_level(&self) -> u64 {
        let mut level: u64 = 1u64 << self.spec.cs1;
        for &p in &self.spec.deasserted_high_during_read {
            level |= 1u64 << p;
        }
        for &p in &self.spec.asserted_low_during_read {
            level |= 1u64 << p;
        }
        // Drive any fixture-specific unservable gate HIGH between cases.
        // For fire-24-a this is redundant with CS1 above; 27C-series
        // fire-32-a fixtures keep this mask at zero because GPIO16 is A16
        // and `/CE` + `/OE` provide the read gates.
        level |= self.spec.unservable_when_high;
        level
    }

    /// Stim-level composition. Mirrors
    /// [`crate::onerom_serving_oracle::ServingOracle::compose_stim_level`]
    /// — including the `case.is_literal` short-circuit that skips the
    /// `deasserted_high_during_read` overlay so SeaBIOS-validator-style
    /// raw 16-bit pin sweeps see their declared pattern on the bus
    /// verbatim.
    fn compose_stim_level(&self, case: &Case) -> u64 {
        if case.is_literal {
            return case.pin_pattern;
        }
        let mut level: u64 = 0;
        for &p in &self.spec.deasserted_high_during_read {
            level |= 1u64 << p;
        }
        // Asserted-low pins stay LOW during stim. Already 0 in `level`.
        level | case.pin_pattern
    }

    /// Format the full CPU-serve report (header, per-case table,
    /// summary, emulator-bounded caveat). Signature mirrors the PIO
    /// oracle's [`ServingOracle::format_report`] for consistency.
    pub fn format_report(&self, sys_clk_hz: u32) -> String {
        let mut out = String::new();
        let ns_available = sys_clk_hz != 0;

        // --- Header -------------------------------------------------------
        let _ = writeln!(out, "OneROM CPU-Serve Oracle — Report");
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

        // --- Per-case table -----------------------------------------------
        if ns_available {
            let _ = writeln!(
                out,
                " {:>5}  {:<20} {:<10} {:<8} {:<8} {:>6} {:>6}  verdict",
                "idx", "label", "pattern", "expected", "observed", "cycles", "ns"
            );
        } else {
            let _ = writeln!(
                out,
                " {:>5}  {:<20} {:<10} {:<8} {:<8} {:>6}  verdict",
                "idx", "label", "pattern", "expected", "observed", "cycles"
            );
        }

        let total = self.results.len();
        for (i, r) in self.results.iter().enumerate() {
            let idx = format!("{}/{}", i + 1, total);
            let pattern = format!("0x{:08X}", r.case.pin_pattern as u32);
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
            let verdict = format_cpu_verdict(&r.verdict);
            if ns_available {
                let ns = r
                    .latency_cycles
                    .map(|c| format!("{}", cycles_to_ns(c, sys_clk_hz)))
                    .unwrap_or_else(|| "—".to_string());
                let _ = writeln!(
                    out,
                    " {:>5}  {:<20} {:<10} {:<8} {:<8} {:>6} {:>6}  {}",
                    idx, r.case.label, pattern, expected, observed, cycles, ns, verdict
                );
            } else {
                let _ = writeln!(
                    out,
                    " {:>5}  {:<20} {:<10} {:<8} {:<8} {:>6}  {}",
                    idx, r.case.label, pattern, expected, observed, cycles, verdict
                );
            }
        }
        let _ = writeln!(out);

        // --- Summary ------------------------------------------------------
        let mut pass = 0usize;
        let mut wrong_byte = 0usize;
        let mut not_driven = 0usize;
        let mut no_stable = 0usize;
        let mut latency_oor = 0usize;
        let mut pass_latencies: Vec<u32> = Vec::new();
        for r in &self.results {
            match r.verdict {
                CpuVerdict::Pass => {
                    pass += 1;
                    if let Some(c) = r.latency_cycles {
                        pass_latencies.push(c);
                    }
                }
                CpuVerdict::WrongByte { .. } => wrong_byte += 1,
                CpuVerdict::DataPinsNotDriven => not_driven += 1,
                CpuVerdict::NoStableByte => no_stable += 1,
                CpuVerdict::LatencyOutOfEnvelope { .. } => latency_oor += 1,
            }
        }
        let fail = total - pass;

        let _ = writeln!(out, "Summary:");
        let _ = writeln!(out, "  {} cases total", total);
        let _ = writeln!(out, "  {} PASS", pass);
        let _ = writeln!(
            out,
            "  {} FAIL  ({} wrong-byte, {} data-pins-not-driven, {} no-stable-byte, {} latency-out-of-envelope)",
            fail, wrong_byte, not_driven, no_stable, latency_oor
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
            } else {
                let _ = writeln!(
                    out,
                    "  latency stats (Pass cases only): min={} max={} mean={} cycles (ns unavailable)",
                    min, max, mean
                );
            }
        }

        let _ = writeln!(out);

        // --- Emulator-bounded caveat (mirrors PIO oracle) -----------------
        let _ = writeln!(
            out,
            "  Latency measured against the emulator's CPU step model;"
        );
        let _ = writeln!(
            out,
            "  the serve loop is a 5-instruction tight loop at 0x{:08X}..=0x{:08X}.",
            CPU_SERVE_LOOP_PC_LO, CPU_SERVE_LOOP_PC_HI
        );
        let _ = writeln!(
            out,
            "  Silicon-calibrated timing remains a future pass via the silicon oracle rig."
        );

        out
    }
}

/// Scan window for picking a shadow-readiness sentinel. We look for
/// the last non-zero byte within the final `SENTINEL_SCAN_WINDOW`
/// bytes of the lifted shadow — picked from the tail because the
/// firmware's SRAM copy runs sequentially from offset 0 upward, so a
/// non-zero byte near the end is only populated once the copy is
/// nearly complete. Low-offset bytes (e.g. the first 0x100) showed
/// empirically coincident matches against early pre-init SRAM state
/// on `test-sdrr-0-cpu.bin`, firing the tripwire a full ~8 500 cycles
/// before the shadow was actually populated.
pub const SENTINEL_SCAN_WINDOW: usize = 256;

/// Pick a shadow-readiness sentinel: scan the final
/// [`SENTINEL_SCAN_WINDOW`] bytes of `shadow` *from the end backward*
/// for the first non-zero byte and return its (SRAM offset, expected
/// value). Returns `None` if the scan window is all zeroes (or if
/// `shadow.len() < SENTINEL_SCAN_WINDOW`) — the caller should fall
/// back to the PC-only sync check (no tripwire protection is possible
/// when every byte we could probe is legitimately zero).
///
/// Tail-biased: a sentinel near the end of the shadow is only
/// populated after the firmware's sequential CPU copy has reached
/// that offset, which is a precise signal that the shadow is (very
/// nearly) fully in place. Low-offset sentinels were empirically
/// unreliable — early SRAM state (stack, IVT, runtime_info) can
/// coincidentally match a low-offset shadow byte and fire the
/// tripwire before the shadow copy has run.
///
/// Used by the binary driver to seed the sync detector's tripwire;
/// extracted as a pure function so it can be unit-tested without an
/// `Emulator` or `Bus` in the loop.
pub fn find_shadow_sentinel(shadow: &[u8]) -> Option<(u32, u8)> {
    if shadow.len() < SENTINEL_SCAN_WINDOW {
        return None;
    }
    let start = shadow.len() - SENTINEL_SCAN_WINDOW;
    // Scan from the end backward so the returned offset is the
    // highest non-zero index within the window — i.e. the byte
    // written latest by a sequential CPU copy.
    for i in (start..shadow.len()).rev() {
        if shadow[i] != 0 {
            return Some((i as u32, shadow[i]));
        }
    }
    None
}

/// Pure tripwire helper: returns `true` iff the sentinel is `None`
/// (no tripwire configured → PC check alone decides) or the SRAM byte
/// at the sentinel offset matches the sentinel value (shadow copy has
/// reached or passed this offset).
///
/// Split out from [`is_synced_cpu`] so it can be unit-tested via a
/// probe closure without constructing an `Emulator`. `read_sram_u8`
/// abstracts over the SRAM access — real callers pass a closure over
/// `bus.memory.sram_read8`; tests pass a closure over a fake map.
fn shadow_tripwire_ok<F>(sentinel: Option<(u32, u8)>, mut read_sram_u8: F) -> bool
where
    F: FnMut(u32) -> u8,
{
    match sentinel {
        None => true,
        Some((offset, expected)) => read_sram_u8(offset) == expected,
    }
}

/// Sync detection for the CPU-serve build: returns `true` once core 0's
/// PC lands inside the serve loop range
/// `CPU_SERVE_LOOP_PC_LO..=CPU_SERVE_LOOP_PC_HI` **and** the shadow-
/// readiness tripwire has tripped.
///
/// The PC check alone is not sufficient — the CPU transits the serve-
/// loop PC range during firmware init *before* the SRAM shadow copy
/// has finished. On `test-sdrr-0-cpu.bin`, the PC first enters the
/// range around cycle ~8 400 but the full shadow isn't in place until
/// ~cycle 17 000; declaring sync at the first PC hit caused every
/// non-zero-expected byte to read back as 0x00 (the pre-copy SRAM
/// state).
///
/// The tripwire `sentinel` is `(sram_offset, expected_byte)` picked
/// from the lifted shadow by [`find_shadow_sentinel`]. When the byte
/// at `sram_offset` matches, the firmware's shadow copy has at least
/// reached that offset — sufficient proxy for "shadow populated". If
/// `sentinel` is `None` (scan window was all-zero), we degrade to the
/// bare PC check.
///
/// Takes a whole `&Emulator` (rather than just a `&Bus`) because the
/// PC lives in the core's register file, not on the bus. The PIO-mode
/// sibling takes `&mut Bus` because its sync condition is purely
/// register-level (PIO1.CTRL + PIO2.CTRL both non-zero).
pub fn is_synced_cpu(emu: &Emulator, sentinel: Option<(u32, u8)>) -> bool {
    let pc = emu.core(0).regs.pc();
    if !(CPU_SERVE_LOOP_PC_LO..=CPU_SERVE_LOOP_PC_HI).contains(&pc) {
        return false;
    }
    shadow_tripwire_ok(sentinel, |offset| emu.bus.memory.sram_read8(offset))
}

// ---------------------------------------------------------------------------
// Stability state machine (pure — unit-testable without an emulator)
// ---------------------------------------------------------------------------

/// Accumulator state for the per-tick stability detector.
///
/// Kept out of `run_case` so the detector is a pure function
/// (`observe_tick`) and can be exercised against synthetic traces in
/// tests without an emulator in the loop.
#[derive(Debug, Default, Clone, Copy)]
struct StabilityState {
    /// Have we *ever* seen `oe_data == 0xFF` since stimulus apply? Used
    /// to distinguish `DataPinsNotDriven` from `NoStableByte` on
    /// timeout.
    oen_ever_set: bool,
    /// First byte observed with OEN fully asserted (the "starting"
    /// byte the CPU was driving when we began observing). Used to
    /// detect a transition — the CPU executing a fresh serve
    /// iteration after sampling the new pin state.
    initial_byte: Option<u8>,
    /// Has the observed byte (with OEN=0xFF) ever differed from
    /// `initial_byte`? If true, the CPU has executed at least one
    /// fresh store since stimulus apply, and stability can anchor on
    /// any subsequent value (including the new steady-state byte).
    byte_changed_since_stim: bool,
    /// The byte currently accumulating as a stable-run candidate,
    /// `None` between runs.
    last_byte: Option<u8>,
    /// Tick index (within the per-case observation window) at which
    /// the current stable-run candidate started. `None` between runs.
    stable_start_tick: Option<u32>,
    /// Length of the current stable-run candidate (in cycles).
    stable_run: u32,
}

/// Terminal decision from the stability detector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StabilityDecision {
    /// The byte has held steady for `MIN_STABLE_CYCLES` consecutive
    /// cycles (with OEN fully asserted) after either a transition was
    /// detected or the zero-byte-trust timeout elapsed.
    Stable { byte: u8, at_tick: u32 },
}

/// Pure per-tick stability detector. Returns `Some(decision)` when a
/// terminal verdict can be declared; `None` means "keep ticking".
///
/// Gating rules:
/// 1. Skip pre-floor ticks (`tick < MIN_FRESH_ARRIVAL_CYCLE_CPU`) —
///    any stable run formed before the CPU could have executed a
///    fresh loop iteration is definitionally stale-byte residue.
/// 2. Allow stability to anchor iff *either* a byte transition has
///    been observed since stimulus apply (the CPU has demonstrably
///    executed at least one fresh STRB), *or* the zero-byte trust
///    timeout has elapsed (handling the legitimate "expected byte is
///    0x00, no transition visible" case).
/// 3. Partial OEN (`oe_data != 0xFF`) always resets the stable-run
///    accumulator.
///
/// The detector is a state machine on `StabilityState`; the only
/// coupling back to `run_case` is the returned `StabilityDecision`.
fn observe_tick(
    state: &mut StabilityState,
    tick: u32,
    oe_data: u8,
    data_byte: u8,
) -> Option<StabilityDecision> {
    // Partial drive → reset the stable-run accumulator (but preserve
    // transition/latch state: a mid-case OEN drop shouldn't erase the
    // fact that we've already witnessed the CPU store a fresh byte).
    if oe_data != 0xFF {
        state.last_byte = None;
        state.stable_start_tick = None;
        state.stable_run = 0;
        return None;
    }

    state.oen_ever_set = true;

    // Capture the "starting" byte (the byte the CPU was driving at the
    // moment stimulus was applied) and detect transitions against it.
    match state.initial_byte {
        None => state.initial_byte = Some(data_byte),
        Some(initial) if data_byte != initial => state.byte_changed_since_stim = true,
        _ => {}
    }

    // Stability may only anchor after the fresh-arrival floor AND
    // after either a transition has been seen or the zero-byte
    // timeout has elapsed.
    let past_floor = tick >= MIN_FRESH_ARRIVAL_CYCLE_CPU;
    let past_zero_timeout = tick >= ZERO_BYTE_TRUST_TIMEOUT_CPU;
    let may_anchor = past_floor && (state.byte_changed_since_stim || past_zero_timeout);

    if !may_anchor {
        // Don't accumulate stable-run state until gating is satisfied —
        // otherwise a pre-floor run would carry over and declare
        // stability the instant `may_anchor` flips true.
        state.last_byte = None;
        state.stable_start_tick = None;
        state.stable_run = 0;
        return None;
    }

    // Stability tracking proper.
    if state.last_byte == Some(data_byte) {
        state.stable_run += 1;
        if state.stable_run >= MIN_STABLE_CYCLES as u32 {
            let at_tick = state.stable_start_tick.unwrap_or(tick);
            return Some(StabilityDecision::Stable {
                byte: data_byte,
                at_tick,
            });
        }
    } else {
        state.last_byte = Some(data_byte);
        state.stable_start_tick = Some(tick);
        state.stable_run = 1;
    }

    None
}

// ---------------------------------------------------------------------------
// Envelope post-processing
// ---------------------------------------------------------------------------

/// Apply the emulator-bounded latency envelope to a [`CpuCaseResult`].
/// Mirrors the PIO oracle's [`apply_envelope`] contract: a `Pass` with
/// out-of-envelope latency is reclassified, everything else passes
/// through.
pub fn apply_envelope(result: CpuCaseResult) -> CpuCaseResult {
    match result.verdict {
        CpuVerdict::Pass => match result.latency_cycles {
            Some(cycles) if !CPU_ENVELOPE_CYCLES.contains(&cycles) => CpuCaseResult {
                verdict: CpuVerdict::LatencyOutOfEnvelope { cycles },
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

fn cycles_to_ns(cycles: u32, sys_clk_hz: u32) -> u64 {
    (cycles as u64) * 1_000_000_000 / (sys_clk_hz as u64)
}

fn format_cpu_verdict(v: &CpuVerdict) -> String {
    match v {
        CpuVerdict::Pass => "Pass".to_string(),
        CpuVerdict::WrongByte { expected, observed } => {
            format!("WrongByte(exp=0x{:02X}, obs=0x{:02X})", expected, observed)
        }
        CpuVerdict::DataPinsNotDriven => "DataPinsNotDriven".to_string(),
        CpuVerdict::NoStableByte => "NoStableByte".to_string(),
        CpuVerdict::LatencyOutOfEnvelope { cycles } => {
            format!("LatencyOutOfEnvelope({})", cycles)
        }
    }
}

// ---------------------------------------------------------------------------
// Boot-time image_sel forcing helper
// ---------------------------------------------------------------------------

/// Offset of `sdrr_pins_t` within the OneROM firmware flash image.
/// Matches the constant documented at the top of
/// `onerom_full_system_rp2350.rs` and the journal's hand-decode of the
/// fixtures. All SDRR bakes for the RP2350 family land this struct here;
/// the firmware does not relocate it.
const SDRR_PINS_FLASH_OFFSET: usize = 0x80FC;

/// Offset of `sel[MAX_IMG_SEL_PINS]` within `sdrr_pins_t` (see
/// `sdrr/include/config_base.h`). The array holds up to 7 sel-pin
/// GPIO numbers, with 0xFF (== `INVALID_PIN`) marking unused entries;
/// the entry index is the bit position in the decoded `image_sel`
/// value (entry 0 → bit 0, entry 1 → bit 1, etc.).
const SDRR_PINS_SEL_OFFSET: usize = 52;

/// Maximum number of image-select pins supported by SDRR firmware,
/// mirroring `MAX_IMG_SEL_PINS` in `sdrr/include/config_base.h`.
const MAX_IMG_SEL_PINS: usize = 7;

/// Offset of `sel_jumper_pull` (bit field, LSB = pin 0) within
/// `sdrr_pins_t`. Each bit indicates the direction of the jumper pull
/// when closed for that pin: `1` → jumper-to-high, `0` → jumper-to-low.
/// See `setup_sel_pins` / `get_sel_value` in `sdrr/src/rp235x.c`: the
/// firmware applies the *opposite* pull via the pad, then XORs the
/// sampled GPIOs with a per-pin flip-bits mask so that "closed"
/// always decodes as `1`.
const SDRR_PINS_SEL_JUMPER_PULL_OFFSET: usize = 59;

/// Sentinel value in `sel[]` indicating "pin not wired".
const SEL_INVALID_PIN: u8 = 0xFF;

/// Upper bound on valid GPIO pin numbers for the RP2350A MCU bake
/// (`MAX_USED_GPIOS` in `sdrr/include/reg-rp235x.h`). Pins at or above
/// this value are rejected by the firmware and by this helper.
const MAX_USED_GPIOS_RP2350A: u8 = 30;

/// Parse `sdrr_pins_t.sel[]` and `sel_jumper_pull` out of a OneROM
/// firmware image and return the list of valid (gpio_pin, pull_dir)
/// tuples in the order the firmware reads them. `pull_dir == true`
/// means the jumper pulls the pin high when closed (firmware applies
/// pull-down; raw HIGH decodes to `1`). `pull_dir == false` means the
/// jumper pulls low when closed (firmware applies pull-up; raw LOW
/// decodes to `1`).
///
/// Returns `None` if the flash image is too short to contain the
/// struct. Returns an empty vec if the fixture declares no sel pins
/// (all entries `INVALID_PIN`).
fn parse_sel_pins(flash: &[u8]) -> Option<Vec<(u8, bool)>> {
    let end = SDRR_PINS_FLASH_OFFSET.checked_add(SDRR_PINS_SEL_JUMPER_PULL_OFFSET + 1)?;
    if flash.len() < end {
        return None;
    }
    let sel_base = SDRR_PINS_FLASH_OFFSET + SDRR_PINS_SEL_OFFSET;
    let pull_bits = flash[SDRR_PINS_FLASH_OFFSET + SDRR_PINS_SEL_JUMPER_PULL_OFFSET];
    let mut pins = Vec::with_capacity(MAX_IMG_SEL_PINS);
    for ii in 0..MAX_IMG_SEL_PINS {
        let pin = flash[sel_base + ii];
        if pin == SEL_INVALID_PIN || pin >= MAX_USED_GPIOS_RP2350A {
            continue;
        }
        let pull_dir = (pull_bits >> ii) & 1 != 0;
        pins.push((pin, pull_dir));
    }
    Some(pins)
}

/// Force the SDRR firmware to boot into `rom_set_index` by driving the
/// image-select GPIOs via the emulator's external-input stimulus path
/// before the firmware samples them at boot.
///
/// Why this exists. SDRR picks its active ROM set from jumpers at boot:
/// `check_sel_pins()` reads the sel-pin GPIOs, XORs them against a
/// per-pin `flip_bits` mask (so "jumper closed" always decodes to `1`),
/// and the resulting value modulo `rom_set_count` selects the ROM set.
/// The fire-24-a RP2350 board wires sel pins to GPIO 27/28/29 with all
/// pulls configured so firmware-applied pull-ups and XOR-flip give
/// `sel_value == 7` when the jumpers float; on a 4-set fixture (1541)
/// that lands on index 3, a 27C301 EPROM image with a different pin
/// layout than ROM set 0. The shared [`CpuServingOracle`] library
/// hardcodes ROM-set-0 pin constants — so the sweep needs set 0.
///
/// The emulator does not model pad pull-up/pull-down resistors, so
/// floating sel pins read as `0` in raw GPIO (not as the pulled-up `1`
/// they'd read on silicon). Under the firmware's XOR-flip that decodes
/// to `sel_value == 7` regardless, same wrong outcome.
///
/// Fix: for each sel pin, compute the raw GPIO level the firmware has
/// to see to decode the target bit of `rom_set_index`, then pin that
/// level externally via `gpio_external_in` / `gpio_external_mask`
/// before the firmware reads the pins. The firmware's `disable_sel_pins`
/// call later in boot doesn't conflict — it only clears pad pulls;
/// leaving the external stimulus engaged through sync is harmless
/// because `CpuServingOracle::run_case` rewrites the mask to its own
/// CS+ADDR set once per case.
///
/// Return `Err` if the flash image is malformed, the fixture declares
/// no sel pins (no way to force via this mechanism — firmware would
/// fall through to its default ROM 0 anyway, so caller should skip),
/// or the requested index exceeds what the sel pin count can encode.
///
/// Call **after** `emu.reset()` (which zeros the external stimulus) and
/// **before** any `emu.run(...)` that lets the firmware reach
/// `check_sel_pins()`.
pub fn force_rom_set_index_via_sel_pins(
    emu: &mut Emulator,
    flash: &[u8],
    rom_set_index: u32,
) -> Result<(), String> {
    let pins = parse_sel_pins(flash)
        .ok_or_else(|| "flash image too short to contain sdrr_pins_t".to_string())?;
    if pins.is_empty() {
        return Err("fixture declares no image-select pins".to_string());
    }
    let max_encodable: u64 = 1u64 << pins.len();
    if (rom_set_index as u64) >= max_encodable {
        return Err(format!(
            "rom_set_index {} exceeds range of {} sel pin(s) (max {})",
            rom_set_index,
            pins.len(),
            max_encodable - 1
        ));
    }

    let mut mask: u32 = 0;
    let mut value: u32 = 0;
    for (ii, &(pin, pull_dir)) in pins.iter().enumerate() {
        mask |= 1u32 << pin;
        // `flip_bits` bit is set iff pull_dir == 0 (firmware applied
        // pull-up because jumper pulls low). The decoded bit is
        // `(raw >> pin) ^ flip_bit`; we want that to equal
        // `(rom_set_index >> ii) & 1`, so the raw bit we drive is
        // `decoded ^ flip_bit`.
        let flip_bit = if pull_dir { 0 } else { 1 };
        let decoded_bit = (rom_set_index >> ii) & 1;
        let raw_bit = decoded_bit ^ flip_bit;
        value |= raw_bit << pin;
    }

    // OR into existing stimulus rather than replace — keeps this helper
    // composable with any prior setup (none today, but `gpio_external_*`
    // are shared bus fields and a future caller may stage other pins
    // alongside).
    emu.bus.gpio_external_mask |= mask;
    let prev = emu.bus.gpio_external_in.load(Ordering::Relaxed);
    emu.bus
        .gpio_external_in
        .store((prev & !mask) | (value & mask), Ordering::Relaxed);
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fire24a_fixture_path() -> PathBuf {
        let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        p.push("fixtures");
        p.push("onerom-fire-24-a-rp2350-seabios-cpu.bin");
        p
    }

    fn fire24a_spec() -> FixtureSpec {
        let p = fire24a_fixture_path();
        let flash =
            std::fs::read(&p).unwrap_or_else(|e| panic!("read {} failed: {}", p.display(), e));
        FixtureSpec::from_flash(&flash).expect("fire-24-a parse must succeed")
    }

    fn empty_shadow_for(spec: &FixtureSpec) -> Box<[u8]> {
        vec![0u8; spec.shadow_size].into_boxed_slice()
    }

    /// Byte-extraction semantics: given a `gpio_in` word whose bits
    /// `data_base..data_base+8` carry the data byte, the oracle's
    /// observation pipeline yields exactly those 8 bits.
    #[test]
    fn observed_byte_comes_from_gpio_in_at_data_base() {
        let spec = fire24a_spec();
        let data_base = spec.data_pins[0]; // fire-24-a: 16

        // Walking-1 + known pattern: each byte value appears exactly
        // once across the data-pin range.
        for byte in 0..=255u8 {
            let gpio_in: u32 = (byte as u32) << data_base;
            let observed = ((gpio_in >> data_base) & 0xFF) as u8;
            assert_eq!(observed, byte);
        }

        // Bits below data_base must NOT leak into the observed byte.
        let mask_below: u32 = (1u32 << data_base) - 1;
        let observed = ((mask_below >> data_base) & 0xFF) as u8;
        assert_eq!(observed, 0x00);
    }

    /// OEN-data-mask shape: the oracle rejects partial drive.
    #[test]
    fn data_pin_mask_covers_data_pin_range() {
        let spec = fire24a_spec();
        let data_base = spec.data_pins[0];
        let data_mask: u32 = 0xFFu32 << data_base;

        // fire-24-a: data_base = 16 → mask = 0x00FF_0000.
        assert_eq!(data_mask, 0xFFu32 << 16);

        let all_driven: u32 = 0xFFFF_FFFF;
        let oe_data = ((all_driven >> data_base) & 0xFF) as u8;
        assert_eq!(oe_data, 0xFF);

        let partial: u32 = 1u32 << data_base;
        let oe_data = ((partial >> data_base) & 0xFF) as u8;
        assert_eq!(oe_data, 0x01);
    }

    /// Envelope pass-through.
    #[test]
    fn apply_envelope_passes_through_in_range_latency() {
        let spec = fire24a_spec();
        let case = Case::from_addr("test", 0x1800, &spec);
        let in_range = *CPU_ENVELOPE_CYCLES.start() + 5;
        assert!(CPU_ENVELOPE_CYCLES.contains(&in_range));
        let result = CpuCaseResult {
            case,
            expected_byte: Some(0x42),
            observed_byte: Some(0x42),
            latency_cycles: Some(in_range),
            verdict: CpuVerdict::Pass,
        };
        let out = apply_envelope(result);
        assert_eq!(out.verdict, CpuVerdict::Pass);
        assert_eq!(out.latency_cycles, Some(in_range));
    }

    /// Envelope rewrite.
    #[test]
    fn apply_envelope_rewrites_out_of_range_latency() {
        let spec = fire24a_spec();
        let case = Case::from_addr("test", 0x1800, &spec);
        let out_of_range = *CPU_ENVELOPE_CYCLES.end() + 50;
        assert!(!CPU_ENVELOPE_CYCLES.contains(&out_of_range));
        let result = CpuCaseResult {
            case,
            expected_byte: Some(0x42),
            observed_byte: Some(0x42),
            latency_cycles: Some(out_of_range),
            verdict: CpuVerdict::Pass,
        };
        let out = apply_envelope(result);
        assert_eq!(
            out.verdict,
            CpuVerdict::LatencyOutOfEnvelope {
                cycles: out_of_range
            }
        );
        assert_eq!(out.latency_cycles, Some(out_of_range));
    }

    /// Non-Pass verdicts are never rewritten by the envelope check.
    #[test]
    fn apply_envelope_leaves_non_pass_verdicts_alone() {
        let spec = fire24a_spec();
        let case = Case::from_addr("test", 0x1800, &spec);

        for verdict in [
            CpuVerdict::WrongByte {
                expected: 0x42,
                observed: 0xFF,
            },
            CpuVerdict::DataPinsNotDriven,
            CpuVerdict::NoStableByte,
        ] {
            let out = apply_envelope(CpuCaseResult {
                case,
                expected_byte: Some(0x42),
                observed_byte: None,
                latency_cycles: None,
                verdict,
            });
            assert_eq!(
                out.verdict, verdict,
                "envelope must not rewrite {:?}",
                verdict
            );
        }
    }

    /// CPU default cases mirror the PIO default cases (same generator
    /// — `cpu_default_cases` is a thin wrapper over `default_cases`).
    #[test]
    fn cpu_default_cases_mirror_pio_default_cases() {
        let spec = fire24a_spec();
        let pio = crate::onerom_serving_oracle::default_cases(&spec);
        let cpu = cpu_default_cases(&spec);
        assert_eq!(cpu.len(), pio.len());
        for (a, b) in cpu.iter().zip(pio.iter()) {
            assert_eq!(a.label, b.label);
            assert_eq!(a.pin_pattern, b.pin_pattern);
        }
    }

    /// Shadow capture roundtrip via the test-only constructor.
    #[test]
    fn shadow_lookup_offset_equals_stimulus_low_16() {
        let spec = fire24a_spec();
        let mut shadow = empty_shadow_for(&spec);
        // walk1 A0 case (addr 0x1801).
        let case = Case::from_addr("walk1 A0", 0x1801, &spec);
        // The CPU's serve loop looks up shadow[stim_level & 0xFFFF];
        // stim_level = case.pin_pattern | deasserted_high pins. Compute
        // it the same way the oracle does.
        let mut stim_level: u64 = 0;
        for &p in &spec.deasserted_high_during_read {
            stim_level |= 1u64 << p;
        }
        stim_level |= case.pin_pattern;
        let offset = (stim_level & 0xFFFF) as usize;
        shadow[offset] = 0xA5;

        let oracle = CpuServingOracle::new_with_shadow(spec, shadow);
        assert_eq!(oracle.shadow()[offset], 0xA5);
    }

    /// Drive a synthetic trace through `observe_tick` and return the
    /// terminal decision plus the tick at which it fired.
    fn run_trace(ticks: &[(u8, u8)]) -> Option<(StabilityDecision, u32)> {
        let mut state = StabilityState::default();
        for (i, &(oe, byte)) in ticks.iter().enumerate() {
            if let Some(d) = observe_tick(&mut state, i as u32, oe, byte) {
                return Some((d, i as u32));
            }
        }
        None
    }

    /// Premature-lock regression: before the fix, 4+ consecutive 0x00
    /// bytes with OE=0xFF immediately after stimulus apply would latch
    /// a stable decision at `at_tick=0` with byte=0x00. With the
    /// `MIN_FRESH_ARRIVAL_CYCLE_CPU` floor + transition latch, a
    /// run of zeroes with no subsequent transition must NOT anchor
    /// before the zero-byte trust timeout.
    #[test]
    fn stability_rejects_premature_zero_lock_before_floor() {
        // 5 cycles of OE=0xFF + byte=0x00 — enough to trip
        // `MIN_STABLE_CYCLES=4` if the floor weren't enforced.
        let trace = vec![(0xFFu8, 0x00u8); 5];
        let out = run_trace(&trace);
        assert!(
            out.is_none(),
            "pre-floor zero run should not declare stability; got {:?}",
            out
        );
    }

    /// Locks on a non-zero byte that arrives after a transition and
    /// persists for `MIN_STABLE_CYCLES`. The starting byte is 0x00 and
    /// a fresh store switches it to 0x42 at tick=`MIN_FRESH_ARRIVAL_CYCLE_CPU`.
    #[test]
    fn stability_locks_on_nonzero_after_transition() {
        let floor = MIN_FRESH_ARRIVAL_CYCLE_CPU as usize;
        let mut trace: Vec<(u8, u8)> = vec![(0xFF, 0x00); floor];
        // Fresh byte 0x42 held steady for MIN_STABLE_CYCLES cycles.
        for _ in 0..MIN_STABLE_CYCLES {
            trace.push((0xFF, 0x42));
        }
        let (decision, _fire_tick) = run_trace(&trace).expect("decision expected");
        match decision {
            StabilityDecision::Stable { byte, at_tick } => {
                assert_eq!(byte, 0x42);
                // at_tick is the first tick of the stable run; the
                // transition happens at `floor`, the run completes at
                // `floor + MIN_STABLE_CYCLES - 1`, and at_tick ==
                // `floor`.
                assert_eq!(
                    at_tick, floor as u32,
                    "at_tick should be the first tick of the stable run"
                );
            }
        }
    }

    /// All-zero trace: no transition ever happens, so stability can
    /// only anchor via the zero-byte trust timeout. Once the timeout
    /// elapses and `MIN_STABLE_CYCLES` further zero cycles accrue,
    /// the detector must lock on 0x00.
    #[test]
    fn stability_times_out_to_zero_when_no_transition() {
        let trace: Vec<(u8, u8)> = vec![(0xFF, 0x00); PER_CASE_TIMEOUT as usize];
        let (decision, _fire_tick) =
            run_trace(&trace).expect("zero-byte timeout should eventually fire");
        match decision {
            StabilityDecision::Stable { byte, at_tick } => {
                assert_eq!(byte, 0x00, "should trust the zero once the timeout elapses");
                // at_tick must be >= the zero-byte trust timeout (the
                // earliest cycle at which accumulation is allowed to
                // begin for an all-zero trace).
                assert!(
                    at_tick >= ZERO_BYTE_TRUST_TIMEOUT_CPU,
                    "at_tick={} should be >= ZERO_BYTE_TRUST_TIMEOUT_CPU={}",
                    at_tick,
                    ZERO_BYTE_TRUST_TIMEOUT_CPU
                );
            }
        }
    }

    /// Partial OEN (oe_data != 0xFF) must reset the stable-run
    /// accumulator — we never PASS on half-driven output.
    #[test]
    fn stability_resets_on_partial_oen() {
        let floor = MIN_FRESH_ARRIVAL_CYCLE_CPU as usize;
        // Seed with enough floor cycles + a transition to enable the
        // latch, then a 3-cycle run of 0x42 (short by one), an OEN
        // dip, then another 3-cycle run — neither run individually
        // satisfies MIN_STABLE_CYCLES=4 so no decision should fire.
        let mut trace: Vec<(u8, u8)> = vec![(0xFF, 0x00); floor];
        trace.push((0xFF, 0x42)); // transition
        trace.extend(vec![(0xFF, 0x42); 2]); // 3-cycle run total
        trace.push((0x0F, 0x42)); // OEN partial — reset
        trace.extend(vec![(0xFF, 0x42); 3]); // another 3-cycle run
        let out = run_trace(&trace);
        assert!(
            out.is_none(),
            "two short runs separated by an OEN dip should not anchor; got {:?}",
            out
        );
    }

    // -------------------------------------------------------------------
    // Shadow-readiness tripwire tests
    //
    // The false-sync bug: the CPU transits the serve-loop PC range
    // during firmware init *before* the SRAM shadow copy completes, so
    // the bare PC check would trip at ~cycle 8 400 (stale SRAM) rather
    // than ~cycle 17 000 (shadow populated). The fix gates sync on a
    // sentinel byte within the freshly-copied shadow; these tests pin
    // the two pure helpers driving that gate.
    // -------------------------------------------------------------------

    /// `find_shadow_sentinel` scans the tail window from the end and
    /// returns the highest non-zero index — the latest-written byte
    /// in a sequential CPU copy.
    #[test]
    fn find_shadow_sentinel_returns_last_nonzero_in_tail_window() {
        let spec = fire24a_spec();
        let mut shadow = empty_shadow_for(&spec);
        let tail_start = spec.shadow_size - SENTINEL_SCAN_WINDOW;
        // Two non-zero bytes within the tail window — the higher-
        // offset one must win (scan is end-backward).
        shadow[tail_start + 10] = 0x5A;
        shadow[tail_start + 200] = 0xA5;

        let out = find_shadow_sentinel(&shadow);
        assert_eq!(
            out,
            Some(((tail_start + 200) as u32, 0xA5)),
            "sentinel must be the last non-zero byte in the tail window"
        );
    }

    /// A non-zero byte *before* the tail window does not count.
    #[test]
    fn find_shadow_sentinel_ignores_nonzero_before_tail_window() {
        let spec = fire24a_spec();
        let mut shadow = empty_shadow_for(&spec);
        let tail_start = spec.shadow_size - SENTINEL_SCAN_WINDOW;
        shadow[tail_start - 1] = 0xA5;

        let out = find_shadow_sentinel(&shadow);
        assert_eq!(
            out, None,
            "sentinel scan must not see before the tail window"
        );
    }

    /// A uniformly-zero scan window yields `None`.
    #[test]
    fn find_shadow_sentinel_returns_none_on_all_zero_window() {
        let spec = fire24a_spec();
        let shadow = empty_shadow_for(&spec);
        assert_eq!(find_shadow_sentinel(&shadow), None);
    }

    /// The very last byte of the shadow is within the tail window,
    /// and a non-zero byte at `shadow.len() - 1` wins.
    #[test]
    fn find_shadow_sentinel_picks_shadow_size_minus_one_when_set() {
        let spec = fire24a_spec();
        let mut shadow = empty_shadow_for(&spec);
        let len = spec.shadow_size;
        let tail_start = len - SENTINEL_SCAN_WINDOW;
        shadow[tail_start + 100] = 0x11;
        shadow[len - 1] = 0x3F;

        let out = find_shadow_sentinel(&shadow);
        assert_eq!(
            out,
            Some(((len - 1) as u32, 0x3F)),
            "shadow.len()-1 must win when set"
        );
    }

    /// Tripwire: with no sentinel configured, the helper returns
    /// `true` unconditionally — semantic is "PC check decides alone".
    #[test]
    fn shadow_tripwire_ok_returns_true_when_no_sentinel() {
        let never_called = |_offset: u32| -> u8 {
            panic!("SRAM probe must not be invoked when sentinel is None");
        };
        assert!(shadow_tripwire_ok(None, never_called));
    }

    /// Tripwire: with a sentinel configured, the helper returns
    /// `false` while the SRAM byte at the sentinel offset is still 0
    /// (pre-copy) and flips to `true` once the byte matches the
    /// expected value (copy has reached or passed the sentinel
    /// offset). This is the exact false-sync regression gate.
    #[test]
    fn shadow_tripwire_gates_on_sentinel_byte_value() {
        let sentinel = Some((42u32, 0xA5u8));

        // Pre-copy: SRAM probe returns 0 at the sentinel offset. The
        // tripwire must reject this — PC alone would have mis-declared
        // sync here. Panic on any other offset so a refactor that
        // accidentally probes the wrong byte is caught loudly.
        let pre_copy = |offset: u32| -> u8 {
            assert_eq!(offset, 42, "tripwire must probe at the sentinel offset");
            0x00
        };
        assert!(
            !shadow_tripwire_ok(sentinel, pre_copy),
            "tripwire must reject sync while SRAM at sentinel offset is still 0"
        );

        // Post-copy: SRAM probe returns the expected sentinel value.
        let post_copy = |offset: u32| -> u8 {
            assert_eq!(offset, 42);
            0xA5
        };
        assert!(
            shadow_tripwire_ok(sentinel, post_copy),
            "tripwire must accept sync once SRAM at sentinel offset matches"
        );

        // Wrong-value post-copy: SRAM probe returns a non-zero value
        // that doesn't match the expected sentinel (e.g. a partial /
        // misaligned copy). Tripwire must reject — this is the exact
        // bit-level integrity check the sentinel protocol guarantees.
        let wrong_value = |_offset: u32| -> u8 { 0x42 };
        assert!(
            !shadow_tripwire_ok(sentinel, wrong_value),
            "tripwire must reject SRAM bytes that don't match the sentinel"
        );
    }

    // -------------------------------------------------------------------
    // image_sel helper tests — pure, no emulator in the loop.
    //
    // The helper's job is: given a OneROM firmware image, compute the
    // raw GPIO level required on each sel pin so the firmware decodes
    // `rom_set_index`. These tests pin the encoding math against the
    // fire-24-a sel layout (pins 27/28/29, all pull_dir=0), which
    // matches both the test-sdrr-0 and 1541 fixtures bundled in the
    // crate.
    // -------------------------------------------------------------------

    /// Build a synthetic flash image just large enough to expose
    /// `sdrr_pins_t.sel[]` + `sel_jumper_pull`.
    fn synth_pins_flash(sel: &[u8], pull_bits: u8) -> Vec<u8> {
        let mut flash = vec![0u8; SDRR_PINS_FLASH_OFFSET + SDRR_PINS_SEL_JUMPER_PULL_OFFSET + 1];
        let base = SDRR_PINS_FLASH_OFFSET + SDRR_PINS_SEL_OFFSET;
        // Fill all MAX_IMG_SEL_PINS entries, padding with INVALID_PIN.
        for ii in 0..MAX_IMG_SEL_PINS {
            flash[base + ii] = sel.get(ii).copied().unwrap_or(SEL_INVALID_PIN);
        }
        flash[SDRR_PINS_FLASH_OFFSET + SDRR_PINS_SEL_JUMPER_PULL_OFFSET] = pull_bits;
        flash
    }

    #[test]
    fn parse_sel_pins_decodes_fire_24_a_layout() {
        // Matches the fire-24-a.json pin config and both bundled
        // fixtures' bake: sel = [27, 28, 29, INVALID...], pulls all 0.
        let flash = synth_pins_flash(&[27, 28, 29], 0);
        let pins = parse_sel_pins(&flash).expect("parse");
        assert_eq!(pins, vec![(27, false), (28, false), (29, false)]);
    }

    #[test]
    fn parse_sel_pins_skips_invalid_and_out_of_range() {
        // sel[1] = INVALID_PIN, sel[2] = out of range → both dropped.
        // The remaining two pins keep their original array position for
        // bit-assignment purposes (the caller uses iter-index as the
        // encoded bit).
        let flash = synth_pins_flash(&[5, SEL_INVALID_PIN, 99, 10], 0b1010);
        let pins = parse_sel_pins(&flash).expect("parse");
        // Pin 5 at array index 0 → pull_bits bit 0 = 0 → pull_dir=false.
        // Pin 10 at array index 3 → pull_bits bit 3 = 1 → pull_dir=true.
        assert_eq!(pins, vec![(5, false), (10, true)]);
    }

    #[test]
    fn parse_sel_pins_returns_none_for_short_flash() {
        assert!(parse_sel_pins(&[0u8; 100]).is_none());
    }

    /// Encoding math: with sel=[27,28,29] and pull_dir=false on each,
    /// `flip_bit=1` on all. The raw GPIO we drive must be
    /// `decoded_bit ^ 1`; i.e. to get `sel_value = rom_set_index`, raw
    /// pin `i` should be `!bit(i, rom_set_index)`.
    ///
    /// Uses an `EmulatorBuilder` so we exercise the exact Bus fields
    /// the production call site writes.
    #[test]
    fn force_rom_set_index_sets_bus_fields_correctly() {
        use rp2350_emu::{Config, EmulatorBuilder};

        // Each case: (rom_set_index, expected raw value on pins 27/28/29).
        // pull_dir=false on all three → raw = !decoded.
        // index 0 → decoded 000 → raw 111 → bits 27|28|29 all set.
        // index 3 → decoded 011 → raw 100 → only bit 29 set.
        let flash = synth_pins_flash(&[27, 28, 29], 0);
        let expected_mask = (1u32 << 27) | (1u32 << 28) | (1u32 << 29);
        let cases = [
            (0u32, expected_mask),               // raw 111
            (1u32, (1u32 << 28) | (1u32 << 29)), // raw 110
            (3u32, 1u32 << 29),                  // raw 100
            (7u32, 0u32),                        // raw 000
        ];
        for (index, expected_val) in cases {
            let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
            force_rom_set_index_via_sel_pins(&mut emu, &flash, index).expect("force");
            assert_eq!(
                emu.bus.gpio_external_mask, expected_mask,
                "mask for index {}",
                index
            );
            assert_eq!(
                emu.bus.gpio_external_in.load(Ordering::Relaxed),
                expected_val,
                "value for index {}",
                index
            );
        }
    }

    #[test]
    fn force_rom_set_index_rejects_out_of_range() {
        use rp2350_emu::{Config, EmulatorBuilder};
        // 3 sel pins → max encodable index = 7.
        let flash = synth_pins_flash(&[27, 28, 29], 0);
        let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        assert!(force_rom_set_index_via_sel_pins(&mut emu, &flash, 8).is_err());
    }

    #[test]
    fn force_rom_set_index_rejects_no_sel_pins() {
        use rp2350_emu::{Config, EmulatorBuilder};
        let flash = synth_pins_flash(&[], 0);
        let mut emu = EmulatorBuilder::new(Config::default()).build().unwrap();
        assert!(force_rom_set_index_via_sel_pins(&mut emu, &flash, 0).is_err());
    }
}

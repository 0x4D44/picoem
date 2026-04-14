//! Reference cycle counts for the benchmark sections.
//!
//! The `REFERENCE_TABLE` lists the expected cycle count for each section
//! of `roms/rp2040/benchmark.bin`. The bench poller reads this table
//! when it observes a section-done sentinel and compares the emulator's
//! measured cycles against the entry at the matching index.
//!
//! # RP2040 status
//!
//! Phase 7 ships `mdrp2040app` as a TUI demo mirroring `mdrp2350app`,
//! but the RP2040 ROM collection so far only includes `blinky.bin` —
//! there is no `benchmark.bin` yet. The benchmark panel therefore stays
//! in "waiting..." state with the default firmware. The table below is
//! kept in place so [`crate::devices::bench::BenchmarkPoller`] and its
//! unit tests have a non-empty reference to exercise; when an RP2040
//! benchmark firmware is added, real cycle counts (either emulator-
//! captured or hardware-measured via probe-rs) should replace the zero
//! placeholders.
//!
//! # How to regenerate on real hardware
//!
//! 1. Flash `roms/rp2040/benchmark.bin` (once it exists) to a Raspberry
//!    Pi Pico via probe-rs:
//!
//!    ```text
//!    probe-rs download --chip RP2040 roms/rp2040/benchmark.bin
//!    ```
//!
//! 2. Attach via SWD, let the firmware run to phase = 0xFF, and read
//!    `0x2003_FF00..0x2003_FFFF`. Real-hardware cycle counts come from
//!    the SWD-side measurement.
//!
//! 3. Paste the observed values into the `REFERENCE_TABLE` below.

#[cfg(test)]
use crate::snapshot::{BenchmarkReport, BenchmarkSection};

#[derive(Clone, Copy)]
pub struct BenchmarkReference {
    pub name: &'static str,
    pub iterations: u32,
    pub cycles: u64,
}

/// Ordered by phase index — section N is at index N-1. Placeholder
/// entries (cycles = 0) until an RP2040 benchmark firmware exists.
pub const REFERENCE_TABLE: &[BenchmarkReference] = &[
    BenchmarkReference {
        name: "arith_add",
        iterations: 1_000_000,
        cycles: 0,
    },
    BenchmarkReference {
        name: "arith_mul",
        iterations: 1_000_000,
        cycles: 0,
    },
    BenchmarkReference {
        name: "mem_seq_ld",
        iterations: 500_000,
        cycles: 0,
    },
    BenchmarkReference {
        name: "bit_rev",
        iterations: 1_000_000,
        cycles: 0,
    },
    BenchmarkReference {
        name: "ldm_8regs",
        iterations: 50_000,
        cycles: 0,
    },
];

/// Build a fully-populated [`BenchmarkReport`] from the reference table —
/// useful as a placeholder or as the "expected" shape in unit tests.
#[cfg(test)]
pub fn reference_report() -> BenchmarkReport {
    BenchmarkReport {
        sections: REFERENCE_TABLE
            .iter()
            .map(|r| BenchmarkSection {
                name: r.name,
                emu_cycles: r.cycles,
                ref_cycles: r.cycles,
                iterations: r.iterations,
            })
            .collect(),
        complete: true,
        stall: None,
    }
}

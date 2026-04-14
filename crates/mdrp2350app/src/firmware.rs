//! Reference cycle counts for the benchmark sections.
//!
//! The `REFERENCE_TABLE` lists the expected cycle count for each section
//! of `roms/rp2350/benchmark.bin`. The bench poller (LLD §8.2) reads this table
//! when it observes a section-done sentinel and compares the emulator's
//! measured cycles against the entry at the matching index.
//!
//! # How to regenerate on real hardware
//!
//! 1. Flash `roms/rp2350/benchmark.bin` to a Raspberry Pi Pico 2 via probe-rs:
//!
//!    ```text
//!    probe-rs download --chip RP235x_Arm roms/rp2350/benchmark.bin
//!    ```
//!
//! 2. Attach via SWD, let the firmware run to phase = 0xFF, and read
//!    `0x2003_FF00..0x2003_FFFF`. Real-hardware cycle counts come from
//!    the SWD-side measurement (single-stepping cycles, or enabling DWT
//!    via probe-rs at capture time). See LLD §8.5.
//!
//! 3. Paste the observed values into the `REFERENCE_TABLE` below.
//!
//! # Placeholder values
//!
//! These numbers were **captured from the emulator itself** as a
//! placeholder because the sandboxed development environment has no
//! real hardware attached. Replace with real-hardware values when
//! available (see procedure above). Until that happens, the Δ column
//! in the bench panel is effectively an emulator-vs-itself consistency
//! check — all rows will read zero.

#[cfg(test)]
use crate::snapshot::{BenchmarkReport, BenchmarkSection};

#[derive(Clone, Copy)]
pub struct BenchmarkReference {
    pub name: &'static str,
    pub iterations: u32,
    pub cycles: u64,
}

/// Ordered by phase index — section N is at index N-1. Any change to the
/// benchmark firmware section order must be mirrored here.
///
/// Captured from the emulator itself as a placeholder. Replace with
/// real-hardware values when available (see module docs).
pub const REFERENCE_TABLE: &[BenchmarkReference] = &[
    BenchmarkReference {
        name: "arith_add",
        iterations: 1_000_000,
        cycles: 3_000_006,
    },
    BenchmarkReference {
        name: "arith_mul",
        iterations: 1_000_000,
        cycles: 4_000_007,
    },
    BenchmarkReference {
        name: "arith_sdiv",
        iterations: 100_000,
        cycles: 700_007,
    },
    BenchmarkReference {
        name: "mem_seq_ld",
        iterations: 500_000,
        cycles: 2_000_007,
    },
    BenchmarkReference {
        name: "bit_clz",
        iterations: 1_000_000,
        cycles: 3_000_006,
    },
    BenchmarkReference {
        name: "ldm_8regs",
        iterations: 50_000,
        cycles: 600_006,
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

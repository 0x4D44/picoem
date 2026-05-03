//! One-shot diagnostic: is the fixture's per-ROM-set 64 KiB shadow a
//! direct GPIO-state-indexed lookup table, or a permuted/scrambled one?
//! Originally aimed at the 1541 template; now also used to probe
//! candidate templates (e.g. `test-sdrr-0`) as alternatives.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin probe_seabios_fixture_capability --release
//!   cargo run -p mdpicoem-harness --bin probe_seabios_fixture_capability --release -- \
//!       --fixture <path>
//!
//! `rom_set_count` is read from the fixture's metadata header rather
//! than hard-coded, so this works on fixtures with any number of sets.

use std::collections::HashSet;

use mdpicoem_harness::onerom_serving_oracle::{
    DEFAULT_CASES, lift_shadow_from_flash_pub, parse_rom_set_layout, stimulus_level_pub,
};

const DEFAULT_FIXTURE: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

/// Bitwise-reflected CRC32 (IEEE 802.3 / zlib) — table-free; lets us
/// compare against the 0xDB903413 reference in the predecessor journal.
fn crc32(bytes: &[u8]) -> u32 {
    let mut c: u32 = 0xFFFF_FFFF;
    for &b in bytes {
        c ^= b as u32;
        for _ in 0..8 {
            c = (c >> 1) ^ (0xEDB8_8320 & 0u32.wrapping_sub(c & 1));
        }
    }
    !c
}

fn distinct(s: &[u8]) -> usize { s.iter().copied().collect::<HashSet<_>>().len() }

fn hex32(b: &[u8]) -> String {
    b.iter().map(|x| format!("{:02X}", x)).collect::<Vec<_>>().join(" ")
}

fn parse_cli() -> String {
    let mut fixture = DEFAULT_FIXTURE.to_string();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--fixture" => {
                fixture = args
                    .next()
                    .unwrap_or_else(|| {
                        eprintln!("--fixture needs a value");
                        std::process::exit(2);
                    });
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: probe_seabios_fixture_capability [--fixture <path>]\n\
                     defaults to {DEFAULT_FIXTURE}"
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unrecognised argument: {other}");
                std::process::exit(2);
            }
        }
    }
    fixture
}

fn main() {
    let fixture = parse_cli();
    let flash = match std::fs::read(&fixture) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read fixture {}: {}", fixture, e);
            std::process::exit(2);
        }
    };
    println!("fixture: {} ({} bytes)\n", fixture, flash.len());

    // Read rom_set_count from the fixture's metadata header rather
    // than assuming 4. `parse_rom_set_layout` walks the SDRR struct
    // chain and returns one slot per declared ROM set.
    let layout = match parse_rom_set_layout(&flash) {
        Some(v) => v,
        None => {
            eprintln!("parse_rom_set_layout returned None — fixture metadata mismatch");
            std::process::exit(3);
        }
    };
    let rom_set_count = layout.len();
    println!("rom_set_count: {}", rom_set_count);
    for (k, slot) in layout.iter().enumerate() {
        println!(
            "  set {}: data_offset=0x{:06X} size={}",
            k, slot.data_offset, slot.size
        );
    }
    println!();

    let shadows: Vec<Option<Box<[u8; 0x10000]>>> = (0..rom_set_count)
        .map(|i| lift_shadow_from_flash_pub(&flash, i as u8))
        .collect();

    for (idx, sh_opt) in shadows.iter().enumerate() {
        println!("=== rom_set {} ===", idx);
        let Some(sh) = sh_opt else {
            println!("rom_set {}: lift_shadow_from_flash_pub returned None", idx);
            continue;
        };
        let sh: &[u8; 0x10000] = sh;
        println!("rom_set {}: distinct_bytes_count = {}", idx, distinct(sh));
        println!("rom_set {}: crc32 = 0x{:08X}", idx, crc32(sh));
        println!("rom_set {}: first_32 = {}", idx, hex32(&sh[..32]));
        for c in DEFAULT_CASES {
            let pin_state = stimulus_level_pub(c.addr_bits) & 0xFFFF;
            println!(
                "set={} addr_bits=0x{:04X} pin_state=0x{:04X} shadow[pin_state]=0x{:02X}  ({})",
                idx, c.addr_bits, pin_state, sh[pin_state as usize], c.label
            );
        }
        println!();
    }

    let eq = |a: usize, b: usize| -> bool {
        match (&shadows[a], &shadows[b]) {
            (Some(x), Some(y)) => x.as_ref() == y.as_ref(),
            _ => false,
        }
    };
    for a in 0..rom_set_count {
        for b in (a + 1)..rom_set_count {
            println!("shadow_set_{}_eq_set_{}: {}", a, b, eq(a, b));
        }
    }

    let dist0 = shadows
        .first()
        .and_then(|s| s.as_ref())
        .map(|s| distinct(s.as_ref()))
        .unwrap_or(0);
    let any_loaded = shadows.iter().any(|s| s.is_some());
    let verdict = if !any_loaded {
        "?  (no rom_sets parsed — fixture metadata mismatch)"
    } else if dist0 <= 1 {
        "all-zeros  (set 0 has <=1 distinct byte; lift returned blank shadow)"
    } else {
        "see-data-above  (distinct>1 — judge direct-lookup vs permuted from walk1 rows + crc32 vs 0xDB903413)"
    };
    println!("\nVERDICT: {}", verdict);
}

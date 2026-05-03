//! One-shot diagnostic: is the 1541 fixture's per-ROM-set 64 KiB shadow
//! a direct GPIO-state-indexed lookup table, or a permuted/scrambled one?
//! Decides whether 256 KiB SeaBIOS can be served via 4 ROM sets x 64 KiB.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --bin probe_seabios_fixture_capability --release

use std::collections::HashSet;

use mdpicoem_harness::onerom_serving_oracle::{
    DEFAULT_CASES, lift_shadow_from_flash_pub, stimulus_level_pub,
};

const FIXTURE: &str = "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";

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

fn main() {
    let flash = match std::fs::read(FIXTURE) {
        Ok(b) => b,
        Err(e) => { eprintln!("failed to read fixture {}: {}", FIXTURE, e); std::process::exit(2); }
    };
    println!("fixture: {} ({} bytes)\n", FIXTURE, flash.len());

    let shadows: Vec<Option<Box<[u8; 0x10000]>>> =
        (0u8..4).map(|i| lift_shadow_from_flash_pub(&flash, i)).collect();

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
    for (a, b) in [(0,1),(0,2),(0,3),(1,2),(1,3),(2,3)] {
        println!("shadow_set_{}_eq_set_{}: {}", a, b, eq(a,b));
    }

    let dist0 = shadows[0].as_ref().map(|s| distinct(s.as_ref())).unwrap_or(0);
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

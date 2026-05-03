//! Build an SDRR-format fixture for the 256 KiB SeaBIOS image.
//!
//! Loads the 1541 fixture as a template (carrying the SDRR firmware +
//! struct envelope), then overwrites each of its 4 ROM sets (4 x 64 KiB
//! = 256 KiB total shadow capacity) with the corresponding 64 KiB
//! quarter of the SeaBIOS image. The shadow is a direct GPIO-state-
//! indexed LUT — each `shadow[pin_state]` byte is exactly the SeaBIOS
//! byte at the matching offset within that quarter.
//!
//! Usage:
//!   cargo run -p mdpicoem-harness --release --bin build_seabios_fixture
//!   cargo run -p mdpicoem-harness --release --bin build_seabios_fixture -- \
//!       --template <path> --seabios <path> --output <path>

use std::path::PathBuf;
use std::process::ExitCode;

use mdpicoem_harness::onerom_serving_oracle::{SHADOW_SIZE, parse_rom_set_layout};

const DEFAULT_TEMPLATE: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin";
const DEFAULT_SEABIOS: &str = "crates/mdpicoem-harness/fixtures/sources/seabios-256k.bin";
const DEFAULT_OUTPUT: &str =
    "crates/mdpicoem-harness/fixtures/onerom-fire-24-a-rp2350-seabios-cpu.bin";

const SEABIOS_SIZE: usize = 256 * 1024;
const EXPECTED_ROM_SET_COUNT: usize = 4;

/// Bitwise-reflected CRC32 (IEEE 802.3 / zlib) — table-free; matches the
/// `probe_seabios_fixture_capability` helper so the human can compare
/// printed CRCs across the two binaries.
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

struct Cli {
    template: PathBuf,
    seabios: PathBuf,
    output: PathBuf,
}

fn parse_cli() -> Result<Cli, String> {
    let mut template = PathBuf::from(DEFAULT_TEMPLATE);
    let mut seabios = PathBuf::from(DEFAULT_SEABIOS);
    let mut output = PathBuf::from(DEFAULT_OUTPUT);

    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--template" => {
                template = PathBuf::from(args.next().ok_or("--template needs a value")?);
            }
            "--seabios" => {
                seabios = PathBuf::from(args.next().ok_or("--seabios needs a value")?);
            }
            "--output" => {
                output = PathBuf::from(args.next().ok_or("--output needs a value")?);
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: build_seabios_fixture [--template <path>] [--seabios <path>] [--output <path>]"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unrecognised argument: {other}")),
        }
    }
    Ok(Cli {
        template,
        seabios,
        output,
    })
}

fn main() -> ExitCode {
    mdpicoem_harness::harness_tracing_init();

    let cli = match parse_cli() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(2);
        }
    };

    println!("template: {}", cli.template.display());
    println!("seabios:  {}", cli.seabios.display());
    println!("output:   {}", cli.output.display());

    let mut fixture = match std::fs::read(&cli.template) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read template: {e}");
            return ExitCode::from(2);
        }
    };
    let seabios = match std::fs::read(&cli.seabios) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("failed to read seabios image: {e}");
            return ExitCode::from(2);
        }
    };
    println!(
        "loaded template ({} bytes), seabios ({} bytes)",
        fixture.len(),
        seabios.len()
    );

    if seabios.len() != SEABIOS_SIZE {
        eprintln!(
            "seabios image must be exactly {} bytes; got {}",
            SEABIOS_SIZE,
            seabios.len()
        );
        return ExitCode::from(3);
    }

    let layout = match parse_rom_set_layout(&fixture) {
        Some(v) => v,
        None => {
            eprintln!("failed to parse SDRR ROM set layout from template");
            return ExitCode::from(3);
        }
    };
    if layout.len() != EXPECTED_ROM_SET_COUNT {
        eprintln!(
            "template has {} ROM sets; expected exactly {}",
            layout.len(),
            EXPECTED_ROM_SET_COUNT
        );
        return ExitCode::from(3);
    }
    for (k, slot) in layout.iter().enumerate() {
        if slot.size != SHADOW_SIZE {
            eprintln!(
                "ROM set {} has size {} bytes; expected {} (SHADOW_SIZE)",
                k, slot.size, SHADOW_SIZE
            );
            return ExitCode::from(3);
        }
        println!(
            "  rom_set {}: data_offset=0x{:08X} size=0x{:X}",
            k, slot.data_offset, slot.size
        );
    }

    // Normalise the per-set `roms[]` pointer at descriptor `+0x08..+0x0B`.
    // The 1541 template has overlapping per-set roms[] arrays — set 3's
    // pointer lands inside another set's array, so the firmware
    // dereferences garbage as a chip_type/pin_map descriptor and never
    // syncs. Copying set 0's pointer into all sets ensures every set
    // boots through the same firmware code path with the same valid
    // sdrr_rom_info_t.
    const ROMS_PTR_FIELD: usize = 0x08;
    let canonical_ptr: [u8; 4] = fixture[layout[0].descriptor_offset + ROMS_PTR_FIELD
        ..layout[0].descriptor_offset + ROMS_PTR_FIELD + 4]
        .try_into()
        .expect("4-byte slice");
    for slot in layout.iter().skip(1) {
        let dst = slot.descriptor_offset + ROMS_PTR_FIELD;
        fixture[dst..dst + 4].copy_from_slice(&canonical_ptr);
    }
    println!(
        "patched {} ROM-set descriptors to share set 0's roms[] pointer (was unique-per-set in template)",
        layout.len()
    );

    // Each ROM set k receives the k-th 64 KiB chunk of seabios.
    // Direct copy — no permutation. Validator drives raw 16-bit GPIO
    // states so `shadow[pin_state] == seabios[k * 0x10000 + pin_state]`.
    for (k, slot) in layout.iter().enumerate() {
        let src_lo = k * SHADOW_SIZE;
        let src_hi = src_lo + SHADOW_SIZE;
        let dst_lo = slot.data_offset;
        let dst_hi = dst_lo + SHADOW_SIZE;
        fixture[dst_lo..dst_hi].copy_from_slice(&seabios[src_lo..src_hi]);
    }
    println!(
        "patched {} ROM sets ({} bytes total)",
        layout.len(),
        layout.len() * SHADOW_SIZE
    );

    if let Err(e) = std::fs::write(&cli.output, &fixture) {
        eprintln!("failed to write output: {e}");
        return ExitCode::from(2);
    }

    println!("output: {} bytes", fixture.len());
    println!("crc32:  0x{:08X}", crc32(&fixture));
    println!("done");
    ExitCode::SUCCESS
}

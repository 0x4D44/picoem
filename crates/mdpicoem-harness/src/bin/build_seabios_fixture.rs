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

const DEFAULT_TEMPLATE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/onerom-fire-24-a-rp2350-1541-cpu.bin"
);
const DEFAULT_SEABIOS: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/sources/seabios-256k.bin"
);
const DEFAULT_OUTPUT: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/onerom-fire-24-a-rp2350-seabios-cpu.bin"
);

const SEABIOS_SIZE: usize = 256 * 1024;
const EXPECTED_ROM_SET_COUNT: usize = 4;

/// Documented SHA-256 of `fixtures/sources/seabios-256k.bin` per
/// `wrk_journals/2026.05.03 - JRN - SDRR SeaBIOS fixture.md`. Asserted
/// at build time so a silent upstream change in mddosem's SeaBIOS
/// surfaces immediately — the chunk-to-shadow mapping bakes the source
/// bytes into the fixture, so a different source equals a different
/// fixture without anyone realising.
const EXPECTED_SEABIOS_SHA256: [u8; 32] = [
    0xae, 0x6f, 0x6a, 0xa9, 0x73, 0xaa, 0xcc, 0xc1, 0x43, 0xf5, 0x7a, 0xa9, 0x60, 0xfb, 0x03, 0x5f,
    0xd9, 0xde, 0x4d, 0xae, 0xe4, 0xad, 0x0c, 0xd7, 0x13, 0x32, 0x2f, 0x8c, 0x25, 0x9e, 0x76, 0x50,
];

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

/// Minimal inlined SHA-256 (FIPS 180-4). One-shot only — no streaming
/// API, no incremental update; takes a single byte slice and returns
/// the 32-byte digest. Intended only for the `EXPECTED_SEABIOS_SHA256`
/// guard below — if you need SHA-256 elsewhere in the harness, prefer
/// adding a workspace dep on `sha2` over duplicating this.
fn sha256(input: &[u8]) -> [u8; 32] {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];

    // Padded message: original || 0x80 || 0x00s || u64-be-bit-length.
    let bit_len: u64 = (input.len() as u64).wrapping_mul(8);
    let mut padded = Vec::with_capacity(input.len() + 72);
    padded.extend_from_slice(input);
    padded.push(0x80);
    while (padded.len() % 64) != 56 {
        padded.push(0x00);
    }
    padded.extend_from_slice(&bit_len.to_be_bytes());

    for chunk in padded.chunks_exact(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes(chunk[i * 4..i * 4 + 4].try_into().unwrap());
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let (mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh) =
            (h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
        h[5] = h[5].wrapping_add(f);
        h[6] = h[6].wrapping_add(g);
        h[7] = h[7].wrapping_add(hh);
    }

    let mut out = [0u8; 32];
    for (i, &word) in h.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    out
}

fn hex_digest(d: &[u8; 32]) -> String {
    let mut s = String::with_capacity(64);
    for &b in d {
        s.push_str(&format!("{:02x}", b));
    }
    s
}

#[cfg(test)]
mod sha256_tests {
    use super::sha256;

    /// FIPS 180-4 Appendix B test vector: SHA-256("abc").
    #[test]
    fn sha256_abc() {
        let expected = [
            0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
            0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
            0xf2, 0x00, 0x15, 0xad,
        ];
        assert_eq!(sha256(b"abc"), expected);
    }

    /// FIPS 180-4 Appendix B test vector: SHA-256(""). Pads exactly to
    /// the empty boundary, exercising the pad-length edge.
    #[test]
    fn sha256_empty() {
        let expected = [
            0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f,
            0xb9, 0x24, 0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b,
            0x78, 0x52, 0xb8, 0x55,
        ];
        assert_eq!(sha256(b""), expected);
    }

    /// Multi-block input (longer than one 512-bit chunk). FIPS 180-4
    /// Appendix B vector: 56-byte test string.
    #[test]
    fn sha256_56byte_string() {
        let input = b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
        let expected = [
            0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8, 0xe5, 0xc0, 0x26, 0x93, 0x0c, 0x3e,
            0x60, 0x39, 0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff, 0x21, 0x67, 0xf6, 0xec, 0xed, 0xd4,
            0x19, 0xdb, 0x06, 0xc1,
        ];
        assert_eq!(sha256(input), expected);
    }
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

    eprintln!(
        "build_seabios_fixture: template={} seabios={} output={}",
        cli.template.display(),
        cli.seabios.display(),
        cli.output.display()
    );

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

    let actual_sha = sha256(&seabios);
    if actual_sha != EXPECTED_SEABIOS_SHA256 {
        eprintln!(
            "seabios-256k.bin SHA-256 mismatch — did upstream SeaBIOS in mddosem change? \
             Update EXPECTED_SEABIOS_SHA256 in build_seabios_fixture.rs and re-run.\n\
               expected: {}\n\
               actual:   {}",
            hex_digest(&EXPECTED_SEABIOS_SHA256),
            hex_digest(&actual_sha)
        );
        return ExitCode::from(3);
    }
    println!("seabios sha256: {}", hex_digest(&actual_sha));

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
    //
    // Empirical evidence (from probe_rom_set_descriptors on the 1541 template,
    // see wrk_journals/2026.05.03 - JRN - SDRR SeaBIOS fixture.md):
    //   set 0 +0x08: 0x1000_C210  (canonical, lands on a valid sdrr_rom_info_t)
    //   set 1 +0x08: 0x1000_C20C  (-4 bytes; still a valid descriptor by overlap)
    //   set 2 +0x08: 0x1000_C208  (-8 bytes; still valid by overlap)
    //   set 3 +0x08: 0x1000_C204  (-12 bytes; lands on the array LENGTH cell
    //                              `u32 = 0x00000004`; firmware casts 4 → ptr
    //                              and faults)
    // We patch sets 1, 2, 3 to share set 0's pointer so all four route through
    // the same firmware code path with the same pin map / chip type.
    //
    // WARNING: this patch is template-specific to the 1541 fixture. The
    // other ROM-set entries in this template are NOT alternate copies of
    // the 1541 ROM — they are DIFFERENT chip variants (set 3 is a 27C301
    // per onerom_stress_cpu_rp2350.rs's file header comment). Forcing
    // set 0's roms[] pointer onto sets 1/2/3 routes all four through
    // set 0's pin map (CS=GPIO13, data=GPIO16..23). This works only
    // because Stream B drives raw 16-bit GPIO patterns at CS1=GPIO13
    // and doesn't care about the original chip-variant identity. Do NOT
    // blindly apply this patch to other SDRR templates.
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

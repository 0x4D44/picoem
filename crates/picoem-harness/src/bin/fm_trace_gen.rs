// fm_trace_gen — emit a hand-crafted picogus-tap v1 CSV trace that
// drives an OPL3 to play one sustained tone on channel 0, then key-off.
//
// Trace format spec: `third_party/README.md` and the canonical fixture
// `crates/picoem-harness/fixtures/sample_gus.trace`. Lower-bank OPL3
// register writes go through ports 0x388 (addr) / 0x389 (data); the
// upper bank (registers 0x100..) is reached via 0x38A / 0x38B. 2 µs
// spacing between events keeps us well clear of any plausible firmware
// ack window.
//
// Frequency-to-(block,fnum) mapping uses 2^(19-block) divisor —
// empirically calibrated to match PicoGUS-as-deployed (slot 5
// SB-DBOPL3 in picogus-v4.0.0.bin), which produces 2× the
// textbook OPL3 frequency for unknown firmware-side reasons.
// See wrk_journals/2026.04.25 - JRN - FM Pivot.md session 27.
//
// Supported tone frequency range under this calibration: ~0.1 Hz ..
// ~12.4 kHz (OPL3 BLOCK 0..=7, F-num 0..=1023 against the 49716 Hz
// reference). --duration-ms must be at least 10 ms (below that the
// OPL3 release envelope cannot complete).

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

const OPL3_ADDR_PORT: u16 = 0x388;
const OPL3_DATA_PORT: u16 = 0x389;
const OPL3_ADDR_PORT_HI: u16 = 0x38a;
const OPL3_DATA_PORT_HI: u16 = 0x38b;
const EVENT_SPACING_NS: u64 = 2_000;
const MIN_DURATION_MS: u64 = 10;

#[derive(Debug)]
struct Args {
    out: PathBuf,
    frequency: f64,
    duration_ms: u64,
}

fn parse_args() -> Result<Args, String> {
    let raw: Vec<String> = std::env::args().skip(1).collect();
    let mut out: Option<PathBuf> = None;
    let mut frequency: f64 = 440.0;
    let mut duration_ms: u64 = 1000;
    let mut i = 0;
    while i < raw.len() {
        match raw[i].as_str() {
            "--out" => {
                i += 1;
                if i >= raw.len() {
                    return Err("--out requires a path".into());
                }
                out = Some(PathBuf::from(&raw[i]));
            }
            "--frequency" => {
                i += 1;
                if i >= raw.len() {
                    return Err("--frequency requires Hz".into());
                }
                frequency = raw[i]
                    .parse::<f64>()
                    .map_err(|e| format!("invalid --frequency '{}': {e}", raw[i]))?;
                if !frequency.is_finite() || frequency <= 0.0 {
                    return Err("--frequency must be a finite value > 0".into());
                }
            }
            "--duration-ms" => {
                i += 1;
                if i >= raw.len() {
                    return Err("--duration-ms requires ms".into());
                }
                duration_ms = raw[i]
                    .parse::<u64>()
                    .map_err(|e| format!("invalid --duration-ms '{}': {e}", raw[i]))?;
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument '{other}'")),
        }
        i += 1;
    }
    if duration_ms < MIN_DURATION_MS {
        return Err(format!(
            "--duration-ms must be at least {MIN_DURATION_MS} (OPL3 envelope release floor); got {duration_ms}"
        ));
    }
    Ok(Args {
        out: out.ok_or("--out is required")?,
        frequency,
        duration_ms,
    })
}

fn print_usage() {
    eprintln!(
        "Usage:\n  \
         fm_trace_gen --out <path> [--frequency <hz>] [--duration-ms <ms>]\n\
         \n\
         --out          Required. Output trace file (picogus-tap v1 CSV).\n\
         --frequency    Optional (default 440.0). Tone frequency in Hz.\n\
         \x20              Supported range: ~0.1 Hz .. ~12.4 kHz (OPL3 BLOCK 0..=7).\n\
         --duration-ms  Optional (default 1000). Hold time before key-off.\n\
         \x20              Minimum 10 ms (OPL3 envelope release floor)."
    );
}

#[cfg(test)]
fn f_num_for_freq(freq_hz: f64, block: u8) -> u16 {
    let denom = 49716.0_f64 / (1u64 << (19 - block as u32)) as f64;
    let raw = (freq_hz / denom).round() as i64;
    raw.clamp(0, 1023) as u16
}

/// Pick the smallest BLOCK in 0..=7 such that the OPL3 F-num for `freq_hz`
/// fits in the legal 10-bit field. Smaller BLOCK gives finer pitch
/// resolution; we walk up only when forced to.
fn choose_block_and_fnum(freq_hz: f64) -> Result<(u8, u16), String> {
    for block in 0..=7u8 {
        let denom = (1u32 << (19 - block as u32)) as f64;
        let fnum = (freq_hz * denom / 49716.0).round() as i64;
        if (0..=1023).contains(&fnum) {
            return Ok((block, fnum as u16));
        }
    }
    Err(format!(
        "frequency {freq_hz} Hz out of range for OPL3 (block 0..=7, F-num 0..=1023)"
    ))
}

fn reg_write_events(base_ns: u64, reg: u8, data: u8) -> [(u64, u16, u8); 2] {
    [
        (base_ns, OPL3_ADDR_PORT, reg),
        (base_ns + EVENT_SPACING_NS, OPL3_DATA_PORT, data),
    ]
}

fn reg_write_events_hi(base_ns: u64, reg: u8, data: u8) -> [(u64, u16, u8); 2] {
    [
        (base_ns, OPL3_ADDR_PORT_HI, reg),
        (base_ns + EVENT_SPACING_NS, OPL3_DATA_PORT_HI, data),
    ]
}

fn build_events(frequency: f64, duration_ms: u64) -> Result<Vec<(u64, u16, u8)>, String> {
    let (block, f_num) = choose_block_and_fnum(frequency)?;
    let f_num_lo = (f_num & 0xFF) as u8;
    let f_num_hi = ((f_num >> 8) & 0x03) as u8;
    let kon_byte = 0x20 | (block << 2) | f_num_hi; // KON=1
    let off_byte = (block << 2) | f_num_hi; // KON=0

    let setup: &[(u8, u8)] = &[
        (0x20, 0x01), // op0 (mod ch0): MULT=1
        (0x40, 0x10), // op0: KSL=00, output level=0x10
        (0x60, 0xF0), // op0: AR=15, DR=0
        (0x80, 0x00), // op0: SL=0, RR=0
        (0xE0, 0x00), // op0: waveform = sine
        (0x23, 0x01), // op1 (car ch0): MULT=1
        (0x43, 0x00), // op1: full volume
        (0x63, 0xF0), // op1: AR=15, DR=0
        (0x83, 0x00), // op1: SL=0, RR=0
        (0xE3, 0x00), // op1: waveform = sine
        (0xC0, 0x31), // ch0: FB=0, ALG=1, CHA/CHB=11 (L+R)
        (0xA0, f_num_lo),
        // ALG=1 ("parallel") makes both operators carriers — simpler near-pure
        // tone with less timbral coupling than ALG=0 (FM/series), which is what
        // we want for a clean reference A. KON last so the envelope generator
        // sees the final operator/channel config before the key event.
        (0xB0, kon_byte),
    ];

    let mut events: Vec<(u64, u16, u8)> = Vec::with_capacity(setup.len() * 2 + 4);
    let mut t: u64 = 0;

    // OPL3 NEW bit (reg 0x105 in upper bank) must be set first; without it
    // the chip stays OPL2-compatible and the CHA/CHB stereo bits in 0xC0
    // (and any 4-op / extended-channel features) are ignored.
    let enable_pair = reg_write_events_hi(t, 0x05, 0x01);
    events.extend_from_slice(&enable_pair);
    t = enable_pair[1].0 + EVENT_SPACING_NS;

    for &(reg, data) in setup {
        let pair = reg_write_events(t, reg, data);
        events.extend_from_slice(&pair);
        t = pair[1].0 + EVENT_SPACING_NS;
    }

    let hold_ns = duration_ms.saturating_mul(1_000_000);
    let off_base = t.saturating_add(hold_ns);
    let off_pair = reg_write_events(off_base, 0xB0, off_byte);
    events.extend_from_slice(&off_pair);

    Ok(events)
}

fn write_trace<W: Write>(w: &mut W, events: &[(u64, u16, u8)]) -> std::io::Result<()> {
    writeln!(w, "# picogus-tap v1")?;
    writeln!(w, "ns,port,value,kind")?;
    for &(ns, port, value) in events {
        writeln!(w, "{ns},0x{port:03x},0x{value:02x},write8")?;
    }
    Ok(())
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            print_usage();
            std::process::exit(2);
        }
    };

    let events = match build_events(args.frequency, args.duration_ms) {
        Ok(ev) => ev,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(2);
        }
    };
    let file = match File::create(&args.out) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("error: failed to create '{}': {e}", args.out.display());
            std::process::exit(1);
        }
    };
    let mut w = BufWriter::new(file);
    if let Err(e) = write_trace(&mut w, &events) {
        eprintln!("error: failed to write trace: {e}");
        std::process::exit(1);
    }
    if let Err(e) = w.flush() {
        eprintln!("error: failed to flush trace: {e}");
        std::process::exit(1);
    }

    println!(
        "wrote {} events ({} Hz, {} ms hold) to {}",
        events.len(),
        args.frequency,
        args.duration_ms,
        args.out.display()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn f_num_for_440hz_block4_is_about_0x122() {
        // Empirical calibration uses 2^(19-block). For BLOCK=4 the divisor
        // is 49716 / 2^15 ≈ 1.5172, so 440 Hz → ~290 → 0x122.
        let n = f_num_for_freq(440.0, 4);
        let target = 0x122_u16;
        let diff = (n as i32 - target as i32).abs();
        assert!(diff <= 1, "f_num={n:#x}, expected ~{target:#x}");
    }

    #[test]
    fn events_alternate_addr_then_data_ports() {
        let events = build_events(440.0, 1000).unwrap();
        for (i, &(_, port, _)) in events.iter().enumerate() {
            // Even indices are address writes; odd are data. The first pair
            // uses the upper-bank ports (0x38a/0x38b); everything after uses
            // the lower-bank ports (0x388/0x389).
            let (addr_port, data_port) = if i < 2 {
                (OPL3_ADDR_PORT_HI, OPL3_DATA_PORT_HI)
            } else {
                (OPL3_ADDR_PORT, OPL3_DATA_PORT)
            };
            let expected = if i % 2 == 0 { addr_port } else { data_port };
            assert_eq!(
                port, expected,
                "event {i}: port {port:#05x}, expected {expected:#05x}"
            );
        }
    }

    #[test]
    fn final_event_is_key_off() {
        let events = build_events(440.0, 1000).unwrap();
        let last = events.last().expect("at least one event");
        // Last event must be the data write of the key-off pair.
        assert_eq!(last.1, OPL3_DATA_PORT);
        assert_eq!(last.2 & 0x20, 0, "KON bit must be clear: {:#04x}", last.2);
        // Penultimate event must be the matching register-select to 0xB0.
        let prev = events[events.len() - 2];
        assert_eq!(prev.1, OPL3_ADDR_PORT);
        assert_eq!(prev.2, 0xB0);
    }

    #[test]
    fn csv_starts_with_magic_and_header() {
        let events = build_events(440.0, 10).unwrap();
        let mut buf: Vec<u8> = Vec::new();
        write_trace(&mut buf, &events).unwrap();
        let text = String::from_utf8(buf).unwrap();
        let mut lines = text.lines();
        assert_eq!(lines.next(), Some("# picogus-tap v1"));
        assert_eq!(lines.next(), Some("ns,port,value,kind"));
    }

    #[test]
    fn timestamps_are_monotonic_nondecreasing() {
        let events = build_events(440.0, 1000).unwrap();
        let mut prev = 0_u64;
        for &(ns, _, _) in &events {
            assert!(ns >= prev, "non-monotonic timestamp: {prev} -> {ns}");
            prev = ns;
        }
    }

    #[test]
    fn first_pair_enables_opl3_new_bit() {
        let events = build_events(440.0, 1000).unwrap();
        // Item 1: the very first register-programming pair must be the
        // OPL3 NEW-bit enable (reg 0x105 = 0x01) on the upper-bank ports.
        assert!(events.len() >= 2, "expected at least two events");
        assert_eq!(events[0].1, 0x38a, "first event port");
        assert_eq!(events[0].2, 0x05, "first event value (reg 0x105 low byte)");
        assert_eq!(events[1].1, 0x38b, "second event port");
        assert_eq!(events[1].2, 0x01, "second event value (NEW bit set)");
    }

    #[test]
    fn block_and_fnum_match_known_220hz() {
        // Empirical calibration: smallest block where F-num fits in 10 bits.
        // For 220 Hz, blocks 0..=1 overflow (fnum > 1023); block=2 gives
        // fnum = 220 * 2^17 / 49716 ≈ 580 = 0x244, which fits.
        let (block, fnum) = choose_block_and_fnum(220.0).unwrap();
        assert_eq!(block, 2);
        assert_eq!(fnum, 0x244);
    }

    #[test]
    fn block_and_fnum_match_known_440hz() {
        // 440 Hz under the empirical calibration: block=3 gives
        // fnum = 440 * 2^16 / 49716 ≈ 580 = 0x244.
        let (block, fnum) = choose_block_and_fnum(440.0).unwrap();
        assert_eq!(block, 3);
        assert_eq!(fnum, 0x244);
    }

    #[test]
    fn block_and_fnum_uses_largest_block_for_high_freq() {
        // 10000 Hz is near the top of the OPL3 representable range under
        // the empirical calibration (~12.4 kHz max). block=7 is the only
        // one that fits.
        let (block, fnum) = choose_block_and_fnum(10_000.0).unwrap();
        assert_eq!(block, 7);
        assert!((1..=1023).contains(&fnum), "fnum out of range: {fnum}");
    }

    #[test]
    fn block_and_fnum_rejects_above_max() {
        // BLOCK=7, F-num=1023 represents the highest tone OPL3 can encode
        // under the empirical calibration (~12.4 kHz). A frequency well
        // past that must error out. 49716 Hz at BLOCK=7 yields fnum=4096
        // which overflows the 10-bit field.
        assert!(choose_block_and_fnum(49716.0).is_err());
        assert!(choose_block_and_fnum(100_000.0).is_err());
        assert!(choose_block_and_fnum(1_000_000.0).is_err());
    }

    #[test]
    fn build_events_rejects_zero_block_overflow() {
        assert!(build_events(1_000_000.0, 1000).is_err());
    }

    #[test]
    fn csv_round_trip_parses() {
        let events = build_events(440.0, 1000).unwrap();
        let mut buf: Vec<u8> = Vec::new();
        write_trace(&mut buf, &events).unwrap();
        let text = String::from_utf8(buf).unwrap();
        for (lineno, line) in text.lines().enumerate() {
            if lineno < 2 {
                continue; // skip magic + header
            }
            let cols: Vec<&str> = line.split(',').collect();
            assert_eq!(cols.len(), 4, "line {lineno}: bad column count: {line}");

            // Column 0: u64 timestamp.
            cols[0].parse::<u64>().unwrap_or_else(|_| {
                eprintln!("error: line {lineno}: ns parse: {}", cols[0]);
                std::process::exit(2);
            });

            // Column 1: 0x prefix, lowercase hex, exactly 3 digits.
            let port = cols[1];
            assert!(port.starts_with("0x"), "line {lineno}: port no 0x: {port}");
            let port_digits = &port[2..];
            assert_eq!(port_digits.len(), 3, "line {lineno}: port digits: {port}");
            assert!(
                port_digits
                    .chars()
                    .all(|c| c.is_ascii_digit() || ('a'..='f').contains(&c)),
                "line {lineno}: port not lowercase hex: {port}"
            );

            // Column 3: kind.
            let kind = cols[3];
            assert!(
                matches!(kind, "write8" | "write16" | "read8" | "read16"),
                "line {lineno}: bad kind: {kind}"
            );

            // Column 2: 0x prefix, lowercase hex, width matches kind.
            let value = cols[2];
            assert!(
                value.starts_with("0x"),
                "line {lineno}: value no 0x: {value}"
            );
            let value_digits = &value[2..];
            let expected_width = match kind {
                "write8" | "read8" => 2,
                "write16" | "read16" => 4,
                _ => unreachable!(),
            };
            assert_eq!(
                value_digits.len(),
                expected_width,
                "line {lineno}: value width: {value} (kind {kind})"
            );
            assert!(
                value_digits
                    .chars()
                    .all(|c| c.is_ascii_digit() || ('a'..='f').contains(&c)),
                "line {lineno}: value not lowercase hex: {value}"
            );
        }
    }
}

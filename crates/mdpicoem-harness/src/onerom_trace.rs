//! Shared parser for `onerom_*.trace` oracle files.
//!
//! Historical context: `onerom_pio_diff_rp2350.rs` carried a rich parser
//! (instrs + per-SM regs + body rows), and `onerom_snapshot_fmt.rs`
//! grew its own stripped-down copy that only cared about the instr
//! section. That duplication is consolidated here.
//!
//! Grammar (whitespace-separated, `#` comments):
//!
//! ```text
//! instr <block> <count> <hex_word>...
//! reg   <block> <sm> <clkdiv> <execctrl> <shiftctrl> <pinctrl>
//! <cycle> <input_drive> <input_level> <out_drive> <out_level>     # body row
//! ```
//!
//! All hex fields may be written with or without a `0x` prefix.

use std::path::Path;

/// Per-SM register row from a `reg` line.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SmReg {
    pub clkdiv: u32,
    pub execctrl: u32,
    pub shiftctrl: u32,
    pub pinctrl: u32,
}

/// One body row of the trace.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BodyRow {
    pub cycle: u32,
    pub input_drive: u32,
    pub input_level: u32,
    pub out_drive: u32,
    pub out_level: u32,
}

/// Full parsed trace.
#[derive(Default, Debug)]
pub struct Trace {
    /// Program words in load order (currently a single block concatenation —
    /// matches the existing trace format).
    pub instrs: Vec<u16>,
    /// Per-SM register configuration (index = sm number; slots unwritten
    /// by the trace remain `SmReg::default()`).
    pub regs: [SmReg; 4],
    /// Cycle-by-cycle pin drive/level observations.
    pub body: Vec<BodyRow>,
}

/// Parse a trace file from disk.
///
/// Returns a string error (rather than `io::Error`) to preserve the
/// existing callers' error-handling surface.
pub fn parse_trace(path: &Path) -> Result<Trace, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("read {}: {}", path.display(), e))?;
    parse_trace_str(&text)
}

/// Parse a trace from an in-memory string. Separated so tests can
/// pin down parser behaviour without touching the filesystem.
pub fn parse_trace_str(text: &str) -> Result<Trace, String> {
    let mut trace = Trace::default();
    for (lineno, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(rest) = line.strip_prefix("instr ") {
            parse_instr(rest, &mut trace)
                .map_err(|e| format!("line {}: {}", lineno + 1, e))?;
        } else if let Some(rest) = line.strip_prefix("reg ") {
            parse_reg(rest, &mut trace)
                .map_err(|e| format!("line {}: {}", lineno + 1, e))?;
        } else {
            parse_body_row(line, &mut trace)
                .map_err(|e| format!("line {}: {}", lineno + 1, e))?;
        }
    }
    Ok(trace)
}

/// Convenience helper: parse and return just the `instr` words.
/// Used by callers that only care about bytecode identity.
pub fn instrs_only(path: &Path) -> std::io::Result<Vec<u16>> {
    let text = std::fs::read_to_string(path)?;
    match parse_trace_str(&text) {
        Ok(t) => Ok(t.instrs),
        Err(e) => Err(std::io::Error::new(std::io::ErrorKind::InvalidData, e)),
    }
}

fn parse_instr(rest: &str, trace: &mut Trace) -> Result<(), String> {
    let mut toks = rest.split_ascii_whitespace();
    let _block: u8 = next_num(&mut toks, "block")?;
    let count: u32 = next_num(&mut toks, "count")?;
    for _ in 0..count {
        let word = toks.next().ok_or("missing instr word")?;
        let v = u16::from_str_radix(word.trim_start_matches("0x"), 16)
            .map_err(|_| format!("bad instr hex: {}", word))?;
        trace.instrs.push(v);
    }
    Ok(())
}

fn parse_reg(rest: &str, trace: &mut Trace) -> Result<(), String> {
    let mut toks = rest.split_ascii_whitespace();
    let _block: u8 = next_num(&mut toks, "block")?;
    let sm: u8 = next_num(&mut toks, "sm")?;
    if sm >= 4 {
        return Err(format!("sm out of range: {}", sm));
    }
    let clkdiv = next_hex(&mut toks, "clkdiv")?;
    let execctrl = next_hex(&mut toks, "execctrl")?;
    let shiftctrl = next_hex(&mut toks, "shiftctrl")?;
    let pinctrl = next_hex(&mut toks, "pinctrl")?;
    trace.regs[sm as usize] = SmReg { clkdiv, execctrl, shiftctrl, pinctrl };
    Ok(())
}

fn parse_body_row(line: &str, trace: &mut Trace) -> Result<(), String> {
    let mut toks = line.split_ascii_whitespace();
    let cycle: u32 = next_num(&mut toks, "cycle")?;
    let input_drive = next_hex(&mut toks, "input_drive")?;
    let input_level = next_hex(&mut toks, "input_level")?;
    let out_drive = next_hex(&mut toks, "out_drive")?;
    let out_level = next_hex(&mut toks, "out_level")?;
    trace.body.push(BodyRow { cycle, input_drive, input_level, out_drive, out_level });
    Ok(())
}

fn next_num<'a, T: std::str::FromStr>(
    toks: &mut impl Iterator<Item = &'a str>,
    what: &str,
) -> Result<T, String> {
    let s = toks.next().ok_or_else(|| format!("missing {}", what))?;
    s.parse::<T>().map_err(|_| format!("bad {} number: {}", what, s))
}

fn next_hex<'a>(
    toks: &mut impl Iterator<Item = &'a str>,
    what: &str,
) -> Result<u32, String> {
    let s = toks.next().ok_or_else(|| format!("missing {}", what))?;
    u32::from_str_radix(s.trim_start_matches("0x"), 16)
        .map_err(|_| format!("bad {} hex: {}", what, s))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_instr_and_reg_and_body() {
        let text = "\
# comment line
instr 1 2 0x1234 0xABCD
reg 1 0 0x00010000 0x00020000 0x00030000 0x00040000
0 0xAA 0x55 0xFF 0x00
1 0x00 0x00 0x01 0x01
";
        let t = parse_trace_str(text).unwrap();
        assert_eq!(t.instrs, vec![0x1234, 0xABCD]);
        assert_eq!(t.regs[0], SmReg {
            clkdiv: 0x0001_0000,
            execctrl: 0x0002_0000,
            shiftctrl: 0x0003_0000,
            pinctrl: 0x0004_0000,
        });
        assert_eq!(t.body.len(), 2);
        assert_eq!(t.body[0].cycle, 0);
        assert_eq!(t.body[1].out_level, 0x01);
    }

    #[test]
    fn rejects_bad_hex() {
        let err = parse_trace_str("instr 0 1 0xNOTHEX").unwrap_err();
        assert!(err.contains("bad instr hex"), "got: {}", err);
    }

    #[test]
    fn rejects_out_of_range_sm() {
        let err = parse_trace_str("reg 0 5 0x0 0x0 0x0 0x0").unwrap_err();
        assert!(err.contains("sm out of range"), "got: {}", err);
    }
}

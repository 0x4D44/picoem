//! OneROM snapshot formatter + oracle-branch decision (F.3).
//!
//! Produces two views of a [`SyncReport`]:
//!
//! 1. **Trace-shaped block** — mirrors `crates/mdpicoem-harness/oracles/onerom_2364.trace`
//!    line-for-line so a visual `diff` against the committed trace shows
//!    instantly whether the bytecode / per-SM regs diverge.
//! 2. **Human-readable table** — one row per SM, columns covering the
//!    readback-safe register set plus `ADDR`, `LAST_INSN`, and
//!    `enabled`. For eyes, not for diffs.
//!
//! The oracle-branch decision replays the HLD §4 tree:
//! - Shape match (PIO1 SM0 enabled; PIO2 SM0+SM1 enabled)?
//!   - Yes → bytecode match against oracle's `instr` words?
//!     - Yes → `A_CycleAccurate` (reuse committed trace).
//!     - No  → `B_SmokeTest` (architecture matches, bytecode differs).
//!   - No → `C_ArchitectureDiffers`.
//!
//! Design: `wrk_docs/2026.04.14 - HLD - OneROM Full-System Oracle.md` §4.

use std::fmt::Write as _;
use std::path::Path;

use crate::onerom_sync::{PioSnapshot, SmSnapshot, SyncReport};
use crate::onerom_trace;

/// Which oracle strategy the observed state lands on. Variant names map
/// directly onto HLD §4 branches A / B / C.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OracleBranch {
    /// Branch A: shape matches and bytecode matches — the committed
    /// trace applies. Run F.4 as a strict cycle-by-cycle diff.
    CycleAccurate,
    /// Branch B: shape matches, bytecode differs — use the
    /// timing-envelope smoke test from `piorom.c` (data appears within
    /// 11–14 cycles of CS, etc.).
    SmokeTest,
    /// Branch C: shape itself differs from `piorom.c`'s three-SM +
    /// two-block design. Re-design the harness around whatever
    /// structure we see.
    ArchitectureDiffers,
}

/// Render a snapshot in trace-file + table form.
pub fn format_snapshot(report: &SyncReport) -> String {
    let mut out = String::new();
    format_trace_block(&mut out, report);
    out.push('\n');
    format_table(&mut out, report);
    out
}

/// Trace-file-shaped view — matches the oracle format exactly for
/// visual diffing.
fn format_trace_block(out: &mut String, report: &SyncReport) {
    writeln!(out, "# trace-shaped snapshot at cycle {}", report.cycle).unwrap();
    for pio in [&report.pio0, &report.pio1, &report.pio2] {
        emit_instr_line(out, pio);
    }
    for pio in [&report.pio0, &report.pio1, &report.pio2] {
        emit_reg_lines(out, pio);
    }
}

/// Emit the `instr <block> <count> <hex words>...` line for blocks whose
/// program memory contains at least one non-zero word.
fn emit_instr_line(out: &mut String, pio: &PioSnapshot) {
    let count = program_len(&pio.instr_mem);
    if count == 0 {
        return;
    }
    write!(out, "instr {} {}", pio.block, count).unwrap();
    for i in 0..count {
        write!(out, " 0x{:04X}", pio.instr_mem[i]).unwrap();
    }
    out.push('\n');
}

/// Emit `reg <block> <sm> 0x<clkdiv> 0x<execctrl> 0x<shiftctrl> 0x<pinctrl>`
/// lines for enabled SMs (SM enable bits stored in `pio.ctrl & 0xF`).
fn emit_reg_lines(out: &mut String, pio: &PioSnapshot) {
    let sm_enable = pio.ctrl & 0xF;
    for sm in 0..4 {
        if (sm_enable >> sm) & 1 == 0 {
            continue;
        }
        let s = &pio.sms[sm];
        writeln!(
            out,
            "reg {} {} 0x{:08X} 0x{:08X} 0x{:08X} 0x{:08X}",
            pio.block, sm, s.clkdiv, s.execctrl, s.shiftctrl, s.pinctrl,
        )
        .unwrap();
    }
}

/// Program length = index of the last non-zero `instr_mem` entry + 1, or 0
/// if the whole table is zero. Matches how `parse_instr` in the diff harness
/// serialises program memory.
fn program_len(instr_mem: &[u16; 32]) -> usize {
    for i in (0..32).rev() {
        if instr_mem[i] != 0 {
            return i + 1;
        }
    }
    0
}

/// Wide, human-oriented table. One row per SM across all three blocks.
fn format_table(out: &mut String, report: &SyncReport) {
    writeln!(
        out,
        "block sm CLKDIV     EXECCTRL   SHIFTCTRL  PINCTRL    ADDR       LAST_INSN  enabled"
    )
    .unwrap();
    for pio in [&report.pio0, &report.pio1, &report.pio2] {
        let sm_enable = pio.ctrl & 0xF;
        for sm in 0..4usize {
            let enabled = (sm_enable >> sm) & 1 != 0;
            let s = &pio.sms[sm];
            emit_table_row(out, s, enabled);
        }
    }
}

fn emit_table_row(out: &mut String, s: &SmSnapshot, enabled: bool) {
    writeln!(
        out,
        "{:>5} {:>2} 0x{:08X} 0x{:08X} 0x{:08X} 0x{:08X} 0x{:08X} 0x{:08X} {}",
        s.block, s.sm, s.clkdiv, s.execctrl, s.shiftctrl, s.pinctrl, s.addr, s.last_insn, enabled
    )
    .unwrap();
}

/// Decide which oracle strategy the observed state lands on.
///
/// Reads and parses `oracle_path` (same format as `onerom_2364.trace`), then
/// walks the §4 decision tree. The returned `String` is a short
/// human-readable reason describing *why* the branch was chosen.
pub fn decide_oracle_branch(
    report: &SyncReport,
    oracle_path: &Path,
) -> (OracleBranch, String) {
    let oracle_instrs = match onerom_trace::instrs_only(oracle_path) {
        Ok(v) => v,
        Err(e) => {
            return (
                OracleBranch::SmokeTest,
                format!("oracle parse failed ({}); defaulting to smoke test", e),
            );
        }
    };

    // Shape check: PIO1 has exactly SM0 enabled; PIO2 has SM0+SM1 enabled.
    let shape_ok = (report.pio1.ctrl & 0xF) == 0b0001 && (report.pio2.ctrl & 0xF) == 0b0011;
    if !shape_ok {
        return (
            OracleBranch::ArchitectureDiffers,
            format!(
                "PIO1.CTRL=0x{:X} PIO2.CTRL=0x{:X} does not match piorom.c shape \
                 (expected PIO1=0b0001, PIO2=0b0011)",
                report.pio1.ctrl & 0xF,
                report.pio2.ctrl & 0xF,
            ),
        );
    }

    // Bytecode check: concat(PIO1[0..len1], PIO2[0..len2]) vs oracle.instrs.
    let len1 = program_len(&report.pio1.instr_mem);
    let len2 = program_len(&report.pio2.instr_mem);
    let mut ours: Vec<u16> = Vec::with_capacity(len1 + len2);
    ours.extend_from_slice(&report.pio1.instr_mem[..len1]);
    ours.extend_from_slice(&report.pio2.instr_mem[..len2]);

    if ours == oracle_instrs {
        (
            OracleBranch::CycleAccurate,
            format!(
                "bytecode matches oracle ({} words); reuse committed trace",
                ours.len()
            ),
        )
    } else {
        (
            OracleBranch::SmokeTest,
            format!(
                "bytecode differs (ours={} words, oracle={} words); \
                 architecture matches piorom.c shape so smoke test applies",
                ours.len(),
                oracle_instrs.len()
            ),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::onerom_sync::{PioSnapshot, SmSnapshot, SyncReport};
    use std::path::PathBuf;

    fn empty_pio(block: u8) -> PioSnapshot {
        PioSnapshot {
            block,
            ctrl: 0,
            instr_mem: [0; 32],
            sms: [SmSnapshot::default(); 4],
            dbg_padout: 0,
            dbg_padoe: 0,
        }
    }

    fn synthetic_report() -> SyncReport {
        let mut pio1 = empty_pio(1);
        pio1.ctrl = 0b0001;
        let mut pio2 = empty_pio(2);
        pio2.ctrl = 0b0011;
        SyncReport {
            cycle: 7069,
            pio0: empty_pio(0),
            pio1,
            pio2,
        }
    }

    fn oracle_path() -> PathBuf {
        // `cargo test` sets CWD to the crate root (`crates/mdpicoem-harness`),
        // so the oracle path is relative to that — not the workspace root.
        // Use CARGO_MANIFEST_DIR to stay robust against cwd drift.
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        manifest.join("oracles").join("onerom_2364.trace")
    }

    #[test]
    fn format_snapshot_includes_trace_and_table() {
        let mut report = synthetic_report();
        report.pio1.instr_mem[0] = 0x1234;
        report.pio1.sms[0].clkdiv = 0x1_0000;
        let out = format_snapshot(&report);
        assert!(out.contains("instr 1 1 0x1234"), "trace line missing: {}", out);
        assert!(out.contains("reg 1 0 0x00010000"), "reg line missing: {}", out);
        assert!(out.contains("block sm CLKDIV"), "table header missing: {}", out);
    }

    /// Synthetic report with correct shape (PIO1=0b0001, PIO2=0b0011) but
    /// empty INSTR_MEMs → bytecode does not match the oracle. Must pick
    /// `B_SmokeTest` with a reason that mentions "bytecode" or "differs".
    #[test]
    fn decide_oracle_branch_flags_bytecode_diff() {
        let report = synthetic_report();
        let (branch, reason) = decide_oracle_branch(&report, &oracle_path());
        assert_eq!(branch, OracleBranch::SmokeTest, "reason={}", reason);
        let low = reason.to_ascii_lowercase();
        assert!(
            low.contains("bytecode") || low.contains("differs"),
            "reason should mention bytecode/differs, got: {}",
            reason
        );
    }

    /// Shape mismatch → `ArchitectureDiffers`.
    #[test]
    fn decide_oracle_branch_flags_shape_diff() {
        let mut report = synthetic_report();
        report.pio1.ctrl = 0b0011; // PIO1 has two SMs instead of one
        let (branch, _reason) = decide_oracle_branch(&report, &oracle_path());
        assert_eq!(branch, OracleBranch::ArchitectureDiffers);
    }
}

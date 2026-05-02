//! Static instruction cycle table panel.
//!
//! Displays a hardcoded summary of Cortex-M0+ instruction classes and
//! their measured cycle counts. The data is purely static — no inputs
//! besides the `Frame` and `Rect` — so this panel is safe to render
//! alongside any firmware.
//!
//! Cycle counts come from the per-instruction unit tests in
//! `crates/rp2040_emu/src/tests.rs`, which encode the measured M0+ cost
//! for each instruction. M0+ is simpler than M33 — no divide, no
//! dual-issue, and the Thumb-32 subset is limited to BL / DSB / DMB /
//! ISB / MRS / MSR.

use ratatui::Frame;
use ratatui::layout::Rect;
use ratatui::text::Line;
use ratatui::widgets::{Block, Borders, Paragraph};

/// Three rows × two columns. The columns are hand-picked instruction
/// classes that cover the main M0+ cycle categories (1-cycle ALU,
/// 2-cycle load, multi-cycle LDM/STM, 32-bit BL, variable-cycle branch).
/// Keep it small — the panel is a glanceable cheat sheet, not a datasheet.
const ROWS: &[(&str, &str, &str, &str)] = &[
    ("ADDS Rd,Rn,Rm", "1", "MULS Rd,Rm", "1"),
    ("LDR Rt,[Rn,#i]", "2", "LDM 8 regs", "9"),
    ("B label", "1-3", "BL label", "4"),
];

const COL_LEFT_OP: usize = 16;
const COL_LEFT_CY: usize = 5;
const COL_RIGHT_OP: usize = 14;
const COL_RIGHT_CY: usize = 5;

pub fn draw_isa(area: Rect, f: &mut Frame) {
    let block = Block::default()
        .borders(Borders::ALL)
        .title("isa cycle table");

    let lines: Vec<Line<'static>> = ROWS.iter().map(|row| Line::from(format_row(row))).collect();

    f.render_widget(Paragraph::new(lines).block(block), area);
}

fn format_row(row: &(&str, &str, &str, &str)) -> String {
    let (left_op, left_cy, right_op, right_cy) = *row;
    format!(
        "{lop:<lop_w$}{lcy:<lcy_w$}{rop:<rop_w$}{rcy:<rcy_w$}",
        lop = left_op,
        lcy = left_cy,
        rop = right_op,
        rcy = right_cy,
        lop_w = COL_LEFT_OP,
        lcy_w = COL_LEFT_CY,
        rop_w = COL_RIGHT_OP,
        rcy_w = COL_RIGHT_CY,
    )
}

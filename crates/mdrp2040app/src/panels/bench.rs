//! Benchmark comparison panel.
//!
//! Draws the section table from [`Snapshot::benchmark`] with a color per
//! row: green for Δ == 0, red for Δ != 0, yellow for the stall indicator.
//!
//! While the firmware is still running sections, the panel shows the
//! rows that have completed so far plus a trailing "waiting..." line.
//! Before any sections complete, the panel shows only "waiting...".

use ratatui::Frame;
use ratatui::layout::Rect;
use ratatui::style::{Color, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph};

use crate::snapshot::{BenchmarkReport, BenchmarkSection, Snapshot};

const COL_NAME: usize = 14;
const COL_ITERS: usize = 10;
const COL_REAL: usize = 10;
const COL_EMU: usize = 10;
const COL_DELTA: usize = 10;

pub fn draw_bench(area: Rect, f: &mut Frame, snap: &Snapshot) {
    let block = Block::default().borders(Borders::ALL).title("benchmark");

    let lines: Vec<Line<'static>> = match &snap.benchmark {
        None => vec![Line::from(Span::styled(
            "waiting...".to_string(),
            Style::default().fg(Color::Yellow),
        ))],
        Some(report) => build_lines(report),
    };

    f.render_widget(Paragraph::new(lines).block(block), area);
}

fn build_lines(report: &BenchmarkReport) -> Vec<Line<'static>> {
    let mut lines: Vec<Line<'static>> = Vec::with_capacity(report.sections.len() + 2);

    lines.push(Line::from(header_row()));

    for section in &report.sections {
        lines.push(Line::from(Span::styled(
            section_row(section),
            row_style(section),
        )));
    }

    if let Some(phase) = report.stall {
        // Stall surfaces in the panel as "stalled at phase 0xNN after Ns".
        lines.push(Line::from(Span::styled(
            format!("stalled at phase {:#04x} after 10s", phase),
            Style::default().fg(Color::Yellow),
        )));
    } else if !report.complete {
        lines.push(Line::from(Span::styled(
            "waiting...".to_string(),
            Style::default().fg(Color::Yellow),
        )));
    }

    lines
}

fn header_row() -> String {
    format!(
        "{name:<cn$}{iters:>ci$}{real:>cr$}{emu:>ce$}{delta:>cd$}",
        name = "section",
        iters = "iters",
        real = "real",
        emu = "emu",
        delta = "delta",
        cn = COL_NAME,
        ci = COL_ITERS,
        cr = COL_REAL,
        ce = COL_EMU,
        cd = COL_DELTA,
    )
}

fn section_row(s: &BenchmarkSection) -> String {
    let delta = (s.emu_cycles as i128) - (s.ref_cycles as i128);
    format!(
        "{name:<cn$}{iters:>ci$}{real:>cr$}{emu:>ce$}{delta:>+cd$}",
        name = s.name,
        iters = s.iterations,
        real = s.ref_cycles,
        emu = s.emu_cycles,
        delta = delta,
        cn = COL_NAME,
        ci = COL_ITERS,
        cr = COL_REAL,
        ce = COL_EMU,
        cd = COL_DELTA,
    )
}

fn row_style(s: &BenchmarkSection) -> Style {
    if s.emu_cycles == s.ref_cycles {
        Style::default().fg(Color::Green)
    } else {
        Style::default().fg(Color::Red)
    }
}

use ratatui::Frame;
use ratatui::layout::Rect;
use ratatui::text::Line;
use ratatui::widgets::{Block, Borders, Paragraph};

use crate::snapshot::Snapshot;

pub fn draw_status(area: Rect, f: &mut Frame, snap: &Snapshot) {
    let total_secs = snap.wall_ms / 1000;
    let hh = total_secs / 3600;
    let mm = (total_secs % 3600) / 60;
    let ss = total_secs % 60;

    let lines = vec![
        Line::from(format!("cycles: {}", snap.cycles)),
        Line::from(format!("wall:   {:02}:{:02}:{:02}", hh, mm, ss)),
        Line::from(format!("MHz:    {:.1}", snap.effective_mhz)),
        Line::from(format!("PC:     {:#010x}", snap.pc)),
    ];

    let block = Block::default().borders(Borders::ALL).title("status");
    f.render_widget(Paragraph::new(lines).block(block), area);
}

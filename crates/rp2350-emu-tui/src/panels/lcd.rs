use ratatui::Frame;
use ratatui::layout::Rect;
use ratatui::text::Line;
use ratatui::widgets::{Block, Borders, Paragraph};

use crate::snapshot::Snapshot;

pub fn draw_lcd(area: Rect, f: &mut Frame, snap: &Snapshot) {
    let row0 = row_to_display_string(&snap.lcd.rows[0]);
    let row1 = row_to_display_string(&snap.lcd.rows[1]);

    let lines = vec![
        Line::from("+--------------------+"),
        Line::from(format!("|{}|", row0)),
        Line::from(format!("|{}|", row1)),
        Line::from("+--------------------+"),
        Line::from(format!(
            "cursor: ({:>2}, {})",
            snap.lcd.cursor.0, snap.lcd.cursor.1
        )),
    ];

    let block = Block::default().borders(Borders::ALL).title("lcd 20x2");
    f.render_widget(Paragraph::new(lines).block(block), area);
}

fn row_to_display_string(row: &[u8; 20]) -> String {
    row.iter()
        .map(|&b| {
            if (0x20..=0x7E).contains(&b) {
                b as char
            } else {
                ' '
            }
        })
        .collect()
}

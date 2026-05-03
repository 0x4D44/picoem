use ratatui::Frame;
use ratatui::layout::Rect;
use ratatui::text::Line;
use ratatui::widgets::{Block, Borders, Paragraph};

use crate::snapshot::Snapshot;

const LED_BIT: u32 = 1 << 25;

pub fn draw_gpio(area: Rect, f: &mut Frame, snap: &Snapshot) {
    let led_on = snap.gpio_out & LED_BIT != 0;
    // Filled circle for on, hollow circle for off.
    let led_char = if led_on { '●' } else { '○' };

    let lines = vec![
        Line::from(format!("GPIO_OUT: {:#010x}", snap.gpio_out)),
        Line::from(format!("GPIO_OE:  {:#010x}", snap.gpio_oe)),
        Line::from(""),
        Line::from(format!("LED 25: {}", led_char)),
    ];

    let block = Block::default().borders(Borders::ALL).title("gpio / led");
    f.render_widget(Paragraph::new(lines).block(block), area);
}

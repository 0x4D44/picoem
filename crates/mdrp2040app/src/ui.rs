use std::io::stdout;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyModifiers};
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::widgets::{Block, Borders};
use ratatui::{Frame, Terminal};

use crate::panels::bench::draw_bench;
use crate::panels::gpio::draw_gpio;
use crate::panels::isa::draw_isa;
use crate::panels::lcd::draw_lcd;
use crate::panels::status::draw_status;
use crate::snapshot::Snapshot;

pub fn run(
    snapshot: Arc<RwLock<Snapshot>>,
    shutdown: Arc<AtomicBool>,
    sim_handle: &JoinHandle<anyhow::Result<()>>,
) -> anyhow::Result<()> {
    let frame_period = Duration::from_millis(33);
    let mut next_frame = Instant::now() + frame_period;

    let mut terminal = Terminal::new(CrosstermBackend::new(stdout()))?;

    loop {
        if sim_handle.is_finished() {
            shutdown.store(true, Ordering::Relaxed);
            break;
        }

        let snap = snapshot.read().unwrap_or_else(|e| e.into_inner()).clone();
        terminal.draw(|f| draw_dashboard(f, &snap))?;

        let budget = next_frame.saturating_duration_since(Instant::now());
        if event::poll(budget)?
            && let Event::Key(k) = event::read()?
        {
            let ctrl_c =
                k.code == KeyCode::Char('c') && k.modifiers.contains(KeyModifiers::CONTROL);
            if k.code == KeyCode::Char('q') || ctrl_c {
                shutdown.store(true, Ordering::Relaxed);
                break;
            }
        }
        next_frame += frame_period;
    }

    Ok(())
}

fn draw_dashboard(f: &mut Frame, snap: &Snapshot) {
    let area = f.area();

    let outer = Block::default()
        .borders(Borders::ALL)
        .title("mdrp2040app showcase");
    let inner = outer.inner(area);
    f.render_widget(outer, area);

    // Vertical split: status row / LCD / bench / isa / help line.
    // Heights are tuned so all five panels fit in a standard 80x24
    // terminal while still leaving the bench panel room to grow with
    // the 6-section report.
    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(6),  // status + gpio row
            Constraint::Length(7),  // LCD panel
            Constraint::Length(10), // benchmark panel
            Constraint::Length(5),  // isa cycle table
            Constraint::Min(0),
            Constraint::Length(1), // help line
        ])
        .split(inner);

    let top = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(40), Constraint::Percentage(60)])
        .split(rows[0]);

    draw_status(top[0], f, snap);
    draw_gpio(top[1], f, snap);
    draw_lcd(rows[1], f, snap);
    draw_bench(rows[2], f, snap);
    draw_isa(rows[3], f);

    draw_help(rows[5], f);
}

fn draw_help(area: Rect, f: &mut Frame) {
    use ratatui::text::Line;
    use ratatui::widgets::Paragraph;
    f.render_widget(Paragraph::new(Line::from("q: quit")), area);
}

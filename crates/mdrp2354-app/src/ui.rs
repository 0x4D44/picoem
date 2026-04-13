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

use crate::panels::gpio::draw_gpio;
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
        .title("mdrp2354 showcase");
    let inner = outer.inner(area);
    f.render_widget(outer, area);

    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(6),
            Constraint::Min(0),
            Constraint::Length(1),
        ])
        .split(inner);

    let top = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(40), Constraint::Percentage(60)])
        .split(rows[0]);

    draw_status(top[0], f, snap);
    draw_gpio(top[1], f, snap);

    draw_help(rows[2], f);
}

fn draw_help(area: Rect, f: &mut Frame) {
    use ratatui::text::Line;
    use ratatui::widgets::Paragraph;
    f.render_widget(Paragraph::new(Line::from("q: quit")), area);
}

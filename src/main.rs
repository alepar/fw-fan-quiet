//! Entry point: wires sampler -> channel -> TEA model -> view, plus telemetry
//! and logging. Milestone 1: live read-only monitoring dashboard.

mod event;
mod logging;
mod model;
mod ring;
mod sensors;
mod telemetry;
mod types;
mod ui;

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use clap::Parser;
use color_eyre::Result;
use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, unbounded};

use event::Event;
use model::Model;
use sensors::sampler::Sampler;
use telemetry::{Record, Telemetry};
use ui::view::view;

/// Root check: RAPL energy counter is root-readable only, and it's the first
/// sensor we cannot live without (actuators later are root-only too).
const RAPL_ENERGY_PATH: &str = "/sys/class/powercap/intel-rapl:0/energy_uj";

/// Redraw at least this often even when no events arrive.
const RECV_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Parser)]
#[command(
    name = "bazerame-fans",
    about = "Fan-noise-first power manager (M1: monitor)"
)]
struct Args {
    /// Directory for JSONL telemetry logs (falls back to `.` if unwritable).
    #[arg(long, default_value = "/var/lib/bazerame-fans/telemetry")]
    telemetry_dir: PathBuf,
    /// Directory for tracing logs (falls back to `.` if unwritable).
    #[arg(long, default_value = "/var/lib/bazerame-fans/log")]
    log_dir: PathBuf,
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();
    let _log_guard = logging::init(&args.log_dir);

    if let Err(e) = std::fs::File::open(RAPL_ENERGY_PATH) {
        eprintln!("bazerame-fans needs root for RAPL/actuators - run: sudo ./bazerame-fans");
        eprintln!("(cannot open {RAPL_ENERGY_PATH} for read: {e})");
        std::process::exit(1);
    }

    let mut telemetry = telemetry::open_with_fallback(&args.telemetry_dir, Path::new("."));
    match &telemetry {
        Some(t) => tracing::info!("telemetry log: {}", t.path().display()),
        None => tracing::warn!("telemetry disabled: no writable directory"),
    }

    let (ui_tx, ui_rx) = unbounded::<Event>();
    let shutdown = Arc::new(AtomicBool::new(false));
    // Controller joins as a second subscriber in Task 14.
    let sampler = Sampler::new_system().spawn(vec![ui_tx.clone()], Arc::clone(&shutdown));

    // try_init() also installs a terminal-restoring panic hook (0.28.1+; we're
    // on 0.30, verified in its src/init.rs), so a panic mid-draw cannot leave
    // the terminal raw. Unlike init() it returns Err instead of panicking when
    // there is no usable tty (e.g. stdin redirected).
    let init_result = ratatui::try_init();
    let result = match init_result {
        Ok(mut terminal) => {
            spawn_input_thread(ui_tx);
            let result = run(&mut terminal, &ui_rx, telemetry.as_mut());
            ratatui::restore();
            result
        }
        Err(e) => Err(color_eyre::eyre::eyre!(e).wrap_err("cannot initialize terminal UI")),
    };

    shutdown.store(true, Ordering::Relaxed);
    if let Some(t) = telemetry.as_mut() {
        t.flush();
    }
    if sampler.join().is_err() {
        tracing::error!("sampler thread panicked");
    }
    // The input thread stays blocked in crossterm::event::read() with no
    // portable way to interrupt it, so it is deliberately left detached;
    // process exit reclaims it (its send fails once ui_rx is dropped anyway).
    result
}

/// Event loop: draw, then wait (bounded) for the next event and fold it into
/// the model. Every sample is also mirrored to the telemetry log.
fn run(
    terminal: &mut ratatui::DefaultTerminal,
    rx: &Receiver<Event>,
    mut telemetry: Option<&mut Telemetry>,
) -> Result<()> {
    let mut model = Model::new();
    while model.running {
        terminal.draw(|f| view(&model, f))?;
        match rx.recv_timeout(RECV_TIMEOUT) {
            Ok(ev) => {
                if let (Event::Sample(s), Some(t)) = (&ev, telemetry.as_deref_mut()) {
                    t.log(&Record::Sample(s));
                }
                model.update(ev);
            }
            // Timeout is the redraw tick: loop around and draw again.
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => {
                tracing::error!("all event senders gone, exiting");
                break;
            }
        }
    }
    Ok(())
}

/// Forwards key events to the UI channel. Blocking read on a dedicated
/// thread; exits when the receiver is gone or the read fails.
fn spawn_input_thread(tx: Sender<Event>) {
    std::thread::Builder::new()
        .name("input".into())
        .spawn(move || {
            loop {
                match crossterm::event::read() {
                    Ok(crossterm::event::Event::Key(key)) => {
                        if tx.send(Event::Input(key)).is_err() {
                            return;
                        }
                    }
                    Ok(_) => {} // resize/mouse/etc: redraw happens on timeout anyway
                    Err(e) => {
                        tracing::warn!("input thread: read failed, exiting: {e}");
                        return;
                    }
                }
            }
        })
        .expect("failed to spawn input thread");
}

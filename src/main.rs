//! Entry point: wires sampler -> channel -> TEA model -> view, plus telemetry
//! and logging. Milestone 1: live read-only monitoring dashboard.

mod actuators;
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

use actuators::cmd::RealRunner;
use actuators::cpu::CpuActuator;
use actuators::gpu::GpuActuator;
use actuators::guard::RestoreGuard;
use actuators::smu_module::SmuModule;
use event::Event;
use model::Model;
use sensors::sampler::Sampler;
use telemetry::{Record, Telemetry};
use ui::view::view;

/// Root check: RAPL energy counter is root-readable only, and it's the first
/// sensor we cannot live without (actuators later are root-only too).
const RAPL_ENERGY_PATH: &str = "/sys/class/powercap/intel-rapl:0/energy_uj";

/// Redraw at least this often even when no events arrive. Load-bearing: the
/// input thread drops Resize events, so this timeout is what guarantees a
/// prompt redraw at the new size after a terminal resize.
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

    // Root check before logging::init: a non-root run should print the hint
    // and exit without leaving a stray fallback log file in the cwd.
    if let Err(e) = std::fs::File::open(RAPL_ENERGY_PATH) {
        eprintln!("bazerame-fans needs root for RAPL/actuators - run: sudo ./bazerame-fans");
        eprintln!("(cannot open {RAPL_ENERGY_PATH} for read: {e})");
        std::process::exit(1);
    }

    let _log_guard = logging::init(&args.log_dir);

    let mut telemetry = telemetry::open_with_fallback(&args.telemetry_dir, Path::new("."));
    match &telemetry {
        Some(t) => tracing::info!("telemetry log: {}", t.path().display()),
        None => tracing::warn!("telemetry disabled: no writable directory"),
    }

    // Actuators + restore guard, BEFORE ratatui init so a failed terminal
    // init still restores hardware. ensure_unloaded failure is only warned:
    // CPU actuation will then fail visibly later, monitoring still works.
    let smu = match SmuModule::ensure_unloaded(&RealRunner, Path::new("/sys/kernel")) {
        Ok(smu) => Some(smu),
        Err(e) => {
            tracing::warn!("ryzen_smu unload failed (CPU actuation will fail later): {e}");
            None
        }
    };
    let cpu = CpuActuator::new(
        RealRunner,
        PathBuf::from("/sys/firmware/acpi/platform_profile"),
    );
    let gpu = match GpuActuator::new() {
        Ok(gpu) => Some(gpu),
        Err(e) => {
            tracing::warn!("GPU actuator unavailable (no clock control this run): {e}");
            None
        }
    };
    // LOAD-BEARING: the guard lives on MAIN's stack, declared after _log_guard
    // (so its Drop still gets logged) and before everything else it must
    // outlive. A panic anywhere below unwinds through here and restores
    // hardware; a panic in another thread cannot run this drop, which is why
    // actuation stays reachable from main.
    // TODO(task-14): controller will borrow/own actuators; guard stays on
    // main's stack as the final safety net; coordination TBD in task 14.
    let mut guard = RestoreGuard::new(RealRunner, Some(cpu), gpu, smu);
    // Belt-and-suspenders against a previous SIGKILL'd run leaving locks set.
    guard.startup_reset();

    // SIGINT/SIGTERM/SIGHUP set term_flag; the event loop treats it like 'q',
    // so the process leaves through the normal return path and the guard
    // drops on main's stack. SIGHUP matters: closing the terminal window (or
    // the pty master dying) would otherwise kill us without running drops.
    // SIGKILL cannot be caught - startup_reset above covers it.
    let term_flag = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&term_flag))?;
    signal_hook::flag::register(signal_hook::consts::SIGTERM, Arc::clone(&term_flag))?;
    signal_hook::flag::register(signal_hook::consts::SIGHUP, Arc::clone(&term_flag))?;

    let (ui_tx, ui_rx) = unbounded::<Event>();
    let shutdown = Arc::new(AtomicBool::new(false));
    // Controller joins as a second subscriber in Task 14.
    let sampler = Sampler::new_system().spawn(vec![ui_tx.clone()], Arc::clone(&shutdown));

    // try_init() returns Err instead of panicking when there is no usable
    // tty (e.g. stdin redirected). It installs a terminal-restoring panic
    // hook, which install_tty_safe_panic_hook() below replaces with a
    // dead-tty-safe equivalent.
    // ORDER (shutdown, normal path): 'q' or SIGINT/SIGTERM/SIGHUP stops the
    // event loop -> terminal restore -> sampler shutdown+join, telemetry
    // flush -> guard drops (GPU release -> CPU restore -> ryzen_smu reload)
    // -> _log_guard drops last so every restore step is still logged.
    // ORDER (panic path): the tty-safe panic hook restores the terminal
    // first, then main's unwind runs the same drops in the same order.
    let init_result = ratatui::try_init();
    let result = match init_result {
        Ok(mut terminal) => {
            install_tty_safe_panic_hook();
            spawn_input_thread(ui_tx);
            let result = run(&mut terminal, &ui_rx, telemetry.as_mut(), &term_flag);
            // try_restore, NOT restore(): restore() reports failure via
            // eprintln!, which itself panics when stderr is a dead tty (e.g.
            // the terminal hung up). Verified on-machine via pty-hangup repro.
            if let Err(e) = ratatui::try_restore() {
                tracing::warn!("terminal restore failed (harmless if the tty is gone): {e}");
            }
            // Skip Terminal's Drop: on a dead tty its show_cursor() fails and
            // it eprintln!s the error, which PANICS (dead stderr) and aborts
            // the process before the guard below can restore hardware
            // (observed as SIGABRT on-machine). We already restored the
            // screen above; try to show the cursor, then leak the handful of
            // buffer bytes - the process is exiting anyway.
            let _ = terminal.show_cursor();
            std::mem::forget(terminal);
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
    // The input thread stays blocked in crossterm::event::read(). A
    // poll(100ms)+shutdown-flag loop would let it exit cleanly, but detaching
    // is a deliberate simplicity choice: it owns nothing needing cleanup,
    // its send fails once ui_rx drops, and process exit reclaims it.
    result
}

/// Event loop: draw, then wait (bounded) for the next event and fold it into
/// the model. Every sample is also mirrored to the telemetry log. A raised
/// `term_flag` (SIGINT/SIGTERM/SIGHUP) is treated exactly like 'q': the loop
/// ends and shutdown proceeds through the normal return path.
fn run(
    terminal: &mut ratatui::DefaultTerminal,
    rx: &Receiver<Event>,
    mut telemetry: Option<&mut Telemetry>,
    term_flag: &AtomicBool,
) -> Result<()> {
    let mut model = Model::new();
    while model.running {
        if term_flag.load(Ordering::Relaxed) {
            tracing::info!("caught termination signal (SIGINT/SIGTERM/SIGHUP), shutting down");
            model.running = false;
            continue;
        }
        terminal.draw(|f| view(&model, f))?;
        match rx.recv_timeout(RECV_TIMEOUT) {
            Ok(first) => {
                // Coalesce bursts: fold everything already queued into the
                // model before spending a draw on it.
                let mut next = Some(first);
                while let Some(ev) = next {
                    if let (Event::Sample(s), Some(t)) = (&ev, telemetry.as_deref_mut()) {
                        t.log(&Record::Sample(s));
                    }
                    model.update(ev);
                    next = rx.try_recv().ok();
                }
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

/// Replaces the panic hook chain (color_eyre's, wrapped by ratatui's) with a
/// tty-safe one. ratatui's hook restores the terminal via `restore()`, which
/// reports failure with eprintln! - and eprintln! itself panics when stderr
/// is a dead tty (terminal hangup). A panic inside the panic hook aborts the
/// process instantly: no unwind, no RestoreGuard, hardware left dirty
/// (observed on-machine). This hook restores the terminal quietly, logs the
/// panic to the tracing file, and only best-effort-writes to stderr; the
/// hardware restore then happens during main's unwind.
fn install_tty_safe_panic_hook() {
    std::panic::set_hook(Box::new(|info| {
        let _ = ratatui::try_restore();
        let backtrace = std::backtrace::Backtrace::force_capture();
        tracing::error!("panic: {info}\n{backtrace}");
        use std::io::Write;
        let _ = writeln!(std::io::stderr(), "{info}\n{backtrace}");
    }));
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

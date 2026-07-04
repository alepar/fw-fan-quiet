//! Entry point: wires sampler -> channel -> TEA model -> view, plus telemetry
//! and logging. Milestone 1: live read-only monitoring dashboard.

mod actuators;
mod control;
mod event;
mod logging;
mod model;
mod ring;
mod sensors;
mod telemetry;
mod types;
mod ui;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use clap::Parser;
use color_eyre::Result;
use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, unbounded};

use actuators::cmd::RealRunner;
use actuators::cpu::{CpuActuator, PLATFORM_PROFILE_PATH};
use actuators::gpu::GpuActuator;
use actuators::guard::{FinalRestore, RestoreGuard};
use actuators::smu_module::SmuModule;
use control::Command;
use control::controller::{self, Controller};
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

    let telemetry = telemetry::open_with_fallback(&args.telemetry_dir, Path::new("."));
    match &telemetry {
        Some(t) => tracing::info!("telemetry log: {}", t.path().display()),
        None => tracing::warn!("telemetry disabled: no writable directory"),
    }
    // Shared with the controller thread: main logs samples, the controller
    // logs decisions. Lock scopes stay one-call tiny on both sides.
    let telemetry = Arc::new(Mutex::new(telemetry));

    // SIGINT/SIGTERM/SIGHUP set term_flag; the event loop treats it like 'q',
    // so the process leaves through the normal return path where the
    // controller is told to Quit and restores hardware before main returns.
    // SIGHUP matters: closing the terminal window (or
    // the pty master dying) would otherwise kill us without running drops.
    // Registered BEFORE any hardware is touched below: a signal arriving
    // mid-construction only sets the flag instead of terminating us without
    // drops. SIGKILL cannot be caught - startup_reset below covers it.
    let term_flag = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&term_flag))?;
    signal_hook::flag::register(signal_hook::consts::SIGTERM, Arc::clone(&term_flag))?;
    signal_hook::flag::register(signal_hook::consts::SIGHUP, Arc::clone(&term_flag))?;

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
    // Recorded BEFORE the smu handle moves into the guard: FinalRestore needs
    // it to rebuild the reload obligation on the panic path.
    let smu_was_unloaded = smu.as_ref().is_some_and(SmuModule::unloaded_by_us);
    let cpu = CpuActuator::new(RealRunner, PathBuf::from(PLATFORM_PROFILE_PATH));
    let gpu = match GpuActuator::new() {
        Ok(gpu) => Some(gpu),
        Err(e) => {
            tracing::warn!("GPU actuator unavailable (no clock control this run): {e}");
            None
        }
    };
    // Actuator ownership, two layers (resolves the Task-13 ownership TODO):
    // the WORKING actuators live in `guard`, which moves into the controller
    // thread below — the single place hardware writes happen. On clean
    // shutdown (Command::Quit or channel disconnect) the controller restores
    // and flips `restored`; main joins it. `_final_restore` stays LOAD-BEARING
    // on MAIN's stack for the paths where the controller never gets there
    // (e.g. a main-thread panic kills other threads without running their
    // drops, but unwinds its own stack): its Drop sees `restored == false`,
    // builds fresh short-lived actuators and reruns the same best-effort
    // restore. Restores are idempotent, so a double restore is harmless.
    // Declared after _log_guard so its Drop still gets logged.
    let restored = Arc::new(AtomicBool::new(false));
    let _final_restore = FinalRestore::new(Arc::clone(&restored), smu_was_unloaded);
    let mut guard = RestoreGuard::new(RealRunner, Some(cpu), gpu, smu);
    // Belt-and-suspenders against a previous SIGKILL'd run leaving locks set.
    guard.startup_reset();

    let (ui_tx, ui_rx) = unbounded::<Event>();
    let (ctl_sample_tx, ctl_sample_rx) = unbounded::<Event>();
    let (cmd_tx, cmd_rx) = unbounded::<Command>();
    let shutdown = Arc::new(AtomicBool::new(false));
    let sampler =
        Sampler::new_system().spawn(vec![ui_tx.clone(), ctl_sample_tx], Arc::clone(&shutdown));
    let ctl = controller::spawn(
        Controller::new(guard),
        ctl_sample_rx,
        cmd_rx,
        ui_tx.clone(),
        Arc::clone(&telemetry),
        Arc::clone(&restored),
    );

    // try_init() returns Err instead of panicking when there is no usable
    // tty (e.g. stdin redirected). It installs a terminal-restoring panic
    // hook, which install_tty_safe_panic_hook() below replaces with a
    // dead-tty-safe equivalent.
    // ORDER (shutdown, normal path): 'q' or SIGINT/SIGTERM/SIGHUP stops the
    // event loop -> terminal restore -> Command::Quit + controller join FIRST
    // (controller runs GPU release -> CPU restore -> ryzen_smu reload, then
    // flips `restored`) -> telemetry flush, sampler join -> _final_restore
    // drops as a no-op (restored is true) -> _log_guard drops last so every
    // restore step is still logged.
    // ORDER (panic path): the tty-safe panic hook restores the terminal
    // first, then main's unwind drops cmd_tx/ui_rx (the controller sees the
    // disconnect and restores from its thread) and _final_restore's Drop
    // covers the race where the process exits before the controller does.
    // Terminal::drop can never run on ANY path (normal or unwind): it is
    // wrapped in ManuallyDrop below, because on a dead tty its show_cursor()
    // fails and it eprintln!s the error, which PANICS (dead stderr) - during
    // unwind that is a double panic -> SIGABRT before any hardware restore
    // can run (observed on-machine as SIGABRT via coredumpctl).
    let init_result = ratatui::try_init();
    let result = match init_result {
        Ok(terminal) => {
            // ManuallyDrop, immediately: if run() panics, unwinding would
            // otherwise drop `terminal` (hidden_cursor is true after the
            // first draw) and abort as described above. DerefMut keeps the
            // &mut terminal calls below working; the handful of leaked
            // buffer bytes are reclaimed at process exit.
            let mut terminal = std::mem::ManuallyDrop::new(terminal);
            install_tty_safe_panic_hook();
            spawn_input_thread(ui_tx);
            let result = run(&mut terminal, &ui_rx, &telemetry, &term_flag);
            // try_restore, NOT restore(): restore() reports failure via
            // eprintln!, which itself panics when stderr is a dead tty (e.g.
            // the terminal hung up). Verified on-machine via pty-hangup repro.
            if let Err(e) = ratatui::try_restore() {
                tracing::warn!("terminal restore failed (harmless if the tty is gone): {e}");
            }
            // Best-effort replacement for the cursor restore Terminal::drop
            // would have done.
            let _ = terminal.show_cursor();
            result
        }
        Err(e) => Err(color_eyre::eyre::eyre!(e).wrap_err("cannot initialize terminal UI")),
    };

    shutdown.store(true, Ordering::Relaxed);
    // Controller FIRST: Quit makes it restore hardware; joining before any
    // other teardown guarantees stock state is back even if a later step
    // hangs. A send failure means the controller already exited (it restores
    // on channel disconnect too) — the join below still reaps it.
    if cmd_tx.send(Command::Quit).is_err() {
        tracing::warn!("controller already gone at shutdown");
    }
    if ctl.join().is_err() {
        tracing::error!("controller thread panicked");
    }
    if let Some(t) = telemetry::lock(&telemetry).as_mut() {
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
    telemetry: &Mutex<Option<Telemetry>>,
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
                    if let Event::Sample(s) = &ev {
                        if let Some(t) = telemetry::lock(telemetry).as_mut() {
                            t.log(&Record::Sample(s));
                        }
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

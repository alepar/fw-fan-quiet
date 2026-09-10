//! Entry point: wires sampler -> channel -> TEA model -> view, plus telemetry
//! and logging. Milestone 1: live read-only monitoring dashboard.

mod actuators;
mod calib;
mod config;
mod control;
mod event;
mod fanctrl;
#[cfg(test)]
mod integration_tests;
mod led;
mod logging;
mod model;
mod ring;
mod selftest;
mod sensors;
mod state;
mod sync_util;
mod telemetry;
#[cfg(test)]
mod test_support;
mod types;
mod ui;

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use color_eyre::Result;
use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, unbounded};

use actuators::cmd::RealRunner;
use actuators::cpu::{CpuActuator, PLATFORM_PROFILE_PATH};
use actuators::gpu::{BoxedGpu, GpuActuator};
use actuators::guard::{FinalRestore, RestoreGuard};
use actuators::smu_module::SmuModule;
use control::Command;
use control::controller::{self, Controller};
use event::Event;
use fanctrl::client::{FanctrlSource, UnixFanctrlClient};
use model::Model;
use sensors::hwmon::Hwmon;
use sensors::poller::{self, FanctrlPoller, SharedFanctrl, SharedNvme, spawn_nvme_poller};
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
    /// Config file (TOML); missing or invalid falls back to defaults.
    #[arg(long, default_value = "/etc/bazerame-fans/config.toml")]
    config: PathBuf,
    /// Persisted calibration state (JSON); missing means "not calibrated".
    #[arg(long, default_value = "/var/lib/bazerame-fans/state.json")]
    state_file: PathBuf,
    /// Default (no subcommand): the live TUI dashboard.
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    /// Hardware selftest: exercise actuators + sensors end to end and print
    /// plain [ OK ]/[FAIL] lines (~25 s; needs root; no TUI).
    Selftest,
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let args = Args::parse();

    // Selftest path: plain stdout report + stderr logging, never the TUI or
    // file logging. Runs before the root check below so the selftest prints
    // its own [FAIL] root-check line.
    if let Some(Commands::Selftest) = args.command {
        logging::init_stderr();
        std::process::exit(selftest::run());
    }

    // Root check before logging::init: a non-root run should print the hint
    // and exit without leaving a stray fallback log file in the cwd.
    if let Err(e) = std::fs::File::open(RAPL_ENERGY_PATH) {
        eprintln!("bazerame-fans needs root for RAPL/actuators - run: sudo ./bazerame-fans");
        eprintln!("(cannot open {RAPL_ENERGY_PATH} for read: {e})");
        std::process::exit(1);
    }

    let _log_guard = logging::init(&args.log_dir);

    // Loaded at startup so config/state problems surface in the log from day
    // one; the controller consumes the fan target, floors and fast limit
    // (and saves the fan target back on change).
    let config = config::Config::load(&args.config);
    // Persisted calibration (model + LUT) seeds the controller; a fresh
    // calibration run overwrites the file through the same path.
    let persisted = state::PersistedState::load(&args.state_file);

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
    let gpu: Option<BoxedGpu> = match GpuActuator::new() {
        Ok(gpu) => Some(Box::new(gpu)),
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
    // briefly waits out an in-flight controller restore (grace poll), then
    // builds fresh short-lived actuators and reruns the same best-effort
    // restore. A concurrent double restore is power-safe; worst case it
    // clobbers the user's platform-profile preference.
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

    // LED wattage display: opens the modules up front. Its sample sender only
    // joins the sampler fan-out when at least one module is live — otherwise a
    // dropped receiver would make the sampler treat it as a dead subscriber and
    // shut the whole pipeline down. The thread drains until the sampler drops
    // its sender at shutdown; joined after the sampler so that drop happens.
    let (led_sample_tx, led_sample_rx) = unbounded::<Event>();
    let led = led::spawn(
        config.leds.clone(),
        config.cpu_max_w,
        config.gpu_max_w,
        led_sample_rx,
    );
    // fw-fanctrl socket poller + NVMe poller (design doc §3.4 / Task 14):
    // each gets its own thread and owns its own source outright, publishing
    // into a small `Arc<Mutex<_>>` the sampler tick only ever reads -- see
    // `sensors::poller`'s module doc for why neither lives on the sampler
    // tick itself, and why the socket client is not what is shared.
    // Construction only, here: the cadence/merge logic is
    // `sensors::poller`'s and `Sampler`'s.
    let fanctrl_snapshot: SharedFanctrl = poller::shared_fanctrl();
    let fanctrl_client = Box::new(UnixFanctrlClient::new(config.fanctrl_socket.clone()))
        as Box<dyn FanctrlSource + Send>;
    let fanctrl_poller = FanctrlPoller::new(
        fanctrl_client,
        Arc::clone(&fanctrl_snapshot),
        Instant::now(),
    );
    let fanctrl_poller_thread = fanctrl_poller.spawn(Arc::clone(&shutdown));

    let nvme_hwmon = Hwmon::discover(Path::new("/sys/class/hwmon"));
    let nvme_cache: SharedNvme = Arc::new(Mutex::new(None));
    let nvme_poller_thread = spawn_nvme_poller(
        move || nvme_hwmon.nvme_composite_c(),
        Arc::clone(&nvme_cache),
        Arc::clone(&shutdown),
    );

    let mut sampler_txs = vec![ui_tx.clone(), ctl_sample_tx];
    if led.is_some() {
        sampler_txs.push(led_sample_tx);
    }
    let sampler =
        Sampler::new_system(fanctrl_snapshot, nvme_cache).spawn(sampler_txs, Arc::clone(&shutdown));
    let ctl = controller::spawn(
        Controller::new(guard, persisted, args.state_file, config, args.config),
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
    // event loop -> Command::Quit + controller join FIRST (GPU release ->
    // CPU restore -> ryzen_smu reload, then flips `restored`) -> terminal
    // restore -> telemetry flush, sampler join -> _final_restore drops as a
    // no-op (restored is true) -> _log_guard drops last so every restore
    // step is still logged. Hardware restore deliberately precedes ALL
    // terminal I/O: try_restore writes to stdout, and a frozen/SIGSTOP'd
    // terminal emulator with a full pty buffer can block those writes
    // indefinitely with main neither returning nor unwinding — neither the
    // disconnect path nor FinalRestore would fire. The tty staying in raw
    // mode ~400 ms longer is the accepted cost.
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
            let result = run(&mut terminal, &ui_rx, &cmd_tx, &telemetry, &term_flag);
            // Hardware restore BEFORE any terminal I/O (see ORDER above).
            shutdown.store(true, Ordering::Relaxed);
            quit_and_join_controller(cmd_tx, ctl);
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
        Err(e) => {
            shutdown.store(true, Ordering::Relaxed);
            quit_and_join_controller(cmd_tx, ctl);
            Err(color_eyre::eyre::eyre!(e).wrap_err("cannot initialize terminal UI"))
        }
    };

    if let Some(t) = telemetry::lock(&telemetry).as_mut() {
        t.flush();
    }
    if sampler.join().is_err() {
        tracing::error!("sampler thread panicked");
    }
    // The fanctrl/NVMe poller threads: `shutdown` is already set above, so
    // both are already exiting (or exited) by the time we get here; this is
    // just reaping them, same as the sampler join above.
    if fanctrl_poller_thread.join().is_err() {
        tracing::error!("fanctrl poller thread panicked");
    }
    if nvme_poller_thread.join().is_err() {
        tracing::error!("nvme poller thread panicked");
    }
    // After the sampler: it held the only live led_sample sender, so its exit
    // disconnects the LED channel, letting that thread blank the panels and
    // return. A no-op when the feature was inert (`led` is None).
    if let Some(led) = led {
        if led.join().is_err() {
            tracing::error!("led thread panicked");
        }
    }
    // The input thread stays blocked in crossterm::event::read(). A
    // poll(100ms)+shutdown-flag loop would let it exit cleanly, but detaching
    // is a deliberate simplicity choice: it owns nothing needing cleanup,
    // its send fails once ui_rx drops, and process exit reclaims it.
    result
}

/// How long shutdown waits for the controller to finish restoring hardware
/// before giving up on it and continuing (terminal restore in particular).
/// Generous: the restore itself is ~400 ms, and every external command it
/// runs is independently bounded by `actuators::cmd::RUN_TIMEOUT` (5 s), so
/// this only bites when the controller thread is wedged somewhere with no
/// timeout of its own.
const CONTROLLER_JOIN_TIMEOUT: Duration = Duration::from_secs(20);

/// Outcome of [`join_with_timeout`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JoinOutcome {
    Joined,
    Panicked,
    TimedOut,
}

/// `JoinHandle::join` with a deadline. `join` itself cannot be interrupted,
/// so the handle is moved into a helper thread that reports back over a
/// channel; on timeout the helper is simply left running (it owns nothing
/// but the handle and exits when the thread it is waiting on does).
fn join_with_timeout(handle: std::thread::JoinHandle<()>, timeout: Duration) -> JoinOutcome {
    let (done_tx, done_rx) = unbounded();
    // A spawn failure here must not itself be fatal: fall back to reporting
    // a timeout, which is the same "carry on with shutdown" behaviour.
    if std::thread::Builder::new()
        .name("join-waiter".to_string())
        .spawn(move || {
            let _ = done_tx.send(handle.join().is_ok());
        })
        .is_err()
    {
        return JoinOutcome::TimedOut;
    }
    match done_rx.recv_timeout(timeout) {
        Ok(true) => JoinOutcome::Joined,
        Ok(false) => JoinOutcome::Panicked,
        Err(_) => JoinOutcome::TimedOut,
    }
}

/// Restore hardware first: tell the controller to Quit and JOIN it, so stock
/// state is guaranteed back before any later teardown step (terminal I/O in
/// particular) gets a chance to block. A send failure means the controller
/// already exited (it restores on channel disconnect too); the join still
/// reaps it either way.
///
/// The join is BOUNDED ([`CONTROLLER_JOIN_TIMEOUT`]): hardware restore comes
/// first, but it may not come *forever*, or a wedged controller thread would
/// leave the terminal in raw mode with no cursor for as long as the process
/// lives. On expiry we log and continue; `FinalRestore`'s `Drop` and the
/// controller's own disconnect path are still there to restore the hardware.
fn quit_and_join_controller(cmd_tx: Sender<Command>, ctl: std::thread::JoinHandle<()>) {
    if cmd_tx.send(Command::Quit).is_err() {
        tracing::warn!("controller already gone at shutdown");
    }
    match join_with_timeout(ctl, CONTROLLER_JOIN_TIMEOUT) {
        JoinOutcome::Joined => {}
        JoinOutcome::Panicked => tracing::error!("controller thread panicked"),
        JoinOutcome::TimedOut => tracing::error!(
            "controller thread did not finish restoring within {CONTROLLER_JOIN_TIMEOUT:?}; \
             continuing shutdown so the terminal is restored"
        ),
    }
}

/// Event loop: draw, then wait (bounded) for the next event and fold it into
/// the model; commands the update returns (manual-mode keys) are forwarded to
/// the controller. Every sample is also mirrored to the telemetry log. A
/// raised `term_flag` (SIGINT/SIGTERM/SIGHUP) is treated exactly like 'q':
/// the loop ends and shutdown proceeds through the normal return path.
fn run(
    terminal: &mut ratatui::DefaultTerminal,
    rx: &Receiver<Event>,
    cmd_tx: &Sender<Command>,
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
                'events: while let Some(ev) = next {
                    if let Event::Sample(s) = &ev {
                        if let Some(t) = telemetry::lock(telemetry).as_mut() {
                            // `ec_ma` (design §3.5) is the controller's own
                            // live EC boxcar average, not part of `Sample`
                            // (Task 15) -- the model's echoed status is the
                            // freshest copy this loop has of it.
                            t.log(&Record::sample(s, model.status.ec_ma_c));
                        }
                    }
                    for c in model.update(ev) {
                        if cmd_tx.send(c).is_err() {
                            // Controller died: shut down cleanly (warn once;
                            // dropping the queued events is fine mid-exit).
                            tracing::warn!("controller command channel closed, shutting down");
                            model.running = false;
                            break 'events;
                        }
                    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_selftest_subcommand() {
        let args = Args::try_parse_from(["bazerame-fans", "selftest"]).unwrap();
        assert!(matches!(args.command, Some(Commands::Selftest)));
    }

    #[test]
    fn no_subcommand_means_tui() {
        let args = Args::try_parse_from(["bazerame-fans"]).unwrap();
        assert!(args.command.is_none());
    }

    #[test]
    fn flags_still_parse_alongside_subcommand() {
        let args =
            Args::try_parse_from(["bazerame-fans", "--log-dir", "/tmp/x", "selftest"]).unwrap();
        assert!(matches!(args.command, Some(Commands::Selftest)));
        assert_eq!(args.log_dir, PathBuf::from("/tmp/x"));
    }

    // --- finding 4: shutdown may never hang on the controller join --------

    #[test]
    fn join_with_timeout_reports_a_clean_join() {
        let h = std::thread::spawn(|| {});
        assert_eq!(
            join_with_timeout(h, Duration::from_secs(5)),
            JoinOutcome::Joined
        );
    }

    #[test]
    fn join_with_timeout_reports_a_panicking_thread() {
        let h = std::thread::spawn(|| panic!("controller blew up"));
        assert_eq!(
            join_with_timeout(h, Duration::from_secs(5)),
            JoinOutcome::Panicked
        );
    }

    /// The pin: a controller thread wedged in an untimed hardware call must
    /// NOT keep the terminal in raw mode forever. Before the bound, this
    /// blocked for the full lifetime of the stuck thread.
    #[test]
    fn join_with_timeout_gives_up_on_a_wedged_thread() {
        let release = Arc::new(AtomicBool::new(false));
        let held = Arc::clone(&release);
        let h = std::thread::spawn(move || {
            while !held.load(Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(5));
            }
        });
        let started = Instant::now();
        let outcome = join_with_timeout(h, Duration::from_millis(150));
        let elapsed = started.elapsed();
        release.store(true, Ordering::Relaxed);

        assert_eq!(outcome, JoinOutcome::TimedOut);
        assert!(
            elapsed < Duration::from_secs(5),
            "gave up only after {elapsed:?}; the join is not bounded"
        );
    }
}

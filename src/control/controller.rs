//! Controller: testable core (`Controller`) + thread shell (`spawn`).
//!
//! Ownership (resolves the Task-13 TODO): the controller thread OWNS the
//! working actuators via `RestoreGuard`. On clean shutdown (Command::Quit or
//! command-channel disconnect) it runs `restore_all` itself and flips the
//! shared `restored` flag; main joins it. Main's stack keeps a `FinalRestore`
//! guard as the panic-path safety net (see `actuators::guard`).
//!
//! The core is pure over the `Runner` seam: `on_sample`/`on_command` mutate
//! state and return `Effect`s describing what happened, so every behavior is
//! unit-testable with `FakeRunner` and hand-fed samples. The shell only maps
//! effects to channel sends and telemetry records.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender, never, select};

use crate::actuators::cmd::Runner;
use crate::actuators::guard::RestoreGuard;
use crate::calib::burner::Burner;
use crate::calib::runner::{CalibRunner, RunnerEffect};
use crate::control::lut::ClockWattsLut;
use crate::control::thermal_model::ThermalModel;
use crate::event::Event;
use crate::state::PersistedState;
use crate::telemetry::{self, Record, Telemetry};
use crate::types::Sample;

/// UI-facing calibration progress, re-exported so the view/model layers name
/// it without reaching into `calib::`.
pub use crate::calib::runner::CalibProgress as CalibProgressLite;

/// Reapply active limits at least this often (defends against PPD/tuned
/// clobbering the ryzenadj limits behind our back; design §3).
const REASSERT_PERIOD_S: f64 = 10.0;
/// A sample must exceed the CPU limit by this margin to count as a
/// stickiness violation (RAPL vs STAPM accounting slack).
const STICKINESS_MARGIN_W: f64 = 5.0;
/// Consecutive violating samples before the stickiness watchdog fires.
const STICKINESS_SAMPLES: u8 = 3;
/// How long the `Resumed` flag stays visible after a suspend/resume.
const RESUMED_FLAG_S: f64 = 30.0;
/// Fan target clamp range (RPM); the control loop consumes it in M4.
const FAN_TARGET_MIN_RPM: f64 = 1000.0;
const FAN_TARGET_MAX_RPM: f64 = 7000.0;
/// Startup fan target (RPM). Shared with the UI model so the controller's
/// echoed status and the displayed default can never diverge.
pub const DEFAULT_FAN_TARGET_RPM: f64 = 3000.0;

/// UI -> controller commands: the manual-mode keys emit the setters and
/// `ReleaseAll`; `Quit` comes from main's shutdown sequence only.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// Sustained CPU package watts (converted to mW and clamped inside).
    SetCpuW(f64),
    /// GPU max clock in MHz (clamped inside).
    SetGpuMaxClock(u32),
    /// Back to Monitor mode: restore stock limits but keep running.
    ReleaseAll,
    /// Stored + echoed in status; the control loop uses it in M4.
    SetFanTarget(f64),
    /// Begin guided calibration (honored in Monitor mode only).
    StartCalibration,
    /// Abort a running calibration (release everything, back to Monitor).
    AbortCalibration,
    /// Restore hardware and exit the controller thread.
    Quit,
}

/// Controller mode. Auto comes later. TODO(task-25): Auto.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mode {
    #[default]
    Monitor,
    Manual,
    /// The calibration runner owns actuation; manual commands and the
    /// reassert/stickiness machinery are suspended.
    Calibrating,
}

impl Mode {
    /// Telemetry/UI string form.
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Monitor => "monitor",
            Mode::Manual => "manual",
            Mode::Calibrating => "calibrating",
        }
    }
}

/// Active watchdog/status flags shown in the UI and telemetry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StatusFlag {
    /// RAPL keeps measuring well above the commanded CPU limit.
    LimitNotSticking,
    /// A suspend/resume was detected recently (cleared after 30 s).
    Resumed,
}

impl StatusFlag {
    /// Telemetry string form.
    pub fn as_str(self) -> &'static str {
        match self {
            StatusFlag::LimitNotSticking => "limit_not_sticking",
            StatusFlag::Resumed => "resumed",
        }
    }
}

/// What the controller is doing right now; sent to the UI (and mirrored into
/// telemetry `Decision` records) whenever it changes.
#[derive(Clone, Debug, PartialEq)]
pub struct ControlStatus {
    pub mode: Mode,
    /// Commanded (clamped) sustained CPU limit, watts.
    pub cpu_limit_w: Option<f64>,
    /// Commanded (clamped) GPU max clock, MHz.
    pub gpu_max_mhz: Option<u32>,
    /// Stored fan target (RPM); consumed by the control loop in M4.
    pub fan_target_rpm: f64,
    /// Currently active flags.
    pub flags: Vec<StatusFlag>,
    /// Calibration wizard progress; Some exactly while Calibrating.
    pub calib: Option<CalibProgressLite>,
}

/// Hand-written (not derived) so `fan_target_rpm` starts at the real default
/// instead of an unrepresentable 0.0 in the first Status event.
impl Default for ControlStatus {
    fn default() -> Self {
        Self {
            mode: Mode::default(),
            cpu_limit_w: None,
            gpu_max_mhz: None,
            fan_target_rpm: DEFAULT_FAN_TARGET_RPM,
            flags: Vec::new(),
            calib: None,
        }
    }
}

/// What one `on_sample`/`on_command` call did — consumed by the thread shell
/// (channel sends + telemetry) and asserted on directly in tests.
#[derive(Clone, Debug, PartialEq)]
pub enum Effect {
    /// CPU sustained limit commanded (clamped watts).
    CpuSet(f64),
    /// GPU max clock commanded (clamped MHz).
    GpuSet(u32),
    /// Limits released back to stock (actuators stay owned).
    Released,
    /// Active limits were reapplied (periodic / stickiness / resume).
    Reasserted { cause: &'static str },
    /// `status` changed vs. before the call (shell sends Event::Status).
    StatusChanged { cause: &'static str },
    /// Telemetry-only note: something worth a Decision record happened but
    /// the user-visible status is unchanged (e.g. a repeated calibration
    /// NeedsGpuLoad nag — offline analysis needs the full nag history).
    Noted { cause: &'static str },
    /// Hardware restored; the thread shell must exit its loop.
    Quit,
}

/// Testable controller core. Owns the actuators through `RestoreGuard`, so
/// hardware is restored even if the thread shell exits abnormally.
pub struct Controller<R: Runner> {
    guard: RestoreGuard<R>,
    status: ControlStatus,
    /// Consecutive samples measuring over the CPU limit (stickiness watchdog).
    stick_violations: u8,
    /// `t_mono` until which the `Resumed` flag stays visible.
    resumed_until: Option<f64>,
    /// `t_mono` of the last (re)assert, None until the first post-command sample.
    last_reassert: Option<f64>,
    /// Running calibration session; Some exactly while `Mode::Calibrating`.
    calib: Option<CalibRunner>,
    /// CPU burner owned on the runner's behalf (StartBurner/StopBurner).
    burner: Option<Burner>,
    /// Where `RunnerEffect::SaveState` persists to (`--state-file`).
    state_path: PathBuf,
    /// Fitted thermal model: loaded from the state file at construction,
    /// replaced by a fresh calibration.
    /// TODO(task-23/25): consumed by the auto-mode allocator; unread until then.
    #[allow(dead_code)]
    model: Option<ThermalModel>,
    /// GPU clock→watts LUT, same lifecycle as `model`.
    /// TODO(task-24): consumed by the GPU watts→clock PI; unread until then.
    #[allow(dead_code)]
    lut: Option<ClockWattsLut>,
}

impl<R: Runner> Controller<R> {
    pub fn new(guard: RestoreGuard<R>, persisted: PersistedState, state_path: PathBuf) -> Self {
        Self {
            guard,
            status: ControlStatus::default(),
            stick_violations: 0,
            resumed_until: None,
            last_reassert: None,
            calib: None,
            burner: None,
            state_path,
            model: persisted.model,
            lut: persisted.lut,
        }
    }

    /// Current status (the shell clones it into `Event::Status`).
    pub fn status(&self) -> &ControlStatus {
        &self.status
    }

    /// Restore stock hardware state (idempotent; delegates to the guard).
    /// The shell calls this on loop exit; `Command::Quit` also runs it.
    pub fn restore_all(&mut self) {
        self.guard.restore_all();
    }

    /// Consume one command, actuate, mutate status; returns what happened.
    /// Actuator failures are warned and leave the status untouched — the UI
    /// keeps showing what is actually applied, never what merely was asked.
    pub fn on_command(&mut self, c: Command) -> Vec<Effect> {
        // While calibrating the runner owns actuation: manual setters and
        // release are rejected outright (Esc/AbortCalibration is the way to
        // take control back).
        if self.status.mode == Mode::Calibrating
            && matches!(
                c,
                Command::SetCpuW(_) | Command::SetGpuMaxClock(_) | Command::ReleaseAll
            )
        {
            tracing::warn!("manual command rejected while calibrating: {c:?}");
            return Vec::new();
        }
        let before = self.status.clone();
        let mut effects = Vec::new();
        let cause = match c {
            Command::SetCpuW(w) => {
                match self.guard.cpu.as_ref() {
                    None => tracing::warn!("no CPU actuator this run; ignoring SetCpuW({w})"),
                    Some(cpu) => match cpu.set_sustained_mw((w * 1000.0).round() as u32) {
                        Ok(clamped_mw) => {
                            let clamped_w = f64::from(clamped_mw) / 1000.0;
                            self.status.cpu_limit_w = Some(clamped_w);
                            self.status.mode = Mode::Manual;
                            effects.push(Effect::CpuSet(clamped_w));
                        }
                        Err(e) => tracing::warn!("SetCpuW({w}) failed, status unchanged: {e}"),
                    },
                }
                "command:set_cpu_w"
            }
            Command::SetGpuMaxClock(mhz) => {
                match self.guard.gpu.as_mut() {
                    None => {
                        tracing::warn!("no GPU actuator this run; ignoring SetGpuMaxClock({mhz})");
                    }
                    Some(gpu) => match gpu.set_max_clock(mhz) {
                        Ok(()) => {
                            self.status.gpu_max_mhz = gpu.applied();
                            self.status.mode = Mode::Manual;
                            effects.push(Effect::GpuSet(gpu.applied().unwrap_or(mhz)));
                        }
                        Err(e) => {
                            tracing::warn!("SetGpuMaxClock({mhz}) failed, status unchanged: {e}");
                        }
                    },
                }
                "command:set_gpu_max_clock"
            }
            Command::ReleaseAll => {
                // NOT guard.restore_all(): the smu module must stay unloaded
                // and the actuators stay owned — the session keeps running.
                if let Some(gpu) = self.guard.gpu.as_mut() {
                    match gpu.release() {
                        Ok(()) => tracing::info!("released GPU clock locks"),
                        Err(e) => tracing::warn!("release: GPU clock release failed: {e}"),
                    }
                }
                if let Some(cpu) = self.guard.cpu.as_ref() {
                    if let Err(e) = cpu.restore_stock() {
                        tracing::warn!("release: CPU stock restore failed: {e}");
                    }
                }
                self.status.cpu_limit_w = None;
                self.status.gpu_max_mhz = None;
                self.status.mode = Mode::Monitor;
                self.remove_flag(StatusFlag::LimitNotSticking);
                self.stick_violations = 0;
                self.last_reassert = None;
                effects.push(Effect::Released);
                "release"
            }
            Command::SetFanTarget(rpm) => {
                self.status.fan_target_rpm = rpm.clamp(FAN_TARGET_MIN_RPM, FAN_TARGET_MAX_RPM);
                "command:set_fan_target"
            }
            Command::StartCalibration => {
                if self.status.mode != Mode::Monitor {
                    tracing::warn!(
                        "StartCalibration ignored: mode is {}, not monitor",
                        self.status.mode.as_str()
                    );
                } else {
                    let mut runner = CalibRunner::new();
                    let runner_effects = runner.start();
                    self.calib = Some(runner);
                    self.status.mode = Mode::Calibrating;
                    self.apply_calib_effects(runner_effects);
                    self.sync_calib_status();
                }
                "calib:start"
            }
            Command::AbortCalibration => {
                match self.calib.take() {
                    None => tracing::warn!("AbortCalibration ignored: no calibration running"),
                    Some(mut runner) => {
                        self.apply_calib_effects(runner.abort());
                        self.end_calibration();
                    }
                }
                "calib:aborted"
            }
            Command::Quit => {
                // Abort a running calibration FIRST: burner threads stopped
                // and calibration limits released before the guard's full
                // restore (which reloads ryzen_smu last).
                if let Some(mut runner) = self.calib.take() {
                    self.apply_calib_effects(runner.abort());
                    self.end_calibration();
                }
                self.guard.restore_all();
                effects.push(Effect::Quit);
                return effects;
            }
        };
        if self.status != before {
            effects.push(Effect::StatusChanged { cause });
        }
        effects
    }

    /// Consume one 1 Hz sample: resume handling, stickiness watchdog and the
    /// periodic reassert (all t_mono-driven); returns what happened. While
    /// Calibrating, all of that is SUSPENDED — the sample goes to the
    /// calibration runner, which owns actuation (the stickiness watchdog
    /// would fight the runner's deliberate low limits, and the runner
    /// re-commands each point itself).
    pub fn on_sample(&mut self, s: &Sample) -> Vec<Effect> {
        if self.status.mode == Mode::Calibrating {
            return self.on_calib_sample(s);
        }
        let before = self.status.clone();
        let mut effects = Vec::new();
        // First status-affecting stage wins the Decision `cause`; the record
        // carries the full status either way.
        let mut cause: Option<&'static str> = None;

        // Resume: firmware may have forgotten our limits across the suspend.
        if s.resumed {
            if let Some(all_ok) = self.reassert_actuators() {
                self.last_reassert = Some(s.t_mono);
                // Telemetry honesty (as in the periodic path): a failed
                // attempt must not count as a phantom reassert.
                effects.push(Effect::Reasserted {
                    cause: if all_ok { "resume" } else { "resume_failed" },
                });
            }
            self.add_flag(StatusFlag::Resumed);
            self.resumed_until = Some(s.t_mono + RESUMED_FLAG_S);
            cause.get_or_insert("resume");
        } else if self.resumed_until.is_some_and(|until| s.t_mono >= until) {
            self.remove_flag(StatusFlag::Resumed);
            self.resumed_until = None;
            cause.get_or_insert("resume");
        }

        // Stickiness watchdog: RAPL says the commanded limit is not holding.
        // cpu_pkg_w == 0.0 is RAPL warmup/invalid — neither a violation nor
        // evidence of compliance, so it leaves the streak untouched.
        if let Some(limit) = self.status.cpu_limit_w {
            if s.cpu_pkg_w > 0.0 {
                if s.cpu_pkg_w > limit + STICKINESS_MARGIN_W {
                    self.stick_violations += 1;
                    if self.stick_violations >= STICKINESS_SAMPLES {
                        // Reset so re-triggering needs a fresh streak instead
                        // of hammering ryzenadj at 1 Hz.
                        self.stick_violations = 0;
                        tracing::warn!(
                            "CPU limit not sticking: {} W measured vs {limit} W commanded \
                             ({STICKINESS_SAMPLES} consecutive samples); reasserting",
                            s.cpu_pkg_w
                        );
                        if let Some(all_ok) = self.reassert_actuators() {
                            self.last_reassert = Some(s.t_mono);
                            effects.push(Effect::Reasserted {
                                cause: if all_ok {
                                    "stickiness"
                                } else {
                                    "stickiness_failed"
                                },
                            });
                        }
                        self.add_flag(StatusFlag::LimitNotSticking);
                        cause.get_or_insert("stickiness");
                    }
                } else {
                    self.stick_violations = 0;
                    self.remove_flag(StatusFlag::LimitNotSticking);
                    cause.get_or_insert("stickiness");
                }
            }
        }

        // Periodic reassert (defends against PPD/tuned clobbers). The first
        // sample after a command only pins the baseline.
        if self.status.cpu_limit_w.is_some() || self.status.gpu_max_mhz.is_some() {
            match self.last_reassert {
                None => self.last_reassert = Some(s.t_mono),
                Some(last) if s.t_mono - last >= REASSERT_PERIOD_S => {
                    if let Some(all_ok) = self.reassert_actuators() {
                        // Advance the baseline even on failure: the retry
                        // cadence stays 10 s. Telemetry honesty: a failed
                        // attempt must not count as a phantom reassert.
                        self.last_reassert = Some(s.t_mono);
                        let cause = if all_ok {
                            "reassert"
                        } else {
                            "reassert_failed"
                        };
                        effects.push(Effect::Reasserted { cause });
                    }
                }
                Some(_) => {}
            }
        }

        if self.status != before {
            effects.push(Effect::StatusChanged {
                cause: cause.unwrap_or("sample"),
            });
        }
        effects
    }

    /// One calibrating-mode sample: feed the runner, execute its effects,
    /// refresh the wizard progress in status.
    fn on_calib_sample(&mut self, s: &Sample) -> Vec<Effect> {
        let before = self.status.clone();
        let runner_effects = match self.calib.as_mut() {
            Some(runner) => runner.on_sample(s),
            None => {
                // Defensive: mode says Calibrating but no runner; recover.
                tracing::warn!("Calibrating mode without a runner; returning to Monitor");
                self.end_calibration();
                Vec::new()
            }
        };
        let cause = self.apply_calib_effects(runner_effects);
        self.sync_calib_status();
        let mut effects = Vec::new();
        if self.status != before {
            effects.push(Effect::StatusChanged {
                cause: cause.unwrap_or("calib:progress"),
            });
        } else if let Some(cause) = cause {
            // Notable runner event without a status delta (e.g. the second
            // and later NeedsGpuLoad nags: needs_load is already true).
            // Still worth a telemetry Decision line.
            effects.push(Effect::Noted { cause });
        }
        effects
    }

    /// Execute one batch of runner effects against the guard's actuators,
    /// the burner and the state file. Actuator failures are warned — the
    /// runner records MEASURED watts, so a missed command skews one point
    /// instead of breaking the machine. Returns the most significant
    /// telemetry cause the batch produced.
    fn apply_calib_effects(&mut self, effects: Vec<RunnerEffect>) -> Option<&'static str> {
        /// Higher wins when a batch carries several notable events (the
        /// final batch is releases + PointRecorded + Fitted + Finished).
        fn rank(cause: &str) -> u8 {
            match cause {
                "calib:failed" => 5,
                "calib:fitted" => 4,
                "calib:finished" => 3,
                "calib:point_recorded" => 2,
                _ => 1,
            }
        }
        fn raise(cur: &mut Option<&'static str>, c: &'static str) {
            if cur.is_none_or(|old| rank(c) > rank(old)) {
                *cur = Some(c);
            }
        }
        let mut cause: Option<&'static str> = None;
        let mut ended = false;
        for effect in effects {
            match effect {
                RunnerEffect::SetCpuW(w) => match self.guard.cpu.as_ref() {
                    None => tracing::warn!("calib: no CPU actuator; SetCpuW({w}) skipped"),
                    Some(cpu) => match cpu.set_sustained_mw((w * 1000.0).round() as u32) {
                        Ok(clamped_mw) => {
                            self.status.cpu_limit_w = Some(f64::from(clamped_mw) / 1000.0);
                        }
                        Err(e) => tracing::warn!("calib: SetCpuW({w}) failed: {e}"),
                    },
                },
                RunnerEffect::SetGpuMaxClock(mhz) => match self.guard.gpu.as_mut() {
                    None => tracing::warn!("calib: no GPU actuator; SetGpuMaxClock({mhz}) skipped"),
                    Some(gpu) => match gpu.set_max_clock(mhz) {
                        Ok(()) => self.status.gpu_max_mhz = gpu.applied(),
                        Err(e) => tracing::warn!("calib: SetGpuMaxClock({mhz}) failed: {e}"),
                    },
                },
                RunnerEffect::ReleaseCpu => {
                    if let Some(cpu) = self.guard.cpu.as_ref() {
                        if let Err(e) = cpu.restore_stock() {
                            tracing::warn!("calib: CPU stock restore failed: {e}");
                        }
                    }
                    self.status.cpu_limit_w = None;
                }
                RunnerEffect::ReleaseGpu => {
                    if let Some(gpu) = self.guard.gpu.as_mut() {
                        if let Err(e) = gpu.release() {
                            tracing::warn!("calib: GPU clock release failed: {e}");
                        }
                    }
                    self.status.gpu_max_mhz = None;
                }
                RunnerEffect::StartBurner(n) => {
                    // Replace any running burner: the runner re-commands the
                    // CPU side on every point entry.
                    if let Some(old) = self.burner.take() {
                        old.stop();
                    }
                    self.burner = Some(Burner::start(n));
                }
                RunnerEffect::StopBurner => {
                    if let Some(burner) = self.burner.take() {
                        burner.stop();
                    }
                }
                RunnerEffect::NeedsGpuLoad => raise(&mut cause, "calib:needs_load"),
                RunnerEffect::PointRecorded { phase, idx, detail } => {
                    tracing::info!("calib: {phase} point {idx} recorded: {detail}");
                    raise(&mut cause, "calib:point_recorded");
                }
                RunnerEffect::Fitted {
                    a,
                    b,
                    e,
                    c,
                    max_residual,
                } => {
                    tracing::info!(
                        "calib: fitted a={a:.2} b={b:.2} e={e:.3} c={c:.0} \
                         (max residual {max_residual:.0} RPM)"
                    );
                    raise(&mut cause, "calib:fitted");
                }
                RunnerEffect::SaveState(state) => {
                    self.model = state.model.clone();
                    self.lut = state.lut.clone();
                    match state.save(&self.state_path) {
                        Ok(()) => {
                            tracing::info!("calib: state saved to {}", self.state_path.display());
                        }
                        Err(e) => tracing::warn!(
                            "calib: state save to {} failed: {e}",
                            self.state_path.display()
                        ),
                    }
                }
                RunnerEffect::Failed(msg) => {
                    tracing::warn!("calibration failed: {msg}");
                    raise(&mut cause, "calib:failed");
                    ended = true;
                }
                RunnerEffect::Finished => {
                    raise(&mut cause, "calib:finished");
                    ended = true;
                }
            }
        }
        if ended {
            self.end_calibration();
        }
        cause
    }

    /// Mirror the runner's progress into status (None once it's gone).
    fn sync_calib_status(&mut self) {
        self.status.calib = self.calib.as_ref().map(CalibRunner::progress);
    }

    /// Back to Monitor: drop the runner, stop the burner (defensive — the
    /// runner's own StopBurner normally already ran), clear the wizard.
    fn end_calibration(&mut self) {
        if let Some(burner) = self.burner.take() {
            burner.stop();
        }
        self.calib = None;
        self.status.mode = Mode::Monitor;
        self.status.calib = None;
    }

    /// Reapply whatever limits are currently commanded (same values). Errors
    /// are warned — the periodic retry IS the recovery. `None` if nothing was
    /// commanded; otherwise `Some(all_calls_succeeded)` so telemetry can
    /// distinguish real reasserts from failed attempts.
    fn reassert_actuators(&mut self) -> Option<bool> {
        let mut any = false;
        let mut all_ok = true;
        if let (Some(w), Some(cpu)) = (self.status.cpu_limit_w, self.guard.cpu.as_ref()) {
            any = true;
            if let Err(e) = cpu.set_sustained_mw((w * 1000.0).round() as u32) {
                all_ok = false;
                tracing::warn!("reassert: CPU limit ({w} W) failed: {e}");
            }
        }
        if let Some(mhz) = self.status.gpu_max_mhz {
            if let Some(gpu) = self.guard.gpu.as_mut() {
                any = true;
                if let Err(e) = gpu.set_max_clock(mhz) {
                    all_ok = false;
                    tracing::warn!("reassert: GPU max clock ({mhz} MHz) failed: {e}");
                }
            }
        }
        any.then_some(all_ok)
    }

    fn add_flag(&mut self, flag: StatusFlag) {
        if !self.status.flags.contains(&flag) {
            self.status.flags.push(flag);
        }
    }

    fn remove_flag(&mut self, flag: StatusFlag) {
        self.status.flags.retain(|&f| f != flag);
    }
}

/// Thread shell: `select!` over samples and commands, mapping effects to
/// `Event::Status` sends and telemetry `Decision` records. On loop exit
/// (Quit or command-channel disconnect) it restores hardware and flips
/// `restored` so main's `FinalRestore` knows to stand down.
pub fn spawn<R: Runner + Send + 'static>(
    controller: Controller<R>,
    sample_rx: Receiver<Event>,
    cmd_rx: Receiver<Command>,
    ui_tx: Sender<Event>,
    telemetry: Arc<Mutex<Option<Telemetry>>>,
    restored: Arc<AtomicBool>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("controller".into())
        .spawn(move || {
            let mut controller = controller;
            let mut sample_rx = sample_rx;
            // t_mono of the latest sample, stamped into command-driven
            // Decision records (0.0 before the first sample).
            let mut t_mono = 0.0_f64;
            loop {
                // Nothing here may block long between iterations: repeated
                // SIGINT only re-sets main's flag, so shutdown latency is
                // bounded by one iteration (restore_stock's 200 ms profile
                // toggle is the accepted worst case).
                select! {
                    recv(sample_rx) -> msg => match msg {
                        Ok(Event::Sample(s)) => {
                            t_mono = s.t_mono;
                            let effects = controller.on_sample(&s);
                            apply_effects(&effects, &controller, t_mono, &ui_tx, &telemetry);
                        }
                        Ok(_) => {} // only samples arrive on this channel
                        Err(_) => {
                            // Sampler gone (it exits first on shutdown, or
                            // died). Keep serving commands: main is still
                            // alive and will send Quit.
                            tracing::debug!("controller: sample channel disconnected");
                            sample_rx = never();
                        }
                    },
                    recv(cmd_rx) -> msg => match msg {
                        Ok(cmd) => {
                            let effects = controller.on_command(cmd);
                            let quit =
                                apply_effects(&effects, &controller, t_mono, &ui_tx, &telemetry);
                            if quit {
                                break;
                            }
                        }
                        // Main dropped cmd_tx without sending Quit (panic
                        // path): restore from here — main's FinalRestore only
                        // covers the race where we don't get to finish.
                        Err(_) => {
                            tracing::info!(
                                "controller: command channel disconnected, shutting down"
                            );
                            break;
                        }
                    },
                }
            }
            // Idempotent: already ran if we broke out via Command::Quit.
            controller.restore_all();
            restored.store(true, Ordering::SeqCst);
            tracing::info!("controller: hardware restored, exiting");
        })
        .expect("failed to spawn controller thread")
}

/// Map one effect batch to the outside world: status changes go to the UI,
/// and any batch that changed status or reasserted becomes one telemetry
/// `Decision` record. Returns true if the shell must quit.
fn apply_effects<R: Runner>(
    effects: &[Effect],
    controller: &Controller<R>,
    t_mono: f64,
    ui_tx: &Sender<Event>,
    telemetry: &Mutex<Option<Telemetry>>,
) -> bool {
    let mut quit = false;
    let mut status_changed = false;
    let mut cause: Option<&'static str> = None;
    for effect in effects {
        match effect {
            Effect::Reasserted { cause: c } | Effect::Noted { cause: c } => {
                cause.get_or_insert(c);
            }
            Effect::StatusChanged { cause: c } => {
                status_changed = true;
                cause.get_or_insert(c);
            }
            Effect::Quit => quit = true,
            Effect::CpuSet(_) | Effect::GpuSet(_) | Effect::Released => {}
        }
    }
    let status = controller.status();
    if status_changed {
        // A send failure means the UI is gone; shutdown is already underway.
        let _ = ui_tx.send(Event::Status(status.clone()));
    }
    if let Some(cause) = cause {
        let record = Record::Decision {
            t_mono,
            mode: status.mode.as_str().to_string(),
            cpu_limit_w: status.cpu_limit_w,
            gpu_max_mhz: status.gpu_max_mhz,
            fan_target_rpm: status.fan_target_rpm,
            cause: cause.to_string(),
            flags: status
                .flags
                .iter()
                .map(|f| f.as_str().to_string())
                .collect(),
        };
        if let Some(t) = telemetry::lock(telemetry).as_mut() {
            t.log(&record);
        }
    }
    quit
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuators::cmd::test_support::{FakeRunner, output_with_code};
    use crate::actuators::cpu::CpuActuator;
    use crate::actuators::smu_module::SmuModule;
    use std::fs;
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    /// Unique-per-test profile file fixture; caller removes the dir when done.
    fn profile_fixture(name: &str) -> (PathBuf, PathBuf) {
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-{name}",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("platform_profile");
        fs::write(&path, "balanced\n").unwrap();
        (dir, path)
    }

    fn cpu_actuator(runner: &FakeRunner, profile_path: PathBuf) -> CpuActuator<&FakeRunner> {
        let mut cpu = CpuActuator::new(runner, profile_path);
        cpu.toggle_delay = Duration::from_millis(1); // keep tests fast
        cpu
    }

    /// Controller over a FakeRunner-backed CPU actuator (no GPU: NVML needs
    /// hardware) and an smu module "we unloaded" (so Quit's reload shows up).
    fn controller(runner: &FakeRunner, profile_path: PathBuf) -> Controller<&FakeRunner> {
        Controller::new(
            RestoreGuard::new(
                runner,
                Some(cpu_actuator(runner, profile_path)),
                None,
                Some(SmuModule::assume_unloaded()),
            ),
            PersistedState::default(),
            PathBuf::from("/nonexistent/state.json"),
        )
    }

    /// No profile file needed: tests that never touch restore_stock.
    fn controller_no_profile(runner: &FakeRunner) -> Controller<&FakeRunner> {
        controller(runner, PathBuf::from("/nonexistent/platform_profile"))
    }

    fn ryzenadj_calls(runner: &FakeRunner) -> Vec<Vec<String>> {
        runner
            .calls()
            .into_iter()
            .filter(|(prog, _)| prog == "ryzenadj")
            .map(|(_, args)| args)
            .collect()
    }

    fn modprobe_reload_calls(runner: &FakeRunner) -> usize {
        runner
            .calls()
            .iter()
            .filter(|(prog, args)| prog == "modprobe" && args == &vec!["ryzen_smu".to_string()])
            .count()
    }

    fn expected_args(mw: u32) -> Vec<String> {
        vec![
            format!("--stapm-limit={mw}"),
            format!("--slow-limit={mw}"),
            "--fast-limit=53000".to_string(),
        ]
    }

    fn sample_at(t_mono: f64) -> Sample {
        Sample {
            t_mono,
            ..Sample::default()
        }
    }

    fn sample_with_power(t_mono: f64, cpu_pkg_w: f64) -> Sample {
        Sample {
            t_mono,
            cpu_pkg_w,
            ..Sample::default()
        }
    }

    fn status_changes(effects: &[Effect]) -> usize {
        effects
            .iter()
            .filter(|e| matches!(e, Effect::StatusChanged { .. }))
            .count()
    }

    fn has_reassert(effects: &[Effect], want_cause: &str) -> bool {
        effects
            .iter()
            .any(|e| matches!(e, Effect::Reasserted { cause } if *cause == want_cause))
    }

    #[test]
    fn set_cpu_w_clamps_applies_and_updates_status() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);

        // 5 W clamps to the 10 W floor.
        let effects = ctl.on_command(Command::SetCpuW(5.0));
        assert_eq!(ryzenadj_calls(&runner), vec![expected_args(10_000)]);
        assert_eq!(ctl.status().mode, Mode::Manual);
        assert_eq!(ctl.status().cpu_limit_w, Some(10.0));
        assert!(effects.contains(&Effect::CpuSet(10.0)), "got {effects:?}");
        assert_eq!(status_changes(&effects), 1);

        // Identical command again: actuator re-commanded, but the status is
        // unchanged so no StatusChanged is emitted (no 1 Hz status spam).
        let effects = ctl.on_command(Command::SetCpuW(5.0));
        assert_eq!(ryzenadj_calls(&runner).len(), 2);
        assert_eq!(status_changes(&effects), 0, "got {effects:?}");
    }

    #[test]
    fn set_cpu_w_failure_leaves_status_unchanged() {
        let runner = FakeRunner::new();
        let mut failed = output_with_code(1);
        failed.stderr = b"Unable to get os_access Obj\n".to_vec();
        runner.push_result(Ok(failed));
        let mut ctl = controller_no_profile(&runner);

        let effects = ctl.on_command(Command::SetCpuW(20.0));
        assert!(effects.is_empty(), "got {effects:?}");
        assert_eq!(*ctl.status(), ControlStatus::default());
    }

    #[test]
    fn set_cpu_w_without_actuator_is_a_warned_noop() {
        let runner = FakeRunner::new();
        let mut ctl: Controller<&FakeRunner> = Controller::new(
            RestoreGuard::new(&runner, None, None, None),
            PersistedState::default(),
            PathBuf::from("/nonexistent/state.json"),
        );

        let effects = ctl.on_command(Command::SetCpuW(20.0));
        assert!(effects.is_empty(), "got {effects:?}");
        assert!(runner.calls().is_empty());
        assert_eq!(*ctl.status(), ControlStatus::default());
    }

    #[test]
    fn set_gpu_without_actuator_is_a_warned_noop() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);

        let effects = ctl.on_command(Command::SetGpuMaxClock(1500));
        assert!(effects.is_empty(), "got {effects:?}");
        assert_eq!(ctl.status().gpu_max_mhz, None);
        assert_eq!(ctl.status().mode, Mode::Monitor);
    }

    #[test]
    fn release_all_restores_stock_and_resets_status() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("release-all");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::SetCpuW(20.0));

        let effects = ctl.on_command(Command::ReleaseAll);

        // CPU stock restore toggled the platform profile back; no GPU (None)
        // is skipped without error; the smu module stays UNLOADED (release
        // keeps the session alive; a reload would break further actuation).
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        assert_eq!(modprobe_reload_calls(&runner), 0);
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert_eq!(ctl.status().gpu_max_mhz, None);
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert_eq!(status_changes(&effects), 1);

        // Actuators stay owned: manual mode still works afterwards.
        let effects = ctl.on_command(Command::SetCpuW(20.0));
        assert_eq!(ctl.status().cpu_limit_w, Some(20.0));
        assert_eq!(status_changes(&effects), 1);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn fan_target_is_clamped_and_stored() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);

        let effects = ctl.on_command(Command::SetFanTarget(100.0));
        assert_eq!(ctl.status().fan_target_rpm, 1000.0);
        assert_eq!(status_changes(&effects), 1);

        ctl.on_command(Command::SetFanTarget(9999.0));
        assert_eq!(ctl.status().fan_target_rpm, 7000.0);

        ctl.on_command(Command::SetFanTarget(3000.0));
        assert_eq!(ctl.status().fan_target_rpm, 3000.0);
        // Fan target alone is not actuation: mode stays Monitor.
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ryzenadj_calls(&runner).is_empty());
    }

    #[test]
    fn reasserts_cpu_limit_every_10s() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));
        assert_eq!(ryzenadj_calls(&runner).len(), 1);

        // First sample only sets the baseline; nothing before 10 s elapse.
        assert!(ctl.on_sample(&sample_at(0.0)).is_empty());
        assert!(ctl.on_sample(&sample_at(5.0)).is_empty());
        assert!(ctl.on_sample(&sample_at(9.9)).is_empty());
        assert_eq!(ryzenadj_calls(&runner).len(), 1);

        // 10 s past the baseline: reassert with the SAME args.
        let effects = ctl.on_sample(&sample_at(10.1));
        assert!(has_reassert(&effects, "reassert"), "got {effects:?}");
        assert_eq!(
            ryzenadj_calls(&runner),
            vec![expected_args(20_000), expected_args(20_000)]
        );
        // Reassert alone changes nothing user-visible: no status spam.
        assert_eq!(status_changes(&effects), 0);
    }

    #[test]
    fn default_status_starts_at_the_real_fan_target() {
        // Not 0.0: the first Status event must never show an unrepresentable
        // target (shared const keeps model display and controller in sync).
        assert_eq!(ControlStatus::default().fan_target_rpm, 3000.0);
        assert_eq!(
            ControlStatus::default().fan_target_rpm,
            DEFAULT_FAN_TARGET_RPM
        );
    }

    #[test]
    fn failed_reassert_reports_reassert_failed() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));
        assert!(ctl.on_sample(&sample_at(0.0)).is_empty()); // baseline

        // The next ryzenadj invocation (the 10 s reassert) fails.
        runner.push_result(Ok(output_with_code(1)));
        let effects = ctl.on_sample(&sample_at(10.1));
        assert!(
            has_reassert(&effects, "reassert_failed"),
            "a failed attempt must not count as a phantom reassert, got {effects:?}"
        );
        assert_eq!(status_changes(&effects), 0, "status is unchanged");
        assert_eq!(ryzenadj_calls(&runner).len(), 2, "initial set + attempt");

        // The baseline still advanced: retry follows the normal 10 s cadence.
        assert!(ctl.on_sample(&sample_at(10.2)).is_empty());
        let effects = ctl.on_sample(&sample_at(20.2));
        assert!(has_reassert(&effects, "reassert"), "got {effects:?}");
        assert_eq!(ryzenadj_calls(&runner).len(), 3);
    }

    #[test]
    fn no_reassert_without_active_limits() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        assert!(ctl.on_sample(&sample_at(0.0)).is_empty());
        assert!(ctl.on_sample(&sample_at(20.0)).is_empty());
        assert!(runner.calls().is_empty());
    }

    #[test]
    fn stickiness_flags_after_three_consecutive_violations() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));

        // 26 W > 20 + 5 margin: two violations are not enough.
        assert!(ctl.on_sample(&sample_with_power(1.0, 26.0)).is_empty());
        assert!(ctl.on_sample(&sample_with_power(2.0, 26.0)).is_empty());
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));

        // Third consecutive violation: immediate reassert + flag.
        let effects = ctl.on_sample(&sample_with_power(3.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ryzenadj_calls(&runner).len(), 2, "initial set + reassert");

        // A compliant sample clears the flag.
        let effects = ctl.on_sample(&sample_with_power(4.0, 19.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        assert_eq!(status_changes(&effects), 1);
    }

    #[test]
    fn stickiness_ignores_invalid_zero_power() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));

        // cpu_pkg_w == 0.0 means RAPL warmup/invalid: neither a violation
        // nor evidence of compliance.
        assert!(ctl.on_sample(&sample_with_power(1.0, 26.0)).is_empty());
        assert!(ctl.on_sample(&sample_with_power(2.0, 26.0)).is_empty());
        assert!(ctl.on_sample(&sample_with_power(3.0, 0.0)).is_empty());
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        let effects = ctl.on_sample(&sample_with_power(4.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
    }

    #[test]
    fn resume_reasserts_and_flags_for_30s() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));
        ctl.on_sample(&sample_at(100.0)); // reassert baseline

        let resumed = Sample {
            t_mono: 200.0,
            resumed: true,
            ..Sample::default()
        };
        let effects = ctl.on_sample(&resumed);
        assert!(has_reassert(&effects, "resume"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::Resumed));
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ryzenadj_calls(&runner).len(), 2, "initial set + resume");

        // 29 s later: still flagged.
        ctl.on_sample(&sample_at(229.0));
        assert!(ctl.status().flags.contains(&StatusFlag::Resumed));

        // 31 s after the resume: flag cleared (one status change).
        let effects = ctl.on_sample(&sample_at(231.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::Resumed));
        assert_eq!(status_changes(&effects), 1);
    }

    #[test]
    fn failed_resume_reassert_reports_resume_failed() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));

        // The next ryzenadj invocation (the resume reassert) fails.
        runner.push_result(Ok(output_with_code(1)));
        let resumed = Sample {
            t_mono: 200.0,
            resumed: true,
            ..Sample::default()
        };
        let effects = ctl.on_sample(&resumed);
        assert!(
            has_reassert(&effects, "resume_failed"),
            "a failed attempt must not count as a phantom resume reassert, got {effects:?}"
        );
        assert!(!has_reassert(&effects, "resume"), "got {effects:?}");
        // The Resumed flag is about the suspend, not the reassert: still set.
        assert!(ctl.status().flags.contains(&StatusFlag::Resumed));
    }

    #[test]
    fn failed_stickiness_reassert_reports_stickiness_failed() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));

        ctl.on_sample(&sample_with_power(1.0, 26.0));
        ctl.on_sample(&sample_with_power(2.0, 26.0));
        // The third violation triggers the reassert, which fails.
        runner.push_result(Ok(output_with_code(1)));
        let effects = ctl.on_sample(&sample_with_power(3.0, 26.0));
        assert!(
            has_reassert(&effects, "stickiness_failed"),
            "a failed attempt must not count as a phantom stickiness reassert, got {effects:?}"
        );
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
    }

    #[test]
    fn resume_without_limits_only_flags() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);

        let resumed = Sample {
            t_mono: 50.0,
            resumed: true,
            ..Sample::default()
        };
        let effects = ctl.on_sample(&resumed);
        assert!(
            !effects
                .iter()
                .any(|e| matches!(e, Effect::Reasserted { .. })),
            "nothing to reassert, got {effects:?}"
        );
        assert!(ctl.status().flags.contains(&StatusFlag::Resumed));
        assert!(runner.calls().is_empty());
    }

    #[test]
    fn quit_restores_all() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("quit");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::SetCpuW(20.0));

        let effects = ctl.on_command(Command::Quit);
        assert_eq!(effects, vec![Effect::Quit]);
        // Full restore sequence: profile toggled back + ryzen_smu reloaded.
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        assert_eq!(modprobe_reload_calls(&runner), 1);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn spawn_processes_commands_and_quits_promptly() {
        let (dir, path) = profile_fixture("spawn-smoke");
        let mut cpu = CpuActuator::new(FakeRunner::new(), path);
        cpu.toggle_delay = Duration::from_millis(1);
        let guard = RestoreGuard::new(FakeRunner::new(), Some(cpu), None, None);
        let controller = Controller::new(
            guard,
            PersistedState::default(),
            PathBuf::from("/nonexistent/state.json"),
        );

        let (ui_tx, ui_rx) = crossbeam_channel::unbounded();
        let (_sample_tx, sample_rx) = crossbeam_channel::unbounded::<Event>();
        let (cmd_tx, cmd_rx) = crossbeam_channel::unbounded();
        let telemetry: Arc<Mutex<Option<Telemetry>>> = Arc::new(Mutex::new(None));
        let restored = Arc::new(AtomicBool::new(false));
        let handle = spawn(
            controller,
            sample_rx,
            cmd_rx,
            ui_tx,
            telemetry,
            Arc::clone(&restored),
        );

        cmd_tx.send(Command::SetCpuW(20.0)).unwrap();
        match ui_rx.recv_timeout(Duration::from_secs(3)) {
            Ok(Event::Status(st)) => {
                assert_eq!(st.cpu_limit_w, Some(20.0));
                assert_eq!(st.mode, Mode::Manual);
            }
            other => panic!("expected Event::Status, got {other:?}"),
        }

        cmd_tx.send(Command::Quit).unwrap();
        let quit_sent = Instant::now();
        handle.join().expect("controller thread must not panic");
        assert!(
            quit_sent.elapsed() < Duration::from_secs(3),
            "quit must be prompt, took {:?}",
            quit_sent.elapsed()
        );
        assert!(
            restored.load(Ordering::SeqCst),
            "shell must flip the restored flag after restore_all"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    // --- Task 22: calibration integration ---

    use crate::calib::runner::MATRIX_POINTS;

    /// Sweep-phase sample: GPU pinned at `clock` drawing clock/30 watts.
    fn sweep_pinned(clock: u32) -> Sample {
        Sample {
            gpu_util_pct: 99.0,
            gpu_sm_mhz: f64::from(clock),
            gpu_w: f64::from(clock) / 30.0,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            fan1_rpm: 3000.0,
            fan_valid: true,
            ..Sample::default()
        }
    }

    /// The sample a well-behaved system produces on matrix point `idx`
    /// (measured CPU 2 W under the commanded limit; fans from a synthetic
    /// affine surface so every point settles).
    fn matrix_point_sample(idx: usize) -> Sample {
        let (cpu_t, gpu_t) = MATRIX_POINTS[idx];
        let cpu = if cpu_t > 5.0 { cpu_t - 2.0 } else { 4.0 };
        let (gpu, util) = if gpu_t > 0.0 {
            (gpu_t, 97.0)
        } else {
            (10.0, 3.0)
        };
        Sample {
            cpu_pkg_w: cpu,
            gpu_w: gpu,
            gpu_w_valid: true,
            gpu_util_pct: util,
            gpu_mhz_valid: true,
            fan1_rpm: 25.0 * cpu + 15.0 * gpu + 0.1 * cpu * gpu + 800.0,
            fan_valid: true,
            ..Sample::default()
        }
    }

    /// Drive the whole LUT sweep through the controller with pinned samples.
    fn drive_sweep(ctl: &mut Controller<&FakeRunner>) {
        use crate::calib::lut_sweep::SWEEP_CLOCKS;
        for (i, &clock) in SWEEP_CLOCKS.iter().enumerate() {
            for _ in 0..60 {
                ctl.on_sample(&sweep_pinned(clock));
                let calib = ctl.status().calib.as_ref().expect("calibrating");
                if calib.phase != "lut sweep" || calib.step > i {
                    break;
                }
            }
        }
        let calib = ctl.status().calib.as_ref().expect("calibrating");
        assert_eq!(calib.phase, "matrix", "sweep must finish: {calib:?}");
    }

    /// Drive matrix point `idx` to its recording through the controller.
    fn drive_matrix_point(ctl: &mut Controller<&FakeRunner>, idx: usize) {
        for _ in 0..300 {
            ctl.on_sample(&matrix_point_sample(idx));
            match ctl.status().calib.as_ref() {
                None => return, // calibration finished after the last point
                Some(calib) if calib.step > idx => return,
                Some(_) => {}
            }
        }
        panic!("matrix point {idx} never recorded through the controller");
    }

    #[test]
    fn start_calibration_only_from_monitor() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);

        // From Manual mode: warned no-op, status untouched.
        ctl.on_command(Command::SetCpuW(20.0));
        let effects = ctl.on_command(Command::StartCalibration);
        assert!(effects.is_empty(), "got {effects:?}");
        assert_eq!(ctl.status().mode, Mode::Manual);
        assert!(ctl.status().calib.is_none());
        assert!(ctl.calib.is_none());

        // Back to Monitor: calibration starts (LUT sweep phase).
        ctl.on_command(Command::ReleaseAll);
        let effects = ctl.on_command(Command::StartCalibration);
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ctl.status().mode, Mode::Calibrating);
        let calib = ctl.status().calib.as_ref().expect("wizard progress set");
        assert_eq!(calib.phase, "lut sweep");
        assert_eq!(calib.total, 10);
        // The sweep's first clock lock was attempted (no GPU actuator in
        // tests: warned no-op, gpu_max_mhz stays None).
        assert_eq!(ctl.status().gpu_max_mhz, None);
    }

    #[test]
    fn manual_commands_rejected_while_calibrating() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::StartCalibration);
        let calls_before = runner.calls().len();

        for cmd in [
            Command::SetCpuW(20.0),
            Command::SetGpuMaxClock(1500),
            Command::ReleaseAll,
        ] {
            let effects = ctl.on_command(cmd);
            assert!(effects.is_empty(), "{cmd:?} must be rejected: {effects:?}");
        }
        assert_eq!(runner.calls().len(), calls_before, "no actuation happened");
        assert_eq!(ctl.status().mode, Mode::Calibrating);

        // Fan target is not actuation: still accepted while calibrating.
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(ctl.status().fan_target_rpm, 2500.0);
    }

    #[test]
    fn calibration_effects_drive_actuators_and_burner() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-actuators");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        // Point 0 (idle/idle): no burner, no ryzenadj set.
        assert!(ctl.burner.is_none());
        assert!(ryzenadj_calls(&runner).is_empty());
        drive_matrix_point(&mut ctl, 0);

        // Point 1 (15 W): ryzenadj commanded with the matrix wattage and the
        // burner is running.
        assert_eq!(ryzenadj_calls(&runner), vec![expected_args(15_000)]);
        assert!(ctl.burner.is_some(), "burner must run for a loaded point");
        assert_eq!(ctl.status().cpu_limit_w, Some(15.0));
        drive_matrix_point(&mut ctl, 1);

        // Point 2 (30 W): re-commanded.
        assert_eq!(
            ryzenadj_calls(&runner),
            vec![expected_args(15_000), expected_args(30_000)]
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn repeated_needs_load_nags_are_noted_for_telemetry() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-nag");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        for idx in 0..4 {
            drive_matrix_point(&mut ctl, idx);
        }
        // Point 4 wants 35 GPU W; the GPU sits idle. First nag flips
        // needs_load: a StatusChanged Decision.
        let stalled = Sample {
            cpu_pkg_w: 4.0,
            gpu_w: 10.0,
            gpu_w_valid: true,
            gpu_util_pct: 3.0,
            gpu_mhz_valid: true,
            fan1_rpm: 1000.0,
            fan_valid: true,
            ..Sample::default()
        };
        for _ in 0..9 {
            assert!(ctl.on_sample(&stalled).is_empty());
        }
        let effects = ctl.on_sample(&stalled);
        assert_eq!(
            effects,
            vec![Effect::StatusChanged {
                cause: "calib:needs_load"
            }]
        );
        // Later nags change no status (needs_load already true) but must
        // still surface as telemetry-only notes, so offline analysis sees
        // the full nag history.
        for _ in 0..9 {
            assert!(ctl.on_sample(&stalled).is_empty());
        }
        let effects = ctl.on_sample(&stalled);
        assert_eq!(
            effects,
            vec![Effect::Noted {
                cause: "calib:needs_load"
            }]
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn abort_calibration_releases_and_returns_to_monitor() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-abort");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        drive_matrix_point(&mut ctl, 0);
        drive_matrix_point(&mut ctl, 1); // burner + 30 W limit now active
        assert!(ctl.burner.is_some());

        let effects = ctl.on_command(Command::AbortCalibration);
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ctl.status().calib.is_none());
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert!(ctl.calib.is_none());
        assert!(ctl.burner.is_none(), "abort must stop the burner");
        // Release toggled the profile back; smu stays untouched (session on).
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        assert_eq!(modprobe_reload_calls(&runner), 0);

        // Manual mode works again after the abort.
        ctl.on_command(Command::SetCpuW(20.0));
        assert_eq!(ctl.status().mode, Mode::Manual);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn quit_during_calibration_aborts_then_restores() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-quit");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        drive_matrix_point(&mut ctl, 0);
        drive_matrix_point(&mut ctl, 1); // burner + limit active

        let effects = ctl.on_command(Command::Quit);
        assert_eq!(effects, vec![Effect::Quit]);
        assert!(ctl.burner.is_none(), "quit must stop the burner");
        assert!(ctl.calib.is_none());
        // Profile restored and ryzen_smu reloaded exactly once — and the
        // reload is the LAST runner call, i.e. the calibration release
        // (profile toggle) happened before the guard's final restore.
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        assert_eq!(modprobe_reload_calls(&runner), 1);
        let calls = runner.calls();
        assert_eq!(
            calls.last().map(|(prog, _)| prog.as_str()),
            Some("modprobe"),
            "smu reload must come last: {calls:?}"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn full_calibration_persists_state_and_keeps_model() {
        let runner = FakeRunner::new();
        let (dir, profile) = profile_fixture("calib-full");
        let state_path = dir.join("state.json");
        let guard = RestoreGuard::new(
            &runner,
            Some(cpu_actuator(&runner, profile)),
            None,
            Some(SmuModule::assume_unloaded()),
        );
        let mut ctl = Controller::new(guard, PersistedState::default(), state_path.clone());
        assert!(ctl.model.is_none() && ctl.lut.is_none());

        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        for idx in 0..MATRIX_POINTS.len() {
            drive_matrix_point(&mut ctl, idx);
        }

        // Finished: back to Monitor, wizard gone, everything released.
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ctl.status().calib.is_none());
        assert!(ctl.burner.is_none());
        assert_eq!(ctl.status().cpu_limit_w, None);

        // The state file exists, parses and carries the fitted model + LUT;
        // the controller kept them for M4.
        let saved = PersistedState::load(&state_path);
        let model = saved.model.expect("model persisted");
        assert!((model.a - 25.0).abs() < 0.05 * 25.0, "a = {}", model.a);
        assert_eq!(saved.lut.expect("lut persisted").len(), 10);
        saved
            .calibrated_at
            .expect("calibrated_at set")
            .parse::<u64>()
            .expect("unix seconds");
        assert!(ctl.model.is_some() && ctl.lut.is_some());

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn persisted_state_seeds_controller_model_and_lut() {
        let runner = FakeRunner::new();
        let mut lut = ClockWattsLut::new();
        lut.insert(2000, 60.0);
        let persisted = PersistedState {
            model: None,
            lut: Some(lut.clone()),
            calibrated_at: None,
        };
        let ctl: Controller<&FakeRunner> = Controller::new(
            RestoreGuard::new(&runner, None, None, None),
            persisted,
            PathBuf::from("/nonexistent/state.json"),
        );
        assert_eq!(ctl.lut, Some(lut));
        assert!(ctl.model.is_none());
    }
}

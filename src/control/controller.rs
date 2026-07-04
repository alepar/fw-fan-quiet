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
use crate::calib::steady::{STEADY_N, STEADY_RPM_TOLERANCE, is_steady, tail_mean};
use crate::config::Config;
use crate::control::allocator::{self, AllocInput, Allocator};
use crate::control::gpu_pid::GpuPid;
use crate::control::lut::ClockWattsLut;
use crate::control::thermal_model::ThermalModel;
use crate::control::trim::{MAX_TRIM_AUTHORITY_RPM, Trim};
use crate::control::trust::{Trust, TrustMonitor};
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
/// Auto-mode allocator cadence (design §3: retarget the contour split every
/// 5 s; the GPU PI runs every sample in between).
const ALLOC_PERIOD_S: f64 = 5.0;
/// A sample must exceed the CPU limit by this margin to count as a
/// stickiness violation (RAPL vs STAPM accounting slack).
const STICKINESS_MARGIN_W: f64 = 5.0;
/// Consecutive violating samples before the stickiness watchdog fires.
const STICKINESS_SAMPLES: u8 = 3;
/// How long the `Resumed` flag stays visible after a suspend/resume.
const RESUMED_FLAG_S: f64 = 30.0;
/// Cap on the Auto-mode fan-RPM window feeding the trim integrator's
/// steadiness gate (`is_steady` needs STEADY_N=20; a little slack beyond
/// that is harmless).
const FAN_WINDOW_CAP: usize = 30;
/// `TargetUnreachable` clears once the trim offset drops below this fraction
/// of its +max — hysteresis so the flag doesn't flicker at the bound.
const TRIM_CLEAR_FRACTION: f64 = 0.9;
/// Online RLS forgetting factor (design §3: λ ≈ 0.99).
const RLS_LAMBDA: f64 = 0.99;
/// Auto-mode cadence of the "auto:model_snapshot" telemetry Decision
/// carrying the live a/b/e/c: per-line params would be too heavy, one line a
/// minute keeps the online-RLS trajectory reviewable offline.
const MODEL_SNAPSHOT_PERIOD_S: f64 = 60.0;
/// Trim gain scale while the trust monitor reports Distrust: keep absorbing
/// the acoustic error, but at half speed — the evidence is suspect.
const DISTRUST_TRIM_KI_SCALE: f64 = 0.5;
/// Fan target clamp range (RPM); the Auto-mode allocator consumes the
/// target live via `status.fan_target_rpm`.
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
    /// Fan target (RPM): stored, echoed in status, persisted to config on
    /// change, and consumed live by the Auto-mode allocator.
    SetFanTarget(f64),
    /// Enter/leave the closed-loop Auto mode. Explicit bool (not a toggle) so
    /// a queued duplicate keypress can never flip the mode back unnoticed.
    SetAuto(bool),
    /// Begin guided calibration (honored in Monitor mode only).
    StartCalibration,
    /// Abort a running calibration (release everything, back to Monitor).
    AbortCalibration,
    /// Restore hardware and exit the controller thread.
    Quit,
}

/// Controller mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Mode {
    #[default]
    Monitor,
    Manual,
    /// The calibration runner owns actuation; manual commands and the
    /// reassert/stickiness machinery are suspended.
    Calibrating,
    /// The closed loop owns actuation (allocator + GPU PI); manual setters
    /// are rejected, but the reassert/stickiness/resume machinery stays
    /// ACTIVE (it keys off `status.cpu_limit_w`/`gpu_max_mhz`, which hold
    /// the current auto allocation).
    Auto,
}

impl Mode {
    /// Telemetry/UI string form.
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Monitor => "monitor",
            Mode::Manual => "manual",
            Mode::Calibrating => "calibrating",
            Mode::Auto => "auto",
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
    /// Auto mode was requested without a calibrated model + LUT. Cleared on
    /// a successful Auto entry or when a calibration lands its fit.
    NotCalibrated,
    /// The trim integrator is pinned at its +max authority: the model
    /// persistently under-predicts and even the maximum budget cut cannot
    /// reach the fan target — fans above target, check intake/ambient
    /// (research 03 §6: surface a status when the floor is hit instead of
    /// silently collapsing performance). Clears once the offset drops below
    /// [`TRIM_CLEAR_FRACTION`] of max.
    TargetUnreachable,
    /// The trust monitor's verdict (Task 27): the model's steady-state
    /// residual EWMA has been over 300 RPM for 5+ minutes. While set, RLS
    /// updates are frozen and the trim runs at half gain. Clears when the
    /// EWMA recovers (steady evidence only) or on Auto exit.
    ModelDistrust,
}

impl StatusFlag {
    /// Telemetry string form.
    pub fn as_str(self) -> &'static str {
        match self {
            StatusFlag::LimitNotSticking => "limit_not_sticking",
            StatusFlag::Resumed => "resumed",
            StatusFlag::NotCalibrated => "not_calibrated",
            StatusFlag::TargetUnreachable => "target_unreachable",
            StatusFlag::ModelDistrust => "model_distrust",
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
    /// Stored fan target (RPM); the Auto-mode allocator consumes it live.
    pub fan_target_rpm: f64,
    /// Current trim offset (RPM); nonzero only in Auto mode. Positive =
    /// model under-predicts = budget cut (shown dim in the UI header).
    pub trim_rpm: f64,
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
            trim_rpm: 0.0,
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
    /// One Auto-mode allocator step ran (every 5 s): the WHY behind the
    /// resulting limits, mirrored into the Decision record's demand/alloc
    /// fields (cause "auto:allocate"). `gpu_w` is both the allocation and
    /// the PI's new watts target.
    AutoAllocated {
        demand_cpu: f64,
        demand_gpu: f64,
        cpu_w: f64,
        gpu_w: f64,
    },
    /// One online RLS update was ACCEPTED (Auto, steady sample, excitation
    /// gate open). The shell logs it as its OWN Decision record (cause
    /// "auto:rls") so an acceptance coinciding with e.g. a trim update never
    /// shadows either cause in offline review.
    RlsAccepted,
    /// Periodic (60 s) Auto-mode snapshot of the LIVE model parameters —
    /// its own Decision record (cause "auto:model_snapshot") carrying
    /// a/b/e/c for offline controller-quality review.
    ModelSnapshot { a: f64, b: f64, e: f64, c: f64 },
    /// Hardware restored; the thread shell must exit its loop.
    Quit,
}

/// Auto-mode loop state; Some exactly while `Mode::Auto`. Dropped whole on
/// exit, so re-entry always starts from a fresh PI (integrator cleared) and
/// a fresh allocator (conservative start) — the `reset()`s the design asks
/// for happen by construction.
struct AutoState {
    /// GPU watts→clock inner PI (1 Hz).
    pid: GpuPid,
    /// Contour allocator (every ALLOC_PERIOD_S).
    allocator: Allocator,
    /// t_mono of the last allocator step; None → step on the next sample.
    last_alloc: Option<f64>,
    /// Current PI watts target (allocator output); demand input next step.
    gpu_target_w: Option<f64>,
    /// Bounded ambient trim integrator (Task 26). Lives here so it resets
    /// on Auto exit (fresh on re-entry) but survives fan-target changes
    /// (ambient didn't change).
    trim: Trim,
    /// Fan-RPM window feeding the trim steadiness gate; fan-invalid samples
    /// land as NaN (the charts/steady.rs convention), so `is_steady` rejects
    /// any tail spanning a sensor outage — never integrate across one.
    fan_window: std::collections::VecDeque<f64>,
    /// Model trust monitor (Task 27), fed the steady-gated |residual|. Lives
    /// here so trust state resets on Auto exit, like the trim.
    trust: TrustMonitor,
    /// Latest trust verdict; stands between steady windows (no evidence, no
    /// change). While true: RLS frozen, trim at [`DISTRUST_TRIM_KI_SCALE`].
    distrusted: bool,
    /// t_mono of the last model_snapshot record; None → snapshot on the next
    /// sample (Auto entry logs the baseline params immediately).
    last_snapshot: Option<f64>,
}

impl AutoState {
    fn new() -> Self {
        Self {
            pid: GpuPid::new(),
            allocator: Allocator::new(),
            last_alloc: None,
            gpu_target_w: None,
            trim: Trim::new(),
            fan_window: std::collections::VecDeque::new(),
            trust: TrustMonitor::new(),
            distrusted: false,
            last_snapshot: None,
        }
    }
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
    /// replaced by a fresh calibration; inverted to the target-RPM contour
    /// by the Auto-mode allocator.
    model: Option<ThermalModel>,
    /// GPU clock→watts LUT, same lifecycle as `model`; the GPU PI's
    /// feedforward.
    lut: Option<ClockWattsLut>,
    /// User config (fan target, floors, fast limit). Mutated + saved when
    /// the fan target changes.
    config: Config,
    /// Where `config` persists to (`--config`).
    config_path: PathBuf,
    /// Auto-mode loop state; Some exactly while `Mode::Auto`.
    auto: Option<AutoState>,
}

impl<R: Runner> Controller<R> {
    pub fn new(
        mut guard: RestoreGuard<R>,
        persisted: PersistedState,
        state_path: PathBuf,
        config: Config,
        config_path: PathBuf,
    ) -> Self {
        // Belt and suspenders (Config::load already sanitizes): out-of-range
        // floors would trip the GPU PI's clamp / the allocator's debug
        // assert once Auto starts. No construction path may skip this.
        let config = config.sanitized();
        // Config owns the burst ceiling; the actuator default only covers a
        // hypothetical config-less construction.
        if let Some(cpu) = guard.cpu.as_mut() {
            cpu.fast_limit_mw = config.fast_limit_mw;
        }
        let status = ControlStatus {
            fan_target_rpm: config
                .fan_target_rpm
                .clamp(FAN_TARGET_MIN_RPM, FAN_TARGET_MAX_RPM),
            ..ControlStatus::default()
        };
        Self {
            guard,
            status,
            stick_violations: 0,
            resumed_until: None,
            last_reassert: None,
            calib: None,
            burner: None,
            state_path,
            model: persisted.model,
            lut: persisted.lut,
            config,
            config_path,
            auto: None,
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
        // While calibrating the runner owns actuation: manual setters,
        // release and Auto entry are rejected outright (Esc/AbortCalibration
        // is the way to take control back).
        if self.status.mode == Mode::Calibrating
            && matches!(
                c,
                Command::SetCpuW(_)
                    | Command::SetGpuMaxClock(_)
                    | Command::ReleaseAll
                    | Command::SetAuto(_)
            )
        {
            tracing::warn!("manual command rejected while calibrating: {c:?}");
            return Vec::new();
        }
        // While in Auto the closed loop owns actuation: manual setters are
        // rejected (SetFanTarget stays allowed — it retargets the contour
        // live; ReleaseAll/SetAuto(false) are the ways out).
        if self.status.mode == Mode::Auto
            && matches!(c, Command::SetCpuW(_) | Command::SetGpuMaxClock(_))
        {
            tracing::warn!("manual command rejected while in auto mode: {c:?}");
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
                            // Fresh command = fresh assert: any violation
                            // streak against the previous limit is stale.
                            self.stick_violations = 0;
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
                // Exiting Auto too: the loop state drops whole, so a later
                // re-entry starts from a fresh PI + conservative allocator.
                self.auto = None;
                self.release_to_stock();
                effects.push(Effect::Released);
                "release"
            }
            Command::SetFanTarget(rpm) => {
                let clamped = rpm.clamp(FAN_TARGET_MIN_RPM, FAN_TARGET_MAX_RPM);
                if clamped != self.status.fan_target_rpm {
                    self.status.fan_target_rpm = clamped;
                    // The Auto allocator reads status.fan_target_rpm on its
                    // next step: no extra wiring needed for a live retarget.
                    // Persist on CHANGE only (a held key repeats the same
                    // clamped value at the bounds — never spam the disk);
                    // save failure is warned, the in-session target applies.
                    self.config.fan_target_rpm = clamped;
                    if let Err(e) = self.config.save(&self.config_path) {
                        tracing::warn!(
                            "config save to {} failed (fan target still active): {e}",
                            self.config_path.display()
                        );
                    }
                }
                "command:set_fan_target"
            }
            Command::SetAuto(true) => {
                if self.status.mode == Mode::Auto {
                    tracing::warn!("SetAuto(true) ignored: already in auto mode");
                    "auto:on"
                } else if self.model.is_none() || self.lut.is_none() {
                    tracing::warn!(
                        "auto mode requires a calibrated model + LUT; run a calibration (k) first"
                    );
                    self.add_flag(StatusFlag::NotCalibrated);
                    "auto:not_calibrated"
                } else {
                    self.remove_flag(StatusFlag::NotCalibrated);
                    let mut auto = AutoState::new();
                    match self.guard.gpu.as_ref() {
                        // Carried-over review decision: with a GPU lock
                        // applied right now (e.g. entering from Manual),
                        // seed the PI's rate-limit reference from it so the
                        // first PI command cannot jump >105 MHz from what is
                        // in force. With no lock applied (stock), the PI's
                        // documented first-jump-to-feedforward is safe — it
                        // only moves DOWNWARD from the stock 3090 MHz.
                        Some(gpu) => auto.pid.seed_last_clock(gpu.applied()),
                        None => tracing::warn!(
                            "no GPU actuator this run: auto mode will shape the CPU only"
                        ),
                    }
                    self.auto = Some(auto);
                    self.status.mode = Mode::Auto;
                    // Any manual limits stay in force for <1 s: the first
                    // sample runs the allocator, which starts from its
                    // conservative start and replaces them.
                    "auto:on"
                }
            }
            Command::SetAuto(false) => {
                if self.status.mode == Mode::Auto {
                    self.auto = None; // fresh PI/allocator on re-entry
                    self.release_to_stock();
                    effects.push(Effect::Released);
                } else {
                    tracing::warn!("SetAuto(false) ignored: not in auto mode");
                }
                "auto:off"
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
            // The pre-suspend fan window is thermally stale (the machine
            // cooled while asleep) — clear it so the trim integrator can't
            // fire on a 20-sample tail spanning the suspend (review finding).
            if let Some(auto) = &mut self.auto {
                auto.fan_window.clear();
            }
            cause.get_or_insert("resume");
        } else if self.resumed_until.is_some_and(|until| s.t_mono >= until) {
            self.remove_flag(StatusFlag::Resumed);
            self.resumed_until = None;
            cause.get_or_insert("resume");
        }

        // Auto loop (allocator + GPU PI) BEFORE the stickiness/reassert
        // machinery, so the watchdogs below see the freshly commanded
        // allocation. Unlike Calibrating, those watchdogs stay ACTIVE in
        // Auto: they key off status.cpu_limit_w/gpu_max_mhz, which hold the
        // current auto allocation — reassert reapplies it, stickiness
        // watches it, resume (above) restores it.
        if self.status.mode == Mode::Auto {
            self.on_auto_sample(s, &mut effects, &mut cause);
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

    /// One Auto-mode control step, driven off the 1 Hz samples (t_mono-based
    /// like the reassert): every [`ALLOC_PERIOD_S`] an allocator step
    /// retargets both devices on the fan-target contour; every sample the
    /// GPU watts→clock PI trims the locked clock toward its watts target.
    ///
    /// Sensor-loss semantics: the PI is driven by the GPU WATTS sensor, not
    /// the fan — a lost fan sensor freezes the allocator (inside
    /// `Allocator::step`, which then never raises power past the floor) but
    /// the PI keeps holding the last watts target off `gpu_w`; a lost GPU
    /// watts sensor holds the PI (skip) while the allocator keeps running.
    fn on_auto_sample(
        &mut self,
        s: &Sample,
        effects: &mut Vec<Effect>,
        cause: &mut Option<&'static str>,
    ) {
        // Defensive: Auto without its state/model cannot control anything —
        // fail toward stock (every failure path degrades to louder fans or
        // stock behavior, design §5). Unreachable in practice: entry
        // requires model+LUT and they are only ever replaced, never cleared.
        if self.auto.is_none() || self.model.is_none() || self.lut.is_none() {
            tracing::warn!("auto mode lost its state/model; releasing to Monitor");
            self.auto = None;
            self.release_to_stock();
            effects.push(Effect::Released);
            cause.get_or_insert("auto:degraded");
            return;
        }
        let auto = self.auto.as_mut().expect("checked above");
        let model = self.model.as_ref().expect("checked above");
        let lut = self.lut.as_ref().expect("checked above");

        // Trim steadiness window: fan-invalid samples land as NaN (the
        // steady.rs convention) so the 20-sample steady tail can never span
        // a sensor outage — an outage restarts the settling clock.
        if auto.fan_window.len() >= FAN_WINDOW_CAP {
            auto.fan_window.pop_front();
        }
        auto.fan_window.push_back(if s.fan_valid {
            s.max_fan_rpm()
        } else {
            f64::NAN
        });

        // Allocator step, every ALLOC_PERIOD_S (first sample after entry
        // included: last_alloc starts None).
        if auto
            .last_alloc
            .is_none_or(|last| s.t_mono - last >= ALLOC_PERIOD_S)
        {
            auto.last_alloc = Some(s.t_mono);
            let demand = allocator::demand(
                s,
                self.status.cpu_limit_w,
                auto.gpu_target_w,
                self.status.gpu_max_mhz,
            );
            let target_rpm = self.status.fan_target_rpm;
            // Positive trim shifts the contour down (fewer watts): the model
            // under-predicted, so the real machine needs a smaller budget to
            // hit the target. Floors still win — the allocator/PI clamps
            // bound the trim's effect (design invariant: floors > trim).
            let trim_rpm = auto.trim.offset_rpm();
            let contour = |pc: f64| model.gpu_watts_on_contour(target_rpm, trim_rpm, pc);
            let (cpu_w, gpu_w) = auto.allocator.step(&AllocInput {
                contour: &contour,
                demand,
                floors: (self.config.cpu_floor_w, self.config.gpu_floor_mhz),
                measured_fan_rpm: s.max_fan_rpm(),
                fan_target_rpm: target_rpm,
                fan_valid: s.fan_valid,
            });
            // Bumpless retarget: the PI keeps its trim + rate reference.
            auto.pid.set_target_w(gpu_w);
            auto.gpu_target_w = Some(gpu_w);
            // Command the CPU only when the allocation moved: a held/frozen
            // allocation (deadband, lost fan) must not re-command at 5 s
            // cadence — the 10 s reassert already defends the applied value.
            if self.status.cpu_limit_w != Some(cpu_w) {
                match self.guard.cpu.as_ref() {
                    None => {
                        tracing::warn!("auto: no CPU actuator; allocation {cpu_w} W not applied");
                    }
                    Some(cpu) => match cpu.set_sustained_mw((cpu_w * 1000.0).round() as u32) {
                        Ok(clamped_mw) => {
                            let clamped_w = f64::from(clamped_mw) / 1000.0;
                            self.status.cpu_limit_w = Some(clamped_w);
                            // A violation streak measured against the OLD
                            // limit is stale evidence: the fresh allocation
                            // gets a full 3-sample streak before the
                            // stickiness watchdog may fire.
                            self.stick_violations = 0;
                            effects.push(Effect::CpuSet(clamped_w));
                        }
                        Err(e) => {
                            tracing::warn!("auto: CPU allocation ({cpu_w} W) failed: {e}");
                        }
                    },
                }
            }
            effects.push(Effect::AutoAllocated {
                demand_cpu: demand.cpu_starved,
                demand_gpu: demand.gpu_starved,
                cpu_w,
                gpu_w,
            });
            cause.get_or_insert("auto:allocate");
        }

        // GPU PI, every sample. Invalid NVML watts → skip (the PI holds; it
        // must never chase a phantom reading). update() returning None is
        // the in-deadband hold — no command, no Decision (1 Hz spam guard).
        if s.gpu_w_valid
            && let Some(clock) = auto.pid.update(s.gpu_w, lut, self.config.gpu_floor_mhz)
        {
            match self.guard.gpu.as_mut() {
                // Warned once at Auto entry, not here (1 Hz spam).
                None => {}
                Some(gpu) => match gpu.set_max_clock(clock) {
                    Ok(()) => {
                        let applied = gpu.applied();
                        // Same clock re-commanded (e.g. pinned at the floor)
                        // changes nothing user-visible: no status, no record.
                        if applied != self.status.gpu_max_mhz {
                            self.status.gpu_max_mhz = applied;
                            effects.push(Effect::GpuSet(applied.unwrap_or(clock)));
                            cause.get_or_insert("auto:gpu_clock");
                        }
                    }
                    Err(e) => {
                        tracing::warn!("auto: GPU clock ({clock} MHz) failed: {e}");
                        // PI honesty: update() already committed `clock` as
                        // its rate-limit reference, but the hardware still
                        // holds the old lock (or none). Re-seed from what is
                        // actually applied so the next command rate-limits
                        // from hardware state, not from failed intent.
                        auto.pid.seed_last_clock(self.status.gpu_max_mhz);
                    }
                },
            }
        }

        // Online adaptation tier, last (after allocator + PI, so it sees
        // this sample's allocation): trim + RLS + trust (Tasks 26/27). All
        // three share ONE gate: only on fan-valid samples whose 20-sample
        // window is steady — never adapt on transients or lost sensors
        // (design invariant; non-steady/invalid samples freeze this tier
        // exactly like they freeze the allocator).
        //
        // Separation of concerns: the trim absorbs offset drift FAST (hard-
        // bounded at ±400 RPM) while RLS reshapes the a/b/e/c surface only
        // under excitation — at a CONSTANT operating point `rls_update`'s
        // excitation gate rejects everything and ONLY the trim moves, so the
        // two cannot fight over the same steady-state error (the intended
        // split: trim = offset, RLS = shape). Under excitation both DO
        // integrate the same offset; the overlap is accepted: RLS absorbs
        // it into the surface, and the trim's leftover (bounded ≤ 400 RPM)
        // simply freezes with the near-zero residual until an error sign
        // flip walks it back.
        //
        // predicted = the PRE-update model at the CURRENT operating point
        // (applied CPU allocation, PI watts target): the residual measures
        // the model we are currently controlling with; trust and trim
        // consume it, THEN RLS adapts.
        //
        // Persistence semantics: RLS mutates `self.model` in place, so the
        // allocator's contour uses the adapted surface LIVE — but nothing
        // here saves it. The state file keeps the CALIBRATED parameters
        // (written only by a finished calibration): online adaptation is
        // session-only, and a restart reverts to calibrated + fresh
        // adaptation.
        if s.fan_valid
            && is_steady(
                auto.fan_window.make_contiguous(),
                STEADY_N,
                STEADY_RPM_TOLERANCE,
            )
            && let (Some(cpu_w), Some(gpu_w)) = (self.status.cpu_limit_w, auto.gpu_target_w)
        {
            let model = self.model.as_mut().expect("checked above");
            let predicted = model.predict(cpu_w, gpu_w);
            // Integrate the steady tail's mean, not the single latest sample:
            // the ±100 RPM steadiness tolerance would otherwise leak ±5 RPM of
            // per-update noise into the trim (review nit).
            let measured = tail_mean(auto.fan_window.make_contiguous(), STEADY_N)
                .unwrap_or_else(|| s.max_fan_rpm());
            // Trust verdict first: it decides whether this very sample may
            // adapt the model. Between steady windows the last verdict
            // stands (no evidence, no change).
            auto.distrusted =
                auto.trust.observe(s.t_mono, (measured - predicted).abs()) == Trust::Distrust;
            // RLS, frozen while distrusted: adapting toward readings we no
            // longer trust would launder the fault into the model. The
            // excitation + covariance gates live inside `rls_update`.
            if !auto.distrusted && model.rls_update(cpu_w, gpu_w, measured, RLS_LAMBDA) {
                effects.push(Effect::RlsAccepted);
            }
            // Trim last, at half gain while distrusted.
            let ki_scale = if auto.distrusted {
                DISTRUST_TRIM_KI_SCALE
            } else {
                1.0
            };
            if auto
                .trim
                .update_scaled(s.t_mono, measured, predicted, ki_scale)
            {
                self.status.trim_rpm = auto.trim.offset_rpm();
                cause.get_or_insert("auto:trim");
            }
        }
        let distrusted = auto.distrusted;
        let offset = auto.trim.offset_rpm();
        // Model snapshot cadence check here (while `auto` is borrowed); the
        // effect is pushed below, after the flag edits release the borrow.
        let snapshot_due = auto
            .last_snapshot
            .is_none_or(|last| s.t_mono - last >= MODEL_SNAPSHOT_PERIOD_S);
        if snapshot_due {
            auto.last_snapshot = Some(s.t_mono);
        }

        // ModelDistrust flag mirrors the trust verdict (transitions only).
        let flagged = self.status.flags.contains(&StatusFlag::ModelDistrust);
        if distrusted && !flagged {
            self.add_flag(StatusFlag::ModelDistrust);
            cause.get_or_insert("auto:distrust");
        } else if !distrusted && flagged {
            self.remove_flag(StatusFlag::ModelDistrust);
            cause.get_or_insert("auto:distrust_cleared");
        }

        // Saturated at +max: even the maximum budget cut cannot reach the
        // target — surface it instead of silently losing performance
        // (research 03 §6). Hysteresis: clears below 90% of max. The cause
        // is claimed only on an actual flag TRANSITION, so a later stage's
        // status change in the same sample can't get mislabeled "auto:trim".
        let flagged = self.status.flags.contains(&StatusFlag::TargetUnreachable);
        if offset >= MAX_TRIM_AUTHORITY_RPM && !flagged {
            self.add_flag(StatusFlag::TargetUnreachable);
            cause.get_or_insert("auto:trim");
        } else if offset < TRIM_CLEAR_FRACTION * MAX_TRIM_AUTHORITY_RPM && flagged {
            self.remove_flag(StatusFlag::TargetUnreachable);
            cause.get_or_insert("auto:trim");
        }

        // Periodic model snapshot for offline review (own Decision record;
        // see Effect::ModelSnapshot). Emitted from the first Auto sample —
        // the baseline the later lines are read against.
        if snapshot_due {
            let m = self.model.as_ref().expect("checked above");
            effects.push(Effect::ModelSnapshot {
                a: m.a,
                b: m.b,
                e: m.e,
                c: m.c,
            });
        }
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
                    if self.model.is_some() && self.lut.is_some() {
                        // A landed fit satisfies the Auto-entry requirement.
                        self.remove_flag(StatusFlag::NotCalibrated);
                    }
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

    /// Back to Monitor with stock limits, actuators kept (the session goes
    /// on). NOT `guard.restore_all()`: the smu module must stay unloaded.
    /// Shared by ReleaseAll and the Auto-mode exit.
    fn release_to_stock(&mut self) {
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
        // Trim + trust state live in AutoState (dropped by every Auto exit
        // path before reaching here); mirror the resets into the visible
        // status.
        self.status.trim_rpm = 0.0;
        self.remove_flag(StatusFlag::TargetUnreachable);
        self.remove_flag(StatusFlag::ModelDistrust);
        self.remove_flag(StatusFlag::LimitNotSticking);
        self.stick_violations = 0;
        self.last_reassert = None;
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
            // Initial status push: the config-seeded fan target must show in
            // the UI before the first change-driven Status event (the model's
            // built-in default only matches a default config).
            let _ = ui_tx.send(Event::Status(controller.status().clone()));
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
    // (demand_cpu, demand_gpu, alloc_cpu_w, alloc_gpu_w) from an Auto-mode
    // allocator step in this batch; the WHY behind an "auto:allocate" record.
    let mut auto_alloc: Option<(f64, f64, f64, f64)> = None;
    let mut rls_accepted = false;
    let mut model_snapshot: Option<(f64, f64, f64, f64)> = None;
    for effect in effects {
        match effect {
            Effect::Reasserted { cause: c } | Effect::Noted { cause: c } => {
                cause.get_or_insert(c);
            }
            Effect::StatusChanged { cause: c } => {
                status_changed = true;
                cause.get_or_insert(c);
            }
            Effect::AutoAllocated {
                demand_cpu,
                demand_gpu,
                cpu_w,
                gpu_w,
            } => {
                auto_alloc = Some((*demand_cpu, *demand_gpu, *cpu_w, *gpu_w));
                cause.get_or_insert("auto:allocate");
            }
            Effect::RlsAccepted => rls_accepted = true,
            Effect::ModelSnapshot { a, b, e, c } => model_snapshot = Some((*a, *b, *e, *c)),
            Effect::Quit => quit = true,
            Effect::CpuSet(_) | Effect::GpuSet(_) | Effect::Released => {}
        }
    }
    let status = controller.status();
    if status_changed {
        // A send failure means the UI is gone; shutdown is already underway.
        let _ = ui_tx.send(Event::Status(status.clone()));
    }
    // One "main" Decision per batch (whatever claimed the cause first), plus
    // STANDALONE records for an RLS acceptance and/or model snapshot in the
    // same batch — separate lines, so neither cause can shadow the other in
    // offline review.
    let decision = |cause: &'static str,
                    alloc: Option<(f64, f64, f64, f64)>,
                    model: Option<(f64, f64, f64, f64)>| {
        Record::Decision {
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
            demand_cpu: alloc.map(|a| a.0),
            demand_gpu: alloc.map(|a| a.1),
            alloc_cpu_w: alloc.map(|a| a.2),
            alloc_gpu_w: alloc.map(|a| a.3),
            // The allocator's gpu_w IS the PI target (set_target_w).
            pi_target_w: alloc.map(|a| a.3),
            // Every Auto-mode decision carries the current trim (offline
            // analysis wants the trim context on allocate lines too);
            // non-auto lines skip it to stay lean.
            trim_rpm: (status.mode == Mode::Auto).then_some(status.trim_rpm),
            model_a: model.map(|m| m.0),
            model_b: model.map(|m| m.1),
            model_e: model.map(|m| m.2),
            model_c: model.map(|m| m.3),
        }
    };
    if cause.is_some() || rls_accepted || model_snapshot.is_some() {
        if let Some(t) = telemetry::lock(telemetry).as_mut() {
            if let Some(cause) = cause {
                t.log(&decision(cause, auto_alloc, None));
            }
            if rls_accepted {
                t.log(&decision("auto:rls", None, None));
            }
            if let Some(m) = model_snapshot {
                t.log(&decision("auto:model_snapshot", None, Some(m)));
            }
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

    /// Controller over a FakeRunner-backed CPU actuator (no GPU — Auto tests
    /// pass a FakeGpu explicitly) and an smu module "we unloaded" (so Quit's
    /// reload shows up).
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
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
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
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
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
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
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

        // The shell pushes one initial Status at startup (so a config-seeded
        // fan target shows before any change); the command's echo follows.
        match ui_rx.recv_timeout(Duration::from_secs(3)) {
            Ok(Event::Status(st)) => {
                assert_eq!(st.cpu_limit_w, None, "initial status precedes commands");
                assert_eq!(st.mode, Mode::Monitor);
            }
            other => panic!("expected the initial Event::Status, got {other:?}"),
        }
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
            Command::SetAuto(true),
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
        let mut ctl = Controller::new(
            guard,
            PersistedState::default(),
            state_path.clone(),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
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
        // the controller kept them, so Auto mode can start right away.
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

    // --- Task 25: auto mode ---

    use crate::actuators::gpu::test_support::{FakeGpu, GpuCall};
    use crate::control::thermal_model::CalibPoint;

    /// Truth surface `rpm = 25·pc + 15·pg + 0.1·pc·pg + 800` (the same one
    /// the thermal_model tests use), fit into a model.
    fn fitted_model() -> ThermalModel {
        let pts: Vec<CalibPoint> = [
            (5.0, 0.0),
            (45.0, 0.0),
            (5.0, 100.0),
            (45.0, 100.0),
            (20.0, 40.0),
        ]
        .iter()
        .map(|&(pc, pg)| CalibPoint {
            cpu_w: pc,
            gpu_w: pg,
            rpm: 25.0 * pc + 15.0 * pg + 0.1 * pc * pg + 800.0,
        })
        .collect();
        ThermalModel::fit_batch(&pts).unwrap()
    }

    /// 3-point LUT: 30 W @ 1200, 60 W @ 2000, 100 W @ 2800 MHz.
    fn lut3() -> ClockWattsLut {
        let mut lut = ClockWattsLut::new();
        lut.insert(1200, 30.0);
        lut.insert(2000, 60.0);
        lut.insert(2800, 100.0);
        lut
    }

    fn calibrated() -> PersistedState {
        PersistedState {
            model: Some(fitted_model()),
            lut: Some(lut3()),
            calibrated_at: None,
        }
    }

    /// Calibrated controller with a CPU actuator AND a FakeGpu; also returns
    /// the GPU call-log handle.
    fn auto_controller(
        runner: &FakeRunner,
        profile_path: PathBuf,
        config: Config,
    ) -> (Controller<&FakeRunner>, Arc<Mutex<Vec<GpuCall>>>) {
        let gpu = FakeGpu::new();
        let gpu_calls = gpu.calls();
        let ctl = Controller::new(
            RestoreGuard::new(
                runner,
                Some(cpu_actuator(runner, profile_path)),
                Some(Box::new(gpu)),
                None,
            ),
            calibrated(),
            PathBuf::from("/nonexistent/state.json"),
            config,
            PathBuf::from("/nonexistent/config.toml"),
        );
        (ctl, gpu_calls)
    }

    fn auto_controller_no_profile(
        runner: &FakeRunner,
    ) -> (Controller<&FakeRunner>, Arc<Mutex<Vec<GpuCall>>>) {
        auto_controller(
            runner,
            PathBuf::from("/nonexistent/platform_profile"),
            Config::default(),
        )
    }

    /// A gaming-ish sample: both devices busy, fan well below the 3000 RPM
    /// target, GPU drawing 10 W (far under any target: the PI must raise the
    /// clock). `cpu_pkg_w` stays 0 (RAPL-warmup semantics) so the stickiness
    /// watchdog stays quiet and CPU demand falls back to utilization.
    fn busy_at(t: f64) -> Sample {
        Sample {
            t_mono: t,
            cpu_util_pct: 100.0,
            gpu_util_pct: 100.0,
            gpu_w: 10.0,
            gpu_w_valid: true,
            fan1_rpm: 1700.0,
            fan_valid: true,
            ..Sample::default()
        }
    }

    fn gpu_sets(calls: &Mutex<Vec<GpuCall>>) -> Vec<u32> {
        calls
            .lock()
            .unwrap()
            .iter()
            .filter_map(|c| match c {
                GpuCall::Set(mhz) => Some(*mhz),
                GpuCall::Release => None,
            })
            .collect()
    }

    fn alloc_of(effects: &[Effect]) -> Option<(f64, f64)> {
        effects.iter().find_map(|e| match e {
            Effect::AutoAllocated { cpu_w, gpu_w, .. } => Some((*cpu_w, *gpu_w)),
            _ => None,
        })
    }

    #[test]
    fn auto_entry_without_model_flags_not_calibrated() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner); // uncalibrated

        let effects = ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Monitor, "must stay in Monitor");
        assert!(ctl.status().flags.contains(&StatusFlag::NotCalibrated));
        assert_eq!(status_changes(&effects), 1);
        assert!(ctl.auto.is_none());
        assert!(runner.calls().is_empty(), "no actuation on a refused entry");

        // Samples keep flowing through the plain Monitor path.
        assert!(ctl.on_sample(&busy_at(0.0)).is_empty());
        assert!(runner.calls().is_empty());
    }

    #[test]
    fn auto_entry_allocates_cpu_and_drives_gpu_pi() {
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls) = auto_controller_no_profile(&runner);

        let effects = ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Auto);
        assert_eq!(status_changes(&effects), 1);
        assert!(!ctl.status().flags.contains(&StatusFlag::NotCalibrated));
        assert!(
            ryzenadj_calls(&runner).is_empty(),
            "entry itself must not actuate; the first sample does"
        );

        // First sample: the allocator steps from the conservative (15, 30)
        // start toward the both-starved optimum, up-rate-limited to
        // (17, 32) → ryzenadj 17 W + PI target 32 W. The PI (measured 10 W)
        // commands FF(32 W) = 1253 MHz + the clamped +400 MHz correction.
        let effects = ctl.on_sample(&busy_at(0.0));
        assert_eq!(alloc_of(&effects), Some((17.0, 32.0)));
        assert_eq!(ryzenadj_calls(&runner), vec![expected_args(17_000)]);
        assert_eq!(ctl.status().cpu_limit_w, Some(17.0));
        assert!(effects.contains(&Effect::CpuSet(17.0)), "got {effects:?}");
        assert_eq!(gpu_sets(&gpu_calls), vec![1653]);
        assert_eq!(ctl.status().gpu_max_mhz, Some(1653));
        assert!(effects.contains(&Effect::GpuSet(1653)), "got {effects:?}");
        assert_eq!(status_changes(&effects), 1);
    }

    #[test]
    fn allocator_runs_every_5s_not_every_sample() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        let mut alloc_times = Vec::new();
        for t in [0.0, 1.0, 2.0, 3.0, 4.0, 4.9, 5.0, 6.0, 9.9] {
            let effects = ctl.on_sample(&busy_at(t));
            if alloc_of(&effects).is_some() {
                alloc_times.push(t);
            }
        }
        assert_eq!(
            alloc_times,
            vec![0.0, 5.0],
            "allocator must act on entry and every 5 s, not per sample"
        );
        // Each acting step changed the allocation → exactly one ryzenadj
        // command per step.
        assert_eq!(ryzenadj_calls(&runner).len(), 2);
    }

    #[test]
    fn fan_target_change_shifts_next_allocation() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0)); // (17, 32)
        let effects = ctl.on_sample(&busy_at(5.0));
        assert_eq!(alloc_of(&effects), Some((19.0, 34.0)), "premise");

        // Live retarget: 1000 RPM turns the measured 1700 RPM into a 700 RPM
        // overshoot on a collapsed contour — ups forbidden, hard cuts.
        ctl.on_command(Command::SetFanTarget(1000.0));
        let effects = ctl.on_sample(&busy_at(10.0));
        assert_eq!(
            alloc_of(&effects),
            Some((19.0, 18.0)),
            "next step must consume the new target (gpu cut at the overshoot rate)"
        );
    }

    #[test]
    fn config_floor_honored_with_zero_demand() {
        let runner = FakeRunner::new();
        let config = Config {
            cpu_floor_w: 20.0,
            ..Config::default()
        };
        let (mut ctl, _gpu_calls) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            config,
        );
        ctl.on_command(Command::SetAuto(true));

        // Fully idle machine, fan already sitting at the target.
        let idle_at = |t: f64| Sample {
            t_mono: t,
            gpu_w: 10.0,
            gpu_w_valid: true,
            fan1_rpm: 3000.0,
            fan_valid: true,
            ..Sample::default()
        };
        for t in [0.0, 5.0, 10.0, 15.0] {
            let effects = ctl.on_sample(&idle_at(t));
            if let Some((cpu_w, _)) = alloc_of(&effects) {
                assert!(cpu_w >= 20.0, "allocation {cpu_w} W fell below the floor");
            }
        }
        assert_eq!(ctl.status().cpu_limit_w, Some(20.0));
        // Every ryzenadj command (allocations AND reasserts) honored it.
        for args in ryzenadj_calls(&runner) {
            let mw: u32 = args[0]
                .strip_prefix("--stapm-limit=")
                .unwrap()
                .parse()
                .unwrap();
            assert!(mw >= 20_000, "commanded {mw} mW below the 20 W floor");
        }
    }

    #[test]
    fn manual_and_calibration_commands_rejected_in_auto() {
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        let cpu_calls_before = ryzenadj_calls(&runner).len();
        let gpu_calls_before = gpu_sets(&gpu_calls).len();

        for cmd in [Command::SetCpuW(40.0), Command::SetGpuMaxClock(3000)] {
            let effects = ctl.on_command(cmd);
            assert!(effects.is_empty(), "{cmd:?} must be rejected: {effects:?}");
        }
        let effects = ctl.on_command(Command::StartCalibration);
        assert!(effects.is_empty(), "got {effects:?}");
        assert_eq!(ctl.status().mode, Mode::Auto);
        assert!(ctl.status().calib.is_none());
        assert_eq!(ryzenadj_calls(&runner).len(), cpu_calls_before);
        assert_eq!(gpu_sets(&gpu_calls).len(), gpu_calls_before);

        // SetFanTarget stays allowed: it retargets the contour live.
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(ctl.status().fan_target_rpm, 2500.0);
        assert_eq!(ctl.status().mode, Mode::Auto);
    }

    #[test]
    fn set_auto_false_releases_to_stock_and_resets_loop_state() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("auto-exit");
        let (mut ctl, gpu_calls) = auto_controller(&runner, path.clone(), Config::default());
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert!(ctl.status().cpu_limit_w.is_some(), "premise: limits active");

        let effects = ctl.on_command(Command::SetAuto(false));
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert_eq!(ctl.status().gpu_max_mhz, None);
        assert!(ctl.auto.is_none());
        // GPU locks released + CPU stock restored (profile toggled back);
        // the smu module stays untouched — the session keeps running.
        assert_eq!(gpu_calls.lock().unwrap().last(), Some(&GpuCall::Release));
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        assert_eq!(modprobe_reload_calls(&runner), 0);

        // Re-entry starts FRESH: conservative allocator start and a clean PI
        // (rate reference cleared with the released lock).
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&busy_at(100.0));
        assert_eq!(alloc_of(&effects), Some((17.0, 32.0)));

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn reassert_stays_active_in_auto() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0)); // alloc 17 W; reassert baseline t=0
        assert_eq!(ryzenadj_calls(&runner).len(), 1);

        // 10.1 s later: the allocator retargets (19 W) AND the periodic
        // reassert fires, re-commanding the fresh auto allocation.
        let effects = ctl.on_sample(&busy_at(10.1));
        assert!(has_reassert(&effects, "reassert"), "got {effects:?}");
        assert_eq!(
            ryzenadj_calls(&runner),
            vec![
                expected_args(17_000),
                expected_args(19_000),
                expected_args(19_000)
            ],
            "reassert must reapply the CURRENT auto allocation"
        );
    }

    #[test]
    fn fan_invalid_freezes_allocator_but_pi_keeps_working() {
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        let invalid_fan_at = |t: f64| Sample {
            fan_valid: false,
            ..busy_at(t)
        };
        // First step: frozen at the conservative start — a lost fan sensor
        // must never raise power. The one initial command applies it.
        let effects = ctl.on_sample(&invalid_fan_at(0.0));
        assert_eq!(alloc_of(&effects), Some((15.0, 30.0)));
        assert_eq!(ryzenadj_calls(&runner), vec![expected_args(15_000)]);

        // Later allocator steps stay frozen: no new CPU command...
        let effects = ctl.on_sample(&invalid_fan_at(5.0));
        assert_eq!(alloc_of(&effects), Some((15.0, 30.0)));
        assert_eq!(
            ryzenadj_calls(&runner).len(),
            1,
            "frozen: no 5 s re-command"
        );

        // ...besides the 10 s reassert, which keeps defending what is applied.
        let effects = ctl.on_sample(&invalid_fan_at(10.1));
        assert!(has_reassert(&effects, "reassert"), "got {effects:?}");
        assert_eq!(ryzenadj_calls(&runner).len(), 2);
        assert_eq!(ctl.status().cpu_limit_w, Some(15.0));

        // The PI runs off the GPU WATTS sensor, not the fan: it kept driving
        // the clock toward the 30 W target the whole time.
        assert!(
            !gpu_sets(&gpu_calls).is_empty(),
            "PI must keep working off gpu_w while the fan is lost"
        );
        assert_eq!(ctl.status().gpu_max_mhz, Some(1600)); // FF(30)=1200 + 400
    }

    #[test]
    fn gpu_pi_seeded_from_applied_lock_at_entry() {
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls) = auto_controller_no_profile(&runner);
        // A manual lock is in force when Auto starts.
        ctl.on_command(Command::SetGpuMaxClock(1500));
        assert_eq!(ctl.status().gpu_max_mhz, Some(1500), "premise");
        ctl.on_command(Command::SetAuto(true));

        ctl.on_sample(&busy_at(0.0));
        let sets = gpu_sets(&gpu_calls);
        assert_eq!(sets[0], 1500, "the manual lock");
        let first_pi = sets[1];
        assert!(
            first_pi.abs_diff(1500) <= 105,
            "first PI command ({first_pi} MHz) jumped >105 MHz from the applied 1500"
        );
        // Concretely: rate-limited toward the 1653 MHz desired clock.
        assert_eq!(first_pi, 1605);
    }

    #[test]
    fn set_fan_target_persists_config_on_change_only() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-fan-persist",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let config_path = dir.join("config.toml");
        let mut ctl: Controller<&FakeRunner> = Controller::new(
            RestoreGuard::new(&runner, None, None, None),
            PersistedState::default(),
            PathBuf::from("/nonexistent/state.json"),
            Config::default(),
            config_path.clone(),
        );

        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(Config::load(&config_path).fan_target_rpm, 2500.0);

        // Unchanged target (e.g. a key held at the clamp bound): no re-save.
        fs::remove_file(&config_path).unwrap();
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert!(
            !config_path.exists(),
            "an unchanged target must not spam config saves"
        );

        // A real change saves again, preserving the other fields.
        ctl.on_command(Command::SetFanTarget(2000.0));
        let saved = Config::load(&config_path);
        assert_eq!(saved.fan_target_rpm, 2000.0);
        assert_eq!(saved.cpu_floor_w, Config::default().cpu_floor_w);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn config_fast_limit_reaches_ryzenadj() {
        let runner = FakeRunner::new();
        let config = Config {
            fast_limit_mw: 60_000,
            ..Config::default()
        };
        let mut ctl: Controller<&FakeRunner> = Controller::new(
            RestoreGuard::new(
                &runner,
                Some(cpu_actuator(
                    &runner,
                    PathBuf::from("/nonexistent/platform_profile"),
                )),
                None,
                None,
            ),
            PersistedState::default(),
            PathBuf::from("/nonexistent/state.json"),
            config,
            PathBuf::from("/nonexistent/config.toml"),
        );
        ctl.on_command(Command::SetCpuW(20.0));
        assert_eq!(
            ryzenadj_calls(&runner),
            vec![vec![
                "--stapm-limit=20000".to_string(),
                "--slow-limit=20000".to_string(),
                "--fast-limit=60000".to_string(),
            ]]
        );
    }

    #[test]
    fn config_seeds_status_fan_target() {
        let runner = FakeRunner::new();
        let config = Config {
            fan_target_rpm: 2200.0,
            ..Config::default()
        };
        let ctl: Controller<&FakeRunner> = Controller::new(
            RestoreGuard::new(&runner, None, None, None),
            PersistedState::default(),
            PathBuf::from("/nonexistent/state.json"),
            config,
            PathBuf::from("/nonexistent/config.toml"),
        );
        assert_eq!(ctl.status().fan_target_rpm, 2200.0);
    }

    #[test]
    fn auto_allocate_decision_carries_demand_fields() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-auto-telemetry",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&busy_at(0.0));

        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));
        apply_effects(&effects, &ctl, 0.0, &ui_tx, &telemetry);
        // A non-auto decision afterwards: its line must skip the auto fields.
        apply_effects(
            &[Effect::StatusChanged { cause: "release" }],
            &ctl,
            1.0,
            &ui_tx,
            &telemetry,
        );
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let t = guard.as_mut().unwrap();
            t.flush();
            t.path().to_path_buf()
        };

        let contents = fs::read_to_string(&path).unwrap();
        let decisions: Vec<serde_json::Value> = contents
            .lines()
            .map(|l| serde_json::from_str(l).unwrap())
            .filter(|v: &serde_json::Value| v["kind"] == "decision")
            .collect();
        // Three lines: the allocate decision, the first-sample model
        // snapshot (its own record, Task 27) and the release.
        assert_eq!(decisions.len(), 3, "got: {contents}");

        let auto = &decisions[0];
        assert_eq!(auto["cause"], "auto:allocate");
        assert_eq!(auto["mode"], "auto");
        assert_eq!(auto["demand_cpu"], 1.0);
        assert_eq!(auto["demand_gpu"], 1.0);
        assert_eq!(auto["alloc_cpu_w"], 17.0);
        assert_eq!(auto["alloc_gpu_w"], 32.0);
        assert_eq!(auto["pi_target_w"], 32.0);
        assert_eq!(auto["cpu_limit_w"], 17.0);
        assert!(
            auto.get("model_a").is_none(),
            "model params belong to snapshot lines only: {auto}"
        );

        let snapshot = &decisions[1];
        assert_eq!(snapshot["cause"], "auto:model_snapshot");
        for key in ["demand_cpu", "demand_gpu", "alloc_cpu_w", "alloc_gpu_w"] {
            assert!(
                snapshot.get(key).is_none(),
                "{key} must be absent on snapshot decisions: {snapshot}"
            );
        }

        let plain = &decisions[2];
        assert_eq!(plain["cause"], "release");
        for key in ["demand_cpu", "demand_gpu", "alloc_cpu_w", "alloc_gpu_w"] {
            assert!(
                plain.get(key).is_none(),
                "{key} must be absent on non-auto decisions: {plain}"
            );
        }

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn out_of_range_config_floors_never_panic_auto() {
        // Reviewer-confirmed crash pre-fix: gpu_floor_mhz > 3090 made the
        // PI's f64::clamp (min > max) panic on the first Auto tick, and
        // cpu_floor_w > 54 tripped the allocator's floor debug_assert.
        // Controller::new sanitizes (as does Config::load for file configs).
        let runner = FakeRunner::new();
        let config = Config {
            cpu_floor_w: 99.0,
            gpu_floor_mhz: 4000,
            ..Config::default()
        };
        let (mut ctl, _gpu_calls) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            config,
        );
        ctl.on_command(Command::SetAuto(true));
        for t in [0.0, 1.0, 2.0, 5.0] {
            ctl.on_sample(&busy_at(t)); // panicked here before the fix
        }
        // Floors landed clamped to the hardware envelope.
        assert_eq!(ctl.status().cpu_limit_w, Some(54.0));
        assert_eq!(ctl.status().gpu_max_mhz, Some(3090));
    }

    #[test]
    fn alloc_change_resets_stickiness_streak() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0)); // limit 17 W
        let hot_at = |t: f64| Sample {
            cpu_pkg_w: 30.0, // violates any limit here (margin 5 W)
            ..busy_at(t)
        };

        // Two violations against the 17 W allocation: one short of firing.
        ctl.on_sample(&hot_at(1.0));
        ctl.on_sample(&hot_at(2.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));

        // t=5: the allocator retargets to 19 W. The stale 2-strike streak
        // was measured against the OLD limit — the fresh allocation must
        // get a full 3-sample streak, so this sample must NOT fire.
        let effects = ctl.on_sample(&hot_at(5.0));
        assert_eq!(ctl.status().cpu_limit_w, Some(19.0), "premise: retargeted");
        assert!(
            !ctl.status().flags.contains(&StatusFlag::LimitNotSticking),
            "stale streak fired one sample into a fresh allocation"
        );
        assert!(!has_reassert(&effects, "stickiness"), "got {effects:?}");

        // The watchdog still works: three fresh violations fire as usual.
        ctl.on_sample(&hot_at(6.0));
        let effects = ctl.on_sample(&hot_at(7.0));
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
    }

    #[test]
    fn pi_rate_reference_tracks_hardware_on_failed_gpu_set() {
        let runner = FakeRunner::new();
        let gpu = FakeGpu::new();
        let gpu_calls = gpu.calls();
        let gpu_failures = gpu.failures();
        let mut ctl = Controller::new(
            RestoreGuard::new(
                &runner,
                Some(cpu_actuator(
                    &runner,
                    PathBuf::from("/nonexistent/platform_profile"),
                )),
                Some(Box::new(gpu)),
                None,
            ),
            calibrated(),
            PathBuf::from("/nonexistent/state.json"),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        ctl.on_command(Command::SetGpuMaxClock(1500));
        ctl.on_command(Command::SetAuto(true)); // PI seeded from applied 1500

        // The first PI command (1605, rate-limited from 1500) FAILS: the
        // hardware still holds 1500, so status must not move and the failed
        // attempt must not become the rate reference.
        *gpu_failures.lock().unwrap() = 1;
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.status().gpu_max_mhz, Some(1500));
        assert_eq!(gpu_sets(&gpu_calls), vec![1500], "only the manual lock");

        // Next tick: the retry must rate-limit from the APPLIED 1500 (→ 1605
        // again), not from the failed 1605 intent (which would allow 1653).
        ctl.on_sample(&busy_at(1.0));
        assert_eq!(gpu_sets(&gpu_calls), vec![1500, 1605]);
        assert_eq!(ctl.status().gpu_max_mhz, Some(1605));
    }

    // --- Task 26: trim integrator ---

    use crate::control::trim::MAX_TRIM_AUTHORITY_RPM;

    /// `busy_at` with a chosen fan reading (the trim window watches the fan).
    fn busy_fan_at(t: f64, fan_rpm: f64) -> Sample {
        Sample {
            fan1_rpm: fan_rpm,
            ..busy_at(t)
        }
    }

    fn has_status_cause(effects: &[Effect], want: &str) -> bool {
        effects
            .iter()
            .any(|e| matches!(e, Effect::StatusChanged { cause } if *cause == want))
    }

    #[test]
    fn steady_auto_samples_move_trim_toward_model_error() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        // 19 steady samples (t=0..=18): window not yet 20 long — no trim.
        for t in 0..19 {
            ctl.on_sample(&busy_at(f64::from(t)));
            assert_eq!(ctl.status().trim_rpm, 0.0, "trim moved early at t={t}");
        }

        // 20th steady sample: first trim update. Allocation is (23, 38)
        // (four +2 steps from the conservative start), so the model
        // predicts 25*23 + 15*38 + 0.1*23*38 + 800 = 2032.4 RPM; measured
        // 1700 → error -332.4 → trim = -16.62 (model OVER-predicts here,
        // so the offset goes negative = more budget).
        let effects = ctl.on_sample(&busy_at(19.0));
        assert!(has_status_cause(&effects, "auto:trim"), "got {effects:?}");
        let trim = ctl.status().trim_rpm;
        assert!((trim - (-16.62)).abs() < 0.01, "trim = {trim}");
    }

    #[test]
    fn non_steady_window_freezes_trim() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Fan bouncing 1500/1700: spread 200 > the 100 RPM tolerance, so
        // the window is never steady — the trim must never integrate on a
        // transient, no matter how long it lasts.
        for t in 0..=45 {
            let rpm = if t % 2 == 0 { 1500.0 } else { 1700.0 };
            ctl.on_sample(&busy_fan_at(f64::from(t), rpm));
        }
        assert_eq!(ctl.status().trim_rpm, 0.0);
    }

    #[test]
    fn fan_invalid_freezes_trim_and_restarts_the_settling_clock() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        // 18 valid steady samples, then an 8 s fan outage, then valid again:
        // the outage lands as NaN in the window, so the 20-sample steady
        // tail restarts — no update may span the gap (a lost sensor must
        // freeze the trim exactly like it freezes the allocator).
        for t in 0..18 {
            ctl.on_sample(&busy_at(f64::from(t)));
        }
        for t in 18..26 {
            let s = Sample {
                fan_valid: false,
                ..busy_at(f64::from(t))
            };
            ctl.on_sample(&s);
            assert_eq!(ctl.status().trim_rpm, 0.0, "trim moved on invalid fan");
        }
        for t in 26..45 {
            ctl.on_sample(&busy_at(f64::from(t)));
            assert_eq!(ctl.status().trim_rpm, 0.0, "tail spans the outage at t={t}");
        }
        // 20 clean samples after the outage (t=26..=45): integrates again.
        ctl.on_sample(&busy_at(45.0));
        assert_ne!(ctl.status().trim_rpm, 0.0);
    }

    /// Calibrated controller pinned at a CONSTANT operating point: a 54 W
    /// CPU floor plus a 1000 RPM target park the allocator at (54 W, 0 W)
    /// within one step (the contour is negative-clamped at 54 W for any
    /// plausible fan reading), so the RLS excitation gate stays closed and
    /// trim/trust behavior is isolated from model adaptation.
    fn pinned_op_controller(
        runner: &FakeRunner,
    ) -> (Controller<&FakeRunner>, Arc<Mutex<Vec<GpuCall>>>) {
        auto_controller(
            runner,
            PathBuf::from("/nonexistent/platform_profile"),
            Config {
                cpu_floor_w: 54.0,
                fan_target_rpm: 1000.0,
                ..Config::default()
            },
        )
    }

    /// The calibrated model's prediction at the pinned (54, 0) point:
    /// 25·54 + 800. Feeding this as the fan reading makes the (one)
    /// first-steady-sample RLS acceptance carry zero error, so the model
    /// stays EXACTLY calibrated for the rest of the scenario.
    const PINNED_PREDICT_RPM: f64 = 2150.0;

    fn rls_accepts(effects: &[Effect]) -> usize {
        effects
            .iter()
            .filter(|e| matches!(e, Effect::RlsAccepted))
            .count()
    }

    #[test]
    fn persistent_residual_at_frozen_point_saturates_trim_and_flags_unreachable() {
        // This is ALSO the Task-27 separation-of-concerns sanity test: at a
        // constant operating point the excitation gate freezes RLS, so a
        // measured drift is absorbed by the trim ONLY (trim = fast offset,
        // RLS = shape-under-excitation; they never fight over one error).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Clean baseline: fan matches the model exactly, so the single
        // first-steady-sample RLS acceptance (t=19) changes nothing.
        let mut accepts = 0;
        for t in 0..40 {
            accepts += rls_accepts(&ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM)));
        }
        assert_eq!(accepts, 1, "exactly the documented first-sample accept");
        // Not bit-zero: the SVD fit reproduces 2150 to ~1e-9, and the trim
        // dutifully integrates that dust.
        assert!(ctl.status().trim_rpm.abs() < 1e-9);

        // Blocked intake: +295 RPM of measured drift the frozen model can't
        // explain. 295 < the 300 RPM distrust threshold — this test isolates
        // the TargetUnreachable path (ModelDistrust must stay clear). The
        // trim walks up at 14.75 RPM/update and pins at +400 by t=599.
        for t in 40..=610 {
            accepts += rls_accepts(&ctl.on_sample(&busy_fan_at(f64::from(t), 2445.0)));
        }
        assert_eq!(ctl.status().trim_rpm, MAX_TRIM_AUTHORITY_RPM);
        assert!(
            ctl.status().flags.contains(&StatusFlag::TargetUnreachable),
            "saturated +max must surface TargetUnreachable"
        );
        assert!(
            !ctl.status().flags.contains(&StatusFlag::ModelDistrust),
            "sub-threshold residual must not distrust the model"
        );
        // ONLY the trim moved: no further RLS acceptance, and the model is
        // still bit-exactly the calibrated one.
        assert_eq!(accepts, 1, "excitation gate must freeze RLS at one point");
        let m = ctl.model.as_ref().unwrap();
        let calib = fitted_model();
        for (got, want) in [
            (m.a, calib.a),
            (m.b, calib.b),
            (m.e, calib.e),
            (m.c, calib.c),
        ] {
            // ~1e-9 slack: the t=19 acceptance integrated the SVD fit's dust.
            assert!((got - want).abs() < 1e-6, "param moved: {got} vs {want}");
        }

        // Recovery (blanket removed): fans drop under the prediction, the
        // error flips sign and the trim walks off the clamp; the flag clears
        // below 90% of max (360).
        for t in 611..=800 {
            ctl.on_sample(&busy_fan_at(f64::from(t), 2050.0));
        }
        assert!(
            ctl.status().trim_rpm < TRIM_CLEAR_FRACTION * MAX_TRIM_AUTHORITY_RPM,
            "trim = {}",
            ctl.status().trim_rpm
        );
        assert!(
            !ctl.status().flags.contains(&StatusFlag::TargetUnreachable),
            "flag must clear below 90% of max"
        );
    }

    #[test]
    fn auto_exit_resets_trim() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=19 {
            ctl.on_sample(&busy_at(f64::from(t)));
        }
        assert_ne!(ctl.status().trim_rpm, 0.0, "premise: trim accumulated");

        // SetAuto(false) resets (AutoState drops whole; status mirrors it).
        ctl.on_command(Command::SetAuto(false));
        assert_eq!(ctl.status().trim_rpm, 0.0);

        // Re-entry starts fresh and needs a fresh 20-sample steady window.
        ctl.on_command(Command::SetAuto(true));
        for t in 100..119 {
            ctl.on_sample(&busy_at(f64::from(t)));
            assert_eq!(ctl.status().trim_rpm, 0.0, "stale trim after re-entry");
        }
        ctl.on_sample(&busy_at(119.0));
        assert_ne!(ctl.status().trim_rpm, 0.0);

        // ReleaseAll is the other Auto exit: resets too.
        ctl.on_command(Command::ReleaseAll);
        assert_eq!(ctl.status().trim_rpm, 0.0);
        assert_eq!(ctl.status().mode, Mode::Monitor);
    }

    #[test]
    fn fan_target_change_keeps_trim() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=19 {
            ctl.on_sample(&busy_at(f64::from(t)));
        }
        let trim = ctl.status().trim_rpm;
        assert_ne!(trim, 0.0, "premise: trim accumulated");

        // Retargeting the fan goal does NOT reset the trim: the ambient
        // (what the trim measures) didn't change with the user's target.
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(ctl.status().trim_rpm, trim);
        assert_eq!(ctl.status().mode, Mode::Auto);
    }

    #[test]
    fn trim_decisions_reach_telemetry_with_offset() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-trim-telemetry",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));
        for t in 0..19 {
            ctl.on_sample(&busy_at(f64::from(t)));
        }
        // t=19: the trim update; t=20: an allocator step using the trim.
        let effects = ctl.on_sample(&busy_at(19.0));
        apply_effects(&effects, &ctl, 19.0, &ui_tx, &telemetry);
        let effects = ctl.on_sample(&busy_at(20.0));
        apply_effects(&effects, &ctl, 20.0, &ui_tx, &telemetry);
        // Auto exit: a non-Auto decision afterwards must skip trim_rpm.
        let effects = ctl.on_command(Command::SetAuto(false));
        apply_effects(&effects, &ctl, 21.0, &ui_tx, &telemetry);
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let t = guard.as_mut().unwrap();
            t.flush();
            t.path().to_path_buf()
        };

        let contents = fs::read_to_string(&path).unwrap();
        let decisions: Vec<serde_json::Value> = contents
            .lines()
            .map(|l| serde_json::from_str(l).unwrap())
            .filter(|v: &serde_json::Value| v["kind"] == "decision")
            .collect();

        let trim_line = decisions
            .iter()
            .find(|d| d["cause"] == "auto:trim")
            .unwrap_or_else(|| panic!("no auto:trim decision in {contents}"));
        let offset = trim_line["trim_rpm"].as_f64().expect("trim_rpm present");
        assert!((offset - (-16.62)).abs() < 0.01, "trim_rpm = {offset}");

        // The allocate line right after carries the current trim too.
        let alloc_line = decisions
            .iter()
            .find(|d| d["cause"] == "auto:allocate" && d["t_mono"] == 20.0)
            .unwrap_or_else(|| panic!("no t=20 allocate decision in {contents}"));
        assert_eq!(alloc_line["trim_rpm"], trim_line["trim_rpm"]);

        // Non-Auto decisions stay lean: no trim_rpm key.
        let off_line = decisions
            .iter()
            .find(|d| d["cause"] == "auto:off")
            .unwrap_or_else(|| panic!("no auto:off decision in {contents}"));
        assert!(
            off_line.get("trim_rpm").is_none(),
            "trim_rpm must be skipped outside Auto: {off_line}"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn positive_trim_cuts_the_commanded_gpu_allocation_end_to_end() {
        // The decisive sign check for the trim→contour wiring: a POSITIVE
        // trim must yield FEWER commanded GPU watts than the untrimmed
        // contour would (under-prediction → smaller budget). Kills both
        // sign mutants in the allocator's contour closure: `trim_rpm → 0.0`
        // (allocation lands on the untrimmed contour) and `trim_rpm →
        // -trim_rpm` (positive feedback: MORE budget, above the untrimmed
        // contour).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Build exactly +100 RPM of trim at the frozen (54, 0) point, with
        // the model staying bit-for-bit calibrated (RLS excitation-frozen,
        // as pinned by persistent_residual_…): clean baseline, then a +400
        // step → five 20 s trim updates of +20 (t = 59..139), well before
        // the t=427 distrust onset.
        for t in 0..40 {
            ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM));
        }
        for t in 40..=139 {
            ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM + 400.0));
        }
        let trim = ctl.status().trim_rpm;
        assert!((trim - 100.0).abs() < 1e-6, "premise: trim = {trim}");

        // Now raise the target to 4000 RPM so the contour at pc=54 is
        // INTERIOR (neither zero-clamped nor at GPU_MAX), and feed a fan
        // sawtooth (2400/2650, spread 250 > the 100 RPM steadiness
        // tolerance, always valid): the whole adaptation tier freezes (no
        // steady window → no RLS, no trim movement, no trust evidence)
        // while the allocator keeps walking the GPU allocation up toward
        // the TRIMMED contour — isolating exactly the trim term.
        ctl.on_command(Command::SetFanTarget(4000.0));
        let mut last_gpu = None;
        for t in 140..=400 {
            let fan = if t % 2 == 0 { 2400.0 } else { 2650.0 };
            if let Some((_, gpu_w)) = alloc_of(&ctl.on_sample(&busy_fan_at(f64::from(t), fan))) {
                last_gpu = Some(gpu_w);
            }
        }
        let gpu_w = last_gpu.expect("allocator ran");
        assert_eq!(ctl.status().trim_rpm, trim, "trim frozen while unsteady");

        // The model is still calibrated, so both contours are exact:
        // untrimmed (4000 − 800 − 25·54)/20.4 ≈ 90.7 W, trimmed ≈ 85.8 W.
        let m = ctl.model.as_ref().unwrap();
        let untrimmed = m.gpu_watts_on_contour(4000.0, 0.0, 54.0).unwrap();
        let trimmed = m.gpu_watts_on_contour(4000.0, trim, 54.0).unwrap();
        assert!((untrimmed - 90.7).abs() < 0.1, "untrimmed = {untrimmed}");
        assert!(
            (gpu_w - trimmed).abs() < 2.5,
            "allocation must settle on the TRIMMED contour: {gpu_w} vs {trimmed}"
        );
        assert!(
            gpu_w < untrimmed - 2.0,
            "positive trim must CUT the allocation below the untrimmed \
             contour: {gpu_w} vs {untrimmed}"
        );
    }

    // --- Task 27: online RLS + trust monitor ---

    #[test]
    fn steady_drifted_samples_accept_rls_and_move_the_model() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-rls-telemetry",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let calib = fitted_model();

        // busy_at: fan steady at 1700 while the model expects ~2030 at the
        // walking allocation — a real drift. The first steady sample (t=19)
        // must feed RLS (operating point fresh → excitation gate open).
        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));
        let mut first_accept = None;
        for t in 0..=30 {
            let effects = ctl.on_sample(&busy_at(f64::from(t)));
            if rls_accepts(&effects) > 0 && first_accept.is_none() {
                first_accept = Some(t);
            }
            apply_effects(&effects, &ctl, f64::from(t), &ui_tx, &telemetry);
        }
        assert_eq!(first_accept, Some(19), "first steady sample feeds RLS");

        // The model moved TOWARD the measured 1700 at the operating point.
        let cpu_w = ctl.status().cpu_limit_w.unwrap();
        let gpu_w = ctl.auto.as_ref().unwrap().gpu_target_w.unwrap();
        let m = ctl.model.as_ref().unwrap();
        let (adapted, calibrated) = (m.predict(cpu_w, gpu_w), calib.predict(cpu_w, gpu_w));
        assert!(
            (adapted - 1700.0).abs() < (calibrated - 1700.0).abs(),
            "prediction must move toward the drift: {adapted} vs {calibrated}"
        );

        // Each acceptance is its own telemetry Decision, cause "auto:rls".
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let t = guard.as_mut().unwrap();
            t.flush();
            t.path().to_path_buf()
        };
        let contents = fs::read_to_string(&path).unwrap();
        let rls_line = contents
            .lines()
            .map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap())
            .find(|v| v["kind"] == "decision" && v["cause"] == "auto:rls")
            .unwrap_or_else(|| panic!("no auto:rls decision in {contents}"));
        assert_eq!(rls_line["mode"], "auto");
        assert!(rls_line["trim_rpm"].is_number(), "trim context carried");

        fs::remove_dir_all(&dir).unwrap();
    }

    /// Drive a pinned-op controller into ModelDistrust: clean baseline, then
    /// a +400 RPM measured step the (excitation-frozen) model cannot
    /// explain. Asserts the transition telemetry cause and returns the
    /// t_mono of the first flagged sample — deterministically t=427: the
    /// EWMA crosses 300 after 69 feeds of 400 (starting t=59, once the
    /// stepped window turns steady) and the 300 s sustain follows.
    fn drive_to_distrust(ctl: &mut Controller<&FakeRunner>) -> u32 {
        ctl.on_command(Command::SetAuto(true));
        for t in 0..40 {
            ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM));
        }
        for t in 40..=500 {
            let effects = ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM + 400.0));
            if ctl.status().flags.contains(&StatusFlag::ModelDistrust) {
                assert!(
                    has_status_cause(&effects, "auto:distrust"),
                    "flag transition must claim its cause, got {effects:?}"
                );
                return t;
            }
        }
        panic!("ModelDistrust never tripped");
    }

    #[test]
    fn distrust_flags_freezes_rls_halves_trim_and_recovers() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        let flagged_at = drive_to_distrust(&mut ctl);
        // NOT at the first EWMA crossing (t=127): only after the 300 s
        // sustain. Trim so far ran at FULL gain: 19 updates × 20 RPM.
        assert_eq!(flagged_at, 427);
        let trim_at_flag = ctl.status().trim_rpm;
        assert!((trim_at_flag - 380.0).abs() < 1e-6, "trim = {trim_at_flag}");

        // HALF gain from here: the next 20 s trim update (t=439) moves by
        // 0.5·KI·400 = +10 RPM — a trusted update would apply +20.
        for t in flagged_at + 1..=flagged_at + 20 {
            ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM + 400.0));
        }
        assert!(
            (ctl.status().trim_rpm - (trim_at_flag + 10.0)).abs() < 1e-6,
            "distrusted trim must integrate at half gain, got {}",
            ctl.status().trim_rpm
        );

        // RLS frozen BY DISTRUST, not merely by excitation: retargeting to
        // 7000 RPM walks the GPU allocation up (+2 W / 5 s), the operating
        // point moves past the excitation gate — and still nothing may be
        // learned from readings we distrust.
        ctl.on_command(Command::SetFanTarget(7000.0));
        let m = ctl.model.as_ref().unwrap();
        let params = (m.a, m.b, m.e, m.c);
        let mut t = flagged_at + 21;
        for _ in 0..25 {
            let effects = ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM + 400.0));
            assert_eq!(rls_accepts(&effects), 0, "RLS must stay frozen at t={t}");
            t += 1;
        }
        let m = ctl.model.as_ref().unwrap();
        assert_eq!((m.a, m.b, m.e, m.c), params, "params moved while frozen");
        assert!(ctl.status().flags.contains(&StatusFlag::ModelDistrust));

        // Recovery: collapse back to the frozen (54, 0) point and feed the
        // fan the model expects — the EWMA decays under 300 and the verdict
        // returns Ok: flag clears (with its cause), trim gain restores.
        ctl.on_command(Command::SetFanTarget(1000.0));
        let mut cleared_at = None;
        for _ in 0..80 {
            let effects = ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM));
            if !ctl.status().flags.contains(&StatusFlag::ModelDistrust) {
                // The clear is a status change; its "auto:distrust_cleared"
                // cause may be shadowed if the sample also allocates (first
                // claim wins — the per-sample single-Decision design).
                assert_eq!(status_changes(&effects), 1, "got {effects:?}");
                cleared_at = Some(t);
                break;
            }
            t += 1;
        }
        let cleared_at = cleared_at.expect("distrust never cleared");
        assert!(!ctl.auto.as_ref().unwrap().distrusted, "gain restored");

        // And RLS unfreezes: excite the operating point again → an update
        // is accepted once the point has moved past the gate.
        ctl.on_command(Command::SetFanTarget(7000.0));
        let mut accepted = false;
        for t in cleared_at + 1..cleared_at + 30 {
            let effects = ctl.on_sample(&busy_fan_at(f64::from(t), PINNED_PREDICT_RPM));
            if rls_accepts(&effects) > 0 {
                accepted = true;
                break;
            }
        }
        assert!(accepted, "RLS must resume after trust recovery");
    }

    #[test]
    fn auto_exit_resets_trust_and_distrust_flag() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        drive_to_distrust(&mut ctl);
        assert!(ctl.status().flags.contains(&StatusFlag::ModelDistrust));

        // Auto exit: the flag leaves the status with the mode…
        ctl.on_command(Command::SetAuto(false));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(!ctl.status().flags.contains(&StatusFlag::ModelDistrust));

        // …and re-entry starts with FRESH trust (AutoState dropped whole).
        ctl.on_command(Command::SetAuto(true));
        assert!(!ctl.status().flags.contains(&StatusFlag::ModelDistrust));
        let auto = ctl.auto.as_ref().unwrap();
        assert!(!auto.distrusted);
        assert_eq!(auto.trust.ewma(), 0.0);
    }

    #[test]
    fn model_snapshot_decision_every_60s_in_auto() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-snapshot-telemetry",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));
        for t in 0..=125 {
            let effects = ctl.on_sample(&busy_at(f64::from(t)));
            apply_effects(&effects, &ctl, f64::from(t), &ui_tx, &telemetry);
        }
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let t = guard.as_mut().unwrap();
            t.flush();
            t.path().to_path_buf()
        };

        let contents = fs::read_to_string(&path).unwrap();
        let snapshots: Vec<serde_json::Value> = contents
            .lines()
            .map(|l| serde_json::from_str(l).unwrap())
            .filter(|v: &serde_json::Value| v["cause"] == "auto:model_snapshot")
            .collect();
        let times: Vec<f64> = snapshots
            .iter()
            .map(|s| s["t_mono"].as_f64().unwrap())
            .collect();
        assert_eq!(times, vec![0.0, 60.0, 120.0], "in {contents}");
        // The entry snapshot is the calibrated baseline the later lines are
        // read against; by t=60 online RLS has adapted the surface.
        let a0 = snapshots[0]["model_a"].as_f64().unwrap();
        assert!((a0 - 25.0).abs() < 1e-6, "baseline a = {a0}");
        for key in ["model_a", "model_b", "model_e", "model_c"] {
            assert!(snapshots[1][key].is_number(), "{key} missing");
        }
        let c60 = snapshots[1]["model_c"].as_f64().unwrap();
        assert!((c60 - 800.0).abs() > 1.0, "c must have adapted, got {c60}");

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn online_rls_never_touches_the_persisted_state() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-rls-persist",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let state_path = dir.join("state.json");
        calibrated().save(&state_path).unwrap();

        let gpu = FakeGpu::new();
        let mut ctl = Controller::new(
            RestoreGuard::new(
                &runner,
                Some(cpu_actuator(
                    &runner,
                    PathBuf::from("/nonexistent/platform_profile"),
                )),
                Some(Box::new(gpu)),
                None,
            ),
            PersistedState::load(&state_path),
            state_path.clone(),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=40 {
            ctl.on_sample(&busy_at(f64::from(t)));
        }

        // In memory the model adapted (session-only)…
        let calib = fitted_model();
        let m = ctl.model.as_ref().unwrap();
        assert!(
            (m.c - calib.c).abs() > 1.0,
            "premise: online RLS adapted the model, c = {}",
            m.c
        );
        // …but the state FILE still carries the CALIBRATED parameters: a
        // restart reverts to calibrated + fresh adaptation (only a finished
        // calibration writes the state file).
        let saved = PersistedState::load(&state_path).model.expect("model");
        assert_eq!(
            (saved.a, saved.b, saved.e, saved.c),
            (calib.a, calib.b, calib.e, calib.c)
        );

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
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        assert_eq!(ctl.lut, Some(lut));
        assert!(ctl.model.is_none());
    }
}

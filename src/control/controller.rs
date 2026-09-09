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

use crate::actuators::WriteVerdict;
use crate::actuators::cmd::Runner;
use crate::actuators::guard::RestoreGuard;
use crate::calib::burner::Burner;
use crate::calib::runner::{CalibRunner, RunnerEffect};
use crate::calib::steady::{STEADY_N, STEADY_RPM_TOLERANCE, is_steady, tail_mean};
use crate::config::Config;
use crate::control::allocator::{self, AllocInput, Allocator};
use crate::control::cooldown::{self, CommandedPoint};
use crate::control::gpu_pid::GpuPid;
use crate::control::kalman::{Kalman, MAX_BIAS_AUTHORITY_RPM};
use crate::control::lut::ClockWattsLut;
use crate::control::thermal_model::ThermalModel;
use crate::control::trust::{Trust, TrustMonitor};
use crate::control::watchdog::{ThermalWatchdog, Trip};
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
/// Stricter streak while the post-resume window is open: limits are most
/// likely to silently revert right after a resume (firmware reasserts its
/// own defaults late, PPD/tuned re-apply profiles on wakeup), so the
/// watchdog fires one sample earlier while the evidence is hottest.
const STICKINESS_SAMPLES_STRICT: u8 = 2;
/// Elevated-stickiness window after a resume (see above).
const RESUMED_STRICT_S: f64 = 60.0;
/// How long the `Resumed` flag stays visible after a suspend/resume.
const RESUMED_FLAG_S: f64 = 30.0;
/// Cap on the Auto-mode fan-RPM window feeding the adaptation tier's
/// steadiness gate (`is_steady` needs STEADY_N=20; a little slack beyond
/// that is harmless). The observed-watts windows share it: they pair with
/// the fan window's tail into one KF update, so their horizons must match.
const FAN_WINDOW_CAP: usize = 30;
/// Cap on the cooldown ring: 40 s of 1 Hz history — deliberately > the
/// gate's 30 s window so the coverage condition (a point ≥ WINDOW_S old)
/// stays satisfiable after pruning (see `cooldown::cooldown_open`).
const COMMANDED_RING_CAP: usize = 40;
/// Span (seconds ≙ 1 Hz samples) of the fan-slope estimate fed to the
/// allocator's velocity gate: long enough to average sample-to-sample RPM
/// jitter, short enough to see the mid-cycle 30–50 RPM/s transients the
/// gate exists to catch (`allocator::SLOPE_GATE_RPM_S`).
const FAN_SLOPE_SPAN_S: usize = 10;
/// Samples averaged into the smoothed fan RPM the allocator's band checks
/// (deadband / raise gate / overshoot) see. A 5-sample tail mean halves the
/// ~92 RPM soak-noise stdev (≈92 → ≈45) on the value those edges test, so a
/// single tach blip from a near-edge equilibrium can no longer fire the
/// mandatory overshoot drain — the exact mechanism of the field relay
/// (run-1783720682, 2026-07-10). Detection of a genuine overshoot is delayed
/// only ~2–3 s (30–50 RPM/s transients still cross within the span), an
/// accepted trade. `pub(crate)` so the allocator's soak-cycle sim reads the
/// same const it is wired from.
pub(crate) const FAN_SMOOTH_N: usize = 5;
/// `TargetUnreachable` clears once the KF bias drops below this fraction
/// of its +max — hysteresis so the flag doesn't flicker at the bound.
const TRIM_CLEAR_FRACTION: f64 = 0.9;
/// Auto-mode cadence of the "auto:model_snapshot" telemetry Decision
/// carrying the live a/b/e/c: one line a minute keeps the calibrated
/// baseline reviewable offline (the KF corrects OUTSIDE the surface, so
/// these parameters only ever change when a calibration lands).
const MODEL_SNAPSHOT_PERIOD_S: f64 = 60.0;
/// Achievement-gate margin on the CPU leg (W): the measured package draw
/// must be within this of (or above) the commanded sustained limit for the
/// adaptation tier to treat the operating point as actually TESTED. 3 W is
/// the normal sag of a busy-but-not-pinned load under its cap; anything
/// deeper means the load, not the limit, chose the power.
const ACHIEVED_CPU_MARGIN_W: f64 = 3.0;
/// Achievement-gate margin on the GPU leg (W), against the PI's watts
/// target. Wider than the CPU's: GPU draw telemetry is noisier and the PI
/// dithers the clock around the target by a few watts.
const ACHIEVED_GPU_MARGIN_W: f64 = 5.0;
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
    /// Safety floors (CPU sustained watts, GPU max-clock MHz), carrying BOTH
    /// current values (the UI model steps them locally). Sanitized, echoed
    /// in status, persisted to config on change; the Auto allocator/PI
    /// consume them on their next step. Rejected only while Calibrating —
    /// floors are safety config, not actuation, so they are allowed in every
    /// other mode and (like SetFanTarget) pass the emergency acknowledge
    /// gate without consuming the acknowledge.
    SetFloors { cpu_w: f64, gpu_mhz: u32 },
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

/// Which loop (if any) currently owns actuation in Auto mode (design §2.5,
/// the arbiter — `fwloop`, not yet wired here: this task only defines the
/// type). `TempLoop` closes on the fw-fanctrl replica's temperature error
/// (Mode A); `RpmLoop` falls back to the fan-RPM error when TempLoop's
/// preconditions aren't met (Mode B); `Released` means neither loop has
/// anything to close on and caps sit at stock.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LoopMode {
    // Not yet produced outside tests — the arbiter that decides between
    // them (design §2.5) is a later task; only `Released` (the default) is
    // reachable through today's call sites.
    #[allow(dead_code)]
    TempLoop,
    #[allow(dead_code)]
    RpmLoop,
    #[default]
    Released,
}

/// Coarse severity tier for a [`StatusFlag`], ordered loudest-first so a
/// derived `Ord`/`PartialOrd` sorts a flag list severity-first (declaration
/// order IS the ranking: `Critical < Warning < Info`).
// Classifies StatusFlag (used by its own tests); nothing outside tests
// calls it yet — Task 15 wires it into the header.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Critical,
    Warning,
    Info,
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
    /// The Kalman BIAS is pinned at its +max authority: the fans stay
    /// persistently over target even at the maximum budget cut (a model
    /// bias beyond the KF's authority, or floors holding power above the
    /// contour) — check intake/ambient (research 03 §6: surface a status
    /// when the floor is hit instead of silently collapsing performance).
    /// Clears once the bias drops below [`TRIM_CLEAR_FRACTION`] of max.
    TargetUnreachable,
    /// The thermal watchdog tripped (3 consecutive samples at/over 95 °C
    /// Tctl or 87 °C GPU) and everything was released toward stock. REQUIRES
    /// MANUAL RE-ARM: never clears on its own — the first actuating command
    /// (`a`, `c`/`g`, `k`) only acknowledges (clears the flag + re-arms the
    /// watchdog) without executing; the second press acts normally.
    ThermalEmergency,
    /// The watchdog's sensor-lost trip (10 consecutive samples without a
    /// valid CPU temperature while limits were applied): assume hot, same
    /// release + manual-re-arm semantics as [`StatusFlag::ThermalEmergency`]
    /// (design amendment, Task 7: a lost sensor must never let the watchdog
    /// go blind).
    SensorLost,
    // The seven flags below are the Task 4 type surface's new additions:
    // none is raised by any call site yet (the guards/arbiter that would
    // raise them are later tasks), so each needs an explicit dead_code
    // allow; their own coverage/severity tests still construct them.
    /// The fw-fanctrl socket is absent or stale (design §2.5): TempLoop is
    /// unavailable and the loop falls to RpmLoop. Informational while in
    /// RpmLoop; clears when the socket returns.
    #[allow(dead_code)]
    FanctrlLost,
    /// The controller's EC replica disagrees with fw-fanctrl's own `print
    /// all` view for 3 consecutive scored views (design §2.6): TempLoop is
    /// unavailable until 3 consecutive views agree again.
    #[allow(dead_code)]
    EcMismatch,
    /// `slope_at(T*) > 2 %/°C` (design §2.7): the loop runs, but the
    /// operating point sits on a steep segment of the fw-fanctrl curve.
    /// Informational only.
    #[allow(dead_code)]
    SteepCurve,
    /// A permanent loss of Mode A (unlike the transient [`Self::SteepCurve`],
    /// this does not clear on its own) — a warning, not merely informational,
    /// since it is a standing loss of the primary control loop rather than a
    /// momentary steepness note.
    #[allow(dead_code)]
    CurveInvalid,
    /// dGPU at/over its hot threshold (design §2.8, default 90 °C, exit
    /// 85 °C): the GPU share is overridden down at each allocator tick.
    #[allow(dead_code)]
    GpuHot,
    /// The NVMe `Composite` sensor is at/over its hot threshold (design
    /// §2.8, default 80 °C). Reporting only — no control action.
    #[allow(dead_code)]
    NvmeHot,
    /// Six consecutive `Unreadable`/`Unverifiable` actuator read-backs
    /// (design §2.9): informational, cleared by the next `Verified`.
    #[allow(dead_code)]
    ReadbackBlind,
}

impl StatusFlag {
    /// Telemetry string form.
    pub fn as_str(self) -> &'static str {
        match self {
            StatusFlag::LimitNotSticking => "limit_not_sticking",
            StatusFlag::Resumed => "resumed",
            StatusFlag::NotCalibrated => "not_calibrated",
            StatusFlag::TargetUnreachable => "target_unreachable",
            StatusFlag::ThermalEmergency => "thermal_emergency",
            StatusFlag::SensorLost => "sensor_lost",
            StatusFlag::FanctrlLost => "fanctrl_lost",
            StatusFlag::EcMismatch => "ec_mismatch",
            StatusFlag::SteepCurve => "steep_curve",
            StatusFlag::CurveInvalid => "curve_invalid",
            StatusFlag::GpuHot => "gpu_hot",
            StatusFlag::NvmeHot => "nvme_hot",
            StatusFlag::ReadbackBlind => "readback_blind",
        }
    }
}

/// Severity tier for every [`StatusFlag`] variant (exhaustive match: adding
/// a variant without extending this fails the build rather than silently
/// defaulting). UI ordering/styling is Task 15's job; this only classifies.
/// Called only from its own tests today — no production call site wires
/// flag classification into rendering yet.
#[allow(dead_code)]
pub fn flag_severity(flag: StatusFlag) -> Severity {
    match flag {
        StatusFlag::ThermalEmergency => Severity::Critical,
        StatusFlag::SensorLost => Severity::Critical,
        StatusFlag::TargetUnreachable => Severity::Critical,
        StatusFlag::CurveInvalid => Severity::Warning,
        StatusFlag::EcMismatch => Severity::Warning,
        StatusFlag::FanctrlLost => Severity::Warning,
        StatusFlag::GpuHot => Severity::Warning,
        StatusFlag::LimitNotSticking => Severity::Warning,
        StatusFlag::NotCalibrated => Severity::Warning,
        StatusFlag::ReadbackBlind => Severity::Info,
        StatusFlag::SteepCurve => Severity::Info,
        StatusFlag::NvmeHot => Severity::Info,
        StatusFlag::Resumed => Severity::Info,
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
    /// CPU sustained-watts floor (config, live-editable via SetFloors).
    pub cpu_floor_w: f64,
    /// GPU max-clock floor in MHz (config, live-editable via SetFloors).
    pub gpu_floor_mhz: u32,
    /// CPU/GPU operating maxes (watts) from config: carried on the status so
    /// the UI (percent-of-max chart, manual-step ceiling) reads the same
    /// source of truth the controller allocates against.
    pub cpu_max_w: f64,
    pub gpu_max_w: f64,
    /// Currently active flags.
    pub flags: Vec<StatusFlag>,
    /// Calibration wizard progress; Some exactly while Calibrating.
    pub calib: Option<CalibProgressLite>,
    /// Which loop the arbiter (design §2.5, `fwloop` — not yet wired here)
    /// currently owns; `Released` outside Auto and until the arbiter is
    /// wired in. Distinct from [`Mode`]: `Mode` is the controller's own
    /// top-level state (Monitor/Manual/Calibrating/Auto), `LoopMode` is
    /// which closed loop is regulating within Auto.
    pub loop_mode: LoopMode,
    /// TempLoop's target replica temperature (°C), Some only while the
    /// arbiter has a live target (Task 15 renders it; this task only
    /// carries the field).
    pub t_star_c: Option<f64>,
    /// The controller's live `EcAverage` moving-mean replica (design §2.6),
    /// Some only while an Auto session has observed at least one sample.
    pub ec_ma_c: Option<f64>,
    /// Name of the sensor currently driving the fw-fanctrl argmax, Some
    /// only while TempLoop is (or was last) controllable.
    pub ec_argmax: Option<String>,
    /// Last commanded fw-fanctrl duty step, Some only in TempLoop.
    pub duty_cmd: Option<u8>,
    /// The DutyRpmTable-snapped fan RPM the last commanded duty implies;
    /// 0.0 outside TempLoop.
    pub snapped_rpm: f64,
    /// Name of the fw-fanctrl curve/strategy currently in force, Some only
    /// once the loop has selected one.
    pub strategy: Option<String>,
    /// The single power budget the (not-yet-wired) integrator is holding;
    /// 0.0 until the arbiter/budget machinery (a later task) drives it.
    pub budget_w: f64,
}

/// Hand-written (not derived) so `fan_target_rpm` and the floors start at
/// the real defaults instead of unrepresentable zeros in the first Status
/// event.
impl Default for ControlStatus {
    fn default() -> Self {
        let config = Config::default();
        Self {
            mode: Mode::default(),
            cpu_limit_w: None,
            gpu_max_mhz: None,
            fan_target_rpm: DEFAULT_FAN_TARGET_RPM,
            cpu_floor_w: config.cpu_floor_w,
            gpu_floor_mhz: config.gpu_floor_mhz,
            cpu_max_w: config.cpu_max_w,
            gpu_max_w: config.gpu_max_w,
            flags: Vec::new(),
            calib: None,
            loop_mode: LoopMode::default(),
            t_star_c: None,
            ec_ma_c: None,
            ec_argmax: None,
            duty_cmd: None,
            snapped_rpm: 0.0,
            strategy: None,
            budget_w: 0.0,
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
        /// Which loop produced this allocation (design §2.5's arbiter; not
        /// yet wired here — `LoopMode::default()` until it is).
        mode: LoopMode,
        /// The loop's control error (°C for TempLoop, RPM for RpmLoop per
        /// `mode`); 0.0 until the arbiter/error machinery lands.
        error: f64,
        /// The single power budget behind this allocation; 0.0 until the
        /// budget integrator (a later task) drives it.
        budget_w: f64,
        /// Set when the integrator is frozen this tick and names why (design
        /// §2.9's `Freeze` reasons); `None` while integrating normally.
        freeze: Option<&'static str>,
    },
    /// A status flag transitioned (emitted on EVERY genuine add/remove,
    /// plan Task 14); the shell mirrors it into a standalone telemetry
    /// `Record::Flag` line IN ADDITION to the Decision record (whose
    /// `flags` field carries the full post-transition list) — offline
    /// analysis gets a greppable per-flag transition stream.
    Flagged { flag: &'static str, active: bool },
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
    /// 2-state Kalman filter (design §1): `[bias, gain]`. Seeded from
    /// persistence on Auto entry (Task 6; identity until then), covariance
    /// always fresh; written back on Auto exit (Task 6). Supersedes the
    /// `Trim` integrator: `bias` carries its role and safety contract 1:1.
    kf: Kalman,
    /// Fan-RPM window feeding the adaptation steadiness gate; fan-invalid
    /// samples land as NaN (the charts/steady.rs convention), so `is_steady`
    /// rejects any tail spanning a sensor outage — never adapt across one.
    fan_window: std::collections::VecDeque<f64>,
    /// 20-sample OBSERVED-watts windows (design §3 data conventions: learn
    /// from observed watts, plan in commanded). Invalid samples land as NaN
    /// so `tail_mean` refuses to average across a sensor outage — paired
    /// with `fan_window`'s tail into one KF update.
    cpu_w_window: std::collections::VecDeque<f64>,
    gpu_w_window: std::collections::VecDeque<f64>,
    /// Ring of recent COMMANDED points for the cooldown gate (design §0).
    /// One entry per sample; capped at [`COMMANDED_RING_CAP`] — which MUST
    /// stay > WINDOW_S seconds of 1 Hz history, or the gate's coverage
    /// condition becomes unsatisfiable and adaptation silently freezes.
    commanded_ring: std::collections::VecDeque<CommandedPoint>,
    /// Model trust monitor (Task 27), fed the steady-gated |residual|
    /// against the KF-CORRECTED prediction. Lives here so trust state
    /// resets on Auto exit, like the filter.
    trust: TrustMonitor,
    /// Latest trust verdict; stands between steady windows (no evidence, no
    /// change). While true: KF updates frozen entirely.
    distrusted: bool,
    /// t_mono of the last model_snapshot record; None → snapshot on the next
    /// sample (Auto entry logs the baseline params immediately).
    last_snapshot: Option<f64>,
}

impl AutoState {
    /// `(bias, gain)` seed the Kalman filter (sanitized inside
    /// `Kalman::new`); the covariance always starts at the fresh prior.
    fn new(bias: f64, gain: f64) -> Self {
        Self {
            pid: GpuPid::new(),
            allocator: Allocator::new(),
            last_alloc: None,
            gpu_target_w: None,
            kf: Kalman::new(bias, gain),
            fan_window: std::collections::VecDeque::new(),
            cpu_w_window: std::collections::VecDeque::new(),
            gpu_w_window: std::collections::VecDeque::new(),
            commanded_ring: std::collections::VecDeque::new(),
            trust: TrustMonitor::new(),
            distrusted: false,
            last_snapshot: None,
        }
    }
}

/// Fan-RPM slope estimate (RPM/s) over the last [`FAN_SLOPE_SPAN_S`] seconds
/// of the Auto fan window: (newest − sample span back) / span across the
/// 1 Hz samples. None when the window is shorter than span+1 samples or ANY
/// sample in the span is non-finite (the fan-invalid-lands-as-NaN
/// convention): a slope bridging a sensor outage is fiction. The allocator
/// treats None as "insufficient evidence" and allows raises — see
/// `AllocInput::fan_slope_rpm_s`. `pub(crate)` so the allocator's
/// field-replay convergence test drives the exact estimator wired here.
pub(crate) fn fan_slope_rpm_s(window: &[f64]) -> Option<f64> {
    let start = window.len().checked_sub(FAN_SLOPE_SPAN_S + 1)?;
    let span = &window[start..];
    if span.iter().any(|v| !v.is_finite()) {
        return None;
    }
    Some((span[FAN_SLOPE_SPAN_S] - span[0]) / FAN_SLOPE_SPAN_S as f64)
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
    /// `t_mono` until which the post-resume elevated-stickiness window is
    /// open ([`STICKINESS_SAMPLES_STRICT`] instead of the normal streak).
    strict_until: Option<f64>,
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
    /// Wall-clock stamp of the loaded calibration, carried so an Auto-exit
    /// state write preserves it (only a finished calibration sets it).
    calibrated_at: Option<String>,
    /// Kalman `[bias, gain]` seed for the NEXT Auto entry (design §2).
    /// Loaded from the state file, captured from the live filter on every
    /// Auto exit, reset to the identity when a new calibration lands (a
    /// fresh surface invalidates old corrections). The covariance is never
    /// part of this: each session starts confident about nothing but
    /// centered on what it learned.
    persisted_bias: f64,
    persisted_gain: f64,
    /// User config (fan target, floors, fast limit). Mutated + saved when
    /// the fan target changes.
    config: Config,
    /// Where `config` persists to (`--config`).
    config_path: PathBuf,
    /// Auto-mode loop state; Some exactly while `Mode::Auto`.
    auto: Option<AutoState>,
    /// Thermal watchdog (Task 28): observes EVERY sample in EVERY mode
    /// (including Calibrating, where the rest of the sample machinery is
    /// suspended); a trip ACTS only when something is commanded.
    watchdog: ThermalWatchdog,
    /// `Effect::Flagged` transitions recorded by `add_flag`/`remove_flag`
    /// since the last drain; `on_command`/`on_sample` drain them into their
    /// returned batch so every flag transition lands in telemetry exactly
    /// once.
    pending_flags: Vec<Effect>,
    /// Debounce for the idle-Monitor watchdog warn: the immediate re-arm
    /// means a persistently hot idle machine re-trips every TRIP_STREAK
    /// samples, so warn once per continuous idle-trip episode (reset when
    /// the watchdog goes quiet again — genuinely cool/valid evidence).
    idle_trip_warned: bool,
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
        // Config owns the burst ceiling and the sustained operating max; the
        // actuator defaults only cover a hypothetical config-less construction.
        // set_sustained_max_mw re-clamps to the hardware ceiling as a backstop.
        if let Some(cpu) = guard.cpu.as_mut() {
            cpu.fast_limit_mw = config.fast_limit_mw;
            cpu.set_sustained_max_mw((config.cpu_max_w * 1000.0) as u32);
        }
        let status = ControlStatus {
            fan_target_rpm: config
                .fan_target_rpm
                .clamp(FAN_TARGET_MIN_RPM, FAN_TARGET_MAX_RPM),
            cpu_floor_w: config.cpu_floor_w,
            gpu_floor_mhz: config.gpu_floor_mhz,
            cpu_max_w: config.cpu_max_w,
            gpu_max_w: config.gpu_max_w,
            ..ControlStatus::default()
        };
        Self {
            guard,
            status,
            stick_violations: 0,
            resumed_until: None,
            strict_until: None,
            last_reassert: None,
            calib: None,
            burner: None,
            state_path,
            model: persisted.model,
            lut: persisted.lut,
            calibrated_at: persisted.calibrated_at,
            persisted_bias: persisted.adapt_bias,
            persisted_gain: persisted.adapt_gain,
            config,
            config_path,
            auto: None,
            watchdog: ThermalWatchdog::new(),
            pending_flags: Vec::new(),
            idle_trip_warned: false,
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
        // Emergency acknowledge (deliberate two-step): while a watchdog flag
        // is up, the FIRST actuating command only clears the flag(s) and
        // re-arms the watchdog — it does NOT execute. The user must see the
        // emergency and consciously press again; the second press acts
        // normally. Quit/ReleaseAll/SetFanTarget/SetFloors pass through
        // (none of them can re-apply limits behind a tripped watchdog —
        // floors only bound what a FUTURE allocation may command). The
        // latch and the
        // emergency flags move in lockstep (trip sets both, this gate clears
        // both), so gating on the latch is gating on the flags.
        if self.watchdog.is_tripped()
            && matches!(
                c,
                Command::SetCpuW(_)
                    | Command::SetGpuMaxClock(_)
                    | Command::SetAuto(true)
                    | Command::StartCalibration
            )
        {
            tracing::warn!("emergency acknowledged by {c:?}; command swallowed, watchdog re-armed");
            let mut effects = Vec::new();
            for flag in [StatusFlag::ThermalEmergency, StatusFlag::SensorLost] {
                // remove_flag records the Flagged effect on genuine removal.
                self.remove_flag(flag);
            }
            self.watchdog.rearm();
            self.drain_flag_effects(&mut effects);
            effects.push(Effect::StatusChanged {
                cause: "watchdog:rearmed",
            });
            return effects;
        }
        // While calibrating the runner owns actuation: manual setters,
        // release, Auto entry and floor edits are rejected outright
        // (Esc/AbortCalibration is the way to take control back; a floor
        // change mid-run would silently skew the calibration points).
        if self.status.mode == Mode::Calibrating
            && matches!(
                c,
                Command::SetCpuW(_)
                    | Command::SetGpuMaxClock(_)
                    | Command::ReleaseAll
                    | Command::SetAuto(_)
                    | Command::SetFloors { .. }
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
                    // fw-fanctrl-loop-j6s: non-Verified verdicts (Mismatch/
                    // Unreadable/Unverifiable) are mapped to today's plain
                    // write-failure behaviour -- the freeze/flag/reassert/
                    // three-strike wiring lands there, not here.
                    Some(cpu) => match cpu.set_sustained_mw((w * 1000.0).round() as u32) {
                        WriteVerdict::Verified(clamped_w) => {
                            self.status.cpu_limit_w = Some(clamped_w);
                            self.status.mode = Mode::Manual;
                            // Fresh command = fresh assert: any violation
                            // streak against the previous limit is stale.
                            self.stick_violations = 0;
                            effects.push(Effect::CpuSet(clamped_w));
                        }
                        verdict => {
                            tracing::warn!(
                                "SetCpuW({w}) not verified, status unchanged: {verdict:?}"
                            );
                        }
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
                // Exiting Auto too: the loop state drops whole (fresh PI +
                // conservative allocator on re-entry), with the learned
                // [bias, gain] persisted on the way out.
                self.exit_auto_and_persist();
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
            Command::SetFloors { cpu_w, gpu_mhz } => {
                // Same clamps as config load (Config::sanitized): out-of-
                // range floors would panic the GPU PI's clamp / trip the
                // allocator's debug assert on the next Auto step.
                let sanitized = Config {
                    cpu_floor_w: cpu_w,
                    gpu_floor_mhz: gpu_mhz,
                    ..self.config.clone()
                }
                .sanitized();
                if sanitized != self.config {
                    self.config = sanitized;
                    self.status.cpu_floor_w = self.config.cpu_floor_w;
                    self.status.gpu_floor_mhz = self.config.gpu_floor_mhz;
                    // The allocator reads self.config's floors on its next
                    // step, the GPU PI on its next update: no extra wiring
                    // for a live retarget. Persist on CHANGE only (a held
                    // key repeats the clamped value at the bounds — never
                    // spam the disk); save failure is warned, the in-session
                    // floors apply.
                    if let Err(e) = self.config.save(&self.config_path) {
                        tracing::warn!(
                            "config save to {} failed (floors still active): {e}",
                            self.config_path.display()
                        );
                    }
                }
                "command:set_floors"
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
                    // Seed the KF from the persisted [bias, gain] (design
                    // §2): gain is taught only at rare operating-point
                    // swings, so session-only state would relearn it every
                    // evening — persistence converts the slow learning into
                    // a one-time cost. Covariance still starts fresh (it is
                    // never persisted), and Kalman::new sanitizes a corrupt
                    // seed before it can touch the contour.
                    let mut auto = AutoState::new(self.persisted_bias, self.persisted_gain);
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
                    // The seeded bias/gain no longer mirror into
                    // `ControlStatus` (Task 4's type surface drops
                    // trim_rpm/gain); telemetry/tests read them straight off
                    // `self.auto.as_ref().unwrap().kf` instead.
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
                    // Fresh PI/allocator on re-entry; learned [bias, gain]
                    // persisted on the way out (design §2).
                    self.exit_auto_and_persist();
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
                // Clean quit persists a live Auto session's [bias, gain]
                // (design §2) before the hardware restore.
                self.exit_auto_and_persist();
                self.guard.restore_all();
                self.drain_flag_effects(&mut effects);
                effects.push(Effect::Quit);
                return effects;
            }
        };
        self.drain_flag_effects(&mut effects);
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
        // Thermal watchdog FIRST, before any mode dispatch: it observes in
        // every mode (Calibrating included — the runner's deliberate limits
        // are exactly what an emergency must release). A trip only ACTS when
        // something is commanded; in pure Monitor with nothing applied there
        // is nothing to release, so stay armed instead of latching a trip
        // that would blind the watchdog for the next Manual/Auto session.
        match self.watchdog.observe(s) {
            Trip::None => {
                // End of an idle-trip episode only on genuinely quiet
                // evidence: right after an idle re-arm the next hot/invalid
                // samples ALSO return Trip::None while the streak rebuilds,
                // and resetting on those would re-warn every TRIP_STREAK
                // samples forever.
                if self.idle_trip_warned && self.watchdog.is_quiet() {
                    self.idle_trip_warned = false;
                }
            }
            trip if self.anything_commanded() => return self.emergency_release(trip),
            trip => {
                // Diagnostically interesting even with nothing to release —
                // but warned once per continuous idle-trip episode, not on
                // every re-trip of a persistently hot idle machine.
                if !self.idle_trip_warned {
                    self.idle_trip_warned = true;
                    tracing::warn!("watchdog tripped ({trip:?}) in idle Monitor; re-arming");
                }
                self.watchdog.rearm();
            }
        }
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
            // Device-global GPU state first (persistence mode): independent
            // of whether any lock is applied, and once per resume — not on
            // the per-limit reassert below.
            if let Some(gpu) = self.guard.gpu.as_mut() {
                gpu.resumed();
            }
            // Elevated stickiness: limits are most likely to silently
            // revert right AFTER a resume, so for RESUMED_STRICT_S the
            // watchdog fires on a 2-sample streak instead of 3.
            self.strict_until = Some(s.t_mono + RESUMED_STRICT_S);
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
            // The pre-suspend windows are thermally stale (the machine
            // cooled while asleep) — clear them so the adaptation tier can't
            // fire on a 20-sample tail spanning the suspend (review finding).
            // The cooldown ring too: the commanded point may not have moved
            // across the suspend, but "stationary" pre-suspend evidence says
            // nothing about the post-resume thermal state — re-observe the
            // full 30 s window (absence of evidence is not stationarity).
            if let Some(auto) = &mut self.auto {
                auto.fan_window.clear();
                auto.cpu_w_window.clear();
                auto.gpu_w_window.clear();
                auto.commanded_ring.clear();
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
                    // Post-resume strict window: fire one sample earlier.
                    let needed = if self.strict_until.is_some_and(|until| s.t_mono < until) {
                        STICKINESS_SAMPLES_STRICT
                    } else {
                        STICKINESS_SAMPLES
                    };
                    self.stick_violations += 1;
                    if self.stick_violations >= needed {
                        // Reset so re-triggering needs a fresh streak instead
                        // of hammering ryzenadj at 1 Hz.
                        self.stick_violations = 0;
                        tracing::warn!(
                            "CPU limit not sticking: {} W measured vs {limit} W commanded \
                             ({needed} consecutive samples); reasserting",
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

        self.drain_flag_effects(&mut effects);
        if self.status != before {
            effects.push(Effect::StatusChanged {
                cause: cause.unwrap_or("sample"),
            });
        } else if let Some(cause) = cause {
            // A cause fired (e.g. a KF adaptation or model snapshot) with
            // no visible status delta — trim_rpm/gain no longer live on
            // `status` (Task 4), so a KF-only sample can no longer make
            // `self.status != before` true on its own. Still worth a
            // telemetry Decision line (mirrors `on_calib_sample`'s same
            // fallback below).
            effects.push(Effect::Noted { cause });
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
            // Plain drop, deliberately NOT exit_auto_and_persist: this is a
            // fault path (Auto lost its state mid-flight), so the seed from
            // the last CLEAN exit stands rather than whatever this session
            // had half-learned.
            self.auto = None;
            self.release_to_stock();
            effects.push(Effect::Released);
            cause.get_or_insert("auto:degraded");
            return;
        }
        let auto = self.auto.as_mut().expect("checked above");
        let model = self.model.as_ref().expect("checked above");
        let lut = self.lut.as_ref().expect("checked above");

        // Adaptation steadiness window: fan-invalid samples land as NaN (the
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
        // Observed-watts windows (design §3): NaN on invalid so `tail_mean`
        // refuses to average across an outage.
        for (win, valid, val) in [
            (&mut auto.cpu_w_window, s.cpu_pkg_w > 0.0, s.cpu_pkg_w),
            (&mut auto.gpu_w_window, s.gpu_w_valid, s.gpu_w),
        ] {
            if win.len() >= FAN_WINDOW_CAP {
                win.pop_front();
            }
            win.push_back(if valid { val } else { f64::NAN });
        }

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
            // Positive bias shifts the contour down (fewer watts): the model
            // under-predicted, so the real machine needs a smaller budget to
            // hit the target; the gain rescales the GPU-slope divisor the
            // same way. Floors still win — the allocator/PI clamps bound the
            // KF's effect (design invariant: floors > adaptation).
            let bias = auto.kf.bias();
            let gain = auto.kf.gain();
            // Fan slope AND the smoothed RPM off the SAME window that gates
            // adaptation (one borrow of the contiguous slice):
            // - the slope feeds the allocator's velocity gate — only push
            //   power when the fan response to previous pushes has been heard
            //   (2026-07 fan-lag limit-cycle fix; see `SLOPE_GATE_RPM_S`);
            // - the ~5 s tail mean feeds the deadband / raise-gate / overshoot
            //   band checks: halving the soak-noise stdev stops a single tach
            //   blip from a near-edge equilibrium firing the mandatory drain
            //   (see `FAN_SMOOTH_N`, `allocator::RAISE_HOLD_RPM`). Fallback to
            //   the raw latest sample when the window is broken by an outage
            //   (NaN tail → `tail_mean` None) — conservative, today's value.
            let fan_window = auto.fan_window.make_contiguous();
            let fan_slope = fan_slope_rpm_s(fan_window);
            let measured_fan_rpm =
                tail_mean(fan_window, FAN_SMOOTH_N).unwrap_or_else(|| s.max_fan_rpm());
            let contour = |pc: f64| model.gpu_watts_on_contour(target_rpm, bias, gain, pc);
            let (cpu_w, gpu_w) = auto.allocator.step(&AllocInput {
                contour: &contour,
                demand,
                floors: (self.config.cpu_floor_w, self.config.gpu_floor_mhz),
                measured_fan_rpm,
                fan_target_rpm: target_rpm,
                fan_valid: s.fan_valid,
                fan_slope_rpm_s: fan_slope,
                cpu_max_w: self.config.cpu_max_w,
                gpu_max_w: self.config.gpu_max_w,
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
                    // fw-fanctrl-loop-j6s: see the SetCpuW comment above --
                    // same non-Verified-as-failure mapping applies here.
                    Some(cpu) => match cpu.set_sustained_mw((cpu_w * 1000.0).round() as u32) {
                        WriteVerdict::Verified(clamped_w) => {
                            self.status.cpu_limit_w = Some(clamped_w);
                            // A violation streak measured against the OLD
                            // limit is stale evidence: the fresh allocation
                            // gets a full 3-sample streak before the
                            // stickiness watchdog may fire.
                            self.stick_violations = 0;
                            effects.push(Effect::CpuSet(clamped_w));
                        }
                        verdict => {
                            tracing::warn!(
                                "auto: CPU allocation ({cpu_w} W) not verified: {verdict:?}"
                            );
                        }
                    },
                }
            }
            // Vetoed overshoot hold (2026-07-14 design §3): a Noted line on
            // episode ENTRY, pushed BEFORE AutoAllocated so it claims the
            // batch's Decision cause (apply_effects: first claim wins). A
            // vetoed hold changes no status, so without the explicit Note
            // the crest would never surface for session grading.
            if auto.allocator.overshoot_settle_started() {
                effects.push(Effect::Noted {
                    cause: "auto:overshoot_settle",
                });
            }
            effects.push(Effect::AutoAllocated {
                demand_cpu: demand.cpu_starved,
                demand_gpu: demand.gpu_starved,
                cpu_w,
                gpu_w,
                // The arbiter (design §2.5) isn't wired up yet — this
                // adaptation-tier allocate step predates it, so the new
                // fields carry only their Task 4 defaults.
                mode: LoopMode::default(),
                error: 0.0,
                budget_w: 0.0,
                freeze: None,
            });
            cause.get_or_insert("auto:allocate");
        }

        // Cooldown ring (design §0): the commanded operating point at THIS
        // sample — one entry per second whether or not the allocator moved,
        // so the gate's trailing window is dense.
        if let (Some(pc), Some(pg)) = (self.status.cpu_limit_w, auto.gpu_target_w) {
            if auto.commanded_ring.len() >= COMMANDED_RING_CAP {
                auto.commanded_ring.pop_front();
            }
            auto.commanded_ring.push_back(CommandedPoint {
                t_mono: s.t_mono,
                cpu_w: pc,
                gpu_w: pg,
            });
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
        // this sample's allocation): the cooldown-gated 2-state Kalman
        // filter + the trust monitor (adaptation v2; design §0/§1). FOUR
        // gates stand in series, each owning one edge of the causal chain
        // `command → drawn power → fan RPM`:
        //
        //   1. COOLDOWN (design §0, first-class after the 2026-07-09
        //      staircase incident): the commanded operating point must have
        //      been stationary (±2 W, both legs) for the FULL trailing
        //      30 s, with ring coverage proving we observed that window.
        //      Fan-trace flatness cannot distinguish equilibrium from a
        //      slow coordinated ramp — the +2 W/5 s allocator staircase
        //      sailed through `is_steady` AND the achievement gate while
        //      the fans lagged 30+ s behind, winding the old trim to the
        //      −400 pin within 3 minutes of a load onset.
        //   2. STEADINESS (`is_steady`): the fan end has settled — never
        //      adapt on transients or lost sensors (non-steady/invalid
        //      samples freeze this tier exactly like the allocator).
        //   3. ACHIEVEMENT: the load actually DREW the commanded budget
        //      (margins below) — command ≈ drawn power.
        //   4. FINITE WINDOW MEANS: the 20-sample observed-watts means must
        //      exist (no sensor outage in the tail) — the KF learns at the
        //      OBSERVED operating point, so its regressor needs the same
        //      outage protection the fan window has.
        //
        // Achievement gate rationale (2026-07 field capture #3): adaptation
        // may only learn at operating points the load actually TESTED. An
        // fps-capped game left the PI target at 80 W while driver DVFS
        // could spend only 49.7 W (a max clock is not a floor; util read
        // 100%): the fans were honestly quiet, but graded against the
        // COMMANDED point the old trim wound measured−target to the −400
        // pin and a ~1450 RPM phantom residual fired ModelDistrust — so
        // when the game resumed drawing, the loop regulated fans to
        // effectively target+400 for minutes, at reduced unwind gain. An
        // under-consumed budget means the model was never exercised at the
        // commanded point: there is nothing to learn and everything to
        // corrupt.
        //
        // The gate is deliberately symmetric: budget-CUTTING (positive
        // bias) updates are skipped too while unachieved. Fans over target
        // at a half-tested point (say CPU achieved, GPU not) are already
        // handled by the allocator's model-independent overshoot backstop,
        // which cuts power regardless of the KF — while adapting from such
        // a point risks exactly this artifact class in the other
        // direction. The trust monitor does not observe AT ALL while any
        // gate is closed — no decay-toward-Ok either: an ungated sample is
        // no evidence about the model in either direction, so the verdict
        // stands frozen, and a ModelDistrust fired from artifacts clears
        // only once gated samples bring the EWMA back down (accepted).
        // FIFTH gate (2026-07-14 design §3), ahead of the other four: while
        // the allocator holds under the transient-vs-static drain veto, the
        // fan end is in a KNOWN EC transient with the command resting — so
        // the cooldown gate opens BY CONSTRUCTION and a crest plateau can
        // pass `is_steady` (the 2026-07-14 field plateau came within 46 RPM
        // of the ±100 tolerance). A sample admitted there would feed
        // +250..+400 RPM of momentum into the bias and grade the trust EWMA
        // against a transient: skip the whole tier, same shape as the
        // fan-invalid freeze. An EC transient is evidence about the EC, not
        // the model.
        let now = s.t_mono;
        let cur_cmd = (self.status.cpu_limit_w, auto.gpu_target_w);
        if s.fan_valid
            && !auto.allocator.overshoot_settle_active()
            && cooldown::cooldown_open(auto.commanded_ring.make_contiguous(), now)
            && is_steady(
                auto.fan_window.make_contiguous(),
                STEADY_N,
                STEADY_RPM_TOLERANCE,
            )
            && let (Some(cpu_cmd), Some(gpu_cmd)) = cur_cmd
            && s.cpu_pkg_w >= cpu_cmd - ACHIEVED_CPU_MARGIN_W
            && s.gpu_w >= gpu_cmd - ACHIEVED_GPU_MARGIN_W
            && let Some(measured) = tail_mean(auto.fan_window.make_contiguous(), STEADY_N)
            && let Some(pc_obs) = tail_mean(auto.cpu_w_window.make_contiguous(), STEADY_N)
            && let Some(pg_obs) = tail_mean(auto.gpu_w_window.make_contiguous(), STEADY_N)
        {
            let model = self.model.as_ref().expect("checked above");
            // KF primitives at the OBSERVED operating point (design §3:
            // learn from observed watts — calibration recorded measured
            // watts, so the surface's input domain is observed power).
            // Windowed means, mirroring calibration: raw 1 Hz jitter never
            // reaches the regressor.
            let baseline = model.a * pc_obs + model.c;
            let w = model.b * pg_obs + model.e * pc_obs * pg_obs;
            let corrected = baseline + auto.kf.bias() + auto.kf.gain() * w;
            // Trust verdict first: it decides whether this very sample may
            // adapt. Graded against the CORRECTED prediction — the surface
            // we are actually controlling with. Between gated samples the
            // last verdict stands (no evidence, no change).
            auto.distrusted =
                auto.trust.observe(now, (measured - corrected).abs()) == Trust::Distrust;
            // KF frozen ENTIRELY while distrusted (design §1: suspect
            // evidence is rare and discrete; not worth half-weighting).
            // No positive feedback in the update itself: with the allocator
            // holding the plant on the corrected contour the innovation
            // equals the control error (see kalman.rs module docs for the
            // trim incident that mandates this form).
            if !auto.distrusted && auto.kf.update(now, measured, baseline, w) {
                cause.get_or_insert("auto:kf");
            }
        }
        let offset = auto.kf.bias();
        // Model snapshot cadence check here (while `auto` is borrowed); the
        // effect is pushed below, after the flag edits release the borrow.
        let snapshot_due = auto
            .last_snapshot
            .is_none_or(|last| s.t_mono - last >= MODEL_SNAPSHOT_PERIOD_S);
        if snapshot_due {
            auto.last_snapshot = Some(s.t_mono);
        }

        // The trust verdict (`auto.distrusted`) no longer mirrors into a
        // StatusFlag — ModelDistrust is removed from the Task 4 type
        // surface (its UI/telemetry visibility is Task 12/15's concern; the
        // KF-freeze behavior above, which is what the adaptation tier's
        // tests actually exercise, is unaffected).

        // Bias saturated at +max: even the maximum budget cut cannot reach
        // the target — surface it instead of silently losing performance
        // (research 03 §6). Hysteresis: clears below 90% of max. The cause
        // is claimed only on an actual flag TRANSITION, so a later stage's
        // status change in the same sample can't get mislabeled "auto:kf".
        let flagged = self.status.flags.contains(&StatusFlag::TargetUnreachable);
        if offset >= MAX_BIAS_AUTHORITY_RPM && !flagged {
            self.add_flag(StatusFlag::TargetUnreachable);
            cause.get_or_insert("auto:kf");
        } else if offset < TRIM_CLEAR_FRACTION * MAX_BIAS_AUTHORITY_RPM && flagged {
            self.remove_flag(StatusFlag::TargetUnreachable);
            cause.get_or_insert("auto:kf");
        }

        // Periodic model snapshot for offline review (own Decision record).
        // Emitted from the first Auto sample — the baseline the later lines
        // are read against. `Effect::ModelSnapshot` is removed from the
        // Task 4 type surface (it carried no plain type — a/b/e/c came from
        // `ThermalModel`), so the cause alone travels on the effect and
        // `apply_effects` reads the live model params straight off the
        // controller when it sees this cause (values can't drift between
        // push and consumption — same on_sample call).
        if snapshot_due {
            effects.push(Effect::Noted {
                cause: "auto:model_snapshot",
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
        // A landed fit clears NotCalibrated (apply_calib_effects): the
        // transition must reach telemetry from this path too.
        self.drain_flag_effects(&mut effects);
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
                    // fw-fanctrl-loop-j6s: see the SetCpuW comment in
                    // on_command -- same non-Verified-as-failure mapping.
                    Some(cpu) => match cpu.set_sustained_mw((w * 1000.0).round() as u32) {
                        WriteVerdict::Verified(clamped_w) => {
                            self.status.cpu_limit_w = Some(clamped_w);
                        }
                        verdict => {
                            tracing::warn!("calib: SetCpuW({w}) not verified: {verdict:?}");
                        }
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
                    self.calibrated_at = state.calibrated_at.clone();
                    // A fresh surface invalidates old corrections (design
                    // §2): reset the Kalman seed to the identity. The state
                    // written below already carries identity adapt fields
                    // (the runner builds it via ..default()); this keeps
                    // the in-memory seed in lockstep so a later Auto exit
                    // cannot leak the stale pair back to disk.
                    self.persisted_bias = 0.0;
                    self.persisted_gain = 1.0;
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

    /// True when the controller has anything in force an emergency could
    /// release: any non-Monitor mode owns actuation, and applied limits
    /// count even during mode transitions (belt and suspenders — Monitor
    /// implies no limits by construction).
    fn anything_commanded(&self) -> bool {
        self.status.mode != Mode::Monitor
            || self.status.cpu_limit_w.is_some()
            || self.status.gpu_max_mhz.is_some()
    }

    /// Watchdog trip: release EVERYTHING toward stock and latch the flag.
    /// Same shape as ReleaseAll (gpu release + cpu restore_stock) plus a
    /// calibration abort (burner stopped) and an Auto exit (AutoState
    /// dropped). Afterwards `status.cpu_limit_w`/`gpu_max_mhz` are None, so
    /// the stickiness/reassert machinery has nothing to reapply — the
    /// release holds until the user re-arms (see the acknowledge gate in
    /// `on_command`).
    fn emergency_release(&mut self, trip: Trip) -> Vec<Effect> {
        let (flag, cause) = match trip {
            Trip::Thermal => (StatusFlag::ThermalEmergency, "watchdog:thermal_emergency"),
            Trip::SensorLost => (StatusFlag::SensorLost, "watchdog:sensor_lost"),
            Trip::None => unreachable!("emergency_release called without a trip"),
        };
        tracing::warn!("{cause}: releasing all limits toward stock (manual re-arm required)");
        // A running calibration aborts first: burner threads stopped and the
        // runner's own releases applied (same order as the Quit path).
        if let Some(mut runner) = self.calib.take() {
            self.apply_calib_effects(runner.abort());
            self.end_calibration();
        }
        // Auto exits hard: dropping AutoState means no allocator/PI step can
        // ever re-command until a fresh (post-re-arm) Auto entry. The
        // learned [bias, gain] still persists — the trip is thermal, not
        // evidence against the correction, and the clamps bound any harm.
        self.exit_auto_and_persist();
        self.release_to_stock();
        self.add_flag(flag);
        let mut effects = vec![Effect::Released];
        // Carries the emergency flag itself PLUS whatever release_to_stock
        // genuinely cleared (LimitNotSticking/TargetUnreachable/...).
        self.drain_flag_effects(&mut effects);
        effects.push(Effect::StatusChanged { cause });
        effects
    }

    /// Write model + LUT + Kalman `[bias, gain]` to the state file (design
    /// §2). Called from [`exit_auto_and_persist`](Self::exit_auto_and_persist)
    /// only — never per-update (no disk churn) — and the covariance is never
    /// serialized. Save failure is warned, not fatal: the in-memory seed
    /// still carries the session.
    fn save_persisted_state(&self) {
        let state = PersistedState {
            model: self.model.clone(),
            lut: self.lut.clone(),
            calibrated_at: self.calibrated_at.clone(),
            adapt_bias: self.persisted_bias,
            adapt_gain: self.persisted_gain,
        };
        if let Err(e) = state.save(&self.state_path) {
            tracing::warn!(
                "adapt: state save to {} failed: {e}",
                self.state_path.display()
            );
        }
    }

    /// Drop the Auto loop state, capturing the live Kalman `[bias, gain]`
    /// into the persisted seed and writing the state file (design §2:
    /// persist on Auto exit; a quit from Auto is covered because every exit
    /// path funnels through here). No-op when not in Auto: a Monitor/Manual
    /// session learned nothing and must not churn the state file.
    fn exit_auto_and_persist(&mut self) {
        if let Some(auto) = self.auto.take() {
            self.persisted_bias = auto.kf.bias();
            self.persisted_gain = auto.kf.gain();
            self.save_persisted_state();
        }
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
        // KF + trust state live in AutoState (dropped by every Auto exit
        // path before reaching here); `status` no longer mirrors trim/gain
        // (Task 4), so there is nothing left to reset there.
        self.remove_flag(StatusFlag::TargetUnreachable);
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
            // fw-fanctrl-loop-j6s: see the SetCpuW comment in on_command --
            // same non-Verified-as-failure mapping; `all_ok` still only
            // tracks whether the reassert attempt landed, same as before.
            if !matches!(
                cpu.set_sustained_mw((w * 1000.0).round() as u32),
                WriteVerdict::Verified(_)
            ) {
                all_ok = false;
                tracing::warn!("reassert: CPU limit ({w} W) not verified");
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

    /// Set a flag. A GENUINE insertion (not already set) also records an
    /// `Effect::Flagged { active: true }` into `pending_flags` — plan Task
    /// 14 promises a telemetry Flag line on every status-flag transition.
    fn add_flag(&mut self, flag: StatusFlag) {
        if !self.status.flags.contains(&flag) {
            self.status.flags.push(flag);
            self.pending_flags.push(Effect::Flagged {
                flag: flag.as_str(),
                active: true,
            });
        }
    }

    /// Clear a flag; a GENUINE removal records `Flagged { active: false }`
    /// (idempotent re-clears leave no trace — see [`Self::add_flag`]).
    fn remove_flag(&mut self, flag: StatusFlag) {
        let before = self.status.flags.len();
        self.status.flags.retain(|&f| f != flag);
        if self.status.flags.len() != before {
            self.pending_flags.push(Effect::Flagged {
                flag: flag.as_str(),
                active: false,
            });
        }
    }

    /// Move the flag transitions recorded since the last drain into
    /// `effects`. Every `on_command`/`on_sample` return path that could have
    /// touched a flag drains, so each transition is emitted exactly once.
    fn drain_flag_effects(&mut self, effects: &mut Vec<Effect>) {
        effects.append(&mut self.pending_flags);
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
    let mut model_snapshot: Option<(f64, f64, f64, f64)> = None;
    // Status-flag transitions in this batch (watchdog or otherwise): each
    // becomes a standalone Record::Flag line (in addition to the Decision
    // carrying the full list).
    let mut flagged: Vec<(&'static str, bool)> = Vec::new();
    for effect in effects {
        match effect {
            Effect::Reasserted { cause: c } => {
                cause.get_or_insert(c);
            }
            Effect::Noted { cause: c } => {
                // "auto:model_snapshot" carries no payload of its own
                // (`Effect::ModelSnapshot` is removed from the Task 4 type
                // surface); read the live model params straight off the
                // controller instead — they can't have moved between this
                // effect's push and this read, both within the same
                // on_sample call.
                if *c == "auto:model_snapshot"
                    && let Some(m) = controller.model.as_ref()
                {
                    model_snapshot = Some((m.a, m.b, m.e, m.c));
                }
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
                ..
            } => {
                auto_alloc = Some((*demand_cpu, *demand_gpu, *cpu_w, *gpu_w));
                cause.get_or_insert("auto:allocate");
            }
            Effect::Flagged { flag, active } => flagged.push((flag, *active)),
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
    // a STANDALONE record for a model snapshot in the same batch — separate
    // lines, so neither cause can shadow the other in offline review.
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
            // non-auto lines skip it to stay lean. `ControlStatus` no
            // longer carries trim_rpm/gain (Task 4), so read them straight
            // off the live KF — `controller.auto` is Some exactly while
            // Mode::Auto, giving the same gating the old status field did.
            trim_rpm: controller.auto.as_ref().map(|a| a.kf.bias()),
            gain: controller.auto.as_ref().map(|a| a.kf.gain()),
            model_a: model.map(|m| m.0),
            model_b: model.map(|m| m.1),
            model_e: model.map(|m| m.2),
            model_c: model.map(|m| m.3),
        }
    };
    if cause.is_some() || model_snapshot.is_some() || !flagged.is_empty() {
        if let Some(t) = telemetry::lock(telemetry).as_mut() {
            // Flag transitions first: the Decision that follows already
            // shows the post-transition flag list.
            for (flag, active) in flagged {
                t.log(&Record::Flag {
                    t_mono,
                    flag: flag.to_string(),
                    active,
                });
            }
            if let Some(cause) = cause {
                t.log(&decision(cause, auto_alloc, None));
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

    // --- Task 4: StatusFlag severity coverage ---

    /// Hand-maintained (not derived) so a new `StatusFlag` variant left out
    /// here is a silent test gap rather than a compile error — the
    /// EXHAUSTIVE MATCH inside `flag_severity` is what actually forces every
    /// variant to be classified; this list drives the coverage test below
    /// over that same fixed set.
    const ALL_STATUS_FLAGS: [StatusFlag; 13] = [
        StatusFlag::LimitNotSticking,
        StatusFlag::Resumed,
        StatusFlag::NotCalibrated,
        StatusFlag::TargetUnreachable,
        StatusFlag::ThermalEmergency,
        StatusFlag::SensorLost,
        StatusFlag::FanctrlLost,
        StatusFlag::EcMismatch,
        StatusFlag::SteepCurve,
        StatusFlag::CurveInvalid,
        StatusFlag::GpuHot,
        StatusFlag::NvmeHot,
        StatusFlag::ReadbackBlind,
    ];

    #[test]
    fn flag_severity_covers_every_flag() {
        // NOTE: this cannot actually fail. `flag_severity`'s match has no
        // wildcard arm, so the real coverage guarantee — every StatusFlag
        // variant maps to a Severity — is enforced at COMPILE time (add a
        // variant without extending the match and the build breaks); this
        // loop only documents that guarantee against the hand-maintained
        // list the brief asks for. Kept as a named anchor for that list
        // rather than removed, since a future variant missing from
        // `ALL_STATUS_FLAGS` (unlike one missing from the match) would
        // compile silently.
        for flag in ALL_STATUS_FLAGS {
            let _ = flag_severity(flag);
        }
    }

    #[test]
    fn curve_invalid_is_warning_steep_curve_is_info_and_they_differ() {
        // A permanent loss of Mode A (CurveInvalid) must not share the
        // informational SteepCurve severity (brief, design §2.7/§2.8).
        assert_eq!(flag_severity(StatusFlag::CurveInvalid), Severity::Warning);
        assert_eq!(flag_severity(StatusFlag::SteepCurve), Severity::Info);
        assert_ne!(
            flag_severity(StatusFlag::CurveInvalid),
            flag_severity(StatusFlag::SteepCurve)
        );
    }

    // --- Task 4: ControlStatus's new field set ---

    #[test]
    fn control_status_carries_the_new_loop_fields() {
        // Constructs every field the Task 4 type surface adds to
        // ControlStatus and reads each back — a stray typo'd field name or
        // wrong type here would fail to compile, and a wrong value read
        // back would fail one of these asserts.
        let cs = ControlStatus {
            loop_mode: LoopMode::TempLoop,
            t_star_c: Some(62.5),
            ec_ma_c: Some(61.0),
            ec_argmax: Some("cpu".to_string()),
            duty_cmd: Some(7),
            snapped_rpm: 3200.0,
            strategy: Some("balanced".to_string()),
            budget_w: 45.0,
            ..ControlStatus::default()
        };
        assert_eq!(cs.loop_mode, LoopMode::TempLoop);
        assert_eq!(cs.t_star_c, Some(62.5));
        assert_eq!(cs.ec_ma_c, Some(61.0));
        assert_eq!(cs.ec_argmax.as_deref(), Some("cpu"));
        assert_eq!(cs.duty_cmd, Some(7));
        assert_eq!(cs.snapped_rpm, 3200.0);
        assert_eq!(cs.strategy.as_deref(), Some("balanced"));
        assert_eq!(cs.budget_w, 45.0);
    }

    #[test]
    fn control_status_default_has_no_loop_state_yet() {
        // The arbiter isn't wired up in this task — a fresh status must
        // read as "nothing decided yet", not as a stale TempLoop/RpmLoop.
        let cs = ControlStatus::default();
        assert_eq!(cs.loop_mode, LoopMode::Released);
        assert_eq!(cs.t_star_c, None);
        assert_eq!(cs.ec_ma_c, None);
        assert_eq!(cs.ec_argmax, None);
        assert_eq!(cs.duty_cmd, None);
        assert_eq!(cs.snapped_rpm, 0.0);
        assert_eq!(cs.strategy, None);
        assert_eq!(cs.budget_w, 0.0);
    }

    // --- Task 4: Effect::AutoAllocated's new field set ---

    #[test]
    fn auto_allocated_carries_the_new_arbiter_fields() {
        // Destructure by name (not `..` from a match) so a typo'd or
        // missing field name fails to compile rather than being silently
        // ignored by an `_` pattern.
        let Effect::AutoAllocated {
            demand_cpu,
            demand_gpu,
            cpu_w,
            gpu_w,
            mode,
            error,
            budget_w,
            freeze,
        } = (Effect::AutoAllocated {
            demand_cpu: 20.0,
            demand_gpu: 30.0,
            cpu_w: 18.0,
            gpu_w: 28.0,
            mode: LoopMode::RpmLoop,
            error: -4.5,
            budget_w: 46.0,
            freeze: Some("actuator_mismatch"),
        })
        else {
            unreachable!()
        };
        assert_eq!(
            (demand_cpu, demand_gpu, cpu_w, gpu_w),
            (20.0, 30.0, 18.0, 28.0)
        );
        assert_eq!(mode, LoopMode::RpmLoop);
        assert_eq!(error, -4.5);
        assert_eq!(budget_w, 46.0);
        assert_eq!(freeze, Some("actuator_mismatch"));
    }

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

    /// The commanded (write) `ryzenadj` calls only -- excludes the `--info`
    /// read-back calls that `CpuActuator::set_sustained_mw` now makes after
    /// every write (design §2.9, fw-fanctrl-loop-jpg): existing tests using
    /// this helper assert "what did we command", which read-back is not.
    fn ryzenadj_calls(runner: &FakeRunner) -> Vec<Vec<String>> {
        runner
            .calls()
            .into_iter()
            .filter(|(prog, args)| prog == "ryzenadj" && !(args.len() == 1 && args[0] == "--info"))
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

        // Third consecutive violation: immediate reassert + flag (with its
        // Flagged effect — every genuine transition reaches telemetry).
        let effects = ctl.on_sample(&sample_with_power(3.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        assert!(has_flagged(&effects, "limit_not_sticking", true));
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ryzenadj_calls(&runner).len(), 2, "initial set + reassert");

        // A compliant sample clears the flag (Flagged again, active=false).
        let effects = ctl.on_sample(&sample_with_power(4.0, 19.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        assert!(has_flagged(&effects, "limit_not_sticking", false));
        assert_eq!(status_changes(&effects), 1);

        // Staying compliant is NOT a transition: no Flagged spam.
        let effects = ctl.on_sample(&sample_with_power(5.0, 19.0));
        assert!(
            !effects.iter().any(|e| matches!(e, Effect::Flagged { .. })),
            "got {effects:?}"
        );
    }

    #[test]
    fn stickiness_flag_transitions_reach_telemetry_as_flag_lines() {
        // Task 14 promise: EVERY status-flag transition lands as a
        // standalone Record::Flag JSONL line, not just the watchdog's.
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-flag-telemetry",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let mut ctl = controller_no_profile(&runner);
        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));

        let effects = ctl.on_command(Command::SetCpuW(20.0));
        apply_effects(&effects, &ctl, 0.0, &ui_tx, &telemetry);
        // Three violations set the flag, one compliant sample clears it.
        for t in 1..=3 {
            let effects = ctl.on_sample(&sample_with_power(f64::from(t), 26.0));
            apply_effects(&effects, &ctl, f64::from(t), &ui_tx, &telemetry);
        }
        let effects = ctl.on_sample(&sample_with_power(4.0, 19.0));
        apply_effects(&effects, &ctl, 4.0, &ui_tx, &telemetry);
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let t = guard.as_mut().unwrap();
            t.flush();
            t.path().to_path_buf()
        };

        let contents = fs::read_to_string(&path).unwrap();
        let flags: Vec<serde_json::Value> = contents
            .lines()
            .map(|l| serde_json::from_str(l).unwrap())
            .filter(|v: &serde_json::Value| v["kind"] == "flag")
            .collect();
        assert_eq!(flags.len(), 2, "set + clear, got: {contents}");
        assert_eq!(flags[0]["flag"], "limit_not_sticking");
        assert_eq!(flags[0]["active"], true);
        assert_eq!(flags[0]["t_mono"], 3.0);
        assert_eq!(flags[1]["flag"], "limit_not_sticking");
        assert_eq!(flags[1]["active"], false);
        assert_eq!(flags[1]["t_mono"], 4.0);

        fs::remove_dir_all(&dir).unwrap();
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
            // A healthy machine reports a valid, cool Tctl: without this the
            // long calibration drives would trip the sensor-lost watchdog.
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
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
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
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
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
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
            ..PersistedState::default()
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
            // Valid, cool Tctl: long Auto drives must not starve the
            // watchdog's sensor-lost streak into a phantom trip.
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
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
        // commands FF(32 W) = 1253 MHz + the unclamped +440 MHz correction
        // (error 22 W: P 110 + I 330; within the 1000 MHz authority).
        let effects = ctl.on_sample(&busy_at(0.0));
        assert_eq!(alloc_of(&effects), Some((17.0, 32.0)));
        assert_eq!(ryzenadj_calls(&runner), vec![expected_args(17_000)]);
        assert_eq!(ctl.status().cpu_limit_w, Some(17.0));
        assert!(effects.contains(&Effect::CpuSet(17.0)), "got {effects:?}");
        assert_eq!(gpu_sets(&gpu_calls), vec![1693]);
        assert_eq!(ctl.status().gpu_max_mhz, Some(1693));
        assert!(effects.contains(&Effect::GpuSet(1693)), "got {effects:?}");
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
        // overshoot on a collapsed contour — ups forbidden, hard cuts: gpu
        // at the 16 W overshoot rate, cpu by the backstop's minimum 2 W cut
        // (the candidate wants MORE cpu, but overshoot means every axis must
        // genuinely decrease until the floors).
        ctl.on_command(Command::SetFanTarget(1000.0));
        let effects = ctl.on_sample(&busy_at(10.0));
        assert_eq!(
            alloc_of(&effects),
            Some((17.0, 18.0)),
            "next step must consume the new target (gpu cut at the overshoot rate)"
        );
    }

    #[test]
    fn fan_slope_estimate_needs_full_valid_span() {
        // Shorter than span+1 samples → None (insufficient evidence).
        assert_eq!(fan_slope_rpm_s(&[1500.0; FAN_SLOPE_SPAN_S]), None);
        // Flat 11-sample window → 0 RPM/s; a 100 RPM rise over the span →
        // +10 RPM/s.
        let mut w = vec![1500.0; FAN_SLOPE_SPAN_S + 1];
        assert_eq!(fan_slope_rpm_s(&w), Some(0.0));
        w[FAN_SLOPE_SPAN_S] = 1600.0;
        assert_eq!(fan_slope_rpm_s(&w), Some(10.0));
        // Only the span tail counts: older garbage (even NaN) is ignored.
        let mut w = vec![f64::NAN; 5];
        w.extend((0..=FAN_SLOPE_SPAN_S).map(|i| 1400.0 + 30.0 * i as f64));
        assert_eq!(fan_slope_rpm_s(&w), Some(30.0));
        // NaN inside the span (fan-invalid sample) → None: never estimate a
        // slope across a sensor outage.
        let mut w = vec![1500.0; FAN_SLOPE_SPAN_S + 1];
        w[5] = f64::NAN;
        assert_eq!(fan_slope_rpm_s(&w), None);
    }

    #[test]
    fn rising_fan_window_gates_allocator_raises() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Fans climbing 30 RPM/s toward the (still far) target: mid-cycle
        // territory for the velocity gate.
        let rising = |t: f64| Sample {
            fan1_rpm: 1400.0 + 30.0 * t,
            ..busy_at(t)
        };
        let mut allocs = Vec::new();
        for t in 0..=10 {
            if let Some(a) = alloc_of(&ctl.on_sample(&rising(f64::from(t)))) {
                allocs.push(a);
            }
        }
        // t=0 and t=5: window too short for a slope (None) → raises proceed.
        assert_eq!(allocs[0], (17.0, 32.0));
        assert_eq!(allocs[1], (19.0, 34.0));
        // t=10: 11 samples of +30 RPM/s → the allocator holds the raise.
        assert_eq!(allocs[2], allocs[1], "climbing fans must gate the raise");

        // Fans flatten: once the slope span is flat again, raises resume.
        let flat = |t: f64| Sample {
            fan1_rpm: 1700.0,
            ..busy_at(t)
        };
        let mut resumed = Vec::new();
        for t in 11..=25 {
            if let Some(a) = alloc_of(&ctl.on_sample(&flat(f64::from(t)))) {
                resumed.push(a);
            }
        }
        // t=15: the span still remembers the climb (slope > gate) → hold;
        // t=20 and beyond: flat span → the raise proceeds again.
        assert_eq!(resumed[0], allocs[2], "slope memory must keep the gate");
        assert!(
            resumed[1].0 > allocs[2].0 && resumed[1].1 > allocs[2].1,
            "flat window must let raises proceed: {resumed:?}"
        );
    }

    #[test]
    fn flat_fan_window_lets_raises_proceed() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        // busy_at holds fan1_rpm at a constant 1700: every slope estimate is
        // 0 RPM/s once the window fills, and the allocator keeps climbing at
        // the up rate exactly as before the gate existed — until the GPU leg
        // enters the approach taper band of its candidate, where the last
        // watts land at UP_RATE_TAPER_W (36 → 37, not 38).
        let mut allocs = Vec::new();
        for t in 0..=15 {
            if let Some(a) = alloc_of(&ctl.on_sample(&busy_at(f64::from(t)))) {
                allocs.push(a);
            }
        }
        assert_eq!(
            allocs,
            vec![(17.0, 32.0), (19.0, 34.0), (21.0, 36.0), (23.0, 37.0)]
        );
    }

    /// GPU-heavy soak sample: fan reading configurable, an invalid fan lands
    /// as NaN in the window (the steady.rs convention). CPU stays idle so the
    /// allocation rests GPU-only; the fan sits at the 3000 RPM default target,
    /// so the raise gate holds the loop at the conservative (15, 30) start —
    /// the fixed prior state every band-check assertion below cuts from.
    fn soak_sample(t: f64, fan: f64, fan_valid: bool) -> Sample {
        Sample {
            t_mono: t,
            gpu_util_pct: 100.0,
            gpu_w: 40.0,
            gpu_w_valid: true,
            fan1_rpm: fan,
            fan_valid,
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            ..Sample::default()
        }
    }

    #[test]
    fn allocator_sees_the_smoothed_fan_not_a_single_spike() {
        // The band checks (deadband / raise gate / overshoot) must see the
        // FAN_SMOOTH_N tail mean, not the last raw sample: one tach blip from
        // a near-target equilibrium is exactly what fired the field relay
        // (run-1783720682). A single +300 RPM spike on the last sample of an
        // otherwise in-band window averages to target+60 across the 5-sample
        // tail — under the +150 overshoot line — so the overshoot regime must
        // NOT engage; five consecutive elevated samples DO cross it. (Since
        // the 2026-07-14 drain veto, crossing from a contour-covered held
        // point HOLDS rather than cuts — the regime entry is observed via
        // `overshoot_settle_active`, not a drained watt.)

        // Single spike: warm 30 in-band samples (loop holds at (15, 30)), then
        // one +300 blip at the t=30 allocator step.
        let runner_a = FakeRunner::new();
        let (mut ctl_a, _gpu_a) = auto_controller_no_profile(&runner_a);
        ctl_a.on_command(Command::SetAuto(true));
        for t in 0..=29 {
            ctl_a.on_sample(&soak_sample(f64::from(t), 3000.0, true));
        }
        // t=30 alloc: tail = [3000, 3000, 3000, 3000, 3300] → mean 3060.
        let spike = alloc_of(&ctl_a.on_sample(&soak_sample(30.0, 3300.0, true)))
            .expect("t=30 is an allocator step");

        // Sustained: the same held start, then five consecutive +300 samples
        // fill the smoothing tail before the t=30 step.
        let runner_b = FakeRunner::new();
        let (mut ctl_b, _gpu_b) = auto_controller_no_profile(&runner_b);
        ctl_b.on_command(Command::SetAuto(true));
        for t in 0..=25 {
            ctl_b.on_sample(&soak_sample(f64::from(t), 3000.0, true));
        }
        for t in 26..=29 {
            ctl_b.on_sample(&soak_sample(f64::from(t), 3300.0, true));
        }
        // t=30 alloc: tail = [3300; 5] → mean 3300 > target + 150.
        let sustained = alloc_of(&ctl_b.on_sample(&soak_sample(30.0, 3300.0, true)))
            .expect("t=30 is an allocator step");

        assert_eq!(
            spike.1, 30.0,
            "one blip must not cut the allocation: {spike:?}"
        );
        assert!(
            !ctl_a
                .auto
                .as_ref()
                .unwrap()
                .allocator
                .overshoot_settle_active(),
            "one blip must not enter the overshoot regime at all"
        );
        // The sustained crest engages the overshoot regime — proven by the
        // drain veto holding (the contour covers the held (15, 30) point) —
        // and the allocation is NOT raised.
        assert_eq!(
            sustained.1, 30.0,
            "the veto must hold the sustained crest: {sustained:?}"
        );
        assert!(
            ctl_b
                .auto
                .as_ref()
                .unwrap()
                .allocator
                .overshoot_settle_active(),
            "five elevated samples must engage the overshoot regime"
        );
    }

    #[test]
    fn broken_smoothing_window_falls_back_to_the_raw_sample() {
        // A fan-invalid sample (NaN) inside the last 5 breaks tail_mean → the
        // allocator falls back to the RAW latest reading (design: fallback
        // matters only when the CURRENT sample is valid but the window is
        // broken — an invalid current sample takes the freeze path instead).
        // The raw +300 value crosses the overshoot line even though the
        // 5-sample MEAN would not (cf. the spike case above), proving the raw
        // fallback — not a mean over the outage — reached the band check.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=25 {
            ctl.on_sample(&soak_sample(f64::from(t), 3000.0, true));
        }
        ctl.on_sample(&soak_sample(26.0, 3000.0, true));
        ctl.on_sample(&soak_sample(27.0, 0.0, false)); // fan invalid → NaN in tail
        ctl.on_sample(&soak_sample(28.0, 3000.0, true));
        ctl.on_sample(&soak_sample(29.0, 3000.0, true));
        // t=30 alloc: tail = [3000, NaN, 3000, 3000, 3300] → tail_mean None →
        // fallback to the raw 3300 → the overshoot regime engages (as the
        // 5-sample MEAN would not — cf. the spike case above), proving the
        // raw fallback reached the band check. Regime entry shows as the
        // drain veto holding (the contour covers the held point).
        let out = alloc_of(&ctl.on_sample(&soak_sample(30.0, 3300.0, true)))
            .expect("t=30 is an allocator step");
        assert_eq!(
            out.1, 30.0,
            "the veto must hold the raw-fallback crest: {out:?}"
        );
        assert!(
            ctl.auto
                .as_ref()
                .unwrap()
                .allocator
                .overshoot_settle_active(),
            "raw fallback must engage the overshoot regime when smoothing is broken"
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
        // FF(30)=1200; error 20 W wants +700 of correction, delivered in
        // rate-limited steps: 1600 (t=0), 1705 (t=5), 1810 (t=10.1).
        assert_eq!(ctl.status().gpu_max_mhz, Some(1810));
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
        // Concretely: rate-limited toward the ~1693 MHz desired clock.
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
        // again), not from the failed 1605 intent (which would allow 1710).
        ctl.on_sample(&busy_at(1.0));
        assert_eq!(gpu_sets(&gpu_calls), vec![1500, 1605]);
        assert_eq!(ctl.status().gpu_max_mhz, Some(1605));
    }

    // --- Adaptation tier: cooldown-gated Kalman filter (adaptation v2) ---

    /// `busy_at` with a chosen fan reading (the adaptation window and the
    /// allocator's deadband/overshoot checks watch the fan).
    fn busy_fan_at(t: f64, fan_rpm: f64) -> Sample {
        Sample {
            fan1_rpm: fan_rpm,
            ..busy_at(t)
        }
    }

    /// `busy_fan_at` with the measured draws tracking what the controller
    /// currently commands (RAPL at the CPU limit, GPU at the PI watts
    /// target): the plant actually SPENDS its budget, so the adaptation
    /// tier's achievement gate sees a genuinely tested operating point.
    /// The GPU draw mirrors the target even at 0 W: the KF learns at
    /// OBSERVED watts, so busy_at's phantom 10 W draw against a 0 W command
    /// would silently shift the regressor off the pinned fixture's w = 0.
    /// Before the first allocation the plain busy draws stand in. Bind the
    /// sample before `on_sample` (the receiver borrow overlaps otherwise).
    fn achieved_fan_at(ctl: &Controller<&FakeRunner>, t: f64, fan_rpm: f64) -> Sample {
        let mut s = busy_fan_at(t, fan_rpm);
        if let Some(w) = ctl.status().cpu_limit_w {
            s.cpu_pkg_w = w;
        }
        if let Some(w) = ctl.auto.as_ref().and_then(|a| a.gpu_target_w) {
            s.gpu_w = w;
        }
        s
    }

    #[test]
    fn kf_adaptation_waits_out_the_cooldown_window() {
        // The 2026-07-09 incident shape, silenced (design §0): a demand-
        // driven climb walks the commanded point every 5 s, and although
        // the fan window is steady from t=19 and every point is achieved,
        // the cooldown gate keeps the WHOLE adaptation tier silent — the
        // trust EWMA staying at exactly 0.0 proves not a single sample got
        // through (a KF update rejected by the gain clamp would still have
        // fed trust first).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Phase 1: fans flat at 1700 (steady), draws tracking the commands
        // (achieved). The allocator staircases from (17, 32) toward the
        // 3000 RPM contour optimum (~(40, 63)): while the climb is moving
        // (+2 W / 5 s through at least t=70) the trailing 30 s always
        // contains points ≥ 4 W off the current one, so the gate is CLOSED
        // — proven by the trust EWMA still being exactly 0.0 at t=70 (a KF
        // update rejected by the gain clamp would have fed trust first).
        for t in 0..=70 {
            let s = achieved_fan_at(&ctl, f64::from(t), 1700.0);
            ctl.on_sample(&s);
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                0.0,
                "bias moved at t={t}"
            );
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.gain(),
                1.0,
                "gain moved at t={t}"
            );
        }
        assert_eq!(
            ctl.auto.as_ref().unwrap().trust.ewma(),
            0.0,
            "cooldown must gate the tier BEFORE trust is fed"
        );

        // Tail of phase 1: the allocation parks and the gate opens once the
        // trailing 30 s is stationary. Fans pinned at 1700 against a
        // ~3000-predicting point is a huge discrepancy — under the
        // 2026-07-10 tuning the KF ACCEPTS it and lets the BOUNDED states
        // do their jobs: bias (saturating, minutes to unwind) walks down in
        // k·innovation steps while gain creeps at ≤ MAX_GAIN_STEP per
        // update — one sample must never decide the slope (the incident:
        // the loose prior let the first gated sample slam gain to the 0.6
        // floor; the reject-whole box itself stays unit-tested in
        // kalman.rs).
        let mut prev = (
            ctl.auto.as_ref().unwrap().kf.bias(),
            ctl.auto.as_ref().unwrap().kf.gain(),
        );
        for t in 71..100 {
            let s = achieved_fan_at(&ctl, f64::from(t), 1700.0);
            ctl.on_sample(&s);
            let cur = (
                ctl.auto.as_ref().unwrap().kf.bias(),
                ctl.auto.as_ref().unwrap().kf.gain(),
            );
            assert!(
                cur.0 <= prev.0,
                "bias may only walk DOWN against under-target fans at t={t}"
            );
            assert!(
                (cur.1 - prev.1).abs() <= crate::control::kalman::MAX_GAIN_STEP + 1e-12,
                "gain slammed at t={t}: {} -> {}",
                prev.1,
                cur.1
            );
            prev = cur;
        }

        // Phase 2: hold everything flat — the plant now reads the KF-
        // CORRECTED prediction at the commanded (= observed) point plus
        // 80 RPM, a constant offset the bias can absorb (tracking the
        // corrected surface keeps the innovation at exactly +80 regardless
        // of what phase 1 left in the state). Adaptation is asserted on
        // the STATE, not the "auto:kf" Decision cause: the gate reopens
        // 30 s after the last allocation move, which in this sim is always
        // on the 5 s allocator grid, so the cause is shadowed by that
        // sample's "auto:allocate" here (the Decision record still carries
        // the moved bias/gain; field telemetry 2026-07-10 shows the cause
        // surfacing off-grid in reality, t=207.1).
        //
        // Instrumented cadence (kept for the field): each accepted update
        // shifts the corrected contour enough that the allocator re-steps
        // past its deadband, closing the cooldown for another 30 s — the
        // KF self-throttles to ~1 update per 35–40 s while its own
        // corrections are still moving the plant, so the 120 s window
        // below carries ~3 updates of ≈ +2.4 RPM bias each.
        let m = fitted_model();
        let before_bias = ctl.auto.as_ref().unwrap().kf.bias();
        for t in 100..220 {
            let pc = ctl.status().cpu_limit_w.unwrap();
            let pg = ctl.auto.as_ref().unwrap().gpu_target_w.unwrap();
            let (bias, gain) = (
                ctl.auto.as_ref().unwrap().kf.bias(),
                ctl.auto.as_ref().unwrap().kf.gain(),
            );
            let w = (m.b + m.e * pc) * pg;
            let corrected = m.a * pc + m.c + bias + gain * w;
            let s = achieved_fan_at(&ctl, f64::from(t), corrected + 80.0);
            ctl.on_sample(&s);
        }
        let bias = ctl.auto.as_ref().unwrap().kf.bias();
        assert!(
            bias > before_bias + 5.0,
            "KF must absorb the +80 offset once the hold is proven: \
             bias {before_bias} -> {bias}"
        );
    }

    /// Controller-level replay of the captured 2026-07-09 wind-up incident
    /// (design §6): an idle→load onset drives the real allocator's +2 W/5 s
    /// staircase while the fans answer through a dead-time + first-order
    /// lag, over a model that UNDER-predicts by a constant in-authority
    /// offset (the field bias). The fielded trim integrated this wind-up to
    /// its −400 pin within 3 minutes, overshot +810 RPM, limit-cycled, and
    /// fired a false ModelDistrust.
    ///
    /// This test tracked the ORIGINAL adaptation-v2 watch-item: under CONSTANT
    /// max demand the plant+allocator settled into a mild residual relay
    /// (commanded point moving every ≤25 s), the 30 s cooldown gate never
    /// opened, and the bias was never learned in-cycle — adaptation starved
    /// rather than poisoned. The 2026-07-10 raise gate + RPM smoothing
    /// (allocator::RAISE_HOLD_RPM, FAN_SMOOTH_N) RESOLVE exactly that
    /// watch-item: the raise gate stops the re-climb at target−50 so the loop
    /// finally RESTS ≥30 s, the cooldown opens (~t=165 here), and the KF
    /// learns the in-authority under-prediction — driving the fans onto
    /// target (mean |err| ≈16 RPM over the last 200 s, vs the relay it
    /// replaced). The incident's own failure modes stay closed throughout:
    /// no false ModelDistrust ever, and overshoot bounded to a fraction of
    /// the field's +810 RPM. (Genuine ≥30 s dwells also learn on fluctuating
    /// loads — see kf_adaptation_waits_out_the_cooldown_window.)
    #[test]
    fn wind_up_staircase_rests_and_adapts() {
        // Plant: fans reach `predict(pc, pg) + PLANT_BIAS` through a 10 s
        // dead time and a 15 s first-order response ("fans lag power
        // ~15-25 s late"); PLANT_BIAS is IN authority, so the whole error
        // is the KF's to absorb — at a genuine dwell.
        const PLANT_BIAS: f64 = 250.0;
        const TAU: f64 = 15.0;
        const DEAD: usize = 10;
        const TARGET: f64 = 3000.0; // Config::default() fan target
        let m = fitted_model();
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        let mut rpm = m.predict(0.0, 0.0) + PLANT_BIAS; // idle-settled fans
        let mut pipe: std::collections::VecDeque<(f64, f64)> =
            std::iter::repeat_n((0.0, 0.0), DEAD).collect();
        let mut last_cmd: Option<(f64, f64)> = None;
        let mut move_count = 0u32; // commanded-point moves (staircase proof)
        let mut max_rpm: f64 = 0.0;
        let mut tail_abs_err = Vec::new();
        // Mirror of the controller's cooldown ring (post-step command, one
        // point per sample, same 40 s retention), for the mid-cycle guard
        // below: it must use the real `cooldown_open` semantics, because a
        // per-step move threshold is evaded by exactly the +2 W/5 s
        // staircase — the incident's own lesson (cooldown.rs module docs).
        let mut mirror: Vec<CommandedPoint> = Vec::new();
        let (mut prev_trim, mut prev_gain) = (0.0_f64, 1.0_f64);
        for t in 0..900u32 {
            // Fans answer the COMMANDED point DEAD seconds late.
            let cmd = (
                ctl.status().cpu_limit_w.unwrap_or(0.0),
                ctl.auto
                    .as_ref()
                    .and_then(|a| a.gpu_target_w)
                    .unwrap_or(0.0),
            );
            let moved = last_cmd
                .is_none_or(|(pc, pg)| (cmd.0 - pc).abs() > 0.1 || (cmd.1 - pg).abs() > 0.1);
            if moved {
                last_cmd = Some(cmd);
                move_count += 1;
            }
            pipe.push_back(cmd);
            let (fpc, fpg) = pipe.pop_front().unwrap();
            rpm += (m.predict(fpc, fpg) + PLANT_BIAS - rpm) / TAU;
            max_rpm = max_rpm.max(rpm);

            // Achieved sample: draws track the commands, GPU-busy load.
            let s = Sample {
                t_mono: f64::from(t),
                cpu_util_pct: 100.0,
                gpu_util_pct: 100.0,
                cpu_pkg_w: cmd.0,
                gpu_w: cmd.1,
                gpu_w_valid: true,
                fan1_rpm: rpm,
                fan_valid: true,
                cpu_temp_c: 60.0,
                cpu_temp_valid: true,
                ..Sample::default()
            };
            ctl.on_sample(&s);
            assert!(
                !ctl.auto.as_ref().unwrap().distrusted,
                "false ModelDistrust at t={t} (the incident's failure #4)"
            );
            // The incident's exact failure mode stays closed even though
            // adaptation now resumes: every trim/gain move must coincide
            // with a commanded point at REST per the real gate semantics —
            // learning at genuine dwells only, never mid-staircase.
            if let (Some(pc), Some(pg)) = (
                ctl.status().cpu_limit_w,
                ctl.auto.as_ref().and_then(|a| a.gpu_target_w),
            ) {
                if mirror.len() >= COMMANDED_RING_CAP {
                    mirror.remove(0);
                }
                mirror.push(CommandedPoint {
                    t_mono: f64::from(t),
                    cpu_w: pc,
                    gpu_w: pg,
                });
            }
            if ctl.auto.as_ref().unwrap().kf.bias() != prev_trim
                || ctl.auto.as_ref().unwrap().kf.gain() != prev_gain
            {
                assert!(
                    cooldown::cooldown_open(&mirror, f64::from(t)),
                    "adaptation consumed a mid-cycle sample at t={t} \
                     (commanded point not at rest for the full window)"
                );
            }
            prev_trim = ctl.auto.as_ref().unwrap().kf.bias();
            prev_gain = ctl.auto.as_ref().unwrap().kf.gain();
            if t >= 700 {
                tail_abs_err.push((rpm - TARGET).abs());
            }
        }

        // A real staircase happened (the incident shape, not a trivial
        // hold): the onset climb alone moves the point ~25 times.
        assert!(move_count > 20, "no staircase: {move_count} moves");
        // The watch-item is RESOLVED, not merely quiet: the raise gate let
        // the loop rest, so the cooldown opened and the adaptation tier
        // CONSUMED samples (trust EWMA now > 0) and learned the in-authority
        // under-prediction (positive trim shifts the contour down toward
        // target). The pre-fix relay left both at exactly 0.
        assert!(
            ctl.auto.as_ref().unwrap().trust.ewma() > 0.0,
            "adaptation must resume once the raise gate lets the loop rest"
        );
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias() > 0.0,
            "the KF must learn the under-prediction: trim {}",
            ctl.auto.as_ref().unwrap().kf.bias()
        );
        // Gain stays inside the KF's tight prior — bias, not gain, carries a
        // pure additive offset; this only guards against a runaway.
        assert!(
            (0.5..=2.0).contains(&ctl.auto.as_ref().unwrap().kf.gain()),
            "gain ran away: {}",
            ctl.auto.as_ref().unwrap().kf.gain()
        );
        // And the learning CONVERGES the loop onto target — the residual is
        // now far inside the deadband (≈16 RPM measured), not the field's
        // relay — while overshoot stays a fraction of the incident's +810.
        let mean_err = tail_abs_err.iter().sum::<f64>() / tail_abs_err.len() as f64;
        assert!(
            mean_err < 150.0,
            "loop did not converge in band (mean |err| last 200 s = {mean_err:.0} RPM)"
        );
        assert!(
            max_rpm < TARGET + 400.0,
            "overshoot {:.0} RPM rivals the incident's +810",
            max_rpm - TARGET
        );
    }

    #[test]
    fn non_steady_window_freezes_adaptation() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Fan bouncing 1500/1700: spread 200 > the 100 RPM tolerance, so
        // the window is never steady — the KF must never adapt on a
        // transient, no matter how long it lasts.
        for t in 0..=45 {
            let rpm = if t % 2 == 0 { 1500.0 } else { 1700.0 };
            ctl.on_sample(&busy_fan_at(f64::from(t), rpm));
        }
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), 0.0);
        assert_eq!(ctl.auto.as_ref().unwrap().kf.gain(), 1.0);
    }

    #[test]
    fn fan_invalid_freezes_adaptation_and_restarts_the_settling_clock() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Pinned fixture with a +100 RPM plant offset: 18 valid steady
        // samples, then an 8 s fan outage, then valid again. The outage
        // lands as NaN in the window, so the 20-sample steady tail restarts
        // — no update may span the gap (a lost sensor must freeze the KF
        // exactly like it freezes the allocator). The cooldown gate opens
        // at t=45 (the fixture's entry drain parks at t=15; see
        // pinned_op_controller), so the outage-restarted settling clock is
        // exactly what gates the t=30..44 stretch here.
        for t in 0..18 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            ctl.on_sample(&s);
        }
        for t in 18..26 {
            let s = Sample {
                fan_valid: false,
                ..achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0)
            };
            ctl.on_sample(&s);
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                0.0,
                "bias moved on invalid fan"
            );
        }
        for t in 26..45 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            ctl.on_sample(&s);
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                0.0,
                "tail spans the outage at t={t}"
            );
        }
        // 20 clean samples after the outage (t=26..=45): adapts again.
        let s = achieved_fan_at(&ctl, 45.0, PINNED_PREDICT_RPM + 100.0);
        ctl.on_sample(&s);
        assert_ne!(ctl.auto.as_ref().unwrap().kf.bias(), 0.0);
    }

    #[test]
    fn adaptation_isolated_while_drain_veto_holds() {
        // 2026-07-14 design §3: a vetoed crest satisfies the cooldown gate
        // BY CONSTRUCTION (the command rests through the hold) and a crest
        // plateau passes is_steady (constant reading here) and achievement
        // (draws mirror commands) — every pre-veto gate admits it. Without
        // the isolation clause the KF would ingest +300 RPM of EC transient
        // as model error and the trust EWMA would grade the model against
        // it. The whole tier must stay silent instead.
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-veto-isolation",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Settle at the pinned point with zero innovation until well past
        // cooldown-open (t=45): premise state is exactly [0, 1].
        for t in 0..=50 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM);
            let effects = ctl.on_sample(&s);
            apply_effects(&effects, &ctl, f64::from(t), &ui_tx, &telemetry);
        }
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias().abs() < 1e-9,
            "premise: zero innovation, bias = {}",
            ctl.auto.as_ref().unwrap().kf.bias()
        );

        // EC-style crest: +300 over target, constant. The allocator vetoes
        // the drain (the contour covers the pinned point) — 45 s stays
        // inside the 60 s veto window — and the adaptation tier must not
        // move state nor feed trust, no matter how steady the plateau is.
        for t in 51..=95 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 300.0);
            let effects = ctl.on_sample(&s);
            apply_effects(&effects, &ctl, f64::from(t), &ui_tx, &telemetry);
            assert!(
                ctl.auto.as_ref().unwrap().kf.bias().abs() < 1e-9,
                "KF ingested a vetoed crest at t={t}: bias = {}",
                ctl.auto.as_ref().unwrap().kf.bias()
            );
            assert!(
                (ctl.auto.as_ref().unwrap().kf.gain() - 1.0).abs() < 1e-9,
                "gain moved on a vetoed crest at t={t}"
            );
        }
        assert!(
            ctl.auto.as_ref().unwrap().trust.ewma() < 1e-9,
            "trust must not be fed during a vetoed crest: ewma = {}",
            ctl.auto.as_ref().unwrap().trust.ewma()
        );

        // Crest decays: back in band (+80). The veto clears on band
        // re-entry, the command never moved, and after the 20-sample steady
        // tail refills the KF resumes — the +80 is real model error again.
        for t in 96..=140 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 80.0);
            let effects = ctl.on_sample(&s);
            apply_effects(&effects, &ctl, f64::from(t), &ui_tx, &telemetry);
        }
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias() > 1.0,
            "adaptation must resume once the crest clears: bias = {}",
            ctl.auto.as_ref().unwrap().kf.bias()
        );

        // Telemetry grading hook (design §3): the vetoed steps surface as
        // `auto:overshoot_settle` decisions, and none appear after recovery.
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let t = guard.as_mut().unwrap();
            t.flush();
            t.path().to_path_buf()
        };
        let contents = fs::read_to_string(&path).unwrap();
        let settle_ts: Vec<f64> = contents
            .lines()
            .map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap())
            .filter(|v| v["kind"] == "decision" && v["cause"] == "auto:overshoot_settle")
            .map(|v| v["t_mono"].as_f64().unwrap())
            .collect();
        assert!(
            !settle_ts.is_empty(),
            "vetoed crest must surface as auto:overshoot_settle in {contents}"
        );
        assert!(
            settle_ts.iter().all(|t| (51.0..=95.0).contains(t)),
            "settle causes outside the crest window: {settle_ts:?}"
        );
        fs::remove_dir_all(&dir).unwrap();
    }

    /// Calibrated controller pinned at a CONSTANT operating point: a 54 W
    /// CPU floor plus a target equal to the model's prediction there park
    /// the allocator at (54 W, 0 W) — the entry allocation drains the
    /// conservative 30 W GPU start to 0 at the 8 W/5 s down rate, parking
    /// at t=15, so the cooldown gate (whole trailing 30 s within ±2 W of
    /// the current point) opens at t=45. With the GPU leg at 0 W the KF
    /// regressor `w = b·pg + e·pc·pg` is 0: the gain state is inert (no
    /// update can move it or clamp-reject on it), so these scenarios
    /// exercise PURE bias dynamics — the old trim's role, carried 1:1.
    /// With target == prediction, feeding `PINNED_PREDICT_RPM + x` drives
    /// the KF innovation with exactly `x − bias` and the trust monitor
    /// with the same residual.
    fn pinned_op_controller(
        runner: &FakeRunner,
    ) -> (Controller<&FakeRunner>, Arc<Mutex<Vec<GpuCall>>>) {
        auto_controller(
            runner,
            PathBuf::from("/nonexistent/platform_profile"),
            Config {
                cpu_floor_w: 54.0,
                fan_target_rpm: PINNED_PREDICT_RPM,
                ..Config::default()
            },
        )
    }

    /// The calibrated model's prediction at the pinned (54, 0) point:
    /// 25·54 + 800 — and the pinned fixture's fan TARGET. Feeding this as
    /// the fan reading makes the KF innovation exactly zero (a gated
    /// zero-error update is still ACCEPTED and consumes the cadence slot,
    /// without moving the state).
    const PINNED_PREDICT_RPM: f64 = 2150.0;

    /// [`pinned_op_controller`] with a caller-chosen persisted state and a
    /// REAL state-file path: the Task-6 persistence tests read back what an
    /// Auto exit wrote.
    fn pinned_op_controller_with_state(
        runner: &FakeRunner,
        persisted: PersistedState,
        state_path: PathBuf,
    ) -> (Controller<&FakeRunner>, Arc<Mutex<Vec<GpuCall>>>) {
        let gpu = FakeGpu::new();
        let gpu_calls = gpu.calls();
        let ctl = Controller::new(
            RestoreGuard::new(
                runner,
                Some(cpu_actuator(
                    runner,
                    PathBuf::from("/nonexistent/platform_profile"),
                )),
                Some(Box::new(gpu)),
                None,
            ),
            persisted,
            state_path,
            Config {
                cpu_floor_w: 54.0,
                fan_target_rpm: PINNED_PREDICT_RPM,
                ..Config::default()
            },
            PathBuf::from("/nonexistent/config.toml"),
        );
        (ctl, gpu_calls)
    }

    /// Drive the pinned fixture through one learning stretch: entry drain
    /// parks at t=15, the cooldown gate opens at t=45, and a +100 RPM plant
    /// offset moves the bias positive at the first gated update.
    fn learn_some_bias(ctl: &mut Controller<&FakeRunner>) -> (f64, f64) {
        for t in 0..=60 {
            let s = achieved_fan_at(ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            ctl.on_sample(&s);
        }
        (
            ctl.auto.as_ref().unwrap().kf.bias(),
            ctl.auto.as_ref().unwrap().kf.gain(),
        )
    }

    #[test]
    fn auto_exit_persists_kalman_state_to_disk() {
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-kf-persist",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let state_path = dir.join("state.json");
        let runner = FakeRunner::new();
        let (mut ctl, _g) =
            pinned_op_controller_with_state(&runner, calibrated(), state_path.clone());
        ctl.on_command(Command::SetAuto(true));
        let (learned, gain) = learn_some_bias(&mut ctl);
        assert!(learned > 0.0, "premise: the KF learned a bias");

        // Auto exit writes [bias, gain] to the state file, next to the
        // model + LUT it must NOT clobber (design §2). float_roundtrip is
        // on, so the readback is bit-exact.
        ctl.on_command(Command::SetAuto(false));
        let saved = PersistedState::load(&state_path);
        assert_eq!(saved.adapt_bias, learned);
        assert_eq!(saved.adapt_gain, gain);
        assert!(saved.model.is_some(), "model clobbered by the adapt write");
        assert!(saved.lut.is_some(), "lut clobbered by the adapt write");

        // A NEXT session seeded from that file starts Auto centered on the
        // learned correction — visible in status from entry, live in the
        // filter — and keeps learning FROM it (the one-time-cost rationale:
        // gain/bias are not relearned from scratch every session).
        let runner2 = FakeRunner::new();
        let (mut ctl2, _g2) = pinned_op_controller_with_state(&runner2, saved, state_path.clone());
        ctl2.on_command(Command::SetAuto(true));
        assert_eq!(ctl2.auto.as_ref().unwrap().kf.bias(), learned);
        assert_eq!(ctl2.auto.as_ref().unwrap().kf.gain(), gain);
        let (learned2, _) = learn_some_bias(&mut ctl2);
        assert!(
            learned2 > learned,
            "second session must build on the seed: {learned2} vs {learned}"
        );

        // Clean quit from Auto persists too (design §2: "written on Auto
        // exit and clean quit").
        ctl2.on_command(Command::Quit);
        let saved2 = PersistedState::load(&state_path);
        assert_eq!(saved2.adapt_bias, learned2);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn seeded_state_is_live_from_auto_entry() {
        // The persisted seed must be IN FORCE from the first second of a
        // session — the whole point of persistence is that the second
        // onset (next session) lands near target from the start.
        let runner = FakeRunner::new();
        let persisted = PersistedState {
            adapt_bias: -120.0,
            adapt_gain: 1.1,
            ..calibrated()
        };
        let (mut ctl, _g) = pinned_op_controller_with_state(
            &runner,
            persisted,
            PathBuf::from("/nonexistent/state.json"),
        );
        ctl.on_command(Command::SetAuto(true));
        // Mirrored into status at entry, before any sample…
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), -120.0);
        assert_eq!(ctl.auto.as_ref().unwrap().kf.gain(), 1.1);
        // …and live in the filter the allocator's contour reads.
        let auto = ctl.auto.as_ref().unwrap();
        assert_eq!(auto.kf.bias(), -120.0);
        assert_eq!(auto.kf.gain(), 1.1);
        // A sample does not reset it (no adaptation yet: gate closed).
        let s = achieved_fan_at(&ctl, 0.0, PINNED_PREDICT_RPM);
        ctl.on_sample(&s);
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), -120.0);
        assert_eq!(ctl.auto.as_ref().unwrap().kf.gain(), 1.1);
    }

    #[test]
    fn new_calibration_resets_persisted_adaptation() {
        // A fresh surface invalidates old corrections (design §2): when a
        // calibration lands its SaveState, the in-memory seed must reset to
        // the identity alongside the identity the state file just got.
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-kf-calib-reset",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let state_path = dir.join("state.json");
        let runner = FakeRunner::new();
        let stale = PersistedState {
            adapt_bias: 300.0,
            adapt_gain: 1.4,
            ..calibrated()
        };
        let (mut ctl, _g) = pinned_op_controller_with_state(&runner, stale, state_path.clone());
        ctl.apply_calib_effects(vec![RunnerEffect::SaveState(Box::new(PersistedState {
            model: Some(fitted_model()),
            lut: Some(lut3()),
            calibrated_at: Some("1783650000".to_string()),
            ..PersistedState::default()
        }))]);
        // The next Auto entry seeds from the identity, not the stale pair.
        ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), 0.0);
        assert_eq!(ctl.auto.as_ref().unwrap().kf.gain(), 1.0);
        // And an Auto exit re-writes identity + the new calibration stamp
        // (a stale-seed leak here would poison every later session).
        ctl.on_command(Command::SetAuto(false));
        let saved = PersistedState::load(&state_path);
        assert_eq!(saved.adapt_bias, 0.0);
        assert_eq!(saved.adapt_gain, 1.0);
        assert_eq!(saved.calibrated_at.as_deref(), Some("1783650000"));
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn persistent_residual_at_frozen_point_saturates_bias_and_flags_unreachable() {
        // At the pinned point the KF innovation is `offset − bias`
        // (negative feedback), so only a plant offset BEYOND the ±400
        // authority can pin the bias: it saturates at the clamp (never
        // rejecting — the bias clamp is the saturating one) and surfaces
        // TargetUnreachable, the trim's exact terminal state.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Clean baseline: fan matches the corrected prediction exactly, so
        // the gated zero-innovation updates (from t=45, when the cooldown
        // opens) consume cadence slots without moving the bias.
        for t in 0..40 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM);
            ctl.on_sample(&s);
        }
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), 0.0);

        // Blocked intake: fans steady +500 RPM over the baseline
        // prediction while the 54 W floor leaves the allocator nothing to
        // cut (the contour is already zero-clamped) — the offset exceeds
        // the bias authority, so pinning + flagging is the CORRECT
        // terminal state. ModelDistrust must stay clear: the residual the
        // trust monitor sees is `500 − bias`, which the KF itself drives
        // under the 300 RPM threshold within ~10 updates — faster than the
        // 300 s distrust sustain (adaptation absorbing an in-reach part of
        // the error is exactly what keeps the verdict honest; contrast
        // drive_to_distrust, whose +750 offset leaves ≥ 350 RPM behind).
        for t in 40..=750 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 500.0);
            ctl.on_sample(&s);
        }
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), MAX_BIAS_AUTHORITY_RPM);
        assert!(
            ctl.status().flags.contains(&StatusFlag::TargetUnreachable),
            "saturated +max must surface TargetUnreachable"
        );
        assert!(
            !ctl.auto.as_ref().unwrap().distrusted,
            "the shrinking residual must not distrust the model"
        );
        // The KF corrects OUTSIDE the surface: a/b/e/c stay bit-identical
        // to the calibrated fit (nothing in Auto mutates the model now).
        let m = ctl.model.as_ref().unwrap();
        let calib = fitted_model();
        assert_eq!((m.a, m.b, m.e, m.c), (calib.a, calib.b, calib.e, calib.c));

        // Recovery (blanket removed): the offset falls back inside the
        // authority (+300), the innovation flips negative and the bias
        // walks off the clamp; the flag clears below 90% of max (360).
        for t in 751..=1000 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 300.0);
            ctl.on_sample(&s);
        }
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias() < TRIM_CLEAR_FRACTION * MAX_BIAS_AUTHORITY_RPM,
            "bias = {}",
            ctl.auto.as_ref().unwrap().kf.bias()
        );
        assert!(
            !ctl.status().flags.contains(&StatusFlag::TargetUnreachable),
            "flag must clear below 90% of max"
        );
        assert!(
            !ctl.auto.as_ref().unwrap().distrusted,
            "recovery must not fire distrust either"
        );
    }

    #[test]
    fn auto_exit_resets_gates_but_carries_the_learned_correction() {
        // The design-§3 lifecycle: AutoState drops whole on exit (fresh
        // covariance, fresh steadiness window, fresh cooldown ring), but
        // the learned [bias, gain] is CAPTURED into the persisted seed and
        // re-entry starts centered on it — within a session exactly like
        // across sessions (Task 6; the pre-persistence behavior reset the
        // filter to identity on every exit).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));
        // +100 RPM plant offset: the first gated update (t=45, when the
        // cooldown opens over the parked point) moves the bias.
        for t in 0..=45 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            ctl.on_sample(&s);
        }
        let learned = ctl.auto.as_ref().unwrap().kf.bias();
        assert_ne!(learned, 0.0, "premise: bias accumulated");

        // SetAuto(false): AutoState (and its KF) drops whole while OUT of
        // Auto (nothing is being corrected in Monitor)…
        ctl.on_command(Command::SetAuto(false));
        assert!(ctl.auto.is_none());

        // …but re-entry seeds from the captured pair, live immediately.
        // The GATES are still fresh: a fresh steadiness window AND a fresh
        // cooldown ring — the re-entry allocation re-drains to the pinned
        // point (parks t=115), so no further adaptation before t=145.
        ctl.on_command(Command::SetAuto(true));
        assert_eq!(
            ctl.auto.as_ref().unwrap().kf.bias(),
            learned,
            "seed must carry over"
        );
        for t in 100..145 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            ctl.on_sample(&s);
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                learned,
                "gates must hold the seeded state frozen after re-entry at t={t}"
            );
        }
        // First gated update BUILDS on the seed (negative feedback toward
        // the +100 plant offset), rather than restarting from zero.
        let s = achieved_fan_at(&ctl, 145.0, PINNED_PREDICT_RPM + 100.0);
        ctl.on_sample(&s);
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias() > learned,
            "update must build on the seed: {} vs {learned}",
            ctl.auto.as_ref().unwrap().kf.bias()
        );

        // ReleaseAll is the other Auto exit: same drop-whole + capture.
        ctl.on_command(Command::ReleaseAll);
        assert!(ctl.auto.is_none());
        assert_eq!(ctl.status().mode, Mode::Monitor);
        ctl.on_command(Command::SetAuto(true));
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias() > learned,
            "ReleaseAll must capture too"
        );
    }

    #[test]
    fn fan_target_change_keeps_kf_state() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=45 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            ctl.on_sample(&s);
        }
        let trim = ctl.auto.as_ref().unwrap().kf.bias();
        assert_ne!(trim, 0.0, "premise: bias accumulated");

        // Retargeting the fan goal does NOT reset the filter: the model's
        // offset error (what the bias measures) didn't change with the
        // user's target.
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), trim);
        assert_eq!(ctl.status().mode, Mode::Auto);
    }

    #[test]
    fn kf_decisions_reach_telemetry_with_offset() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-kf-telemetry",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Clean hold until t=26, then a +150 step: the steadiness clock
        // restarts and the first gated update lands at t=46 — deliberately
        // OFF the 5 s allocator grid, so its "auto:kf" Decision cause is
        // not shadowed by that sample's "auto:allocate" (first claim wins).
        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));
        for t in 0..=50 {
            let fan = if t < 27 {
                PINNED_PREDICT_RPM
            } else {
                PINNED_PREDICT_RPM + 150.0
            };
            let s = achieved_fan_at(&ctl, f64::from(t), fan);
            let effects = ctl.on_sample(&s);
            apply_effects(&effects, &ctl, f64::from(t), &ui_tx, &telemetry);
        }
        // Auto exit: a non-Auto decision afterwards must skip trim_rpm.
        let effects = ctl.on_command(Command::SetAuto(false));
        apply_effects(&effects, &ctl, 51.0, &ui_tx, &telemetry);
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

        let kf_line = decisions
            .iter()
            .find(|d| d["cause"] == "auto:kf")
            .unwrap_or_else(|| panic!("no auto:kf decision in {contents}"));
        assert_eq!(kf_line["t_mono"], 46.0, "first gated update off the grid");
        let offset = kf_line["trim_rpm"].as_f64().expect("trim_rpm present");
        // First-step Kalman gain ≈ 0.05 on a +150 innovation: positive,
        // well under one full step (the kalman.rs suite owns the exact
        // constants; here only the wiring and the sign are under test).
        assert!(offset > 0.0 && offset < 150.0, "trim_rpm = {offset}");
        // Both learned states ride every Auto decision (Task 7): at the
        // pinned point w = 0, so the gain must still read exactly 1.0.
        assert_eq!(kf_line["gain"], 1.0, "gain missing/moved: {kf_line}");

        // The allocate line right after carries the current bias too.
        let alloc_line = decisions
            .iter()
            .find(|d| d["cause"] == "auto:allocate" && d["t_mono"] == 50.0)
            .unwrap_or_else(|| panic!("no t=50 allocate decision in {contents}"));
        assert_eq!(alloc_line["trim_rpm"], kf_line["trim_rpm"]);
        assert_eq!(alloc_line["gain"], 1.0);

        // Non-Auto decisions stay lean: no trim_rpm/gain keys.
        let off_line = decisions
            .iter()
            .find(|d| d["cause"] == "auto:off")
            .unwrap_or_else(|| panic!("no auto:off decision in {contents}"));
        assert!(
            off_line.get("trim_rpm").is_none(),
            "trim_rpm must be skipped outside Auto: {off_line}"
        );
        assert!(
            off_line.get("gain").is_none(),
            "gain must be skipped outside Auto: {off_line}"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn positive_bias_cuts_the_commanded_gpu_allocation_end_to_end() {
        // The decisive sign check for the bias→contour wiring: a POSITIVE
        // bias must yield FEWER commanded GPU watts than the uncorrected
        // contour would (under-prediction → smaller budget). Kills both
        // sign mutants in the allocator's contour closure: `bias → 0.0`
        // (allocation lands on the uncorrected contour) and `bias → -bias`
        // (positive feedback: MORE budget, above the uncorrected contour).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Build bias at the pinned (54, 0) point: clean baseline, then a
        // +500 step. The step restarts the steadiness clock, so updates
        // land at t = 65, 85, …, 145 (the t=45 baseline update consumed
        // the cadence slot) — six ~25 RPM steps, bias ≈ 113 by t=160.
        for t in 0..40 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM);
            ctl.on_sample(&s);
        }
        for t in 40..=160 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 500.0);
            ctl.on_sample(&s);
        }
        let trim = ctl.auto.as_ref().unwrap().kf.bias();
        assert!(trim > 50.0, "premise: bias accumulated, got {trim}");
        assert_eq!(
            ctl.auto.as_ref().unwrap().kf.gain(),
            1.0,
            "w = 0 at the pinned point: gain inert"
        );

        // Now raise the target to 4000 RPM so the contour at pc=54 is
        // INTERIOR (neither zero-clamped nor at GPU_MAX), and feed a fan
        // sawtooth (2400/2650, spread 250 > the 100 RPM steadiness
        // tolerance, always valid): the whole adaptation tier freezes (no
        // steady window → no KF movement, no trust evidence — the pre-flip
        // trust EWMA stands frozen without ever reaching a verdict) while
        // the allocator walks the GPU allocation up toward the CORRECTED
        // contour — isolating exactly the bias term.
        ctl.on_command(Command::SetFanTarget(4000.0));
        let mut last_gpu = None;
        for t in 161..=600 {
            let fan = if t % 2 == 0 { 2400.0 } else { 2650.0 };
            if let Some((_, gpu_w)) = alloc_of(&ctl.on_sample(&busy_fan_at(f64::from(t), fan))) {
                last_gpu = Some(gpu_w);
            }
        }
        let gpu_w = last_gpu.expect("allocator ran");
        assert_eq!(
            ctl.auto.as_ref().unwrap().kf.bias(),
            trim,
            "bias frozen while unsteady"
        );

        // The model is still calibrated, so both contours are exact:
        // uncorrected (4000 − 800 − 25·54)/20.4 ≈ 90.7 W; the corrected
        // one sits bias/20.4 ≈ 5.5 W lower.
        let m = ctl.model.as_ref().unwrap();
        let uncorrected = m.gpu_watts_on_contour(4000.0, 0.0, 1.0, 54.0).unwrap();
        let corrected = m.gpu_watts_on_contour(4000.0, trim, 1.0, 54.0).unwrap();
        assert!(
            (uncorrected - 90.7).abs() < 0.1,
            "uncorrected = {uncorrected}"
        );
        assert!(
            (gpu_w - corrected).abs() < 2.5,
            "allocation must settle on the CORRECTED contour: {gpu_w} vs {corrected}"
        );
        assert!(
            gpu_w < uncorrected - 2.0,
            "positive bias must CUT the allocation below the uncorrected \
             contour: {gpu_w} vs {uncorrected}"
        );
    }

    #[test]
    fn kf_absorbs_model_bias_and_lands_fans_on_target() {
        // End-to-end replay of the 2026-07 field incidents on the v2 tier.
        // The plant answers every operating point 139 RPM louder than the
        // calibrated surface predicts (a +139 RPM model bias). Phase 1 is
        // the 2026-07-09 incident's wind-up: the allocator staircases
        // toward the contour — the cooldown gate must keep the KF at
        // identity the whole climb (the old trim integrated the wind-up's
        // below-target control error and pinned at −400 within 3 minutes).
        // Once the commanded point parks, the KF absorbs the offset in
        // punctuated cooldown cycles and the fans land ON target with no
        // flags (the original field session parked them 261 RPM BELOW
        // target with ~15 W of budget withheld).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let target = ctl.status().fan_target_rpm; // default 3000
        let m = fitted_model();

        let mut fan = f64::NAN;
        for t in 0..3000 {
            // The plant reads the TRUE surface (calibrated + 139) at the
            // point the controller currently holds; draws achieved.
            let pc = ctl.status().cpu_limit_w.unwrap_or(15.0);
            let pg = ctl
                .auto
                .as_ref()
                .and_then(|a| a.gpu_target_w)
                .unwrap_or(30.0);
            fan = m.predict(pc, pg) + 139.0;
            let s = achieved_fan_at(&ctl, f64::from(t), fan);
            ctl.on_sample(&s);
            if t < 80 {
                assert_eq!(
                    ctl.auto.as_ref().unwrap().kf.bias(),
                    0.0,
                    "wind-up adapted at t={t}"
                );
                assert_eq!(
                    ctl.auto.as_ref().unwrap().kf.gain(),
                    1.0,
                    "wind-up adapted at t={t}"
                );
            }
        }
        // Fans end inside the allocator's ±150 RPM band around the target.
        assert!(
            (fan - target).abs() < 150.0,
            "fans must land on target: {fan} vs {target}"
        );
        // How the correction splits between bias and gain is covariance-
        // weighted (a held operating point cannot fully separate them —
        // kalman.rs owns those dynamics); here the correction must be
        // engaged, in authority, and flag-free.
        let (trim, gain) = (
            ctl.auto.as_ref().unwrap().kf.bias(),
            ctl.auto.as_ref().unwrap().kf.gain(),
        );
        assert!(trim.abs() < MAX_BIAS_AUTHORITY_RPM, "bias pinned: {trim}");
        assert!(trim != 0.0 || gain != 1.0, "KF never engaged");
        assert!(
            !ctl.status().flags.contains(&StatusFlag::TargetUnreachable),
            "an in-authority offset must not flag TargetUnreachable"
        );
        assert!(
            !ctl.auto.as_ref().unwrap().distrusted,
            "the wind-up must not fire ModelDistrust (incident step 4)"
        );
    }

    #[test]
    fn on_target_fans_with_wrong_model_fire_distrust_and_pin_the_bias_low() {
        use crate::control::trust::DISTRUST_RPM;
        // Fans exactly ON target at a floor-pinned point the model badly
        // over-predicts (predicts 2150 at (54, 0), fans read 1400). The KF
        // — unlike the old control-error trim, which saw zero error here
        // and held — grades the MODEL: it walks the bias negative toward
        // the real surface and saturates at the −400 authority with ~350
        // RPM of residual left. That leftover keeps the trust EWMA over
        // the 300 RPM threshold, so ModelDistrust fires (its semantics are
        // unchanged: model persistently wrong beyond what adaptation can
        // absorb = "recalibrate when convenient") and freezes the filter
        // at the clamp. TargetUnreachable keys off the +max pin only: fans
        // ON target must never read as an unreachable target.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            Config {
                cpu_floor_w: 54.0,
                fan_target_rpm: 1400.0,
                ..Config::default()
            },
        );
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=800 {
            let s = achieved_fan_at(&ctl, f64::from(t), 1400.0);
            ctl.on_sample(&s);
        }
        assert!(
            ctl.auto.as_ref().unwrap().distrusted,
            "trust must keep watching the model residual"
        );
        assert!(ctl.auto.as_ref().unwrap().trust.ewma() > DISTRUST_RPM);
        assert_eq!(
            ctl.auto.as_ref().unwrap().kf.bias(),
            -MAX_BIAS_AUTHORITY_RPM,
            "the KF hands back what it can, bounded at −max"
        );
        assert!(
            !ctl.status().flags.contains(&StatusFlag::TargetUnreachable),
            "on-target fans must never read as an unreachable target"
        );
    }

    #[test]
    fn unachieved_budget_freezes_adaptation_and_trust() {
        // Field capture #3 (2026-07): a game entered a light / fps-capped
        // state. The allocator kept raising the budget (fans honestly quiet
        // under the 3250 target) but the load could not SPEND it — driver
        // DVFS held ~1670 MHz and only 49.7 W of an 80 W GPU budget were
        // drawn (a max clock is not a floor), RAPL read 20 W under a
        // walking CPU limit. Grading adaptation at that COMMANDED-but-
        // untested point wound the old trim to the −400 pin and fed the
        // trust monitor a ~1450 RPM phantom residual → ModelDistrust — so
        // when the game resumed drawing, the loop regulated fans to
        // target+400 for minutes at reduced unwind gain. With the
        // achievement gate, an under-consumed budget teaches nothing: the
        // KF and trust must FREEZE, exactly. (The cooldown gate also stays
        // closed during the budget walk; once the allocation parks at its
        // ceiling, achievement is the gate that owns this scenario.)
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            Config {
                fan_target_rpm: 3250.0,
                ..Config::default()
            },
        );
        ctl.on_command(Command::SetAuto(true));

        for t in 0..=600 {
            // Fans spin down from the previous hot state (unsteady ramp,
            // ~10 RPM/s) and settle at 2150; the window turns steady at
            // t≈44, by which time the walking CPU limit has already
            // outrun the 20 W draw by more than the 3 W margin — the
            // point is never achieved while eligible.
            let fan = (2400.0 - 10.0 * f64::from(t)).max(2150.0);
            let s = Sample {
                cpu_pkg_w: 20.0,
                gpu_w: 49.7,
                ..busy_fan_at(f64::from(t), fan)
            };
            ctl.on_sample(&s);
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                0.0,
                "bias moved at t={t}"
            );
        }
        // Premise: the budget genuinely outran the draw on the CPU leg.
        let limit = ctl.status().cpu_limit_w.unwrap();
        assert!(limit > 23.0 + 1e-9, "premise: limit walked, got {limit}");
        // Nothing was learned: no KF movement, no trust evidence, no flags.
        assert_eq!(ctl.auto.as_ref().unwrap().kf.bias(), 0.0);
        assert_eq!(ctl.auto.as_ref().unwrap().kf.gain(), 1.0);
        assert_eq!(ctl.auto.as_ref().unwrap().trust.ewma(), 0.0);
        assert!(!ctl.auto.as_ref().unwrap().distrusted);
        assert!(!ctl.status().flags.contains(&StatusFlag::TargetUnreachable));
    }

    #[test]
    fn achievement_gate_margin_boundaries() {
        // KF movement after a parked, cooldown-open, steady hold whose
        // draws are offset from the commanded point by (dcpu, dgpu):
        // inside the margins the first gated sample moves the state,
        // outside them the tier stays frozen at identity. A 54 W CPU floor
        // collapses the allocator's grid to pc=54, so the candidate — and
        // with it the commanded point — cannot move when the draw offsets
        // skew the demand estimate: the allocation walks the GPU leg to
        // the 3000 RPM contour (~41.7 W) by t=30 and parks; the cooldown
        // opens at t=55 (30 s past the last >2 W-off ring point), and the
        // fan flip at t=50 restarts the steadiness clock so the first
        // gated sample lands at t=69.
        let kf_moved = |dcpu: f64, dgpu: f64| -> bool {
            let runner = FakeRunner::new();
            let (mut ctl, _gpu_calls) = auto_controller(
                &runner,
                PathBuf::from("/nonexistent/platform_profile"),
                Config {
                    cpu_floor_w: 54.0,
                    ..Config::default()
                },
            );
            ctl.on_command(Command::SetAuto(true));
            let m = fitted_model();
            for t in 0..50 {
                let s = achieved_fan_at(&ctl, f64::from(t), 1700.0);
                ctl.on_sample(&s);
            }
            for t in 50..80 {
                let pc = ctl.status().cpu_limit_w.unwrap();
                let pg = ctl.auto.as_ref().unwrap().gpu_target_w.unwrap();
                let mut s = achieved_fan_at(&ctl, f64::from(t), m.predict(pc, pg) + 80.0);
                s.cpu_pkg_w += dcpu;
                s.gpu_w += dgpu;
                ctl.on_sample(&s);
            }
            ctl.auto.as_ref().unwrap().kf.bias() != 0.0
                || ctl.auto.as_ref().unwrap().kf.gain() != 1.0
        };
        // GPU margin: exactly target−5 is achieved, 1 W past is not.
        assert!(kf_moved(0.0, -5.0));
        assert!(!kf_moved(0.0, -6.0));
        // CPU margin: exactly limit−3 is achieved, past it is not.
        assert!(kf_moved(-3.0, 0.0));
        assert!(!kf_moved(-3.5, 0.0));
    }

    #[test]
    fn adaptation_resumes_when_the_budget_is_consumed_again() {
        // The recovery half of the field capture: a long unachieved stretch
        // must leave the filter untouched AND ready — once the load spends
        // its budget again, adaptation resumes (the KF cadence slot was
        // never consumed, so the first achieved gated sample updates
        // immediately).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        ctl.on_command(Command::SetAuto(true));

        // 100 unachieved samples: the CPU draw sags 3.5 W under its 54 W
        // limit — just past the 3 W margin (and close enough that the
        // observed-watts window mean stays near the commanded point when
        // the draw recovers), fans steady +100 over the prediction. The
        // cooldown gate is open from t=45 and the window steady from t=19,
        // so the achievement gate ALONE is what freezes here.
        for t in 0..100 {
            let mut s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            s.cpu_pkg_w = 50.5;
            ctl.on_sample(&s);
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                0.0,
                "bias moved at t={t}"
            );
        }
        // Draw resumes at the commanded point, fans still over: the first
        // achieved sample (t=100) updates immediately, the next at t=120.
        let mut at_100 = f64::NAN;
        for t in 100..125 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 100.0);
            ctl.on_sample(&s);
            if t == 100 {
                at_100 = ctl.auto.as_ref().unwrap().kf.bias();
            }
        }
        assert!(at_100 > 0.0, "first achieved sample must update: {at_100}");
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias() > at_100,
            "second update (t=120) must build on the first: {} vs {at_100}",
            ctl.auto.as_ref().unwrap().kf.bias()
        );
    }

    // --- Trust monitor (Task 27, on the v2 tier) ---

    /// Drive a pinned-op controller into ModelDistrust: clean baseline,
    /// then a +750 RPM plant offset the ±400 bias authority cannot absorb —
    /// the residual stays ≥ 350 RPM even once the bias pins, holding the
    /// trust EWMA over the 300 RPM threshold through the 300 s sustain.
    /// (An IN-authority offset can no longer distrust: the KF drives the
    /// residual under the threshold faster than the sustain — see
    /// persistent_residual_at_frozen_point_saturates_bias_and_flags_unreachable.)
    /// Returns the t_mono of the first sample where `auto.distrusted` turns
    /// true. `ModelDistrust` is removed from the Task 4 type surface (no
    /// `StatusFlag`, so no telemetry `Flagged` effect on this transition
    /// any more) — the trust verdict itself, read straight off `AutoState`,
    /// is what the adaptation tier's tests actually exercise.
    fn drive_to_distrust(ctl: &mut Controller<&FakeRunner>) -> u32 {
        ctl.on_command(Command::SetAuto(true));
        for t in 0..40 {
            let s = achieved_fan_at(ctl, f64::from(t), PINNED_PREDICT_RPM);
            ctl.on_sample(&s);
        }
        for t in 40..=900 {
            let s = achieved_fan_at(ctl, f64::from(t), PINNED_PREDICT_RPM + 750.0);
            ctl.on_sample(&s);
            if ctl.auto.as_ref().unwrap().distrusted {
                return t;
            }
        }
        panic!("ModelDistrust never tripped");
    }

    #[test]
    fn kf_freezes_entirely_while_distrusted() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        let flagged_at = drive_to_distrust(&mut ctl);
        // By flag time (EWMA crossing + 300 s sustain) the bias has long
        // pinned: it reaches +400 within ~16 updates of the offset onset,
        // well inside the sustain window.
        let (trim, gain) = (
            ctl.auto.as_ref().unwrap().kf.bias(),
            ctl.auto.as_ref().unwrap().kf.gain(),
        );
        assert_eq!(trim, MAX_BIAS_AUTHORITY_RPM);

        // Hold 200 more flat/steady/achieved samples of the same +750
        // offset. This phase alone cannot prove the freeze (review
        // mutation finding): at the +400 pin a POSITIVE innovation makes
        // an unfrozen update a saturating no-op anyway (and w = 0 keeps
        // the gain inert), so it only parks the trust EWMA at its ≈350
        // floor for the probe below.
        for t in flagged_at + 1..=flagged_at + 200 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + 750.0);
            ctl.on_sample(&s);
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                trim,
                "bias moved while distrusted"
            );
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.gain(),
                gain,
                "gain moved while distrusted"
            );
        }
        assert!(ctl.auto.as_ref().unwrap().distrusted);

        // The probe that BITES (design §1 "frozen ENTIRELY"; the old tier
        // kept integrating the trim at HALF gain here): fans 200 RPM
        // BELOW the corrected prediction — a NEGATIVE innovation, which an
        // unfrozen filter would visibly absorb by walking the bias OFF the
        // +400 clamp at its first cadence tick (a positive one would just
        // re-saturate). The |200| residual sits UNDER the 300 RPM distrust
        // threshold, so the EWMA decays from ≈350 toward 200 and would
        // eventually clear the flag — the probe stays short (20 unsteady
        // samples after the fan step + 15 gated ones, EWMA ≈ 311 at the
        // end) and asserts the flag is STILL up on every sample, so the
        // held state can only mean the freeze itself.
        let probe_start = flagged_at + 201;
        for t in probe_start..probe_start + 35 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + trim - 200.0);
            ctl.on_sample(&s);
            assert!(
                ctl.auto.as_ref().unwrap().distrusted,
                "probe outlived the distrust flag at t={t}; shorten it"
            );
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.bias(),
                trim,
                "bias absorbed a negative innovation while distrusted (t={t})"
            );
            assert_eq!(
                ctl.auto.as_ref().unwrap().kf.gain(),
                gain,
                "gain moved while distrusted"
            );
        }

        // Recovery: the plant falls back to what the CORRECTED model
        // expects (baseline + the pinned bias) — the residual goes to
        // zero, the EWMA decays under 300 and the flag clears.
        let mut t = probe_start + 35;
        let mut cleared_at = None;
        for _ in 0..200 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + trim);
            ctl.on_sample(&s);
            if !ctl.auto.as_ref().unwrap().distrusted {
                cleared_at = Some(t);
                break;
            }
            t += 1;
        }
        let cleared_at = cleared_at.expect("distrust never cleared");
        assert!(!ctl.auto.as_ref().unwrap().distrusted);

        // And the filter RESUMES: an offset 100 RPM under the corrected
        // prediction unwinds the pinned bias — the freeze was the verdict,
        // not a latch (the cadence slot was never consumed while frozen,
        // so the first trusted sample may update immediately).
        for t in cleared_at + 1..=cleared_at + 60 {
            let s = achieved_fan_at(&ctl, f64::from(t), PINNED_PREDICT_RPM + trim - 100.0);
            ctl.on_sample(&s);
        }
        assert!(
            ctl.auto.as_ref().unwrap().kf.bias() < trim,
            "bias must walk off the clamp once trusted again: {}",
            ctl.auto.as_ref().unwrap().kf.bias()
        );
    }

    #[test]
    fn auto_exit_resets_trust_and_distrust_flag() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = pinned_op_controller(&runner);
        drive_to_distrust(&mut ctl);
        assert!(ctl.auto.as_ref().unwrap().distrusted);

        // Auto exit: AutoState (trust monitor included) drops whole…
        ctl.on_command(Command::SetAuto(false));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ctl.auto.is_none());

        // …and re-entry starts with FRESH trust (AutoState dropped whole).
        ctl.on_command(Command::SetAuto(true));
        assert!(!ctl.auto.as_ref().unwrap().distrusted);
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
            let s = achieved_fan_at(&ctl, f64::from(t), 2800.0);
            let effects = ctl.on_sample(&s);
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
        // Every snapshot carries the CALIBRATED surface: the KF corrects
        // outside a/b/e/c, so nothing in Auto may move these parameters —
        // the snapshot line is the offline reviewer's proof of that.
        let calib = fitted_model();
        for (i, snap) in snapshots.iter().enumerate() {
            for (key, want) in [
                ("model_a", calib.a),
                ("model_b", calib.b),
                ("model_e", calib.e),
                ("model_c", calib.c),
            ] {
                let got = snap[key]
                    .as_f64()
                    .unwrap_or_else(|| panic!("{key} missing"));
                assert!(
                    (got - want).abs() < 1e-9,
                    "snapshot {i}: {key} = {got}, want the calibrated {want}"
                );
            }
        }

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
            ..PersistedState::default()
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

    // --- Task 28: thermal watchdog + emergency release ---

    /// Tctl over the 95 °C trip; every other sensor invalid/idle (the
    /// watchdog must trip on temperature alone).
    fn overheat_at(t: f64) -> Sample {
        Sample {
            t_mono: t,
            cpu_temp_c: 96.0,
            cpu_temp_valid: true,
            ..Sample::default()
        }
    }

    fn has_flagged(effects: &[Effect], want_flag: &str, want_active: bool) -> bool {
        effects.iter().any(|e| {
            matches!(e, Effect::Flagged { flag, active }
                if *flag == want_flag && *active == want_active)
        })
    }

    fn has_status_change_cause(effects: &[Effect], want: &str) -> bool {
        effects
            .iter()
            .any(|e| matches!(e, Effect::StatusChanged { cause } if *cause == want))
    }

    #[test]
    fn thermal_emergency_in_auto_releases_everything_and_stays_released() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-auto");
        let (mut ctl, gpu_calls) = auto_controller(&runner, path.clone(), Config::default());
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0)); // limits applied: 17 W + 1653 MHz
        assert_eq!(ryzenadj_calls(&runner).len(), 1, "premise");
        assert_eq!(gpu_sets(&gpu_calls).len(), 1, "premise");

        // Two hot samples: not enough (transient spikes must not release).
        assert!(ctl.on_sample(&overheat_at(1.0)).is_empty());
        assert!(ctl.on_sample(&overheat_at(2.0)).is_empty());
        assert_eq!(ctl.status().mode, Mode::Auto);

        // Third consecutive hot sample: full release toward stock.
        let effects = ctl.on_sample(&overheat_at(3.0));
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(has_flagged(&effects, "thermal_emergency", true));
        assert!(has_status_change_cause(
            &effects,
            "watchdog:thermal_emergency"
        ));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert_eq!(ctl.status().gpu_max_mhz, None);
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));
        assert!(ctl.auto.is_none(), "AutoState must drop whole");
        // GPU locks released + CPU stock restored (profile toggled back).
        assert!(
            gpu_calls
                .lock()
                .unwrap()
                .iter()
                .any(|c| matches!(c, GpuCall::Release)),
            "GPU release missing: {:?}",
            gpu_calls.lock().unwrap()
        );
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");

        // 20 more samples spanning the 10 s reassert window (hot AND cool):
        // nothing may re-enter Auto or reassert the released limits, and the
        // tripped watchdog must not re-fire.
        let (cpu_calls, gpu_cmds) = (ryzenadj_calls(&runner).len(), gpu_sets(&gpu_calls).len());
        for t in 4..24 {
            let s = if t % 2 == 0 {
                overheat_at(f64::from(t))
            } else {
                busy_at(f64::from(t))
            };
            assert!(ctl.on_sample(&s).is_empty(), "released state must hold");
        }
        assert_eq!(
            ryzenadj_calls(&runner).len(),
            cpu_calls,
            "zero new ryzenadj"
        );
        assert_eq!(gpu_sets(&gpu_calls).len(), gpu_cmds, "zero new GPU locks");
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn emergency_acknowledge_first_press_rearms_second_acts() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-ack");
        let (mut ctl, _gpu_calls) = auto_controller(&runner, path, Config::default());
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        for t in 1..=3 {
            ctl.on_sample(&overheat_at(f64::from(t)));
        }
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));

        // First `a`: acknowledge only — flag cleared, watchdog re-armed,
        // Auto NOT entered, nothing actuated.
        let calls_before = runner.calls().len();
        let effects = ctl.on_command(Command::SetAuto(true));
        assert!(has_flagged(&effects, "thermal_emergency", false));
        assert!(has_status_change_cause(&effects, "watchdog:rearmed"));
        assert!(!ctl.status().flags.contains(&StatusFlag::ThermalEmergency));
        assert_eq!(ctl.status().mode, Mode::Monitor, "ack must not enter Auto");
        assert!(ctl.auto.is_none());
        assert_eq!(runner.calls().len(), calls_before, "ack must not actuate");

        // Second `a`: acts normally.
        ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Auto);

        // Re-armed for real: a fresh 3-sample hot streak trips again.
        ctl.on_sample(&busy_at(10.0));
        ctl.on_sample(&overheat_at(11.0));
        ctl.on_sample(&overheat_at(12.0));
        let effects = ctl.on_sample(&overheat_at(13.0));
        assert!(has_flagged(&effects, "thermal_emergency", true));
        assert_eq!(ctl.status().mode, Mode::Monitor);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn emergency_in_manual_releases_and_manual_ack_swallows_first_command() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-manual");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::SetCpuW(20.0));

        ctl.on_sample(&overheat_at(1.0));
        ctl.on_sample(&overheat_at(2.0));
        let effects = ctl.on_sample(&overheat_at(3.0));
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(has_status_change_cause(
            &effects,
            "watchdog:thermal_emergency"
        ));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");

        // First c-press: swallowed acknowledge (no ryzenadj, still Monitor).
        let cpu_calls = ryzenadj_calls(&runner).len();
        let effects = ctl.on_command(Command::SetCpuW(20.0));
        assert!(has_status_change_cause(&effects, "watchdog:rearmed"));
        assert_eq!(ryzenadj_calls(&runner).len(), cpu_calls);
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);

        // Second press acts normally.
        ctl.on_command(Command::SetCpuW(20.0));
        assert_eq!(ctl.status().mode, Mode::Manual);
        assert_eq!(ctl.status().cpu_limit_w, Some(20.0));
        assert_eq!(ryzenadj_calls(&runner).len(), cpu_calls + 1);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn non_actuating_commands_pass_through_without_acknowledging() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-passthrough");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::SetCpuW(20.0));
        for t in 1..=3 {
            ctl.on_sample(&overheat_at(f64::from(t)));
        }
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));

        // Fan target and ReleaseAll execute normally and do NOT count as the
        // acknowledgment (they cannot re-apply limits, so the emergency flag
        // must stay visible until a deliberate re-arm).
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(ctl.status().fan_target_rpm, 2500.0);
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));
        let effects = ctl.on_command(Command::ReleaseAll);
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));

        // Quit works regardless of the flag.
        let effects = ctl.on_command(Command::Quit);
        assert_eq!(effects, vec![Effect::Quit]);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn emergency_during_calibration_aborts_it() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-calib");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        drive_matrix_point(&mut ctl, 0);
        drive_matrix_point(&mut ctl, 1); // burner + 30 W limit now active
        assert!(ctl.burner.is_some(), "premise: loaded matrix point");

        ctl.on_sample(&overheat_at(1000.0));
        ctl.on_sample(&overheat_at(1001.0));
        let effects = ctl.on_sample(&overheat_at(1002.0));
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(has_status_change_cause(
            &effects,
            "watchdog:thermal_emergency"
        ));
        assert!(ctl.burner.is_none(), "emergency must stop the burner");
        assert!(ctl.calib.is_none());
        assert!(ctl.status().calib.is_none());
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));
        // The calibration release toggled the profile back; smu untouched.
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        assert_eq!(modprobe_reload_calls(&runner), 0);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn ten_invalid_temp_samples_with_a_limit_trip_sensor_lost() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-sensor-lost");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::SetCpuW(20.0));

        // sample_at has cpu_temp_valid == false: nine are not enough.
        for t in 1..=9 {
            assert!(ctl.on_sample(&sample_at(f64::from(t))).is_empty());
        }
        assert_eq!(ctl.status().cpu_limit_w, Some(20.0));

        // The tenth trips: assume hot, release, distinct SensorLost flag.
        let effects = ctl.on_sample(&sample_at(10.0));
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(has_flagged(&effects, "sensor_lost", true));
        assert!(has_status_change_cause(&effects, "watchdog:sensor_lost"));
        assert!(ctl.status().flags.contains(&StatusFlag::SensorLost));
        assert!(!ctl.status().flags.contains(&StatusFlag::ThermalEmergency));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");

        // Same manual re-arm semantics as the thermal trip.
        let effects = ctl.on_command(Command::SetCpuW(20.0));
        assert!(has_flagged(&effects, "sensor_lost", false));
        assert!(!ctl.status().flags.contains(&StatusFlag::SensorLost));
        assert_eq!(ctl.status().cpu_limit_w, None, "first press only acks");
        ctl.on_command(Command::SetCpuW(20.0));
        assert_eq!(ctl.status().cpu_limit_w, Some(20.0));

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn watchdog_stays_armed_in_pure_monitor() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-monitor");
        let mut ctl = controller(&runner, path);

        // Hot (and invalid) samples with nothing commanded: nothing to
        // release, no flags, no effects — and NO latched trip that would
        // blind the watchdog later.
        for t in 0..20 {
            assert!(ctl.on_sample(&overheat_at(f64::from(t))).is_empty());
        }
        for t in 20..40 {
            assert!(ctl.on_sample(&sample_at(f64::from(t))).is_empty());
        }
        assert!(ctl.status().flags.is_empty());

        // Enter Manual while still hot: a fresh streak trips promptly.
        ctl.on_command(Command::SetCpuW(20.0));
        ctl.on_sample(&overheat_at(40.0));
        ctl.on_sample(&overheat_at(41.0));
        let effects = ctl.on_sample(&overheat_at(42.0));
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn gpu_heat_trips_too() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("watchdog-gpu-heat");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::SetCpuW(20.0));

        // GPU at its 87 °C threshold, CPU valid and cool: the OR trips.
        let gpu_hot_at = |t: f64| Sample {
            t_mono: t,
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            gpu_temp_c: 87.0,
            gpu_temp_valid: true,
            ..Sample::default()
        };
        ctl.on_sample(&gpu_hot_at(1.0));
        ctl.on_sample(&gpu_hot_at(2.0));
        let effects = ctl.on_sample(&gpu_hot_at(3.0));
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::ThermalEmergency));

        fs::remove_dir_all(&dir).unwrap();
    }

    // --- Task 29: resume hardening + floor editing ---

    /// (controller, gpu call log, resumed-hook counter) fixture triple.
    type ResumeFixture<'r> = (
        Controller<&'r FakeRunner>,
        Arc<Mutex<Vec<GpuCall>>>,
        Arc<Mutex<usize>>,
    );

    /// Calibrated controller like `auto_controller`, additionally exposing
    /// the FakeGpu's resumed-hook counter.
    fn auto_controller_with_resume_counter(runner: &FakeRunner) -> ResumeFixture<'_> {
        let gpu = FakeGpu::new();
        let gpu_calls = gpu.calls();
        let resumed_count = gpu.resumed_count();
        let ctl = Controller::new(
            RestoreGuard::new(
                runner,
                Some(cpu_actuator(
                    runner,
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
        (ctl, gpu_calls, resumed_count)
    }

    #[test]
    fn resume_pokes_gpu_resumed_hook_in_manual() {
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls, resumed_count) = auto_controller_with_resume_counter(&runner);
        ctl.on_command(Command::SetGpuMaxClock(1500));
        assert_eq!(*resumed_count.lock().unwrap(), 0, "premise");

        let effects = ctl.on_sample(&Sample {
            t_mono: 100.0,
            resumed: true,
            ..Sample::default()
        });
        // Persistence hook poked exactly once, and the lock reasserted.
        assert_eq!(*resumed_count.lock().unwrap(), 1);
        assert!(has_reassert(&effects, "resume"), "got {effects:?}");
        assert_eq!(gpu_sets(&gpu_calls), vec![1500, 1500], "set + reassert");

        // Ordinary samples must NOT poke the hook (it is per-resume, not
        // per-reassert: the 10 s reassert at 110.1 stays hook-free).
        ctl.on_sample(&sample_at(105.0));
        let effects = ctl.on_sample(&sample_at(110.2));
        assert!(has_reassert(&effects, "reassert"), "got {effects:?}");
        assert_eq!(*resumed_count.lock().unwrap(), 1);
    }

    #[test]
    fn resume_pokes_gpu_resumed_hook_in_auto() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls, resumed_count) = auto_controller_with_resume_counter(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(*resumed_count.lock().unwrap(), 0, "premise");

        let effects = ctl.on_sample(&Sample {
            resumed: true,
            ..busy_at(1.0)
        });
        assert_eq!(*resumed_count.lock().unwrap(), 1);
        assert!(has_reassert(&effects, "resume"), "got {effects:?}");
        assert_eq!(ctl.status().mode, Mode::Auto, "auto survives the resume");
    }

    #[test]
    fn resume_without_gpu_actuator_still_reasserts() {
        // No GPU this run: the resume path must not assume the hook exists.
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));
        let effects = ctl.on_sample(&Sample {
            t_mono: 100.0,
            resumed: true,
            ..Sample::default()
        });
        assert!(has_reassert(&effects, "resume"), "got {effects:?}");
    }

    #[test]
    fn strict_stickiness_window_fires_on_two_violations_after_resume() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));

        // Resume at t=100: the strict window opens (until t=160).
        ctl.on_sample(&Sample {
            t_mono: 100.0,
            resumed: true,
            ..Sample::default()
        });

        // TWO violations suffice inside the window (normally three).
        assert!(
            !ctl.status().flags.contains(&StatusFlag::LimitNotSticking),
            "premise"
        );
        ctl.on_sample(&sample_with_power(101.0, 26.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        let effects = ctl.on_sample(&sample_with_power(102.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));

        // A compliant sample clears the flag and the streak.
        ctl.on_sample(&sample_with_power(103.0, 19.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));

        // Past the 60 s window (t >= 160): back to the normal 3-sample rule.
        ctl.on_sample(&sample_with_power(161.0, 26.0));
        let effects = ctl.on_sample(&sample_with_power(162.0, 26.0));
        assert!(
            !has_reassert(&effects, "stickiness"),
            "two violations after the strict window must not fire, got {effects:?}"
        );
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        let effects = ctl.on_sample(&sample_with_power(163.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
    }

    #[test]
    fn set_floors_sanitizes_echoes_and_persists_on_change_only() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-floors-persist",
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
        // Construction seeds the status floors from config.
        assert_eq!(ctl.status().cpu_floor_w, 15.0);
        assert_eq!(ctl.status().gpu_floor_mhz, 1000);

        let effects = ctl.on_command(Command::SetFloors {
            cpu_w: 20.0,
            gpu_mhz: 1105,
        });
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ctl.status().cpu_floor_w, 20.0);
        assert_eq!(ctl.status().gpu_floor_mhz, 1105);
        let saved = Config::load(&config_path);
        assert_eq!(saved.cpu_floor_w, 20.0);
        assert_eq!(saved.gpu_floor_mhz, 1105);
        assert_eq!(
            saved.fan_target_rpm,
            Config::default().fan_target_rpm,
            "other config fields preserved"
        );
        // Floors are config, not actuation: mode stays Monitor, no commands.
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(runner.calls().is_empty());

        // Unchanged floors (held key at a clamp bound): no re-save, no
        // status spam.
        fs::remove_file(&config_path).unwrap();
        let effects = ctl.on_command(Command::SetFloors {
            cpu_w: 20.0,
            gpu_mhz: 1105,
        });
        assert_eq!(status_changes(&effects), 0, "got {effects:?}");
        assert!(
            !config_path.exists(),
            "unchanged floors must not spam config saves"
        );

        // Out-of-range floors sanitize exactly like config load.
        ctl.on_command(Command::SetFloors {
            cpu_w: 99.0,
            gpu_mhz: 500,
        });
        assert_eq!(ctl.status().cpu_floor_w, 54.0);
        assert_eq!(ctl.status().gpu_floor_mhz, 1000);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn set_floors_shifts_the_next_allocation() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        // Fully idle machine at the fan target: allocation sits at the floor.
        let idle_at = |t: f64| Sample {
            t_mono: t,
            gpu_w: 10.0,
            gpu_w_valid: true,
            fan1_rpm: 3000.0,
            fan_valid: true,
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            ..Sample::default()
        };
        ctl.on_sample(&idle_at(0.0));
        ctl.on_sample(&idle_at(5.0));
        assert_eq!(ctl.status().cpu_limit_w, Some(15.0), "premise: at floor");

        // Raise the CPU floor: honored from the next allocator step on.
        ctl.on_command(Command::SetFloors {
            cpu_w: 25.0,
            gpu_mhz: 1000,
        });
        assert_eq!(ctl.status().mode, Mode::Auto, "floors allowed in Auto");
        let effects = ctl.on_sample(&idle_at(10.0));
        let (cpu_w, _) = alloc_of(&effects).expect("allocator step due");
        assert!(cpu_w >= 25.0, "allocation {cpu_w} W below the new floor");
        assert_eq!(ctl.status().cpu_limit_w, Some(cpu_w));
    }

    #[test]
    fn set_floors_rejected_while_calibrating() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::StartCalibration);
        assert_eq!(ctl.status().mode, Mode::Calibrating, "premise");

        let effects = ctl.on_command(Command::SetFloors {
            cpu_w: 25.0,
            gpu_mhz: 1200,
        });
        assert!(effects.is_empty(), "got {effects:?}");
        assert_eq!(ctl.status().cpu_floor_w, 15.0, "floors unchanged");
        assert_eq!(ctl.status().gpu_floor_mhz, 1000);
        assert_eq!(ctl.config.cpu_floor_w, 15.0);
    }

    #[test]
    fn set_floors_passes_the_emergency_gate_without_acknowledging() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("floors-emergency");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::SetCpuW(20.0));
        for t in 1..=3 {
            ctl.on_sample(&overheat_at(f64::from(t)));
        }
        assert!(
            ctl.status().flags.contains(&StatusFlag::ThermalEmergency),
            "premise: tripped"
        );

        // Floors don't actuate: they apply immediately AND leave the
        // emergency latched (the acknowledge stays with the user).
        let effects = ctl.on_command(Command::SetFloors {
            cpu_w: 25.0,
            gpu_mhz: 1200,
        });
        assert_eq!(ctl.status().cpu_floor_w, 25.0);
        assert_eq!(ctl.status().gpu_floor_mhz, 1200);
        assert!(
            ctl.status().flags.contains(&StatusFlag::ThermalEmergency),
            "floors must not consume the acknowledge"
        );
        assert!(
            !has_status_change_cause(&effects, "watchdog:rearmed"),
            "got {effects:?}"
        );

        // The two-step acknowledge still works as designed afterwards.
        ctl.on_command(Command::SetCpuW(20.0)); // ack only
        assert_eq!(ctl.status().cpu_limit_w, None);
        ctl.on_command(Command::SetCpuW(20.0)); // acts
        assert_eq!(ctl.status().cpu_limit_w, Some(20.0));

        fs::remove_dir_all(&dir).unwrap();
    }
}

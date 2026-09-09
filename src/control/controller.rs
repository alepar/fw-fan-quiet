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

use std::collections::BTreeMap;
use std::time::Instant;

use crate::actuators::WriteVerdict;
use crate::actuators::cmd::Runner;
use crate::actuators::gpu::GpuLockVerifier;
use crate::actuators::guard::RestoreGuard;
use crate::calib::burner::Burner;
use crate::calib::runner::{CalibContext, CalibRunner, RunnerEffect};
use crate::calib::steady::tail_mean;
use crate::config::Config;
use crate::control::allocator::{self, AllocInput, Allocator};
use crate::control::budget::{Budget, Freeze as BudgetFreeze, LoopError, LoopGains};
use crate::control::gpu_pid::GpuPid;
use crate::control::guards::{GuardState, Guards, gpu_share_override};
use crate::control::lut::ClockWattsLut;
use crate::control::mode::{Arbiter, ArbiterInput, Decision};
use crate::control::watchdog::{ThermalWatchdog, Trip};
use crate::event::Event;
use crate::fanctrl::curve::Curve;
use crate::fanctrl::table::DutyRpmTable;
use crate::sensors::ec::EcAverage;
use crate::state::PersistedState;
use crate::telemetry::{self, Record, Telemetry};
use crate::types::Sample;

/// UI-facing calibration progress, re-exported so the view/model layers name
/// it without reaching into `calib::`.
pub use crate::calib::runner::CalibProgress as CalibProgressLite;

/// Reapply active limits at least this often (defends against PPD/tuned
/// clobbering the ryzenadj limits behind our back; design §3).
const REASSERT_PERIOD_S: f64 = 10.0;
/// Auto-mode allocator cadence (design §3: retarget the CPU/GPU split every
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
/// Cap on the Auto-mode fan-RPM window (`auto.fan_window`, 1 Hz samples).
const FAN_WINDOW_CAP: usize = 30;
/// Fan target clamp range (RPM); the Auto-mode allocator consumes the
/// target live via `status.fan_target_rpm`.
const FAN_TARGET_MIN_RPM: f64 = 1000.0;
const FAN_TARGET_MAX_RPM: f64 = 7000.0;
/// Startup fan target (RPM). Shared with the UI model so the controller's
/// echoed status and the displayed default can never diverge.
pub const DEFAULT_FAN_TARGET_RPM: f64 = 3000.0;
/// Tail-window size (samples, 1 Hz) `rpm_smoothed` averages over — design
/// §3.2's data flow names `FAN_SMOOTH_N` without pinning a value; short
/// enough that Mode B's error tracks a real fan-speed change within a few
/// seconds, long enough to reject single-sample tach noise.
const FAN_SMOOTH_N: usize = 5;
/// Default dGPU-hot guard threshold, °C (design §2.8).
const GPU_HOT_C_DEFAULT: f64 = crate::control::guards::GPU_HOT_C_DEFAULT;
/// Default NVMe-hot guard threshold, °C (design §2.8).
const NVME_HOT_C_DEFAULT: f64 = crate::control::guards::NVME_HOT_C_DEFAULT;
/// Consecutive scored `Mismatch` verdicts (after the one re-read) before an
/// actuator releases to stock with its flag held (design §2.9).
const MISMATCH_RELEASE_STRIKES: u8 = 3;
/// Consecutive `Unreadable`/`Unverifiable` verdicts before `ReadbackBlind`
/// is raised (design §2.9).
const READBACK_BLIND_STRIKES: u8 = 6;
/// How long a candidate `Mismatch` is suppressed after an `on_ac` edge,
/// seconds (design §2.9: "suppressed for 3 ticks"; ticks here are 1 Hz
/// samples, the cadence the actuator write/verify path runs at).
const ON_AC_EDGE_SUPPRESS_S: f64 = 3.0;
/// Sample-count span `replica_slope_5s_c_per_s` measures the EC replica's
/// own slope over (design §2.6: "the replica's own slope over the last
/// 5 s"), at the 1 Hz sample rate.
const EC_SLOPE_WINDOW_S: usize = 5;

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
    TempLoop,
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
    /// Auto mode was requested without a calibrated LUT. Cleared on a
    /// successful Auto entry or when a calibration lands its fit.
    NotCalibrated,
    /// The fans stay persistently over target even at the maximum budget
    /// cut — check intake/ambient (research 03 §6: surface a status when
    /// the floor is hit instead of silently collapsing performance). Not
    /// raised by any call site today: the adaptation tier that used to
    /// drive it off the Kalman bias is gone (`fw-fanctrl-loop-24s`); a
    /// later task (the budget integrator/arbiter) re-wires this flag.
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
/// Default `EcAverage` boxcar interval before the first fw-fanctrl view has
/// ever been observed this session (§Facts: 60 on both live curves).
const DEFAULT_MA_INTERVAL: usize = 60;

/// Shared actuator read-back verdict state machine (design §2.9): one
/// instance per actuator (CPU, GPU). The caller re-reads a candidate
/// `Mismatch` once itself (re-invoking the write for CPU; re-scoring
/// `verify_lock` against the same sample for GPU, which has no second NVML
/// reading available within one 1 Hz tick) and feeds THIS method only the
/// confirmed, post-re-read verdict — [`VerdictState`] itself never sees the
/// unconfirmed first read. A confirmed `Mismatch` sets `LimitNotSticking` +
/// `Freeze::ActuatorMismatch` + an immediate reassert on the same tick;
/// [`MISMATCH_RELEASE_STRIKES`] consecutive confirmed mismatches release the
/// actuator to stock with the flag held (the read-back keeps running every
/// reassert period so a later `Verified` is producible); `Unreadable`/
/// `Unverifiable` are non-events; [`READBACK_BLIND_STRIKES`] consecutive
/// `Unreadable` raise `ReadbackBlind` until the next `Verified`.
#[derive(Debug, Clone, Copy, Default)]
struct VerdictState {
    /// Consecutive confirmed (post-re-read) `Mismatch` verdicts.
    mismatch_streak: u8,
    /// Consecutive `Unreadable` verdicts (any `Verified`/`Mismatch`/
    /// `Unverifiable` resets it — only `Unreadable` accumulates here).
    unreadable_streak: u8,
    /// Set once [`MISMATCH_RELEASE_STRIKES`] have released this actuator;
    /// only a `Verified` clears it (§2.9: the flag stays held across the
    /// release even though the strike count itself resets to 0).
    released: bool,
    /// `ReadbackBlind` is currently latched for this actuator.
    blind: bool,
}

/// What the controller should do this tick in response to one actuator's
/// freshly confirmed verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VerdictOutcome {
    /// Nothing new: still quiet, or a non-event that didn't cross a
    /// threshold.
    Quiet,
    /// First `Verified` after trouble: strikes/blind/released all clear.
    Recovered,
    /// A freshly confirmed `Mismatch`, not yet the release strike.
    Mismatch,
    /// The release-strike `Mismatch` (release to stock, flag held).
    Released,
    /// The `ReadbackBlind`-raising `Unreadable`.
    Blind,
}

impl VerdictState {
    /// True while this actuator is mid-episode (a confirmed mismatch
    /// streak in progress, or already released) — the controller reads
    /// this at the top of a tick, BEFORE this tick's own fresh verdict is
    /// known, to decide whether the shared budget starts this tick frozen
    /// (§2.9's "freeze" applies to the tick the trouble is discovered on;
    /// since the freeze/write ordering is circular within one tick — the
    /// write needs `u`, `u` needs the freeze — an episode already in
    /// progress as of the LAST tick's verdict is what gates THIS tick's
    /// step, one tick behind the very first sighting).
    fn in_episode(&self) -> bool {
        self.mismatch_streak > 0 || self.released
    }

    /// Feed one tick's CONFIRMED (already re-read once, if it was a
    /// candidate mismatch) [`WriteVerdict`]. `suppress` is true within
    /// [`ON_AC_EDGE_SUPPRESS_S`] of an `on_ac` edge — a mismatch is dropped
    /// outright (not scored at all) while suppressed, exactly like
    /// RyzenAdj's own documented AC-transition reassertion.
    fn observe(&mut self, verdict: WriteVerdict, suppress: bool) -> VerdictOutcome {
        match verdict {
            WriteVerdict::Verified(_) => {
                self.unreadable_streak = 0;
                let was_trouble = self.mismatch_streak > 0 || self.released || self.blind;
                self.mismatch_streak = 0;
                self.released = false;
                self.blind = false;
                if was_trouble {
                    VerdictOutcome::Recovered
                } else {
                    VerdictOutcome::Quiet
                }
            }
            WriteVerdict::Unreadable => {
                if suppress {
                    return VerdictOutcome::Quiet;
                }
                self.unreadable_streak = self.unreadable_streak.saturating_add(1);
                if self.unreadable_streak >= READBACK_BLIND_STRIKES && !self.blind {
                    self.blind = true;
                    VerdictOutcome::Blind
                } else {
                    VerdictOutcome::Quiet
                }
            }
            WriteVerdict::Unverifiable => {
                self.unreadable_streak = 0;
                VerdictOutcome::Quiet
            }
            WriteVerdict::Mismatch { .. } => {
                self.unreadable_streak = 0;
                if suppress {
                    return VerdictOutcome::Quiet;
                }
                self.mismatch_streak = self.mismatch_streak.saturating_add(1);
                if self.mismatch_streak >= MISMATCH_RELEASE_STRIKES {
                    self.mismatch_streak = 0;
                    self.released = true;
                    VerdictOutcome::Released
                } else {
                    VerdictOutcome::Mismatch
                }
            }
        }
    }
}

struct AutoState {
    /// GPU watts→clock inner PI (1 Hz).
    pid: GpuPid,
    /// Contour allocator (every ALLOC_PERIOD_S).
    allocator: Allocator,
    /// t_mono of the last allocator step; None → step on the next sample.
    last_alloc: Option<f64>,
    /// Current PI watts target (allocator output); demand input next step.
    gpu_target_w: Option<f64>,
    /// Fan-RPM window feeding `rpm_smoothed` (design §3.2's data flow);
    /// fan-invalid samples land as NaN (the charts/steady.rs convention) so
    /// a tail spanning a sensor outage is never mistaken for settled
    /// evidence.
    fan_window: std::collections::VecDeque<f64>,
    /// The single budget integrator (design §2.4).
    budget: Budget,
    /// The mode arbiter (design §2.5-§2.7).
    arbiter: Arbiter,
    /// Live `EcAverage` replica (design §2.6): owned here, not rebuilt per
    /// tick, so its boxcar survives across samples within one Auto session.
    ec_avg: EcAverage,
    /// The controller's own read of `ec_avg`'s last push (design §2.6:
    /// "supplies `ec_ma` to the arbiter and to `ControlStatus.ec_ma_c`").
    ec_ma: Option<f64>,
    /// The MHz `gpu_verifier` is currently scoped to; `GpuLockVerifier`
    /// exposes no getter for its own locked value, so this is tracked
    /// alongside it to know when a freshly applied clock needs a fresh
    /// verifier (its violation streak is scoped to one locked value).
    gpu_verifier_mhz: Option<u32>,
    /// Short raw `ec.max_c` history feeding `replica_slope_5s_c_per_s`
    /// (design §2.6's scoring skip rule): capped at
    /// `EC_SLOPE_WINDOW_S / sample period (1 s)` samples.
    ec_slope_window: std::collections::VecDeque<f64>,
    /// dGPU + NVMe thermal guards (design §2.8).
    guards: Guards,
    /// Read-back verdict state, one per actuator (design §2.9).
    cpu_verdict: VerdictState,
    gpu_verdict: VerdictState,
    /// GPU lock verifier for the currently-commanded clock; recreated
    /// whenever a new clock is applied (`verify_lock`'s streak is scoped to
    /// one locked value).
    gpu_verifier: Option<GpuLockVerifier>,
    /// Loop mode as of the last arbiter call, used to detect the
    /// `Released` -> usable transition (re-seed from the floors, §2.4) and
    /// to emit `Noted { mode: ... }` on every genuine change.
    last_mode: LoopMode,
    /// True once `budget` has been seeded this engagement (auto entry, or
    /// the most recent re-engagement from `Released`); cleared whenever the
    /// loop mode becomes `Released` so the NEXT usable tick re-seeds.
    budget_seeded: bool,
    /// True once `ec_avg` has been seeded from `view.ma_temperature` this
    /// engagement (design §2.6: "seeded ... on auto entry, on
    /// re-engagement, and at calibration exit" — every case funnels
    /// through a fresh engagement, since a fresh `AutoState` is
    /// constructed on every entry); cleared alongside `budget_seeded`.
    ec_seeded: bool,
    /// (draw, cap, floor) each axis carried into `Budget::set_demand_state`
    /// —_this_ tick's demand-limited verdict is judged against the
    /// PREVIOUS tick's post-guard-override cap (this tick's own cap does
    /// not exist yet: it is `split_budget`'s output, which itself needs
    /// this tick's `u`, which needs the demand-limited verdict first).
    /// Seeded at the floors on entry.
    last_cpu_cap_w: f64,
    last_gpu_cap_w: f64,
    /// The RPM/temperature error's sign as of the last 5 s budget step,
    /// mirrored into `ArbiterInput::error_sign` on every arbiter call in
    /// between (§2.7's low/high unreachable rules read it every tick, not
    /// just on budget-step ticks).
    last_error_sign: f64,
    /// Cached target duty (design §2.3: `duty_for_rpm`), recomputed every
    /// 5 s budget step and reused by every arbiter call in between.
    target_duty: u8,
    /// `s.on_ac` as of the previous sample, to detect an edge.
    last_on_ac: Option<bool>,
    /// `t_mono` until which a candidate actuator `Mismatch` is suppressed
    /// (design §2.9: 3 ticks after an `on_ac` edge).
    on_ac_suppress_until: Option<f64>,
}

impl AutoState {
    fn new(gains: &LoopGains) -> Self {
        Self {
            pid: GpuPid::new(),
            allocator: Allocator::new(),
            last_alloc: None,
            gpu_target_w: None,
            fan_window: std::collections::VecDeque::new(),
            budget: Budget::new(gains),
            arbiter: Arbiter::new(),
            ec_avg: EcAverage::new(DEFAULT_MA_INTERVAL),
            ec_ma: None,
            gpu_verifier_mhz: None,
            ec_slope_window: std::collections::VecDeque::new(),
            guards: Guards::new(GPU_HOT_C_DEFAULT, NVME_HOT_C_DEFAULT),
            cpu_verdict: VerdictState::default(),
            gpu_verdict: VerdictState::default(),
            gpu_verifier: None,
            last_mode: LoopMode::default(),
            budget_seeded: false,
            ec_seeded: false,
            last_cpu_cap_w: 0.0,
            last_gpu_cap_w: 0.0,
            last_error_sign: 0.0,
            target_duty: 0,
            last_on_ac: None,
            on_ac_suppress_until: None,
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
    /// GPU clock→watts LUT: loaded from the state file at construction,
    /// replaced by a fresh calibration; the GPU PI's feedforward.
    lut: Option<ClockWattsLut>,
    /// Wall-clock stamp of the loaded calibration, carried so an Auto-exit
    /// state write preserves it (only a finished calibration sets it).
    calibrated_at: Option<String>,
    /// Fitted PI gains for both loop legs (design §2.4); `None` until a
    /// step test lands, in which case `Budget::new` falls back to
    /// `LoopGains::default()`. Loaded from the state file, replaced by a
    /// fresh fit.
    loop_gains: Option<LoopGains>,
    /// Duty<->RPM lookup (design §2.3), persisted across sessions.
    duty_rpm_table: DutyRpmTable,
    /// Warm-start budget seeds, keyed by `WarmStart::key` (design §2.4).
    warm_start: BTreeMap<String, f64>,
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

/// Static `Effect::Noted` cause for a genuine `LoopMode` transition (design
/// §2.5: "every transition emits `Noted`"). Exhaustive over the 6 possible
/// directed transitions between the 3 modes (the 3 `from == to` diagonal
/// cells never reach this — callers only invoke it on a genuine change).
fn mode_transition_cause(from: LoopMode, to: LoopMode) -> &'static str {
    use LoopMode::{Released, RpmLoop, TempLoop};
    match (from, to) {
        (Released, TempLoop) => "mode:released->temploop",
        (Released, RpmLoop) => "mode:released->rpmloop",
        (TempLoop, RpmLoop) => "mode:temploop->rpmloop",
        (TempLoop, Released) => "mode:temploop->released",
        (RpmLoop, TempLoop) => "mode:rpmloop->temploop",
        (RpmLoop, Released) => "mode:rpmloop->released",
        (TempLoop, TempLoop) | (RpmLoop, RpmLoop) | (Released, Released) => "mode:unchanged",
    }
}

/// Unwraps a `LoopError`'s scalar value (°C for `Temp`, RPM for `Rpm`) —
/// `LoopError::value` is private to `budget`, so callers outside it match
/// on the public fields instead.
fn loop_error_value(err: LoopError) -> f64 {
    match err {
        LoopError::Temp { e_c } => e_c,
        LoopError::Rpm { e_rpm } => e_rpm,
    }
}

/// Telemetry string for a `Budget::Freeze` reason (design §2.9's
/// `AutoAllocated.freeze` field).
fn freeze_str(f: BudgetFreeze) -> &'static str {
    match f {
        BudgetFreeze::ActuatorMismatch => "actuator_mismatch",
        BudgetFreeze::Calibrating => "calibrating",
        BudgetFreeze::Released => "released",
        BudgetFreeze::DemandLimited => "demand_limited",
    }
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
            lut: persisted.lut,
            calibrated_at: persisted.calibrated_at,
            loop_gains: persisted.loop_gains,
            duty_rpm_table: persisted.duty_rpm_table,
            warm_start: persisted.warm_start,
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
        // rejected (SetFanTarget stays allowed — it retargets the fan
        // target live; ReleaseAll/SetAuto(false) are the ways out).
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
                // conservative allocator on re-entry).
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
                } else if self.lut.is_none() {
                    tracing::warn!(
                        "auto mode requires a calibrated LUT; run a calibration (k) first"
                    );
                    self.add_flag(StatusFlag::NotCalibrated);
                    "auto:not_calibrated"
                } else {
                    self.remove_flag(StatusFlag::NotCalibrated);
                    let gains = self.loop_gains.unwrap_or_default();
                    let mut auto = AutoState::new(&gains);
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
                    // Fresh PI/allocator on re-entry.
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
                // Clean quit drops a live Auto session's loop state before
                // the hardware restore.
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
            // The pre-suspend fan window and EC boxcar are thermally stale
            // (the machine cooled while asleep) — clear both so neither can
            // read a slope/mean spanning the suspend as settled evidence
            // (review finding; design §2.6: "cleared on a resumed sample").
            if let Some(auto) = &mut self.auto {
                auto.fan_window.clear();
                auto.ec_avg = EcAverage::new(DEFAULT_MA_INTERVAL);
                auto.ec_ma = None;
                auto.ec_slope_window.clear();
                // Re-arm the "seed from view.ma_temperature" one-shot (the
                // window-push block below only seeds when !ec_seeded) so
                // THIS sample's own view (if any) re-seeds it immediately,
                // exactly like a fresh auto entry.
                auto.ec_seeded = false;
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
                    // §2.9's actuator-verdict episodes share this flag with
                    // the RAPL check: only clear it here when NEITHER
                    // source still reports trouble — `sync_limit_not_sticking`
                    // (called from the verdict path) only ever ADDS, so this
                    // is the flag's one and only remover.
                    let verdict_trouble = self
                        .auto
                        .as_ref()
                        .is_some_and(|a| a.cpu_verdict.in_episode() || a.gpu_verdict.in_episode());
                    if !verdict_trouble {
                        self.remove_flag(StatusFlag::LimitNotSticking);
                    }
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
            // A cause fired (e.g. a held/frozen allocator step) with no
            // visible status delta. Still worth a telemetry Decision line
            // (mirrors `on_calib_sample`'s same fallback below).
            effects.push(Effect::Noted { cause });
        }
        effects
    }

    /// One Auto-mode control step, driven off the 1 Hz samples (t_mono-based
    /// like the reassert): every [`ALLOC_PERIOD_S`] an allocator step runs
    /// (see its stubbed-split note below); every sample the GPU
    /// watts→clock PI trims the locked clock toward its watts target.
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
        // Defensive: Auto without its state cannot control anything — fail
        // toward stock (every failure path degrades to louder fans or stock
        // behavior, design §5). Unreachable in practice: entry requires the
        // LUT and it is only ever replaced, never cleared.
        if self.auto.is_none() || self.lut.is_none() {
            tracing::warn!("auto mode lost its state; releasing to Monitor");
            // Plain drop, deliberately NOT exit_auto_and_persist: this is a
            // fault path (Auto lost its state mid-flight).
            self.auto = None;
            self.release_to_stock();
            effects.push(Effect::Released);
            cause.get_or_insert("auto:degraded");
            return;
        }

        // ---- Every sample: guards (§2.8), ahead of the arbiter ----
        let guard_state = {
            let auto = self.auto.as_mut().expect("checked above");
            auto.guards.step(
                if s.gpu_temp_valid {
                    Some(s.gpu_temp_c)
                } else {
                    None
                },
                s.nvme_temp_c,
            )
        };
        self.sync_bool_flag(StatusFlag::GpuHot, guard_state.gpu_hot);
        self.sync_bool_flag(StatusFlag::NvmeHot, guard_state.nvme_hot);

        // ---- Every sample: window pushes (fan, EC replica) ----
        {
            let auto = self.auto.as_mut().expect("checked above");
            if auto.fan_window.len() >= FAN_WINDOW_CAP {
                auto.fan_window.pop_front();
            }
            auto.fan_window.push_back(if s.fan_valid {
                s.max_fan_rpm()
            } else {
                f64::NAN
            });

            // Seed from `view.ma_temperature` on the first sample of this
            // engagement that carries a view (auto entry / re-engagement,
            // §2.6) — before any `push` below, so the very first mean read
            // already reflects the seed rather than a bare single sample.
            if !auto.ec_seeded
                && let Some(view) = &s.fanctrl
            {
                auto.ec_avg.reseed(view.ma_temperature);
                auto.ec_ma = Some(view.ma_temperature);
                auto.ec_seeded = true;
            }
            // `set_interval` on view change (§2.6), matching fw-fanctrl's
            // own boxcar width for the currently resolved strategy.
            if s.fanctrl_view_changed
                && let Some(view) = &s.fanctrl
            {
                auto.ec_avg.set_interval(view.ma_interval as usize);
            }

            if let Some(ec) = &s.ec {
                let max_c = f64::from(ec.max_c);
                auto.ec_ma = auto.ec_avg.push(max_c);
                if auto.ec_slope_window.len() >= EC_SLOPE_WINDOW_S {
                    auto.ec_slope_window.pop_front();
                }
                auto.ec_slope_window.push_back(max_c);
            }
        }
        self.status.ec_ma_c = self.auto.as_ref().expect("checked above").ec_ma;
        self.status.ec_argmax = s.ec.as_ref().map(|e| e.argmax.as_str().to_string());

        // ---- Every sample: on_ac edge (§2.9's actuator-verdict grace) ----
        {
            let auto = self.auto.as_mut().expect("checked above");
            if auto.last_on_ac.is_some_and(|last| last != s.on_ac) {
                auto.on_ac_suppress_until = Some(s.t_mono + ON_AC_EDGE_SUPPRESS_S);
            }
            auto.last_on_ac = Some(s.on_ac);
        }

        // ---- Every 5 s: bounds + target_duty, ahead of this tick's
        // arbiter call so the call already sees fresh values ----
        let due = self
            .auto
            .as_ref()
            .expect("checked above")
            .last_alloc
            .is_none_or(|last| s.t_mono - last >= ALLOC_PERIOD_S);
        if due {
            let lut = self.lut.as_ref().expect("checked above");
            let gpu_floor_w = lut
                .watts_for_clock(self.config.gpu_floor_mhz)
                .unwrap_or(0.0);
            let lo = self.config.cpu_floor_w + gpu_floor_w;
            let hi = self.config.cpu_max_w + self.config.gpu_max_w;
            let target_duty = self.duty_rpm_table.duty_for_rpm(self.status.fan_target_rpm);
            let auto = self.auto.as_mut().expect("checked above");
            auto.budget.set_bounds(lo, hi);
            auto.target_duty = target_duty;
        }

        // ---- Every sample: the arbiter tick (design §2.6: reconciliation
        // is scored on THIS sample, not deferred to the next 5 s tick, so a
        // view-changed edge that lands off the allocator's own cadence is
        // never dropped) ----
        let curve_valid = s
            .fanctrl
            .as_ref()
            .is_none_or(|v| Curve::from_points(v.curve.clone()).is_ok());
        let (replica_slope_5s_c_per_s, view_to_sample_gap_s) = if s.fanctrl_view_changed {
            let auto = self.auto.as_ref().expect("checked above");
            let slope = if auto.ec_slope_window.len() >= 2 {
                let first = *auto.ec_slope_window.front().expect("len >= 2");
                let last = *auto.ec_slope_window.back().expect("len >= 2");
                (last - first) / (auto.ec_slope_window.len() - 1) as f64
            } else {
                0.0
            };
            let gap = s
                .fanctrl
                .as_ref()
                .and_then(|v| v.all_observed_at)
                .map(|t| Instant::now().saturating_duration_since(t).as_secs_f64())
                .unwrap_or(0.0);
            (slope, gap)
        } else {
            (0.0, 0.0)
        };
        let decision = {
            let auto = self.auto.as_mut().expect("checked above");
            let input = ArbiterInput {
                fanctrl: s.fanctrl.as_ref(),
                freshness: s.fanctrl_freshness,
                view_changed: s.fanctrl_view_changed,
                ec: s.ec.as_ref(),
                ec_ma: auto.ec_ma,
                fan_valid: s.fan_valid,
                target_duty: auto.target_duty,
                at_lower_bound_for: auto.budget.at_lower_bound_for(),
                at_upper_bound_for: auto.budget.at_upper_bound_for(),
                error_sign: auto.last_error_sign,
                curve_valid,
                replica_slope_5s_c_per_s,
                view_to_sample_gap_s,
            };
            auto.arbiter.decide(&input)
        };

        // Status fields mirrored every tick (brief: "Status fields are
        // mirrored every tick").
        self.mirror_decision(s, &decision);

        // reseed_ma: re-seed the live EcAverage immediately, whenever it
        // fires (reconciliation clearing or an MA-check failure, §2.6).
        if let Some(v) = decision.reseed_ma {
            let auto = self.auto.as_mut().expect("checked above");
            auto.ec_avg.reseed(v);
            auto.ec_ma = Some(v);
            self.status.ec_ma_c = Some(v);
        }

        // t_star_changed: resync_error immediately, whatever cadence this
        // call happens to be on (§2.4: "on every T* or snapped-target
        // re-derivation" — resync only touches e_prev, so it needs no
        // alignment with the 5 s budget step).
        if decision.t_star_changed
            && let Some(err) = self.compute_loop_error(decision.mode, decision.t_star)
        {
            let e = loop_error_value(err);
            let auto = self.auto.as_mut().expect("checked above");
            auto.budget.resync_error(e);
        }

        // Update the error-sign mirror for the NEXT tick's ArbiterInput,
        // and detect the mode transition (Noted, Released hand-off).
        if let Some(err) = self.compute_loop_error(decision.mode, decision.t_star) {
            let auto = self.auto.as_mut().expect("checked above");
            auto.last_error_sign = loop_error_value(err).signum();
        }
        self.handle_mode_transition(s, decision.mode, effects);

        if due {
            self.auto.as_mut().expect("checked above").last_alloc = Some(s.t_mono);
            self.run_budget_and_allocate(s, &decision, guard_state, effects, cause);
        }

        self.run_gpu_pi(s, effects, cause);
    }

    /// Sets/clears a plain (non-arbiter-owned) flag from a live bool
    /// reading — `GpuHot`/`NvmeHot` (§2.8).
    fn sync_bool_flag(&mut self, flag: StatusFlag, active: bool) {
        if active {
            self.add_flag(flag);
        } else {
            self.remove_flag(flag);
        }
    }

    /// Mirrors every `ControlStatus` field the arbiter's `Decision` and
    /// this sample drive, every tick.
    fn mirror_decision(&mut self, s: &Sample, decision: &Decision) {
        self.status.loop_mode = decision.mode;
        self.status.t_star_c = decision.t_star;
        self.status.strategy = s.fanctrl.as_ref().map(|v| v.strategy.clone());

        if decision.mode == LoopMode::TempLoop
            && let Some(view) = &s.fanctrl
            && let Ok(curve) = Curve::from_points(view.curve.clone())
        {
            let target_duty = self.auto.as_ref().expect("in auto").target_duty;
            match curve.nearest_tread(target_duty) {
                Some(d) => {
                    self.status.duty_cmd = Some(d);
                    self.status.snapped_rpm = self.duty_rpm_table.rpm_for_duty(d);
                }
                None => {
                    self.status.duty_cmd = None;
                    self.status.snapped_rpm = 0.0;
                }
            }
        } else {
            self.status.duty_cmd = None;
            self.status.snapped_rpm = 0.0;
        }

        for flag in [
            StatusFlag::FanctrlLost,
            StatusFlag::EcMismatch,
            StatusFlag::SteepCurve,
            StatusFlag::CurveInvalid,
            StatusFlag::SensorLost,
        ] {
            self.sync_bool_flag(flag, decision.flags.contains(&flag));
        }
    }

    /// The loop error the current `mode`/`t_star` would produce right now
    /// (design §2.4's `LoopError`), or `None` when it isn't computable yet
    /// (no `t_star`/`ec_ma` for TempLoop, no finite `rpm_smoothed` for
    /// RpmLoop, or `Released`).
    fn compute_loop_error(&self, mode: LoopMode, t_star: Option<f64>) -> Option<LoopError> {
        let auto = self.auto.as_ref()?;
        match mode {
            LoopMode::TempLoop => {
                let t_star = t_star?;
                let ma = auto.ec_ma?;
                Some(LoopError::Temp { e_c: t_star - ma })
            }
            LoopMode::RpmLoop => {
                let target_rpm = self.duty_rpm_table.rpm_for_duty(auto.target_duty);
                let smoothed = self.rpm_smoothed_now(auto);
                if smoothed.is_finite() {
                    Some(LoopError::Rpm {
                        e_rpm: target_rpm - smoothed,
                    })
                } else {
                    None
                }
            }
            LoopMode::Released => None,
        }
    }

    /// `rpm_smoothed` (design §3.2's data flow): the `FAN_SMOOTH_N` tail
    /// mean of the fan window, falling back to the window's most recent
    /// entry (which may itself be NaN, correctly propagating "unknown"
    /// when the fan just dropped out) before the window has filled.
    fn rpm_smoothed_now(&self, auto: &AutoState) -> f64 {
        let v: Vec<f64> = auto.fan_window.iter().copied().collect();
        tail_mean(&v, FAN_SMOOTH_N).unwrap_or_else(|| v.last().copied().unwrap_or(f64::NAN))
    }

    /// Loop-mode transition side effects: `Noted { mode }`, and the
    /// `Released` hand-off in both directions (design §2.5: caps released
    /// to stock on entering `Released`; re-engaging re-seeds `u` from the
    /// floors, §2.4).
    fn handle_mode_transition(&mut self, _s: &Sample, mode: LoopMode, effects: &mut Vec<Effect>) {
        let auto = self.auto.as_mut().expect("checked above");
        let from = auto.last_mode;
        if from == mode {
            return;
        }
        auto.last_mode = mode;
        effects.push(Effect::Noted {
            cause: mode_transition_cause(from, mode),
        });

        if mode == LoopMode::Released {
            // Caps released to stock (design §2.5); Mode::Auto itself is
            // untouched — the top-level controller mode stays Auto, only
            // the loop hands actuation back to stock while it has nothing
            // to close a loop on.
            if let Some(gpu) = self.guard.gpu.as_mut() {
                if let Err(e) = gpu.release() {
                    tracing::warn!("auto: Released GPU clock release failed: {e}");
                }
            }
            if let Some(cpu) = self.guard.cpu.as_ref()
                && let Err(e) = cpu.restore_stock()
            {
                tracing::warn!("auto: Released CPU stock restore failed: {e}");
            }
            self.status.cpu_limit_w = None;
            self.status.gpu_max_mhz = None;
            let auto = self.auto.as_mut().expect("checked above");
            auto.gpu_target_w = None;
            auto.gpu_verifier = None;
            // The next usable tick's budget bounds-setting re-seeds `u`
            // from the floors (§2.4) — see the `budget_seeded` gate in
            // `run_budget_and_allocate`. `ec_seeded` clears the same way so
            // a later re-engagement re-seeds `ec_avg` too (§2.6).
            auto.budget_seeded = false;
            auto.ec_seeded = false;
        }
    }

    /// Every-5-s budget step + allocation (design §3.2's data flow): the
    /// anti-windup halt, the PI step, `split_budget` (+ guard override +
    /// slew clamp), the CPU write/verdict.
    fn run_budget_and_allocate(
        &mut self,
        s: &Sample,
        decision: &Decision,
        guard_state: GuardState,
        effects: &mut Vec<Effect>,
        cause: &mut Option<&'static str>,
    ) {
        let lut = self.lut.as_ref().expect("checked by caller").clone();
        let gpu_floor_w = lut
            .watts_for_clock(self.config.gpu_floor_mhz)
            .unwrap_or(0.0);
        let cpu_floor_w = self.config.cpu_floor_w;
        let cpu_max_w = self.config.cpu_max_w;

        // Guard override (§2.8): while `gpu_hot`, the GPU's effective max
        // ratchets down from LAST tick's own cap toward its floor.
        let last_gpu_cap_w = self.auto.as_ref().expect("in auto").last_gpu_cap_w;
        let gpu_max_w = if guard_state.gpu_hot {
            gpu_share_override(last_gpu_cap_w, gpu_floor_w)
        } else {
            self.config.gpu_max_w
        };

        // Seed on the first tick of this engagement — auto entry, or
        // re-engaging from `Released` (design §2.4).
        if !self.auto.as_ref().expect("in auto").budget_seeded {
            let auto = self.auto.as_mut().expect("in auto");
            auto.budget.seed(cpu_floor_w + gpu_floor_w);
            auto.budget_seeded = true;
        }

        // Mode B's gain schedule (§2.4): resolved every tick from the
        // arbiter's current slope so it is always current by the time an
        // `Rpm`-kind step needs it, including on the very tick the loop
        // switches into RpmLoop. `None` (no resolved curve, `active:
        // false`/`absent` socket) applies the conservative `0.25x` clamp.
        {
            let auto = self.auto.as_mut().expect("in auto");
            auto.budget.scale_rpm_gain(decision.slope);
        }

        // This tick's loop error (design §2.4).
        let err_opt = self.compute_loop_error(decision.mode, decision.t_star);
        let err = err_opt.unwrap_or(LoopError::Temp { e_c: 0.0 });
        let error_sign = err_opt.map(loop_error_value).unwrap_or(0.0).signum();

        // Freeze priority: `Released` (hard hold) > an in-progress actuator
        // mismatch episode (hard hold, judged against the state as of
        // BEFORE this tick's own write — see `VerdictState::in_episode`'s
        // doc) > the demand-limited halt (directional).
        let actuator_hold = {
            let auto = self.auto.as_ref().expect("in auto");
            auto.cpu_verdict.in_episode() || auto.gpu_verdict.in_episode()
        };
        let freeze = if decision.mode == LoopMode::Released {
            Some(BudgetFreeze::Released)
        } else if actuator_hold {
            Some(BudgetFreeze::ActuatorMismatch)
        } else {
            let (last_cpu_cap_w, last_gpu_cap_w) = {
                let auto = self.auto.as_ref().expect("in auto");
                (auto.last_cpu_cap_w, auto.last_gpu_cap_w)
            };
            let cpu_axis = (s.cpu_pkg_w, last_cpu_cap_w, cpu_floor_w);
            let gpu_axis = (s.gpu_w, last_gpu_cap_w, gpu_floor_w);
            let auto = self.auto.as_mut().expect("in auto");
            let halted = auto.budget.set_demand_state(cpu_axis, gpu_axis, error_sign);
            halted.then_some(BudgetFreeze::DemandLimited)
        };

        let u = {
            let auto = self.auto.as_mut().expect("in auto");
            auto.budget.step(err, freeze)
        };
        self.status.budget_w = u;

        // Demand + split (design §3.1). The RAW split (pre-slew-clamp) is
        // what §2.4 means by "the cap `split_budget` actually produced" —
        // stored for NEXT tick's demand-limited judgement; the
        // slew-clamped/quantized `Allocator::step` output is the actual
        // command.
        let demand = allocator::demand(
            s,
            self.status.cpu_limit_w,
            self.auto.as_ref().expect("in auto").gpu_target_w,
            self.status.gpu_max_mhz,
        );
        let (raw_cpu, raw_gpu) =
            allocator::split_budget(u, demand, cpu_floor_w, gpu_floor_w, cpu_max_w, gpu_max_w);
        {
            let auto = self.auto.as_mut().expect("in auto");
            auto.last_cpu_cap_w = raw_cpu;
            auto.last_gpu_cap_w = raw_gpu;
        }
        let (cpu_w, gpu_w) = {
            let auto = self.auto.as_mut().expect("in auto");
            auto.allocator.step(&AllocInput {
                budget_w: u,
                demand,
                floors: cpu_floor_w,
                cpu_max_w,
                gpu_max_w,
                gpu_floor_w,
            })
        };
        {
            let auto = self.auto.as_mut().expect("in auto");
            auto.pid.set_target_w(gpu_w);
            auto.gpu_target_w = Some(gpu_w);
        }

        // CPU write + read-back verdict (design §2.9). A released (3-strike)
        // actuator has `status.cpu_limit_w == None`, so the `!=` check
        // below keeps forcing a fresh write every 5 s tick on its own —
        // exactly "the write plus read-back keeps running every reassert
        // period" (more often, if anything, never less).
        let suppress = self
            .auto
            .as_ref()
            .expect("in auto")
            .on_ac_suppress_until
            .is_some_and(|until| s.t_mono < until);
        let cpu_mw = (cpu_w * 1000.0).round() as u32;
        // Released: caps stay released to stock (design §2.5) — the
        // allocator still computes a hypothetical split for telemetry
        // (`AutoAllocated` below), but nothing is written; `Freeze::Released`
        // already held `u` above, so re-commanding here would just fight
        // the hand-off `handle_mode_transition` already performed.
        let need_write = decision.mode != LoopMode::Released
            && (self.status.cpu_limit_w != Some(cpu_w)
                || self.auto.as_ref().expect("in auto").cpu_verdict.released);
        if need_write {
            match self.guard.cpu.as_ref() {
                None => {
                    tracing::warn!("auto: no CPU actuator; allocation {cpu_w} W not applied");
                }
                Some(cpu) => {
                    let mut verdict = cpu.set_sustained_mw(cpu_mw);
                    if matches!(verdict, WriteVerdict::Mismatch { .. }) && !suppress {
                        // Re-read once before scoring (design §2.9).
                        verdict = cpu.set_sustained_mw(cpu_mw);
                    }
                    if let WriteVerdict::Verified(clamped_w) = verdict {
                        self.status.cpu_limit_w = Some(clamped_w);
                        self.stick_violations = 0;
                        effects.push(Effect::CpuSet(clamped_w));
                    }
                    let outcome = {
                        let auto = self.auto.as_mut().expect("in auto");
                        auto.cpu_verdict.observe(verdict, suppress)
                    };
                    if outcome == VerdictOutcome::Released {
                        if let Err(e) = cpu.restore_stock() {
                            tracing::warn!("auto: CPU stock restore on release failed: {e}");
                        }
                        self.status.cpu_limit_w = None;
                    }
                    self.apply_verdict_outcome(true, outcome, err_opt, effects, cause);
                }
            }
        }

        effects.push(Effect::AutoAllocated {
            demand_cpu: demand.cpu_starved,
            demand_gpu: demand.gpu_starved,
            cpu_w,
            gpu_w,
            mode: decision.mode,
            error: loop_error_value(err),
            budget_w: u,
            freeze: freeze.map(freeze_str),
        });
        cause.get_or_insert("auto:allocate");
    }

    /// GPU PI target, every sample, + `verify_lock` verdict (design §2.9,
    /// the shared rule applied identically to the GPU axis).
    fn run_gpu_pi(
        &mut self,
        s: &Sample,
        effects: &mut Vec<Effect>,
        cause: &mut Option<&'static str>,
    ) {
        if !s.gpu_w_valid || self.status.loop_mode == LoopMode::Released {
            // Released: caps stay released to stock (design §2.5) — see the
            // matching guard in `run_budget_and_allocate`.
            return;
        }
        let lut = self.lut.as_ref().expect("checked by caller").clone();
        let clock = {
            let auto = self.auto.as_mut().expect("in auto");
            auto.pid.update(s.gpu_w, &lut, self.config.gpu_floor_mhz)
        };
        let Some(clock) = clock else {
            return;
        };
        match self.guard.gpu.as_mut() {
            // Warned once at Auto entry, not here (1 Hz spam).
            None => {}
            Some(gpu) => match gpu.set_max_clock(clock) {
                Ok(()) => {
                    let applied = gpu.applied();
                    if applied != self.status.gpu_max_mhz {
                        self.status.gpu_max_mhz = applied;
                        effects.push(Effect::GpuSet(applied.unwrap_or(clock)));
                        cause.get_or_insert("auto:gpu_clock");
                    }
                    let locked = applied.unwrap_or(clock);
                    let suppress = self
                        .auto
                        .as_ref()
                        .expect("in auto")
                        .on_ac_suppress_until
                        .is_some_and(|until| s.t_mono < until);
                    {
                        let auto = self.auto.as_mut().expect("in auto");
                        if auto.gpu_verifier_mhz != Some(locked) {
                            auto.gpu_verifier = Some(GpuLockVerifier::new(locked));
                            auto.gpu_verifier_mhz = Some(locked);
                        }
                    }
                    let gpu_sm_mhz = s.gpu_sm_mhz.round() as u32;
                    let mut verdict = {
                        let auto = self.auto.as_mut().expect("in auto");
                        auto.gpu_verifier
                            .as_mut()
                            .expect("just set above")
                            .verify_lock(s.gpu_util_pct, gpu_sm_mhz)
                    };
                    if matches!(verdict, WriteVerdict::Mismatch { .. }) && !suppress {
                        let auto = self.auto.as_mut().expect("in auto");
                        verdict = auto
                            .gpu_verifier
                            .as_mut()
                            .expect("just set above")
                            .verify_lock(s.gpu_util_pct, gpu_sm_mhz);
                    }
                    let outcome = {
                        let auto = self.auto.as_mut().expect("in auto");
                        auto.gpu_verdict.observe(verdict, suppress)
                    };
                    if outcome == VerdictOutcome::Released {
                        if let Err(e) = gpu.release() {
                            tracing::warn!("auto: GPU release on verdict-release failed: {e}");
                        }
                        self.status.gpu_max_mhz = None;
                        let auto = self.auto.as_mut().expect("in auto");
                        auto.gpu_verifier = None;
                        auto.gpu_verifier_mhz = None;
                    }
                    let err_opt =
                        self.compute_loop_error(self.status.loop_mode, self.status.t_star_c);
                    self.apply_verdict_outcome(false, outcome, err_opt, effects, cause);
                }
                Err(e) => {
                    tracing::warn!("auto: GPU clock ({clock} MHz) failed: {e}");
                    // PI honesty: update() already committed `clock` as its
                    // rate-limit reference, but the hardware still holds
                    // the old lock (or none). Re-seed from what is
                    // actually applied so the next command rate-limits
                    // from hardware state, not from failed intent.
                    let applied = self.status.gpu_max_mhz;
                    let auto = self.auto.as_mut().expect("in auto");
                    auto.pid.seed_last_clock(applied);
                }
            },
        }
    }

    /// Applies one actuator's verdict-outcome side effects: a `Noted`
    /// telemetry line, an unfreeze resync on recovery, and the shared
    /// flags (design §2.9). `current_error` is this tick's loop error
    /// value (for the recovery resync — "the first `Verified` ... calls
    /// `resync_error`").
    fn apply_verdict_outcome(
        &mut self,
        is_cpu: bool,
        outcome: VerdictOutcome,
        current_error: Option<LoopError>,
        effects: &mut Vec<Effect>,
        cause: &mut Option<&'static str>,
    ) {
        let (recovered, mismatch, released, blind) = if is_cpu {
            (
                "auto:cpu_verdict_recovered",
                "auto:cpu_mismatch",
                "auto:cpu_released",
                "auto:cpu_readback_blind",
            )
        } else {
            (
                "auto:gpu_verdict_recovered",
                "auto:gpu_mismatch",
                "auto:gpu_released",
                "auto:gpu_readback_blind",
            )
        };
        match outcome {
            VerdictOutcome::Quiet => {}
            VerdictOutcome::Recovered => {
                effects.push(Effect::Noted { cause: recovered });
                cause.get_or_insert(recovered);
                if let Some(e) = current_error {
                    let auto = self.auto.as_mut().expect("in auto");
                    auto.budget.resync_error(loop_error_value(e));
                }
            }
            VerdictOutcome::Mismatch => {
                effects.push(Effect::Noted { cause: mismatch });
                cause.get_or_insert(mismatch);
            }
            VerdictOutcome::Released => {
                effects.push(Effect::Noted { cause: released });
                cause.get_or_insert(released);
            }
            VerdictOutcome::Blind => {
                effects.push(Effect::Noted { cause: blind });
                cause.get_or_insert(blind);
            }
        }
        self.sync_limit_not_sticking();
        self.sync_readback_blind();
    }

    /// Raises `LimitNotSticking` when either actuator is mid-episode
    /// (design §2.9). Never clears it: clearing is the RAPL stickiness
    /// watchdog's job (`on_sample`), which is patched to also require
    /// verdict state to be clear — a single owner for "add", a single
    /// owner for "remove", so the two mechanisms sharing this flag never
    /// fight each other.
    fn sync_limit_not_sticking(&mut self) {
        let trouble = self
            .auto
            .as_ref()
            .is_some_and(|a| a.cpu_verdict.in_episode() || a.gpu_verdict.in_episode());
        if trouble {
            self.add_flag(StatusFlag::LimitNotSticking);
        }
    }

    /// Fully syncs `ReadbackBlind` from verdict state both ways — nothing
    /// else touches this flag, so add/remove can both live here.
    fn sync_readback_blind(&mut self) {
        let blind = self
            .auto
            .as_ref()
            .is_some_and(|a| a.cpu_verdict.blind || a.gpu_verdict.blind);
        self.sync_bool_flag(StatusFlag::ReadbackBlind, blind);
    }

    /// One calibrating-mode sample: feed the runner, execute its effects,
    /// refresh the wizard progress in status.
    fn on_calib_sample(&mut self, s: &Sample) -> Vec<Effect> {
        let before = self.status.clone();
        let runner_effects = match self.calib.as_mut() {
            // CalibContext::default() until fw-fanctrl-loop-438 wires the
            // real arbiter/budget-integrator signals through.
            Some(runner) => runner.on_sample(s, &CalibContext::default()),
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
                "calib:skipped" => 5,
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
                // fw-fanctrl-loop-438 wires this through split_budget/the
                // integrator; until then the step test's requested power is
                // ignored (see the on_calib_sample call site's own note).
                RunnerEffect::SetBudget(_) => {}
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
                RunnerEffect::Fitted { gains, fitted_at } => {
                    tracing::info!("calib: step-test fitted {gains:?} at t_mono={fitted_at}");
                    raise(&mut cause, "calib:fitted");
                }
                RunnerEffect::Noted(reason) => {
                    tracing::info!("calib: step test skipped: {reason}");
                    raise(&mut cause, "calib:skipped");
                }
                RunnerEffect::SaveState(state) => {
                    self.lut = state.lut.clone();
                    self.calibrated_at = state.calibrated_at.clone();
                    if self.lut.is_some() {
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
        // ever re-command until a fresh (post-re-arm) Auto entry.
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

    /// Write the full calibration/loop state to the state file. Called from
    /// [`exit_auto_and_persist`](Self::exit_auto_and_persist) only — never
    /// per-update (no disk churn). Save failure is warned, not fatal: the
    /// in-memory state still carries the session.
    fn save_persisted_state(&self) {
        let state = PersistedState {
            lut: self.lut.clone(),
            calibrated_at: self.calibrated_at.clone(),
            loop_gains: self.loop_gains,
            duty_rpm_table: self.duty_rpm_table.clone(),
            warm_start: self.warm_start.clone(),
        };
        if let Err(e) = state.save(&self.state_path) {
            tracing::warn!(
                "auto: state save to {} failed: {e}",
                self.state_path.display()
            );
        }
    }

    /// Drop the Auto loop state and write the state file (a quit from Auto
    /// is covered because every exit path funnels through here). No-op when
    /// not in Auto: a Monitor/Manual session has nothing to drop.
    fn exit_auto_and_persist(&mut self) {
        if self.auto.take().is_some() {
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
        self.remove_flag(StatusFlag::TargetUnreachable);
        self.remove_flag(StatusFlag::LimitNotSticking);
        // fw-fanctrl-loop-j6s: the arbiter/guards/verdict flags are owned
        // entirely by `on_auto_sample`, which stops running the moment
        // `self.auto` drops here — without an explicit clear, a flag that
        // happened to be up at exit (e.g. `GpuHot` mid-episode) would stay
        // stuck forever, since nothing outside Auto ever touches it again.
        for flag in [
            StatusFlag::FanctrlLost,
            StatusFlag::EcMismatch,
            StatusFlag::SteepCurve,
            StatusFlag::CurveInvalid,
            StatusFlag::GpuHot,
            StatusFlag::NvmeHot,
            StatusFlag::ReadbackBlind,
        ] {
            self.remove_flag(flag);
        }
        self.status.loop_mode = LoopMode::default();
        self.status.t_star_c = None;
        self.status.ec_ma_c = None;
        self.status.ec_argmax = None;
        self.status.duty_cmd = None;
        self.status.snapped_rpm = 0.0;
        self.status.strategy = None;
        self.status.budget_w = 0.0;
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
    // One "main" Decision per batch (whatever claimed the cause first).
    let decision = |cause: &'static str, alloc: Option<(f64, f64, f64, f64)>| Record::Decision {
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
        t_star: status.t_star_c,
        budget_w: status.budget_w,
        freeze: None,
    };
    if cause.is_some() || !flagged.is_empty() {
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
                t.log(&decision(cause, auto_alloc));
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

    // --- Task 18/22: calibration integration ---

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

    /// A step-test settle-phase sample. The controller's `on_calib_sample`
    /// call site always hands the runner `CalibContext::default()` this
    /// task (fw-fanctrl-loop-438 wires the real arbiter/budget signals
    /// through) — `fanctrl_active` is therefore always false, so any drive
    /// through the controller settles never and times out at the 5-minute
    /// cap. That is exactly what these tests exercise: burner/actuator
    /// bookkeeping around a step test the controller cannot yet complete.
    fn settle_sample() -> Sample {
        Sample {
            cpu_pkg_w: 10.0,
            gpu_w: 5.0,
            gpu_w_valid: true,
            gpu_util_pct: 3.0,
            fan1_rpm: 3000.0,
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
                if calib.phase != "lut" || calib.step > i {
                    break;
                }
            }
        }
        let calib = ctl.status().calib.as_ref().expect("calibrating");
        assert_eq!(calib.phase, "step", "sweep must finish: {calib:?}");
    }

    /// Drive `settle_sample()`s through the controller until the step test
    /// gives up (5-minute settle cap under the always-default `CalibContext`)
    /// and calibration finishes.
    fn drive_step_test_to_skip(ctl: &mut Controller<&FakeRunner>) {
        for _ in 0..300 {
            ctl.on_sample(&settle_sample());
            if ctl.status().calib.is_none() {
                return;
            }
        }
        panic!("step test never concluded through the controller");
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
        assert_eq!(calib.phase, "lut");
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
    fn step_test_starts_the_burner_on_entry_and_stops_it_on_skip() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-actuators");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        // The burner starts unconditionally on step-test entry, before any
        // gate is ever checked (the ordering fact design §3.3 turns on).
        assert!(
            ctl.burner.is_some(),
            "burner must start as soon as the sweep hands off"
        );
        // SetBudget is ignored this task (fw-fanctrl-loop-438 wires it): no
        // CPU limit is ever commanded during the step test here.
        assert!(ryzenadj_calls(&runner).is_empty());

        drive_step_test_to_skip(&mut ctl);
        assert!(ctl.burner.is_none(), "burner must stop once the step skips");
        assert_eq!(ctl.status().mode, Mode::Monitor);

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn repeated_sweep_needs_load_nags_are_noted_for_telemetry() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-nag");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        // GPU idle throughout: the sweep's first clock never pins. First
        // nag flips needs_load: a StatusChanged Decision.
        let idle = Sample {
            gpu_util_pct: 5.0,
            gpu_sm_mhz: 300.0,
            gpu_w: 15.0,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            fan1_rpm: 1500.0,
            fan_valid: true,
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            ..Sample::default()
        };
        for _ in 0..9 {
            assert!(ctl.on_sample(&idle).is_empty());
        }
        let effects = ctl.on_sample(&idle);
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
            assert!(ctl.on_sample(&idle).is_empty());
        }
        let effects = ctl.on_sample(&idle);
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
        assert!(ctl.burner.is_some(), "burner running during the step test");
        for _ in 0..10 {
            ctl.on_sample(&settle_sample());
        }

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
        for _ in 0..10 {
            ctl.on_sample(&settle_sample()); // burner active mid-settle
        }

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
    fn full_calibration_persists_the_lut_with_no_gains_when_the_step_test_skips() {
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
        assert!(ctl.lut.is_none());

        ctl.on_command(Command::StartCalibration);
        drive_sweep(&mut ctl);
        drive_step_test_to_skip(&mut ctl);

        // Finished: back to Monitor, wizard gone, everything released.
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ctl.status().calib.is_none());
        assert!(ctl.burner.is_none());
        assert_eq!(ctl.status().cpu_limit_w, None);

        // The state file exists, parses and carries the LUT (PersistedState
        // no longer carries a model field, fw-fanctrl-loop-dsh); the
        // controller kept it, so Auto mode can start right away.
        // `loop_gains` stays None: with `CalibContext::default()` the step
        // test's settle gate never clears, so it skips and keeps defaults.
        let saved = PersistedState::load(&state_path);
        assert_eq!(saved.lut.expect("lut persisted").len(), 10);
        assert_eq!(saved.loop_gains, None);
        saved
            .calibrated_at
            .expect("calibrated_at set")
            .parse::<u64>()
            .expect("unix seconds");
        assert!(ctl.lut.is_some());

        fs::remove_dir_all(&dir).unwrap();
    }

    // --- Task 25: auto mode ---

    use crate::actuators::gpu::test_support::{FakeGpu, GpuCall};
    use crate::fanctrl::client::{FanctrlView, Freshness};
    use crate::sensors::ec::EcReading;
    use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};

    #[test]
    fn persisted_state_seeds_controller_lut() {
        let runner = FakeRunner::new();
        let mut lut = ClockWattsLut::new();
        lut.insert(2000, 60.0);
        let persisted = PersistedState {
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
    }

    // --- Task 12 (fw-fanctrl-loop-24s): auto mode with no learned thermal model ---
    //
    // The five-gate adaptation tier (the drift filter, the trust gate and
    // the command-quiet window), the fitted plant-inversion split and the
    // periodic model-snapshot Note are gone. These tests exercise what is
    // left: Auto entry now gates on the LUT alone, and the allocator's
    // split is stubbed degenerate, so a step can only ever hold at the
    // floor-raised last point (never explore above it) — see the
    // `on_auto_sample` comment at the stub's construction for why that is
    // the correct behavior with no model.

    /// Auto entry only needs the LUT now (no thermal model).
    fn calibrated() -> PersistedState {
        let mut lut = ClockWattsLut::new();
        lut.insert(1200, 30.0);
        lut.insert(2000, 60.0);
        lut.insert(2800, 100.0);
        PersistedState {
            lut: Some(lut),
            calibrated_at: None,
            ..PersistedState::default()
        }
    }

    /// Calibrated (LUT-only) controller with a CPU actuator AND a FakeGpu;
    /// also returns the GPU call-log handle. Mode starts at Monitor — the
    /// caller enters Auto explicitly.
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
    /// target. `cpu_pkg_w` stays 0 (RAPL-warmup semantics) so the
    /// stickiness watchdog stays quiet.
    fn busy_at(t: f64) -> Sample {
        Sample {
            fan_valid: true,
            fan1_rpm: 2500.0,
            fan2_rpm: 2400.0,
            gpu_w_valid: true,
            gpu_w: 10.0,
            ..sample_at(t)
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
    fn auto_entry_without_lut_flags_not_calibrated() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner); // uncalibrated

        let effects = ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Monitor, "must stay in Monitor");
        assert!(ctl.status().flags.contains(&StatusFlag::NotCalibrated));
        assert_eq!(status_changes(&effects), 1);
    }

    #[test]
    fn auto_entry_with_lut_only_engages_rpm_loop_and_moves_off_the_floor() {
        // fw-fanctrl-loop-j6s: the arbiter and budget are now wired. With
        // no fanctrl view but a valid fan reading, RpmLoop engages on the
        // very first tick (no entry hysteresis for the fallback loop,
        // design §2.5) and the integrator immediately starts moving off
        // the seeded floor sum — no more "stubbed, degenerate, held at the
        // floor forever" behavior.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Auto);

        let floor = ctl.status().cpu_floor_w;
        let floor_sum = floor + 30.0; // cpu_floor_w (15) + gpu_floor_w (30)
        let _ = ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.status().loop_mode, LoopMode::RpmLoop);
        // The scalar budget itself moved off the seeded floor sum this
        // very tick (RpmLoop's error is +530 RPM, calling for more) —
        // `auto_allocate_decision_carries_the_real_arbiter_fields` below
        // hand-derives the exact value (45.424); this test only needs
        // "moved", not the precise number, so a small movement suffices
        // and stays robust to a future gain-tuning change.
        assert!(
            ctl.status().budget_w > floor_sum,
            "budget must have moved off the seeded floor sum {floor_sum}: {}",
            ctl.status().budget_w
        );
    }

    #[test]
    fn auto_allocate_decision_carries_the_real_arbiter_fields() {
        // fw-fanctrl-loop-j6s: `AutoAllocated`'s mode/error/budget_w/freeze
        // fields now carry the real arbiter/budget output, hand-derived:
        // RpmLoop's target duty is `duty_for_rpm(3000)` = 36 (nearest of
        // the seeded points to the default 3000 RPM target), whose table
        // RPM is 3030; `rpm_smoothed` on the very first tick falls back to
        // the single fan reading in the window (`busy_at`'s 2500 RPM), so
        // `e_rpm = 3030 - 2500 = 530`. The budget seeds to
        // `cpu_floor_w + gpu_floor_w = 15 + 30 = 45`, then one velocity-PI
        // step at the `0.25x`-scheduled default RPM gains
        // (`kc = 0.0028 * 0.25 = 0.0007`, `ti = 35`, `PI_PERIOD_S = 5`)
        // adds `kc*(e_k - 0) + (kc*5/35)*e_k = 0.371 + 0.053 = 0.424`, i.e.
        // `u = 45.424` (unfrozen: neither actuator has a mismatch episode
        // yet and the demand-limited guard never fires with both axes'
        // last-tick caps still at 0).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&busy_at(0.0));
        let arbiter_fields = effects.iter().find_map(|e| match e {
            Effect::AutoAllocated {
                mode,
                error,
                budget_w,
                freeze,
                ..
            } => Some((*mode, *error, *budget_w, *freeze)),
            _ => None,
        });
        let (mode, error, budget_w, freeze) = arbiter_fields.expect("AutoAllocated effect");
        assert_eq!(mode, LoopMode::RpmLoop);
        assert!((error - 530.0).abs() < 1e-9, "got {error}");
        assert!((budget_w - 45.424).abs() < 1e-9, "got {budget_w}");
        assert_eq!(freeze, None);
    }

    #[test]
    fn some_gains_loaded_into_budget_none_uses_defaults() {
        // A custom `loop_gains` with 10x the default `kc_w_per_rpm` must
        // change the very first RpmLoop step's magnitude vs the
        // `LoopGains::default()` baseline (`auto_allocate_decision_carries_
        // the_real_arbiter_fields`'s 45.424) — proof `Budget::new` actually
        // received the persisted gains, not silently defaulted.
        let runner = FakeRunner::new();
        let gpu = FakeGpu::new();
        let custom_gains = LoopGains {
            kc_w_per_c: 0.22,
            ti_s: 35.0,
            kc_w_per_rpm: 0.028,
            ti_rpm_s: 35.0,
        };
        let persisted = PersistedState {
            loop_gains: Some(custom_gains),
            ..calibrated()
        };
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
            persisted,
            PathBuf::from("/nonexistent/state.json"),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&busy_at(0.0));
        let budget_w = effects
            .iter()
            .find_map(|e| match e {
                Effect::AutoAllocated { budget_w, .. } => Some(*budget_w),
                _ => None,
            })
            .expect("AutoAllocated effect");
        assert!(
            budget_w > 45.424 + 1.0,
            "10x kc_w_per_rpm must produce a materially larger step than the default-gains \
             baseline (45.424): got {budget_w}"
        );
    }

    #[test]
    fn the_integrator_floor_tracks_a_live_floor_change() {
        // Design §2.4: `lo = cpu_floor_w + lut.watts_at(gpu_floor_mhz)`,
        // recomputed every 5 s tick. Raising the CPU floor well above the
        // current `u` mid-session must pull the exposed budget up to the
        // new floor sum on the very next allocator tick — `Budget::
        // set_bounds` re-clamps `u` immediately, and `step`'s own `clamp`
        // enforces the same bound regardless of the PI increment that tick.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert!(
            ctl.status().budget_w < 70.0,
            "premise: budget starts near the low floor sum, got {}",
            ctl.status().budget_w
        );

        let gpu_floor_mhz = ctl.status().gpu_floor_mhz;
        ctl.on_command(Command::SetFloors {
            cpu_w: 40.0,
            gpu_mhz: gpu_floor_mhz,
        });
        let effects = ctl.on_sample(&busy_at(ALLOC_PERIOD_S));
        let budget_w = effects
            .iter()
            .find_map(|e| match e {
                Effect::AutoAllocated { budget_w, .. } => Some(*budget_w),
                _ => None,
            })
            .expect("AutoAllocated effect");
        assert!(
            budget_w >= 70.0,
            "budget's lower bound must track the raised floor (40 + 30 gpu floor = 70): {budget_w}"
        );
    }

    #[test]
    fn set_auto_false_releases_to_stock_and_resets_loop_state() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.status().mode, Mode::Auto);

        let effects = ctl.on_command(Command::SetAuto(false));
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert_eq!(ctl.status().gpu_max_mhz, None);
        assert!(ctl.auto.is_none());
        assert!(effects.contains(&Effect::Released));
    }

    // --- Task 28: thermal watchdog + emergency release ---

    /// Tctl over the 95 °C trip; every other sensor invalid/idle (the
    /// watchdog must trip on temperature alone).
    fn overheat_at(t: f64) -> Sample {
        Sample {
            t_mono: t,
            cpu_temp_c: 96.0,
            cpu_temp_valid: true,
            // fw-fanctrl-loop-j6s: the arbiter is now wired, so a
            // fan-invalid sample would legitimately drop the loop to
            // `Released` (a real, intended `Noted` transition) — this
            // fixture keeps a valid fan reading so the watchdog's own
            // 3-strike behavior stays what is under test here, undisturbed
            // by an unrelated mode transition.
            fan_valid: true,
            fan1_rpm: 2500.0,
            fan2_rpm: 2400.0,
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
        for _ in 0..10 {
            ctl.on_sample(&settle_sample()); // burner active mid-settle
        }
        assert!(ctl.burner.is_some(), "premise: burner running mid-settle");

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

    // --- Restored tier-independent tests (fw-fanctrl-loop-24s fix round 1) ---
    //
    // These were dropped alongside the adaptation-tier deletion even though
    // none of them exercise the deleted adaptation-tier machinery: they
    // cover the GPU watts->clock PI, command gating in Auto, plain Config
    // persistence, and a real historical crash regression. Ported unchanged
    // (or trivially, per the comments below) from the pre-refactor
    // controller.rs. The fan-slope-estimator test that used to open this
    // section is gone: fw-fanctrl-loop-zct's fix round 1
    // (fw-fanctrl-loop-64a5f50) deleted `fan_slope_rpm_s` itself as
    // genuinely dead code once the scalar-budget-split allocator dropped
    // its only caller (`AllocInput::fan_slope_rpm_s`).

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

        // SetFanTarget stays allowed: it retargets the fan target live.
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(ctl.status().fan_target_rpm, 2500.0);
        assert_eq!(ctl.status().mode, Mode::Auto);
    }

    #[test]
    fn fan_invalid_with_no_fanctrl_view_releases_to_stock() {
        // fw-fanctrl-loop-j6s: with the arbiter wired, a lost fan sensor
        // AND no fanctrl view is exactly design §2.5's `Released` row
        // ("nothing to close a loop on") — no allocator "freeze at the
        // floor" fallback survives; caps release to stock immediately and
        // stay there (no CPU write, no GPU PI command) until something
        // usable comes back.
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        let invalid_fan_at = |t: f64| Sample {
            fan_valid: false,
            ..busy_at(t)
        };
        let effects = ctl.on_sample(&invalid_fan_at(0.0));
        assert_eq!(ctl.status().loop_mode, LoopMode::Released);
        assert_eq!(ctl.status().cpu_limit_w, None);
        assert_eq!(ctl.status().gpu_max_mhz, None);
        assert!(
            ryzenadj_calls(&runner).is_empty(),
            "Released: no CPU write at all"
        );
        // No `Noted` here: `AutoState`'s `last_mode` already starts at
        // `Released` (its own `Default`), and this fixture never had a
        // fanctrl view either, so this is not a genuine transition — see
        // `mode_transitions_emit_noted` for the transition case itself.
        let _ = effects;

        // Stays released across later ticks (including a 10 s reassert
        // boundary) — nothing to reassert since nothing is applied.
        ctl.on_sample(&invalid_fan_at(5.0));
        let effects = ctl.on_sample(&invalid_fan_at(10.1));
        assert!(!has_reassert(&effects, "reassert"), "got {effects:?}");
        assert!(ryzenadj_calls(&runner).is_empty());
        assert!(
            gpu_sets(&gpu_calls).is_empty(),
            "GPU PI must not command anything while Released"
        );
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
        // Concretely: with the split stubbed degenerate (Task 12), the
        // allocator's first step always holds at its conservative 30 W
        // GPU target (unlike the pre-refactor fitted-model split, which
        // could climb off it on the very first step) — FF(30)=1200 + the
        // fresh-integrator correction lands the desired clock at 1600,
        // inside the ±105 MHz window from the seeded 1500, so the rate
        // limit doesn't even bind. (Corroborated by
        // `fan_invalid_freezes_allocator_but_pi_keeps_working`'s identical
        // unseeded first step, which lands on the same 1600 MHz.)
        assert_eq!(first_pi, 1600);
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

    // =====================================================================
    // fw-fanctrl-loop-j6s: controller loop integration — the wired arbiter,
    // budget, EcAverage, guards and shared actuator-verdict rule.
    // =====================================================================

    // ---- VerdictState: the shared actuator read-back rule (§2.9), unit
    // tested directly (it is a private struct in this module) rather than
    // via a scripted FakeRunner --info readback, which would need to
    // reproduce ryzenadj's exact table-parsing format on top of everything
    // else this test module already scripts.

    #[test]
    fn verdictstate_confirmed_mismatch_is_a_non_release_outcome_until_the_third() {
        let mut v = VerdictState::default();
        let mismatch = WriteVerdict::Mismatch {
            field: "slow",
            commanded: 20.0,
            read: 25.0,
        };
        assert_eq!(v.observe(mismatch, false), VerdictOutcome::Mismatch);
        assert!(v.in_episode());
        assert_eq!(v.observe(mismatch, false), VerdictOutcome::Mismatch);
        assert!(!v.released);
        assert_eq!(v.observe(mismatch, false), VerdictOutcome::Released);
        assert!(v.released);
        assert_eq!(v.mismatch_streak, 0, "strikes clear on release");
        assert!(
            v.in_episode(),
            "the flag stays held — in_episode still true"
        );
    }

    #[test]
    fn verdictstate_verified_after_a_release_recovers_and_clears_everything() {
        let mut v = VerdictState::default();
        let mismatch = WriteVerdict::Mismatch {
            field: "slow",
            commanded: 20.0,
            read: 25.0,
        };
        for _ in 0..3 {
            v.observe(mismatch, false);
        }
        assert!(v.released);
        let outcome = v.observe(WriteVerdict::Verified(20.0), false);
        assert_eq!(outcome, VerdictOutcome::Recovered);
        assert!(!v.released);
        assert!(!v.in_episode());
    }

    #[test]
    fn verdictstate_suppressed_mismatch_near_an_on_ac_edge_is_never_scored() {
        let mut v = VerdictState::default();
        let mismatch = WriteVerdict::Mismatch {
            field: "slow",
            commanded: 20.0,
            read: 25.0,
        };
        for _ in 0..10 {
            assert_eq!(v.observe(mismatch, true), VerdictOutcome::Quiet);
        }
        assert!(!v.in_episode());
    }

    #[test]
    fn verdictstate_six_consecutive_unreadable_raises_blind_cleared_by_verified() {
        let mut v = VerdictState::default();
        for _ in 0..5 {
            assert_eq!(
                v.observe(WriteVerdict::Unreadable, false),
                VerdictOutcome::Quiet
            );
        }
        assert_eq!(
            v.observe(WriteVerdict::Unreadable, false),
            VerdictOutcome::Blind
        );
        assert!(v.blind);
        // No freeze/strike from this — Unreadable never touches mismatch_streak.
        assert!(!v.in_episode());
        let outcome = v.observe(WriteVerdict::Verified(20.0), false);
        assert_eq!(outcome, VerdictOutcome::Recovered);
        assert!(!v.blind);
    }

    #[test]
    fn verdictstate_unreadable_and_unverifiable_are_non_events() {
        let mut v = VerdictState::default();
        assert_eq!(
            v.observe(WriteVerdict::Unverifiable, false),
            VerdictOutcome::Quiet
        );
        assert!(!v.in_episode());
        assert!(!v.blind);
        assert_eq!(v.mismatch_streak, 0);
        assert_eq!(v.unreadable_streak, 0);
    }

    // ---- Controller-level fixtures: a real fanctrl view + EC reading, for
    // the TempLoop scenarios (mirrors `mode.rs`'s own test fixtures). ----

    static EC_FIXTURE_COUNTER_C: AtomicU64 = AtomicU64::new(0);

    fn ec_reading_c(sensors: &[(&str, f64)]) -> EcReading {
        let n = EC_FIXTURE_COUNTER_C.fetch_add(1, AtomicOrdering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "bazerame-controller-test-{}-{n}",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        for (i, (label, c)) in sensors.iter().enumerate() {
            let idx = i + 1;
            fs::write(dir.join(format!("temp{idx}_label")), format!("{label}\n")).unwrap();
            fs::write(
                dir.join(format!("temp{idx}_input")),
                format!("{}\n", (*c * 1000.0) as i64),
            )
            .unwrap();
        }
        let reading = EcReading::read(&dir).expect("fixture should yield a reading");
        fs::remove_dir_all(&dir).unwrap();
        reading
    }

    /// quiet16-shaped curve (mirrors `mode.rs`'s own fixture): duty 31 at
    /// 75 °C. `target_duty` at the default 3000 RPM fan target snaps to 36
    /// (nearest table entry), which this curve does not have a tread for —
    /// `nearest_tread` resolves it down to 31, so `t_star` is 75.0.
    const TEMP_CURVE: &[(f64, u8)] = &[
        (0.0, 15),
        (55.0, 15),
        (65.0, 21),
        (75.0, 31),
        (82.0, 37),
        (88.0, 55),
        (95.0, 100),
    ];

    /// One TempLoop-eligible sample: a fresh, active fanctrl view (`temp`/
    /// `ma_temp`), an EC reading whose `cpu@4c` argmax matches `temp`
    /// exactly (a reconciliation match, not a mismatch) with ambient well
    /// below the feasibility margin, and a valid fan reading (so a
    /// core-condition failure never falls all the way to `Released`).
    fn temploop_sample(t: f64, temp: f64, ma_temp: f64, curve: &[(f64, u8)]) -> Sample {
        let view = FanctrlView {
            strategy: "quiet16".to_string(),
            active: true,
            speed_pct: 31,
            temperature: temp,
            ma_temperature: ma_temp,
            ma_interval: 60,
            curve: curve.to_vec(),
            observed_at: Instant::now(),
            all_observed_at: Some(Instant::now()),
        };
        let ec = ec_reading_c(&[("ambient_f75303@4d", 40.0), ("cpu@4c", temp)]);
        Sample {
            t_mono: t,
            fan_valid: true,
            fan1_rpm: 3000.0,
            fan2_rpm: 2950.0,
            ec: Some(ec),
            ec_valid: true,
            fanctrl: Some(view),
            fanctrl_freshness: Freshness::Fresh,
            fanctrl_view_changed: true,
            ..Sample::default()
        }
    }

    /// Drives `n` (>= 3) `temploop_sample`s through `ctl`, one per second
    /// starting at `t0`, and returns the last call's effects. `ENTRY_HYSTERESIS_TICKS`
    /// (3, `control::mode`) means TempLoop engages on the 3rd call.
    fn drive_temploop(ctl: &mut Controller<&FakeRunner>, t0: f64, n: u32) -> Vec<Effect> {
        let mut last = Vec::new();
        for i in 0..n {
            last = ctl.on_sample(&temploop_sample(t0 + f64::from(i), 75.0, 74.0, TEMP_CURVE));
        }
        last
    }

    #[test]
    fn temploop_tick_computes_t_star_minus_ma_and_moves_the_budget() {
        // temp=75 (EC argmax matches, reconciled+matched), ma_temp=74 -> a
        // clear positive T*-MA error once T* resolves. Expected T* is
        // computed via the SAME `Curve::t_star` the arbiter itself calls
        // (target_duty 36, TEMP_CURVE has no explicit breakpoint at 36 but
        // — unlike a genuinely SKIPPED duty — its interpolated segment
        // between 31@75C and 37@82C gives 36 its own real tread interval,
        // so `nearest_tread(36)` resolves to itself, not a fallback).
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let floor_sum = ctl.status().cpu_floor_w + 30.0;
        let target_duty = DutyRpmTable::default().duty_for_rpm(ctl.status().fan_target_rpm);
        let expected_t_star = Curve::from_points(TEMP_CURVE.to_vec())
            .expect("TEMP_CURVE is monotone")
            .t_star(target_duty)
            .expect("36 has a real tread on TEMP_CURVE");

        // TempLoop's own entry hysteresis (mode.rs's ENTRY_HYSTERESIS_TICKS
        // = 3) engages by the 3rd sample, but the budget/allocate step only
        // runs on the 5 s allocator cadence — drive through the NEXT due
        // tick (t=5, the 6th sample) so there is a fresh `AutoAllocated`
        // reflecting the already-engaged TempLoop.
        drive_temploop(&mut ctl, 0.0, 5);
        assert_eq!(ctl.status().loop_mode, LoopMode::TempLoop);
        assert_eq!(ctl.status().t_star_c, Some(expected_t_star));
        let effects = drive_temploop(&mut ctl, 5.0, 1);
        let (mode, error, budget_w, _) = effects
            .iter()
            .find_map(|e| match e {
                Effect::AutoAllocated {
                    mode,
                    error,
                    budget_w,
                    freeze,
                    ..
                } => Some((*mode, *error, *budget_w, *freeze)),
                _ => None,
            })
            .expect("AutoAllocated effect on the 5 s tick");
        assert_eq!(mode, LoopMode::TempLoop);
        // MA (seeded 74, boxcar-drifting toward the steady 75 argmax) stays
        // strictly below T* the whole run, so the error is always positive
        // — computing it exactly would mean re-deriving `EcAverage`'s own
        // off-by-one boxcar mean by hand; the sign and the budget's
        // movement off the floor are what this test is really after.
        assert!(error > 0.0, "T* - MA must still be positive: {error}");
        assert!(
            budget_w > floor_sum,
            "budget must have moved off the seeded floor {floor_sum}: {budget_w}"
        );
    }

    #[test]
    fn rejected_curve_raises_curve_invalid_and_falls_to_rpmloop() {
        // A descending duty segment: Curve::from_points rejects it (only
        // duty must be non-decreasing in file order — see `CurveError`).
        let bad_curve: &[(f64, u8)] = &[(50.0, 40), (60.0, 30)];
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&temploop_sample(0.0, 75.0, 74.0, bad_curve));
        assert!(ctl.status().flags.contains(&StatusFlag::CurveInvalid));
        assert_eq!(ctl.status().loop_mode, LoopMode::RpmLoop);
        // The 0.25x-scheduled RPM gain path (decision.slope is None with no
        // resolved curve, same as the no-curve-at-all case in
        // `auto_allocate_decision_carries_the_real_arbiter_fields`):
        // `temploop_sample`'s fan reading is 3000 RPM, target_duty 36 ->
        // 3030 RPM, so e_rpm = 30; kc = 0.0028*0.25 = 0.0007, ti=35,
        // PI_PERIOD_S=5: `+= kc*30 + (kc*5/35)*30 = 0.021+0.003 = 0.024`
        // over the 45 W floor seed.
        let budget_w = effects.iter().find_map(|e| match e {
            Effect::AutoAllocated { budget_w, .. } => Some(*budget_w),
            _ => None,
        });
        assert!(
            (budget_w.expect("AutoAllocated effect") - 45.024).abs() < 1e-9,
            "got {budget_w:?}"
        );
    }

    #[test]
    fn leaving_auto_clears_a_stuck_guard_flag() {
        // A flag raised by the (now-dropped) `Guards`/arbiter must not
        // survive `ReleaseAll` — nothing outside Auto ever touches these
        // flags again, so a missed clear here would stick forever.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&Sample {
            nvme_temp_c: Some(85.0),
            ..busy_at(0.0)
        });
        assert!(ctl.status().flags.contains(&StatusFlag::NvmeHot), "premise");

        ctl.on_command(Command::ReleaseAll);
        assert!(!ctl.status().flags.contains(&StatusFlag::NvmeHot));
        assert_eq!(ctl.status().loop_mode, LoopMode::default());
    }

    #[test]
    fn nvme_hot_tick_raises_the_flag_and_leaves_the_budget_unchanged() {
        // Two otherwise-identical RpmLoop sessions, one with an NVMe-hot
        // reading on every tick, one without: the budget trajectory must
        // be bit-for-bit identical (NVMe is reporting-only, §2.8) while
        // the hot session alone raises the flag.
        let cold_runner = FakeRunner::new();
        let (mut cold, _g1) = auto_controller_no_profile(&cold_runner);
        cold.on_command(Command::SetAuto(true));
        let hot_runner = FakeRunner::new();
        let (mut hot, _g2) = auto_controller_no_profile(&hot_runner);
        hot.on_command(Command::SetAuto(true));

        let mut cold_budget = 0.0;
        let mut hot_budget = 0.0;
        for i in 0..3 {
            let t = f64::from(i) * ALLOC_PERIOD_S;
            let ce = cold.on_sample(&busy_at(t));
            let he = hot.on_sample(&Sample {
                nvme_temp_c: Some(85.0),
                ..busy_at(t)
            });
            if let Some(b) = ce.iter().find_map(|e| match e {
                Effect::AutoAllocated { budget_w, .. } => Some(*budget_w),
                _ => None,
            }) {
                cold_budget = b;
            }
            if let Some(b) = he.iter().find_map(|e| match e {
                Effect::AutoAllocated { budget_w, .. } => Some(*budget_w),
                _ => None,
            }) {
                hot_budget = b;
            }
        }
        assert!(!cold.status().flags.contains(&StatusFlag::NvmeHot));
        assert!(hot.status().flags.contains(&StatusFlag::NvmeHot));
        assert_eq!(
            cold_budget, hot_budget,
            "NVMe HOT must not move the budget at all (reporting-only, §2.8)"
        );
        assert_eq!(cold.status().loop_mode, hot.status().loop_mode);
    }

    #[test]
    fn fan_dropout_clears_fan_valid_and_drops_rpmloop_within_one_sample() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.status().loop_mode, LoopMode::RpmLoop);

        let dropout = Sample {
            fan_valid: false,
            ..busy_at(ALLOC_PERIOD_S)
        };
        ctl.on_sample(&dropout);
        assert_eq!(
            ctl.status().loop_mode,
            LoopMode::Released,
            "a single fan-invalid sample must drop RpmLoop within one window"
        );
    }

    #[test]
    fn reconciliation_is_scored_on_the_1hz_sample_carrying_the_view_not_the_5s_tick() {
        // A persistent EC/view disagreement (argmax steady at 90, never
        // ramping — so the slope-based skip guard never suppresses
        // scoring) latches EC MISMATCH on its 3rd consecutive scored view,
        // at t=2 — one tick before the next 5 s allocator boundary (t=5).
        // If reconciliation were only scored on the 5 s tick, none of
        // t=0,1,2 would ever be scored at all inside this window.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        for t in [0.0, 1.0, 2.0] {
            let mut s = temploop_sample(t, 75.0, 74.0, TEMP_CURVE);
            s.ec = Some(ec_reading_c(&[
                ("ambient_f75303@4d", 40.0),
                ("cpu@4c", 90.0),
            ]));
            ctl.on_sample(&s);
        }
        assert!(
            ctl.status().flags.contains(&StatusFlag::EcMismatch),
            "EC MISMATCH must latch from the off-5s-tick samples alone"
        );
    }

    #[test]
    fn reengaging_from_released_reseeds_the_budget_without_a_step() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.status().loop_mode, LoopMode::RpmLoop);

        // Drop to Released (fan invalid), then come back.
        ctl.on_sample(&Sample {
            fan_valid: false,
            ..busy_at(ALLOC_PERIOD_S)
        });
        assert_eq!(ctl.status().loop_mode, LoopMode::Released);
        assert_eq!(ctl.status().cpu_limit_w, None, "caps released to stock");

        let effects = ctl.on_sample(&busy_at(2.0 * ALLOC_PERIOD_S));
        assert_eq!(ctl.status().loop_mode, LoopMode::RpmLoop);
        let budget_w = effects
            .iter()
            .find_map(|e| match e {
                Effect::AutoAllocated { budget_w, .. } => Some(*budget_w),
                _ => None,
            })
            .expect("AutoAllocated on re-engagement");
        // Re-seeded from the floors (45.0) — NOT a fresh integrator: this is
        // the SAME `Budget` instance, so leaving the `Released` freeze also
        // triggers `Budget::step`'s own generic "leaving any freeze" resync
        // (`e_prev` reset to this tick's own error), meaning this step
        // carries only the INTEGRAL term, no proportional kick — literally
        // "re-engages without a step": `kc=0.0007, e=530 -> (kc*5/35)*530 =
        // 0.053`, i.e. `u = 45.053`, not 45.424 (which is what a brand new,
        // never-stepped integrator would produce from the same seed+error —
        // see `auto_allocate_decision_carries_the_real_arbiter_fields`).
        assert!(
            (budget_w - 45.053).abs() < 1e-9,
            "re-engagement must reseed from the floors with no kick, not resume from the old u: {budget_w}"
        );
    }

    #[test]
    fn mode_transitions_emit_noted() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&busy_at(0.0));
        assert!(
            effects.iter().any(|e| matches!(
                e,
                Effect::Noted { cause } if *cause == "mode:released->rpmloop"
            )),
            "got {effects:?}"
        );
    }

    #[test]
    fn resumed_sample_clears_the_ec_boxcar() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        drive_temploop(&mut ctl, 0.0, 3);
        assert!(ctl.status().ec_ma_c.is_some(), "premise: ec_ma seeded");

        let resumed = Sample {
            resumed: true,
            ..temploop_sample(100.0, 75.0, 74.0, TEMP_CURVE)
        };
        ctl.on_sample(&resumed);
        // The resumed sample itself immediately re-seeds ec_avg from THIS
        // sample's view.ma_temperature (the "re-engagement" seeding rule,
        // §2.6) rather than leaving it cleared to None — so the visible
        // post-resume state is the fresh seed, not a gap. What this proves
        // is that the pre-resume boxcar contents (74.0's accumulated
        // history) were discarded rather than carried through the clear:
        // the resumed sample's own EcAverage instance was freshly
        // constructed this tick (see the `resumed` handling in `on_sample`).
        assert_eq!(ctl.status().ec_ma_c, Some(74.0));
    }

    #[test]
    fn demand_limited_anti_windup_eventually_holds_an_idle_budget_off_the_ceiling() {
        // A long idle run (near-zero draw on both axes, but a target that
        // keeps calling for more heat) must not wind the budget up to the
        // hard ceiling — the demand-limited halt (§2.4's decided rule)
        // must eventually engage and hold it, the same qualitative property
        // `spike_antiwindup.rs`'s scenario 1 measures at the `Budget` level,
        // replayed here through the full controller wiring.
        let runner = FakeRunner::new();
        let config = Config {
            // A very high fan target keeps RpmLoop's error positive
            // (calling for more) for the whole run.
            fan_target_rpm: 6900.0,
            ..Config::default()
        };
        let (mut ctl, _gpu) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            config,
        );
        ctl.on_command(Command::SetAuto(true));
        let hi = ctl.status().cpu_max_w + ctl.status().gpu_max_w;

        let mut froze = false;
        for i in 0..200 {
            let t = f64::from(i) * ALLOC_PERIOD_S;
            // fan_valid, but near-zero CPU/GPU draw and no NVML/RAPL demand
            // signal — an idle machine whose fans nonetheless read low
            // (nothing to close the RPM loop's error).
            let s = Sample {
                cpu_pkg_w: 0.1,
                gpu_w: 0.1,
                gpu_w_valid: true,
                ..busy_at(t)
            };
            let effects = ctl.on_sample(&s);
            if effects.iter().any(|e| {
                matches!(
                    e,
                    Effect::AutoAllocated {
                        freeze: Some("demand_limited"),
                        ..
                    }
                )
            }) {
                froze = true;
            }
        }
        assert!(froze, "demand-limited halt never engaged over 200 ticks");
        assert!(
            ctl.status().budget_w < hi,
            "budget must not have wound all the way to the ceiling: {} >= {hi}",
            ctl.status().budget_w
        );
    }
}

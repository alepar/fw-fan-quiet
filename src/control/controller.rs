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
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use crate::actuators::WriteVerdict;
use crate::actuators::cmd::Runner;
use crate::actuators::gpu::{GpuCommandEvidence, GpuLockVerifier};
use crate::actuators::guard::RestoreGuard;
use crate::calib::burner::Burner;
use crate::calib::runner::{PerDeviceCalibRunner as CalibRunner, RunnerEffect};
use crate::calib::step::PerDeviceCalibContext;
use crate::config::Config;
use crate::control::device_loop::{
    ActuatorState, DeviceDecision, DeviceLoop, Hold, Mhz, TickInput, W, default_gains,
};
use crate::control::guards::{
    CPU_MAX_RATCHET_DOWN_RATE_W, GPU_MAX_RATCHET_DOWN_RATE_MHZ, Guards, MaxRatchet,
};
use crate::control::tstar::{
    Device as TStarDevice, EntrySeed, HeldDeviceInput, SensorClass, SensorReading, TStarInput,
    TStarSource,
};
use crate::control::watchdog::{ThermalWatchdog, Trip};
use crate::event::Event;
use crate::fanctrl::client::Freshness;
use crate::fanctrl::curve::Curve;
use crate::fanctrl::table::DutyRpmTable;
use crate::sensors::ec::{EcGroup, EcReplica, ReconciliationObservation};
use crate::state::{PersistedState, TStarSeed, WarmStartEntry, warm_start_key};
use crate::telemetry::{self, Record, Telemetry};
use crate::types::{Sample, TelemetryFlag};

/// UI-facing calibration progress, re-exported so the view/model layers name
/// it without reaching into `calib::`.
pub use crate::calib::runner::CalibProgress as CalibProgressLite;
use crate::calib::runner::{CalibGainChange, CalibOutcome};

/// Reapply active limits at least this often (defends against PPD/tuned
/// clobbering the ryzenadj limits behind our back; design §3).
const REASSERT_PERIOD_S: f64 = 10.0;
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
/// Fan target clamp range (RPM); the Auto controller consumes the
/// target live via `status.fan_target_rpm`.
const FAN_TARGET_MIN_RPM: f64 = 1000.0;
const FAN_TARGET_MAX_RPM: f64 = 7000.0;
/// Startup fan target (RPM). Shared with the UI model so the controller's
/// echoed status and the displayed default can never diverge.
pub const DEFAULT_FAN_TARGET_RPM: f64 = 3000.0;
/// Tail-window size (samples, 1 Hz) `rpm_smoothed` averages over — design
/// §3.2's data flow names `FAN_SMOOTH_N` without pinning a value; short
/// enough that Held's error tracks a real fan-speed change within a few
/// seconds, long enough to reject single-sample tach noise.
const FAN_SMOOTH_N: usize = 5;
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
/// A fresh socket view is not a fair reconciliation input while the local
/// replica is still moving this quickly.
const RECONCILIATION_SKIP_SLOPE_C_PER_S: f64 = 0.5;
/// A view captured this far before its carrying sample is likewise not fair
/// evidence for reconciliation.
const RECONCILIATION_SKIP_GAP_S: f64 = 2.0;
/// Steady-window length for passive warm-start/refinement (design §2.3):
/// "population stdev of the smoothed RPM series < 60 over >= 40 s", at the
/// 1 Hz sample rate.
const STEADY_WINDOW_N: usize = 40;
/// Population-stdev ceiling (RPM) the steady window's `rpm_smoothed` series
/// must stay under to count as settled (design §2.3).
const STEADY_WINDOW_STDEV_MAX_RPM: f64 = 60.0;

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
    /// change, and consumed live by the Auto controller.
    SetFanTarget(f64),
    /// Safety floors (CPU sustained watts, GPU max-clock MHz), carrying BOTH
    /// current values (the UI model steps them locally). Sanitized, echoed
    /// in status, persisted to config on change; the Auto controller/PI
    /// consume them on their next step. Rejected only while Calibrating —
    /// floors are safety config, not actuation, so they are allowed in every
    /// other mode and (like SetFanTarget) pass the emergency acknowledge
    /// gate without consuming the acknowledge.
    SetFloors { cpu_w: f64, gpu_mhz: u32 },
    /// Enter/leave the closed-loop Auto mode. Explicit bool (not a toggle) so
    /// a queued duplicate keypress can never flip the mode back unnoticed.
    SetAuto(bool),
    /// Begin guided calibration from Monitor, or suspend a live Auto session.
    StartCalibration,
    /// Abort calibration, restore its held pair, and return to its origin mode.
    AbortCalibration,
    /// Dismiss the retained result without touching actuation.
    DismissCalibrationOutcome,
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
    /// The closed loop owns actuation (two DeviceLoops); manual setters
    /// are rejected, but the reassert/stickiness/resume machinery stays
    /// active (it keys off the applied CPU and GPU caps in status).
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

/// Coarse severity tier for a [`StatusFlag`], ordered loudest-first so a
/// derived `Ord`/`PartialOrd` sorts a flag list severity-first (declaration
/// order IS the ranking: `Critical < Warning < Info`).
#[cfg(test)]
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
    /// Fitted gains are absent for the active strategy and interval.
    NotCalibrated,
    /// At least one device cannot reach the requested target at a bound.
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
    /// The fw-fanctrl socket is absent or stale; the shared source holds.
    FanctrlLost,
    /// The controller's EC replica disagrees with fw-fanctrl's own `print
    /// all` view for 3 consecutive scored views; Curve is quarantined until
    /// 3 consecutive views agree again.
    EcMismatch,
    /// `slope_at(T*) > 2 %/°C` (design §2.7): the loop runs, but the
    /// operating point sits on a steep segment of the fw-fanctrl curve.
    /// Informational only.
    SteepCurve,
    /// The active fw-fanctrl curve is invalid and cannot supply T*.
    CurveInvalid,
    /// dGPU at/over its hot threshold (design §2.8, default 88 °C, flag
    /// exit 86 °C): the GPU maximum ratchets toward its configured floor;
    /// maximum recovery uses the stricter 84 °C gate.
    GpuHot,
    /// The NVMe `Composite` sensor is at/over its hot threshold (design
    /// §2.8, default 80 °C). Reporting only — no control action.
    NvmeHot,
    /// Six consecutive `Unreadable`/`Unverifiable` actuator read-backs
    /// (design §2.9): informational, cleared by the next `Verified`.
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
/// defaulting). Called only from its own tests: `ui/view.rs` ships its own
/// ranking/styling, deliberately diverging on `NvmeHot`.
#[cfg(test)]
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
        StatusFlag::NotCalibrated => Severity::Info,
        StatusFlag::ReadbackBlind => Severity::Info,
        StatusFlag::SteepCurve => Severity::Info,
        StatusFlag::NvmeHot => Severity::Warning,
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
    /// Stored fan target (RPM); the Auto controller consumes it live.
    pub fan_target_rpm: f64,
    /// CPU sustained-watts floor (config, live-editable via SetFloors).
    pub cpu_floor_w: f64,
    /// GPU max-clock floor in MHz (config, live-editable via SetFloors).
    pub gpu_floor_mhz: u32,
    /// CPU operating max (watts), shared with the actuator and UI scale.
    pub cpu_max_w: f64,
    /// Currently active flags.
    pub flags: Vec<StatusFlag>,
    /// Calibration wizard progress; Some exactly while Calibrating.
    pub calib: Option<CalibProgressLite>,
    /// Last calibration outcome, retained after the live wizard closes.
    pub calib_outcome: Option<CalibOutcome>,
    /// Shared source target temperature (°C), present while one is usable.
    pub t_star_c: Option<f64>,
    /// The controller's raw reconciliation moving mean (design §2.6),
    /// Some only while an Auto session has observed at least one sample.
    pub ec_ma_c: Option<f64>,
    /// Name of the sensor currently driving the fw-fanctrl argmax, Some
    /// while a socket view identifies one.
    pub ec_argmax: Option<String>,
    /// Last commanded fw-fanctrl duty step, when Curve supplies T*.
    pub duty_cmd: Option<u8>,
    /// The DutyRpmTable-snapped fan RPM the last commanded duty implies;
    /// 0.0 outside Curve.
    pub snapped_rpm: f64,
    /// Name of the fw-fanctrl curve/strategy currently in force, Some only
    /// once the loop has selected one.
    pub strategy: Option<String>,
    /// Shared T* source state for telemetry and the TUI. `None` until the
    /// per-device controller wiring supplies a live source output.
    pub tstar_state: Option<crate::types::TelemetryTStarState>,
    /// Last CPU DeviceLoop decision with its selected gains source.
    pub cpu: Option<crate::types::TelemetryDevice>,
    /// Last GPU DeviceLoop decision with its selected gains source.
    pub gpu: Option<crate::types::TelemetryDevice>,
    /// Labelled schema-v3 diagnostics emitted alongside simple status flags.
    /// The controller wiring fills this from T*/EC/device-loop diagnostics.
    pub telemetry_flags: Vec<crate::types::TelemetryFlag>,
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
            flags: Vec::new(),
            calib: None,
            calib_outcome: None,
            t_star_c: None,
            ec_ma_c: None,
            ec_argmax: None,
            duty_cmd: None,
            snapped_rpm: 0.0,
            strategy: None,
            tstar_state: None,
            cpu: None,
            gpu: None,
            telemetry_flags: Vec::new(),
        }
    }
}

/// What one `on_sample`/`on_command` call did — consumed by the thread shell
/// (channel sends + telemetry) and asserted on directly in tests.
#[derive(Clone, Debug, PartialEq)]
pub enum Effect {
    /// Sample-local diagnostic snapshot, captured before terminal effects drop the runner.
    Calibration {
        diagnostics: Box<crate::calib::step::CalibrationDiagnostics>,
        gpu_util_pct: f64,
        view_fresh: bool,
        view_changed: bool,
        reconciliation_ma_c: Option<f64>,
        socket_ma_c: Option<f64>,
    },
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
    /// the user-visible status is unchanged (for example a calibration note).
    Noted { cause: &'static str },
    /// A status flag transitioned (emitted on every genuine add/remove);
    /// the shell mirrors it into a standalone telemetry
    /// `Record::Flag` line IN ADDITION to the Decision record (whose
    /// `flags` field carries the full post-transition list) — offline
    /// analysis gets a greppable per-flag transition stream.
    Flagged { flag: &'static str, active: bool },
    /// Hardware restored; the thread shell must exit its loop.
    Quit,
}

/// Auto-mode loop state; Some exactly while `Mode::Auto`. Dropped whole on
/// exit, so re-entry always starts from a fresh PI (integrator cleared) and
/// Default EC replica boxcar interval before the first fw-fanctrl view has
/// ever been observed this session (§Facts: 60 on both live curves).
const DEFAULT_MA_INTERVAL: usize = 60;

/// Shared actuator read-back verdict state machine (design §2.9): one
/// instance per actuator (CPU, GPU). The caller re-reads a candidate
/// `Mismatch` once itself (re-invoking the write for CPU; re-scoring
/// `verify_lock` against the same sample for GPU, which has no second NVML
/// reading available within one 1 Hz tick) and feeds THIS method only the
/// confirmed, post-re-read verdict — [`VerdictState`] itself never sees the
/// unconfirmed first read. A confirmed `Mismatch` sets `LimitNotSticking` +
/// a DeviceLoop mismatch hold plus an immediate reassert on the same tick;
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
    /// known, to hold the affected DeviceLoop before its next step.
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
    replica: EcReplica,
    #[cfg(test)]
    reconciliation_scored_count: u64,
    tstar: TStarSource,
    cpu_loop: DeviceLoop<W>,
    gpu_loop: DeviceLoop<Mhz>,
    cpu_ratchet: MaxRatchet<W>,
    gpu_ratchet: MaxRatchet<Mhz>,
    cpu_draw_window: std::collections::VecDeque<f64>,
    last_sample_t_mono: Option<f64>,
    prior_cpu_hold: Hold,
    prior_gpu_hold: Hold,
    cpu_actuator_state: ActuatorState,
    gpu_actuator_state: ActuatorState,
    last_cpu_write_t_mono: Option<f64>,
    last_gpu_write_t_mono: Option<f64>,
    cpu_gains_source: crate::types::GainsSource,
    gpu_gains_source: crate::types::GainsSource,
    cpu_hot_streak: u8,
    cpu_hot: bool,
    cpu_entry_seeded: bool,
    gpu_entry_seeded: bool,
    cpu_shadow_entry_seeded: bool,
    gpu_shadow_entry_seeded: bool,
    cpu_restore_thermal_pending: bool,
    gpu_restore_thermal_pending: bool,
    /// Choose the entry seed once on the first sample: a qualified fresh
    /// strategy seed when available, otherwise the current device groups.
    source_initialised: bool,
    /// The first fresh socket view separately seeds reconciliation history;
    /// it never reconstructs an already-running TStarSource.
    view_initialised: bool,
    /// Fan-RPM window feeding `rpm_smoothed` (design §3.2's data flow);
    /// fan-invalid samples land as NaN (the charts/steady.rs convention) so
    /// a tail spanning a sensor outage is never mistaken for settled
    /// evidence.
    fan_window: std::collections::VecDeque<f64>,
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
    /// Successful GPU commands, ordered by completion on the sample's
    /// monotonic clock.  A sample is verified before its own write, against
    /// the latest entry that completed before acquisition.
    gpu_commands: std::collections::VecDeque<GpuCommandEvidence>,
    gpu_command_generation: u64,
    /// `s.on_ac` as of the previous sample, to detect an edge.
    last_on_ac: Option<bool>,
    /// `t_mono` until which a candidate actuator `Mismatch` is suppressed
    /// (design §2.9: 3 ticks after an `on_ac` edge).
    on_ac_suppress_until: Option<f64>,
    /// Steady-window accumulator for passive warm-start/refinement (design
    /// §2.3): `rpm_smoothed` values pushed while every steady-window gate
    /// (active, `speed_pct == target_duty`, no guard override, `u` off both
    /// bounds) holds this tick; cleared on any gate miss and whenever
    /// `steady_key` changes.
    steady_window: std::collections::VecDeque<f64>,
    /// The warm-start key `steady_window`'s accumulated samples belong to.
    /// A change (strategy edit, snapped-duty change, AC edge) clears the
    /// window — the re-key itself never re-seeds `u` (§2.4); it only
    /// changes which key the *next* steady window records into.
    steady_key: Option<String>,
    refinement_window: std::collections::VecDeque<f64>,
    refinement_key: Option<String>,
}

impl AutoState {
    /// `gpu_hot_c`/`nvme_hot_c` come from the live `Config` (integration
    /// sweep, fw-fanctrl-loop-eb9.17): this constructor used to hard-code
    /// `Guards::new(GPU_HOT_C_DEFAULT, NVME_HOT_C_DEFAULT)`, so an edited
    /// `gpu_hot_c`/`nvme_hot_c` in `config.toml` had zero effect on the
    /// actual guard thresholds — the two keys round-tripped through
    /// `Config::load`/`save` and appeared on `ControlStatus`/the config
    /// fixture tests, but the value driving `Guards::step`'s hysteresis was
    /// always the compiled-in default, invisible on this machine only
    /// because that default equals the shipped default.
    fn new(config: &Config) -> Self {
        let cpu_gains = config
            .cpu_gains
            .unwrap_or_else(|| default_gains::<W>(DEFAULT_MA_INTERVAL as u32));
        let gpu_gains = config
            .gpu_gains
            .unwrap_or_else(|| default_gains::<Mhz>(DEFAULT_MA_INTERVAL as u32));
        Self {
            replica: EcReplica::new(DEFAULT_MA_INTERVAL),
            #[cfg(test)]
            reconciliation_scored_count: 0,
            tstar: TStarSource::new(EntrySeed::Fallback(config.cpu_hot_c - 2.0)),
            cpu_loop: DeviceLoop::new(cpu_gains),
            gpu_loop: DeviceLoop::new(gpu_gains),
            cpu_ratchet: MaxRatchet::new(
                config.cpu_floor_w,
                config.cpu_max_w,
                config.cpu_max_w,
                CPU_MAX_RATCHET_DOWN_RATE_W,
                config.cpu_hot_c - 5.0,
            ),
            gpu_ratchet: MaxRatchet::new(
                f64::from(config.gpu_floor_mhz),
                f64::from(config.gpu_max_mhz),
                f64::from(config.gpu_max_mhz),
                GPU_MAX_RATCHET_DOWN_RATE_MHZ,
                config.gpu_hot_c - 4.0,
            ),
            cpu_draw_window: std::collections::VecDeque::new(),
            last_sample_t_mono: None,
            prior_cpu_hold: Hold::None,
            prior_gpu_hold: Hold::None,
            cpu_actuator_state: ActuatorState::Unverifiable,
            gpu_actuator_state: ActuatorState::Unverifiable,
            last_cpu_write_t_mono: None,
            last_gpu_write_t_mono: None,
            cpu_gains_source: if config.cpu_gains.is_some() {
                crate::types::GainsSource::Config
            } else {
                crate::types::GainsSource::Default
            },
            gpu_gains_source: if config.gpu_gains.is_some() {
                crate::types::GainsSource::Config
            } else {
                crate::types::GainsSource::Default
            },
            cpu_hot_streak: 0,
            cpu_hot: false,
            cpu_entry_seeded: false,
            gpu_entry_seeded: false,
            cpu_shadow_entry_seeded: false,
            gpu_shadow_entry_seeded: false,
            cpu_restore_thermal_pending: false,
            gpu_restore_thermal_pending: false,
            source_initialised: false,
            view_initialised: false,
            fan_window: std::collections::VecDeque::new(),
            gpu_verifier_mhz: None,
            ec_slope_window: std::collections::VecDeque::new(),
            guards: Guards::new(config.gpu_hot_c, config.nvme_hot_c),
            cpu_verdict: VerdictState::default(),
            gpu_verdict: VerdictState::default(),
            gpu_verifier: None,
            gpu_commands: std::collections::VecDeque::new(),
            gpu_command_generation: 0,
            last_on_ac: None,
            on_ac_suppress_until: None,
            steady_window: std::collections::VecDeque::new(),
            steady_key: None,
            refinement_window: std::collections::VecDeque::new(),
            refinement_key: None,
        }
    }
}

/// Test-only snapshot of internal safety state that is deliberately absent
/// from runtime status and telemetry.
#[cfg(test)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ControllerDiagnostics {
    pub(crate) cpu_guard_max: f64,
    pub(crate) gpu_guard_max: f64,
    pub(crate) cpu_actuator: ActuatorState,
    pub(crate) gpu_actuator: ActuatorState,
    pub(crate) cpu_mismatch_strikes: u8,
    pub(crate) gpu_mismatch_strikes: u8,
    pub(crate) cpu_released: bool,
    pub(crate) gpu_released: bool,
    pub(crate) reconciliation_ma_c: Option<f64>,
    pub(crate) reconciliation_ready: bool,
    pub(crate) reconciliation_ever_scored: bool,
    pub(crate) reconciliation_mismatch: bool,
    pub(crate) reconciliation_scored_count: u64,
}

/// Testable controller core. Owns the actuators through `RestoreGuard`, so
/// hardware is restored even if the thread shell exits abnormally.
pub struct Controller<R: Runner> {
    guard: RestoreGuard<R>,
    completion_clock: Box<dyn Fn() -> Instant + Send>,
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
    /// Wall-clock stamp of the loaded calibration, carried so an Auto-exit
    /// state write preserves it (only a finished calibration sets it).
    calibrated_at: Option<String>,
    /// Duty<->RPM lookup (design §2.3), persisted across sessions.
    duty_rpm_table: DutyRpmTable,
    /// Paired device-loop entry caps keyed by strategy, duty, and AC state.
    persisted_warm_start: BTreeMap<String, WarmStartEntry>,
    /// Per-device fitted CPU gains keyed by strategy and interval.
    persisted_cpu_gains: BTreeMap<String, crate::control::device_loop::Gains>,
    /// Per-device fitted GPU gains keyed by strategy and interval.
    persisted_gpu_gains: BTreeMap<String, crate::control::device_loop::Gains>,
    /// Qualified Held-mode seed, retained without refreshing its age.
    persisted_t_star_last_good: Option<TStarSeed>,
    /// User config (fan target, floors, fast limit). Mutated + saved when
    /// the fan target changes.
    config: Config,
    /// Where `config` persists to (`--config`).
    config_path: PathBuf,
    /// Auto-mode loop state; retained unchanged while an Auto-originated
    /// calibration temporarily owns the actuators.
    auto: Option<AutoState>,
    /// Whether the current calibration suspended an Auto session.
    calib_started_from_auto: bool,
    /// One sample hold after an Auto-originated calibration restores its
    /// pair; this resynchronises sample time without ticking or writing.
    calib_reentry_hold: bool,
    /// Thermal watchdog (Task 28): observes EVERY sample in EVERY mode
    /// (including Calibrating, where the rest of the sample machinery is
    /// suspended); a trip ACTS only when something is commanded.
    watchdog: ThermalWatchdog,
    /// Flag transitions and CPU maintenance verdict notes since the last drain; `on_command`/`on_sample` drain them into their
    /// returned batch so every flag transition lands in telemetry exactly
    /// once.
    pending_effects: Vec<Effect>,
    /// Debounce for the idle-Monitor watchdog warn: the immediate re-arm
    /// means a persistently hot idle machine re-trips every TRIP_STREAK
    /// samples, so warn once per continuous idle-trip episode (reset when
    /// the watchdog goes quiet again — genuinely cool/valid evidence).
    idle_trip_warned: bool,
    /// Per-device group boxcars used only during calibration.
    calib_replica: Option<EcReplica>,
    /// Raw reconciliation history used to apply the same fair-view slope
    /// gate during calibration as during Auto.
    calib_ec_slope_window: std::collections::VecDeque<f64>,
    /// Frozen socket identity for the calibration replica and persisted key.
    calib_frozen_strategy: Option<String>,
    calib_frozen_interval: Option<u32>,
    /// Latest synchronously verified CPU calibration write and completion.
    calib_cpu_verified: bool,
    calib_cpu_checked_at_s: Option<f64>,
    calib_cpu_readback: Option<WriteVerdict>,
    calib_cpu_completed_at_s: Option<f64>,
    /// GPU verification stays paired to acquisition time across every
    /// calibration command, including held and stepped locks.
    calib_gpu_verified: bool,
    calib_gpu_completed_at_s: Option<f64>,
    calib_gpu_verifier: Option<GpuLockVerifier>,
    calib_gpu_commands: std::collections::VecDeque<GpuCommandEvidence>,
    calib_gpu_command_generation: u64,
    /// Main's shutdown flag (roast-pr-2 finding 2), installed by [`spawn`].
    /// `None` in unit tests and any construction that never shuts down.
    ///
    /// This is the STOP FENCE for an abandoned controller thread: main's
    /// controller join is bounded, so a thread that was wedged in an untimed
    /// call (NVML) can unwedge *after* `FinalRestore::drop` has already put
    /// the hardware back to stock. Without the fence it would then service a
    /// still-queued `Sample`, re-issue a CPU cap, and leave that cap in
    /// place at process exit. Checked at the top of `on_sample` and again
    /// immediately before every actuator WRITE (never before a restore —
    /// restores must always be allowed through).
    shutdown: Option<Arc<AtomicBool>>,
}

/// Population standard deviation (divides by `n`, not `n - 1`) — the
/// steady-window detector's gate is stated as a population stdev (design
/// §2.3), and the window is the entire population being judged, not a
/// sample drawn from a larger one. `values` must be non-empty (callers only
/// invoke this on a full [`STEADY_WINDOW_N`]-sample window).
fn population_stdev(values: &[f64]) -> f64 {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
    variance.sqrt()
}

impl<R: Runner> Controller<R> {
    pub fn new(
        mut guard: RestoreGuard<R>,
        persisted: PersistedState,
        state_path: PathBuf,
        config: Config,
        config_path: PathBuf,
    ) -> Self {
        // No construction path may bypass the configured hardware bounds.
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
            ..ControlStatus::default()
        };
        Self {
            guard,
            completion_clock: Box::new(Instant::now),
            status,
            stick_violations: 0,
            resumed_until: None,
            strict_until: None,
            last_reassert: None,
            calib: None,
            burner: None,
            state_path,
            calibrated_at: persisted.calibrated_at,
            duty_rpm_table: persisted.duty_rpm_table,
            // Preserve paired device entry state for the live DeviceLoops.
            persisted_warm_start: persisted.warm_start,
            persisted_cpu_gains: persisted.cpu_gains,
            persisted_gpu_gains: persisted.gpu_gains,
            persisted_t_star_last_good: persisted.t_star_last_good,
            config,
            config_path,
            auto: None,
            calib_started_from_auto: false,
            calib_reentry_hold: false,
            watchdog: ThermalWatchdog::new(),
            pending_effects: Vec::new(),
            idle_trip_warned: false,
            calib_replica: None,
            calib_ec_slope_window: std::collections::VecDeque::new(),
            calib_frozen_strategy: None,
            calib_frozen_interval: None,
            calib_cpu_verified: false,
            calib_cpu_checked_at_s: None,
            calib_cpu_readback: None,
            calib_cpu_completed_at_s: None,
            calib_gpu_verified: false,
            calib_gpu_completed_at_s: None,
            calib_gpu_verifier: None,
            calib_gpu_commands: std::collections::VecDeque::new(),
            calib_gpu_command_generation: 0,
            shutdown: None,
        }
    }

    /// Install main's shutdown flag as this controller's stop fence
    /// (roast-pr-2 finding 2). Called by [`spawn`]; tests that need the
    /// fence call it directly.
    pub fn set_shutdown_flag(&mut self, shutdown: Arc<AtomicBool>) {
        self.shutdown = Some(shutdown);
    }

    /// True once main has begun shutting down (its `shutdown` flag is set
    /// strictly BEFORE `Command::Quit` is sent, so this also means "Quit is
    /// pending"). No actuator write may be issued while this holds.
    fn shutting_down(&self) -> bool {
        self.shutdown
            .as_ref()
            .is_some_and(|f| f.load(Ordering::Relaxed))
    }

    /// Current status (the shell clones it into `Event::Status`).
    pub fn status(&self) -> &ControlStatus {
        &self.status
    }

    /// Exposes safety internals to closed-loop acceptance tests without
    /// changing runtime status or serialized telemetry.
    #[cfg(test)]
    pub(crate) fn test_diagnostics(&mut self) -> Option<ControllerDiagnostics> {
        let auto = self.auto.as_mut()?;
        Some(ControllerDiagnostics {
            cpu_guard_max: auto.cpu_ratchet.step(false, false, None),
            gpu_guard_max: auto.gpu_ratchet.step(false, false, None),
            cpu_actuator: auto.cpu_actuator_state,
            gpu_actuator: auto.gpu_actuator_state,
            cpu_mismatch_strikes: auto.cpu_verdict.mismatch_streak,
            gpu_mismatch_strikes: auto.gpu_verdict.mismatch_streak,
            cpu_released: auto.cpu_verdict.released,
            gpu_released: auto.gpu_verdict.released,
            reconciliation_ma_c: auto.replica.reconciliation_ma(),
            reconciliation_ready: auto.replica.reconciliation_ready(),
            reconciliation_ever_scored: auto.replica.ever_scored(),
            reconciliation_mismatch: auto.replica.ec_mismatch(),
            reconciliation_scored_count: auto.reconciliation_scored_count,
        })
    }

    /// Mirrors the shell's Decision flag composition for acceptance tests.
    #[cfg(test)]
    pub(crate) fn test_emitted_telemetry_flags(&self) -> Vec<crate::types::TelemetryFlag> {
        decision_telemetry_flags(&self.status)
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
        // floors only bound future device-loop commands). The
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
                    // Stop fence (roast-pr-2 finding 2): main raises
                    // `shutdown` before it sends Quit, so a manual key that
                    // raced shutdown must not land a cap either.
                    Some(_) if self.shutting_down() => {
                        tracing::debug!("shutting down; ignoring SetCpuW({w})");
                    }
                    None => tracing::warn!("no CPU actuator this run; ignoring SetCpuW({w})"),
                    // fw-fanctrl-loop-j6s: non-Verified verdicts (Mismatch/
                    // Unreadable/Unverifiable) are mapped to today's plain
                    // write-failure behavior; verified commands alone update status.
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
                // Stop fence (roast-pr-2 finding 2); read before the
                // scrutinee takes its &mut borrow of `self.guard`.
                let fenced = self.shutting_down();
                match self.guard.gpu.as_mut() {
                    Some(_) if fenced => {
                        tracing::debug!("shutting down; ignoring SetGpuMaxClock({mhz})");
                    }
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
                // Exiting Auto drops its session state before restoring stock.
                self.exit_auto_and_persist();
                self.release_to_stock();
                effects.push(Effect::Released);
                "release"
            }
            Command::SetFanTarget(rpm) => {
                let clamped = rpm.clamp(FAN_TARGET_MIN_RPM, FAN_TARGET_MAX_RPM);
                if clamped != self.status.fan_target_rpm {
                    self.status.fan_target_rpm = clamped;
                    // The shared target source reads this value on its next tick.
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
                // range floors would make a DeviceLoop clamp invalid.
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
                    // Both device loops read these floors on their next
                    // step. Persist on CHANGE only (a held
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
                } else {
                    // Per-device T* loops use direct CPU watts and GPU SM
                    // clocks, so a GPU watts model is calibration data, not
                    // an Auto-entry precondition. Missing fitted device
                    // gains remain informational until the live key resolves.
                    self.add_flag(StatusFlag::NotCalibrated);
                    let auto = AutoState::new(&self.config);
                    self.auto = Some(auto);
                    self.status.mode = Mode::Auto;
                    // Any manual limits stay in force for <1 s: the first
                    // sample runs the controller, which starts from its
                    // conservative start and replaces them.
                    "auto:on"
                }
            }
            Command::SetAuto(false) => {
                if self.status.mode == Mode::Auto {
                    // Fresh DeviceLoops on re-entry.
                    self.exit_auto_and_persist();
                    self.release_to_stock();
                    effects.push(Effect::Released);
                } else {
                    tracing::warn!("SetAuto(false) ignored: not in auto mode");
                }
                "auto:off"
            }
            Command::StartCalibration => {
                if !matches!(self.status.mode, Mode::Monitor | Mode::Auto) {
                    tracing::warn!(
                        "StartCalibration ignored: mode {} cannot be suspended",
                        self.status.mode.as_str()
                    );
                } else {
                    self.status.calib_outcome = Some(CalibOutcome::default());
                    self.calib_started_from_auto = self.status.mode == Mode::Auto;
                    self.calib_reentry_hold = false;
                    let mut runner = CalibRunner::new(
                        self.persisted_cpu_gains.clone(),
                        self.persisted_gpu_gains.clone(),
                    );
                    let runner_effects = runner.start();
                    self.calib = Some(runner);
                    self.status.mode = Mode::Calibrating;
                    self.calib_replica = Some(EcReplica::new(DEFAULT_MA_INTERVAL));
                    self.calib_ec_slope_window.clear();
                    self.calib_frozen_strategy = None;
                    self.calib_frozen_interval = None;
                    self.calib_cpu_verified = false;
                    self.calib_cpu_checked_at_s = None;
                    self.calib_cpu_readback = None;
                    self.calib_cpu_completed_at_s = None;
                    self.calib_gpu_verified = false;
                    self.calib_gpu_completed_at_s = None;
                    self.calib_gpu_verifier = None;
                    self.calib_gpu_commands.clear();
                    self.calib_gpu_command_generation = 0;
                    self.apply_calib_effects(runner_effects, None);
                    self.sync_calib_status();
                }
                "calib:start"
            }
            Command::DismissCalibrationOutcome => {
                if self.calib.is_none() {
                    self.status.calib_outcome = None;
                }
                "calib:dismissed"
            }
            Command::AbortCalibration => {
                match self.calib.take() {
                    None => tracing::warn!("AbortCalibration ignored: no calibration running"),
                    Some(mut runner) => {
                        self.apply_calib_effects(runner.abort(), None);
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
                    self.apply_calib_effects(runner.abort(), None);
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
        // STOP FENCE (roast-pr-2 finding 2), before anything else: once main
        // has begun shutting down, queued samples are DRAINED and ignored.
        // Main's controller join is bounded, so this thread may still be
        // alive after `FinalRestore` restored stock hardware; servicing a
        // sample here would re-issue a CPU cap that nothing would ever undo.
        if self.shutting_down() {
            return Vec::new();
        }
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
            Trip::Thermal
                if self.status.mode == Mode::Calibrating && self.anything_commanded() =>
            {
                // Terminate through the calibration runner first. This
                // restores its established held pair and makes the reason
                // observable even if the configurable calibration guard is
                // below or above the watchdog's fixed 95 C boundary.
                let before = self.status.clone();
                let runner_effects = self.calib.as_mut().map_or_else(Vec::new, |runner| {
                    runner.abort_with_reason(
                        "global watchdog: CPU Tctl reached 95C for 3 samples".into(),
                    )
                });
                let cause = self.apply_calib_effects(runner_effects, Some(s));
                self.sync_calib_status();
                let mut effects = Vec::new();
                if self.status != before {
                    effects.push(Effect::StatusChanged {
                        cause: cause.unwrap_or("calib:skipped"),
                    });
                } else if let Some(cause) = cause {
                    effects.push(Effect::Noted { cause });
                }
                // The global watchdog remains the final safety authority
                // and releases the restored pair to stock.
                effects.extend(self.emergency_release(Trip::Thermal));
                return effects;
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
        if self.status.mode == Mode::Auto && self.calib_reentry_hold {
            self.calib_reentry_hold = false;
            self.reseed_auto_after_calibration(s);
            return Vec::new();
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
            self.add_flag(StatusFlag::Resumed);
            self.resumed_until = Some(s.t_mono + RESUMED_FLAG_S);
            // The pre-suspend fan window and EC boxcar are thermally stale
            // (the machine cooled while asleep) — clear both so neither can
            // read a slope/mean spanning the suspend as settled evidence
            // (review finding; design §2.6: "cleared on a resumed sample").
            if let Some(auto) = &mut self.auto {
                auto.fan_window.clear();
                auto.ec_slope_window.clear();
                // Integration-sweep regression:
                // the steady-window detector's own accumulated RPM samples
                // are exactly as stale across a suspend as the fan/EC
                // windows above -- design §2.2 names it explicitly ("clears
                // ... the steady window"). Left uncleared, a window that was
                // one sample from completing pre-suspend would complete on
                // the very next post-resume sample and write a warm-start/
                // refinement entry from readings spanning the gap.
                auto.steady_window.clear();
                auto.steady_key = None;
                auto.refinement_window.clear();
                auto.refinement_key = None;
                // A post-suspend clock observation cannot be paired to a
                // pre-suspend NVML command.  Keep the first post-resume
                // verification explicitly Unverifiable until a new command
                // has completed and a later sample can observe it.
                auto.gpu_commands.clear();
                auto.gpu_command_generation = 0;
                auto.gpu_verifier = None;
                auto.gpu_verifier_mhz = None;
                auto.cpu_hot_streak = 0;
                auto.cpu_hot = false;
                auto.guards = Guards::new(self.config.gpu_hot_c, self.config.nvme_hot_c);
                auto.cpu_verdict = VerdictState::default();
                auto.gpu_verdict = VerdictState::default();
                auto.cpu_actuator_state = ActuatorState::Unverifiable;
                auto.gpu_actuator_state = ActuatorState::Unverifiable;
            }
            if let Some(all_ok) = self.reassert_actuators(s) {
                self.last_reassert = Some(s.t_mono);
                // Telemetry honesty (as in the periodic path): a failed
                // attempt must not count as a phantom reassert.
                effects.push(Effect::Reasserted {
                    cause: if all_ok { "resume" } else { "resume_failed" },
                });
            }
            cause.get_or_insert("resume");
        } else if self.resumed_until.is_some_and(|until| s.t_mono >= until) {
            self.remove_flag(StatusFlag::Resumed);
            self.resumed_until = None;
            cause.get_or_insert("resume");
        }

        // Auto loop (two DeviceLoops) BEFORE the stickiness/reassert
        // machinery, so the watchdogs below see the freshly commanded caps.
        // Unlike Calibrating, those watchdogs stay active in Auto: they key
        // off status.cpu_limit_w/gpu_max_mhz — reassert reapplies them, stickiness
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
                             ({needed} consecutive samples); checking cap read-back",
                            s.cpu_pkg_w
                        );
                        if let Some(all_ok) = self.reassert_actuators(s) {
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

        // Periodic maintenance: CPU read-first repair, GPU lock reassert.
        // Defends against PPD/tuned clobbers. The first
        // sample after a command only pins the baseline.
        if self.status.cpu_limit_w.is_some() || self.status.gpu_max_mhz.is_some() {
            match self.last_reassert {
                None => self.last_reassert = Some(s.t_mono),
                Some(last) if s.t_mono - last >= REASSERT_PERIOD_S => {
                    self.last_reassert = Some(s.t_mono);
                    if let Some(all_ok) = self.reassert_actuators(s) {
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
            // A cause fired (e.g. a held/frozen control step) with no
            // visible status delta. Still worth a telemetry Decision line
            // (mirrors `on_calib_sample`'s same fallback below).
            effects.push(Effect::Noted { cause });
        }
        effects
    }

    /// Runs both per-device loops from one 1 Hz sample.
    fn on_auto_sample(
        &mut self,
        s: &Sample,
        effects: &mut Vec<Effect>,
        cause: &mut Option<&'static str>,
    ) {
        self.on_auto_sample_v4(s, effects, cause);
    }

    fn on_auto_sample_v4(
        &mut self,
        s: &Sample,
        effects: &mut Vec<Effect>,
        cause: &mut Option<&'static str>,
    ) {
        let target_duty = self.duty_rpm_table.duty_for_rpm(self.status.fan_target_rpm);
        let view = s.fanctrl.as_ref();
        let (cpu_decision, gpu_decision, target, flags) = {
            let auto = self.auto.as_mut().expect("AutoState exists in Auto");
            // Keep the observed gap intact for TStarSource: it treats a
            // finite gap over seven seconds as invalid Held control time
            // and resets its cadence/dwells. DeviceLoop applies its own
            // two-second control-time bound, so passing the raw delta does
            // not turn suspend time into PI or slew time there.
            let dt_s = auto
                .last_sample_t_mono
                .map(|last| s.t_mono - last)
                .filter(|delta| delta.is_finite() && *delta > 0.0)
                .unwrap_or(0.0);
            auto.last_sample_t_mono = Some(s.t_mono);

            if s.resumed {
                auto.replica.reset(
                    view.map(|v| v.ma_temperature),
                    s.ec.as_ref().and_then(|ec| ec.cpu_group_c),
                    s.ec.as_ref().and_then(|ec| ec.gpu_group_c),
                );
                auto.cpu_draw_window.clear();
                auto.last_cpu_write_t_mono = None;
                auto.last_gpu_write_t_mono = None;
            }
            // Resolve on the first usable view as well as on every view
            // replacement.  An Auto entry can happen after the poller has
            // already cached its first view, in which case there is no
            // `fanctrl_view_changed` edge for this controller session.
            if s.fanctrl_view_changed || !auto.view_initialised {
                if let Some(view) = view {
                    auto.replica.set_interval(view.ma_interval as usize);
                    let key = format!("{}:{}", view.strategy, view.ma_interval);
                    let (cpu_gains, cpu_source) = self
                        .config
                        .cpu_gains
                        .map(|gains| (gains, crate::types::GainsSource::Config))
                        .or_else(|| {
                            self.persisted_cpu_gains
                                .get(&key)
                                .copied()
                                .map(|gains| (gains, crate::types::GainsSource::Fitted))
                        })
                        .unwrap_or_else(|| {
                            (
                                default_gains::<W>(view.ma_interval),
                                crate::types::GainsSource::Default,
                            )
                        });
                    let (gpu_gains, gpu_source) = self
                        .config
                        .gpu_gains
                        .map(|gains| (gains, crate::types::GainsSource::Config))
                        .or_else(|| {
                            self.persisted_gpu_gains
                                .get(&key)
                                .copied()
                                .map(|gains| (gains, crate::types::GainsSource::Fitted))
                        })
                        .unwrap_or_else(|| {
                            (
                                default_gains::<Mhz>(view.ma_interval),
                                crate::types::GainsSource::Default,
                            )
                        });
                    auto.cpu_loop.set_gains(cpu_gains);
                    auto.gpu_loop.set_gains(gpu_gains);
                    auto.cpu_gains_source = cpu_source;
                    auto.gpu_gains_source = gpu_source;
                }
            }
            // Actuator-verdict suppression is a per-session edge. Maintain
            // it before either paired verdict is consumed below.
            if auto.last_on_ac.is_some_and(|last| last != s.on_ac) {
                auto.on_ac_suppress_until = Some(s.t_mono + ON_AC_EDGE_SUPPRESS_S);
            }
            auto.last_on_ac = Some(s.on_ac);
            // A controller can enter Auto before the first socket poll has
            // completed.  Once that first fresh view arrives, seed the raw
            // reconciliation history from its MA before appending this
            // sample; otherwise Curve waits a full moving-average window
            // even though it has a contemporaneous, fair socket seed.
            if !auto.view_initialised && view.is_some() && s.fanctrl_freshness == Freshness::Fresh
            {
                auto.replica.reset(
                    view.map(|view| view.ma_temperature),
                    s.ec.as_ref().and_then(|ec| ec.cpu_group_c),
                    s.ec.as_ref().and_then(|ec| ec.gpu_group_c),
                );
                auto.view_initialised = true;
            }
            if let Some(ec) = s.ec.as_ref() {
                if auto.ec_slope_window.len() >= EC_SLOPE_WINDOW_S {
                    auto.ec_slope_window.pop_front();
                }
                if let Some(raw_max) = ec.reconciliation_max_c {
                    auto.ec_slope_window.push_back(f64::from(raw_max));
                }
            }
            auto.replica.tick(s.ec.as_ref());

            let reconciliation = match (view, s.ec.as_ref()) {
                // Reconciliation is a comparison to the socket's *new* All
                // snapshot.  Re-scoring the cached view against every later
                // 1 Hz EC sample manufactures mismatches while the fanctrl
                // poller is simply between polls.
                (Some(view), Some(ec))
                    if s.fanctrl_freshness == Freshness::Fresh && s.fanctrl_view_changed =>
                {
                    let slope = auto
                        .ec_slope_window
                        .front()
                        .zip(auto.ec_slope_window.back())
                        .map_or(0.0, |(first, last)| {
                            (last - first)
                                / auto.ec_slope_window.len().saturating_sub(1).max(1) as f64
                        });
                    let gap_s = view
                        .all_observed_at
                        .map(|captured| Instant::now().saturating_duration_since(captured).as_secs_f64())
                        .unwrap_or(0.0);
                    let observation = if slope >= RECONCILIATION_SKIP_SLOPE_C_PER_S
                        || gap_s >= RECONCILIATION_SKIP_GAP_S
                    {
                        ReconciliationObservation::Skipped
                    } else {
                        let max_matches = ec.reconciliation_max_c.is_some_and(|raw_max| {
                            (f64::from(raw_max) - view.temperature).abs() <= 1.0
                        });
                        let ma_matches = auto
                            .replica
                            .reconciliation_ma()
                            .map(|ma| (ma - view.ma_temperature).abs() <= 1.0);
                        ReconciliationObservation::Scored {
                            max_matches,
                            ma_matches,
                            socket_ma: view.ma_temperature,
                        }
                    };
                    auto.replica.score_reconciliation(observation)
                }
                _ => auto
                    .replica
                    .score_reconciliation(ReconciliationObservation::Skipped),
            };
            #[cfg(test)]
            if reconciliation.scored {
                auto.reconciliation_scored_count += 1;
            }
            if let Some(reseed) = reconciliation.reseed_ma {
                auto.replica.reset_after_mismatch_clear(
                    Some(reseed),
                    s.ec.as_ref().and_then(|ec| ec.cpu_group_c),
                    s.ec.as_ref().and_then(|ec| ec.gpu_group_c),
                );
            }

            let cpu_group = auto.replica.cpu_group_ma();
            let gpu_group = auto.replica.gpu_group_ma();
            // A saved T* is meaningful only after the socket has supplied
            // the strategy and current fan target.  Do this once, before the
            // first source tick, so a later view/key refresh cannot reseed a
            // live target or step either device loop.
            if !auto.source_initialised {
                let now_unix_s = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map_or(0, |duration| duration.as_secs());
                let qualified = view.filter(|_| s.fanctrl_freshness == Freshness::Fresh)
                    .and_then(|view| PersistedState {
                        t_star_last_good: self.persisted_t_star_last_good.clone(),
                        ..PersistedState::default()
                    }.qualified_seed(
                        &view.strategy, self.status.fan_target_rpm.round() as u32,
                        now_unix_s, 0.0, self.config.cpu_hot_c.max(self.config.gpu_hot_c),
                    ));
                let seed = qualified.map(EntrySeed::Qualified).unwrap_or(EntrySeed::Groups {
                    cpu_c: cpu_group, gpu_c: gpu_group,
                });
                auto.tstar = TStarSource::new(seed);
                auto.source_initialised = true;
            }
            let sensors = s.ec.as_ref().map_or_else(Vec::new, |ec| {
                ec.all
                    .iter()
                    .map(|(label, value_c)| SensorReading {
                        label: label.as_str().to_owned(),
                        value_c: *value_c,
                        class: match label.group() {
                            EcGroup::Cpu | EcGroup::Gpu => SensorClass::Controllable,
                            EcGroup::Uncontrollable => SensorClass::KnownUncontrollable,
                            EcGroup::Unknown => SensorClass::Unknown,
                        },
                    })
                    .collect()
            });
            let argmax_lead_c = s.ec.as_ref().map_or(0.0, |ec| {
                let argmax = ec
                    .all
                    .iter()
                    .find(|(label, _)| label == &ec.argmax)
                    .map(|(_, value)| *value)
                    .unwrap_or(f64::from(ec.max_c));
                let runner_up = ec
                    .all
                    .iter()
                    .filter(|(label, _)| label != &ec.argmax)
                    .map(|(_, value)| *value)
                    .fold(f64::NEG_INFINITY, f64::max);
                if runner_up.is_finite() {
                    (argmax - runner_up).max(0.0)
                } else {
                    1.1
                }
            });
            let source = auto.tstar.tick(&TStarInput {
                dt_s,
                now_unix_s: SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map_or(0, |duration| duration.as_secs()),
                fan_valid: s.fan_valid,
                ec_valid: s.ec_valid,
                watchdog_release: false,
                resumed: s.resumed,
                // `Fresh` means the currently cached `print all` view is
                // inside its socket TTL.  A view-change edge is only for
                // replica interval/gain re-resolution; using it as the
                // source's continuous-entry evidence would make a stable
                // 1 Hz control run wait for a new socket poll each second.
                fresh_view: view.is_some() && s.fanctrl_freshness == Freshness::Fresh,
                replica_reconciled: reconciliation.trusted,
                curve_points: view.map(|v| v.curve.clone()),
                snapped_duty: Some(target_duty),
                strategy: view.map(|v| v.strategy.clone()),
                requested_fan_target_rpm: self.status.fan_target_rpm.round() as u32,
                fan_rpm: s.fan_valid.then(|| s.max_fan_rpm()),
                previous_holds: vec![
                    HeldDeviceInput::new(TStarDevice::Cpu, auto.prior_cpu_hold, cpu_group),
                    HeldDeviceInput::new(TStarDevice::Gpu, auto.prior_gpu_hold, gpu_group),
                ],
                sensors,
                argmax_label: s.ec.as_ref().map(|ec| ec.argmax.as_str().to_owned()),
                argmax_lead_c,
                cpu_group_c: cpu_group,
                gpu_group_c: gpu_group,
                cpu_hot_c: self.config.cpu_hot_c,
                gpu_hot_c: self.config.gpu_hot_c,
                auto_exit: false,
            });

            let guard = auto
                .guards
                .step(s.gpu_temp_valid.then_some(s.gpu_temp_c), s.nvme_temp_c);
            if s.cpu_temp_valid && s.cpu_temp_c >= self.config.cpu_hot_c {
                auto.cpu_hot_streak = auto.cpu_hot_streak.saturating_add(1);
            } else {
                auto.cpu_hot_streak = 0;
            }
            if !s.cpu_temp_valid || s.cpu_temp_c <= self.config.cpu_hot_c - 3.0 {
                auto.cpu_hot = false;
            } else if auto.cpu_hot_streak >= 3 {
                auto.cpu_hot = true;
            }
            auto.cpu_ratchet.set_floor(self.config.cpu_floor_w);
            auto.gpu_ratchet.set_floor(f64::from(self.config.gpu_floor_mhz));
            let cpu_max = auto.cpu_ratchet.step(
                s.cpu_temp_valid,
                auto.cpu_hot,
                s.cpu_temp_valid.then_some(s.cpu_temp_c),
            );
            let gpu_max = auto.gpu_ratchet.step(
                s.gpu_temp_valid,
                guard.gpu_hot,
                s.gpu_temp_valid.then_some(s.gpu_temp_c),
            );

            // A zero package-power sample is the RAPL warm-up/no-reading
            // sentinel in this controller.  It cannot make the required
            // five-valid-sample draw mean available.
            if s.cpu_pkg_w.is_finite() && s.cpu_pkg_w > 0.0 {
                auto.cpu_draw_window.push_back(s.cpu_pkg_w);
                if auto.cpu_draw_window.len() > 5 {
                    auto.cpu_draw_window.pop_front();
                }
            } else {
                // A reset/invalid RAPL delta invalidates the whole mean;
                // five later valid deltas are required before shadow returns.
                auto.cpu_draw_window.clear();
            }
            let cpu_draw = (auto.cpu_draw_window.len() == 5).then(|| {
                auto.cpu_draw_window.iter().sum::<f64>() / auto.cpu_draw_window.len() as f64
            });
            let t_star = source.t_star.unwrap_or(self.config.cpu_hot_c - 2.0);
            // A qualified paired record is advisory entry state.  It is
            // keyed to the same strategy/duty/power-source tuple as the
            // steady-window writer, and either device's live draw floor can
            // raise it before the advisory seed becomes usable. It never overwrites a
            // command already in force: that command remains `last_applied`
            // and output slew carries the loop toward the candidate.
            let warm = view.and_then(|view| {
                self.persisted_warm_start
                    .get(&warm_start_key(&view.strategy, target_duty, s.on_ac))
            });
            // Cold entry starts each present device at its safe configured
            // ceiling; only a cap already in force overrides that seed.
            // Delaying the GPU seed until its group exists preserves the
            // absent-from-start rule (no speculative lock for a powered-off
            // dGPU).  Shadow remains at the thermal seed until the CPU has
            // five valid draw samples.
            if let Some(group) = cpu_group
                && !auto.cpu_entry_seeded
            {
                let entry_floor = cpu_draw
                    .map(|draw| draw + self.config.shadow_headroom_cpu_w)
                    .unwrap_or(self.config.cpu_floor_w);
                let thermal = warm.filter(|_| cpu_draw.is_some())
                    .map(|record| record.cpu_cap_w)
                    .unwrap_or(self.config.cpu_max_w)
                    .max(entry_floor)
                    .min(cpu_max);
                auto.cpu_loop.seed_candidates(
                    thermal,
                    // Before the five-sample mean exists shadow remains
                    // thermal-only.  Once it exists, the first candidate is
                    // the measured draw plus headroom, not a fall from max.
                    cpu_draw.map_or(thermal, |_| entry_floor),
                    self.status.cpu_limit_w,
                    t_star - group,
                );
                auto.cpu_entry_seeded = true;
            }
            if let Some(group) = gpu_group
                && !auto.gpu_entry_seeded
            {
                let entry_floor = if s.gpu_mhz_valid {
                    s.gpu_sm_mhz + self.config.shadow_headroom_gpu_mhz
                } else {
                    f64::from(self.config.gpu_floor_mhz)
                };
                let thermal = warm.filter(|_| s.gpu_mhz_valid)
                    .map(|record| f64::from(record.gpu_lock_mhz))
                    .unwrap_or(f64::from(self.config.gpu_max_mhz))
                    .max(entry_floor)
                    .min(gpu_max);
                auto.gpu_loop.seed_candidates(
                    thermal,
                    if s.gpu_mhz_valid { entry_floor } else { thermal },
                    self.status.gpu_max_mhz.map(f64::from),
                    t_star - group,
                );
                auto.gpu_entry_seeded = true;
            }
            if let Some(draw) = cpu_draw
                && let Some(group) = cpu_group
                && !auto.cpu_shadow_entry_seeded
                && auto.cpu_actuator_state != ActuatorState::Mismatch
                && !auto.cpu_verdict.in_episode() && !s.resumed
            {
                let entry_floor = (draw + self.config.shadow_headroom_cpu_w)
                    .clamp(self.config.cpu_floor_w, cpu_max);
                if let Some(record) = warm {
                    // The advisory thermal seed becomes usable only with
                    // entry draw evidence. Transfer preserves requested and
                    // last_applied, so ordinary output slew remains in force.
                    auto.cpu_loop.transfer_thermal(record.cpu_cap_w.max(entry_floor).min(cpu_max),
                        t_star - group);
                }
                auto.cpu_loop.seed_initial_shadow(entry_floor);
                auto.cpu_shadow_entry_seeded = true;
            }
            if s.gpu_mhz_valid && let Some(group) = gpu_group
                && !auto.gpu_shadow_entry_seeded
                && auto.gpu_actuator_state != ActuatorState::Mismatch
                && !auto.gpu_verdict.in_episode() && !s.resumed
            {
                let entry_floor = (s.gpu_sm_mhz + self.config.shadow_headroom_gpu_mhz)
                    .clamp(f64::from(self.config.gpu_floor_mhz), gpu_max);
                if let Some(record) = warm {
                    auto.gpu_loop.transfer_thermal(f64::from(record.gpu_lock_mhz).max(entry_floor).min(gpu_max),
                        t_star - group);
                }
                auto.gpu_loop.seed_initial_shadow(entry_floor);
                auto.gpu_shadow_entry_seeded = true;
            }
            auto.cpu_restore_thermal_pending |= source.restore_thermal;
            auto.gpu_restore_thermal_pending |= source.restore_thermal;
            let cpu_restored = auto.cpu_restore_thermal_pending && cpu_group.is_some()
                && auto.cpu_actuator_state != ActuatorState::Mismatch
                && !auto.cpu_verdict.in_episode() && !s.resumed;
            let gpu_restored = auto.gpu_restore_thermal_pending && gpu_group.is_some()
                && auto.gpu_actuator_state != ActuatorState::Mismatch
                && !auto.gpu_verdict.in_episode() && !s.resumed;
            if cpu_restored {
                auto.cpu_loop.transfer_thermal(cpu_max, t_star - cpu_group.expect("present group"));
                auto.cpu_restore_thermal_pending = false;
            } else if source.resync && let Some(group) = cpu_group {
                auto.cpu_loop.resync_error(t_star - group);
            }
            if gpu_restored {
                auto.gpu_loop.transfer_thermal(gpu_max, t_star - gpu_group.expect("present group"));
                auto.gpu_restore_thermal_pending = false;
            } else if source.resync && let Some(group) = gpu_group {
                auto.gpu_loop.resync_error(t_star - group);
            }
            let cpu = auto.cpu_loop.tick(TickInput {
                t_star,
                group_c: cpu_group,
                draw: cpu_draw,
                floor: self.config.cpu_floor_w,
                max: cpu_max,
                mode: source.thermal_mode,
                actuator: auto.cpu_actuator_state,
                dt_s,
                resumed: s.resumed,
                delta_tstar: if source.resync || cpu_restored { 0.0 } else { source.delta_tstar },
                shadow_headroom: self.config.shadow_headroom_cpu_w,
                shadow_fall_rate: self.config.shadow_fall_rate_cpu,
                shadow_enabled: true,
            });
            let gpu = auto.gpu_loop.tick(TickInput {
                t_star,
                group_c: gpu_group,
                draw: s.gpu_mhz_valid.then_some(s.gpu_sm_mhz),
                floor: f64::from(self.config.gpu_floor_mhz),
                max: gpu_max,
                mode: source.thermal_mode,
                actuator: auto.gpu_actuator_state,
                dt_s,
                resumed: s.resumed,
                delta_tstar: if source.resync || gpu_restored { 0.0 } else { source.delta_tstar },
                shadow_headroom: self.config.shadow_headroom_gpu_mhz,
                shadow_fall_rate: self.config.shadow_fall_rate_gpu,
                shadow_enabled: self.config.gpu_shadow_enabled,
            });
            auto.prior_cpu_hold = cpu.hold;
            auto.prior_gpu_hold = gpu.hold;
            (
                cpu,
                gpu,
                source,
                (guard.gpu_hot, guard.nvme_hot, reconciliation.ec_mismatch),
            )
        };

        self.sync_bool_flag(StatusFlag::GpuHot, flags.0);
        self.sync_bool_flag(StatusFlag::NvmeHot, flags.1);
        self.sync_bool_flag(StatusFlag::EcMismatch, flags.2);
        self.sync_bool_flag(
            StatusFlag::FanctrlLost,
            !matches!(s.fanctrl_freshness, Freshness::Fresh),
        );
        self.sync_bool_flag(
            StatusFlag::CurveInvalid,
            view.is_some_and(|view| Curve::from_points(view.curve.clone()).is_err()),
        );
        self.sync_bool_flag(
            StatusFlag::SteepCurve,
            target
                .flags
                .iter()
                .any(|flag| matches!(flag, crate::control::tstar::TStarFlag::SteepCurve)),
        );
        self.sync_bool_flag(
            StatusFlag::TargetUnreachable,
            target.flags.iter().any(|flag| {
                matches!(
                    flag,
                    crate::control::tstar::TStarFlag::TargetUnreachable(_)
                        | crate::control::tstar::TStarFlag::DeviceUnreachable { .. }
                )
            }),
        );
        let source_state_changed = self.status.tstar_state != Some(target.state.into());
        self.status.tstar_state = Some(target.state.into());
        self.status.t_star_c = target.t_star;
        self.status.strategy = view.map(|v| v.strategy.clone());
        self.status.cpu = Some(
            (
                cpu_decision,
                self.auto.as_ref().expect("auto").cpu_gains_source,
            )
                .into(),
        );
        self.status.gpu = Some(
            (
                gpu_decision,
                self.auto.as_ref().expect("auto").gpu_gains_source,
            )
                .into(),
        );
        self.status.ec_ma_c = self
            .auto
            .as_ref()
            .expect("auto")
            .replica
            .reconciliation_ma();
        self.status.telemetry_flags = target.flags.iter().map(Into::into).collect();
        for (device, lost) in [
            (crate::types::TelemetryDeviceName::Cpu, cpu_decision.group_lost),
            (crate::types::TelemetryDeviceName::Gpu, gpu_decision.group_lost),
        ] {
            if lost { self.status.telemetry_flags.push(crate::types::TelemetryFlag::GroupLost { device, active: true }); }
        }
        if let Some(ec) = &s.ec {
            for diagnostic in &ec.diagnostics {
                if let crate::sensors::ec::EcDiagnostic::EcImplausible { label } = diagnostic {
                    self.status.telemetry_flags.push(crate::types::TelemetryFlag::EcImplausible {
                        label: label.as_str().to_owned(), active: true,
                    });
                }
            }
        }
        let fitted = view.is_some_and(|view| {
            let key = format!("{}:{}", view.strategy, view.ma_interval);
            self.persisted_cpu_gains.contains_key(&key) && self.persisted_gpu_gains.contains_key(&key)
        });
        self.sync_bool_flag(StatusFlag::NotCalibrated, !fitted);
        if let Some(request) = target.persistence.as_ref() {
            // The source owns the cadence and safety gate.  Keep its
            // original timestamp so unrelated saves cannot refresh a seed.
            self.persisted_t_star_last_good = Some(request.seed.clone());
            self.save_persisted_state();
        }

        self.status.ec_argmax = s.ec.as_ref().map(|ec| ec.argmax.as_str().to_owned());
        self.status.duty_cmd = if target.state == crate::control::tstar::TStarState::Curve {
            view.and_then(|view| Curve::from_points(view.curve.clone()).ok())
                .and_then(|curve| curve.nearest_tread(target_duty))
        } else { None };
        self.status.snapped_rpm = self.status.duty_cmd
            .map_or(0.0, |duty| self.duty_rpm_table.rpm_for_duty(duty));
        if target.state == crate::control::tstar::TStarState::Released {
            // End only the device engagement. The shared source retains its
            // session quarantines and sees Released -> usable on recovery.
            let auto = self.auto.as_mut().expect("auto");
            auto.cpu_loop.reset_engagement();
            auto.gpu_loop.reset_engagement();
            auto.replica.reset(None, None, None);
            auto.cpu_draw_window.clear();
            auto.fan_window.clear();
            auto.ec_slope_window.clear();
            auto.steady_window.clear();
            auto.steady_key = None;
            auto.refinement_window.clear();
            auto.refinement_key = None;
            auto.cpu_entry_seeded = false;
            auto.gpu_entry_seeded = false;
            auto.cpu_shadow_entry_seeded = false;
            auto.gpu_shadow_entry_seeded = false;
            auto.cpu_restore_thermal_pending = false;
            auto.gpu_restore_thermal_pending = false;
            auto.view_initialised = false;
            auto.last_sample_t_mono = None;
            auto.last_cpu_write_t_mono = None;
            auto.last_gpu_write_t_mono = None;
            auto.gpu_commands.clear();
            auto.gpu_command_generation = 0;
            auto.gpu_verifier = None;
            auto.cpu_verdict = VerdictState::default();
            auto.gpu_verdict = VerdictState::default();
            auto.cpu_actuator_state = ActuatorState::Unverifiable;
            auto.gpu_actuator_state = ActuatorState::Unverifiable;
            auto.prior_cpu_hold = Hold::None;
            auto.prior_gpu_hold = Hold::None;
            self.status.cpu = None;
            self.status.gpu = None;
            self.stick_violations = 0;
            self.last_reassert = None;
            self.remove_flag(StatusFlag::ReadbackBlind);
            self.remove_flag(StatusFlag::LimitNotSticking);
            if !source_state_changed { return; }
            if let Some(gpu) = self.guard.gpu.as_mut() {
                let _ = gpu.release();
            }
            if let Some(cpu) = self.guard.cpu.as_ref() {
                let _ = cpu.restore_stock();
            }
            self.status.cpu_limit_w = None;
            self.status.gpu_max_mhz = None;
            effects.push(Effect::Released);
            cause.get_or_insert("auto:released");
            return;
        }

        self.write_device_decisions_v4(s, cpu_decision, gpu_decision, effects, cause);
        self.observe_v4_steady(s, cpu_decision, gpu_decision, target_duty, flags.0);
    }

    fn command_completed_at(&self, sample: &Sample) -> f64 {
        sample.t_mono + sample.acquired_at.map_or(0.0, |at| {
            (self.completion_clock)().saturating_duration_since(at).as_secs_f64()
        })
    }

    fn record_gpu_command(&mut self, sample: &Sample, applied: u32) {
        let completed_at = self.command_completed_at(sample);
        if let Some(auto) = self.auto.as_mut() {
            auto.gpu_loop.note_applied(f64::from(applied));
            auto.last_gpu_write_t_mono = Some(completed_at);
            auto.gpu_command_generation = auto.gpu_command_generation.saturating_add(1);
            auto.gpu_commands.push_back(GpuCommandEvidence::new(
                applied, completed_at, auto.gpu_command_generation,
            ));
            while auto.gpu_commands.len() > 3 { auto.gpu_commands.pop_front(); }
        }
    }

    fn write_device_decisions_v4(
        &mut self,
        s: &Sample,
        cpu: DeviceDecision,
        gpu: DeviceDecision,
        effects: &mut Vec<Effect>,
        cause: &mut Option<&'static str>,
    ) {
        let cpu_due = self
            .auto
            .as_ref()
            .expect("auto")
            .last_cpu_write_t_mono
            .is_none_or(|last| s.t_mono - last >= 2.0);
        let cpu_changed = self
            .status
            .cpu_limit_w
            .is_none_or(|applied| (applied - cpu.cap).abs() > 0.01);
        if cpu.write_allowed
            && (cpu.write_immediately || (cpu_due && (cpu_changed
                || self.auto.as_ref().expect("auto").cpu_verdict.in_episode())))
            && !self.shutting_down()
        {
            if let Some(actuator) = self.guard.cpu.as_ref() {
                let suppress = self
                    .auto
                    .as_ref()
                    .expect("auto")
                    .on_ac_suppress_until
                    .is_some_and(|until| s.t_mono < until);
                let cpu_mw = (cpu.cap * 1000.0).round() as u32;
                let mut verdict = actuator.set_sustained_mw(cpu_mw);
                // Confirm a candidate mismatch with exactly one re-write and
                // paired read-back.  The shutdown fence is checked again
                // because the first synchronous call can take time.
                if matches!(verdict, WriteVerdict::Mismatch { .. })
                    && !suppress
                    && !self.shutting_down()
                {
                    verdict = actuator.set_sustained_mw(cpu_mw);
                }
                // Include the blocking write/read-back (and corrective retry)
                // in the cadence baseline, even when the attempt failed.
                let completed_at = self.command_completed_at(s);
                let auto = self.auto.as_mut().expect("auto");
                auto.last_cpu_write_t_mono = Some(completed_at);
                auto.cpu_actuator_state = match verdict {
                    WriteVerdict::Verified(value) => {
                        auto.cpu_loop.note_applied(value);
                        self.status.cpu_limit_w = Some(value);
                        effects.push(Effect::CpuSet(value));
                        ActuatorState::Verified
                    }
                    WriteVerdict::Mismatch { .. } if suppress => ActuatorState::Unverifiable,
                    WriteVerdict::Mismatch { .. } => ActuatorState::Mismatch,
                    WriteVerdict::Unreadable | WriteVerdict::Unverifiable => {
                        ActuatorState::Unverifiable
                    }
                };
                let outcome = auto.cpu_verdict.observe(verdict, suppress);
                let error = cpu.err_c;
                let released = outcome == VerdictOutcome::Released;
                let _ = auto;
                if released {
                    if let Err(error) = actuator.restore_stock() {
                        tracing::warn!(
                            "auto: CPU stock restore on verdict release failed: {error}"
                        );
                    }
                    self.status.cpu_limit_w = None;
                }
                self.apply_verdict_outcome(true, outcome, error, effects, cause);
            }
        }
        let gpu_due = self
            .auto
            .as_ref()
            .expect("auto")
            .last_gpu_write_t_mono
            .is_none_or(|last| s.t_mono - last >= 2.0);
        let requested = gpu.cap.round() as u32;
        let gpu_changed = self.status.gpu_max_mhz != Some(requested);
        let mut gpu_released = false;
        let mut gpu_corrective = false;
        // Pair this acquired clock sample before issuing this tick's command.
        // The first sample after entry/resume has no completed command and is
        // intentionally Unverifiable.
        if s.gpu_mhz_valid && !s.resumed {
            let auto = self.auto.as_mut().expect("auto");
            let verifier = auto
                .gpu_verifier
                .get_or_insert_with(|| GpuLockVerifier::new(requested));
            let verdict = verifier.verify_paired(
                s.gpu_util_pct,
                s.gpu_sm_mhz.round() as u32,
                s.t_mono,
                auto.gpu_commands.make_contiguous(),
            );
            let suppress = auto.on_ac_suppress_until.is_some_and(|until| s.t_mono < until);
            auto.gpu_actuator_state = match verdict {
                WriteVerdict::Verified(_) => ActuatorState::Verified,
                WriteVerdict::Mismatch { .. } if suppress => ActuatorState::Unverifiable,
                WriteVerdict::Mismatch { .. } => ActuatorState::Mismatch,
                WriteVerdict::Unreadable | WriteVerdict::Unverifiable => ActuatorState::Unverifiable,
            };
            let outcome = auto.gpu_verdict.observe(verdict, suppress);
            let error = gpu.err_c;
            let released = outcome == VerdictOutcome::Released;
            gpu_released = released;
            gpu_corrective = outcome == VerdictOutcome::Mismatch;
            let _ = auto;
            if released {
                if let Some(actuator) = self.guard.gpu.as_mut()
                    && let Err(error) = actuator.release()
                {
                    tracing::warn!("auto: GPU release on verdict release failed: {error}");
                }
                self.status.gpu_max_mhz = None;
            }
            self.apply_verdict_outcome(false, outcome, error, effects, cause);
        }
        if gpu.write_allowed && !gpu_released
            && (gpu.write_immediately || gpu_corrective || (gpu_due && gpu_changed))
            && !self.shutting_down()
        {
            if let Some(actuator) = self.guard.gpu.as_mut() {
                if actuator.set_max_clock(requested).is_ok() {
                    let applied = actuator.applied().unwrap_or(requested);
                    self.record_gpu_command(s, applied);
                    self.status.gpu_max_mhz = Some(applied);
                    effects.push(Effect::GpuSet(applied));
                } else {
                    let completed_at = self.command_completed_at(s);
                    self.auto.as_mut().expect("auto").last_gpu_write_t_mono = Some(completed_at);
                }
            }
        }
        cause.get_or_insert("auto:device_loops");
    }

    /// Persist a paired warm start only after both device feedback groups and
    /// the fan have settled at this view's duty.  The entry is advisory: it
    /// never writes hardware and is only consumed at a later Auto entry.
    fn observe_v4_steady(
        &mut self,
        s: &Sample,
        cpu: DeviceDecision,
        gpu: DeviceDecision,
        target_duty: u8,
        gpu_guarded: bool,
    ) {
        let Some(view) = s.fanctrl.as_ref() else {
            let auto = self.auto.as_mut().expect("auto");
            auto.fan_window.clear();
            auto.steady_window.clear();
            auto.steady_key = None;
            auto.refinement_window.clear();
            auto.refinement_key = None;
            return;
        };
        let warm_key = warm_start_key(&view.strategy, target_duty, s.on_ac);
        let refinement_key = warm_key.clone();
        let rpm = s.fan_valid.then(|| s.max_fan_rpm()).filter(|rpm| rpm.is_finite());
        let groups_settled = self.status.t_star_c.zip(cpu.group_c).zip(gpu.group_c)
            .is_some_and(|((target, cpu_c), gpu_c)| {
                (cpu_c - target).abs() <= 1.0 && (gpu_c - target).abs() <= 1.0
            });
        let target_rpm = self.duty_rpm_table.rpm_for_duty(target_duty);
        let auto = self.auto.as_mut().expect("auto");
        let qualifies = view.active
            && s.fanctrl_freshness == Freshness::Fresh
            && view.speed_pct == target_duty
            && groups_settled
            && !gpu_guarded && !auto.cpu_hot
            && rpm.is_some()
            && self.status.cpu_limit_w.is_some()
            && self.status.gpu_max_mhz.is_some()
            && !matches!(cpu.hold, Hold::ActuatorMismatch)
            && !matches!(gpu.hold, Hold::ActuatorMismatch)
            && !auto.cpu_verdict.in_episode() && !auto.gpu_verdict.in_episode();
        if let Some(rpm) = rpm {
            auto.fan_window.push_back(rpm);
            if auto.fan_window.len() > FAN_SMOOTH_N { auto.fan_window.pop_front(); }
        } else {
            auto.fan_window.clear();
        }
        let smoothed = auto.fan_window.iter().sum::<f64>() / auto.fan_window.len().max(1) as f64;
        // Each bounded sliding window starts over on a key/gate change.
        // Once full, every further qualified stable tick can refine/save.
        let observe = |window: &mut std::collections::VecDeque<f64>| {
            window.push_back(smoothed);
            if window.len() > STEADY_WINDOW_N { window.pop_front(); }
            if window.len() != STEADY_WINDOW_N { return None; }
            let values: Vec<_> = window.iter().copied().collect();
            if population_stdev(&values) >= STEADY_WINDOW_STDEV_MAX_RPM { return None; }
            let mean = values.iter().sum::<f64>() / values.len() as f64;
            Some(mean)
        };
        let refine = if auto.refinement_key.as_ref() != Some(&refinement_key) || !qualifies {
            auto.refinement_key = Some(refinement_key);
            auto.refinement_window.clear();
            None
        } else {
            observe(&mut auto.refinement_window)
        };
        let warm_qualifies = qualifies && view.speed_pct == target_duty
            && (smoothed - target_rpm).abs() <= STEADY_WINDOW_STDEV_MAX_RPM;
        let warm = if auto.steady_key.as_ref() != Some(&warm_key) || !warm_qualifies {
            auto.steady_key = Some(warm_key.clone());
            auto.steady_window.clear();
            false
        } else {
            observe(&mut auto.steady_window).is_some()
        };
        if let Some(mean_rpm) = refine {
            // The daemon held this exact requested duty for the full
            // window; the table retains its independent 25% outlier gate.
            self.duty_rpm_table.refine(target_duty, mean_rpm);
        }
        if warm {
            self.persisted_warm_start.insert(warm_key, WarmStartEntry {
                cpu_cap_w: self.status.cpu_limit_w.expect("qualified CPU cap"),
                gpu_lock_mhz: self.status.gpu_max_mhz.expect("qualified GPU cap"),
            });
        }
        if refine.is_some() || warm { self.save_persisted_state(); }
    }

    /// Steady-window detector for passive warm-start/refinement (design
    /// §2.3). Accumulates `rpm_smoothed` into `AutoState::steady_window`
    /// while every gate holds this tick: `active`, the view's own
    /// `speed_pct == target_duty` (a window whose achieved duty differs
    /// from the target's must never write into the target's entry — a
    /// `GPU HOT` episode or a device bound can run fw-fanctrl a tread away
    /// from what `target_duty` names), no guard override (`GuardState::
    /// gpu_hot`), and `u` off both bounds. Any gate miss, or a warm-start
    /// key change (strategy/target_duty/on_ac — §2.4: a re-key restarts the
    /// window under the new key without touching `u`), clears the window.
    /// Once the window reaches [`STEADY_WINDOW_N`] samples, a population
    /// stdev under [`STEADY_WINDOW_STDEV_MAX_RPM`] records the current `u`
    /// into `warm_start[key]` and refines `duty_rpm_table` toward the
    /// window's mean — every tick the window stays this settled, not just
    /// once (§2.4: "the current `u` is written ... whenever the loop has
    /// been steady", i.e. the latest settled device pair).
    fn sync_bool_flag(&mut self, flag: StatusFlag, active: bool) {
        if active {
            self.add_flag(flag);
        } else {
            self.remove_flag(flag);
        }
    }

    /// Applies one actuator verdict to its DeviceLoop and release state.
    fn apply_verdict_outcome(
        &mut self,
        is_cpu: bool,
        outcome: VerdictOutcome,
        _current_error: Option<f64>,
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

    /// One calibrating-mode sample: build this tick's `PerDeviceCalibContext`, feed
    /// the runner, execute its effects, refresh the wizard progress in
    /// status.
    fn on_calib_sample(&mut self, s: &Sample) -> Vec<Effect> {
        let before = self.status.clone();
        let ctx = self.build_calib_context(s);
        let runner_effects = match self.calib.as_mut() {
            Some(runner) => runner.on_sample(s, &ctx),
            None => {
                tracing::warn!("Calibrating mode without a runner; returning to Monitor");
                self.end_calibration();
                Vec::new()
            }
        };
        let diagnostic = self.calib.as_ref().and_then(|runner| runner.diagnostics()).map(|snapshot| Effect::Calibration {
            diagnostics: Box::new(snapshot.clone()),
            gpu_util_pct: s.gpu_util_pct,
            view_fresh: s.fanctrl_freshness == Freshness::Fresh,
            view_changed: s.fanctrl_view_changed,
            reconciliation_ma_c: self.calib_replica.as_ref().and_then(EcReplica::reconciliation_ma),
            socket_ma_c: s.fanctrl.as_ref().map(|view| view.ma_temperature),
        });
        let cause = self.apply_calib_effects(runner_effects, Some(s));
        self.sync_calib_status();
        let mut effects = Vec::new();
        effects.extend(diagnostic);
        self.drain_flag_effects(&mut effects);
        if self.status != before {
            effects.push(Effect::StatusChanged { cause: cause.unwrap_or("calib:progress") });
        } else if let Some(cause) = cause {
            effects.push(Effect::Noted { cause });
        }
        effects
    }

    fn build_calib_context(&mut self, s: &Sample) -> PerDeviceCalibContext {
        let mut cpu_cap_reset_reason = None;
        if self.calib.is_some() && !self.shutting_down()
            && (self.calib_cpu_checked_at_s.is_none_or(|at| s.t_mono - at >= REASSERT_PERIOD_S)
                || self.calib.as_ref().is_some_and(|runner| runner.finishing_response(s.t_mono)))
            && let (Some(w), Some(cpu)) = (self.status.cpu_limit_w, self.guard.cpu.as_ref())
        {
            // Read-only checks are harmless under a hot guard, but a repair
            // must not precede the runner's thermal abort/restore path.
            let hot = (s.cpu_temp_valid && s.cpu_temp_c >= self.config.cpu_hot_c)
                || (s.gpu_temp_valid && s.gpu_temp_c >= self.config.gpu_hot_c)
                || (s.ec_valid && s.ec.as_ref().is_some_and(|ec| ec.max_c >= 95));
            let check = cpu.maintain_sustained_mw((w * 1000.0).round() as u32, || !hot && !self.shutting_down());
            self.calib_cpu_checked_at_s = Some(self.command_completed_at(s));
            self.calib_cpu_readback = Some(check.observed);
            self.calib_cpu_verified = matches!(check.verdict(), WriteVerdict::Verified(_));
            if let WriteVerdict::Mismatch { field, commanded, read } = check.observed {
                let reason = format!("CPU cap reset: {field} read {read:.2}W, expected {commanded:.2}W; repair {:?}", check.repair);
                tracing::warn!("calib: {reason}");
                cpu_cap_reset_reason = Some(reason);
            } else if !self.calib_cpu_verified {
                tracing::warn!("calib: CPU cap read-back failed: {:?}", check.observed);
            }
        }
        if self.calib_frozen_strategy.is_none()
            && let Some(view) = s.fanctrl.as_ref().filter(|view| !view.strategy.is_empty() && view.ma_interval > 0)
        {
            self.calib_frozen_strategy = Some(view.strategy.clone());
            self.calib_frozen_interval = Some(view.ma_interval);
            if let Some(replica) = self.calib_replica.as_mut() {
                replica.set_interval(view.ma_interval as usize);
                replica.reset(
                    Some(view.ma_temperature),
                    s.ec.as_ref().and_then(|ec| ec.cpu_group_c),
                    s.ec.as_ref().and_then(|ec| ec.gpu_group_c),
                );
            }
        }
        if let Some(replica) = self.calib_replica.as_mut() {
            if self.calib_ec_slope_window.len() >= EC_SLOPE_WINDOW_S {
                self.calib_ec_slope_window.pop_front();
            }
            if let Some(raw_max) = s.ec.as_ref().and_then(|ec| ec.reconciliation_max_c) {
                self.calib_ec_slope_window.push_back(f64::from(raw_max));
            }
            replica.tick(s.ec.as_ref());

            let observation = match (s.fanctrl.as_ref(), s.ec.as_ref()) {
                (Some(view), Some(ec))
                    if s.fanctrl_freshness == Freshness::Fresh && s.fanctrl_view_changed =>
                {
                    let slope = self
                        .calib_ec_slope_window
                        .front()
                        .zip(self.calib_ec_slope_window.back())
                        .map_or(0.0, |(first, last)| {
                            (last - first)
                                / self.calib_ec_slope_window.len().saturating_sub(1).max(1) as f64
                        });
                    let gap_s = view
                        .all_observed_at
                        .map(|captured| {
                            Instant::now()
                                .saturating_duration_since(captured)
                                .as_secs_f64()
                        })
                        .unwrap_or(0.0);
                    if slope >= RECONCILIATION_SKIP_SLOPE_C_PER_S
                        || gap_s >= RECONCILIATION_SKIP_GAP_S
                    {
                        ReconciliationObservation::Skipped
                    } else {
                        ReconciliationObservation::Scored {
                            max_matches: ec.reconciliation_max_c.is_some_and(|raw_max| {
                                (f64::from(raw_max) - view.temperature).abs() <= 1.0
                            }),
                            ma_matches: replica
                                .reconciliation_ma()
                                .map(|ma| (ma - view.ma_temperature).abs() <= 1.0),
                            socket_ma: view.ma_temperature,
                        }
                    }
                }
                _ => ReconciliationObservation::Skipped,
            };
            if let Some(reseed) = replica.score_reconciliation(observation).reseed_ma {
                replica.reset_after_mismatch_clear(
                    Some(reseed),
                    s.ec.as_ref().and_then(|ec| ec.cpu_group_c),
                    s.ec.as_ref().and_then(|ec| ec.gpu_group_c),
                );
            }
        }
        self.calib_gpu_verified = if s.gpu_mhz_valid && !s.resumed {
            self.calib_gpu_verifier.as_mut().map(|verifier| matches!(
                verifier.verify_paired(
                    s.gpu_util_pct,
                    s.gpu_sm_mhz.round() as u32,
                    s.t_mono,
                    self.calib_gpu_commands.make_contiguous(),
                ),
                WriteVerdict::Verified(_)
            )).unwrap_or(false)
        } else { false };
        let fanctrl_active =
            s.fanctrl_freshness == Freshness::Fresh && s.fanctrl.as_ref().is_some_and(|v| v.active);
        let argmax_controllable = s.ec.as_ref().is_some_and(|e| e.argmax.is_controllable());
        PerDeviceCalibContext {
            cpu_cap_w: self.status.cpu_limit_w,
            cpu_cap_verified: self.calib_cpu_verified,
            cpu_cap_checked_at_s: self.calib_cpu_checked_at_s,
            cpu_cap_readback: self.calib_cpu_readback,
            cpu_cap_reset_reason,
            cpu_cap_completed_at_s: self.calib_cpu_completed_at_s,
            gpu_cap_mhz: self.status.gpu_max_mhz,
            gpu_cap_verified: self.calib_gpu_verified,
            gpu_cap_completed_at_s: self.calib_gpu_completed_at_s,
            use_current_caps: self.calib_started_from_auto,
            cpu_floor_w: self.config.cpu_floor_w,
            gpu_floor_mhz: self.config.gpu_floor_mhz,
            cpu_max_w: self.config.cpu_max_w,
            gpu_max_mhz: self.config.gpu_max_mhz,
            cpu_group_c: self.calib_replica.as_ref().and_then(EcReplica::cpu_group_ma),
            gpu_group_c: self.calib_replica.as_ref().and_then(EcReplica::gpu_group_ma),
            ec_mismatch: self.calib_replica.as_ref().is_some_and(EcReplica::ec_mismatch),
            fanctrl_active,
            argmax_controllable,
            cpu_hot_c: self.config.cpu_hot_c,
            gpu_hot_c: self.config.gpu_hot_c,
            strategy: s.fanctrl.as_ref().map(|view| view.strategy.clone()),
            ma_interval: s.fanctrl.as_ref().map(|view| view.ma_interval),
        }
    }

    fn apply_calib_effects(
        &mut self,
        effects: Vec<RunnerEffect>,
        s: Option<&Sample>,
    ) -> Option<&'static str> {
        /// Higher wins when a batch carries several notable events (the
        /// final batch may contain releases, fitted gains, persistence, and finish).
        fn rank(cause: &str) -> u8 {
            match cause {
                "calib:skipped" => 5,
                "calib:fitted" => 4,
                "calib:finished" => 3,
                _ => 1,
            }
        }
        fn raise(cur: &mut Option<&'static str>, c: &'static str) {
            if cur.is_none_or(|old| rank(c) > rank(old)) {
                *cur = Some(c);
            }
        }
        if let (Some(outcome), Some(view)) = (
            self.status.calib_outcome.as_mut(), s.and_then(|sample| sample.fanctrl.as_ref()),
        ) {
            let key = format!("{}:{}", view.strategy, view.ma_interval);
            outcome.retained_cpu.get_or_insert_with(|| self.config.cpu_gains
                .or_else(|| self.persisted_cpu_gains.get(&key).copied())
                .unwrap_or_else(|| default_gains::<W>(view.ma_interval)));
            outcome.retained_gpu.get_or_insert_with(|| self.config.gpu_gains
                .or_else(|| self.persisted_gpu_gains.get(&key).copied())
                .unwrap_or_else(|| default_gains::<Mhz>(view.ma_interval)));
        }
        let mut cause: Option<&'static str> = None;
        let mut ended = false;
        for effect in effects {
            match effect {
                RunnerEffect::SetCpuMaxWatts(w) if self.shutting_down() => {
                    tracing::debug!("calib: shutting down; SetCpuMaxWatts({w}) skipped");
                }
                RunnerEffect::SetCpuMaxWatts(w) => {
                    self.calib_cpu_verified = false;
                    match self.guard.cpu.as_ref() {
                        None => {
                            tracing::warn!("calib: no CPU actuator; SetCpuMaxWatts({w}) skipped")
                        }
                        Some(cpu) => match cpu.set_sustained_mw((w * 1000.0).round() as u32) {
                            WriteVerdict::Verified(applied) => {
                                self.status.cpu_limit_w = Some(applied);
                                self.calib_cpu_verified = true;
                                self.calib_cpu_completed_at_s =
                                    s.map(|sample| self.command_completed_at(sample));
                                self.calib_cpu_checked_at_s = self.calib_cpu_completed_at_s;
                                self.calib_cpu_readback = Some(WriteVerdict::Verified(applied));
                            }
                            verdict => {
                                self.calib_cpu_checked_at_s = s.map(|sample| self.command_completed_at(sample));
                                self.calib_cpu_readback = Some(verdict);
                                tracing::warn!(
                                    "calib: SetCpuMaxWatts({w}) not verified: {verdict:?}"
                                )
                            }
                        },
                    }
                }
                // Stop fence (roast-pr-2 finding 2) on the write arm only.
                RunnerEffect::SetGpuMaxClock(mhz) if self.shutting_down() => {
                    tracing::debug!("calib: shutting down; SetGpuMaxClock({mhz}) skipped");
                }
                RunnerEffect::SetGpuMaxClock(mhz) => {
                    self.calib_gpu_verified = false;
                    let completed_at = s.map(|sample| self.command_completed_at(sample));
                    match self.guard.gpu.as_mut() {
                        None => {
                            tracing::warn!("calib: no GPU actuator; SetGpuMaxClock({mhz}) skipped")
                        }
                        Some(gpu) => match gpu.set_max_clock(mhz) {
                            Ok(()) => {
                                self.status.gpu_max_mhz = gpu.applied();
                                if let (Some(applied), Some(completed_at)) =
                                    (gpu.applied(), completed_at)
                                {
                                    self.calib_gpu_completed_at_s = Some(completed_at);
                                    self.calib_gpu_command_generation =
                                        self.calib_gpu_command_generation.saturating_add(1);
                                    self.calib_gpu_commands.push_back(GpuCommandEvidence::new(
                                        applied,
                                        completed_at,
                                        self.calib_gpu_command_generation,
                                    ));
                                    while self.calib_gpu_commands.len() > 3 {
                                        self.calib_gpu_commands.pop_front();
                                    }
                                    self.calib_gpu_verifier
                                        .get_or_insert_with(|| GpuLockVerifier::new(applied));
                                }
                            }
                            Err(e) => {
                                tracing::warn!("calib: SetGpuMaxClock({mhz}) failed: {e}")
                            }
                        },
                    }
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
                RunnerEffect::FittedDevice { device, gains } => {
                    tracing::info!("calib: {device:?} step-test fitted {gains:?}");
                    let view = s.and_then(|sample| sample.fanctrl.as_ref());
                    let interval = view.map_or(DEFAULT_MA_INTERVAL as u32, |view| view.ma_interval);
                    let key = view.map(|view| format!("{}:{}", view.strategy, view.ma_interval));
                    let before = match device {
                        crate::calib::step::CalibDevice::Cpu => key.as_ref()
                            .and_then(|key| self.persisted_cpu_gains.get(key)).copied()
                            .unwrap_or_else(|| default_gains::<W>(interval)),
                        crate::calib::step::CalibDevice::Gpu => key.as_ref()
                            .and_then(|key| self.persisted_gpu_gains.get(key)).copied()
                            .unwrap_or_else(|| default_gains::<Mhz>(interval)),
                    };
                    let overridden = match device {
                        crate::calib::step::CalibDevice::Cpu => self.config.cpu_gains.is_some(),
                        crate::calib::step::CalibDevice::Gpu => self.config.gpu_gains.is_some(),
                    };
                    let outcome = self.status.calib_outcome.get_or_insert_with(CalibOutcome::default);
                    outcome.changes.push(CalibGainChange { device, before, after: gains });
                    if overridden {
                        outcome.notes.push(format!("{device:?}: config override remains active; remove it to use the new calibration gains."));
                    }
                    raise(&mut cause, "calib:fitted");
                }
                RunnerEffect::Retrying(reason) => {
                    tracing::warn!("calib: {reason}");
                    self.status.calib_outcome.get_or_insert_with(CalibOutcome::default).notes.push(reason);
                    raise(&mut cause, "calib:retry");
                }
                RunnerEffect::Noted(reason) => {
                    tracing::info!("calib: {reason}");
                    self.status.calib_outcome.get_or_insert_with(CalibOutcome::default)
                        .errors.push(reason);
                    raise(&mut cause, "calib:skipped");
                }
                RunnerEffect::SaveState(state) => {
                    self.calibrated_at = state.calibrated_at.clone();
                    self.persisted_cpu_gains = state.cpu_gains;
                    self.persisted_gpu_gains = state.gpu_gains;
                    let current_key = s.and_then(|sample| sample.fanctrl.as_ref())
                        .map(|view| format!("{}:{}", view.strategy, view.ma_interval));
                    if current_key.as_ref().is_some_and(|key| {
                        self.persisted_cpu_gains.contains_key(key)
                            && self.persisted_gpu_gains.contains_key(key)
                    }) {
                        self.remove_flag(StatusFlag::NotCalibrated);
                    } else {
                        self.add_flag(StatusFlag::NotCalibrated);
                    }
                    let saved = self.try_save_persisted_state();
                    let outcome = self.status.calib_outcome.get_or_insert_with(CalibOutcome::default);
                    outcome.applied = true;
                    outcome.saved = saved.is_ok();
                    if let Err(error) = saved {
                        outcome.errors.push(format!("Could not save calibration: {error}"));
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
            self.apply_calib_effects(runner.abort(), None);
            self.end_calibration();
        }
        // Auto exits hard: dropping AutoState means no DeviceLoops step can
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

    /// Write the full calibration/loop state to the state file. Called when
    /// Auto exits through [`exit_auto_and_persist`](Self::exit_auto_and_persist)
    /// and from the calibration completion paths, never per-update (no disk
    /// churn). Save failure is warned, not fatal: the in-memory state still
    /// carries the session.
    fn save_persisted_state(&self) {
        if let Err(error) = self.try_save_persisted_state() {
            tracing::warn!("auto: state save to {} failed: {error}", self.state_path.display());
        }
    }

    fn try_save_persisted_state(&self) -> std::io::Result<()> {
        let state = PersistedState {
            calibrated_at: self.calibrated_at.clone(),
            cpu_gains: self.persisted_cpu_gains.clone(),
            gpu_gains: self.persisted_gpu_gains.clone(),
            duty_rpm_table: self.duty_rpm_table.clone(),
            warm_start: self.persisted_warm_start.clone(),
            t_star_last_good: self.persisted_t_star_last_good.clone(),
        };
        state.save(&self.state_path)
    }

    /// Drop the Auto loop state and write the state file (a quit from Auto
    /// is covered because every exit path funnels through here). No-op when
    /// not in Auto: a Monitor/Manual session has nothing to drop.
    fn exit_auto_and_persist(&mut self) {
        if self.auto.take().is_some() {
            self.save_persisted_state();
            self.status.cpu = None;
            self.status.gpu = None;
            self.status.tstar_state = None;
            self.status.telemetry_flags.clear();
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
        // Loop, guard, and verdict flags are owned by `on_auto_sample`,
        // which stops running the moment
        // `self.auto` drops here — without an explicit clear, a flag that
        // happened to be up at exit (e.g. `GpuHot` mid-episode) would stay
        // stuck forever, since nothing outside Auto ever touches it again.
        for flag in [
            StatusFlag::NotCalibrated,
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
        self.status.t_star_c = None;
        self.status.tstar_state = None;
        self.status.cpu = None;
        self.status.gpu = None;
        self.status.telemetry_flags.clear();
        self.status.ec_ma_c = None;
        self.status.ec_argmax = None;
        self.status.duty_cmd = None;
        self.status.snapped_rpm = 0.0;
        self.status.strategy = None;
        self.stick_violations = 0;
        self.last_reassert = None;
    }

    /// Mirror the runner's progress into status (None once it's gone).
    fn sync_calib_status(&mut self) {
        self.status.calib = self.calib.as_ref().map(CalibRunner::progress);
    }

    /// Drop the runner, stop the burner and clear the wizard. A Monitor
    /// origin returns to Monitor; an Auto origin resumes the exact suspended
    /// loop/TStar objects after one no-write sample-time resynchronisation.
    fn end_calibration(&mut self) {
        if let Some(burner) = self.burner.take() {
            burner.stop();
        }
        self.calib = None;
        self.calib_replica = None;
        self.calib_ec_slope_window.clear();
        self.calib_frozen_strategy = None;
        self.calib_frozen_interval = None;
        self.calib_cpu_verified = false;
        self.calib_cpu_checked_at_s = None;
        self.calib_cpu_readback = None;
        self.calib_cpu_completed_at_s = None;
        self.calib_gpu_verified = false;
        self.calib_gpu_completed_at_s = None;
        self.calib_gpu_verifier = None;
        self.calib_gpu_commands.clear();
        self.calib_gpu_command_generation = 0;
        if self.calib_started_from_auto && self.auto.is_some() {
            if let Some(auto) = self.auto.as_mut() {
                if let Some(cpu) = self.status.cpu_limit_w {
                    auto.cpu_loop.note_applied(cpu);
                }
                if let Some(gpu) = self.status.gpu_max_mhz {
                    auto.gpu_loop.note_applied(f64::from(gpu));
                }
                auto.last_sample_t_mono = None;
                auto.last_cpu_write_t_mono = None;
                auto.last_gpu_write_t_mono = None;
                auto.cpu_actuator_state = ActuatorState::Unverifiable;
                auto.gpu_actuator_state = ActuatorState::Unverifiable;
                auto.gpu_commands.clear();
                auto.gpu_verifier = None;
                auto.cpu_verdict = VerdictState::default();
                auto.gpu_verdict = VerdictState::default();
            }
            self.status.mode = Mode::Auto;
            self.calib_reentry_hold = true;
        } else {
            self.status.mode = Mode::Monitor;
            self.calib_reentry_hold = false;
        }
        self.calib_started_from_auto = false;
        self.status.calib = None;
    }

    /// The first sample after an Auto-originated calibration is a held
    /// resynchronisation point: rebuild every observation history from this
    /// sample while leaving the suspended loops/TStar and applied pair
    /// untouched. The caller returns before any actuator path runs.
    fn reseed_auto_after_calibration(&mut self, s: &Sample) {
        let Some(auto) = self.auto.as_mut() else { return; };
        let view = s
            .fanctrl
            .as_ref()
            .filter(|_| s.fanctrl_freshness == Freshness::Fresh);
        if let Some(view) = view {
            auto.replica.set_interval(view.ma_interval as usize);
        }
        auto.replica.reset(
            view.map(|view| view.ma_temperature),
            s.ec.as_ref().and_then(|ec| ec.cpu_group_c),
            s.ec.as_ref().and_then(|ec| ec.gpu_group_c),
        );
        auto.view_initialised = view.is_some();

        auto.cpu_draw_window.clear();
        if s.cpu_pkg_w.is_finite() && s.cpu_pkg_w > 0.0 {
            auto.cpu_draw_window.push_back(s.cpu_pkg_w);
        }
        auto.fan_window.clear();
        auto.fan_window.push_back(if s.fan_valid {
            s.max_fan_rpm()
        } else {
            f64::NAN
        });
        auto.ec_slope_window.clear();
        if let Some(raw_max) = s.ec.as_ref().and_then(|ec| ec.reconciliation_max_c) {
            auto.ec_slope_window.push_back(f64::from(raw_max));
        }
        auto.steady_window.clear();
        auto.steady_key = None;
        auto.refinement_window.clear();
        auto.refinement_key = None;
        auto.last_on_ac = Some(s.on_ac);
        auto.on_ac_suppress_until = None;
        auto.cpu_hot_streak = u8::from(
            s.cpu_temp_valid && s.cpu_temp_c >= self.config.cpu_hot_c,
        );
        auto.cpu_hot = false;
        auto.guards = Guards::new(self.config.gpu_hot_c, self.config.nvme_hot_c);
        auto.guards.step(
            s.gpu_temp_valid.then_some(s.gpu_temp_c),
            s.nvme_temp_c,
        );
        auto.last_sample_t_mono = Some(s.t_mono);
    }

    /// Reapply whatever limits are currently commanded (same values). Errors
    /// are warned — the periodic retry IS the recovery. `None` if nothing was
    /// commanded; otherwise `Some(all_calls_succeeded)` so telemetry can
    /// distinguish real reasserts from failed attempts.
    fn reassert_actuators(&mut self, s: &Sample) -> Option<bool> {
        // Stop fence (roast-pr-2 finding 2): a reassert re-issues the cap we
        // are about to restore away from. Nothing was attempted, so report
        // "nothing commanded" rather than a failed attempt.
        if self.shutting_down() {
            return None;
        }
        let mut any = false;
        let mut all_ok = true;
        let mut cpu_outcome = None;
        if let (Some(w), Some(cpu)) = (self.status.cpu_limit_w, self.guard.cpu.as_ref()) {
            let check = cpu.maintain_sustained_mw((w * 1000.0).round() as u32, || !self.shutting_down());
            tracing::debug!("CPU cap maintenance: requested {w} W, {check:?}");
            // A read-only match is not a reassert. Failed reads are still
            // surfaced as failed maintenance, without an unverified write.
            any |= check.repair.is_some() || !matches!(check.observed, WriteVerdict::Verified(_));
            if !matches!(check.verdict(), WriteVerdict::Verified(_)) {
                all_ok = false;
                tracing::warn!("CPU cap maintenance failed ({w} W): {check:?}");
            }
            let completed_at = self.command_completed_at(s);
            if check.repair.is_some() {
                tracing::info!("CPU cap repaired ({w} W): {check:?}");
            }
            if let Some(auto) = self.auto.as_mut() {
                let suppress = auto.on_ac_suppress_until.is_some_and(|until| s.t_mono < until);
                auto.cpu_actuator_state = match check.verdict() {
                    WriteVerdict::Verified(_) => ActuatorState::Verified,
                    WriteVerdict::Mismatch { .. } if !suppress => ActuatorState::Mismatch,
                    _ => ActuatorState::Unverifiable,
                };
                // Blocking reads also bound subsequent write attempts.
                auto.last_cpu_write_t_mono = Some(completed_at);
                let outcome = auto.cpu_verdict.observe(check.verdict(), suppress);
                cpu_outcome = Some(outcome);
                if outcome == VerdictOutcome::Released {
                    if let Err(error) = cpu.restore_stock() {
                        tracing::warn!("CPU maintenance release failed: {error}");
                    }
                    self.status.cpu_limit_w = None;
                }
            }
        }
        if let Some(outcome) = cpu_outcome {
            let mut events = Vec::new();
            self.apply_verdict_outcome(true, outcome, None, &mut events, &mut None);
            self.pending_effects.extend(events);
        }
        if let Some(mhz) = self.status.gpu_max_mhz {
            if let Some(gpu) = self.guard.gpu.as_mut() {
                any = true;
                if let Err(e) = gpu.set_max_clock(mhz) {
                    all_ok = false;
                    tracing::warn!("reassert: GPU max clock ({mhz} MHz) failed: {e}");
                    let completed_at = self.command_completed_at(s);
                    if let Some(auto) = self.auto.as_mut() {
                        auto.last_gpu_write_t_mono = Some(completed_at);
                    }
                } else {
                    let applied = gpu.applied().unwrap_or(mhz);
                    self.record_gpu_command(s, applied);
                }
            }
        }
        any.then_some(all_ok)
    }

    /// Set a flag. A GENUINE insertion (not already set) also records an
    /// `Effect::Flagged { active: true }` into `pending_effects` — plan Task
    /// 14 promises a telemetry Flag line on every status-flag transition.
    fn add_flag(&mut self, flag: StatusFlag) {
        if !self.status.flags.contains(&flag) {
            self.status.flags.push(flag);
            self.pending_effects.push(Effect::Flagged {
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
            self.pending_effects.push(Effect::Flagged {
                flag: flag.as_str(),
                active: false,
            });
        }
    }

    /// Move the flag transitions recorded since the last drain into
    /// `effects`. Every `on_command`/`on_sample` return path that could have
    /// touched a flag drains, so each transition is emitted exactly once.
    fn drain_flag_effects(&mut self, effects: &mut Vec<Effect>) {
        effects.append(&mut self.pending_effects);
    }
}

/// Thread shell: `select!` over samples and commands, mapping effects to
/// `Event::Status` sends and telemetry `Decision` records. On loop exit
/// (Quit or command-channel disconnect) it restores hardware and flips
/// `restored` so main's `FinalRestore` knows to stand down.
///
/// `shutdown` is main's shutdown flag, installed on the controller as its
/// stop fence (roast-pr-2 finding 2). Main's join of this thread is BOUNDED,
/// so this thread can outlive both the join and `FinalRestore`'s restore;
/// the fence guarantees an orphaned thread issues no further actuator write.
pub fn spawn<R: Runner + Send + 'static>(
    controller: Controller<R>,
    sample_rx: Receiver<Event>,
    cmd_rx: Receiver<Command>,
    ui_tx: Sender<Event>,
    telemetry: Arc<Mutex<Option<Telemetry>>>,
    restored: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("controller".into())
        .spawn(move || {
            let mut controller = controller;
            controller.set_shutdown_flag(Arc::clone(&shutdown));
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
                            // Drain, don't service: once shutdown began (and
                            // so Quit is pending) queued samples are dropped
                            // on the floor. `on_sample` fences too — this
                            // arm skips the work and the telemetry record as
                            // well (roast-pr-2 finding 2).
                            if !shutdown.load(Ordering::Relaxed) {
                                let effects = controller.on_sample(&s);
                                apply_effects(
                                    &effects, &controller, t_mono, &ui_tx, &telemetry,
                                );
                            }
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

fn decision_telemetry_flags(status: &ControlStatus) -> Vec<TelemetryFlag> {
    let mut flags = status.telemetry_flags.clone();
    flags.extend(
        status
            .flags
            .iter()
            .map(|flag| TelemetryFlag::legacy(flag.as_str())),
    );
    flags
}

#[cfg(test)]
pub(crate) fn test_decision_telemetry_flags(status: &ControlStatus) -> Vec<TelemetryFlag> {
    decision_telemetry_flags(status)
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
    // Status-flag transitions in this batch (watchdog or otherwise): each
    // becomes a standalone Record::Flag line (in addition to the Decision
    // carrying the full list).
    let mut flagged: Vec<(&'static str, bool)> = Vec::new();
    for effect in effects {
        match effect {
            Effect::Calibration { diagnostics, gpu_util_pct, view_fresh, view_changed, reconciliation_ma_c, socket_ma_c } => {
                if let Some(t) = telemetry::lock(telemetry).as_mut() {
                    t.log(&Record::Calibration {
                        t_mono, diagnostics, gpu_util_pct: *gpu_util_pct,
                        view_fresh: *view_fresh, view_changed: *view_changed,
                        reconciliation_ma_c: *reconciliation_ma_c, socket_ma_c: *socket_ma_c,
                    });
                }
            }
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
    let decision = |cause: &'static str| {
        let flags = decision_telemetry_flags(status);
        Record::Decision {
            t_mono,
            mode: status.mode.as_str().to_string(),
            cpu_limit_w: status.cpu_limit_w,
            gpu_max_mhz: status.gpu_max_mhz,
            fan_target_rpm: status.fan_target_rpm,
            cause: cause.to_string(),
            flags,
            t_star: status.t_star_c,
            tstar_state: status.tstar_state,
            cpu: status.cpu.clone(),
            gpu: status.gpu.clone(),
        }
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
                t.log(&decision(cause));
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
        // A permanent loss of Curve (CurveInvalid) must not share the
        // informational SteepCurve severity (brief, design §2.7/§2.8).
        assert_eq!(flag_severity(StatusFlag::CurveInvalid), Severity::Warning);
        assert_eq!(flag_severity(StatusFlag::SteepCurve), Severity::Info);
        assert_ne!(
            flag_severity(StatusFlag::CurveInvalid),
            flag_severity(StatusFlag::SteepCurve)
        );
    }

    // --- Task 4: ControlStatus's new field set ---

    fn profile_fixture(name: &str) -> (PathBuf, PathBuf) {
        let dir = std::env::temp_dir().join(format!(
            "fw-fan-quiet-controller-test-{}-{name}",
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

    fn queue_cpu_reset(runner: &FakeRunner, slow_w: f64) {
        use crate::actuators::cmd::test_support::{output_with_stdout, ryzenadj_info_table};
        runner.push_result(Ok(output_with_stdout(&ryzenadj_info_table(slow_w, 53.0, slow_w))));
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
    fn checks_cpu_limit_every_10s_without_rewriting_a_match() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));
        let baseline = runner.calls().len();
        for t in [0.0, 5.0, 9.9] { assert!(ctl.on_sample(&sample_at(t)).is_empty()); }
        assert_eq!(runner.calls().len(), baseline);
        assert!(ctl.on_sample(&sample_at(10.1)).is_empty());
        assert_eq!(runner.calls().len(), baseline + 1);
        assert_eq!(ryzenadj_calls(&runner), vec![expected_args(20_000)]);
        assert!(ctl.on_sample(&sample_at(11.0)).is_empty());
        assert_eq!(runner.calls().len(), baseline + 1, "successful read advances cadence");
        queue_cpu_reset(&runner, 40.0);
        let effects = ctl.on_sample(&sample_at(20.2));
        assert!(has_reassert(&effects, "reassert"));
        assert_eq!(ryzenadj_calls(&runner), vec![expected_args(20_000), expected_args(20_000)]);
    }

    // --- roast-pr-2 finding 2: the abandoned-thread stop fence ------------

    /// The pin: main's controller join is BOUNDED, so this thread can still
    /// be alive after `FinalRestore` put the hardware back to stock. A
    /// sample serviced then would re-issue a CPU cap that nothing undoes.
    /// Without the fence this reasserts and the call count goes to 2.
    #[test]
    fn a_sample_after_shutdown_began_issues_no_actuator_write() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        let shutdown = Arc::new(AtomicBool::new(false));
        ctl.set_shutdown_flag(Arc::clone(&shutdown));

        ctl.on_command(Command::SetCpuW(20.0));
        assert_eq!(ryzenadj_calls(&runner).len(), 1);
        ctl.on_sample(&sample_at(0.0)); // reassert baseline

        // Main begins shutting down; the orphaned thread unwedges and finds
        // a queued sample that is well past the reassert period.
        shutdown.store(true, Ordering::Relaxed);
        let effects = ctl.on_sample(&sample_at(10.1));

        assert!(
            effects.is_empty(),
            "sample was serviced anyway: {effects:?}"
        );
        assert_eq!(
            ryzenadj_calls(&runner).len(),
            1,
            "an actuator write landed after shutdown began: {:?}",
            ryzenadj_calls(&runner)
        );
    }

    /// roast-pr-3: the fence at the top of `on_sample` is not enough. The
    /// resume reassert's GPU `set_max_clock` (untimed NVML in production)
    /// runs earlier in the SAME sample as the controller's CPU write, and
    /// main can raise `shutdown` while it is in flight — the primary Auto
    /// CPU write must re-check the flag itself. Modelled with a FakeGpu that
    /// raises the flag from inside `set_max_clock`. Fails before the fix
    /// with the controller overwriting the cap the reassert just re-issued.
    #[test]
    fn a_shutdown_raised_mid_sample_fences_the_auto_cpu_write() {
        let runner = FakeRunner::new();
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut gpu = FakeGpu::new();
        gpu.raise_on_set(Arc::clone(&shutdown));
        let gpu_calls = gpu.calls();
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
        ctl.set_shutdown_flag(Arc::clone(&shutdown));

        // Manual caps in force, then the reassert baseline pinned at t=0.
        ctl.on_command(Command::SetCpuW(20.0));
        ctl.on_command(Command::SetGpuMaxClock(2000));
        shutdown.store(false, Ordering::Relaxed); // the manual set raised it
        ctl.on_sample(&sample_at(0.0));
        assert_eq!(
            ctl.status().cpu_limit_w,
            Some(20.0),
            "premise: manual cap in force"
        );
        let gpu_sets_before = gpu_calls.lock().unwrap().len();

        // Enter Auto. Its first device-loop decision lands on the next sample, which
        // reports a RESUME: the resume reassert (CPU 20 re-write, then the
        // GPU re-set that raises `shutdown` mid-sample) runs BEFORE the
        // controller, whose 15 W CPU decision differs from the applied 20 W.
        // (The periodic reassert runs AFTER the controller, so it cannot
        // model this race; the resume path is the one that can.)
        ctl.on_command(Command::SetAuto(true));
        shutdown.store(false, Ordering::Relaxed);
        ctl.on_sample(&Sample {
            resumed: true,
            ..busy_at(REASSERT_PERIOD_S)
        });

        assert!(
            gpu_calls.lock().unwrap().len() > gpu_sets_before,
            "premise: the reassert re-set the GPU on this sample"
        );
        assert!(
            shutdown.load(Ordering::Relaxed),
            "premise: that GPU set raised the flag mid-sample"
        );
        assert_eq!(
            ctl.status().cpu_limit_w,
            Some(20.0),
            "the controller wrote a new CPU cap after shutdown was raised mid-sample"
        );
    }

    /// Same fence on the command path: main raises `shutdown` strictly
    /// before it sends `Quit`, so a manual key that raced shutdown must not
    /// land a cap either.
    #[test]
    fn a_manual_command_after_shutdown_began_issues_no_actuator_write() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        let shutdown = Arc::new(AtomicBool::new(true));
        ctl.set_shutdown_flag(shutdown);

        ctl.on_command(Command::SetCpuW(20.0));

        assert!(ryzenadj_calls(&runner).is_empty());
        assert_eq!(ctl.status().cpu_limit_w, None);
    }

    /// The restore path is deliberately NOT fenced: shutdown is exactly when
    /// it must run.
    #[test]
    fn restore_all_still_runs_with_the_shutdown_fence_raised() {
        let runner = FakeRunner::new();
        let (dir, profile) = profile_fixture("shutdown-fence-restore");
        let mut ctl = controller(&runner, profile);
        ctl.on_command(Command::SetCpuW(20.0));
        ctl.set_shutdown_flag(Arc::new(AtomicBool::new(true)));

        ctl.restore_all();

        assert_eq!(modprobe_reload_calls(&runner), 1, "stock was not restored");
        let _ = fs::remove_dir_all(dir);
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
        assert_eq!(ryzenadj_calls(&runner).len(), 1, "failed read must not cause a blind write");

        // The baseline still advanced: retry follows the normal 10 s cadence.
        assert!(ctl.on_sample(&sample_at(10.2)).is_empty());
        let effects = ctl.on_sample(&sample_at(20.2));
        assert!(effects.is_empty(), "matching read needs no rewrite: {effects:?}");
        assert_eq!(ryzenadj_calls(&runner).len(), 1);
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
        queue_cpu_reset(&runner, 40.0);
        let effects = ctl.on_sample(&sample_with_power(3.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        assert!(has_flagged(&effects, "limit_not_sticking", true));
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ryzenadj_calls(&runner).len(), 2, "initial set + reassert");

        // A compliant sample clears the flag (Flagged again, active=false).
        queue_cpu_reset(&runner, 40.0);
        let effects = ctl.on_sample(&sample_with_power(4.0, 19.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        assert!(has_flagged(&effects, "limit_not_sticking", false));
        assert_eq!(status_changes(&effects), 1);

        // Staying compliant is NOT a transition: no Flagged spam.
        queue_cpu_reset(&runner, 40.0);
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
            "fw-fan-quiet-controller-test-{}-flag-telemetry",
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
    fn calibration_telemetry_emits_blocked_samples_and_terminal_gate_snapshot() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        let dir = std::env::temp_dir().join(format!("calibration-gates-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let telemetry = Mutex::new(Some(Telemetry::open(&dir).unwrap()));
        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        ctl.on_command(Command::StartCalibration);
        for now in [0.0, 1.0, 2.0, 601.0, 602.0] {
            let mut sample = busy_at(now);
            sample.gpu_util_pct = 0.0;
            let effects = ctl.on_sample(&sample);
            apply_effects(&effects, &ctl, now, &ui_tx, &telemetry);
        }
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let sink = guard.as_mut().unwrap();
            sink.flush();
            sink.path().to_path_buf()
        };
        let records: Vec<serde_json::Value> = fs::read_to_string(path).unwrap().lines()
            .map(|line| serde_json::from_str::<serde_json::Value>(line).unwrap())
            .filter(|line| line["kind"] == "calibration").collect();
        assert_eq!(records.len(), 4, "one record per calibration sample, including timeout");
        let blocked = &records[2];
        assert_eq!(blocked["t_mono"], 2.0);
        assert_eq!(blocked["phase"], "settle");
        assert_eq!(blocked["context"]["gpu_cap_verified"], false);
        assert_eq!(blocked["gpu_util_pct"], 0.0);
        assert_eq!(blocked["gates"][0]["name"], "caps_held");
        assert_eq!(blocked["gates"][0]["satisfied"], false);
        assert_eq!(blocked["gates"][0]["duration_s"], 2.0);
        assert_eq!(blocked["windows"][0]["samples"], 0);
        assert!(blocked["windows"][0]["span"].is_null());
        assert_eq!(records[3]["phase"], "settle");
        assert_eq!(records[3]["next_phase"], "done");
        assert_eq!(records[3]["gates"][0]["satisfied"], false);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn production_apply_effects_emits_v3_fields_and_omits_retired_fields() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "fw-fan-quiet-controller-test-{}-v3-decision",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&busy_at(0.0));
        assert!(ctl.status.t_star_c.is_some(), "real Auto sample must publish T*");
        assert!(ctl.status.cpu.is_some() && ctl.status.gpu.is_some());
        let (ui_tx, _ui_rx) = crossbeam_channel::unbounded();
        let telemetry = Arc::new(Mutex::new(Some(Telemetry::open(&dir).unwrap())));

        apply_effects(&effects, &ctl, 0.0, &ui_tx, &telemetry);
        let path = {
            let mut guard = telemetry::lock(&telemetry);
            let t = guard.as_mut().unwrap();
            t.flush();
            t.path().to_path_buf()
        };
        let decision: serde_json::Value = fs::read_to_string(&path)
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .find(|line: &serde_json::Value| line["kind"] == "decision")
            .expect("production effect must emit one decision");
        for retired in ["budget_w", "freeze", "pi_target_w", "alloc_cpu_w", "alloc_gpu_w", "demand_cpu", "demand_gpu"] {
            assert!(decision.get(retired).is_none(), "retired {retired}: {decision}");
        }
        assert_eq!(decision["tstar_state"], "held");
        assert!(decision["t_star"].is_number(), "{decision}");
        assert!(decision["cpu_limit_w"].is_number(), "{decision}");
        assert!(decision["gpu_max_mhz"].is_number(), "{decision}");
        for device in ["cpu", "gpu"] {
            for field in ["group_c", "err_c", "thermal", "shadow", "cap", "selected", "hold", "gains_source"] {
                assert!(!decision[device][field].is_null(), "{device}.{field}: {decision}");
            }
        }
        assert!(decision["flags"].as_array().is_some_and(|flags| !flags.is_empty()), "{decision}");

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
        queue_cpu_reset(&runner, 40.0);
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
        queue_cpu_reset(&runner, 40.0);
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
            Arc::new(AtomicBool::new(false)),
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

    /// A step-test settle-phase sample. It carries no `fanctrl` view, so the
    /// controller's real `build_calib_context` computes `fanctrl_active:
    /// false` — the settle gate is therefore never met and any drive
    /// through the controller never settles, timing out at the cap. These
    /// tests exercise burner and actuator bookkeeping around that timeout.
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

    /// Enter the revision-4 shared settle phase and apply its held pair.
    fn drive_shared_settle_entry(ctl: &mut Controller<&FakeRunner>) {
        ctl.on_sample(&settle_sample());
        let calib = ctl.status().calib.as_ref().expect("calibrating");
        assert_eq!(calib.phase, "settle", "must start with shared settle: {calib:?}");
    }

    /// Drive `settle_sample()`s through the controller until calibration
    /// gives up (10-minute settle cap under the always-default `CalibContext`)
    /// and calibration finishes.
    fn drive_calibration_to_skip(ctl: &mut Controller<&FakeRunner>) {
        for second in 1..=600 {
            let mut sample = settle_sample();
            sample.t_mono = f64::from(second);
            ctl.on_sample(&sample);
            if ctl.status().calib.is_none() {
                return;
            }
        }
        panic!("calibration never concluded through the controller");
    }

    #[test]
    fn start_calibration_from_monitor_or_auto_but_not_manual() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);

        // From Manual mode: warned no-op, status untouched.
        ctl.on_command(Command::SetCpuW(20.0));
        let effects = ctl.on_command(Command::StartCalibration);
        assert!(effects.is_empty(), "got {effects:?}");
        assert_eq!(ctl.status().mode, Mode::Manual);
        assert!(ctl.status().calib.is_none());
        assert!(ctl.calib.is_none());

        // Back to Monitor: calibration starts directly in shared settle.
        ctl.on_command(Command::ReleaseAll);
        let effects = ctl.on_command(Command::StartCalibration);
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ctl.status().mode, Mode::Calibrating);
        let calib = ctl.status().calib.as_ref().expect("wizard progress set");
        assert_eq!(calib.phase, "settle");
        assert_eq!(calib.total, 3);
        assert_eq!(ctl.status().gpu_max_mhz, None);

        ctl.on_command(Command::AbortCalibration);
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_command(Command::StartCalibration);
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ctl.status().mode, Mode::Calibrating);
        assert!(ctl.auto.is_some(), "Auto loop state is suspended in place");
    }

    #[test]
    fn monitor_calibration_establishes_verified_floors_and_restores_that_pair_on_abort() {
        let runner = FakeRunner::new();
        let config = Config { cpu_floor_w: 8.0, ..Config::default() };
        let (mut ctl, gpu_calls) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            config,
        );
        ctl.on_command(Command::StartCalibration);

        let mut first = busy_at(0.0);
        first.gpu_sm_mhz = 1_000.0;
        ctl.on_sample(&first);
        assert_eq!(
            (ctl.status.cpu_limit_w, ctl.status.gpu_max_mhz),
            (Some(10.0), Some(1_000)),
            "Monitor has no applied pair, so calibration must establish configured floors through normal actuators"
        );

        let mut verified = busy_at(1.0);
        verified.gpu_sm_mhz = 1_000.0;
        ctl.on_sample(&verified);
        let context = ctl.build_calib_context(&verified);
        assert!(context.cpu_cap_verified);
        assert!(context.gpu_cap_verified);
        assert_eq!(context.cpu_cap_w, Some(10.0));
        assert_eq!(context.gpu_cap_mhz, Some(1_000));
        assert!(!context.use_current_caps, "Monitor origin selects configured floors");

        ctl.on_command(Command::AbortCalibration);
        assert_eq!(ctl.status.mode, Mode::Monitor);
        assert_eq!((ctl.status.cpu_limit_w, ctl.status.gpu_max_mhz), (Some(10.0), Some(1_000)));
        assert_eq!(
            gpu_calls.lock().unwrap().as_slice(),
            &[GpuCall::Set(1_000), GpuCall::Set(1_000)],
            "abort restores the established floor lock and does not release or leave a step active"
        );
        let cpu_writes = ryzenadj_calls(&runner);
        assert_eq!(cpu_writes, vec![expected_args(10_000), expected_args(10_000)]);
    }

    #[test]
    fn calibration_gpu_verification_pairs_observation_with_completed_command() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        let acquired = Instant::now();
        ctl.completion_clock = Box::new(move || acquired + Duration::from_secs(3));
        ctl.on_command(Command::StartCalibration);
        let first = Sample {
            acquired_at: Some(acquired),
            gpu_sm_mhz: 1_000.0,
            ..busy_at(0.0)
        };
        ctl.on_sample(&first);

        let early = Sample {
            acquired_at: Some(acquired + Duration::from_secs(1)),
            gpu_sm_mhz: 1_000.0,
            ..busy_at(1.0)
        };
        let early_context = ctl.build_calib_context(&early);
        assert!(!early_context.gpu_cap_verified, "a pre-completion observation cannot verify the lock");

        let after = Sample {
            acquired_at: Some(acquired + Duration::from_secs(4)),
            gpu_sm_mhz: 1_000.0,
            ..busy_at(4.0)
        };
        let after_context = ctl.build_calib_context(&after);
        assert!(after_context.gpu_cap_verified);
        assert_eq!(after_context.gpu_cap_completed_at_s, Some(3.0));
        ctl.on_command(Command::AbortCalibration);
    }

    #[test]
    fn calibration_context_exhaustively_uses_live_controller_and_sample_sources() {
        let runner = FakeRunner::new();
        let config = Config {
            cpu_floor_w: 16.0,
            gpu_floor_mhz: 1_100,
            cpu_max_w: 50.0,
            gpu_max_mhz: 2_900,
            cpu_hot_c: 91.0,
            gpu_hot_c: 89.0,
            ..Config::default()
        };
        let (mut ctl, _gpu) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            config,
        );
        ctl.on_command(Command::StartCalibration);
        let mut sample = busy_at(0.0);
        sample.gpu_sm_mhz = 1_100.0;
        let context = ctl.build_calib_context(&sample);
        let PerDeviceCalibContext {
            cpu_cap_w,
            cpu_cap_verified,
            cpu_cap_checked_at_s,
            cpu_cap_readback,
            cpu_cap_reset_reason,
            cpu_cap_completed_at_s,
            gpu_cap_mhz,
            gpu_cap_verified,
            gpu_cap_completed_at_s,
            use_current_caps,
            cpu_floor_w,
            gpu_floor_mhz,
            cpu_max_w,
            gpu_max_mhz,
            cpu_group_c,
            gpu_group_c,
            fanctrl_active,
            ec_mismatch,
            argmax_controllable,
            cpu_hot_c,
            gpu_hot_c,
            strategy,
            ma_interval,
        } = context;
        assert_eq!(cpu_cap_checked_at_s, None);
        assert_eq!(cpu_cap_readback, None);
        assert_eq!(cpu_cap_reset_reason, None);
        assert_eq!((cpu_cap_w, gpu_cap_mhz), (None, None));
        assert!(!cpu_cap_verified && !gpu_cap_verified);
        assert_eq!((cpu_cap_completed_at_s, gpu_cap_completed_at_s), (None, None));
        assert!(!use_current_caps);
        assert_eq!((cpu_floor_w, gpu_floor_mhz), (16.0, 1_100));
        assert_eq!((cpu_max_w, gpu_max_mhz), (50.0, 2_900));
        assert_eq!((cpu_group_c, gpu_group_c), (Some(75.0), Some(74.0)));
        assert!(fanctrl_active && argmax_controllable);
        assert!(!ec_mismatch);
        assert_eq!((cpu_hot_c, gpu_hot_c), (91.0, 89.0));
        assert_eq!(strategy.as_deref(), Some("quiet16"));
        assert_eq!(ma_interval, Some(60));
    }

    fn calibration_reconciliation_sample(
        t: f64,
        ec_max_c: f64,
        socket_max_c: f64,
        stale_by: Option<Duration>,
    ) -> Sample {
        let mut sample = curve_sample(t, socket_max_c, socket_max_c, TEMP_CURVE);
        sample.ec = Some(ec_reading_c(&[
            ("ambient_f75303@4d", 40.0),
            ("cpu@4c", ec_max_c),
        ]));
        sample.fanctrl.as_mut().expect("view").all_observed_at =
            Some(Instant::now() - stale_by.unwrap_or_default());
        sample
    }

    #[test]
    fn calibration_reconciliation_skips_fast_slopes_and_stale_views() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::StartCalibration);
        for (t, ec) in [(0.0, 70.0), (1.0, 75.0), (2.0, 80.0)] {
            let context = ctl.build_calib_context(&calibration_reconciliation_sample(
                t, ec, 60.0, None,
            ));
            assert!(!context.ec_mismatch, "fast local slope must skip scoring");
        }
        for t in 3..6 {
            let context = ctl.build_calib_context(&calibration_reconciliation_sample(
                f64::from(t), 80.0, 60.0, Some(Duration::from_secs(3)),
            ));
            assert!(!context.ec_mismatch, "stale view must skip scoring");
        }
    }

    #[test]
    fn calibration_reconciliation_latches_after_three_scored_mismatches() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::StartCalibration);
        for t in 0..3 {
            let context = ctl.build_calib_context(&calibration_reconciliation_sample(
                f64::from(t), 80.0, 60.0, None,
            ));
            assert_eq!(context.ec_mismatch, t == 2);
        }
    }

    #[test]
    fn calibration_reconciliation_three_matches_clear_and_reseed() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::StartCalibration);
        for t in 0..3 {
            ctl.build_calib_context(&calibration_reconciliation_sample(
                f64::from(t), 80.0, 60.0, None,
            ));
        }
        assert!(ctl.calib_replica.as_ref().is_some_and(EcReplica::ec_mismatch));
        let mut final_context = None;
        for t in 3..6 {
            final_context = Some(ctl.build_calib_context(&calibration_reconciliation_sample(
                f64::from(t), 60.0, 60.0, None,
            )));
        }
        assert!(!final_context.expect("context").ec_mismatch);
        let replica = ctl.calib_replica.as_ref().expect("calibration replica");
        assert_eq!(replica.reconciliation_ma(), Some(60.0));
        assert_eq!(replica.cpu_group_ma(), Some(60.0));
    }

    #[test]
    fn failed_calibration_cpu_write_cannot_establish_an_applied_pair() {
        use crate::actuators::cmd::test_support::output_with_code;

        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(1)));
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::StartCalibration);
        let mut sample = busy_at(0.0);
        sample.gpu_sm_mhz = 1_000.0;
        ctl.on_sample(&sample);
        assert_eq!(ctl.status.cpu_limit_w, None);
        assert!(!ctl.calib_cpu_verified);
        assert_eq!(ctl.calib_cpu_completed_at_s, None);
        assert_eq!(ctl.status.gpu_max_mhz, Some(1_000));
        assert_eq!(ctl.status.calib.as_ref().map(|progress| progress.phase.as_str()), Some("settle"));
        ctl.on_command(Command::AbortCalibration);
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
    fn calibration_starts_the_burner_on_entry_and_stops_it_on_skip() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-actuators");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        drive_shared_settle_entry(&mut ctl);
        // The burner starts unconditionally on calibration entry, before any
        // gate is ever checked (the ordering fact design §3.3 turns on).
        assert!(
            ctl.burner.is_some(),
            "burner must start as soon as calibration begins"
        );
        // Calibration also applies the held CPU/GPU pair before settle
        // detection begins, so a ryzenadj call has already happened.
        assert!(
            !ryzenadj_calls(&runner).is_empty(),
            "calibration entry must land the held CPU command"
        );
        assert_eq!(
            ctl.status().cpu_limit_w,
            Some(15.0),
            "held at the CPU floor"
        );

        drive_calibration_to_skip(&mut ctl);
        assert!(ctl.burner.is_none(), "burner must stop once the step skips");
        assert_eq!(ctl.status().mode, Mode::Monitor);
        // The skip path restores the CPU floor before calibration ends.
        assert_eq!(ctl.status().cpu_limit_w, Some(15.0));

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn shared_settle_does_not_emit_obsolete_lut_load_nags() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-nag");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        // GPU idle no longer drives an independent retired GPU calibration. Settle may
        // wait on its physical gates, but it must not emit obsolete load nags.
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
        for _ in 0..20 {
            let effects = ctl.on_sample(&idle);
            assert!(!effects.iter().any(|effect| matches!(effect,
                Effect::Noted { cause: "calib:needs_load" }
                | Effect::StatusChanged { cause: "calib:needs_load" })));
        }

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn auto_maintenance_releases_after_repeated_failed_repairs_and_stops_owning_cap() {
        use crate::actuators::cmd::test_support::{output_with_stdout, ryzenadj_info_table};
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("auto-maintenance-release");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::SetCpuW(15.0));
        ctl.on_command(Command::SetAuto(true));
        for attempt in 1..=3 {
            queue_cpu_reset(&runner, 40.0); // read before repair
            runner.push_result(Ok(output_with_code(0))); // repair write
            queue_cpu_reset(&runner, 40.0); // repair did not stick
            if attempt == 3 {
                runner.push_result(Ok(output_with_stdout(&ryzenadj_info_table(54.0, 53.0, 54.0))));
            }
            assert_eq!(ctl.reassert_actuators(&sample_at(f64::from(attempt * 10))), Some(false));
            assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        }
        assert!(ctl.status().cpu_limit_w.is_none());
        assert!(ctl.auto.as_ref().unwrap().cpu_verdict.released);
        let calls = runner.calls().len();
        assert_eq!(ctl.reassert_actuators(&sample_at(40.0)), None);
        assert_eq!(runner.calls().len(), calls, "maintenance must not reacquire a released cap");
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn calibration_refreshes_cpu_readback_and_preserves_a_reset_after_repair() {
        use crate::actuators::cmd::test_support::{output_with_stdout, ryzenadj_info_table};
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-cap-reset");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        ctl.apply_calib_effects(vec![RunnerEffect::SetCpuMaxWatts(15.0)], Some(&sample_at(0.0)));
        let before = runner.calls().len();
        ctl.build_calib_context(&sample_at(9.0));
        assert_eq!(runner.calls().len(), before);
        runner.push_result(Ok(output_with_stdout(&ryzenadj_info_table(40.0, 53.0, 40.0))));
        let ctx = ctl.build_calib_context(&sample_at(10.0));
        assert!(ctx.cpu_cap_reset_reason.as_ref().is_some_and(|s| s.contains("40")), "{ctx:?}");
        assert!(ctx.cpu_cap_verified, "repair verified");
        assert!(matches!(ctx.cpu_cap_readback, Some(WriteVerdict::Mismatch { read: 40.0, .. })));
        assert_eq!(ctx.cpu_cap_checked_at_s, Some(10.0));
        let next = ctl.build_calib_context(&sample_at(11.0));
        assert!(next.cpu_cap_reset_reason.is_none(), "disturbance is a one-tick event");
        ctl.on_command(Command::AbortCalibration);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn cap_maintenance_reads_before_writing_and_skips_a_matching_cpu_cap() {
        use crate::actuators::cmd::test_support::{output_with_stdout, ryzenadj_info_table};
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("cpu-maintenance");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::SetCpuW(15.0));
        let before = runner.calls().len();
        ctl.reassert_actuators(&sample_at(10.0));
        assert_eq!(&runner.calls()[before..], &[("ryzenadj".into(), vec!["--info".into()])]);
        runner.push_result(Ok(output_with_stdout(&ryzenadj_info_table(40.0, 53.0, 40.0))));
        let before = runner.calls().len();
        assert_eq!(ctl.reassert_actuators(&sample_at(20.0)), Some(true));
        let calls = runner.calls();
        assert_eq!(calls[before].1, vec!["--info"]);
        assert!(calls[before + 1].1.contains(&"--slow-limit=15000".into()));
        assert_eq!(calls[before + 2].1, vec!["--info"]);
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn calibration_outcome_reports_saved_partial_success_and_save_errors() {
        use crate::calib::step::CalibDevice;
        use crate::control::device_loop::Gains;
        for (both, fail_save) in [(false, false), (true, false), (true, true)] {
            let runner = FakeRunner::new();
            let (dir, path) = profile_fixture("calib-outcome-save");
            let mut ctl = controller(&runner, path);
            ctl.state_path = if fail_save { dir.clone() } else { dir.join("state.json") };
            ctl.on_command(Command::StartCalibration);
            let mut sample = curve_sample(60.0, 60.0, 60.0, &[(40.0, 20), (80.0, 40)]);
            let view = sample.fanctrl.as_mut().unwrap();
            view.strategy = "quiet16".into();
            view.ma_interval = 60;
            let key = "quiet16:60".to_string();
            let old = Gains { kc: 0.3, ti_s: 40.0 };
            ctl.persisted_cpu_gains.insert(key.clone(), old);
            let cpu = Gains { kc: 0.4, ti_s: 35.0 };
            let gpu = Gains { kc: 24.0, ti_s: 35.0 };
            let mut state = PersistedState::default();
            state.cpu_gains.insert(key.clone(), cpu);
            let mut effects = vec![RunnerEffect::FittedDevice { device: CalibDevice::Cpu, gains: cpu }];
            if both {
                state.gpu_gains.insert(key.clone(), gpu);
                effects.push(RunnerEffect::FittedDevice { device: CalibDevice::Gpu, gains: gpu });
            } else {
                effects.push(RunnerEffect::Noted("GPU fit rejected: fitted response 2.00C is below 3C".into()));
            }
            effects.extend([RunnerEffect::SaveState(Box::new(state)), RunnerEffect::Finished]);
            ctl.apply_calib_effects(effects, Some(&sample));
            let outcome = ctl.status().calib_outcome.as_ref().unwrap();
            assert_eq!(outcome.changes[0].before, old);
            assert_eq!(outcome.changes[0].after, cpu);
            assert!(outcome.applied);
            assert_eq!(outcome.saved, !fail_save);
            assert_eq!(outcome.title(), if fail_save { "error: gains not saved" } else if both { "success" } else { "partial success" });
            assert_eq!(ctl.persisted_cpu_gains[&key], cpu);
            if !both {
                assert!(outcome.details().contains("GPU: unchanged; Kc 22.2218"), "{}", outcome.details());
            }
            if fail_save {
                assert!(outcome.details().contains("NOT saved"));
                assert!(outcome.errors.iter().any(|error| error.contains("Could not save calibration")));
            } else {
                assert_eq!(PersistedState::load(&ctl.state_path).cpu_gains[&key], cpu);
            }
            let calls = runner.calls().len();
            ctl.on_command(Command::DismissCalibrationOutcome);
            assert!(ctl.status().calib_outcome.is_none());
            assert_eq!(runner.calls().len(), calls);
            fs::remove_dir_all(dir).unwrap();
        }
    }

    #[test]
    fn calibration_outcome_remains_visible_after_completion_and_abort() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-outcome");
        let mut ctl = controller(&runner, path);
        ctl.on_command(Command::StartCalibration);
        ctl.on_command(Command::AbortCalibration);
        assert!(format!("{:?}", ctl.status()).contains("calibration aborted"));
        ctl.on_sample(&Sample::default());
        assert!(format!("{:?}", ctl.status()).contains("calibration aborted"));
        ctl.on_command(Command::StartCalibration);
        assert!(!format!("{:?}", ctl.status()).contains("calibration aborted"));
        ctl.apply_calib_effects(vec![
            RunnerEffect::Noted("CPU fit rejected: fitted response 2.00C is below 3C".into()),
            RunnerEffect::Noted("GPU fit rejected: gain exceeds 4x default".into()),
            RunnerEffect::Finished,
        ], None);
        let status = format!("{:?}", ctl.status());
        assert!(status.contains("fitted response 2.00C"), "{status}");
        assert!(status.contains("gain exceeds 4x default"), "{status}");
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn abort_calibration_releases_and_returns_to_monitor() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("calib-abort");
        let mut ctl = controller(&runner, path.clone());
        ctl.on_command(Command::StartCalibration);
        drive_shared_settle_entry(&mut ctl);
        assert!(ctl.burner.is_some(), "burner running during the step test");
        for _ in 0..10 {
            ctl.on_sample(&settle_sample());
        }

        let effects = ctl.on_command(Command::AbortCalibration);
        assert_eq!(status_changes(&effects), 1);
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ctl.status().calib.is_none());
        assert_eq!(ctl.status().cpu_limit_w, Some(Config::default().cpu_floor_w));
        assert!(ctl.calib.is_none());
        assert!(ctl.burner.is_none(), "abort must stop the burner");
        // Release toggled the profile back; smu stays untouched (session on).
        assert_eq!(fs::read_to_string(&path).unwrap().trim(), "balanced");
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
        drive_shared_settle_entry(&mut ctl);
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
    fn calibration_skip_stamps_state_without_retired_fields() {
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

        ctl.on_command(Command::StartCalibration);
        drive_shared_settle_entry(&mut ctl);
        drive_calibration_to_skip(&mut ctl);

        // Finished: back to Monitor, wizard gone, burner stopped. The CPU
        // limit itself is left pinned at the floor because the per-device
        // runner restores the held pair rather than releasing to stock.
        assert_eq!(ctl.status().mode, Mode::Monitor);
        assert!(ctl.status().calib.is_none());
        assert!(ctl.burner.is_none());
        assert_eq!(
            ctl.status().cpu_limit_w,
            Some(Config::default().cpu_floor_w)
        );

        // The state file records the completion stamp without reviving the
        // superseded retired scalar gains. `settle_sample()` carries no `fanctrl`
        // view, so `fanctrl_active` is always false and the step test's
        // settle gate never clears — it skips and keeps defaults.
        let saved = PersistedState::load(&state_path);
        let wire = serde_json::to_value(&saved).unwrap();
        assert!(wire.get("lut").is_none() && wire.get("loop_gains").is_none());
        saved
            .calibrated_at
            .expect("calibrated_at set")
            .parse::<u64>()
            .expect("unix seconds");

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn calibration_save_preserves_the_duty_rpm_table_and_warm_start_it_does_not_own() {
        // Regression: `RunnerEffect::SaveState`'s `PersistedState` carries
        // only what the step-test runner itself owns (`calibrated_at` and
        // the two gain maps) — its `duty_rpm_table`/`warm_start` are bare
        // `..PersistedState::default()` filler, NOT the controller's actual
        // table/warm-start map. Saving that struct directly to disk would
        // silently wipe both on every calibration. The controller must fold
        // in only the fields the runner owns, then persist the FULL state
        // through `save_persisted_state`.
        let runner = FakeRunner::new();
        let (dir, profile) = profile_fixture("calib-preserves-table");
        let state_path = dir.join("state.json");
        let guard = RestoreGuard::new(
            &runner,
            Some(cpu_actuator(&runner, profile)),
            None,
            Some(SmuModule::assume_unloaded()),
        );
        let mut pre_refined_table = DutyRpmTable::default();
        pre_refined_table.refine(30, 2600.0); // a real, observable change from the seed
        let mut pre_warm_start = BTreeMap::new();
        pre_warm_start.insert(
            "quiet16:36:batt".to_string(),
            WarmStartEntry {
                cpu_cap_w: 77.0,
                gpu_lock_mhz: 1800,
            },
        );
        let persisted = PersistedState {
            duty_rpm_table: pre_refined_table.clone(),
            warm_start: pre_warm_start.clone(),
            ..PersistedState::default()
        };
        let mut ctl = Controller::new(
            guard,
            persisted,
            state_path.clone(),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );

        ctl.on_command(Command::StartCalibration);
        drive_shared_settle_entry(&mut ctl);
        drive_calibration_to_skip(&mut ctl);
        assert_eq!(ctl.status().mode, Mode::Monitor, "calibration finished");

        let saved = PersistedState::load(&state_path);
        assert_eq!(
            saved.duty_rpm_table, pre_refined_table,
            "the pre-existing refined table must survive a calibration save unchanged"
        );
        assert_eq!(
            saved.warm_start, pre_warm_start,
            "the pre-existing warm-start map must survive a calibration save unchanged"
        );
        // The in-memory controller state agrees too (not just the file).
        assert_eq!(ctl.duty_rpm_table, pre_refined_table);
        assert_eq!(ctl.persisted_warm_start, pre_warm_start);

        fs::remove_dir_all(&dir).unwrap();
    }

    // --- Task 25: auto mode ---

    use crate::actuators::gpu::test_support::{FakeGpu, GpuCall};
    use crate::fanctrl::client::{FanctrlView, Freshness};
    use crate::sensors::ec::EcReading;
    use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};

    fn calibrated() -> PersistedState {
        PersistedState::default()
    }

    /// Controller with a CPU actuator and FakeGpu; also returns the GPU
    /// call-log handle. Mode starts at Monitor — the
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

    /// A complete v4 Auto sample: fresh curve/replica data plus both device
    /// groups and direct draw feedback.  Lifecycle tests use this rather
    /// than relying on the removed retired controller's fan-only fallback.
    fn busy_at(t: f64) -> Sample {
        let mut sample = curve_sample(t, 75.0, 74.0, TEMP_CURVE);
        sample.cpu_pkg_w = 25.0;
        sample.gpu_w_valid = true;
        sample.gpu_w = 60.0;
        sample.gpu_mhz_valid = true;
        sample.gpu_sm_mhz = 1_800.0;
        sample.gpu_util_pct = 95.0;
        sample.ec = Some(ec_reading_c(&[
            ("ambient_f75303@4d", 40.0),
            ("cpu@4c", 75.0),
            ("gpu_vr_f75303@4d", 74.0),
        ]));
        sample
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


    #[test]
    fn uncalibrated_auto_entry_uses_direct_per_device_loops() {
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner); // uncalibrated

        let effects = ctl.on_command(Command::SetAuto(true));
        assert_eq!(
            ctl.status().mode,
            Mode::Auto,
            "direct watt/MHz loops need no calibration gate"
        );
        assert!(ctl.status().flags.contains(&StatusFlag::NotCalibrated));
        assert_eq!(status_changes(&effects), 1);
    }

    #[test]
    fn auto_entry_publishes_safe_device_floors() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        assert_eq!(ctl.status().mode, Mode::Auto);

        let _ = ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.status().tstar_state, Some(crate::types::TelemetryTStarState::Held));
        assert!(
            ctl.status()
                .cpu
                .as_ref()
                .is_some_and(|cpu| cpu.cap >= ctl.status().cpu_floor_w)
        );
        assert!(
            ctl.status()
                .gpu
                .as_ref()
                .is_some_and(|gpu| gpu.cap >= f64::from(ctl.status().gpu_floor_mhz))
        );
    }

    #[test]
    fn auto_sample_publishes_independent_device_decisions() {
        // The first sample publishes both independent device decisions.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let effects = ctl.on_sample(&busy_at(0.0));
        assert!(
            effects
                .iter()
                .any(|effect| matches!(effect, Effect::CpuSet(_)))
        );
        assert!(
            effects
                .iter()
                .any(|effect| matches!(effect, Effect::GpuSet(_)))
        );
        let status = ctl.status();
        assert!(status.cpu.is_some() && status.gpu.is_some());
        assert_eq!(
            status.tstar_state,
            Some(crate::types::TelemetryTStarState::Held)
        );
    }

    #[test]
    fn missing_active_keyed_gains_use_defaults() {
        // V4 resolves each device's gains from the first fresh view.
        let runner = FakeRunner::new();
        let config = Config {
            cpu_gains: Some(crate::control::device_loop::Gains {
                kc: 0.22,
                ti_s: 35.0,
            }),
            gpu_gains: Some(crate::control::device_loop::Gains {
                kc: 2.1,
                ti_s: 35.0,
            }),
            ..Config::default()
        };
        let (mut ctl, _gpu) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            config,
        );
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(
            ctl.status().cpu.as_ref().expect("CPU").gains_source,
            crate::types::GainsSource::Config
        );
        assert_eq!(
            ctl.status().gpu.as_ref().expect("GPU").gains_source,
            crate::types::GainsSource::Config
        );
    }
    #[test]
    fn live_floor_change_bounds_the_next_device_decision() {
        // Runtime floor changes apply on the next device-loop decision.
        // Raising the CPU floor mid-session must bound the next CPU cap.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        ctl.on_sample(&busy_at(1.0));
        let gpu_floor_mhz = ctl.status().gpu_floor_mhz;
        ctl.on_command(Command::SetFloors {
            cpu_w: 40.0,
            gpu_mhz: gpu_floor_mhz,
        });
        ctl.on_sample(&busy_at(5.0));
        let cpu = ctl.status().cpu.as_ref().expect("CPU decision");
        assert!(
            cpu.cap >= 40.0,
            "CPU cap must respect the raised independent floor: {cpu:?}"
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
            // Keep a valid fan reading so the test isolates the watchdog's
            // three-strike behavior from the source's Released path.
            fan_valid: true,
            fan1_rpm: 2500.0,
            fan2_rpm: 2400.0,
            ..busy_at(t)
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
        let first = ctl.on_sample(&overheat_at(1.0));
        let second = ctl.on_sample(&overheat_at(2.0));
        assert!(!first.contains(&Effect::Released) && !second.contains(&Effect::Released));
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
        let (mut ctl, gpu_calls) = auto_controller(&runner, path.clone(), Config::default());
        ctl.on_command(Command::StartCalibration);
        drive_shared_settle_entry(&mut ctl);
        for _ in 0..10 {
            ctl.on_sample(&settle_sample()); // burner active mid-settle
        }
        assert!(ctl.burner.is_some(), "premise: burner running mid-settle");

        let first = ctl.on_sample(&overheat_at(10.0));
        assert_eq!(ctl.status.mode, Mode::Calibrating, "first hot sample: {first:?}");
        let second = ctl.on_sample(&overheat_at(11.0));
        assert_eq!(ctl.status.mode, Mode::Calibrating, "second hot sample: {second:?}");
        let effects = ctl.on_sample(&overheat_at(12.0));
        let causes: Vec<_> = effects.iter().filter_map(|effect| match effect {
            Effect::StatusChanged { cause } | Effect::Noted { cause } => Some(*cause),
            _ => None,
        }).collect();
        assert_eq!(
            causes.first().copied(),
            Some("calib:skipped"),
            "the runner's restore-pair + Noted terminal outcome must precede watchdog release: {effects:?}"
        );
        assert!(effects.contains(&Effect::Released), "got {effects:?}");
        assert!(has_status_change_cause(
            &effects,
            "watchdog:thermal_emergency"
        ));
        let cpu_writes = ryzenadj_calls(&runner);
        assert!(
            cpu_writes.iter().all(|args| args == &expected_args(15_000)),
            "thermal terminal must only establish/restore the clamped CPU floor, never issue a step: {cpu_writes:?}"
        );
        assert_eq!(
            gpu_calls.lock().unwrap().as_slice(),
            &[GpuCall::Set(1_000), GpuCall::Set(1_000), GpuCall::Release],
            "held GPU floor restore must precede the watchdog's final stock release"
        );
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
    fn auto_calibration_freezes_device_loops_and_tstar_then_reenters_without_a_cap_step() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(40.0));
        ctl.on_command(Command::SetGpuMaxClock(1_800));
        ctl.on_command(Command::SetAuto(true));
        for t in 0..8 {
            ctl.on_sample(&curve_sample(f64::from(t), 75.0, 74.0, TEMP_CURVE));
        }
        let held_pair = (ctl.status.cpu_limit_w, ctl.status.gpu_max_mhz);
        {
            let auto = ctl.auto.as_mut().expect("Auto state");
            auto.replica.reset(Some(90.0), Some(89.0), Some(88.0));
            auto.cpu_draw_window = [99.0; 5].into();
            auto.fan_window = [9_999.0; 5].into();
            auto.ec_slope_window = [90.0; 5].into();
            auto.steady_window = [9_999.0; 40].into();
            auto.steady_key = Some("stale".into());
            auto.refinement_window = [9_999.0; 40].into();
            auto.refinement_key = Some("stale".into());
        }
        let before = {
            let auto = ctl.auto.as_ref().expect("Auto state");
            (
                auto.cpu_loop.thermal(),
                auto.cpu_loop.requested(),
                auto.gpu_loop.thermal(),
                auto.gpu_loop.requested(),
                auto.tstar.state(),
                ctl.status.t_star_c,
            )
        };

        ctl.on_command(Command::StartCalibration);
        assert_eq!(ctl.status.mode, Mode::Calibrating);
        ctl.on_sample(&curve_sample(8.0, 75.0, 74.0, TEMP_CURVE));
        let during = {
            let auto = ctl.auto.as_ref().expect("suspended Auto state");
            (
                auto.cpu_loop.thermal(),
                auto.cpu_loop.requested(),
                auto.gpu_loop.thermal(),
                auto.gpu_loop.requested(),
                auto.tstar.state(),
                ctl.status.t_star_c,
            )
        };
        assert_eq!(during, before, "calibration must not tick either loop or TStar");

        ctl.on_command(Command::AbortCalibration);
        assert_eq!(ctl.status.mode, Mode::Auto);
        assert_eq!((ctl.status.cpu_limit_w, ctl.status.gpu_max_mhz), held_pair);
        let cpu_writes = ryzenadj_calls(&runner).len();
        let gpu_writes = gpu_sets(&_gpu_calls).len();
        let effects = ctl.on_sample(&busy_at(9.0));
        assert!(!effects.iter().any(|effect| matches!(effect, Effect::CpuSet(_) | Effect::GpuSet(_))), "first re-entry sample must hold the restored pair: {effects:?}");
        assert_eq!(ryzenadj_calls(&runner).len(), cpu_writes, "held reseed must not write CPU");
        assert_eq!(gpu_sets(&_gpu_calls).len(), gpu_writes, "held reseed must not write GPU");
        assert_eq!((ctl.status.cpu_limit_w, ctl.status.gpu_max_mhz), held_pair);
        {
            let auto = ctl.auto.as_mut().expect("resumed Auto state");
            assert_eq!(auto.replica.cpu_group_ma(), Some(75.0));
            assert_eq!(auto.replica.gpu_group_ma(), Some(74.0));
            assert_eq!(auto.replica.reconciliation_ma(), Some(74.0));
            assert_eq!(auto.cpu_draw_window.iter().copied().collect::<Vec<_>>(), vec![25.0]);
            assert_eq!(auto.fan_window.iter().copied().collect::<Vec<_>>(), vec![3_000.0]);
            assert_eq!(auto.ec_slope_window.iter().copied().collect::<Vec<_>>(), vec![75.0]);
            assert!(auto.steady_window.is_empty());
            assert_eq!(auto.steady_key, None);
            assert!(auto.refinement_window.is_empty());
            assert_eq!(auto.refinement_key, None);
            assert!(auto.view_initialised);
        }

        // A runner-driven terminal skip resumes the same suspended Auto
        // state and gets the same one-sample no-write re-entry contract.
        ctl.on_command(Command::StartCalibration);
        ctl.on_sample(&curve_sample(10.0, 75.0, 74.0, TEMP_CURVE));
        let mut changed = curve_sample(11.0, 75.0, 74.0, TEMP_CURVE);
        changed.fanctrl.as_mut().expect("view").strategy = "performance".into();
        let terminal = ctl.on_sample(&changed);
        assert!(has_status_change_cause(&terminal, "calib:skipped"), "{terminal:?}");
        assert_eq!(ctl.status.mode, Mode::Auto);
        assert_eq!((ctl.status.cpu_limit_w, ctl.status.gpu_max_mhz), held_pair);
        let effects = ctl.on_sample(&curve_sample(12.0, 75.0, 74.0, TEMP_CURVE));
        assert!(!effects.iter().any(|effect| matches!(effect, Effect::CpuSet(_) | Effect::GpuSet(_))), "first re-entry sample after runner terminal must hold: {effects:?}");
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

        // GPU at its trip threshold (91 °C), CPU valid and cool: the OR trips.
        let gpu_hot_at = |t: f64| Sample {
            t_mono: t,
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            gpu_temp_c: crate::control::watchdog::GPU_TRIP_C,
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
        assert!(
            !ctl.auto
                .as_ref()
                .expect("auto state")
                .gpu_commands
                .is_empty(),
            "premise: the first successful GPU command is paired to later samples"
        );

        let effects = ctl.on_sample(&Sample {
            resumed: true,
            ..busy_at(1.0)
        });
        assert_eq!(*resumed_count.lock().unwrap(), 1);
        assert!(has_reassert(&effects, "resume"), "got {effects:?}");
        assert_eq!(ctl.status().mode, Mode::Auto, "auto survives the resume");
        let auto = ctl.auto.as_ref().expect("auto state");
        assert_eq!(
            auto.gpu_commands.len(),
            2,
            "pre-suspend pairing is gone; both completed resume reasserts remain"
        );
        assert_eq!(auto.gpu_command_generation, 2);
    }

    #[test]
    fn resume_without_gpu_actuator_still_reasserts() {
        // No GPU this run: the resume path must not assume the hook exists.
        let runner = FakeRunner::new();
        let mut ctl = controller_no_profile(&runner);
        ctl.on_command(Command::SetCpuW(20.0));
        queue_cpu_reset(&runner, 40.0);
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
        queue_cpu_reset(&runner, 40.0);
        let effects = ctl.on_sample(&sample_with_power(102.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));

        // A compliant sample clears the flag and the streak.
        ctl.on_sample(&sample_with_power(103.0, 19.0));
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));

        // Past the 60 s window (t >= 160): back to the normal 3-sample rule.
        ctl.on_sample(&sample_with_power(161.0, 26.0));
        queue_cpu_reset(&runner, 40.0);
        let effects = ctl.on_sample(&sample_with_power(162.0, 26.0));
        assert!(
            !has_reassert(&effects, "stickiness"),
            "two violations after the strict window must not fire, got {effects:?}"
        );
        assert!(!ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
        queue_cpu_reset(&runner, 40.0);
        let effects = ctl.on_sample(&sample_with_power(163.0, 26.0));
        assert!(has_reassert(&effects, "stickiness"), "got {effects:?}");
        assert!(ctl.status().flags.contains(&StatusFlag::LimitNotSticking));
    }

    #[test]
    fn set_floors_sanitizes_echoes_and_persists_on_change_only() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "fw-fan-quiet-controller-test-{}-floors-persist",
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

    /// Calibration owns its actuator pair, so interactive floor changes are
    /// rejected until calibration ends.
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

    // Configuration and command-gating regressions.

    #[test]
    fn out_of_range_config_floors_never_panic_auto() {
        // Controller::new sanitizes invalid device ranges before Auto entry.
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
    fn manual_commands_are_rejected_while_calibration_suspends_auto() {
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
        assert_eq!(status_changes(&effects), 1, "got {effects:?}");
        assert_eq!(ctl.status().mode, Mode::Calibrating);
        assert!(ctl.status().calib.is_some());
        assert!(ctl.auto.is_some(), "Auto state remains suspended in place");
        assert_eq!(ryzenadj_calls(&runner).len(), cpu_calls_before);
        assert_eq!(gpu_sets(&gpu_calls).len(), gpu_calls_before);

        ctl.on_command(Command::AbortCalibration);
        // SetFanTarget stays allowed after Auto resumes: it retargets the fan target live.
        ctl.on_command(Command::SetFanTarget(2500.0));
        assert_eq!(ctl.status().fan_target_rpm, 2500.0);
        assert_eq!(ctl.status().mode, Mode::Auto);
    }

    #[test]
    fn fan_invalid_with_no_fanctrl_view_releases_to_stock() {
        // A lost fan sensor and no fanctrl view is design §2.5's `Released` row
        // ("nothing to close a loop on") — no controller "freeze at the
        // floor" fallback survives; caps release to stock immediately and
        // stay there (no CPU or GPU command) until something
        // usable comes back.
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        let invalid_fan_at = |t: f64| Sample {
            fan_valid: false,
            ..busy_at(t)
        };
        let effects = ctl.on_sample(&invalid_fan_at(0.0));
        assert_eq!(ctl.status().tstar_state, Some(crate::types::TelemetryTStarState::Released));
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
            "GPU device loop must not command anything while Released"
        );
    }

    #[test]
    fn set_fan_target_persists_config_on_change_only() {
        let runner = FakeRunner::new();
        let dir = std::env::temp_dir().join(format!(
            "fw-fan-quiet-controller-test-{}-fan-persist",
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
    // Controller loop integration: device loops, EC replica, guards, and the
    // shared actuator-verdict rule.
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
    // the Curve scenarios. ----

    static EC_FIXTURE_COUNTER_C: AtomicU64 = AtomicU64::new(0);

    fn ec_reading_c(sensors: &[(&str, f64)]) -> EcReading {
        let n = EC_FIXTURE_COUNTER_C.fetch_add(1, AtomicOrdering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "fw-fan-quiet-controller-test-{}-{n}",
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

    /// quiet16-shaped curve: duty 31 at
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

    /// One Curve-eligible sample: a fresh, active fanctrl view (`temp`/
    /// `ma_temp`), an EC reading whose `cpu@4c` argmax matches `temp`
    /// exactly (a reconciliation match, not a mismatch) with ambient well
    /// below the feasibility margin, and a valid fan reading (so a
    /// core-condition failure never falls all the way to `Released`).
    fn curve_sample(t: f64, temp: f64, ma_temp: f64, curve: &[(f64, u8)]) -> Sample {
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
            // A healthy machine reports a valid, cool Tctl. Required now
            // that Curve entry takes 15 samples (§2.5's 15 s at the 1 Hz
            // sample cadence, roast-pr-1 finding 6): without it the
            // sensor-lost watchdog trips mid-climb and releases everything
            // before the loop can ever engage.
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            ..Sample::default()
        }
    }

    fn drive_curve(ctl: &mut Controller<&FakeRunner>, t0: f64, n: u32) -> Vec<Effect> {
        let mut last = Vec::new();
        for i in 0..n {
            last = ctl.on_sample(&curve_sample(t0 + f64::from(i), 75.0, 74.0, TEMP_CURVE));
        }
        last
    }

    /// roast PR-2 finding 6: `SAMPLE_PERIOD_S` is the sampler's real cadence,
    /// by construction rather than by coincidence. The spec timers §2.5's
    /// 15 s and §2.7's 60 s are derived from it via `ticks_for`, so a second,
    /// unlinked copy of the cadence would silently rescale both the day the
    /// sampler's tick changes — with the whole suite still green. This test
    /// cannot fail while the derivation stands; it fails to compile (and the
    /// timers stay right) if someone restates the constant instead.
    #[test]
    fn v4_warm_pair_seeds_the_cpu_loop_above_its_entry_floor() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        let duty = DutyRpmTable::default().duty_for_rpm(ctl.status().fan_target_rpm);
        ctl.persisted_warm_start.insert(
            warm_start_key("quiet16", duty, false),
            WarmStartEntry {
                cpu_cap_w: 40.0,
                gpu_lock_mhz: 2_400,
            },
        );
        ctl.on_command(Command::SetAuto(true));
        for t in 0..5 { ctl.on_sample(&busy_at(f64::from(t))); }

        let auto = ctl.auto.as_ref().expect("Auto state");
        assert!(auto.cpu_entry_seeded && auto.cpu_shadow_entry_seeded);
        assert_eq!(auto.cpu_loop.thermal(), 40.0);
        assert!(
            auto.cpu_loop.thermal() >= ctl.config.cpu_floor_w,
            "the paired seed must still respect the live entry floor"
        );
    }

    #[test]
    fn per_device_calibration_writes_and_restores_both_caps_through_normal_paths() {
        let runner = FakeRunner::new();
        let (mut ctl, gpu_calls) = auto_controller_no_profile(&runner);
        ctl.apply_calib_effects(
            vec![
                RunnerEffect::SetCpuMaxWatts(23.0),
                RunnerEffect::SetGpuMaxClock(1_500),
            ],
            None,
        );
        assert_eq!(ctl.status().cpu_limit_w, Some(23.0));
        assert_eq!(ctl.status().gpu_max_mhz, Some(1_500));
        assert!(gpu_sets(&gpu_calls).contains(&1_500));

        ctl.apply_calib_effects(
            vec![
                RunnerEffect::SetCpuMaxWatts(ctl.config.cpu_floor_w),
                RunnerEffect::SetGpuMaxClock(ctl.config.gpu_floor_mhz),
            ],
            None,
        );
        assert_eq!(ctl.status().cpu_limit_w, Some(ctl.config.cpu_floor_w));
        assert_eq!(ctl.status().gpu_max_mhz, Some(ctl.config.gpu_floor_mhz));
    }

    #[test]
    fn rejected_curve_raises_curve_invalid_and_falls_to_held() {
        // A descending duty segment: Curve::from_points rejects it (only
        // duty must be non-decreasing in file order — see `CurveError`).
        let bad_curve: &[(f64, u8)] = &[(50.0, 40), (60.0, 30)];
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&curve_sample(0.0, 75.0, 74.0, bad_curve));
        assert!(ctl.status().flags.contains(&StatusFlag::CurveInvalid));
        assert_eq!(ctl.status().tstar_state, Some(crate::types::TelemetryTStarState::Held));
        assert!(
            ctl.status().cpu.is_some(),
            "Held fallback keeps the CPU loop observable"
        );
    }

    #[test]
    fn leaving_auto_clears_a_stuck_guard_flag() {
        // A guard flag must not survive `ReleaseAll` — nothing outside Auto touches these
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
        assert!(ctl.status().tstar_state.is_none());
    }

    #[test]
    fn nvme_hot_tick_raises_the_flag_and_leaves_device_caps_unchanged() {
        // Two otherwise-identical Held sessions, one with an NVMe-hot
        // reading on every tick, one without: both device-cap trajectories must
        // be bit-for-bit identical (NVMe is reporting-only, §2.8) while
        // the hot session alone raises the flag.
        let cold_runner = FakeRunner::new();
        let (mut cold, _g1) = auto_controller_no_profile(&cold_runner);
        cold.on_command(Command::SetAuto(true));
        let hot_runner = FakeRunner::new();
        let (mut hot, _g2) = auto_controller_no_profile(&hot_runner);
        hot.on_command(Command::SetAuto(true));

        let mut cold_caps = (0.0, 0.0);
        let mut hot_caps = (0.0, 0.0);
        for i in 0..3 {
            let t = f64::from(i) * 5.0;
            let ce = cold.on_sample(&busy_at(t));
            let he = hot.on_sample(&Sample {
                nvme_temp_c: Some(85.0),
                ..busy_at(t)
            });
            assert!(!ce.is_empty() || cold.status.cpu.is_some());
            assert!(!he.is_empty() || hot.status.cpu.is_some());
            cold_caps = (cold.status.cpu.as_ref().unwrap().cap, cold.status.gpu.as_ref().unwrap().cap);
            hot_caps = (hot.status.cpu.as_ref().unwrap().cap, hot.status.gpu.as_ref().unwrap().cap);
        }
        assert!(!cold.status().flags.contains(&StatusFlag::NvmeHot));
        assert!(hot.status().flags.contains(&StatusFlag::NvmeHot));
        assert_eq!(
            cold_caps, hot_caps,
            "NVMe HOT must not move either device cap (reporting-only, §2.8)"
        );
        assert_eq!(cold.status().tstar_state, hot.status().tstar_state);
    }

    #[test]
    fn reconciliation_is_scored_on_the_1hz_sample_carrying_the_view_not_the_5s_tick() {
        // A persistent EC/view disagreement (argmax steady at 90, never
        // ramping — so the slope-based skip guard never suppresses
        // scoring) latches EC MISMATCH on its 3rd consecutive scored view,
        // at t=2 — one tick before the next 5 s controller boundary (t=5).
        // If reconciliation were only scored on the 5 s tick, none of
        // t=0,1,2 would ever be scored at all inside this window.
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));

        for t in [0.0, 1.0, 2.0] {
            let mut s = curve_sample(t, 75.0, 74.0, TEMP_CURVE);
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
    fn auto_entry_without_a_warm_start_seeds_from_the_measured_draw_not_the_floors() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        // The CPU shadow remains thermal-only until five valid package-power
        // observations have arrived; a one-off sample must not invent draw.
        ctl.on_sample(&Sample {
            cpu_pkg_w: 14.0,
            gpu_w: 99.0,
            ..busy_at(0.0)
        });
        let first = ctl.status().cpu.as_ref().expect("CPU decision");
        assert_eq!(first.shadow, first.thermal);
        for t in 1..5 {
            ctl.on_sample(&Sample {
                t_mono: f64::from(t),
                cpu_pkg_w: 14.0,
                gpu_w: 99.0,
                ..busy_at(f64::from(t))
            });
        }
        let seeded = ctl
            .status()
            .cpu
            .as_ref()
            .expect("CPU decision after five samples");
        assert_eq!(seeded.shadow, 14.0 + ctl.config.shadow_headroom_cpu_w);
    }

    // ---- fw-fanctrl-loop-438: steady-window / warm-start / calibration hooks ----

    /// An Auto-eligible Held sample carrying a fanctrl view (so
    /// `self.status.strategy` is known — the warm-start key needs it) but
    /// no EC reading (`ec: None`), which keeps the source in Held regardless
    /// of the view's own content.
    /// The tests below only care about the Held error (`rpm_for_duty
    /// (target_duty) - rpm_smoothed`) and the warm-start key, not Curve/
    /// reconciliation.
    fn rpm_view_sample(t: f64, fan_rpm: f64, strategy: &str, speed_pct: u8, on_ac: bool) -> Sample {
        let view = FanctrlView {
            strategy: strategy.to_string(),
            active: true,
            speed_pct,
            temperature: 60.0,
            ma_temperature: 60.0,
            ma_interval: 60,
            curve: TEMP_CURVE.to_vec(),
            observed_at: Instant::now(),
            all_observed_at: Some(Instant::now()),
        };
        Sample {
            t_mono: t,
            fan_valid: true,
            fan1_rpm: fan_rpm,
            fan2_rpm: fan_rpm,
            on_ac,
            fanctrl: Some(view),
            fanctrl_freshness: Freshness::Fresh,
            // A healthy machine reports a valid, cool Tctl: without this a
            // long run (this file's steady-window tests drive 40+ samples)
            // trips the sensor-lost watchdog and releases everything.
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            ..busy_at(t)
        }
    }

    fn keyed_auto_sample(t: f64, strategy: &str, duty: u8, on_ac: bool) -> Sample {
        let mut sample = busy_at(t);
        let target_rpm = DutyRpmTable::default().rpm_for_duty(duty);
        sample.fan1_rpm = target_rpm;
        sample.fan2_rpm = target_rpm;
        sample.on_ac = on_ac;
        let view = sample.fanctrl.as_mut().expect("busy sample has fanctrl view");
        view.strategy = strategy.to_string();
        view.speed_pct = duty;
        sample
    }

    fn assert_rekey_preserves_live_candidates(
        changed_strategy: &str,
        changed_duty: u8,
        changed_on_ac: bool,
    ) -> Controller<&'static FakeRunner> {
        let changed_runner = Box::leak(Box::new(FakeRunner::new()));
        let reference_runner = Box::leak(Box::new(FakeRunner::new()));
        let (mut changed, _changed_gpu) = auto_controller_no_profile(changed_runner);
        let (mut reference, _reference_gpu) = auto_controller_no_profile(reference_runner);
        let hostile_key = warm_start_key(changed_strategy, changed_duty, changed_on_ac);
        for ctl in [&mut changed, &mut reference] {
            ctl.persisted_warm_start.insert(
                hostile_key.clone(),
                WarmStartEntry {
                    cpu_cap_w: ctl.config.cpu_floor_w + 1.0,
                    gpu_lock_mhz: ctl.config.gpu_floor_mhz + 1,
                },
            );
            ctl.on_command(Command::SetAuto(true));
            ctl.on_sample(&keyed_auto_sample(0.0, "quiet16", 36, false));
            let auto = ctl.auto.as_mut().expect("Auto state");
            auto.steady_key = Some(warm_start_key("quiet16", 36, false));
            auto.steady_window = [3_030.0; 12].into();
        }

        if changed_duty != 36 {
            changed.on_command(Command::SetFanTarget(
                changed.duty_rpm_table.rpm_for_duty(changed_duty),
            ));
            reference.on_command(Command::SetFanTarget(
                reference.duty_rpm_table.rpm_for_duty(changed_duty),
            ));
        }
        changed.on_sample(&keyed_auto_sample(1.0, changed_strategy, changed_duty, changed_on_ac));
        reference.on_sample(&keyed_auto_sample(1.0, "quiet16", changed_duty, false));

        let changed_auto = changed.auto.as_ref().expect("Auto state");
        let reference_auto = reference.auto.as_ref().expect("Auto state");
        assert_eq!(changed_auto.cpu_loop.thermal(), reference_auto.cpu_loop.thermal());
        assert_eq!(changed_auto.gpu_loop.thermal(), reference_auto.gpu_loop.thermal());
        assert_ne!(changed_auto.cpu_loop.thermal(), changed.config.cpu_floor_w + 1.0);
        assert_ne!(changed_auto.gpu_loop.thermal(), f64::from(changed.config.gpu_floor_mhz + 1));
        assert_eq!(changed_auto.steady_key.as_deref(), Some(hostile_key.as_str()));
        assert!(changed_auto.steady_window.is_empty(), "a key change restarts the window");

        let cpu_cap = changed.status.cpu_limit_w.expect("CPU cap");
        let gpu_cap = f64::from(changed.status.gpu_max_mhz.expect("GPU cap"));
        changed.status.t_star_c = Some(75.0);
        {
            let auto = changed.auto.as_mut().expect("Auto state");
            auto.cpu_verdict = VerdictState::default();
            auto.gpu_verdict = VerdictState::default();
            auto.cpu_hot = false;
            auto.fan_window.clear();
        }
        let decision = |group_c, cap| DeviceDecision {
            t_star: 75.0,
            group_c: Some(group_c),
            err_c: Some(75.0 - group_c),
            thermal: cap,
            shadow: cap,
            cap,
            selected: crate::control::device_loop::Selected::Thermal,
            hold: Hold::None,
            write_allowed: true,
            group_lost: false,
            write_immediately: false,
        };
        for t in 2..42 {
            changed.observe_v4_steady(
                &keyed_auto_sample(
                    f64::from(t),
                    changed_strategy,
                    changed_duty,
                    changed_on_ac,
                ),
                decision(75.0, cpu_cap),
                decision(74.0, gpu_cap),
                changed_duty,
                false,
            );
        }
        assert_eq!(
            changed.auto.as_ref().expect("Auto state").steady_window.len(),
            STEADY_WINDOW_N,
            "qualified samples must fill the re-keyed window",
        );
        let recorded = changed.persisted_warm_start.get(&hostile_key).expect(
            "the settled pair must be recorded under the new key",
        );
        assert_eq!(recorded.cpu_cap_w, changed.status.cpu_limit_w.expect("CPU cap"));
        assert_eq!(recorded.gpu_lock_mhz, changed.status.gpu_max_mhz.expect("GPU cap"));
        changed
    }

    #[test]
    fn strategy_change_rekeys_without_reseeding_live_device_loops() {
        assert_rekey_preserves_live_candidates("cool16", 36, false);
    }

    #[test]
    fn ac_change_rekeys_without_reseeding_live_device_loops() {
        assert_rekey_preserves_live_candidates("quiet16", 36, true);
    }

    #[test]
    fn snapped_duty_change_rekeys_without_reseeding_live_device_loops() {
        assert_rekey_preserves_live_candidates("quiet16", 40, false);
    }

    #[test]
    fn released_reentry_consumes_the_matching_paired_warm_start() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        let duty = ctl.duty_rpm_table.duty_for_rpm(ctl.status.fan_target_rpm);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..5 { ctl.on_sample(&keyed_auto_sample(f64::from(t), "quiet16", duty, false)); }
        {
            let auto = ctl.auto.as_ref().expect("Auto");
            assert_ne!(auto.cpu_loop.thermal(), 40.0, "initial entry has no matching seed");
            assert_ne!(auto.gpu_loop.thermal(), 2_400.0, "initial entry has no matching seed");
        }
        let mut released = keyed_auto_sample(5.0, "quiet16", duty, false);
        released.fan_valid = false;
        released.fanctrl = None;
        ctl.on_sample(&released);
        assert_eq!(ctl.status.tstar_state, Some(crate::types::TelemetryTStarState::Released));
        {
            let auto = ctl.auto.as_ref().expect("Auto");
            assert_eq!(auto.cpu_loop.requested(), None);
            assert_eq!(auto.gpu_loop.requested(), None);
            assert!(!auto.cpu_entry_seeded && !auto.gpu_entry_seeded);
            assert!(ctl.status.cpu.is_none() && ctl.status.gpu.is_none());
        }
        ctl.persisted_warm_start.insert(
            warm_start_key("quiet16", duty, false),
            WarmStartEntry { cpu_cap_w: 40.0, gpu_lock_mhz: 2_400 },
        );
        for t in 6..11 { ctl.on_sample(&keyed_auto_sample(f64::from(t), "quiet16", duty, false)); }
        let auto = ctl.auto.as_ref().expect("Auto");
        assert_eq!(auto.cpu_loop.thermal(), 40.0);
        assert_eq!(auto.gpu_loop.thermal(), 2_400.0);
    }

    #[test]
    fn mismatched_achieved_duty_restarts_the_steady_window() {
        let runner = FakeRunner::new();
        let (mut ctl, _gpu) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&keyed_auto_sample(0.0, "quiet16", 36, false));
        let cpu_cap = ctl.status.cpu_limit_w.expect("CPU cap");
        let gpu_cap = f64::from(ctl.status.gpu_max_mhz.expect("GPU cap"));
        ctl.status.t_star_c = Some(75.0);
        {
            let auto = ctl.auto.as_mut().expect("Auto");
            auto.cpu_verdict = VerdictState::default();
            auto.gpu_verdict = VerdictState::default();
            auto.cpu_hot = false;
            auto.fan_window.clear();
        }
        let decision = |group_c, cap| DeviceDecision {
            t_star: 75.0,
            group_c: Some(group_c),
            err_c: Some(75.0 - group_c),
            thermal: cap,
            shadow: cap,
            cap,
            selected: crate::control::device_loop::Selected::Thermal,
            hold: Hold::None,
            write_allowed: true,
            group_lost: false,
            write_immediately: false,
        };
        for t in 1..=30 {
            ctl.observe_v4_steady(
                &keyed_auto_sample(f64::from(t), "quiet16", 36, false),
                decision(75.0, cpu_cap),
                decision(74.0, gpu_cap),
                36,
                false,
            );
        }
        assert_eq!(ctl.auto.as_ref().expect("Auto").steady_window.len(), 30);
        let table_before = ctl.duty_rpm_table.clone();
        let mut mismatch = keyed_auto_sample(31.0, "quiet16", 36, false);
        mismatch.fanctrl.as_mut().expect("view").speed_pct = 99;
        ctl.observe_v4_steady(
            &mismatch,
            decision(75.0, cpu_cap),
            decision(74.0, gpu_cap),
            36,
            false,
        );
        assert!(ctl.auto.as_ref().expect("Auto").steady_window.is_empty());
        ctl.observe_v4_steady(
            &keyed_auto_sample(32.0, "quiet16", 36, false),
            decision(75.0, cpu_cap),
            decision(74.0, gpu_cap),
            36,
            false,
        );
        assert_eq!(ctl.auto.as_ref().expect("Auto").steady_window.len(), 1);
        assert!(!ctl.persisted_warm_start.contains_key(&warm_start_key("quiet16", 36, false)));
        assert_eq!(ctl.duty_rpm_table, table_before, "partial windows cannot refine the table");
    }

    #[test]
    fn calib_context_real_wiring_lets_the_step_test_actually_settle() {
        // Regression: before this task, the controller always handed the
        // runner `CalibContext::default()`, so `fanctrl_active` was always
        // false and the step test's settle gate could NEVER clear — every
        // real calibration timed out at the 5-minute cap. With the real
        // `CalibContext` wired through, a scripted run with an active,
        // reconciled fanctrl view and a controllable, steady EC reading
        // must actually leave the settle sub-phase.
        let runner = FakeRunner::new();
        let (dir, profile) = profile_fixture("calib-context-real");
        let mut ctl = Controller::new(
            RestoreGuard::new(
                &runner,
                Some(cpu_actuator(&runner, profile)),
                Some(Box::new(FakeGpu::new())),
                None,
            ),
            PersistedState::default(),
            dir.join("state.json"),
            Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        ctl.on_command(Command::StartCalibration);
        assert_eq!(
            ctl.status().calib.as_ref().map(|c| c.phase.clone()),
            Some("settle".to_string())
        );

        let settle = |t_mono: f64| -> Sample {
            let view = FanctrlView {
                strategy: "quiet16".to_string(),
                active: true,
                speed_pct: 31,
                temperature: 60.0,
                ma_temperature: 60.0,
                ma_interval: 60,
                curve: TEMP_CURVE.to_vec(),
                observed_at: Instant::now(),
                all_observed_at: Some(Instant::now()),
            };
            let ec = ec_reading_c(&[("apu@4c", 60.0), ("gpu_vr_f75303@4d", 55.0)]);
            Sample {
                t_mono,
                fan_valid: true,
                fan1_rpm: 3000.0,
                ec: Some(ec),
                ec_valid: true,
                fanctrl: Some(view),
                fanctrl_freshness: Freshness::Fresh,
                fanctrl_view_changed: true,
                cpu_temp_c: 60.0,
                cpu_temp_valid: true,
                gpu_mhz_valid: true,
                gpu_sm_mhz: 1_000.0,
                gpu_util_pct: 95.0,
                ..Sample::default()
            }
        };

        for i in 0..65u64 {
            ctl.on_sample(&settle(i as f64));
        }
        let calib = ctl.status().calib.as_ref().expect("still calibrating");
        assert_eq!(calib.phase, "cpu_step", "settle advances to CPU step");
        assert!(
            calib.step >= 1,
            "must have LEFT the settle sub-phase within 65 flat, active, \
             reconciled samples: {calib:?}"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    fn queue_confirmed_cpu_mismatch(runner: &FakeRunner) {
        for _ in 0..2 {
            crate::actuators::cmd::test_support::queue_ryzenadj_readback(runner, 0.1, 53.0, 0.0);
        }
    }

    #[test]
    fn cpu_mismatch_freezes_flags_reasserts_and_releases_to_stock_then_a_later_verified_recovers() {
        // Three paired mismatches at the V4 cadence release CPU; the next
        // verified write re-engages it and clears the hold.
        let runner = FakeRunner::new();
        let (dir, profile_path) = profile_fixture("cpu-mismatch");
        let (mut ctl, _gpu) = auto_controller(&runner, profile_path, Config::default());
        ctl.on_command(Command::SetAuto(true));
        for _ in 0..3 {
            queue_confirmed_cpu_mismatch(&runner);
        }
        let mut effects_at = std::collections::HashMap::new();
        let mut status_at = std::collections::HashMap::new();
        for i in 0..=7_u32 {
            let effects = ctl.on_sample(&Sample {
                cpu_pkg_w: 25.0,
                cpu_temp_valid: true,
                cpu_temp_c: 60.0,
                ..busy_at(f64::from(i))
            });
            effects_at.insert(i, effects);
            status_at.insert(i, ctl.status().clone());
        }
        fs::remove_dir_all(&dir).unwrap();
        let noted = |i, cause| {
            effects_at[&i]
                .iter()
                .any(|e| matches!(e, Effect::Noted { cause: got } if *got == cause))
        };
        assert!(noted(2, "auto:cpu_mismatch"), "{:?}", effects_at[&2]);
        assert!(status_at[&2].flags.contains(&StatusFlag::LimitNotSticking));
        assert!(matches!(
            status_at[&2].cpu.as_ref().expect("CPU").hold,
            crate::types::TelemetryHold::ActuatorMismatch
        ));
        assert!(noted(4, "auto:cpu_released"), "{:?}", effects_at[&2]);
        assert_eq!(status_at[&4].cpu_limit_w, None);
        assert!(
            noted(6, "auto:cpu_verdict_recovered"),
            "{:?}",
            effects_at[&6]
        );
        assert!(status_at[&6].cpu_limit_w.is_some());
        assert!(!matches!(
            status_at[&7].cpu.as_ref().expect("CPU").hold,
            crate::types::TelemetryHold::ActuatorMismatch
        ));
    }
    #[test]
    fn gpu_and_nvme_hot_thresholds_come_from_the_live_config_not_the_compiled_defaults() {
        let runner = FakeRunner::new();
        let config = Config {
            gpu_hot_c: 86.0,
            nvme_hot_c: 55.0,
            ..Config::default()
        };
        // Sanity: the values under test are the ones the guards will see.
        let config = config.sanitized();
        assert_eq!((config.gpu_hot_c, config.nvme_hot_c), (86.0, 55.0));
        let (mut ctl, _gpu) = auto_controller(
            &runner,
            PathBuf::from("/nonexistent/platform_profile"),
            config,
        );
        ctl.on_command(Command::SetAuto(true));
        let s = Sample {
            gpu_temp_valid: true,
            gpu_temp_c: 87.0, // below GPU_HOT_C_DEFAULT (88), above the configured 86
            nvme_temp_c: Some(60.0), // below NVME_HOT_C_DEFAULT (80), above the configured 55
            cpu_temp_valid: true,
            cpu_temp_c: 60.0,
            ..busy_at(5.0)
        };
        ctl.on_sample(&s);
        assert!(
            ctl.status().flags.contains(&StatusFlag::GpuHot),
            "87C must trip a configured 86C gpu_hot_c threshold even though it's \
             under the compiled-in 88C default: {:?}",
            ctl.status().flags
        );
        assert!(
            ctl.status().flags.contains(&StatusFlag::NvmeHot),
            "60C must trip a configured 55C nvme_hot_c threshold even though it's \
             well under the compiled-in 80C default: {:?}",
            ctl.status().flags
        );
    }

    #[test]
    fn review_socketless_entry_seeds_groups_once() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let mut sample = curve_sample(0.0, 60.0, 60.0, TEMP_CURVE);
        sample.fanctrl = None;
        sample.fanctrl_freshness = Freshness::Absent;
        ctl.on_sample(&sample);
        assert_eq!(ctl.status.t_star_c, Some(60.0));
        assert!(ctl.auto.as_ref().unwrap().source_initialised);
    }

    #[test]
    fn review_released_clears_engagement_and_releases_only_on_transition() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..6 { ctl.on_sample(&busy_at(f64::from(t))); }
        ctl.add_flag(StatusFlag::ReadbackBlind);
        let invalid = Sample { fan_valid: false, ..busy_at(6.0) };
        assert!(ctl.on_sample(&invalid).contains(&Effect::Released));
        let auto = ctl.auto.as_ref().unwrap();
        assert!(!auto.cpu_entry_seeded && !auto.gpu_entry_seeded);
        assert!(auto.cpu_draw_window.is_empty());
        assert!(auto.gpu_commands.is_empty());
        assert!(!ctl.status.flags.contains(&StatusFlag::ReadbackBlind));
        assert!(!ctl.on_sample(&Sample { t_mono: 7.0, ..invalid }).contains(&Effect::Released));
        ctl.on_sample(&busy_at(8.0));
        assert!(ctl.auto.as_ref().unwrap().cpu_entry_seeded);
    }

    #[test]
    fn review_auto_exit_clears_v3_decisions() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        assert!(ctl.status.flags.contains(&StatusFlag::NotCalibrated));
        ctl.on_sample(&busy_at(0.0));
        assert!(ctl.status.cpu.is_some());
        ctl.on_command(Command::SetAuto(false));
        assert!(ctl.status.cpu.is_none() && ctl.status.gpu.is_none());
        assert!(ctl.status.tstar_state.is_none());
        assert!(ctl.status.telemetry_flags.is_empty());
        assert!(
            !ctl.status.flags.contains(&StatusFlag::NotCalibrated),
            "the active-key calibration diagnostic must not leak into Monitor"
        );
    }

    #[test]
    fn unmarked_long_sample_gap_restarts_held_cadence_without_advancing_target() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let held_sample = |t| {
            let mut sample = busy_at(t);
            sample.fan1_rpm = 2_000.0;
            sample.fan2_rpm = 1_950.0;
            sample
        };
        ctl.on_sample(&held_sample(0.0));
        ctl.on_sample(&held_sample(1.0));
        assert_eq!(ctl.status.tstar_state, Some(crate::types::TelemetryTStarState::Held));
        let before = ctl.status.t_star_c.expect("Held target");

        ctl.on_sample(&held_sample(7_201.0));

        assert_eq!(ctl.status.tstar_state, Some(crate::types::TelemetryTStarState::Held));
        assert_eq!(
            ctl.status.t_star_c,
            Some(before),
            "wall time is not valid Held PI control time"
        );
    }

    #[test]
    fn review_cpu_hot_hysteresis_and_resume_clear_streak() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..3 {
            ctl.on_sample(&Sample { cpu_temp_valid: true, cpu_temp_c: 90.0, ..busy_at(f64::from(t)) });
        }
        let previous = ctl.auto.as_mut().unwrap().cpu_ratchet.step(false, false, None);
        ctl.on_sample(&Sample { cpu_temp_valid: true, cpu_temp_c: 88.0, ..busy_at(3.0) });
        let next = ctl.auto.as_mut().unwrap().cpu_ratchet.step(false, false, None);
        assert_eq!(next, previous - CPU_MAX_RATCHET_DOWN_RATE_W);
        ctl.on_sample(&Sample { resumed: true, cpu_temp_valid: true, cpu_temp_c: 90.0, ..busy_at(100.0) });
        assert_eq!(ctl.auto.as_ref().unwrap().cpu_hot_streak, 1);
        assert!(!ctl.auto.as_ref().unwrap().cpu_verdict.in_episode());
    }

    #[test]
    fn review_live_floors_bound_hot_ratchets() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..25 {
            ctl.on_sample(&Sample { cpu_temp_valid: true, cpu_temp_c: 90.0,
                gpu_temp_valid: true, gpu_temp_c: 89.0, ..busy_at(f64::from(t)) });
        }
        ctl.on_command(Command::SetFloors { cpu_w: 40.0, gpu_mhz: 2500 });
        ctl.on_sample(&Sample { cpu_temp_valid: true, cpu_temp_c: 90.0,
            gpu_temp_valid: true, gpu_temp_c: 89.0, ..busy_at(25.0) });
        let auto = ctl.auto.as_mut().unwrap();
        assert!(auto.cpu_ratchet.step(false, false, None) >= 40.0);
        assert!(auto.gpu_ratchet.step(false, false, None) >= 2500.0);
    }

    #[test]
    fn review_gpu_commands_use_completion_time_including_reassert() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        let acquired = Instant::now();
        ctl.completion_clock = Box::new(move || acquired + Duration::from_secs(3));
        ctl.on_command(Command::SetAuto(true));
        let sample = Sample { acquired_at: Some(acquired), ..busy_at(10.0) };
        ctl.on_sample(&sample);
        assert_eq!(ctl.auto.as_ref().unwrap().gpu_commands.back().unwrap().completed_at_s, 13.0);
        ctl.reassert_actuators(&sample);
        let auto = ctl.auto.as_mut().unwrap();
        assert_eq!(auto.gpu_commands.len(), 2);
        assert_eq!(auto.gpu_commands.back().unwrap().completed_at_s, 13.0);
        assert_eq!(auto.gpu_verifier.as_mut().unwrap().verify_paired(
            95.0, 3000, 12.0, auto.gpu_commands.make_contiguous()), WriteVerdict::Unverifiable);
    }

    #[test]
    fn review_failed_gpu_attempts_keep_two_second_cadence() {
        let runner = FakeRunner::new();
        let fake = FakeGpu::new();
        let calls = fake.calls();
        *fake.failures().lock().unwrap() = 10;
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.guard.gpu = Some(Box::new(fake));
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.auto.as_ref().unwrap().last_gpu_write_t_mono, Some(0.0));
        ctl.on_sample(&busy_at(1.0));
        assert_eq!(ctl.auto.as_ref().unwrap().last_gpu_write_t_mono, Some(0.0));
        ctl.on_sample(&busy_at(2.0));
        assert_eq!(ctl.auto.as_ref().unwrap().last_gpu_write_t_mono, Some(2.0));
        assert!(calls.lock().unwrap().is_empty());
        assert!(ctl.status.gpu_max_mhz.is_none());
    }

    #[test]
    fn review_gpu_mismatch_rewrites_then_release_does_not_relock_same_sample() {
        let runner = FakeRunner::new();
        let (mut ctl, calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        // Ignore the actual cap for enough consecutive loaded observations.
        for t in 2..5 { ctl.on_sample(&Sample { gpu_sm_mhz: 4000.0, ..busy_at(f64::from(t)) }); }
        let count = calls.lock().unwrap().len();
        ctl.on_sample(&Sample { gpu_sm_mhz: 4000.0, ..busy_at(5.0) });
        assert!(calls.lock().unwrap().len() > count, "confirmed mismatch must reassert even an unchanged cap");
        ctl.on_sample(&Sample { gpu_sm_mhz: 4000.0, ..busy_at(6.0) });
        assert!(ctl.status.gpu_max_mhz.is_none(), "release must survive the rest of its sample");
    }

    #[test]
    fn review_upward_curve_edit_restores_thermal_candidate_only() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        drive_curve(&mut ctl, 0.0, 25);
        assert_eq!(ctl.status.tstar_state, Some(crate::types::TelemetryTStarState::Curve));
        let auto = ctl.auto.as_mut().unwrap();
        auto.cpu_loop.transfer_thermal(30.0, 0.0);
        auto.cpu_loop.transfer_shadow(35.0);
        let applied = ctl.status.cpu_limit_w;
        let higher: Vec<_> = TEMP_CURVE.iter().map(|(temp, duty)| (temp + 4.0, *duty)).collect();
        ctl.on_sample(&curve_sample(25.0, 75.0, 74.0, &higher));
        assert_eq!(ctl.auto.as_ref().unwrap().cpu_loop.thermal(), ctl.config.cpu_max_w);
        assert_eq!(ctl.status.cpu.as_ref().unwrap().shadow, 35.0);
        assert!(ctl.status.cpu_limit_w.unwrap() <= applied.unwrap() + 10.0);
    }

    #[test]
    fn review_raw_reconciliation_and_implausible_diagnostics_remain_distinct() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..5 {
            let mut s = curve_sample(f64::from(t), 120.0, 120.0, TEMP_CURVE);
            s.ec = Some(ec_reading_c(&[("cpu@4c", 70.0), ("ambient_f75303@4d", 40.0), ("charger_f75303@4d", 120.0)]));
            ctl.on_sample(&s);
        }
        assert!(!ctl.status.flags.contains(&StatusFlag::EcMismatch));
        assert!(ctl.status.telemetry_flags.contains(&crate::types::TelemetryFlag::EcImplausible {
            label: "charger_f75303@4d".into(), active: true,
        }));
    }

    #[test]
    fn review_group_lost_emits_device_diagnostic() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        for t in 1..=61 {
            ctl.on_sample(&curve_sample(f64::from(t), 75.0, 74.0, TEMP_CURVE));
        }
        assert!(ctl.status.telemetry_flags.contains(&crate::types::TelemetryFlag::GroupLost {
            device: crate::types::TelemetryDeviceName::Gpu, active: true,
        }));
    }

    #[test]
    fn review_not_calibrated_is_informational_without_fitted_gains() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        assert_eq!(ctl.status.mode, Mode::Auto);
        assert!(ctl.status.flags.contains(&StatusFlag::NotCalibrated));
        assert_eq!(flag_severity(StatusFlag::NotCalibrated), Severity::Info);
    }

    #[test]
    fn review_ac_suppressed_gpu_mismatch_does_not_latch_device_loop() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        for t in 1..=3 {
            ctl.on_sample(&Sample { gpu_sm_mhz: 4000.0, on_ac: true, ..busy_at(f64::from(t)) });
        }
        assert_ne!(ctl.auto.as_ref().unwrap().gpu_actuator_state, ActuatorState::Mismatch);
        assert!(!ctl.auto.as_ref().unwrap().gpu_verdict.in_episode());
    }

    #[test]
    fn review_biased_steady_fan_refines_before_pair_becomes_qualified() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=40 { ctl.on_sample(&rpm_view_sample(f64::from(t), 2800.0, "quiet16", 36, false)); }
        assert!(ctl.persisted_warm_start.is_empty(), "40 out-of-tolerance samples cannot qualify a pair");
        assert!(ctl.duty_rpm_table.rpm_for_duty(36) < 3030.0, "a stable measured duty must refine an inaccurate table");
        for t in 41..=400 { ctl.on_sample(&rpm_view_sample(f64::from(t), 2800.0, "quiet16", 36, false)); }
        assert!(!ctl.persisted_warm_start.is_empty(), "40 subsequent in-tolerance samples qualify the pair");
    }

    #[test]
    fn review_first_socket_view_seeds_replica_without_replacing_running_source() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        let mut sample = curve_sample(0.0, 60.0, 60.0, TEMP_CURVE);
        sample.fanctrl = None;
        sample.fanctrl_freshness = Freshness::Absent;
        ctl.on_sample(&sample);
        ctl.on_sample(&curve_sample(1.0, 74.0, 74.0, TEMP_CURVE));
        assert_eq!(ctl.status.ec_ma_c, Some(74.0));
        assert_eq!(ctl.status.t_star_c, Some(60.0), "first socket must not replace running TStarSource");
    }

    #[test]
    fn review_resume_discards_mismatch_hold_and_quit_clears_decisions() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        ctl.auto.as_mut().unwrap().cpu_actuator_state = ActuatorState::Mismatch;
        ctl.on_sample(&busy_at(1.0));
        assert_eq!(ctl.status.cpu.as_ref().unwrap().hold, crate::types::TelemetryHold::ActuatorMismatch);
        ctl.on_sample(&Sample { resumed: true, ..busy_at(100.0) });
        assert_ne!(ctl.status.cpu.as_ref().unwrap().hold, crate::types::TelemetryHold::ActuatorMismatch);
        ctl.on_command(Command::Quit);
        assert!(ctl.status.cpu.is_none() && ctl.status.gpu.is_none());
    }

    #[test]
    fn review_curve_reports_live_refined_rpm_reference() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        drive_curve(&mut ctl, 0.0, 25);
        let duty = ctl.duty_rpm_table.duty_for_rpm(ctl.status.fan_target_rpm);
        ctl.duty_rpm_table.refine(duty, 2900.0);
        ctl.on_sample(&curve_sample(25.0, 75.0, 74.0, TEMP_CURVE));
        assert_eq!(ctl.status.duty_cmd, Some(duty));
        assert_eq!(ctl.status.snapped_rpm, ctl.duty_rpm_table.rpm_for_duty(duty));
        assert_ne!(ctl.status.snapped_rpm, DutyRpmTable::default().rpm_for_duty(duty));
    }

    #[test]
    fn review_live_floor_can_raise_a_previously_lower_configured_ceiling() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller(&runner, PathBuf::from("/nonexistent/platform_profile"),
            Config { cpu_max_w: 30.0, gpu_max_mhz: 1500, ..Config::default() });
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        ctl.on_command(Command::SetFloors { cpu_w: 40.0, gpu_mhz: 2000 });
        ctl.on_sample(&busy_at(1.0));
        assert!(ctl.status.cpu.as_ref().unwrap().cap >= ctl.config.cpu_floor_w);
        assert!(ctl.status.gpu.as_ref().unwrap().cap >= f64::from(ctl.config.gpu_floor_mhz));
    }

    #[test]
    fn review_resume_records_every_completed_reassert_without_scoring_its_sample() {
        let runner = FakeRunner::new();
        let (mut ctl, calls) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&busy_at(0.0));
        let before = calls.lock().unwrap().len();
        ctl.on_sample(&Sample { resumed: true, ..busy_at(100.0) });
        let writes = calls.lock().unwrap()[before..].iter().filter(|c| matches!(c, GpuCall::Set(_))).count();
        let auto = ctl.auto.as_ref().unwrap();
        assert_eq!(auto.gpu_commands.len(), writes);
        assert_eq!(auto.gpu_actuator_state, ActuatorState::Unverifiable);
    }

    #[test]
    fn review_invalid_rapl_delta_requires_five_new_draw_samples() {
        for invalid in [0.0, f64::NAN, -1.0] {
            let runner = FakeRunner::new();
            let (mut ctl, _) = auto_controller_no_profile(&runner);
            ctl.on_command(Command::SetAuto(true));
            for t in 0..5 { ctl.on_sample(&busy_at(f64::from(t))); }
            ctl.on_sample(&Sample { cpu_pkg_w: invalid, ..busy_at(5.0) });
            assert_eq!(ctl.status.cpu.as_ref().unwrap().hold, crate::types::TelemetryHold::DrawUnavailable);
            for t in 6..10 {
                ctl.on_sample(&busy_at(f64::from(t)));
                assert_eq!(ctl.status.cpu.as_ref().unwrap().hold, crate::types::TelemetryHold::DrawUnavailable);
            }
            ctl.on_sample(&busy_at(10.0));
            assert_ne!(ctl.status.cpu.as_ref().unwrap().hold, crate::types::TelemetryHold::DrawUnavailable);
        }
    }

    #[test]
    fn review_gpu_first_clock_seeds_shadow_after_thermal_only_entry() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&Sample { gpu_mhz_valid: false, ..busy_at(0.0) });
        let gpu = ctl.status.gpu.as_ref().unwrap();
        assert_eq!(gpu.shadow, gpu.thermal);
        ctl.on_sample(&busy_at(1.0));
        assert_eq!(ctl.status.gpu.as_ref().unwrap().shadow, 1800.0 + ctl.config.shadow_headroom_gpu_mhz);
    }

    #[test]
    fn review_upward_curve_restore_waits_for_mismatch_recovery() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        drive_curve(&mut ctl, 0.0, 25);
        let auto = ctl.auto.as_mut().unwrap();
        auto.cpu_loop.transfer_thermal(30.0, 0.0);
        auto.cpu_loop.transfer_shadow(35.0);
        auto.cpu_actuator_state = ActuatorState::Mismatch;
        auto.last_cpu_write_t_mono = Some(25.0);
        let higher: Vec<_> = TEMP_CURVE.iter().map(|(temp, duty)| (temp + 4.0, *duty)).collect();
        ctl.on_sample(&curve_sample(25.0, 75.0, 74.0, &higher));
        assert_eq!(ctl.auto.as_ref().unwrap().cpu_loop.thermal(), 30.0);
        ctl.auto.as_mut().unwrap().cpu_actuator_state = ActuatorState::Verified;
        ctl.on_sample(&curve_sample(26.0, 75.0, 74.0, &higher));
        assert_eq!(ctl.auto.as_ref().unwrap().cpu_loop.thermal(), ctl.config.cpu_max_w);
    }

    #[test]
    fn review_every_qualified_steady_tick_refines_and_updates_the_pair() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.on_command(Command::SetAuto(true));
        for t in 0..=40 { ctl.on_sample(&rpm_view_sample(f64::from(t), 3000.0, "quiet16", 36, false)); }
        let first = ctl.duty_rpm_table.rpm_for_duty(36);
        assert_eq!(first, 3024.0);
        ctl.on_sample(&rpm_view_sample(41.0, 3000.0, "quiet16", 36, false));
        assert_eq!(ctl.duty_rpm_table.rpm_for_duty(36), 0.8 * first + 0.2 * 3000.0);
        let pair = &ctl.persisted_warm_start[&warm_start_key("quiet16", 36, false)];
        assert_eq!(pair.cpu_cap_w, ctl.status.cpu_limit_w.unwrap());
        assert_eq!(pair.gpu_lock_mhz, ctl.status.gpu_max_mhz.unwrap());
    }

    #[test]
    fn review_round2_low_cpu_warm_waits_for_five_valid_draws() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        let duty = ctl.duty_rpm_table.duty_for_rpm(ctl.status.fan_target_rpm);
        ctl.persisted_warm_start.insert(warm_start_key("quiet16", duty, false),
            WarmStartEntry { cpu_cap_w: 15.0, gpu_lock_mhz: 1000 });
        ctl.on_command(Command::SetAuto(true));
        for t in 0..4 {
            ctl.on_sample(&Sample { cpu_pkg_w: 0.0, ..busy_at(f64::from(t)) });
            assert!(ctl.status.cpu.as_ref().unwrap().thermal >= 50.0,
                "an unusable warm record must not lower thermal-only entry");
        }
        for t in 4..8 { ctl.on_sample(&busy_at(f64::from(t))); }
        ctl.on_sample(&busy_at(8.0));
        let cpu = ctl.status.cpu.as_ref().unwrap();
        assert!(cpu.thermal >= 35.0 && cpu.shadow >= 35.0, "{cpu:?}");
        // CPU downward output is deliberately unrestricted; the advisory
        // record must never push it below the live entry headroom.
        assert!(cpu.cap >= 35.0, "{cpu:?}");
        assert!(ctl.status.cpu_limit_w.is_some_and(|cap| cap >= 35.0));
    }

    #[test]
    fn review_round2_low_gpu_warm_waits_for_first_clock() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        let duty = ctl.duty_rpm_table.duty_for_rpm(ctl.status.fan_target_rpm);
        ctl.persisted_warm_start.insert(warm_start_key("quiet16", duty, false),
            WarmStartEntry { cpu_cap_w: 15.0, gpu_lock_mhz: 1000 });
        ctl.on_command(Command::SetAuto(true));
        for t in 0..4 {
            ctl.on_sample(&Sample { gpu_mhz_valid: false, ..busy_at(f64::from(t)) });
            assert!(ctl.status.gpu.as_ref().unwrap().thermal >= 3000.0,
                "an unusable warm record must not lower thermal-only entry");
        }
        let before = ctl.status.gpu.as_ref().unwrap().cap;
        ctl.on_sample(&busy_at(4.0));
        let gpu = ctl.status.gpu.as_ref().unwrap();
        let entry_floor = 1800.0 + ctl.config.shadow_headroom_gpu_mhz;
        assert!(gpu.thermal >= entry_floor && gpu.shadow >= entry_floor, "{gpu:?}");
        assert!(gpu.cap >= entry_floor && (gpu.cap - before).abs() <= 105.0, "{gpu:?}");
    }

    // Advancing the shared clock inside the actuator models a blocking
    // three-second failure without wall-clock sleeps or scheduling races.
    #[derive(Clone)]
    struct ClockedWriteFailure {
        now: Arc<Mutex<Instant>>,
        attempts: Arc<Mutex<usize>>,
    }

    impl ClockedWriteFailure {
        fn fail(&self) {
            *self.now.lock().unwrap() += Duration::from_secs(3);
            *self.attempts.lock().unwrap() += 1;
        }
    }

    impl crate::actuators::cmd::Runner for ClockedWriteFailure {
        fn run(&self, program: &str, args: &[&str]) -> std::io::Result<std::process::Output> {
            if program == "ryzenadj" && args.iter().any(|arg| arg.starts_with("--stapm-limit=")) {
                self.fail();
                return Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "blocking write failure"));
            }
            if program == "ryzenadj" && args == ["--info"] {
                use crate::actuators::cmd::test_support::{output_with_stdout, ryzenadj_info_table};
                return Ok(output_with_stdout(&ryzenadj_info_table(54.0, 53.0, 54.0)));
            }
            Ok(crate::actuators::cmd::test_support::output_with_code(0))
        }
    }

    impl crate::actuators::gpu::GpuClockCtl for ClockedWriteFailure {
        fn set_max_clock(&mut self, _: u32) -> color_eyre::Result<()> {
            self.fail();
            Err(color_eyre::eyre::eyre!("blocking write failure"))
        }
        fn release(&mut self) -> color_eyre::Result<()> { Ok(()) }
        fn applied(&self) -> Option<u32> { None }
    }

    #[test]
    fn review_round2_failed_cpu_cadence_starts_at_completion() {
        let start = Instant::now();
        let fake = ClockedWriteFailure {
            now: Arc::new(Mutex::new(start)), attempts: Arc::new(Mutex::new(0)),
        };
        let mut ctl = Controller::new(
            RestoreGuard::new(&fake, Some(CpuActuator::new(&fake,
                PathBuf::from("/nonexistent/platform_profile"))), None, None),
            calibrated(), PathBuf::from("/nonexistent/state.json"), Config::default(),
            PathBuf::from("/nonexistent/config.toml"),
        );
        let clock = Arc::clone(&fake.now);
        ctl.completion_clock = Box::new(move || *clock.lock().unwrap());
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&Sample { acquired_at: Some(start), ..busy_at(0.0) });
        assert_eq!(ctl.auto.as_ref().unwrap().last_cpu_write_t_mono, Some(3.0));
        for t in [2_u64, 4] {
            *fake.now.lock().unwrap() = start + Duration::from_secs(t.max(3));
            ctl.on_sample(&Sample { acquired_at: Some(start + Duration::from_secs(t)), ..busy_at(t as f64) });
            assert_eq!(*fake.attempts.lock().unwrap(), 1, "buffered sample must not retry");
        }
        *fake.now.lock().unwrap() = start + Duration::from_secs(5);
        ctl.on_sample(&Sample { acquired_at: Some(start + Duration::from_secs(5)), ..busy_at(5.0) });
        assert_eq!(*fake.attempts.lock().unwrap(), 2);
        assert_eq!(ctl.auto.as_ref().unwrap().last_cpu_write_t_mono, Some(8.0));
        assert!(ctl.status.cpu_limit_w.is_none());

        // A cap from an earlier successful command may also be reasserted.
        ctl.status.cpu_limit_w = Some(40.0);
        *fake.now.lock().unwrap() = start + Duration::from_secs(10);
        let reassert = Sample { acquired_at: Some(start + Duration::from_secs(10)), ..busy_at(10.0) };
        assert_eq!(ctl.reassert_actuators(&reassert), Some(false));
        ctl.last_reassert = Some(10.0); // caller schedules the next periodic reassert
        assert_eq!(ctl.auto.as_ref().unwrap().last_cpu_write_t_mono, Some(13.0));
        ctl.on_sample(&Sample { acquired_at: Some(start + Duration::from_secs(12)), ..busy_at(12.0) });
        assert_eq!(*fake.attempts.lock().unwrap(), 3);
    }

    #[test]
    fn review_round2_failed_gpu_cadence_starts_at_completion() {
        let start = Instant::now();
        let fake = ClockedWriteFailure {
            now: Arc::new(Mutex::new(start)), attempts: Arc::new(Mutex::new(0)),
        };
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.guard.gpu = Some(Box::new(fake.clone()));
        let clock = Arc::clone(&fake.now);
        ctl.completion_clock = Box::new(move || *clock.lock().unwrap());
        ctl.on_command(Command::SetAuto(true));
        ctl.on_sample(&Sample { acquired_at: Some(start), ..busy_at(0.0) });
        assert_eq!(ctl.auto.as_ref().unwrap().last_gpu_write_t_mono, Some(3.0));
        for t in [2_u64, 4] {
            *fake.now.lock().unwrap() = start + Duration::from_secs(t.max(3));
            ctl.on_sample(&Sample { acquired_at: Some(start + Duration::from_secs(t)), ..busy_at(t as f64) });
            assert_eq!(*fake.attempts.lock().unwrap(), 1, "buffered sample must not retry");
        }
        *fake.now.lock().unwrap() = start + Duration::from_secs(5);
        ctl.on_sample(&Sample { acquired_at: Some(start + Duration::from_secs(5)), ..busy_at(5.0) });
        assert_eq!(*fake.attempts.lock().unwrap(), 2);
        assert_eq!(ctl.auto.as_ref().unwrap().last_gpu_write_t_mono, Some(8.0));
        assert!(ctl.auto.as_ref().unwrap().gpu_commands.is_empty());
        assert!(ctl.status.gpu_max_mhz.is_none());

        ctl.status.gpu_max_mhz = Some(2000);
        *fake.now.lock().unwrap() = start + Duration::from_secs(10);
        let reassert = Sample { acquired_at: Some(start + Duration::from_secs(10)), ..busy_at(10.0) };
        assert_eq!(ctl.reassert_actuators(&reassert), Some(false));
        ctl.last_reassert = Some(10.0);
        assert_eq!(ctl.auto.as_ref().unwrap().last_gpu_write_t_mono, Some(13.0));
        ctl.on_sample(&Sample { acquired_at: Some(start + Duration::from_secs(12)), ..busy_at(12.0) });
        assert_eq!(*fake.attempts.lock().unwrap(), 3);
        assert!(ctl.auto.as_ref().unwrap().gpu_commands.is_empty());
    }

    #[test]
    fn test_diagnostics_reports_live_guard_and_verdict_state_without_status_changes() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        assert!(ctl.test_diagnostics().is_none());
        ctl.on_command(Command::SetAuto(true));
        let before = ctl.status().clone();
        let diagnostics = ctl.test_diagnostics().expect("Auto diagnostics");
        assert_eq!(diagnostics.cpu_guard_max, ctl.config.cpu_max_w);
        assert_eq!(diagnostics.gpu_guard_max, f64::from(ctl.config.gpu_max_mhz));
        assert_eq!(diagnostics.cpu_actuator, ActuatorState::Unverifiable);
        assert_eq!(diagnostics.gpu_actuator, ActuatorState::Unverifiable);
        assert_eq!(diagnostics.cpu_mismatch_strikes, 0);
        assert_eq!(diagnostics.gpu_mismatch_strikes, 0);
        assert!(!diagnostics.cpu_released && !diagnostics.gpu_released);
        assert_eq!(diagnostics.reconciliation_ma_c, None);
        assert!(!diagnostics.reconciliation_ready);
        assert!(!diagnostics.reconciliation_ever_scored);
        assert!(!diagnostics.reconciliation_mismatch);
        assert_eq!(diagnostics.reconciliation_scored_count, 0);
        assert_eq!(ctl.status(), &before, "diagnostic read changed public status");
    }

    #[test]
    fn test_emitted_telemetry_flags_matches_decision_composition_without_mutation() {
        let runner = FakeRunner::new();
        let (mut ctl, _) = auto_controller_no_profile(&runner);
        ctl.status.telemetry_flags = vec![crate::types::TelemetryFlag::SteepCurve { active: true }];
        ctl.status.flags = vec![StatusFlag::NotCalibrated];
        let before = ctl.status().clone();
        assert_eq!(
            ctl.test_emitted_telemetry_flags(),
            vec![
                crate::types::TelemetryFlag::SteepCurve { active: true },
                crate::types::TelemetryFlag::legacy(StatusFlag::NotCalibrated.as_str()),
            ]
        );
        assert_eq!(ctl.status(), &before, "flag composition changed public status");
    }
}

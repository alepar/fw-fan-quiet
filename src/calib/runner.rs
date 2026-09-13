//! Sample-driven calibration runners. [`PerDeviceCalibRunner`] is the active
//! revision-4 path: shared settle, CPU-watt step, GPU-clock step, keyed gain
//! persistence.

use crate::state::PersistedState;
use crate::calib::step::{CalibDevice, PerDeviceCalibContext, PerDeviceStepTest};
use crate::types::Sample;

/// Burner threads for the CPU step test; comfortably above the core count.
pub const BURNER_THREADS: usize = 24;

#[cfg(test)]
mod per_device_runner_tests {
    use super::*;
    use crate::calib::step::{DEVICE_FIT_WINDOW_S, PerDeviceCalibContext};
    use crate::control::device_loop::Gains;

    #[test]
    fn per_device_runner_starts_directly_in_shared_settle() {
        let mut runner = PerDeviceCalibRunner::new(Default::default(), Default::default());
        assert!(runner.start().is_empty());
        let progress = runner.progress();
        assert_eq!(progress.phase, "settle");
        assert_eq!((progress.step, progress.total), (0, 3));
    }

    fn context() -> PerDeviceCalibContext {
        PerDeviceCalibContext {
            cpu_cap_w: Some(20.0), cpu_cap_verified: true,
            cpu_cap_checked_at_s: None, cpu_cap_readback: None, cpu_cap_reset_reason: None,
            cpu_cap_completed_at_s: Some(0.0),
            gpu_cap_mhz: Some(1_000), gpu_cap_verified: true,
            gpu_cap_completed_at_s: Some(0.0), use_current_caps: true,
            cpu_floor_w: 8.0, gpu_floor_mhz: 210,
            cpu_max_w: 80.0, gpu_max_mhz: 3_090,
            cpu_group_c: Some(50.0), gpu_group_c: Some(55.0),
            fanctrl_active: true, ec_mismatch: false, argmax_controllable: true,
            cpu_hot_c: 90.0, gpu_hot_c: 88.0,
            strategy: Some("quiet16".into()), ma_interval: Some(60),
        }
    }

    fn response(base: f64, gain: f64, tau: f64, theta: f64, delta: f64, t: f64) -> f64 {
        if t <= theta { base } else { base + gain * delta * (1.0 - (-(t - theta) / tau).exp()) }
    }

    fn sample_at(t_mono: f64) -> Sample {
        Sample { t_mono, fan_valid: true, fan1_rpm: 2_400.0, ..Sample::default() }
    }

    fn drive_to_cpu_response(runner: &mut PerDeviceCalibRunner, ctx: &PerDeviceCalibContext) {
        runner.on_sample(&sample_at(0.0), ctx);
        let mut effects = Vec::new();
        for second in 1..=61 {
            effects = runner.on_sample(&sample_at(f64::from(second)), ctx);
        }
        assert!(effects.contains(&RunnerEffect::SetCpuMaxWatts(35.0)), "{effects:?}");
    }

    #[test]
    fn accepted_gains_merge_under_strategy_interval_and_completion_is_stamped() {
        let mut existing = std::collections::BTreeMap::new();
        existing.insert("cool16:30".into(), Gains { kc: 0.3, ti_s: 30.0 });
        let mut runner = PerDeviceCalibRunner::new(existing.clone(), Default::default());
        runner.start();
        let ctx = context();
        drive_to_cpu_response(&mut runner, &ctx);
        let mut cpu_step = ctx.clone();
        cpu_step.cpu_cap_w = Some(35.0);
        cpu_step.cpu_cap_completed_at_s = Some(61.0);
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            let mut tick = cpu_step.clone();
            tick.cpu_group_c = Some(response(50.0, 0.8, 35.0, 20.0, 15.0, elapsed as f64));
            tick.gpu_group_c = Some(response(55.0, 0.005, 35.0, 20.0, 15.0, elapsed as f64));
            runner.on_sample(&sample_at(61.0 + elapsed as f64), &tick);
        }
        let mut recovery = ctx.clone();
        recovery.cpu_cap_completed_at_s = Some(421.0);
        recovery.gpu_cap_completed_at_s = Some(421.0);
        let mut recovery_end = Vec::new();
        for elapsed in 1..=61 {
            recovery_end = runner.on_sample(&sample_at(421.0 + elapsed as f64), &recovery);
        }
        assert_eq!(recovery_end, vec![RunnerEffect::SetGpuMaxClock(1_500)]);
        let mut gpu_step = ctx.clone();
        gpu_step.cpu_cap_completed_at_s = Some(482.0);
        gpu_step.gpu_cap_mhz = Some(1_500);
        gpu_step.gpu_cap_completed_at_s = Some(482.0);
        let mut end = Vec::new();
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            let mut tick = gpu_step.clone();
            tick.gpu_group_c = Some(response(55.0, 0.009_527_65, 32.564, 38.4515, 500.0, elapsed as f64));
            tick.cpu_group_c = Some(response(50.0, 0.001, 15.0, 60.0, 500.0, elapsed as f64));
            end = runner.on_sample(&sample_at(482.0 + elapsed as f64), &tick);
        }
        let state = end.iter().find_map(|effect| match effect {
            RunnerEffect::SaveState(state) => Some(state.as_ref()),
            _ => None,
        }).expect("terminal save");
        assert_eq!(state.cpu_gains.get("cool16:30"), existing.get("cool16:30"));
        assert!(state.cpu_gains.contains_key("quiet16:60"));
        assert!(state.gpu_gains.contains_key("quiet16:60"));
        state.calibrated_at.as_deref().expect("stamp").parse::<u64>().expect("unix stamp");
        assert!(end.contains(&RunnerEffect::Finished));
    }

    #[test]
    fn rejected_device_keeps_the_previously_resolved_keyed_gain() {
        let old = Gains { kc: 0.25, ti_s: 40.0 };
        let mut cpu = std::collections::BTreeMap::new();
        cpu.insert("quiet16:60".into(), old);
        let mut runner = PerDeviceCalibRunner::new(cpu, Default::default());
        runner.start();
        let ctx = context();
        drive_to_cpu_response(&mut runner, &ctx);
        let mut cpu_step = ctx.clone();
        cpu_step.cpu_cap_w = Some(35.0);
        cpu_step.cpu_cap_completed_at_s = Some(61.0);
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            runner.on_sample(&sample_at(61.0 + elapsed as f64), &cpu_step);
        }
        let mut recovery = ctx.clone();
        recovery.cpu_cap_completed_at_s = Some(421.0);
        recovery.gpu_cap_completed_at_s = Some(421.0);
        for elapsed in 1..=61 {
            runner.on_sample(&sample_at(421.0 + elapsed as f64), &recovery);
        }
        let mut gpu_step = ctx.clone();
        gpu_step.cpu_cap_completed_at_s = Some(482.0);
        gpu_step.gpu_cap_mhz = Some(1_500);
        gpu_step.gpu_cap_completed_at_s = Some(482.0);
        let mut end = Vec::new();
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            end = runner.on_sample(&sample_at(482.0 + elapsed as f64), &gpu_step);
        }
        let state = end.iter().find_map(|effect| match effect {
            RunnerEffect::SaveState(state) => Some(state.as_ref()),
            _ => None,
        }).expect("terminal save");
        assert_eq!(state.cpu_gains.get("quiet16:60"), Some(&old));
    }
}

/// What one `start`/`on_sample`/`abort` call did — mapped onto actuators,
/// burner, state file and UI by the controller; asserted on in tests.
#[derive(Debug, Clone, PartialEq)]
pub enum RunnerEffect {
    /// Direct CPU sustained-cap write for the per-device calibration runner.
    SetCpuMaxWatts(f64),
    /// Lock the GPU max clock (MHz).
    SetGpuMaxClock(u32),
    /// Start the CPU burner with this many threads (replaces a running one).
    StartBurner(usize),
    /// Stop the CPU burner.
    StopBurner,
    /// One native per-device thermal model landed successfully.
    FittedDevice {
        device: crate::calib::step::CalibDevice,
        gains: crate::control::device_loop::Gains,
    },
    /// A device step was skipped or rejected; existing keyed gains remain.
    Noted(String),
    /// A disturbed measurement is being retried, not rejected permanently.
    Retrying(String),
    /// Persist this state (the runner produces it; the controller saves it).
    SaveState(Box<PersistedState>),
    /// Calibration ended; individual fits may have been rejected. Terminal.
    Finished,
}

/// UI-facing progress snapshot (mirrored into `ControlStatus.calib`). Fields
/// only change on real transitions (never per-sample counters), so the
/// status diff / telemetry Decision cadence stays event-driven.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CalibProgress {
    /// `"settle"`, a device step, `"done"`, or `"aborted"`.
    pub phase: String,
    /// Completed steps within the phase.
    pub step: usize,
    /// Total steps within the phase.
    pub total: usize,
    /// The user must start a GPU-heavy load for progress to continue.
    pub needs_load: bool,
    /// Human-readable detail line for the wizard panel.
    pub note: String,
}

/// Retained until the next calibration, independently of live progress.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CalibOutcome {
    pub changes: Vec<CalibGainChange>,
    pub errors: Vec<String>,
    pub notes: Vec<String>,
    pub retained_cpu: Option<crate::control::device_loop::Gains>,
    pub retained_gpu: Option<crate::control::device_loop::Gains>,
    /// Fits are staged until the runner supplies SaveState.
    pub applied: bool,
    pub saved: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CalibGainChange {
    pub device: CalibDevice,
    pub before: crate::control::device_loop::Gains,
    pub after: crate::control::device_loop::Gains,
}

impl CalibOutcome {
    pub fn title(&self) -> &'static str {
        if !self.applied || self.changes.is_empty() {
            "failed"
        } else if !self.saved {
            "error: gains not saved"
        } else if self.changes.len() < 2 || !self.errors.is_empty() {
            "partial success"
        } else {
            "success"
        }
    }

    pub fn details(&self) -> String {
        let mut lines = Vec::new();
        for device in [CalibDevice::Cpu, CalibDevice::Gpu] {
            let (name, unit) = match device {
                CalibDevice::Cpu => ("CPU", "W/C"),
                CalibDevice::Gpu => ("GPU", "MHz/C"),
            };
            match self.changes.iter().find(|change| change.device == device).filter(|_| self.applied) {
                Some(change) => lines.push(format!(
                    "{name}: Kc {:.4} -> {:.4} {unit}; Ti {:.2} -> {:.2}s",
                    change.before.kc, change.after.kc, change.before.ti_s, change.after.ti_s
                )),
                None => {
                    let retained = match device {
                        CalibDevice::Cpu => self.retained_cpu,
                        CalibDevice::Gpu => self.retained_gpu,
                    };
                    lines.push(match retained {
                        Some(gains) => format!("{name}: unchanged; Kc {:.4} {unit}, Ti {:.2}s retained", gains.kc, gains.ti_s),
                        None => format!("{name}: unchanged (existing gains retained; calibration context unavailable)"),
                    });
                }
            }
        }
        if self.applied && !self.changes.is_empty() {
            lines.push(if self.saved { "Calibration gains updated and saved." } else { "Calibration gains updated in memory, but NOT saved." }.into());
        } else {
            lines.push("No gain changes applied.".into());
        }
        if self.applied { lines.extend(self.notes.iter().cloned()); }
        lines.extend(self.errors.iter().cloned());
        lines.join("\n")
    }
}

/// Revision-4 calibration runner: one shared settle followed by native CPU
/// watts and GPU clock steps.
pub struct PerDeviceCalibRunner {
    step: PerDeviceStepTest,
    cpu_gains: std::collections::BTreeMap<String, crate::control::device_loop::Gains>,
    gpu_gains: std::collections::BTreeMap<String, crate::control::device_loop::Gains>,
    started: bool,
    finished: bool,
    note: String,
    retry_note: Option<String>,
}

impl PerDeviceCalibRunner {
    pub fn new(
        cpu_gains: std::collections::BTreeMap<String, crate::control::device_loop::Gains>,
        gpu_gains: std::collections::BTreeMap<String, crate::control::device_loop::Gains>,
    ) -> Self {
        Self {
            step: PerDeviceStepTest::new(),
            cpu_gains,
            gpu_gains,
            started: false,
            finished: false,
            note: String::new(),
            retry_note: None,
        }
    }

    pub fn start(&mut self) -> Vec<RunnerEffect> {
        self.started = true;
        self.note = "holding both caps; waiting for groups and fans to settle".into();
        Vec::new()
    }

    pub fn on_sample(
        &mut self,
        s: &Sample,
        ctx: &PerDeviceCalibContext,
    ) -> Vec<RunnerEffect> {
        if !self.started || self.finished { return Vec::new(); }
        let mut effects = self.step.on_sample(s, ctx);
        let key = self.step.key().map(str::to_owned);
        for effect in &effects {
            match effect {
                RunnerEffect::FittedDevice { device: CalibDevice::Cpu, gains } => {
                    if let Some(key) = &key { self.cpu_gains.insert(key.clone(), *gains); }
                }
                RunnerEffect::FittedDevice { device: CalibDevice::Gpu, gains } => {
                    if let Some(key) = &key { self.gpu_gains.insert(key.clone(), *gains); }
                }
                RunnerEffect::Retrying(reason) => self.retry_note = Some(reason.clone()),
                RunnerEffect::Noted(_) => self.retry_note = None,
                _ => {}
            }
        }
        if effects.iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { .. })) {
            self.retry_note = None;
        }
        self.note = if self.step.in_settle() {
            "holding both caps; waiting for groups and fans to settle".into()
        } else {
            match self.step.phase() {
                CalibDevice::Cpu => "CPU watts step".into(),
                CalibDevice::Gpu => "GPU clock step".into(),
            }
        };
        if let Some(retry) = &self.retry_note {
            self.note = format!("{retry}\n{}", self.note);
        }
        if self.step.done() {
            self.finished = true;
            self.note = "calibration complete".into();
            effects.push(RunnerEffect::SaveState(Box::new(PersistedState {
                calibrated_at: Some(unix_secs_string()),
                cpu_gains: self.cpu_gains.clone(),
                gpu_gains: self.gpu_gains.clone(),
                ..PersistedState::default()
            })));
            effects.push(RunnerEffect::Finished);
        }
        effects
    }

    pub fn abort(&mut self) -> Vec<RunnerEffect> {
        self.abort_with_reason("calibration aborted".into())
    }

    /// Thermal-watchdog terminal path. The controller applies this batch
    /// before its global stock release so the established held pair is
    /// restored first and the terminal reason remains observable.
    pub fn abort_with_reason(&mut self, reason: String) -> Vec<RunnerEffect> {
        if self.finished { return Vec::new(); }
        self.finished = true;
        self.note = "aborted".into();
        self.step.abort(reason)
    }

    pub fn finishing_response(&self, now_s: f64) -> bool { self.step.finishing_response(now_s) }

    pub fn diagnostics(&self) -> Option<&crate::calib::step::CalibrationDiagnostics> {
        self.step.diagnostics()
    }

    pub fn progress(&self) -> CalibProgress {
        if self.finished {
            return CalibProgress { phase: "done".into(), step: 3, total: 3, note: self.note.clone(), ..CalibProgress::default() };
        }
        let (phase, step) = if !self.started || self.step.in_settle() {
            ("settle", 0)
        } else {
            match self.step.phase() {
                CalibDevice::Cpu => ("cpu_step", 1),
                CalibDevice::Gpu => ("gpu_step", 2),
            }
        };
        CalibProgress {
            phase: phase.into(),
            step,
            total: 3,
            note: self.note.clone(),
            ..CalibProgress::default()
        }
    }
}

/// Wall-clock unix seconds as a string (for `calibrated_at`).
fn unix_secs_string() -> String {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs())
        .to_string()
}

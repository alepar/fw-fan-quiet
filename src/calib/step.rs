//! Revision-4 per-device calibration. [`PerDeviceStepTest`] establishes and
//! holds a CPU-watt/GPU-clock pair, waits for both EC groups and the fans to
//! settle, then records independent CPU and GPU FOPDT steps. Every request
//! is returned as a [`RunnerEffect`] for the controller's normal actuator
//! paths; hot guards restore the pair before any step write can win.
//! The fitted gains are persisted by the revision-4 runner.

use std::collections::VecDeque;

use crate::calib::fopdt::{
    MIN_EC_RESPONSE_C, derive_device_gains, fit_fopdt,
};
use crate::calib::runner::{BURNER_THREADS, RunnerEffect};
use crate::control::device_loop::{Mhz, W, default_gains};
use crate::types::Sample;

/// EC max above this aborts the step and restores the floor (design §3.3).
const EC_MAX_ABORT_C: f64 = 95.0;

pub const DEVICE_SETTLE_S: usize = 600;
pub const DEVICE_FIT_WINDOW_S: usize = 360;
/// The sampler is 1 Hz. Allow one delayed tick, but never let a sparse trace
/// masquerade as continuous full-window response coverage.
const DEVICE_RESPONSE_MAX_GAP_S: f64 = 2.0;
pub const DEVICE_STEP_W: f64 = 15.0;
pub const DEVICE_STEP_MHZ: u32 = 500;
const DEVICE_GROUP_WINDOW_S: usize = 60;
const DEVICE_FAN_WINDOW_S: usize = 20;
const DEVICE_CPU_GROUP_SPAN_C: f64 = 1.0;
const DEVICE_GPU_GROUP_SPAN_C: f64 = 2.0;
const DEVICE_FAN_SPAN_RPM: f64 = 150.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibDevice {
    Cpu,
    Gpu,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct PerDeviceCalibContext {
    pub cpu_cap_w: Option<f64>,
    pub cpu_cap_verified: bool,
    pub cpu_cap_checked_at_s: Option<f64>,
    pub cpu_cap_readback: Option<crate::actuators::WriteVerdict>,
    pub cpu_cap_reset_reason: Option<String>,
    pub cpu_cap_completed_at_s: Option<f64>,
    pub gpu_cap_mhz: Option<u32>,
    pub gpu_cap_verified: bool,
    pub gpu_cap_completed_at_s: Option<f64>,
    /// Auto calibration holds its already-applied pair. Monitor has no
    /// applied pair, so the runner establishes the configured floors.
    pub use_current_caps: bool,
    pub cpu_floor_w: f64,
    pub gpu_floor_mhz: u32,
    pub cpu_max_w: f64,
    pub gpu_max_mhz: u32,
    pub cpu_group_c: Option<f64>,
    pub gpu_group_c: Option<f64>,
    pub fanctrl_active: bool,
    pub ec_mismatch: bool,
    pub argmax_controllable: bool,
    pub cpu_hot_c: f64,
    pub gpu_hot_c: f64,
    pub strategy: Option<String>,
    pub ma_interval: Option<u32>,
}

impl Default for PerDeviceCalibContext {
    fn default() -> Self {
        Self {
            cpu_cap_w: None,
            cpu_cap_verified: false,
            cpu_cap_checked_at_s: None,
            cpu_cap_readback: None,
            cpu_cap_reset_reason: None,
            cpu_cap_completed_at_s: None,
            gpu_cap_mhz: None,
            gpu_cap_verified: false,
            gpu_cap_completed_at_s: None,
            use_current_caps: false,
            cpu_floor_w: 8.0,
            gpu_floor_mhz: 1000,
            cpu_max_w: 80.0,
            gpu_max_mhz: 3090,
            cpu_group_c: None,
            gpu_group_c: None,
            fanctrl_active: false,
            ec_mismatch: false,
            argmax_controllable: false,
            cpu_hot_c: 90.0,
            gpu_hot_c: 88.0,
            strategy: None,
            ma_interval: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DeviceSub {
    Settle,
    Cpu,
    RecoverGpu,
    Gpu,
    Done,
}

#[derive(Debug, Clone, Copy, Default)]
struct GateDuration {
    satisfied: bool,
    observed: bool,
    since_s: f64,
    last_s: f64,
}

impl GateDuration {
    fn observe(&mut self, now_s: f64, satisfied: bool) {
        if !self.observed || self.satisfied != satisfied {
            self.since_s = now_s;
        }
        self.observed = true;
        self.satisfied = satisfied;
        self.last_s = now_s;
    }

    fn describe(&self, name: &str) -> String {
        let status = if self.satisfied { "satisfied" } else { "failed" };
        let duration = if self.observed { (self.last_s - self.since_s).max(0.0) } else { 0.0 };
        format!("{name}={status} for {duration:.3}s")
    }
}

/// Snapshot of the exact inputs and windows used by one calibration tick.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct CalibrationDiagnostics {
    pub phase: &'static str,
    pub next_phase: &'static str,
    pub run_elapsed_s: f64,
    pub phase_elapsed_s: f64,
    pub timeout_s: f64,
    pub baseline_cpu_w: f64,
    pub baseline_gpu_mhz: u32,
    pub phase_commanded_at_s: Option<f64>,
    pub context: PerDeviceCalibContext,
    /// None outside settling/recovery; old gate state is never reported as current.
    pub gates: Option<Vec<CalibrationGate>>,
    pub windows: Option<Vec<CalibrationWindow>>,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct CalibrationGate {
    pub name: &'static str,
    pub satisfied: bool,
    pub observed: bool,
    pub last_observed_at_s: Option<f64>,
    /// Continuous duration of the current satisfied/failed state.
    pub duration_s: f64,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct CalibrationWindow {
    pub name: &'static str,
    pub samples: usize,
    pub coverage_s: f64,
    pub required_s: f64,
    pub span: Option<f64>,
    pub limit: f64,
    pub unit: &'static str,
}

impl CalibrationWindow {
    fn snapshot(name: &'static str, values: &VecDeque<(f64, f64)>, required_s: f64, limit: f64, unit: &'static str) -> Self {
        let coverage_s = values.front().zip(values.back()).map_or(0.0, |(first, last)| (last.0 - first.0).max(0.0));
        let span = (!values.is_empty()).then(|| {
            let low = values.iter().map(|(_, v)| *v).fold(f64::INFINITY, f64::min);
            let high = values.iter().map(|(_, v)| *v).fold(f64::NEG_INFINITY, f64::max);
            high - low
        });
        Self { name, samples: values.len(), coverage_s, required_s, span, limit, unit }
    }
}

pub struct PerDeviceStepTest {
    sub: DeviceSub,
    diagnostics: Option<CalibrationDiagnostics>,
    initialized: bool,
    run_started_at_s: Option<f64>,
    settle_started_at_s: Option<f64>,
    cpu_retries: u8,
    gpu_retries: u8,
    phase_commanded_at_s: Option<f64>,
    last_sample_at_s: Option<f64>,
    cpu_hot_streak: u8,
    baseline_cpu_w: f64,
    baseline_gpu_mhz: u32,
    step_cpu_w: f64,
    step_gpu_mhz: u32,
    applied_step_cpu_w: Option<f64>,
    applied_step_gpu_mhz: Option<u32>,
    recovery_started_at_s: Option<f64>,
    response_last_at_s: Option<f64>,
    key: Option<String>,
    interval: Option<u32>,
    cpu_group: VecDeque<(f64, f64)>,
    gpu_group: VecDeque<(f64, f64)>,
    fan: VecDeque<(f64, f64)>,
    settle_gates: [GateDuration; 7],
    primary: Vec<(f64, f64)>,
}

impl PerDeviceStepTest {
    pub fn new() -> Self {
        Self {
            sub: DeviceSub::Settle,
            diagnostics: None,
            initialized: false,
            run_started_at_s: None,
            settle_started_at_s: None,
            cpu_retries: 0,
            gpu_retries: 0,
            phase_commanded_at_s: None,
            last_sample_at_s: None,
            cpu_hot_streak: 0,
            baseline_cpu_w: 0.0,
            baseline_gpu_mhz: 0,
            step_cpu_w: 0.0,
            step_gpu_mhz: 0,
            applied_step_cpu_w: None,
            applied_step_gpu_mhz: None,
            recovery_started_at_s: None,
            response_last_at_s: None,
            key: None,
            interval: None,
            cpu_group: VecDeque::new(),
            gpu_group: VecDeque::new(),
            fan: VecDeque::new(),
            settle_gates: std::array::from_fn(|_| GateDuration::default()),
            primary: Vec::new(),
        }
    }

    pub fn done(&self) -> bool {
        self.sub == DeviceSub::Done
    }

    pub fn phase(&self) -> CalibDevice {
        if matches!(self.sub, DeviceSub::RecoverGpu | DeviceSub::Gpu) {
            CalibDevice::Gpu
        } else {
            CalibDevice::Cpu
        }
    }

    pub fn key(&self) -> Option<&str> {
        self.key.as_deref()
    }

    pub fn in_settle(&self) -> bool {
        self.sub == DeviceSub::Settle
    }

    pub fn finishing_response(&self, now_s: f64) -> bool {
        matches!(self.sub, DeviceSub::Cpu | DeviceSub::Gpu)
            && self.phase_commanded_at_s.is_some_and(|at| now_s - at >= DEVICE_FIT_WINDOW_S as f64)
    }

    pub fn diagnostics(&self) -> Option<&CalibrationDiagnostics> {
        self.diagnostics.as_ref()
    }

    fn phase_name(&self) -> &'static str {
        match self.sub {
            DeviceSub::Settle => "settle",
            DeviceSub::Cpu => "cpu_step",
            DeviceSub::RecoverGpu => "gpu_recovery",
            DeviceSub::Gpu => "gpu_step",
            DeviceSub::Done => "done",
        }
    }

    pub fn on_sample(&mut self, s: &Sample, ctx: &PerDeviceCalibContext) -> Vec<RunnerEffect> {
        let phase = self.phase_name();
        let settling = matches!(self.sub, DeviceSub::Settle | DeviceSub::RecoverGpu);
        let start = match self.sub {
            DeviceSub::Settle => self.settle_started_at_s,
            DeviceSub::RecoverGpu => self.recovery_started_at_s,
            _ => self.phase_commanded_at_s,
        }.unwrap_or(s.t_mono);
        let effects = self.on_sample_inner(s, ctx);
        let gates = settling.then(|| {
            ["caps_held", "fanctrl_active", "ec_match", "argmax_controllable", "cpu_group_flat", "gpu_group_flat", "fans_flat"]
                .into_iter().zip(self.settle_gates.iter()).map(|(name, gate)| CalibrationGate {
                    name, satisfied: gate.satisfied, observed: gate.observed,
                    last_observed_at_s: gate.observed.then_some(gate.last_s),
                    duration_s: if gate.observed { (gate.last_s - gate.since_s).max(0.0) } else { 0.0 },
                }).collect()
        });
        let windows = settling.then(|| vec![
            CalibrationWindow::snapshot("cpu_group", &self.cpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_CPU_GROUP_SPAN_C, "celsius"),
            CalibrationWindow::snapshot("gpu_group", &self.gpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_GPU_GROUP_SPAN_C, "celsius"),
            CalibrationWindow::snapshot("fans", &self.fan, DEVICE_FAN_WINDOW_S as f64, DEVICE_FAN_SPAN_RPM, "rpm"),
        ]);
        self.diagnostics = Some(CalibrationDiagnostics {
            phase, next_phase: self.phase_name(),
            run_elapsed_s: (s.t_mono - self.run_started_at_s.unwrap_or(s.t_mono)).max(0.0),
            phase_elapsed_s: (s.t_mono - start).max(0.0),
            timeout_s: if settling { DEVICE_SETTLE_S as f64 } else { DEVICE_FIT_WINDOW_S as f64 },
            baseline_cpu_w: self.baseline_cpu_w, baseline_gpu_mhz: self.baseline_gpu_mhz,
            phase_commanded_at_s: self.phase_commanded_at_s,
            context: ctx.clone(), gates, windows,
        });
        effects
    }

    fn on_sample_inner(
        &mut self,
        s: &Sample,
        ctx: &PerDeviceCalibContext,
    ) -> Vec<RunnerEffect> {
        let now_s = s.t_mono;
        if !now_s.is_finite() || self.last_sample_at_s.is_some_and(|last| now_s < last) {
            return self.abort("calibration sample clock moved backwards or was invalid".into());
        }
        self.last_sample_at_s = Some(now_s);
        let first_sample = !self.initialized;
        if first_sample {
            self.initialized = true;
            self.run_started_at_s = Some(now_s);
            self.settle_started_at_s = Some(now_s);
            self.baseline_cpu_w = if ctx.use_current_caps {
                ctx.cpu_cap_w.unwrap_or(ctx.cpu_floor_w)
            } else {
                ctx.cpu_floor_w
            };
            self.baseline_gpu_mhz = if ctx.use_current_caps {
                ctx.gpu_cap_mhz.unwrap_or(ctx.gpu_floor_mhz)
            } else {
                ctx.gpu_floor_mhz
            };
            if let Some((key, interval)) = context_key(ctx) {
                self.key = Some(key);
                self.interval = Some(interval);
            }
        }
        if let Some(reason) = self.guard_reason(s, ctx) {
            return self.abort(reason);
        }
        if let Some(reason) = self.key_change_reason(ctx) {
            return self.abort(reason);
        }
        if !first_sample && let Some(reason) = &ctx.cpu_cap_reset_reason {
            return self.retry_after_cpu_cap_reset(now_s, reason);
        }
        if first_sample {
            for (gate, satisfied) in self.settle_gates.iter_mut().zip([
                false,
                ctx.fanctrl_active,
                !ctx.ec_mismatch,
                ctx.argmax_controllable,
                false,
                false,
                false,
            ]) {
                gate.observe(now_s, satisfied);
            }
            return vec![
                RunnerEffect::StartBurner(BURNER_THREADS),
                RunnerEffect::SetCpuMaxWatts(self.baseline_cpu_w),
                RunnerEffect::SetGpuMaxClock(self.baseline_gpu_mhz),
            ];
        }
        match self.sub {
            DeviceSub::Settle => self.on_settle(now_s, s, ctx),
            DeviceSub::Cpu => self.on_response(now_s, ctx, CalibDevice::Cpu),
            DeviceSub::RecoverGpu => self.on_gpu_recovery(now_s, s, ctx),
            DeviceSub::Gpu => self.on_response(now_s, ctx, CalibDevice::Gpu),
            DeviceSub::Done => Vec::new(),
        }
    }

    fn retry_after_cpu_cap_reset(&mut self, now_s: f64, reason: &str) -> Vec<RunnerEffect> {
        let device = self.phase();
        let retries = match device { CalibDevice::Cpu => &mut self.cpu_retries, CalibDevice::Gpu => &mut self.gpu_retries };
        if *retries >= 2 {
            return self.abort(format!("{} calibration failed after 2 retries: {reason}", device_name(device)));
        }
        *retries += 1;
        let note = format!("{} retry {}/2: {reason}; discarded response, settling again", device_name(device), *retries);
        self.primary.clear();
        self.response_last_at_s = None;
        self.phase_commanded_at_s = None;
        self.applied_step_cpu_w = None;
        self.applied_step_gpu_mhz = None;
        self.cpu_group.clear();
        self.gpu_group.clear();
        self.fan.clear();
        self.settle_gates = std::array::from_fn(|_| GateDuration::default());
        match device {
            CalibDevice::Cpu => {
                self.sub = DeviceSub::Settle;
                self.settle_started_at_s = Some(now_s);
            }
            CalibDevice::Gpu => {
                self.sub = DeviceSub::RecoverGpu;
                self.recovery_started_at_s = Some(now_s);
            }
        }
        vec![
            RunnerEffect::SetCpuMaxWatts(self.baseline_cpu_w),
            RunnerEffect::SetGpuMaxClock(self.baseline_gpu_mhz),
            RunnerEffect::Retrying(note),
        ]
    }

    fn key_change_reason(&mut self, ctx: &PerDeviceCalibContext) -> Option<String> {
        let current = context_key(ctx);
        match (&self.key, self.interval, current) {
            (None, None, Some((key, interval))) => {
                self.key = Some(key);
                self.interval = Some(interval);
                None
            }
            (Some(key), Some(interval), Some((current_key, current_interval)))
                if *key == current_key && interval == current_interval =>
            {
                None
            }
            (Some(key), Some(interval), _) => Some(format!(
                "strategy or moving-average interval changed during calibration (frozen {key}/{interval})"
            )),
            _ => None,
        }
    }

    fn guard_reason(&mut self, s: &Sample, ctx: &PerDeviceCalibContext) -> Option<String> {
        self.cpu_hot_streak = if s.cpu_temp_valid && s.cpu_temp_c >= ctx.cpu_hot_c {
            self.cpu_hot_streak.saturating_add(1)
        } else {
            0
        };
        if s.ec_valid && s.ec.as_ref().is_some_and(|ec| f64::from(ec.max_c) >= EC_MAX_ABORT_C) {
            Some(format!("EC max reached {EC_MAX_ABORT_C:.0}C"))
        } else if s.gpu_temp_valid && s.gpu_temp_c >= ctx.gpu_hot_c {
            Some(format!("GPU die reached {:.1}C", ctx.gpu_hot_c))
        } else if self.cpu_hot_streak >= 3 {
            Some(format!("CPU Tctl reached {:.1}C for 3 samples", ctx.cpu_hot_c))
        } else {
            None
        }
    }

    fn on_settle(
        &mut self,
        now_s: f64,
        s: &Sample,
        ctx: &PerDeviceCalibContext,
    ) -> Vec<RunnerEffect> {
        if self.phase_commanded_at_s.is_none()
            && ctx.cpu_cap_verified
            && ctx.gpu_cap_verified
            && let (Some(cpu), Some(gpu), Some(cpu_at), Some(gpu_at)) = (
                ctx.cpu_cap_w,
                ctx.gpu_cap_mhz,
                ctx.cpu_cap_completed_at_s,
                ctx.gpu_cap_completed_at_s,
            )
            && cpu_at >= self.run_started_at_s.unwrap_or(now_s)
            && gpu_at >= self.run_started_at_s.unwrap_or(now_s)
        {
            self.baseline_cpu_w = cpu;
            self.baseline_gpu_mhz = gpu;
            self.step_cpu_w = (cpu + DEVICE_STEP_W).min(ctx.cpu_max_w);
            self.step_gpu_mhz = gpu.saturating_add(DEVICE_STEP_MHZ).min(ctx.gpu_max_mhz);
            self.phase_commanded_at_s = Some(cpu_at.max(gpu_at));
        }
        let caps_held = self.phase_commanded_at_s.is_some()
            && ctx.cpu_cap_verified
            && ctx.gpu_cap_verified
            && ctx.cpu_cap_w.is_some_and(|w| (w - self.baseline_cpu_w).abs() <= 0.25)
            && ctx.gpu_cap_mhz == Some(self.baseline_gpu_mhz);
        if caps_held && ctx.fanctrl_active && !ctx.ec_mismatch && ctx.argmax_controllable {
            push_timed_optional(&mut self.cpu_group, ctx.cpu_group_c, now_s, DEVICE_GROUP_WINDOW_S as f64);
            push_timed_optional(&mut self.gpu_group, ctx.gpu_group_c, now_s, DEVICE_GROUP_WINDOW_S as f64);
            push_timed_optional(
                &mut self.fan,
                s.fan_valid.then(|| s.max_fan_rpm()),
                now_s,
                DEVICE_FAN_WINDOW_S as f64,
            );
        } else {
            self.cpu_group.clear();
            self.gpu_group.clear();
            self.fan.clear();
        }

        let cpu_flat = timed_span_within(&self.cpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_CPU_GROUP_SPAN_C);
        let gpu_flat = timed_span_within(&self.gpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_GPU_GROUP_SPAN_C);
        let fans_flat = timed_span_within(&self.fan, DEVICE_FAN_WINDOW_S as f64, DEVICE_FAN_SPAN_RPM);
        for (gate, satisfied) in self.settle_gates.iter_mut().zip([
            caps_held,
            ctx.fanctrl_active,
            !ctx.ec_mismatch,
            ctx.argmax_controllable,
            cpu_flat,
            gpu_flat,
            fans_flat,
        ]) {
            gate.observe(now_s, satisfied);
        }
        let settled = self.settle_gates.iter().all(|gate| gate.satisfied);
        if settled {
            let Some(_key) = self.key.as_deref() else {
                return self.abort("settle completed without a strategy".into());
            };
            let Some(_interval) = self.interval else {
                return self.abort("settle completed without a moving-average interval".into());
            };
            self.sub = DeviceSub::Cpu;
            self.phase_commanded_at_s = Some(now_s);
            self.applied_step_cpu_w = None;
            self.response_last_at_s = None;
            self.primary.clear();
            return vec![RunnerEffect::SetCpuMaxWatts(self.step_cpu_w)];
        }
        if now_s - self.settle_started_at_s.unwrap_or(now_s) >= DEVICE_SETTLE_S as f64 {
            return self.abort(self.settle_timeout_reason());
        }
        Vec::new()
    }

    fn settle_timeout_reason(&self) -> String {
        let names = [
            "caps held",
            "fanctrl active",
            "EC match",
            "argmax controllable",
            "CPU group flat",
            "GPU group flat",
            "fans flat",
        ];
        let details = names
            .iter()
            .zip(&self.settle_gates)
            .map(|(name, gate)| gate.describe(name))
            .collect::<Vec<_>>()
            .join("; ");
        format!("settle timed out after {}.000s: {details}", DEVICE_SETTLE_S)
    }

    fn on_response(
        &mut self,
        now_s: f64,
        ctx: &PerDeviceCalibContext,
        device: CalibDevice,
    ) -> Vec<RunnerEffect> {
        let commanded_at = self.phase_commanded_at_s.unwrap_or(now_s);
        let new_command_applied = match device {
            CalibDevice::Cpu => {
                if self.applied_step_cpu_w.is_none()
                    && ctx.cpu_cap_verified
                    && ctx.cpu_cap_completed_at_s.is_some_and(|at| at >= commanded_at)
                {
                    self.applied_step_cpu_w = ctx.cpu_cap_w;
                    self.phase_commanded_at_s = ctx.cpu_cap_completed_at_s;
                }
                self.applied_step_cpu_w.is_some()
            }
            CalibDevice::Gpu => {
                if self.applied_step_gpu_mhz.is_none()
                    && ctx.gpu_cap_verified
                    && ctx.gpu_cap_completed_at_s.is_some_and(|at| at >= commanded_at)
                {
                    self.applied_step_gpu_mhz = ctx.gpu_cap_mhz;
                    self.phase_commanded_at_s = ctx.gpu_cap_completed_at_s;
                }
                self.applied_step_gpu_mhz.is_some()
            }
        };
        if !new_command_applied {
            if now_s - commanded_at >= DEVICE_FIT_WINDOW_S as f64 {
                return self.abort(format!(
                    "{} step rejected: no verified applied pair for 360.000s",
                    device_name(device)
                ));
            }
            return Vec::new();
        }
        let pair_held = ctx.cpu_cap_verified
            && ctx.gpu_cap_verified
            && ctx.cpu_cap_w.is_some_and(|w| {
                let expected = if device == CalibDevice::Cpu {
                    self.applied_step_cpu_w.unwrap_or(self.step_cpu_w)
                } else {
                    self.baseline_cpu_w
                };
                (w - expected).abs() <= 0.25
            })
            && ctx.gpu_cap_mhz == Some(if device == CalibDevice::Gpu {
                self.applied_step_gpu_mhz.unwrap_or(self.step_gpu_mhz)
            } else {
                self.baseline_gpu_mhz
            });
        if !ctx.cpu_cap_verified && ctx.cpu_cap_readback.is_some() {
            return self.abort(format!("{} step rejected: CPU cap read-back unverified ({:?})", device_name(device), ctx.cpu_cap_readback));
        }
        if !pair_held {
            return self.abort(format!(
                "{} step rejected: verified applied pair changed during response",
                device_name(device)
            ));
        }
        let primary = match device {
            CalibDevice::Cpu => ctx.cpu_group_c,
            CalibDevice::Gpu => ctx.gpu_group_c,
        };
        let t = now_s - self.phase_commanded_at_s.unwrap_or(now_s);
        let Some(primary) = primary.filter(|value| value.is_finite()) else {
            return self.finish_device_response(now_s, device, vec![RunnerEffect::Noted(format!(
                "{} fit rejected: primary group coverage missing or interrupted",
                device_name(device)
            ))]);
        };
        let coverage_gap = self.response_last_at_s.map_or(t, |last| now_s - last);
        if coverage_gap > DEVICE_RESPONSE_MAX_GAP_S {
            return self.finish_device_response(now_s, device, vec![RunnerEffect::Noted(format!(
                "{} fit rejected: group coverage interrupted for {coverage_gap:.3}s",
                device_name(device)
            ))]);
        }
        self.response_last_at_s = Some(now_s);
        self.primary.push((t, primary));
        if t < DEVICE_FIT_WINDOW_S as f64 { return Vec::new(); }

        let effects = self.conclude_device(device, self.interval.unwrap_or(60));
        self.finish_device_response(now_s, device, effects)
    }

    fn finish_device_response(
        &mut self,
        now_s: f64,
        device: CalibDevice,
        mut effects: Vec<RunnerEffect>,
    ) -> Vec<RunnerEffect> {
        self.primary.clear();
        self.response_last_at_s = None;
        match device {
            CalibDevice::Cpu => {
                // Restore the complete held pair after the CPU step. The
                // CPU group is still carrying the step response, so wait
                // for the same physical flatness conditions before the GPU
                // step starts from a settled thermal state.
                self.sub = DeviceSub::RecoverGpu;
                self.recovery_started_at_s = Some(now_s);
                self.phase_commanded_at_s = None;
                self.applied_step_gpu_mhz = None;
                self.cpu_group.clear();
                self.gpu_group.clear();
                self.fan.clear();
                self.settle_gates = std::array::from_fn(|_| GateDuration::default());
                effects.insert(0, RunnerEffect::SetCpuMaxWatts(self.baseline_cpu_w));
                effects.insert(1, RunnerEffect::SetGpuMaxClock(self.baseline_gpu_mhz));
            }
            CalibDevice::Gpu => {
                self.sub = DeviceSub::Done;
                effects.insert(0, RunnerEffect::SetGpuMaxClock(self.baseline_gpu_mhz));
                effects.insert(0, RunnerEffect::SetCpuMaxWatts(self.baseline_cpu_w));
                effects.push(RunnerEffect::StopBurner);
            }
        }
        effects
    }

    fn on_gpu_recovery(
        &mut self,
        now_s: f64,
        s: &Sample,
        ctx: &PerDeviceCalibContext,
    ) -> Vec<RunnerEffect> {
        let recovery_started = self.recovery_started_at_s.unwrap_or(now_s);
        let caps_held = ctx.cpu_cap_verified
            && ctx.gpu_cap_verified
            && ctx.cpu_cap_completed_at_s.is_some_and(|at| at >= recovery_started)
            && ctx.gpu_cap_completed_at_s.is_some_and(|at| at >= recovery_started)
            && ctx.cpu_cap_w.is_some_and(|w| (w - self.baseline_cpu_w).abs() <= 0.25)
            && ctx.gpu_cap_mhz == Some(self.baseline_gpu_mhz);
        if caps_held && ctx.fanctrl_active && !ctx.ec_mismatch && ctx.argmax_controllable {
            push_timed_optional(&mut self.cpu_group, ctx.cpu_group_c, now_s, DEVICE_GROUP_WINDOW_S as f64);
            push_timed_optional(&mut self.gpu_group, ctx.gpu_group_c, now_s, DEVICE_GROUP_WINDOW_S as f64);
            push_timed_optional(&mut self.fan, s.fan_valid.then(|| s.max_fan_rpm()), now_s, DEVICE_FAN_WINDOW_S as f64);
        } else {
            self.cpu_group.clear();
            self.gpu_group.clear();
            self.fan.clear();
        }
        let cpu_flat = timed_span_within(&self.cpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_CPU_GROUP_SPAN_C);
        let gpu_flat = timed_span_within(&self.gpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_GPU_GROUP_SPAN_C);
        let fans_flat = timed_span_within(&self.fan, DEVICE_FAN_WINDOW_S as f64, DEVICE_FAN_SPAN_RPM);
        for (gate, satisfied) in self.settle_gates.iter_mut().zip([
            caps_held,
            ctx.fanctrl_active,
            !ctx.ec_mismatch,
            ctx.argmax_controllable,
            cpu_flat,
            gpu_flat,
            fans_flat,
        ]) {
            gate.observe(now_s, satisfied);
        }
        if self.settle_gates.iter().all(|gate| gate.satisfied) {
            self.sub = DeviceSub::Gpu;
            self.phase_commanded_at_s = Some(now_s);
            self.response_last_at_s = None;
            self.primary.clear();
            return vec![RunnerEffect::SetGpuMaxClock(self.step_gpu_mhz)];
        }
        if now_s - recovery_started >= DEVICE_SETTLE_S as f64 {
            return self.abort(format!("between-step recovery {}", self.settle_timeout_reason()));
        }
        Vec::new()
    }

    fn conclude_device(&self, device: CalibDevice, interval: u32) -> Vec<RunnerEffect> {
        let step = match device {
            CalibDevice::Cpu => self.applied_step_cpu_w.unwrap_or(self.baseline_cpu_w) - self.baseline_cpu_w,
            CalibDevice::Gpu => {
                f64::from(self.applied_step_gpu_mhz.unwrap_or(self.baseline_gpu_mhz))
                    - f64::from(self.baseline_gpu_mhz)
            }
        };
        let defaults = match device {
            CalibDevice::Cpu => default_gains::<W>(interval),
            CalibDevice::Gpu => default_gains::<Mhz>(interval),
        };
        let fit = fit_fopdt(&self.primary, step, MIN_EC_RESPONSE_C);
        let gains = fit.and_then(|fit| derive_device_gains(&fit, defaults));
        match gains {
            Ok(gains) => vec![RunnerEffect::FittedDevice { device, gains }],
            Err(reason) => vec![RunnerEffect::Noted(format!(
                "{} fit rejected: {reason}", device_name(device)
            ))],
        }
    }

    pub fn abort(&mut self, reason: String) -> Vec<RunnerEffect> {
        if !self.initialized {
            self.sub = DeviceSub::Done;
            return vec![RunnerEffect::StopBurner, RunnerEffect::Noted(reason)];
        }
        self.sub = DeviceSub::Done;
        vec![
            RunnerEffect::SetCpuMaxWatts(self.baseline_cpu_w),
            RunnerEffect::SetGpuMaxClock(self.baseline_gpu_mhz),
            RunnerEffect::StopBurner,
            RunnerEffect::Noted(reason),
        ]
    }
}

impl Default for PerDeviceStepTest {
    fn default() -> Self { Self::new() }
}

fn device_name(device: CalibDevice) -> &'static str {
    match device { CalibDevice::Cpu => "CPU", CalibDevice::Gpu => "GPU" }
}

fn context_key(ctx: &PerDeviceCalibContext) -> Option<(String, u32)> {
    let strategy = ctx.strategy.as_deref().filter(|strategy| !strategy.is_empty())?;
    let interval = ctx.ma_interval.filter(|interval| *interval > 0)?;
    Some((format!("{strategy}:{interval}"), interval))
}

fn push_timed_optional(
    window: &mut VecDeque<(f64, f64)>,
    value: Option<f64>,
    now_s: f64,
    horizon_s: f64,
) {
    match value.filter(|v| v.is_finite()) {
        Some(value) => {
            window.push_back((now_s, value));
            // Keep the most recent sample at or before the window boundary.
            // Dropping it on a jittered 1 Hz stream leaves only ~59/~19 s
            // of coverage, so even perfectly flat windows can never settle.
            // The retained boundary value participates in the span check.
            while window
                .get(1)
                .is_some_and(|(at, _)| now_s - at >= horizon_s)
            {
                window.pop_front();
            }
        }
        None => window.clear(),
    }
}

fn timed_span_within(window: &VecDeque<(f64, f64)>, horizon_s: f64, tolerance: f64) -> bool {
    let Some((first_at, _)) = window.front() else { return false; };
    let Some((last_at, _)) = window.back() else { return false; };
    if last_at - first_at < horizon_s { return false; }
    let lo = window.iter().map(|(_, value)| *value).fold(f64::INFINITY, f64::min);
    let hi = window.iter().map(|(_, value)| *value).fold(f64::NEG_INFINITY, f64::max);
    hi - lo <= tolerance
}



#[cfg(test)]
mod per_device_behavior_tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    static EC_COUNTER: AtomicU64 = AtomicU64::new(0);

    fn ec_at(temp_c: f64) -> crate::sensors::ec::EcReading {
        let n = EC_COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("bazerame-device-step-{}-{n}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("temp1_label"), "apu@4c\n").unwrap();
        std::fs::write(dir.join("temp1_input"), format!("{}\n", (temp_c * 1000.0) as i64)).unwrap();
        let reading = crate::sensors::ec::EcReading::read(&dir).expect("EC fixture");
        std::fs::remove_dir_all(dir).unwrap();
        reading
    }

    fn context() -> PerDeviceCalibContext {
        PerDeviceCalibContext {
            cpu_cap_w: Some(70.0),
            cpu_cap_verified: true,
            cpu_cap_checked_at_s: None,
            cpu_cap_readback: None,
            cpu_cap_reset_reason: None,
            cpu_cap_completed_at_s: Some(0.0),
            gpu_cap_mhz: Some(2_800),
            gpu_cap_verified: true,
            gpu_cap_completed_at_s: Some(0.0),
            use_current_caps: true,
            cpu_floor_w: 8.0,
            gpu_floor_mhz: 210,
            cpu_max_w: 80.0,
            gpu_max_mhz: 3_090,
            cpu_group_c: Some(50.0),
            gpu_group_c: Some(55.0),
            fanctrl_active: true,
            ec_mismatch: false,
            argmax_controllable: true,
            cpu_hot_c: 90.0,
            gpu_hot_c: 88.0,
            strategy: Some("quiet16".into()),
            ma_interval: Some(60),
        }
    }

    fn sample_at(t_mono: f64) -> Sample {
        Sample {
            t_mono,
            fan_valid: true,
            fan1_rpm: 2_400.0,
            ..Sample::default()
        }
    }

    #[test]
    fn calibration_settles_with_jitter_in_entry_and_recovery() {
        for recovery in [false, true] {
            for interval in [1.0, 1.001] {
                let mut step = PerDeviceStepTest::new();
                let ctx = context();
                step.on_sample(&sample_at(0.0), &ctx);
                if recovery {
                    step.sub = DeviceSub::RecoverGpu;
                    step.recovery_started_at_s = Some(0.0);
                    step.baseline_cpu_w = ctx.cpu_cap_w.unwrap();
                    step.baseline_gpu_mhz = ctx.gpu_cap_mhz.unwrap();
                }
                for second in 0..=60 {
                    step.on_sample(&sample_at(f64::from(second) * interval), &ctx);
                }
                let d = step.diagnostics().unwrap();
                assert_eq!(d.context, ctx);
                assert_eq!(d.phase, if recovery { "gpu_recovery" } else { "settle" });
                let gates = d.gates.as_ref().unwrap();
                assert_eq!(gates.len(), 7);
                assert!(gates[..4].iter().all(|gate| gate.observed && gate.satisfied));
                let windows = d.windows.as_ref().unwrap();
                assert_eq!(windows.iter().map(|w| w.limit).collect::<Vec<_>>(), vec![1.0, 2.0, 150.0]);
                assert!(windows.iter().all(|w| w.span == Some(0.0) && w.samples > 1));
                assert!(gates.iter().all(|gate| gate.satisfied));
                assert_eq!(d.next_phase, if recovery { "gpu_step" } else { "cpu_step" });
                assert!(windows.iter().all(|w| w.coverage_s >= w.required_s));
                assert!((windows[0].coverage_s - 60.0 * interval).abs() < 1e-9);
                assert!((windows[2].coverage_s - 20.0 * interval).abs() < 1e-9);
                step.on_sample(&sample_at(62.0), &ctx);
                let response = step.diagnostics().unwrap();
                assert!(response.gates.is_none() && response.windows.is_none(),
                    "response phase must not publish stale settling measurements");
            }
        }
    }

    #[test]
    fn settling_window_retains_only_one_boundary_sample_and_expires_old_outlier() {
        let mut window = VecDeque::new();
        for (at, value) in [(0.0, 10.0), (1.001, 0.0), (19.019, 0.0)] {
            push_timed_optional(&mut window, Some(value), at, 20.0);
        }
        assert!(!timed_span_within(&window, 20.0, 1.0));
        push_timed_optional(&mut window, Some(0.0), 20.020, 20.0);
        assert_eq!(window.front().unwrap().0, 0.0);
        assert!(!timed_span_within(&window, 20.0, 1.0), "boundary outlier still counts");
        push_timed_optional(&mut window, Some(0.0), 21.021, 20.0);
        assert_eq!(window.front().unwrap().0, 1.001);
        assert!(timed_span_within(&window, 20.0, 1.0));
        push_timed_optional(&mut window, None, 22.022, 20.0);
        assert!(window.is_empty(), "missing readings still discard coverage");
        push_timed_optional(&mut window, Some(0.0), 23.023, 20.0);
        assert!(!timed_span_within(&window, 20.0, 1.0));
    }

    #[test]
    fn per_device_settle_spans_apply_to_entry_and_recovery() {
        for recovery in [false, true] {
            for (cpu_span, gpu_span, fan_span, accepted) in [
                (1.0, 2.0, 150.0, true),
                (1.01, 2.0, 150.0, false),
                (1.0, 2.01, 150.0, false),
                (1.0, 2.0, 151.0, false),
            ] {
                let mut step = PerDeviceStepTest::new();
                let mut ctx = context();
                step.on_sample(&sample_at(0.0), &ctx);
                if recovery {
                    step.sub = DeviceSub::RecoverGpu;
                    step.recovery_started_at_s = Some(0.0);
                    step.baseline_cpu_w = ctx.cpu_cap_w.unwrap();
                    step.baseline_gpu_mhz = ctx.gpu_cap_mhz.unwrap();
                }
                let mut effects = Vec::new();
                for second in 0..=60 {
                    let high = second % 2 == 1;
                    ctx.cpu_group_c = Some(50.0 + if high { cpu_span } else { 0.0 });
                    ctx.gpu_group_c = Some(55.0 + if high { gpu_span } else { 0.0 });
                    let mut sample = sample_at(f64::from(second));
                    sample.fan1_rpm += if high { fan_span } else { 0.0 };
                    effects = step.on_sample(&sample, &ctx);
                }
                let advanced = effects.iter().any(|effect| matches!(effect,
                    RunnerEffect::SetCpuMaxWatts(_) | RunnerEffect::SetGpuMaxClock(_)));
                assert_eq!(advanced, accepted,
                    "recovery={recovery} CPU={cpu_span} GPU={gpu_span} fan={fan_span}");
            }
        }
    }

    fn enter_cpu_step(step: &mut PerDeviceStepTest, ctx: &PerDeviceCalibContext) {
        step.on_sample(&sample_at(0.0), ctx);
        step.on_sample(&sample_at(0.0), ctx);
        step.on_sample(&sample_at(40.0), ctx);
        let effects = step.on_sample(&sample_at(60.0), ctx);
        assert!(effects.iter().any(|effect| {
            matches!(effect, RunnerEffect::SetCpuMaxWatts(_))
        }));
    }

    fn cpu_step_context(
        settled: &PerDeviceCalibContext,
        applied_cpu_w: f64,
    ) -> PerDeviceCalibContext {
        PerDeviceCalibContext {
            cpu_cap_w: Some(applied_cpu_w),
            cpu_cap_completed_at_s: Some(60.0),
            ..settled.clone()
        }
    }

    #[test]
    fn shared_settle_steps_each_device_with_clamps_and_restores_the_pair() {
        let mut step = PerDeviceStepTest::new();
        let ctx = context();

        assert_eq!(step.on_sample(&sample_at(0.0), &ctx), vec![
            RunnerEffect::StartBurner(BURNER_THREADS),
            RunnerEffect::SetCpuMaxWatts(70.0),
            RunnerEffect::SetGpuMaxClock(2_800),
        ]);
        step.on_sample(&sample_at(0.0), &ctx);
        assert!(step.on_sample(&sample_at(39.999), &ctx).is_empty());
        assert!(step.on_sample(&sample_at(40.0), &ctx).is_empty());
        assert_eq!(step.on_sample(&sample_at(60.0), &ctx), vec![RunnerEffect::SetCpuMaxWatts(80.0)]);

        let response = |base: f64, gain: f64, tau: f64, theta: f64, delta: f64, t: f64| {
            if t <= theta { base } else { base + gain * delta * (1.0 - (-(t - theta) / tau).exp()) }
        };
        let mut cpu_end = Vec::new();
        let cpu_ctx = cpu_step_context(&ctx, 80.0);
        for elapsed in 1..=360 {
            let mut tick = cpu_ctx.clone();
            tick.cpu_group_c = Some(response(50.0, 0.8, 35.0, 20.0, 10.0, elapsed as f64));
            tick.gpu_group_c = Some(response(55.0, 0.05, 35.0, 20.0, 10.0, elapsed as f64));
            cpu_end = step.on_sample(&sample_at(60.0 + elapsed as f64), &tick);
        }
        assert!(cpu_end.contains(&RunnerEffect::SetCpuMaxWatts(70.0)));
        assert!(cpu_end.contains(&RunnerEffect::SetGpuMaxClock(2_800)));
        assert!(cpu_end.iter().any(|effect| matches!(effect,
            RunnerEffect::FittedDevice { device: CalibDevice::Cpu, .. })));

        let recovered = PerDeviceCalibContext {
            cpu_cap_w: Some(70.0),
            cpu_cap_completed_at_s: Some(420.0),
            gpu_cap_mhz: Some(2_800),
            gpu_cap_completed_at_s: Some(420.0),
            ..ctx.clone()
        };
        let mut recovery_end = Vec::new();
        for elapsed in 1..=61 {
            recovery_end = step.on_sample(&sample_at(420.0 + elapsed as f64), &recovered);
        }
        assert_eq!(recovery_end, vec![RunnerEffect::SetGpuMaxClock(3_090)]);

        let mut gpu_end = Vec::new();
        let gpu_ctx = PerDeviceCalibContext {
            cpu_cap_w: Some(70.0),
            cpu_cap_completed_at_s: Some(481.0),
            gpu_cap_mhz: Some(3_090),
            gpu_cap_completed_at_s: Some(481.0),
            ..ctx.clone()
        };
        for elapsed in 1..=360 {
            let mut tick = gpu_ctx.clone();
            tick.gpu_group_c = Some(response(55.0, 0.012, 32.564, 38.4515, 290.0, elapsed as f64));
            tick.cpu_group_c = Some(response(50.0, 0.001, 15.0, 60.0, 290.0, elapsed as f64));
            gpu_end = step.on_sample(&sample_at(481.0 + elapsed as f64), &tick);
        }
        assert!(gpu_end.contains(&RunnerEffect::SetGpuMaxClock(2_800)));
        assert!(gpu_end.contains(&RunnerEffect::StopBurner));
        assert!(gpu_end.iter().any(|effect| matches!(effect,
            RunnerEffect::FittedDevice { device: CalibDevice::Gpu, .. })));
        assert!(step.done());
        assert_eq!(step.key(), Some("quiet16:60"));
    }

    #[test]
    fn settle_timeout_names_every_failed_condition_and_the_600_second_duration() {
        let cases = [
            ("fanctrl active", { let mut c = context(); c.fanctrl_active = false; c }),
            ("EC match", { let mut c = context(); c.ec_mismatch = true; c }),
            ("argmax controllable", { let mut c = context(); c.argmax_controllable = false; c }),
            ("CPU group flat", { let mut c = context(); c.cpu_group_c = None; c }),
            ("GPU group flat", { let mut c = context(); c.gpu_group_c = None; c }),
            ("caps held", { let mut c = context(); c.cpu_cap_w = None; c }),
        ];
        for (expected, ctx) in cases {
            let mut step = PerDeviceStepTest::new();
            step.on_sample(&sample_at(0.0), &ctx);
            let end = step.on_sample(&sample_at(600.0), &ctx);
            let reason = end.iter().find_map(|effect| match effect {
                RunnerEffect::Noted(reason) => Some(reason.as_str()),
                _ => None,
            }).expect("timeout note");
            assert!(reason.contains(expected), "{expected}: {reason}");
            assert!(reason.contains("600.000s"), "{reason}");
        }
    }

    #[test]
    fn cpu_hot_requires_three_samples_and_guard_abort_restores_both_caps() {
        let mut step = PerDeviceStepTest::new();
        let ctx = context();
        let cool = Sample { cpu_temp_valid: true, cpu_temp_c: 60.0, ..Sample::default() };
        step.on_sample(&cool, &ctx);
        let hot = Sample { cpu_temp_valid: true, cpu_temp_c: 90.0, ..Sample::default() };
        assert!(step.on_sample(&hot, &ctx).is_empty());
        assert!(step.on_sample(&hot, &ctx).is_empty());
        let end = step.on_sample(&hot, &ctx);
        assert_eq!(&end[..3], &[
            RunnerEffect::SetCpuMaxWatts(70.0),
            RunnerEffect::SetGpuMaxClock(2_800),
            RunnerEffect::StopBurner,
        ]);
        assert!(matches!(end.last(), Some(RunnerEffect::Noted(reason)) if reason.contains("3 samples")));
    }

    #[test]
    fn gpu_hot_on_the_first_sample_wins_over_settle_writes_and_restores_both_caps() {
        let mut step = PerDeviceStepTest::new();
        let ctx = context();
        let hot = Sample { gpu_temp_valid: true, gpu_temp_c: 88.0, ..Sample::default() };
        let effects = step.on_sample(&hot, &ctx);
        assert_eq!(&effects[..3], &[
            RunnerEffect::SetCpuMaxWatts(70.0),
            RunnerEffect::SetGpuMaxClock(2_800),
            RunnerEffect::StopBurner,
        ]);
        assert!(!effects.contains(&RunnerEffect::SetCpuMaxWatts(80.0)));
        assert!(matches!(effects.last(), Some(RunnerEffect::Noted(reason)) if reason.contains("GPU die")));
    }

    #[test]
    fn monitor_without_applied_caps_uses_and_restores_configured_floors() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.cpu_cap_w = None;
        ctx.gpu_cap_mhz = None;
        ctx.cpu_floor_w = 15.0;
        ctx.gpu_floor_mhz = 1_000;
        let hot = Sample { gpu_temp_valid: true, gpu_temp_c: 88.0, ..Sample::default() };
        let effects = step.on_sample(&hot, &ctx);
        assert_eq!(&effects[..3], &[
            RunnerEffect::SetCpuMaxWatts(15.0),
            RunnerEffect::SetGpuMaxClock(1_000),
            RunnerEffect::StopBurner,
        ]);
        assert!(!effects.iter().any(|effect| matches!(effect,
            RunnerEffect::SetCpuMaxWatts(w) if *w > 15.0)));
        assert!(!effects.iter().any(|effect| matches!(effect,
            RunnerEffect::SetGpuMaxClock(mhz) if *mhz > 1_000)));
    }

    #[test]
    fn ec_at_exactly_95c_aborts_and_restores_before_any_step_write() {
        let mut step = PerDeviceStepTest::new();
        let hot = Sample { ec_valid: true, ec: Some(ec_at(95.0)), ..Sample::default() };
        let effects = step.on_sample(&hot, &context());
        assert_eq!(&effects[..3], &[
            RunnerEffect::SetCpuMaxWatts(70.0),
            RunnerEffect::SetGpuMaxClock(2_800),
            RunnerEffect::StopBurner,
        ]);
        assert!(matches!(effects.last(), Some(RunnerEffect::Noted(reason)) if reason.contains("EC max")));
    }

    #[test]
    fn noisy_fans_get_their_own_600_second_timeout_reason() {
        let mut step = PerDeviceStepTest::new();
        let ctx = context();
        let first = Sample { fan1_rpm: 2_300.0, ..sample_at(0.0) };
        step.on_sample(&first, &ctx);
        let mut end = Vec::new();
        for i in 0..=DEVICE_SETTLE_S {
            let sample = Sample {
                fan_valid: true,
                fan1_rpm: if i % 2 == 0 { 2_300.0 } else { 2_500.0 },
                t_mono: i as f64,
                ..Sample::default()
            };
            end = step.on_sample(&sample, &ctx);
        }
        assert!(end.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("fans flat") && reason.contains("600.000s"))));
    }

    #[test]
    fn cross_device_temperature_changes_do_not_reject_a_valid_cpu_fit() {
        for excursion in [-20.0, 20.0] {
            let mut step = PerDeviceStepTest::new();
            let mut ctx = context();
            ctx.cpu_cap_w = Some(20.0);
            enter_cpu_step(&mut step, &ctx);
            let response_ctx = cpu_step_context(&ctx, 35.0);
            let mut end = Vec::new();
            for elapsed in 1..=DEVICE_FIT_WINDOW_S {
                let mut tick = response_ctx.clone();
                tick.cpu_group_c = Some(50.0 + 12.0 * (1.0 - (-(elapsed as f64 - 20.0).max(0.0) / 35.0).exp()));
                tick.gpu_group_c = Some(55.0 + excursion);
                end = step.on_sample(&sample_at(60.0 + elapsed as f64), &tick);
            }
            assert!(end.iter().any(|effect| matches!(effect,
                RunnerEffect::FittedDevice { device: CalibDevice::Cpu, .. })), "{end:?}");
        }
    }

    #[test]
    fn cpu_retry_resettles_and_fits_only_a_fresh_full_response() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &ctx);
        let mut disturbed = cpu_step_context(&ctx, 35.0);
        disturbed.cpu_cap_reset_reason = Some("CPU limit changed to 40W".into());
        step.primary.push((1.0, 99.0));
        step.on_sample(&sample_at(100.0), &disturbed);
        let mut restored = ctx.clone();
        restored.cpu_cap_completed_at_s = Some(100.0);
        restored.gpu_cap_completed_at_s = Some(100.0);
        for second in 101..=161 {
            let effects = step.on_sample(&sample_at(f64::from(second)), &restored);
            if second < 161 { assert!(effects.is_empty(), "{effects:?}"); }
            else { assert_eq!(effects, vec![RunnerEffect::SetCpuMaxWatts(35.0)]); }
        }
        assert!(step.primary.is_empty());
        let mut response = cpu_step_context(&restored, 35.0);
        response.cpu_cap_completed_at_s = Some(161.0);
        for elapsed in 1..=360 {
            response.cpu_group_c = Some(50.0 + 12.0 * (1.0 - (-(f64::from(elapsed) - 20.0).max(0.0) / 35.0).exp()));
            let effects = step.on_sample(&sample_at(161.0 + f64::from(elapsed)), &response);
            let fitted = effects.iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { device: CalibDevice::Cpu, .. }));
            assert_eq!(fitted, elapsed == 360, "{effects:?}");
        }
    }

    #[test]
    fn cap_reset_discards_response_and_bounds_retries_for_each_device() {
        for device in [CalibDevice::Cpu, CalibDevice::Gpu] {
            let mut step = PerDeviceStepTest::new();
            let ctx = context();
            enter_cpu_step(&mut step, &ctx);
            for attempt in 0..3 {
                step.sub = if device == CalibDevice::Cpu { DeviceSub::Cpu } else { DeviceSub::Gpu };
                step.primary.push((1.0, 65.0));
                let mut disturbed = ctx.clone();
                disturbed.cpu_cap_w = Some(if device == CalibDevice::Cpu { 80.0 } else { 70.0 });
                disturbed.cpu_cap_reset_reason = Some("CPU cap reset: read 40W instead of 15W".into());
                let effects = step.on_sample(&sample_at(100.0 + f64::from(attempt)), &disturbed);
                assert!(!effects.iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { .. })));
                assert!(effects.contains(&RunnerEffect::SetCpuMaxWatts(70.0)), "{effects:?}");
                if attempt < 2 {
                    assert!(!step.done(), "{effects:?}");
                    assert!(step.primary.is_empty());
                    assert_eq!(step.sub, if device == CalibDevice::Cpu { DeviceSub::Settle } else { DeviceSub::RecoverGpu });
                    assert!(format!("{effects:?}").contains(&format!("retry {}/2", attempt + 1)));
                } else {
                    assert!(step.done());
                    assert!(effects.iter().any(|effect| matches!(effect, RunnerEffect::Noted(reason) if reason.contains("2 retries"))));
                }
            }
        }
    }

    #[test]
    fn recorded_september_run_accepts_both_fits_with_measured_gpu_default() {
        let data = include_str!("fixtures/2026-09-12-responses.csv");
        for (name, device, expected_kc) in [("cpu", CalibDevice::Cpu, 0.332_081_5), ("gpu", CalibDevice::Gpu, 22.221_804_8)] {
            let mut step = PerDeviceStepTest::new();
            step.baseline_cpu_w = 15.0;
            step.applied_step_cpu_w = Some(30.0);
            step.baseline_gpu_mhz = 1_000;
            step.applied_step_gpu_mhz = Some(1_500);
            step.primary = data.lines().filter_map(|line| {
                let mut fields = line.split(',');
                if fields.next()? != name { return None; }
                Some((fields.next()?.parse().unwrap(), fields.next()?.parse().unwrap()))
            }).collect();
            assert_eq!(step.primary.len(), 360);
            let effects = step.conclude_device(device, 60);
            assert!(effects.iter().any(|effect| matches!(effect,
                RunnerEffect::FittedDevice { device: fitted, gains }
                if *fitted == device && (gains.kc - expected_kc).abs() < 0.0001)), "{effects:?}");
        }
    }

    #[test]
    fn fit_rejections_explain_response_and_gain_limits_separately() {
        let mut step = PerDeviceStepTest::new();
        step.applied_step_cpu_w = Some(30.0);
        step.baseline_cpu_w = 15.0;
        for (amplitude, tau, expected) in [(2.0, 35.0, "fitted response"), (4.0, 300.0, "outside allowed")] {
            step.primary = (1..=360).map(|t| {
                (t as f64, 50.0 + amplitude * (1.0 - (-(t as f64 - 20.0).max(0.0) / tau).exp()))
            }).collect();
            let effects = step.conclude_device(CalibDevice::Cpu, 60);
            assert!(effects.iter().any(|effect| matches!(effect,
                RunnerEffect::Noted(reason) if reason.contains(expected))), "{effects:?}");
        }
    }

    #[test]
    fn primary_response_below_three_c_rejects_only_that_device() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &ctx);
        let response_ctx = cpu_step_context(&ctx, 35.0);
        let mut end = Vec::new();
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            let mut tick = response_ctx.clone();
            tick.cpu_group_c = Some(50.0 + 2.9 * (1.0 - (-(elapsed as f64 - 20.0).max(0.0) / 35.0).exp()));
            tick.gpu_group_c = Some(55.0);
            end = step.on_sample(&sample_at(60.0 + elapsed as f64), &tick);
        }
        assert!(end.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("below 3C"))));
        assert!(end.contains(&RunnerEffect::SetGpuMaxClock(2_800)));
        let recovered = PerDeviceCalibContext {
            cpu_cap_w: Some(20.0),
            cpu_cap_completed_at_s: Some(420.0),
            gpu_cap_completed_at_s: Some(420.0),
            ..ctx
        };
        for elapsed in 1..=60 {
            assert!(step.on_sample(&sample_at(420.0 + elapsed as f64), &recovered).is_empty());
        }
        assert_eq!(
            step.on_sample(&sample_at(481.0), &recovered),
            vec![RunnerEffect::SetGpuMaxClock(3_090)]
        );
    }

    #[test]
    fn settle_windows_use_elapsed_time_at_20_and_60_second_boundaries() {
        let mut step = PerDeviceStepTest::new();
        let ctx = context();
        let at = |t_mono: f64| Sample {
            t_mono,
            fan_valid: true,
            fan1_rpm: 2_400.0,
            ..Sample::default()
        };

        step.on_sample(&at(0.0), &ctx);
        for i in 0..80 {
            let t = 39.999 * f64::from(i) / 79.0;
            assert!(
                !step.on_sample(&at(t), &ctx).contains(&RunnerEffect::SetCpuMaxWatts(80.0)),
                "dense samples before 60 elapsed seconds must not satisfy settle"
            );
        }
        assert!(step.on_sample(&at(40.0), &ctx).is_empty());
        for i in 0..80 {
            let t = 40.001 + 19.998 * f64::from(i) / 79.0;
            assert!(!step.on_sample(&at(t), &ctx).contains(&RunnerEffect::SetCpuMaxWatts(80.0)));
        }
        let effects = step.on_sample(&at(60.0), &ctx);
        assert!(effects.contains(&RunnerEffect::SetCpuMaxWatts(80.0)));

        let mut fan_boundary = PerDeviceStepTest::new();
        fan_boundary.on_sample(&at(0.0), &ctx);
        let without_fan = Sample { fan_valid: false, ..at(0.0) };
        fan_boundary.on_sample(&without_fan, &ctx);
        fan_boundary.on_sample(&Sample { fan_valid: false, ..at(39.999) }, &ctx);
        fan_boundary.on_sample(&at(40.0), &ctx);
        assert!(fan_boundary.on_sample(&at(59.999), &ctx).is_empty());
        let at_boundary = fan_boundary.on_sample(&at(60.0), &ctx);
        assert!(at_boundary.contains(&RunnerEffect::SetCpuMaxWatts(80.0)), "{at_boundary:?}");
    }

    #[test]
    fn response_and_timeout_use_exact_360_and_600_second_boundaries() {
        let mut timeout = PerDeviceStepTest::new();
        let mut blocked = context();
        blocked.fanctrl_active = false;
        let at = |t_mono: f64| Sample { t_mono, fan_valid: true, fan1_rpm: 2_400.0, ..Sample::default() };
        timeout.on_sample(&at(0.0), &blocked);
        for i in 0..700 {
            let t = 599.999 * f64::from(i) / 699.0;
            assert!(!timeout.on_sample(&at(t), &blocked).iter().any(|effect| matches!(effect, RunnerEffect::Noted(_))));
        }
        assert!(timeout.on_sample(&at(600.0), &blocked).iter().any(|effect| matches!(effect, RunnerEffect::Noted(reason) if reason.contains("600"))));

        let mut response = PerDeviceStepTest::new();
        response.on_sample(&at(0.0), &context());
        for t in [0.0, 20.0, 40.0, 60.0] { response.on_sample(&at(t), &context()); }
        let response_ctx = cpu_step_context(&context(), 80.0);
        for i in 0..400 {
            let t = 60.001 + 359.998 * f64::from(i) / 399.0;
            let mut ctx = response_ctx.clone();
            ctx.cpu_group_c = Some(50.0 + 8.0 * (1.0 - (-(t - 80.0).max(0.0) / 35.0).exp()));
            assert!(!response.on_sample(&at(t), &ctx).iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { .. })));
        }
        let mut ctx = response_ctx;
        ctx.cpu_group_c = Some(58.0);
        assert!(response.on_sample(&at(420.0), &ctx).iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { device: CalibDevice::Cpu, .. })));
    }

    #[test]
    fn response_rejects_a_failed_or_externally_changed_applied_pair() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &ctx);
        ctx.cpu_cap_w = Some(35.0);
        ctx.cpu_cap_completed_at_s = Some(60.0);
        ctx.gpu_cap_mhz = Some(2_700);
        let effects = step.on_sample(&sample_at(61.0), &ctx);
        assert!(effects.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("applied pair"))));
    }

    #[test]
    fn response_does_not_require_the_other_groups_trace() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &ctx);
        let mut response = cpu_step_context(&ctx, 35.0);
        let mut end = Vec::new();
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            response.cpu_group_c = Some(50.0 + 12.0 * (1.0 - (-(elapsed as f64 - 20.0).max(0.0) / 35.0).exp()));
            response.gpu_group_c = None;
            end = step.on_sample(&sample_at(60.0 + elapsed as f64), &response);
        }
        assert!(end.iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { device: CalibDevice::Cpu, .. })), "{end:?}");
    }

    #[test]
    fn clamped_verified_cpu_step_uses_the_native_applied_delta_and_failed_write_never_fits() {
        let mut step = PerDeviceStepTest::new();
        let mut settled = context();
        settled.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &settled);
        let clamped = cpu_step_context(&settled, 30.0);
        let response = |elapsed: f64| {
            if elapsed <= 20.0 {
                50.0
            } else {
                50.0 + 0.8 * 10.0 * (1.0 - (-(elapsed - 20.0) / 35.0).exp())
            }
        };
        let mut end = Vec::new();
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            let mut tick = clamped.clone();
            tick.cpu_group_c = Some(response(elapsed as f64));
            end = step.on_sample(&sample_at(60.0 + elapsed as f64), &tick);
        }
        let gains = end.iter().find_map(|effect| match effect {
            RunnerEffect::FittedDevice { device: CalibDevice::Cpu, gains } => Some(*gains),
            _ => None,
        }).expect("clamped verified step must fit");
        let expected_kc = 35.0 / (0.8 * (90.0 + 20.0));
        assert!((gains.kc - expected_kc).abs() / expected_kc < 0.1, "{gains:?}");

        let mut failed = PerDeviceStepTest::new();
        enter_cpu_step(&mut failed, &settled);
        let mut no_write = settled;
        no_write.cpu_cap_verified = false;
        no_write.cpu_cap_completed_at_s = None;
        let effects = failed.on_sample(&sample_at(420.0), &no_write);
        assert!(effects.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("no verified applied pair"))));
        assert!(!effects.iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { .. })));
    }

    #[test]
    fn strategy_or_interval_change_aborts_without_saving_under_the_new_key() {
        let mut step = PerDeviceStepTest::new();
        let ctx = context();
        enter_cpu_step(&mut step, &ctx);
        let mut changed = ctx;
        changed.strategy = Some("performance".into());
        changed.ma_interval = Some(30);
        let effects = step.on_sample(&sample_at(61.0), &changed);
        assert!(effects.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("changed during calibration"))));
        assert!(step.done());
        assert_eq!(step.key(), Some("quiet16:60"));
    }

    #[test]
    fn settle_timeout_reports_status_and_actual_duration_for_every_condition() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.fanctrl_active = false;
        let at = |t_mono: f64| Sample { t_mono, fan_valid: true, fan1_rpm: 2_400.0, ..Sample::default() };
        step.on_sample(&at(0.0), &ctx);
        let effects = step.on_sample(&at(600.0), &ctx);
        let reason = effects.iter().find_map(|effect| match effect {
            RunnerEffect::Noted(reason) => Some(reason.as_str()),
            _ => None,
        }).expect("timeout reason");
        for field in ["caps held=", "fanctrl active=", "EC match=", "argmax controllable=", "CPU group flat=", "GPU group flat=", "fans flat="] {
            assert!(reason.contains(field), "missing {field:?}: {reason}");
        }
        assert!(reason.contains("failed for 600.000s"), "{reason}");
        assert!(reason.contains("satisfied for"), "{reason}");
    }
}

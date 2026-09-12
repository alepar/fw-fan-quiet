//! Revision-4 per-device calibration. [`PerDeviceStepTest`] establishes and
//! holds a CPU-watt/GPU-clock pair, waits for both EC groups and the fans to
//! settle, then records independent CPU and GPU FOPDT steps. Every request
//! is returned as a [`RunnerEffect`] for the controller's normal actuator
//! paths; hot guards restore the pair before any step write can win.
//!
//! The older scalar [`StepTest`] remains as a compatibility surface until
//! the scheduled deletion sweep removes the scalar budget and LUT code.

use std::collections::VecDeque;

use crate::calib::fopdt::{
    MIN_EC_RESPONSE_C, MIN_RPM_RESPONSE, derive_device_gains, derive_gains, fit_fopdt,
};
use crate::calib::runner::{BURNER_THREADS, RunnerEffect};
use crate::calib::steady::{STEADY_N, STEADY_RPM_TOLERANCE, is_steady, tail_mean};
use crate::control::budget::LoopGains;
use crate::control::device_loop::{Mhz, W, default_gains};
use crate::sensors::ec::EcLabel;
use crate::types::Sample;

/// Settle detection: the EC moving average must be flat within this many °C
/// over [`EC_FLAT_WINDOW_S`] seconds (design §3.3).
#[allow(dead_code)]
const EC_FLAT_TOLERANCE_C: f64 = 0.5;
/// Settle/baseline window, seconds (== samples at the 1 Hz sample rate).
#[allow(dead_code)]
const EC_FLAT_WINDOW_S: usize = 60;
/// Settle detection gives up after this many samples (5 min, design §3.3).
#[allow(dead_code)]
const SETTLE_CAP_SAMPLES: usize = 300;

/// The step size requested on top of the floor, watts (design §3.3).
#[allow(dead_code)]
const STEP_W: f64 = 30.0;
/// The step holds at most this long (5 min, design §3.3).
#[allow(dead_code)]
const STEP_CAP_SAMPLES: usize = 300;
/// ...or exits early once the EC average has been flat this long (90 s,
/// design §3.3). Reuses [`EC_FLAT_TOLERANCE_C`] as the flatness band.
#[allow(dead_code)]
const STEP_FLAT_WINDOW_S: usize = 90;
/// EC max above this aborts the step and restores the floor (design §3.3).
const EC_MAX_ABORT_C: f64 = 95.0;
/// Below this measured total power delta the step counts as "never loaded"
/// (design §3.3's "the applied power never rose" skip condition) — a
/// physically negligible threshold, well under any real burner/GPU draw.
#[allow(dead_code)]
const MIN_STEP_DELTA_W: f64 = 1.0;

/// Utilization above which the GPU counts as loaded — mirrors
/// `lut_sweep::PIN_UTIL_MIN_PCT` (private to that module, so this restates
/// the same 90 % threshold rather than reaching into it).
#[allow(dead_code)]
const PIN_UTIL_MIN_PCT: f64 = 90.0;
/// `NeedsGpuLoad` nag cadence during the step (mirrors the sweep's own
/// `NEEDS_LOAD_EVERY`).
#[allow(dead_code)]
const NEEDS_LOAD_EVERY: usize = 10;

pub const DEVICE_SETTLE_S: usize = 600;
pub const DEVICE_FIT_WINDOW_S: usize = 360;
/// The sampler is 1 Hz. Allow one delayed tick, but never let a sparse trace
/// masquerade as continuous full-window response coverage.
const DEVICE_RESPONSE_MAX_GAP_S: f64 = 2.0;
pub const DEVICE_STEP_W: f64 = 15.0;
pub const DEVICE_STEP_MHZ: u32 = 500;
const DEVICE_GROUP_WINDOW_S: usize = 60;
const DEVICE_FAN_WINDOW_S: usize = 20;
const DEVICE_GROUP_SPAN_C: f64 = 0.5;
const DEVICE_FAN_SPAN_RPM: f64 = 150.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibDevice {
    Cpu,
    Gpu,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PerDeviceCalibContext {
    pub cpu_cap_w: Option<f64>,
    pub cpu_cap_verified: bool,
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

pub struct PerDeviceStepTest {
    sub: DeviceSub,
    initialized: bool,
    run_started_at_s: Option<f64>,
    phase_commanded_at_s: Option<f64>,
    last_sample_at_s: Option<f64>,
    cpu_hot_streak: u8,
    baseline_cpu_w: f64,
    baseline_gpu_mhz: u32,
    baseline_cpu_group_c: f64,
    baseline_gpu_group_c: f64,
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
    secondary: Vec<(f64, f64)>,
}

impl PerDeviceStepTest {
    pub fn new() -> Self {
        Self {
            sub: DeviceSub::Settle,
            initialized: false,
            run_started_at_s: None,
            phase_commanded_at_s: None,
            last_sample_at_s: None,
            cpu_hot_streak: 0,
            baseline_cpu_w: 0.0,
            baseline_gpu_mhz: 0,
            baseline_cpu_group_c: 0.0,
            baseline_gpu_group_c: 0.0,
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
            secondary: Vec::new(),
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

    pub fn on_sample(
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

        let cpu_flat = timed_span_within(&self.cpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_GROUP_SPAN_C);
        let gpu_flat = timed_span_within(&self.gpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_GROUP_SPAN_C);
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
            self.baseline_cpu_group_c = timed_mean(&self.cpu_group).expect("full CPU settle window");
            self.baseline_gpu_group_c = timed_mean(&self.gpu_group).expect("full GPU settle window");
            self.sub = DeviceSub::Cpu;
            self.phase_commanded_at_s = Some(now_s);
            self.applied_step_cpu_w = None;
            self.response_last_at_s = None;
            self.primary.clear();
            self.secondary.clear();
            return vec![RunnerEffect::SetCpuMaxWatts(self.step_cpu_w)];
        }
        if now_s - self.run_started_at_s.unwrap_or(now_s) >= DEVICE_SETTLE_S as f64 {
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
        if !pair_held {
            return self.abort(format!(
                "{} step rejected: verified applied pair changed during response",
                device_name(device)
            ));
        }
        let (primary, secondary) = match device {
            CalibDevice::Cpu => (ctx.cpu_group_c, ctx.gpu_group_c),
            CalibDevice::Gpu => (ctx.gpu_group_c, ctx.cpu_group_c),
        };
        let t = now_s - self.phase_commanded_at_s.unwrap_or(now_s);
        let Some(primary) = primary.filter(|value| value.is_finite()) else {
            return self.finish_device_response(now_s, device, vec![RunnerEffect::Noted(format!(
                "{} fit rejected: primary group coverage missing or interrupted",
                device_name(device)
            ))]);
        };
        let Some(secondary) = secondary.filter(|value| value.is_finite()) else {
            return self.finish_device_response(now_s, device, vec![RunnerEffect::Noted(format!(
                "{} fit rejected: secondary group coverage missing or interrupted",
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
        self.secondary.push((t, secondary));
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
        self.secondary.clear();
        self.response_last_at_s = None;
        match device {
            CalibDevice::Cpu => {
                // Restore the complete held pair after the CPU step. The
                // CPU group is still carrying the step response, so wait
                // for the same physical flatness conditions before the GPU
                // step; otherwise CPU cooling would be misclassified as a
                // GPU-step cross term on every real run.
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
        let cpu_flat = timed_span_within(&self.cpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_GROUP_SPAN_C);
        let gpu_flat = timed_span_within(&self.gpu_group, DEVICE_GROUP_WINDOW_S as f64, DEVICE_GROUP_SPAN_C);
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
            self.baseline_cpu_group_c = timed_mean(&self.cpu_group).expect("full CPU recovery window");
            self.baseline_gpu_group_c = timed_mean(&self.gpu_group).expect("full GPU recovery window");
            self.sub = DeviceSub::Gpu;
            self.phase_commanded_at_s = Some(now_s);
            self.response_last_at_s = None;
            self.primary.clear();
            self.secondary.clear();
            return vec![RunnerEffect::SetGpuMaxClock(self.step_gpu_mhz)];
        }
        if now_s - recovery_started >= DEVICE_SETTLE_S as f64 {
            return self.abort(format!("between-step recovery {}", self.settle_timeout_reason()));
        }
        Vec::new()
    }

    fn conclude_device(&self, device: CalibDevice, interval: u32) -> Vec<RunnerEffect> {
        let primary_baseline = match device {
            CalibDevice::Cpu => self.baseline_cpu_group_c,
            CalibDevice::Gpu => self.baseline_gpu_group_c,
        };
        let secondary_baseline = match device {
            CalibDevice::Cpu => self.baseline_gpu_group_c,
            CalibDevice::Gpu => self.baseline_cpu_group_c,
        };
        let primary_delta = tail_series_mean(&self.primary).map_or(0.0, |v| v - primary_baseline);
        let secondary_delta = self
            .secondary
            .iter()
            .map(|(_, value)| (value - secondary_baseline).abs())
            .fold(0.0, f64::max);
        let cross_limit = 1.0_f64.max(0.2 * primary_delta.abs());
        if secondary_delta > cross_limit {
            return vec![RunnerEffect::Noted(format!(
                "{} fit rejected: other group moved {:.2}C > {:.2}C (load changed)",
                device_name(device), secondary_delta, cross_limit
            ))];
        }
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
            Some(gains) => vec![RunnerEffect::FittedDevice { device, gains }],
            None => vec![RunnerEffect::Noted(format!(
                "{} fit rejected: primary response below 3C or model invalid",
                device_name(device)
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
            while window
                .front()
                .is_some_and(|(at, _)| now_s - at > horizon_s)
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

fn tail_series_mean(series: &[(f64, f64)]) -> Option<f64> {
    if series.is_empty() { return None; }
    let n = series.len().min(60);
    Some(series.iter().rev().take(n).map(|(_, v)| v).sum::<f64>() / n as f64)
}

fn timed_mean(series: &VecDeque<(f64, f64)>) -> Option<f64> {
    (!series.is_empty())
        .then(|| series.iter().map(|(_, value)| *value).sum::<f64>() / series.len() as f64)
}

/// What the controller knows this sample that the step needs, built from
/// the same inputs the auto loop uses (design §3.3). The step never reads
/// the socket or the arbiter itself.
#[allow(dead_code)] // legacy scalar step context until task .12
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CalibContext {
    /// Current successful CPU cap, if the controller has one. The
    /// per-device calibration algorithm consumes this in task .8; carrying
    /// it now keeps the controller-to-runner boundary explicit.
    pub cpu_cap_w: Option<f64>,
    /// Current successful GPU lock, if a dGPU is present. See
    /// [`Self::cpu_cap_w`] for why this is a context seam rather than a
    /// step-test input yet.
    pub gpu_cap_mhz: Option<u32>,
    /// The live EC moving average (`EcAverage::push`'s output), if seeded.
    pub ec_ma: Option<f64>,
    /// Three-strikes-latched EC/replica disagreement (design §2.6).
    pub ec_mismatch: bool,
    /// fw-fanctrl socket fresh and reporting `active: true`.
    pub fanctrl_active: bool,
    /// The EC argmax sensor is one the controller can steer (design §2.5).
    pub argmax_controllable: bool,
    /// The budget integrator's current `(lo, hi)` clamp bounds, watts.
    pub budget_bounds: (f64, f64),
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
            tick.gpu_group_c = Some(response(55.0, 0.02, 15.0, 60.0, 290.0, elapsed as f64));
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
    fn cross_term_above_max_one_or_twenty_percent_rejects_as_load_changed() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &ctx);
        let response_ctx = cpu_step_context(&ctx, 35.0);
        let mut end = Vec::new();
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            let mut tick = response_ctx.clone();
            tick.cpu_group_c = Some(if elapsed < 20 { 50.0 } else { 62.0 });
            tick.gpu_group_c = Some(if elapsed < 20 { 55.0 } else { 58.1 });
            end = step.on_sample(&sample_at(60.0 + elapsed as f64), &tick);
        }
        assert!(end.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("load changed"))));
        assert!(!end.iter().any(|effect| matches!(effect,
            RunnerEffect::FittedDevice { device: CalibDevice::Cpu, .. })));
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
            tick.cpu_group_c = Some(if elapsed < 20 { 50.0 } else { 52.9 });
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
    fn a_secondary_excursion_that_returns_to_baseline_still_rejects_the_fit() {
        let mut step = PerDeviceStepTest::new();
        let mut ctx = context();
        ctx.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &ctx);
        let response_ctx = cpu_step_context(&ctx, 35.0);
        let mut end = Vec::new();
        for elapsed in 1..=DEVICE_FIT_WINDOW_S {
            let mut tick = response_ctx.clone();
            tick.cpu_group_c = Some(if elapsed < 20 { 50.0 } else { 62.0 });
            tick.gpu_group_c = Some(if (100..180).contains(&elapsed) { 59.0 } else { 55.0 });
            end = step.on_sample(&sample_at(60.0 + elapsed as f64), &tick);
        }
        assert!(end.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("load changed"))));
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
    fn response_rejects_a_wholly_missing_secondary_group_instead_of_treating_it_as_zero() {
        let mut step = PerDeviceStepTest::new();
        let mut settled = context();
        settled.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &settled);
        let mut response = cpu_step_context(&settled, 35.0);
        response.gpu_group_c = None;
        let effects = step.on_sample(&sample_at(61.0), &response);
        assert!(effects.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("secondary group coverage"))), "{effects:?}");
        assert!(!effects.iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { .. })));
    }

    #[test]
    fn response_rejects_an_interrupted_secondary_group_trace() {
        let mut step = PerDeviceStepTest::new();
        let mut settled = context();
        settled.cpu_cap_w = Some(20.0);
        enter_cpu_step(&mut step, &settled);
        let response = cpu_step_context(&settled, 35.0);
        for elapsed in 1..=40 {
            assert!(step.on_sample(&sample_at(60.0 + f64::from(elapsed)), &response).is_empty());
        }
        let mut interrupted = response;
        interrupted.gpu_group_c = None;
        let effects = step.on_sample(&sample_at(101.0), &interrupted);
        assert!(effects.iter().any(|effect| matches!(effect,
            RunnerEffect::Noted(reason) if reason.contains("secondary group coverage"))), "{effects:?}");
        assert!(!effects.iter().any(|effect| matches!(effect, RunnerEffect::FittedDevice { .. })));
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

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Sub {
    /// Burner running, budget held at the floor, waiting for EC + RPM to
    /// settle.
    Settle,
    /// Budget stepped by `STEP_W`; recording the response.
    Step,
}

/// The step-test state machine. Construct fresh per calibration session;
/// [`StepTest::enter`] starts it once the LUT sweep hands off.
#[allow(dead_code)] // inactive legacy scalar step retained until task .12
pub struct StepTest {
    sub: Sub,
    elapsed: usize,
    lo: f64,

    // Settle-phase accumulation windows, reset together whenever a gate is
    // unmet or a reading is invalid (never fabricate settled evidence from
    // missing or gated data).
    ec_settle: VecDeque<f64>,
    rpm_settle: VecDeque<f64>,
    cpu_settle: VecDeque<f64>,
    gpu_settle: VecDeque<f64>,

    /// Smoothed draw at the moment settle completed — the fit's "before".
    baseline_cpu_w: f64,
    baseline_gpu_w: f64,

    // Step-phase recording.
    ec_series: Vec<(f64, f64)>,
    rpm_series: Vec<(f64, f64)>,
    ec_flat: VecDeque<f64>,
    cpu_step: VecDeque<f64>,
    gpu_step: VecDeque<f64>,
    argmax_at_step_start: Option<EcLabel>,
    non_accum: usize,
    needs_load: bool,
}

#[allow(dead_code)]
impl StepTest {
    pub fn new() -> Self {
        Self {
            sub: Sub::Settle,
            elapsed: 0,
            lo: 0.0,
            ec_settle: VecDeque::new(),
            rpm_settle: VecDeque::new(),
            cpu_settle: VecDeque::new(),
            gpu_settle: VecDeque::new(),
            baseline_cpu_w: 0.0,
            baseline_gpu_w: 0.0,
            ec_series: Vec::new(),
            rpm_series: Vec::new(),
            ec_flat: VecDeque::new(),
            cpu_step: VecDeque::new(),
            gpu_step: VecDeque::new(),
            argmax_at_step_start: None,
            non_accum: 0,
            needs_load: false,
        }
    }

    /// True while the step itself (not the settle hold) is recording — the
    /// runner uses this to report `needs_load` in its progress snapshot.
    pub fn needs_load(&self) -> bool {
        self.needs_load
    }

    /// True once the budget has been stepped (progress reporting only).
    pub fn stepping(&self) -> bool {
        self.sub == Sub::Step
    }

    /// Begin the phase: the burner starts unconditionally, before any gate
    /// is ever evaluated (see the module-level ordering note), and the
    /// budget is held at the floor while settle detection runs.
    pub fn enter(&mut self, budget_bounds: (f64, f64)) -> Vec<RunnerEffect> {
        *self = Self::new();
        self.lo = budget_bounds.0;
        vec![
            RunnerEffect::StartBurner(BURNER_THREADS),
            RunnerEffect::SetBudget(self.lo),
        ]
    }

    /// Consume one 1 Hz sample.
    pub fn on_sample(&mut self, s: &Sample, ctx: &CalibContext) -> Vec<RunnerEffect> {
        match self.sub {
            Sub::Settle => self.on_settle_sample(s, ctx),
            Sub::Step => self.on_step_sample(s, ctx),
        }
    }

    fn on_settle_sample(&mut self, s: &Sample, ctx: &CalibContext) -> Vec<RunnerEffect> {
        self.elapsed += 1;

        let ok = ctx.fanctrl_active
            && !ctx.ec_mismatch
            && ctx.argmax_controllable
            && s.ec_valid
            && s.fan_valid
            && ctx.ec_ma.is_some();

        if ok {
            let ec_ma = ctx.ec_ma.expect("checked above");
            push_capped(&mut self.ec_settle, ec_ma, EC_FLAT_WINDOW_S);
            push_capped(&mut self.rpm_settle, s.max_fan_rpm(), STEADY_N);
            push_capped(&mut self.cpu_settle, s.cpu_pkg_w, EC_FLAT_WINDOW_S);
            push_capped(&mut self.gpu_settle, s.gpu_w, EC_FLAT_WINDOW_S);

            let ec_flat = is_steady(
                self.ec_settle.make_contiguous(),
                EC_FLAT_WINDOW_S,
                EC_FLAT_TOLERANCE_C,
            );
            let rpm_flat = is_steady(
                self.rpm_settle.make_contiguous(),
                STEADY_N,
                STEADY_RPM_TOLERANCE,
            );
            if ec_flat && rpm_flat {
                self.baseline_cpu_w =
                    tail_mean(self.cpu_settle.make_contiguous(), EC_FLAT_WINDOW_S)
                        .expect("cpu window pushed in lockstep with ec_settle");
                self.baseline_gpu_w =
                    tail_mean(self.gpu_settle.make_contiguous(), EC_FLAT_WINDOW_S)
                        .expect("gpu window pushed in lockstep with ec_settle");
                self.sub = Sub::Step;
                self.elapsed = 0;
                return vec![RunnerEffect::SetBudget(self.lo + STEP_W)];
            }
        } else {
            self.ec_settle.clear();
            self.rpm_settle.clear();
            self.cpu_settle.clear();
            self.gpu_settle.clear();
        }

        if self.elapsed >= SETTLE_CAP_SAMPLES {
            return self.conclude_skip(
                "step-test settle timed out: fanctrl_active/ec_mismatch/argmax_controllable \
                 never held long enough to settle"
                    .to_string(),
            );
        }
        Vec::new()
    }

    fn on_step_sample(&mut self, s: &Sample, ctx: &CalibContext) -> Vec<RunnerEffect> {
        self.elapsed += 1;
        // `t = 0` at the first step sample (the instant `SetBudget` was
        // raised), matching `fit_fopdt`'s "t measured from the step" origin.
        let t = (self.elapsed - 1) as f64;

        // Hard gates: once the step itself is under way, losing either one
        // aborts on the spot rather than waiting out a cap — the recording
        // would otherwise carry a discontinuity the fit cannot explain.
        if !ctx.fanctrl_active || ctx.ec_mismatch {
            return self.conclude_skip("fanctrl inactive or ec mismatch mid-step".to_string());
        }
        if s.ec_valid
            && s.ec
                .as_ref()
                .is_some_and(|r| f64::from(r.max_c) > EC_MAX_ABORT_C)
        {
            return self.conclude_skip("ec max exceeded 95\u{b0}C mid-step".to_string());
        }
        if s.ec_valid {
            let label = s.ec.as_ref().map(|r| r.argmax.clone());
            match &self.argmax_at_step_start {
                None => self.argmax_at_step_start = label,
                Some(start) if label.as_ref() != Some(start) => {
                    return self.conclude_skip("argmax label changed mid-step".to_string());
                }
                Some(_) => {}
            }
        }

        let mut effects = Vec::new();
        if s.gpu_util_pct < PIN_UTIL_MIN_PCT {
            self.non_accum += 1;
            if self.non_accum.is_multiple_of(NEEDS_LOAD_EVERY) {
                self.needs_load = true;
                effects.push(RunnerEffect::NeedsGpuLoad);
            }
        } else {
            self.non_accum = 0;
            self.needs_load = false;
        }

        if s.ec_valid
            && let Some(ec_ma) = ctx.ec_ma
        {
            self.ec_series.push((t, ec_ma));
            push_capped(&mut self.ec_flat, ec_ma, STEP_FLAT_WINDOW_S);
            push_capped(&mut self.cpu_step, s.cpu_pkg_w, EC_FLAT_WINDOW_S);
            push_capped(&mut self.gpu_step, s.gpu_w, EC_FLAT_WINDOW_S);
        }
        if s.fan_valid {
            self.rpm_series.push((t, s.max_fan_rpm()));
        }

        let flat_done = is_steady(
            self.ec_flat.make_contiguous(),
            STEP_FLAT_WINDOW_S,
            EC_FLAT_TOLERANCE_C,
        );
        if flat_done || self.elapsed >= STEP_CAP_SAMPLES {
            effects.extend(self.conclude_step(s));
        }
        effects
    }

    /// Step recording done (cap or early-flat exit): reject an unloaded
    /// step, fit both signals on the MEASURED delta (never `STEP_W`), and
    /// derive gains — or skip with a `Noted` reason at any rejection.
    fn conclude_step(&mut self, s: &Sample) -> Vec<RunnerEffect> {
        let final_cpu = tail_mean(self.cpu_step.make_contiguous(), EC_FLAT_WINDOW_S)
            .unwrap_or(self.baseline_cpu_w);
        let final_gpu = tail_mean(self.gpu_step.make_contiguous(), EC_FLAT_WINDOW_S)
            .unwrap_or(self.baseline_gpu_w);
        let delta_w = (final_cpu - self.baseline_cpu_w) + (final_gpu - self.baseline_gpu_w);

        if delta_w < MIN_STEP_DELTA_W {
            return self.conclude_skip("applied power never rose during the step".to_string());
        }

        let ec_fit = fit_fopdt(&self.ec_series, delta_w, MIN_EC_RESPONSE_C);
        let rpm_fit = fit_fopdt(&self.rpm_series, delta_w, MIN_RPM_RESPONSE);
        let gains = match (ec_fit, rpm_fit) {
            (Some(ec), Some(rpm)) => derive_gains(&ec, &rpm, &LoopGains::default()),
            _ => None,
        };
        let Some(gains) = gains else {
            return self.conclude_skip("step-test fit rejected".to_string());
        };

        let fitted_at = s.t_mono.max(0.0).round() as u64;
        vec![
            RunnerEffect::SetBudget(self.lo),
            RunnerEffect::StopBurner,
            RunnerEffect::Fitted { gains, fitted_at },
        ]
    }

    /// Terminal skip: restore the floor, stop the burner, note why. The
    /// caller (the runner) still finishes calibration and keeps the
    /// defaults — this is not a hard failure.
    fn conclude_skip(&mut self, reason: String) -> Vec<RunnerEffect> {
        vec![
            RunnerEffect::SetBudget(self.lo),
            RunnerEffect::StopBurner,
            RunnerEffect::Noted(reason),
        ]
    }
}

impl Default for StepTest {
    fn default() -> Self {
        Self::new()
    }
}

fn push_capped(window: &mut VecDeque<f64>, v: f64, cap: usize) {
    if window.len() == cap {
        window.pop_front();
    }
    window.push_back(v);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::ec::EcReading;
    use std::sync::atomic::{AtomicU64, Ordering};

    // ---- EcReading fixture (EcLabel's constructors are private to
    // sensors::ec; a synthetic hwmon dir via the module's own `read()` is
    // the established way to build one from outside, mirrors mode.rs's own
    // `ec_reading` test helper). ----

    static EC_FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    fn ec_reading(sensors: &[(&str, f64)]) -> EcReading {
        let n = EC_FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir =
            std::env::temp_dir().join(format!("bazerame-step-test-{}-{n}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for (i, (label, c)) in sensors.iter().enumerate() {
            let idx = i + 1;
            std::fs::write(dir.join(format!("temp{idx}_label")), format!("{label}\n")).unwrap();
            std::fs::write(
                dir.join(format!("temp{idx}_input")),
                format!("{}\n", (*c * 1000.0) as i64),
            )
            .unwrap();
        }
        let reading = EcReading::read(&dir).expect("fixture should yield a reading");
        std::fs::remove_dir_all(&dir).unwrap();
        reading
    }

    fn happy_ctx() -> CalibContext {
        CalibContext {
            cpu_cap_w: None,
            gpu_cap_mhz: None,
            ec_ma: Some(45.0),
            ec_mismatch: false,
            fanctrl_active: true,
            argmax_controllable: true,
            budget_bounds: (10.0, 130.0),
        }
    }

    /// A settle-phase sample: flat EC/RPM, cool EC max, CPU-controllable
    /// argmax, low GPU utilization (idle GPU during the burner-only hold).
    fn settle_sample() -> Sample {
        Sample {
            cpu_pkg_w: 10.0,
            gpu_w: 5.0,
            gpu_w_valid: true,
            gpu_util_pct: 3.0,
            fan1_rpm: 3000.0,
            fan_valid: true,
            ec: Some(ec_reading(&[("apu@4c", 60.0)])),
            ec_valid: true,
            ..Sample::default()
        }
    }

    /// Ground-truth FOPDT step response used by the fit-shaped tests: EC and
    /// RPM legs chosen so `derive_gains` accepts both (Kc within [0.25,4]x
    /// the `LoopGains::default()` on each leg — see the report for the
    /// arithmetic).
    const TAU: f64 = 35.0;
    const THETA: f64 = 10.0;
    const K_EC: f64 = 0.5; // degC per measured watt
    const K_RPM: f64 = 60.0; // RPM per measured watt
    const BASE_EC: f64 = 45.0;
    const BASE_RPM: f64 = 3000.0;

    fn step_response(base: f64, k: f64, delta_w: f64, t: f64) -> f64 {
        if t <= THETA {
            base
        } else {
            base + k * delta_w * (1.0 - (-(t - THETA) / TAU).exp())
        }
    }

    /// A step-phase sample at (0-indexed) step time `t`, with measured
    /// draw fixed at `cpu_w`/`gpu_w` (so the total measured delta is
    /// `(cpu_w - 10.0) + (gpu_w - 5.0)`, deliberately not `STEP_W`) and EC
    /// average / RPM following the ground-truth response for that delta.
    fn step_sample(t: f64, cpu_w: f64, gpu_w: f64, delta_w: f64) -> (Sample, CalibContext) {
        let ec_ma = step_response(BASE_EC, K_EC, delta_w, t);
        let rpm = step_response(BASE_RPM, K_RPM, delta_w, t);
        let s = Sample {
            cpu_pkg_w: cpu_w,
            gpu_w,
            gpu_w_valid: true,
            gpu_util_pct: 95.0,
            fan1_rpm: rpm,
            fan_valid: true,
            ec: Some(ec_reading(&[("apu@4c", 60.0)])),
            ec_valid: true,
            ..Sample::default()
        };
        let ctx = CalibContext {
            ec_ma: Some(ec_ma),
            ..happy_ctx()
        };
        (s, ctx)
    }

    /// Drive the step sub-phase to its conclusion (cap or early flat exit)
    /// with a fixed measured delta; returns every effect produced.
    fn drive_step(step: &mut StepTest, cpu_w: f64, gpu_w: f64, delta_w: f64) -> Vec<RunnerEffect> {
        let mut all = Vec::new();
        for i in 0..STEP_CAP_SAMPLES {
            let (s, ctx) = step_sample(i as f64, cpu_w, gpu_w, delta_w);
            let effects = step.on_sample(&s, &ctx);
            let concluded = effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::Fitted { .. } | RunnerEffect::Noted(_)));
            all.extend(effects);
            if concluded {
                return all;
            }
        }
        panic!("step never concluded within the cap");
    }

    // --- ordering: burner starts before settle, stops after the step ---

    #[test]
    fn enter_starts_the_burner_and_holds_the_floor_before_any_gate_is_checked() {
        let mut step = StepTest::new();
        let effects = step.enter((10.0, 130.0));
        assert_eq!(
            effects,
            vec![
                RunnerEffect::StartBurner(BURNER_THREADS),
                RunnerEffect::SetBudget(10.0),
            ]
        );
    }

    #[test]
    fn burner_stops_only_after_the_step_concludes() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        // 59 settle samples: no StopBurner yet (still settling).
        for _ in 0..(EC_FLAT_WINDOW_S - 1) {
            let effects = step.on_sample(&settle_sample(), &ctx);
            assert!(
                !effects.contains(&RunnerEffect::StopBurner),
                "burner stopped mid-settle"
            );
        }
        // 60th sample settles and steps the budget -- still no StopBurner.
        let effects = step.on_sample(&settle_sample(), &ctx);
        assert!(effects.contains(&RunnerEffect::SetBudget(10.0 + STEP_W)));
        assert!(!effects.contains(&RunnerEffect::StopBurner));

        let all = drive_step(&mut step, 22.0, 17.0, 24.0);
        assert!(
            all.contains(&RunnerEffect::StopBurner),
            "burner must stop once the step concludes: {all:?}"
        );
    }

    // --- the self-skip regression ---

    #[test]
    fn idle_uncontrollable_argmax_does_not_self_skip_once_loaded_argmax_is_controllable() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let idle_ctx = CalibContext {
            argmax_controllable: false,
            ..happy_ctx()
        };
        // 30 "idle" samples where the argmax is uncontrollable: must not
        // skip -- just withhold accumulation.
        for _ in 0..30 {
            let effects = step.on_sample(&settle_sample(), &idle_ctx);
            assert!(
                !effects.iter().any(|e| matches!(e, RunnerEffect::Noted(_))),
                "must not self-skip on an idle uncontrollable argmax: {effects:?}"
            );
        }
        // Now "loaded": argmax_controllable flips true and settle proceeds
        // normally -- the full 60-sample dwell, not shortened by the 30
        // idle samples above.
        let loaded_ctx = happy_ctx();
        let mut stepped = false;
        for i in 0..EC_FLAT_WINDOW_S {
            let effects = step.on_sample(&settle_sample(), &loaded_ctx);
            if effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::SetBudget(w) if *w > 10.0))
            {
                stepped = true;
                break;
            }
            assert!(
                i < EC_FLAT_WINDOW_S - 1,
                "settle must still complete within a fresh 60-sample dwell"
            );
        }
        assert!(stepped, "loaded run must proceed to the step");
    }

    // --- settle detection: flat EC + steady RPM, 5 min cap ---

    #[test]
    fn settle_requires_both_ec_flat_and_rpm_steady() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for i in 0..(EC_FLAT_WINDOW_S - 1) {
            let effects = step.on_sample(&settle_sample(), &ctx);
            assert!(
                effects.is_empty(),
                "must not step before the 60-sample dwell (i={i}): {effects:?}"
            );
        }
        let effects = step.on_sample(&settle_sample(), &ctx);
        assert_eq!(effects, vec![RunnerEffect::SetBudget(10.0 + STEP_W)]);
    }

    #[test]
    fn settle_gives_up_at_the_five_minute_cap_and_keeps_defaults() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        // A gate that never clears: fanctrl never active.
        let bad_ctx = CalibContext {
            fanctrl_active: false,
            ..happy_ctx()
        };
        let mut all = Vec::new();
        for _ in 0..SETTLE_CAP_SAMPLES {
            all.extend(step.on_sample(&settle_sample(), &bad_ctx));
        }
        assert!(
            all.iter().any(|e| matches!(e, RunnerEffect::Noted(_))),
            "must skip with a Noted reason at the cap: {all:?}"
        );
        assert!(all.contains(&RunnerEffect::SetBudget(10.0)));
        assert!(all.contains(&RunnerEffect::StopBurner));
        assert!(
            !all.iter().any(|e| matches!(e, RunnerEffect::Fitted { .. })),
            "a settle timeout must never fit"
        );
    }

    // --- the step: SetBudget(lo + 30), NeedsLoad nag ---

    #[test]
    fn step_requests_floor_plus_thirty_watts() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..(EC_FLAT_WINDOW_S - 1) {
            step.on_sample(&settle_sample(), &ctx);
        }
        let effects = step.on_sample(&settle_sample(), &ctx);
        assert_eq!(effects, vec![RunnerEffect::SetBudget(40.0)]);
    }

    #[test]
    fn gpu_below_pin_threshold_nags_needs_load_every_ten_samples() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            step.on_sample(&settle_sample(), &ctx);
        }
        assert!(step.stepping());

        let (mut s, ctx) = step_sample(0.0, 22.0, 17.0, 24.0);
        s.gpu_util_pct = 10.0; // below the pin threshold
        for i in 0..9 {
            let effects = step.on_sample(&s, &ctx);
            assert!(effects.is_empty(), "sample {i}: {effects:?}");
        }
        let effects = step.on_sample(&s, &ctx);
        assert_eq!(effects, vec![RunnerEffect::NeedsGpuLoad]);
        assert!(step.needs_load());
    }

    // --- measured delta drives the gain, never STEP_W ---

    #[test]
    fn gain_is_derived_from_the_measured_delta_not_the_nominal_thirty_watts() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            step.on_sample(&settle_sample(), &ctx);
        }
        // Commanded 30 W split unevenly and imperfectly tracked: measured
        // delta is 24 W (cpu +12, gpu +12), not the nominal 30.
        let all = drive_step(&mut step, 22.0, 17.0, 24.0);
        let fitted = all.iter().find_map(|e| match e {
            RunnerEffect::Fitted { gains, .. } => Some(*gains),
            _ => None,
        });
        let gains = fitted.unwrap_or_else(|| panic!("expected a fit to land: {all:?}"));

        // What the fit would have produced had it (wrongly) used the
        // nominal STEP_W as the gain denominator: k_ec_from_30 = k_ec *
        // (24/30) would be the response magnitude actually measured, so
        // dividing by 30 instead of 24 understates K by a factor of
        // 24/30 = 0.8, which OVERSTATES Kc by 1/0.8 = 1.25x.
        let wrong_kc = gains.kc_w_per_c * 0.8;
        assert!(
            (gains.kc_w_per_c - wrong_kc).abs() > 0.01 * gains.kc_w_per_c,
            "kc_w_per_c = {} must differ from the nominal-30W variant {wrong_kc}",
            gains.kc_w_per_c
        );
        // Directly: the response was generated from K_EC against a 24 W
        // step, so Kc = tau / (K_EC * (lambda + theta)) with lambda =
        // max(90, 3*theta) = 90 -- independent of delta_w by construction,
        // which is exactly why this assertion is only meaningful together
        // with the "fit converges near ground truth" check below.
        let lambda = (3.0 * THETA).max(90.0);
        let expected_kc = TAU / (K_EC * (lambda + THETA));
        assert!(
            (gains.kc_w_per_c - expected_kc).abs() < 0.15 * expected_kc,
            "kc_w_per_c = {} not within 15% of the ground-truth-K derivation {expected_kc}",
            gains.kc_w_per_c
        );
    }

    #[test]
    fn successful_fit_stamps_fitted_at_from_the_sample_clock() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            step.on_sample(&settle_sample(), &ctx);
        }
        let mut all = Vec::new();
        let mut last_t_mono = 0.0;
        for i in 0..STEP_CAP_SAMPLES {
            let (mut s, ctx) = step_sample(i as f64, 22.0, 17.0, 24.0);
            s.t_mono = 1_000.0 + i as f64;
            last_t_mono = s.t_mono;
            let effects = step.on_sample(&s, &ctx);
            let concluded = effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::Fitted { .. } | RunnerEffect::Noted(_)));
            all.extend(effects);
            if concluded {
                break;
            }
        }
        let fitted_at = all.iter().find_map(|e| match e {
            RunnerEffect::Fitted { fitted_at, .. } => Some(*fitted_at),
            _ => None,
        });
        assert_eq!(fitted_at, Some(last_t_mono.round() as u64));
    }

    // --- skip paths: unloaded step, 95C abort, argmax handover, rejected fit ---

    #[test]
    fn unloaded_step_skips_with_noted_reason_and_keeps_defaults() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            step.on_sample(&settle_sample(), &ctx);
        }
        // Measured draw never rises above the baseline: delta_w == 0.
        let all = drive_step(&mut step, 10.0, 5.0, 0.0);
        assert!(
            all.iter()
                .any(|e| matches!(e, RunnerEffect::Noted(reason) if reason.contains("never rose"))),
            "expected an unloaded-step skip: {all:?}"
        );
        assert!(!all.iter().any(|e| matches!(e, RunnerEffect::Fitted { .. })));
    }

    /// The `MIN_STEP_DELTA_W` gate is `delta_w < MIN_STEP_DELTA_W`, so the
    /// boundary itself is *loaded*: exactly 1.0 W must not be rejected as
    /// "never rose", and anything under it must. The unloaded test above
    /// drives a zero delta, well clear of the threshold; this pins the
    /// comparison's direction and inclusivity (ledger: task 18 self-reported
    /// the boundary as unmeasured).
    ///
    /// The delta `conclude_step` measures is the step-phase DRAW minus the
    /// settle baseline (`settle_sample`: 10 W CPU + 5 W GPU) — NOT
    /// `step_sample`'s fourth argument, which only shapes the EC/RPM
    /// response. So the boundary is scripted through `cpu_w`: 11.0 is a
    /// delta of exactly 1.0 (exact in f64), 11.0 − 1e-3 is just under.
    #[test]
    fn min_step_delta_boundary_is_inclusive() {
        let never_rose = |effects: &[RunnerEffect]| {
            effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::Noted(reason) if reason.contains("never rose")))
        };

        let mut at = StepTest::new();
        at.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            at.on_sample(&settle_sample(), &ctx);
        }
        let at_boundary = drive_step(&mut at, 10.0 + MIN_STEP_DELTA_W, 5.0, 24.0);
        assert!(
            !never_rose(&at_boundary),
            "a delta of exactly MIN_STEP_DELTA_W ({MIN_STEP_DELTA_W} W) is loaded and must not be \
             rejected as 'never rose': {at_boundary:?}"
        );

        let mut under = StepTest::new();
        under.enter((10.0, 130.0));
        for _ in 0..EC_FLAT_WINDOW_S {
            under.on_sample(&settle_sample(), &ctx);
        }
        let just_under = drive_step(&mut under, 10.0 + MIN_STEP_DELTA_W - 1e-3, 5.0, 24.0);
        assert!(
            never_rose(&just_under),
            "a delta just under MIN_STEP_DELTA_W must be rejected as 'never rose': {just_under:?}"
        );
        assert!(!just_under.iter().any(|e| matches!(e, RunnerEffect::Fitted { .. })));
    }

    #[test]
    fn ec_over_95c_aborts_mid_step_and_restores_the_floor() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            step.on_sample(&settle_sample(), &ctx);
        }
        assert!(step.stepping());

        let (mut s, ctx) = step_sample(0.0, 22.0, 17.0, 24.0);
        s.ec = Some(ec_reading(&[("apu@4c", 96.0)]));
        let effects = step.on_sample(&s, &ctx);
        assert!(effects.contains(&RunnerEffect::SetBudget(10.0)));
        assert!(effects.contains(&RunnerEffect::StopBurner));
        assert!(
            effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::Noted(reason) if reason.contains("95"))),
            "expected a 95C-abort skip: {effects:?}"
        );
    }

    #[test]
    fn argmax_label_change_mid_step_skips_and_keeps_defaults() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            step.on_sample(&settle_sample(), &ctx);
        }
        let (s0, ctx0) = step_sample(0.0, 22.0, 17.0, 24.0);
        let effects = step.on_sample(&s0, &ctx0);
        assert!(effects.iter().all(|e| !matches!(e, RunnerEffect::Noted(_))));

        // A different sensor takes over as argmax: sensor handover.
        let (mut s1, ctx1) = step_sample(1.0, 22.0, 17.0, 24.0);
        s1.ec = Some(ec_reading(&[("cpu@4c", 60.0)]));
        let effects = step.on_sample(&s1, &ctx1);
        assert!(
            effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::Noted(reason) if reason.contains("argmax"))),
            "expected an argmax-handover skip: {effects:?}"
        );
        assert!(
            !effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::Fitted { .. }))
        );
    }

    #[test]
    fn a_rejected_fit_skips_with_noted_reason_and_keeps_defaults() {
        let mut step = StepTest::new();
        step.enter((10.0, 130.0));
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            step.on_sample(&settle_sample(), &ctx);
        }
        // A tiny response (well under the 3C EC identifiability floor)
        // still counts as "loaded" (delta_w clears MIN_STEP_DELTA_W) but
        // fit_fopdt itself rejects the magnitude.
        let mut all = Vec::new();
        for i in 0..STEP_CAP_SAMPLES {
            let t = i as f64;
            let ec_ma = step_response(45.0, 0.02, 24.0, t); // 0.02*24 = 0.48C
            let rpm = step_response(3000.0, 60.0, 24.0, t);
            let s = Sample {
                cpu_pkg_w: 22.0,
                gpu_w: 17.0,
                gpu_w_valid: true,
                gpu_util_pct: 95.0,
                fan1_rpm: rpm,
                fan_valid: true,
                ec: Some(ec_reading(&[("apu@4c", 60.0)])),
                ec_valid: true,
                ..Sample::default()
            };
            let ctx = CalibContext {
                ec_ma: Some(ec_ma),
                ..happy_ctx()
            };
            let effects = step.on_sample(&s, &ctx);
            let concluded = effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::Fitted { .. } | RunnerEffect::Noted(_)));
            all.extend(effects);
            if concluded {
                break;
            }
        }
        assert!(
            all.iter()
                .any(|e| matches!(e, RunnerEffect::Noted(reason) if reason.contains("fit"))),
            "expected a rejected-fit skip: {all:?}"
        );
        assert!(!all.iter().any(|e| matches!(e, RunnerEffect::Fitted { .. })));
    }
}

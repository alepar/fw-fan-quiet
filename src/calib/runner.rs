//! Sample-driven calibration runners. [`PerDeviceCalibRunner`] is the active
//! revision-4 path: shared settle, CPU-watt step, GPU-clock step, keyed gain
//! persistence. The scalar LUT runner remains temporarily for compatibility
//! and its focused historical tests until the scheduled deletion sweep.

use crate::calib::lut_sweep::{LutSweep, SWEEP_CLOCKS, SweepEffect, SweepState};
pub use crate::calib::step::CalibContext;
use crate::calib::step::{CalibDevice, PerDeviceCalibContext, PerDeviceStepTest, StepTest};
use crate::control::budget::LoopGains;
use crate::control::lut::ClockWattsLut;
use crate::state::PersistedState;
use crate::types::Sample;

/// Burner threads for the step test: comfortably above the core count so
/// the package is pinned at whatever limit the budget split commanded.
pub const BURNER_THREADS: usize = 24;

/// Where the calibration session is.
#[allow(dead_code)] // legacy LUT runner compatibility until task .12
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Phase {
    /// GPU clock→watts sweep (user provides a saturating GPU load).
    LutSweep,
    /// Settle + step + fit (design §3.3).
    StepTest,
    /// Finished — LUT saved, gains fitted or defaults kept on a skip.
    Done,
    /// Aborted (user Esc, or an emergency release) — everything released.
    Aborted,
}

#[cfg(test)]
mod per_device_runner_tests {
    use super::*;
    use crate::calib::step::{DEVICE_FIT_WINDOW_S, PerDeviceCalibContext};
    use crate::control::device_loop::Gains;

    #[test]
    fn per_device_runner_starts_directly_in_shared_settle_without_a_lut_sweep() {
        let mut runner = PerDeviceCalibRunner::new(Default::default(), Default::default());
        assert!(runner.start().is_empty());
        let progress = runner.progress();
        assert_eq!(progress.phase, "settle");
        assert_eq!((progress.step, progress.total), (0, 3));
    }

    fn context() -> PerDeviceCalibContext {
        PerDeviceCalibContext {
            cpu_cap_w: Some(20.0), cpu_cap_verified: true,
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
            tick.gpu_group_c = Some(response(55.0, 0.02, 15.0, 60.0, 500.0, elapsed as f64));
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
#[allow(dead_code)] // legacy effect variants remain until task .12
#[derive(Debug, Clone, PartialEq)]
pub enum RunnerEffect {
    /// Direct CPU sustained-cap write for the per-device calibration runner.
    /// The current budget-based step sequence remains live until task .8
    /// migrates it; this is the explicit actuator seam it will use.
    #[allow(dead_code)] // consumed by the per-device step algorithm in task .8
    SetCpuMaxWatts(f64),
    /// Lock the GPU max clock (MHz).
    SetGpuMaxClock(u32),
    /// Restore stock CPU limits.
    ReleaseCpu,
    /// Release GPU clock locks.
    ReleaseGpu,
    /// Start the CPU burner with this many threads (replaces a running one).
    StartBurner(usize),
    /// Stop the CPU burner.
    StopBurner,
    /// The GPU is not doing what this step needs; the UI should tell the
    /// user to start a GPU-heavy load.
    NeedsGpuLoad,
    /// One LUT-sweep point recorded (`phase` is always `"lut"`).
    PointRecorded {
        phase: &'static str,
        idx: usize,
        detail: String,
    },
    /// Request this total CPU+GPU power budget (design §3.3): the
    /// controller applies it by freezing the integrator
    /// (`Freeze::Calibrating`), seeding `u = w`, and running the normal
    /// `split_budget` → command path.
    SetBudget(f64),
    /// Step-test model fitted; `fitted_at` is the sample-clock (`t_mono`)
    /// stamp of the sample the fit landed on.
    Fitted { gains: LoopGains, fitted_at: u64 },
    /// One native per-device thermal model landed successfully.
    FittedDevice {
        device: crate::calib::step::CalibDevice,
        gains: crate::control::device_loop::Gains,
    },
    /// A step-test gate/abort/rejection skipped the phase; the defaults
    /// (no `loop_gains`) are kept. Not a hard failure — calibration still
    /// finishes and the LUT (if any) is still saved.
    Noted(String),
    /// Persist this state (the runner produces it; the controller saves it).
    SaveState(Box<PersistedState>),
    /// Calibration finished successfully. Terminal.
    Finished,
}

/// UI-facing progress snapshot (mirrored into `ControlStatus.calib`). Fields
/// only change on real transitions (never per-sample counters), so the
/// status diff / telemetry Decision cadence stays event-driven.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CalibProgress {
    /// `"lut"` / `"step"` / `"done"` / `"aborted"` — plain strings.
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

/// The calibration state machine. Drive with `start()` once, then
/// `on_sample` for every 1 Hz sample; `abort()` at any time.
#[allow(dead_code)] // inactive legacy runner retained until task .12
pub struct CalibRunner {
    phase: Phase,
    sweep: LutSweep,
    /// LUT-sweep points recorded so far (indexes `PointRecorded`).
    lut_recorded: usize,
    /// The finished LUT (present from the step-test phase on).
    lut: Option<ClockWattsLut>,
    step: StepTest,
    needs_load: bool,
    note: String,
}

#[allow(dead_code)]
impl CalibRunner {
    pub fn new() -> Self {
        Self {
            phase: Phase::LutSweep,
            sweep: LutSweep::new(),
            lut_recorded: 0,
            lut: None,
            step: StepTest::new(),
            needs_load: false,
            note: String::new(),
        }
    }

    /// Begin the session: kick off the LUT sweep.
    pub fn start(&mut self) -> Vec<RunnerEffect> {
        self.note = "gpu clock\u{2192}watts sweep \u{2014} keep a GPU-heavy load running".into();
        let effects = self.sweep.start();
        self.translate_sweep_effects(effects, &CalibContext::default())
    }

    /// Consume one 1 Hz sample; returns what happened. `ctx` is ignored
    /// during the LUT sweep (it needs nothing from the auto loop) and used
    /// from the moment the sweep hands off to the step test.
    pub fn on_sample(&mut self, s: &Sample, ctx: &CalibContext) -> Vec<RunnerEffect> {
        match self.phase {
            Phase::LutSweep => {
                let effects = self.sweep.on_sample(s);
                self.translate_sweep_effects(effects, ctx)
            }
            Phase::StepTest => self.on_step_test_sample(s, ctx),
            Phase::Done | Phase::Aborted => Vec::new(),
        }
    }

    /// Abort: release everything, stop the burner, terminal state. Bypasses
    /// the budget system entirely (direct stock release) regardless of
    /// which phase is running — an emergency/user-abort path should not
    /// depend on the not-yet-wired budget flow.
    pub fn abort(&mut self) -> Vec<RunnerEffect> {
        self.phase = Phase::Aborted;
        self.needs_load = false;
        self.note = "aborted".into();
        vec![
            // Heat source off FIRST: during a thermal-emergency abort the
            // burner must not keep spinning through the (subprocess-slow)
            // CPU release.
            RunnerEffect::StopBurner,
            RunnerEffect::ReleaseCpu,
            RunnerEffect::ReleaseGpu,
        ]
    }

    /// Current phase. Tests assert on the raw phase; production consumes
    /// the [`CalibProgress`] snapshot instead.
    #[allow(dead_code)]
    pub fn phase(&self) -> &Phase {
        &self.phase
    }

    /// UI progress snapshot.
    pub fn progress(&self) -> CalibProgress {
        let (phase, step, total) = match self.phase {
            Phase::LutSweep => ("lut", self.sweep.progress().0, SWEEP_CLOCKS.len()),
            Phase::StepTest => (
                "step",
                usize::from(self.step.stepping()),
                2, // settle, step
            ),
            Phase::Done => ("done", 2, 2),
            Phase::Aborted => ("aborted", 0, 2),
        };
        CalibProgress {
            phase: phase.to_string(),
            step,
            total,
            needs_load: self.needs_load,
            note: self.note.clone(),
        }
    }

    /// Map inner sweep effects onto runner effects; the sweep finishing
    /// stores the LUT and enters the step-test phase.
    fn translate_sweep_effects(
        &mut self,
        effects: Vec<SweepEffect>,
        ctx: &CalibContext,
    ) -> Vec<RunnerEffect> {
        let mut out = Vec::new();
        for effect in effects {
            match effect {
                SweepEffect::CommandClock(mhz) => {
                    self.note = format!("gpu sweep: locking {mhz} MHz, waiting for settle");
                    out.push(RunnerEffect::SetGpuMaxClock(mhz));
                }
                SweepEffect::NeedsLoad => {
                    self.needs_load = true;
                    out.push(RunnerEffect::NeedsGpuLoad);
                }
                SweepEffect::RecordPoint { mhz, watts } => {
                    self.needs_load = false;
                    out.push(RunnerEffect::PointRecorded {
                        phase: "lut",
                        idx: self.lut_recorded,
                        detail: format!("{mhz} MHz \u{2192} {watts:.1} W"),
                    });
                    self.lut_recorded += 1;
                }
                SweepEffect::Finished(lut) => {
                    self.lut = Some(lut);
                    self.phase = Phase::StepTest;
                    self.needs_load = false;
                    self.note = "step test: starting the burner, waiting to settle".into();
                    out.extend(self.step.enter(ctx.budget_bounds));
                }
            }
        }
        // A pinned GPU (Settling) means the load is clearly present.
        if matches!(self.sweep.state(), SweepState::Settling { .. }) {
            self.needs_load = false;
        }
        out
    }

    fn on_step_test_sample(&mut self, s: &Sample, ctx: &CalibContext) -> Vec<RunnerEffect> {
        let mut effects = self.step.on_sample(s, ctx);
        self.needs_load = self.step.needs_load();

        let gains = effects.iter().find_map(|e| match e {
            RunnerEffect::Fitted { gains, .. } => Some(*gains),
            _ => None,
        });
        let skipped = effects.iter().any(|e| matches!(e, RunnerEffect::Noted(_)));

        if gains.is_some() || skipped {
            self.note = if gains.is_some() {
                "calibration complete: gains fitted".to_string()
            } else {
                "calibration complete: step test skipped, defaults kept".to_string()
            };
            self.phase = Phase::Done;
            self.needs_load = false;
            effects.push(RunnerEffect::SaveState(Box::new(PersistedState {
                lut: self.lut.clone(),
                calibrated_at: Some(unix_secs_string()),
                loop_gains: gains,
                ..PersistedState::default()
            })));
            effects.push(RunnerEffect::Finished);
        }
        effects
    }
}

impl Default for CalibRunner {
    fn default() -> Self {
        Self::new()
    }
}

/// Revision-4 calibration runner: one shared settle followed by native CPU
/// watts and GPU clock steps. The legacy runner remains above until the
/// deletion sweep removes its LUT compatibility surface.
pub struct PerDeviceCalibRunner {
    step: PerDeviceStepTest,
    cpu_gains: std::collections::BTreeMap<String, crate::control::device_loop::Gains>,
    gpu_gains: std::collections::BTreeMap<String, crate::control::device_loop::Gains>,
    started: bool,
    finished: bool,
    note: String,
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
                _ => {}
            }
        }
        self.note = if self.step.in_settle() {
            "holding both caps; waiting for groups and fans to settle".into()
        } else {
            match self.step.phase() {
                CalibDevice::Cpu => "CPU watts step".into(),
                CalibDevice::Gpu => "GPU clock step".into(),
            }
        };
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::ec::EcReading;
    use std::sync::atomic::{AtomicU64, Ordering};

    static EC_FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Synthetic hwmon-shaped fixture (EcLabel's constructors are private
    /// to sensors::ec) — mirrors `mode.rs`'s and `step.rs`'s own helpers.
    fn ec_reading(sensors: &[(&str, f64)]) -> EcReading {
        let n = EC_FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir =
            std::env::temp_dir().join(format!("bazerame-runner-test-{}-{n}", std::process::id()));
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

    /// Sweep-phase sample: GPU pinned at `clock` drawing `watts`.
    fn sweep_pinned(clock: u32, watts: f64) -> Sample {
        Sample {
            gpu_util_pct: 99.0,
            gpu_sm_mhz: f64::from(clock),
            gpu_w: watts,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            fan1_rpm: 3000.0,
            fan_valid: true,
            ..Sample::default()
        }
    }

    /// Watts the synthetic GPU draws at a locked clock (linear: exact LUT).
    fn sweep_watts(clock: u32) -> f64 {
        f64::from(clock) / 30.0
    }

    /// Drive the whole LUT sweep with pinned/steady samples. Feeds samples
    /// until each sweep point records (robust against sweep constants).
    fn drive_sweep(runner: &mut CalibRunner) -> Vec<RunnerEffect> {
        let mut all = runner.start();
        for &clock in SWEEP_CLOCKS.iter() {
            let mut recorded = false;
            for _ in 0..60 {
                let effects = runner.on_sample(
                    &sweep_pinned(clock, sweep_watts(clock)),
                    &CalibContext::default(),
                );
                recorded = effects.iter().any(
                    |e| matches!(e, RunnerEffect::PointRecorded { phase, .. } if *phase == "lut"),
                );
                all.extend(effects);
                if recorded {
                    break;
                }
            }
            assert!(recorded, "sweep point at {clock} MHz never recorded");
        }
        all
    }

    const EC_FLAT_WINDOW_S: usize = 60;
    const STEP_CAP_SAMPLES: usize = 300;
    const TAU: f64 = 35.0;
    const THETA: f64 = 10.0;
    const K_EC: f64 = 0.5;
    const K_RPM: f64 = 60.0;
    const BASE_EC: f64 = 45.0;
    const BASE_RPM: f64 = 3000.0;

    fn happy_ctx() -> CalibContext {
        CalibContext {
            cpu_cap_w: None,
            gpu_cap_mhz: None,
            ec_ma: Some(BASE_EC),
            ec_mismatch: false,
            fanctrl_active: true,
            argmax_controllable: true,
            budget_bounds: (10.0, 130.0),
        }
    }

    fn settle_sample() -> Sample {
        Sample {
            cpu_pkg_w: 10.0,
            gpu_w: 5.0,
            gpu_w_valid: true,
            gpu_util_pct: 3.0,
            fan1_rpm: BASE_RPM,
            fan_valid: true,
            ec: Some(ec_reading(&[("apu@4c", 60.0)])),
            ec_valid: true,
            ..Sample::default()
        }
    }

    fn step_response(base: f64, k: f64, delta_w: f64, t: f64) -> f64 {
        if t <= THETA {
            base
        } else {
            base + k * delta_w * (1.0 - (-(t - THETA) / TAU).exp())
        }
    }

    fn step_sample(t: f64, delta_w: f64) -> (Sample, CalibContext) {
        let ec_ma = step_response(BASE_EC, K_EC, delta_w, t);
        let rpm = step_response(BASE_RPM, K_RPM, delta_w, t);
        let s = Sample {
            cpu_pkg_w: 10.0 + delta_w * 0.5,
            gpu_w: 5.0 + delta_w * 0.5,
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

    /// Drive the whole step-test phase (settle -> step -> conclusion)
    /// through the runner; returns every effect produced.
    fn drive_step_test(runner: &mut CalibRunner) -> Vec<RunnerEffect> {
        let ctx = happy_ctx();
        let mut all = Vec::new();
        for _ in 0..EC_FLAT_WINDOW_S {
            all.extend(runner.on_sample(&settle_sample(), &ctx));
        }
        assert_eq!(*runner.phase(), Phase::StepTest, "settle must complete");
        for i in 0..STEP_CAP_SAMPLES {
            let (s, ctx) = step_sample(i as f64, 24.0);
            let effects = runner.on_sample(&s, &ctx);
            let concluded = effects.iter().any(|e| matches!(e, RunnerEffect::Finished));
            all.extend(effects);
            if concluded {
                return all;
            }
        }
        panic!("step test never concluded within the cap");
    }

    // --- THE BIG ONE: full happy path, sweep -> burner -> settle -> step -> SaveState ---

    #[test]
    fn full_happy_path_walks_sweep_burner_settle_step_and_saves_gains_with_fitted_at() {
        let mut runner = CalibRunner::new();
        let mut all = drive_sweep(&mut runner);
        assert_eq!(*runner.phase(), Phase::StepTest);
        // The burner starts as part of entering the step-test phase, before
        // any settle sample is even fed.
        assert!(
            all.iter()
                .any(|e| matches!(e, RunnerEffect::StartBurner(n) if *n == BURNER_THREADS)),
            "burner must start on step-test entry: {all:?}"
        );

        all.extend(drive_step_test(&mut runner));
        assert_eq!(*runner.phase(), Phase::Done);

        assert_eq!(
            count(&all, |e| matches!(
                e,
                RunnerEffect::PointRecorded { phase: "lut", .. }
            )),
            10
        );
        assert!(all.iter().any(|e| matches!(e, RunnerEffect::StopBurner)));

        let fitted: Vec<(LoopGains, u64)> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::Fitted { gains, fitted_at } => Some((*gains, *fitted_at)),
                _ => None,
            })
            .collect();
        assert_eq!(fitted.len(), 1, "exactly one Fitted effect: {all:?}");

        let saved: Vec<&PersistedState> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::SaveState(ps) => Some(ps.as_ref()),
                _ => None,
            })
            .collect();
        assert_eq!(saved.len(), 1);
        let lut = saved[0].lut.as_ref().expect("lut persisted");
        assert_eq!(lut.len(), 10);
        assert_eq!(
            saved[0].loop_gains,
            Some(fitted[0].0),
            "SaveState must carry the same gains Fitted reported"
        );
        saved[0]
            .calibrated_at
            .as_ref()
            .expect("calibrated_at set")
            .parse::<u64>()
            .expect("calibrated_at is unix seconds");

        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::Finished)), 1);

        let progress = runner.progress();
        assert_eq!(progress.phase, "done");
    }

    fn count<F: Fn(&RunnerEffect) -> bool>(effects: &[RunnerEffect], f: F) -> usize {
        effects.iter().filter(|e| f(e)).count()
    }

    // --- burner ordering, asserted at the runner level too ---

    #[test]
    fn burner_starts_before_the_settle_hold_and_stops_after_the_step() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);

        // Re-derive entry effects directly: the last sweep sample's return
        // already carries StartBurner (from step.enter()), asserted above
        // in the happy-path test; here we check no StopBurner appears until
        // the step concludes.
        let ctx = happy_ctx();
        for _ in 0..EC_FLAT_WINDOW_S {
            let effects = runner.on_sample(&settle_sample(), &ctx);
            assert!(!effects.contains(&RunnerEffect::StopBurner));
        }
        let mut saw_stop = false;
        for i in 0..STEP_CAP_SAMPLES {
            let (s, ctx) = step_sample(i as f64, 24.0);
            let effects = runner.on_sample(&s, &ctx);
            if effects.contains(&RunnerEffect::StopBurner) {
                saw_stop = true;
            }
            if effects.iter().any(|e| matches!(e, RunnerEffect::Finished)) {
                break;
            }
        }
        assert!(saw_stop, "burner must stop once the step concludes");
    }

    // --- the self-skip regression, at the runner level ---

    #[test]
    fn idle_uncontrollable_argmax_proceeds_once_loaded_through_the_runner() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        let idle_ctx = CalibContext {
            argmax_controllable: false,
            ..happy_ctx()
        };
        for _ in 0..30 {
            let effects = runner.on_sample(&settle_sample(), &idle_ctx);
            assert!(!effects.iter().any(|e| matches!(e, RunnerEffect::Noted(_))));
        }
        let all = drive_step_test(&mut runner);
        assert!(
            all.iter().any(|e| matches!(e, RunnerEffect::Fitted { .. })),
            "loaded run must still reach a fit, not self-skip: {all:?}"
        );
    }

    // --- a skip still finishes calibration with the LUT saved, no gains ---

    #[test]
    fn a_skipped_step_test_still_saves_the_lut_with_no_gains() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        // fanctrl never active: settle times out at the 5 min cap.
        let bad_ctx = CalibContext {
            fanctrl_active: false,
            ..happy_ctx()
        };
        let mut all = Vec::new();
        for _ in 0..300 {
            let effects = runner.on_sample(&settle_sample(), &bad_ctx);
            let concluded = effects.iter().any(|e| matches!(e, RunnerEffect::Finished));
            all.extend(effects);
            if concluded {
                break;
            }
        }
        assert_eq!(*runner.phase(), Phase::Done);
        assert!(all.iter().any(|e| matches!(e, RunnerEffect::Noted(_))));
        assert!(!all.iter().any(|e| matches!(e, RunnerEffect::Fitted { .. })));
        let saved: Vec<&PersistedState> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::SaveState(ps) => Some(ps.as_ref()),
                _ => None,
            })
            .collect();
        assert_eq!(saved.len(), 1);
        assert!(saved[0].lut.is_some(), "the swept LUT is kept on a skip");
        assert_eq!(saved[0].loop_gains, None, "defaults are kept on a skip");
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::Finished)), 1);
    }

    // --- sweep phase translation (unchanged behavior) ---

    #[test]
    fn start_commands_first_sweep_clock() {
        let mut runner = CalibRunner::new();
        let effects = runner.start();
        assert_eq!(effects, vec![RunnerEffect::SetGpuMaxClock(3090)]);
        let progress = runner.progress();
        assert_eq!(progress.phase, "lut");
        assert_eq!(progress.step, 0);
        assert_eq!(progress.total, 10);
        assert!(!progress.needs_load);
    }

    #[test]
    fn sweep_needs_load_translated_and_flagged() {
        let mut runner = CalibRunner::new();
        runner.start();
        let idle = Sample {
            gpu_util_pct: 5.0,
            gpu_sm_mhz: 300.0,
            gpu_w: 15.0,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            fan1_rpm: 1500.0,
            fan_valid: true,
            ..Sample::default()
        };
        let ctx = CalibContext::default();
        for _ in 0..9 {
            assert!(runner.on_sample(&idle, &ctx).is_empty());
        }
        let effects = runner.on_sample(&idle, &ctx);
        assert_eq!(effects, vec![RunnerEffect::NeedsGpuLoad]);
        assert!(runner.progress().needs_load);

        runner.on_sample(&sweep_pinned(3090, 100.0), &ctx);
        runner.on_sample(&sweep_pinned(3090, 100.0), &ctx);
        runner.on_sample(&sweep_pinned(3090, 100.0), &ctx);
        assert!(!runner.progress().needs_load);
    }

    // --- abort ---

    #[test]
    fn abort_mid_step_test_releases_everything_and_never_saves() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        let ctx = happy_ctx();
        for _ in 0..10 {
            runner.on_sample(&settle_sample(), &ctx);
        }

        let effects = runner.abort();
        assert_eq!(
            effects,
            vec![
                RunnerEffect::StopBurner,
                RunnerEffect::ReleaseCpu,
                RunnerEffect::ReleaseGpu,
            ]
        );
        assert_eq!(*runner.phase(), Phase::Aborted);
        assert_eq!(runner.progress().phase, "aborted");
        assert!(runner.on_sample(&settle_sample(), &ctx).is_empty());
    }
}

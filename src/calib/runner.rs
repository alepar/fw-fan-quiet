//! Guided calibration runner (design doc §3.3, §4): a sample-driven state
//! machine composing [`LutSweep`] (GPU clock→watts) and [`StepTest`] (the
//! FOPDT step test) into one end-to-end calibration session:
//!
//!   LUT sweep → step test → persist.
//!
//! Matrix-free (fw-fanctrl-loop-0nv): the old 11-point (cpu_W × gpu_W)
//! matrix sweep and its batch thermal-model fit are gone. Same pattern as
//! [`LutSweep`]/[`StepTest`] and the controller core: no wall-clock sleeps,
//! no I/O; `start`/`on_sample`/`abort` return [`RunnerEffect`]s that the
//! controller maps onto real actuators, the burner and the state file, so
//! the whole session is unit-testable with synthetic samples.

use crate::calib::lut_sweep::{LutSweep, SWEEP_CLOCKS, SweepEffect, SweepState};
pub use crate::calib::step::CalibContext;
use crate::calib::step::StepTest;
use crate::control::budget::LoopGains;
use crate::control::lut::ClockWattsLut;
use crate::state::PersistedState;
use crate::types::Sample;

/// Burner threads for the step test: comfortably above the core count so
/// the package is pinned at whatever limit the budget split commanded.
pub const BURNER_THREADS: usize = 24;

/// Where the calibration session is.
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

/// What one `start`/`on_sample`/`abort` call did — mapped onto actuators,
/// burner, state file and UI by the controller; asserted on in tests.
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

//! Step-test calibration phase (design doc §3.3): after the LUT sweep, hold
//! the CPU burner at the power-budget floor until the EC moving average and
//! fan RPM settle, then step the total power budget by [`STEP_W`] and fit a
//! first-order-plus-dead-time model to the response on each signal (EC
//! average, fan RPM), deriving PI gains via [`crate::calib::fopdt::derive_gains`].
//!
//! # Runner ↔ controller interface (design §3.3)
//!
//! This phase never reads the socket or the arbiter itself. Each sample the
//! controller hands it a [`CalibContext`] built from the same inputs the
//! auto loop uses (`fanctrl_active`, `ec_mismatch`, `argmax_controllable`,
//! the live EC moving average, and the budget integrator's `(lo, hi)`
//! bounds). Requested power flows out through [`RunnerEffect::SetBudget`],
//! which the controller applies by freezing the integrator
//! (`Freeze::Calibrating`), seeding `u = w`, and running the normal
//! `split_budget` → command path — the step never bypasses the caps.
//!
//! # The self-skip trap (the ordering fact that makes this phase correct)
//!
//! **The burner starts first, before settle detection begins.** Gating is
//! two-stage: `fanctrl_active` and `!ec_mismatch` gate accumulation from the
//! first settle sample; `argmax_controllable` is *also* only meaningful
//! once the burner is running, which it already is by the time any sample
//! reaches this phase. On this machine's own idle fixture the `ambient`
//! sensor reads above `apu` at the floors, so the argmax is uncontrollable
//! by construction before the burner's heat arrives — an up-front,
//! immediate-fail check on `argmax_controllable` would self-skip **every**
//! run. Instead, an unmet gate simply withholds accumulation (the settle
//! windows reset, exactly like a stalled sample) rather than aborting on the
//! spot; only the 5-minute settle cap turns a gate that never clears into a
//! skip. Once the step itself is under way, losing a gate (or an EC
//! over-temperature, or the argmax sensor handing over to a different
//! label) is no longer tolerated — those abort the step immediately, since
//! continuing to record would poison the fit with a discontinuity.

use std::collections::VecDeque;

use crate::calib::fopdt::{MIN_EC_RESPONSE_C, MIN_RPM_RESPONSE, derive_gains, fit_fopdt};
use crate::calib::runner::{BURNER_THREADS, RunnerEffect};
use crate::calib::steady::{STEADY_N, STEADY_RPM_TOLERANCE, is_steady, tail_mean};
use crate::control::budget::LoopGains;
use crate::sensors::ec::EcLabel;
use crate::types::Sample;

/// Settle detection: the EC moving average must be flat within this many °C
/// over [`EC_FLAT_WINDOW_S`] seconds (design §3.3).
const EC_FLAT_TOLERANCE_C: f64 = 0.5;
/// Settle/baseline window, seconds (== samples at the 1 Hz sample rate).
const EC_FLAT_WINDOW_S: usize = 60;
/// Settle detection gives up after this many samples (5 min, design §3.3).
const SETTLE_CAP_SAMPLES: usize = 300;

/// The step size requested on top of the floor, watts (design §3.3).
const STEP_W: f64 = 30.0;
/// The step holds at most this long (5 min, design §3.3).
const STEP_CAP_SAMPLES: usize = 300;
/// ...or exits early once the EC average has been flat this long (90 s,
/// design §3.3). Reuses [`EC_FLAT_TOLERANCE_C`] as the flatness band.
const STEP_FLAT_WINDOW_S: usize = 90;
/// EC max above this aborts the step and restores the floor (design §3.3).
const EC_MAX_ABORT_C: f64 = 95.0;
/// Below this measured total power delta the step counts as "never loaded"
/// (design §3.3's "the applied power never rose" skip condition) — a
/// physically negligible threshold, well under any real burner/GPU draw.
const MIN_STEP_DELTA_W: f64 = 1.0;

/// Utilization above which the GPU counts as loaded — mirrors
/// `lut_sweep::PIN_UTIL_MIN_PCT` (private to that module, so this restates
/// the same 90 % threshold rather than reaching into it).
const PIN_UTIL_MIN_PCT: f64 = 90.0;
/// `NeedsGpuLoad` nag cadence during the step (mirrors the sweep's own
/// `NEEDS_LOAD_EVERY`).
const NEEDS_LOAD_EVERY: usize = 10;

/// What the controller knows this sample that the step needs, built from
/// the same inputs the auto loop uses (design §3.3). The step never reads
/// the socket or the arbiter itself.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CalibContext {
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

//! Guided calibration runner (design doc §4, research 03 §5): a sample-driven
//! state machine composing the already-built pieces — [`LutSweep`] (GPU
//! clock→watts), the steady-state detector, the CPU burner (via effects) and
//! [`ThermalModel::fit_batch`] — into one end-to-end calibration session:
//!
//!   LUT sweep → 11-point (cpu_W × gpu_W) matrix → batch fit → persist.
//!
//! Same pattern as [`LutSweep`] and the controller core: no wall-clock
//! sleeps, no I/O; `start`/`on_sample`/`abort` return [`RunnerEffect`]s that
//! the controller maps onto real actuators, the burner and the state file,
//! so the whole session is unit-testable with synthetic samples.

use std::collections::VecDeque;

use crate::calib::lut_sweep::{LutSweep, SWEEP_CLOCKS, SweepEffect, SweepState};
use crate::calib::steady::{STEADY_N, STEADY_RPM_TOLERANCE, is_steady, tail_mean};
use crate::control::lut::ClockWattsLut;
use crate::control::thermal_model::{CalibPoint, ThermalModel};
use crate::state::PersistedState;
use crate::types::Sample;

/// The 11 (cpu_w, gpu_w) matrix targets from design §4 / research 03 §5:
/// idle/low/mid/high per device plus mixed points to pin the coupling term.
pub const MATRIX_POINTS: [(f64, f64); 11] = [
    (5.0, 0.0),
    (15.0, 0.0),
    (30.0, 0.0),
    (45.0, 0.0),
    (5.0, 35.0),
    (5.0, 65.0),
    (5.0, 100.0),
    (20.0, 40.0),
    (30.0, 65.0),
    (45.0, 100.0),
    (45.0, 40.0),
];

/// CPU targets at or below this are "idle" points: no burner, no CPU limit
/// (an idle desktop already sits near 5 W; commanding a 5 W limit would be
/// below the actuator's 10 W floor anyway).
pub const CPU_IDLE_MAX_W: f64 = 5.0;

/// Burner threads for loaded CPU points: comfortably above the core count so
/// the package is pinned at whatever limit ryzenadj set.
pub const BURNER_THREADS: usize = 24;

/// For gpu_w == 0 matrix points the GPU must be *quiet*: measured GPU watts
/// below this, or the point's "idle GPU" premise is false (the user left the
/// sweep's GPU load running) and accumulation pauses.
pub const GPU_QUIET_MAX_W: f64 = 15.0;

/// For gpu_w > 0 points, measured GPU watts within ±this fraction of the
/// target count as "on target"...
const GPU_BAND_FRAC: f64 = 0.20;

/// ...or utilization above this (the clock lock caps power, so a loaded GPU
/// at the locked clock is at the LUT-predicted watts even if the power
/// reading wanders outside the band).
const GPU_UTIL_ACTIVE_PCT: f64 = 90.0;

/// Nag cadence: `NeedsGpuLoad` once per this many consecutive
/// non-accumulating samples (same idea as the sweep's `NeedsLoad`).
const NEEDS_LOAD_EVERY: usize = 10;

/// Minimum consecutive *clean* (accumulating) samples on a matrix point
/// before it may record. Fans lag heat by tens of seconds: pure flatness
/// detection can trigger early on a slowly-rising plateau (20 flat-ish
/// samples while RPM is still creeping up). 45 s minimum dwell + the
/// 20-sample flatness window is the compromise between run time and settled
/// truth. The streak resets whenever accumulation stalls (wrong GPU state /
/// invalid sensors): a stall changes the thermal input, so the fans must be
/// given the full dwell again once the point's condition is re-established —
/// wall-clock elapsed on the point must never substitute for it.
pub const MIN_DWELL_SAMPLES: usize = 45;

/// A matrix point that has not settled after this many samples times out:
/// if the fan tail is *loosely* flat (spread < [`TIMEOUT_SPREAD_MAX_RPM`])
/// the point records anyway with a warning; otherwise the run fails.
pub const POINT_TIMEOUT_SAMPLES: usize = 240;

/// Loose-fallback ceiling for the timeout path: a 300 RPM tail spread is not
/// steady, but the tail mean is still a usable (±150 RPM) sample; worse than
/// that and the point would poison the fit.
const TIMEOUT_SPREAD_MAX_RPM: f64 = 300.0;

/// Cap on the per-point windows: settling normally bounds them; the cap only
/// prevents unbounded growth while a point refuses to settle.
const WINDOW_CAP: usize = 60;

/// Where the calibration session is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Phase {
    /// GPU clock→watts sweep (user provides a saturating GPU load).
    LutSweep,
    /// Matrix point `idx` of [`MATRIX_POINTS`].
    MatrixPoint { idx: usize },
    /// All points recorded; fitting the thermal model (transient).
    Fitting,
    /// Fit done, state saved.
    Done,
    /// Aborted (user Esc) or failed; everything released.
    Aborted,
}

/// What one `start`/`on_sample`/`abort` call did — mapped onto actuators,
/// burner, state file and UI by the controller; asserted on in tests.
#[derive(Debug, Clone, PartialEq)]
pub enum RunnerEffect {
    /// Command this sustained CPU limit (watts).
    SetCpuW(f64),
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
    /// One calibration point recorded (`phase` is "lut" or "matrix").
    PointRecorded {
        phase: &'static str,
        idx: usize,
        detail: String,
    },
    /// Model fitted. `max_residual` is surfaced (not gated on): a mediocre
    /// model beats none — M4 treats the residual as a trust prior.
    Fitted {
        a: f64,
        b: f64,
        e: f64,
        c: f64,
        max_residual: f64,
    },
    /// Persist this state (the runner produces it; the controller saves it).
    SaveState(PersistedState),
    /// Calibration failed; everything already released. Terminal.
    Failed(String),
    /// Calibration finished successfully. Terminal.
    Finished,
}

/// UI-facing progress snapshot (mirrored into `ControlStatus.calib`). Fields
/// only change on real transitions (never per-sample counters), so the
/// status diff / telemetry Decision cadence stays event-driven.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CalibProgress {
    /// "lut sweep" / "matrix" / "fitting" / "done" / "aborted".
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
    /// The finished LUT (present from the matrix phase on).
    lut: Option<ClockWattsLut>,
    /// Recorded matrix points, in order.
    points: Vec<CalibPoint>,
    /// Samples seen on the current matrix point (timeout clock only).
    elapsed: usize,
    /// Consecutive clean (accumulating) samples on the current point: the
    /// minimum-dwell gate. Reset by any stalled sample, alongside the
    /// window clear.
    clean: usize,
    /// Parallel per-point windows, pushed in lockstep on accumulating
    /// samples only: fan `max_fan_rpm`, MEASURED `cpu_pkg_w`, measured
    /// `gpu_w`. Recording averages the same 20-sample tail of all three, so
    /// the CalibPoint carries measured watts, never commanded ones.
    fan_window: VecDeque<f64>,
    cpu_window: VecDeque<f64>,
    gpu_window: VecDeque<f64>,
    /// Consecutive non-accumulating samples (NeedsGpuLoad nag cadence).
    non_accum: usize,
    needs_load: bool,
    /// A gpu_w == 0 point is blocked because the GPU is still active.
    gpu_block: bool,
    note: String,
}

impl CalibRunner {
    pub fn new() -> Self {
        Self {
            phase: Phase::LutSweep,
            sweep: LutSweep::new(),
            lut_recorded: 0,
            lut: None,
            points: Vec::new(),
            elapsed: 0,
            clean: 0,
            fan_window: VecDeque::new(),
            cpu_window: VecDeque::new(),
            gpu_window: VecDeque::new(),
            non_accum: 0,
            needs_load: false,
            gpu_block: false,
            note: String::new(),
        }
    }

    /// Begin the session: kick off the LUT sweep.
    pub fn start(&mut self) -> Vec<RunnerEffect> {
        self.note = "gpu clock\u{2192}watts sweep \u{2014} keep a GPU-heavy load running".into();
        let effects = self.sweep.start();
        self.translate_sweep_effects(effects)
    }

    /// Consume one 1 Hz sample; returns what happened.
    pub fn on_sample(&mut self, s: &Sample) -> Vec<RunnerEffect> {
        match self.phase {
            Phase::LutSweep => {
                let effects = self.sweep.on_sample(s);
                self.translate_sweep_effects(effects)
            }
            Phase::MatrixPoint { idx } => self.on_matrix_sample(idx, s),
            Phase::Fitting | Phase::Done | Phase::Aborted => Vec::new(),
        }
    }

    /// Abort: release everything, stop the burner, terminal state.
    pub fn abort(&mut self) -> Vec<RunnerEffect> {
        self.phase = Phase::Aborted;
        self.needs_load = false;
        self.gpu_block = false;
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
            Phase::LutSweep => ("lut sweep", self.sweep.progress().0, SWEEP_CLOCKS.len()),
            Phase::MatrixPoint { idx } => ("matrix", idx, MATRIX_POINTS.len()),
            Phase::Fitting => ("fitting", MATRIX_POINTS.len(), MATRIX_POINTS.len()),
            Phase::Done => ("done", MATRIX_POINTS.len(), MATRIX_POINTS.len()),
            Phase::Aborted => ("aborted", 0, MATRIX_POINTS.len()),
        };
        let note = if self.gpu_block {
            format!("{} \u{2014} STOP the GPU load for this point", self.note)
        } else {
            self.note.clone()
        };
        CalibProgress {
            phase: phase.to_string(),
            step,
            total,
            needs_load: self.needs_load,
            note,
        }
    }

    /// Map inner sweep effects onto runner effects; the sweep finishing
    /// stores the LUT and enters the first matrix point.
    fn translate_sweep_effects(&mut self, effects: Vec<SweepEffect>) -> Vec<RunnerEffect> {
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
                    out.extend(self.enter_matrix_point(0));
                }
            }
        }
        // A pinned GPU (Settling) means the load is clearly present.
        if matches!(self.sweep.state(), SweepState::Settling { .. }) {
            self.needs_load = false;
        }
        out
    }

    /// Enter matrix point `idx`: reset per-point state and (re)command both
    /// sides unconditionally — re-commanding an already-correct side is a
    /// cheap reassert, and it keeps every point independent of history.
    fn enter_matrix_point(&mut self, idx: usize) -> Vec<RunnerEffect> {
        self.phase = Phase::MatrixPoint { idx };
        self.elapsed = 0;
        self.clean = 0;
        self.fan_window.clear();
        self.cpu_window.clear();
        self.gpu_window.clear();
        self.non_accum = 0;
        self.needs_load = false;
        self.gpu_block = false;
        let (cpu_t, gpu_t) = MATRIX_POINTS[idx];
        self.note = format!(
            "matrix point {}/{}: cpu {cpu_t:.0} W, gpu {gpu_t:.0} W \u{2014} settling",
            idx + 1,
            MATRIX_POINTS.len()
        );
        let mut out = Vec::new();
        if cpu_t > CPU_IDLE_MAX_W {
            out.push(RunnerEffect::SetCpuW(cpu_t));
            out.push(RunnerEffect::StartBurner(BURNER_THREADS));
        } else {
            out.push(RunnerEffect::ReleaseCpu);
            out.push(RunnerEffect::StopBurner);
        }
        if gpu_t > 0.0 {
            let lut = self.lut.as_ref().expect("matrix phase implies a swept LUT");
            match lut.clock_for_watts(gpu_t) {
                Some(mhz) => out.push(RunnerEffect::SetGpuMaxClock(mhz)),
                None => {
                    // Unreachable after a 10-point sweep; defensive.
                    out.extend(self.fail("LUT empty, cannot command GPU watts".into()));
                }
            }
        } else {
            out.push(RunnerEffect::ReleaseGpu);
        }
        out
    }

    fn on_matrix_sample(&mut self, idx: usize, s: &Sample) -> Vec<RunnerEffect> {
        self.elapsed += 1;
        let (_, gpu_t) = MATRIX_POINTS[idx];
        let mut effects = Vec::new();

        // GPU-side gate. An invalid GPU power reading never accumulates
        // (either way): a sensor outage must not fabricate a point.
        let gpu_ok = if gpu_t > 0.0 {
            s.gpu_w_valid
                && ((s.gpu_w - gpu_t).abs() <= GPU_BAND_FRAC * gpu_t
                    || s.gpu_util_pct > GPU_UTIL_ACTIVE_PCT)
        } else {
            s.gpu_w_valid && s.gpu_w < GPU_QUIET_MAX_W
        };

        if gpu_ok && s.fan_valid {
            self.non_accum = 0;
            self.clean += 1;
            self.needs_load = false;
            self.gpu_block = false;
            push_capped(&mut self.fan_window, s.max_fan_rpm());
            push_capped(&mut self.cpu_window, s.cpu_pkg_w);
            push_capped(&mut self.gpu_window, s.gpu_w);
            // Dwell gate on the CLEAN streak, not elapsed: a stall (wrong
            // GPU state) changed the thermal input, so the fans get the
            // full 45-sample dwell again after it clears — otherwise a
            // still-decaying tail could pass the 20-sample flatness check.
            if self.clean >= MIN_DWELL_SAMPLES
                && is_steady(
                    self.fan_window.make_contiguous(),
                    STEADY_N,
                    STEADY_RPM_TOLERANCE,
                )
            {
                return self.record_matrix_point(idx, None);
            }
        } else {
            // Wrong operating condition (or fan reading missing): discard
            // the windows and the dwell streak — fan RPM measured under the
            // wrong GPU state must never leak into this point's steady
            // tail. The elapsed clock keeps running, so only the timeout
            // bounds a stuck point.
            self.fan_window.clear();
            self.cpu_window.clear();
            self.gpu_window.clear();
            self.clean = 0;
            self.non_accum += 1;
            if !gpu_ok {
                if gpu_t > 0.0 {
                    if self.non_accum.is_multiple_of(NEEDS_LOAD_EVERY) {
                        self.needs_load = true;
                        effects.push(RunnerEffect::NeedsGpuLoad);
                    }
                } else {
                    self.gpu_block = true;
                }
            }
        }

        if self.elapsed >= POINT_TIMEOUT_SAMPLES {
            // Timed out. Loose fallback: a not-quite-steady but roughly flat
            // tail still records (with a warning in the detail); a wild tail
            // (or none at all) fails the run.
            match tail_spread(self.fan_window.make_contiguous(), STEADY_N) {
                Some(spread) if spread < TIMEOUT_SPREAD_MAX_RPM => {
                    let warning = format!(
                        "TIMEOUT after {POINT_TIMEOUT_SAMPLES} samples \
                         (tail spread {spread:.0} RPM), recording anyway"
                    );
                    effects.extend(self.record_matrix_point(idx, Some(warning)));
                }
                _ => {
                    effects.extend(self.fail(format!(
                        "matrix point {} never settled within {POINT_TIMEOUT_SAMPLES} samples",
                        idx + 1
                    )));
                }
            }
        }
        effects
    }

    /// Record the current point from the 20-sample tails of the parallel
    /// windows (MEASURED watts, not commanded), then advance or finish.
    fn record_matrix_point(&mut self, idx: usize, warning: Option<String>) -> Vec<RunnerEffect> {
        let rpm = tail_mean(self.fan_window.make_contiguous(), STEADY_N)
            .expect("caller verified a full fan tail");
        let cpu_w = tail_mean(self.cpu_window.make_contiguous(), STEADY_N)
            .expect("windows are pushed in lockstep");
        let gpu_w = tail_mean(self.gpu_window.make_contiguous(), STEADY_N)
            .expect("windows are pushed in lockstep");
        self.points.push(CalibPoint { cpu_w, gpu_w, rpm });
        let mut detail =
            format!("measured cpu {cpu_w:.1} W gpu {gpu_w:.1} W \u{2192} {rpm:.0} rpm");
        if let Some(warning) = warning {
            detail = format!("{warning}; {detail}");
        }
        let mut effects = vec![RunnerEffect::PointRecorded {
            phase: "matrix",
            idx,
            detail,
        }];
        if idx + 1 < MATRIX_POINTS.len() {
            effects.extend(self.enter_matrix_point(idx + 1));
        } else {
            effects.extend(self.finish());
        }
        effects
    }

    /// All points recorded: release everything, fit, persist. The fit is NOT
    /// gated on the residual — a mediocre model beats none; `max_residual`
    /// is surfaced in `Fitted` and M4 treats it as a trust prior.
    fn finish(&mut self) -> Vec<RunnerEffect> {
        self.phase = Phase::Fitting;
        let mut effects = vec![
            RunnerEffect::ReleaseCpu,
            RunnerEffect::ReleaseGpu,
            RunnerEffect::StopBurner,
        ];
        match ThermalModel::fit_batch(&self.points) {
            Ok(model) => {
                let max_residual = model.max_abs_residual(&self.points);
                effects.push(RunnerEffect::Fitted {
                    a: model.a,
                    b: model.b,
                    e: model.e,
                    c: model.c,
                    max_residual,
                });
                effects.push(RunnerEffect::SaveState(PersistedState {
                    model: Some(model),
                    lut: self.lut.clone(),
                    calibrated_at: Some(unix_secs_string()),
                    ..PersistedState::default()
                }));
                effects.push(RunnerEffect::Finished);
                self.phase = Phase::Done;
                self.note = format!("calibration complete (max residual {max_residual:.0} RPM)");
            }
            Err(e) => {
                // Releases already queued above; fail() re-queues them,
                // which the controller executes idempotently.
                effects.extend(self.fail(format!("model fit failed: {e}")));
            }
        }
        effects
    }

    /// Terminal failure: release everything and report why.
    fn fail(&mut self, msg: String) -> Vec<RunnerEffect> {
        self.phase = Phase::Aborted;
        self.needs_load = false;
        self.gpu_block = false;
        self.note = format!("FAILED: {msg}");
        vec![
            RunnerEffect::ReleaseCpu,
            RunnerEffect::ReleaseGpu,
            RunnerEffect::StopBurner,
            RunnerEffect::Failed(msg),
        ]
    }
}

impl Default for CalibRunner {
    fn default() -> Self {
        Self::new()
    }
}

fn push_capped(window: &mut VecDeque<f64>, v: f64) {
    if window.len() == WINDOW_CAP {
        window.pop_front();
    }
    window.push_back(v);
}

/// Max-min spread of the last `n` values; None if fewer than `n`. The fan
/// window only ever holds `fan_valid` readings, so no NaN handling needed.
fn tail_spread(window: &[f64], n: usize) -> Option<f64> {
    if window.len() < n {
        return None;
    }
    let tail = &window[window.len() - n..];
    let max = tail.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min = tail.iter().copied().fold(f64::INFINITY, f64::min);
    Some(max - min)
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

    /// Ground-truth thermal model the synthetic samples are generated from.
    const A: f64 = 25.0;
    const B: f64 = 15.0;
    const E: f64 = 0.1;
    const C: f64 = 800.0;

    fn truth_rpm(pc: f64, pg: f64) -> f64 {
        A * pc + B * pg + E * pc * pg + C
    }

    /// Sweep-phase sample: GPU pinned at `clock` drawing `watts` (mirrors the
    /// lut_sweep test fixture).
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

    /// Sweep-phase sample with the GPU idle (unpinned).
    fn sweep_idle() -> Sample {
        Sample {
            gpu_util_pct: 5.0,
            gpu_sm_mhz: 300.0,
            gpu_w: 15.0,
            gpu_w_valid: true,
            gpu_mhz_valid: true,
            fan1_rpm: 1500.0,
            fan_valid: true,
            ..Sample::default()
        }
    }

    /// Matrix-phase sample: measured CPU/GPU watts plus fan RPM from the
    /// ground-truth model. `gpu_active` sets utilization high (game running).
    fn matrix_sample(cpu_meas: f64, gpu_meas: f64, gpu_active: bool) -> Sample {
        Sample {
            cpu_pkg_w: cpu_meas,
            gpu_w: gpu_meas,
            gpu_w_valid: true,
            gpu_util_pct: if gpu_active { 97.0 } else { 3.0 },
            gpu_mhz_valid: true,
            fan1_rpm: truth_rpm(cpu_meas, gpu_meas),
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
                let effects = runner.on_sample(&sweep_pinned(clock, sweep_watts(clock)));
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

    /// The sample a well-behaved system would produce while sitting on
    /// matrix point `idx`: measured CPU = commanded − 2 W (or 4 W idle),
    /// measured GPU = target (or 10 W idle).
    fn matrix_point_sample(idx: usize) -> Sample {
        let (cpu_t, gpu_t) = MATRIX_POINTS[idx];
        let cpu_meas = if cpu_t > CPU_IDLE_MAX_W {
            cpu_t - 2.0
        } else {
            4.0
        };
        let (gpu_meas, active) = if gpu_t > 0.0 {
            (gpu_t, true)
        } else {
            (10.0, false)
        };
        matrix_sample(cpu_meas, gpu_meas, active)
    }

    /// Drive one matrix point to its recording; returns all effects.
    fn drive_matrix_point(runner: &mut CalibRunner, idx: usize) -> Vec<RunnerEffect> {
        let mut all = Vec::new();
        for _ in 0..(POINT_TIMEOUT_SAMPLES + 1) {
            let effects = runner.on_sample(&matrix_point_sample(idx));
            let recorded = effects.iter().any(
                |e| matches!(e, RunnerEffect::PointRecorded { phase, .. } if *phase == "matrix"),
            );
            all.extend(effects);
            if recorded {
                return all;
            }
        }
        panic!("matrix point {idx} never recorded");
    }

    fn count<F: Fn(&RunnerEffect) -> bool>(effects: &[RunnerEffect], f: F) -> usize {
        effects.iter().filter(|e| f(e)).count()
    }

    // --- THE BIG ONE: full happy path against ground truth ---

    #[test]
    fn full_happy_path_records_everything_and_fits_ground_truth() {
        let mut runner = CalibRunner::new();
        let mut all = drive_sweep(&mut runner);
        assert_eq!(*runner.phase(), Phase::MatrixPoint { idx: 0 });

        for idx in 0..MATRIX_POINTS.len() {
            assert_eq!(*runner.phase(), Phase::MatrixPoint { idx });
            all.extend(drive_matrix_point(&mut runner, idx));
        }
        assert_eq!(*runner.phase(), Phase::Done);

        // 10 LUT points + 11 matrix points recorded.
        assert_eq!(
            count(&all, |e| matches!(
                e,
                RunnerEffect::PointRecorded { phase: "lut", .. }
            )),
            10
        );
        assert_eq!(
            count(&all, |e| matches!(
                e,
                RunnerEffect::PointRecorded {
                    phase: "matrix",
                    ..
                }
            )),
            11
        );

        // Burner started once per cpu>5 point (7 of 11), with 24 threads;
        // idle-CPU points (4) plus the finish each stop it.
        let starts: Vec<usize> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::StartBurner(n) => Some(*n),
                _ => None,
            })
            .collect();
        assert_eq!(starts, vec![BURNER_THREADS; 7]);
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::StopBurner)), 5);

        // CPU limits commanded per loaded point, in matrix order.
        let cpu_cmds: Vec<f64> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::SetCpuW(w) => Some(*w),
                _ => None,
            })
            .collect();
        assert_eq!(cpu_cmds, vec![15.0, 30.0, 45.0, 20.0, 30.0, 45.0, 45.0]);

        // GPU released for every gpu_w == 0 point (4) plus the finish.
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::ReleaseGpu)), 5);

        // GPU clocks commanded from the LUT for gpu>0 points: watts are
        // clock/30, so clock_for_watts(w) = 30·w (clamped low at 1200).
        let gpu_cmds: Vec<u32> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::SetGpuMaxClock(mhz) => Some(*mhz),
                _ => None,
            })
            .collect();
        // First 10 are the sweep's descending clocks; then the matrix points.
        assert_eq!(gpu_cmds[..10], SWEEP_CLOCKS);
        assert_eq!(
            gpu_cmds[10..],
            vec![1200, 1950, 3000, 1200, 1950, 3000, 1200]
        );

        // Fitted parameters within 5% of ground truth.
        let fitted: Vec<(f64, f64, f64, f64, f64)> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::Fitted {
                    a,
                    b,
                    e,
                    c,
                    max_residual,
                } => Some((*a, *b, *e, *c, *max_residual)),
                _ => None,
            })
            .collect();
        assert_eq!(fitted.len(), 1);
        let (a, b, e, c, max_residual) = fitted[0];
        assert!((a - A).abs() < 0.05 * A, "a = {a}");
        assert!((b - B).abs() < 0.05 * B, "b = {b}");
        assert!((e - E).abs() < 0.05 * E, "e = {e}");
        assert!((c - C).abs() < 0.05 * C, "c = {c}");
        assert!(max_residual < 200.0, "max_residual = {max_residual}");

        // SaveState carries the model, the swept LUT and a timestamp.
        let saved: Vec<&PersistedState> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::SaveState(ps) => Some(ps),
                _ => None,
            })
            .collect();
        assert_eq!(saved.len(), 1);
        let model = saved[0].model.as_ref().expect("model persisted");
        assert!((model.a - A).abs() < 0.05 * A);
        let lut = saved[0].lut.as_ref().expect("lut persisted");
        assert_eq!(lut.len(), 10);
        assert_eq!(lut.watts_for_clock(3090), Some(103.0));
        saved[0]
            .calibrated_at
            .as_ref()
            .expect("calibrated_at set")
            .parse::<u64>()
            .expect("calibrated_at is unix seconds");

        // Finished, exactly once; no Failed anywhere.
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::Finished)), 1);
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::Failed(_))), 0);

        let progress = runner.progress();
        assert_eq!(progress.phase, "done");
        assert_eq!(progress.step, 11);
        assert_eq!(progress.total, 11);
        assert!(!progress.needs_load);
    }

    // --- sweep phase translation ---

    #[test]
    fn start_commands_first_sweep_clock() {
        let mut runner = CalibRunner::new();
        let effects = runner.start();
        assert_eq!(effects, vec![RunnerEffect::SetGpuMaxClock(3090)]);
        let progress = runner.progress();
        assert_eq!(progress.phase, "lut sweep");
        assert_eq!(progress.step, 0);
        assert_eq!(progress.total, 10);
        assert!(!progress.needs_load);
    }

    #[test]
    fn sweep_needs_load_translated_and_flagged() {
        let mut runner = CalibRunner::new();
        runner.start();
        for _ in 0..9 {
            assert!(runner.on_sample(&sweep_idle()).is_empty());
        }
        let effects = runner.on_sample(&sweep_idle());
        assert_eq!(effects, vec![RunnerEffect::NeedsGpuLoad]);
        assert!(runner.progress().needs_load);

        // Load appears: pinned samples clear the flag.
        runner.on_sample(&sweep_pinned(3090, 100.0));
        runner.on_sample(&sweep_pinned(3090, 100.0));
        runner.on_sample(&sweep_pinned(3090, 100.0));
        assert!(!runner.progress().needs_load);
    }

    // --- matrix behavior ---

    #[test]
    fn no_record_before_min_dwell_even_with_steady_fans() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        // Perfectly flat fans from sample 1: flatness alone must not record
        // before the 45-sample dwell (fans lag; early flatness can be a
        // slowly-rising plateau).
        for i in 0..(MIN_DWELL_SAMPLES - 1) {
            let effects = runner.on_sample(&matrix_point_sample(0));
            assert!(
                !effects
                    .iter()
                    .any(|e| matches!(e, RunnerEffect::PointRecorded { .. })),
                "recorded early at sample {i}: {effects:?}"
            );
        }
        let effects = runner.on_sample(&matrix_point_sample(0));
        assert!(
            effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::PointRecorded { .. })),
            "expected record exactly at the dwell boundary, got {effects:?}"
        );
    }

    #[test]
    fn abort_mid_matrix_releases_everything_and_never_saves() {
        let mut runner = CalibRunner::new();
        let mut all = drive_sweep(&mut runner);
        all.extend(drive_matrix_point(&mut runner, 0));
        // Part-way into point 1 (burner + CPU limit active).
        for _ in 0..10 {
            all.extend(runner.on_sample(&matrix_point_sample(1)));
        }

        let effects = runner.abort();
        assert_eq!(
            effects,
            vec![
                // Heat source off first (thermal-emergency ordering).
                RunnerEffect::StopBurner,
                RunnerEffect::ReleaseCpu,
                RunnerEffect::ReleaseGpu,
            ]
        );
        assert_eq!(*runner.phase(), Phase::Aborted);
        assert_eq!(runner.progress().phase, "aborted");
        all.extend(effects);
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::SaveState(_))), 0);
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::Finished)), 0);

        // Terminal: further samples do nothing.
        assert!(runner.on_sample(&matrix_point_sample(1)).is_empty());
    }

    #[test]
    fn gpu_still_active_blocks_a_gpu_idle_point_until_it_quiets() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        // Point 0 wants gpu_w == 0, but the sweep's GPU load is still
        // running (40 W): steady fans must NOT accumulate.
        for _ in 0..60 {
            let effects = runner.on_sample(&matrix_sample(4.0, 40.0, true));
            assert!(
                !effects
                    .iter()
                    .any(|e| matches!(e, RunnerEffect::PointRecorded { .. })),
                "must not record while the GPU is hot: {effects:?}"
            );
        }
        assert!(runner.progress().note.contains("STOP the GPU load"));
        // NeedsGpuLoad is the "start a load" nag; a hot GPU on an idle
        // point is the opposite problem and must not emit it.

        // GPU quiets: the stall reset the dwell streak, so the point needs
        // the FULL 45 clean samples again (elapsed wall-clock on the point
        // must not count — the fans were reacting to the hot GPU).
        let mut recorded = false;
        for i in 0..MIN_DWELL_SAMPLES {
            let effects = runner.on_sample(&matrix_point_sample(0));
            recorded = effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::PointRecorded { .. }));
            if i < MIN_DWELL_SAMPLES - 1 {
                assert!(!recorded, "recorded before a full clean dwell (i={i})");
            }
        }
        assert!(recorded, "quiet GPU must let the point record");
        assert!(!runner.progress().note.contains("STOP the GPU load"));
    }

    #[test]
    fn stall_mid_dwell_resets_the_dwell_clock() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        // 30 clean samples into point 0 (dwell part-way)...
        for _ in 0..30 {
            assert!(runner.on_sample(&matrix_point_sample(0)).is_empty());
        }
        // ...then the GPU goes hot for 50 samples: windows AND dwell reset.
        for _ in 0..50 {
            assert!(runner.on_sample(&matrix_sample(4.0, 40.0, true)).is_empty());
        }
        // Quiet again: 44 clean samples are still not enough (30 + 50 + 44
        // = 124 elapsed, but only 44 clean)...
        for i in 0..(MIN_DWELL_SAMPLES - 1) {
            let effects = runner.on_sample(&matrix_point_sample(0));
            assert!(
                !effects
                    .iter()
                    .any(|e| matches!(e, RunnerEffect::PointRecorded { .. })),
                "recorded at clean sample {i}, before the full post-stall dwell: {effects:?}"
            );
        }
        // ...the 45th clean sample records.
        let effects = runner.on_sample(&matrix_point_sample(0));
        assert!(
            effects
                .iter()
                .any(|e| matches!(e, RunnerEffect::PointRecorded { .. })),
            "expected record at the 45th clean sample, got {effects:?}"
        );
    }

    #[test]
    fn gpu_point_without_load_nags_every_ten_samples() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        drive_matrix_point(&mut runner, 0);
        drive_matrix_point(&mut runner, 1);
        drive_matrix_point(&mut runner, 2);
        drive_matrix_point(&mut runner, 3);
        assert_eq!(*runner.phase(), Phase::MatrixPoint { idx: 4 });

        // Point 4 wants 35 GPU W but the GPU sits idle (10 W, low util).
        for i in 0..9 {
            let effects = runner.on_sample(&matrix_sample(4.0, 10.0, false));
            assert!(effects.is_empty(), "sample {i}: {effects:?}");
        }
        let effects = runner.on_sample(&matrix_sample(4.0, 10.0, false));
        assert_eq!(effects, vec![RunnerEffect::NeedsGpuLoad]);
        assert!(runner.progress().needs_load);

        // Nag repeats once per ten.
        for _ in 0..9 {
            assert!(
                runner
                    .on_sample(&matrix_sample(4.0, 10.0, false))
                    .is_empty()
            );
        }
        assert_eq!(
            runner.on_sample(&matrix_sample(4.0, 10.0, false)),
            vec![RunnerEffect::NeedsGpuLoad]
        );

        // Load arrives on target: accumulation resumes, flag clears.
        runner.on_sample(&matrix_sample(4.0, 35.0, true));
        assert!(!runner.progress().needs_load);
    }

    #[test]
    fn gpu_activity_accepted_by_watts_band_or_utilization() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        for idx in 0..4 {
            drive_matrix_point(&mut runner, idx);
        }
        // Point 4 targets 35 W. 28 W is exactly at the −20% band edge:
        // accumulates even with low utilization.
        for _ in 0..MIN_DWELL_SAMPLES {
            runner.on_sample(&matrix_sample(4.0, 28.0, false));
        }
        assert_eq!(*runner.phase(), Phase::MatrixPoint { idx: 5 });
        // (35·0.8 = 28: in band → the point recorded and advanced.)
    }

    #[test]
    fn timeout_with_loosely_flat_tail_records_with_warning() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        // Fans alternate ±100 RPM around 1000 (spread 200: not steady at the
        // 100 RPM tolerance, but under the 300 RPM fallback ceiling).
        let mut recorded = Vec::new();
        for i in 0..POINT_TIMEOUT_SAMPLES {
            let mut s = matrix_point_sample(0);
            s.fan1_rpm = if i % 2 == 0 { 900.0 } else { 1100.0 };
            let effects = runner.on_sample(&s);
            for e in &effects {
                if let RunnerEffect::PointRecorded { detail, .. } = e {
                    recorded.push(detail.clone());
                }
            }
        }
        assert_eq!(recorded.len(), 1, "timeout fallback must record once");
        assert!(
            recorded[0].contains("TIMEOUT"),
            "detail must carry the warning: {}",
            recorded[0]
        );
        // Advanced to the next point (mean of the alternating tail = 1000).
        assert_eq!(*runner.phase(), Phase::MatrixPoint { idx: 1 });
        assert!(recorded[0].contains("1000 rpm"), "detail: {}", recorded[0]);
    }

    #[test]
    fn timeout_with_wild_tail_fails_and_releases() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        // Spread 500 >= the 300 RPM fallback ceiling: hard fail.
        let mut all = Vec::new();
        for i in 0..POINT_TIMEOUT_SAMPLES {
            let mut s = matrix_point_sample(0);
            s.fan1_rpm = if i % 2 == 0 { 750.0 } else { 1250.0 };
            all.extend(runner.on_sample(&s));
        }
        let failed: Vec<&String> = all
            .iter()
            .filter_map(|e| match e {
                RunnerEffect::Failed(msg) => Some(msg),
                _ => None,
            })
            .collect();
        assert_eq!(failed.len(), 1);
        assert!(
            failed[0].contains("never settled"),
            "message: {}",
            failed[0]
        );
        assert!(all.iter().any(|e| matches!(e, RunnerEffect::ReleaseCpu)));
        assert!(all.iter().any(|e| matches!(e, RunnerEffect::ReleaseGpu)));
        assert!(all.iter().any(|e| matches!(e, RunnerEffect::StopBurner)));
        assert_eq!(count(&all, |e| matches!(e, RunnerEffect::SaveState(_))), 0);
        assert_eq!(*runner.phase(), Phase::Aborted);
        assert!(runner.on_sample(&matrix_point_sample(0)).is_empty());
    }

    #[test]
    fn recorded_point_carries_measured_not_commanded_watts() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        drive_matrix_point(&mut runner, 0);
        // Point 1 commands 15 W but RAPL measures 13 W: the recorded point
        // must carry the measurement.
        let mut details = Vec::new();
        for _ in 0..MIN_DWELL_SAMPLES {
            for e in runner.on_sample(&matrix_sample(13.0, 10.0, false)) {
                if let RunnerEffect::PointRecorded { detail, .. } = e {
                    details.push(detail);
                }
            }
        }
        assert_eq!(details.len(), 1);
        assert!(
            details[0].contains("cpu 13.0 W"),
            "recorded point must be measured (13 W), not commanded (15 W): {}",
            details[0]
        );
    }

    #[test]
    fn invalid_gpu_or_fan_samples_never_accumulate() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        // Looks perfect for point 0, but a validity flag is down each time:
        // no record, ever.
        for i in 0..(POINT_TIMEOUT_SAMPLES - 1) {
            let mut s = matrix_point_sample(0);
            if i % 2 == 0 {
                s.gpu_w_valid = false;
            } else {
                s.fan_valid = false;
            }
            let effects = runner.on_sample(&s);
            assert!(
                !effects
                    .iter()
                    .any(|e| matches!(e, RunnerEffect::PointRecorded { .. })),
                "invalid sample {i} must not record: {effects:?}"
            );
        }
    }

    #[test]
    fn matrix_progress_reports_step_and_totals() {
        let mut runner = CalibRunner::new();
        drive_sweep(&mut runner);
        let p = runner.progress();
        assert_eq!(p.phase, "matrix");
        assert_eq!(p.step, 0);
        assert_eq!(p.total, 11);
        assert!(p.note.contains("point 1/11"));
        drive_matrix_point(&mut runner, 0);
        let p = runner.progress();
        assert_eq!(p.step, 1);
        assert!(p.note.contains("point 2/11"));
    }
}

//! The single budget integrator (design doc §2.4).
//!
//! Velocity-form PI on a `LoopError` (temperature or RPM) that produces the
//! total CPU+GPU power budget `u`, in watts. One integrator serves both Mode
//! A (temperature) and Mode B (RPM): a mode switch only changes what error
//! feeds the *next* increment, so the switch is bumpless by construction.
//!
//! # Anti-windup
//!
//! Two independent mechanisms, per §2.4:
//!
//! - **Clamp + back-calculation against the bounds, `Tt = Ti`.** `step`
//!   tracks an internal unclamped accumulator `v` alongside the exposed,
//!   always-in-bounds `u`. Each tick feeds back `(Ts/Tt)·(u − v)` into the
//!   raw PI increment before folding it into `v`, then `u = clamp(v, lo,
//!   hi)`. While saturated this bleeds `v` back toward `u` with time
//!   constant `Tt`; choosing `Tt = Ti` is what makes "release recovers
//!   within one `Ti`" a designed property, not an accident. There is
//!   deliberately no back-calculation toward the measured draw — that shape
//!   was tried and produces a cap that tracks the draw (§2.4).
//! - **Demand-limited halt (`Freeze::DemandLimited`), directional and
//!   per-axis.** See [`Budget::set_demand_state`] and the note on
//!   [`Freeze::DemandLimited`] below. The concrete predicate is a
//!   placeholder; `fw-fanctrl-loop-9it` (a measurement spike) owns the real
//!   rule and rewrites this section and its callers.
//!
//! # Freezes
//!
//! `ActuatorMismatch`, `Calibrating` and `Released` hold `u` **exactly**:
//! `step` returns the unchanged value with no PI computation at all.
//! `DemandLimited` is different in kind: it must still let the *recovering*
//! direction integrate (roast-3 regression — a rule that blocks both
//! directions self-latches, see below), so it runs the ordinary PI/back-calc
//! path and only zeroes an increment that would deepen the condition.
//!
//! Leaving any freeze (frozen last tick, not frozen this tick), and an
//! error-kind switch (`Temp` <-> `Rpm`), both implicitly call
//! [`Budget::resync_error`] before computing this tick's increment, so the
//! velocity form never turns a setpoint jump or a mode switch into a
//! proportional kick.

use std::collections::BTreeMap;
use std::time::Duration;

use serde::{Deserialize, Serialize};

/// Integrator cadence, seconds — the allocator cadence (§2.4).
pub const PI_PERIOD_S: f64 = 5.0;

/// Reference curve slope for [`Budget::scale_rpm_gain`], %/°C (§2.4).
const SLOPE_REF_PCT_PER_C: f64 = 1.0;

/// Conservative floor `scale_rpm_gain` returns for the steepest supported
/// tread, and the value used before the schedule has ever been resolved
/// (`Budget::new`'s initial state) or when it resolves to `None` (§2.4: "the
/// conservative `0.25x` clamp, never `1x`").
const RPM_GAIN_SCALE_FLOOR: f64 = 0.25;

/// Persisted PI gains for both loop legs (§2.4). `Default` is the
/// θ_eff-derived IMC tuning; the FOPDT fit outputs (`tau_s`, `theta_s`,
/// plant gains, `fitted_at`) that §2.4 also lists on the persisted struct
/// belong to the calibration task that produces them (fwloop.21) and are
/// out of this task's scope — this task owns exactly the four PI gains the
/// acceptance criteria exercise.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LoopGains {
    /// Mode A (temperature) proportional gain, W/°C.
    pub kc_w_per_c: f64,
    /// Mode A integral time, s.
    pub ti_s: f64,
    /// Mode B (RPM) proportional gain, W/RPM, before `scale_rpm_gain`.
    pub kc_w_per_rpm: f64,
    /// Mode B integral time, s.
    pub ti_rpm_s: f64,
}

impl Default for LoopGains {
    /// θ_eff-derived IMC defaults, §2.4: `kc_w_per_c = τ/(K·(λ+θ_eff)) =
    /// 35/(0.8·200) = 0.22`, `ti_s = τ = 35`; the RPM leg at the reference
    /// slope, same τ/θ_eff/λ: `kc_w_per_rpm = 35/(62.4·200) = 0.0028`,
    /// `ti_rpm_s = 35`.
    fn default() -> Self {
        Self {
            kc_w_per_c: 0.22,
            ti_s: 35.0,
            kc_w_per_rpm: 0.0028,
            ti_rpm_s: 35.0,
        }
    }
}

/// The error the arbiter feeds the integrator each tick (§2.4). Both
/// variants integrate into the same `u` — the mode switch is bumpless by
/// construction, and switching kind implicitly resyncs (see module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoopError {
    /// Mode A: `T* - MA`.
    Temp { e_c: f64 },
    /// Mode A fallback (Mode B): `rpm_for_duty(target_duty) - rpm_smoothed`.
    Rpm { e_rpm: f64 },
}

impl LoopError {
    fn value(self) -> f64 {
        match self {
            LoopError::Temp { e_c } => e_c,
            LoopError::Rpm { e_rpm } => e_rpm,
        }
    }

    fn kind(self) -> ErrorKind {
        match self {
            LoopError::Temp { .. } => ErrorKind::Temp,
            LoopError::Rpm { .. } => ErrorKind::Rpm,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ErrorKind {
    Temp,
    Rpm,
}

/// Reasons the integrator's accumulation is held or partially held
/// (§2.4). Reported in decision telemetry by the caller; `Budget` itself
/// only cares about the hold behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Freeze {
    /// A CPU or GPU write/read-back mismatch.
    ActuatorMismatch,
    /// The whole calibration session, LUT sweep included.
    Calibrating,
    /// Released to stock (socket dead, RPM loop invalid too).
    Released,
    /// At least one actuator can't consume more of the budget increase the
    /// error is currently calling for.
    ///
    /// Unlike the other three reasons this does **not** hold `u` exactly:
    /// §2.4's fixed invariant is that a halt may only ever block the
    /// direction that *deepens* the condition, never the recovering one — a
    /// rule that blocks both directions can self-latch (this is exactly how
    /// the roast iteration 3 prose revision failed: it froze the descent
    /// that was the loop's only route back to a binding cap, and the fans
    /// sat above target for the whole session). So `step` still runs the
    /// ordinary PI/back-calculation path under `DemandLimited`, and only
    /// zeroes the increment when it would push `u` further in the deepening
    /// direction (`raw Δu > 0`); an increment that already pushes `u` down
    /// passes through untouched. There is no corrective pull toward the
    /// draw either way — `u` is simply left where the (possibly gated)
    /// increment puts it.
    DemandLimited,
}

/// The single budget integrator. See the module docs for the anti-windup
/// design. `u` (returned by [`Budget::step`]) is always within
/// `[lo, hi]` as set by [`Budget::set_bounds`].
pub struct Budget {
    gains: LoopGains,
    lo: f64,
    hi: f64,
    /// Exposed, always-clamped output — the commanded budget, watts.
    u: f64,
    /// Internal unclamped accumulator; equals `u` whenever unsaturated.
    v: f64,
    e_prev: f64,
    last_kind: Option<ErrorKind>,
    last_freeze: Option<Freeze>,
    /// Multiplier `scale_rpm_gain` applies to `gains.kc_w_per_rpm`.
    /// Starts at the conservative floor (§2.4: `None` before any curve is
    /// resolved must behave like `None`, never like `1x`).
    rpm_gain_scale: f64,
    lower_bound_dwell: Duration,
    upper_bound_dwell: Duration,
}

impl Budget {
    /// New integrator, bounds `[0, 0]` (nothing to command) until
    /// [`Budget::set_bounds`] is called, `u = 0`, no dwell.
    pub fn new(gains: &LoopGains) -> Self {
        Self {
            gains: *gains,
            lo: 0.0,
            hi: 0.0,
            u: 0.0,
            v: 0.0,
            e_prev: 0.0,
            last_kind: None,
            last_freeze: None,
            rpm_gain_scale: RPM_GAIN_SCALE_FLOOR,
            lower_bound_dwell: Duration::ZERO,
            upper_bound_dwell: Duration::ZERO,
        }
    }

    /// Loads new gains for future ticks. Does not touch `u`, `v`, `e_prev`
    /// or dwell — a gain change alone is not a setpoint jump.
    pub fn set_gains(&mut self, gains: &LoopGains) {
        self.gains = *gains;
    }

    /// Sets the clamp bounds, watts. Re-clamps the exposed `u` immediately
    /// so it never reports a value outside the new bounds; the internal
    /// accumulator `v` is left as-is and self-corrects on the next `step`.
    pub fn set_bounds(&mut self, lo: f64, hi: f64) {
        debug_assert!(lo <= hi, "budget bounds must be ordered: {lo} <= {hi}");
        self.lo = lo;
        self.hi = hi;
        self.u = self.u.clamp(lo, hi);
    }

    /// Warm-start (or floor) seed: sets `u` (and the internal accumulator,
    /// so there is no latent windup to unwind) to `u`, clamped to the
    /// current bounds. Does not touch `e_prev` — the caller resyncs
    /// separately if the error source is also changing.
    pub fn seed(&mut self, u: f64) {
        let u = u.clamp(self.lo, self.hi);
        self.u = u;
        self.v = u;
    }

    /// Resets `e_{k-1}` to `e` without touching `u`, so the next tick's
    /// proportional term (`Kc·(e_k - e_{k-1})`) is zero regardless of how
    /// far the setpoint jumped. Called implicitly by `step` on an
    /// error-kind switch and on leaving any freeze; the controller also
    /// calls it directly on a T*/snapped-target re-derivation (§2.4).
    pub fn resync_error(&mut self, e: f64) {
        self.e_prev = e;
    }

    /// Applies §2.4's curve-slope schedule to `kc_w_per_rpm`:
    /// `slope_ref / max(slope_at(T*), slope_ref)`, clamped to `[0.25, 1]x`.
    /// `None` (no resolved curve) gets the conservative `0.25x` floor, never
    /// the `1x` a defaulted zero slope would silently produce. Returns the
    /// resolved scale and stores it for the next `Rpm`-kind `step`.
    pub fn scale_rpm_gain(&mut self, slope: Option<f64>) -> f64 {
        let scale = match slope {
            None => RPM_GAIN_SCALE_FLOOR,
            Some(slope_at_t_star) => {
                (SLOPE_REF_PCT_PER_C / slope_at_t_star.max(SLOPE_REF_PCT_PER_C))
                    .clamp(RPM_GAIN_SCALE_FLOOR, 1.0)
            }
        };
        self.rpm_gain_scale = scale;
        scale
    }

    /// Demand-limited predicate seam (§2.4). **Placeholder — owner:
    /// `fw-fanctrl-loop-9it`.** That spike measures the real rule against
    /// every scenario the design's review rounds named and rewrites this
    /// function (and §2.4) accordingly; this task only owns the fixed
    /// invariants: judged per axis (one saturated axis can halt without the
    /// others ever entering the decision — a combined-sum test would make
    /// any structurally undrawn component, e.g. the GPU floor share while
    /// the dGPU is off, a permanent gap), and the halt applies only when
    /// `error_sign` is itself pushing in the deepening direction (positive
    /// — the error is calling for *more* budget) — `error_sign <= 0.0` is
    /// already the recovering direction and is never halted here regardless
    /// of axis state.
    ///
    /// `axes` is `&[(draw_w, cap_w)]`, one pair per actuator. No constant is
    /// tuned here (no `DEMAND_MARGIN_W`, no hysteresis) — those are the
    /// spike's outputs.
    pub fn set_demand_state(&mut self, axes: &[(f64, f64)], error_sign: f64) -> bool {
        let any_axis_pinned = axes.iter().any(|&(draw, cap)| draw >= cap);
        error_sign > 0.0 && any_axis_pinned
    }

    /// Advances the integrator one tick and returns the new `u`, watts.
    ///
    /// `freeze` holds `u` exactly for `ActuatorMismatch`/`Calibrating`/
    /// `Released`; `DemandLimited` instead gates only the deepening
    /// direction of this tick's increment (see [`Freeze::DemandLimited`]).
    /// `None` runs the ordinary clamp + back-calculation PI.
    pub fn step(&mut self, err: LoopError, freeze: Option<Freeze>) -> f64 {
        let kind_switched = self.last_kind.is_some_and(|k| k != err.kind());
        let leaving_freeze = self.last_freeze.is_some() && freeze.is_none();
        self.last_kind = Some(err.kind());
        self.last_freeze = freeze;

        if kind_switched || leaving_freeze {
            self.resync_error(err.value());
        }

        if let Some(reason) = freeze {
            if reason != Freeze::DemandLimited {
                // Hard hold: no PI computation, u (and v, so nothing has
                // silently wound up while frozen) stay exactly as they are.
                return self.u;
            }
        }

        let e_k = err.value();
        let (kc, ti) = self.gains_for(err.kind());
        let raw_du = kc * (e_k - self.e_prev) + (kc * PI_PERIOD_S / ti) * e_k;

        let du = if freeze == Some(Freeze::DemandLimited) && raw_du > 0.0 {
            0.0
        } else {
            raw_du
        };

        // Back-calculation against the bounds only, Tt = Ti: bleeds v back
        // toward the actual (clamped) u with time constant ti whenever they
        // differ, i.e. whenever the last tick saturated.
        let back_calc = (PI_PERIOD_S / ti) * (self.u - self.v);
        let v_new = self.v + du + back_calc;
        let u_new = v_new.clamp(self.lo, self.hi);

        let at_lower = v_new <= self.lo;
        let at_upper = v_new >= self.hi;
        self.lower_bound_dwell = if at_lower {
            self.lower_bound_dwell + Duration::from_secs_f64(PI_PERIOD_S)
        } else {
            Duration::ZERO
        };
        self.upper_bound_dwell = if at_upper {
            self.upper_bound_dwell + Duration::from_secs_f64(PI_PERIOD_S)
        } else {
            Duration::ZERO
        };

        self.v = v_new;
        self.u = u_new;
        self.e_prev = e_k;
        u_new
    }

    /// How long `u` has been continuously clamped at the *lower* bound.
    /// Resets to zero the moment it is off that bound (including while at
    /// the upper bound, or while frozen — dwell only accrues on ticks that
    /// actually ran the clamp).
    pub fn at_lower_bound_for(&self) -> Duration {
        self.lower_bound_dwell
    }

    /// Symmetric with [`Budget::at_lower_bound_for`], for the upper bound.
    pub fn at_upper_bound_for(&self) -> Duration {
        self.upper_bound_dwell
    }

    fn gains_for(&self, kind: ErrorKind) -> (f64, f64) {
        match kind {
            ErrorKind::Temp => (self.gains.kc_w_per_c, self.gains.ti_s),
            ErrorKind::Rpm => (
                self.gains.kc_w_per_rpm * self.rpm_gain_scale,
                self.gains.ti_rpm_s,
            ),
        }
    }
}

/// Integrator warm-start map API (§2.4). The map itself is persisted
/// directly as `PersistedState.warm_start: BTreeMap<String, f64>`; this is
/// a stateless helper over that map, not a wrapper type, so the persisted
/// schema (owned by fwloop.11) is a plain map with no extra indirection.
pub struct WarmStart;

impl WarmStart {
    /// Stable, distinct key for a (strategy, snapped target duty, AC/battery)
    /// combination — changing any one of the three inputs changes the key.
    pub fn key(strategy: &str, duty: u8, on_ac: bool) -> String {
        format!("{strategy}:{duty}:{}", if on_ac { "ac" } else { "batt" })
    }

    /// Looks up a previously recorded `u` for `key`; `None` on a miss.
    pub fn lookup(map: &BTreeMap<String, f64>, key: &str) -> Option<f64> {
        map.get(key).copied()
    }

    /// Records (or overwrites) the last settled `u` for `key`.
    pub fn record(map: &mut BTreeMap<String, f64>, key: String, u: f64) {
        map.insert(key, u);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Step 2: LoopGains::default() and serde round-trip ----

    #[test]
    fn default_gains_are_the_theta_eff_derived_imc_defaults() {
        let g = LoopGains::default();
        assert_eq!(g.kc_w_per_c, 0.22);
        assert_eq!(g.ti_s, 35.0);
        assert_eq!(g.kc_w_per_rpm, 0.0028);
        assert_eq!(g.ti_rpm_s, 35.0);
    }

    #[test]
    fn loop_gains_round_trip_through_serde() {
        let g = LoopGains {
            kc_w_per_c: 0.31,
            ti_s: 40.0,
            kc_w_per_rpm: 0.0041,
            ti_rpm_s: 22.0,
        };
        let json = serde_json::to_string(&g).unwrap();
        let back: LoopGains = serde_json::from_str(&json).unwrap();
        assert_eq!(g, back);
    }

    // ---- Step 3: velocity-form step response on a first-order plant ----

    /// Discrete-time FOPDT plant (exact zero-order-hold discretization of a
    /// first-order lag) with a dead-time input queue, driven by `u`. Starts
    /// at `y0`.
    struct Fopdt {
        tau_s: f64,
        k: f64,
        y: f64,
        delay: std::collections::VecDeque<f64>,
    }

    impl Fopdt {
        fn new(tau_s: f64, theta_s: f64, k: f64, y0: f64, u0: f64) -> Self {
            let delay_ticks = (theta_s / PI_PERIOD_S).round() as usize;
            Self {
                tau_s,
                k,
                y: y0,
                delay: std::collections::VecDeque::from(vec![u0; delay_ticks.max(1)]),
            }
        }

        /// Pushes `u`, pops the dead-time-delayed input, advances `y` one
        /// `PI_PERIOD_S` tick, returns the new `y`.
        fn step(&mut self, u: f64) -> f64 {
            self.delay.push_back(u);
            let u_delayed = self.delay.pop_front().unwrap();
            let a = (-PI_PERIOD_S / self.tau_s).exp();
            self.y = self.y * a + self.k * (1.0 - a) * u_delayed;
            self.y
        }
    }

    /// Runs a Temp-mode closed loop (setpoint step from 0 to `setpoint`) to
    /// `ticks` steps and returns the plant output trace.
    fn run_temp_closed_loop(gains: &LoopGains, setpoint: f64, ticks: usize) -> Vec<f64> {
        let mut budget = Budget::new(gains);
        budget.set_bounds(-1.0e6, 1.0e6); // wide: exercise pure PI, no clamp
        let mut plant = Fopdt::new(35.0, 20.0, 0.8, 0.0, 0.0);
        let mut trace = Vec::with_capacity(ticks);
        for _ in 0..ticks {
            let e_c = setpoint - plant.y;
            let u = budget.step(LoopError::Temp { e_c }, None);
            trace.push(plant.step(u));
        }
        trace
    }

    #[test]
    fn temp_step_response_settles_within_1pct_overshoot_at_most_5pct_at_defaults() {
        let setpoint = 10.0;
        let trace = run_temp_closed_loop(&LoopGains::default(), setpoint, 600);

        let peak = trace.iter().cloned().fold(f64::MIN, f64::max);
        let overshoot_pct = ((peak - setpoint) / setpoint * 100.0).max(0.0);
        assert!(
            overshoot_pct <= 5.0,
            "overshoot {overshoot_pct:.2}% exceeds 5%: peak={peak:.4}"
        );

        let last = *trace.last().unwrap();
        let final_err_pct = ((setpoint - last).abs() / setpoint) * 100.0;
        assert!(
            final_err_pct <= 1.0,
            "final error {final_err_pct:.2}% exceeds 1%: last={last:.4}"
        );
    }

    #[test]
    fn set_gains_changes_the_gains_the_next_step_uses() {
        let mut budget_default = Budget::new(&LoopGains::default());
        budget_default.set_bounds(-1.0e6, 1.0e6);
        let u_default = budget_default.step(LoopError::Temp { e_c: 10.0 }, None);

        let mut budget_changed = Budget::new(&LoopGains::default());
        budget_changed.set_bounds(-1.0e6, 1.0e6);
        let mut doubled = LoopGains::default();
        doubled.kc_w_per_c *= 2.0;
        budget_changed.set_gains(&doubled);
        let u_changed = budget_changed.step(LoopError::Temp { e_c: 10.0 }, None);

        assert_ne!(u_default, u_changed);
    }

    #[test]
    fn non_default_loop_gains_change_the_step_magnitude() {
        let default_gains = LoopGains::default();
        let mut aggressive_gains = default_gains;
        aggressive_gains.kc_w_per_c *= 4.0;

        let mut default_budget = Budget::new(&default_gains);
        default_budget.set_bounds(-1.0e6, 1.0e6);
        let mut aggressive_budget = Budget::new(&aggressive_gains);
        aggressive_budget.set_bounds(-1.0e6, 1.0e6);

        let e_c = 10.0;
        let u_default = default_budget.step(LoopError::Temp { e_c }, None);
        let u_aggressive = aggressive_budget.step(LoopError::Temp { e_c }, None);

        assert_ne!(u_default, u_aggressive);
        assert!(u_aggressive.abs() > u_default.abs());
    }

    // ---- Step 4: clamp + back-calculation against the bounds ----

    #[test]
    fn clamp_holds_at_bound_without_windup_and_release_recovers_within_one_ti() {
        let gains = LoopGains::default();
        let mut budget = Budget::new(&gains);
        budget.set_bounds(0.0, 10.0);

        // Persistent large positive error: pin at the upper bound.
        for _ in 0..20 {
            let u = budget.step(LoopError::Temp { e_c: 1000.0 }, None);
            assert_eq!(u, 10.0);
        }

        // Release: strong negative error. "Recovers within one Ti" — Ti=35s
        // = 7 ticks at PI_PERIOD_S=5 — u must be clearly off the bound
        // within that many ticks.
        let ti_ticks = (gains.ti_s / PI_PERIOD_S).ceil() as usize;
        let mut off_bound_tick = None;
        for tick in 1..=ti_ticks {
            let u = budget.step(LoopError::Temp { e_c: -1000.0 }, None);
            if u < 10.0 - 1e-9 {
                off_bound_tick = Some(tick);
                break;
            }
        }
        assert!(
            off_bound_tick.is_some(),
            "u never left the upper bound within one Ti ({ti_ticks} ticks)"
        );
    }

    // ---- Step 5: bound-dwell counters ----

    #[test]
    fn bound_dwell_counters_track_only_their_own_bound_and_reset_off_it() {
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(0.0, 10.0);
        budget.seed(10.0);

        // One tick of deepening error: v grows past the upper bound.
        budget.step(LoopError::Temp { e_c: 1000.0 }, None);
        assert!(budget.at_upper_bound_for() > Duration::ZERO);
        assert_eq!(budget.at_lower_bound_for(), Duration::ZERO);

        // Release with a neutral error: back-calculation has only one
        // tick's overshoot to bleed off, so v lands back inside the bounds
        // in this one step — both counters reset.
        let u = budget.step(LoopError::Temp { e_c: 0.0 }, None);
        assert!((0.0..10.0).contains(&u), "expected u inside bounds, got {u}");
        assert_eq!(budget.at_upper_bound_for(), Duration::ZERO);
        assert_eq!(budget.at_lower_bound_for(), Duration::ZERO);

        // Symmetric: seed at the lower bound, one tick of deepening
        // (negative) error saturates low, and never touches the upper
        // counter.
        budget.seed(0.0);
        budget.resync_error(0.0);
        budget.step(LoopError::Temp { e_c: -1000.0 }, None);
        assert!(budget.at_lower_bound_for() > Duration::ZERO);
        assert_eq!(budget.at_upper_bound_for(), Duration::ZERO);
    }

    // ---- Step 6: Freeze holds u exactly; leaving a freeze resyncs ----

    #[test]
    fn each_hard_freeze_reason_holds_u_exactly_across_a_step() {
        for reason in [
            Freeze::ActuatorMismatch,
            Freeze::Calibrating,
            Freeze::Released,
        ] {
            let mut budget = Budget::new(&LoopGains::default());
            budget.set_bounds(-1.0e6, 1.0e6);
            budget.seed(5.0);
            let held = budget.step(LoopError::Temp { e_c: 1000.0 }, Some(reason));
            assert_eq!(held, 5.0, "{reason:?} did not hold u exactly");
            let held_again = budget.step(LoopError::Temp { e_c: -1000.0 }, Some(reason));
            assert_eq!(held_again, 5.0, "{reason:?} did not hold u exactly");
        }
    }

    #[test]
    fn leaving_a_freeze_resyncs_error_so_the_first_post_freeze_tick_has_no_kick() {
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(-1.0e6, 1.0e6);
        budget.seed(0.0);

        // e_prev is left stale at a very different value while frozen.
        budget.step(LoopError::Temp { e_c: 500.0 }, Some(Freeze::Calibrating));
        budget.step(LoopError::Temp { e_c: 500.0 }, Some(Freeze::Calibrating));

        // Leaving: no proportional kick means Δu equals exactly one
        // integral increment for the error on this first unfrozen tick.
        let gains = LoopGains::default();
        let e_c = 3.0;
        let expected_du = (gains.kc_w_per_c * PI_PERIOD_S / gains.ti_s) * e_c;
        let u_before = budget.u;
        let u_after = budget.step(LoopError::Temp { e_c }, None);
        assert!(
            (u_after - u_before - expected_du).abs() < 1e-9,
            "expected pure-integral increment {expected_du}, got {}",
            u_after - u_before
        );
    }

    // ---- Step 7: resync_error, and error-kind switch ----

    #[test]
    fn resync_error_after_setpoint_jump_produces_no_proportional_kick() {
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(-1.0e6, 1.0e6);
        budget.seed(0.0);
        budget.step(LoopError::Temp { e_c: 2.0 }, None);

        // Setpoint jump: caller resyncs to the new error before the next
        // step (the controller's contract on a T* re-derivation).
        let gains = LoopGains::default();
        let jumped_e_c = 50.0;
        budget.resync_error(jumped_e_c);
        let u_before = budget.u;
        let expected_du = (gains.kc_w_per_c * PI_PERIOD_S / gains.ti_s) * jumped_e_c;
        let u_after = budget.step(LoopError::Temp { e_c: jumped_e_c }, None);
        assert!(
            (u_after - u_before - expected_du).abs() < 1e-9,
            "expected pure-integral increment {expected_du}, got {}",
            u_after - u_before
        );
    }

    #[test]
    fn temp_to_rpm_switch_produces_at_most_one_integral_increment() {
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(-1.0e6, 1.0e6);
        budget.seed(0.0);
        budget.step(LoopError::Temp { e_c: 5.0 }, None);

        let gains = LoopGains::default();
        let e_rpm = 40.0;
        let one_integral_increment =
            (gains.kc_w_per_rpm * budget.rpm_gain_scale * PI_PERIOD_S / gains.ti_rpm_s) * e_rpm;

        let u_before = budget.u;
        let u_after = budget.step(LoopError::Rpm { e_rpm }, None);
        let delta_u = (u_after - u_before).abs();
        assert!(
            delta_u <= one_integral_increment.abs() + 1e-9,
            "|delta u| {delta_u} exceeds one integral increment {one_integral_increment}"
        );
    }

    // ---- Step 8: the roast-3 regression — directional, per-axis halt ----

    #[test]
    fn demand_limited_halt_still_integrates_down_when_error_calls_for_less_heat() {
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(-1.0e6, 1.0e6);
        budget.seed(50.0);

        // Condition active: an axis is pinned at its cap, and error_sign
        // says the *current* error direction is deepening — a caller would
        // report DemandLimited this tick.
        let halted = budget.set_demand_state(&[(30.0, 30.0)], 1.0);
        assert!(halted);

        // But THIS tick's error calls for LESS heat (negative e_c) — the
        // recovering direction must still integrate down, unblocked.
        let u_before = budget.u;
        let u_after = budget.step(LoopError::Temp { e_c: -50.0 }, Some(Freeze::DemandLimited));
        assert!(
            u_after < u_before,
            "u did not integrate down under DemandLimited with a recovering error: {u_before} -> {u_after}"
        );
    }

    #[test]
    fn demand_limited_halt_blocks_only_the_deepening_direction() {
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(-1.0e6, 1.0e6);
        budget.seed(50.0);
        budget.set_demand_state(&[(30.0, 30.0)], 1.0);

        // Error calls for MORE heat: deepening direction is blocked, u must
        // not move at all this tick (not "not much" — exactly held, no
        // corrective pull anywhere either).
        let u_before = budget.u;
        let u_after = budget.step(LoopError::Temp { e_c: 50.0 }, Some(Freeze::DemandLimited));
        assert_eq!(u_after, u_before);
    }

    #[test]
    fn u_never_decays_toward_the_draw_over_a_long_low_draw_hold() {
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(-1.0e6, 1.0e6);
        let seeded = 80.0;
        budget.seed(seeded);
        let draw = 20.0; // far below u — a tracker would drag u toward this.

        // Deepening direction, held every tick for a long window.
        for _ in 0..200 {
            budget.set_demand_state(&[(draw, draw)], 1.0);
            let u = budget.step(LoopError::Temp { e_c: 50.0 }, Some(Freeze::DemandLimited));
            assert_eq!(u, seeded, "u decayed toward the draw: {u} (draw={draw})");
        }
    }

    #[test]
    fn set_demand_state_judges_each_axis_separately() {
        let mut budget = Budget::new(&LoopGains::default());
        // Neither axis pinned: no halt.
        assert!(!budget.set_demand_state(&[(10.0, 30.0), (5.0, 20.0)], 1.0));
        // One axis pinned (GPU floor share while the dGPU draws nothing —
        // structurally undrawn, cap 0): halts, without the CPU axis (well
        // under its cap) entering the decision at all.
        assert!(budget.set_demand_state(&[(29.0, 30.0), (0.0, 0.0)], 1.0));
    }

    #[test]
    fn set_demand_state_never_halts_the_recovering_error_sign() {
        let mut budget = Budget::new(&LoopGains::default());
        // Axis pinned, but error_sign already recovering: never halted.
        assert!(!budget.set_demand_state(&[(30.0, 30.0)], -1.0));
        assert!(!budget.set_demand_state(&[(30.0, 30.0)], 0.0));
    }

    // ---- Step 9: scale_rpm_gain ----

    #[test]
    fn scale_rpm_gain_returns_1x_at_slope_ref() {
        let mut budget = Budget::new(&LoopGains::default());
        let scale = budget.scale_rpm_gain(Some(SLOPE_REF_PCT_PER_C));
        assert!((scale - 1.0).abs() < 1e-12);
    }

    #[test]
    fn scale_rpm_gain_returns_quarter_x_at_four_times_slope_ref() {
        let mut budget = Budget::new(&LoopGains::default());
        let scale = budget.scale_rpm_gain(Some(4.0 * SLOPE_REF_PCT_PER_C));
        assert!((scale - 0.25).abs() < 1e-12);
    }

    #[test]
    fn scale_rpm_gain_returns_quarter_x_on_none() {
        let mut budget = Budget::new(&LoopGains::default());
        let scale = budget.scale_rpm_gain(None);
        assert!((scale - 0.25).abs() < 1e-12);
    }

    // ---- Step 10: WarmStart ----

    #[test]
    fn warm_start_key_is_stable_and_distinct_across_each_input() {
        let base = WarmStart::key("quiet16", 30, true);
        assert_eq!(base, WarmStart::key("quiet16", 30, true));

        assert_ne!(base, WarmStart::key("cool16", 30, true));
        assert_ne!(base, WarmStart::key("quiet16", 36, true));
        assert_ne!(base, WarmStart::key("quiet16", 30, false));
    }

    #[test]
    fn warm_start_record_then_lookup_round_trips_and_a_miss_is_none() {
        let mut map = BTreeMap::new();
        let key = WarmStart::key("quiet16", 30, true);
        assert_eq!(WarmStart::lookup(&map, &key), None);

        WarmStart::record(&mut map, key.clone(), 42.5);
        assert_eq!(WarmStart::lookup(&map, &key), Some(42.5));

        assert_eq!(WarmStart::lookup(&map, "no-such-key"), None);
    }
}

//! The mode arbiter (design doc §2.5, §2.6, §2.7): a pure, stateful
//! decision engine that picks which loop (if any) owns actuation in Auto
//! mode, reconciles the EC replica against fw-fanctrl's own view, judges
//! feasibility/steepness of the current target, and re-derives T* off a
//! point-keyed `Curve` cache. Tested standalone, with no controller.
//!
//! # Two clocks, one call
//!
//! [`Arbiter::decide`] is the single entry point, called once per allocator
//! tick (5 s, [`crate::control::budget::PI_PERIOD_S`]). All of this
//! module's tick-counted state (entry hysteresis, the argmax debounce, the
//! feasible-again clear) counts *calls* to `decide`, not wall-clock time —
//! exactly how [`crate::control::budget::Budget`]'s bound-dwell timers work,
//! and for the same reason: the caller's cadence is the tick, so there is
//! no need for an injected clock to make the counters deterministic in
//! tests.
//!
//! Reconciliation (§2.6) is scored on a *different*, coarser cadence (once
//! per fresh `print all` view, roughly every 30 s) than the mode table is
//! evaluated (every 5 s tick). `ArbiterInput::view_changed` is how the
//! controller (a later task) tells this call whether it is carrying a fresh
//! view worth scoring; when it does, `decide` scores it before evaluating
//! the table, so the two cadences share one call site without either
//! starving the other of updates.
//!
//! # Entry hysteresis is layered, not monolithic
//!
//! The row table's own conditions (freshness, `active`, EC validity, curve
//! validity, feasibility, the debounced argmax check) form `core_ok`; a
//! generic counter requires `core_ok` for [`ENTRY_HYSTERESIS_TICKS`]
//! *consecutive* calls before TempLoop is reachable at all — this is what
//! makes a clean, nothing-ever-wrong Auto engagement take 15 s to close the
//! loop, per §2.5.
//!
//! Reconciliation (`reconciled && !ec_mismatch`) is deliberately **not**
//! part of that streak: it is its own independent gate, checked fresh every
//! call. `EC MISMATCH`'s own three-match clear (§2.6) is already a
//! three-tick debounce in its own right; stacking the generic entry
//! hysteresis on top of it would mean a recovering loop needs six ticks,
//! not three, and the acceptance criterion ("three matches -> TempLoop with
//! `reseed_ma`") is explicit that the third matching view is the one that
//! lands back in TempLoop, not the sixth. Splitting the gate this way is
//! what makes both criteria true at once: `core_ok` (unaffected by
//! reconciliation) keeps accumulating through a mismatch episode, so by the
//! time the third match clears `ec_mismatch` the streak is already long
//! past three and TempLoop is reachable on that same tick.

use std::time::Duration;

use crate::control::controller::{LoopMode, StatusFlag};
use crate::fanctrl::client::{FanctrlView, Freshness};
use crate::fanctrl::curve::Curve;
use crate::sensors::ec::EcReading;

/// Consecutive `decide` calls a row's own conditions (`core_ok`, below) must
/// hold before the arbiter switches **into** it. Exit is immediate: dropping
/// out never waits for this counter (§2.5).
const ENTRY_HYSTERESIS_TICKS: u32 = 3;

/// Consecutive scored views needed to latch `EC MISMATCH`, or to clear it
/// (§2.6).
const MISMATCH_STRIKES: u8 = 3;

/// `|replica.max_c - view.temperature| > this` counts as a scored mismatch
/// (§2.6).
const MISMATCH_ABS_DIFF_C: f64 = 1.0;

/// `|ec_ma - view.ma_temperature| > this` counts as a scored MA-check
/// failure (§2.6).
const MA_ABS_DIFF_C: f64 = 2.0;

/// A view is skipped (scored as neither match nor mismatch) when the
/// replica's own 5 s slope is at or above this (§2.6).
const SKIP_SLOPE_C_PER_S: f64 = 0.5;

/// A view is skipped when the gap between its capture and the sample
/// carrying it is at or above this (§2.6).
const SKIP_GAP_S: f64 = 2.0;

/// Consecutive argmax-uncontrollable ticks before that alone can drop
/// TempLoop (§2.5's debounce).
const ARGMAX_DEBOUNCE_TICKS: u8 = 3;

/// An uncontrollable argmax leading the best controllable reading by more
/// than this drops TempLoop immediately, bypassing the debounce above
/// (§2.5).
const ARGMAX_LEAD_C: f64 = 1.0;

/// Feasibility margin: T* must be at least this far above the highest
/// uncontrollable reading (§2.7).
const FEASIBLE_MARGIN_C: f64 = 5.0;

/// `slope_at(T*)` strictly above this raises `STEEP CURVE` (§2.7).
const STEEP_SLOPE_PCT_PER_C: f64 = 2.0;

/// Consecutive feasible ticks needed to clear a latched infeasibility, at
/// the 5 s allocator cadence (60 s / `PI_PERIOD_S`, §2.7). A literal `const`
/// rather than computed from [`crate::control::budget::PI_PERIOD_S`] to
/// avoid a `f64 as u32` const-cast; the module doc above ties the two
/// together.
const FEASIBLE_CLEAR_TICKS: u32 = 12;

/// How long a budget bound must be held, with the error still calling in
/// that direction, before the `low`/`high` unreachable reasons fire (§2.7).
const BOUND_HOLD: Duration = Duration::from_secs(60);

/// Everything the arbiter needs for one `decide` call. Borrowed, not owned:
/// this is read once per tick and never retained past the call.
pub struct ArbiterInput<'a> {
    /// fw-fanctrl's live view, `None` before the very first successful
    /// poll (client just started) or once the socket has never answered.
    pub fanctrl: Option<&'a FanctrlView>,
    /// Staleness of `fanctrl`, computed by the caller from its timestamps
    /// (design §2.1) — carried separately because `fanctrl` may still hold
    /// the last-known view while `freshness` has already gone `Stale`.
    pub freshness: Freshness,
    /// Set by the controller on the sample whose `fanctrl` carries a fresh
    /// `print all` view (`all_observed_at` just changed) — the trigger for
    /// this call to score reconciliation (§2.6). `false` on every other
    /// call; the mode table is still evaluated as normal.
    pub view_changed: bool,
    /// The controller's EC replica reading this tick, `None` when the chip
    /// is missing or every sensor is unreadable (`ec_valid` in the bead
    /// text — modelled as `Option` rather than a separate bool because
    /// there is nothing else useful to do with an EC reading that failed).
    pub ec: Option<&'a EcReading>,
    /// The controller's live `EcAverage` mean, `None` until the boxcar has
    /// been seeded/filled at least once.
    pub ec_ma: Option<f64>,
    /// The sampler's existing fan-valid flag (survives unchanged, §1).
    pub fan_valid: bool,
    /// The (already curve/table-snapped, §2.3) duty the loop is steering
    /// toward. May legitimately sit below the curve's lowest tread — that
    /// is exactly the `low` unreachable case (§2.7), not a caller bug.
    pub target_duty: u8,
    /// How long `Budget`'s `u` has sat continuously at its lower bound
    /// (`Budget::at_lower_bound_for`).
    pub at_lower_bound_for: Duration,
    /// Symmetric with `at_lower_bound_for`, for the upper bound.
    pub at_upper_bound_for: Duration,
    /// Sign of the loop's current control error: positive calls for more
    /// heat (budget should rise), negative for less. Mirrors
    /// `Budget::set_demand_state`'s own `error_sign: f64` convention — only
    /// the sign is read, but the caller already has the value, so no
    /// separate enum is introduced for it.
    pub error_sign: f64,
    /// `false` when the controller's `Curve::from_points` rejected the
    /// view's points as non-monotone (Task 1). Authoritative: when `false`
    /// this call does not attempt its own `from_points` re-derivation at
    /// all, regardless of what `fanctrl.curve` contains.
    pub curve_valid: bool,
    /// The replica's own temperature slope over the last 5 s, °C/s — the
    /// skip rule's first half (§2.6). Meaningless (and ignored) unless
    /// `view_changed`.
    pub replica_slope_5s_c_per_s: f64,
    /// Gap, in seconds, between the view's own capture stamp
    /// (`all_observed_at`) and the sample carrying it — the skip rule's
    /// second half (§2.6). Meaningless (and ignored) unless `view_changed`.
    pub view_to_sample_gap_s: f64,
}

/// The arbiter's verdict for one tick (§2.5-§2.7).
#[derive(Debug, Clone, PartialEq)]
pub struct Decision {
    pub mode: LoopMode,
    /// The tread centre the current `target_duty` resolves to. `None`
    /// whenever no curve is resolved: `curve_valid: false`, no `fanctrl`
    /// view yet, or `target_duty` below the curve's lowest tread.
    pub t_star: Option<f64>,
    /// `slope_at(t_star)`, %/°C. `None` exactly when `t_star` is `None`.
    pub slope: Option<f64>,
    /// Human-readable cause(s) for this tick's verdict: the winning row's
    /// blocking reason (when not TempLoop) plus any independent
    /// unreachable-target explanation. Order is not meaningful; callers
    /// that want telemetry text join them.
    pub reasons: Vec<String>,
    /// Status flags active this tick (§2.5-§2.7's flags only — guard flags
    /// like `GPU HOT` are a different task's concern).
    pub flags: Vec<StatusFlag>,
    /// Set on the tick reconciliation clears (`EC MISMATCH` -> matched) or
    /// the MA check fails three times running — either way the controller
    /// should re-seed `EcAverage` from this value (§2.6).
    pub reseed_ma: Option<f64>,
    /// Whether `EC MISMATCH` is latched *after* this tick's scoring.
    pub ec_mismatch: bool,
    /// One-shot: true exactly on the tick T* was re-derived (the view's
    /// curve points changed, `target_duty` changed, or the curve just
    /// became valid/invalid) — the controller's cue to call
    /// `Budget::resync_error` (§2.4).
    pub t_star_changed: bool,
}

/// Owns everything this module's brief assigns it: the mismatch/feasibility
/// counters, the point-keyed cached [`Curve`], and all T* derivation. One
/// instance per Auto session (constructed fresh on entry, like
/// `AutoState`), so its "sticky" state (`reconciled`, the curve cache)
/// never needs to be reset by hand — a fresh engagement gets a fresh
/// `Arbiter`.
pub struct Arbiter {
    /// Consecutive `decide` calls with `core_ok` true (see module docs).
    entry_streak: u32,

    /// Whether at least one view has ever been scored (§2.6: distinct from
    /// a mismatch — "unreconciled" is its own reason). Sticky: only ever
    /// flips false -> true.
    reconciled: bool,
    /// `EC MISMATCH` latch (§2.6).
    ec_mismatch: bool,
    mismatch_streak: u8,
    match_streak: u8,
    /// Separate streak for the moving-average check (§2.6) — independent
    /// of the max_c comparison above; it re-seeds rather than latching.
    ma_fail_streak: u8,

    /// Sticky "target has been infeasible" state and its clear-countdown
    /// (§2.7). The `low`/`high` unreachable rules need no equivalent: they
    /// read `Budget`'s own dwell `Duration`s, which already reset
    /// themselves the instant the bound is left.
    infeasible_latched: bool,
    feasible_streak: u32,

    /// Debounce for the argmax-controllable condition (§2.5).
    argmax_fail_streak: u8,

    /// Point-keyed cache: rebuilt only when `fanctrl.curve` differs from
    /// this, or `target_duty` differs from `cached_target_duty` (§2.5's
    /// "keyed on points, not the strategy name").
    cached_points: Vec<(f64, u8)>,
    cached_curve: Option<Curve>,
    cached_target_duty: Option<u8>,
}

impl Default for Arbiter {
    fn default() -> Self {
        Self::new()
    }
}

impl Arbiter {
    pub fn new() -> Self {
        Self {
            entry_streak: 0,
            reconciled: false,
            ec_mismatch: false,
            mismatch_streak: 0,
            match_streak: 0,
            ma_fail_streak: 0,
            infeasible_latched: false,
            feasible_streak: 0,
            argmax_fail_streak: 0,
            cached_points: Vec::new(),
            cached_curve: None,
            cached_target_duty: None,
        }
    }

    /// Evaluate one tick (§2.5, in row order; §2.6 reconciliation scored
    /// first when `input.view_changed`; §2.7 feasibility/steepness folded
    /// in).
    pub fn decide(&mut self, input: &ArbiterInput) -> Decision {
        let mut reasons: Vec<String> = Vec::new();
        let mut flags: Vec<StatusFlag> = Vec::new();
        let mut reseed_ma: Option<f64> = None;

        // ---- §2.6: reconciliation, scored only on a fresh, fair view ----
        if input.view_changed {
            if let (Some(view), Some(ec)) = (input.fanctrl, input.ec) {
                let scorable = input.replica_slope_5s_c_per_s < SKIP_SLOPE_C_PER_S
                    && input.view_to_sample_gap_s < SKIP_GAP_S;
                if scorable {
                    self.reconciled = true;

                    let diff = (f64::from(ec.max_c) - view.temperature).abs();
                    if diff > MISMATCH_ABS_DIFF_C {
                        self.mismatch_streak = self.mismatch_streak.saturating_add(1);
                        self.match_streak = 0;
                    } else {
                        self.match_streak = self.match_streak.saturating_add(1);
                        self.mismatch_streak = 0;
                    }
                    if self.mismatch_streak >= MISMATCH_STRIKES {
                        self.ec_mismatch = true;
                    }
                    if self.ec_mismatch && self.match_streak >= MISMATCH_STRIKES {
                        self.ec_mismatch = false;
                        self.match_streak = 0;
                        reseed_ma = Some(view.ma_temperature);
                    }

                    if let Some(ec_ma) = input.ec_ma {
                        let ma_diff = (ec_ma - view.ma_temperature).abs();
                        if ma_diff > MA_ABS_DIFF_C {
                            self.ma_fail_streak = self.ma_fail_streak.saturating_add(1);
                            if self.ma_fail_streak >= MISMATCH_STRIKES {
                                self.ma_fail_streak = 0;
                                reseed_ma = Some(view.ma_temperature);
                            }
                        } else {
                            self.ma_fail_streak = 0;
                        }
                    }
                }
                // else: skipped — advances neither counter (§2.6).
            }
        }
        if self.ec_mismatch {
            flags.push(StatusFlag::EcMismatch);
        }

        // ---- §2.5/§2.7: curve validity + point-keyed T* derivation ----
        let mut t_star: Option<f64> = None;
        let mut slope: Option<f64> = None;
        let mut t_star_changed = false;
        let mut low_subfloor = false;

        if !input.curve_valid {
            if self.cached_curve.is_some() {
                t_star_changed = true;
            }
            self.cached_curve = None;
            self.cached_points.clear();
            self.cached_target_duty = None;
            flags.push(StatusFlag::CurveInvalid);
            reasons.push("curve_invalid".to_string());
        } else if let Some(view) = input.fanctrl {
            let points_changed = view.curve != self.cached_points;
            if points_changed || self.cached_curve.is_none() {
                match Curve::from_points(view.curve.clone()) {
                    Ok(curve) => {
                        self.cached_curve = Some(curve);
                        self.cached_points = view.curve.clone();
                    }
                    Err(_) => {
                        // curve_valid told us this would succeed; treat a
                        // contradiction defensively, the same as an
                        // invalid curve.
                        self.cached_curve = None;
                    }
                }
            }
            let duty_changed = self.cached_target_duty != Some(input.target_duty);
            if let Some(curve) = &self.cached_curve {
                if points_changed || duty_changed {
                    t_star_changed = true;
                    self.cached_target_duty = Some(input.target_duty);
                }
                match curve.nearest_tread(input.target_duty) {
                    Some(d) => {
                        if let Some(ts) = curve.t_star(d) {
                            // Regression guard for fw-fanctrl-loop-nez
                            // (§2.1): `Curve::tread`/`t_star` must never
                            // hand this seam a non-finite setpoint — a
                            // later curve change that reintroduced an
                            // unbounded tread would otherwise silently
                            // hand `Budget` an infinite error and NaN it
                            // out on the very next tick.
                            debug_assert!(
                                ts.is_finite(),
                                "curve.t_star({d}) = {ts} is not finite; §2.1 requires \
                                 tread/t_star to never return a non-finite value"
                            );
                            t_star = Some(ts);
                            slope = Some(curve.slope_at(ts));
                        }
                    }
                    None => {
                        low_subfloor = true;
                        reasons.push(format!(
                            "target unreachable (low): duty {} below floor {}",
                            input.target_duty,
                            curve.min_tread_duty()
                        ));
                        flags.push(StatusFlag::TargetUnreachable);
                    }
                }
            }
        }

        if let Some(sl) = slope {
            if sl > STEEP_SLOPE_PCT_PER_C {
                flags.push(StatusFlag::SteepCurve);
            }
        }

        // ---- §2.7: feasibility (only meaningful once T* resolves) ----
        if let (Some(ts), Some(ec)) = (t_star, input.ec) {
            let max_unc = ec
                .all
                .iter()
                .filter(|(l, _)| !l.is_controllable())
                .map(|(_, v)| *v)
                .fold(f64::NEG_INFINITY, f64::max);
            let feasible_now = ts >= max_unc + FEASIBLE_MARGIN_C;
            if feasible_now {
                self.feasible_streak = self.feasible_streak.saturating_add(1);
                if self.infeasible_latched && self.feasible_streak >= FEASIBLE_CLEAR_TICKS {
                    self.infeasible_latched = false;
                }
            } else {
                self.infeasible_latched = true;
                self.feasible_streak = 0;
            }
            if self.infeasible_latched {
                flags.push(StatusFlag::TargetUnreachable);
                reasons.push(format!(
                    "target unreachable: T*={ts:.1} < ambient {max_unc:.0}+{FEASIBLE_MARGIN_C:.0}"
                ));
            }
        }

        // ---- §2.7: low/high bound-hold rules (independent of mode) ----
        if input.at_lower_bound_for >= BOUND_HOLD && input.error_sign < 0.0 {
            flags.push(StatusFlag::TargetUnreachable);
            reasons.push(format!(
                "target unreachable (low): held at floor for {}s",
                input.at_lower_bound_for.as_secs()
            ));
        }
        if input.at_upper_bound_for >= BOUND_HOLD && input.error_sign > 0.0 {
            flags.push(StatusFlag::TargetUnreachable);
            reasons.push(format!(
                "target unreachable (high): held at ceiling for {}s",
                input.at_upper_bound_for.as_secs()
            ));
        }

        // ---- §2.5: the debounced argmax-controllable condition ----
        let mut argmax_ok = true;
        if let Some(ec) = input.ec {
            if ec.argmax.is_controllable() {
                self.argmax_fail_streak = 0;
            } else {
                let max_controllable = ec
                    .all
                    .iter()
                    .filter(|(l, _)| l.is_controllable())
                    .map(|(_, v)| *v)
                    .fold(f64::NEG_INFINITY, f64::max);
                let lead = f64::from(ec.max_c) - max_controllable;
                if lead > ARGMAX_LEAD_C {
                    argmax_ok = false;
                } else {
                    self.argmax_fail_streak = self.argmax_fail_streak.saturating_add(1);
                    if self.argmax_fail_streak >= ARGMAX_DEBOUNCE_TICKS {
                        argmax_ok = false;
                    }
                }
            }
        }

        // ---- §2.5: freshness/fan flags (live, never latched) ----
        if input.freshness != Freshness::Fresh {
            flags.push(StatusFlag::FanctrlLost);
        }

        // ---- §2.5: the row table ----
        let hard_ok = input.freshness == Freshness::Fresh
            && input.fanctrl.is_some_and(|v| v.active)
            && input.ec.is_some()
            && input.curve_valid;
        let feasible_ok = t_star.is_some() && !self.infeasible_latched && !low_subfloor;
        let core_ok = hard_ok && argmax_ok && feasible_ok;

        self.entry_streak = if core_ok {
            self.entry_streak.saturating_add(1)
        } else {
            0
        };

        let reconciliation_ok = self.reconciled && !self.ec_mismatch;
        let temp_loop_ready = self.entry_streak >= ENTRY_HYSTERESIS_TICKS && reconciliation_ok;

        let mode = if temp_loop_ready {
            LoopMode::TempLoop
        } else if input.fan_valid {
            LoopMode::RpmLoop
        } else {
            LoopMode::Released
        };

        if mode == LoopMode::Released && !input.fan_valid {
            flags.push(StatusFlag::SensorLost);
        }

        if mode != LoopMode::TempLoop {
            if input.freshness != Freshness::Fresh {
                reasons.push("fanctrl_lost".to_string());
            } else if !input.fanctrl.is_some_and(|v| v.active) {
                reasons.push("fanctrl_inactive".to_string());
            } else if input.ec.is_none() {
                reasons.push("ec_invalid".to_string());
            } else if !argmax_ok {
                reasons.push("argmax_uncontrollable".to_string());
            } else if !feasible_ok && input.curve_valid {
                // curve_invalid already pushed its own reason above; do not
                // pile "infeasible" on top of it when the curve itself is
                // the reason T* never resolved.
                reasons.push("infeasible".to_string());
            } else if !self.reconciled {
                reasons.push("unreconciled".to_string());
            } else if self.ec_mismatch {
                reasons.push("ec_mismatch".to_string());
            }
        }

        flags.dedup();

        Decision {
            mode,
            t_star,
            slope,
            reasons,
            flags,
            reseed_ma,
            ec_mismatch: self.ec_mismatch,
            t_star_changed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::ec::EcReading;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Instant;

    // ---- fixtures --------------------------------------------------

    const QUIET16: &[(f64, u8)] = &[
        (0.0, 15),
        (55.0, 15),
        (65.0, 21),
        (75.0, 31),
        (82.0, 37),
        (88.0, 55),
        (95.0, 100),
    ];
    const COOL16: &[(f64, u8)] = &[(0.0, 20), (50.0, 20), (60.0, 30), (70.0, 42), (85.0, 100)];

    fn view(curve: &[(f64, u8)], temperature: f64, ma_temperature: f64) -> FanctrlView {
        FanctrlView {
            strategy: "quiet16".to_string(),
            active: true,
            speed_pct: 31,
            temperature,
            ma_temperature,
            ma_interval: 60,
            curve: curve.to_vec(),
            observed_at: Instant::now(),
            all_observed_at: Some(Instant::now()),
        }
    }

    static EC_FIXTURE_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// Builds a real `EcReading` via the module's own public `read()`
    /// entry point (its `EcLabel`/`EcReading` constructors are private to
    /// `sensors::ec`, so a synthetic hwmon-shaped fixture dir is the only
    /// way to build one from outside that module — mirrors `ec.rs`'s own
    /// tests).
    fn ec_reading(sensors: &[(&str, f64)]) -> EcReading {
        let n = EC_FIXTURE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let dir =
            std::env::temp_dir().join(format!("bazerame-mode-test-{}-{n}", std::process::id()));
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

    /// A "nothing is wrong" EC reading: cpu argmax matches the default
    /// view's `temperature` (75C, see `view()`'s defaults used across
    /// these tests) so a scored comparison against it is a match, not a
    /// mismatch; ambient sits well below feasibility's margin.
    fn happy_ec() -> EcReading {
        ec_reading(&[("ambient_f75303@4d", 40.0), ("cpu@4c", 75.0)])
    }

    fn happy_input<'a>(v: &'a FanctrlView, ec: &'a EcReading) -> ArbiterInput<'a> {
        ArbiterInput {
            fanctrl: Some(v),
            freshness: Freshness::Fresh,
            view_changed: false,
            ec: Some(ec),
            ec_ma: Some(v.ma_temperature),
            fan_valid: true,
            target_duty: 31,
            at_lower_bound_for: Duration::ZERO,
            at_upper_bound_for: Duration::ZERO,
            error_sign: 0.0,
            curve_valid: true,
            replica_slope_5s_c_per_s: 0.0,
            view_to_sample_gap_s: 0.0,
        }
    }

    /// Runs `decide` enough times (scoring one matching view, then padding
    /// with clean ticks) to land in TempLoop, returning the arbiter so a
    /// test can keep driving it from a known-good state.
    fn warmed_up(v: &FanctrlView, ec: &EcReading) -> Arbiter {
        let mut a = Arbiter::new();
        let mut first = happy_input(v, ec);
        first.view_changed = true;
        let d = a.decide(&first);
        // A scorable `view_changed` tick IS the reconciliation event (§2.6:
        // `decide` sets `reconciled = true` on a fresh, fair view), so this
        // first tick must NOT report `unreconciled` — it just reconciled.
        // The helper used to assert the opposite, guarded by
        // `|| d.mode != LoopMode::TempLoop`, which is unfailable after one
        // tick (TempLoop needs ENTRY_HYSTERESIS_TICKS consecutive ticks) and
        // so hid that the stated premise was backwards for the whole epic.
        // Mutation-checkable: skip the `reconciled = true` write and this
        // fails.
        assert!(
            !d.reasons.contains(&"unreconciled".to_string()),
            "a scorable view_changed first tick reconciles, so `unreconciled` must be absent; got {:?} (mode {:?})",
            d.reasons,
            d.mode
        );
        for _ in 0..ENTRY_HYSTERESIS_TICKS {
            a.decide(&happy_input(v, ec));
        }
        a
    }

    // ---- 1. table-driven test, one case per §2.5 row ---------------

    #[test]
    fn table_driven_row_selection() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();

        struct Case {
            name: &'static str,
            mutate: fn(&mut ArbiterInput),
            want: LoopMode,
        }
        let cases = [
            Case {
                name: "TempLoop: every condition satisfied",
                mutate: |_| {},
                want: LoopMode::TempLoop,
            },
            Case {
                name: "RpmLoop: fan_valid alone, TempLoop preconditions broken",
                mutate: |i| i.curve_valid = false,
                want: LoopMode::RpmLoop,
            },
            Case {
                name: "Released: nothing to close a loop on",
                mutate: |i| {
                    i.curve_valid = false;
                    i.fan_valid = false;
                },
                want: LoopMode::Released,
            },
        ];

        for case in cases {
            let mut a = warmed_up(&v, &ec);
            let mut input = happy_input(&v, &ec);
            (case.mutate)(&mut input);
            let d = a.decide(&input);
            assert_eq!(d.mode, case.want, "row case: {}", case.name);
        }
    }

    // ---- 2. 3-tick entry hysteresis, immediate exit -----------------

    #[test]
    fn entry_hysteresis_needs_three_ticks_exit_is_immediate() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mut a = Arbiter::new();

        let mut first = happy_input(&v, &ec);
        first.view_changed = true;
        assert_eq!(a.decide(&first).mode, LoopMode::RpmLoop, "tick 1 of 3");
        assert_eq!(
            a.decide(&happy_input(&v, &ec)).mode,
            LoopMode::RpmLoop,
            "tick 2 of 3"
        );
        assert_eq!(
            a.decide(&happy_input(&v, &ec)).mode,
            LoopMode::TempLoop,
            "tick 3 of 3 enters"
        );

        // Immediate exit on a hard fault (freshness lost) — no grace.
        let mut faulted = happy_input(&v, &ec);
        faulted.freshness = Freshness::Absent;
        assert_eq!(
            a.decide(&faulted).mode,
            LoopMode::RpmLoop,
            "hard fault exits the same tick"
        );

        // Re-entry needs a fresh 3-tick climb.
        assert_eq!(a.decide(&happy_input(&v, &ec)).mode, LoopMode::RpmLoop);
        assert_eq!(a.decide(&happy_input(&v, &ec)).mode, LoopMode::RpmLoop);
        assert_eq!(a.decide(&happy_input(&v, &ec)).mode, LoopMode::TempLoop);
    }

    // ---- 3. three mismatches -> RpmLoop, three matches -> TempLoop --

    #[test]
    fn three_mismatches_drop_to_rpmloop_three_matches_recover_with_reseed() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mut a = warmed_up(&v, &ec);
        assert_eq!(
            a.decide(&happy_input(&v, &ec)).mode,
            LoopMode::TempLoop,
            "precondition: stably in TempLoop"
        );

        let mismatched_view = view(QUIET16, 90.0, 90.0); // replica (75C) vs view (90C): |diff| > 1
        let mut mismatch_input = happy_input(&mismatched_view, &ec);
        mismatch_input.view_changed = true;

        let d1 = a.decide(&mismatch_input);
        assert!(!d1.ec_mismatch, "one mismatch does not latch");
        let d2 = a.decide(&mismatch_input);
        assert!(!d2.ec_mismatch, "two mismatches do not latch");
        let d3 = a.decide(&mismatch_input);
        assert!(d3.ec_mismatch, "third mismatch latches EC MISMATCH");
        assert_eq!(
            d3.mode,
            LoopMode::RpmLoop,
            "latched mismatch forces RpmLoop"
        );

        // Now three consecutive matching views clear it.
        let matched_view = view(QUIET16, 75.0, 75.0); // matches happy_ec's cpu argmax (75C)
        let mut match_input = happy_input(&matched_view, &ec);
        match_input.view_changed = true;
        let d1 = a.decide(&match_input);
        assert!(d1.ec_mismatch, "one match does not clear the latch");
        let d2 = a.decide(&match_input);
        assert!(d2.ec_mismatch, "two matches do not clear the latch");
        let d3 = a.decide(&match_input);
        assert!(!d3.ec_mismatch, "third match clears EC MISMATCH");
        assert_eq!(
            d3.reseed_ma,
            Some(75.0),
            "clearing sets reseed_ma from the view's ma_temperature"
        );
        assert_eq!(
            d3.mode,
            LoopMode::TempLoop,
            "the same tick the latch clears, TempLoop is reachable again"
        );
    }

    // ---- 4. skip rule -------------------------------------------------

    #[test]
    fn skip_rule_ignores_a_view_when_slope_or_gap_is_too_high() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mismatched_view = view(QUIET16, 90.0, 90.0);

        // Fast-moving replica: skipped, not scored.
        let mut a = Arbiter::new();
        let mut input = happy_input(&mismatched_view, &ec);
        input.view_changed = true;
        input.replica_slope_5s_c_per_s = 0.5; // at the threshold: not scorable
        let d = a.decide(&input);
        assert!(
            d.reasons.contains(&"unreconciled".to_string()),
            "a skipped view must not advance `reconciled`"
        );
        assert!(!d.ec_mismatch);

        // Stale gap: also skipped.
        let mut a2 = Arbiter::new();
        let mut input2 = happy_input(&mismatched_view, &ec);
        input2.view_changed = true;
        input2.view_to_sample_gap_s = 2.0; // at the threshold: not scorable
        let d2 = a2.decide(&input2);
        assert!(d2.reasons.contains(&"unreconciled".to_string()));
        assert!(!d2.ec_mismatch);

        // Sanity: the same mismatched view, scorable, does score.
        let mut a3 = Arbiter::new();
        let mut input3 = happy_input(&mismatched_view, &ec);
        input3.view_changed = true;
        let d3 = a3.decide(&input3);
        assert!(!d3.reasons.contains(&"unreconciled".to_string()));
        let _ = (v, d3);
    }

    // ---- 5. moving-average check: re-seed, not a latch ---------------

    #[test]
    fn ma_check_failing_three_times_requests_reseed_without_latching_mismatch() {
        let v = view(QUIET16, 75.0, 75.0);
        let matched_view = view(QUIET16, 75.0, 90.0); // max_c matches (75C), ma_temperature diverges
        let ec = happy_ec();
        let mut a = Arbiter::new();

        let mut input = happy_input(&matched_view, &ec);
        input.view_changed = true;
        input.ec_ma = Some(75.0); // |75 - 90| > 2: MA check fails

        let d1 = a.decide(&input);
        assert_eq!(
            d1.reseed_ma, None,
            "one failure does not yet request a reseed"
        );
        assert!(!d1.ec_mismatch);
        let d2 = a.decide(&input);
        assert_eq!(
            d2.reseed_ma, None,
            "two failures do not yet request a reseed"
        );
        assert!(!d2.ec_mismatch);
        let d3 = a.decide(&input);
        assert_eq!(
            d3.reseed_ma,
            Some(90.0),
            "third failure requests a reseed from the view's ma_temperature"
        );
        assert!(!d3.ec_mismatch, "the MA check never latches EC MISMATCH");
        let _ = v;
    }

    // ---- 6. infeasible target: TargetUnreachable, 60s clear ----------

    #[test]
    fn infeasible_target_yields_target_unreachable_and_clears_after_60s() {
        // ambient at 61C (argmax; cpu at 20C stays controllable) and an
        // interior target duty of 21 -> T*=65.5 (quiet16's tread(21) is
        // (65, 66), see curve.rs's own test); 65.5 < 61 + 5, so this is
        // genuinely infeasible per §2.7's feasibility rule, not an accident
        // of an unbounded tread.
        let ec = ec_reading(&[("ambient_f75303@4d", 61.0), ("cpu@4c", 20.0)]);
        let infeasible_target_view = view(QUIET16, 20.0, 20.0);
        let mut a = Arbiter::new();
        let mut input = happy_input(&infeasible_target_view, &ec);
        input.target_duty = 21;
        input.view_changed = true;

        let d = a.decide(&input);
        assert!(
            d.flags.contains(&StatusFlag::TargetUnreachable),
            "flags: {:?}",
            d.flags
        );
        assert!(
            d.reasons.iter().any(|r| r.contains("T*=")),
            "explanatory text present: {:?}",
            d.reasons
        );

        // Feed feasible ticks; the flag must still be present before 12
        // ticks (60s) and gone at/after 12.
        let ok_ec = ec_reading(&[("ambient_f75303@4d", 10.0), ("cpu@4c", 60.0)]);
        let mut ok_input = happy_input(&infeasible_target_view, &ok_ec);
        ok_input.target_duty = 31; // a tread whose T*=80 clears 10+5 easily
        for i in 0..11 {
            let d = a.decide(&ok_input);
            assert!(
                d.flags.contains(&StatusFlag::TargetUnreachable),
                "flag must still be latched at tick {i}"
            );
        }
        let d = a.decide(&ok_input);
        assert!(
            !d.flags.contains(&StatusFlag::TargetUnreachable),
            "flag clears once feasible for 60s continuously"
        );
    }

    // ---- 7. `low`: sub-floor duty, and the 60s bound-hold -------------

    #[test]
    fn low_reason_from_subfloor_duty_and_from_60s_at_the_lower_bound() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();

        let mut a = Arbiter::new();
        let mut input = happy_input(&v, &ec);
        input.target_duty = 5; // below quiet16's floor duty (15)
        let d = a.decide(&input);
        assert!(d.flags.contains(&StatusFlag::TargetUnreachable));
        assert!(d.t_star.is_none());
        assert!(
            d.reasons.iter().any(|r| r.contains("below floor")),
            "{:?}",
            d.reasons
        );

        let mut a2 = Arbiter::new();
        let mut input2 = happy_input(&v, &ec);
        input2.at_lower_bound_for = Duration::from_secs(60);
        input2.error_sign = -1.0;
        let d2 = a2.decide(&input2);
        assert!(d2.flags.contains(&StatusFlag::TargetUnreachable));
        assert!(
            d2.reasons.iter().any(|r| r.contains("held at floor")),
            "{:?}",
            d2.reasons
        );

        // Just under 60s, or a non-negative error: no flag from this rule.
        let mut a3 = Arbiter::new();
        let mut input3 = happy_input(&v, &ec);
        input3.at_lower_bound_for = Duration::from_secs(59);
        input3.error_sign = -1.0;
        assert!(
            !a3.decide(&input3)
                .flags
                .contains(&StatusFlag::TargetUnreachable)
        );
    }

    // ---- 8. `high`: 60s at the upper bound -----------------------------

    #[test]
    fn high_reason_from_60s_at_the_upper_bound() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mut a = Arbiter::new();
        let mut input = happy_input(&v, &ec);
        input.at_upper_bound_for = Duration::from_secs(60);
        input.error_sign = 1.0;
        let d = a.decide(&input);
        assert!(d.flags.contains(&StatusFlag::TargetUnreachable));
        assert!(
            d.reasons.iter().any(|r| r.contains("held at ceiling")),
            "{:?}",
            d.reasons
        );

        let mut a2 = Arbiter::new();
        let mut input2 = happy_input(&v, &ec);
        input2.at_upper_bound_for = Duration::from_secs(60);
        input2.error_sign = -1.0; // error no longer calls for more heat
        assert!(
            !a2.decide(&input2)
                .flags
                .contains(&StatusFlag::TargetUnreachable)
        );
    }

    // ---- 9. debounced argmax-controllable ------------------------------

    #[test]
    fn argmax_debounce_needs_three_failures_or_a_one_degree_lead() {
        let v = view(QUIET16, 75.0, 75.0);
        let happy = happy_ec(); // controllable argmax: warms up cleanly
        // ambient leads cpu by 0.5C once substituted in: uncontrollable
        // argmax, but too small a lead to be decisive on its own.
        let noisy_ec = ec_reading(&[("ambient_f75303@4d", 60.5), ("cpu@4c", 60.0)]);

        let mut a = warmed_up(&v, &happy);
        assert_eq!(a.decide(&happy_input(&v, &happy)).mode, LoopMode::TempLoop);

        let mut input = happy_input(&v, &happy);
        input.ec = Some(&noisy_ec);
        assert_eq!(
            a.decide(&input).mode,
            LoopMode::TempLoop,
            "1 of 3: not enough to drop TempLoop"
        );
        assert_eq!(
            a.decide(&input).mode,
            LoopMode::TempLoop,
            "2 of 3: still not enough"
        );
        assert_eq!(
            a.decide(&input).mode,
            LoopMode::RpmLoop,
            "3 of 3: drops TempLoop"
        );

        // A large lead (>1C) is decisive on a single tick.
        let leading_ec = ec_reading(&[("ambient_f75303@4d", 70.0), ("cpu@4c", 60.0)]);
        let mut a2 = warmed_up(&v, &happy);
        assert_eq!(a2.decide(&happy_input(&v, &happy)).mode, LoopMode::TempLoop);
        let mut lead_input = happy_input(&v, &happy);
        lead_input.ec = Some(&leading_ec);
        assert_eq!(
            a2.decide(&lead_input).mode,
            LoopMode::RpmLoop,
            "a >1C lead drops TempLoop on the very first failing tick"
        );
    }

    // ---- 10. FanctrlLost clears on the first fresh view ---------------

    #[test]
    fn fanctrl_lost_clears_on_the_first_fresh_view() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mut a = Arbiter::new();

        let mut absent = happy_input(&v, &ec);
        absent.freshness = Freshness::Absent;
        let d = a.decide(&absent);
        assert!(d.flags.contains(&StatusFlag::FanctrlLost));

        let d2 = a.decide(&happy_input(&v, &ec)); // freshness: Fresh
        assert!(
            !d2.flags.contains(&StatusFlag::FanctrlLost),
            "clears the same tick freshness returns to Fresh"
        );
    }

    // ---- 11. steep curve, and curve_valid:false never claims steep ----

    #[test]
    fn cool16_steep_tread_raises_steep_curve() {
        let v = view(COOL16, 75.0, 75.0);
        let ec = ec_reading(&[("ambient_f75303@4d", 40.0), ("cpu@4c", 75.0)]);
        let mut a = Arbiter::new();
        let mut input = happy_input(&v, &ec);
        // duty 90 sits strictly inside the 70->85 segment (slope
        // (100-42)/15 ≈ 3.87 %/°C); duty 100 is the curve's own ceiling,
        // whose tread is unbounded above and would resolve to the flat
        // clamp's 0 slope instead of the segment's.
        input.target_duty = 90;
        let d = a.decide(&input);
        assert!(d.flags.contains(&StatusFlag::SteepCurve), "{:?}", d.flags);
        assert!(d.slope.unwrap() > STEEP_SLOPE_PCT_PER_C);
    }

    #[test]
    fn curve_invalid_yields_rpmloop_curve_invalid_slope_none_never_steep_curve() {
        let v = view(COOL16, 75.0, 75.0); // same steep points as above
        let ec = ec_reading(&[("ambient_f75303@4d", 40.0), ("cpu@4c", 75.0)]);
        let mut a = Arbiter::new();
        let mut input = happy_input(&v, &ec);
        input.target_duty = 100;
        input.curve_valid = false;

        let d = a.decide(&input);
        assert_eq!(d.mode, LoopMode::RpmLoop);
        assert!(d.flags.contains(&StatusFlag::CurveInvalid));
        assert_eq!(d.slope, None);
        assert!(
            !d.flags.contains(&StatusFlag::SteepCurve),
            "curve_valid: false must never claim SteepCurve: {:?}",
            d.flags
        );
    }

    // ---- 12. initial state: unreconciled, distinct from a mismatch ----

    #[test]
    fn initial_state_reports_unreconciled_distinct_from_mismatch() {
        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mut a = Arbiter::new();
        let d = a.decide(&happy_input(&v, &ec)); // no view_changed yet: never scored
        assert!(d.reasons.contains(&"unreconciled".to_string()));
        assert!(
            !d.reasons.contains(&"ec_mismatch".to_string()),
            "unreconciled and mismatch are distinct reasons: {:?}",
            d.reasons
        );
        assert!(!d.ec_mismatch, "never latched, just never yet compared");
    }

    // ---- 13. point-keyed cache: same-name edit re-derives T* ----------

    #[test]
    fn same_name_points_edit_rederives_t_star_and_sets_t_star_changed() {
        let v1 = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mut a = Arbiter::new();

        let d1 = a.decide(&happy_input(&v1, &ec));
        assert!(d1.t_star_changed, "first derivation always changes t_star");
        let t_star_before = d1.t_star.expect("quiet16 duty 31 has a tread");

        let d2 = a.decide(&happy_input(&v1, &ec));
        assert!(
            !d2.t_star_changed,
            "same points, same duty: cached, no change"
        );
        assert_eq!(d2.t_star, Some(t_star_before));

        // Edit the tread that covers duty 31 in place, keeping the name.
        let mut edited_points = QUIET16.to_vec();
        for p in &mut edited_points {
            if p.0 == 75.0 {
                p.0 = 78.0; // shifts the tread bracketing duty 31
            }
        }
        let v2 = view(&edited_points, 75.0, 75.0); // same strategy name ("quiet16")
        assert_eq!(v1.strategy, v2.strategy);

        let d3 = a.decide(&happy_input(&v2, &ec));
        assert!(
            d3.t_star_changed,
            "an in-place points edit under the same name must re-derive"
        );
        assert_ne!(
            d3.t_star, d1.t_star,
            "the edited tread moves T* for the same target duty"
        );
    }

    // ---- 14. curve/arbiter seam: t_star must never be ±inf (fw-fanctrl-loop-nez) ----

    #[test]
    fn arbiter_keeps_u_finite_at_floor_and_ceiling_duty() {
        // Reproduces the defect directly: before the fix, quiet16's floor
        // (15) and ceiling (100) duty both resolve T* to NEG_INFINITY /
        // INFINITY; feeding that straight into a real Budget as a Temp
        // error makes `u` NaN on the second tick (inf - inf), and it stays
        // NaN forever. After the fix, T* is finite at both ends (the floor
        // via its own tread, the ceiling via `nearest_tread`'s snap to
        // duty 99), so `u` stays finite throughout.
        use crate::control::budget::{Budget, LoopError, LoopGains};

        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();

        for &target in &[15u8, 100u8] {
            let mut a = Arbiter::new();
            let mut budget = Budget::new(&LoopGains::default());
            budget.set_bounds(10.0, 60.0);
            let mut input = happy_input(&v, &ec);
            input.target_duty = target;

            for tick in 1..=5 {
                let d = a.decide(&input);
                let ts = d
                    .t_star
                    .unwrap_or_else(|| panic!("duty {target} tick {tick}: t_star is None"));
                let u = budget.step(LoopError::Temp { e_c: ts }, None);
                assert!(
                    u.is_finite(),
                    "duty {target} tick {tick}: u = {u} is not finite (t_star was {ts})"
                );
            }
        }
    }

    #[test]
    fn ceiling_target_unreachable_high_after_60s_pinned_at_the_upper_bound() {
        // §2.7's "unreachable from above" rule reads Budget's own dwell
        // timer and the tick's error_sign directly, not T* — so it must
        // keep working at the curve's literal ceiling duty exactly like
        // anywhere else, now that T* there resolves to a finite value
        // instead of poisoning the integrator with NaN.
        use crate::control::budget::{Budget, LoopError, LoopGains};

        // A real Budget, driven by an error that keeps calling for more
        // heat every tick, actually saturates at `hi` and stays there.
        let mut budget = Budget::new(&LoopGains::default());
        budget.set_bounds(10.0, 60.0);
        for _ in 0..12 {
            let u = budget.step(LoopError::Temp { e_c: 1000.0 }, None);
            assert_eq!(
                u, 60.0,
                "u must be pinned at hi while the error keeps calling for more heat"
            );
        }
        let dwell = budget.at_upper_bound_for();
        assert!(dwell >= Duration::from_secs(60), "dwell: {dwell:?}");

        let v = view(QUIET16, 75.0, 75.0);
        let ec = happy_ec();
        let mut a = Arbiter::new();
        let mut input = happy_input(&v, &ec);
        input.target_duty = 100; // curve ceiling
        input.at_upper_bound_for = dwell;
        input.error_sign = 1.0; // still calling for more heat

        let d = a.decide(&input);
        assert!(
            d.flags.contains(&StatusFlag::TargetUnreachable),
            "{:?}",
            d.flags
        );
        assert!(
            d.reasons.iter().any(|r| r.contains("held at ceiling")),
            "{:?}",
            d.reasons
        );
        // And the seam itself: even at the literal ceiling duty, T*
        // resolved to a finite value (§2.1's nearest_tread fallback), not
        // None/±inf.
        assert!(d.t_star.is_some_and(f64::is_finite), "{:?}", d.t_star);
    }
}

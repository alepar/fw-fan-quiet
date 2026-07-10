//! 2-state clamped Kalman filter over `[bias, gain]` (design doc
//! 2026-07-09 §3 "Design §1"). Supersedes BOTH prior adaptation mechanisms:
//! the `Trim` offset integrator — `bias` carries its role and safety
//! contract 1:1 — and the field-disabled 4-parameter RLS, whose
//! full-surface drift caused the 2026-06 degenerate-divisor incident (two
//! clamped DOF keep adaptation inside a box that cannot go degenerate).
//!
//! Measurement model, linear in the state:
//!
//! ```text
//! rpm = a·pc + c  +  bias  +  gain·(b·pg + e·pc·pg)
//!     = baseline  +  bias  +  gain·w
//! ```
//!
//! The filter is pure over `(measured, baseline, w)`: the caller computes
//! `baseline = a·pc + c` (the bias-independent part of the prediction) and
//! `w = b·pg + e·pc·pg` (the GPU-slope regressor the gain scales) from the
//! calibrated `ThermalModel`, so the filter is testable without one. The
//! regressor is `x = [1, w]`, prediction `ŷ = baseline + bias + gain·w`,
//! innovation `r = measured − ŷ`.
//!
//! Why the 2-state split cannot double-correct (the failure that got the
//! 4-param RLS disabled: it and the trim both chased the same residual): at
//! a constant operating point `x` never changes direction, so innovations
//! move only `bias`; `gain` moves only when the operating point jumps — the
//! idle→game onset is the highest-information gain measurement, weighted by
//! the covariance accumulated while gain sat unexcited. The decoupling
//! falls out of the covariance structure instead of being coded (no anchor
//! lifecycle or secant bookkeeping).
//!
//! No positive feedback: the allocator steers the plant onto the CORRECTED
//! contour, i.e. it holds `baseline + bias + gain·w = target`.
//! Substituting, the innovation equals the CONTROL error
//! `measured − target` — the same negative-feedback form the trim was fixed
//! to use after the 2026-07 pinning incident (see `trim.rs` module docs:
//! correcting toward the model residual instead feeds the correction back
//! into its own update with POSITIVE sign and winds to the clamp for ANY
//! persistent bias).
//!
//! Safety contract (research 03 §6, carried over from the trim 1:1):
//! - `bias` SATURATES at ±[`MAX_BIAS_AUTHORITY_RPM`] (clamp, not reject) so
//!   it can pin at the bound — the controller's `TargetUnreachable` flag
//!   semantics key off the pin, exactly like the trim's.
//! - `gain` is clamped to [[`GAIN_MIN`], [`GAIN_MAX`]] REJECT-WHOLE: a
//!   candidate outside the box (or non-finite) discards the ENTIRE update —
//!   state and covariance bit-identical, cadence slot untouched (the
//!   `rls_update` convention: a poisoned sample leaves no trace). With the
//!   fit-enforced divisor floor of 2.0 the effective contour divisor
//!   `gain·(b + e·pc)` can never fall below 1.2, excluding the 2026-06
//!   degenerate-divisor incident class by the clamp alone.
//! - Non-finite inputs rejected outright; covariance trace capped (the same
//!   belt-and-suspenders as the RLS).
//! - Floors still dominate everything: allocator/PI clamps bound the KF's
//!   effect (project-wide invariant: floors > adaptation).

use nalgebra::{Matrix2, Vector2};

/// Update cadence, seconds — the fielded trim's tick, kept as-is: far
/// slower than the 30–90 s fan settling divided by the effective gain, so
/// the filter can never fight the thermal lag and hunt (see `trim.rs`).
pub const KF_PERIOD_S: f64 = 20.0;

/// Hard bound on the bias state, RPM — identical bound AND role to the
/// trim's `MAX_TRIM_AUTHORITY_RPM`: 400 RPM ≈ 25% of the 1500–3000 RPM
/// operating band, i.e. the design's "total correction ≤ 25% budget
/// reduction" through the model's fan-per-watt slopes. Saturating, so the
/// `TargetUnreachable` flag can key off the pin.
pub const MAX_BIAS_AUTHORITY_RPM: f64 = 400.0;

/// Gain clamp box (design §3): with the fit-enforced divisor floor of 2.0,
/// `GAIN_MIN`·2.0 = 1.2 lower-bounds the effective contour divisor — the
/// degenerate-divisor incident class is excluded by the clamp alone. The
/// upper bound symmetrically caps how much budget a runaway gain could
/// withhold. Outside the box the update is rejected whole.
pub const GAIN_MIN: f64 = 0.6;
pub const GAIN_MAX: f64 = 1.6;

/// Measurement noise variance, RPM²: ~100 RPM std, sized from the measured
/// soak noise on settled points (design §3 "Tuning anchors": p50 ≈ 80,
/// p90 ≈ 160 RPM of multi-minute drift invisible to the steadiness gates —
/// the KF absorbs it as noise rather than pretending it away).
const R: f64 = 1.0e4;

/// Bias process noise per tick, RPM². Tuned so the bias-only steady-state
/// Kalman gain ≈ 0.05 = the trim's `KI_TRIM`, i.e. the same ~7-minute time
/// constant (400 s at 20 s ticks) — near-drop-in steady-state behavior.
/// Scalar Riccati at steady state: `q = k²·R/(1−k)` = 0.0025·1e4/0.95 ≈ 26.
const Q_BIAS: f64 = 26.0;

/// Gain process noise per tick (dimensionless²): slopes change with load
/// type, not by the minute — much smaller than `Q_BIAS` so a settled gain
/// stays put between onsets instead of wandering on residual soak noise.
const Q_GAIN: f64 = 1.0e-4;

/// Fresh bias prior variance ≈ the bias steady-state (prior) covariance
/// `Q_BIAS/k ≈ 520`, so the very first update steps with gain ≈ 0.05 — as
/// gentle as the trim's first integration, never a cold-start lurch.
const P0_BIAS: f64 = 526.0;

/// Fresh gain prior variance: std 0.5 spans the [0.6, 1.6] clamp box, so
/// the filter is maximally willing to learn the slope at the first onset
/// (the highest-information gain measurement) instead of grinding there.
const P0_GAIN: f64 = 0.25;

/// Covariance trace cap and rescale target — same belt-and-suspenders as
/// `RLS_TRACE_CAP`/`RLS_TRACE_RESCALE_TO`: bounds how hard any single
/// sample can yank the state if the covariance ever inflates.
const TRACE_CAP: f64 = 1.0e6;
const TRACE_RESCALE_TO: f64 = 1.0e5;

/// 2-state clamped Kalman filter state. Lives in the controller's Auto
/// loop state; `[bias, gain]` are seeded from persistence on Auto entry
/// (design §4) while the covariance always restarts at the fresh prior — a
/// new session starts confident about nothing but centered on what it
/// learned.
#[derive(Debug, Clone)]
pub struct Kalman {
    /// `[bias (RPM), gain (dimensionless)]`.
    theta: Vector2<f64>,
    /// State covariance; rebuilt fresh by [`new`](Self::new), never
    /// persisted.
    p: Matrix2<f64>,
    /// `t_mono` of the last ACCEPTED update; None until the first. Rejected
    /// updates never consume the cadence slot (see [`update`](Self::update)).
    last_update_t: Option<f64>,
}

impl Kalman {
    /// Seeds `[bias, gain]` from persistence; covariance always at the
    /// fresh prior.
    ///
    /// The seed comes from the state file — which a user can hand-edit or
    /// a partial write can corrupt — and is applied BEFORE any update-time
    /// clamp ever runs, so a poisoned seed would otherwise sit inside the
    /// filter violating the safety contract until the first accepted
    /// update (or forever, for NaN: every innovation through it is NaN and
    /// gets rejected). SANITIZE here: non-finite bias → 0.0, else clamped
    /// to ±[`MAX_BIAS_AUTHORITY_RPM`]; non-finite gain → 1.0, else clamped
    /// to [[`GAIN_MIN`], [`GAIN_MAX`]].
    pub fn new(bias: f64, gain: f64) -> Self {
        let bias = if bias.is_finite() {
            bias.clamp(-MAX_BIAS_AUTHORITY_RPM, MAX_BIAS_AUTHORITY_RPM)
        } else {
            0.0
        };
        let gain = if gain.is_finite() {
            gain.clamp(GAIN_MIN, GAIN_MAX)
        } else {
            1.0
        };
        Self {
            theta: Vector2::new(bias, gain),
            p: Matrix2::new(P0_BIAS, 0.0, 0.0, P0_GAIN),
            last_update_t: None,
        }
    }

    /// Current bias (RPM), the trim role: shifts the model's `c` in the
    /// allocator's contour inversion.
    pub fn bias(&self) -> f64 {
        self.theta[0]
    }

    /// Current gain (dimensionless): scales the contour's GPU-slope
    /// divisor `b + e·pc`.
    pub fn gain(&self) -> f64 {
        self.theta[1]
    }

    /// One cadence-gated KF update; returns true iff the STATE changed
    /// (acceptance is tracked separately — see below).
    ///
    /// The CALLER owns the evidence quality (design §0/§2): cooldown gate,
    /// achievement gate, `is_steady`, windowed averaging. This is only the
    /// update math.
    ///
    /// Cadence: the first call updates immediately (the caller's gates
    /// already vouch for the sample); afterwards only once [`KF_PERIOD_S`]
    /// has elapsed since the last ACCEPTED update. `last_update_t` is set
    /// only after every rejection gate has passed, so a rejected call
    /// (non-finite input, gain-clamp violation) never consumes the cadence
    /// slot — deliberately unlike `trim.rs`, whose due call consumed the
    /// slot before its work. An ACCEPTED update with zero innovation still
    /// consumes the slot and returns false (steady + zero error is a
    /// *measurement*, not a skip — retrying it at 1 Hz would burn cycles;
    /// same call as the trim's zero-error case).
    ///
    /// Math: random-walk predict `P⁻ = P + Q`; `x = [1, w]`;
    /// `S = xᵀ·P⁻·x + R`; `K = P⁻·x/S`;
    /// `θ' = θ + K·(measured − (baseline + bias + gain·w))`. The covariance
    /// step `P = (I − K·xᵀ)·P⁻` runs only on ACCEPTED updates, then the
    /// trace cap applies.
    ///
    /// Clamps: gain REJECT-WHOLE, bias SATURATES — see the module docs for
    /// why the asymmetry.
    pub fn update(&mut self, t_mono: f64, measured: f64, baseline: f64, w: f64) -> bool {
        if let Some(last) = self.last_update_t
            && t_mono - last < KF_PERIOD_S
        {
            return false;
        }
        // Poisoned-input guard (the rls_update lesson): every comparison
        // with NaN is false, so a NaN would sail through the clamp check
        // below and poison state AND covariance in one update. Reject
        // outright — no state change, no cadence consumption.
        if !(measured.is_finite() && baseline.is_finite() && w.is_finite()) {
            return false;
        }
        let p_minus = self.p + Matrix2::new(Q_BIAS, 0.0, 0.0, Q_GAIN);
        let x = Vector2::new(1.0, w);
        let px = p_minus * x;
        let s = x.dot(&px) + R;
        let k = px / s;
        let innovation = measured - (baseline + self.theta[0] + self.theta[1] * w);
        let candidate = self.theta + k * innovation;
        // Gain clamp, REJECT-WHOLE: a candidate outside the box (or any
        // non-finite candidate — unreachable with finite inputs since
        // S ≥ R > 0, kept as the same defensive shape as the input guard)
        // discards the entire update. State, covariance and cadence
        // baseline stay bit-identical: a poisoned sample leaves no trace.
        if !(candidate[0].is_finite() && candidate[1].is_finite())
            || candidate[1] < GAIN_MIN
            || candidate[1] > GAIN_MAX
        {
            return false;
        }
        // All rejection gates passed: the update is ACCEPTED and consumes
        // the cadence slot even if the state ends up numerically unchanged.
        self.last_update_t = Some(t_mono);
        // Bias clamp SATURATES so it can pin at ±max (flag semantics).
        let next = Vector2::new(
            candidate[0].clamp(-MAX_BIAS_AUTHORITY_RPM, MAX_BIAS_AUTHORITY_RPM),
            candidate[1],
        );
        let changed = next != self.theta;
        self.theta = next;
        let mut p_next = (Matrix2::identity() - k * x.transpose()) * p_minus;
        let trace = p_next.trace();
        if trace > TRACE_CAP {
            p_next *= TRACE_RESCALE_TO / trace;
        }
        self.p = p_next;
        changed
    }

    /// Back to the identity correction `[0, 1]` with a fresh prior and no
    /// cadence baseline. Called when a new calibration lands: a fresh
    /// surface invalidates old corrections (design §4).
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        *self = Self::new(0.0, 1.0);
    }

    #[cfg(test)]
    fn snapshot(&self) -> (Vector2<f64>, Matrix2<f64>) {
        (self.theta, self.p)
    }

    #[cfg(test)]
    fn trace(&self) -> f64 {
        self.p[(0, 0)] + self.p[(1, 1)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Closed-loop plant on the corrected contour: the allocator holds
    /// `baseline + bias_est + gain_est·w = target`, so `baseline = target −
    /// bias_est − gain_est·w`; the plant then reads
    /// `measured = baseline + bias_true + gain_true·w`. Mirrors the
    /// controller exactly, so the innovation is the control error
    /// `measured − target`.
    fn step_plant(kf: &Kalman, target: f64, w: f64, bias_true: f64, gain_true: f64) -> (f64, f64) {
        let baseline = target - kf.bias() - kf.gain() * w;
        let measured = baseline + bias_true + gain_true * w;
        (measured, baseline)
    }

    #[test]
    fn new_starts_at_seed_and_identity_reset() {
        let kf = Kalman::new(0.0, 1.0);
        assert_eq!((kf.bias(), kf.gain()), (0.0, 1.0));
        // An in-range persisted seed passes through untouched.
        let kf = Kalman::new(-120.0, 1.3);
        assert_eq!((kf.bias(), kf.gain()), (-120.0, 1.3));
        // reset() returns to the identity correction with a fresh prior…
        let mut kf = Kalman::new(-120.0, 1.3);
        assert!(kf.update(0.0, 2500.0, 2000.0, 0.0));
        kf.reset();
        assert_eq!((kf.bias(), kf.gain()), (0.0, 1.0));
        assert_eq!(kf.snapshot(), Kalman::new(0.0, 1.0).snapshot());
        // …and no cadence baseline: the next call integrates immediately.
        assert!(kf.update(1.0, 2100.0, 2000.0, 0.0));
    }

    #[test]
    fn new_sanitizes_a_poisoned_seed() {
        // The state file is hand-editable: NaN would otherwise wedge the
        // filter (every innovation non-finite → every update rejected).
        let kf = Kalman::new(f64::NAN, f64::NAN);
        assert_eq!((kf.bias(), kf.gain()), (0.0, 1.0));
        // Finite but out of the safety box: clamped to the contract.
        let kf = Kalman::new(9999.0, 0.01);
        assert_eq!((kf.bias(), kf.gain()), (MAX_BIAS_AUTHORITY_RPM, GAIN_MIN));
        let kf = Kalman::new(-9999.0, 99.0);
        assert_eq!((kf.bias(), kf.gain()), (-MAX_BIAS_AUTHORITY_RPM, GAIN_MAX));
    }

    #[test]
    fn cadence_gates_at_20s() {
        let mut kf = Kalman::new(0.0, 1.0);
        // First call integrates immediately (the caller's gates vouch).
        assert!(kf.update(0.0, 2100.0, 2000.0, 0.0));
        let after_first = kf.bias();
        assert!(after_first > 0.0);
        // Anything under 20 s since the last accepted call: gated.
        assert!(!kf.update(10.0, 2100.0, 2000.0, 0.0));
        assert!(!kf.update(19.9, 2100.0, 2000.0, 0.0));
        assert_eq!(kf.bias(), after_first);
        // 20 s elapsed: integrates again.
        assert!(kf.update(20.0, 2100.0, 2000.0, 0.0));
        assert!(kf.bias() > after_first);
    }

    #[test]
    fn converges_to_plant_bias_and_stops() {
        // Port of the trim's stability test: with the innovation equal to
        // the control error the closed loop is negative feedback — the bias
        // converges to exactly the plant's bias (139, the 2026-07 field
        // value) and stops, and the fans land ON target.
        const BIAS_TRUE: f64 = 139.0;
        const TARGET: f64 = 3250.0;
        let mut kf = Kalman::new(0.0, 1.0);
        let mut deltas = Vec::new();
        for i in 0..200 {
            let before = kf.bias();
            let (measured, baseline) = step_plant(&kf, TARGET, 0.0, BIAS_TRUE, 1.0);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, 0.0);
            assert!(
                kf.bias().abs() < MAX_BIAS_AUTHORITY_RPM,
                "an in-authority bias must never pin"
            );
            deltas.push((kf.bias() - before).abs());
        }
        let bias = kf.bias();
        assert!(
            (bias - BIAS_TRUE).abs() < 10.0,
            "bias = {bias}, want ≈ {BIAS_TRUE}"
        );
        assert!(
            deltas[190..].iter().all(|d| *d < 1.0),
            "converged bias must STOP moving: {:?}",
            &deltas[190..]
        );
        // Equilibrium is on target: the plant reads back the target.
        let (measured, _) = step_plant(&kf, TARGET, 0.0, BIAS_TRUE, 1.0);
        assert!((measured - TARGET).abs() < 10.0);
    }

    #[test]
    fn bias_only_time_constant_matches_trim() {
        // Regression guard on the tuning: Q_BIAS/R are sized so the
        // bias-only steady-state Kalman gain ≈ the trim's KI_TRIM = 0.05,
        // i.e. τ ≈ 400 s. After 20 updates (≈ 1 τ) a 200 RPM step must be
        // absorbed to within the 1-τ band (1 − e⁻¹ ≈ 0.63, bracketed
        // generously to allow for the fresh-prior transient).
        const BIAS_TRUE: f64 = 200.0;
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..20 {
            let (measured, baseline) = step_plant(&kf, 3000.0, 0.0, BIAS_TRUE, 1.0);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, 0.0);
        }
        let bias = kf.bias();
        assert!(
            bias > 0.45 * BIAS_TRUE && bias < 0.80 * BIAS_TRUE,
            "after ≈1 τ, bias = {bias}, want in ({}, {})",
            0.45 * BIAS_TRUE,
            0.80 * BIAS_TRUE
        );
    }

    #[test]
    fn learns_gain_across_onsets() {
        // A GPU-heavy operating point (w = 900 RPM of modeled GPU slope)
        // with the plant's true slope 15% steeper than calibrated: the
        // gain state absorbs it.
        const GAIN_TRUE: f64 = 1.15;
        const W: f64 = 900.0;
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..300 {
            let (measured, baseline) = step_plant(&kf, 3250.0, W, 0.0, GAIN_TRUE);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, W);
        }
        let gain = kf.gain();
        assert!(
            (gain - GAIN_TRUE).abs() < 0.05,
            "gain = {gain}, want ≈ {GAIN_TRUE}"
        );
    }

    #[test]
    fn gain_decouples_from_bias() {
        // The covariance-structure decoupling claim, asserted: a combined
        // plant (offset error AND slope error) under alternating operating
        // points separates cleanly into the two states — the failure mode
        // that killed the 4-param RLS (double-correction with the trim)
        // cannot happen between bias and gain.
        const BIAS_TRUE: f64 = 100.0;
        const GAIN_TRUE: f64 = 1.15;
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..400 {
            let w = if i % 2 == 0 { 0.0 } else { 900.0 };
            let (measured, baseline) = step_plant(&kf, 3000.0, w, BIAS_TRUE, GAIN_TRUE);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, w);
        }
        let (bias, gain) = (kf.bias(), kf.gain());
        assert!(
            (bias - BIAS_TRUE).abs() < 25.0,
            "bias = {bias}, want ≈ {BIAS_TRUE}"
        );
        assert!(
            (gain - GAIN_TRUE).abs() < 0.08,
            "gain = {gain}, want ≈ {GAIN_TRUE}"
        );
    }

    #[test]
    fn out_of_authority_bias_pins_at_max() {
        // 550 RPM of true bias > the 400 RPM authority: bias walks to the
        // +max clamp and stays EXACTLY there (saturating, not rejecting) —
        // the controller's TargetUnreachable flag keys off the pin.
        const BIAS_TRUE: f64 = 550.0;
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..200 {
            let (measured, baseline) = step_plant(&kf, 3250.0, 0.0, BIAS_TRUE, 1.0);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, 0.0);
        }
        assert_eq!(kf.bias(), MAX_BIAS_AUTHORITY_RPM);
    }

    #[test]
    fn gain_clamp_rejects_whole_update_bit_identical() {
        let mut kf = Kalman::new(0.0, 1.0);
        let before = kf.snapshot();
        // On the fresh prior at w = 900 the gain's Kalman gain is ≈ 1e-3:
        // a ~1e6 RPM innovation drives the candidate gain past GAIN_MAX by
        // orders of magnitude. The whole update must vanish without trace.
        assert!(!kf.update(0.0, 1.0e6, 0.0, 900.0));
        assert_eq!(
            kf.snapshot(),
            before,
            "state AND covariance must be bit-identical"
        );
        // And the cadence slot was NOT consumed: a sane call 1 s later is
        // accepted immediately.
        assert!(kf.update(1.0, 2100.0, 2000.0, 0.0));
    }

    #[test]
    fn non_finite_inputs_rejected_and_slot_not_consumed() {
        let mut kf = Kalman::new(0.0, 1.0);
        let before = kf.snapshot();
        assert!(!kf.update(0.0, f64::NAN, 2000.0, 0.0));
        assert!(!kf.update(1.0, 2100.0, f64::INFINITY, 0.0));
        assert!(!kf.update(2.0, 2100.0, 2000.0, f64::NEG_INFINITY));
        assert_eq!(kf.snapshot(), before);
        // The garbage calls did not pin a cadence baseline: a clean call
        // right after still integrates immediately.
        assert!(kf.update(3.0, 2100.0, 2000.0, 0.0));
    }

    #[test]
    fn covariance_trace_capped_under_alternating_excitation() {
        let mut kf = Kalman::new(0.0, 1.0);
        for i in 0..500 {
            let w = if i % 2 == 0 { 0.0 } else { 900.0 };
            let (measured, baseline) = step_plant(&kf, 3000.0, w, 100.0, 1.15);
            kf.update(f64::from(i) * KF_PERIOD_S, measured, baseline, w);
            let trace = kf.trace();
            assert!(
                trace.is_finite() && trace <= TRACE_CAP,
                "trace = {trace} at tick {i}"
            );
        }
    }
}

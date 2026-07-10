//! Bounded ambient trim integrator (design doc §3; research 03 §6 "bounded
//! integrator authority — the key safeguard").
//!
//! SUPERSEDED (2026-07, adaptation v2): production adaptation is the
//! 2-state Kalman filter in `kalman.rs`, whose `bias` state carries this
//! integrator's role and safety contract 1:1. This module survives
//! test-only (`#[cfg(test)]` in `control/mod.rs`) as the allocator's
//! field-replay simulation stand-in (see `allocator.rs::simulate_field_cycle`)
//! and as the historical reference for the control-error-not-residual
//! lesson below.
//!
//! Slowest tier of the control cascade: integrates the CONTROL error
//! (measured − target) fan RPM into an offset added to the thermal model's
//! `c` (via `gpu_watts_on_contour`'s trim argument), so ambient/airflow/dust
//! drift is absorbed without refitting the model. Positive offset ⇒ the fans
//! persistently run over the target at the commanded budget (the model
//! under-predicts) ⇒ the contour shifts DOWN and commands fewer watts.
//!
//! Why the CONTROL error and not the model residual (measured − predicted):
//! the allocator steers the operating point onto the TRIMMED contour, i.e.
//! it holds `predicted = target − trim`. Substituting, the model-error
//! integrand equals `(measured − target) + trim` — the trim feeds back into
//! its own update with POSITIVE sign, so for ANY persistent model bias it
//! winds to the ±max clamp and its equilibrium (measured − target = −trim)
//! is unstable: the 2026-07 field session had it pinned at +400 with the
//! fans parked stable 260 RPM BELOW target and ~15 W of budget withheld.
//! With the control error the closed loop is `error = bias − trim` (bias =
//! the model's offset error at the operating point): NEGATIVE feedback, the
//! offset converges geometrically to exactly the model's bias, the error
//! goes to zero, and the fans land ON target.
//!
//! The safety contract (research 03 §6): the offset saturates at
//! [`MAX_TRIM_AUTHORITY_RPM`], so a wrong model / blocked intake can cut the
//! budget by a bounded amount only — and the allocator's floors clamp even
//! that (floors > trim, project-wide invariant). Once the +max is pinned the
//! controller surfaces `StatusFlag::TargetUnreachable` instead of silently
//! collapsing performance.

/// Integration cadence, seconds (design §3: "every 20 s, minutes-scale time
/// constant"). Gated on `t_mono` like the controller's reassert; must stay
/// far slower than the 30–90 s fan settling divided by the gain — see
/// [`KI_TRIM`].
pub const TRIM_PERIOD_S: f64 = 20.0;

/// Integrator gain, dimensionless (RPM offset per RPM of error, per update).
/// With the 20 s cadence the effective time constant is TRIM_PERIOD_S /
/// KI_TRIM = 400 s ≈ 7 minutes — "minutes-scale", per design §3, so the
/// integrator can never fight the 30–90 s thermal lag and hunt.
pub const KI_TRIM: f64 = 0.05;

/// Hard bound on the offset, RPM (research 03 §6: cap the *total* cumulative
/// correction). 400 RPM ≈ 25% of the typical 1500–3000 RPM operating band,
/// which — through the model's fan-per-watt slopes — corresponds to the
/// design's "total correction ≤ 25% budget reduction" (e.g. at b ≈ 15 RPM/W
/// a 400 RPM trim is a ~27 W GPU cut out of the ~100 W envelope).
pub const MAX_TRIM_AUTHORITY_RPM: f64 = 400.0;

/// Bounded trim integrator state. Owned by the controller's Auto-mode loop
/// state, so it drops (resets) on Auto exit and starts fresh on re-entry;
/// it deliberately survives fan-target changes (ambient didn't change).
#[derive(Debug, Clone, Default)]
pub struct Trim {
    /// Cumulative offset added to the model's `c`, clamped to
    /// ±[`MAX_TRIM_AUTHORITY_RPM`].
    offset_rpm: f64,
    /// `t_mono` of the last cadence-consuming update; None until the first.
    last_update_t: Option<f64>,
}

impl Trim {
    pub fn new() -> Self {
        Self::default()
    }

    /// Current offset (RPM), to pass as `gpu_watts_on_contour`'s trim arg.
    pub fn offset_rpm(&self) -> f64 {
        self.offset_rpm
    }

    /// One gated integration step; returns true iff the offset changed.
    ///
    /// The CALLER owns the steadiness verdict: it must only call this when
    /// the fan window is steady (`calib::steady::is_steady`) and the sample
    /// is fan-valid — never integrate on transients or lost sensors.
    ///
    /// Cadence: the FIRST steady call integrates immediately (the caller's
    /// gate already guarantees 20 steady samples behind it, so the error is
    /// trustworthy) and pins the baseline; subsequent calls integrate only
    /// once [`TRIM_PERIOD_S`] has elapsed since the last consuming call.
    /// A due call with zero error still advances the baseline (steady +
    /// zero error is a *measurement*, not a skip — retrying it at 1 Hz
    /// would just burn cycles).
    ///
    /// Anti-windup: plain clamp to ±[`MAX_TRIM_AUTHORITY_RPM`]. There is no
    /// hidden state to wind past the bound (the offset IS the state), so
    /// back-calculation buys nothing at this 20 s cadence — an error sign
    /// flip walks back from the clamp on the very next update.
    ///
    /// Equilibrium: given the allocator holds the plant on the trimmed
    /// contour (`predicted = target − trim`), the integrated error equals
    /// `bias − trim`, so the offset converges to exactly the model's bias
    /// at the operating point and stops (see the module docs for why the
    /// model residual instead would be positive feedback). The clamp and
    /// the controller's `TargetUnreachable` flag engage only when the true
    /// bias exceeds the ±max authority.
    ///
    /// Production code goes through [`update_scaled`](Self::update_scaled)
    /// (the controller always passes an explicit gain scale); this plain
    /// form is kept as the canonical unscaled API and test baseline.
    #[allow(dead_code)]
    pub fn update(&mut self, t_mono: f64, measured_rpm: f64, target_rpm: f64) -> bool {
        self.update_scaled(t_mono, measured_rpm, target_rpm, 1.0)
    }

    /// [`update`](Self::update) with the gain scaled by `ki_scale` for THIS
    /// step: the effective gain is `KI_TRIM · ki_scale`. The controller
    /// passes 0.5 while the trust monitor reports `ModelDistrust` (Task 27)
    /// — keep correcting the acoustics, but gently, since the residual
    /// evidence is suspect. Cadence, clamping and validity handling are
    /// identical to `update`.
    pub fn update_scaled(
        &mut self,
        t_mono: f64,
        measured_rpm: f64,
        target_rpm: f64,
        ki_scale: f64,
    ) -> bool {
        if let Some(last) = self.last_update_t
            && t_mono - last < TRIM_PERIOD_S
        {
            return false;
        }
        // Defensive: a non-finite reading must neither poison the offset
        // nor consume the cadence slot (callers already gate validity).
        if !measured_rpm.is_finite() || !target_rpm.is_finite() {
            return false;
        }
        self.last_update_t = Some(t_mono);
        let next = (self.offset_rpm + KI_TRIM * ki_scale * (measured_rpm - target_rpm))
            .clamp(-MAX_TRIM_AUTHORITY_RPM, MAX_TRIM_AUTHORITY_RPM);
        let changed = next != self.offset_rpm;
        self.offset_rpm = next;
        changed
    }

    /// Back to zero offset and no baseline. The controller resets by
    /// dropping its whole Auto loop state instead; kept as the explicit API
    /// for callers that hold on to a Trim.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_starts_at_zero() {
        assert_eq!(Trim::new().offset_rpm(), 0.0);
    }

    #[test]
    fn first_steady_call_integrates_immediately() {
        let mut t = Trim::new();
        // The caller's steadiness gate is the trust source: the first call
        // may integrate right away (documented decision).
        assert!(t.update(100.0, 2100.0, 2000.0));
        assert!((t.offset_rpm() - KI_TRIM * 100.0).abs() < 1e-12);
    }

    #[test]
    fn integrates_at_20s_cadence_only() {
        let mut t = Trim::new();
        assert!(t.update(0.0, 2100.0, 2000.0)); // baseline + first step
        let after_first = t.offset_rpm();
        // Anything under 20 s since the last consuming call: no update.
        assert!(!t.update(10.0, 2100.0, 2000.0));
        assert!(!t.update(19.9, 2100.0, 2000.0));
        assert_eq!(t.offset_rpm(), after_first);
        // 20 s elapsed: integrates again.
        assert!(t.update(20.0, 2100.0, 2000.0));
        assert!((t.offset_rpm() - 2.0 * KI_TRIM * 100.0).abs() < 1e-12);
    }

    #[test]
    fn zero_error_advances_the_baseline_without_change() {
        let mut t = Trim::new();
        assert!(t.update(0.0, 2100.0, 2000.0));
        // Due update with zero error: no change reported...
        assert!(!t.update(20.0, 2000.0, 2000.0));
        // ...but it consumed the cadence slot: 10 s later is still gated.
        assert!(!t.update(30.0, 2100.0, 2000.0));
        assert!(t.update(40.0, 2100.0, 2000.0));
    }

    #[test]
    fn integrates_control_error_measured_minus_target() {
        // One step of the field numbers: fans 150 RPM over a 3250 target →
        // offset moves by exactly KI_TRIM · 150.
        let mut t = Trim::new();
        assert!(t.update(0.0, 3400.0, 3250.0));
        assert!((t.offset_rpm() - KI_TRIM * 150.0).abs() < 1e-12);
    }

    #[test]
    fn sign_measured_above_target_is_positive() {
        // Fans over target (model under-predicts at this budget) → positive
        // offset → contour commands FEWER watts (it subtracts the trim).
        let mut t = Trim::new();
        t.update(0.0, 2300.0, 2000.0);
        assert!(t.offset_rpm() > 0.0);
        let mut t = Trim::new();
        t.update(0.0, 1700.0, 2000.0);
        assert!(t.offset_rpm() < 0.0);
    }

    /// Closed-loop plant on the trimmed contour: the allocator holds
    /// `predicted = target − trim`, so with a fixed model bias the fans
    /// settle at `measured = target − trim + bias` between trim updates.
    fn plant(target: f64, trim: &Trim, bias: f64) -> f64 {
        target - trim.offset_rpm() + bias
    }

    #[test]
    fn converges_to_model_bias_and_stops() {
        // The stability fix, asserted at the update law: integrating the
        // CONTROL error makes the closed loop `error = bias − trim` —
        // negative feedback, geometric convergence to trim == bias (139,
        // the 2026-07 field bias). The old model-error integrand was
        // `(measured − target) + trim`: positive feedback that wound to
        // the clamp for ANY persistent bias.
        const BIAS: f64 = 139.0;
        const TARGET: f64 = 3250.0;
        let mut t = Trim::new();
        let mut deltas = Vec::new();
        for i in 0..100 {
            let before = t.offset_rpm();
            let measured = plant(TARGET, &t, BIAS);
            t.update(f64::from(i) * TRIM_PERIOD_S, measured, TARGET);
            assert!(
                t.offset_rpm().abs() < MAX_TRIM_AUTHORITY_RPM,
                "an in-authority bias must never pin the trim"
            );
            deltas.push((t.offset_rpm() - before).abs());
        }
        let trim = t.offset_rpm();
        assert!((trim - BIAS).abs() < 10.0, "trim = {trim}, want ≈ {BIAS}");
        assert!(
            deltas[90..].iter().all(|d| *d < 1.0),
            "converged trim must STOP moving: {:?}",
            &deltas[90..]
        );
        // And the equilibrium is on target: the plant reads back the bias.
        assert!((plant(TARGET, &t, BIAS) - TARGET).abs() < 10.0);
    }

    #[test]
    fn bias_beyond_authority_pins_at_max() {
        // 550 RPM of true bias > the 400 RPM authority: the trim walks to
        // the +max clamp and stays — the caller's TargetUnreachable flag is
        // now accurate (fans genuinely over target at the maximum cut).
        const BIAS: f64 = 550.0;
        const TARGET: f64 = 3250.0;
        let mut t = Trim::new();
        for i in 0..100 {
            let measured = plant(TARGET, &t, BIAS);
            t.update(f64::from(i) * TRIM_PERIOD_S, measured, TARGET);
        }
        assert_eq!(t.offset_rpm(), MAX_TRIM_AUTHORITY_RPM);
    }

    #[test]
    fn sustained_error_walks_to_cap_and_stops() {
        let mut t = Trim::new();
        // +300 RPM error = +15 RPM per update: reaches +400 in 27 updates.
        for i in 0..40 {
            t.update(f64::from(i) * 20.0, 2300.0, 2000.0);
            assert!(t.offset_rpm() <= MAX_TRIM_AUTHORITY_RPM);
        }
        assert_eq!(t.offset_rpm(), MAX_TRIM_AUTHORITY_RPM);
        // Pinned: further same-sign updates change nothing (returns false).
        assert!(!t.update(1000.0, 2300.0, 2000.0));
        assert_eq!(t.offset_rpm(), MAX_TRIM_AUTHORITY_RPM);
    }

    #[test]
    fn no_windup_past_the_clamp_sign_flip_walks_back_immediately() {
        let mut t = Trim::new();
        // Slam into the +400 clamp with huge errors for a long time.
        for i in 0..50 {
            t.update(f64::from(i) * 20.0, 9000.0, 2000.0);
        }
        assert_eq!(t.offset_rpm(), MAX_TRIM_AUTHORITY_RPM);
        // First opposite-sign update moves off the clamp by exactly one
        // step — no hidden wound-up state to burn off first.
        assert!(t.update(2000.0, 1900.0, 2000.0));
        assert!((t.offset_rpm() - (MAX_TRIM_AUTHORITY_RPM - KI_TRIM * 100.0)).abs() < 1e-12);
    }

    #[test]
    fn clamps_at_negative_max_too() {
        let mut t = Trim::new();
        for i in 0..50 {
            t.update(f64::from(i) * 20.0, 2000.0, 9000.0);
        }
        assert_eq!(t.offset_rpm(), -MAX_TRIM_AUTHORITY_RPM);
    }

    #[test]
    fn non_finite_inputs_are_ignored_and_do_not_consume_the_slot() {
        let mut t = Trim::new();
        assert!(!t.update(0.0, f64::NAN, 2000.0));
        assert!(!t.update(1.0, 2100.0, f64::INFINITY));
        assert_eq!(t.offset_rpm(), 0.0);
        // The garbage calls did not pin a baseline: a clean first call
        // still integrates immediately.
        assert!(t.update(2.0, 2100.0, 2000.0));
    }

    #[test]
    fn update_scaled_scales_the_gain_per_step() {
        // Same error, ki_scale 0.5 → exactly half the movement (the Task-27
        // distrust behavior), and the cadence slot is consumed as usual.
        let mut full = Trim::new();
        let mut half = Trim::new();
        assert!(full.update_scaled(0.0, 2100.0, 2000.0, 1.0));
        assert!(half.update_scaled(0.0, 2100.0, 2000.0, 0.5));
        assert!((full.offset_rpm() - KI_TRIM * 100.0).abs() < 1e-12);
        assert!((half.offset_rpm() - 0.5 * KI_TRIM * 100.0).abs() < 1e-12);
        assert!(!half.update_scaled(10.0, 2100.0, 2000.0, 0.5), "cadence");
        // update() is the ki_scale = 1.0 case.
        let mut plain = Trim::new();
        assert!(plain.update(0.0, 2100.0, 2000.0));
        assert_eq!(plain.offset_rpm(), full.offset_rpm());
    }

    #[test]
    fn reset_clears_offset_and_baseline() {
        let mut t = Trim::new();
        t.update(0.0, 2300.0, 2000.0);
        assert_ne!(t.offset_rpm(), 0.0);
        t.reset();
        assert_eq!(t.offset_rpm(), 0.0);
        // Baseline gone too: the next call integrates immediately.
        assert!(t.update(1.0, 2100.0, 2000.0));
    }
}

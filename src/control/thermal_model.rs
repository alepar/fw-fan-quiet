//! Steady-state thermal model (design doc §3): `rpm = a·pc + b·pg + e·pc·pg + c`
//! where pc = CPU sustained watts, pg = GPU watts and rpm is the steady-state
//! `max(fan1, fan2)` RPM. Batch-fit from the 11-point calibration matrix
//! (Task 22), inverted to the ≤target-RPM contour by the allocator (Task 23),
//! and adapted online via trim + gated RLS (Tasks 26/27). Persisted in the
//! state file (Task 21), hence the serde derives.

use nalgebra::{DMatrix, DVector, Matrix4, Vector4};

use crate::control::allocator::CPU_MAX_W;

/// Excitation gate (Task 27 windup finding, quantified in the plan review):
/// an RLS update is accepted only if the operating point moved by more than
/// this (`|Δpc| + |Δpg|`, watts) since the last ACCEPTED update. At a
/// constant operating point the update carries no new information, yet the
/// covariance grows ×(1/λ) per step in the unexcited directions — measured:
/// diag(P) 1e4 → 2.3e8 after ~1k same-point updates, after which one ±20 RPM
/// noisy sample at a new point jumped `e` by ~25%.
pub const RLS_EXCITATION_MIN_W: f64 = 2.0;
/// Covariance trace cap: after an accepted update, if trace(P) exceeds this…
pub const RLS_TRACE_CAP: f64 = 1e6;
/// …P is rescaled to this trace. Belt to the excitation gate's suspenders: a
/// slowly drifting operating point can keep passing the gate while still
/// leaving directions unexcited, so the trace must stay bounded regardless.
pub const RLS_TRACE_RESCALE_TO: f64 = 1e5;
/// Contour-divisor floor (RPM per GPU watt): the smallest `b + e·pc` an
/// adapted or fitted model may claim anywhere on the allocator's pc range
/// `[0, CPU_MAX_W]`. A physical floor: 100 GPU watts must be able to move
/// the fans by at least 200 RPM.
///
/// Field incident (2026-06): online RLS drifted a calibrated model
/// (a=110.9, b=26.9, e=−0.47, c=−18.2) to a=62.5, b=35.9, e=−1.28, c=−105.
/// At the live pc = 28 W the contour divisor `b + e·pc` was 0.18 — near
/// zero, so `gpu_watts_on_contour` claimed GPU watts were acoustically
/// free: the contour exploded to thousands of watts (clamped to 100) and
/// the trim's ±400 RPM authority divided into nothing. The allocation
/// stuck at (28, 92) with the fans indefinitely OVER the 3250 RPM target.
/// The old slope-sanity gate rejected `a < 0` and `b < 0` but never
/// guarded the DIVISOR; this floor closes that hole.
pub const MIN_CONTOUR_DIVISOR: f64 = 2.0;

/// True iff `b + e·pc >= MIN_CONTOUR_DIVISOR` for every pc in
/// `[0, CPU_MAX_W]`. The divisor is linear in pc, so checking the two
/// endpoints suffices.
fn divisor_floor_ok(b: f64, e: f64) -> bool {
    b >= MIN_CONTOUR_DIVISOR && b + e * CPU_MAX_W >= MIN_CONTOUR_DIVISOR
}

/// Fitted model parameters plus (transient) RLS covariance.
///
/// The covariance `p` is deliberately not persisted: after a load it is
/// rebuilt as `I·1e4` on the first `rls_update`, i.e. online adaptation
/// restarts from a fresh prior around the persisted parameters.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThermalModel {
    pub a: f64,
    pub b: f64,
    pub e: f64,
    pub c: f64,
    /// RLS covariance; rebuilt on load (first `rls_update` after deserialize).
    #[serde(skip)]
    p: Option<Matrix4<f64>>,
    /// Operating point `(pc, pg)` of the last ACCEPTED RLS update — the
    /// excitation gate's reference. Like `p`, transient online-adaptation
    /// bookkeeping: not serialized, so gating restarts fresh after a load.
    #[serde(skip)]
    last_rls_point: Option<(f64, f64)>,
}

/// One steady-state calibration measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibPoint {
    pub cpu_w: f64,
    pub gpu_w: f64,
    pub rpm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FitError {
    /// Fewer than 4 points: the 4-parameter model is underdetermined.
    TooFewPoints,
    /// The design matrix is rank-deficient (e.g. all points identical).
    Degenerate,
}

impl std::fmt::Display for FitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FitError::TooFewPoints => write!(f, "need at least 4 calibration points"),
            FitError::Degenerate => {
                write!(f, "calibration points are degenerate (rank-deficient fit)")
            }
        }
    }
}

impl std::error::Error for FitError {}

impl ThermalModel {
    /// Batch least squares via SVD over the 4-column design matrix
    /// `[pc, pg, pc*pg, 1]`.
    pub fn fit_batch(points: &[CalibPoint]) -> Result<ThermalModel, FitError> {
        if points.len() < 4 {
            return Err(FitError::TooFewPoints);
        }
        let x = DMatrix::from_fn(points.len(), 4, |i, j| {
            let p = &points[i];
            match j {
                0 => p.cpu_w,
                1 => p.gpu_w,
                2 => p.cpu_w * p.gpu_w,
                _ => 1.0,
            }
        });
        let y = DVector::from_iterator(points.len(), points.iter().map(|p| p.rpm));
        let svd = x.svd(true, true);
        // Rank test relative to the largest singular value: identical or
        // collinear points collapse the column space below 4.
        let max_sv = svd.singular_values.max();
        let eps = max_sv * 1e-9;
        if max_sv <= 0.0 || svd.rank(eps) < 4 {
            return Err(FitError::Degenerate);
        }
        let theta = svd.solve(&y, eps).map_err(|_| FitError::Degenerate)?;
        let (a, b, mut e, c) = (theta[0], theta[1], theta[2], theta[3]);
        // Divisor-floor sanity (see [`MIN_CONTOUR_DIVISOR`]): a fit whose
        // `b + e·pc` dips below the floor anywhere on [0, CPU_MAX_W] would
        // hand the allocator a non-invertible contour. Clamp `e` up so the
        // divisor at pc = CPU_MAX_W sits exactly at the floor, rather than
        // erroring — a mediocre-but-invertible model beats a failed
        // calibration; the caller's residuals are computed against the
        // clamped model and stay honest.
        if !divisor_floor_ok(b, e) {
            let e_min = (MIN_CONTOUR_DIVISOR - b) / CPU_MAX_W;
            let clamped = e.max(e_min);
            tracing::warn!(
                "fit_batch: contour divisor floor violated \
                 (b={b:.3}, e={e:.4} -> min divisor over [0, {CPU_MAX_W}] W below \
                 {MIN_CONTOUR_DIVISOR}); clamping e to {clamped:.4}"
            );
            e = clamped;
        }
        Ok(ThermalModel {
            a,
            b,
            e,
            c,
            p: None,
            last_rls_point: None,
        })
    }

    pub fn predict(&self, pc: f64, pg: f64) -> f64 {
        self.a * pc + self.b * pg + self.e * pc * pg + self.c
    }

    /// `measured − predicted` per point, in input order.
    pub fn residuals(&self, points: &[CalibPoint]) -> Vec<f64> {
        points
            .iter()
            .map(|p| p.rpm - self.predict(p.cpu_w, p.gpu_w))
            .collect()
    }

    pub fn max_abs_residual(&self, points: &[CalibPoint]) -> f64 {
        self.residuals(points)
            .iter()
            .fold(0.0, |acc, r| acc.max(r.abs()))
    }

    /// RLS update with forgetting factor `lambda` (design: 0.99). Initializes
    /// covariance `P = I·1e4` on first use (or after load). Returns false
    /// (no state change whatsoever) when the update is rejected by any gate:
    ///
    /// - Non-finite gate: any NaN/±inf among `pc`/`pg`/`rpm` — NaN passes
    ///   the other gates (every NaN comparison is false) and one such
    ///   sample would poison params and covariance permanently.
    /// - Excitation gate: the operating point must have moved by more than
    ///   [`RLS_EXCITATION_MIN_W`] (`|Δpc| + |Δpg|`) since the last ACCEPTED
    ///   update — same-point updates carry no information and only wind up
    ///   the covariance (the Task-27 review finding).
    /// - Slope-sanity gate: any update that would make `a` negative
    ///   (physics: more power can never mean less fan), or that would push
    ///   the contour divisor `b + e·pc` below [`MIN_CONTOUR_DIVISOR`] for
    ///   ANY pc in `[0, CPU_MAX_W]` — the 2026-06 field incident (see the
    ///   constant's docs) was a drift past `b + e·pc ≈ 0` that sailed
    ///   through the old `b < 0` check.
    ///
    /// After an accepted update, trace(P) is capped: above [`RLS_TRACE_CAP`]
    /// it is rescaled to [`RLS_TRACE_RESCALE_TO`], bounding how hard a noisy
    /// sample can yank the parameters no matter what the update history was.
    pub fn rls_update(&mut self, pc: f64, pg: f64, rpm: f64, lambda: f64) -> bool {
        // Poisoned-input guard: every comparison with NaN is false, so a NaN
        // input sails through both gates below and would permanently poison
        // params AND covariance in one update. Reject non-finite inputs
        // outright (no state change whatsoever).
        if !(pc.is_finite() && pg.is_finite() && rpm.is_finite()) {
            return false;
        }
        if let Some((last_pc, last_pg)) = self.last_rls_point
            && (pc - last_pc).abs() + (pg - last_pg).abs() <= RLS_EXCITATION_MIN_W
        {
            return false;
        }
        let p = self.p.unwrap_or_else(|| Matrix4::identity() * 1e4);
        let x = Vector4::new(pc, pg, pc * pg, 1.0);
        // Textbook RLS with forgetting (research doc 03 §2):
        //   k = P·x / (λ + xᵀ·P·x);  θ ← θ + k·err;  P ← (P − k·xᵀ·P) / λ
        let px = p * x;
        let k = px / (lambda + x.dot(&px));
        let err = rpm - self.predict(pc, pg);
        let theta = Vector4::new(self.a, self.b, self.e, self.c) + k * err;
        // Slope-sanity gate: more power can never mean less fan, and the
        // contour divisor `b + e·pc` must stay ≥ MIN_CONTOUR_DIVISOR across
        // the whole pc range (linear in pc: both endpoints checked inside
        // `divisor_floor_ok`). No divisor check on the `a` side — `a` never
        // divides anything. Reject the whole update (including the
        // covariance step) so a poisoned sample leaves no trace.
        if theta[0] < 0.0 || !divisor_floor_ok(theta[1], theta[2]) {
            return false;
        }
        (self.a, self.b, self.e, self.c) = (theta[0], theta[1], theta[2], theta[3]);
        self.last_rls_point = Some((pc, pg));
        let mut p_next = (p - k * (x.transpose() * p)) / lambda;
        let trace = p_next.trace();
        if trace > RLS_TRACE_CAP {
            p_next *= RLS_TRACE_RESCALE_TO / trace;
        }
        self.p = Some(p_next);
        true
    }

    /// The ≤target-RPM contour: GPU watts as a function of CPU watts, with a
    /// trim offset added to `c`: `pg = (target − (c+trim) − a·pc) / (b + e·pc)`,
    /// clamped to >= 0. A clamped answer of `Some(0.0)` means "GPU gets
    /// nothing at this pc", not "impossible". `None` when the divisor
    /// `b + e·pc < MIN_CONTOUR_DIVISOR / 2` (degenerate model): a near-zero
    /// divisor turns the contour into thousands of phantom watts (the
    /// 2026-06 field incident divided by 0.18), so an honest None → the
    /// allocator's freeze path beats an insane Some. Half the floor, not
    /// the floor itself, so a model gated AT the floor still answers;
    /// belt-and-suspenders for a degenerate model inherited from disk that
    /// never passed the RLS/fit gates.
    pub fn gpu_watts_on_contour(&self, target_rpm: f64, trim: f64, pc: f64) -> Option<f64> {
        let divisor = self.b + self.e * pc;
        if divisor < MIN_CONTOUR_DIVISOR / 2.0 {
            return None;
        }
        let pg = (target_rpm - (self.c + trim) - self.a * pc) / divisor;
        Some(pg.max(0.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: f64 = 25.0;
    const B: f64 = 15.0;
    const E: f64 = 0.1;
    const C: f64 = 800.0;

    /// The 11 calibration (cpu_w, gpu_w) points from design doc §4.
    const POINTS: [(f64, f64); 11] = [
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

    fn truth_rpm(pc: f64, pg: f64) -> f64 {
        A * pc + B * pg + E * pc * pg + C
    }

    fn exact_points() -> Vec<CalibPoint> {
        POINTS
            .iter()
            .map(|&(pc, pg)| CalibPoint {
                cpu_w: pc,
                gpu_w: pg,
                rpm: truth_rpm(pc, pg),
            })
            .collect()
    }

    fn exact_model() -> ThermalModel {
        ThermalModel::fit_batch(&exact_points()).unwrap()
    }

    /// Deterministic pseudo-noise in [-20, 20) RPM: inline LCG, no rand crate.
    fn lcg_noise(state: &mut u64) -> f64 {
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*state >> 32) as f64 / f64::from(u32::MAX)) * 40.0 - 20.0
    }

    #[test]
    fn fit_recovers_known_params() {
        let m = exact_model();
        assert!((m.a - A).abs() < 1e-6, "a = {}", m.a);
        assert!((m.b - B).abs() < 1e-6, "b = {}", m.b);
        assert!((m.e - E).abs() < 1e-6, "e = {}", m.e);
        assert!((m.c - C).abs() < 1e-6, "c = {}", m.c);
    }

    #[test]
    fn fit_with_noise_close() {
        let mut seed = 0xbeef_cafe_u64;
        let noisy: Vec<CalibPoint> = POINTS
            .iter()
            .map(|&(pc, pg)| CalibPoint {
                cpu_w: pc,
                gpu_w: pg,
                rpm: truth_rpm(pc, pg) + lcg_noise(&mut seed),
            })
            .collect();
        let m = ThermalModel::fit_batch(&noisy).unwrap();
        assert!((m.a - A).abs() < 0.1 * A, "a = {}", m.a);
        assert!((m.b - B).abs() < 0.1 * B, "b = {}", m.b);
        assert!((m.e - E).abs() < 0.1 * E, "e = {}", m.e);
        assert!((m.c - C).abs() < 0.1 * C, "c = {}", m.c);
    }

    #[test]
    fn too_few_points_err() {
        let pts = &exact_points()[..3];
        assert_eq!(ThermalModel::fit_batch(pts), Err(FitError::TooFewPoints));
    }

    #[test]
    fn degenerate_collinear_points_err() {
        // Distinct but collinear (pg = 2·pc): rank-deficient without being
        // duplicates — pins the rank test against regressing to a
        // duplicate-only check.
        let pts: Vec<CalibPoint> = (1..=8)
            .map(|i| {
                let pc = f64::from(i) * 5.0;
                CalibPoint {
                    cpu_w: pc,
                    gpu_w: 2.0 * pc,
                    rpm: 800.0 + 30.0 * pc,
                }
            })
            .collect();
        assert!(matches!(
            ThermalModel::fit_batch(&pts),
            Err(FitError::Degenerate)
        ));
    }

    #[test]
    fn degenerate_err() {
        let pts = vec![
            CalibPoint {
                cpu_w: 5.0,
                gpu_w: 10.0,
                rpm: 1000.0
            };
            5
        ];
        assert_eq!(ThermalModel::fit_batch(&pts), Err(FitError::Degenerate));
    }

    #[test]
    fn predict_matches_formula() {
        let m = exact_model();
        let (pc, pg) = (12.5, 42.0);
        assert!((m.predict(pc, pg) - truth_rpm(pc, pg)).abs() < 1e-6);
    }

    #[test]
    fn residuals_zero_on_exact_fit() {
        let pts = exact_points();
        let m = ThermalModel::fit_batch(&pts).unwrap();
        for r in m.residuals(&pts) {
            assert!(r.abs() < 1e-6, "residual {r}");
        }
    }

    #[test]
    fn max_abs_residual_picks_the_max() {
        let m = exact_model();
        let mut pts = exact_points();
        pts[3].rpm += 50.0; // one point off by +50
        pts[7].rpm -= 120.0; // another off by -120 (the max, and negative)
        assert!((m.max_abs_residual(&pts) - 120.0).abs() < 1e-6);
    }

    #[test]
    fn rls_converges_offset_drift() {
        let mut m = exact_model();
        // Fan surface drifted up by +150 RPM (dust/ambient): same slopes,
        // offset c+150. Feed 200 steady-state samples across the matrix.
        for i in 0..200 {
            let (pc, pg) = POINTS[i % POINTS.len()];
            m.rls_update(pc, pg, truth_rpm(pc, pg) + 150.0, 0.99);
        }
        assert!((m.c - (C + 150.0)).abs() < 0.2 * 150.0, "c = {}", m.c);
        assert!((m.a - A).abs() < 0.05 * A, "a = {}", m.a);
        assert!((m.b - B).abs() < 0.05 * B, "b = {}", m.b);
        assert!((m.e - E).abs() < 0.05 * E, "e = {}", m.e);
    }

    #[test]
    fn rls_rejects_negative_slope_poison() {
        let mut m = exact_model();
        let before = m.clone();
        // First update, so P = I*1e4 and the gain is ~0.5 along [pc, 1]:
        // rpm=0 at (pc=1, pg=0) has innovation ≈ -825, pushing a to ~-387.
        let accepted = m.rls_update(1.0, 0.0, 0.0, 0.99);
        assert!(!accepted);
        assert_eq!(m.a, before.a);
        assert_eq!(m.b, before.b);
        assert_eq!(m.e, before.e);
        assert_eq!(m.c, before.c);
    }

    #[test]
    fn rls_rejects_non_finite_inputs() {
        let mut m = exact_model();
        // Live covariance + excitation reference first, so "no state change"
        // covers those too (not just the params).
        assert!(m.rls_update(20.0, 40.0, truth_rpm(20.0, 40.0), 0.99));
        let before = m.clone();
        let good_rpm = truth_rpm(45.0, 100.0);
        for (pc, pg, rpm) in [
            (f64::NAN, 100.0, good_rpm),
            (45.0, f64::NAN, good_rpm),
            (45.0, 100.0, f64::NAN),
            (f64::INFINITY, 100.0, good_rpm),
            (45.0, f64::NEG_INFINITY, good_rpm),
            (45.0, 100.0, f64::INFINITY),
        ] {
            assert!(
                !m.rls_update(pc, pg, rpm, 0.99),
                "({pc}, {pg}, {rpm}) must be rejected"
            );
            // Bit-identical params AND covariance: a rejected poisoned
            // sample leaves no trace at all.
            assert_eq!(m, before, "state changed after ({pc}, {pg}, {rpm})");
        }
        // Still alive: the next finite, excited update is accepted.
        assert!(m.rls_update(45.0, 100.0, good_rpm, 0.99));
    }

    #[test]
    fn contour_roundtrip() {
        let m = exact_model();
        let target = 3000.0;
        for trim in [0.0, 100.0] {
            for pc in [10.0, 25.0, 40.0] {
                let pg = m.gpu_watts_on_contour(target, trim, pc).unwrap();
                assert!(pg > 0.0, "expected unclamped contour at pc={pc}");
                // With trim added to c, the *trimmed* model hits the target:
                // predict + trim == target.
                let rpm = m.predict(pc, pg) + trim;
                assert!((rpm - target).abs() < 1e-6, "pc={pc} trim={trim} rpm={rpm}");
            }
        }
    }

    #[test]
    fn contour_clamps_negative() {
        let m = exact_model();
        // Target below the zero-GPU floor at this pc: raw pg is negative,
        // clamped to 0.0 ("GPU gets nothing", not "impossible").
        assert_eq!(m.gpu_watts_on_contour(500.0, 0.0, 10.0), Some(0.0));
        // Degenerate divisor b + e*pc <= 1e-9 -> None.
        let flat = ThermalModel {
            a: A,
            b: 0.0,
            e: 0.0,
            c: C,
            p: None,
            last_rls_point: None,
        };
        assert_eq!(flat.gpu_watts_on_contour(3000.0, 0.0, 10.0), None);
    }

    #[test]
    fn serde_roundtrip_skips_covariance() {
        let mut m = exact_model();
        // Give the source model a live covariance so the skip is observable.
        assert!(m.rls_update(20.0, 40.0, truth_rpm(20.0, 40.0), 0.99));
        let json = serde_json::to_string(&m).unwrap();
        let mut back: ThermalModel = serde_json::from_str(&json).unwrap();
        assert_eq!(back.a, m.a);
        assert_eq!(back.b, m.b);
        assert_eq!(back.e, m.e);
        assert_eq!(back.c, m.c);
        // Covariance + excitation bookkeeping were skipped: `back` equals
        // `m` with both cleared, not `m`.
        assert_eq!(
            back,
            ThermalModel {
                p: None,
                last_rls_point: None,
                ..m.clone()
            }
        );
        assert_ne!(back, m);
        // And RLS still works after deserialize (P re-initialized, excitation
        // gating fresh: even the SAME operating point is accepted again).
        assert!(back.rls_update(20.0, 40.0, truth_rpm(20.0, 40.0) + 10.0, 0.99));
    }

    // --- Contour divisor floor (2026-06 field incident) ---

    #[test]
    fn rls_rejects_divisor_floor_poison() {
        // Params shaped like the drifted field model but still healthy:
        // divisor at pc=54 is 35.9 − 0.47·54 ≈ 10.5.
        let mut m = ThermalModel {
            a: 62.5,
            b: 35.9,
            e: -0.47,
            c: -105.0,
            p: None,
            last_rls_point: None,
        };
        let before = m.clone();
        // Fresh P = I·1e4 at (50, 90): Δe ≈ pc·pg·err/|x|² ≈ −0.20 for
        // err = −900, pushing the candidate e to ≈ −0.67 where
        // b + e·54 ≈ −0.3 < MIN_CONTOUR_DIVISOR while a and b both stay
        // positive — the old a<0/b<0 gate would have ACCEPTED this update.
        let rpm = m.predict(50.0, 90.0) - 900.0;
        assert!(!m.rls_update(50.0, 90.0, rpm, 0.99));
        assert_eq!(m, before, "rejected update must leave no trace");
        // A small innovation keeps the divisor ≥ MIN everywhere → accepted.
        let rpm = m.predict(50.0, 90.0) - 50.0;
        assert!(m.rls_update(50.0, 90.0, rpm, 0.99));
        assert!(m.b >= MIN_CONTOUR_DIVISOR);
        assert!(m.b + m.e * CPU_MAX_W >= MIN_CONTOUR_DIVISOR);
    }

    #[test]
    fn contour_none_below_half_divisor_floor() {
        // The field model's divisor at pc=28 was 0.18; even 0.5 must be
        // None (was Some(thousands of phantom watts) under the 1e-9 test).
        let m = ThermalModel {
            a: 62.5,
            b: 0.5,
            e: 0.0,
            c: -105.0,
            p: None,
            last_rls_point: None,
        };
        assert_eq!(m.gpu_watts_on_contour(3250.0, 0.0, 28.0), None);
        // At/above half the floor the contour still answers: a gated model
        // sits at ≥ MIN_CONTOUR_DIVISOR, comfortably above this threshold.
        let m = ThermalModel {
            b: 1.5,
            ..m.clone()
        };
        assert!(m.gpu_watts_on_contour(3250.0, 0.0, 28.0).is_some());
    }

    #[test]
    fn fit_batch_clamps_e_to_divisor_floor() {
        // Truth surface shaped like the drifted field model: e so negative
        // that b + e·54 ≈ −42. The fit must come back INVERTIBLE (e clamped
        // up to the floor at pc = CPU_MAX_W, warn path), not error — a
        // mediocre-but-invertible model beats a failed calibration.
        let pts: Vec<CalibPoint> = POINTS
            .iter()
            .map(|&(pc, pg)| CalibPoint {
                cpu_w: pc,
                gpu_w: pg,
                rpm: 62.5 * pc + 35.9 * pg - 1.28 * pc * pg - 105.0,
            })
            .collect();
        let m = ThermalModel::fit_batch(&pts).unwrap();
        assert!((m.a - 62.5).abs() < 1e-6, "a = {}", m.a);
        assert!((m.b - 35.9).abs() < 1e-6, "b = {}", m.b);
        let e_min = (MIN_CONTOUR_DIVISOR - m.b) / CPU_MAX_W;
        assert!((m.e - e_min).abs() < 1e-6, "e = {} (want {e_min})", m.e);
        assert!(m.b + m.e * CPU_MAX_W >= MIN_CONTOUR_DIVISOR - 1e-9);
        // Residuals are recomputed against the CLAMPED model: at the high
        // pc·pg corner the clamped e leaves a large, honest residual.
        assert!(m.max_abs_residual(&pts) > 1000.0);
        // And the clamped model's contour is answerable across the range.
        for pc in [0.0, 28.0, CPU_MAX_W] {
            assert!(
                m.gpu_watts_on_contour(3250.0, 0.0, pc).is_some(),
                "contour degenerate at pc={pc}"
            );
        }
    }

    // --- Task 27: windup gates ---

    #[test]
    fn excitation_gate_rejects_same_point_updates() {
        let mut m = exact_model();
        let mut seed = 0x1234_5678_u64;
        // (a) 1000 same-point updates: only the first is accepted; the rest
        // change NOTHING (params or covariance), so P cannot wind up.
        assert!(m.rls_update(30.0, 65.0, truth_rpm(30.0, 65.0), 0.99));
        let after_first = m.clone();
        for _ in 0..999 {
            let rpm = truth_rpm(30.0, 65.0) + lcg_noise(&mut seed);
            assert!(!m.rls_update(30.0, 65.0, rpm, 0.99));
        }
        assert_eq!(m, after_first, "rejected updates must leave no trace");
        let trace = m.p.expect("covariance live").trace();
        assert!(
            trace <= 4e4,
            "same-point trace must stay at/below the fresh prior, got {trace:e}"
        );
    }

    #[test]
    fn excitation_gate_threshold_is_2w_of_combined_movement() {
        let mut m = exact_model();
        assert!(m.rls_update(30.0, 65.0, truth_rpm(30.0, 65.0), 0.99));
        // |Δpc| + |Δpg| = 2.0: not strictly greater — rejected.
        assert!(!m.rls_update(31.0, 66.0, truth_rpm(31.0, 66.0), 0.99));
        // 2.2 W of combined movement: accepted…
        assert!(m.rls_update(31.0, 66.2, truth_rpm(31.0, 66.2), 0.99));
        // …and the reference is the last ACCEPTED point (31, 66.2), so the
        // original point is now far enough away again.
        assert!(m.rls_update(30.0, 65.0, truth_rpm(30.0, 65.0), 0.99));
    }

    #[test]
    fn covariance_trace_capped_under_alternating_excitation() {
        // (b) Two alternating far-apart points keep the excitation gate open
        // but leave two of the four regressor directions unexcited: without
        // the cap their covariance grows ×(1/λ) per step and the trace passes
        // 1e6 within ~460 updates. With the cap it must stay bounded after
        // EVERY update.
        let mut m = exact_model();
        let mut max_trace: f64 = 0.0;
        for i in 0..500 {
            let (pc, pg) = if i % 2 == 0 {
                (5.0, 0.0)
            } else {
                (45.0, 100.0)
            };
            assert!(m.rls_update(pc, pg, truth_rpm(pc, pg), 0.99), "i={i}");
            let trace = m.p.expect("covariance live").trace();
            assert!(trace <= RLS_TRACE_CAP, "trace {trace:e} after update {i}");
            max_trace = max_trace.max(trace);
        }
        // The windup pressure was real (the run pushed well past the rescale
        // target) — i.e. the cap was doing work, not idling.
        assert!(
            max_trace > 5.0 * RLS_TRACE_RESCALE_TO,
            "expected windup pressure, max trace {max_trace:e}"
        );
    }

    #[test]
    fn gated_rls_shrugs_off_noisy_sample_after_same_point_soak() {
        // (c) The plan's quantified scenario: ~1k same-point updates used to
        // blow diag(P) up to 2.3e8, after which ONE ±20 RPM noisy sample at a
        // new point jumped `e` by ~25%. With the gates the soak is inert and
        // the noisy sample moves `e` by well under 5%.
        //
        // One full-rank pass over the calibration matrix first: a session
        // that has seen varied operating points has a settled covariance
        // (a completely fresh P = I·1e4 prior is legitimately "uncertain" —
        // one sample may move `e` by >10% no matter what the soak does; the
        // finding is about the SOAK not being allowed to re-inflate P).
        let mut m = exact_model();
        for &(pc, pg) in &POINTS {
            assert!(m.rls_update(pc, pg, truth_rpm(pc, pg), 0.99));
        }
        for _ in 0..1000 {
            m.rls_update(30.0, 65.0, truth_rpm(30.0, 65.0), 0.99);
        }
        let e_before = m.e;
        assert!(m.rls_update(45.0, 100.0, truth_rpm(45.0, 100.0) + 20.0, 0.99));
        let moved = (m.e - e_before).abs();
        assert!(
            moved < 0.05 * E,
            "e moved {moved:e} (>{:e}) on one noisy sample — windup not tamed",
            0.05 * E
        );
    }
}

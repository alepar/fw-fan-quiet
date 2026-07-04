//! Steady-state thermal model (design doc §3): `rpm = a·pc + b·pg + e·pc·pg + c`
//! where pc = CPU sustained watts, pg = GPU watts and rpm is the steady-state
//! `max(fan1, fan2)` RPM. Batch-fit from the 11-point calibration matrix
//! (Task 22), inverted to the ≤target-RPM contour by the allocator (Task 23),
//! and adapted online via trim + gated RLS (Tasks 26/27). Persisted in the
//! state file (Task 21), hence the serde derives.

use nalgebra::{DMatrix, DVector, Matrix4, Vector4};

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

// TODO(task-22/23/26/27): calibration runner, allocator and trim/RLS loops are
// the consumers; dead until then.
#[allow(dead_code)]
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
        Ok(ThermalModel {
            a: theta[0],
            b: theta[1],
            e: theta[2],
            c: theta[3],
            p: None,
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
    /// covariance `P = I·1e4` on first use (or after load). Rejects (returns
    /// false, no state change) any update that would make `a` or `b` negative
    /// — physics: more power can never mean less fan.
    pub fn rls_update(&mut self, pc: f64, pg: f64, rpm: f64, lambda: f64) -> bool {
        let p = self.p.unwrap_or_else(|| Matrix4::identity() * 1e4);
        let x = Vector4::new(pc, pg, pc * pg, 1.0);
        // Textbook RLS with forgetting (research doc 03 §2):
        //   k = P·x / (λ + xᵀ·P·x);  θ ← θ + k·err;  P ← (P − k·xᵀ·P) / λ
        let px = p * x;
        let k = px / (lambda + x.dot(&px));
        let err = rpm - self.predict(pc, pg);
        let theta = Vector4::new(self.a, self.b, self.e, self.c) + k * err;
        // Slope-sanity gate: more power can never mean less fan. Reject the
        // whole update (including the covariance step) so a poisoned sample
        // leaves no trace.
        if theta[0] < 0.0 || theta[1] < 0.0 {
            return false;
        }
        (self.a, self.b, self.e, self.c) = (theta[0], theta[1], theta[2], theta[3]);
        self.p = Some((p - k * (x.transpose() * p)) / lambda);
        true
    }

    /// The ≤target-RPM contour: GPU watts as a function of CPU watts, with a
    /// trim offset added to `c`: `pg = (target − (c+trim) − a·pc) / (b + e·pc)`,
    /// clamped to >= 0. A clamped answer of `Some(0.0)` means "GPU gets
    /// nothing at this pc", not "impossible". `None` only when the divisor
    /// `b + e·pc <= 1e-9` (degenerate model).
    pub fn gpu_watts_on_contour(&self, target_rpm: f64, trim: f64, pc: f64) -> Option<f64> {
        let divisor = self.b + self.e * pc;
        if divisor <= 1e-9 {
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
        // Covariance was skipped: `back` equals `m` with p cleared, not `m`.
        assert_eq!(
            back,
            ThermalModel {
                p: None,
                ..m.clone()
            }
        );
        assert_ne!(back, m);
        // And RLS still works after deserialize (P re-initialized).
        assert!(back.rls_update(30.0, 65.0, truth_rpm(30.0, 65.0) + 10.0, 0.99));
    }
}

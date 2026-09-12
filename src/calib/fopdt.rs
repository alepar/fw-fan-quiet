//! FOPDT identification and IMC gain derivation for the native per-device
//! calibration steps. Fits operate on the already-filtered device-group
//! response, so the fitted dead time is used directly.

use crate::control::device_loop::Gains;

/// Minimum identifiable EC response, °C (design doc §3.3).
pub const MIN_EC_RESPONSE_C: f64 = 3.0;
/// Minimum accepted time constant, s (design doc §3.3) — below this the fit
/// is too fast to trust (and, degenerate, could drive `lambda` here would
/// otherwise not run away only by luck).
const MIN_TAU_S: f64 = 5.0;
/// IMC lambda floor, s (§2.4) — "not the boxcar's margin, a separate lower
/// bound."
const LAMBDA_FLOOR_S: f64 = 90.0;
/// IMC lambda multiplier on `theta_hat` (§2.4).
const LAMBDA_THETA_MULT: f64 = 3.0;
/// Accepted band for a derived `Kc` relative to the corresponding default
/// (§3.3) — the last line of defense against `Kc = tau/(K*(lambda+theta))`
/// running away as `K` approaches zero from above.
const KC_RATIO_LO: f64 = 0.25;
const KC_RATIO_HI: f64 = 4.0;

/// A fitted first-order-plus-dead-time plant: `k` is the steady-state gain
/// (output-per-watt: °C/W for the EC leg, RPM/W for the fan leg), `tau` the
/// time constant (s), `theta` the dead time (s) **as fitted** — see the
/// theta-trap note above.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fopdt {
    pub k: f64,
    pub tau: f64,
    pub theta: f64,
}

/// Fits `y(t) = y0 + k*step_w*(1 - exp(-(t-theta)/tau))` (`y0` for
/// `t <= theta`) to `data` by least squares, searching `(tau, theta)` with a
/// multi-resolution grid (for fixed `tau, theta` the model is linear in
/// `y0` and `k*step_w`, so each grid point's inner fit is a closed-form
/// two-parameter OLS) and returning the fit with the lowest sum of squared
/// residuals.
///
/// `data` need not be sorted; `t` is measured from the same origin the step
/// was applied at (a point with `t <= theta` is treated as pre-step).
///
/// Returns `None` if there are fewer than 4 points, `step_w` is zero or
/// non-finite, the search fails to find a usable `(tau, theta)` (degenerate
/// data — no variation to fit), or any of the rejection rules fire: the
/// fitted `k <= 0`, `tau < 5 s`, or the identified response magnitude
/// (`|k * step_w|`) is under `min_response`.
pub fn fit_fopdt(data: &[(f64, f64)], step_w: f64, min_response: f64) -> Option<Fopdt> {
    if data.len() < 4 || step_w == 0.0 || !step_w.is_finite() {
        return None;
    }
    let (tau, theta) = search_tau_theta(data)?;
    let (_a, b, _sse) = linear_fit_at(data, tau, theta)?;
    let k = b / step_w;
    // NaN-aware: `k <= 0.0` alone would silently accept a NaN result.
    if k.is_nan() || k <= 0.0 {
        return None;
    }
    if tau < MIN_TAU_S {
        return None;
    }
    if b.abs() < min_response {
        return None;
    }
    Some(Fopdt { k, tau, theta })
}

/// For fixed `(tau, theta)`, the FOPDT step model is linear in `y0` (`a`)
/// and `k*step_w` (`b`): `y_i = a + b*g_i` where `g_i` is the model's
/// dimensionless step-response shape at `t_i`. Solves that 2-parameter OLS
/// in closed form and returns `(a, b, sse)`.
///
/// Returns `None` for `tau <= 0` or when `g` has no variation across `data`
/// (every point on the same side of `theta` with the same shape value —
/// nothing to regress against).
fn linear_fit_at(data: &[(f64, f64)], tau: f64, theta: f64) -> Option<(f64, f64, f64)> {
    if tau.is_nan() || tau <= 0.0 {
        return None;
    }
    let n = data.len() as f64;
    let g: Vec<f64> = data
        .iter()
        .map(|&(t, _)| {
            if t <= theta {
                0.0
            } else {
                1.0 - (-(t - theta) / tau).exp()
            }
        })
        .collect();
    let g_bar = g.iter().sum::<f64>() / n;
    let y_bar = data.iter().map(|&(_, y)| y).sum::<f64>() / n;

    let mut num = 0.0;
    let mut den = 0.0;
    for (i, &(_, y)) in data.iter().enumerate() {
        let gd = g[i] - g_bar;
        num += gd * (y - y_bar);
        den += gd * gd;
    }
    if den < 1e-9 {
        return None;
    }
    let b = num / den;
    let a = y_bar - b * g_bar;

    let sse = data
        .iter()
        .zip(g.iter())
        .map(|(&(_, y), &gi)| {
            let e = y - (a + b * gi);
            e * e
        })
        .sum();
    Some((a, b, sse))
}

/// Multi-resolution grid search for the `(tau, theta)` minimizing
/// [`linear_fit_at`]'s SSE. Coarse-to-fine: each round evaluates a
/// `GRID_N x GRID_N` grid over the current `[lo, hi]` window per parameter,
/// then re-centers a `SHRINK`-scaled window on the round's best point for
/// the next round.
fn search_tau_theta(data: &[(f64, f64)]) -> Option<(f64, f64)> {
    let t_min = data.iter().map(|&(t, _)| t).fold(f64::INFINITY, f64::min);
    let t_max = data
        .iter()
        .map(|&(t, _)| t)
        .fold(f64::NEG_INFINITY, f64::max);
    let span = t_max - t_min;
    if span.is_nan() || span <= 0.0 {
        return None;
    }

    const GRID_N: usize = 25;
    const ROUNDS: usize = 9;
    const SHRINK: f64 = 0.35;

    let mut tau_lo = 0.1_f64;
    let mut tau_hi = 3.0 * span;
    let mut theta_lo = 0.0_f64;
    let mut theta_hi = 0.5 * span;

    let mut best: Option<(f64, f64, f64)> = None;
    for _round in 0..ROUNDS {
        let mut round_best: Option<(f64, f64, f64)> = None;
        for i in 0..GRID_N {
            let tau = tau_lo + (tau_hi - tau_lo) * i as f64 / (GRID_N - 1) as f64;
            for j in 0..GRID_N {
                let theta = theta_lo + (theta_hi - theta_lo) * j as f64 / (GRID_N - 1) as f64;
                if let Some((_, _, sse)) = linear_fit_at(data, tau, theta) {
                    let better = match round_best {
                        Some((_, _, best_sse)) => sse < best_sse,
                        None => true,
                    };
                    if better {
                        round_best = Some((tau, theta, sse));
                    }
                }
            }
        }
        let (btau, btheta, _) = round_best?;
        best = round_best;

        let tau_span = ((tau_hi - tau_lo) * SHRINK).max(1e-6);
        let theta_span = ((theta_hi - theta_lo) * SHRINK).max(1e-6);
        tau_lo = (btau - tau_span / 2.0).max(0.01);
        tau_hi = btau + tau_span / 2.0;
        theta_lo = (btheta - theta_span / 2.0).max(0.0);
        theta_hi = btheta + theta_span / 2.0;
    }
    best.map(|(tau, theta, _)| (tau, theta))
}

/// Derives `Kc = tau/(K*(lambda + theta))`, `Ti = tau`, `lambda =
/// max(90, 3*theta)`, independently for `ec` and `rpm`, using each `theta`
/// **exactly as fitted** (see the theta-trap module note). Rejects (returns
/// `None`) if either signal has `k <= 0`, `tau < 5 s` (both re-checked here
/// defensively — a caller can construct a [`Fopdt`] by hand, not only via
/// [`fit_fopdt`]), or the resulting `Kc` falls outside `[0.25, 4]x` the
pub fn derive_device_gains(fopdt: &Fopdt, defaults: Gains) -> Option<Gains> {
    let (kc, ti_s) = derive_one(fopdt, defaults.kc)?;
    Some(Gains { kc, ti_s })
}

/// One signal's `(Kc, Ti)` derivation plus its rejection rules. `Ti = tau`
/// unconditionally; `Kc` is checked against `[0.25, 4]x default_kc`.
fn derive_one(fopdt: &Fopdt, default_kc: f64) -> Option<(f64, f64)> {
    if fopdt.k.is_nan() || fopdt.k <= 0.0 || fopdt.tau.is_nan() || fopdt.tau < MIN_TAU_S {
        return None;
    }
    let lambda = (LAMBDA_THETA_MULT * fopdt.theta).max(LAMBDA_FLOOR_S);
    let kc = fopdt.tau / (fopdt.k * (lambda + fopdt.theta));
    if !kc.is_finite() {
        return None;
    }
    let ratio = kc / default_kc;
    if !(KC_RATIO_LO..=KC_RATIO_HI).contains(&ratio) {
        return None;
    }
    Some((kc, fopdt.tau))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derive_device_gains_uses_fitted_theta_as_is_in_native_units() {
        let fit = Fopdt {
            k: 0.8,
            tau: 35.0,
            theta: 20.0,
        };
        let defaults = crate::control::device_loop::Gains {
            kc: 35.0 / (0.8 * 110.0),
            ti_s: 35.0,
        };

        let gains = derive_device_gains(&fit, defaults).expect("native fit should derive");

        assert!((gains.kc - 35.0 / (0.8 * (90.0 + 20.0))).abs() < 1e-12);
        assert_eq!(gains.ti_s, 35.0);
    }

    /// Small deterministic xorshift32 PRNG — matches the existing
    /// convention (`src/fanctrl/table.rs`): "All simulation and plant tests
    /// use a seeded, hand-rolled RNG" (Global Constraints), so this adds no
    /// new crate dependency.
    struct Xorshift32(u32);
    impl Xorshift32 {
        fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            x
        }
        /// Uniform in `[lo, hi)`.
        fn next_f64(&mut self, lo: f64, hi: f64) -> f64 {
            let unit = f64::from(self.next_u32()) / f64::from(u32::MAX);
            lo + unit * (hi - lo)
        }
    }

    fn within_pct(actual: f64, expected: f64, pct: f64) -> bool {
        (actual - expected).abs() <= pct * expected.abs()
    }

    /// Plant + sampling parameters for [`synthetic_step`].
    struct StepSpec {
        y0: f64,
        k: f64,
        tau: f64,
        theta: f64,
        step_w: f64,
        n: usize,
        dt: f64,
        noise: f64,
    }

    /// Generates a noisy synthetic FOPDT step-response trace: `spec.n`
    /// samples spaced `spec.dt` apart from `t = 0` (the step instant), with
    /// uniform noise in `[-spec.noise, spec.noise]` from `rng`.
    fn synthetic_step(spec: &StepSpec, rng: &mut Xorshift32) -> Vec<(f64, f64)> {
        (0..spec.n)
            .map(|i| {
                let t = i as f64 * spec.dt;
                let clean = if t <= spec.theta {
                    spec.y0
                } else {
                    spec.y0
                        + spec.k * spec.step_w * (1.0 - (-(t - spec.theta) / spec.tau).exp())
                };
                (t, clean + rng.next_f64(-spec.noise, spec.noise))
            })
            .collect()
    }

    // ---- fit_fopdt: recovers K/tau/theta within 10% on a noisy step ----

    #[test]
    fn fit_fopdt_recovers_k_tau_theta_within_10_percent() {
        let mut rng = Xorshift32(0xC0FF_EE01);
        let (k, tau, theta, step_w) = (0.8, 35.0, 20.0, 30.0);
        // 300 s at 5 s cadence (PI_PERIOD_S) covers 5*tau + theta = 195 s
        // with margin for the tail to settle.
        let data = synthetic_step(
            &StepSpec {
                y0: 40.0,
                k,
                tau,
                theta,
                step_w,
                n: 61,
                dt: 5.0,
                noise: 0.2,
            },
            &mut rng,
        );

        let fit = fit_fopdt(&data, step_w, MIN_EC_RESPONSE_C).expect("fit should succeed");

        assert!(
            within_pct(fit.k, k, 0.10),
            "k = {} not within 10% of {k}",
            fit.k
        );
        assert!(
            within_pct(fit.tau, tau, 0.10),
            "tau = {} not within 10% of {tau}",
            fit.tau
        );
        assert!(
            within_pct(fit.theta, theta, 0.10),
            "theta = {} not within 10% of {theta}",
            fit.theta
        );
    }

    // ---- derive_gains: the theta assertion, written explicitly ----

    #[test]
    fn fit_fopdt_rejects_non_positive_k() {
        let mut rng = Xorshift32(0x1234_5678);
        // Power increasing while the measured value falls: fits to k < 0.
        let data = synthetic_step(
            &StepSpec {
                y0: 50.0,
                k: -0.5,
                tau: 35.0,
                theta: 20.0,
                step_w: 30.0,
                n: 61,
                dt: 5.0,
                noise: 0.1,
            },
            &mut rng,
        );
        assert_eq!(fit_fopdt(&data, 30.0, MIN_EC_RESPONSE_C), None);
    }

    #[test]
    fn fit_fopdt_rejects_tau_under_5s() {
        let mut rng = Xorshift32(0x2233_4455);
        // True tau = 2 s, well under the 5 s floor.
        let data = synthetic_step(
            &StepSpec {
                y0: 40.0,
                k: 0.8,
                tau: 2.0,
                theta: 1.0,
                step_w: 30.0,
                n: 61,
                dt: 0.5,
                noise: 0.05,
            },
            &mut rng,
        );
        assert_eq!(fit_fopdt(&data, 30.0, MIN_EC_RESPONSE_C), None);
    }

    #[test]
    fn fit_fopdt_rejects_ec_response_under_3c() {
        let mut rng = Xorshift32(0x3344_5566);
        // k*step_w = 0.05*30 = 1.5 degC, under the 3 degC EC floor; tau/k
        // sign are otherwise valid so only the magnitude rule fires.
        let data = synthetic_step(
            &StepSpec {
                y0: 40.0,
                k: 0.05,
                tau: 35.0,
                theta: 20.0,
                step_w: 30.0,
                n: 61,
                dt: 5.0,
                noise: 0.02,
            },
            &mut rng,
        );
        assert_eq!(fit_fopdt(&data, 30.0, MIN_EC_RESPONSE_C), None);
    }

    #[test]
    fn derive_gains_rejects_kc_outside_ratio_band_directly() {
        let fit = Fopdt { k: 0.01, tau: 35.0, theta: 20.0 };
        let defaults = Gains { kc: 0.4, ti_s: 35.0 };
        assert_eq!(derive_device_gains(&fit, defaults), None);
    }

    #[test]
    fn derive_gains_rejects_non_positive_k_directly() {
        let fit = Fopdt { k: -0.1, tau: 35.0, theta: 20.0 };
        assert_eq!(derive_device_gains(&fit, Gains { kc: 0.4, ti_s: 35.0 }), None);
    }

    #[test]
    fn derive_gains_rejects_tau_under_5s_directly() {
        let fit = Fopdt { k: 0.8, tau: 4.0, theta: 1.0 };
        assert_eq!(derive_device_gains(&fit, Gains { kc: 0.4, ti_s: 35.0 }), None);
    }
}

//! Model trust monitor (design §3 / plan Task 27): a slow EWMA of the
//! absolute steady-state residual `|measured − predicted|` fan RPM. When the
//! model is persistently wrong by more than the fans' whole deadband — and
//! stays wrong for minutes — the controller must stop *learning* from it
//! (freeze RLS: adapting toward data we no longer trust would launder the
//! fault into the model) and correct only gently (trim at half gain), while
//! telling the user via `StatusFlag::ModelDistrust`.
//!
//! Fed by the controller on steady-gated samples ONLY (the same 20-sample
//! fan-window gate as the trim integrator): between steady windows no
//! evidence arrives and the last verdict stands.

/// EWMA smoothing factor per steady observation (~1 Hz while steady): a
/// time constant of ~50 samples, deliberately slow so a single transient
/// cannot flip the verdict.
pub const EWMA_ALPHA: f64 = 0.02;
/// Distrust threshold on the EWMA (RPM). 300 RPM is twice the allocator's
/// deadband: the model is not just off, it is off by more than the control
/// layer can hide.
pub const DISTRUST_RPM: f64 = 300.0;
/// How long the EWMA must stay above [`DISTRUST_RPM`] continuously before
/// `Distrust` is reported (seconds): 5 minutes, matching the trim's
/// minutes-scale tier — a real airflow/model fault persists; a hiccup ends.
pub const DISTRUST_SUSTAIN_S: f64 = 300.0;

/// The verdict returned by every [`TrustMonitor::observe`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trust {
    Ok,
    /// EWMA over [`DISTRUST_RPM`] sustained for [`DISTRUST_SUSTAIN_S`].
    Distrust,
}

/// See the module docs. Owned by the controller's Auto-mode loop state, so
/// it drops (resets) on Auto exit, like the trim.
#[derive(Debug, Clone, Default)]
pub struct TrustMonitor {
    /// EWMA of `|measured − predicted|` (RPM); starts at 0 (full trust).
    ewma_abs_residual: f64,
    /// `t_mono` when the EWMA first exceeded [`DISTRUST_RPM`] in the current
    /// over-threshold stretch; None while at/under the threshold.
    distrust_since: Option<f64>,
}

impl TrustMonitor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Current EWMA (RPM), for logging/inspection.
    #[allow(dead_code)]
    pub fn ewma(&self) -> f64 {
        self.ewma_abs_residual
    }

    /// Feed one steady-gated residual (same gate as the trim integrator).
    /// Updates the EWMA and returns the current trust state: `Distrust` only
    /// once the EWMA has been over [`DISTRUST_RPM`] for a continuous
    /// [`DISTRUST_SUSTAIN_S`]; the EWMA dropping back to/below the threshold
    /// clears the stretch immediately.
    pub fn observe(&mut self, t_mono: f64, abs_residual: f64) -> Trust {
        self.ewma_abs_residual =
            (1.0 - EWMA_ALPHA) * self.ewma_abs_residual + EWMA_ALPHA * abs_residual;
        if self.ewma_abs_residual > DISTRUST_RPM {
            let since = *self.distrust_since.get_or_insert(t_mono);
            if t_mono - since >= DISTRUST_SUSTAIN_S {
                return Trust::Distrust;
            }
        } else {
            self.distrust_since = None;
        }
        Trust::Ok
    }

    /// Back to full trust (EWMA 0, no over-threshold stretch). The
    /// controller resets by dropping its whole Auto loop state instead; kept
    /// as the explicit API for callers that hold on to a monitor.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ewma_math_and_zero_start() {
        let mut m = TrustMonitor::new();
        assert_eq!(m.ewma(), 0.0);
        assert_eq!(m.observe(0.0, 100.0), Trust::Ok);
        assert!((m.ewma() - EWMA_ALPHA * 100.0).abs() < 1e-12);
        assert_eq!(m.observe(1.0, 100.0), Trust::Ok);
        let expected = (1.0 - EWMA_ALPHA) * (EWMA_ALPHA * 100.0) + EWMA_ALPHA * 100.0;
        assert!((m.ewma() - expected).abs() < 1e-12);
    }

    #[test]
    fn sustained_over_threshold_distrusts_after_exactly_300s() {
        let mut m = TrustMonitor::new();
        // Constant 400 RPM residual at 1 Hz: the EWMA crosses 300 once
        // 400·(1 − 0.98^(t+1)) > 300, i.e. at t = 68; distrust follows 300 s
        // of continuous over-threshold, i.e. first at t = 368.
        let mut first_distrust = None;
        for t in 0..400 {
            if m.observe(f64::from(t), 400.0) == Trust::Distrust && first_distrust.is_none() {
                first_distrust = Some(t);
            }
        }
        assert_eq!(first_distrust, Some(368));
    }

    #[test]
    fn spike_does_not_distrust() {
        let mut m = TrustMonitor::new();
        // One enormous spike pushes the EWMA over the threshold instantly…
        assert_eq!(m.observe(0.0, 20_000.0), Trust::Ok);
        assert!(m.ewma() > DISTRUST_RPM);
        // …but it decays back under within ~15 s of clean samples: the 300 s
        // sustain is never reached, so the verdict stays Ok throughout.
        for t in 1..600 {
            assert_eq!(m.observe(f64::from(t), 0.0), Trust::Ok, "t={t}");
        }
        assert!(m.ewma() < 1.0);
    }

    #[test]
    fn recovery_clears_and_a_relapse_needs_a_fresh_300s() {
        let mut m = TrustMonitor::new();
        let mut t = 0.0;
        let mut feed = |m: &mut TrustMonitor, rpm: f64, n: usize| {
            let mut last = Trust::Ok;
            for _ in 0..n {
                last = m.observe(t, rpm);
                t += 1.0;
            }
            last
        };
        assert_eq!(feed(&mut m, 400.0, 400), Trust::Distrust);
        // Clean samples pull the EWMA under 300 → Ok again (stretch cleared).
        assert_eq!(feed(&mut m, 0.0, 60), Trust::Ok);
        assert!(m.ewma() < DISTRUST_RPM);
        // Relapse: over-threshold again, but distrust needs a FRESH 300 s —
        // 100 s of it is not enough…
        assert_eq!(feed(&mut m, 2000.0, 100), Trust::Ok);
        // …while riding it out to 300+ s is.
        assert_eq!(feed(&mut m, 2000.0, 250), Trust::Distrust);
    }

    #[test]
    fn reset_returns_to_full_trust() {
        let mut m = TrustMonitor::new();
        for t in 0..400 {
            m.observe(f64::from(t), 400.0);
        }
        assert_eq!(m.observe(400.0, 400.0), Trust::Distrust);
        m.reset();
        assert_eq!(m.ewma(), 0.0);
        assert_eq!(m.observe(401.0, 400.0), Trust::Ok);
    }
}

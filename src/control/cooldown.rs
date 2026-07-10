//! Adaptation command-cooldown gate (design doc §0, the primary fix for the
//! 2026-07-09 wind-up incident). The adaptation tier may consume a sample only
//! if the COMMANDED operating point `(cpu_w allocation, gpu_w PI target)` has
//! stayed within [`TOL_W`] of the current point, on both legs, for the entire
//! trailing [`WINDOW_S`].
//!
//! Compares against the WHOLE window, not the last step: a per-step threshold
//! is evaded by exactly the +2 W/5 s staircase that walked the trim to the
//! −400 pin (the same lesson as the RLS excitation gate's drifting-point
//! caveat). Gates on commanded values only — the observed side is owned by
//! the achievement gate (drawn ≈ commanded) and `is_steady` (fan end); each
//! gate owns one edge of `command → drawn power → fan RPM`.

/// Stationarity tolerance per leg (W). 2 W clears steady-state allocator/PI
/// dither (observed ±1.9 W holds in the captured session) without letting a
/// staircase through.
pub const TOL_W: f64 = 2.0;
/// Trailing window (s). 30 s, not more: the 20-sample `is_steady` gate stacks
/// on top for ~50 s of combined protection, and the KF noise model absorbs
/// the residual heat-soak error (design §0).
pub const WINDOW_S: f64 = 30.0;

/// One commanded operating point in the controller's ring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandedPoint {
    pub t_mono: f64,
    pub cpu_w: f64,
    pub gpu_w: f64,
}

/// True iff the ring proves the commanded point held within [`TOL_W`] on both
/// legs across the full trailing [`WINDOW_S`] ending at `now`:
/// - COVERAGE: at least one recorded point is `≥ WINDOW_S` old — without it
///   we have not yet OBSERVED a full window of stationarity, so the gate
///   stays closed (the first 30 s after Auto entry, and after any ring
///   clear). Absence of evidence is not stationarity.
/// - STATIONARITY: every point within the trailing window is within `TOL_W`
///   of the current (latest) commanded point on both legs.
pub fn cooldown_open(ring: &[CommandedPoint], now: f64) -> bool {
    let Some(cur) = ring.last() else {
        return false;
    };
    if !ring.iter().any(|p| now - p.t_mono >= WINDOW_S) {
        return false;
    }
    ring.iter()
        .filter(|p| now - p.t_mono <= WINDOW_S)
        .all(|p| (p.cpu_w - cur.cpu_w).abs() <= TOL_W && (p.gpu_w - cur.gpu_w).abs() <= TOL_W)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1 Hz flat history at `(cpu_w, gpu_w)` covering `t0..=t1` inclusive.
    fn flat(t0: i64, t1: i64, cpu_w: f64, gpu_w: f64) -> Vec<CommandedPoint> {
        (t0..=t1)
            .map(|t| CommandedPoint { t_mono: t as f64, cpu_w, gpu_w })
            .collect()
    }

    #[test]
    fn empty_or_short_history_is_closed() {
        // No evidence at all → closed.
        assert!(!cooldown_open(&[], 100.0));
        // 20 s of perfectly flat history: stationary, but no point is ≥30 s
        // old, so a full window has not been OBSERVED yet → still closed.
        let ring = flat(80, 100, 45.0, 60.0);
        assert!(!cooldown_open(&ring, 100.0));
    }

    #[test]
    fn flat_history_opens_after_30s() {
        // 31 flat points spanning exactly 30 s: the t=70 point is exactly
        // WINDOW_S old at now=100, satisfying coverage at the boundary.
        let ring = flat(70, 100, 45.0, 60.0);
        assert!(cooldown_open(&ring, 100.0));
    }

    #[test]
    fn staircase_never_opens() {
        // The incident shape: gpu_w climbs 0.4 W/s (= +2 W/5 s), cpu_w flat.
        // Each STEP is far inside TOL_W; the whole-window compare must still
        // reject it at every instant along the ramp.
        let mut ring: Vec<CommandedPoint> = Vec::new();
        for t in 70..=100 {
            let gpu_w = 60.0 + 0.4 * (t - 70) as f64;
            ring.push(CommandedPoint { t_mono: t as f64, cpu_w: 45.0, gpu_w });
            assert!(
                !cooldown_open(&ring, t as f64),
                "staircase opened the gate at t={t}"
            );
        }
    }

    #[test]
    fn small_dither_does_not_close_the_gate() {
        // ±1.9 W alternating dither on both legs around the held point (the
        // observed steady-state allocator/PI wobble), current point at the
        // hold, full 30 s coverage → must stay open, or adaptation would be
        // blocked forever at equilibrium. The gate measures excursion FROM
        // THE CURRENT POINT, so 1.9 < TOL_W is the property under test.
        let mut ring: Vec<CommandedPoint> = (70..100)
            .map(|t| {
                let d = if t % 2 == 0 { 1.9 } else { -1.9 };
                CommandedPoint { t_mono: t as f64, cpu_w: 45.0 + d, gpu_w: 60.0 + d }
            })
            .collect();
        ring.push(CommandedPoint { t_mono: 100.0, cpu_w: 45.0, gpu_w: 60.0 });
        assert!(cooldown_open(&ring, 100.0));
    }

    #[test]
    fn a_move_closes_then_reopens_30s_later() {
        // Long flat hold at gpu 60 W, then a 10 W jump at t=100, held after.
        let mut ring = flat(60, 99, 45.0, 60.0);
        ring.extend(flat(100, 135, 45.0, 70.0));
        // now=125: 60 W points (t=95..99) are still inside the window → closed.
        assert!(!cooldown_open(&ring[..=65], 125.0));
        // now=130+: only 70 W points remain within the trailing 30 s, and the
        // t=100 point provides coverage → open again.
        assert!(cooldown_open(&ring[..=70], 130.0));
        assert!(cooldown_open(&ring, 135.0));
    }

    #[test]
    fn nan_commanded_point_never_opens_the_gate() {
        // With the all(within-tol) shape NaN → false → closed automatically;
        // this test PINS that so a refactor to `!any(out_of_tol)` cannot
        // silently invert the NaN behavior (NaN comparisons are always false
        // on BOTH shapes' predicates — only one of them fails closed).
        let mut ring = flat(70, 100, 45.0, 60.0);
        ring[10].cpu_w = f64::NAN;
        assert!(!cooldown_open(&ring, 100.0));

        let mut ring = flat(70, 100, 45.0, 60.0);
        ring[20].gpu_w = f64::NAN;
        assert!(!cooldown_open(&ring, 100.0));

        // NaN in the CURRENT (latest) point → nothing can be "within TOL_W of
        // it" → closed.
        let mut ring = flat(70, 100, 45.0, 60.0);
        ring.last_mut().unwrap().gpu_w = f64::NAN;
        assert!(!cooldown_open(&ring, 100.0));
    }
}

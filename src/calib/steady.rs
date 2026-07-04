//! Steady-state detection over sample windows. Pure functions, no I/O:
//! calibration (Task 19/22) records a point only when the fan RPM window is
//! steady, and the trim integrator/RLS (Task 26) gates its updates the same
//! way.

/// Samples the tail window must span before we call it steady. At the 1 Hz
/// sampling rate this is 20 seconds of settling time.
pub const STEADY_N: usize = 20;

/// Max-min spread (RPM) the tail window may have and still count as steady.
pub const STEADY_RPM_TOLERANCE: f64 = 100.0;

/// Window is steady when it has >= `n` samples and the max-min spread of the
/// last `n` is <= `tolerance`.
///
/// Any NaN inside the tail (validity gaps land in windows as NaN, per the
/// charts convention) makes the window NOT steady: a sensor outage must never
/// fabricate a calibration point. NaNs older than the tail are ignored.
pub fn is_steady(window: &[f64], n: usize, tolerance: f64) -> bool {
    if window.len() < n {
        return false;
    }
    let tail = &window[window.len() - n..];
    // f64::max/min propagate nothing useful for NaN detection (they *ignore*
    // NaN), so reject it explicitly before computing the spread.
    if tail.iter().any(|v| v.is_nan()) {
        return false;
    }
    let max = tail.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min = tail.iter().copied().fold(f64::INFINITY, f64::min);
    max - min <= tolerance
}

/// Mean of the last `n` values. Returns None if the window has fewer than `n`
/// samples or the tail contains NaN (same rationale as [`is_steady`]: never
/// average across a sensor outage). Callers check [`is_steady`] first.
pub fn tail_mean(window: &[f64], n: usize) -> Option<f64> {
    if window.len() < n || n == 0 {
        return None;
    }
    let tail = &window[window.len() - n..];
    if tail.iter().any(|v| v.is_nan()) {
        return None;
    }
    Some(tail.iter().sum::<f64>() / n as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    const N: usize = 20;
    const TOL: f64 = 100.0;

    #[test]
    fn flat_window_is_steady_with_exact_mean() {
        let w = vec![1500.0; N];
        assert!(is_steady(&w, N, TOL));
        assert_eq!(tail_mean(&w, N), Some(1500.0));
    }

    #[test]
    fn ramp_wider_than_tolerance_is_not_steady() {
        // 1400..1600 over 20 samples: spread 200 > 100.
        let w: Vec<f64> = (0..N).map(|i| 1400.0 + 200.0 * i as f64 / 19.0).collect();
        assert!(!is_steady(&w, N, TOL));
    }

    #[test]
    fn spread_exactly_at_tolerance_is_steady() {
        // Alternating 1450/1550: spread exactly 100 (<= is steady).
        let w: Vec<f64> = (0..N)
            .map(|i| if i % 2 == 0 { 1450.0 } else { 1550.0 })
            .collect();
        assert!(is_steady(&w, N, TOL));
    }

    #[test]
    fn shorter_than_n_is_not_steady_and_has_no_mean() {
        let w = vec![1500.0; N - 1];
        assert!(!is_steady(&w, N, TOL));
        assert_eq!(tail_mean(&w, N), None);
    }

    #[test]
    fn values_older_than_tail_are_ignored() {
        let mut w = vec![9999.0; 10];
        w.extend(std::iter::repeat_n(1500.0, N));
        assert!(is_steady(&w, N, TOL));
    }

    #[test]
    fn nan_in_tail_means_not_steady_and_no_mean() {
        let mut w = vec![1500.0; N];
        w[N / 2] = f64::NAN;
        assert!(!is_steady(&w, N, TOL));
        assert_eq!(tail_mean(&w, N), None);
    }

    #[test]
    fn nan_older_than_tail_is_still_steady() {
        let mut w = vec![f64::NAN; 5];
        w.extend(std::iter::repeat_n(1500.0, N));
        assert!(is_steady(&w, N, TOL));
        assert_eq!(tail_mean(&w, N), Some(1500.0));
    }

    #[test]
    fn tail_mean_averages_exactly_the_last_n() {
        // 10 junk values, then a tail whose mean is easy to verify:
        // alternating -50/+50 around 1500 keeps the mean at exactly 1500,
        // and any junk leaking into the average would drag it far off.
        let mut w = vec![42.0; 10];
        let tail: Vec<f64> = (0..N)
            .map(|i| if i % 2 == 0 { 1450.0 } else { 1550.0 })
            .collect();
        w.extend(&tail);
        let expect = tail.iter().sum::<f64>() / N as f64;
        assert_eq!(tail_mean(&w, N), Some(expect));
        assert_eq!(expect, 1500.0);
    }
}

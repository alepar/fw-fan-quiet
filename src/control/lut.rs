//! Clock→watts lookup table built by the calibration sweep (Task 19) and
//! consumed as the feedforward term of the GPU watts→clock inner PI loop
//! (Task 24). Persisted in the state file (Task 21), hence the serde derives.

/// Piecewise-linear map from locked SM clock (MHz) to steady-state GPU power
/// (watts). Points are kept sorted by clock.
///
/// Measured data can be mildly non-monotonic (noise, boost quirks). We do not
/// try to repair it: the inverse lookup scans brackets from the highest clock
/// down and takes the first one whose interpolated watts fit the budget, so
/// non-monotonicity only ever yields a *conservative* (lower-clock) answer.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ClockWattsLut {
    /// (mhz, watts), sorted ascending by mhz, unique mhz.
    points: Vec<(u32, f64)>,
}

impl ClockWattsLut {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Insert a calibration point, keeping points sorted by mhz. A point with
    /// the same mhz replaces the old one (re-running a sweep step updates it).
    pub fn insert(&mut self, mhz: u32, watts: f64) {
        match self.points.binary_search_by_key(&mhz, |&(m, _)| m) {
            Ok(i) => self.points[i].1 = watts,
            Err(i) => self.points.insert(i, (mhz, watts)),
        }
    }

    // Introspection conveniences: `clock_for_watts` below is the only entry
    // the control path consumes; len/is_empty/watts_for_clock serve tests
    // and future UI/telemetry (calibration summary, predicted watts).
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Inverse lookup: the largest clock whose predicted watts stay <= the
    /// target budget (we command a clock to ACHIEVE a watts budget, so we
    /// must stay under it). Linear interpolation between bracketing points;
    /// clamped to [lowest_mhz, highest_mhz] — never below the lowest known
    /// clock even if the budget is tiny, never above the highest even if the
    /// budget is huge. None if the LUT is empty.
    pub fn clock_for_watts(&self, target_w: f64) -> Option<u32> {
        let &(hi_mhz, hi_w) = self.points.last()?;
        if target_w >= hi_w {
            return Some(hi_mhz); // clamp high
        }
        // Scan brackets from the highest clock down; take the first one whose
        // interpolated watts fit the budget. With monotonic data this is the
        // exact inverse; with a non-monotonic dip it lands at (or below) the
        // highest fitting clock — conservative, never over budget.
        for pair in self.points.windows(2).rev() {
            let (m0, w0) = pair[0];
            let (m1, w1) = pair[1];
            if target_w >= w1 {
                // Believed unreachable; kept as defense-in-depth. Induction:
                // the pre-loop clamp guarantees target < the first bracket's
                // w1 (the highest point's watts), and falling through a
                // bracket implies target < its w0 (both fall-through paths
                // require it), which is the next bracket's w1. Should the
                // scan ever change, returning the bracket top — the largest
                // in-bracket clock whose watts fit — stays correct.
                return Some(m1);
            }
            if w1 > w0 && target_w >= w0 {
                // w0 <= target < w1: interpolate, floored (stay under budget).
                let frac = (target_w - w0) / (w1 - w0);
                let mhz = f64::from(m0) + frac * (f64::from(m1) - f64::from(m0));
                // The epsilon absorbs float error just below an exact MHz so
                // flooring cannot knock an exact answer down by one.
                return Some((mhz + 1e-6).floor() as u32);
            }
            // Whole bracket above the budget (or descending): keep scanning.
        }
        Some(self.points[0].0) // clamp low: never below the lowest known clock
    }

    /// Forward lookup for telemetry/UI: interpolated watts at `mhz`, clamped
    /// to the LUT's clock range at both ends. None if the LUT is empty.
    #[allow(dead_code)]
    pub fn watts_for_clock(&self, mhz: u32) -> Option<f64> {
        let &(lo_mhz, lo_w) = self.points.first()?;
        let &(hi_mhz, hi_w) = self.points.last()?;
        if mhz <= lo_mhz {
            return Some(lo_w);
        }
        if mhz >= hi_mhz {
            return Some(hi_w);
        }
        match self.points.binary_search_by_key(&mhz, |&(m, _)| m) {
            Ok(i) => Some(self.points[i].1),
            Err(i) => {
                // Between points[i-1] and points[i] (both exist: mhz is
                // strictly inside the clock range).
                let (m0, w0) = self.points[i - 1];
                let (m1, w1) = self.points[i];
                let frac = f64::from(mhz - m0) / f64::from(m1 - m0);
                Some(w0 + frac * (w1 - w0))
            }
        }
    }
}

impl Default for ClockWattsLut {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 3-point LUT from the plan: 30 W @ 1200, 60 W @ 2000, 100 W @ 2800.
    fn lut3() -> ClockWattsLut {
        let mut lut = ClockWattsLut::new();
        lut.insert(1200, 30.0);
        lut.insert(2000, 60.0);
        lut.insert(2800, 100.0);
        lut
    }

    #[test]
    fn clock_for_watts_exact_point() {
        assert_eq!(lut3().clock_for_watts(60.0), Some(2000));
    }

    #[test]
    fn clock_for_watts_interpolates_between_points() {
        // 45 W is halfway between 30 W @ 1200 and 60 W @ 2000 -> 1600 MHz.
        assert_eq!(lut3().clock_for_watts(45.0), Some(1600));
    }

    #[test]
    fn clock_for_watts_clamps_high() {
        assert_eq!(lut3().clock_for_watts(120.0), Some(2800));
    }

    #[test]
    fn clock_for_watts_clamps_low_never_below_lowest_known() {
        assert_eq!(lut3().clock_for_watts(10.0), Some(1200));
    }

    #[test]
    fn watts_for_clock_interpolates() {
        let w = lut3().watts_for_clock(1600).unwrap();
        assert!((w - 45.0).abs() < 1e-9, "got {w}");
    }

    #[test]
    fn watts_for_clock_exact_and_clamped() {
        let lut = lut3();
        assert_eq!(lut.watts_for_clock(2000), Some(60.0));
        assert_eq!(lut.watts_for_clock(500), Some(30.0)); // clamp low
        assert_eq!(lut.watts_for_clock(4000), Some(100.0)); // clamp high
    }

    #[test]
    fn insert_unsorted_ends_up_sorted() {
        let mut lut = ClockWattsLut::new();
        lut.insert(2800, 100.0);
        lut.insert(1200, 30.0);
        lut.insert(2000, 60.0);
        // Sorted order is observable through interpolation correctness.
        assert_eq!(lut.len(), 3);
        assert_eq!(lut.clock_for_watts(45.0), Some(1600));
        assert_eq!(lut.clock_for_watts(60.0), Some(2000));
    }

    #[test]
    fn insert_duplicate_mhz_replaces_len_unchanged() {
        let mut lut = lut3();
        lut.insert(2000, 70.0);
        assert_eq!(lut.len(), 3);
        assert_eq!(lut.watts_for_clock(2000), Some(70.0));
    }

    #[test]
    fn empty_lut_returns_none() {
        let lut = ClockWattsLut::new();
        assert!(lut.is_empty());
        assert_eq!(lut.clock_for_watts(50.0), None);
        assert_eq!(lut.watts_for_clock(2000), None);
    }

    #[test]
    fn single_point_lut_clamps_both_ways() {
        let mut lut = ClockWattsLut::new();
        lut.insert(2000, 60.0);
        assert_eq!(lut.clock_for_watts(10.0), Some(2000));
        assert_eq!(lut.clock_for_watts(200.0), Some(2000));
        assert_eq!(lut.watts_for_clock(100), Some(60.0));
        assert_eq!(lut.watts_for_clock(9000), Some(60.0));
    }

    #[test]
    fn non_monotonic_data_yields_conservative_lower_clock() {
        // Dip at 2000 MHz: 60 W @ 1200, 50 W @ 2000 (dip), 100 W @ 2800.
        let mut lut = ClockWattsLut::new();
        lut.insert(1200, 60.0);
        lut.insert(2000, 50.0);
        lut.insert(2800, 100.0);
        // 55 W: scanning from the top, the (2000, 2800) bracket interpolates
        // 55 W at 2080 MHz — the highest clock fitting the budget.
        assert_eq!(lut.clock_for_watts(55.0), Some(2080));
        // 40 W fits nowhere (min measured is 50 W): clamp to lowest clock.
        assert_eq!(lut.clock_for_watts(40.0), Some(1200));
    }

    #[test]
    fn serde_round_trip() {
        let lut = lut3();
        let json = serde_json::to_string(&lut).unwrap();
        let back: ClockWattsLut = serde_json::from_str(&json).unwrap();
        assert_eq!(back, lut);
    }
}

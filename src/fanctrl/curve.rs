//! Piecewise-linear fw-fanctrl curve model (design doc §2.1): a faithful copy
//! of `FanController.py`'s own interpolation — points kept in **file order**
//! (never sorted), flat clamp below the first point and above the last,
//! `int()` **truncation** (not rounding) when reading a duty back off the
//! continuous interpolation.
//!
//! `duty_at` is fw-fanctrl's own forward direction (temperature -> duty). The
//! rest of this module is the *inverse* the loop needs to turn a target duty
//! into a setpoint temperature: `tread` (the maximal temperature interval
//! that reads back as a given integer duty), `t_star` (its centre — the
//! "tread" concept from §2.1/§6: duty is int-truncated, so the inverse curve
//! is a staircase, and centring the setpoint on a tread's width is a free
//! deadband against that staircase) and `slope_at` (§2.7's steepness test).

use std::fmt;

/// `Curve::from_points` rejects a curve whose duty is not non-decreasing in
/// file order. A domain error only — see design doc §2.1: reusing a flag
/// here (`STEEP CURVE` or inventing one) is explicitly Task 16's job
/// (`CURVE INVALID`), not this constructor's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveError {
    /// No points at all: `duty_at`/`tread` would have nothing to clamp to.
    Empty,
    /// `points[i + 1].1 < points[i].1` for some `i`: a duty's preimage would
    /// be a union of disjoint intervals, `tread` would be ill-posed, and a
    /// setpoint could land on a falling segment.
    DescendingSegment { at_index: usize },
}

impl fmt::Display for CurveError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CurveError::Empty => write!(f, "curve has no points"),
            CurveError::DescendingSegment { at_index } => write!(
                f,
                "curve duty decreases from point {at_index} to {} (file order)",
                at_index + 1
            ),
        }
    }
}

impl std::error::Error for CurveError {}

/// A validated fw-fanctrl curve: points in file order, duty non-decreasing
/// (ties allowed — a flat tread is normal, e.g. `quiet16`'s 0->55 lead-in).
#[derive(Debug, Clone, PartialEq)]
pub struct Curve {
    /// (temperature °C, duty %), file order, `points[i+1].1 >= points[i].1`.
    points: Vec<(f64, u8)>,
}

impl Curve {
    /// Validates and wraps a point list. Points are **not** sorted by
    /// temperature: file order is load-bearing (matches fw-fanctrl, which
    /// never sorts either) and a descending *duty* segment is rejected
    /// outright rather than silently reordered.
    pub fn from_points(points: Vec<(f64, u8)>) -> Result<Curve, CurveError> {
        if points.is_empty() {
            return Err(CurveError::Empty);
        }
        for (i, w) in points.windows(2).enumerate() {
            if w[1].1 < w[0].1 {
                return Err(CurveError::DescendingSegment { at_index: i });
            }
        }
        Ok(Curve { points })
    }

    /// The lowest duty this curve ever reports (flat-clamped below the first
    /// point).
    fn min_duty(&self) -> u8 {
        self.points[0].1
    }

    /// The highest duty this curve ever reports (flat-clamped above the last
    /// point).
    fn max_duty(&self) -> u8 {
        self.points[self.points.len() - 1].1
    }

    /// The continuous (pre-truncation) interpolated duty at `t`, flat-clamped
    /// outside `[first.0, last.0]`. Internal: `duty_at` truncates this;
    /// `tread`'s search inverts it.
    fn continuous_duty_at(&self, t: f64) -> f64 {
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        if t <= first.0 {
            return first.1 as f64;
        }
        if t >= last.0 {
            return last.1 as f64;
        }
        for w in self.points.windows(2) {
            let (t0, d0) = w[0];
            let (t1, d1) = w[1];
            if t <= t1 {
                if t1 == t0 {
                    // Vertical jump (two points at the same temperature, a
                    // legal but degenerate curve): resolve to the arriving
                    // (higher) duty. Both segments agree at every OTHER
                    // point by construction, so this only matters exactly
                    // at t0==t1.
                    return d1 as f64;
                }
                let frac = (t - t0) / (t1 - t0);
                return d0 as f64 + frac * (d1 as f64 - d0 as f64);
            }
        }
        last.1 as f64 // unreachable given the t >= last.0 clamp above
    }

    /// fw-fanctrl's own forward direction: temperature -> commanded duty.
    /// `int()` truncation, matching `FanController.py` (verified on cool16:
    /// `T_eff` 51.8 -> 21, not 22).
    pub fn duty_at(&self, t: f64) -> u8 {
        // continuous_duty_at is always within [0, 100] (points are u8), so
        // this cast neither truncates a huge magnitude nor needs a manual
        // floor: `as u8` on a nonnegative f64 truncates toward zero, exactly
        // Python's int().
        self.continuous_duty_at(t) as u8
    }

    /// Smallest `t` with `continuous_duty_at(t) >= y`. `tread` calls this
    /// with `y` equal to a duty in `[min_duty, max_duty]` (the tread's near
    /// bound) and with `y` one more than that (its far bound) — so `y` can
    /// legitimately reach `max_duty + 1`, one past anything the curve ever
    /// attains; see the fallback below, which is exactly what makes that
    /// call meaningful rather than unreachable.
    fn first_t_reaching(&self, y: f64) -> f64 {
        for w in self.points.windows(2) {
            let (t0, d0) = w[0];
            let (t1, d1) = w[1];
            let (d0, d1) = (d0 as f64, d1 as f64);
            if d1 >= y {
                if d0 >= y {
                    return t0;
                }
                // d0 < y <= d1, so d1 > d0 (a real, non-vertical rise) unless
                // t1 == t0 (vertical jump), handled the same way
                // continuous_duty_at resolves it: the jump lands exactly at
                // t0==t1 regardless of where inside [d0, d1) y falls.
                if t1 == t0 {
                    return t0;
                }
                let frac = (y - d0) / (d1 - d0);
                return t0 + frac * (t1 - t0);
            }
        }
        // y > max_duty: the curve never attains it. This is not a caller
        // bug — it is exactly the far-bound call for a duty at (or one
        // below) `max_duty`, per the doc comment above. §2.1's "maximal
        // interval" is bounded by the curve's own domain at that duty, and
        // the domain's own edge (the last point) is precisely the right
        // value to report: `tread` compares this against its near bound
        // and folds an equal/degenerate result to `None` itself, so this
        // fallback only has to be `>=` any real near bound, never a sentinel.
        self.points[self.points.len() - 1].0
    }

    /// The maximal temperature interval where `duty_at(t) == d` (design doc
    /// §2.1), computed by the **same rule at every duty** — `min_duty()`/
    /// `max_duty()` (the curve's own floor/ceiling) get no special case.
    /// `None` when `d` is outside `[min_duty, max_duty]`, when the curve
    /// jumps clean over `d` (a vertical, same-temperature duty step) — the
    /// "skipped integer" `nearest_tread` snaps around — *or*, per §2.1's
    /// settled endpoint semantics, when `d` is the floor/ceiling duty and
    /// the curve attains it only at a single defining point with no flat
    /// run on that side (true of both `quiet16` and `cool16` at their
    /// ceiling: each reaches its top duty only at its very last point).
    ///
    /// The interval is unbounded in principle at the curve's own floor or
    /// ceiling — fw-fanctrl reports that duty at any arbitrarily low/high
    /// temperature past the curve's own domain — but this module's domain
    /// ends at `points.first().0`/`points.last().0`, and §2.1 now says the
    /// tread is that unbounded interval **intersected with the curve's own
    /// domain**, never left open. Where the curve defines a genuine flat
    /// run at that extreme (both `quiet16` and `cool16` at their floor),
    /// the intersection is a real, finite, non-empty interval, same shape
    /// as any interior duty. Where the extreme is attained only
    /// instantaneously (both curves at their ceiling), the intersection is
    /// empty and this returns `None` — but that is not itself an error:
    /// `nearest_tread` (§2.3) already snaps a duty with no tread of its own
    /// to the nearest one that has one, so a target sitting exactly on such
    /// a ceiling still resolves to a finite T* one duty in. Either way, no
    /// caller downstream of `tread`/`t_star` can be handed a non-finite
    /// temperature again.
    pub fn tread(&self, d: u8) -> Option<(f64, f64)> {
        if d < self.min_duty() || d > self.max_duty() {
            return None;
        }
        let t_lo = self.first_t_reaching(d as f64);
        let t_hi = self.first_t_reaching(d as f64 + 1.0);
        if t_lo >= t_hi {
            None
        } else {
            Some((t_lo, t_hi))
        }
    }

    /// Centre of `tread(d)` — the setpoint a target duty resolves to.
    /// `None` exactly when `tread(d)` is `None`; whenever it is `Some`,
    /// both of `tread`'s endpoints are finite (§2.1), so this is too —
    /// `t_star` is never `±inf`.
    pub fn t_star(&self, d: u8) -> Option<f64> {
        self.tread(d).map(|(lo, hi)| (lo + hi) / 2.0)
    }

    /// Segment slope in %/°C at `t`; `0.0` on the flat clamps (strictly
    /// outside `[first.0, last.0]`). At a temperature shared by two segments
    /// (an interior breakpoint), resolves to the segment **starting** there
    /// (the later of the two) — an arbitrary but documented tie-break, since
    /// the design only specifies the clamp value, not this case.
    pub fn slope_at(&self, t: f64) -> f64 {
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        if t < first.0 || t > last.0 {
            return 0.0;
        }
        let mut idx = 0;
        for (i, w) in self.points.windows(2).enumerate() {
            if w[0].0 <= t {
                idx = i;
            } else {
                break;
            }
        }
        let (t0, d0) = self.points[idx];
        let (t1, d1) = self.points[idx + 1];
        if t1 == t0 {
            // Vertical jump: infinite slope. Not exercised by quiet16/cool16
            // (neither has one); kept so this never divides by zero on a
            // curve that does.
            return f64::INFINITY;
        }
        (d1 as f64 - d0 as f64) / (t1 - t0)
    }

    /// The lowest duty with `tread(d).is_some()` — always `min_duty()`
    /// unless the curve has a same-temperature jump exactly at its floor
    /// (checked generically rather than assumed, since `from_points` does
    /// not reject that case).
    pub fn min_tread_duty(&self) -> u8 {
        for d in self.min_duty()..=self.max_duty() {
            if self.tread(d).is_some() {
                return d;
            }
        }
        self.min_duty() // unreachable in practice: the floor always has one
    }

    /// Resolves an arbitrary integer duty to the nearest one this curve
    /// actually has a tread for (design doc §2.3: a snapped target the curve
    /// skips over must fall back to something reachable). Ties (equal
    /// distance below and above) prefer the **lower** duty (quieter).
    /// `None` below `min_tread_duty()` — there is nothing lower to fall back
    /// to.
    pub fn nearest_tread(&self, d: u8) -> Option<u8> {
        if d < self.min_tread_duty() {
            return None;
        }
        if self.tread(d).is_some() {
            return Some(d);
        }
        // Expand outward by distance; lower checked before upper at each
        // distance so a tie prefers lower, per the doc comment above.
        for k in 1..=100u16 {
            if k <= u16::from(d) {
                let lower = d - k as u8;
                if self.tread(lower).is_some() {
                    return Some(lower);
                }
            }
            if u16::from(d) + k <= 100 {
                let upper = d + k as u8;
                if self.tread(upper).is_some() {
                    return Some(upper);
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §Facts / brief reference data, used verbatim and inline (this task
    /// must not depend on Task 2's `tests/fixtures/` corpus).
    fn quiet16() -> Curve {
        Curve::from_points(vec![
            (0.0, 15),
            (55.0, 15),
            (65.0, 21),
            (75.0, 31),
            (82.0, 37),
            (88.0, 55),
            (95.0, 100),
        ])
        .unwrap()
    }

    fn cool16() -> Curve {
        Curve::from_points(vec![
            (0.0, 20),
            (50.0, 20),
            (60.0, 30),
            (70.0, 42),
            (85.0, 100),
        ])
        .unwrap()
    }

    // --- Step 2: duty_at interpolation + truncation ---------------------

    #[test]
    fn quiet16_duty_at_matches_defining_points() {
        let c = quiet16();
        assert_eq!(c.duty_at(0.0), 15);
        assert_eq!(c.duty_at(55.0), 15);
        assert_eq!(c.duty_at(65.0), 21);
        assert_eq!(c.duty_at(75.0), 31);
        assert_eq!(c.duty_at(82.0), 37);
        assert_eq!(c.duty_at(88.0), 55);
        assert_eq!(c.duty_at(95.0), 100);
    }

    #[test]
    fn quiet16_duty_at_interpolates_and_clamps() {
        let c = quiet16();
        // Below the first point: flat clamp.
        assert_eq!(c.duty_at(-40.0), 15);
        // Above the last point: flat clamp.
        assert_eq!(c.duty_at(200.0), 100);
        // Midpoint of the 65->75 segment (slope 1.0 %/°C): 21 + 5 = 26.
        assert_eq!(c.duty_at(70.0), 26);
    }

    #[test]
    fn cool16_truncation_case_from_the_facts() {
        // Verified truncation case on cool16: T_eff 51.8 -> duty 21 (not the
        // rounded 22): 20 + (51.8-50)/(60-50)*10 = 21.8, int() truncates.
        let c = cool16();
        assert_eq!(c.duty_at(51.8), 21);
    }

    // --- Step 3: descending segment rejected -----------------------------

    #[test]
    fn descending_duty_segment_is_rejected() {
        let err = Curve::from_points(vec![(0.0, 50), (10.0, 30)]).unwrap_err();
        assert_eq!(err, CurveError::DescendingSegment { at_index: 0 });
    }

    #[test]
    fn descending_segment_error_is_a_plain_domain_error() {
        // No flag, no panic: just a value the caller can match on and
        // display. std::error::Error is implemented so `?` composes.
        let err = Curve::from_points(vec![(0.0, 50), (10.0, 30)]).unwrap_err();
        let _: &dyn std::error::Error = &err;
        assert_eq!(
            err.to_string(),
            "curve duty decreases from point 0 to 1 (file order)"
        );
    }

    #[test]
    fn empty_curve_is_rejected() {
        assert_eq!(Curve::from_points(vec![]).unwrap_err(), CurveError::Empty);
    }

    #[test]
    fn flat_and_rising_segments_are_accepted() {
        // A tie (flat segment) is not a descent.
        assert!(Curve::from_points(vec![(0.0, 15), (10.0, 15), (20.0, 30)]).is_ok());
    }

    // --- Step 4: tread / t_star / slope_at -------------------------------

    #[test]
    fn quiet16_treads_at_each_defining_duty() {
        let c = quiet16();
        // Flat lead-in 0->55 at duty 15, then rising 0.6 %/°C into the 65
        // point: duty stays 15 (truncated) until the continuous value hits
        // 16, at 55 + 1/0.6 = 56.6667. Floor duty: unbounded below.
        let (lo, hi) = c.tread(15).unwrap();
        assert_eq!(lo, 0.0); // clamped to quiet16's own first point (§2.1):
        // a genuine flat lead-in (0->55 at duty 15), so the floor tread is
        // finite, not the old NEG_INFINITY.
        assert!((hi - 56.666_666_666_666_67).abs() < 1e-9, "got {hi}");

        // duty 21 is hit exactly at t=65 (a defining point); the next
        // segment (65->75, slope 1.0 %/°C) reaches 22 at t=66.
        assert_eq!(c.tread(21), Some((65.0, 66.0)));

        // Ceiling duty (100): attained only at the single last point
        // (95,100), no flat run — §2.1's "single instantaneous point" case,
        // so `tread` itself has nothing to report (see
        // `nearest_tread_snaps_the_ceiling_to_the_adjacent_duty` for what
        // actually resolves a target sitting on it).
        assert_eq!(c.tread(100), None);
    }

    #[test]
    fn cool16_treads_at_each_defining_duty() {
        let c = cool16();
        // Flat lead-in 0->50 at duty 20; rising into 60 at 1.0 %/°C reaches
        // 21 at t=51.
        let (lo, hi) = c.tread(20).unwrap();
        assert_eq!(lo, 0.0); // clamped to cool16's own first point (§2.1)
        assert_eq!(hi, 51.0);

        // 60->70 rises 30->42 (1.2 %/°C): duty 30 at t=60 exactly, reaches
        // 31 at 60 + 1/1.2 = 60.8333.
        let (lo, hi) = c.tread(30).unwrap();
        assert_eq!(lo, 60.0);
        assert!((hi - 60.833_333_333_333_34).abs() < 1e-9, "got {hi}");

        // Ceiling duty (100): same "single instantaneous point" case as
        // quiet16 — attained only at the last point (85,100), no flat run.
        assert_eq!(c.tread(100), None);
    }

    #[test]
    fn tread_none_outside_the_curves_range() {
        let c = quiet16();
        assert_eq!(c.tread(14), None); // below the floor (15)
        assert_eq!(c.tread(101), None); // above the ceiling (100, u8 max anyway)
    }

    #[test]
    fn t_star_is_the_tread_midpoint() {
        let c = quiet16();
        // tread(21) == (65, 66) -> centre 65.5.
        assert_eq!(c.t_star(21), Some(65.5));
        assert_eq!(c.t_star(14), None);
    }

    #[test]
    fn slope_at_inside_and_at_segment_boundaries() {
        let c = quiet16();
        // Flat lead-in: 0 %/°C, tested at an interior point and at the
        // clamp boundary t=0 itself (t=0 belongs to the curve, not the
        // clamp: the clamp is strictly t<0).
        assert_eq!(c.slope_at(30.0), 0.0);
        assert_eq!(c.slope_at(0.0), 0.0);
        // 65->75 segment: (31-21)/(75-65) = 1.0 %/°C, at an interior point
        // and at its right boundary (t=75, shared with the next segment;
        // slope_at resolves boundaries to the segment starting there).
        assert!((c.slope_at(70.0) - 1.0).abs() < 1e-9);
        // At t=75 the NEXT segment (75->82, (37-31)/(82-75) ~= 0.857) wins,
        // by the documented right-preferring tie-break.
        assert!(
            (c.slope_at(75.0) - 6.0 / 7.0).abs() < 1e-9,
            "got {}",
            c.slope_at(75.0)
        );
        // Strictly outside the domain: flat clamp, 0.
        assert_eq!(c.slope_at(-10.0), 0.0);
        assert_eq!(c.slope_at(200.0), 0.0);
    }

    #[test]
    fn cool16_slope_steep_segment() {
        let c = cool16();
        // 70->85 segment: (100-42)/(85-70) = 58/15 ~= 3.8667 %/°C (steep,
        // > 2 %/°C per §2.7 — this is the curve §2.7 names as steep above
        // 70 °C).
        assert!((c.slope_at(78.0) - 58.0 / 15.0).abs() < 1e-9);
    }

    // --- Step 5: nearest_tread / min_tread_duty --------------------------

    #[test]
    fn min_tread_duty_is_the_curve_floor() {
        assert_eq!(quiet16().min_tread_duty(), 15);
        assert_eq!(cool16().min_tread_duty(), 20);
    }

    #[test]
    fn nearest_tread_returns_self_when_reachable() {
        let c = quiet16();
        assert_eq!(c.nearest_tread(21), Some(21));
    }

    #[test]
    fn nearest_tread_snaps_the_ceiling_to_the_adjacent_duty() {
        // Neither curve's max duty has a tread of its own (§2.1: attained
        // only at a single point, no flat run) — this is the existing
        // "skipped integer" fallback (§2.3) absorbing the new empty-ceiling
        // case, not new logic. Both curves' 99 has a (narrow but real)
        // tread just below the ceiling, so that's what a target of 100
        // resolves to.
        assert_eq!(quiet16().tread(100), None);
        assert_eq!(quiet16().nearest_tread(100), Some(99));
        assert_eq!(cool16().tread(100), None);
        assert_eq!(cool16().nearest_tread(100), Some(99));
    }

    #[test]
    fn nearest_tread_below_floor_is_none() {
        let c = quiet16();
        assert_eq!(c.nearest_tread(14), None);
        assert_eq!(c.nearest_tread(0), None);
    }

    #[test]
    fn nearest_tread_skipped_integer_resolves_to_nearest_lower() {
        // Synthetic curve with a genuine skip: a vertical jump at t=50 from
        // duty 10 straight to 20, so 11..=19 have no tread at all. Neither
        // quiet16 nor cool16 has a real skip (a continuous, non-descending
        // piecewise-linear curve attains every integer between its floor
        // and ceiling by the intermediate value theorem) — a same-
        // temperature jump is the only way to construct one, so this test
        // builds its own curve rather than reusing quiet16/cool16.
        let c = Curve::from_points(vec![(0.0, 10), (50.0, 10), (50.0, 20), (100.0, 100)]).unwrap();
        assert_eq!(c.tread(15), None); // confirm it is genuinely skipped
        // 15 is equidistant from 10 and 20; ties prefer lower.
        assert_eq!(c.nearest_tread(15), Some(10));
        // 17 is closer to 20 (distance 3) than to 10 (distance 7).
        assert_eq!(c.nearest_tread(17), Some(20));
        // 12 is closer to 10 (distance 2) than to 20 (distance 8).
        assert_eq!(c.nearest_tread(12), Some(10));
    }

    // ---- fw-fanctrl-loop-nez: the curve/arbiter seam — no ±inf tread ----

    #[test]
    fn tread_at_min_and_max_duty_matches_the_decided_semantics() {
        // §2.1: the floor/ceiling get no special case — computed the same
        // way as every other duty. Both curves define a genuine flat
        // lead-in at their floor, so the floor tread is finite and starts
        // exactly at the curve's own first point; both attain their
        // ceiling duty only at a single defining point (no flat run), so
        // the ceiling tread is empty.
        for c in [quiet16(), cool16()] {
            let lo_d = c.min_duty();
            let hi_d = c.max_duty();
            let (t_lo, t_hi) = c.tread(lo_d).expect("floor tread must be Some");
            assert_eq!(
                t_lo, c.points[0].0,
                "floor tread must start at the curve's first point"
            );
            assert!(
                t_hi.is_finite() && t_hi > t_lo,
                "floor tread must be finite and non-empty"
            );
            assert_eq!(
                c.tread(hi_d),
                None,
                "ceiling tread must be empty (no flat run there)"
            );
        }
    }

    #[test]
    fn t_star_is_none_or_finite_across_the_whole_duty_range() {
        // Standing invariant (fw-fanctrl-loop-nez): no future curve edit
        // can reintroduce a ±inf T* without this failing, on either test
        // curve, at every duty the curve reports.
        for c in [quiet16(), cool16()] {
            for d in c.min_duty()..=c.max_duty() {
                if let Some(ts) = c.t_star(d) {
                    assert!(ts.is_finite(), "t_star({d}) = {ts} is not finite");
                }
            }
        }
    }
}

//! Duty<->RPM lookup the loop snaps a user RPM target through (design doc
//! §2.3): seeded from a one-time measurement (§Facts), refined passively as
//! the controller observes steady windows. Persisted in `state.json` (Task
//! 13 owns the persisted-state wiring; this module only owns the table
//! itself and its serde shape).

use std::collections::BTreeMap;

/// Measured duty->RPM points (§Facts, validated on-machine 2026-09-08: at
/// `quiet16` EC max 75 °C the curve gives duty 31 and the fans ran a
/// measured 2649 RPM against this table's interpolated 2638 — under 0.5 %
/// error).
const SEED_POINTS: [(u8, f64); 10] = [
    (15, 1195.0),
    (20, 1670.0),
    (27, 2300.0),
    (30, 2560.0),
    (36, 3030.0),
    (40, 3380.0),
    (44, 3670.0),
    (48, 3950.0),
    (52, 4180.0),
    (85, 5920.0),
];

/// A refinement is rejected outright (no update at all, existing/absent
/// entry untouched) when the observed mean is further than this fraction of
/// the entry's current value — design doc §2.3's guard against a `GPU HOT`
/// or budget-bound window (duty fw-fanctrl actually ran a tread away from
/// the target's) corrupting the table by a full tread.
const REFINE_REJECT_FRACTION: f64 = 0.25;

/// Blend weight for the observed mean in `refine`'s
/// `rpm <- 0.8*rpm + 0.2*mean`.
const REFINE_MEAN_WEIGHT: f64 = 0.2;

/// Margin a clamped refinement is pushed past a neighbouring duty's RPM by,
/// to keep the table **strictly** increasing rather than merely
/// non-decreasing (a tie would make `duty_for_rpm`'s "nearest, ties go down"
/// rule and `rpm_for_duty`'s interpolation both operate on a degenerate,
/// zero-width bracket). Tiny relative to real RPM magnitudes (~1000s), so it
/// never itself becomes the dominant term across repeated refinements.
const MONOTONE_MARGIN_RPM: f64 = 1e-6;

/// Piecewise-linear duty (%) -> RPM map, flat-clamped outside its known
/// range. `Default` is the ten seeded points (§Facts) and is also this
/// type's serde default, so a legacy `state.json` predating this field
/// loads the seed unchanged (see the `serde_default_...` test below, which
/// exercises that specifically via `#[serde(default)]` on a field of this
/// type — this module does not own `state.json` itself, Task 13 does).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DutyRpmTable {
    points: BTreeMap<u8, f64>,
}

impl Default for DutyRpmTable {
    fn default() -> Self {
        DutyRpmTable {
            points: SEED_POINTS.into_iter().collect(),
        }
    }
}

impl DutyRpmTable {
    /// Interpolated RPM at `duty`, flat-clamped to the table's lowest/
    /// highest known duty. `f64::NAN` only if the table is empty (cannot
    /// happen via `Default`; `refine` never removes entries).
    pub fn rpm_for_duty(&self, duty: u8) -> f64 {
        let Some((&lo_d, &lo_r)) = self.points.iter().next() else {
            return f64::NAN;
        };
        let (&hi_d, &hi_r) = self
            .points
            .iter()
            .next_back()
            .expect("checked non-empty above");
        if duty <= lo_d {
            return lo_r;
        }
        if duty >= hi_d {
            return hi_r;
        }
        match self.points.get(&duty) {
            Some(&r) => r,
            None => {
                // duty is strictly inside the known range but not itself a
                // key: bracket it between the nearest lower and upper keys
                // (both guaranteed to exist by the clamp checks above).
                let (&d0, &r0) = self.points.range(..duty).next_back().expect("below hi_d");
                let (&d1, &r1) = self.points.range(duty..).next().expect("above lo_d");
                let frac = f64::from(duty - d0) / f64::from(d1 - d0);
                r0 + frac * (r1 - r0)
            }
        }
    }

    /// The table's own duty entries (its "treads"), nearest to `target_rpm`
    /// by that entry's stored RPM value; ties go to the **lower** (quieter)
    /// duty. Design doc §2.3.
    pub fn duty_for_rpm(&self, target_rpm: f64) -> u8 {
        let mut best: Option<(u8, f64)> = None; // (duty, |diff|)
        for (&d, &r) in &self.points {
            let diff = (r - target_rpm).abs();
            best = match best {
                None => Some((d, diff)),
                // Strictly-less only: the map iterates duty ascending, so
                // keeping the first (lowest-duty) entry on a tie already
                // prefers lower without an explicit `<=`.
                Some((_, best_diff)) if diff < best_diff => Some((d, diff)),
                Some(prev) => Some(prev),
            };
        }
        best.expect("DutyRpmTable is never empty (Default seeds it, refine never removes)")
            .0
    }

    /// Passive refinement (design doc §2.3): blends the entry for `duty`
    /// toward `mean_rpm` (`rpm <- 0.8*rpm + 0.2*mean`, entry created from
    /// the current interpolated value if absent), rejects outright when
    /// `mean_rpm` is more than 25 % off the current value, and clamps the
    /// result so the table stays **strictly** increasing in duty. Callers
    /// (the controller) are responsible for the steady-window gating
    /// described in §2.3 — this method only enforces the two invariants
    /// above, unconditionally, on whatever `(duty, mean_rpm)` it is given.
    pub fn refine(&mut self, duty: u8, mean_rpm: f64) {
        let current = self
            .points
            .get(&duty)
            .copied()
            .unwrap_or_else(|| self.rpm_for_duty(duty));
        if current > 0.0 && (mean_rpm - current).abs() > REFINE_REJECT_FRACTION * current {
            return;
        }
        let mut blended = (1.0 - REFINE_MEAN_WEIGHT) * current + REFINE_MEAN_WEIGHT * mean_rpm;
        let lower_rpm = self.points.range(..duty).next_back().map(|(_, &r)| r);
        let upper_rpm = self
            .points
            .range(duty.saturating_add(1)..)
            .next()
            .map(|(_, &r)| r);
        match (lower_rpm, upper_rpm) {
            (Some(lo), Some(hi)) => {
                // Both neighbours present: `hi > lo` is the invariant this
                // method maintains, but repeated refinements at the flat-
                // clamped extremes (all sharing one interpolated prior) can
                // squeeze existing neighbours to within a few
                // `MONOTONE_MARGIN_RPM` of each other — applying the fixed
                // margin to *both* sides independently can then hand back
                // an interval whose bounds land on the same f64 once
                // rounded, silently losing strict order (caught by the
                // 100-refinement property test below). Scaling the margin
                // to a quarter of the ACTUAL gap keeps both ends strictly
                // inside `(lo, hi)` regardless of how narrow the gap has
                // become, only falling back to the fixed constant when
                // there's room to spare.
                let margin = ((hi - lo) / 4.0).min(MONOTONE_MARGIN_RPM);
                blended = blended.clamp(lo + margin, hi - margin);
            }
            (Some(lo), None) => blended = blended.max(lo + MONOTONE_MARGIN_RPM),
            (None, Some(hi)) => blended = blended.min(hi - MONOTONE_MARGIN_RPM),
            (None, None) => {}
        }
        self.points.insert(duty, blended);
    }

    /// True iff every entry's RPM is strictly greater than the previous
    /// entry's, in duty order. Test-only: this is the invariant `refine`
    /// maintains, checked here rather than duplicated at each call site.
    #[cfg(test)]
    fn is_strictly_increasing(&self) -> bool {
        self.points.values().is_sorted_by(|a, b| a < b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Step 6: Default + serde default ---------------------------------

    #[test]
    fn default_is_exactly_the_ten_seeded_points() {
        let t = DutyRpmTable::default();
        let mut got: Vec<(u8, f64)> = t.points.iter().map(|(&d, &r)| (d, r)).collect();
        got.sort_by_key(|&(d, _)| d);
        let mut want = SEED_POINTS.to_vec();
        want.sort_by_key(|&(d, _)| d);
        assert_eq!(got, want);
        assert_eq!(got.len(), 10);
    }

    #[derive(serde::Serialize, serde::Deserialize)]
    struct HostWithDefault {
        #[serde(default)]
        duty_rpm_table: DutyRpmTable,
    }

    #[test]
    fn json_missing_the_field_deserialises_to_default() {
        // This is the exact shape Task 13's PersistedState relies on: a
        // legacy state.json predating this field must load the seed.
        let host: HostWithDefault = serde_json::from_str("{}").unwrap();
        assert_eq!(host.duty_rpm_table, DutyRpmTable::default());
    }

    #[test]
    fn serde_round_trip_preserves_refined_values() {
        let mut t = DutyRpmTable::default();
        t.refine(30, 2600.0);
        let json = serde_json::to_string(&t).unwrap();
        let back: DutyRpmTable = serde_json::from_str(&json).unwrap();
        assert_eq!(back, t);
    }

    // --- Step 7: interpolation, snap ties ---------------------------------

    #[test]
    fn rpm_for_duty_exact_and_interpolated() {
        let t = DutyRpmTable::default();
        assert_eq!(t.rpm_for_duty(30), 2560.0);
        // Halfway between 30->2560 and 36->3030: 2795.0.
        assert_eq!(t.rpm_for_duty(33), 2795.0);
    }

    #[test]
    fn rpm_for_duty_clamps_outside_known_range() {
        let t = DutyRpmTable::default();
        assert_eq!(t.rpm_for_duty(0), 1195.0); // below lowest key (15)
        assert_eq!(t.rpm_for_duty(100), 5920.0); // above highest key (85)
    }

    #[test]
    fn duty_for_rpm_picks_nearest_entry() {
        let t = DutyRpmTable::default();
        // 2560 is an exact entry (duty 30).
        assert_eq!(t.duty_for_rpm(2560.0), 30);
        // 2400 is closer to 2300 (duty 27, |100|) than 2560 (duty 30, |160|).
        assert_eq!(t.duty_for_rpm(2400.0), 27);
    }

    #[test]
    fn duty_for_rpm_ties_go_down() {
        let t = DutyRpmTable::default();
        // Exact midpoint of 2300 (duty 27) and 2560 (duty 30): 2430.
        assert_eq!(t.duty_for_rpm(2430.0), 27);
    }

    // --- Step 8: refine ----------------------------------------------------

    #[test]
    fn refine_blends_toward_the_observed_mean() {
        let mut t = DutyRpmTable::default();
        // duty 30's seed is 2560.0; mean 2600 is 1.56 % off, well inside the
        // 25 % band. 0.8*2560 + 0.2*2600 = 2568.0.
        t.refine(30, 2600.0);
        assert_eq!(t.rpm_for_duty(30), 2568.0);
    }

    #[test]
    fn refine_creates_an_entry_when_absent_from_the_interpolated_prior() {
        let mut t = DutyRpmTable::default();
        // duty 33 has no entry; its prior is the interpolated 2795.0 (see
        // rpm_for_duty_exact_and_interpolated above). Mean 2800 is well
        // inside 25 % of that: 0.8*2795 + 0.2*2800 = 2796.0.
        t.refine(33, 2800.0);
        assert_eq!(t.rpm_for_duty(33), 2796.0);
    }

    #[test]
    fn refine_rejects_a_jump_over_25_percent() {
        let mut t = DutyRpmTable::default();
        let before = t.rpm_for_duty(30);
        // 2560 * 1.30 = 3328, a 30 % jump: rejected outright, entry
        // untouched.
        t.refine(30, 3328.0);
        assert_eq!(t.rpm_for_duty(30), before);
    }

    #[test]
    fn refine_accepts_exactly_at_the_25_percent_edge() {
        let mut t = DutyRpmTable::default();
        // 2560 * 1.25 = 3200.0, exactly 25 %: the design says "rejected
        // when > 25 %", so exactly-25 % is accepted (a boundary the
        // implementation must get right rather than round away).
        t.refine(30, 3200.0);
        // 0.8*2560 + 0.2*3200 = 2688.0.
        assert_eq!(t.rpm_for_duty(30), 2688.0);
    }

    #[test]
    fn refine_that_would_invert_two_adjacent_duties_is_clamped() {
        // Seeded duty 48 (3950.0) and duty 52 (4180.0) are only 5.8 % apart
        // — closer than the ~5 % a single refine can move a value even at
        // the 25 % acceptance edge (0.8*c + 0.2*1.25c = 1.05c), so it takes
        // two upward refinements of 48, each riding the boundary, to reach
        // the point where the unclamped blend would push past 52's 4180.
        let mut t = DutyRpmTable::default();
        t.refine(48, 3950.0 * 1.25); // accepted (exactly 25% over 3950) -> 4147.5
        assert_eq!(t.rpm_for_duty(48), 4147.5);

        t.refine(48, 4147.5 * 1.25); // accepted (exactly 25% over the new current)
        // Unclamped this would blend to 0.8*4147.5 + 0.2*(4147.5*1.25) =
        // 4354.875, past duty 52's 4180.0 — inverted. The clamp instead
        // pins it just under 4180.0.
        let d48 = t.rpm_for_duty(48);
        assert!(
            d48 < 4180.0,
            "duty 48 ({d48}) must stay below duty 52's 4180.0"
        );
        assert!(
            (d48 - (4180.0 - MONOTONE_MARGIN_RPM)).abs() < 1e-9,
            "got {d48}"
        );
        assert!(t.is_strictly_increasing());
    }

    // --- Step 9: property test, 100 noisy refinements ----------------------

    /// Small deterministic xorshift32 PRNG — the crate adds no new
    /// dependency for this (Global Constraints: "All simulation and plant
    /// tests use a seeded, hand-rolled RNG").
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

    #[test]
    fn stays_strictly_increasing_after_100_noisy_refinements() {
        let mut t = DutyRpmTable::default();
        let mut rng = Xorshift32(0xC0FF_EE01);
        for _ in 0..100 {
            // Duty is a percentage: 0..=100, the type's real operating
            // domain (the u8 field is wider only because Rust has no u7).
            // Both existing entries and brand-new interpolated-prior
            // creations get exercised since the range spans well past the
            // ten seeded keys.
            let duty = (rng.next_u32() % 101) as u8;
            let current = t.rpm_for_duty(duty);
            // Noise within the acceptance band, both directions, including
            // right at the 25 % edge occasionally.
            let pct = rng.next_f64(-0.25, 0.25);
            let mean = current * (1.0 + pct);
            t.refine(duty, mean);
            assert!(
                t.is_strictly_increasing(),
                "table not strictly increasing after refining duty {duty} toward {mean}: {:?}",
                t.points
            );
        }
    }
}

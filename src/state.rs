//! Calibration state persistence (JSON, `/var/lib/bazerame-fans/state.json`
//! in production, `--state-file` overridable). Holds the GPU clock→watts
//! LUT, the fitted loop PI gains, the duty↔RPM table and the warm-start
//! budget seeds so a reboot skips recalibration and resumes near its last
//! working point. Loading NEVER crashes: missing or corrupt state just means
//! "not calibrated yet". Cargo.toml enables serde_json's `float_roundtrip`
//! so this file's f64 fields (LUT watts, gains, warm-start budgets) survive
//! save→load bit-exact.
//!
//! Schema v2 (`fw-fanctrl-loop-dsh`): the old `model` field and its two
//! Kalman-correction scalars (the learned thermal model's bias and gain
//! terms) are gone along with the adaptation tier that used them
//! (`fw-fanctrl-loop-24s`). A v1 file's `model` and bias/gain correction
//! keys are simply unknown fields to this schema and are ignored by serde;
//! `duty_rpm_table`, `loop_gains` and `warm_start` are missing from a v1 file
//! and come back at their `#[serde(default)]` values (the ten seeded points,
//! `None` and empty respectively) via `PersistedState`'s own `Default`.

use std::collections::BTreeMap;
use std::path::Path;

use crate::config::write_atomic;
use crate::control::budget::LoopGains;
use crate::control::lut::ClockWattsLut;
use crate::fanctrl::table::DutyRpmTable;

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PersistedState {
    /// GPU clock→watts LUT from the calibration sweep.
    pub lut: Option<ClockWattsLut>,
    /// When calibration finished, as a unix-seconds string.
    pub calibrated_at: Option<String>,
    /// Fitted PI gains for both loop legs (design §2.4), from the FOPDT
    /// step-test fit. `None` until a step test has landed; `Budget::new`
    /// falls back to `LoopGains::default()` in that case.
    pub loop_gains: Option<LoopGains>,
    /// Duty↔RPM lookup (design §2.3), passively refined by the controller.
    /// Its own `Default`/serde default is the ten seeded points, so a
    /// legacy file predating this field loads the seed unchanged.
    pub duty_rpm_table: DutyRpmTable,
    /// Warm-start budget seeds, keyed by `WarmStart::key` (design §2.4).
    /// Empty on a legacy or fresh file.
    pub warm_start: BTreeMap<String, f64>,
}

impl PersistedState {
    /// Load from `path`. Missing file → default (info log); unreadable or
    /// corrupt JSON → default + warning. NEVER crashes on bad state.
    pub fn load(path: &Path) -> PersistedState {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                tracing::info!("no state at {}, starting uncalibrated", path.display());
                return PersistedState::default();
            }
            Err(e) => {
                tracing::warn!(
                    "cannot read state {}, starting uncalibrated: {e}",
                    path.display()
                );
                return PersistedState::default();
            }
        };
        match serde_json::from_str::<PersistedState>(&text) {
            Ok(state) => state.validated(),
            Err(e) => {
                tracing::warn!(
                    "corrupt state {}, starting uncalibrated: {e}",
                    path.display()
                );
                PersistedState::default()
            }
        }
    }

    /// Enforce, on the way in from disk, the invariants the in-process
    /// constructors hold by construction. Serde checks the JSON *shape*, not
    /// the values, and a schema-skewed or hand-edited `state.json` otherwise
    /// panics the control loop several layers away from here:
    ///
    /// - an empty `duty_rpm_table` panics `DutyRpmTable::duty_for_rpm`'s
    ///   `best.expect(...)`, and a non-monotone one panics `refine`'s
    ///   `f64::clamp(lo + margin, hi - margin)` with `min > max`;
    /// - `loop_gains` with a zero/negative/non-finite integral time makes
    ///   `Budget::step` divide by zero and drives the commanded budget
    ///   permanently to NaN.
    ///
    /// Each invalid field is dropped back to its "not calibrated" value with
    /// a warning — never a panic, per this module's "Loading NEVER crashes"
    /// contract. Fields are validated independently: a bad table does not
    /// throw away good gains.
    fn validated(mut self) -> PersistedState {
        if !self.duty_rpm_table.is_valid() {
            tracing::warn!(
                "state duty_rpm_table is empty, non-finite or not strictly \
                 increasing; falling back to the seeded table"
            );
            self.duty_rpm_table = DutyRpmTable::default();
        }
        if let Some(gains) = self.loop_gains
            && !gains.is_valid()
        {
            tracing::warn!(
                "state loop_gains {gains:?} are not all finite and positive; \
                 falling back to the default gains (uncalibrated)"
            );
            self.loop_gains = None;
        }
        self
    }

    /// Atomic save: write `<path>.tmp`, then rename over `path`. Creates the
    /// parent directory if needed.
    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        let text = serde_json::to_string_pretty(self).map_err(std::io::Error::other)?;
        write_atomic(path, text.as_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-state-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn lut3() -> ClockWattsLut {
        let mut lut = ClockWattsLut::new();
        lut.insert(1200, 30.0);
        lut.insert(2000, 60.0);
        lut.insert(2800, 100.0);
        lut
    }

    fn gains() -> LoopGains {
        LoopGains {
            kc_w_per_c: 0.31,
            ti_s: 42.0,
            kc_w_per_rpm: 0.0041,
            ti_rpm_s: 28.0,
        }
    }

    #[test]
    fn legacy_v1_file_loads_lut_intact_table_seeded_warm_start_empty_gains_none() {
        // tests/fixtures/state_v1.json (fw-fanctrl-loop-blm) is a real pre-
        // migration file: it carries the old `model` field and its bias/gain
        // correction scalars, none of which this schema has any more.
        let path = crate::test_support::fixtures::path("state_v1.json");
        let state = PersistedState::load(&path);

        let lut = state.lut.as_ref().expect("lut persisted in the fixture");
        assert_eq!(lut.len(), 10, "fixture's LUT has 10 swept points");
        // Spot-check both ends of the sweep survived unknown-key tolerant
        // deserialization intact (not just "some value").
        assert_eq!(lut.watts_for_clock(1177), Some(39.36073333333333));
        assert_eq!(lut.watts_for_clock(2618), Some(99.89833333333333));

        assert_eq!(
            state.duty_rpm_table,
            DutyRpmTable::default(),
            "a v1 file predates duty_rpm_table: it must come back as the ten seeded points"
        );
        assert!(
            state.warm_start.is_empty(),
            "a v1 file predates warm_start: it must come back empty"
        );
        assert_eq!(
            state.loop_gains, None,
            "a v1 file predates loop_gains: it must come back None"
        );
        assert_eq!(state.calibrated_at, Some("1783230048".to_string()));
    }

    #[test]
    fn new_schema_round_trips_populated_warm_start_and_gains() {
        let dir = fixture_dir("roundtrip-v2");
        let path = dir.join("state.json");
        let mut table = DutyRpmTable::default();
        table.refine(30, 2600.0); // differs from the untouched seed
        let mut warm_start = BTreeMap::new();
        warm_start.insert("quiet16|30|ac".to_string(), 45.5);
        warm_start.insert("cool16|20|bat".to_string(), 12.0);
        let state = PersistedState {
            lut: Some(lut3()),
            calibrated_at: Some("1751500000".to_string()),
            loop_gains: Some(gains()),
            duty_rpm_table: table,
            warm_start,
        };

        state.save(&path).unwrap();
        let back = PersistedState::load(&path);

        assert_eq!(back, state);
        // Behavioral equivalence for the LUT, not just field equality.
        for w in [10.0, 45.0, 60.0, 120.0] {
            assert_eq!(
                back.lut.as_ref().unwrap().clock_for_watts(w),
                state.lut.as_ref().unwrap().clock_for_watts(w)
            );
        }
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn missing_file_gives_default() {
        let dir = fixture_dir("missing");
        let state = PersistedState::load(&dir.join("nope.json"));
        // The whole-struct equality is the assertion; per-field checks after
        // it cannot fail independently of it, so none are repeated here.
        assert_eq!(state, PersistedState::default());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn corrupt_file_gives_default_no_panic() {
        let dir = fixture_dir("corrupt");
        let path = dir.join("state.json");
        fs::write(&path, "{ not json").unwrap();
        assert_eq!(PersistedState::load(&path), PersistedState::default());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn save_over_existing_is_atomic_and_leaves_no_tmp() {
        let dir = fixture_dir("atomic");
        let path = dir.join("state.json");
        PersistedState::default().save(&path).unwrap();
        let updated = PersistedState {
            lut: Some(lut3()),
            ..PersistedState::default()
        };
        updated.save(&path).unwrap();
        assert_eq!(PersistedState::load(&path), updated);
        let names: Vec<String> = fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        assert_eq!(names, vec!["state.json".to_string()]);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn save_creates_parent_dir() {
        let dir = fixture_dir("parents");
        let path = dir.join("var/lib/state.json");
        PersistedState::default().save(&path).unwrap();
        assert_eq!(PersistedState::load(&path), PersistedState::default());
        fs::remove_dir_all(&dir).unwrap();
    }

    // --- finding 7: load-time invariant validation ------------------------
    //
    // Serde only checks the JSON shape. Each case below is well-formed JSON
    // that deserialises fine and then panics (or NaNs) the control loop
    // several layers away; `load` must drop the offending field to its
    // "not calibrated" value instead. Every case round-trips through a real
    // file, i.e. through the production `load` path.

    /// Write `json` to a fresh fixture file, load it, and remove the dir.
    fn load_json(name: &str, json: &str) -> PersistedState {
        let dir = fixture_dir(name);
        let path = dir.join("state.json");
        fs::write(&path, json).unwrap();
        let state = PersistedState::load(&path);
        fs::remove_dir_all(&dir).unwrap();
        state
    }

    #[test]
    fn empty_duty_rpm_table_falls_back_to_the_seed() {
        // `duty_for_rpm` on an empty table hits `best.expect(...)`.
        let state = load_json("empty-table", r#"{ "duty_rpm_table": { "points": {} } }"#);
        assert_eq!(state.duty_rpm_table, DutyRpmTable::default());
        // The seeded table answers instead of panicking.
        assert!(state.duty_rpm_table.duty_for_rpm(2300.0) > 0);
    }

    #[test]
    fn non_monotone_duty_rpm_table_falls_back_to_the_seed() {
        // RPM falling with duty makes `refine`'s clamp(lo + m, hi - m) panic
        // with min > max on any refinement between the two entries.
        let state = load_json(
            "non-monotone-table",
            r#"{ "duty_rpm_table": { "points": { "15": 5000.0, "40": 1200.0, "85": 5920.0 } } }"#,
        );
        assert_eq!(state.duty_rpm_table, DutyRpmTable::default());
        let mut table = state.duty_rpm_table.clone();
        table.refine(30, 2560.0); // would panic on the persisted shape
    }

    #[test]
    fn non_finite_duty_rpm_table_entry_falls_back_to_the_seed() {
        // serde_json accepts no bare `NaN`, but a value large enough to
        // overflow f64 parses as `inf` — non-finite all the same.
        let state = load_json(
            "inf-table",
            r#"{ "duty_rpm_table": { "points": { "15": 1195.0, "85": 1e400 } } }"#,
        );
        assert_eq!(state.duty_rpm_table, DutyRpmTable::default());
        assert!(state.duty_rpm_table.rpm_for_duty(85).is_finite());
    }

    #[test]
    fn a_valid_persisted_table_is_kept_verbatim() {
        // The guard must not eat good calibration data.
        let state = load_json(
            "valid-table",
            r#"{ "duty_rpm_table": { "points": { "15": 1200.0, "40": 3400.0, "85": 5900.0 } } }"#,
        );
        assert_ne!(state.duty_rpm_table, DutyRpmTable::default());
        assert_eq!(state.duty_rpm_table.rpm_for_duty(15), 1200.0);
    }

    /// A `loop_gains` JSON object with the four fields set as given.
    fn gains_json(kc_c: &str, ti_s: &str, kc_rpm: &str, ti_rpm: &str) -> String {
        format!(
            r#"{{ "loop_gains": {{ "kc_w_per_c": {kc_c}, "ti_s": {ti_s},
                 "kc_w_per_rpm": {kc_rpm}, "ti_rpm_s": {ti_rpm} }} }}"#
        )
    }

    #[test]
    fn zero_integral_time_drops_the_persisted_gains() {
        // ti_s == 0 => `Budget::step`'s `kc * PI_PERIOD_S / ti` is inf and
        // `u` goes permanently NaN.
        for json in [
            gains_json("0.31", "0.0", "0.0041", "28.0"),
            gains_json("0.31", "42.0", "0.0041", "0.0"),
        ] {
            let state = load_json("zero-ti", &json);
            assert_eq!(state.loop_gains, None, "zero integral time must be dropped");
        }
    }

    #[test]
    fn negative_or_non_finite_gains_are_dropped() {
        for json in [
            gains_json("0.31", "-42.0", "0.0041", "28.0"),
            gains_json("-0.31", "42.0", "0.0041", "28.0"),
            gains_json("0.31", "1e400", "0.0041", "28.0"),
            gains_json("0.31", "42.0", "0.0", "28.0"),
        ] {
            let state = load_json("bad-gains", &json);
            assert_eq!(state.loop_gains, None, "rejected: {json}");
        }
    }

    #[test]
    fn valid_gains_survive_the_check_and_a_bad_table_does_not_take_them_down() {
        let json = r#"{ "loop_gains": { "kc_w_per_c": 0.31, "ti_s": 42.0,
                        "kc_w_per_rpm": 0.0041, "ti_rpm_s": 28.0 },
                        "duty_rpm_table": { "points": {} } }"#;
        let state = load_json("gains-kept", json);
        assert_eq!(state.loop_gains, Some(gains()));
        assert_eq!(state.duty_rpm_table, DutyRpmTable::default());
    }
}

//! Calibration state persistence (JSON, `/var/lib/bazerame-fans/state.json`
//! in production, `--state-file` overridable). Holds the GPU clock→watts
//! LUT, the fitted loop PI gains, the duty↔RPM table and the warm-start
//! budget seeds so a reboot skips recalibration and resumes near its last
//! working point. Loading NEVER crashes: missing or corrupt state just means
//! "not calibrated yet". Cargo.toml enables serde_json's `float_roundtrip`
//! so this file's f64 fields (LUT watts, gains, warm-start budgets) survive
//! save→load bit-exact.
//!
//! Schema v2 (`fw-fanctrl-loop-dsh`): the old `model`/`adapt_bias`/
//! `adapt_gain` fields (the learned thermal model + its Kalman correction)
//! are gone along with the adaptation tier that used them
//! (`fw-fanctrl-loop-24s`). A v1 file's `model`/`adapt_bias`/`adapt_gain`
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
        match serde_json::from_str(&text) {
            Ok(state) => state,
            Err(e) => {
                tracing::warn!(
                    "corrupt state {}, starting uncalibrated: {e}",
                    path.display()
                );
                PersistedState::default()
            }
        }
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
        // migration file: it carries `model`, `adapt_bias` and `adapt_gain`,
        // none of which this schema has any more.
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
        assert_eq!(state, PersistedState::default());
        assert!(state.lut.is_none());
        assert_eq!(state.duty_rpm_table, DutyRpmTable::default());
        assert!(state.warm_start.is_empty());
        assert_eq!(state.loop_gains, None);
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
}

//! Calibration state persistence (JSON, `/var/lib/bazerame-fans/state.json`
//! in production, `--state-file` overridable). Holds the fitted thermal model
//! and the GPU clock→watts LUT so a reboot skips recalibration. Loading NEVER
//! crashes: missing or corrupt state just means "not calibrated yet".
//! Cargo.toml enables serde_json's `float_roundtrip` so the model's f64
//! parameters survive save→load bit-exact.

use std::path::Path;

use crate::config::write_atomic;
use crate::control::lut::ClockWattsLut;
use crate::control::thermal_model::ThermalModel;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PersistedState {
    /// Fitted thermal model; None until calibration has run.
    pub model: Option<ThermalModel>,
    /// GPU clock→watts LUT from the calibration sweep.
    pub lut: Option<ClockWattsLut>,
    /// When calibration finished, as a unix-seconds string.
    pub calibrated_at: Option<String>,
    /// Persisted Kalman bias (RPM offset added to the model's `c`); the
    /// covariance is deliberately NOT persisted (fresh prior each session,
    /// mirroring `ThermalModel`'s serde-skipped `P`). Defaults to the
    /// identity correction so a pre-v2 or fresh state file adapts from zero.
    #[serde(default)]
    pub adapt_bias: f64,
    /// Persisted Kalman gain (multiplier on the model's GPU-slope term).
    /// Defaults to 1.0 (identity). Reset to `[0, 1]` whenever a new
    /// calibration lands (a fresh surface invalidates old corrections).
    #[serde(default = "default_gain")]
    pub adapt_gain: f64,
}

fn default_gain() -> f64 {
    1.0
}

impl Default for PersistedState {
    fn default() -> Self {
        Self {
            model: None,
            lut: None,
            calibrated_at: None,
            adapt_bias: 0.0,
            adapt_gain: 1.0,
        }
    }
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
    use crate::control::thermal_model::CalibPoint;
    use std::fs;
    use std::path::PathBuf;

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-state-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// A real model fit from 5 synthetic points on rpm = 25pc + 15pg
    /// + 0.1·pc·pg + 800 (2×2 grid + center: full-rank for the bilinear fit).
    fn fitted_model() -> ThermalModel {
        let points: Vec<CalibPoint> = [
            (5.0, 0.0),
            (45.0, 0.0),
            (5.0, 100.0),
            (45.0, 100.0),
            (20.0, 40.0),
        ]
        .iter()
        .map(|&(pc, pg)| CalibPoint {
            cpu_w: pc,
            gpu_w: pg,
            rpm: 25.0 * pc + 15.0 * pg + 0.1 * pc * pg + 800.0,
        })
        .collect();
        ThermalModel::fit_batch(&points).unwrap()
    }

    fn lut3() -> ClockWattsLut {
        let mut lut = ClockWattsLut::new();
        lut.insert(1200, 30.0);
        lut.insert(2000, 60.0);
        lut.insert(2800, 100.0);
        lut
    }

    #[test]
    fn roundtrip_model_and_lut_behave_identically_after_reload() {
        let dir = fixture_dir("roundtrip");
        let path = dir.join("state.json");
        let state = PersistedState {
            model: Some(fitted_model()),
            lut: Some(lut3()),
            calibrated_at: Some("1751500000".to_string()),
            ..PersistedState::default()
        };
        state.save(&path).unwrap();
        let back = PersistedState::load(&path);
        assert_eq!(back, state);
        // Behavioral equivalence, not just field equality: the reloaded model
        // predicts the same RPM and the reloaded LUT inverts the same clocks.
        let (model, back_model) = (state.model.unwrap(), back.model.unwrap());
        for (pc, pg) in [(10.0, 20.0), (30.0, 65.0), (45.0, 100.0)] {
            assert_eq!(back_model.predict(pc, pg), model.predict(pc, pg));
        }
        let (lut, back_lut) = (state.lut.unwrap(), back.lut.unwrap());
        for w in [10.0, 45.0, 60.0, 120.0] {
            assert_eq!(back_lut.clock_for_watts(w), lut.clock_for_watts(w));
        }
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn missing_file_gives_default() {
        let dir = fixture_dir("missing");
        let state = PersistedState::load(&dir.join("nope.json"));
        assert_eq!(state, PersistedState::default());
        assert!(state.model.is_none());
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
    fn adapt_state_roundtrips_and_defaults_to_identity() {
        // A pre-v2 state file has no adapt fields: serde(default) must load them
        // as the identity correction (bias 0, gain 1), never panic.
        let legacy = r#"{ "model": null, "lut": null, "calibrated_at": null }"#;
        let s: PersistedState = serde_json::from_str(legacy).unwrap();
        assert_eq!(s.adapt_bias, 0.0);
        assert_eq!(s.adapt_gain, 1.0);

        // And a written pair survives the roundtrip.
        let dir = fixture_dir("adapt");
        let path = dir.join("state.json");
        let saved = PersistedState {
            adapt_bias: -137.0,
            adapt_gain: 1.15,
            ..PersistedState::default()
        };
        saved.save(&path).unwrap();
        assert_eq!(PersistedState::load(&path), saved);
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

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
use crate::control::device_loop::Gains;
use crate::control::lut::ClockWattsLut;
use crate::fanctrl::table::DutyRpmTable;

#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PersistedState {
    /// Temporary old-controller compatibility only. It is never serialized
    /// and legacy on-disk values are discarded at the load boundary.
    #[serde(skip, default)]
    pub lut: Option<ClockWattsLut>,
    /// When calibration finished, as a unix-seconds string.
    pub calibrated_at: Option<String>,
    /// Temporary old-controller compatibility only. It is never serialized.
    #[serde(skip, default)]
    pub loop_gains: Option<LoopGains>,
    /// Fitted per-device gains keyed by `<strategy>:<ma_interval>`.
    pub cpu_gains: BTreeMap<String, Gains>,
    /// Fitted per-device gains keyed by `<strategy>:<ma_interval>`.
    pub gpu_gains: BTreeMap<String, Gains>,
    /// Duty↔RPM lookup (design §2.3), passively refined by the controller.
    /// Its own `Default`/serde default is the ten seeded points, so a
    /// legacy file predating this field loads the seed unchanged.
    pub duty_rpm_table: DutyRpmTable,
    /// Paired per-device warm-start caps, keyed by `WarmStart::key`.
    pub warm_start: BTreeMap<String, WarmStartEntry>,
    /// Last qualified setpoint for Held-mode startup.
    pub t_star_last_good: Option<TStarSeed>,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct WarmStartEntry {
    pub cpu_cap_w: f64,
    pub gpu_lock_mhz: u32,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TStarSeed {
    pub strategy: String,
    pub fan_target_rpm: u32,
    pub value_c: f64,
    pub saved_at_unix_s: u64,
}

const FAN_TARGET_MIN_RPM: u32 = 1000;
const FAN_TARGET_MAX_RPM: u32 = 7000;
const T_STAR_MAX_C: f64 = 110.0;
const T_STAR_MAX_AGE_S: u64 = 6 * 60 * 60;

/// The v3 fields are deliberately decoded from their own wire values. A
/// `null` emitted for a non-finite float, or one malformed map entry, must
/// not make serde discard unrelated calibration state at the struct level.
#[derive(Default)]
struct DecodedDeviceLoopState {
    cpu_gains: BTreeMap<String, Gains>,
    gpu_gains: BTreeMap<String, Gains>,
    warm_start: BTreeMap<String, WarmStartEntry>,
    t_star_last_good: Option<TStarSeed>,
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
        let mut value = match serde_json::from_str::<serde_json::Value>(&text) {
            Ok(value) => value,
            Err(e) => {
                tracing::warn!(
                    "corrupt state {}, starting uncalibrated: {e}",
                    path.display()
                );
                return PersistedState::default();
            }
        };
        Self::drop_legacy_fields(&mut value);
        let device_loop = Self::take_device_loop_fields(&mut value);
        match serde_json::from_value::<PersistedState>(value) {
            Ok(mut state) => {
                state.cpu_gains = device_loop.cpu_gains;
                state.gpu_gains = device_loop.gpu_gains;
                state.warm_start = device_loop.warm_start;
                state.t_star_last_good = device_loop.t_star_last_good;
                state.validated()
            }
            Err(e) => {
                tracing::warn!(
                    "corrupt state {}, starting uncalibrated: {e}",
                    path.display()
                );
                PersistedState::default()
            }
        }
    }

    /// Take v3 records out of the general persisted-state wire object and
    /// decode their entries independently. The remaining legacy-compatible
    /// fields can then use ordinary `PersistedState` deserialization without
    /// a malformed v3 child making the entire file look corrupt.
    fn take_device_loop_fields(value: &mut serde_json::Value) -> DecodedDeviceLoopState {
        let Some(object) = value.as_object_mut() else {
            return DecodedDeviceLoopState::default();
        };
        DecodedDeviceLoopState {
            cpu_gains: Self::decode_gain_map(object.remove("cpu_gains"), "cpu_gains"),
            gpu_gains: Self::decode_gain_map(object.remove("gpu_gains"), "gpu_gains"),
            warm_start: Self::decode_warm_start_map(object.remove("warm_start")),
            t_star_last_good: Self::decode_tstar_seed(object.remove("t_star_last_good")),
        }
    }

    fn decode_gain_map(
        value: Option<serde_json::Value>,
        field: &str,
    ) -> BTreeMap<String, Gains> {
        let Some(value) = value else {
            return BTreeMap::new();
        };
        let Some(entries) = value.as_object() else {
            tracing::warn!("state {field} is not a map; dropping it");
            return BTreeMap::new();
        };
        entries
            .iter()
            .filter_map(|(key, value)| match serde_json::from_value::<Gains>(value.clone()) {
                Ok(gains) => Some((key.clone(), gains)),
                Err(e) => {
                    tracing::warn!("state {field} has malformed entry for {key:?}; dropping it: {e}");
                    None
                }
            })
            .collect()
    }

    fn decode_warm_start_map(value: Option<serde_json::Value>) -> BTreeMap<String, WarmStartEntry> {
        let Some(value) = value else {
            return BTreeMap::new();
        };
        let Some(entries) = value.as_object() else {
            tracing::warn!("state warm_start is not a map; dropping it");
            return BTreeMap::new();
        };
        entries
            .iter()
            .filter_map(|(key, value)| match serde_json::from_value::<WarmStartEntry>(value.clone()) {
                Ok(entry) => Some((key.clone(), entry)),
                Err(e) => {
                    tracing::warn!("state warm_start has malformed paired entry for {key:?}; dropping it: {e}");
                    None
                }
            })
            .collect()
    }

    fn decode_tstar_seed(value: Option<serde_json::Value>) -> Option<TStarSeed> {
        let value = value?;
        if value.is_null() {
            return None;
        }
        match serde_json::from_value::<TStarSeed>(value) {
            Ok(seed) => Some(seed),
            Err(e) => {
                tracing::warn!("state t_star_last_good is malformed; dropping it: {e}");
                None
            }
        }
    }

    /// Remove superseded on-disk representations before deserializing the
    /// new schema. One warning per category keeps normal upgrades useful
    /// without a warning per map entry.
    fn drop_legacy_fields(value: &mut serde_json::Value) {
        let Some(object) = value.as_object_mut() else { return };
        if object.remove("lut").is_some() {
            tracing::warn!("state migration: ignoring legacy lut");
        }
        if object.remove("loop_gains").is_some() {
            tracing::warn!("state migration: ignoring legacy loop_gains");
        }
        if object
            .get("t_star_last_good")
            .is_some_and(|seed| !seed.is_object() && !seed.is_null())
        {
            object.remove("t_star_last_good");
            tracing::warn!("state migration: ignoring legacy scalar t_star_last_good");
        }
        let mut dropped_bare_warm_start = false;
        if let Some(warm_start) = object.get_mut("warm_start").and_then(serde_json::Value::as_object_mut) {
            warm_start.retain(|_, entry| {
                let paired = entry.is_object();
                dropped_bare_warm_start |= !paired;
                paired
            });
        }
        if dropped_bare_warm_start {
            tracing::warn!("state migration: ignoring legacy scalar warm_start entries");
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
    ///   permanently to NaN;
    /// - a `lut` with a non-finite watts entry feeds `Budget::set_bounds`'s
    ///   `f64::clamp` a non-finite bound, which panics, and one whose clocks
    ///   are out of order or duplicated silently mis-answers both lookups;
    /// - a non-finite `warm_start` value seeds `u`/`v` to NaN, which the
    ///   velocity-form update never recovers from.
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
        if let Some(lut) = &self.lut
            && !lut.is_valid()
        {
            tracing::warn!(
                "state lut ({} points) is empty, has a non-finite or negative \
                 watts entry, or its clocks are not strictly increasing; \
                 discarding it (uncalibrated, recalibration required)",
                lut.len()
            );
            self.lut = None;
        }
        self.cpu_gains.retain(|key, gains| {
            let valid = !key.is_empty() && gains.is_valid();
            if !valid {
                tracing::warn!("state cpu_gains has invalid entry for {key:?}; dropping it");
            }
            valid
        });
        self.gpu_gains.retain(|key, gains| {
            let valid = !key.is_empty() && gains.is_valid();
            if !valid {
                tracing::warn!("state gpu_gains has invalid entry for {key:?}; dropping it");
            }
            valid
        });
        self.warm_start.retain(|key, entry| {
            let valid = !key.is_empty() && entry.cpu_cap_w.is_finite() && entry.cpu_cap_w >= 0.0;
            if !valid {
                tracing::warn!("state warm_start has invalid paired entry for {key:?}; dropping it");
            }
            valid
        });
        let invalid_tstar_seed = self.t_star_last_good.as_ref().is_some_and(|seed| {
            self.qualified_seed(
                &seed.strategy,
                seed.fan_target_rpm,
                seed.saved_at_unix_s,
                0.0,
                T_STAR_MAX_C,
            )
            .is_none()
        });
        if invalid_tstar_seed {
            tracing::warn!("state t_star_last_good is invalid; dropping it");
            self.t_star_last_good = None;
        }
        self
    }

    /// Returns a persisted T* only if it is still applicable to this
    /// strategy/target and timestamp. Bounds are current runtime feasibility
    /// limits, so a valid saved value is always clamped at use time.
    pub fn qualified_seed(
        &self,
        strategy: &str,
        fan_target_rpm: u32,
        now_unix_s: u64,
        floor_c: f64,
        ceiling_c: f64,
    ) -> Option<f64> {
        let seed = self.t_star_last_good.as_ref()?;
        if !seed.is_valid()
            || seed.strategy != strategy
            || seed.fan_target_rpm != fan_target_rpm
            || now_unix_s
                .checked_sub(seed.saved_at_unix_s)
                .is_none_or(|age| age > T_STAR_MAX_AGE_S)
            || !floor_c.is_finite()
            || !ceiling_c.is_finite()
            || floor_c > ceiling_c
        {
            return None;
        }
        Some(seed.value_c.clamp(floor_c, ceiling_c))
    }

    /// Atomic save: write `<path>.tmp`, then rename over `path`. Creates the
    /// parent directory if needed.
    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        let text = serde_json::to_string_pretty(self).map_err(std::io::Error::other)?;
        write_atomic(path, text.as_bytes())
    }
}

impl TStarSeed {
    fn is_valid(&self) -> bool {
        !self.strategy.is_empty()
            && (FAN_TARGET_MIN_RPM..=FAN_TARGET_MAX_RPM).contains(&self.fan_target_rpm)
            && self.value_c.is_finite()
            && self.value_c > 0.0
            && self.value_c <= T_STAR_MAX_C
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;
    use std::sync::{Arc, Mutex, OnceLock};
    use std::thread::ThreadId;
    use tracing::field::{Field, Visit};
    use tracing::{Event, Subscriber};
    use tracing_subscriber::layer::{Context, Layer};
    use tracing_subscriber::prelude::*;

    type CapturedEvents = Vec<(ThreadId, String)>;
    type LogBuffer = Arc<Mutex<CapturedEvents>>;

    #[derive(Clone)]
    struct CapturedLogLayer(LogBuffer);

    struct MessageVisitor(String);

    impl Visit for MessageVisitor {
        fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
            if field.name() == "message" {
                self.0 = format!("{value:?}");
            }
        }
    }

    impl<S: Subscriber> Layer<S> for CapturedLogLayer {
        fn on_event(&self, event: &Event<'_>, _context: Context<'_, S>) {
            let mut visitor = MessageVisitor(String::new());
            event.record(&mut visitor);
            self.0
                .lock()
                .unwrap()
                .push((std::thread::current().id(), visitor.0));
        }
    }

    static CAPTURED_LOGS: OnceLock<LogBuffer> = OnceLock::new();

    fn captured_logs() -> LogBuffer {
        CAPTURED_LOGS
            .get_or_init(|| {
                let logs = Arc::new(Mutex::new(Vec::new()));
                tracing::subscriber::set_global_default(
                    tracing_subscriber::registry().with(CapturedLogLayer(Arc::clone(&logs))),
                )
                .expect("state tests install the only global tracing subscriber");
                logs
            })
            .clone()
    }

    fn capture_logs(run: impl FnOnce()) -> String {
        let buffer = captured_logs();
        let thread = std::thread::current().id();
        let start = buffer.lock().unwrap().len();
        run();
        buffer.lock().unwrap()[start..]
            .iter()
            .filter(|(event_thread, _)| *event_thread == thread)
            .map(|(_, message)| message.as_str())
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-state-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn legacy_v1_file_ignores_lut_and_seeds_new_fields() {
        // tests/fixtures/state_v1.json (fw-fanctrl-loop-blm) is a real pre-
        // migration file: it carries the old `model` field and its bias/gain
        // correction scalars, none of which this schema has any more.
        let path = crate::test_support::fixtures::path("state_v1.json");
        let state = PersistedState::load(&path);

        assert_eq!(state.lut, None, "the legacy LUT is intentionally ignored");

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
        warm_start.insert(
            "quiet16|30|ac".to_string(),
            WarmStartEntry { cpu_cap_w: 45.5, gpu_lock_mhz: 1800 },
        );
        warm_start.insert(
            "cool16|20|bat".to_string(),
            WarmStartEntry { cpu_cap_w: 12.0, gpu_lock_mhz: 1200 },
        );
        let state = PersistedState {
            calibrated_at: Some("1751500000".to_string()),
            cpu_gains: BTreeMap::from([("quiet16:60".into(), Gains { kc: 0.2, ti_s: 35.0 })]),
            gpu_gains: BTreeMap::from([("quiet16:60".into(), Gains { kc: 2.1, ti_s: 15.0 })]),
            duty_rpm_table: table,
            warm_start,
            t_star_last_good: Some(TStarSeed {
                strategy: "quiet16".into(),
                fan_target_rpm: 3000,
                value_c: 70.0,
                saved_at_unix_s: 100,
            }),
            ..PersistedState::default()
        };

        state.save(&path).unwrap();
        let back = PersistedState::load(&path);

        assert_eq!(back, state);
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
            cpu_gains: BTreeMap::from([("quiet16:60".into(), Gains { kc: 0.2, ti_s: 35.0 })]),
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
    fn legacy_gains_are_ignored_and_a_bad_table_falls_back_independently() {
        let json = r#"{ "loop_gains": { "kc_w_per_c": 0.31, "ti_s": 42.0,
                        "kc_w_per_rpm": 0.0041, "ti_rpm_s": 28.0 },
                        "duty_rpm_table": { "points": {} } }"#;
        let state = load_json("gains-kept", json);
        assert_eq!(state.loop_gains, None);
        assert_eq!(state.duty_rpm_table, DutyRpmTable::default());
    }

    // --- finding 4: the other two fields (lut, warm_start) ----------------
    //
    // On non-finite values reaching `load`: they cannot, on this crate's
    // serde_json configuration. `float_roundtrip` (Cargo.toml:17) swaps in
    // the lossless float parser, which returns `Error("number out of
    // range")` for `1e400`/`1e309`/`1.8e308` rather than saturating to
    // `inf`, and `to_string` writes a non-finite f64 as `null`, which
    // deserializes back as a type error — so an infinity can neither be
    // written by `save` nor read by `load`; both routes land in the
    // "corrupt state, starting uncalibrated" branch instead. The finiteness
    // half of each guard is therefore defense-in-depth against a future
    // parser/feature change, and is pinned where it can actually be
    // exercised — `ClockWattsLut::is_valid` and `WarmStart::drop_non_finite`
    // unit tests, which build the bad values in Rust. What a hand edit CAN
    // produce is well-formed JSON with the values in the wrong order; that
    // is what the load-path tests below drive.

    #[test]
    fn out_of_order_lut_clocks_discard_the_lut() {
        // `insert` keeps points sorted and unique by mhz; `watts_for_clock`
        // `binary_search_by_key`s on that and `clock_for_watts` scans
        // `windows(2)` assuming ascending clocks, so a hand-edited
        // descending pair silently mis-answers both — and a wrong watts
        // answer is what sets `Budget`'s bounds.
        let state = load_json(
            "unsorted-lut",
            r#"{ "lut": { "points": [[2800, 100.0], [1200, 30.0]] } }"#,
        );
        assert_eq!(state.lut, None, "out-of-order clocks discard the LUT");
    }

    #[test]
    fn duplicate_or_empty_lut_points_discard_the_lut() {
        let dup = load_json(
            "dup-lut",
            r#"{ "lut": { "points": [[1200, 30.0], [1200, 40.0]] } }"#,
        );
        assert_eq!(dup.lut, None, "duplicate clocks discard the LUT");

        // An empty points array is not "a calibration with no points", it is
        // "not calibrated" — every lookup on it returns None anyway.
        let empty = load_json("empty-lut", r#"{ "lut": { "points": [] } }"#);
        assert_eq!(empty.lut, None, "an empty LUT is not a calibration");
    }

    #[test]
    fn a_negative_lut_watts_entry_discards_the_lut() {
        // Negative watts are physically impossible from the sweep and would
        // drag `set_bounds`' `lo` below `cpu_floor_w`.
        let state = load_json(
            "negative-lut",
            r#"{ "lut": { "points": [[1200, 30.0], [2000, -60.0]] } }"#,
        );
        assert_eq!(state.lut, None, "a negative watts entry discards the LUT");
    }

    #[test]
    fn legacy_lut_and_scalar_warm_start_are_ignored_independently() {
        let state = load_json(
            "lut-kept",
            r#"{ "lut": { "points": [[1200, 30.0], [2000, 60.0], [2800, 100.0]] },
                 "warm_start": { "quiet16:30:ac": 45.5, "cool16:20:batt": 12.0 },
                 "duty_rpm_table": { "points": {} } }"#,
        );
        assert_eq!(state.lut, None);
        assert!(state.warm_start.is_empty());
        assert_eq!(state.duty_rpm_table, DutyRpmTable::default());
    }

    #[test]
    fn legacy_lut_is_ignored_regardless_of_its_shape() {
        let state = load_json(
            "dip-lut",
            r#"{ "lut": { "points": [[1200, 60.0], [2000, 50.0], [2800, 100.0]] } }"#,
        );
        assert_eq!(state.lut, None);
    }

    #[test]
    fn paired_warm_starts_and_keyed_gains_round_trip() {
        let dir = fixture_dir("device-loop-v3");
        let path = dir.join("state.json");
        let state = PersistedState {
            cpu_gains: BTreeMap::from([("quiet16:60".into(), Gains { kc: 0.2, ti_s: 35.0 })]),
            gpu_gains: BTreeMap::from([("quiet16:60".into(), Gains { kc: 2.1, ti_s: 15.0 })]),
            warm_start: BTreeMap::from([(
                "quiet16:36:ac".into(),
                WarmStartEntry { cpu_cap_w: 42.0, gpu_lock_mhz: 1800 },
            )]),
            t_star_last_good: Some(TStarSeed {
                strategy: "quiet16".into(),
                fan_target_rpm: 3000,
                value_c: 70.0,
                saved_at_unix_s: 100,
            }),
            ..PersistedState::default()
        };
        state.save(&path).unwrap();
        assert_eq!(PersistedState::load(&path), state);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn malformed_new_records_drop_individually_and_keep_valid_siblings() {
        let state = load_json(
            "isolated-new-record-decode",
            r#"{
                "calibrated_at": "1751500000",
                "duty_rpm_table": { "points": { "15": 1200.0, "40": 3400.0, "85": 5900.0 } },
                "cpu_gains": {
                    "cpu-good": { "kc": 0.2, "ti_s": 35.0 },
                    "cpu-null": null,
                    "cpu-non-finite-shaped": { "kc": "NaN", "ti_s": 35.0 },
                    "cpu-invalid": { "kc": 0.0, "ti_s": 35.0 }
                },
                "gpu_gains": {
                    "gpu-good": { "kc": 2.1, "ti_s": 15.0 },
                    "gpu-null": null,
                    "gpu-non-finite-shaped": { "kc": "NaN", "ti_s": 15.0 },
                    "gpu-invalid": { "kc": 2.1, "ti_s": 0.0 }
                },
                "warm_start": {
                    "warm-good": { "cpu_cap_w": 42.0, "gpu_lock_mhz": 1800 },
                    "warm-null": null,
                    "warm-non-finite-shaped": { "cpu_cap_w": "NaN", "gpu_lock_mhz": 1800 },
                    "warm-invalid": { "cpu_cap_w": -1.0, "gpu_lock_mhz": 1800 }
                },
                "t_star_last_good": {
                    "strategy": "quiet16", "fan_target_rpm": 3000,
                    "value_c": null, "saved_at_unix_s": 100
                }
            }"#,
        );

        assert_eq!(state.calibrated_at.as_deref(), Some("1751500000"));
        assert_eq!(state.duty_rpm_table.rpm_for_duty(40), 3400.0);
        assert_eq!(
            state.cpu_gains,
            BTreeMap::from([("cpu-good".into(), Gains { kc: 0.2, ti_s: 35.0 })])
        );
        assert_eq!(
            state.gpu_gains,
            BTreeMap::from([("gpu-good".into(), Gains { kc: 2.1, ti_s: 15.0 })])
        );
        assert_eq!(
            state.warm_start,
            BTreeMap::from([(
                "warm-good".into(),
                WarmStartEntry { cpu_cap_w: 42.0, gpu_lock_mhz: 1800 }
            )])
        );
        assert_eq!(state.t_star_last_good, None);
    }

    #[test]
    fn null_or_non_finite_shaped_tstar_drops_only_the_seed() {
        for value_c in ["null", r#""NaN""#] {
            let state = load_json(
                "bad-tstar-wire-value",
                &format!(
                    r#"{{
                        "calibrated_at": "1751500000",
                        "cpu_gains": {{ "cpu-good": {{ "kc": 0.2, "ti_s": 35.0 }} }},
                        "t_star_last_good": {{
                            "strategy": "quiet16", "fan_target_rpm": 3000,
                            "value_c": {value_c}, "saved_at_unix_s": 100
                        }}
                    }}"#
                ),
            );
            assert_eq!(state.calibrated_at.as_deref(), Some("1751500000"));
            assert_eq!(state.cpu_gains.len(), 1);
            assert_eq!(state.t_star_last_good, None, "rejected: {value_c}");
        }
    }

    #[test]
    fn non_finite_tstar_value_is_dropped() {
        let state = PersistedState {
            t_star_last_good: Some(TStarSeed {
                strategy: "quiet16".into(),
                fan_target_rpm: 3000,
                value_c: f64::NAN,
                saved_at_unix_s: 100,
            }),
            ..PersistedState::default()
        }
        .validated();
        assert_eq!(state.t_star_last_good, None);
    }

    #[test]
    fn qualified_tstar_seed_enforces_key_age_and_bounds() {
        let seed = TStarSeed {
            strategy: "quiet16".into(),
            fan_target_rpm: 3000,
            value_c: 98.0,
            saved_at_unix_s: 100,
        };
        let state = PersistedState { t_star_last_good: Some(seed), ..PersistedState::default() };
        assert_eq!(state.qualified_seed("quiet16", 3000, 21_700, 60.0, 80.0), Some(80.0));
        assert_eq!(state.qualified_seed("cool16", 3000, 101, 0.0, 100.0), None);
        assert_eq!(state.qualified_seed("quiet16", 3001, 101, 0.0, 100.0), None);
        assert_eq!(state.qualified_seed("quiet16", 3000, 21_701, 0.0, 100.0), None);
        assert_eq!(state.qualified_seed("quiet16", 3000, 99, 0.0, 100.0), None);
    }

    #[test]
    fn mixed_legacy_state_keeps_new_fields_and_drops_each_legacy_shape() {
        let state = load_json(
            "mixed-legacy-v3",
            r#"{
                "lut": { "points": [[1200, 30.0]] },
                "loop_gains": { "kc_w_per_c": 0.2, "ti_s": 30, "kc_w_per_rpm": 0.01, "ti_rpm_s": 30 },
                "warm_start": {
                    "old": 45.0,
                    "new": { "cpu_cap_w": 42.0, "gpu_lock_mhz": 1800 }
                },
                "t_star_last_good": 70.0,
                "cpu_gains": { "quiet16:60": { "kc": 0.2, "ti_s": 35.0 } },
                "gpu_gains": { "quiet16:60": { "kc": 2.1, "ti_s": 15.0 } }
            }"#,
        );
        assert_eq!(state.lut, None);
        assert_eq!(state.loop_gains, None);
        assert_eq!(state.warm_start.len(), 1);
        assert_eq!(
            state.warm_start["new"],
            WarmStartEntry { cpu_cap_w: 42.0, gpu_lock_mhz: 1800 }
        );
        assert_eq!(state.t_star_last_good, None);
        assert_eq!(state.cpu_gains.len(), 1);
        assert_eq!(state.gpu_gains.len(), 1);
    }

    #[test]
    fn legacy_migration_warns_once_per_category() {
        let mut loaded = None;
        let logs = capture_logs(|| {
            loaded = Some(load_json(
                "migration-warnings",
                r#"{
                    "lut": { "points": [] },
                    "loop_gains": { "kc_w_per_c": 0.2 },
                    "warm_start": { "old-a": 40.0, "old-b": 45.0 },
                    "t_star_last_good": 70.0
                }"#,
            ));
        });
        let state = loaded.expect("load ran");
        assert_eq!(state, PersistedState::default());
        for warning in [
            "state migration: ignoring legacy lut",
            "state migration: ignoring legacy loop_gains",
            "state migration: ignoring legacy scalar warm_start entries",
            "state migration: ignoring legacy scalar t_star_last_good",
        ] {
            assert_eq!(
                logs.matches(warning).count(),
                1,
                "expected exactly one {warning:?} warning: {logs}"
            );
        }
    }

    #[test]
    fn invalid_qualified_seed_is_dropped_and_unrelated_save_keeps_its_timestamp() {
        for seed in [
            r#"{ "strategy": "", "fan_target_rpm": 3000, "value_c": 70.0, "saved_at_unix_s": 100 }"#,
            r#"{ "strategy": "quiet16", "fan_target_rpm": 999, "value_c": 70.0, "saved_at_unix_s": 100 }"#,
            r#"{ "strategy": "quiet16", "fan_target_rpm": 3000, "value_c": -1.0, "saved_at_unix_s": 100 }"#,
        ] {
            let state = load_json("bad-tstar", &format!(r#"{{ "t_star_last_good": {seed} }}"#));
            assert_eq!(state.t_star_last_good, None);
        }

        let dir = fixture_dir("seed-timestamp-preserved");
        let path = dir.join("state.json");
        let state = PersistedState {
            duty_rpm_table: DutyRpmTable::default(),
            t_star_last_good: Some(TStarSeed {
                strategy: "quiet16".into(),
                fan_target_rpm: 3000,
                value_c: 70.0,
                saved_at_unix_s: 100,
            }),
            ..PersistedState::default()
        };
        state.save(&path).unwrap();
        let mut unrelated = PersistedState::load(&path);
        unrelated.duty_rpm_table.refine(30, 2600.0);
        unrelated.save(&path).unwrap();
        assert_eq!(
            PersistedState::load(&path)
                .t_star_last_good
                .expect("seed retained")
                .saved_at_unix_s,
            100
        );
        fs::remove_dir_all(&dir).unwrap();
    }
}

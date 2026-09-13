//! Append-only JSONL telemetry log for offline controller-quality review:
//! every 1 Hz [`Sample`] and every controller decision
//! becomes one JSON line loadable into pandas/DuckDB.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::types::{Sample, TelemetryDevice, TelemetryFlag, TelemetryTStarState};

/// Bumped whenever the line format changes; stamped into the run_start line.
/// 3 (per-device loops): sample lines add independently nullable
/// `cpu_group_c`/`gpu_group_c`; decision lines add typed T* state,
/// per-device candidates/holds/gain sources, and labelled/polarity flags.
// 4 adds per-sample calibration diagnostics; existing record shapes are unchanged.
// 5 adds fresh CPU cap read-back timestamps and reset evidence to calibration context.
const SCHEMA_VERSION: u32 = 5;
/// Flush at least once every this many records...
const FLUSH_EVERY_RECORDS: u32 = 10;
/// ...and no less often than this, so a quiet log still hits disk.
const FLUSH_MAX_INTERVAL: Duration = Duration::from_secs(5);
/// How many `run-<secs>[-N].jsonl` names to try before giving up.
const MAX_NAME_ATTEMPTS: u32 = 10;

/// One JSONL line. Internally tagged so each line carries a `kind` field.
// Records are built one at a time on the stack and passed by reference
// straight into `log()` — never stored or collected — so the Decision
// variant's size (it grew a tail of Option fields) costs nothing; boxing
// would only complicate every construction site.
#[allow(clippy::large_enum_variant)]
#[derive(serde::Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Record<'a> {
    /// One evaluated calibration tick, including the tick ending the run.
    Calibration {
        t_mono: f64,
        #[serde(flatten)]
        diagnostics: &'a crate::calib::step::CalibrationDiagnostics,
        gpu_util_pct: f64,
        view_fresh: bool,
        view_changed: bool,
        reconciliation_ma_c: Option<f64>,
        socket_ma_c: Option<f64>,
    },
    /// One 1 Hz sensor snapshot. `sample`'s own fields flatten straight
    /// onto this line (its `ec`/`fanctrl`/`fanctrl_freshness` fields stay
    /// `#[serde(skip)]`ped on `Sample` itself — `Instant` isn't
    /// serializable and the raw structs aren't the wire shape design §3.5
    /// wants); the seven fields below re-surface exactly the columns §3.5
    /// asks for, flattened out of `sample.ec`/`sample.fanctrl`/
    /// `sample.nvme_temp_c`. Build with [`Record::sample`] rather than the
    /// struct literal directly.
    Sample {
        #[serde(flatten)]
        sample: &'a Sample,
        /// `sample.ec`'s replica max reading (design §2.2), °C.
        ec_max: Option<i32>,
        /// `sample.ec`'s argmax sensor label.
        ec_argmax: Option<&'a str>,
        /// `sample.ec`'s independently averaged CPU control group, °C.
        cpu_group_c: Option<f64>,
        /// `sample.ec`'s independently averaged GPU control group, °C.
        gpu_group_c: Option<f64>,
        /// The controller's live EC boxcar moving average (design §2.6).
        /// Stateful and owned by the controller, not `Sample` — the
        /// caller supplies it (see [`Record::sample`]).
        ec_ma: Option<f64>,
        /// `sample.nvme_temp_c`, under the design §3.5 column name.
        nvme_c: Option<f64>,
        /// `sample.fanctrl`'s reported fan duty, percent.
        fanctrl_speed: Option<u8>,
        /// `sample.fanctrl`'s `active` flag (design §2.5: `false` means
        /// the EC's own curve, not fw-fanctrl, is driving the fans).
        fanctrl_active: Option<bool>,
        /// `sample.fanctrl`'s resolved strategy name.
        strategy: Option<&'a str>,
    },
    /// One status-flag transition (any [`crate::control::controller::StatusFlag`],
    /// by its `as_str()` name): emitted by the controller shell alongside
    /// the Decision record, so offline analysis gets a greppable per-flag
    /// stream (Decision lines carry the full flag list, not the transition).
    Flag {
        t_mono: f64,
        flag: String,
        active: bool,
    },
    /// One controller decision: emitted by the controller thread whenever a
    /// status change or reassert happens, with a short `cause` string
    /// ("command:set_cpu_w", "reassert", "stickiness", "resume", "release",
    /// "auto:device_loops", ...).
    Decision {
        t_mono: f64,
        mode: String,
        cpu_limit_w: Option<f64>,
        gpu_max_mhz: Option<u32>,
        fan_target_rpm: f64,
        cause: String,
        /// Structured v3 flags preserve label, device/bound and polarity.
        flags: Vec<TelemetryFlag>,
        /// Shared source target temperature, °C;
        /// mirrors `ControlStatus.t_star_c` while the source has a target.
        #[serde(skip_serializing_if = "Option::is_none")]
        t_star: Option<f64>,
        /// State of the shared T* source; `None` outside Auto.
        tstar_state: Option<TelemetryTStarState>,
        /// CPU DeviceLoop decision; `None` outside Auto.
        cpu: Option<TelemetryDevice>,
        /// GPU DeviceLoop decision; see [`Self::Decision::cpu`].
        gpu: Option<TelemetryDevice>,
    },
}

impl<'a> Record<'a> {
    /// Builds a `sample` telemetry line from a sensor snapshot plus the
    /// controller's live EC boxcar average (design §3.5's `ec_ma` column).
    /// `ec_ma` is threaded in by the caller rather than read off `sample`
    /// because `EcAverage` is stateful and owned by the controller (design
    /// §2.6, `sensors::ec` module docs) — a single `Sample` never carries
    /// it. Every other new column derives straight from `sample` itself.
    pub fn sample(sample: &'a Sample, ec_ma: Option<f64>) -> Self {
        Record::Sample {
            sample,
            ec_max: sample.ec.as_ref().map(|e| e.max_c),
            ec_argmax: sample.ec.as_ref().map(|e| e.argmax.as_str()),
            cpu_group_c: sample.ec.as_ref().and_then(|e| e.cpu_group_c),
            gpu_group_c: sample.ec.as_ref().and_then(|e| e.gpu_group_c),
            ec_ma,
            nvme_c: sample.nvme_temp_c,
            fanctrl_speed: sample.fanctrl.as_ref().map(|v| v.speed_pct),
            fanctrl_active: sample.fanctrl.as_ref().map(|v| v.active),
            strategy: sample.fanctrl.as_ref().map(|v| v.strategy.as_str()),
        }
    }
}

/// First line of every file: anchors the monotonic axis to wall clock and
/// stamps the schema version for offline loaders.
#[derive(serde::Serialize)]
struct RunStart {
    kind: &'static str,
    t_mono: f64,
    t_wall: f64,
    schema_version: u32,
}

/// What actually hits the file: the caller's record plus a top-level
/// wall-clock stamp (`t_wall`, unix seconds), so every line is analyzable
/// on the wall-clock axis without joining against run_start.
#[derive(serde::Serialize)]
struct Stamped<'a> {
    #[serde(flatten)]
    record: &'a Record<'a>,
    t_wall: f64,
}

/// Unix seconds now, as f64 (0.0 if the clock is before the epoch).
fn wall_unix_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// Buffered JSONL writer with a bounded-staleness flush policy.
pub struct Telemetry {
    writer: BufWriter<File>,
    path: PathBuf,
    /// Records written since the last flush (flush every 10).
    records_since_flush: u32,
    /// When the last flush happened (flush if > 5 s ago).
    last_flush: Instant,
    /// Write errors warn once, then stay silent (writes are still attempted).
    warned: bool,
}

impl Telemetry {
    /// Opens `<dir>/run-<unix_secs>.jsonl`, creating `dir` as needed. Never
    /// truncates an existing file: on a name collision (restart within the
    /// same second) it retries with `-1`, `-2`, ... suffixes, and writes a
    /// `run_start` header line into the fresh file.
    pub fn open(dir: &Path) -> io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let wall = wall_unix_secs();
        let unix_secs = wall as u64;
        for attempt in 0..MAX_NAME_ATTEMPTS {
            let name = if attempt == 0 {
                format!("run-{unix_secs}.jsonl")
            } else {
                format!("run-{unix_secs}-{attempt}.jsonl")
            };
            let path = dir.join(name);
            match File::options().write(true).create_new(true).open(&path) {
                Ok(file) => {
                    let mut t = Self::from_parts(file, path);
                    t.write_line(&RunStart {
                        kind: "run_start",
                        // Telemetry opens at process start -- the same moment
                        // the sampler pins its t_mono epoch -- so pairing 0.0
                        // with t_wall anchors the monotonic axis to wall
                        // clock within startup milliseconds. Per-record
                        // t_wall makes any residual skew a non-issue.
                        t_mono: 0.0,
                        t_wall: wall,
                        schema_version: SCHEMA_VERSION,
                    });
                    return Ok(t);
                }
                Err(e) if e.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(e) => return Err(e),
            }
        }
        Err(io::Error::new(
            io::ErrorKind::AlreadyExists,
            format!("no free run-{unix_secs}*.jsonl name after {MAX_NAME_ATTEMPTS} attempts"),
        ))
    }

    /// Wraps an already-open file (also the seam for write-failure tests).
    fn from_parts(file: File, path: PathBuf) -> Self {
        Self {
            writer: BufWriter::new(file),
            path,
            records_since_flush: 0,
            last_flush: Instant::now(),
            warned: false,
        }
    }

    /// Where this log lives (for startup log messages).
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Serializes one record as a JSON line, stamped with the current wall
    /// clock (`t_wall`). Flushes every 10 records or if more than 5 s passed
    /// since the last flush. Errors warn once, then stay silent: later
    /// records are still attempted, so they are dropped while the sink is
    /// broken and resume if it recovers. Never panics.
    pub fn log(&mut self, r: &Record) {
        self.write_line(&Stamped {
            record: r,
            t_wall: wall_unix_secs(),
        });
        self.records_since_flush += 1;
        if self.records_since_flush >= FLUSH_EVERY_RECORDS
            || self.last_flush.elapsed() > FLUSH_MAX_INTERVAL
        {
            self.flush();
        }
    }

    /// Serializes to one buffer, then writes it with a single `write_all`:
    /// a serialize-side failure can never leave a partial line behind.
    fn write_line<T: serde::Serialize>(&mut self, value: &T) {
        let mut line = match serde_json::to_vec(value) {
            Ok(line) => line,
            Err(e) => {
                self.warn_once(&io::Error::from(e));
                return;
            }
        };
        line.push(b'\n');
        if let Err(e) = self.writer.write_all(&line) {
            self.warn_once(&e);
        }
    }

    /// Explicit flush for shutdown paths.
    pub fn flush(&mut self) {
        if let Err(e) = self.writer.flush() {
            self.warn_once(&e);
        }
        self.records_since_flush = 0;
        self.last_flush = Instant::now();
    }

    fn warn_once(&mut self, e: &io::Error) {
        if !self.warned {
            tracing::warn!(
                path = %self.path.display(),
                "telemetry write failed (suppressing further warnings): {e}"
            );
            self.warned = true;
        }
    }
}

/// Poison-tolerant lock for the telemetry sink shared by main (samples) and
/// the controller thread (decisions): if the other thread panicked while
/// holding the lock, keep logging instead of cascading the panic. Callers
/// keep lock scopes one-call tiny.
///
/// Delegates to [`crate::sync_util::lock`], the single copy of this rule --
/// the sensor path adopted it in roast PR-1 finding 3.
pub fn lock(
    shared: &std::sync::Mutex<Option<Telemetry>>,
) -> std::sync::MutexGuard<'_, Option<Telemetry>> {
    crate::sync_util::lock(shared)
}

/// Opens under `preferred_dir`, falling back to `fallback_dir` if that fails
/// (production passes "."); `None` only if both fail (telemetry disabled).
pub fn open_with_fallback(preferred_dir: &Path, fallback_dir: &Path) -> Option<Telemetry> {
    match Telemetry::open(preferred_dir) {
        Ok(t) => Some(t),
        Err(e) => {
            tracing::warn!(
                "cannot open telemetry log under {}: {e}; falling back to {}",
                preferred_dir.display(),
                fallback_dir.display()
            );
            match Telemetry::open(fallback_dir) {
                Ok(t) => Some(t),
                Err(e) => {
                    tracing::warn!("telemetry fallback failed too, telemetry disabled: {e}");
                    None
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "fw-fan-quiet-telemetry-test-{}-{name}",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn sample_at(t_mono: f64) -> Sample {
        Sample {
            t_mono,
            ..Sample::default()
        }
    }

    #[test]
    fn roundtrip_records() {
        let dir = fixture_dir("roundtrip");
        let mut t = Telemetry::open(&dir).unwrap();
        t.log(&Record::sample(&sample_at(1.0), None));
        t.log(&Record::sample(&sample_at(2.0), None));
        t.log(&Record::Flag {
            t_mono: 3.0,
            flag: "resumed".into(),
            active: true,
        });
        t.log(&Record::Decision {
            t_mono: 4.0,
            mode: "manual".into(),
            cpu_limit_w: Some(20.0),
            gpu_max_mhz: None,
            fan_target_rpm: 3000.0,
            cause: "command:set_cpu_w".into(),
            flags: vec![TelemetryFlag::legacy("resumed")],
            t_star: None,
            tstar_state: None,
            cpu: None,
            gpu: None,
        });
        t.flush();

        let contents = fs::read_to_string(t.path()).unwrap();
        let lines: Vec<&str> = contents.lines().collect();
        assert_eq!(
            lines.len(),
            5,
            "run_start header + 4 records, got: {contents:?}"
        );

        let start: serde_json::Value = serde_json::from_str(lines[0]).unwrap();
        assert_eq!(start["kind"], "run_start");
        assert!(start["t_mono"].is_number());
        assert!(
            start["t_wall"].as_f64().unwrap() > 1.5e9,
            "t_wall must be real unix seconds"
        );
        assert_eq!(start["schema_version"], 5);

        let first: serde_json::Value = serde_json::from_str(lines[1]).unwrap();
        assert_eq!(first["kind"], "sample");
        assert_eq!(first["t_mono"], 1.0);
        assert!(
            first["t_wall"].as_f64().unwrap() > 1.5e9,
            "sample lines carry a top-level wall-clock stamp"
        );
        let second: serde_json::Value = serde_json::from_str(lines[2]).unwrap();
        assert_eq!(second["t_mono"], 2.0);
        let flag: serde_json::Value = serde_json::from_str(lines[3]).unwrap();
        assert_eq!(flag["kind"], "flag");
        assert_eq!(flag["flag"], "resumed");
        assert_eq!(flag["active"], true);
        assert!(
            flag["t_wall"].as_f64().unwrap() > 1.5e9,
            "flag lines carry a top-level wall-clock stamp"
        );
        let decision: serde_json::Value = serde_json::from_str(lines[4]).unwrap();
        assert_eq!(decision["kind"], "decision");
        assert_eq!(decision["t_mono"], 4.0);
        assert_eq!(decision["mode"], "manual");
        assert_eq!(decision["cpu_limit_w"], 20.0);
        assert_eq!(decision["gpu_max_mhz"], serde_json::Value::Null);
        assert_eq!(decision["fan_target_rpm"], 3000.0);
        assert_eq!(decision["cause"], "command:set_cpu_w");
        assert_eq!(decision["flags"][0]["name"], "legacy");
        assert_eq!(decision["flags"][0]["flag"], "resumed");
        assert_eq!(decision["flags"][0]["active"], true);
        // None auto fields are skipped entirely: non-auto lines stay lean.
        for key in [
            "demand_cpu",
            "demand_gpu",
            "alloc_cpu_w",
            "alloc_gpu_w",
            "pi_target_w",
            "t_star",
            "freeze",
        ] {
            assert!(
                decision.get(key).is_none(),
                "{key} must be skipped when None: {decision}"
            );
        }
        assert!(decision.get("budget_w").is_none());
        assert!(
            decision["t_wall"].as_f64().unwrap() > 1.5e9,
            "decision lines carry a top-level wall-clock stamp"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    /// Design §3.5: the `sample` line's sensor-group columns, each
    /// derived from a populated `Sample` (`ec`, `nvme_temp_c`, `fanctrl`)
    /// plus the caller-supplied `ec_ma`. A value the record could actually
    /// fail to carry (a real EC reading, a real `FanctrlView`) — not a
    /// `Sample::default()`, which would let every field trivially pass at
    /// `null`.
    #[test]
    fn sample_line_carries_the_new_sensor_columns() {
        use crate::fanctrl::client::FanctrlView;
        use crate::test_support::fixtures;
        use std::time::Instant;

        let dir = fixture_dir("sample-new-columns");
        let ec = crate::sensors::ec::EcReading::read(&fixtures::path("hwmon/cros_ec_load"))
            .expect("fixture must yield a reading");
        let expected_max = ec.max_c;
        let expected_argmax = ec.argmax.as_str().to_string();
        let sample = Sample {
            ec: Some(ec),
            nvme_temp_c: Some(63.5),
            fanctrl: Some(FanctrlView {
                strategy: "quiet16".into(),
                active: true,
                speed_pct: 42,
                temperature: 75.0,
                ma_temperature: 74.2,
                ma_interval: 60,
                curve: vec![(0.0, 15), (95.0, 100)],
                observed_at: Instant::now(),
                all_observed_at: Some(Instant::now()),
            }),
            ..Sample::default()
        };

        let mut t = Telemetry::open(&dir).unwrap();
        t.log(&Record::sample(&sample, Some(70.8)));
        t.flush();

        let contents = fs::read_to_string(t.path()).unwrap();
        let line: serde_json::Value = serde_json::from_str(contents.lines().nth(1).unwrap())
            .expect("line 1 is the sample record");
        assert_eq!(line["ec_max"], expected_max);
        assert_eq!(line["ec_argmax"], expected_argmax);
        assert_eq!(line["ec_ma"], 70.8);
        assert_eq!(line["nvme_c"], 63.5);
        assert_eq!(line["fanctrl_speed"], 42);
        assert_eq!(line["fanctrl_active"], true);
        assert_eq!(line["strategy"], "quiet16");
        // The raw sub-structs stay off the wire; only the flattened
        // columns above surface them.
        assert!(line.get("ec").is_none());
        assert!(line.get("fanctrl").is_none());

        fs::remove_dir_all(&dir).unwrap();
    }

    /// A `Sample::default()` has no EC/fanctrl/NVMe reading at all: every
    /// new column must come back `null`, not panic or fabricate a value.
    #[test]
    fn sample_line_new_columns_are_null_without_a_reading() {
        let dir = fixture_dir("sample-new-columns-absent");
        let mut t = Telemetry::open(&dir).unwrap();
        t.log(&Record::sample(&sample_at(1.0), None));
        t.flush();

        let contents = fs::read_to_string(t.path()).unwrap();
        let line: serde_json::Value = serde_json::from_str(contents.lines().nth(1).unwrap())
            .expect("line 1 is the sample record");
        for key in [
            "ec_max",
            "ec_argmax",
            "ec_ma",
            "nvme_c",
            "fanctrl_speed",
            "fanctrl_active",
            "strategy",
        ] {
            assert!(
                line[key].is_null(),
                "{key} must be null without a reading: {line}"
            );
        }

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn schema_v3_sample_groups_are_independently_nullable() {
        use crate::test_support::fixtures;

        let mut ec = crate::sensors::ec::EcReading::read(&fixtures::path("hwmon/cros_ec_load"))
            .expect("fixture must yield a reading");
        ec.cpu_group_c = Some(50.0);
        ec.gpu_group_c = Some(70.0);
        let both = Sample {
            ec: Some(ec.clone()),
            ..Sample::default()
        };
        let both_wire = serde_json::to_value(Record::sample(&both, None)).unwrap();
        assert_eq!(both_wire["cpu_group_c"], 50.0);
        assert_eq!(both_wire["gpu_group_c"], 70.0);

        ec.gpu_group_c = None;
        let missing_gpu = Sample {
            ec: Some(ec),
            ..Sample::default()
        };
        let missing_gpu_wire = serde_json::to_value(Record::sample(&missing_gpu, None)).unwrap();
        assert_eq!(missing_gpu_wire["cpu_group_c"], 50.0);
        assert!(missing_gpu_wire["gpu_group_c"].is_null());

        let mut gpu_only_ec = missing_gpu.ec.unwrap();
        gpu_only_ec.cpu_group_c = None;
        gpu_only_ec.gpu_group_c = Some(70.0);
        let missing_cpu = Sample {
            ec: Some(gpu_only_ec),
            ..Sample::default()
        };
        let missing_cpu_wire = serde_json::to_value(Record::sample(&missing_cpu, None)).unwrap();
        assert!(missing_cpu_wire["cpu_group_c"].is_null());
        assert_eq!(missing_cpu_wire["gpu_group_c"], 70.0);
    }

    /// The decision line carries every revision-4 field and none of the
    /// retired controller fields. Check raw JSON text, not only the
    /// struct shape (a struct that no longer HAS a removed field always
    /// "lacks" it trivially; this catches a field merely renamed back in,
    /// or a stray value smuggled into `cause`/a flag string).
    #[test]
    fn decision_line_drops_retired_fields_and_carries_revision_four_fields() {
        let dir = fixture_dir("decision-new-fields");
        let mut t = Telemetry::open(&dir).unwrap();
        t.log(&Record::Decision {
            t_mono: 10.0,
            mode: "auto".into(),
            cpu_limit_w: Some(30.0),
            gpu_max_mhz: Some(1950),
            fan_target_rpm: 3000.0,
            cause: "auto:device_loops".into(),
            flags: vec![],
            t_star: Some(71.5),
            tstar_state: Some(TelemetryTStarState::Held),
            cpu: Some(TelemetryDevice {
                group_c: Some(70.0),
                err_c: Some(1.5),
                thermal: 30.0,
                shadow: 31.0,
                cap: 30.0,
                selected: crate::types::TelemetrySelected::Thermal,
                hold: crate::types::TelemetryHold::None,
                gains_source: crate::types::GainsSource::Config,
            }),
            gpu: Some(TelemetryDevice {
                group_c: Some(70.5),
                err_c: Some(1.0),
                thermal: 1950.0,
                shadow: 2000.0,
                cap: 1950.0,
                selected: crate::types::TelemetrySelected::Thermal,
                hold: crate::types::TelemetryHold::None,
                gains_source: crate::types::GainsSource::Config,
            }),
        });
        t.flush();

        let contents = fs::read_to_string(t.path()).unwrap();
        let raw = contents.lines().nth(1).unwrap();
        assert!(!raw.contains("\"gain\""), "raw line: {raw}");
        assert!(!raw.contains("model_a"), "raw line: {raw}");
        assert!(!raw.contains("model_b"), "raw line: {raw}");
        assert!(!raw.contains("model_e"), "raw line: {raw}");
        assert!(!raw.contains("model_c"), "raw line: {raw}");

        let line: serde_json::Value = serde_json::from_str(raw).unwrap();
        assert_eq!(line["t_star"], 71.5);
        for retired in ["budget_w", "freeze", "pi_target_w", "alloc_cpu_w", "alloc_gpu_w", "demand_cpu", "demand_gpu"] {
            assert!(line.get(retired).is_none(), "retired {retired}: {line}");
        }

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn schema_v3_decision_carries_tstar_devices_and_detailed_flags() {
        use crate::control::device_loop::{DeviceDecision, Hold, Selected};
        use crate::types::{GainsSource, TelemetryDevice, TelemetryFlag, TelemetryTStarState};

        let device = |selected, hold, source| {
            TelemetryDevice::from((
                DeviceDecision {
                    t_star: 72.0,
                    group_c: Some(70.0),
                    err_c: Some(2.0),
                    thermal: 2100.0,
                    shadow: 2050.0,
                    cap: 2050.0,
                    selected,
                    hold,
                    write_allowed: true,
                    group_lost: false,
                    write_immediately: false,
                },
                source,
            ))
        };
        let decision = Record::Decision {
            t_mono: 10.0,
            mode: "auto".into(),
            cpu_limit_w: Some(30.0),
            gpu_max_mhz: Some(2050),
            fan_target_rpm: 3000.0,
            cause: "auto:device_loops".into(),
            flags: vec![TelemetryFlag::ArgmaxStuck {
                label: "ambient_f75303@4d".into(),
                active: true,
            }],
            t_star: Some(72.0),
            tstar_state: Some(TelemetryTStarState::Curve),
            cpu: Some(device(Selected::Thermal, Hold::None, GainsSource::Config)),
            gpu: Some(device(Selected::Shadow, Hold::Shadow, GainsSource::Fitted)),
        };

        let wire = serde_json::to_value(&decision).unwrap();
        assert_eq!(wire["tstar_state"], "curve");
        assert_eq!(wire["cpu"]["group_c"], 70.0);
        assert_eq!(wire["cpu"]["selected"], "thermal");
        assert_eq!(wire["gpu"]["hold"]["kind"], "shadow");
        assert_eq!(wire["flags"][0]["name"], "argmax_stuck");
        assert_eq!(wire["flags"][0]["label"], "ambient_f75303@4d");
        assert_eq!(wire["flags"][0]["active"], true);
        for retired in ["budget_w", "freeze", "pi_target_w", "alloc_cpu_w", "alloc_gpu_w", "demand_cpu", "demand_gpu"] {
            assert!(wire.get(retired).is_none(), "retired {retired}: {wire}");
        }
    }

    #[test]
    fn monitor_emission_explicitly_marks_v3_device_state_absent() {
        let decision = Record::Decision {
            t_mono: 1.0,
            mode: "monitor".into(),
            cpu_limit_w: None,
            gpu_max_mhz: None,
            fan_target_rpm: 3000.0,
            cause: "command:release_all".into(),
            flags: Vec::new(),
            t_star: None,
            tstar_state: None,
            cpu: None,
            gpu: None,
        };

        let wire = serde_json::to_value(decision).unwrap();
        for field in ["tstar_state", "cpu", "gpu"] {
            assert!(
                wire.get(field).is_some_and(serde_json::Value::is_null),
                "{field} must be an explicit null outside Auto: {wire}"
            );
        }
    }

    #[test]
    fn open_creates_nested_dir() {
        let root = fixture_dir("nested");
        let nested = root.join("a/b/c");
        let t = Telemetry::open(&nested).unwrap();
        assert!(t.path().starts_with(&nested));
        assert!(t.path().exists());

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn unwritable_dir_falls_back() {
        let root = fixture_dir("fallback");
        // A FILE where a directory is needed: create_dir_all(blocker/sub) fails.
        let blocker = root.join("blocker");
        fs::write(&blocker, "not a dir").unwrap();
        let fallback = root.join("fallback-dir");

        let t =
            open_with_fallback(&blocker.join("sub"), &fallback).expect("fallback should succeed");
        assert!(
            t.path().starts_with(&fallback),
            "fallback file must land under the fallback dir, got {:?}",
            t.path()
        );
        assert!(t.path().exists());

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn reopen_same_second_does_not_truncate() {
        let dir = fixture_dir("no-truncate");
        let mut first = Telemetry::open(&dir).unwrap();
        first.log(&Record::Flag {
            t_mono: 1.0,
            flag: "x".into(),
            active: true,
        });
        first.flush();
        let lines_before = fs::read_to_string(first.path()).unwrap().lines().count();

        // Near-certainly the same unix second: must pick a suffixed name
        // instead of truncating the first run's file.
        let second = Telemetry::open(&dir).unwrap();
        assert_ne!(first.path(), second.path());
        let lines_after = fs::read_to_string(first.path()).unwrap().lines().count();
        assert_eq!(
            lines_before, lines_after,
            "reopening must not clobber the previous run's file"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    /// On Linux writes to an unlinked fd still succeed, so this guards the
    /// no-panic contract, not the error path (see the read-only-file test).
    #[test]
    fn log_survives_unlinked_file() {
        let dir = fixture_dir("file-gone");
        let mut t = Telemetry::open(&dir).unwrap();
        t.log(&Record::sample(&sample_at(0.0), None));
        fs::remove_dir_all(&dir).unwrap();

        // 20 records force flushes past the deleted file; must not panic.
        for i in 1..=20 {
            t.log(&Record::sample(&sample_at(f64::from(i)), None));
        }
        t.flush();
    }

    #[test]
    fn write_failure_warns_once_then_drops_silently() {
        // A read-only File makes every flush fail (buffered writes succeed),
        // exercising the warn-once-then-silent path without panicking.
        let dir = fixture_dir("read-only");
        let path = dir.join("read-only.jsonl");
        fs::write(&path, "").unwrap();
        let file = File::open(&path).unwrap(); // read-only handle

        let mut t = Telemetry::from_parts(file, path);
        for i in 0..20 {
            t.log(&Record::sample(&sample_at(f64::from(i)), None));
        }
        assert!(
            t.warned,
            "flushing to a read-only file must trip the warn-once flag"
        );
        t.flush();

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn flushes_after_ten_records_without_explicit_flush() {
        let dir = fixture_dir("flush-policy");
        let mut t = Telemetry::open(&dir).unwrap();
        for i in 0..10 {
            t.log(&Record::sample(&sample_at(f64::from(i)), None));
        }
        // No explicit flush: the 10-record policy must have flushed already
        // (run_start header + 10 records).
        let contents = fs::read_to_string(t.path()).unwrap();
        assert_eq!(contents.lines().count(), 11);

        fs::remove_dir_all(&dir).unwrap();
    }
}

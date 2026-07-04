//! Append-only JSONL telemetry log for offline controller-quality review:
//! every 1 Hz [`Sample`] (and, from Task 14, every controller decision)
//! becomes one JSON line loadable into pandas/DuckDB.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::types::Sample;

/// Bumped whenever the line format changes; stamped into the run_start line.
const SCHEMA_VERSION: u32 = 1;
/// Flush at least once every this many records...
const FLUSH_EVERY_RECORDS: u32 = 10;
/// ...and no less often than this, so a quiet log still hits disk.
const FLUSH_MAX_INTERVAL: Duration = Duration::from_secs(5);
/// How many `run-<secs>[-N].jsonl` names to try before giving up.
const MAX_NAME_ATTEMPTS: u32 = 10;

/// One JSONL line. Internally tagged so each line carries a `kind` field.
#[derive(serde::Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Record<'a> {
    Sample(&'a Sample),
    // TODO(task-28): per-flag transition records (sensor-lost etc.); flags
    // currently ride along inside Decision lines.
    #[allow(dead_code)]
    Flag {
        t_mono: f64,
        flag: String,
        active: bool,
    },
    /// One controller decision: emitted by the controller thread whenever a
    /// status change or reassert happens, with a short `cause` string
    /// ("command:set_cpu_w", "reassert", "stickiness", "resume", "release",
    /// "auto:allocate", "auto:gpu_clock", ...). The `demand_*`/`alloc_*`/
    /// `pi_target_w` fields carry the WHY of an Auto-mode allocator step
    /// (cause "auto:allocate"); they are None — and skipped on the wire to
    /// keep lines lean — for every other decision.
    Decision {
        t_mono: f64,
        mode: String,
        cpu_limit_w: Option<f64>,
        gpu_max_mhz: Option<u32>,
        fan_target_rpm: f64,
        cause: String,
        flags: Vec<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        demand_cpu: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        demand_gpu: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        alloc_cpu_w: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        alloc_gpu_w: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pi_target_w: Option<f64>,
        /// Current trim offset (RPM); carried on every Auto-mode decision
        /// (cause "auto:trim" marks the updates), None otherwise.
        #[serde(skip_serializing_if = "Option::is_none")]
        trim_rpm: Option<f64>,
    },
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
pub fn lock(
    shared: &std::sync::Mutex<Option<Telemetry>>,
) -> std::sync::MutexGuard<'_, Option<Telemetry>> {
    shared
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
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
            "bazerame-telemetry-test-{}-{name}",
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
        t.log(&Record::Sample(&sample_at(1.0)));
        t.log(&Record::Sample(&sample_at(2.0)));
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
            flags: vec!["resumed".into()],
            demand_cpu: None,
            demand_gpu: None,
            alloc_cpu_w: None,
            alloc_gpu_w: None,
            pi_target_w: None,
            trim_rpm: None,
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
        assert_eq!(start["schema_version"], 1);

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
        assert_eq!(decision["flags"][0], "resumed");
        // None auto fields are skipped entirely: non-auto lines stay lean.
        for key in [
            "demand_cpu",
            "demand_gpu",
            "alloc_cpu_w",
            "alloc_gpu_w",
            "pi_target_w",
            "trim_rpm",
        ] {
            assert!(
                decision.get(key).is_none(),
                "{key} must be skipped when None: {decision}"
            );
        }
        assert!(
            decision["t_wall"].as_f64().unwrap() > 1.5e9,
            "decision lines carry a top-level wall-clock stamp"
        );

        fs::remove_dir_all(&dir).unwrap();
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
        t.log(&Record::Sample(&sample_at(0.0)));
        fs::remove_dir_all(&dir).unwrap();

        // 20 records force flushes past the deleted file; must not panic.
        for i in 1..=20 {
            t.log(&Record::Sample(&sample_at(f64::from(i))));
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
            t.log(&Record::Sample(&sample_at(f64::from(i))));
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
            t.log(&Record::Sample(&sample_at(f64::from(i))));
        }
        // No explicit flush: the 10-record policy must have flushed already
        // (run_start header + 10 records).
        let contents = fs::read_to_string(t.path()).unwrap();
        assert_eq!(contents.lines().count(), 11);

        fs::remove_dir_all(&dir).unwrap();
    }
}

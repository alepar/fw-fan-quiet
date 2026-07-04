//! Append-only JSONL telemetry log for offline controller-quality review:
//! every 1 Hz [`Sample`] (and, from Task 14, every controller decision)
//! becomes one JSON line loadable into pandas/DuckDB.

// Consumed by main wiring in Task 9.
#![allow(dead_code)]

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::types::Sample;

/// Flush at least once every this many records...
const FLUSH_EVERY_RECORDS: u32 = 10;
/// ...and no less often than this, so a quiet log still hits disk.
const FLUSH_MAX_INTERVAL: Duration = Duration::from_secs(5);

/// One JSONL line. Internally tagged so each line carries a `kind` field.
#[derive(serde::Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Record<'a> {
    Sample(&'a Sample),
    Flag {
        t_mono: f64,
        flag: String,
        active: bool,
    },
    // TODO(task-14): Decision variant for controller decisions.
}

/// Buffered JSONL writer with a bounded-staleness flush policy.
pub struct Telemetry {
    writer: BufWriter<File>,
    path: PathBuf,
    /// Records written since the last flush (flush every 10).
    records_since_flush: u32,
    /// When the last flush happened (flush if > 5 s ago).
    last_flush: Instant,
    /// Write errors warn once, then drop records silently.
    warned: bool,
}

impl Telemetry {
    /// Opens `<dir>/run-<unix_secs>.jsonl`, creating `dir` as needed.
    pub fn open(dir: &Path) -> std::io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let unix_secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        let path = dir.join(format!("run-{unix_secs}.jsonl"));
        let file = File::create(&path)?;
        Ok(Self::from_parts(file, path))
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

    /// Serializes one record as a JSON line. Flushes every 10 records or if
    /// more than 5 s passed since the last flush. Errors warn once, then
    /// records are dropped silently; never panics.
    pub fn log(&mut self, r: &Record) {
        let written = serde_json::to_writer(&mut self.writer, r)
            .map_err(std::io::Error::from)
            .and_then(|()| self.writer.write_all(b"\n"));
        if let Err(e) = written {
            self.warn_once(&e);
        }
        self.records_since_flush += 1;
        if self.records_since_flush >= FLUSH_EVERY_RECORDS
            || self.last_flush.elapsed() > FLUSH_MAX_INTERVAL
        {
            self.flush();
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

    fn warn_once(&mut self, e: &std::io::Error) {
        if !self.warned {
            tracing::warn!(
                path = %self.path.display(),
                "telemetry write failed, dropping records from here on: {e}"
            );
            self.warned = true;
        }
    }
}

/// Opens under `preferred_dir`, falling back to the current directory if that
/// fails; `None` only if both fail (telemetry disabled).
pub fn open_with_fallback(preferred_dir: &Path) -> Option<Telemetry> {
    match Telemetry::open(preferred_dir) {
        Ok(t) => Some(t),
        Err(e) => {
            tracing::warn!(
                "cannot open telemetry log under {}: {e}; falling back to current directory",
                preferred_dir.display()
            );
            match Telemetry::open(Path::new(".")) {
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
        t.flush();

        let contents = fs::read_to_string(t.path()).unwrap();
        let lines: Vec<&str> = contents.lines().collect();
        assert_eq!(lines.len(), 3, "expected 3 JSONL lines, got: {contents:?}");

        let first: serde_json::Value = serde_json::from_str(lines[0]).unwrap();
        assert_eq!(first["kind"], "sample");
        assert_eq!(first["t_mono"], 1.0);
        let second: serde_json::Value = serde_json::from_str(lines[1]).unwrap();
        assert_eq!(second["t_mono"], 2.0);
        let flag: serde_json::Value = serde_json::from_str(lines[2]).unwrap();
        assert_eq!(flag["kind"], "flag");
        assert_eq!(flag["flag"], "resumed");
        assert_eq!(flag["active"], true);

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

        let t = open_with_fallback(&blocker.join("sub")).expect("fallback should succeed");
        let path = t.path().to_path_buf();
        assert_eq!(
            path.parent(),
            Some(Path::new(".")),
            "fallback file must land in the current directory, got {path:?}"
        );
        assert!(path.exists());

        drop(t);
        fs::remove_file(&path).unwrap();
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn log_never_panics_after_file_gone() {
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
        // No explicit flush: the 10-record policy must have flushed already.
        let contents = fs::read_to_string(t.path()).unwrap();
        assert_eq!(contents.lines().count(), 10);

        fs::remove_dir_all(&dir).unwrap();
    }
}

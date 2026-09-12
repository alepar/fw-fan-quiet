//! `cros_ec` sensor replica: mirrors fw-fanctrl's own `--thermal` sensor
//! rule (max over every positive reading) closely enough to compare against
//! the socket's reported `temperature`, plus the boxcar moving average
//! fw-fanctrl keeps internally (`EcAverage`), off-by-one included.
//!
//! The checked-in idle fixture keeps the dGPU sensors unavailable, while the
//! loaded fixtures exercise their positive readings. The raw reconciliation
//! stream still follows fw-fanctrl's positive-only rule; the control stream
//! applies the additional plausibility gate documented below.

use std::collections::{BTreeSet, HashMap};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};

/// Highest `tempN` index scanned per chip directory. The known `cros_ec`
/// layout on this machine uses 1..=8; this is scanned generously past that
/// so a firmware with more sensors is not silently truncated. Missing
/// indices (no `tempN_label`) are skipped, not treated as the end of the
/// scan.
const MAX_TEMP_INDEX: u32 = 32;

/// Absolute control-stream plausibility ceiling. Values above this remain
/// available only to fw-fanctrl reconciliation.
pub const EC_PLAUSIBLE_MAX_C: f64 = 110.0;

/// The labels exposed by the Framework 16 EC thermal table.
const EXPECTED_LABELS: [&str; 8] = [
    "ambient_f75303@4d",
    "charger_f75303@4d",
    "cpu@4c",
    "apu_f75303@4d",
    "gpu_vr_f75303@4d",
    "gpu_vram_f75303@4d",
    "gpu_amb_f75303@4d",
    "gpu_temp@40",
];

/// Thermal-control ownership for one EC sensor label.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EcGroup {
    Cpu,
    Gpu,
    Uncontrollable,
    Unknown,
}

/// A sensor diagnostic discovered while constructing an EC reading.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EcDiagnostic {
    /// A labelled sensor produced a value outside the control-stream gate.
    EcImplausible { label: EcLabel },
    /// An expected label is missing, or firmware exposed a label outside
    /// the exact eight-label table.
    EcUnknownLabel {
        label: EcLabel,
        kind: EcUnknownLabelKind,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EcUnknownLabelKind {
    MissingExpected,
    Unexpected,
}

#[derive(Debug)]
struct EcScan {
    reading: Option<EcReading>,
    diagnostics: Vec<EcDiagnostic>,
}

#[derive(Default)]
struct EcDiagnosticLogger {
    previous_by_dir: HashMap<PathBuf, Vec<EcDiagnostic>>,
}

impl EcDiagnosticLogger {
    /// Records and logs a changed diagnostic set. Returns whether the set
    /// changed, which keeps the change-only behavior directly testable.
    fn report(&mut self, dir: &Path, diagnostics: &[EcDiagnostic]) -> bool {
        let previous = self.previous_by_dir.get(dir);
        let changed = previous.map_or(!diagnostics.is_empty(), |old| old != diagnostics);
        self.previous_by_dir
            .insert(dir.to_path_buf(), diagnostics.to_vec());
        if !changed {
            return false;
        }

        if diagnostics.is_empty() {
            tracing::info!("ec {}: sensor diagnostics cleared", dir.display());
        } else {
            for diagnostic in diagnostics {
                match diagnostic {
                    EcDiagnostic::EcImplausible { label } => tracing::info!(
                        "ec {}: EcImplausible label={}",
                        dir.display(),
                        label.as_str()
                    ),
                    EcDiagnostic::EcUnknownLabel { label, kind } => tracing::info!(
                        "ec {}: EcUnknownLabel label={} kind={kind:?}",
                        dir.display(),
                        label.as_str()
                    ),
                }
            }
        }
        true
    }
}

static DIAGNOSTIC_LOGGER: OnceLock<Mutex<EcDiagnosticLogger>> = OnceLock::new();

/// A `cros_ec` sensor label (the trimmed contents of a `tempN_label` file,
/// e.g. `cpu@4c`, `ambient_f75303@4d`).
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EcLabel(String);

impl EcLabel {
    fn new(raw: &str) -> Self {
        Self(raw.trim().to_string())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    /// Classifies the eight known labels exactly, then accepts the carried
    /// device-prefix fallbacks used by firmware variants. An unfamiliar
    /// non-device label is deliberately [`EcGroup::Unknown`].
    pub fn group(&self) -> EcGroup {
        match self.0.as_str() {
            "ambient_f75303@4d" | "charger_f75303@4d" => EcGroup::Uncontrollable,
            "cpu@4c" | "apu_f75303@4d" => EcGroup::Cpu,
            "gpu_vr_f75303@4d" | "gpu_vram_f75303@4d" | "gpu_amb_f75303@4d" | "gpu_temp@40" => {
                EcGroup::Gpu
            }
            label if label.starts_with("cpu") || label.starts_with("apu") => EcGroup::Cpu,
            label if label.starts_with("gpu_") => EcGroup::Gpu,
            _ => EcGroup::Unknown,
        }
    }

    pub fn is_controllable(&self) -> bool {
        matches!(self.group(), EcGroup::Cpu | EcGroup::Gpu)
    }

    /// True only for the two known ambient/charger sensors.
    #[allow(dead_code)] // consumed by the forthcoming T* source
    pub fn is_uncontrollable(&self) -> bool {
        self.group() == EcGroup::Uncontrollable
    }
}

/// One `cros_ec` max-temperature reading: the highest positive `tempN`
/// value across every labelled sensor, plus which sensor produced it and
/// every positive reading that took part.
#[derive(Debug, Clone, PartialEq)]
pub struct EcReading {
    pub max_c: i32,
    pub argmax: EcLabel,
    /// Every plausible control reading in degrees C (unrounded), in sysfs
    /// (`tempN`) order.
    pub all: Vec<(EcLabel, f64)>,
    /// Maximum plausible CPU-group reading.
    pub cpu_group_c: Option<f64>,
    /// Maximum plausible GPU-group reading; `None` while the dGPU sensors
    /// are absent or unpowered.
    pub gpu_group_c: Option<f64>,
    /// Positive finite raw maximum, including values above the control
    /// plausibility ceiling, rounded like fw-fanctrl.
    pub reconciliation_max_c: Option<i32>,
    /// Diagnostics for the labels observed during this scan.
    pub diagnostics: Vec<EcDiagnostic>,
}

impl EcReading {
    /// Reads every `tempN_label` / `tempN_input` pair under `dir` (a
    /// `cros_ec` hwmon chip directory), drops readings that are `<= 0`,
    /// unreadable, or unparsable — that covers both the machine's constant
    /// `-150` sentinel and a labelled sensor with no `_input` file at all
    /// (the ENODATA convention) — rounds the surviving max to the nearest
    /// integer °C, and breaks ties on the max by sysfs order (lowest `N`
    /// wins, since sensors are scanned in ascending `N` and only a
    /// strictly greater reading replaces the current argmax).
    ///
    /// Returns `None` if the directory yields no positive reading at all
    /// (chip present but every sensor dead, or the directory doesn't
    /// exist) — callers use this to drive `Sample.ec_valid`.
    pub fn read(dir: &Path) -> Option<Self> {
        let scan = Self::scan(dir);
        let logger = DIAGNOSTIC_LOGGER.get_or_init(|| Mutex::new(EcDiagnosticLogger::default()));
        logger
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .report(dir, &scan.diagnostics);
        scan.reading
    }

    fn scan(dir: &Path) -> EcScan {
        let mut readings = Vec::new();
        let mut present_labels = BTreeSet::new();
        for n in 1..=MAX_TEMP_INDEX {
            let label_path = dir.join(format!("temp{n}_label"));
            let label_text = match fs::read_to_string(&label_path) {
                Ok(s) => s,
                Err(_) => continue, // no sensor at this index
            };
            let label = EcLabel::new(&label_text);
            present_labels.insert(label.clone());
            let input_path = dir.join(format!("temp{n}_input"));
            let raw_milli: f64 = match fs::read_to_string(&input_path) {
                Ok(s) => match s.trim().parse() {
                    Ok(v) => v,
                    Err(e) => {
                        tracing::debug!("ec {}: unparseable value: {e}", input_path.display());
                        continue;
                    }
                },
                Err(e) => {
                    // Covers both "file absent" (ENODATA convention) and
                    // any other read failure -- both dropped the same way.
                    tracing::debug!("ec {}: {e}", input_path.display());
                    continue;
                }
            };
            readings.push((label, raw_milli / 1000.0));
        }
        Self::from_scan_parts(readings, present_labels)
    }

    /// Builds the separate raw-reconciliation and plausible-control streams.
    /// `None` means no plausible control reading was present.
    #[cfg(test)]
    fn from_readings(readings: Vec<(EcLabel, f64)>) -> Option<Self> {
        let present_labels = readings.iter().map(|(label, _)| label.clone()).collect();
        Self::from_scan_parts(readings, present_labels).reading
    }

    fn from_scan_parts(readings: Vec<(EcLabel, f64)>, present_labels: BTreeSet<EcLabel>) -> EcScan {
        let reconciliation_max_c = readings
            .iter()
            .map(|(_, value)| *value)
            .filter(|value| value.is_finite() && *value > 0.0)
            .max_by(f64::total_cmp)
            .map(|value| value.round() as i32);

        let expected: BTreeSet<EcLabel> = EXPECTED_LABELS
            .iter()
            .map(|raw| EcLabel::new(raw))
            .collect();
        let mut diagnostics: BTreeSet<EcDiagnostic> = BTreeSet::new();
        for label in expected.difference(&present_labels) {
            diagnostics.insert(EcDiagnostic::EcUnknownLabel {
                label: label.clone(),
                kind: EcUnknownLabelKind::MissingExpected,
            });
        }
        for label in present_labels.difference(&expected) {
            diagnostics.insert(EcDiagnostic::EcUnknownLabel {
                label: label.clone(),
                kind: EcUnknownLabelKind::Unexpected,
            });
        }

        let mut all = Vec::new();
        for (label, value) in readings {
            if value.is_finite() && value > 0.0 && value <= EC_PLAUSIBLE_MAX_C {
                all.push((label, value));
            } else {
                diagnostics.insert(EcDiagnostic::EcImplausible { label });
            }
        }
        let diagnostics: Vec<EcDiagnostic> = diagnostics.into_iter().collect();

        let mut best_idx = None;
        let mut best_val = f64::NEG_INFINITY;
        let mut cpu_group_c: Option<f64> = None;
        let mut gpu_group_c: Option<f64> = None;
        for (i, (_, v)) in all.iter().enumerate() {
            if *v > best_val {
                best_val = *v;
                best_idx = Some(i);
            }
            let group_max = match all[i].0.group() {
                EcGroup::Cpu => &mut cpu_group_c,
                EcGroup::Gpu => &mut gpu_group_c,
                EcGroup::Uncontrollable | EcGroup::Unknown => continue,
            };
            *group_max = Some(group_max.map_or(*v, |current| current.max(*v)));
        }
        let reading = best_idx.map(|idx| EcReading {
            max_c: best_val.round() as i32,
            argmax: all[idx].0.clone(),
            all,
            cpu_group_c,
            gpu_group_c,
            reconciliation_max_c,
            diagnostics: diagnostics.clone(),
        });
        EcScan {
            reading,
            diagnostics,
        }
    }
}

/// fw-fanctrl's boxcar moving average, replicated exactly including its
/// off-by-one: `adapt_speed` reads `mean(buffer)` (samples so far, `n-N..
/// n-1`) *before* appending the current tick's own sample. [`push`] mirrors
/// that in one call — it returns the mean as of *before* `sample_c` is
/// folded in, then appends `sample_c` (dropped, like a `<= 0` EC reading,
/// if it isn't positive) for the *next* call to see. Do not "simplify" this
/// to return the post-push mean; that changes the replica's output and the
/// tests below pin the exact off-by-one sequence with literal values.
pub struct EcAverage {
    /// Retained non-zero samples, oldest first, capped at `interval`.
    buffer: std::collections::VecDeque<f64>,
    interval: usize,
    /// Set by `reseed`, or once the buffer naturally reaches `interval`
    /// samples. Sticky: nothing un-sets it (there is no "un-seed"
    /// operation, only `reseed`, which always leaves the boxcar seeded).
    seeded: bool,
}

/// fw-fanctrl's deque maxlen — `set_interval` clamps to this regardless of
/// what the socket reports.
pub const MAX_INTERVAL: usize = 100;

impl EcAverage {
    /// `interval` is clamped to `[1, MAX_INTERVAL]`.
    pub fn new(interval: usize) -> Self {
        Self {
            buffer: std::collections::VecDeque::new(),
            interval: interval.clamp(1, MAX_INTERVAL),
            seeded: false,
        }
    }

    /// Returns the boxcar mean of samples `n-N..n-1` (`None` before any
    /// sample has ever been retained), then folds `sample_c` into the
    /// buffer for the next call — dropped without affecting the returned
    /// mean if it is `<= 0`, mirroring the EC reading rule. The buffer
    /// never holds more than the current interval; the oldest sample is
    /// dropped to make room.
    pub fn push(&mut self, sample_c: f64) -> Option<f64> {
        let mean = self.mean();
        if sample_c > 0.0 {
            self.buffer.push_back(sample_c);
            while self.buffer.len() > self.interval {
                self.buffer.pop_front();
            }
            if self.buffer.len() >= self.interval {
                self.seeded = true;
            }
        }
        mean
    }

    fn mean(&self) -> Option<f64> {
        if self.buffer.is_empty() {
            None
        } else {
            Some(self.buffer.iter().sum::<f64>() / self.buffer.len() as f64)
        }
    }

    /// Grows or shrinks the window, capped at [`MAX_INTERVAL`], **without
    /// clearing** retained samples: shrinking drops the oldest samples down
    /// to the new size; growing only raises the cap, so the mean is
    /// unaffected until later `push` calls fill the wider window.
    pub fn set_interval(&mut self, n: usize) {
        self.interval = n.clamp(1, MAX_INTERVAL);
        while self.buffer.len() > self.interval {
            self.buffer.pop_front();
        }
        if self.buffer.len() >= self.interval {
            self.seeded = true;
        }
    }

    /// Replaces the buffer's entire contents with a single sample and
    /// marks the boxcar seeded. The **only** operation that clears
    /// retained samples.
    pub fn reseed(&mut self, value: f64) {
        self.buffer.clear();
        self.buffer.push_back(value);
        self.seeded = true;
    }

    /// True once `reseed` has been called, or the boxcar has accumulated a
    /// full window of `interval` samples on its own. False before either
    /// has happened — a mean read at that point averages fewer than a full
    /// window and callers should not trust it as one.
    ///
    /// No production call site today (integration sweep, `fw-fanctrl-loop-nsc`):
    /// the controller achieves the same "don't trust an underfilled mean"
    /// invariant a different way — it calls `reseed` itself on the very
    /// first sample of every engagement that carries a fw-fanctrl view
    /// (`Controller`'s own `ec_seeded` latch, before any `push`), so by the
    /// time `ec_avg.push` is ever read as `ec_ma` the boxcar has already
    /// been seeded. `is_seeded`/`sample_count` remain here as the design's
    /// own owned API surface for `EcAverage` (§2.2/§2.6) and are exercised
    /// directly by this module's own tests.
    #[allow(dead_code)]
    pub fn is_seeded(&self) -> bool {
        self.seeded
    }

    /// Retained sample count (`<= interval`). See [`Self::is_seeded`] for
    /// why this has no production call site today.
    #[allow(dead_code)]
    pub fn sample_count(&self) -> usize {
        self.buffer.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::fixtures;
    use std::fs;
    use std::path::PathBuf;

    // --- EcReading, on the checked-in fixtures -----------------------

    #[test]
    fn cros_ec_idle_max_is_ambient_47_85_rounds_to_48() {
        let dir = fixtures::path("hwmon/cros_ec_idle");
        let reading = EcReading::read(&dir).expect("cros_ec_idle should yield a reading");
        assert_eq!(reading.max_c, 48, "47.85 should round to 48");
        assert_eq!(reading.argmax.as_str(), "ambient_f75303@4d");
        // The three -150 sensors and the input-less gpu_temp@40 sensor are
        // dropped, but the reading itself stays valid (Some, not None) and
        // the surviving positive readings are exactly ambient/charger/apu/cpu.
        let labels: Vec<&str> = reading.all.iter().map(|(l, _)| l.as_str()).collect();
        assert_eq!(
            labels.len(),
            4,
            "expected 4 surviving readings, got {labels:?}"
        );
        assert!(!labels.iter().any(|l| l.starts_with("gpu")));
    }

    #[test]
    fn loaded_fixtures_have_gpu_argmax_near_83_while_idle_has_none() {
        let idle = EcReading::read(&fixtures::path("hwmon/cros_ec_idle"))
            .expect("cros_ec_idle should yield a reading");
        assert_eq!(idle.argmax.as_str(), "ambient_f75303@4d");
        assert_eq!(idle.gpu_group_c, None);

        let dir = fixtures::path("hwmon/cros_ec_load");
        let reading = EcReading::read(&dir).expect("cros_ec_load should yield a reading");
        assert_eq!(reading.max_c, 83);
        assert_eq!(reading.argmax.as_str(), "gpu_amb_f75303@4d");
        assert_eq!(reading.gpu_group_c, Some(83.0));

        let dgpu = EcReading::read(&fixtures::path("hwmon/cros_ec_dgpu_on"))
            .expect("cros_ec_dgpu_on should yield a reading");
        assert_eq!(dgpu.max_c, 83);
        assert_eq!(dgpu.argmax.as_str(), "gpu_amb_f75303@4d");
        assert_eq!(dgpu.gpu_group_c, Some(83.0));
    }

    #[test]
    fn controllable_and_uncontrollable_labels_classify_correctly() {
        let cases = [
            ("ambient_f75303@4d", EcGroup::Uncontrollable),
            ("charger_f75303@4d", EcGroup::Uncontrollable),
            ("cpu@4c", EcGroup::Cpu),
            ("apu_f75303@4d", EcGroup::Cpu),
            ("gpu_vr_f75303@4d", EcGroup::Gpu),
            ("gpu_vram_f75303@4d", EcGroup::Gpu),
            ("gpu_amb_f75303@4d", EcGroup::Gpu),
            ("gpu_temp@40", EcGroup::Gpu),
            ("cpu_extra", EcGroup::Cpu),
            ("apu_extra", EcGroup::Cpu),
            ("gpu_extra", EcGroup::Gpu),
            ("unrelated_aux", EcGroup::Unknown),
        ];

        for (raw, expected) in cases {
            let label = EcLabel::new(raw);
            assert_eq!(label.group(), expected, "wrong group for {raw}");
            assert_eq!(
                label.is_controllable(),
                matches!(expected, EcGroup::Cpu | EcGroup::Gpu),
                "wrong controllability for {raw}"
            );
            assert_eq!(
                label.is_uncontrollable(),
                expected == EcGroup::Uncontrollable,
                "wrong uncontrollability for {raw}"
            );
        }
    }

    // --- synthetic: a positive gpu_* reading joins the max -------------

    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-ec-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn write_sensor(dir: &Path, n: u32, label: &str, milli_c: i64) {
        fs::write(dir.join(format!("temp{n}_label")), format!("{label}\n")).unwrap();
        fs::write(dir.join(format!("temp{n}_input")), format!("{milli_c}\n")).unwrap();
    }

    fn write_label(dir: &Path, n: u32, label: &str) {
        fs::write(dir.join(format!("temp{n}_label")), format!("{label}\n")).unwrap();
    }

    fn readings(values: &[(&str, f64)]) -> Vec<(EcLabel, f64)> {
        values
            .iter()
            .map(|(label, value)| (EcLabel::new(label), *value))
            .collect()
    }

    #[test]
    fn implausible_values_are_excluded_from_control_but_high_values_reconcile() {
        let reading = EcReading::from_readings(readings(&[
            ("cpu@4c", 50.0),
            ("gpu_vr_f75303@4d", 70.0),
            ("ambient_f75303@4d", 60.0),
            ("gpu_bad", 150.0),
            ("negative_sensor", -150.0),
            ("nan_sensor", f64::NAN),
            ("infinite_sensor", f64::INFINITY),
        ]))
        .expect("plausible values keep the control reading valid");

        assert_eq!(reading.max_c, 70);
        assert_eq!(reading.argmax.as_str(), "gpu_vr_f75303@4d");
        assert_eq!(reading.cpu_group_c, Some(50.0));
        assert_eq!(reading.gpu_group_c, Some(70.0));
        assert_eq!(reading.reconciliation_max_c, Some(150));
        assert_eq!(
            reading
                .all
                .iter()
                .map(|(label, _)| label.as_str())
                .collect::<Vec<_>>(),
            ["cpu@4c", "gpu_vr_f75303@4d", "ambient_f75303@4d"]
        );
        for rejected in [
            "gpu_bad",
            "negative_sensor",
            "nan_sensor",
            "infinite_sensor",
        ] {
            assert!(reading.diagnostics.iter().any(|diagnostic| matches!(
                diagnostic,
                EcDiagnostic::EcImplausible { label } if label.as_str() == rejected
            )));
        }
    }

    #[test]
    fn scanned_nonfinite_and_sentinel_values_are_rejected_from_both_streams() {
        let dir = fixture_dir("scanned-implausible-values");
        write_sensor(&dir, 1, "cpu@4c", 50_000);
        write_sensor(&dir, 2, "gpu_vr_f75303@4d", -150_000);
        write_label(&dir, 3, "gpu_vram_f75303@4d");
        fs::write(dir.join("temp3_input"), "NaN\n").unwrap();
        write_label(&dir, 4, "gpu_amb_f75303@4d");
        fs::write(dir.join("temp4_input"), "inf\n").unwrap();
        write_sensor(&dir, 5, "gpu_temp@40", 150_000);

        let reading = EcReading::read(&dir).expect("the CPU reading is plausible");
        assert_eq!(reading.max_c, 50);
        assert_eq!(reading.cpu_group_c, Some(50.0));
        assert_eq!(reading.gpu_group_c, None);
        assert_eq!(reading.reconciliation_max_c, Some(150));
        assert_eq!(
            reading
                .all
                .iter()
                .map(|(label, _)| label.as_str())
                .collect::<Vec<_>>(),
            ["cpu@4c"]
        );
        for rejected in [
            "gpu_vr_f75303@4d",
            "gpu_vram_f75303@4d",
            "gpu_amb_f75303@4d",
            "gpu_temp@40",
        ] {
            assert!(reading.diagnostics.iter().any(|diagnostic| matches!(
                diagnostic,
                EcDiagnostic::EcImplausible { label } if label.as_str() == rejected
            )));
        }

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn plausible_105_remains_in_its_group_while_raw_only_input_is_invalid() {
        let reading = EcReading::from_readings(readings(&[("gpu_temp@40", 105.0)]))
            .expect("105 C is inside the absolute plausibility bound");
        assert_eq!(reading.max_c, 105);
        assert_eq!(reading.gpu_group_c, Some(105.0));
        assert_eq!(reading.reconciliation_max_c, Some(105));

        assert_eq!(
            EcReading::from_readings(readings(&[("gpu_temp@40", 150.0)])),
            None,
            "a raw-only value must not keep EC control validity true"
        );
    }

    #[test]
    fn label_diagnostics_change_once_and_present_labels_without_input_are_not_missing() {
        let dir = fixture_dir("diagnostic-changes");
        for (n, label) in EXPECTED_LABELS.iter().take(7).enumerate() {
            write_label(&dir, n as u32 + 1, label);
        }
        fs::write(dir.join("temp3_input"), "50000\n").unwrap();

        let initial = EcReading::scan(&dir);
        assert!(initial.diagnostics.contains(&EcDiagnostic::EcUnknownLabel {
            label: EcLabel::new("gpu_temp@40"),
            kind: EcUnknownLabelKind::MissingExpected,
        }));
        let mut logger = EcDiagnosticLogger::default();
        assert!(logger.report(&dir, &initial.diagnostics));
        assert!(!logger.report(&dir, &initial.diagnostics));

        write_label(&dir, 8, "gpu_temp@40");
        let complete = EcReading::scan(&dir);
        assert!(complete.diagnostics.is_empty());
        assert!(logger.report(&dir, &complete.diagnostics));
        assert!(!logger.report(&dir, &complete.diagnostics));

        fs::remove_file(dir.join("temp8_label")).unwrap();
        let missing = EcReading::scan(&dir);
        assert!(missing.diagnostics.contains(&EcDiagnostic::EcUnknownLabel {
            label: EcLabel::new("gpu_temp@40"),
            kind: EcUnknownLabelKind::MissingExpected,
        }));
        assert!(logger.report(&dir, &missing.diagnostics));
        assert!(!logger.report(&dir, &missing.diagnostics));

        write_sensor(&dir, 9, "unrelated_aux", 90_000);
        let unexpected = EcReading::scan(&dir);
        assert!(unexpected
            .diagnostics
            .contains(&EcDiagnostic::EcUnknownLabel {
                label: EcLabel::new("unrelated_aux"),
                kind: EcUnknownLabelKind::Unexpected,
            }));
        let reading = unexpected
            .reading
            .expect("CPU and unknown values are plausible");
        assert_eq!(reading.argmax.as_str(), "unrelated_aux");
        assert_eq!(reading.argmax.group(), EcGroup::Unknown);
        assert!(!reading.argmax.is_controllable());
        assert!(!reading.argmax.is_uncontrollable());
        assert!(logger.report(&dir, &unexpected.diagnostics));
        assert!(!logger.report(&dir, &unexpected.diagnostics));

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_synthetic_positive_gpu_reading_joins_the_max_and_is_controllable() {
        let dir = fixture_dir("synthetic-gpu");
        write_sensor(&dir, 1, "ambient_f75303@4d", 40_000); // 40.0 C
        write_sensor(&dir, 2, "cpu@4c", 45_000); // 45.0 C
        write_sensor(&dir, 3, "gpu_amb_f75303@4d", 90_000); // 90.0 C -- a future firmware waking up

        let reading = EcReading::read(&dir).expect("synthetic tree should yield a reading");
        assert_eq!(reading.max_c, 90);
        assert_eq!(reading.argmax.as_str(), "gpu_amb_f75303@4d");
        assert!(reading.argmax.is_controllable());

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn ties_in_the_max_are_broken_by_sysfs_order() {
        let dir = fixture_dir("tie");
        // temp1 (apu) and temp2 (cpu) tie exactly; temp1 must win because
        // it is scanned first (lower tempN = earlier in sysfs order).
        write_sensor(&dir, 1, "apu_f75303@4d", 50_000);
        write_sensor(&dir, 2, "cpu@4c", 50_000);

        let reading = EcReading::read(&dir).expect("tie tree should yield a reading");
        assert_eq!(reading.max_c, 50);
        assert_eq!(
            reading.argmax.as_str(),
            "apu_f75303@4d",
            "a tie must resolve to the lower tempN (sysfs order), not cpu@4c"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn empty_directory_yields_no_reading() {
        let dir = fixture_dir("empty");
        assert_eq!(EcReading::read(&dir), None);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn missing_directory_yields_no_reading() {
        assert_eq!(EcReading::read(Path::new("/nonexistent/ec-dir")), None);
    }

    // --- EcAverage: boxcar off-by-one -----------------------------------

    #[test]
    fn push_returns_the_pre_push_mean_literal_values() {
        // interval 3. Each push's return is the mean of what was already
        // retained -- never including the value just passed in. Written as
        // literal expected values on purpose: a "fix" that folds sample_c
        // into its own return would change every number below.
        let mut avg = EcAverage::new(3);
        assert_eq!(
            avg.push(10.0),
            None,
            "nothing retained before the first push"
        );
        assert_eq!(avg.push(20.0), Some(10.0), "mean of [10]");
        assert_eq!(avg.push(30.0), Some(15.0), "mean of [10, 20]");
        assert_eq!(avg.push(40.0), Some(20.0), "mean of [10, 20, 30]");
        // buffer is now capped at 3: [20, 30, 40]
        assert_eq!(avg.push(50.0), Some(30.0), "mean of [20, 30, 40]");
    }

    #[test]
    fn non_positive_samples_are_dropped_without_affecting_the_mean() {
        let mut avg = EcAverage::new(3);
        avg.push(10.0);
        avg.push(20.0);
        // A dropped sample still returns the current mean, but does not
        // get retained.
        assert_eq!(avg.push(0.0), Some(15.0));
        assert_eq!(
            avg.push(-5.0),
            Some(15.0),
            "still [10, 20], unaffected by the drop"
        );
        assert_eq!(avg.sample_count(), 2);
    }

    #[test]
    fn set_interval_grows_and_shrinks_without_clearing_and_caps_at_100() {
        let mut avg = EcAverage::new(5);
        for v in [10.0, 20.0, 30.0, 40.0, 50.0] {
            avg.push(v);
        }
        assert_eq!(avg.sample_count(), 5);

        // Shrink to 3: drops the two oldest, keeps the rest -- no clear.
        avg.set_interval(3);
        assert_eq!(avg.sample_count(), 3);
        assert_eq!(avg.push(60.0), Some(40.0), "mean of retained [30, 40, 50]");
        // buffer is now [40, 50, 60] (capped at the interval-3 during that push)

        // Grow the interval: the 3 retained samples are untouched.
        avg.set_interval(10);
        assert_eq!(
            avg.sample_count(),
            3,
            "growing keeps every retained sample, it does not clear"
        );
        // Subsequent pushes now fill toward the wider window instead of
        // being capped back down to 3.
        avg.push(70.0);
        avg.push(80.0);
        assert_eq!(avg.sample_count(), 5, "buffer grew to [40, 50, 60, 70, 80]");

        let mut fresh = EcAverage::new(1);
        fresh.set_interval(usize::MAX);
        for v in 1..=150 {
            fresh.push(f64::from(v));
        }
        assert_eq!(fresh.sample_count(), MAX_INTERVAL, "interval caps at 100");
    }

    #[test]
    fn reseed_is_the_only_clearing_operation() {
        let mut avg = EcAverage::new(3);
        avg.push(10.0);
        avg.push(20.0);
        assert_eq!(avg.sample_count(), 2);

        avg.reseed(99.0);
        assert_eq!(
            avg.sample_count(),
            1,
            "reseed replaces the contents with one value"
        );
        assert_eq!(
            avg.push(1.0),
            Some(99.0),
            "the seeded value is what the next push sees"
        );
    }

    #[test]
    fn is_seeded_false_until_reseed_or_a_full_window() {
        let mut avg = EcAverage::new(3);
        assert!(!avg.is_seeded());
        avg.push(10.0);
        assert!(!avg.is_seeded(), "1 of 3 samples is not a full window");
        avg.push(20.0);
        assert!(!avg.is_seeded(), "2 of 3 samples is not a full window");
        avg.push(30.0);
        assert!(avg.is_seeded(), "3 of 3 samples is a full window");

        let mut seeded_directly = EcAverage::new(100);
        assert!(!seeded_directly.is_seeded());
        seeded_directly.reseed(42.0);
        assert!(
            seeded_directly.is_seeded(),
            "reseed always seeds, regardless of interval"
        );
        assert_eq!(seeded_directly.sample_count(), 1);
    }
}

//! `cros_ec` sensor replica: mirrors fw-fanctrl's own `--thermal` sensor
//! rule (max over every positive reading) closely enough to compare against
//! the socket's reported `temperature`, plus the boxcar moving average
//! fw-fanctrl keeps internally (`EcAverage`), off-by-one included.
//!
//! §Facts (2026-09-08, dGPU powered at 18.9 W): the `gpu_amb`, `gpu_vr` and
//! `gpu_vram` sensors still read −150 and `gpu_temp@40` still returns
//! ENODATA. They never report on this machine. The rule below is
//! nonetheless "every positive reading joins the max" — it matches
//! fw-fanctrl's own regex and costs nothing if a future firmware makes them
//! live; a synthetic positive `gpu_*` reading is exercised in the tests
//! below.

use std::fs;
use std::path::Path;

/// Highest `tempN` index scanned per chip directory. The known `cros_ec`
/// layout on this machine uses 1..=8; this is scanned generously past that
/// so a firmware with more sensors is not silently truncated. Missing
/// indices (no `tempN_label`) are skipped, not treated as the end of the
/// scan.
const MAX_TEMP_INDEX: u32 = 32;

/// A `cros_ec` sensor label (the trimmed contents of a `tempN_label` file,
/// e.g. `cpu@4c`, `ambient_f75303@4d`).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct EcLabel(String);

impl EcLabel {
    fn new(raw: &str) -> Self {
        Self(raw.trim().to_string())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    /// **Controllable** = `apu`, `cpu`, `gpu_*` (the fan curve can be
    /// steered by managing those thermal sources); **uncontrollable** =
    /// everything else (`ambient`, `charger` on this machine — the
    /// classification is a safe default for a label neither prefix set
    /// recognizes, not just those two).
    pub fn is_controllable(&self) -> bool {
        self.0.starts_with("apu") || self.0.starts_with("cpu") || self.0.starts_with("gpu_")
    }
}

/// One `cros_ec` max-temperature reading: the highest positive `tempN`
/// value across every labelled sensor, plus which sensor produced it and
/// every positive reading that took part.
#[derive(Debug, Clone, PartialEq)]
pub struct EcReading {
    pub max_c: i32,
    pub argmax: EcLabel,
    /// Every positive, readable, parseable reading in degrees C (unrounded),
    /// in sysfs (`tempN`) order.
    pub all: Vec<(EcLabel, f64)>,
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
        let mut all = Vec::new();
        for n in 1..=MAX_TEMP_INDEX {
            let label_path = dir.join(format!("temp{n}_label"));
            let label_text = match fs::read_to_string(&label_path) {
                Ok(s) => s,
                Err(_) => continue, // no sensor at this index
            };
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
            let c = raw_milli / 1000.0;
            if c <= 0.0 {
                continue;
            }
            all.push((EcLabel::new(&label_text), c));
        }
        Self::from_readings(all)
    }

    /// Picks the max (and argmax, sysfs-order tie broken) from an
    /// already-filtered reading list. `None` if `all` is empty.
    fn from_readings(all: Vec<(EcLabel, f64)>) -> Option<Self> {
        let mut best_idx = None;
        let mut best_val = f64::NEG_INFINITY;
        for (i, (_, v)) in all.iter().enumerate() {
            if *v > best_val {
                best_val = *v;
                best_idx = Some(i);
            }
        }
        let idx = best_idx?;
        Some(EcReading {
            max_c: best_val.round() as i32,
            argmax: all[idx].0.clone(),
            all,
        })
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
    fn cros_ec_load_max_is_cpu_74_85_rounds_to_75_matching_the_socket() {
        let dir = fixtures::path("hwmon/cros_ec_load");
        let reading = EcReading::read(&dir).expect("cros_ec_load should yield a reading");
        assert_eq!(reading.max_c, 75, "74.85 should round to 75");
        assert_eq!(reading.argmax.as_str(), "cpu@4c");

        let load_json = fs::read_to_string(fixtures::path("fanctrl/print_all_load.json")).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&load_json).unwrap();
        let temperature = parsed["temperature"].as_f64().unwrap();
        assert_eq!(
            f64::from(reading.max_c),
            temperature,
            "replica max should match the socket's reported temperature"
        );
    }

    #[test]
    fn cros_ec_dgpu_on_gpu_sensors_still_never_report() {
        // §Facts: measured with the dGPU powered at 18.9 W. The rule is
        // still "every positive reading joins the max" (next test), but on
        // *this* fixture the max must come from cpu/ambient because the
        // gpu_* sensors are still dead.
        let dir = fixtures::path("hwmon/cros_ec_dgpu_on");
        let reading = EcReading::read(&dir).expect("cros_ec_dgpu_on should yield a reading");
        assert!(
            !reading.argmax.as_str().starts_with("gpu"),
            "argmax {} should not be a gpu_* sensor -- they never report on this machine",
            reading.argmax.as_str()
        );
        assert!(
            reading
                .all
                .iter()
                .all(|(l, _)| !l.as_str().starts_with("gpu")),
            "no gpu_* sensor should have survived the positive-reading filter"
        );
        // Same idle-shaped tree as cros_ec_idle (ambient/apu/cpu unchanged
        // by dGPU state on this machine), so the max is the same 48 °C.
        assert_eq!(reading.max_c, 48);
        assert_eq!(reading.argmax.as_str(), "ambient_f75303@4d");
    }

    // --- EcLabel classification ---------------------------------------

    #[test]
    fn controllable_and_uncontrollable_labels_classify_correctly() {
        assert!(EcLabel::new("apu_f75303@4d").is_controllable());
        assert!(EcLabel::new("cpu@4c").is_controllable());
        assert!(EcLabel::new("gpu_amb_f75303@4d").is_controllable());
        assert!(!EcLabel::new("ambient_f75303@4d").is_controllable());
        assert!(!EcLabel::new("charger_f75303@4d").is_controllable());
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

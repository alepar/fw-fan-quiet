//! 1 Hz sampler: owns all sensor structs, flattens their `Option` readings
//! into a dense [`Sample`] (None -> 0.0 + validity flags), and fans each
//! sample out to every subscriber channel from a dedicated thread.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;

use crate::event::Event;
use crate::sensors::cpu::{self, CpuUtil};
use crate::sensors::gpu::GpuSensor;
use crate::sensors::hwmon::Hwmon;
use crate::sensors::rapl::RaplReader;
use crate::types::Sample;

/// Target sampling cadence.
const SAMPLE_PERIOD: Duration = Duration::from_secs(1);

/// The inter-sample sleep checks the shutdown flag at least this often, so
/// quitting never waits out a full sample period.
const SHUTDOWN_POLL: Duration = Duration::from_millis(250);

/// Sleeps `total` in slices of at most [`SHUTDOWN_POLL`], returning early
/// once `shutdown` flips.
fn sleep_unless_shutdown(total: Duration, shutdown: &AtomicBool) {
    let mut remaining = total;
    while !remaining.is_zero() && !shutdown.load(Ordering::Relaxed) {
        let slice = remaining.min(SHUTDOWN_POLL);
        std::thread::sleep(slice);
        remaining -= slice;
    }
}

/// Monotonic gap larger than this between consecutive samples means the
/// machine slept (normal cadence is ~1 s).
const RESUME_GAP_S: f64 = 5.0;

/// True if the monotonic-time gap between consecutive samples indicates a
/// suspend/resume rather than normal 1 s cadence.
pub fn is_resume_gap(prev_t: f64, cur_t: f64) -> bool {
    cur_t - prev_t > RESUME_GAP_S
}

/// Owns all sensors; `sample()` produces one flattened [`Sample`].
pub struct Sampler {
    hwmon: Hwmon,
    rapl: Option<RaplReader>,
    cpu_util: CpuUtil,
    gpu: Option<GpuSensor>,
    cpufreq_base: PathBuf,
    /// Epoch for `t_mono` (set at construction, i.e. process start).
    /// Linux `Instant` is CLOCK_BOOTTIME-backed since Rust 1.87 (pinned in
    /// Cargo.toml): it advances during suspend, which `is_resume_gap` needs.
    epoch: Instant,
    /// `t_mono` of the previous `sample()` call, for resume detection.
    prev_t: Option<f64>,
}

impl Sampler {
    /// Production constructor: real sysfs/procfs paths and NVML.
    /// Logs a warning once if the NVIDIA GPU is unavailable.
    pub fn new_system() -> Self {
        let gpu = match GpuSensor::new() {
            Ok(gpu) => Some(gpu),
            Err(e) => {
                tracing::warn!("NVML unavailable, sampling without dGPU: {e}");
                None
            }
        };
        Self::with_paths(
            Path::new("/sys/class/hwmon"),
            Some(Path::new("/sys/class/powercap/intel-rapl:0")),
            Path::new("/proc/stat"),
            Path::new("/sys/devices/system/cpu"),
            gpu,
        )
    }

    /// Path-injectable constructor (tests pass fixture trees; `new_system`
    /// passes the real ones).
    pub fn with_paths(
        hwmon_root: &Path,
        rapl_base: Option<&Path>,
        stat_path: &Path,
        cpufreq_base: &Path,
        gpu: Option<GpuSensor>,
    ) -> Self {
        Self {
            hwmon: Hwmon::discover(hwmon_root),
            rapl: rapl_base.and_then(RaplReader::new),
            cpu_util: CpuUtil::new(stat_path),
            gpu,
            cpufreq_base: cpufreq_base.to_path_buf(),
            epoch: Instant::now(),
            prev_t: None,
        }
    }

    /// Reads every sensor and returns one flattened sample stamped with the
    /// current monotonic time.
    pub fn sample(&mut self) -> Sample {
        let t_mono = self.epoch.elapsed().as_secs_f64();
        self.sample_at(t_mono)
    }

    /// Testable core of `sample()`: builds the sample for an injected
    /// `t_mono` (tests drive resume detection without sleeping).
    fn sample_at(&mut self, t_mono: f64) -> Sample {
        let resumed = self
            .prev_t
            .is_some_and(|prev_t| is_resume_gap(prev_t, t_mono));
        self.prev_t = Some(t_mono);

        let fans = self.hwmon.fan_rpms();
        let cpu_temp = self.hwmon.cpu_temp_c();
        // GpuReading::default() is all-None, so a missing sensor and a failed
        // read flatten identically (gpu_w_valid false).
        let gpu = self.gpu.as_ref().map(|g| g.read()).unwrap_or_default();

        Sample {
            t_mono,
            fan1_rpm: fans.map_or(0.0, |(fan1, _)| fan1),
            fan2_rpm: fans.map_or(0.0, |(_, fan2)| fan2),
            fan_valid: fans.is_some(),
            cpu_temp_c: cpu_temp.unwrap_or(0.0),
            cpu_temp_valid: cpu_temp.is_some(),
            // RAPL None on the first call is a normal warmup, so cpu_pkg_w
            // has no validity flag; it just flattens to 0.0.
            cpu_pkg_w: self
                .rapl
                .as_mut()
                .and_then(RaplReader::read_watts)
                .unwrap_or(0.0),
            igpu_w: self.hwmon.igpu_w().unwrap_or(0.0),
            gpu_w: gpu.power_w.unwrap_or(0.0),
            gpu_w_valid: gpu.power_w.is_some(),
            gpu_temp_c: gpu.temp_c.unwrap_or(0.0),
            gpu_temp_valid: gpu.temp_c.is_some(),
            gpu_sm_mhz: gpu.sm_mhz.unwrap_or(0.0),
            gpu_mhz_valid: gpu.sm_mhz.is_some(),
            gpu_util_pct: gpu.util_pct.unwrap_or(0.0),
            cpu_util_pct: self.cpu_util.read_util_pct().unwrap_or(0.0),
            cpu_avg_mhz: cpu::avg_freq_mhz(&self.cpufreq_base).unwrap_or(0.0),
            resumed,
        }
    }

    /// Runs the 1 Hz loop on a dedicated thread until `shutdown` flips or a
    /// receiver disappears. Each sample is sent to every tx.
    pub fn spawn(
        mut self,
        txs: Vec<Sender<Event>>,
        shutdown: Arc<AtomicBool>,
    ) -> std::thread::JoinHandle<()> {
        std::thread::Builder::new()
            .name("sampler".into())
            .spawn(move || {
                while !shutdown.load(Ordering::Relaxed) {
                    let iter_start = Instant::now();
                    let sample = self.sample();
                    for tx in &txs {
                        if tx.send(Event::Sample(sample)).is_err() {
                            if shutdown.load(Ordering::Relaxed) {
                                tracing::debug!("sampler: receiver gone during shutdown, exiting");
                            } else {
                                tracing::warn!("sampler: receiver died unexpectedly, exiting");
                            }
                            return;
                        }
                    }
                    // Land iterations on a ~1 s cadence: sleep whatever the
                    // sensor reads left of the period, waking early on shutdown.
                    if let Some(remainder) = SAMPLE_PERIOD.checked_sub(iter_start.elapsed()) {
                        sleep_unless_shutdown(remainder, &shutdown);
                    }
                }
                tracing::debug!("sampler: shutdown flag set, exiting");
            })
            .expect("failed to spawn sampler thread")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn detects_monotonic_gap() {
        assert!(!is_resume_gap(1.0, 2.0)); // normal 1 s cadence
        assert!(is_resume_gap(1.0, 9.0)); // > 5 s gap => we slept
    }

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "bazerame-sampler-test-{}-{name}",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn add_chip(root: &Path, hwmon: &str, name: &str, files: &[(&str, &str)]) {
        let dir = root.join(hwmon);
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("name"), format!("{name}\n")).unwrap();
        for (file, contents) in files {
            fs::write(dir.join(file), format!("{contents}\n")).unwrap();
        }
    }

    /// hwmon fixture tree with fans + k10temp + amdgpu present.
    fn full_hwmon_fixture(name: &str) -> PathBuf {
        let root = fixture_dir(name);
        add_chip(&root, "hwmon0", "k10temp", &[("temp1_input", "49375")]);
        add_chip(
            &root,
            "hwmon1",
            "framework_laptop",
            &[("fan1_input", "1467"), ("fan2_input", "1452")],
        );
        add_chip(&root, "hwmon2", "amdgpu", &[("power1_average", "8041000")]);
        root
    }

    /// Sampler over a fixture hwmon root, everything else absent
    /// (rapl None, gpu None, nonexistent stat/cpufreq paths).
    fn fixture_sampler(hwmon_root: &Path) -> Sampler {
        Sampler::with_paths(
            hwmon_root,
            None,
            Path::new("/nonexistent/stat"),
            Path::new("/nonexistent/cpu-base"),
            None,
        )
    }

    #[test]
    fn sample_flattens_and_flags() {
        let root = full_hwmon_fixture("flatten");
        let mut sampler = fixture_sampler(&root);

        let s = sampler.sample();
        assert!(s.fan_valid);
        assert_eq!(s.fan1_rpm, 1467.0);
        assert_eq!(s.fan2_rpm, 1452.0);
        assert!(s.cpu_temp_valid);
        assert_eq!(s.cpu_temp_c, 49.375);
        assert!(!s.gpu_w_valid, "no NVML sensor => gpu_w invalid");
        assert!(!s.gpu_temp_valid, "no NVML sensor => gpu_temp invalid");
        assert!(!s.gpu_mhz_valid, "no NVML sensor => gpu_mhz invalid");
        assert_eq!(s.gpu_w, 0.0);
        assert_eq!(s.cpu_pkg_w, 0.0, "no RAPL => flattened to 0.0");
        assert!(
            (s.igpu_w - 8.041).abs() < 1e-9,
            "expected 8.041 W, got {}",
            s.igpu_w
        );
        assert!(!s.resumed, "first sample has no gap to detect");

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn sample_missing_hwmon_flags_false() {
        let root = fixture_dir("empty-hwmon");
        let mut sampler = fixture_sampler(&root);

        let s = sampler.sample();
        assert!(!s.fan_valid);
        assert_eq!(s.fan1_rpm, 0.0);
        assert_eq!(s.fan2_rpm, 0.0);
        assert!(!s.cpu_temp_valid);
        assert_eq!(s.cpu_temp_c, 0.0);

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn resumed_flag_set_on_gap() {
        let root = fixture_dir("resume-gap");
        let mut sampler = fixture_sampler(&root);

        assert!(!sampler.sample_at(1.0).resumed, "first sample: no baseline");
        assert!(!sampler.sample_at(2.0).resumed, "normal 1 s cadence");
        assert!(sampler.sample_at(9.0).resumed, "7 s gap => resumed");
        assert!(!sampler.sample_at(10.0).resumed, "back to normal cadence");

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn spawn_sends_samples_and_shuts_down() {
        let root = fixture_dir("spawn-smoke");
        let sampler = fixture_sampler(&root);

        let (tx, rx) = crossbeam_channel::unbounded();
        let shutdown = Arc::new(AtomicBool::new(false));
        let handle = sampler.spawn(vec![tx], Arc::clone(&shutdown));

        let event = rx
            .recv_timeout(Duration::from_secs(3))
            .expect("should receive a sample promptly");
        assert!(matches!(event, Event::Sample(_)));

        shutdown.store(true, Ordering::Relaxed);
        let flipped_at = Instant::now();
        handle.join().expect("sampler thread should not panic");
        assert!(
            flipped_at.elapsed() < Duration::from_millis(500),
            "thread must exit promptly after shutdown (took {:?}); the \
             inter-sample sleep must poll the flag, not sleep a full period",
            flipped_at.elapsed()
        );

        fs::remove_dir_all(&root).unwrap();
    }
}

//! 1 Hz sampler: owns all sensor structs, flattens their `Option` readings
//! into a dense [`Sample`] (None -> 0.0 + validity flags), and fans each
//! sample out to every subscriber channel from a dedicated thread.
//!
//! Since Task 14 (fwloop.9), the tick also reads the `cros_ec` replica and
//! AC-adapter presence directly (cheap sysfs reads, same cost class as the
//! sensors above) and *merges* -- never polls -- the fw-fanctrl view and the
//! NVMe temperature, both produced by their own background threads in
//! `sensors::poller`. See that module's doc comment for why those two stay
//! off this tick.

use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;

use crate::event::Event;
use crate::fanctrl::client::{FanctrlView, Freshness};
use crate::sensors::cpu::{self, CpuUtil};
use crate::sensors::ec::EcReading;
use crate::sensors::gpu::GpuSensor;
use crate::sensors::hwmon::{Hwmon, on_ac};
use crate::sensors::poller::{self, SharedFanctrl, SharedNvme};
use crate::sensors::rapl::RaplReader;
use crate::types::Sample;

/// Target sampling cadence.
const SAMPLE_PERIOD: Duration = Duration::from_secs(1);

/// The inter-sample sleep checks the shutdown flag at least this often, so
/// quitting never waits out a full sample period. `pub(crate)`: also used by
/// `sensors::poller`'s own two background threads, which want the same
/// shutdown responsiveness without duplicating this helper.
const SHUTDOWN_POLL: Duration = Duration::from_millis(250);

/// Sleeps `total` in slices of at most [`SHUTDOWN_POLL`], returning early
/// once `shutdown` flips.
pub(crate) fn sleep_unless_shutdown(total: Duration, shutdown: &AtomicBool) {
    let mut remaining = total;
    while !remaining.is_zero() && !shutdown.load(Ordering::Relaxed) {
        let slice = remaining.min(SHUTDOWN_POLL);
        std::thread::sleep(slice);
        remaining -= slice;
    }
}

/// Finds a hwmon chip directory by its `name` file's trimmed contents (the
/// same convention `Hwmon::discover` uses internally). `Hwmon` itself only
/// exposes named per-sensor getters, not a raw chip directory, and widening
/// its API is outside this task's file scope (`sensors/hwmon.rs` belongs to
/// an earlier task) -- `EcReading::read` (fwloop.3) wants the raw `cros_ec`
/// directory, so this is the narrow, local equivalent of that one scan.
fn find_chip_dir(hwmon_root: &Path, chip_name: &str) -> Option<PathBuf> {
    let entries = fs::read_dir(hwmon_root).ok()?;
    for entry in entries.flatten() {
        let dir = entry.path();
        if let Ok(name) = fs::read_to_string(dir.join("name")) {
            if name.trim() == chip_name {
                return Some(dir);
            }
        }
    }
    None
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
    /// `/sys/class/power_supply`-style root for `on_ac` (design doc §3.4).
    power_supply_root: PathBuf,
    /// The `cros_ec` chip directory, resolved once at construction (its
    /// `name` file doesn't change at runtime). `None` when the chip is
    /// absent -- every tick's `ec`/`ec_valid` then flattens accordingly,
    /// same shape as every other optional sensor here.
    cros_ec_dir: Option<PathBuf>,
    /// fw-fanctrl's last-known state, merged (never polled) from the
    /// background `FanctrlPoller` -- see `sensors::poller`.
    fanctrl: SharedFanctrl,
    /// NVMe last-good-value cache, merged (never read directly) from its own
    /// background thread -- see `sensors::poller`.
    nvme_cache: SharedNvme,
    /// `all_observed_at` of the previous tick's `fanctrl` view (`None`
    /// before any view has ever been observed), for `fanctrl_view_changed`.
    prev_all_observed_at: Option<Instant>,
    /// Epoch for `t_mono` (set at construction, i.e. process start).
    /// Linux `Instant` is CLOCK_BOOTTIME-backed since Rust 1.87 (pinned in
    /// Cargo.toml): it advances during suspend, which `is_resume_gap` needs.
    epoch: Instant,
    /// `t_mono` of the previous `sample()` call, for resume detection.
    prev_t: Option<f64>,
}

impl Sampler {
    /// Production constructor: real sysfs/procfs paths and NVML. `fanctrl`
    /// and `nvme_cache` are constructed and their background threads spawned
    /// by `main.rs` (poller construction/shutdown is that module's job, not
    /// the sampler's). Logs a warning once if the NVIDIA GPU is unavailable.
    pub fn new_system(fanctrl: SharedFanctrl, nvme_cache: SharedNvme) -> Self {
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
            Path::new("/sys/class/power_supply"),
            gpu,
            fanctrl,
            nvme_cache,
        )
    }

    /// Path-injectable constructor (tests pass fixture trees; `new_system`
    /// passes the real ones).
    #[allow(clippy::too_many_arguments)]
    pub fn with_paths(
        hwmon_root: &Path,
        rapl_base: Option<&Path>,
        stat_path: &Path,
        cpufreq_base: &Path,
        power_supply_root: &Path,
        gpu: Option<GpuSensor>,
        fanctrl: SharedFanctrl,
        nvme_cache: SharedNvme,
    ) -> Self {
        Self {
            hwmon: Hwmon::discover(hwmon_root),
            rapl: rapl_base.and_then(RaplReader::new),
            cpu_util: CpuUtil::new(stat_path),
            gpu,
            cpufreq_base: cpufreq_base.to_path_buf(),
            power_supply_root: power_supply_root.to_path_buf(),
            cros_ec_dir: find_chip_dir(hwmon_root, "cros_ec"),
            fanctrl,
            nvme_cache,
            prev_all_observed_at: None,
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
    /// `t_mono` (tests drive resume detection without sleeping). The same
    /// `t_mono` also derives the `Instant` used for `fanctrl`'s freshness and
    /// the NVMe cache's staleness check -- one source of truth, so tests can
    /// drive both deterministically through this single parameter instead of
    /// a second, independent clock (design doc §3.4: "the poller's own
    /// `Instant` drives only its cadence, never a freshness verdict").
    fn sample_at(&mut self, t_mono: f64) -> Sample {
        let resumed = self
            .prev_t
            .is_some_and(|prev_t| is_resume_gap(prev_t, t_mono));
        self.prev_t = Some(t_mono);
        let now = self.epoch + Duration::from_secs_f64(t_mono);

        let fans = self.hwmon.fan_rpms();
        let cpu_temp = self.hwmon.cpu_temp_c();
        // GpuReading::default() is all-None, so a missing sensor and a failed
        // read flatten identically (gpu_w_valid false).
        let gpu = self.gpu.as_ref().map(|g| g.read()).unwrap_or_default();

        let ec = self.cros_ec_dir.as_deref().and_then(EcReading::read);
        let ec_valid = ec.is_some();
        let on_ac_flag = on_ac(&self.power_supply_root).unwrap_or(false);
        let nvme_temp_c = poller::read_nvme(&self.nvme_cache, now);
        let (fanctrl, fanctrl_freshness, fanctrl_view_changed) = self.merge_fanctrl(now);

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
            ec,
            ec_valid,
            nvme_temp_c,
            fanctrl,
            fanctrl_freshness,
            fanctrl_view_changed,
            on_ac: on_ac_flag,
        }
    }

    /// Reads (never polls) the shared `fanctrl` handle: the current view,
    /// its freshness as of `now`, and whether this tick is the first one to
    /// observe a new `All` view since the previous tick (`Sample`'s doc
    /// comment on `fanctrl_view_changed` has the exact rule).
    fn merge_fanctrl(&mut self, now: Instant) -> (Option<FanctrlView>, Freshness, bool) {
        let guard = self.fanctrl.lock().expect("fanctrl source mutex poisoned");
        let view = guard.view().cloned();
        let freshness = guard.freshness(now);
        drop(guard);
        let all_observed_at = view.as_ref().and_then(|v| v.all_observed_at);
        let changed = all_observed_at != self.prev_all_observed_at;
        self.prev_all_observed_at = all_observed_at;
        (view, freshness, changed)
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
                    // Sample is Clone but no longer Copy (it now owns an
                    // EcReading/FanctrlView, each carrying a Vec/String) --
                    // every subscriber needs its own clone.
                    for tx in &txs {
                        if tx.send(Event::Sample(sample.clone())).is_err() {
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
    use crate::fanctrl::client::{FanctrlSource, Freshness, PrintCommand};
    use crate::sensors::poller::{FanctrlPoller, spawn_nvme_poller};
    use crate::test_support::fakes::{FakeFanctrl, ScriptedOutcome};
    use std::fs;
    use std::sync::Mutex;
    use std::sync::mpsc;

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

    /// An empty `FakeFanctrl` behind the shared handle `Sampler`/`FanctrlPoller`
    /// expect -- no view, no scripted outcomes, freshness `Stale`.
    fn empty_fanctrl() -> SharedFanctrl {
        Arc::new(Mutex::new(
            Box::new(FakeFanctrl::new()) as Box<dyn FanctrlSource + Send>
        ))
    }

    fn empty_nvme_cache() -> SharedNvme {
        Arc::new(Mutex::new(None))
    }

    /// Sampler over a fixture hwmon root, everything else absent (rapl None,
    /// gpu None, nonexistent stat/cpufreq/power-supply paths, an empty fake
    /// fanctrl source, an empty NVMe cache).
    fn fixture_sampler(hwmon_root: &Path) -> Sampler {
        fixture_sampler_with(hwmon_root, empty_fanctrl(), empty_nvme_cache())
    }

    /// Same as `fixture_sampler`, but with caller-supplied fanctrl/NVMe
    /// handles -- for tests that drive those sources themselves.
    fn fixture_sampler_with(
        hwmon_root: &Path,
        fanctrl: SharedFanctrl,
        nvme_cache: SharedNvme,
    ) -> Sampler {
        Sampler::with_paths(
            hwmon_root,
            None,
            Path::new("/nonexistent/stat"),
            Path::new("/nonexistent/cpu-base"),
            Path::new("/nonexistent/power-supply"),
            None,
            fanctrl,
            nvme_cache,
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
        // No cros_ec chip in this fixture, an empty fake fanctrl source, and
        // a nonexistent power-supply root: the new fields must flatten the
        // same way the pre-existing ones do on a missing/absent source.
        assert!(!s.ec_valid, "no cros_ec chip in this fixture");
        assert_eq!(s.ec, None);
        assert_eq!(s.fanctrl, None, "empty fake fanctrl source: no view yet");
        assert!(!s.fanctrl_view_changed);
        assert_eq!(s.nvme_temp_c, None, "no nvme cache entry yet");
        assert!(!s.on_ac, "power-supply root does not exist");

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

    // --- Step 1 (EC/AC) + step 4 (fanctrl_view_changed) -------------------

    #[test]
    fn ec_and_on_ac_merge_into_sample_each_tick() {
        let root = fixture_dir("ec-and-ac");
        add_chip(
            &root,
            "hwmon3",
            "cros_ec",
            &[("temp1_label", "cpu@4c"), ("temp1_input", "50000")],
        );
        let power_supply_root = fixture_dir("ec-and-ac-power-supply");
        let acad = power_supply_root.join("ACAD");
        fs::create_dir_all(&acad).unwrap();
        fs::write(acad.join("online"), "1\n").unwrap();

        let mut sampler = Sampler::with_paths(
            &root,
            None,
            Path::new("/nonexistent/stat"),
            Path::new("/nonexistent/cpu-base"),
            &power_supply_root,
            None,
            empty_fanctrl(),
            empty_nvme_cache(),
        );
        let s = sampler.sample();
        assert!(s.ec_valid);
        let ec =
            s.ec.expect("cros_ec chip present with one positive reading");
        assert_eq!(ec.max_c, 50);
        assert_eq!(ec.argmax.as_str(), "cpu@4c");
        assert!(s.on_ac, "ACAD/online = 1");

        fs::remove_dir_all(&root).unwrap();
        fs::remove_dir_all(&power_supply_root).unwrap();
    }

    fn all_outcome(speed_pct: u8) -> ScriptedOutcome {
        ScriptedOutcome::All {
            strategy: "quiet16".to_string(),
            active: true,
            speed_pct,
            temperature: 75.0,
            ma_temperature: 75.0,
            ma_interval: 60,
            curve: vec![(0.0, 15), (95.0, 100)],
        }
    }

    #[test]
    fn fanctrl_view_changed_true_once_per_new_all_view_only() {
        // Scripted BEFORE boxing: once behind `Box<dyn FanctrlSource>` the
        // fake can only be driven through the trait (`poll`/`view`/
        // `freshness`), not re-scripted -- so every outcome this test's own
        // `poll` calls below will consume must be queued up front, in call
        // order: All, Speed, Speed, All (four calls total, interleaved with
        // sampler ticks that never themselves poll).
        let mut fake = FakeFanctrl::new();
        fake.script(all_outcome(31)); // first All
        fake.script(ScriptedOutcome::Speed(32)); // speed-only refresh
        fake.script(ScriptedOutcome::Speed(33)); // another speed-only refresh
        fake.script(all_outcome(40)); // second, new All view
        let source: SharedFanctrl =
            Arc::new(Mutex::new(Box::new(fake) as Box<dyn FanctrlSource + Send>));

        let mut sampler = fixture_sampler_with(
            Path::new("/nonexistent/hwmon"),
            Arc::clone(&source),
            empty_nvme_cache(),
        );

        // No poll yet: no view, so nothing has "changed".
        let s0 = sampler.sample_at(0.0);
        assert!(s0.fanctrl.is_none());
        assert!(!s0.fanctrl_view_changed);

        // First All lands between t=0 and t=1.
        source
            .lock()
            .unwrap()
            .poll(PrintCommand::All, Instant::now())
            .unwrap();
        let s1 = sampler.sample_at(1.0);
        assert!(s1.fanctrl.is_some());
        assert!(s1.fanctrl_view_changed, "first All view must flip changed");

        // Two Speed-only refreshes in a row must never set it.
        source
            .lock()
            .unwrap()
            .poll(PrintCommand::Speed, Instant::now())
            .unwrap();
        assert!(!sampler.sample_at(2.0).fanctrl_view_changed);
        source
            .lock()
            .unwrap()
            .poll(PrintCommand::Speed, Instant::now())
            .unwrap();
        assert!(!sampler.sample_at(3.0).fanctrl_view_changed);

        // A second All view flips it again, exactly once.
        source
            .lock()
            .unwrap()
            .poll(PrintCommand::All, Instant::now())
            .unwrap();
        assert!(sampler.sample_at(4.0).fanctrl_view_changed);
        assert!(
            !sampler.sample_at(5.0).fanctrl_view_changed,
            "must not still read true on the tick after the flip"
        );
    }

    // --- Step 5: the NVMe isolation test -----------------------------------

    #[test]
    fn nvme_blocking_read_does_not_stall_sampler_or_fanctrl_poller() {
        // Synchronization instead of a real 60s sleep: the "NVMe read"
        // blocks on this channel until the test releases it, deterministically
        // modelling the SMART admin command's worst case (a 60s kernel
        // `admin_timeout`) without either a flaky or a slow test.
        let (release_tx, release_rx) = mpsc::channel::<()>();
        let read_entered = Arc::new(AtomicBool::new(false));
        let read_entered2 = Arc::clone(&read_entered);
        let read = move || {
            read_entered2.store(true, Ordering::Relaxed);
            let _ = release_rx.recv();
            Some(55.0)
        };

        let shutdown = Arc::new(AtomicBool::new(false));
        let nvme_cache = empty_nvme_cache();
        let nvme_thread = spawn_nvme_poller(read, Arc::clone(&nvme_cache), Arc::clone(&shutdown));

        // A fanctrl source that always succeeds, so freshness reads Fresh
        // and the poller's own command log keeps growing while nvme is
        // stuck.
        let mut fake = FakeFanctrl::new();
        for _ in 0..8 {
            fake.script(all_outcome(31));
        }
        let fanctrl_source: SharedFanctrl =
            Arc::new(Mutex::new(Box::new(fake) as Box<dyn FanctrlSource + Send>));
        let poller = FanctrlPoller::new(Arc::clone(&fanctrl_source), Instant::now());
        let poller_thread = poller.spawn(Arc::clone(&shutdown));

        // Bounded wait for both: the nvme thread stuck inside its blocking
        // read, and the fanctrl poller's first (seeding) poll landed.
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            let nvme_ready = read_entered.load(Ordering::Relaxed);
            let fanctrl_ready = fanctrl_source.lock().unwrap().view().is_some();
            if nvme_ready && fanctrl_ready {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "setup did not converge in time (nvme_ready={nvme_ready}, fanctrl_ready={fanctrl_ready})"
            );
            std::thread::sleep(Duration::from_millis(5));
        }

        // While the nvme thread is genuinely stuck: sampling must not block
        // (this call itself would hang the test if the nvme read were ever
        // reachable from the sampler tick), nvme_temp_c must surface as
        // None (nothing has ever been published), and fanctrl freshness must
        // read Fresh -- proving the fanctrl poller's own cadence was never
        // touched by the stuck nvme thread either.
        let hwmon_root = fixture_dir("nvme-isolation");
        let mut sampler = fixture_sampler_with(
            &hwmon_root,
            Arc::clone(&fanctrl_source),
            Arc::clone(&nvme_cache),
        );
        for i in 0..3 {
            let s = sampler.sample_at(f64::from(i));
            assert_eq!(
                s.nvme_temp_c, None,
                "nvme thread is still blocked: no last-good value published yet"
            );
            assert_eq!(
                s.fanctrl_freshness,
                Freshness::Fresh,
                "fanctrl must stay Fresh, unaffected by the blocked nvme read"
            );
        }

        release_tx
            .send(())
            .expect("nvme thread should still be listening");
        shutdown.store(true, Ordering::Relaxed);
        nvme_thread
            .join()
            .expect("nvme poller thread should not panic");
        poller_thread
            .join()
            .expect("fanctrl poller thread should not panic");
        fs::remove_dir_all(&hwmon_root).unwrap();
    }
}

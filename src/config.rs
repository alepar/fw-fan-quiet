//! User configuration (TOML, `/etc/bazerame-fans/config.toml` in production,
//! `--config` overridable). Loading NEVER crashes: a missing file means
//! defaults, a corrupt file means defaults plus a warning — the daemon must
//! come up and manage fans regardless of config state. `#[serde(default)]`
//! makes every field individually optional, so a partial file overrides only
//! what it names and unknown fields are ignored (serde's default behavior).

use std::io::Write;
use std::path::{Path, PathBuf};

use crate::actuators::gpu::clamp_gpu_clock;
use crate::control::allocator::CPU_MAX_W;
use crate::control::controller::DEFAULT_FAN_TARGET_RPM;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Config {
    /// Steady-state fan RPM the allocator holds the machine at.
    pub fan_target_rpm: f64,
    /// CPU sustained-watts floor: never allocate below this.
    pub cpu_floor_w: f64,
    /// GPU locked-clock floor (MHz): never lock below this.
    pub gpu_floor_mhz: u32,
    /// CPU fast (short-burst) PPT limit handed to ryzenadj, milliwatts.
    pub fast_limit_mw: u32,
    /// Online RLS slope adaptation in Auto mode. OFF by default: the
    /// 2026-06/07 field sessions found the distrust configuration — RLS
    /// frozen, trim-only adaptation — gave the best control behavior of the
    /// whole evening, while live slope adaptation double-corrected against
    /// the trim and was what walked `e` into the degenerate contour-divisor
    /// incident. Calibrated shape + bounded trim + fan feedback is the
    /// robust configuration; set true to experiment with live adaptation.
    pub online_rls: bool,
    /// LED matrix wattage display (`[leds]` table). Optional feature; its own
    /// `enabled` flag defaults on but a missing/failed module just stays dark.
    pub leds: LedConfig,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            fan_target_rpm: DEFAULT_FAN_TARGET_RPM,
            cpu_floor_w: 15.0,
            gpu_floor_mhz: 1000,
            fast_limit_mw: 53_000,
            online_rls: false,
            leds: LedConfig::default(),
        }
    }
}

/// Configuration for the two Framework 16 LED Matrix modules (`[leds]`).
///
/// The two modules ship with an identical USB serial number, so they are
/// addressed by their stable `by-path` USB-topology symlink rather than by
/// serial. Defaults match THIS machine's bays (verified by lighting each
/// panel): the left bay drives the CPU gauge, the right bay the GPU gauge. If
/// the modules are ever swapped between bays, set `cpu_port`/`gpu_port`.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct LedConfig {
    /// Master switch for the whole LED feature.
    pub enabled: bool,
    /// Serial device for the CPU (left) gauge.
    pub cpu_port: String,
    /// Serial device for the GPU (right) gauge.
    pub gpu_port: String,
    /// Watts that fill the CPU panel to the top.
    pub cpu_full_scale_w: f64,
    /// Watts that fill the GPU panel to the top.
    pub gpu_full_scale_w: f64,
    /// Global PWM brightness sent to both modules (0-255).
    pub brightness: u8,
    /// Reverse the time axis (set if newest ends up at the bottom).
    pub flip_time: bool,
    /// Reverse the CPU (left) panel's wattage-bar growth direction.
    pub cpu_flip_watts: bool,
    /// Reverse the GPU (right) panel's wattage-bar growth direction.
    pub gpu_flip_watts: bool,
}

impl Default for LedConfig {
    fn default() -> Self {
        LedConfig {
            enabled: true,
            cpu_port: "/dev/serial/by-path/pci-0000:c4:00.0-usb-0:4.2:1.0".to_string(),
            gpu_port: "/dev/serial/by-path/pci-0000:c4:00.0-usb-0:3.3:1.0".to_string(),
            cpu_full_scale_w: 60.0,
            gpu_full_scale_w: 100.0,
            brightness: 100,
            // Within a column, LED index 0 is the panel's physical top
            // (calibrated on this machine), so newest-on-top needs no time
            // flip. Wattage bars grow "inside out" — from each panel's inner
            // edge nearest the keyboard: the left (CPU) panel anchors on its
            // right/inner edge (flipped), the right (GPU) panel on its
            // left/inner edge (unflipped). Verified live on this machine.
            flip_time: false,
            cpu_flip_watts: true,
            gpu_flip_watts: false,
        }
    }
}

impl Config {
    /// Load from `path`. Missing file → defaults (info log); unreadable or
    /// unparseable file → defaults + warning. NEVER crashes on bad config.
    pub fn load(path: &Path) -> Config {
        let text = match std::fs::read_to_string(path) {
            Ok(text) => text,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                tracing::info!("no config at {}, using defaults", path.display());
                return Config::default();
            }
            Err(e) => {
                tracing::warn!("cannot read config {}, using defaults: {e}", path.display());
                return Config::default();
            }
        };
        match toml::from_str::<Config>(&text) {
            Ok(config) => config.sanitized(),
            Err(e) => {
                tracing::warn!("bad config {}, using defaults: {e}", path.display());
                Config::default()
            }
        }
    }

    /// Clamp out-of-range floors to the hardware envelope, warning when it
    /// bites. A bad config value must degrade to a sane floor, never panic
    /// the control loop downstream (`f64::clamp` with min > max panics; the
    /// allocator debug-asserts its floor range). Applied on load AND again
    /// at controller construction (belt and suspenders for programmatic
    /// configs).
    pub fn sanitized(mut self) -> Self {
        let gpu = clamp_gpu_clock(self.gpu_floor_mhz);
        if gpu != self.gpu_floor_mhz {
            tracing::warn!(
                "config gpu_floor_mhz {} outside the actuator range; clamped to {gpu}",
                self.gpu_floor_mhz
            );
            self.gpu_floor_mhz = gpu;
        }
        // Non-finite (TOML can encode nan/inf) would survive clamp() as NaN:
        // fall back to the default floor instead.
        let cpu = if self.cpu_floor_w.is_finite() {
            self.cpu_floor_w.clamp(0.0, CPU_MAX_W)
        } else {
            Config::default().cpu_floor_w
        };
        if cpu != self.cpu_floor_w {
            tracing::warn!(
                "config cpu_floor_w {} outside [0, {CPU_MAX_W}]; clamped to {cpu}",
                self.cpu_floor_w
            );
            self.cpu_floor_w = cpu;
        }
        self
    }

    /// Atomic save: write `<path>.tmp`, then rename over `path`. Creates the
    /// parent directory if needed.
    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        let text = toml::to_string_pretty(self).map_err(std::io::Error::other)?;
        write_atomic(path, text.as_bytes())
    }
}

/// Atomic file write shared by config (TOML) and state (JSON): write the
/// full contents to `<path>.tmp`, fsync, then rename over `path` — a crash
/// mid-write leaves the old file intact, never a truncated one. Creates the
/// parent directory first.
pub fn write_atomic(path: &Path, contents: &[u8]) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    let mut file = std::fs::File::create(&tmp)?;
    file.write_all(contents)?;
    // fsync BEFORE the rename: otherwise the rename can hit disk before the
    // data does and a power cut leaves an empty file at the final path.
    file.sync_all()?;
    std::fs::rename(&tmp, path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "bazerame-config-test-{}-{name}",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn roundtrip_save_load() {
        let dir = fixture_dir("roundtrip");
        let path = dir.join("config.toml");
        let config = Config {
            fan_target_rpm: 2600.0,
            cpu_floor_w: 12.0,
            gpu_floor_mhz: 1200,
            fast_limit_mw: 60_000,
            online_rls: true,
            leds: LedConfig {
                enabled: false,
                cpu_port: "/dev/ttyACM9".to_string(),
                gpu_port: "/dev/ttyACM8".to_string(),
                cpu_full_scale_w: 45.0,
                gpu_full_scale_w: 90.0,
                brightness: 200,
                flip_time: true,
                cpu_flip_watts: false,
                gpu_flip_watts: true,
            },
        };
        config.save(&path).unwrap();
        assert_eq!(Config::load(&path), config);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn floors_out_of_range_are_clamped_on_load() {
        let dir = fixture_dir("clamp-floors");
        let path = dir.join("config.toml");
        // Above the hardware envelope: an unclamped 4000 MHz floor would
        // panic the GPU PI's f64::clamp (min > max) on the first Auto tick.
        fs::write(&path, "gpu_floor_mhz = 4000\ncpu_floor_w = 99.0\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.gpu_floor_mhz, 3090);
        assert_eq!(config.cpu_floor_w, 54.0);
        // Below it: floors clamp up to the actuator minimum / zero.
        fs::write(&path, "gpu_floor_mhz = 100\ncpu_floor_w = -5.0\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.gpu_floor_mhz, 1000);
        assert_eq!(config.cpu_floor_w, 0.0);
        // Non-finite cpu floor (TOML encodes nan) falls back to the default.
        fs::write(&path, "cpu_floor_w = nan\n").unwrap();
        assert_eq!(
            Config::load(&path).cpu_floor_w,
            Config::default().cpu_floor_w
        );
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn missing_file_gives_defaults() {
        let dir = fixture_dir("missing");
        let config = Config::load(&dir.join("nope.toml"));
        assert_eq!(config, Config::default());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn corrupt_file_gives_defaults_no_panic() {
        let dir = fixture_dir("corrupt");
        let path = dir.join("config.toml");
        fs::write(&path, "not toml!").unwrap();
        assert_eq!(Config::load(&path), Config::default());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn partial_file_overrides_only_named_fields() {
        let dir = fixture_dir("partial");
        let path = dir.join("config.toml");
        fs::write(&path, "fan_target_rpm = 2500\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.fan_target_rpm, 2500.0);
        let defaults = Config::default();
        assert_eq!(config.cpu_floor_w, defaults.cpu_floor_w);
        assert_eq!(config.gpu_floor_mhz, defaults.gpu_floor_mhz);
        assert_eq!(config.fast_limit_mw, defaults.fast_limit_mw);
        assert_eq!(config.online_rls, defaults.online_rls);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn unknown_fields_tolerated() {
        let dir = fixture_dir("unknown");
        let path = dir.join("config.toml");
        fs::write(&path, "fan_target_rpm = 2500\nfuture_knob = true\n").unwrap();
        assert_eq!(Config::load(&path).fan_target_rpm, 2500.0);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn save_creates_parent_dir() {
        let dir = fixture_dir("parents");
        let path = dir.join("a/b/config.toml");
        Config::default().save(&path).unwrap();
        assert_eq!(Config::load(&path), Config::default());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn save_over_existing_is_atomic_and_leaves_no_tmp() {
        let dir = fixture_dir("atomic");
        let path = dir.join("config.toml");
        Config::default().save(&path).unwrap();
        let updated = Config {
            fan_target_rpm: 2000.0,
            ..Config::default()
        };
        updated.save(&path).unwrap();
        assert_eq!(Config::load(&path), updated);
        // No .tmp (or any other stray file) left behind.
        let names: Vec<String> = fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        assert_eq!(names, vec!["config.toml".to_string()]);
        fs::remove_dir_all(&dir).unwrap();
    }
}

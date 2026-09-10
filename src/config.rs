//! User configuration (TOML, `/etc/bazerame-fans/config.toml` in production,
//! `--config` overridable). Loading NEVER crashes: a missing file means
//! defaults, a corrupt file means defaults plus a warning — the daemon must
//! come up and manage fans regardless of config state. `#[serde(default)]`
//! makes every field individually optional, so a partial file overrides only
//! what it names and unknown fields are ignored (serde's default behavior).

use std::io::Write;
use std::path::{Path, PathBuf};

use crate::actuators::gpu::clamp_gpu_clock;
use crate::control::allocator::{CPU_MAX_W, GPU_MAX_W};
use crate::control::controller::DEFAULT_FAN_TARGET_RPM;
use crate::control::guards::{GPU_HOT_C_DEFAULT, GPU_HYSTERESIS_C, NVME_HOT_C_DEFAULT};
use crate::control::watchdog::GPU_TRIP_C;

/// Lower bound for `cpu_max_w`: below the actuator's ~10 W sustained floor the
/// grid-search and the manual-mode clamps would degenerate.
const CPU_MAX_W_FLOOR: f64 = 10.0;

/// Lower bound for `gpu_hot_c`. The card idles in the 40s and cruises in the
/// 60s–70s under load, so an enter threshold at or below that would latch the
/// soft guard permanently hot and ratchet the GPU share to its floor forever
/// (the guard only clears at `enter − GPU_HYSTERESIS_C`).
const GPU_HOT_C_FLOOR: f64 = 60.0;
/// Upper bound for `gpu_hot_c`: the soft guard must get its ratchet-down turn
/// BEFORE the hard thermal watchdog trips, so the enter threshold stays
/// strictly below [`GPU_TRIP_C`] — and by at least the hysteresis band, so the
/// guard's *exit* is meaningful rather than sitting above the trip point.
const GPU_HOT_C_CEIL: f64 = GPU_TRIP_C - GPU_HYSTERESIS_C;
/// `nvme_hot_c` bounds. Reporting-only guard, so the range only has to keep
/// the flag from being stuck on (drives idle in the 30s–40s) or unreachable
/// (consumer NVMe throttles in the 80s and its own critical is ~90).
const NVME_HOT_C_FLOOR: f64 = 50.0;
const NVME_HOT_C_CEIL: f64 = 90.0;

/// Default fw-fanctrl `AF_UNIX` command socket (design doc §2.1 / research
/// doc §"The socket").
const DEFAULT_FANCTRL_SOCKET: &str = "/run/fw-fanctrl/.fw-fanctrl.commands.sock";

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
    // Unknown keys are always ignored (`serde(default)` without
    // `deny_unknown_fields`), so old config files never fail to load just
    // because a key was removed. Two real examples that must keep loading:
    // `online_rls` (removed 2026-07, adaptation v2 — the 2-state Kalman
    // filter owns Auto-mode adaptation, and full-surface RLS was
    // field-disabled after the degenerate-divisor incident) and
    // `nvme_boost_rpm` (never shipped — an early NVMe-guard design that
    // raised the fan target for a hot drive, dropped per §2.8/§Facts:
    // raising the target raises T* and therefore the CPU/GPU budget,
    // injecting more heat into a scenario measured to have nothing to raise
    // it for).
    /// CPU sustained operating max (watts): the single source of truth for the
    /// "100%" CPU power. The allocator grid-searches up to it, the CPU actuator
    /// clamps commanded sustained power to it, and the TUI/LED displays scale by
    /// it. Defaults to and is clamped to the HX 370 cTDP ceiling
    /// ([`CPU_MAX_W`]); lower it to soft-cap the CPU (quieter, less power).
    pub cpu_max_w: f64,
    /// GPU operating max (watts): same role for the GPU. Defaults to / clamped
    /// to the RTX 5070 module TGP ([`GPU_MAX_W`]).
    pub gpu_max_w: f64,
    /// dGPU guard enter threshold (°C, exit is this − 5). See
    /// [`crate::control::guards`] for the hysteresis and the 87 °C
    /// card-spec derivation of the default.
    pub gpu_hot_c: f64,
    /// NVMe guard enter threshold (°C, exit is this − 5). Reporting-only —
    /// see [`crate::control::guards`].
    pub nvme_hot_c: f64,
    /// LED matrix wattage display (`[leds]` table). Optional feature; its own
    /// `enabled` flag defaults on but a missing/failed module just stays dark.
    pub leds: LedConfig,
    /// `AF_UNIX` socket path for the fw-fanctrl client (design doc §2.1).
    /// This binary never writes to it — see `fanctrl::client`'s read-only
    /// `PrintCommand`.
    pub fanctrl_socket: PathBuf,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            fan_target_rpm: DEFAULT_FAN_TARGET_RPM,
            cpu_floor_w: 15.0,
            gpu_floor_mhz: 1000,
            fast_limit_mw: 53_000,
            cpu_max_w: CPU_MAX_W,
            gpu_max_w: GPU_MAX_W,
            gpu_hot_c: GPU_HOT_C_DEFAULT,
            nvme_hot_c: NVME_HOT_C_DEFAULT,
            leds: LedConfig::default(),
            fanctrl_socket: PathBuf::from(DEFAULT_FANCTRL_SOCKET),
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
        // Operating maxes are the source of truth for CPU/GPU "100%", but they
        // are hard-clamped to the hardware ceilings so a config typo can never
        // push the allocator/actuator past the silicon (CPU) or module (GPU)
        // limit. Non-finite (TOML nan/inf) falls back to the ceiling default.
        let cpu_max = if self.cpu_max_w.is_finite() {
            self.cpu_max_w.clamp(CPU_MAX_W_FLOOR, CPU_MAX_W)
        } else {
            CPU_MAX_W
        };
        if cpu_max != self.cpu_max_w {
            tracing::warn!(
                "config cpu_max_w {} outside [{CPU_MAX_W_FLOOR}, {CPU_MAX_W}]; clamped to {cpu_max}",
                self.cpu_max_w
            );
            self.cpu_max_w = cpu_max;
        }
        let gpu_max = if self.gpu_max_w.is_finite() {
            self.gpu_max_w.clamp(1.0, GPU_MAX_W)
        } else {
            GPU_MAX_W
        };
        if gpu_max != self.gpu_max_w {
            tracing::warn!(
                "config gpu_max_w {} outside [1, {GPU_MAX_W}]; clamped to {gpu_max}",
                self.gpu_max_w
            );
            self.gpu_max_w = gpu_max;
        }
        // Floor is bounded by the (now-sanitized) operating max, not the raw
        // ceiling: floor ≤ cpu_max_w keeps the allocator's floor-in-range
        // debug-assert and grid honest. Non-finite falls back to the default.
        let cpu = if self.cpu_floor_w.is_finite() {
            self.cpu_floor_w.clamp(0.0, self.cpu_max_w)
        } else {
            Config::default().cpu_floor_w.min(self.cpu_max_w)
        };
        if cpu != self.cpu_floor_w {
            tracing::warn!(
                "config cpu_floor_w {} outside [0, {}]; clamped to {cpu}",
                self.cpu_floor_w,
                self.cpu_max_w
            );
            self.cpu_floor_w = cpu;
        }
        // The two guard thresholds (§2.8). NaN is the sharp edge here: every
        // comparison in `guards::hysteresis` is false against NaN, so a
        // `gpu_hot_c = nan` silently disables the dGPU guard entirely with no
        // flag and no log. Below the floor latches the guard permanently hot;
        // at or above GPU_TRIP_C the hard watchdog's emergency release fires
        // before the soft ratchet ever gets a turn.
        let gpu_hot = if self.gpu_hot_c.is_finite() {
            self.gpu_hot_c.clamp(GPU_HOT_C_FLOOR, GPU_HOT_C_CEIL)
        } else {
            GPU_HOT_C_DEFAULT
        };
        if gpu_hot != self.gpu_hot_c {
            tracing::warn!(
                "config gpu_hot_c {} outside [{GPU_HOT_C_FLOOR}, {GPU_HOT_C_CEIL}] \
                 (must stay below the {GPU_TRIP_C} °C hard trip); clamped to {gpu_hot}",
                self.gpu_hot_c
            );
            self.gpu_hot_c = gpu_hot;
        }
        let nvme_hot = if self.nvme_hot_c.is_finite() {
            self.nvme_hot_c.clamp(NVME_HOT_C_FLOOR, NVME_HOT_C_CEIL)
        } else {
            NVME_HOT_C_DEFAULT
        };
        if nvme_hot != self.nvme_hot_c {
            tracing::warn!(
                "config nvme_hot_c {} outside [{NVME_HOT_C_FLOOR}, {NVME_HOT_C_CEIL}]; \
                 clamped to {nvme_hot}",
                self.nvme_hot_c
            );
            self.nvme_hot_c = nvme_hot;
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
            cpu_max_w: 50.0,
            gpu_max_w: 90.0,
            gpu_hot_c: 86.0,
            nvme_hot_c: 78.0,
            leds: LedConfig {
                enabled: false,
                cpu_port: "/dev/ttyACM9".to_string(),
                gpu_port: "/dev/ttyACM8".to_string(),
                brightness: 200,
                flip_time: true,
                cpu_flip_watts: false,
                gpu_flip_watts: true,
            },
            fanctrl_socket: PathBuf::from("/run/fw-fanctrl/custom.sock"),
        };
        config.save(&path).unwrap();
        assert_eq!(Config::load(&path), config);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn fanctrl_socket_defaults_and_round_trips() {
        assert_eq!(
            Config::default().fanctrl_socket,
            PathBuf::from("/run/fw-fanctrl/.fw-fanctrl.commands.sock")
        );

        let dir = fixture_dir("fanctrl-socket");
        let path = dir.join("config.toml");
        fs::write(&path, "fanctrl_socket = \"/tmp/alt.sock\"\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.fanctrl_socket, PathBuf::from("/tmp/alt.sock"));
        // A file that doesn't name the key at all still gets the default —
        // this is the exact shape a legacy config.toml predating this key
        // is in.
        fs::write(&path, "fan_target_rpm = 2500\n").unwrap();
        assert_eq!(
            Config::load(&path).fanctrl_socket,
            Config::default().fanctrl_socket
        );
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
    fn operating_maxes_clamp_to_hardware_ceilings() {
        let dir = fixture_dir("clamp-maxes");
        let path = dir.join("config.toml");
        // Above the silicon/module ceilings: clamped down so the allocator and
        // actuator can never be told to exceed the hardware.
        fs::write(&path, "cpu_max_w = 999.0\ngpu_max_w = 999.0\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.cpu_max_w, 54.0);
        assert_eq!(config.gpu_max_w, 100.0);
        // A lowered cpu_max also lowers the floor's ceiling: floor can't exceed
        // the operating max (else the allocator's floor-in-range assert trips).
        fs::write(&path, "cpu_max_w = 30.0\ncpu_floor_w = 40.0\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.cpu_max_w, 30.0);
        assert_eq!(config.cpu_floor_w, 30.0, "floor clamped to cpu_max_w");
        // Non-finite maxes fall back to the ceilings.
        fs::write(&path, "cpu_max_w = nan\ngpu_max_w = inf\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.cpu_max_w, 54.0);
        assert_eq!(config.gpu_max_w, 100.0);
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
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn unknown_keys_are_ignored() {
        let dir = fixture_dir("unknown");
        let path = dir.join("config.toml");
        // Three unknown-key shapes that must all still load: `online_rls` is
        // a REAL removed legacy key (adaptation v2), `nvme_boost_rpm` is a
        // key that was designed and then dropped before ever shipping (the
        // rejected NVMe-guard target-raise, §2.8), and `totally_made_up_key`
        // stands in for any future removal or typo — none of them should be
        // able to fail a load.
        fs::write(
            &path,
            "fan_target_rpm = 2500\n\
             online_rls = true\n\
             nvme_boost_rpm = 500\n\
             totally_made_up_key = \"whatever\"\n",
        )
        .unwrap();
        let config = Config::load(&path);
        assert_eq!(config.fan_target_rpm, 2500.0);
        // Loading survived AND fell through to real defaults for everything
        // the file didn't name — a config that silently zeroed unnamed
        // fields on an unknown key would also pass the line above.
        assert_eq!(config.gpu_hot_c, Config::default().gpu_hot_c);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn guard_thresholds_default_and_round_trip() {
        let dir = fixture_dir("guard-thresholds");
        let path = dir.join("config.toml");
        let defaults = Config::default();
        assert_eq!(defaults.gpu_hot_c, 88.0);
        assert_eq!(defaults.nvme_hot_c, 80.0);
        let config = Config {
            gpu_hot_c: 86.0,
            nvme_hot_c: 77.0,
            ..Config::default()
        };
        config.save(&path).unwrap();
        assert_eq!(Config::load(&path), config);
        fs::remove_dir_all(&dir).unwrap();
    }

    // --- finding 8: gpu_hot_c / nvme_hot_c go through sanitized() too -----

    #[test]
    fn nan_gpu_hot_c_falls_back_to_the_default_instead_of_disabling_the_guard() {
        // Every comparison in `guards::hysteresis` is false against NaN, so
        // an unsanitized NaN silently switches the dGPU guard off for good.
        let config = Config {
            gpu_hot_c: f64::NAN,
            ..Config::default()
        }
        .sanitized();
        assert_eq!(config.gpu_hot_c, GPU_HOT_C_DEFAULT);
        // Same for +inf, which is finite-looking in neither sense.
        let config = Config {
            gpu_hot_c: f64::INFINITY,
            ..Config::default()
        }
        .sanitized();
        assert_eq!(config.gpu_hot_c, GPU_HOT_C_DEFAULT);
    }

    #[test]
    fn gpu_hot_c_at_or_below_idle_is_raised_to_the_floor() {
        // Below the card's cruising range the guard would latch hot forever
        // and ratchet the GPU share to its floor for the whole session.
        for value in [-10.0, 0.0, 45.0, 59.9] {
            let config = Config {
                gpu_hot_c: value,
                ..Config::default()
            }
            .sanitized();
            assert_eq!(config.gpu_hot_c, GPU_HOT_C_FLOOR, "gpu_hot_c = {value}");
        }
    }

    #[test]
    fn gpu_hot_c_stays_strictly_below_the_hard_trip_with_hysteresis_margin() {
        for value in [90.0, GPU_TRIP_C, 120.0] {
            let config = Config {
                gpu_hot_c: value,
                ..Config::default()
            }
            .sanitized();
            assert_eq!(config.gpu_hot_c, GPU_HOT_C_CEIL, "gpu_hot_c = {value}");
            assert!(
                config.gpu_hot_c < GPU_TRIP_C,
                "the soft guard must get its turn before the hard watchdog"
            );
            assert!(
                config.gpu_hot_c + GPU_HYSTERESIS_C <= GPU_TRIP_C,
                "the guard's exit band must fit under the trip point"
            );
        }
    }

    #[test]
    fn nvme_hot_c_is_sanitized_the_same_way() {
        let nan = Config {
            nvme_hot_c: f64::NAN,
            ..Config::default()
        }
        .sanitized();
        assert_eq!(nan.nvme_hot_c, NVME_HOT_C_DEFAULT);
        let low = Config {
            nvme_hot_c: 10.0,
            ..Config::default()
        }
        .sanitized();
        assert_eq!(low.nvme_hot_c, NVME_HOT_C_FLOOR);
        let high = Config {
            nvme_hot_c: 500.0,
            ..Config::default()
        }
        .sanitized();
        assert_eq!(high.nvme_hot_c, NVME_HOT_C_CEIL);
    }

    #[test]
    fn in_range_guard_thresholds_are_left_alone_by_sanitized() {
        let config = Config {
            gpu_hot_c: 84.0,
            nvme_hot_c: 77.0,
            ..Config::default()
        }
        .sanitized();
        assert_eq!(config.gpu_hot_c, 84.0);
        assert_eq!(config.nvme_hot_c, 77.0);
        // And the defaults themselves must survive their own sanitizer.
        let d = Config::default().sanitized();
        assert_eq!(d.gpu_hot_c, GPU_HOT_C_DEFAULT);
        assert_eq!(d.nvme_hot_c, NVME_HOT_C_DEFAULT);
    }

    #[test]
    fn a_bad_gpu_hot_c_in_a_real_file_is_clamped_on_load() {
        let dir = fixture_dir("gpu-hot-clamp");
        let path = dir.join("config.toml");
        fs::write(&path, "gpu_hot_c = 99.0\nnvme_hot_c = nan\n").unwrap();
        let config = Config::load(&path);
        assert_eq!(config.gpu_hot_c, GPU_HOT_C_CEIL);
        assert_eq!(config.nvme_hot_c, NVME_HOT_C_DEFAULT);
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

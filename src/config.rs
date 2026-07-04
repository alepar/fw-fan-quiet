//! User configuration (TOML, `/etc/bazerame-fans/config.toml` in production,
//! `--config` overridable). Loading NEVER crashes: a missing file means
//! defaults, a corrupt file means defaults plus a warning — the daemon must
//! come up and manage fans regardless of config state. `#[serde(default)]`
//! makes every field individually optional, so a partial file overrides only
//! what it names and unknown fields are ignored (serde's default behavior).

use std::io::Write;
use std::path::{Path, PathBuf};

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
}

impl Default for Config {
    fn default() -> Self {
        Config {
            fan_target_rpm: DEFAULT_FAN_TARGET_RPM,
            cpu_floor_w: 15.0,
            gpu_floor_mhz: 1000,
            fast_limit_mw: 53_000,
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
        match toml::from_str(&text) {
            Ok(config) => config,
            Err(e) => {
                tracing::warn!("bad config {}, using defaults: {e}", path.display());
                Config::default()
            }
        }
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
            gpu_floor_mhz: 900,
            fast_limit_mw: 60_000,
        };
        config.save(&path).unwrap();
        assert_eq!(Config::load(&path), config);
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

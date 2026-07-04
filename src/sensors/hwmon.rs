//! hwmon chip discovery and sensor reads (fans, CPU temp, iGPU power).
//!
//! Scans `<root>/hwmon*/name` once at construction to map chip name -> dir;
//! getters re-read the value files on every call. Missing chips or files
//! yield 0.0 -- sensor absence must never crash the sampler.

// Consumed by the sampler thread in Task 7.
#![allow(dead_code)]

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

/// Map of hwmon chip name (contents of `name`, trimmed) to its directory.
pub struct Hwmon {
    chips: HashMap<String, PathBuf>,
}

impl Hwmon {
    /// Scans `root` (production: `/sys/class/hwmon`) for chip directories.
    /// Entries with an unreadable `name` file are skipped silently.
    pub fn discover(root: &Path) -> Self {
        let mut chips = HashMap::new();
        let entries = match fs::read_dir(root) {
            Ok(entries) => entries,
            Err(e) => {
                tracing::debug!("hwmon root {} unreadable: {e}", root.display());
                return Self { chips };
            }
        };
        for entry in entries.flatten() {
            let dir = entry.path();
            if let Ok(name) = fs::read_to_string(dir.join("name")) {
                chips.insert(name.trim().to_string(), dir);
            }
        }
        Self { chips }
    }

    /// True if a chip with this name was discovered.
    pub fn has_chip(&self, name: &str) -> bool {
        self.chips.contains_key(name)
    }

    /// Fan speeds in RPM from `framework_laptop` (fallback: `cros_ec`).
    /// Missing chip or files read as 0.0.
    pub fn fan_rpms(&self) -> (f64, f64) {
        let dir = self
            .chips
            .get("framework_laptop")
            .or_else(|| self.chips.get("cros_ec"));
        let Some(dir) = dir else {
            return (0.0, 0.0);
        };
        (
            read_f64(&dir.join("fan1_input")),
            read_f64(&dir.join("fan2_input")),
        )
    }

    /// CPU Tctl in degrees C from `k10temp` (`temp1_input`, millidegrees).
    /// Missing chip or file reads as 0.0.
    pub fn cpu_temp_c(&self) -> f64 {
        self.read_chip_value("k10temp", "temp1_input") / 1000.0
    }

    /// iGPU power in watts from `amdgpu` (`power1_average`, microwatts).
    /// Missing chip or file reads as 0.0.
    pub fn igpu_w(&self) -> f64 {
        self.read_chip_value("amdgpu", "power1_average") / 1e6
    }

    fn read_chip_value(&self, chip: &str, file: &str) -> f64 {
        match self.chips.get(chip) {
            Some(dir) => read_f64(&dir.join(file)),
            None => 0.0,
        }
    }
}

/// Reads and parses a sysfs value file; any failure yields 0.0.
fn read_f64(path: &Path) -> f64 {
    match fs::read_to_string(path) {
        Ok(s) => s.trim().parse().unwrap_or_else(|e| {
            tracing::debug!("hwmon {}: unparseable value: {e}", path.display());
            0.0
        }),
        Err(e) => {
            tracing::debug!("hwmon {}: {e}", path.display());
            0.0
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
            "bazerame-hwmon-test-{}-{name}",
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

    #[test]
    fn discovers_and_reads_all_chips() {
        let root = fixture_dir("full");
        add_chip(&root, "hwmon0", "k10temp", &[("temp1_input", "49375")]);
        add_chip(
            &root,
            "hwmon1",
            "framework_laptop",
            &[("fan1_input", "1467"), ("fan2_input", "1452")],
        );
        add_chip(&root, "hwmon2", "amdgpu", &[("power1_average", "8041000")]);

        let hwmon = Hwmon::discover(&root);
        assert!(hwmon.has_chip("k10temp"));
        assert!(hwmon.has_chip("framework_laptop"));
        assert!(hwmon.has_chip("amdgpu"));

        assert_eq!(hwmon.cpu_temp_c(), 49.375);
        assert_eq!(hwmon.fan_rpms(), (1467.0, 1452.0));
        let igpu = hwmon.igpu_w();
        assert!((igpu - 8.041).abs() < 1e-9, "expected 8.041 W, got {igpu}");

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn empty_root_all_getters_zero() {
        let root = fixture_dir("empty");

        let hwmon = Hwmon::discover(&root);
        assert!(!hwmon.has_chip("k10temp"));
        assert_eq!(hwmon.fan_rpms(), (0.0, 0.0));
        assert_eq!(hwmon.cpu_temp_c(), 0.0);
        assert_eq!(hwmon.igpu_w(), 0.0);

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn missing_root_all_getters_zero() {
        let hwmon = Hwmon::discover(Path::new("/nonexistent/hwmon-root"));
        assert_eq!(hwmon.fan_rpms(), (0.0, 0.0));
        assert_eq!(hwmon.cpu_temp_c(), 0.0);
        assert_eq!(hwmon.igpu_w(), 0.0);
    }

    #[test]
    fn fans_fall_back_to_cros_ec() {
        let root = fixture_dir("cros-ec");
        add_chip(
            &root,
            "hwmon0",
            "cros_ec",
            &[("fan1_input", "1200"), ("fan2_input", "1300")],
        );

        let hwmon = Hwmon::discover(&root);
        assert_eq!(hwmon.fan_rpms(), (1200.0, 1300.0));

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn fans_prefer_framework_laptop_over_cros_ec() {
        let root = fixture_dir("prefer-framework");
        add_chip(
            &root,
            "hwmon0",
            "cros_ec",
            &[("fan1_input", "1200"), ("fan2_input", "1300")],
        );
        add_chip(
            &root,
            "hwmon1",
            "framework_laptop",
            &[("fan1_input", "1467"), ("fan2_input", "1452")],
        );

        let hwmon = Hwmon::discover(&root);
        assert_eq!(hwmon.fan_rpms(), (1467.0, 1452.0));

        fs::remove_dir_all(&root).unwrap();
    }
}

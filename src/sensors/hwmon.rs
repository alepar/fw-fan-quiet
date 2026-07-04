//! hwmon chip discovery and sensor reads (fans, CPU temp, iGPU power).
//!
//! Scans `<root>/hwmon*/name` once at construction to map chip name -> dir;
//! getters re-read the value files on every call. Missing chips or files
//! yield `None` -- sensor absence must never crash the sampler, and a lost
//! sensor stays distinguishable from a legitimate zero reading (e.g. 0 RPM).

// Consumed by the sampler thread in Task 7.
#![allow(dead_code)]

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

/// Map of hwmon chip name (contents of `name`, trimmed) to its directory.
pub struct Hwmon {
    /// Chip name -> directory. Duplicate chip names are last-write-wins in
    /// nondeterministic directory-iteration order -- known limitation, not
    /// reachable on target hardware (the chip names we read are unique).
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
        let mut names: Vec<&str> = chips.keys().map(String::as_str).collect();
        names.sort_unstable();
        tracing::debug!("hwmon: discovered chips: {names:?}");
        Self { chips }
    }

    /// True if a chip with this name was discovered.
    pub fn has_chip(&self, name: &str) -> bool {
        self.chips.contains_key(name)
    }

    /// Fan speeds in RPM from `framework_laptop` (fallback: `cros_ec`).
    /// None if neither chip is present or either fan file fails to read.
    pub fn fan_rpms(&self) -> Option<(f64, f64)> {
        let dir = self
            .chips
            .get("framework_laptop")
            .or_else(|| self.chips.get("cros_ec"));
        let Some(dir) = dir else {
            tracing::debug!("hwmon: no framework_laptop or cros_ec fan chip");
            return None;
        };
        Some((
            read_f64(&dir.join("fan1_input"))?,
            read_f64(&dir.join("fan2_input"))?,
        ))
    }

    /// CPU Tctl in degrees C from `k10temp` (`temp1_input`, millidegrees).
    /// None if the chip or file is absent/unreadable.
    pub fn cpu_temp_c(&self) -> Option<f64> {
        Some(self.read_chip_value("k10temp", "temp1_input")? / 1000.0)
    }

    /// iGPU power in watts from `amdgpu` (`power1_average`, microwatts).
    /// None if the chip or file is absent/unreadable.
    pub fn igpu_w(&self) -> Option<f64> {
        Some(self.read_chip_value("amdgpu", "power1_average")? / 1e6)
    }

    fn read_chip_value(&self, chip: &str, file: &str) -> Option<f64> {
        let Some(dir) = self.chips.get(chip) else {
            tracing::debug!("hwmon: chip {chip} not present");
            return None;
        };
        read_f64(&dir.join(file))
    }
}

/// Reads and parses a sysfs value file; any failure yields None.
fn read_f64(path: &Path) -> Option<f64> {
    match fs::read_to_string(path) {
        Ok(s) => match s.trim().parse() {
            Ok(v) => Some(v),
            Err(e) => {
                tracing::debug!("hwmon {}: unparseable value: {e}", path.display());
                None
            }
        },
        Err(e) => {
            tracing::debug!("hwmon {}: {e}", path.display());
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-hwmon-test-{}-{name}", std::process::id()));
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

        assert_eq!(hwmon.cpu_temp_c(), Some(49.375));
        assert_eq!(hwmon.fan_rpms(), Some((1467.0, 1452.0)));
        let igpu = hwmon.igpu_w().expect("amdgpu power should read");
        assert!((igpu - 8.041).abs() < 1e-9, "expected 8.041 W, got {igpu}");

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn empty_root_all_getters_none() {
        let root = fixture_dir("empty");

        let hwmon = Hwmon::discover(&root);
        assert!(!hwmon.has_chip("k10temp"));
        assert_eq!(hwmon.fan_rpms(), None);
        assert_eq!(hwmon.cpu_temp_c(), None);
        assert_eq!(hwmon.igpu_w(), None);

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn missing_root_all_getters_none() {
        let hwmon = Hwmon::discover(Path::new("/nonexistent/hwmon-root"));
        assert_eq!(hwmon.fan_rpms(), None);
        assert_eq!(hwmon.cpu_temp_c(), None);
        assert_eq!(hwmon.igpu_w(), None);
    }

    #[test]
    fn missing_fan_file_is_none() {
        // Chip present but fan2_input unreadable: the pair is None, not a
        // fabricated (rpm, 0.0) that would look like a stopped fan.
        let root = fixture_dir("missing-fan-file");
        add_chip(
            &root,
            "hwmon0",
            "framework_laptop",
            &[("fan1_input", "1467")],
        );

        let hwmon = Hwmon::discover(&root);
        assert_eq!(hwmon.fan_rpms(), None);

        fs::remove_dir_all(&root).unwrap();
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
        assert_eq!(hwmon.fan_rpms(), Some((1200.0, 1300.0)));

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
        assert_eq!(hwmon.fan_rpms(), Some((1467.0, 1452.0)));

        fs::remove_dir_all(&root).unwrap();
    }
}

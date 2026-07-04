//! RAPL package power sensor via the powercap interface.
//!
//! Reads the monotonically increasing `energy_uj` counter (wraps at
//! `max_energy_range_uj`) and derives watts from the wrap-aware delta.

// Consumed by the sampler thread in Task 7.
#![allow(dead_code)]

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Watts from two microjoule counter samples `dt_s` seconds apart,
/// accounting for counter wraparound at `max_range_uj`.
pub fn watts_from_counters(prev_uj: u64, cur_uj: u64, max_range_uj: u64, dt_s: f64) -> f64 {
    if dt_s <= 0.0 {
        return 0.0;
    }
    let delta = if cur_uj >= prev_uj {
        cur_uj - prev_uj
    } else {
        max_range_uj - prev_uj + cur_uj
    };
    delta as f64 / 1e6 / dt_s
}

/// Reads package power from a powercap RAPL zone directory
/// (e.g. `/sys/class/powercap/intel-rapl:0`).
pub struct RaplReader {
    energy_path: PathBuf,
    max_range_uj: u64,
    prev: Option<(u64, Instant)>,
}

impl RaplReader {
    /// `base` is the directory containing `energy_uj` and `max_energy_range_uj`.
    /// Returns None if `max_energy_range_uj` can't be read/parsed.
    pub fn new(base: &Path) -> Option<Self> {
        let max_range_uj = read_u64(&base.join("max_energy_range_uj"))?;
        Some(Self {
            energy_path: base.join("energy_uj"),
            max_range_uj,
            prev: None,
        })
    }

    /// Returns average watts since the previous call, or None on the first
    /// call or any read/parse failure.
    pub fn read_watts(&mut self) -> Option<f64> {
        let cur_uj = read_u64(&self.energy_path)?;
        let now = Instant::now();
        let watts = self.prev.map(|(prev_uj, prev_t)| {
            watts_from_counters(prev_uj, cur_uj, self.max_range_uj, (now - prev_t).as_secs_f64())
        });
        self.prev = Some((cur_uj, now));
        watts
    }
}

fn read_u64(path: &Path) -> Option<u64> {
    fs::read_to_string(path).ok()?.trim().parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX_RANGE: u64 = u64::MAX;

    #[test]
    fn watts_from_energy_delta() {
        // 10 J in 2 s = 5 W
        assert_eq!(watts_from_counters(1_000_000, 11_000_000, MAX_RANGE, 2.0), 5.0);
    }

    #[test]
    fn watts_across_wraparound() {
        let max = 1_000_000u64;
        // prev near max, cur small: delta = max - prev + cur = 300_000 uJ over 1s = 0.3 W
        assert!((watts_from_counters(900_000, 200_000, max, 1.0) - 0.3).abs() < 1e-9);
    }

    #[test]
    fn watts_zero_dt_is_zero() {
        assert_eq!(watts_from_counters(0, 100, MAX_RANGE, 0.0), 0.0);
    }

    #[test]
    fn reader_reads_fixture_dir() {
        let dir = std::env::temp_dir().join(format!(
            "bazerame-rapl-test-{}",
            std::process::id()
        ));
        fs::create_dir_all(&dir).unwrap();

        fs::write(dir.join("max_energy_range_uj"), "262143328850\n").unwrap();
        fs::write(dir.join("energy_uj"), "1000000\n").unwrap();

        let mut reader = RaplReader::new(&dir).expect("fixture dir should construct");

        // First call: no previous sample.
        assert_eq!(reader.read_watts(), None);

        fs::write(dir.join("energy_uj"), "2000000\n").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(50));

        let watts = reader.read_watts().expect("second call should yield watts");
        assert!(watts > 0.0, "expected positive watts, got {watts}");

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn reader_missing_dir_is_none() {
        assert!(RaplReader::new(Path::new("/nonexistent/rapl-zone")).is_none());
    }
}

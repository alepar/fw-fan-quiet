//! RAPL package power sensor via the powercap interface.
//!
//! Reads the monotonically increasing `energy_uj` counter (wraps at
//! `max_energy_range_uj`) and derives watts from the wrap-aware delta.

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
    } else if prev_uj <= max_range_uj {
        (max_range_uj - prev_uj) + cur_uj
    } else {
        return 0.0; // counter outside declared range: invalid sample
    };
    delta as f64 / 1e6 / dt_s
}

/// Readings above this are treated as invalid (counter reset, not real power).
const MAX_PLAUSIBLE_WATTS: f64 = 1000.0;

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
            watts_from_counters(
                prev_uj,
                cur_uj,
                self.max_range_uj,
                (now - prev_t).as_secs_f64(),
            )
        });
        self.prev = Some((cur_uj, now));
        // Plausibility clamp: counter resets (suspend/resume, driver reload),
        // undetectable double-wraps, and reset-vs-wrap ambiguity all show up as
        // absurd wattage; drop the sample instead of spiking the control loop.
        watts.filter(|&w| w <= MAX_PLAUSIBLE_WATTS)
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
        assert_eq!(
            watts_from_counters(1_000_000, 11_000_000, MAX_RANGE, 2.0),
            5.0
        );
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
    fn watts_prev_beyond_range_is_zero() {
        // prev outside the declared counter range: invalid sample, not an
        // underflowing wrap computation.
        let max = 1_000_000u64;
        assert_eq!(watts_from_counters(max + 1, 0, max, 1.0), 0.0);
    }

    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-rapl-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn reader_reads_fixture_dir() {
        let dir = fixture_dir("basic");

        fs::write(dir.join("max_energy_range_uj"), "262143328850\n").unwrap();
        fs::write(dir.join("energy_uj"), "1000000\n").unwrap();

        let mut reader = RaplReader::new(&dir).expect("fixture dir should construct");

        // First call: no previous sample.
        assert_eq!(reader.read_watts(), None);

        fs::write(dir.join("energy_uj"), "2000000\n").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(50));

        // 1 J over >= 50 ms is at most 20 W; a missing 1e6 divisor would blow past this.
        let watts = reader.read_watts().expect("second call should yield watts");
        assert!(
            watts > 0.0 && watts <= 20.0,
            "expected plausible watts in (0, 20], got {watts}"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn reader_rejects_implausible_spike() {
        let dir = fixture_dir("spike");

        fs::write(dir.join("max_energy_range_uj"), "262143328850\n").unwrap();
        fs::write(dir.join("energy_uj"), "1000000\n").unwrap();

        let mut reader = RaplReader::new(&dir).expect("fixture dir should construct");
        assert_eq!(reader.read_watts(), None);

        // Jump of ~100 kJ in ~50 ms => ~2 MW: a counter reset / garbage sample,
        // which must not reach the control loop.
        fs::write(dir.join("energy_uj"), "100000001000000\n").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(50));
        assert_eq!(reader.read_watts(), None);

        // State must still have advanced: a subsequent sane delta yields Some.
        fs::write(dir.join("energy_uj"), "100000002000000\n").unwrap();
        std::thread::sleep(std::time::Duration::from_millis(50));
        let watts = reader
            .read_watts()
            .expect("sane delta after spike should yield watts");
        assert!(
            watts > 0.0 && watts <= 20.0,
            "expected plausible watts in (0, 20], got {watts}"
        );

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn reader_missing_dir_is_none() {
        assert!(RaplReader::new(Path::new("/nonexistent/rapl-zone")).is_none());
    }
}

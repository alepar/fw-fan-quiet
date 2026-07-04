//! CPU utilization (from `/proc/stat` deltas) and average core frequency
//! (mean of `cpu*/cpufreq/scaling_cur_freq`) sensors.

use std::fs;
use std::path::{Path, PathBuf};

/// Utilization percent from two `/proc/stat` snapshots: 100 * (1 - didle/dtotal),
/// where idle includes the `idle` and `iowait` fields of the aggregate `cpu ` line.
/// None on parse failure, counter regression, or zero total delta.
pub fn util_from_stat_lines(prev: &str, cur: &str) -> Option<f64> {
    let (prev_idle, prev_total) = parse_aggregate_line(prev)?;
    let (cur_idle, cur_total) = parse_aggregate_line(cur)?;
    let d_total = cur_total.checked_sub(prev_total).filter(|&d| d > 0)?;
    let d_idle = cur_idle.checked_sub(prev_idle)?;
    Some(100.0 * (1.0 - d_idle as f64 / d_total as f64))
}

/// (idle_all, total) jiffies from the first aggregate `cpu ` line of a
/// /proc/stat snapshot. Fields: user nice system idle iowait irq softirq
/// steal guest guest_nice; idle_all = idle + iowait, total = sum of the
/// first 8 fields. guest/guest_nice are excluded from total because the
/// kernel already folds guest time into user -- counting them again would
/// double-count (canonical htop/mpstat formula).
fn parse_aggregate_line(snapshot: &str) -> Option<(u64, u64)> {
    let line = snapshot
        .lines()
        .find(|l| l.split_whitespace().next() == Some("cpu"))?;
    let fields: Vec<u64> = line
        .split_whitespace()
        .skip(1)
        .map(str::parse)
        .collect::<Result<_, _>>()
        .ok()?;
    if fields.len() < 5 {
        tracing::debug!("cpu stat line has {} fields, need >= 5", fields.len());
        return None;
    }
    let idle_all = fields[3].checked_add(fields[4])?;
    let total = fields
        .iter()
        .take(8)
        .try_fold(0u64, |acc, &f| acc.checked_add(f))?;
    Some((idle_all, total))
}

/// Stateful utilization reader over a stat file (production: `/proc/stat`).
pub struct CpuUtil {
    stat_path: PathBuf,
    prev: Option<String>,
}

impl CpuUtil {
    pub fn new(stat_path: &Path) -> Self {
        Self {
            stat_path: stat_path.to_path_buf(),
            prev: None,
        }
    }

    /// Utilization percent since the previous call; None on the first call
    /// or any read/parse failure.
    pub fn read_util_pct(&mut self) -> Option<f64> {
        let cur = match fs::read_to_string(&self.stat_path) {
            Ok(s) => s,
            Err(e) => {
                tracing::debug!("cpu stat {}: {e}", self.stat_path.display());
                return None;
            }
        };
        let util = self
            .prev
            .as_deref()
            .and_then(|prev| util_from_stat_lines(prev, &cur));
        self.prev = Some(cur);
        util
    }
}

/// Mean of `<base>/cpu*/cpufreq/scaling_cur_freq` in MHz (files are kHz).
/// Unreadable cpus are skipped; None if no cpu dir yields a reading.
pub fn avg_freq_mhz(base: &Path) -> Option<f64> {
    let entries = match fs::read_dir(base) {
        Ok(entries) => entries,
        Err(e) => {
            tracing::debug!("cpufreq base {} unreadable: {e}", base.display());
            return None;
        }
    };
    let mut sum_khz = 0.0;
    let mut count = 0u32;
    for entry in entries.flatten() {
        if !is_cpu_dir_name(&entry.file_name().to_string_lossy()) {
            continue;
        }
        let path = entry.path().join("cpufreq").join("scaling_cur_freq");
        match fs::read_to_string(&path).map(|s| s.trim().parse::<f64>()) {
            Ok(Ok(khz)) => {
                sum_khz += khz;
                count += 1;
            }
            Ok(Err(e)) => tracing::debug!("cpufreq {}: unparseable value: {e}", path.display()),
            Err(e) => tracing::debug!("cpufreq {}: {e}", path.display()),
        }
    }
    if count == 0 {
        tracing::debug!(
            "cpufreq: no readable cpu*/cpufreq/scaling_cur_freq under {}",
            base.display()
        );
        return None;
    }
    Some(sum_khz / f64::from(count) / 1000.0)
}

/// True for `cpu<N>` directory names (`cpu0`, `cpu23`), false for siblings
/// like `cpufreq`, `cpuidle`, or bare `cpu`.
fn is_cpu_dir_name(name: &str) -> bool {
    name.strip_prefix("cpu")
        .is_some_and(|rest| !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn util_from_known_deltas() {
        // fields: user nice system idle iowait irq softirq steal guest guest_nice
        // deltas: user +100, idle +50, iowait +50 => didle_all 100, dtotal 200,
        // busy = 100/200 = 50%
        let prev = "cpu  1000 0 500 8000 200 0 0 0 0 0";
        let cur = "cpu  1100 0 500 8050 250 0 0 0 0 0";
        let u = util_from_stat_lines(prev, cur).unwrap();
        assert!((u - 50.0).abs() < 1e-9, "expected 50%, got {u}");
    }

    #[test]
    fn util_ignores_guest_fields() {
        // Same first-8-field deltas as util_from_known_deltas, but with large
        // guest/guest_nice deltas (+600, +300). Guest time is already folded
        // into user by the kernel; if it were summed into total the result
        // would be 100/1100 idle, not 100/200.
        let prev = "cpu  1000 0 500 8000 200 0 0 0 300 100";
        let cur = "cpu  1100 0 500 8050 250 0 0 0 900 400";
        let u = util_from_stat_lines(prev, cur).unwrap();
        assert!((u - 50.0).abs() < 1e-9, "expected 50%, got {u}");
    }

    #[test]
    fn util_garbage_is_none() {
        assert_eq!(util_from_stat_lines("nonsense", "cpu  1 2 3"), None);
    }

    #[test]
    fn util_zero_delta_is_none() {
        let line = "cpu  1 2 3 4 5 6 7 8 9 10";
        assert_eq!(util_from_stat_lines(line, line), None);
    }

    #[test]
    fn util_counter_regression_is_none() {
        // Total went backwards (e.g. stale snapshot ordering): invalid sample.
        let prev = "cpu  1100 0 500 8050 250 0 0 0 0 0";
        let cur = "cpu  1000 0 500 8000 200 0 0 0 0 0";
        assert_eq!(util_from_stat_lines(prev, cur), None);
    }

    #[test]
    fn util_parses_aggregate_line_from_full_stat() {
        // Full /proc/stat snapshots: per-cpu lines must not shadow the aggregate.
        let prev = "cpu  1000 0 500 8000 200 0 0 0 0 0\n\
                    cpu0 500 0 250 4000 100 0 0 0 0 0\n\
                    intr 12345\n";
        let cur = "cpu  1100 0 500 8050 250 0 0 0 0 0\n\
                   cpu0 550 0 250 4025 125 0 0 0 0 0\n\
                   intr 12399\n";
        let u = util_from_stat_lines(prev, cur).unwrap();
        assert!((u - 50.0).abs() < 1e-9, "expected 50%, got {u}");
    }

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-cpu-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn cpu_util_reader_first_none_then_some() {
        let dir = fixture_dir("util-reader");
        let stat = dir.join("stat");
        fs::write(&stat, "cpu  1000 0 500 8000 200 0 0 0 0 0\n").unwrap();

        let mut reader = CpuUtil::new(&stat);
        assert_eq!(reader.read_util_pct(), None, "first call has no baseline");

        fs::write(&stat, "cpu  1100 0 500 8050 250 0 0 0 0 0\n").unwrap();
        let u = reader
            .read_util_pct()
            .expect("second call should yield util");
        assert!((u - 50.0).abs() < 1e-9, "expected 50%, got {u}");

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn cpu_util_reader_missing_file_is_none() {
        let mut reader = CpuUtil::new(Path::new("/nonexistent/stat"));
        assert_eq!(reader.read_util_pct(), None);
        assert_eq!(reader.read_util_pct(), None);
    }

    fn add_cpu(base: &Path, name: &str, freq_khz: &str) {
        let dir = base.join(name).join("cpufreq");
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join("scaling_cur_freq"), format!("{freq_khz}\n")).unwrap();
    }

    #[test]
    fn avg_freq_means_over_cpus() {
        let base = fixture_dir("freq-basic");
        add_cpu(&base, "cpu0", "2000000");
        add_cpu(&base, "cpu1", "4000000");

        assert_eq!(avg_freq_mhz(&base), Some(3000.0));

        fs::remove_dir_all(&base).unwrap();
    }

    #[test]
    fn avg_freq_empty_dir_is_none() {
        let base = fixture_dir("freq-empty");
        assert_eq!(avg_freq_mhz(&base), None);
        fs::remove_dir_all(&base).unwrap();
    }

    #[test]
    fn avg_freq_missing_base_is_none() {
        assert_eq!(avg_freq_mhz(Path::new("/nonexistent/cpu-base")), None);
    }

    #[test]
    fn avg_freq_skips_non_cpu_number_dirs() {
        // `cpuidle`, `cpufreq` etc. live alongside cpuN dirs; garbage inside
        // them must not poison the mean.
        let base = fixture_dir("freq-nonmatching");
        add_cpu(&base, "cpu0", "2000000");
        add_cpu(&base, "cpuidle", "garbage");
        add_cpu(&base, "cpufreq", "999999999");

        assert_eq!(avg_freq_mhz(&base), Some(2000.0));

        fs::remove_dir_all(&base).unwrap();
    }

    #[test]
    fn avg_freq_skips_unreadable_cpu() {
        // cpu1 has no scaling_cur_freq (e.g. offline core): skipped, not fatal.
        let base = fixture_dir("freq-partial");
        add_cpu(&base, "cpu0", "2000000");
        fs::create_dir_all(base.join("cpu1").join("cpufreq")).unwrap();

        assert_eq!(avg_freq_mhz(&base), Some(2000.0));

        fs::remove_dir_all(&base).unwrap();
    }
}

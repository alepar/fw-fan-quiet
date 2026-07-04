//! CPU sustained-power actuator (design doc §3): clamps and applies the
//! sustained package power via ryzenadj (STAPM + slow limit), while leaving
//! the stock fast (burst) limit untouched so short spikes stay responsive.
//! Stock restore works by toggling the ACPI platform profile: firmware
//! reasserts its own limits on a profile change (verified on this machine).

use std::io;
use std::path::PathBuf;
use std::time::Duration;

use super::cmd::Runner;

/// Absolute safety floor (design §5): below ~10 W risks UI stalls and
/// resume instability.
const MIN_SUSTAINED_MW: u32 = 10_000;
/// HX 370 cTDP ceiling.
const MAX_SUSTAINED_MW: u32 = 54_000;
/// Stock burst ceiling (verified on-machine invocation shape).
const DEFAULT_FAST_LIMIT_MW: u32 = 53_000;

// TODO(task-13/14/16): consumed by RestoreGuard, controller wiring, selftest.
#[allow(dead_code)]
pub struct CpuActuator<R: Runner> {
    runner: R,
    /// Stock burst ceiling left untouched so short spikes stay fast
    /// (design §3). Pub so config (Task 21) can set it.
    pub fast_limit_mw: u32,
    /// Production: /sys/firmware/acpi/platform_profile.
    profile_path: PathBuf,
    /// Pause between the profile toggle writes so firmware registers both.
    toggle_delay: Duration,
}

#[allow(dead_code)] // TODO(task-13/14/16): consumed by RestoreGuard + controller wiring.
impl<R: Runner> CpuActuator<R> {
    pub fn new(runner: R, profile_path: PathBuf) -> Self {
        Self {
            runner,
            fast_limit_mw: DEFAULT_FAST_LIMIT_MW,
            profile_path,
            toggle_delay: Duration::from_millis(200),
        }
    }

    /// Clamp `mw` to [10_000, 54_000] and command it as the sustained limit:
    /// `ryzenadj --stapm-limit=<mw> --slow-limit=<mw> --fast-limit=<fast>`.
    /// Returns the clamped value actually commanded; Err on spawn failure or
    /// nonzero exit (stderr included in the error).
    pub fn set_sustained_mw(&self, mw: u32) -> io::Result<u32> {
        let mw = mw.clamp(MIN_SUSTAINED_MW, MAX_SUSTAINED_MW);
        let args = [
            format!("--stapm-limit={mw}"),
            format!("--slow-limit={mw}"),
            format!("--fast-limit={}", self.fast_limit_mw),
        ];
        let args: Vec<&str> = args.iter().map(String::as_str).collect();
        let output = self.runner.run("ryzenadj", &args)?;
        if !output.status.success() {
            return Err(io::Error::other(format!(
                "ryzenadj failed ({}): {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )));
        }
        Ok(mw)
    }

    /// Restore stock CPU limits by toggling the platform profile: firmware
    /// reasserts its own limits on a profile change. Reads the current
    /// profile, writes a different one, waits, writes the original back.
    pub fn restore_stock(&self) -> io::Result<()> {
        let original = std::fs::read_to_string(&self.profile_path)?;
        let original = original.trim();
        // A same-value write would be a no-op the firmware may ignore, so
        // toggle through a *different* profile.
        let intermediate = if original == "low-power" {
            "balanced"
        } else {
            "low-power"
        };
        std::fs::write(&self.profile_path, intermediate)?;
        std::thread::sleep(self.toggle_delay);
        std::fs::write(&self.profile_path, original)?;
        tracing::info!("restored stock CPU limits via platform profile toggle ({original})");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuators::cmd::test_support::{FakeRunner, output_with_code};
    use std::fs;
    use std::path::PathBuf;

    fn actuator(runner: FakeRunner) -> CpuActuator<FakeRunner> {
        CpuActuator::new(runner, PathBuf::from("/nonexistent/platform_profile"))
    }

    fn expected_args(mw: u32) -> Vec<String> {
        vec![
            format!("--stapm-limit={mw}"),
            format!("--slow-limit={mw}"),
            "--fast-limit=53000".to_string(),
        ]
    }

    #[test]
    fn set_clamps_low() {
        let cpu = actuator(FakeRunner::new());
        assert_eq!(cpu.set_sustained_mw(5_000).unwrap(), 10_000);
        assert_eq!(
            cpu.runner.calls(),
            vec![("ryzenadj".to_string(), expected_args(10_000))]
        );
    }

    #[test]
    fn set_clamps_high() {
        let cpu = actuator(FakeRunner::new());
        assert_eq!(cpu.set_sustained_mw(90_000).unwrap(), 54_000);
        let calls = cpu.runner.calls();
        assert_eq!(calls.len(), 1);
        assert!(calls[0].1.contains(&"--stapm-limit=54000".to_string()));
    }

    #[test]
    fn set_passes_through_in_range() {
        let cpu = actuator(FakeRunner::new());
        assert_eq!(cpu.set_sustained_mw(20_000).unwrap(), 20_000);
        assert_eq!(
            cpu.runner.calls(),
            vec![("ryzenadj".to_string(), expected_args(20_000))]
        );
    }

    #[test]
    fn set_propagates_failure() {
        let runner = FakeRunner::new();
        let mut failed = output_with_code(1);
        failed.stderr = b"Unable to get os_access Obj\n".to_vec();
        runner.push_result(Ok(failed));
        let cpu = actuator(runner);

        let err = cpu.set_sustained_mw(20_000).unwrap_err();
        assert!(
            err.to_string().contains("Unable to get os_access Obj"),
            "error should carry stderr, got: {err}"
        );

        // Spawn failure propagates too.
        let runner = FakeRunner::new();
        runner.push_result(Err(io::Error::other("no such binary")));
        let cpu = actuator(runner);
        assert!(cpu.set_sustained_mw(20_000).is_err());
    }

    /// Unique-per-test profile file fixture; caller removes the dir when done.
    fn profile_fixture(name: &str, content: &str) -> (PathBuf, PathBuf) {
        let dir =
            std::env::temp_dir().join(format!("bazerame-cpu-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("platform_profile");
        fs::write(&path, content).unwrap();
        (dir, path)
    }

    fn fast_actuator(profile_path: PathBuf) -> CpuActuator<FakeRunner> {
        let mut cpu = CpuActuator::new(FakeRunner::new(), profile_path);
        cpu.toggle_delay = Duration::from_millis(1); // keep tests fast
        cpu
    }

    #[test]
    fn restore_toggles_profile() {
        let (dir, path) = profile_fixture("balanced", "balanced\n");
        let cpu = fast_actuator(path.clone());

        cpu.restore_stock().unwrap();

        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn restore_from_low_power_toggles_via_balanced() {
        let (dir, path) = profile_fixture("low-power", "low-power\n");
        let cpu = fast_actuator(path.clone());

        cpu.restore_stock().unwrap();

        assert_eq!(fs::read_to_string(&path).unwrap(), "low-power");
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn restore_missing_file_is_err() {
        let cpu = fast_actuator(PathBuf::from("/nonexistent/platform_profile"));
        assert!(cpu.restore_stock().is_err());
    }
}

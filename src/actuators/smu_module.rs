//! Bazzite ryzen_smu workaround (design doc §1, verified on this machine):
//! the installed ryzen_smu 0.1.7 lacks Strix Point PM-table support. While
//! it is loaded, ryzenadj picks the kmod backend and fails ("Unable to get
//! os_access Obj"); with it unloaded, ryzenadj falls back to libpci SMN
//! access, which works. So before CPU actuation we unload the module, and
//! restore it on exit iff we were the ones who unloaded it.

use std::io;
use std::path::Path;

use super::cmd::Runner;

/// Tracks whether *we* unloaded ryzen_smu, so restore is a no-op otherwise.
pub struct SmuModule {
    unloaded_by_us: bool,
}

impl SmuModule {
    /// True iff `<sysfs>/ryzen_smu_drv` exists AND
    /// `<sysfs>/ryzen_smu_drv/pm_table` does NOT exist — the
    /// module-loaded-but-broken-for-this-CPU state that blocks ryzenadj.
    /// Production `sysfs_kernel_dir`: `/sys/kernel`.
    pub fn needs_unload(sysfs_kernel_dir: &Path) -> bool {
        let drv = sysfs_kernel_dir.join("ryzen_smu_drv");
        drv.exists() && !drv.join("pm_table").exists()
    }

    /// If `needs_unload`: run `modprobe -r ryzen_smu`; on success remember we
    /// did it. Returns Err if modprobe cannot run or exits nonzero (caller
    /// decides; CPU actuation without the unload fails visibly later).
    pub fn ensure_unloaded<R: Runner>(runner: &R, sysfs_kernel_dir: &Path) -> io::Result<Self> {
        if !Self::needs_unload(sysfs_kernel_dir) {
            return Ok(Self {
                unloaded_by_us: false,
            });
        }
        let output = runner.run("modprobe", &["-r", "ryzen_smu"])?;
        if !output.status.success() {
            return Err(io::Error::other(format!(
                "modprobe -r ryzen_smu failed ({}): {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            )));
        }
        tracing::info!("unloaded ryzen_smu (no pm_table support; blocks ryzenadj)");
        Ok(Self {
            unloaded_by_us: true,
        })
    }

    /// A module handle that behaves as if we unloaded ryzen_smu, so
    /// `restore` WILL reload it. `FinalRestore` uses this to rebuild the
    /// reload obligation on the panic path (from main's recorded
    /// `smu_was_unloaded`); guard/controller tests use it to observe the
    /// restore sequence without a sysfs fixture.
    pub fn assume_unloaded() -> Self {
        Self {
            unloaded_by_us: true,
        }
    }

    /// True iff `restore` will reload the module (we unloaded it). Main
    /// records this at startup for `FinalRestore`.
    pub fn unloaded_by_us(&self) -> bool {
        self.unloaded_by_us
    }

    /// `modprobe ryzen_smu` ONLY if we unloaded it; idempotent (second call
    /// is a no-op). Failure is warned, not propagated (best-effort exit path).
    pub fn restore<R: Runner>(&mut self, runner: &R) {
        if !self.unloaded_by_us {
            return;
        }
        // Clear the flag first so a second call is a no-op even if the
        // reload fails: retrying on the exit path would not help.
        self.unloaded_by_us = false;
        match runner.run("modprobe", &["ryzen_smu"]) {
            Ok(output) if output.status.success() => {
                tracing::info!("reloaded ryzen_smu");
            }
            Ok(output) => tracing::warn!(
                "modprobe ryzen_smu failed ({}): {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            ),
            Err(e) => tracing::warn!("modprobe ryzen_smu could not run: {e}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuators::cmd::test_support::{FakeRunner, output_with_code};
    use std::fs;
    use std::path::PathBuf;

    /// Unique-per-test fixture root; caller removes it when done.
    fn fixture_dir(name: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("bazerame-smu-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Fixture with `ryzen_smu_drv/` present (a `version` file inside, like
    /// the real /sys/kernel/ryzen_smu_drv) and optionally `pm_table`.
    fn module_fixture(name: &str, pm_table: bool) -> PathBuf {
        let dir = fixture_dir(name);
        let drv = dir.join("ryzen_smu_drv");
        fs::create_dir_all(&drv).unwrap();
        fs::write(drv.join("version"), "0.1.7\n").unwrap();
        if pm_table {
            fs::write(drv.join("pm_table"), "").unwrap();
        }
        dir
    }

    #[test]
    fn needs_unload_true_when_module_without_pm_table() {
        let dir = module_fixture("no-pm-table", false);
        assert!(SmuModule::needs_unload(&dir));
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn needs_unload_false_when_pm_table_present() {
        let dir = module_fixture("pm-table", true);
        assert!(!SmuModule::needs_unload(&dir));
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn needs_unload_false_when_module_absent() {
        let dir = fixture_dir("absent");
        assert!(!SmuModule::needs_unload(&dir));
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn ensure_unloaded_runs_modprobe_r() {
        let dir = module_fixture("unload", false);
        let runner = FakeRunner::new();

        let smu = SmuModule::ensure_unloaded(&runner, &dir).unwrap();

        assert!(smu.unloaded_by_us);
        assert_eq!(
            runner.calls(),
            vec![(
                "modprobe".to_string(),
                vec!["-r".to_string(), "ryzen_smu".to_string()]
            )]
        );
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn ensure_unloaded_noop_when_not_needed() {
        let dir = module_fixture("noop", true);
        let runner = FakeRunner::new();

        let smu = SmuModule::ensure_unloaded(&runner, &dir).unwrap();

        assert!(!smu.unloaded_by_us);
        assert!(runner.calls().is_empty());
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn restore_reloads_only_if_we_unloaded() {
        let runner = FakeRunner::new();

        // We unloaded it: restore reloads, second call is a no-op.
        let mut smu = SmuModule {
            unloaded_by_us: true,
        };
        smu.restore(&runner);
        smu.restore(&runner);
        assert_eq!(
            runner.calls(),
            vec![("modprobe".to_string(), vec!["ryzen_smu".to_string()])]
        );

        // We did not unload it: restore never calls modprobe.
        let runner = FakeRunner::new();
        let mut smu = SmuModule {
            unloaded_by_us: false,
        };
        smu.restore(&runner);
        assert!(runner.calls().is_empty());
    }

    #[test]
    fn ensure_unloaded_propagates_modprobe_failure() {
        // modprobe exits nonzero.
        let dir = module_fixture("fail-status", false);
        let runner = FakeRunner::new();
        runner.push_result(Ok(output_with_code(1)));
        assert!(SmuModule::ensure_unloaded(&runner, &dir).is_err());
        fs::remove_dir_all(&dir).unwrap();

        // modprobe cannot be spawned at all.
        let dir = module_fixture("fail-io", false);
        let runner = FakeRunner::new();
        runner.push_result(Err(io::Error::other("no such binary")));
        assert!(SmuModule::ensure_unloaded(&runner, &dir).is_err());
        fs::remove_dir_all(&dir).unwrap();
    }
}

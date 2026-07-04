//! Hardware restore on every exit path (design doc §5), in two layers:
//!
//! - [`RestoreGuard`] owns the working actuators and moves INTO the
//!   controller thread (the only place hardware writes happen). On clean
//!   shutdown (Command::Quit or command-channel disconnect) the controller
//!   runs `restore_all` itself, flips the shared `restored` flag, and main
//!   joins it before exiting.
//! - [`FinalRestore`] sits on MAIN's stack as the safety net for the paths
//!   where the controller thread never gets to restore: a panicking main
//!   kills other threads without running their drops, but unwinds its own
//!   stack. Its Drop checks `restored`; if the controller didn't get there,
//!   it rebuilds fresh short-lived actuators and runs the same best-effort
//!   restore sequence. Restores are idempotent, so the race where both
//!   layers restore is harmless.
//!
//! Signals (SIGINT/SIGTERM/SIGHUP) are funneled into the normal return path
//! by main's `term_flag`; SIGKILL cannot be caught — `startup_reset` on the
//! *next* run covers that hole.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use super::cmd::{RealRunner, Runner};
use super::cpu::{CpuActuator, PLATFORM_PROFILE_PATH};
use super::gpu::GpuActuator;
use super::smu_module::SmuModule;

/// Owns the hardware-restore responsibility. Every field is an Option:
/// `None` if construction failed at startup (degraded run) or already
/// restored (after `restore_all`).
pub struct RestoreGuard<R: Runner> {
    /// Used for the ryzen_smu reload; `CpuActuator` owns its own runner
    /// (both are `RealRunner`, a ZST, in production).
    runner: R,
    pub cpu: Option<CpuActuator<R>>,
    pub gpu: Option<GpuActuator>,
    pub smu: Option<SmuModule>,
}

impl<R: Runner> RestoreGuard<R> {
    pub fn new(
        runner: R,
        cpu: Option<CpuActuator<R>>,
        gpu: Option<GpuActuator>,
        smu: Option<SmuModule>,
    ) -> Self {
        Self {
            runner,
            cpu,
            gpu,
            smu,
        }
    }

    /// Belt-and-suspenders against a previous SIGKILL'd run: release GPU
    /// clock locks and toggle the CPU profile back to stock once at startup.
    /// KEEPS the actuators (unlike `restore_all`) — the session needs them.
    /// The smu module is untouched: `ensure_unloaded` already ran at startup
    /// and reloading it here would break ryzenadj again. Errors are warned,
    /// never fail startup.
    pub fn startup_reset(&mut self) {
        if let Some(gpu) = self.gpu.as_mut() {
            match gpu.release() {
                Ok(()) => tracing::info!("startup reset: GPU clock locks released"),
                Err(e) => tracing::warn!("startup reset: GPU clock release failed: {e}"),
            }
        }
        if let Some(cpu) = self.cpu.as_ref() {
            if let Err(e) = cpu.restore_stock() {
                tracing::warn!("startup reset: CPU stock restore failed: {e}");
            }
        }
    }

    /// Best-effort restore of stock hardware state, callable explicitly.
    /// Idempotent: each actuator is `take()`n, so a second call is a no-op.
    /// Order: GPU release → CPU restore → ryzen_smu reload last (CPU restore
    /// goes through the platform profile, not ryzenadj, so it doesn't need
    /// the module unloaded — but keeping the reload last is free insurance).
    /// Failures are warned and never propagated: Drop must not panic.
    pub fn restore_all(&mut self) {
        if let Some(mut gpu) = self.gpu.take() {
            match gpu.release() {
                Ok(()) => tracing::info!("restored GPU clocks (locks released)"),
                Err(e) => tracing::warn!("restore: GPU clock release failed: {e}"),
            }
        }
        if let Some(cpu) = self.cpu.take() {
            // Success is logged by restore_stock itself.
            if let Err(e) = cpu.restore_stock() {
                tracing::warn!("restore: CPU stock restore failed: {e}");
            }
        }
        if let Some(mut smu) = self.smu.take() {
            // Logs its own success/failure; no-op unless we unloaded it.
            smu.restore(&self.runner);
        }
    }
}

impl<R: Runner> Drop for RestoreGuard<R> {
    fn drop(&mut self) {
        self.restore_all();
    }
}

/// Main-stack safety net for the panic path (see module doc). Holds no
/// actuators — the working ones live in the controller thread — and only
/// constructs fresh short-lived ones in Drop if the controller never
/// restored (`restored` still false).
pub struct FinalRestore {
    /// Flipped by the controller thread after its `restore_all` ran.
    restored: Arc<AtomicBool>,
    /// Whether startup unloaded ryzen_smu (so the fresh restore must reload it).
    smu_was_unloaded: bool,
    /// Test seam: when set, Drop records that the fresh-actuator restore
    /// WOULD have run instead of touching real hardware/NVML.
    #[cfg(test)]
    probe: Option<Arc<AtomicBool>>,
}

impl FinalRestore {
    pub fn new(restored: Arc<AtomicBool>, smu_was_unloaded: bool) -> Self {
        Self {
            restored,
            smu_was_unloaded,
            #[cfg(test)]
            probe: None,
        }
    }

    #[cfg(test)]
    fn with_probe(
        restored: Arc<AtomicBool>,
        smu_was_unloaded: bool,
        probe: Arc<AtomicBool>,
    ) -> Self {
        Self {
            restored,
            smu_was_unloaded,
            probe: Some(probe),
        }
    }
}

impl Drop for FinalRestore {
    fn drop(&mut self) {
        if self.restored.load(Ordering::SeqCst) {
            tracing::debug!("final restore: controller already restored, nothing to do");
            return;
        }
        #[cfg(test)]
        if let Some(probe) = &self.probe {
            probe.store(true, Ordering::SeqCst);
            return;
        }
        tracing::warn!(
            "final restore: controller never restored (panic path?); \
             restoring with fresh actuators"
        );
        let cpu = CpuActuator::new(RealRunner, PathBuf::from(PLATFORM_PROFILE_PATH));
        let gpu = match GpuActuator::new() {
            Ok(gpu) => Some(gpu),
            Err(e) => {
                tracing::warn!("final restore: GPU actuator unavailable: {e}");
                None
            }
        };
        let smu = self.smu_was_unloaded.then(SmuModule::assume_unloaded);
        RestoreGuard::new(RealRunner, Some(cpu), gpu, smu).restore_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actuators::cmd::test_support::FakeRunner;
    use std::fs;
    use std::path::PathBuf;
    use std::time::Duration;

    /// Unique-per-test profile file fixture; caller removes the dir when done.
    fn profile_fixture(name: &str) -> (PathBuf, PathBuf) {
        let dir =
            std::env::temp_dir().join(format!("bazerame-guard-test-{}-{name}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("platform_profile");
        fs::write(&path, "balanced\n").unwrap();
        (dir, path)
    }

    fn cpu_actuator(runner: &FakeRunner, profile_path: PathBuf) -> CpuActuator<&FakeRunner> {
        let mut cpu = CpuActuator::new(runner, profile_path);
        cpu.toggle_delay = Duration::from_millis(1); // keep tests fast
        cpu
    }

    fn modprobe_reload_calls(runner: &FakeRunner) -> usize {
        runner
            .calls()
            .iter()
            .filter(|(prog, args)| prog == "modprobe" && args == &vec!["ryzen_smu".to_string()])
            .count()
    }

    #[test]
    fn restore_all_runs_all_and_is_idempotent() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("restore-all");
        let mut guard = RestoreGuard::new(
            &runner,
            Some(cpu_actuator(&runner, path.clone())),
            None, // GPU needs NVML hardware; untestable here
            Some(SmuModule::assume_unloaded()),
        );

        guard.restore_all();

        // CPU: profile file toggled back to its original content.
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");
        // SMU: module reloaded exactly once.
        assert_eq!(modprobe_reload_calls(&runner), 1);
        // Everything consumed.
        assert!(guard.cpu.is_none() && guard.gpu.is_none() && guard.smu.is_none());

        // Second call: zero additional runner calls, file untouched.
        let calls_before = runner.calls().len();
        fs::write(&path, "quiet\n").unwrap(); // sentinel: cpu restore would rewrite it
        guard.restore_all();
        assert_eq!(runner.calls().len(), calls_before);
        assert_eq!(fs::read_to_string(&path).unwrap(), "quiet\n");

        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn restore_continues_past_failures() {
        let runner = FakeRunner::new();
        // CPU restore fails: the profile file does not exist.
        let cpu = cpu_actuator(&runner, PathBuf::from("/nonexistent/platform_profile"));
        let mut guard =
            RestoreGuard::new(&runner, Some(cpu), None, Some(SmuModule::assume_unloaded()));

        guard.restore_all();

        // SMU restore still ran despite the CPU failure.
        assert_eq!(modprobe_reload_calls(&runner), 1);
    }

    #[test]
    fn drop_runs_restore() {
        let runner = FakeRunner::new();
        {
            let _guard = RestoreGuard::<&FakeRunner>::new(
                &runner,
                None,
                None,
                Some(SmuModule::assume_unloaded()),
            );
        } // dropped here

        assert_eq!(modprobe_reload_calls(&runner), 1);
    }

    #[test]
    fn final_restore_is_noop_when_controller_already_restored() {
        let restored = Arc::new(AtomicBool::new(true));
        let probe = Arc::new(AtomicBool::new(false));
        drop(FinalRestore::with_probe(
            Arc::clone(&restored),
            true,
            Arc::clone(&probe),
        ));
        assert!(
            !probe.load(Ordering::SeqCst),
            "restored=true must short-circuit before any actuator construction"
        );
    }

    #[test]
    fn final_restore_fires_when_controller_never_restored() {
        let restored = Arc::new(AtomicBool::new(false));
        let probe = Arc::new(AtomicBool::new(false));
        drop(FinalRestore::with_probe(
            Arc::clone(&restored),
            false,
            Arc::clone(&probe),
        ));
        assert!(
            probe.load(Ordering::SeqCst),
            "restored=false must reach the fresh-actuator restore path"
        );
    }

    #[test]
    fn startup_reset_keeps_actuators() {
        let runner = FakeRunner::new();
        let (dir, path) = profile_fixture("startup-reset");
        let mut guard = RestoreGuard::new(
            &runner,
            Some(cpu_actuator(&runner, path.clone())),
            None,
            Some(SmuModule::assume_unloaded()),
        );

        guard.startup_reset();

        // Actuators kept for the session; smu untouched (no modprobe yet).
        assert!(guard.cpu.is_some() && guard.smu.is_some());
        assert_eq!(modprobe_reload_calls(&runner), 0);
        assert_eq!(fs::read_to_string(&path).unwrap(), "balanced");

        // Full restore still performs the whole sequence afterwards.
        fs::write(&path, "quiet\n").unwrap(); // pretend the session changed it
        guard.restore_all();
        assert_eq!(fs::read_to_string(&path).unwrap(), "quiet");
        assert_eq!(modprobe_reload_calls(&runner), 1);

        fs::remove_dir_all(&dir).unwrap();
    }
}

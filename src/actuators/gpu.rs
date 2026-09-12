//! NVIDIA GPU max-clock actuator via NVML locked clocks.
//!
//! Locks graphics clocks to `(MIN_LOCK_MHZ, clamp_gpu_clock(mhz))` -- the min
//! stays at the hardware floor so idle clocks aren't pinned up (verified
//! on-machine during design: locking min=max pinned idle clocks at 1500).
//! Same `Nvml`/`Device` lifetime pattern as `crate::sensors::gpu`: only
//! `Nvml` is stored; the device handle is re-fetched per call.

use nvml_wrapper::Nvml;
use nvml_wrapper::enums::device::GpuLockedClocksSetting;

use super::WriteVerdict;

/// Hardware clock floor on this RTX 5070 Laptop (min supported graphics
/// clock); used as the lock's min so idle clocks stay free to drop.
const MIN_LOCK_MHZ: u32 = 210;
/// Design §3 GPU perf floor default: never command a ceiling below this.
const MIN_MAX_CLOCK_MHZ: u32 = 1000;
/// Max supported graphics clock on this RTX 5070 Laptop.
const MAX_MAX_CLOCK_MHZ: u32 = 3090;

const DEVICE_INDEX: u32 = 0;

/// Clamp a commanded max clock to [1000, 3090] MHz. The driver snaps to its
/// ~7.5 MHz bins on its own; we don't need to.
pub fn clamp_gpu_clock(mhz: u32) -> u32 {
    mhz.clamp(MIN_MAX_CLOCK_MHZ, MAX_MAX_CLOCK_MHZ)
}

/// Utilisation floor (design §2.9): below this, load isn't heavy enough to
/// trust the SM-clock read-back either way.
const VERIFY_UTIL_FLOOR_PCT: f64 = 90.0;
/// LUT-sweep pin-rule slack (design §2.9, reused from the calibration
/// sweep): the pinned clock may run this many MHz above the locked ceiling
/// before it counts as a violation.
const VERIFY_CLOCK_SLACK_MHZ: u32 = 30;
/// Consecutive violating samples before a `Mismatch` is scored (design
/// §2.9's "over 3 samples" — matches the LUT sweep's own pin rule; a lone
/// over-clock sample is normal boost-clock noise at the pin edge).
const VERIFY_STRIKES: u32 = 3;

/// GPU lock read-back verification (design §2.9): there is no NVML read of
/// the applied lock itself, so verification is indirect -- while the GPU is
/// under load (`gpu_util` above the floor), the measured SM clock must stay
/// at or below `locked + slack`. One state machine per locked value; the
/// controller call site that constructs and drives this is Task 19's
/// (`fw-fanctrl-loop-j6s`) -- this type is a standalone, fully unit-tested
/// building block until then.
#[derive(Debug, Clone, Copy)]
pub struct GpuLockVerifier {
    locked_mhz: u32,
    violation_streak: u32,
}

impl GpuLockVerifier {
    /// New verifier for a lock just commanded at `locked_mhz`.
    pub fn new(locked_mhz: u32) -> Self {
        Self {
            locked_mhz,
            violation_streak: 0,
        }
    }

    /// Score one sample against the LUT-sweep pin rule. Below the
    /// utilisation floor: `Unverifiable` (not a failure — there isn't
    /// enough load to trust the reading), and the streak resets (a lull
    /// tells us nothing about whether the NEXT loaded sample would still
    /// violate). At/above the floor: `Verified` when the clock stays within
    /// `locked + slack`; a single overshoot only counts a strike
    /// (`Unverifiable` while the streak is building) — only
    /// `VERIFY_STRIKES` CONSECUTIVE overshoots score a `Mismatch`, naming
    /// the pinned clock.
    pub fn verify_lock(&mut self, gpu_util: f64, gpu_sm_mhz: u32) -> WriteVerdict {
        if gpu_util <= VERIFY_UTIL_FLOOR_PCT {
            self.violation_streak = 0;
            return WriteVerdict::Unverifiable;
        }
        let ceiling = self.locked_mhz + VERIFY_CLOCK_SLACK_MHZ;
        if gpu_sm_mhz <= ceiling {
            self.violation_streak = 0;
            return WriteVerdict::Verified(f64::from(gpu_sm_mhz));
        }
        self.violation_streak += 1;
        if self.violation_streak >= VERIFY_STRIKES {
            WriteVerdict::Mismatch {
                field: "gpu_sm_mhz",
                commanded: f64::from(ceiling),
                read: f64::from(gpu_sm_mhz),
            }
        } else {
            WriteVerdict::Unverifiable
        }
    }
}

/// Actuation seam for the GPU clock lock (Task 25): the real [`GpuActuator`]
/// talks NVML, which needs hardware and root — untestable in unit tests. The
/// controller reaches the GPU only through this trait (boxed in
/// `RestoreGuard`), so tests substitute `test_support::FakeGpu` and can
/// observe the Auto-mode PI's clock commands. `Send` supertrait: the guard
/// moves into the controller thread.
pub trait GpuClockCtl: Send {
    /// Lock graphics clocks to `(210, clamp_gpu_clock(mhz))` and remember the
    /// applied value on success.
    fn set_max_clock(&mut self, mhz: u32) -> color_eyre::Result<()>;
    /// Reset GPU locked clocks to default and clear applied state.
    fn release(&mut self) -> color_eyre::Result<()>;
    /// Last successfully applied max clock, if a lock is active.
    fn applied(&self) -> Option<u32>;
    /// Suspend/resume hook (Task 29): the driver may have forgotten
    /// device-global state across the suspend. Default no-op; the real
    /// actuator re-enables persistence mode (enabled once in
    /// `GpuActuator::new`, not covered by the per-limit reassert path).
    fn resumed(&mut self) {}
}

/// The trait-object form everything stores (`RestoreGuard`, constructors).
pub type BoxedGpu = Box<dyn GpuClockCtl>;

/// Applies max-clock locks to device 0 via NVML. Construction fails if the
/// NVIDIA driver is absent or no device is present.
pub struct GpuActuator {
    nvml: Nvml,
    /// Last successfully applied max clock (clamped), if any lock is active.
    applied_mhz: Option<u32>,
}

impl GpuActuator {
    /// Initializes NVML, verifies device 0 exists, and enables persistence
    /// mode. Persistence failure is only warned (needs root; not required
    /// for locks to apply, only for surviving driver reloads).
    pub fn new() -> color_eyre::Result<Self> {
        let nvml = Nvml::init()?;
        let mut device = nvml.device_by_index(DEVICE_INDEX)?;
        if let Err(e) = device.set_persistent(true) {
            tracing::warn!("nvml: enabling persistence mode failed (locks still work): {e}");
        }
        Ok(Self {
            nvml,
            applied_mhz: None,
        })
    }
}

impl GpuClockCtl for GpuActuator {
    /// Lock graphics clocks to `(210, clamp_gpu_clock(mhz))` and remember the
    /// applied value on success. NVML errors (e.g. no root) propagate.
    fn set_max_clock(&mut self, mhz: u32) -> color_eyre::Result<()> {
        let max_clock_mhz = clamp_gpu_clock(mhz);
        let mut device = self.nvml.device_by_index(DEVICE_INDEX)?;
        device.set_gpu_locked_clocks(GpuLockedClocksSetting::Numeric {
            min_clock_mhz: MIN_LOCK_MHZ,
            max_clock_mhz,
        })?;
        self.applied_mhz = Some(max_clock_mhz);
        Ok(())
    }

    /// Reset GPU locked clocks to default and clear applied state. Errors
    /// propagate; exit-path callers warn instead of propagating.
    fn release(&mut self) -> color_eyre::Result<()> {
        let mut device = self.nvml.device_by_index(DEVICE_INDEX)?;
        device.reset_gpu_locked_clocks()?;
        self.applied_mhz = None;
        Ok(())
    }

    /// Last successfully applied max clock, if a lock is active.
    fn applied(&self) -> Option<u32> {
        self.applied_mhz
    }

    /// Re-enable persistence mode after a suspend/resume: it was enabled at
    /// construction, and a resume-triggered driver reload can drop it. The
    /// call is idempotent and cheap; failure is only warned, exactly like at
    /// construction (locks still work without it). The controller calls this
    /// once per detected resume — never on the 1 Hz reassert path.
    fn resumed(&mut self) {
        match self.nvml.device_by_index(DEVICE_INDEX) {
            Ok(mut device) => {
                if let Err(e) = device.set_persistent(true) {
                    tracing::warn!("nvml: re-enabling persistence after resume failed: {e}");
                }
            }
            Err(e) => tracing::warn!("nvml: device lookup after resume failed: {e}"),
        }
    }
}

/// Shared test double for controller/guard tests (cfg(test) makes it
/// crate-visible in test builds only).
#[cfg(test)]
pub mod test_support {
    use super::{GpuClockCtl, clamp_gpu_clock};
    use std::sync::{Arc, Mutex};

    /// One recorded call on the fake (Set carries the CLAMPED clock, mirroring
    /// what the real actuator would apply).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum GpuCall {
        Set(u32),
        Release,
    }

    /// How the fake card reports an accepted clock command.  The modes model
    /// the two verifier acceptance legs without weakening production policy.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub enum FakeGpuBehavior {
        #[default]
        Immediate,
        OneCommandLag,
        Ignore,
    }

    /// Observable command completion record.  Tests set the logical times so
    /// histories are deterministic and never depend on wall-clock hardware.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct GpuHistory {
        pub call: GpuCall,
        pub acquired_at_s: f64,
        pub completed_at_s: f64,
        pub reported_mhz: Option<u32>,
    }

    /// Cloneable observer retained by tests after a fake moves into a boxed
    /// controller seam.  It intentionally exposes only observations, never
    /// a way to alter the fake's actuator behavior.
    #[derive(Clone)]
    pub struct FakeGpuHandles {
        calls: Arc<Mutex<Vec<GpuCall>>>,
        history: Arc<Mutex<Vec<GpuHistory>>>,
        reported_mhz: Arc<Mutex<Option<u32>>>,
    }

    impl FakeGpuHandles {
        pub fn calls(&self) -> Vec<GpuCall> {
            self.calls.lock().unwrap().clone()
        }

        pub fn history(&self) -> Vec<GpuHistory> {
            self.history.lock().unwrap().clone()
        }

        pub fn reported_sm_clock(&self) -> Option<u32> {
            *self.reported_mhz.lock().unwrap()
        }
    }

    /// Recording stand-in for `GpuActuator`. The call log and the failure
    /// injector are behind `Arc`s so tests keep handles after the fake moves
    /// into the controller's `RestoreGuard`.
    #[derive(Default)]
    pub struct FakeGpu {
        applied: Option<u32>,
        calls: Arc<Mutex<Vec<GpuCall>>>,
        fail_sets: Arc<Mutex<usize>>,
        resumed_count: Arc<Mutex<usize>>,
        /// Raised on every successful `set_max_clock`, when armed: lets a
        /// test model a flag flipping WHILE the (real-world untimed) NVML
        /// call is in flight — e.g. main's `shutdown` racing a sample.
        raise_on_set: Option<Arc<std::sync::atomic::AtomicBool>>,
        behavior: FakeGpuBehavior,
        reported_mhz: Arc<Mutex<Option<u32>>>,
        prior_requested_mhz: Option<u32>,
        command_times: (f64, f64),
        history: Arc<Mutex<Vec<GpuHistory>>>,
    }

    impl FakeGpu {
        pub fn new() -> Self {
            Self::default()
        }

        pub fn with_behavior(behavior: FakeGpuBehavior) -> Self {
            Self {
                behavior,
                ..Self::default()
            }
        }

        /// Timestamp the next command's acquisition and completion in the
        /// deterministic simulated clock used by acceptance scenarios.
        pub fn set_command_times(&mut self, acquired_at_s: f64, completed_at_s: f64) {
            self.command_times = (acquired_at_s, completed_at_s);
        }

        pub fn handles(&self) -> FakeGpuHandles {
            FakeGpuHandles {
                calls: Arc::clone(&self.calls),
                history: Arc::clone(&self.history),
                reported_mhz: Arc::clone(&self.reported_mhz),
            }
        }

        pub fn reported_sm_clock(&self) -> Option<u32> {
            *self.reported_mhz.lock().unwrap()
        }

        pub fn history(&self) -> Vec<GpuHistory> {
            self.history.lock().unwrap().clone()
        }

        /// Arm the mid-call flag raise (see `raise_on_set`).
        pub fn raise_on_set(&mut self, flag: Arc<std::sync::atomic::AtomicBool>) {
            self.raise_on_set = Some(flag);
        }

        /// Shared handle to the call log (clone it before boxing the fake).
        pub fn calls(&self) -> Arc<Mutex<Vec<GpuCall>>> {
            Arc::clone(&self.calls)
        }

        /// Shared failure injector: set `*handle.lock() = n` to make the
        /// next `n` `set_max_clock` calls fail (like the real actuator, a
        /// failed call leaves `applied` untouched and is not recorded).
        pub fn failures(&self) -> Arc<Mutex<usize>> {
            Arc::clone(&self.fail_sets)
        }

        /// Shared counter of `resumed()` hook invocations (Task 29): tests
        /// assert the controller pokes the hook exactly once per resume.
        pub fn resumed_count(&self) -> Arc<Mutex<usize>> {
            Arc::clone(&self.resumed_count)
        }
    }

    impl GpuClockCtl for FakeGpu {
        fn set_max_clock(&mut self, mhz: u32) -> color_eyre::Result<()> {
            {
                let mut fail = self.fail_sets.lock().unwrap();
                if *fail > 0 {
                    *fail -= 1;
                    return Err(color_eyre::eyre::eyre!("injected set_max_clock failure"));
                }
            }
            let clamped = clamp_gpu_clock(mhz);
            self.applied = Some(clamped);
            self.calls.lock().unwrap().push(GpuCall::Set(clamped));
            let reported_mhz = match self.behavior {
                FakeGpuBehavior::Immediate => Some(clamped),
                FakeGpuBehavior::OneCommandLag => self.prior_requested_mhz.or(Some(clamped)),
                // A card that ignores changes keeps its first accepted clock
                // visible to the verifier across every later descent.
                FakeGpuBehavior::Ignore => self.reported_sm_clock().or(Some(clamped)),
            };
            *self.reported_mhz.lock().unwrap() = reported_mhz;
            self.prior_requested_mhz = Some(clamped);
            self.history.lock().unwrap().push(GpuHistory {
                call: GpuCall::Set(clamped),
                acquired_at_s: self.command_times.0,
                completed_at_s: self.command_times.1,
                reported_mhz,
            });
            if let Some(flag) = &self.raise_on_set {
                flag.store(true, std::sync::atomic::Ordering::Relaxed);
            }
            Ok(())
        }

        fn release(&mut self) -> color_eyre::Result<()> {
            self.applied = None;
            self.calls.lock().unwrap().push(GpuCall::Release);
            *self.reported_mhz.lock().unwrap() = None;
            self.prior_requested_mhz = None;
            self.history.lock().unwrap().push(GpuHistory {
                call: GpuCall::Release,
                acquired_at_s: self.command_times.0,
                completed_at_s: self.command_times.1,
                reported_mhz: None,
            });
            Ok(())
        }

        fn applied(&self) -> Option<u32> {
            self.applied
        }

        fn resumed(&mut self) {
            *self.resumed_count.lock().unwrap() += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::{FakeGpu, FakeGpuBehavior};
    use super::*;

    #[test]
    fn clamp_raises_below_floor() {
        assert_eq!(clamp_gpu_clock(999), 1000);
        assert_eq!(clamp_gpu_clock(0), 1000);
    }

    #[test]
    fn clamp_lowers_above_ceiling() {
        assert_eq!(clamp_gpu_clock(5000), 3090);
        assert_eq!(clamp_gpu_clock(3091), 3090);
    }

    #[test]
    fn clamp_passes_through_in_range() {
        assert_eq!(clamp_gpu_clock(1500), 1500);
        assert_eq!(clamp_gpu_clock(1000), 1000);
        assert_eq!(clamp_gpu_clock(3090), 3090);
    }

    #[test]
    fn fake_gpu_records_timestamped_one_command_lag_and_ignore_histories() {
        let mut lag = FakeGpu::with_behavior(FakeGpuBehavior::OneCommandLag);
        lag.set_command_times(10.0, 10.2);
        lag.set_max_clock(3090).unwrap();
        lag.set_command_times(11.0, 11.2);
        lag.set_max_clock(2985).unwrap();
        lag.set_command_times(12.0, 12.2);
        lag.set_max_clock(2880).unwrap();
        assert_eq!(lag.reported_sm_clock(), Some(2985));
        let history = lag.history();
        assert_eq!(history.len(), 3);
        assert_eq!(history[1].acquired_at_s, 11.0);
        assert_eq!(history[1].completed_at_s, 11.2);
        assert_eq!(history[1].reported_mhz, Some(3090));
        assert_eq!(history[2].reported_mhz, Some(2985));

        let mut ignoring = FakeGpu::with_behavior(FakeGpuBehavior::Ignore);
        ignoring.set_max_clock(3090).unwrap();
        ignoring.set_max_clock(2985).unwrap();
        ignoring.set_max_clock(2880).unwrap();
        assert_eq!(ignoring.reported_sm_clock(), Some(3090));
        let mut verifier = GpuLockVerifier::new(2880);
        for _ in 0..3 {
            verifier.verify_lock(100.0, ignoring.reported_sm_clock().unwrap());
        }
        assert!(matches!(
            verifier.verify_lock(100.0, ignoring.reported_sm_clock().unwrap()),
            WriteVerdict::Mismatch { .. }
        ));
    }

    #[test]
    fn boxed_fake_keeps_shared_lag_and_ignore_histories_for_verifier_pairs() {
        let fake = FakeGpu::with_behavior(FakeGpuBehavior::OneCommandLag);
        let handles = fake.handles();
        let mut gpu: Box<dyn GpuClockCtl> = Box::new(fake);
        gpu.set_max_clock(3090).unwrap();
        gpu.set_max_clock(2985).unwrap();
        assert_eq!(handles.reported_sm_clock(), Some(3090));
        let mut prior_pair = GpuLockVerifier::new(3090);
        assert_eq!(
            prior_pair.verify_lock(100.0, handles.reported_sm_clock().unwrap()),
            WriteVerdict::Verified(3090.0)
        );
        gpu.release().unwrap();
        gpu.set_max_clock(2880).unwrap();
        assert_eq!(
            handles.reported_sm_clock(),
            Some(2880),
            "release clears the lag predecessor"
        );
        assert_eq!(handles.calls().len(), 4);
        assert_eq!(handles.history().len(), 4);

        let ignoring = FakeGpu::with_behavior(FakeGpuBehavior::Ignore);
        let handles = ignoring.handles();
        let mut gpu: Box<dyn GpuClockCtl> = Box::new(ignoring);
        gpu.set_max_clock(3090).unwrap();
        gpu.set_max_clock(2880).unwrap();
        let mut verifier = GpuLockVerifier::new(2880);
        assert_eq!(
            verifier.verify_lock(100.0, handles.reported_sm_clock().unwrap()),
            WriteVerdict::Unverifiable
        );
        assert_eq!(
            verifier.verify_lock(100.0, handles.reported_sm_clock().unwrap()),
            WriteVerdict::Unverifiable
        );
        assert!(matches!(
            verifier.verify_lock(100.0, handles.reported_sm_clock().unwrap()),
            WriteVerdict::Mismatch { .. }
        ));
    }

    #[test]
    fn one_command_lag_descent_never_scores_a_current_lock_mismatch() {
        let fake = FakeGpu::with_behavior(FakeGpuBehavior::OneCommandLag);
        let handles = fake.handles();
        let mut gpu: Box<dyn GpuClockCtl> = Box::new(fake);
        let mut commands: Vec<u32> = (0..20).map(|step| 3090 - step * 105).collect();
        commands.push(1000);
        for (step, command) in commands.iter().copied().enumerate() {
            gpu.set_max_clock(command).unwrap();
            let mut verifier = GpuLockVerifier::new(command);
            let verdict = verifier.verify_lock(100.0, handles.reported_sm_clock().unwrap());
            assert!(
                !matches!(verdict, WriteVerdict::Mismatch { .. }),
                "step {step}: {command} MHz reported {:?}",
                handles.reported_sm_clock()
            );
        }
        assert_eq!(handles.history().len(), commands.len());
        assert_eq!(handles.history().last().unwrap().reported_mhz, Some(1095));

        let ignoring = FakeGpu::with_behavior(FakeGpuBehavior::Ignore);
        let handles = ignoring.handles();
        let mut gpu: Box<dyn GpuClockCtl> = Box::new(ignoring);
        for command in [3090, 2985, 2880, 2775] {
            gpu.set_max_clock(command).unwrap();
        }
        assert!(
            handles
                .history()
                .iter()
                .all(|entry| entry.reported_mhz == Some(3090))
        );
    }

    /// Step 5 (TDD): below the 90% utilisation floor, `Unverifiable` --
    /// regardless of how far over the pin the reported clock is.
    #[test]
    fn below_util_floor_is_unverifiable_even_when_clock_is_wildly_over() {
        let mut v = GpuLockVerifier::new(2000);
        assert_eq!(v.verify_lock(89.9, 5000), WriteVerdict::Unverifiable);
        assert_eq!(v.verify_lock(0.0, 5000), WriteVerdict::Unverifiable);
        assert_eq!(v.verify_lock(90.0, 5000), WriteVerdict::Unverifiable);
    }

    /// Above the floor and within `locked + 30`: `Verified`.
    #[test]
    fn above_util_floor_within_slack_is_verified() {
        let mut v = GpuLockVerifier::new(2000);
        assert_eq!(v.verify_lock(95.0, 2000), WriteVerdict::Verified(2000.0));
        assert_eq!(v.verify_lock(95.0, 2030), WriteVerdict::Verified(2030.0));
    }

    /// Mismatch requires 3 CONSECUTIVE over-pin samples, not 1 or 2 -- a
    /// wrong implementation that scored on the first (or second) violation
    /// would fail these two assertions before the loop ever reaches 3.
    #[test]
    fn mismatch_only_fires_on_the_third_consecutive_overshoot() {
        let mut v = GpuLockVerifier::new(2000); // ceiling 2030
        assert_eq!(
            v.verify_lock(95.0, 2031),
            WriteVerdict::Unverifiable,
            "1st consecutive overshoot must not yet be Mismatch"
        );
        assert_eq!(
            v.verify_lock(95.0, 2031),
            WriteVerdict::Unverifiable,
            "2nd consecutive overshoot must not yet be Mismatch"
        );
        assert_eq!(
            v.verify_lock(95.0, 2031),
            WriteVerdict::Mismatch {
                field: "gpu_sm_mhz",
                commanded: 2030.0,
                read: 2031.0,
            },
            "3rd consecutive overshoot must score Mismatch"
        );
    }

    /// A compliant sample between overshoots resets the streak: 2 + 2 never
    /// reaches 3.
    #[test]
    fn a_compliant_sample_resets_the_overshoot_streak() {
        let mut v = GpuLockVerifier::new(2000);
        assert_eq!(v.verify_lock(95.0, 2031), WriteVerdict::Unverifiable);
        assert_eq!(v.verify_lock(95.0, 2031), WriteVerdict::Unverifiable);
        assert_eq!(v.verify_lock(95.0, 2000), WriteVerdict::Verified(2000.0));
        // Streak reset: this is only the first overshoot again.
        assert_eq!(v.verify_lock(95.0, 2031), WriteVerdict::Unverifiable);
    }

    /// A below-floor sample between overshoots also resets the streak (an
    /// unloaded lull tells us nothing about whether the NEXT loaded sample
    /// would still violate).
    #[test]
    fn a_below_floor_sample_resets_the_overshoot_streak() {
        let mut v = GpuLockVerifier::new(2000);
        assert_eq!(v.verify_lock(95.0, 2031), WriteVerdict::Unverifiable);
        assert_eq!(v.verify_lock(95.0, 2031), WriteVerdict::Unverifiable);
        assert_eq!(v.verify_lock(10.0, 2031), WriteVerdict::Unverifiable); // resets
        assert_eq!(
            v.verify_lock(95.0, 2031),
            WriteVerdict::Unverifiable,
            "1st since the reset"
        );
        assert_eq!(
            v.verify_lock(95.0, 2031),
            WriteVerdict::Unverifiable,
            "2nd since the reset"
        );
        assert_eq!(
            v.verify_lock(95.0, 2031),
            WriteVerdict::Mismatch {
                field: "gpu_sm_mhz",
                commanded: 2030.0,
                read: 2031.0,
            },
            "3rd since the reset"
        );
    }

    /// Manual smoke check against the real GPU: exercises NVML init and the
    /// lock call path. As a non-root user the lock call is expected to fail
    /// (NVML requires root for clock locks); if it somehow succeeds, release
    /// immediately. Run with: cargo test -- --ignored nvml_lock
    #[test]
    #[ignore = "requires NVIDIA GPU; run manually"]
    fn nvml_lock_smoke() {
        let mut actuator = GpuActuator::new().expect("NVML init + device 0");
        assert_eq!(actuator.applied(), None);
        match actuator.set_max_clock(3000) {
            Ok(()) => {
                println!("set_max_clock(3000) unexpectedly permitted; releasing");
                // Release BEFORE asserting so a failed assert can't leave clocks locked.
                let applied = actuator.applied();
                actuator.release().expect("release after successful lock");
                assert_eq!(applied, Some(3000));
                assert_eq!(actuator.applied(), None);
            }
            Err(e) => {
                println!("set_max_clock(3000) failed as expected without root: {e}");
                assert_eq!(actuator.applied(), None);
            }
        }
    }
}

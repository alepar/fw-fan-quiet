//! NVIDIA GPU max-clock actuator via NVML locked clocks.
//!
//! Locks graphics clocks to `(MIN_LOCK_MHZ, clamp_gpu_clock(mhz))` -- the min
//! stays at the hardware floor so idle clocks aren't pinned up (verified
//! on-machine during design: locking min=max pinned idle clocks at 1500).
//! Same `Nvml`/`Device` lifetime pattern as `crate::sensors::gpu`: only
//! `Nvml` is stored; the device handle is re-fetched per call.

use nvml_wrapper::Nvml;
use nvml_wrapper::enums::device::GpuLockedClocksSetting;

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

    /// Recording stand-in for `GpuActuator`. The call log is behind an `Arc`
    /// so tests keep a handle after the fake moves into the controller's
    /// `RestoreGuard`.
    #[derive(Default)]
    pub struct FakeGpu {
        applied: Option<u32>,
        calls: Arc<Mutex<Vec<GpuCall>>>,
    }

    impl FakeGpu {
        pub fn new() -> Self {
            Self::default()
        }

        /// Shared handle to the call log (clone it before boxing the fake).
        pub fn calls(&self) -> Arc<Mutex<Vec<GpuCall>>> {
            Arc::clone(&self.calls)
        }
    }

    impl GpuClockCtl for FakeGpu {
        fn set_max_clock(&mut self, mhz: u32) -> color_eyre::Result<()> {
            let clamped = clamp_gpu_clock(mhz);
            self.applied = Some(clamped);
            self.calls.lock().unwrap().push(GpuCall::Set(clamped));
            Ok(())
        }

        fn release(&mut self) -> color_eyre::Result<()> {
            self.applied = None;
            self.calls.lock().unwrap().push(GpuCall::Release);
            Ok(())
        }

        fn applied(&self) -> Option<u32> {
            self.applied
        }
    }
}

#[cfg(test)]
mod tests {
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

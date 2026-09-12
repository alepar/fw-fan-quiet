//! Thermal guards (design doc §2.8): dGPU and NVMe hysteresis, run every
//! sample in any auto mode ahead of the arbiter. Both inputs are
//! `Option<f64>` — an absent reading (dGPU unpowered, NVML N/A, nvme chip
//! missing) makes that guard inactive and clears any hot state it was
//! holding.
//!
//! - **dGPU** drives [`gpu_share_override`]: while hot, the caller ratchets
//!   the GPU's allocator share down toward its LUT floor at
//!   [`crate::control::allocator::DOWN_RATE_W`] per allocator tick, starting
//!   from the present draw. `gpu_hot_c` defaults to 88 °C (exit 86). The
//!   card's NVML T.Limit specs (measured 2026-09-09, §Facts) put its park /
//!   max-operating point at 87 °C, Slowdown at 89 and Shutdown at 92 — a
//!   5 °C band. 88 sits one degree above the park point so it does not fire
//!   during ordinary sustained load, and three below the hard watchdog
//!   (`watchdog::GPU_TRIP_C`, 91) so this ratchet actually gets a turn
//!   before the emergency release. Under a 100 W gpu-burn the die settles at
//!   82–83 °C on `quiet16` with the fans free (`docs/research/2026-09-09-
//!   gpu-burn-fanctrl-quiet16.csv`); reaching 88 therefore means the fans
//!   are being held below what the card needs — i.e. our cap is the thing
//!   to ease, which is exactly what this guard does.
//! - **NVMe is reporting-only.** §Facts (measured 2026-09-08): under
//!   sustained I/O the drive climbed 67 → 80 °C in 30 s while the fans were
//!   already pinned near max, and the EC max *fell* 74 → 69 °C over the same
//!   window because the load was I/O-bound and left the SoC idle. Airflow at
//!   full tilt did not hold the drive, and the guard's only lever — raising
//!   the fan target — would raise T* and therefore the CPU/GPU budget, i.e.
//!   inject more heat into a scenario with nothing to raise it for. So
//!   `nvme_hot` only sets a flag; nothing here reads the user's RPM target
//!   or the power budget, and **there is no `effective_target` function of
//!   any kind in this module.** [`GuardState`] carries exactly the two
//!   flags — see `guard_state_carries_only_the_two_flags` below, which
//!   destructures it exhaustively so an added field fails the build.

use crate::control::allocator::DOWN_RATE_W;
use crate::control::device_loop::{Mhz, W};
use std::marker::PhantomData;

/// Default `gpu_hot_c` (°C). Exit is this minus [`GPU_HYSTERESIS_C`] (86 °C).
/// See the module docs for the measured derivation.
pub const GPU_HOT_C_DEFAULT: f64 = 88.0;
/// Default `nvme_hot_c` (°C). Exit is this minus [`NVME_HYSTERESIS_C`] (75 °C).
/// Reporting-only threshold — see the module docs.
pub const NVME_HOT_C_DEFAULT: f64 = 80.0;
/// dGPU hysteresis band: exit = enter − this. Narrow on purpose: the card's
/// usable band is only 5 °C wide (park 87 → shutdown 92), and the EC's
/// `gpu_vr` sensor — the steady-state argmax under GPU load — has a long
/// thermal tail (measured 2026-09-09: still 67–71 °C forty seconds after
/// the die was back at 50), so a 5 °C band would latch the guard well past
/// the episode.
pub const GPU_HYSTERESIS_C: f64 = 2.0;
/// NVMe hysteresis band: exit = enter − this. The guard is reporting-only,
/// so a wide band only affects how long the flag shows.
const NVME_HYSTERESIS_C: f64 = 5.0;

/// dGPU maximum-clock reduction per valid GPU-hot control sample (MHz).
#[allow(dead_code)] // Wired by the following controller integration task.
pub const GPU_MAX_RATCHET_DOWN_RATE_MHZ: f64 = 105.0;
/// CPU sustained-power reduction per valid CPU-hot control sample (W).
#[allow(dead_code)] // Wired by the following controller integration task.
pub const CPU_MAX_RATCHET_DOWN_RATE_W: f64 = 2.0;
/// The GPU die must be this far below its enter threshold before recovery.
#[allow(dead_code)] // Wired by the following controller integration task.
pub const GPU_MAX_RATCHET_RECOVERY_MARGIN_C: f64 = 4.0;
/// Tctl must be this far below its enter threshold before CPU recovery.
#[allow(dead_code)] // Wired by the following controller integration task.
pub const CPU_MAX_RATCHET_RECOVERY_MARGIN_C: f64 = 5.0;

/// Per-device ceiling ratchet used by the CPU and GPU hot guards.  It only
/// moves on valid control samples: an active guard lowers the ceiling, while
/// a cool die/Tctl reading restores it at half the lowering rate.
#[allow(dead_code)] // Wired by the following controller integration task.
pub struct MaxRatchet<U> {
    floor: f64,
    ceiling: f64,
    current: f64,
    down_rate: f64,
    recovery_c: f64,
    unit: PhantomData<U>,
}

#[allow(dead_code)] // Wired by the following controller integration task.
impl<U> MaxRatchet<U> {
    pub fn new(floor: f64, ceiling: f64, current: f64, down_rate: f64, recovery_c: f64) -> Self {
        let (floor, ceiling) = if floor <= ceiling {
            (floor, ceiling)
        } else {
            (ceiling, floor)
        };
        Self {
            floor,
            ceiling,
            current: current.clamp(floor, ceiling),
            down_rate,
            recovery_c,
            unit: PhantomData,
        }
    }

    /// Applies a live floor without discarding the current hot ratchet.
    pub fn set_floor(&mut self, floor: f64) {
        self.ceiling = self.ceiling.max(floor);
        self.floor = floor;
        self.current = self.current.clamp(self.floor, self.ceiling);
    }

    /// Advances one control sample and returns the new device ceiling.
    pub fn step(&mut self, valid_control: bool, active: bool, temperature_c: Option<f64>) -> f64 {
        if !valid_control {
            return self.current;
        }
        if active {
            self.current = (self.current - self.down_rate).max(self.floor);
        } else if temperature_c.is_some_and(|temperature| temperature <= self.recovery_c) {
            self.current = (self.current + self.down_rate / 2.0).min(self.ceiling);
        }
        self.current
    }
}

#[allow(dead_code)] // Wired by the following controller integration task.
impl MaxRatchet<Mhz> {
    /// Builds the GPU max-clock ratchet from its configured hot threshold.
    pub fn gpu(floor: f64, ceiling: f64, current: f64, gpu_hot_c: f64) -> Self {
        Self::new(
            floor,
            ceiling,
            current,
            GPU_MAX_RATCHET_DOWN_RATE_MHZ,
            gpu_hot_c - GPU_MAX_RATCHET_RECOVERY_MARGIN_C,
        )
    }
}

#[allow(dead_code)] // Wired by the following controller integration task.
impl MaxRatchet<W> {
    /// Builds the CPU sustained-power ratchet from its configured hot threshold.
    pub fn cpu(floor: f64, ceiling: f64, current: f64, cpu_hot_c: f64) -> Self {
        Self::new(
            floor,
            ceiling,
            current,
            CPU_MAX_RATCHET_DOWN_RATE_W,
            cpu_hot_c - CPU_MAX_RATCHET_RECOVERY_MARGIN_C,
        )
    }
}

/// Per-tick guard flags. Deliberately only these two `bool`s: the NVMe guard
/// is reporting-only and the dGPU override is applied directly to the
/// allocator's GPU share by [`gpu_share_override`], not carried through this
/// struct — there is no target or budget field here, ever.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuardState {
    pub gpu_hot: bool,
    pub nvme_hot: bool,
}

/// dGPU + NVMe thermal guards (§2.8). Owns each axis's hysteresis state (the
/// previous tick's hot/not-hot) so [`Guards::step`] only needs this tick's
/// readings.
pub struct Guards {
    gpu_hot_c: f64,
    nvme_hot_c: f64,
    gpu_hot: bool,
    nvme_hot: bool,
}

impl Guards {
    /// New guards with the given enter thresholds (°C; exit = enter −
    /// [`GPU_HYSTERESIS_C`] for the dGPU, enter − [`NVME_HYSTERESIS_C`] for
    /// the NVMe); both axes start cold.
    pub fn new(gpu_hot_c: f64, nvme_hot_c: f64) -> Self {
        Guards {
            gpu_hot_c,
            nvme_hot_c,
            gpu_hot: false,
            nvme_hot: false,
        }
    }

    /// Advance both guards one tick and return the resulting flags. `None`
    /// means that sensor's reading is unavailable this tick: the guard goes
    /// inactive and any hot state clears, regardless of the last reading.
    pub fn step(&mut self, gpu_temp_c: Option<f64>, nvme_temp_c: Option<f64>) -> GuardState {
        self.gpu_hot = hysteresis(self.gpu_hot, gpu_temp_c, self.gpu_hot_c, GPU_HYSTERESIS_C);
        self.nvme_hot = hysteresis(
            self.nvme_hot,
            nvme_temp_c,
            self.nvme_hot_c,
            NVME_HYSTERESIS_C,
        );
        GuardState {
            gpu_hot: self.gpu_hot,
            nvme_hot: self.nvme_hot,
        }
    }
}

/// One axis of enter/exit hysteresis: enters (`true`) once `temp_c` reaches
/// `enter_c`, stays hot through the band, and clears at `enter_c − band_c`.
/// `None` always returns `false`, regardless of `was_hot`.
fn hysteresis(was_hot: bool, temp_c: Option<f64>, enter_c: f64, band_c: f64) -> bool {
    let Some(t) = temp_c else {
        return false;
    };
    if !was_hot && t >= enter_c {
        true
    } else if was_hot && t <= enter_c - band_c {
        false
    } else {
        was_hot
    }
}

/// dGPU share override while `gpu_hot` (§2.8): ratchets the GPU's allocator
/// share down from the present draw at `DOWN_RATE_W` per allocator tick,
/// never below `gpu_floor_w`.
pub fn gpu_share_override(current_gpu_w: f64, gpu_floor_w: f64) -> f64 {
    (current_gpu_w - DOWN_RATE_W).max(gpu_floor_w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::device_loop::{
        ActuatorState, DeviceLoop, Gains, Hold, Mhz, ThermalMode, TickInput, W,
    };

    #[test]
    fn gpu_max_ratchet_lowers_once_per_valid_active_sample() {
        let mut ratchet = MaxRatchet::<Mhz>::new(1_000.0, 3_090.0, 3_090.0, 105.0, 84.0);

        assert_eq!(ratchet.step(true, true, Some(88.0)), 2_985.0);
        assert_eq!(ratchet.step(true, true, Some(88.0)), 2_880.0);
        assert_eq!(ratchet.step(false, true, Some(88.0)), 2_880.0);
    }

    #[test]
    fn device_ratchets_use_their_specified_units_and_recovery_gates() {
        let mut gpu = MaxRatchet::<Mhz>::gpu(1_000.0, 3_090.0, 1_000.0, 88.0);
        let mut cpu = MaxRatchet::<W>::cpu(8.0, 54.0, 54.0, 90.0);

        assert_eq!(
            gpu.step(true, false, Some(85.0)),
            1_000.0,
            "GPU exit band cannot recover"
        );
        assert_eq!(
            cpu.step(true, true, Some(90.0)),
            52.0,
            "CPU lowers in watts, not MHz"
        );
        assert_eq!(
            cpu.step(true, false, Some(86.0)),
            52.0,
            "CPU exit band cannot recover"
        );
        assert_eq!(gpu.step(true, false, Some(84.0)), 1_052.5);
        assert_eq!(cpu.step(true, false, Some(85.0)), 53.0);
    }

    #[test]
    fn ratchets_stop_at_floor_and_recovery_stops_at_configured_max() {
        let mut gpu = MaxRatchet::<Mhz>::gpu(1_000.0, 3_090.0, 1_105.0, 88.0);
        assert_eq!(gpu.step(true, true, Some(95.0)), 1_000.0);
        assert_eq!(
            gpu.step(true, true, Some(95.0)),
            1_000.0,
            "floor is idempotent"
        );
        assert_eq!(
            gpu.step(false, true, Some(95.0)),
            1_000.0,
            "invalid samples do not ratchet"
        );

        let mut cpu = MaxRatchet::<W>::cpu(8.0, 9.5, 8.8, 90.0);
        assert_eq!(cpu.step(true, false, Some(85.0)), 9.5);
        assert_eq!(
            cpu.step(true, false, Some(85.0)),
            9.5,
            "configured max caps recovery"
        );
    }

    #[test]
    fn gpu_exit_band_flapping_cannot_restore_or_cycle_its_max() {
        let mut guard = Guards::new(88.0, 80.0);
        let mut ratchet = MaxRatchet::<Mhz>::gpu(1_000.0, 3_090.0, 1_000.0, 88.0);

        assert!(guard.step(Some(88.0), None).gpu_hot);
        assert_eq!(ratchet.step(true, true, Some(88.0)), 1_000.0);
        for temperature in [87.9, 86.5, 86.0, 86.5, 87.9, 88.0] {
            let active = guard.step(Some(temperature), None).gpu_hot;
            assert_eq!(
                ratchet.step(true, active, Some(temperature)),
                1_000.0,
                "{temperature}C in the enter/exit band must not re-open the ceiling"
            );
        }
    }

    #[test]
    fn hot_die_ratchet_clamps_a_cool_averaged_group_during_bypass_and_mismatch() {
        for (mode, actuator, expected_hold) in [
            (ThermalMode::Bypass, ActuatorState::Verified, Hold::Bypass),
            (
                ThermalMode::Regulate,
                ActuatorState::Mismatch,
                Hold::ActuatorMismatch,
            ),
        ] {
            let mut loop_ = DeviceLoop::<W>::new(Gains {
                kc: 1.0,
                ti_s: 10.0,
            });
            loop_.seed(100.0, 0.0);
            loop_.transfer_shadow(100.0);
            let mut tick = TickInput {
                t_star: 70.0,
                group_c: Some(60.0),
                draw: Some(50.0),
                floor: 10.0,
                max: 100.0,
                mode,
                actuator,
                dt_s: 1.0,
                resumed: false,
                delta_tstar: 0.0,
                shadow_headroom: 10.0,
                shadow_fall_rate: 0.33,
                shadow_enabled: false,
            };
            loop_.tick(tick);

            let mut ratchet = MaxRatchet::<W>::cpu(10.0, 100.0, 100.0, 90.0);
            tick.max = ratchet.step(true, true, Some(90.0));
            let guarded = loop_.tick(tick);
            assert_eq!(guarded.cap, 98.0);
            assert_eq!(guarded.thermal, 98.0);
            assert_eq!(loop_.thermal(), 98.0);
            assert_eq!(guarded.hold, expected_hold);
            assert!(guarded.write_immediately);

            tick.max = ratchet.step(true, false, Some(85.0));
            let recovered = loop_.tick(tick);
            assert_eq!(loop_.thermal(), 98.0, "recovery raises only the ceiling");
            assert!(!recovered.write_immediately);
        }
    }

    #[test]
    fn gpu_hysteresis_enters_at_threshold_and_exits_two_below() {
        let mut g = Guards::new(88.0, 80.0);
        assert!(!g.step(Some(87.9), None).gpu_hot, "below enter: cold");
        assert!(g.step(Some(88.0), None).gpu_hot, "at enter: hot");
        assert!(
            g.step(Some(87.0), None).gpu_hot,
            "in the band (exit < t < enter): stays hot"
        );
        assert!(!g.step(Some(86.0), None).gpu_hot, "at exit: clears");
    }

    #[test]
    fn nvme_hysteresis_enters_at_threshold_and_exits_five_below() {
        let mut g = Guards::new(88.0, 80.0);
        assert!(!g.step(None, Some(79.9)).nvme_hot, "below enter: cold");
        assert!(g.step(None, Some(80.0)).nvme_hot, "at enter: hot");
        assert!(
            g.step(None, Some(76.0)).nvme_hot,
            "in the band (exit < t < enter): stays hot"
        );
        assert!(!g.step(None, Some(75.0)).nvme_hot, "at exit: clears");
    }

    #[test]
    fn none_reading_deactivates_and_clears_an_already_hot_gpu_guard() {
        let mut g = Guards::new(88.0, 80.0);
        assert!(g.step(Some(95.0), None).gpu_hot, "primed hot");
        let state = g.step(None, None);
        assert!(!state.gpu_hot, "absent reading clears hot state");
        // And it does not silently re-latch hot on the very next tick just
        // because the axis was hot two ticks ago — the guard must have
        // actually reset, not merely reported false once.
        let state = g.step(Some(87.0), None);
        assert!(
            !state.gpu_hot,
            "87 is below the 88 enter threshold (and above the 86 exit): a reset \
             guard stays cold, a guard that only 'reported' false would still be \
             latched hot"
        );
    }

    #[test]
    fn none_reading_deactivates_and_clears_an_already_hot_nvme_guard() {
        let mut g = Guards::new(88.0, 80.0);
        assert!(g.step(None, Some(85.0)).nvme_hot, "primed hot");
        let state = g.step(None, None);
        assert!(!state.nvme_hot, "absent reading clears hot state");
    }

    #[test]
    fn guards_are_independent_axes() {
        // A hot GPU must never flip nvme_hot and vice versa (a shared bool,
        // or thresholds swapped between axes, would fail this).
        let mut g = Guards::new(88.0, 80.0);
        let state = g.step(Some(95.0), Some(10.0));
        assert!(state.gpu_hot);
        assert!(!state.nvme_hot);
        let mut g = Guards::new(88.0, 80.0);
        let state = g.step(Some(10.0), Some(85.0));
        assert!(!state.gpu_hot);
        assert!(state.nvme_hot);
    }

    #[test]
    fn gpu_share_override_ratchets_down_at_down_rate() {
        // Ordinary case, well above the floor: current − DOWN_RATE_W.
        assert_eq!(gpu_share_override(50.0, 5.0), 50.0 - DOWN_RATE_W);
    }

    #[test]
    fn gpu_share_override_never_drops_below_the_floor() {
        // current − DOWN_RATE_W would undercut the floor; the floor wins.
        assert_eq!(gpu_share_override(10.0, 5.0), 5.0);
        // Already at the floor: stays exactly at the floor, not below it.
        assert_eq!(gpu_share_override(5.0, 5.0), 5.0);
    }

    #[test]
    fn guard_state_carries_only_the_two_flags() {
        // Exhaustive destructure (no `..`): if anyone adds a third field to
        // GuardState (an effective_target, a budget delta — the deleted
        // design), this stops compiling. That is the point of this test.
        let GuardState { gpu_hot, nvme_hot } = Guards::new(88.0, 80.0).step(None, None);
        assert_eq!((gpu_hot, nvme_hot), (false, false));
    }
}

//! Thermal guards (design doc §2.8): dGPU and NVMe hysteresis, run every
//! sample in any auto mode ahead of the arbiter. Both inputs are
//! `Option<f64>` — an absent reading (dGPU unpowered, NVML N/A, nvme chip
//! missing) makes that guard inactive and clears any hot state it was
//! holding.
//!
//! - **dGPU** drives [`gpu_share_override`]: while hot, the caller ratchets
//!   the GPU's allocator share down toward its LUT floor at
//!   [`crate::control::allocator::DOWN_RATE_W`] per allocator tick, starting
//!   from the present draw. `gpu_hot_c` defaults to 90 °C (exit 85), derived
//!   from the card's own `GPU Target Temperature Specification` of 87 °C
//!   (§Facts, measured 2026-09-08): the card deliberately parks at 87 °C
//!   under sustained load, so any threshold below it fires during normal
//!   gaming.
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

/// Default `gpu_hot_c` (°C). Exit is this minus [`HYSTERESIS_C`] (85 °C).
/// See the module docs for the 87 °C card-spec derivation.
pub const GPU_HOT_C_DEFAULT: f64 = 90.0;
/// Default `nvme_hot_c` (°C). Exit is this minus [`HYSTERESIS_C`] (75 °C).
/// Reporting-only threshold — see the module docs.
pub const NVME_HOT_C_DEFAULT: f64 = 80.0;
/// Hysteresis band shared by both guards: exit = enter − this.
const HYSTERESIS_C: f64 = 5.0;

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
    /// New guards with the given enter thresholds (°C, exit = enter − 5);
    /// both axes start cold.
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
        self.gpu_hot = hysteresis(self.gpu_hot, gpu_temp_c, self.gpu_hot_c);
        self.nvme_hot = hysteresis(self.nvme_hot, nvme_temp_c, self.nvme_hot_c);
        GuardState {
            gpu_hot: self.gpu_hot,
            nvme_hot: self.nvme_hot,
        }
    }
}

/// One axis of enter/exit hysteresis: enters (`true`) once `temp_c` reaches
/// `enter_c`, stays hot through the band, and clears at `enter_c −
/// HYSTERESIS_C`. `None` always returns `false`, regardless of `was_hot`.
fn hysteresis(was_hot: bool, temp_c: Option<f64>, enter_c: f64) -> bool {
    let Some(t) = temp_c else {
        return false;
    };
    if !was_hot && t >= enter_c {
        true
    } else if was_hot && t <= enter_c - HYSTERESIS_C {
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

    #[test]
    fn gpu_hysteresis_enters_at_threshold_and_exits_five_below() {
        let mut g = Guards::new(90.0, 80.0);
        assert!(!g.step(Some(89.9), None).gpu_hot, "below enter: cold");
        assert!(g.step(Some(90.0), None).gpu_hot, "at enter: hot");
        assert!(
            g.step(Some(86.0), None).gpu_hot,
            "in the band (exit < t < enter): stays hot"
        );
        assert!(!g.step(Some(85.0), None).gpu_hot, "at exit: clears");
    }

    #[test]
    fn nvme_hysteresis_enters_at_threshold_and_exits_five_below() {
        let mut g = Guards::new(90.0, 80.0);
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
        let mut g = Guards::new(90.0, 80.0);
        assert!(g.step(Some(95.0), None).gpu_hot, "primed hot");
        let state = g.step(None, None);
        assert!(!state.gpu_hot, "absent reading clears hot state");
        // And it does not silently re-latch hot on the very next tick just
        // because the axis was hot two ticks ago — the guard must have
        // actually reset, not merely reported false once.
        let state = g.step(Some(87.0), None);
        assert!(
            !state.gpu_hot,
            "87 is below the 90 enter threshold: a reset guard stays cold, \
             a guard that only 'reported' false would still be latched hot"
        );
    }

    #[test]
    fn none_reading_deactivates_and_clears_an_already_hot_nvme_guard() {
        let mut g = Guards::new(90.0, 80.0);
        assert!(g.step(None, Some(85.0)).nvme_hot, "primed hot");
        let state = g.step(None, None);
        assert!(!state.nvme_hot, "absent reading clears hot state");
    }

    #[test]
    fn guards_are_independent_axes() {
        // A hot GPU must never flip nvme_hot and vice versa (a shared bool,
        // or thresholds swapped between axes, would fail this).
        let mut g = Guards::new(90.0, 80.0);
        let state = g.step(Some(95.0), Some(10.0));
        assert!(state.gpu_hot);
        assert!(!state.nvme_hot);
        let mut g = Guards::new(90.0, 80.0);
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
        let GuardState { gpu_hot, nvme_hot } = Guards::new(90.0, 80.0).step(None, None);
        assert_eq!((gpu_hot, nvme_hot), (false, false));
    }
}

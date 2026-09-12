//! `FanctrlEmulator` / `ThermalPlant` / `FanPlant` / `ChainedPlant` — the
//! fw-fanctrl replica and chained physical plant driving the closed-loop
//! acceptance sims (design doc §5, Task 17 / fwloop.16).
//!
//! Layout, outermost first:
//! - [`Xorshift32`]: the hand-rolled seeded RNG every plant here draws noise
//!   from — no new crate dependency (Global Constraints).
//! - [`FanctrlEmulator`]: a faithful replica of upstream fw-fanctrl's own
//!   internal loop (`docs/research/05-fw-fanctrl-loop.md` §1) — the boxcar
//!   moving average (reusing [`crate::sensors::ec::EcAverage`], the exact
//!   off-by-one this crate already implements), `eff = min(MA, current)`,
//!   the [`crate::fanctrl::curve::Curve`] duty lookup, and the two upstream
//!   quirks that make the replica's average diverge from the socket's in
//!   ways the instantaneous value hides (no history append while paused;
//!   a hardcoded 50 °C on a scripted sensor-read failure) — the whole
//!   reason `EC MISMATCH` exists.
//! - [`ThermalPlant`]: watts -> controllable EC °C (first-order + dead
//!   time), plus labelled ambient/charger channels and scriptable `gpu_*`
//!   channels, emitted as an [`crate::sensors::ec::EcReading`].
//! - [`FanPlant`]: duty -> RPM via its own seeded table (a separate object
//!   from the controller's [`crate::fanctrl::table::DutyRpmTable`] — design
//!   doc §5), plus an EC-autofan mode driving RPM straight off EC
//!   temperature on the measured staircase (§Facts) whenever the emulator
//!   is not actively steering the fans.
//! - [`ChainedPlant`]: composes the three into a full [`crate::types::Sample`]
//!   per 1 Hz tick, including the demand model (measured draw vs commanded
//!   cap) and the simulated fw-fanctrl poll cadence (`print speed` every
//!   5 s, `print all` every 30 s) that drives `fanctrl_view_changed`.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use crate::fanctrl::client::{FanctrlView, Freshness, compute_freshness};
use crate::fanctrl::curve::{Curve, CurveError};
use crate::fanctrl::table::DutyRpmTable;
use crate::sensors::ec::{EcAverage, EcReading};
use crate::types::Sample;

// --- Xorshift32: hand-rolled seeded RNG (no new crate dependency) --------

/// A small, deterministic xorshift32 PRNG — [`FanPlant`]'s noise source.
/// Same algorithm as the one `fanctrl/table.rs`'s own property test uses,
/// kept as a separate, private copy here rather than shared: that one is
/// `#[cfg(test)]`-local to a single test module, and this module has no
/// need to expose RNG internals beyond the seed a caller passes to
/// [`FanPlant::new`].
struct Xorshift32(u32);

impl Xorshift32 {
    /// A seed of `0` degenerates the xorshift recurrence to an all-zero
    /// fixed point, so it is nudged to a fixed nonzero value instead of
    /// silently producing constant "noise".
    fn new(seed: u32) -> Self {
        Xorshift32(if seed == 0 { 0x9E37_79B9 } else { seed })
    }

    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Uniform in `[lo, hi]` inclusive-ish (float, so the upper bound is
    /// reached only in the limit).
    fn next_f64(&mut self, lo: f64, hi: f64) -> f64 {
        let unit = f64::from(self.next_u32()) / f64::from(u32::MAX);
        lo + unit * (hi - lo)
    }
}

// --- FanctrlEmulator -------------------------------------------------------

/// One tick's raw "current temperature" reading, as fw-fanctrl's own
/// `framework_tool --thermal` call would produce it. `Failed` reproduces
/// upstream's own hardcoded-50°C-on-failure quirk (§Facts /
/// `docs/research/05-fw-fanctrl-loop.md` §1); the emulator applies the
/// substitution, not the caller, so the injected value stays in exactly
/// one place.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SensorRead {
    Ok(f64),
    Failed,
}

/// The hardcoded fallback upstream fw-fanctrl returns on any
/// `framework_tool` failure (§Facts).
const FRAMEWORK_TOOL_FAILURE_C: f64 = 50.0;

/// A faithful replica of upstream fw-fanctrl's own internal control loop
/// (`docs/research/05-fw-fanctrl-loop.md` §1): a single named strategy
/// (curve), a boxcar moving average with fw-fanctrl's off-by-one, and
/// `eff = min(moving_average, current)` truncated through the curve to a
/// commanded duty. Ticks at 1 Hz, driven explicitly by the caller (there is
/// no background thread here — this is a deterministic test double).
pub struct FanctrlEmulator {
    strategy_name: String,
    points: Vec<(f64, u8)>,
    curve: Curve,
    ma_interval: u32,
    average: EcAverage,
    /// Mirrors the socket's `active` field: `true` unless the replica has
    /// been "paused" (`set_active(false)`) — design doc §2.5: `active:
    /// false` is exactly upstream's `pause`, which hands the fans to the
    /// EC's own `--autofanctrl` curve and freezes fw-fanctrl's own loop
    /// (buffer and current reading both) until resumed.
    active: bool,
    /// Set by [`Self::kill_socket`] / [`Self::revive_socket`]: simulates
    /// the daemon process itself being gone (not merely paused). Design
    /// doc §2.5: a killed daemon's unit runs `ExecStopPost --autofanctrl`,
    /// so this is the same fan-plant regime as `active: false`, but
    /// [`Self::view`] reports it as no view at all (an absent socket),
    /// not a reachable-but-paused one.
    socket_dead: bool,
    /// Last raw "current" reading (`temperature` on the emitted view).
    temperature: f64,
    /// Last boxcar mean (`ma_temperature` on the emitted view) — the value
    /// `push` returned on the most recent live tick, i.e. the mean of
    /// samples *before* that tick's own reading was folded in.
    ma_temperature: f64,
    /// Last commanded duty (`speed_pct` on the emitted view).
    speed_pct: u8,
}

impl FanctrlEmulator {
    /// Builds an emulator for one named strategy, starting `active` and
    /// with a live socket. `ma_interval` is the strategy's
    /// `movingAverageInterval` (§Facts: 60 on both live curves), clamped
    /// the same way [`EcAverage::new`] clamps it.
    pub fn new(
        strategy_name: impl Into<String>,
        points: Vec<(f64, u8)>,
        ma_interval: u32,
    ) -> Result<Self, CurveError> {
        let curve = Curve::from_points(points.clone())?;
        Ok(FanctrlEmulator {
            strategy_name: strategy_name.into(),
            points,
            curve,
            ma_interval,
            average: EcAverage::new(ma_interval as usize),
            active: true,
            socket_dead: false,
            temperature: 0.0,
            ma_temperature: 0.0,
            speed_pct: 0,
        })
    }

    /// One 1 Hz tick. While the socket is dead ([`Self::kill_socket`]) or
    /// paused (`active == false`), this is a complete no-op — no history
    /// append, no duty recompute — reproducing upstream's "buffer survives
    /// pause ... the first post-resume duty averages stale samples"
    /// (`docs/research/05-fw-fanctrl-loop.md` §1). Otherwise: resolves
    /// `sensor` (substituting the hardcoded 50 °C on [`SensorRead::Failed`]),
    /// pushes it into the boxcar (fw-fanctrl's own off-by-one: the returned
    /// mean is of samples *before* this one), computes
    /// `eff = min(mean_before_this_sample, current)` — falling back to
    /// `eff = current` on the very first-ever live tick, before the boxcar
    /// has retained anything to average (upstream's deque starts empty;
    /// there is no documented "first tick" behavior to copy, so this is
    /// this replica's own reasonable choice, not a measured fact) — and
    /// truncates `eff` through the curve to the new commanded duty.
    pub fn tick(&mut self, sensor: SensorRead) {
        if self.socket_dead || !self.active {
            return;
        }
        let raw = match sensor {
            SensorRead::Ok(v) => v,
            SensorRead::Failed => FRAMEWORK_TOOL_FAILURE_C,
        };
        let mean_before = self.average.push(raw);
        let ma = mean_before.unwrap_or(raw);
        let eff = ma.min(raw);
        self.temperature = raw;
        self.ma_temperature = ma;
        self.speed_pct = self.curve.duty_at(eff);
    }

    /// Pause (`active: false`) / resume (`active: true`) — design doc
    /// §2.5's `active` field, upstream's `pause`/`resume` command.
    pub fn set_active(&mut self, active: bool) {
        self.active = active;
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Kills the socket: the daemon process itself is gone (design doc
    /// §2.5's `absent` regime), not merely paused. [`Self::view`] returns
    /// `None` while this is set, and [`Self::tick`] stops updating state,
    /// same as being paused.
    pub fn kill_socket(&mut self) {
        self.socket_dead = true;
    }

    pub fn revive_socket(&mut self) {
        self.socket_dead = false;
    }

    pub fn is_socket_dead(&self) -> bool {
        self.socket_dead
    }

    /// True whenever the fan plant should be driven by the EC's own
    /// autofan curve instead of this emulator's commanded duty — design
    /// doc §2.5: "`absent` and `active: false` are therefore one plant
    /// regime, not two."
    pub fn wants_ec_autofan(&self) -> bool {
        self.socket_dead || !self.active
    }

    /// Replaces the curve's points **under the same strategy name** (design
    /// doc §2.3's live curve edit: fw-fanctrl reloads its config, the
    /// resolved strategy's points change, the name does not). Returns the
    /// [`CurveError`] unchanged (and leaves the previous curve in place) if
    /// the new points don't validate.
    pub fn edit_curve_in_place(&mut self, points: Vec<(f64, u8)>) -> Result<(), CurveError> {
        let curve = Curve::from_points(points.clone())?;
        self.points = points;
        self.curve = curve;
        Ok(())
    }

    pub fn strategy_name(&self) -> &str {
        &self.strategy_name
    }

    pub fn speed_pct(&self) -> u8 {
        self.speed_pct
    }

    pub fn ma_interval(&self) -> u32 {
        self.ma_interval
    }

    /// A live snapshot of fw-fanctrl's in-memory state, as `print all`
    /// would report it right now — `None` while the socket is dead
    /// (design doc §2.5). Both stamps are `now`: unlike the real socket
    /// client, this is not a cached, independently-polled view — it is
    /// this instant's live state, stamped by whoever is asking. Callers
    /// simulating the production poll cadence (5 s `print speed`, 30 s
    /// `print all`) decide how often to call this and what to keep between
    /// calls — see [`ChainedPlant`].
    pub fn view(&self, now: Instant) -> Option<FanctrlView> {
        if self.socket_dead {
            return None;
        }
        Some(FanctrlView {
            strategy: self.strategy_name.clone(),
            active: self.active,
            speed_pct: self.speed_pct,
            temperature: self.temperature,
            ma_temperature: self.ma_temperature,
            ma_interval: self.ma_interval,
            curve: self.points.clone(),
            observed_at: now,
            all_observed_at: Some(now),
        })
    }
}

#[cfg(test)]
mod emulator_tests {
    use super::*;

    fn cool16() -> FanctrlEmulator {
        FanctrlEmulator::new(
            "cool16",
            vec![(0.0, 20), (50.0, 20), (60.0, 30), (70.0, 42), (85.0, 100)],
            60,
        )
        .unwrap()
    }

    // --- Step 1: truncation case + boxcar off-by-one ----------------------

    #[test]
    fn reproduces_the_verified_truncation_case_via_the_ma_branch() {
        // §Facts: T_eff 51.8 -> duty 21 on cool16. Drive the boxcar so its
        // pre-push mean is exactly 51.8 (two samples averaging to it), with
        // this tick's own raw reading above that mean so eff picks the MA
        // branch, not the current branch.
        let mut e = cool16();
        e.tick(SensorRead::Ok(50.0));
        e.tick(SensorRead::Ok(53.6)); // mean([50.0]) = 50.0, then buffer [50, 53.6]
        // Pre-push mean of [50.0, 53.6] is 51.8 exactly.
        e.tick(SensorRead::Ok(60.0));
        assert_eq!(e.ma_temperature, 51.8, "boxcar mean should be 51.8");
        assert_eq!(e.temperature, 60.0);
        assert_eq!(
            e.speed_pct(),
            21,
            "eff = min(51.8, 60.0) = 51.8 -> cool16 duty_at(51.8) truncates to 21, not 22"
        );
    }

    #[test]
    fn boxcar_off_by_one_the_mean_used_excludes_this_ticks_own_sample() {
        // interval=3 boxcar via a fresh emulator with a small window, pinned
        // to literal values exactly like ec.rs's own off-by-one test.
        let mut e = FanctrlEmulator::new("t", vec![(0.0, 0), (100.0, 100)], 3).unwrap();
        e.tick(SensorRead::Ok(10.0));
        assert_eq!(
            e.ma_temperature, 10.0,
            "first tick: no prior samples, eff falls back to current"
        );
        e.tick(SensorRead::Ok(20.0));
        assert_eq!(
            e.ma_temperature, 10.0,
            "mean of [10] before this sample is folded in"
        );
        e.tick(SensorRead::Ok(30.0));
        assert_eq!(e.ma_temperature, 15.0, "mean of [10, 20]");
        e.tick(SensorRead::Ok(40.0));
        assert_eq!(
            e.ma_temperature, 20.0,
            "mean of [10, 20, 30] (buffer capped at 3)"
        );
    }

    // --- Step 2: a scripted temperature drop drives eff from `current` ----

    #[test]
    fn a_temperature_drop_drives_eff_from_the_current_branch() {
        let mut e = cool16();
        // Warm the boxcar up high so MA sits well above a later drop.
        for _ in 0..5 {
            e.tick(SensorRead::Ok(80.0));
        }
        assert!(e.ma_temperature > 70.0, "MA should have caught up near 80");
        // A sudden drop: current (30.0) is now far below MA, so
        // eff = min(MA, current) = current, "instant on falling temps"
        // (docs/research/05-fw-fanctrl-loop.md §1).
        e.tick(SensorRead::Ok(30.0));
        assert_eq!(e.temperature, 30.0);
        assert!(
            e.ma_temperature > 30.0,
            "MA should still be lagging above the dropped current"
        );
        assert_eq!(
            e.speed_pct(),
            e.curve.duty_at(30.0),
            "eff must have taken the current branch, not the still-elevated MA"
        );
    }

    // --- Step 3: paused freezes history; a read failure injects 50C -------

    #[test]
    fn a_paused_emulator_stops_appending_history() {
        let mut e = cool16();
        e.tick(SensorRead::Ok(55.0));
        e.tick(SensorRead::Ok(56.0));
        let ma_before_pause = e.ma_temperature;
        let temp_before_pause = e.temperature;
        let duty_before_pause = e.speed_pct();

        e.set_active(false);
        // Ticks while paused must be complete no-ops: no history append, no
        // recomputed duty, no changed temperature/MA -- upstream hands the
        // fans to the EC and its own loop does not run at all.
        for _ in 0..10 {
            e.tick(SensorRead::Ok(99.0));
        }
        assert_eq!(
            e.ma_temperature, ma_before_pause,
            "MA must not move while paused"
        );
        assert_eq!(
            e.temperature, temp_before_pause,
            "current reading must not move while paused"
        );
        assert_eq!(
            e.speed_pct(),
            duty_before_pause,
            "duty must not move while paused"
        );

        e.set_active(true);
        e.tick(SensorRead::Ok(60.0));
        // The very next post-resume tick still averages the STALE
        // pre-pause buffer (docs/research/05-fw-fanctrl-loop.md §1: "the
        // first post-resume duty averages stale samples"), not the 99.0s
        // that were ticked while paused.
        assert_eq!(
            e.ma_temperature, 55.5,
            "post-resume mean should be the pre-pause buffer [55.0, 56.0], not the paused 99.0 ticks"
        );
    }

    #[test]
    fn a_scripted_sensor_read_failure_injects_50c() {
        let mut e = cool16();
        e.tick(SensorRead::Ok(70.0));
        e.tick(SensorRead::Failed);
        assert_eq!(
            e.temperature, 50.0,
            "a failed read reports the hardcoded 50C"
        );
        e.tick(SensorRead::Ok(70.0));
        // The failure's 50.0 must have joined the boxcar too (it is a real
        // sample as far as fw-fanctrl's own loop is concerned) -- mean of
        // [70.0, 50.0] before this third sample is folded in.
        assert_eq!(
            e.ma_temperature, 60.0,
            "the injected 50C must have been retained in the boxcar"
        );
    }

    // --- Step 4: in-place curve edit ---------------------------------------

    #[test]
    fn edit_curve_in_place_keeps_the_strategy_name_and_changes_the_points() {
        let mut e = cool16();
        let t0 = Instant::now();
        e.tick(SensorRead::Ok(65.0));
        let before = e.view(t0).unwrap();
        assert_eq!(before.strategy, "cool16");
        assert_eq!(
            before.curve,
            vec![(0.0, 20), (50.0, 20), (60.0, 30), (70.0, 42), (85.0, 100)]
        );

        let new_points = vec![(0.0, 10), (50.0, 10), (60.0, 50), (85.0, 90)];
        e.edit_curve_in_place(new_points.clone()).unwrap();
        e.tick(SensorRead::Ok(65.0));
        let t1 = t0 + Duration::from_secs(5);
        let after = e.view(t1).unwrap();
        assert_eq!(
            after.strategy, "cool16",
            "strategy name must not change on an in-place edit"
        );
        assert_eq!(
            after.curve, new_points,
            "the emitted view must carry the new points"
        );
        assert_eq!(
            after.all_observed_at,
            Some(t1),
            "a later view() call reflects a later stamp"
        );
        assert_eq!(e.strategy_name(), "cool16");
        assert_eq!(e.ma_interval(), 60);
    }

    #[test]
    fn edit_curve_in_place_rejects_an_invalid_curve_and_keeps_the_old_one() {
        let mut e = cool16();
        let err = e
            .edit_curve_in_place(vec![(0.0, 50), (10.0, 30)])
            .unwrap_err();
        assert_eq!(err, CurveError::DescendingSegment { at_index: 0 });
        let view = e.view(Instant::now()).unwrap();
        assert_eq!(
            view.curve,
            vec![(0.0, 20), (50.0, 20), (60.0, 30), (70.0, 42), (85.0, 100)]
        );
    }

    // --- socket death / active toggling feed wants_ec_autofan --------------

    #[test]
    fn wants_ec_autofan_true_iff_dead_or_inactive() {
        let mut e = cool16();
        assert!(!e.wants_ec_autofan());
        e.set_active(false);
        assert!(e.wants_ec_autofan());
        e.set_active(true);
        assert!(!e.wants_ec_autofan());
        e.kill_socket();
        assert!(e.wants_ec_autofan());
        assert_eq!(
            e.view(Instant::now()),
            None,
            "a dead socket has no view at all"
        );
        e.revive_socket();
        assert!(!e.wants_ec_autofan());
        assert!(e.view(Instant::now()).is_some());
    }

    #[test]
    fn a_dead_socket_also_freezes_ticks() {
        let mut e = cool16();
        e.tick(SensorRead::Ok(55.0));
        let ma_before = e.ma_temperature;
        e.kill_socket();
        e.tick(SensorRead::Ok(90.0));
        assert_eq!(
            e.ma_temperature, ma_before,
            "a dead socket's internal loop is not running either"
        );
    }
}

// --- ThermalPlant -----------------------------------------------------------

/// Every tick in this module is 1 s (design doc §5's "1 s tick", matched
/// throughout the chain so `ThermalPlant`'s dead-time delay line can be
/// measured in whole ticks rather than carrying a separate `dt` parameter
/// every caller would have to keep in lockstep with everyone else's).
pub const TICK_S: f64 = 1.0;

/// FOPDT time constant, s (design doc §5 / §2.4's `tau_s` default).
const THERMAL_TAU_S: f64 = 35.0;
/// FOPDT dead time, s.
const THERMAL_THETA_S: f64 = 20.0;
/// FOPDT steady-state gain, °C per drawn watt.
const THERMAL_K_C_PER_W: f64 = 0.8;

/// CPU raw EC-group plant parameters.  Kept public so robustness scenarios
/// can perturb the physical plant without changing controller defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CpuPlantParams {
    pub tau_s: f64,
    pub k_c_per_w: f64,
    pub theta_eff_s: f64,
}

impl Default for CpuPlantParams {
    fn default() -> Self {
        Self {
            tau_s: THERMAL_TAU_S,
            k_c_per_w: THERMAL_K_C_PER_W,
            theta_eff_s: THERMAL_THETA_S,
        }
    }
}

/// GPU raw EC-group parameters. `k_c_per_mhz` is the controller-facing
/// small-signal gain; its default maps through the physical 0.4 C/W path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpuPlantParams {
    pub tau_s: f64,
    pub k_c_per_mhz: f64,
    pub theta_eff_s: f64,
}

impl GpuPlantParams {
    pub const fn nominal() -> Self {
        Self {
            tau_s: 15.0,
            k_c_per_mhz: 0.02,
            theta_eff_s: 90.0,
        }
    }
}

impl Default for GpuPlantParams {
    fn default() -> Self {
        Self::nominal()
    }
}

/// Per-sample physical fault injection, used internally by [`ChainedPlant`]
/// and exposed for focused plant tests.  Every value is applied before the
/// production EC parser sees the scratch hwmon tree.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalFaults {
    pub cpu_group_present: bool,
    pub gpu_group_present: bool,
    pub cpu_stuck_c: Option<f64>,
    pub gpu_stuck_c: Option<f64>,
    pub ambient_c: Option<f64>,
    pub charger_c: Option<f64>,
    pub raw_only_c: Option<f64>,
    pub unknown_c: Option<f64>,
    pub ec_invalid: bool,
}

impl Default for ThermalFaults {
    fn default() -> Self {
        Self {
            cpu_group_present: true,
            gpu_group_present: true,
            cpu_stuck_c: None,
            gpu_stuck_c: None,
            ambient_c: None,
            charger_c: None,
            raw_only_c: None,
            unknown_c: None,
            ec_invalid: false,
        }
    }
}

/// watts -> controllable EC °C (design doc §5: first order, τ 35 s, θ 20 s,
/// K 0.8 °C/W, plus an ambient offset), with separately labelled
/// `ambient`/`charger` channels and scriptable `gpu_*` channels, all
/// emitted together as a real [`EcReading`].
///
/// [`EcLabel`] has no public constructor (by design: outside
/// `sensors::ec`, nothing should be able to fabricate a label that did not
/// come from an actual sysfs read) and this task's `filesTouched` is
/// `src/test_support/plant.rs` only, so this plant does not build an
/// `EcReading` field-by-field. Instead it round-trips through
/// [`EcReading::read`] itself, the same public entry point production code
/// uses: each tick it writes the current channel values into a small
/// scratch directory laid out exactly like a `cros_ec` hwmon chip
/// (`tempN_label` / `tempN_input`, the ENODATA convention for an absent
/// `gpu_*` reading) and reads it straight back. This is slower than
/// building the struct directly, but it means every rounding, drop and
/// tie-break rule the replica applies is *exactly* production's, not a
/// second copy of it that could drift.
pub struct ThermalPlant {
    cpu_params: CpuPlantParams,
    gpu_params: GpuPlantParams,
    /// Steady-state controllable temperature at zero drawn watts.
    ambient_base_c: f64,
    /// Pending watts inputs not yet past the dead time, oldest first.
    cpu_delay: VecDeque<(f64, f64)>,
    gpu_delay: VecDeque<(f64, f64)>,
    /// Exact carried scalar delay for cap/fraction scripts.  It remains
    /// separate from the elapsed-time per-device queues by design.
    legacy_delay: VecDeque<f64>,
    controllable_c: f64,
    gpu_group_c: f64,
    ambient_c: f64,
    charger_c: f64,
    /// A single representative EC `gpu_*` channel (labelled
    /// `gpu_amb_f75303@4d`) — §Facts found these sensors never report on
    /// this machine even with the dGPU powered, so exercising "one comes
    /// alive" only needs one scriptable channel, matching
    /// `sensors::ec`'s own synthetic test of the same fact.
    gpu_ec_c: Option<f64>,
    dir: PathBuf,
}

impl ThermalPlant {
    /// `ambient_base_c` is the controllable channel's steady-state
    /// temperature at zero drawn watts (also used to seed `controllable_c`
    /// and, absent a `set_ambient_charger` call, the ambient/charger
    /// channels a few degrees below it).
    pub fn new(ambient_base_c: f64) -> Self {
        static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir =
            std::env::temp_dir().join(format!("bzf-plant-thermal-{}-{n}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("scratch hwmon dir must be creatable");
        for (n, label) in [
            "cpu@4c",
            "apu_f75303@4d",
            "ambient_f75303@4d",
            "charger_f75303@4d",
            "gpu_vr_f75303@4d",
            "gpu_vram_f75303@4d",
            "gpu_amb_f75303@4d",
            "gpu_temp@40",
            "unknown_thermal",
        ]
        .iter()
        .enumerate()
        {
            std::fs::write(
                dir.join(format!("temp{}_label", n + 1)),
                format!("{label}\n"),
            )
            .unwrap();
        }
        ThermalPlant {
            cpu_params: CpuPlantParams::default(),
            gpu_params: GpuPlantParams::nominal(),
            ambient_base_c,
            cpu_delay: VecDeque::new(),
            gpu_delay: VecDeque::new(),
            legacy_delay: VecDeque::new(),
            controllable_c: ambient_base_c,
            gpu_group_c: ambient_base_c,
            ambient_c: ambient_base_c - 2.0,
            charger_c: ambient_base_c - 4.0,
            gpu_ec_c: None,
            dir,
        }
    }

    /// Scripts the uncontrollable `ambient`/`charger` channels (design doc
    /// §5: "separately labelled ambient/charger channels").
    pub fn set_ambient_charger(&mut self, ambient_c: f64, charger_c: f64) {
        self.ambient_c = ambient_c;
        self.charger_c = charger_c;
    }

    /// Scripts the representative EC `gpu_*` channel: `Some` makes it a
    /// live positive reading (the "a future firmware wakes it up" case
    /// `sensors::ec` already tests for), `None` removes its `_input` file
    /// entirely (the ENODATA convention, matching §Facts' actual measured
    /// behavior with the dGPU powered).
    pub fn set_gpu_ec(&mut self, gpu_ec_c: Option<f64>) {
        self.gpu_ec_c = gpu_ec_c;
    }

    pub fn set_cpu_plant_params(&mut self, params: CpuPlantParams) {
        self.cpu_params = params;
    }

    pub fn set_gpu_plant_params(&mut self, params: GpuPlantParams) {
        self.gpu_params = params;
    }

    pub fn gpu_plant_params(&self) -> GpuPlantParams {
        self.gpu_params
    }

    pub fn controllable_c(&self) -> f64 {
        self.controllable_c
    }

    /// One 1 s tick: `watts` is the **drawn** power (post-demand-model —
    /// see [`ChainedPlant`]'s doc comment), not a commanded cap. Advances
    /// the dead-time delay line and the first-order lag, writes the four
    /// channels into the scratch hwmon tree, and reads a fresh
    /// [`EcReading`] back.
    pub fn tick(&mut self, watts: f64) -> EcReading {
        self.tick_legacy_scalar(
            watts,
            ThermalFaults {
                gpu_group_present: self.gpu_ec_c.is_some(),
                ..ThermalFaults::default()
            },
        )
        .expect("legacy thermal tick always has a CPU EC channel")
    }

    /// The carried scalar plant uses an integer-tick delay and explicit Euler
    /// update.  Old cap/fraction acceptance traces are phase-sensitive, so
    /// routing them through the analytic per-device model changes outcomes.
    fn tick_legacy_scalar(&mut self, watts: f64, faults: ThermalFaults) -> Option<EcReading> {
        self.legacy_delay.push_back(watts);
        let delayed = if self.legacy_delay.len() > (THERMAL_THETA_S / TICK_S) as usize {
            self.legacy_delay
                .pop_front()
                .expect("the just-appended scalar delay is nonempty")
        } else {
            0.0
        };
        let target = self.ambient_base_c + THERMAL_K_C_PER_W * delayed;
        self.controllable_c += (target - self.controllable_c) * (TICK_S / THERMAL_TAU_S);
        self.emit_reading(faults)
    }

    /// Advances independent CPU/GPU group nodes.  The GPU target deliberately
    /// computes its heat through `draw_w * 0.4 C/W`: converting the tunable
    /// MHz gain to a W gain preserves that physical path at every grid point.
    pub fn tick_nodes(
        &mut self,
        cpu_w: f64,
        gpu_draw_w: f64,
        dt_s: f64,
        faults: ThermalFaults,
    ) -> Option<EcReading> {
        let dt_s = if dt_s.is_finite() {
            dt_s.max(0.0)
        } else {
            TICK_S
        };
        self.cpu_delay.push_back((cpu_w, dt_s));
        self.gpu_delay.push_back((gpu_draw_w, dt_s));
        let delayed = |queue: &mut VecDeque<(f64, f64)>, theta_s: f64| {
            let theta_s = theta_s.max(0.0);
            let mut elapsed_past_delay_s: f64 =
                queue.iter().map(|(_, held)| *held).sum::<f64>() - theta_s;
            let mut output = 0.0;
            while elapsed_past_delay_s > 0.0 {
                let Some((value, held)) = queue.pop_front() else {
                    break;
                };
                output = value;
                if elapsed_past_delay_s < held {
                    queue.push_front((value, held - elapsed_past_delay_s));
                    break;
                }
                elapsed_past_delay_s -= held;
            }
            output
        };
        let delayed_cpu_w = delayed(&mut self.cpu_delay, self.cpu_params.theta_eff_s);
        let delayed_gpu_w = delayed(&mut self.gpu_delay, self.gpu_params.theta_eff_s);
        let cpu_cross_c = 0.1 * (self.gpu_group_c - self.ambient_base_c);
        let gpu_cross_c = 0.1 * (self.controllable_c - self.ambient_base_c);
        let cpu_target =
            self.ambient_base_c + self.cpu_params.k_c_per_w * delayed_cpu_w + cpu_cross_c;
        // k=.02 C/MHz corresponds to .4 C/W at the measured 0.05 W/MHz
        // slope.  Keeping 0.4 explicit makes the heat path auditable.
        let gpu_heat_c = delayed_gpu_w * 0.4 * (self.gpu_params.k_c_per_mhz / 0.02);
        let gpu_target = self.ambient_base_c + gpu_heat_c + gpu_cross_c;
        let update = |node: &mut f64, target: f64, tau_s: f64| {
            let tau_s = tau_s.max(f64::EPSILON);
            *node += (target - *node) * (1.0 - (-dt_s / tau_s).exp());
        };
        update(&mut self.controllable_c, cpu_target, self.cpu_params.tau_s);
        update(&mut self.gpu_group_c, gpu_target, self.gpu_params.tau_s);

        self.emit_reading(faults)
    }

    fn emit_reading(&mut self, faults: ThermalFaults) -> Option<EcReading> {
        let write_milli = |name: &str, c: f64| {
            std::fs::write(
                self.dir.join(name),
                format!("{}\n", (c * 1000.0).round() as i64),
            )
            .unwrap();
        };
        let write_or_remove = |name: &str, value: Option<f64>| match value {
            Some(value) => write_milli(name, value),
            None => {
                let _ = std::fs::remove_file(self.dir.join(name));
            }
        };
        let cpu_c = faults.cpu_stuck_c.unwrap_or(self.controllable_c);
        let gpu_c = faults
            .gpu_stuck_c
            .or(self.gpu_ec_c)
            .unwrap_or(self.gpu_group_c);
        write_or_remove(
            "temp1_input",
            (!faults.ec_invalid && faults.cpu_group_present).then_some(cpu_c),
        );
        write_or_remove(
            "temp2_input",
            (!faults.ec_invalid && faults.cpu_group_present).then_some(cpu_c - 0.4),
        );
        write_or_remove(
            "temp3_input",
            (!faults.ec_invalid).then_some(faults.ambient_c.unwrap_or(self.ambient_c)),
        );
        write_or_remove(
            "temp4_input",
            (!faults.ec_invalid).then_some(faults.charger_c.unwrap_or(self.charger_c)),
        );
        for (index, name) in ["temp5_input", "temp6_input", "temp7_input", "temp8_input"]
            .iter()
            .enumerate()
        {
            let legacy_only = self.gpu_ec_c.is_some() && index != 2;
            write_or_remove(
                name,
                (!faults.ec_invalid && faults.gpu_group_present && !legacy_only).then_some(gpu_c),
            );
        }
        // Raw-only values deliberately share a GPU label: the EC parser
        // excludes them from control while retaining them for reconciliation.
        if let Some(raw) = faults.raw_only_c {
            write_milli("temp5_input", raw);
        }
        write_or_remove(
            "temp9_input",
            (!faults.ec_invalid).then_some(faults.unknown_c).flatten(),
        );
        EcReading::read(&self.dir)
    }
}

impl Drop for ThermalPlant {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[cfg(test)]
mod thermal_plant_tests {
    use super::*;

    // --- Step 6: FOPDT dead time + first-order lag -------------------------

    #[test]
    fn a_watts_step_does_not_move_the_controllable_channel_before_theta() {
        let mut p = ThermalPlant::new(40.0);
        let before = p.controllable_c();
        for _ in 0..19 {
            p.tick(50.0); // a real step, but still inside the 20s dead time
        }
        assert!(
            (p.controllable_c() - before).abs() < 1e-9,
            "controllable channel must not move before theta=20s elapses, got {}",
            p.controllable_c()
        );
    }

    #[test]
    fn the_controllable_channel_approaches_the_fopdt_steady_state_after_theta() {
        let mut p = ThermalPlant::new(40.0);
        let step_w = 10.0; // K*step = 8.0C steady-state rise
        for _ in 0..600 {
            p.tick(step_w);
        }
        let expected_ss = 40.0 + THERMAL_K_C_PER_W * step_w;
        assert!(
            (p.controllable_c() - expected_ss).abs() < 0.1,
            "after 600s (>> tau=35s past theta) should be within 0.1C of steady state {expected_ss}, got {}",
            p.controllable_c()
        );
    }

    #[test]
    fn emitted_ec_reading_carries_the_labelled_channels() {
        let mut p = ThermalPlant::new(40.0);
        p.set_ambient_charger(45.0, 30.0);
        let reading = p.tick(0.0);
        // Ambient (45.0) exceeds the still-baseline controllable channel
        // (40.0, watts have not reached it yet) and the charger (30.0).
        assert_eq!(
            reading.max_c, 45,
            "ambient should be the argmax before any watts arrive"
        );
        assert_eq!(reading.argmax.as_str(), "ambient_f75303@4d");
        let labels: Vec<&str> = reading.all.iter().map(|(l, _)| l.as_str()).collect();
        assert!(labels.contains(&"cpu@4c"));
        assert!(labels.contains(&"ambient_f75303@4d"));
        assert!(labels.contains(&"charger_f75303@4d"));
        assert!(
            !labels.iter().any(|l| l.starts_with("gpu")),
            "no gpu_* reading was scripted"
        );
    }

    #[test]
    fn a_scripted_gpu_ec_reading_joins_the_max_when_set_and_disappears_when_cleared() {
        let mut p = ThermalPlant::new(40.0);
        p.set_gpu_ec(Some(95.0));
        let reading = p.tick(0.0);
        assert_eq!(reading.max_c, 95);
        assert_eq!(reading.argmax.as_str(), "gpu_amb_f75303@4d");
        assert!(reading.argmax.is_controllable());

        p.set_gpu_ec(None);
        let reading = p.tick(0.0);
        assert!(
            !reading
                .all
                .iter()
                .any(|(l, _)| l.as_str().starts_with("gpu")),
            "clearing the scripted gpu_* reading must remove it (ENODATA), not zero it"
        );
    }
}

// --- FanPlant ----------------------------------------------------------

/// One-tick momentum-kick bump on positive slew (design doc §5, "the
/// 2026-07-14 finding"), in RPM. More than twice [`FAN_NOISE_RPM`] so a
/// kicked tick is *unconditionally* outside the noise band regardless of
/// which way the noise draw falls (`kick - FAN_NOISE_RPM > FAN_NOISE_RPM`),
/// not merely on the noise draws a particular seed happens to produce.
const MOMENTUM_KICK_RPM: f64 = 200.0;
/// Symmetric RPM noise band (design doc §5: "±90 RPM noise").
const FAN_NOISE_RPM: f64 = 90.0;

/// The measured EC-autofan staircase (§Facts, 2026-09-08: fw-fanctrl
/// paused, CPU load ramp, 300 samples). Anchor points in ascending EC max
/// °C; `ec_autofan_rpm_base` flat-clamps below 61 and above 73.
///
/// **The 67–73 °C plateau (4748 RPM) is well sampled (n=102 at 71 °C) and
/// load-bearing for the Mode-B-under-`active:false` no-authority result —
/// treat it as measured.** The 61–64 °C segment is thin (n=4 at 64 °C);
/// its rising-branch shape and any hysteresis width are **not**
/// established (§Facts limitation), so callers must not derive claims
/// about the real EC from that part, only from the plateau. This module
/// models it as a straight-line interpolation through the four measured
/// medians purely so the plant has *some* deterministic value to return
/// there — an approximation, not a second measurement.
const EC_AUTOFAN_STAIRCASE: [(f64, f64); 5] = [
    (61.0, 4096.0),
    (62.0, 4096.0),
    (63.0, 4520.0),
    (64.0, 4658.0),
    (67.0, 4748.0),
];

/// Deterministic (noise-free) EC-autofan RPM at a given EC max temperature:
/// piecewise-linear through [`EC_AUTOFAN_STAIRCASE`], flat-clamped below
/// 61 °C (to the lowest measured point — nothing is measured below there)
/// and at/above 67 °C (the measured plateau, which §Facts's 71 °C sampling
/// says continues flat through 73 °C and this plant assumes holds beyond
/// that too, absent any measurement suggesting otherwise).
fn ec_autofan_rpm_base(ec_max_c: f64) -> f64 {
    let (first_t, first_r) = EC_AUTOFAN_STAIRCASE[0];
    if ec_max_c <= first_t {
        return first_r;
    }
    let (last_t, last_r) = EC_AUTOFAN_STAIRCASE[EC_AUTOFAN_STAIRCASE.len() - 1];
    if ec_max_c >= last_t {
        return last_r;
    }
    for w in EC_AUTOFAN_STAIRCASE.windows(2) {
        let (t0, r0) = w[0];
        let (t1, r1) = w[1];
        if ec_max_c <= t1 {
            let frac = (ec_max_c - t0) / (t1 - t0);
            return r0 + frac * (r1 - r0);
        }
    }
    last_r // unreachable given the >= last_t clamp above
}

/// duty -> RPM (design doc §5): its own seeded table (a separate object
/// from the controller's [`DutyRpmTable`] — "the plant's table is
/// therefore a separate object from the controller's seed", so passive
/// refinement has something real to converge toward), a configurable
/// per-duty offset, a one-sided momentum kick on positive slew, ±90 RPM
/// noise from a seeded [`Xorshift32`], and an EC-autofan mode for whenever
/// [`FanctrlEmulator::wants_ec_autofan`] is true.
pub struct FanPlant {
    table: DutyRpmTable,
    offset_rpm: f64,
    /// The previous tick's noise-free base RPM (whichever mode produced
    /// it), used only to detect a positive slew for the momentum kick.
    prev_base_rpm: f64,
    rng: Xorshift32,
}

impl FanPlant {
    pub fn new(seed: u32) -> Self {
        FanPlant {
            table: DutyRpmTable::default(),
            offset_rpm: 0.0,
            prev_base_rpm: 0.0,
            rng: Xorshift32::new(seed),
        }
    }

    /// A uniform per-duty RPM bias applied to every table lookup (design
    /// doc §5: "a configurable per-duty offset" that "shifts steady RPM by
    /// the configured amount").
    pub fn set_offset_rpm(&mut self, offset_rpm: f64) {
        self.offset_rpm = offset_rpm;
    }

    /// The noise-free, kick-free RPM this plant's table resolves `duty`
    /// to, offset included. Exposed directly (not only through [`Self::tick`])
    /// so the offset acceptance criterion ("shifts steady RPM by *exactly*
    /// the configured amount") can be checked without averaging out noise.
    pub fn base_rpm_for_duty(&self, duty: u8) -> f64 {
        self.table.rpm_for_duty(duty) + self.offset_rpm
    }

    /// One 1 s tick under normal (fw-fanctrl-commanded) operation: `duty`
    /// is the emulator's current `speed_pct`.
    pub fn tick(&mut self, duty: u8) -> f64 {
        let base = self.base_rpm_for_duty(duty);
        self.step(base)
    }

    /// One 1 s tick under EC-autofan operation (design doc §5 / §2.5):
    /// `ec_max_c` drives RPM straight off the measured staircase instead
    /// of any commanded duty.
    pub fn tick_ec_autofan(&mut self, ec_max_c: f64) -> f64 {
        let base = ec_autofan_rpm_base(ec_max_c);
        self.step(base)
    }

    /// Shared tail of both tick variants: the one-sided momentum kick
    /// (only on a positive slew in the noise-free base, whichever mode
    /// produced it) plus noise, floored at 0 RPM (a physical fan cannot
    /// spin at a negative speed, though the offset/noise combination could
    /// otherwise drive the sum below zero at a very low duty).
    fn step(&mut self, base: f64) -> f64 {
        let kick = if base > self.prev_base_rpm {
            MOMENTUM_KICK_RPM
        } else {
            0.0
        };
        self.prev_base_rpm = base;
        let noise = self.rng.next_f64(-FAN_NOISE_RPM, FAN_NOISE_RPM);
        (base + kick + noise).max(0.0)
    }
}

#[cfg(test)]
mod fan_plant_tests {
    use super::*;

    // --- Step 7: table offset, momentum kick, noise -------------------------

    #[test]
    fn the_configured_offset_shifts_steady_rpm_by_exactly_that_amount() {
        let mut p = FanPlant::new(1);
        let unshifted = p.base_rpm_for_duty(30);
        p.set_offset_rpm(200.0);
        assert_eq!(p.base_rpm_for_duty(30), unshifted + 200.0);
        p.set_offset_rpm(-75.0);
        assert_eq!(p.base_rpm_for_duty(30), unshifted - 75.0);
    }

    #[test]
    fn the_momentum_kick_only_fires_on_a_positive_slew() {
        let mut p = FanPlant::new(42);
        // Settle at duty 20 first so prev_base_rpm reflects a real reading,
        // not the constructor's 0.0 placeholder (which would itself look
        // like a positive slew on the very first tick).
        p.tick(20);
        let base_20 = p.base_rpm_for_duty(20);

        // A rise to duty 40 is a positive slew: output must exceed
        // base+max noise, which only the kick can explain.
        let base_40 = p.base_rpm_for_duty(40);
        assert!(
            base_40 > base_20,
            "test setup: duty 40 must be a real rise over duty 20"
        );
        let risen = p.tick(40);
        assert!(
            risen > base_40 + FAN_NOISE_RPM,
            "a positive slew must be kicked above the noise band: got {risen}, base {base_40}"
        );

        // A fall back to duty 20 must NOT be kicked: output stays inside
        // the base +/- noise band.
        let fallen = p.tick(20);
        assert!(
            (fallen - base_20).abs() <= FAN_NOISE_RPM,
            "a negative slew must not be kicked: got {fallen}, base {base_20}"
        );

        // Holding steady at the same duty is flat, not a positive slew:
        // no kick either.
        let held = p.tick(20);
        assert!(
            (held - base_20).abs() <= FAN_NOISE_RPM,
            "a flat slew (same duty) must not be kicked: got {held}, base {base_20}"
        );
    }

    #[test]
    fn noise_is_bounded_and_reproducible_from_the_seed() {
        let mut a = FanPlant::new(777);
        let mut b = FanPlant::new(777);
        let mut seq_a = Vec::new();
        let mut seq_b = Vec::new();
        for duty in [20, 20, 20, 40, 40, 30, 30] {
            seq_a.push(a.tick(duty));
            seq_b.push(b.tick(duty));
        }
        assert_eq!(
            seq_a, seq_b,
            "two plants with the same seed and the same duty script must produce identical output"
        );

        let mut c = FanPlant::new(31415);
        let base = c.base_rpm_for_duty(30);
        c.tick(30); // establish prev_base_rpm = base so the next tick is flat (no kick)
        let steady = c.tick(30);
        assert!(
            (steady - base).abs() <= FAN_NOISE_RPM,
            "flat-duty noise must stay within +/-{FAN_NOISE_RPM} RPM of the base: got {steady}, base {base}"
        );

        // A different seed must not reproduce the same sequence (sanity:
        // this is really drawing from the RNG, not returning a constant).
        let mut d = FanPlant::new(2718);
        let seq_d: Vec<f64> = [20, 20, 20, 40, 40, 30, 30]
            .iter()
            .map(|&du| d.tick(du))
            .collect();
        assert_ne!(seq_a, seq_d, "a different seed must diverge from seq_a");
    }

    // --- Step 8 (EC-autofan RPM curve itself; wiring lives in ChainedPlant) -

    #[test]
    fn ec_autofan_plateau_values_are_exact() {
        // §Facts: flat at ~4748 RPM across 67-73C -- the well-sampled,
        // load-bearing part. Asserted exactly, per the brief.
        assert_eq!(ec_autofan_rpm_base(67.0), 4748.0);
        assert_eq!(ec_autofan_rpm_base(70.0), 4748.0);
        assert_eq!(ec_autofan_rpm_base(71.0), 4748.0);
        assert_eq!(ec_autofan_rpm_base(73.0), 4748.0);
        assert_eq!(
            ec_autofan_rpm_base(80.0),
            4748.0,
            "plateau assumed to continue past 73C"
        );
    }

    #[test]
    fn ec_autofan_steep_segment_medians_are_reproduced_loosely() {
        // §Facts: thin data (n=4 at 64C), rising branch/hysteresis not
        // established -- only the medians themselves are asserted, and
        // only approximately (this plant's straight-line interpolation
        // between them, not a second measurement).
        assert_eq!(ec_autofan_rpm_base(61.0), 4096.0);
        assert_eq!(ec_autofan_rpm_base(62.0), 4096.0);
        assert_eq!(ec_autofan_rpm_base(63.0), 4520.0);
        assert_eq!(ec_autofan_rpm_base(64.0), 4658.0);
        // Between 64 and 67 (not directly measured): must land strictly
        // between the two bracketing medians, nothing more precise claimed.
        let between = ec_autofan_rpm_base(65.5);
        assert!(
            (4658.0..=4748.0).contains(&between),
            "got {between}, expected loosely between the 64C and 67C medians"
        );
    }

    #[test]
    fn ec_autofan_is_far_above_quiet16_at_a_higher_temperature() {
        // §Facts: "the EC is far more aggressive than quiet16 (4748 RPM
        // where fw-fanctrl asks 2649 at a *higher* temperature)". Not a
        // literal reproduction of that exact pair (this is the plant's
        // duty->RPM table, not quiet16's curve), but the same qualitative
        // shape must hold: the EC plateau is far above anything the
        // measured duty table produces even at its own ceiling.
        let table = DutyRpmTable::default();
        assert!(ec_autofan_rpm_base(71.0) > table.rpm_for_duty(52));
    }
}

// --- ChainedPlant ------------------------------------------------------

/// Simulated `print speed` cadence (design doc §2.1: every allocator tick,
/// 5 s) — how often [`ChainedPlant::tick`] refreshes only `speed_pct` on
/// the carried-forward view, leaving every other `All`-only field stale.
const SPEED_POLL_EVERY_TICKS: u64 = 5;
/// Simulated `print all` cadence (design doc §2.1: 30 s) — how often a
/// full view refresh happens, which is the only thing that can move
/// `all_observed_at` and therefore `fanctrl_view_changed`.
const ALL_POLL_EVERY_TICKS: u64 = 30;

/// Every scripted input a scenario can drive on one [`ChainedPlant::tick`]
/// (design doc §5). `Default` is a quiescent, fully-alive, zero-power
/// tick — a scenario only needs to set the fields it cares about.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TickScript {
    /// Commanded CPU power cap, W.
    pub cpu_cap_w: f64,
    /// Commanded GPU power cap, W.
    pub gpu_cap_w: f64,
    /// Fraction of `cpu_cap_w` actually drawn this tick, `[0, 1]` (design
    /// doc §5's demand model: "how much of the commanded cap is actually
    /// drawn"). `1.0` = fully demand-saturated; lower values are a lull.
    pub cpu_demand_frac: f64,
    /// Fraction of `gpu_cap_w` actually drawn this tick, `[0, 1]`.
    pub gpu_demand_frac: f64,
    pub cpu_util_pct: f64,
    pub gpu_util_pct: f64,
    pub gpu_sm_mhz: f64,
    /// Revision-4 GPU physical model inputs.  When both are `Some`, they
    /// supersede the legacy cap/fraction pair while preserving it for older
    /// scenario callers.
    pub gpu_lock_mhz: Option<f64>,
    pub gpu_load_level: Option<f64>,
    /// Independent guard-driving die/Tctl inputs.  `None` follows the
    /// physical node, which keeps old tests deterministic.
    pub cpu_tctl_c: Option<f64>,
    /// `None` = dGPU unpowered/unsensed this tick — gates `gpu_w`,
    /// `gpu_sm_mhz` and their validity flags to the same "absent" reading
    /// (`Sample`'s own convention: 0.0 + `false`), regardless of
    /// `gpu_cap_w`/`gpu_demand_frac`, since an unpowered card cannot draw
    /// power either.
    pub gpu_temp_c: Option<f64>,
    pub nvme_temp_c: Option<f64>,
    /// A scripted `framework_tool` failure this tick (design doc §5's
    /// upstream quirk: injects the hardcoded 50 °C into the emulator).
    pub sensor_read_failed: bool,
    /// A scripted resume-from-suspend edge, passed straight through to
    /// `Sample.resumed`.
    pub resumed: bool,
    /// A scripted socket death this tick (design doc §2.5's `absent`
    /// regime) — independent of the emulator's own `active` flag; see
    /// [`FanctrlEmulator::wants_ec_autofan`].
    pub socket_dead: bool,
    pub on_ac: bool,
    pub cpu_group_present: bool,
    pub gpu_group_present: bool,
    pub cpu_stuck_c: Option<f64>,
    pub gpu_stuck_c: Option<f64>,
    pub ambient_c: Option<f64>,
    pub charger_c: Option<f64>,
    pub raw_only_c: Option<f64>,
    pub unknown_c: Option<f64>,
    pub fan_outage: bool,
    pub ec_invalid: bool,
    pub force_stale_view: bool,
    pub force_view_changed: bool,
    pub gpu_draw_available: bool,
    /// A scripted monotonic gap for resume/backlog scenarios.  The physical
    /// nodes use the same elapsed interval and the returned sample carries it.
    pub elapsed_s: f64,
}

impl Default for TickScript {
    fn default() -> Self {
        TickScript {
            cpu_cap_w: 0.0,
            gpu_cap_w: 0.0,
            cpu_demand_frac: 1.0,
            gpu_demand_frac: 1.0,
            cpu_util_pct: 0.0,
            gpu_util_pct: 0.0,
            gpu_sm_mhz: 0.0,
            gpu_lock_mhz: None,
            gpu_load_level: None,
            cpu_tctl_c: None,
            gpu_temp_c: None,
            nvme_temp_c: None,
            sensor_read_failed: false,
            resumed: false,
            socket_dead: false,
            on_ac: true,
            cpu_group_present: true,
            gpu_group_present: true,
            cpu_stuck_c: None,
            gpu_stuck_c: None,
            ambient_c: None,
            charger_c: None,
            raw_only_c: None,
            unknown_c: None,
            fan_outage: false,
            ec_invalid: false,
            force_stale_view: false,
            force_view_changed: false,
            gpu_draw_available: true,
            elapsed_s: TICK_S,
        }
    }
}

/// September full-load draw curve.  The final segment represents the power
/// limit plateau rather than a calibration LUT.
const GPU_FULL_LOAD_POINTS: [(f64, f64); 8] = [
    (1000.0, 45.0),
    (1197.0, 49.3),
    (1402.0, 53.5),
    (1612.0, 64.2),
    (1807.0, 75.9),
    (1995.0, 90.8),
    (2143.0, 99.4),
    (3090.0, 100.0),
];

/// Full-load GPU draw at a lock.  Values outside the measured range clamp to
/// its endpoint rather than inventing unmeasured power behavior.
pub fn gpu_full_load_power_w(clock_mhz: f64) -> f64 {
    let clock_mhz = clock_mhz.clamp(1000.0, 3090.0);
    for pair in GPU_FULL_LOAD_POINTS.windows(2) {
        let [(x0, y0), (x1, y1)] = pair else {
            unreachable!()
        };
        if clock_mhz <= *x1 {
            let fraction = (clock_mhz - *x0) / (*x1 - *x0);
            return *y0 + fraction * (*y1 - *y0);
        }
    }
    100.0
}

/// Clock a fully loaded power-limited card can report.  It reaches the knee
/// at full load, making reported clock `min(lock, power_limit_clock)`.
pub fn gpu_clock_at_power_limit(load_level: f64) -> f64 {
    let load_level = load_level.clamp(0.0, 1.0);
    3090.0 - (3090.0 - 2143.0) * load_level
}

/// Composes [`FanctrlEmulator`], [`ThermalPlant`] and [`FanPlant`] into a
/// full [`Sample`] per 1 Hz tick (design doc §5) — the plant Task 22's
/// closed-loop acceptance sims drive the controller against.
///
/// **Draw, not cap, is what heats the plant.** `TickScript`'s demand
/// fractions gate the commanded caps down to a measured draw *before* that
/// power reaches [`ThermalPlant`] — without this, a controller under test
/// could raise its cap indefinitely while nothing actually warms up, which
/// is exactly the demand-starved wind-up scenario `fwloop.24`'s spike
/// needs the plant to be able to produce (design doc §2.4).
pub struct ChainedPlant {
    emulator: FanctrlEmulator,
    thermal: ThermalPlant,
    fan: FanPlant,
    t_mono: f64,
    base_instant: Instant,
    tick_count: u64,
    /// The view embedded in the most recently emitted `Sample.fanctrl` —
    /// carried forward across ticks exactly like the production
    /// `FanctrlPoller`'s cache: a failed poll (dead socket) leaves it
    /// untouched rather than clearing it, matching
    /// `UnixFanctrlClient::poll`'s own "leaves the view untouched" error
    /// path. `Sample.fanctrl_freshness` (via `compute_freshness`), not
    /// this field's presence, is what tells a consumer the socket is gone.
    last_view: Option<FanctrlView>,
    last_all_observed_at: Option<Instant>,
}

impl ChainedPlant {
    /// `strategy_name`/`points`/`ma_interval` seed the [`FanctrlEmulator`];
    /// `ambient_base_c` seeds the [`ThermalPlant`] (and its uncontrollable
    /// ambient/charger channels, a few degrees below it, adjustable
    /// afterward via [`Self::thermal_mut`]); `fan_seed` seeds the
    /// [`FanPlant`]'s noise.
    pub fn new(
        strategy_name: impl Into<String>,
        points: Vec<(f64, u8)>,
        ma_interval: u32,
        ambient_base_c: f64,
        fan_seed: u32,
    ) -> Result<Self, CurveError> {
        Ok(ChainedPlant {
            emulator: FanctrlEmulator::new(strategy_name, points, ma_interval)?,
            thermal: ThermalPlant::new(ambient_base_c),
            fan: FanPlant::new(fan_seed),
            t_mono: 0.0,
            base_instant: Instant::now(),
            tick_count: 0,
            last_view: None,
            last_all_observed_at: None,
        })
    }

    pub fn emulator_mut(&mut self) -> &mut FanctrlEmulator {
        &mut self.emulator
    }

    pub fn thermal_mut(&mut self) -> &mut ThermalPlant {
        &mut self.thermal
    }

    pub fn set_gpu_plant_params(&mut self, params: GpuPlantParams) {
        self.thermal.set_gpu_plant_params(params);
    }

    pub fn set_cpu_plant_params(&mut self, params: CpuPlantParams) {
        self.thermal.set_cpu_plant_params(params);
    }

    pub fn gpu_plant_params(&self) -> GpuPlantParams {
        self.thermal.gpu_plant_params()
    }

    pub fn fan_mut(&mut self) -> &mut FanPlant {
        &mut self.fan
    }

    pub fn t_mono(&self) -> f64 {
        self.t_mono
    }

    /// One 1 Hz tick: advances every sub-plant, applies the demand model,
    /// drives the fan (fw-fanctrl-commanded or EC-autofan, per
    /// [`FanctrlEmulator::wants_ec_autofan`]), simulates the production
    /// poll cadence, and returns the resulting [`Sample`].
    pub fn tick(&mut self, script: &TickScript) -> Sample {
        self.tick_count += 1;
        let elapsed_s = if script.elapsed_s.is_finite() && script.elapsed_s > 0.0 {
            script.elapsed_s
        } else {
            TICK_S
        };
        self.t_mono += elapsed_s;
        let now = self.base_instant + Duration::from_secs_f64(self.t_mono);

        if script.socket_dead {
            self.emulator.kill_socket();
        } else {
            self.emulator.revive_socket();
        }

        // Demand model: the caps are what the controller asked for, the
        // draw is what actually happened -- only the draw heats anything.
        let cpu_pkg_w = script.cpu_cap_w * script.cpu_demand_frac.clamp(0.0, 1.0);
        let gpu_present = (script.gpu_temp_c.is_some() || self.thermal.gpu_ec_c.is_some())
            && script.gpu_group_present;
        let clock_model = script.gpu_lock_mhz.is_some() && script.gpu_load_level.is_some();
        let per_device_fault = !script.cpu_group_present
            || !script.gpu_group_present
            || script.cpu_stuck_c.is_some()
            || script.gpu_stuck_c.is_some()
            || script.ambient_c.is_some()
            || script.charger_c.is_some()
            || script.raw_only_c.is_some()
            || script.unknown_c.is_some()
            || script.ec_invalid;
        let (gpu_w, gpu_sm_mhz) = if gpu_present {
            if let (Some(lock_mhz), Some(load_level)) = (script.gpu_lock_mhz, script.gpu_load_level)
            {
                let load_level = load_level.clamp(0.0, 1.0);
                (
                    load_level * gpu_full_load_power_w(lock_mhz),
                    lock_mhz.min(gpu_clock_at_power_limit(load_level)),
                )
            } else {
                (
                    script.gpu_cap_w * script.gpu_demand_frac.clamp(0.0, 1.0),
                    script.gpu_sm_mhz,
                )
            }
        } else {
            (0.0, 0.0)
        };
        let faults = ThermalFaults {
            cpu_group_present: script.cpu_group_present,
            gpu_group_present: gpu_present,
            cpu_stuck_c: script.cpu_stuck_c,
            gpu_stuck_c: script.gpu_stuck_c,
            ambient_c: script.ambient_c,
            charger_c: script.charger_c,
            raw_only_c: script.raw_only_c,
            unknown_c: script.unknown_c,
            ec_invalid: script.ec_invalid,
        };
        // Existing cap/fraction scripts retain the carried scalar dynamics.
        // Clock/load and new fault scripts enter the elapsed-time two-node
        // model, so the two physical contracts cannot perturb each other.
        let ec_reading = if clock_model || per_device_fault {
            self.thermal.tick_nodes(cpu_pkg_w, gpu_w, elapsed_s, faults)
        } else {
            self.thermal.tick_legacy_scalar(
                cpu_pkg_w + gpu_w,
                ThermalFaults {
                    gpu_group_present: self.thermal.gpu_ec_c.is_some(),
                    ..faults
                },
            )
        };
        let current_c = ec_reading
            .as_ref()
            .map_or(0.0, |reading| f64::from(reading.max_c));
        let sensor = if script.sensor_read_failed {
            SensorRead::Failed
        } else {
            SensorRead::Ok(current_c)
        };
        self.emulator.tick(sensor);

        let rpm = if script.fan_outage {
            0.0
        } else if self.emulator.wants_ec_autofan() {
            self.fan.tick_ec_autofan(current_c)
        } else {
            self.fan.tick(self.emulator.speed_pct())
        };

        let mut fanctrl_view_changed = false;
        if self.tick_count.is_multiple_of(ALL_POLL_EVERY_TICKS) {
            if let Some(v) = self.emulator.view(now) {
                self.last_view = Some(v);
            }
            // A failed poll (dead socket) leaves `last_view` untouched --
            // see the field's own doc comment.
            let new_stamp = self.last_view.as_ref().and_then(|v| v.all_observed_at);
            if new_stamp.is_some() && new_stamp != self.last_all_observed_at {
                fanctrl_view_changed = true;
            }
            self.last_all_observed_at = new_stamp;
        } else if self.tick_count.is_multiple_of(SPEED_POLL_EVERY_TICKS)
            && !self.emulator.is_socket_dead()
        {
            if let Some(view) = &mut self.last_view {
                view.speed_pct = self.emulator.speed_pct();
                view.observed_at = now;
            }
            // No view yet to refresh: a Speed-only success before the
            // first All has nothing to attach to -- same rule as
            // `UnixFanctrlClient::poll`.
        }

        let fanctrl_freshness = if script.force_stale_view {
            Freshness::Stale
        } else {
            compute_freshness(self.emulator.is_socket_dead(), self.last_view.as_ref(), now)
        };

        Sample {
            t_mono: self.t_mono,
            fan1_rpm: rpm,
            fan2_rpm: rpm,
            cpu_temp_c: script.cpu_tctl_c.unwrap_or(self.thermal.controllable_c()),
            cpu_pkg_w,
            igpu_w: 0.0,
            gpu_w,
            gpu_temp_c: script.gpu_temp_c.unwrap_or(0.0),
            gpu_sm_mhz: if gpu_present && script.gpu_draw_available {
                gpu_sm_mhz
            } else {
                0.0
            },
            gpu_util_pct: script.gpu_util_pct,
            cpu_util_pct: script.cpu_util_pct,
            cpu_avg_mhz: 0.0,
            resumed: script.resumed,
            fan_valid: !script.fan_outage,
            cpu_temp_valid: true,
            gpu_w_valid: gpu_present && script.gpu_draw_available,
            gpu_temp_valid: gpu_present,
            gpu_mhz_valid: gpu_present && script.gpu_draw_available,
            ec_valid: ec_reading.is_some(),
            ec: ec_reading,
            nvme_temp_c: script.nvme_temp_c,
            fanctrl: self.last_view.clone(),
            fanctrl_freshness,
            fanctrl_view_changed: fanctrl_view_changed || script.force_view_changed,
            on_ac: script.on_ac,
        }
    }
}

#[cfg(test)]
mod chained_plant_tests {
    use super::*;

    fn quiet16_plant(ambient_base_c: f64, fan_seed: u32) -> ChainedPlant {
        ChainedPlant::new(
            "quiet16",
            vec![
                (0.0, 15),
                (55.0, 15),
                (65.0, 21),
                (75.0, 31),
                (82.0, 37),
                (88.0, 55),
                (95.0, 100),
            ],
            60,
            ambient_base_c,
            fan_seed,
        )
        .unwrap()
    }

    // --- Step 5: fanctrl_view_changed set exactly once per new print-all --

    #[test]
    fn fanctrl_view_changed_fires_exactly_once_per_new_print_all_view() {
        let mut p = quiet16_plant(40.0, 1);
        let script = TickScript::default();
        let mut changed_ticks = Vec::new();
        for t in 1..=95u64 {
            let sample = p.tick(&script);
            if sample.fanctrl_view_changed {
                changed_ticks.push(t);
            }
        }
        // 95 ticks: print-all lands on ticks 30, 60, 90 -- exactly 3 new
        // views, so exactly 3 changed samples, at exactly those ticks.
        assert_eq!(changed_ticks, vec![30, 60, 90]);
        assert!((p.t_mono() - 95.0 * TICK_S).abs() < 1e-9);
    }

    #[test]
    fn a_later_speed_only_poll_refreshes_speed_pct_without_setting_view_changed() {
        let mut p = quiet16_plant(40.0, 1);
        let script = TickScript::default();
        for _ in 1..30 {
            p.tick(&script);
        }
        let all_sample = p.tick(&script); // tick 30: the first All poll
        assert!(all_sample.fanctrl.is_some());
        let stamp_after_all = all_sample.fanctrl.as_ref().unwrap().all_observed_at;

        for _ in 31..35 {
            p.tick(&script); // ticks 31-34: neither cadence
        }
        let speed_sample = p.tick(&script); // tick 35: Speed-only cadence
        let view = speed_sample
            .fanctrl
            .expect("view must still be carried forward");
        assert!(
            !speed_sample.fanctrl_view_changed,
            "a Speed-only refresh must not set fanctrl_view_changed"
        );
        assert_eq!(
            view.all_observed_at, stamp_after_all,
            "a Speed-only refresh must not bump all_observed_at"
        );
        assert_eq!(
            view.speed_pct,
            p.emulator_mut().speed_pct(),
            "a Speed-only refresh must still update speed_pct"
        );
    }

    #[test]
    fn fanctrl_is_none_until_the_first_print_all_lands() {
        let mut p = quiet16_plant(40.0, 1);
        let script = TickScript::default();
        for t in 1..30 {
            let sample = p.tick(&script);
            assert!(
                sample.fanctrl.is_none(),
                "no All poll has landed yet at tick {t}"
            );
            assert!(!sample.fanctrl_view_changed);
        }
        let sample = p.tick(&script); // tick 30
        assert!(sample.fanctrl.is_some());
        assert!(sample.fanctrl_view_changed);
    }

    // --- Step 6: open-loop watts -> RPM lag, 26-30s -------------------------

    /// Shared setup for the lag test: a synthetic strategy with a flat
    /// lead-in and a single tread boundary a few degrees above the
    /// baseline temperature, `ma_interval=1` so the boxcar (already
    /// covered on its own in `emulator_tests`) is not itself the thing
    /// under test here -- only the FOPDT dead time/lag and the fan's own
    /// response are.
    fn lag_test_plant() -> ChainedPlant {
        ChainedPlant::new(
            "lagtest",
            vec![(0.0, 20), (42.0, 20), (85.0, 90)],
            1,
            40.0,
            9001,
        )
        .unwrap()
    }

    #[test]
    fn an_open_loop_watts_step_shows_a_26_to_30s_watts_to_rpm_lag() {
        let mut p = lag_test_plant();
        let mut script = TickScript::default();

        // Warm-start at zero watts so the plant is at rest before the step
        // (the boxcar's own transient is not what this test measures).
        for _ in 0..10 {
            p.tick(&script);
        }
        let duty = p.emulator_mut().speed_pct();
        let base_rpm = p.fan_mut().base_rpm_for_duty(duty);

        script.cpu_cap_w = 20.0; // drawn watts (demand_frac defaults to 1.0)
        let mut lag_ticks = None;
        for t in 1..=120u64 {
            let sample = p.tick(&script);
            if lag_ticks.is_none() && (sample.max_fan_rpm() - base_rpm).abs() > FAN_NOISE_RPM {
                lag_ticks = Some(t);
            }
        }
        let lag = lag_ticks.expect("RPM must move detectably within 120s of the step");
        assert!(
            (26..=30).contains(&lag),
            "watts->RPM lag should land in [26,30]s, got {lag}s (theta=20s dead time + the FOPDT rise to the curve's tread boundary)"
        );
    }

    // --- Step 8: socket death / active:false -> EC-autofan, at the full chain -

    #[test]
    fn a_scripted_socket_death_yields_absent_and_ec_autofan_rpm() {
        let mut p = quiet16_plant(70.0, 5);
        let script = TickScript {
            socket_dead: true,
            ..Default::default()
        };
        let sample = (0..6).map(|_| p.tick(&script)).last().unwrap();
        assert_eq!(sample.fanctrl_freshness, Freshness::Absent);
        assert!(
            sample.fanctrl.is_none(),
            "a socket that has never once succeeded has no view"
        );
        let ec_max_c = f64::from(sample.ec.as_ref().unwrap().max_c);
        let expected = ec_autofan_rpm_base(ec_max_c);
        assert!(
            (sample.max_fan_rpm() - expected).abs() <= FAN_NOISE_RPM + MOMENTUM_KICK_RPM,
            "dead-socket RPM should track the EC staircase at {ec_max_c}C ({expected}), got {}",
            sample.max_fan_rpm()
        );
        // Not driven by any commanded duty: the emulator's own duty logic
        // never ran (frozen by the dead socket), so it is still whatever
        // it started at (0), which the table would map far below the
        // staircase's plateau -- confirming this is genuinely autofan, not
        // a coincidental match.
        assert_eq!(p.emulator_mut().speed_pct(), 0);
    }

    #[test]
    fn active_false_drives_rpm_from_the_ec_staircase_instead_of_commanded_duty() {
        let mut p = quiet16_plant(71.0, 6);
        let mut script = TickScript::default();
        // Warm up alive so the emulator commands a real (non-floor) duty.
        for _ in 0..3 {
            p.tick(&script);
        }
        let commanded_duty = p.emulator_mut().speed_pct();
        let commanded_base_rpm = p.fan_mut().base_rpm_for_duty(commanded_duty);

        p.emulator_mut().set_active(false);
        script.socket_dead = false; // socket is alive, just paused
        let sample = p.tick(&script);
        assert!(!p.emulator_mut().is_active());

        let ec_max_c = f64::from(sample.ec.as_ref().unwrap().max_c);
        let expected = ec_autofan_rpm_base(ec_max_c);
        assert!(
            (sample.max_fan_rpm() - expected).abs() <= FAN_NOISE_RPM + MOMENTUM_KICK_RPM,
            "active:false RPM should track the EC staircase, got {} expected ~{expected}",
            sample.max_fan_rpm()
        );
        assert!(
            (sample.max_fan_rpm() - commanded_base_rpm).abs() > FAN_NOISE_RPM,
            "the EC staircase RPM at ~71C must differ meaningfully from the pre-pause commanded duty's RPM \
             (staircase {expected} vs commanded-duty base {commanded_base_rpm}) -- otherwise this test cannot \
             tell autofan from coincidence"
        );
    }

    // --- Step 9: demand model ------------------------------------------------

    #[test]
    fn cpu_heavy_vs_gpu_heavy_utilisation_shifts_the_demand_split() {
        let mut cpu_heavy = quiet16_plant(40.0, 1);
        let mut script = TickScript {
            cpu_cap_w: 30.0,
            gpu_cap_w: 30.0,
            gpu_temp_c: Some(60.0),
            cpu_demand_frac: 0.9,
            gpu_demand_frac: 0.1,
            ..Default::default()
        };
        let cpu_heavy_sample = cpu_heavy.tick(&script);
        assert!(cpu_heavy_sample.cpu_pkg_w > cpu_heavy_sample.gpu_w);

        let mut gpu_heavy = quiet16_plant(40.0, 1);
        script.cpu_demand_frac = 0.1;
        script.gpu_demand_frac = 0.9;
        let gpu_heavy_sample = gpu_heavy.tick(&script);
        assert!(gpu_heavy_sample.gpu_w > gpu_heavy_sample.cpu_pkg_w);
    }

    #[test]
    fn a_low_demand_script_emits_a_measured_draw_well_below_the_commanded_cap() {
        let mut p = quiet16_plant(40.0, 1);
        let script = TickScript {
            cpu_cap_w: 40.0,
            gpu_cap_w: 40.0,
            gpu_temp_c: Some(50.0),
            cpu_demand_frac: 0.1, // a lull: only 10% of the cap is drawn
            gpu_demand_frac: 0.1,
            ..Default::default()
        };
        let sample = p.tick(&script);
        assert!(
            (sample.cpu_pkg_w - 4.0).abs() < 1e-9,
            "got {}",
            sample.cpu_pkg_w
        );
        assert!((sample.gpu_w - 4.0).abs() < 1e-9, "got {}", sample.gpu_w);
        assert!(
            sample.cpu_pkg_w < script.cpu_cap_w / 2.0,
            "draw must sit well below the cap"
        );
        assert!(sample.gpu_w < script.gpu_cap_w / 2.0);
    }

    #[test]
    fn an_unpowered_dgpu_gates_gpu_watts_and_validity_to_zero_regardless_of_the_cap() {
        let mut p = quiet16_plant(40.0, 1);
        let script = TickScript {
            gpu_cap_w: 50.0,
            gpu_demand_frac: 1.0,
            gpu_temp_c: None, // unpowered/unsensed
            ..Default::default()
        };
        let sample = p.tick(&script);
        assert_eq!(sample.gpu_w, 0.0);
        assert!(!sample.gpu_w_valid);
        assert!(!sample.gpu_temp_valid);
        assert!(!sample.gpu_mhz_valid);
    }

    // --- Step 10: scripted resumed edge --------------------------------------

    #[test]
    fn a_scripted_resumed_edge_appears_on_the_emitted_sample() {
        let mut p = quiet16_plant(40.0, 1);
        let mut script = TickScript::default();
        let normal = p.tick(&script);
        assert!(!normal.resumed);
        script.resumed = true;
        let resumed = p.tick(&script);
        assert!(resumed.resumed);
        script.resumed = false;
        let after = p.tick(&script);
        assert!(
            !after.resumed,
            "resumed must not stick past the scripted tick"
        );
    }

    // --- The powered-dGPU EC case (design doc §5: "so the powered-dGPU
    // case can be run") ---------------------------------------------------

    #[test]
    fn a_scripted_powered_dgpu_ec_channel_shows_up_in_the_emitted_ec_reading() {
        let mut p = quiet16_plant(40.0, 1);
        p.thermal_mut().set_ambient_charger(45.0, 30.0);
        p.thermal_mut().set_gpu_ec(Some(96.0));
        let sample = p.tick(&TickScript::default());
        let ec = sample.ec.expect("ChainedPlant always emits an EcReading");
        assert_eq!(
            ec.max_c, 96,
            "the scripted gpu_* channel must win the max, exactly like §Facts's synthetic case"
        );
        assert_eq!(ec.argmax.as_str(), "gpu_amb_f75303@4d");
    }
}

/// Revision-4 behavioural contract for the per-device plant.  These tests
/// deliberately use the public scripts rather than reaching into the plant:
/// later controller acceptance legs need the same observable seams.
#[cfg(test)]
mod per_device_plant_tests {
    use super::*;

    fn plant() -> ChainedPlant {
        ChainedPlant::new(
            "per-device",
            vec![(0.0, 20), (60.0, 40), (95.0, 100)],
            1,
            42.0,
            77,
        )
        .unwrap()
    }

    #[test]
    fn gpu_full_load_plateaus_at_100w_and_reports_the_power_limited_clock() {
        let mut p = plant();
        let mut script = TickScript {
            gpu_lock_mhz: Some(3090.0),
            gpu_load_level: Some(1.0),
            gpu_temp_c: Some(42.0),
            ..Default::default()
        };

        let plateau = p.tick(&script);
        assert!(
            (plateau.gpu_w - 100.0).abs() < 0.01,
            "got {} W",
            plateau.gpu_w
        );
        assert_eq!(plateau.gpu_sm_mhz, 2143.0);

        script.gpu_lock_mhz = Some(1995.0);
        let below_knee = p.tick(&script);
        assert!(
            (below_knee.gpu_w - 90.8).abs() < 0.01,
            "got {} W",
            below_knee.gpu_w
        );
        assert_eq!(below_knee.gpu_sm_mhz, 1995.0);
    }

    #[test]
    fn september_power_curve_preserves_each_knot_and_cpu_bursts_are_not_sustained_draw() {
        for (clock, expected_w) in GPU_FULL_LOAD_POINTS {
            assert!(
                (gpu_full_load_power_w(clock) - expected_w).abs() < f64::EPSILON,
                "{clock} MHz"
            );
        }
        let mut p = plant();
        p.set_cpu_plant_params(CpuPlantParams::default());
        let sustained = TickScript {
            cpu_cap_w: 40.0,
            ..Default::default()
        };
        assert_eq!(p.tick(&sustained).cpu_pkg_w, 40.0);
        let burst = TickScript {
            cpu_cap_w: 70.0,
            ..Default::default()
        };
        assert_eq!(p.tick(&burst).cpu_pkg_w, 70.0);
        assert_eq!(p.tick(&sustained).cpu_pkg_w, 40.0);
    }

    #[test]
    fn separate_gpu_node_uses_the_point_four_heat_path_and_cross_couples_to_cpu() {
        let mut p = plant();
        p.set_gpu_plant_params(GpuPlantParams::nominal());
        let script = TickScript {
            gpu_lock_mhz: Some(3090.0),
            gpu_load_level: Some(1.0),
            gpu_temp_c: Some(42.0),
            ..Default::default()
        };
        let mut sample = p.tick(&script);
        for _ in 0..900 {
            sample = p.tick(&script);
        }
        let ec = sample.ec.expect("plant has a valid EC reading");
        assert!((ec.gpu_group_c.expect("GPU group") - 82.0).abs() < 0.6);
        assert!((ec.cpu_group_c.expect("CPU group") - 46.0).abs() < 0.6);
    }

    #[test]
    fn gpu_parameter_grid_and_faults_are_observable_at_the_sample_boundary() {
        let mut p = plant();
        for tau_s in [8.0, 15.0, 25.0, 50.0] {
            for k_c_per_mhz in [0.01, 0.02, 0.03] {
                for theta_eff_s in [45.0, 90.0, 135.0] {
                    let params = GpuPlantParams {
                        tau_s,
                        k_c_per_mhz,
                        theta_eff_s,
                    };
                    p.set_gpu_plant_params(params);
                    assert_eq!(p.gpu_plant_params(), params);
                }
            }
        }

        let sample = p.tick(&TickScript {
            gpu_temp_c: Some(105.0),
            cpu_tctl_c: Some(105.0),
            gpu_stuck_c: Some(105.0),
            ambient_c: Some(105.0),
            charger_c: Some(105.0),
            raw_only_c: Some(150.0),
            unknown_c: Some(76.0),
            ..Default::default()
        });
        let ec = sample.ec.expect("105C is plausible control data");
        assert_eq!(ec.gpu_group_c, Some(105.0));
        assert_eq!(ec.reconciliation_max_c, Some(150));
        assert!(
            ec.all
                .iter()
                .any(|(label, _)| label.as_str() == "unknown_thermal")
        );
        assert_eq!(sample.gpu_temp_c, 105.0);
        assert_eq!(sample.cpu_temp_c, 105.0);

        let lost = p.tick(&TickScript {
            gpu_temp_c: Some(42.0),
            ambient_c: Some(-150.0),
            charger_c: Some(-150.0),
            ..Default::default()
        });
        let ec = lost
            .ec
            .expect("the raw sentinel pair does not make EC invalid by itself");
        assert!(ec.cpu_group_c.is_some());
        assert!(ec.gpu_group_c.is_some());
        assert!(lost.ec_valid);

        let group_lost = p.tick(&TickScript {
            cpu_group_present: false,
            gpu_group_present: false,
            gpu_temp_c: Some(42.0),
            ..Default::default()
        });
        let ec = group_lost.ec.expect("ambient/charger keep whole EC valid");
        assert_eq!(ec.cpu_group_c, None);
        assert_eq!(ec.gpu_group_c, None);
    }

    #[test]
    fn scripted_outage_ec_loss_and_view_faults_are_visible_without_hardware() {
        let mut p = plant();
        let sample = p.tick(&TickScript {
            fan_outage: true,
            gpu_draw_available: false,
            force_stale_view: true,
            force_view_changed: true,
            ..Default::default()
        });
        assert!(!sample.fan_valid);
        assert!(!sample.gpu_w_valid);
        assert_eq!(sample.fanctrl_freshness, Freshness::Stale);
        assert!(sample.fanctrl_view_changed);

        let invalid = p.tick(&TickScript {
            ec_invalid: true,
            resumed: true,
            elapsed_s: 7200.0,
            ..Default::default()
        });
        assert!(!invalid.ec_valid);
        assert!(invalid.ec.is_none());
        assert!(invalid.resumed);
        assert!((invalid.t_mono - 7201.0).abs() < f64::EPSILON);
    }
}

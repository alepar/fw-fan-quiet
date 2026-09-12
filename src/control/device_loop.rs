//! Pure per-device temperature-loop control (§2.3, revision 4).

use serde::{Deserialize, Serialize};

/// CPU sustained-power units (watts).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct W;

/// GPU maximum-clock units (MHz).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mhz;

mod sealed {
    pub trait Sealed {}
}

/// Unit-specific plant constants and actuator resolution.
pub trait DeviceUnit: sealed::Sealed + Copy {
    const TAU_S: f64;
    const PLANT_GAIN: f64;
    const BASE_DELAY_S: f64;
    const GRID: f64;
    const RISE_RATE: f64;
    const FALL_RATE: f64;
}

impl sealed::Sealed for W {}
impl DeviceUnit for W {
    const TAU_S: f64 = 35.0;
    const PLANT_GAIN: f64 = 0.8;
    const BASE_DELAY_S: f64 = 20.0;
    const GRID: f64 = 0.5;
    const RISE_RATE: f64 = 10.0;
    const FALL_RATE: f64 = f64::INFINITY;
}

impl sealed::Sealed for Mhz {}
impl DeviceUnit for Mhz {
    const TAU_S: f64 = 15.0;
    const PLANT_GAIN: f64 = 0.02;
    // EC lag (20 s) plus the measured gpu_vr tail (40 s).
    const BASE_DELAY_S: f64 = 60.0;
    const GRID: f64 = 1.0;
    const RISE_RATE: f64 = 105.0;
    const FALL_RATE: f64 = 105.0;
}

/// Velocity-form PI gains in actuator units per degree Celsius.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Gains {
    pub kc: f64,
    pub ti_s: f64,
}

impl Gains {
    pub fn is_valid(&self) -> bool {
        self.kc.is_finite() && self.kc > 0.0 && self.ti_s.is_finite() && self.ti_s > 0.0
    }
}

/// Live-interval IMC defaults from design §2.3.
pub fn default_gains<U: DeviceUnit>(ma_interval: u32) -> Gains {
    let theta_eff = U::BASE_DELAY_S + f64::from(ma_interval) / 2.0;
    let lambda = 90.0_f64.max(3.0 * theta_eff);
    Gains {
        kc: U::TAU_S / (U::PLANT_GAIN * (lambda + theta_eff)),
        ti_s: U::TAU_S,
    }
}

pub const PI_PERIOD_S: f64 = 5.0;
pub const GROUP_UNAVAILABLE_DWELL_S: f64 = 60.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThermalMode {
    Regulate,
    Bypass,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActuatorState {
    Verified,
    Mismatch,
    Unverifiable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Bound {
    Floor,
    Max,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Hold {
    None,
    Shadow,
    Clamp(Bound),
    ActuatorMismatch,
    GroupUnavailable,
    DrawUnavailable,
    Bypass,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Selected {
    Thermal,
    Shadow,
    Floor,
    Max,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TickInput {
    pub t_star: f64,
    pub group_c: Option<f64>,
    pub draw: Option<f64>,
    pub floor: f64,
    pub max: f64,
    pub mode: ThermalMode,
    pub actuator: ActuatorState,
    pub dt_s: f64,
    pub resumed: bool,
    pub delta_tstar: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DeviceDecision {
    pub t_star: f64,
    pub group_c: Option<f64>,
    pub err_c: Option<f64>,
    pub thermal: f64,
    pub shadow: f64,
    pub cap: f64,
    pub selected: Selected,
    pub hold: Hold,
    /// False only for a group absent since this Auto session began.
    pub write_allowed: bool,
    /// A previously available group exceeded its 60 s unavailable dwell.
    pub group_lost: bool,
    /// A safety-bound move must bypass the controller's ordinary cadence.
    pub write_immediately: bool,
}

/// Selects between already-clamped, unquantised candidates.
pub fn select_candidates(thermal: f64, shadow: f64, floor: f64, max: f64) -> (f64, Selected) {
    let thermal = thermal.clamp(floor, max);
    let shadow = shadow.clamp(floor, max);
    let requested = thermal.min(shadow);
    let selected = if requested == floor {
        Selected::Floor
    } else if requested == max {
        Selected::Max
    } else if thermal <= shadow {
        Selected::Thermal
    } else {
        Selected::Shadow
    };
    (requested, selected)
}

/// Pure control state for one actuator.
pub struct DeviceLoop<U: DeviceUnit> {
    gains: Gains,
    thermal: f64,
    shadow: f64,
    requested: Option<f64>,
    last_applied: Option<f64>,
    e_prev: f64,
    elapsed_s: f64,
    floor: f64,
    max: f64,
    bounds_initialised: bool,
    group_seen: bool,
    group_missing: bool,
    group_missing_s: f64,
    draw_missing_s: f64,
    mismatch_latched: bool,
    pending_immediate: bool,
    unit: std::marker::PhantomData<U>,
}

impl<U: DeviceUnit> DeviceLoop<U> {
    pub fn new(gains: Gains) -> Self {
        debug_assert!(gains.is_valid());
        Self {
            gains,
            thermal: 0.0,
            shadow: 0.0,
            requested: None,
            last_applied: None,
            e_prev: 0.0,
            elapsed_s: 0.0,
            floor: f64::NEG_INFINITY,
            max: f64::INFINITY,
            bounds_initialised: false,
            group_seen: false,
            group_missing: false,
            group_missing_s: 0.0,
            draw_missing_s: 0.0,
            mismatch_latched: false,
            pending_immediate: false,
            unit: std::marker::PhantomData,
        }
    }

    /// Seeds the loop from a cap already in force and synchronises its error.
    pub fn seed(&mut self, cap: f64, error: f64) {
        self.seed_candidates(cap, cap, Some(cap), error);
    }

    /// Candidate-level entry seam used by the shadow/handover extension.
    pub fn seed_candidates(&mut self, thermal: f64, shadow: f64, applied: Option<f64>, error: f64) {
        self.thermal = thermal;
        self.shadow = shadow;
        self.requested = applied.or(Some(thermal.min(shadow)));
        self.last_applied = applied;
        self.e_prev = error;
        self.elapsed_s = 0.0;
        self.group_seen = true;
    }

    /// Bumplessly transfers thermal ownership to `cap`.
    pub fn transfer_thermal(&mut self, cap: f64, error: f64) {
        self.thermal = cap.clamp(self.floor, self.max);
        self.e_prev = error;
        self.elapsed_s = 0.0;
    }

    /// Seeds the second candidate without changing thermal PI state.
    pub fn transfer_shadow(&mut self, cap: f64) {
        self.shadow = cap.clamp(self.floor, self.max);
    }

    pub fn resync_error(&mut self, error: f64) {
        self.e_prev = error;
    }

    pub fn note_applied(&mut self, cap: f64) {
        self.last_applied = Some(cap.clamp(self.floor, self.max));
    }

    pub fn thermal(&self) -> f64 {
        self.thermal
    }

    pub fn requested(&self) -> Option<f64> {
        self.requested
    }

    /// Applies a guard ceiling. Lowering clamps every cap-bearing state;
    /// raising only opens the ceiling and leaves those states untouched.
    pub fn clamp_max(&mut self, max: f64) -> bool {
        if !max.is_finite() || max >= self.max {
            if max.is_finite() {
                self.max = max;
            }
            return false;
        }
        self.max = max.max(self.floor);
        let mut changed = false;
        for value in [&mut self.thermal, &mut self.shadow] {
            if *value > self.max {
                *value = self.max;
                changed = true;
            }
        }
        if let Some(value) = self.requested.as_mut()
            && *value > self.max
        {
            *value = self.max;
            changed = true;
        }
        if let Some(value) = self.last_applied.as_mut()
            && *value > self.max
        {
            *value = self.max;
            changed = true;
        }
        self.pending_immediate |= changed;
        changed
    }

    pub fn tick(&mut self, input: TickInput) -> DeviceDecision {
        let dt = control_dt(input.dt_s);
        let (floor, max) = ordered_bounds(input.floor, input.max);
        let floor_changed = self.bounds_initialised && floor != self.floor;
        let max_lowered = self.bounds_initialised && max < self.max;
        if !self.bounds_initialised {
            self.floor = floor;
            self.max = max;
            self.bounds_initialised = true;
            self.pending_immediate |= self.clamp_all_to_bounds();
        } else {
            self.floor = floor;
            if max_lowered {
                self.clamp_max(max);
            } else {
                self.max = max;
            }
            let bounds_clamped = self.clamp_all_to_bounds();
            self.pending_immediate |= floor_changed && bounds_clamped;
        }

        let err = input.group_c.map(|group| input.t_star - group);

        if input.resumed {
            self.elapsed_s = 0.0;
            self.group_missing_s = 0.0;
            self.draw_missing_s = 0.0;
            if let Some(error) = err {
                self.resync_error(error);
            }
            let cap = self
                .last_applied
                .or(self.requested)
                .unwrap_or(self.max)
                .clamp(self.floor, self.max);
            self.requested = Some(cap);
            let write_allowed = input.group_c.is_some() || self.group_seen;
            self.pending_immediate = write_allowed;
            return self.decision(input, cap, self.thermal, self.shadow, write_allowed, false);
        }

        let Some(error) = err else {
            self.elapsed_s = 0.0;
            self.draw_missing_s = 0.0;
            self.group_missing = true;
            if self.group_seen {
                self.group_missing_s += dt;
            }
            let group_lost = self.group_seen && self.group_missing_s >= GROUP_UNAVAILABLE_DWELL_S;
            let cap = if group_lost {
                self.max
            } else {
                self.last_applied.unwrap_or(self.max)
            };
            let cap = self.quantize(cap).clamp(self.floor, self.max);
            self.requested = Some(cap);
            return self.decision(
                input,
                cap,
                self.thermal,
                self.max,
                self.group_seen,
                group_lost,
            );
        };

        let recovered_group = self.group_missing;
        if !self.group_seen || recovered_group {
            self.group_seen = true;
            self.group_missing = false;
            self.group_missing_s = 0.0;
            if recovered_group {
                self.thermal = self.max;
                self.shadow = self.max;
                self.requested = Some(self.quantize(self.max));
            }
            self.resync_error(error);
            self.elapsed_s = 0.0;
        }

        match input.actuator {
            ActuatorState::Mismatch => self.mismatch_latched = true,
            ActuatorState::Verified if self.mismatch_latched => {
                self.mismatch_latched = false;
                self.resync_error(error);
                self.elapsed_s = 0.0;
            }
            ActuatorState::Verified | ActuatorState::Unverifiable => {}
        }

        if self.mismatch_latched {
            self.elapsed_s = 0.0;
            let cap = self
                .requested
                .unwrap_or(self.max)
                .clamp(self.floor, self.max);
            return self.decision(input, cap, self.thermal, self.shadow, true, false);
        }

        // eb9.4 replaces this explicit max stub with the draw-tracking
        // candidate. Keeping it as a real candidate makes its selector and
        // transfer seam independently testable now.
        self.shadow = self.max;
        if input.draw.is_none() {
            self.draw_missing_s += dt;
        } else {
            self.draw_missing_s = 0.0;
        }

        let thermal_candidate = match input.mode {
            ThermalMode::Bypass => self.max,
            ThermalMode::Regulate => {
                self.e_prev += finite_or_zero(input.delta_tstar);
                self.elapsed_s += dt;
                if self.elapsed_s >= PI_PERIOD_S {
                    let elapsed = self.elapsed_s.min(7.0);
                    let proportional = self.gains.kc * (error - self.e_prev);
                    let integral = self.gains.kc * elapsed / self.gains.ti_s * error;
                    let farther = (self.thermal <= self.floor && integral < 0.0)
                        || (self.thermal >= self.max && integral > 0.0);
                    self.thermal =
                        (self.thermal + proportional + if farther { 0.0 } else { integral })
                            .clamp(self.floor, self.max);
                    self.e_prev = error;
                    self.elapsed_s = 0.0;
                }
                self.thermal
            }
        };

        let (target, selected) =
            select_candidates(thermal_candidate, self.shadow, self.floor, self.max);
        let cap = self.slew_and_quantize(target, selected, dt, floor_changed || max_lowered);
        self.requested = Some(cap);
        self.decision(input, cap, thermal_candidate, self.shadow, true, false)
    }

    fn slew_and_quantize(
        &self,
        target: f64,
        selected: Selected,
        dt: f64,
        bypass_slew: bool,
    ) -> f64 {
        let desired = self.quantize(target).clamp(self.floor, self.max);
        let Some(previous) = self.requested else {
            return desired;
        };
        if bypass_slew {
            return desired;
        }
        let rise_rate = if selected == Selected::Shadow && U::GRID == 1.0 {
            300.0
        } else {
            U::RISE_RATE
        };
        let moved = if desired > previous {
            desired.min(previous + rise_rate * dt)
        } else {
            desired.max(previous - U::FALL_RATE * dt)
        };
        self.quantize(moved).clamp(self.floor, self.max)
    }

    fn quantize(&self, value: f64) -> f64 {
        (value / U::GRID).round() * U::GRID
    }

    fn clamp_all_to_bounds(&mut self) -> bool {
        let mut changed = false;
        let thermal = self.thermal.clamp(self.floor, self.max);
        changed |= thermal != self.thermal;
        self.thermal = thermal;
        let shadow = self.shadow.clamp(self.floor, self.max);
        changed |= shadow != self.shadow;
        self.shadow = shadow;
        if let Some(value) = self.requested.as_mut() {
            let clamped = value.clamp(self.floor, self.max);
            changed |= clamped != *value;
            *value = clamped;
        }
        if let Some(value) = self.last_applied.as_mut() {
            let clamped = value.clamp(self.floor, self.max);
            changed |= clamped != *value;
            *value = clamped;
        }
        changed
    }

    fn decision(
        &mut self,
        input: TickInput,
        cap: f64,
        thermal: f64,
        shadow: f64,
        write_allowed: bool,
        group_lost: bool,
    ) -> DeviceDecision {
        let selected = if input.group_c.is_none() {
            if cap == self.floor {
                Selected::Floor
            } else if cap == self.max {
                Selected::Max
            } else {
                Selected::Thermal
            }
        } else {
            select_candidates(thermal, shadow, self.floor, self.max).1
        };
        let hold = if input.group_c.is_none() {
            Hold::GroupUnavailable
        } else if self.mismatch_latched {
            Hold::ActuatorMismatch
        } else if input.mode == ThermalMode::Bypass {
            Hold::Bypass
        } else if input.draw.is_none() {
            Hold::DrawUnavailable
        } else {
            match selected {
                Selected::Floor => Hold::Clamp(Bound::Floor),
                Selected::Max => Hold::Clamp(Bound::Max),
                Selected::Shadow => Hold::Shadow,
                Selected::Thermal => Hold::None,
            }
        };
        let write_immediately = std::mem::take(&mut self.pending_immediate);
        DeviceDecision {
            t_star: input.t_star,
            group_c: input.group_c,
            err_c: input.group_c.map(|group| input.t_star - group),
            thermal,
            shadow,
            cap,
            selected,
            hold,
            write_allowed,
            group_lost,
            write_immediately,
        }
    }
}

fn control_dt(dt_s: f64) -> f64 {
    if dt_s.is_finite() && dt_s >= 0.0 {
        dt_s.min(2.0)
    } else {
        0.0
    }
}

fn finite_or_zero(value: f64) -> f64 {
    if value.is_finite() { value } else { 0.0 }
}

fn ordered_bounds(floor: f64, max: f64) -> (f64, f64) {
    let max = if max.is_finite() { max } else { 0.0 };
    let floor = if floor.is_finite() {
        floor.min(max)
    } else {
        max
    };
    (floor, max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn live_interval_changes_cpu_and_gpu_imc_defaults() {
        let cpu_60 = default_gains::<W>(60);
        close(cpu_60.ti_s, 35.0);
        close(cpu_60.kc, 0.21875);

        let gpu_60 = default_gains::<Mhz>(60);
        close(gpu_60.ti_s, 15.0);
        close(gpu_60.kc, 2.083_333_333_333_333_5);

        assert_ne!(default_gains::<W>(20).kc, cpu_60.kc);
        assert_ne!(default_gains::<Mhz>(20).kc, gpu_60.kc);
    }

    fn input(group_c: Option<f64>, dt_s: f64) -> TickInput {
        TickInput {
            t_star: 70.0,
            group_c,
            draw: Some(40.0),
            floor: 10.0,
            max: 100.0,
            mode: ThermalMode::Regulate,
            actuator: ActuatorState::Verified,
            dt_s,
            resumed: false,
            delta_tstar: 0.0,
        }
    }

    #[test]
    fn elapsed_cadence_runs_once_at_five_seconds_and_bounds_backlog() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(50.0, 0.0);

        for dt in [1.0, 0.4, 1.6] {
            close(loop_.tick(input(Some(60.0), dt)).thermal, 50.0);
        }
        close(loop_.tick(input(Some(60.0), 2.0)).thermal, 65.0);

        let mut bounded = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        bounded.seed(50.0, 10.0);
        for dt in [-1.0, f64::NAN, f64::INFINITY] {
            close(bounded.tick(input(Some(60.0), dt)).thermal, 50.0);
        }
        close(bounded.tick(input(Some(60.0), 100.0)).thermal, 50.0);
        close(bounded.tick(input(Some(60.0), 100.0)).thermal, 50.0);
        // A third backlogged sample reaches the cadence, but at most 2 s was
        // credited to each sample: 50 + 1*6/10*10.
        close(bounded.tick(input(Some(60.0), 100.0)).thermal, 56.0);
    }

    #[test]
    fn clamp_suppression_updates_previous_error_and_allows_unwind() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut low = DeviceLoop::<W>::new(gains);
        low.seed(10.0, 0.0);
        for dt in [2.0, 2.0, 1.0] {
            low.tick(input(Some(80.0), dt));
        }
        close(low.thermal(), 10.0);
        for dt in [2.0, 2.0, 1.0] {
            low.tick(input(Some(68.0), dt));
        }
        // e_prev was updated to -10 at the floor: reversing to +2 retains
        // the +12 proportional unwind instead of treating the old error as 0.
        close(low.thermal(), 23.0);

        let mut high = DeviceLoop::<W>::new(gains);
        high.seed(100.0, 0.0);
        for dt in [2.0, 2.0, 1.0] {
            high.tick(input(Some(60.0), dt));
        }
        close(high.thermal(), 100.0);
        for dt in [2.0, 2.0, 1.0] {
            high.tick(input(Some(72.0), dt));
        }
        close(high.thermal(), 87.0);
    }

    #[test]
    fn coincident_target_and_measurement_changes_keep_measurement_p_term() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(50.0, 0.0);
        loop_.tick(input(Some(70.0), 2.0));
        loop_.tick(input(Some(70.0), 2.0));

        let mut changed = input(Some(73.0), 1.0);
        changed.t_star = 72.0;
        changed.delta_tstar = 2.0;
        // e_prev shifts from 0 to +2. New err is -1, so P is -3 (the
        // measured +3 C), followed by I=-0.5. Cancelling all P would yield
        // 49.5 instead.
        close(loop_.tick(changed).thermal, 46.5);
    }

    #[test]
    fn resumed_sample_holds_applied_cap_and_restarts_elapsed_time() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(50.0, 20.0);
        loop_.note_applied(50.0);
        loop_.tick(input(Some(50.0), 2.0));
        loop_.tick(input(Some(50.0), 2.0));

        let mut resumed = input(Some(50.0), 7_200.0);
        resumed.resumed = true;
        let held = loop_.tick(resumed);
        close(held.cap, 50.0);
        assert!(held.write_immediately);
        close(loop_.thermal(), 50.0);

        // Resume cleared the pre-suspend four seconds. Two post-resume
        // samples are still below cadence; the third performs one update.
        close(loop_.tick(input(Some(50.0), 2.0)).thermal, 50.0);
        close(loop_.tick(input(Some(50.0), 2.0)).thermal, 50.0);
        close(loop_.tick(input(Some(50.0), 1.0)).thermal, 60.0);
    }

    #[test]
    fn resume_clears_missing_group_dwell_and_still_skips_an_absent_device() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut lost = DeviceLoop::<W>::new(gains);
        lost.seed(40.0, 0.0);
        for _ in 0..29 {
            assert!(!lost.tick(input(None, 2.0)).group_lost);
        }
        let mut resumed = input(None, 7_200.0);
        resumed.resumed = true;
        let decision = lost.tick(resumed);
        close(decision.cap, 40.0);
        assert!(!decision.group_lost);
        assert!(decision.write_allowed);
        assert!(!lost.tick(input(None, 2.0)).group_lost);

        let mut absent = DeviceLoop::<W>::new(gains);
        let decision = absent.tick(resumed);
        close(decision.cap, 100.0);
        assert!(!decision.write_allowed);
        assert!(!decision.group_lost);
    }

    #[test]
    fn selector_preserves_bound_identity_and_prefers_thermal_on_interior_ties() {
        assert_eq!(
            select_candidates(10.0, 10.0, 10.0, 100.0),
            (10.0, Selected::Floor)
        );
        assert_eq!(
            select_candidates(100.0, 100.0, 10.0, 100.0),
            (100.0, Selected::Max)
        );
        assert_eq!(
            select_candidates(50.0, 50.0, 10.0, 100.0),
            (50.0, Selected::Thermal)
        );
        assert_eq!(
            select_candidates(60.0, 50.0, 10.0, 100.0),
            (50.0, Selected::Shadow)
        );
    }

    #[test]
    fn cpu_quantisation_precedes_the_final_bounds_clamp() {
        let mut rounded = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        rounded.seed(10.26, 0.0);
        close(rounded.tick(input(Some(70.0), 0.0)).cap, 10.5);

        let mut bounded = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        bounded.seed(10.26, 0.0);
        let mut capped = input(Some(70.0), 0.0);
        capped.max = 10.3;
        close(bounded.tick(capped).cap, 10.3);
    }

    #[test]
    fn requested_slew_accumulates_each_sample_without_an_applied_note() {
        let mut loop_ = DeviceLoop::<Mhz>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(1_500.0, 0.0);
        loop_.transfer_thermal(2_000.0, 0.0);

        let mut tick = input(Some(70.0), 1.0);
        tick.floor = 1_000.0;
        tick.max = 3_090.0;
        close(loop_.tick(tick).cap, 1_605.0);
        close(loop_.tick(tick).cap, 1_710.0);
        close(loop_.tick(tick).cap, 1_815.0);
        close(loop_.requested().unwrap(), 1_815.0);
    }

    #[test]
    fn absent_group_skips_writes_but_lost_group_releases_to_max_and_recovers() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut absent = DeviceLoop::<W>::new(gains);
        let decision = absent.tick(input(None, 2.0));
        close(decision.cap, 100.0);
        assert_eq!(decision.selected, Selected::Max);
        assert_eq!(decision.hold, Hold::GroupUnavailable);
        assert!(!decision.write_allowed);
        assert!(!decision.group_lost);
        for _ in 0..40 {
            assert!(!absent.tick(input(None, 2.0)).group_lost);
        }

        let mut lost = DeviceLoop::<W>::new(gains);
        lost.seed(40.0, 0.0);
        lost.note_applied(40.0);
        for _ in 0..29 {
            let decision = lost.tick(input(None, 2.0));
            close(decision.cap, 40.0);
            assert!(!decision.group_lost);
        }
        let decision = lost.tick(input(None, 2.0));
        close(decision.cap, 100.0);
        assert_eq!(decision.selected, Selected::Max);
        assert!(decision.write_allowed);
        assert!(decision.group_lost);

        let recovered = lost.tick(input(Some(60.0), 1.0));
        close(recovered.thermal, 100.0);
        close(recovered.cap, 100.0);
        assert!(!recovered.group_lost);
    }

    #[test]
    fn missing_draw_reports_hold_without_freezing_thermal() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(50.0, 0.0);
        let mut tick = input(Some(60.0), 2.0);
        tick.draw = None;
        assert_eq!(loop_.tick(tick).hold, Hold::DrawUnavailable);
        assert_eq!(loop_.tick(tick).hold, Hold::DrawUnavailable);
        tick.dt_s = 1.0;
        let decision = loop_.tick(tick);
        assert_eq!(decision.hold, Hold::DrawUnavailable);
        close(decision.thermal, 65.0);
    }

    #[test]
    fn mismatch_freezes_until_verified_while_unverifiable_is_normally_live() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut loop_ = DeviceLoop::<W>::new(gains);
        loop_.seed(50.0, 0.0);
        let mut tick = input(Some(60.0), 2.0);
        tick.actuator = ActuatorState::Mismatch;
        for _ in 0..3 {
            let decision = loop_.tick(tick);
            assert_eq!(decision.hold, Hold::ActuatorMismatch);
            close(decision.thermal, 50.0);
        }
        tick.actuator = ActuatorState::Unverifiable;
        assert_eq!(loop_.tick(tick).hold, Hold::ActuatorMismatch);
        close(loop_.thermal(), 50.0);

        tick.actuator = ActuatorState::Verified;
        loop_.tick(tick);
        loop_.tick(tick);
        tick.dt_s = 1.0;
        close(loop_.tick(tick).thermal, 55.0);

        let mut live = DeviceLoop::<W>::new(gains);
        live.seed(50.0, 10.0);
        tick.actuator = ActuatorState::Unverifiable;
        tick.dt_s = 2.0;
        live.tick(tick);
        live.tick(tick);
        tick.dt_s = 1.0;
        close(live.tick(tick).thermal, 55.0);
    }

    #[test]
    fn lowered_guard_max_clamps_bypass_and_mismatch_state_immediately() {
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
            loop_.seed(80.0, 0.0);
            loop_.transfer_shadow(90.0);
            let mut tick = input(Some(70.0), 1.0);
            tick.mode = mode;
            tick.actuator = actuator;
            tick.max = 60.0;
            let guarded = loop_.tick(tick);
            close(guarded.cap, 60.0);
            close(guarded.thermal, 60.0);
            assert_eq!(guarded.hold, expected_hold);
            assert!(guarded.write_immediately);

            tick.max = 100.0;
            let reopened = loop_.tick(tick);
            close(loop_.thermal(), 60.0);
            assert!(!reopened.write_immediately);
        }
    }

    #[test]
    fn raised_floor_bypasses_slew_and_requests_an_immediate_write() {
        let mut loop_ = DeviceLoop::<Mhz>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(1_200.0, 0.0);
        let mut tick = input(Some(70.0), 1.0);
        tick.floor = 1_000.0;
        tick.max = 3_090.0;
        loop_.tick(tick);

        tick.floor = 1_800.0;
        let raised = loop_.tick(tick);
        close(raised.cap, 1_800.0);
        assert!(raised.write_immediately);
    }

    #[test]
    fn mismatch_and_resume_preserve_both_seeded_candidates() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        for resumed in [false, true] {
            let mut loop_ = DeviceLoop::<W>::new(gains);
            loop_.seed_candidates(80.0, 70.0, Some(70.0), 0.0);
            let mut tick = input(Some(70.0), 1.0);
            tick.actuator = ActuatorState::Mismatch;
            tick.resumed = resumed;
            let held = loop_.tick(tick);
            close(held.thermal, 80.0);
            close(held.shadow, 70.0);
            close(held.cap, 70.0);
        }
    }

    #[test]
    fn seed_and_explicit_resync_do_not_create_a_proportional_kick() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(50.0, 10.0);
        for dt in [2.0, 2.0, 1.0] {
            loop_.tick(input(Some(60.0), dt));
        }
        close(loop_.thermal(), 55.0);

        loop_.resync_error(-10.0);
        for dt in [2.0, 2.0, 1.0] {
            loop_.tick(input(Some(80.0), dt));
        }
        close(loop_.thermal(), 50.0);
    }

    fn first_order_cpu_response() -> (f64, f64) {
        let ambient = 40.0;
        let target = 80.0;
        let initial_cap = 10.0;
        let mut group = ambient + 0.8 * initial_cap;
        let mut delayed = std::collections::VecDeque::from(vec![initial_cap; 50]);
        let mut loop_ = DeviceLoop::<W>::new(default_gains::<W>(60));
        loop_.seed(initial_cap, target - group);
        let mut peak = group;
        for _ in 0..3_000 {
            let decision = loop_.tick(TickInput {
                t_star: target,
                group_c: Some(group),
                draw: Some(initial_cap),
                floor: 10.0,
                max: 100.0,
                mode: ThermalMode::Regulate,
                actuator: ActuatorState::Verified,
                dt_s: 1.0,
                resumed: false,
                delta_tstar: 0.0,
            });
            loop_.note_applied(decision.cap);
            delayed.push_back(decision.cap);
            let applied = delayed.pop_front().unwrap();
            group += (ambient + 0.8 * applied - group) / 35.0;
            peak = peak.max(group);
        }
        (group, peak)
    }

    fn first_order_gpu_response() -> (f64, f64) {
        let ambient = 40.0;
        let target = 80.0;
        let initial_cap = 1_000.0;
        let mut group = ambient + 0.02 * initial_cap;
        let mut delayed = std::collections::VecDeque::from(vec![initial_cap; 90]);
        let mut loop_ = DeviceLoop::<Mhz>::new(default_gains::<Mhz>(60));
        loop_.seed(initial_cap, target - group);
        let mut peak = group;
        for _ in 0..3_000 {
            let decision = loop_.tick(TickInput {
                t_star: target,
                group_c: Some(group),
                draw: Some(initial_cap),
                floor: 1_000.0,
                max: 3_090.0,
                mode: ThermalMode::Regulate,
                actuator: ActuatorState::Verified,
                dt_s: 1.0,
                resumed: false,
                delta_tstar: 0.0,
            });
            loop_.note_applied(decision.cap);
            delayed.push_back(decision.cap);
            let applied = delayed.pop_front().unwrap();
            group += (ambient + 0.02 * applied - group) / 15.0;
            peak = peak.max(group);
        }
        (group, peak)
    }

    #[test]
    fn live_imc_defaults_settle_first_order_cpu_and_gpu_within_overshoot_bar() {
        for (name, target, initial, response) in [
            ("cpu", 80.0, 48.0, first_order_cpu_response()),
            ("gpu", 80.0, 60.0, first_order_gpu_response()),
        ] {
            let (settled, peak) = response;
            let step = target - initial;
            assert!(
                (settled - target).abs() <= step * 0.01,
                "{name} settled at {settled}, target {target}"
            );
            assert!(
                peak <= target + step * 0.05,
                "{name} peaked at {peak}, target {target}"
            );
        }
    }
}

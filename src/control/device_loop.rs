//! Pure per-device temperature-loop control (§2.3, revision 4).

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

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
    const HOT_HEADROOM: f64;
}

impl sealed::Sealed for W {}
impl DeviceUnit for W {
    const TAU_S: f64 = 35.0;
    const PLANT_GAIN: f64 = 0.8;
    const BASE_DELAY_S: f64 = 20.0;
    const GRID: f64 = 0.5;
    const RISE_RATE: f64 = 10.0;
    const FALL_RATE: f64 = f64::INFINITY;
    const HOT_HEADROOM: f64 = 2.0;
}

impl sealed::Sealed for Mhz {}
impl DeviceUnit for Mhz {
    // GPU fit from run-1789252085 (2026-09-12, quiet16, 60 s MA).
    const TAU_S: f64 = 32.564_086_253_945_035;
    const PLANT_GAIN: f64 = 0.009_527_650_224_779_704;
    // The fitted 38.4515 s delay already contains the 60 s MA lag.
    // Remove its 30 s contribution here; default_gains adds the live MA.
    const BASE_DELAY_S: f64 = 38.451_542_929_544_05 - 30.0;
    const GRID: f64 = 1.0;
    const RISE_RATE: f64 = 105.0;
    const FALL_RATE: f64 = 105.0;
    const HOT_HEADROOM: f64 = 100.0;
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
    /// Draw margin used to form the shadow candidate.
    pub shadow_headroom: f64,
    /// Maximum downward shadow movement per second.
    pub shadow_fall_rate: f64,
    /// Whether this device's shadow candidate is active.
    pub shadow_enabled: bool,
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
    group_lost: bool,
    group_reset_pending: bool,
    group_missing_s: f64,
    draw_missing_s: f64,
    draw_missing: bool,
    draw_clock_s: f64,
    recent_draw: VecDeque<(f64, f64)>,
    mismatch_latched: bool,
    pending_immediate: bool,
    prev_group: Option<f64>,
    prev_t_star: Option<f64>,
    previous_selected: Option<Selected>,
    hot_episode_armed: bool,
    /// A hot engagement may precede usable draw history; trim once it arrives.
    hot_entry_trim_pending: bool,
    hot_rearm_s: f64,
    control_entry_pending: bool,
    entry_candidates_seeded: bool,
    last_mode: Option<ThermalMode>,
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
            group_lost: false,
            group_reset_pending: false,
            group_missing_s: 0.0,
            draw_missing_s: 0.0,
            draw_missing: false,
            draw_clock_s: 0.0,
            recent_draw: VecDeque::new(),
            mismatch_latched: false,
            pending_immediate: false,
            prev_group: None,
            prev_t_star: None,
            previous_selected: None,
            hot_episode_armed: true,
            hot_entry_trim_pending: false,
            hot_rearm_s: 0.0,
            control_entry_pending: true,
            entry_candidates_seeded: false,
            last_mode: None,
            unit: std::marker::PhantomData,
        }
    }

    /// Re-resolve the PI gains for a strategy or moving-average interval
    /// change without turning it into a new control entry.  Callers must use
    /// the same validated gain source as [`Self::new`]; keeping every other
    /// field intact is what prevents a gain-key change from stepping an
    /// already-applied cap.
    pub fn set_gains(&mut self, gains: Gains) {
        debug_assert!(gains.is_valid());
        self.gains = gains;
    }

    /// Clears an ended engagement while preserving the resolved gains.
    pub fn reset_engagement(&mut self) {
        *self = Self::new(self.gains);
    }

    /// Seeds the loop from a cap already in force and synchronises its error.
    #[cfg(test)]
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
        self.entry_candidates_seeded = true;
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

    /// Installs the first-ever measured draw candidate after thermal-only
    /// entry. Later draw outages use the ordinary applied-cap recovery seam.
    pub fn seed_initial_shadow(&mut self, cap: f64) {
        self.transfer_shadow(cap);
        self.draw_missing = false;
        self.draw_missing_s = 0.0;
    }

    pub fn resync_error(&mut self, error: f64) {
        self.e_prev = error;
    }

    pub fn note_applied(&mut self, cap: f64) {
        self.last_applied = Some(cap.clamp(self.floor, self.max));
    }

    #[cfg(test)]
    pub fn thermal(&self) -> f64 {
        self.thermal
    }

    #[cfg(test)]
    pub fn requested(&self) -> Option<f64> {
        self.requested
    }

    /// Applies a guard ceiling. Lowering clamps the candidates and requested
    /// cap; `last_applied` remains a record of the last successful command.
    pub fn clamp_max(&mut self, max: f64) -> bool {
        if !max.is_finite() || max >= self.max {
            if max.is_finite() {
                self.max = max;
            }
            return false;
        }
        self.max = max.max(self.floor);
        let request_lowered = self.requested.is_some_and(|value| value > self.max);
        let applied_above_max = self.last_applied.is_some_and(|value| value > self.max);
        for value in [&mut self.thermal, &mut self.shadow] {
            if *value > self.max {
                *value = self.max;
            }
        }
        if let Some(value) = self.requested.as_mut()
            && *value > self.max
        {
            *value = self.max;
        }
        self.pending_immediate |= request_lowered || applied_above_max;
        request_lowered
    }

    pub fn tick(&mut self, input: TickInput) -> DeviceDecision {
        let dt = control_dt(input.dt_s);
        self.draw_clock_s += dt;
        if input.resumed || input.dt_s > 2.0 || dt == 0.0
            || input.group_c.is_none() || input.actuator == ActuatorState::Mismatch
            || (self.mismatch_latched && input.actuator != ActuatorState::Verified)
        {
            self.recent_draw.clear();
        } else if let Some(draw) = input.draw.filter(|d| d.is_finite() && *d > 0.0) {
            self.recent_draw.push_back((self.draw_clock_s, draw));
            while self.recent_draw.len() > 1
                && self.recent_draw[1].0 <= self.draw_clock_s - 5.0
            {
                self.recent_draw.pop_front();
            }
        } else {
            self.recent_draw.clear();
        }
        let (floor, max) = ordered_bounds(input.floor, input.max);
        let floor_changed = self.bounds_initialised && floor != self.floor;
        let max_lowered = self.bounds_initialised && max < self.max;
        let mut max_request_lowered = false;
        if !self.bounds_initialised {
            self.floor = floor;
            self.max = max;
            self.bounds_initialised = true;
            self.pending_immediate |= self.clamp_all_to_bounds();
        } else {
            self.floor = floor;
            if max_lowered {
                max_request_lowered = self.clamp_max(max);
            } else {
                self.max = max;
            }
            let bounds_clamped = self.clamp_all_to_bounds();
            self.pending_immediate |= floor_changed && bounds_clamped;
        }

        let err = input.group_c.map(|group| input.t_star - group);
        if input.mode != ThermalMode::Regulate || !input.shadow_enabled
            || err.is_some_and(|error| error >= 0.0)
        {
            self.hot_entry_trim_pending = false;
        }
        if input.resumed {
            self.mismatch_latched = false;
        }
        let mismatch_recovered = match input.actuator {
            ActuatorState::Mismatch => {
                self.mismatch_latched = true;
                false
            }
            ActuatorState::Verified if self.mismatch_latched => {
                self.mismatch_latched = false;
                true
            }
            ActuatorState::Verified | ActuatorState::Unverifiable => false,
        };

        if input.resumed {
            self.elapsed_s = 0.0;
            self.group_missing_s = 0.0;
            self.group_missing = input.group_c.is_none();
            self.group_lost = false;
            self.group_reset_pending = false;
            self.draw_missing_s = 0.0;
            self.draw_missing = false;
            self.hot_rearm_s = 0.0;
            self.prev_group = None;
            self.prev_t_star = None;
            self.previous_selected = None;
            self.last_mode = Some(input.mode);
            if let Some(error) = err {
                self.resync_error(error);
            }
            let cap = self
                .last_applied
                .or(self.requested)
                .unwrap_or(self.max)
                .clamp(self.floor, self.max);
            if !self.entry_candidates_seeded {
                self.thermal = cap;
                self.shadow = cap;
                self.entry_candidates_seeded = true;
            }
            self.requested = Some(cap);
            let write_allowed = input.group_c.is_some() || self.group_seen;
            self.pending_immediate = write_allowed;
            return self.decision(input, cap, self.thermal, self.shadow, write_allowed, false);
        }

        let Some(error) = err else {
            self.elapsed_s = 0.0;
            self.hot_rearm_s = 0.0;
            self.group_missing = true;
            if self.group_seen {
                self.group_missing_s += dt;
            }
            if self.group_seen
                && !self.group_lost
                && self.group_missing_s >= GROUP_UNAVAILABLE_DWELL_S
            {
                self.group_lost = true;
                self.requested = Some(self.quantize(self.max));
                self.group_reset_pending = true;
            }
            if self.group_reset_pending && !self.mismatch_latched {
                self.thermal = self.max;
                self.shadow = self.max;
                self.group_reset_pending = false;
            }
            let cap = if self.group_lost {
                self.max
            } else {
                self.last_applied.unwrap_or(self.max)
            };
            let cap = self.quantize(cap).clamp(self.floor, self.max);
            return self.decision(
                input,
                cap,
                self.thermal,
                self.max,
                self.group_seen,
                self.group_lost,
            );
        };

        let first_group = !self.group_seen;
        let recovered_group = self.group_missing;
        if first_group || recovered_group {
            self.group_seen = true;
            self.group_missing = false;
            self.group_missing_s = 0.0;
            self.group_lost = false;
        }
        if self.group_reset_pending && !self.mismatch_latched {
            self.thermal = self.max;
            self.shadow = if input.shadow_enabled {
                input
                    .draw
                    .map(|draw| shadow_target(draw, input.shadow_headroom, self.floor, self.max))
                    .unwrap_or(self.max)
            } else {
                self.max
            };
            self.requested = Some(self.quantize(self.max));
            self.group_reset_pending = false;
        }
        let resynced_error = first_group || recovered_group || mismatch_recovered;
        if resynced_error {
            self.resync_error(error);
            self.elapsed_s = 0.0;
        }

        if self.mismatch_latched {
            self.elapsed_s = 0.0;
            if error < 0.0 {
                self.hot_rearm_s = 0.0;
            }
            let cap = self
                .requested
                .unwrap_or(self.max)
                .clamp(self.floor, self.max);
            let decision = self.decision(input, cap, self.thermal, self.shadow, true, false);
            self.record_valid_tick(
                input.group_c.expect("valid group"),
                input.t_star,
                decision.selected,
            );
            return decision;
        }

        let group = input.group_c.expect("valid group");
        let mode_changed = self.last_mode.is_some_and(|mode| mode != input.mode);
        if mode_changed {
            let cap = self
                .last_applied
                .or(self.requested)
                .unwrap_or(self.max)
                .clamp(self.floor, self.max);
            match input.mode {
                ThermalMode::Bypass => self.transfer_shadow(cap),
                ThermalMode::Regulate => {
                    self.transfer_thermal(cap, error);
                    self.hot_episode_armed = error >= 0.0;
                    self.hot_rearm_s = 0.0;
                }
            }
            self.requested = Some(self.quantize(cap));
            let thermal = if input.mode == ThermalMode::Bypass {
                self.max
            } else {
                self.thermal
            };
            let decision = self.decision(input, cap, thermal, self.shadow, true, false);
            self.record_valid_tick(group, input.t_star, decision.selected);
            self.control_entry_pending = false;
            self.last_mode = Some(input.mode);
            return decision;
        }

        // Hot entry has already transferred from the applied cap, but may
        // have had no power reading yet. Once history is ready, skip unused
        // headroom exactly once; never increase a PI candidate already lower.
        if self.hot_entry_trim_pending && !resynced_error
            && self.recent_draw.front()
                .is_some_and(|(time, _)| self.draw_clock_s - time >= 5.0)
        {
            self.hot_entry_trim_pending = false;
            let applied = self.last_applied.or(self.requested).unwrap_or(self.max)
                .clamp(self.floor, self.max);
            let peak = self.recent_draw.iter().map(|(_, draw)| *draw).fold(0.0, f64::max);
            let thermal = self.thermal.min(applied)
                .min((peak + U::HOT_HEADROOM.min(input.shadow_headroom))
                    .clamp(self.floor, self.max));
            if thermal < self.thermal {
                self.transfer_thermal(thermal, error);
                self.requested = Some(applied);
                let (target, selected) = select_candidates(thermal, self.shadow, self.floor, self.max);
                let cap = self.slew_and_quantize(target, selected, dt, false);
                self.requested = Some(cap);
                let decision = self.decision(input, cap, thermal, self.shadow, true, false);
                self.record_valid_tick(group, input.t_star, decision.selected);
                self.last_mode = Some(input.mode);
                return decision;
            }
        }

        // A moving target can make us hot without crossing the old target.
        // Transfer once from the applied cap; never follow hot draw dips.
        let handover = input.mode == ThermalMode::Regulate
            && !resynced_error
            && self.hot_episode_armed
            && self.previous_selected == Some(Selected::Shadow)
            // Let draw recovery reseed the shadow before a hot transfer.
            && !(self.draw_missing && input.draw.is_some())
            && error < 0.0;
        let initial_hot_entry =
            self.control_entry_pending && input.mode == ThermalMode::Regulate && error < 0.0;
        if handover || initial_hot_entry {
            let cap = if initial_hot_entry {
                if let Some(cap) = self.last_applied {
                    cap.clamp(self.floor, self.max)
                } else if self.entry_candidates_seeded {
                    self.shadow.clamp(self.floor, self.max)
                } else if input.shadow_enabled {
                    input
                        .draw
                        .map(|draw| {
                            shadow_target(draw, input.shadow_headroom, self.floor, self.max)
                        })
                        .unwrap_or(self.max)
                } else {
                    self.max
                }
            } else {
                self.last_applied
                    .or(self.requested)
                    .unwrap_or(self.shadow)
                    .clamp(self.floor, self.max)
            };
            // Trim only at a hot handoff with a full recent history, never
            // repeatedly as draw falls. Use its peak to reject brief dips.
            let thermal_cap = if handover && self.recent_draw.front()
                .is_some_and(|(time, _)| self.draw_clock_s - time >= 5.0)
            {
                let peak = self.recent_draw.iter().map(|(_, draw)| *draw).fold(0.0, f64::max);
                cap.min((peak + U::HOT_HEADROOM.min(input.shadow_headroom))
                    .clamp(self.floor, self.max))
            } else { cap };
            self.thermal = thermal_cap;
            let cap = if thermal_cap < cap {
                // Pending requests may not have reached the rate-limited
                // writer. A hot reduction must slew from hardware, not them.
                self.requested = Some(cap);
                self.slew_and_quantize(thermal_cap, Selected::Thermal, dt, false)
            } else { cap };
            if initial_hot_entry {
                self.shadow = cap;
                self.hot_entry_trim_pending = input.shadow_enabled;
            }
            self.requested = Some(self.quantize(cap));
            self.resync_error(error);
            self.elapsed_s = 0.0;
            self.hot_episode_armed = false;
            self.hot_rearm_s = 0.0;
            let decision = self.decision(input, cap, self.thermal, self.shadow, true, false);
            self.record_valid_tick(group, input.t_star, decision.selected);
            self.control_entry_pending = false;
            self.last_mode = Some(input.mode);
            return decision;
        }

        if self.hot_episode_armed {
            // An idle loop is already armed; no dwell is required until the
            // first hot episode has consumed the arm.
        } else if error >= 0.0 {
            self.hot_rearm_s += dt;
            if self.hot_rearm_s >= PI_PERIOD_S {
                self.hot_episode_armed = true;
                self.hot_rearm_s = 0.0;
            }
        } else {
            self.hot_rearm_s = 0.0;
        }

        let draw_returned = input.draw.is_some() && self.draw_missing;
        let shadow_target = if input.shadow_enabled {
            input
                .draw
                .map(|draw| shadow_target(draw, input.shadow_headroom, self.floor, self.max))
                .unwrap_or(self.max)
        } else {
            self.max
        };
        if input.draw.is_none() {
            self.draw_missing = true;
            self.draw_missing_s += dt;
        } else {
            self.draw_missing = false;
            self.draw_missing_s = 0.0;
        }
        if draw_returned {
            self.transfer_shadow(self.last_applied.or(self.requested).unwrap_or(self.shadow));
        } else if !input.shadow_enabled {
            self.shadow = self.max;
        } else if input.draw.is_some() {
            let moves = input.mode == ThermalMode::Bypass || error >= 0.0;
            if moves {
                self.shadow = self.move_shadow_toward(
                    shadow_target,
                    dt,
                    input.shadow_headroom,
                    input.shadow_fall_rate,
                );
            }
        } else if self.draw_missing_s >= GROUP_UNAVAILABLE_DWELL_S {
            self.shadow = self.move_shadow_toward(
                self.max,
                dt,
                input.shadow_headroom,
                input.shadow_fall_rate,
            );
        }

        let thermal_candidate = match input.mode {
            ThermalMode::Bypass => self.max,
            ThermalMode::Regulate => {
                if !resynced_error {
                    self.e_prev += finite_or_zero(input.delta_tstar);
                }
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
        let cap =
            self.slew_and_quantize(target, selected, dt, floor_changed || max_request_lowered);
        self.requested = Some(cap);
        let decision = self.decision(input, cap, thermal_candidate, self.shadow, true, false);
        self.record_valid_tick(group, input.t_star, decision.selected);
        self.control_entry_pending = false;
        self.last_mode = Some(input.mode);
        decision
    }

    fn move_shadow_toward(&self, target: f64, dt: f64, headroom: f64, fall_rate: f64) -> f64 {
        let rise_rate = if U::GRID == 0.5 {
            positive_or_zero(headroom)
        } else {
            300.0
        };
        let fall_rate = positive_or_zero(fall_rate);
        let delta = target - self.shadow;
        let step = if delta >= 0.0 {
            rise_rate * dt
        } else {
            fall_rate * dt
        };
        (self.shadow + delta.clamp(-step, step)).clamp(self.floor, self.max)
    }

    fn record_valid_tick(&mut self, group: f64, t_star: f64, selected: Selected) {
        self.prev_group = Some(group);
        self.prev_t_star = Some(t_star);
        self.previous_selected = Some(selected);
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
        changed |= self
            .last_applied
            .is_some_and(|value| value < self.floor || value > self.max);
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

fn positive_or_zero(value: f64) -> f64 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        0.0
    }
}

fn shadow_target(draw: f64, headroom: f64, floor: f64, max: f64) -> f64 {
    (draw + positive_or_zero(headroom)).clamp(floor, max)
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
        close(gpu_60.ti_s, 32.564_086_253_945_035);
        close(gpu_60.kc, 22.221_804_817_775_82);

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
            shadow_headroom: 10.0,
            shadow_fall_rate: 0.33,
            shadow_enabled: false,
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

        let mut recovered_on_resume = DeviceLoop::<W>::new(gains);
        recovered_on_resume.seed_candidates(80.0, 100.0, Some(60.0), 0.0);
        for _ in 0..29 {
            recovered_on_resume.tick(input(None, 2.0));
        }
        let mut resumed_present = input(Some(70.0), 7_200.0);
        resumed_present.resumed = true;
        close(recovered_on_resume.tick(resumed_present).cap, 60.0);
        let live = recovered_on_resume.tick(input(Some(70.0), 1.0));
        close(live.thermal, 80.0);
        close(live.cap, 70.0);
    }

    #[test]
    fn resumed_absent_loop_keeps_its_held_cap_on_first_hot_entry() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        let mut resumed = input(None, 7_200.0);
        resumed.resumed = true;
        resumed.shadow_enabled = true;
        let held = loop_.tick(resumed);
        close(held.cap, 100.0);

        let mut hot = input(Some(80.0), 1.0);
        hot.shadow_enabled = true;
        let entered = loop_.tick(hot);
        close(entered.thermal, 100.0);
        close(entered.cap, 100.0);
        close(loop_.requested().expect("entry request"), 100.0);
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
    fn shadow_candidate_uses_unit_rise_and_configured_fall_slews() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut cpu = DeviceLoop::<W>::new(gains);
        cpu.seed_candidates(100.0, 50.0, Some(50.0), 10.0);
        let mut cpu_tick = input(Some(60.0), 1.0);
        cpu_tick.draw = Some(80.0);
        cpu_tick.shadow_headroom = 10.0;
        cpu_tick.shadow_fall_rate = 0.5;
        cpu_tick.shadow_enabled = true;
        let raised = cpu.tick(cpu_tick);
        close(raised.shadow, 60.0);
        close(raised.cap, 60.0);
        assert_eq!(raised.selected, Selected::Shadow);
        assert_eq!(raised.hold, Hold::Shadow);

        cpu_tick.draw = Some(20.0);
        let fallen = cpu.tick(cpu_tick);
        close(fallen.shadow, 59.5);

        let mut gpu = DeviceLoop::<Mhz>::new(gains);
        gpu.seed_candidates(3_090.0, 1_000.0, Some(1_000.0), 10.0);
        let mut gpu_tick = input(Some(60.0), 1.0);
        gpu_tick.floor = 1_000.0;
        gpu_tick.max = 3_090.0;
        gpu_tick.draw = Some(1_500.0);
        gpu_tick.shadow_headroom = 30.0;
        gpu_tick.shadow_fall_rate = 10.0;
        gpu_tick.shadow_enabled = true;
        let raised = gpu.tick(gpu_tick);
        close(raised.shadow, 1_300.0);
        close(raised.cap, 1_300.0);

        gpu_tick.draw = Some(1_000.0);
        let fallen = gpu.tick(gpu_tick);
        close(fallen.shadow, 1_290.0);
    }

    #[test]
    fn hot_entry_with_late_draw_skips_deadzone_once_after_five_valid_seconds() {
        let mut cpu = DeviceLoop::<W>::new(Gains { kc: 0.279475988, ti_s: 29.809252495 });
        let mut tick = input(Some(85.0), 1.0);
        tick.t_star = 81.0;
        tick.max = 54.0;
        tick.floor = 15.0;
        tick.shadow_enabled = true;
        tick.draw = None;
        for _ in 0..4 {
            let d = cpu.tick(tick);
            cpu.note_applied(d.cap);
        }
        tick.draw = Some(39.0);
        for _ in 0..5 {
            let d = cpu.tick(tick);
            cpu.note_applied(d.cap);
            assert!(d.thermal > 50.0, "must collect a full five-second history");
        }
        let trimmed = cpu.tick(tick);
        close(trimmed.thermal, 41.0);
        close(trimmed.cap, 41.0);
        cpu.note_applied(trimmed.cap);
        tick.draw = Some(20.0);
        for _ in 0..10 {
            let d = cpu.tick(tick);
            cpu.note_applied(d.cap);
            assert!(d.thermal > 40.0, "must not chase later draw dips");
        }
    }

    fn pending_hot_cpu() -> (DeviceLoop<W>, TickInput) {
        let mut cpu = DeviceLoop::<W>::new(Gains { kc: 0.28, ti_s: 30.0 });
        let mut tick = input(Some(74.0), 1.0);
        tick.max = 54.0;
        tick.shadow_enabled = true;
        tick.draw = None;
        let d = cpu.tick(tick);
        cpu.note_applied(d.cap);
        tick.draw = Some(39.0);
        (cpu, tick)
    }

    #[test]
    fn deferred_hot_trim_restarts_evidence_after_every_sampling_interruption() {
        for interruption in 0..6 {
            let (mut cpu, tick) = pending_hot_cpu();
            for _ in 0..4 {
                let d = cpu.tick(tick);
                cpu.note_applied(d.cap);
            }
            let mut gap = tick;
            match interruption {
                0 => gap.resumed = true,
                1 => gap.group_c = None,
                2 => gap.draw = None,
                3 => gap.actuator = ActuatorState::Mismatch,
                4 => gap.dt_s = 3.0,
                _ => gap.dt_s = 0.0,
            }
            cpu.tick(gap);
            for _ in 0..5 {
                let d = cpu.tick(tick);
                assert!(d.thermal > 50.0, "premature trim after interruption {interruption}");
                cpu.note_applied(d.cap);
            }
            let d = cpu.tick(tick);
            close(d.thermal, 41.0);
        }
    }

    #[test]
    fn deferred_hot_trim_waits_for_fresh_history_after_mismatch_latch_clears() {
        let (mut cpu, tick) = pending_hot_cpu();
        let mut mismatch = tick;
        mismatch.actuator = ActuatorState::Mismatch;
        cpu.tick(mismatch);
        let mut blind = tick;
        blind.actuator = ActuatorState::Unverifiable;
        for _ in 0..8 { cpu.tick(blind); }
        for _ in 0..5 {
            let d = cpu.tick(tick);
            assert!(d.thermal > 50.0, "mismatch-latched readings cannot qualify history");
            cpu.note_applied(d.cap);
        }
        close(cpu.tick(tick).thermal, 41.0);
    }

    #[test]
    fn deferred_hot_trim_is_cancelled_when_cool_or_shadow_disabled() {
        for disable_shadow in [false, true] {
            let (mut cpu, tick) = pending_hot_cpu();
            let mut cancel = tick;
            if disable_shadow { cancel.shadow_enabled = false; }
            else { cancel.group_c = Some(69.0); }
            cpu.tick(cancel);
            for _ in 0..8 {
                let d = cpu.tick(tick);
                cpu.note_applied(d.cap);
                assert!(d.thermal > 50.0);
            }
        }
    }

    #[test]
    fn deferred_hot_trim_keeps_lower_thermal_candidate_and_uses_recent_peak() {
        let (mut cpu, tick) = pending_hot_cpu();
        for _ in 0..5 { cpu.tick(tick); }
        let mut dip = tick;
        dip.draw = Some(20.0);
        close(cpu.tick(dip).thermal, 41.0);

        let (mut cpu, tick) = pending_hot_cpu();
        cpu.transfer_thermal(30.0, -4.0);
        cpu.note_applied(30.0);
        for _ in 0..6 {
            let d = cpu.tick(tick);
            assert!(d.thermal <= 30.0);
            assert!(d.cap <= 30.0);
            cpu.note_applied(d.cap);
        }
    }

    #[test]
    fn deferred_gpu_trim_uses_applied_cap_and_existing_downward_slew() {
        let mut gpu = DeviceLoop::<Mhz>::new(Gains { kc: 1.0, ti_s: 30.0 });
        gpu.seed_candidates(3090.0, 2800.0, Some(2800.0), -4.0);
        let mut tick = input(Some(74.0), 1.0);
        tick.floor = 1000.0;
        tick.max = 3090.0;
        tick.shadow_headroom = 300.0;
        tick.shadow_enabled = true;
        tick.draw = None;
        gpu.tick(tick);
        tick.draw = Some(2400.0);
        for _ in 0..5 { gpu.tick(tick); }
        gpu.requested = Some(3000.0); // a request still ahead of the writer
        let d = gpu.tick(tick);
        close(d.thermal, 2500.0);
        close(d.cap, 2695.0);
    }

    #[test]
    fn hot_headroom_slews_from_applied_not_an_unwritten_request() {
        let mut gpu = DeviceLoop::<Mhz>::new(Gains { kc: 1.0, ti_s: 10.0 });
        gpu.seed_candidates(3090.0, 2600.0, Some(2450.0), 10.0);
        let mut tick = input(Some(60.0), 1.0);
        tick.floor = 1000.0; tick.max = 3090.0;
        tick.shadow_enabled = true; tick.shadow_headroom = 300.0;
        tick.draw = Some(2300.0);
        for _ in 0..6 { gpu.tick(tick); }
        tick.group_c = Some(71.0);
        let hot = gpu.tick(tick);
        close(hot.thermal, 2400.0);
        close(hot.cap, 2400.0);
    }

    #[test]
    fn hot_handoff_trims_headroom_using_recent_peak_and_preserves_gpu_slew() {
        let gains = Gains { kc: 1.0, ti_s: 10.0 };
        let mut cpu = DeviceLoop::<W>::new(gains);
        cpu.seed_candidates(100.0, 50.0, Some(50.0), 10.0);
        let mut tick = input(Some(60.0), 1.0);
        tick.shadow_enabled = true;
        tick.draw = Some(40.0);
        for _ in 0..6 { cpu.tick(tick); }
        tick.group_c = Some(71.0);
        tick.draw = Some(10.0); // a brief dip must not seed a 12 W cap
        let hot = cpu.tick(tick);
        close(hot.thermal, 42.0);
        close(hot.cap, 42.0);
        for _ in 0..3 { close(cpu.tick(tick).thermal, 42.0); }

        let mut gpu = DeviceLoop::<Mhz>::new(gains);
        gpu.seed_candidates(3090.0, 2800.0, Some(2800.0), 10.0);
        tick = input(Some(60.0), 1.0);
        tick.floor = 1000.0;
        tick.max = 3090.0;
        tick.shadow_enabled = true;
        tick.shadow_headroom = 300.0;
        tick.draw = Some(2500.0);
        for _ in 0..6 { gpu.tick(tick); }
        tick.group_c = Some(71.0);
        tick.draw = Some(1000.0);
        let hot = gpu.tick(tick);
        close(hot.thermal, 2600.0);
        close(hot.cap, 2695.0); // existing 105 MHz/s downward slew
    }

    #[test]
    fn measured_crossing_hands_thermal_over_once_and_rearms_after_five_cool_seconds() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut loop_ = DeviceLoop::<W>::new(gains);
        loop_.seed_candidates(100.0, 50.0, Some(50.0), 10.0);
        let mut cool = input(Some(60.0), 1.0);
        cool.shadow_enabled = true;
        assert_eq!(loop_.tick(cool).selected, Selected::Shadow);

        let mut hot = cool;
        hot.group_c = Some(71.0);
        let handed = loop_.tick(hot);
        close(handed.thermal, 50.0);
        close(handed.cap, 50.0);

        hot.draw = Some(10.0);
        for _ in 0..3 {
            assert_eq!(loop_.tick(hot).thermal, 50.0);
        }

        let mut cool = hot;
        cool.group_c = Some(60.0);
        cool.draw = Some(40.0);
        cool.dt_s = 2.0;
        loop_.tick(cool);
        loop_.tick(cool);
        cool.dt_s = 1.0;
        loop_.tick(cool);

        loop_.transfer_thermal(100.0, 10.0);
        loop_.transfer_shadow(50.0);
        assert_eq!(loop_.tick(cool).selected, Selected::Shadow);
        hot.dt_s = 1.0;
        hot.draw = Some(40.0);
        close(loop_.tick(hot).thermal, 42.0);
    }

    #[test]
    fn target_only_hot_sign_change_hands_over_but_hot_draw_dip_does_not_track_thermal() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut target_only = DeviceLoop::<W>::new(gains);
        target_only.seed_candidates(100.0, 50.0, Some(50.0), 10.0);
        let mut cool = input(Some(60.0), 1.0);
        cool.shadow_enabled = true;
        assert_eq!(target_only.tick(cool).selected, Selected::Shadow);
        let mut changed_target = cool;
        changed_target.t_star = 50.0;
        changed_target.delta_tstar = -20.0;
        let unchanged = target_only.tick(changed_target);
        close(unchanged.thermal, 50.0);

        let mut replay = DeviceLoop::<W>::new(gains);
        let mut dipped = DeviceLoop::<W>::new(gains);
        for loop_ in [&mut replay, &mut dipped] {
            loop_.seed_candidates(80.0, 50.0, Some(50.0), -10.0);
        }
        let mut hot = input(Some(80.0), 2.0);
        hot.draw = Some(40.0);
        hot.shadow_enabled = true;
        for tick in 0..30 {
            replay.tick(hot);
            if tick == 5 {
                hot.draw = Some(10.0);
            }
            dipped.tick(hot);
        }
        close(replay.thermal(), dipped.thermal());
    }

    #[test]
    fn missing_draw_dwell_return_and_shadow_toggle_use_slew_without_a_step() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut loop_ = DeviceLoop::<W>::new(gains);
        loop_.seed_candidates(100.0, 50.0, Some(50.0), 10.0);
        let mut missing = input(Some(60.0), 2.0);
        missing.draw = None;
        missing.shadow_enabled = true;
        for _ in 0..29 {
            close(loop_.tick(missing).shadow, 50.0);
        }
        let expired = loop_.tick(missing);
        close(expired.shadow, 70.0);
        close(expired.cap, 70.0);

        let mut returned = missing;
        returned.draw = Some(40.0);
        let returned = loop_.tick(returned);
        close(returned.shadow, 50.0);
        close(returned.cap, 50.0);

        let mut gpu = DeviceLoop::<Mhz>::new(gains);
        gpu.seed_candidates(3_090.0, 1_000.0, Some(1_000.0), 10.0);
        let mut toggle = input(Some(60.0), 1.0);
        toggle.floor = 1_000.0;
        toggle.max = 3_090.0;
        toggle.draw = Some(1_000.0);
        toggle.shadow_headroom = 300.0;
        toggle.shadow_fall_rate = 10.0;
        toggle.shadow_enabled = false;
        let disabled = gpu.tick(toggle);
        close(disabled.shadow, 3_090.0);
        close(disabled.cap, 1_105.0);
        toggle.shadow_enabled = true;
        let enabled = gpu.tick(toggle);
        close(enabled.shadow, 3_080.0);
        close(enabled.cap, 1_405.0);
    }

    #[test]
    fn bypass_transfers_unequal_candidates_without_a_step_and_initial_hot_entry_is_bumpless() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut loop_ = DeviceLoop::<W>::new(gains);
        loop_.seed_candidates(40.0, 80.0, Some(30.0), 10.0);
        let mut tick = input(Some(60.0), 0.0);
        tick.draw = Some(70.0);
        tick.shadow_enabled = true;
        close(loop_.tick(tick).cap, 30.0);
        tick.mode = ThermalMode::Bypass;
        tick.dt_s = 1.0;
        let entering = loop_.tick(tick);
        close(entering.cap, 30.0);
        assert_eq!(entering.hold, Hold::Bypass);
        close(entering.shadow, 30.0);
        let bypassed = loop_.tick(tick);
        assert!(bypassed.shadow > 30.0);

        tick.mode = ThermalMode::Regulate;
        let exiting = loop_.tick(tick);
        close(exiting.cap, 30.0);
        close(exiting.thermal, 30.0);

        let mut initial_hot = DeviceLoop::<W>::new(gains);
        let mut hot = input(Some(80.0), 1.0);
        hot.draw = Some(40.0);
        hot.shadow_enabled = true;
        let entered = initial_hot.tick(hot);
        close(entered.thermal, 50.0);
        close(entered.shadow, 50.0);
        close(entered.cap, 50.0);

        let mut clamped = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        clamped.seed_candidates(90.0, 50.0, Some(150.0), 10.0);
        close(clamped.tick(hot).thermal, 100.0);
    }

    #[test]
    fn seeded_first_hot_entry_transfers_from_the_successfully_applied_cap() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed_candidates(90.0, 50.0, Some(50.0), 10.0);
        let mut hot = input(Some(80.0), 1.0);
        hot.draw = Some(40.0);
        hot.shadow_enabled = true;

        let entered = loop_.tick(hot);
        close(entered.thermal, 50.0);
        close(entered.shadow, 50.0);
        close(entered.cap, 50.0);
    }

    #[test]
    fn initial_hot_entry_without_an_applied_cap_uses_the_seeded_shadow() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed_candidates(90.0, 50.0, None, 10.0);
        let mut hot = input(Some(80.0), 1.0);
        hot.draw = Some(80.0);
        hot.shadow_enabled = true;

        let entered = loop_.tick(hot);
        close(entered.thermal, 50.0);
        close(entered.cap, 50.0);
        close(loop_.requested().expect("entry request"), 50.0);
    }

    #[test]
    fn mismatch_defers_mode_transfer_and_negative_samples_break_hot_rearm() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed_candidates(40.0, 80.0, Some(30.0), 10.0);
        let mut regulate = input(Some(60.0), 0.0);
        regulate.draw = Some(70.0);
        regulate.shadow_enabled = true;
        loop_.tick(regulate);

        let mut mismatch = regulate;
        mismatch.mode = ThermalMode::Bypass;
        mismatch.actuator = ActuatorState::Mismatch;
        mismatch.group_c = Some(80.0);
        let held = loop_.tick(mismatch);
        close(held.thermal, 40.0);
        close(held.shadow, 80.0);
        assert_eq!(held.hold, Hold::ActuatorMismatch);

        mismatch.actuator = ActuatorState::Verified;
        let transferred = loop_.tick(mismatch);
        close(transferred.cap, 30.0);
        close(transferred.shadow, 30.0);
        assert_eq!(transferred.hold, Hold::Bypass);

        mismatch.mode = ThermalMode::Regulate;
        mismatch.actuator = ActuatorState::Mismatch;
        let held_exit = loop_.tick(mismatch);
        close(held_exit.thermal, 40.0);
        close(held_exit.shadow, 30.0);
        mismatch.actuator = ActuatorState::Verified;
        let exited = loop_.tick(mismatch);
        close(exited.thermal, 30.0);
        close(exited.cap, 30.0);

        loop_.hot_episode_armed = false;
        loop_.hot_rearm_s = 4.0;
        mismatch.actuator = ActuatorState::Mismatch;
        loop_.tick(mismatch);
        close(loop_.hot_rearm_s, 0.0);
    }

    #[test]
    fn hot_rearm_needs_five_contiguous_seconds_and_missing_draw_presence_reseeds_at_zero_dt() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut loop_ = DeviceLoop::<W>::new(gains);
        loop_.seed_candidates(100.0, 50.0, Some(30.0), 10.0);
        loop_.hot_episode_armed = false;
        let mut cool = input(Some(60.0), 2.0);
        cool.shadow_enabled = true;
        loop_.tick(cool);
        loop_.tick(cool);
        assert!(!loop_.hot_episode_armed);
        cool.dt_s = 1.0;
        loop_.tick(cool);
        assert!(loop_.hot_episode_armed);

        loop_.hot_rearm_s = 4.0;
        loop_.tick(input(None, 0.0));
        close(loop_.hot_rearm_s, 0.0);

        let mut missing = input(Some(80.0), 0.0);
        missing.draw = None;
        missing.shadow_enabled = true;
        loop_.transfer_shadow(50.0);
        loop_.tick(missing);
        missing.draw = Some(80.0);
        let returned = loop_.tick(missing);
        close(returned.shadow, 30.0);
    }

    #[test]
    fn hot_shadow_cases_keep_pi_state_and_bypass_still_moves_with_jittered_elapsed_time() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };
        let mut replay = DeviceLoop::<W>::new(gains);
        let mut dipped = DeviceLoop::<W>::new(gains);
        for loop_ in [&mut replay, &mut dipped] {
            loop_.seed_candidates(80.0, 50.0, Some(50.0), -10.0);
        }
        let mut replay_tick = input(Some(80.0), 2.0);
        replay_tick.draw = Some(40.0);
        replay_tick.shadow_enabled = true;
        let mut dipped_tick = replay_tick;
        for tick in 0..30 {
            replay.tick(replay_tick);
            if tick == 5 {
                dipped_tick.draw = Some(10.0);
            }
            dipped.tick(dipped_tick);
        }
        close(replay.thermal, dipped.thermal);
        close(replay.e_prev, dipped.e_prev);
        close(replay.shadow, dipped.shadow);
        assert_eq!(replay.requested, dipped.requested);
        close(replay.elapsed_s, dipped.elapsed_s);

        let mut hot_missing = input(Some(80.0), 2.0);
        hot_missing.draw = None;
        hot_missing.shadow_enabled = true;
        let before_missing = replay.shadow;
        for _ in 0..29 {
            close(replay.tick(hot_missing).shadow, before_missing);
        }
        assert!(replay.tick(hot_missing).shadow > before_missing);

        let mut bypass = DeviceLoop::<W>::new(gains);
        bypass.seed_candidates(100.0, 50.0, Some(50.0), -10.0);
        let mut bypass_tick = input(Some(80.0), 0.4);
        bypass_tick.mode = ThermalMode::Bypass;
        bypass_tick.draw = Some(80.0);
        bypass_tick.shadow_enabled = true;
        let first = bypass.tick(bypass_tick);
        close(first.shadow, 54.0);
        bypass_tick.dt_s = 1.6;
        close(bypass.tick(bypass_tick).shadow, 70.0);
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
    fn lowered_max_does_not_bypass_upward_slew_when_request_is_below_the_new_ceiling() {
        let mut loop_ = DeviceLoop::<Mhz>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed_candidates(2_500.0, 3_090.0, Some(1_500.0), 0.0);
        let mut tick = input(Some(70.0), 0.0);
        tick.floor = 1_000.0;
        tick.max = 3_090.0;
        close(loop_.tick(tick).cap, 1_500.0);

        tick.dt_s = 1.0;
        tick.max = 2_000.0;
        let lowered = loop_.tick(tick);
        close(lowered.thermal, 2_000.0);
        close(lowered.cap, 1_605.0);
        assert!(!lowered.write_immediately);
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
    fn short_group_dropout_preserves_candidates_and_pending_request_on_recovery() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed_candidates(80.0, 100.0, Some(60.0), 0.0);
        let first = loop_.tick(input(Some(70.0), 1.0));
        close(first.cap, 70.0);
        close(loop_.requested().unwrap(), 70.0);

        for _ in 0..29 {
            let held = loop_.tick(input(None, 2.0));
            close(held.cap, 60.0);
            assert!(!held.group_lost);
        }
        close(loop_.thermal(), 80.0);
        close(loop_.requested().unwrap(), 70.0);

        let recovered = loop_.tick(input(Some(70.0), 1.0));
        close(recovered.thermal, 80.0);
        close(recovered.cap, 80.0);
        close(loop_.requested().unwrap(), 80.0);
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
    fn failed_bound_writes_do_not_change_the_last_successfully_applied_cap() {
        let gains = Gains {
            kc: 1.0,
            ti_s: 10.0,
        };

        let mut ceiling = DeviceLoop::<W>::new(gains);
        ceiling.seed(80.0, 0.0);
        ceiling.tick(input(Some(70.0), 0.0));
        let mut lowered = input(Some(70.0), 0.0);
        lowered.max = 60.0;
        close(ceiling.tick(lowered).cap, 60.0);
        let mut reopened_missing = input(None, 0.0);
        reopened_missing.max = 100.0;
        close(ceiling.tick(reopened_missing).cap, 80.0);

        let mut floor = DeviceLoop::<W>::new(gains);
        floor.seed(20.0, 0.0);
        floor.tick(input(Some(70.0), 0.0));
        let mut raised = input(Some(70.0), 0.0);
        raised.floor = 40.0;
        close(floor.tick(raised).cap, 40.0);
        close(floor.tick(input(None, 0.0)).cap, 20.0);
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
    fn resumed_mismatch_latches_before_the_resume_hold_and_unverifiable_stays_frozen() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed_candidates(80.0, 70.0, Some(70.0), 0.0);

        let mut tick = input(Some(60.0), 2.0);
        tick.resumed = true;
        tick.actuator = ActuatorState::Mismatch;
        let resumed = loop_.tick(tick);
        assert_eq!(resumed.hold, Hold::ActuatorMismatch);
        close(resumed.thermal, 80.0);
        close(resumed.shadow, 70.0);

        tick.resumed = false;
        tick.actuator = ActuatorState::Unverifiable;
        for _ in 0..3 {
            let frozen = loop_.tick(tick);
            assert_eq!(frozen.hold, Hold::ActuatorMismatch);
            close(frozen.thermal, 80.0);
            close(frozen.shadow, 70.0);
        }
    }

    #[test]
    fn group_loss_recovery_does_not_mutate_candidates_while_mismatch_is_latched() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed_candidates(80.0, 70.0, Some(70.0), 0.0);

        let mut mismatch = input(Some(70.0), 1.0);
        mismatch.actuator = ActuatorState::Mismatch;
        loop_.tick(mismatch);

        mismatch.group_c = None;
        mismatch.dt_s = 2.0;
        for _ in 0..30 {
            loop_.tick(mismatch);
        }
        close(loop_.thermal(), 80.0);
        close(loop_.requested().unwrap(), 100.0);

        let mut recovered = input(Some(70.0), 1.0);
        recovered.actuator = ActuatorState::Unverifiable;
        let frozen = loop_.tick(recovered);
        assert_eq!(frozen.hold, Hold::ActuatorMismatch);
        close(frozen.thermal, 80.0);
        close(frozen.shadow, 70.0);

        recovered.actuator = ActuatorState::Verified;
        let reset = loop_.tick(recovered);
        close(reset.thermal, 100.0);
        close(reset.shadow, 100.0);
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

    #[test]
    fn replacing_gains_keeps_the_live_candidates_and_pi_history() {
        let original = Gains {
            kc: 0.5,
            ti_s: 20.0,
        };
        let replacement = Gains {
            kc: 1.5,
            ti_s: 35.0,
        };
        let mut loop_ = DeviceLoop::<W>::new(original);
        loop_.seed_candidates(70.0, 62.0, Some(62.0), 4.0);
        loop_.elapsed_s = 3.0;
        loop_.set_gains(replacement);

        assert_eq!(loop_.gains, replacement);
        assert_eq!(loop_.thermal, 70.0);
        assert_eq!(loop_.shadow, 62.0);
        assert_eq!(loop_.requested, Some(62.0));
        assert_eq!(loop_.last_applied, Some(62.0));
        assert_eq!(loop_.e_prev, 4.0);
        assert_eq!(loop_.elapsed_s, 3.0);
    }

    #[test]
    fn group_recovery_resync_absorbs_simultaneous_target_motion_without_a_false_kick() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(50.0, 0.0);
        loop_.tick(input(None, 2.0));

        let mut recovered = input(Some(60.0), 1.0);
        recovered.t_star = 72.0;
        recovered.delta_tstar = 2.0;
        close(loop_.tick(recovered).thermal, 50.0);
        recovered.dt_s = 2.0;
        recovered.delta_tstar = 0.0;
        close(loop_.tick(recovered).thermal, 50.0);
        close(loop_.tick(recovered).thermal, 56.0);
    }

    #[test]
    fn mismatch_recovery_resync_absorbs_simultaneous_target_motion_without_a_false_kick() {
        let mut loop_ = DeviceLoop::<W>::new(Gains {
            kc: 1.0,
            ti_s: 10.0,
        });
        loop_.seed(50.0, 0.0);
        let mut mismatch = input(Some(70.0), 2.0);
        mismatch.actuator = ActuatorState::Mismatch;
        loop_.tick(mismatch);

        let mut recovered = input(Some(60.0), 1.0);
        recovered.t_star = 72.0;
        recovered.delta_tstar = 2.0;
        close(loop_.tick(recovered).thermal, 50.0);
        recovered.dt_s = 2.0;
        recovered.delta_tstar = 0.0;
        close(loop_.tick(recovered).thermal, 50.0);
        close(loop_.tick(recovered).thermal, 56.0);
    }

    fn first_order_cpu_response(ma_interval: u32) -> (f64, f64) {
        let ambient = 40.0;
        let target = 80.0;
        let initial_cap = 10.0;
        let mut group = ambient + 0.8 * initial_cap;
        let mut delayed = std::collections::VecDeque::from(vec![initial_cap; (W::BASE_DELAY_S + f64::from(ma_interval) / 2.0).ceil() as usize]);
        let mut loop_ = DeviceLoop::<W>::new(default_gains::<W>(ma_interval));
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
                shadow_headroom: 10.0,
                shadow_fall_rate: 0.33,
                shadow_enabled: false,
            });
            loop_.note_applied(decision.cap);
            delayed.push_back(decision.cap);
            let applied = delayed.pop_front().unwrap();
            group += (ambient + 0.8 * applied - group) / 35.0;
            peak = peak.max(group);
        }
        (group, peak)
    }

    fn first_order_gpu_response(ma_interval: u32) -> (f64, f64) {
        // Recorded GPU model from the 2026-09-12 calibration (60 s MA).
        // This replaces the provisional K=.02, tau=15, delay=90 s nominal
        // model; that older model overshoots 11.12 C with these new gains.
        let ambient = 60.0;
        let plant_k = 0.009_527_650_224_779_704;
        let plant_tau = 32.564_086_253_945_035;
        let target = 80.0;
        let initial_cap = 1_000.0;
        let mut group = ambient + plant_k * initial_cap;
        let mut delayed = std::collections::VecDeque::from(vec![initial_cap; (Mhz::BASE_DELAY_S + f64::from(ma_interval) / 2.0).ceil() as usize]);
        let mut loop_ = DeviceLoop::<Mhz>::new(default_gains::<Mhz>(ma_interval));
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
                shadow_headroom: 300.0,
                shadow_fall_rate: 10.0,
                shadow_enabled: false,
            });
            loop_.note_applied(decision.cap);
            delayed.push_back(decision.cap);
            let applied = delayed.pop_front().unwrap();
            group += (ambient + plant_k * applied - group) / plant_tau;
            peak = peak.max(group);
        }
        (group, peak)
    }

    #[test]
    fn live_imc_defaults_settle_first_order_cpu_and_gpu_within_overshoot_bar() {
        for ma_interval in [30, 60] {
        for (name, target, initial, response) in [
            ("cpu", 80.0, 48.0, first_order_cpu_response(ma_interval)),
            ("gpu", 80.0, 69.527_650_224_779_7, first_order_gpu_response(ma_interval)),
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
}

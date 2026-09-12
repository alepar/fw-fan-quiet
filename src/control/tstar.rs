//! Shared per-device temperature target source (design §2.4).
//!
//! This is deliberately pure: the controller owns socket polling, persistence writes and
//! DeviceLoop calls.  It returns target deltas so ordinary setpoint motion is never mistaken
//! for an error resync.

use std::collections::{BTreeMap, VecDeque};

use crate::control::device_loop::{Bound, Hold, ThermalMode};
use crate::fanctrl::curve::Curve;
use crate::state::TStarSeed;

pub const ENTRY_HYSTERESIS_S: f64 = 15.0;
const STUCK_WINDOW_S: f64 = 300.0;
const STUCK_SPAN_C: f64 = 0.25;
const QUARANTINE_RECOVERY_SAMPLES: u8 = 30;
const QUARANTINE_RECOVERY_DELTA_C: f64 = 0.5;
const SAVE_PERIOD_S: f64 = 60.0;
const FAN_TARGET_MIN_RPM: u32 = 1000;
const FAN_TARGET_MAX_RPM: u32 = 7000;
const ARGMAX_DEBOUNCE_SAMPLES: u8 = 3;
const DECISIVE_ARGMAX_LEAD_C: f64 = 1.0;
const FAN_SMOOTH_N: usize = 5;
const HELD_PI_PERIOD_S: f64 = 5.0;
const HELD_KC_C_PER_RPM: f64 = 2.9e-4;
const HELD_TI_S: f64 = 35.0;
const HELD_LAMBDA_S: f64 = 1440.0;
const SLOPE_REFERENCE_PCT_PER_C: f64 = 1.0;
const STEEP_SLOPE_PCT_PER_C: f64 = 2.0;
const HELD_MAX_STEP_C: f64 = 0.5;
const DEVICE_UNREACHABLE_DWELL_S: f64 = 60.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TStarState {
    Curve,
    Held,
    Uncontrollable,
    Released,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorClass {
    Controllable,
    KnownUncontrollable,
    Unknown,
}

/// The two independent thermal loops consuming this shared target.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Device {
    Cpu,
    Gpu,
}

/// The previous DeviceLoop decision consumed by Held.  `TStarSource::tick`
/// runs before the next device ticks, so this is deliberately last tick's
/// state rather than a decision made during this call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeldDeviceInput {
    pub device: Device,
    pub previous_hold: Hold,
    pub group_c: Option<f64>,
}

impl HeldDeviceInput {
    pub fn new(device: Device, previous_hold: Hold, group_c: Option<f64>) -> Self {
        Self {
            device,
            previous_hold,
            group_c,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SensorReading {
    pub label: String,
    pub value_c: f64,
    pub class: SensorClass,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TStarFlag {
    ArgmaxUncontrollable(String),
    ArgmaxStuck(String),
    EcUnknownLabel(String),
    EcUncontrollableUnavailable,
    TargetUnreachable(Bound),
    SteepCurve,
    DeviceUnreachable { device: Device, bound: Bound },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feasibility {
    pub floor_c: f64,
    pub ceiling_c: f64,
    pub uncontrollable_available: bool,
    /// The unconstrained `max(uncontrollable) + 5 C`, when known.  This is
    /// retained after clamping so callers can distinguish a collapsed high
    /// interval from an unavailable one.
    pub raw_floor_c: Option<f64>,
}

/// The one shared interval. The caller supplies only plausible EC readings; quarantined labels
/// are excluded before this helper is called.
pub fn feasibility(
    readings: impl Iterator<Item = (SensorClass, f64)>,
    cpu_hot_c: f64,
    gpu_hot_c: f64,
) -> Feasibility {
    let ceiling_c = (finite_or(cpu_hot_c, 90.0) - 2.0).min(finite_or(gpu_hot_c, 90.0) - 2.0);
    let hottest = readings
        .filter_map(|(class, value)| {
            (class == SensorClass::KnownUncontrollable && value.is_finite() && value > 0.0)
                .then_some(value)
        })
        .reduce(f64::max);
    match hottest {
        Some(value) => Feasibility {
            floor_c: (value + 5.0).min(ceiling_c),
            ceiling_c,
            uncontrollable_available: true,
            raw_floor_c: Some(value + 5.0),
        },
        None => Feasibility {
            floor_c: ceiling_c,
            ceiling_c,
            uncontrollable_available: false,
            raw_floor_c: None,
        },
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EntrySeed {
    Qualified(f64),
    Groups {
        cpu_c: Option<f64>,
        gpu_c: Option<f64>,
    },
    Fallback(f64),
}

#[derive(Debug, Clone, PartialEq)]
pub struct TStarInput {
    pub dt_s: f64,
    pub now_unix_s: u64,
    pub fan_valid: bool,
    pub ec_valid: bool,
    pub watchdog_release: bool,
    pub resumed: bool,
    pub fresh_view: bool,
    pub replica_reconciled: bool,
    pub curve_points: Option<Vec<(f64, u8)>>,
    pub snapped_duty: Option<u8>,
    pub strategy: Option<String>,
    pub requested_fan_target_rpm: u32,
    /// Current maximum fan tachometer reading.  A missing reading keeps the
    /// last raw sample as the RPM PI fallback while the caller's `fan_valid`
    /// still controls safety release.
    pub fan_rpm: Option<f64>,
    /// Last-tick decisions from each device loop, used only by Held's
    /// directional anti-windup and bound dwell diagnostics.
    pub previous_holds: Vec<HeldDeviceInput>,
    pub sensors: Vec<SensorReading>,
    pub argmax_label: Option<String>,
    /// Difference from the next hottest plausible reading.  A lead above
    /// 1 C is decisive; smaller leads use the three-sample debounce.
    pub argmax_lead_c: f64,
    pub cpu_group_c: Option<f64>,
    pub gpu_group_c: Option<f64>,
    pub cpu_hot_c: f64,
    pub gpu_hot_c: f64,
    pub auto_exit: bool,
}

impl Default for TStarInput {
    fn default() -> Self {
        Self {
            dt_s: 1.0,
            now_unix_s: 0,
            fan_valid: true,
            ec_valid: true,
            watchdog_release: false,
            resumed: false,
            fresh_view: false,
            replica_reconciled: false,
            curve_points: None,
            snapped_duty: None,
            strategy: None,
            requested_fan_target_rpm: 3000,
            fan_rpm: None,
            previous_holds: vec![],
            sensors: vec![],
            argmax_label: None,
            argmax_lead_c: 0.0,
            cpu_group_c: None,
            gpu_group_c: None,
            cpu_hot_c: 90.0,
            gpu_hot_c: 88.0,
            auto_exit: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PersistenceRequest {
    pub seed: TStarSeed,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TStarOutput {
    pub state: TStarState,
    pub t_star: Option<f64>,
    pub delta_tstar: f64,
    pub thermal_mode: ThermalMode,
    pub flags: Vec<TStarFlag>,
    pub restore_thermal: bool,
    pub reseed: bool,
    /// Device loops resynchronise their previous measured error on this
    /// explicit recovery path; ordinary T* changes use `delta_tstar` only.
    pub resync: bool,
    /// A Bypass/Regulate handoff occurred; DeviceLoop uses its existing transfer hook.
    pub mode_transfer: bool,
    pub persistence: Option<PersistenceRequest>,
    pub curve_derivations: u64,
    /// Effective current feedback used by Held's RPM PI, including the raw
    /// fallback when the smoothing window cannot be updated.
    pub held_fan_rpm: Option<f64>,
    /// Gain multiplier selected from the current curve slope.
    pub held_schedule: f64,
    /// `HELD_LAMBDA_S / held_schedule`, exported for offline verification.
    pub held_lambda_eff_s: f64,
}

#[derive(Debug, Clone)]
struct PendingStuck {
    label: String,
    samples: VecDeque<(f64, f64)>,
}

/// Session-scoped state. Construct a fresh source on each Auto entry; a quarantine persists
/// within that one session even over resume, as required by the safety backstop.
pub struct TStarSource {
    state: TStarState,
    target: Option<f64>,
    initial_seed: Option<EntrySeed>,
    qualified_seed: Option<f64>,
    gate_label: Option<String>,
    gate_elapsed_s: f64,
    argmax_label: Option<String>,
    argmax_streak: u8,
    last_reconciled: Option<bool>,
    cached_curve_key: Option<(Vec<(f64, u8)>, u8)>,
    cached_curve_target: Option<f64>,
    curve_derivations: u64,
    pending_stuck: Option<PendingStuck>,
    quarantines: BTreeMap<String, (f64, u8)>,
    control_s: f64,
    dirty: bool,
    last_save_s: Option<f64>,
    held_override: Option<f64>,
    fan_window: VecDeque<f64>,
    held_last_raw_rpm: Option<f64>,
    held_elapsed_s: f64,
    held_error_prev: Option<f64>,
    device_bound_dwell_s: BTreeMap<Device, (Bound, f64)>,
}

impl TStarSource {
    pub fn new(seed: EntrySeed) -> Self {
        let qualified_seed = match seed {
            EntrySeed::Qualified(value) => Some(value),
            _ => None,
        };
        Self {
            state: TStarState::Held,
            target: None,
            initial_seed: Some(seed),
            qualified_seed,
            gate_label: None,
            gate_elapsed_s: 0.0,
            argmax_label: None,
            argmax_streak: 0,
            last_reconciled: None,
            cached_curve_key: None,
            cached_curve_target: None,
            curve_derivations: 0,
            pending_stuck: None,
            quarantines: BTreeMap::new(),
            control_s: 0.0,
            dirty: false,
            last_save_s: None,
            held_override: None,
            fan_window: VecDeque::with_capacity(FAN_SMOOTH_N),
            held_last_raw_rpm: None,
            held_elapsed_s: 0.0,
            held_error_prev: None,
            device_bound_dwell_s: BTreeMap::new(),
        }
    }

    /// Reserved for Task 8's Held RPM PI. It produces a normal delta and never a resync.
    #[cfg(test)]
    pub fn set_held_target(&mut self, target_c: f64) {
        self.held_override = target_c.is_finite().then_some(target_c);
    }
    #[cfg(test)]
    pub fn state(&self) -> TStarState {
        self.state
    }
    #[cfg(test)]
    pub fn quarantined(&self, label: &str) -> bool {
        self.quarantines.contains_key(label)
    }

    pub fn tick(&mut self, input: &TStarInput) -> TStarOutput {
        let held_dt = held_control_dt(input.dt_s);
        let held_wall_gap = held_dt == 0.0 && input.dt_s.is_finite() && input.dt_s > 7.0;
        // A wall gap is observation time, not control time. The fresh sample
        // may establish the first post-gap label observation, but none of
        // the gap may advance a dwell, PI cadence, or stuck window.
        let dt = if held_wall_gap { 0.0 } else { sane_dt(input.dt_s) };
        self.control_s += dt;
        let previous = self.target;
        let state_before = self.state;
        let mut held_fan_rpm = None;
        let continuity_gap = input.resumed || held_wall_gap;
        if continuity_gap {
            for (_, recovery_streak) in self.quarantines.values_mut() {
                *recovery_streak = 0;
            }
        }
        let quarantine_recovered =
            !continuity_gap && self.update_quarantine_recovery(input);
        let reconciliation_recovered =
            self.last_reconciled == Some(false) && input.replica_reconciled;
        self.last_reconciled = Some(input.replica_reconciled);
        if continuity_gap {
            self.pending_stuck = None;
            self.gate_label = None;
            self.gate_elapsed_s = 0.0;
            self.argmax_label = None;
            self.argmax_streak = 0;
            self.reset_held_state();
        }
        let usable: Vec<&SensorReading> = input
            .sensors
            .iter()
            .filter(|r| {
                !self.quarantines.contains_key(&r.label) && r.value_c.is_finite() && r.value_c > 0.0
            })
            .collect();
        let mut bounds = feasibility(
            usable.iter().map(|r| (r.class, r.value_c)),
            input.cpu_hot_c,
            input.gpu_hot_c,
        );
        let mut flags = Vec::new();
        for label in self.quarantines.keys() {
            flags.push(TStarFlag::ArgmaxStuck(label.clone()));
        }
        let mut reseed = false;
        let mut resync = false;
        let mut restore_thermal = false;
        let mut held_upper_saturation = false;

        if !input.fan_valid || !input.ec_valid || input.watchdog_release || input.auto_exit {
            self.reset_held_state();
            self.state = TStarState::Released;
            self.pending_stuck = None;
            self.gate_label = None;
            self.gate_elapsed_s = 0.0;
        } else {
            let reengaged = state_before == TStarState::Released;
            if reengaged {
                self.reset_held_state();
            }
            held_fan_rpm = self.observe_fan_rpm(input.fan_rpm);
            if self.target.is_none() || reengaged {
                self.target = Some(if reengaged {
                    self.reengagement_target(input, bounds)
                } else {
                    self.entry_target(input, bounds)
                });
                self.dirty = true;
                reseed = true;
                resync = true;
            }
            if reengaged || quarantine_recovered || reconciliation_recovered {
                self.state = TStarState::Held;
                self.pending_stuck = None;
                self.gate_label = None;
                self.gate_elapsed_s = 0.0;
                self.argmax_label = None;
                self.argmax_streak = 0;
                reseed = true;
                resync = true;
            }
            let argmax = input
                .argmax_label
                .as_deref()
                .and_then(|label| input.sensors.iter().find(|r| r.label == label));
            let class = argmax.map(|r| r.class);
            let argmax_quarantined = input
                .argmax_label
                .as_ref()
                .is_some_and(|label| self.quarantines.contains_key(label));
            if let Some(r) = argmax.filter(|r| r.class == SensorClass::Unknown) {
                flags.push(TStarFlag::EcUnknownLabel(r.label.clone()));
            }
            let debounced =
                self.advance_argmax_debounce(argmax, input.fresh_view, input.argmax_lead_c);
            let before_key = self.cached_curve_key.clone();
            let curve_target = self.curve_target(input, bounds);
            let view_usable =
                input.fresh_view && input.replica_reconciled && curve_target.is_some();
            // This timer belongs exclusively to a Held -> Curve entry.  Bypass
            // routing has its own argmax debounce and must never wait for it.
            let curve_entry_eligible = view_usable
                && class == Some(SensorClass::Controllable)
                && debounced
                && !argmax_quarantined
                && self.quarantines.is_empty();
            self.advance_gate(
                argmax,
                self.state == TStarState::Held && curve_entry_eligible,
                dt,
            );
            self.observe_stuck(argmax, input, debounced);
            if let Some(label) = self.new_stuck_label() {
                let value = self.value_of(input, &label).unwrap_or_default();
                self.quarantines.insert(label.clone(), (value, 0));
                flags.push(TStarFlag::ArgmaxStuck(label));
                self.pending_stuck = None;
                self.state = TStarState::Held;
                let usable: Vec<&SensorReading> = input
                    .sensors
                    .iter()
                    .filter(|r| {
                        !self.quarantines.contains_key(&r.label)
                            && r.value_c.is_finite()
                            && r.value_c > 0.0
                    })
                    .collect();
                bounds = feasibility(
                    usable.iter().map(|r| (r.class, r.value_c)),
                    input.cpu_hot_c,
                    input.gpu_hot_c,
                );
                self.target = Some(self.entry_target(input, bounds));
                self.dirty = true;
                reseed = true;
                resync = true;
            }
            let quarantined = !self.quarantines.is_empty();
            let controllable_argmax = view_usable
                && class == Some(SensorClass::Controllable)
                && debounced
                && !argmax_quarantined
                && !quarantined;
            let controllable_view = view_usable
                && class == Some(SensorClass::Controllable)
                && !argmax_quarantined
                && !quarantined;
            let uncontrollable_argmax = view_usable
                && class == Some(SensorClass::KnownUncontrollable)
                && debounced
                && !argmax_quarantined
                && !quarantined;
            let uncontrollable_view = view_usable
                && class == Some(SensorClass::KnownUncontrollable)
                && !argmax_quarantined
                && !quarantined;
            let held_to_curve = self.state == TStarState::Held
                && controllable_argmax
                && self.gate_elapsed_s >= ENTRY_HYSTERESIS_S;
            if reengaged
                || quarantine_recovered
                || reconciliation_recovered
                || quarantined
                || argmax_quarantined
                || class == Some(SensorClass::Unknown)
            {
                self.state = TStarState::Held;
            } else {
                match self.state {
                    TStarState::Held if uncontrollable_argmax => {
                        self.state = TStarState::Uncontrollable;
                    }
                    TStarState::Held if held_to_curve => {
                        self.enter_curve(
                            curve_target.expect("eligible curve"),
                            bounds,
                            before_key,
                            &mut restore_thermal,
                        );
                    }
                    TStarState::Curve if controllable_view => {
                        // Curve remains active while its qualified controllable
                        // argmax persists; curve changes still update T*.
                        self.enter_curve(
                            curve_target.expect("usable curve"),
                            bounds,
                            before_key,
                            &mut restore_thermal,
                        );
                    }
                    TStarState::Curve if uncontrollable_view => {
                        if uncontrollable_argmax {
                            self.state = TStarState::Uncontrollable;
                        }
                    }
                    TStarState::Uncontrollable if uncontrollable_view => {
                        // A qualified Bypass input is stable indefinitely.  It
                        // never reuses the Held -> Curve entry timer.
                    }
                    TStarState::Uncontrollable if controllable_view => {
                        // Leaving Bypass is a mode transfer to Held.  The next
                        // continuous Held interval must earn Curve entry anew.
                        if controllable_argmax {
                            self.state = TStarState::Held;
                            self.gate_label = None;
                            self.gate_elapsed_s = 0.0;
                        }
                    }
                    _ => {
                        self.state = TStarState::Held;
                        self.clamp_target(bounds);
                    }
                }
            }
            let (held_schedule, steep_curve) = self.held_schedule(input);
            if steep_curve {
                flags.push(TStarFlag::SteepCurve);
            }
            if self.state == TStarState::Held && !reengaged {
                if let Some(held) = self.held_override.take() {
                    self.set_target(held, bounds);
                    self.held_elapsed_s = 0.0;
                    self.held_error_prev = None;
                } else {
                    held_upper_saturation =
                        self.drive_held(input, bounds, held_fan_rpm, held_schedule, held_dt);
                }
            } else {
                self.held_elapsed_s = 0.0;
                self.held_error_prev = None;
            }
            if self.state == TStarState::Uncontrollable {
                if let Some(r) = argmax {
                    flags.push(TStarFlag::ArgmaxUncontrollable(r.label.clone()));
                }
            }
            if reengaged || quarantine_recovered || reconciliation_recovered {
                // The recovery sample itself establishes no eligibility; the
                // next fresh sample begins both debounce and the 15 s gate.
                self.gate_label = None;
                self.gate_elapsed_s = 0.0;
                self.argmax_label = None;
                self.argmax_streak = 0;
            }

            if let Some(target) = self.target {
                flags.extend(self.device_unreachable_flags(input, target, held_dt));
            }
        }
        let mut feasibility_flags = self.feasibility_flags(input, bounds);
        if held_upper_saturation
            && !feasibility_flags.contains(&TStarFlag::TargetUnreachable(Bound::Max))
        {
            feasibility_flags.push(TStarFlag::TargetUnreachable(Bound::Max));
        }
        flags.extend(feasibility_flags);
        let persistence = self.persistence_request(input, bounds, &flags, state_before);
        let target = self.target;
        let (held_schedule, _) = self.held_schedule(input);
        TStarOutput {
            state: self.state,
            t_star: target,
            delta_tstar: match (previous, target) {
                (Some(old), Some(new)) => new - old,
                _ => 0.0,
            },
            thermal_mode: if self.state == TStarState::Uncontrollable {
                ThermalMode::Bypass
            } else {
                ThermalMode::Regulate
            },
            flags,
            restore_thermal,
            reseed,
            resync,
            mode_transfer: (state_before == TStarState::Uncontrollable)
                != (self.state == TStarState::Uncontrollable),
            persistence,
            curve_derivations: self.curve_derivations,
            held_fan_rpm,
            held_schedule,
            held_lambda_eff_s: HELD_LAMBDA_S / held_schedule,
        }
    }

    fn observe_fan_rpm(&mut self, fan_rpm: Option<f64>) -> Option<f64> {
        let current = fan_rpm.filter(|rpm| rpm.is_finite() && *rpm >= 0.0);
        if let Some(raw) = current {
            self.held_last_raw_rpm = Some(raw);
            self.fan_window.push_back(raw);
            if self.fan_window.len() > FAN_SMOOTH_N {
                self.fan_window.pop_front();
            }
        }
        if current.is_some() && self.fan_window.len() == FAN_SMOOTH_N {
            Some(self.fan_window.iter().sum::<f64>() / FAN_SMOOTH_N as f64)
        } else {
            self.held_last_raw_rpm
        }
    }

    fn reset_held_state(&mut self) {
        self.fan_window.clear();
        self.held_last_raw_rpm = None;
        self.held_elapsed_s = 0.0;
        self.held_error_prev = None;
        self.device_bound_dwell_s.clear();
    }

    fn feasibility_flags(&self, input: &TStarInput, bounds: Feasibility) -> Vec<TStarFlag> {
        let mut flags = Vec::new();
        if !bounds.uncontrollable_available {
            flags.push(TStarFlag::EcUncontrollableUnavailable);
        }
        if bounds.raw_floor_c.is_some_and(|raw| raw > bounds.ceiling_c)
            || self
                .raw_curve_target(input)
                .is_some_and(|target| target > bounds.ceiling_c)
        {
            flags.push(TStarFlag::TargetUnreachable(Bound::Max));
        }
        if self.curve_target_is_below_floor(input) {
            flags.push(TStarFlag::TargetUnreachable(Bound::Floor));
        }
        flags
    }

    /// Gain scheduling is based on the target's local curve slope.  A
    /// syntactically present but unresolvable curve is intentionally the
    /// conservative .25x case rather than an assumed flat segment.
    fn held_schedule(&self, input: &TStarInput) -> (f64, bool) {
        let Some(target) = self.target else {
            return (0.25, false);
        };
        let Some(points) = input.curve_points.clone() else {
            return (0.25, false);
        };
        let Ok(curve) = Curve::from_points(points) else {
            return (0.25, false);
        };
        let slope = curve.slope_at(target);
        if !slope.is_finite() {
            return (0.25, true);
        }
        let schedule =
            (SLOPE_REFERENCE_PCT_PER_C / slope.max(SLOPE_REFERENCE_PCT_PER_C)).clamp(0.25, 1.0);
        (schedule, slope > STEEP_SLOPE_PCT_PER_C)
    }

    fn curve_target_is_below_floor(&self, input: &TStarInput) -> bool {
        let (Some(points), Some(duty)) = (input.curve_points.clone(), input.snapped_duty) else {
            return false;
        };
        Curve::from_points(points)
            .ok()
            .is_some_and(|curve| curve.nearest_tread(duty).is_none())
    }

    fn raw_curve_target(&self, input: &TStarInput) -> Option<f64> {
        let (Some(points), Some(duty)) = (input.curve_points.clone(), input.snapped_duty) else {
            return None;
        };
        let curve = Curve::from_points(points).ok()?;
        curve.t_star(curve.nearest_tread(duty)?)
    }

    fn drive_held(
        &mut self,
        input: &TStarInput,
        bounds: Feasibility,
        fan_rpm: Option<f64>,
        schedule: f64,
        dt: f64,
    ) -> bool {
        let Some(fan_rpm) = fan_rpm else {
            return false;
        };
        if input.resumed || dt == 0.0 {
            return false;
        }
        self.held_elapsed_s += dt;
        if self.held_elapsed_s < HELD_PI_PERIOD_S {
            return false;
        }
        let elapsed = self.held_elapsed_s.min(7.0);
        self.held_elapsed_s = 0.0;
        let error = f64::from(input.requested_fan_target_rpm) - fan_rpm;
        let previous_error = self.held_error_prev.unwrap_or(error);
        let gain = HELD_KC_C_PER_RPM * schedule;
        let proportional = gain * (error - previous_error);
        let integral = gain * elapsed / HELD_TI_S * error;
        // Always advance the error history, including when an actuator or
        // bound makes this direction unavailable.  Releasing that hold then
        // cannot turn accumulated RPM error into a proportional kick.
        self.held_error_prev = Some(error);
        let Some(current) = self.target else {
            return false;
        };
        let at_floor = current <= bounds.floor_c;
        let at_ceiling = current >= bounds.ceiling_c;
        let integral_blocked = (integral > 0.0
            && (all_block_upward(&input.previous_holds) || at_ceiling))
            || (integral < 0.0 && (all_block_downward(&input.previous_holds) || at_floor));
        // Preserve the unsuppressed PI candidate for feasibility reporting.
        // At a ceiling, a steady positive error has no P term and its I term
        // is intentionally withheld from the command, but it still means the
        // current shared ceiling cannot satisfy the requested fan RPM.
        let unsuppressed_requested =
            current + (proportional + integral).clamp(-HELD_MAX_STEP_C, HELD_MAX_STEP_C);
        let attempted_upper_saturation = error > 0.0 && unsuppressed_requested > bounds.ceiling_c;
        let permitted_integral = if integral_blocked { 0.0 } else { integral };
        let requested =
            current + (proportional + permitted_integral).clamp(-HELD_MAX_STEP_C, HELD_MAX_STEP_C);
        self.set_target(requested, bounds);
        attempted_upper_saturation
    }

    fn device_unreachable_flags(
        &mut self,
        input: &TStarInput,
        target: f64,
        dt: f64,
    ) -> Vec<TStarFlag> {
        if input.resumed || dt == 0.0 {
            self.device_bound_dwell_s.clear();
            return vec![];
        }
        self.device_bound_dwell_s.retain(|device, _| {
            input
                .previous_holds
                .iter()
                .any(|input| input.device == *device)
        });
        let mut flags = Vec::new();
        for device in &input.previous_holds {
            let candidate = match (device.previous_hold, device.group_c) {
                (Hold::Clamp(Bound::Max), Some(group)) if group.is_finite() && group < target => {
                    Some(Bound::Max)
                }
                (Hold::Clamp(Bound::Floor), Some(group)) if group.is_finite() && group > target => {
                    Some(Bound::Floor)
                }
                _ => None,
            };
            match candidate {
                Some(bound) => {
                    let entry = self
                        .device_bound_dwell_s
                        .entry(device.device)
                        .or_insert((bound, 0.0));
                    if entry.0 != bound {
                        *entry = (bound, 0.0);
                    }
                    entry.1 += dt;
                    if entry.1 >= DEVICE_UNREACHABLE_DWELL_S {
                        flags.push(TStarFlag::DeviceUnreachable {
                            device: device.device,
                            bound,
                        });
                    }
                }
                None => {
                    self.device_bound_dwell_s.remove(&device.device);
                }
            }
        }
        flags
    }

    fn entry_target(&mut self, input: &TStarInput, bounds: Feasibility) -> f64 {
        let raw = match self.initial_seed.take() {
            Some(EntrySeed::Qualified(v)) | Some(EntrySeed::Fallback(v)) => v,
            Some(EntrySeed::Groups { cpu_c, gpu_c }) => cpu_c
                .into_iter()
                .chain(gpu_c)
                .reduce(f64::max)
                .unwrap_or(bounds.ceiling_c),
            None => input
                .cpu_group_c
                .into_iter()
                .chain(input.gpu_group_c)
                .reduce(f64::max)
                .unwrap_or(bounds.ceiling_c),
        };
        raw.clamp(bounds.floor_c, bounds.ceiling_c)
    }
    fn reengagement_target(&self, input: &TStarInput, bounds: Feasibility) -> f64 {
        self.qualified_seed
            .unwrap_or_else(|| {
                input
                    .cpu_group_c
                    .into_iter()
                    .chain(input.gpu_group_c)
                    .reduce(f64::max)
                    .unwrap_or(bounds.ceiling_c)
            })
            .clamp(bounds.floor_c, bounds.ceiling_c)
    }
    fn set_target(&mut self, raw: f64, bounds: Feasibility) {
        let next = raw.clamp(bounds.floor_c, bounds.ceiling_c);
        if self.target != Some(next) {
            self.target = Some(next);
            self.dirty = true;
        }
    }
    fn clamp_target(&mut self, bounds: Feasibility) {
        if let Some(target) = self.target {
            self.set_target(target, bounds);
        }
    }
    fn advance_argmax_debounce(
        &mut self,
        argmax: Option<&SensorReading>,
        fresh: bool,
        lead_c: f64,
    ) -> bool {
        let Some(r) = argmax.filter(|r| {
            fresh
                && matches!(
                    r.class,
                    SensorClass::Controllable | SensorClass::KnownUncontrollable
                )
        }) else {
            self.argmax_label = None;
            self.argmax_streak = 0;
            return false;
        };
        if self.argmax_label.as_deref() == Some(r.label.as_str()) {
            self.argmax_streak = self.argmax_streak.saturating_add(1);
        } else {
            self.argmax_label = Some(r.label.clone());
            self.argmax_streak = 1;
        }
        lead_c.is_finite() && lead_c > DECISIVE_ARGMAX_LEAD_C
            || self.argmax_streak >= ARGMAX_DEBOUNCE_SAMPLES
    }

    fn advance_gate(&mut self, argmax: Option<&SensorReading>, eligible: bool, dt: f64) {
        let Some(r) = argmax.filter(|_| eligible) else {
            self.gate_label = None;
            self.gate_elapsed_s = 0.0;
            return;
        };
        if self.gate_label.as_deref() == Some(r.label.as_str()) {
            self.gate_elapsed_s += dt;
        } else {
            self.gate_label = Some(r.label.clone());
            self.gate_elapsed_s = dt;
        }
    }
    fn curve_target(&mut self, input: &TStarInput, bounds: Feasibility) -> Option<f64> {
        let points = input.curve_points.as_ref()?;
        let duty = input.snapped_duty?;
        let key = (points.clone(), duty);
        if self.cached_curve_key.as_ref() != Some(&key) {
            let curve = Curve::from_points(points.clone()).ok()?;
            let resolved = curve.nearest_tread(duty)?;
            self.cached_curve_target = curve.t_star(resolved);
            self.cached_curve_key = Some(key);
            self.curve_derivations += 1;
        }
        self.cached_curve_target
            .map(|value| value.clamp(bounds.floor_c, bounds.ceiling_c))
    }
    fn enter_curve(
        &mut self,
        next: f64,
        bounds: Feasibility,
        before_key: Option<(Vec<(f64, u8)>, u8)>,
        restore_thermal: &mut bool,
    ) {
        self.state = TStarState::Curve;
        *restore_thermal =
            self.target.is_some_and(|old| next > old) && self.cached_curve_key != before_key;
        self.set_target(next, bounds);
    }
    fn observe_stuck(
        &mut self,
        argmax: Option<&SensorReading>,
        input: &TStarInput,
        debounced: bool,
    ) {
        let Some(r) = argmax.filter(|r| {
            input.fresh_view
                && r.class == SensorClass::KnownUncontrollable
                && debounced
                && !self.quarantines.contains_key(&r.label)
        }) else {
            self.pending_stuck = None;
            return;
        };
        if self
            .pending_stuck
            .as_ref()
            .is_none_or(|p| p.label != r.label)
        {
            self.pending_stuck = Some(PendingStuck {
                label: r.label.clone(),
                samples: VecDeque::new(),
            });
        }
        let p = self.pending_stuck.as_mut().expect("made above");
        p.samples.push_back((self.control_s, r.value_c));
        while p
            .samples
            .front()
            .is_some_and(|(t, _)| self.control_s - t > STUCK_WINDOW_S)
        {
            p.samples.pop_front();
        }
    }
    fn new_stuck_label(&self) -> Option<String> {
        let p = self.pending_stuck.as_ref()?;
        let (first, _) = *p.samples.front()?;
        if self.control_s - first < STUCK_WINDOW_S {
            return None;
        }
        let (min, max) = p
            .samples
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), (_, v)| {
                (lo.min(*v), hi.max(*v))
            });
        ((max - min) <= STUCK_SPAN_C).then(|| p.label.clone())
    }
    fn update_quarantine_recovery(&mut self, input: &TStarInput) -> bool {
        let mut recovered_any = false;
        let labels: Vec<_> = self.quarantines.keys().cloned().collect();
        for label in labels {
            let (at, streak) = self.quarantines[&label];
            let recovered = input.fresh_view
                && !input.resumed
                && self
                    .value_of(input, &label)
                    .is_some_and(|v| (v - at).abs() > QUARANTINE_RECOVERY_DELTA_C);
            let next = if recovered {
                streak.saturating_add(1)
            } else {
                0
            };
            if next >= QUARANTINE_RECOVERY_SAMPLES {
                self.quarantines.remove(&label);
                recovered_any = true;
            } else if let Some(entry) = self.quarantines.get_mut(&label) {
                entry.1 = next;
            }
        }
        recovered_any
    }
    fn value_of(&self, input: &TStarInput, label: &str) -> Option<f64> {
        input
            .sensors
            .iter()
            .find(|r| r.label == label)
            .and_then(|r| (r.value_c.is_finite() && r.value_c > 0.0).then_some(r.value_c))
    }
    fn persistence_request(
        &mut self,
        input: &TStarInput,
        bounds: Feasibility,
        flags: &[TStarFlag],
        state_before: TStarState,
    ) -> Option<PersistenceRequest> {
        let controlled_now = matches!(self.state, TStarState::Held | TStarState::Curve);
        let controlled_before_exit =
            input.auto_exit && matches!(state_before, TStarState::Held | TStarState::Curve);
        let valid = (controlled_now || controlled_before_exit)
            && self.target.is_some()
            && input.strategy.as_ref().is_some_and(|s| !s.is_empty())
            && (input.cpu_group_c.is_some() || input.gpu_group_c.is_some())
            && self.quarantines.is_empty()
            && bounds.uncontrollable_available
            && !input
                .sensors
                .iter()
                .any(|reading| reading.class == SensorClass::Unknown)
            && !flags
                .iter()
                .any(|f| matches!(f, TStarFlag::EcUnknownLabel(_)));
        let held_exit = state_before == TStarState::Held && self.state == TStarState::Curve;
        let due = self
            .last_save_s
            .is_none_or(|last| self.control_s - last >= SAVE_PERIOD_S);
        if !valid || !(input.auto_exit || held_exit || (self.dirty && due)) {
            return None;
        }
        let seed = TStarSeed {
            strategy: input.strategy.clone().expect("valid"),
            fan_target_rpm: input
                .requested_fan_target_rpm
                .clamp(FAN_TARGET_MIN_RPM, FAN_TARGET_MAX_RPM),
            value_c: self.target.expect("valid"),
            saved_at_unix_s: input.now_unix_s,
        };
        self.last_save_s = Some(self.control_s);
        self.dirty = false;
        Some(PersistenceRequest { seed })
    }
}
fn sane_dt(dt: f64) -> f64 {
    if dt.is_finite() && dt > 0.0 {
        dt.min(60.0)
    } else {
        0.0
    }
}

/// Held is driven from regular 1 Hz samples.  A suspend/wall-clock jump is
/// not valid control time: it starts both the PI cadence and bound dwell over.
fn held_control_dt(dt: f64) -> f64 {
    if dt.is_finite() && dt > 0.0 && dt <= 7.0 {
        dt
    } else {
        0.0
    }
}

fn blocks_up(hold: Hold) -> bool {
    match hold {
        Hold::Clamp(Bound::Max)
        | Hold::Shadow
        | Hold::Bypass
        | Hold::ActuatorMismatch
        | Hold::GroupUnavailable => true,
        Hold::None | Hold::Clamp(Bound::Floor) | Hold::DrawUnavailable => false,
    }
}

fn blocks_down(hold: Hold) -> bool {
    match hold {
        Hold::Clamp(Bound::Floor)
        | Hold::Bypass
        | Hold::ActuatorMismatch
        | Hold::GroupUnavailable => true,
        Hold::None | Hold::Clamp(Bound::Max) | Hold::Shadow | Hold::DrawUnavailable => false,
    }
}

fn all_block_upward(devices: &[HeldDeviceInput]) -> bool {
    !devices.is_empty() && devices.iter().all(|device| blocks_up(device.previous_hold))
}

fn all_block_downward(devices: &[HeldDeviceInput]) -> bool {
    !devices.is_empty()
        && devices
            .iter()
            .all(|device| blocks_down(device.previous_hold))
}

fn finite_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        fallback
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::device_loop::{Bound, Hold};
    fn ambient(v: f64) -> SensorReading {
        SensorReading {
            label: "ambient_f75303@4d".into(),
            value_c: v,
            class: SensorClass::KnownUncontrollable,
        }
    }
    fn cpu(v: f64) -> SensorReading {
        SensorReading {
            label: "cpu@4c".into(),
            value_c: v,
            class: SensorClass::Controllable,
        }
    }
    fn curve() -> Vec<(f64, u8)> {
        vec![
            (0.0, 15),
            (55.0, 15),
            (65.0, 21),
            (75.0, 31),
            (82.0, 37),
            (88.0, 55),
            (95.0, 100),
        ]
    }
    fn input() -> TStarInput {
        TStarInput {
            fresh_view: true,
            replica_reconciled: true,
            curve_points: Some(curve()),
            snapped_duty: Some(31),
            strategy: Some("quiet16".into()),
            sensors: vec![ambient(25.0), cpu(70.0)],
            argmax_label: Some("cpu@4c".into()),
            cpu_group_c: Some(70.0),
            gpu_group_c: Some(60.0),
            ..TStarInput::default()
        }
    }
    fn to_curve(s: &mut TStarSource, i: &TStarInput) -> TStarOutput {
        let mut out = s.tick(i);
        for _ in 1..17 {
            out = s.tick(i);
        }
        out
    }

    #[test]
    fn held_rpm_pi_uses_five_sample_tail_mean_and_raw_fallback_at_elapsed_cadence() {
        let mut i = input();
        i.fresh_view = false;
        i.fan_rpm = Some(2_000.0);
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        source.tick(&i);
        let mut out = None;
        for rpm in [2_100.0, 2_200.0, 2_300.0, 2_400.0] {
            i.fan_rpm = Some(rpm);
            out = Some(source.tick(&i));
        }
        let out = out.expect("five samples");
        let before = out.t_star.expect("seeded target");
        assert!(out
            .held_fan_rpm
            .is_some_and(|rpm| (rpm - 2_200.0).abs() < 1e-9));
        assert!(out.delta_tstar > 0.0 && out.delta_tstar <= 0.5);

        i.fan_rpm = None;
        i.dt_s = 5.0;
        let fallback = source.tick(&i);
        assert_eq!(fallback.held_fan_rpm, Some(2_400.0));
        assert!(fallback.t_star.expect("target").is_finite());
        assert!((fallback.t_star.expect("target") - before).abs() <= 0.5);
    }

    #[test]
    fn held_schedule_feasibility_and_flags_are_exported() {
        let mut i = input();
        i.fresh_view = false;
        i.dt_s = 5.0;
        i.fan_rpm = Some(2_000.0);
        i.cpu_hot_c = 82.0;
        i.gpu_hot_c = 88.0;
        i.curve_points = None;
        i.sensors = vec![ambient(90.0), cpu(70.0)];
        let out = TStarSource::new(EntrySeed::Fallback(70.0)).tick(&i);
        assert_eq!(out.t_star, Some(80.0));
        assert!(out
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Max)));
        assert_eq!(out.held_schedule, 0.25);
        assert_eq!(out.held_lambda_eff_s, 5760.0);

        let mut empty = i;
        empty.sensors = vec![cpu(70.0)];
        let out = TStarSource::new(EntrySeed::Fallback(70.0)).tick(&empty);
        assert!(out.flags.contains(&TStarFlag::EcUncontrollableUnavailable));
        assert_eq!(out.t_star, Some(80.0));
    }

    #[test]
    fn held_schedule_is_one_at_the_reference_slope_and_quarter_without_a_curve() {
        let mut reference = input();
        reference.fresh_view = false;
        let out = TStarSource::new(EntrySeed::Fallback(70.0)).tick(&reference);
        assert_eq!(out.held_schedule, 1.0);
        assert_eq!(out.held_lambda_eff_s, HELD_LAMBDA_S);

        reference.curve_points = None;
        let no_curve = TStarSource::new(EntrySeed::Fallback(70.0)).tick(&reference);
        assert_eq!(no_curve.held_schedule, 0.25);
        assert_eq!(no_curve.held_lambda_eff_s, HELD_LAMBDA_S / 0.25);
    }

    #[test]
    fn held_pi_uses_velocity_gains_and_caps_accumulated_elapsed_at_seven_seconds() {
        let mut i = input();
        i.fresh_view = false;
        i.curve_points = None;
        i.fan_rpm = Some(2_000.0);
        i.dt_s = 4.0;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        assert_eq!(source.tick(&i).delta_tstar, 0.0);
        let out = source.tick(&i);
        let expected = HELD_KC_C_PER_RPM * 0.25 * 7.0 / HELD_TI_S * 1_000.0;
        assert!((out.delta_tstar - expected).abs() < 1e-12, "{out:?}");
        assert!(out.delta_tstar < HELD_MAX_STEP_C);
    }

    #[test]
    fn feasibility_distinguishes_curve_high_from_raw_high_and_curve_low() {
        let mut high_curve = input();
        high_curve.cpu_hot_c = 82.0;
        high_curve.gpu_hot_c = 88.0;
        high_curve.curve_points = Some(vec![(60.0, 20), (100.0, 60)]);
        high_curve.snapped_duty = Some(50);
        high_curve.sensors = vec![ambient(25.0), cpu(70.0)];
        let curve_high = TStarSource::new(EntrySeed::Fallback(70.0)).tick(&high_curve);
        assert!(curve_high
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Max)));
        assert_eq!(curve_high.t_star, Some(70.0));

        let mut raw_high = high_curve.clone();
        raw_high.curve_points = None;
        raw_high.sensors = vec![ambient(90.0), cpu(70.0)];
        let raw_high = TStarSource::new(EntrySeed::Fallback(70.0)).tick(&raw_high);
        assert_eq!(raw_high.t_star, Some(80.0));
        assert!(raw_high
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Max)));

        let mut low_curve = input();
        low_curve.curve_points = Some(vec![(60.0, 20), (80.0, 40)]);
        low_curve.snapped_duty = Some(10);
        let low = TStarSource::new(EntrySeed::Fallback(70.0)).tick(&low_curve);
        assert!(low
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Floor)));
    }

    #[test]
    fn released_interruption_resets_held_pi_and_device_dwell_before_reentry() {
        let mut i = input();
        i.fresh_view = false;
        i.dt_s = 5.0;
        i.fan_rpm = Some(3_000.0);
        i.previous_holds = vec![HeldDeviceInput::new(
            Device::Gpu,
            Hold::Clamp(Bound::Floor),
            Some(90.0),
        )];
        let mut source = TStarSource::new(EntrySeed::Qualified(80.0));
        for _ in 0..11 {
            assert!(!source
                .tick(&i)
                .flags
                .contains(&TStarFlag::DeviceUnreachable {
                    device: Device::Gpu,
                    bound: Bound::Floor,
                }));
        }
        i.fan_valid = false;
        let released = source.tick(&i);
        assert_eq!(released.state, TStarState::Released);
        assert_eq!(released.held_fan_rpm, None);

        i.fan_valid = true;
        i.fan_rpm = Some(4_000.0);
        let reentered = source.tick(&i);
        assert_eq!(reentered.state, TStarState::Held);
        assert_eq!(reentered.delta_tstar, 0.0);
        assert!(reentered.resync);
        assert!(!reentered.flags.contains(&TStarFlag::DeviceUnreachable {
            device: Device::Gpu,
            bound: Bound::Floor,
        }));
        assert_eq!(reentered.held_fan_rpm, Some(4_000.0));
    }

    #[test]
    fn quarantine_recomputes_and_replaces_stale_feasibility_flags_on_that_tick() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 2.0;
        i.cpu_hot_c = 90.0;
        i.gpu_hot_c = 88.0;
        i.sensors = vec![ambient(90.0), cpu(70.0)];
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut out = source.tick(&i);
        for _ in 0..400 {
            out = source.tick(&i);
            if source.quarantined("ambient_f75303@4d") {
                break;
            }
        }
        assert!(source.quarantined("ambient_f75303@4d"));
        assert!(out.flags.contains(&TStarFlag::EcUncontrollableUnavailable));
        assert!(!out
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Max)));
    }

    #[test]
    fn held_directional_antiwindup_reads_previous_holds_exhaustively() {
        let mut i = input();
        i.fresh_view = false;
        i.dt_s = 5.0;
        i.fan_rpm = Some(2_000.0);
        i.previous_holds = vec![
            HeldDeviceInput::new(Device::Cpu, Hold::DrawUnavailable, Some(70.0)),
            HeldDeviceInput::new(Device::Gpu, Hold::Shadow, Some(70.0)),
        ];
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        source.tick(&i); // seed/reseed has no externally reported delta
        let permits_up = source.tick(&i);
        assert!(permits_up.delta_tstar > 0.0);

        i.previous_holds = vec![
            HeldDeviceInput::new(Device::Cpu, Hold::Clamp(Bound::Max), Some(70.0)),
            HeldDeviceInput::new(Device::Gpu, Hold::Shadow, Some(70.0)),
        ];
        let blocked_up = source.tick(&i);
        assert_eq!(blocked_up.delta_tstar, 0.0);

        i.fan_rpm = Some(5_000.0);
        i.previous_holds = vec![
            HeldDeviceInput::new(Device::Cpu, Hold::Clamp(Bound::Floor), Some(70.0)),
            HeldDeviceInput::new(Device::Gpu, Hold::Bypass, Some(70.0)),
        ];
        let blocked_down = source.tick(&i);
        assert!(blocked_down.delta_tstar < 0.0);
    }

    #[test]
    fn held_blocked_directions_suppress_only_integral_not_proportional_motion() {
        let mut i = input();
        i.fresh_view = false;
        i.dt_s = 5.0;
        i.fan_rpm = Some(3_000.0);
        i.previous_holds = vec![
            HeldDeviceInput::new(Device::Cpu, Hold::Clamp(Bound::Max), Some(70.0)),
            HeldDeviceInput::new(Device::Gpu, Hold::Shadow, Some(70.0)),
        ];
        let mut upward = TStarSource::new(EntrySeed::Fallback(70.0));
        upward.tick(&i); // establish e_prev = 0
        i.fan_rpm = Some(2_000.0);
        let up = upward.tick(&i);
        assert!(
            (up.delta_tstar - HELD_KC_C_PER_RPM * 1_000.0).abs() < 1e-12,
            "{up:?}"
        );

        i.fan_rpm = Some(3_000.0);
        i.previous_holds = vec![
            HeldDeviceInput::new(Device::Cpu, Hold::Clamp(Bound::Floor), Some(70.0)),
            HeldDeviceInput::new(Device::Gpu, Hold::Bypass, Some(70.0)),
        ];
        let mut downward = TStarSource::new(EntrySeed::Fallback(70.0));
        downward.tick(&i); // establish e_prev = 0
        i.fan_rpm = Some(4_000.0);
        let down = downward.tick(&i);
        assert!(
            (down.delta_tstar + HELD_KC_C_PER_RPM * 1_000.0).abs() < 1e-12,
            "{down:?}"
        );
    }

    #[test]
    fn held_positive_error_attempting_the_ceiling_emits_target_unreachable_high() {
        let mut i = input();
        i.fresh_view = false;
        i.curve_points = None;
        i.dt_s = 5.0;
        i.fan_rpm = Some(3_000.0);
        let mut source = TStarSource::new(EntrySeed::Fallback(86.0));
        assert_eq!(source.tick(&i).t_star, Some(86.0));
        i.fan_rpm = Some(1_000.0);
        let saturated = source.tick(&i);
        assert_eq!(saturated.t_star, Some(86.0));
        assert!(saturated
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Max)));
    }

    #[test]
    fn held_steady_positive_error_at_ceiling_keeps_target_unreachable_high() {
        let mut i = input();
        i.fresh_view = false;
        i.curve_points = None;
        i.dt_s = 5.0;
        i.fan_rpm = Some(3_000.0);
        let mut source = TStarSource::new(EntrySeed::Fallback(86.0));
        assert_eq!(source.tick(&i).t_star, Some(86.0));

        i.fan_rpm = Some(1_000.0);
        let changing_error = source.tick(&i);
        assert!(changing_error
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Max)));

        // On the next Held PI update P is zero, but the unsuppressed positive
        // integral candidate still proves the ceiling cannot meet demand.
        let steady_error = source.tick(&i);
        assert_eq!(steady_error.t_star, Some(86.0));
        assert!(steady_error
            .flags
            .contains(&TStarFlag::TargetUnreachable(Bound::Max)));
    }

    #[test]
    fn every_hold_has_an_explicit_held_direction() {
        let cases = [
            (Hold::None, false, false),
            (Hold::Shadow, true, false),
            (Hold::Clamp(Bound::Floor), false, true),
            (Hold::Clamp(Bound::Max), true, false),
            (Hold::ActuatorMismatch, true, true),
            (Hold::GroupUnavailable, true, true),
            (Hold::DrawUnavailable, false, false),
            (Hold::Bypass, true, true),
        ];
        for (hold, up, down) in cases {
            assert_eq!(blocks_up(hold), up, "{hold:?} upward");
            assert_eq!(blocks_down(hold), down, "{hold:?} downward");
        }
        let cpu_none = HeldDeviceInput::new(Device::Cpu, Hold::None, Some(70.0));
        let gpu_shadow = HeldDeviceInput::new(Device::Gpu, Hold::Shadow, Some(70.0));
        assert!(!all_block_upward(&[cpu_none, gpu_shadow]));
        assert!(!all_block_downward(&[cpu_none, gpu_shadow]));
    }

    #[test]
    fn every_previous_hold_pair_blocks_only_when_both_previous_decisions_do() {
        let holds = [
            Hold::None,
            Hold::Shadow,
            Hold::Clamp(Bound::Floor),
            Hold::Clamp(Bound::Max),
            Hold::ActuatorMismatch,
            Hold::GroupUnavailable,
            Hold::DrawUnavailable,
            Hold::Bypass,
        ];
        for cpu in holds {
            for gpu in holds {
                let pair = [
                    HeldDeviceInput::new(Device::Cpu, cpu, Some(70.0)),
                    HeldDeviceInput::new(Device::Gpu, gpu, Some(70.0)),
                ];
                assert_eq!(all_block_upward(&pair), blocks_up(cpu) && blocks_up(gpu));
                assert_eq!(
                    all_block_downward(&pair),
                    blocks_down(cpu) && blocks_down(gpu)
                );
            }
        }
    }

    #[test]
    fn held_device_unreachable_requires_sixty_seconds_of_valid_previous_bound() {
        let mut i = input();
        i.fresh_view = false;
        i.dt_s = 5.0;
        i.fan_rpm = Some(3_000.0);
        i.previous_holds = vec![HeldDeviceInput::new(
            Device::Gpu,
            Hold::Clamp(Bound::Floor),
            Some(90.0),
        )];
        let mut source = TStarSource::new(EntrySeed::Fallback(80.0));
        for _ in 0..11 {
            assert!(!source
                .tick(&i)
                .flags
                .contains(&TStarFlag::DeviceUnreachable {
                    device: Device::Gpu,
                    bound: Bound::Floor,
                }));
        }
        assert!(source
            .tick(&i)
            .flags
            .contains(&TStarFlag::DeviceUnreachable {
                device: Device::Gpu,
                bound: Bound::Floor,
            }));
        i.resumed = true;
        assert!(!source
            .tick(&i)
            .flags
            .contains(&TStarFlag::DeviceUnreachable {
                device: Device::Gpu,
                bound: Bound::Floor,
            }));
    }

    #[test]
    fn held_wall_gap_restarts_bound_dwell_and_steep_curve_is_informational() {
        let mut i = input();
        i.fresh_view = false;
        i.dt_s = 5.0;
        i.fan_rpm = Some(3_000.0);
        i.curve_points = Some(vec![(60.0, 10), (70.0, 40), (80.0, 70)]);
        i.previous_holds = vec![HeldDeviceInput::new(
            Device::Cpu,
            Hold::Clamp(Bound::Max),
            Some(60.0),
        )];
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..11 {
            assert!(source.tick(&i).flags.contains(&TStarFlag::SteepCurve));
        }
        i.dt_s = 7200.0;
        let after_gap = source.tick(&i);
        assert!(!after_gap.flags.contains(&TStarFlag::DeviceUnreachable {
            device: Device::Cpu,
            bound: Bound::Max,
        }));
        i.dt_s = 5.0;
        for _ in 0..11 {
            assert!(!source
                .tick(&i)
                .flags
                .contains(&TStarFlag::DeviceUnreachable {
                    device: Device::Cpu,
                    bound: Bound::Max,
                }));
        }
        assert!(source
            .tick(&i)
            .flags
            .contains(&TStarFlag::DeviceUnreachable {
                device: Device::Cpu,
                bound: Bound::Max,
            }));
    }

    #[test]
    fn auto_enters_held_before_a_continuous_15_second_curve_gate() {
        let mut s = TStarSource::new(EntrySeed::Fallback(80.0));
        let i = input();
        assert_eq!(s.tick(&i).state, TStarState::Held);
        assert_eq!(to_curve(&mut s, &i).state, TStarState::Curve);
    }
    #[test]
    fn entry_seed_uses_groups_or_ceiling_and_clamps_to_known_ambient_floor() {
        let mut i = input();
        i.sensors = vec![ambient(83.0), cpu(70.0)];
        let mut s = TStarSource::new(EntrySeed::Groups {
            cpu_c: Some(70.0),
            gpu_c: Some(65.0),
        });
        assert_eq!(s.tick(&i).t_star, Some(86.0));
        i.cpu_group_c = None;
        i.gpu_group_c = None;
        let mut f = TStarSource::new(EntrySeed::Groups {
            cpu_c: None,
            gpu_c: None,
        });
        assert_eq!(f.tick(&i).t_star, Some(86.0));
    }
    #[test]
    fn unchanged_curve_key_does_not_rederive_and_curve_change_reports_delta() {
        let mut s = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut i = input();
        let a = to_curve(&mut s, &i);
        let b = s.tick(&i);
        assert_eq!(a.curve_derivations, b.curve_derivations);
        i.snapped_duty = Some(37);
        let c = s.tick(&i);
        assert_ne!(c.delta_tstar, 0.0);
        assert!(c.restore_thermal);
        assert!(!c.reseed);
    }
    #[test]
    fn exact_uncontrollable_bypasses_but_unknown_is_held() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        let mut s = TStarSource::new(EntrySeed::Fallback(70.0));
        assert_eq!(to_curve(&mut s, &i).state, TStarState::Uncontrollable);
        let mut u = input();
        u.sensors.push(SensorReading {
            label: "mystery".into(),
            value_c: 90.0,
            class: SensorClass::Unknown,
        });
        u.argmax_label = Some("mystery".into());
        let mut s = TStarSource::new(EntrySeed::Fallback(70.0));
        let out = to_curve(&mut s, &u);
        assert_eq!(out.state, TStarState::Held);
        assert!(out
            .flags
            .iter()
            .any(|f| matches!(f, TStarFlag::EcUnknownLabel(_))));
    }
    #[test]
    fn invalid_inputs_release_and_curve_loss_retains_target() {
        let mut s = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut i = input();
        let target = to_curve(&mut s, &i).t_star;
        i.fresh_view = false;
        assert_eq!(s.tick(&i).state, TStarState::Held);
        assert_eq!(s.tick(&i).t_star, target);
        i.ec_valid = false;
        assert_eq!(s.tick(&i).state, TStarState::Released);
    }
    #[test]
    fn stable_uncontrollable_quarantines_and_needs_30_fresh_recovery_samples() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.sensors = vec![ambient(80.0), cpu(70.0)];
        let mut s = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..318 {
            s.tick(&i);
        }
        assert!(s.quarantined("ambient_f75303@4d"));
        assert_eq!(s.state(), TStarState::Held);
        i.sensors[0].value_c = 81.0;
        for _ in 0..29 {
            s.tick(&i);
        }
        assert!(s.quarantined("ambient_f75303@4d"));
        s.tick(&i);
        assert!(!s.quarantined("ambient_f75303@4d"));
    }

    fn quarantine_recovery_gap_case(resumed: bool) {
        let label = "ambient_f75303@4d";
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        source.quarantines.insert(label.into(), (80.0, 0));
        let mut sample = input();
        sample.argmax_label = Some(label.into());
        sample.sensors = vec![ambient(81.0), cpu(70.0)];
        for _ in 0..29 {
            source.tick(&sample);
        }
        assert!(source.quarantined(label), "premise: 29 matches do not recover");

        sample.dt_s = 7_200.0;
        sample.resumed = resumed;
        source.tick(&sample);
        assert!(source.quarantined(label), "the gap sample cannot complete recovery");

        sample.dt_s = 1.0;
        sample.resumed = false;
        for _ in 0..29 {
            source.tick(&sample);
        }
        assert!(source.quarantined(label), "recovery needs 30 new continuous samples after a gap");
        source.tick(&sample);
        assert!(!source.quarantined(label));
    }

    #[test]
    fn quarantine_recovery_streak_does_not_cross_unmarked_wall_gap() {
        quarantine_recovery_gap_case(false);
    }

    #[test]
    fn quarantine_recovery_streak_does_not_cross_resumed_gap() {
        quarantine_recovery_gap_case(true);
    }
    #[test]
    fn persistence_is_rate_limited_and_rejects_uncontrolled_state() {
        let mut s = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut i = input();
        i.fresh_view = false;
        assert!(s.tick(&i).persistence.is_some());
        assert!(s.tick(&i).persistence.is_none());
        for _ in 0..58 {
            s.set_held_target(71.0);
            assert!(s.tick(&i).persistence.is_none());
        }
        s.set_held_target(72.0);
        assert!(s.tick(&i).persistence.is_some());
        let mut u = input();
        u.argmax_label = Some("ambient_f75303@4d".into());
        let mut b = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..17 {
            b.tick(&u);
        }
        assert_eq!(b.state(), TStarState::Uncontrollable);
        assert!(b.tick(&u).persistence.is_none());
    }

    #[test]
    fn unknown_sensor_visibility_blocks_persistence_even_when_it_is_not_argmax() {
        let mut i = input();
        i.sensors.push(SensorReading {
            label: "unexpected".into(),
            value_c: 30.0,
            class: SensorClass::Unknown,
        });
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        assert!(source.tick(&i).persistence.is_none());
    }
    #[test]
    fn curve_cadence_and_auto_exit_save_are_not_lost_to_released_state() {
        let mut s = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut i = input();
        to_curve(&mut s, &i);
        assert!(s.tick(&i).persistence.is_none());
        i.auto_exit = true;
        let out = s.tick(&i);
        assert_eq!(out.state, TStarState::Released);
        assert!(out.persistence.is_some());
    }

    #[test]
    fn argmax_needs_three_fresh_samples_unless_its_lead_is_decisive() {
        let mut i = input();
        i.argmax_lead_c = 0.5;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..16 {
            assert_eq!(source.tick(&i).state, TStarState::Held);
        }
        assert_eq!(source.tick(&i).state, TStarState::Curve);

        let mut decisive = input();
        decisive.argmax_lead_c = 1.1;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..14 {
            assert_eq!(source.tick(&decisive).state, TStarState::Held);
        }
        assert_eq!(source.tick(&decisive).state, TStarState::Curve);
    }

    #[test]
    fn a_stale_or_mismatched_view_exits_bypass_to_held_with_transfer() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 2.0;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        assert_eq!(to_curve(&mut source, &i).state, TStarState::Uncontrollable);
        i.replica_reconciled = false;
        let out = source.tick(&i);
        assert_eq!(out.state, TStarState::Held);
        assert!(out.mode_transfer);
    }

    #[test]
    fn released_recovery_reseeds_held_without_direct_curve_entry() {
        let mut i = input();
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        i.ec_valid = false;
        assert_eq!(source.tick(&i).state, TStarState::Released);
        i.ec_valid = true;
        let out = source.tick(&i);
        assert_eq!(out.state, TStarState::Held);
        assert!(out.reseed && out.resync);
    }

    #[test]
    fn every_curve_gate_condition_resets_the_continuous_entry_window() {
        let mut i = input();
        i.argmax_lead_c = 2.0;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..10 {
            source.tick(&i);
        }
        i.replica_reconciled = false;
        assert_eq!(source.tick(&i).state, TStarState::Held);
        i.replica_reconciled = true;
        assert_eq!(source.tick(&i).state, TStarState::Held);
        for _ in 0..14 {
            assert_eq!(source.tick(&i).state, TStarState::Held);
        }
        assert_eq!(source.tick(&i).state, TStarState::Curve);

        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut invalid = input();
        invalid.argmax_lead_c = 2.0;
        invalid.curve_points = None;
        for _ in 0..30 {
            source.tick(&invalid);
        }
        assert_eq!(source.state(), TStarState::Held);
    }

    #[test]
    fn rolling_span_above_a_quarter_degree_never_quarantines_the_argmax() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 2.0;
        i.sensors = vec![ambient(80.0), cpu(70.0)];
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        for tick in 0..400 {
            i.sensors[0].value_c = if tick % 2 == 0 { 80.0 } else { 80.3 };
            source.tick(&i);
        }
        assert!(!source.quarantined("ambient_f75303@4d"));
        assert_eq!(source.state(), TStarState::Uncontrollable);
    }

    #[test]
    fn cleared_quarantine_restarts_hysteresis_and_stays_held_on_the_30th_sample() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 2.0;
        i.sensors = vec![ambient(80.0), cpu(70.0)];
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        for _ in 0..318 {
            source.tick(&i);
        }
        i.sensors[0].value_c = 81.0;
        i.argmax_lead_c = 0.5;
        for _ in 0..29 {
            source.tick(&i);
        }
        let out = source.tick(&i);
        assert!(!source.quarantined("ambient_f75303@4d"));
        assert_eq!(out.state, TStarState::Held);
        assert!(out.reseed && out.resync);
        for _ in 0..2 {
            assert_eq!(source.tick(&i).state, TStarState::Held);
        }
        assert_eq!(source.tick(&i).state, TStarState::Uncontrollable);
    }

    #[test]
    fn downward_curve_and_held_changes_report_deltas_without_thermal_restore() {
        let mut i = input();
        i.argmax_lead_c = 2.0;
        i.snapped_duty = Some(37);
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        to_curve(&mut source, &i);
        i.snapped_duty = Some(31);
        let curve_out = source.tick(&i);
        assert!(curve_out.delta_tstar < 0.0);
        assert!(!curve_out.restore_thermal && !curve_out.resync);
        i.fresh_view = false;
        source.tick(&i);
        source.set_held_target(65.0);
        let held_out = source.tick(&i);
        assert!(held_out.delta_tstar < 0.0);
        assert!(!held_out.restore_thermal && !held_out.resync);
    }

    #[test]
    fn qualified_seed_wins_and_persistence_rekeys_a_sanitised_target() {
        let mut i = input();
        i.fresh_view = false;
        i.requested_fan_target_rpm = 99_999;
        let mut source = TStarSource::new(EntrySeed::Qualified(75.0));
        let first = source.tick(&i);
        assert_eq!(first.t_star, Some(75.0));
        assert_eq!(
            first
                .persistence
                .expect("initial valid save")
                .seed
                .fan_target_rpm,
            7000
        );
        i.strategy = Some("cool16".into());
        source.set_held_target(74.0);
        for _ in 0..59 {
            source.tick(&i);
        }
        let saved = source.tick(&i).persistence.expect("rekeyed periodic save");
        assert_eq!(saved.seed.strategy, "cool16");
        assert_eq!(saved.seed.value_c, 74.0);
    }

    #[test]
    fn persistence_rejects_released_missing_groups_empty_feasibility_and_quarantine() {
        let mut released = input();
        released.fan_valid = false;
        assert!(TStarSource::new(EntrySeed::Fallback(70.0))
            .tick(&released)
            .persistence
            .is_none());

        let mut no_groups = input();
        no_groups.fresh_view = false;
        no_groups.cpu_group_c = None;
        no_groups.gpu_group_c = None;
        assert!(TStarSource::new(EntrySeed::Fallback(70.0))
            .tick(&no_groups)
            .persistence
            .is_none());

        let mut no_floor = input();
        no_floor.fresh_view = false;
        no_floor.sensors = vec![cpu(70.0)];
        assert!(TStarSource::new(EntrySeed::Fallback(70.0))
            .tick(&no_floor)
            .persistence
            .is_none());

        let mut stuck = input();
        stuck.argmax_label = Some("ambient_f75303@4d".into());
        stuck.argmax_lead_c = 2.0;
        stuck.sensors = vec![ambient(80.0), cpu(70.0)];
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut out = source.tick(&stuck);
        for _ in 0..317 {
            out = source.tick(&stuck);
        }
        assert!(source.quarantined("ambient_f75303@4d"));
        assert!(out.persistence.is_none());
    }

    #[test]
    fn established_curve_uses_argmax_debounce_before_bypass_and_decisive_leads_bypass_now() {
        let mut i = input();
        i.argmax_lead_c = 2.0;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        to_curve(&mut source, &i);
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 0.5;
        assert_eq!(source.tick(&i).state, TStarState::Curve);
        assert_eq!(source.tick(&i).state, TStarState::Curve);
        assert_eq!(source.tick(&i).state, TStarState::Uncontrollable);

        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut decisive = input();
        decisive.argmax_lead_c = 2.0;
        to_curve(&mut source, &decisive);
        decisive.argmax_label = Some("ambient_f75303@4d".into());
        assert_eq!(source.tick(&decisive).state, TStarState::Uncontrollable);
    }

    #[test]
    fn uncontrollable_exit_debounces_to_held_then_requires_a_new_curve_window() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 2.0;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let entered = source.tick(&i);
        assert_eq!(source.state(), TStarState::Uncontrollable);
        assert!(entered.mode_transfer);
        for _ in 0..4 {
            let out = source.tick(&i);
            assert_eq!(out.state, TStarState::Uncontrollable);
            assert!(!out.mode_transfer);
        }
        let frozen = source.tick(&i).t_star;
        i.argmax_label = Some("cpu@4c".into());
        i.argmax_lead_c = 0.5;
        for _ in 0..2 {
            let out = source.tick(&i);
            assert_eq!(out.state, TStarState::Uncontrollable);
            assert_eq!(out.t_star, frozen);
            assert!(!out.mode_transfer);
        }
        let exited = source.tick(&i);
        assert_eq!(exited.state, TStarState::Held);
        assert_eq!(exited.t_star, frozen);
        assert!(exited.mode_transfer);
        for _ in 0..14 {
            let out = source.tick(&i);
            assert_eq!(out.state, TStarState::Held);
            assert!(!out.mode_transfer);
        }
        let curved = source.tick(&i);
        assert_eq!(curved.state, TStarState::Curve);
        assert_ne!(curved.t_star, frozen);
        assert!(curved.delta_tstar > 0.0);
        assert!(!curved.mode_transfer);
    }

    #[test]
    fn released_reengagement_reuses_qualified_seed_instead_of_current_groups() {
        let mut i = input();
        i.fresh_view = false;
        i.cpu_group_c = Some(65.0);
        i.gpu_group_c = Some(60.0);
        let mut source = TStarSource::new(EntrySeed::Qualified(75.0));
        assert_eq!(source.tick(&i).t_star, Some(75.0));
        i.ec_valid = false;
        source.tick(&i);
        i.ec_valid = true;
        let out = source.tick(&i);
        assert_eq!(out.state, TStarState::Held);
        assert_eq!(out.t_star, Some(75.0));
    }

    #[test]
    fn quarantine_tick_recomputes_the_empty_feasibility_flag() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 2.0;
        i.sensors = vec![ambient(80.0), cpu(70.0)];
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let mut out = source.tick(&i);
        for _ in 0..317 {
            out = source.tick(&i);
        }
        assert!(source.quarantined("ambient_f75303@4d"));
        assert!(out.flags.contains(&TStarFlag::EcUncontrollableUnavailable));
        assert_eq!(out.t_star, Some(86.0));
    }

    #[test]
    fn curve_to_uncontrollable_debounces_then_stays_bypassed_without_a_gate() {
        let mut i = input();
        i.argmax_lead_c = 2.0;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let initial_curve = to_curve(&mut source, &i);
        assert_eq!(initial_curve.state, TStarState::Curve);
        let curve_target = initial_curve.t_star;

        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 0.5;
        for _ in 0..2 {
            let out = source.tick(&i);
            assert_eq!(out.state, TStarState::Curve);
            assert_eq!(out.t_star, curve_target);
            assert!(!out.mode_transfer);
        }
        let bypass = source.tick(&i);
        assert_eq!(bypass.state, TStarState::Uncontrollable);
        assert_eq!(bypass.t_star, curve_target);
        assert!(bypass.mode_transfer);
        for _ in 0..20 {
            let out = source.tick(&i);
            assert_eq!(out.state, TStarState::Uncontrollable);
            assert_eq!(out.t_star, curve_target);
            assert!(!out.mode_transfer);
        }
    }

    #[test]
    fn decisive_uncontrollable_argmax_bypasses_immediately_from_held() {
        let mut i = input();
        i.argmax_label = Some("ambient_f75303@4d".into());
        i.argmax_lead_c = 1.1;
        let mut source = TStarSource::new(EntrySeed::Fallback(70.0));
        let out = source.tick(&i);
        assert_eq!(out.state, TStarState::Uncontrollable);
        assert!(out.mode_transfer);
        assert!(out
            .flags
            .contains(&TStarFlag::ArgmaxUncontrollable("ambient_f75303@4d".into())));
    }
}

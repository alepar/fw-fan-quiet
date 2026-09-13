// 1 Hz sensor snapshot shared by all threads (consumed from later tasks).

use crate::fanctrl::client::{FanctrlView, Freshness};
use crate::sensors::ec::EcReading;

/// Origin of the per-device PI gains selected for an Auto session.  This is
/// a wire type rather than the configuration representation so telemetry is
/// stable while persistence and configuration evolve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GainsSource {
    Config,
    Fitted,
    Default,
}

/// State of the shared T* source on the telemetry wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TelemetryTStarState {
    Curve,
    Held,
    Uncontrollable,
    Released,
}

impl From<crate::control::tstar::TStarState> for TelemetryTStarState {
    fn from(value: crate::control::tstar::TStarState) -> Self {
        match value {
            crate::control::tstar::TStarState::Curve => Self::Curve,
            crate::control::tstar::TStarState::Held => Self::Held,
            crate::control::tstar::TStarState::Uncontrollable => Self::Uncontrollable,
            crate::control::tstar::TStarState::Released => Self::Released,
        }
    }
}

/// Device name carried by flags that apply to only one loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TelemetryDeviceName {
    Cpu,
    Gpu,
}

impl From<crate::control::tstar::Device> for TelemetryDeviceName {
    fn from(value: crate::control::tstar::Device) -> Self {
        match value {
            crate::control::tstar::Device::Cpu => Self::Cpu,
            crate::control::tstar::Device::Gpu => Self::Gpu,
        }
    }
}

/// Serde-friendly copy of a [`crate::control::device_loop::Selected`] value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TelemetrySelected {
    Thermal,
    Shadow,
    Floor,
    Max,
}

impl From<crate::control::device_loop::Selected> for TelemetrySelected {
    fn from(value: crate::control::device_loop::Selected) -> Self {
        match value {
            crate::control::device_loop::Selected::Thermal => Self::Thermal,
            crate::control::device_loop::Selected::Shadow => Self::Shadow,
            crate::control::device_loop::Selected::Floor => Self::Floor,
            crate::control::device_loop::Selected::Max => Self::Max,
        }
    }
}

/// Serde-friendly copy of a [`crate::control::device_loop::Bound`] value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TelemetryBound {
    Floor,
    Max,
}

impl From<crate::control::device_loop::Bound> for TelemetryBound {
    fn from(value: crate::control::device_loop::Bound) -> Self {
        match value {
            crate::control::device_loop::Bound::Floor => Self::Floor,
            crate::control::device_loop::Bound::Max => Self::Max,
        }
    }
}

/// Per-device hold reason on the telemetry wire.  `Clamp` retains the bound
/// that made the distinction useful offline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TelemetryHold {
    None,
    Shadow,
    Clamp { bound: TelemetryBound },
    ActuatorMismatch,
    GroupUnavailable,
    DrawUnavailable,
    Bypass,
}

impl From<crate::control::device_loop::Hold> for TelemetryHold {
    fn from(value: crate::control::device_loop::Hold) -> Self {
        match value {
            crate::control::device_loop::Hold::None => Self::None,
            crate::control::device_loop::Hold::Shadow => Self::Shadow,
            crate::control::device_loop::Hold::Clamp(bound) => Self::Clamp {
                bound: bound.into(),
            },
            crate::control::device_loop::Hold::ActuatorMismatch => Self::ActuatorMismatch,
            crate::control::device_loop::Hold::GroupUnavailable => Self::GroupUnavailable,
            crate::control::device_loop::Hold::DrawUnavailable => Self::DrawUnavailable,
            crate::control::device_loop::Hold::Bypass => Self::Bypass,
        }
    }
}

/// Full per-device Decision payload in telemetry schema v3.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TelemetryDevice {
    pub group_c: Option<f64>,
    pub err_c: Option<f64>,
    pub thermal: f64,
    pub shadow: f64,
    pub cap: f64,
    pub selected: TelemetrySelected,
    pub hold: TelemetryHold,
    pub gains_source: GainsSource,
}

impl From<(crate::control::device_loop::DeviceDecision, GainsSource)> for TelemetryDevice {
    fn from(
        (decision, gains_source): (crate::control::device_loop::DeviceDecision, GainsSource),
    ) -> Self {
        Self {
            group_c: decision.group_c,
            err_c: decision.err_c,
            thermal: decision.thermal,
            shadow: decision.shadow,
            cap: decision.cap,
            selected: decision.selected.into(),
            hold: decision.hold.into(),
            gains_source,
        }
    }
}

/// Structured, polarity-carrying decision flag for schema v3.  Unlike the
/// legacy string list, labels and bounds survive the JSONL line so offline
/// consumers can identify the sensor or device that caused a state change.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "name", rename_all = "snake_case")]
pub enum TelemetryFlag {
    ArgmaxUncontrollable {
        label: String,
        active: bool,
    },
    ArgmaxStuck {
        label: String,
        active: bool,
    },
    EcUnknownLabel {
        label: String,
        active: bool,
    },
    EcImplausible {
        label: String,
        active: bool,
    },
    EcUncontrollableUnavailable {
        active: bool,
    },
    GroupLost {
        device: TelemetryDeviceName,
        active: bool,
    },
    DeviceUnreachable {
        device: TelemetryDeviceName,
        bound: TelemetryBound,
        active: bool,
    },
    TargetUnreachable {
        bound: TelemetryBound,
        active: bool,
    },
    SteepCurve {
        active: bool,
    },
    /// Existing controller flags are retained until the controller wiring
    /// supplies their richer v3 equivalents.
    Legacy {
        flag: String,
        active: bool,
    },
}

impl TelemetryFlag {
    pub fn legacy(flag: impl Into<String>) -> Self {
        Self::Legacy {
            flag: flag.into(),
            active: true,
        }
    }
}

/// Converts a live T* diagnostic into its labelled, active v3 wire
/// representation. A diagnostic clears by disappearing from the next
/// decision snapshot; `active` remains on the wire for consumers that merge
/// snapshots with standalone flag events.
impl From<&crate::control::tstar::TStarFlag> for TelemetryFlag {
    fn from(flag: &crate::control::tstar::TStarFlag) -> Self {
        use crate::control::tstar::TStarFlag;

        match flag {
            TStarFlag::ArgmaxUncontrollable(label) => Self::ArgmaxUncontrollable {
                label: label.clone(),
                active: true,
            },
            TStarFlag::ArgmaxStuck(label) => Self::ArgmaxStuck {
                label: label.clone(),
                active: true,
            },
            TStarFlag::EcUnknownLabel(label) => Self::EcUnknownLabel {
                label: label.clone(),
                active: true,
            },
            TStarFlag::EcUncontrollableUnavailable => {
                Self::EcUncontrollableUnavailable { active: true }
            }
            TStarFlag::TargetUnreachable(bound) => Self::TargetUnreachable {
                bound: (*bound).into(),
                active: true,
            },
            TStarFlag::SteepCurve => Self::SteepCurve { active: true },
            TStarFlag::DeviceUnreachable { device, bound } => Self::DeviceUnreachable {
                device: (*device).into(),
                bound: (*bound).into(),
                active: true,
            },
        }
    }
}

/// `Sample` is no longer `Copy` (it carries an `EcReading` and a
/// `FanctrlView`, both of which own a `Vec`/`String`) -- every fan-out site
/// (`sensors::sampler::Sampler::spawn`) must `.clone()` per subscriber
/// instead of relying on an implicit bitwise copy.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Sample {
    pub t_mono: f64, // seconds, monotonic
    /// Acquisition origin on the same clock as t_mono, for command completion pairing.
    /// Synthetic samples omit this and use their deterministic t_mono directly.
    #[serde(skip)]
    pub acquired_at: Option<std::time::Instant>,
    pub fan1_rpm: f64,
    pub fan2_rpm: f64,
    pub cpu_temp_c: f64, // Tctl
    pub cpu_pkg_w: f64,  // RAPL delta
    pub igpu_w: f64,     // amdgpu
    pub gpu_w: f64,      // NVML
    pub gpu_temp_c: f64,
    pub gpu_sm_mhz: f64,
    pub gpu_util_pct: f64,
    pub cpu_util_pct: f64,
    pub cpu_avg_mhz: f64,
    pub resumed: bool, // monotonic jump detected since last sample

    // Validity flags: 0.0 is a legitimate reading (e.g. fans stopped), so the
    // sampler flattens None -> 0.0 for the numeric fields but records sensor
    // presence here. "Sensor lost" stays distinguishable from a real zero --
    // the controller must never raise power off a phantom reading.
    pub fan_valid: bool,      // hwmon fan_rpms() returned Some
    pub cpu_temp_valid: bool, // hwmon cpu_temp_c() returned Some
    pub gpu_w_valid: bool,    // NVML power reading returned Some
    pub gpu_temp_valid: bool, // NVML temperature reading returned Some
    pub gpu_mhz_valid: bool,  // NVML SM clock reading returned Some

    // --- fw-fanctrl closed-loop additions (design doc §3.4 / fwloop.9) ---
    /// `cros_ec` sensor replica (design doc §2.2), read at the sampler tick
    /// (cheap sysfs reads, unlike the NVMe SMART read below). `None` when the
    /// chip is missing or every reading was dropped -- see `ec_valid`.
    /// Skipped on the telemetry wire for now: `EcReading` doesn't derive
    /// `Serialize` (owned by an earlier task) and the flattened
    /// `ec_max`/`ec_argmax`/`ec_ma` telemetry columns are a later task's
    /// (design doc §3.5, fwloop.15).
    #[serde(skip)]
    pub ec: Option<EcReading>,
    pub ec_valid: bool,
    /// NVMe composite temperature (design doc §3.4): a stamped last-good
    /// value merged in from a dedicated background thread
    /// (`sensors::poller::read_nvme`), never read on this tick directly --
    /// a SMART admin read can block for the kernel's 60 s `admin_timeout`,
    /// which would otherwise stall every `Sample`. `None` when no reading has
    /// ever landed, or the last one is stale.
    pub nvme_temp_c: Option<f64>,
    /// fw-fanctrl's last-known state (design doc §2.1), merged in from the
    /// background `FanctrlPoller` (`sensors::poller`) -- this tick never
    /// polls the socket itself. `None` until the first successful `print
    /// all`. Skipped on the telemetry wire for the same reason as `ec`:
    /// `FanctrlView` carries `Instant` stamps, which serde cannot serialize,
    /// and the flattened telemetry columns are fwloop.15's.
    #[serde(skip)]
    pub fanctrl: Option<FanctrlView>,
    /// [`Freshness`] of `fanctrl` as of this sample's `t_mono` (computed from
    /// `FanctrlView`'s own monotonic stamps -- never this sample's wall
    /// clock). Skipped on the telemetry wire; see `fanctrl`'s doc comment.
    #[serde(skip)]
    pub fanctrl_freshness: Freshness,
    /// True on exactly the first sample whose `fanctrl` view's
    /// `all_observed_at` differs from the previous sample's -- a
    /// `Speed`-only refresh (which never touches `all_observed_at`) never
    /// sets this.
    pub fanctrl_view_changed: bool,
    /// AC adapter presence (`/sys/class/power_supply/ACAD/online`). A plain
    /// `bool`, not `Option` + validity flag like the sensors above -- design
    /// doc §3.4 specifies `on_ac: bool` directly; a missing/unreadable/
    /// unexpected reading flattens to `false` (assume battery, the more
    /// conservative of the two guesses for the read-back mismatch grace
    /// period this feeds, §2.9).
    pub on_ac: bool,
}

impl Sample {
    /// Both fans share one cooling assembly; control targets the higher reading.
    pub fn max_fan_rpm(&self) -> f64 {
        self.fan1_rpm.max(self.fan2_rpm)
    }
}

impl Default for Sample {
    /// Manual (not derived): `fanctrl_freshness: Freshness` has no `Default`
    /// of its own (it lives in an earlier task's file, `fanctrl/client.rs`,
    /// outside this task's scope to extend) -- every other field is either a
    /// primitive or an `Option`, both `Default`-able for free.
    fn default() -> Self {
        Sample {
            t_mono: 0.0,
            acquired_at: None,
            fan1_rpm: 0.0,
            fan2_rpm: 0.0,
            cpu_temp_c: 0.0,
            cpu_pkg_w: 0.0,
            igpu_w: 0.0,
            gpu_w: 0.0,
            gpu_temp_c: 0.0,
            gpu_sm_mhz: 0.0,
            gpu_util_pct: 0.0,
            cpu_util_pct: 0.0,
            cpu_avg_mhz: 0.0,
            resumed: false,
            fan_valid: false,
            cpu_temp_valid: false,
            gpu_w_valid: false,
            gpu_temp_valid: false,
            gpu_mhz_valid: false,
            ec: None,
            ec_valid: false,
            nvme_temp_c: None,
            fanctrl: None,
            // No poll has ever been attempted for a bare-default Sample
            // (mainly test fixtures): matches what `compute_freshness` itself
            // returns for that exact state (`last_absent: false, view: None`)
            // -- see `fanctrl::client::compute_freshness`.
            fanctrl_freshness: Freshness::Stale,
            fanctrl_view_changed: false,
            on_ac: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_carries_every_new_fw_fanctrl_loop_field() {
        // A `Sample::default()` must expose all seven new fields with the
        // documented "nothing observed yet" defaults -- this is the
        // field-presence test implementation step 1 asks for; the merge
        // logic that actually populates them from live sources is exercised
        // in `sensors::sampler`'s and `sensors::poller`'s own test modules.
        let s = Sample::default();
        assert_eq!(s.ec, None);
        assert!(!s.ec_valid);
        assert_eq!(s.nvme_temp_c, None);
        assert_eq!(s.fanctrl, None);
        assert_eq!(s.fanctrl_freshness, Freshness::Stale);
        assert!(!s.fanctrl_view_changed);
        assert!(!s.on_ac);
        // resumed already existed before this task; still present.
        assert!(!s.resumed);
    }

    #[test]
    fn telemetry_device_roundtrips_every_selected_hold_and_gains_source() {
        use crate::control::device_loop::{DeviceDecision, Hold, Selected};

        let selections = [
            Selected::Thermal,
            Selected::Shadow,
            Selected::Floor,
            Selected::Max,
        ];
        let holds = [
            Hold::None,
            Hold::Shadow,
            Hold::Clamp(crate::control::device_loop::Bound::Floor),
            Hold::Clamp(crate::control::device_loop::Bound::Max),
            Hold::ActuatorMismatch,
            Hold::GroupUnavailable,
            Hold::DrawUnavailable,
            Hold::Bypass,
        ];
        let sources = [
            GainsSource::Config,
            GainsSource::Fitted,
            GainsSource::Default,
        ];

        for selected in selections {
            for hold in holds {
                for gains_source in sources {
                    let decision = DeviceDecision {
                        t_star: 72.0,
                        group_c: Some(70.0),
                        err_c: Some(2.0),
                        thermal: 2100.0,
                        shadow: 2050.0,
                        cap: 2050.0,
                        selected,
                        hold,
                        write_allowed: true,
                        group_lost: false,
                        write_immediately: false,
                    };
                    let wire = TelemetryDevice::from((decision, gains_source));
                    let json = serde_json::to_string(&wire).unwrap();
                    assert_eq!(
                        serde_json::from_str::<TelemetryDevice>(&json).unwrap(),
                        wire
                    );
                }
            }
        }
    }

    #[test]
    fn telemetry_flags_preserve_tstar_sensor_labels_and_polarity() {
        use crate::control::device_loop::Bound;
        use crate::control::tstar::{Device, TStarFlag};

        let tstar_flags = [
            TStarFlag::ArgmaxUncontrollable("ambient_f75303@4d".into()),
            TStarFlag::ArgmaxStuck("charger_f75303@4d".into()),
            TStarFlag::EcUnknownLabel("future_sensor".into()),
            TStarFlag::EcUncontrollableUnavailable,
            TStarFlag::TargetUnreachable(Bound::Floor),
            TStarFlag::SteepCurve,
            TStarFlag::DeviceUnreachable {
                device: Device::Gpu,
                bound: Bound::Max,
            },
        ];
        let mut wire_flags: Vec<_> = tstar_flags.iter().map(TelemetryFlag::from).collect();
        wire_flags.extend([
            TelemetryFlag::EcImplausible {
                label: "gpu_temp@40".into(),
                active: true,
            },
            TelemetryFlag::GroupLost {
                device: TelemetryDeviceName::Cpu,
                active: false,
            },
        ]);

        let json = serde_json::to_value(&wire_flags).unwrap();
        assert_eq!(json[0]["name"], "argmax_uncontrollable");
        assert_eq!(json[0]["label"], "ambient_f75303@4d");
        assert_eq!(json[1]["name"], "argmax_stuck");
        assert_eq!(json[2]["name"], "ec_unknown_label");
        assert_eq!(json[3]["name"], "ec_uncontrollable_unavailable");
        assert_eq!(json[4]["bound"], "floor");
        assert_eq!(json[5]["name"], "steep_curve");
        assert_eq!(json[6]["device"], "gpu");
        assert_eq!(json[6]["bound"], "max");
        assert_eq!(json[7]["name"], "ec_implausible");
        assert_eq!(json[7]["label"], "gpu_temp@40");
        assert_eq!(json[8]["name"], "group_lost");
        assert_eq!(json[8]["active"], false);
        assert!(json
            .as_array()
            .unwrap()
            .iter()
            .all(|flag| flag["active"].is_boolean()));
    }
}

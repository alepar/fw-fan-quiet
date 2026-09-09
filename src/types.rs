// 1 Hz sensor snapshot shared by all threads (consumed from later tasks).

use crate::fanctrl::client::{FanctrlView, Freshness};
use crate::sensors::ec::EcReading;

/// `Sample` is no longer `Copy` (it carries an `EcReading` and a
/// `FanctrlView`, both of which own a `Vec`/`String`) -- every fan-out site
/// (`sensors::sampler::Sampler::spawn`) must `.clone()` per subscriber
/// instead of relying on an implicit bitwise copy.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Sample {
    pub t_mono: f64, // seconds, monotonic
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
}

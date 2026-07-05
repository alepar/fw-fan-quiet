//! Thermal watchdog (design §3): pure trip logic, no actuation.
//!
//! Thermal safety outranks acoustics: sustained Tctl ≥ [`CPU_TRIP_C`] or GPU
//! ≥ [`GPU_TRIP_C`] means our caps may be strangling the cooling response —
//! the controller must release EVERYTHING toward stock. The watchdog is a
//! one-shot latch: it trips once, then stays silent until [`rearm`]ed by an
//! explicit user acknowledgment (design amendment: an emergency must never
//! clear itself; `ThermalWatchdog::rearm`).
//!
//! Sensor-lost rule (Task 7 amendment): [`SENSOR_LOST_STREAK`] consecutive
//! samples with `cpu_temp_valid == false` ALSO trip — assume hot. A lost
//! sensor must never let the watchdog go blind while limits are applied.

use crate::types::Sample;

/// Tctl trip threshold (°C): sustained readings at/above this trip.
pub const CPU_TRIP_C: f64 = 95.0;
/// GPU temperature trip threshold (°C).
pub const GPU_TRIP_C: f64 = 87.0;
/// Consecutive hot samples (either device) before the thermal trip.
pub const TRIP_STREAK: u8 = 3;
/// Consecutive `cpu_temp_valid == false` samples before the sensor-lost
/// trip (assume hot).
pub const SENSOR_LOST_STREAK: u8 = 10;

/// One observation's verdict. The plan sketched `{ None, Emergency }`; the
/// emergency carries its reason here because the controller surfaces two
/// DISTINCT flags (`ThermalEmergency` vs `SensorLost`) with identical
/// release + manual-re-arm semantics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trip {
    None,
    /// [`TRIP_STREAK`] consecutive samples with a valid sensor at/over its
    /// trip threshold.
    Thermal,
    /// [`SENSOR_LOST_STREAK`] consecutive samples without a valid CPU
    /// temperature: assume hot.
    SensorLost,
}

/// Pure trigger logic; the controller feeds it every sample and acts on the
/// returned [`Trip`].
pub struct ThermalWatchdog {
    /// Consecutive samples with some valid sensor at/over its threshold.
    hot_streak: u8,
    /// Consecutive samples with `cpu_temp_valid == false`.
    lost_streak: u8,
    /// Latched after a trip; `observe` returns `Trip::None` (no re-trip
    /// spam) until `rearm`.
    tripped: bool,
}

impl ThermalWatchdog {
    pub fn new() -> Self {
        Self {
            hot_streak: 0,
            lost_streak: 0,
            tripped: false,
        }
    }

    /// Feed one 1 Hz sample. Returns a trip exactly once (then `Trip::None`
    /// while tripped, until [`Self::rearm`]).
    ///
    /// Streak semantics:
    /// - A sample is HOT if any VALID sensor reads at/over its threshold;
    ///   [`TRIP_STREAK`] consecutive hot samples trip (`Trip::Thermal`).
    /// - A sample with at least one valid sensor and none hot is COOL and
    ///   resets the hot streak.
    /// - A sample with NO valid temperature neither extends nor resets the
    ///   hot streak — but [`SENSOR_LOST_STREAK`] consecutive samples without
    ///   a valid CPU temperature trip on their own (`Trip::SensorLost`).
    pub fn observe(&mut self, s: &Sample) -> Trip {
        if self.tripped {
            return Trip::None;
        }
        if s.cpu_temp_valid {
            self.lost_streak = 0;
        } else {
            self.lost_streak = self.lost_streak.saturating_add(1);
            if self.lost_streak >= SENSOR_LOST_STREAK {
                self.tripped = true;
                return Trip::SensorLost;
            }
        }
        let cpu_hot = s.cpu_temp_valid && s.cpu_temp_c >= CPU_TRIP_C;
        let gpu_hot = s.gpu_temp_valid && s.gpu_temp_c >= GPU_TRIP_C;
        if cpu_hot || gpu_hot {
            self.hot_streak = self.hot_streak.saturating_add(1);
            if self.hot_streak >= TRIP_STREAK {
                self.tripped = true;
                return Trip::Thermal;
            }
        } else if s.cpu_temp_valid || s.gpu_temp_valid {
            // Genuinely cool evidence resets; an all-invalid sample leaves
            // the streak untouched (it is neither hot nor cool evidence).
            self.hot_streak = 0;
        }
        Trip::None
    }

    /// Latched after a trip (the controller keeps the emergency flag up and
    /// swallows the first actuating command as the acknowledgment).
    pub fn is_tripped(&self) -> bool {
        self.tripped
    }

    /// Nothing latched and no adverse streak building. The controller uses
    /// this to detect the END of an idle-Monitor trip episode: a mere
    /// `Trip::None` is not enough, because after an idle re-arm the next
    /// hot/invalid samples also return `Trip::None` while the streak
    /// rebuilds toward the next trip.
    pub fn is_quiet(&self) -> bool {
        !self.tripped && self.hot_streak == 0 && self.lost_streak == 0
    }

    /// Manual re-arm (the user acknowledged the emergency): clears the latch
    /// AND both streaks, so re-tripping needs fresh consecutive evidence.
    pub fn rearm(&mut self) {
        self.tripped = false;
        self.hot_streak = 0;
        self.lost_streak = 0;
    }
}

impl Default for ThermalWatchdog {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Valid-and-cool on both sensors.
    fn cool() -> Sample {
        Sample {
            cpu_temp_c: 60.0,
            cpu_temp_valid: true,
            gpu_temp_c: 50.0,
            gpu_temp_valid: true,
            ..Sample::default()
        }
    }

    fn hot_cpu() -> Sample {
        Sample {
            cpu_temp_c: CPU_TRIP_C,
            ..cool()
        }
    }

    fn hot_gpu() -> Sample {
        Sample {
            gpu_temp_c: GPU_TRIP_C,
            ..cool()
        }
    }

    /// No valid temperature at all.
    fn invalid() -> Sample {
        Sample::default()
    }

    #[test]
    fn trips_on_three_consecutive_hot_cpu_samples() {
        let mut wd = ThermalWatchdog::new();
        assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        assert!(!wd.is_tripped());
        assert_eq!(wd.observe(&hot_cpu()), Trip::Thermal);
        assert!(wd.is_tripped());
    }

    #[test]
    fn trips_on_three_consecutive_hot_gpu_samples() {
        let mut wd = ThermalWatchdog::new();
        assert_eq!(wd.observe(&hot_gpu()), Trip::None);
        assert_eq!(wd.observe(&hot_gpu()), Trip::None);
        assert_eq!(wd.observe(&hot_gpu()), Trip::Thermal);
    }

    #[test]
    fn mixed_hot_samples_share_one_streak() {
        // Either device over ITS threshold counts: the streak measures
        // "machine too hot", not one sensor.
        let mut wd = ThermalWatchdog::new();
        assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        assert_eq!(wd.observe(&hot_gpu()), Trip::None);
        assert_eq!(wd.observe(&hot_cpu()), Trip::Thermal);
    }

    #[test]
    fn a_cool_sample_resets_the_hot_streak() {
        let mut wd = ThermalWatchdog::new();
        wd.observe(&hot_cpu());
        wd.observe(&hot_cpu());
        assert_eq!(wd.observe(&cool()), Trip::None);
        // Fresh streak needed: two more are not enough...
        assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        // ...the third trips.
        assert_eq!(wd.observe(&hot_cpu()), Trip::Thermal);
    }

    #[test]
    fn just_below_thresholds_never_trips() {
        let mut wd = ThermalWatchdog::new();
        let warm = Sample {
            cpu_temp_c: CPU_TRIP_C - 0.1,
            gpu_temp_c: GPU_TRIP_C - 0.1,
            ..cool()
        };
        for _ in 0..20 {
            assert_eq!(wd.observe(&warm), Trip::None);
        }
        assert!(!wd.is_tripped());
    }

    #[test]
    fn invalid_samples_neither_extend_nor_reset_the_hot_streak() {
        let mut wd = ThermalWatchdog::new();
        wd.observe(&hot_cpu());
        wd.observe(&hot_cpu());
        // Invalid gap: not hot evidence (no trip), not cool (no reset).
        assert_eq!(wd.observe(&invalid()), Trip::None);
        assert_eq!(wd.observe(&hot_cpu()), Trip::Thermal);
    }

    #[test]
    fn ten_consecutive_invalid_samples_trip_sensor_lost() {
        let mut wd = ThermalWatchdog::new();
        for _ in 0..9 {
            assert_eq!(wd.observe(&invalid()), Trip::None);
        }
        assert_eq!(wd.observe(&invalid()), Trip::SensorLost);
        assert!(wd.is_tripped());
    }

    #[test]
    fn a_valid_sample_resets_the_lost_streak() {
        let mut wd = ThermalWatchdog::new();
        for _ in 0..9 {
            wd.observe(&invalid());
        }
        wd.observe(&cool());
        for _ in 0..9 {
            assert_eq!(wd.observe(&invalid()), Trip::None);
        }
        assert_eq!(wd.observe(&invalid()), Trip::SensorLost);
    }

    #[test]
    fn gpu_only_validity_does_not_feed_the_lost_streak_reset() {
        // The sensor-lost rule watches the CPU temperature specifically: a
        // valid GPU reading must not mask a lost Tctl.
        let mut wd = ThermalWatchdog::new();
        let gpu_only = Sample {
            gpu_temp_c: 50.0,
            gpu_temp_valid: true,
            ..Sample::default()
        };
        for _ in 0..9 {
            assert_eq!(wd.observe(&gpu_only), Trip::None);
        }
        assert_eq!(wd.observe(&gpu_only), Trip::SensorLost);
    }

    #[test]
    fn tripped_watchdog_stays_silent_until_rearmed() {
        let mut wd = ThermalWatchdog::new();
        wd.observe(&hot_cpu());
        wd.observe(&hot_cpu());
        assert_eq!(wd.observe(&hot_cpu()), Trip::Thermal);
        // Still hot: no re-trip spam.
        for _ in 0..20 {
            assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        }
        assert!(wd.is_tripped());
    }

    #[test]
    fn rearm_allows_retripping_with_a_fresh_streak() {
        let mut wd = ThermalWatchdog::new();
        wd.observe(&hot_cpu());
        wd.observe(&hot_cpu());
        assert_eq!(wd.observe(&hot_cpu()), Trip::Thermal);
        wd.rearm();
        assert!(!wd.is_tripped());
        // Streaks were cleared: a full fresh streak is needed.
        assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        assert_eq!(wd.observe(&hot_cpu()), Trip::None);
        assert_eq!(wd.observe(&hot_cpu()), Trip::Thermal);
    }

    #[test]
    fn is_quiet_tracks_streaks_and_latch() {
        let mut wd = ThermalWatchdog::new();
        assert!(wd.is_quiet());
        // A building hot streak is not quiet, even though observe still
        // returns Trip::None (the idle-warn debounce keys off this).
        wd.observe(&hot_cpu());
        assert!(!wd.is_quiet());
        // Cool evidence resets the streak: quiet again.
        wd.observe(&cool());
        assert!(wd.is_quiet());
        // A building lost streak is not quiet either.
        wd.observe(&invalid());
        assert!(!wd.is_quiet());
        wd.observe(&cool());
        assert!(wd.is_quiet());
        // Tripped (latched) is never quiet; re-arm restores quiet.
        for _ in 0..3 {
            wd.observe(&hot_cpu());
        }
        assert!(wd.is_tripped());
        assert!(!wd.is_quiet());
        wd.rearm();
        assert!(wd.is_quiet());
    }

    #[test]
    fn rearm_clears_the_lost_streak_too() {
        let mut wd = ThermalWatchdog::new();
        for _ in 0..10 {
            wd.observe(&invalid());
        }
        assert!(wd.is_tripped());
        wd.rearm();
        for _ in 0..9 {
            assert_eq!(wd.observe(&invalid()), Trip::None);
        }
        assert_eq!(wd.observe(&invalid()), Trip::SensorLost);
    }
}

// 1 Hz sensor snapshot shared by all threads (consumed from later tasks).
#![allow(dead_code)]

#[derive(Debug, Clone, Copy, Default)]
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
}

impl Sample {
    /// Both fans share one cooling assembly; control targets the higher reading.
    pub fn max_fan_rpm(&self) -> f64 {
        self.fan1_rpm.max(self.fan2_rpm)
    }
}

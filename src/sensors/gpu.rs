//! NVIDIA GPU sensor via NVML (power, temperature, SM clock, utilization).
//!
//! Thin wrapper over `nvml_wrapper`. Only `Nvml` is stored; the device handle
//! is re-fetched by index on every read (cheap lookup, avoids the
//! self-referential `Nvml`/`Device` lifetime knot). Each getter is
//! individually tolerant: a failed NVML call logs at debug and yields `None`
//! -- a driver hiccup must never kill sampling.

use nvml_wrapper::Nvml;
use nvml_wrapper::enum_wrappers::device::{Clock, TemperatureSensor};

/// Milliwatts (NVML's `power_usage` unit) to watts.
pub fn mw_to_w(mw: u32) -> f64 {
    f64::from(mw) / 1000.0
}

/// One GPU sample. Any field is `None` if its NVML call failed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GpuReading {
    pub power_w: Option<f64>,
    pub temp_c: Option<f64>,
    pub sm_mhz: Option<f64>,
    pub util_pct: Option<f64>,
}

/// Handle to device 0 via NVML. Construction fails if the NVIDIA driver is
/// absent or no device is present; callers keep `Option<GpuSensor>` and
/// sample without GPU readings in that case.
pub struct GpuSensor {
    nvml: Nvml,
}

const DEVICE_INDEX: u32 = 0;

impl GpuSensor {
    /// Initializes NVML and verifies device 0 exists.
    pub fn new() -> color_eyre::Result<Self> {
        let nvml = Nvml::init()?;
        nvml.device_by_index(DEVICE_INDEX)?;
        Ok(Self { nvml })
    }

    /// Reads all fields; each failed getter becomes `None` (logged at debug).
    pub fn read(&self) -> GpuReading {
        let device = match self.nvml.device_by_index(DEVICE_INDEX) {
            Ok(d) => d,
            Err(e) => {
                tracing::debug!("nvml: device_by_index({DEVICE_INDEX}) failed: {e}");
                return GpuReading::default();
            }
        };
        GpuReading {
            power_w: device
                .power_usage()
                .map(mw_to_w)
                .inspect_err(|e| tracing::debug!("nvml: power_usage failed: {e}"))
                .ok(),
            temp_c: device
                .temperature(TemperatureSensor::Gpu)
                .map(f64::from)
                .inspect_err(|e| tracing::debug!("nvml: temperature failed: {e}"))
                .ok(),
            sm_mhz: device
                .clock_info(Clock::SM)
                .map(f64::from)
                .inspect_err(|e| tracing::debug!("nvml: clock_info(SM) failed: {e}"))
                .ok(),
            util_pct: device
                .utilization_rates()
                .map(|u| f64::from(u.gpu))
                .inspect_err(|e| tracing::debug!("nvml: utilization_rates failed: {e}"))
                .ok(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mw_to_w_converts() {
        assert_eq!(mw_to_w(15_500), 15.5);
        assert_eq!(mw_to_w(0), 0.0);
        assert_eq!(mw_to_w(1), 0.001);
    }

    /// Manual smoke check against the real GPU (requires NVIDIA driver).
    /// Run with: cargo test -- --ignored nvml_smoke
    #[test]
    #[ignore = "requires NVIDIA GPU + driver; run manually"]
    fn nvml_smoke_reads_real_gpu() {
        let sensor = GpuSensor::new().expect("NVML init + device 0");
        let reading = sensor.read();
        println!("GpuReading: {reading:?}");
        let power = reading.power_w.expect("power_w should read on real GPU");
        let temp = reading.temp_c.expect("temp_c should read on real GPU");
        assert!(
            (1.0..=600.0).contains(&power),
            "implausible power: {power} W"
        );
        assert!((10.0..=110.0).contains(&temp), "implausible temp: {temp} C");
        assert!(reading.sm_mhz.is_some(), "sm_mhz should read on real GPU");
        assert!(
            reading.util_pct.is_some(),
            "util_pct should read on real GPU"
        );
    }
}

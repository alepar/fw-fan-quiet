// Sensors are consumed by the sampler thread (Task 7).
pub mod cpu;
pub mod ec;
pub mod gpu;
pub mod hwmon;
// FanctrlPoller + the NVMe last-good-value poller (Task 14 / fwloop.9):
// background threads the sampler tick merges from, never polls on.
pub mod poller;
pub mod rapl;
pub mod sampler;

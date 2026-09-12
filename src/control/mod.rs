//! Control layer (design doc §2/§3): the controller thread is the ONLY place
//! hardware writes happen after startup. It owns the actuators (via
//! `RestoreGuard`) and turns UI commands + 1 Hz samples into actuation,
//! reasserts, watchdog flags and `ControlStatus` updates.

pub mod allocator;
pub mod budget;
pub mod controller;
// Staged core API: controller/shadow consumers land in the following beads.
#[allow(dead_code)]
pub mod device_loop;
pub mod gpu_pid;
pub mod guards;
pub mod lut;
pub mod mode;
#[cfg(test)]
mod sim_tests;
#[cfg(test)]
pub mod spike_antiwindup;
pub mod watchdog;

// Mode/StatusFlag are reached via `controller::` where needed (view/tests).
pub use controller::{Command, ControlStatus};

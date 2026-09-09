//! Control layer (design doc §2/§3): the controller thread is the ONLY place
//! hardware writes happen after startup. It owns the actuators (via
//! `RestoreGuard`) and turns UI commands + 1 Hz samples into actuation,
//! reasserts, watchdog flags and `ControlStatus` updates.

pub mod allocator;
pub mod budget;
pub mod controller;
pub mod cooldown;
pub mod gpu_pid;
pub mod guards;
pub mod kalman;
pub mod lut;
pub mod mode;
pub mod thermal_model;
// Superseded by `kalman` in production; retained test-only as the allocator
// field-replay sim's minutes-scale adaptation stand-in.
#[cfg(test)]
pub mod trim;
pub mod trust;
pub mod watchdog;

// Mode/StatusFlag are reached via `controller::` where needed (view/tests).
pub use controller::{Command, ControlStatus};

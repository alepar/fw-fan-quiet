//! Control layer (design doc §2/§3): the controller thread is the ONLY place
//! hardware writes happen after startup. It owns the actuators (via
//! `RestoreGuard`) and turns UI commands + 1 Hz samples into actuation,
//! reasserts, watchdog flags and `ControlStatus` updates.

pub mod controller;

// Mode/StatusFlag are reached via `controller::` where needed (view/tests).
pub use controller::{Command, ControlStatus};

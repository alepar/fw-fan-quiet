//! Control layer (design doc §2/§3): the controller thread is the ONLY place
//! hardware writes happen after startup. It owns the actuators (via
//! `RestoreGuard`) and turns UI commands + 1 Hz samples into actuation,
//! reasserts, watchdog flags and `ControlStatus` updates.

pub mod controller;

// Mode/StatusFlag join this re-export when the UI consumes them (Task 15).
pub use controller::{Command, ControlStatus};

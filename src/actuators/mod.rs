//! Actuators: command runner seam + hardware knobs. Consumed by the
//! controller thread (Task 14) and RestoreGuard.

pub mod cmd;
pub mod cpu;
pub mod gpu;
pub mod guard;
pub mod smu_module;

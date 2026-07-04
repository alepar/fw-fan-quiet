//! Actuators: command runner seam + hardware knobs. Consumed by the
//! controller thread (Task 14) and RestoreGuard (Task 13).

pub mod cmd;
pub mod smu_module;

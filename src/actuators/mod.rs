//! Actuators: command runner seam + hardware knobs. Owned by the controller
//! thread via RestoreGuard; FinalRestore rebuilds fresh ones on the panic path.

pub mod cmd;
pub mod cpu;
pub mod gpu;
pub mod guard;
pub mod smu_module;

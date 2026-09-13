//! Actuators: command runner seam + hardware knobs. Owned by the controller
//! thread via RestoreGuard; FinalRestore rebuilds fresh ones on the panic path.

pub mod cmd;
pub mod cpu;
pub mod gpu;
pub mod guard;
pub mod smu_module;

/// Read-back verification outcome for a commanded hardware limit (design doc
/// §2.9): a write is not trusted until the hardware itself confirms it
/// stuck, because RyzenAdj's own documentation names the platform silently
/// reasserting vendor defaults (AC unplug, power-profile change, periodic
/// resets) as a real and recurring failure mode.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum WriteVerdict {
    /// Read-back confirmed the commanded value (within the actuator's
    /// tolerance). Carries the verified value.
    Verified(f64),
    /// Read-back disagreed with what was commanded, naming the offending
    /// field.
    Mismatch {
        field: &'static str,
        commanded: f64,
        read: f64,
    },
    /// The read-back itself could not be performed (e.g. `ryzenadj --info`
    /// fails while `ryzen_smu` is loaded) — a precondition failure, not a
    /// hardware fault, so this must never be scored as `Mismatch`.
    Unreadable,
    /// No verification was possible right now (e.g. GPU utilisation below
    /// the 90% floor the SM-clock check needs) — not a failure.
    Unverifiable,
}

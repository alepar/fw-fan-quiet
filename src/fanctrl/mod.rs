//! fw-fanctrl socket client and curve model (design doc §2.1, §2.3).
//!
//! `curve`/`table` (Task 1) are the pure domain model: the piecewise-linear
//! curve fw-fanctrl itself runs, and the duty<->RPM table the controller
//! snaps targets through. `client` (Task 7) is the read-only socket client
//! (`UnixFanctrlClient`, `FanctrlView`, `PrintCommand`) that feeds `curve`'s
//! `Curve::from_points` from the live strategy.

pub mod client;
pub mod curve;
pub mod table;

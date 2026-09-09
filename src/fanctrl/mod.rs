//! fw-fanctrl socket client and curve model (design doc §2.1, §2.3).
//!
//! Task 1 lands only the pure domain model (`curve`, `table`): the piecewise-
//! linear curve fw-fanctrl itself runs, and the duty<->RPM table the
//! controller snaps targets through. The socket client (`FanctrlClient`,
//! `FanctrlView`) is Task 7 and lands in this module later.

pub mod curve;
pub mod table;

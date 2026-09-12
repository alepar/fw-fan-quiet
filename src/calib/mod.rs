//! Calibration building blocks (Milestone 3). The CPU burner lives here
//! because the selftest (Task 16) and the calibration sweep (Task 19) share
//! it as their synthetic load source.

pub mod burner;
pub mod fopdt;
#[allow(dead_code)] // staged compatibility; task .12 deletes the LUT sweep
pub mod lut_sweep;
pub mod runner;
#[allow(dead_code)] // used by the legacy step compatibility path until .12
pub mod steady;
pub mod step;

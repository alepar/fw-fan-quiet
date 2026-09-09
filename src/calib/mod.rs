//! Calibration building blocks (Milestone 3). The CPU burner lives here
//! because the selftest (Task 16) and the calibration sweep (Task 19) share
//! it as their synthetic load source.

pub mod burner;
pub mod fopdt;
pub mod lut_sweep;
pub mod runner;
pub mod steady;

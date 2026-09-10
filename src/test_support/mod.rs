//! Test-only support shared across the fanctrl-loop test suite. The whole
//! tree is gated by `#[cfg(test)] mod test_support;` in `src/main.rs`, so
//! nothing under here is compiled into the release binary.
//!
//! - `fixtures`: resolves paths into the checked-in `tests/fixtures/`
//!   corpus (Task 2).
//! - `fakes`: a scripted `FanctrlSource` fake for controller/unit tests
//!   (Task 7 / fwloop.2).
//! - `plant`: the fw-fanctrl emulator + chained physical plant used by the
//!   closed-loop simulations (Task 17 / fwloop.16).
pub mod fakes;
pub mod fixtures;
pub mod plant;

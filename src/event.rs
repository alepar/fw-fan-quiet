//! Cross-thread event enum: everything the UI and controller threads receive
//! arrives as one of these over crossbeam channels.

// Consumed once the UI (Task 8) and controller (Task 14) threads land.
#![allow(dead_code)]

use crate::types::Sample;

/// Controller status placeholder.
/// TODO(task-14): replace with the real controller status (mode, caps,
/// watchdog flags, ...) when the controller thread lands.
#[derive(Clone, Debug, Default)]
pub struct ControlStatus {}

/// Events multiplexed onto the per-thread channels.
#[derive(Clone, Debug)]
pub enum Event {
    /// 1 Hz sensor snapshot from the sampler thread.
    Sample(Sample),
    /// Keyboard input from the input thread.
    Input(crossterm::event::KeyEvent),
    /// Controller status update.
    Status(ControlStatus),
    /// Periodic redraw tick.
    Tick,
}

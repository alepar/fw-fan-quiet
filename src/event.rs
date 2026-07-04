//! Cross-thread event enum: everything the UI and controller threads receive
//! arrives as one of these over crossbeam channels.

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
    // TODO(task-14): constructed by the controller thread.
    #[allow(dead_code)]
    Status(ControlStatus),
    /// Periodic redraw tick. Nothing constructs it yet: the main loop's
    /// recv timeout currently plays this role.
    #[allow(dead_code)]
    Tick,
}

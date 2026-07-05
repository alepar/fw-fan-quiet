//! Cross-thread event enum: everything the UI and controller threads receive
//! arrives as one of these over crossbeam channels.

use crate::control::ControlStatus;
use crate::types::Sample;

/// Events multiplexed onto the per-thread channels.
#[derive(Clone, Debug)]
pub enum Event {
    /// 1 Hz sensor snapshot from the sampler thread.
    Sample(Sample),
    /// Keyboard input from the input thread.
    Input(crossterm::event::KeyEvent),
    /// Controller status update (sent only when the status actually changed).
    Status(ControlStatus),
}
